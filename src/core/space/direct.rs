//! A span of a space resolved once to the memory behind it, for a master
//! that reads the same page again and again.
//!
//! [`AddressSpace::read`] takes the topology read guard, walks the dispatch
//! table to a [`FlatEntry`](super::FlatEntry), checks permissions and
//! constraints, and only then reads a byte out of a store. For an interpreter
//! fetching an instruction stream that is the same answer several times per
//! guest instruction: the bytes of one instruction nearly always sit in one
//! page, and so do the bytes of the next. A [`DirectSpan`] is that answer kept
//! — *which store, at what offset* — together with the topology generation it
//! was taken under, which is the one thing that can make it wrong
//! (`ROADMAP.md` §15, invariant 3).
//!
//! # Exactness
//!
//! A read through a span must be **bit-identical** to the read
//! [`AddressSpace::read`] would have made, errors included, or the cache is a
//! source of divergence rather than speed. So a span exists only where the
//! slow path cannot fail and has nothing left to decide, and each refusal in
//! [`SpaceView::direct_span`] is one of those decisions:
//!
//! * One flat entry covers the **whole** span, and it is a single leaf — a
//!   combined (wired-or) range has several responders and no one store.
//! * The leaf is RAM or ROM. A device is a call, every time.
//! * The leaf is not **rebasable**. A bank switch slides its offset and
//!   deliberately bumps no generation (see the module docs of
//!   [`space`](super), "Two kinds of change"), so an offset taken from one
//!   would go stale with nothing to notice it.
//! * The leaf does not **repeat**: a mirrored window wraps its offset partway
//!   through, and one offset cannot express that.
//! * The mapping grants the permission asked for.
//! * The constraints are permissive — every width, any alignment, no
//!   secure-only or privileged-only rule — so there is no check left that
//!   depends on the access.
//! * The store really holds every byte of the span.
//!
//! These are the conditions `jit::Tlb::fill` already caches a page under,
//! less the one it needs and a reader does not: a *directed* entry, whose
//! writes go somewhere other than its reads, is still one store to read from.
//!
//! # Read-only on purpose
//!
//! A store is more than its bytes: it marks the dirty bitmap, it ends other
//! masters' exclusive reservations ([`Monitor`](super::Monitor)), and a
//! translation cache has to hear about it. Nothing here writes, so none of
//! that can be skipped by accident.
//!
//! # No `unsafe`
//!
//! The bytes are reached by offset through the store's own bounds-checked
//! accessors, never through a host pointer — which is what keeps this working
//! when guest RAM lives in a `SharedArrayBuffer` (`ROADMAP.md` §11.2).
//!
//! [`AddressSpace::read`]: super::AddressSpace::read

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{
    AddressSpace, EntryKind, FlatTarget, MemAttrs, MemResult, Perms, RamStore, RomStore, SpaceView,
};
use crate::core::value::{Endian, Width};

/// The memory behind a [`DirectSpan`].
#[derive(Debug, Clone)]
enum Backing {
    /// Writable memory, read with relaxed atomic loads — the same loads
    /// [`RamStore::read_value`] makes, so a store by another master is seen
    /// exactly as early as the slow path would see it.
    Ram(Arc<RamStore>),
    /// Read-only memory.
    Rom(Arc<RomStore>),
}

/// A contiguous span of a space, resolved to the store that backs it.
///
/// Valid for exactly as long as [`AddressSpace::generation`] still answers
/// [`DirectSpan::generation`]; the holder compares the two before every use
/// and throws the span away when they differ. It holds its store alive, so a
/// span that outlives its mapping is merely stale, never dangling.
///
/// [`AddressSpace::generation`]: super::AddressSpace::generation
#[derive(Debug, Clone)]
pub struct DirectSpan {
    backing: Backing,
    /// Offset in the store of the span's first byte.
    offset: u64,
    /// How many bytes the span covers.
    len: u64,
    endian: Endian,
    generation: u64,
}

impl DirectSpan {
    /// The topology generation this resolution was taken under.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// How many bytes the span covers.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the span covers nothing. Never true of one
    /// [`SpaceView::direct_span`] returned.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The byte order of the region behind the span.
    #[inline]
    #[must_use]
    pub fn endian(&self) -> Endian {
        self.endian
    }

    /// The byte `at` bytes into the span, or `None` past its end.
    #[inline]
    #[must_use]
    pub fn read_u8(&self, at: u64) -> Option<u8> {
        if at >= self.len {
            return None;
        }
        // `offset + at` cannot wrap: `direct_span` proved `offset + len` is
        // inside the store.
        let off = self.offset + at;
        match &self.backing {
            Backing::Ram(s) => s.read_u8(off).ok(),
            Backing::Rom(s) => s
                .read_value(off, Width::U8, Endian::Little)
                .ok()
                .map(|v| v as u8),
        }
    }

    /// The `width`-byte value `at` bytes into the span, in the region's byte
    /// order, or `None` if it does not lie wholly inside the span.
    #[inline]
    #[must_use]
    pub fn read_value(&self, at: u64, width: Width) -> Option<u64> {
        let end = at.checked_add(width.bytes())?;
        if end > self.len {
            return None;
        }
        let off = self.offset + at;
        match &self.backing {
            Backing::Ram(s) => s.read_value(off, width, self.endian).ok(),
            Backing::Rom(s) => s.read_value(off, width, self.endian).ok(),
        }
    }
}

/// One remembered page of a [`ReadCache`].
#[derive(Debug)]
struct Slot {
    /// The page's address, or [`Slot::EMPTY`].
    page: u64,
    /// The topology generation the answer was taken under.
    generation: u64,
    /// The permission the answer was taken for.
    need: Perms,
    /// The store behind the page, or `None` for a page that has to keep
    /// taking [`AddressSpace::read`](super::AddressSpace::read) — which is
    /// worth remembering too, or every access to a device page would probe
    /// the flat view twice.
    span: Option<DirectSpan>,
}

impl Slot {
    /// No page address equals this: its low twelve bits are set.
    const EMPTY: u64 = u64::MAX;
}

/// A master's own cache of [`DirectSpan`]s, one per page it has read lately.
///
/// The software TLB `ROADMAP.md` §4.1 puts *above* the address space, in its
/// plainest form: direct-mapped on the page number, read-only, and exact.
/// [`ReadCache::read`] is [`AddressSpace::read`] — the same value and the same
/// errors, because a page is served from here only under the conditions in the
/// [module docs](self) and everything else is handed to the space unchanged.
/// What a hit skips is the topology read guard, the dispatch table, the flat
/// entry and the checks, which is most of what a load from RAM costs.
///
/// Per master and not shared: the holder passes the same space every time.
/// A cache that has been used with one space and is handed another would
/// answer from the first for as long as the two generations happened to
/// agree, so a core that is attached to a new space starts a new cache.
///
/// Derived state (`CLAUDE.md`, *Devices*): never serialized, and
/// [`Clone`] yields an **empty** cache rather than a copy — the one reason a
/// core's state is cloned is to snapshot it, and a snapshot carries no
/// derived state.
///
/// [`AddressSpace::read`]: super::AddressSpace::read
#[derive(Debug, Default)]
pub struct ReadCache {
    /// Empty until the first read, so constructing a core allocates nothing.
    slots: Vec<Slot>,
}

impl Clone for ReadCache {
    fn clone(&self) -> ReadCache {
        ReadCache::new()
    }
}

impl ReadCache {
    /// The granule: 4 KiB, the smallest page any guest in the tree maps.
    pub const PAGE: u64 = 0x1000;

    /// How many pages are remembered. A power of two, because the index is a
    /// mask.
    pub const SLOTS: usize = 64;

    /// An empty cache.
    #[must_use]
    pub const fn new() -> ReadCache {
        ReadCache { slots: Vec::new() }
    }

    /// Forget everything. Never needed for correctness within one space — a
    /// stale slot fails its own generation test — and required when the
    /// holder is moved to another.
    pub fn clear(&mut self) {
        self.slots.clear();
    }

    /// [`AddressSpace::read`](super::AddressSpace::read), served from a
    /// remembered store where the page allows it.
    ///
    /// # Errors
    ///
    /// Exactly those of the read it replaces.
    ///
    /// # `inline(always)`
    ///
    /// `CLAUDE.md` allows `#[inline]` on a hot-path accessor and nothing
    /// stronger, so the stronger form owes a reason, and it is the one
    /// [`RamStore::read_value`] gives: with a plain `#[inline]` LLVM inlines
    /// this into its caller while there is one caller and into none of them
    /// once there are two. Every core that adopts the cache is another call
    /// site, so the first adopter's fast path got slower each time another
    /// core gained one — measured on a riscv64 Linux boot, the interpreter
    /// went from 760 to 800 host instructions per guest instruction when
    /// `cpu::arm::a64` became the second, and is at 773 with this.
    #[inline(always)]
    pub fn read(
        &mut self,
        space: &AddressSpace,
        addr: u64,
        width: Width,
        attrs: MemAttrs,
    ) -> MemResult<u64> {
        if let Some(span) = self.span_at(space, addr, attrs.read_perm())
            && let Some(value) = span.read_value(addr & (Self::PAGE - 1), width)
        {
            return Ok(value);
        }
        // A page that is not plain memory, or an access that runs off the end
        // of the one that is.
        space.read(addr, width, attrs)
    }

    /// The store behind the page holding `addr`, for reads checked against
    /// `need`, if that page is one a [`DirectSpan`] can describe. The span
    /// starts at the page, not at `addr`.
    ///
    /// For a holder that reads many bytes of one page in a row and would
    /// rather index the span itself than come back through
    /// [`read`](ReadCache::read) for each.
    ///
    /// `inline(always)` with [`read`](ReadCache::read), and for its reason:
    /// this is the whole of that function's fast path.
    #[inline(always)]
    pub fn span_at(&mut self, space: &AddressSpace, addr: u64, need: Perms) -> Option<&DirectSpan> {
        let page = addr & !(Self::PAGE - 1);
        let index = (page >> 12) as usize & (Self::SLOTS - 1);
        let generation = space.generation();
        let fresh = self
            .slots
            .get(index)
            .is_some_and(|s| s.page == page && s.generation == generation && s.need == need);
        if !fresh {
            self.fill(space, index, page, generation, need);
        }
        self.slots.get(index)?.span.as_ref()
    }

    /// Out of line on purpose: this is the miss, and its probe of the flat
    /// view has no business in the hit's code.
    #[inline(never)]
    fn fill(
        &mut self,
        space: &AddressSpace,
        index: usize,
        page: u64,
        generation: u64,
        need: Perms,
    ) {
        if self.slots.is_empty() {
            self.slots.resize_with(Self::SLOTS, || Slot {
                page: Slot::EMPTY,
                generation: 0,
                need: Perms::NONE,
                span: None,
            });
        }
        let span = space.direct_span(page, Self::PAGE, need);
        let Some(slot) = self.slots.get_mut(index) else {
            return;
        };
        // The span's own generation where there is one: it was read under the
        // guard that walked the map. Otherwise the one read before the probe,
        // which is the older of the two a racing retopology could leave and so
        // the one that makes the next use probe again.
        slot.generation = span.as_ref().map_or(generation, DirectSpan::generation);
        slot.page = page;
        slot.need = need;
        slot.span = span;
    }
}

impl SpaceView<'_> {
    /// Resolve the `len` bytes at `addr` to the store behind them, if a read
    /// of any width anywhere in that span is one this view could not refuse.
    ///
    /// `need` is the permission the reads will be checked for —
    /// [`MemAttrs::read_perm`](super::MemAttrs::read_perm) of the attributes
    /// they would have carried, so a master that marks its fetches asks for
    /// [`Perms::EXEC`] and one that does not asks for [`Perms::READ`]. A
    /// request that includes [`Perms::WRITE`] is refused: the span cannot
    /// write.
    ///
    /// `None` is "take the slow path", never an error; see the
    /// [module docs](self) for each condition.
    #[must_use]
    pub fn direct_span(&self, addr: u64, len: u64, need: Perms) -> Option<DirectSpan> {
        if len == 0 || need.contains(Perms::WRITE) {
            return None;
        }
        let end = addr.checked_add(len)?;
        let entry = self.flat_view().entry(self.locate(addr)?)?;
        if entry.start() > addr || entry.end() < end {
            return None;
        }
        let EntryKind::Single(leaf) = entry.kind() else {
            return None;
        };
        if leaf.is_rebasable() || leaf.period().is_some() {
            return None;
        }
        let c = leaf.constraints();
        let permissive = c.min == Width::U8
            && c.max == Width::U64
            && !c.natural_alignment
            && !c.secure_only
            && !c.privileged_only;
        if !leaf.perms().contains(need) || !permissive {
            return None;
        }
        let offset = leaf.offset_of(addr - entry.start());
        let (backing, store_len) = match leaf.target() {
            FlatTarget::Ram(s) => (Backing::Ram(Arc::clone(s)), s.len()),
            FlatTarget::Rom { store, .. } => (Backing::Rom(Arc::clone(store)), store.len()),
            _ => return None,
        };
        if offset.checked_add(len)? > store_len {
            return None;
        }
        Some(DirectSpan {
            backing,
            offset,
            len,
            endian: c.endian,
            // Read under the view's guard, which excludes the retopology that
            // would bump it: this is the generation of what was just walked.
            generation: self.space().generation(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{
        AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RomWrite, UnassignedPolicy,
    };
    use alloc::vec::Vec;

    const PAGE: u64 = 0x1000;

    #[derive(Debug)]
    struct Dev;

    impl MemOps for Dev {
        fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
            dst.fill(0x5a);
            Ok(())
        }

        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            Ok(())
        }
    }

    fn space() -> AddressSpace {
        AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT)
    }

    fn patterned_ram(len: u64) -> Arc<RamStore> {
        let ram = Arc::new(RamStore::new(len));
        for i in 0..len {
            ram.write_u8(i, (i.wrapping_mul(7) ^ (i >> 8)) as u8)
                .unwrap();
        }
        ram
    }

    #[test]
    fn a_span_reads_exactly_what_the_space_reads() {
        let space = space();
        let ram = patterned_ram(4 * PAGE);
        // Mapped at an offset that is not the store's zero, so a span that
        // forgot the base reads the wrong page rather than the right one.
        space
            .topology()
            .map(Region::ram("ram", ram), 0x8000)
            .unwrap();
        let span = space
            .direct_span(0x8000 + PAGE, PAGE, Perms::READ)
            .expect("plain RAM is direct");
        assert_eq!(span.len(), PAGE);
        assert_eq!(span.generation(), space.generation());
        for at in 0..PAGE {
            let addr = 0x8000 + PAGE + at;
            assert_eq!(
                span.read_u8(at).map(u64::from),
                space.read(addr, Width::U8, MemAttrs::DEFAULT).ok(),
                "byte {at:#x}"
            );
        }
        for width in [Width::U16, Width::U32, Width::U64] {
            for at in [0, 1, 7, PAGE - width.bytes()] {
                assert_eq!(
                    span.read_value(at, width),
                    space
                        .read(0x8000 + PAGE + at, width, MemAttrs::DEFAULT)
                        .ok(),
                    "{width:?} at {at:#x}"
                );
            }
        }
        // Past the end is the caller's slow path, not a read of the next page.
        assert_eq!(span.read_u8(PAGE), None);
        assert_eq!(span.read_value(PAGE - 1, Width::U16), None);
        assert_eq!(span.read_value(u64::MAX, Width::U16), None);
    }

    #[test]
    fn a_span_sees_a_store_made_after_it_was_taken() {
        // The bytes are read live, never copied: self-modifying code needs no
        // invalidation because there is nothing to invalidate.
        let space = space();
        space
            .topology()
            .map(Region::ram("ram", Arc::new(RamStore::new(PAGE))), 0)
            .unwrap();
        let span = space.direct_span(0, PAGE, Perms::READ).unwrap();
        assert_eq!(span.read_u8(0x10), Some(0));
        space
            .write(0x10, Width::U8, 0xc3, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(span.read_u8(0x10), Some(0xc3));
    }

    #[test]
    fn rom_is_direct_and_keeps_its_byte_order() {
        let space = space();
        let image: Vec<u8> = (0..=255u8).collect();
        let rom = Arc::new(RomStore::new(image));
        space
            .topology()
            .map(Region::rom("rom", rom, RomWrite::Ignore), 0xf000)
            .unwrap();
        let span = space.direct_span(0xf000, 0x100, Perms::READ).unwrap();
        assert_eq!(span.read_u8(0x42), Some(0x42));
        assert_eq!(
            span.read_value(0x10, Width::U32),
            space.read(0xf010, Width::U32, MemAttrs::DEFAULT).ok()
        );
        assert_eq!(span.endian(), space.endian_at(0xf000));
    }

    #[test]
    fn a_retopology_moves_the_generation_a_span_carries() {
        let space = space();
        space
            .topology()
            .map(Region::ram("a", Arc::new(RamStore::new(PAGE))), 0)
            .unwrap();
        let span = space.direct_span(0, PAGE, Perms::READ).unwrap();
        space
            .topology()
            .map(Region::ram("b", Arc::new(RamStore::new(PAGE))), PAGE)
            .unwrap();
        assert_ne!(
            span.generation(),
            space.generation(),
            "the holder's one validity test must fail after a remap"
        );
    }

    #[test]
    fn everything_the_slow_path_still_has_to_decide_is_refused() {
        let space = space();
        let ram = patterned_ram(2 * PAGE);
        let region: RegionRef = Region::ram("ram", Arc::clone(&ram)).into();
        let window = Region::alias("bank", region.clone(), 0, PAGE).unwrap();
        let mirror = Region::mirror("mirror", region.clone(), 4 * PAGE).unwrap();
        {
            let mut t = space.topology();
            t.map(region, 0).unwrap();
            t.map(window, 0x1_0000).unwrap();
            t.map(mirror, 0x2_0000).unwrap();
            t.map(Region::io("dev", PAGE, Arc::new(Dev)), 0x3_0000)
                .unwrap();
            t.map_with_perms(
                Region::ram("wo", Arc::new(RamStore::new(PAGE))),
                0x4_0000,
                Perms::WRITE,
            )
            .unwrap();
            t.map_with_perms(
                Region::ram("rw", Arc::new(RamStore::new(PAGE))),
                0x5_0000,
                Perms::RW,
            )
            .unwrap();
        }
        // The control: the same store, mapped plainly.
        assert!(space.direct_span(0, PAGE, Perms::READ).is_some());
        // A span that runs off the end of its entry.
        assert!(space.direct_span(PAGE, 2 * PAGE, Perms::READ).is_none());
        // Nothing mapped.
        assert!(space.direct_span(0x8_0000, PAGE, Perms::READ).is_none());
        // A bank-switched window slides without moving the generation.
        assert!(space.direct_span(0x1_0000, PAGE, Perms::READ).is_none());
        // A mirror wraps its offset partway through.
        assert!(space.direct_span(0x2_0000, PAGE, Perms::READ).is_none());
        // A device is a call, every time.
        assert!(space.direct_span(0x3_0000, PAGE, Perms::READ).is_none());
        // A mapping that does not grant the direction.
        assert!(space.direct_span(0x4_0000, PAGE, Perms::READ).is_none());
        // A fetch is checked for EXEC and nothing else.
        assert!(space.direct_span(0x5_0000, PAGE, Perms::READ).is_some());
        assert!(space.direct_span(0x5_0000, PAGE, Perms::EXEC).is_none());
        // And the span cannot write, so it will not pretend to have checked.
        assert!(space.direct_span(0, PAGE, Perms::RW).is_none());
        assert!(space.direct_span(0, 0, Perms::READ).is_none());
        assert!(space.direct_span(u64::MAX, 2, Perms::READ).is_none());
    }

    /// A space with one of everything a read can land on: RAM, a ROM, a
    /// device, a read-protected page, a banked window and a hole. The window
    /// is handed back so a test can slide it.
    fn mixed() -> (AddressSpace, RegionRef) {
        let space = space();
        let ram: RegionRef = Region::ram("ram", patterned_ram(4 * PAGE)).into();
        let window: RegionRef = Region::alias("bank", ram.clone(), PAGE, PAGE)
            .unwrap()
            .into();
        let image: Vec<u8> = (0..2 * PAGE).map(|i| (i * 13) as u8).collect();
        {
            let mut t = space.topology();
            t.map(ram, 0).unwrap();
            t.map(
                Region::rom("rom", Arc::new(RomStore::new(image)), RomWrite::Ignore),
                0x1_0000,
            )
            .unwrap();
            t.map(Region::io("dev", PAGE, Arc::new(Dev)), 0x2_0000)
                .unwrap();
            t.map_with_perms(
                Region::ram("wo", Arc::new(RamStore::new(PAGE))),
                0x3_0000,
                Perms::WRITE,
            )
            .unwrap();
            t.map(window.clone(), 0x4_0000).unwrap();
        }
        (space, window)
    }

    /// Every address the cache is asked about, at every width, against the
    /// space itself — values and errors both.
    fn agree(cache: &mut ReadCache, space: &AddressSpace) {
        let bases = [0, PAGE, 0x1_0000, 0x2_0000, 0x3_0000, 0x4_0000, 0x8_0000];
        // A fetch is checked for a different permission and must not be
        // answered from a slot a load filled.
        let fetch = MemAttrs::DEFAULT.with_purpose(crate::core::space::AccessPurpose::FETCH);
        for attrs in [MemAttrs::DEFAULT, fetch] {
            for base in bases {
                // The page's first bytes, its last ones, and the widths that
                // run off its end into whatever is next.
                for at in [0, 1, 0x7fd, PAGE - 8, PAGE - 4, PAGE - 2, PAGE - 1] {
                    for width in [Width::U8, Width::U16, Width::U32, Width::U64] {
                        let addr = base + at;
                        assert_eq!(
                            cache.read(space, addr, width, attrs),
                            space.read(addr, width, attrs),
                            "{width:?} at {addr:#x}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn the_cache_reads_what_the_space_reads_errors_included() {
        let (space, _) = mixed();
        let mut cache = ReadCache::new();
        // Twice: once filling, once answering from what was filled.
        agree(&mut cache, &space);
        agree(&mut cache, &space);
    }

    #[test]
    fn the_cache_follows_a_store_a_remap_and_a_bank_switch() {
        let (space, window) = mixed();
        let mut cache = ReadCache::new();
        agree(&mut cache, &space);
        // A store is seen because nothing was copied.
        space
            .write(0x10, Width::U32, 0xdead_beef, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(
            cache.read(&space, 0x10, Width::U32, MemAttrs::DEFAULT),
            Ok(0xdead_beef)
        );
        // A bank switch moves no generation at all, which is why a banked
        // page was never cached in the first place.
        let generation = space.generation();
        let before = cache.read(&space, 0x4_0000, Width::U32, MemAttrs::DEFAULT);
        space.rebase(&window, 2 * PAGE).unwrap();
        assert_eq!(space.generation(), generation);
        let after = cache.read(&space, 0x4_0000, Width::U32, MemAttrs::DEFAULT);
        assert_ne!(
            before, after,
            "the window slid and the cache did not see it"
        );
        agree(&mut cache, &space);
        // A remap moves the generation every slot is checked against.
        space
            .topology()
            .map_with_priority(Region::ram("over", patterned_ram(PAGE)), 0x1_0000, 1)
            .unwrap();
        assert_ne!(space.generation(), generation);
        agree(&mut cache, &space);
    }

    #[test]
    fn a_cloned_or_cleared_cache_is_empty_and_still_right() {
        let (space, _) = mixed();
        let mut cache = ReadCache::new();
        agree(&mut cache, &space);
        let mut copy = cache.clone();
        assert!(copy.slots.is_empty(), "derived state is not copied");
        agree(&mut copy, &space);
        cache.clear();
        assert!(cache.slots.is_empty());
        agree(&mut cache, &space);
    }
}
