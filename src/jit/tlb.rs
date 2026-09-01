//! The software TLB: a guest address, resolved to a host addend, in a mask, a
//! compare and an add.
//!
//! `ROADMAP.md` §9.1's first mechanism, and the one it says everything else is
//! secondary to. Per-CPU, direct-mapped, [`DEFAULT_ENTRIES`] per set, split by
//! access type, and an entry is exactly the pair the roadmap names: a guest
//! page tag, and either a host addend or a marker saying this page has to go
//! the slow way.
//!
//! # What is cached, and what is not
//!
//! This caches the *host* half of an access — the part after the guest's own
//! MMU has answered. The caller translates (its own TLB, in
//! `cpu::riscv::mmu`, caches that half and charges the walk), and hands both
//! addresses in: the **virtual** one, which the tag is taken from, and the
//! **physical** one, which the resolution is taken from. In bare mode they are
//! the same number and nothing is lost.
//!
//! The resolution is only ever cached for **plain writable RAM covering the
//! whole guest page**, and the conditions in [`Tlb::fill`] are each there
//! because getting one wrong is a silent wrong answer rather than a slow one.
//! The most interesting is `is_rebasable`: an MMC3 cartridge slides an alias's
//! offset ~15 000 times a second and deliberately bumps **no** generation
//! counter (`core::space`, "Two kinds of change"), so an addend derived from
//! one would go stale with nothing to notice it. Such a page is marked
//! uncacheable and takes the slow path forever — which is the roadmap's "IO
//! slot" doing the job it exists for.
//!
//! # Why a hit is allowed to skip the space's read guard
//!
//! [`AddressSpace::read`] takes the topology read guard, walks the dispatch
//! table to a [`FlatEntry`](crate::core::space::FlatEntry), checks permissions
//! and constraints, and then copies bytes out of a
//! [`RamStore`](crate::core::space::RamStore). A hit here does the last step
//! only. Every earlier step was performed once, at fill time, and is valid
//! until the topology generation changes — which is what [`Epoch`] carries and
//! [`Tlb::sync`] acts on. Under threads that is sound as far as the safe-point
//! protocol reaches; the module docs say where the line is.
//!
//! # Exactness
//!
//! A hit must produce **bit-identical** results to the slow path, including
//! its errors, or the cache is a source of divergence rather than speed. The
//! fill conditions exist to make that provable rather than likely:
//! permissions are checked at fill time against the same
//! [`Perms`](crate::core::space::Perms) bits [`FlatLeaf::read`] checks;
//! constraints must be permissive, so there is no width or alignment rule left
//! to apply; the region's byte order travels in the entry and the same
//! [`Endian::load`] assembles the value; and an access that would cross the
//! page the entry describes takes the slow path.
//!
//! # What a caller may not reach through here
//!
//! There is no `read_driven`, and that is a refusal rather than an omission:
//! [`AccessConstraints::drives_data_bus`] says whether a read came from the far
//! side of the master's pins, a master that models an **open-bus latch** reads
//! it on every access, and the answer is a property of the region rather than
//! of the bytes. Serving such a master from an entry would silently make every
//! access look driven. The 6502 is that master; it also has no MMU, so it
//! wants a TLB indexed on physical addresses more than it wants this one. A
//! core that needs the bit calls the address space.
//!
//! [`AddressSpace::read`]: crate::core::space::AddressSpace::read
//! [`AccessConstraints::drives_data_bus`]: crate::core::space::AccessConstraints::drives_data_bus
//! [`FlatLeaf::read`]: crate::core::space::FlatLeaf::read

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::space::{
    AccessConstraints, AddressSpace, FlatTarget, MemAttrs, MemResult, Perms, RamStore,
};
use crate::core::value::{Endian, Width};
use crate::ir::AccessKind;

/// The translation granule this TLB indexes by, in bytes.
///
/// 4 KiB: the smallest page every scheme any of our guests implements has
/// (Sv32 and Sv39 both, x86's non-PSE tables, ARM's small pages). A guest with
/// a larger page simply fills several entries, which costs entries and never
/// correctness; a guest with a *smaller* one would make an entry describe
/// memory it does not own, so the number is checked rather than assumed by
/// every frontend that fills this.
pub const PAGE_SIZE: u64 = 4096;

/// The mask selecting the offset within a [`PAGE_SIZE`] page.
pub const PAGE_MASK: u64 = PAGE_SIZE - 1;

/// Entries per access-type set, and therefore three times this in total.
///
/// `ROADMAP.md` §9.1's number. A power of two, because the index is a mask.
pub const DEFAULT_ENTRIES: u64 = 4096;

/// The two counters a translation cache is stale against.
///
/// Both are read from things that already exist rather than maintained here:
/// [`topology`](Epoch::topology) is
/// [`AddressSpace::generation`](crate::core::space::AddressSpace::generation),
/// and [`translation`](Epoch::translation) is the guest MMU's own counter —
/// `Csrs::translation_gen` on RISC-V, the one `cpu::riscv::lift`'s
/// `Origin::Paged` already folds into a block's key. The module docs have the
/// table of what each invalidates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, PartialOrd, Ord, Hash)]
pub struct Epoch {
    /// The address space's topology generation.
    pub topology: u64,
    /// The guest MMU's translation generation.
    pub translation: u64,
}

/// The part of a CPU's world that changes too often to flush on.
///
/// Everything else — the ASID, `SUM`, `MXR`, which root page table is in
/// force — bumps the guest's translation generation, so it lands in [`Epoch`]
/// and costs a flush. Privilege does not: a supervisor guest changes it on
/// every trap and every return, and flushing 12 288 entries per system call
/// would cost more than the TLB saves. So it is tagged instead, exactly, in
/// bits the page number does not use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, PartialOrd, Ord, Hash)]
pub struct Context {
    /// The guest's privilege level, as the guest numbers it.
    ///
    /// Only the low three bits are kept, which covers every core in the tree:
    /// RISC-V has three levels, ARM four exception levels, x86 four rings. A
    /// guest with more would alias, so the number is stated rather than
    /// assumed.
    pub level: u8,
    /// Whether the guest address is virtual.
    ///
    /// Separates a bare-mode access at some number from a translated access at
    /// the same number, which is the [`Origin`] distinction one level down.
    ///
    /// [`Origin`]: crate::cpu::riscv::lift::Origin
    pub translating: bool,
}

impl Context {
    /// The tag bits this context contributes, below the page number.
    #[inline]
    const fn bits(self) -> u64 {
        ((self.level as u64 & 7) << 2) | ((self.translating as u64) << 1)
    }
}

/// One cached resolution.
///
/// `ROADMAP.md` §9.1's `{ guest page tag, host addend | IO slot }`, with the
/// store index standing in for the addend's other half: guest RAM is addressed
/// by byte offset into a store and never handed out as a slice
/// (`ROADMAP.md` §11.2), so "host addend" is `(which store, what offset)`.
#[derive(Debug, Clone, Copy)]
struct Entry {
    /// `page | context | VALID`, or [`Entry::EMPTY`].
    ///
    /// The page's low twelve bits are zero by construction, so the context
    /// rides in them and one `u64` compare decides the whole tag. No hashing,
    /// so two different worlds cannot alias into one hit.
    tag: u64,
    /// `guest address + addend` is the offset in [`Entry::store`].
    addend: u64,
    /// Index into [`Tlb::stores`], or [`Entry::SLOW`].
    store: u32,
    /// The region's byte order, so a hit assembles the value the way the slow
    /// path would.
    endian: Endian,
}

impl Entry {
    /// No tag can equal this: a valid tag's low bit is set and its page number
    /// is bounded by the guest's address width.
    const EMPTY: u64 = 0;
    /// The tag's valid bit, in a bit the page number does not use.
    const VALID: u64 = 1;
    /// This page is known not to be cacheable. The roadmap's "IO slot": a
    /// resolution that says *go the slow way* is still worth remembering,
    /// because otherwise every MMIO access re-probes the flat view twice.
    const SLOW: u32 = u32::MAX;

    const fn empty() -> Entry {
        Entry {
            tag: Entry::EMPTY,
            addend: 0,
            store: Entry::SLOW,
            endian: Endian::Little,
        }
    }
}

/// What a [`Tlb`] has been asked to do.
///
/// Derived state in the strict sense of `ROADMAP.md` §4.5 — never serialized,
/// safe to throw away — so these are for `rsemu`'s statistics and for tests
/// that assert a page was cached rather than that it happened to be fast.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TlbStats {
    /// Accesses served entirely from an entry.
    pub hits: u64,
    /// Accesses whose page had no entry.
    pub misses: u64,
    /// Accesses whose page had an entry saying it is not cacheable.
    pub slow: u64,
    /// Accesses that spanned two pages and were not looked up at all.
    pub split: u64,
    /// Entries written.
    pub fills: u64,
    /// Fills refused because the page is not plain RAM covering the page.
    pub refused: u64,
    /// Whole-TLB invalidations, from an [`Epoch`] change or a reset.
    pub flushes: u64,
}

/// A per-CPU software TLB in front of one address space.
///
/// Not `Clone` and not shared: `ROADMAP.md` §9.1 says per-CPU, and a shared
/// one would need a lock on the path whose entire purpose is not having one.
#[derive(Debug)]
pub struct Tlb {
    space: Arc<AddressSpace>,
    /// One set per [`AccessKind`], indexed by `AccessKind` in declaration
    /// order. Split so a store stream cannot evict the loads it is
    /// interleaved with, and so the store set is the only one a
    /// self-modifying-code check has to consult.
    sets: [Box<[Entry]>; 3],
    /// `entries - 1`; the index is `(addr >> 12) & mask`.
    mask: u64,
    /// The stores entries point into, so an entry costs a `u32` rather than an
    /// `Arc` clone and a drop on every fill.
    stores: Vec<Arc<RamStore>>,
    epoch: Epoch,
    stats: TlbStats,
}

/// How many distinct RAM stores one TLB will hold addends into.
///
/// A machine has a handful — main RAM, a framebuffer, an SRAM. The cap bounds
/// the linear scan a fill does, and a machine that exceeds it loses speed
/// rather than correctness: further pages are simply marked uncacheable.
const MAX_STORES: usize = 64;

impl Tlb {
    /// A TLB with [`DEFAULT_ENTRIES`] entries per access type.
    #[must_use]
    pub fn new(space: Arc<AddressSpace>) -> Tlb {
        Tlb::with_entries(space, DEFAULT_ENTRIES)
    }

    /// A TLB with `entries` entries per access type.
    ///
    /// `entries` is rounded up to a power of two and to at least one, because
    /// the index is a mask and a mask is the whole point.
    #[must_use]
    pub fn with_entries(space: Arc<AddressSpace>, entries: u64) -> Tlb {
        let entries = entries.max(1).next_power_of_two();
        let n = usize::try_from(entries).unwrap_or(usize::MAX);
        let epoch = Epoch {
            topology: space.generation(),
            translation: 0,
        };
        Tlb {
            space,
            sets: [
                vec![Entry::empty(); n].into_boxed_slice(),
                vec![Entry::empty(); n].into_boxed_slice(),
                vec![Entry::empty(); n].into_boxed_slice(),
            ],
            mask: entries - 1,
            stores: Vec::new(),
            epoch,
            stats: TlbStats::default(),
        }
    }

    /// The address space this TLB fronts.
    #[inline]
    #[must_use]
    pub fn space(&self) -> &Arc<AddressSpace> {
        &self.space
    }

    /// What this TLB has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> TlbStats {
        self.stats
    }

    /// The epoch every live entry was filled under.
    #[inline]
    #[must_use]
    pub fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// Adopt `epoch`, throwing everything away if it differs.
    ///
    /// Called at a block boundary — the same place a CPU checks its
    /// [`ExitFlag`](crate::core::sched::ExitFlag) — so that a stop-the-world
    /// retopology or an `SFENCE.VMA` is observed before the next access, not
    /// after it. Returns whether anything was thrown away.
    pub fn sync(&mut self, epoch: Epoch) -> bool {
        if self.epoch == epoch {
            return false;
        }
        self.epoch = epoch;
        self.flush();
        true
    }

    /// The topology half of the current epoch, read from the space itself.
    ///
    /// A caller that has no other source for it can build an [`Epoch`] from
    /// this and its own MMU's counter.
    #[inline]
    #[must_use]
    pub fn topology_generation(&self) -> u64 {
        self.space.generation()
    }

    /// Throw every entry away.
    pub fn flush(&mut self) {
        for set in &mut self.sets {
            set.fill(Entry::empty());
        }
        self.stores.clear();
        self.stats.flushes += 1;
    }

    /// Throw away whatever is cached for the page containing `addr`.
    ///
    /// Exact, and cheap: the index is a function of the page alone, so one
    /// page occupies exactly one entry per set however many contexts have
    /// touched it. What `SFENCE.VMA rs1, x0` — a flush of one address rather
    /// than the world — wants, when a core would rather not bump its
    /// translation generation for it.
    pub fn invalidate_page(&mut self, addr: u64) {
        let index = self.index(addr);
        for set in &mut self.sets {
            set[index] = Entry::empty();
        }
    }

    /// Read `width` bytes at guest address `addr`, whose physical address is
    /// `phys`.
    ///
    /// `phys` equals `addr` when translation is off. `kind` selects the set,
    /// and [`AccessKind::Store`] takes the slow path rather than reading out
    /// of the store set: a store entry was admitted on
    /// [`Perms::WRITE`](crate::core::space::Perms::WRITE) alone, so serving a
    /// read from one would answer where a write-only mapping must refuse.
    ///
    /// # Errors
    ///
    /// Exactly what [`AddressSpace::read`] would have said.
    ///
    /// [`AddressSpace::read`]: crate::core::space::AddressSpace::read
    pub fn read(
        &mut self,
        kind: AccessKind,
        addr: u64,
        phys: u64,
        width: Width,
        ctx: Context,
        attrs: MemAttrs,
    ) -> MemResult<u64> {
        let n = width.bytes() as usize;
        if matches!(kind, AccessKind::Store) {
            return self.space.read(phys, width, attrs);
        }
        if !within_page(addr, width) {
            self.stats.split += 1;
            return self.space.read(phys, width, attrs);
        }
        match self.probe(kind, addr, ctx) {
            Probe::Ram {
                store,
                offset,
                endian,
            } => {
                self.stats.hits += 1;
                let mut buf = [0u8; 8];
                self.stores[store].read_at(offset, &mut buf[..n])?;
                endian.load(&buf[..n], width)
            }
            Probe::Slow => {
                self.stats.slow += 1;
                self.space.read(phys, width, attrs)
            }
            Probe::Miss => {
                self.stats.misses += 1;
                self.fill(kind, addr, phys, ctx);
                self.space.read(phys, width, attrs)
            }
        }
    }

    /// Write the low `width` bytes of `value` at guest address `addr`, whose
    /// physical address is `phys`.
    ///
    /// # Errors
    ///
    /// Exactly what [`AddressSpace::write`] would have said.
    ///
    /// [`AddressSpace::write`]: crate::core::space::AddressSpace::write
    pub fn write(
        &mut self,
        addr: u64,
        phys: u64,
        width: Width,
        value: u64,
        ctx: Context,
        attrs: MemAttrs,
    ) -> MemResult {
        let n = width.bytes() as usize;
        if !within_page(addr, width) {
            self.stats.split += 1;
            return self.space.write(phys, width, value, attrs);
        }
        match self.probe(AccessKind::Store, addr, ctx) {
            Probe::Ram {
                store,
                offset,
                endian,
            } => {
                self.stats.hits += 1;
                let mut buf = [0u8; 8];
                endian.store(&mut buf[..n], width, value)?;
                self.stores[store].write_at(offset, &buf[..n])
            }
            Probe::Slow => {
                self.stats.slow += 1;
                self.space.write(phys, width, value, attrs)
            }
            Probe::Miss => {
                self.stats.misses += 1;
                self.fill(AccessKind::Store, addr, phys, ctx);
                self.space.write(phys, width, value, attrs)
            }
        }
    }

    /// Resolve the page holding `addr` and record the answer.
    ///
    /// Every condition below is a case where a cached addend would be a wrong
    /// answer rather than a slow one, so each is named:
    ///
    /// * The virtual and physical page offsets must agree — a translation
    ///   preserves them, and one that does not is not a page translation.
    /// * One flat entry must cover the **whole** guest page. A region that
    ///   ends mid-page would make the addend valid for part of it.
    /// * The entry must be [`is_direct_ram`] — one leaf, RAM, and no separate
    ///   write target, so there is nothing left to dispatch.
    /// * The leaf must not be **rebasable**. A bank switch slides its offset
    ///   and bumps no counter, so an addend taken from one goes stale
    ///   invisibly. This is the condition the whole design turns on.
    /// * The leaf must not **repeat**: a mirrored window wraps its offset
    ///   partway through, and an addend cannot express that.
    /// * The mapping must permit the direction. Note that a fetch is checked
    ///   for `READ`, not `EXEC`, because
    ///   [`Perms::EXEC`](crate::core::space::Perms::EXEC) is carried and not
    ///   enforced — matching the slow path exactly is the requirement, not
    ///   improving on it.
    /// * The constraints must be permissive, so no width, alignment,
    ///   secure-only or privileged-only rule survives to be skipped.
    ///
    /// Anything else is remembered as uncacheable, which costs one entry and
    /// saves re-probing the flat view on every access to an MMIO page.
    ///
    /// [`is_direct_ram`]: crate::core::space::FlatEntry::is_direct_ram
    pub fn fill(&mut self, kind: AccessKind, addr: u64, phys: u64, ctx: Context) {
        let index = self.index(addr);
        let tag = tag(addr, ctx);
        let entry = self.resolve(kind, addr, phys);
        if entry.store == Entry::SLOW {
            self.stats.refused += 1;
        }
        self.stats.fills += 1;
        self.sets[set_of(kind)][index] = Entry { tag, ..entry };
    }

    /// The resolution for `phys`, or an uncacheable marker.
    fn resolve(&mut self, kind: AccessKind, addr: u64, phys: u64) -> Entry {
        let slow = Entry::empty();
        // The view's read guard is released before the store is interned, so
        // no path here holds the topology lock while touching this TLB.
        let Some((store, offset, endian)) = self.probe_space(kind, addr, phys) else {
            return slow;
        };
        let Some(index) = self.intern(&store) else {
            return slow;
        };
        Entry {
            tag: Entry::EMPTY,
            addend: offset.wrapping_sub(addr & !PAGE_MASK),
            store: index,
            endian,
        }
    }

    /// Ask the address space what backs `phys`, under the conditions
    /// [`Tlb::fill`] documents. `None` means "not cacheable".
    fn probe_space(
        &self,
        kind: AccessKind,
        addr: u64,
        phys: u64,
    ) -> Option<(Arc<RamStore>, u64, Endian)> {
        if addr & PAGE_MASK != phys & PAGE_MASK {
            return None;
        }
        let page = phys & !PAGE_MASK;
        let view = self.space.try_view()?;
        let entry = view.flat_view().entry(view.locate(page)?)?;
        if entry.start() > page || entry.end() < page.checked_add(PAGE_SIZE)? {
            return None;
        }
        if !entry.is_direct_ram() {
            return None;
        }
        let leaf = entry.leaf()?;
        if leaf.is_rebasable() || leaf.period().is_some() {
            return None;
        }
        let need = match kind {
            // EXEC is carried, not enforced: `FlatLeaf::read` checks READ for
            // a fetch too, and this path exists to agree with it.
            AccessKind::Fetch | AccessKind::Load => Perms::READ,
            AccessKind::Store => Perms::WRITE,
        };
        if !leaf.perms().contains(need) || !permissive(leaf.constraints()) {
            return None;
        }
        let FlatTarget::Ram(store) = leaf.target() else {
            return None;
        };
        Some((
            Arc::clone(store),
            leaf.offset_of(page - entry.start()),
            entry.endian(),
        ))
    }

    /// The index of `store` in the addend table, adding it if there is room.
    fn intern(&mut self, store: &Arc<RamStore>) -> Option<u32> {
        if let Some(i) = self.stores.iter().position(|s| Arc::ptr_eq(s, store)) {
            return u32::try_from(i).ok();
        }
        if self.stores.len() >= MAX_STORES {
            return None;
        }
        self.stores.push(Arc::clone(store));
        u32::try_from(self.stores.len() - 1).ok()
    }

    /// Look one page up. Mask, compare, add — and nothing else.
    #[inline]
    fn probe(&self, kind: AccessKind, addr: u64, ctx: Context) -> Probe {
        let entry = &self.sets[set_of(kind)][self.index(addr)];
        if entry.tag != tag(addr, ctx) {
            return Probe::Miss;
        }
        if entry.store == Entry::SLOW {
            return Probe::Slow;
        }
        Probe::Ram {
            store: entry.store as usize,
            offset: addr.wrapping_add(entry.addend),
            endian: entry.endian,
        }
    }

    #[inline]
    fn index(&self, addr: u64) -> usize {
        // `mask` is bounded by the allocation, so this always fits.
        ((addr >> 12) & self.mask) as usize
    }
}

/// What a probe found.
enum Probe {
    Ram {
        store: usize,
        offset: u64,
        endian: Endian,
    },
    Slow,
    Miss,
}

/// The set an access type reads.
#[inline]
const fn set_of(kind: AccessKind) -> usize {
    match kind {
        AccessKind::Fetch => 0,
        AccessKind::Load => 1,
        AccessKind::Store => 2,
    }
}

/// The tag for a page under a context.
#[inline]
const fn tag(addr: u64, ctx: Context) -> u64 {
    (addr & !PAGE_MASK) | ctx.bits() | Entry::VALID
}

/// Whether a `width`-byte access at `addr` stays inside one page.
#[inline]
const fn within_page(addr: u64, width: Width) -> bool {
    (addr & PAGE_MASK) + width.bytes() <= PAGE_SIZE
}

/// Whether a region's constraints leave nothing for the fast path to check.
///
/// Deliberately conservative: a region that accepts every width at every
/// alignment from every requester has no rule an addend could skip. Anything
/// narrower takes the slow path, where the real check lives.
#[inline]
fn permissive(c: AccessConstraints) -> bool {
    c.min == Width::U8
        && c.max == Width::U64
        && !c.natural_alignment
        && !c.secure_only
        && !c.privileged_only
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::error::BusError;
    use crate::core::space::{Region, RomStore, UnassignedPolicy};
    use alloc::sync::Arc;

    const BASE: u64 = 0x2000_0000;
    const SIZE: u64 = 8 * PAGE_SIZE;

    fn space() -> (Arc<AddressSpace>, Arc<RamStore>) {
        let ram = Arc::new(RamStore::new(SIZE));
        let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), BASE)
            .expect("one region maps");
        (Arc::new(space), ram)
    }

    fn tlb() -> (Tlb, Arc<RamStore>) {
        let (space, ram) = space();
        (Tlb::with_entries(space, 64), ram)
    }

    const BARE: Context = Context {
        level: 3,
        translating: false,
    };

    #[test]
    fn a_hit_returns_what_the_address_space_would_have_returned() {
        let (mut tlb, ram) = tlb();
        ram.write_at(0x40, &0x1122_3344_5566_7788u64.to_le_bytes())
            .expect("in range");
        let addr = BASE + 0x40;
        // First access misses and fills; the value still comes from the space.
        let first = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U64,
            BARE,
            MemAttrs::DEFAULT,
        );
        assert_eq!(first, Ok(0x1122_3344_5566_7788));
        assert_eq!(tlb.stats().misses, 1);
        // The second is served from the entry, and agrees.
        let second = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U64,
            BARE,
            MemAttrs::DEFAULT,
        );
        assert_eq!(second, first);
        assert_eq!(tlb.stats().hits, 1);
        assert_eq!(
            second,
            tlb.space().read(addr, Width::U64, MemAttrs::DEFAULT)
        );
    }

    #[test]
    fn a_write_through_the_fast_path_marks_the_store_dirty() {
        let (mut tlb, ram) = tlb();
        let addr = BASE + 0x1004;
        for _ in 0..2 {
            tlb.write(addr, addr, Width::U32, 0xdead_beef, BARE, MemAttrs::DEFAULT)
                .expect("the write lands");
        }
        assert_eq!(tlb.stats().hits, 1, "the second write hit");
        assert_eq!(ram.read_u8(0x1004), Ok(0xef));
        // Snapshots read this bitmap, so a fast-path write that skipped it
        // would produce a save state missing the guest's own memory.
        assert!(ram.is_page_dirty(0x1004 / ram.page_size()));
    }

    #[test]
    fn loads_and_stores_do_not_share_a_set() {
        let (mut tlb, _ram) = tlb();
        let addr = BASE + 0x2000;
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 1);
        // The store set has never seen this page, so the first store misses.
        let before = tlb.stats().misses;
        tlb.write(addr, addr, Width::U8, 1, BARE, MemAttrs::DEFAULT)
            .expect("the write lands");
        assert_eq!(tlb.stats().misses, before + 1);
    }

    #[test]
    fn a_different_privilege_level_is_a_different_entry() {
        let (mut tlb, _ram) = tlb();
        let addr = BASE + 0x3000;
        let user = Context {
            level: 0,
            translating: true,
        };
        let supervisor = Context {
            level: 1,
            translating: true,
        };
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                user,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 1);
        let before = tlb.stats().misses;
        let _ = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U8,
            supervisor,
            MemAttrs::DEFAULT,
        );
        assert_eq!(
            tlb.stats().misses,
            before + 1,
            "a supervisor access must not hit a user entry"
        );
    }

    #[test]
    fn a_bare_access_never_hits_a_translated_entry_at_the_same_number() {
        let (mut tlb, _ram) = tlb();
        let addr = BASE + 0x4000;
        let paged = Context {
            level: 1,
            translating: true,
        };
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                paged,
                MemAttrs::DEFAULT,
            );
        }
        let before = tlb.stats().misses;
        let _ = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U8,
            BARE,
            MemAttrs::DEFAULT,
        );
        assert_eq!(tlb.stats().misses, before + 1);
    }

    #[test]
    fn an_access_that_spans_two_pages_is_never_served_from_an_entry() {
        let (mut tlb, ram) = tlb();
        ram.write_at(PAGE_SIZE - 4, &[1, 2, 3, 4, 5, 6, 7, 8])
            .expect("in range");
        let addr = BASE + PAGE_SIZE - 4;
        let want = tlb.space().read(addr, Width::U64, MemAttrs::DEFAULT);
        for _ in 0..4 {
            assert_eq!(
                tlb.read(
                    AccessKind::Load,
                    addr,
                    addr,
                    Width::U64,
                    BARE,
                    MemAttrs::DEFAULT
                ),
                want
            );
        }
        assert_eq!(tlb.stats().hits, 0);
        assert_eq!(tlb.stats().split, 4);
    }

    #[test]
    fn a_rom_page_is_remembered_as_uncacheable_rather_than_reprobed() {
        let (space, _ram) = space();
        let rom = Arc::new(RomStore::zeroed(PAGE_SIZE));
        space
            .topology()
            .map(
                Region::rom("rom", rom, crate::core::space::RomWrite::Ignore),
                BASE + SIZE,
            )
            .expect("the rom maps");
        let mut tlb = Tlb::with_entries(Arc::clone(&space), 64);
        let addr = BASE + SIZE;
        for _ in 0..3 {
            assert_eq!(
                tlb.read(
                    AccessKind::Load,
                    addr,
                    addr,
                    Width::U8,
                    BARE,
                    MemAttrs::DEFAULT
                ),
                Ok(0)
            );
        }
        assert_eq!(tlb.stats().misses, 1, "the flat view is probed once");
        assert_eq!(tlb.stats().slow, 2, "and then remembered as uncacheable");
        assert_eq!(tlb.stats().refused, 1);
        assert_eq!(tlb.stats().hits, 0);
    }

    #[test]
    fn a_rebasable_window_is_never_cached() {
        // A bank switch slides an alias's offset and bumps no generation
        // counter, so an addend taken from one would go stale invisibly. This
        // is the condition the design turns on; if it is ever relaxed, this
        // test is what says so.
        let (space, ram) = space();
        let window = Region::alias(
            "bank",
            Region::ram("backing", Arc::clone(&ram)),
            0,
            PAGE_SIZE,
        )
        .expect("an alias over the ram");
        space
            .topology()
            .map(window, BASE + SIZE)
            .expect("the window maps");
        let mut tlb = Tlb::with_entries(Arc::clone(&space), 64);
        let addr = BASE + SIZE;
        for _ in 0..3 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 0, "a rebasable leaf is never cached");
        assert_eq!(tlb.stats().refused, 1);
    }

    #[test]
    fn a_read_only_mapping_is_not_cached_for_stores() {
        let (space, ram) = space();
        space
            .topology()
            .map_with_perms(
                Region::ram("ro", Arc::clone(&ram)),
                BASE + SIZE,
                Perms::READ,
            )
            .expect("the read-only view maps");
        let mut tlb = Tlb::with_entries(Arc::clone(&space), 64);
        let addr = BASE + SIZE;
        for _ in 0..3 {
            assert_eq!(
                tlb.write(addr, addr, Width::U8, 0xaa, BARE, MemAttrs::DEFAULT),
                Err(BusError::Protected),
                "the fast path must refuse exactly where the slow path does"
            );
        }
        assert_eq!(tlb.stats().hits, 0);
        // and the load side of the same page is cacheable.
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 1);
    }

    #[test]
    fn a_write_only_mapping_never_answers_a_read_from_its_store_entry() {
        // A store entry is admitted on WRITE alone, so it must not be able to
        // answer a read. The set the read would land in is the one the store
        // filled, which is exactly why `read` refuses the store kind outright.
        let (space, ram) = space();
        space
            .topology()
            .map_with_perms(
                Region::ram("wo", Arc::clone(&ram)),
                BASE + SIZE,
                Perms::WRITE,
            )
            .expect("the write-only view maps");
        let mut tlb = Tlb::with_entries(Arc::clone(&space), 64);
        let addr = BASE + SIZE;
        for _ in 0..2 {
            tlb.write(addr, addr, Width::U8, 0x5a, BARE, MemAttrs::DEFAULT)
                .expect("the write lands");
        }
        assert_eq!(tlb.stats().hits, 1, "the store side is cached");
        for kind in [AccessKind::Load, AccessKind::Fetch, AccessKind::Store] {
            assert_eq!(
                tlb.read(kind, addr, addr, Width::U8, BARE, MemAttrs::DEFAULT),
                Err(BusError::Protected),
                "a {kind:?} through a write-only mapping must be refused"
            );
        }
    }

    #[test]
    fn a_topology_change_invalidates_every_entry() {
        let (mut tlb, _ram) = tlb();
        let addr = BASE + 0x5000;
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 1);
        let ram2 = Arc::new(RamStore::new(PAGE_SIZE));
        tlb.space()
            .topology()
            .map(Region::ram("other", ram2), BASE + 0x10_0000)
            .expect("a second region maps");
        let epoch = Epoch {
            topology: tlb.topology_generation(),
            translation: 0,
        };
        assert!(tlb.sync(epoch), "the generation moved");
        let before = tlb.stats().misses;
        let _ = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U8,
            BARE,
            MemAttrs::DEFAULT,
        );
        assert_eq!(tlb.stats().misses, before + 1);
    }

    #[test]
    fn a_translation_generation_bump_invalidates_every_entry() {
        let (mut tlb, _ram) = tlb();
        let paged = Context {
            level: 1,
            translating: true,
        };
        let addr = BASE + 0x6000;
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                paged,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 1);
        // An SFENCE.VMA: the same virtual address now means something else.
        let mut epoch = tlb.epoch();
        epoch.translation += 1;
        assert!(tlb.sync(epoch));
        let before = tlb.stats().misses;
        let _ = tlb.read(
            AccessKind::Load,
            addr,
            addr,
            Width::U8,
            paged,
            MemAttrs::DEFAULT,
        );
        assert_eq!(tlb.stats().misses, before + 1);
    }

    #[test]
    fn one_page_can_be_invalidated_without_the_rest() {
        let (mut tlb, _ram) = tlb();
        let a = BASE;
        let b = BASE + PAGE_SIZE;
        for addr in [a, b, a, b] {
            let _ = tlb.read(
                AccessKind::Load,
                addr,
                addr,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 2);
        tlb.invalidate_page(a);
        let before = tlb.stats();
        let _ = tlb.read(AccessKind::Load, b, b, Width::U8, BARE, MemAttrs::DEFAULT);
        assert_eq!(tlb.stats().hits, before.hits + 1, "b survived");
        let before = tlb.stats();
        let _ = tlb.read(AccessKind::Load, a, a, Width::U8, BARE, MemAttrs::DEFAULT);
        assert_eq!(tlb.stats().misses, before.misses + 1, "a did not");
    }

    #[test]
    fn the_fast_path_and_the_slow_path_agree_on_every_width_and_offset() {
        let (mut tlb, ram) = tlb();
        for i in 0..256u64 {
            ram.write_u8(i, (i * 7) as u8).expect("in range");
        }
        for width in [Width::U8, Width::U16, Width::U32, Width::U64] {
            for off in 0..64u64 {
                let addr = BASE + off;
                let want = tlb.space().read(addr, width, MemAttrs::DEFAULT);
                // Twice, so the second is a hit.
                for _ in 0..2 {
                    assert_eq!(
                        tlb.read(AccessKind::Load, addr, addr, width, BARE, MemAttrs::DEFAULT),
                        want,
                        "width {width:?} at {addr:#x}"
                    );
                }
            }
        }
        assert!(tlb.stats().hits > 0);
    }

    #[test]
    fn an_unmapped_page_faults_the_same_way_twice() {
        let (mut tlb, _ram) = tlb();
        let addr = BASE + 0x100_0000;
        for _ in 0..3 {
            assert_eq!(
                tlb.read(
                    AccessKind::Load,
                    addr,
                    addr,
                    Width::U8,
                    BARE,
                    MemAttrs::DEFAULT
                ),
                Err(BusError::Unassigned)
            );
        }
    }

    #[test]
    fn a_page_the_region_only_half_covers_is_not_cached() {
        // The region ends mid-page, so one addend cannot describe the page:
        // the top half is unmapped and must still fault.
        let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
        let ram = Arc::new(RamStore::new(PAGE_SIZE / 2));
        space
            .topology()
            .map(Region::ram("half", ram), BASE)
            .expect("the half page maps");
        let mut tlb = Tlb::with_entries(Arc::new(space), 64);
        for _ in 0..2 {
            let _ = tlb.read(
                AccessKind::Load,
                BASE,
                BASE,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT,
            );
        }
        assert_eq!(tlb.stats().hits, 0);
        assert_eq!(
            tlb.read(
                AccessKind::Load,
                BASE + PAGE_SIZE - 1,
                BASE + PAGE_SIZE - 1,
                Width::U8,
                BARE,
                MemAttrs::DEFAULT
            ),
            Err(BusError::Unassigned)
        );
    }
}
