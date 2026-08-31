//! The level-3 memory map: an address space with no devices in it.
//!
//! `ROADMAP.md` phase 5b asks for *"a memory map with no devices in it. Not a
//! `Device`, not on a bus — nothing in the guest can address it."* This is
//! that: an [`AddressSpace`] whose every region is anonymous RAM, plus the
//! bookkeeping that says which ranges exist, how big they are, what they are
//! allowed to be used for, and what to call them.
//!
//! # Why the bookkeeping is a list and not a bitmap
//!
//! An address space can already answer "what is mapped at `x`". It cannot
//! answer "what mappings exist", and a level-3 consumer needs that in three
//! places: `/proc/self/maps`, a snapshot, and deciding where to put the next
//! anonymous mapping. So [`UserMemory`] keeps an ordered list of ranges — a
//! `BTreeMap` keyed by base, so enumeration order is an address order and not
//! a hash order (CLAUDE.md, "Determinism").
//!
//! # What this is not
//!
//! It is not `mmap(2)`. There is no file, no descriptor, no `MAP_` flag and no
//! errno here, because a file is an operating system's idea and §2.1 puts
//! those in the consumer. What is here is the three operations a memory
//! management unit actually performs — make a range exist, stop it existing,
//! change what it permits — and the ability to say what exists now.
//!
//! # The gap, stated rather than papered over
//!
//! [`Prot`] is enforced on accesses **this module** performs — the ones a
//! consumer makes on the guest's behalf. It is only partly enforced on
//! accesses the *guest* performs: a [`Prot::NONE`] range is simply absent from
//! the address space, so the guest faults on it, but a read-only range is
//! mapped read-write and a guest store to it succeeds. Fixing that needs a
//! per-mapping permission bit in [`core::space`](crate::core::space) and a
//! write-fault path out of it, which do not exist. The same missing mechanism
//! is why [`UserMemory::duplicate`] copies eagerly instead of sharing
//! copy-on-write. Both are generic mechanisms and belong in `core::space` when
//! they land (`ROADMAP.md` §0, "generic first"), not as a special case here.

use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::{BusError, Error, Result};
use crate::core::space::{AddressSpace, MappingId, MemAttrs, RamStore, Region};
use crate::core::state::{Sink, Source};
use crate::core::sync::{self, LockRank};

/// The granularity every range in this module is aligned and sized to.
///
/// 4 KiB, which is the page size of every architecture rsemu has a 64-bit core
/// for. It is a constant rather than a parameter because a consumer that had
/// to ask would have to ask on every call, and no guest in level 3 can observe
/// a different answer: there is no page table for it to look at.
pub const PAGE_SIZE: u64 = 4096;

/// What a range of the map may be used for.
///
/// A bit set rather than an enumeration, because the three permissions
/// genuinely combine and every one of the eight combinations is a thing a real
/// program asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(transparent)]
pub struct Prot(pub u8);

impl Prot {
    /// No access at all. Such a range is *absent* from the address space, so
    /// the guest faults on it — which is exactly the intent.
    pub const NONE: Prot = Prot(0);
    /// The range may be read.
    pub const READ: Prot = Prot(1);
    /// The range may be written.
    pub const WRITE: Prot = Prot(2);
    /// Instructions may be fetched from the range.
    pub const EXEC: Prot = Prot(4);
    /// Readable and writable — an anonymous mapping's usual shape.
    pub const RW: Prot = Prot(3);
    /// Readable and executable — a text segment's usual shape.
    pub const RX: Prot = Prot(5);
    /// Everything.
    pub const RWX: Prot = Prot(7);

    /// Whether every bit of `other` is set here.
    #[must_use]
    pub const fn contains(self, other: Prot) -> bool {
        self.0 & other.0 == other.0
    }

    /// The union of two permission sets.
    #[must_use]
    pub const fn union(self, other: Prot) -> Prot {
        Prot(self.0 | other.0)
    }

    /// Whether this is [`Prot::NONE`].
    #[must_use]
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
}

impl core::fmt::Display for Prot {
    /// The `rwx` form `/proc/*/maps` uses, so a consumer that prints one does
    /// not have to reinvent the spelling.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let bit = |set: bool, c: char| if set { c } else { '-' };
        write!(
            f,
            "{}{}{}",
            bit(self.contains(Prot::READ), 'r'),
            bit(self.contains(Prot::WRITE), 'w'),
            bit(self.contains(Prot::EXEC), 'x'),
        )
    }
}

/// One range of the map, as a consumer sees it.
///
/// Plain data, snapshot-shaped and `/proc/self/maps`-shaped: this is what
/// [`UserMemory::mappings`] hands back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MappingInfo {
    /// The first address in the range.
    pub base: u64,
    /// Its length in bytes, always a multiple of [`PAGE_SIZE`].
    pub len: u64,
    /// What it permits.
    pub prot: Prot,
    /// What to call it. Free-form and consumer-chosen — `[heap]`, `[stack]`,
    /// a path — because naming is the consumer's vocabulary, not ours.
    pub name: String,
}

/// One range, and the storage behind it.
#[derive(Debug)]
struct Vma {
    len: u64,
    prot: Prot,
    store: Arc<RamStore>,
    /// `None` while the range is [`Prot::NONE`] and therefore absent from the
    /// address space.
    mapping: Option<MappingId>,
    name: String,
}

/// The ranges, and where to place the next unplaced one.
#[derive(Debug)]
struct Inner {
    vmas: BTreeMap<u64, Vma>,
    /// Where an unplaced mapping search starts, growing downwards, as every
    /// Linux-shaped layout does.
    hint: u64,
    /// The lowest address an unplaced mapping may be given.
    floor: u64,
}

/// A level-3 process's memory: an address space made only of anonymous RAM.
///
/// Held as an `Arc` and shared by every core that runs in it, which is what
/// makes threads of one process threads: they share this object, and processes
/// do not. Nothing here knows what a thread or a process *is* — that is the
/// consumer's model — but the sharing this type permits is what lets the
/// consumer have one.
///
/// # Locking
///
/// One [`sync::Mutex`] at [`LockRank::MACHINE`] — the rank above
/// `TOPOLOGY`, which is the one [`AddressSpace`] takes. That is the honest
/// rank rather than a convenient one: this type *is* the level-3 analogue of a
/// machine's memory configuration, and every call travels outward from here
/// into the address space and never back. Holding it across a remap is
/// therefore legal, and it has to be: placing a mapping and creating it must
/// be one atomic step, or two threads of a process calling `mmap` at once can
/// be handed the same address.
///
/// Guest accesses do not touch this lock at all — a core reaches
/// [`UserMemory::space`] directly — so nothing on the hot path queues behind
/// an `mmap`.
#[derive(Debug)]
pub struct UserMemory {
    space: Arc<AddressSpace>,
    inner: sync::Mutex<Inner>,
}

impl UserMemory {
    /// An empty map over an address space `bits` wide.
    ///
    /// Unplaced mappings are placed downwards from the top of the space,
    /// leaving the bottom [`PAGE_SIZE`] unmapped so a null dereference faults
    /// — which is a property of the *map*, not of an operating system, and is
    /// why it is here rather than in the consumer.
    ///
    /// # Panics
    ///
    /// If `bits` is 0 or greater than 64, which [`AddressSpace::new`] rejects.
    #[must_use]
    pub fn new(bits: u32) -> UserMemory {
        let space = AddressSpace::new("user", bits);
        let top = if bits >= 64 {
            !(PAGE_SIZE - 1)
        } else {
            1u64 << bits
        };
        UserMemory {
            space: Arc::new(space),
            inner: sync::Mutex::with_rank(
                LockRank::MACHINE,
                Inner {
                    vmas: BTreeMap::new(),
                    hint: top,
                    floor: PAGE_SIZE,
                },
            ),
        }
    }

    /// The address space a core attaches to.
    ///
    /// This is the whole point of reusing [`core::space`](crate::core::space)
    /// rather than inventing a level-3 memory model: the same cores, the same
    /// snapshots and the same debugger work at all three levels because they
    /// all read guest memory the same way (§2, "Three levels of execution").
    #[must_use]
    pub fn space(&self) -> &Arc<AddressSpace> {
        &self.space
    }

    /// Where an unplaced mapping search starts, and the lowest address it may
    /// return.
    ///
    /// The default leaves the bottom page free. A consumer that wants a
    /// different layout — a low `mmap` base, a reserved region — says so here
    /// rather than by placing everything by hand.
    ///
    /// # Errors
    ///
    /// If `floor` is not below `top`, or either is not page aligned.
    pub fn set_placement(&self, floor: u64, top: u64) -> Result<()> {
        if !floor.is_multiple_of(PAGE_SIZE) || !top.is_multiple_of(PAGE_SIZE) || floor >= top {
            return Err(config(alloc::format!(
                "placement range {floor:#x}..{top:#x} must be page aligned and non-empty"
            )));
        }
        let mut inner = self.inner.lock();
        inner.floor = floor;
        inner.hint = top;
        Ok(())
    }

    /// Make `len` bytes exist at `base`, replacing anything already there.
    ///
    /// The `MAP_FIXED` shape, and the one a program loader wants: it is told
    /// where a segment goes and has no say in it.
    ///
    /// # Errors
    ///
    /// If `base` or `len` is not page aligned, `len` is zero, or the range
    /// runs off the end of the address space.
    pub fn map_at(&self, base: u64, len: u64, prot: Prot, name: &str) -> Result<()> {
        self.check_range(base, len)?;
        let mut inner = self.inner.lock();
        self.unmap_locked(&mut inner, base, len)?;
        self.insert_locked(&mut inner, base, len, prot, name, None)
    }

    /// Make `len` bytes exist somewhere, and say where.
    ///
    /// Placement is top-down from the hint and is a pure function of the map's
    /// current contents, so the same sequence of calls produces the same
    /// addresses on every host and on every run — which is what makes a
    /// level-3 run reproducible at all.
    ///
    /// Choosing the address and creating the mapping happen under one lock, so
    /// two threads of a process calling this at the same time cannot be handed
    /// the same address.
    ///
    /// # Errors
    ///
    /// If `len` is zero or not page aligned, or there is no free range left.
    pub fn map(&self, len: u64, prot: Prot, name: &str) -> Result<u64> {
        if len == 0 || !len.is_multiple_of(PAGE_SIZE) {
            return Err(config(alloc::format!(
                "mapping length {len:#x} must be a non-zero multiple of {PAGE_SIZE:#x}"
            )));
        }
        let mut inner = self.inner.lock();
        let base = Self::free_range(&inner, len)?;
        self.insert_locked(&mut inner, base, len, prot, name, None)?;
        Ok(base)
    }

    /// The address a mapping of `len` bytes would be placed at, without
    /// placing it.
    ///
    /// Advisory, and only that: another thread may map something there before
    /// the caller does. [`map`](UserMemory::map) does both steps at once and is
    /// what a consumer should reach for; this exists so a consumer can *ask*.
    ///
    /// # Errors
    ///
    /// If nothing that big is free.
    pub fn find_free(&self, len: u64) -> Result<u64> {
        Self::free_range(&self.inner.lock(), len)
    }

    /// Stop `len` bytes at `base` existing.
    ///
    /// Ranges that only partly overlap are trimmed and, where the removal is
    /// in their middle, split. Unmapping a range nothing is mapped in is not
    /// an error — the caller asked for it to be gone and it is.
    ///
    /// # Errors
    ///
    /// If `base` or `len` is not page aligned, or the range runs off the end
    /// of the address space.
    pub fn unmap(&self, base: u64, len: u64) -> Result<()> {
        self.check_range(base, len)?;
        let mut inner = self.inner.lock();
        self.unmap_locked(&mut inner, base, len)
    }

    /// Change what `len` bytes at `base` permit.
    ///
    /// Ranges that only partly overlap are split, exactly as [`unmap`] splits
    /// them. A range that is not mapped is skipped rather than created:
    /// permission is a property of a mapping and there is nothing to give it
    /// to.
    ///
    /// [`unmap`]: UserMemory::unmap
    ///
    /// # Errors
    ///
    /// If `base` or `len` is not page aligned, or the range runs off the end
    /// of the address space.
    pub fn protect(&self, base: u64, len: u64, prot: Prot) -> Result<()> {
        self.check_range(base, len)?;
        let end = base + len;
        let mut inner = self.inner.lock();
        for vbase in Self::overlapping(&inner, base, end) {
            let Some(vma) = inner.vmas.remove(&vbase) else {
                continue;
            };
            if let Some(id) = vma.mapping {
                self.space.topology().unmap(id)?;
            }
            let vend = vbase + vma.len;
            let head = base.max(vbase);
            let tail = end.min(vend);
            let src = |offset| Some((Arc::clone(&vma.store), offset));
            if vbase < head {
                self.insert_locked(&mut inner, vbase, head - vbase, vma.prot, &vma.name, src(0))?;
            }
            self.insert_locked(
                &mut inner,
                head,
                tail - head,
                prot,
                &vma.name,
                src(head - vbase),
            )?;
            if vend > tail {
                self.insert_locked(
                    &mut inner,
                    tail,
                    vend - tail,
                    vma.prot,
                    &vma.name,
                    src(tail - vbase),
                )?;
            }
        }
        Ok(())
    }

    /// Every range that exists, in address order.
    ///
    /// Address order rather than insertion order because that is the order
    /// `/proc/*/maps` is in and the order a human reads a memory map in, and
    /// because it is an order at all — a `HashMap` here would make a snapshot
    /// hash depend on nothing (CLAUDE.md, "Determinism").
    #[must_use]
    pub fn mappings(&self) -> Vec<MappingInfo> {
        let inner = self.inner.lock();
        inner
            .vmas
            .iter()
            .map(|(base, vma)| MappingInfo {
                base: *base,
                len: vma.len,
                prot: vma.prot,
                name: vma.name.clone(),
            })
            .collect()
    }

    /// What is mapped at `addr`, if anything.
    #[must_use]
    pub fn mapping_at(&self, addr: u64) -> Option<MappingInfo> {
        let inner = self.inner.lock();
        inner
            .vmas
            .range(..=addr)
            .next_back()
            .filter(|(base, vma)| **base + vma.len > addr)
            .map(|(base, vma)| MappingInfo {
                base: *base,
                len: vma.len,
                prot: vma.prot,
                name: vma.name.clone(),
            })
    }

    /// How many bytes are mapped in total.
    #[must_use]
    pub fn mapped_bytes(&self) -> u64 {
        let inner = self.inner.lock();
        inner.vmas.values().map(|vma| vma.len).sum()
    }

    // -----------------------------------------------------------------
    // Access on the guest's behalf
    // -----------------------------------------------------------------

    /// Read `dst.len()` bytes from `addr`, checking [`Prot::READ`].
    ///
    /// What a consumer uses when the guest asked it to look at something. The
    /// check is the point: a guest that passes a pointer into a
    /// [`Prot::NONE`] range must be told no, and the check has to happen here
    /// because the address space cannot make it.
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if any of the range is unmapped,
    /// [`BusError::BadAccess`] if any of it is not readable.
    pub fn read_bytes(&self, addr: u64, dst: &mut [u8]) -> Result<()> {
        self.check_prot(addr, dst.len() as u64, Prot::READ)?;
        self.space.read_bytes(addr, dst, MemAttrs::DEFAULT)?;
        Ok(())
    }

    /// Write `src` at `addr`, checking [`Prot::WRITE`].
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if any of the range is unmapped,
    /// [`BusError::BadAccess`] if any of it is not writable.
    pub fn write_bytes(&self, addr: u64, src: &[u8]) -> Result<()> {
        self.check_prot(addr, src.len() as u64, Prot::WRITE)?;
        self.space.write_bytes(addr, src, MemAttrs::DEFAULT)?;
        Ok(())
    }

    /// Write `src` at `addr` **without** checking permission.
    ///
    /// How a program loader fills a read-only text segment, and how an
    /// anonymous mapping is seeded before the guest can see it. Separate from
    /// [`write_bytes`](UserMemory::write_bytes) rather than a flag, because
    /// the two have different callers: one is acting for the guest and must be
    /// refused, the other is acting for the consumer and must not be.
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if any of the range is unmapped. Permission is
    /// not consulted; existence still is.
    pub fn init_bytes(&self, addr: u64, src: &[u8]) -> Result<()> {
        // Written through the range's own store rather than through the
        // address space, because a `Prot::NONE` range is deliberately absent
        // from the space and a loader still has to be able to fill it. The
        // range must not straddle two mappings; a loader fills one segment at
        // a time and a snapshot one range at a time.
        let (store, offset) = self.store_at(addr, src.len() as u64)?;
        store.write_at(offset, src)?;
        Ok(())
    }

    /// Read a little-endian `u64` from `addr`.
    ///
    /// # Errors
    ///
    /// As [`read_bytes`](UserMemory::read_bytes).
    pub fn read_u64(&self, addr: u64) -> Result<u64> {
        let mut buf = [0u8; 8];
        self.read_bytes(addr, &mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write a little-endian `u64` at `addr`.
    ///
    /// # Errors
    ///
    /// As [`write_bytes`](UserMemory::write_bytes).
    pub fn write_u64(&self, addr: u64, value: u64) -> Result<()> {
        self.write_bytes(addr, &value.to_le_bytes())
    }

    // -----------------------------------------------------------------
    // Copying and snapshots
    // -----------------------------------------------------------------

    /// A second map with the same ranges and the same bytes.
    ///
    /// What a consumer's `fork(2)` needs. The copy is **eager**: every mapped
    /// byte is copied now. Copy-on-write would be better and needs a
    /// write-fault path out of [`core::space`](crate::core::space) that does
    /// not exist — see the module docs. Nothing about this signature changes
    /// when it does.
    ///
    /// # Errors
    ///
    /// If the new map cannot be built, which in practice means allocation.
    pub fn duplicate(&self) -> Result<Arc<UserMemory>> {
        let bits = self.space.bits();
        let copy = Arc::new(UserMemory::new(bits));
        let (floor, hint) = {
            let inner = self.inner.lock();
            (inner.floor, inner.hint)
        };
        {
            let mut inner = copy.inner.lock();
            inner.floor = floor;
            inner.hint = hint;
        }
        for info in self.mappings() {
            copy.map_at(info.base, info.len, info.prot, &info.name)?;
            let mut buf = alloc::vec![0u8; info.len as usize];
            self.raw_read(info.base, &mut buf)?;
            copy.init_bytes(info.base, &buf)?;
        }
        Ok(copy)
    }

    /// Write the whole map — its ranges and their bytes — to `sink`.
    ///
    /// Generic over [`Sink`] rather than taking a
    /// [`ChunkWriter`](crate::core::state::ChunkWriter) because a
    /// [`UserMemory`] is not a [`Device`](crate::core::Device) and has no
    /// instance path of its own: the consumer decides which chunk it belongs
    /// in and what to call it.
    ///
    /// # Errors
    ///
    /// If the sink fails, or a mapped range cannot be read back.
    pub fn save<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        let (floor, hint) = {
            let inner = self.inner.lock();
            (inner.floor, inner.hint)
        };
        sink.write_u32(self.space.bits())?;
        sink.write_u64(floor)?;
        sink.write_u64(hint)?;
        let maps = self.mappings();
        sink.write_seq_len(maps.len() as u64)?;
        for info in maps {
            sink.write_u64(info.base)?;
            sink.write_u64(info.len)?;
            sink.write_u8(info.prot.0)?;
            sink.write_str(&info.name)?;
            let mut buf = alloc::vec![0u8; info.len as usize];
            self.raw_read(info.base, &mut buf)?;
            sink.write_all(&buf)?;
        }
        Ok(())
    }

    /// Read a map back from `source`, replacing everything this one holds.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the encoding is malformed or describes a map wider
    /// than this one.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, source: &mut S) -> Result<()> {
        let bits = source.read_u32()?;
        if bits != self.space.bits() {
            return Err(Error::State(alloc::format!(
                "snapshot holds a {bits}-bit map and this one is {}-bit",
                self.space.bits()
            )));
        }
        let floor = source.read_u64()?;
        let hint = source.read_u64()?;
        for info in self.mappings() {
            self.unmap(info.base, info.len)?;
        }
        {
            let mut inner = self.inner.lock();
            inner.floor = floor;
            inner.hint = hint;
        }
        let count = source.read_seq_len(25)?;
        for _ in 0..count {
            let base = source.read_u64()?;
            let len = source.read_u64()?;
            let prot = Prot(source.read_u8()?);
            let name = source.read_str()?;
            let bytes = source.take(usize::try_from(len).map_err(|_| {
                Error::State(alloc::format!(
                    "mapping of {len:#x} bytes does not fit in memory"
                ))
            })?)?;
            self.map_at(base, len, prot, name)?;
            self.init_bytes(base, bytes)?;
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Internals
    // -----------------------------------------------------------------

    /// Read without consulting permission, for `save` and `duplicate`.
    fn raw_read(&self, addr: u64, dst: &mut [u8]) -> Result<()> {
        let (store, offset) = self.store_at(addr, dst.len() as u64)?;
        store.read_at(offset, dst)?;
        Ok(())
    }

    /// The store holding `len` bytes at `addr`, and the offset into it.
    ///
    /// Refuses a range that straddles two mappings: every caller here works
    /// one range at a time, and silently splitting would hide a bug.
    fn store_at(&self, addr: u64, len: u64) -> Result<(Arc<RamStore>, u64)> {
        let inner = self.inner.lock();
        let Some((base, vma)) = inner.vmas.range(..=addr).next_back() else {
            return Err(Error::Bus(BusError::Unassigned));
        };
        let offset = addr - *base;
        if offset.checked_add(len).is_none_or(|end| end > vma.len) {
            return Err(Error::Bus(BusError::Unassigned));
        }
        Ok((Arc::clone(&vma.store), offset))
    }

    /// Create `len` bytes at `base`, optionally copying them from `src`.
    ///
    /// The one place a range comes into existence. A range that permits
    /// nothing is *not* put into the address space, so the guest faults on it
    /// while the bookkeeping still remembers it is reserved.
    fn insert_locked(
        &self,
        inner: &mut Inner,
        base: u64,
        len: u64,
        prot: Prot,
        name: &str,
        src: Option<(Arc<RamStore>, u64)>,
    ) -> Result<()> {
        let store = Arc::new(RamStore::new(len));
        if let Some((from, offset)) = src {
            // Copying rather than aliasing: sharing a store between two ranges
            // needs the copy-on-write mechanism the module docs name as
            // missing.
            let mut buf = alloc::vec![0u8; len as usize];
            from.read_at(offset, &mut buf)?;
            store.write_at(0, &buf)?;
        }
        let mapping = if prot.is_none() {
            None
        } else {
            let region = Region::ram(name, Arc::clone(&store));
            Some(self.space.topology().map(region, base)?)
        };
        inner.vmas.insert(
            base,
            Vma {
                len,
                prot,
                store,
                mapping,
                name: name.to_string(),
            },
        );
        Ok(())
    }

    /// [`unmap`](UserMemory::unmap) with the lock already held.
    fn unmap_locked(&self, inner: &mut Inner, base: u64, len: u64) -> Result<()> {
        let end = base + len;
        for vbase in Self::overlapping(inner, base, end) {
            let Some(vma) = inner.vmas.remove(&vbase) else {
                continue;
            };
            if let Some(id) = vma.mapping {
                self.space.topology().unmap(id)?;
            }
            // Whatever of the range survives is rebuilt as its own mapping,
            // carrying its bytes with it.
            let vend = vbase + vma.len;
            if vbase < base {
                self.insert_locked(
                    inner,
                    vbase,
                    base - vbase,
                    vma.prot,
                    &vma.name,
                    Some((Arc::clone(&vma.store), 0)),
                )?;
            }
            if vend > end {
                self.insert_locked(
                    inner,
                    end,
                    vend - end,
                    vma.prot,
                    &vma.name,
                    Some((Arc::clone(&vma.store), end - vbase)),
                )?;
            }
        }
        Ok(())
    }

    /// The bases of every range that overlaps `base..end`, in address order.
    fn overlapping(inner: &Inner, base: u64, end: u64) -> Vec<u64> {
        let mut out = Vec::new();
        if let Some((vbase, vma)) = inner.vmas.range(..base).next_back()
            && *vbase + vma.len > base
        {
            out.push(*vbase);
        }
        out.extend(inner.vmas.range(base..end).map(|(vbase, _)| *vbase));
        out
    }

    /// Where a mapping of `len` bytes goes: downwards from the hint, stepping
    /// over every occupied range.
    ///
    /// A pure function of the map's contents, which is what makes placement
    /// reproducible — nothing here consults the host, a random source, or an
    /// allocator's whim.
    fn free_range(inner: &Inner, len: u64) -> Result<u64> {
        let mut candidate_end = inner.hint;
        loop {
            if candidate_end < inner.floor.saturating_add(len) {
                return Err(config(alloc::format!(
                    "no free range of {len:#x} bytes above {:#x}",
                    inner.floor
                )));
            }
            let base = candidate_end - len;
            // The highest range starting at or below `base` may still overlap,
            // and so may the first one starting inside the window.
            let clash = inner
                .vmas
                .range(..=base)
                .next_back()
                .filter(|(vbase, vma)| **vbase + vma.len > base)
                .map(|(vbase, _)| *vbase)
                .or_else(|| {
                    inner
                        .vmas
                        .range(base..candidate_end)
                        .next()
                        .map(|(vbase, _)| *vbase)
                });
            match clash {
                // Strictly below `candidate_end`, so this terminates.
                Some(vbase) => candidate_end = vbase,
                None => return Ok(base),
            }
        }
    }

    /// Reject a range that is not page aligned or does not fit.
    fn check_range(&self, base: u64, len: u64) -> Result<()> {
        if !base.is_multiple_of(PAGE_SIZE) || !len.is_multiple_of(PAGE_SIZE) {
            return Err(config(alloc::format!(
                "range {base:#x}..+{len:#x} must be aligned to {PAGE_SIZE:#x}"
            )));
        }
        let end = base.checked_add(len).ok_or_else(|| {
            config(alloc::format!(
                "range {base:#x}..+{len:#x} wraps the address space"
            ))
        })?;
        if self.space.bits() < 64 && end > (1u64 << self.space.bits()) {
            return Err(config(alloc::format!(
                "range {base:#x}..{end:#x} does not fit a {}-bit address space",
                self.space.bits()
            )));
        }
        Ok(())
    }

    /// Check that `len` bytes at `addr` exist and permit `want`.
    fn check_prot(&self, addr: u64, len: u64, want: Prot) -> Result<()> {
        if len == 0 {
            return Ok(());
        }
        let end = addr
            .checked_add(len)
            .ok_or(Error::Bus(BusError::Unassigned))?;
        let inner = self.inner.lock();
        let mut cursor = addr;
        while cursor < end {
            let Some((base, vma)) = inner.vmas.range(..=cursor).next_back() else {
                return Err(Error::Bus(BusError::Unassigned));
            };
            let vend = *base + vma.len;
            if vend <= cursor {
                return Err(Error::Bus(BusError::Unassigned));
            }
            if !vma.prot.contains(want) {
                return Err(Error::Bus(BusError::BadAccess));
            }
            cursor = vend;
        }
        Ok(())
    }
}

/// A configuration error naming this module.
fn config(message: String) -> Error {
    Error::Config {
        at: "user memory".to_string(),
        message,
    }
}
