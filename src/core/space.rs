//! Address spaces, memory regions, flat views and dispatch (`ROADMAP.md`
//! §4.1).
//!
//! The single most important abstraction in the crate. Everything else — every
//! CPU core, every device, every bus — reaches memory through here, so the
//! shape of this module is the shape of the emulator.
//!
//! # The model
//!
//! A **region tree** flattened into a **dispatch table**. A tree because that
//! is how real hardware composes: a chipset contains a bridge contains a
//! device, each with its own window. A flat table because a tree walk per
//! access would be ruinous. The tree is what a machine description writes and
//! what a human reasons about; [`FlatView`] is a derived cache
//! (`ROADMAP.md` §15, invariant 3) rebuilt whenever the topology changes.
//!
//! ```text
//!   Region tree                FlatView (sorted, non-overlapping)  Dispatch
//!   ───────────                ──────────────────────────────────  ────────
//!   root                       $0000 +$2000  RAM  wrap $800        page 0: Direct(0)
//!    ├ wram-mirror @$0000 ───► $2000 +$2000  IO   wrap $8          page 1: Direct(0)
//!    ├ ppu-mirror  @$2000      $4000 +$0020  IO                    page 2: Mapped(1)
//!    ├ apu         @$4000      $8000 +$8000  ROM  +bank offset     page 4: SubPage
//!    └ prg-bank    @$8000                                          page 6: Unassigned
//! ```
//!
//! # Two kinds of change, and only one is expensive
//!
//! This distinction is load-bearing, and the API enforces it in the type
//! system:
//!
//! | | Rebase | Retopology |
//! | --- | --- | --- |
//! | Method | [`AddressSpace::rebase`] — takes `&self` | [`AddressSpace::map`] / [`unmap`](AddressSpace::unmap) / [`remap`](AddressSpace::remap) — take `&mut self` |
//! | What changed | an alias's offset slides; the region *set* is identical | regions added, removed, resized, re-prioritized |
//! | Examples | cartridge bank switching | BAR enable/disable, hotplug, ROM shadowing toggle |
//! | Cost | one atomic store per affected flat entry | full flatten + table rebuild |
//! | Generation counter | untouched | bumped |
//!
//! An MMC3 cartridge rebanks ~15 000 times a second. If that rebuilt the flat
//! view and bumped the generation — invalidating every TLB and translation
//! block — the NES would be a slideshow. So a rebase touches the alias's
//! atomic offset cell and the cached offsets of the flat entries that read it,
//! and nothing else: not the entry list, not its ordering, not the dispatch
//! table, not the generation counter.
//!
//! # `unsafe`
//!
//! None. `ROADMAP.md` §0 sanctions "the RAM host-pointer fast path" as one of
//! six places `unsafe` may be re-enabled; this module does not spend it. See
//! [`RamStore`] for what is used instead and what it costs.
//!
//! # What is deliberately not here yet
//!
//! - **The software TLB** (§4.1's last paragraph). It sits *above* this module,
//!   between a CPU and its space, because it is per-CPU state and this is not.
//!   Everything it needs exists: [`AddressSpace::generation`] to invalidate on,
//!   [`AddressSpace::locate`] to fill from, and a write path that records dirty
//!   pages rather than relying on host signals, which wasm does not have.
//! - **Guest-physical / guest-virtual newtypes.** `CLAUDE.md` wants them
//!   distinct, and they will be — but which of the two a given space is
//!   addressed by is a property of the master, not of the space, so the newtypes
//!   belong with the MMU that translates between them. Addresses here are plain
//!   `u64`.
//! - **`core::sync`.** This module needs no lock: the access path is atomics
//!   only, and a topology change takes `&mut self`. When the seam lands, nothing
//!   here has to change.

mod attrs;
mod dispatch;
mod flat;
mod region;
mod store;

#[cfg(test)]
mod tests;

pub use attrs::{AccessConstraints, MemAttrs, MemOps, MemResult, RequesterId};
pub use dispatch::{Dispatch, DispatchEntry, DispatchPolicy};
pub use flat::{EntryKind, FlatEntry, FlatLeaf, FlatTarget, FlatView};
pub use region::{
    Alias, AliasId, CombinePolicy, Container, Mapping, MappingId, Region, RegionKind, RegionRef,
    RomWrite,
};
pub use store::{DEFAULT_PAGE_BITS, RamStore, RomStore};

use crate::core::error::{BusError, Error};
use crate::core::value::{Endian, Width};
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};
use flat::RebaseIndex;

/// What an access to an address with nothing mapped at it does.
///
/// Not a style preference: a 6502 reading unmapped space sees the last value
/// on the bus, an ARM system bus raises an abort, and a PC ISA read returns
/// `0xFF`. Guessing `0xFF` everywhere is exactly the guesswork this type
/// exists to prevent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum UnassignedAction {
    /// Raise [`BusError::Unassigned`], which the CPU turns into an exception.
    #[default]
    Fault,
    /// Reads return all-ones; writes are discarded.
    ReadAsOnes,
    /// Reads return zero; writes are discarded.
    ReadAsZeros,
}

/// The per-space unassigned-access policy: what happens, and whether it is
/// counted.
///
/// `ROADMAP.md` §4.1 lists "fault / read-as-ones / read-as-zeros / log". Log is
/// treated here as orthogonal to the other three rather than a fourth
/// alternative — "read as ones, and tell me about it" is the configuration
/// anyone debugging a machine actually wants, and "log, and then do what?" has
/// no answer otherwise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub struct UnassignedPolicy {
    /// What the access does.
    pub action: UnassignedAction,
    /// Whether to record it in the space's [`UnassignedLog`].
    pub log: bool,
}

impl UnassignedPolicy {
    /// Fault, silently.
    pub const FAULT: UnassignedPolicy = UnassignedPolicy {
        action: UnassignedAction::Fault,
        log: false,
    };
    /// Read as all-ones — an open bus with pull-ups, and the ISA convention.
    pub const ONES: UnassignedPolicy = UnassignedPolicy {
        action: UnassignedAction::ReadAsOnes,
        log: false,
    };
    /// Read as zero.
    pub const ZEROS: UnassignedPolicy = UnassignedPolicy {
        action: UnassignedAction::ReadAsZeros,
        log: false,
    };

    /// The same policy, counted in the space's log.
    #[must_use]
    pub const fn logged(mut self) -> Self {
        self.log = true;
        self
    }
}

/// A running tally of accesses that hit nothing.
///
/// Debug accesses are deliberately excluded: a monitor walking a machine's
/// address space must not move a counter that a person is reading
/// (`ROADMAP.md` §15, invariant 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct UnassignedLog {
    /// How many unassigned accesses have been made.
    pub count: u64,
    /// The address of the most recent one.
    pub last_addr: u64,
    /// Whether the most recent one was a write.
    pub last_was_write: bool,
}

/// One address space: the view of memory that one bus master has.
///
/// Per-master, deliberately. The CPU's view is not the DMA engine's view is not
/// the GPU's view, and a machine whose devices share one global "memory" cannot
/// express an IOMMU, a bridge window, or a cartridge that sees a different bus
/// than the CPU does.
///
/// `Send + Sync`: the read and write paths take `&self` and use only atomics,
/// so several CPU threads may drive the same space at once. Topology changes
/// take `&mut self` and are therefore a stop-the-world operation by
/// construction — which is what the safe-point protocol (`ROADMAP.md` §4.7)
/// already requires them to be.
#[derive(Debug)]
pub struct AddressSpace {
    name: String,
    bits: u32,
    endian: Endian,
    unassigned: UnassignedPolicy,
    combine: CombinePolicy,
    dispatch_policy: DispatchPolicy,
    root: Vec<(MappingId, Mapping)>,
    next_id: u64,
    flat: FlatView,
    dispatch: Option<Dispatch>,
    rebase_index: RebaseIndex,
    generation: AtomicU64,
    unassigned_count: AtomicU64,
    unassigned_last: AtomicU64,
    unassigned_last_write: AtomicU64,
}

impl AddressSpace {
    /// A new, empty space of `bits` address bits.
    ///
    /// # Panics
    ///
    /// If `bits` is 0 or greater than 64.
    #[must_use]
    pub fn new(name: impl Into<String>, bits: u32) -> Self {
        assert!(bits > 0 && bits <= 64, "address width out of range");
        AddressSpace {
            name: name.into(),
            bits,
            endian: Endian::Little,
            unassigned: UnassignedPolicy::FAULT,
            combine: CombinePolicy::Priority,
            dispatch_policy: DispatchPolicy::Flat,
            root: Vec::new(),
            next_id: 1,
            flat: FlatView::default(),
            dispatch: None,
            rebase_index: RebaseIndex::new(),
            generation: AtomicU64::new(1),
            unassigned_count: AtomicU64::new(0),
            unassigned_last: AtomicU64::new(0),
            unassigned_last_write: AtomicU64::new(0),
        }
    }

    /// Set what happens on an access to an unmapped address.
    #[must_use]
    pub fn with_unassigned(mut self, policy: UnassignedPolicy) -> Self {
        self.unassigned = policy;
        self
    }

    /// Set the byte order used to decode an access that hit nothing.
    ///
    /// Mapped accesses always use their own region's byte order; this is only
    /// the fallback.
    #[must_use]
    pub fn with_endian(mut self, endian: Endian) -> Self {
        self.endian = endian;
        self
    }

    /// Set how overlapping root-level mappings are resolved.
    ///
    /// [`CombinePolicy::Priority`] unless the machine is an open-bus system
    /// that genuinely wire-ORs.
    #[must_use]
    pub fn with_combine(mut self, combine: CombinePolicy) -> Self {
        self.combine = combine;
        self
    }

    /// Opt in to (or out of) a dense page-granular dispatch table.
    ///
    /// Opt-in per space because the table is real memory: `ROADMAP.md` §4.1
    /// budgets 16 MiB per space for the low 4 GiB, a dozen masters is 200 MiB,
    /// and on `wasm32` it is a chunk of a 32-bit linear memory. See
    /// [`dispatch`](DispatchPolicy) for what this implementation spends
    /// instead.
    ///
    /// Takes effect on the next retopology; use [`AddressSpace::rebuild`] to
    /// force one.
    #[must_use]
    pub fn with_dispatch(mut self, policy: DispatchPolicy) -> Self {
        self.dispatch_policy = policy;
        self
    }

    /// The space's name, as the machine description gave it.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Address width in bits.
    #[inline]
    #[must_use]
    pub fn bits(&self) -> u32 {
        self.bits
    }

    /// Size of the space in bytes, saturating at [`u64::MAX`] for a 64-bit
    /// space.
    #[inline]
    #[must_use]
    pub fn size(&self) -> u64 {
        if self.bits >= 64 {
            u64::MAX
        } else {
            1u64 << self.bits
        }
    }

    /// The topology generation.
    ///
    /// Every derived cache in the emulator — TLB entries, translation blocks,
    /// direct pointers — records this and throws itself away when it changes
    /// (`ROADMAP.md` §15, invariant 3). A rebase does **not** change it.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Relaxed)
    }

    /// The flattened view. Derived state; valid for the current generation.
    #[inline]
    #[must_use]
    pub fn flat_view(&self) -> &FlatView {
        &self.flat
    }

    /// The dense dispatch table, if this space has one.
    #[inline]
    #[must_use]
    pub fn dispatch(&self) -> Option<&Dispatch> {
        self.dispatch.as_ref()
    }

    /// The root mappings, in mapping order.
    pub fn mappings(&self) -> impl Iterator<Item = (MappingId, &Mapping)> {
        self.root.iter().map(|(id, m)| (*id, m))
    }

    /// Unassigned accesses seen so far, when the policy asks for them to be
    /// logged.
    #[must_use]
    pub fn unassigned_log(&self) -> UnassignedLog {
        UnassignedLog {
            count: self.unassigned_count.load(Ordering::Relaxed),
            last_addr: self.unassigned_last.load(Ordering::Relaxed),
            last_was_write: self.unassigned_last_write.load(Ordering::Relaxed) != 0,
        }
    }

    // -----------------------------------------------------------------
    // Retopology — the expensive kind of change
    // -----------------------------------------------------------------

    /// Map `region` at `base` with priority 0. **Retopology.**
    ///
    /// # Errors
    ///
    /// If the region does not fit in the space, or the tree cannot be
    /// flattened.
    pub fn map(&mut self, region: impl Into<RegionRef>, base: u64) -> Result<MappingId, Error> {
        self.map_with(Mapping::new(region, base))
    }

    /// Map `region` at `base` with an explicit priority, higher winning where
    /// regions overlap. **Retopology.**
    ///
    /// This is how a PCI BAR sits over RAM, how a boot ROM shadows the reset
    /// vector, and how a cartridge mapper puts a window over the open bus.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::map`].
    pub fn map_with_priority(
        &mut self,
        region: impl Into<RegionRef>,
        base: u64,
        priority: i32,
    ) -> Result<MappingId, Error> {
        self.map_with(Mapping::new(region, base).with_priority(priority))
    }

    /// Add a prepared [`Mapping`]. **Retopology.**
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::map`].
    pub fn map_with(&mut self, mapping: Mapping) -> Result<MappingId, Error> {
        self.check_fits(&mapping)?;
        let id = MappingId(self.next_id);
        self.next_id += 1;
        self.root.push((id, mapping));
        match self.rebuild() {
            Ok(()) => Ok(id),
            Err(e) => {
                self.root.pop();
                // Leave the space in the state the caller had before the
                // failed map, rather than half-changed.
                let _ = self.rebuild();
                Err(e)
            }
        }
    }

    /// Remove a mapping. **Retopology.**
    ///
    /// # Errors
    ///
    /// If `id` is not a mapping of this space.
    pub fn unmap(&mut self, id: MappingId) -> Result<(), Error> {
        let Some(pos) = self.root.iter().position(|(i, _)| *i == id) else {
            return Err(Error::Config {
                at: self.name.clone(),
                message: alloc::format!("no mapping {id:?} in this space"),
            });
        };
        self.root.remove(pos);
        self.rebuild()
    }

    /// Move a mapping to a new base address. **Retopology.**
    ///
    /// A PCI BAR address change lands here, and `ROADMAP.md` §4.1 files that
    /// under "rebase". It is not one, and cannot be: moving a mapping changes
    /// the *addresses* in the flat view, not an offset behind an entry, so the
    /// sorted view has to be rebuilt and every cache keyed on the old address
    /// has to be invalidated. The roadmap's cheap case is real, but it is the
    /// one where a fixed aperture's *contents* slide — [`AddressSpace::rebase`].
    ///
    /// # Errors
    ///
    /// If `id` is not a mapping of this space, or the region no longer fits.
    pub fn remap(&mut self, id: MappingId, base: u64) -> Result<(), Error> {
        let Some(pos) = self.root.iter().position(|(i, _)| *i == id) else {
            return Err(Error::Config {
                at: self.name.clone(),
                message: alloc::format!("no mapping {id:?} in this space"),
            });
        };
        let old = self.root[pos].1.base;
        self.root[pos].1.base = base;
        let mapping = self.root[pos].1.clone();
        if let Err(e) = self.check_fits(&mapping) {
            self.root[pos].1.base = old;
            return Err(e);
        }
        self.rebuild()
    }

    /// Reflatten and rebuild the dispatch table, bumping the generation.
    /// **Retopology.**
    ///
    /// Called for you by every topology change; public because a container's
    /// contents can be rebuilt out from under a space and a machine may need
    /// to say so.
    ///
    /// # Errors
    ///
    /// If the region tree cannot be flattened.
    pub fn rebuild(&mut self) -> Result<(), Error> {
        let children: Vec<Mapping> = self.root.iter().map(|(_, m)| m.clone()).collect();
        let (flat, index) = FlatView::build(&children, self.size(), self.combine)?;
        self.dispatch = Dispatch::build(&flat, self.dispatch_policy);
        self.flat = flat;
        self.rebase_index = index;
        // `&mut self`, so this cannot race; `fetch_add` for the ordering, not
        // for the atomicity.
        self.generation.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn check_fits(&self, mapping: &Mapping) -> Result<(), Error> {
        // A 64-bit space reports its size as `u64::MAX`, so a region that ends
        // exactly at 2^64 is rejected. Nothing real is mapped there, and the
        // alternative is an off-by-one in every bounds check downstream.
        let fits = mapping
            .base
            .checked_add(mapping.region.len())
            .is_some_and(|e| e <= self.size());
        if !fits {
            return Err(Error::Config {
                at: self.name.clone(),
                message: alloc::format!(
                    "region `{}` at {:#x} (+{:#x}) does not fit in a {}-bit space",
                    mapping.region.name(),
                    mapping.base,
                    mapping.region.len(),
                    self.bits
                ),
            });
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Rebase — the cheap kind of change
    // -----------------------------------------------------------------

    /// Slide an alias's window to `offset`. **Rebase.**
    ///
    /// Takes `&self`: this is the operation a cartridge mapper performs from
    /// inside its own MMIO write handler, ~15 000 times a second, while the
    /// CPU thread is running. It stores the new offset in the alias's atomic
    /// cell and refreshes the cached offset of every flat entry that reads it
    /// — one relaxed store each. The flat view, the dispatch table, and the
    /// generation counter are untouched, so no TLB and no translation block is
    /// invalidated.
    ///
    /// # Errors
    ///
    /// - If `region` is not an alias.
    /// - If it is not rebasable — its target is a container, so sliding the
    ///   window would change *which* regions appear in it, which is a
    ///   retopology however it is spelled.
    /// - If the window would run off the end of the target. That too changes
    ///   the region set; rebuild instead.
    pub fn rebase(&self, region: &RegionRef, offset: u64) -> Result<(), Error> {
        let Some(alias) = region.as_alias() else {
            return Err(Error::Config {
                at: region.name().to_string(),
                message: "not an alias".to_string(),
            });
        };
        if !alias.is_rebasable() {
            return Err(Error::Config {
                at: region.name().to_string(),
                message: "alias targets a container; sliding it is a retopology".to_string(),
            });
        }
        let end = offset.checked_add(region.len());
        if end.is_none_or(|e| e > alias.target().len()) {
            return Err(Error::Config {
                at: region.name().to_string(),
                message: alloc::format!(
                    "offset {offset:#x} (+{:#x}) runs off the end of `{}`",
                    region.len(),
                    alias.target().name()
                ),
            });
        }
        alias.cell().store(offset, Ordering::Relaxed);
        self.flat.rebase(&self.rebase_index, alias.id());
        Ok(())
    }

    // -----------------------------------------------------------------
    // Access
    // -----------------------------------------------------------------

    /// Locate the flat entry covering `addr`, consulting the dispatch table
    /// first when there is one.
    #[inline]
    #[must_use]
    pub fn locate(&self, addr: u64) -> Option<usize> {
        if let Some(d) = &self.dispatch {
            match d.lookup(addr) {
                Some(DispatchEntry::Unassigned) => return None,
                Some(DispatchEntry::Mapped(i) | DispatchEntry::Direct(i)) => {
                    return Some(i as usize);
                }
                // Sub-page, or above the table's reach: fall through.
                Some(DispatchEntry::SubPage) | None => {}
            }
        }
        self.flat.find(addr)
    }

    /// Read `width` bytes at `addr` and assemble them into a value using the
    /// target region's byte order.
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if nothing is mapped and the policy faults,
    /// [`BusError::BadAccess`] if the region rejects the width or alignment,
    /// [`BusError::Retry`] if the target is busy and nothing has happened yet.
    #[inline]
    pub fn read(&self, addr: u64, width: Width, attrs: MemAttrs) -> MemResult<u64> {
        let n = width.bytes() as usize;
        let mut buf = [0u8; 8];
        let endian = self.read_span(addr, &mut buf[..n], attrs, Some(width))?;
        endian.load(&buf[..n], width)
    }

    /// Write the low `width` bytes of `value` at `addr`, in the target
    /// region's byte order.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    #[inline]
    pub fn write(&self, addr: u64, width: Width, value: u64, attrs: MemAttrs) -> MemResult {
        let n = width.bytes() as usize;
        let mut buf = [0u8; 8];
        // Byte order has to be known before the bytes exist, so the target is
        // located twice for a write. The second lookup is a dispatch-table
        // index or a binary search, not a tree walk.
        self.endian_at(addr).store(&mut buf[..n], width, value)?;
        self.write_span(addr, &buf[..n], attrs, Some(width))
    }

    /// Read raw bytes in ascending address order — a DMA burst, a debugger
    /// dump, a snapshot.
    ///
    /// No byte-order conversion is applied: these are bytes, not a value.
    /// Regions that do not accept bulk transfers reject this
    /// ([`AccessConstraints::allow_bulk`]).
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    pub fn read_bytes(&self, addr: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.read_span(addr, dst, attrs, None).map(|_| ())
    }

    /// Write raw bytes in ascending address order.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    pub fn write_bytes(&self, addr: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.write_span(addr, src, attrs, None)
    }

    /// The byte order a value-typed access at `addr` would use.
    #[inline]
    #[must_use]
    pub fn endian_at(&self, addr: u64) -> Endian {
        self.locate(addr)
            .and_then(|i| self.flat.entry(i))
            .map_or(self.endian, FlatEntry::endian)
    }

    /// The distance from `addr` to the next mapped byte, capped at `max`.
    fn gap_len(&self, addr: u64, max: u64) -> u64 {
        let entries = self.flat.entries();
        let i = entries.partition_point(|e| e.start() <= addr);
        match entries.get(i) {
            Some(e) => (e.start() - addr).min(max),
            None => max,
        }
    }

    fn read_span(
        &self,
        addr: u64,
        dst: &mut [u8],
        attrs: MemAttrs,
        width: Option<Width>,
    ) -> MemResult<Endian> {
        let total = dst.len() as u64;
        if total == 0 {
            return Ok(self.endian);
        }
        addr.checked_add(total - 1).ok_or(BusError::BadAccess)?;
        let mut endian = None;
        let mut done = 0u64;
        let mut committed = false;
        while done < total {
            let a = addr + done;
            let remaining = total - done;
            let (n, res) = match self.locate(a) {
                Some(i) => {
                    let e = self.flat.entry(i).expect("index came from locate");
                    let rel = a - e.start();
                    let n = e.run_len(rel).min(remaining);
                    if endian.is_none() {
                        endian = Some(e.endian());
                    }
                    let piece = &mut dst[usize_of(done)..usize_of(done + n)];
                    let w = if n == total { width } else { None };
                    (n, e.read(rel, piece, attrs, w))
                }
                None => {
                    let n = self.gap_len(a, remaining);
                    let piece = &mut dst[usize_of(done)..usize_of(done + n)];
                    (n, self.unassigned_read(a, piece, attrs))
                }
            };
            // `Retry` is only legal before any side effect or partial
            // transfer. Re-running a half-completed multi-region access would
            // read a FIFO twice; the dispatcher refuses instead.
            match res {
                Err(BusError::Retry) if committed => return Err(BusError::BadAccess),
                Err(e) => return Err(e),
                Ok(()) => {}
            }
            committed = true;
            done += n;
        }
        Ok(endian.unwrap_or(self.endian))
    }

    fn write_span(
        &self,
        addr: u64,
        src: &[u8],
        attrs: MemAttrs,
        width: Option<Width>,
    ) -> MemResult {
        let total = src.len() as u64;
        if total == 0 {
            return Ok(());
        }
        addr.checked_add(total - 1).ok_or(BusError::BadAccess)?;
        let mut done = 0u64;
        let mut committed = false;
        while done < total {
            let a = addr + done;
            let remaining = total - done;
            let (n, res) = match self.locate(a) {
                Some(i) => {
                    let e = self.flat.entry(i).expect("index came from locate");
                    let rel = a - e.start();
                    let n = e.run_len(rel).min(remaining);
                    let piece = &src[usize_of(done)..usize_of(done + n)];
                    let w = if n == total { width } else { None };
                    (n, e.write(rel, piece, attrs, w))
                }
                None => {
                    let n = self.gap_len(a, remaining);
                    (n, self.unassigned_write(a, attrs))
                }
            };
            match res {
                Err(BusError::Retry) if committed => return Err(BusError::BadAccess),
                Err(e) => return Err(e),
                Ok(()) => {}
            }
            committed = true;
            done += n;
        }
        Ok(())
    }

    fn note_unassigned(&self, addr: u64, is_write: bool, attrs: MemAttrs) {
        // A debug access must not move a counter someone is watching.
        if !self.unassigned.log || attrs.debug {
            return;
        }
        self.unassigned_count.fetch_add(1, Ordering::Relaxed);
        self.unassigned_last.store(addr, Ordering::Relaxed);
        self.unassigned_last_write
            .store(u64::from(is_write), Ordering::Relaxed);
    }

    fn unassigned_read(&self, addr: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.note_unassigned(addr, false, attrs);
        match self.unassigned.action {
            UnassignedAction::Fault => Err(BusError::Unassigned),
            UnassignedAction::ReadAsOnes => {
                dst.fill(0xff);
                Ok(())
            }
            UnassignedAction::ReadAsZeros => {
                dst.fill(0x00);
                Ok(())
            }
        }
    }

    fn unassigned_write(&self, addr: u64, attrs: MemAttrs) -> MemResult {
        self.note_unassigned(addr, true, attrs);
        match self.unassigned.action {
            UnassignedAction::Fault => Err(BusError::Unassigned),
            UnassignedAction::ReadAsOnes | UnassignedAction::ReadAsZeros => Ok(()),
        }
    }
}

/// Narrow a transfer offset to a host index.
///
/// Offsets inside a single access are bounded by the buffer the caller
/// supplied, which is already a host slice, so this cannot truncate.
#[inline]
fn usize_of(v: u64) -> usize {
    v as usize
}
