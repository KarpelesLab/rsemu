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
//! | Method | [`AddressSpace::rebase`] — a **read** guard | [`AddressSpace::topology`] then [`map`](TopologyGuard::map) / [`unmap`](TopologyGuard::unmap) / [`remap`](TopologyGuard::remap) — a **write** guard |
//! | What changed | an alias's offset slides; the region *set* is identical | regions added, removed, resized, re-prioritized |
//! | Examples | cartridge bank switching | BAR enable/disable, hotplug, ROM shadowing toggle |
//! | Cost | one atomic store per affected flat entry | full flatten + table rebuild, **once per guard** |
//! | Generation counter | untouched | bumped |
//!
//! A retopology's flatten is deferred to the moment the guard closes, so a
//! batch of mappings costs one rebuild rather than one each — see
//! [`TopologyGuard`] for the board that made that the difference between 354 ms
//! and 2 ms.
//!
//! An MMC3 cartridge rebanks ~15 000 times a second. If that rebuilt the flat
//! view and bumped the generation — invalidating every TLB and translation
//! block — the NES would be a slideshow. So a rebase touches the alias's
//! atomic offset cell and the cached offsets of the flat entries that read it,
//! and nothing else: not the entry list, not its ordering, not the dispatch
//! table, not the generation counter.
//!
//! # The mapping layer: what answers, and on what terms
//!
//! A region list says *what answers an address*. A [`Mapping`] says *under what
//! terms* — [`Perms`], carried on the placement rather than on the region,
//! because a ROM chip is a ROM chip and whether this bus may write to it is a
//! property of the decode in front of it. Permissions intersect down the tree,
//! so a child of a read-only container is read-only however it was mapped.
//!
//! Three things that look like separate features are this one mechanism:
//!
//! * **A mapping that faults.** A write to a mapping without [`Perms::WRITE`]
//!   raises [`BusError::Protected`] — distinct from [`BusError::BadAccess`],
//!   which is the difference between "you cannot do that here" and "you cannot
//!   do that at all".
//! * **Copy-on-write.** Two spaces map one store, both without
//!   [`Perms::WRITE`]. The first store faults; whoever is driving the machine
//!   gives that side a private copy, [`replace`](TopologyGuard::replace)s the
//!   mapping, and reissues. Nothing here knows what a process is —
//!   [`usermode`](crate::usermode) builds `fork` out of exactly this.
//! * **One address, two chips.** The flattener resolves reads and writes
//!   **separately**: the highest-priority mapping that permits reads need not
//!   be the one that permits writes. A Master System's slot 2 reads a ROM bank
//!   and writes the on-cartridge RAM `$FFFC` bit 3 switched in — two
//!   overlapping mappings with complementary permissions, and no new region
//!   kind. [`Region::split`] remains the terser spelling for the narrower case
//!   where the two halves are two [`MemOps`] at one address rather than two
//!   stores.
//!
//! With every mapping [`Perms::RWX`] — every machine that has never mentioned
//! permission — both directions resolve to the same winner, and every entry is
//! the shape it always was.
//!
//! **The fault does not resolve itself.** It cannot: the access holds the
//! space's read guard and resolving means a retopology, which is the lock
//! inversion this module's ladder exists to forbid. So a permission fault
//! leaves the space, exactly as a page fault leaves a CPU, and comes back as a
//! reissued access once whoever handled it is somewhere a
//! [`TopologyGuard`] may be taken.
//!
//! # Why a guard, and not `&mut self`
//!
//! Topology used to take `&mut self`, which made the distinction above
//! borrow-checker enforced for free. It also made the design unusable: a PCI
//! BAR write remaps memory *from inside the device's own MMIO write handler*
//! (`ROADMAP.md` §4.7), device methods take `&self`, and once a machine has
//! wrapped a space in an `Arc` and handed clones to its bus masters (§4.4's
//! `Initiator`) nothing can ever borrow it mutably again. Hot-plug and BAR
//! moves were impossible by construction.
//!
//! The mutable half of a space therefore lives behind one [`RwLock`] at
//! [`LockRank::TOPOLOGY`], and the rebase/retopology distinction stays in the
//! type system as *which guard* the caller has to be holding:
//!
//! - [`AddressSpace::rebase`] and every access take a **read** guard
//!   ([`SpaceView`]), so a cheap window slide still provably cannot touch
//!   topology — a `SpaceView` has no method that can.
//! - [`AddressSpace::topology`] takes the **write** guard
//!   ([`TopologyGuard`]) and is the only route to
//!   [`map`](TopologyGuard::map), [`unmap`](TopologyGuard::unmap),
//!   [`remap`](TopologyGuard::remap) and [`rebuild`](TopologyGuard::rebuild).
//!   One acquisition covers a whole batch of maps.
//!
//! `map` is reachable only by naming `topology()`, so a retopology is as
//! explicit and as greppable as `&mut self` was — and callable from `&self`.
//!
//! # A remap from a write handler goes through `Deferred`
//!
//! [`LockRank::TOPOLOGY`] sits **above** [`LockRank::BUS`] and
//! [`LockRank::DEVICE`], because the ladder runs in the direction calls travel:
//! a retopology may call down into buses and devices, never the reverse. A CPU
//! holds a `BUS`-ranked lock across the accesses it issues, and the access path
//! itself holds this space's topology lock for reading — so a handler that
//! reached back for the *write* guard would invert the ladder and, on a
//! threaded backend, deadlock against a concurrent retopology that is waiting
//! for that same `BUS` lock.
//!
//! That is not left to good intentions. Acquiring the write guard while an
//! access is in flight is a lock-order violation, and
//! [`LockRank::enter`](crate::core::sync::LockRank::enter) panics on it in
//! debug builds naming both ranks — whether or not a CPU's `BUS` lock is also
//! held, because the access path's own read guard is already recorded at
//! `TOPOLOGY` and `TOPOLOGY <= TOPOLOGY` fails the strictly-increasing rule.
//! Under the `single` backend the lock itself catches the re-entry in release
//! builds too.
//!
//! The supported spelling is [`Deferred`](crate::core::device::Deferred): the
//! handler pushes the remap and returns, its critical section is released, and
//! whoever drove the access drains the queue.
//!
//! ```
//! use std::sync::Arc;
//!
//! use rsemu::core::device::Deferred;
//! use rsemu::core::space::{AddressSpace, MappingId};
//!
//! // Inside a BAR write handler: queue the remap, do not perform it.
//! fn bar_written(space: &Arc<AddressSpace>, bar: MappingId, base: u64, q: &mut Deferred) {
//!     let space = Arc::clone(space);
//!     q.push(move || {
//!         // Runs after the handler returned and the access released its
//!         // read guard, so `TOPOLOGY` is the outermost lock again.
//!         let _ = space.topology().remap(bar, base);
//!     });
//! }
//! ```
//!
//! Two *different* spaces cannot have their topology guards open at once
//! either: the same rank twice is a violation, so a cross-space retopology is
//! two sequential guards rather than one atomic step. That is a real
//! limitation and it is deliberate — the alternative is a rank per space, which
//! is no ladder at all.
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
//! - **A retopology that reaches for the safe point on its own.** The protocol
//!   itself now exists — [`Scheduler::stop_the_world`] is §4.7's generation
//!   counter and per-CPU exit flag — but nothing in this module calls it, so a
//!   retopology racing a running CPU still costs that CPU a
//!   [`BusError::Retry`] rather than a stall. The lock is what makes that
//!   *correct*; stopping the world first is what would make it free. A caller
//!   that cares can already take the guard around its own remap, which is what
//!   a snapshot does.
//!
//! [`Scheduler::stop_the_world`]: crate::core::sched::Scheduler::stop_the_world
//!
//! [`LockRank::TOPOLOGY`]: crate::core::sync::LockRank::TOPOLOGY
//! [`LockRank::BUS`]: crate::core::sync::LockRank::BUS
//! [`LockRank::DEVICE`]: crate::core::sync::LockRank::DEVICE

mod attrs;
mod dispatch;
mod flat;
mod region;
mod store;

#[cfg(test)]
mod tests;

pub use attrs::{AccessConstraints, MemAttrs, MemOps, MemResult, Perms, RequesterId};
pub use dispatch::{Dispatch, DispatchEntry, DispatchPolicy};
pub use flat::{EntryKind, FlatEntry, FlatLeaf, FlatTarget, FlatView};
pub use region::{
    Alias, AliasId, CombinePolicy, Container, Mapping, MappingId, Region, RegionKind, RegionRef,
    RomWrite,
};
pub use store::{DEFAULT_PAGE_BITS, HOST_PAGE, RamStore, RomStore};

use crate::core::error::{BusError, Error};
use crate::core::sync::{LockRank, RwLock, RwLockReadGuard, RwLockWriteGuard};
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
    /// Reads return the last byte the *master* drove
    /// ([`MemAttrs::bus`](crate::core::space::MemAttrs::bus)); writes are
    /// discarded.
    ///
    /// What a board with no bus-error line and no pull-ups actually does. On a
    /// NES this is load-bearing rather than cosmetic: software reads `$4000`
    /// expecting `$40` back, and a DMC DMA that steals a cycle changes the
    /// answer.
    OpenBus,
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
    /// Read back whatever the master last drove — a bus with nothing on it.
    pub const OPEN_BUS: UnassignedPolicy = UnassignedPolicy {
        action: UnassignedAction::OpenBus,
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

/// Everything about a space that a retopology changes, under one lock.
///
/// One struct so that one acquisition covers all of it: a flat view that
/// disagreed with the mapping list it was built from, or a dispatch table that
/// indexed a flat view it had never seen, is precisely the derived-cache bug
/// invariant 3 exists to prevent.
#[derive(Debug)]
struct Topology {
    root: Vec<(MappingId, Mapping)>,
    next_id: u64,
    flat: FlatView,
    dispatch: Option<Dispatch>,
    rebase_index: RebaseIndex,
    /// Set when `root` has changed and the derived state has not caught up.
    ///
    /// Only ever true *while a [`TopologyGuard`] is open*, which is the whole
    /// point: the guard excludes every reader, so nothing can observe the
    /// disagreement, and the flatten happens once when the guard closes rather
    /// than once per mapping. See [`TopologyGuard`] for why that matters.
    dirty: bool,
}

/// One address space: the view of memory that one bus master has.
///
/// Per-master, deliberately. The CPU's view is not the DMA engine's view is not
/// the GPU's view, and a machine whose devices share one global "memory" cannot
/// express an IOMMU, a bridge window, or a cartridge that sees a different bus
/// than the CPU does.
///
/// `Send + Sync`, and *every* method takes `&self`: a space is meant to be
/// shared through an `Arc` between the CPUs that execute from it and the
/// devices that initiate accesses on it, and to stay retopologisable
/// afterwards. Topology lives behind an [`RwLock`] at
/// [`LockRank::TOPOLOGY`](crate::core::sync::LockRank::TOPOLOGY) — see the
/// [module docs](self) for which guard does what, and why a remap from inside
/// an MMIO handler has to be deferred.
#[derive(Debug)]
pub struct AddressSpace {
    // Immutable after construction: set only by the `with_*` builders, which
    // take `self` by value, so no lock guards them and the access path can read
    // them without one.
    name: String,
    bits: u32,
    endian: Endian,
    unassigned: UnassignedPolicy,
    combine: CombinePolicy,
    dispatch_policy: DispatchPolicy,
    topo: RwLock<Topology>,
    // Outside the lock on purpose: a derived cache validating itself wants the
    // generation *without* acquiring anything, which is the whole point of
    // having a generation counter.
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
            topo: RwLock::with_rank(
                LockRank::TOPOLOGY,
                Topology {
                    root: Vec::new(),
                    next_id: 1,
                    flat: FlatView::default(),
                    dispatch: None,
                    rebase_index: RebaseIndex::new(),
                    dirty: false,
                },
            ),
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
    /// Takes effect on the next retopology; use [`TopologyGuard::rebuild`] to
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
    ///
    /// Lock-free on purpose: a cache checking whether it is still valid must
    /// not queue behind the retopology that invalidated it.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Relaxed)
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
    // The two guards
    // -----------------------------------------------------------------

    /// Open the space for **retopology**: the only route to
    /// [`map`](TopologyGuard::map), [`map_with`](TopologyGuard::map_with),
    /// [`map_with_priority`](TopologyGuard::map_with_priority),
    /// [`unmap`](TopologyGuard::unmap), [`remap`](TopologyGuard::remap) and
    /// [`rebuild`](TopologyGuard::rebuild).
    ///
    /// Takes the write guard, so a batch of maps costs one acquisition and is
    /// atomic with respect to every reader.
    ///
    /// # Where this may be called from
    ///
    /// Machine assembly, a safe point, or a
    /// [`Deferred`](crate::core::device::Deferred) action — anywhere no access
    /// is in flight. **Not** from inside an MMIO handler: the access that
    /// reached that handler holds this same lock for reading and a CPU holds a
    /// `BUS`-ranked lock above it, so acquiring `TOPOLOGY` there inverts the
    /// ladder. See the [module docs](self) for the deferred spelling.
    ///
    /// # Panics
    ///
    /// In debug builds, if the lock order would be violated — which covers the
    /// inline-remap-from-a-handler mistake, and also a second space's topology
    /// guard opened while this one is held. Under the `single` backend the
    /// lock catches a re-entrant acquisition in release builds too.
    #[must_use]
    pub fn topology(&self) -> TopologyGuard<'_> {
        TopologyGuard {
            space: self,
            topo: self.topo.write(),
        }
    }

    /// [`topology`](AddressSpace::topology), but `None` rather than blocking
    /// when the space is in use.
    ///
    /// For a caller with somewhere else to be — a monitor offering to hot-plug,
    /// a test asserting that a retopology is impossible right now. A failed
    /// try-lock cannot join a deadlock cycle, so this is order-exempt and never
    /// trips the rank check.
    #[must_use]
    pub fn try_topology(&self) -> Option<TopologyGuard<'_>> {
        Some(TopologyGuard {
            space: self,
            topo: self.topo.try_write()?,
        })
    }

    /// Open the space for reading: accesses, lookups, the flattened view.
    ///
    /// Hold one across a burst — a DMA transfer, a debugger dump, a
    /// disassembly — and every access in it sees one consistent topology
    /// instead of re-acquiring per byte and possibly straddling a remap.
    ///
    /// # Panics
    ///
    /// If a retopology holds the space. The read side is acquired
    /// *non-blocking* on purpose: a reader that waited here while holding a
    /// `BUS`-ranked lock would be the other half of the deadlock the ladder
    /// exists to prevent, since a retopology is allowed to take `BUS` locks
    /// underneath `TOPOLOGY`. Access-path callers use [`AddressSpace::read`]
    /// and friends, which turn the same condition into [`BusError::Retry`];
    /// this method is for setup, tests and monitors, where a concurrent
    /// retopology means someone skipped the safe point.
    #[must_use]
    pub fn view(&self) -> SpaceView<'_> {
        self.try_view()
            .expect("address space is being retopologised; use `read`/`try_view` on an access path")
    }

    /// [`view`](AddressSpace::view), but `None` rather than panicking while a
    /// retopology holds the space.
    #[must_use]
    pub fn try_view(&self) -> Option<SpaceView<'_>> {
        Some(SpaceView {
            space: self,
            topo: self.topo.try_read()?,
        })
    }

    // -----------------------------------------------------------------
    // Rebase — the cheap kind of change
    // -----------------------------------------------------------------

    /// Slide an alias's window to `offset`. **Rebase.**
    ///
    /// Takes a *read* guard: this is the operation a cartridge mapper performs
    /// from inside its own MMIO write handler, ~15 000 times a second, while
    /// the CPU thread is running. It stores the new offset in the alias's
    /// atomic cell and refreshes the cached offset of every flat entry that
    /// reads it — one relaxed store each. The flat view, the dispatch table and
    /// the generation counter are untouched, so no TLB and no translation block
    /// is invalidated, and no retopology can be smuggled in this way.
    ///
    /// # Errors
    ///
    /// - If `region` is not an alias.
    /// - If it is not rebasable — its target is a container, so sliding the
    ///   window would change *which* regions appear in it, which is a
    ///   retopology however it is spelled.
    /// - If the window would run off the end of the target. That too changes
    ///   the region set; rebuild instead.
    /// - [`BusError::Retry`] if a retopology holds the space, in which case the
    ///   window is about to be rebuilt anyway.
    pub fn rebase(&self, region: &RegionRef, offset: u64) -> Result<(), Error> {
        self.try_view()
            .ok_or(Error::Bus(BusError::Retry))?
            .rebase(region, offset)
    }

    // -----------------------------------------------------------------
    // Access
    // -----------------------------------------------------------------

    /// Locate the flat entry covering `addr`, consulting the dispatch table
    /// first when there is one.
    ///
    /// The index is into [`SpaceView::flat_view`] and means nothing outside the
    /// generation it was obtained in.
    ///
    /// # Panics
    ///
    /// As [`AddressSpace::view`].
    #[inline]
    #[must_use]
    pub fn locate(&self, addr: u64) -> Option<usize> {
        self.view().locate(addr)
    }

    /// The byte order a value-typed access at `addr` would use.
    ///
    /// # Panics
    ///
    /// As [`AddressSpace::view`].
    #[inline]
    #[must_use]
    pub fn endian_at(&self, addr: u64) -> Endian {
        self.view().endian_at(addr)
    }

    /// Read `width` bytes at `addr` and assemble them into a value using the
    /// target region's byte order.
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if nothing is mapped and the policy faults,
    /// [`BusError::BadAccess`] if the region rejects the width or alignment,
    /// [`BusError::Retry`] if the target is busy — or if a retopology holds the
    /// space, in which case nothing has happened yet and the access may be
    /// reissued.
    #[inline]
    pub fn read(&self, addr: u64, width: Width, attrs: MemAttrs) -> MemResult<u64> {
        self.try_view()
            .ok_or(BusError::Retry)?
            .read(addr, width, attrs)
    }

    /// Read, and say whether anything on the far side of the master's pins
    /// drove the data bus.
    ///
    /// `false` means the value came from an unmapped address or from a
    /// register on the master's own die, and a master that models an open-bus
    /// latch must leave that latch alone. Everything else can use
    /// [`AddressSpace::read`] and ignore the question.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    #[inline]
    pub fn read_driven(&self, addr: u64, width: Width, attrs: MemAttrs) -> MemResult<(u64, bool)> {
        self.try_view()
            .ok_or(BusError::Retry)?
            .read_driven(addr, width, attrs)
    }

    /// Write the low `width` bytes of `value` at `addr`, in the target
    /// region's byte order.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    #[inline]
    pub fn write(&self, addr: u64, width: Width, value: u64, attrs: MemAttrs) -> MemResult {
        self.try_view()
            .ok_or(BusError::Retry)?
            .write(addr, width, value, attrs)
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
        self.try_view()
            .ok_or(BusError::Retry)?
            .read_bytes(addr, dst, attrs)
    }

    /// Write raw bytes in ascending address order.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`].
    pub fn write_bytes(&self, addr: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.try_view()
            .ok_or(BusError::Retry)?
            .write_bytes(addr, src, attrs)
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
            UnassignedAction::OpenBus => {
                // Every byte of a wide read floats the same way: the master
                // drove one byte last and there is nothing else on the wires.
                dst.fill(attrs.bus);
                Ok(())
            }
        }
    }

    fn unassigned_write(&self, addr: u64, attrs: MemAttrs) -> MemResult {
        self.note_unassigned(addr, true, attrs);
        match self.unassigned.action {
            UnassignedAction::Fault => Err(BusError::Unassigned),
            UnassignedAction::ReadAsOnes
            | UnassignedAction::ReadAsZeros
            | UnassignedAction::OpenBus => Ok(()),
        }
    }

    /// Everything about a mapping that has to be true before it may be added.
    ///
    /// Two things, and the second is here rather than in the flattener for a
    /// structural reason: [`TopologyGuard`] defers its flatten to the end of a
    /// batch, so by the time the tree is walked there is no caller left to
    /// return an error to. Depth is the only way flattening can fail, and
    /// [`Region::depth`] is maintained at construction, so the check is O(1)
    /// and lands where the mistake was made.
    ///
    /// Reads only immutable fields, so it is callable with either guard held.
    fn check_mapping(&self, mapping: &Mapping) -> Result<(), Error> {
        self.check_fits(mapping)?;
        if mapping.region.depth() > flat::MAX_DEPTH {
            return Err(Error::Config {
                at: self.name.clone(),
                message: alloc::format!(
                    "region `{}` nests {} deep and the limit is {}",
                    mapping.region.name(),
                    mapping.region.depth(),
                    flat::MAX_DEPTH
                ),
            });
        }
        Ok(())
    }

    /// Whether `mapping` lies inside the space.
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
}

// ---------------------------------------------------------------------------
// The read guard
// ---------------------------------------------------------------------------

/// A consistent view of one [`AddressSpace`]: accesses, lookups, the flat view,
/// and rebase.
///
/// Holds the space's topology lock for **reading**, which is what keeps the
/// rebase/retopology distinction a type-level fact rather than a convention. A
/// `SpaceView` can slide an alias window ([`rebase`](SpaceView::rebase)) and
/// touch every byte in the space, and it has no method that can add, remove or
/// move a region — for that a caller must name [`AddressSpace::topology`].
///
/// Not `Send`: a lock guard belongs to the thread that took it.
#[derive(Debug)]
pub struct SpaceView<'a> {
    space: &'a AddressSpace,
    topo: RwLockReadGuard<'a, Topology>,
}

impl SpaceView<'_> {
    /// The space this view is of.
    #[inline]
    #[must_use]
    pub fn space(&self) -> &AddressSpace {
        self.space
    }

    /// The flattened view. Derived state; valid for the current generation.
    #[inline]
    #[must_use]
    pub fn flat_view(&self) -> &FlatView {
        &self.topo.flat
    }

    /// The dense dispatch table, if this space has one.
    #[inline]
    #[must_use]
    pub fn dispatch(&self) -> Option<&Dispatch> {
        self.topo.dispatch.as_ref()
    }

    /// The root mappings, in mapping order.
    pub fn mappings(&self) -> impl Iterator<Item = (MappingId, &Mapping)> {
        self.topo.root.iter().map(|(id, m)| (*id, m))
    }

    /// Locate the flat entry covering `addr`, consulting the dispatch table
    /// first when there is one.
    #[inline]
    #[must_use]
    pub fn locate(&self, addr: u64) -> Option<usize> {
        if let Some(d) = &self.topo.dispatch {
            match d.lookup(addr) {
                Some(DispatchEntry::Unassigned) => return None,
                Some(DispatchEntry::Mapped(i) | DispatchEntry::Direct(i)) => {
                    return Some(i as usize);
                }
                // Sub-page, or above the table's reach: fall through.
                Some(DispatchEntry::SubPage) | None => {}
            }
        }
        self.topo.flat.find(addr)
    }

    /// The byte order a value-typed access at `addr` would use.
    #[inline]
    #[must_use]
    pub fn endian_at(&self, addr: u64) -> Endian {
        self.locate(addr)
            .and_then(|i| self.topo.flat.entry(i))
            .map_or(self.space.endian, FlatEntry::endian)
    }

    /// Slide an alias's window to `offset`. **Rebase.**
    ///
    /// [`AddressSpace::rebase`] is this with the guard taken for you; use this
    /// form when a mapper rebanks several windows at once.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::rebase`], less the `Retry` case: this view already
    /// holds the lock.
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
        self.topo.flat.rebase(&self.topo.rebase_index, alias.id());
        Ok(())
    }

    /// Read `width` bytes at `addr` and assemble them into a value using the
    /// target region's byte order.
    ///
    /// # Errors
    ///
    /// As [`AddressSpace::read`], less the retopology `Retry` case.
    #[inline]
    pub fn read(&self, addr: u64, width: Width, attrs: MemAttrs) -> MemResult<u64> {
        let n = width.bytes() as usize;
        let mut buf = [0u8; 8];
        let endian = self.read_span(addr, &mut buf[..n], attrs, Some(width))?;
        endian.load(&buf[..n], width)
    }

    /// Read, and say whether anything actually drove the master's data bus.
    ///
    /// # Errors
    ///
    /// As [`SpaceView::read`].
    pub fn read_driven(&self, addr: u64, width: Width, attrs: MemAttrs) -> MemResult<(u64, bool)> {
        let n = width.bytes() as usize;
        let mut buf = [0u8; 8];
        let mut driven = true;
        let endian = self.read_span_driven(addr, &mut buf[..n], attrs, Some(width), &mut driven)?;
        Ok((endian.load(&buf[..n], width)?, driven))
    }

    /// Write the low `width` bytes of `value` at `addr`, in the target
    /// region's byte order.
    ///
    /// # Errors
    ///
    /// As [`SpaceView::read`].
    #[inline]
    pub fn write(&self, addr: u64, width: Width, value: u64, attrs: MemAttrs) -> MemResult {
        let n = width.bytes() as usize;
        let mut buf = [0u8; 8];
        // Byte order has to be known before the bytes exist, so the target is
        // located twice for a write. The second lookup is a dispatch-table
        // index or a binary search, not a tree walk — and both happen under
        // this one guard, so they cannot disagree.
        self.endian_at(addr).store(&mut buf[..n], width, value)?;
        self.write_span(addr, &buf[..n], attrs, Some(width))
    }

    /// Read raw bytes in ascending address order.
    ///
    /// # Errors
    ///
    /// As [`SpaceView::read`].
    pub fn read_bytes(&self, addr: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.read_span(addr, dst, attrs, None).map(|_| ())
    }

    /// Write raw bytes in ascending address order.
    ///
    /// # Errors
    ///
    /// As [`SpaceView::read`].
    pub fn write_bytes(&self, addr: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.write_span(addr, src, attrs, None)
    }

    /// The distance from `addr` to the next mapped byte, capped at `max`.
    fn gap_len(&self, addr: u64, max: u64) -> u64 {
        let entries = self.topo.flat.entries();
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
        let mut driven = true;
        self.read_span_driven(addr, dst, attrs, width, &mut driven)
    }

    /// [`SpaceView::read_span`], also reporting whether anything on the far
    /// side of the master's pins drove the data bus.
    fn read_span_driven(
        &self,
        addr: u64,
        dst: &mut [u8],
        attrs: MemAttrs,
        width: Option<Width>,
        driven: &mut bool,
    ) -> MemResult<Endian> {
        let total = dst.len() as u64;
        if total == 0 {
            return Ok(self.space.endian);
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
                    let e = self.topo.flat.entry(i).expect("index came from locate");
                    let rel = a - e.start();
                    let n = e.read_run_len(rel).min(remaining);
                    if endian.is_none() {
                        endian = Some(e.endian());
                    }
                    let piece = &mut dst[usize_of(done)..usize_of(done + n)];
                    let w = if n == total { width } else { None };
                    *driven &= e.drives_data_bus();
                    (n, e.read(rel, piece, attrs, w))
                }
                None => {
                    let n = self.gap_len(a, remaining);
                    let piece = &mut dst[usize_of(done)..usize_of(done + n)];
                    // Nothing answered, so nothing drove the wires either: the
                    // byte the master reads back is the byte it left there.
                    *driven = false;
                    (n, self.space.unassigned_read(a, piece, attrs))
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
        Ok(endian.unwrap_or(self.space.endian))
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
                    let e = self.topo.flat.entry(i).expect("index came from locate");
                    let rel = a - e.start();
                    let n = e.write_run_len(rel).min(remaining);
                    let piece = &src[usize_of(done)..usize_of(done + n)];
                    let w = if n == total { width } else { None };
                    (n, e.write(rel, piece, attrs, w))
                }
                None => {
                    let n = self.gap_len(a, remaining);
                    (n, self.space.unassigned_write(a, attrs))
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
}

// ---------------------------------------------------------------------------
// The write guard
// ---------------------------------------------------------------------------

/// Exclusive access to one [`AddressSpace`]'s topology: the only way to change
/// its region set.
///
/// Obtained from [`AddressSpace::topology`], which is where the rules about
/// when it may be taken are written down. Holding it excludes every reader —
/// every access, on every thread — so keep it short, and keep it out of MMIO
/// handlers, which are readers by definition.
///
/// Every mutating method here is a **retopology**: between them they reflatten
/// the tree, rebuild the dispatch table and bump the generation, invalidating
/// every derived cache in the machine.
///
/// # One rebuild per guard, not one per call
///
/// The flatten is **deferred to the moment the guard closes**. A `map` marks
/// the topology dirty and returns; the tree is walked once, when the last
/// mutation is in.
///
/// That is not a micro-optimisation. An incompletely decoded bus has to spell
/// its missing decode out as a mapping per repeat unit — a Master System's
/// A8-A15 go nowhere, so the same four chips answer at 256 different 16-bit
/// port addresses and the machine file says so 1280 times. Rebuilding per call
/// made realizing that board quadratic: 354 ms in release and 3 s in debug, for
/// a board with five chips on it. Deferring makes it linear in the batch.
///
/// The consequence a caller sees is that [`flat_view`](TopologyGuard::flat_view)
/// and [`dispatch`](TopologyGuard::dispatch) take `&mut self`: reading derived
/// state through an open guard has to catch it up first. Everything outside
/// this module reads them through a [`SpaceView`] instead, where they are
/// always current — the guard is closed by then.
///
/// A deferred flatten cannot fail, because the only way flattening *can* fail
/// is an over-deep region tree and [`map`](TopologyGuard::map) rejects one
/// eagerly. See [`Region::depth`].
#[derive(Debug)]
pub struct TopologyGuard<'a> {
    space: &'a AddressSpace,
    topo: RwLockWriteGuard<'a, Topology>,
}

impl Drop for TopologyGuard<'_> {
    fn drop(&mut self) {
        self.sync();
    }
}

impl TopologyGuard<'_> {
    /// The space this guard is open on.
    #[inline]
    #[must_use]
    pub fn space(&self) -> &AddressSpace {
        self.space
    }

    /// The flattened view as it stands. Derived state.
    ///
    /// Takes `&mut self` because it may have to perform the deferred flatten
    /// first — see the type's docs.
    #[inline]
    pub fn flat_view(&mut self) -> &FlatView {
        self.sync();
        &self.topo.flat
    }

    /// The dense dispatch table, if this space has one.
    ///
    /// Takes `&mut self` for the same reason as
    /// [`flat_view`](TopologyGuard::flat_view).
    #[inline]
    pub fn dispatch(&mut self) -> Option<&Dispatch> {
        self.sync();
        self.topo.dispatch.as_ref()
    }

    /// The root mappings, in mapping order.
    pub fn mappings(&self) -> impl Iterator<Item = (MappingId, &Mapping)> {
        self.topo.root.iter().map(|(id, m)| (*id, m))
    }

    /// Map `region` at `base` with priority 0 and no restriction on what it
    /// permits. **Retopology.**
    ///
    /// # Errors
    ///
    /// If the region does not fit in the space, or nests too deeply to
    /// flatten.
    pub fn map(&mut self, region: impl Into<RegionRef>, base: u64) -> Result<MappingId, Error> {
        self.map_with(Mapping::new(region, base))
    }

    /// Map `region` at `base` on the terms `perms` allows. **Retopology.**
    ///
    /// A read-only mapping over writable memory, a write-only aperture over a
    /// ROM: the store is untouched and only this placement is narrowed. A
    /// write to a mapping without [`Perms::WRITE`] raises
    /// [`BusError::Protected`], which a consumer can tell apart from a bad
    /// width and act on — break a copy-on-write share, widen the permission —
    /// and then reissue. See [`Perms`].
    ///
    /// # Errors
    ///
    /// As [`TopologyGuard::map`].
    pub fn map_with_perms(
        &mut self,
        region: impl Into<RegionRef>,
        base: u64,
        perms: Perms,
    ) -> Result<MappingId, Error> {
        self.map_with(Mapping::new(region, base).with_perms(perms))
    }

    /// Map `region` at `base` with an explicit priority, higher winning where
    /// regions overlap. **Retopology.**
    ///
    /// This is how a PCI BAR sits over RAM, how a boot ROM shadows the reset
    /// vector, and how a cartridge mapper puts a window over the open bus.
    ///
    /// # Errors
    ///
    /// As [`TopologyGuard::map`].
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
    /// As [`TopologyGuard::map`].
    pub fn map_with(&mut self, mapping: Mapping) -> Result<MappingId, Error> {
        self.space.check_mapping(&mapping)?;
        let id = MappingId(self.topo.next_id);
        self.topo.next_id += 1;
        self.topo.root.push((id, mapping));
        self.topo.dirty = true;
        Ok(id)
    }

    /// Remove a mapping. **Retopology.**
    ///
    /// # Errors
    ///
    /// If `id` is not a mapping of this space.
    pub fn unmap(&mut self, id: MappingId) -> Result<(), Error> {
        let Some(pos) = self.topo.root.iter().position(|(i, _)| *i == id) else {
            return Err(self.no_such_mapping(id));
        };
        self.topo.root.remove(pos);
        self.topo.dirty = true;
        Ok(())
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
        let Some(pos) = self.topo.root.iter().position(|(i, _)| *i == id) else {
            return Err(self.no_such_mapping(id));
        };
        let old = self.topo.root[pos].1.base;
        self.topo.root[pos].1.base = base;
        let mapping = self.topo.root[pos].1.clone();
        if let Err(e) = self.space.check_mapping(&mapping) {
            self.topo.root[pos].1.base = old;
            return Err(e);
        }
        self.topo.dirty = true;
        Ok(())
    }

    /// Change the terms a mapping answers on, leaving everything else alone.
    /// **Retopology.**
    ///
    /// This is `mprotect(2)`'s shape, and it is a retopology rather than a
    /// rebase on purpose: a permission change has to invalidate every TLB
    /// entry and translation block that recorded the old terms, and the
    /// generation counter is how that is announced (`ROADMAP.md` §4.1). It is
    /// cheap in the sense that matters — one flatten for a whole batch under
    /// one guard — not in the sense that nothing is invalidated.
    ///
    /// # Errors
    ///
    /// If `id` is not a mapping of this space.
    pub fn reprotect(&mut self, id: MappingId, perms: Perms) -> Result<(), Error> {
        let Some(pos) = self.topo.root.iter().position(|(i, _)| *i == id) else {
            return Err(self.no_such_mapping(id));
        };
        self.topo.root[pos].1.perms = perms;
        self.topo.dirty = true;
        Ok(())
    }

    /// Swap what a mapping places, keeping its identity and its position in
    /// the overlap order. **Retopology.**
    ///
    /// The operation a copy-on-write fault resolves itself with: the same
    /// address, the same priority, a private store behind it and
    /// [`Perms::WRITE`] restored. Doing it as `unmap` + `map` would work but
    /// would move the mapping to the back of the tie-breaking order, which is
    /// a guest-visible change nobody asked for.
    ///
    /// # Errors
    ///
    /// If `id` is not a mapping of this space, or the new mapping does not fit
    /// or nests too deeply.
    pub fn replace(&mut self, id: MappingId, mapping: Mapping) -> Result<(), Error> {
        let Some(pos) = self.topo.root.iter().position(|(i, _)| *i == id) else {
            return Err(self.no_such_mapping(id));
        };
        self.space.check_mapping(&mapping)?;
        self.topo.root[pos].1 = mapping;
        self.topo.dirty = true;
        Ok(())
    }

    /// The permissions a mapping currently answers on.
    #[must_use]
    pub fn perms_of(&self, id: MappingId) -> Option<Perms> {
        self.topo
            .root
            .iter()
            .find(|(i, _)| *i == id)
            .map(|(_, m)| m.perms)
    }

    /// Reflatten and rebuild the dispatch table, bumping the generation.
    /// **Retopology.**
    ///
    /// Every mutation on this guard defers to here, and the guard performs it
    /// when it closes, so a caller almost never needs this. It stays public
    /// because a container's contents can be rebuilt out from under a space
    /// and a machine may need to say so.
    pub fn rebuild(&mut self) {
        self.topo.dirty = true;
        self.sync();
    }

    /// Perform the deferred flatten, if there is one.
    ///
    /// Infallible, and that is a property of [`map_with`](TopologyGuard::map_with)
    /// rather than of this function: nesting depth is the only way flattening
    /// can fail and a mapping too deep to flatten is rejected when it is added.
    /// If that invariant were ever broken the space would be left answering
    /// nothing — loudly wrong rather than quietly stale, which is the only
    /// defensible choice this far from the caller.
    fn sync(&mut self) {
        if !self.topo.dirty {
            return;
        }
        self.topo.dirty = false;
        let children: Vec<Mapping> = self.topo.root.iter().map(|(_, m)| m.clone()).collect();
        let (flat, index) = match FlatView::build(&children, self.space.size(), self.space.combine)
        {
            Ok(built) => built,
            Err(_) => {
                debug_assert!(false, "`map` accepted a mapping that cannot be flattened");
                (FlatView::default(), RebaseIndex::new())
            }
        };
        self.topo.dispatch = Dispatch::build(&flat, self.space.dispatch_policy);
        self.topo.flat = flat;
        self.topo.rebase_index = index;
        // The write guard excludes every reader, so this cannot race;
        // `fetch_add` for the ordering, not for the atomicity.
        self.space.generation.fetch_add(1, Ordering::Relaxed);
    }

    fn no_such_mapping(&self, id: MappingId) -> Error {
        Error::Config {
            at: self.space.name.clone(),
            message: alloc::format!("no mapping {id:?} in this space"),
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
