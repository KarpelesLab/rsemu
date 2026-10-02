//! The exclusive monitors behind `LDREX`/`STREX`.
//!
//! ARMv6 replaced `SWP` as the synchronization primitive with a pair: a
//! load-exclusive *marks* an address, and a store-exclusive to it succeeds
//! only if nothing has disturbed the mark in between (DDI 0406C A3.4). Two
//! monitors decide "nothing":
//!
//! - the **local** monitor, one per core, which this module implements and
//!   the core owns as architectural state; and
//! - the **global** monitor, which tracks marks across cores for shareable
//!   memory and is cleared by *another* observer's store to the marked
//!   granule (A3.4.2).
//!
//! # What is modelled
//!
//! The local monitor, fully: `LDREX*` marks the physical granule, `STREX*`
//! passes only if the mark is set and covers the address, and every
//! store-exclusive, `CLREX`, exception entry and exception return clears it.
//! Clearing on exception entry and return is always a permitted outcome — a
//! store-exclusive may fail for IMPLEMENTATION DEFINED reasons (A3.4.5) —
//! and it is the safe one: an interrupted pair retries rather than
//! succeeding across a handler that may have written the location.
//!
//! The global monitor is the [`GlobalMonitor`] seam, and [`SharedMonitor`]
//! the implementation a machine shares between its cores — in a machine
//! file, an `arm.exclusive` object the cores name as their `monitor`. Each
//! core is handed it with
//! [`Arm::attach_global_monitor`](super::Arm::attach_global_monitor), and
//! the interpreter consults it at exactly the three points the architecture
//! names — mark, check-on-store-exclusive, and every other store — without
//! any change to the instruction implementations. Without one, a
//! store-exclusive answers to its own core's local monitor, which is right
//! for a single core and wrong for several: another core's store between a
//! load-exclusive and its store-exclusive goes unseen, and two threads both
//! take one lock.
//!
//! # The granule
//!
//! Tags are kept at [`GRANULE`] resolution, the Exclusives Reservation
//! Granule a Cortex-A9 reports in `CTR.ERG` (eight words). The architecture
//! lets a store-exclusive to a *different* address in the same granule
//! succeed or fail at the implementation's choice (A3.4.3); succeeding is the
//! choice a granule-tagged monitor makes.

use core::fmt;

use crate::core::space::RequesterId;

/// The Exclusives Reservation Granule, in bytes: 2^`CTR.ERG` words with
/// Cortex-A9's `ERG` of 3 (Cortex-A9 TRM, "Cache Type Register").
pub const GRANULE: u32 = 32;

/// One core's local exclusive monitor.
///
/// Architectural state, so it is saved with the core: a snapshot taken
/// between an `LDREX` and its `STREX` must restore to a machine whose
/// `STREX` still succeeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LocalMonitor {
    /// The granule-aligned physical address marked, if any.
    tag: Option<u32>,
}

impl LocalMonitor {
    /// The Open Access state: nothing marked.
    pub const OPEN: LocalMonitor = LocalMonitor { tag: None };

    /// Mark the granule containing `pa` (`MarkExclusiveLocal`).
    pub fn mark(&mut self, pa: u32) {
        self.tag = Some(pa & !(GRANULE - 1));
    }

    /// Whether a store-exclusive to `pa` passes (`IsExclusiveLocal`).
    #[must_use]
    pub fn covers(&self, pa: u32) -> bool {
        self.tag == Some(pa & !(GRANULE - 1))
    }

    /// Back to Open Access (`ClearExclusiveLocal`).
    pub fn clear(&mut self) {
        self.tag = None;
    }

    /// The marked granule, for a debugger and for the snapshot.
    #[must_use]
    pub const fn tag(&self) -> Option<u32> {
        self.tag
    }

    /// Rebuild from a snapshot.
    #[must_use]
    pub const fn from_tag(tag: Option<u32>) -> LocalMonitor {
        LocalMonitor { tag }
    }
}

/// A global exclusive monitor, shared by every core in a coherence domain.
///
/// Not implemented by anything in this crate yet; see the module docs.
/// Every method is called from inside a core's step with its execution lock
/// held, so an implementation takes its own lock at a rank below
/// [`LockRank::BUS`](crate::core::sync::LockRank::BUS) and must not call
/// back into any core.
pub trait GlobalMonitor: Send + Sync + fmt::Debug {
    /// `MarkExclusiveGlobal`: `requester` has load-exclusived `size` bytes at
    /// `pa`.
    fn mark(&self, requester: RequesterId, pa: u64, size: u32);

    /// `IsExclusiveGlobal`, then clear: whether `requester`'s store-exclusive
    /// to `pa` may proceed. Called only after the local monitor passed. The
    /// requester's own mark is cleared whatever the answer.
    fn store_exclusive(&self, requester: RequesterId, pa: u64, size: u32) -> bool;

    /// `requester` wrote `size` bytes at `pa` — any store, exclusive or not
    /// — so every *other* requester's mark on that granule is lost.
    fn observe_store(&self, requester: RequesterId, pa: u64, size: u32);
}

/// A global exclusive monitor shared by the cores of one machine.
///
/// One mark per core, kept in a slot a core claims the first time it marks
/// anything. Lock-free, because [`observe_store`](GlobalMonitor::observe_store)
/// runs on every store every core makes: a store compares one word per
/// other core and writes nothing unless it hits a mark.
///
/// Granule-tagged like the local monitor ([`GRANULE`]): another core's store
/// anywhere in a marked granule clears the mark, which the architecture
/// permits (DDI 0406C A3.4.3) and which is what keeps a lock word and the
/// data beside it consistent.
#[derive(Debug)]
pub struct SharedMonitor {
    slots: alloc::vec::Vec<Slot>,
}

#[derive(Debug)]
struct Slot {
    owner: crate::core::sync::AtomicU32,
    tag: crate::core::sync::AtomicU64,
}

/// A slot's tag when nothing is marked.
const UNMARKED: u64 = u64::MAX;

impl SharedMonitor {
    /// A monitor for at most `cores` cores.
    #[must_use]
    pub fn new(cores: usize) -> SharedMonitor {
        SharedMonitor {
            slots: (0..cores)
                .map(|_| Slot {
                    owner: crate::core::sync::AtomicU32::new(RequesterId::ANONYMOUS.0),
                    tag: crate::core::sync::AtomicU64::new(UNMARKED),
                })
                .collect(),
        }
    }

    fn granule(pa: u64) -> u64 {
        pa & !u64::from(GRANULE - 1)
    }

    /// `requester`'s slot, claiming a free one the first time.
    fn slot(&self, requester: RequesterId) -> Option<&Slot> {
        use crate::core::sync::Ordering;
        if let Some(slot) = self
            .slots
            .iter()
            .find(|s| s.owner.load(Ordering::Acquire) == requester.0)
        {
            return Some(slot);
        }
        self.slots.iter().find(|s| {
            s.owner
                .compare_exchange(
                    RequesterId::ANONYMOUS.0,
                    requester.0,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
        })
    }

    /// Clear every mark — what a reset of the whole machine does.
    pub fn clear(&self) {
        for slot in &self.slots {
            slot.tag
                .store(UNMARKED, crate::core::sync::Ordering::Release);
        }
    }
}

impl GlobalMonitor for SharedMonitor {
    fn mark(&self, requester: RequesterId, pa: u64, _size: u32) {
        if let Some(slot) = self.slot(requester) {
            slot.tag
                .store(Self::granule(pa), crate::core::sync::Ordering::Release);
        }
    }

    fn store_exclusive(&self, requester: RequesterId, pa: u64, _size: u32) -> bool {
        // A core this monitor has no room for is treated as alone: its local
        // monitor already passed.
        let Some(slot) = self.slot(requester) else {
            return true;
        };
        slot.tag.swap(UNMARKED, crate::core::sync::Ordering::AcqRel) == Self::granule(pa)
    }

    fn observe_store(&self, requester: RequesterId, pa: u64, size: u32) {
        use crate::core::sync::Ordering;
        let first = Self::granule(pa);
        let last = Self::granule(pa + u64::from(size.max(1)) - 1);
        for slot in &self.slots {
            if slot.owner.load(Ordering::Relaxed) == requester.0 {
                continue;
            }
            let tag = slot.tag.load(Ordering::Acquire);
            if tag != UNMARKED && (tag == first || tag == last) {
                let _ =
                    slot.tag
                        .compare_exchange(tag, UNMARKED, Ordering::AcqRel, Ordering::Acquire);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: RequesterId = RequesterId(7);
    const B: RequesterId = RequesterId(9);

    #[test]
    fn another_cores_store_breaks_a_mark_and_its_own_does_not() {
        let m = SharedMonitor::new(2);
        m.mark(A, 0x1004, 4);
        m.observe_store(A, 0x1008, 4);
        assert!(
            m.store_exclusive(A, 0x1004, 4),
            "a core's own store keeps its mark"
        );
        m.mark(A, 0x1004, 4);
        m.observe_store(B, 0x101c, 4);
        assert!(
            !m.store_exclusive(A, 0x1004, 4),
            "the other core's store in the granule broke it"
        );
    }

    #[test]
    fn two_cores_racing_for_one_lock_word_get_one_winner() {
        // A: LDREX; B: LDREX, STREX (wins, and that store breaks A's mark);
        // A: STREX fails and retries.
        let m = SharedMonitor::new(2);
        m.mark(A, 0x2000, 4);
        m.mark(B, 0x2000, 4);
        assert!(m.store_exclusive(B, 0x2000, 4));
        m.observe_store(B, 0x2000, 4);
        assert!(!m.store_exclusive(A, 0x2000, 4));
    }

    #[test]
    fn a_store_exclusive_consumes_the_mark() {
        let m = SharedMonitor::new(1);
        m.mark(A, 0x40, 4);
        assert!(m.store_exclusive(A, 0x40, 4));
        assert!(
            !m.store_exclusive(A, 0x40, 4),
            "no second success without a new mark"
        );
    }

    #[test]
    fn a_mark_covers_its_granule_and_nothing_else() {
        let mut m = LocalMonitor::OPEN;
        assert!(!m.covers(0x1000));
        m.mark(0x1004);
        assert!(m.covers(0x1000));
        assert!(m.covers(0x101c));
        assert!(!m.covers(0x1020));
        m.clear();
        assert!(!m.covers(0x1004));
    }
}
