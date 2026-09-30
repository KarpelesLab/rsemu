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
//! **The global monitor is not modelled yet**; there is one core per
//! machine until SMP lands. The seam for it is [`GlobalMonitor`]: a machine
//! with several cores builds one, hands it to each with
//! [`Arm::attach_global_monitor`](super::Arm::attach_global_monitor), and
//! the interpreter consults it at exactly the three points the architecture
//! names — mark, check-on-store-exclusive, and every other store — without
//! any change to the instruction implementations.
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

#[cfg(test)]
mod tests {
    use super::*;

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
