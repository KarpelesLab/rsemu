//! The host's monotonic clock, as [`core::sched`](crate::core::sched) wants it
//! injected.
//!
//! Nothing under `core/`, `cpu/`, `dev/` or `machine/` may read a wall clock
//! (`ROADMAP.md` §15, invariant 4), so the scheduler takes a
//! [`HostClock`] rather than reaching for one.
//! This is the implementation of that trait for a machine running on a real
//! operating system, and it is deliberately the *only* one in the tree: a
//! second reading of the wall clock somewhere else in a frontend is exactly the
//! bug the injection exists to prevent.
//!
//! # Why it belongs to the rate controller and nothing else
//!
//! [`RateController`](crate::core::sched::RateController) is what makes a live
//! frontend run at human speed: virtual time is paced against this clock, and
//! [`Scheduler::pace`](crate::core::sched::Scheduler::pace) says how long to
//! wait. A frontend that measured its own elapsed time and slept on that would
//! be a second, unsynchronised rate controller — and the first thing that would
//! go wrong is that the two would disagree about how much a stall cost, which
//! is the case `RateControl::Realtime`'s catch-up limit exists to handle.

use std::time::Instant;

use crate::core::sched::HostClock;

/// [`Instant`], as a [`HostClock`].
///
/// The origin is whenever this was constructed, which satisfies the trait's
/// only requirement — monotonic nanoseconds from some fixed, arbitrary point.
/// A saturating conversion, because `Instant`'s span is longer than a `u64` of
/// nanoseconds (584 years) and a process that ran that long deserves a wrong
/// answer rather than a panic.
#[derive(Debug, Clone, Copy)]
pub struct MonotonicClock {
    origin: Instant,
}

impl MonotonicClock {
    /// A clock whose origin is now.
    #[must_use]
    pub fn new() -> MonotonicClock {
        MonotonicClock {
            origin: Instant::now(),
        }
    }
}

impl Default for MonotonicClock {
    fn default() -> MonotonicClock {
        MonotonicClock::new()
    }
}

impl HostClock for MonotonicClock {
    fn monotonic_nanos(&self) -> u64 {
        u64::try_from(self.origin.elapsed().as_nanos()).unwrap_or(u64::MAX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_starts_near_zero_and_never_goes_backwards() {
        let clock = MonotonicClock::new();
        let first = clock.monotonic_nanos();
        // A second of slack: the assertion is "the origin is construction", not
        // a timing measurement, and a loaded CI box is allowed to be slow.
        assert!(first < 1_000_000_000, "{first}");
        let second = clock.monotonic_nanos();
        assert!(second >= first);
    }

    #[test]
    fn it_is_the_trait_the_scheduler_asks_for() {
        fn takes(_: Box<dyn HostClock>) {}
        takes(Box::new(MonotonicClock::new()));
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<MonotonicClock>();
    }
}
