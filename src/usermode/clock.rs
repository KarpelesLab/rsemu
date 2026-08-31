//! Virtual time for a level-3 run.
//!
//! `ROADMAP.md` §4.2 gives the scheduler ownership of time, and phase 5b says
//! that rule still holds *"when the scheduled thing is a thread rather than a
//! CPU"*. This is how it holds: a level-3 run has one oscillator, guest threads
//! advance it by the ticks they execute, and every question about "what time is
//! it" is answered from that and never from the host.
//!
//! # Why this matters more here than at level 1
//!
//! At level 1 a guest reads a timer chip, and the timer chip is on the same
//! clock forest as the CPU, so time is already exact. At level 3 there is no
//! timer chip: the guest asks its kernel, and if the kernel answers from the
//! host clock then the answer is a non-deterministic input crossing into the
//! machine — §0's exact words, and phase 5b's stated determinism gate.
//!
//! A [`GuestClock`] makes that answer a **function of execution instead**: the
//! same program run twice sees the same clock, on any host, under a debugger,
//! a year apart. The consumer never has to record or replay a clock read,
//! because there is nothing non-deterministic left to record — which is a
//! better outcome than journalling it, and the reason clock reads are the one
//! external input the journal in [`super::journal`] deliberately does not
//! handle.
//!
//! # Not a wall clock
//!
//! [`GuestClock::now`] starts at zero. A consumer that has to present a
//! calendar date adds its own epoch to it — and choosing that epoch is exactly
//! the kind of policy §2.1 puts on the consumer's side of the line.

use crate::core::clock::{ClockForest, DomainId, GlobalTime, OscillatorId, Rational};
use crate::core::error::{Error, Result};
use crate::core::state::{Sink, Source};
use crate::core::sync::{self, LockRank};

/// The default rate: one tick per nanosecond.
///
/// A tick is one guest bus access — an instruction fetch, a load, a store —
/// which is the currency every core in this crate already charges in
/// (`ROADMAP.md` §6). Calling that a nanosecond makes a level-3 guest look
/// like a 1 GHz machine, which is close enough to real that timing code
/// behaves and far enough from a promise that nobody mistakes it for accuracy.
/// A consumer that wants a different rate says so.
pub const DEFAULT_HZ: u64 = 1_000_000_000;

/// The state behind the lock.
#[derive(Debug)]
struct Inner {
    forest: ClockForest,
    osc: OscillatorId,
    domain: DomainId,
}

/// The one clock a level-3 run has.
///
/// Shared by every guest thread — held as an `Arc`, or by the
/// [`ThreadSet`](super::ThreadSet) that owns one — because "what time is it"
/// must have one answer for a whole run and not one per thread.
///
/// # Locking
///
/// One [`sync::Mutex`] at [`LockRank::SCHED`], matching the rank the machine
/// scheduler's own state sits at. Nothing outward is called under it.
#[derive(Debug)]
pub struct GuestClock {
    inner: sync::Mutex<Inner>,
}

impl GuestClock {
    /// A clock ticking at [`DEFAULT_HZ`], reading zero.
    #[must_use]
    pub fn new() -> GuestClock {
        // A whole-number frequency the forest already accepts, so this cannot
        // fail; `with_rate` is the fallible form for anything else.
        GuestClock::with_rate(Rational::integer(DEFAULT_HZ))
            .expect("an integral frequency is always a valid oscillator")
    }

    /// A clock ticking at `rate` hertz, reading zero.
    ///
    /// A [`Rational`] rather than a float, because §0 forbids floats in the
    /// time path and because a guest whose clock is 7/3 of something is a real
    /// configuration that a float would round.
    ///
    /// # Errors
    ///
    /// If the forest rejects the frequency — zero, or one whose denominator
    /// cannot be represented.
    pub fn with_rate(rate: Rational) -> Result<GuestClock> {
        let mut forest = ClockForest::new();
        let domain = forest
            .add_oscillator("guest", rate)
            .map_err(|e| clock_error(&e))?;
        let osc = forest.root_of(domain).map_err(|e| clock_error(&e))?;
        Ok(GuestClock {
            inner: sync::Mutex::with_rank(
                LockRank::SCHED,
                Inner {
                    forest,
                    osc,
                    domain,
                },
            ),
        })
    }

    /// How many ticks have been executed since the run began.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        let inner = self.inner.lock();
        inner.forest.ticks(inner.domain).unwrap_or(0)
    }

    /// The virtual instant the run has reached.
    #[must_use]
    pub fn now(&self) -> GlobalTime {
        let inner = self.inner.lock();
        inner
            .forest
            .global_time(inner.osc)
            .unwrap_or(GlobalTime::ZERO)
    }

    /// The virtual instant the run has reached, in nanoseconds.
    ///
    /// The form a consumer's `clock_gettime` wants. Exact integer arithmetic
    /// all the way down: no float ever touches this number.
    #[must_use]
    pub fn nanos(&self) -> u64 {
        self.now().as_nanos()
    }

    /// The virtual instant `ticks` from the start of the run.
    ///
    /// How a consumer turns "wake me in 10 ms" into something comparable with
    /// [`now`](GuestClock::now) without leaving the tick domain.
    #[must_use]
    pub fn at_tick(&self, tick: u64) -> GlobalTime {
        let inner = self.inner.lock();
        inner
            .forest
            .global_time_of_tick(inner.domain, tick)
            .unwrap_or(GlobalTime::ZERO)
    }

    /// The first tick at or after `at`.
    ///
    /// The inverse of [`at_tick`](GuestClock::at_tick), and what a scheduler
    /// needs to turn a deadline back into a budget.
    #[must_use]
    pub fn tick_of(&self, at: GlobalTime) -> u64 {
        let inner = self.inner.lock();
        inner.forest.units_at_global(inner.osc, at).unwrap_or(0)
    }

    /// Move the clock forward by `ticks` executed ticks.
    ///
    /// Called by whoever ran a guest thread, with what that thread reported
    /// consuming. This is the only way time moves in a level-3 run: nothing
    /// here reads a host clock, and nothing here can (`ROADMAP.md` §0).
    pub fn advance(&self, ticks: u64) {
        if ticks == 0 {
            return;
        }
        let mut inner = self.inner.lock();
        let domain = inner.domain;
        // A saturating forest error would mean the domain vanished, which
        // cannot happen: it was created in the constructor and nothing removes
        // one.
        let _ = inner.forest.advance_domain(domain, ticks);
    }

    /// Move the clock forward *to* `at`, if it is not already past it.
    ///
    /// What a scheduler does when every thread is asleep: rather than a host
    /// `sleep`, virtual time jumps to the earliest deadline, which is both
    /// instant and reproducible. Returns the ticks skipped.
    pub fn advance_to(&self, at: GlobalTime) -> u64 {
        let mut inner = self.inner.lock();
        let osc = inner.osc;
        inner.forest.advance_to_global(osc, at).unwrap_or(0)
    }

    /// Write the clock's position.
    ///
    /// The rate is *not* written: it is configuration the consumer chose, like
    /// an [`ExitMask`](crate::core::exec::ExitMask), and a restore into a
    /// differently-rated clock should be the consumer's error to catch rather
    /// than a silent reconfiguration.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    pub fn save<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        sink.write_u64(self.ticks())
    }

    /// Read the clock's position back.
    ///
    /// # Errors
    ///
    /// If the source is truncated, or the forest refuses the position.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, source: &mut S) -> Result<()> {
        let ticks = source.read_u64()?;
        let mut inner = self.inner.lock();
        let (osc, domain) = (inner.osc, inner.domain);
        inner
            .forest
            .restore_unit_position(osc, ticks)
            .map_err(|e| clock_error(&e))?;
        inner
            .forest
            .restore_ticks(domain, ticks)
            .map_err(|e| clock_error(&e))?;
        Ok(())
    }
}

impl Default for GuestClock {
    fn default() -> Self {
        GuestClock::new()
    }
}

/// Wrap a clock-forest error as a crate error.
fn clock_error(e: &crate::core::clock::ClockError) -> Error {
    Error::Config {
        at: "guest clock".into(),
        message: alloc::format!("{e}"),
    }
}
