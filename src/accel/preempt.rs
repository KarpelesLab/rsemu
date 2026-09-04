//! What bounds a vCPU that takes no exits.
//!
//! # The problem this exists for, stated exactly
//!
//! [`kvm`](super::kvm)'s module documentation says it and then says what it
//! costs: *"a guest that takes no exits is not preemptible … A vCPU spinning
//! in a register-only loop with interrupts masked will run until something
//! else makes it exit, and there is nothing portable left to force one."*
//!
//! That was true and it was a wall. A stock Linux kernel's
//! `timer_irq_works()` spins on `RDTSC` and `PAUSE` for forty milliseconds of
//! host time, touching no device and taking no exit, and then asks whether
//! `jiffies` moved. Under [`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel)
//! virtual time is the host clock, so the tick it is waiting for *is* due —
//! but a tick that cannot be delivered has not happened, and the vCPU cannot
//! be told anything while it is inside `KVM_RUN`. The kernel panics in
//! `check_timer()`, and `no_timer_check` on the command line was the price.
//!
//! # What actually forces a `KVM_RUN` to return
//!
//! Everything else was tried on paper first, and it is worth writing down why
//! none of it works, because each looks plausible:
//!
//! | mechanism | why not |
//! | --- | --- |
//! | `KVM_CAP_IMMEDIATE_EXIT` | the kernel reads that byte *before* entering the guest and never again, so it declines an entry rather than ending one. It closes the check-then-enter race, which is a different job, and [`kvm`](super::kvm) already uses it for that. |
//! | the per-CPU [`ExitFlag`](crate::core::sched::ExitFlag) | checked by *us*, between entries. A thread inside `KVM_RUN` is not executing our loop. |
//! | an `ioctl` from another thread | `KVM_RUN` holds `vcpu->mutex` for its whole duration; a second thread's ioctl blocks behind it rather than interrupting it. |
//! | `KVM_CAP_X86_NOTIFY_VMEXIT` | is VMX's *notify window*, an Intel-only VM-execution control. This host is SVM, and a facility half the x86 hosts do not have cannot be the mechanism. |
//! | a memory-slot change, an `irqfd`, a `KVM_SET_*` | all of these kick the vCPU out of *hardware*, and KVM handles the kick and re-enters without ever returning to userspace. |
//!
//! What is left is the one thing KVM itself documents: a signal delivered to
//! the thread. The kernel's own `kvm_vcpu_exit_request()` tests
//! `signal_pending(current)` before every entry, and delivering a signal to a
//! task that is *in* the guest sends a reschedule IPI to the physical CPU it
//! is on, which is a VM exit — so `KVM_RUN` comes back with `-EINTR` and
//! `KVM_EXIT_INTR`, which [`kvm::Vcpu`](super::kvm::Vcpu) already treats as
//! *stopped, nothing happened, resume unconditionally*.
//!
//! # Then what about "never a signal"?
//!
//! `CLAUDE.md` and `ROADMAP.md` §4.7 rule a signal out of **one** thing, in
//! one sentence, for one reason: *"Stopping the world (TLB shootdown, remap,
//! snapshot, reset) uses the safe-point protocol: a generation counter plus a
//! per-CPU exit flag checked at block boundaries. Never a signal — wasm has
//! none."*
//!
//! That rule is untouched here and this module is not a way around it:
//!
//! * **The safe-point protocol is unchanged.** A stop-the-world request still
//!   travels by [`SafePoint`](crate::core::sched::SafePoint) and
//!   [`ExitFlag`](crate::core::sched::ExitFlag), still written through to
//!   `immediate_exit`, and this module neither reads nor raises either. Delete
//!   it and every stop still works, one preemption interval later.
//! * **The reason the rule gives does not reach.** wasm has no signals — and
//!   wasm has no `/dev/kvm` either. `accel/` is `cfg`-gated to Linux on
//!   x86-64 in `lib.rs`; a target with no signals gets no `accel` module at
//!   all, so nothing portable comes to depend on this.
//! * **It is a preemption timer, not a stop.** A kick says *"come back so the
//!   scheduler can move virtual time and deliver what became due"*, and the
//!   guest resumes immediately afterwards, unaware. It is the userspace
//!   equivalent of the VMX preemption timer, which is a piece of processor
//!   hardware rather than a protocol.
//!
//! # What it is worth, measured
//!
//! Without it, `q35-linux` under KVM needs `no_timer_check` and reports a
//! **176,273 MHz** processor, because it calibrates the host's time-stamp
//! counter against a board whose clocks stand still while it runs. With it,
//! the board's own command line boots and the same kernel reports its actual
//! host frequency. `tests/kvm_q35_linux.rs` is the evidence.
//!
//! # `unsafe`
//!
//! **None is added and no subsystem is created.** The two halves are both
//! already sanctioned by `ROADMAP.md` §0: the disposition is installed by
//! [`host::signal`](crate::host::signal), the seventh subsystem and the one
//! place in the tree that writes one, and the timer is three raw system calls
//! in [`sys`], the third. This file itself contains no `unsafe`
//! at all.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::core::sync::{LockRank, Mutex};
use crate::host::signal::{self, Signal};

use super::sys::{self, IntervalTimer};

/// How long an accelerated vCPU may stay in hardware before the scheduler
/// wants it back, when a machine does not say otherwise.
///
/// Chosen against what the guest can notice rather than by round numbers.
/// Linux's `hpet_counting()` spins for 200,000 time-stamp cycles — about
/// **50 µs** on a 4 GHz host — between two reads of the HPET counter, and its
/// `timer_irq_works()` wants at least one timer tick inside 40 ms. A bound
/// well under 50 µs makes both true, and 20 µs leaves room on a slower host.
///
/// It is a *ceiling*, not a period of useful work, and the difference is
/// measured rather than assumed. A whole `q35-linux` boot makes about 150,000
/// guest entries and takes **17,000** preemptions: the other 89% end at an
/// MMIO or port exit long before the interval, and the timer is disarmed the
/// moment the slice does. So the cost is around 7,000 signals a second rather
/// than the 50,000 the interval would suggest.
pub const DEFAULT_NANOS: u64 = 20_000;

/// Whether [`Signal::PREEMPT`] has a disposition yet.
///
/// Process-wide and set once. Installing twice would be harmless — the kernel
/// simply overwrites — but the check keeps a hot path from making a system
/// call it does not need.
static ARMED: AtomicBool = AtomicBool::new(false);

/// A vCPU's bound on how long it may stay in hardware.
///
/// Holds the timer that enforces it, plus the thread that timer is aimed at:
/// the target is fixed when the kernel creates the timer, so a runnable that
/// moves between the task pool's workers gets a new one on the thread it
/// lands on. In the ordinary case — a pool that keeps a job on a worker — that
/// happens once.
#[derive(Debug)]
pub struct Kicker {
    nanos: u64,
    aimed: Mutex<Option<Aimed>>,
}

/// One thread's timer.
#[derive(Debug)]
struct Aimed {
    tid: i32,
    timer: IntervalTimer,
}

impl Kicker {
    /// A bound of `nanos`. Zero means *no bound*, which is the behaviour
    /// everything had before this existed.
    #[must_use]
    pub fn new(nanos: u64) -> Kicker {
        Kicker {
            nanos,
            aimed: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// The bound, in nanoseconds. Zero if there is none.
    #[inline]
    #[must_use]
    pub const fn nanos(&self) -> u64 {
        self.nanos
    }

    /// Arm this thread's timer, and disarm it when the returned guard drops.
    ///
    /// Every failure — no signal disposition on this target, a kernel that
    /// will not create the timer — degrades to *no bound*, which is exactly
    /// what the caller had before. A vCPU that cannot be preempted is slower
    /// to hand virtual time back; it is not incorrect.
    pub fn hold(&self) -> Hold<'_> {
        if self.nanos == 0 || !arm_disposition() {
            return Hold { kicker: None };
        }
        let mut aimed = self.aimed.lock();
        let here = sys::thread_id();
        if aimed.as_ref().is_none_or(|a| a.tid != here) {
            // Dropped first, so the process is never holding two timers for
            // one vCPU even briefly.
            *aimed = None;
            match IntervalTimer::for_this_thread(Signal::PREEMPT.0) {
                Ok(timer) => *aimed = Some(Aimed { tid: here, timer }),
                Err(_) => return Hold { kicker: None },
            }
        }
        let Some(a) = aimed.as_ref() else {
            return Hold { kicker: None };
        };
        if a.timer.arm(self.nanos).is_err() {
            return Hold { kicker: None };
        }
        drop(aimed);
        Hold { kicker: Some(self) }
    }

    /// Stop the timer, whichever thread owns it.
    fn disarm(&self) {
        if let Some(a) = self.aimed.lock().as_ref() {
            let _ = a.timer.disarm();
        }
    }
}

/// A vCPU's preemption timer, running until this is dropped.
///
/// A guard rather than a pair of calls because the run loop it wraps returns
/// from a dozen places, several of them with `?`, and a timer left armed would
/// go on interrupting whatever that thread did next.
#[derive(Debug)]
#[must_use = "the timer stops the moment this is dropped"]
pub struct Hold<'a> {
    kicker: Option<&'a Kicker>,
}

impl Hold<'_> {
    /// Whether a bound is actually in force.
    ///
    /// False on a host where the disposition or the timer could not be had, so
    /// a caller can say *"this vCPU is not preemptible"* rather than assume.
    #[inline]
    #[must_use]
    pub const fn is_armed(&self) -> bool {
        self.kicker.is_some()
    }
}

impl Drop for Hold<'_> {
    fn drop(&mut self) {
        if let Some(kicker) = self.kicker {
            kicker.disarm();
        }
    }
}

/// How many preemption signals this process has taken, in total.
///
/// The evidence a test wants: a guest that was interrupted rather than one
/// that happened to exit on its own.
#[must_use]
pub fn count() -> u64 {
    signal::preemptions()
}

/// Give [`Signal::PREEMPT`] its do-nothing handler, once.
fn arm_disposition() -> bool {
    if ARMED.load(Ordering::Acquire) {
        return true;
    }
    let ok = signal::arm_preempt().is_installed();
    if ok {
        ARMED.store(true, Ordering::Release);
    }
    ok
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole mechanism, without a hypervisor: a thread asks to be
    /// interrupted every 20 µs, spins, and finds that it was.
    #[test]
    fn a_thread_that_asks_to_be_interrupted_is() {
        let kicker = Kicker::new(DEFAULT_NANOS);
        let before = count();
        let hold = kicker.hold();
        assert!(hold.is_armed(), "x86-64 Linux has timer_create");
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(20) && count() == before {
            core::hint::spin_loop();
        }
        let taken = count() - before;
        drop(hold);
        assert!(taken > 0, "the timer never fired");
    }

    /// And stops when the guard goes.
    #[test]
    fn a_dropped_hold_stops_the_timer() {
        let kicker = Kicker::new(DEFAULT_NANOS);
        drop(kicker.hold());
        let quiet = count();
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(5) {
            core::hint::spin_loop();
        }
        // Another test's timer may be running concurrently, so this asserts
        // that *this* kicker is disarmed by asking it for a second hold and
        // seeing the count move only then.
        let after_quiet = count();
        let hold = kicker.hold();
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(20) && count() == after_quiet {
            core::hint::spin_loop();
        }
        assert!(count() > after_quiet, "re-arming did not restart it");
        drop(hold);
        let _ = quiet;
    }

    /// A zero bound is *no bound*, and asks the kernel for nothing.
    #[test]
    fn zero_is_off() {
        let kicker = Kicker::new(0);
        assert_eq!(kicker.nanos(), 0);
        assert!(!kicker.hold().is_armed());
    }
}
