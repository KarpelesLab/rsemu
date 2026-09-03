//! The host asking a run to stop, and how that reaches the machine.
//!
//! A `rsemu run` ends in one of two ways. It reaches its `--for` deadline, or
//! somebody presses Ctrl-C. The first was always handled; the second was not.
//! `src/bin/rsemu.rs`'s `finish` is what makes a `--drive` durable — *"every
//! way out of a run goes through here, including the failing ones"* — and a
//! signal with its default disposition walks straight past it, because the
//! process is gone before `main` returns. On a qcow2 that is not staleness:
//! the data clusters are in the file and the L1/L2 tables that find them are
//! in `fstool`'s write-back cache, so what is left on disk is an image with a
//! hole where the guest's sector used to be.
//!
//! So this module exists to turn three signals into *a flag the run loop
//! reads*, and nothing more.
//!
//! # What is handled, and what each one means
//!
//! | Signal | Where it comes from | What it does here |
//! | --- | --- | --- |
//! | `SIGINT` (2) | Ctrl-C on a cooked terminal, or `kill -INT` | stop at the next slice boundary and flush |
//! | `SIGTERM` (15) | `kill`, a supervisor, `systemctl stop` | the same |
//! | `SIGHUP` (1) | the controlling terminal went away | the same: nobody is watching the console any more, but the disk still matters |
//!
//! **`SIGQUIT` is deliberately left alone.** Its conventional meaning is *quit
//! and dump core*, and an emulator wedged in a guest loop is exactly when
//! somebody wants a core file. Taking it over would remove the one escape
//! hatch that produces evidence.
//!
//! **A second signal of the same kind kills the process outright.** The
//! handler is installed with `SA_RESETHAND`, so the kernel restores the
//! default disposition *before* entering it: the first Ctrl-C asks for a clean
//! stop, and a second one — from a user who has decided the clean stop is
//! taking too long — gets what Ctrl-C has always got. Nothing here has to
//! count anything for that to work, and a run wedged somewhere that never
//! reaches a slice boundary is still escapable without `SIGKILL`.
//!
//! Ctrl-C on a **raw** terminal never reaches this: the kernel stops turning
//! that keystroke into a signal, and [`Terminal`](super::terminal::Terminal)
//! consumes the `0x03` byte itself. The two mechanisms cover disjoint cases
//! and the run loops consult both.
//!
//! # What runs in the handler
//!
//! One relaxed atomic store of the signal number, and a return. That is the
//! whole handler. A signal handler runs on a borrowed stack with almost
//! nothing safe to call — no allocation, no locking, no `Arc` — so everything
//! else happens on the main path, at a point the run loop was already going to
//! check.
//!
//! # The safe-point protocol, and why the handler does not touch it
//!
//! `ROADMAP.md` §4.7 fixes how the world is stopped: *"a generation counter
//! plus a per-CPU exit flag checked at translation-block boundaries — never a
//! host signal, because wasm has none"*. This does not change that and could
//! not: it is a `host/` facility, and the machine below it still has exactly
//! one stop mechanism.
//!
//! What the run loop does when [`caught`] answers `Some` is
//! [`SafePoint::request`](crate::core::sched::SafePoint::request) — §4.7's own
//! call, made from the main path with the machine between slices. The signal
//! *requests* a stop through the existing mechanism; it never performs one.
//! Reaching the machine's [`SafePoint`](crate::core::sched::SafePoint) from
//! inside the handler would mean loading an `Arc` out of a `OnceLock` in
//! async-signal context, which is precisely what the handler rule forbids, and
//! it would buy a stop that arrives one slice sooner.
//!
//! # Determinism
//!
//! A signal is a non-deterministic host input, and it does **not** cross into
//! the machine: it injects nothing a guest can observe, changes no device
//! state, and delivers no input event. All it does is end the run at a slice
//! boundary — the same boundary a shorter `--for` would have ended it at. So
//! it needs no record/replay seam: a recorded session that was interrupted is
//! a recording of a shorter run, and it replays exactly.
//!
//! # Why a raw system call
//!
//! `CLAUDE.md`: *"OS interaction is by raw syscall (the `purestd` pattern), not
//! via `libc`"*. `std` exposes no signal handling at all, and unlike raw
//! terminal mode — which [`terminal`](super::terminal) reaches by running
//! `stty`, because terminal settings belong to a device another program can
//! act on — a signal disposition is *this process's own* state and no external
//! program can set it. There is no pure-`std` route, so this is
//! `rt_sigaction(2)` by `syscall` instruction, on the model of `accel::sys`.
//!
//! ## Sources
//!
//! * The x86-64 System V syscall convention: number in `rax`, arguments in
//!   `rdi`, `rsi`, `rdx`, `r10`, result in `rax`, `rcx` and `r11` destroyed by
//!   the `syscall` instruction (*Intel SDM* volume 2, `SYSCALL`; System V
//!   AMD64 ABI supplement).
//! * `rt_sigaction` is call 13 and `rt_sigreturn` is call 15 in the x86-64
//!   table (`arch/x86/entry/syscalls/syscall_64.tbl`), stable ABI since 2.6.
//! * The kernel's `struct sigaction` for architectures that define
//!   `SA_RESTORER` — handler, flags, restorer, mask, in that order — and the
//!   `SA_*` flag values are `asm-generic/signal.h` plus x86's
//!   `arch/x86/include/uapi/asm/signal.h`. `sigaction(2)`, "C library/kernel
//!   differences", documents how that structure differs from libc's and that
//!   the raw call takes the size of the signal set as its fourth argument.
//! * `SA_RESTORER` is **mandatory** on x86-64: the kernel has no trampoline of
//!   its own there and refuses to build a signal frame for a handler without
//!   one, which is why a two-instruction `rt_sigreturn` stub is assembled
//!   below. Other architectures do not need it; this module does not target
//!   them.
//!
//! # Where this works
//!
//! x86-64 Linux, which is where a raw `syscall` instruction and those numbers
//! mean anything. Everywhere else — wasm, macOS, Windows, another Linux
//! architecture — [`arm`] answers [`Arming::Unavailable`] and every signal
//! keeps the disposition it has always had. That is the previous behaviour
//! rather than a new failure, and [`Arming`] is returned rather than swallowed
//! so a caller can say so.

use core::fmt;
use core::sync::atomic::{AtomicI32, Ordering};

/// A host signal, by number.
///
/// A newtype rather than an enum because the list is the kernel's; only the
/// three this module acts on are named.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Signal(pub i32);

impl Signal {
    /// The controlling terminal went away.
    pub const HUP: Signal = Signal(1);
    /// Ctrl-C on a cooked terminal, or `kill -INT`.
    pub const INT: Signal = Signal(2);
    /// The polite ask: `kill`, a supervisor, `systemctl stop`.
    pub const TERM: Signal = Signal(15);

    /// Its short name, as `kill -l` prints it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Signal::HUP => "SIGHUP",
            Signal::INT => "SIGINT",
            Signal::TERM => "SIGTERM",
            _ => "signal",
        }
    }
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Signal::HUP | Signal::INT | Signal::TERM => f.write_str(self.name()),
            Signal(n) => write!(f, "signal {n}"),
        }
    }
}

/// Every signal [`arm`] takes over, in the order it installs them.
pub const SHUTDOWN: &[Signal] = &[Signal::INT, Signal::TERM, Signal::HUP];

/// What [`arm`] managed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arming {
    /// The handlers are in place: [`caught`] will report a shutdown request,
    /// and a run that consults it stops cleanly.
    Installed,
    /// This host has no facility this module can use, so every signal keeps
    /// its default disposition and a `SIGINT` still ends the process where it
    /// stands. See the module documentation's "Where this works".
    Unavailable,
}

impl Arming {
    /// Whether a run can expect [`caught`] to answer.
    #[must_use]
    pub const fn is_installed(self) -> bool {
        matches!(self, Arming::Installed)
    }
}

/// The signal number the handler last stored, or zero.
///
/// The **only** thing the handler touches, and a relaxed store into it is the
/// whole of what runs in async-signal context. Two signals racing means the
/// second wins, which is not a distinction a shutdown cares about.
static CAUGHT: AtomicI32 = AtomicI32::new(0);

/// Take over the shutdown signals for this process.
///
/// Idempotent in effect — installing the same disposition twice is harmless —
/// but a caller should do it once, early, before there is anything to lose.
///
/// Nothing else in the process may install a handler for [`SHUTDOWN`]: the
/// disposition is process-wide and the last writer wins.
pub fn arm() -> Arming {
    imp::arm()
}

/// Which shutdown signal has been delivered, if one has.
///
/// A plain read, safe to call as often as a loop likes; it does not clear the
/// flag, so a loop that asks twice gets the same answer twice. Call it where
/// the run already pauses — between slices — and never from a handler.
#[must_use]
pub fn caught() -> Option<Signal> {
    match CAUGHT.load(Ordering::Relaxed) {
        0 => None,
        n => Some(Signal(n)),
    }
}

/// Forget any signal that has been delivered.
///
/// For a test that drives [`arm`] and then wants a clean slate. A run has no
/// use for it: once a shutdown is asked for, it is asked for.
pub fn clear() {
    CAUGHT.store(0, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// x86-64 Linux: the raw system call
// ---------------------------------------------------------------------------

#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
mod imp {
    //! `rt_sigaction`, by `syscall` instruction.
    //!
    //! # This is the seventh `#[allow(unsafe_code)]` site, and it was reviewed
    //!
    //! `ROADMAP.md` §0 used to name **six** subsystems and say *"six is the
    //! ceiling; a seventh is a design review, not a commit"*. A host signal
    //! disposition was none of those six — it is a genuinely new category, not
    //! the unused sixth slot (which is per-CPU execution state). That review
    //! happened and the answer was to admit it: §0 and `CLAUDE.md` now name
    //! seven, with this module as the seventh, and an *eighth* is the thing
    //! that needs a review now. The ceiling moved deliberately and once.
    //!
    //! What can be argued on the merits is that nothing cheaper works. `std`
    //! has no signal API; a signal disposition is this process's own state, so
    //! there is no external program to shell out to the way
    //! [`terminal`](super::super::terminal) shells out to `stty`; and `libc`
    //! is named in `CLAUDE.md`'s dependency policy as a crate this project
    //! does not take, with the only sanctioned purity breaks being macOS
    //! Hypervisor.framework and Windows WHPX. So the alternative to this file
    //! is not a safer implementation — it is keeping the data-loss defect.
    //!
    //! So it is written to be the smallest such site in the tree: a single
    //! `asm!` block, a two-instruction `global_asm!` trampoline, and one
    //! wrapper that fills in a structure of four integers. Nothing here is
    //! generic, nothing takes a caller-supplied pointer, and the only function
    //! pointer that reaches the kernel is [`handler`] below.

    #![allow(unsafe_code)]

    use core::sync::atomic::Ordering;

    use super::{Arming, CAUGHT, SHUTDOWN, Signal};

    /// `rt_sigaction`, x86-64 call 13.
    const SYS_RT_SIGACTION: u64 = 13;

    /// The kernel is told the handler has a restorer; on x86-64 it has no
    /// trampoline of its own and will not build a frame without one.
    const SA_RESTORER: u64 = 0x0400_0000;
    /// Restart an interrupted system call rather than failing it with `EINTR`.
    /// Without this the stdin reader thread's blocking `read` comes back an
    /// error and a console session decides its input reached end of file.
    const SA_RESTART: u64 = 0x1000_0000;
    /// Put the default disposition back *before* entering the handler, so a
    /// second Ctrl-C ends the process the way the first one used to.
    const SA_RESETHAND: u64 = 0x8000_0000;

    /// The size of `sigset_t` the kernel expects as `rt_sigaction`'s fourth
    /// argument: 64 signals, eight bytes.
    const SIGSETSIZE: u64 = 8;

    /// The kernel's `struct sigaction` on an architecture with `SA_RESTORER`.
    ///
    /// **Not** libc's: the field order is handler, flags, restorer, mask, and
    /// the mask is a bare 64-bit set rather than a padded structure
    /// (`sigaction(2)`, "C library/kernel differences").
    #[repr(C)]
    struct KernelSigaction {
        handler: u64,
        flags: u64,
        restorer: u64,
        mask: u64,
    }

    /// What the kernel calls. **This is the whole of the async-signal-context
    /// code in rsemu**: one relaxed store and a return.
    extern "C" fn handler(signal: i32) {
        CAUGHT.store(signal, Ordering::Relaxed);
    }

    // The `rt_sigreturn` trampoline the kernel returns through. `global_asm!`
    // rather than a Rust function so that nothing about it depends on a
    // prologue: the kernel jumps here with the signal frame on the stack and
    // the only correct thing to do is issue call 15, which never returns.
    core::arch::global_asm!(
        ".globl rsemu_sigreturn",
        ".hidden rsemu_sigreturn",
        ".type rsemu_sigreturn, @function",
        "rsemu_sigreturn:",
        "mov rax, 15",
        "syscall",
        ".size rsemu_sigreturn, . - rsemu_sigreturn",
    );

    unsafe extern "C" {
        /// The trampoline assembled above. Never called from Rust — its
        /// address is handed to the kernel as `sa_restorer`, and that is all.
        fn rsemu_sigreturn();
    }

    /// Issue `rt_sigaction`, the only system call this module makes.
    ///
    /// Private, and the one `asm!` block: `act` is always the structure built
    /// by [`install`] below, so there is no way for a caller elsewhere to hand
    /// the kernel a pointer of its own.
    ///
    /// # Safety
    ///
    /// `act` must point to a live, correctly initialised [`KernelSigaction`]
    /// for the duration of the call, and `signal` must be a signal number this
    /// process may take over.
    #[inline]
    unsafe fn rt_sigaction(signal: i32, act: *const KernelSigaction) -> i64 {
        let ret: i64;
        // SAFETY: the register assignment is the x86-64 Linux kernel calling
        // convention (see the module's Sources). `rcx` and `r11` are declared
        // clobbered because the `syscall` instruction itself overwrites them
        // with the return address and the saved flags; not declaring them is
        // the classic way to corrupt a caller. `nostack` is correct because
        // `syscall` neither pushes nor uses the red zone. The kernel reads the
        // four words `act` points at and nothing else — the third argument is
        // a null `oldact`, which `sigaction(2)` documents as "do not report
        // the previous disposition" rather than as a pointer it writes — and
        // that `act` is live and initialised is the caller's obligation,
        // stated above.
        unsafe {
            core::arch::asm!(
                "syscall",
                inlateout("rax") SYS_RT_SIGACTION => ret,
                in("rdi") i64::from(signal),
                in("rsi") act,
                in("rdx") 0u64,
                in("r10") SIGSETSIZE,
                lateout("rcx") _,
                lateout("r11") _,
                options(nostack)
            );
        }
        ret
    }

    /// Point one signal at [`handler`].
    ///
    /// Returns whether the kernel accepted it. It refuses `SIGKILL` and
    /// `SIGSTOP` and nothing else this module asks for, so a refusal means the
    /// host is not what the module documentation claims.
    fn install(signal: Signal) -> bool {
        let act = KernelSigaction {
            handler: handler as *const () as u64,
            flags: SA_RESTORER | SA_RESTART | SA_RESETHAND,
            restorer: rsemu_sigreturn as *const () as u64,
            // Nothing extra is blocked while the handler runs. It stores one
            // word; there is no critical section to protect.
            mask: 0,
        };
        // SAFETY: `act` is a live local of exactly the layout the kernel reads
        // and it outlives the call, and `signal` is one of `SHUTDOWN` — none
        // of which is `SIGKILL` or `SIGSTOP`. The kernel copies the structure
        // and keeps no pointer into it.
        let ret = unsafe { rt_sigaction(signal.0, &raw const act) };
        ret == 0
    }

    /// Install the handler for every signal in [`SHUTDOWN`].
    pub(super) fn arm() -> Arming {
        let mut all = true;
        for signal in SHUTDOWN {
            all &= install(*signal);
        }
        if all {
            Arming::Installed
        } else {
            Arming::Unavailable
        }
    }
}

// ---------------------------------------------------------------------------
// everywhere else
// ---------------------------------------------------------------------------

#[cfg(not(all(target_os = "linux", target_arch = "x86_64")))]
mod imp {
    //! No facility, and no pretending there is one.
    //!
    //! wasm has no signals at all; macOS and Windows have them but not through
    //! a `syscall` instruction with Linux's numbers, and reaching them would
    //! mean `libc`, which the dependency policy forbids. Saying so is the
    //! honest answer — [`Arming::Unavailable`] leaves every signal exactly the
    //! disposition it had before rsemu started.

    use super::Arming;

    pub(super) fn arm() -> Arming {
        Arming::Unavailable
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_named_signal_prints_its_name_and_an_unnamed_one_its_number() {
        assert_eq!(Signal::INT.to_string(), "SIGINT");
        assert_eq!(Signal::TERM.to_string(), "SIGTERM");
        assert_eq!(Signal::HUP.to_string(), "SIGHUP");
        assert_eq!(Signal(9).to_string(), "signal 9");
        assert_eq!(Signal(9).name(), "signal");
    }

    #[test]
    fn nothing_is_caught_until_something_is() {
        clear();
        assert_eq!(caught(), None);
        CAUGHT.store(Signal::TERM.0, Ordering::Relaxed);
        assert_eq!(caught(), Some(Signal::TERM));
        clear();
        assert_eq!(caught(), None);
    }

    /// A host with no facility must say so rather than claim a run is
    /// protected; a host with one must actually install, twice over, because
    /// `rsemu debug` reaches `run` after `debug` has already armed.
    #[test]
    fn arming_says_what_this_host_can_do() {
        let armed = arm();
        assert_eq!(armed, arm(), "arming is idempotent");
        #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
        assert!(armed.is_installed(), "x86-64 Linux has rt_sigaction");
        #[cfg(not(all(target_os = "linux", target_arch = "x86_64")))]
        assert!(!armed.is_installed());
        clear();
    }
}
