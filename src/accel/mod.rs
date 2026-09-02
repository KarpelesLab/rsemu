//! Hardware acceleration: execution engines that are not ours.
//!
//! `ROADMAP.md` §10 and phase 7. When the guest ISA is the host ISA, guest
//! code can run on the host's own silicon instead of being interpreted, and
//! the emulator's job narrows to *being the machine around it* — the memory
//! map, the devices, the interrupt controller, the clock.
//!
//! # The seam, and whether it existed
//!
//! §4.6 promises that execution engines are interchangeable, and asks a new
//! backend to fit the existing shape rather than invent a third. The honest
//! answer to *"does that seam exist?"* is **half**, and the useful half is the
//! one this module needs:
//!
//! * [`core::exec::ExitingCore`](crate::core::exec::ExitingCore) is exactly the
//!   right seam and was written with this backend in mind — its own module
//!   documentation says *"an accel backend implements it by translating its own
//!   exit structure into an [`Exit`](crate::core::exec::Exit)"*, and
//!   [`ExitReason::MMIO`](crate::core::exec::ExitReason::MMIO),
//!   [`HALT`](crate::core::exec::ExitReason::HALT) and
//!   [`SHUTDOWN`](crate::core::exec::ExitReason::SHUTDOWN) are already reserved
//!   for §10. [`kvm::Vcpu`] implements it and needed no change to it.
//! * [`core::sched::Runnable`](crate::core::sched::Runnable) is the scheduler's
//!   side and likewise fits: a budget in, a [`Consumed`](crate::core::sched::Consumed)
//!   out.
//!
//! What does **not** exist yet, said plainly rather than papered over:
//!
//! * **There is no `Cpu` trait.** §4.6 shows one (`run`/`interrupt`/`regs`/`mmu`)
//!   and the crate does not have it; each core is a [`Device`](crate::core::Device)
//!   that happens to be a [`Runnable`](crate::core::sched::Runnable), with its
//!   register file reached through its own concrete type. So "the same seam the
//!   interpreter uses" is two traits and a convention, not one trait.
//! * **`engine = "kvm"` is not a machine-file property yet.** Every core in the
//!   crate accepts `engine` and rejects anything but `"interp"`, in
//!   `cpu::x86`'s property reader *and* in its validator schema. So a *board*
//!   still cannot ask to be accelerated; a **host** can, through
//!   [`Bindings::replace`](crate::machine::Bindings::replace), and [`cpu`] is
//!   the device class it replaces `cpu.x86` with. Closing the remaining gap is
//!   four lines in `src/cpu/x86/mod.rs` — `"kvm"` added to the property
//!   reader's `or_enum` list and to `schema_for`'s `values` — and this module
//!   deliberately does not make them.
//! * **[`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel) is
//!   unimplemented in the scheduler**, which refuses it with
//!   `SchedError::ModeUnimplemented`. That refusal is load-bearing and is left
//!   standing, and it is what an accelerated board actually wants: under
//!   [`Parallel`](crate::core::sched::ThreadingMode::Parallel) virtual time is
//!   still the emulated grid, so a guest that runs for a host millisecond
//!   between exits advances the board's clock by one budget, not by a
//!   millisecond.
//!
//! What used to be on that list and no longer is:
//!
//! * **An interrupt reaches an accelerated guest on a board.** The vector comes
//!   from the board's 8259A or local APIC on an *acknowledge cycle*
//!   ([`IntAck`](crate::core::wire::IntAck)), and [`cpu::AccelCpu`] is the CPU
//!   device on the receiving end of that wire.
//!   [`kvm::Vcpu::run_until_exit_with`] runs the cycle between guest entries,
//!   outside every lock this module holds, and injects what the controller
//!   answers with.
//! * **A second processor is started by the guest's own `INIT` and Start-Up.**
//!   [`LocalController`](crate::core::wire::LocalController) is asked once per
//!   slice and the restart sequences run on the interpreter the device carries,
//!   which is how both engines get one implementation of *Intel SDM* Vol 3A
//!   Table 9-1 rather than two.
//!
//! # A board, not a harness
//!
//! [`cpu`] is where a board's processor lives: an accelerated `cpu.x86` with
//! the interpreter's own properties, pins, acknowledge cycle and snapshot
//! chunk, whose guest instructions run on the host. `tests/kvm_smp.rs` builds
//! `machines/pc-apic.machine` verbatim with two of them and lets the guest
//! start the second one.
//!
//! [`board`] closes the gap phase 7 named: it takes an
//! [`AddressSpace`](crate::core::space::AddressSpace) — a real one, from a real
//! `.machine` file — and installs everything in its flat view that can be a
//! memory slot, RAM read/write and ROM read-only. That became possible when
//! [`RamStore`](crate::core::space::RamStore) and
//! [`RomStore`](crate::core::space::RomStore) gained host-page-aligned
//! allocations, which is the one `src/core/` change this phase needed: before
//! it, a board's declared `ram` had allocation alignment **1** and
//! `KVM_SET_USER_MEMORY_REGION` rejected it outright.
//!
//! There is consequently **no accel-private RAM type any more**. An earlier
//! round had one, `accel::mem::HostPages`, an anonymous `mmap` with the same
//! byte-offset API; it existed only because `RamStore` could not be a slot,
//! and keeping it would have meant two kinds of guest memory and boards that
//! were accelerable and boards that were not.
//!
//! # Determinism
//!
//! An accelerated run is **not reproducible**, and nothing here pretends
//! otherwise. [`kvm::Vcpu::into_runnable`] refuses a deterministic
//! [`ThreadingMode`](crate::core::sched::ThreadingMode) rather than trusting a
//! caller to know, which is the same structural refusal
//! `Machine::set_recorder` makes.
//!
//! # `unsafe`, and where it is
//!
//! Two of `ROADMAP.md` §0's six sanctioned subsystems meet here, and this
//! module uses **both and only those two**:
//!
//! | Site | Where | What |
//! | --- | --- | --- |
//! | the raw-syscall accel backends | [`sys`] | one `asm!` block, plus the wrappers that establish what each kernel entry point needs |
//! | the RAM host-pointer fast path | [`sys::Mapping::cells`] | one `from_raw_parts`, for the `kvm_run` page |
//!
//! No seventh site is created and neither existing one is widened. Every block
//! carries a `// SAFETY:` comment naming its invariant and who upholds it.
//!
//! Worth noting what page-aligning `RamStore` did *not* cost: nothing.
//! `core::space` reports a host address as a `u64` obtained from
//! `Vec::as_ptr`, which is safe, and the only code that dereferences it is a
//! kernel that was handed it — so the store that a guest's hardware writes
//! directly still contains **no `unsafe` at all**, and this subsystem's count
//! of files that opt back in is unchanged.
//!
//! # Portability
//!
//! The whole module is `cfg`-gated to **Linux on x86-64** in `lib.rs`: it is a
//! `syscall` instruction and a set of `ioctl` numbers, neither of which means
//! anything on another target. A `wasm32-*` or macOS build with the feature on
//! therefore gets a crate with no `accel` at all, rather than one that has it
//! and refuses. `host/`, `jit/` and `accel/` may use `std` (`CLAUDE.md`); this
//! module in fact needs none of it, because the kernel is reached by syscall —
//! the feature implies `std` only to keep it out of the `no_std` gate.

use core::fmt;

pub mod board;
pub mod kvm;
pub mod sys;

#[cfg(feature = "cpu-x86")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-x86")))]
pub mod cpu;

#[cfg(feature = "cpu-x86")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-x86")))]
pub mod state;

/// Everything an acceleration backend can fail at.
///
/// A module-local error rather than a bare
/// [`Error`](crate::core::Error) variant per case, because the distinction
/// callers actually make is *"this host has no KVM, skip"* against everything
/// else — and that distinction is worth a variant of its own. It converts into
/// the crate error for a caller that just wants `?`.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum AccelError {
    /// There is no usable `/dev/kvm`: it does not exist, or this user cannot
    /// open it.
    ///
    /// **Not a failure.** Every test in this subsystem treats it as a reason to
    /// skip, and a front end should treat it as a reason to fall back to the
    /// interpreter.
    Unavailable(sys::Errno),
    /// A system call failed.
    Sys {
        /// What was being attempted, for the message.
        what: &'static str,
        /// What the kernel said.
        errno: sys::Errno,
    },
    /// The host's KVM is there but cannot do what is being asked of it.
    Unsupported(&'static str),
    /// A routed exit hit a bus fault: the guest touched an address no device
    /// answers, or answered on terms the access did not meet.
    Bus {
        /// The guest address, or the port number for a port access.
        addr: u64,
        /// Why the address space refused.
        err: crate::core::BusError,
    },
    /// An accelerated core was asked to run in a mode that claims
    /// reproducibility, which it cannot provide.
    Nondeterministic(crate::core::sched::ThreadingMode),
}

impl fmt::Display for AccelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AccelError::Unavailable(errno) => {
                write!(f, "no usable /dev/kvm on this host ({errno})")
            }
            AccelError::Sys { what, errno } => write!(f, "{what}: {errno}"),
            AccelError::Unsupported(what) => f.write_str(what),
            AccelError::Bus { addr, err } => {
                write!(f, "a guest access at {addr:#x} was refused: {err}")
            }
            AccelError::Nondeterministic(mode) => write!(
                f,
                "an accelerated CPU cannot run in `{mode}` mode: hardware execution is not \
                 reproducible, and a state hash taken over one would be meaningless"
            ),
        }
    }
}

impl AccelError {
    /// Whether this means *"there is no KVM here"* rather than *"KVM went
    /// wrong"*.
    #[must_use]
    pub const fn is_unavailable(&self) -> bool {
        matches!(self, AccelError::Unavailable(_))
    }
}

impl From<AccelError> for crate::core::Error {
    fn from(e: AccelError) -> crate::core::Error {
        use alloc::string::ToString;
        crate::core::Error::Accel(e.to_string())
    }
}

/// The result type this module uses.
pub type AccelResult<T> = Result<T, AccelError>;

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn unavailable_is_distinguishable_from_broken() {
        let missing = AccelError::Unavailable(sys::Errno::ENOENT);
        assert!(missing.is_unavailable());
        assert!(missing.to_string().contains("/dev/kvm"));

        let broken = AccelError::Sys {
            what: "KVM_RUN",
            errno: sys::Errno::EINVAL,
        };
        assert!(!broken.is_unavailable());
        assert_eq!(broken.to_string(), "KVM_RUN: EINVAL");
    }

    #[test]
    fn a_deterministic_mode_is_refused_with_a_reason_a_person_can_read() {
        use crate::core::sched::ThreadingMode;
        let e = AccelError::Nondeterministic(ThreadingMode::Deterministic);
        let text = e.to_string();
        assert!(text.contains("deterministic"));
        assert!(text.contains("not"));
    }

    #[test]
    fn an_accel_error_converts_into_the_crate_error() {
        let e: crate::core::Error = AccelError::Unsupported("no").into();
        assert!(matches!(e, crate::core::Error::Accel(_)));
    }
}
