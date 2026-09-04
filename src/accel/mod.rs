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
//! * **`engine = "kvm"` is not a machine-file property, and is not going to
//!   be.** Every core in the crate accepts `engine` and rejects anything but
//!   the engines it implements, in `cpu::x86`'s property reader *and* in its
//!   validator schema, and adding `"kvm"` to both lists is four lines that
//!   should not be written. `engine` chooses between implementations of the
//!   **same processor** — `interp` and `jit` answer `CPUID` identically and
//!   `tests/x86_engines.rs` asserts their state hashes match at every
//!   checkpoint — while a vCPU answers from the host's silicon, cannot be
//!   replayed, and exists only on Linux/x86-64. A board file naming it would
//!   be a board that does not build on a Mac, and a board file is meant to be
//!   portable text. So acceleration stays a **host** decision, made through
//!   [`Bindings::replace`](crate::machine::Bindings::replace) with [`cpu`] as
//!   the class that displaces `cpu.x86` — and `rsemu run <board> --accel kvm`
//!   is that call from a command line (`src/bin/rsemu.rs`).
//!
//! What used to be on that list and no longer is:
//!
//! * **A user who is not writing a test can ask for it.** `--accel kvm` opens
//!   the backend before the machine is built and installs it; it implies
//!   [`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel),
//!   because an accelerated board whose clocks are not slaved to the wall is
//!   the boot failure **Time** below is about, and it refuses a `--threading`
//!   that was actually typed rather than overruling it silently.
//! * **A machine realized in `Accel` is handed a host clock.** That mode has
//!   no other source of elapsed time, so a machine without one failed every
//!   round with `SchedError::NoHostClock` and every caller had to know the
//!   rule. `machine::realize` now installs `host::clock::MonotonicClock` when
//!   the mode asks for one, and `Machine::set_host_clock` still overrides it.
//! * **Two accelerated processors bring each other up.**
//!   `machines/q35-linux-smp.machine` is `q35-linux` with a second `cpu.x86`,
//!   a second `pc.lapic` naming it, and `0xfee00000` decoding to the APIC
//!   `window` — and a stock Gentoo 6.6.67 kernel prints `smp: Brought up 1
//!   node, 2 CPUs` on it in 1.7 seconds of wall clock
//!   (`tests/kvm_q35_linux_smp.rs`). That board was committed as a
//!   *reproduction* of a machine that stopped 126 console lines in, and what
//!   was in the way was **Time**, exactly as its own note said.
//! * **[`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel) is
//!   implemented.** Virtual time is read off the host clock, so a guest that
//!   runs for a host millisecond between exits advances the board's clocks by
//!   a millisecond. That was *"the single thing standing between this backend
//!   and an unmodified guest"*, and **Time** below is what it took.
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
//! * **A stock Linux kernel boots to userspace on a board's own machine file.**
//!   `tests/kvm_q35_linux.rs` runs `machines/q35-linux.machine` with a Gentoo
//!   6.6.67 `bzImage` in its socket: the kernel enumerates the PCI bus, binds
//!   the NVM Express driver, mounts an initramfs, reaches a shell and reads a
//!   signature off the emulated namespace — **in about three seconds of wall
//!   clock against the interpreted run's sixteen minutes**, on the board's own
//!   command line with nothing added to it. Two things had to exist for that
//!   and both are in this module: a `CPUID` table ([`kvm::board_cpuid`]) and an
//!   engine that can execute what hardware cannot fetch ([`cpu::AccelCpu`]'s
//!   interpreter fallback). The third was **Time**, below, and it was not in
//!   this module at all.
//!
//! # The interrupt controllers stay in userspace, and that is the decision
//!
//! KVM offers three arrangements, and this backend deliberately takes the one
//! that looks like the most work:
//!
//! | arrangement | who owns the local APIC, I/O APIC and 8259A |
//! | --- | --- |
//! | `KVM_CREATE_IRQCHIP` | the kernel, all three, plus `KVM_CREATE_PIT2` for the 8254 |
//! | `KVM_CAP_SPLIT_IRQCHIP` | the kernel owns the local APIC; userspace owns the rest |
//! | **none** — what this is | **the board**: `pc.lapic`, `pc.ioapic`, `pc.pic`, `pc.imcr`, `pc.pit`, `pc.hpet` and the wires between them |
//!
//! **Because the board is the machine.** On `machines/q35-linux.machine` the
//! interrupt path is not a detail a hypervisor could stand in for: the MADT is
//! *generated from the devices that are actually there*, `_PRT` from the `PIRQ`
//! routers the bridge is holding, IRQ0 arrives through a multiplexer built out
//! of one `wire.not` and five `wire.and`s because that is what the HPET's
//! `LEG_RT_CNF` bit is, and the IMCR decides which of two drivers owns `INTR`.
//! An in-kernel local APIC would make the guest's APIC and the board's APIC two
//! different objects — the table would describe one and the interrupts would
//! come from the other, `Machine::save` would snapshot the wrong one, the
//! monitor and the debugger would read the wrong one, and every route would
//! have to be transcribed a second time into KVM's GSI routing table.
//!
//! Three consequences that are the point rather than side effects:
//!
//! * **One implementation is tested twice.** The I/O APIC's redirection-entry
//!   polarity — level-triggered and active low, PCI Local Bus 3.0 §2.2.6 —
//!   was wrong until this week, in `dev::pc::ioapic`. Under an in-kernel
//!   irqchip that model would be dead code on an accelerated board and the
//!   two engines would have stopped testing the same thing, which is precisely
//!   what §4.6 asks them to do.
//! * **The console comparison means something.** An accelerated run and an
//!   interpreted run of one board can be compared line for line only if the
//!   interrupt they are both waiting on came out of the same device.
//! * **The snapshot needs no second architectural-state model.** A device's
//!   chunk is the same chunk under either engine, so [`state`] has only the
//!   *core* to carry. `KVM_GET_LAPIC`/`KVM_SET_LAPIC` translated into
//!   `pc.lapic`'s chunk would be a second one — and `ROADMAP.md` phase 7 names
//!   "LAPIC/x2APIC state" as part of the engine-independent model precisely
//!   because it is not free.
//!
//! **What it costs, plainly.** Every APIC and I/O APIC access is an exit,
//! including the end-of-interrupt at the close of every interrupt; there is no
//! `irqfd`, no `ioeventfd`, no posted interrupt and no paravirtual EOI; and the
//! local APIC's timer is a device on the board's virtual time rather than a
//! host `hrtimer`. A whole `q35-linux` boot to a shell measures about 132,000
//! guest entries, of which roughly 69,000 are port accesses and 63,000 are
//! MMIO — and it still takes three seconds. The split irqchip is the natural
//! upgrade if that ever stops being true, and it is *only* available at the
//! price named above.
//!
//! It is also why [`kvm::board_cpuid`] clears the x2APIC and TSC-deadline bits:
//! the board's local APIC is a device that implements neither, and a processor
//! must not advertise what the machine around it has not got.
//!
//! # Time, which is where this used to be unfinished
//!
//! **Virtual time did not advance while a vCPU was inside `KVM_RUN`.** A
//! scheduler round ends when every runnable returns; an accelerated processor
//! returns when the *guest* exits; so a guest that ran without exiting held
//! the round and the board's clocks stood still for as long as it took. A
//! delay loop is exactly such a guest, and a kernel is full of them.
//!
//! Two things a stock Linux kernel does were that one fact, and both are
//! written out in `tests/kvm_q35_linux.rs`: `hpet_counting()` read the HPET
//! counter twice within one round and found it unmoved, and
//! `timer_irq_works()` waited out a delay loop between two reads of `jiffies`
//! and saw no tick, which panics `check_timer()` unless the command line says
//! `no_timer_check`. The visible symptom in between was that a kernel
//! calibrating its time-stamp counter against a board timer measured the
//! *host's* TSC against *virtual* microseconds and concluded it was on a
//! **176,273 MHz** processor, so every delay it computed was wrong by
//! forty-four times.
//!
//! That word is off the command line and that number is now about 3,993 MHz on
//! a 3,993,994 kHz host — it moves in the last digits run to run, because it is
//! a measurement rather than a constant, which is the point. Two halves, one on each side of this module's floor:
//!
//! * **[`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel), in
//!   `core::sched`** — §4.2's *"CPUs run in hardware and virtual time is
//!   slaved to the host clock; the scheduler becomes a deadline service"*. A
//!   round's elapsed virtual time is read off the injected
//!   [`HostClock`](crate::core::sched::HostClock) rather than taken from what
//!   the runnables reported, because a runnable that reports its whole budget
//!   — which is all an accelerated core *can* report — makes the board's
//!   clocks run at a rate set by the quantum. The seam was already there;
//!   nothing in the run path called it.
//! * **A preemption interval, in [`preempt`]** — because slaving time to the
//!   wall is not enough on its own. A guest spinning in `RDTSC` and `PAUSE`
//!   takes no exits at all, so the round cannot end and the tick that is *due*
//!   cannot be delivered. Bounding that needs the kernel to interrupt the
//!   thread, and the only mechanism that does is a signal. [`preempt`] works
//!   through every alternative — `immediate_exit`, the exit flag, an `ioctl`
//!   from another thread, VMX's notify window — and why each fails, and why
//!   the rule that forbids a signal in the *safe-point protocol* does not
//!   reach a preemption timer in a Linux-only module.
//!
//! What it costs is stated where it is set: [`cpu`]'s slice under this mode is
//! **one guest exit long**, so every access a guest makes to a device sees the
//! wall clock as of that access, at the price of a scheduler round per exit —
//! about 130,000 of them across a `q35-linux` boot, in three seconds.
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
//! # What a user gives up by asking for this
//!
//! Said in one place, because "faster" is the only half anyone reads.
//!
//! | given up | why, and what it means in practice |
//! | --- | --- |
//! | **Reproducibility** | instruction timing, the instant an interrupt is taken and the TSC all come from the host. [`kvm::Vcpu::into_runnable`] and [`cpu::AccelCpus::open`] *refuse* a deterministic [`ThreadingMode`](crate::core::sched::ThreadingMode) rather than trusting a caller to know. So: no record/replay, no rewind, and no state hash that would mean anything. |
//! | **The board's declared timing** | see **Time** above. Under [`ThreadingMode::Accel`](crate::core::sched::ThreadingMode::Accel) the board's clocks track the *host's* wall clock rather than the frequencies the machine file names, so a guest measuring one clock against another gets consistent answers and a guest measuring anything against the machine file gets host time. That is a real trade and not the same as the old one: guest-visible timing is now *modelled by the host*, where before it was not modelled at all. |
//! | **The processor the machine file asked for** | `cpu::x86` answers `CPUID` from its declared [`Variant`](crate::cpu::x86::Variant); an accelerated one answers from the host's silicon, filtered by [`kvm::board_cpuid`]. A guest takes different paths through its own feature dispatch under the two engines and prints a different model name. |
//! | **The A20 gate** | modelled inside the interpreter for want of anywhere else to put it, so a guest's *hardware* accesses are not masked ([`cpu`]). A board that closes the gate and relies on the megabyte wrap is a board to interpret. |
//! | **Three fields of the snapshot** | a pending `NMI`, the `NMI` level and `halted` are this module's rather than the shell's, for want of a public setter; [`cpu`] names each and the one-line change that would end it. |
//! | **A hard bound on *stopping*** | a stop-the-world request reaches a guest only at its next exit, because the safe-point protocol is a flag the guest's own exits are checked against ([`kvm`]). [`preempt`]'s interval bounds how long an *entry* lasts, which is a different question and does not make a stop synchronous. |
//!
//! What is *not* given up: the memory map, the devices, the interrupt
//! controllers, the wires, the ACPI tables, the snapshot format and the
//! console. Those are the board, and they are the same board.
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
//! Three of `ROADMAP.md` §0's seven sanctioned subsystems meet here, and this
//! module uses **those three and no others**:
//!
//! | Site | Where | What |
//! | --- | --- | --- |
//! | the raw-syscall accel backends | [`sys`] | one `asm!` block, plus the wrappers that establish what each kernel entry point needs — the `ioctl`s, the maps, and the interval timer [`preempt`] arms |
//! | the RAM host-pointer fast path | [`sys::Mapping::cells`] | one `from_raw_parts`, for the `kvm_run` page |
//! | the host signal disposition | [`crate::host::signal`], *not here* | [`preempt`] needs `Signal::PREEMPT` to have a handler, and asks the one module in the tree that installs one |
//!
//! **No eighth subsystem is created and no existing one is widened.**
//! [`preempt`] itself contains no `unsafe` at all: it is a `timer_create` and
//! two `timer_settime`s through [`sys`]'s existing wrappers, and a disposition
//! installed by a `host/` module that already had that job. Every block
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
pub mod preempt;
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
