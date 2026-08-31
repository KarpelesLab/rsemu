//! The execution seam: how a core stops, and why.
//!
//! `ROADMAP.md` §4.6 says the core does not know what a CPU *is* beyond "give
//! it a budget, get back what it consumed". That is enough for level 1 — a
//! machine whose guest handles its own traps — and it is not enough for the
//! other two things a core has to do:
//!
//! * **Level 3** (§2, "Three levels of execution"): there is no guest kernel,
//!   so an `ecall` / `syscall` / `svc` must **leave the core** instead of
//!   vectoring to a handler inside the guest, and so must a fault, because
//!   there is nothing in the guest to take it.
//! * **Hardware acceleration** (§10): a KVM or HVF vCPU comes back out of its
//!   run ioctl for an MMIO access, a halt or a shutdown, and the run loop has
//!   to be told which.
//!
//! Those are the same shape, so this module is the one mechanism both use
//! rather than two that drift (§2.1, phase 5b). An interpreter implements
//! [`ExitingCore`] by consulting a mask before it vectors a trap; an accel
//! backend implements it by translating its own exit structure into an
//! [`Exit`].
//!
//! # The contract
//!
//! ```text
//!   mask says SYSCALL exits ──► core runs ──► hits `ecall`
//!                                   │
//!                          Run { consumed, exit: Some(Exit { SYSCALL, pc, .. }) }
//!                                   │
//!                          consumer services it, writes the result register
//!                                   │
//!                              run_to_exit again
//! ```
//!
//! Nothing here knows what a syscall *means*. The number, the arguments, the
//! errno and the file descriptor are an operating system's business and belong
//! to the consumer — §2.1 draws that line at *"Linux is not hardware"*, and
//! this module is the rsemu side of it.
//!
//! # Where the program counter is left
//!
//! [`Exit::pc`] is **always the address of the instruction that caused the
//! exit**. The core's own program counter is left at the **resume point**,
//! which differs by reason on purpose:
//!
//! | Reason | core `pc` afterwards | why |
//! | --- | --- | --- |
//! | [`SYSCALL`](ExitReason::SYSCALL) | *past* the instruction | resuming continues the program |
//! | [`BREAKPOINT`](ExitReason::BREAKPOINT) | *at* the instruction | a debugger reports the address it stopped on |
//! | [`FAULT`](ExitReason::FAULT) | *at* the instruction | map the missing page, resume, and the access happens |
//!
//! Leaving a syscall's program counter *past* the instruction is a deliberate
//! choice and the opposite of the obvious one. It makes **resuming**
//! unconditional and **retrying** explicit: a consumer that decides the call
//! must block calls `set_pc(exit.pc)` and the guest re-executes it, which is
//! visible, idempotent and impossible to do twice by accident. The other way
//! round — leaving the program counter on the instruction and advancing it as
//! a side effect of writing the return register — makes the common path carry
//! the hidden state, and a return written twice silently lands in the middle
//! of the next instruction. It is also what hardware already does: an
//! accelerated vCPU is past the trap by the time the host sees it, so this
//! convention is the one both engines can honour.
//!
//! # A syscall exit is a safe point
//!
//! Worth stating, because it is what makes a mid-process snapshot possible at
//! all (`ROADMAP.md` phase 5b's gate). At an exit the guest has *finished* an
//! instruction: the register file, the program counter and memory are a
//! complete architectural state, and the only thing outstanding is a value
//! somebody else owes it. A snapshot taken at an exit therefore needs no
//! notion of a half-executed instruction — it needs the consumer's own record
//! of what it was in the middle of doing, which is the consumer's to keep.

use crate::core::sched::{Budget, Consumed};

/// Why a core stopped before its budget ran out.
///
/// An **extensible enumeration** (CLAUDE.md, "Type conventions"): a
/// `#[repr(transparent)]` newtype with `pub const` variants, so a later accel
/// backend or a downstream crate can add a reason without that being a
/// breaking change and without a `_ => unreachable!()` anywhere.
///
/// Ids `1..32` are **maskable** — they can be named in an [`ExitMask`], which
/// is how a core is told to hand a trap out rather than vector it. Ids `32..`
/// are reserved for reasons a core raises whether it was asked to or not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct ExitReason(pub u16);

impl ExitReason {
    /// Not a reason. Reserved so a zeroed [`Exit`] is never mistaken for one.
    pub const NONE: ExitReason = ExitReason(0);

    /// The guest executed an environment call — `ecall`, `syscall`, `svc`,
    /// `int 0x80`.
    ///
    /// Level 3's whole reason for existing. The core's program counter is left
    /// *past* the instruction; see the module docs.
    pub const SYSCALL: ExitReason = ExitReason(1);

    /// The guest executed a breakpoint instruction — `ebreak`, `int3`, `bkpt`.
    ///
    /// The core's program counter is left *at* the instruction.
    pub const BREAKPOINT: ExitReason = ExitReason(2);

    /// An architectural exception with nowhere to go, because in level 3 there
    /// is no guest kernel to install a handler: an illegal instruction, an
    /// access fault, a misaligned access, a page fault.
    ///
    /// [`Exit::detail`] carries the architecture's own cause code,
    /// [`Exit::address`] the faulting address where the architecture reports
    /// one, and [`Exit::access`] what the guest was trying to do — which is
    /// the part a demand-paging or copy-on-write consumer needs and the only
    /// part of a fault that is the same on every architecture. The program
    /// counter is left *at* the instruction, so a consumer that maps the
    /// missing page simply resumes.
    pub const FAULT: ExitReason = ExitReason(3);

    /// The core has nothing to do until something outside it happens — `wfi`,
    /// `hlt`.
    ///
    /// Reserved for §10's accel backends; no interpreter in this crate raises
    /// it yet, and a level-3 guest has no interrupt that could end the wait.
    pub const HALT: ExitReason = ExitReason(4);

    /// An access the execution engine cannot perform itself and wants the
    /// framework to do — §10's `KVM_EXIT_MMIO`.
    ///
    /// Reserved for the accel backends. An interpreter never raises it,
    /// because an interpreter reaches
    /// [`AddressSpace`](crate::core::space::AddressSpace) directly.
    pub const MMIO: ExitReason = ExitReason(5);

    /// The guest asked its execution environment to stop — §10's
    /// `KVM_EXIT_SHUTDOWN`, a triple fault, an SBI `SYSTEM_RESET`.
    pub const SHUTDOWN: ExitReason = ExitReason(6);

    /// The engine cannot continue and it is not the guest's fault: an
    /// instruction the JIT declined to translate, a backend error.
    ///
    /// Distinct from [`FAULT`](ExitReason::FAULT) because a fault is something
    /// the guest did and this is something we did.
    pub const INTERNAL: ExitReason = ExitReason(7);

    /// Whether this reason can be named in an [`ExitMask`].
    #[must_use]
    pub const fn is_maskable(self) -> bool {
        self.0 >= 1 && self.0 < 32
    }

    /// A short name, for diagnostics. `None` for a reason this build does not
    /// know, which is exactly what an open enumeration has to allow.
    #[must_use]
    pub const fn name(self) -> Option<&'static str> {
        match self.0 {
            0 => Some("none"),
            1 => Some("syscall"),
            2 => Some("breakpoint"),
            3 => Some("fault"),
            4 => Some("halt"),
            5 => Some("mmio"),
            6 => Some("shutdown"),
            7 => Some("internal"),
            _ => None,
        }
    }
}

impl core::fmt::Display for ExitReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "exit reason #{}", self.0),
        }
    }
}

/// What the guest was doing to the address an exit names.
///
/// A real enum rather than an open one: an access is a read, a write or a
/// fetch, that list has been closed since the first memory management unit,
/// and a consumer genuinely wants exhaustiveness here — copy-on-write and
/// demand paging both branch on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Access {
    /// No address is involved: an illegal instruction, a syscall, a halt.
    #[default]
    None,
    /// A data read.
    Read,
    /// A data write. The one a copy-on-write consumer is looking for.
    Write,
    /// An instruction fetch.
    Execute,
}

impl Access {
    /// Whether this access would modify memory.
    #[must_use]
    pub const fn is_write(self) -> bool {
        matches!(self, Access::Write)
    }
}

/// Which reasons leave the core instead of being handled inside the guest.
///
/// A mask rather than a `bool` because one core serves three consumers and
/// they want different subsets: level 3 wants [`SYSCALL`](ExitReason::SYSCALL)
/// and [`FAULT`](ExitReason::FAULT), a debugger wants
/// [`BREAKPOINT`](ExitReason::BREAKPOINT), a level-1 machine wants none of
/// them — and a level-3 guest under a debugger wants all three.
///
/// An empty mask is the default, and is exactly the behaviour every core in
/// this crate had before this module existed: traps vector into the guest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(transparent)]
pub struct ExitMask(u32);

impl ExitMask {
    /// Nothing exits; every trap vectors into the guest. The default.
    pub const NONE: ExitMask = ExitMask(0);

    /// What a level-3 (`qemu-user`-shaped) consumer wants: environment calls
    /// and faults come out, because there is no guest kernel to take them.
    pub const USER: ExitMask = ExitMask::NONE
        .with(ExitReason::SYSCALL)
        .with(ExitReason::FAULT);

    /// The raw bits, for a snapshot or an FFI boundary.
    #[must_use]
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// A mask from raw bits. Bits naming a reason this build does not know are
    /// preserved rather than dropped: they may mean something to a newer core.
    #[must_use]
    pub const fn from_bits(bits: u32) -> ExitMask {
        ExitMask(bits)
    }

    /// The same mask with `reason` added. A reason that is not
    /// [maskable](ExitReason::is_maskable) is ignored.
    #[must_use]
    pub const fn with(self, reason: ExitReason) -> ExitMask {
        if reason.is_maskable() {
            ExitMask(self.0 | (1 << reason.0))
        } else {
            self
        }
    }

    /// The same mask with `reason` removed.
    #[must_use]
    pub const fn without(self, reason: ExitReason) -> ExitMask {
        if reason.is_maskable() {
            ExitMask(self.0 & !(1 << reason.0))
        } else {
            self
        }
    }

    /// Whether `reason` leaves the core.
    #[must_use]
    pub const fn contains(self, reason: ExitReason) -> bool {
        reason.is_maskable() && self.0 & (1 << reason.0) != 0
    }

    /// Whether nothing at all exits.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// Why a core stopped, and where.
///
/// Deliberately plain data — `Copy`, no allocation, no lifetime — because it
/// crosses a crate boundary on every syscall a level-3 guest makes, and
/// because an accel backend fills one in from a raw exit structure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Exit {
    /// What happened.
    pub reason: ExitReason,
    /// The address of the instruction that caused it — *not* where the core
    /// will resume. See the module docs for the difference.
    pub pc: u64,
    /// That instruction's encoded length in bytes, so a consumer can rewind to
    /// restart it, or step past it, without decoding anything.
    ///
    /// Zero when the exit is not attributable to one instruction.
    pub len: u8,
    /// What the guest was doing to [`address`](Exit::address).
    ///
    /// The architecture-independent half of a fault, and the half a
    /// demand-paging or copy-on-write consumer branches on.
    pub access: Access,
    /// Reason-specific, and **architecture-specific by construction**: the
    /// architecture's own trap cause code for [`FAULT`](ExitReason::FAULT) —
    /// `scause` on RISC-V, the vector number on x86 — and zero otherwise.
    ///
    /// A consumer that does not know the architecture compares it against
    /// nothing and uses [`access`](Exit::access) instead; one that does know
    /// already knows the encoding.
    pub detail: u64,
    /// Reason-specific: the faulting address for [`FAULT`](ExitReason::FAULT)
    /// where the architecture reports one, the accessed address for
    /// [`MMIO`](ExitReason::MMIO), zero otherwise.
    pub address: u64,
}

impl Exit {
    /// An exit with no address and no architectural detail.
    #[must_use]
    pub const fn new(reason: ExitReason, pc: u64, len: u8) -> Exit {
        Exit {
            reason,
            pc,
            len,
            access: Access::None,
            detail: 0,
            address: 0,
        }
    }

    /// The same exit carrying an architectural cause code.
    #[must_use]
    pub const fn with_detail(mut self, detail: u64) -> Exit {
        self.detail = detail;
        self
    }

    /// The same exit carrying an address and the access that reached it.
    #[must_use]
    pub const fn with_access(mut self, address: u64, access: Access) -> Exit {
        self.address = address;
        self.access = access;
        self
    }

    /// Where the core resumes if nothing rewrites its program counter.
    ///
    /// A convenience for the two callers who need to reason about the
    /// difference rather than take it on trust — the module table says which
    /// reason resumes where.
    #[must_use]
    pub const fn resume_pc(&self) -> u64 {
        if self.reason.0 == ExitReason::SYSCALL.0 {
            self.pc.wrapping_add(self.len as u64)
        } else {
            self.pc
        }
    }
}

/// What one call to [`ExitingCore::run_to_exit`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Run {
    /// Ticks of the core's own clock domain that were consumed — the same
    /// currency [`Runnable::run`] reports, so one core can be driven by the
    /// machine scheduler and by a level-3 run loop without two accounting
    /// schemes.
    ///
    /// [`Runnable::run`]: crate::core::sched::Runnable::run
    pub consumed: Consumed,
    /// Why it stopped, or `None` if it simply ran out of budget.
    ///
    /// `None` rather than a `BUDGET` reason because "the budget ended" is not
    /// something that happened *to the guest*: nothing is pending, no value is
    /// owed, and resuming is unconditional. It is also how a level-3 consumer
    /// preempts a thread — in ticks, which are deterministic, rather than
    /// against a host clock, which is not.
    pub exit: Option<Exit>,
}

impl Run {
    /// A run that used its budget without exiting.
    #[must_use]
    pub const fn completed(consumed: Consumed) -> Run {
        Run {
            consumed,
            exit: None,
        }
    }

    /// A run that stopped at `exit`.
    #[must_use]
    pub const fn exited(consumed: Consumed, exit: Exit) -> Run {
        Run {
            consumed,
            exit: Some(exit),
        }
    }
}

/// A core that can be run until it *exits* — stops at an instruction boundary
/// and says why.
///
/// This is the seam `ROADMAP.md` §2.1 names as rsemu's half of level 3, and it
/// is public API another crate builds on: a consumer holds an
/// `Arc<dyn ExitingCore>`, drives it in a loop, and services whatever comes
/// back.
///
/// # What is deliberately not here
///
/// **The syscall's number and arguments.** Which register carries them is an
/// ABI, an ABI belongs to an operating system, and a consumer that knows what
/// a syscall means already knows the architecture it is reading. A consumer
/// reaches those through the concrete core type it chose — `Hart::x` for
/// RISC-V — which is public for exactly that reason.
///
/// **A thread-local-storage register.** It looks architectural and is not: on
/// RISC-V `tp` is register `x4` and a pure calling convention, so there is
/// nothing for a core to expose. The same argument that keeps the syscall
/// number out keeps this out.
///
/// **Cloning a core.** A thread that starts as a copy of another is
/// [`Device::save`] into a chunk and [`Device::load`] into a fresh core, which
/// already exists, is already versioned, and is already round-trip tested.
///
/// [`Device::save`]: crate::core::device::Device::save
/// [`Device::load`]: crate::core::device::Device::load
///
/// `Send + Sync + Debug`, and every method takes `&self`, like the rest of the
/// device-facing surface: a core is shared and holds its state behind interior
/// mutability (`ROADMAP.md` §0). `Debug` is a supertrait for the same reason
/// [`CharDevice`](crate::host::chardev::CharDevice) has one — anything holding
/// a collection of these has to be able to derive its own.
pub trait ExitingCore: Send + Sync + core::fmt::Debug {
    /// Which reasons currently leave the core.
    fn exit_mask(&self) -> ExitMask;

    /// Set which reasons leave the core.
    ///
    /// Takes effect at the next instruction, not retroactively. It is
    /// *configuration* rather than architectural state: a reset does not clear
    /// it and a snapshot does not carry it, because the consumer that wanted
    /// the mask is the one still there after a restore.
    fn set_exit_mask(&self, mask: ExitMask);

    /// Run until the budget is exhausted or something in the mask happens.
    ///
    /// Consuming less than the budget without exiting is legitimate — the same
    /// contract [`Runnable::run`] has. Consuming *more* is a bug.
    ///
    /// [`Runnable::run`]: crate::core::sched::Runnable::run
    fn run_to_exit(&self, budget: Budget) -> Run;

    /// Where the core will resume.
    fn pc(&self) -> u64;

    /// Set where the core will resume.
    ///
    /// How a consumer retries an interrupted call (`set_pc(exit.pc)`), steps
    /// over a breakpoint, or enters a signal handler.
    fn set_pc(&self, pc: u64);

    /// The stack pointer.
    ///
    /// On the trait, unlike every other register, because starting a guest
    /// thread means setting exactly two things and this is the other one.
    fn sp(&self) -> u64;

    /// Set the stack pointer.
    fn set_sp(&self, sp: u64);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mask_holds_the_reasons_put_in_it() {
        let mask = ExitMask::NONE
            .with(ExitReason::SYSCALL)
            .with(ExitReason::BREAKPOINT);
        assert!(mask.contains(ExitReason::SYSCALL));
        assert!(mask.contains(ExitReason::BREAKPOINT));
        assert!(!mask.contains(ExitReason::FAULT));
        assert!(!mask.is_empty());

        let mask = mask.without(ExitReason::SYSCALL);
        assert!(!mask.contains(ExitReason::SYSCALL));
        assert!(mask.contains(ExitReason::BREAKPOINT));
    }

    #[test]
    fn the_user_mask_is_syscalls_and_faults() {
        assert!(ExitMask::USER.contains(ExitReason::SYSCALL));
        assert!(ExitMask::USER.contains(ExitReason::FAULT));
        assert!(!ExitMask::USER.contains(ExitReason::BREAKPOINT));
    }

    #[test]
    fn an_empty_mask_is_the_default() {
        assert_eq!(ExitMask::default(), ExitMask::NONE);
        assert!(ExitMask::default().is_empty());
        assert!(!ExitMask::NONE.contains(ExitReason::SYSCALL));
    }

    #[test]
    fn reason_zero_is_never_a_mask_bit() {
        // Bit 0 must stay clear, or a zeroed mask would claim to contain
        // `NONE` and every `contains` would need a guard.
        assert!(!ExitReason::NONE.is_maskable());
        assert_eq!(ExitMask::NONE.with(ExitReason::NONE), ExitMask::NONE);
        assert!(!ExitMask::from_bits(u32::MAX).contains(ExitReason::NONE));
    }

    #[test]
    fn an_unknown_reason_survives_a_mask_round_trip() {
        // Bits this build does not understand are preserved rather than
        // dropped: an open enumeration whose mask silently forgets is not
        // open.
        let bits = ExitMask::USER.bits() | (1 << 20);
        assert_eq!(ExitMask::from_bits(bits).bits(), bits);
        assert!(ExitMask::from_bits(bits).contains(ExitReason(20)));
    }

    #[test]
    fn reasons_above_the_mask_width_are_not_maskable() {
        assert!(!ExitReason(32).is_maskable());
        assert!(!ExitReason(u16::MAX).is_maskable());
        assert_eq!(ExitMask::NONE.with(ExitReason(40)), ExitMask::NONE);
    }

    #[test]
    fn a_reason_names_itself_or_admits_it_cannot() {
        assert_eq!(ExitReason::SYSCALL.name(), Some("syscall"));
        assert_eq!(ExitReason(999).name(), None);
        assert_eq!(alloc::format!("{}", ExitReason::FAULT), "fault");
        assert_eq!(alloc::format!("{}", ExitReason(999)), "exit reason #999");
    }

    #[test]
    fn an_exit_carries_its_detail_address_and_access() {
        let exit = Exit::new(ExitReason::FAULT, 0x1000, 4)
            .with_detail(13)
            .with_access(0xdead_beef, Access::Write);
        assert_eq!(exit.reason, ExitReason::FAULT);
        assert_eq!(exit.pc, 0x1000);
        assert_eq!(exit.len, 4);
        assert_eq!(exit.detail, 13);
        assert_eq!(exit.address, 0xdead_beef);
        assert!(exit.access.is_write());
        assert!(!Access::Read.is_write());
        assert_eq!(Access::default(), Access::None);
    }

    #[test]
    fn only_a_syscall_resumes_past_its_instruction() {
        assert_eq!(
            Exit::new(ExitReason::SYSCALL, 0x1000, 4).resume_pc(),
            0x1004
        );
        assert_eq!(Exit::new(ExitReason::FAULT, 0x1000, 4).resume_pc(), 0x1000);
        assert_eq!(
            Exit::new(ExitReason::BREAKPOINT, 0x1000, 2).resume_pc(),
            0x1000
        );
    }

    #[test]
    fn a_completed_run_has_no_exit() {
        let run = Run::completed(Consumed::new(7));
        assert_eq!(run.consumed.ticks, 7);
        assert!(run.exit.is_none());

        let run = Run::exited(Consumed::new(3), Exit::new(ExitReason::SYSCALL, 0x20, 4));
        assert_eq!(run.consumed.ticks, 3);
        assert_eq!(run.exit.unwrap().reason, ExitReason::SYSCALL);
    }
}
