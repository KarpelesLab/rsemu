//! The interpreter.
//!
//! # Delay slots
//!
//! Every SH-4A branch except `BT` and `BF` is *delayed*: the instruction
//! after it runs before control moves (manual §3.1). The manual is explicit
//! that "a delayed branch instruction and its associated delay slot
//! instruction are indivisible" (§5.6.4) — so no interrupt is accepted between
//! them, and an exception in the slot reports the **branch's** address in
//! `SPC`, which is how the return from it re-executes the branch and the slot
//! together.
//!
//! [`State::delay`] is that pairing. A delayed branch computes its target —
//! from the registers *as they are at the branch*, which matters when the slot
//! changes the register (§10.1.57, note) — stores it there and moves the PC to
//! the slot. The next step sees the pending target, runs the slot instruction
//! under the slot rules, and only then jumps. `BT/S` and `BF/S` that are *not*
//! taken still make the next instruction a delay slot (§3.1), so they store
//! the fall-through address as the target.
//!
//! # Exceptions
//!
//! Section 5 sorts exceptions into *re-execution* type (the instruction did
//! not happen: `SPC` is its own address, or the branch's in a slot),
//! *completion* type (`TRAPA`: `SPC` is the next instruction), and the
//! resets. [`Trap`] carries the type, the code that lands in `EXPEVT` and the
//! vector offset, decided where the fault is detected. Every instruction does
//! all its memory accesses **before** it writes a register, so a fault leaves
//! the register file exactly as it was and re-execution is correct — `MAC.L`
//! and the post-increment forms are written in that order on purpose.
//!
//! An exception while `SR.BL` is set is a manual reset (§5.5.3), and a TLB
//! multiple hit is a reset too (§7.5.1).
//!
//! # One cycle is one bus access
//!
//! The manual's cycle column is a pipeline's issue rate, not a fact about the
//! bus, and this core does not model the pipeline. A cycle is charged
//! *because* an access happened — every instruction fetch, every load and
//! store, each longword of a store-queue flush — which is the accounting
//! CLAUDE.md asks for ("CPU cores").
//!
//! # Sources
//!
//! *SH-4A Software Manual*, REJ09B0003-0150. The operation of each instruction
//! is from its entry in section 10, cited at the arms where the manual's C
//! description decides something non-obvious; the exception model is section
//! 5; the address map and translation are section 7.

use crate::core::exec::{Access as ExitAccess, Exit, ExitMask, ExitReason};
use crate::core::space::{AccessPurpose, AddressSpace, MemAttrs};
use crate::core::value::{Endian, Width};

use super::isa::{self, Op};
use super::mmu::{
    self, Ctl, Hit, ItlbEntry, Mmu, PmbEntry, UtlbEntry, lrui_touch, lrui_victim, page_mask,
    pmb_mask, reg,
};
use super::{Config, Lines};

/// `SR` bits (manual §2.2.4).
pub mod sr {
    /// The condition bit.
    pub const T: u32 = 1 << 0;
    /// The `MAC` saturation bit.
    pub const S: u32 = 1 << 1;
    /// The interrupt mask level.
    pub const IMASK: u32 = 0xf << 4;
    /// The division `Q` bit.
    pub const Q: u32 = 1 << 8;
    /// The division `M` bit.
    pub const M: u32 = 1 << 9;
    /// FPU disable.
    pub const FD: u32 = 1 << 15;
    /// Exception and interrupt block.
    pub const BL: u32 = 1 << 28;
    /// Register bank select (privileged mode only).
    pub const RB: u32 = 1 << 29;
    /// Privileged mode.
    pub const MD: u32 = 1 << 30;
    /// What `LDC Rm,SR` keeps (§10.2.4: `SR = R[m] & 0x700083F3`).
    pub const WRITABLE: u32 = 0x7000_83f3;
    /// The reset value: `MD`, `RB`, `BL` and `IMASK = 1111`, `FD = 0`
    /// (table 2.1).
    pub const RESET: u32 = MD | RB | BL | IMASK;
}

/// `FPSCR` bits (manual §6.3.2).
pub mod fpscr {
    /// Round to zero (`RM = 01`); `00` is round to nearest and the other two
    /// are reserved.
    pub const RM: u32 = 0b11;
    /// The flag field.
    pub const FLAG: u32 = 0x1f << 2;
    /// The enable field.
    pub const ENABLE: u32 = 0x1f << 7;
    /// The cause field, `E` included.
    pub const CAUSE: u32 = 0x3f << 12;
    /// Denormalised numbers are zero.
    pub const DN: u32 = 1 << 18;
    /// Double precision.
    pub const PR: u32 = 1 << 19;
    /// Pair transfers.
    pub const SZ: u32 = 1 << 20;
    /// Register bank.
    pub const FR: u32 = 1 << 21;
    /// What `LDS` keeps (§10.2.5: `FPSCR_MASK 0x003FFFFF`).
    pub const WRITABLE: u32 = 0x003f_ffff;
    /// The reset value (table 2.1): `DN` set and round to zero.
    pub const RESET: u32 = 0x0004_0001;
}

/// Exception codes, as they land in `EXPEVT` and `INTEVT` (manual table 5.3).
pub mod exc {
    /// Power-on reset.
    pub const POWER_ON: u32 = 0x000;
    /// Manual reset — and any exception taken with `SR.BL` set.
    pub const MANUAL_RESET: u32 = 0x020;
    /// TLB miss, read or fetch.
    pub const TLB_MISS_READ: u32 = 0x040;
    /// TLB miss, write.
    pub const TLB_MISS_WRITE: u32 = 0x060;
    /// Initial page write.
    pub const INITIAL_WRITE: u32 = 0x080;
    /// TLB protection violation, read or fetch.
    pub const TLB_PROT_READ: u32 = 0x0a0;
    /// TLB protection violation, write.
    pub const TLB_PROT_WRITE: u32 = 0x0c0;
    /// Address error, read or fetch.
    pub const ADDR_READ: u32 = 0x0e0;
    /// Address error, write.
    pub const ADDR_WRITE: u32 = 0x100;
    /// FPU exception.
    pub const FPU: u32 = 0x120;
    /// TLB multiple hit (a reset).
    pub const TLB_MULTI: u32 = 0x140;
    /// `TRAPA`.
    pub const TRAPA: u32 = 0x160;
    /// General illegal instruction.
    pub const ILLEGAL: u32 = 0x180;
    /// Slot illegal instruction.
    pub const SLOT_ILLEGAL: u32 = 0x1a0;
    /// Non-maskable interrupt (in `INTEVT`).
    pub const NMI: u32 = 0x1c0;
    /// General FPU disable.
    pub const FPU_DISABLE: u32 = 0x800;
    /// Slot FPU disable.
    pub const SLOT_FPU_DISABLE: u32 = 0x820;
}

/// Vector offsets from `VBR` (table 5.3).
pub mod vector {
    /// Most general exceptions.
    pub const GENERAL: u32 = 0x100;
    /// TLB misses.
    pub const TLB_MISS: u32 = 0x400;
    /// Interrupts.
    pub const INTERRUPT: u32 = 0x600;
}

/// The reset vector (§5.3.2).
pub const RESET_VECTOR: u32 = 0xa000_0000;

/// When an exception is taken relative to the instruction that raised it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Kind {
    /// The instruction did not happen and is re-executed on return.
    Reexec,
    /// The instruction completed; `SPC` is the one after it (`TRAPA`).
    Completion,
    /// The processor resets (`EXPEVT` gets the code, `PC` the reset vector).
    Reset,
}

/// An exception the current instruction raised.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Trap {
    /// What lands in `EXPEVT`.
    pub code: u32,
    /// The offset from `VBR`.
    pub vector: u32,
    /// The faulting address, for `TEA` and `PTEH.VPN`.
    pub tea: Option<u32>,
    /// Re-execution, completion or reset.
    pub kind: Kind,
}

impl Trap {
    /// A re-execution-type exception through the general vector.
    const fn general(code: u32) -> Trap {
        Trap {
            code,
            vector: vector::GENERAL,
            tea: None,
            kind: Kind::Reexec,
        }
    }

    /// An exception that reports an address.
    const fn at(code: u32, vector: u32, va: u32) -> Trap {
        Trap {
            code,
            vector,
            tea: Some(va),
            kind: Kind::Reexec,
        }
    }

    /// A TLB multiple hit, or an unmapped P1/P2 address in 32-bit mode — both
    /// reset the processor with `EXPEVT = H'140` (§7.5.1, §7.7.4).
    const fn multi(va: u32) -> Trap {
        Trap {
            code: exc::TLB_MULTI,
            vector: 0,
            tea: Some(va),
            kind: Kind::Reset,
        }
    }
}

/// Which kind of access a translation is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Access {
    /// An instruction fetch.
    Fetch,
    /// A data read.
    Read,
    /// A data write.
    Write,
}

impl Access {
    /// The address-error code this access raises.
    const fn address_error(self) -> u32 {
        match self {
            Access::Write => exc::ADDR_WRITE,
            _ => exc::ADDR_READ,
        }
    }
}

/// Where a translated access goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Target {
    /// The physical bus, at this address.
    Bus(u64),
    /// The P4 area at this address: the core's own registers and arrays, or
    /// the bus at the same address for anything the core does not own.
    P4(u32),
}

/// The architectural state of one processor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct State {
    /// The sixteen general registers as currently visible: `R0`–`R7` are the
    /// bank `SR.MD && SR.RB` selects.
    pub r: [u32; 16],
    /// The `R0`–`R7` of the bank that is *not* visible — what `LDC`/`STC`
    /// `Rn_BANK` reach.
    pub bank: [u32; 8],
    /// `SR`.
    pub sr: u32,
    /// `GBR`.
    pub gbr: u32,
    /// `SSR`.
    pub ssr: u32,
    /// `SPC`.
    pub spc: u32,
    /// `SGR`.
    pub sgr: u32,
    /// `DBR`.
    pub dbr: u32,
    /// `VBR`.
    pub vbr: u32,
    /// `MACH`.
    pub mach: u32,
    /// `MACL`.
    pub macl: u32,
    /// `PR`.
    pub pr: u32,
    /// The address of the next instruction to execute.
    pub pc: u32,
    /// The branch target the instruction at `pc` is the delay slot of.
    pub delay: Option<u32>,
    /// The two floating-point register banks, `FPRn_BANK0` and
    /// `FPRn_BANK1`, as raw bits.
    pub fpr: [[u32; 16]; 2],
    /// `FPSCR`.
    pub fpscr: u32,
    /// `FPUL`.
    pub fpul: u32,
    /// The `MOVLI`/`MOVCO` link flag.
    pub ldst: bool,
    /// Whether a `SLEEP` has stopped the processor.
    pub sleeping: bool,
    /// The registers that live in P4.
    pub ctl: Ctl,
    /// The TLBs and the PMB.
    pub mmu: Mmu,
    /// The two store queues.
    pub sq: [[u32; 8]; 2],
    /// Bus accesses since reset.
    pub cycles: u64,
    /// Accesses already executed past the last budget, owed to the next one.
    pub debt: u64,
    /// How many accesses the address space refused.
    pub faults: u64,
}

impl State {
    /// The power-on state for a configuration (table 2.1, table 11.2).
    pub(super) fn new(cfg: &Config) -> State {
        let mut st = State {
            r: [0; 16],
            bank: [0; 8],
            sr: sr::RESET,
            gbr: 0,
            ssr: 0,
            spc: 0,
            sgr: 0,
            dbr: 0,
            vbr: 0,
            mach: 0,
            macl: 0,
            pr: 0,
            pc: cfg.reset_vector,
            delay: None,
            fpr: [[0; 16]; 2],
            fpscr: fpscr::RESET,
            fpul: 0,
            ldst: false,
            sleeping: false,
            ctl: Ctl::default(),
            mmu: Mmu::default(),
            sq: [[0; 8]; 2],
            cycles: 0,
            debt: 0,
            faults: 0,
        };
        st.reset_registers(cfg, exc::POWER_ON);
        st
    }

    /// What a reset does to the CPU (§5.6.1, table 11.2): `SR`, `VBR`, `PC`,
    /// `FPSCR` and the control registers. A manual reset and a TLB
    /// multiple-hit reset keep the TLBs (§7.3.1); a power-on reset clears
    /// their `V` bits.
    pub(super) fn reset_registers(&mut self, cfg: &Config, code: u32) {
        self.set_sr(sr::RESET);
        self.vbr = 0;
        self.pc = cfg.reset_vector;
        self.delay = None;
        self.fpscr = fpscr::RESET;
        self.ldst = false;
        self.sleeping = false;
        let tea = self.ctl.tea;
        let keep = (self.ctl.pteh, self.ctl.ptel, self.ctl.ttb, self.ctl.tra);
        self.ctl = Ctl {
            expevt: code,
            cpuopm: mmu::CPUOPM_RESET,
            ..Ctl::default()
        };
        if code == exc::POWER_ON {
            self.mmu.invalidate_all();
        } else {
            // "TEA: Retained" across a manual reset (table 7.2), and the other
            // undefined-at-reset registers are simply not disturbed.
            self.ctl.tea = tea;
            (self.ctl.pteh, self.ctl.ptel, self.ctl.ttb, self.ctl.tra) = keep;
        }
    }

    /// Whether `R0`–`R7` are currently bank 1.
    #[inline]
    const fn bank1_visible(sr: u32) -> bool {
        sr & sr::MD != 0 && sr & sr::RB != 0
    }

    /// Write `SR`, swapping the visible bank of `R0`–`R7` if the write moves
    /// it. Every write of `SR` goes through here — `LDC`, `RTE`, exception
    /// entry — so the bank in `r` is never stale.
    pub(super) fn set_sr(&mut self, value: u32) {
        let value = value & sr::WRITABLE;
        if Self::bank1_visible(self.sr) != Self::bank1_visible(value) {
            for i in 0..8 {
                core::mem::swap(&mut self.r[i], &mut self.bank[i]);
            }
        }
        self.sr = value;
    }

    /// `R0_BANK0`–`R7_BANK0`, whichever of the two arrays holds them.
    #[must_use]
    pub(super) fn bank0(&self) -> [u32; 8] {
        let mut out = [0; 8];
        if Self::bank1_visible(self.sr) {
            out.copy_from_slice(&self.bank);
        } else {
            out.copy_from_slice(&self.r[..8]);
        }
        out
    }

    /// `R0_BANK1`–`R7_BANK1`.
    #[must_use]
    pub(super) fn bank1(&self) -> [u32; 8] {
        let mut out = [0; 8];
        if Self::bank1_visible(self.sr) {
            out.copy_from_slice(&self.r[..8]);
        } else {
            out.copy_from_slice(&self.bank);
        }
        out
    }

    /// Set both banks at once, placing each where `SR` says it is visible.
    pub(super) fn set_banks(&mut self, bank0: [u32; 8], bank1: [u32; 8]) {
        let (vis, hid) = if Self::bank1_visible(self.sr) {
            (bank1, bank0)
        } else {
            (bank0, bank1)
        };
        self.r[..8].copy_from_slice(&vis);
        self.bank = hid;
    }

    /// `SR.T`.
    #[inline]
    pub(super) const fn t(&self) -> bool {
        self.sr & sr::T != 0
    }

    /// Set or clear `SR.T`.
    #[inline]
    pub(super) fn set_t(&mut self, t: bool) {
        self.sr = (self.sr & !sr::T) | u32::from(t);
    }

    /// Whether the processor is in privileged mode.
    #[inline]
    pub(super) const fn privileged(&self) -> bool {
        self.sr & sr::MD != 0
    }
}

/// An interrupt the processor has decided to accept.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Accepted {
    /// The code for `INTEVT`.
    code: u32,
    /// The priority level, for `SR.IMASK` under `CPUOPM.INTMU`.
    level: u32,
    /// Whether it is the NMI, which consumes the latched edge.
    nmi: bool,
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    pub(super) st: &'a mut State,
    space: &'a AddressSpace,
    pub(super) cfg: &'a Config,
    lines: &'a Lines,
    /// Which architectural traps leave the core instead of vectoring.
    exits: ExitMask,
    /// Set instead of vectoring, when a trap is named in `exits`.
    exit: Option<Exit>,
    /// Accesses charged by this step.
    used: u64,
    /// The address of the instruction being executed.
    pub(super) this_pc: u32,
    /// Whether it is in a delay slot.
    in_slot: bool,
    /// Where control goes after a non-branching instruction.
    next_pc: u32,
    /// A delayed branch this instruction set up.
    new_delay: Option<u32>,
}

impl<'a> Exec<'a> {
    /// Borrow a processor for some steps.
    pub(super) fn new(
        st: &'a mut State,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
        exits: ExitMask,
    ) -> Exec<'a> {
        Exec {
            st,
            space,
            cfg,
            lines,
            exits,
            exit: None,
            used: 0,
            this_pc: 0,
            in_slot: false,
            next_pc: 0,
            new_delay: None,
        }
    }

    /// Take the exit the last step produced, if it produced one.
    pub(super) fn take_exit(&mut self) -> Option<Exit> {
        self.exit.take()
    }

    /// Execute one instruction, take one exception or interrupt, or idle one
    /// cycle asleep. Returns the accesses charged, which is never zero.
    pub(super) fn step(&mut self) -> u64 {
        self.used = 0;
        if self.st.sleeping {
            // "In sleep or standby mode, an interrupt is accepted even if the
            // BL bit in SR is set to 1" (§5.7, 2.B).
            if let Some(irq) = self.acceptable_interrupt(true) {
                self.st.sleeping = false;
                self.take_interrupt(irq);
            }
            return self.used.max(1);
        }
        // A delayed branch and its slot are indivisible (§5.6.4): no
        // interrupt between them.
        if self.st.delay.is_none()
            && let Some(irq) = self.acceptable_interrupt(false)
        {
            self.take_interrupt(irq);
            return self.used.max(1);
        }
        self.this_pc = self.st.pc;
        self.in_slot = self.st.delay.is_some();
        if let Err(trap) = self.execute() {
            self.handle(trap);
        }
        self.used.max(1)
    }

    /// Whether the processor is asleep with nothing to wake it — what lets a
    /// run loop skip the rest of a budget instead of spinning.
    pub(super) fn idle(&self) -> bool {
        self.st.sleeping && self.acceptable_interrupt(true).is_none()
    }

    // -----------------------------------------------------------------
    // Interrupts and exceptions
    // -----------------------------------------------------------------

    /// The interrupt the processor would accept now, if any.
    ///
    /// The NMI is accepted whenever `SR.BL` is clear, regardless of `IMASK`
    /// (§5.6.3); whether it may also break through `BL` is an interrupt
    /// controller setting this core does not have, so it waits. A general
    /// request is accepted when its level exceeds `SR.IMASK` and `BL` is clear
    /// (§5.6.3, *General Interrupt Request*).
    fn acceptable_interrupt(&self, asleep: bool) -> Option<Accepted> {
        let blocked = self.st.sr & sr::BL != 0 && !asleep;
        if blocked {
            return None;
        }
        if self.lines.nmi_pending() {
            return Some(Accepted {
                code: exc::NMI,
                level: 16,
                nmi: true,
            });
        }
        let (level, code) = self.lines.request();
        let imask = (self.st.sr & sr::IMASK) >> 4;
        (level > imask).then_some(Accepted {
            code,
            level,
            nmi: false,
        })
    }

    /// Accept an interrupt: a completion-type event, so `SPC` is the
    /// instruction that would have run next (§5.6.3).
    fn take_interrupt(&mut self, irq: Accepted) {
        if irq.nmi {
            self.lines.take_nmi();
        }
        self.st.ctl.intevt = irq.code & 0x3fff;
        let pc = self.st.pc;
        self.enter(pc, vector::INTERRUPT);
        // `CPUOPM.INTMU` decides whether the accepted level is copied into
        // `SR.IMASK` (appendix A); the NMI has no level to copy.
        if !irq.nmi && self.st.ctl.cpuopm & mmu::CPUOPM_INTMU != 0 {
            let sr = (self.st.sr & !sr::IMASK) | ((irq.level & 0xf) << 4);
            self.st.set_sr(sr);
        }
    }

    /// Save `PC`, `SR` and `R15`, block, switch to the privileged bank and go
    /// to `VBR + offset` (§5.3.1, steps 1–7).
    fn enter(&mut self, spc: u32, offset: u32) {
        let st = &mut *self.st;
        st.spc = spc;
        st.ssr = st.sr;
        st.sgr = st.r[15];
        st.set_sr(st.sr | sr::MD | sr::RB | sr::BL);
        st.pc = st.vbr.wrapping_add(offset);
        st.delay = None;
        // "If an interrupt or exception occurs, LDST is cleared to 0"
        // (§10.1.37).
        st.ldst = false;
    }

    /// Deliver an exception: as an exit if one is armed for it, as a reset, or
    /// through the vector table.
    fn handle(&mut self, trap: Trap) {
        if let Some(exit) = self.exit_for(&trap) {
            if trap.code == exc::TRAPA {
                // Completion type: the consumer resumes after the trap.
                self.st.pc = self.this_pc.wrapping_add(2);
            }
            self.exit = Some(exit);
            return;
        }
        if let Some(va) = trap.tea {
            self.st.ctl.tea = va;
            self.st.ctl.pteh = (va & 0xffff_fc00) | (self.st.ctl.pteh & 0xff);
        }
        if trap.kind == Kind::Reset {
            self.st.reset_registers(self.cfg, trap.code);
            return;
        }
        if self.st.sr & sr::BL != 0 {
            // "When the BL bit in SR is 1 and an exception other than a user
            // break is generated, the CPU's internal registers ... are set to
            // their states following a manual reset" (§5.5.3).
            self.st.reset_registers(self.cfg, exc::MANUAL_RESET);
            return;
        }
        let spc = match trap.kind {
            Kind::Completion => self.this_pc.wrapping_add(2),
            // In a slot, the PC of the delayed branch (§5.7, 3.A).
            _ if self.in_slot => self.this_pc.wrapping_sub(2),
            _ => self.this_pc,
        };
        self.st.ctl.expevt = trap.code;
        self.enter(spc, trap.vector);
    }

    /// Whether `trap` should leave the core rather than vector into the guest,
    /// and as what.
    fn exit_for(&self, trap: &Trap) -> Option<Exit> {
        if self.exits.is_empty() {
            return None;
        }
        let (reason, access) = match trap.code {
            exc::TRAPA => (ExitReason::SYSCALL, ExitAccess::None),
            exc::ILLEGAL | exc::SLOT_ILLEGAL | exc::FPU_DISABLE | exc::SLOT_FPU_DISABLE => {
                (ExitReason::FAULT, ExitAccess::None)
            }
            exc::FPU => (ExitReason::FAULT, ExitAccess::None),
            exc::ADDR_READ | exc::TLB_MISS_READ | exc::TLB_PROT_READ => {
                (ExitReason::FAULT, ExitAccess::Read)
            }
            exc::ADDR_WRITE | exc::TLB_MISS_WRITE | exc::TLB_PROT_WRITE | exc::INITIAL_WRITE => {
                (ExitReason::FAULT, ExitAccess::Write)
            }
            _ => return None,
        };
        if !self.exits.contains(reason) {
            return None;
        }
        let exit = Exit::new(reason, u64::from(self.this_pc), 2).with_detail(u64::from(trap.code));
        match (access, trap.tea) {
            (ExitAccess::None, _) | (_, None) => Some(exit),
            (_, Some(addr)) => Some(exit.with_access(u64::from(addr), access)),
        }
    }

    /// The illegal-instruction exception for this position: general, or slot
    /// in a delay slot (§5.6.2).
    fn illegal(&self) -> Trap {
        Trap::general(if self.in_slot {
            exc::SLOT_ILLEGAL
        } else {
            exc::ILLEGAL
        })
    }

    /// The FPU-disable exception for this position.
    fn fpu_disabled(&self) -> Trap {
        Trap::general(if self.in_slot {
            exc::SLOT_FPU_DISABLE
        } else {
            exc::FPU_DISABLE
        })
    }

    /// The illegal-instruction exception for an FPU encoding whose `PR`/`SZ`
    /// combination the manual calls `undefined_operation()` (§10.3) — taken
    /// as an illegal instruction, which is what an undefined code is.
    pub(super) fn undefined(&self) -> Trap {
        self.illegal()
    }

    // -----------------------------------------------------------------
    // The register file
    // -----------------------------------------------------------------

    /// Charge one bus access.
    #[inline]
    fn charge(&mut self) {
        self.used += 1;
        self.st.cycles = self.st.cycles.wrapping_add(1);
    }

    /// Read a general register.
    #[inline]
    pub(super) fn r(&self, i: usize) -> u32 {
        self.st.r[i]
    }

    /// Write a general register.
    #[inline]
    pub(super) fn set_r(&mut self, i: usize, v: u32) {
        self.st.r[i] = v;
    }

    /// The memory attributes for a data or fetch access in the current mode.
    #[inline]
    fn attrs(&self, fetch: bool) -> MemAttrs {
        let a = MemAttrs::DEFAULT
            .with_requester(self.cfg.requester)
            .with_privileged(self.st.privileged());
        if fetch {
            a.with_purpose(AccessPurpose::FETCH)
        } else {
            a
        }
    }

    // -----------------------------------------------------------------
    // Translation
    // -----------------------------------------------------------------

    /// Translate a virtual address for an access of `kind` (manual §7.1.1 and
    /// flowcharts 7.9 and 7.10).
    pub(super) fn resolve(&mut self, va: u32, kind: Access) -> Result<Target, Trap> {
        let md = self.st.privileged();
        let ctl = &self.st.ctl;
        match va >> 29 {
            7 => {
                // P4. In user mode only the store queues are reachable, and
                // only when `MMUCR.SQMD` allows it (§8.7.4).
                if !md
                    && !(kind != Access::Fetch
                        && va < 0xe400_0000
                        && ctl.mmucr & mmu::mmucr::SQMD == 0)
                {
                    return Err(Trap::at(kind.address_error(), vector::GENERAL, va));
                }
                Ok(Target::P4(va))
            }
            4 | 5 => {
                if !md {
                    return Err(Trap::at(kind.address_error(), vector::GENERAL, va));
                }
                if !ctl.se() {
                    return Ok(Target::Bus(u64::from(va & 0x1fff_ffff)));
                }
                match self.st.mmu.search_pmb(va) {
                    Hit::One(i) => {
                        let e = self.st.mmu.pmb[i];
                        let mask = pmb_mask(e.sz);
                        Ok(physical((e.ppn & mask) | (va & !mask), true))
                    }
                    // An unmapped P1/P2 address in 32-bit mode resets "by the
                    // TLB" (§7.7.4, 2), and a multiple hit is unguaranteed;
                    // both take the multiple-hit reset.
                    _ => Err(Trap::multi(va)),
                }
            }
            region => {
                if !md && region >= 4 {
                    return Err(Trap::at(kind.address_error(), vector::GENERAL, va));
                }
                if !ctl.at() {
                    return Ok(if ctl.se() {
                        Target::Bus(u64::from(va))
                    } else {
                        Target::Bus(u64::from(va & 0x1fff_ffff))
                    });
                }
                let phys = if kind == Access::Fetch {
                    self.itlb_translate(va)?
                } else {
                    self.utlb_translate(va, kind)?
                };
                Ok(physical(phys, self.st.ctl.se()))
            }
        }
    }

    /// A data access through the UTLB (flowchart 7.9).
    fn utlb_translate(&mut self, va: u32, kind: Access) -> Result<u32, Trap> {
        let md = self.st.privileged();
        let (asid, sv) = (self.st.ctl.asid(), self.st.ctl.sv());
        self.st.ctl.tick_urc();
        let e = match self.st.mmu.search_utlb(va, asid, sv, md) {
            Hit::One(i) => self.st.mmu.utlb[i],
            Hit::Multiple => return Err(Trap::multi(va)),
            Hit::Miss => {
                let code = if kind == Access::Write {
                    exc::TLB_MISS_WRITE
                } else {
                    exc::TLB_MISS_READ
                };
                return Err(Trap::at(code, vector::TLB_MISS, va));
            }
        };
        let write = kind == Access::Write;
        let permitted = match (md, e.pr) {
            (true, 0 | 2) => !write,
            (true, _) => true,
            (false, 0 | 1) => false,
            (false, 2) => !write,
            (false, _) => true,
        };
        if !permitted {
            let code = if write {
                exc::TLB_PROT_WRITE
            } else {
                exc::TLB_PROT_READ
            };
            return Err(Trap::at(code, vector::GENERAL, va));
        }
        if write && !e.d {
            return Err(Trap::at(exc::INITIAL_WRITE, vector::GENERAL, va));
        }
        let mask = page_mask(e.sz);
        Ok((e.ppn & mask) | (va & !mask))
    }

    /// An instruction fetch through the ITLB, refilling it from the UTLB on a
    /// miss (flowchart 7.10, §7.4.4).
    fn itlb_translate(&mut self, va: u32) -> Result<u32, Trap> {
        let md = self.st.privileged();
        let (asid, sv) = (self.st.ctl.asid(), self.st.ctl.sv());
        let used = match self.st.mmu.search_itlb(va, asid, sv, md) {
            Hit::One(i) => i,
            Hit::Multiple => return Err(Trap::multi(va)),
            Hit::Miss => {
                self.st.ctl.tick_urc();
                let src = match self.st.mmu.search_utlb(va, asid, sv, md) {
                    Hit::One(j) => self.st.mmu.utlb[j],
                    Hit::Multiple => return Err(Trap::multi(va)),
                    Hit::Miss => {
                        return Err(Trap::at(exc::TLB_MISS_READ, vector::TLB_MISS, va));
                    }
                };
                let victim = lrui_victim(self.st.ctl.lrui());
                self.st.mmu.itlb[victim] = ItlbEntry {
                    vpn: src.vpn,
                    asid: src.asid,
                    ppn: src.ppn,
                    sz: src.sz,
                    pr: src.pr & 2 != 0,
                    v: true,
                    sh: src.sh,
                    c: src.c,
                };
                victim
            }
        };
        let lrui = lrui_touch(self.st.ctl.lrui(), used);
        self.st.ctl.set_lrui(lrui);
        let e = self.st.mmu.itlb[used];
        if !md && !e.pr {
            return Err(Trap::at(exc::TLB_PROT_READ, vector::GENERAL, va));
        }
        let mask = page_mask(e.sz);
        Ok((e.ppn & mask) | (va & !mask))
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// Fetch the instruction at `pc`.
    fn fetch(&mut self, pc: u32) -> Result<u16, Trap> {
        if pc & 1 != 0 {
            return Err(Trap::at(exc::ADDR_READ, vector::GENERAL, pc));
        }
        let target = self.resolve(pc, Access::Fetch)?;
        Ok(self.access_read(target, Width::U16, true) as u16)
    }

    /// Read `width` from a virtual address, checking alignment.
    pub(super) fn read(&mut self, va: u32, width: Width) -> Result<u32, Trap> {
        if va & (width.bytes() as u32 - 1) != 0 {
            return Err(Trap::at(exc::ADDR_READ, vector::GENERAL, va));
        }
        let target = self.resolve(va, Access::Read)?;
        Ok(self.access_read(target, width, false))
    }

    /// Write `width` to a virtual address, checking alignment.
    pub(super) fn write(&mut self, va: u32, width: Width, value: u32) -> Result<(), Trap> {
        if va & (width.bytes() as u32 - 1) != 0 {
            return Err(Trap::at(exc::ADDR_WRITE, vector::GENERAL, va));
        }
        let target = self.resolve(va, Access::Write)?;
        self.access_write(target, width, value);
        Ok(())
    }

    /// Check that a write to `va` would be permitted, without writing — for
    /// an instruction whose second access must not happen if its first would
    /// fault, and for the cache-block operations.
    pub(super) fn probe(&mut self, va: u32, width: Width, kind: Access) -> Result<Target, Trap> {
        if va & (width.bytes() as u32 - 1) != 0 {
            return Err(Trap::at(kind.address_error(), vector::GENERAL, va));
        }
        self.resolve(va, kind)
    }

    /// Read from a resolved target.
    ///
    /// The SH-4A CPU has no bus-error exception (section 5 lists none), so a
    /// refused access reads as zero and is counted in [`State::faults`] — a
    /// diagnostic a machine with a hole in its map will see climbing. A fetch
    /// of zero is an undefined instruction, so a jump into nothing still traps.
    pub(super) fn access_read(&mut self, target: Target, width: Width, fetch: bool) -> u32 {
        match target {
            Target::Bus(addr) => self.bus_read(addr, width, fetch),
            Target::P4(addr) => self.p4_read(addr, width, fetch),
        }
    }

    /// Write to a resolved target.
    pub(super) fn access_write(&mut self, target: Target, width: Width, value: u32) {
        match target {
            Target::Bus(addr) => self.bus_write(addr, width, value),
            Target::P4(addr) => self.p4_write(addr, width, value),
        }
    }

    /// One read on the bus.
    fn bus_read(&mut self, addr: u64, width: Width, fetch: bool) -> u32 {
        self.charge();
        match self.space.read(addr, width, self.attrs(fetch)) {
            Ok(v) => v as u32,
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                0
            }
        }
    }

    /// One write on the bus.
    fn bus_write(&mut self, addr: u64, width: Width, value: u32) {
        self.charge();
        if self
            .space
            .write(addr, width, u64::from(value), self.attrs(false))
            .is_err()
        {
            self.st.faults = self.st.faults.wrapping_add(1);
        }
    }

    /// Read a P4 address: the core's own registers and arrays, or the bus.
    fn p4_read(&mut self, a: u32, width: Width, fetch: bool) -> u32 {
        let st = &*self.st;
        let se = st.ctl.se();
        let v = match a >> 24 {
            // The store queues are write-only at this address (§8.7.2).
            0xe0..=0xe3 => 0,
            // Cache arrays: no cache is modelled, so every line reads as
            // invalid.
            0xf0 | 0xf1 | 0xf4 | 0xf5 => 0,
            0xf2 => {
                let e = &st.mmu.itlb[((a >> 8) & 3) as usize];
                e.vpn | (u32::from(e.v) << 8) | u32::from(e.asid)
            }
            0xf3 if a & 0x0080_0000 == 0 => {
                let e = &st.mmu.itlb[((a >> 8) & 3) as usize];
                e.ppn
                    | (u32::from(e.v) << 8)
                    | (u32::from(e.sz >> 1) << 7)
                    | (u32::from(e.pr) << 6)
                    | (u32::from(e.sz & 1) << 4)
                    | (u32::from(e.c) << 3)
                    | (u32::from(e.sh) << 1)
            }
            0xf6 => match (a >> 20) & 0xf {
                0 => {
                    let e = &st.mmu.utlb[((a >> 8) & 0x3f) as usize];
                    e.vpn | (u32::from(e.d) << 9) | (u32::from(e.v) << 8) | u32::from(e.asid)
                }
                1 => {
                    let e = &st.mmu.pmb[((a >> 8) & 0xf) as usize];
                    e.vpn | (u32::from(e.v) << 8)
                }
                _ => 0,
            },
            0xf7 => match (a >> 20) & 0xf {
                0 => {
                    let e = &st.mmu.utlb[((a >> 8) & 0x3f) as usize];
                    utlb_data(e, se)
                }
                1 => {
                    let e = &st.mmu.pmb[((a >> 8) & 0xf) as usize];
                    e.ppn
                        | (u32::from(e.ub) << 9)
                        | (u32::from(e.v) << 8)
                        | (u32::from(e.sz >> 1) << 7)
                        | (u32::from(e.sz & 1) << 4)
                        | (u32::from(e.c) << 3)
                        | u32::from(e.wt)
                }
                _ => 0,
            },
            _ => match a {
                reg::PTEH => st.ctl.pteh,
                reg::PTEL => st.ctl.ptel,
                reg::TTB => st.ctl.ttb,
                reg::TEA => st.ctl.tea,
                reg::MMUCR => st.ctl.mmucr,
                reg::CCR => st.ctl.ccr,
                reg::TRA => st.ctl.tra,
                reg::EXPEVT => st.ctl.expevt,
                reg::INTEVT => st.ctl.intevt,
                reg::PVR => self.cfg.pvr,
                reg::QACR0 => st.ctl.qacr0,
                reg::QACR1 => st.ctl.qacr1,
                reg::PRR => self.cfg.prr,
                reg::LSA0 | reg::LSA1 | reg::LDA0 | reg::LDA1 => {
                    st.ctl.lmem[((a - reg::LSA0) >> 2) as usize]
                }
                reg::PASCR => st.ctl.pascr,
                reg::RAMCR => st.ctl.ramcr,
                reg::IRMCR => st.ctl.irmcr,
                reg::CPUOPM => st.ctl.cpuopm,
                _ if (reg::SQ_READ..reg::SQ_READ + 0x40).contains(&a) => {
                    st.sq[((a >> 5) & 1) as usize][((a >> 2) & 7) as usize]
                }
                _ => return self.bus_read(u64::from(a), width, fetch),
            },
        };
        self.charge();
        narrow(v, a, width)
    }

    /// Write a P4 address.
    fn p4_write(&mut self, a: u32, width: Width, value: u32) {
        let se = self.st.ctl.se();
        match a >> 24 {
            0xe0..=0xe3 => {
                // A store-queue write (§8.7.2). Longword and quadword only; a
                // quadword arrives here as two longwords.
                self.st.sq[((a >> 5) & 1) as usize][((a >> 2) & 7) as usize] = value;
            }
            0xf0 | 0xf1 | 0xf4 | 0xf5 => {}
            0xf2 => {
                let e = &mut self.st.mmu.itlb[((a >> 8) & 3) as usize];
                e.vpn = value & 0xffff_fc00;
                e.v = value & 0x100 != 0;
                e.asid = value as u8;
            }
            0xf3 if a & 0x0080_0000 == 0 => {
                let e = &mut self.st.mmu.itlb[((a >> 8) & 3) as usize];
                e.ppn = value & ppn_mask(se);
                e.v = value & 0x100 != 0;
                e.sz = size_bits(value);
                e.pr = value & 0x40 != 0;
                e.c = value & 0x8 != 0;
                e.sh = value & 0x2 != 0;
            }
            0xf3 => {}
            0xf6 => match (a >> 20) & 0xf {
                0 => self.utlb_address_write(a, value),
                1 => {
                    let e = &mut self.st.mmu.pmb[((a >> 8) & 0xf) as usize];
                    e.vpn = value & 0xff00_0000;
                    e.v = value & 0x100 != 0;
                }
                _ => {}
            },
            0xf7 => match (a >> 20) & 0xf {
                0 => {
                    let e = &mut self.st.mmu.utlb[((a >> 8) & 0x3f) as usize];
                    *e = UtlbEntry {
                        ppn: value & ppn_mask(se),
                        ub: se && value & 0x200 != 0,
                        v: value & 0x100 != 0,
                        sz: size_bits(value),
                        pr: ((value >> 5) & 3) as u8,
                        c: value & 0x8 != 0,
                        d: value & 0x4 != 0,
                        sh: value & 0x2 != 0,
                        wt: value & 0x1 != 0,
                        ..*e
                    };
                }
                1 => {
                    let e = &mut self.st.mmu.pmb[((a >> 8) & 0xf) as usize];
                    *e = PmbEntry {
                        ppn: value & 0xff00_0000,
                        ub: value & 0x200 != 0,
                        v: value & 0x100 != 0,
                        sz: size_bits(value),
                        c: value & 0x8 != 0,
                        wt: value & 0x1 != 0,
                        ..*e
                    };
                }
                _ => {}
            },
            _ => {
                let ctl = &mut self.st.ctl;
                match a {
                    reg::PTEH => ctl.pteh = value & 0xffff_fcff,
                    reg::PTEL => ctl.ptel = value & if se { 0xffff_ffff } else { 0x1fff_fdff },
                    reg::TTB => ctl.ttb = value,
                    reg::TEA => ctl.tea = value,
                    reg::MMUCR => {
                        if value & mmu::mmucr::TI != 0 {
                            self.st.mmu.invalidate_tlbs();
                        }
                        self.st.ctl.mmucr = value & mmu::mmucr::WRITABLE & !mmu::mmucr::TI;
                    }
                    reg::CCR => ctl.ccr = value,
                    reg::TRA => ctl.tra = value & 0x3fc,
                    reg::EXPEVT => ctl.expevt = value & 0xfff,
                    reg::INTEVT => ctl.intevt = value & 0x3fff,
                    reg::PVR | reg::PRR => {}
                    reg::QACR0 => ctl.qacr0 = value & 0xfc,
                    reg::QACR1 => ctl.qacr1 = value & 0xfc,
                    reg::LSA0 | reg::LSA1 | reg::LDA0 | reg::LDA1 => {
                        ctl.lmem[((a - reg::LSA0) >> 2) as usize] = value;
                    }
                    reg::PASCR => ctl.pascr = value & 0x8000_00ff,
                    reg::RAMCR => ctl.ramcr = value,
                    reg::IRMCR => ctl.irmcr = value & 0x1f,
                    // Reserved bits keep their reset value (appendix A).
                    reg::CPUOPM => ctl.cpuopm = 0x3c0 | (value & 0x28),
                    _ => return self.bus_write(u64::from(a), width, value),
                }
            }
        }
        self.charge();
    }

    /// A write to the UTLB address array (§7.6.3): direct, or — with the `A`
    /// bit — associative, updating `D` and `V` of whichever UTLB entry and `V`
    /// of whichever ITLB entry match the VPN in the data and the ASID in
    /// `PTEH`. An associative miss does nothing and raises nothing.
    fn utlb_address_write(&mut self, a: u32, value: u32) {
        let vpn = value & 0xffff_fc00;
        let d = value & 0x200 != 0;
        let v = value & 0x100 != 0;
        if a & 0x80 == 0 {
            let e = &mut self.st.mmu.utlb[((a >> 8) & 0x3f) as usize];
            e.vpn = vpn;
            e.d = d;
            e.v = v;
            e.asid = value as u8;
            return;
        }
        let md = self.st.privileged();
        let (asid, sv) = (self.st.ctl.asid(), self.st.ctl.sv());
        if let Hit::One(i) = self.st.mmu.search_utlb(vpn, asid, sv, md) {
            self.st.mmu.utlb[i].d = d;
            self.st.mmu.utlb[i].v = v;
        }
        if let Hit::One(i) = self.st.mmu.search_itlb(vpn, asid, sv, md) {
            self.st.mmu.itlb[i].v = v;
        }
    }

    /// Flush a store queue to memory: `PREF` on the store-queue area (§8.7.3).
    fn flush_store_queue(&mut self, va: u32) -> Result<(), Trap> {
        let q = ((va >> 5) & 1) as usize;
        let base = if self.st.ctl.at() {
            // The UTLB maps the SQ area; bits 9–5 come from the address.
            self.utlb_translate(va, Access::Read)? & !0x1f
        } else {
            // QACRn[4:2] (extended to [7:2] in 32-bit mode) supply the top
            // physical bits the SQ address cannot (§8.7.3, §8.8).
            let qacr = if q == 0 {
                self.st.ctl.qacr0
            } else {
                self.st.ctl.qacr1
            };
            let top = if self.st.ctl.se() {
                qacr & 0xfc
            } else {
                qacr & 0x1c
            };
            (top << 24) | (va & 0x03ff_ffe0)
        };
        let data = self.st.sq[q];
        for (i, word) in data.iter().enumerate() {
            self.bus_write(u64::from(base) + 4 * i as u64, Width::U32, *word);
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Execution
    // -----------------------------------------------------------------

    /// Fetch, decode, check and execute the instruction at the PC.
    fn execute(&mut self) -> Result<(), Trap> {
        let pc = self.this_pc;
        let word = self.fetch(pc)?;
        let Some(insn) = isa::decode(word) else {
            return Err(self.illegal());
        };
        if self.in_slot && insn.slot_illegal() {
            return Err(Trap::general(exc::SLOT_ILLEGAL));
        }
        if insn.privileged() && !self.st.privileged() {
            return Err(self.illegal());
        }
        if insn.fpu() && self.st.sr & sr::FD != 0 {
            return Err(self.fpu_disabled());
        }
        self.next_pc = pc.wrapping_add(2);
        self.new_delay = None;
        if insn.fpu() && word >> 12 == 0xf {
            self.fpu(insn.op, word)?;
        } else {
            self.op(insn.op, word)?;
        }
        if self.in_slot {
            // The slot is done: now the branch happens.
            self.st.pc = self.st.delay.take().unwrap_or(self.next_pc);
        } else {
            self.st.pc = self.next_pc;
            self.st.delay = self.new_delay;
        }
        Ok(())
    }

    /// Set up a delayed branch to `target`.
    #[inline]
    fn delayed(&mut self, target: u32) {
        self.new_delay = Some(target);
    }

    /// Execute an integer or system instruction.
    #[allow(clippy::too_many_lines)]
    fn op(&mut self, op: Op, w: u16) -> Result<(), Trap> {
        let n = isa::n(w);
        let m = isa::m(w);
        let pc = self.this_pc;
        match op {
            // -- data transfer -------------------------------------------------
            Op::Mov => self.set_r(n, self.r(m)),
            Op::MovI => self.set_r(n, isa::i8s(w)),
            Op::MovwPc => {
                let v = self.read(isa::pc_word(pc, w), Width::U16)?;
                self.set_r(n, sext16(v));
            }
            Op::MovlPc => {
                let v = self.read(isa::pc_long(pc, w), Width::U32)?;
                self.set_r(n, v);
            }
            Op::Mova => self.set_r(0, isa::pc_long(pc, w)),
            Op::MovbS => self.write(self.r(n), Width::U8, self.r(m))?,
            Op::MovwS => self.write(self.r(n), Width::U16, self.r(m))?,
            Op::MovlS => self.write(self.r(n), Width::U32, self.r(m))?,
            Op::MovbL => {
                let v = self.read(self.r(m), Width::U8)?;
                self.set_r(n, sext8(v));
            }
            Op::MovwL => {
                let v = self.read(self.r(m), Width::U16)?;
                self.set_r(n, sext16(v));
            }
            Op::MovlL => {
                let v = self.read(self.r(m), Width::U32)?;
                self.set_r(n, v);
            }
            // §10.1.30: the stored value is `Rm` as it was, then `Rn` moves —
            // `MOV.L R1,@-R1` stores the old R1.
            Op::MovbM => self.push(n, Width::U8, self.r(m))?,
            Op::MovwM => self.push(n, Width::U16, self.r(m))?,
            Op::MovlM => self.push(n, Width::U32, self.r(m))?,
            // §10.1.30: `if (n!=m) R[m] += size` — the load wins.
            Op::MovbP => {
                let v = self.read(self.r(m), Width::U8)?;
                self.post_inc(m, n, 1);
                self.set_r(n, sext8(v));
            }
            Op::MovwP => {
                let v = self.read(self.r(m), Width::U16)?;
                self.post_inc(m, n, 2);
                self.set_r(n, sext16(v));
            }
            Op::MovlP => {
                let v = self.read(self.r(m), Width::U32)?;
                self.post_inc(m, n, 4);
                self.set_r(n, v);
            }
            Op::MovbS4 => {
                let a = self.r(m).wrapping_add(isa::d4(w));
                self.write(a, Width::U8, self.r(0))?;
            }
            Op::MovwS4 => {
                let a = self.r(m).wrapping_add(isa::d4(w) << 1);
                self.write(a, Width::U16, self.r(0))?;
            }
            Op::MovlS4 => {
                let a = self.r(n).wrapping_add(isa::d4(w) << 2);
                self.write(a, Width::U32, self.r(m))?;
            }
            Op::MovbL4 => {
                let a = self.r(m).wrapping_add(isa::d4(w));
                let v = self.read(a, Width::U8)?;
                self.set_r(0, sext8(v));
            }
            Op::MovwL4 => {
                let a = self.r(m).wrapping_add(isa::d4(w) << 1);
                let v = self.read(a, Width::U16)?;
                self.set_r(0, sext16(v));
            }
            Op::MovlL4 => {
                let a = self.r(m).wrapping_add(isa::d4(w) << 2);
                let v = self.read(a, Width::U32)?;
                self.set_r(n, v);
            }
            Op::MovbS0 => self.write(self.r(n).wrapping_add(self.r(0)), Width::U8, self.r(m))?,
            Op::MovwS0 => self.write(self.r(n).wrapping_add(self.r(0)), Width::U16, self.r(m))?,
            Op::MovlS0 => self.write(self.r(n).wrapping_add(self.r(0)), Width::U32, self.r(m))?,
            Op::MovbL0 => {
                let v = self.read(self.r(m).wrapping_add(self.r(0)), Width::U8)?;
                self.set_r(n, sext8(v));
            }
            Op::MovwL0 => {
                let v = self.read(self.r(m).wrapping_add(self.r(0)), Width::U16)?;
                self.set_r(n, sext16(v));
            }
            Op::MovlL0 => {
                let v = self.read(self.r(m).wrapping_add(self.r(0)), Width::U32)?;
                self.set_r(n, v);
            }
            Op::MovbSg => {
                let a = self.st.gbr.wrapping_add(isa::i8u(w));
                self.write(a, Width::U8, self.r(0))?;
            }
            Op::MovwSg => {
                let a = self.st.gbr.wrapping_add(isa::i8u(w) << 1);
                self.write(a, Width::U16, self.r(0))?;
            }
            Op::MovlSg => {
                let a = self.st.gbr.wrapping_add(isa::i8u(w) << 2);
                self.write(a, Width::U32, self.r(0))?;
            }
            Op::MovbLg => {
                let v = self.read(self.st.gbr.wrapping_add(isa::i8u(w)), Width::U8)?;
                self.set_r(0, sext8(v));
            }
            Op::MovwLg => {
                let v = self.read(self.st.gbr.wrapping_add(isa::i8u(w) << 1), Width::U16)?;
                self.set_r(0, sext16(v));
            }
            Op::MovlLg => {
                let v = self.read(self.st.gbr.wrapping_add(isa::i8u(w) << 2), Width::U32)?;
                self.set_r(0, v);
            }
            Op::Movt => self.set_r(n, u32::from(self.st.t())),
            Op::Swapb => {
                let v = self.r(m);
                self.set_r(n, (v & 0xffff_0000) | ((v & 0xff) << 8) | ((v >> 8) & 0xff));
            }
            Op::Swapw => self.set_r(n, self.r(m).rotate_left(16)),
            Op::Xtrct => self.set_r(n, (self.r(m) << 16) | (self.r(n) >> 16)),
            Op::Movcal => self.write(self.r(n), Width::U32, self.r(0))?,
            // §10.1.37: the read happens and LDST is set; an exception before
            // the MOVCO clears it.
            Op::Movli => {
                let v = self.read(self.r(n), Width::U32)?;
                self.st.ldst = true;
                self.set_r(0, v);
            }
            // §10.1.36: T ← LDST; store only if it was set; LDST ← 0.
            Op::Movco => {
                if self.st.ldst {
                    self.write(self.r(n), Width::U32, self.r(0))?;
                }
                let t = self.st.ldst;
                self.st.set_t(t);
                self.st.ldst = false;
            }
            Op::Movua | Op::MovuaP => {
                let a = self.r(n);
                let v = self.read_unaligned(a)?;
                // §10.1.39: `if(m != 0) R[m] += 4` — `@R0+` loads and does not
                // increment.
                if op == Op::MovuaP && n != 0 {
                    self.set_r(n, a.wrapping_add(4));
                }
                self.set_r(0, v);
            }

            // -- arithmetic ------------------------------------------------------
            Op::Add => self.set_r(n, self.r(n).wrapping_add(self.r(m))),
            Op::AddI => self.set_r(n, self.r(n).wrapping_add(isa::i8s(w))),
            Op::Addc => {
                let sum = u64::from(self.r(n)) + u64::from(self.r(m)) + u64::from(self.st.t());
                self.set_r(n, sum as u32);
                self.st.set_t(sum >> 32 != 0);
            }
            Op::Addv => {
                let (a, b) = (self.r(n), self.r(m));
                let r = a.wrapping_add(b);
                self.set_r(n, r);
                self.st.set_t(((a ^ r) & (b ^ r)) >> 31 != 0);
            }
            Op::Sub => self.set_r(n, self.r(n).wrapping_sub(self.r(m))),
            Op::Subc => {
                let (a, b) = (self.r(n), self.r(m));
                let t = u64::from(self.st.t());
                let r = u64::from(a).wrapping_sub(u64::from(b)).wrapping_sub(t);
                self.set_r(n, r as u32);
                self.st.set_t(u64::from(b) + t > u64::from(a));
            }
            Op::Subv => {
                let (a, b) = (self.r(n), self.r(m));
                let r = a.wrapping_sub(b);
                self.set_r(n, r);
                self.st.set_t(((a ^ b) & (a ^ r)) >> 31 != 0);
            }
            Op::Neg => self.set_r(n, 0u32.wrapping_sub(self.r(m))),
            Op::Negc => {
                let b = u64::from(self.r(m)) + u64::from(self.st.t());
                self.set_r(n, 0u64.wrapping_sub(b) as u32);
                self.st.set_t(b != 0);
            }
            Op::CmpEq => self.st.set_t(self.r(n) == self.r(m)),
            Op::CmpEqI => self.st.set_t(self.r(0) == isa::i8s(w)),
            Op::CmpHs => self.st.set_t(self.r(n) >= self.r(m)),
            Op::CmpGe => self.st.set_t(self.r(n) as i32 >= self.r(m) as i32),
            Op::CmpHi => self.st.set_t(self.r(n) > self.r(m)),
            Op::CmpGt => self.st.set_t(self.r(n) as i32 > self.r(m) as i32),
            Op::CmpPz => self.st.set_t(self.r(n) as i32 >= 0),
            Op::CmpPl => self.st.set_t(self.r(n) as i32 > 0),
            Op::CmpStr => {
                let x = self.r(n) ^ self.r(m);
                let any = x & 0xff == 0
                    || x & 0xff00 == 0
                    || x & 0x00ff_0000 == 0
                    || x & 0xff00_0000 == 0;
                self.st.set_t(any);
            }
            Op::Dt => {
                let v = self.r(n).wrapping_sub(1);
                self.set_r(n, v);
                self.st.set_t(v == 0);
            }
            Op::Extsb => self.set_r(n, sext8(self.r(m))),
            Op::Extsw => self.set_r(n, sext16(self.r(m))),
            Op::Extub => self.set_r(n, self.r(m) & 0xff),
            Op::Extuw => self.set_r(n, self.r(m) & 0xffff),
            Op::Div0s => {
                let q = self.r(n) >> 31 != 0;
                let mb = self.r(m) >> 31 != 0;
                self.set_qm(q, mb);
                self.st.set_t(q != mb);
            }
            Op::Div0u => {
                self.st.sr &= !(sr::Q | sr::M | sr::T);
            }
            Op::Div1 => self.div1(n, m),
            Op::Mull => self.st.macl = self.r(n).wrapping_mul(self.r(m)),
            Op::Mulsw => {
                let p = i32::from(self.r(n) as u16 as i16) * i32::from(self.r(m) as u16 as i16);
                self.st.macl = p as u32;
            }
            Op::Muluw => {
                self.st.macl = (self.r(n) & 0xffff) * (self.r(m) & 0xffff);
            }
            Op::Dmuls => {
                let p = i64::from(self.r(n) as i32) * i64::from(self.r(m) as i32);
                self.set_mac(p as u64);
            }
            Op::Dmulu => {
                let p = u64::from(self.r(n)) * u64::from(self.r(m));
                self.set_mac(p);
            }
            Op::Macl => self.mac_l(n, m)?,
            Op::Macw => self.mac_w(n, m)?,
            Op::Clrmac => self.set_mac(0),

            // -- logic -----------------------------------------------------------
            Op::And => self.set_r(n, self.r(n) & self.r(m)),
            Op::AndI => self.set_r(0, self.r(0) & isa::i8u(w)),
            Op::Or => self.set_r(n, self.r(n) | self.r(m)),
            Op::OrI => self.set_r(0, self.r(0) | isa::i8u(w)),
            Op::Xor => self.set_r(n, self.r(n) ^ self.r(m)),
            Op::XorI => self.set_r(0, self.r(0) ^ isa::i8u(w)),
            Op::Not => self.set_r(n, !self.r(m)),
            Op::Tst => self.st.set_t(self.r(n) & self.r(m) == 0),
            Op::TstI => self.st.set_t(self.r(0) & isa::i8u(w) == 0),
            Op::TstB => {
                let v = self.read(self.r(0).wrapping_add(self.st.gbr), Width::U8)?;
                self.st.set_t(v & isa::i8u(w) == 0);
            }
            Op::AndB | Op::OrB | Op::XorB => {
                let a = self.r(0).wrapping_add(self.st.gbr);
                // The write is checked before the read happens, so a fault on
                // either leaves memory untouched (§5.6.4, two-access priority).
                self.probe(a, Width::U8, Access::Read)?;
                let target = self.probe(a, Width::U8, Access::Write)?;
                let v = self.read(a, Width::U8)?;
                let imm = isa::i8u(w);
                let r = match op {
                    Op::AndB => v & imm,
                    Op::OrB => v | imm,
                    _ => v ^ imm,
                };
                self.access_write(target, Width::U8, r);
            }
            // §10.1.77: read, test, set bit 7, write back, bus held.
            Op::Tas => {
                let a = self.r(n);
                self.probe(a, Width::U8, Access::Read)?;
                let target = self.probe(a, Width::U8, Access::Write)?;
                let v = self.read(a, Width::U8)?;
                self.access_write(target, Width::U8, v | 0x80);
                self.st.set_t(v & 0xff == 0);
            }

            // -- shifts ----------------------------------------------------------
            Op::Shll | Op::Shal => {
                let v = self.r(n);
                self.st.set_t(v >> 31 != 0);
                self.set_r(n, v << 1);
            }
            Op::Shlr => {
                let v = self.r(n);
                self.st.set_t(v & 1 != 0);
                self.set_r(n, v >> 1);
            }
            Op::Shar => {
                let v = self.r(n);
                self.st.set_t(v & 1 != 0);
                self.set_r(n, ((v as i32) >> 1) as u32);
            }
            Op::Rotl => {
                let v = self.r(n);
                self.st.set_t(v >> 31 != 0);
                self.set_r(n, v.rotate_left(1));
            }
            Op::Rotr => {
                let v = self.r(n);
                self.st.set_t(v & 1 != 0);
                self.set_r(n, v.rotate_right(1));
            }
            Op::Rotcl => {
                let v = self.r(n);
                let t = u32::from(self.st.t());
                self.st.set_t(v >> 31 != 0);
                self.set_r(n, (v << 1) | t);
            }
            Op::Rotcr => {
                let v = self.r(n);
                let t = u32::from(self.st.t());
                self.st.set_t(v & 1 != 0);
                self.set_r(n, (v >> 1) | (t << 31));
            }
            Op::Shll2 => self.set_r(n, self.r(n) << 2),
            Op::Shlr2 => self.set_r(n, self.r(n) >> 2),
            Op::Shll8 => self.set_r(n, self.r(n) << 8),
            Op::Shlr8 => self.set_r(n, self.r(n) >> 8),
            Op::Shll16 => self.set_r(n, self.r(n) << 16),
            Op::Shlr16 => self.set_r(n, self.r(n) >> 16),
            // §10.1.61 and §10.1.64: a non-negative Rm shifts left by its low
            // five bits; a negative one shifts right by `(~Rm & 0x1F) + 1`,
            // which is 32 — all sign, or all zero — when those bits are zero.
            Op::Shad => {
                let (v, s) = (self.r(n), self.r(m));
                let r = if s as i32 >= 0 {
                    v << (s & 0x1f)
                } else if s & 0x1f == 0 {
                    ((v as i32) >> 31) as u32
                } else {
                    ((v as i32) >> ((!s & 0x1f) + 1)) as u32
                };
                self.set_r(n, r);
            }
            Op::Shld => {
                let (v, s) = (self.r(n), self.r(m));
                let r = if s as i32 >= 0 {
                    v << (s & 0x1f)
                } else if s & 0x1f == 0 {
                    0
                } else {
                    v >> ((!s & 0x1f) + 1)
                };
                self.set_r(n, r);
            }

            // -- branches --------------------------------------------------------
            Op::Bt => {
                if self.st.t() {
                    self.next_pc = isa::branch8(pc, w);
                }
            }
            Op::Bf => {
                if !self.st.t() {
                    self.next_pc = isa::branch8(pc, w);
                }
            }
            // §3.1: an untaken BT/S or BF/S still has a delay slot.
            Op::Bts => {
                let target = if self.st.t() {
                    isa::branch8(pc, w)
                } else {
                    pc.wrapping_add(4)
                };
                self.delayed(target);
            }
            Op::Bfs => {
                let target = if self.st.t() {
                    pc.wrapping_add(4)
                } else {
                    isa::branch8(pc, w)
                };
                self.delayed(target);
            }
            Op::Bra => self.delayed(isa::branch12(pc, w)),
            Op::Bsr => {
                self.st.pr = pc.wrapping_add(4);
                self.delayed(isa::branch12(pc, w));
            }
            Op::Braf => self.delayed(pc.wrapping_add(4).wrapping_add(self.r(n))),
            Op::Bsrf => {
                let target = pc.wrapping_add(4).wrapping_add(self.r(n));
                self.st.pr = pc.wrapping_add(4);
                self.delayed(target);
            }
            Op::Jmp => self.delayed(self.r(n)),
            Op::Jsr => {
                let target = self.r(n);
                self.st.pr = pc.wrapping_add(4);
                self.delayed(target);
            }
            Op::Rts => self.delayed(self.st.pr),
            // §10.1.57: SR ← SSR now, so the slot runs under it; PC ← SPC
            // after the slot.
            Op::Rte => {
                let (ssr, spc) = (self.st.ssr, self.st.spc);
                self.st.set_sr(ssr);
                self.delayed(spc);
            }

            // -- system control --------------------------------------------------
            Op::Clrt => self.st.set_t(false),
            Op::Sett => self.st.set_t(true),
            Op::Clrs => self.st.sr &= !sr::S,
            Op::Sets => self.st.sr |= sr::S,
            Op::Nop | Op::Synco | Op::Prefi | Op::Icbi => {}
            Op::Pref => {
                let a = self.r(n);
                // Prefetch raises nothing — except on the store-queue area,
                // where it is the flush and is checked like a read (§10.1.51,
                // §8.7.3, §8.7.4).
                if a >> 26 == 0x38 {
                    if !self.st.privileged() && !self.sq_user_ok() {
                        return Err(Trap::at(exc::ADDR_READ, vector::GENERAL, a));
                    }
                    self.flush_store_queue(a)?;
                }
            }
            // §7.5.5: OCBP and OCBWB check as reads, OCBI as a write.
            Op::Ocbi => {
                self.probe(self.r(n) & !0x1f, Width::U8, Access::Write)?;
            }
            Op::Ocbp | Op::Ocbwb => {
                self.probe(self.r(n) & !0x1f, Width::U8, Access::Read)?;
            }
            Op::Sleep => self.st.sleeping = true,
            Op::Trapa => {
                self.st.ctl.tra = isa::i8u(w) << 2;
                return Err(Trap {
                    code: exc::TRAPA,
                    vector: vector::GENERAL,
                    tea: None,
                    kind: Kind::Completion,
                });
            }
            Op::Ldtlb => self.ldtlb(),
            Op::LdcSr => {
                let v = self.r(n);
                self.st.set_sr(v);
            }
            Op::LdcGbr => self.st.gbr = self.r(n),
            Op::LdcVbr => self.st.vbr = self.r(n),
            Op::LdcSsr => self.st.ssr = self.r(n),
            Op::LdcSpc => self.st.spc = self.r(n),
            Op::LdcSgr => self.st.sgr = self.r(n),
            Op::LdcDbr => self.st.dbr = self.r(n),
            Op::LdcBank => self.st.bank[isa::bank(w)] = self.r(n),
            Op::LdsMach => self.st.mach = self.r(n),
            Op::LdsMacl => self.st.macl = self.r(n),
            Op::LdsPr => self.st.pr = self.r(n),
            Op::LdsFpul => self.st.fpul = self.r(n),
            Op::LdsFpscr => self.st.fpscr = self.r(n) & fpscr::WRITABLE,
            Op::LdclSr => {
                let v = self.pop(n)?;
                self.st.set_sr(v);
            }
            Op::LdclGbr => self.st.gbr = self.pop(n)?,
            Op::LdclVbr => self.st.vbr = self.pop(n)?,
            Op::LdclSsr => self.st.ssr = self.pop(n)?,
            Op::LdclSpc => self.st.spc = self.pop(n)?,
            Op::LdclSgr => self.st.sgr = self.pop(n)?,
            Op::LdclDbr => self.st.dbr = self.pop(n)?,
            Op::LdclBank => {
                let v = self.pop(n)?;
                self.st.bank[isa::bank(w)] = v;
            }
            Op::LdslMach => self.st.mach = self.pop(n)?,
            Op::LdslMacl => self.st.macl = self.pop(n)?,
            Op::LdslPr => self.st.pr = self.pop(n)?,
            Op::LdslFpul => self.st.fpul = self.pop(n)?,
            Op::LdslFpscr => self.st.fpscr = self.pop(n)? & fpscr::WRITABLE,
            Op::StcSr => self.set_r(n, self.st.sr),
            Op::StcGbr => self.set_r(n, self.st.gbr),
            Op::StcVbr => self.set_r(n, self.st.vbr),
            Op::StcSsr => self.set_r(n, self.st.ssr),
            Op::StcSpc => self.set_r(n, self.st.spc),
            Op::StcSgr => self.set_r(n, self.st.sgr),
            Op::StcDbr => self.set_r(n, self.st.dbr),
            Op::StcBank => self.set_r(n, self.st.bank[isa::bank(w)]),
            Op::StsMach => self.set_r(n, self.st.mach),
            Op::StsMacl => self.set_r(n, self.st.macl),
            Op::StsPr => self.set_r(n, self.st.pr),
            Op::StsFpul => self.set_r(n, self.st.fpul),
            Op::StsFpscr => self.set_r(n, self.st.fpscr & fpscr::WRITABLE),
            Op::StclSr => self.push(n, Width::U32, self.st.sr)?,
            Op::StclGbr => self.push(n, Width::U32, self.st.gbr)?,
            Op::StclVbr => self.push(n, Width::U32, self.st.vbr)?,
            Op::StclSsr => self.push(n, Width::U32, self.st.ssr)?,
            Op::StclSpc => self.push(n, Width::U32, self.st.spc)?,
            Op::StclSgr => self.push(n, Width::U32, self.st.sgr)?,
            Op::StclDbr => self.push(n, Width::U32, self.st.dbr)?,
            Op::StclBank => self.push(n, Width::U32, self.st.bank[isa::bank(w)])?,
            Op::StslMach => self.push(n, Width::U32, self.st.mach)?,
            Op::StslMacl => self.push(n, Width::U32, self.st.macl)?,
            Op::StslPr => self.push(n, Width::U32, self.st.pr)?,
            Op::StslFpul => self.push(n, Width::U32, self.st.fpul)?,
            Op::StslFpscr => self.push(n, Width::U32, self.st.fpscr & fpscr::WRITABLE)?,

            // Every FPU operation is routed to `fpu` before getting here.
            _ => return Err(self.illegal()),
        }
        Ok(())
    }

    /// Whether user mode may use the store queues (`MMUCR.SQMD`, §8.7.4).
    fn sq_user_ok(&self) -> bool {
        self.st.ctl.mmucr & mmu::mmucr::SQMD == 0
    }

    /// `Rn -= size; (Rn) ← value`, with the register moved only once the
    /// store has succeeded.
    pub(super) fn push(&mut self, n: usize, width: Width, value: u32) -> Result<(), Trap> {
        let a = self.r(n).wrapping_sub(width.bytes() as u32);
        self.write(a, width, value)?;
        self.set_r(n, a);
        Ok(())
    }

    /// `value ← (Rm); Rm += 4`.
    fn pop(&mut self, m: usize) -> Result<u32, Trap> {
        let a = self.r(m);
        let v = self.read(a, Width::U32)?;
        self.set_r(m, a.wrapping_add(4));
        Ok(v)
    }

    /// The post-increment of `@Rm+,Rn`: skipped when `m == n`, so the loaded
    /// value is what the register ends up holding.
    fn post_inc(&mut self, m: usize, n: usize, by: u32) {
        if m != n {
            self.set_r(m, self.r(m).wrapping_add(by));
        }
    }

    /// Set `SR.Q` and `SR.M`.
    fn set_qm(&mut self, q: bool, m: bool) {
        let mut v = self.st.sr & !(sr::Q | sr::M);
        if q {
            v |= sr::Q;
        }
        if m {
            v |= sr::M;
        }
        self.st.sr = v;
    }

    /// `MACH:MACL ← value`.
    fn set_mac(&mut self, value: u64) {
        self.st.mach = (value >> 32) as u32;
        self.st.macl = value as u32;
    }

    /// `DIV1`, transcribed from the C description in §10.1.17.
    fn div1(&mut self, n: usize, m: usize) {
        let old_q = self.st.sr & sr::Q != 0;
        let mbit = self.st.sr & sr::M != 0;
        let mut q = self.r(n) >> 31 != 0;
        let divisor = self.r(m);
        let mut rn = (self.r(n) << 1) | u32::from(self.st.t());
        let before = rn;
        if old_q == mbit {
            rn = rn.wrapping_sub(divisor);
            let borrow = rn > before;
            q = if q == mbit { borrow } else { !borrow };
        } else {
            rn = rn.wrapping_add(divisor);
            let carry = rn < before;
            q = if q == mbit { carry } else { !carry };
        }
        self.set_r(n, rn);
        self.set_qm(q, mbit);
        self.st.set_t(q == mbit);
    }

    /// `MAC.L @Rm+,@Rn+` (§10.1.28). Both reads happen before either register
    /// moves; `Rn` is read first, then `Rm` — at `Rn + 4` when they are the
    /// same register.
    fn mac_l(&mut self, n: usize, m: usize) -> Result<(), Trap> {
        let an = self.r(n);
        let am = if m == n {
            an.wrapping_add(4)
        } else {
            self.r(m)
        };
        let vn = self.read(an, Width::U32)? as i32;
        let vm = self.read(am, Width::U32)? as i32;
        self.set_r(n, an.wrapping_add(4));
        self.set_r(m, am.wrapping_add(4));
        let product = i64::from(vn) * i64::from(vm);
        let mac = ((u64::from(self.st.mach) << 32) | u64::from(self.st.macl)) as i64;
        if self.st.sr & sr::S == 0 {
            self.set_mac(mac.wrapping_add(product) as u64);
        } else {
            // "The addition to the MAC register contents is a saturation
            // operation at the 48th bit": only the low 48 bits of MAC are
            // valid, and the result is limited to H'FFFF8000_00000000 ..
            // H'00007FFF_FFFFFFFF. MACH's top half is left alone, as the C
            // description's final `MACH = (Res2 & 0xFFFF) | (MACH &
            // 0xFFFF0000)` has it.
            let acc = (mac << 16) >> 16;
            let sum = acc + product;
            let sat = sum.clamp(-(1i64 << 47), (1i64 << 47) - 1);
            self.st.mach = (self.st.mach & 0xffff_0000) | ((sat >> 32) as u32 & 0xffff);
            self.st.macl = sat as u32;
        }
        Ok(())
    }

    /// `MAC.W @Rm+,@Rn+` (§10.1.29).
    fn mac_w(&mut self, n: usize, m: usize) -> Result<(), Trap> {
        let an = self.r(n);
        let am = if m == n {
            an.wrapping_add(2)
        } else {
            self.r(m)
        };
        let vn = self.read(an, Width::U16)? as u16 as i16;
        let vm = self.read(am, Width::U16)? as u16 as i16;
        self.set_r(n, an.wrapping_add(2));
        self.set_r(m, am.wrapping_add(2));
        let product = i32::from(vn) * i32::from(vm);
        if self.st.sr & sr::S == 0 {
            let mac = ((u64::from(self.st.mach) << 32) | u64::from(self.st.macl)) as i64;
            self.set_mac(mac.wrapping_add(i64::from(product)) as u64);
        } else {
            // A 32-bit saturating accumulate in MACL. The description says
            // that "if overflow occurs, the LSB of the MACH register is set to
            // 1"; the C description omits that line. The prose is followed,
            // because it is the one that describes an observable effect.
            let sum = i64::from(self.st.macl as i32) + i64::from(product);
            if sum > i64::from(i32::MAX) {
                self.st.macl = 0x7fff_ffff;
                self.st.mach |= 1;
            } else if sum < i64::from(i32::MIN) {
                self.st.macl = 0x8000_0000;
                self.st.mach |= 1;
            } else {
                self.st.macl = sum as u32;
            }
        }
        Ok(())
    }

    /// `MOVUA.L`: a longword from any address, assembled a byte at a time in
    /// the configured byte order when it is not aligned (§10.1.39 — no
    /// address error for a misaligned address).
    fn read_unaligned(&mut self, a: u32) -> Result<u32, Trap> {
        if a & 3 == 0 {
            return self.read(a, Width::U32);
        }
        let mut bytes = [0u8; 4];
        // Every byte is translated before any is read, so a fault on the
        // second page leaves no access behind on the first.
        let mut targets = [Target::Bus(0); 4];
        for (i, t) in targets.iter_mut().enumerate() {
            *t = self.resolve(a.wrapping_add(i as u32), Access::Read)?;
        }
        for (b, t) in bytes.iter_mut().zip(targets) {
            *b = self.access_read(t, Width::U8, false) as u8;
        }
        Ok(match self.cfg.endian {
            Endian::Big => u32::from_be_bytes(bytes),
            Endian::Little => u32::from_le_bytes(bytes),
        })
    }

    /// `LDTLB`: PTEH and PTEL into the UTLB entry `MMUCR.URC` names (§7.4.3
    /// and figure 7.11 — the page-management bits come from PTEL).
    fn ldtlb(&mut self) {
        let se = self.st.ctl.se();
        let (pteh, ptel) = (self.st.ctl.pteh, self.st.ctl.ptel);
        let i = self.st.ctl.urc();
        self.st.mmu.utlb[i] = UtlbEntry {
            vpn: pteh & 0xffff_fc00,
            asid: pteh as u8,
            ppn: ptel & ppn_mask(se),
            sz: size_bits(ptel),
            pr: ((ptel >> 5) & 3) as u8,
            v: ptel & 0x100 != 0,
            sh: ptel & 0x2 != 0,
            c: ptel & 0x8 != 0,
            d: ptel & 0x4 != 0,
            wt: ptel & 0x1 != 0,
            ub: se && ptel & 0x200 != 0,
        };
    }
}

/// Route a translated physical address: the control-register area (area 7
/// through the TLB in 29-bit mode, `0xFC00_0000..` in 32-bit mode) goes to
/// the P4 handler at the address it aliases (§7.1.1, §7.7.2 item 3).
#[inline]
fn physical(phys: u32, se: bool) -> Target {
    if se {
        if phys >= 0xfc00_0000 {
            Target::P4(phys)
        } else {
            Target::Bus(u64::from(phys))
        }
    } else if phys & 0x1fff_ffff >= 0x1c00_0000 {
        Target::P4((phys & 0x1fff_ffff) | 0xe000_0000)
    } else {
        Target::Bus(u64::from(phys & 0x1fff_ffff))
    }
}

/// The PPN bits a TLB entry holds: 28–10, or 31–10 in 32-bit mode (§7.7.6).
#[inline]
const fn ppn_mask(se: bool) -> u32 {
    if se { 0xffff_fc00 } else { 0x1fff_fc00 }
}

/// The `SZ` field of a PTEL or memory-mapped data word: bit 7 is `SZ1`, bit
/// 4 is `SZ0`.
#[inline]
const fn size_bits(v: u32) -> u8 {
    (((v >> 6) & 2) | ((v >> 4) & 1)) as u8
}

/// A UTLB entry as its data-array word (§7.6.4).
fn utlb_data(e: &UtlbEntry, se: bool) -> u32 {
    e.ppn
        | (u32::from(se && e.ub) << 9)
        | (u32::from(e.v) << 8)
        | (u32::from(e.sz >> 1) << 7)
        | (u32::from(e.pr) << 5)
        | (u32::from(e.sz & 1) << 4)
        | (u32::from(e.c) << 3)
        | (u32::from(e.d) << 2)
        | (u32::from(e.sh) << 1)
        | u32::from(e.wt)
}

/// A narrower read of a 32-bit register. The manual makes every one of them
/// "32-bit access only" and leaves other widths undefined; this answers the
/// addressed bytes as a little-endian register would, rather than inventing a
/// fault the CPU has no way to raise.
fn narrow(v: u32, a: u32, width: Width) -> u32 {
    match width {
        Width::U8 => (v >> (8 * (a & 3))) & 0xff,
        Width::U16 => (v >> (8 * (a & 2))) & 0xffff,
        _ => v,
    }
}

/// Sign-extend a byte.
#[inline]
const fn sext8(v: u32) -> u32 {
    v as u8 as i8 as i32 as u32
}

/// Sign-extend a word.
#[inline]
const fn sext16(v: u32) -> u32 {
    v as u16 as i16 as i32 as u32
}
