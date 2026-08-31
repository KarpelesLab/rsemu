//! The ARMv7E-M interpreter.
//!
//! # The timing model, and what it does not claim
//!
//! A Cortex-M4 is a three-stage pipeline with a prefetch unit and, on an M7,
//! a superscalar one with caches. Neither is the core's to model: the wait
//! states, the flash accelerator and (on M7) the caches belong to the SoC.
//! So the model is stated rather than implied, and it is deliberately simple:
//!
//! | Contribution | Cycles |
//! | --- | --- |
//! | Every instruction | 1 |
//! | Each data access | 1 |
//! | A taken branch, or any other write to the PC | +2 (the pipeline refill) |
//! | Exception entry | 12, the architected latency of a stacking sequence |
//! | Exception return | 10 |
//! | `SDIV` / `UDIV` | +2 (the M4's 2–12 cycle divider, at its fast end) |
//!
//! That reproduces the Cortex-M4 TRM's published figures for the common
//! cases — `LDR` is 2, `STR` is 2, a taken branch is 3, `LDM` of *n*
//! registers is *n* + 1 — without pretending to model the pipeline. What is
//! **not** modelled: the prefetch unit's speculative fetches (we fetch
//! exactly what we execute), branch prediction, wait states, and the M7's
//! caches and dual issue.
//!
//! # `PC` reads as the instruction plus four
//!
//! Kept literally, as the ARMv5TE core does: after the fetch `r[15]` holds
//! the instruction's address plus four, so every register read is a plain
//! array index. That is true of both encoding widths — a thirty-two-bit
//! instruction still reads the PC as its own address plus four, not plus six
//! (DDI 0403 A5.1.2). If the instruction did not write the PC, the last thing
//! [`Exec::step`] does is set it to the next instruction's address.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403: A2.3 (`BranchTo`
//! and the PC-writing rules), A5 (encodings), A7.3 (`IT` and
//! `ConditionPassed`), A7.7 (per-instruction pseudocode), B1.4 (registers and
//! execution modes), B1.5 (the exception model), B3.5 (the MPU). Cycle counts
//! from the Cortex-M4 and Cortex-M7 Technical Reference Manuals' instruction
//! timing tables. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::{Endian, Width};

use super::isa::{
    BitfieldOp, Cond, DpOp, DualMulOp, ExtendOp, HalfMulOp, HintOp, Insn, MemOffset, MiscOp,
    Operand, SatQOp, Shift, ShiftType, Size, decode, is_32bit,
};
use super::sys::{Access, Exception, MPU_REGIONS, Sys, ccr, control, exc_return, fsr, in_ppb};
use super::{Config, xpsr};

/// A fault on its way back up to [`Exec::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Trap {
    /// Which exception this fault raises, before escalation.
    pub exc: Exception,
    /// The `CFSR` bits to set.
    pub status: u32,
    /// The address to put in `MMFAR` or `BFAR`, if the fault has one.
    pub far: Option<u32>,
}

impl Trap {
    /// An undefined instruction.
    const UNDEFINED: Trap = Trap {
        exc: Exception::USAGE_FAULT,
        status: fsr::UF_UNDEFINSTR,
        far: None,
    };
    /// An attempt to execute in a state this architecture does not have.
    const INVSTATE: Trap = Trap {
        exc: Exception::USAGE_FAULT,
        status: fsr::UF_INVSTATE,
        far: None,
    };
    /// An illegal exception return.
    const INVPC: Trap = Trap {
        exc: Exception::USAGE_FAULT,
        status: fsr::UF_INVPC,
        far: None,
    };
    /// A coprocessor instruction with no coprocessor behind it.
    const NOCP: Trap = Trap {
        exc: Exception::USAGE_FAULT,
        status: fsr::UF_NOCP,
        far: None,
    };
}

/// `Ok`, or "this instruction faulted; abandon it".
pub(super) type Ex<T = ()> = core::result::Result<T, Trap>;

/// The architectural state one core owns.
///
/// Separate from [`ArmV7m`](super::ArmV7m) because the interrupt *inputs*
/// live outside the lock: a device asserting an IRQ from inside an MMIO write
/// the CPU itself issued would otherwise re-enter the CPU's own critical
/// section (the re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone)]
pub(super) struct State {
    /// The sixteen visible registers. `r[13]` is whichever stack pointer is
    /// currently selected; `r[15]` is the PC.
    pub r: [u32; 16],
    /// The stack pointer that is *not* currently in `r[13]`.
    pub sp_other: u32,
    /// Whether `r[13]` currently holds the process stack pointer.
    pub sp_is_psp: bool,
    /// `xPSR`: `APSR`, `IPSR` and `EPSR` in one word.
    pub xpsr: u32,
    /// `PRIMASK.PM`.
    pub primask: bool,
    /// `FAULTMASK.FM`.
    pub faultmask: bool,
    /// `BASEPRI`.
    pub basepri: u8,
    /// `CONTROL`.
    pub control: u32,
    /// The NVIC, SCB, SysTick and MPU.
    pub sys: Sys,
    /// Cycles executed since power-on.
    pub cycles: u64,
    /// Sleeping in `WFI` or `WFE`.
    pub asleep: bool,
    /// The event register `SEV` sets and `WFE` consumes.
    pub event: bool,
    /// A reset sequence is owed.
    pub reset_pending: bool,
    /// The core is locked up: a fault was taken at a priority no handler can
    /// preempt, and nothing but reset gets out (DDI 0403 B1.5.15).
    pub locked_up: bool,
    /// The local exclusive monitor: the address a successful `LDREX` tagged.
    pub exclusive: Option<u32>,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u32,
    /// The comment field of the last `SVC` executed.
    pub last_svc: u8,
    /// The comment field of the last `BKPT` executed.
    pub last_bkpt: u8,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) fn new(cfg: &Config) -> State {
        State {
            r: [0; 16],
            sp_other: 0,
            sp_is_psp: false,
            // Thread mode, Thumb state. `EPSR.T` must be set or the very
            // first instruction is an INVSTATE UsageFault.
            xpsr: xpsr::T,
            primask: false,
            faultmask: false,
            basepri: 0,
            control: 0,
            sys: Sys::new(
                cfg.cpuid,
                cfg.priority_bits,
                if cfg.ext.mpu { MPU_REGIONS as u8 } else { 0 },
            ),
            cycles: 0,
            asleep: false,
            event: false,
            reset_pending: true,
            locked_up: false,
            exclusive: None,
            faults: 0,
            last_fault: 0,
            last_svc: 0,
            last_bkpt: 0,
        }
    }

    /// Whether the core is in Handler mode.
    #[inline]
    pub(super) const fn in_handler(&self) -> bool {
        self.xpsr & xpsr::EXCEPTION != 0
    }

    /// The exception currently being handled, or [`Exception::THREAD`].
    #[inline]
    pub(super) const fn current_exception(&self) -> Exception {
        Exception((self.xpsr & xpsr::EXCEPTION) as u16)
    }

    /// Whether the current mode is privileged.
    ///
    /// Handler mode always is; Thread mode is unless `CONTROL.nPRIV` says
    /// otherwise (DDI 0403 B1.4.1).
    #[inline]
    pub(super) const fn privileged(&self) -> bool {
        self.in_handler() || self.control & control::NPRIV == 0
    }

    /// `ITSTATE`, gathered from the two places `EPSR` keeps it.
    #[inline]
    pub(super) const fn itstate(&self) -> u8 {
        (((self.xpsr >> 10) & 0x3f) << 2) as u8 | ((self.xpsr >> 25) & 3) as u8
    }

    /// Write `ITSTATE` back into `EPSR`.
    #[inline]
    pub(super) const fn set_itstate(&mut self, it: u8) {
        let mask = (0x3f << 10) | (3 << 25);
        let it = it as u32;
        self.xpsr = (self.xpsr & !mask) | ((it >> 2) << 10) | ((it & 3) << 25);
    }

    /// `ITAdvance()` (DDI 0403 A7.3.2).
    #[inline]
    pub(super) const fn it_advance(&mut self) {
        let it = self.itstate();
        if it & 0b111 == 0 {
            self.set_itstate(0);
        } else {
            self.set_itstate((it & 0b1110_0000) | ((it << 1) & 0b0001_1111));
        }
    }

    /// The condition governing the next instruction.
    #[inline]
    pub(super) const fn current_cond(&self) -> Cond {
        let it = self.itstate();
        if it == 0 { Cond::AL } else { Cond(it >> 4) }
    }

    /// Move `r[13]` to the bank the current mode and `CONTROL.SPSEL` select.
    ///
    /// Handler mode always uses the main stack whatever `SPSEL` says, which
    /// is why this cannot be derived from `CONTROL` alone
    /// (DDI 0403 B1.4.1).
    pub(super) const fn sync_stack(&mut self) {
        let want_psp = !self.in_handler() && self.control & control::SPSEL != 0;
        if want_psp != self.sp_is_psp {
            let tmp = self.r[13];
            self.r[13] = self.sp_other;
            self.sp_other = tmp;
            self.sp_is_psp = want_psp;
        }
    }

    /// The main stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn msp(&self) -> u32 {
        if self.sp_is_psp {
            self.sp_other
        } else {
            self.r[13]
        }
    }

    /// The process stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn psp(&self) -> u32 {
        if self.sp_is_psp {
            self.r[13]
        } else {
            self.sp_other
        }
    }

    /// Write the main stack pointer.
    pub(super) const fn set_msp(&mut self, value: u32) {
        if self.sp_is_psp {
            self.sp_other = value;
        } else {
            self.r[13] = value;
        }
    }

    /// Write the process stack pointer.
    pub(super) const fn set_psp(&mut self, value: u32) {
        if self.sp_is_psp {
            self.r[13] = value;
        } else {
            self.sp_other = value;
        }
    }

    /// The current execution priority (DDI 0403 B1.5.4's
    /// `ExecutionPriority`).
    ///
    /// The lowest of: the priority of the most urgent active exception, zero
    /// if `PRIMASK` is set, minus one if `FAULTMASK` is set, and `BASEPRI`'s
    /// group priority if it is non-zero.
    #[must_use]
    pub(super) fn execution_priority(&self) -> i32 {
        let mut prio = self.sys.active_priority();
        if self.faultmask {
            prio = prio.min(-1);
        }
        if self.primask {
            prio = prio.min(0);
        }
        if self.basepri != 0 {
            prio = prio.min(i32::from(self.basepri & self.sys.group_mask()));
        }
        prio
    }
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    cfg: &'a Config,
    attrs: MemAttrs,
    /// Address of the instruction being executed.
    insn_addr: u32,
    /// Set when the instruction wrote the PC, so the fall-through advance is
    /// skipped.
    branched: bool,
    /// Cycles this step has charged.
    used: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(state: &'a mut State, space: &'a AddressSpace, cfg: &'a Config) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        Exec {
            state,
            space,
            cfg,
            attrs,
            insn_addr: 0,
            branched: false,
            used: 0,
        }
    }

    /// Run one reset sequence, one exception entry, or one instruction.
    ///
    /// `external` is the level of each external interrupt input, sampled once
    /// outside the lock. Level-sensitive inputs re-pend for as long as they
    /// are asserted, which is what a peripheral that has not been serviced
    /// does; an edge-triggered source pends itself through `NVIC_ISPR` or
    /// `STIR` instead.
    pub(super) fn step(&mut self, external: &[u32]) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }
        if self.state.locked_up {
            self.cycle(1);
            self.tick_time();
            return self.used;
        }

        self.merge_external(external);

        // Arbitration happens before anything else in the step, so an
        // exception that arrives while a lower-priority one is being taken is
        // simply the one that wins here. That collapses *late arrival* into
        // ordinary priority selection: stacking is atomic in this model, so
        // there is no window for an arrival to land in the middle of it.
        if let Some((exc, prio)) = self.state.sys.highest_pending()
            && prio < self.state.execution_priority()
        {
            let ret = self.state.r[15];
            self.state.sys.set_pending(exc, false);
            self.exception_entry(exc, ret);
            self.tick_time();
            return self.used;
        }

        if self.state.asleep {
            // `WFI` wakes on any *enabled* exception becoming pending, even
            // one the masks would stop from being taken — the wake-up and the
            // exception are separate things (DDI 0403 B1.5.17).
            if self.state.sys.highest_pending().is_some() {
                self.state.asleep = false;
            } else {
                self.cycle(1);
                self.tick_time();
                return self.used;
            }
        }

        self.execute_one();
        self.tick_time();
        self.used
    }

    /// Fold the external interrupt levels into the NVIC's pending bits.
    fn merge_external(&mut self, external: &[u32]) {
        for (i, word) in external.iter().enumerate() {
            if *word == 0 {
                continue;
            }
            let mut bits = *word;
            while bits != 0 {
                let bit = bits.trailing_zeros();
                bits &= bits - 1;
                let n = i * 32 + bit as usize + 16;
                if n < Exception::COUNT {
                    self.state.sys.set_pending(Exception(n as u16), true);
                }
            }
        }
    }

    /// Advance SysTick by the cycles this step charged, pending the exception
    /// if the counter wrapped and `TICKINT` is set.
    fn tick_time(&mut self) {
        if self.state.sys.tick_systick(self.used) && self.state.sys.syst_csr & 0b10 != 0 {
            self.state.sys.set_pending(Exception::SYSTICK, true);
        }
    }

    /// The reset sequence: `SP` from vector zero, `PC` from vector one
    /// (DDI 0403 B1.5.5).
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.asleep = false;
        self.state.locked_up = false;
        self.state.xpsr = xpsr::T;
        self.state.control = 0;
        self.state.primask = false;
        self.state.faultmask = false;
        self.state.basepri = 0;
        self.state.sp_is_psp = false;
        self.state.sys.active = [0; Exception::COUNT / 32];
        self.state.sys.pending = [0; Exception::COUNT / 32];
        let base = self.state.sys.vtor;
        // The reset vector's stack pointer has its bottom two bits ignored;
        // an entry with `EPSR.T` clear is a HardFault on the first fetch, and
        // that is exactly what leaving `T` unset here produces.
        let sp = self.read_vector(base).unwrap_or(0);
        let pc = self.read_vector(base.wrapping_add(4)).unwrap_or(0);
        self.state.r[13] = sp & !3;
        self.state.sp_other = 0;
        self.state.r[15] = pc & !1;
        if pc & 1 == 0 {
            self.state.xpsr &= !xpsr::T;
        }
        self.cycle(2);
    }

    /// Read one vector table entry, without the MPU or the fault machinery.
    fn read_vector(&mut self, addr: u32) -> Option<u32> {
        self.space
            .read(u64::from(addr), Width::U32, self.attrs)
            .ok()
            .map(|v| self.to_cpu_order(addr, Width::U32, v as u32))
    }

    // -----------------------------------------------------------------
    // Cycles, flags, registers
    // -----------------------------------------------------------------

    fn cycle(&mut self, n: u64) {
        self.used += n;
        self.state.cycles = self.state.cycles.wrapping_add(n);
    }

    #[inline]
    fn flag(&self, mask: u32) -> bool {
        self.state.xpsr & mask != 0
    }

    #[inline]
    fn set_flag(&mut self, mask: u32, on: bool) {
        if on {
            self.state.xpsr |= mask;
        } else {
            self.state.xpsr &= !mask;
        }
    }

    fn set_nz(&mut self, value: u32) {
        self.set_flag(xpsr::N, value & 0x8000_0000 != 0);
        self.set_flag(xpsr::Z, value == 0);
    }

    #[inline]
    fn reg(&self, index: u8) -> u32 {
        self.state.r[(index & 0xf) as usize]
    }

    /// Write a register. Writing the PC here is *not* a branch: every
    /// instruction that can write the PC in ARMv7-M goes through
    /// [`Exec::bx_write_pc`] or [`Exec::branch_write_pc`] instead, so a plain
    /// write of `R15` can only come from an encoding the architecture calls
    /// UNPREDICTABLE. Dropping it keeps such an encoding from silently
    /// becoming a wild branch.
    #[inline]
    fn set_reg(&mut self, index: u8, value: u32) {
        let index = (index & 0xf) as usize;
        if index != 15 {
            self.state.r[index] = value;
        }
    }

    /// `BranchWritePC`: stay in Thumb state and drop bit zero
    /// (DDI 0403 A2.3.1).
    fn branch_write_pc(&mut self, target: u32) {
        self.state.r[15] = target & !1;
        self.branched = true;
        self.cycle(2);
    }

    /// `BXWritePC`: bit zero must be set, or the core would be asking for ARM
    /// state, which this architecture does not have. In Handler mode a target
    /// in the top sixteenth of the address space is an exception return
    /// instead (DDI 0403 A2.3.1, B1.5.8).
    fn bx_write_pc(&mut self, target: u32) -> Ex {
        if self.state.in_handler() && exc_return::is_magic(target) {
            return self.exception_return(target);
        }
        if target & 1 == 0 {
            return Err(Trap::INVSTATE);
        }
        self.state.r[15] = target & !1;
        self.branched = true;
        self.cycle(2);
        Ok(())
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// The byte order to present a multi-byte value in.
    fn to_cpu_order(&self, addr: u32, width: Width, value: u32) -> u32 {
        if self.cfg.endian == Endian::Little
            || self.space.endian_at(u64::from(addr)) == self.cfg.endian
        {
            return value;
        }
        match width {
            Width::U8 => value,
            Width::U16 => u32::from((value as u16).swap_bytes()),
            _ => value.swap_bytes(),
        }
    }

    /// Turn a refused access into the right fault.
    ///
    /// A refused *fetch* is `IBUSERR`; a refused data access is `PRECISERR`
    /// with `BFAR` valid. This core has no write buffer, so there is no
    /// imprecise case to report (DDI 0403 B1.5.14).
    fn bus_fault(&mut self, addr: u32, access: Access) -> Trap {
        self.state.faults = self.state.faults.wrapping_add(1);
        self.state.last_fault = addr;
        match access {
            Access::Fetch => Trap {
                exc: Exception::BUS_FAULT,
                status: fsr::BF_IBUSERR,
                far: None,
            },
            _ => Trap {
                exc: Exception::BUS_FAULT,
                status: fsr::BF_PRECISERR | fsr::BF_BFARVALID,
                far: Some(addr),
            },
        }
    }

    /// The MemManage fault an MPU refusal raises.
    fn mpu_fault(&self, addr: u32, access: Access) -> Trap {
        Trap {
            exc: Exception::MEM_MANAGE,
            status: if access == Access::Fetch {
                fsr::MM_IACCVIOL
            } else {
                fsr::MM_DACCVIOL | fsr::MM_MMARVALID
            },
            far: if access == Access::Fetch {
                None
            } else {
                Some(addr)
            },
        }
    }

    /// Check the MPU for an access of `bytes` bytes at `addr`.
    ///
    /// Both ends are checked, because an unaligned access can straddle a
    /// region boundary and the architecture faults if either half is refused.
    fn check_mpu(&mut self, addr: u32, bytes: u32, access: Access, privileged: bool) -> Ex {
        let prio = self.state.execution_priority();
        let last = addr.wrapping_add(bytes - 1);
        if self.state.sys.mpu_permits(addr, access, privileged, prio)
            && self.state.sys.mpu_permits(last, access, privileged, prio)
        {
            return Ok(());
        }
        Err(self.mpu_fault(addr, access))
    }

    /// Reject an unaligned access when `CCR.UNALIGN_TRP` asks, or when the
    /// instruction is one the architecture never allows to be unaligned.
    fn check_alignment(&self, addr: u32, bytes: u32, always: bool) -> Ex {
        if bytes > 1
            && !addr.is_multiple_of(bytes)
            && (always || self.state.sys.ccr & ccr::UNALIGN_TRP != 0)
        {
            return Err(Trap {
                exc: Exception::USAGE_FAULT,
                status: fsr::UF_UNALIGNED,
                far: None,
            });
        }
        Ok(())
    }

    /// Read `bytes` bytes, honouring the private peripheral bus, the MPU and
    /// unaligned support.
    fn read_mem(&mut self, addr: u32, bytes: u32, privileged: bool) -> Ex<u32> {
        self.cycle(1);
        if in_ppb(addr) {
            return self.read_ppb(addr, bytes);
        }
        self.check_mpu(addr, bytes, Access::Read, privileged)?;
        let attrs = self.attrs.with_privileged(privileged);
        if addr.is_multiple_of(bytes) {
            let width = width_of(bytes);
            return match self.space.read(u64::from(addr), width, attrs) {
                Ok(v) => Ok(self.to_cpu_order(addr, width, v as u32)),
                Err(_) => Err(self.bus_fault(addr, Access::Read)),
            };
        }
        // Unaligned: assemble byte by byte, which is what byte-invariant
        // addressing means and what makes big-endian BE-8 fall out for free.
        let mut value = 0u32;
        for k in 0..bytes {
            let at = addr.wrapping_add(k);
            let byte = match self.space.read(u64::from(at), Width::U8, attrs) {
                Ok(v) => v as u32,
                Err(_) => return Err(self.bus_fault(addr, Access::Read)),
            };
            let lane = if self.cfg.endian == Endian::Little {
                k
            } else {
                bytes - 1 - k
            };
            value |= byte << (8 * lane);
        }
        Ok(value)
    }

    /// Write `bytes` bytes, honouring the private peripheral bus, the MPU and
    /// unaligned support.
    fn write_mem(&mut self, addr: u32, bytes: u32, value: u32, privileged: bool) -> Ex {
        self.cycle(1);
        if in_ppb(addr) {
            return self.write_ppb(addr, bytes, value);
        }
        self.check_mpu(addr, bytes, Access::Write, privileged)?;
        let attrs = self.attrs.with_privileged(privileged);
        if addr.is_multiple_of(bytes) {
            let width = width_of(bytes);
            let value = self.to_cpu_order(addr, width, value);
            return match self
                .space
                .write(u64::from(addr), width, u64::from(value), attrs)
            {
                Ok(()) => Ok(()),
                Err(_) => Err(self.bus_fault(addr, Access::Write)),
            };
        }
        for k in 0..bytes {
            let at = addr.wrapping_add(k);
            let lane = if self.cfg.endian == Endian::Little {
                k
            } else {
                bytes - 1 - k
            };
            let byte = u64::from((value >> (8 * lane)) & 0xff);
            if self
                .space
                .write(u64::from(at), Width::U8, byte, attrs)
                .is_err()
            {
                return Err(self.bus_fault(addr, Access::Write));
            }
        }
        Ok(())
    }

    /// Read from the private peripheral bus.
    ///
    /// Sub-word reads take the containing word and extract, which is what the
    /// byte-addressable priority registers need and what the architecture
    /// permits for them (DDI 0403 B3.2.3).
    fn read_ppb(&mut self, addr: u32, bytes: u32) -> Ex<u32> {
        let Some(word) = self.state.sys.read_word(addr, self.attrs.debug) else {
            return Err(self.bus_fault(addr, Access::Read));
        };
        Ok(extract(word, addr, bytes))
    }

    /// Write to the private peripheral bus.
    ///
    /// A sub-word write is a read-modify-write of the containing word. That
    /// is exactly right for `NVIC_IPR` and `SHPR1-3`, the only registers the
    /// architecture guarantees byte access to; for the write-one-to-set
    /// registers a sub-word write is not architecturally defined and this is
    /// simply one legal answer.
    fn write_ppb(&mut self, addr: u32, bytes: u32, value: u32) -> Ex {
        let full = if bytes == 4 {
            value
        } else {
            let Some(old) = self.state.sys.read_word(addr, true) else {
                return Err(self.bus_fault(addr, Access::Write));
            };
            insert(old, addr, bytes, value)
        };
        if self.state.sys.write_word(addr, full) {
            Ok(())
        } else {
            Err(self.bus_fault(addr, Access::Write))
        }
    }

    /// Fetch one instruction halfword.
    fn fetch(&mut self, addr: u32) -> Ex<u16> {
        // Instructions are always little-endian in ARMv7-M, whatever the data
        // endianness (DDI 0403 A3.3).
        self.check_mpu(addr, 2, Access::Fetch, self.state.privileged())?;
        match self.space.read(u64::from(addr), Width::U16, self.attrs) {
            Ok(v) => Ok(v as u16),
            Err(_) => Err(self.bus_fault(addr, Access::Fetch)),
        }
    }

    // -----------------------------------------------------------------
    // Exceptions
    // -----------------------------------------------------------------

    /// Record a fault's status bits and take it, escalating where the
    /// architecture says to (DDI 0403 B1.5.4).
    fn take_trap(&mut self, trap: Trap, return_address: u32) {
        self.state.sys.cfsr |= trap.status;
        if let Some(far) = trap.far {
            if trap.exc == Exception::MEM_MANAGE {
                self.state.sys.mmfar = far;
            } else {
                self.state.sys.bfar = far;
            }
        }
        let prio = self.state.execution_priority();
        let mut target = trap.exc;
        if target.is_configurable_fault()
            && (!self.state.sys.is_enabled(target) || self.state.sys.priority_of(target) >= prio)
        {
            self.state.sys.hfsr |= fsr::HF_FORCED;
            target = Exception::HARD_FAULT;
        }
        if target == Exception::HARD_FAULT && prio <= -1 {
            // A HardFault at a priority HardFault cannot preempt is a lockup.
            self.state.locked_up = true;
            self.state.r[15] = 0xffff_fffe;
            self.branched = true;
            return;
        }
        self.exception_entry(target, return_address);
    }

    /// Push the exception frame and vector to a handler (DDI 0403 B1.5.6).
    fn exception_entry(&mut self, exc: Exception, return_address: u32) {
        // Taking an exception ends a `WFI` or `WFE` whether or not the
        // instruction stream ever gets back to it.
        self.state.asleep = false;
        let frame_align = self.state.sys.ccr & ccr::STKALIGN != 0;
        let mut sp = self.state.r[13];
        let aligned = frame_align && sp & 4 != 0;
        if aligned {
            sp = sp.wrapping_sub(4);
        }
        sp = sp.wrapping_sub(32);
        // The `SPSEL`-selected stack is the one that gets the frame; the
        // handler then runs on the main stack.
        self.state.r[13] = sp;

        let privileged = self.state.privileged();
        let xpsr_stacked =
            (self.state.xpsr & !(xpsr::EXCEPTION | 0x0600_fc00)) | if aligned { 1 << 9 } else { 0 };
        let frame = [
            self.state.r[0],
            self.state.r[1],
            self.state.r[2],
            self.state.r[3],
            self.state.r[12],
            self.state.r[14],
            return_address,
            xpsr_stacked | (self.state.xpsr & xpsr::EXCEPTION),
        ];
        let mut stack_failed = false;
        for (k, value) in frame.iter().enumerate() {
            let at = sp.wrapping_add((k as u32) * 4);
            if self.write_mem(at, 4, *value, privileged).is_err() {
                stack_failed = true;
            }
        }
        if stack_failed {
            // The frame is lost; the architecture reports `STKERR` and the
            // fault escalates. Reporting it and continuing into the handler
            // is the behaviour a debugger can act on.
            self.state.sys.cfsr |= fsr::BF_STKERR;
            self.state.sys.hfsr |= fsr::HF_FORCED;
        }

        let lr = if self.state.in_handler() {
            exc_return::HANDLER_MSP
        } else if self.state.control & control::SPSEL != 0 {
            exc_return::THREAD_PSP
        } else {
            exc_return::THREAD_MSP
        };
        self.state.r[14] = lr;

        // Handler mode, on the main stack, with `ITSTATE` cleared.
        self.state.xpsr = (self.state.xpsr & !(xpsr::EXCEPTION | xpsr::IT_MASK))
            | u32::from(exc.0) & xpsr::EXCEPTION;
        self.state.sync_stack();
        self.state.sys.set_active(exc, true);
        self.state.sys.set_pending(exc, false);
        self.state.exclusive = None;

        let vector = self.state.sys.vtor.wrapping_add(exc.vector_offset());
        match self.read_vector(vector) {
            Some(entry) => {
                self.state.r[15] = entry & !1;
                self.set_flag(xpsr::T, entry & 1 != 0);
            }
            None => {
                // A vector fetch that faults is `HFSR.VECTTBL`, and it is
                // taken as a HardFault whatever the original exception was.
                self.state.sys.hfsr |= fsr::HF_VECTTBL;
                self.state.locked_up = true;
                self.state.r[15] = 0xffff_fffe;
            }
        }
        self.branched = true;
        self.cycle(12);
    }

    /// Return from an exception (DDI 0403 B1.5.8).
    fn exception_return(&mut self, magic: u32) -> Ex {
        let returning = self.state.current_exception();
        if !self.state.sys.is_active(returning) {
            return Err(Trap::INVPC);
        }
        // Only the three integer values are legal here. The floating-point
        // ones (`0xFFFFFFE1`, `E9`, `ED`) name a frame this core never
        // pushes, because it has no FPU.
        let (to_handler, use_psp) = match magic {
            exc_return::HANDLER_MSP => (true, false),
            exc_return::THREAD_MSP => (false, false),
            exc_return::THREAD_PSP => (false, true),
            _ => return Err(Trap::INVPC),
        };
        let nested = self.state.sys.active_count() > 1;
        if !to_handler && nested && self.state.sys.ccr & ccr::NONBASETHRDENA == 0 {
            return Err(Trap::INVPC);
        }
        if to_handler && !nested {
            return Err(Trap::INVPC);
        }

        self.state.sys.set_active(returning, false);
        // `FAULTMASK` is released by any return except one from NMI.
        if returning != Exception::NMI {
            self.state.faultmask = false;
        }

        // Tail-chaining: if something is already waiting that the processor
        // can now take, go straight to it rather than unstacking a frame the
        // next entry would only push again. `LR` keeps the same `EXC_RETURN`,
        // because it still describes the same interrupted context.
        if let Some((next, prio)) = self.state.sys.highest_pending()
            && prio < self.state.execution_priority()
        {
            self.state.sys.set_pending(next, false);
            self.state.sys.set_active(next, true);
            self.state.xpsr = (self.state.xpsr & !(xpsr::EXCEPTION | xpsr::IT_MASK))
                | u32::from(next.0) & xpsr::EXCEPTION;
            self.state.sync_stack();
            let vector = self.state.sys.vtor.wrapping_add(next.vector_offset());
            match self.read_vector(vector) {
                Some(entry) => {
                    self.state.r[15] = entry & !1;
                    self.set_flag(xpsr::T, entry & 1 != 0);
                }
                None => {
                    self.state.sys.hfsr |= fsr::HF_VECTTBL;
                    self.state.locked_up = true;
                    self.state.r[15] = 0xffff_fffe;
                }
            }
            self.branched = true;
            self.cycle(6);
            return Ok(());
        }

        // Switch to the stack the frame is on before popping it.
        if to_handler {
            self.state.control &= !control::SPSEL;
        } else {
            self.state.xpsr &= !xpsr::EXCEPTION;
            if use_psp {
                self.state.control |= control::SPSEL;
            } else {
                self.state.control &= !control::SPSEL;
            }
        }
        self.state.sync_stack();

        let privileged = self.state.privileged();
        let sp = self.state.r[13];
        let mut frame = [0u32; 8];
        let mut unstack_failed = false;
        for (k, slot) in frame.iter_mut().enumerate() {
            let at = sp.wrapping_add((k as u32) * 4);
            match self.read_mem(at, 4, privileged) {
                Ok(v) => *slot = v,
                Err(_) => unstack_failed = true,
            }
        }
        if unstack_failed {
            self.state.sys.cfsr |= fsr::BF_UNSTKERR;
            self.state.sys.hfsr |= fsr::HF_FORCED;
        }
        self.state.r[0] = frame[0];
        self.state.r[1] = frame[1];
        self.state.r[2] = frame[2];
        self.state.r[3] = frame[3];
        self.state.r[12] = frame[4];
        self.state.r[14] = frame[5];

        let stacked_xpsr = frame[7];
        let mut sp = sp.wrapping_add(32);
        if self.state.sys.ccr & ccr::STKALIGN != 0 && stacked_xpsr & (1 << 9) != 0 {
            sp = sp.wrapping_add(4);
        }
        self.state.r[13] = sp;

        // Restore the flags, `GE`, `ITSTATE`, `T` and the exception number in
        // one go; the reserved bits are dropped.
        self.state.xpsr = stacked_xpsr & (xpsr::WRITABLE | xpsr::EXCEPTION);
        self.state.sync_stack();
        self.state.r[15] = frame[6] & !1;
        self.state.exclusive = None;
        self.branched = true;
        self.cycle(10);
        Ok(())
    }

    // -----------------------------------------------------------------
    // The instruction loop
    // -----------------------------------------------------------------

    /// Fetch, decode, and execute one instruction.
    fn execute_one(&mut self) {
        let pc = self.state.r[15];
        self.insn_addr = pc;
        self.branched = false;
        self.cycle(1);

        if !self.flag(xpsr::T) {
            // `EPSR.T` clear means the core was asked to enter ARM state,
            // which this architecture does not have.
            self.take_trap(Trap::INVSTATE, pc);
            return;
        }

        let first = match self.fetch(pc) {
            Ok(h) => h,
            Err(trap) => {
                self.take_trap(trap, pc);
                return;
            }
        };
        let wide = is_32bit(first);
        let second = if wide {
            match self.fetch(pc.wrapping_add(2)) {
                Ok(h) => h,
                Err(trap) => {
                    self.take_trap(trap, pc);
                    return;
                }
            }
        } else {
            0
        };
        let width = if wide { 4 } else { 2 };
        // The PC reads as the instruction's address plus four in both widths.
        self.state.r[15] = pc.wrapping_add(4);

        let mut insn = decode(first, second);
        let it = self.state.itstate();
        // The sixteen-bit data-processing encodings have no `S` bit: they set
        // the flags outside an `IT` block and leave them alone inside one
        // (DDI 0403 A7.7.76's "MOVS outside IT block; MOV<c> inside IT
        // block", and the same note on every one of its neighbours). This is
        // not decoration — the condition of every later slot in the block is
        // evaluated against the *current* flags, so an `IT` block whose first
        // instruction clobbered them would take the wrong branch of itself.
        if !wide && it != 0 {
            insn = suppress_flags(insn);
        }
        let passed = self.state.current_cond().passes(self.state.xpsr)
            && match insn {
                // A conditional branch carries its own condition; inside an
                // `IT` block the architecture calls the combination
                // UNPREDICTABLE, and requiring both is the reading that never
                // executes something neither condition wanted.
                Insn::Branch { cond: Some(c), .. } => c.passes(self.state.xpsr),
                _ => true,
            };

        let outcome = if passed { self.execute(insn) } else { Ok(()) };

        // `IT` sets ITSTATE; everything else advances it.
        if !matches!(insn, Insn::It { .. }) && it != 0 {
            self.state.it_advance();
        }

        match outcome {
            Ok(()) => {
                if !self.branched {
                    self.state.r[15] = pc.wrapping_add(width);
                }
            }
            Err(trap) => {
                // A synchronous fault stacks the *faulting* instruction's
                // address, so the handler sees what went wrong; `SVC` and
                // `BKPT` stack the next one and do not come through here
                // (DDI 0403 B1.5.6).
                self.state.r[15] = pc;
                self.take_trap(trap, pc);
            }
        }
    }

    /// Execute one decoded instruction.
    #[allow(clippy::too_many_lines)] // One arm per instruction shape.
    fn execute(&mut self, insn: Insn) -> Ex {
        // An instruction the configured part does not have must trap, not
        // execute: that is how a guest probes for the DSP extension, and
        // decoding it anyway would tell the guest a Cortex-M3 is a
        // Cortex-M4 (`ROADMAP.md` §6.1.1).
        if !self.cfg.ext.dsp && needs_dsp(insn) {
            return Err(Trap::UNDEFINED);
        }
        match insn {
            Insn::DataProc {
                op,
                s,
                rd,
                rn,
                operand,
            } => self.data_proc(op, s, rd, rn, operand),
            Insn::ShiftReg { ty, s, rd, rn, rm } => {
                let value = self.reg(rn);
                let amount = self.reg(rm) & 0xff;
                let (result, c) = shift_reg(ty, value, amount, self.flag(xpsr::C));
                self.set_reg(rd, result);
                if s {
                    self.set_nz(result);
                    self.set_flag(xpsr::C, c);
                }
                Ok(())
            }
            Insn::Adr { rd, imm, add } => {
                let base = self.state.r[15] & !3;
                self.set_reg(
                    rd,
                    if add {
                        base.wrapping_add(imm)
                    } else {
                        base.wrapping_sub(imm)
                    },
                );
                Ok(())
            }
            Insn::MovImm16 { top, rd, imm } => {
                let old = self.reg(rd);
                self.set_reg(
                    rd,
                    if top {
                        (old & 0xffff) | (u32::from(imm) << 16)
                    } else {
                        u32::from(imm)
                    },
                );
                Ok(())
            }
            Insn::Branch { offset, .. } => {
                let target = self.state.r[15].wrapping_add(offset as u32);
                self.branch_write_pc(target);
                Ok(())
            }
            Insn::BranchLink { offset } => {
                let target = self.state.r[15].wrapping_add(offset as u32);
                self.state.r[14] = self.insn_addr.wrapping_add(4) | 1;
                self.branch_write_pc(target);
                Ok(())
            }
            Insn::Bx { rm } => {
                let target = self.reg(rm);
                self.bx_write_pc(target)
            }
            Insn::Blx { rm } => {
                let target = self.reg(rm);
                // `BLX Rm` is only encoded sixteen bits wide, so the return
                // address is always the next halfword.
                self.state.r[14] = self.insn_addr.wrapping_add(2) | 1;
                self.bx_write_pc(target)
            }
            Insn::Cbz {
                nonzero,
                rn,
                offset,
            } => {
                if (self.reg(rn) != 0) == nonzero {
                    let target = self.state.r[15].wrapping_add(offset);
                    self.branch_write_pc(target);
                }
                Ok(())
            }
            Insn::TableBranch { rn, rm, half } => {
                let base = if rn == 15 {
                    self.state.r[15]
                } else {
                    self.reg(rn)
                };
                let index = self.reg(rm);
                let privileged = self.state.privileged();
                let offset = if half {
                    let at = base.wrapping_add(index.wrapping_mul(2));
                    self.check_alignment(at, 2, true)?;
                    self.read_mem(at, 2, privileged)?
                } else {
                    self.read_mem(base.wrapping_add(index), 1, privileged)?
                };
                let target = self.state.r[15].wrapping_add(offset.wrapping_mul(2));
                self.branch_write_pc(target);
                Ok(())
            }
            Insn::It { cond, mask } => {
                self.state.set_itstate((cond.0 << 4) | mask);
                Ok(())
            }
            Insn::LoadStore { .. } => self.load_store(insn),
            Insn::LoadLiteral {
                size,
                signed,
                rt,
                imm,
                add,
            } => {
                let base = self.state.r[15] & !3;
                let addr = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                let privileged = self.state.privileged();
                self.check_alignment(addr, size.bytes(), false)?;
                let value = self.read_mem(addr, size.bytes(), privileged)?;
                let value = extend(value, size, signed);
                if rt == 15 {
                    self.bx_write_pc(value)
                } else {
                    self.set_reg(rt, value);
                    Ok(())
                }
            }
            Insn::LoadStoreDual {
                load,
                rt,
                rt2,
                rn,
                imm,
                index,
                add,
                wback,
            } => {
                let base = if rn == 15 {
                    self.state.r[15] & !3
                } else {
                    self.reg(rn)
                };
                let offset = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                let addr = if index { offset } else { base };
                // `LDRD` and `STRD` are always word-aligned, whatever
                // `UNALIGN_TRP` says (DDI 0403 A7.7.49).
                self.check_alignment(addr, 4, true)?;
                let privileged = self.state.privileged();
                if load {
                    let a = self.read_mem(addr, 4, privileged)?;
                    let b = self.read_mem(addr.wrapping_add(4), 4, privileged)?;
                    self.set_reg(rt, a);
                    self.set_reg(rt2, b);
                } else {
                    let a = self.reg(rt);
                    let b = self.reg(rt2);
                    self.write_mem(addr, 4, a, privileged)?;
                    self.write_mem(addr.wrapping_add(4), 4, b, privileged)?;
                }
                if wback && rn != 15 {
                    self.set_reg(rn, offset);
                }
                Ok(())
            }
            Insn::LoadStoreExclusive {
                load,
                size,
                rd,
                rt,
                rn,
                imm,
            } => {
                let addr = self.reg(rn).wrapping_add(imm);
                self.check_alignment(addr, size.bytes(), true)?;
                let privileged = self.state.privileged();
                if load {
                    let value = self.read_mem(addr, size.bytes(), privileged)?;
                    self.state.exclusive = Some(addr);
                    self.set_reg(rt, value);
                } else {
                    // The local monitor: a store succeeds only if this core
                    // tagged exactly this address and nothing has cleared the
                    // tag since. There is no global monitor because there is
                    // no second master in this model.
                    let ok = self.state.exclusive == Some(addr);
                    if ok {
                        let value = self.reg(rt);
                        self.write_mem(addr, size.bytes(), value, privileged)?;
                    }
                    self.state.exclusive = None;
                    self.set_reg(rd, u32::from(!ok));
                }
                Ok(())
            }
            Insn::ClearExclusive => {
                self.state.exclusive = None;
                Ok(())
            }
            Insn::LoadStoreMultiple {
                load,
                rn,
                list,
                wback,
                before,
            } => self.block_transfer(load, rn, list, wback, before),
            Insn::Mul {
                rd,
                rn,
                rm,
                ra,
                sub,
                s,
            } => {
                let product = self.reg(rn).wrapping_mul(self.reg(rm));
                let result = match ra {
                    None => product,
                    Some(ra) if sub => self.reg(ra).wrapping_sub(product),
                    Some(ra) => self.reg(ra).wrapping_add(product),
                };
                self.set_reg(rd, result);
                if s {
                    // `MULS` sets `N` and `Z` and leaves `C` and `V` alone —
                    // unlike ARMv4, where `C` was destroyed
                    // (DDI 0403 A7.7.84).
                    self.set_nz(result);
                }
                Ok(())
            }
            Insn::MulLong {
                signed,
                accumulate,
                rdlo,
                rdhi,
                rn,
                rm,
                umaal,
            } => {
                let a = self.reg(rn);
                let b = self.reg(rm);
                let lo = self.reg(rdlo);
                let hi = self.reg(rdhi);
                let result = if umaal {
                    // `UMAAL` accumulates *both* destination registers as
                    // separate addends, which is what makes it useful for
                    // long multiplication (DDI 0403 A7.7.203).
                    u64::from(a) * u64::from(b) + u64::from(lo) + u64::from(hi)
                } else if signed {
                    let p = i64::from(a as i32) * i64::from(b as i32);
                    let acc = if accumulate {
                        ((u64::from(hi) << 32) | u64::from(lo)) as i64
                    } else {
                        0
                    };
                    p.wrapping_add(acc) as u64
                } else {
                    let p = u64::from(a) * u64::from(b);
                    let acc = if accumulate {
                        (u64::from(hi) << 32) | u64::from(lo)
                    } else {
                        0
                    };
                    p.wrapping_add(acc)
                };
                self.set_reg(rdlo, result as u32);
                self.set_reg(rdhi, (result >> 32) as u32);
                self.cycle(1);
                Ok(())
            }
            Insn::Div { signed, rd, rn, rm } => {
                let a = self.reg(rn);
                let b = self.reg(rm);
                self.cycle(2);
                if b == 0 {
                    if self.state.sys.ccr & ccr::DIV_0_TRP != 0 {
                        return Err(Trap {
                            exc: Exception::USAGE_FAULT,
                            status: fsr::UF_DIVBYZERO,
                            far: None,
                        });
                    }
                    self.set_reg(rd, 0);
                    return Ok(());
                }
                let result = if signed {
                    // The one overflowing case wraps rather than trapping:
                    // `INT_MIN / -1` is defined to give `INT_MIN`
                    // (DDI 0403 A7.7.127).
                    (a as i32).wrapping_div(b as i32) as u32
                } else {
                    a / b
                };
                self.set_reg(rd, result);
                Ok(())
            }
            Insn::HalfMul {
                op,
                rd,
                rn,
                rm,
                ra,
                x,
                y,
            } => {
                self.half_multiply(op, rd, rn, rm, ra, x, y);
                Ok(())
            }
            Insn::DualMul {
                op,
                rd,
                rn,
                rm,
                ra,
                x,
            } => {
                self.dual_multiply(op, rd, rn, rm, ra, x);
                Ok(())
            }
            Insn::Sat {
                unsigned,
                halves,
                rd,
                rn,
                imm,
                shift,
            } => {
                self.saturate(unsigned, halves, rd, rn, imm, shift);
                Ok(())
            }
            Insn::SatQ { op, rd, rn, rm } => {
                self.saturating_arith(op, rd, rn, rm);
                Ok(())
            }
            Insn::Simd {
                mode,
                shape,
                rd,
                rn,
                rm,
            } => {
                let a = self.reg(rn);
                let b = self.reg(rm);
                let (result, ge) = super::dsp::simd(mode, shape, a, b);
                self.set_reg(rd, result);
                if let Some(ge) = ge {
                    self.state.xpsr = (self.state.xpsr & !xpsr::GE) | (u32::from(ge) << 16);
                }
                Ok(())
            }
            Insn::Sel { rd, rn, rm } => {
                let ge = ((self.state.xpsr & xpsr::GE) >> 16) as u8;
                let a = self.reg(rn);
                let b = self.reg(rm);
                let mut result = 0u32;
                for k in 0..4 {
                    let src = if ge & (1 << k) != 0 { a } else { b };
                    result |= src & (0xffu32 << (8 * k));
                }
                self.set_reg(rd, result);
                Ok(())
            }
            Insn::Usad { rd, rn, rm, ra } => {
                let a = self.reg(rn);
                let b = self.reg(rm);
                let mut sum = if ra == 15 { 0 } else { self.reg(ra) };
                for k in 0..4 {
                    let x = (a >> (8 * k)) & 0xff;
                    let y = (b >> (8 * k)) & 0xff;
                    sum = sum.wrapping_add(x.abs_diff(y));
                }
                self.set_reg(rd, sum);
                Ok(())
            }
            Insn::Pkh {
                tb,
                rd,
                rn,
                rm,
                shift,
            } => {
                let a = self.reg(rn);
                let b = shift_imm(shift, self.reg(rm), self.flag(xpsr::C)).0;
                let result = if tb {
                    (a & 0xffff_0000) | (b & 0x0000_ffff)
                } else {
                    (a & 0x0000_ffff) | (b & 0xffff_0000)
                };
                self.set_reg(rd, result);
                Ok(())
            }
            Insn::Extend {
                op,
                rd,
                rn,
                rm,
                rotate,
            } => {
                let rotated = self.reg(rm).rotate_right(u32::from(rotate));
                let value = super::dsp::extend(op, rotated);
                let result = if rn == 15 {
                    value
                } else {
                    super::dsp::extend_accumulate(op, self.reg(rn), value)
                };
                self.set_reg(rd, result);
                Ok(())
            }
            Insn::Misc { op, rd, rm } => {
                let v = self.reg(rm);
                let result = match op {
                    MiscOp::Clz => v.leading_zeros(),
                    MiscOp::Rbit => v.reverse_bits(),
                    MiscOp::Rev => v.swap_bytes(),
                    MiscOp::Rev16 => (v >> 8 & 0x00ff_00ff) | (v << 8 & 0xff00_ff00),
                    MiscOp::Revsh => {
                        let half = ((v >> 8) & 0xff) | ((v & 0xff) << 8);
                        i32::from(half as u16 as i16) as u32
                    }
                };
                self.set_reg(rd, result);
                Ok(())
            }
            Insn::Bitfield {
                op,
                rd,
                rn,
                lsb,
                width,
            } => {
                self.bitfield(op, rd, rn, lsb, width);
                Ok(())
            }
            Insn::Mrs { rd, sysm } => {
                let value = self.read_special(sysm);
                self.set_reg(rd, value);
                Ok(())
            }
            Insn::Msr { rn, sysm, mask } => {
                let value = self.reg(rn);
                self.write_special(sysm, mask, value);
                Ok(())
            }
            Insn::Cps { enable, i, f } => {
                if self.state.privileged() {
                    if i {
                        self.state.primask = !enable;
                    }
                    if f {
                        // `FAULTMASK` may only be *set* from a priority above
                        // −1; clearing is always allowed
                        // (DDI 0403 B1.4.3).
                        if enable {
                            self.state.faultmask = false;
                        } else if self.state.execution_priority() > -1 {
                            self.state.faultmask = true;
                        }
                    }
                }
                Ok(())
            }
            Insn::Barrier { op, .. } => {
                // One access at a time, in program order: there is nothing
                // for a barrier to order. `ISB` still has to be honoured in
                // the sense that any decoded-instruction cache would be
                // flushed — this core has none.
                let _ = op;
                Ok(())
            }
            Insn::Hint { op } => {
                match op {
                    HintOp::Wfi => {
                        self.state.asleep = true;
                    }
                    HintOp::Wfe => {
                        if self.state.event {
                            self.state.event = false;
                        } else {
                            self.state.asleep = true;
                        }
                    }
                    HintOp::Sev => self.state.event = true,
                    _ => {}
                }
                Ok(())
            }
            Insn::Bkpt { imm } => {
                self.state.last_bkpt = imm;
                // With no debug monitor and no halting debug, `BKPT` is a
                // HardFault with `DEBUGEVT` (DDI 0403 C1.4.1).
                self.state.sys.hfsr |= fsr::HF_DEBUGEVT;
                let ret = self.insn_addr.wrapping_add(2);
                self.exception_entry(Exception::HARD_FAULT, ret);
                Ok(())
            }
            Insn::Svc { imm } => {
                self.state.last_svc = imm;
                let ret = self.insn_addr.wrapping_add(2);
                let prio = self.state.execution_priority();
                if self.state.sys.priority_of(Exception::SVCALL) >= prio {
                    // An `SVC` that cannot preempt the current priority is a
                    // HardFault, not a pending SVCall (DDI 0403 B1.5.4).
                    self.state.sys.hfsr |= fsr::HF_FORCED;
                    self.exception_entry(Exception::HARD_FAULT, ret);
                } else {
                    self.exception_entry(Exception::SVCALL, ret);
                }
                Ok(())
            }
            Insn::Coproc { .. } => Err(Trap::NOCP),
            Insn::Udf { .. } | Insn::Undefined => Err(Trap::UNDEFINED),
        }
    }

    // -----------------------------------------------------------------
    // Data processing
    // -----------------------------------------------------------------

    fn data_proc(&mut self, op: DpOp, s: bool, rd: u8, rn: u8, operand: Operand) -> Ex {
        let carry_in = self.flag(xpsr::C);
        let (b, shifter_carry) = match operand {
            Operand::Imm { value, carry } => (value, carry.unwrap_or(carry_in)),
            Operand::Reg { rm, shift } => shift_imm(shift, self.reg(rm), carry_in),
        };
        let a = if op.is_unary() { 0 } else { self.reg(rn) };
        let (result, c, v) = alu(op, a, b, carry_in, shifter_carry, self.flag(xpsr::V));
        if !op.is_test() {
            if rd == 15 {
                // The only data-processing writes to the PC that ARMv7-M
                // defines are the sixteen-bit `ADD`/`MOV` high-register
                // forms, and both are `ALUWritePC`, which is a plain branch
                // that stays in Thumb state (DDI 0403 A2.3.1).
                self.branch_write_pc(result);
            } else {
                self.set_reg(rd, result);
            }
        }
        if s {
            self.set_nz(result);
            self.set_flag(xpsr::C, c);
            self.set_flag(xpsr::V, v);
        }
        Ok(())
    }

    fn bitfield(&mut self, op: BitfieldOp, rd: u8, rn: u8, lsb: u8, width: u8) {
        let lsb = u32::from(lsb) & 31;
        let width = u32::from(width).clamp(1, 32);
        let mask = if width >= 32 {
            u32::MAX
        } else {
            (1u32 << width) - 1
        };
        match op {
            BitfieldOp::Ubfx => {
                let v = (self.reg(rn) >> lsb) & mask;
                self.set_reg(rd, v);
            }
            BitfieldOp::Sbfx => {
                let v = (self.reg(rn) >> lsb) & mask;
                let shift = 32 - width;
                self.set_reg(rd, ((v << shift) as i32 >> shift) as u32);
            }
            BitfieldOp::Bfi => {
                let field = (self.reg(rn) & mask) << lsb;
                let hole = mask << lsb;
                let old = self.reg(rd);
                self.set_reg(rd, (old & !hole) | field);
            }
            BitfieldOp::Bfc => {
                let hole = mask << lsb;
                let old = self.reg(rd);
                self.set_reg(rd, old & !hole);
            }
        }
    }

    // -----------------------------------------------------------------
    // Memory instructions
    // -----------------------------------------------------------------

    fn load_store(&mut self, insn: Insn) -> Ex {
        let Insn::LoadStore {
            load,
            size,
            signed,
            rt,
            rn,
            offset,
            index,
            add,
            wback,
            unpriv,
        } = insn
        else {
            return Err(Trap::UNDEFINED);
        };
        let base = self.reg(rn);
        let delta = match offset {
            MemOffset::Imm(v) => v,
            MemOffset::Reg { rm, lsl } => self.reg(rm) << u32::from(lsl),
        };
        let offset_addr = if add {
            base.wrapping_add(delta)
        } else {
            base.wrapping_sub(delta)
        };
        let addr = if index { offset_addr } else { base };
        // `LDRT`/`STRT` are unprivileged whatever mode the core is in.
        let privileged = self.state.privileged() && !unpriv;
        self.check_alignment(addr, size.bytes(), false)?;
        if load {
            let raw = self.read_mem(addr, size.bytes(), privileged)?;
            let value = extend(raw, size, signed);
            if wback {
                self.set_reg(rn, offset_addr);
            }
            if rt == 15 {
                return self.bx_write_pc(value);
            }
            self.set_reg(rt, value);
        } else {
            let value = self.reg(rt);
            self.write_mem(addr, size.bytes(), value, privileged)?;
            if wback {
                self.set_reg(rn, offset_addr);
            }
        }
        Ok(())
    }

    fn block_transfer(&mut self, load: bool, rn: u8, list: u16, wback: bool, before: bool) -> Ex {
        let count = list.count_ones();
        if count == 0 {
            // An empty register list is UNPREDICTABLE and every encoding that
            // could produce one requires at least one register; treating it
            // as undefined keeps a corrupt instruction stream from silently
            // moving the base.
            return Err(Trap::UNDEFINED);
        }
        let base = self.reg(rn);
        let span = count * 4;
        let start = if before {
            base.wrapping_sub(span)
        } else {
            base
        };
        let end_base = if before {
            base.wrapping_sub(span)
        } else {
            base.wrapping_add(span)
        };
        // A block transfer is always word-aligned; the architecture faults on
        // an unaligned base rather than splitting the access.
        self.check_alignment(start, 4, true)?;
        let privileged = self.state.privileged();

        if load && wback {
            // Writeback before the loads, so a list containing the base ends
            // up holding the loaded value. The encodings already refuse
            // writeback when the base is in the list, so this only matters
            // for the base's own final value.
            self.set_reg(rn, end_base);
        }
        let mut addr = start;
        let mut pc_value = None;
        for index in 0u8..16 {
            if list & (1 << index) == 0 {
                continue;
            }
            if load {
                let value = self.read_mem(addr, 4, privileged)?;
                if index == 15 {
                    pc_value = Some(value);
                } else {
                    self.set_reg(index, value);
                }
            } else {
                let value = self.reg(index);
                self.write_mem(addr, 4, value, privileged)?;
            }
            addr = addr.wrapping_add(4);
        }
        if !load && wback {
            self.set_reg(rn, end_base);
        }
        if let Some(value) = pc_value {
            // `POP {..., pc}` interworks, which is how an exception return
            // from a handler that pushed `LR` gets home.
            return self.bx_write_pc(value);
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // DSP
    // -----------------------------------------------------------------

    /// Signed saturating add, setting the sticky `Q` flag on overflow.
    fn q_add(&mut self, a: i32, b: i32) -> i32 {
        match a.checked_add(b) {
            Some(v) => v,
            None => {
                self.set_flag(xpsr::Q, true);
                if a < 0 { i32::MIN } else { i32::MAX }
            }
        }
    }

    /// Signed saturating subtract, setting the sticky `Q` flag on overflow.
    fn q_sub(&mut self, a: i32, b: i32) -> i32 {
        match a.checked_sub(b) {
            Some(v) => v,
            None => {
                self.set_flag(xpsr::Q, true);
                if a < 0 { i32::MIN } else { i32::MAX }
            }
        }
    }

    fn saturating_arith(&mut self, op: SatQOp, rd: u8, rn: u8, rm: u8) {
        // The assembler syntax is `QADD Rd, Rm, Rn`, and it is `Rn` — the
        // first halfword's register field — that the `QD` forms double
        // (DDI 0403 A7.7.129).
        let n = self.reg(rn) as i32;
        let m = self.reg(rm) as i32;
        let result = match op {
            SatQOp::Qadd => self.q_add(m, n),
            SatQOp::Qsub => self.q_sub(m, n),
            SatQOp::Qdadd => {
                let doubled = self.q_add(n, n);
                self.q_add(m, doubled)
            }
            SatQOp::Qdsub => {
                let doubled = self.q_add(n, n);
                self.q_sub(m, doubled)
            }
        };
        self.set_reg(rd, result as u32);
    }

    fn saturate(&mut self, unsigned: bool, halves: bool, rd: u8, rn: u8, imm: u8, shift: Shift) {
        let value = self.reg(rn);
        if halves {
            let bits = u32::from(imm);
            let mut result = 0u32;
            let mut saturated = false;
            for k in 0..2 {
                let half = i32::from(((value >> (16 * k)) as u16) as i16);
                let (v, sat) = if unsigned {
                    sat_unsigned(half, bits)
                } else {
                    sat_signed(half, bits)
                };
                saturated |= sat;
                result |= ((v as u32) & 0xffff) << (16 * k);
            }
            self.set_reg(rd, result);
            if saturated {
                self.set_flag(xpsr::Q, true);
            }
            return;
        }
        let shifted = shift_imm(shift, value, self.flag(xpsr::C)).0 as i32;
        let (v, sat) = if unsigned {
            sat_unsigned(shifted, u32::from(imm))
        } else {
            sat_signed(shifted, u32::from(imm))
        };
        self.set_reg(rd, v as u32);
        if sat {
            self.set_flag(xpsr::Q, true);
        }
    }

    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn half_multiply(&mut self, op: HalfMulOp, rd: u8, rn: u8, rm: u8, ra: u8, x: bool, y: bool) {
        let n = self.reg(rn);
        let m = self.reg(rm);
        match op {
            HalfMulOp::Smul => {
                let p = half_of(n, x).wrapping_mul(half_of(m, y));
                self.set_reg(rd, p as u32);
            }
            HalfMulOp::Smla => {
                let p = half_of(n, x).wrapping_mul(half_of(m, y));
                let acc = self.reg(ra) as i32;
                if p.checked_add(acc).is_none() {
                    self.set_flag(xpsr::Q, true);
                }
                self.set_reg(rd, p.wrapping_add(acc) as u32);
            }
            HalfMulOp::Smulw => {
                let wide = i64::from(n as i32) * i64::from(half_of(m, y));
                self.set_reg(rd, (wide >> 16) as u32);
            }
            HalfMulOp::Smlaw => {
                let wide = i64::from(n as i32) * i64::from(half_of(m, y));
                let p = (wide >> 16) as i32;
                let acc = self.reg(ra) as i32;
                if p.checked_add(acc).is_none() {
                    self.set_flag(xpsr::Q, true);
                }
                self.set_reg(rd, p.wrapping_add(acc) as u32);
            }
            HalfMulOp::Smlal => {
                // `rd` is `RdHi` and `ra` is `RdLo` for this encoding.
                let p = i64::from(half_of(n, x)) * i64::from(half_of(m, y));
                let acc = ((u64::from(self.reg(rd)) << 32) | u64::from(self.reg(ra))) as i64;
                let result = acc.wrapping_add(p) as u64;
                self.set_reg(ra, result as u32);
                self.set_reg(rd, (result >> 32) as u32);
            }
        }
        self.cycle(1);
    }

    fn dual_multiply(&mut self, op: DualMulOp, rd: u8, rn: u8, rm: u8, ra: u8, x: bool) {
        let n = self.reg(rn);
        let m = self.reg(rm);
        // The `X` bit swaps `Rm`'s halves before the products are formed.
        let m = if x && !op.bit_is_round() {
            m.rotate_right(16)
        } else {
            m
        };
        let p0 = i32::from(n as u16 as i16) as i64 * i32::from(m as u16 as i16) as i64;
        let p1 =
            i32::from((n >> 16) as u16 as i16) as i64 * i32::from((m >> 16) as u16 as i16) as i64;
        match op {
            DualMulOp::Smuad | DualMulOp::Smlad => {
                let sum = p0
                    + p1
                    + if op == DualMulOp::Smlad {
                        i64::from(self.reg(ra) as i32)
                    } else {
                        0
                    };
                if sum != i64::from(sum as i32) {
                    self.set_flag(xpsr::Q, true);
                }
                self.set_reg(rd, sum as u32);
            }
            DualMulOp::Smusd | DualMulOp::Smlsd => {
                let diff = p0 - p1
                    + if op == DualMulOp::Smlsd {
                        i64::from(self.reg(ra) as i32)
                    } else {
                        0
                    };
                if diff != i64::from(diff as i32) {
                    self.set_flag(xpsr::Q, true);
                }
                self.set_reg(rd, diff as u32);
            }
            DualMulOp::Smmul | DualMulOp::Smmla | DualMulOp::Smmls => {
                let product = i64::from(n as i32) * i64::from(m as i32);
                let acc = match op {
                    DualMulOp::Smmla => i64::from(self.reg(ra) as i32) << 32,
                    DualMulOp::Smmls => i64::from(self.reg(ra) as i32) << 32,
                    _ => 0,
                };
                let total = if op == DualMulOp::Smmls {
                    acc - product
                } else {
                    acc + product
                };
                // The `R` bit rounds rather than truncating, by adding a half
                // before the shift (DDI 0403 A7.7.163).
                let total = if x {
                    total.wrapping_add(0x8000_0000)
                } else {
                    total
                };
                self.set_reg(rd, (total >> 32) as u32);
            }
            DualMulOp::Smlald | DualMulOp::Smlsld => {
                let acc = ((u64::from(self.reg(rd)) << 32) | u64::from(self.reg(ra))) as i64;
                let combined = if op == DualMulOp::Smlald {
                    p0 + p1
                } else {
                    p0 - p1
                };
                let result = acc.wrapping_add(combined) as u64;
                self.set_reg(ra, result as u32);
                self.set_reg(rd, (result >> 32) as u32);
            }
        }
        self.cycle(1);
    }

    // -----------------------------------------------------------------
    // Special registers
    // -----------------------------------------------------------------

    /// `MRS` (DDI 0403 B5.2.2).
    ///
    /// `EPSR` reads as zero through `MRS` — the `T` bit and `ITSTATE` are
    /// deliberately not visible to software this way.
    fn read_special(&self, sysm: u8) -> u32 {
        let privileged = self.state.privileged();
        match sysm {
            0 => self.state.xpsr & (xpsr::FLAGS | xpsr::GE),
            1 => self.state.xpsr & (xpsr::FLAGS | xpsr::GE | xpsr::EXCEPTION),
            2 => self.state.xpsr & (xpsr::FLAGS | xpsr::GE),
            3 => self.state.xpsr & (xpsr::FLAGS | xpsr::GE | xpsr::EXCEPTION),
            5 => self.state.xpsr & xpsr::EXCEPTION,
            6 => 0,
            7 => self.state.xpsr & xpsr::EXCEPTION,
            8 => self.state.msp(),
            9 => self.state.psp(),
            16 => u32::from(self.state.primask && privileged),
            17 | 18 => {
                if privileged {
                    u32::from(self.state.basepri)
                } else {
                    0
                }
            }
            19 => u32::from(self.state.faultmask && privileged),
            20 => self.state.control,
            _ => 0,
        }
    }

    /// `MSR` (DDI 0403 B5.2.3).
    ///
    /// Unprivileged code may write the `APSR` forms and nothing else; the
    /// architecture makes every other write a no-op rather than a fault.
    fn write_special(&mut self, sysm: u8, mask: u8, value: u32) {
        let privileged = self.state.privileged();
        match sysm {
            0..=3 => {
                if mask & 0b10 != 0 {
                    self.state.xpsr = (self.state.xpsr & !xpsr::FLAGS) | (value & xpsr::FLAGS);
                }
                if mask & 0b01 != 0 {
                    self.state.xpsr = (self.state.xpsr & !xpsr::GE) | (value & xpsr::GE);
                }
            }
            8 if privileged => self.state.set_msp(value & !3),
            9 if privileged => self.state.set_psp(value & !3),
            16 if privileged => self.state.primask = value & 1 != 0,
            17 if privileged => self.state.basepri = value as u8,
            18 if privileged => {
                // `BASEPRI_MAX` only ever raises the priority ceiling: a
                // write that would lower it is ignored (DDI 0403 B5.2.3).
                let new = value as u8;
                if new != 0 && (self.state.basepri == 0 || new < self.state.basepri) {
                    self.state.basepri = new;
                }
            }
            19 if privileged => {
                if value & 1 == 0 {
                    self.state.faultmask = false;
                } else if self.state.execution_priority() > -1 {
                    self.state.faultmask = true;
                }
            }
            20 if privileged => {
                let mut control = self.state.control & !control::NPRIV;
                control |= value & control::NPRIV;
                if !self.state.in_handler() {
                    control = (control & !control::SPSEL) | (value & control::SPSEL);
                }
                self.state.control = control;
                self.state.sync_stack();
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------

/// The `Width` a byte count names.
const fn width_of(bytes: u32) -> Width {
    match bytes {
        1 => Width::U8,
        2 => Width::U16,
        _ => Width::U32,
    }
}

/// Extract the `bytes`-wide field at `addr`'s offset within a word.
const fn extract(word: u32, addr: u32, bytes: u32) -> u32 {
    let shift = (addr & 3) * 8;
    let mask = if bytes >= 4 {
        u32::MAX
    } else {
        (1u32 << (bytes * 8)) - 1
    };
    (word >> shift) & mask
}

/// Put a `bytes`-wide field back at `addr`'s offset within a word.
const fn insert(word: u32, addr: u32, bytes: u32, value: u32) -> u32 {
    let shift = (addr & 3) * 8;
    let mask = if bytes >= 4 {
        u32::MAX
    } else {
        ((1u32 << (bytes * 8)) - 1) << shift
    };
    (word & !mask) | ((value << shift) & mask)
}

/// Sign- or zero-extend a loaded value.
const fn extend(raw: u32, size: Size, signed: bool) -> u32 {
    match (size, signed) {
        (Size::Byte, true) => ((raw as u8) as i8) as i32 as u32,
        (Size::Half, true) => ((raw as u16) as i16) as i32 as u32,
        _ => raw,
    }
}

/// One half of a register, sign-extended.
const fn half_of(value: u32, top: bool) -> i32 {
    if top {
        ((value >> 16) as u16 as i16) as i32
    } else {
        (value as u16 as i16) as i32
    }
}

/// Saturate to `bits` signed bits, reporting whether it clamped.
fn sat_signed(value: i32, bits: u32) -> (i32, bool) {
    if bits >= 32 {
        return (value, false);
    }
    let max = (1i32 << (bits - 1)) - 1;
    let min = -(1i32 << (bits - 1));
    if value > max {
        (max, true)
    } else if value < min {
        (min, true)
    } else {
        (value, false)
    }
}

/// Saturate to `bits` unsigned bits, reporting whether it clamped.
fn sat_unsigned(value: i32, bits: u32) -> (i32, bool) {
    let max = if bits >= 32 {
        u32::MAX as i32
    } else {
        ((1u32 << bits) - 1) as i32
    };
    if value > max {
        (max, true)
    } else if value < 0 {
        (0, true)
    } else {
        (value, false)
    }
}

/// The barrel shifter with a constant amount (DDI 0403 A7.4.2's `Shift_C`).
pub(super) fn shift_imm(shift: Shift, value: u32, carry_in: bool) -> (u32, bool) {
    match shift.ty {
        ShiftType::Rrx => ((u32::from(carry_in) << 31) | (value >> 1), value & 1 != 0),
        ShiftType::Lsl if shift.amount == 0 => (value, carry_in),
        ty => shift_reg(ty, value, u32::from(shift.amount), carry_in),
    }
}

/// The barrel shifter with a variable amount, which is the same operation
/// with the "more than thirty-one" cases spelled out.
///
/// A shift of thirty-two is not a shift of thirty-one: `LSL #32` leaves bit 0
/// in the carry and `LSL #33` leaves nothing, and Rust's shift operators
/// would panic or wrap rather than say so.
pub(super) fn shift_reg(ty: ShiftType, value: u32, amount: u32, carry_in: bool) -> (u32, bool) {
    if amount == 0 {
        return (value, carry_in);
    }
    match ty {
        ShiftType::Lsl => match amount {
            1..=31 => (value << amount, value & (1 << (32 - amount)) != 0),
            32 => (0, value & 1 != 0),
            _ => (0, false),
        },
        ShiftType::Lsr => match amount {
            1..=31 => (value >> amount, value & (1 << (amount - 1)) != 0),
            32 => (0, value & 0x8000_0000 != 0),
            _ => (0, false),
        },
        ShiftType::Asr => {
            if amount >= 32 {
                let sign = value & 0x8000_0000 != 0;
                (if sign { u32::MAX } else { 0 }, sign)
            } else {
                (
                    ((value as i32) >> amount) as u32,
                    value & (1 << (amount - 1)) != 0,
                )
            }
        }
        ShiftType::Ror => {
            let low = amount & 31;
            if low == 0 {
                (value, value & 0x8000_0000 != 0)
            } else {
                (value.rotate_right(low), value & (1 << (low - 1)) != 0)
            }
        }
        // `RRX` has no variable form; `DecodeRegShift` never produces it.
        ShiftType::Rrx => ((u32::from(carry_in) << 31) | (value >> 1), value & 1 != 0),
    }
}

/// One data-processing operation and the flags it would set.
///
/// Returns `(result, c, v)`. The caller decides whether to write the result
/// and whether to commit the flags, which is what separates `SUB` from `CMP`
/// and `S` from no `S`.
pub(super) fn alu(
    op: DpOp,
    a: u32,
    b: u32,
    carry_in: bool,
    shifter_carry: bool,
    v_in: bool,
) -> (u32, bool, bool) {
    // Every arithmetic operation is `a + b + carry` with one operand possibly
    // inverted, which is how the hardware does it and why the carry flag
    // means "not borrow" on a subtract.
    fn add(a: u32, b: u32, carry: bool) -> (u32, bool, bool) {
        let wide = u64::from(a) + u64::from(b) + u64::from(carry);
        let result = wide as u32;
        let c = wide > u64::from(u32::MAX);
        let v = (a ^ result) & (b ^ result) & 0x8000_0000 != 0;
        (result, c, v)
    }
    match op {
        DpOp::And | DpOp::Tst => (a & b, shifter_carry, v_in),
        DpOp::Bic => (a & !b, shifter_carry, v_in),
        DpOp::Orr => (a | b, shifter_carry, v_in),
        DpOp::Orn => (a | !b, shifter_carry, v_in),
        DpOp::Eor | DpOp::Teq => (a ^ b, shifter_carry, v_in),
        DpOp::Add | DpOp::Cmn => add(a, b, false),
        DpOp::Adc => add(a, b, carry_in),
        DpOp::Sbc => add(a, !b, carry_in),
        DpOp::Sub | DpOp::Cmp => add(a, !b, true),
        DpOp::Rsb => add(!a, b, true),
        DpOp::Mov => (b, shifter_carry, v_in),
        DpOp::Mvn => (!b, shifter_carry, v_in),
    }
}

/// Whether an instruction belongs to the DSP (E) extension.
///
/// The `16`-lane extends (`SXTB16`, `UXTAB16` and friends) are DSP; the plain
/// `SXTB`/`UXTH` forms are in base ARMv7-M and a Cortex-M3 has them
/// (DDI 0403 A4.4.1's "packing and unpacking" table).
pub(super) const fn needs_dsp(insn: Insn) -> bool {
    match insn {
        Insn::Simd { .. }
        | Insn::SatQ { .. }
        | Insn::HalfMul { .. }
        | Insn::DualMul { .. }
        | Insn::Sel { .. }
        | Insn::Usad { .. }
        | Insn::Pkh { .. } => true,
        // `SSAT16` and `USAT16` are DSP; the plain 32-bit saturates are not.
        Insn::Sat { halves, .. } => halves,
        Insn::Extend { op, rn, .. } => {
            matches!(op, ExtendOp::Sxtb16 | ExtendOp::Uxtb16) || rn != 15
        }
        _ => false,
    }
}

/// The sixteen-bit encodings whose `S` is implied by not being in an `IT`
/// block, rewritten as if `S` were clear.
///
/// `CMP`, `CMN`, `TST` and `TEQ` are left alone: they have no destination, so
/// setting the flags is the whole instruction.
const fn suppress_flags(insn: Insn) -> Insn {
    match insn {
        Insn::DataProc {
            op,
            s: _,
            rd,
            rn,
            operand,
        } if !op.is_test() => Insn::DataProc {
            op,
            s: false,
            rd,
            rn,
            operand,
        },
        Insn::ShiftReg { ty, rd, rn, rm, .. } => Insn::ShiftReg {
            ty,
            s: false,
            rd,
            rn,
            rm,
        },
        Insn::Mul {
            rd,
            rn,
            rm,
            ra,
            sub,
            ..
        } => Insn::Mul {
            rd,
            rn,
            rm,
            ra,
            sub,
            s: false,
        },
        other => other,
    }
}
