//! The ARMv5TE interpreter.
//!
//! # The timing model, and why it is not a cycle table
//!
//! A 6502 cycle *is* a bus access, so that core needs no cycle counter at all.
//! An ARM9 is not like that: it has a five-stage pipeline, a prefetch unit,
//! Harvard caches and a write buffer, and the cost of an instruction depends on
//! what the two caches were holding — none of which is the core's to know,
//! because the caches and the TCMs live behind CP15 and belong to the SoC
//! (see [`super::cp`]).
//!
//! Inventing a precision we cannot justify would be worse than admitting the
//! model, so the model is stated plainly. It is ARM's own **S/N/I accounting**
//! (the instruction-cycle summary in the ARM7TDMI and ARM9 datasheets),
//! evaluated against **zero-wait-state memory**:
//!
//! | Contribution | Cycles |
//! | --- | --- |
//! | Instruction fetch | 1 (one bus access) |
//! | Each data access | 1 (one bus access) |
//! | Register-controlled shift | +1 internal |
//! | Multiply | +*m* internal, *m* from the early-termination rule |
//! | Any load (`LDR`, `LDM`, `LDRH`, `SWP`, …) | +1 internal, the address-to-data cycle |
//! | Any write to `R15`, including exception entry | +2 internal, the pipeline refill |
//!
//! Each *access* is charged where it happens, so a device on the bus sees the
//! traffic in the order hardware would. The arithmetic then reproduces ARM's
//! published totals exactly: `LDR` is 1 + 1 + 1 = 3 against ARM's 1S + 1N +
//! 1I; `STR` is 2 against 2N; `LDM` of *n* registers is *n* + 2 against
//! *n*S + 1N + 1I; `STM` is *n* + 1 against (*n* − 1)S + 2N; `B` is 3 against
//! 2S + 1N. The one bookkeeping difference is *where* a branch's refill lands:
//! ARM charges the target's fetch to the branch, we charge it to the target.
//! Total elapsed time over any run is the same.
//!
//! What is **not** modelled: the prefetch unit's speculative fetches (we fetch
//! exactly the instruction we execute, once), cache hits and misses, the write
//! buffer, and wait states. A SoC that wants those adds them in its memory
//! system, where they belong.
//!
//! # `R15` reads as the instruction plus eight
//!
//! Kept literally rather than compensated for: after the fetch, `r[15]` holds
//! the instruction's address plus eight in ARM state and plus four in Thumb,
//! so every register read is a plain array index. If the instruction did not
//! write `R15`, the last thing [`Exec::step`] does is set it to the next
//! instruction's address. A register-controlled shift makes `R15` read plus
//! *twelve* instead, which is the extra pipeline cycle showing through; the
//! architecture calls that case UNPREDICTABLE, and this is the ARM7TDMI
//! behaviour every assembler that relies on it expects.
//!
//! # Sources
//!
//! ARM ARM (DDI 0100): A2.5 (registers and modes), A2.6 (exceptions, their
//! priority, and the `R14` values each one saves), A2.8 (endianness), A3
//! (the encoding tables), A4.1 (per-instruction operation pseudocode), A5
//! (addressing modes), A6/A7 (Thumb), A10 (the DSP extensions). Cycle counts
//! from the instruction-cycle-timing summaries in ARM's ARM7TDMI and
//! ARM9 datasheets. No emulator source of any licence was consulted.

use alloc::sync::Arc;

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::{Endian, Width};

use super::cp::{AccessKind, Coprocessor, CpEffect, CpFault, CpOp, CpTransfer, Fault, Mmu};
use super::isa::{
    Decoded, DpOp, ExtraOp, Half, HalfMulOp, Index, Insn, Offset, Operand, SatOp, Shift, ShiftType,
};
use super::thumb::{AluOp, HiOp, ImmOp, MemRegOp, MemSize, SmallOperand, Thumb};
use super::{Config, Mode, Regs, psr};

/// The seven ARM exceptions, in the order their vectors appear.
///
/// Ordered by *priority*, highest first, because that is the one property the
/// core has to get right and the vector offsets fall out of the same list
/// (ARM ARM A2.6, tables "Exception priorities" and "Exception vectors").
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Exception {
    /// Priority 1. Enters Supervisor mode with both interrupts masked.
    Reset,
    /// Priority 2. `R14_abt` is the aborting instruction's address plus eight.
    DataAbort,
    /// Priority 3. Enters FIQ mode and masks FIQ as well as IRQ.
    Fiq,
    /// Priority 4.
    Irq,
    /// Priority 5. `R14_abt` is the aborting instruction's address plus four.
    PrefetchAbort,
    /// Priority 6, and mutually exclusive with [`Exception::Swi`] — an
    /// instruction cannot be both.
    Undefined,
    /// Priority 6.
    Swi,
}

impl Exception {
    /// The vector's offset from the vector base.
    #[must_use]
    pub const fn vector(self) -> u32 {
        match self {
            Exception::Reset => 0x00,
            Exception::Undefined => 0x04,
            Exception::Swi => 0x08,
            Exception::PrefetchAbort => 0x0c,
            Exception::DataAbort => 0x10,
            Exception::Irq => 0x18,
            Exception::Fiq => 0x1c,
        }
    }

    /// The mode the core enters.
    #[must_use]
    pub const fn mode(self) -> Mode {
        match self {
            Exception::Reset | Exception::Swi => Mode::SUPERVISOR,
            Exception::Undefined => Mode::UNDEFINED,
            Exception::PrefetchAbort | Exception::DataAbort => Mode::ABORT,
            Exception::Irq => Mode::IRQ,
            Exception::Fiq => Mode::FIQ,
        }
    }

    /// Whether entry masks FIQ as well as IRQ.
    ///
    /// Only reset and FIQ itself do. An IRQ handler that has not saved state
    /// yet must still be interruptible by an FIQ — that is the whole point of
    /// having two interrupt inputs (ARM ARM A2.6.8).
    #[must_use]
    pub const fn masks_fiq(self) -> bool {
        matches!(self, Exception::Reset | Exception::Fiq)
    }

    /// A short name, for tracing.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Exception::Reset => "reset",
            Exception::DataAbort => "data abort",
            Exception::Fiq => "fiq",
            Exception::Irq => "irq",
            Exception::PrefetchAbort => "prefetch abort",
            Exception::Undefined => "undefined instruction",
            Exception::Swi => "swi",
        }
    }
}

/// An access that failed, on its way back up to [`Exec::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Abort {
    kind: AccessKind,
    va: u32,
    fault: Fault,
}

/// `Ok` or "an access aborted; abandon this instruction".
type Ex<T = ()> = core::result::Result<T, Abort>;

/// The architectural state one core owns.
///
/// Separate from [`Arm`](super::Arm) because the interrupt *lines* live
/// outside the lock: a device asserting IRQ from inside an MMIO write the CPU
/// itself issued would otherwise re-enter the CPU's own critical section and
/// deadlock (the re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file, including the banked registers and the SPSRs.
    pub regs: Regs,
    /// Bus cycles executed since power-on.
    pub cycles: u64,
    /// Waiting for an interrupt, as `MCR p15, 0, Rd, c7, c0, 4` asks.
    pub halted: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u32,
    /// The comment field of the last `SWI` executed.
    pub last_swi: u32,
    /// The comment field of the last `BKPT` executed.
    pub last_bkpt: u16,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new() -> State {
        State {
            regs: Regs::new(),
            cycles: 0,
            halted: false,
            reset_pending: true,
            faults: 0,
            last_fault: 0,
            last_swi: 0,
            last_bkpt: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    mmu: &'a dyn Mmu,
    coprocessors: &'a [Option<Arc<dyn Coprocessor>>; 16],
    cfg: &'a Config,
    attrs: MemAttrs,
    /// Address of the instruction being executed.
    insn_addr: u32,
    /// Set when the instruction wrote `R15`, so the fall-through advance is
    /// skipped.
    branched: bool,
    /// Cycles this step has charged.
    used: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        mmu: &'a dyn Mmu,
        coprocessors: &'a [Option<Arc<dyn Coprocessor>>; 16],
        cfg: &'a Config,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        Exec {
            state,
            space,
            mmu,
            coprocessors,
            cfg,
            attrs,
            insn_addr: 0,
            branched: false,
            used: 0,
        }
    }

    /// Run one reset sequence, one exception entry, or one instruction.
    ///
    /// Returns the cycles charged. The interrupt inputs are sampled once, at
    /// the top: ARM samples them at instruction boundaries, and an instruction
    /// is never interrupted part-way (ARM ARM A2.6.8).
    pub(super) fn step(&mut self, irq: bool, fiq: bool) -> u64 {
        if self.state.reset_pending {
            self.state.reset_pending = false;
            self.state.halted = false;
            self.take_exception(Exception::Reset, 0);
            return self.used;
        }

        // An interrupt wakes a halted core whether or not it is masked — the
        // wake-up and the exception are separate things, and `WFI` returns on
        // the line, not on the handler (ARM926EJ-S TRM, "Wait for interrupt").
        if self.state.halted {
            if !(irq || fiq) {
                self.cycle(1);
                return self.used;
            }
            self.state.halted = false;
        }

        if fiq && !self.flag(psr::F) {
            let lr = self.state.regs.r[15].wrapping_add(4);
            self.take_exception(Exception::Fiq, lr);
            return self.used;
        }
        if irq && !self.flag(psr::I) {
            let lr = self.state.regs.r[15].wrapping_add(4);
            self.take_exception(Exception::Irq, lr);
            return self.used;
        }

        if self.flag(psr::T) {
            self.step_thumb();
        } else {
            self.step_arm();
        }
        self.used
    }

    // -----------------------------------------------------------------
    // Cycles
    // -----------------------------------------------------------------

    /// Charge `n` cycles.
    fn cycle(&mut self, n: u64) {
        self.used += n;
        self.state.cycles = self.state.cycles.wrapping_add(n);
    }

    // -----------------------------------------------------------------
    // Flags and registers
    // -----------------------------------------------------------------

    fn flag(&self, mask: u32) -> bool {
        self.state.regs.cpsr & mask != 0
    }

    fn set_flag(&mut self, mask: u32, on: bool) {
        if on {
            self.state.regs.cpsr |= mask;
        } else {
            self.state.regs.cpsr &= !mask;
        }
    }

    /// Set `N` and `Z` from a 32-bit result.
    fn set_nz(&mut self, value: u32) {
        self.set_flag(psr::N, value & 0x8000_0000 != 0);
        self.set_flag(psr::Z, value == 0);
    }

    #[inline]
    fn reg(&self, index: u8) -> u32 {
        self.state.regs.r[(index & 0xf) as usize]
    }

    /// Read a register, adding `extra` if it is `R15`.
    ///
    /// `extra` is four exactly when a register-controlled shift is in play.
    #[inline]
    fn reg_plus(&self, index: u8, extra: u32) -> u32 {
        let v = self.reg(index);
        if index & 0xf == 15 {
            v.wrapping_add(extra)
        } else {
            v
        }
    }

    /// Write a register, treating `R15` as a branch.
    ///
    /// Writing `R15` from an instruction that is not a branch is
    /// UNPREDICTABLE, so the question is which legal answer to give. Measured
    /// against the corpus, an ARM7TDMI flushes the prefetch queue for every
    /// such write — `MUL`, `SWP`, a base writeback — with exactly one
    /// exception, [`Exec::write_pc_without_flush`], which `MRS` uses.
    #[inline]
    fn set_reg(&mut self, index: u8, value: u32) {
        if index & 0xf == 15 {
            self.branch_to(value);
            return;
        }
        self.state.regs.r[(index & 0xf) as usize] = value;
    }

    /// The one write to `R15` that does *not* flush the prefetch queue.
    ///
    /// `MRS Rd, <psr>` with `Rd == R15` is UNPREDICTABLE, and an ARM7TDMI
    /// answers it differently from every other write to `R15`: the value lands
    /// in the *pipelined* `R15`, nothing is flushed, and the ordinary
    /// end-of-instruction advance still happens, so execution continues at
    /// `value + 4` in pipelined terms. Measured; there is no rule in the
    /// manual to derive it from.
    fn write_pc_without_flush(&mut self, value: u32) {
        let ahead = if self.flag(psr::T) { 4 } else { 8 };
        self.state.regs.r[15] = value.wrapping_add(4).wrapping_sub(ahead);
        self.branched = true;
    }

    /// The value a store of `R15` writes.
    ///
    /// Implementation-defined: the architecture permits the instruction's
    /// address plus eight or plus twelve, and parts differ. ARM926EJ-S stores
    /// plus eight, ARM7TDMI plus twelve, so it is a construction property
    /// rather than a constant (ARM ARM A4.1.99's note).
    fn store_value(&self, index: u8) -> u32 {
        if index & 0xf == 15 {
            // The offset is "pipeline depth in instructions × instruction
            // width", so it halves in Thumb: an ARM7TDMI stores `PC + 12` in
            // ARM state and `PC + 6` in Thumb, an ARM926EJ-S `PC + 8` and
            // `PC + 4`. Measured against the corpus's empty-register-list
            // vectors, which are the only ones that store `R15` in Thumb.
            let offset = u32::from(self.cfg.store_pc_offset);
            let offset = if self.flag(psr::T) {
                offset / 2
            } else {
                offset
            };
            self.insn_addr.wrapping_add(offset)
        } else {
            self.reg(index)
        }
    }

    /// Write `R15` as a plain branch within the current instruction set.
    ///
    /// ARMv5 data-processing writes to `R15` do *not* interwork: the low bits
    /// are ignored and the core stays in whichever state it was in.
    fn branch_to(&mut self, target: u32) {
        // The manual's pseudocode writes the value to `R15` *unmasked* for an
        // ARM data-processing branch (ARM ARM A4.1.35's `Rd = shifter_operand`
        // with `Rd == R15`) and masks only bit 0 in Thumb (A7.1.6). The low
        // address bits are dropped by the fetch, not by the register — an
        // ARMv4T part observably keeps bit 1 in `R15` here — so masking them
        // away at the write is wrong, however tidy it looks.
        self.state.regs.r[15] = if self.flag(psr::T) {
            target & !1
        } else {
            target
        };
        self.branched = true;
        self.cycle(2);
    }

    /// Write `R15` as an interworking branch: bit 0 selects Thumb.
    ///
    /// `BX`, `BLX`, and — new in ARMv5 — every `LDR` and `LDM` that loads
    /// `R15` (ARM ARM A4.1.23, A4.1.20).
    fn branch_exchange(&mut self, target: u32) {
        let thumb = target & 1 != 0;
        self.set_flag(psr::T, thumb);
        // `PC = Rm AND 0xFFFFFFFE` in both states — the pseudocode masks bit 0
        // and nothing else. Branching to a non-word-aligned ARM address is
        // UNPREDICTABLE, but the value that lands in `R15` is still the one
        // the manual writes there.
        self.state.regs.r[15] = target & !1;
        self.branched = true;
        self.cycle(2);
    }

    /// Restore `CPSR` from `SPSR` and branch — how every exception returns.
    ///
    /// `SUBS pc, lr, #4`, `MOVS pc, lr` and `LDM ... {pc}^` all land here. In
    /// a mode with no `SPSR` the architecture calls it UNPREDICTABLE; taking
    /// the branch and leaving `CPSR` alone is the least surprising reading and
    /// is what the ARM7TDMI does.
    fn return_from_exception(&mut self, target: u32) {
        if let Some(spsr) = self.state.regs.spsr() {
            self.state.regs.write_cpsr(spsr);
        }
        self.state.regs.r[15] = if self.flag(psr::T) {
            target & !1
        } else {
            target
        };
        self.branched = true;
        self.cycle(2);
    }

    // -----------------------------------------------------------------
    // Exceptions
    // -----------------------------------------------------------------

    /// Where the vector table sits.
    ///
    /// Either the core was configured for high vectors, or CP15's `V` bit says
    /// so; the machine may fix it and the guest may change it, and both have to
    /// work (ARM ARM A2.6.11).
    fn vector_base(&self) -> u32 {
        if self.cfg.high_vectors || self.mmu.high_vectors() {
            0xffff_0000
        } else {
            0
        }
    }

    /// Enter an exception: bank `CPSR`, set `R14`, mask, and vector.
    fn take_exception(&mut self, kind: Exception, return_address: u32) {
        let old_cpsr = self.state.regs.cpsr;
        self.state.regs.set_mode(kind.mode());
        self.state.regs.set_spsr(old_cpsr);
        if kind != Exception::Reset {
            self.state.regs.r[14] = return_address;
        }
        self.state.regs.cpsr |= psr::I;
        if kind.masks_fiq() {
            self.state.regs.cpsr |= psr::F;
        }
        // Exceptions are always entered in ARM state.
        self.state.regs.cpsr &= !psr::T;
        self.state.regs.r[15] = self.vector_base().wrapping_add(kind.vector());
        self.branched = true;
        // 2S + 1N in ARM's accounting: the refill, plus the cycle the core
        // spends discovering it has to take the exception at all.
        self.cycle(3);
    }

    /// Take the exception an [`Abort`] calls for, and tell CP15 about it.
    fn take_abort(&mut self, abort: Abort) {
        self.mmu.report_abort(abort.va, abort.fault, abort.kind);
        if abort.kind.is_fetch() {
            let lr = self.insn_addr.wrapping_add(4);
            self.take_exception(Exception::PrefetchAbort, lr);
        } else {
            let lr = self.insn_addr.wrapping_add(8);
            self.take_exception(Exception::DataAbort, lr);
        }
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// Whether accesses are privileged right now.
    fn privileged(&self) -> bool {
        self.state.regs.mode().is_privileged()
    }

    fn translate(&mut self, va: u32, kind: AccessKind, privileged: bool) -> Ex<u32> {
        self.mmu
            .translate(va, kind, privileged)
            .map_err(|fault| Abort { kind, va, fault })
    }

    /// Reject a misaligned access, when the machine asked us to.
    ///
    /// Off by default because ARMv5's default is off: CP15's `A` bit enables
    /// alignment checking, and with it clear an unaligned word load rotates
    /// instead of faulting (ARM ARM A2.8.2).
    fn check_alignment(&self, va: u32, width: Width, kind: AccessKind) -> Ex {
        if self.cfg.alignment_faults && !width.is_aligned(u64::from(va)) {
            return Err(Abort {
                kind,
                va,
                fault: Fault::ALIGNMENT,
            });
        }
        Ok(())
    }

    /// The byte order to present a multi-byte value in.
    ///
    /// The address space carries per-region endianness (`ROADMAP.md` §4.1) and
    /// the CPU carries its own; where they differ, the CPU swaps. In the
    /// ordinary all-little-endian machine this costs one comparison and no
    /// lookup.
    fn to_cpu_order(&self, pa: u32, width: Width, value: u32) -> u32 {
        if self.cfg.endian == Endian::Little
            || self.space.endian_at(u64::from(pa)) == self.cfg.endian
        {
            return value;
        }
        match width {
            Width::U8 => value,
            Width::U16 => u32::from((value as u16).swap_bytes()),
            _ => value.swap_bytes(),
        }
    }

    fn load(&mut self, va: u32, width: Width, privileged: bool) -> Ex<u32> {
        self.check_alignment(va, width, AccessKind::Read)?;
        let pa = self.translate(va, AccessKind::Read, privileged)?;
        self.cycle(1);
        let attrs = self.attrs.with_privileged(privileged);
        match self.space.read(u64::from(pa), width, attrs) {
            Ok(v) => Ok(self.to_cpu_order(pa, width, v as u32)),
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = va;
                Err(Abort {
                    kind: AccessKind::Read,
                    va,
                    fault: Fault::EXTERNAL,
                })
            }
        }
    }

    fn store(&mut self, va: u32, width: Width, value: u32, privileged: bool) -> Ex {
        self.check_alignment(va, width, AccessKind::Write)?;
        let pa = self.translate(va, AccessKind::Write, privileged)?;
        self.cycle(1);
        let attrs = self.attrs.with_privileged(privileged);
        let value = self.to_cpu_order(pa, width, value);
        match self
            .space
            .write(u64::from(pa), width, u64::from(value), attrs)
        {
            Ok(()) => Ok(()),
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = va;
                Err(Abort {
                    kind: AccessKind::Write,
                    va,
                    fault: Fault::EXTERNAL,
                })
            }
        }
    }

    /// A word load, with the unaligned rotate ARMv5 performs.
    ///
    /// The bus access is always word-aligned; the loaded value is rotated
    /// right by eight times the address's low two bits, which puts the
    /// addressed byte in the low lane (ARM ARM A4.1.23).
    fn load_word_rotated(&mut self, va: u32, privileged: bool) -> Ex<u32> {
        if self.cfg.alignment_faults {
            self.check_alignment(va, Width::U32, AccessKind::Read)?;
        }
        let value = self.load(va & !3, Width::U32, privileged)?;
        Ok(value.rotate_right((va & 3) * 8))
    }

    /// A halfword load, with the rotate an ARM7TDMI applies to an odd address.
    ///
    /// ARMv5 calls an unaligned `LDRH` UNPREDICTABLE (ARM ARM A4.1.21). The
    /// hardware answer is the same one it gives an unaligned `LDR`: the bus
    /// access is aligned and the value comes back rotated so the addressed
    /// byte lands in the low lane. Measured against the corpus.
    fn load_half_rotated(&mut self, va: u32, privileged: bool) -> Ex<u32> {
        if self.cfg.alignment_faults {
            self.check_alignment(va, Width::U16, AccessKind::Read)?;
        }
        let value = self.load(va & !1, Width::U16, privileged)?;
        Ok(value.rotate_right((va & 1) * 8))
    }

    /// A signed halfword load.
    ///
    /// At an odd address an ARM7TDMI sign-extends the *byte* rather than the
    /// halfword — `LDRSH` degenerates into `LDRSB` — which is one of the
    /// better known consequences of ARMv5 leaving the case UNPREDICTABLE
    /// (ARM ARM A4.1.22). The access on the bus is still a halfword.
    fn load_signed_half(&mut self, va: u32, privileged: bool) -> Ex<u32> {
        if self.cfg.alignment_faults {
            self.check_alignment(va, Width::U16, AccessKind::Read)?;
        }
        let value = self.load(va & !1, Width::U16, privileged)?;
        Ok(if va & 1 != 0 {
            i32::from((value >> 8) as u8 as i8) as u32
        } else {
            i32::from(value as u16 as i16) as u32
        })
    }

    /// An instruction fetch.
    fn fetch(&mut self, va: u32, width: Width) -> Ex<u32> {
        // A fetch never presents the low address bits: an ARM core reads the
        // word containing `R15` and a Thumb core the halfword. This is where
        // an unaligned `R15` stops mattering, which is why the PC-writing
        // helpers above are free to keep the bits the manual keeps.
        let va = va & !(width.bytes() as u32 - 1);
        let pa = self.translate(va, AccessKind::Fetch, self.privileged())?;
        self.cycle(1);
        match self.space.read(u64::from(pa), width, self.attrs) {
            Ok(v) => Ok(self.to_cpu_order(pa, width, v as u32)),
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = va;
                Err(Abort {
                    kind: AccessKind::Fetch,
                    va,
                    fault: Fault::EXTERNAL,
                })
            }
        }
    }

    // -----------------------------------------------------------------
    // The barrel shifter
    // -----------------------------------------------------------------

    /// Shift by an immediate, with the three zero-amount special cases
    /// (ARM ARM A5.1.5–A5.1.11).
    fn shift_immediate(ty: ShiftType, value: u32, amount: u8, carry_in: bool) -> (u32, bool) {
        match (ty, amount) {
            (ShiftType::Lsl, 0) => (value, carry_in),
            (ShiftType::Lsl, n) => (value << n, value & (1 << (32 - u32::from(n))) != 0),
            // `LSR #0` encodes `LSR #32`, which is a zero result and bit 31 in
            // the carry.
            (ShiftType::Lsr, 0) => (0, value & 0x8000_0000 != 0),
            (ShiftType::Lsr, n) => (value >> n, value & (1 << (u32::from(n) - 1)) != 0),
            (ShiftType::Asr, 0) => (
                if value & 0x8000_0000 != 0 {
                    u32::MAX
                } else {
                    0
                },
                value & 0x8000_0000 != 0,
            ),
            (ShiftType::Asr, n) => (
                ((value as i32) >> n) as u32,
                value & (1 << (u32::from(n) - 1)) != 0,
            ),
            // `ROR #0` encodes `RRX`: a 33-bit rotate through the carry.
            (ShiftType::Ror, 0) => ((u32::from(carry_in) << 31) | (value >> 1), value & 1 != 0),
            (ShiftType::Ror, n) => (
                value.rotate_right(u32::from(n)),
                value & (1 << (u32::from(n) - 1)) != 0,
            ),
        }
    }

    /// Shift by the low byte of a register (ARM ARM A5.1.6–A5.1.12).
    ///
    /// A shift of 32 or more is not the same as a shift of 31: `LSL` by 32
    /// leaves bit 0 in the carry and by 33 leaves nothing, and Rust's shift
    /// operators would panic or wrap rather than say so.
    fn shift_register(ty: ShiftType, value: u32, amount: u32, carry_in: bool) -> (u32, bool) {
        let amount = amount & 0xff;
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
                    // A rotate by a multiple of 32 is the identity, but it
                    // still reports bit 31 as the carry.
                    (value, value & 0x8000_0000 != 0)
                } else {
                    (value.rotate_right(low), value & (1 << (low - 1)) != 0)
                }
            }
        }
    }

    /// Evaluate an addressing-mode-1 operand, returning it and the shifter's
    /// carry-out.
    fn eval_operand(&self, operand: Operand, extra: u32, carry_in: bool) -> (u32, bool) {
        match operand {
            Operand::Imm { imm8, rotate } => {
                let value = u32::from(imm8).rotate_right(u32::from(rotate) * 2);
                // A zero rotate leaves the carry alone; anything else takes it
                // from bit 31 of the result (ARM ARM A5.1.3).
                if rotate == 0 {
                    (value, carry_in)
                } else {
                    (value, value & 0x8000_0000 != 0)
                }
            }
            Operand::Reg { rm, shift } => {
                let value = self.reg_plus(rm, extra);
                match shift {
                    Shift::Imm { ty, amount } => Exec::shift_immediate(ty, value, amount, carry_in),
                    Shift::Reg { ty, rs } => {
                        // `Rs` is read in the instruction's *first* cycle,
                        // before the extra internal cycle a register-controlled
                        // shift costs, so `R15` here still reads as the
                        // instruction plus eight — while `Rn` and `Rm`, read
                        // one cycle later, read plus twelve. Measured against
                        // `SingleStepTests/ARM7TDMI`, which is the only
                        // authority available for an UNPREDICTABLE case.
                        let amount = self.reg(rs);
                        Exec::shift_register(ty, value, amount, carry_in)
                    }
                }
            }
        }
    }

    /// Evaluate an addressing-mode-2/3 offset. Never register-shifted, so no
    /// carry comes out of it.
    fn eval_offset(&self, offset: Offset) -> u32 {
        match offset {
            Offset::Imm(imm) => u32::from(imm),
            Offset::Reg { rm, shift } => {
                let value = self.reg(rm);
                match shift {
                    Shift::Imm { ty, amount } => {
                        Exec::shift_immediate(ty, value, amount, self.flag(psr::C)).0
                    }
                    // Addressing mode 2 never encodes a register-controlled
                    // shift; the bit that would say so selects mode 3.
                    Shift::Reg { .. } => value,
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // The ALU
    // -----------------------------------------------------------------

    /// One data-processing operation and the flags it would set.
    ///
    /// Returns `(result, n, z, c, v)`. The caller decides whether to write the
    /// result and whether to commit the flags, which is what separates `SUB`
    /// from `CMP` and `S` from no `S`.
    fn alu(
        op: DpOp,
        a: u32,
        b: u32,
        carry_in: bool,
        shifter_carry: bool,
        v_in: bool,
    ) -> (u32, bool, bool) {
        // Every arithmetic operation is `a + b + carry` with one operand
        // possibly inverted, which is how the hardware does it and why the
        // carry flag means "not borrow" on a subtract.
        fn add(a: u32, b: u32, carry: bool) -> (u32, bool, bool) {
            let wide = u64::from(a) + u64::from(b) + u64::from(carry);
            let result = wide as u32;
            let c = wide > u64::from(u32::MAX);
            let v = (a ^ result) & (b ^ result) & 0x8000_0000 != 0;
            (result, c, v)
        }
        match op {
            DpOp::And | DpOp::Tst => (a & b, shifter_carry, v_in),
            DpOp::Eor | DpOp::Teq => (a ^ b, shifter_carry, v_in),
            DpOp::Sub | DpOp::Cmp => add(a, !b, true),
            DpOp::Rsb => add(!a, b, true),
            DpOp::Add | DpOp::Cmn => add(a, b, false),
            DpOp::Adc => add(a, b, carry_in),
            DpOp::Sbc => add(a, !b, carry_in),
            DpOp::Rsc => add(!a, b, carry_in),
            DpOp::Orr => (a | b, shifter_carry, v_in),
            DpOp::Mov => (b, shifter_carry, v_in),
            DpOp::Bic => (a & !b, shifter_carry, v_in),
            DpOp::Mvn => (!b, shifter_carry, v_in),
        }
    }

    /// Commit `N`, `Z`, `C` and `V` from an [`Exec::alu`] result.
    fn commit_flags(&mut self, result: u32, c: bool, v: bool) {
        self.set_nz(result);
        self.set_flag(psr::C, c);
        self.set_flag(psr::V, v);
    }

    /// How many internal cycles a multiply costs.
    ///
    /// The multiplier retires eight bits of `Rs` per cycle and stops early
    /// once the remaining bits are all sign (or all zero, for an unsigned
    /// long multiply) — which is why `MUL r0, r1, #1` is cheap and
    /// `MUL r0, r1, #0x01000000` is not.
    fn multiply_cycles(rs: u32, signed: bool) -> u64 {
        let terminates = |mask: u32, top: u32| {
            let high = rs & mask;
            high == 0 || (signed && high == mask && rs & top != 0)
        };
        if terminates(0xffff_ff00, 0x80) {
            1
        } else if terminates(0xffff_0000, 0x8000) {
            2
        } else if terminates(0xff00_0000, 0x0080_0000) {
            3
        } else {
            4
        }
    }

    // -----------------------------------------------------------------
    // ARM state
    // -----------------------------------------------------------------

    fn step_arm(&mut self) {
        let pc = self.state.regs.r[15];
        self.insn_addr = pc;
        self.branched = false;
        // Reading R15 must yield the instruction's address plus eight, so put
        // it there before executing and take it back afterwards.
        self.state.regs.r[15] = pc.wrapping_add(8);

        let word = match self.fetch(pc, Width::U32) {
            Ok(w) => w,
            Err(abort) => {
                self.take_abort(abort);
                return;
            }
        };
        let decoded = super::isa::decode(word);
        let saved = self.state.regs.r;
        let outcome = if decoded.passes(self.state.regs.cpsr) {
            self.execute_arm(decoded)
        } else {
            Ok(())
        };
        if let Err(abort) = outcome {
            // The base restored abort model (ARM926EJ-S TRM 3.5): a data abort
            // leaves the base register as it was. We restore the whole general
            // file, which subsumes it — the registers an aborted instruction
            // had already written are architecturally UNPREDICTABLE anyway,
            // and "unchanged" is the reproducible answer.
            self.state.regs.r = saved;
            self.state.regs.r[15] = pc;
            self.take_abort(abort);
            return;
        }
        if !self.branched {
            self.state.regs.r[15] = pc.wrapping_add(4);
        }
    }

    #[allow(clippy::too_many_lines)] // One arm per encoding; splitting it hides the table.
    fn execute_arm(&mut self, decoded: Decoded) -> Ex {
        match decoded.insn {
            Insn::DataProc {
                op,
                s,
                rd,
                rn,
                operand,
            } => {
                let register_shift = operand.is_register_shifted();
                if register_shift {
                    self.cycle(1);
                }
                let extra = if register_shift { 4 } else { 0 };
                let carry_in = self.flag(psr::C);
                let (b, shifter_carry) = self.eval_operand(operand, extra, carry_in);
                let a = if op.reads_rn() {
                    self.reg_plus(rn, extra)
                } else {
                    0
                };
                let (result, c, v) =
                    Exec::alu(op, a, b, carry_in, shifter_carry, self.flag(psr::V));
                if !op.writes_result() {
                    self.commit_flags(result, c, v);
                    return Ok(());
                }
                if rd & 0xf == 15 {
                    if s && self.state.regs.spsr().is_some() {
                        self.return_from_exception(result);
                    } else {
                        // `S` with `Rd == R15` in User or System mode has no
                        // `SPSR` to restore and the architecture calls it
                        // UNPREDICTABLE. An ARM7TDMI sets the flags from the
                        // result as an ordinary `S` would and branches, which
                        // is both measurable and the more useful of the two
                        // readings — silently swallowing the flag update is
                        // the alternative.
                        if s {
                            self.commit_flags(result, c, v);
                        }
                        self.branch_to(result);
                    }
                } else {
                    self.set_reg(rd, result);
                    if s {
                        self.commit_flags(result, c, v);
                    }
                }
                Ok(())
            }
            Insn::Mrs { rd, spsr } => {
                let value = if spsr {
                    self.state.regs.spsr().unwrap_or(self.state.regs.cpsr)
                } else {
                    self.state.regs.cpsr
                };
                if rd & 0xf == 15 {
                    self.write_pc_without_flush(value);
                } else {
                    self.set_reg(rd, value);
                }
                Ok(())
            }
            Insn::Msr {
                spsr,
                mask,
                operand,
            } => {
                let value = match operand {
                    Operand::Imm { .. } => operand.immediate().unwrap_or(0),
                    Operand::Reg { rm, .. } => self.reg(rm),
                };
                self.write_psr(spsr, mask, value);
                Ok(())
            }
            Insn::Bx { rm } => {
                let target = self.reg(rm);
                self.branch_exchange(target);
                Ok(())
            }
            Insn::BlxReg { rm } => {
                let target = self.reg(rm);
                self.state.regs.r[14] = self.insn_addr.wrapping_add(4);
                self.branch_exchange(target);
                Ok(())
            }
            Insn::Clz { rd, rm } => {
                let value = self.reg(rm);
                self.set_reg(rd, value.leading_zeros());
                Ok(())
            }
            Insn::Bkpt { imm } => {
                self.state.last_bkpt = imm;
                // With no debug hardware attached, BKPT is a Prefetch Abort
                // (ARM ARM A4.1.10).
                let lr = self.insn_addr.wrapping_add(4);
                self.take_exception(Exception::PrefetchAbort, lr);
                Ok(())
            }
            Insn::Branch { link, offset } => {
                let target = self.state.regs.r[15].wrapping_add(offset as u32);
                if link {
                    self.state.regs.r[14] = self.insn_addr.wrapping_add(4);
                }
                self.branch_to(target);
                Ok(())
            }
            Insn::BlxImm { offset } => {
                let target = self.state.regs.r[15].wrapping_add(offset as u32);
                self.state.regs.r[14] = self.insn_addr.wrapping_add(4);
                // The immediate form always lands in Thumb state, whatever the
                // low bit of the computed address says (ARM ARM A4.1.11).
                self.set_flag(psr::T, true);
                self.state.regs.r[15] = target & !1;
                self.branched = true;
                self.cycle(2);
                Ok(())
            }
            Insn::Mul {
                accumulate,
                s,
                rd,
                rn,
                rm,
                rs,
            } => {
                // A multiply spends internal cycles before its operands are
                // latched, so `R15` reads as the instruction plus twelve here
                // just as it does under a register-controlled shift. Using
                // `R15` at all is UNPREDICTABLE; this is what an ARM7TDMI
                // does, measured against the corpus.
                let a = self.reg_plus(rm, 4);
                let b = self.reg_plus(rs, 4);
                self.cycle(Exec::multiply_cycles(b, true));
                let mut result = a.wrapping_mul(b);
                if accumulate {
                    self.cycle(1);
                    result = result.wrapping_add(self.reg_plus(rn, 4));
                }
                if s {
                    // `C` is unaffected in ARMv5 and above (ARM ARM A4.1.40);
                    // only ARMv4 destroyed it.
                    self.set_nz(result);
                }
                // Unlike `MRS`, a multiply into `R15` flushes: the corpus
                // shows the refill.
                if rd & 0xf == 15 {
                    self.branch_to(result);
                } else {
                    self.set_reg(rd, result);
                }
                Ok(())
            }
            Insn::MulLong {
                signed,
                accumulate,
                s,
                rdhi,
                rdlo,
                rm,
                rs,
            } => {
                let a = self.reg_plus(rm, 4);
                let b = self.reg_plus(rs, 4);
                self.cycle(Exec::multiply_cycles(b, signed) + 1);
                let product = if signed {
                    (i64::from(a as i32).wrapping_mul(i64::from(b as i32))) as u64
                } else {
                    u64::from(a) * u64::from(b)
                };
                let result = if accumulate {
                    self.cycle(1);
                    let acc = (u64::from(self.reg_plus(rdhi, 4)) << 32)
                        | u64::from(self.reg_plus(rdlo, 4));
                    product.wrapping_add(acc)
                } else {
                    product
                };
                if rdlo & 0xf == 15 {
                    self.branch_to(result as u32);
                } else {
                    self.set_reg(rdlo, result as u32);
                }
                if rdhi & 0xf == 15 {
                    self.branch_to((result >> 32) as u32);
                } else {
                    self.set_reg(rdhi, (result >> 32) as u32);
                }
                if s {
                    self.set_flag(psr::N, result & 0x8000_0000_0000_0000 != 0);
                    self.set_flag(psr::Z, result == 0);
                }
                Ok(())
            }
            Insn::Saturating { op, rd, rm, rn } => {
                let m = self.reg(rm) as i32;
                let n = self.reg(rn) as i32;
                let result = match op {
                    SatOp::QAdd => self.saturating_add(m, n),
                    SatOp::QSub => self.saturating_sub(m, n),
                    SatOp::QDAdd => {
                        let doubled = self.saturating_add(n, n);
                        self.saturating_add(m, doubled)
                    }
                    SatOp::QDSub => {
                        let doubled = self.saturating_add(n, n);
                        self.saturating_sub(m, doubled)
                    }
                };
                self.set_reg(rd, result as u32);
                Ok(())
            }
            Insn::HalfMul {
                op,
                rd,
                rn,
                rm,
                rs,
                x,
                y,
            } => {
                self.half_multiply(op, rd, rn, rm, rs, x, y);
                Ok(())
            }
            Insn::LoadStore {
                load,
                byte,
                up,
                index,
                rn,
                rd,
                offset,
            } => self.load_store(load, byte, up, index, rn, rd, offset),
            Insn::LoadStoreExtra {
                op,
                up,
                index,
                rn,
                rd,
                offset,
            } => self.load_store_extra(op, up, index, rn, rd, offset),
            Insn::BlockTransfer {
                load,
                before,
                up,
                user,
                writeback,
                rn,
                list,
            } => self.block_transfer(load, before, up, user, writeback, rn, list),
            Insn::Swap { byte, rd, rn, rm } => {
                // `SWP` spends an internal cycle between its read and its
                // write, so `R15` reads as the instruction plus twelve here
                // too — the same rule as a multiply or a register-controlled
                // shift. Using `R15` at all is UNPREDICTABLE.
                let addr = self.reg_plus(rn, 4);
                let privileged = self.privileged();
                let value = if byte {
                    self.load(addr, Width::U8, privileged)?
                } else {
                    self.load_word_rotated(addr, privileged)?
                };
                let source = self.reg_plus(rm, 4);
                if byte {
                    self.store(addr, Width::U8, source & 0xff, privileged)?;
                } else {
                    self.store(addr & !3, Width::U32, source, privileged)?;
                }
                self.cycle(1);
                self.set_reg(rd, value);
                Ok(())
            }
            Insn::Swi { imm } => {
                self.state.last_swi = imm;
                let lr = self.insn_addr.wrapping_add(4);
                self.take_exception(Exception::Swi, lr);
                Ok(())
            }
            // A hint with no architectural effect. It still costs its fetch,
            // which is already charged.
            Insn::Pld { .. } => Ok(()),
            Insn::Cdp {
                cp,
                opc1,
                crd,
                crn,
                crm,
                opc2,
                ..
            } => {
                let op = CpOp {
                    cp,
                    opc1,
                    crd,
                    crn,
                    crm,
                    opc2,
                };
                match self.coprocessor(cp).map(|c| c.cdp(op)) {
                    Some(Ok(effect)) => self.apply_effect(effect),
                    _ => self.undefined_instruction(),
                }
                Ok(())
            }
            Insn::CpReg {
                cp,
                load,
                opc1,
                rd,
                crn,
                crm,
                opc2,
                ..
            } => {
                self.coprocessor_register(cp, load, opc1, rd, crn, crm, opc2);
                Ok(())
            }
            Insn::CpRegPair {
                cp,
                load,
                opc,
                rd,
                rn,
                crm,
            } => {
                let Some(coprocessor) = self.coprocessor(cp) else {
                    self.undefined_instruction();
                    return Ok(());
                };
                if load {
                    match coprocessor.mrrc(cp, opc, crm) {
                        Ok(value) => {
                            self.set_reg(rd, value as u32);
                            self.set_reg(rn, (value >> 32) as u32);
                        }
                        Err(CpFault::Undefined) => self.undefined_instruction(),
                    }
                } else {
                    let value = (u64::from(self.reg(rn)) << 32) | u64::from(self.reg(rd));
                    match coprocessor.mcrr(cp, opc, crm, value) {
                        Ok(effect) => self.apply_effect(effect),
                        Err(CpFault::Undefined) => self.undefined_instruction(),
                    }
                }
                Ok(())
            }
            Insn::CpTransfer {
                cp,
                load,
                long,
                crd,
                rn,
                index,
                up,
                offset,
                ..
            } => self.coprocessor_transfer(cp, load, long, crd, rn, index, up, offset),
            Insn::Undefined => {
                self.undefined_instruction();
                Ok(())
            }
        }
    }

    /// Write `CPSR` or the current `SPSR` through an `MSR` field mask.
    fn write_psr(&mut self, spsr: bool, mask: u8, value: u32) {
        let mut byte_mask = 0u32;
        if mask & 0b0001 != 0 {
            byte_mask |= 0x0000_00ff;
        }
        if mask & 0b0010 != 0 {
            byte_mask |= 0x0000_ff00;
        }
        if mask & 0b0100 != 0 {
            byte_mask |= 0x00ff_0000;
        }
        if mask & 0b1000 != 0 {
            byte_mask |= 0xff00_0000;
        }
        if spsr {
            if let Some(current) = self.state.regs.spsr() {
                self.state
                    .regs
                    .set_spsr((current & !byte_mask) | (value & byte_mask));
            }
            return;
        }
        // User mode may only touch the flags byte; everything else is silently
        // ignored rather than faulting (ARM ARM A4.1.39).
        if !self.privileged() {
            byte_mask &= 0xff00_0000;
        }
        // The T bit is written like any other bit of the control byte. The
        // architecture warns programmers not to change it this way and calls
        // the result UNPREDICTABLE, but A4.1.39's pseudocode still assigns
        // `CPSR[7:0] = operand[7:0]` wholesale, and hardware takes the write.
        // Filtering it out would be the emulator silently overriding what the
        // guest asked for, which is the worse failure of the two.
        let new = (self.state.regs.cpsr & !byte_mask) | (value & byte_mask);
        self.state.regs.write_cpsr(new);
    }

    /// Signed saturating add, setting the sticky `Q` flag on overflow.
    fn saturating_add(&mut self, a: i32, b: i32) -> i32 {
        match a.checked_add(b) {
            Some(v) => v,
            None => {
                self.set_flag(psr::Q, true);
                if a < 0 { i32::MIN } else { i32::MAX }
            }
        }
    }

    /// Signed saturating subtract, setting the sticky `Q` flag on overflow.
    fn saturating_sub(&mut self, a: i32, b: i32) -> i32 {
        match a.checked_sub(b) {
            Some(v) => v,
            None => {
                self.set_flag(psr::Q, true);
                if a < 0 { i32::MIN } else { i32::MAX }
            }
        }
    }

    /// One half of a register, sign-extended.
    fn half_of(value: u32, half: Half) -> i32 {
        match half {
            Half::Bottom => i32::from(value as u16 as i16),
            Half::Top => i32::from((value >> 16) as u16 as i16),
        }
    }

    /// The `SMLA`/`SMUL` family (ARM ARM A10.1).
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn half_multiply(&mut self, op: HalfMulOp, rd: u8, rn: u8, rm: u8, rs: u8, x: Half, y: Half) {
        let m = self.reg(rm);
        let s = self.reg(rs);
        self.cycle(if op == HalfMulOp::Smlal { 2 } else { 1 });
        match op {
            HalfMulOp::Smul => {
                let product = Exec::half_of(m, x).wrapping_mul(Exec::half_of(s, y));
                self.set_reg(rd, product as u32);
            }
            HalfMulOp::Smla => {
                let product = Exec::half_of(m, x).wrapping_mul(Exec::half_of(s, y));
                let addend = self.reg(rn) as i32;
                // The product cannot overflow; the accumulation can, and that
                // is what Q records. The result still wraps.
                if product.checked_add(addend).is_none() {
                    self.set_flag(psr::Q, true);
                }
                self.set_reg(rd, product.wrapping_add(addend) as u32);
            }
            HalfMulOp::Smulw | HalfMulOp::Smlaw => {
                let wide = i64::from(m as i32) * i64::from(Exec::half_of(s, y));
                let product = (wide >> 16) as i32;
                if op == HalfMulOp::Smulw {
                    self.set_reg(rd, product as u32);
                } else {
                    let addend = self.reg(rn) as i32;
                    if product.checked_add(addend).is_none() {
                        self.set_flag(psr::Q, true);
                    }
                    self.set_reg(rd, product.wrapping_add(addend) as u32);
                }
            }
            HalfMulOp::Smlal => {
                // `rd` is RdHi and `rn` is RdLo for this one encoding.
                let product = i64::from(Exec::half_of(m, x)) * i64::from(Exec::half_of(s, y));
                let acc = ((u64::from(self.reg(rd)) << 32) | u64::from(self.reg(rn))) as i64;
                let result = acc.wrapping_add(product) as u64;
                self.set_reg(rn, result as u32);
                self.set_reg(rd, (result >> 32) as u32);
            }
        }
    }

    /// Addressing mode 2: `LDR`, `STR`, `LDRB`, `STRB`.
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn load_store(
        &mut self,
        load: bool,
        byte: bool,
        up: bool,
        index: Index,
        rn: u8,
        rd: u8,
        offset: Offset,
    ) -> Ex {
        let base = self.reg(rn);
        let delta = self.eval_offset(offset);
        let adjusted = if up {
            base.wrapping_add(delta)
        } else {
            base.wrapping_sub(delta)
        };
        let address = match index {
            Index::Pre { .. } => adjusted,
            Index::Post { .. } => base,
        };
        // The `T` forms make a privileged access behave as an unprivileged
        // one, which is how a kernel copies from user space safely.
        let privileged = match index {
            Index::Post { unprivileged: true } => false,
            _ => self.privileged(),
        };

        if load {
            let value = if byte {
                self.load(address, Width::U8, privileged)?
            } else {
                self.load_word_rotated(address, privileged)?
            };
            self.cycle(1);
            // Writeback first, so a load into the base register wins.
            if index.writes_base() {
                self.set_reg(rn, adjusted);
            }
            if rd & 0xf == 15 {
                // ARMv5: loading the PC is an interworking branch.
                self.branch_exchange(value);
            } else {
                self.set_reg(rd, value);
            }
        } else {
            let value = self.store_value(rd);
            if byte {
                self.store(address, Width::U8, value & 0xff, privileged)?;
            } else {
                self.store(address & !3, Width::U32, value, privileged)?;
            }
            if index.writes_base() {
                self.set_reg(rn, adjusted);
            }
        }
        Ok(())
    }

    /// Addressing mode 3: halfword, signed byte and doubleword transfers.
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn load_store_extra(
        &mut self,
        op: ExtraOp,
        up: bool,
        index: Index,
        rn: u8,
        rd: u8,
        offset: Offset,
    ) -> Ex {
        let base = self.reg(rn);
        let delta = self.eval_offset(offset);
        let adjusted = if up {
            base.wrapping_add(delta)
        } else {
            base.wrapping_sub(delta)
        };
        let address = match index {
            Index::Pre { .. } => adjusted,
            Index::Post { .. } => base,
        };
        let privileged = self.privileged();

        match op {
            ExtraOp::Strh => {
                let value = self.store_value(rd);
                self.store(address & !1, Width::U16, value & 0xffff, privileged)?;
            }
            ExtraOp::Ldrh => {
                let value = self.load_half_rotated(address, privileged)?;
                self.cycle(1);
                self.finish_extra_load(index, rn, adjusted, rd, value);
                return Ok(());
            }
            ExtraOp::Ldrsb => {
                let value = self.load(address, Width::U8, privileged)?;
                self.cycle(1);
                let value = i32::from(value as u8 as i8) as u32;
                self.finish_extra_load(index, rn, adjusted, rd, value);
                return Ok(());
            }
            ExtraOp::Ldrsh => {
                let value = self.load_signed_half(address, privileged)?;
                self.cycle(1);
                self.finish_extra_load(index, rn, adjusted, rd, value);
                return Ok(());
            }
            ExtraOp::Ldrd => {
                // Rd must be even and must not be R14, whose pair would be the
                // PC; both are UNPREDICTABLE, and refusing is more useful than
                // inventing a meaning.
                if rd & 1 != 0 || rd & 0xf == 14 {
                    self.undefined_instruction();
                    return Ok(());
                }
                let low = self.load(address & !3, Width::U32, privileged)?;
                let high = self.load((address & !3).wrapping_add(4), Width::U32, privileged)?;
                self.cycle(1);
                if index.writes_base() {
                    self.set_reg(rn, adjusted);
                }
                self.set_reg(rd, low);
                self.set_reg(rd + 1, high);
                return Ok(());
            }
            ExtraOp::Strd => {
                if rd & 1 != 0 || rd & 0xf == 14 {
                    self.undefined_instruction();
                    return Ok(());
                }
                let low = self.reg(rd);
                let high = self.reg(rd + 1);
                self.store(address & !3, Width::U32, low, privileged)?;
                self.store((address & !3).wrapping_add(4), Width::U32, high, privileged)?;
            }
        }
        if index.writes_base() {
            self.set_reg(rn, adjusted);
        }
        Ok(())
    }

    /// The tail every addressing-mode-3 load shares.
    fn finish_extra_load(&mut self, index: Index, rn: u8, adjusted: u32, rd: u8, value: u32) {
        if index.writes_base() {
            self.set_reg(rn, adjusted);
        }
        if rd & 0xf == 15 {
            // Not an interworking branch: ARMv5 made *word* loads into `R15`
            // interwork (ARM ARM A4.1.23), and a halfword or signed byte
            // cannot carry an address, so the case stays UNPREDICTABLE and the
            // hardware simply branches. The corpus agrees.
            self.branch_to(value);
        } else {
            self.set_reg(rd, value);
        }
    }

    /// Addressing mode 4: `LDM` and `STM`.
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    #[allow(clippy::too_many_lines)]
    fn block_transfer(
        &mut self,
        load: bool,
        before: bool,
        up: bool,
        user: bool,
        writeback: bool,
        rn: u8,
        list: u16,
    ) -> Ex {
        let base = self.reg(rn);
        let count = list.count_ones();
        // An empty register list is UNPREDICTABLE in ARMv5. The ARM7TDMI
        // transfers R15 alone and moves the base by 0x40, which is what the
        // one public conformance corpus expects, so that is what we do.
        let (effective_list, span) = if count == 0 {
            (0x8000u16, 0x40u32)
        } else {
            (list, count * 4)
        };
        let (start, writeback_value) = match (before, up) {
            (false, true) => (base, base.wrapping_add(span)),
            (true, true) => (base.wrapping_add(4), base.wrapping_add(span)),
            (false, false) => (
                base.wrapping_sub(span).wrapping_add(4),
                base.wrapping_sub(span),
            ),
            (true, false) => (base.wrapping_sub(span), base.wrapping_sub(span)),
        };

        let loads_pc = load && effective_list & 0x8000 != 0;
        // The S bit means "the user-mode bank" unless this is an LDM that
        // restores the PC, in which case it means "and restore CPSR from
        // SPSR" (ARM ARM A4.1.20, A4.1.21).
        let user_bank = user && !loads_pc;
        let privileged = self.privileged();

        if load && writeback {
            // Writeback before the loads, so a list containing the base ends
            // up holding the loaded value rather than the new base.
            self.write_base(rn, writeback_value, user_bank);
        }
        // Whether the base writeback has already overwritten `R15`. If it
        // has, a store of `R15` writes what the register now *holds* — the new
        // base — rather than the pipelined `PC + 12` that an untouched `R15`
        // would contribute.
        let mut pc_overwritten = false;
        if !load && writeback {
            // For a store, the base is written back after the first transfer
            // unless it is the lowest register in the list — that is the one
            // case where hardware stores the original value.
            let lowest = effective_list.trailing_zeros();
            if effective_list & (1 << (rn & 0xf)) != 0 && u32::from(rn & 0xf) != lowest {
                self.write_base(rn, writeback_value, user_bank);
                pc_overwritten = rn & 0xf == 15;
            }
        }

        let mut address = start;
        for index in 0u8..16 {
            if effective_list & (1 << index) == 0 {
                continue;
            }
            // The base of a block transfer is forced word-aligned: bits
            // [1:0] are ignored on the bus and, unlike `LDR`, no rotation is
            // applied (ARM ARM A5.4.1).
            let word = address & !3;
            if load {
                let value = self.load(word, Width::U32, privileged)?;
                if user_bank {
                    self.state.regs.set_reg_in_mode(Mode::USER, index, value);
                } else if index == 15 {
                    if user {
                        // `LDM ... {pc}^`: the exception return.
                        self.return_from_exception(value);
                    } else {
                        self.branch_exchange(value);
                    }
                } else {
                    self.set_reg(index, value);
                }
            } else {
                // `R15` is not a banked register, so the `S` bit's
                // redirection does not apply to it — and the
                // implementation-defined store-of-`R15` offset still does.
                let value = if index == 15 {
                    if pc_overwritten {
                        self.reg(15)
                    } else {
                        self.store_value(index)
                    }
                } else if user_bank {
                    self.state.regs.reg_in_mode(Mode::USER, index)
                } else {
                    self.reg(index)
                };
                self.store(word, Width::U32, value, privileged)?;
            }
            address = address.wrapping_add(4);
        }
        if load {
            self.cycle(1);
        }
        if !load && writeback {
            self.write_base(rn, writeback_value, user_bank);
        }
        Ok(())
    }

    /// Write a block transfer's base register back.
    ///
    /// With the `S` bit set and `R15` absent from the list, the base is *read*
    /// from the current mode's bank but *written back* to the User one — as if
    /// the S bit forced the register file's write port over and left the read
    /// port alone. Combining `S` with writeback is UNPREDICTABLE (ARM ARM
    /// A5.4.6); this is what an ARM7TDMI does, and every one of the corpus's
    /// vectors for the case agrees.
    fn write_base(&mut self, rn: u8, value: u32, user_bank: bool) {
        // `R15` has no User bank to redirect to, and writing it is a branch
        // however it was reached.
        if user_bank && rn & 0xf != 15 {
            self.state.regs.set_reg_in_mode(Mode::USER, rn, value);
        } else {
            self.set_reg(rn, value);
        }
    }

    // -----------------------------------------------------------------
    // Coprocessors
    // -----------------------------------------------------------------

    fn coprocessor(&self, cp: u8) -> Option<&Arc<dyn Coprocessor>> {
        self.coprocessors.get((cp & 0xf) as usize)?.as_ref()
    }

    fn apply_effect(&mut self, effect: CpEffect) {
        if effect.halt {
            self.state.halted = true;
        }
    }

    fn undefined_instruction(&mut self) {
        let lr = self
            .insn_addr
            .wrapping_add(if self.flag(psr::T) { 2 } else { 4 });
        self.take_exception(Exception::Undefined, lr);
    }

    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn coprocessor_register(
        &mut self,
        cp: u8,
        load: bool,
        opc1: u8,
        rd: u8,
        crn: u8,
        crm: u8,
        opc2: u8,
    ) {
        let op = CpOp {
            cp,
            opc1,
            crd: 0,
            crn,
            crm,
            opc2,
        };
        let Some(coprocessor) = self.coprocessor(cp).cloned() else {
            self.undefined_instruction();
            return;
        };
        if load {
            match coprocessor.mrc(op) {
                Ok(value) => {
                    if rd & 0xf == 15 {
                        // `MRC` to R15 loads the flags, not the PC
                        // (ARM ARM A4.1.32).
                        let flags = value & 0xf000_0000;
                        self.state.regs.cpsr = (self.state.regs.cpsr & 0x0fff_ffff) | flags;
                    } else {
                        self.set_reg(rd, value);
                    }
                }
                Err(CpFault::Undefined) => self.undefined_instruction(),
            }
        } else {
            let value = self.reg(rd);
            match coprocessor.mcr(op, value) {
                Ok(effect) => self.apply_effect(effect),
                Err(CpFault::Undefined) => self.undefined_instruction(),
            }
        }
    }

    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn coprocessor_transfer(
        &mut self,
        cp: u8,
        load: bool,
        long: bool,
        crd: u8,
        rn: u8,
        index: Index,
        up: bool,
        offset: u8,
    ) -> Ex {
        let transfer = CpTransfer {
            cp,
            crd,
            long,
            option: offset,
        };
        let Some(coprocessor) = self.coprocessor(cp).cloned() else {
            self.undefined_instruction();
            return Ok(());
        };
        let Ok(words) = coprocessor.transfer_len(transfer) else {
            self.undefined_instruction();
            return Ok(());
        };
        let base = self.reg(rn);
        let delta = u32::from(offset) * 4;
        let adjusted = if up {
            base.wrapping_add(delta)
        } else {
            base.wrapping_sub(delta)
        };
        let mut address = match index {
            Index::Pre { .. } => adjusted,
            Index::Post { .. } => base,
        };
        let privileged = self.privileged();
        for word in 0..words {
            if load {
                let value = self.load(address, Width::U32, privileged)?;
                if coprocessor.write_word(transfer, word, value).is_err() {
                    self.undefined_instruction();
                    return Ok(());
                }
            } else {
                let Ok(value) = coprocessor.read_word(transfer, word) else {
                    self.undefined_instruction();
                    return Ok(());
                };
                self.store(address, Width::U32, value, privileged)?;
            }
            address = address.wrapping_add(4);
        }
        if index.writes_base() {
            self.set_reg(rn, adjusted);
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Thumb state
    // -----------------------------------------------------------------

    fn step_thumb(&mut self) {
        let pc = self.state.regs.r[15];
        self.insn_addr = pc;
        self.branched = false;
        // In Thumb state R15 reads as the instruction's address plus four.
        self.state.regs.r[15] = pc.wrapping_add(4);

        let half = match self.fetch(pc, Width::U16) {
            Ok(w) => w as u16,
            Err(abort) => {
                self.take_abort(abort);
                return;
            }
        };
        let decoded = super::thumb::decode(half);
        let saved = self.state.regs.r;
        if let Err(abort) = self.execute_thumb(decoded) {
            self.state.regs.r = saved;
            self.state.regs.r[15] = pc;
            self.take_abort(abort);
            return;
        }
        if !self.branched {
            self.state.regs.r[15] = pc.wrapping_add(2);
        }
    }

    #[allow(clippy::too_many_lines)] // One arm per format; splitting it hides the table.
    fn execute_thumb(&mut self, insn: Thumb) -> Ex {
        let carry_in = self.flag(psr::C);
        match insn {
            Thumb::ShiftImm { ty, rd, rm, imm } => {
                let value = self.reg(rm);
                let (result, c) = Exec::shift_immediate(ty, value, imm, carry_in);
                self.set_reg(rd, result);
                self.set_nz(result);
                self.set_flag(psr::C, c);
                Ok(())
            }
            Thumb::AddSub {
                sub,
                rd,
                rn,
                operand,
            } => {
                let a = self.reg(rn);
                let b = match operand {
                    SmallOperand::Reg(r) => self.reg(r),
                    SmallOperand::Imm(i) => u32::from(i),
                };
                let op = if sub { DpOp::Sub } else { DpOp::Add };
                let (result, c, v) = Exec::alu(op, a, b, carry_in, carry_in, self.flag(psr::V));
                self.set_reg(rd, result);
                self.commit_flags(result, c, v);
                Ok(())
            }
            Thumb::AluImm { op, rd, imm } => {
                let a = self.reg(rd);
                let b = u32::from(imm);
                let v_in = self.flag(psr::V);
                let (result, c, v) = match op {
                    ImmOp::Mov => Exec::alu(DpOp::Mov, 0, b, carry_in, carry_in, v_in),
                    ImmOp::Cmp => Exec::alu(DpOp::Cmp, a, b, carry_in, carry_in, v_in),
                    ImmOp::Add => Exec::alu(DpOp::Add, a, b, carry_in, carry_in, v_in),
                    ImmOp::Sub => Exec::alu(DpOp::Sub, a, b, carry_in, carry_in, v_in),
                };
                if op != ImmOp::Cmp {
                    self.set_reg(rd, result);
                }
                if op == ImmOp::Mov {
                    // `MOV Rd, #imm8` sets only N and Z; there is no shifter
                    // in this encoding to produce a carry.
                    self.set_nz(result);
                } else {
                    self.commit_flags(result, c, v);
                }
                Ok(())
            }
            Thumb::Alu { op, rd, rm } => {
                self.thumb_alu(op, rd, rm, carry_in);
                Ok(())
            }
            Thumb::HiReg { op, rd, rm } => {
                let a = self.reg(rd);
                let b = self.reg(rm);
                match op {
                    HiOp::Add => {
                        let result = a.wrapping_add(b);
                        if rd & 0xf == 15 {
                            self.branch_to(result);
                        } else {
                            self.set_reg(rd, result);
                        }
                    }
                    HiOp::Cmp => {
                        let (result, c, v) =
                            Exec::alu(DpOp::Cmp, a, b, carry_in, carry_in, self.flag(psr::V));
                        self.commit_flags(result, c, v);
                    }
                    HiOp::Mov => {
                        if rd & 0xf == 15 {
                            self.branch_to(b);
                        } else {
                            self.set_reg(rd, b);
                        }
                    }
                }
                Ok(())
            }
            Thumb::BranchExchange { link, rm } => {
                let target = self.reg(rm);
                if link {
                    // `BLX Rm` returns to the halfword after this one, with
                    // bit 0 set so the return is to Thumb (ARM ARM A7.1.12).
                    self.state.regs.r[14] = self.insn_addr.wrapping_add(2) | 1;
                }
                self.branch_exchange(target);
                Ok(())
            }
            Thumb::LoadLiteral { rd, imm } => {
                // The literal pool is word-aligned relative to PC + 4, which
                // is why bit 1 of the address is dropped.
                let address = (self.state.regs.r[15] & !3).wrapping_add(u32::from(imm) * 4);
                let privileged = self.privileged();
                let value = self.load(address, Width::U32, privileged)?;
                self.cycle(1);
                self.set_reg(rd, value);
                Ok(())
            }
            Thumb::MemReg { op, rd, rn, rm } => {
                let address = self.reg(rn).wrapping_add(self.reg(rm));
                self.thumb_mem_reg(op, rd, address)
            }
            Thumb::MemImm {
                load,
                size,
                rd,
                rn,
                imm,
            } => {
                let address = self.reg(rn).wrapping_add(u32::from(imm) * size.bytes());
                self.thumb_mem_sized(load, size, rd, address)
            }
            Thumb::MemStack { load, rd, imm } => {
                let address = self.reg(13).wrapping_add(u32::from(imm) * 4);
                self.thumb_mem_sized(load, MemSize::Word, rd, address)
            }
            Thumb::AddPcSp { sp, rd, imm } => {
                let base = if sp {
                    self.reg(13)
                } else {
                    self.state.regs.r[15] & !3
                };
                self.set_reg(rd, base.wrapping_add(u32::from(imm) * 4));
                Ok(())
            }
            Thumb::AdjustStack { sub, imm } => {
                let delta = u32::from(imm) * 4;
                let sp = self.reg(13);
                self.set_reg(
                    13,
                    if sub {
                        sp.wrapping_sub(delta)
                    } else {
                        sp.wrapping_add(delta)
                    },
                );
                Ok(())
            }
            Thumb::PushPop { load, extra, list } => {
                let mut full = u16::from(list);
                if extra {
                    full |= if load { 0x8000 } else { 0x4000 };
                }
                // PUSH is STMDB sp!, POP is LDMIA sp!.
                self.block_transfer(load, !load, load, false, true, 13, full)
            }
            Thumb::BlockTransfer { load, rn, list } => {
                self.block_transfer(load, false, true, false, true, rn, u16::from(list))
            }
            Thumb::BranchCond { cond, offset } => {
                if cond.passes(self.state.regs.cpsr) {
                    let target = self.state.regs.r[15].wrapping_add(offset as u32);
                    self.branch_to(target);
                }
                Ok(())
            }
            Thumb::Swi { imm } => {
                self.state.last_swi = u32::from(imm);
                let lr = self.insn_addr.wrapping_add(2);
                self.take_exception(Exception::Swi, lr);
                Ok(())
            }
            Thumb::Bkpt { imm } => {
                self.state.last_bkpt = u16::from(imm);
                let lr = self.insn_addr.wrapping_add(4);
                self.take_exception(Exception::PrefetchAbort, lr);
                Ok(())
            }
            Thumb::Branch { offset } => {
                let target = self.state.regs.r[15].wrapping_add(offset as u32);
                self.branch_to(target);
                Ok(())
            }
            Thumb::BranchLinkPrefix { offset } => {
                self.state.regs.r[14] = self.state.regs.r[15].wrapping_add(offset as u32);
                Ok(())
            }
            Thumb::BranchLinkSuffix { exchange, offset } => {
                let target = self.state.regs.r[14].wrapping_add(offset);
                self.state.regs.r[14] = self.insn_addr.wrapping_add(2) | 1;
                if exchange {
                    self.set_flag(psr::T, false);
                    self.state.regs.r[15] = target & !3;
                } else {
                    self.state.regs.r[15] = target & !1;
                }
                self.branched = true;
                self.cycle(2);
                Ok(())
            }
            Thumb::Undefined => {
                self.undefined_instruction();
                Ok(())
            }
        }
    }

    /// Format 4, the register-to-register ALU. Every operation sets the flags.
    fn thumb_alu(&mut self, op: AluOp, rd: u8, rm: u8, carry_in: bool) {
        let a = self.reg(rd);
        let b = self.reg(rm);
        let v_in = self.flag(psr::V);
        let dp = |op| Exec::alu(op, a, b, carry_in, carry_in, v_in);
        let (result, c, v, writes) = match op {
            AluOp::And => (dp(DpOp::And).0, carry_in, v_in, true),
            AluOp::Eor => (dp(DpOp::Eor).0, carry_in, v_in, true),
            AluOp::Adc => {
                let (r, c, v) = dp(DpOp::Adc);
                (r, c, v, true)
            }
            AluOp::Sbc => {
                let (r, c, v) = dp(DpOp::Sbc);
                (r, c, v, true)
            }
            AluOp::Tst => (dp(DpOp::And).0, carry_in, v_in, false),
            AluOp::Cmp => {
                let (r, c, v) = dp(DpOp::Cmp);
                (r, c, v, false)
            }
            AluOp::Cmn => {
                let (r, c, v) = dp(DpOp::Cmn);
                (r, c, v, false)
            }
            AluOp::Orr => (dp(DpOp::Orr).0, carry_in, v_in, true),
            AluOp::Bic => (dp(DpOp::Bic).0, carry_in, v_in, true),
            AluOp::Mvn => (dp(DpOp::Mvn).0, carry_in, v_in, true),
            // `NEG Rd, Rm` is `RSB Rd, Rm, #0` — note the operand order.
            AluOp::Neg => {
                let (r, c, v) = Exec::alu(DpOp::Rsb, b, 0, carry_in, carry_in, v_in);
                (r, c, v, true)
            }
            AluOp::Mul => {
                self.cycle(Exec::multiply_cycles(a, true));
                (a.wrapping_mul(b), carry_in, v_in, true)
            }
            AluOp::Lsl | AluOp::Lsr | AluOp::Asr | AluOp::Ror => {
                self.cycle(1);
                let ty = match op {
                    AluOp::Lsl => ShiftType::Lsl,
                    AluOp::Lsr => ShiftType::Lsr,
                    AluOp::Asr => ShiftType::Asr,
                    _ => ShiftType::Ror,
                };
                let (r, c) = Exec::shift_register(ty, a, b, carry_in);
                (r, c, v_in, true)
            }
        };
        if writes {
            self.set_reg(rd, result);
        }
        self.commit_flags(result, c, v);
    }

    /// Thumb formats 7 and 8: the eight register-offset accesses.
    fn thumb_mem_reg(&mut self, op: MemRegOp, rd: u8, address: u32) -> Ex {
        let privileged = self.privileged();
        if op.is_load() {
            let value = match op {
                MemRegOp::Ldr => self.load_word_rotated(address, privileged)?,
                MemRegOp::Ldrb => self.load(address, Width::U8, privileged)?,
                MemRegOp::Ldrh => self.load_half_rotated(address, privileged)?,
                MemRegOp::Ldrsb => {
                    let v = self.load(address, Width::U8, privileged)?;
                    i32::from(v as u8 as i8) as u32
                }
                _ => self.load_signed_half(address, privileged)?,
            };
            self.cycle(1);
            self.set_reg(rd, value);
        } else {
            let value = self.reg(rd);
            match op {
                MemRegOp::Strb => self.store(address, Width::U8, value & 0xff, privileged)?,
                MemRegOp::Strh => {
                    self.store(address & !1, Width::U16, value & 0xffff, privileged)?;
                }
                _ => self.store(address & !3, Width::U32, value, privileged)?,
            }
        }
        Ok(())
    }

    /// Thumb formats 9, 10 and 11: the scaled-immediate accesses.
    fn thumb_mem_sized(&mut self, load: bool, size: MemSize, rd: u8, address: u32) -> Ex {
        let privileged = self.privileged();
        if load {
            let value = match size {
                MemSize::Byte => self.load(address, Width::U8, privileged)?,
                MemSize::Half => self.load_half_rotated(address, privileged)?,
                MemSize::Word => self.load_word_rotated(address, privileged)?,
            };
            self.cycle(1);
            self.set_reg(rd, value);
        } else {
            let value = self.reg(rd);
            match size {
                MemSize::Byte => self.store(address, Width::U8, value & 0xff, privileged)?,
                MemSize::Half => {
                    self.store(address & !1, Width::U16, value & 0xffff, privileged)?;
                }
                MemSize::Word => self.store(address & !3, Width::U32, value, privileged)?,
            }
        }
        Ok(())
    }
}
