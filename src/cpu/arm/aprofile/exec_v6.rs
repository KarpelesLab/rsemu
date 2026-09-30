//! The interpreter for the A32 instructions ARMv6, ARMv6K, v6T2 and ARMv7
//! added.
//!
//! A child of `exec` so it can reach [`Exec`]'s private state; split out only
//! because the parent was long enough already. Only [`super::super::isa_v6`]
//! produces the variants handled here, and it produces them only for a part
//! that has them, so nothing in this file asks which architecture it is on.
//!
//! # Sources
//!
//! ARM DDI 0406C: A8.8's per-instruction pseudocode for every operation
//! here; A3.4 for the exclusive monitors; B1.3 for the `CPSR` writes `CPS`
//! and `RFE` make; B1.8's "Wait For Event and Send Event" for `WFE`, `SEV`
//! and the event register; B9.3's `CPS`, `RFE` and `SRS` pages. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

use crate::core::value::Width;

use super::super::cp::AccessKind;
use super::super::isa::{BitfieldOp, Decoded, ExSize, HintOp, Index, Insn};
use super::super::media;
use super::super::{Mode, psr};
use super::{Ex, Exec};

impl Exec<'_> {
    /// Execute one of the instructions this module owns.
    #[allow(clippy::too_many_lines)] // One arm per variant, as in `execute_arm`.
    pub(super) fn execute_v6(&mut self, decoded: Decoded) -> Ex {
        match decoded.insn {
            Insn::Parallel { op, rd, rn, rm } => {
                let (value, ge) = media::parallel(op, self.reg(rn), self.reg(rm));
                self.set_reg(rd, value);
                if let Some(ge) = ge {
                    self.set_ge(ge);
                }
            }
            Insn::Extend {
                signed,
                size,
                rd,
                rn,
                rm,
                rotate,
            } => {
                let addend = rn.map_or(0, |rn| self.reg(rn));
                let value =
                    media::extend(size, signed, addend, self.reg(rm), u32::from(rotate) * 8);
                self.set_reg(rd, value);
            }
            Insn::Sel { rd, rn, rm } => {
                let ge = ((self.state.regs.cpsr & psr::GE) >> 16) as u8;
                let value = media::select(ge, self.reg(rn), self.reg(rm));
                self.set_reg(rd, value);
            }
            Insn::Saturate {
                unsigned,
                bits,
                rd,
                rn,
                asr,
                amount,
            } => {
                let source = self.reg(rn);
                // `ASR #0` encodes `ASR #32`; an arithmetic shift by 31 fills
                // with the sign exactly as 32 would.
                let operand = match (asr, amount) {
                    (false, n) => (source << n) as i32,
                    (true, 0) => (source as i32) >> 31,
                    (true, n) => (source as i32) >> n,
                };
                let (value, saturated) = if unsigned {
                    media::unsigned_sat(i64::from(operand), u32::from(bits))
                } else {
                    let (v, q) = media::signed_sat(i64::from(operand), u32::from(bits));
                    (v as u32, q)
                };
                self.set_reg(rd, value);
                if saturated {
                    self.set_flag(psr::Q, true);
                }
            }
            Insn::Saturate16 {
                unsigned,
                bits,
                rd,
                rn,
            } => {
                let (value, saturated) = media::saturate16(self.reg(rn), u32::from(bits), unsigned);
                self.set_reg(rd, value);
                if saturated {
                    self.set_flag(psr::Q, true);
                }
            }
            Insn::Pack {
                tb,
                rd,
                rn,
                rm,
                amount,
            } => {
                let value = media::pack(tb, self.reg(rn), self.reg(rm), u32::from(amount));
                self.set_reg(rd, value);
            }
            Insn::Reverse { op, rd, rm } => {
                let value = op.apply(self.reg(rm));
                self.set_reg(rd, value);
            }
            Insn::DualMul {
                sub,
                exchange,
                rd,
                rn,
                rm,
                ra,
            } => {
                self.cycle(1);
                let (lo, hi) = media::dual_products(self.reg(rn), self.reg(rm), exchange);
                let mut sum = if sub {
                    i64::from(lo) - i64::from(hi)
                } else {
                    i64::from(lo) + i64::from(hi)
                };
                if let Some(ra) = ra {
                    sum += i64::from(self.reg(ra) as i32);
                }
                // `Q` records that the true sum did not fit (A8.8, `SMLAD`); the
                // register gets the low word either way.
                if sum != i64::from(sum as i32) {
                    self.set_flag(psr::Q, true);
                }
                self.set_reg(rd, sum as u32);
            }
            Insn::DualMulLong {
                sub,
                exchange,
                rdhi,
                rdlo,
                rn,
                rm,
            } => {
                self.cycle(2);
                let (lo, hi) = media::dual_products(self.reg(rn), self.reg(rm), exchange);
                let product = if sub {
                    i64::from(lo) - i64::from(hi)
                } else {
                    i64::from(lo) + i64::from(hi)
                };
                let acc = ((u64::from(self.reg(rdhi)) << 32) | u64::from(self.reg(rdlo))) as i64;
                let result = acc.wrapping_add(product) as u64;
                self.set_reg(rdlo, result as u32);
                self.set_reg(rdhi, (result >> 32) as u32);
            }
            Insn::MulHigh {
                sub,
                round,
                rd,
                rn,
                rm,
                ra,
            } => {
                self.cycle(if ra.is_some() { 2 } else { 1 });
                let product = i64::from(self.reg(rn) as i32) * i64::from(self.reg(rm) as i32);
                let base = ra.map_or(0, |ra| i64::from(self.reg(ra) as i32) << 32);
                // Only bits 63:32 of the exact result are kept, so modular
                // 64-bit arithmetic gives the same answer as the manual's
                // unbounded integers.
                let mut result = if sub {
                    base.wrapping_sub(product)
                } else {
                    base.wrapping_add(product)
                };
                if round {
                    result = result.wrapping_add(0x8000_0000);
                }
                self.set_reg(rd, (result >> 32) as u32);
            }
            Insn::Usad8 { rd, rn, rm, ra } => {
                let sum = media::usad8(self.reg(rn), self.reg(rm));
                let acc = ra.map_or(0, |ra| self.reg(ra));
                self.set_reg(rd, acc.wrapping_add(sum));
            }
            Insn::Umaal { rdhi, rdlo, rn, rm } => {
                self.cycle(2);
                // (2^32 - 1)^2 + 2 * (2^32 - 1) is exactly 2^64 - 1: it cannot
                // overflow, which is the point of the instruction.
                let result = u64::from(self.reg(rn)) * u64::from(self.reg(rm))
                    + u64::from(self.reg(rdhi))
                    + u64::from(self.reg(rdlo));
                self.set_reg(rdlo, result as u32);
                self.set_reg(rdhi, (result >> 32) as u32);
            }
            Insn::Mls { rd, rn, rm, ra } => {
                let b = self.reg(rm);
                self.cycle(Exec::multiply_cycles(b, true) + 1);
                let value = self.reg(ra).wrapping_sub(self.reg(rn).wrapping_mul(b));
                self.set_reg(rd, value);
            }
            Insn::Divide { signed, rd, rn, rm } => {
                // A divider retires a few bits per cycle; the exact count is
                // the part's, and a flat charge keeps the model honest about
                // not knowing it.
                self.cycle(4);
                let (n, m) = (self.reg(rn), self.reg(rm));
                // Division by zero returns zero in the A profile (A8.8's
                // `SDIV` and `UDIV` pages); `INT_MIN / -1` wraps to `INT_MIN`.
                let value = match (signed, m) {
                    (_, 0) => 0,
                    (true, _) => (n as i32).wrapping_div(m as i32) as u32,
                    (false, _) => n / m,
                };
                self.set_reg(rd, value);
            }
            Insn::MovWide { top, rd, imm } => {
                let value = if top {
                    (self.reg(rd) & 0xffff) | (u32::from(imm) << 16)
                } else {
                    u32::from(imm)
                };
                self.set_reg(rd, value);
            }
            Insn::Bitfield {
                op,
                rd,
                rn,
                lsb,
                width,
            } => {
                let (lsb, width) = (u32::from(lsb), u32::from(width));
                let low_mask = if width == 32 {
                    u32::MAX
                } else {
                    (1u32 << width) - 1
                };
                let value = match op {
                    BitfieldOp::Bfc => self.reg(rd) & !(low_mask << lsb),
                    BitfieldOp::Bfi => {
                        let mask = low_mask << lsb;
                        (self.reg(rd) & !mask) | ((self.reg(rn) << lsb) & mask)
                    }
                    BitfieldOp::Ubfx => (self.reg(rn) >> lsb) & low_mask,
                    // Move the field's top bit to bit 31, then shift back
                    // arithmetically. Decode guaranteed `lsb + width <= 32`.
                    BitfieldOp::Sbfx => {
                        (((self.reg(rn) << (32 - lsb - width)) as i32) >> (32 - width)) as u32
                    }
                };
                self.set_reg(rd, value);
            }
            Insn::Cps {
                enable,
                a,
                i,
                f,
                mode,
            } => self.change_processor_state(enable, a, i, f, mode),
            Insn::Setend { big } => self.set_flag(psr::E, big),
            Insn::Srs {
                before,
                up,
                writeback,
                mode,
            } => return self.store_return_state(before, up, writeback, mode),
            Insn::Rfe {
                before,
                up,
                writeback,
                rn,
            } => return self.return_from_exception_rfe(before, up, writeback, rn),
            Insn::LoadExclusive {
                size,
                rt,
                rt2,
                rn,
                imm,
            } => return self.load_exclusive(size, rt, rt2, rn, imm),
            Insn::StoreExclusive {
                size,
                rd,
                rt,
                rt2,
                rn,
                imm,
            } => {
                return self.store_exclusive(size, rd, rt, rt2, rn, imm);
            }
            Insn::TableBranch { half, rn, rm } => return self.table_branch(half, rn, rm),
            Insn::LoadStoreDual {
                load,
                rt,
                rt2,
                rn,
                up,
                index,
                imm,
            } => return self.load_store_dual(load, rt, rt2, rn, up, index, imm),
            Insn::Clrex => self.state.monitor.clear(),
            Insn::Hint { op } => self.hint(op),
            // Barriers order memory against other observers and against the
            // instruction stream. This interpreter performs every access in
            // program order and fetches every instruction afresh, so both
            // orderings already hold; there is nothing to wait for.
            Insn::Barrier { .. } => {}
            // Preload hints with no architectural effect, like `PLD`.
            Insn::Pli { .. } | Insn::Pldw { .. } => {}
            // Monitor mode is not modelled (there is no Secure Monitor to
            // enter), so `SMC` cannot do what it names. Undefined rather than
            // a silent no-op: a guest that calls firmware and carries on as
            // if it had answered is harder to debug than one that traps.
            Insn::Smc { .. } => self.undefined_instruction(),
            Insn::Bxj { rm } => {
                let target = self.reg(rm);
                self.branch_exchange(target);
            }
            // Every ARMv5 variant is handled by `execute_arm` and never gets
            // here.
            _ => self.undefined_instruction(),
        }
        Ok(())
    }

    /// Overwrite `CPSR.GE`.
    fn set_ge(&mut self, ge: u8) {
        self.state.regs.cpsr = (self.state.regs.cpsr & !psr::GE) | (u32::from(ge & 0xf) << 16);
    }

    /// `CPS`: privileged only, and a no-op in User mode rather than an
    /// exception (B9.3, `CPS`).
    fn change_processor_state(
        &mut self,
        enable: Option<bool>,
        a: bool,
        i: bool,
        f: bool,
        mode: Option<u8>,
    ) {
        if !self.privileged() {
            return;
        }
        let mut cpsr = self.state.regs.cpsr;
        if let Some(enable) = enable {
            let mut mask = 0;
            if a {
                mask |= psr::A;
            }
            if i {
                mask |= psr::I;
            }
            if f {
                mask |= psr::F;
            }
            if enable {
                cpsr &= !mask;
            } else {
                cpsr |= mask;
            }
        }
        if let Some(mode) = mode {
            // A mode the part does not have is UNPREDICTABLE; keeping the
            // current one is the answer that cannot bank registers into a
            // mode that does not exist. That includes Monitor, which this
            // core does not model.
            if Mode(mode).is_defined() {
                cpsr = (cpsr & !psr::MODE) | u32::from(mode & 0x1f);
            }
        }
        self.state.regs.write_cpsr(cpsr);
    }

    /// `SRS`: store this mode's `LR` and `SPSR` on `mode`'s stack
    /// (B9.3, `SRS`).
    fn store_return_state(&mut self, before: bool, up: bool, writeback: bool, mode: u8) -> Ex {
        let target = Mode(mode);
        // UNPREDICTABLE in User and System (which have no SPSR to store) and
        // for a mode the part lacks. Undefined is the reading that cannot
        // store garbage onto somebody's stack.
        if !self.privileged() || self.state.regs.spsr().is_none() || !target.is_defined() {
            self.undefined_instruction();
            return Ok(());
        }
        let base = self.state.regs.reg_in_mode(target, 13);
        let mut address = if up { base } else { base.wrapping_sub(8) };
        if before == up {
            address = address.wrapping_add(4);
        }
        Exec::require_alignment(address, 4, AccessKind::Write)?;
        let lr = self.reg(14);
        let spsr = self.state.regs.spsr().unwrap_or(0);
        let privileged = self.privileged();
        self.store(address, Width::U32, lr, privileged)?;
        self.store(address.wrapping_add(4), Width::U32, spsr, privileged)?;
        if writeback {
            let new = if up {
                base.wrapping_add(8)
            } else {
                base.wrapping_sub(8)
            };
            self.state.regs.set_reg_in_mode(target, 13, new);
        }
        Ok(())
    }

    /// `RFE`: load `PC` and `CPSR` from memory — an exception return
    /// (B9.3, `RFE`).
    fn return_from_exception_rfe(&mut self, before: bool, up: bool, writeback: bool, rn: u8) -> Ex {
        if !self.privileged() {
            // UNPREDICTABLE in User mode.
            self.undefined_instruction();
            return Ok(());
        }
        let base = self.reg(rn);
        let mut address = if up { base } else { base.wrapping_sub(8) };
        if before == up {
            address = address.wrapping_add(4);
        }
        Exec::require_alignment(address, 4, AccessKind::Read)?;
        let privileged = self.privileged();
        let pc = self.load(address, Width::U32, privileged)?;
        let cpsr = self.load(address.wrapping_add(4), Width::U32, privileged)?;
        self.cycle(1);
        if writeback {
            let new = if up {
                base.wrapping_add(8)
            } else {
                base.wrapping_sub(8)
            };
            self.set_reg(rn, new);
        }
        // `CPSRWriteByInstr(value, '1111', TRUE)`: every field, including the
        // execution state, then `BranchWritePC` in the state just restored.
        self.state.regs.write_cpsr(cpsr);
        self.it_done = true;
        self.exception_returned();
        self.state.regs.r[15] = if self.flag(psr::T) { pc & !1 } else { pc & !3 };
        self.branched = true;
        self.cycle(2);
        Ok(())
    }

    /// The bookkeeping every exception return shares on ARMv6 and later.
    ///
    /// The local monitor is cleared (see [`super::super::monitor`]), and the
    /// event register is set so that a `WFE` issued after an interrupt
    /// handler already did the work it was about to wait for returns at
    /// once instead of sleeping on an event that has been and gone.
    pub(super) fn exception_returned(&mut self) {
        self.state.monitor.clear();
        self.state.event = true;
    }

    /// Translate an exclusive's address, for the monitor's tag. The access
    /// itself goes through `load`/`store`, whose TLB lookup then hits.
    fn exclusive_pa(&mut self, va: u32, bytes: u32, kind: AccessKind) -> Ex<u32> {
        Exec::require_alignment(va, bytes, kind)?;
        let privileged = self.privileged();
        self.translate(va, kind, privileged)
    }

    /// Whether a doubleword exclusive's register pair is one A32 cannot
    /// encode sensibly: an odd or `R14` first register is UNPREDICTABLE there,
    /// as for `LDRD`. T32 names both registers and has no such rule.
    fn bad_a32_pair(&self, size: ExSize, rt: u8) -> bool {
        size == ExSize::Double && !self.flag(psr::T) && (rt & 1 != 0 || rt == 14)
    }

    /// `LDREX`, `LDREXB`, `LDREXH`, `LDREXD` (A8.8's `LDREX*` pages).
    fn load_exclusive(&mut self, size: ExSize, rt: u8, rt2: u8, rn: u8, imm: u16) -> Ex {
        if self.bad_a32_pair(size, rt) {
            self.undefined_instruction();
            return Ok(());
        }
        let address = self.reg(rn).wrapping_add(u32::from(imm));
        let pa = self.exclusive_pa(address, size.bytes(), AccessKind::Read)?;
        let privileged = self.privileged();
        let (low, high) = match size {
            ExSize::Byte => (self.load(address, Width::U8, privileged)?, 0),
            ExSize::Half => (self.load(address, Width::U16, privileged)?, 0),
            ExSize::Word => (self.load(address, Width::U32, privileged)?, 0),
            ExSize::Double => (
                self.load(address, Width::U32, privileged)?,
                self.load(address.wrapping_add(4), Width::U32, privileged)?,
            ),
        };
        self.cycle(1);
        self.state.monitor.mark(pa);
        if let Some(global) = self.global {
            global.mark(self.cfg.requester, u64::from(pa), size.bytes());
        }
        self.set_reg(rt, low);
        if size == ExSize::Double {
            self.set_reg(rt2, high);
        }
        Ok(())
    }

    /// `STREX`, `STREXB`, `STREXH`, `STREXD` (A8.8's `STREX*` pages).
    fn store_exclusive(&mut self, size: ExSize, rd: u8, rt: u8, rt2: u8, rn: u8, imm: u16) -> Ex {
        if self.bad_a32_pair(size, rt) {
            self.undefined_instruction();
            return Ok(());
        }
        let address = self.reg(rn).wrapping_add(u32::from(imm));
        // Translation and its faults come first, pass or fail: the
        // pseudocode's `ExclusiveMonitorsPass` translates before it looks at
        // either monitor.
        let pa = self.exclusive_pa(address, size.bytes(), AccessKind::Write)?;
        let passed = self.state.monitor.covers(pa)
            && self.global.is_none_or(|global| {
                global.store_exclusive(self.cfg.requester, u64::from(pa), size.bytes())
            });
        // A store-exclusive always leaves the local monitor open.
        self.state.monitor.clear();
        if passed {
            let privileged = self.privileged();
            let value = self.reg(rt);
            match size {
                ExSize::Byte => self.store(address, Width::U8, value & 0xff, privileged)?,
                ExSize::Half => self.store(address, Width::U16, value & 0xffff, privileged)?,
                ExSize::Word => self.store(address, Width::U32, value, privileged)?,
                ExSize::Double => {
                    let high = self.reg(rt2);
                    self.store(address, Width::U32, value, privileged)?;
                    self.store(address.wrapping_add(4), Width::U32, high, privileged)?;
                }
            }
        }
        self.set_reg(rd, u32::from(!passed));
        Ok(())
    }

    /// `TBB` and `TBH` (A8.8.411): a forward branch by twice a table entry.
    ///
    /// The PC reads as the instruction plus four, which is also where the
    /// branch is measured from — so a table placed straight after the
    /// instruction is indexed from its own start.
    fn table_branch(&mut self, half: bool, rn: u8, rm: u8) -> Ex {
        let base = self.reg(rn);
        let index = self.reg(rm);
        let privileged = self.privileged();
        let entry = if half {
            self.load_half_rotated(base.wrapping_add(index << 1), privileged)?
        } else {
            self.load(base.wrapping_add(index), Width::U8, privileged)?
        };
        self.cycle(1);
        let target = self.reg(15).wrapping_add(entry << 1);
        self.branch_to(target);
        Ok(())
    }

    /// T32 `LDRD`/`STRD` (A8.8.72–A8.8.74, A8.8.210): the A32 doubleword
    /// accesses with two independent registers.
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn load_store_dual(
        &mut self,
        load: bool,
        rt: u8,
        rt2: u8,
        rn: u8,
        up: bool,
        index: Index,
        imm: u16,
    ) -> Ex {
        let base = self.base_reg(rn);
        let delta = u32::from(imm);
        let adjusted = if up {
            base.wrapping_add(delta)
        } else {
            base.wrapping_sub(delta)
        };
        let address = match index {
            Index::Post { .. } => base,
            Index::Pre { .. } | Index::Unprivileged => adjusted,
        };
        let privileged = self.privileged();
        if load {
            let (low, high) = self.load_dual(address, privileged)?;
            if index.writes_base() {
                self.set_reg(rn, adjusted);
            }
            self.set_reg(rt, low);
            self.set_reg(rt2, high);
        } else {
            let (low, high) = (self.reg(rt), self.reg(rt2));
            self.store_dual(address, low, high, privileged)?;
            if index.writes_base() {
                self.set_reg(rn, adjusted);
            }
        }
        Ok(())
    }

    /// The `MSR`-space hints.
    fn hint(&mut self, op: HintOp) {
        match op {
            HintOp::Wfi => self.state.halted = true,
            HintOp::Wfe => {
                // B1.8 ("Wait For Event"): a set event register is consumed and the wait
                // completes at once; otherwise sleep until an event or an
                // interrupt. `step` owns the waking.
                if self.state.event {
                    self.state.event = false;
                } else {
                    self.state.halted = true;
                    self.state.waiting_for_event = true;
                }
            }
            // With one core, the core that sent the event is the only one
            // that can see it. `Arm::send_event` is the entry point another
            // core's `SEV` will use once there are several.
            HintOp::Sev => self.state.event = true,
            HintOp::Nop | HintOp::Yield | HintOp::Dbg(_) | HintOp::Other(_) => {}
        }
    }
}
