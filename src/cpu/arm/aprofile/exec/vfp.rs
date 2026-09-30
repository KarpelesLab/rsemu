//! Executing a VFP instruction: the access checks, then the operation.
//!
//! A child of the interpreter module so it can use [`Exec`]'s own load/store
//! path — translation, permission and alignment faults, the bus-fault latch,
//! byte order and cycle charging — rather than a second one. The only hook
//! in the interpreter proper is the routing test at the top of
//! `execute_arm`, which sends every word in the VFP part of the coprocessor
//! space here when the configured part has VFP.
//!
//! # Who may execute what (DDI 0406C, "Enabling Advanced SIMD and
//! floating-point support", and the `CheckVFPEnabled` pseudocode)
//!
//! In order, each failure an Undefined Instruction exception:
//!
//! 1. **`CPACR`** grants the coprocessor the instruction names (`cp10` for
//!    single precision and the system-register moves, `cp11` for double) at
//!    the current privilege: `0b11` everywhere, `0b01` at PL1 only, `0b00`
//!    and the reserved `0b10` nowhere. On a part with no `CPACR` the regime
//!    reports all ones.
//! 2. **`VMRS`/`VMSR` of anything but `FPSCR`** is PL1-only, and — alone
//!    among VFP instructions — does *not* need `FPEXC.EN`: that is how a
//!    kernel turns the unit on, and how it reads `FPSID` to find out what it
//!    has.
//! 3. **`FPEXC.EN`** is set, for everything else. A lazily context-switching
//!    kernel keeps it clear and takes the trap on a task's first VFP
//!    instruction.
//! 4. The part has the instruction: VFPv3 additions on a VFPv2 part, and
//!    `D16`–`D31` on a D16 part, are UNDEFINED.
//! 5. **`FPSCR.Len` and `Stride` are zero** for a data-processing
//!    instruction: the Cortex-A9 FPU does not implement short vectors and
//!    makes such an instruction UNDEFINED (DDI 0408, "VFP short vectors").
//!
//! `CPACR.ASEDIS` and `D32DIS` are not consulted: the first governs Advanced
//! SIMD, which no encoding here is, and the second is left to whichever CP15
//! reports `cp_access` — it reads as zero on a part where it is RAZ.
//!
//! # Aborts part-way through
//!
//! A load multiple gathers every word before writing a single register, so
//! an abort on the last word leaves the register file exactly as it was and
//! the restarted instruction sees what the first attempt did. A store
//! multiple may have written some words when it aborts; the restart writes
//! them again with the same values, which is what the architecture permits
//! (DDI 0406C: an aborted load multiple leaves its destinations
//! UNKNOWN and is restarted from the beginning). The base register
//! is restored by the interpreter's own abort path, as for `LDM`.
//!
//! # Timing
//!
//! Each memory access is charged where it happens, like every other load or
//! store here, and a load pays the one internal address-to-data cycle the
//! integer loads do. The FPU pipeline itself — a Cortex-A9 `VDIV.F64` takes
//! tens of cycles — is not modelled, which is a stated simplification in the
//! same spirit as the cache model the integer core does not have.

use crate::core::value::{Endian, Width};
use crate::float::Round;

use super::super::cp::{AccessKind, Fault};
use super::super::psr;
use super::super::vfp::{self, fpexc, fpscr, sysreg};
use super::super::vfpisa::{self, DataOp, UnaryOp, VfpInsn};
use super::{Abort, Ex, Exec};

impl Exec<'_> {
    /// Execute one word from the VFP part of the coprocessor space, whose
    /// condition has already passed.
    ///
    /// The word is the A32 encoding; bits 27:0 are the T32 encoding too, so a
    /// Thumb-2 front end can call this with its two halfwords assembled.
    pub(super) fn execute_vfp(&mut self, word: u32) -> Ex {
        let Some(unit) = self.cfg.arch.ext.vfp else {
            self.undefined_instruction();
            return Ok(());
        };
        let Some(insn) = vfpisa::decode(word) else {
            self.undefined_instruction();
            return Ok(());
        };
        if !self.vfp_permitted(word, insn, unit) {
            self.undefined_instruction();
            return Ok(());
        }
        self.run_vfp(insn)
    }

    /// Steps 1–5 of the module docs.
    fn vfp_permitted(&self, word: u32, insn: VfpInsn, unit: super::super::arch::Vfp) -> bool {
        let cp = if word & (1 << 8) != 0 { 11 } else { 10 };
        let privileged = self.privileged();
        let granted = match (self.regime.cp_access >> (cp * 2)) & 3 {
            0b11 => true,
            0b01 => privileged,
            _ => false,
        };
        if !granted {
            return false;
        }
        let v = &self.state.vfp;
        match insn {
            VfpInsn::Sys { reg, .. } if reg != sysreg::FPSCR => {
                if !privileged {
                    return false;
                }
            }
            _ => {
                if !v.enabled() {
                    return false;
                }
            }
        }
        if unit.version < 3 && insn.needs_v3() {
            return false;
        }
        if !unit.d32 && insn.max_double().is_some_and(|d| d >= 16) {
            return false;
        }
        !(insn.is_data_processing() && v.fpscr & (fpscr::LEN | fpscr::STRIDE) != 0)
    }

    #[allow(clippy::too_many_lines)] // One arm per form; splitting hides the table.
    fn run_vfp(&mut self, insn: VfpInsn) -> Ex {
        let env = self.state.vfp.env();
        match insn {
            VfpInsn::Data { op, dp, d, n, m } => {
                let r = &self.state.vfp;
                let (a, b, acc) = (r.get(dp, n), r.get(dp, m), r.get(dp, d));
                let (value, flags) = match op {
                    DataOp::Mla => vfp::mul_acc(dp, acc, a, b, false, false, env),
                    DataOp::Mls => vfp::mul_acc(dp, acc, a, b, true, false, env),
                    DataOp::Nmla => vfp::mul_acc(dp, acc, a, b, true, true, env),
                    DataOp::Nmls => vfp::mul_acc(dp, acc, a, b, false, true, env),
                    DataOp::Mul => vfp::mul(dp, a, b, env),
                    DataOp::Nmul => {
                        let (v, f) = vfp::mul(dp, a, b, env);
                        (vfp::neg(dp, v), f)
                    }
                    DataOp::Add => vfp::add(dp, a, b, env),
                    DataOp::Sub => vfp::sub(dp, a, b, env),
                    DataOp::Div => vfp::div(dp, a, b, env),
                };
                self.vfp_finish(dp, d, value, flags);
            }
            VfpInsn::Unary { op, dp, d, m } => {
                let a = self.state.vfp.get(dp, m);
                let (value, flags) = match op {
                    UnaryOp::Mov => (a, crate::float::Flags::NONE),
                    UnaryOp::Abs => (vfp::abs(dp, a), crate::float::Flags::NONE),
                    UnaryOp::Neg => (vfp::neg(dp, a), crate::float::Flags::NONE),
                    UnaryOp::Sqrt => vfp::sqrt(dp, a, env),
                };
                self.vfp_finish(dp, d, value, flags);
            }
            VfpInsn::MovImm { dp, d, imm8 } => {
                self.state.vfp.set(dp, d, vfp::expand_imm(imm8, dp));
            }
            VfpInsn::Cmp {
                dp,
                d,
                m,
                with_zero,
                signal_all,
            } => {
                let r = &self.state.vfp;
                let a = r.get(dp, d);
                let b = if with_zero { 0 } else { r.get(dp, m) };
                let (nzcv, flags) = vfp::compare(dp, a, b, signal_all, env);
                let r = &mut self.state.vfp;
                r.fpscr = (r.fpscr & !fpscr::FLAGS) | nzcv;
                r.accumulate(flags);
            }
            VfpInsn::CvtPrec { to_double, d, m } => {
                let a = self.state.vfp.get(!to_double, m);
                let (value, flags) = vfp::convert_precision(to_double, a, env);
                self.vfp_finish(to_double, d, value, flags);
            }
            VfpInsn::CvtToInt {
                dp,
                d,
                m,
                signed,
                round_zero,
            } => {
                let env = if round_zero {
                    env.round(Round::TowardZero)
                } else {
                    env
                };
                let a = self.state.vfp.get(dp, m);
                let (value, flags) = vfp::to_int(dp, a, signed, env);
                self.vfp_finish(false, d, u64::from(value), flags);
            }
            VfpInsn::CvtFromInt { dp, d, m, signed } => {
                let a = self.state.vfp.s(m);
                let (value, flags) = vfp::from_int(dp, a, signed, env);
                self.vfp_finish(dp, d, value, flags);
            }
            VfpInsn::CvtFixed {
                dp,
                d,
                to_fixed,
                unsigned,
                size,
                fbits,
            } => {
                let (size, fbits) = (u32::from(size), u32::from(fbits));
                if to_fixed {
                    // Always toward zero (A8.8, `VCVT` fixed-point: "The floating-point to
                    // fixed-point operation uses the Round towards Zero
                    // rounding mode").
                    let a = self.state.vfp.get(dp, d);
                    let env = env.round(Round::TowardZero);
                    let (value, flags) = vfp::to_fixed(dp, a, size, fbits, unsigned, env);
                    // `Extend(result, 32 or 64, unsigned)`: the conversion
                    // already delivers the value sign- or zero-extended to 32.
                    let value = if unsigned || !dp {
                        u64::from(value)
                    } else {
                        value as i32 as i64 as u64
                    };
                    self.vfp_finish(dp, d, value, flags);
                } else {
                    // Always to nearest (the same section: "The fixed-point
                    // to floating-point operation uses the Round to Nearest
                    // rounding mode"), from the low `size` bits.
                    let a = self.state.vfp.get(dp, d) as u32;
                    let env = env.round(Round::TiesEven);
                    let (value, flags) = vfp::from_fixed(dp, a, size, fbits, unsigned, env);
                    self.vfp_finish(dp, d, value, flags);
                }
            }
            VfpInsn::CvtHalf { d, m, to_half, top } => {
                let ahp = self.state.vfp.fpscr & fpscr::AHP != 0;
                let shift = if top { 16 } else { 0 };
                let a = self.state.vfp.s(m);
                if to_half {
                    let (half, flags) = vfp::single_to_half(a, ahp, env);
                    // Only the named halfword changes.
                    let old = self.state.vfp.s(d);
                    let value = (old & !(0xffff << shift)) | (u32::from(half) << shift);
                    self.vfp_finish(false, d, u64::from(value), flags);
                } else {
                    let (value, flags) = vfp::half_to_single((a >> shift) as u16, ahp, env);
                    self.vfp_finish(false, d, u64::from(value), flags);
                }
            }
            VfpInsn::MovCore { to_fp, rt, n } => {
                if rt == 15 {
                    self.undefined_instruction();
                } else if to_fp {
                    let v = self.reg(rt);
                    self.state.vfp.set_s(n, v);
                } else {
                    let v = self.state.vfp.s(n);
                    self.set_reg(rt, v);
                }
            }
            VfpInsn::MovCore2 {
                to_fp,
                dp,
                rt,
                rt2,
                m,
            } => {
                // PC in either, the same register twice on the way out, or a
                // single pair running past S31 is UNPREDICTABLE (A8.8, the
                // two-core-register `VMOV` forms); refused.
                if rt == 15 || rt2 == 15 || (!to_fp && rt == rt2) || (!dp && m == 31) {
                    self.undefined_instruction();
                    return Ok(());
                }
                if to_fp {
                    let (lo, hi) = (self.reg(rt), self.reg(rt2));
                    let r = &mut self.state.vfp;
                    if dp {
                        r.set_d(m, (u64::from(hi) << 32) | u64::from(lo));
                    } else {
                        r.set_s(m, lo);
                        r.set_s(m + 1, hi);
                    }
                } else {
                    let r = &self.state.vfp;
                    let (lo, hi) = if dp {
                        let v = r.d(m);
                        (v as u32, (v >> 32) as u32)
                    } else {
                        (r.s(m), r.s(m + 1))
                    };
                    self.set_reg(rt, lo);
                    self.set_reg(rt2, hi);
                }
            }
            VfpInsn::MovScalar {
                to_fp,
                rt,
                d,
                index,
            } => {
                if rt == 15 {
                    self.undefined_instruction();
                } else if to_fp {
                    let v = self.reg(rt);
                    // D[d] word `index` is S[2d + index] only for d < 16, so
                    // write through the double.
                    let shift = u32::from(index) * 32;
                    let r = &mut self.state.vfp;
                    let old = r.d(d);
                    r.set_d(
                        d,
                        (old & !(0xffff_ffffu64 << shift)) | (u64::from(v) << shift),
                    );
                } else {
                    let v = (self.state.vfp.d(d) >> (u32::from(index) * 32)) as u32;
                    self.set_reg(rt, v);
                }
            }
            VfpInsn::Sys { to_core, rt, reg } => self.vfp_sys(to_core, rt, reg),
            VfpInsn::Mem {
                load,
                dp,
                d,
                rn,
                imm,
                add,
            } => {
                // The literal form reads `Align(PC, 4)`.
                let base = if rn == 15 {
                    self.reg(15) & !3
                } else {
                    self.reg(rn)
                };
                let address = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                if load {
                    let value = if dp {
                        let first = self.vfp_load(address)?;
                        let second = self.vfp_load(address.wrapping_add(4))?;
                        self.pair(first, second)
                    } else {
                        u64::from(self.vfp_load(address)?)
                    };
                    self.cycle(1);
                    self.state.vfp.set(dp, d, value);
                } else if dp {
                    let (first, second) = self.split(self.state.vfp.d(d));
                    self.vfp_store(address, first)?;
                    self.vfp_store(address.wrapping_add(4), second)?;
                } else {
                    let v = self.state.vfp.s(d);
                    self.vfp_store(address, v)?;
                }
            }
            VfpInsn::Multi {
                load,
                dp,
                d,
                rn,
                count,
                add,
                writeback,
                words,
            } => return self.vfp_multi(load, dp, d, rn, count, add, writeback, words),
        }
        Ok(())
    }

    /// Write a result and fold its exceptions into `FPSCR`.
    fn vfp_finish(&mut self, dp: bool, d: u8, value: u64, flags: crate::float::Flags) {
        let r = &mut self.state.vfp;
        r.set(dp, d, value);
        r.accumulate(flags);
    }

    /// `VMRS`/`VMSR` (A8.8, and the B4.1 register descriptions for which registers
    /// exist). The access checks have already run.
    fn vfp_sys(&mut self, to_core: bool, rt: u8, reg: u8) {
        let Some(unit) = self.cfg.arch.ext.vfp else {
            self.undefined_instruction();
            return;
        };
        // `APSR_nzcv` is the only meaning R15 has here.
        if rt == 15 && !(to_core && reg == sysreg::FPSCR) {
            self.undefined_instruction();
            return;
        }
        if to_core {
            let value = match reg {
                sysreg::FPSCR => self.state.vfp.fpscr,
                sysreg::FPSID => vfp::fpsid(unit),
                sysreg::MVFR0 => vfp::mvfr0(unit),
                sysreg::MVFR1 => vfp::mvfr1(unit),
                sysreg::FPEXC => self.state.vfp.fpexc,
                // `FPINST`/`FPINST2` are subarchitecture registers a
                // Cortex-A9 does not have.
                _ => {
                    self.undefined_instruction();
                    return;
                }
            };
            if rt == 15 {
                self.state.regs.cpsr = (self.state.regs.cpsr
                    & !(psr::N | psr::Z | psr::C | psr::V))
                    | (value & fpscr::FLAGS);
            } else {
                self.set_reg(rt, value);
            }
        } else {
            let value = self.reg(rt);
            match reg {
                sysreg::FPSCR => self.state.vfp.fpscr = value & fpscr::WRITABLE,
                sysreg::FPEXC => self.state.vfp.fpexc = value & fpexc::WRITABLE,
                // `FPSID` is read-only and a write is ignored.
                sysreg::FPSID => {}
                _ => self.undefined_instruction(),
            }
        }
    }

    /// `VLDM`/`VSTM` (A8.8), including `VPUSH`/`VPOP` and
    /// `FLDMX`/`FSTMX`, whose odd `imm8` moves the base one word further than
    /// the registers it transfers.
    #[allow(clippy::too_many_arguments)] // The encoding has this many fields.
    fn vfp_multi(
        &mut self,
        load: bool,
        dp: bool,
        d: u8,
        rn: u8,
        count: u8,
        add: bool,
        writeback: bool,
        words: u8,
    ) -> Ex {
        if rn == 15 && writeback {
            // UNPREDICTABLE (A8.8, `VLDM`); refused.
            self.undefined_instruction();
            return Ok(());
        }
        let base = self.reg(rn);
        let span = u32::from(words) * 4;
        let mut address = if add { base } else { base.wrapping_sub(span) };
        let per = if dp { 2 } else { 1 };
        let total = usize::from(count) * per;
        if load {
            // Gather first, commit after: an abort leaves no register
            // half-written.
            let mut buf = [0u32; 32];
            for slot in buf.iter_mut().take(total) {
                *slot = self.vfp_load(address)?;
                address = address.wrapping_add(4);
            }
            self.cycle(1);
            for i in 0..count {
                let reg = d + i;
                if dp {
                    let at = usize::from(i) * 2;
                    let value = self.pair(buf[at], buf[at + 1]);
                    self.state.vfp.set_d(reg, value);
                } else {
                    self.state.vfp.set_s(reg, buf[usize::from(i)]);
                }
            }
        } else {
            for i in 0..count {
                let reg = d + i;
                if dp {
                    let (first, second) = self.split(self.state.vfp.d(reg));
                    self.vfp_store(address, first)?;
                    self.vfp_store(address.wrapping_add(4), second)?;
                    address = address.wrapping_add(8);
                } else {
                    let v = self.state.vfp.s(reg);
                    self.vfp_store(address, v)?;
                    address = address.wrapping_add(4);
                }
            }
        }
        if writeback {
            let value = if add {
                base.wrapping_add(span)
            } else {
                base.wrapping_sub(span)
            };
            self.set_reg(rn, value);
        }
        Ok(())
    }

    /// Whether data accesses are big-endian, which decides the order of a
    /// double's two words in memory: `D[d] = if BigEndian() then
    /// word1:word2 else word2:word1` (A8.8, `VLDR`).
    fn vfp_big_endian(&self) -> bool {
        self.cfg.endian == Endian::Big
    }

    /// Two words, in memory order, as a double.
    fn pair(&self, first: u32, second: u32) -> u64 {
        let (hi, lo) = if self.vfp_big_endian() {
            (first, second)
        } else {
            (second, first)
        };
        (u64::from(hi) << 32) | u64::from(lo)
    }

    /// A double as two words, in memory order.
    fn split(&self, value: u64) -> (u32, u32) {
        let (lo, hi) = (value as u32, (value >> 32) as u32);
        if self.vfp_big_endian() {
            (hi, lo)
        } else {
            (lo, hi)
        }
    }

    /// `MemA[address, 4]`: an *aligned* access. Every VFP load and store is
    /// one, so a misaligned address is an Alignment fault whatever
    /// `SCTLR.A` says (A3.2.1, table A3-1).
    fn vfp_aligned(address: u32, kind: AccessKind) -> Ex {
        if address & 3 != 0 {
            return Err(Abort {
                kind,
                va: address,
                fault: Fault::ALIGNMENT,
            });
        }
        Ok(())
    }

    fn vfp_load(&mut self, address: u32) -> Ex<u32> {
        Self::vfp_aligned(address, AccessKind::Read)?;
        let privileged = self.privileged();
        self.load(address, Width::U32, privileged)
    }

    fn vfp_store(&mut self, address: u32, value: u32) -> Ex {
        Self::vfp_aligned(address, AccessKind::Write)?;
        let privileged = self.privileged();
        self.store(address, Width::U32, value, privileged)
    }
}
