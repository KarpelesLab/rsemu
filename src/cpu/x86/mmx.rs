//! Executing MMX: the eight `MM` registers and every instruction that names
//! one.
//!
//! # The registers are the x87 registers
//!
//! There is no MMX register file. `MMn` is the 64-bit significand of x87
//! **physical** register `Rn` (*Intel SDM* volume 1 §9.5), and the aliasing
//! comes with three side effects that make the two views exclusive rather
//! than merely overlapping — each is what an operating system's lazy
//! save/restore and a program's x87 code rely on:
//!
//! - **Every MMX instruction except `EMMS` sets `TOP` to zero and every tag
//!   to valid** before it does anything else, so `MMn` and `ST(n)` name the
//!   same register from then on, and an x87 instruction that follows finds a
//!   full stack rather than silently reading a stale one ([`X87::enter_mmx`]).
//! - **A write to `MMn` sets bits 79:64 of `Rn` to all ones**, so an x87
//!   instruction that reads an MMX result sees a NaN or an infinity rather
//!   than a plausible number ([`X87::set_mm`]).
//! - **`EMMS` marks every register empty**, which is how MMX code hands the
//!   unit back to x87 code ([`X87::empty_mmx`]).
//!
//! Because nothing is stored anywhere else, `FSAVE`/`FRSTOR` and
//! `FXSAVE`/`FXRSTOR` save and restore the MMX state without a line of code
//! that knows it exists, and so does every snapshot of the x87 unit.
//!
//! # The gates
//!
//! The *Intel SDM* volume 2 exception tables for the MMX instructions
//! (§22.25's "Exception Conditions for Legacy SIMD/MMX Instructions" in
//! its several flavours), in the order applied here:
//!
//! | | an `MM`-only instruction | one that also names an `XMM` register |
//! | --- | --- | --- |
//! | the feature is absent | `#UD` | `#UD` |
//! | `CR0.EM` set | `#UD` — not `#NM`: there is no emulation protocol | `#UD` |
//! | `CR4.OSFXSR` clear | — | `#UD` |
//! | `CR0.TS` set | `#NM` | `#NM` |
//! | an x87 exception is pending | `#MF` | `#MF` |
//!
//! The feature is `Features::mmx` for the original set, `sse` as well for
//! the integer instructions SSE added on `MM` registers, `sse2` as well for
//! SSE2's, and `ssse3` as well for the `MM` forms of Supplemental SSE3. The pending-exception check and the transition happen only
//! when the instruction actually names an `MM` register: `CVTPI2PS xmm, m64`
//! reads its doublewords from memory and leaves the x87 unit alone, which is
//! the SDM's description of that form ("for operands of the form `xmm, m64`
//! there is no transition") and the only reading under which the rule is
//! about the registers rather than the mnemonic.
//!
//! # The arithmetic
//!
//! The lane kernels are `fpexec`'s, shared with SSE2's packed-integer half:
//! an MMX operation is the same operation on the low quadword of a 128-bit
//! value. Three families need the operands placed rather than simply
//! zero-extended, because their lanes move between halves — the packs, whose
//! source supplies the *upper* half of the result, and the high-half unpacks,
//! which interleave the top four bytes of a quadword rather than of a double
//! quadword — and each says how where it happens.
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapter 9 ("Programming with Intel MMX Technology"),
//! §9.5 and table 9-2 for the aliasing and its side effects, §9.6 for the
//! transitions; volume 2 for every instruction's page and exception table;
//! volume 3 §2.5 and table 2-2 for `CR0.EM`/`CR0.TS`. No copyleft emulator was
//! consulted (`CLAUDE.md`, provenance).

use crate::float::{B32, B64, Round, binary};

#[cfg(doc)]
use super::fpu::X87;

use super::exec::{Ex, Exec, Fault, VEC_MF, VEC_NM, VEC_UD};
use super::fpexec::{is_shift, packed, shift_lanes};
use super::isa::{Arg, Extension, Fields, Insn, ModRm, Op, seg};
use super::prot::{cr0, cr4};
use super::simd::ssse3;

/// Which extension an `MM`-register encoding arrived with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    /// The Pentium MMX's original set.
    Mmx,
    /// The integer instructions SSE added on `MM` registers, and its two
    /// conversions.
    Sse,
    /// SSE2's: three arithmetic instructions, the two cross-file moves and
    /// the double-precision conversions.
    Sse2,
    /// Supplemental SSE3's, every one of which has an `MM` form.
    Ssse3,
}

/// The extension an operation on `MM` registers needs.
const fn level(op: Op) -> Level {
    match op {
        Op::PAVGB
        | Op::PAVGW
        | Op::PMINUB
        | Op::PMAXUB
        | Op::PMINSW
        | Op::PMAXSW
        | Op::PSADBW
        | Op::PSHUFW
        | Op::PEXTRW
        | Op::PINSRW
        | Op::PMOVMSKB
        | Op::PMULHUW
        | Op::MASKMOVQ
        | Op::MOVNTQ
        | Op::CVTPI2PS
        | Op::CVTPS2PI
        | Op::CVTTPS2PI => Level::Sse,
        Op::PADDQ
        | Op::PSUBQ
        | Op::PMULUDQ
        | Op::MOVQ2DQ
        | Op::MOVDQ2Q
        | Op::CVTPI2PD
        | Op::CVTPD2PI
        | Op::CVTTPD2PI => Level::Sse2,
        _ if matches!(op.extension(), Some(Extension::Ssse3)) => Level::Ssse3,
        _ => Level::Mmx,
    }
}

/// Whether a row also names an `XMM` register or a 128-bit operand, and so
/// answers to `CR4.OSFXSR` as well.
const fn touches_xmm(insn: Insn) -> bool {
    matches!(insn.dst, Arg::Vx | Arg::Wx | Arg::Wq | Arg::Ux)
        || matches!(insn.src, Arg::Vx | Arg::Wx | Arg::Wq | Arg::Ux)
}

/// Whether an operand names an `MM` register given the ModRM mode.
const fn names_mm(arg: Arg, register: bool) -> bool {
    match arg {
        Arg::Pq | Arg::Nq => true,
        Arg::Qq | Arg::Qd => register,
        _ => false,
    }
}

impl Exec<'_> {
    /// Execute one instruction whose row [`Insn::is_mmx`] claims.
    #[allow(clippy::too_many_lines)]
    pub(super) fn mmx_instruction(&mut self, f: &Fields) -> Ex<()> {
        let insn = f.insn;
        let op = insn.op;
        let register = f.modrm.is_some_and(ModRm::is_register);
        let features = self.cfg.features;
        let present = features.mmx
            && match level(op) {
                Level::Mmx => true,
                Level::Sse => features.sse,
                Level::Sse2 => features.sse2,
                Level::Ssse3 => features.ssse3,
            };
        if !present {
            return Err(Fault::bare(VEC_UD));
        }
        // The register-only forms with a memory ModRM, and the one
        // memory-only form with a register ModRM, are not encodings at all.
        let register_only =
            matches!(insn.dst, Arg::Nq | Arg::Ux) || matches!(insn.src, Arg::Nq | Arg::Ux);
        if (register_only && !register) || (op == Op::MOVNTQ && register) {
            return Err(Fault::bare(VEC_UD));
        }
        let cr0_value = self.state.sys.cr0;
        if cr0_value & cr0::EM != 0 {
            return Err(Fault::bare(VEC_UD));
        }
        if touches_xmm(insn) && self.state.sys.cr4 & cr4::OSFXSR == 0 {
            return Err(Fault::bare(VEC_UD));
        }
        if cr0_value & cr0::TS != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        let uses_mm =
            op == Op::EMMS || names_mm(insn.dst, register) || names_mm(insn.src, register);
        if uses_mm {
            if self.x87_pending_fault() {
                return Err(Fault::bare(VEC_MF));
            }
            if op == Op::EMMS {
                self.state.x87.empty_mmx();
                return Ok(());
            }
            self.state.x87.enter_mmx();
        }

        // `MM` registers are named by three bits: `REX.R` and `REX.B` do not
        // reach them. The `XMM` operands of the cross-file forms go through
        // `reg_num`/`rm_num` as every other `XMM` operand does.
        let mreg = f.modrm.map_or(0, |m| m.reg & 7);
        let mrm = f.modrm.map_or(0, |m| m.rm & 7);
        let gy = if f.rex_w() { 8 } else { 4 };

        match op {
            // -- Moves -----------------------------------------------------
            Op::MOVD => {
                // `MOVD`, and `MOVQ` between an `MM` register and a 64-bit
                // general one or memory under `REX.W`. A doubleword load
                // zero-extends into the quadword.
                let width = if f.rex_w() { 8 } else { 4 };
                if insn.dst == Arg::Pq {
                    let v = self.read_arg(f, Arg::Ey, width)?;
                    self.state.x87.set_mm(mreg, v);
                } else {
                    let v = self.state.x87.mm(mreg);
                    let v = if width == 8 { v } else { v & 0xffff_ffff };
                    self.write_arg(f, Arg::Ey, width, v)?;
                }
                Ok(())
            }
            Op::MOVQ | Op::MOVNTQ => {
                if insn.dst == Arg::Pq {
                    let v = self.mm_read(f, insn.src)?;
                    self.state.x87.set_mm(mreg, v);
                    Ok(())
                } else {
                    // A non-temporal store is an ordinary store here, for the
                    // reason `66 0F E7` gives in `isa`.
                    let v = self.state.x87.mm(mreg);
                    self.mm_write(f, v)
                }
            }
            Op::MOVQ2DQ => {
                let v = self.state.x87.mm(mrm);
                self.state.sse.set(f.reg_num(), [v, 0]);
                Ok(())
            }
            Op::MOVDQ2Q => {
                let v = self.state.sse.low(f.rm_num());
                self.state.x87.set_mm(mreg, v);
                Ok(())
            }
            // The byte-masked store to `[DS:rDI]`, as `MASKMOVDQU` but eight
            // bytes wide (*Intel SDM* volume 2, `MASKMOVQ`).
            Op::MASKMOVQ => {
                let data = self.state.x87.mm(mreg);
                let mask = self.state.x87.mm(mrm);
                let sr = f.segment(seg::DS);
                let regs = &self.state.regs;
                let base = match f.addrsize {
                    2 => u64::from(regs.word(7)),
                    4 => u64::from(regs.dword(7)),
                    _ => regs.rdi,
                };
                let wrap = match f.addrsize {
                    2 => 0xffff,
                    4 => 0xffff_ffff,
                    _ => u64::MAX,
                };
                for i in 0..8u64 {
                    if (mask >> (8 * i + 7)) & 1 != 0 {
                        let off = base.wrapping_add(i) & wrap;
                        self.write_mem(sr, off, 1, (data >> (8 * i)) & 0xff)?;
                    }
                }
                Ok(())
            }

            // -- Between an `MM` register and a general one ----------------
            Op::PMOVMSKB => {
                let v = self.state.x87.mm(mrm);
                let mut mask = 0u64;
                for i in 0..8 {
                    mask |= ((v >> (8 * i + 7)) & 1) << i;
                }
                self.write_arg(f, Arg::Gy, gy, mask)
            }
            Op::PEXTRW => {
                let v = self.state.x87.mm(mrm);
                let word = (v >> (16 * (f.imm & 3))) & 0xffff;
                self.write_arg(f, Arg::Gy, gy, word)
            }
            Op::PINSRW => {
                let value = self.read_arg(f, Arg::Ew, 2)? & 0xffff;
                let shift = 16 * (f.imm & 3);
                let v = self.state.x87.mm(mreg);
                self.state
                    .x87
                    .set_mm(mreg, (v & !(0xffff << shift)) | (value << shift));
                Ok(())
            }
            Op::PSHUFW => {
                let src = self.mm_read(f, insn.src)?;
                let mut out = 0u64;
                for i in 0..4 {
                    let pick = (f.imm >> (2 * i)) & 3;
                    out |= ((src >> (16 * pick)) & 0xffff) << (16 * i);
                }
                self.state.x87.set_mm(mreg, out);
                Ok(())
            }

            // -- The conversions -------------------------------------------
            //
            // Each lane goes through the same `float` routine and `MXCSR`
            // environment as SSE's own conversions, so the rounding control,
            // the truncating forms, the integer indefinite `0x8000_0000` and
            // `#XM` on an unmasked exception — with the destination left
            // untouched — come with it.
            Op::CVTPI2PS => {
                let src = self.mm_read(f, insn.src)?;
                let env = self.state.sse.env();
                let (lo, f0) = binary::from_signed::<B32>(i64::from(src as u32 as i32), 32, env);
                let (hi, f1) =
                    binary::from_signed::<B32>(i64::from((src >> 32) as u32 as i32), 32, env);
                self.sse_raise(f0 | f1)?;
                // Only the low quadword: the upper two singles are kept.
                let reg = f.reg_num();
                self.state.sse.set_low(reg, (lo & 0xffff_ffff) | (hi << 32));
                Ok(())
            }
            Op::CVTPI2PD => {
                let src = self.mm_read(f, insn.src)?;
                let env = self.state.sse.env();
                // Every doubleword is exact in binary64; the flags are passed
                // on regardless rather than assumed away.
                let (lo, f0) = binary::from_signed::<B64>(i64::from(src as u32 as i32), 32, env);
                let (hi, f1) =
                    binary::from_signed::<B64>(i64::from((src >> 32) as u32 as i32), 32, env);
                self.sse_raise(f0 | f1)?;
                self.state.sse.set(f.reg_num(), [lo, hi]);
                Ok(())
            }
            Op::CVTPS2PI | Op::CVTTPS2PI => {
                // Two singles: a quadword from memory, unaligned allowed.
                let src = self.sse_read_rm(f, Arg::Wq, false)?[0];
                let env = self.state.sse.env();
                let env = if op == Op::CVTTPS2PI {
                    env.round(Round::TowardZero)
                } else {
                    env
                };
                let (lo, f0) = binary::to_signed::<B32>(src & 0xffff_ffff, 32, env);
                let (hi, f1) = binary::to_signed::<B32>(src >> 32, 32, env);
                self.sse_raise(f0 | f1)?;
                let out = u64::from(lo as u32) | (u64::from(hi as u32) << 32);
                self.state.x87.set_mm(mreg, out);
                Ok(())
            }
            Op::CVTPD2PI | Op::CVTTPD2PI => {
                // Two doubles: sixteen bytes, which must be aligned.
                let src = self.sse_read_rm(f, Arg::Wx, true)?;
                let env = self.state.sse.env();
                let env = if op == Op::CVTTPD2PI {
                    env.round(Round::TowardZero)
                } else {
                    env
                };
                let (lo, f0) = binary::to_signed::<B64>(src[0], 32, env);
                let (hi, f1) = binary::to_signed::<B64>(src[1], 32, env);
                self.sse_raise(f0 | f1)?;
                let out = u64::from(lo as u32) | (u64::from(hi as u32) << 32);
                self.state.x87.set_mm(mreg, out);
                Ok(())
            }

            // -- Everything else is a lane operation -----------------------
            _ => {
                // The shift-by-immediate groups name their operand in `r/m`,
                // the `reg` field being the opcode extension.
                if insn.dst == Arg::Nq {
                    let v = self.state.x87.mm(mrm);
                    let out = shift_lanes(op, [v, 0], f.imm & 0xff)[0];
                    self.state.x87.set_mm(mrm, out);
                    return Ok(());
                }
                let src = self.mm_read(f, insn.src)?;
                let dst = self.state.x87.mm(mreg);
                let out = if is_shift(op) {
                    // The count is the whole quadword: anything past the lane
                    // width clears the lane, or fills it with the sign.
                    shift_lanes(op, [dst, 0], src)[0]
                } else if level(op) == Level::Ssse3 {
                    // Eight bytes of operand, which `PSHUFB`, `PALIGNR` and
                    // the horizontal forms see differently from the low half
                    // of sixteen; see `simd::ssse3`.
                    ssse3(op, [dst, 0], [src, 0], 8, f.imm as u8)[0]
                } else {
                    lanes(op, dst, src)
                };
                self.state.x87.set_mm(mreg, out);
                Ok(())
            }
        }
    }

    /// Read a `Qq` or `Qd` operand: an `MM` register, or eight or four bytes
    /// of memory, zero-extended.
    fn mm_read(&mut self, f: &Fields, arg: Arg) -> Ex<u64> {
        if f.modrm.is_some_and(ModRm::is_register) {
            return Ok(self.state.x87.mm(f.modrm.map_or(0, |m| m.rm & 7)));
        }
        let (sr, off) = self.ea();
        let width = if arg == Arg::Qd { 4 } else { 8 };
        self.read_mem(sr, off, width)
    }

    /// Write a `Qq` operand.
    fn mm_write(&mut self, f: &Fields, value: u64) -> Ex<()> {
        if f.modrm.is_some_and(ModRm::is_register) {
            let rm = f.modrm.map_or(0, |m| m.rm & 7);
            self.state.x87.set_mm(rm, value);
            return Ok(());
        }
        let (sr, off) = self.ea();
        self.write_mem(sr, off, 8, value)
    }
}

/// One two-operand MMX operation on 64-bit values.
///
/// `fpexec`'s [`packed`] on the low quadword for everything whose lanes stay
/// where they are. The two families whose lanes move are placed first:
///
/// - **The packs** take the destination's lanes into the low half of the
///   result and the source's into the high half. With the destination and
///   source as the two quadwords of one 128-bit value, the 128-bit pack's low
///   quadword is exactly that.
/// - **The high-half unpacks** interleave the upper four bytes (two words,
///   one doubleword) of each operand — the upper half of a *quadword* — which
///   is the low-half unpack of the two operands shifted down by thirty-two.
fn lanes(op: Op, dst: u64, src: u64) -> u64 {
    match op {
        Op::PAND => dst & src,
        Op::PANDN => !dst & src,
        Op::POR => dst | src,
        Op::PXOR => dst ^ src,
        Op::PACKSSWB | Op::PACKUSWB | Op::PACKSSDW => packed(op, [dst, src], [0, 0], 0)[0],
        Op::PUNPCKHBW | Op::PUNPCKHWD | Op::PUNPCKHDQ => {
            let low = match op {
                Op::PUNPCKHBW => Op::PUNPCKLBW,
                Op::PUNPCKHWD => Op::PUNPCKLWD,
                _ => Op::PUNPCKLDQ,
            };
            packed(low, [dst >> 32, 0], [src >> 32, 0], 0)[0]
        }
        _ => packed(op, [dst, 0], [src, 0], 0)[0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pack_puts_the_source_in_the_upper_half() {
        // Words [0x0100, 0xff80, 0x0005, 0x7fff] and [-1, 0, 0x8000, 1]:
        // signed saturation to bytes, destination first.
        let dst = 0x7fff_0005_ff80_0100;
        let src = 0x0001_8000_0000_ffff;
        assert_eq!(lanes(Op::PACKSSWB, dst, src), 0x0180_00ff_7f05_807f);
        assert_eq!(lanes(Op::PACKUSWB, dst, src), 0x0100_0000_ff05_00ff);
    }

    #[test]
    fn a_high_unpack_interleaves_the_upper_half_of_a_quadword() {
        let a = 0x8786_8584_8382_8180;
        let b = 0x9796_9594_9392_9190;
        assert_eq!(lanes(Op::PUNPCKHBW, a, b), 0x9787_9686_9585_9484);
        assert_eq!(lanes(Op::PUNPCKLBW, a, b), 0x9383_9282_9181_9080);
        assert_eq!(lanes(Op::PUNPCKHWD, a, b), 0x9796_8786_9594_8584);
        assert_eq!(lanes(Op::PUNPCKHDQ, a, b), 0x9796_9594_8786_8584);
    }

    #[test]
    fn psadbw_on_a_quadword_is_one_sum_in_the_low_word() {
        let a = 0x0000_0000_0000_00ff;
        let b = 0x0101_0101_0101_0100;
        assert_eq!(lanes(Op::PSADBW, a, b), 0xff + 7);
    }
}
