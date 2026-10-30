//! Executing VEX-encoded instructions: AVX.
//!
//! Decoding is `isa`'s. The VEX prefix is recognised there, the row's VEX
//! constraints (`VEX.L`, `VEX.vvvv`, `VEX.W`) are applied there, and an
//! encoding that breaks one arrives here as `Op::UD`. What is here is the
//! gate, the `YMM` operand plumbing, and the instructions.
//!
//! # The gate
//!
//! | | |
//! | --- | --- |
//! | no AVX, real-address or virtual-8086 mode, `CR4.OSXSAVE` clear, `XCR0[2:1]` not `11b` | `#UD` |
//! | `CR0.TS` set | `#NM` |
//!
//! `CR0.EM` is **not** consulted, unlike legacy SSE, and neither is
//! `CR4.OSFXSR`: an operating system says it manages the `YMM` state by
//! enabling it in `XCR0`, and that is the only switch (*Intel SDM* volume 1
//! §14.3, and volume 2 §2.4's exception classes). An unmasked SIMD
//! floating-point exception is `#XM`, or `#UD` without `CR4.OSXMMEXCPT`,
//! exactly as for SSE — the same `Exec::sse_raise`.
//!
//! # Alignment
//!
//! A VEX-encoded memory operand need not be aligned, with the exceptions
//! the exception-class tables name: the explicitly aligned moves
//! (`VMOVAPS`, `VMOVAPD`, `VMOVDQA`) and the non-temporal ones
//! (`VMOVNTPS`, `VMOVNTPD`, `VMOVNTDQ`, `VMOVNTDQA`) must be aligned to
//! their own length — sixteen or thirty-two bytes — and are `#GP(0)`
//! otherwise (*Intel SDM* volume 2 §2.4, exception type 1).
//!
//! # The upper halves
//!
//! A `VEX.128` instruction that writes an `XMM` register zeroes bits 255:128
//! of the `YMM` register under it; a `VEX.256` one writes all 256; a legacy
//! SSE one leaves them as they were (*Intel SDM* volume 1 §14.1.3). The
//! third rule is kept by `fpexec` never touching [`super::fpu::Sse::ymm_hi`],
//! and the first two by every write here going through [`Exec::set_ymm`].
//!
//! # Sharing the arithmetic with SSE
//!
//! Almost every AVX operation on 256 bits is its SSE operation applied to
//! each 128-bit lane independently — the arithmetic, the comparisons, the
//! shuffles and unpacks (with the same immediate in both lanes, or for
//! `VSHUFPD` and the immediate blends the next bits of it), the horizontal
//! adds, the rounds, `VDPPS`. So the lane kernels are written once, as free
//! functions over one 128-bit lane, in `fpexec` and `simd`; the legacy
//! executors call each once, and [`lane_op`] here calls the same one once per
//! lane, with `VEX.vvvv` as the first source. The handful whose `VEX.128` form
//! differs from the legacy one only in where the first source comes from and
//! in zeroing the upper half — the conversions to and from general
//! registers, the inserts and extracts, the string compares — run the legacy
//! executor itself, which reads its first source through
//! `Exec::xmm_src1`, and then have the upper half cleared here.
//!
//! What crosses lanes is written here: the length-changing conversions
//! (`VCVTPS2PD ymm, xmm/m128` and its converses), the broadcasts,
//! `VINSERTF128`, `VEXTRACTF128`, `VPERM2F128`, the masked moves, and the
//! flag-setting tests.
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapter 14 ("Programming with Intel AVX"), volume 2
//! §2.3 (the VEX prefix) and §2.4-§2.8 (the exception classes), and volume
//! 2's instruction pages (also at felixcloutier.com/x86) for each
//! operation; *AMD64 Architecture Programmer's Manual* volume 4 for the
//! same instructions from the other vendor's side. No copyleft emulator was
//! consulted (`CLAUDE.md`, provenance).

use crate::float::{Env, Flags};

use super::exec::{Ex, Exec, Fault, VEC_NM, VEC_UD};
use super::fpexec::{
    approximate, arith, bitwise, compare, convert_dwords, is_arith, is_bitwise, is_shift,
    narrow_doubles, packed, shift_lanes, shuffle, widen_to_doubles,
};
use super::isa::{Arg, Extension, Fields, ModRm, Op, VexForm};
use super::prot::{cr0, cr4, xcr0};
use super::simd::{dot, duplicate, horizontal, pcmpgtq, round, sse41, ssse3};

/// A `YMM`-width value: `[bits 127:0, bits 255:128]`, each `[low, high]`.
type Ymm = [[u64; 2]; 2];

/// The four quadwords of a `YMM` value, low to high.
const fn flat(v: Ymm) -> [u64; 4] {
    [v[0][0], v[0][1], v[1][0], v[1][1]]
}

/// The converse of [`flat`].
const fn unflat(q: [u64; 4]) -> Ymm {
    [[q[0], q[1]], [q[2], q[3]]]
}

impl Exec<'_> {
    /// AVX's gate. See the module documentation.
    fn avx_gate(&self) -> Ex<()> {
        let sys = &self.state.sys;
        let enabled = xcr0::SSE | xcr0::AVX;
        if !self.cfg.features.avx
            || !self.protected()
            || self.v86()
            || sys.cr4 & cr4::OSXSAVE == 0
            || sys.xcr0 & enabled != enabled
        {
            return Err(Fault::bare(VEC_UD));
        }
        if sys.cr0 & cr0::TS != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        Ok(())
    }

    /// A whole `YMM` register.
    #[inline]
    fn ymm(&self, n: u8) -> Ymm {
        self.state.sse.ymm(n)
    }

    /// Write a `YMM` register: all 256 bits when `wide`, and otherwise the
    /// low 128 with the upper half **zeroed** — the rule for every
    /// VEX-encoded write.
    #[inline]
    fn set_ymm(&mut self, n: u8, v: Ymm, wide: bool) {
        self.state.sse.set(n, v[0]);
        self.state.sse.set_hi(n, if wide { v[1] } else { [0; 2] });
    }

    /// Write an `XMM` register VEX-style: the upper half zeroed.
    #[inline]
    fn set_xmm_vex(&mut self, n: u8, v: [u64; 2]) {
        self.set_ymm(n, [v, [0; 2]], false);
    }

    /// Read `bytes` of memory as a `YMM` value, zero-extended.
    fn read_vec_mem(&mut self, sr: u8, off: u64, bytes: u8) -> Ex<Ymm> {
        let mut q = [0u64; 4];
        if bytes < 8 {
            q[0] = self.read_mem(sr, off, bytes)?;
        } else {
            for (i, word) in q.iter_mut().enumerate().take(usize::from(bytes / 8)) {
                *word = self.read_mem(sr, off.wrapping_add(8 * i as u64), 8)?;
            }
        }
        Ok(unflat(q))
    }

    /// Write the low `bytes` of a `YMM` value to memory.
    fn write_vec_mem(&mut self, sr: u8, off: u64, bytes: u8, v: Ymm) -> Ex<()> {
        let q = flat(v);
        if bytes < 8 {
            return self.write_mem(sr, off, bytes, q[0]);
        }
        for (i, &word) in q.iter().enumerate().take(usize::from(bytes / 8)) {
            self.write_mem(sr, off.wrapping_add(8 * i as u64), 8, word)?;
        }
        Ok(())
    }

    /// The memory operand's address, refused with `#GP(0)` when `aligned`
    /// and it is not a multiple of `bytes`.
    fn vex_address(&self, bytes: u8, aligned: bool) -> Ex<(u8, u64)> {
        let (sr, off) = self.ea();
        if aligned && !off.is_multiple_of(u64::from(bytes)) {
            return Err(Fault::gp(0));
        }
        Ok((sr, off))
    }

    /// Read the r/m operand of kind `arg`, at the width this encoding gives
    /// it ([`Fields::mem_bytes`]). A register form is the whole `YMM`
    /// register for the full-length kinds and its `XMM` half for the rest.
    fn vex_read(&mut self, f: &Fields, arg: Arg, aligned: bool) -> Ex<Ymm> {
        if f.modrm.is_some_and(ModRm::is_register) || arg == Arg::Ux {
            let v = self.ymm(f.rm_num());
            let full = matches!(arg, Arg::Wx | Arg::Wqx | Arg::Ux) && f.vector_bytes() == 32;
            return Ok(if full { v } else { [v[0], [0; 2]] });
        }
        let bytes = f.mem_bytes(arg).unwrap_or(16);
        let (sr, off) = self.vex_address(bytes, aligned)?;
        self.read_vec_mem(sr, off, bytes)
    }

    /// The register `imm8[7:4]` names — the `Lx` operand. Three bits of it
    /// outside 64-bit mode (*Intel SDM* volume 2 §2.3.5, `VBLENDVPS`).
    fn is4(&self, f: &Fields) -> u8 {
        let n = (f.imm as u8) >> 4;
        if self.sixty_four() { n } else { n & 7 }
    }

    /// Execute one VEX-encoded instruction.
    pub(super) fn vex_instruction(&mut self, f: &Fields) -> Ex<()> {
        // An encoding the decoder refused is `#UD` before anything else is
        // asked: it is not an AVX instruction whose gate could matter.
        if f.insn.op == Op::UD {
            return Err(Fault::bare(VEC_UD));
        }
        self.avx_gate()?;
        let op = f.insn.op;
        let wide = f.vector_bytes() == 32;
        match op {
            Op::VZEROUPPER | Op::VZEROALL => {
                // Outside 64-bit mode only the first eight registers are
                // architectural, and only they are touched (*Intel SDM*
                // volume 2, `VZEROUPPER`, `VZEROALL`).
                let count = if self.sixty_four() { 16 } else { 8 };
                for n in 0..count {
                    if op == Op::VZEROALL {
                        self.state.sse.set(n, [0; 2]);
                    }
                    self.state.sse.set_hi(n, [0; 2]);
                }
                Ok(())
            }
            Op::MOVUPS
            | Op::MOVUPD
            | Op::MOVDQU
            | Op::MOVAPS
            | Op::MOVAPD
            | Op::MOVDQA
            | Op::LDDQU
            | Op::MOVNTPS
            | Op::MOVNTPD
            | Op::MOVNTDQ
            | Op::MOVNTDQA => self.vex_move(f, wide),
            Op::MOVSS | Op::MOVSD => self.vex_scalar_move(f),
            Op::MOVLPS | Op::MOVLPD | Op::MOVHPS | Op::MOVHPD | Op::MOVHLPS | Op::MOVLHPS => {
                self.vex_half_move(f)
            }
            Op::MOVMSKPS | Op::MOVMSKPD => {
                if !f.modrm.is_some_and(ModRm::is_register) {
                    return Err(Fault::bare(VEC_UD));
                }
                let q = flat(self.vex_read(f, Arg::Ux, false)?);
                let quads = if wide { 4 } else { 2 };
                let mut mask = 0u64;
                for (i, &word) in q.iter().enumerate().take(quads) {
                    if op == Op::MOVMSKPD {
                        mask |= (word >> 63) << i;
                    } else {
                        mask |= ((word >> 31) & 1) << (2 * i);
                        mask |= (word >> 63) << (2 * i + 1);
                    }
                }
                self.write_arg(f, Arg::Gy, if f.rex_w() { 8 } else { 4 }, mask)
            }
            Op::PTEST | Op::VTESTPS | Op::VTESTPD => {
                let src = flat(self.vex_read(f, Arg::Wx, false)?);
                let dst = flat(self.ymm(f.reg_num()));
                let quads = if wide { 4 } else { 2 };
                let bits = match op {
                    Op::VTESTPS => 0x8000_0000_8000_0000,
                    Op::VTESTPD => 1 << 63,
                    _ => u64::MAX,
                };
                self.set_test_flags(&dst[..quads], &src[..quads], bits);
                Ok(())
            }
            Op::CVTPS2PD | Op::CVTDQ2PD => {
                let src = self.vex_read(f, Arg::Wh, false)?;
                let lanes = if wide { 4 } else { 2 };
                let (out, flags) = widen_to_doubles(op, src[0], lanes, self.state.sse.env());
                self.sse_raise(flags)?;
                self.set_ymm(f.reg_num(), unflat(out), wide);
                Ok(())
            }
            Op::CVTPD2PS | Op::CVTPD2DQ | Op::CVTTPD2DQ => {
                let src = flat(self.vex_read(f, Arg::Wx, false)?);
                let lanes = if wide { 4 } else { 2 };
                let (out, flags) = narrow_doubles(op, src, lanes, self.state.sse.env());
                self.sse_raise(flags)?;
                self.set_xmm_vex(f.reg_num(), out);
                Ok(())
            }
            Op::VBROADCASTSS | Op::VBROADCASTSD | Op::VBROADCASTF128 => self.vex_broadcast(f, wide),
            Op::VINSERTF128 => {
                let mut v = self.ymm(f.vvvv());
                v[usize::from(f.imm as u8 & 1)] = self.vex_read(f, Arg::Wdq, false)?[0];
                self.set_ymm(f.reg_num(), v, true);
                Ok(())
            }
            Op::VEXTRACTF128 => {
                let half = self.ymm(f.reg_num())[usize::from(f.imm as u8 & 1)];
                if f.modrm.is_some_and(ModRm::is_register) {
                    self.set_xmm_vex(f.rm_num(), half);
                    Ok(())
                } else {
                    let (sr, off) = self.vex_address(16, false)?;
                    self.write_vec_mem(sr, off, 16, [half, [0; 2]])
                }
            }
            Op::VPERM2F128 => {
                // Each half of the result is one of the four source halves,
                // or zero, by a nibble of the immediate (*Intel SDM* volume
                // 2, `VPERM2F128`).
                let a = self.ymm(f.vvvv());
                let b = self.vex_read(f, Arg::Wx, false)?;
                let pick = [a[0], a[1], b[0], b[1]];
                let imm = f.imm as u8;
                let half = |ctl: u8| {
                    if ctl & 8 != 0 {
                        [0; 2]
                    } else {
                        pick[usize::from(ctl & 3)]
                    }
                };
                self.set_ymm(f.reg_num(), [half(imm), half(imm >> 4)], true);
                Ok(())
            }
            Op::VPERMILPS | Op::VPERMILPD => self.vex_permil(f, wide),
            Op::VMASKMOVPS | Op::VMASKMOVPD => self.vex_maskmov(f, wide),
            // The shift-by-immediate groups: the source is r/m, the count the
            // immediate, and the destination `VEX.vvvv`.
            _ if f.insn.vex.form == VexForm::Ndd => {
                let v = self.state.sse.get(f.rm_num());
                let out = shift_lanes(op, v, u64::from(f.imm as u8));
                self.set_xmm_vex(f.vvvv(), out);
                Ok(())
            }
            // `VEX.128` only, and the legacy instruction with `VEX.vvvv` as
            // its first source and the upper half cleared afterwards.
            Op::MOVD
            | Op::MOVQ
            | Op::UCOMISS
            | Op::COMISS
            | Op::UCOMISD
            | Op::COMISD
            | Op::CVTSI2SS
            | Op::CVTSI2SD
            | Op::CVTSS2SI
            | Op::CVTTSS2SI
            | Op::CVTSD2SI
            | Op::CVTTSD2SI
            | Op::CVTSS2SD
            | Op::CVTSD2SS
            | Op::PEXTRB
            | Op::PEXTRW
            | Op::PEXTRD
            | Op::EXTRACTPS
            | Op::PINSRB
            | Op::PINSRW
            | Op::PINSRD
            | Op::INSERTPS
            | Op::PMOVMSKB
            | Op::MASKMOVDQU
            | Op::PCMPESTRI
            | Op::PCMPESTRM
            | Op::PCMPISTRI
            | Op::PCMPISTRM
            | Op::LDMXCSR
            | Op::STMXCSR => self.vex_legacy(f),
            _ => self.vex_lanes(f, wide),
        }
    }

    /// Run a `VEX.128` instruction through the legacy executor, then clear
    /// the upper half of the `XMM` register it wrote, if it wrote one.
    fn vex_legacy(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        self.sse_dispatch(f)?;
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        let written = match op {
            // Flags, `ECX`, or memory: no vector register.
            Op::UCOMISS
            | Op::COMISS
            | Op::UCOMISD
            | Op::COMISD
            | Op::PCMPESTRI
            | Op::PCMPISTRI
            | Op::MASKMOVDQU => None,
            // The mask forms write `XMM0`, named by no field.
            Op::PCMPESTRM | Op::PCMPISTRM => Some(0),
            _ => match f.insn.dst {
                Arg::Vx | Arg::Vdq => Some(f.reg_num()),
                Arg::Wx | Arg::Wq | Arg::Wd if register_form => Some(f.rm_num()),
                _ => None,
            },
        };
        if let Some(n) = written {
            self.state.sse.set_hi(n, [0; 2]);
        }
        Ok(())
    }

    /// Every operation that works one 128-bit lane at a time, at either
    /// length: read both sources, apply the shared kernel per lane, raise
    /// what any lane raised, and only then write.
    fn vex_lanes(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let a = if f.vvvv_is_source() {
            self.ymm(f.vvvv())
        } else {
            self.ymm(reg)
        };
        let b = self.vex_read(f, f.insn.src, false)?;
        let mask = if f.insn.aux == Arg::Lx {
            self.ymm(self.is4(f))
        } else {
            [[0; 2]; 2]
        };
        let env = self.state.sse.env();
        let imm = f.imm as u8;
        let mut out = [[0u64; 2]; 2];
        let mut flags = Flags::NONE;
        for h in 0..if wide { 2 } else { 1 } {
            let Some((v, fl)) = lane_op(op, a[h], b[h], lane_imm(op, imm, h), mask[h], env) else {
                // Every row with a VEX form is matched above or here; a row
                // that is not is a table this file has not caught up with,
                // and `#UD` is what a part without it would do.
                return Err(Fault::bare(VEC_UD));
            };
            out[h] = v;
            flags |= fl;
        }
        self.sse_raise(flags)?;
        self.set_ymm(reg, out, wide);
        Ok(())
    }

    /// The full-register moves, loads and stores. Only the explicitly
    /// aligned and the non-temporal ones check alignment; the non-temporal
    /// ones and `VLDDQU` have no register form.
    fn vex_move(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let op = f.insn.op;
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        let aligned = matches!(
            op,
            Op::MOVAPS
                | Op::MOVAPD
                | Op::MOVDQA
                | Op::MOVNTPS
                | Op::MOVNTPD
                | Op::MOVNTDQ
                | Op::MOVNTDQA
        );
        let memory_only = matches!(
            op,
            Op::LDDQU | Op::MOVNTPS | Op::MOVNTPD | Op::MOVNTDQ | Op::MOVNTDQA
        );
        if memory_only && register_form {
            return Err(Fault::bare(VEC_UD));
        }
        if matches!(f.insn.dst, Arg::Vx) {
            let v = self.vex_read(f, Arg::Wx, aligned)?;
            self.set_ymm(f.reg_num(), v, wide);
            return Ok(());
        }
        let v = self.ymm(f.reg_num());
        if register_form {
            self.set_ymm(f.rm_num(), v, wide);
            return Ok(());
        }
        let bytes = f.vector_bytes();
        let (sr, off) = self.vex_address(bytes, aligned)?;
        self.write_vec_mem(sr, off, bytes, v)
    }

    /// `VMOVSS` and `VMOVSD`. The load zeroes everything above the element;
    /// the register form takes everything above it from `VEX.vvvv`; the
    /// store writes the element alone (*Intel SDM* volume 2, `MOVSS`).
    fn vex_scalar_move(&mut self, f: &Fields) -> Ex<()> {
        let bytes = if f.insn.op == Op::MOVSS { 4 } else { 8 };
        let mask = if bytes == 4 { 0xffff_ffff } else { u64::MAX };
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        let (to, from) = if matches!(f.insn.dst, Arg::Vx) {
            (f.reg_num(), f.rm_num())
        } else {
            (f.rm_num(), f.reg_num())
        };
        if register_form {
            let mut out = self.state.sse.get(f.vvvv());
            out[0] = (out[0] & !mask) | (self.state.sse.low(from) & mask);
            self.set_xmm_vex(to, out);
            return Ok(());
        }
        let (sr, off) = self.ea();
        if matches!(f.insn.dst, Arg::Vx) {
            let v = self.read_mem(sr, off, bytes)?;
            self.set_xmm_vex(to, [v, 0]);
            Ok(())
        } else {
            let v = self.state.sse.low(from);
            self.write_mem(sr, off, bytes, v & mask)
        }
    }

    /// The half-register moves: `VMOVLPS`, `VMOVHPS`, their `PD` twins, and
    /// the register forms `VMOVHLPS` and `VMOVLHPS`. A load or a register
    /// form replaces one quadword of `VEX.vvvv` and writes the result; a
    /// store writes the quadword alone.
    fn vex_half_move(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let high = matches!(op, Op::MOVHPS | Op::MOVHPD | Op::MOVLHPS);
        let reg = f.reg_num();
        if f.insn.dst == Arg::Mq {
            let v = self.state.sse.get(reg)[usize::from(high)];
            let (sr, off) = self.ea();
            return self.write_mem(sr, off, 8, v);
        }
        let value = match op {
            Op::MOVHLPS => self.state.sse.high(f.rm_num()),
            Op::MOVLHPS => self.state.sse.low(f.rm_num()),
            _ => {
                let (sr, off) = self.ea();
                self.read_mem(sr, off, 8)?
            }
        };
        let mut out = self.state.sse.get(f.vvvv());
        out[usize::from(high)] = value;
        self.set_xmm_vex(reg, out);
        Ok(())
    }

    /// The broadcasts: one element or one 128-bit half of memory into every
    /// lane. Memory sources only — the register forms are AVX2's.
    fn vex_broadcast(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        if f.modrm.is_some_and(ModRm::is_register) {
            return Err(Fault::bare(VEC_UD));
        }
        let src = self.vex_read(f, f.insn.src, false)?;
        let out = match f.insn.op {
            Op::VBROADCASTSS => {
                let d = src[0][0] & 0xffff_ffff;
                let q = d | (d << 32);
                [[q; 2]; 2]
            }
            Op::VBROADCASTSD => [[src[0][0]; 2]; 2],
            _ => [src[0], src[0]],
        };
        self.set_ymm(f.reg_num(), out, wide);
        Ok(())
    }

    /// `VPERMILPS` and `VPERMILPD`, by an immediate or by a control vector:
    /// each element of each 128-bit lane picks an element of the same lane.
    ///
    /// The immediate form's data is r/m and its control `imm8` — two bits per
    /// single, the same for both lanes; one bit per double, the next pair for
    /// the upper lane. The vector form's data is `VEX.vvvv` and its control
    /// r/m — bits 1:0 of each doubleword, or bit 1 of each quadword
    /// (*Intel SDM* volume 2, `VPERMILPS`, `VPERMILPD`).
    fn vex_permil(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let singles = f.insn.op == Op::VPERMILPS;
        let by_immediate = f.insn.aux == Arg::Ib;
        let imm = f.imm as u8;
        let (data, control) = if by_immediate {
            (self.vex_read(f, Arg::Wx, false)?, [[0; 2]; 2])
        } else {
            let data = self.ymm(f.vvvv());
            (data, self.vex_read(f, Arg::Wx, false)?)
        };
        let mut out = [[0u64; 2]; 2];
        for h in 0..if wide { 2 } else { 1 } {
            let lane = data[h];
            if singles {
                let dword = |v: [u64; 2], i: usize| (v[i / 2] >> (32 * (i % 2))) & 0xffff_ffff;
                for i in 0..4 {
                    let sel = if by_immediate {
                        usize::from((imm >> (2 * i)) & 3)
                    } else {
                        (dword(control[h], i) & 3) as usize
                    };
                    out[h][i / 2] |= dword(lane, sel) << (32 * (i % 2));
                }
            } else {
                for i in 0..2 {
                    let sel = if by_immediate {
                        usize::from((imm >> (2 * h + i)) & 1)
                    } else {
                        ((control[h][i] >> 1) & 1) as usize
                    };
                    out[h][i] = lane[sel];
                }
            }
        }
        self.set_ymm(f.reg_num(), out, wide);
        Ok(())
    }

    /// `VMASKMOVPS` and `VMASKMOVPD`: a load or a store of the elements whose
    /// mask element in `VEX.vvvv` has its sign bit set.
    ///
    /// An element whose mask bit is clear is **not accessed at all**: a load
    /// reads zero for it and a store leaves its bytes alone, and neither can
    /// fault on it — the architecture's guarantee, which is what makes the
    /// instruction usable at the edge of a mapping (*Intel SDM* volume 2,
    /// `VMASKMOV`: "Faults occur only due to mask-bit required memory
    /// accesses"). So the elements are transferred one at a time. Memory
    /// operands only, and no alignment requirement.
    fn vex_maskmov(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        if f.modrm.is_some_and(ModRm::is_register) {
            return Err(Fault::bare(VEC_UD));
        }
        let size: u8 = if f.insn.op == Op::VMASKMOVPS { 4 } else { 8 };
        let count = usize::from(f.vector_bytes() / size);
        let mask = flat(self.ymm(f.vvvv()));
        let per = usize::from(8 / size);
        let element = |q: &[u64; 4], i: usize| {
            let word = q[i / per];
            if size == 4 {
                (word >> (32 * (i % 2))) & 0xffff_ffff
            } else {
                word
            }
        };
        let selected = |i: usize| element(&mask, i) >> (8 * u32::from(size) - 1) != 0;
        let (sr, off) = self.ea();
        if matches!(f.insn.dst, Arg::Vx) {
            let mut out = [0u64; 4];
            for i in 0..count {
                if selected(i) {
                    let at = off.wrapping_add((i * usize::from(size)) as u64);
                    let v = self.read_mem(sr, at, size)?;
                    out[i / per] |= v << (32 * (i % per));
                }
            }
            self.set_ymm(f.reg_num(), unflat(out), wide);
            Ok(())
        } else {
            let data = flat(self.ymm(f.reg_num()));
            for i in 0..count {
                if selected(i) {
                    let at = off.wrapping_add((i * usize::from(size)) as u64);
                    self.write_mem(sr, at, size, element(&data, i))?;
                }
            }
            Ok(())
        }
    }
}

/// The immediate one 128-bit lane sees.
///
/// The same byte for most operations; the next bits of it for the three
/// whose 256-bit forms spend one immediate bit per element across the whole
/// register — `VSHUFPD` and `VBLENDPD` two bits per lane, `VBLENDPS` four
/// (*Intel SDM* volume 2, each instruction's `VEX.256` "Operation").
fn lane_imm(op: Op, imm: u8, lane: usize) -> u8 {
    match op {
        Op::SHUFPD | Op::BLENDPD => imm >> (2 * lane),
        Op::BLENDPS => imm >> (4 * lane),
        _ => imm,
    }
}

/// Every operation that works one 128-bit lane at a time, on one lane: `a`
/// the first source, `b` the second, `imm` that lane's immediate and `mask`
/// that lane of the blend mask. `None` for an operation that is not one.
///
/// The kernels are the legacy executors' own, so a `VEX.128` encoding and
/// its legacy twin compute the same bits by construction.
pub(super) fn lane_op(
    op: Op,
    a: [u64; 2],
    b: [u64; 2],
    imm: u8,
    mask: [u64; 2],
    env: Env,
) -> Option<([u64; 2], Flags)> {
    let exact = |v: [u64; 2]| Some((v, Flags::NONE));
    if is_arith(op) {
        return Some(arith(op, a, b, env));
    }
    if is_bitwise(op) {
        return exact(bitwise(op, a, b));
    }
    match op {
        // The full five-bit predicate: VEX reaches all thirty-two.
        Op::CMPPS | Op::CMPPD | Op::CMPSS | Op::CMPSD => Some(compare(op, a, b, imm & 31, env)),
        Op::UNPCKLPS | Op::UNPCKHPS | Op::UNPCKLPD | Op::UNPCKHPD | Op::SHUFPS | Op::SHUFPD => {
            exact(shuffle(op, a, b, imm))
        }
        Op::ADDSUBPS | Op::HADDPS | Op::HSUBPS | Op::ADDSUBPD | Op::HADDPD | Op::HSUBPD => {
            Some(horizontal(op, a, b, env))
        }
        Op::ROUNDPS | Op::ROUNDPD | Op::ROUNDSS | Op::ROUNDSD => Some(round(op, a, b, imm, env)),
        Op::DPPS | Op::DPPD => Some(dot(op, a, b, imm, env)),
        Op::RCPPS | Op::RCPSS | Op::RSQRTPS | Op::RSQRTSS => exact(approximate(op, a, b)),
        Op::CVTDQ2PS | Op::CVTPS2DQ | Op::CVTTPS2DQ => Some(convert_dwords(op, b, env)),
        Op::MOVDDUP | Op::MOVSLDUP | Op::MOVSHDUP => exact(duplicate(op, b)),
        Op::PCMPGTQ => exact(pcmpgtq(a, b)),
        _ if is_shift(op) => exact(shift_lanes(op, a, b[0])),
        _ => match op.extension() {
            Some(Extension::Ssse3) => exact(ssse3(op, a, b, 16, imm)),
            Some(Extension::Sse41) => exact(sse41(op, a, b, mask, imm)),
            _ if op.is_sse2_packed_int() => exact(packed(op, a, b, imm)),
            _ => None,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::x86::isa::{self, Grp, Insn};

    /// Every row with a VEX form, from every table that has one — the
    /// group members rather than the group's primary row, which only says
    /// which table to look in.
    fn vex_rows() -> impl Iterator<Item = Insn> {
        let tables: [&[Insn]; 10] = [
            &isa::TABLE_0F,
            &isa::TABLE_0F_66,
            &isa::TABLE_0F_F3,
            &isa::TABLE_0F_F2,
            &isa::TABLE_0F38_66,
            &isa::TABLE_0F3A_66,
            &isa::GROUP12,
            &isa::GROUP13,
            &isa::GROUP14,
            &isa::GROUP15,
        ];
        tables
            .into_iter()
            .flat_map(|t| t.iter().copied())
            .filter(|i| i.vex.exists() && !matches!(i.vex.form, VexForm::Alt(_)))
            .filter(|i| matches!(i.group, Grp::None | Grp::ModAlt))
            // BMI2's `SHLX` shares the `66 0F38` column and is `bmi`'s.
            .filter(|i| !i.op.is_bmi())
    }

    #[test]
    fn every_vex_row_reaches_an_executor() {
        // `Exec::vex_instruction` dispatches the rows below by name and sends
        // everything else to `lane_op`; a row that is neither would execute
        // as `#UD` on a part that has it. This is the list the dispatcher's
        // named arms cover, and the test fails for a row in neither.
        let named = [
            Op::MOVUPS,
            Op::MOVUPD,
            Op::MOVDQU,
            Op::MOVAPS,
            Op::MOVAPD,
            Op::MOVDQA,
            Op::LDDQU,
            Op::MOVNTPS,
            Op::MOVNTPD,
            Op::MOVNTDQ,
            Op::MOVNTDQA,
            Op::MOVSS,
            Op::MOVSD,
            Op::MOVLPS,
            Op::MOVLPD,
            Op::MOVHPS,
            Op::MOVHPD,
            Op::MOVMSKPS,
            Op::MOVMSKPD,
            Op::PTEST,
            Op::VTESTPS,
            Op::VTESTPD,
            Op::CVTPS2PD,
            Op::CVTDQ2PD,
            Op::CVTPD2PS,
            Op::CVTPD2DQ,
            Op::CVTTPD2DQ,
            Op::VBROADCASTSS,
            Op::VBROADCASTSD,
            Op::VBROADCASTF128,
            Op::VINSERTF128,
            Op::VEXTRACTF128,
            Op::VPERM2F128,
            Op::VPERMILPS,
            Op::VPERMILPD,
            Op::VMASKMOVPS,
            Op::VMASKMOVPD,
            Op::MOVD,
            Op::MOVQ,
            Op::UCOMISS,
            Op::COMISS,
            Op::UCOMISD,
            Op::COMISD,
            Op::CVTSI2SS,
            Op::CVTSI2SD,
            Op::CVTSS2SI,
            Op::CVTTSS2SI,
            Op::CVTSD2SI,
            Op::CVTTSD2SI,
            Op::CVTSS2SD,
            Op::CVTSD2SS,
            Op::PEXTRB,
            Op::PEXTRW,
            Op::PEXTRD,
            Op::EXTRACTPS,
            Op::PINSRB,
            Op::PINSRW,
            Op::PINSRD,
            Op::INSERTPS,
            Op::PMOVMSKB,
            Op::MASKMOVDQU,
            Op::PCMPESTRI,
            Op::PCMPESTRM,
            Op::PCMPISTRI,
            Op::PCMPISTRM,
            Op::LDMXCSR,
            Op::STMXCSR,
        ];
        let zero = [0u64; 2];
        let mut lane_wise = 0;
        for row in vex_rows() {
            if named.contains(&row.op) || row.vex.form == VexForm::Ndd {
                continue;
            }
            assert!(
                lane_op(row.op, zero, zero, 0, zero, Env::X86_SSE).is_some(),
                "{:?} has a VEX form and no executor",
                row.op
            );
            lane_wise += 1;
        }
        assert!(
            lane_wise > 150,
            "the lane-wise path is most of AVX: {lane_wise}"
        );
    }

    #[test]
    fn the_lane_immediate_steps_through_the_byte_only_where_it_must() {
        assert_eq!(lane_imm(Op::SHUFPD, 0b1110, 1), 0b11);
        assert_eq!(lane_imm(Op::BLENDPD, 0b1110, 1), 0b11);
        assert_eq!(lane_imm(Op::BLENDPS, 0xa5, 1), 0x0a);
        assert_eq!(lane_imm(Op::SHUFPS, 0xa5, 1), 0xa5);
        assert_eq!(lane_imm(Op::CMPPS, 0x1f, 1), 0x1f);
    }

    #[test]
    fn a_vex_128_lane_is_the_legacy_kernel() {
        // The same kernel answers both encodings: `lane_op` on one lane is
        // the legacy arithmetic, bit for bit.
        let env = Env::X86_SSE;
        let a = [0x4000_0000_3f80_0000, 0x4080_0000_4040_0000];
        let b = [0x3f80_0000_3f80_0000, 0x3f80_0000_3f80_0000];
        assert_eq!(
            lane_op(Op::ADDPS, a, b, 0, [0; 2], env),
            Some(arith(Op::ADDPS, a, b, env))
        );
        // Except that a VEX compare reads five bits of predicate where the
        // legacy one reads three: 13 is GE_OS, where 5 would be NLT_US.
        // 1 >= 1 and 1 >= 2 in the low quadword; 1 >= 3, 1 >= 4 above.
        let (out, _) = lane_op(Op::CMPPS, b, a, 13, [0; 2], env).unwrap();
        assert_eq!(out, [0xffff_ffff, 0]);
    }
}
