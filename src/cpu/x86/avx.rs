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
//! (`VCVTPS2PD ymm, xmm/m128` and its converses, F16C's two), the
//! broadcasts, the 128-bit inserts, extracts and permutes, AVX2's
//! `VPERMD`/`VPERMQ` family, the masked moves, the gathers, `VPMOVMSKB`,
//! and the flag-setting tests. AVX2's 256-bit integer forms and the FMA
//! family are lane-wise and go through [`lane_op`]; the few lane-wise
//! operations whose upper lane needs a different second source — a shift
//! count, a widening move's next elements — get it from [`Exec::vex_lanes`].
//!
//! The AVX2, FMA and F16C instructions are gated on their own bits before
//! AVX's gate is asked, and AVX2 twice: by the instructions it added and by
//! the `VEX.256` forms of the integer ones (`VexLen::Avx2`).
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapter 14 ("Programming with Intel AVX"), volume 2
//! §2.3 (the VEX prefix) and §2.4-§2.8 (the exception classes), and volume
//! 2's instruction pages (also at felixcloutier.com/x86) for each
//! operation; *AMD64 Architecture Programmer's Manual* volume 4 for the
//! same instructions from the other vendor's side. No copyleft emulator was
//! consulted (`CLAUDE.md`, provenance).

use crate::float::{B16, B32, Env, Flags, Round, binary};

use super::exec::{Ex, Exec, Fault, VEC_NM, VEC_UD};
use super::fpexec::{
    approximate, arith, bitwise, compare, convert_dwords, fused, is_arith, is_bitwise, is_shift,
    narrow_doubles, packed, shift_lanes, shuffle, widen_to_doubles,
};
use super::isa::{Arg, Extension, Fields, ModRm, Op, VexForm, VexLen};
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
        let wide = f.vector_bytes() == 32;
        // The extensions after AVX: an instruction a part does not have is
        // `#UD` whatever the rest of the gate says — `CR0.TS` included,
        // because there is no instruction for `#NM` to defer. AVX2 is
        // asked twice: by the instructions it added (the row's extension)
        // and by the `VEX.256` forms of the integer ones (the row's length).
        if !self.has_extension(f.insn)
            || (wide && f.insn.vex.len == VexLen::Avx2 && !self.cfg.features.avx2)
        {
            return Err(Fault::bare(VEC_UD));
        }
        self.avx_gate()?;
        let op = f.insn.op;
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
            Op::VBROADCASTSS
            | Op::VBROADCASTSD
            | Op::VBROADCASTF128
            | Op::VBROADCASTI128
            | Op::VPBROADCASTB
            | Op::VPBROADCASTW
            | Op::VPBROADCASTD
            | Op::VPBROADCASTQ => self.vex_broadcast(f, wide),
            Op::VINSERTF128 | Op::VINSERTI128 => {
                let mut v = self.ymm(f.vvvv());
                v[usize::from(f.imm as u8 & 1)] = self.vex_read(f, Arg::Wdq, false)?[0];
                self.set_ymm(f.reg_num(), v, true);
                Ok(())
            }
            Op::VEXTRACTF128 | Op::VEXTRACTI128 => {
                let half = self.ymm(f.reg_num())[usize::from(f.imm as u8 & 1)];
                if f.modrm.is_some_and(ModRm::is_register) {
                    self.set_xmm_vex(f.rm_num(), half);
                    Ok(())
                } else {
                    let (sr, off) = self.vex_address(16, false)?;
                    self.write_vec_mem(sr, off, 16, [half, [0; 2]])
                }
            }
            Op::VPERM2F128 | Op::VPERM2I128 => {
                // Each half of the result is one of the four source halves,
                // or zero, by a nibble of the immediate (*Intel SDM* volume
                // 2, `VPERM2F128`, `VPERM2I128`).
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
            Op::VPERMD | Op::VPERMPS => {
                // Across the whole register: each doubleword of the result
                // is the data doubleword its index's low three bits name.
                // The indices are `VEX.vvvv` and the data r/m (*Intel SDM*
                // volume 2, `VPERMD`, `VPERMPS`).
                let index = flat(self.ymm(f.vvvv()));
                let data = flat(self.vex_read(f, Arg::Wx, false)?);
                let mut out = [0u64; 4];
                for i in 0..8 {
                    let sel = (dword_of(&index, i) & 7) as usize;
                    out[i / 2] |= dword_of(&data, sel) << (32 * (i % 2));
                }
                self.set_ymm(f.reg_num(), unflat(out), true);
                Ok(())
            }
            Op::VPERMQ | Op::VPERMPD => {
                // Each quadword of the result is the source quadword two
                // bits of the immediate name (*Intel SDM* volume 2,
                // `VPERMQ`, `VPERMPD`).
                let data = flat(self.vex_read(f, Arg::Wx, false)?);
                let imm = f.imm as u8;
                let out = core::array::from_fn(|i| data[usize::from((imm >> (2 * i)) & 3)]);
                self.set_ymm(f.reg_num(), unflat(out), true);
                Ok(())
            }
            Op::VMASKMOVPS | Op::VMASKMOVPD | Op::VPMASKMOVD | Op::VPMASKMOVQ => {
                self.vex_maskmov(f, wide)
            }
            Op::VPGATHERDD
            | Op::VPGATHERDQ
            | Op::VPGATHERQD
            | Op::VPGATHERQQ
            | Op::VGATHERDPS
            | Op::VGATHERDPD
            | Op::VGATHERQPS
            | Op::VGATHERQPD => self.vex_gather(f, wide),
            Op::PMOVMSKB => {
                // The sign of every byte: sixteen bits, or thirty-two at
                // `VEX.256` (*Intel SDM* volume 2, `PMOVMSKB`), zero-extended
                // into the general register.
                if !f.modrm.is_some_and(ModRm::is_register) {
                    return Err(Fault::bare(VEC_UD));
                }
                let q = flat(self.ymm(f.rm_num()));
                let mut mask = 0u64;
                for (i, &word) in q.iter().enumerate().take(if wide { 4 } else { 2 }) {
                    for byte in 0..8 {
                        mask |= ((word >> (8 * byte + 7)) & 1) << (8 * i + byte);
                    }
                }
                self.write_arg(f, Arg::Gy, if f.rex_w() { 8 } else { 4 }, mask)
            }
            Op::VCVTPH2PS => self.vex_half_to_single(f, wide),
            Op::VCVTPS2PH => self.vex_single_to_half(f, wide),
            // The shift-by-immediate groups: the source is r/m, the count the
            // immediate, and the destination `VEX.vvvv`. Each 128-bit lane
            // shifts on its own, `VPSLLDQ`/`VPSRLDQ` included (*Intel SDM*
            // volume 2, `PSLLDQ`: "VEX.256 … shifts each 128-bit lane").
            _ if f.insn.vex.form == VexForm::Ndd => {
                let v = self.ymm(f.rm_num());
                let count = u64::from(f.imm as u8);
                let high = if wide {
                    shift_lanes(op, v[1], count)
                } else {
                    [0; 2]
                };
                self.set_ymm(f.vvvv(), [shift_lanes(op, v[0], count), high], wide);
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
    ///
    /// Three operations need their second source arranged first. A shift by
    /// a register count takes the count from the low quadword of an
    /// `XMM` register or sixteen bytes of memory at either length, and
    /// applies it to both lanes (*Intel SDM* volume 2, `PSLLW`: "VEX.256 …
    /// xmm3/m128"). The widening moves cross lanes at 256 bits — the upper
    /// lane widens the next elements of the source — so that lane is handed
    /// the source moved down by what the lower lane consumed. And an FMA
    /// reads its destination as a third source, passed where the is4
    /// blends pass their mask.
    fn vex_lanes(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let a = if f.vvvv_is_source() {
            self.ymm(f.vvvv())
        } else {
            self.ymm(reg)
        };
        let shift = is_shift(op);
        let mut b = self.vex_read(f, if shift { Arg::Wdq } else { f.insn.src }, false)?;
        if shift {
            b[1] = b[0];
        } else if let Some(consumed) = widening_step(op) {
            let whole = u128::from(b[0][0]) | (u128::from(b[0][1]) << 64);
            let rest = whole >> (8 * consumed);
            b[1] = [rest as u64, (rest >> 64) as u64];
        }
        let mask = if f.insn.aux == Arg::Lx {
            self.ymm(self.is4(f))
        } else if f.insn.extension() == Some(Extension::Fma) {
            self.ymm(reg)
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

    /// The broadcasts: one element, or one 128-bit half of memory, into
    /// every lane.
    ///
    /// AVX's three take memory only; AVX2 adds the register forms of
    /// `VBROADCASTSS`/`SD` and the integer broadcasts, whose register form
    /// is the low element of an `XMM` register. The 128-bit ones stay
    /// memory-only (*Intel SDM* volume 2, `VBROADCAST`, `VPBROADCAST`).
    fn vex_broadcast(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let op = f.insn.op;
        if f.modrm.is_some_and(ModRm::is_register)
            && (matches!(op, Op::VBROADCASTF128 | Op::VBROADCASTI128) || !self.cfg.features.avx2)
        {
            return Err(Fault::bare(VEC_UD));
        }
        let src = self.vex_read(f, f.insn.src, false)?;
        let e = src[0][0];
        let q = match op {
            Op::VPBROADCASTB => (e & 0xff) * 0x0101_0101_0101_0101,
            Op::VPBROADCASTW => (e & 0xffff) * 0x0001_0001_0001_0001,
            Op::VBROADCASTSS | Op::VPBROADCASTD => (e & 0xffff_ffff) * 0x0000_0001_0000_0001,
            Op::VBROADCASTSD | Op::VPBROADCASTQ => e,
            _ => {
                self.set_ymm(f.reg_num(), [src[0], src[0]], wide);
                return Ok(());
            }
        };
        let out = [[q; 2]; 2];
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

    /// `VMASKMOVPS`, `VMASKMOVPD` and AVX2's `VPMASKMOVD`/`Q`: a load or a
    /// store of the elements whose mask element in `VEX.vvvv` has its sign
    /// bit set.
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
        let size: u8 = if matches!(f.insn.op, Op::VMASKMOVPS | Op::VPMASKMOVD) {
            4
        } else {
            8
        };
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

impl Exec<'_> {
    /// The gathers: one load per element whose mask element's sign is set,
    /// from an address built from a general base, the element's index in a
    /// vector register, the scale and the displacement — a VSIB operand
    /// (*Intel SDM* volume 2 §2.3.12, and `VPGATHERDD`, `VGATHERDPS` and
    /// their siblings).
    ///
    /// The mask is the progress record, which is what makes a fault
    /// restartable: first every mask element becomes all ones or all zeros
    /// by its sign, and the mask is cleared above the elements the
    /// instruction uses; then elements are fetched lowest first, and each
    /// one's mask element is cleared as it completes, whether it was fetched
    /// or skipped. A fault leaves the destination with the elements already
    /// fetched and the mask with them cleared, so the handler's return
    /// re-executes only what is left. Faults must be delivered lowest element
    /// first, and the elements above a fault may or may not have completed;
    /// here none have. On completion the mask is zero and the destination is
    /// cleared above its data.
    ///
    /// `#UD` for a register operand, a ModRM without a SIB byte, a 16-bit
    /// address size, or any two of the destination, the index and the mask
    /// being one register. No alignment, and no SIMD floating-point
    /// exception: a floating-point gather moves bits.
    fn vex_gather(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let op = f.insn.op;
        let Some(m) = f.modrm else {
            return Err(Fault::bare(VEC_UD));
        };
        let Some(sib) = f.sib else {
            return Err(Fault::bare(VEC_UD));
        };
        if m.is_register() || m.rm != 4 || f.addrsize == 2 {
            return Err(Fault::bare(VEC_UD));
        }
        let (dest, mask_reg, index_reg) = (f.reg_num(), f.vvvv(), f.index_num());
        if dest == mask_reg || dest == index_reg || mask_reg == index_reg {
            return Err(Fault::bare(VEC_UD));
        }
        let (size, index_size): (u8, u8) = match op {
            Op::VPGATHERDD | Op::VGATHERDPS => (4, 4),
            Op::VPGATHERDQ | Op::VGATHERDPD => (8, 4),
            Op::VPGATHERQD | Op::VGATHERQPS => (4, 8),
            _ => (8, 8),
        };
        let count = usize::from(if wide { 32 } else { 16 } / size.max(index_size));
        let index = flat(self.ymm(index_reg));
        let mut mask = flat(self.ymm(mask_reg));
        let mut data = flat(self.ymm(dest));
        for i in 0..usize::from(32 / size) {
            let on = i < count && element(&mask, i, size) >> (8 * u32::from(size) - 1) != 0;
            set_element(&mut mask, i, size, if on { u64::MAX } else { 0 });
        }
        self.set_ymm(mask_reg, unflat(mask), true);

        // The common part of every address: base and displacement, summed
        // at the address size as any effective address is.
        let regs = &self.state.regs;
        let base_present = !(sib.base == 5 && m.md == 0);
        let sr = f.mem_segment();
        let base = if f.addrsize == 4 {
            let b = if base_present {
                regs.qword(f.base_num()) as u32
            } else {
                0
            };
            u64::from(b.wrapping_add(f.disp as u32))
        } else {
            let b = if base_present {
                regs.qword(f.base_num())
            } else {
                0
            };
            b.wrapping_add(f.disp as i64 as u64)
        };
        for i in 0..count {
            if element(&mask, i, size) != 0 {
                let raw = element(&index, i, index_size);
                let idx = if index_size == 4 {
                    i64::from(raw as u32 as i32) as u64
                } else {
                    raw
                };
                let offset = base.wrapping_add(idx << sib.scale);
                let at = if f.addrsize == 4 {
                    offset & 0xffff_ffff
                } else {
                    offset
                };
                match self.read_mem(sr, at, size) {
                    Ok(v) => set_element(&mut data, i, size, v),
                    Err(fault) => {
                        self.set_ymm(dest, unflat(data), true);
                        self.set_ymm(mask_reg, unflat(mask), true);
                        return Err(fault);
                    }
                }
            }
            set_element(&mut mask, i, size, 0);
        }
        for i in count..usize::from(32 / size) {
            set_element(&mut data, i, size, 0);
        }
        self.set_ymm(dest, unflat(data), true);
        self.set_ymm(mask_reg, [[0; 2]; 2], true);
        Ok(())
    }

    /// `VCVTPH2PS`: four or eight half-precision values, from the low half
    /// of an `XMM` register or eight or sixteen bytes of memory, as singles.
    ///
    /// Every half is exact as a single, so the only exception is invalid,
    /// for a signaling NaN; `MXCSR.DAZ` is ignored, a denormal half becomes
    /// the normal single it equals, and no denormal-operand exception is
    /// reported (*Intel SDM* volume 2, `VCVTPH2PS`).
    fn vex_half_to_single(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let src = self.vex_read(f, Arg::Wh, false)?[0];
        let env = self.state.sse.env().daz(false);
        let mut out = [0u64; 4];
        let mut flags = Flags::NONE;
        for i in 0..if wide { 8 } else { 4 } {
            let half = (src[i / 4] >> (16 * (i % 4))) & 0xffff;
            let (v, fl) = binary::convert::<B16, B32>(half, env);
            flags |= Flags(fl.0 & !Flags::DENORMAL.0);
            out[i / 2] |= v << (32 * (i % 2));
        }
        self.sse_raise(flags)?;
        self.set_ymm(f.reg_num(), unflat(out), wide);
        Ok(())
    }

    /// `VCVTPS2PH`: four or eight singles as half-precision values, into
    /// the low half of an `XMM` register (the rest zeroed) or eight or
    /// sixteen bytes of memory.
    ///
    /// `imm8[2]` set rounds by `MXCSR.RC`, clear by `imm8[1:0]`, which is
    /// encoded as `RC` is. `MXCSR.FTZ` is ignored — a tiny result is a
    /// denormal half — while `DAZ` applies to the inputs; overflow,
    /// underflow, inexact, invalid and denormal are reported, and an unmasked
    /// one writes nothing (*Intel SDM* volume 2, `VCVTPS2PH`).
    fn vex_single_to_half(&mut self, f: &Fields, wide: bool) -> Ex<()> {
        let src = flat(self.ymm(f.reg_num()));
        let imm = f.imm as u8;
        let mut env = self.state.sse.env().ftz(false);
        if imm & 4 == 0 {
            env = env.round(Round::from_x86_rc(u32::from(imm & 3)));
        }
        let count = if wide { 8 } else { 4 };
        let mut out = [0u64; 2];
        let mut flags = Flags::NONE;
        for i in 0..count {
            let (v, fl) = binary::convert::<B32, B16>(dword_of(&src, i), env);
            flags |= fl;
            out[i / 4] |= v << (16 * (i % 4));
        }
        self.sse_raise(flags)?;
        if f.modrm.is_some_and(ModRm::is_register) {
            self.set_xmm_vex(f.rm_num(), out);
            Ok(())
        } else {
            let bytes = 2 * count as u8;
            let (sr, off) = self.vex_address(bytes, false)?;
            self.write_vec_mem(sr, off, bytes, [out, [0; 2]])
        }
    }
}

/// Doubleword `i` of a flattened `YMM` value.
fn dword_of(q: &[u64; 4], i: usize) -> u64 {
    (q[i / 2] >> (32 * (i % 2))) & 0xffff_ffff
}

/// Element `i`, of `size` bytes (4 or 8), of a flattened `YMM` value.
fn element(q: &[u64; 4], i: usize, size: u8) -> u64 {
    if size == 4 { dword_of(q, i) } else { q[i] }
}

/// Replace element `i`, of `size` bytes (4 or 8).
fn set_element(q: &mut [u64; 4], i: usize, size: u8, v: u64) {
    if size == 4 {
        let shift = 32 * (i % 2);
        q[i / 2] = (q[i / 2] & !(0xffff_ffff << shift)) | ((v & 0xffff_ffff) << shift);
    } else {
        q[i] = v;
    }
}

/// How many bytes of its source a widening move consumes per 128-bit lane
/// of result — where the upper lane's elements begin. `None` for anything
/// else.
fn widening_step(op: Op) -> Option<u32> {
    match op {
        Op::PMOVSXBW | Op::PMOVSXWD | Op::PMOVSXDQ | Op::PMOVZXBW | Op::PMOVZXWD | Op::PMOVZXDQ => {
            Some(8)
        }
        Op::PMOVSXBD | Op::PMOVSXWQ | Op::PMOVZXBD | Op::PMOVZXWQ => Some(4),
        Op::PMOVSXBQ | Op::PMOVZXBQ => Some(2),
        _ => None,
    }
}

/// AVX2's shifts by a vector of counts on one 128-bit lane: each element of
/// `a` shifted by the matching element of `b`, the whole element taken as
/// the count. A count of the element width or more gives zero, or for
/// `VPSRAVD` the sign in every bit (*Intel SDM* volume 2, `VPSLLVD`,
/// `VPSRLVD`, `VPSRAVD`).
fn variable_shift(op: Op, a: [u64; 2], b: [u64; 2]) -> Option<[u64; 2]> {
    let mut out = [0u64; 2];
    match op {
        Op::VPSLLVQ | Op::VPSRLVQ => {
            for i in 0..2 {
                out[i] = match (op, b[i]) {
                    (_, 64..) => 0,
                    (Op::VPSLLVQ, n) => a[i] << n,
                    (_, n) => a[i] >> n,
                };
            }
        }
        Op::VPSLLVD | Op::VPSRLVD | Op::VPSRAVD => {
            let q = [a[0], a[1], 0, 0];
            let c = [b[0], b[1], 0, 0];
            for i in 0..4 {
                let v = dword_of(&q, i) as u32;
                let n = dword_of(&c, i);
                let r = match op {
                    Op::VPSRAVD => ((v as i32) >> n.min(31)) as u32,
                    _ if n >= 32 => 0,
                    Op::VPSLLVD => v << n,
                    _ => v >> n,
                };
                out[i / 2] |= u64::from(r) << (32 * (i % 2));
            }
        }
        _ => return None,
    }
    Some(out)
}

/// The immediate one 128-bit lane sees.
///
/// The same byte for most operations; the next bits of it for the three
/// whose 256-bit forms spend one immediate bit per element across the whole
/// register — `VSHUFPD` and `VBLENDPD` two bits per lane, `VBLENDPS` and
/// `VPBLENDD` four — and for `VMPSADBW`, whose upper lane takes its block
/// selectors from bits 5:3 rather than 2:0 (*Intel SDM* volume 2, each
/// instruction's `VEX.256` "Operation").
fn lane_imm(op: Op, imm: u8, lane: usize) -> u8 {
    match op {
        Op::SHUFPD | Op::BLENDPD => imm >> (2 * lane),
        Op::BLENDPS | Op::VPBLENDD => imm >> (4 * lane),
        Op::MPSADBW => imm >> (3 * lane),
        _ => imm,
    }
}

/// Every operation that works one 128-bit lane at a time, on one lane: `a`
/// the first source, `b` the second, `imm` that lane's immediate and `mask`
/// that lane of the blend mask — or, for an FMA, of the destination, its
/// third source. `None` for an operation that is not one.
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
    if let Some(r) = fused(op, mask, a, b, env) {
        return Some(r);
    }
    if let Some(v) = variable_shift(op, a, b) {
        return exact(v);
    }
    match op {
        Op::VPBLENDD => {
            let mut out = a;
            for i in 0..4 {
                if imm & (1 << i) != 0 {
                    let m = 0xffff_ffffu64 << (32 * (i % 2));
                    out[i / 2] = (out[i / 2] & !m) | (b[i / 2] & m);
                }
            }
            exact(out)
        }
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
            Op::VBROADCASTI128,
            Op::VPBROADCASTB,
            Op::VPBROADCASTW,
            Op::VPBROADCASTD,
            Op::VPBROADCASTQ,
            Op::VINSERTI128,
            Op::VEXTRACTI128,
            Op::VPERM2I128,
            Op::VPERMD,
            Op::VPERMPS,
            Op::VPERMQ,
            Op::VPERMPD,
            Op::VPMASKMOVD,
            Op::VPGATHERDD,
            Op::VPGATHERQD,
            Op::VGATHERDPS,
            Op::VGATHERQPS,
            Op::VCVTPH2PS,
            Op::VCVTPS2PH,
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
    fn fma_form_matches_every_mnemonic() {
        use crate::cpu::x86::fpexec::{Subtract, fma_form};
        // The position arithmetic in `fma_form` against the names it decodes.
        let mut seen = 0;
        for &op in Op::ALL {
            let form = fma_form(op);
            if op.extension() != Some(Extension::Fma) {
                assert_eq!(form, None, "{op:?}");
                continue;
            }
            seen += 1;
            let form = form.unwrap();
            let name = op.mnemonic();
            let (kind, rest) = [
                "vfmaddsub",
                "vfmsubadd",
                "vfnmadd",
                "vfnmsub",
                "vfmadd",
                "vfmsub",
            ]
            .iter()
            .find_map(|k| name.strip_prefix(k).map(|r| (*k, r)))
            .unwrap();
            assert_eq!(rest[..3].parse::<u16>().unwrap(), form.order, "{name}");
            assert_eq!(&rest[3..4] == "s", form.scalar, "{name}");
            assert_eq!(&rest[4..] == "d", form.double, "{name}");
            assert_eq!(kind.starts_with("vfn"), form.negate_product, "{name}");
            let subtract = match kind {
                "vfmaddsub" => Subtract::Even,
                "vfmsubadd" => Subtract::Odd,
                "vfmsub" | "vfnmsub" => Subtract::All,
                _ => Subtract::None,
            };
            assert_eq!(subtract, form.subtract, "{name}");
        }
        assert_eq!(seen, 60);
    }

    #[test]
    fn the_lane_immediate_steps_through_the_byte_only_where_it_must() {
        assert_eq!(lane_imm(Op::SHUFPD, 0b1110, 1), 0b11);
        assert_eq!(lane_imm(Op::BLENDPD, 0b1110, 1), 0b11);
        assert_eq!(lane_imm(Op::BLENDPS, 0xa5, 1), 0x0a);
        assert_eq!(lane_imm(Op::SHUFPS, 0xa5, 1), 0xa5);
        assert_eq!(lane_imm(Op::CMPPS, 0x1f, 1), 0x1f);
        assert_eq!(lane_imm(Op::VPBLENDD, 0xa5, 1), 0x0a);
        assert_eq!(lane_imm(Op::MPSADBW, 0b101_011, 1), 0b101);
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
