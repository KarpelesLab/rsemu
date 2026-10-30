//! Executing the SIMD extensions past SSE2: SSE3 and SSSE3.
//!
//! Every one of these is an SSE instruction in everything but its `CPUID`
//! bit — the same sixteen `XMM` registers, the same `MXCSR`, the same `CR0`
//! and `CR4` gates — so the dispatcher in `fpexec` applies those gates and
//! hands the operation here. What this file adds is the per-extension
//! feature check, which is a property of the *row* ([`Insn::extension`]), and
//! the instructions themselves.
//!
//! # Where the arithmetic comes from
//!
//! The floating-point lanes go through [`crate::float`] with `MXCSR` mapped
//! onto its [`Env`](crate::float::Env), exactly as SSE's own do — no host
//! float is involved — and an unmasked exception leaves the destination
//! untouched, through the same [`Exec::sse_raise`]. The lane kernels are free
//! functions, so the tests at the bottom can check them without a machine.
//!
//! # Sources
//!
//! *Intel SDM* volume 2, each instruction's reference page and "Operation"
//! section, and appendix A's opcode maps. No copyleft emulator was consulted
//! (`CLAUDE.md`, provenance).

use crate::float::{B32, B64, Env, Flags, binary};

use super::exec::{Ex, Exec, Fault, VEC_UD};
use super::isa::{Arg, Extension, Fields, Insn, ModRm, Op};

impl Exec<'_> {
    /// Whether this instance has the extension a row belongs to.
    ///
    /// `true` for a row that belongs to none: those are gated elsewhere, by
    /// the SSE and MMX dispatchers' own checks.
    pub(super) fn has_extension(&self, insn: Insn) -> bool {
        let features = self.cfg.features;
        match insn.extension() {
            None => true,
            Some(Extension::Sse3) => features.sse3,
            Some(Extension::Ssse3) => features.ssse3,
        }
    }

    /// Execute one SSE3-or-later instruction on `XMM` registers, its gates
    /// already passed.
    pub(super) fn simd_extension(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        match op {
            Op::ADDSUBPS | Op::HADDPS | Op::HSUBPS | Op::ADDSUBPD | Op::HADDPD | Op::HSUBPD => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                let (out, flags) = horizontal(op, dst, src, self.state.sse.env());
                // An unmasked exception in any lane leaves the whole
                // destination as it was, which is what the `?` does.
                self.sse_raise(flags)?;
                self.state.sse.set(reg, out);
                Ok(())
            }
            Op::MOVDDUP => {
                // A register source contributes its low double; a memory
                // source is that double alone.
                let src = self.sse_read_rm(f, Arg::Wq, false)?;
                self.state.sse.set(reg, [src[0], src[0]]);
                Ok(())
            }
            Op::MOVSLDUP | Op::MOVSHDUP => {
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                let odd = op == Op::MOVSHDUP;
                let pick = |q: u64| {
                    let lane = if odd { q >> 32 } else { q & 0xffff_ffff };
                    lane | (lane << 32)
                };
                self.state.sse.set(reg, [pick(src[0]), pick(src[1])]);
                Ok(())
            }
            Op::LDDQU => {
                // Memory only: the instruction exists to load data that
                // straddles a cache line, and a register has no such thing.
                if register_form {
                    return Err(Fault::bare(VEC_UD));
                }
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                self.state.sse.set(reg, src);
                Ok(())
            }
            _ if op.extension() == Some(Extension::Ssse3) => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                self.state
                    .sse
                    .set(reg, ssse3(op, dst, src, 16, f.imm as u8));
                Ok(())
            }
            // The table and this match are written together; a row that
            // reaches here unmatched is a decoder bug, and `#UD` is the
            // architecture's answer to an instruction a part does not have.
            _ => Err(Fault::bare(VEC_UD)),
        }
    }
}

/// The four singles of a register, low lane first.
fn singles(v: [u64; 2]) -> [u64; 4] {
    [
        v[0] & 0xffff_ffff,
        v[0] >> 32,
        v[1] & 0xffff_ffff,
        v[1] >> 32,
    ]
}

/// Four singles back into a register.
fn of_singles(l: [u64; 4]) -> [u64; 2] {
    [
        (l[0] & 0xffff_ffff) | (l[1] << 32),
        (l[2] & 0xffff_ffff) | (l[3] << 32),
    ]
}

/// SSE3's horizontal and alternating arithmetic.
///
/// Each result lane is one `float` operation rounded under `MXCSR`, and the
/// flags of every lane are gathered before any is written: the *Intel SDM*
/// volume 2 pages for `HADDPS` and its siblings give the operation lane by
/// lane, and their exception lists are the union of the lanes'.
///
/// - `ADDSUB`: even lanes subtract the source from the destination, odd lanes
///   add.
/// - `HADD`/`HSUB`: the low half of the result is the destination's adjacent
///   pairs, the high half the source's — `d0 + d1`, `d2 + d3`, `s0 + s1`,
///   `s2 + s3` — and the subtraction is always the even lane minus the odd.
pub(super) fn horizontal(op: Op, dst: [u64; 2], src: [u64; 2], env: Env) -> ([u64; 2], Flags) {
    let mut flags = Flags::NONE;
    match op {
        Op::ADDSUBPS | Op::HADDPS | Op::HSUBPS => {
            let (d, s) = (singles(dst), singles(src));
            let mut out = [0u64; 4];
            for (i, lane) in out.iter_mut().enumerate() {
                let (a, b, add) = match op {
                    Op::ADDSUBPS => (d[i], s[i], i % 2 == 1),
                    _ => {
                        // Lanes 0 and 1 pair up the destination, 2 and 3
                        // the source.
                        let from = if i < 2 { d } else { s };
                        let k = 2 * (i % 2);
                        (from[k], from[k + 1], op == Op::HADDPS)
                    }
                };
                let (v, fl) = if add {
                    binary::add::<B32>(a, b, env)
                } else {
                    binary::sub::<B32>(a, b, env)
                };
                flags |= fl;
                *lane = v;
            }
            (of_singles(out), flags)
        }
        _ => {
            let (pairs, add) = match op {
                Op::ADDSUBPD => ([(dst[0], src[0]), (dst[1], src[1])], [false, true]),
                Op::HADDPD => ([(dst[0], dst[1]), (src[0], src[1])], [true, true]),
                _ => ([(dst[0], dst[1]), (src[0], src[1])], [false, false]),
            };
            let mut out = [0u64; 2];
            for i in 0..2 {
                let (a, b) = pairs[i];
                let (v, fl) = if add[i] {
                    binary::add::<B64>(a, b, env)
                } else {
                    binary::sub::<B64>(a, b, env)
                };
                flags |= fl;
                out[i] = v;
            }
            (out, flags)
        }
    }
}

/// A register's bytes, low first.
fn bytes(v: [u64; 2]) -> [u8; 16] {
    let mut out = [0u8; 16];
    out[..8].copy_from_slice(&v[0].to_le_bytes());
    out[8..].copy_from_slice(&v[1].to_le_bytes());
    out
}

/// Sixteen bytes back into a register.
fn of_bytes(b: [u8; 16]) -> [u64; 2] {
    let mut lo = [0u8; 8];
    let mut hi = [0u8; 8];
    lo.copy_from_slice(&b[..8]);
    hi.copy_from_slice(&b[8..]);
    [u64::from_le_bytes(lo), u64::from_le_bytes(hi)]
}

/// A register's words as signed values, low first.
fn words(v: [u64; 2]) -> [i16; 8] {
    let b = bytes(v);
    core::array::from_fn(|i| i16::from_le_bytes([b[2 * i], b[2 * i + 1]]))
}

/// Eight words back into a register.
fn of_words(w: [i16; 8]) -> [u64; 2] {
    let mut b = [0u8; 16];
    for (i, w) in w.iter().enumerate() {
        b[2 * i..2 * i + 2].copy_from_slice(&w.to_le_bytes());
    }
    of_bytes(b)
}

/// A register's doublewords as signed values, low first.
fn dwords(v: [u64; 2]) -> [i32; 4] {
    [
        v[0] as i32,
        (v[0] >> 32) as i32,
        v[1] as i32,
        (v[1] >> 32) as i32,
    ]
}

/// Four doublewords back into a register.
fn of_dwords(d: [i32; 4]) -> [u64; 2] {
    [
        u64::from(d[0] as u32) | (u64::from(d[1] as u32) << 32),
        u64::from(d[2] as u32) | (u64::from(d[3] as u32) << 32),
    ]
}

/// Saturate to a signed word.
fn sat16(v: i32) -> i16 {
    v.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// One SSSE3 operation on `n` bytes of operand: 8 for the `MM` forms, 16
/// for the `XMM` ones. The result's bytes past `n` are zero.
///
/// Mostly the width does not matter, because the operation is per lane; it
/// matters to three families, which is why it is a parameter rather than the
/// `MM` forms being the low half of the `XMM` ones:
///
/// - **`PSHUFB`** indexes with as many bits as there are bytes — three for
///   an `MM` register, four for an `XMM` one — and a set bit 7 writes zero.
/// - **`PALIGNR`** shifts the *concatenation* `destination:source`, sixteen
///   bytes for `MM` operands and thirty-two for `XMM` ones, so the source's
///   bytes are not where a 128-bit shift would put them; a shift of the whole
///   concatenation or more yields zero.
/// - **The horizontal adds and subtracts** put the destination's pairs in
///   the low half of the result and the source's in the high half, and the
///   halves are half of `n`.
///
/// Every subtraction is the even lane minus the odd one above it, and
/// `PMADDUBSW` treats the *destination's* bytes as unsigned and the source's
/// as signed. `PMULHRSW` is `((a * b) >> 14) + 1`, then bits 16:1: a product
/// scaled by 2^-15 and rounded to nearest (*Intel SDM* volume 2, each
/// instruction's Operation section).
#[allow(clippy::too_many_lines)]
pub(super) fn ssse3(op: Op, dst: [u64; 2], src: [u64; 2], n: usize, imm: u8) -> [u64; 2] {
    let (d, s) = (bytes(dst), bytes(src));
    let (dw, sw) = (words(dst), words(src));
    let (dd, sd) = (dwords(dst), dwords(src));
    let nw = n / 2;
    let nd = n / 4;
    match op {
        Op::PSHUFB => {
            let mut out = [0u8; 16];
            for i in 0..n {
                let c = s[i];
                out[i] = if c & 0x80 != 0 {
                    0
                } else {
                    d[usize::from(c) & (n - 1)]
                };
            }
            of_bytes(out)
        }
        Op::PALIGNR => {
            let mut cat = [0u8; 32];
            cat[..n].copy_from_slice(&s[..n]);
            cat[n..2 * n].copy_from_slice(&d[..n]);
            let shift = usize::from(imm);
            let mut out = [0u8; 16];
            for (i, b) in out.iter_mut().take(n).enumerate() {
                *b = if i + shift < 2 * n { cat[i + shift] } else { 0 };
            }
            of_bytes(out)
        }
        Op::PSIGNB | Op::PABSB => {
            let mut out = [0u8; 16];
            for i in 0..n {
                let sign = s[i] as i8;
                out[i] = if op == Op::PABSB {
                    sign.unsigned_abs()
                } else {
                    match sign.signum() {
                        -1 => (d[i] as i8).wrapping_neg() as u8,
                        0 => 0,
                        _ => d[i],
                    }
                };
            }
            of_bytes(out)
        }
        Op::PSIGNW | Op::PABSW | Op::PMULHRSW => {
            let mut out = [0i16; 8];
            for i in 0..nw {
                out[i] = match op {
                    Op::PSIGNW => match sw[i].signum() {
                        -1 => dw[i].wrapping_neg(),
                        0 => 0,
                        _ => dw[i],
                    },
                    Op::PABSW => sw[i].unsigned_abs() as i16,
                    _ => {
                        let t = ((i32::from(dw[i]) * i32::from(sw[i])) >> 14) + 1;
                        (t >> 1) as i16
                    }
                };
            }
            of_words(out)
        }
        Op::PSIGND | Op::PABSD => {
            let mut out = [0i32; 4];
            for i in 0..nd {
                out[i] = if op == Op::PABSD {
                    sd[i].unsigned_abs() as i32
                } else {
                    match sd[i].signum() {
                        -1 => dd[i].wrapping_neg(),
                        0 => 0,
                        _ => dd[i],
                    }
                };
            }
            of_dwords(out)
        }
        Op::PMADDUBSW => {
            let mut out = [0i16; 8];
            for (i, w) in out.iter_mut().take(nw).enumerate() {
                let p = |k: usize| i32::from(d[k]) * i32::from(s[k] as i8);
                *w = sat16(p(2 * i) + p(2 * i + 1));
            }
            of_words(out)
        }
        Op::PHADDW | Op::PHADDSW | Op::PHSUBW | Op::PHSUBSW => {
            let mut out = [0i16; 8];
            let half = nw / 2;
            for (i, w) in out.iter_mut().take(nw).enumerate() {
                let from = if i < half { dw } else { sw };
                let k = 2 * (i % half);
                let (a, b) = (i32::from(from[k]), i32::from(from[k + 1]));
                *w = match op {
                    Op::PHADDW => (a + b) as i16,
                    Op::PHADDSW => sat16(a + b),
                    Op::PHSUBW => (a - b) as i16,
                    _ => sat16(a - b),
                };
            }
            of_words(out)
        }
        Op::PHADDD | Op::PHSUBD => {
            let mut out = [0i32; 4];
            let half = nd / 2;
            for (i, v) in out.iter_mut().take(nd).enumerate() {
                let from = if i < half { dd } else { sd };
                let k = 2 * (i % half);
                *v = if op == Op::PHADDD {
                    from[k].wrapping_add(from[k + 1])
                } else {
                    from[k].wrapping_sub(from[k + 1])
                };
            }
            of_dwords(out)
        }
        _ => dst,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A handful of exact single-precision constants.
    const ONE: u64 = 0x3f80_0000;
    const TWO: u64 = 0x4000_0000;
    const THREE: u64 = 0x4040_0000;
    const FOUR: u64 = 0x4080_0000;
    const FIVE: u64 = 0x40a0_0000;
    const SEVEN: u64 = 0x40e0_0000;
    const MINUS_ONE: u64 = 0xbf80_0000;

    #[test]
    fn haddps_pairs_the_destination_then_the_source() {
        // d = [1, 2, 3, 4], s = [1, 4, 3, 4]: [3, 7, 5, 7].
        let d = of_singles([ONE, TWO, THREE, FOUR]);
        let s = of_singles([ONE, FOUR, THREE, FOUR]);
        let (out, flags) = horizontal(Op::HADDPS, d, s, Env::X86_SSE);
        assert_eq!(out, of_singles([THREE, SEVEN, FIVE, SEVEN]));
        assert_eq!(flags, Flags::NONE);
        // HSUBPS is the even lane minus the odd: [-1, -1, -3, -1].
        let (out, _) = horizontal(Op::HSUBPS, d, s, Env::X86_SSE);
        assert_eq!(singles(out), [MINUS_ONE, MINUS_ONE, 0xc040_0000, MINUS_ONE]);
    }

    #[test]
    fn addsub_subtracts_the_even_lanes_and_adds_the_odd() {
        let d = of_singles([FOUR, FOUR, FOUR, FOUR]);
        let s = of_singles([ONE, ONE, ONE, ONE]);
        let (out, _) = horizontal(Op::ADDSUBPS, d, s, Env::X86_SSE);
        assert_eq!(out, of_singles([THREE, FIVE, THREE, FIVE]));
        // The doubles: 4 - 1 and 4 + 1.
        let d = [0x4010_0000_0000_0000, 0x4010_0000_0000_0000];
        let s = [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000];
        let (out, _) = horizontal(Op::ADDSUBPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0x4008_0000_0000_0000, 0x4014_0000_0000_0000]);
        // HADDPD: 4 + 4 in the low lane, 1 + 1 in the high.
        let (out, _) = horizontal(Op::HADDPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0x4020_0000_0000_0000, 0x4000_0000_0000_0000]);
        let (out, _) = horizontal(Op::HSUBPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0, 0]);
    }

    #[test]
    fn pshufb_indexes_with_four_bits_or_three_and_zeroes_on_bit_seven() {
        let table = [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908];
        // Reverse the bytes, but zero byte 1 (bit 7 of its control) and
        // index 0x1f in byte 15, which an XMM register reads as 0x0f.
        let ctl = of_bytes([
            0x0f, 0x80, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
            0x01, 0x1f,
        ]);
        let out = bytes(ssse3(Op::PSHUFB, table, ctl, 16, 0));
        assert_eq!(out, [15, 0, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 15]);
        // On an MM register the index is three bits: 0x0f reads byte 7.
        let out = bytes(ssse3(Op::PSHUFB, table, ctl, 8, 0));
        assert_eq!(&out[..8], &[7, 0, 5, 4, 3, 2, 1, 0]);
        assert_eq!(&out[8..], &[0; 8]);
    }

    #[test]
    fn palignr_shifts_the_concatenation_with_the_source_low() {
        let d = [0x1716_1514_1312_1110, 0x1f1e_1d1c_1b1a_1918];
        let s = [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908];
        // Shift 4: source bytes 4..15, then destination bytes 0..3.
        let out = bytes(ssse3(Op::PALIGNR, d, s, 16, 4));
        let want: [u8; 16] = core::array::from_fn(|i| {
            let k = i + 4;
            if k < 16 {
                k as u8
            } else {
                0x10 + (k - 16) as u8
            }
        });
        assert_eq!(out, want);
        // 16 is the destination alone; 32 and over is zero.
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 16), d);
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 24)[1], 0);
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 32), [0, 0]);
        // MM: the concatenation is sixteen bytes; shift 3.
        let out = ssse3(Op::PALIGNR, d, s, 8, 3)[0];
        assert_eq!(out, 0x1211_1007_0605_0403);
    }

    #[test]
    fn the_horizontal_word_operations_pair_destination_then_source() {
        let d = of_words([1, 2, 3, 4, 0x7fff, 1, -0x8000, 1]);
        let s = of_words([10, 20, 30, 40, 50, 60, 70, 80]);
        assert_eq!(
            words(ssse3(Op::PHADDW, d, s, 16, 0)),
            [3, 7, -0x8000, -0x7fff, 30, 70, 110, 150]
        );
        assert_eq!(
            words(ssse3(Op::PHADDSW, d, s, 16, 0)),
            [3, 7, 0x7fff, -0x7fff, 30, 70, 110, 150]
        );
        assert_eq!(
            words(ssse3(Op::PHSUBSW, d, s, 16, 0)),
            [-1, -1, 0x7ffe, -0x8000, -10, -10, -10, -10]
        );
        // On MM operands: two pairs from each, d = [1, 2, 3, 4].
        assert_eq!(
            words(ssse3(Op::PHSUBW, d, s, 8, 0)),
            [-1, -1, -10, -10, 0, 0, 0, 0]
        );
        let d = of_dwords([1, 2, 3, 4]);
        let s = of_dwords([5, 7, 11, 13]);
        assert_eq!(dwords(ssse3(Op::PHADDD, d, s, 16, 0)), [3, 7, 12, 24]);
        assert_eq!(dwords(ssse3(Op::PHSUBD, d, s, 8, 0)), [-1, -2, 0, 0]);
    }

    #[test]
    fn pmaddubsw_is_unsigned_times_signed_with_saturation() {
        // 255 * 127 + 255 * 127 = 64770, which saturates to 32767; and
        // 200 * -1 + 3 * 2 = -194.
        let mut d = [0u8; 16];
        let mut s = [0u8; 16];
        d[..4].copy_from_slice(&[255, 255, 200, 3]);
        s[..4].copy_from_slice(&[127, 127, 0xff, 2]);
        let out = words(ssse3(Op::PMADDUBSW, of_bytes(d), of_bytes(s), 16, 0));
        assert_eq!(&out[..2], &[0x7fff, -194]);
    }

    #[test]
    fn pmulhrsw_rounds_the_scaled_product() {
        // 0x4000 * 0x4000 = 2^28; >> 14 = 2^14; + 1; >> 1 = 0x2000.
        // -0x8000 * -0x8000 = 2^30 comes out as 0x8000 — the one case that
        // overflows a word, and it wraps to -0x8000.
        // 3 * 0x4000 = 0xc000; >> 14 = 3; + 1 = 4; >> 1 = 2. And -3 gives
        // -0xc000 >> 14 = -3; + 1 = -2; >> 1 = -1.
        let d = of_words([0x4000, -0x8000, 3, -3, 0, 0, 0, 0]);
        let s = of_words([0x4000, -0x8000, 0x4000, 0x4000, 0, 0, 0, 0]);
        assert_eq!(
            words(ssse3(Op::PMULHRSW, d, s, 16, 0)),
            [0x2000, -0x8000, 2, -1, 0, 0, 0, 0]
        );
    }

    #[test]
    fn psign_and_pabs_follow_the_sign_of_the_source() {
        let d = of_bytes([5; 16]);
        let mut s = [0u8; 16];
        s[..3].copy_from_slice(&[0xff, 0, 1]);
        let out = bytes(ssse3(Op::PSIGNB, d, of_bytes(s), 16, 0));
        assert_eq!(&out[..3], &[0xfb, 0, 5]);
        // -128 has no positive counterpart: its absolute value is 0x80,
        // read unsigned.
        let mut s = [0u8; 16];
        s[..3].copy_from_slice(&[0x80, 0xff, 1]);
        assert_eq!(
            &bytes(ssse3(Op::PABSB, d, of_bytes(s), 16, 0))[..3],
            &[0x80, 1, 1]
        );
        let s = of_dwords([i32::MIN, -7, 7, 0]);
        assert_eq!(
            ssse3(Op::PABSD, d, s, 16, 0),
            of_dwords([i32::MIN, 7, 7, 0])
        );
        assert_eq!(
            dwords(ssse3(Op::PSIGND, of_dwords([9, 9, 9, 9]), s, 16, 0)),
            [-9, -9, 9, 0]
        );
    }

    #[test]
    fn a_signalling_nan_in_one_lane_reports_invalid() {
        let d = of_singles([ONE, 0x7fa0_0000, ONE, ONE]);
        let (_, flags) = horizontal(Op::HADDPS, d, d, Env::X86_SSE);
        assert!(flags.contains(Flags::INVALID));
    }
}
