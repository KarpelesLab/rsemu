//! Executing the SIMD extensions past SSE2: SSE3.
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
    fn a_signalling_nan_in_one_lane_reports_invalid() {
        let d = of_singles([ONE, 0x7fa0_0000, ONE, ONE]);
        let (_, flags) = horizontal(Op::HADDPS, d, d, Env::X86_SSE);
        assert!(flags.contains(Flags::INVALID));
    }
}
