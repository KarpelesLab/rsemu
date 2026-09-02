// A64 conformance: the four operations over a generated operand sweep.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from IEEE 754-2019; no emulator
// source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Why this file exists beside fp_arith.rs
// ---------------------------------------------------------------------------
//
// `fp_arith.rs` is thirty-odd hand-chosen operand pairs: the corners somebody
// thought of. That is the half of a corpus a person can write. The other half
// is volume — SingleStepTests ships ten thousand vectors per opcode for a
// reason — and volume is what nobody can write by hand.
//
// So the operands here are *generated*: a 64-bit LCG, run by `rustc`'s
// constant evaluator at compile time, shaped so that a useful fraction of the
// draws are zeros, subnormals, infinities and NaNs rather than uniformly
// random bit patterns (which are almost all enormous normals whose products
// simply overflow). The expectations are const-evaluated by the same
// `rustc_apfloat` that fp_arith.rs uses, and the guest recomputes each one at
// run time through a real `FADD`/`FSUB`/`FMUL`/`FDIV`.
//
// Eight thousand `binary64` vectors and four thousand `binary32` ones. That is
// still two orders of magnitude short of a SingleStepTests corpus, and saying
// so is the point: this is a wide smoke test with an independent oracle, not
// an exhaustive suite.

#![no_std]
#![no_main]

include!("rt.rs");

/// How many `binary64` operand pairs.
const N64: usize = 2000;
/// How many `binary32` operand pairs.
const N32: usize = 1000;

/// One step of the LCG. The multiplier is Knuth's MMIX constant; the value is
/// the *state*, and the operands are drawn from its high bits, because an
/// LCG's low bits have short periods and would make the sign bit alternate.
const fn step(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407)
}

/// Shape a random word into a `binary64` operand.
///
/// The distribution matters more than the randomness. Uniform bit patterns are
/// almost all normals with exponents near the ends of the range, whose
/// products overflow and whose sums are dominated by one operand — a sweep of
/// those tests one path. So: one draw in sixty-four is a zero or a subnormal,
/// one in sixty-four is an infinity or a NaN, half the rest sit within a
/// factor of `2^64` of one so that sums cancel and products stay finite, and
/// the remainder spans the whole exponent range so overflow and underflow are
/// still reached.
const fn shape64(r: u64) -> u64 {
    let sign = (r >> 63) << 63;
    let mut frac = r & 0x000f_ffff_ffff_ffff;
    let sel = (r >> 52) & 0x7ff;
    let exp: u64 = if sel < 32 {
        // A quarter of these are exact zeros and the rest subnormals. The
        // significand has to be *forced* to zero: a draw from an LCG is never
        // zero by chance, so leaving it would mean the sweep never saw one.
        if sel < 8 {
            frac = 0;
        }
        0
    } else if sel < 64 {
        // A quarter are infinities; the rest are NaNs of both kinds, since
        // the quiet bit comes straight from `frac`.
        if sel < 40 {
            frac = 0;
        }
        0x7ff
    } else if sel < 1024 {
        // 1023 +- 64.
        1023 + ((r >> 20) & 0x7f) - 64
    } else {
        // The whole normal range.
        1 + ((r >> 20) % 0x7fe)
    };
    sign | (exp << 52) | frac
}

/// The same, for `binary32`.
const fn shape32(r: u64) -> u32 {
    let sign = ((r >> 63) as u32) << 31;
    let mut frac = (r as u32) & 0x007f_ffff;
    let sel = (r >> 52) & 0x7ff;
    let exp: u32 = if sel < 32 {
        if sel < 8 {
            frac = 0;
        }
        0
    } else if sel < 64 {
        if sel < 40 {
            frac = 0;
        }
        0xff
    } else if sel < 1024 {
        127 + ((r >> 20) as u32 & 0x1f) - 16
    } else {
        1 + ((r >> 20) as u32 % 0xfe)
    };
    sign | (exp << 23) | frac
}

/// The `binary64` operand pairs.
static OPS64: [(u64, u64); N64] = {
    let mut out = [(0u64, 0u64); N64];
    let mut state = 0x0123_4567_89ab_cdef;
    let mut i = 0;
    while i < N64 {
        state = step(state);
        let a = shape64(state);
        state = step(state);
        let b = shape64(state);
        out[i] = (a, b);
        i += 1;
    }
    out
};

/// The `binary32` operand pairs.
static OPS32: [(u32, u32); N32] = {
    let mut out = [(0u32, 0u32); N32];
    let mut state = 0xfedc_ba98_7654_3210;
    let mut i = 0;
    while i < N32 {
        state = step(state);
        let a = shape32(state);
        state = step(state);
        let b = shape32(state);
        out[i] = (a, b);
        i += 1;
    }
    out
};

macro_rules! expect64 {
    ($name:ident, $op:tt) => {
        static $name: [u64; N64] = {
            let mut out = [0u64; N64];
            let mut i = 0;
            while i < N64 {
                let a = f64::from_bits(OPS64[i].0);
                let b = f64::from_bits(OPS64[i].1);
                out[i] = (a $op b).to_bits();
                i += 1;
            }
            out
        };
    };
}

macro_rules! expect32 {
    ($name:ident, $op:tt) => {
        static $name: [u32; N32] = {
            let mut out = [0u32; N32];
            let mut i = 0;
            while i < N32 {
                let a = f32::from_bits(OPS32[i].0);
                let b = f32::from_bits(OPS32[i].1);
                out[i] = (a $op b).to_bits();
                i += 1;
            }
            out
        };
    };
}

expect64!(ADD64, +);
expect64!(SUB64, -);
expect64!(MUL64, *);
expect64!(DIV64, /);

expect32!(ADD32, +);
expect32!(SUB32, -);
expect32!(MUL32, *);
expect32!(DIV32, /);

fn run() -> Report {
    for i in 0..N64 {
        let (x, y) = OPS64[i];
        let a = f64::from_bits(black_box(x));
        let b = f64::from_bits(black_box(y));
        let cases: [(u64, u64, u64); 4] = [
            ((a + b).to_bits(), ADD64[i], 1),
            ((a - b).to_bits(), SUB64[i], 2),
            ((a * b).to_bits(), MUL64[i], 3),
            ((a / b).to_bits(), DIV64[i], 4),
        ];
        for (got, want, tag) in cases {
            if !same_f64(got, want) {
                return (i as u64 + 1, got, want, tag);
            }
        }
    }
    for i in 0..N32 {
        let (x, y) = OPS32[i];
        let a = f32::from_bits(black_box(x));
        let b = f32::from_bits(black_box(y));
        let cases: [(u64, u64, u64); 4] = [
            (u64::from((a + b).to_bits()), u64::from(ADD32[i]), 5),
            (u64::from((a - b).to_bits()), u64::from(SUB32[i]), 6),
            (u64::from((a * b).to_bits()), u64::from(MUL32[i]), 7),
            (u64::from((a / b).to_bits()), u64::from(DIV32[i]), 8),
        ];
        for (got, want, tag) in cases {
            if !same_f32(got, want) {
                return (100_000 + i as u64 + 1, got, want, tag);
            }
        }
    }
    PASS
}
