// A64 conformance: floating-point code written the way anybody writes it.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from IEEE 754-2019 and
// DDI 0487; no emulator source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Why this guest exists
// ---------------------------------------------------------------------------
//
// `fp_arith.rs` and `fp_convert.rs` feed every operand through `black_box` as
// a *bit pattern* and never write a floating-point literal, because the round
// that added scalar floating point could not run one: LLVM materialises `0.0`
// with `MOVI Dd, #0`, an Advanced SIMD encoding, and vectorises any loop long
// enough to be worth it. That made the two oracle guests carefully unnatural,
// and the honest statement at the time was that *compiled scalar
// floating-point code was not fully runnable*.
//
// This file is the same oracle with the contortion removed. Constants are
// literals, accumulators start at `0.0`, comparisons are against `0.0`, the
// arrays are long enough for the vectoriser to take an interest, and it is
// left alone when it does. The only `black_box` is on the *input*, which is
// what stops the whole program folding into its answer — and that one is not a
// contortion but the definition of a run-time input.
//
// It proves two things at once:
//
// 1. the arithmetic still agrees with `rustc`'s constant evaluator
//    (`rustc_apfloat`, which shares no code with `src/float`), case by case;
// 2. the *encodings* LLVM picks for ordinary code all execute — and nobody
//    here picked them. At the commit that added this file they were `MOVI`
//    (scalar and vector), `LD1`, `ST1` of a single lane, `UMOV`, `ADDV`,
//    `ADD`/`MUL`/`AND` lanewise, `USHLL`/`UADDW`, `FADD`/`FMUL`/`FSUB`
//    lanewise, `FCVTL`/`FCVTN`, `FCVTZS`/`SCVTF` lanewise, `FCMGT` against
//    zero, `EXT`, `DUP` and `ZIP1`.
//
// The first case asserts `ID_AA64PFR0_EL1.FP == .AdvSIMD`, which DDI 0487
// requires and which this core could not honestly report until the vector
// instructions existed. On a core that answered `AdvSIMD = 0b1111` the rest of
// this file would `UNDEF` on its first vector instruction anyway; asserting it
// makes the failure say *why*.

#![no_std]
#![no_main]

include!("rt.rs");

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// Sixteen `binary64` values, as bit patterns so the file says exactly which
/// numbers it means: ordinary magnitudes, a subnormal, a zero of each sign,
/// and values whose products need every bit of the significand.
static XS: [u64; 16] = [
    0x3ff0_0000_0000_0000, // 1.0
    0xbfe0_0000_0000_0000, // -0.5
    0x4009_21fb_5444_2d18, // pi
    0x0000_0000_0000_0000, // +0.0
    0x8000_0000_0000_0000, // -0.0
    0x3fb9_9999_9999_999a, // 0.1
    0xc02e_0000_0000_0000, // -15.0
    0x0000_0000_0000_0001, // the smallest subnormal
    0x4341_c379_37e0_8000, // a value above 2^53
    0x3fe5_5555_5555_5555, // 1/3, rounded
    0xc009_21fb_5444_2d18, // -pi
    0x7fef_ffff_ffff_ffff, // the largest finite
    0x3ca0_0000_0000_0000, // 2^-53
    0x4024_0000_0000_0000, // 10.0
    0xbff8_0000_0000_0000, // -1.5
    0x3ff0_0000_0000_0001, // 1.0 + one ulp
];

/// How long the bulk arrays are. Long enough that LLVM vectorises rather than
/// unrolling — sixteen elements it simply unrolls, which is exactly what the
/// first draft of this file found out.
const N: usize = 256;

/// A deterministic spread of `binary64` values with no infinity or NaN in it,
/// built by arithmetic rather than typed out.
const fn doubles() -> [f64; N] {
    let mut out = [0.0f64; N];
    let mut i = 0;
    while i < N {
        // Inexact on purpose: 0.375 and 0.1 make most of these values that
        // are not binary fractions, so the sums below round.
        out[i] = (i as f64 - 128.0) * 0.375 + 0.1;
        i += 1;
    }
    out
}

/// The same in `binary32`.
const fn singles() -> [f32; N] {
    let mut out = [0.0f32; N];
    let mut i = 0;
    while i < N {
        out[i] = (i as f32 - 128.0) * 0.375 + 0.1;
        i += 1;
    }
    out
}

/// Bytes, from a small linear congruential sequence.
const fn bytes() -> [u8; N] {
    let mut out = [0u8; N];
    let mut state = 1u32;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = (state >> 16) as u8;
        i += 1;
    }
    out
}

/// Words, likewise, with the two extremes forced in.
const fn words() -> [i32; N] {
    let mut out = [0i32; N];
    let mut state = 7u32;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = state as i32 >> 8;
        i += 1;
    }
    out[0] = i32::MIN;
    out[1] = i32::MAX;
    out
}

static DOUBLES: [f64; N] = doubles();
static SINGLES: [f32; N] = singles();
static BYTES: [u8; N] = bytes();
static WORDS: [i32; N] = words();

// ---------------------------------------------------------------------------
// The kernels. Each is a `const fn`, so the host evaluates it and the guest
// runs the same text — a difference between the two answers is a difference
// between `rustc_apfloat` and `src/float`.
// ---------------------------------------------------------------------------

/// Horner's rule with literal coefficients. Every one of them is a constant
/// LLVM must materialise, and the accumulator starts at a literal zero.
const fn poly(x: f64) -> f64 {
    let mut acc = 0.0f64;
    let c = [1.0f64, -0.5, 0.25, -0.125, 0.0625];
    let mut i = 0;
    while i < 5 {
        acc = acc * x + c[i];
        i += 1;
    }
    acc
}

/// Comparison against a literal zero, and the shape a clamp takes.
const fn clamped(x: f64) -> f64 {
    let y = if x < 0.0 { -x } else { x };
    if y > 10.0 {
        10.0
    } else if y < 0.5 {
        0.5
    } else {
        y
    }
}

/// A lanewise multiply-and-add over `binary64` — the loop shape whose only
/// dependence is the array index, so nothing stops the vectoriser.
const fn scale_add(v: &[f64; N]) -> [f64; N] {
    let mut out = [0.0f64; N];
    let mut i = 0;
    while i < N {
        out[i] = v[i] * 1.5 + 0.25;
        i += 1;
    }
    out
}

/// The same in `binary32`, and a subtraction so the sign path is exercised.
const fn scale_sub(v: &[f32; N]) -> [f32; N] {
    let mut out = [0.0f32; N];
    let mut i = 0;
    while i < N {
        out[i] = v[i] * 0.75 - 2.0;
        i += 1;
    }
    out
}

/// `binary32` widened to `binary64` and narrowed back: `FCVTL` and `FCVTN`.
const fn widen_narrow(v: &[f32; N]) -> [f32; N] {
    let mut out = [0.0f32; N];
    let mut i = 0;
    while i < N {
        out[i] = (v[i] as f64 * 0.1) as f32;
        i += 1;
    }
    out
}

/// Signed integer to floating point and back: `SCVTF` and `FCVTZS`.
const fn round_trip(v: &[i32; N]) -> [i32; N] {
    let mut out = [0i32; N];
    let mut i = 0;
    while i < N {
        out[i] = (v[i] as f32 * 0.5) as i32;
        i += 1;
    }
    out
}

/// How many lanes are strictly positive: a compare against `0.0` whose result
/// is a mask rather than a branch.
const fn positives(v: &[f32; N]) -> u32 {
    let mut n = 0u32;
    let mut i = 0;
    while i < N {
        if v[i] > 0.0 {
            n += 1;
        }
        i += 1;
    }
    n
}

/// A byte sum: the widening reduction, `USHLL`/`UADDW` and `ADDV`.
const fn byte_sum(v: &[u8; N]) -> u32 {
    let mut s = 0u32;
    let mut i = 0;
    while i < N {
        s = s.wrapping_add(v[i] as u32);
        i += 1;
    }
    s
}

/// Lanewise square and accumulate over words: `MUL V.4S` and `ADD V.4S`.
const fn word_squares(v: &[i32; N]) -> i32 {
    let mut s = 0i32;
    let mut i = 0;
    while i < N {
        s = s.wrapping_add(v[i].wrapping_mul(v[i]));
        i += 1;
    }
    s
}

/// Reverse a byte array: the shuffles.
const fn reversed(v: &[u8; N]) -> u32 {
    let mut out = [0u8; N];
    let mut i = 0;
    while i < N {
        out[i] = v[N - 1 - i];
        i += 1;
    }
    // Folded back to one number so the comparison is one word.
    let mut h = 2_166_136_261u32;
    let mut i = 0;
    while i < N {
        h = (h ^ out[i] as u32).wrapping_mul(16_777_619);
        i += 1;
    }
    h
}

/// A hash of an array of `binary64` bit patterns, so a whole array's worth of
/// results is one comparison.
const fn hash64(v: &[f64; N]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut i = 0;
    while i < N {
        h = (h ^ v[i].to_bits()).wrapping_mul(0x0000_0100_0000_01b3);
        i += 1;
    }
    h
}

/// The same for `binary32`.
const fn hash32(v: &[f32; N]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut i = 0;
    while i < N {
        h = (h ^ v[i].to_bits() as u64).wrapping_mul(0x0000_0100_0000_01b3);
        i += 1;
    }
    h
}

/// And for words.
const fn hash_i32(v: &[i32; N]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut i = 0;
    while i < N {
        h = (h ^ v[i] as u32 as u64).wrapping_mul(0x0000_0100_0000_01b3);
        i += 1;
    }
    h
}

// ---------------------------------------------------------------------------
// The expectations, evaluated by the host at compile time.
// ---------------------------------------------------------------------------

static POLY: [u64; 16] = {
    let mut out = [0u64; 16];
    let mut i = 0;
    while i < 16 {
        out[i] = poly(f64::from_bits(XS[i])).to_bits();
        i += 1;
    }
    out
};

static CLAMP: [u64; 16] = {
    let mut out = [0u64; 16];
    let mut i = 0;
    while i < 16 {
        out[i] = clamped(f64::from_bits(XS[i])).to_bits();
        i += 1;
    }
    out
};

static SCALE_ADD: u64 = hash64(&scale_add(&DOUBLES));
static SCALE_SUB: u64 = hash32(&scale_sub(&SINGLES));
static WIDENED: u64 = hash32(&widen_narrow(&SINGLES));
static ROUNDTRIP: u64 = hash_i32(&round_trip(&WORDS));
static POSITIVES: u32 = positives(&SINGLES);
static BYTESUM: u32 = byte_sum(&BYTES);
static SQUARES: i32 = word_squares(&WORDS);
static REVERSED: u32 = reversed(&BYTES);

// ---------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------

/// `ID_AA64PFR0_EL1`, read so the guest can say what it is running on.
fn id_aa64pfr0() -> u64 {
    let value: u64;
    unsafe {
        core::arch::asm!("mrs {}, id_aa64pfr0_el1", out(reg) value, options(nomem, nostack));
    }
    value
}

fn run() -> Report {
    // DDI 0487 D17.2.67: `FP` is bits 19:16 and `AdvSIMD` bits 23:20, and the
    // two must hold the same value. Everything below relies on it, because
    // the compiler chose vector instructions for code that says nothing about
    // vectors.
    let pfr0 = id_aa64pfr0();
    let fp = (pfr0 >> 16) & 0xf;
    let advsimd = (pfr0 >> 20) & 0xf;
    if fp != advsimd {
        return (1, advsimd, fp, 0);
    }
    if fp != 0 {
        return (2, fp, 0, 0);
    }

    // The one `black_box`: it makes the arrays run-time inputs, which is what
    // stops the program folding into its answer. Everything below is ordinary
    // code with ordinary literals in it.
    let xs = black_box(&XS);
    let doubles = black_box(&DOUBLES);
    let singles = black_box(&SINGLES);
    let bytes = black_box(&BYTES);
    let words = black_box(&WORDS);

    for i in 0..16 {
        let got = poly(f64::from_bits(xs[i])).to_bits();
        if !same_f64(got, POLY[i]) {
            return (10 + i as u64, got, POLY[i], 1);
        }
        let got = clamped(f64::from_bits(xs[i])).to_bits();
        if !same_f64(got, CLAMP[i]) {
            return (30 + i as u64, got, CLAMP[i], 2);
        }
    }

    let cases: [(u64, u64, u64); 8] = [
        (hash64(&scale_add(doubles)), SCALE_ADD, 3),
        (hash32(&scale_sub(singles)), SCALE_SUB, 4),
        (hash32(&widen_narrow(singles)), WIDENED, 5),
        (hash_i32(&round_trip(words)), ROUNDTRIP, 6),
        (positives(singles) as u64, POSITIVES as u64, 7),
        (byte_sum(bytes) as u64, BYTESUM as u64, 8),
        (word_squares(words) as u32 as u64, SQUARES as u32 as u64, 9),
        (reversed(bytes) as u64, REVERSED as u64, 10),
    ];
    for (i, (got, want, tag)) in cases.into_iter().enumerate() {
        if got != want {
            return (50 + i as u64, got, want, tag);
        }
    }

    PASS
}
