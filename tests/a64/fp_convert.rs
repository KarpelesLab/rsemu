// A64 conformance: conversions, differentially tested against `rustc`'s
// constant evaluator.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from IEEE 754-2019 and
// DDI 0487; no emulator source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Why Rust's `as` is the right thing to write here
// ---------------------------------------------------------------------------
//
// Rust defines a float-to-integer `as` cast to **saturate**, with a NaN giving
// zero. That is not a coincidence and it is not a library routine: it is
// exactly what `FCVTZS`/`FCVTZU` do (DDI 0487 `FPToFixed`), which is why LLVM
// lowers `x as i64` on AArch64 to a bare `fcvtzs x0, d0` with no range check
// around it. So writing the cast in Rust and const-evaluating it gives an
// expectation for the *instruction*, from an implementation that is not ours.
//
// The same holds in the other direction — `i64 as f64` is `SCVTF`, rounded to
// nearest under `FPCR`'s reset value — and for `f64 as f32`, which is `FCVT`.

#![no_std]
#![no_main]

include!("rt.rs");

/// `binary64` values whose conversions are where an implementation goes wrong:
/// the two ends of every integer range, the values just inside and just
/// outside them, the NaNs and the infinities.
static VALUES64: &[u64] = &[
    0x0000_0000_0000_0000, // +0
    0x8000_0000_0000_0000, // -0
    0x3ff0_0000_0000_0000, // 1
    0xbff0_0000_0000_0000, // -1
    0x3fe0_0000_0000_0000, // 0.5 — truncates to zero
    0xbfe0_0000_0000_0000, // -0.5 — truncates to *minus* zero, then to 0
    0x3ff8_0000_0000_0000, // 1.5
    0xbff8_0000_0000_0000, // -1.5
    0x4008_0000_0000_0000, // 3
    0x41df_ffff_ffc0_0000, // 2^31 - 1, exactly
    0x41e0_0000_0000_0000, // 2^31 — one past i32::MAX
    0xc1e0_0000_0000_0000, // -2^31 — exactly i32::MIN
    0xc1e0_0000_0020_0000, // just below i32::MIN
    0x41ef_ffff_ffe0_0000, // 2^32 - 1, exactly u32::MAX
    0x41f0_0000_0000_0000, // 2^32
    0x43df_ffff_ffff_ffff, // just below 2^63
    0x43e0_0000_0000_0000, // 2^63 — one past i64::MAX
    0xc3e0_0000_0000_0000, // -2^63 — exactly i64::MIN
    0xc3e0_0000_0000_0001, // just below i64::MIN
    0x43f0_0000_0000_0000, // 2^64
    0x7fef_ffff_ffff_ffff, // the largest finite
    0xffef_ffff_ffff_ffff,
    0x7ff0_0000_0000_0000, // +inf
    0xfff0_0000_0000_0000, // -inf
    0x7ff8_0000_0000_0000, // a quiet NaN — zero in every direction
    0x0000_0000_0000_0001, // the smallest subnormal
    0x000f_ffff_ffff_ffff, // the largest subnormal
    0x0010_0000_0000_0000, // the smallest normal
    0x3810_0000_0000_0000, // 2^-126, which narrows to a binary32 normal
    0x3690_0000_0000_0000, // 2^-150, which narrows to zero
    0x36a0_0000_0000_0000, // 2^-149, the smallest binary32 subnormal
    0x47ef_ffff_e000_0000, // f32::MAX exactly
    0x47ef_ffff_f000_0000, // the tie that rounds up to binary32 infinity
    0x4059_0000_0000_0000, // 100
    0xc059_0000_0000_0000, // -100
];

/// Integers whose conversion to floating point is inexact, plus the exact
/// ones on either side.
static INTS: &[u64] = &[
    0,
    1,
    0xffff_ffff,
    0x8000_0000,
    0x7fff_ffff,
    0x0020_0000_0000_0000,       // 2^53, the last exactly representable
    0x0020_0000_0000_0001,       // 2^53 + 1, a tie that rounds to even
    0x0020_0000_0000_0003,       // 2^53 + 3, a tie that rounds up
    0x7fff_ffff_ffff_ffff,       // i64::MAX, which rounds up to 2^63
    0x8000_0000_0000_0000,       // i64::MIN as unsigned, exact
    0xffff_ffff_ffff_ffff,       // u64::MAX, which rounds up to 2^64
    0xffff_ffff_ffff_fbff,
    0x0123_4567_89ab_cdef,
];

macro_rules! cast_from64 {
    ($name:ident, $to:ty) => {
        static $name: [u64; VALUES64.len()] = {
            let mut out = [0u64; VALUES64.len()];
            let mut i = 0;
            while i < VALUES64.len() {
                out[i] = f64::from_bits(VALUES64[i]) as $to as u64;
                i += 1;
            }
            out
        };
    };
}

cast_from64!(TO_I32, i32);
cast_from64!(TO_U32, u32);
cast_from64!(TO_I64, i64);
cast_from64!(TO_U64, u64);

/// `f64 as f32`, kept as `binary32` bits.
static NARROW: [u64; VALUES64.len()] = {
    let mut out = [0u64; VALUES64.len()];
    let mut i = 0;
    while i < VALUES64.len() {
        out[i] = (f64::from_bits(VALUES64[i]) as f32).to_bits() as u64;
        i += 1;
    }
    out
};

/// `f32 as f64` of the narrowed value — a widening, which is always exact.
static WIDEN: [u64; VALUES64.len()] = {
    let mut out = [0u64; VALUES64.len()];
    let mut i = 0;
    while i < VALUES64.len() {
        out[i] = (f32::from_bits(NARROW[i] as u32) as f64).to_bits();
        i += 1;
    }
    out
};

macro_rules! cast_to_float {
    ($name:ident, $from:ty, $to:ty, $bits:ty) => {
        static $name: [u64; INTS.len()] = {
            let mut out = [0u64; INTS.len()];
            let mut i = 0;
            while i < INTS.len() {
                out[i] = ((INTS[i] as $from) as $to).to_bits() as u64;
                i += 1;
            }
            out
        };
    };
}

cast_to_float!(FROM_I64, i64, f64, u64);
cast_to_float!(FROM_U64, u64, f64, u64);
cast_to_float!(FROM_I32, i32, f64, u64);
cast_to_float!(FROM_U32, u32, f64, u64);
cast_to_float!(FROM_I64_F32, i64, f32, u32);
cast_to_float!(FROM_U64_F32, u64, f32, u32);

fn run() -> Report {
    for (i, &bits) in VALUES64.iter().enumerate() {
        let v = f64::from_bits(black_box(bits));
        let n = i as u64 + 1;
        // Float to integer: `FCVTZS`/`FCVTZU` at both widths.
        if (v as i32 as u64) != TO_I32[i] {
            return (n, v as i32 as u64, TO_I32[i], 1);
        }
        if (v as u32 as u64) != TO_U32[i] {
            return (n, v as u32 as u64, TO_U32[i], 2);
        }
        if (v as i64 as u64) != TO_I64[i] {
            return (n, v as i64 as u64, TO_I64[i], 3);
        }
        if (v as u64) != TO_U64[i] {
            return (n, v as u64, TO_U64[i], 4);
        }
        // Float to float: `FCVT` both ways.
        let narrow = (v as f32).to_bits() as u64;
        if !same_f32(narrow, NARROW[i]) {
            return (n, narrow, NARROW[i], 5);
        }
        let wide = (f32::from_bits(black_box(NARROW[i] as u32)) as f64).to_bits();
        if !same_f64(wide, WIDEN[i]) {
            return (n, wide, WIDEN[i], 6);
        }
    }
    for (i, &bits) in INTS.iter().enumerate() {
        let v = black_box(bits);
        let n = 100 + i as u64 + 1;
        // Integer to float: `SCVTF`/`UCVTF`, rounded to nearest.
        if ((v as i64) as f64).to_bits() != FROM_I64[i] {
            return (n, ((v as i64) as f64).to_bits(), FROM_I64[i], 7);
        }
        if (v as f64).to_bits() != FROM_U64[i] {
            return (n, (v as f64).to_bits(), FROM_U64[i], 8);
        }
        if ((v as i32) as f64).to_bits() != FROM_I32[i] {
            return (n, ((v as i32) as f64).to_bits(), FROM_I32[i], 9);
        }
        if ((v as u32) as f64).to_bits() != FROM_U32[i] {
            return (n, ((v as u32) as f64).to_bits(), FROM_U32[i], 10);
        }
        let got = ((v as i64) as f32).to_bits() as u64;
        if got != FROM_I64_F32[i] {
            return (n, got, FROM_I64_F32[i], 11);
        }
        let got = (v as f32).to_bits() as u64;
        if got != FROM_U64_F32[i] {
            return (n, got, FROM_U64_F32[i], 12);
        }
    }
    PASS
}
