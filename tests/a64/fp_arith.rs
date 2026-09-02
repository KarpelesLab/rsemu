// A64 conformance: the four basic floating-point operations, differentially
// tested against `rustc`'s constant evaluator.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from IEEE 754-2019 and
// DDI 0487; no emulator source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Where the oracle comes from
// ---------------------------------------------------------------------------
//
// Each expectation below is computed **at compile time**, on the host, by
// `rustc`'s constant evaluator — which is `rustc_apfloat`, a Rust port of
// LLVM's `APFloat` and an IEEE-754 implementation that shares no code and no
// authorship with `src/float`. The same expression is then computed **at run
// time** by the guest, where `black_box` stops the optimiser folding it, so
// the compiler emits a real `FADD D0, D1, D2` that rsemu's soft-float
// executes.
//
// So this file is not a list of numbers somebody typed in and hoped were
// right: a mismatch means one of two independent IEEE-754 implementations is
// wrong, and IEEE 754 §5.1 makes the four operations here correctly rounded
// and therefore unique. That is what makes a corpus this project *generates*
// worth as much as one it downloads, for the part of the space it covers.
//
// What it cannot prove is at the bottom of rt.rs: NaN payloads are outside the
// standard, and constant folding is explicitly not required to reproduce the
// target's.

#![no_std]
#![no_main]

include!("rt.rs");

/// Operand pairs, as `binary64` bit patterns, chosen for the corners rather
/// than for coverage of the number line: every one of these is a case where a
/// wrong implementation gives a *plausible* answer.
static OPS64: &[(u64, u64)] = &[
    // Ordinary, exact.
    (0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000), // 1.0, 2.0
    (0x3fe0_0000_0000_0000, 0x3fd0_0000_0000_0000), // 0.5, 0.25
    // Ordinary, inexact — the decimal literals that are not binary fractions.
    (0x3fb9_9999_9999_999a, 0x3fc9_9999_9999_999a), // 0.1, 0.2
    (0x400921fb54442d18, 0x4005bf0a8b145769),       // pi, e
    // Signed zeros. `(+0) + (-0)` is `+0` at nearest; `(-0) - (+0)` is `-0`.
    (0x0000_0000_0000_0000, 0x8000_0000_0000_0000),
    (0x8000_0000_0000_0000, 0x0000_0000_0000_0000),
    (0x8000_0000_0000_0000, 0x8000_0000_0000_0000),
    // Cancellation to zero, which is `+0` at every rounding mode but one.
    (0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000),
    // A tie that round-to-nearest-even resolves downward: 1 + 2^-53.
    (0x3ff0_0000_0000_0000, 0x3ca0_0000_0000_0000),
    // ... and one it resolves upward: (1 + 2^-52) + 2^-53.
    (0x3ff0_0000_0000_0001, 0x3ca0_0000_0000_0000),
    // Subnormals, exact and inexact.
    (0x0000_0000_0000_0001, 0x0000_0000_0000_0001),
    (0x000f_ffff_ffff_ffff, 0x0000_0000_0000_0001),
    (0x0010_0000_0000_0000, 0x8000_0000_0000_0001), // normal - smallest sub
    // Overflow, and the largest finite.
    (0x7fef_ffff_ffff_ffff, 0x7fef_ffff_ffff_ffff),
    (0x7fef_ffff_ffff_ffff, 0x3ff0_0000_0000_0000),
    (0x7fef_ffff_ffff_ffff, 0x7c90_0000_0000_0000), // MAX + half an ulp
    // Underflow to a subnormal, and to zero.
    (0x0010_0000_0000_0000, 0x3fe0_0000_0000_0000), // MIN_POSITIVE * 0.5
    (0x0000_0000_0000_0001, 0x3fe0_0000_0000_0000), // smallest sub * 0.5
    // Infinities.
    (0x7ff0_0000_0000_0000, 0x3ff0_0000_0000_0000),
    (0x7ff0_0000_0000_0000, 0xfff0_0000_0000_0000),
    (0x7ff0_0000_0000_0000, 0x0000_0000_0000_0000),
    (0x7ff0_0000_0000_0000, 0x7ff0_0000_0000_0000),
    // Zero divided by zero, and a finite divided by zero.
    (0x0000_0000_0000_0000, 0x0000_0000_0000_0000),
    (0x3ff0_0000_0000_0000, 0x0000_0000_0000_0000),
    (0xbff0_0000_0000_0000, 0x0000_0000_0000_0000),
    // NaNs. The payload is not asserted (see `same_f64`); that these produce
    // *a* NaN, and that the surrounding arithmetic is unaffected, is.
    (0x7ff8_0000_0000_0000, 0x3ff0_0000_0000_0000),
    (0x7ff0_0000_0000_0001, 0x3ff0_0000_0000_0000),
    (0x3ff0_0000_0000_0000, 0x7ff8_0000_dead_beef),
    // A quotient that needs the full 53 bits and a sticky remainder.
    (0x3ff0_0000_0000_0000, 0x4008_0000_0000_0000), // 1 / 3
    (0x4059_0000_0000_0000, 0x4021_0000_0000_0000), // 100 / 8.5
    // A product whose exact result is 106 bits wide.
    (0x3ff0_0000_0000_0001, 0x3ff0_0000_0000_0001),
];

/// The same, as `binary32`. Not a subset of the above narrowed: binary32's
/// exponent range makes different values interesting, and a bug in the width
/// plumbing shows as a `binary64` answer where a `binary32` one belongs.
static OPS32: &[(u32, u32)] = &[
    (0x3f80_0000, 0x4000_0000), // 1.0, 2.0
    (0x3dcc_cccd, 0x3e4c_cccd), // 0.1, 0.2
    (0x0000_0001, 0x0000_0001), // smallest subnormals
    (0x007f_ffff, 0x0000_0001), // largest subnormal + smallest
    (0x7f7f_ffff, 0x7f7f_ffff), // overflow
    (0x7f7f_ffff, 0x3f80_0000),
    (0x0080_0000, 0x3f00_0000), // MIN_POSITIVE * 0.5
    (0x3f80_0000, 0x40400000),  // 1 / 3
    (0x3f80_0001, 0x3f80_0001), // a 48-bit exact product
    (0x7f80_0000, 0xff80_0000), // inf - inf
    (0x0000_0000, 0x0000_0000),
    (0x3f80_0000, 0x0000_0000),
    (0x7fc0_0000, 0x3f80_0000), // a quiet NaN
    (0x3f80_0000, 0x3f80_0000), // cancellation
    (0x3f80_0000, 0x3400_0000), // 1 + 2^-23, a tie
];

/// The expected results, evaluated by the host at compile time.
macro_rules! expect64 {
    ($name:ident, $a:ident, $b:ident, $op:tt) => {
        static $name: [u64; OPS64.len()] = {
            let mut out = [0u64; OPS64.len()];
            let mut i = 0;
            while i < OPS64.len() {
                let $a = f64::from_bits(OPS64[i].0);
                let $b = f64::from_bits(OPS64[i].1);
                out[i] = ($a $op $b).to_bits();
                i += 1;
            }
            out
        };
    };
}

macro_rules! expect32 {
    ($name:ident, $a:ident, $b:ident, $op:tt) => {
        static $name: [u32; OPS32.len()] = {
            let mut out = [0u32; OPS32.len()];
            let mut i = 0;
            while i < OPS32.len() {
                let $a = f32::from_bits(OPS32[i].0);
                let $b = f32::from_bits(OPS32[i].1);
                out[i] = ($a $op $b).to_bits();
                i += 1;
            }
            out
        };
    };
}

expect64!(ADD64, a, b, +);
expect64!(SUB64, a, b, -);
expect64!(MUL64, a, b, *);
expect64!(DIV64, a, b, /);

expect32!(ADD32, a, b, +);
expect32!(SUB32, a, b, -);
expect32!(MUL32, a, b, *);
expect32!(DIV32, a, b, /);

fn run() -> Report {
    for (i, &(x, y)) in OPS64.iter().enumerate() {
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
    for (i, &(x, y)) in OPS32.iter().enumerate() {
        let a = f32::from_bits(black_box(x));
        let b = f32::from_bits(black_box(y));
        let cases: [(u64, u64, u64); 4] = [
            ((a + b).to_bits() as u64, ADD32[i] as u64, 5),
            ((a - b).to_bits() as u64, SUB32[i] as u64, 6),
            ((a * b).to_bits() as u64, MUL32[i] as u64, 7),
            ((a / b).to_bits() as u64, DIV32[i] as u64, 8),
        ];
        for (got, want, tag) in cases {
            if !same_f32(got, want) {
                return (100 + i as u64 + 1, got, want, tag);
            }
        }
    }
    PASS
}
