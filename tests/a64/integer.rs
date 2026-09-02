// A64 conformance: the integer core, through whatever LLVM decides to emit.
//
// Copyright (c) Karpeles Lab Inc. MIT.
//
// ---------------------------------------------------------------------------
// What this proves that a unit test does not
// ---------------------------------------------------------------------------
//
// The expectations are const-evaluated on the host, so they are `rustc`'s
// arithmetic rather than ours — but integer arithmetic is not where the
// interest is. It is in the *instruction selection*: nobody here chose which
// encodings this file contains. `a % b` with a variable divisor becomes
// `SDIV` + `MSUB`; a `u128` product becomes `UMULH` + `MADD` + `ADDS`/`ADCS`;
// `rotate_left` becomes `RORV` with a negated amount; `trailing_zeros` becomes
// `RBIT` + `CLZ`; a `u128` division becomes several hundred instructions of
// `compiler_builtins` that no hand-written test would ever produce. Running
// those end to end — fetch, decode, execute, flags, branches — is the part a
// table-driven unit test cannot reach.

#![no_std]
#![no_main]

include!("rt.rs");

/// 64-bit operand pairs: the boundaries, the signs, and the shift amounts that
/// wrap.
static PAIRS: &[(u64, u64)] = &[
    (0, 0),
    (1, 1),
    (0, 1),
    (1, 0),
    (u64::MAX, 1),
    (1, u64::MAX),
    (u64::MAX, u64::MAX),
    (0x8000_0000_0000_0000, 1),
    (0x8000_0000_0000_0000, u64::MAX), // i64::MIN / -1, the wrapping case
    (0x7fff_ffff_ffff_ffff, 1),
    (0x7fff_ffff_ffff_ffff, 0x7fff_ffff_ffff_ffff),
    (0x0123_4567_89ab_cdef, 0xfedc_ba98_7654_3210),
    (0xdead_beef_cafe_f00d, 0x0000_0000_0000_003f),
    (0xdead_beef_cafe_f00d, 0x0000_0000_0000_0040), // a shift of the width
    (0x0000_0000_ffff_ffff, 0x0000_0001_0000_0000),
    (42, 7),
    (1000003, 65537),
    (0xffff_ffff_0000_0001, 0x0000_0000_8000_0000),
];

/// Everything derived from one pair, computed by the host.
///
/// The shape is a flat array rather than a struct so the guest can walk it
/// with an index and report which column disagreed.
const COLUMNS: usize = 22;

static EXPECT: [[u64; COLUMNS]; PAIRS.len()] = {
    let mut out = [[0u64; COLUMNS]; PAIRS.len()];
    let mut i = 0;
    while i < PAIRS.len() {
        let (a, b) = PAIRS[i];
        let (sa, sb) = (a as i64, b as i64);
        out[i] = [
            a.wrapping_add(b),
            a.wrapping_sub(b),
            a.wrapping_mul(b),
            if b == 0 { 0 } else { a / b },
            if b == 0 { 0 } else { a % b },
            if b == 0 { 0 } else { sa.wrapping_div(sb) as u64 },
            if b == 0 { 0 } else { sa.wrapping_rem(sb) as u64 },
            a.wrapping_shl(b as u32),
            a.wrapping_shr(b as u32),
            (sa.wrapping_shr(b as u32)) as u64,
            a.rotate_left((b & 63) as u32),
            a.rotate_right((b & 63) as u32),
            a.leading_zeros() as u64,
            a.trailing_zeros() as u64,
            a.swap_bytes(),
            a.reverse_bits(),
            (a as u32).swap_bytes() as u64,
            ((a as u128 * b as u128) >> 64) as u64,
            ((sa as i128 * sb as i128) >> 64) as u64,
            (a < b) as u64,
            (sa < sb) as u64,
            (a & b) ^ (a | b) ^ !(a ^ b),
        ];
        i += 1;
    }
    out
};

/// The 128-bit half: `compiler_builtins` routines, which are hundreds of
/// instructions of ordinary integer code apiece.
static WIDE: [[u64; 4]; PAIRS.len()] = {
    let mut out = [[0u64; 4]; PAIRS.len()];
    let mut i = 0;
    while i < PAIRS.len() {
        let (a, b) = PAIRS[i];
        let x = ((a as u128) << 64) | b as u128;
        let y = ((b as u128) << 32) | 0x9e37_79b9;
        let q = if y == 0 { 0 } else { x / y };
        let r = if y == 0 { 0 } else { x % y };
        let p = x.wrapping_mul(y);
        out[i] = [q as u64, (q >> 64) as u64, r as u64, p as u64];
        i += 1;
    }
    out
};

fn run() -> Report {
    for (i, &(x, y)) in PAIRS.iter().enumerate() {
        let a = black_box(x);
        let b = black_box(y);
        let (sa, sb) = (a as i64, b as i64);
        let got: [u64; COLUMNS] = [
            a.wrapping_add(b),
            a.wrapping_sub(b),
            a.wrapping_mul(b),
            if b == 0 { 0 } else { a / b },
            if b == 0 { 0 } else { a % b },
            if b == 0 { 0 } else { sa.wrapping_div(sb) as u64 },
            if b == 0 { 0 } else { sa.wrapping_rem(sb) as u64 },
            a.wrapping_shl(b as u32),
            a.wrapping_shr(b as u32),
            (sa.wrapping_shr(b as u32)) as u64,
            a.rotate_left((b & 63) as u32),
            a.rotate_right((b & 63) as u32),
            a.leading_zeros() as u64,
            a.trailing_zeros() as u64,
            a.swap_bytes(),
            a.reverse_bits(),
            (a as u32).swap_bytes() as u64,
            ((a as u128 * b as u128) >> 64) as u64,
            ((sa as i128 * sb as i128) >> 64) as u64,
            (a < b) as u64,
            (sa < sb) as u64,
            (a & b) ^ (a | b) ^ !(a ^ b),
        ];
        for (column, (&g, &w)) in got.iter().zip(EXPECT[i].iter()).enumerate() {
            if g != w {
                return (i as u64 + 1, g, w, column as u64);
            }
        }

        let wx = ((a as u128) << 64) | b as u128;
        let wy = ((b as u128) << 32) | 0x9e37_79b9;
        let q = if wy == 0 { 0 } else { wx / wy };
        let r = if wy == 0 { 0 } else { wx % wy };
        let p = wx.wrapping_mul(wy);
        let wide: [u64; 4] = [q as u64, (q >> 64) as u64, r as u64, p as u64];
        for (column, (&g, &w)) in wide.iter().zip(WIDE[i].iter()).enumerate() {
            if g != w {
                return (i as u64 + 1, g, w, 100 + column as u64);
            }
        }
    }

    // A loop with a carried dependency, so the branch and the flags are
    // exercised rather than only the arithmetic. The constant is checked
    // against the same computation done by the host.
    const SUM: u64 = {
        let mut acc = 0u64;
        let mut i = 0u64;
        while i < 5000 {
            acc = acc.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(i);
            i += 1;
        }
        acc
    };
    let mut acc = black_box(0u64);
    let mut i = black_box(0u64);
    while i < 5000 {
        acc = acc.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(i);
        i += 1;
    }
    if acc != SUM {
        return (900, acc, SUM, 0);
    }
    PASS
}
