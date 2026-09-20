//! The 68881's transcendental functions.
//!
//! # What the manual asks for, and what this gives
//!
//! M68881UM §4.3.2 is unusually candid: "The IEEE specification does not
//! define the error bound to which transcendental (except square root)
//! functions are to be performed ... In general, the worst-case accuracy of
//! any transcendental function is one unit in the last place of **double**
//! precision (which is equal to **4096 units in the last place of
//! extended**). The typical error bound for these instructions is
//! approximately **64 units in the last place of extended precision**." The
//! trigonometric functions add a second limit: "large arguments may lose
//! accuracy during reduction, and very large arguments (greater than
//! approximately 10^20) lose all accuracy" (§4, *FCOS*).
//!
//! So the manual specifies a *bound*, not an algorithm, and it is a loose
//! one. This module computes every function to well inside it:
//!
//! - all of the arithmetic is carried in a **128-bit significand**
//!   ([`Wide`]) and rounded to the destination exactly once, so the error
//!   before that rounding is a few parts in `2^120`;
//! - the trigonometric argument reduction is **exact for every representable
//!   argument**, by Payne–Hanek against sixteen thousand bits of `2/π`,
//!   rather than losing accuracy above `10^20`.
//!
//! That makes this core *more* accurate than the part, which is a deviation
//! and is recorded as one: software written against a 68881's exact rounding
//! of `FSIN` will see different low bits here, and nothing short of that
//! part's microcode could reproduce them. `docs/cpu/m68k.md` says so.
//!
//! **How it was measured**: `tests_transcend.rs` holds a table of arguments
//! and correctly rounded results computed by **GNU `bc -l` at `scale=80`** —
//! a different implementation, in a different language, at four times the
//! precision — and asserts that every one of them comes back exactly. The
//! test's module documentation has the commands.
//!
//! # No host floating point
//!
//! `ROADMAP.md` §9.1. Every line below is integer arithmetic on `u64` and
//! `u128`. There is no `f32` or `f64` in this file.
//!
//! # Sources
//!
//! The series and the reductions are standard mathematics rather than
//! anybody's code: the exponential and trigonometric Taylor series, the
//! `atanh` form of the logarithm, the half-angle recurrence for the arc
//! tangent, and the Payne–Hanek reduction (Payne and Hanek, *Radian
//! reduction for trigonometric functions*, SIGNUM Newsletter 18(1), 1983).
//! The constants are computed, not copied — the generator commands are with
//! each table.

use crate::float::x87::{self, F80};
use crate::float::{Env, Flags, Spec};

use super::isa::fp::FpOp;

// ---------------------------------------------------------------------------
// A 128-bit significand
// ---------------------------------------------------------------------------

/// A sign, a binary exponent and a 128-bit significand.
///
/// The value is `(-1)^sign × (frac / 2^127) × 2^exp`, with `frac` in
/// `[2^127, 2^128)` for everything but zero, which has `frac == 0`. Twice the
/// significand of the format it feeds, which is what makes a chain of forty
/// operations still round correctly at the end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Wide {
    sign: bool,
    exp: i32,
    frac: u128,
}

impl Wide {
    const ZERO: Wide = Wide {
        sign: false,
        exp: 0,
        frac: 0,
    };
    const ONE: Wide = Wide {
        sign: false,
        exp: 0,
        frac: 1 << 127,
    };
    const TWO: Wide = Wide {
        sign: false,
        exp: 1,
        frac: 1 << 127,
    };
    const HALF: Wide = Wide {
        sign: false,
        exp: -1,
        frac: 1 << 127,
    };

    const fn is_zero(self) -> bool {
        self.frac == 0
    }

    const fn neg(self) -> Wide {
        if self.is_zero() {
            return self;
        }
        Wide {
            sign: !self.sign,
            ..self
        }
    }

    const fn abs(self) -> Wide {
        Wide {
            sign: false,
            ..self
        }
    }

    /// The same value scaled by a power of two, which is exact.
    const fn scale2(self, by: i32) -> Wide {
        if self.is_zero() {
            return self;
        }
        Wide {
            exp: self.exp + by,
            ..self
        }
    }

    /// A small integer.
    fn from_u64(v: u64) -> Wide {
        if v == 0 {
            return Wide::ZERO;
        }
        let shift = v.leading_zeros() + 64;
        Wide {
            sign: false,
            exp: 63 - v.leading_zeros() as i32,
            frac: u128::from(v) << shift,
        }
    }

    /// Build from a sign, the exponent of the significand's top bit, and a
    /// 128-bit significand that need not be normalized.
    fn normalized(sign: bool, exp: i32, frac: u128) -> Wide {
        if frac == 0 {
            return Wide::ZERO;
        }
        let shift = frac.leading_zeros();
        Wide {
            sign,
            exp: exp - shift as i32,
            frac: frac << shift,
        }
    }

    /// Whether `self`'s magnitude is at least `other`'s.
    fn at_least(self, other: Wide) -> bool {
        if self.is_zero() {
            return other.is_zero();
        }
        if other.is_zero() {
            return true;
        }
        (self.exp, self.frac) >= (other.exp, other.frac)
    }
}

/// `a + b`.
fn add(a: Wide, b: Wide) -> Wide {
    if a.is_zero() {
        return b;
    }
    if b.is_zero() {
        return a;
    }
    let (big, small) = if a.at_least(b) { (a, b) } else { (b, a) };
    let shift = (big.exp - small.exp) as u32;
    if shift >= 130 {
        return big;
    }
    let aligned = if shift >= 128 { 0 } else { small.frac >> shift };
    if big.sign == small.sign {
        let (sum, carried) = big.frac.overflowing_add(aligned);
        if carried {
            Wide {
                sign: big.sign,
                exp: big.exp + 1,
                frac: (sum >> 1) | (1 << 127),
            }
        } else {
            Wide {
                sign: big.sign,
                exp: big.exp,
                frac: sum,
            }
        }
    } else {
        Wide::normalized(big.sign, big.exp, big.frac - aligned)
    }
}

/// `a - b`.
fn sub(a: Wide, b: Wide) -> Wide {
    add(a, b.neg())
}

/// `a × b`, keeping the top 128 bits and rounding the rest away to nearest.
fn mul(a: Wide, b: Wide) -> Wide {
    if a.is_zero() || b.is_zero() {
        return Wide::ZERO;
    }
    const MASK: u128 = u64::MAX as u128;
    let (ah, al) = (a.frac >> 64, a.frac & MASK);
    let (bh, bl) = (b.frac >> 64, b.frac & MASK);
    let p0 = al * bl;
    let p1 = ah * bl;
    let p2 = al * bh;
    let p3 = ah * bh;
    let middle = (p1 & MASK) + (p2 & MASK) + (p0 >> 64);
    let low = (p0 & MASK) | (middle << 64);
    let high = p3 + (p1 >> 64) + (p2 >> 64) + (middle >> 64);
    // The product of two significands in `[1, 2)` is in `[1, 4)`, so the top
    // bit is either 127 or 126 of `high`.
    let (frac, exp, dropped) = if high >> 127 != 0 {
        (high, a.exp + b.exp + 1, low)
    } else {
        ((high << 1) | (low >> 127), a.exp + b.exp, low << 1)
    };
    // Round the discarded half to nearest; ties up, which is a bias of one
    // part in `2^128` and is dwarfed by everything else here.
    let frac = if dropped >> 127 != 0 {
        match frac.checked_add(1) {
            Some(rounded) => rounded,
            None => {
                return Wide {
                    sign: a.sign != b.sign,
                    exp: exp + 1,
                    frac: 1 << 127,
                };
            }
        }
    } else {
        frac
    };
    Wide {
        sign: a.sign != b.sign,
        exp,
        frac,
    }
}

/// `a ÷ b`, by long division of the significands.
fn div(a: Wide, b: Wide) -> Wide {
    if a.is_zero() || b.is_zero() {
        return Wide::ZERO;
    }
    // Keep the numerator below the denominator so the quotient's 128 bits
    // land where they are wanted; the bit that shifts out is worth one part
    // in `2^128`.
    let (numerator, exp) = if a.frac >= b.frac {
        (a.frac >> 1, a.exp - b.exp)
    } else {
        (a.frac, a.exp - b.exp - 1)
    };
    let mut remainder = numerator;
    let mut quotient: u128 = 0;
    for _ in 0..128 {
        let carry = remainder >> 127;
        remainder <<= 1;
        quotient <<= 1;
        if carry != 0 || remainder >= b.frac {
            remainder = remainder.wrapping_sub(b.frac);
            quotient |= 1;
        }
    }
    Wide {
        sign: a.sign != b.sign,
        exp,
        frac: quotient,
    }
}

/// The square root, by Newton's method.
fn sqrt(a: Wide) -> Wide {
    if a.is_zero() || a.sign {
        return Wide::ZERO;
    }
    // Split off an even power of two, leaving a significand in `[1, 4)` whose
    // square root is in `[1, 2)`.
    let (m, half) = if a.exp % 2 == 0 {
        (
            Wide {
                sign: false,
                exp: 0,
                frac: a.frac,
            },
            a.exp / 2,
        )
    } else {
        (
            Wide {
                sign: false,
                exp: 1,
                frac: a.frac,
            },
            (a.exp - 1) / 2,
        )
    };
    // `x ← (x + m/x) / 2` doubles the correct bits every step, so a seed good
    // to one bit reaches a hundred and twenty-eight in seven; nine leaves
    // room.
    let mut x = Wide::ONE;
    for _ in 0..9 {
        x = mul(add(x, div(m, x)), Wide::HALF);
    }
    x.scale2(half)
}

// ---------------------------------------------------------------------------
// Conversion
// ---------------------------------------------------------------------------

/// Take a finite non-zero [`F80`] apart into a sign, a 64-bit significand
/// normalized to bit 63, and the exponent of its *last* bit.
fn parts(v: F80) -> (bool, u64, i32) {
    let field = v.exp_field();
    let (sig, exp) = if field == 0 {
        (v.sig, -16445)
    } else {
        (v.sig, i32::from(field) - 16446)
    };
    let shift = sig.leading_zeros();
    (v.sign(), sig << shift, exp - shift as i32)
}

/// A finite [`F80`] as a [`Wide`], exactly.
fn from_f80(v: F80) -> Wide {
    if v.sig == 0 && v.exp_field() == 0 {
        return Wide {
            sign: v.sign(),
            ..Wide::ZERO
        };
    }
    let (sign, sig, exp) = parts(v);
    Wide {
        sign,
        exp: exp + 63,
        frac: u128::from(sig) << 64,
    }
}

/// Build an [`F80`] worth `(sig / 2^63) × 2^exp`.
///
/// `sig` need not be normalized — the low half of a [`Wide`] never is — and
/// normalizing it here is what keeps the result a *value* rather than one of
/// the unnormal encodings x87 refuses.
fn f80_from(sign: bool, exp: i32, sig: u64) -> F80 {
    if sig == 0 {
        return F80::new(if sign { 0x8000 } else { 0 }, 0);
    }
    let shift = sig.leading_zeros() as i32;
    let sig = sig << shift;
    let exp = exp - shift;
    let field = exp + 16383;
    let mark = if sign { 0x8000u16 } else { 0 };
    if field >= 1 {
        F80::new((field as u16) | mark, sig)
    } else {
        let down = 1 - field;
        let sig = if down >= 64 { 0 } else { sig >> down };
        F80::new(mark, sig)
    }
}

/// Round a [`Wide`] to the destination format, **once**.
///
/// The 128-bit significand is split into two values that are each exactly
/// representable, and their sum is rounded by `src/float`'s own adder — which
/// adds exactly and rounds at the end, so the result is the correctly rounded
/// one for the whole 128 bits rather than a double rounding.
fn to_f80(w: Wide, spec: Spec, env: Env) -> (F80, Flags) {
    if w.is_zero() {
        return (F80::new(if w.sign { 0x8000 } else { 0 }, 0), Flags::NONE);
    }
    // Outside the format's range the answer is an overflow or an underflow,
    // and the flags and the rounding-mode-dependent value both have to be the
    // ones an arithmetic operation would have produced.
    if w.exp > 16383 {
        let (value, flags) = x87::mul_to(F80::MAX_FINITE, Wide::TWO.as_f80(), spec, env);
        return (apply_sign(value, w.sign), flags);
    }
    if w.exp < -16446 {
        let smallest = F80::new(0, 1);
        let (value, flags) = x87::mul_to(smallest, F80::new(0x3ffe, 1 << 63), spec, env);
        return (apply_sign(value, w.sign), flags);
    }
    let hi = f80_from(w.sign, w.exp, (w.frac >> 64) as u64);
    let lo = f80_from(w.sign, w.exp - 64, w.frac as u64);
    x87::add_to(hi, lo, spec, env)
}

impl Wide {
    /// The exact `F80` of a value whose significand fits in sixty-four bits.
    fn as_f80(self) -> F80 {
        f80_from(self.sign, self.exp, (self.frac >> 64) as u64)
    }
}

/// `value` with `sign` applied.
fn apply_sign(value: F80, sign: bool) -> F80 {
    F80::new(
        (value.sign_exp & 0x7fff) | if sign { 0x8000 } else { 0 },
        value.sig,
    )
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
//
// Each is the correctly rounded 128-bit significand of the real number named,
// computed from a two-hundred-digit decimal value and rounded once, ties to
// even. They are mathematics rather than a transcription of any part's ROM.

/// π.
const PI: Wide = Wide {
    sign: false,
    exp: 1,
    frac: 0xc90f_daa2_2168_c234_c4c6_628b_80dc_1cd1,
};
/// π/2.
const PI_2: Wide = Wide {
    sign: false,
    exp: 0,
    frac: 0xc90f_daa2_2168_c234_c4c6_628b_80dc_1cd1,
};
/// π/4.
const PI_4: Wide = Wide {
    sign: false,
    exp: -1,
    frac: 0xc90f_daa2_2168_c234_c4c6_628b_80dc_1cd1,
};
/// The natural logarithm of two.
const LN2: Wide = Wide {
    sign: false,
    exp: -1,
    frac: 0xb172_17f7_d1cf_79ab_c9e3_b398_03f2_f6af,
};
/// The natural logarithm of ten.
const LN10: Wide = Wide {
    sign: false,
    exp: 1,
    frac: 0x935d_8ddd_aaa8_ac16_ea56_d62b_82d3_0a29,
};

include!("transcend_tables.rs");

/// `1/n` for the divisors the series below use.
fn recip(n: usize) -> Wide {
    RECIPROCAL[n]
}

// ---------------------------------------------------------------------------
// The functions
// ---------------------------------------------------------------------------

/// `e^r` for `|r| <= ln 2 / 2`, by its Taylor series.
///
/// The terms fall by a factor of at least `0.35/k`, so thirty of them put the
/// tail below `2^-135`.
fn exp_small(r: Wide) -> Wide {
    let mut sum = Wide::ONE;
    let mut term = Wide::ONE;
    for k in 1..=30usize {
        term = mul(mul(term, r), recip(k));
        if term.is_zero() {
            break;
        }
        sum = add(sum, term);
    }
    sum
}

/// `e^x`.
fn exp(x: Wide) -> Wide {
    if x.is_zero() {
        return Wide::ONE;
    }
    // n = round(x / ln 2), r = x - n ln 2, so |r| <= ln2/2.
    let n = round_to_i64(div(x, LN2));
    let r = sub(x, mul(Wide::from_i64(n), LN2));
    exp_small(r).scale2(n as i32)
}

/// `e^x - 1`, without the cancellation `exp(x) - 1` would suffer near zero.
fn expm1(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    // Below a quarter the series for `e^x - 1` converges directly and every
    // term is meaningful; above it the subtraction loses nothing.
    if x.exp < -2 {
        let mut sum = Wide::ZERO;
        let mut term = Wide::ONE;
        for k in 1..=40usize {
            term = mul(mul(term, x), recip(k));
            if term.is_zero() {
                break;
            }
            sum = add(sum, term);
        }
        return sum;
    }
    sub(exp(x), Wide::ONE)
}

/// `ln x` for `x > 0`.
fn ln(x: Wide) -> Wide {
    // x = m × 2^e with m in [1/√2, √2), so that z = (m-1)/(m+1) has
    // magnitude at most 0.1716 and the `atanh` series converges quickly.
    let mut e = x.exp;
    let mut m = Wide {
        sign: false,
        exp: 0,
        frac: x.frac,
    };
    // √2's significand, to decide which side of the interval m is on.
    const SQRT2_FRAC: u128 = 0xb504_f333_f9de_6484_597d_89b3_754a_be9f;
    if m.frac >= SQRT2_FRAC {
        m = m.scale2(-1);
        e += 1;
    }
    let z = div(sub(m, Wide::ONE), add(m, Wide::ONE));
    add(
        mul(Wide::from_i64(i64::from(e)), LN2),
        atanh_series(z).scale2(1),
    )
}

/// `ln(1 + x)`, without the cancellation `ln` of a nearby one would suffer.
fn ln1p(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    if x.exp < -2 {
        // ln(1+x) = 2 atanh(x / (2 + x)), and the quotient is computed from
        // x directly, so nothing is lost to the addition of one.
        let z = div(x, add(Wide::TWO, x));
        return atanh_series(z).scale2(1);
    }
    ln(add(Wide::ONE, x))
}

/// `atanh z = z + z³/3 + z⁵/5 + …` for `|z| < 0.18`.
fn atanh_series(z: Wide) -> Wide {
    let z2 = mul(z, z);
    let mut term = z;
    let mut sum = z;
    for k in 1..=32usize {
        term = mul(term, z2);
        if term.is_zero() {
            break;
        }
        sum = add(sum, mul(term, recip(2 * k + 1)));
    }
    sum
}

/// `atan x`.
fn atan(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    let sign = x.sign;
    let mut t = x.abs();
    // atan(t) = π/2 - atan(1/t) brings everything into [0, 1].
    let over = t.exp >= 0 && t.at_least(Wide::ONE);
    if over {
        t = div(Wide::ONE, t);
    }
    // Four half-angle steps, t <- t / (1 + √(1 + t²)), take |t| below 0.05.
    let mut halvings = 0;
    for _ in 0..4 {
        t = div(t, add(Wide::ONE, sqrt(add(Wide::ONE, mul(t, t)))));
        halvings += 1;
    }
    // atan(t) = t - t³/3 + t⁵/5 - …
    let t2 = mul(t, t);
    let mut term = t;
    let mut sum = t;
    for k in 1..=32usize {
        term = mul(term, t2).neg();
        if term.is_zero() {
            break;
        }
        sum = add(sum, mul(term, recip(2 * k + 1)));
    }
    let mut out = sum.scale2(halvings);
    if over {
        out = sub(PI_2, out);
    }
    if sign { out.neg() } else { out }
}

/// `asin x` for `|x| <= 1`.
fn asin(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    let sign = x.sign;
    let a = x.abs();
    // asin(1) is π/2 exactly; the form below would divide by zero.
    let out = if !sub(a, Wide::ONE).is_zero() {
        atan(div(a, sqrt(sub(Wide::ONE, mul(a, a)))))
    } else {
        PI_2
    };
    if sign { out.neg() } else { out }
}

/// `acos x` for `|x| <= 1`.
fn acos(x: Wide) -> Wide {
    // acos(x) = 2 atan(√((1-x)/(1+x))), which keeps its accuracy near +1
    // where π/2 - asin(x) would not.
    let numerator = sub(Wide::ONE, x);
    if numerator.is_zero() {
        return Wide::ZERO;
    }
    let denominator = add(Wide::ONE, x);
    if denominator.is_zero() {
        return PI;
    }
    atan(sqrt(div(numerator, denominator))).scale2(1)
}

/// `sinh x` and `cosh x`.
fn sinh(x: Wide) -> Wide {
    if x.exp < -2 {
        // The series, so that sinh of a tiny argument is the argument.
        let x2 = mul(x, x);
        let mut term = x;
        let mut sum = x;
        for k in 1..=20usize {
            term = mul(mul(term, x2), mul(recip(2 * k), recip(2 * k + 1)));
            if term.is_zero() {
                break;
            }
            sum = add(sum, term);
        }
        return sum;
    }
    let e = exp(x);
    sub(e, div(Wide::ONE, e)).scale2(-1)
}

fn cosh(x: Wide) -> Wide {
    let e = exp(x.abs());
    add(e, div(Wide::ONE, e)).scale2(-1)
}

/// `tanh x`.
fn tanh(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    if x.exp < -2 {
        let s = sinh(x);
        return div(s, sqrt(add(Wide::ONE, mul(s, s))));
    }
    // tanh(x) = 1 - 2/(e^{2x} + 1), which saturates cleanly for large |x|.
    let sign = x.sign;
    let e = exp(x.abs().scale2(1));
    let out = sub(Wide::ONE, div(Wide::TWO, add(e, Wide::ONE)));
    if sign { out.neg() } else { out }
}

/// `atanh x` for `|x| < 1`.
fn atanh(x: Wide) -> Wide {
    if x.is_zero() {
        return x;
    }
    if x.exp < -3 {
        return atanh_series(x);
    }
    // atanh(x) = ½ ln((1+x)/(1-x)).
    ln(div(add(Wide::ONE, x), sub(Wide::ONE, x))).scale2(-1)
}

/// `sin r` and `cos r` for `|r| <= π/4`, by their Taylor series.
fn sin_cos_small(r: Wide) -> (Wide, Wide) {
    let r2 = mul(r, r);
    let mut term = r;
    let mut sine = r;
    for k in 1..=20usize {
        term = mul(mul(term, r2), mul(recip(2 * k), recip(2 * k + 1))).neg();
        if term.is_zero() {
            break;
        }
        sine = add(sine, term);
    }
    let mut term = Wide::ONE;
    let mut cosine = Wide::ONE;
    for k in 1..=20usize {
        term = mul(mul(term, r2), mul(recip(2 * k - 1), recip(2 * k))).neg();
        if term.is_zero() {
            break;
        }
        cosine = add(cosine, term);
    }
    (sine, cosine)
}

/// `sin x` and `cos x`, with the argument reduced exactly.
fn sin_cos(v: F80) -> (Wide, Wide) {
    let x = from_f80(v);
    let (quadrant, r) = reduce(v, x);
    let (sine, cosine) = sin_cos_small(r);
    match quadrant & 3 {
        0 => (sine, cosine),
        1 => (cosine, sine.neg()),
        2 => (sine.neg(), cosine.neg()),
        _ => (cosine.neg(), sine),
    }
}

/// Round a value to the nearest integer, as an `i64`.
fn round_to_i64(v: Wide) -> i64 {
    if v.is_zero() || v.exp < -1 {
        return 0;
    }
    if v.exp >= 63 {
        return if v.sign { i64::MIN } else { i64::MAX };
    }
    // An exponent of -1 puts the whole significand below the units place, so
    // the truncation is zero and only the rounding bit survives.
    let shift = 127 - v.exp;
    let truncated = if shift >= 128 {
        0
    } else {
        (v.frac >> shift) as i64
    };
    let half = (v.frac >> (shift - 1)) & 1;
    let magnitude = truncated + half as i64;
    if v.sign { -magnitude } else { magnitude }
}

impl Wide {
    fn from_i64(v: i64) -> Wide {
        if v == 0 {
            return Wide::ZERO;
        }
        let out = Wide::from_u64(v.unsigned_abs());
        if v < 0 { out.neg() } else { out }
    }
}

// ---------------------------------------------------------------------------
// Argument reduction
// ---------------------------------------------------------------------------

/// Reduce `v` modulo π/2, exactly.
///
/// Returns the quadrant and a remainder of magnitude at most π/4. The method
/// is Payne and Hanek's: the bits of `2/π` that can affect the answer are
/// exactly those from index `e - 1` downward, because everything above that
/// contributes a multiple of four to `x·2/π` and the quadrant only needs it
/// modulo four. Two hundred and fifty-six of those bits multiplied by the
/// sixty-four-bit significand give the fractional part to well over the
/// hundred and twenty-eight this module carries.
fn reduce(v: F80, x: Wide) -> (u8, Wide) {
    if !x.at_least(PI_4) {
        return (0, x);
    }
    let (sign, m, last) = parts(v);
    // x = m × 2^e with m an integer in [2^63, 2^64).
    let e = last;
    let start = if e >= 2 { e - 1 } else { 1 };
    let p = (start - 1) as usize;
    let word = p / 64;
    let offset = (p % 64) as u32;
    let at = |k: usize| TWO_OVER_PI.get(word + k).copied().unwrap_or(0);
    let mut t = [0u64; 4];
    for (k, slot) in t.iter_mut().enumerate() {
        *slot = if offset == 0 {
            at(k)
        } else {
            (at(k) << offset) | (at(k + 1) >> (64 - offset))
        };
    }
    // R = m × T, three hundred and twenty bits, most significant first.
    let mut r = [0u64; 5];
    let mut carry: u128 = 0;
    for k in (0..4).rev() {
        let product = u128::from(m) * u128::from(t[k]) + carry;
        r[k + 1] = product as u64;
        carry = product >> 64;
    }
    r[0] = carry as u64;
    // The value of `x · 2/π` modulo four is `R / 2^shift`.
    let shift = if e >= 2 {
        254i64
    } else {
        256i64 - i64::from(e)
    };
    if shift >= 320 {
        // Nothing survived: the argument is far below π/4 and is its own
        // remainder, which the early return above has already handled for
        // every argument this can reach.
        return (0, x);
    }
    let shift = shift as i32;
    let quadrant = bits_at(&r, shift + 1, 2) as u8;
    // The fraction, to two hundred and fifty-six bits.
    let mut fraction = [0u64; 4];
    for (k, slot) in fraction.iter_mut().enumerate() {
        *slot = bits_at(&r, shift - 1 - 64 * k as i32, 64) as u64;
    }
    // Above a half the remainder is negative and the quadrant steps on.
    let (quadrant, negative, magnitude) = if fraction[0] >> 63 != 0 {
        (quadrant + 1, true, negate256(fraction))
    } else {
        (quadrant, false, fraction)
    };
    let scaled = normalize256(magnitude);
    let r = mul(scaled, PI_2);
    let r = if negative { r.neg() } else { r };
    if sign {
        (quadrant.wrapping_neg() & 3, r.neg())
    } else {
        (quadrant & 3, r)
    }
}

/// `n` bits of a three-hundred-and-twenty-bit value, the topmost at index
/// `top` counted from the least significant bit.
fn bits_at(r: &[u64; 5], top: i32, n: u32) -> u128 {
    let mut out: u128 = 0;
    for i in 0..n as i32 {
        let bit = top - i;
        let value = if !(0..320).contains(&bit) {
            0
        } else {
            (r[4 - (bit as usize / 64)] >> (bit as usize % 64)) & 1
        };
        out = (out << 1) | u128::from(value);
    }
    out
}

/// `2^256 - v`, on a four-word value.
fn negate256(v: [u64; 4]) -> [u64; 4] {
    let mut out = [0u64; 4];
    let mut borrow = 1u128;
    for k in (0..4).rev() {
        let value = (!v[k]) as u128 + borrow;
        out[k] = value as u64;
        borrow = value >> 64;
    }
    out
}

/// A four-word fraction in `(0, 1)` as a [`Wide`].
fn normalize256(v: [u64; 4]) -> Wide {
    let mut lead = 0u32;
    let mut index = 0usize;
    while index < 4 && v[index] == 0 {
        lead += 64;
        index += 1;
    }
    if index == 4 {
        return Wide::ZERO;
    }
    lead += v[index].leading_zeros();
    // The top 128 bits, starting just below the leading one.
    let bit_of = |i: u32| -> u64 {
        let position = lead + i;
        if position >= 256 {
            0
        } else {
            (v[(position / 64) as usize] >> (63 - position % 64)) & 1
        }
    };
    let mut frac: u128 = 0;
    for i in 0..128 {
        frac = (frac << 1) | u128::from(bit_of(i));
    }
    Wide {
        sign: false,
        exp: -1 - lead as i32,
        frac,
    }
}

// ---------------------------------------------------------------------------
// The instruction set
// ---------------------------------------------------------------------------

/// What a transcendental produced.
#[derive(Debug, Clone, Copy)]
pub(super) struct Transcendental {
    /// The result.
    pub value: F80,
    /// `FSINCOS`'s cosine.
    pub second: Option<F80>,
    /// `FPSR` exception bits the operation itself raises — an operand error
    /// or a divide by zero, which are decided before the arithmetic.
    pub exc: u16,
    /// The flags the rounding reported.
    pub flags: Flags,
}

/// Compute one transcendental, or `None` for an opmode that is not one.
///
/// The special cases are each instruction's own operation table in M68881UM
/// §4 and the two exception lists of §6.1.3 and §6.1.6; everything else goes
/// through [`Wide`].
pub(super) fn compute(op: FpOp, src: F80, spec: Spec, env: Env) -> Option<Transcendental> {
    use super::fpu::{is_infinity, is_zero};
    let sign = src.sign();
    let zero = is_zero(src);
    let infinite = is_infinity(src);
    let x = from_f80(src);

    // The three answers a special case can be.
    let error = |exc: u16| Transcendental {
        value: super::fpu::CREATED_NAN,
        second: None,
        exc,
        flags: Flags::NONE,
    };
    let exact = |value: F80| Transcendental {
        value,
        second: None,
        exc: 0,
        flags: Flags::NONE,
    };
    let rounded = |w: Wide| {
        let (value, flags) = to_f80(w, spec, env);
        Transcendental {
            value,
            second: None,
            exc: 0,
            flags,
        }
    };
    let one = F80::new(0x3fff, 1 << 63);
    let signed_zero = F80::new(if sign { 0x8000 } else { 0 }, 0);
    let infinity = |negative: bool| F80::new(if negative { 0xffff } else { 0x7fff }, 1 << 63);

    Some(match op {
        // The trigonometric three: "Set if the source is ±infinity" is their
        // only operand error.
        FpOp::Sin | FpOp::Cos | FpOp::Tan | FpOp::SinCos => {
            if infinite {
                let mut out = error(super::fpu::bits::OPERR);
                if op == FpOp::SinCos {
                    out.second = Some(super::fpu::CREATED_NAN);
                }
                return Some(out);
            }
            if zero {
                return Some(match op {
                    FpOp::Cos => exact(one),
                    FpOp::SinCos => Transcendental {
                        value: signed_zero,
                        second: Some(one),
                        exc: 0,
                        flags: Flags::NONE,
                    },
                    _ => exact(signed_zero),
                });
            }
            let (sine, cosine) = sin_cos(src);
            match op {
                FpOp::Sin => rounded(sine),
                FpOp::Cos => rounded(cosine),
                FpOp::Tan => rounded(div(sine, cosine)),
                _ => {
                    let (value, flags) = to_f80(sine, spec, env);
                    let (second, more) = to_f80(cosine, spec, env);
                    Transcendental {
                        value,
                        second: Some(second),
                        exc: 0,
                        flags: flags | more,
                    }
                }
            }
        }
        // "Source is ±infinity, >+1, or <-1" is an operand error for both
        // inverse functions (Table 6-2).
        FpOp::Asin | FpOp::Acos => {
            if infinite || !Wide::ONE.at_least(x.abs()) {
                return Some(error(super::fpu::bits::OPERR));
            }
            if op == FpOp::Asin {
                if zero {
                    return Some(exact(signed_zero));
                }
                rounded(asin(x))
            } else {
                rounded(acos(x))
            }
        }
        FpOp::Atan => {
            if infinite {
                let (value, flags) = to_f80(if sign { PI_2.neg() } else { PI_2 }, spec, env);
                return Some(Transcendental {
                    value,
                    second: None,
                    exc: 0,
                    flags,
                });
            }
            if zero {
                return Some(exact(signed_zero));
            }
            rounded(atan(x))
        }
        // "Source is > +1, or < -1, Source = ±infinity" is the operand error;
        // a source of exactly ±1 is a divide by zero returning ±infinity.
        FpOp::Atanh => {
            if infinite || !Wide::ONE.at_least(x.abs()) {
                return Some(error(super::fpu::bits::OPERR));
            }
            if sub(x.abs(), Wide::ONE).is_zero() {
                // §6.1.6 lists FATANH among the divide-by-zero cases. Its
                // sentence there names the opposite sign from the one the
                // function has — atanh approaches +infinity from below +1 —
                // and this core computes the mathematical sign, which
                // `docs/cpu/m68k.md` records.
                return Some(Transcendental {
                    value: infinity(sign),
                    second: None,
                    exc: super::fpu::bits::DZ,
                    flags: Flags::NONE,
                });
            }
            if zero {
                return Some(exact(signed_zero));
            }
            rounded(atanh(x))
        }
        FpOp::Sinh => {
            if infinite || zero {
                return Some(exact(src));
            }
            rounded(sinh(x))
        }
        FpOp::Cosh => {
            if infinite {
                return Some(exact(infinity(false)));
            }
            if zero {
                return Some(exact(one));
            }
            rounded(cosh(x))
        }
        FpOp::Tanh => {
            if infinite {
                return Some(exact(F80::new(if sign { 0xbfff } else { 0x3fff }, 1 << 63)));
            }
            if zero {
                return Some(exact(signed_zero));
            }
            rounded(tanh(x))
        }
        // The exponentials have no operand errors at all: an infinite
        // argument is a legitimate limit.
        FpOp::Etox | FpOp::TwoToX | FpOp::TenToX => {
            if infinite {
                return Some(exact(if sign { F80::ZERO } else { infinity(false) }));
            }
            if zero {
                return Some(exact(one));
            }
            let argument = match op {
                FpOp::Etox => x,
                FpOp::TwoToX => mul(x, LN2),
                _ => mul(x, LN10),
            };
            rounded(exp(argument))
        }
        FpOp::EtoxM1 => {
            if infinite {
                return Some(exact(if sign {
                    F80::new(0xbfff, 1 << 63)
                } else {
                    infinity(false)
                }));
            }
            if zero {
                return Some(exact(signed_zero));
            }
            rounded(expm1(x))
        }
        // "Source is <0, Source = -infinity" is the operand error; a zero
        // source is a divide by zero returning minus infinity (§6.1.6).
        FpOp::Logn | FpOp::Log2 | FpOp::Log10 => {
            if sign && !zero {
                return Some(error(super::fpu::bits::OPERR));
            }
            if zero {
                return Some(Transcendental {
                    value: infinity(true),
                    second: None,
                    exc: super::fpu::bits::DZ,
                    flags: Flags::NONE,
                });
            }
            if infinite {
                return Some(exact(infinity(false)));
            }
            let natural = ln(x);
            rounded(match op {
                FpOp::Logn => natural,
                FpOp::Log2 => div(natural, LN2),
                _ => div(natural, LN10),
            })
        }
        // "Source is < -1, Source = -infinity" for the one that takes its
        // argument shifted by one; a source of exactly -1 is the divide by
        // zero.
        FpOp::LognP1 => {
            if infinite {
                if sign {
                    return Some(error(super::fpu::bits::OPERR));
                }
                return Some(exact(infinity(false)));
            }
            if zero {
                return Some(exact(signed_zero));
            }
            let plus_one = add(Wide::ONE, x);
            if plus_one.sign {
                return Some(error(super::fpu::bits::OPERR));
            }
            if plus_one.is_zero() {
                return Some(Transcendental {
                    value: infinity(true),
                    second: None,
                    exc: super::fpu::bits::DZ,
                    flags: Flags::NONE,
                });
            }
            rounded(ln1p(x))
        }
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `Wide` of an `F80` built from an exponent field and a
    /// significand.
    fn wide(field: u16, sig: u64) -> Wide {
        from_f80(F80::new(field, sig))
    }

    #[test]
    fn the_wide_arithmetic_is_exact_where_it_can_be() {
        let one = wide(0x3fff, 1 << 63);
        let two = wide(0x4000, 1 << 63);
        let three = wide(0x4000, 0xc000_0000_0000_0000);
        assert_eq!(add(one, two), three);
        assert_eq!(sub(three, two), one);
        assert_eq!(mul(two, three), wide(0x4001, 0xc000_0000_0000_0000));
        assert_eq!(div(three, two), wide(0x3fff, 0xc000_0000_0000_0000));
        assert_eq!(sub(one, one), Wide::ZERO);
        assert_eq!(mul(one, Wide::ZERO), Wide::ZERO);
        // A quarter of a part in 2^127 survives a round trip through the
        // significand, which is the whole point of carrying twice the width.
        let tiny = Wide {
            sign: false,
            exp: 0,
            frac: (1u128 << 127) | 1,
        };
        assert_ne!(sub(tiny, one), Wide::ZERO);
    }

    #[test]
    fn the_square_root_agrees_with_the_shared_kernel() {
        // `src/float`'s own square root is correctly rounded; this one only
        // has to match it to sixty-four bits, which is what the conversion
        // keeps.
        for (field, sig) in [
            (0x4000u16, 1u64 << 63),
            (0x4003, 1 << 63),
            (0x3fff, 0xc000_0000_0000_0000),
            (0x4020, 0x9000_0000_0000_0000),
            (0x3f00, 1 << 63),
        ] {
            let v = F80::new(field, sig);
            let (reference, _) = x87::sqrt(v, crate::float::x87::Precision::Extended, Env::X87);
            let (mine, _) = to_f80(sqrt(from_f80(v)), F80::SPEC, Env::X87);
            assert_eq!(mine, reference, "sqrt of {field:04x}:{sig:016x}");
        }
    }

    #[test]
    fn the_reduction_is_exact_for_a_huge_argument() {
        // 2^100 is far past the 10^20 at which the manual says a 68881 has
        // "lost all accuracy"; the quadrant and the remainder here come from
        // sixteen thousand bits of 2/π, so they are the true ones.
        let v = F80::new(0x3fff + 100, 1 << 63);
        let (quadrant, r) = reduce(v, from_f80(v));
        assert!(quadrant < 4);
        assert!(PI_4.at_least(r.abs()), "the remainder is inside a quadrant");
        // sin² + cos² = 1 is the cheap invariant that catches a botched
        // reduction.
        let (s, c) = sin_cos(v);
        let sum = add(mul(s, s), mul(c, c));
        let (rounded, _) = to_f80(sum, F80::SPEC, Env::X87);
        assert_eq!(rounded, F80::new(0x3fff, 1 << 63));
    }
}
