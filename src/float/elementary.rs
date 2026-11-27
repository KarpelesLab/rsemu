//! Correctly rounded elementary functions on the 80-bit format: the x87
//! transcendentals.
//!
//! `FSIN`, `FCOS`, `FSINCOS`, `FPTAN`, `FPATAN`, `F2XM1`, `FYL2X` and
//! `FYL2XP1` have no exact answer to compute, and the architecture specifies
//! an accuracy *bound* rather than a bit pattern (*Intel SDM* volume 1
//! §8.3.10: under one ulp to nearest on the Pentium and later, and real parts
//! differ from one another in the last bit). This module delivers the one
//! answer that satisfies that bound on every part at once: the **correctly
//! rounded** value — the exact mathematical result rounded once, in the
//! current direction, to the requested precision. That is also the only
//! answer that is a function of the operands alone, which is the property
//! `ROADMAP.md` §9.1 asks of guest floating point: two hosts compute the same
//! bits because there is exactly one right set of bits.
//!
//! # How a correctly rounded result is reached
//!
//! The table-maker's dilemma: a transcendental value can lie arbitrarily
//! close to a rounding boundary, so no fixed working precision is enough for
//! every input. The cure is Ziv's strategy (A. Ziv, "Fast evaluation of
//! elementary mathematical functions with correctly rounded last bit", *ACM
//! TOMS* 17(3), 1991; J.-M. Muller, *Elementary Functions*, 3rd ed., §12):
//!
//! 1. compute an **enclosure** `[v - e, v + e]` of the true value at a working
//!    precision `w`, with `e` a rigorous bound rather than an estimate;
//! 2. round both ends; if they round to the same result, with the same
//!    exception flags and the same direction, so does everything between them
//!    — rounding is monotone — and in particular the true value;
//! 3. otherwise double `w` and go again.
//!
//! The enclosures come from `float::mp` (private), whose every operation carries an
//! exact integer error bound (see its documentation for the per-operation
//! inequalities). Step 2 compares the ends *strictly outside* the enclosure
//! — `2(v - e) - 1` and `2(v + e) + 1` half-units — so a true value lying
//! exactly on an end is still covered.
//!
//! The loop terminates because, away from the handful of exact cases each
//! function treats first, every result is **irrational**: `sin x`, `cos x`,
//! `tan x` and `atan x` for rational `x != 0` (Lindemann–Weierstrass),
//! `log2 x` for rational `x` not a power of two, and `2^x` for rational
//! non-integral `x` (Gelfond–Schneider). An irrational value is never *on* a
//! boundary, so some finite precision separates it from every boundary. The
//! precision is capped at `W_MAX` bits so the loop is total; reaching the
//! cap would need a 64-bit operand whose result agrees with a rounding
//! boundary to more than eight thousand bits, where the worst cases known for
//! these functions are around twice the format width (Muller §12.3). Should it
//! happen anyway, the result is rounded from that 8192-bit enclosure — wrong
//! by at most one ulp, and still within the architectural bound.
//!
//! # The near-exact cases
//!
//! Ziv's loop cannot finish when the result is extremely close to a
//! representable number *by construction* rather than by chance: `sin x`
//! for `|x| = 2^-1000` is `x` minus about `2^-3000`, and in a directed
//! rounding mode `x` itself is the boundary. Those are argued once, in
//! closed form, by `beside`: when the true value is known to lie strictly
//! within `2^(e-8)` of an exactly representable `x` on a known side, every
//! number in that gap rounds the same way, so a representative of the gap is
//! rounded instead. The thresholds (`|x| < 2^-40` for `sin`, `tan`, `cos`;
//! `|y/x| < 2^-72` for `atan`) are chosen so the Taylor remainder is far
//! inside that gap — each call site gives the inequality.
//!
//! # Range reduction
//!
//! The trigonometric functions reduce `x` by the nearest multiple of `pi/2`
//! with `pi` computed to `w + 16` bits *beyond* the magnitude of `x`, so the
//! reduced argument is accurate to the working precision however large `x`
//! is (the Payne–Hanek observation, done here by brute precision rather than
//! a table). `pi` is Machin's formula, `16 atan(1/5) - 4 atan(1/239)`, and
//! `ln 2` is `2 atanh(1/3)`, both evaluated in the error-tracked fixed point
//! at whatever precision the attempt needs, so there is no stored constant
//! whose last digit anything depends on. The tests check both against
//! independently computed digits.
//!
//! **This is not what Intel hardware does.** An x87 reduces against a 66-bit
//! approximation of `pi` (*Intel SDM* volume 1 §8.3.10, "Pi"), so for a large
//! argument near a multiple of `pi` its `FSIN` is a correct rounding of
//! `sin(x * pi / pi66)` rather than of `sin x`. This module computes `sin x`.
//! The guest-visible difference is confined to such arguments, where the
//! hardware's own documentation calls its answer inaccurate.
//!
//! # What precision control does
//!
//! Every function takes a [`Precision`] and rounds to it, with the 80-bit
//! exponent range. On x86 the caller always passes
//! [`Precision::Extended`]: precision control applies to a **closed** list of
//! instructions — add, subtract, multiply, divide and square root (*Intel SDM*
//! volume 1 §8.1.5.2) — and no transcendental is on it.
//!
//! # Sources
//!
//! *Intel SDM* volume 2, the instruction pages for the special-operand tables
//! and domains; volume 1 §8.3.10 for the accuracy contract. Ziv and Muller as
//! above for the method; the series are the textbook Taylor expansions of
//! `sin`, `cos`, `atan`, `atanh` and `exp`. No library or emulator source of
//! any licence was consulted (`CLAUDE.md`, provenance).

use core::cmp::Ordering;

use super::kernel::{self, Class, Outcome, Parts, Rounded};
use super::mp::{Ax, Big};
use super::x87::{F80, Precision, encode, unpack};
use super::{Env, Flags, Spec};

/// A rounded transcendental result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Elementary {
    /// The result.
    pub value: F80,
    /// The exceptions it raised, the denormal-operand flag included.
    pub flags: Flags,
    /// Whether the delivered magnitude is **greater** than the exact one —
    /// x87's `C1` after an inexact result ("rounded up", *Intel SDM* volume 1
    /// §8.1.3.1). False for an exact result.
    pub rounded_up: bool,
}

/// The first working precision, in bits below the binary point.
///
/// Comfortably more than the 64-bit format needs, so the first attempt almost
/// always decides; doubling after that.
const W0: u32 = 128;

/// The last working precision Ziv's loop tries. See the module
/// documentation for why reaching it is not expected and what happens if it
/// is.
const W_MAX: u32 = 8192;

/// Guard bits below the last place of a near-exact operand: [`beside`]'s gap
/// is `2^(e - GAP)` wide.
const GAP: u32 = 8;

/// An enclosure of a result: the true value lies in
/// `(-1)^neg * [mag - err, mag + err] * 2^exp`.
#[derive(Debug)]
struct Enclosure {
    neg: bool,
    mag: Big,
    err: Big,
    exp: i64,
}

impl Enclosure {
    /// A fixed-point enclosure at scale `2^-w`.
    fn fixed(a: Ax, w: u32) -> Enclosure {
        Enclosure {
            neg: a.neg,
            mag: a.mag,
            err: a.err,
            exp: -i64::from(w),
        }
    }

    /// `a * k * 2^exp` for an exact integer `k`: the bound scales with it.
    fn scaled(neg: bool, a: &Ax, k: &Big, exp: i64) -> Enclosure {
        Enclosure {
            neg,
            mag: a.mag.mul(k),
            err: a.err.mul(k),
            exp,
        }
    }
}

// ---------------------------------------------------------------------------
// Rounding an enclosure
// ---------------------------------------------------------------------------

/// Round the exact value `±sig * 2^exp` (`sig != 0`) and report whether the
/// rounded magnitude exceeds it.
fn round_point(neg: bool, sig: &Big, exp: i64, spec: Spec, env: Env) -> (Outcome, Flags, bool) {
    // `round_exact` takes a `u128`; anything wider is folded down to 126 bits
    // with a sticky bit, which preserves the rounding of any precision up to
    // 123 bits in every direction.
    let bits = sig.bits();
    let (s, e) = if bits > 126 {
        let n = bits - 126;
        let (q, lost) = sig.shr(n);
        (q.to_u128().unwrap_or(0) | u128::from(lost), exp + n as i64)
    } else {
        (sig.to_u128().unwrap_or(0), exp)
    };
    // Far outside the format either way, a clamp changes nothing that
    // rounding can see.
    let e = e.clamp(-(1 << 24), 1 << 24) as i32;
    let (out, flags) = kernel::round_exact(neg, e, s, spec, env);
    (out, flags, magnitude_above(out, s, e))
}

/// Whether `out`'s magnitude is greater than `sig * 2^exp`.
fn magnitude_above(out: Outcome, sig: u128, exp: i32) -> bool {
    match out {
        Outcome::Num(_, Rounded::Inf) => true,
        Outcome::Num(_, Rounded::Finite { exp: re, frac }) => {
            let a = u128::from(frac);
            let ta = i64::from(re) + i64::from(128 - a.leading_zeros());
            let tb = i64::from(exp) + i64::from(128 - sig.leading_zeros());
            if ta != tb {
                return ta > tb;
            }
            // Same top bit: shift the one with the larger exponent up to the
            // other's, which cannot overflow because the two now span the
            // same bits.
            if re >= exp {
                (a << (re - exp) as u32) > sig
            } else {
                a > (sig << (exp - re) as u32)
            }
        }
        _ => false,
    }
}

/// Decide an enclosure, or `None` when its ends round differently.
fn decide(enc: &Enclosure, spec: Spec, env: Env) -> Option<(Outcome, Flags, bool)> {
    if enc.mag.is_zero() {
        return None;
    }
    if enc.err.is_zero() {
        return Some(round_point(enc.neg, &enc.mag, enc.exp, spec, env));
    }
    if enc.mag.cmp(&enc.err) != Ordering::Greater {
        return None;
    }
    // Strictly outside both ends, at half-unit resolution.
    let lo = enc.mag.sub(&enc.err).shl(1).sub(&Big::from_u64(1));
    let hi = enc.mag.add(&enc.err).shl(1).add_u64(1);
    let a = round_point(enc.neg, &lo, enc.exp - 1, spec, env);
    let b = round_point(enc.neg, &hi, enc.exp - 1, spec, env);
    (a == b).then_some(a)
}

/// The parameters a transcendental rounds to at a given [`Precision`]: that
/// many significand bits on the 80-bit format's **own** subnormal grid.
///
/// Deliberately not [`Precision::spec`], which is what `FADD`/`FSUB`/`FMUL`/
/// `FDIV`/`FSQRT` do under precision control and coarsens the grid with the
/// precision. No x87 transcendental honours precision control at all — the
/// only caller passes [`Precision::Extended`], where the two agree — so the
/// reduced precisions here are this module's own generality, and the vectors
/// in `elementary/tests.rs` were computed for this definition of it.
fn rounding_spec(pc: Precision) -> Spec {
    F80::SPEC.with_precision(pc.bits())
}

/// Ziv's loop: evaluate at increasing precision until the enclosure decides.
fn ziv(
    pc: Precision,
    env: Env,
    extra: Flags,
    mut eval: impl FnMut(u32) -> Option<Enclosure>,
) -> Elementary {
    let spec = rounding_spec(pc);
    let mut w = W0;
    loop {
        let enc = eval(w);
        if let Some(enc) = &enc
            && let Some((out, flags, up)) = decide(enc, spec, env)
        {
            return finish(out, flags | extra, up, env);
        }
        if w >= W_MAX {
            // The documented, not expected, exit: round the 8192-bit
            // approximation itself.
            if let Some(enc) = enc
                && !enc.mag.is_zero()
            {
                let (out, flags, up) = round_point(enc.neg, &enc.mag, enc.exp, spec, env);
                return finish(out, flags | Flags::INEXACT | extra, up, env);
            }
            return invalid(env, extra);
        }
        w *= 2;
    }
}

/// Encode a rounded outcome.
fn finish(out: Outcome, flags: Flags, rounded_up: bool, env: Env) -> Elementary {
    Elementary {
        value: encode(out, env),
        flags,
        rounded_up,
    }
}

/// The value just beside an exactly representable `±m * 2^e` (`m != 0`),
/// on the side `above` names, rounded.
///
/// The claim this rests on: let `m` be normalised to bit 63. At any precision
/// up to 64 bits, every rounding boundary near `m * 2^e` — every representable
/// number and every midpoint, in the binade of `m` or in the one below it, on
/// the subnormal grid or off it — is a multiple of `2^(e-2)`. So the open
/// interval of width `2^(e - GAP)` on either side of `m * 2^e` contains no
/// boundary, every value in it rounds identically, and the midpoint of the
/// interval may stand for all of them. A caller must show the true value is
/// in that interval: `|f(x) - m 2^e| < 2^(e-GAP)`, which with `m < 2^64`
/// follows from a relative distance below `2^-(64 + GAP)` = `2^-72`.
fn beside(
    neg: bool,
    m: u64,
    e: i64,
    above: bool,
    pc: Precision,
    env: Env,
    extra: Flags,
) -> Elementary {
    let base = Big::from_u64(m).shl(u64::from(GAP) + 1);
    let rep = if above {
        base.add_u64(1)
    } else {
        base.sub(&Big::from_u64(1))
    };
    let (out, flags, up) = round_point(neg, &rep, e - i64::from(GAP) - 1, rounding_spec(pc), env);
    finish(out, flags | extra, up, env)
}

/// Round an exactly known `±sig * 2^exp`, which may be zero.
fn exact_value(
    neg: bool,
    sig: &Big,
    exp: i64,
    pc: Precision,
    env: Env,
    extra: Flags,
) -> Elementary {
    if sig.is_zero() {
        return special(Outcome::Num(neg, Rounded::Zero), extra, env);
    }
    let (out, flags, up) = round_point(neg, sig, exp, rounding_spec(pc), env);
    finish(out, flags | extra, up, env)
}

/// A result that needs no rounding.
fn special(out: Outcome, flags: Flags, env: Env) -> Elementary {
    finish(out, flags, false, env)
}

/// The invalid-operation result: the default NaN, the indefinite on x87.
fn invalid(env: Env, extra: Flags) -> Elementary {
    special(Outcome::DefaultNan, Flags::INVALID | extra, env)
}

/// A signed infinity.
fn inf(neg: bool, flags: Flags, env: Env) -> Elementary {
    special(Outcome::Num(neg, Rounded::Inf), flags, env)
}

/// A signed zero.
fn zero(neg: bool, flags: Flags, env: Env) -> Elementary {
    special(Outcome::Num(neg, Rounded::Zero), flags, env)
}

/// The NaN result for a set of operands at least one of which is a NaN.
fn nan(ops: &[Parts], env: Env, extra: Flags) -> Elementary {
    match kernel::nan_result(ops, env) {
        Some((out, flags)) => special(out, flags | extra, env),
        None => invalid(env, extra),
    }
}

/// A finite non-zero operand as `(m, e)` with `m` normalised to bit 63 and
/// the value `m * 2^e`.
fn norm(p: &Parts) -> (u64, i64) {
    let lz = p.frac.leading_zeros();
    (p.frac << lz, i64::from(p.exp) - i64::from(lz))
}

/// `m * 2^e` at scale `2^-w`, exact where the scale allows and truncated
/// (with its bound) where it does not.
fn fixed_of(neg: bool, m: u64, e: i64, w: u32) -> Ax {
    let a = Ax::exact(neg, Big::from_u64(m));
    let shift = e + i64::from(w);
    if shift >= 0 {
        a.shl(shift as u64)
    } else {
        a.shr(shift.unsigned_abs())
    }
}

// ---------------------------------------------------------------------------
// Constants and series
// ---------------------------------------------------------------------------

/// Whether a series term is small enough to stop at: at most four units.
fn negligible(t: &Ax) -> bool {
    t.bound().cmp(&Big::from_u64(4)) != Ordering::Greater
}

/// `atan(1/n)` for an integer `n >= 2`: `sum (-1)^k / ((2k+1) n^(2k+1))`.
///
/// Alternating with decreasing terms, so the tail is bounded by its first
/// term, whose bound is added to the error when the loop stops.
fn atan_recip(n: u64, w: u32) -> Ax {
    let n2 = n * n;
    let mut p = Ax::int(1, w).div_u64(n);
    let mut sum = p.clone();
    let mut k = 1u64;
    loop {
        p = p.div_u64(n2);
        let t = p.div_u64(2 * k + 1);
        if negligible(&t) {
            sum.widen(&t.bound());
            return sum;
        }
        sum = if k % 2 == 1 { sum.sub(&t) } else { sum.add(&t) };
        k += 1;
    }
}

/// `pi`, by Machin's formula `16 atan(1/5) - 4 atan(1/239)`, at scale `w`.
///
/// Evaluated sixteen bits deeper and shifted down, so the error the series
/// accumulates (a few units per term) is a unit or two at the scale asked
/// for.
pub(super) fn pi(w: u32) -> Ax {
    let wi = w + 16;
    let v = atan_recip(5, wi)
        .mul_u64(16)
        .sub(&atan_recip(239, wi).mul_u64(4));
    v.shr(16)
}

/// `ln 2 = 2 atanh(1/3) = 2 sum 1 / ((2k+1) 3^(2k+1))`, at scale `w`.
///
/// Positive terms falling by at least a factor of nine, so the tail is under
/// `9/8` of its first term; twice that is added.
pub(super) fn ln2(w: u32) -> Ax {
    let wi = w + 16;
    let mut p = Ax::int(1, wi).div_u64(3);
    let mut sum = p.clone();
    let mut k = 1u64;
    loop {
        p = p.div_u64(9);
        let t = p.div_u64(2 * k + 1);
        if negligible(&t) {
            sum.widen(&t.bound().mul_u64(2));
            break;
        }
        sum = sum.add(&t);
        k += 1;
    }
    sum.mul_u64(2).shr(16)
}

/// Reduce `m * 2^e` (`|x| >= 2^-41`) by the nearest multiple `k` of `pi/2`:
/// `(k mod 4, x - k pi/2)`, the second at scale `w`.
///
/// `x` is held exactly at scale `wr = w + 16 + max(0, e + 64)` and `pi/2` to
/// the same scale, so `k * (pi/2)` is off by at most `k * 2^(1-wr)`, and
/// `k < 2^(e + 64)` makes that below `2^(-w-14)`: negligible after the shift
/// back to scale `w`, and tracked anyway. `k` itself need only be *near* the
/// nearest multiple — the identity `x = k pi/2 + r` is exact for any `k` — so
/// computing it from the approximate `pi/2` costs nothing in accuracy.
fn reduce(m: u64, e: i64, w: u32) -> (u64, Ax) {
    let extra = (e + 64).max(0) as u32 + 16;
    let wr = w + extra;
    let x = Big::from_u64(m).shl((e + i64::from(wr)) as u64);
    let half_pi = pi(wr).shr(1);
    let (k, _) = x.add(&half_pi.mag.shr(1).0).div_rem(&half_pi.mag);
    let r = Ax::exact(false, x).sub(&half_pi.mul_big(&k));
    (k.low_u64() & 3, r.shr(u64::from(extra)))
}

/// `(sin r, cos r)` for `|r| <= pi/4 + 2^-100`, at scale `w`.
///
/// Both Taylor series alternate with decreasing terms there, so each stops
/// with its first omitted term's bound added to the error.
fn sin_cos_series(r: &Ax, w: u32) -> (Ax, Ax) {
    let r2 = r.mul(r, w);
    let mut t = r.clone();
    let mut s = r.clone();
    let mut k = 1u64;
    loop {
        t = t.mul(&r2, w).div_u64((2 * k) * (2 * k + 1));
        if negligible(&t) {
            s.widen(&t.bound());
            break;
        }
        s = if k % 2 == 1 { s.sub(&t) } else { s.add(&t) };
        k += 1;
    }
    let mut t = Ax::int(1, w);
    let mut c = t.clone();
    let mut k = 1u64;
    loop {
        t = t.mul(&r2, w).div_u64((2 * k - 1) * (2 * k));
        if negligible(&t) {
            c.widen(&t.bound());
            break;
        }
        c = if k % 2 == 1 { c.sub(&t) } else { c.add(&t) };
        k += 1;
    }
    (s, c)
}

/// `atan t` for `|t| <= 1/2`: `sum (-1)^k t^(2k+1) / (2k+1)`, alternating
/// and falling by at least four per term.
fn atan_small(t: &Ax, w: u32) -> Ax {
    let t2 = t.mul(t, w);
    let mut p = t.clone();
    let mut sum = t.clone();
    let mut k = 1u64;
    loop {
        p = p.mul(&t2, w);
        let term = p.div_u64(2 * k + 1);
        if negligible(&term) {
            sum.widen(&term.bound());
            return sum;
        }
        sum = if k % 2 == 1 {
            sum.sub(&term)
        } else {
            sum.add(&term)
        };
        k += 1;
    }
}

/// `atan q` for `0 <= q <= 1`. Above one half, `atan q = pi/4 + atan t`
/// with `t = (q - 1) / (q + 1)` in `[-1/3, 0]`.
fn atan_unit(q: &Ax, pi: &Ax, w: u32) -> Option<Ax> {
    let one = Ax::int(1, w);
    if q.mag.cmp(&one.mag.shr(1).0) == Ordering::Greater {
        let t = q.sub(&one).div(&q.add(&one), w)?;
        Some(pi.shr(2).add(&atan_small(&t, w)))
    } else {
        Some(atan_small(q, w))
    }
}

/// `log2(1 + z) / z` for `-1/2 <= z <= 1`, `z != 0` (its limit at zero is
/// fine too), at scale `w`.
///
/// `ln(1 + z) = 2 atanh(s)` with `s = z / (2 + z)`, so
/// `ln(1 + z) / z = 2/(2 + z) * sum s^(2k) / (2k + 1)`, and
/// `|s| <= 1/3` makes the terms positive and falling by at least nine — the
/// tail is under `9/8` of its first term. Dividing by `ln 2` changes base.
/// The quotient is near one, so this is accurate *relative* to the result
/// however small `z` is; the callers multiply by the exact `z` afterwards.
fn log2_ratio(z: &Ax, ln2: &Ax, w: u32) -> Option<Ax> {
    let two = Ax::int(2, w);
    let d = two.add(z);
    let s = z.div(&d, w)?;
    let s2 = s.mul(&s, w);
    let mut sum = Ax::int(1, w);
    let mut p = sum.clone();
    let mut k = 1u64;
    loop {
        p = p.mul(&s2, w);
        let t = p.div_u64(2 * k + 1);
        if negligible(&t) {
            sum.widen(&t.bound().mul_u64(2));
            break;
        }
        sum = sum.add(&t);
        k += 1;
    }
    two.div(&d, w)?.mul(&sum, w).div(ln2, w)
}

/// `(2^f - 1) / f` for `|f| <= 1`, at scale `w`: `ln 2 * sum t^k / (k+1)!`
/// with `t = f ln 2`.
///
/// `|t| < 0.7`, so after the first term each falls by at least a factor of
/// `0.7 / 3`; the tail is under `1/(1 - 0.35)` of its first term and twice
/// the term's bound is added. The ratio lies in `[1/2, 1]`, so as with
/// [`log2_ratio`] the accuracy is relative to the final product.
fn exp2m1_ratio(f: &Ax, ln2: &Ax, w: u32) -> Ax {
    let t = f.mul(ln2, w);
    let mut sum = Ax::int(1, w);
    let mut p = sum.clone();
    let mut k = 1u64;
    loop {
        p = p.mul(&t, w).div_u64(k + 1);
        if negligible(&p) {
            sum.widen(&p.bound().mul_u64(2));
            break;
        }
        sum = sum.add(&p);
        k += 1;
    }
    ln2.mul(&sum, w)
}

/// `log2` of a positive `big * 2^e`: an exact integer when it is a power of
/// two, otherwise an enclosure at scale `w`.
enum Log2 {
    /// `log2` is this integer.
    Exact(i64),
    /// An enclosure of `log2` at scale `w`, or `None` if the attempt failed.
    Approx(Option<Ax>),
}

/// `log2(big * 2^e)` as `n + log2(1 + z)` with `1 + z` in `[sqrt(1/2),
/// sqrt 2)`.
///
/// `z` is exact whenever `big` fits the scale, which a 64-bit operand always
/// does at `w >= 64`. When `n = 0` the result is as small as `z` and a
/// fixed-point enclosure is only as good as `w` is deep, which Ziv's loop
/// supplies: `|z| >= 2^-64` for a 64-bit operand.
fn log2_of(big: &Big, e: i64, ln2: Option<&Ax>, w: u32) -> Log2 {
    let bm = big.bits();
    let n0 = e + bm as i64 - 1;
    if big.is_pow2() {
        return Log2::Exact(n0);
    }
    let Some(ln2) = ln2 else {
        return Log2::Approx(None);
    };
    // f = big / 2^(bm - 1) in [1, 2); compare f^2 with 2 exactly.
    let above = big.mul(big).cmp(&Big::pow2(2 * bm - 1)) == Ordering::Greater;
    let (n, top) = if above { (n0 + 1, bm) } else { (n0, bm - 1) };
    // f' = big / 2^top, and z = f' - 1 at scale w.
    let a = Ax::exact(false, big.clone());
    let fw = if u64::from(w) >= top {
        a.shl(u64::from(w) - top)
    } else {
        a.shr(top - u64::from(w))
    };
    let z = fw.sub(&Ax::int(1, w));
    let Some(h) = log2_ratio(&z, ln2, w) else {
        return Log2::Approx(None);
    };
    let zh = z.mul(&h, w);
    let n_ax = Ax::exact(n < 0, Big::from_u64(n.unsigned_abs()).shl(u64::from(w)));
    Log2::Approx(Some(n_ax.add(&zh)))
}

// ---------------------------------------------------------------------------
// The functions
// ---------------------------------------------------------------------------

/// Which trigonometric function [`trig`] computes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Trig {
    Sin,
    Cos,
    Tan,
}

/// `sin x` — `FSIN`.
///
/// Special operands (*Intel SDM* volume 2, `FSIN`): `±0` is returned
/// unchanged, `±inf` is invalid, a NaN propagates. The `|x| >= 2^63` range
/// check is the instruction's, not the function's, and is the caller's to
/// make; this computes `sin x` for any finite `x`.
#[must_use]
pub fn sin(a: F80, pc: Precision, env: Env) -> Elementary {
    trig(a, Trig::Sin, pc, env)
}

/// `cos x` — `FCOS`. `cos ±0 = +1` exactly.
#[must_use]
pub fn cos(a: F80, pc: Precision, env: Env) -> Elementary {
    trig(a, Trig::Cos, pc, env)
}

/// `tan x` — the value `FPTAN` leaves under the `1.0` it pushes.
#[must_use]
pub fn tan(a: F80, pc: Precision, env: Env) -> Elementary {
    trig(a, Trig::Tan, pc, env)
}

/// The three trigonometric functions.
fn trig(a: F80, which: Trig, pc: Precision, env: Env) -> Elementary {
    let (p, fa) = unpack(a, env);
    let Some(p) = p else {
        return invalid(env, fa);
    };
    match p.class {
        Class::Nan => nan(&[p], env, fa),
        Class::Inf => invalid(env, fa),
        Class::Zero => match which {
            Trig::Cos => exact_value(false, &Big::from_u64(1), 0, pc, env, fa),
            _ => zero(p.sign, fa, env),
        },
        Class::Finite => {
            let (m, e) = norm(&p);
            if e + 63 < -40 {
                // |x| < 2^-40. sin x = x(1 - d) with 0 < d < x^2/6 < 2^-81;
                // tan x = x(1 + d) with 0 < d < x^2/2 < 2^-81; and
                // 1 - cos x < x^2/2 < 2^-81. All under the 2^-72 `beside`
                // needs.
                return match which {
                    Trig::Sin => beside(p.sign, m, e, false, pc, env, fa),
                    Trig::Tan => beside(p.sign, m, e, true, pc, env, fa),
                    Trig::Cos => beside(false, 1 << 63, -63, false, pc, env, fa),
                };
            }
            ziv(pc, env, fa, |w| {
                let (q, r) = reduce(m, e, w);
                let (s, c) = sin_cos_series(&r, w);
                // x = k pi/2 + r with k = q (mod 4).
                let v = match which {
                    Trig::Sin => match q {
                        0 => s,
                        1 => c,
                        2 => s.negated(),
                        _ => c.negated(),
                    },
                    Trig::Cos => match q {
                        0 => c,
                        1 => s.negated(),
                        2 => c.negated(),
                        _ => s,
                    },
                    Trig::Tan => {
                        if q % 2 == 0 {
                            s.div(&c, w)?
                        } else {
                            c.div(&s, w)?.negated()
                        }
                    }
                };
                // sin and tan are odd, cos is even.
                let v = if p.sign && which != Trig::Cos {
                    v.negated()
                } else {
                    v
                };
                Some(Enclosure::fixed(v, w))
            })
        }
    }
}

/// `2^x - 1` — `F2XM1`.
///
/// The instruction's domain is `-1 <= x <= +1` and the manual leaves the
/// result outside it undefined; this computes the correctly rounded `2^x - 1`
/// everywhere, which is one of the answers "undefined" permits and the only
/// one that needs no further justification. `+inf` gives `+inf` and `-inf`
/// gives `-1`, exactly, as the manual's table says.
#[must_use]
pub fn exp2m1(a: F80, pc: Precision, env: Env) -> Elementary {
    let (p, fa) = unpack(a, env);
    let Some(p) = p else {
        return invalid(env, fa);
    };
    match p.class {
        Class::Nan => nan(&[p], env, fa),
        Class::Inf if p.sign => exact_value(true, &Big::from_u64(1), 0, pc, env, fa),
        Class::Inf => inf(false, fa, env),
        Class::Zero => zero(p.sign, fa, env),
        Class::Finite => {
            let (m, e) = norm(&p);
            let top = e + 63;
            if top < 0 {
                // |x| < 1: 2^x - 1 = x * K(x), K in [1/2, 1], so the
                // enclosure is relative to the result however small x is.
                return ziv(pc, env, fa, |w| {
                    let x = fixed_of(p.sign, m, e, w);
                    let k = exp2m1_ratio(&x, &ln2(w), w);
                    Some(Enclosure::scaled(
                        p.sign,
                        &k,
                        &Big::from_u64(m),
                        e - i64::from(w),
                    ))
                });
            }
            exp2m1_wide(p.sign, m, e, pc, env, fa)
        }
    }
}

/// `2^x - 1` for `|x| >= 1`.
fn exp2m1_wide(neg: bool, m: u64, e: i64, pc: Precision, env: Env, fa: Flags) -> Elementary {
    // Far enough out the answer is decided without computing anything:
    // above 2^16384 every value overflows alike, and below -1 + 2^-200 every
    // value is within `beside`'s gap of -1.
    let overflow = |env: Env| exact_value(false, &Big::from_u64(1), 1 << 20, pc, env, fa);
    let near_minus_one = |env: Env| beside(true, 1 << 63, -63, false, pc, env, fa);
    // x = ±(n + g) with n integral and g in [0, 1) exact at 64 bits.
    // |x| >= 1 puts the top bit at or above 2^0, so e >= -63; e >= 0 means
    // |x| >= 2^63, which is decided by the two cases below.
    let (n, g_bits) = if e >= 0 {
        (u64::MAX, 0u64)
    } else {
        let s = e.unsigned_abs();
        (m >> s, m & ((1u64 << s) - 1))
    };
    if !neg && n >= 16385 {
        return overflow(env);
    }
    if neg && n >= 200 {
        return near_minus_one(env);
    }
    if g_bits == 0 {
        // An integer: 2^n - 1, or -(1 - 2^-n), both exact.
        let v = Big::pow2(n).sub(&Big::from_u64(1));
        let exp = if neg { -(n as i64) } else { 0 };
        return exact_value(neg, &v, exp, pc, env, fa);
    }
    let ge = if e < 0 { e } else { 0 };
    ziv(pc, env, fa, |w| {
        let one = Ax::int(1, w);
        let l = ln2(w);
        if neg {
            // x = -(n + g) = -(n + 1) + (1 - g), and
            // 2^x - 1 = 2^-(n+1) * (1 + (2^f - 1)) - 1 with f = 1 - g.
            let f = one.sub(&fixed_of(false, g_bits, ge, w));
            let fk = f.mul(&exp2m1_ratio(&f, &l, w), w);
            let v = one.add(&fk).shr(n + 1).sub(&one);
            Some(Enclosure::fixed(v, w))
        } else {
            // 2^x - 1 = 2^n * ((2^g - 1) + 1 - 2^-n).
            let g = fixed_of(false, g_bits, ge, w);
            let gk = g.mul(&exp2m1_ratio(&g, &l, w), w);
            let v = gk.add(&one).sub(&one.shr(n));
            Some(Enclosure {
                neg: false,
                mag: v.mag,
                err: v.err,
                exp: n as i64 - i64::from(w),
            })
        }
    })
}

/// `atan2(y, x)` — `FPATAN`, which leaves it in `ST(1)` with `y = ST(1)` and
/// `x = ST(0)`.
///
/// The special operands follow *Intel SDM* volume 2's `FPATAN` table, which
/// is the IEEE 754-2019 §9.2.1 `atan2` table: a zero `y` gives `±0` toward a
/// positive `x` and `±pi` toward a negative one (the sign of a zero `x`
/// counts), an infinite `y` gives `±pi/2`, or `±pi/4` / `±3pi/4` against an
/// infinite `x`, and a finite `y` against an infinite `x` gives `±0` or
/// `±pi`. Nothing but a signaling NaN or an unsupported encoding is invalid,
/// and there is no division by zero.
#[must_use]
pub fn atan2(y: F80, x: F80, pc: Precision, env: Env) -> Elementary {
    let (py, fy) = unpack(y, env);
    let (px, fx) = unpack(x, env);
    let fl = fy | fx;
    let (Some(py), Some(px)) = (py, px) else {
        return invalid(env, fl);
    };
    if py.class == Class::Nan || px.class == Class::Nan {
        return nan(&[py, px], env, fl);
    }
    let sy = py.sign;
    let pi_times = |num: u64, shift: u64| {
        ziv(pc, env, fl, |w| {
            let v = pi(w).mul_u64(num).shr(shift);
            Some(Enclosure::fixed(if sy { v.negated() } else { v }, w))
        })
    };
    match (py.class, px.class) {
        (Class::Zero, _) => {
            if px.sign {
                pi_times(1, 0)
            } else {
                zero(sy, fl, env)
            }
        }
        (Class::Inf, Class::Inf) => {
            if px.sign {
                pi_times(3, 2)
            } else {
                pi_times(1, 2)
            }
        }
        (Class::Inf, _) | (_, Class::Zero) => pi_times(1, 1),
        (_, Class::Inf) => {
            if px.sign {
                pi_times(1, 0)
            } else {
                zero(sy, fl, env)
            }
        }
        _ => atan2_finite(&py, &px, pc, env, fl),
    }
}

/// `atan2` of two finite non-zero operands.
fn atan2_finite(py: &Parts, px: &Parts, pc: Precision, env: Env, fl: Flags) -> Elementary {
    let (my, ey) = norm(py);
    let (mx, ex) = norm(px);
    let (sy, sx) = (py.sign, px.sign);
    let swap = (ey, my) > (ex, mx);
    if !swap && !sx && ey - ex <= -73 {
        // atan(q) with q = |y|/x < 2^(ey - ex + 1) <= 2^-72, so
        // q - atan q < q^3/3 and the relative gap is under 2^-145.
        //
        // q is rational and need not be representable, so `beside` does not
        // apply directly; the same argument is made here with q's own digits.
        // Q = floor(my 2^74 / mx) has 74 or 75 bits, so every rounding
        // boundary of a 64-bit (or narrower) result is a multiple of 2^8
        // units of u = 2^(ey - ex - 74). If the division is exact, the true
        // value lies in (Q u - u, Q u); otherwise q lies strictly inside
        // (Q u, (Q + 1) u) at least u 2^-64 above Q u, and atan q is below q
        // by less than q 2^-144 < u 2^-69, so it lies in (Q u, (Q + 1) u) too. Either
        // way an open unit interval with no boundary in it, represented by
        // its midpoint.
        let (q, r) = Big::from_u64(my).shl(74).div_rem(&Big::from_u64(mx));
        let rep = if r.is_zero() {
            q.shl(1).sub(&Big::from_u64(1))
        } else {
            q.shl(1).add_u64(1)
        };
        return exact_value(sy, &rep, ey - ex - 75, pc, env, fl);
    }
    let (mn, en, md, ed) = if swap {
        (mx, ex, my, ey)
    } else {
        (my, ey, mx, ex)
    };
    ziv(pc, env, fl, |w| {
        // q = |num| / |den| in (0, 1] at scale w; |num| <= |den| makes
        // en <= ed, so the shift is at most w.
        let shift = i64::from(w) + en - ed;
        let q = if shift >= 0 {
            let (q, r) = Big::from_u64(mn)
                .shl(shift as u64)
                .div_rem(&Big::from_u64(md));
            let err = Big::from_u64(u64::from(!r.is_zero()));
            Ax {
                neg: false,
                mag: q,
                err,
            }
        } else {
            // q < 2^(shift - w + 1) <= 2^-w: zero, within one unit.
            Ax {
                neg: false,
                mag: Big::zero(),
                err: Big::from_u64(1),
            }
        };
        let p = pi(w);
        let mut a = atan_unit(&q, &p, w)?;
        if swap {
            a = p.shr(1).sub(&a);
        }
        if sx {
            a = p.sub(&a);
        }
        Some(Enclosure::fixed(if sy { a.negated() } else { a }, w))
    })
}

/// `y * log2 x` — `FYL2X`, which leaves it in `ST(1)` with `y = ST(1)` and
/// `x = ST(0)`.
///
/// Special operands, *Intel SDM* volume 2's `FYL2X` table: a negative `x`
/// (other than `-0`) is invalid; a zero `x` gives an infinity of the sign
/// opposite to `y`'s — with the zero-divide exception for a finite `y`, and
/// invalid for a zero `y`; `x = +inf` gives an infinity of `y`'s sign, and is
/// invalid for a zero `y`; `x = 1` gives a zero of `y`'s sign and is invalid
/// for an infinite `y`. Otherwise a zero or infinite `y` gives a zero or
/// infinity whose sign is `y`'s, flipped when `x < 1`.
#[must_use]
pub fn y_log2_x(y: F80, x: F80, pc: Precision, env: Env) -> Elementary {
    let (py, fy) = unpack(y, env);
    let (px, fx) = unpack(x, env);
    let fl = fy | fx;
    let (Some(py), Some(px)) = (py, px) else {
        return invalid(env, fl);
    };
    if py.class == Class::Nan || px.class == Class::Nan {
        return nan(&[py, px], env, fl);
    }
    let sy = py.sign;
    match px.class {
        Class::Zero => {
            return match py.class {
                Class::Zero => invalid(env, fl),
                Class::Inf => inf(!sy, fl, env),
                _ => inf(!sy, fl | Flags::DIV_BY_ZERO, env),
            };
        }
        _ if px.sign => return invalid(env, fl),
        Class::Inf => {
            return match py.class {
                Class::Zero => invalid(env, fl),
                _ => inf(sy, fl, env),
            };
        }
        _ => {}
    }
    let (mx, ex) = norm(&px);
    let is_one = mx == 1 << 63 && ex == -63;
    let below_one = ex + 63 < 0;
    match py.class {
        Class::Inf if is_one => return invalid(env, fl),
        Class::Inf => return inf(sy != below_one, fl, env),
        Class::Zero => return zero(sy != below_one, fl, env),
        _ => {}
    }
    if is_one {
        return zero(sy, fl, env);
    }
    let (my, ey) = norm(&py);
    y_log2(sy, my, ey, &Big::from_u64(mx), ex, pc, env, fl)
}

/// `±my 2^ey * log2(big 2^e)`, for a finite non-zero `y` and a positive
/// `big 2^e` other than one.
#[allow(clippy::too_many_arguments)]
fn y_log2(
    sy: bool,
    my: u64,
    ey: i64,
    big: &Big,
    e: i64,
    pc: Precision,
    env: Env,
    fl: Flags,
) -> Elementary {
    let ybig = Big::from_u64(my);
    if let Log2::Exact(n) = log2_of(big, e, None, 0) {
        // A power of two: y * n is a product of integers, rounded once.
        let v = ybig.mul_u64(n.unsigned_abs());
        return exact_value(sy != (n < 0), &v, ey, pc, env, fl);
    }
    ziv(pc, env, fl, |w| {
        let Log2::Approx(l) = log2_of(big, e, Some(&ln2(w)), w) else {
            return None;
        };
        let l = l?;
        Some(Enclosure::scaled(sy != l.neg, &l, &ybig, ey - i64::from(w)))
    })
}

/// `y * log2(1 + x)` — `FYL2XP1`, `y = ST(1)`, `x = ST(0)`.
///
/// The instruction's domain is `|x| < 1 - sqrt(2)/2`, where it is accurate
/// for a tiny `x` in a way `FYL2X` of `1 + x` cannot be; the manual leaves
/// the result outside the domain undefined, and this computes the correctly
/// rounded value wherever `1 + x > 0`. Special operands follow the `FYL2XP1`
/// table: a zero `x` gives a zero whose sign is the product of the two signs,
/// and is invalid against an infinite `y`; a zero `y` likewise gives a signed
/// zero; an infinite `y` gives an infinity signed by the product. Outside the
/// table, `1 + x < 0` is invalid like a negative `FYL2X` argument and
/// `1 + x = 0` divides by zero like a zero one.
#[must_use]
pub fn y_log2_1p_x(y: F80, x: F80, pc: Precision, env: Env) -> Elementary {
    let (py, fy) = unpack(y, env);
    let (px, fx) = unpack(x, env);
    let fl = fy | fx;
    let (Some(py), Some(px)) = (py, px) else {
        return invalid(env, fl);
    };
    if py.class == Class::Nan || px.class == Class::Nan {
        return nan(&[py, px], env, fl);
    }
    let (sy, sx) = (py.sign, px.sign);
    match px.class {
        Class::Zero => {
            return match py.class {
                Class::Inf => invalid(env, fl),
                _ => zero(sy != sx, fl, env),
            };
        }
        Class::Inf => {
            // Outside the domain: log2(1 + inf) = inf, log2(1 - inf) is not
            // a number.
            return match py.class {
                _ if sx => invalid(env, fl),
                Class::Zero => invalid(env, fl),
                _ => inf(sy, fl, env),
            };
        }
        _ => {}
    }
    let (mx, ex) = norm(&px);
    // Where 1 + x stands against zero: x < -1, x = -1, or x > -1.
    let cmp_minus_one = if sx {
        (ex + 63).cmp(&0).then(mx.cmp(&(1 << 63)))
    } else {
        Ordering::Less
    };
    match (cmp_minus_one, py.class) {
        (Ordering::Greater, _) => return invalid(env, fl),
        (Ordering::Equal, Class::Zero) => return invalid(env, fl),
        (Ordering::Equal, Class::Inf) => return inf(!sy, fl, env),
        (Ordering::Equal, _) => return inf(!sy, fl | Flags::DIV_BY_ZERO, env),
        (_, Class::Zero) => return zero(sy != sx, fl, env),
        (_, Class::Inf) => return inf(sy != sx, fl, env),
        _ => {}
    }
    let (my, ey) = norm(&py);
    let top = ex + 63;
    // -1/2 < x < 1, open at both ends: there the ratio log2(1 + x) / x is
    // computed relative to the result, with |x / (2 + x)| <= 1/3 as
    // `log2_ratio` needs. The two ends are powers of two after adding one,
    // which the exact path below takes, as it takes everything outside.
    let inside = top < -1 || (top == -1 && !sx);
    if inside {
        let k = Big::from_u64(my).mul_u64(mx);
        return ziv(pc, env, fl, |w| {
            let xf = fixed_of(sx, mx, ex, w);
            let h = log2_ratio(&xf, &ln2(w), w)?;
            Some(Enclosure::scaled(sy != sx, &h, &k, ey + ex - i64::from(w)))
        });
    }
    // 1 + x exactly, as big * 2^e.
    let (big, e) = if ex >= 0 {
        (Big::from_u64(mx).shl(ex as u64).add_u64(1), 0)
    } else {
        let one = Big::pow2(ex.unsigned_abs());
        let v = if sx {
            one.sub(&Big::from_u64(mx))
        } else {
            one.add(&Big::from_u64(mx))
        };
        (v, ex)
    };
    y_log2(sy, my, ey, &big, e, pc, env, fl)
}

#[cfg(test)]
mod tests;
