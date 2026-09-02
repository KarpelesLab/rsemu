//! The binary interchange formats: binary32 and binary64.
//!
//! One implementation serves both. Every routine here is generic over
//! [`Format`] and works on the raw bit pattern held in a `u64`, with a
//! binary32 value in the low 32 bits — which is also how a RISC-V `f` register
//! holds one. The arithmetic itself is in `super::kernel`; this file is the
//! encoding, the special cases that need the encoding, and the guest-visible
//! entry points.
//!
//! Sources: IEEE 754-2019 §3.4 (the interchange encodings), §5 (the
//! operations), §5.7.2 (`class`).

use super::kernel::{self, Class, IntValue, Outcome, Parts, Rounded};
use super::{Env, Flags, IntOverflow, Spec};

/// A binary interchange format, described by its two field widths.
pub trait Format: Copy + core::fmt::Debug {
    /// Width of the trailing significand field: 23 or 52.
    const SIG_BITS: u32;
    /// Width of the exponent field: 8 or 11.
    const EXP_BITS: u32;

    /// Total width of the format in bits.
    const BITS: u32 = Self::SIG_BITS + Self::EXP_BITS + 1;
    /// `p`, the precision, counting the hidden bit.
    const PRECISION: u32 = Self::SIG_BITS + 1;
    /// Exponent bias.
    const BIAS: i32 = (1i32 << (Self::EXP_BITS - 1)) - 1;
    /// The smallest exponent of a normal number.
    const EMIN: i32 = 1 - Self::BIAS;
    /// The largest exponent of a finite number.
    const EMAX: i32 = Self::BIAS;
    /// A mask of every bit the format occupies.
    const MASK: u64 = u64::MAX >> (64 - Self::BITS);
    /// The sign bit.
    const SIGN: u64 = 1u64 << (Self::BITS - 1);
    /// A mask of the trailing significand field.
    const SIG_MASK: u64 = (1u64 << Self::SIG_BITS) - 1;
    /// The all-ones exponent field, unshifted.
    const EXP_FIELD_MAX: u64 = (1u64 << Self::EXP_BITS) - 1;
    /// The significand bit that distinguishes a quiet NaN from a signaling one
    /// (IEEE 754-2019 §6.2.1).
    const QUIET: u64 = 1u64 << (Self::SIG_BITS - 1);
    /// Positive infinity.
    const INF: u64 = Self::EXP_FIELD_MAX << Self::SIG_BITS;
    /// The positive quiet NaN with an all-zero payload: RISC-V's canonical NaN
    /// and ARM's default NaN. x86's "QNaN floating-point indefinite" is this
    /// with the sign bit set, which is why the sign is [`super::Nan`]'s
    /// business and not this constant's.
    const QUIET_NAN: u64 = Self::INF | Self::QUIET;
    /// The largest finite magnitude.
    const MAX_FINITE: u64 = Self::INF - 1;
    /// This format's arithmetic parameters.
    const SPEC: Spec = Spec::interchange(Self::PRECISION, Self::EMAX);
}

/// IEEE 754 binary16 — ARM's `H`.
///
/// Armv8.0-A converts to and from this format with `FCVT` whether or not it
/// has `FEAT_FP16`; what the optional feature adds is *arithmetic* in it. So a
/// core that implements only the base level still needs the encoding, and that
/// is what this is here for. Nothing about the arithmetic below is
/// format-specific, so `add::<B16>` works too — a guest with `FEAT_FP16` needs
/// no new kernel, only new rows in its instruction table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct B16;

impl Format for B16 {
    const SIG_BITS: u32 = 10;
    const EXP_BITS: u32 = 5;
}

/// IEEE 754 binary32 — RISC-V's `F` extension, x86's scalar single, ARM's `S`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct B32;

impl Format for B32 {
    const SIG_BITS: u32 = 23;
    const EXP_BITS: u32 = 8;
}

/// IEEE 754 binary64 — RISC-V's `D` extension, x86's scalar double, ARM's `D`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct B64;

impl Format for B64 {
    const SIG_BITS: u32 = 52;
    const EXP_BITS: u32 = 11;
}

/// The ten classes of IEEE 754-2019 §5.7.2's `class` operation, in the
/// standard's own order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    /// A signaling NaN.
    SignalingNan,
    /// A quiet NaN.
    QuietNan,
    /// Minus infinity.
    NegativeInfinity,
    /// A negative normal number.
    NegativeNormal,
    /// A negative subnormal number.
    NegativeSubnormal,
    /// Minus zero.
    NegativeZero,
    /// Plus zero.
    PositiveZero,
    /// A positive subnormal number.
    PositiveSubnormal,
    /// A positive normal number.
    PositiveNormal,
    /// Plus infinity.
    PositiveInfinity,
}

impl Category {
    /// The RISC-V `FCLASS` result: a one-hot mask over the same ten classes in
    /// a different order (Volume I, "Single-Precision Floating-Point Classify
    /// Instruction" — bit 0 is minus infinity and bit 9 is a quiet NaN).
    #[must_use]
    pub const fn riscv_fclass(self) -> u64 {
        match self {
            Category::NegativeInfinity => 1 << 0,
            Category::NegativeNormal => 1 << 1,
            Category::NegativeSubnormal => 1 << 2,
            Category::NegativeZero => 1 << 3,
            Category::PositiveZero => 1 << 4,
            Category::PositiveSubnormal => 1 << 5,
            Category::PositiveNormal => 1 << 6,
            Category::PositiveInfinity => 1 << 7,
            Category::SignalingNan => 1 << 8,
            Category::QuietNan => 1 << 9,
        }
    }
}

/// Take a bit pattern apart, with no environment applied.
fn decode<F: Format>(bits: u64) -> Parts {
    let bits = bits & F::MASK;
    let sign = bits & F::SIGN != 0;
    let field = (bits >> F::SIG_BITS) & F::EXP_FIELD_MAX;
    let frac = bits & F::SIG_MASK;
    if field == F::EXP_FIELD_MAX {
        if frac == 0 {
            Parts {
                sign,
                exp: 0,
                frac: 0,
                class: Class::Inf,
                snan: false,
            }
        } else {
            Parts {
                sign,
                exp: 0,
                frac,
                class: Class::Nan,
                snan: frac & F::QUIET == 0,
            }
        }
    } else if field == 0 {
        Parts {
            sign,
            exp: F::SPEC.min_ulp,
            frac,
            class: if frac == 0 {
                Class::Zero
            } else {
                Class::Finite
            },
            snan: false,
        }
    } else {
        Parts {
            sign,
            exp: field as i32 - F::BIAS - F::SIG_BITS as i32,
            frac: frac | (1u64 << F::SIG_BITS),
            class: Class::Finite,
            snan: false,
        }
    }
}

/// Take an operand apart, applying the environment's subnormal handling.
///
/// x86 reports a subnormal operand (`MXCSR.DE`) and can substitute a zero for
/// it (`MXCSR.DAZ`); ARM's `FPCR.FZ` does the substitution and reports `IDC`.
/// Everywhere else this is [`decode`] with no flags.
fn unpack<F: Format>(bits: u64, env: Env) -> (Parts, Flags) {
    let p = decode::<F>(bits);
    let subnormal = p.class == Class::Finite && p.frac < (1u64 << F::SIG_BITS);
    if !subnormal {
        return (p, Flags::NONE);
    }
    let flags = if env.subnormal_inputs.reports() {
        Flags::DENORMAL
    } else {
        Flags::NONE
    };
    if env.subnormal_inputs.flushes() {
        (Parts::zero(p.sign), flags)
    } else {
        (p, flags)
    }
}

/// Assemble an outcome into this format's bits.
pub(super) fn encode_outcome<F: Format>(out: Outcome, env: Env) -> u64 {
    let sign_bit = |sign: bool| if sign { F::SIGN } else { 0 };
    match out {
        Outcome::DefaultNan => sign_bit(env.nan.default_sign) | F::QUIET_NAN,
        // A propagated NaN always comes back quiet (IEEE 754-2019 §6.2.3).
        Outcome::Nan { sign, payload } => {
            sign_bit(sign) | F::INF | F::QUIET | (payload & F::SIG_MASK)
        }
        Outcome::Num(sign, Rounded::Zero) => sign_bit(sign),
        Outcome::Num(sign, Rounded::Inf) => sign_bit(sign) | F::INF,
        Outcome::Num(sign, Rounded::Finite { exp, frac }) => {
            let msb = 63 - frac.leading_zeros();
            let lead = exp + msb as i32;
            if lead >= F::EMIN {
                let frac = frac << (F::SIG_BITS - msb);
                let field = (lead + F::BIAS) as u64;
                sign_bit(sign) | (field << F::SIG_BITS) | (frac & F::SIG_MASK)
            } else {
                // Subnormal: the exponent field is zero and the significand
                // sits on the format's fixed grid.
                sign_bit(sign) | (frac << (exp - F::SPEC.min_ulp))
            }
        }
    }
}

/// `a + b` (IEEE 754-2019 §5.4.1).
pub fn add<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (out, f) = kernel::add(pa, pb, false, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa | fb)
}

/// `a - b` (§5.4.1).
pub fn sub<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (out, f) = kernel::add(pa, pb, true, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa | fb)
}

/// `a * b` (§5.4.1).
pub fn mul<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (out, f) = kernel::mul(pa, pb, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa | fb)
}

/// `a / b` (§5.4.1).
pub fn div<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (out, f) = kernel::div(pa, pb, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa | fb)
}

/// The square root of `a` (§5.4.1).
pub fn sqrt<F: Format>(a: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (out, f) = kernel::sqrt(pa, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa)
}

/// Round `a` to an integral value **in the same format** (§5.9's
/// `roundToIntegral` family).
///
/// Not a conversion: the result is a float, so `roundToIntegralTiesToEven` of
/// `1e300` is `1e300` and nothing overflows. §5.9 gives six operations, and
/// they differ in exactly two parameters, which is why this is one function:
/// the direction is [`Env::round`], and `signal_inexact` picks between the
/// five `roundToIntegral<direction>` operations, which raise nothing, and
/// `roundToIntegralExact`, which raises inexact when the result differs from
/// the operand.
///
/// That maps onto Arm's family one-for-one: `FRINTN`/`FRINTP`/`FRINTM`/
/// `FRINTZ`/`FRINTA` name their direction and pass `FALSE`, `FRINTI` takes the
/// direction from `FPCR.RMode` and passes `FALSE`, and `FRINTX` is the same as
/// `FRINTI` with `TRUE` — the one that reports (DDI 0487, `FPRoundInt`).
///
/// Neither overflow nor underflow is possible: a value large enough to be at
/// risk of either is already an integer and comes back untouched.
pub fn round_to_integral<F: Format>(a: u64, env: Env, signal_inexact: bool) -> (u64, Flags) {
    let (p, fa) = unpack::<F>(a, env);
    let out = match p.class {
        Class::Nan => match kernel::nan_result(&[p], env) {
            Some((out, f)) => return (encode_outcome::<F>(out, env), f | fa),
            // `nan_result` answers `None` only when no operand is a NaN.
            None => Outcome::DefaultNan,
        },
        // An infinity and a zero are already integral, sign included.
        Class::Inf => Outcome::Num(p.sign, Rounded::Inf),
        Class::Zero => Outcome::Num(p.sign, Rounded::Zero),
        // `exp` is the exponent of the significand's last bit, so a
        // non-negative one means every bit of the value is an integer bit.
        // Returning the operand unchanged here is what keeps the operation
        // total over the whole format rather than only over the part that
        // fits in an integer.
        Class::Finite if p.exp >= 0 => return (a & F::MASK, fa),
        Class::Finite => match kernel::to_integer(p, env) {
            IntValue::Value {
                sign,
                magnitude,
                inexact,
            } => {
                // The operand was not integral, so its magnitude is below the
                // format's first all-integers value — `2^(p-1)` — and the
                // rounded magnitude is at most that. It therefore fits the
                // precision exactly and this step cannot round a second time.
                let (out, _) = kernel::round_exact(sign, 0, magnitude, F::SPEC, env);
                let flags = if signal_inexact && inexact {
                    Flags::INEXACT
                } else {
                    Flags::NONE
                };
                return (encode_outcome::<F>(out, env), flags | fa);
            }
            // Both other variants are the classes already matched above.
            IntValue::Nan => Outcome::DefaultNan,
            IntValue::Inf(sign) => Outcome::Num(sign, Rounded::Inf),
        },
    };
    (encode_outcome::<F>(out, env), fa)
}

/// `a * b + c`, rounded once (§5.4.1's `fusedMultiplyAdd`).
pub fn fma<F: Format>(a: u64, b: u64, c: u64, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (pc, fc) = unpack::<F>(c, env);
    let (out, f) = kernel::fma(pa, pb, pc, F::SPEC, env);
    (encode_outcome::<F>(out, env), f | fa | fb | fc)
}

/// The ordering of two values, or `None` when they are unordered because one
/// is a NaN (§5.11).
///
/// Raises nothing: the caller decides whether its comparison is the quiet or
/// the signaling one.
#[must_use]
pub fn compare<F: Format>(a: u64, b: u64) -> Option<core::cmp::Ordering> {
    let (pa, pb) = (decode::<F>(a), decode::<F>(b));
    if pa.class == Class::Nan || pb.class == Class::Nan {
        return None;
    }
    Some(kernel::compare(pa, pb))
}

/// `a == b`, the quiet comparison: only a signaling NaN raises invalid
/// (§5.11).
pub fn eq<F: Format>(a: u64, b: u64) -> (bool, Flags) {
    let (pa, pb) = (decode::<F>(a), decode::<F>(b));
    if pa.class == Class::Nan || pb.class == Class::Nan {
        let f = if pa.snan || pb.snan {
            Flags::INVALID
        } else {
            Flags::NONE
        };
        return (false, f);
    }
    (
        kernel::compare(pa, pb) == core::cmp::Ordering::Equal,
        Flags::NONE,
    )
}

/// `a < b`, the signaling comparison: **any** NaN raises invalid (§5.11).
pub fn lt<F: Format>(a: u64, b: u64) -> (bool, Flags) {
    match compare::<F>(a, b) {
        None => (false, Flags::INVALID),
        Some(ord) => (ord == core::cmp::Ordering::Less, Flags::NONE),
    }
}

/// `a <= b`, the signaling comparison (§5.11).
pub fn le<F: Format>(a: u64, b: u64) -> (bool, Flags) {
    match compare::<F>(a, b) {
        None => (false, Flags::INVALID),
        Some(ord) => (ord != core::cmp::Ordering::Greater, Flags::NONE),
    }
}

/// The smaller of two values, under [`super::MinMax`]'s rule.
pub fn min<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    min_max::<F>(a, b, true, env)
}

/// The larger of two values, under [`super::MinMax`]'s rule.
pub fn max<F: Format>(a: u64, b: u64, env: Env) -> (u64, Flags) {
    min_max::<F>(a, b, false, env)
}

fn min_max<F: Format>(a: u64, b: u64, want_min: bool, env: Env) -> (u64, Flags) {
    let (pa, fa) = unpack::<F>(a, env);
    let (pb, fb) = unpack::<F>(b, env);
    let (pick, f) = kernel::min_max(pa, pb, want_min, env);
    let bits = match pick {
        // The winner is delivered as the *operand* rather than recomputed, so
        // a NaN payload survives — but an operand the environment flushed has
        // already stopped being that subnormal, and delivering the original
        // encoding would undo the flush.
        Ok(i) => {
            let (parts, raw) = if i == 0 { (pa, a) } else { (pb, b) };
            if parts.class == Class::Zero && raw & F::MASK & !F::SIGN != 0 {
                encode_outcome::<F>(Outcome::Num(parts.sign, Rounded::Zero), env)
            } else {
                raw & F::MASK
            }
        }
        Err(out) => encode_outcome::<F>(out, env),
    };
    (bits, f | fa | fb)
}

/// Which of the ten classes a value belongs to (§5.7.2).
///
/// Classification is not arithmetic: it reads the encoding as it stands, so a
/// subnormal is a subnormal even under flush-to-zero, and it raises nothing.
#[must_use]
pub fn classify<F: Format>(a: u64) -> Category {
    let p = decode::<F>(a);
    match p.class {
        Class::Nan => {
            if p.snan {
                Category::SignalingNan
            } else {
                Category::QuietNan
            }
        }
        Class::Inf => {
            if p.sign {
                Category::NegativeInfinity
            } else {
                Category::PositiveInfinity
            }
        }
        Class::Zero => {
            if p.sign {
                Category::NegativeZero
            } else {
                Category::PositiveZero
            }
        }
        Class::Finite => {
            let subnormal = p.frac < (1u64 << F::SIG_BITS);
            match (p.sign, subnormal) {
                (true, false) => Category::NegativeNormal,
                (true, true) => Category::NegativeSubnormal,
                (false, true) => Category::PositiveSubnormal,
                (false, false) => Category::PositiveNormal,
            }
        }
    }
}

/// Convert between two binary formats (§5.4.2).
///
/// A propagated NaN payload is rescaled rather than truncated at the bottom:
/// widening shifts it up and narrowing shifts it down, so the leading payload
/// bits — the ones software actually sets — survive a round trip through the
/// wider format.
pub fn convert<A: Format, B: Format>(bits: u64, env: Env) -> (u64, Flags) {
    let (p, fin) = unpack::<A>(bits, env);
    let (out, f) = match p.class {
        Class::Nan => {
            let flags = if p.snan { Flags::INVALID } else { Flags::NONE };
            match super::kernel::nan_result(&[p], env) {
                Some((Outcome::Nan { sign, payload }, _)) => {
                    let payload = if B::SIG_BITS >= A::SIG_BITS {
                        payload << (B::SIG_BITS - A::SIG_BITS)
                    } else {
                        payload >> (A::SIG_BITS - B::SIG_BITS)
                    };
                    (Outcome::Nan { sign, payload }, flags)
                }
                _ => (Outcome::DefaultNan, flags),
            }
        }
        Class::Inf => (Outcome::Num(p.sign, Rounded::Inf), Flags::NONE),
        Class::Zero => (Outcome::Num(p.sign, Rounded::Zero), Flags::NONE),
        Class::Finite => kernel::round_exact(p.sign, p.exp, u128::from(p.frac), B::SPEC, env),
    };
    (encode_outcome::<B>(out, env), f | fin)
}

/// Sign-extend a `bits`-wide two's complement value to 64 bits.
fn sign_extend(v: u64, bits: u32) -> i64 {
    if bits >= 64 {
        v as i64
    } else {
        ((v << (64 - bits)) as i64) >> (64 - bits)
    }
}

/// Convert to a signed integer `bits` wide, sign-extended to 64 (§5.4.1's
/// `convertToIntegerExact` family).
///
/// What happens when the value does not fit is [`IntOverflow`]: RISC-V
/// saturates and gives a NaN the most positive value, ARM saturates and gives
/// a NaN zero, x86 delivers the integer indefinite for all three.
pub fn to_signed<F: Format>(value: u64, bits: u32, env: Env) -> (i64, Flags) {
    let (p, fin) = unpack::<F>(value, env);
    let max: u128 = (1u128 << (bits - 1)) - 1;
    let min_mag: u128 = 1u128 << (bits - 1);
    let most_negative = sign_extend(min_mag as u64, bits);
    let most_positive = max as i64;
    let out_of_range = |sign: bool| match env.int_overflow {
        IntOverflow::Indefinite => most_negative,
        _ => {
            if sign {
                most_negative
            } else {
                most_positive
            }
        }
    };
    match kernel::to_integer(p, env) {
        IntValue::Nan => {
            let v = match env.int_overflow {
                IntOverflow::SaturateNanMax => most_positive,
                IntOverflow::SaturateNanZero => 0,
                IntOverflow::Indefinite => most_negative,
            };
            (v, Flags::INVALID | fin)
        }
        IntValue::Inf(sign) => (out_of_range(sign), Flags::INVALID | fin),
        IntValue::Value {
            sign,
            magnitude,
            inexact,
        } => {
            let fits = if sign {
                magnitude <= min_mag
            } else {
                magnitude <= max
            };
            if !fits {
                return (out_of_range(sign), Flags::INVALID | fin);
            }
            let v = if sign {
                sign_extend((magnitude as u64).wrapping_neg(), bits)
            } else {
                magnitude as i64
            };
            let f = if inexact { Flags::INEXACT } else { Flags::NONE };
            (v, f | fin)
        }
    }
}

/// Convert to an unsigned integer `bits` wide.
///
/// A negative input that *rounds* to zero is 0 and merely inexact; anything
/// that rounds below zero is invalid. Under [`IntOverflow::Indefinite`] an
/// out-of-range value delivers the all-ones pattern, which is the unsigned
/// counterpart of the integer indefinite; nothing in this crate depends on
/// that choice yet, because x87 and pre-AVX-512 SSE have no unsigned
/// conversion at all.
pub fn to_unsigned<F: Format>(value: u64, bits: u32, env: Env) -> (u64, Flags) {
    let (p, fin) = unpack::<F>(value, env);
    let max: u128 = if bits >= 64 {
        u128::from(u64::MAX)
    } else {
        (1u128 << bits) - 1
    };
    match kernel::to_integer(p, env) {
        IntValue::Nan => {
            let v = match env.int_overflow {
                IntOverflow::SaturateNanZero => 0,
                _ => max as u64,
            };
            (v, Flags::INVALID | fin)
        }
        IntValue::Inf(sign) => (if sign { 0 } else { max as u64 }, Flags::INVALID | fin),
        IntValue::Value {
            sign,
            magnitude,
            inexact,
        } => {
            if sign && magnitude != 0 {
                (0, Flags::INVALID | fin)
            } else if magnitude > max {
                (max as u64, Flags::INVALID | fin)
            } else {
                let f = if inexact { Flags::INEXACT } else { Flags::NONE };
                (magnitude as u64, f | fin)
            }
        }
    }
}

/// Convert from a signed integer `bits` wide (§5.4.1's `convertFromInt`).
pub fn from_signed<F: Format>(value: i64, bits: u32, env: Env) -> (u64, Flags) {
    let v = sign_extend(value as u64, bits);
    from_magnitude::<F>(v < 0, u128::from(v.unsigned_abs()), env)
}

/// Convert from an unsigned integer `bits` wide.
pub fn from_unsigned<F: Format>(value: u64, bits: u32, env: Env) -> (u64, Flags) {
    let v = if bits >= 64 {
        value
    } else {
        value & ((1u64 << bits) - 1)
    };
    from_magnitude::<F>(false, u128::from(v), env)
}

/// The shared body of the two integer-to-float conversions. An integer zero
/// converts to `+0` in every rounding direction (§5.4.1).
fn from_magnitude<F: Format>(sign: bool, magnitude: u128, env: Env) -> (u64, Flags) {
    if magnitude == 0 {
        return (0, Flags::NONE);
    }
    let (out, f) = kernel::round_exact(sign, 0, magnitude, F::SPEC, env);
    (encode_outcome::<F>(out, env), f)
}
