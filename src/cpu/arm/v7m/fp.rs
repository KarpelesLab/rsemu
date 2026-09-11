//! The FPv4-SP / FPv5-SP floating-point extension: the register file,
//! `FPSCR`, and the Arm rules that wrap [`crate::float`].
//!
//! The arithmetic is not here. Every value this module computes comes out of
//! [`crate::float`], the crate's one software IEEE-754 implementation, because
//! `ROADMAP.md` §9.1 makes guest floating point bit-reproducible across hosts
//! and a second implementation of rounding would be a second set of bugs. What
//! *is* here is the paperwork that turns IEEE-754 into an Arm FPU:
//!
//! 1. **The register file.** Thirty-two single-precision registers `S0`–`S31`,
//!    which are also `D0`–`D15` for the *transfer* instructions even on a
//!    single-precision-only part — `VLDM`/`VSTM`/`VPUSH`/`VPOP` count singles
//!    here, so the pairing only matters to a double-precision encoding this
//!    core does not have.
//! 2. **`FPSCR` as a [`float::Env`].** Every choice IEEE leaves open is a
//!    field of `Env`, and `Env::ARM` already spells Arm's answers; this module
//!    only decides which profile `FPSCR.DN` and `FPSCR.FZ` select and which
//!    direction `FPSCR.RMode` names.
//! 3. **The four rules that are Arm's pseudocode rather than IEEE's**, where
//!    calling `float` straight through would be subtly wrong: `FPMulAdd`'s
//!    operand order and its quiet-NaN-times-infinity override, `FPCompare`'s
//!    four-way `NZCV`, `FPMaxNum`'s substitution of an infinity for a quiet
//!    NaN, and the chained (twice-rounded) multiply-accumulates, which are
//!    *not* fused and must not be routed through `fma`.
//!
//! # What is implemented, and what is not
//!
//! Single precision — FPv4-SP as a Cortex-M4F has it, plus the FPv5 additions
//! a Cortex-M7 with the single-precision FPU has (`VSEL`, `VMAXNM`/`VMINNM`,
//! `VCVTA/N/P/M`, `VRINTA/N/P/M/X/Z/R`), selected per instance by
//! [`FpUnit`](super::FpUnit).
//!
//! **Double precision is not implemented.** The parts modelled here are
//! FPv4-SP-D16 and FPv5-SP-D16, both of which are real configurations, so
//! `MVFR0.D_Precision` reads zero and every `coproc == 0b1011` encoding is
//! UNDEFINED — which is what a single-precision part does. An FPv5-DP-D16
//! Cortex-M7 is therefore *not* modelled, and firmware built `-mfpu=fpv5-d16`
//! will trap on its first `VADD.F64`. That is a stated limitation rather than
//! a silent one.
//!
//! **`FPSCR.AHP` is RES0.** The alternative half-precision format is a second
//! encoding of binary16 with no infinities and no NaNs that only `VCVTB` and
//! `VCVTT` can reach. It is not implemented, so the bit reads back as zero
//! after a write and a guest can tell — which is the honest failure. Honouring
//! the write and then converting in the IEEE format would make every half
//! conversion quietly wrong instead. The IEEE half conversions (`AHP == 0`)
//! are implemented in full.
//!
//! **The exception-enable bits (`IDE`, `IXE`, `UFE`, `OFE`, `DZE`, `IOE`) are
//! RES0**, as they are on every Cortex-M part: trapped floating-point
//! exceptions are not implemented and `MVFR0.FP_Exception_Trapping` says so.
//! `Len` and `Stride` are RES0 too — short vectors are a VFPv2 feature no
//! M-profile part has — and so is `QC`, which needs Advanced SIMD.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403E: A2.5 (the
//! floating-point data types and `FPSCR`), A6.4 and A7.5 (the encodings),
//! A7.7 (the per-instruction pseudocode and the shared `FPAdd`, `FPMul`,
//! `FPMulAdd`, `FPCompare`, `FPProcessNaN`, `FPProcessNaNs`,
//! `FPProcessNaNs3`, `FPRoundInt`, `FPToFixed`, `FixedToFP` and
//! `VFPExpandImm`), B1.5.7 (`FPCCR`, `FPCAR`, `FPDSCR` and lazy state
//! preservation), B3.2.20 (`CPACR`). The `MVFR0`/`MVFR1`/`MVFR2` values are
//! the Cortex-M4 and Cortex-M7 Technical Reference Manuals'. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

use crate::float::{self, B16, B32, Category, Env, Flags, Round};

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// How many single-precision registers a `D16` part has.
pub const S_COUNT: usize = 32;

/// The names a disassembler prints for them.
pub const S_NAMES: [&str; S_COUNT] = [
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "s12", "s13", "s14",
    "s15", "s16", "s17", "s18", "s19", "s20", "s21", "s22", "s23", "s24", "s25", "s26", "s27",
    "s28", "s29", "s30", "s31",
];

/// The floating-point register file and `FPSCR`.
///
/// `FPCCR`, `FPCAR` and `FPDSCR` are **not** here: they are memory-mapped
/// registers in the system block at `0xE000EF34`, so they live in
/// [`super::sys::Sys`] with the rest of the map. What is here is the state an
/// exception frame carries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Fpu {
    /// `S0`–`S31`. Architecturally UNKNOWN at reset, zero here.
    pub s: [u32; S_COUNT],
    /// `FPSCR`.
    pub fpscr: u32,
}

impl Fpu {
    /// The reset state.
    #[must_use]
    pub const fn new() -> Fpu {
        Fpu {
            s: [0; S_COUNT],
            fpscr: 0,
        }
    }

    /// Read `Sn`.
    #[inline]
    #[must_use]
    pub const fn s(&self, index: u8) -> u32 {
        self.s[(index & 31) as usize]
    }

    /// Write `Sn`.
    #[inline]
    pub const fn set_s(&mut self, index: u8, value: u32) {
        self.s[(index & 31) as usize] = value;
    }

    /// The environment the current `FPSCR` describes.
    #[inline]
    #[must_use]
    pub fn env(&self) -> Env {
        env(self.fpscr)
    }

    /// Write a result to `Sd` and fold the exceptions it raised into `FPSCR`,
    /// which is what almost every data-processing instruction ends with.
    #[inline]
    pub fn finish(&mut self, d: u8, value: u32, flags: Flags) {
        self.set_s(d, value);
        accumulate(&mut self.fpscr, flags);
    }
}

impl Default for Fpu {
    fn default() -> Fpu {
        Fpu::new()
    }
}

// ---------------------------------------------------------------------------
// FPSCR
// ---------------------------------------------------------------------------

/// The `FPSCR` bits (DDI 0403E A2.5.3).
pub mod fpscr {
    /// Negative condition flag, written by `VCMP`.
    pub const N: u32 = 1 << 31;
    /// Zero condition flag.
    pub const Z: u32 = 1 << 30;
    /// Carry condition flag.
    pub const C: u32 = 1 << 29;
    /// Overflow condition flag.
    pub const V: u32 = 1 << 28;
    /// Alternative half-precision. **RES0 here** — see the module docs.
    pub const AHP: u32 = 1 << 26;
    /// Default NaN mode.
    pub const DN: u32 = 1 << 25;
    /// Flush-to-zero mode.
    pub const FZ: u32 = 1 << 24;
    /// The low bit of the rounding-mode field.
    pub const RMODE_SHIFT: u32 = 22;
    /// The rounding-mode field, in place.
    pub const RMODE: u32 = 3 << RMODE_SHIFT;
    /// Input denormal, cumulative.
    pub const IDC: u32 = 1 << 7;
    /// Inexact, cumulative.
    pub const IXC: u32 = 1 << 4;
    /// Underflow, cumulative.
    pub const UFC: u32 = 1 << 3;
    /// Overflow, cumulative.
    pub const OFC: u32 = 1 << 2;
    /// Divide by zero, cumulative.
    pub const DZC: u32 = 1 << 1;
    /// Invalid operation, cumulative.
    pub const IOC: u32 = 1 << 0;

    /// The four condition flags together.
    pub const FLAGS: u32 = N | Z | C | V;
    /// The six cumulative exception bits together.
    pub const CUMULATIVE: u32 = IDC | IXC | UFC | OFC | DZC | IOC;
    /// Every bit a `VMSR` may set.
    ///
    /// `AHP`, the six trap enables, `Len`, `Stride` and `QC` are all absent:
    /// each is RES0 on this core and the module docs say why for each.
    pub const WRITABLE: u32 = FLAGS | DN | FZ | RMODE | CUMULATIVE;
}

/// The rounding direction `FPSCR.RMode` selects.
///
/// **Not** the order [`Round`] declares them in, and not x86's: `00` nearest
/// even, `01` toward `+∞`, `10` toward `−∞`, `11` toward zero. Assuming
/// otherwise gives a core that rounds up where the guest asked for down, and
/// no ordinary test finds it.
#[must_use]
pub const fn rounding(fpscr: u32) -> Round {
    match (fpscr >> fpscr::RMODE_SHIFT) & 3 {
        0b00 => Round::TiesEven,
        0b01 => Round::TowardPositive,
        0b10 => Round::TowardNegative,
        _ => Round::TowardZero,
    }
}

/// The floating-point environment `FPSCR` describes.
#[must_use]
pub fn env(fpscr: u32) -> Env {
    let base = if fpscr & fpscr::DN != 0 {
        Env::ARM_DEFAULT_NAN
    } else {
        Env::ARM
    };
    base.round(rounding(fpscr)).flush(fpscr & fpscr::FZ != 0)
}

/// Fold a set of exceptions into `FPSCR`'s cumulative bits.
///
/// The flags are sticky: an operation only ever sets them and nothing but a
/// `VMSR` clears one (IEEE 754-2019 §7.1). `FPSCR`'s cumulative bits are in
/// the same order as `FPSR`'s, which is why this is
/// [`Flags::to_fpsr`](crate::float::Flags::to_fpsr) and not a fourth encoding.
#[inline]
pub fn accumulate(fpscr: &mut u32, flags: Flags) {
    *fpscr |= flags.to_fpsr();
}

// ---------------------------------------------------------------------------
// The arithmetic, all of it through `crate::float`
// ---------------------------------------------------------------------------

/// The sign bit of a single.
const SIGN: u32 = 1 << 31;
/// The default NaN a single-precision operation delivers when `FPSCR.DN` is
/// set, or when an invalid operation has no NaN operand to propagate.
const DEFAULT_NAN: u32 = 0x7fc0_0000;
/// The significand bit that makes a NaN quiet.
const QUIET: u32 = 1 << 22;
/// `+∞`.
const INFINITY: u32 = 0x7f80_0000;

/// `FPNeg`: flip the sign bit, and nothing else.
///
/// Not an arithmetic negation — it neither quietens a NaN nor raises an
/// exception, which is why `VNEG` of a signaling NaN leaves it signaling
/// (DDI 0403E A7.7.226).
#[must_use]
#[inline]
pub const fn neg(a: u32) -> u32 {
    a ^ SIGN
}

/// `FPAbs`: clear the sign bit, with the same caveat as [`neg`].
#[must_use]
#[inline]
pub const fn abs(a: u32) -> u32 {
    a & !SIGN
}

/// Classify a single.
#[must_use]
#[inline]
pub fn classify(a: u32) -> Category {
    float::classify::<B32>(u64::from(a))
}

/// Narrow a `float` result back to a single.
#[inline]
fn single((v, f): (u64, Flags)) -> (u32, Flags) {
    (v as u32, f)
}

/// `a + b`.
#[must_use]
pub fn add(a: u32, b: u32, env: Env) -> (u32, Flags) {
    single(float::add::<B32>(u64::from(a), u64::from(b), env))
}

/// `a - b`.
#[must_use]
pub fn sub(a: u32, b: u32, env: Env) -> (u32, Flags) {
    single(float::sub::<B32>(u64::from(a), u64::from(b), env))
}

/// `a * b`.
#[must_use]
pub fn mul(a: u32, b: u32, env: Env) -> (u32, Flags) {
    single(float::mul::<B32>(u64::from(a), u64::from(b), env))
}

/// `a / b`.
#[must_use]
pub fn div(a: u32, b: u32, env: Env) -> (u32, Flags) {
    single(float::div::<B32>(u64::from(a), u64::from(b), env))
}

/// `sqrt(a)`.
#[must_use]
pub fn sqrt(a: u32, env: Env) -> (u32, Flags) {
    single(float::sqrt::<B32>(u64::from(a), env))
}

/// `FPRoundInt`: round to an integral value, staying in the format.
///
/// `signal_inexact` is `VRINTX`'s and nothing else's — `VRINTR` and the
/// explicit-mode `VRINTA/N/P/M/Z` are all "exact" in IEEE's vocabulary, which
/// means they do *not* raise inexact for a value they changed.
#[must_use]
pub fn round_int(a: u32, env: Env, signal_inexact: bool) -> (u32, Flags) {
    single(float::round_to_integral::<B32>(
        u64::from(a),
        env,
        signal_inexact,
    ))
}

/// `FPMaxNum`/`FPMinNum`, which is `VMAXNM`/`VMINNM` (FPv5).
///
/// DDI 0403E writes it as a substitution followed by `FPMax`, and it is
/// written that way here for the same reason: only a *quiet* NaN is replaced,
/// only when the other operand is not also one, and a signaling NaN still
/// reaches `FPMax` and still raises invalid there. Reaching for `float`'s
/// `MinMax::NonNan` instead would get the signaling case wrong, because that
/// rule returns the other operand for any NaN at all.
#[must_use]
pub fn max_min_num(a: u32, b: u32, want_min: bool, env: Env) -> (u32, Flags) {
    let quiet = |v: u32| classify(v) == Category::QuietNan;
    // The identity element: `−∞` for a maximum, `+∞` for a minimum.
    let identity = if want_min { INFINITY } else { neg(INFINITY) };
    let (a, b) = match (quiet(a), quiet(b)) {
        (true, false) => (identity, b),
        (false, true) => (a, identity),
        _ => (a, b),
    };
    if want_min {
        single(float::min::<B32>(u64::from(a), u64::from(b), env))
    } else {
        single(float::max::<B32>(u64::from(a), u64::from(b), env))
    }
}

/// `FPMulAdd(addend, op1, op2)`: `addend + op1 × op2`, rounded **once**.
///
/// Two things here are Arm's and not IEEE's, and both are why this is not a
/// call straight through to `float::fma`:
///
/// * **The NaN order puts the addend first.** `FPProcessNaNs3` looks for a
///   signaling NaN in the addend, then `op1`, then `op2`, and only then for a
///   quiet one in the same order. `float::fma`'s arguments are `a × b + c`,
///   which would put the addend last and return a different payload from real
///   silicon.
/// * **A quiet-NaN addend loses to `∞ × 0`.** If the addend is a quiet NaN
///   *and* the product is `∞ × 0`, the result is the default NaN with invalid
///   raised; the propagation is overridden. Nothing in IEEE 754 predicts that.
#[must_use]
pub fn mul_add(addend: u32, op1: u32, op2: u32, env: Env) -> (u32, Flags) {
    let is_inf = |v: u32| {
        matches!(
            classify(v),
            Category::NegativeInfinity | Category::PositiveInfinity
        )
    };
    let is_zero = |v: u32| matches!(classify(v), Category::NegativeZero | Category::PositiveZero);

    if classify(addend) == Category::QuietNan
        && ((is_inf(op1) && is_zero(op2)) || (is_zero(op1) && is_inf(op2)))
    {
        return (default_nan(env), Flags::INVALID);
    }
    if let Some(result) = process_nans3([addend, op1, op2], env) {
        return result;
    }
    single(float::fma::<B32>(
        u64::from(op1),
        u64::from(op2),
        u64::from(addend),
        env,
    ))
}

/// `FPProcessNaNs3`: which NaN a three-operand instruction returns, or `None`
/// when no operand is a NaN and the caller should do arithmetic.
fn process_nans3(ops: [u32; 3], env: Env) -> Option<(u32, Flags)> {
    let signaling = ops.iter().any(|&v| classify(v) == Category::SignalingNan);
    let flags = if signaling {
        Flags::INVALID
    } else {
        Flags::NONE
    };
    let pick = ops
        .iter()
        .find(|&&v| classify(v) == Category::SignalingNan)
        .or_else(|| ops.iter().find(|&&v| classify(v) == Category::QuietNan))?;
    if env.nan.propagate == float::Propagate::Default {
        return Some((default_nan(env), flags));
    }
    // A propagated NaN always comes back quiet, sign and payload intact
    // (IEEE 754-2019 §6.2.3).
    Some((pick | QUIET, flags))
}

/// The environment's default NaN.
fn default_nan(env: Env) -> u32 {
    if env.nan.default_sign {
        DEFAULT_NAN | SIGN
    } else {
        DEFAULT_NAN
    }
}

/// `FPCompare`: the four `FPSCR` condition bits a comparison writes.
///
/// The four results are **not** the integer comparison's: *unordered* is
/// `0b0011`, which sets `C` and `V` both, so `VMRS APSR_nzcv` followed by
/// `BVS` is the "was a NaN involved" test and `BHS` is "greater or equal or
/// unordered". Deriving these from a subtraction gets every one of them wrong.
///
/// `signal_all` is `VCMPE`: it raises invalid for a quiet NaN too, where
/// `VCMP` raises only for a signaling one.
#[must_use]
pub fn compare(a: u32, b: u32, signal_all: bool, env: Env) -> (u32, Flags) {
    use core::cmp::Ordering;
    let mut flags = Flags::NONE;
    // `FPCompare` unpacks its operands *through* `FPSCR`, so flush-to-zero
    // applies here as it does to arithmetic: with `FZ` set, a subnormal
    // compares equal to a zero of the same sign and sets `IDC`. A comparison
    // that read the raw encodings would call the same pair unequal.
    let mut flush = |v: u32| {
        let subnormal = matches!(
            classify(v),
            Category::NegativeSubnormal | Category::PositiveSubnormal
        );
        if subnormal && env.subnormal_inputs.flushes() {
            if env.subnormal_inputs.reports() {
                flags |= Flags::DENORMAL;
            }
            v & SIGN
        } else {
            v
        }
    };
    let (a, b) = (flush(a), flush(b));
    let nzcv = match float::compare::<B32>(u64::from(a), u64::from(b)) {
        Some(Ordering::Equal) => fpscr::Z | fpscr::C,
        Some(Ordering::Less) => fpscr::N,
        Some(Ordering::Greater) => fpscr::C,
        None => {
            let snan =
                classify(a) == Category::SignalingNan || classify(b) == Category::SignalingNan;
            if snan || signal_all {
                flags |= Flags::INVALID;
            }
            fpscr::C | fpscr::V
        }
    };
    (nzcv, flags)
}

/// `VCVTB`/`VCVTT` half to single.
///
/// Flush-to-zero never applies to a half-precision *operand*: `FPSCR.FZ` is
/// defined on single precision, and every subnormal half is exactly
/// representable as a *normal* single, so flushing it would throw away a value
/// the destination format holds perfectly well.
#[must_use]
pub fn half_to_single(half: u16, env: Env) -> (u32, Flags) {
    single(float::convert::<B16, B32>(
        u64::from(half),
        env.flush(false),
    ))
}

/// `VCVTB`/`VCVTT` single to half, with the same rule about flushing.
#[must_use]
pub fn single_to_half(value: u32, env: Env) -> (u16, Flags) {
    let (v, f) = float::convert::<B32, B16>(u64::from(value), env.flush(false));
    (v as u16, f)
}

/// `FPToFixed`: float to a `bits`-wide integer, scaled by `2^fbits`.
///
/// Out of range saturates and a NaN gives zero, both with invalid raised —
/// [`float::IntOverflow::SaturateNanZero`], which `Env::ARM` already carries.
/// The result is returned in the low `bits` of a word because that is where a
/// `VCVT` leaves it in `Sd`.
#[must_use]
pub fn to_fixed(value: u32, bits: u32, unsigned: bool, fbits: u32, env: Env) -> (u32, Flags) {
    let (scaled, mut flags) = scale_by_pow2(value, fbits as i32, env);
    let out = if unsigned {
        let (v, f) = float::to_unsigned::<B32>(u64::from(scaled), bits, env);
        flags |= f;
        v as u32
    } else {
        let (v, f) = float::to_signed::<B32>(u64::from(scaled), bits, env);
        flags |= f;
        v as u32
    };
    let mask = if bits >= 32 {
        u32::MAX
    } else {
        (1 << bits) - 1
    };
    (out & mask, flags)
}

/// `FixedToFP`: a `bits`-wide integer scaled by `2^-fbits`, to a single.
///
/// The two steps round once between them for every width this core can name:
/// a 32-bit integer rounds to binary32 in [`float::from_signed`], and the
/// subsequent scaling by a power of two is exact, because no `VCVT` encoding
/// can express an `fbits` large enough to push the result into the subnormal
/// range.
#[must_use]
pub fn from_fixed(value: u32, bits: u32, unsigned: bool, fbits: u32, env: Env) -> (u32, Flags) {
    let (v, mut flags) = if unsigned {
        let masked = if bits >= 32 {
            u64::from(value)
        } else {
            u64::from(value) & ((1 << bits) - 1)
        };
        single(float::from_unsigned::<B32>(masked, bits, env))
    } else {
        single(float::from_signed::<B32>(
            sign_extend(value, bits),
            bits,
            env,
        ))
    };
    let (out, f) = scale_by_pow2(v, -(fbits as i32), env);
    flags |= f;
    (out, flags)
}

/// Sign-extend the low `bits` of `value` to a full `i64`.
///
/// `bits` is 16 or 32 — the two fixed-point container widths a `VCVT` can
/// name — so the shift is always in range.
const fn sign_extend(value: u32, bits: u32) -> i64 {
    if bits >= 32 {
        return value as i32 as i64;
    }
    let v = (value as u64) << (64 - bits);
    (v as i64) >> (64 - bits)
}

/// Scale by `2^shift`, for the fixed-point conversions.
///
/// A real multiply rather than an exponent poke, because the overflow and
/// underflow a conversion at an extreme scale genuinely can produce then come
/// out of the same rounding step as everything else. `FixedToFP` and
/// `FPToFixed` both spell it as a multiplication too. The count is at most
/// thirty-two, so the loop is bounded by the encoding.
#[must_use]
pub fn scale_by_pow2(value: u32, shift: i32, env: Env) -> (u32, Flags) {
    /// `2.0`.
    const TWO: u32 = 0x4000_0000;
    /// `0.5`.
    const HALF: u32 = 0x3f00_0000;
    let (step, count) = if shift >= 0 {
        (TWO, shift)
    } else {
        (HALF, -shift)
    };
    let mut out = value;
    let mut flags = Flags::NONE;
    for _ in 0..count {
        let (v, f) = mul(out, step, env);
        out = v;
        flags |= f;
    }
    (out, flags)
}

/// `VFPExpandImm` for single precision (DDI 0403E A7.7.239).
///
/// The eight-bit field is a sign, a three-bit exponent offset from `1.0` and a
/// four-bit significand: `imm8<7> : NOT(imm8<6>) : Replicate(imm8<6>, 5) :
/// imm8<5:4> : imm8<3:0> : Zeros(19)`.
#[must_use]
pub const fn expand_imm(imm8: u8) -> u32 {
    let imm8 = imm8 as u32;
    let sign = (imm8 >> 7) & 1;
    let b6 = (imm8 >> 6) & 1;
    let replicated: u32 = if b6 != 0 { 0x1f } else { 0 };
    let exp = ((b6 ^ 1) << 7) | (replicated << 2) | ((imm8 >> 4) & 3);
    let frac = (imm8 & 0xf) << 19;
    (sign << 31) | (exp << 23) | frac
}
