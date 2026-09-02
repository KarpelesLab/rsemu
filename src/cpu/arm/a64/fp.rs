//! The SIMD&FP register file, `FPCR`/`FPSR`, and A64's floating-point rules.
//!
//! The arithmetic itself is [`crate::float`] and none of it is here: this
//! module is the *paperwork* that turns a shared IEEE-754 implementation into
//! an Arm FPU. Three jobs, and they are separable on purpose:
//!
//! 1. **The register file.** Thirty-two 128-bit registers, addressed at five
//!    widths (`B`, `H`, `S`, `D`, `Q`) by the same number.
//! 2. **`FPCR` and `FPSR` as a [`float::Env`].** Every guest-specific choice
//!    IEEE leaves open — which NaN survives, when tininess is detected,
//!    whether subnormals are flushed, what an out-of-range conversion gives —
//!    is a field of `Env`, and `Env::ARM` already spells Arm's answers. This
//!    module only decides *which* profile the mode bits select.
//! 3. **The handful of rules that are Arm's own pseudocode rather than
//!    IEEE's**, where calling `float` directly would be subtly wrong. There
//!    are three, and each has a test: `FPMulAdd`'s operand order and its
//!    quiet-NaN-times-infinity override, `FPMaxNum`'s substitution of an
//!    infinity for a quiet NaN, and `FPCompare`'s four-way `NZCV`.
//!
//! # Why a 64-bit value is the currency
//!
//! Every scalar operation here is at most 64 bits wide, so operands and
//! results are `u64` bit patterns and a 128-bit register is only ever *read
//! from* or *written to* at a width. That keeps this module's interface the
//! same shape as `float`'s, which is deliberate: an Advanced SIMD element
//! operation, when one lands, is this arithmetic applied lanewise and needs
//! nothing new from either.
//!
//! # What a write to a SIMD&FP register does to the rest of it
//!
//! DDI 0487 C1.2.2: a scalar floating-point instruction writes its result to
//! the bottom of the destination register **and zeroes everything above it**.
//! That is not an optimisation and it is guest-visible — an `FADD S0` after an
//! `LDR Q0` leaves `V0.D[1]` zero, and software relies on it — so
//! [`Vregs::write`] does it rather than leaving it to each caller to remember.
//!
//! # Deviations, stated rather than hidden
//!
//! * **`FPCR.AHP` is RES0 here.** The alternative half-precision format (no
//!   infinities, no NaNs, one more exponent value) is a second encoding of
//!   binary16 that only `FCVT` and the SIMD widening/narrowing pair can reach.
//!   This core does not implement it, so the bit reads back as zero after a
//!   write and a guest that wanted it can tell — which is the honest failure.
//!   Silently ignoring a set `AHP` would make every half conversion quietly
//!   wrong instead.
//! * **The exception-enable bits (`FPCR.IDE`, `IXE`, `UFE`, `OFE`, `DZE`,
//!   `IOE`) are RES0.** Trapped floating-point exceptions are OPTIONAL in
//!   Armv8-A (DDI 0487 D1: "if trapped exceptions are not implemented, these
//!   bits are RES0"), and a core that has none must read them as zero. The
//!   cumulative bits in `FPSR` are what a guest actually uses.
//! * **`FPCR.FZ16` is RES0**, because half-precision *arithmetic*
//!   (`FEAT_FP16`) is not implemented; half exists here only as a conversion
//!   target, and Armv8.0-A has that unconditionally.
//! * **`FPSR.QC`** (cumulative saturation) is storage: nothing here saturates,
//!   because that is an Advanced SIMD integer property.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487):
//! chapter C3.5 and C7 for the scalar floating-point instructions, D1 for
//! `FPCR`/`FPSR`, and the shared pseudocode chapter for `FPUnpack`,
//! `FPProcessNaN`, `FPProcessNaNs`, `FPProcessNaNs3`, `FPMulAdd`, `FPMax`,
//! `FPMaxNum`, `FPCompare`, `FPRoundInt`, `FPToFixed`, `FixedToFP` and
//! `VFPExpandImm`. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use crate::float::{self, B16, B32, B64, Category, Env, Flags, Round};

use super::isa::Nzcv;

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// How many SIMD&FP registers there are.
///
/// Thirty-two, and — unlike the general registers — all thirty-two are real:
/// there is no `V31` that means something else, because A64 spells the stack
/// pointer and the zero register in the *general* file only.
pub const V_COUNT: usize = 32;

/// The names a disassembler and a debugger print for the SIMD&FP registers at
/// their full width.
pub const V_NAMES: [&str; V_COUNT] = [
    "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14",
    "v15", "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23", "v24", "v25", "v26", "v27",
    "v28", "v29", "v30", "v31",
];

/// The thirty-two 128-bit SIMD&FP registers.
///
/// A `u128` per register rather than two `u64`s: `LDR Q0` and `STR Q0` move
/// all sixteen bytes as one access as far as the guest is concerned, and
/// splitting the storage would put a byte-order decision in a place that has
/// no business making one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Vregs {
    v: [u128; V_COUNT],
}

impl Vregs {
    /// The reset state: architecturally UNKNOWN, and zero here.
    #[must_use]
    pub const fn new() -> Vregs {
        Vregs { v: [0; V_COUNT] }
    }

    /// The whole 128-bit register.
    #[inline]
    #[must_use]
    pub const fn q(&self, index: u32) -> u128 {
        self.v[(index & 31) as usize]
    }

    /// Overwrite the whole 128-bit register.
    #[inline]
    pub const fn set_q(&mut self, index: u32, value: u128) {
        self.v[(index & 31) as usize] = value;
    }

    /// Read the low `bytes` bytes of a register, zero-extended to 64.
    ///
    /// `bytes` is at most eight; the sixteen-byte width is [`Vregs::q`],
    /// because it does not fit in the currency the arithmetic uses.
    #[inline]
    #[must_use]
    pub const fn read(&self, index: u32, bytes: u64) -> u64 {
        let value = self.v[(index & 31) as usize] as u64;
        if bytes >= 8 {
            value
        } else {
            value & ((1u64 << (8 * bytes)) - 1)
        }
    }

    /// Write the low `bytes` bytes of a register, **zeroing the rest**.
    ///
    /// DDI 0487 C1.2.2. The zeroing is the part worth stating: it is what
    /// makes `FMOV S0, S1` clear `V0.D[1]` rather than merge into it.
    #[inline]
    pub const fn write(&mut self, index: u32, bytes: u64, value: u64) {
        let masked = if bytes >= 8 {
            value
        } else {
            value & ((1u64 << (8 * bytes)) - 1)
        };
        self.v[(index & 31) as usize] = masked as u128;
    }

    /// The top half of a register, which `FMOV Xd, Vn.D[1]` reads.
    #[inline]
    #[must_use]
    pub const fn high(&self, index: u32) -> u64 {
        (self.v[(index & 31) as usize] >> 64) as u64
    }

    /// Write the top half of a register, leaving the bottom alone — the one
    /// place a SIMD&FP write is a merge rather than a replacement, which is
    /// exactly why `FMOV Vd.D[1], Xn` has its own encoding.
    #[inline]
    pub const fn set_high(&mut self, index: u32, value: u64) {
        let slot = &mut self.v[(index & 31) as usize];
        *slot = (*slot & 0xffff_ffff_ffff_ffff) | ((value as u128) << 64);
    }
}

impl Default for Vregs {
    fn default() -> Vregs {
        Vregs::new()
    }
}

// ---------------------------------------------------------------------------
// Precision
// ---------------------------------------------------------------------------

/// Which floating-point format an encoding's `ptype` field names.
///
/// DDI 0487: `ptype` is bits 23:22 of every scalar floating-point encoding —
/// `00` single, `01` double, `11` half. `10` is unallocated, which is why this
/// decodes to an `Option` rather than defaulting to something.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Prec {
    /// binary16. Arm's `H`.
    Half,
    /// binary32. Arm's `S`.
    Single,
    /// binary64. Arm's `D`.
    Double,
}

impl Prec {
    /// Decode a `ptype` field. `None` is the unallocated `0b10`.
    #[must_use]
    pub const fn from_ptype(bits: u32) -> Option<Prec> {
        match bits & 3 {
            0b00 => Some(Prec::Single),
            0b01 => Some(Prec::Double),
            0b11 => Some(Prec::Half),
            _ => None,
        }
    }

    /// The width in bytes.
    #[must_use]
    pub const fn bytes(self) -> u64 {
        match self {
            Prec::Half => 2,
            Prec::Single => 4,
            Prec::Double => 8,
        }
    }

    /// The letter a register of this width is printed with.
    #[must_use]
    pub const fn letter(self) -> char {
        match self {
            Prec::Half => 'h',
            Prec::Single => 's',
            Prec::Double => 'd',
        }
    }
}

/// Dispatch a `float` function on a [`Prec`].
///
/// A macro rather than a trait object because the format is a *type* parameter
/// in `float` — that is what makes one implementation serve three formats with
/// no dynamic dispatch and no duplicated arithmetic.
macro_rules! by_prec {
    ($prec:expr, $f:ident $(, $arg:expr)*) => {
        match $prec {
            Prec::Half => float::$f::<B16>($($arg),*),
            Prec::Single => float::$f::<B32>($($arg),*),
            Prec::Double => float::$f::<B64>($($arg),*),
        }
    };
}

// ---------------------------------------------------------------------------
// FPCR and FPSR
// ---------------------------------------------------------------------------

/// The `FPCR` bits this core acts on (DDI 0487 D17.2.x, `FPCR`).
pub mod fpcr {
    /// Rounding mode, bits 23:22.
    ///
    /// **Not x86's order**, and not the order `float::Round` lists them in:
    /// `00` nearest-even, `01` toward `+∞`, `10` toward `−∞`, `11` toward
    /// zero. Assuming otherwise gives a core that rounds up where the guest
    /// asked for down, which is a bug no ordinary test finds.
    pub const RMODE_SHIFT: u32 = 22;
    /// The rounding-mode field, in place.
    pub const RMODE: u64 = 3 << RMODE_SHIFT;
    /// Flush-to-zero, for single and double precision.
    pub const FZ: u64 = 1 << 24;
    /// Default NaN: every propagated NaN is replaced by the default one.
    pub const DN: u64 = 1 << 25;

    /// Every bit a write may set. Everything else is RES0 — see the module
    /// documentation for which, and why each is not implemented rather than
    /// forgotten.
    pub const WRITABLE: u64 = RMODE | FZ | DN;
}

/// The `FPSR` bits this core acts on (DDI 0487 D17.2.x, `FPSR`).
pub mod fpsr {
    /// Invalid operation, cumulative.
    pub const IOC: u64 = 1 << 0;
    /// Divide by zero, cumulative.
    pub const DZC: u64 = 1 << 1;
    /// Overflow, cumulative.
    pub const OFC: u64 = 1 << 2;
    /// Underflow, cumulative.
    pub const UFC: u64 = 1 << 3;
    /// Inexact, cumulative.
    pub const IXC: u64 = 1 << 4;
    /// Input denormal, cumulative. Set only when `FPCR.FZ` flushed an operand.
    pub const IDC: u64 = 1 << 7;
    /// Cumulative saturation, an Advanced SIMD integer flag. Storage here.
    pub const QC: u64 = 1 << 27;

    /// Every bit a write may set.
    ///
    /// Bits 31:28 are deliberately absent: those are the *AArch32*
    /// floating-point condition flags, and this core implements AArch64 only
    /// (`ID_AA64PFR0_EL1` says EL0 and EL1 are AArch64-only), so they are RES0
    /// here. In AArch64 the condition flags live in `PSTATE.NZCV`, which
    /// `FCMP` writes and `MRS Xt, NZCV` reads.
    pub const WRITABLE: u64 = IOC | DZC | OFC | UFC | IXC | IDC | QC;
}

/// The rounding direction `FPCR.RMode` selects.
#[must_use]
pub const fn rounding(fpcr: u64) -> Round {
    match (fpcr >> fpcr::RMODE_SHIFT) & 3 {
        0b00 => Round::TiesEven,
        0b01 => Round::TowardPositive,
        0b10 => Round::TowardNegative,
        _ => Round::TowardZero,
    }
}

/// The floating-point environment `FPCR` describes, for an operation at
/// `prec`.
///
/// `FPCR.FZ` deliberately does **not** apply at half precision: flushing there
/// is `FPCR.FZ16`, which needs `FEAT_FP16` and is RES0 on this core. A guest
/// that sets `FZ` and then converts a subnormal half must still see the exact
/// value, and folding the two bits together is how that stops being true.
#[must_use]
pub fn env(fpcr: u64, prec: Prec) -> Env {
    let base = if fpcr & fpcr::DN != 0 {
        Env::ARM_DEFAULT_NAN
    } else {
        Env::ARM
    };
    let flush = fpcr & fpcr::FZ != 0 && prec != Prec::Half;
    base.round(rounding(fpcr)).flush(flush)
}

/// Fold a set of exceptions into `FPSR`'s cumulative bits.
///
/// The flags are sticky: an operation only ever sets them, and nothing but a
/// guest write to `FPSR` clears one (IEEE 754-2019 §7.1).
#[inline]
pub fn accumulate(fpsr: &mut u64, flags: Flags) {
    *fpsr |= u64::from(flags.to_fpsr());
}

// ---------------------------------------------------------------------------
// The arithmetic, at a precision
// ---------------------------------------------------------------------------

/// `a + b`.
#[must_use]
pub fn add(prec: Prec, a: u64, b: u64, env: Env) -> (u64, Flags) {
    by_prec!(prec, add, a, b, env)
}

/// `a - b`.
#[must_use]
pub fn sub(prec: Prec, a: u64, b: u64, env: Env) -> (u64, Flags) {
    by_prec!(prec, sub, a, b, env)
}

/// `a * b`.
#[must_use]
pub fn mul(prec: Prec, a: u64, b: u64, env: Env) -> (u64, Flags) {
    by_prec!(prec, mul, a, b, env)
}

/// `a / b`.
#[must_use]
pub fn div(prec: Prec, a: u64, b: u64, env: Env) -> (u64, Flags) {
    by_prec!(prec, div, a, b, env)
}

/// The square root of `a`.
#[must_use]
pub fn sqrt(prec: Prec, a: u64, env: Env) -> (u64, Flags) {
    by_prec!(prec, sqrt, a, env)
}

/// Which of the ten IEEE classes a value belongs to. Raises nothing.
#[must_use]
pub fn classify(prec: Prec, a: u64) -> Category {
    by_prec!(prec, classify, a)
}

/// The sign bit of a value at this precision.
#[must_use]
pub const fn sign_bit(prec: Prec) -> u64 {
    1u64 << (8 * prec.bytes() - 1)
}

/// `FNEG`: flip the sign bit, NaNs included, raising nothing.
///
/// DDI 0487 `FPNeg` is a bit operation and not an arithmetic one, which is why
/// `FNEG` of a signaling NaN does **not** raise invalid and does not quieten
/// it. Computing it as `0 - x` would get both wrong, and would also turn
/// `-0.0` into `+0.0` under round-toward-negative.
#[must_use]
pub const fn neg(prec: Prec, a: u64) -> u64 {
    a ^ sign_bit(prec)
}

/// `FABS`: clear the sign bit, on the same terms as [`neg`].
#[must_use]
pub const fn abs(prec: Prec, a: u64) -> u64 {
    a & !sign_bit(prec)
}

/// `FRINTx`: round to an integral value in the same format.
///
/// `signal_inexact` is `FRINTX`'s and nothing else's.
#[must_use]
pub fn round_int(prec: Prec, a: u64, env: Env, signal_inexact: bool) -> (u64, Flags) {
    by_prec!(prec, round_to_integral, a, env, signal_inexact)
}

/// `FMAX`/`FMIN`: the comparison propagates a NaN.
#[must_use]
pub fn max_min(prec: Prec, a: u64, b: u64, want_min: bool, env: Env) -> (u64, Flags) {
    if want_min {
        by_prec!(prec, min, a, b, env)
    } else {
        by_prec!(prec, max, a, b, env)
    }
}

/// `FMAXNM`/`FMINNM`: a **quiet** NaN loses to a number.
///
/// DDI 0487 `FPMaxNum` is written as a substitution followed by `FPMax`, and
/// it is written that way here for the same reason: only a quiet NaN is
/// replaced, only when the other operand is not also one, and a signaling NaN
/// still reaches `FPMax` and still raises invalid there. Reaching for
/// `float`'s `MinMax::NonNan` instead would get the signaling case wrong,
/// because that rule returns the *other operand* for any NaN.
#[must_use]
pub fn max_min_num(prec: Prec, a: u64, b: u64, want_min: bool, env: Env) -> (u64, Flags) {
    let quiet = |v: u64| classify(prec, v) == Category::QuietNan;
    // The identity element: `-∞` for a maximum, `+∞` for a minimum.
    let identity = inf(prec, !want_min);
    let (a, b) = match (quiet(a), quiet(b)) {
        (true, false) => (identity, b),
        (false, true) => (a, identity),
        _ => (a, b),
    };
    max_min(prec, a, b, want_min, env)
}

/// An infinity of the given sign, at a precision.
#[must_use]
pub const fn inf(prec: Prec, negative: bool) -> u64 {
    let value = match prec {
        Prec::Half => 0x7c00,
        Prec::Single => 0x7f80_0000,
        Prec::Double => 0x7ff0_0000_0000_0000,
    };
    if negative {
        value | sign_bit(prec)
    } else {
        value
    }
}

/// `FMADD` and its three siblings: `addend + op1 * op2`, rounded once.
///
/// Two things here are Arm's and not IEEE's, and both are why this is not a
/// call straight through to `float::fma`:
///
/// * **The NaN order is the addend first.** `FPProcessNaNs3(typeA, type1,
///   type2, …)` looks for a signaling NaN in the addend, then `op1`, then
///   `op2`, and only then for a quiet one in the same order. `float::fma`'s
///   arguments are `a * b + c`, so passing them straight through would put the
///   addend last and return a different NaN payload from real silicon.
/// * **A quiet-NaN addend loses to `∞ × 0`.** If the addend is a quiet NaN
///   *and* the product is `∞ × 0`, the result is the **default** NaN with
///   invalid raised — the propagation is overridden. That is a genuine special
///   case in `FPMulAdd`, it fires before the ordinary multiply-add path, and
///   nothing about IEEE 754 predicts it.
#[must_use]
pub fn mul_add(prec: Prec, addend: u64, op1: u64, op2: u64, env: Env) -> (u64, Flags) {
    let class = |v: u64| classify(prec, v);
    let (ca, c1, c2) = (class(addend), class(op1), class(op2));
    let is_inf = |c: Category| matches!(c, Category::NegativeInfinity | Category::PositiveInfinity);
    let is_zero = |c: Category| matches!(c, Category::NegativeZero | Category::PositiveZero);

    // The override, which beats the propagation below: it fires on a *quiet*
    // NaN addend only, so a signaling one still propagates through
    // `process_nans`.
    if ca == Category::QuietNan && ((is_inf(c1) && is_zero(c2)) || (is_zero(c1) && is_inf(c2))) {
        return (default_nan(prec, env), Flags::INVALID);
    }
    if let Some(result) = process_nans(prec, [addend, op1, op2], env) {
        return result;
    }
    by_prec!(prec, fma, op1, op2, addend, env)
}

/// `FPProcessNaNs3`: which NaN a three-operand instruction returns.
///
/// Written out here rather than borrowed from `float` because the *order* is
/// the whole content of the rule and `float::fma`'s argument order is
/// `a * b + c`, which puts the addend last. Arm searches the addend first, for
/// a signaling NaN across all three and only then for a quiet one — and
/// `float::fma` also checks `∞ × 0` before it looks at NaNs at all, which is
/// IEEE's order and not Arm's.
///
/// `None` means no operand was a NaN and the caller should do arithmetic.
fn process_nans(prec: Prec, ops: [u64; 3], env: Env) -> Option<(u64, Flags)> {
    let class = |v: u64| classify(prec, v);
    let signaling = ops.iter().any(|&v| class(v) == Category::SignalingNan);
    let flags = if signaling {
        Flags::INVALID
    } else {
        Flags::NONE
    };
    let pick = ops
        .iter()
        .find(|&&v| class(v) == Category::SignalingNan)
        .or_else(|| ops.iter().find(|&&v| class(v) == Category::QuietNan))?;
    if env.nan.propagate == float::Propagate::Default {
        return Some((default_nan(prec, env), flags));
    }
    // A propagated NaN always comes back quiet, sign and payload intact
    // (IEEE 754-2019 §6.2.3).
    Some((pick | quiet_bit(prec), flags))
}

/// The significand bit that distinguishes a quiet NaN from a signaling one.
const fn quiet_bit(prec: Prec) -> u64 {
    match prec {
        Prec::Half => 1 << 9,
        Prec::Single => 1 << 22,
        Prec::Double => 1 << 51,
    }
}

/// The environment's default NaN at a precision.
fn default_nan(prec: Prec, env: Env) -> u64 {
    let quiet = match prec {
        Prec::Half => 0x7e00,
        Prec::Single => 0x7fc0_0000,
        Prec::Double => 0x7ff8_0000_0000_0000,
    };
    if env.nan.default_sign {
        quiet | sign_bit(prec)
    } else {
        quiet
    }
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

/// `FCMP`/`FCMPE`: the `NZCV` a floating-point comparison writes.
///
/// DDI 0487 `FPCompare`. The four results are not the integer comparison's:
/// *unordered* is `0b0011`, which sets `C` **and** `V`, so `B.VS` after an
/// `FCMP` is the "was a NaN involved" test and `B.HS` is "greater or equal or
/// unordered". Deriving these from a subtraction would get every one of them
/// wrong.
///
/// `signal_all` is `FCMPE`: it raises invalid for a quiet NaN too, where
/// `FCMP` raises only for a signaling one.
#[must_use]
pub fn compare(prec: Prec, a: u64, b: u64, signal_all: bool, env: Env) -> (Nzcv, Flags) {
    use core::cmp::Ordering;
    // `FPCompare` unpacks both operands *through* `FPCR`, so flush-to-zero
    // applies here as it does to arithmetic: with `FPCR.FZ` set, a subnormal
    // compares equal to a zero of the same sign and sets `FPSR.IDC`. A
    // comparison that read the raw encodings would call the same pair unequal
    // and report nothing — and a guest doing `if (x == 0.0)` after enabling
    // flush-to-zero would take the wrong branch.
    let (a, fa) = flush_operand(prec, a, env);
    let (b, fb) = flush_operand(prec, b, env);
    let inputs = fa | fb;
    match by_prec!(prec, compare, a, b) {
        Some(Ordering::Less) => (Nzcv::new(true, false, false, false), inputs),
        Some(Ordering::Equal) => (Nzcv::new(false, true, true, false), inputs),
        Some(Ordering::Greater) => (Nzcv::new(false, false, true, false), inputs),
        None => {
            let signaling = classify(prec, a) == Category::SignalingNan
                || classify(prec, b) == Category::SignalingNan;
            let flags = if signaling || signal_all {
                Flags::INVALID
            } else {
                Flags::NONE
            };
            (Nzcv::new(false, false, true, true), flags | inputs)
        }
    }
}

/// `FPUnpack`'s treatment of a subnormal operand, as a bit operation.
///
/// The arithmetic in `float` does this itself, from the same [`Env`] fields —
/// this exists only for [`compare`], which reaches `float` through a function
/// that takes no environment because comparison is not arithmetic and raises
/// nothing of its own.
fn flush_operand(prec: Prec, value: u64, env: Env) -> (u64, Flags) {
    let subnormal = matches!(
        classify(prec, value),
        Category::NegativeSubnormal | Category::PositiveSubnormal
    );
    if !subnormal {
        return (value, Flags::NONE);
    }
    let flags = if env.subnormal_inputs.reports() {
        Flags::DENORMAL
    } else {
        Flags::NONE
    };
    let value = if env.subnormal_inputs.flushes() {
        value & sign_bit(prec)
    } else {
        value
    };
    (value, flags)
}

// ---------------------------------------------------------------------------
// Conversion
// ---------------------------------------------------------------------------

/// `FCVT`: convert between two floating-point precisions.
///
/// The environment is asked for twice on purpose: flushing is a property of
/// the *format* being flushed, and `FPCR.FZ` does not reach half precision
/// (see [`env()`]). A narrowing conversion into half must therefore not flush
/// its result, and a widening one out of half must not flush its operand.
#[must_use]
pub fn convert(from: Prec, to: Prec, value: u64, fpcr: u64) -> (u64, Flags) {
    // The operand is unpacked under the source format's rules and the result
    // is rounded under the destination's; when either end is half, neither
    // flushes, which is what taking the stricter of the two gives.
    let env = if from == Prec::Half || to == Prec::Half {
        env(fpcr, Prec::Half)
    } else {
        env(fpcr, from)
    };
    match (from, to) {
        (Prec::Half, Prec::Single) => float::convert::<B16, B32>(value, env),
        (Prec::Half, Prec::Double) => float::convert::<B16, B64>(value, env),
        (Prec::Single, Prec::Half) => float::convert::<B32, B16>(value, env),
        (Prec::Single, Prec::Double) => float::convert::<B32, B64>(value, env),
        (Prec::Double, Prec::Half) => float::convert::<B64, B16>(value, env),
        (Prec::Double, Prec::Single) => float::convert::<B64, B32>(value, env),
        // Same-format `FCVT` is unallocated and never decodes; answering the
        // operand back is the only harmless thing to do if it ever did.
        _ => (value, Flags::NONE),
    }
}

/// `FCVTZS`/`FCVTNS`/… and their unsigned halves: float to integer.
///
/// `bits` is 32 or 64. Out of range saturates and a NaN gives zero, both with
/// invalid raised — [`float::IntOverflow::SaturateNanZero`], which is what
/// `Env::ARM` already carries.
#[must_use]
pub fn to_int(prec: Prec, value: u64, bits: u32, signed: bool, env: Env) -> (u64, Flags) {
    if signed {
        let (v, f) = by_prec!(prec, to_signed, value, bits, env);
        (v as u64, f)
    } else {
        by_prec!(prec, to_unsigned, value, bits, env)
    }
}

/// `SCVTF`/`UCVTF`: integer to float.
#[must_use]
pub fn from_int(prec: Prec, value: u64, bits: u32, signed: bool, env: Env) -> (u64, Flags) {
    if signed {
        by_prec!(prec, from_signed, value as i64, bits, env)
    } else {
        by_prec!(prec, from_unsigned, value, bits, env)
    }
}

/// Scale a floating-point value by `2^shift`, for the fixed-point conversions.
///
/// Multiplying by a power of two is exact whenever it does not leave the
/// format's range, so this is a real multiply rather than an exponent poke:
/// the underflow and overflow a fixed-point conversion at an extreme scale
/// genuinely can produce then come out of the same rounding step as everything
/// else. `FixedToFP` and `FPToFixed` both spell it as a multiplication too.
#[must_use]
pub fn scale_by_pow2(prec: Prec, value: u64, shift: i32, env: Env) -> (u64, Flags) {
    let mut out = value;
    let mut flags = Flags::NONE;
    let (step, count) = if shift >= 0 {
        (two(prec), shift)
    } else {
        (half(prec), -shift)
    };
    // A fixed-point scale is at most 64, so this is a bounded loop over a
    // small count rather than a shift that has to worry about overflowing an
    // exponent field.
    for _ in 0..count {
        let (v, f) = mul(prec, out, step, env);
        out = v;
        flags |= f;
    }
    (out, flags)
}

/// `2.0` at a precision.
const fn two(prec: Prec) -> u64 {
    match prec {
        Prec::Half => 0x4000,
        Prec::Single => 0x4000_0000,
        Prec::Double => 0x4000_0000_0000_0000,
    }
}

/// `0.5` at a precision.
const fn half(prec: Prec) -> u64 {
    match prec {
        Prec::Half => 0x3800,
        Prec::Single => 0x3f00_0000,
        Prec::Double => 0x3fe0_0000_0000_0000,
    }
}

/// `VFPExpandImm`: the 8-bit immediate `FMOV` carries.
///
/// DDI 0487 shared pseudocode. The immediate is a sign, a three-bit exponent
/// offset and a four-bit fraction, expanded into the destination format — so
/// `#0x70` is `1.0` at every precision and the *same* eight bits mean
/// different numbers of significand zeroes in each.
#[must_use]
pub const fn expand_imm(imm8: u32, prec: Prec) -> u64 {
    let (exp_bits, frac_bits) = match prec {
        Prec::Half => (5u32, 10u32),
        Prec::Single => (8, 23),
        Prec::Double => (11, 52),
    };
    let sign = ((imm8 >> 7) & 1) as u64;
    let b = ((imm8 >> 6) & 1) as u64;
    // exp = NOT(b) : Replicate(b, E-3) : imm8<5:4>
    let mut exp = 1 - b;
    let mut i = 0;
    while i < exp_bits - 3 {
        exp = (exp << 1) | b;
        i += 1;
    }
    exp = (exp << 2) | (((imm8 >> 4) & 3) as u64);
    let frac = ((imm8 & 0xf) as u64) << (frac_bits - 4);
    (sign << (exp_bits + frac_bits)) | (exp << frac_bits) | frac
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Host `f64` bits, used only to write readable expectations — the
    /// implementation itself never sees a host float.
    fn d(v: f64) -> u64 {
        v.to_bits()
    }

    fn s(v: f32) -> u64 {
        u64::from(v.to_bits())
    }

    /// `ROADMAP.md` §9.1, one level up.
    ///
    /// `src/float` asserts that *its* sources contain no host floating point.
    /// That is necessary and not sufficient: a core could compute a guest's
    /// arithmetic in software and then reach for an `f64` anyway — to
    /// normalise a comparison, to print an immediate, to take a square root
    /// the subsystem does not expose — and a state hash would stop being
    /// reproducible across hosts for exactly one instruction. So the same test
    /// runs over the two files that carry the guest's floating point: this
    /// one, and the interpreter that calls it.
    ///
    /// Comments and the test modules are excluded, because both may name the
    /// host types freely — this file's own tests write their expectations as
    /// `d(1.5)` precisely so they read as arithmetic.
    #[test]
    fn no_host_float_on_the_guest_path() {
        let sources = [
            ("fp.rs", include_str!("fp.rs")),
            ("exec.rs", include_str!("exec.rs")),
        ];
        for (name, src) in sources {
            // Everything from the test module down is exempt.
            let body = src.split("#[cfg(test)]").next().unwrap_or(src);
            for (n, line) in body.lines().enumerate() {
                let code = match line.find("//") {
                    Some(i) => &line[..i],
                    None => line,
                };
                for needle in ["f16", "f32", "f64", "f128", "sqrtf", "libm"] {
                    assert!(
                        !code.contains(needle),
                        "{name}:{}: `{needle}` on the guest path — guest \
                         floating point must go through `crate::float`",
                        n + 1
                    );
                }
            }
        }
    }

    #[test]
    fn a_scalar_write_zeroes_the_rest_of_the_register() {
        let mut v = Vregs::new();
        v.set_q(0, u128::MAX);
        v.write(0, 4, 0x1234_5678);
        assert_eq!(v.q(0), 0x1234_5678);
        // ... and the 128-bit write does not.
        v.set_q(1, 0xdead_beef_cafe_f00d_0123_4567_89ab_cdef);
        assert_eq!(v.read(1, 8), 0x0123_4567_89ab_cdef);
        assert_eq!(v.high(1), 0xdead_beef_cafe_f00d);
    }

    #[test]
    fn the_high_half_is_a_merge_and_the_low_half_is_not() {
        let mut v = Vregs::new();
        v.set_q(3, u128::MAX);
        v.set_high(3, 0);
        assert_eq!(v.q(3), 0x0000_0000_0000_0000_ffff_ffff_ffff_ffff);
        v.write(3, 8, 1);
        assert_eq!(v.q(3), 1);
    }

    /// The rounding-mode encoding is Arm's, and it is not x86's — `01` is
    /// toward `+∞` here and toward `−∞` there.
    #[test]
    fn the_rounding_mode_field_is_arms_order() {
        assert_eq!(rounding(0), Round::TiesEven);
        assert_eq!(rounding(1 << 22), Round::TowardPositive);
        assert_eq!(rounding(2 << 22), Round::TowardNegative);
        assert_eq!(rounding(3 << 22), Round::TowardZero);
    }

    #[test]
    fn flush_to_zero_does_not_reach_half_precision() {
        let fz = fpcr::FZ;
        assert!(env(fz, Prec::Single).flush_outputs);
        assert!(!env(fz, Prec::Half).flush_outputs);
        assert!(env(0, Prec::Single).nan.propagate == float::Propagate::SignalingFirst);
        assert!(env(fpcr::DN, Prec::Single).nan.propagate == float::Propagate::Default);
    }

    /// `VFPExpandImm` against the values the manual tabulates.
    #[test]
    fn the_move_immediate_expands_as_the_pseudocode_says() {
        assert_eq!(expand_imm(0x70, Prec::Double), d(1.0));
        assert_eq!(expand_imm(0x70, Prec::Single), s(1.0));
        assert_eq!(expand_imm(0x70, Prec::Half), 0x3c00);
        assert_eq!(expand_imm(0xf0, Prec::Double), d(-1.0));
        assert_eq!(expand_imm(0x00, Prec::Double), d(2.0));
        assert_eq!(expand_imm(0x60, Prec::Double), d(0.5));
        assert_eq!(expand_imm(0x2e, Prec::Single), s(15.0));
    }

    /// DDI 0487 `FPCompare`: unordered sets `C` and `V`, which no integer
    /// comparison ever does.
    #[test]
    fn an_unordered_comparison_sets_c_and_v() {
        let nan = d(f64::NAN);
        let (flags, exc) = compare(Prec::Double, nan, d(1.0), false, Env::ARM);
        assert_eq!(flags, Nzcv::new(false, false, true, true));
        // `FCMP` is quiet about a quiet NaN; `FCMPE` is not.
        assert_eq!(exc, Flags::NONE);
        assert_eq!(
            compare(Prec::Double, nan, d(1.0), true, Env::ARM).1,
            Flags::INVALID
        );
        // A signaling NaN raises under both.
        let snan = 0x7ff0_0000_0000_0001;
        assert_eq!(
            compare(Prec::Double, snan, d(1.0), false, Env::ARM).1,
            Flags::INVALID
        );
        assert_eq!(
            compare(Prec::Double, d(1.0), d(2.0), false, Env::ARM).0,
            Nzcv::new(true, false, false, false)
        );
        assert_eq!(
            compare(Prec::Double, d(2.0), d(2.0), false, Env::ARM).0,
            Nzcv::new(false, true, true, false)
        );
        assert_eq!(
            compare(Prec::Double, d(3.0), d(2.0), false, Env::ARM).0,
            Nzcv::new(false, false, true, false)
        );
    }

    /// `FPCR.FZ` reaches the comparison too: a subnormal flushed to zero
    /// compares *equal* to zero and reports `FPSR.IDC`.
    #[test]
    fn flush_to_zero_reaches_the_comparison() {
        let sub = 1u64; // the smallest positive subnormal
        // Without flushing it is greater than zero and raises nothing.
        let (flags, exc) = compare(Prec::Double, sub, 0, false, Env::ARM);
        assert_eq!(flags, Nzcv::new(false, false, true, false));
        assert_eq!(exc, Flags::NONE);
        // With `FPCR.FZ` it is equal to zero, and the flush is reported.
        let (flags, exc) = compare(Prec::Double, sub, 0, false, Env::ARM.flush(true));
        assert_eq!(flags, Nzcv::new(false, true, true, false));
        assert_eq!(exc, Flags::DENORMAL);
    }

    /// `FMAXNM` drops a quiet NaN and keeps a signaling one, which is the
    /// difference between it and `FMAX`.
    #[test]
    fn the_num_forms_prefer_a_number_to_a_quiet_nan() {
        let e = Env::ARM;
        let qnan = 0x7ff8_0000_0000_0000u64;
        let snan = 0x7ff0_0000_0000_0001u64;
        assert_eq!(max_min_num(Prec::Double, qnan, d(2.0), false, e).0, d(2.0));
        assert_eq!(max_min_num(Prec::Double, d(2.0), qnan, true, e).0, d(2.0));
        // Two quiet NaNs: neither is replaced, so the ordinary propagation
        // rule picks one.
        assert_eq!(max_min_num(Prec::Double, qnan, qnan, false, e).0, qnan);
        // A signaling NaN is not replaced and still raises invalid.
        let (value, flags) = max_min_num(Prec::Double, snan, d(2.0), false, e);
        assert_eq!(flags, Flags::INVALID);
        assert_eq!(value, qnan | 1);
        // `FMAX` propagates the quiet NaN that `FMAXNM` dropped.
        assert_eq!(max_min(Prec::Double, qnan, d(2.0), false, e).0, qnan);
    }

    /// `FPMax` of two zeros takes the most positive sign, and `FPMin` the most
    /// negative — a rule a magnitude comparison alone would call a tie.
    #[test]
    fn the_sign_of_a_zero_decides_a_max() {
        let e = Env::ARM;
        assert_eq!(max_min(Prec::Double, d(0.0), d(-0.0), false, e).0, d(0.0));
        assert_eq!(max_min(Prec::Double, d(-0.0), d(0.0), false, e).0, d(0.0));
        assert_eq!(max_min(Prec::Double, d(0.0), d(-0.0), true, e).0, d(-0.0));
    }

    /// `FPMulAdd`'s two Arm-specific rules.
    #[test]
    fn multiply_add_takes_its_nan_from_the_addend_first() {
        let e = Env::ARM;
        let qnan_a = 0x7ff8_0000_0000_00aa_u64;
        let qnan_1 = 0x7ff8_0000_0000_00bb_u64;
        // The addend's payload wins over `op1`'s, which is the order
        // `FPProcessNaNs3` specifies and the opposite of `a * b + c`.
        assert_eq!(mul_add(Prec::Double, qnan_a, qnan_1, d(1.0), e).0, qnan_a);
        // A signaling operand still beats a quiet one, wherever it sits.
        let snan_2 = 0x7ff0_0000_0000_00cc_u64;
        let (value, flags) = mul_add(Prec::Double, qnan_a, d(1.0), snan_2, e);
        assert_eq!(value, 0x7ff8_0000_0000_00cc);
        assert_eq!(flags, Flags::INVALID);
        // The override: a quiet-NaN addend with an infinity times a zero gives
        // the *default* NaN and raises invalid, discarding the payload.
        let (value, flags) = mul_add(Prec::Double, qnan_a, d(f64::INFINITY), d(0.0), e);
        assert_eq!(value, 0x7ff8_0000_0000_0000);
        assert_eq!(flags, Flags::INVALID);
        // Without a NaN addend it is an ordinary fused multiply-add, rounded
        // once: 1 + 2^-53 * 2^-53 is inexact but not zero.
        let (value, _) = mul_add(Prec::Double, d(1.0), d(3.0), d(4.0), e);
        assert_eq!(value, d(13.0));
    }

    #[test]
    fn negation_is_a_bit_operation_and_not_a_subtraction() {
        // A signaling NaN survives `FNEG` unquieted and raises nothing.
        let snan = 0x7ff0_0000_0000_0001u64;
        assert_eq!(neg(Prec::Double, snan), snan | (1 << 63));
        assert_eq!(abs(Prec::Double, snan | (1 << 63)), snan);
        // ... and `-0.0` negates to `+0.0` rather than staying negative.
        assert_eq!(neg(Prec::Double, d(-0.0)), d(0.0));
    }

    #[test]
    fn the_fixed_point_scale_is_exact_where_it_can_be() {
        let e = Env::ARM;
        assert_eq!(scale_by_pow2(Prec::Double, d(1.0), 4, e).0, d(16.0));
        assert_eq!(scale_by_pow2(Prec::Double, d(1.0), -4, e).0, d(0.0625));
        assert_eq!(scale_by_pow2(Prec::Double, d(3.0), 0, e).0, d(3.0));
        assert_eq!(scale_by_pow2(Prec::Double, d(1.0), 4, e).1, Flags::NONE);
    }
}
