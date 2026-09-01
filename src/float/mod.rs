//! Software IEEE-754 arithmetic — the floating-point unit every guest shares.
//!
//! `ROADMAP.md` §9.1 makes this a named deliverable rather than an assumption:
//! **guest floating point executed on host floating point cannot give
//! bit-identical results across hosts.** x86 and AArch64 disagree about NaN
//! payloads and about flush-to-zero, wasm canonicalises NaNs, x87's 80-bit
//! format exists on no other host at all, and none of them reports the guest's
//! sticky exception flags. So guest arithmetic goes through the integer code
//! below and nothing else — there is no host-float fast path here, not even
//! behind a flag, so there is nothing that can silently be left on. The
//! `no_host_float_in_the_implementation` test reads these sources back and
//! asserts it.
//!
//! # What is implemented
//!
//! * [`binary`] — the interchange formats [`B32`] and [`B64`], as one
//!   implementation over [`Format`], on raw bit patterns held in a `u64`.
//! * [`x87`] — the 80-bit double extended format, with the explicit integer
//!   bit, the unsupported encodings, and x87's precision control.
//! * `add`/`sub`/`mul`/`div`/`sqrt`/`fma`, comparisons, `min`/`max`,
//!   classification, format conversion and both directions of integer
//!   conversion, at all five rounding modes, with exact subnormals and the
//!   sticky exception flags.
//!
//! # What is guest-specific, and is therefore a parameter
//!
//! The arithmetic is the same everywhere; the paperwork around it is not. Each
//! of these is a field of [`Env`], and [`Env`]'s constants are the profiles:
//!
//! | Question | RISC-V | x86 SSE | x87 | ARM (`FPCR.DN=0`) |
//! | --- | --- | --- | --- | --- |
//! | which NaN comes out | the default one | first operand | larger significand | signaling first |
//! | the default NaN's sign | `+` | `-` | `-` | `+` |
//! | tininess detected | after rounding | after rounding | after rounding | before rounding |
//! | subnormals | exact | mode bits | exact | mode bits |
//! | `min`/`max` of a NaN | the other operand | the second operand | — | propagated |
//! | integer conversion out of range | saturates | indefinite | indefinite | saturates |
//!
//! The exception flags are the same five events everywhere, but no two guests
//! agree on the bit order, so [`Flags`] carries them in IEEE's order and hands
//! out each guest's encoding — [`Flags::to_fcsr`], [`Flags::to_mxcsr`],
//! [`Flags::to_fpsr`], [`Flags::to_x87_status`].
//!
//! # Why the results are reproducible
//!
//! Every operation is computed on a wide integer significand that is **exact**
//! before it is rounded once, at the end, by one private rounding step
//! (`kernel::round_exact`). There is no double rounding and no host state:
//! `mul` keeps the full product, `div` develops more quotient bits than the
//! format has so the remainder is only ever a sticky bit, `fma` keeps the whole
//! product and the addend in one 256-bit window, and every alignment shift
//! folds what it drops into a sticky bit rather than discarding it.
//!
//! # Sources
//!
//! * **IEEE 754-2019** for the arithmetic: §3.3 (formats), §4.3 (rounding
//!   attributes), §5 (operations), §6.2 (NaNs), §7 (exceptions).
//! * *The RISC-V Instruction Set Manual, Volume I* (CC-BY-4.0) for the RISC-V
//!   profile: NaN generation and propagation, `fcsr`, FMIN/FMAX, FCVT.
//! * *Intel 64 and IA-32 Architectures Software Developer's Manual, Volume 1*
//!   for the x86 profiles: §4.2.2 (the 80-bit format), §4.8.3.4 and Table 4-7
//!   (NaN handling), §4.9 (the exceptions and `MXCSR`), §8.1.5 (x87's control
//!   word and precision control), §8.2.2 (unsupported encodings).
//! * *Arm Architecture Reference Manual* for the ARM profile: the
//!   floating-point data types, flush-to-zero, `FPCR.DN`, `FPSR`.
//!
//! No emulator source of any licence was consulted for any part of this module
//! (`CLAUDE.md`, provenance).

pub mod binary;
mod kernel;
pub mod x87;

#[cfg(test)]
mod tests;

pub use binary::{
    B32, B64, Category, Format, add, classify, compare, convert, div, eq, fma, from_signed,
    from_unsigned, le, lt, max, min, mul, sqrt, sub, to_signed, to_unsigned,
};

/// A rounding-direction attribute.
///
/// The five are IEEE 754-2019 §4.3.1 (the two round-to-nearest attributes) and
/// §4.3.2 (the three directed attributes). A guest that has fewer simply never
/// names the others: x86 has no ties-away mode, and RISC-V's `RMM` is exactly
/// `roundTiesToAway`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Round {
    /// `roundTiesToEven` — the IEEE default, and every guest's reset state.
    #[default]
    TiesEven,
    /// `roundTiesToAway` — ties away from zero. RISC-V `RMM`; x86 has none.
    TiesAway,
    /// `roundTowardZero` — truncate.
    TowardZero,
    /// `roundTowardNegative` — round down.
    TowardNegative,
    /// `roundTowardPositive` — round up.
    TowardPositive,
}

impl Round {
    /// Decode RISC-V's three-bit `rm` field, or `None` for a reserved encoding.
    ///
    /// Volume I: 0 `RNE`, 1 `RTZ`, 2 `RDN`, 3 `RUP`, 4 `RMM`; 5 and 6 are
    /// reserved and 7 means "use `fcsr.frm`", which the caller has to resolve
    /// before it gets here — an instruction naming a reserved mode is illegal,
    /// and `None` is how that reaches the decoder.
    #[must_use]
    pub const fn from_riscv_rm(bits: u32) -> Option<Round> {
        match bits {
            0 => Some(Round::TiesEven),
            1 => Some(Round::TowardZero),
            2 => Some(Round::TowardNegative),
            3 => Some(Round::TowardPositive),
            4 => Some(Round::TiesAway),
            _ => None,
        }
    }

    /// The RISC-V `rm` encoding of this mode.
    #[must_use]
    pub const fn riscv_rm(self) -> u32 {
        match self {
            Round::TiesEven => 0,
            Round::TowardZero => 1,
            Round::TowardNegative => 2,
            Round::TowardPositive => 3,
            Round::TiesAway => 4,
        }
    }

    /// Decode x86's two-bit rounding control, shared by `MXCSR.RC` and the x87
    /// control word's `RC` (SDM Volume 1, §8.1.5.3 and §10.2.3.1).
    ///
    /// x86 has no ties-away mode, so every encoding is valid and there is no
    /// `None` to return.
    #[must_use]
    pub const fn from_x86_rc(bits: u32) -> Round {
        match bits & 3 {
            0 => Round::TiesEven,
            1 => Round::TowardNegative,
            2 => Round::TowardPositive,
            _ => Round::TowardZero,
        }
    }

    /// This mode's x86 `RC` encoding. `roundTiesToAway` has none and maps to
    /// nearest-even — x86 cannot ask for it, so nothing reaches this with it.
    #[must_use]
    pub const fn x86_rc(self) -> u32 {
        match self {
            Round::TiesEven | Round::TiesAway => 0,
            Round::TowardNegative => 1,
            Round::TowardPositive => 2,
            Round::TowardZero => 3,
        }
    }
}

/// The five sticky exception flags, plus x86's sixth.
///
/// IEEE 754-2019 §7 defines five exceptions and does not define their
/// encoding; every guest picks its own bit order, so this type holds them in
/// the standard's own listing order and converts on the way out. Never cleared
/// by an arithmetic operation — only by a write to the guest's status
/// register.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Flags(pub u32);

impl Flags {
    /// No exception.
    pub const NONE: Flags = Flags(0);
    /// Invalid operation (§7.2).
    pub const INVALID: Flags = Flags(1 << 0);
    /// Division by zero (§7.3).
    pub const DIV_BY_ZERO: Flags = Flags(1 << 1);
    /// Overflow (§7.4).
    pub const OVERFLOW: Flags = Flags(1 << 2);
    /// Underflow (§7.5): the result is tiny *and* inexact.
    pub const UNDERFLOW: Flags = Flags(1 << 3);
    /// Inexact (§7.6).
    pub const INEXACT: Flags = Flags(1 << 4);
    /// Denormal operand — **not** an IEEE exception. x86 alone reports that a
    /// source operand was subnormal (SDM Volume 1, §4.9.1.2); every other
    /// guest here leaves this clear.
    pub const DENORMAL: Flags = Flags(1 << 5);

    /// Whether no exception at all is set.
    #[must_use]
    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Whether every flag in `other` is set here.
    #[must_use]
    #[inline]
    pub const fn contains(self, other: Flags) -> bool {
        self.0 & other.0 == other.0
    }

    /// The union of two flag sets, as a `const fn`.
    #[must_use]
    #[inline]
    pub const fn union(self, other: Flags) -> Flags {
        Flags(self.0 | other.0)
    }

    /// RISC-V `fcsr.fflags`: `NX` 0, `UF` 1, `OF` 2, `DZ` 3, `NV` 4 (Volume I).
    /// There is no denormal flag, so that bit is dropped.
    #[must_use]
    pub const fn to_fcsr(self) -> u32 {
        let mut v = 0;
        if self.contains(Flags::INEXACT) {
            v |= 1 << 0;
        }
        if self.contains(Flags::UNDERFLOW) {
            v |= 1 << 1;
        }
        if self.contains(Flags::OVERFLOW) {
            v |= 1 << 2;
        }
        if self.contains(Flags::DIV_BY_ZERO) {
            v |= 1 << 3;
        }
        if self.contains(Flags::INVALID) {
            v |= 1 << 4;
        }
        v
    }

    /// x86 `MXCSR`: `IE` 0, `DE` 1, `ZE` 2, `OE` 3, `UE` 4, `PE` 5 (SDM
    /// Volume 1, §10.2.3.1). The x87 status word uses the same six bits in the
    /// same order, which is why [`Flags::to_x87_status`] is this function.
    #[must_use]
    pub const fn to_mxcsr(self) -> u32 {
        let mut v = 0;
        if self.contains(Flags::INVALID) {
            v |= 1 << 0;
        }
        if self.contains(Flags::DENORMAL) {
            v |= 1 << 1;
        }
        if self.contains(Flags::DIV_BY_ZERO) {
            v |= 1 << 2;
        }
        if self.contains(Flags::OVERFLOW) {
            v |= 1 << 3;
        }
        if self.contains(Flags::UNDERFLOW) {
            v |= 1 << 4;
        }
        if self.contains(Flags::INEXACT) {
            v |= 1 << 5;
        }
        v
    }

    /// The x87 status word's exception bits (SDM Volume 1, §8.1.3): `IE` 0,
    /// `DE` 1, `ZE` 2, `OE` 3, `UE` 4, `PE` 5 — the same order `MXCSR` uses.
    #[must_use]
    pub const fn to_x87_status(self) -> u32 {
        self.to_mxcsr()
    }

    /// AArch64 `FPSR`: `IOC` 0, `DZC` 1, `OFC` 2, `UFC` 3, `IXC` 4, and the
    /// input-denormal `IDC` at 7 rather than beside the others.
    #[must_use]
    pub const fn to_fpsr(self) -> u32 {
        let mut v = 0;
        if self.contains(Flags::INVALID) {
            v |= 1 << 0;
        }
        if self.contains(Flags::DIV_BY_ZERO) {
            v |= 1 << 1;
        }
        if self.contains(Flags::OVERFLOW) {
            v |= 1 << 2;
        }
        if self.contains(Flags::UNDERFLOW) {
            v |= 1 << 3;
        }
        if self.contains(Flags::INEXACT) {
            v |= 1 << 4;
        }
        if self.contains(Flags::DENORMAL) {
            v |= 1 << 7;
        }
        v
    }
}

impl core::ops::BitOr for Flags {
    type Output = Flags;
    #[inline]
    fn bitor(self, rhs: Flags) -> Flags {
        Flags(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for Flags {
    #[inline]
    fn bitor_assign(&mut self, rhs: Flags) {
        self.0 |= rhs.0;
    }
}

/// Which NaN an operation that has a NaN operand returns.
///
/// IEEE 754-2019 §6.2 *recommends* returning one of the input NaNs, quieted,
/// and leaves the choice to the implementation. Every guest made a different
/// one, and a guest's own software can see the difference, so this is a
/// parameter rather than a convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Propagate {
    /// Never propagate: return the default NaN. RISC-V ("NaN Generation and
    /// Propagation": the canonical NaN, payload discarded) and ARM with
    /// `FPCR.DN` set.
    Default,
    /// The first NaN operand, quieted. x86 SSE (SDM Volume 1, Table 4-7: with
    /// two NaN sources the *first source operand* wins, whatever its kind).
    FirstNan,
    /// A signaling NaN in operand order first, then a quiet one, quieted
    /// either way. ARM with `FPCR.DN` clear (`FPProcessNaNs`).
    SignalingFirst,
    /// x87 (SDM Volume 1, Table 4-7): a quiet NaN paired with a signaling one
    /// wins outright, and two NaNs of the same kind are decided by the larger
    /// significand.
    ///
    /// The manual does not say which operand wins when two same-kind NaNs have
    /// **equal** significands; this implementation takes the first, and the
    /// choice is marked here rather than hidden.
    LargerSignificand,
}

/// The NaN rules of a guest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Nan {
    /// Which input NaN, if any, survives.
    pub propagate: Propagate,
    /// The sign bit of the default NaN — the one an invalid operation with no
    /// NaN input returns. RISC-V's canonical NaN and ARM's default NaN are
    /// positive; x86's "QNaN floating-point indefinite" is negative (SDM
    /// Volume 1, §4.2.2 and Table 4-1).
    pub default_sign: bool,
}

/// When tininess is detected, which decides whether underflow is signalled.
///
/// IEEE 754-2019 §7.5 permits either, and requires an implementation to be
/// consistent about it. The two differ for exactly one class of result: one
/// that is below the smallest normal before rounding but rounds up to it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tininess {
    /// After rounding. RISC-V (Volume I) and x86 (SDM Volume 1 §4.9.1.5
    /// defines underflow on "the magnitude of the rounded result with
    /// unbounded exponent").
    AfterRounding,
    /// Before rounding. ARM (the Underflow exception is defined on "the result
    /// of an operation, produced before rounding").
    BeforeRounding,
}

/// What happens to a subnormal *operand*.
///
/// Four states, each of which is exactly one hardware configuration — which is
/// why this is an enumeration rather than two booleans that can be set to a
/// combination no processor has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputSubnormal {
    /// Used exactly and not reported. RISC-V, and ARM with `FPCR.FZ` clear.
    Exact,
    /// Used exactly and reported: x86 signals the denormal-operand exception
    /// (`#D`) for a subnormal source (SDM Volume 1, §4.9.1.2). SSE with
    /// `MXCSR.DAZ` clear, and x87.
    Flagged,
    /// Replaced by a zero of the same sign and **not** reported —
    /// `MXCSR.DAZ` suppresses the denormal-operand exception it would
    /// otherwise raise (SDM Volume 1, §10.2.3.4).
    Flushed,
    /// Replaced by a zero of the same sign and reported: ARM's `FPCR.FZ` sets
    /// `FPSR.IDC` for an input it flushed.
    FlushedFlagged,
}

impl InputSubnormal {
    /// Whether a subnormal operand is replaced by zero.
    #[must_use]
    #[inline]
    pub const fn flushes(self) -> bool {
        matches!(
            self,
            InputSubnormal::Flushed | InputSubnormal::FlushedFlagged
        )
    }

    /// Whether a subnormal operand is reported.
    #[must_use]
    #[inline]
    pub const fn reports(self) -> bool {
        matches!(
            self,
            InputSubnormal::Flagged | InputSubnormal::FlushedFlagged
        )
    }
}

/// What `min`/`max` do with a NaN, and with two zeros of different sign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MinMax {
    /// The non-NaN operand, or the default NaN if both are NaNs; `-0` is
    /// smaller than `+0`. RISC-V FMIN/FMAX, which is IEEE 754-2019 §9.6's
    /// `minimumNumber`/`maximumNumber`.
    NonNan,
    /// The second source operand whenever the comparison does not strictly
    /// select the first — so a NaN in either operand, and `min(±0, ∓0)`, both
    /// return the second. x86 `MINSS`/`MAXSS` (SDM Volume 2).
    SecondOperand,
    /// The NaN rules of [`Nan::propagate`] apply, and the zero comparison is
    /// signed. ARM `FMIN`/`FMAX` with `FPCR.DN` clear.
    PropagateNan,
}

/// What a float-to-integer conversion does when the value does not fit.
///
/// IEEE 754-2019 §7.2 makes it an invalid operation and does not say what the
/// delivered result is, so this is a parameter with three real answers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IntOverflow {
    /// Saturate to the nearest end of the range; a NaN gives the *most
    /// positive* value. RISC-V FCVT (Volume I).
    SaturateNanMax,
    /// Saturate to the nearest end of the range; a NaN gives zero. ARM
    /// `FCVTZS`/`FCVTZU`.
    SaturateNanZero,
    /// The "integer indefinite" value — the most negative representable
    /// integer — for a NaN, an infinity, and anything out of range. x86
    /// `CVTTSD2SI` and x87 `FIST` (SDM Volume 1, §4.2.2.1). An unsigned
    /// conversion has no such value, so it saturates to zero.
    Indefinite,
}

/// A guest's floating-point personality, plus the dynamic mode bits.
///
/// One value carries everything an operation needs that is not the operands:
/// the rounding direction from the guest's control register or instruction
/// field, and the fixed rules of the architecture. The constants are the
/// profiles; [`Env::round`] sets the direction on one of them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Env {
    /// The rounding direction (IEEE 754-2019 §4.3).
    pub round: Round,
    /// The NaN rules.
    pub nan: Nan,
    /// When tininess is detected.
    pub tininess: Tininess,
    /// What happens to a subnormal operand. x86 `MXCSR.DAZ` and the
    /// denormal-operand exception; ARM `FPCR.FZ`.
    pub subnormal_inputs: InputSubnormal,
    /// Flush a tiny *result* to a zero of the same sign, raising underflow and
    /// inexact. x86 `MXCSR.FTZ`; ARM `FPCR.FZ`.
    pub flush_outputs: bool,
    /// The `min`/`max` rule.
    pub min_max: MinMax,
    /// The float-to-integer overflow rule.
    pub int_overflow: IntOverflow,
}

impl Env {
    /// RISC-V `F`/`D`: one canonical NaN and no payload propagation, subnormals
    /// always exact, tininess after rounding, saturating conversions.
    pub const RISCV: Env = Env {
        round: Round::TiesEven,
        nan: Nan {
            propagate: Propagate::Default,
            default_sign: false,
        },
        tininess: Tininess::AfterRounding,
        subnormal_inputs: InputSubnormal::Exact,
        flush_outputs: false,
        min_max: MinMax::NonNan,
        int_overflow: IntOverflow::SaturateNanMax,
    };

    /// x86 SSE with `MXCSR.FTZ` and `.DAZ` clear — see [`Env::daz`] and
    /// [`Env::ftz`] for those two bits.
    pub const X86_SSE: Env = Env {
        round: Round::TiesEven,
        nan: Nan {
            propagate: Propagate::FirstNan,
            default_sign: true,
        },
        tininess: Tininess::AfterRounding,
        subnormal_inputs: InputSubnormal::Flagged,
        flush_outputs: false,
        min_max: MinMax::SecondOperand,
        int_overflow: IntOverflow::Indefinite,
    };

    /// The x87 FPU. Differs from [`Env::X86_SSE`] in its NaN rule, and it has
    /// no flush-to-zero at all — the 80-bit format's exponent range is wide
    /// enough that Intel never added one.
    pub const X87: Env = Env {
        nan: Nan {
            propagate: Propagate::LargerSignificand,
            default_sign: true,
        },
        min_max: MinMax::PropagateNan,
        ..Env::X86_SSE
    };

    /// AArch64/AArch32 with `FPCR.DN` clear and `FPCR.FZ` clear.
    pub const ARM: Env = Env {
        round: Round::TiesEven,
        nan: Nan {
            propagate: Propagate::SignalingFirst,
            default_sign: false,
        },
        tininess: Tininess::BeforeRounding,
        subnormal_inputs: InputSubnormal::Exact,
        flush_outputs: false,
        min_max: MinMax::PropagateNan,
        int_overflow: IntOverflow::SaturateNanZero,
    };

    /// ARM with `FPCR.DN` set: the default NaN replaces every propagated one.
    pub const ARM_DEFAULT_NAN: Env = Env {
        nan: Nan {
            propagate: Propagate::Default,
            default_sign: false,
        },
        ..Env::ARM
    };

    /// This environment with a different rounding direction.
    #[must_use]
    #[inline]
    pub const fn round(self, round: Round) -> Env {
        Env { round, ..self }
    }

    /// This environment with both halves of ARM's `FPCR.FZ`: inputs flushed
    /// and reported through `FPSR.IDC`, and tiny results flushed.
    #[must_use]
    #[inline]
    pub const fn flush(self, flush: bool) -> Env {
        Env {
            subnormal_inputs: if flush {
                InputSubnormal::FlushedFlagged
            } else {
                InputSubnormal::Exact
            },
            flush_outputs: flush,
            ..self
        }
    }

    /// This environment with x86's `MXCSR.DAZ`, which replaces a subnormal
    /// operand with zero *and* suppresses the denormal-operand exception.
    #[must_use]
    #[inline]
    pub const fn daz(self, on: bool) -> Env {
        Env {
            subnormal_inputs: if on {
                InputSubnormal::Flushed
            } else {
                InputSubnormal::Flagged
            },
            ..self
        }
    }

    /// This environment with x86's `MXCSR.FTZ`.
    #[must_use]
    #[inline]
    pub const fn ftz(self, on: bool) -> Env {
        Env {
            flush_outputs: on,
            ..self
        }
    }
}

impl Default for Env {
    /// IEEE 754-2019 §4.3's default attribute, no flushing, no propagation —
    /// the environment a format's own arithmetic is defined in.
    fn default() -> Env {
        Env::RISCV
    }
}

/// The arithmetic parameters of a format, with no encoding attached.
///
/// IEEE 754-2019 §3.3 describes a binary format by its radix, precision `p`
/// and maximum exponent `emax`. The third field is what makes x87's precision
/// control expressible: setting `PC` to 53 changes `precision` and leaves the
/// exponent range — and therefore `min_ulp`, the exponent of the smallest
/// subnormal's last bit — where the 80-bit register has it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Spec {
    /// `p`: how many significand bits the result keeps, counting the leading
    /// one.
    pub precision: u32,
    /// `emax`: the exponent of the largest finite magnitude.
    pub emax: i32,
    /// The exponent of the least significant bit of the smallest subnormal,
    /// which is `emin - (p - 1)` when the rounding precision is the storage
    /// precision and stays put when it is not.
    pub min_ulp: i32,
}

impl Spec {
    /// The parameters of an interchange format: `emin = 1 - emax` and the
    /// subnormal grid follows the storage precision (§3.3).
    #[must_use]
    pub const fn interchange(precision: u32, emax: i32) -> Spec {
        Spec {
            precision,
            emax,
            min_ulp: 1 - emax - (precision as i32 - 1),
        }
    }

    /// `emin`, the exponent of the smallest normal magnitude.
    #[must_use]
    #[inline]
    pub const fn emin(self) -> i32 {
        1 - self.emax
    }

    /// The same exponent range at a shorter precision, which is what x87's
    /// precision control does (SDM Volume 1, §8.1.5.2).
    #[must_use]
    #[inline]
    pub const fn with_precision(self, precision: u32) -> Spec {
        Spec { precision, ..self }
    }
}
