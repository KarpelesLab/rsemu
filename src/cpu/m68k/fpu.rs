//! The MC68881/MC68882 floating-point coprocessor.
//!
//! Everything here is the *MC68881/MC68882 Floating-Point Coprocessor User's
//! Manual* (M68881UM) and M68000PRM §5: §2 for the registers, §3 for the
//! seven data formats, §4 for the instruction set and its condition codes,
//! and §6 for the exception model.
//!
//! # The arithmetic is `src/float`, and only `src/float`
//!
//! `ROADMAP.md` §9.1: guest floating point computed on host floating point
//! cannot be bit-identical across hosts, so none of it is. Every value below
//! is an [`F80`] and every operation goes through [`crate::float::x87`],
//! which is integer arithmetic rounded exactly once. There is no host `f32`
//! or `f64` anywhere in this file and the test at the bottom of
//! `src/float/tests.rs` is what keeps it that way for the shared kernel.
//!
//! # How the 68881's extended format differs from x87's
//!
//! The *value* encoding is the same one: a sign, a fifteen-bit biased
//! exponent, and a sixty-four-bit significand with the integer bit
//! **explicit** (M68881UM Table 3-3; SDM Volume 1 §4.2.2). Four things are
//! not the same, and each is handled here rather than in `src/float`:
//!
//! 1. **In memory it is ninety-six bits, not eighty.** The 68881 stores a
//!    word of sign and exponent, a word of zeros, then the eight significand
//!    bytes, big-endian (Table 3-3). x87 stores ten little-endian bytes.
//!    [`read_extended`] and [`write_extended`] are the whole of that
//!    difference.
//! 2. **Unnormalized numbers are values.** An encoding with a non-zero
//!    exponent and a clear integer bit is what the manual calls
//!    *unnormalized*, and "unnormalized inputs are always converted to
//!    normalized or denormalized numbers or zero before being used"
//!    (M68881UM §3.2.2's note, §3.5.1). x87 calls the same encoding an
//!    *unnormal*, declares it unsupported, and answers an invalid operation
//!    (SDM Volume 1, §8.2.2). So does this core's `x87` module — which is
//!    right for x87 — so every operand is put through [`canonical`] first,
//!    which normalizes it exactly as the 68881 does.
//! 3. **Pseudo-infinities and pseudo-NaNs are ordinary ones.** For an
//!    infinity "the most significant bit of the mantissa (the integer bit) is
//!    a don't care" (Figure 3-6) and for a NaN it "can be either one or zero"
//!    (§3.2.5); x87 rejects both. [`canonical`] sets the bit.
//! 4. **The NaN the unit creates is its own.** "When NANs are created by the
//!    FPCP, the NANs always contain the same bit pattern in the mantissa; for
//!    any precision, all bits of the mantissa are ones" (§3.2.5) — where x87
//!    delivers a *negative* quiet NaN with an all-zero payload. The manual
//!    does not give the sign of the one the 68881 makes; this core uses a
//!    positive one, which is the reading every assembler's `NAN` constant
//!    takes, and says so here rather than leaving it to be discovered.
//!    Generation is intercepted in [`generated_nan`] rather than changed in
//!    `src/float`, because x87's indefinite is x87's.
//!
//! A fifth difference is in the *rounding precision*, not the format: x87's
//! `PC` shortens the significand and leaves the exponent range alone, while
//! the 68881's `FPCR` **PREC** shortens both — "if the single or double
//! precision mode is selected, the exponent value is in the correct range for
//! the single or double precision format" (§2.2.2). That is why this uses
//! `x87`'s `*_to` entry points, which take the format parameters directly,
//! rather than its `Precision`.
//!
//! # The propagation rule
//!
//! "If either, but not both, operand of an operation is a NAN, and it is a
//! non-signaling NAN, then that NAN is returned as the result. If both
//! operands are non-signaling NANs, then the destination operand
//! non-signaling NAN is returned" (§4.5.4.1). That is [`Propagate::FirstNan`]
//! with the *destination* passed first, which is why every dyadic call below
//! puts the floating-point register before the source.
//!
//! # What is not modelled
//!
//! - **The coprocessor interface on the bus.** A real 68881 is a device in
//!   CPU space and the main processor talks to it by reading and writing
//!   coprocessor interface registers (M68881UM §7). `MemAttrs` carries no
//!   function code, so nothing in this framework can answer a CPU-space
//!   cycle; the instructions are executed directly instead. What that costs
//!   is the *bus trace* of a floating-point instruction — the operand
//!   transfers a logic analyser would see — and the protocol violation and
//!   "not ready" responses, which no program should depend on. Everything
//!   architectural survives: the registers, the exception vectors, the stack
//!   frames and `FSAVE`'s state frames.
//! - **Concurrency.** A 68881 runs beside the main processor and a 68882 runs
//!   two instructions at once; here an instruction completes before the next
//!   one starts, which is what a program that waits for its result sees
//!   anyway. The `FPIAR` still records the instruction's address, because a
//!   trap handler reads it.
//! - **Execution time.** Charged as the main processor's, not the
//!   coprocessor's; `timing.rs` says so and the conformance ledger carries
//!   it.

use crate::float::x87::{self, F80};
use crate::float::{
    Env, Flags, InputSubnormal, IntOverflow, MinMax, Nan, Propagate, Round, Spec, Tininess,
};

use super::isa::fp::{Fmt, FpOp};

/// Which coprocessor is attached, if any.
///
/// The two parts have the same programming model (M68881UM §2); the MC68882
/// differs in speed, in the size of its `FSAVE` frames, and in running two
/// instructions at once. A machine says which it has because the frame sizes
/// are visible to an operating system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Coprocessor {
    /// None: every F-line word is the line-F exception.
    #[default]
    None,
    /// An MC68881.
    M68881,
    /// An MC68882.
    M68882,
}

impl Coprocessor {
    /// Every value the `fpu` property accepts, in order.
    pub const ALL: [Coprocessor; 3] = [Coprocessor::None, Coprocessor::M68881, Coprocessor::M68882];

    /// The name the property spells it with.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Coprocessor::None => "none",
            Coprocessor::M68881 => "68881",
            Coprocessor::M68882 => "68882",
        }
    }

    /// The coprocessor a property value names.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Coprocessor> {
        Coprocessor::ALL.into_iter().find(|c| c.name() == name)
    }

    /// Whether one is attached at all.
    #[inline]
    #[must_use]
    pub const fn present(self) -> bool {
        !matches!(self, Coprocessor::None)
    }

    /// How many bytes an `FSAVE` idle frame occupies, the format long word
    /// included: 28 on a 68881 and 60 on a 68882 (M68881UM §4, *FSAVE*).
    #[must_use]
    pub const fn idle_frame(self) -> u32 {
        match self {
            Coprocessor::M68882 => 60,
            _ => 28,
        }
    }
}

impl core::fmt::Display for Coprocessor {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name())
    }
}

/// `FPCR` and `FPSR` bit positions (M68881UM Figures 2-2 to 2-7).
pub(in super::super) mod bits {
    /// Branch or set on unordered — `FPCR` enable and `FPSR` exception, bit
    /// 15 of each byte pair.
    pub(in super::super) const BSUN: u16 = 0x8000;
    /// A signalling NaN was an operand.
    pub(in super::super) const SNAN: u16 = 0x4000;
    /// The operation has no mathematical interpretation for its operands.
    pub(in super::super) const OPERR: u16 = 0x2000;
    /// Overflow.
    pub(in super::super) const OVFL: u16 = 0x1000;
    /// Underflow.
    pub(in super::super) const UNFL: u16 = 0x0800;
    /// Divide by zero.
    pub(in super::super) const DZ: u16 = 0x0400;
    /// An inexact operation.
    pub(in super::super) const INEX2: u16 = 0x0200;
    /// An inexact packed-decimal input.
    pub(in super::super) const INEX1: u16 = 0x0100;

    /// The accrued byte's invalid-operation bit.
    pub(in super::super) const AEXC_IOP: u8 = 0x80;
    /// Accrued overflow.
    pub(in super::super) const AEXC_OVFL: u8 = 0x40;
    /// Accrued underflow.
    pub(in super::super) const AEXC_UNFL: u8 = 0x20;
    /// Accrued divide by zero.
    pub(in super::super) const AEXC_DZ: u8 = 0x10;
    /// Accrued inexact.
    pub(in super::super) const AEXC_INEX: u8 = 0x08;

    /// `FPSR` bit 27: the result is negative.
    pub(in super::super) const CC_N: u32 = 0x0800_0000;
    /// Bit 26: the result is a zero.
    pub(in super::super) const CC_Z: u32 = 0x0400_0000;
    /// Bit 25: the result is an infinity.
    pub(in super::super) const CC_I: u32 = 0x0200_0000;
    /// Bit 24: the result is a NaN, or the comparison was unordered.
    pub(in super::super) const CC_NAN: u32 = 0x0100_0000;

    /// Every `FPCR` bit with storage: the enable byte and `PREC`/`RND`. Bits
    /// 31–16 "are always read as zero and are ignored during write
    /// operations", and so are bits 3–0 of the mode byte (§2.2).
    pub(in super::super) const FPCR_IMPLEMENTED: u32 = 0x0000_fff0;
    /// Every `FPSR` bit with storage: the condition codes, the quotient byte,
    /// the exception byte and the five accrued bits.
    pub(in super::super) const FPSR_IMPLEMENTED: u32 = 0x0fff_fff8;
}

/// The eight vectors the coprocessor's exceptions take, highest priority
/// first — "the bits of the ENABLE byte are organized in decreasing priority,
/// left to right, i.e. BSUN is the highest priority, and INEX1 is the lowest"
/// (M68881UM §2.2.1), and the vector numbers are MC68030UM Table 8-1.
const EXCEPTION_VECTORS: [(u16, u8); 8] = [
    (bits::BSUN, 48),
    (bits::SNAN, 54),
    (bits::OPERR, 52),
    (bits::OVFL, 53),
    (bits::UNFL, 51),
    (bits::DZ, 50),
    (bits::INEX2, 49),
    (bits::INEX1, 49),
];

/// The coprocessor's programmer-visible state.
///
/// `Copy` for the same reason [`super::exec::State`] is: the register file
/// travels by value through `regs()` and the snapshot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Fpu {
    /// `FP0`–`FP7`, always in the extended format.
    pub fp: [F80; 8],
    /// The control register: an enable byte and a mode byte.
    pub fpcr: u32,
    /// The status register: condition codes, a quotient byte, an exception
    /// byte and an accrued byte.
    pub fpsr: u32,
    /// The address of the instruction a trap handler should look at.
    pub fpiar: u32,
    /// Whether the unit is in the state a reset or a null `FRESTORE` leaves,
    /// which is what decides whether `FSAVE` writes a null frame
    /// (M68881UM §4, *FSAVE*).
    pub null: bool,
}

/// The NaN a reset leaves in every data register.
///
/// "A reset function or a restore operation of the null state sets FP0-FP7 to
/// positive non-signaling not-a-numbers" (M68881UM §2.1), and the NaN the
/// unit makes has an all-ones mantissa (§3.2.5).
pub(super) const CREATED_NAN: F80 = F80::new(0x7fff, u64::MAX);

impl Default for Fpu {
    fn default() -> Fpu {
        Fpu::RESET
    }
}

impl Fpu {
    /// The state a reset or a null `FRESTORE` leaves.
    pub(super) const RESET: Fpu = Fpu {
        fp: [CREATED_NAN; 8],
        fpcr: 0,
        fpsr: 0,
        fpiar: 0,
        null: true,
    };

    /// The rounding direction `FPCR`'s **RND** field names (§2.2.2).
    #[must_use]
    pub(super) const fn round(&self) -> Round {
        match (self.fpcr >> 4) & 3 {
            0 => Round::TiesEven,
            1 => Round::TowardZero,
            2 => Round::TowardNegative,
            _ => Round::TowardPositive,
        }
    }

    /// The format parameters `FPCR`'s **PREC** field names.
    ///
    /// Unlike x87's precision control this shortens the exponent range as
    /// well, so a single-precision result overflows above `2^128` even though
    /// it is stored in the extended format (§2.2.2). `11` is "undefined,
    /// reserved" and is read as extended, which is what a cleared register
    /// would have given anyway.
    #[must_use]
    pub(super) const fn spec(&self) -> Spec {
        match (self.fpcr >> 6) & 3 {
            1 => Spec::interchange(24, 127),
            2 => Spec::interchange(53, 1023),
            _ => F80::SPEC,
        }
    }

    /// The arithmetic environment: the 68881's rules plus the current
    /// rounding direction.
    #[must_use]
    pub(super) const fn env(&self) -> Env {
        Env {
            round: self.round(),
            ..M68881
        }
    }

    /// Clear the exception byte, as the start of "most operations" does
    /// (§2.3.3).
    pub(super) const fn clear_exceptions(&mut self) {
        self.fpsr &= !0x0000_ff00;
    }

    /// Set exception bits.
    pub(super) const fn raise(&mut self, exc: u16) {
        self.fpsr |= (exc as u32) & 0x0000_ff00;
    }

    /// Fold the exception byte into the accrued byte, as the end of every
    /// operation but `FMOVEM` and `FMOVE` to a control register does.
    ///
    /// The five equations are §2.3.4's, written out:
    /// `IOP = BSUN | SNAN | OPERR`, `OVFL = OVFL`,
    /// `UNFL = UNFL & INEX2`, `DZ = DZ`, `INEX = INEX1 | INEX2 | OVFL`.
    pub(super) const fn accrue(&mut self) {
        let exc = (self.fpsr & 0x0000_ff00) as u16;
        let mut acc = 0u8;
        if exc & (bits::BSUN | bits::SNAN | bits::OPERR) != 0 {
            acc |= bits::AEXC_IOP;
        }
        if exc & bits::OVFL != 0 {
            acc |= bits::AEXC_OVFL;
        }
        if exc & bits::UNFL != 0 && exc & bits::INEX2 != 0 {
            acc |= bits::AEXC_UNFL;
        }
        if exc & bits::DZ != 0 {
            acc |= bits::AEXC_DZ;
        }
        if exc & (bits::INEX1 | bits::INEX2 | bits::OVFL) != 0 {
            acc |= bits::AEXC_INEX;
        }
        self.fpsr |= acc as u32;
    }

    /// The vector of the highest-priority enabled exception now pending, if
    /// any.
    ///
    /// "When multiple exceptions occur with traps enabled for more than one
    /// exception class, the highest priority exception is reported; the lower
    /// priority exceptions are never reported or taken" (§2.2.1).
    #[must_use]
    pub(super) fn pending_trap(&self) -> Option<u8> {
        let live = ((self.fpsr & 0x0000_ff00) & (self.fpcr & 0x0000_ff00)) as u16;
        EXCEPTION_VECTORS
            .into_iter()
            .find(|(bit, _)| live & bit != 0)
            .map(|(_, vector)| vector)
    }

    /// Set the four condition code bits from a result (§2.3.1, Table 2-1).
    ///
    /// "The setting of the FPCP condition codes is independent of the
    /// operation executed; the condition codes only indicate the data type of
    /// the result."
    pub(super) fn set_condition(&mut self, value: F80) {
        let value = canonical(value);
        let mut cc = 0u32;
        if value.sign() {
            cc |= bits::CC_N;
        }
        if value.exp_field() == 0x7fff {
            if value.sig & !(1u64 << 63) == 0 {
                cc |= bits::CC_I;
            } else {
                cc |= bits::CC_NAN;
                // "The operation result data type determines how the four
                // condition code bits are set", and a NaN's sign is one of
                // the eight combinations Table 2-1 lists.
            }
        } else if value.sig == 0 {
            cc |= bits::CC_Z;
        }
        self.fpsr = (self.fpsr & !0x0f00_0000) | cc;
    }

    /// The four condition code bits, as `(N, Z, I, NAN)`.
    #[must_use]
    pub(super) const fn condition(&self) -> (bool, bool, bool, bool) {
        (
            self.fpsr & bits::CC_N != 0,
            self.fpsr & bits::CC_Z != 0,
            self.fpsr & bits::CC_I != 0,
            self.fpsr & bits::CC_NAN != 0,
        )
    }

    /// Record the sign and the low seven bits of a quotient, as `FMOD` and
    /// `FREM` do (§2.3.2).
    pub(super) const fn set_quotient(&mut self, negative: bool, magnitude: u8) {
        let byte = ((negative as u32) << 7) | ((magnitude & 0x7f) as u32);
        self.fpsr = (self.fpsr & !0x00ff_0000) | (byte << 16);
    }
}

/// The 68881's floating-point personality.
///
/// Every field is a statement from the manual:
///
/// - **NaN propagation** is §4.5.4.1's destination-wins rule, which is
///   `FirstNan` with the destination passed first.
/// - **The default NaN's sign** never reaches an output, because
///   [`generated_nan`] replaces every NaN the unit creates with §3.2.5's
///   all-ones one; it is set positive to match.
/// - **Tininess** is detected on "the intermediate result exponent", before
///   rounding (§6.1.5).
/// - **Subnormal operands** are used exactly and are not reported: the 68881
///   has no denormal-operand exception class at all (§2.3.3's eight are the
///   whole list).
/// - **No flush to zero**: §6.1.5's trap-disabled result "is either a
///   denormalized number or zero".
/// - **`min`/`max`** never arise; the 68881 has no such instruction.
/// - **Integer conversion out of range** gives "the largest positive or
///   negative integer that can fit in the specified destination format size"
///   (§6.1.3) — saturation. A NaN is handled before the conversion, because
///   the 68881's answer there is its significand's top bits rather than any
///   of the three conventions `src/float` knows.
const M68881: Env = Env {
    round: Round::TiesEven,
    nan: Nan {
        propagate: Propagate::FirstNan,
        default_sign: false,
    },
    tininess: Tininess::BeforeRounding,
    subnormal_inputs: InputSubnormal::Exact,
    flush_outputs: false,
    min_max: MinMax::PropagateNan,
    int_overflow: IntOverflow::SaturateNanMax,
};

/// The encoding the 68881 reads out of a ninety-six-bit extended operand, or
/// out of a data register.
///
/// Three of this module's four format differences from x87 are here:
///
/// - an **unnormalized** number — a non-zero exponent with the integer bit
///   clear — is normalized, or turned into a signed zero when its mantissa is
///   all zeros (§3.5.1);
/// - a **pseudo-infinity** and a **pseudo-NaN** — the maximum exponent with
///   the integer bit clear — are an ordinary infinity and an ordinary NaN
///   (Figure 3-6, §3.2.5);
/// - everything else is already what `src/float`'s `x87` module reads, a
///   pseudo-denormal included.
///
/// Doing it here rather than in `src/float` is deliberate: x87's reading of
/// those encodings is *correct for x87*, and a shared module must not be bent
/// to one guest's rules.
#[must_use]
pub(super) const fn canonical(v: F80) -> F80 {
    const INTEGER: u64 = 1 << 63;
    let field = v.exp_field();
    if field == 0x7fff {
        // An infinity or a NaN whichever way the integer bit is set.
        return F80::new(v.sign_exp, v.sig | INTEGER);
    }
    if field == 0 || v.sig & INTEGER != 0 {
        // Already a value x87 reads the same way: a zero, a subnormal, a
        // pseudo-denormal or a normal.
        return v;
    }
    if v.sig == 0 {
        // "If the external operand is an extended precision unnormalized zero
        // (i.e., with a mantissa of all zeros), the number is converted to an
        // extended precision normalized zero."
        return F80::new(v.sign_exp & 0x8000, 0);
    }
    // Shift the significand up until the integer bit is set, taking the
    // exponent down with it. If the exponent reaches zero first the value is
    // subnormal, and the encoding for that is exponent field zero with the
    // significand shifted by however much was left.
    let shift = v.sig.leading_zeros();
    if (shift as u16) < field {
        F80::new(v.sign_exp - shift as u16, v.sig << shift)
    } else {
        // `field - 1` shifts reach exponent field 1, and one more reaches the
        // subnormal encoding, which shares that exponent.
        F80::new(v.sign_exp & 0x8000, v.sig << (field - 1))
    }
}

/// The NaN the unit delivers when it makes one, rather than propagating one.
///
/// `src/float` hands back x87's "floating-point indefinite" — a negative quiet
/// NaN with an all-zero payload — for an invalid operation with no NaN input.
/// The 68881's is all ones (§3.2.5), so a result that is a NaN when no
/// operand was one is replaced.
#[must_use]
pub(super) fn generated_nan(result: F80, operands: &[F80]) -> F80 {
    if result.exp_field() != 0x7fff || result.sig & !(1u64 << 63) == 0 {
        return result;
    }
    if operands.iter().any(|v| is_nan(*v)) {
        return result;
    }
    CREATED_NAN
}

/// Whether an encoding is a NaN under the 68881's reading.
#[must_use]
pub(super) const fn is_nan(v: F80) -> bool {
    v.exp_field() == 0x7fff && v.sig & !(1u64 << 63) != 0
}

/// Whether an encoding is a signalling NaN: a NaN whose leading fraction bit
/// — "the MSB of the mantissa minus one for extended precision" — is clear
/// (§3.2.5).
#[must_use]
pub(super) const fn is_snan(v: F80) -> bool {
    is_nan(v) && v.sig & (1 << 62) == 0
}

/// Whether an encoding is an infinity.
#[must_use]
pub(super) const fn is_infinity(v: F80) -> bool {
    v.exp_field() == 0x7fff && v.sig & !(1u64 << 63) == 0
}

/// Whether an encoding is a zero.
#[must_use]
pub(super) const fn is_zero(v: F80) -> bool {
    v.exp_field() == 0 && v.sig == 0
}

/// Whether a finite value is below the smallest normal of `spec` — the
/// 68881's tininess, which it reports whether or not the result is also
/// inexact.
///
/// `src/float` reports underflow only for a result that is both tiny *and*
/// inexact, which is IEEE 754-2019 §7.5's rule for the *flag*. The 68881 sets
/// `FPSR`'s **UNFL** on tininess alone and reaches the same place by ANDing
/// it with **INEX2** on the way into the accrued byte (§2.3.4), so this is
/// what the exception byte needs.
#[must_use]
pub(super) fn is_tiny(v: F80, spec: Spec) -> bool {
    if is_nan(v) || is_infinity(v) || is_zero(v) {
        return false;
    }
    let field = v.exp_field();
    if field == 0 {
        return true;
    }
    i32::from(field) - 16383 < spec.emin()
}

/// Turn `src/float`'s flags into `FPSR` exception bits.
///
/// The mapping is one to one except for the two the 68881 splits differently:
/// its **OPERR** is the invalid operation that is *not* a signalling NaN, and
/// its **INEX1** is reserved for a packed-decimal input, so a `Flags::INEXACT`
/// is always **INEX2**.
#[must_use]
pub(super) fn exceptions_from(flags: Flags, snan: bool) -> u16 {
    let mut out = 0;
    if flags.contains(Flags::INVALID) {
        out |= if snan { bits::SNAN } else { bits::OPERR };
    }
    if snan {
        out |= bits::SNAN;
    }
    if flags.contains(Flags::DIV_BY_ZERO) {
        out |= bits::DZ;
    }
    if flags.contains(Flags::OVERFLOW) {
        out |= bits::OVFL;
    }
    if flags.contains(Flags::UNDERFLOW) {
        out |= bits::UNFL;
    }
    if flags.contains(Flags::INEXACT) {
        out |= bits::INEX2;
    }
    out
}

// ---------------------------------------------------------------------------
// The seven data formats
// ---------------------------------------------------------------------------

/// Read the ninety-six bits of an extended-precision operand.
///
/// "The most-significant byte is located at the lowest address": a word of
/// sign and exponent, a word the manual marks zero and the processor ignores,
/// then the eight significand bytes (M68881UM Table 3-3).
#[must_use]
pub(super) const fn read_extended(hi: u32, mid: u32, lo: u32) -> F80 {
    F80::new((hi >> 16) as u16, ((mid as u64) << 32) | lo as u64)
}

/// The three long words an extended-precision store writes.
#[must_use]
pub(super) const fn write_extended(v: F80) -> [u32; 3] {
    [
        (v.sign_exp as u32) << 16,
        (v.sig >> 32) as u32,
        v.sig as u32,
    ]
}

/// Widen an external operand of `fmt` into the extended format.
///
/// `raw` holds the operand right-justified: one, two or four bytes for the
/// integers, four for single, the two halves of a double, or the ninety-six
/// bits of an extended one already assembled by [`read_extended`].
#[must_use]
pub(super) fn widen(fmt: Fmt, raw: u64, extended: F80, env: Env) -> (F80, Flags) {
    use crate::float::{B32, B64};
    match fmt {
        Fmt::Byte => x87::from_signed(raw as i8 as i64, 8, env),
        Fmt::Word => x87::from_signed(raw as i16 as i64, 16, env),
        Fmt::Long => x87::from_signed(raw as i32 as i64, 32, env),
        Fmt::Single => x87::from_binary::<B32>(raw & 0xffff_ffff, env),
        Fmt::Double => x87::from_binary::<B64>(raw, env),
        // Already ninety-six bits of the format the registers hold; the only
        // work is the 68881's own reading of the encoding.
        Fmt::Extended | Fmt::Packed => (canonical(extended), Flags::NONE),
    }
}

/// Narrow a register to an external operand of `fmt`.
///
/// Returns the operand right-justified, the three long words an extended
/// store needs, and the flags. A NaN bound for an integer destination is the
/// 68881's own rule and not any of the conventions `src/float` offers: "if
/// the destination is B, W, or L and the floating-point number to be stored
/// is a NAN, then the 8, 16, or 32 most significant bits of the NAN
/// significand are stored as the result" (§6.1.3).
#[must_use]
pub(super) fn narrow(fmt: Fmt, v: F80, env: Env) -> (u64, [u32; 3], Flags) {
    use crate::float::{B32, B64};
    let v = canonical(v);
    let integer = |bits: u32| -> (u64, Flags) {
        if is_nan(v) {
            let top = v.sig >> (64 - bits);
            return (top, Flags::INVALID);
        }
        let (value, flags) = x87::to_signed(v, bits, env);
        (value as u64, flags)
    };
    match fmt {
        Fmt::Byte => {
            let (raw, f) = integer(8);
            (raw & 0xff, [0; 3], f)
        }
        Fmt::Word => {
            let (raw, f) = integer(16);
            (raw & 0xffff, [0; 3], f)
        }
        Fmt::Long => {
            let (raw, f) = integer(32);
            (raw & 0xffff_ffff, [0; 3], f)
        }
        Fmt::Single => {
            let (bits, f) = x87::to_binary::<B32>(v, env);
            (bits & 0xffff_ffff, [0; 3], f)
        }
        Fmt::Double => {
            let (bits, f) = x87::to_binary::<B64>(v, env);
            (bits, [0; 3], f)
        }
        // An extended store is the register's own bits: "no operation
        // performed by the FPCP can create an unnormalized result, [so] the
        // result of moving a floating-point data register to an extended
        // precision external destination can never be an unnormalized
        // number" (§3.5.2).
        Fmt::Extended | Fmt::Packed => (0, write_extended(v), Flags::NONE),
    }
}

/// The value whose data type gives `FCMP`'s condition codes.
///
/// M68881UM §4's *FCMP* operation table is an ordered three-way test rather
/// than the subtraction the description names, and three of its entries say
/// so: the **I** bit is "always cleared by the FCMP instruction, since it is
/// not used by any of the conditional predicate equations"; two equal
/// infinities compare **equal** where a subtraction would give a NaN; and an
/// equal pair sets **N** from the *destination's* sign, which is why
/// `FCMP -0,-0` reports negative and zero.
///
/// Returning a stand-in value rather than the codes themselves keeps one
/// path through [`Fpu::set_condition`], and every stand-in is a data type
/// Table 2-1 already covers.
#[must_use]
pub(super) fn compare_result(dest: F80, src: F80) -> F80 {
    /// 1.0 — a positive normalized number, which sets nothing.
    const POSITIVE: F80 = F80::new(0x3fff, 1 << 63);
    /// -1.0 — a negative normalized number, which sets **N**.
    const NEGATIVE: F80 = F80::new(0xbfff, 1 << 63);
    const NEGATIVE_ZERO: F80 = F80::new(0x8000, 0);

    let (dest, src) = (canonical(dest), canonical(src));
    // "If either operand is a NAN, refer to 4.5.4 NANs": the destination's
    // wins when both are, and the condition codes come from the NaN itself.
    if is_nan(dest) && is_nan(src) {
        return dest;
    }
    if is_nan(dest) {
        return dest;
    }
    if is_nan(src) {
        return src;
    }
    match x87::compare(dest, src) {
        Some(core::cmp::Ordering::Less) => NEGATIVE,
        Some(core::cmp::Ordering::Greater) => POSITIVE,
        // Equal, so the result is a zero — and its sign is the
        // destination's, which is the table's `NZ` for every negative
        // destination against an equal source.
        _ => {
            if dest.sign() {
                NEGATIVE_ZERO
            } else {
                F80::ZERO
            }
        }
    }
}

/// The on-chip constant ROM `FMOVECR` reads (M68881UM §4, *FMOVECR*).
///
/// Each value is the **correctly rounded** 64-bit significand of the real
/// number the manual names, computed to two hundred decimal digits and
/// rounded once, ties to even. They are not transcribed from a particular
/// part: the manual says outright that "the values contained at offsets other
/// than those defined above are reserved for the use of Motorola, and may be
/// different on various mask sets of the FPCP", so an offset the table does
/// not define reads as zero here and the defined ones are the mathematical
/// values rather than one mask set's.
#[must_use]
pub(super) const fn constant(offset: u8) -> F80 {
    match offset & 0x7f {
        0x00 => F80::new(0x4000, 0xc90f_daa2_2168_c235), // pi
        0x0b => F80::new(0x3ffd, 0x9a20_9a84_fbcf_f799), // log10(2)
        0x0c => F80::new(0x4000, 0xadf8_5458_a2bb_4a9b), // e
        0x0d => F80::new(0x3fff, 0xb8aa_3b29_5c17_f0bc), // log2(e)
        0x0e => F80::new(0x3ffd, 0xde5b_d8a9_3728_7195), // log10(e)
        0x0f => F80::ZERO,
        0x30 => F80::new(0x3ffe, 0xb172_17f7_d1cf_79ac), // ln(2)
        0x31 => F80::new(0x4000, 0x935d_8ddd_aaa8_ac17), // ln(10)
        0x32 => F80::new(0x3fff, 0x8000_0000_0000_0000), // 10^0
        0x33 => F80::new(0x4002, 0xa000_0000_0000_0000), // 10^1
        0x34 => F80::new(0x4005, 0xc800_0000_0000_0000), // 10^2
        0x35 => F80::new(0x400c, 0x9c40_0000_0000_0000), // 10^4
        0x36 => F80::new(0x4019, 0xbebc_2000_0000_0000), // 10^8
        0x37 => F80::new(0x4034, 0x8e1b_c9bf_0400_0000), // 10^16
        0x38 => F80::new(0x4069, 0x9dc5_ada8_2b70_b59e), // 10^32
        0x39 => F80::new(0x40d3, 0xc278_1f49_ffcf_a6d5), // 10^64
        0x3a => F80::new(0x41a8, 0x93ba_47c9_80e9_8ce0), // 10^128
        0x3b => F80::new(0x4351, 0xaa7e_ebfb_9df9_de8e), // 10^256
        0x3c => F80::new(0x46a3, 0xe319_a0ae_a60e_91c7), // 10^512
        0x3d => F80::new(0x4d48, 0xc976_7586_8175_0c17), // 10^1024
        0x3e => F80::new(0x5a92, 0x9e8b_3b5d_c53d_5de5), // 10^2048
        0x3f => F80::new(0x7525, 0xc460_5202_8a20_979b), // 10^4096
        _ => F80::ZERO,
    }
}

// ---------------------------------------------------------------------------
// The arithmetic
// ---------------------------------------------------------------------------

/// What one operation produced.
#[derive(Debug, Clone, Copy)]
pub(super) struct Computed {
    /// The result, which is also what sets the condition codes.
    pub value: F80,
    /// Whether it is written to the destination register.
    pub store: bool,
    /// The exception bits, already corrected for the operation's own Status
    /// Register section.
    pub exc: u16,
    /// The sign and magnitude `FMOD` and `FREM` put in the quotient byte.
    pub quotient: Option<(bool, u8)>,
    /// Which format's exponent range decides tininess: the rounding
    /// precision for most operations, the extended range for the two single
    /// ones (M68881UM §6.1.5's note).
    pub tininess: Spec,
}

impl Computed {
    fn new(value: F80, flags: Flags, spec: Spec) -> Computed {
        Computed {
            value,
            store: true,
            exc: exceptions_from(flags, false),
            quotient: None,
            tininess: spec,
        }
    }

    /// The same, with bits the instruction's Status Register section says are
    /// cleared taken back out.
    fn clearing(mut self, bits: u16) -> Computed {
        self.exc &= !bits;
        self
    }
}

/// The result of an operand error: "an extended precision non-signaling NAN
/// (with all ones mantissa) is stored in the destination floating-point data
/// register" (M68881UM §6.1.3).
fn operand_error(spec: Spec) -> Computed {
    Computed {
        value: CREATED_NAN,
        store: true,
        exc: bits::OPERR,
        quotient: None,
        tininess: spec,
    }
}

/// Compute one arithmetic operation.
///
/// `dest` is the destination register's value and `src` the source operand,
/// both already put through [`canonical`]. The exception bits that come back
/// are `src/float`'s flags mapped through [`exceptions_from`], **minus** the
/// bits the instruction's own Status Register section in M68881UM §4 marks
/// *Cleared* — which is the whole of the per-instruction exception rules that
/// are not simply the arithmetic's. `SNAN` is the caller's, because only it
/// knows whether an operand was signalling, and `UNFL` for tininess is the
/// caller's too, because only it knows the rounding precision.
#[must_use]
pub(super) fn operate(op: FpOp, dest: F80, src: F80, spec: Spec, env: Env) -> Computed {
    // A NaN operand short-circuits every operation: "All operations involving
    // a NAN operand as an input return a NAN result" (§3.2.5), and which one
    // is §4.5.4.1's destination-wins rule.
    if is_nan(src) || (op.is_dyadic() && is_nan(dest)) {
        let chosen = if op.is_dyadic() && is_nan(dest) {
            dest
        } else if is_nan(src) {
            src
        } else {
            dest
        };
        // Quieting a signalling NaN is "setting the SNAN bit in the operand
        // to a one" (§4.5.4.2).
        let quiet = F80::new(chosen.sign_exp, chosen.sig | (1 << 62));
        return Computed {
            value: quiet,
            store: op.writes_destination(),
            exc: 0,
            quotient: None,
            tininess: spec,
        };
    }

    match op {
        // "Stores the absolute value of that number in the destination" and
        // "FABS will round the result to the precision selected in the
        // floating-point control register" (M68000PRM §5, *FABS*). Its
        // Status Register section clears every bit but `SNAN` and the `UNFL`
        // a denormalized source raises.
        FpOp::Abs | FpOp::Neg => {
            let signed = match op {
                FpOp::Abs => F80::new(src.sign_exp & 0x7fff, src.sig),
                _ => F80::new(src.sign_exp ^ 0x8000, src.sig),
            };
            let (value, flags) = x87::round_to(signed, spec, env);
            Computed::new(value, flags, spec)
                .clearing(bits::OPERR | bits::OVFL | bits::DZ | bits::INEX2 | bits::UNFL)
        }
        FpOp::Move => {
            let (value, flags) = x87::round_to(src, spec, env);
            Computed::new(value, flags, spec)
        }
        // "Sets the condition code bits according to the data type of the
        // result"; nothing else happens.
        FpOp::Tst => Computed {
            value: src,
            store: false,
            exc: 0,
            quotient: None,
            tininess: spec,
        },
        FpOp::Cmp => Computed {
            value: compare_result(dest, src),
            store: false,
            exc: 0,
            quotient: None,
            tininess: spec,
        },
        FpOp::Sqrt => {
            let (value, flags) = x87::sqrt_to(src, spec, env);
            Computed::new(generated_nan(value, &[src]), flags, spec)
        }
        // "Rounds the source operand to an integer value, using the rounding
        // mode specified by the FPCR" — and `FINTRZ` "always uses the
        // round-to-zero mode, regardless of the current rounding mode".
        FpOp::Int | FpOp::IntRz => {
            let mode = if op == FpOp::Int {
                env
            } else {
                env.round(Round::TowardZero)
            };
            let (value, flags) = x87::round_to_integral(src, mode);
            Computed::new(value, flags, spec).clearing(bits::OVFL | bits::UNFL | bits::DZ)
        }
        // "Extracts the binary exponent, removes the exponent bias, converts
        // the exponent to an extended precision floating-point number";
        // `±0` gives `±0.0` and an infinity is an operand error (§4,
        // *FGETEXP*'s operation table).
        FpOp::GetExp => {
            if is_infinity(src) {
                return operand_error(spec);
            }
            if is_zero(src) {
                return Computed::new(src, Flags::NONE, spec);
            }
            let (exponent, _, flags) = x87::extract(src, env);
            Computed::new(exponent, flags, spec)
                .clearing(bits::OVFL | bits::UNFL | bits::DZ | bits::INEX2 | bits::OPERR)
        }
        // The same split for the mantissa, which comes back in `[1, 2)` with
        // the source's sign.
        FpOp::GetMan => {
            if is_infinity(src) {
                return operand_error(spec);
            }
            if is_zero(src) {
                return Computed::new(src, Flags::NONE, spec);
            }
            let (_, mantissa, flags) = x87::extract(src, env);
            Computed::new(mantissa, flags, spec)
                .clearing(bits::OVFL | bits::UNFL | bits::DZ | bits::INEX2 | bits::OPERR)
        }
        // The four that are simply the arithmetic. The destination comes
        // first in every one, both because that is the operation — `FSUB` is
        // "FPn - Source" and `FDIV` is "FPn / Source" — and because the NaN
        // rule needs it there.
        FpOp::Add => {
            let (value, flags) = x87::add_to(dest, src, spec, env);
            Computed::new(generated_nan(value, &[dest, src]), flags, spec)
        }
        FpOp::Sub => {
            let (value, flags) = x87::sub_to(dest, src, spec, env);
            Computed::new(generated_nan(value, &[dest, src]), flags, spec)
        }
        FpOp::Mul => {
            let (value, flags) = x87::mul_to(dest, src, spec, env);
            Computed::new(generated_nan(value, &[dest, src]), flags, spec)
        }
        FpOp::Div => {
            let (value, flags) = x87::div_to(dest, src, spec, env);
            Computed::new(generated_nan(value, &[dest, src]), flags, spec)
        }
        // "Regardless of the precision specified by the PREC bits, these
        // instructions round the result mantissa to single precision and
        // generate an extended precision exponent which may be out of range
        // for a single precision number" (§2.2.2) — so the significand is
        // shortened and the exponent range is the 80-bit one, which also
        // means "these instructions can never report an underflow as long as
        // the intermediate result is large enough to be represented in the
        // extended precision format" (§6.1.5).
        FpOp::SglMul | FpOp::SglDiv => {
            let single = F80::SPEC.with_precision(24);
            let (value, flags) = if op == FpOp::SglMul {
                x87::mul_to(dest, src, single, env)
            } else {
                x87::div_to(dest, src, single, env)
            };
            let mut out = Computed::new(generated_nan(value, &[dest, src]), flags, single);
            out.tininess = F80::SPEC;
            out
        }
        // "Multiplies the destination by 2 raised to the power of the source,
        // where the source is first converted to an integer": an infinite
        // source is an operand error (Table 6-2).
        FpOp::Scale => {
            if is_infinity(src) {
                return operand_error(spec);
            }
            // The source is truncated toward zero, and a magnitude no
            // exponent can reach saturates rather than wrapping — `scale`
            // clamps far outside the range so a huge shift still overflows
            // or underflows in the right direction.
            let (by, _) = x87::to_signed(src, 64, env.round(Round::TowardZero));
            let (value, flags) = x87::scale(dest, by, env);
            let (value, rounding) = x87::round_to(value, spec, env);
            let flags = flags | rounding;
            Computed::new(generated_nan(value, &[dest, src]), flags, spec)
        }
        // "FPn - (Source * N)", where N is the quotient truncated (`FMOD`) or
        // rounded to nearest even (`FREM`). Both are exact, so the only
        // exceptions are the operand errors of Table 6-2: an infinite
        // destination or a zero source.
        FpOp::Mod | FpOp::Rem => {
            if is_infinity(dest) || is_zero(src) {
                return operand_error(spec);
            }
            let (value, negative, magnitude) = remainder(dest, src, op == FpOp::Rem);
            let mut out = Computed::new(value, Flags::NONE, spec);
            out.quotient = Some((negative, magnitude));
            out
        }
        // Every transcendental, which this core does not compute yet; the
        // caller has already refused them through `implemented`.
        _ => operand_error(spec),
    }
}

/// `FMOD`'s and `FREM`'s remainder, and the quotient's sign and low seven
/// bits.
///
/// Returns the exact remainder — "the result of the subtraction is always
/// exact" for both, because it is a difference of two representable values
/// that is itself representable.
///
/// This is long division rather than [`x87::remainder`] for one reason: x87's
/// `FPREM` reduces *partially* when the operands are more than sixty-three
/// binades apart and leaves the guest to loop, and the quotient bits are
/// undefined when it does. The 68881's `FMOD` and `FREM` always complete and
/// always report the quotient's low seven bits (§2.3.2), so the whole
/// quotient has to be developed. The loop runs once per binade of separation,
/// which is at most the exponent range and only for operands that far apart.
fn remainder(dest: F80, src: F80, ieee: bool) -> (F80, bool, u8) {
    // A zero destination or an infinite source leaves the destination alone.
    if is_zero(dest) || is_infinity(src) {
        return (dest, false, 0);
    }
    let (sign_a, exp_a, sig_a) = significand(dest);
    let (sign_b, exp_b, sig_b) = significand(src);
    let d = exp_a - exp_b;
    if d < 0 {
        // |dest| < |src| / 2: the truncated quotient is zero, and the
        // nearest-even one is zero as well unless the halves are close
        // enough, which `d < -1` rules out.
        if d < -1 {
            return (dest, sign_a != sign_b, 0);
        }
    }
    // Long division of `sig_a * 2^d` by `sig_b`, one quotient bit per step.
    let mut rem = u128::from(sig_a);
    let mut quotient: u64;
    if d >= 0 {
        let first = u128::from(sig_b) <= rem;
        if first {
            rem -= u128::from(sig_b);
        }
        let mut running = u64::from(first);
        for _ in 0..d {
            rem <<= 1;
            let bit = u128::from(sig_b) <= rem;
            if bit {
                rem -= u128::from(sig_b);
            }
            running = (running << 1) | u64::from(bit);
        }
        quotient = running;
    } else {
        // `d == -1`: line the divisor up with the dividend instead, so the
        // single quotient bit is decided against twice the dividend.
        rem <<= 1;
        let bit = u128::from(sig_b) <= rem;
        if bit {
            rem -= u128::from(sig_b);
        }
        quotient = u64::from(bit);
    }
    let scale = if d >= 0 { exp_b } else { exp_a };
    let mut sign = sign_a;
    let mut value = rem;
    if ieee {
        // "Round the quotient to nearest, ties to even", which turns the
        // remainder negative whenever it was more than half the divisor.
        let twice = rem * 2;
        let den = u128::from(sig_b);
        if twice > den || (twice == den && quotient & 1 == 1) {
            quotient = quotient.wrapping_add(1);
            value = den - rem;
            sign = !sign;
        }
    }
    let negative = sign_a != sign_b;
    (
        from_significand(sign, scale, value),
        negative,
        (quotient & 0x7f) as u8,
    )
}

/// A finite non-zero value as `(sign, ulp exponent, significand)`, with the
/// significand normalized to bit 63.
fn significand(v: F80) -> (bool, i32, u64) {
    let field = v.exp_field();
    let (exp, sig) = if field == 0 {
        // A subnormal's last bit is worth `2^-16445`.
        (-16445i32, v.sig)
    } else {
        (i32::from(field) - 16383 - 63, v.sig)
    };
    let shift = sig.leading_zeros() as i32;
    (v.sign(), exp - shift, sig << shift)
}

/// Build a value from a sign, an exponent for the significand's last bit, and
/// a significand — exactly, which is all a remainder ever needs.
fn from_significand(sign: bool, exp: i32, sig: u128) -> F80 {
    if sig == 0 {
        return F80::new(if sign { 0x8000 } else { 0 }, 0);
    }
    let width = 128 - sig.leading_zeros() as i32;
    // Line the top bit up with bit 63.
    let shift = 64 - width;
    let (sig, exp) = if shift >= 0 {
        ((sig << shift) as u64, exp - shift)
    } else {
        ((sig >> -shift) as u64, exp - shift)
    };
    let unbiased = exp + 63;
    let field = unbiased + 16383;
    if field >= 1 {
        F80::new((field as u16) | if sign { 0x8000 } else { 0 }, sig)
    } else {
        // Subnormal: the exponent field is zero and the significand slides
        // down to the fixed grid.
        let down = 1 - field;
        let sig = if down >= 64 { 0 } else { sig >> down };
        F80::new(if sign { 0x8000 } else { 0 }, sig)
    }
}

/// Whether an opmode is one this core computes.
///
/// An operation that is not takes the line-F exception, which is what a main
/// processor does when no coprocessor answers (MC68020UM §7.5.2) and what a
/// 68040 does for the transcendentals it dropped — the pattern an F-line
/// handler that emulates them is written against. `docs/cpu/m68k.md` has the
/// ledger of what is here and what is not.
#[must_use]
pub(super) const fn implemented(op: FpOp) -> bool {
    matches!(
        op,
        FpOp::Move
            | FpOp::Tst
            | FpOp::Cmp
            | FpOp::Abs
            | FpOp::Neg
            | FpOp::Sqrt
            | FpOp::Int
            | FpOp::IntRz
            | FpOp::GetExp
            | FpOp::GetMan
            | FpOp::Add
            | FpOp::Sub
            | FpOp::Mul
            | FpOp::Div
            | FpOp::SglMul
            | FpOp::SglDiv
            | FpOp::Scale
            | FpOp::Mod
            | FpOp::Rem
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The value `sig × 2^(field - 16383 - 63)`.
    fn f(field: u16, sig: u64) -> F80 {
        F80::new(field, sig)
    }

    #[test]
    fn an_unnormalized_number_is_normalized_before_use() {
        // M68881UM §3.5.1: "If an external operand is an extended precision
        // unnormalized number, the number is normalized before it is used in
        // an arithmetic operation."
        // $4000 with the significand one place down is the same value as
        // $3fff with it in place — that is, 2.0 written twice.
        let two = f(0x4000, 0x8000_0000_0000_0000);
        let unnormal = f(0x4001, 0x4000_0000_0000_0000);
        assert_eq!(canonical(unnormal), two);
        assert_eq!(canonical(two), two, "a normal is left alone");
        // A negative one keeps its sign.
        let neg = f(0x8000 | 0x4001, 0x4000_0000_0000_0000);
        assert_eq!(canonical(neg), f(0x8000 | 0x4000, 0x8000_0000_0000_0000));
        // Eight places down.
        assert_eq!(canonical(f(0x4008, 0x0080_0000_0000_0000)), two);
    }

    #[test]
    fn an_unnormalized_zero_becomes_a_signed_zero() {
        // "If the external operand is an extended precision unnormalized zero
        // (i.e., with a mantissa of all zeros), the number is converted to an
        // extended precision normalized zero."
        assert_eq!(canonical(f(0x4000, 0)), F80::ZERO);
        assert_eq!(canonical(f(0x8000 | 0x4000, 0)), F80::new(0x8000, 0));
    }

    #[test]
    fn normalizing_below_the_exponent_range_gives_a_denormal() {
        // An unnormalized number whose significand cannot be shifted up far
        // enough lands on the subnormal grid, which is exponent field zero.
        let v = f(0x0003, 0x0000_0000_0000_0001);
        let out = canonical(v);
        assert_eq!(out.exp_field(), 0, "the subnormal encoding");
        assert_eq!(out.sig, 1 << 2, "shifted by the exponent it had to spare");
    }

    #[test]
    fn a_pseudo_infinity_and_a_pseudo_nan_are_ordinary_ones() {
        // Figure 3-6: for an infinity "the most significant bit of the
        // mantissa (the integer bit) is a don't care"; §3.2.5 says the same
        // for a NaN. x87 calls both unsupported.
        assert_eq!(canonical(f(0x7fff, 0)), F80::INFINITY);
        assert!(is_infinity(canonical(f(0x7fff, 0))));
        let pseudo_nan = f(0x7fff, 0x4000_0000_0000_0001);
        assert!(is_nan(canonical(pseudo_nan)));
        assert_eq!(canonical(pseudo_nan).sig, 0xc000_0000_0000_0001);
        // And x87 itself still rejects them, which is right for x87.
        assert_eq!(
            x87::classify(f(0x7fff, 0)),
            crate::float::x87::X87Class::Unsupported
        );
    }

    #[test]
    fn the_created_nan_is_the_manuals_and_not_x87s() {
        // §3.2.5: "for any precision, all bits of the mantissa are ones".
        assert_eq!(CREATED_NAN.sig, u64::MAX);
        assert_eq!(CREATED_NAN.exp_field(), 0x7fff);
        assert!(!CREATED_NAN.sign(), "positive, as this core reads it");
        assert!(is_nan(CREATED_NAN) && !is_snan(CREATED_NAN));
        // x87's indefinite is a different value, and it is what `src/float`
        // delivers, so generation is intercepted.
        assert_ne!(F80::INDEFINITE, CREATED_NAN);
        assert_eq!(generated_nan(F80::INDEFINITE, &[F80::ZERO]), CREATED_NAN);
        // A propagated NaN is kept: one of the operands was a NaN.
        let payload = f(0x7fff, 0xc000_0000_0000_1234);
        assert_eq!(generated_nan(payload, &[payload, F80::ZERO]), payload);
        // And a number is never touched.
        assert_eq!(generated_nan(F80::ZERO, &[]), F80::ZERO);
    }

    #[test]
    fn the_rounding_precision_shortens_the_exponent_range_too() {
        // §2.2.2: "if the single or double precision mode is selected, the
        // exponent value is in the correct range for the single or double
        // precision format" — which x87's PC does not do.
        let mut fpu = Fpu::RESET;
        assert_eq!(fpu.spec(), F80::SPEC);
        fpu.fpcr = 1 << 6;
        assert_eq!(fpu.spec(), Spec::interchange(24, 127));
        fpu.fpcr = 2 << 6;
        assert_eq!(fpu.spec(), Spec::interchange(53, 1023));
        // "11 (undefined, reserved)" reads as extended here.
        fpu.fpcr = 3 << 6;
        assert_eq!(fpu.spec(), F80::SPEC);
    }

    #[test]
    fn the_rounding_modes_are_the_manuals_order() {
        // Figure 2-3: 00 to nearest, 01 toward zero, 10 toward minus
        // infinity, 11 toward plus infinity.
        let with = |bits: u32| {
            Fpu {
                fpcr: bits << 4,
                ..Fpu::RESET
            }
            .round()
        };
        assert_eq!(with(0), Round::TiesEven);
        assert_eq!(with(1), Round::TowardZero);
        assert_eq!(with(2), Round::TowardNegative);
        assert_eq!(with(3), Round::TowardPositive);
    }

    #[test]
    fn the_condition_codes_are_table_2_1() {
        let mut fpu = Fpu::RESET;
        let cc = |fpu: &mut Fpu, v: F80| {
            fpu.set_condition(v);
            fpu.condition()
        };
        assert_eq!(cc(&mut fpu, F80::ZERO), (false, true, false, false));
        assert_eq!(
            cc(&mut fpu, F80::new(0x8000, 0)),
            (true, true, false, false)
        );
        assert_eq!(cc(&mut fpu, F80::INFINITY), (false, false, true, false));
        assert_eq!(
            cc(&mut fpu, F80::new(0xffff, 1 << 63)),
            (true, false, true, false)
        );
        assert_eq!(cc(&mut fpu, CREATED_NAN), (false, false, false, true));
        assert_eq!(
            cc(&mut fpu, F80::new(0xffff, u64::MAX)),
            (true, false, false, true)
        );
        // 1.0 and -1.0 are "normalized", which is all four bits clear.
        let one = F80::new(0x3fff, 1 << 63);
        assert_eq!(cc(&mut fpu, one), (false, false, false, false));
        assert_eq!(
            cc(&mut fpu, F80::new(0xbfff, 1 << 63)),
            (true, false, false, false)
        );
    }

    #[test]
    fn the_accrued_byte_is_the_five_equations() {
        // §2.3.4's equations, each tested on its own.
        let accrued = |exc: u16| {
            let mut fpu = Fpu::RESET;
            fpu.raise(exc);
            fpu.accrue();
            (fpu.fpsr & 0xff) as u8
        };
        assert_eq!(accrued(bits::BSUN), bits::AEXC_IOP);
        assert_eq!(accrued(bits::SNAN), bits::AEXC_IOP);
        assert_eq!(accrued(bits::OPERR), bits::AEXC_IOP);
        assert_eq!(accrued(bits::DZ), bits::AEXC_DZ);
        assert_eq!(accrued(bits::INEX1), bits::AEXC_INEX);
        assert_eq!(accrued(bits::INEX2), bits::AEXC_INEX);
        // Overflow accrues as overflow *and* inexact.
        assert_eq!(accrued(bits::OVFL), bits::AEXC_OVFL | bits::AEXC_INEX);
        // Underflow alone accrues nothing: it needs INEX2 with it.
        assert_eq!(accrued(bits::UNFL), 0);
        assert_eq!(
            accrued(bits::UNFL | bits::INEX2),
            bits::AEXC_UNFL | bits::AEXC_INEX
        );
    }

    #[test]
    fn the_highest_priority_enabled_exception_is_the_one_reported() {
        // §2.2.1: "the bits of the ENABLE byte are organized in decreasing
        // priority, left to right, i.e. BSUN is the highest priority, and
        // INEX1 is the lowest".
        let mut fpu = Fpu::RESET;
        fpu.raise(bits::OPERR | bits::INEX2);
        assert_eq!(fpu.pending_trap(), None, "no trap is enabled");
        fpu.fpcr = u32::from(bits::INEX2);
        assert_eq!(fpu.pending_trap(), Some(49));
        fpu.fpcr = u32::from(bits::OPERR | bits::INEX2);
        assert_eq!(fpu.pending_trap(), Some(52), "OPERR outranks INEX2");
        fpu.raise(bits::SNAN);
        fpu.fpcr |= u32::from(bits::SNAN);
        assert_eq!(fpu.pending_trap(), Some(54), "and SNAN outranks OPERR");
    }

    #[test]
    fn the_extended_format_in_memory_is_ninety_six_bits() {
        // Table 3-3: a word of sign and exponent, a word of zeros, then the
        // significand. 1.0 is $3fff 0000 8000000000000000.
        let one = F80::new(0x3fff, 1 << 63);
        assert_eq!(write_extended(one), [0x3fff_0000, 0x8000_0000, 0]);
        assert_eq!(read_extended(0x3fff_0000, 0x8000_0000, 0), one);
        // The middle word is ignored on the way in, as the hardware ignores
        // it.
        assert_eq!(read_extended(0x3fff_beef, 0x8000_0000, 0), one);
    }

    #[test]
    fn tininess_is_measured_against_the_rounding_precisions_range() {
        // §6.1.5's note: "An underflow can occur when the destination is a
        // floating-point data register and the selected rounding precision is
        // single or double even if the intermediate result is large enough to
        // be represented as an extended precision number."
        let small = F80::new(0x3fff - 200, 1 << 63); // 2^-200
        assert!(!is_tiny(small, F80::SPEC), "an extended normal");
        assert!(
            is_tiny(small, Spec::interchange(24, 127)),
            "tiny for single"
        );
        assert!(
            !is_tiny(small, Spec::interchange(53, 1023)),
            "not for double"
        );
        assert!(!is_tiny(F80::ZERO, F80::SPEC), "a zero is not tiny");
        assert!(!is_tiny(F80::INFINITY, F80::SPEC));
        assert!(!is_tiny(CREATED_NAN, F80::SPEC));
        // An extended subnormal is tiny at every precision.
        assert!(is_tiny(F80::new(0, 1), F80::SPEC));
    }
}
