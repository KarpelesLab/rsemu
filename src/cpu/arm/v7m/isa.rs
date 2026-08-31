//! The Thumb-2 instruction set, described once.
//!
//! ARMv7E-M has exactly one instruction set: T32. There is no ARM state, so
//! there is no second decoder — but T32 is two *encodings*, sixteen-bit and
//! thirty-two-bit, and [`decode`] takes both. Which one a halfword starts is
//! decided by [`is_32bit`], and a caller that has not fetched the second
//! halfword yet needs that answer before it can.
//!
//! One [`Insn`] value serves the interpreter and the disassembler, which is
//! the crate's rule for instruction tables (CLAUDE.md, "CPU cores"): decode
//! and disassembly cannot drift apart because there is only one description.
//!
//! # Shape of the decode
//!
//! [`decode`] is a transcription of DDI 0403's A5 encoding tables, in their
//! order:
//!
//! | Function | Manual |
//! | --- | --- |
//! | [`decode_16`] | A5.2, 16-bit Thumb |
//! | `decode_32` | A5.3, 32-bit Thumb |
//! | [`thumb_expand_imm`] | A5.3.2, the modified-immediate expansion |
//! | [`decode_imm_shift`] | A7.4.2, `DecodeImmShift` |
//!
//! Everything the interpreter needs to *execute* is in the returned value;
//! nothing re-reads the raw encoding. An encoding this architecture does not
//! define decodes to [`Insn::Undefined`], which the interpreter turns into a
//! UsageFault with `UFSR.UNDEFINSTR` set — never a panic and never a silent
//! `NOP`.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403 — A5 (the encoding
//! tables), A6.3/A7.3 (conditional execution and `IT`), A7.4 (shift and
//! immediate helpers) and A7.7 (the alphabetical instruction list). No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

// ---------------------------------------------------------------------------
// Bit helpers
// ---------------------------------------------------------------------------

/// Bits `hi..=lo` of `word`, shifted down.
#[inline]
#[must_use]
pub const fn field(word: u32, hi: u32, lo: u32) -> u32 {
    (word >> lo) & ((1u32 << (hi - lo + 1)) - 1)
}

/// Whether bit `n` of `word` is set.
#[inline]
#[must_use]
pub const fn bit(word: u32, n: u32) -> bool {
    word & (1 << n) != 0
}

/// Sign-extend the low `bits` of `value` to 32 bits.
#[inline]
#[must_use]
pub const fn sign_extend(value: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((value << shift) as i32) >> shift
}

// ---------------------------------------------------------------------------
// Conditions
// ---------------------------------------------------------------------------

/// A four-bit condition code (DDI 0403 A7.3).
///
/// A newtype rather than an enum because guest data reaches it: `IT` puts an
/// arbitrary four-bit value in `ITSTATE`, and `0b1111` — which the encodings
/// no longer use — must round-trip rather than panic.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Cond(pub u8);

impl Cond {
    /// Equal — `Z` set.
    pub const EQ: Cond = Cond(0);
    /// Always. The encoding `0b1110`, which most instructions imply.
    pub const AL: Cond = Cond(0b1110);

    /// Whether this condition holds for the given `APSR` flag bits.
    ///
    /// `0b1111` is "always" here. The architecture no longer assigns it a
    /// meaning in an encoding, but `ITSTATE[7:4]` can hold it and the
    /// pseudocode's `ConditionPassed` treats `cond<3:1> == '111'` as true
    /// (DDI 0403 A7.3.1).
    #[must_use]
    pub const fn passes(self, apsr: u32) -> bool {
        let n = apsr & (1 << 31) != 0;
        let z = apsr & (1 << 30) != 0;
        let c = apsr & (1 << 29) != 0;
        let v = apsr & (1 << 28) != 0;
        let base = match self.0 >> 1 {
            0b000 => z,
            0b001 => c,
            0b010 => n,
            0b011 => v,
            0b100 => c && !z,
            0b101 => n == v,
            0b110 => (n == v) && !z,
            _ => true,
        };
        // The low bit inverts, except for the "always" pair where inverting
        // would mean "never" and the architecture reserves the encoding.
        if self.0 & 1 != 0 && self.0 != 0b1111 {
            !base
        } else {
            base
        }
    }

    /// The assembler suffix, empty for `AL`.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self.0 & 0xf {
            0b0000 => "EQ",
            0b0001 => "NE",
            0b0010 => "CS",
            0b0011 => "CC",
            0b0100 => "MI",
            0b0101 => "PL",
            0b0110 => "VS",
            0b0111 => "VC",
            0b1000 => "HI",
            0b1001 => "LS",
            0b1010 => "GE",
            0b1011 => "LT",
            0b1100 => "GT",
            0b1101 => "LE",
            _ => "",
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.suffix())
    }
}

// ---------------------------------------------------------------------------
// Register names
// ---------------------------------------------------------------------------

/// A register number, printed the way a disassembly should read it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RegName(pub u8);

impl fmt::Display for RegName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 & 0xf {
            13 => f.write_str("sp"),
            14 => f.write_str("lr"),
            15 => f.write_str("pc"),
            n => write!(f, "r{n}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Shifts
// ---------------------------------------------------------------------------

/// One of the barrel shifter's five operations (DDI 0403 A7.4.2).
///
/// `RRX` is a separate variant here rather than "`ROR` by zero" as it is in
/// the encoding, because after `DecodeImmShift` the amount is always a real
/// shift distance and the caller never has to remember the special case
/// again.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShiftType {
    /// Logical shift left.
    Lsl,
    /// Logical shift right.
    Lsr,
    /// Arithmetic shift right.
    Asr,
    /// Rotate right.
    Ror,
    /// Rotate right by one through the carry flag.
    Rrx,
}

impl ShiftType {
    /// Decode the two-bit `type` field of a register-shifted operand.
    ///
    /// Never yields [`ShiftType::Rrx`]: that only exists once the amount is
    /// known to be zero, which is [`decode_imm_shift`]'s job.
    #[must_use]
    pub const fn from_bits(bits: u32) -> ShiftType {
        match bits & 3 {
            0 => ShiftType::Lsl,
            1 => ShiftType::Lsr,
            2 => ShiftType::Asr,
            _ => ShiftType::Ror,
        }
    }

    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            ShiftType::Lsl => "LSL",
            ShiftType::Lsr => "LSR",
            ShiftType::Asr => "ASR",
            ShiftType::Ror => "ROR",
            ShiftType::Rrx => "RRX",
        }
    }
}

impl fmt::Display for ShiftType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

/// A shift applied to a register operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Shift {
    /// Which shift.
    pub ty: ShiftType,
    /// How far, already resolved: `1..=32` for a real shift, `1` for `RRX`,
    /// and `0` only for `LSL #0`, which is no shift at all.
    pub amount: u8,
}

impl Shift {
    /// No shift.
    pub const NONE: Shift = Shift {
        ty: ShiftType::Lsl,
        amount: 0,
    };

    /// Whether this shift does nothing.
    #[must_use]
    pub const fn is_none(self) -> bool {
        matches!(self.ty, ShiftType::Lsl) && self.amount == 0
    }
}

impl fmt::Display for Shift {
    /// Prints as `, LSL #3`, or nothing at all for `LSL #0`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_none() {
            return Ok(());
        }
        if self.ty == ShiftType::Rrx {
            return f.write_str(", RRX");
        }
        write!(f, ", {} #{}", self.ty, self.amount)
    }
}

/// `DecodeImmShift(type, imm5)` (DDI 0403 A7.4.2).
///
/// The three zero-amount rewrites live here and nowhere else: `LSR #0` and
/// `ASR #0` mean thirty-two, `ROR #0` means `RRX`, and only `LSL #0` really
/// means "no shift".
#[must_use]
pub const fn decode_imm_shift(ty: u32, imm5: u32) -> Shift {
    match ty & 3 {
        0 => Shift {
            ty: ShiftType::Lsl,
            amount: imm5 as u8,
        },
        1 => Shift {
            ty: ShiftType::Lsr,
            amount: if imm5 == 0 { 32 } else { imm5 as u8 },
        },
        2 => Shift {
            ty: ShiftType::Asr,
            amount: if imm5 == 0 { 32 } else { imm5 as u8 },
        },
        _ => {
            if imm5 == 0 {
                Shift {
                    ty: ShiftType::Rrx,
                    amount: 1,
                }
            } else {
                Shift {
                    ty: ShiftType::Ror,
                    amount: imm5 as u8,
                }
            }
        }
    }
}

/// `ThumbExpandImm_C(imm12, carry_in)` (DDI 0403 A5.3.2).
///
/// Returns the expanded value and the carry it produces, or `None` for the
/// carry where the expansion leaves `APSR.C` alone. The two halves of the
/// encoding are genuinely different operations: the low quarter replicates a
/// byte into one of four patterns and touches no flag, and the rest is an
/// eight-bit value with its top bit forced set, rotated right, whose bit 31
/// becomes the carry.
#[must_use]
pub const fn thumb_expand_imm(imm12: u32) -> (u32, Option<bool>) {
    if field(imm12, 11, 10) == 0 {
        let byte = imm12 & 0xff;
        let value = match field(imm12, 9, 8) {
            0b00 => byte,
            0b01 => (byte << 16) | byte,
            0b10 => (byte << 24) | (byte << 8),
            _ => (byte << 24) | (byte << 16) | (byte << 8) | byte,
        };
        (value, None)
    } else {
        let unrotated = 0x80 | (imm12 & 0x7f);
        let value = unrotated.rotate_right(field(imm12, 11, 7));
        (value, Some(value & 0x8000_0000 != 0))
    }
}

// ---------------------------------------------------------------------------
// Operation enumerations
// ---------------------------------------------------------------------------

/// The data-processing operations, in the order the T32 `op` field lists them
/// where it lists them at all (DDI 0403 A5.3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DpOp {
    /// Bitwise and.
    And,
    /// Bit clear: `Rn AND NOT operand`.
    Bic,
    /// Bitwise or.
    Orr,
    /// Bitwise or not: `Rn OR NOT operand`. No ARMv5 counterpart.
    Orn,
    /// Bitwise exclusive or.
    Eor,
    /// Add.
    Add,
    /// Add with carry.
    Adc,
    /// Subtract with borrow.
    Sbc,
    /// Subtract.
    Sub,
    /// Reverse subtract: `operand - Rn`.
    Rsb,
    /// Move.
    Mov,
    /// Move not.
    Mvn,
    /// Test: `AND`, flags only.
    Tst,
    /// Test equivalence: `EOR`, flags only.
    Teq,
    /// Compare: `SUB`, flags only.
    Cmp,
    /// Compare negative: `ADD`, flags only.
    Cmn,
}

impl DpOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            DpOp::And => "AND",
            DpOp::Bic => "BIC",
            DpOp::Orr => "ORR",
            DpOp::Orn => "ORN",
            DpOp::Eor => "EOR",
            DpOp::Add => "ADD",
            DpOp::Adc => "ADC",
            DpOp::Sbc => "SBC",
            DpOp::Sub => "SUB",
            DpOp::Rsb => "RSB",
            DpOp::Mov => "MOV",
            DpOp::Mvn => "MVN",
            DpOp::Tst => "TST",
            DpOp::Teq => "TEQ",
            DpOp::Cmp => "CMP",
            DpOp::Cmn => "CMN",
        }
    }

    /// Whether the operation discards its result and only sets flags.
    #[must_use]
    pub const fn is_test(self) -> bool {
        matches!(self, DpOp::Tst | DpOp::Teq | DpOp::Cmp | DpOp::Cmn)
    }

    /// Whether the operation ignores `Rn`.
    #[must_use]
    pub const fn is_unary(self) -> bool {
        matches!(self, DpOp::Mov | DpOp::Mvn)
    }
}

/// How wide a memory access is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Size {
    /// One byte.
    Byte,
    /// Two bytes.
    Half,
    /// Four bytes.
    Word,
}

impl Size {
    /// How many bytes move.
    #[must_use]
    pub const fn bytes(self) -> u32 {
        match self {
            Size::Byte => 1,
            Size::Half => 2,
            Size::Word => 4,
        }
    }

    /// The mnemonic suffix: `B`, `H` or nothing.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            Size::Byte => "B",
            Size::Half => "H",
            Size::Word => "",
        }
    }
}

/// The offset half of an addressing mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MemOffset {
    /// An unsigned immediate; the instruction's `add` flag carries the sign.
    Imm(u32),
    /// A register, optionally shifted left by nought to three.
    Reg {
        /// The offset register.
        rm: u8,
        /// How far left to shift it.
        lsl: u8,
    },
}

impl fmt::Display for MemOffset {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            MemOffset::Imm(v) => write!(f, "#{v}"),
            MemOffset::Reg { rm, lsl: 0 } => write!(f, "{}", RegName(rm)),
            MemOffset::Reg { rm, lsl } => write!(f, "{}, LSL #{lsl}", RegName(rm)),
        }
    }
}

/// The second operand of a data-processing instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operand {
    /// An already-expanded immediate, and the carry the expansion produced.
    ///
    /// `None` means the expansion does not touch `APSR.C`; the interpreter
    /// then uses the carry flag's current value as the shifter carry, which
    /// is what leaves `C` unchanged.
    Imm {
        /// The value.
        value: u32,
        /// The shifter carry-out, if the encoding produces one.
        carry: Option<bool>,
    },
    /// A register with a constant shift.
    Reg {
        /// The register.
        rm: u8,
        /// Its shift.
        shift: Shift,
    },
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Operand::Imm { value, .. } => write!(f, "#{value}"),
            Operand::Reg { rm, shift } => write!(f, "{}{shift}", RegName(rm)),
        }
    }
}

/// The `SMLA`/`SMUL` half-word multiply family (DDI 0403 A7.7.166 onward).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HalfMulOp {
    /// `SMULxy`: 16 × 16 → 32.
    Smul,
    /// `SMLAxy`: 16 × 16 + 32, saturating flag on overflow.
    Smla,
    /// `SMULWy`: 32 × 16 → the top 32 bits of a 48-bit product.
    Smulw,
    /// `SMLAWy`: the same, accumulated.
    Smlaw,
    /// `SMLALxy`: 16 × 16 accumulated into a 64-bit pair.
    Smlal,
}

/// The dual-multiply family, which treats each operand as two halves
/// (DDI 0403 A7.7.156 onward).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DualMulOp {
    /// `SMUAD`: sum of the two half products.
    Smuad,
    /// `SMLAD`: the same, accumulated.
    Smlad,
    /// `SMUSD`: difference of the two half products.
    Smusd,
    /// `SMLSD`: the same, accumulated.
    Smlsd,
    /// `SMMUL`: the top 32 bits of a 32 × 32 product.
    Smmul,
    /// `SMMLA`: the same, accumulated.
    Smmla,
    /// `SMMLS`: the same, subtracted from the accumulator.
    Smmls,
    /// `SMLALD`: the sum, accumulated into a 64-bit pair.
    Smlald,
    /// `SMLSLD`: the difference, accumulated into a 64-bit pair.
    Smlsld,
}

impl DualMulOp {
    /// The assembler mnemonic, without the `X` suffix.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            DualMulOp::Smuad => "SMUAD",
            DualMulOp::Smlad => "SMLAD",
            DualMulOp::Smusd => "SMUSD",
            DualMulOp::Smlsd => "SMLSD",
            DualMulOp::Smmul => "SMMUL",
            DualMulOp::Smmla => "SMMLA",
            DualMulOp::Smmls => "SMMLS",
            DualMulOp::Smlald => "SMLALD",
            DualMulOp::Smlsld => "SMLSLD",
        }
    }

    /// Whether the `X`/`R` bit means "round" rather than "cross".
    ///
    /// The `SMM*` instructions reuse the bit for rounding, which is why the
    /// disassembler cannot print one suffix for the whole family.
    #[must_use]
    pub const fn bit_is_round(self) -> bool {
        matches!(self, DualMulOp::Smmul | DualMulOp::Smmla | DualMulOp::Smmls)
    }
}

/// The saturating 32-bit arithmetic of the DSP extension (DDI 0403 A7.7.128).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SatQOp {
    /// `QADD Rd, Rm, Rn`.
    Qadd,
    /// `QSUB Rd, Rm, Rn`.
    Qsub,
    /// `QDADD Rd, Rm, Rn`: `Rm + SAT(2 × Rn)`.
    Qdadd,
    /// `QDSUB Rd, Rm, Rn`: `Rm - SAT(2 × Rn)`.
    Qdsub,
}

impl SatQOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            SatQOp::Qadd => "QADD",
            SatQOp::Qsub => "QSUB",
            SatQOp::Qdadd => "QDADD",
            SatQOp::Qdsub => "QDSUB",
        }
    }
}

/// How a SIMD add or subtract treats its results (DDI 0403 A5.3.13/A5.3.14).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SimdMode {
    /// Signed, wrapping; sets the `GE` bits.
    Signed,
    /// Signed, saturating.
    SignedSat,
    /// Signed, halved.
    SignedHalve,
    /// Unsigned, wrapping; sets the `GE` bits.
    Unsigned,
    /// Unsigned, saturating.
    UnsignedSat,
    /// Unsigned, halved.
    UnsignedHalve,
}

impl SimdMode {
    /// The mnemonic prefix: `S`, `Q`, `SH`, `U`, `UQ` or `UH`.
    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            SimdMode::Signed => "S",
            SimdMode::SignedSat => "Q",
            SimdMode::SignedHalve => "SH",
            SimdMode::Unsigned => "U",
            SimdMode::UnsignedSat => "UQ",
            SimdMode::UnsignedHalve => "UH",
        }
    }

    /// Whether the mode is unsigned.
    #[must_use]
    pub const fn is_unsigned(self) -> bool {
        matches!(
            self,
            SimdMode::Unsigned | SimdMode::UnsignedSat | SimdMode::UnsignedHalve
        )
    }

    /// Whether the mode sets the `GE` bits. Only the plain forms do.
    #[must_use]
    pub const fn sets_ge(self) -> bool {
        matches!(self, SimdMode::Signed | SimdMode::Unsigned)
    }
}

/// Which lane arrangement a SIMD add or subtract uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SimdShape {
    /// Four byte lanes, added.
    Add8,
    /// Four byte lanes, subtracted.
    Sub8,
    /// Two halfword lanes, added.
    Add16,
    /// Two halfword lanes, subtracted.
    Sub16,
    /// Exchange then add/subtract: `Rd[15:0] = Rn[15:0] - Rm[31:16]`,
    /// `Rd[31:16] = Rn[31:16] + Rm[15:0]`.
    Asx,
    /// Subtract then exchange: the mirror of [`SimdShape::Asx`].
    Sax,
}

impl SimdShape {
    /// The mnemonic body: `ADD8`, `ASX` and so on.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            SimdShape::Add8 => "ADD8",
            SimdShape::Sub8 => "SUB8",
            SimdShape::Add16 => "ADD16",
            SimdShape::Sub16 => "SUB16",
            SimdShape::Asx => "ASX",
            SimdShape::Sax => "SAX",
        }
    }
}

/// The sign- and zero-extending moves, with and without an accumulator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExtendOp {
    /// Signed byte to word.
    Sxtb,
    /// Signed halfword to word.
    Sxth,
    /// Two signed bytes to two halfwords.
    Sxtb16,
    /// Unsigned byte to word.
    Uxtb,
    /// Unsigned halfword to word.
    Uxth,
    /// Two unsigned bytes to two halfwords.
    Uxtb16,
}

impl ExtendOp {
    /// The mnemonic body, without the `A` an accumulating form inserts.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            ExtendOp::Sxtb => "SXTB",
            ExtendOp::Sxth => "SXTH",
            ExtendOp::Sxtb16 => "SXTB16",
            ExtendOp::Uxtb => "UXTB",
            ExtendOp::Uxth => "UXTH",
            ExtendOp::Uxtb16 => "UXTB16",
        }
    }

    /// The accumulating mnemonic: `SXTAB` and friends.
    #[must_use]
    pub const fn accumulating_mnemonic(self) -> &'static str {
        match self {
            ExtendOp::Sxtb => "SXTAB",
            ExtendOp::Sxth => "SXTAH",
            ExtendOp::Sxtb16 => "SXTAB16",
            ExtendOp::Uxtb => "UXTAB",
            ExtendOp::Uxth => "UXTAH",
            ExtendOp::Uxtb16 => "UXTAB16",
        }
    }
}

/// The one-operand bit manipulations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MiscOp {
    /// Count leading zeros.
    Clz,
    /// Reverse the bit order.
    Rbit,
    /// Reverse the byte order of a word.
    Rev,
    /// Reverse the byte order within each halfword.
    Rev16,
    /// Reverse the bottom halfword's bytes and sign-extend.
    Revsh,
}

impl MiscOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            MiscOp::Clz => "CLZ",
            MiscOp::Rbit => "RBIT",
            MiscOp::Rev => "REV",
            MiscOp::Rev16 => "REV16",
            MiscOp::Revsh => "REVSH",
        }
    }
}

/// The bitfield instructions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BitfieldOp {
    /// Signed bitfield extract.
    Sbfx,
    /// Unsigned bitfield extract.
    Ubfx,
    /// Bitfield insert.
    Bfi,
    /// Bitfield clear.
    Bfc,
}

/// The memory-ordering instructions. All three are `NOP` in a core that
/// executes one access at a time in program order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BarrierOp {
    /// Data memory barrier.
    Dmb,
    /// Data synchronization barrier.
    Dsb,
    /// Instruction synchronization barrier.
    Isb,
}

impl BarrierOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            BarrierOp::Dmb => "DMB",
            BarrierOp::Dsb => "DSB",
            BarrierOp::Isb => "ISB",
        }
    }
}

/// The architectural hints (DDI 0403 A7.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HintOp {
    /// Do nothing.
    Nop,
    /// Yield: a hint to a hyperthreaded implementation, `NOP` here.
    Yield,
    /// Wait for event.
    Wfe,
    /// Wait for interrupt.
    Wfi,
    /// Send event.
    Sev,
    /// Debug hint, carrying a four-bit option.
    Dbg(u8),
    /// A preload hint: `PLD`, `PLDW` or `PLI`. Architecturally a `NOP` in a
    /// core with no caches of its own.
    Preload,
}

impl HintOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            HintOp::Nop => "NOP",
            HintOp::Yield => "YIELD",
            HintOp::Wfe => "WFE",
            HintOp::Wfi => "WFI",
            HintOp::Sev => "SEV",
            HintOp::Dbg(_) => "DBG",
            HintOp::Preload => "PLD",
        }
    }
}

// ---------------------------------------------------------------------------
// The instruction
// ---------------------------------------------------------------------------

/// One decoded T32 instruction.
///
/// The variants group encodings by *semantics* rather than by encoding table,
/// because that is what the interpreter dispatches on: `ADD Rd, Rn, #imm`
/// arrives here identically whether it was the sixteen-bit `ADDS` or the
/// thirty-two-bit `ADD.W`, and only [`Insn::width_of`] remembers which.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Insn {
    /// A data-processing operation with a register or immediate operand.
    DataProc {
        /// Which operation.
        op: DpOp,
        /// Whether it writes the flags.
        s: bool,
        /// Destination.
        rd: u8,
        /// First operand; ignored for `MOV` and `MVN`.
        rn: u8,
        /// Second operand.
        operand: Operand,
    },
    /// A shift by the low byte of a register: `LSL`, `LSR`, `ASR`, `ROR`.
    ShiftReg {
        /// Which shift. Never [`ShiftType::Rrx`], which has no register form.
        ty: ShiftType,
        /// Whether it writes the flags.
        s: bool,
        /// Destination.
        rd: u8,
        /// Value to shift.
        rn: u8,
        /// Shift amount, in the low byte.
        rm: u8,
    },
    /// `ADR`: the PC plus or minus an immediate, PC aligned to four.
    Adr {
        /// Destination.
        rd: u8,
        /// The immediate.
        imm: u32,
        /// Add rather than subtract.
        add: bool,
    },
    /// `MOVW` and `MOVT`: a sixteen-bit immediate into half of a register.
    MovImm16 {
        /// Write the top half, leaving the bottom alone (`MOVT`).
        top: bool,
        /// Destination.
        rd: u8,
        /// The immediate.
        imm: u16,
    },
    /// A PC-relative branch, conditional or not.
    Branch {
        /// The condition, or `None` for an unconditional branch.
        ///
        /// A conditional branch inside an `IT` block is a different thing
        /// from an unconditional one and the architecture forbids the
        /// combination; keeping them distinct is what lets the interpreter
        /// say so.
        cond: Option<Cond>,
        /// Byte offset from `PC + 4`.
        offset: i32,
    },
    /// `BL`: a branch that sets `LR`.
    BranchLink {
        /// Byte offset from `PC + 4`.
        offset: i32,
    },
    /// `BX Rm`.
    Bx {
        /// Register holding the target; bit 0 must be set.
        rm: u8,
    },
    /// `BLX Rm`.
    Blx {
        /// Register holding the target; bit 0 must be set.
        rm: u8,
    },
    /// `CBZ` and `CBNZ`: compare against zero and branch forward.
    Cbz {
        /// Branch when the register is *non*-zero.
        nonzero: bool,
        /// The register tested.
        rn: u8,
        /// Forward byte offset from `PC + 4`; never negative.
        offset: u32,
    },
    /// `TBB` and `TBH`: a byte or halfword table of forward branch offsets.
    TableBranch {
        /// Base of the table.
        rn: u8,
        /// Index into it.
        rm: u8,
        /// Halfword entries rather than byte entries.
        half: bool,
    },
    /// `IT`: make the next one to four instructions conditional.
    It {
        /// The base condition.
        cond: Cond,
        /// The `then`/`else` mask, never zero.
        mask: u8,
    },
    /// A single load or store with a register base.
    LoadStore {
        /// Whether it reads memory.
        load: bool,
        /// How wide.
        size: Size,
        /// Sign-extend a byte or halfword load.
        signed: bool,
        /// Data register.
        rt: u8,
        /// Base register.
        rn: u8,
        /// The offset.
        offset: MemOffset,
        /// Apply the offset before the access (`P`).
        index: bool,
        /// Add rather than subtract the offset (`U`).
        add: bool,
        /// Write the computed address back to `Rn` (`W`).
        wback: bool,
        /// This is an `LDRT`/`STRT`: the access is unprivileged whatever the
        /// core's mode.
        unpriv: bool,
    },
    /// A PC-relative load: `LDR Rt, [pc, #±imm]` and its sized forms.
    LoadLiteral {
        /// How wide.
        size: Size,
        /// Sign-extend a byte or halfword.
        signed: bool,
        /// Destination.
        rt: u8,
        /// The immediate.
        imm: u32,
        /// Add rather than subtract.
        add: bool,
    },
    /// `LDRD` and `STRD`.
    LoadStoreDual {
        /// Whether it reads memory.
        load: bool,
        /// First data register.
        rt: u8,
        /// Second data register.
        rt2: u8,
        /// Base register; fifteen means a literal load.
        rn: u8,
        /// The already-scaled immediate offset.
        imm: u32,
        /// Apply the offset before the access.
        index: bool,
        /// Add rather than subtract.
        add: bool,
        /// Write the computed address back.
        wback: bool,
    },
    /// `LDREX`, `STREX` and their byte and halfword forms.
    LoadStoreExclusive {
        /// Whether it reads memory.
        load: bool,
        /// How wide.
        size: Size,
        /// Where a store reports success; unused by a load.
        rd: u8,
        /// Data register.
        rt: u8,
        /// Base register.
        rn: u8,
        /// The already-scaled immediate offset.
        imm: u32,
    },
    /// `CLREX`.
    ClearExclusive,
    /// `LDM`, `STM`, `PUSH` and `POP`.
    LoadStoreMultiple {
        /// Whether it reads memory.
        load: bool,
        /// Base register.
        rn: u8,
        /// One bit per register.
        list: u16,
        /// Write the updated base back.
        wback: bool,
        /// Decrement before rather than increment after.
        before: bool,
    },
    /// `MUL`, `MLA` and `MLS`.
    Mul {
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// The accumulator, or `None` for a plain `MUL`.
        ra: Option<u8>,
        /// Subtract the product from the accumulator (`MLS`).
        sub: bool,
        /// Whether it writes the flags. Only the sixteen-bit `MULS` does.
        s: bool,
    },
    /// The 64-bit multiplies: `SMULL`, `UMULL`, `SMLAL`, `UMLAL`, `UMAAL`.
    MulLong {
        /// Signed rather than unsigned.
        signed: bool,
        /// Accumulate into the destination pair.
        accumulate: bool,
        /// Low half of the destination pair.
        rdlo: u8,
        /// High half of the destination pair.
        rdhi: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// This is `UMAAL`, which accumulates *both* halves separately.
        umaal: bool,
    },
    /// `SDIV` and `UDIV`.
    Div {
        /// Signed rather than unsigned.
        signed: bool,
        /// Destination.
        rd: u8,
        /// Dividend.
        rn: u8,
        /// Divisor.
        rm: u8,
    },
    /// The halfword multiply family.
    HalfMul {
        /// Which operation.
        op: HalfMulOp,
        /// Destination, or `RdHi` for `SMLAL`.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// Accumulator, or `RdLo` for `SMLAL`.
        ra: u8,
        /// Take the top half of `Rn`.
        x: bool,
        /// Take the top half of `Rm`.
        y: bool,
    },
    /// The dual multiply family.
    DualMul {
        /// Which operation.
        op: DualMulOp,
        /// Destination, or `RdHi` for the long forms.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// Accumulator, or `RdLo` for the long forms; fifteen means none.
        ra: u8,
        /// Swap `Rm`'s halves, or round, depending on the operation.
        x: bool,
    },
    /// `SSAT`, `USAT`, `SSAT16` and `USAT16`.
    Sat {
        /// Unsigned saturation.
        unsigned: bool,
        /// Saturate each halfword separately.
        halves: bool,
        /// Destination.
        rd: u8,
        /// Source.
        rn: u8,
        /// The saturation position, one to thirty-two (signed) or nought to
        /// thirty-one (unsigned).
        imm: u8,
        /// The shift applied before saturating.
        shift: Shift,
    },
    /// The saturating 32-bit arithmetic.
    SatQ {
        /// Which operation.
        op: SatQOp,
        /// Destination.
        rd: u8,
        /// The operand that is doubled, for the `QD` forms.
        rn: u8,
        /// The other operand.
        rm: u8,
    },
    /// A SIMD add or subtract.
    Simd {
        /// How results are treated.
        mode: SimdMode,
        /// Which lanes, and in which arrangement.
        shape: SimdShape,
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
    },
    /// `SEL`: pick bytes from `Rn` or `Rm` per the `GE` bits.
    Sel {
        /// Destination.
        rd: u8,
        /// Source for a set `GE` bit.
        rn: u8,
        /// Source for a clear `GE` bit.
        rm: u8,
    },
    /// `USAD8` and `USADA8`.
    Usad {
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// Accumulator; fifteen means none.
        ra: u8,
    },
    /// `PKHBT` and `PKHTB`.
    Pkh {
        /// The `TB` form: bottom half from `Rn`'s top, top half from the
        /// shifted `Rm`.
        tb: bool,
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand.
        rm: u8,
        /// The shift applied to `Rm`.
        shift: Shift,
    },
    /// A sign- or zero-extending move, with an optional accumulator.
    Extend {
        /// Which operation.
        op: ExtendOp,
        /// Destination.
        rd: u8,
        /// The accumulator; fifteen means there is none.
        rn: u8,
        /// Source.
        rm: u8,
        /// A rotate applied to the source first: nought, eight, sixteen or
        /// twenty-four.
        rotate: u8,
    },
    /// A one-operand bit manipulation.
    Misc {
        /// Which operation.
        op: MiscOp,
        /// Destination.
        rd: u8,
        /// Source.
        rm: u8,
    },
    /// A bitfield instruction.
    Bitfield {
        /// Which operation.
        op: BitfieldOp,
        /// Destination.
        rd: u8,
        /// Source; unused by `BFC`.
        rn: u8,
        /// Position of the field's low bit.
        lsb: u8,
        /// How many bits wide the field is.
        width: u8,
    },
    /// `MRS Rd, <spec_reg>`.
    Mrs {
        /// Destination.
        rd: u8,
        /// The `SYSm` field naming the special register.
        sysm: u8,
    },
    /// `MSR <spec_reg>, Rn`.
    Msr {
        /// Source.
        rn: u8,
        /// The `SYSm` field naming the special register.
        sysm: u8,
        /// The two-bit write mask, for the `APSR` forms.
        mask: u8,
    },
    /// `CPSIE` and `CPSID`.
    Cps {
        /// Enable rather than disable.
        enable: bool,
        /// Affect `PRIMASK` (the `i` flag).
        i: bool,
        /// Affect `FAULTMASK` (the `f` flag).
        f: bool,
    },
    /// A memory barrier.
    Barrier {
        /// Which barrier.
        op: BarrierOp,
        /// The four-bit option field.
        option: u8,
    },
    /// An architectural hint.
    Hint {
        /// Which hint.
        op: HintOp,
    },
    /// `BKPT`.
    Bkpt {
        /// The eight-bit comment field.
        imm: u8,
    },
    /// `SVC`.
    Svc {
        /// The eight-bit comment field.
        imm: u8,
    },
    /// `UDF` — permanently undefined, and the encoding a compiler emits to
    /// mean "this is unreachable".
    Udf {
        /// The comment field.
        imm: u16,
    },
    /// A coprocessor, Advanced SIMD or floating-point encoding.
    ///
    /// Kept distinct from [`Insn::Undefined`] because the fault differs: a
    /// coprocessor instruction with no coprocessor behind it raises
    /// `UFSR.NOCP`, not `UFSR.UNDEFINSTR`, and firmware detecting an absent
    /// FPU reads exactly that bit (DDI 0403 B3.2.15).
    Coproc {
        /// Which coprocessor the encoding names. Ten and eleven are the
        /// floating-point unit.
        cp: u8,
    },
    /// An encoding this architecture does not define.
    Undefined,
}

impl Insn {
    /// Whether the encoding this came from was thirty-two bits wide.
    ///
    /// Not carried in the value: the caller knows, because it decided how
    /// much to fetch. This is [`is_32bit`] restated for a decoded
    /// instruction's *first* halfword and exists only so the two answers
    /// cannot disagree.
    #[must_use]
    pub const fn width_of(first: u16) -> u32 {
        if is_32bit(first) { 4 } else { 2 }
    }
}

/// Whether `first` starts a thirty-two-bit instruction (DDI 0403 A5.1).
///
/// The three escape prefixes are `0b11101`, `0b11110` and `0b11111`; every
/// other top-five-bit pattern is a complete sixteen-bit instruction. `0b11100`
/// is the unconditional sixteen-bit `B`, which is why the test is not simply
/// "the top three bits are ones".
#[inline]
#[must_use]
pub const fn is_32bit(first: u16) -> bool {
    matches!(first >> 11, 0b11101..=0b11111)
}

/// Decode a whole instruction.
///
/// `first` is the halfword at the PC and `second` the one after it, which the
/// caller need only have fetched when [`is_32bit`] says so; it is ignored for
/// a sixteen-bit encoding.
#[must_use]
pub fn decode(first: u16, second: u16) -> Insn {
    if is_32bit(first) {
        decode_32(first, second)
    } else {
        decode_16(first)
    }
}

// ---------------------------------------------------------------------------
// 16-bit encodings (DDI 0403 A5.2)
// ---------------------------------------------------------------------------

/// Decode a sixteen-bit Thumb instruction (DDI 0403 A5.2).
#[must_use]
#[allow(clippy::too_many_lines)] // One arm per encoding group; splitting hides the table.
pub fn decode_16(raw: u16) -> Insn {
    let w = u32::from(raw);
    let rd = field(w, 2, 0) as u8;
    let rn = field(w, 5, 3) as u8;
    match field(w, 15, 10) {
        // A5.2.1 Shift (immediate), add, subtract, move, and compare.
        0b000000..=0b001111 => decode_16_shift_add(w),
        // A5.2.2 Data processing.
        0b010000 => {
            let rm = rn;
            match field(w, 9, 6) {
                0b0000 => dp_reg(DpOp::And, true, rd, rd, rm),
                0b0001 => dp_reg(DpOp::Eor, true, rd, rd, rm),
                0b0010 => Insn::ShiftReg {
                    ty: ShiftType::Lsl,
                    s: true,
                    rd,
                    rn: rd,
                    rm,
                },
                0b0011 => Insn::ShiftReg {
                    ty: ShiftType::Lsr,
                    s: true,
                    rd,
                    rn: rd,
                    rm,
                },
                0b0100 => Insn::ShiftReg {
                    ty: ShiftType::Asr,
                    s: true,
                    rd,
                    rn: rd,
                    rm,
                },
                0b0101 => dp_reg(DpOp::Adc, true, rd, rd, rm),
                0b0110 => dp_reg(DpOp::Sbc, true, rd, rd, rm),
                0b0111 => Insn::ShiftReg {
                    ty: ShiftType::Ror,
                    s: true,
                    rd,
                    rn: rd,
                    rm,
                },
                0b1000 => dp_reg(DpOp::Tst, true, 0, rd, rm),
                // `RSBS Rd, Rn, #0` — the assembler still spells it `NEG`.
                0b1001 => Insn::DataProc {
                    op: DpOp::Rsb,
                    s: true,
                    rd,
                    rn: rm,
                    operand: Operand::Imm {
                        value: 0,
                        carry: None,
                    },
                },
                0b1010 => dp_reg(DpOp::Cmp, true, 0, rd, rm),
                0b1011 => dp_reg(DpOp::Cmn, true, 0, rd, rm),
                0b1100 => dp_reg(DpOp::Orr, true, rd, rd, rm),
                0b1101 => Insn::Mul {
                    rd,
                    rn: rm,
                    rm: rd,
                    ra: None,
                    sub: false,
                    s: true,
                },
                0b1110 => dp_reg(DpOp::Bic, true, rd, rd, rm),
                _ => dp_reg(DpOp::Mvn, true, rd, 0, rm),
            }
        }
        // A5.2.3 Special data instructions and branch and exchange.
        0b010001 => {
            let rm = field(w, 6, 3) as u8;
            let hd = rd | if bit(w, 7) { 8 } else { 0 };
            match field(w, 9, 8) {
                0b00 => Insn::DataProc {
                    op: DpOp::Add,
                    s: false,
                    rd: hd,
                    rn: hd,
                    operand: Operand::Reg {
                        rm,
                        shift: Shift::NONE,
                    },
                },
                0b01 => Insn::DataProc {
                    op: DpOp::Cmp,
                    s: true,
                    rd: 0,
                    rn: hd,
                    operand: Operand::Reg {
                        rm,
                        shift: Shift::NONE,
                    },
                },
                0b10 => Insn::DataProc {
                    op: DpOp::Mov,
                    s: false,
                    rd: hd,
                    rn: 0,
                    operand: Operand::Reg {
                        rm,
                        shift: Shift::NONE,
                    },
                },
                _ => {
                    if bit(w, 7) {
                        Insn::Blx { rm }
                    } else {
                        Insn::Bx { rm }
                    }
                }
            }
        }
        // LDR (literal).
        0b010010 | 0b010011 => Insn::LoadLiteral {
            size: Size::Word,
            signed: false,
            rt: field(w, 10, 8) as u8,
            imm: field(w, 7, 0) * 4,
            add: true,
        },
        // A5.2.4 Load/store single data item, register offset.
        0b010100..=0b010111 => {
            let rm = field(w, 8, 6) as u8;
            let (load, size, signed) = match field(w, 11, 9) {
                0b000 => (false, Size::Word, false),
                0b001 => (false, Size::Half, false),
                0b010 => (false, Size::Byte, false),
                0b011 => (true, Size::Byte, true),
                0b100 => (true, Size::Word, false),
                0b101 => (true, Size::Half, false),
                0b110 => (true, Size::Byte, false),
                _ => (true, Size::Half, true),
            };
            Insn::LoadStore {
                load,
                size,
                signed,
                rt: rd,
                rn,
                offset: MemOffset::Reg { rm, lsl: 0 },
                index: true,
                add: true,
                wback: false,
                unpriv: false,
            }
        }
        // Load/store with a scaled five-bit immediate: word, byte, halfword.
        0b011000..=0b100011 => {
            let size = match field(w, 15, 12) {
                0b0110 => Size::Word,
                0b0111 => Size::Byte,
                _ => Size::Half,
            };
            Insn::LoadStore {
                load: bit(w, 11),
                size,
                signed: false,
                rt: rd,
                rn,
                offset: MemOffset::Imm(field(w, 10, 6) * size.bytes()),
                index: true,
                add: true,
                wback: false,
                unpriv: false,
            }
        }
        // Load/store relative to SP.
        0b100100..=0b100111 => Insn::LoadStore {
            load: bit(w, 11),
            size: Size::Word,
            signed: false,
            rt: field(w, 10, 8) as u8,
            rn: 13,
            offset: MemOffset::Imm(field(w, 7, 0) * 4),
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        },
        // ADR — `ADD Rd, pc, #imm8*4`.
        0b101000 | 0b101001 => Insn::Adr {
            rd: field(w, 10, 8) as u8,
            imm: field(w, 7, 0) * 4,
            add: true,
        },
        // ADD Rd, sp, #imm8*4.
        0b101010 | 0b101011 => Insn::DataProc {
            op: DpOp::Add,
            s: false,
            rd: field(w, 10, 8) as u8,
            rn: 13,
            operand: Operand::Imm {
                value: field(w, 7, 0) * 4,
                carry: None,
            },
        },
        // A5.2.5 Miscellaneous 16-bit instructions.
        0b101100..=0b101111 => decode_16_misc(w),
        // STM / LDM.
        0b110000 | 0b110001 => Insn::LoadStoreMultiple {
            load: false,
            rn: field(w, 10, 8) as u8,
            list: (w & 0xff) as u16,
            wback: true,
            before: false,
        },
        0b110010 | 0b110011 => {
            let rn = field(w, 10, 8) as u8;
            let list = (w & 0xff) as u16;
            Insn::LoadStoreMultiple {
                load: true,
                rn,
                list,
                // "`LDM` with the base in the list does not write back" —
                // DDI 0403 A7.7.41's `wback` is `registers<n> == '0'`.
                wback: list & (1 << rn) == 0,
                before: false,
            }
        }
        // Conditional branch, UDF and SVC.
        0b110100..=0b110111 => match field(w, 11, 8) {
            0b1110 => Insn::Udf {
                imm: (w & 0xff) as u16,
            },
            0b1111 => Insn::Svc {
                imm: (w & 0xff) as u8,
            },
            cond => Insn::Branch {
                cond: Some(Cond(cond as u8)),
                offset: sign_extend(w & 0xff, 8) * 2,
            },
        },
        // Unconditional branch.
        _ => Insn::Branch {
            cond: None,
            offset: sign_extend(w & 0x7ff, 11) * 2,
        },
    }
}

/// A data-processing instruction with an unshifted register operand.
const fn dp_reg(op: DpOp, s: bool, rd: u8, rn: u8, rm: u8) -> Insn {
    Insn::DataProc {
        op,
        s,
        rd,
        rn,
        operand: Operand::Reg {
            rm,
            shift: Shift::NONE,
        },
    }
}

/// A data-processing instruction with a plain unsigned immediate that has no
/// carry of its own.
const fn dp_imm(op: DpOp, s: bool, rd: u8, rn: u8, value: u32) -> Insn {
    Insn::DataProc {
        op,
        s,
        rd,
        rn,
        operand: Operand::Imm { value, carry: None },
    }
}

/// A5.2.1: shift by immediate, add, subtract, move and compare.
fn decode_16_shift_add(w: u32) -> Insn {
    let rd = field(w, 2, 0) as u8;
    let rn = field(w, 5, 3) as u8;
    match field(w, 13, 9) {
        // The three immediate shifts. `LSL #0` is `MOV Rd, Rm` (T2), which is
        // the same instruction with a zero shift, so it needs no special case
        // beyond what the shift helper already does.
        0b00000..=0b01011 => {
            let shift = decode_imm_shift(field(w, 12, 11), field(w, 10, 6));
            Insn::DataProc {
                op: DpOp::Mov,
                s: true,
                rd,
                rn: 0,
                operand: Operand::Reg { rm: rn, shift },
            }
        }
        0b01100 => dp_reg(DpOp::Add, true, rd, rn, field(w, 8, 6) as u8),
        0b01101 => dp_reg(DpOp::Sub, true, rd, rn, field(w, 8, 6) as u8),
        0b01110 => dp_imm(DpOp::Add, true, rd, rn, field(w, 8, 6)),
        0b01111 => dp_imm(DpOp::Sub, true, rd, rn, field(w, 8, 6)),
        0b10000..=0b10011 => dp_imm(DpOp::Mov, true, field(w, 10, 8) as u8, 0, w & 0xff),
        0b10100..=0b10111 => dp_imm(DpOp::Cmp, true, 0, field(w, 10, 8) as u8, w & 0xff),
        0b11000..=0b11011 => {
            let r = field(w, 10, 8) as u8;
            dp_imm(DpOp::Add, true, r, r, w & 0xff)
        }
        _ => {
            let r = field(w, 10, 8) as u8;
            dp_imm(DpOp::Sub, true, r, r, w & 0xff)
        }
    }
}

/// A5.2.5: the `1011` block.
fn decode_16_misc(w: u32) -> Insn {
    let rd = field(w, 2, 0) as u8;
    let rm = field(w, 5, 3) as u8;
    match field(w, 11, 5) {
        // ADD/SUB SP, SP, #imm7*4.
        0b0000000..=0b0000011 => dp_imm(DpOp::Add, false, 13, 13, field(w, 6, 0) * 4),
        0b0000100..=0b0000111 => dp_imm(DpOp::Sub, false, 13, 13, field(w, 6, 0) * 4),
        // CBZ / CBNZ. The offset is `i:imm5:'0'`, always forward.
        0b0001000..=0b0001111
        | 0b0011000..=0b0011111
        | 0b1001000..=0b1001111
        | 0b1011000..=0b1011111 => Insn::Cbz {
            nonzero: bit(w, 11),
            rn: rd,
            offset: (field(w, 9, 9) << 6) | (field(w, 7, 3) << 1),
        },
        0b0010000 | 0b0010001 => extend16(ExtendOp::Sxth, rd, rm),
        0b0010010 | 0b0010011 => extend16(ExtendOp::Sxtb, rd, rm),
        0b0010100 | 0b0010101 => extend16(ExtendOp::Uxth, rd, rm),
        0b0010110 | 0b0010111 => extend16(ExtendOp::Uxtb, rd, rm),
        // PUSH: `M` adds LR.
        0b0100000..=0b0101111 => Insn::LoadStoreMultiple {
            load: false,
            rn: 13,
            list: ((w & 0xff) as u16) | if bit(w, 8) { 0x4000 } else { 0 },
            wback: true,
            before: true,
        },
        // CPS.
        0b0110011 => Insn::Cps {
            enable: !bit(w, 4),
            i: bit(w, 1),
            f: bit(w, 0),
        },
        0b1010000 | 0b1010001 => Insn::Misc {
            op: MiscOp::Rev,
            rd,
            rm,
        },
        0b1010010 | 0b1010011 => Insn::Misc {
            op: MiscOp::Rev16,
            rd,
            rm,
        },
        0b1010110 | 0b1010111 => Insn::Misc {
            op: MiscOp::Revsh,
            rd,
            rm,
        },
        // POP: `P` adds PC.
        0b1100000..=0b1101111 => Insn::LoadStoreMultiple {
            load: true,
            rn: 13,
            list: ((w & 0xff) as u16) | if bit(w, 8) { 0x8000 } else { 0 },
            wback: true,
            before: false,
        },
        0b1110000..=0b1110111 => Insn::Bkpt {
            imm: (w & 0xff) as u8,
        },
        // IT and the hints share `1011 1111`; a zero mask means a hint.
        0b1111000..=0b1111111 => {
            let mask = field(w, 3, 0) as u8;
            if mask != 0 {
                Insn::It {
                    cond: Cond(field(w, 7, 4) as u8),
                    mask,
                }
            } else {
                Insn::Hint {
                    op: match field(w, 7, 4) {
                        0b0000 => HintOp::Nop,
                        0b0001 => HintOp::Yield,
                        0b0010 => HintOp::Wfe,
                        0b0011 => HintOp::Wfi,
                        0b0100 => HintOp::Sev,
                        // Everything else in this block is a hint the
                        // architecture has not assigned, and an unassigned
                        // hint executes as a NOP (DDI 0403 A7.7.87).
                        _ => HintOp::Nop,
                    },
                }
            }
        }
        _ => Insn::Undefined,
    }
}

/// A sixteen-bit extend, which never accumulates and never rotates.
const fn extend16(op: ExtendOp, rd: u8, rm: u8) -> Insn {
    Insn::Extend {
        op,
        rd,
        rn: 15,
        rm,
        rotate: 0,
    }
}

// ---------------------------------------------------------------------------
// 32-bit encodings (DDI 0403 A5.3)
// ---------------------------------------------------------------------------

/// Decode a thirty-two-bit T32 instruction (DDI 0403 A5.3).
fn decode_32(hw1: u16, hw2: u16) -> Insn {
    let a = u32::from(hw1);
    let b = u32::from(hw2);
    let op1 = field(a, 12, 11);
    let op2 = field(a, 10, 4);
    match op1 {
        0b01 => {
            if op2 & 0b1000000 != 0 {
                // Coprocessor, Advanced SIMD, floating point (A5.3.18).
                Insn::Coproc {
                    cp: field(b, 11, 8) as u8,
                }
            } else if op2 & 0b1100100 == 0b0000000 {
                decode_32_ldm_stm(a, b)
            } else if op2 & 0b1100100 == 0b0000100 {
                decode_32_dual_exclusive(a, b)
            } else {
                decode_32_dp_shifted(a, b)
            }
        }
        0b10 => {
            if bit(b, 15) {
                decode_32_branch_misc(a, b)
            } else if bit(a, 9) {
                decode_32_dp_plain_imm(a, b)
            } else {
                decode_32_dp_modified_imm(a, b)
            }
        }
        0b11 => {
            if op2 & 0b1000000 != 0 {
                Insn::Coproc {
                    cp: field(b, 11, 8) as u8,
                }
            } else if op2 & 0b1110001 == 0b0000000 {
                decode_32_store_single(a, b)
            } else if op2 & 0b1100111 == 0b0000001
                || op2 & 0b1100111 == 0b0000011
                || op2 & 0b1100111 == 0b0000101
            {
                decode_32_load_single(a, b)
            } else if op2 & 0b1110000 == 0b0100000 {
                decode_32_dp_register(a, b)
            } else if op2 & 0b1111000 == 0b0110000 {
                decode_32_multiply(a, b)
            } else if op2 & 0b1111000 == 0b0111000 {
                decode_32_long_multiply(a, b)
            } else {
                Insn::Undefined
            }
        }
        _ => Insn::Undefined,
    }
}

/// The `op` field shared by A5.3.1 and A5.3.11, and the aliasing rules that
/// turn `Rd == PC` into a test and `Rn == PC` into a move.
fn dp_op_from_bits(op: u32, s: bool, rd: u8, rn: u8) -> Option<(DpOp, u8, u8)> {
    let (base, test, unary) = match op {
        0b0000 => (DpOp::And, Some(DpOp::Tst), None),
        0b0001 => (DpOp::Bic, None, None),
        0b0010 => (DpOp::Orr, None, Some(DpOp::Mov)),
        0b0011 => (DpOp::Orn, None, Some(DpOp::Mvn)),
        0b0100 => (DpOp::Eor, Some(DpOp::Teq), None),
        0b1000 => (DpOp::Add, Some(DpOp::Cmn), None),
        0b1010 => (DpOp::Adc, None, None),
        0b1011 => (DpOp::Sbc, None, None),
        0b1101 => (DpOp::Sub, Some(DpOp::Cmp), None),
        0b1110 => (DpOp::Rsb, None, None),
        _ => return None,
    };
    if rd == 15
        && s
        && let Some(test) = test
    {
        return Some((test, 0, rn));
    }
    if rn == 15
        && let Some(unary) = unary
    {
        return Some((unary, rd, 0));
    }
    if rd == 15 {
        // `Rd == PC` outside the test aliases is not an encoding this
        // architecture defines: T32 has no data-processing write to the PC.
        return None;
    }
    Some((base, rd, rn))
}

/// A5.3.1: data processing with a modified immediate.
fn decode_32_dp_modified_imm(a: u32, b: u32) -> Insn {
    let s = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let imm12 = (field(a, 10, 10) << 11) | (field(b, 14, 12) << 8) | (b & 0xff);
    let (value, carry) = thumb_expand_imm(imm12);
    match dp_op_from_bits(field(a, 8, 5), s, rd, rn) {
        Some((op, rd, rn)) => Insn::DataProc {
            op,
            s,
            rd,
            rn,
            operand: Operand::Imm { value, carry },
        },
        None => Insn::Undefined,
    }
}

/// A5.3.3: data processing with a plain binary immediate.
fn decode_32_dp_plain_imm(a: u32, b: u32) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let i = field(a, 10, 10);
    let imm3 = field(b, 14, 12);
    let imm8 = b & 0xff;
    let imm12 = (i << 11) | (imm3 << 8) | imm8;
    // `imm3:imm2` is the shift amount of the saturate and bitfield forms.
    let imm5 = (imm3 << 2) | field(b, 7, 6);
    match field(a, 8, 4) {
        0b00000 => {
            if rn == 15 {
                Insn::Adr {
                    rd,
                    imm: imm12,
                    add: true,
                }
            } else {
                dp_imm(DpOp::Add, false, rd, rn, imm12)
            }
        }
        0b00100 => Insn::MovImm16 {
            top: false,
            rd,
            imm: ((field(a, 3, 0) << 12) | imm12) as u16,
        },
        0b01010 => {
            if rn == 15 {
                Insn::Adr {
                    rd,
                    imm: imm12,
                    add: false,
                }
            } else {
                dp_imm(DpOp::Sub, false, rd, rn, imm12)
            }
        }
        0b01100 => Insn::MovImm16 {
            top: true,
            rd,
            imm: ((field(a, 3, 0) << 12) | imm12) as u16,
        },
        // SSAT with a left shift, and SSAT with a right shift or SSAT16.
        0b10000 | 0b10010 => {
            let sh = u32::from(bit(a, 5));
            if sh == 1 && imm5 == 0 {
                Insn::Sat {
                    unsigned: false,
                    halves: true,
                    rd,
                    rn,
                    imm: (field(b, 4, 0) + 1) as u8,
                    shift: Shift::NONE,
                }
            } else {
                Insn::Sat {
                    unsigned: false,
                    halves: false,
                    rd,
                    rn,
                    imm: (field(b, 4, 0) + 1) as u8,
                    shift: decode_imm_shift(if sh == 1 { 2 } else { 0 }, imm5),
                }
            }
        }
        0b10100 => Insn::Bitfield {
            op: BitfieldOp::Sbfx,
            rd,
            rn,
            lsb: imm5 as u8,
            width: (field(b, 4, 0) + 1) as u8,
        },
        0b10110 => {
            // `msb` rather than a width: the field runs from `lsb` to `msb`
            // inclusive, and an msb below the lsb is UNPREDICTABLE.
            let msb = field(b, 4, 0);
            if msb < imm5 {
                return Insn::Undefined;
            }
            Insn::Bitfield {
                op: if rn == 15 {
                    BitfieldOp::Bfc
                } else {
                    BitfieldOp::Bfi
                },
                rd,
                rn,
                lsb: imm5 as u8,
                width: (msb - imm5 + 1) as u8,
            }
        }
        0b11000 | 0b11010 => {
            let sh = u32::from(bit(a, 5));
            if sh == 1 && imm5 == 0 {
                Insn::Sat {
                    unsigned: true,
                    halves: true,
                    rd,
                    rn,
                    imm: field(b, 4, 0) as u8,
                    shift: Shift::NONE,
                }
            } else {
                Insn::Sat {
                    unsigned: true,
                    halves: false,
                    rd,
                    rn,
                    imm: field(b, 4, 0) as u8,
                    shift: decode_imm_shift(if sh == 1 { 2 } else { 0 }, imm5),
                }
            }
        }
        0b11100 => Insn::Bitfield {
            op: BitfieldOp::Ubfx,
            rd,
            rn,
            lsb: imm5 as u8,
            width: (field(b, 4, 0) + 1) as u8,
        },
        _ => Insn::Undefined,
    }
}

/// A5.3.11: data processing with a shifted register.
fn decode_32_dp_shifted(a: u32, b: u32) -> Insn {
    let s = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let shift = decode_imm_shift(field(b, 5, 4), (field(b, 14, 12) << 2) | field(b, 7, 6));
    let op = field(a, 8, 5);
    if op == 0b0110 {
        // PKHBT / PKHTB: the same field layout, a different meaning for the
        // shift type bit, and `S` must be zero.
        // `hw2` is `(0) imm3 Rd imm2 tb 0 Rm`: bit 5 is `tb` and bit 4 is a
        // fixed zero, which is the opposite of where a shifted-register
        // operand keeps its shift type.
        if s || bit(b, 4) {
            return Insn::Undefined;
        }
        let tb = bit(b, 5);
        return Insn::Pkh {
            tb,
            rd,
            rn,
            rm,
            shift: if tb {
                decode_imm_shift(2, (field(b, 14, 12) << 2) | field(b, 7, 6))
            } else {
                decode_imm_shift(0, (field(b, 14, 12) << 2) | field(b, 7, 6))
            },
        };
    }
    match dp_op_from_bits(op, s, rd, rn) {
        Some((DpOp::Mov, rd, _)) => {
            // `MOV Rd, Rm, <shift> #n` — the assembler spells the shifted
            // forms `LSL`, `LSR`, `ASR` and `ROR`, and `RRX` has no amount.
            Insn::DataProc {
                op: DpOp::Mov,
                s,
                rd,
                rn: 0,
                operand: Operand::Reg { rm, shift },
            }
        }
        Some((op, rd, rn)) => Insn::DataProc {
            op,
            s,
            rd,
            rn,
            operand: Operand::Reg { rm, shift },
        },
        None => Insn::Undefined,
    }
}

/// A5.3.4: branches and miscellaneous control.
fn decode_32_branch_misc(a: u32, b: u32) -> Insn {
    let op = field(a, 10, 4);
    let op1 = field(b, 14, 12);
    let s = field(a, 10, 10);
    let j1 = field(b, 13, 13);
    let j2 = field(b, 11, 11);
    let imm11 = b & 0x7ff;
    match op1 {
        0b000 | 0b010 => {
            if op & 0b0111000 == 0b0111000 {
                return decode_32_system(a, b, op);
            }
            // B<cond>.W (T3): ±1 MiB.
            let imm = (s << 20) | (j2 << 19) | (j1 << 18) | (field(a, 9, 6) << 12) | (imm11 << 1);
            // The condition field is `hw1[9:6]`, and `111x` is not a
            // condition — those encodings are the system space handled above.
            Insn::Branch {
                cond: Some(Cond(field(a, 9, 6) as u8)),
                offset: sign_extend(imm, 21),
            }
        }
        0b001 | 0b011 => {
            let i1 = 1 - (j1 ^ s);
            let i2 = 1 - (j2 ^ s);
            let imm = (s << 24) | (i1 << 23) | (i2 << 22) | (field(a, 9, 0) << 12) | (imm11 << 1);
            Insn::Branch {
                cond: None,
                offset: sign_extend(imm, 25),
            }
        }
        0b101 | 0b111 => {
            let i1 = 1 - (j1 ^ s);
            let i2 = 1 - (j2 ^ s);
            let imm = (s << 24) | (i1 << 23) | (i2 << 22) | (field(a, 9, 0) << 12) | (imm11 << 1);
            Insn::BranchLink {
                offset: sign_extend(imm, 25),
            }
        }
        // `BLX (immediate)` would be here. ARMv7-M has no ARM state, so the
        // encoding is undefined rather than an interworking call
        // (DDI 0403 A5.3.4's note).
        _ => Insn::Undefined,
    }
}

/// The `0111xxx` corner of A5.3.4: `MSR`, `MRS`, `CPS`, hints and barriers.
fn decode_32_system(a: u32, b: u32, op: u32) -> Insn {
    match op {
        0b0111000 | 0b0111001 => Insn::Msr {
            rn: field(a, 3, 0) as u8,
            sysm: (b & 0xff) as u8,
            mask: field(b, 11, 10) as u8,
        },
        0b0111010 => {
            // CPS and the hints share this encoding; `hw2[10:8] == 0` selects
            // the hints.
            if field(b, 10, 8) == 0 {
                Insn::Hint {
                    op: match b & 0xff {
                        0x00 => HintOp::Nop,
                        0x01 => HintOp::Yield,
                        0x02 => HintOp::Wfe,
                        0x03 => HintOp::Wfi,
                        0x04 => HintOp::Sev,
                        v if v & 0xf0 == 0xf0 => HintOp::Dbg((v & 0xf) as u8),
                        _ => HintOp::Nop,
                    },
                }
            } else {
                Insn::Undefined
            }
        }
        0b0111011 => match field(b, 7, 4) {
            0b0010 => Insn::ClearExclusive,
            0b0100 => Insn::Barrier {
                op: BarrierOp::Dsb,
                option: field(b, 3, 0) as u8,
            },
            0b0101 => Insn::Barrier {
                op: BarrierOp::Dmb,
                option: field(b, 3, 0) as u8,
            },
            0b0110 => Insn::Barrier {
                op: BarrierOp::Isb,
                option: field(b, 3, 0) as u8,
            },
            _ => Insn::Undefined,
        },
        0b0111110 | 0b0111111 => Insn::Mrs {
            rd: field(b, 11, 8) as u8,
            sysm: (b & 0xff) as u8,
        },
        // `1111111` with `hw2[15:12] == 1010` is the permanently undefined
        // `UDF.W` (DDI 0403 A7.7.194).
        0b1111111 => Insn::Udf {
            imm: ((field(a, 3, 0) << 12) | (b & 0xfff)) as u16,
        },
        _ => Insn::Undefined,
    }
}

/// A5.3.5: load and store multiple.
fn decode_32_ldm_stm(a: u32, b: u32) -> Insn {
    let l = bit(a, 4);
    let w = bit(a, 5);
    let rn = field(a, 3, 0) as u8;
    let list = b as u16;
    match field(a, 8, 7) {
        0b01 => Insn::LoadStoreMultiple {
            load: l,
            rn,
            list,
            wback: w && !(l && list & (1 << rn) != 0),
            before: false,
        },
        0b10 => Insn::LoadStoreMultiple {
            load: l,
            rn,
            list,
            wback: w && !(l && list & (1 << rn) != 0),
            before: true,
        },
        // `SRS` and `RFE` — ARMv7-M has no banked modes, so neither exists.
        _ => Insn::Undefined,
    }
}

/// A5.3.6: load/store dual, load/store exclusive, table branch.
fn decode_32_dual_exclusive(a: u32, b: u32) -> Insn {
    let p = bit(a, 8);
    let u = bit(a, 7);
    let w = bit(a, 5);
    let l = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rt = field(b, 15, 12) as u8;
    let rt2 = field(b, 11, 8) as u8;
    if !p && !w {
        if !u {
            return Insn::LoadStoreExclusive {
                load: l,
                size: Size::Word,
                rd: rt2,
                rt,
                rn,
                imm: (b & 0xff) * 4,
            };
        }
        return match (l, field(b, 7, 4)) {
            (false, 0b0100) => Insn::LoadStoreExclusive {
                load: false,
                size: Size::Byte,
                rd: field(b, 3, 0) as u8,
                rt,
                rn,
                imm: 0,
            },
            (false, 0b0101) => Insn::LoadStoreExclusive {
                load: false,
                size: Size::Half,
                rd: field(b, 3, 0) as u8,
                rt,
                rn,
                imm: 0,
            },
            (true, 0b0000) => Insn::TableBranch {
                rn,
                rm: field(b, 3, 0) as u8,
                half: false,
            },
            (true, 0b0001) => Insn::TableBranch {
                rn,
                rm: field(b, 3, 0) as u8,
                half: true,
            },
            (true, 0b0100) => Insn::LoadStoreExclusive {
                load: true,
                size: Size::Byte,
                rd: 0,
                rt,
                rn,
                imm: 0,
            },
            (true, 0b0101) => Insn::LoadStoreExclusive {
                load: true,
                size: Size::Half,
                rd: 0,
                rt,
                rn,
                imm: 0,
            },
            _ => Insn::Undefined,
        };
    }
    Insn::LoadStoreDual {
        load: l,
        rt,
        rt2,
        rn,
        imm: (b & 0xff) * 4,
        index: p,
        add: u,
        wback: w,
    }
}

/// A5.3.10: store a single data item.
fn decode_32_store_single(a: u32, b: u32) -> Insn {
    let size = match field(a, 6, 5) {
        0b00 => Size::Byte,
        0b01 => Size::Half,
        0b10 => Size::Word,
        _ => return Insn::Undefined,
    };
    let rn = field(a, 3, 0) as u8;
    let rt = field(b, 15, 12) as u8;
    if rn == 15 {
        return Insn::Undefined;
    }
    if bit(a, 7) {
        return Insn::LoadStore {
            load: false,
            size,
            signed: false,
            rt,
            rn,
            offset: MemOffset::Imm(b & 0xfff),
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        };
    }
    if field(b, 11, 6) == 0 {
        return Insn::LoadStore {
            load: false,
            size,
            signed: false,
            rt,
            rn,
            offset: MemOffset::Reg {
                rm: field(b, 3, 0) as u8,
                lsl: field(b, 5, 4) as u8,
            },
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        };
    }
    if !bit(b, 11) {
        return Insn::Undefined;
    }
    let (p, u, wb) = (bit(b, 10), bit(b, 9), bit(b, 8));
    Insn::LoadStore {
        load: false,
        size,
        signed: false,
        rt,
        rn,
        offset: MemOffset::Imm(b & 0xff),
        index: p,
        add: u,
        wback: wb,
        // `STRT` is `P == 1, U == 1, W == 0` in this space.
        unpriv: p && u && !wb,
    }
}

/// A5.3.7/A5.3.8/A5.3.9: load a single data item, and the preload hints.
fn decode_32_load_single(a: u32, b: u32) -> Insn {
    let signed = bit(a, 8);
    let size = match field(a, 6, 5) {
        0b00 => Size::Byte,
        0b01 => Size::Half,
        0b10 => Size::Word,
        _ => return Insn::Undefined,
    };
    if signed && size == Size::Word {
        return Insn::Undefined;
    }
    let rn = field(a, 3, 0) as u8;
    let rt = field(b, 15, 12) as u8;
    // `Rt == PC` on a byte or halfword load is a preload hint rather than a
    // load (DDI 0403 A7.7.88 `PLD`, A7.7.90 `PLI`).
    if rt == 15 && size != Size::Word {
        return Insn::Hint {
            op: HintOp::Preload,
        };
    }
    if rn == 15 {
        return Insn::LoadLiteral {
            size,
            signed,
            rt,
            imm: b & 0xfff,
            add: bit(a, 7),
        };
    }
    if bit(a, 7) {
        return Insn::LoadStore {
            load: true,
            size,
            signed,
            rt,
            rn,
            offset: MemOffset::Imm(b & 0xfff),
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        };
    }
    if field(b, 11, 6) == 0 {
        return Insn::LoadStore {
            load: true,
            size,
            signed,
            rt,
            rn,
            offset: MemOffset::Reg {
                rm: field(b, 3, 0) as u8,
                lsl: field(b, 5, 4) as u8,
            },
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        };
    }
    if !bit(b, 11) {
        return Insn::Undefined;
    }
    let (p, u, wb) = (bit(b, 10), bit(b, 9), bit(b, 8));
    Insn::LoadStore {
        load: true,
        size,
        signed,
        rt,
        rn,
        offset: MemOffset::Imm(b & 0xff),
        index: p,
        add: u,
        wback: wb,
        unpriv: p && u && !wb,
    }
}

/// A5.3.12: data processing with a register operand, and the groups it
/// contains — parallel arithmetic (A5.3.13/A5.3.14) and the miscellaneous
/// operations (A5.3.15).
fn decode_32_dp_register(a: u32, b: u32) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let op1 = field(a, 7, 4);
    let op2 = field(b, 7, 4);
    if field(b, 15, 12) != 0b1111 {
        return Insn::Undefined;
    }
    if op2 == 0 && op1 & 0b1000 == 0 {
        // The register-controlled shifts.
        return Insn::ShiftReg {
            ty: ShiftType::from_bits(field(a, 6, 5)),
            s: bit(a, 4),
            rd,
            rn,
            rm,
        };
    }
    if op1 & 0b1000 == 0 && op2 & 0b1000 != 0 {
        let op = match op1 & 0b0111 {
            0b000 => ExtendOp::Sxth,
            0b001 => ExtendOp::Uxth,
            0b010 => ExtendOp::Sxtb16,
            0b011 => ExtendOp::Uxtb16,
            0b100 => ExtendOp::Sxtb,
            0b101 => ExtendOp::Uxtb,
            _ => return Insn::Undefined,
        };
        return Insn::Extend {
            op,
            rd,
            rn,
            rm,
            rotate: (field(b, 5, 4) * 8) as u8,
        };
    }
    if op1 & 0b1000 != 0 && op2 & 0b1000 == 0 {
        // Parallel addition and subtraction (A5.3.13/A5.3.14). `hw2[7]` is
        // clear for the whole group; `hw2[6]` picks the unsigned table and
        // `hw2[5:4]` the treatment within it.
        let mode = match field(b, 6, 4) {
            0b000 => SimdMode::Signed,
            0b001 => SimdMode::SignedSat,
            0b010 => SimdMode::SignedHalve,
            0b100 => SimdMode::Unsigned,
            0b101 => SimdMode::UnsignedSat,
            0b110 => SimdMode::UnsignedHalve,
            _ => return Insn::Undefined,
        };
        let shape = match field(a, 6, 4) {
            0b000 => SimdShape::Add8,
            0b001 => SimdShape::Add16,
            0b010 => SimdShape::Asx,
            0b100 => SimdShape::Sub8,
            0b101 => SimdShape::Sub16,
            0b110 => SimdShape::Sax,
            _ => return Insn::Undefined,
        };
        return Insn::Simd {
            mode,
            shape,
            rd,
            rn,
            rm,
        };
    }
    if field(a, 7, 6) == 0b10 && field(b, 7, 6) == 0b10 {
        // A5.3.15, the miscellaneous operations: `QADD`, `REV`, `SEL`,
        // `CLZ` and their neighbours, selected by `hw1[5:4]` and `hw2[5:4]`.
        return match (field(a, 5, 4), field(b, 5, 4)) {
            (0b00, 0b00) => Insn::SatQ {
                op: SatQOp::Qadd,
                rd,
                rn,
                rm,
            },
            (0b00, 0b01) => Insn::SatQ {
                op: SatQOp::Qdadd,
                rd,
                rn,
                rm,
            },
            (0b00, 0b10) => Insn::SatQ {
                op: SatQOp::Qsub,
                rd,
                rn,
                rm,
            },
            (0b00, 0b11) => Insn::SatQ {
                op: SatQOp::Qdsub,
                rd,
                rn,
                rm,
            },
            (0b01, 0b00) => Insn::Misc {
                op: MiscOp::Rev,
                rd,
                rm,
            },
            (0b01, 0b01) => Insn::Misc {
                op: MiscOp::Rev16,
                rd,
                rm,
            },
            (0b01, 0b10) => Insn::Misc {
                op: MiscOp::Rbit,
                rd,
                rm,
            },
            (0b01, 0b11) => Insn::Misc {
                op: MiscOp::Revsh,
                rd,
                rm,
            },
            (0b10, 0b00) => Insn::Sel { rd, rn, rm },
            (0b11, 0b00) => Insn::Misc {
                op: MiscOp::Clz,
                rd,
                rm,
            },
            _ => Insn::Undefined,
        };
    }
    Insn::Undefined
}

/// A5.3.16: multiply, multiply-accumulate, and absolute difference.
fn decode_32_multiply(a: u32, b: u32) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let ra = field(b, 15, 12) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let op2 = field(b, 5, 4);
    match field(a, 6, 4) {
        0b000 => match op2 {
            0b00 => Insn::Mul {
                rd,
                rn,
                rm,
                ra: if ra == 15 { None } else { Some(ra) },
                sub: false,
                s: false,
            },
            0b01 => Insn::Mul {
                rd,
                rn,
                rm,
                ra: Some(ra),
                sub: true,
                s: false,
            },
            _ => Insn::Undefined,
        },
        0b001 => Insn::HalfMul {
            op: if ra == 15 {
                HalfMulOp::Smul
            } else {
                HalfMulOp::Smla
            },
            rd,
            rn,
            rm,
            ra,
            x: bit(b, 5),
            y: bit(b, 4),
        },
        0b010 if op2 & 0b10 == 0 => Insn::DualMul {
            op: if ra == 15 {
                DualMulOp::Smuad
            } else {
                DualMulOp::Smlad
            },
            rd,
            rn,
            rm,
            ra,
            x: bit(b, 4),
        },
        0b011 if op2 & 0b10 == 0 => Insn::HalfMul {
            op: if ra == 15 {
                HalfMulOp::Smulw
            } else {
                HalfMulOp::Smlaw
            },
            rd,
            rn,
            rm,
            ra,
            x: false,
            y: bit(b, 4),
        },
        0b100 if op2 & 0b10 == 0 => Insn::DualMul {
            op: if ra == 15 {
                DualMulOp::Smusd
            } else {
                DualMulOp::Smlsd
            },
            rd,
            rn,
            rm,
            ra,
            x: bit(b, 4),
        },
        0b101 if op2 & 0b10 == 0 => Insn::DualMul {
            op: if ra == 15 {
                DualMulOp::Smmul
            } else {
                DualMulOp::Smmla
            },
            rd,
            rn,
            rm,
            ra,
            x: bit(b, 4),
        },
        0b110 if op2 & 0b10 == 0 => Insn::DualMul {
            op: DualMulOp::Smmls,
            rd,
            rn,
            rm,
            ra,
            x: bit(b, 4),
        },
        0b111 if op2 == 0 => Insn::Usad { rd, rn, rm, ra },
        _ => Insn::Undefined,
    }
}

/// A5.3.17: long multiply, long multiply-accumulate, and divide.
fn decode_32_long_multiply(a: u32, b: u32) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let rdlo = field(b, 15, 12) as u8;
    let rdhi = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let op2 = field(b, 7, 4);
    match (field(a, 6, 4), op2) {
        (0b000, 0b0000) => Insn::MulLong {
            signed: true,
            accumulate: false,
            rdlo,
            rdhi,
            rn,
            rm,
            umaal: false,
        },
        (0b001, 0b1111) => Insn::Div {
            signed: true,
            rd: rdhi,
            rn,
            rm,
        },
        (0b010, 0b0000) => Insn::MulLong {
            signed: false,
            accumulate: false,
            rdlo,
            rdhi,
            rn,
            rm,
            umaal: false,
        },
        (0b011, 0b1111) => Insn::Div {
            signed: false,
            rd: rdhi,
            rn,
            rm,
        },
        (0b100, 0b0000) => Insn::MulLong {
            signed: true,
            accumulate: true,
            rdlo,
            rdhi,
            rn,
            rm,
            umaal: false,
        },
        (0b100, 0b1000..=0b1011) => Insn::HalfMul {
            op: HalfMulOp::Smlal,
            rd: rdhi,
            rn,
            rm,
            ra: rdlo,
            x: bit(b, 5),
            y: bit(b, 4),
        },
        (0b100, 0b1100 | 0b1101) => Insn::DualMul {
            op: DualMulOp::Smlald,
            rd: rdhi,
            rn,
            rm,
            ra: rdlo,
            x: bit(b, 4),
        },
        (0b101, 0b1100 | 0b1101) => Insn::DualMul {
            op: DualMulOp::Smlsld,
            rd: rdhi,
            rn,
            rm,
            ra: rdlo,
            x: bit(b, 4),
        },
        (0b110, 0b0000) => Insn::MulLong {
            signed: false,
            accumulate: true,
            rdlo,
            rdhi,
            rn,
            rm,
            umaal: false,
        },
        (0b110, 0b0110) => Insn::MulLong {
            signed: false,
            accumulate: true,
            rdlo,
            rdhi,
            rn,
            rm,
            umaal: true,
        },
        _ => Insn::Undefined,
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

/// Formats a register list as `{r0-r3, lr}`.
struct RegList(u16);

impl fmt::Display for RegList {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("{")?;
        let mut first = true;
        let mut i = 0u8;
        while i < 16 {
            if self.0 & (1 << i) == 0 {
                i += 1;
                continue;
            }
            let start = i;
            while i < 16 && self.0 & (1 << i) != 0 {
                i += 1;
            }
            let end = i - 1;
            if !first {
                f.write_str(", ")?;
            }
            first = false;
            match end - start {
                0 => write!(f, "{}", RegName(start))?,
                1 => write!(f, "{}, {}", RegName(start), RegName(end))?,
                _ => write!(f, "{}-{}", RegName(start), RegName(end))?,
            }
        }
        f.write_str("}")
    }
}

/// The `[Rn, #imm]!` / `[Rn], #imm` shapes of an addressing mode.
struct Address<'a> {
    rn: u8,
    offset: &'a MemOffset,
    index: bool,
    add: bool,
    wback: bool,
}

impl fmt::Display for Address<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sign = if self.add { "" } else { "-" };
        let zero = matches!(*self.offset, MemOffset::Imm(0));
        if self.index {
            if zero {
                write!(f, "[{}]", RegName(self.rn))?;
            } else {
                write!(f, "[{}, {sign}{}]", RegName(self.rn), self.offset)?;
            }
            if self.wback {
                f.write_str("!")?;
            }
            Ok(())
        } else {
            write!(f, "[{}], {sign}{}", RegName(self.rn), self.offset)
        }
    }
}

impl fmt::Display for Insn {
    /// Unified assembler syntax, as DDI 0403's A7.7 pages spell it.
    #[allow(clippy::too_many_lines)] // One arm per instruction; splitting hides the table.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = |on: bool| if on { "S" } else { "" };
        match *self {
            Insn::DataProc {
                op,
                s: sf,
                rd,
                rn,
                operand,
            } => {
                // A shifted `MOV` prints as the shift it is.
                if op == DpOp::Mov
                    && let Operand::Reg { rm, shift } = operand
                    && !shift.is_none()
                {
                    if shift.ty == ShiftType::Rrx {
                        return write!(f, "RRX{} {}, {}", s(sf), RegName(rd), RegName(rm));
                    }
                    return write!(
                        f,
                        "{}{} {}, {}, #{}",
                        shift.ty,
                        s(sf),
                        RegName(rd),
                        RegName(rm),
                        shift.amount
                    );
                }
                if op.is_test() {
                    return write!(f, "{} {}, {operand}", op.mnemonic(), RegName(rn));
                }
                if op.is_unary() {
                    return write!(f, "{}{} {}, {operand}", op.mnemonic(), s(sf), RegName(rd));
                }
                write!(
                    f,
                    "{}{} {}, {}, {operand}",
                    op.mnemonic(),
                    s(sf),
                    RegName(rd),
                    RegName(rn)
                )
            }
            Insn::ShiftReg {
                ty,
                s: sf,
                rd,
                rn,
                rm,
            } => write!(
                f,
                "{ty}{} {}, {}, {}",
                s(sf),
                RegName(rd),
                RegName(rn),
                RegName(rm)
            ),
            Insn::Adr { rd, imm, add } => {
                let sign = if add { "" } else { "-" };
                write!(f, "ADR {}, {sign}#{imm}", RegName(rd))
            }
            Insn::MovImm16 { top, rd, imm } => write!(
                f,
                "{} {}, #{imm:#x}",
                if top { "MOVT" } else { "MOVW" },
                RegName(rd)
            ),
            Insn::Branch { cond, offset } => match cond {
                Some(c) => write!(f, "B{c} {offset:+}"),
                None => write!(f, "B {offset:+}"),
            },
            Insn::BranchLink { offset } => write!(f, "BL {offset:+}"),
            Insn::Bx { rm } => write!(f, "BX {}", RegName(rm)),
            Insn::Blx { rm } => write!(f, "BLX {}", RegName(rm)),
            Insn::Cbz {
                nonzero,
                rn,
                offset,
            } => write!(
                f,
                "{} {}, +{offset}",
                if nonzero { "CBNZ" } else { "CBZ" },
                RegName(rn)
            ),
            Insn::TableBranch { rn, rm, half } => {
                if half {
                    write!(f, "TBH [{}, {}, LSL #1]", RegName(rn), RegName(rm))
                } else {
                    write!(f, "TBB [{}, {}]", RegName(rn), RegName(rm))
                }
            }
            Insn::It { cond, mask } => {
                // The *lowest* set bit terminates the mask; every bit above it
                // is one more slot, spelled `T` where it matches the base
                // condition's low bit and `E` where it inverts
                // (DDI 0403 A7.7.38).
                f.write_str("IT")?;
                let last = mask.trailing_zeros();
                let mut i = 3u32;
                while i > last {
                    let same = u32::from(mask >> i) & 1 == u32::from(cond.0) & 1;
                    f.write_str(if same { "T" } else { "E" })?;
                    i -= 1;
                }
                write!(f, " {cond}")
            }
            Insn::LoadStore {
                load,
                size,
                signed,
                rt,
                rn,
                offset,
                index,
                add,
                wback,
                unpriv,
            } => write!(
                f,
                "{}{}{}{} {}, {}",
                if load { "LDR" } else { "STR" },
                if signed { "S" } else { "" },
                size.suffix(),
                if unpriv { "T" } else { "" },
                RegName(rt),
                Address {
                    rn,
                    offset: &offset,
                    index,
                    add,
                    wback
                }
            ),
            Insn::LoadLiteral {
                size,
                signed,
                rt,
                imm,
                add,
            } => write!(
                f,
                "LDR{}{} {}, [pc, {}#{imm}]",
                if signed { "S" } else { "" },
                size.suffix(),
                RegName(rt),
                if add { "" } else { "-" }
            ),
            Insn::LoadStoreDual {
                load,
                rt,
                rt2,
                rn,
                imm,
                index,
                add,
                wback,
            } => write!(
                f,
                "{}D {}, {}, {}",
                if load { "LDR" } else { "STR" },
                RegName(rt),
                RegName(rt2),
                Address {
                    rn,
                    offset: &MemOffset::Imm(imm),
                    index,
                    add,
                    wback
                }
            ),
            Insn::LoadStoreExclusive {
                load,
                size,
                rd,
                rt,
                rn,
                imm,
            } => {
                if load {
                    write!(
                        f,
                        "LDREX{} {}, [{}, #{imm}]",
                        size.suffix(),
                        RegName(rt),
                        RegName(rn)
                    )
                } else {
                    write!(
                        f,
                        "STREX{} {}, {}, [{}, #{imm}]",
                        size.suffix(),
                        RegName(rd),
                        RegName(rt),
                        RegName(rn)
                    )
                }
            }
            Insn::ClearExclusive => f.write_str("CLREX"),
            Insn::LoadStoreMultiple {
                load,
                rn,
                list,
                wback,
                before,
            } => {
                if rn == 13 && wback && load && !before {
                    return write!(f, "POP {}", RegList(list));
                }
                if rn == 13 && wback && !load && before {
                    return write!(f, "PUSH {}", RegList(list));
                }
                write!(
                    f,
                    "{}{} {}{}, {}",
                    if load { "LDM" } else { "STM" },
                    if before { "DB" } else { "IA" },
                    RegName(rn),
                    if wback { "!" } else { "" },
                    RegList(list)
                )
            }
            Insn::Mul {
                rd,
                rn,
                rm,
                ra,
                sub,
                s: sf,
            } => match ra {
                None => write!(
                    f,
                    "MUL{} {}, {}, {}",
                    s(sf),
                    RegName(rd),
                    RegName(rn),
                    RegName(rm)
                ),
                Some(ra) => write!(
                    f,
                    "{} {}, {}, {}, {}",
                    if sub { "MLS" } else { "MLA" },
                    RegName(rd),
                    RegName(rn),
                    RegName(rm),
                    RegName(ra)
                ),
            },
            Insn::MulLong {
                signed,
                accumulate,
                rdlo,
                rdhi,
                rn,
                rm,
                umaal,
            } => {
                let name = if umaal {
                    "UMAAL"
                } else {
                    match (signed, accumulate) {
                        (true, false) => "SMULL",
                        (true, true) => "SMLAL",
                        (false, false) => "UMULL",
                        (false, true) => "UMLAL",
                    }
                };
                write!(
                    f,
                    "{name} {}, {}, {}, {}",
                    RegName(rdlo),
                    RegName(rdhi),
                    RegName(rn),
                    RegName(rm)
                )
            }
            Insn::Div { signed, rd, rn, rm } => write!(
                f,
                "{} {}, {}, {}",
                if signed { "SDIV" } else { "UDIV" },
                RegName(rd),
                RegName(rn),
                RegName(rm)
            ),
            Insn::HalfMul {
                op,
                rd,
                rn,
                rm,
                ra,
                x,
                y,
            } => {
                let half = |top: bool| if top { "T" } else { "B" };
                match op {
                    HalfMulOp::Smul => write!(
                        f,
                        "SMUL{}{} {}, {}, {}",
                        half(x),
                        half(y),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    ),
                    HalfMulOp::Smla => write!(
                        f,
                        "SMLA{}{} {}, {}, {}, {}",
                        half(x),
                        half(y),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm),
                        RegName(ra)
                    ),
                    HalfMulOp::Smulw => write!(
                        f,
                        "SMULW{} {}, {}, {}",
                        half(y),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    ),
                    HalfMulOp::Smlaw => write!(
                        f,
                        "SMLAW{} {}, {}, {}, {}",
                        half(y),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm),
                        RegName(ra)
                    ),
                    HalfMulOp::Smlal => write!(
                        f,
                        "SMLAL{}{} {}, {}, {}, {}",
                        half(x),
                        half(y),
                        RegName(ra),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    ),
                }
            }
            Insn::DualMul {
                op,
                rd,
                rn,
                rm,
                ra,
                x,
            } => {
                let suffix = match (x, op.bit_is_round()) {
                    (false, _) => "",
                    (true, false) => "X",
                    (true, true) => "R",
                };
                match op {
                    DualMulOp::Smuad | DualMulOp::Smusd | DualMulOp::Smmul => write!(
                        f,
                        "{}{suffix} {}, {}, {}",
                        op.mnemonic(),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    ),
                    DualMulOp::Smlald | DualMulOp::Smlsld => write!(
                        f,
                        "{}{suffix} {}, {}, {}, {}",
                        op.mnemonic(),
                        RegName(ra),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    ),
                    _ => write!(
                        f,
                        "{}{suffix} {}, {}, {}, {}",
                        op.mnemonic(),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm),
                        RegName(ra)
                    ),
                }
            }
            Insn::Sat {
                unsigned,
                halves,
                rd,
                rn,
                imm,
                shift,
            } => write!(
                f,
                "{}SAT{} {}, #{imm}, {}{shift}",
                if unsigned { "U" } else { "S" },
                if halves { "16" } else { "" },
                RegName(rd),
                RegName(rn)
            ),
            // The assembler syntax is `QADD Rd, Rm, Rn` — the doubled
            // operand of the `QD` forms is `Rn`, and it comes last.
            Insn::SatQ { op, rd, rn, rm } => write!(
                f,
                "{} {}, {}, {}",
                op.mnemonic(),
                RegName(rd),
                RegName(rm),
                RegName(rn)
            ),
            Insn::Simd {
                mode,
                shape,
                rd,
                rn,
                rm,
            } => write!(
                f,
                "{}{} {}, {}, {}",
                mode.prefix(),
                shape.mnemonic(),
                RegName(rd),
                RegName(rn),
                RegName(rm)
            ),
            Insn::Sel { rd, rn, rm } => {
                write!(f, "SEL {}, {}, {}", RegName(rd), RegName(rn), RegName(rm))
            }
            Insn::Usad { rd, rn, rm, ra } => {
                if ra == 15 {
                    write!(f, "USAD8 {}, {}, {}", RegName(rd), RegName(rn), RegName(rm))
                } else {
                    write!(
                        f,
                        "USADA8 {}, {}, {}, {}",
                        RegName(rd),
                        RegName(rn),
                        RegName(rm),
                        RegName(ra)
                    )
                }
            }
            Insn::Pkh {
                tb,
                rd,
                rn,
                rm,
                shift,
            } => write!(
                f,
                "PKH{} {}, {}, {}{shift}",
                if tb { "TB" } else { "BT" },
                RegName(rd),
                RegName(rn),
                RegName(rm)
            ),
            Insn::Extend {
                op,
                rd,
                rn,
                rm,
                rotate,
            } => {
                let rot = if rotate == 0 {
                    Rotate(None)
                } else {
                    Rotate(Some(rotate))
                };
                if rn == 15 {
                    write!(f, "{} {}, {}{rot}", op.mnemonic(), RegName(rd), RegName(rm))
                } else {
                    write!(
                        f,
                        "{} {}, {}, {}{rot}",
                        op.accumulating_mnemonic(),
                        RegName(rd),
                        RegName(rn),
                        RegName(rm)
                    )
                }
            }
            Insn::Misc { op, rd, rm } => {
                write!(f, "{} {}, {}", op.mnemonic(), RegName(rd), RegName(rm))
            }
            Insn::Bitfield {
                op,
                rd,
                rn,
                lsb,
                width,
            } => match op {
                BitfieldOp::Sbfx => {
                    write!(f, "SBFX {}, {}, #{lsb}, #{width}", RegName(rd), RegName(rn))
                }
                BitfieldOp::Ubfx => {
                    write!(f, "UBFX {}, {}, #{lsb}, #{width}", RegName(rd), RegName(rn))
                }
                BitfieldOp::Bfi => {
                    write!(f, "BFI {}, {}, #{lsb}, #{width}", RegName(rd), RegName(rn))
                }
                BitfieldOp::Bfc => write!(f, "BFC {}, #{lsb}, #{width}", RegName(rd)),
            },
            Insn::Mrs { rd, sysm } => {
                write!(f, "MRS {}, {}", RegName(rd), SysReg(sysm))
            }
            Insn::Msr { rn, sysm, .. } => {
                write!(f, "MSR {}, {}", SysReg(sysm), RegName(rn))
            }
            Insn::Cps { enable, i, f: ff } => write!(
                f,
                "CPSI{} {}{}",
                if enable { "E" } else { "D" },
                if i { "i" } else { "" },
                if ff { "f" } else { "" }
            ),
            Insn::Barrier { op, option } => write!(f, "{} #{option}", op.mnemonic()),
            Insn::Hint { op } => match op {
                HintOp::Dbg(n) => write!(f, "DBG #{n}"),
                other => f.write_str(other.mnemonic()),
            },
            Insn::Bkpt { imm } => write!(f, "BKPT #{imm}"),
            Insn::Svc { imm } => write!(f, "SVC #{imm}"),
            Insn::Udf { imm } => write!(f, "UDF #{imm}"),
            Insn::Coproc { cp } => write!(f, "<coproc p{cp}>"),
            Insn::Undefined => f.write_str("UNDEFINED"),
        }
    }
}

/// The `, ROR #8` an extend prints when it rotates.
struct Rotate(Option<u8>);

impl fmt::Display for Rotate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            None => Ok(()),
            Some(n) => write!(f, ", ROR #{n}"),
        }
    }
}

/// A special register named by an `MRS`/`MSR` `SYSm` field (DDI 0403 B5.1.1).
#[derive(Debug, Clone, Copy)]
pub struct SysReg(pub u8);

impl fmt::Display for SysReg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self.0 {
            0 => "APSR",
            1 => "IAPSR",
            2 => "EAPSR",
            3 => "XPSR",
            5 => "IPSR",
            6 => "EPSR",
            7 => "IEPSR",
            8 => "MSP",
            9 => "PSP",
            16 => "PRIMASK",
            17 => "BASEPRI",
            18 => "BASEPRI_MAX",
            19 => "FAULTMASK",
            20 => "CONTROL",
            _ => return write!(f, "SYSm#{}", self.0),
        })
    }
}
