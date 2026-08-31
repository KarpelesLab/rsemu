//! The ARM instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). ARM
//! is not a 256-entry opcode matrix like a 6502, so the "one table" here is a
//! **single decoder that produces a fully semantic [`Insn`]**. Two consumers
//! read it and nothing else:
//!
//! - the interpreter (`super::exec`), which matches on the variant and its
//!   named fields rather than re-extracting bit ranges;
//! - the disassembler ([`Decoded`]'s `Display`, and [`super::disasm`] on top
//!   of it).
//!
//! A field the interpreter reads is therefore a field the disassembler prints,
//! and an encoding either exists in both or in neither.
//!
//! # Why there is no cycle column
//!
//! Same reason as the 6502 core: the cost of an instruction is the cost of the
//! accesses it makes. `LDM` costs one cycle per register in its list, which is
//! a property of the operand; a register-controlled shift costs an internal
//! cycle, which is a property of the addressing mode. See `super::exec` for
//! the timing model that falls out of that.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual* (ARM DDI 0100, the "ARM ARM"), the
//! ARMv5 revisions:
//!
//! - A3.1–A3.15, the ARM instruction-set encoding tables — every discriminator
//!   in [`decode`] comes from these, including A3.4's "Miscellaneous
//!   instructions" and A3.5's "Multiplies and extra load/stores".
//! - A5.1, addressing mode 1 (the barrel shifter); A5.2 mode 2 (word and
//!   unsigned byte); A5.3 mode 3 (halfword, signed byte, doubleword); A5.4
//!   mode 4 (load and store multiple); A5.5 mode 5 (coprocessor).
//! - A4.1, the alphabetical instruction list, for each operation's operand
//!   fields and assembler syntax.
//! - A10, the DSP (E) extensions: `QADD`, `SMLA<x><y>` and relatives.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

/// Extract bits `hi..=lo` of `word`.
///
/// Written to stay correct at `hi == 31, lo == 0`, where the obvious
/// `(1 << (hi - lo + 1)) - 1` overflows.
#[inline]
#[must_use]
pub const fn field(word: u32, hi: u32, lo: u32) -> u32 {
    (word >> lo) & (u32::MAX >> (31 - (hi - lo)))
}

/// Whether bit `n` of `word` is set.
#[inline]
#[must_use]
pub const fn bit(word: u32, n: u32) -> bool {
    word & (1 << n) != 0
}

// ---------------------------------------------------------------------------
// Condition codes
// ---------------------------------------------------------------------------

/// The four-bit condition field every ARM instruction carries (ARM ARM A3.2).
///
/// A `#[repr(transparent)]` newtype with `pub const` variants rather than an
/// enum: the field is four bits wide and every one of the sixteen values is
/// meaningful, so exhaustiveness buys nothing and the round trip through a raw
/// encoding stays free (CLAUDE.md, "Type conventions").
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Cond(pub u8);

impl Cond {
    /// Equal — `Z` set.
    pub const EQ: Cond = Cond(0x0);
    /// Not equal — `Z` clear.
    pub const NE: Cond = Cond(0x1);
    /// Unsigned higher or same — `C` set. Also spelled `HS`.
    pub const CS: Cond = Cond(0x2);
    /// Unsigned lower — `C` clear. Also spelled `LO`.
    pub const CC: Cond = Cond(0x3);
    /// Negative — `N` set.
    pub const MI: Cond = Cond(0x4);
    /// Positive or zero — `N` clear.
    pub const PL: Cond = Cond(0x5);
    /// Overflow — `V` set.
    pub const VS: Cond = Cond(0x6);
    /// No overflow — `V` clear.
    pub const VC: Cond = Cond(0x7);
    /// Unsigned higher.
    pub const HI: Cond = Cond(0x8);
    /// Unsigned lower or same.
    pub const LS: Cond = Cond(0x9);
    /// Signed greater than or equal.
    pub const GE: Cond = Cond(0xa);
    /// Signed less than.
    pub const LT: Cond = Cond(0xb);
    /// Signed greater than.
    pub const GT: Cond = Cond(0xc);
    /// Signed less than or equal.
    pub const LE: Cond = Cond(0xd);
    /// Always.
    pub const AL: Cond = Cond(0xe);
    /// The encoding that used to mean "never".
    ///
    /// ARMv5 reclaimed it: `0b1111` is no longer a condition at all but a
    /// separate instruction space holding `BLX` (immediate), `PLD` and the
    /// `xxx2` coprocessor forms (ARM ARM A3.1). [`decode`] routes it there and
    /// never asks [`Cond::passes`] about it.
    pub const NV: Cond = Cond(0xf);

    /// The suffix an assembler writes, empty for [`Cond::AL`].
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 & 0xf {
            0x0 => "EQ",
            0x1 => "NE",
            0x2 => "CS",
            0x3 => "CC",
            0x4 => "MI",
            0x5 => "PL",
            0x6 => "VS",
            0x7 => "VC",
            0x8 => "HI",
            0x9 => "LS",
            0xa => "GE",
            0xb => "LT",
            0xc => "GT",
            0xd => "LE",
            0xe => "",
            _ => "NV",
        }
    }

    /// Whether this condition holds for the given PSR (ARM ARM A3.2).
    ///
    /// `psr` is a whole CPSR; only `N`, `Z`, `C` and `V` are looked at.
    /// [`Cond::NV`] reports `false` — it is not a condition in ARMv5, and any
    /// caller reaching it with a `0b1111` encoding has skipped [`decode`]'s
    /// unconditional space.
    #[must_use]
    pub const fn passes(self, psr: u32) -> bool {
        let n = bit(psr, 31);
        let z = bit(psr, 30);
        let c = bit(psr, 29);
        let v = bit(psr, 28);
        match self.0 & 0xf {
            0x0 => z,
            0x1 => !z,
            0x2 => c,
            0x3 => !c,
            0x4 => n,
            0x5 => !n,
            0x6 => v,
            0x7 => !v,
            0x8 => c && !z,
            0x9 => !c || z,
            0xa => n == v,
            0xb => n != v,
            0xc => !z && n == v,
            0xd => z || n != v,
            0xe => true,
            _ => false,
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// Addressing mode 1: the barrel shifter
// ---------------------------------------------------------------------------

/// One of the barrel shifter's four operations (ARM ARM A5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShiftType {
    /// Logical shift left.
    Lsl,
    /// Logical shift right.
    Lsr,
    /// Arithmetic shift right.
    Asr,
    /// Rotate right; with a zero immediate it encodes `RRX` instead.
    Ror,
}

impl ShiftType {
    /// Decode the two-bit `shift` field.
    #[must_use]
    pub const fn from_bits(bits: u32) -> ShiftType {
        match bits & 0b11 {
            0 => ShiftType::Lsl,
            1 => ShiftType::Lsr,
            2 => ShiftType::Asr,
            _ => ShiftType::Ror,
        }
    }

    /// The assembler mnemonic.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            ShiftType::Lsl => "LSL",
            ShiftType::Lsr => "LSR",
            ShiftType::Asr => "ASR",
            ShiftType::Ror => "ROR",
        }
    }
}

impl fmt::Display for ShiftType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How much a shifted-register operand is shifted by.
///
/// The raw encoding is kept rather than pre-resolved, because the special
/// cases are not cosmetic: `LSR #0` *means* `LSR #32`, `ASR #0` means
/// `ASR #32`, and `ROR #0` is `RRX` — three different operations that all
/// encode `amount == 0` (ARM ARM A5.1.7, A5.1.9, A5.1.11). Resolving at decode
/// time would lose which one was written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Shift {
    /// Shift by a five-bit immediate, with the zero special cases above.
    Imm {
        /// Which shift.
        ty: ShiftType,
        /// The raw `imm5` field, `0..=31`.
        amount: u8,
    },
    /// Shift by the low byte of a register.
    ///
    /// Costs the instruction one internal cycle, and makes reads of `R15`
    /// yield the instruction's address plus twelve rather than plus eight.
    Reg {
        /// Which shift.
        ty: ShiftType,
        /// The register holding the amount; only its low eight bits are used.
        rs: u8,
    },
}

impl Shift {
    /// Whether this is the encoding that means "no shift at all", `LSL #0`.
    ///
    /// The disassembler omits it; the interpreter still has to know, because
    /// `LSL #0` passes the carry flag through unchanged where every other
    /// shift computes one.
    #[must_use]
    pub const fn is_none(self) -> bool {
        matches!(
            self,
            Shift::Imm {
                ty: ShiftType::Lsl,
                amount: 0
            }
        )
    }
}

impl fmt::Display for Shift {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Shift::Imm {
                ty: ShiftType::Ror,
                amount: 0,
            } => f.write_str("RRX"),
            Shift::Imm { ty, amount: 0 } => write!(f, "{ty} #32"),
            Shift::Imm { ty, amount } => write!(f, "{ty} #{amount}"),
            Shift::Reg { ty, rs } => write!(f, "{ty} {}", RegName(rs)),
        }
    }
}

/// A data-processing instruction's second operand (ARM ARM A5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operand {
    /// An eight-bit value rotated right by twice a four-bit field.
    ///
    /// Both halves are kept because the shifter carry-out depends on the
    /// *rotate* rather than on the value: a rotate of zero leaves `C` alone,
    /// any other rotate sets it from bit 31 of the result (ARM ARM A5.1.3).
    Imm {
        /// The eight-bit immediate before rotation.
        imm8: u8,
        /// Half the rotate amount, `0..=15`.
        rotate: u8,
    },
    /// A register, optionally shifted.
    Reg {
        /// The register.
        rm: u8,
        /// How it is shifted.
        shift: Shift,
    },
}

impl Operand {
    /// The value of an immediate operand, or `None` for a register one.
    #[must_use]
    pub const fn immediate(self) -> Option<u32> {
        match self {
            Operand::Imm { imm8, rotate } => Some((imm8 as u32).rotate_right((rotate as u32) * 2)),
            Operand::Reg { .. } => None,
        }
    }

    /// Whether evaluating this operand needs a register-controlled shift.
    #[must_use]
    pub const fn is_register_shifted(self) -> bool {
        matches!(
            self,
            Operand::Reg {
                shift: Shift::Reg { .. },
                ..
            }
        )
    }
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Operand::Imm { .. } => {
                let v = self.immediate().unwrap_or(0);
                write!(f, "#{v}")
            }
            Operand::Reg { rm, shift } if shift.is_none() => write!(f, "{}", RegName(rm)),
            Operand::Reg { rm, shift } => write!(f, "{}, {shift}", RegName(rm)),
        }
    }
}

/// Formats a register number the way an ARM assembler spells it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
// Data-processing operations
// ---------------------------------------------------------------------------

/// The sixteen data-processing operations (ARM ARM A3.4, `opcode` field).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DpOp {
    /// Bitwise and.
    And,
    /// Bitwise exclusive or.
    Eor,
    /// Subtract.
    Sub,
    /// Reverse subtract: operand minus `Rn`.
    Rsb,
    /// Add.
    Add,
    /// Add with carry.
    Adc,
    /// Subtract with borrow.
    Sbc,
    /// Reverse subtract with borrow.
    Rsc,
    /// Test: `And` discarding the result.
    Tst,
    /// Test equivalence: `Eor` discarding the result.
    Teq,
    /// Compare: `Sub` discarding the result.
    Cmp,
    /// Compare negative: `Add` discarding the result.
    Cmn,
    /// Bitwise or.
    Orr,
    /// Move.
    Mov,
    /// Bit clear: `Rn AND NOT operand`.
    Bic,
    /// Move not.
    Mvn,
}

impl DpOp {
    /// Decode the four-bit `opcode` field.
    #[must_use]
    pub const fn from_bits(bits: u32) -> DpOp {
        match bits & 0xf {
            0x0 => DpOp::And,
            0x1 => DpOp::Eor,
            0x2 => DpOp::Sub,
            0x3 => DpOp::Rsb,
            0x4 => DpOp::Add,
            0x5 => DpOp::Adc,
            0x6 => DpOp::Sbc,
            0x7 => DpOp::Rsc,
            0x8 => DpOp::Tst,
            0x9 => DpOp::Teq,
            0xa => DpOp::Cmp,
            0xb => DpOp::Cmn,
            0xc => DpOp::Orr,
            0xd => DpOp::Mov,
            0xe => DpOp::Bic,
            _ => DpOp::Mvn,
        }
    }

    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            DpOp::And => "AND",
            DpOp::Eor => "EOR",
            DpOp::Sub => "SUB",
            DpOp::Rsb => "RSB",
            DpOp::Add => "ADD",
            DpOp::Adc => "ADC",
            DpOp::Sbc => "SBC",
            DpOp::Rsc => "RSC",
            DpOp::Tst => "TST",
            DpOp::Teq => "TEQ",
            DpOp::Cmp => "CMP",
            DpOp::Cmn => "CMN",
            DpOp::Orr => "ORR",
            DpOp::Mov => "MOV",
            DpOp::Bic => "BIC",
            DpOp::Mvn => "MVN",
        }
    }

    /// Whether the operation writes `Rd`.
    ///
    /// False for the four compare forms, which exist only for their flags and
    /// always encode `S`.
    #[must_use]
    pub const fn writes_result(self) -> bool {
        !matches!(self, DpOp::Tst | DpOp::Teq | DpOp::Cmp | DpOp::Cmn)
    }

    /// Whether the operation reads `Rn`.
    ///
    /// False for `MOV` and `MVN`, whose `Rn` field is not used.
    #[must_use]
    pub const fn reads_rn(self) -> bool {
        !matches!(self, DpOp::Mov | DpOp::Mvn)
    }

    /// Whether `S` takes `C` from the barrel shifter rather than from the ALU.
    ///
    /// The logical operations pass the shifter's carry-out through and leave
    /// `V` alone; the arithmetic ones compute both from the adder (ARM ARM
    /// A4.1.4 versus A4.1.3).
    #[must_use]
    pub const fn is_logical(self) -> bool {
        matches!(
            self,
            DpOp::And
                | DpOp::Eor
                | DpOp::Tst
                | DpOp::Teq
                | DpOp::Orr
                | DpOp::Mov
                | DpOp::Bic
                | DpOp::Mvn
        )
    }
}

impl fmt::Display for DpOp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

// ---------------------------------------------------------------------------
// Addressing modes 2 and 3
// ---------------------------------------------------------------------------

/// Whether the offset is applied before or after the base is used, and what
/// happens to the base (ARM ARM A5.2, A5.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Index {
    /// `[Rn, offset]`, optionally `!` to write the new base back.
    Pre {
        /// Whether `Rn` is updated.
        writeback: bool,
    },
    /// `[Rn], offset` — the base is used unchanged and always updated after.
    Post {
        /// The `W` bit, which in a post-indexed word or byte access means
        /// "make this access as if unprivileged": `LDRT`, `STRT`, `LDRBT`,
        /// `STRBT` (ARM ARM A4.1.24). Never set for addressing mode 3.
        unprivileged: bool,
    },
}

impl Index {
    /// Whether the base register is updated by this access.
    #[must_use]
    pub const fn writes_base(self) -> bool {
        match self {
            Index::Pre { writeback } => writeback,
            Index::Post { .. } => true,
        }
    }
}

/// The offset half of addressing modes 2 and 3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Offset {
    /// An unsigned immediate — twelve bits in mode 2, eight in mode 3.
    Imm(u16),
    /// A register, shifted by an immediate in mode 2 and never in mode 3.
    Reg {
        /// The offset register.
        rm: u8,
        /// How it is shifted. Always `LSL #0` in addressing mode 3.
        shift: Shift,
    },
}

impl Offset {
    /// Whether this offset contributes nothing — `#0`, the bare `[Rn]` form.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        matches!(self, Offset::Imm(0))
    }
}

/// Formats one addressing-mode-2/3 operand: `[r1, #-4]!`, `[r1], r2, LSL #2`.
struct Addressing {
    rn: u8,
    up: bool,
    index: Index,
    offset: Offset,
}

impl fmt::Display for Addressing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sign = if self.up { "" } else { "-" };
        let base = RegName(self.rn);
        match self.index {
            // `[Rn]` and `[Rn, #0]` are the same access; the shorter spelling
            // is what an assembler writes and what a reader expects.
            Index::Pre { writeback } if self.offset.is_zero() && !writeback => {
                write!(f, "[{base}]")
            }
            Index::Pre { writeback } => {
                write!(f, "[{base}, ")?;
                write_offset(f, self.offset, sign)?;
                f.write_str("]")?;
                if writeback {
                    f.write_str("!")?;
                }
                Ok(())
            }
            Index::Post { .. } => {
                write!(f, "[{base}], ")?;
                write_offset(f, self.offset, sign)
            }
        }
    }
}

fn write_offset(f: &mut fmt::Formatter<'_>, offset: Offset, sign: &str) -> fmt::Result {
    match offset {
        Offset::Imm(imm) => write!(f, "#{sign}{imm}"),
        Offset::Reg { rm, shift } if shift.is_none() => write!(f, "{sign}{}", RegName(rm)),
        Offset::Reg { rm, shift } => write!(f, "{sign}{}, {shift}", RegName(rm)),
    }
}

/// Which of addressing mode 3's six operations this is (ARM ARM A5.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExtraOp {
    /// Store halfword.
    Strh,
    /// Load unsigned halfword.
    Ldrh,
    /// Load signed byte.
    Ldrsb,
    /// Load signed halfword.
    Ldrsh,
    /// Load doubleword into `Rd` and `Rd + 1` (ARMv5TE).
    Ldrd,
    /// Store doubleword from `Rd` and `Rd + 1` (ARMv5TE).
    Strd,
}

impl ExtraOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            ExtraOp::Strh => "STRH",
            ExtraOp::Ldrh => "LDRH",
            ExtraOp::Ldrsb => "LDRSB",
            ExtraOp::Ldrsh => "LDRSH",
            ExtraOp::Ldrd => "LDRD",
            ExtraOp::Strd => "STRD",
        }
    }

    /// Whether the operation reads memory.
    #[must_use]
    pub const fn is_load(self) -> bool {
        matches!(
            self,
            ExtraOp::Ldrh | ExtraOp::Ldrsb | ExtraOp::Ldrsh | ExtraOp::Ldrd
        )
    }
}

// ---------------------------------------------------------------------------
// Saturating and half-word multiply operations (the E extensions)
// ---------------------------------------------------------------------------

/// The four saturating add/subtract operations (ARM ARM A4.1.26–A4.1.29).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SatOp {
    /// `Rd = SAT(Rm + Rn)`.
    QAdd,
    /// `Rd = SAT(Rm - Rn)`.
    QSub,
    /// `Rd = SAT(Rm + SAT(Rn * 2))`.
    QDAdd,
    /// `Rd = SAT(Rm - SAT(Rn * 2))`.
    QDSub,
}

impl SatOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            SatOp::QAdd => "QADD",
            SatOp::QSub => "QSUB",
            SatOp::QDAdd => "QDADD",
            SatOp::QDSub => "QDSUB",
        }
    }
}

/// Which half of a register a signed-multiply operand comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Half {
    /// Bits 15..0.
    Bottom,
    /// Bits 31..16.
    Top,
}

impl Half {
    /// `B` or `T`, as the mnemonic suffix spells it.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            Half::Bottom => "B",
            Half::Top => "T",
        }
    }
}

/// The half-word multiply family of the DSP extensions (ARM ARM A10).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HalfMulOp {
    /// `SMLA<x><y>`: `Rd = Rm.<x> * Rs.<y> + Rn`, sticky `Q` on overflow.
    Smla,
    /// `SMLAW<y>`: `Rd = ((Rm * Rs.<y>) >> 16) + Rn`.
    Smlaw,
    /// `SMULW<y>`: `Rd = (Rm * Rs.<y>) >> 16`.
    Smulw,
    /// `SMLAL<x><y>`: `RdHi:RdLo += Rm.<x> * Rs.<y>`.
    Smlal,
    /// `SMUL<x><y>`: `Rd = Rm.<x> * Rs.<y>`.
    Smul,
}

// ---------------------------------------------------------------------------
// The decoded instruction
// ---------------------------------------------------------------------------

/// One ARM instruction, decoded into named fields.
///
/// This is the *whole* description: the interpreter never re-reads the raw
/// word, and neither does the disassembler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Insn {
    /// Data processing (ARM ARM A3.4).
    DataProc {
        /// Which operation.
        op: DpOp,
        /// Whether the flags are updated. Writing `R15` with `S` set also
        /// restores `CPSR` from `SPSR`, which is how an exception returns.
        s: bool,
        /// Destination register.
        rd: u8,
        /// First operand register; ignored by `MOV` and `MVN`.
        rn: u8,
        /// Second operand, through the barrel shifter.
        operand: Operand,
    },
    /// `MRS` — move the `CPSR` or `SPSR` into a general register.
    Mrs {
        /// Destination register.
        rd: u8,
        /// Read `SPSR` rather than `CPSR`.
        spsr: bool,
    },
    /// `MSR` — write selected fields of the `CPSR` or `SPSR`.
    Msr {
        /// Write `SPSR` rather than `CPSR`.
        spsr: bool,
        /// The four-bit field mask: control, extension, status, flags.
        mask: u8,
        /// The value, immediate or register (the register form never shifts).
        operand: Operand,
    },
    /// `BX` — branch and exchange instruction set.
    Bx {
        /// Register holding the target; bit 0 selects Thumb.
        rm: u8,
    },
    /// `BLX` (register form) — branch with link and exchange.
    BlxReg {
        /// Register holding the target; bit 0 selects Thumb.
        rm: u8,
    },
    /// `CLZ` — count leading zeros (ARMv5).
    Clz {
        /// Destination.
        rd: u8,
        /// Source.
        rm: u8,
    },
    /// `BKPT` — software breakpoint (ARMv5).
    Bkpt {
        /// The sixteen-bit comment field, for whatever is debugging.
        imm: u16,
    },
    /// `B` and `BL`.
    Branch {
        /// Whether `R14` is set to the return address.
        link: bool,
        /// Sign-extended byte offset from the instruction's address plus
        /// eight.
        offset: i32,
    },
    /// `BLX` (immediate form) — always changes to Thumb state (ARMv5).
    BlxImm {
        /// Sign-extended byte offset from the instruction's address plus
        /// eight, already including the extra halfword from the `H` bit.
        offset: i32,
    },
    /// `MUL` and `MLA`.
    Mul {
        /// Add `Rn` to the product.
        accumulate: bool,
        /// Set `N` and `Z` from the result.
        s: bool,
        /// Destination.
        rd: u8,
        /// Addend, for `MLA`.
        rn: u8,
        /// First multiplicand.
        rm: u8,
        /// Second multiplicand.
        rs: u8,
    },
    /// `UMULL`, `UMLAL`, `SMULL`, `SMLAL`.
    MulLong {
        /// Treat the operands as signed.
        signed: bool,
        /// Add the existing `RdHi:RdLo` to the product.
        accumulate: bool,
        /// Set `N` and `Z` from the 64-bit result.
        s: bool,
        /// High half of the destination pair.
        rdhi: u8,
        /// Low half of the destination pair.
        rdlo: u8,
        /// First multiplicand.
        rm: u8,
        /// Second multiplicand.
        rs: u8,
    },
    /// The saturating add and subtract family (ARMv5TE).
    Saturating {
        /// Which operation.
        op: SatOp,
        /// Destination.
        rd: u8,
        /// The operand that is not doubled.
        rm: u8,
        /// The operand that `QDADD`/`QDSUB` double.
        rn: u8,
    },
    /// The signed half-word multiply family (ARMv5TE).
    HalfMul {
        /// Which operation.
        op: HalfMulOp,
        /// Destination, or the high half of the pair for `SMLAL<x><y>`.
        rd: u8,
        /// Accumulator, or the low half of the pair for `SMLAL<x><y>`.
        rn: u8,
        /// First multiplicand.
        rm: u8,
        /// Second multiplicand.
        rs: u8,
        /// Which half of `Rm` is used; ignored by the `W` forms.
        x: Half,
        /// Which half of `Rs` is used.
        y: Half,
    },
    /// `LDR`, `STR`, `LDRB`, `STRB` and their `T` variants (addressing mode 2).
    LoadStore {
        /// Whether this reads memory.
        load: bool,
        /// Whether the access is one byte rather than one word.
        byte: bool,
        /// Add the offset rather than subtract it.
        up: bool,
        /// Pre- or post-indexed.
        index: Index,
        /// Base register.
        rn: u8,
        /// Data register.
        rd: u8,
        /// The offset.
        offset: Offset,
    },
    /// Addressing mode 3: halfword, signed byte and doubleword transfers.
    LoadStoreExtra {
        /// Which operation.
        op: ExtraOp,
        /// Add the offset rather than subtract it.
        up: bool,
        /// Pre- or post-indexed.
        index: Index,
        /// Base register.
        rn: u8,
        /// Data register; the doubleword forms also use `rd + 1`.
        rd: u8,
        /// The offset.
        offset: Offset,
    },
    /// `LDM` and `STM` (addressing mode 4).
    BlockTransfer {
        /// Whether this reads memory.
        load: bool,
        /// Adjust the address before the first transfer rather than after the
        /// last.
        before: bool,
        /// Ascending rather than descending.
        up: bool,
        /// The `S` bit: transfer the user-mode bank, or — for an `LDM` whose
        /// list contains `R15` — restore `CPSR` from `SPSR`.
        user: bool,
        /// Update the base register.
        writeback: bool,
        /// Base register.
        rn: u8,
        /// One bit per register, `R0` in bit 0.
        list: u16,
    },
    /// `SWP` and `SWPB`.
    Swap {
        /// Swap one byte rather than one word.
        byte: bool,
        /// Destination.
        rd: u8,
        /// Address register.
        rn: u8,
        /// Source of the value written.
        rm: u8,
    },
    /// `SWI` — software interrupt.
    Swi {
        /// The 24-bit comment field. The handler reads it back out of memory;
        /// nothing in hardware looks at it.
        imm: u32,
    },
    /// `PLD` — preload hint (ARMv5TE), architecturally without side effects
    /// beyond a possible cache fill.
    Pld {
        /// Base register.
        rn: u8,
        /// Add the offset rather than subtract it.
        up: bool,
        /// The offset.
        offset: Offset,
    },
    /// `CDP` / `CDP2` — coprocessor data operation.
    Cdp {
        /// Coprocessor number.
        cp: u8,
        /// First opcode field.
        opc1: u8,
        /// Destination coprocessor register.
        crd: u8,
        /// First operand coprocessor register.
        crn: u8,
        /// Second operand coprocessor register.
        crm: u8,
        /// Second opcode field.
        opc2: u8,
        /// Whether this is the unconditional `CDP2` encoding.
        second: bool,
    },
    /// `MCR` / `MRC` and their `2` forms — move between an ARM register and a
    /// coprocessor register.
    CpReg {
        /// Coprocessor number.
        cp: u8,
        /// Read from the coprocessor (`MRC`) rather than write to it (`MCR`).
        load: bool,
        /// First opcode field.
        opc1: u8,
        /// The ARM register.
        rd: u8,
        /// First coprocessor register.
        crn: u8,
        /// Second coprocessor register.
        crm: u8,
        /// Second opcode field.
        opc2: u8,
        /// Whether this is the unconditional `MCR2`/`MRC2` encoding.
        second: bool,
    },
    /// `MCRR` / `MRRC` — move a register *pair* to or from a coprocessor
    /// (ARMv5TE).
    CpRegPair {
        /// Coprocessor number.
        cp: u8,
        /// Read from the coprocessor (`MRRC`).
        load: bool,
        /// The opcode field.
        opc: u8,
        /// First ARM register.
        rd: u8,
        /// Second ARM register.
        rn: u8,
        /// Coprocessor register.
        crm: u8,
    },
    /// `LDC` / `STC` and their `2` forms — coprocessor load and store.
    CpTransfer {
        /// Coprocessor number.
        cp: u8,
        /// Read from memory (`LDC`).
        load: bool,
        /// The `N` bit, a coprocessor-defined "long" flag.
        long: bool,
        /// Coprocessor register.
        crd: u8,
        /// Base register.
        rn: u8,
        /// Pre- or post-indexed.
        index: Index,
        /// Add the offset rather than subtract it.
        up: bool,
        /// Word offset; the byte offset is four times this.
        offset: u8,
        /// Whether this is the unconditional `LDC2`/`STC2` encoding.
        second: bool,
    },
    /// An encoding this architecture does not define.
    ///
    /// Taken as an Undefined Instruction exception, which is what the
    /// architecture requires — never silently skipped.
    Undefined,
}

/// A decoded instruction together with its condition and its raw encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Decoded {
    /// The raw 32-bit word, kept for tracing and for `Undefined` reporting.
    pub raw: u32,
    /// The condition field. Always [`Cond::AL`] for the unconditional
    /// encodings, which are unconditional by definition.
    pub cond: Cond,
    /// What the instruction does.
    pub insn: Insn,
}

impl Decoded {
    /// Whether the instruction executes, given a PSR's flags.
    #[must_use]
    pub const fn passes(&self, psr: u32) -> bool {
        self.cond.passes(psr)
    }

    /// Whether this encoding is architecturally undefined.
    #[must_use]
    pub const fn is_undefined(&self) -> bool {
        matches!(self.insn, Insn::Undefined)
    }
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// Decode one ARM instruction word.
///
/// Never fails: an encoding with no meaning decodes to [`Insn::Undefined`],
/// which the interpreter turns into an Undefined Instruction exception.
///
/// The discriminators follow ARM ARM A3.1's top-level table and the two
/// sub-tables it refers to, A3.4 ("Miscellaneous instructions") and A3.5
/// ("Multiplies and extra load/store instructions"), in that order — the order
/// matters, because the misc and multiply spaces are carved *out of* the
/// data-processing space rather than sitting beside it.
#[must_use]
pub fn decode(raw: u32) -> Decoded {
    let cond = Cond(field(raw, 31, 28) as u8);
    if cond == Cond::NV {
        // ARMv5 reclaimed `0b1111` as a separate, unconditional space.
        return Decoded {
            raw,
            cond: Cond::AL,
            insn: decode_unconditional(raw),
        };
    }
    Decoded {
        raw,
        cond,
        insn: decode_conditional(raw),
    }
}

/// The `cond == 0b1111` space (ARM ARM A3.1, "Unconditional instructions").
fn decode_unconditional(raw: u32) -> Insn {
    match field(raw, 27, 25) {
        // 1111 101H <offset24>: BLX to a Thumb routine. H supplies bit 1 of
        // the target, so the reachable granularity is a halfword.
        0b101 => {
            let imm24 = field(raw, 23, 0);
            // Sign-extend the 24-bit field and scale by four in one shift
            // pair, so the sign survives the widening.
            let signed = ((imm24 << 8) as i32) >> 6;
            let h = if bit(raw, 24) { 2 } else { 0 };
            Insn::BlxImm { offset: signed | h }
        }
        // 1111 01x1 x101 ...: PLD, the one memory hint ARMv5TE defines. It is
        // an addressing-mode-2 load with P = 1, W = 0, Rd = 0b1111.
        0b010 | 0b011 if bit(raw, 24) && bit(raw, 22) && bit(raw, 20) && !bit(raw, 21) => {
            decode_pld(raw)
        }
        // The `2` coprocessor forms exist unconditionally in ARMv5.
        0b110 => decode_coprocessor_transfer(raw, true),
        0b111 if !bit(raw, 24) => decode_coprocessor_op(raw, true),
        _ => Insn::Undefined,
    }
}

/// `PLD [Rn, offset]` — addressing mode 2 with `P == 1`, `U` free, `W == 0`.
fn decode_pld(raw: u32) -> Insn {
    let offset = if bit(raw, 25) {
        Offset::Reg {
            rm: field(raw, 3, 0) as u8,
            shift: Shift::Imm {
                ty: ShiftType::from_bits(field(raw, 6, 5)),
                amount: field(raw, 11, 7) as u8,
            },
        }
    } else {
        Offset::Imm(field(raw, 11, 0) as u16)
    };
    Insn::Pld {
        rn: field(raw, 19, 16) as u8,
        up: bit(raw, 23),
        offset,
    }
}

/// Everything with a real condition field.
fn decode_conditional(raw: u32) -> Insn {
    match field(raw, 27, 25) {
        0b000 => decode_group_000(raw),
        0b001 => decode_group_001(raw),
        0b010 => decode_load_store(raw, false),
        0b011 => {
            // A3.1: a set bit 4 in this group is the "media" space, which
            // ARMv6 defines and ARMv5 leaves undefined.
            if bit(raw, 4) {
                Insn::Undefined
            } else {
                decode_load_store(raw, true)
            }
        }
        0b100 => decode_block_transfer(raw),
        0b101 => {
            let imm24 = field(raw, 23, 0);
            let offset = ((imm24 << 8) as i32) >> 6;
            Insn::Branch {
                link: bit(raw, 24),
                offset,
            }
        }
        0b110 => decode_coprocessor_transfer(raw, false),
        _ => {
            if bit(raw, 24) {
                Insn::Swi {
                    imm: field(raw, 23, 0),
                }
            } else {
                decode_coprocessor_op(raw, false)
            }
        }
    }
}

/// `0b000`: data processing with a register operand, plus the multiply, misc
/// and extra load/store spaces carved out of it (ARM ARM A3.4, A3.5).
fn decode_group_000(raw: u32) -> Insn {
    if bit(raw, 7) && bit(raw, 4) {
        // A3.5: bits 7 and 4 both set is the multiply / extra load-store
        // space, never a data-processing instruction.
        return if field(raw, 7, 4) == 0b1001 {
            decode_multiply_or_swap(raw)
        } else {
            decode_extra_load_store(raw)
        };
    }
    if field(raw, 24, 23) == 0b10 && !bit(raw, 20) {
        // A3.4: opcode `10xx` with S clear is not a compare — it is the
        // miscellaneous space.
        return decode_misc(raw);
    }
    decode_data_proc_reg(raw)
}

/// `MUL`, `MLA`, the long multiplies, and `SWP` (ARM ARM A3.5).
fn decode_multiply_or_swap(raw: u32) -> Insn {
    match field(raw, 27, 23) {
        0b00000 => Insn::Mul {
            accumulate: bit(raw, 21),
            s: bit(raw, 20),
            rd: field(raw, 19, 16) as u8,
            rn: field(raw, 15, 12) as u8,
            rm: field(raw, 3, 0) as u8,
            rs: field(raw, 11, 8) as u8,
        },
        0b00001 => Insn::MulLong {
            signed: bit(raw, 22),
            accumulate: bit(raw, 21),
            s: bit(raw, 20),
            rdhi: field(raw, 19, 16) as u8,
            rdlo: field(raw, 15, 12) as u8,
            rm: field(raw, 3, 0) as u8,
            rs: field(raw, 11, 8) as u8,
        },
        0b00010 if field(raw, 21, 20) == 0 && field(raw, 11, 8) == 0 => Insn::Swap {
            byte: bit(raw, 22),
            rd: field(raw, 15, 12) as u8,
            rn: field(raw, 19, 16) as u8,
            rm: field(raw, 3, 0) as u8,
        },
        _ => Insn::Undefined,
    }
}

/// Addressing mode 3 (ARM ARM A5.3): halfword, signed byte, doubleword.
fn decode_extra_load_store(raw: u32) -> Insn {
    let load = bit(raw, 20);
    let op = match (load, field(raw, 6, 5)) {
        (false, 0b01) => ExtraOp::Strh,
        (false, 0b10) => ExtraOp::Ldrd,
        (false, 0b11) => ExtraOp::Strd,
        (true, 0b01) => ExtraOp::Ldrh,
        (true, 0b10) => ExtraOp::Ldrsb,
        (true, 0b11) => ExtraOp::Ldrsh,
        // `sh == 0b00` is the multiply space, handled before we get here.
        _ => return Insn::Undefined,
    };
    let offset = if bit(raw, 22) {
        Offset::Imm(((field(raw, 11, 8) << 4) | field(raw, 3, 0)) as u16)
    } else {
        Offset::Reg {
            rm: field(raw, 3, 0) as u8,
            shift: Shift::Imm {
                ty: ShiftType::Lsl,
                amount: 0,
            },
        }
    };
    let index = if bit(raw, 24) {
        Index::Pre {
            writeback: bit(raw, 21),
        }
    } else if bit(raw, 21) {
        // W must be clear in a post-indexed mode 3 access; ARMv5 has no
        // `LDRHT`.
        return Insn::Undefined;
    } else {
        Index::Post {
            unprivileged: false,
        }
    };
    Insn::LoadStoreExtra {
        op,
        up: bit(raw, 23),
        index,
        rn: field(raw, 19, 16) as u8,
        rd: field(raw, 15, 12) as u8,
        offset,
    }
}

/// The miscellaneous space (ARM ARM A3.4): `MRS`, `MSR`, `BX`, `BLX`, `CLZ`,
/// `BKPT`, the saturating arithmetic and the half-word multiplies.
fn decode_misc(raw: u32) -> Insn {
    let op = field(raw, 22, 21);
    if bit(raw, 7) {
        // Bit 7 set with bit 4 clear is the signed multiply family; both set
        // was already routed to the multiply/extra-load-store space.
        //
        // Bits 7..4 are `1 y x 0` (ARM ARM A4.1.87 and relatives): bit 6 is
        // `y`, which picks a half of `Rs`, and bit 5 is `x`, which picks a half
        // of `Rm`. Getting these the wrong way round produces an emulator that
        // passes every `BB` test and fails every `BT` one.
        let x = if bit(raw, 5) { Half::Top } else { Half::Bottom };
        let y = if bit(raw, 6) { Half::Top } else { Half::Bottom };
        let op = match op {
            0b00 => HalfMulOp::Smla,
            // The `W` forms have no `x` — `Rm` is used whole — so the same bit
            // 5 chooses between accumulating and not (ARM ARM A4.1.85,
            // A4.1.89).
            0b01 if bit(raw, 5) => HalfMulOp::Smulw,
            0b01 => HalfMulOp::Smlaw,
            0b10 => HalfMulOp::Smlal,
            _ => HalfMulOp::Smul,
        };
        return Insn::HalfMul {
            op,
            rd: field(raw, 19, 16) as u8,
            rn: field(raw, 15, 12) as u8,
            rm: field(raw, 3, 0) as u8,
            rs: field(raw, 11, 8) as u8,
            x,
            y,
        };
    }
    match field(raw, 7, 4) {
        0b0000 if op & 0b01 == 0 => Insn::Mrs {
            rd: field(raw, 15, 12) as u8,
            spsr: bit(raw, 22),
        },
        0b0000 => Insn::Msr {
            spsr: bit(raw, 22),
            mask: field(raw, 19, 16) as u8,
            operand: Operand::Reg {
                rm: field(raw, 3, 0) as u8,
                shift: Shift::Imm {
                    ty: ShiftType::Lsl,
                    amount: 0,
                },
            },
        },
        0b0001 if op == 0b01 => Insn::Bx {
            rm: field(raw, 3, 0) as u8,
        },
        0b0001 if op == 0b11 => Insn::Clz {
            rd: field(raw, 15, 12) as u8,
            rm: field(raw, 3, 0) as u8,
        },
        0b0011 if op == 0b01 => Insn::BlxReg {
            rm: field(raw, 3, 0) as u8,
        },
        0b0101 => Insn::Saturating {
            op: match op {
                0b00 => SatOp::QAdd,
                0b01 => SatOp::QSub,
                0b10 => SatOp::QDAdd,
                _ => SatOp::QDSub,
            },
            rd: field(raw, 15, 12) as u8,
            rm: field(raw, 3, 0) as u8,
            rn: field(raw, 19, 16) as u8,
        },
        0b0111 if op == 0b01 => Insn::Bkpt {
            imm: ((field(raw, 19, 8) << 4) | field(raw, 3, 0)) as u16,
        },
        _ => Insn::Undefined,
    }
}

/// Data processing with a register second operand (ARM ARM A5.1.4–A5.1.11).
fn decode_data_proc_reg(raw: u32) -> Insn {
    let ty = ShiftType::from_bits(field(raw, 6, 5));
    let shift = if bit(raw, 4) {
        Shift::Reg {
            ty,
            rs: field(raw, 11, 8) as u8,
        }
    } else {
        Shift::Imm {
            ty,
            amount: field(raw, 11, 7) as u8,
        }
    };
    Insn::DataProc {
        op: DpOp::from_bits(field(raw, 24, 21)),
        s: bit(raw, 20),
        rd: field(raw, 15, 12) as u8,
        rn: field(raw, 19, 16) as u8,
        operand: Operand::Reg {
            rm: field(raw, 3, 0) as u8,
            shift,
        },
    }
}

/// `0b001`: data processing with an immediate, and `MSR` immediate.
fn decode_group_001(raw: u32) -> Insn {
    let operand = Operand::Imm {
        imm8: field(raw, 7, 0) as u8,
        rotate: field(raw, 11, 8) as u8,
    };
    if field(raw, 24, 23) == 0b10 && !bit(raw, 20) {
        // Opcode `10xx` with S clear: `MSR` immediate if bit 21 is set,
        // architecturally undefined otherwise (ARM ARM A3.4's note).
        return if bit(raw, 21) {
            Insn::Msr {
                spsr: bit(raw, 22),
                mask: field(raw, 19, 16) as u8,
                operand,
            }
        } else {
            Insn::Undefined
        };
    }
    Insn::DataProc {
        op: DpOp::from_bits(field(raw, 24, 21)),
        s: bit(raw, 20),
        rd: field(raw, 15, 12) as u8,
        rn: field(raw, 19, 16) as u8,
        operand,
    }
}

/// Addressing mode 2 (ARM ARM A5.2): `LDR`, `STR`, `LDRB`, `STRB`.
fn decode_load_store(raw: u32, register_offset: bool) -> Insn {
    let offset = if register_offset {
        Offset::Reg {
            rm: field(raw, 3, 0) as u8,
            shift: Shift::Imm {
                ty: ShiftType::from_bits(field(raw, 6, 5)),
                amount: field(raw, 11, 7) as u8,
            },
        }
    } else {
        Offset::Imm(field(raw, 11, 0) as u16)
    };
    let index = if bit(raw, 24) {
        Index::Pre {
            writeback: bit(raw, 21),
        }
    } else {
        Index::Post {
            unprivileged: bit(raw, 21),
        }
    };
    Insn::LoadStore {
        load: bit(raw, 20),
        byte: bit(raw, 22),
        up: bit(raw, 23),
        index,
        rn: field(raw, 19, 16) as u8,
        rd: field(raw, 15, 12) as u8,
        offset,
    }
}

/// Addressing mode 4 (ARM ARM A5.4): `LDM` and `STM`.
fn decode_block_transfer(raw: u32) -> Insn {
    Insn::BlockTransfer {
        load: bit(raw, 20),
        before: bit(raw, 24),
        up: bit(raw, 23),
        user: bit(raw, 22),
        writeback: bit(raw, 21),
        rn: field(raw, 19, 16) as u8,
        list: field(raw, 15, 0) as u16,
    }
}

/// Addressing mode 5 (ARM ARM A5.5): `LDC`/`STC`, and the `MCRR`/`MRRC` pair
/// moves that ARMv5TE carved out of the same space.
fn decode_coprocessor_transfer(raw: u32, second: bool) -> Insn {
    if field(raw, 24, 21) == 0b0010 {
        return Insn::CpRegPair {
            cp: field(raw, 11, 8) as u8,
            load: bit(raw, 20),
            opc: field(raw, 7, 4) as u8,
            rd: field(raw, 15, 12) as u8,
            rn: field(raw, 19, 16) as u8,
            crm: field(raw, 3, 0) as u8,
        };
    }
    let index = if bit(raw, 24) {
        Index::Pre {
            writeback: bit(raw, 21),
        }
    } else if bit(raw, 21) {
        Index::Post {
            unprivileged: false,
        }
    } else {
        // P == 0, W == 0 is the "unindexed" form, where the eight-bit field is
        // a coprocessor option rather than an offset. It is still a transfer
        // at [Rn]; the option travels in `offset` for the coprocessor to read.
        Index::Pre { writeback: false }
    };
    Insn::CpTransfer {
        cp: field(raw, 11, 8) as u8,
        load: bit(raw, 20),
        long: bit(raw, 22),
        crd: field(raw, 15, 12) as u8,
        rn: field(raw, 19, 16) as u8,
        index,
        up: bit(raw, 23),
        offset: field(raw, 7, 0) as u8,
        second,
    }
}

/// `CDP`, `MCR` and `MRC` (ARM ARM A4.1.19, A4.1.32, A4.1.30).
fn decode_coprocessor_op(raw: u32, second: bool) -> Insn {
    if bit(raw, 4) {
        Insn::CpReg {
            cp: field(raw, 11, 8) as u8,
            load: bit(raw, 20),
            opc1: field(raw, 23, 21) as u8,
            rd: field(raw, 15, 12) as u8,
            crn: field(raw, 19, 16) as u8,
            crm: field(raw, 3, 0) as u8,
            opc2: field(raw, 7, 5) as u8,
            second,
        }
    } else {
        Insn::Cdp {
            cp: field(raw, 11, 8) as u8,
            opc1: field(raw, 23, 20) as u8,
            crd: field(raw, 15, 12) as u8,
            crn: field(raw, 19, 16) as u8,
            crm: field(raw, 3, 0) as u8,
            opc2: field(raw, 7, 5) as u8,
            second,
        }
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

/// The `LDM`/`STM` addressing suffix (ARM ARM A5.4.2).
const fn block_suffix(before: bool, up: bool) -> &'static str {
    match (before, up) {
        (false, true) => "IA",
        (true, true) => "IB",
        (false, false) => "DA",
        (true, false) => "DB",
    }
}

/// The four `MSR` field letters, in the order an assembler writes them.
const fn msr_fields(mask: u8) -> [&'static str; 4] {
    [
        if mask & 0b0001 != 0 { "c" } else { "" },
        if mask & 0b0010 != 0 { "x" } else { "" },
        if mask & 0b0100 != 0 { "s" } else { "" },
        if mask & 0b1000 != 0 { "f" } else { "" },
    ]
}

impl fmt::Display for Decoded {
    /// ARM assembler syntax: uppercase mnemonics, lowercase register names,
    /// the condition suffix where there is one.
    ///
    /// Branch targets print as signed offsets rather than absolute addresses,
    /// because a bare [`Decoded`] does not know where it came from.
    /// [`super::disasm`] does, and prints the address.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let c = self.cond;
        match self.insn {
            Insn::DataProc {
                op,
                s,
                rd,
                rn,
                operand,
            } => {
                let flag = if s && op.writes_result() { "S" } else { "" };
                write!(f, "{op}{c}{flag} ")?;
                if op.writes_result() {
                    write!(f, "{}, ", RegName(rd))?;
                }
                if op.reads_rn() {
                    write!(f, "{}, ", RegName(rn))?;
                }
                write!(f, "{operand}")
            }
            Insn::Mrs { rd, spsr } => {
                let src = if spsr { "SPSR" } else { "CPSR" };
                write!(f, "MRS{c} {}, {src}", RegName(rd))
            }
            Insn::Msr {
                spsr,
                mask,
                operand,
            } => {
                let dst = if spsr { "SPSR" } else { "CPSR" };
                let [a, b, cc, d] = msr_fields(mask);
                write!(f, "MSR{c} {dst}_{a}{b}{cc}{d}, {operand}")
            }
            Insn::Bx { rm } => write!(f, "BX{c} {}", RegName(rm)),
            Insn::BlxReg { rm } => write!(f, "BLX{c} {}", RegName(rm)),
            Insn::Clz { rd, rm } => write!(f, "CLZ{c} {}, {}", RegName(rd), RegName(rm)),
            Insn::Bkpt { imm } => write!(f, "BKPT #{imm}"),
            Insn::Branch { link, offset } => {
                let l = if link { "L" } else { "" };
                write!(f, "B{l}{c} {offset:+}")
            }
            Insn::BlxImm { offset } => write!(f, "BLX {offset:+}"),
            Insn::Mul {
                accumulate,
                s,
                rd,
                rn,
                rm,
                rs,
            } => {
                let flag = if s { "S" } else { "" };
                if accumulate {
                    write!(
                        f,
                        "MLA{c}{flag} {}, {}, {}, {}",
                        RegName(rd),
                        RegName(rm),
                        RegName(rs),
                        RegName(rn)
                    )
                } else {
                    write!(
                        f,
                        "MUL{c}{flag} {}, {}, {}",
                        RegName(rd),
                        RegName(rm),
                        RegName(rs)
                    )
                }
            }
            Insn::MulLong {
                signed,
                accumulate,
                s,
                rdhi,
                rdlo,
                rm,
                rs,
            } => {
                let sign = if signed { "S" } else { "U" };
                let kind = if accumulate { "MLAL" } else { "MULL" };
                let flag = if s { "S" } else { "" };
                write!(
                    f,
                    "{sign}{kind}{c}{flag} {}, {}, {}, {}",
                    RegName(rdlo),
                    RegName(rdhi),
                    RegName(rm),
                    RegName(rs)
                )
            }
            Insn::Saturating { op, rd, rm, rn } => write!(
                f,
                "{}{c} {}, {}, {}",
                op.mnemonic(),
                RegName(rd),
                RegName(rm),
                RegName(rn)
            ),
            Insn::HalfMul {
                op,
                rd,
                rn,
                rm,
                rs,
                x,
                y,
            } => match op {
                HalfMulOp::Smla => write!(
                    f,
                    "SMLA{}{}{c} {}, {}, {}, {}",
                    x.suffix(),
                    y.suffix(),
                    RegName(rd),
                    RegName(rm),
                    RegName(rs),
                    RegName(rn)
                ),
                HalfMulOp::Smlaw => write!(
                    f,
                    "SMLAW{}{c} {}, {}, {}, {}",
                    y.suffix(),
                    RegName(rd),
                    RegName(rm),
                    RegName(rs),
                    RegName(rn)
                ),
                HalfMulOp::Smulw => write!(
                    f,
                    "SMULW{}{c} {}, {}, {}",
                    y.suffix(),
                    RegName(rd),
                    RegName(rm),
                    RegName(rs)
                ),
                HalfMulOp::Smlal => write!(
                    f,
                    "SMLAL{}{}{c} {}, {}, {}, {}",
                    x.suffix(),
                    y.suffix(),
                    RegName(rn),
                    RegName(rd),
                    RegName(rm),
                    RegName(rs)
                ),
                HalfMulOp::Smul => write!(
                    f,
                    "SMUL{}{}{c} {}, {}, {}",
                    x.suffix(),
                    y.suffix(),
                    RegName(rd),
                    RegName(rm),
                    RegName(rs)
                ),
            },
            Insn::LoadStore {
                load,
                byte,
                up,
                index,
                rn,
                rd,
                offset,
            } => {
                let name = if load { "LDR" } else { "STR" };
                let b = if byte { "B" } else { "" };
                let t = match index {
                    Index::Post {
                        unprivileged: true, ..
                    } => "T",
                    _ => "",
                };
                write!(
                    f,
                    "{name}{c}{b}{t} {}, {}",
                    RegName(rd),
                    Addressing {
                        rn,
                        up,
                        index,
                        offset
                    }
                )
            }
            Insn::LoadStoreExtra {
                op,
                up,
                index,
                rn,
                rd,
                offset,
            } => write!(
                f,
                "{}{c} {}, {}",
                op.mnemonic(),
                RegName(rd),
                Addressing {
                    rn,
                    up,
                    index,
                    offset
                }
            ),
            Insn::BlockTransfer {
                load,
                before,
                up,
                user,
                writeback,
                rn,
                list,
            } => {
                let name = if load { "LDM" } else { "STM" };
                let mode = block_suffix(before, up);
                let w = if writeback { "!" } else { "" };
                let u = if user { "^" } else { "" };
                write!(
                    f,
                    "{name}{c}{mode} {}{w}, {}{u}",
                    RegName(rn),
                    RegList(list)
                )
            }
            Insn::Swap { byte, rd, rn, rm } => {
                let b = if byte { "B" } else { "" };
                write!(
                    f,
                    "SWP{c}{b} {}, {}, [{}]",
                    RegName(rd),
                    RegName(rm),
                    RegName(rn)
                )
            }
            Insn::Swi { imm } => write!(f, "SWI{c} #{imm}"),
            Insn::Pld { rn, up, offset } => write!(
                f,
                "PLD {}",
                Addressing {
                    rn,
                    up,
                    index: Index::Pre { writeback: false },
                    offset
                }
            ),
            Insn::Cdp {
                cp,
                opc1,
                crd,
                crn,
                crm,
                opc2,
                second,
            } => {
                let two = if second { "2" } else { "" };
                let cond = if second { Cond::AL } else { c };
                write!(
                    f,
                    "CDP{two}{cond} p{cp}, #{opc1}, c{crd}, c{crn}, c{crm}, #{opc2}"
                )
            }
            Insn::CpReg {
                cp,
                load,
                opc1,
                rd,
                crn,
                crm,
                opc2,
                second,
            } => {
                let name = if load { "MRC" } else { "MCR" };
                let two = if second { "2" } else { "" };
                let cond = if second { Cond::AL } else { c };
                write!(
                    f,
                    "{name}{two}{cond} p{cp}, #{opc1}, {}, c{crn}, c{crm}, #{opc2}",
                    RegName(rd)
                )
            }
            Insn::CpRegPair {
                cp,
                load,
                opc,
                rd,
                rn,
                crm,
            } => {
                let name = if load { "MRRC" } else { "MCRR" };
                write!(
                    f,
                    "{name}{c} p{cp}, #{opc}, {}, {}, c{crm}",
                    RegName(rd),
                    RegName(rn)
                )
            }
            Insn::CpTransfer {
                cp,
                load,
                long,
                crd,
                rn,
                index,
                up,
                offset,
                second,
            } => {
                let name = if load { "LDC" } else { "STC" };
                let two = if second { "2" } else { "" };
                let l = if long { "L" } else { "" };
                let cond = if second { Cond::AL } else { c };
                write!(
                    f,
                    "{name}{two}{cond}{l} p{cp}, c{crd}, {}",
                    Addressing {
                        rn,
                        up,
                        index,
                        // The word offset is scaled by four in the syntax.
                        offset: Offset::Imm(u16::from(offset) * 4),
                    }
                )
            }
            Insn::Undefined => write!(f, "UNDEFINED ; 0x{:08x}", self.raw),
        }
    }
}
