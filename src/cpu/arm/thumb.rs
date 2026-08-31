//! The Thumb instruction set, described once — same rule as [`super::isa`].
//!
//! Thumb is decoded into its own semantic [`Thumb`] enum rather than being
//! rewritten into the ARM one. Two reasons, and the second is the real one:
//!
//! - the *syntax* differs. `LSL r0, r1, #3` and `MOVS r0, r1, LSL #3` are the
//!   same operation, but only one of them is what a Thumb disassembly should
//!   print, and a disassembler nobody can read against the source is not
//!   doing its job.
//! - the *flag rules* differ. Almost every Thumb data-processing instruction
//!   sets the flags unconditionally, while the high-register forms
//!   (`ADD`/`MOV`/`CMP` with `H1` or `H2`) set none — an `S` bit that is
//!   implied by the encoding rather than carried in it.
//!
//! The *semantics* are not duplicated: `super::exec` executes a [`Thumb`] with
//! the same barrel shifter, ALU and memory helpers the ARM path uses, so there
//! is one implementation of `ADC`, one of `LDR`, and one shifter.
//!
//! # Sources
//!
//! ARM ARM (DDI 0100) A6, "The Thumb Instruction Set": A6.1's encoding
//! summary, and A7.1's alphabetical list for each instruction's operands and
//! flag effects. `BLX` in both its forms is A7.1.11 and A7.1.12 — ARMv5T
//! additions, as is `BKPT` (A7.1.10).

use core::fmt;

use super::isa::{Cond, RegName, ShiftType, bit, field};

/// Extract bits `hi..=lo` of a Thumb halfword.
#[inline]
#[must_use]
const fn hfield(word: u16, hi: u32, lo: u32) -> u16 {
    field(word as u32, hi, lo) as u16
}

/// The four operations of the eight-bit-immediate format (ARM ARM A6.1,
/// format 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ImmOp {
    /// `MOV Rd, #imm8`.
    Mov,
    /// `CMP Rd, #imm8`.
    Cmp,
    /// `ADD Rd, #imm8`.
    Add,
    /// `SUB Rd, #imm8`.
    Sub,
}

impl ImmOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            ImmOp::Mov => "MOV",
            ImmOp::Cmp => "CMP",
            ImmOp::Add => "ADD",
            ImmOp::Sub => "SUB",
        }
    }
}

/// The sixteen register-to-register ALU operations (ARM ARM A6.1, format 4).
///
/// All sixteen set the flags. `NEG` and `MUL` have no ARM data-processing
/// counterpart with the same encoding, which is why this is its own enum
/// rather than a reuse of [`DpOp`](super::isa::DpOp).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AluOp {
    /// Bitwise and.
    And,
    /// Bitwise exclusive or.
    Eor,
    /// Logical shift left by `Rs`.
    Lsl,
    /// Logical shift right by `Rs`.
    Lsr,
    /// Arithmetic shift right by `Rs`.
    Asr,
    /// Add with carry.
    Adc,
    /// Subtract with borrow.
    Sbc,
    /// Rotate right by `Rs`.
    Ror,
    /// Test.
    Tst,
    /// Negate: `0 - Rm`.
    Neg,
    /// Compare.
    Cmp,
    /// Compare negative.
    Cmn,
    /// Bitwise or.
    Orr,
    /// Multiply.
    Mul,
    /// Bit clear.
    Bic,
    /// Move not.
    Mvn,
}

impl AluOp {
    /// Decode the four-bit `op` field.
    #[must_use]
    pub const fn from_bits(bits: u16) -> AluOp {
        match bits & 0xf {
            0x0 => AluOp::And,
            0x1 => AluOp::Eor,
            0x2 => AluOp::Lsl,
            0x3 => AluOp::Lsr,
            0x4 => AluOp::Asr,
            0x5 => AluOp::Adc,
            0x6 => AluOp::Sbc,
            0x7 => AluOp::Ror,
            0x8 => AluOp::Tst,
            0x9 => AluOp::Neg,
            0xa => AluOp::Cmp,
            0xb => AluOp::Cmn,
            0xc => AluOp::Orr,
            0xd => AluOp::Mul,
            0xe => AluOp::Bic,
            _ => AluOp::Mvn,
        }
    }

    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            AluOp::And => "AND",
            AluOp::Eor => "EOR",
            AluOp::Lsl => "LSL",
            AluOp::Lsr => "LSR",
            AluOp::Asr => "ASR",
            AluOp::Adc => "ADC",
            AluOp::Sbc => "SBC",
            AluOp::Ror => "ROR",
            AluOp::Tst => "TST",
            AluOp::Neg => "NEG",
            AluOp::Cmp => "CMP",
            AluOp::Cmn => "CMN",
            AluOp::Orr => "ORR",
            AluOp::Mul => "MUL",
            AluOp::Bic => "BIC",
            AluOp::Mvn => "MVN",
        }
    }
}

/// The three high-register operations (ARM ARM A6.1, format 5).
///
/// These are the only Thumb data-processing instructions that do **not** set
/// the flags — except `CMP`, which exists only for them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HiOp {
    /// `ADD Rd, Rm` — no flags.
    Add,
    /// `CMP Rn, Rm` — flags only.
    Cmp,
    /// `MOV Rd, Rm` — no flags.
    Mov,
}

impl HiOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            HiOp::Add => "ADD",
            HiOp::Cmp => "CMP",
            HiOp::Mov => "MOV",
        }
    }
}

/// The eight register-offset memory operations (ARM ARM A6.1, formats 7 & 8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MemRegOp {
    /// Store word.
    Str,
    /// Store halfword.
    Strh,
    /// Store byte.
    Strb,
    /// Load signed byte.
    Ldrsb,
    /// Load word.
    Ldr,
    /// Load unsigned halfword.
    Ldrh,
    /// Load unsigned byte.
    Ldrb,
    /// Load signed halfword.
    Ldrsh,
}

impl MemRegOp {
    /// Decode the three-bit opcode at bits 11..9.
    #[must_use]
    pub const fn from_bits(bits: u16) -> MemRegOp {
        match bits & 0b111 {
            0 => MemRegOp::Str,
            1 => MemRegOp::Strh,
            2 => MemRegOp::Strb,
            3 => MemRegOp::Ldrsb,
            4 => MemRegOp::Ldr,
            5 => MemRegOp::Ldrh,
            6 => MemRegOp::Ldrb,
            _ => MemRegOp::Ldrsh,
        }
    }

    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            MemRegOp::Str => "STR",
            MemRegOp::Strh => "STRH",
            MemRegOp::Strb => "STRB",
            MemRegOp::Ldrsb => "LDRSB",
            MemRegOp::Ldr => "LDR",
            MemRegOp::Ldrh => "LDRH",
            MemRegOp::Ldrb => "LDRB",
            MemRegOp::Ldrsh => "LDRSH",
        }
    }

    /// Whether the operation reads memory.
    #[must_use]
    pub const fn is_load(self) -> bool {
        matches!(
            self,
            MemRegOp::Ldrsb | MemRegOp::Ldr | MemRegOp::Ldrh | MemRegOp::Ldrb | MemRegOp::Ldrsh
        )
    }
}

/// How wide an immediate-offset memory access is (ARM ARM A6.1, formats 9 &
/// 10).
///
/// The offset field is scaled by the access width, which is why the width has
/// to survive decode rather than being folded into a byte offset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MemSize {
    /// One byte; the offset is unscaled.
    Byte,
    /// Two bytes; the offset is scaled by two.
    Half,
    /// Four bytes; the offset is scaled by four.
    Word,
}

impl MemSize {
    /// The number of bytes transferred, and the offset scale factor.
    #[must_use]
    pub const fn bytes(self) -> u32 {
        match self {
            MemSize::Byte => 1,
            MemSize::Half => 2,
            MemSize::Word => 4,
        }
    }

    /// The mnemonic suffix: `B`, `H` or nothing.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            MemSize::Byte => "B",
            MemSize::Half => "H",
            MemSize::Word => "",
        }
    }
}

/// One decoded Thumb instruction.
///
/// The variant names follow the ARM ARM's format numbering loosely; the field
/// names follow its assembler syntax, because those are what both consumers
/// need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Thumb {
    /// Format 1: `LSL`/`LSR`/`ASR` by a five-bit immediate. Sets the flags.
    ShiftImm {
        /// Which shift. `ROR` has no format-1 encoding.
        ty: ShiftType,
        /// Destination.
        rd: u8,
        /// Source.
        rm: u8,
        /// The raw `imm5`; zero means 32 for `LSR` and `ASR`, and "no shift"
        /// for `LSL` (ARM ARM A7.1.68).
        imm: u8,
    },
    /// Format 2: `ADD`/`SUB` of two low registers, or of a three-bit
    /// immediate. Sets the flags.
    AddSub {
        /// Subtract rather than add.
        sub: bool,
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Second operand: a register or a three-bit immediate.
        operand: SmallOperand,
    },
    /// Format 3: `MOV`/`CMP`/`ADD`/`SUB` with an eight-bit immediate. Sets the
    /// flags.
    AluImm {
        /// Which operation.
        op: ImmOp,
        /// The register, both source and destination where there is one.
        rd: u8,
        /// The immediate.
        imm: u8,
    },
    /// Format 4: register-to-register ALU. Sets the flags.
    Alu {
        /// Which operation.
        op: AluOp,
        /// Destination and, for most operations, first operand.
        rd: u8,
        /// Second operand.
        rm: u8,
    },
    /// Format 5: `ADD`/`CMP`/`MOV` reaching the high registers. Only `CMP`
    /// touches the flags.
    HiReg {
        /// Which operation.
        op: HiOp,
        /// Destination and first operand; may be `R8`–`R15`.
        rd: u8,
        /// Second operand; may be `R8`–`R15`.
        rm: u8,
    },
    /// Format 5: `BX` and `BLX` (register).
    BranchExchange {
        /// Set `LR` to the return address (`BLX`, ARMv5T).
        link: bool,
        /// Register holding the target; bit 0 selects the instruction set.
        rm: u8,
    },
    /// Format 6: `LDR Rd, [PC, #imm*4]`, the literal pool load.
    LoadLiteral {
        /// Destination.
        rd: u8,
        /// Word offset from `(PC + 4) & ~3`.
        imm: u8,
    },
    /// Formats 7 and 8: memory access with a register offset.
    MemReg {
        /// Which operation.
        op: MemRegOp,
        /// Data register.
        rd: u8,
        /// Base register.
        rn: u8,
        /// Offset register.
        rm: u8,
    },
    /// Formats 9 and 10: memory access with a scaled immediate offset.
    MemImm {
        /// Whether this reads memory.
        load: bool,
        /// How wide.
        size: MemSize,
        /// Data register.
        rd: u8,
        /// Base register.
        rn: u8,
        /// The raw offset field, before scaling.
        imm: u8,
    },
    /// Format 11: `LDR`/`STR` relative to `SP`.
    MemStack {
        /// Whether this reads memory.
        load: bool,
        /// Data register.
        rd: u8,
        /// Word offset from `SP`.
        imm: u8,
    },
    /// Format 12: `ADD Rd, PC, #imm*4` and `ADD Rd, SP, #imm*4`.
    AddPcSp {
        /// Base on `SP` rather than `PC`.
        sp: bool,
        /// Destination.
        rd: u8,
        /// Word offset.
        imm: u8,
    },
    /// Format 13: `ADD SP, #imm*4` and `SUB SP, #imm*4`.
    AdjustStack {
        /// Subtract rather than add.
        sub: bool,
        /// Word offset, seven bits.
        imm: u8,
    },
    /// Format 14: `PUSH` and `POP`.
    PushPop {
        /// `POP` rather than `PUSH`.
        load: bool,
        /// `LR` for a `PUSH`, `PC` for a `POP`.
        extra: bool,
        /// One bit per low register.
        list: u8,
    },
    /// Format 15: `LDMIA`/`STMIA` with writeback.
    BlockTransfer {
        /// Whether this reads memory.
        load: bool,
        /// Base register, always written back.
        rn: u8,
        /// One bit per low register.
        list: u8,
    },
    /// Format 16: conditional branch.
    BranchCond {
        /// The condition. `0b1110` and `0b1111` are not branches and never
        /// reach here.
        cond: Cond,
        /// Byte offset from `PC + 4`.
        offset: i32,
    },
    /// Format 17: `SWI`.
    Swi {
        /// The eight-bit comment field.
        imm: u8,
    },
    /// `BKPT` (ARMv5T).
    Bkpt {
        /// The eight-bit comment field.
        imm: u8,
    },
    /// Format 18: unconditional branch.
    Branch {
        /// Byte offset from `PC + 4`.
        offset: i32,
    },
    /// Format 19, first halfword: `LR = PC + 4 + (offset << 12)`.
    ///
    /// A separate instruction, not half of one: it executes on its own, and an
    /// interrupt may be taken between the two halves.
    BranchLinkPrefix {
        /// The already-scaled and sign-extended high part of the offset.
        offset: i32,
    },
    /// Format 19, second halfword: `BL` or `BLX` completing the pair.
    BranchLinkSuffix {
        /// Switch to ARM state and clear bit 1 of the target (`BLX`).
        exchange: bool,
        /// The offset added to `LR`, already scaled by two.
        offset: u32,
    },
    /// An encoding this architecture does not define.
    Undefined,
}

/// The second operand of the format-2 `ADD`/`SUB`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SmallOperand {
    /// A low register.
    Reg(u8),
    /// A three-bit immediate.
    Imm(u8),
}

impl fmt::Display for SmallOperand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            SmallOperand::Reg(r) => write!(f, "{}", RegName(r)),
            SmallOperand::Imm(i) => write!(f, "#{i}"),
        }
    }
}

/// Decode one Thumb halfword.
///
/// Never fails: an undefined encoding decodes to [`Thumb::Undefined`], which
/// the interpreter turns into an Undefined Instruction exception.
///
/// The order of the arms follows ARM ARM A6.1's encoding table top to bottom;
/// the narrower patterns (`BKPT` inside the `1011` block, `SWI` and the
/// undefined `0b1110` condition inside the conditional-branch block) are
/// tested before the block they sit in.
#[must_use]
#[allow(clippy::too_many_lines)] // One arm per format; splitting it hides the table.
pub fn decode(raw: u16) -> Thumb {
    let rd = hfield(raw, 2, 0) as u8;
    let rn = hfield(raw, 5, 3) as u8;
    match hfield(raw, 15, 13) {
        0b000 => {
            if hfield(raw, 12, 11) == 0b11 {
                // Format 2: the only sub-case where bits 12..11 are not a
                // shift type.
                let sub = bit(u32::from(raw), 9);
                let operand = if bit(u32::from(raw), 10) {
                    SmallOperand::Imm(hfield(raw, 8, 6) as u8)
                } else {
                    SmallOperand::Reg(hfield(raw, 8, 6) as u8)
                };
                Thumb::AddSub {
                    sub,
                    rd,
                    rn,
                    operand,
                }
            } else {
                Thumb::ShiftImm {
                    ty: ShiftType::from_bits(u32::from(hfield(raw, 12, 11))),
                    rd,
                    rm: rn,
                    imm: hfield(raw, 10, 6) as u8,
                }
            }
        }
        0b001 => Thumb::AluImm {
            op: match hfield(raw, 12, 11) {
                0 => ImmOp::Mov,
                1 => ImmOp::Cmp,
                2 => ImmOp::Add,
                _ => ImmOp::Sub,
            },
            rd: hfield(raw, 10, 8) as u8,
            imm: hfield(raw, 7, 0) as u8,
        },
        0b010 => decode_group_010(raw),
        0b011 => Thumb::MemImm {
            load: bit(u32::from(raw), 11),
            size: if bit(u32::from(raw), 12) {
                MemSize::Byte
            } else {
                MemSize::Word
            },
            rd,
            rn,
            imm: hfield(raw, 10, 6) as u8,
        },
        0b100 => {
            if bit(u32::from(raw), 12) {
                Thumb::MemStack {
                    load: bit(u32::from(raw), 11),
                    rd: hfield(raw, 10, 8) as u8,
                    imm: hfield(raw, 7, 0) as u8,
                }
            } else {
                Thumb::MemImm {
                    load: bit(u32::from(raw), 11),
                    size: MemSize::Half,
                    rd,
                    rn,
                    imm: hfield(raw, 10, 6) as u8,
                }
            }
        }
        0b101 => {
            if !bit(u32::from(raw), 12) {
                Thumb::AddPcSp {
                    sp: bit(u32::from(raw), 11),
                    rd: hfield(raw, 10, 8) as u8,
                    imm: hfield(raw, 7, 0) as u8,
                }
            } else {
                decode_misc(raw)
            }
        }
        0b110 => {
            if !bit(u32::from(raw), 12) {
                Thumb::BlockTransfer {
                    load: bit(u32::from(raw), 11),
                    rn: hfield(raw, 10, 8) as u8,
                    list: hfield(raw, 7, 0) as u8,
                }
            } else {
                match hfield(raw, 11, 8) {
                    // `1101 1110` is architecturally undefined, and `1101
                    // 1111` is SWI — neither is a branch.
                    0b1110 => Thumb::Undefined,
                    0b1111 => Thumb::Swi {
                        imm: hfield(raw, 7, 0) as u8,
                    },
                    cond => Thumb::BranchCond {
                        cond: Cond(cond as u8),
                        // Sign-extend the eight-bit field, then scale by two.
                        offset: i32::from(hfield(raw, 7, 0) as u8 as i8) * 2,
                    },
                }
            }
        }
        _ => decode_branch(raw),
    }
}

/// `0b010`: the ALU, high-register, literal-load and register-offset formats.
fn decode_group_010(raw: u16) -> Thumb {
    let rd = hfield(raw, 2, 0) as u8;
    let rn = hfield(raw, 5, 3) as u8;
    if bit(u32::from(raw), 12) {
        // Formats 7 and 8 share one three-bit opcode.
        return Thumb::MemReg {
            op: MemRegOp::from_bits(hfield(raw, 11, 9)),
            rd,
            rn,
            rm: hfield(raw, 8, 6) as u8,
        };
    }
    if bit(u32::from(raw), 11) {
        return Thumb::LoadLiteral {
            rd: hfield(raw, 10, 8) as u8,
            imm: hfield(raw, 7, 0) as u8,
        };
    }
    if bit(u32::from(raw), 10) {
        // Format 5: H1 and H2 extend the register numbers to four bits.
        let rm = (hfield(raw, 6, 3) as u8) & 0xf;
        let hd = rd | if bit(u32::from(raw), 7) { 8 } else { 0 };
        return match hfield(raw, 9, 8) {
            0b00 => Thumb::HiReg {
                op: HiOp::Add,
                rd: hd,
                rm,
            },
            0b01 => Thumb::HiReg {
                op: HiOp::Cmp,
                rd: hd,
                rm,
            },
            0b10 => Thumb::HiReg {
                op: HiOp::Mov,
                rd: hd,
                rm,
            },
            // `BX`/`BLX`: H1 selects the link form in ARMv5T, and the low
            // three bits of the encoding must be zero.
            _ => Thumb::BranchExchange {
                link: bit(u32::from(raw), 7),
                rm,
            },
        };
    }
    Thumb::Alu {
        op: AluOp::from_bits(hfield(raw, 9, 6)),
        rd,
        rm: rn,
    }
}

/// The `1011` block: stack adjustment, `PUSH`/`POP` and `BKPT`.
fn decode_misc(raw: u16) -> Thumb {
    match hfield(raw, 11, 8) {
        0b0000 => Thumb::AdjustStack {
            sub: bit(u32::from(raw), 7),
            imm: hfield(raw, 6, 0) as u8,
        },
        0b0100 | 0b0101 | 0b1100 | 0b1101 => Thumb::PushPop {
            load: bit(u32::from(raw), 11),
            extra: bit(u32::from(raw), 8),
            list: hfield(raw, 7, 0) as u8,
        },
        0b1110 => Thumb::Bkpt {
            imm: hfield(raw, 7, 0) as u8,
        },
        _ => Thumb::Undefined,
    }
}

/// `0b111`: the branch formats, including the two halves of `BL`/`BLX`.
fn decode_branch(raw: u16) -> Thumb {
    let imm11 = u32::from(hfield(raw, 10, 0));
    match hfield(raw, 12, 11) {
        // 11100: B <offset11>, sign-extended and scaled by two.
        0b00 => Thumb::Branch {
            offset: (((imm11 << 21) as i32) >> 21) * 2,
        },
        // 11101: the BLX suffix. Bit 0 must be zero — an odd offset here is
        // UNPREDICTABLE, and masking it is what makes the target word-aligned.
        0b01 => Thumb::BranchLinkSuffix {
            exchange: true,
            offset: (imm11 << 1) & !0b11,
        },
        // 11110: the shared prefix. The 11-bit field is bits 22..12 of a
        // signed 23-bit offset.
        0b10 => Thumb::BranchLinkPrefix {
            offset: ((imm11 << 21) as i32) >> 9,
        },
        // 11111: the BL suffix.
        _ => Thumb::BranchLinkSuffix {
            exchange: false,
            offset: imm11 << 1,
        },
    }
}

/// Formats a Thumb register list as `{r0-r3, lr}`.
struct RegList {
    list: u8,
    extra: Option<u8>,
}

impl fmt::Display for RegList {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("{")?;
        let mut first = true;
        let mut i = 0u8;
        while i < 8 {
            if self.list & (1 << i) == 0 {
                i += 1;
                continue;
            }
            let start = i;
            while i < 8 && self.list & (1 << i) != 0 {
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
        if let Some(r) = self.extra {
            let sep = if first { "" } else { ", " };
            write!(f, "{sep}{}", RegName(r))?;
        }
        f.write_str("}")
    }
}

impl fmt::Display for Thumb {
    /// Thumb assembler syntax, as the ARM ARM's A7.1 pages spell it.
    #[allow(clippy::too_many_lines)] // One arm per format; splitting it hides the table.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Thumb::ShiftImm { ty, rd, rm, imm } => {
                // `LSR #0` and `ASR #0` mean 32; `LSL #0` means no shift at
                // all, and an assembler writes it as `MOV`.
                let shown = if imm == 0 && ty != ShiftType::Lsl {
                    32
                } else {
                    u32::from(imm)
                };
                write!(f, "{ty} {}, {}, #{shown}", RegName(rd), RegName(rm))
            }
            Thumb::AddSub {
                sub,
                rd,
                rn,
                operand,
            } => {
                let name = if sub { "SUB" } else { "ADD" };
                write!(f, "{name} {}, {}, {operand}", RegName(rd), RegName(rn))
            }
            Thumb::AluImm { op, rd, imm } => {
                write!(f, "{} {}, #{imm}", op.mnemonic(), RegName(rd))
            }
            Thumb::Alu { op, rd, rm } => {
                write!(f, "{} {}, {}", op.mnemonic(), RegName(rd), RegName(rm))
            }
            Thumb::HiReg { op, rd, rm } => {
                write!(f, "{} {}, {}", op.mnemonic(), RegName(rd), RegName(rm))
            }
            Thumb::BranchExchange { link, rm } => {
                let name = if link { "BLX" } else { "BX" };
                write!(f, "{name} {}", RegName(rm))
            }
            Thumb::LoadLiteral { rd, imm } => {
                write!(f, "LDR {}, [pc, #{}]", RegName(rd), u32::from(imm) * 4)
            }
            Thumb::MemReg { op, rd, rn, rm } => write!(
                f,
                "{} {}, [{}, {}]",
                op.mnemonic(),
                RegName(rd),
                RegName(rn),
                RegName(rm)
            ),
            Thumb::MemImm {
                load,
                size,
                rd,
                rn,
                imm,
            } => {
                let name = if load { "LDR" } else { "STR" };
                write!(
                    f,
                    "{name}{} {}, [{}, #{}]",
                    size.suffix(),
                    RegName(rd),
                    RegName(rn),
                    u32::from(imm) * size.bytes()
                )
            }
            Thumb::MemStack { load, rd, imm } => {
                let name = if load { "LDR" } else { "STR" };
                write!(f, "{name} {}, [sp, #{}]", RegName(rd), u32::from(imm) * 4)
            }
            Thumb::AddPcSp { sp, rd, imm } => {
                let base = if sp { "sp" } else { "pc" };
                write!(f, "ADD {}, {base}, #{}", RegName(rd), u32::from(imm) * 4)
            }
            Thumb::AdjustStack { sub, imm } => {
                let name = if sub { "SUB" } else { "ADD" };
                write!(f, "{name} sp, #{}", u32::from(imm) * 4)
            }
            Thumb::PushPop { load, extra, list } => {
                let name = if load { "POP" } else { "PUSH" };
                let extra = match (extra, load) {
                    (true, true) => Some(15),
                    (true, false) => Some(14),
                    (false, _) => None,
                };
                write!(f, "{name} {}", RegList { list, extra })
            }
            Thumb::BlockTransfer { load, rn, list } => {
                let name = if load { "LDMIA" } else { "STMIA" };
                write!(
                    f,
                    "{name} {}!, {}",
                    RegName(rn),
                    RegList { list, extra: None }
                )
            }
            Thumb::BranchCond { cond, offset } => write!(f, "B{cond} {offset:+}"),
            Thumb::Swi { imm } => write!(f, "SWI #{imm}"),
            Thumb::Bkpt { imm } => write!(f, "BKPT #{imm}"),
            Thumb::Branch { offset } => write!(f, "B {offset:+}"),
            Thumb::BranchLinkPrefix { offset } => write!(f, "BL(prefix) {offset:+}"),
            Thumb::BranchLinkSuffix { exchange, offset } => {
                let name = if exchange { "BLX" } else { "BL" };
                write!(f, "{name}(suffix) +{offset}")
            }
            Thumb::Undefined => f.write_str("UNDEFINED"),
        }
    }
}
