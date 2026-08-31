//! The MC68000 instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds one declarative description, [`TABLE`], from which
//! everything else is derived:
//!
//! - the interpreter's decode ([`decode`]), which reads [`Insn::src`],
//!   [`Insn::dst`] and [`Insn::size`] to know where the operands live;
//! - the disassembler ([`super::disasm`]), which formats from the same row;
//! - introspection: mnemonics, one-line summaries, and which encodings are
//!   privileged.
//!
//! # Why the table is patterns and not 65 536 rows
//!
//! A 68000 opcode word carries its operands *inside* the opcode: the register
//! number is bits 11–9, the effective address is bits 5–0, the size is bits
//! 7–6. A dense row-per-encoding table would be 65 536 entries of which all but
//! a few hundred are copies. So a row is a `(mask, value)` pattern plus the
//! *positions* its operands occupy ([`Arg`]), and [`decode`] scans the patterns
//! that share the opcode's top nibble — the 68000 opcode map is organised by
//! that nibble, so the scan is a handful of comparisons.
//!
//! First match wins, so the list is ordered specific-before-general; a test
//! asserts the ordering property the scan depends on.
//!
//! # Why there is no cycle column
//!
//! Deliberate, and for the same reason as the 6502 core: a 68000 cycle count is
//! a property of the *operand*, not of the opcode. `ADD.W (A0),D0` and
//! `ADD.W (d8,A0,Xn),D0` are the same row. The interpreter charges four cycles
//! per bus access and adds the internal cycles the *M68000 User's Manual*
//! section 8 tables call for at the point they happen, which is also what makes
//! the prefetch queue observable rather than notional.
//!
//! # Sources
//!
//! *M68000 Family Programmer's Reference Manual* (Motorola M68000PRM/AD) for
//! the encodings, the condition-code rules and the addressing-mode legality
//! tables; the *MC68000 8-/16-/32-Bit Microprocessors User's Manual*
//! (MC68000UM) section 8 for instruction timing and section 6 for exception
//! processing. `docs/cpu/other.md` records where to find both. No copyleft
//! emulator was consulted.

use core::fmt;

/// An operand width.
///
/// The 68000 spells these `.B`, `.W` and `.L`, and almost every instruction
/// that has a size encodes it in two bits of the opcode — but *which* two bits
/// differs by family, which is what [`SizeSpec`] is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Size {
    /// 8 bits. Only the low byte of a data register takes part.
    Byte,
    /// 16 bits.
    Word,
    /// 32 bits.
    Long,
}

impl Size {
    /// How many bytes the operand occupies in memory.
    #[must_use]
    pub const fn bytes(self) -> u32 {
        match self {
            Size::Byte => 1,
            Size::Word => 2,
            Size::Long => 4,
        }
    }

    /// The mask of the bits the operand actually uses.
    #[must_use]
    pub const fn mask(self) -> u32 {
        match self {
            Size::Byte => 0xff,
            Size::Word => 0xffff,
            Size::Long => 0xffff_ffff,
        }
    }

    /// The sign bit of an operand of this size.
    #[must_use]
    pub const fn sign_bit(self) -> u32 {
        match self {
            Size::Byte => 0x80,
            Size::Word => 0x8000,
            Size::Long => 0x8000_0000,
        }
    }

    /// The assembler suffix: `b`, `w` or `l`.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            Size::Byte => "b",
            Size::Word => "w",
            Size::Long => "l",
        }
    }
}

impl fmt::Display for Size {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.suffix())
    }
}

/// Where in the opcode word the operand size is encoded, if anywhere.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SizeSpec {
    /// The instruction has no size, or only one.
    None,
    /// Always this size, whatever the opcode says.
    Fixed(Size),
    /// Bits 7–6: `00` byte, `01` word, `10` long, `11` invalid.
    ///
    /// The ordinary arithmetic/logic encoding.
    Bits76,
    /// Bit 6: clear word, set long. `MOVEM`, `EXT`, `MOVEP`.
    Bit6,
    /// Bit 8: clear word, set long. The `<ea>,An` forms — `ADDA`, `SUBA`,
    /// `CMPA` — whose opmode field is `011`/`111`.
    Bit8,
    /// Bits 13–12, the `MOVE` encoding: `01` byte, `11` word, `10` long.
    Move,
    /// Byte for a memory destination, long for a data register — the bit
    /// instructions (`BTST`, `BCHG`, `BCLR`, `BSET`), whose operand size is a
    /// property of *where* the bit is (M68000PRM, *BTST*).
    BitOp,
}

impl SizeSpec {
    /// Resolve the size for a concrete opcode word.
    ///
    /// `None` means the encoding is invalid — bits 7–6 = `11` outside the
    /// families that give that combination a meaning — and the caller must
    /// raise an illegal-instruction exception.
    #[must_use]
    pub const fn resolve(self, opcode: u16) -> Option<Size> {
        match self {
            SizeSpec::None => Some(Size::Word),
            SizeSpec::Fixed(size) => Some(size),
            SizeSpec::Bits76 => match (opcode >> 6) & 3 {
                0 => Some(Size::Byte),
                1 => Some(Size::Word),
                2 => Some(Size::Long),
                _ => None,
            },
            SizeSpec::Bit6 => {
                if opcode & 0x0040 == 0 {
                    Some(Size::Word)
                } else {
                    Some(Size::Long)
                }
            }
            SizeSpec::Bit8 => {
                if opcode & 0x0100 == 0 {
                    Some(Size::Word)
                } else {
                    Some(Size::Long)
                }
            }
            SizeSpec::Move => match (opcode >> 12) & 3 {
                1 => Some(Size::Byte),
                3 => Some(Size::Word),
                2 => Some(Size::Long),
                _ => None,
            },
            // Long when the destination is a data register, byte otherwise.
            SizeSpec::BitOp => {
                if (opcode >> 3) & 7 == 0 {
                    Some(Size::Long)
                } else {
                    Some(Size::Byte)
                }
            }
        }
    }
}

/// One of the twelve 68000 addressing modes.
///
/// Decoded from the six-bit effective-address field: three mode bits and three
/// register bits, with mode `111` using the register bits as a sub-mode
/// (M68000PRM §2, *Addressing Capabilities*).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// `Dn` — the operand is a data register.
    DataReg,
    /// `An` — the operand is an address register.
    AddrReg,
    /// `(An)` — address register indirect.
    Indirect,
    /// `(An)+` — postincrement.
    PostInc,
    /// `-(An)` — predecrement.
    PreDec,
    /// `(d16,An)` — indirect with a signed 16-bit displacement.
    Disp16,
    /// `(d8,An,Xn)` — indirect with index and a signed 8-bit displacement.
    Index8,
    /// `(xxx).W` — absolute short, sign-extended to 32 bits.
    AbsShort,
    /// `(xxx).L` — absolute long.
    AbsLong,
    /// `(d16,PC)` — program-counter relative.
    PcDisp16,
    /// `(d8,PC,Xn)` — program-counter relative with index.
    PcIndex8,
    /// `#<data>` — immediate.
    Imm,
}

impl Mode {
    /// Decode a six-bit effective-address field, or `None` if it names no mode.
    ///
    /// Mode `111` has only five defined sub-modes on the 68000; registers 5, 6
    /// and 7 are reserved and decode as an illegal instruction.
    #[must_use]
    pub const fn decode(ea: u16) -> Option<(Mode, u8)> {
        let reg = (ea & 7) as u8;
        let mode = match (ea >> 3) & 7 {
            0 => Mode::DataReg,
            1 => Mode::AddrReg,
            2 => Mode::Indirect,
            3 => Mode::PostInc,
            4 => Mode::PreDec,
            5 => Mode::Disp16,
            6 => Mode::Index8,
            _ => match reg {
                0 => Mode::AbsShort,
                1 => Mode::AbsLong,
                2 => Mode::PcDisp16,
                3 => Mode::PcIndex8,
                4 => Mode::Imm,
                _ => return None,
            },
        };
        Some((mode, reg))
    }

    /// This mode's bit in an [`EaSet`].
    #[must_use]
    pub const fn bit(self) -> u16 {
        1 << (self as u16)
    }

    /// How many extension words the mode needs, for a given operand size.
    ///
    /// Part of the instruction's length, which is why it belongs here rather
    /// than in the interpreter: the disassembler needs the same answer.
    #[must_use]
    pub const fn ext_words(self, size: Size) -> u32 {
        match self {
            Mode::Disp16 | Mode::Index8 | Mode::AbsShort | Mode::PcDisp16 | Mode::PcIndex8 => 1,
            Mode::AbsLong => 2,
            Mode::Imm => match size {
                Size::Long => 2,
                _ => 1,
            },
            _ => 0,
        }
    }

    /// Whether the mode names a memory location rather than a register.
    #[must_use]
    pub const fn is_memory(self) -> bool {
        !matches!(self, Mode::DataReg | Mode::AddrReg | Mode::Imm)
    }
}

/// A set of addressing modes an operand slot accepts.
///
/// The 68000 manual expresses operand legality as overlapping categories —
/// *data*, *memory*, *control*, *alterable* and their intersections — and an
/// encoding outside its category is an illegal instruction, not a don't-care.
/// Modelling that explicitly is what lets [`decode`] reject `MOVE.B A0,D0`
/// (M68000PRM §2.2, *Effective Addressing Mode Categories*).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EaSet(pub u16);

impl EaSet {
    /// Every mode.
    pub const ALL: EaSet = EaSet(0x0fff);
    /// *Data*: everything but `An`.
    pub const DATA: EaSet = EaSet(Self::ALL.0 & !Mode::AddrReg.bit());
    /// *Memory*: everything that is not a register and not immediate.
    pub const MEM: EaSet =
        EaSet(Self::ALL.0 & !(Mode::DataReg.bit() | Mode::AddrReg.bit() | Mode::Imm.bit()));
    /// *Alterable*: everything that can be written, so not PC-relative and not
    /// immediate.
    pub const ALTERABLE: EaSet =
        EaSet(Self::ALL.0 & !(Mode::PcDisp16.bit() | Mode::PcIndex8.bit() | Mode::Imm.bit()));
    /// *Data alterable*.
    pub const DATA_ALT: EaSet = EaSet(Self::DATA.0 & Self::ALTERABLE.0);
    /// *Memory alterable*.
    pub const MEM_ALT: EaSet = EaSet(Self::MEM.0 & Self::ALTERABLE.0);
    /// *Control*: modes that name an address without a size or an autoupdate.
    pub const CONTROL: EaSet = EaSet(
        Mode::Indirect.bit()
            | Mode::Disp16.bit()
            | Mode::Index8.bit()
            | Mode::AbsShort.bit()
            | Mode::AbsLong.bit()
            | Mode::PcDisp16.bit()
            | Mode::PcIndex8.bit(),
    );
    /// *Control alterable*.
    pub const CONTROL_ALT: EaSet = EaSet(Self::CONTROL.0 & Self::ALTERABLE.0);
    /// The `MOVEM` register-to-memory destinations: control alterable plus
    /// `-(An)`.
    pub const MOVEM_TO_MEM: EaSet = EaSet(Self::CONTROL_ALT.0 | Mode::PreDec.bit());
    /// The `MOVEM` memory-to-register sources: control plus `(An)+`.
    pub const MOVEM_TO_REG: EaSet = EaSet(Self::CONTROL.0 | Mode::PostInc.bit());
    /// No mode at all — an operand slot that is not an effective address.
    pub const NONE: EaSet = EaSet(0);

    /// Whether `mode` is in the set.
    #[must_use]
    pub const fn contains(self, mode: Mode) -> bool {
        self.0 & mode.bit() != 0
    }

    /// The same set without `mode`.
    #[must_use]
    pub const fn without(self, mode: Mode) -> EaSet {
        EaSet(self.0 & !mode.bit())
    }
}

/// Where one operand of an instruction lives.
///
/// This is the part of the encoding a dense table cannot express: a 68000
/// operand is a *field position* in the opcode word, not a value. Both the
/// interpreter and the disassembler read these, so an operand the disassembler
/// prints is by construction the one the interpreter used.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Arg {
    /// This slot is unused.
    None,
    /// An effective address in bits 5–0.
    Ea,
    /// `MOVE`'s destination: mode in bits 8–6, register in bits 11–9 — the two
    /// halves swapped relative to every other encoding.
    EaDst,
    /// The data register in bits 11–9.
    DnHi,
    /// The data register in bits 2–0.
    DnLo,
    /// The address register in bits 11–9.
    AnHi,
    /// The address register in bits 2–0.
    AnLo,
    /// `ABCD`/`SBCD`/`ADDX`/`SUBX` source: `Dy` (bits 2–0) when bit 3 is
    /// clear, `-(Ay)` when it is set.
    RmLo,
    /// The same instructions' destination: `Dx` (bits 11–9) or `-(Ax)`.
    RmHi,
    /// `CMPM`'s source `(Ay)+`, register in bits 2–0.
    PostLo,
    /// `CMPM`'s destination `(Ax)+`, register in bits 11–9.
    PostHi,
    /// Immediate extension words, of the instruction's own size.
    Imm,
    /// A bit number, in the low byte of one extension word.
    ///
    /// Distinct from [`Arg::Imm`] because it is *always* one word, while the
    /// bit instructions' operand size is long when they address a data
    /// register. Folding the two together makes the instruction's length
    /// depend on its operand size, which for these four it does not.
    BitNumber,
    /// A quick immediate 1–8 in bits 11–9, where `000` means 8.
    Quick,
    /// `MOVEQ`'s signed byte in bits 7–0.
    QuickByte,
    /// `TRAP`'s vector number in bits 3–0.
    Vector,
    /// `Bcc`'s 8-bit displacement, or the following word when it is zero.
    Disp8,
    /// A signed 16-bit displacement in the next extension word.
    Disp16,
    /// The condition code register — the low byte of `SR`.
    Ccr,
    /// The whole status register.
    Sr,
    /// The user stack pointer.
    Usp,
    /// A shift count: bits 11–9 as 1–8 when bit 5 is clear, `Dn` modulo 64 when
    /// it is set.
    ShiftCount,
    /// `MOVEM`'s register-list mask, in the next extension word.
    RegList,
    /// `MOVEP`'s `(d16,Ay)` operand, register in bits 2–0.
    MovepEa,
}

impl Arg {
    /// Whether this slot is an effective address, and so subject to
    /// [`Insn::src_modes`] / [`Insn::dst_modes`].
    #[must_use]
    pub const fn is_ea(self) -> bool {
        matches!(self, Arg::Ea | Arg::EaDst)
    }
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
macro_rules! define_ops {
    ($($name:ident = $mnemonic:literal, $summary:literal;)*) => {
        /// One operation, independent of how its operands are addressed.
        ///
        /// A variant carries a mnemonic ([`Op::mnemonic`]) so a disassembler
        /// cannot print a name the interpreter does not implement.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum Op {
            $(
                #[doc = $summary]
                $name,
            )*
        }

        impl Op {
            /// The assembler mnemonic, without a size suffix.
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(Op::$name => $mnemonic,)* }
            }

            /// A one-line description, for `rsemu describe` and the monitor.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(Op::$name => $summary,)* }
            }

            /// Every operation this core implements, in declaration order.
            pub const ALL: &'static [Op] = &[$(Op::$name,)*];
        }
    };
}

define_ops! {
    Abcd = "ABCD", "add binary-coded decimal with extend";
    Add = "ADD", "add binary";
    Adda = "ADDA", "add to an address register, without touching the flags";
    Addi = "ADDI", "add an immediate";
    Addq = "ADDQ", "add a quick immediate 1-8";
    Addx = "ADDX", "add with extend";
    And = "AND", "logical AND";
    Andi = "ANDI", "logical AND with an immediate";
    AndiToCcr = "ANDI", "AND an immediate into the condition codes";
    AndiToSr = "ANDI", "AND an immediate into the status register (privileged)";
    Asl = "ASL", "arithmetic shift left";
    Asr = "ASR", "arithmetic shift right";
    Bcc = "B", "branch conditionally";
    Bchg = "BCHG", "test a bit and change it";
    Bclr = "BCLR", "test a bit and clear it";
    Bra = "BRA", "branch always";
    Bset = "BSET", "test a bit and set it";
    Bsr = "BSR", "branch to subroutine";
    Btst = "BTST", "test a bit";
    Chk = "CHK", "check a register against bounds, trapping if outside";
    Clr = "CLR", "clear an operand to zero";
    Cmp = "CMP", "compare";
    Cmpa = "CMPA", "compare with an address register";
    Cmpi = "CMPI", "compare with an immediate";
    Cmpm = "CMPM", "compare memory with memory, postincrementing both";
    Dbcc = "DB", "test a condition, decrement and branch";
    Divs = "DIVS", "signed divide";
    Divu = "DIVU", "unsigned divide";
    Eor = "EOR", "exclusive OR";
    Eori = "EORI", "exclusive OR with an immediate";
    EoriToCcr = "EORI", "exclusive-OR an immediate into the condition codes";
    EoriToSr = "EORI", "exclusive-OR an immediate into the status register (privileged)";
    Exg = "EXG", "exchange two registers";
    Ext = "EXT", "sign-extend a data register";
    Illegal = "ILLEGAL", "take an illegal-instruction exception";
    Jmp = "JMP", "jump";
    Jsr = "JSR", "jump to subroutine";
    Lea = "LEA", "load an effective address";
    LineA = "LINEA", "unimplemented instruction, $A line emulator trap";
    LineF = "LINEF", "unimplemented instruction, $F line emulator trap";
    Link = "LINK", "link and allocate a stack frame";
    Lsl = "LSL", "logical shift left";
    Lsr = "LSR", "logical shift right";
    Move = "MOVE", "move data";
    Movea = "MOVEA", "move data to an address register, without touching the flags";
    MoveFromSr = "MOVE", "move the status register to a destination";
    MoveToCcr = "MOVE", "move a source into the condition codes";
    MoveToSr = "MOVE", "move a source into the status register (privileged)";
    MoveUsp = "MOVE", "move to or from the user stack pointer (privileged)";
    Movem = "MOVEM", "move multiple registers to or from memory";
    Movep = "MOVEP", "move peripheral data, every other byte";
    Moveq = "MOVEQ", "move a sign-extended byte immediate to a data register";
    Muls = "MULS", "signed multiply";
    Mulu = "MULU", "unsigned multiply";
    Nbcd = "NBCD", "negate binary-coded decimal with extend";
    Neg = "NEG", "negate";
    Negx = "NEGX", "negate with extend";
    Nop = "NOP", "no operation";
    Not = "NOT", "ones complement";
    Or = "OR", "logical inclusive OR";
    Ori = "ORI", "logical inclusive OR with an immediate";
    OriToCcr = "ORI", "OR an immediate into the condition codes";
    OriToSr = "ORI", "OR an immediate into the status register (privileged)";
    Pea = "PEA", "push an effective address";
    Reset = "RESET", "assert the reset line (privileged)";
    Rol = "ROL", "rotate left";
    Ror = "ROR", "rotate right";
    Roxl = "ROXL", "rotate left through extend";
    Roxr = "ROXR", "rotate right through extend";
    Rte = "RTE", "return from exception (privileged)";
    Rtr = "RTR", "return and restore the condition codes";
    Rts = "RTS", "return from subroutine";
    Sbcd = "SBCD", "subtract binary-coded decimal with extend";
    Scc = "S", "set a byte to all ones or all zeros on a condition";
    Stop = "STOP", "load the status register and stop (privileged)";
    Sub = "SUB", "subtract binary";
    Suba = "SUBA", "subtract from an address register, without touching the flags";
    Subi = "SUBI", "subtract an immediate";
    Subq = "SUBQ", "subtract a quick immediate 1-8";
    Subx = "SUBX", "subtract with extend";
    Swap = "SWAP", "swap the halves of a data register";
    Tas = "TAS", "test an operand and set its high bit, indivisibly";
    Trap = "TRAP", "take a TRAP #n exception";
    Trapv = "TRAPV", "take an overflow exception if V is set";
    Tst = "TST", "test an operand against zero";
    Unlk = "UNLK", "unlink a stack frame";
}

impl Op {
    /// Whether the mnemonic takes a condition-code suffix from bits 11–8.
    #[must_use]
    pub const fn is_conditional(self) -> bool {
        matches!(self, Op::Bcc | Op::Dbcc | Op::Scc)
    }
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

/// One of the sixteen condition codes, as encoded in bits 11–8.
///
/// `T`/`F` are the always/never pair; on `Bcc` those two encodings mean `BRA`
/// and `BSR` instead, which is why the branch rows name their own operations
/// (M68000PRM §3.2, *Condition Tests*).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cond(pub u8);

impl Cond {
    /// The condition in bits 11–8 of an opcode.
    #[must_use]
    pub const fn from_opcode(opcode: u16) -> Cond {
        Cond(((opcode >> 8) & 0xf) as u8)
    }

    /// The assembler suffix: `T`, `F`, `HI`, `LS`, `CC`, …
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 {
            0x0 => "T",
            0x1 => "F",
            0x2 => "HI",
            0x3 => "LS",
            0x4 => "CC",
            0x5 => "CS",
            0x6 => "NE",
            0x7 => "EQ",
            0x8 => "VC",
            0x9 => "VS",
            0xa => "PL",
            0xb => "MI",
            0xc => "GE",
            0xd => "LT",
            0xe => "GT",
            _ => "LE",
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// One row of the instruction description: everything known about a family of
/// encodings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// Where the operand size comes from.
    pub size: SizeSpec,
    /// Where the source operand lives.
    pub src: Arg,
    /// Where the destination operand lives.
    pub dst: Arg,
    /// Which addressing modes the source accepts, if it is an effective
    /// address.
    pub src_modes: EaSet,
    /// Which addressing modes the destination accepts, if it is an effective
    /// address.
    pub dst_modes: EaSet,
    /// Whether the encoding may only be executed in supervisor state.
    pub privileged: bool,
}

impl Insn {
    /// The row every unassigned encoding decodes to.
    pub const ILLEGAL: Insn = Insn {
        op: Op::Illegal,
        size: SizeSpec::None,
        src: Arg::None,
        dst: Arg::None,
        src_modes: EaSet::NONE,
        dst_modes: EaSet::NONE,
        privileged: false,
    };

    const fn new(op: Op, size: SizeSpec, src: Arg, dst: Arg) -> Insn {
        Insn {
            op,
            size,
            src,
            dst,
            src_modes: EaSet::ALL,
            dst_modes: EaSet::ALL,
            privileged: false,
        }
    }

    const fn src_ea(mut self, modes: EaSet) -> Insn {
        self.src_modes = modes;
        self
    }

    const fn dst_ea(mut self, modes: EaSet) -> Insn {
        self.dst_modes = modes;
        self
    }

    const fn privileged(mut self) -> Insn {
        self.privileged = true;
        self
    }
}

/// The encoding that *is* the `ILLEGAL` instruction.
///
/// Every unassigned word takes the same exception, so [`decode`] gives them
/// all the same row — but a disassembler needs to tell a deliberate `ILLEGAL`
/// from a word that happens to decode to nothing, and this is the one place
/// that distinction lives.
pub const ILLEGAL_OPCODE: u16 = 0x4afc;

/// A `(mask, value)` pattern and the row it selects.
#[derive(Debug, Clone, Copy)]
pub struct Pattern {
    /// Which opcode bits the pattern constrains.
    pub mask: u16,
    /// What those bits must equal.
    pub value: u16,
    /// The row this encoding belongs to.
    pub insn: Insn,
}

impl Pattern {
    /// Whether `opcode` matches.
    #[inline]
    #[must_use]
    pub const fn matches(&self, opcode: u16) -> bool {
        opcode & self.mask == self.value
    }
}

macro_rules! table {
    ($($mask:literal $value:literal => $insn:expr;)*) => {
        /// The instruction description: one row per encoding family, ordered
        /// specific before general, and grouped by the opcode's top nibble.
        ///
        /// This is the only description of the instruction set in the crate.
        pub static TABLE: &[Pattern] = &[
            $(Pattern { mask: $mask, value: $value, insn: $insn },)*
        ];
    };
}

use Arg::{
    AnHi, AnLo, BitNumber, Ccr, Disp8, Disp16, DnHi, DnLo, Ea, EaDst, Imm, MovepEa, PostHi, PostLo,
    Quick, QuickByte, RegList, RmHi, RmLo, ShiftCount, Sr, Usp, Vector,
};
use Size::{Byte, Long, Word};
use SizeSpec::{Bit6, Bit8, BitOp, Bits76, Fixed, Move as MoveSize};

table! {
    // ---- line 0: immediates, static bit operations, MOVEP ----------------
    0xffff 0x003c => Insn::new(Op::OriToCcr,  Fixed(Byte), Imm, Ccr);
    0xffff 0x007c => Insn::new(Op::OriToSr,   Fixed(Word), Imm, Sr).privileged();
    0xffff 0x023c => Insn::new(Op::AndiToCcr, Fixed(Byte), Imm, Ccr);
    0xffff 0x027c => Insn::new(Op::AndiToSr,  Fixed(Word), Imm, Sr).privileged();
    0xffff 0x0a3c => Insn::new(Op::EoriToCcr, Fixed(Byte), Imm, Ccr);
    0xffff 0x0a7c => Insn::new(Op::EoriToSr,  Fixed(Word), Imm, Sr).privileged();
    // MOVEP shares bit 8 with the dynamic bit instructions and is told apart
    // by its mode field being 001, which those forbid. Bit 7 is the direction,
    // and it gets a row of its own rather than a runtime test, so the
    // disassembler cannot print the operands the wrong way round.
    0xf1b8 0x0108 => Insn::new(Op::Movep,     Bit6, MovepEa, DnHi);
    0xf1b8 0x0188 => Insn::new(Op::Movep,     Bit6, DnHi, MovepEa);
    0xff00 0x0000 => Insn::new(Op::Ori,       Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x0200 => Insn::new(Op::Andi,      Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x0400 => Insn::new(Op::Subi,      Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x0600 => Insn::new(Op::Addi,      Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x0800 => Insn::new(Op::Btst,      BitOp, BitNumber, Ea)
                        .dst_ea(EaSet::DATA.without(Mode::Imm));
    0xffc0 0x0840 => Insn::new(Op::Bchg,      BitOp, BitNumber, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x0880 => Insn::new(Op::Bclr,      BitOp, BitNumber, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x08c0 => Insn::new(Op::Bset,      BitOp, BitNumber, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x0a00 => Insn::new(Op::Eori,      Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x0c00 => Insn::new(Op::Cmpi,      Bits76, Imm, Ea).dst_ea(EaSet::DATA_ALT);
    0xf1c0 0x0100 => Insn::new(Op::Btst,      BitOp, DnHi, Ea).dst_ea(EaSet::DATA);
    0xf1c0 0x0140 => Insn::new(Op::Bchg,      BitOp, DnHi, Ea).dst_ea(EaSet::DATA_ALT);
    0xf1c0 0x0180 => Insn::new(Op::Bclr,      BitOp, DnHi, Ea).dst_ea(EaSet::DATA_ALT);
    0xf1c0 0x01c0 => Insn::new(Op::Bset,      BitOp, DnHi, Ea).dst_ea(EaSet::DATA_ALT);

    // ---- lines 1-3: MOVE and MOVEA --------------------------------------
    // A byte MOVE has no address-register operand at either end: there is no
    // MOVEA.B, and `An` is not a byte-addressable source.
    0xf000 0x1000 => Insn::new(Op::Move,  MoveSize, Ea, EaDst)
                        .src_ea(EaSet::DATA).dst_ea(EaSet::DATA_ALT);
    0xf1c0 0x2040 => Insn::new(Op::Movea, MoveSize, Ea, AnHi).src_ea(EaSet::ALL);
    0xf000 0x2000 => Insn::new(Op::Move,  MoveSize, Ea, EaDst)
                        .src_ea(EaSet::ALL).dst_ea(EaSet::DATA_ALT);
    0xf1c0 0x3040 => Insn::new(Op::Movea, MoveSize, Ea, AnHi).src_ea(EaSet::ALL);
    0xf000 0x3000 => Insn::new(Op::Move,  MoveSize, Ea, EaDst)
                        .src_ea(EaSet::ALL).dst_ea(EaSet::DATA_ALT);

    // ---- line 4: the miscellaneous group --------------------------------
    0xffff 0x4afc => Insn::new(Op::Illegal, SizeSpec::None, Arg::None, Arg::None);
    0xffff 0x4e70 => Insn::new(Op::Reset,   SizeSpec::None, Arg::None, Arg::None).privileged();
    0xffff 0x4e71 => Insn::new(Op::Nop,     SizeSpec::None, Arg::None, Arg::None);
    // The destination is SR and the summary says so; the slot is left empty
    // because no assembler writes `STOP #$2700,SR` and the disassembler prints
    // what the slots hold.
    0xffff 0x4e72 => Insn::new(Op::Stop,    Fixed(Word), Imm, Arg::None).privileged();
    0xffff 0x4e73 => Insn::new(Op::Rte,     SizeSpec::None, Arg::None, Arg::None).privileged();
    0xffff 0x4e75 => Insn::new(Op::Rts,     SizeSpec::None, Arg::None, Arg::None);
    0xffff 0x4e76 => Insn::new(Op::Trapv,   SizeSpec::None, Arg::None, Arg::None);
    0xffff 0x4e77 => Insn::new(Op::Rtr,     SizeSpec::None, Arg::None, Arg::None);
    0xfff0 0x4e40 => Insn::new(Op::Trap,    SizeSpec::None, Vector, Arg::None);
    0xfff8 0x4e50 => Insn::new(Op::Link,    Fixed(Word), AnLo, Disp16);
    0xfff8 0x4e58 => Insn::new(Op::Unlk,    Fixed(Long), AnLo, Arg::None);
    0xfff8 0x4e60 => Insn::new(Op::MoveUsp, Fixed(Long), AnLo, Usp).privileged();
    0xfff8 0x4e68 => Insn::new(Op::MoveUsp, Fixed(Long), Usp, AnLo).privileged();
    0xfff8 0x4840 => Insn::new(Op::Swap,    Fixed(Word), DnLo, Arg::None);
    0xfff8 0x4880 => Insn::new(Op::Ext,     Bit6, DnLo, Arg::None);
    0xfff8 0x48c0 => Insn::new(Op::Ext,     Bit6, DnLo, Arg::None);
    0xffc0 0x40c0 => Insn::new(Op::MoveFromSr, Fixed(Word), Sr, Ea)
                        .dst_ea(EaSet::DATA_ALT);
    0xffc0 0x44c0 => Insn::new(Op::MoveToCcr,  Fixed(Word), Ea, Ccr)
                        .src_ea(EaSet::DATA);
    0xffc0 0x46c0 => Insn::new(Op::MoveToSr,   Fixed(Word), Ea, Sr)
                        .src_ea(EaSet::DATA).privileged();
    0xffc0 0x4800 => Insn::new(Op::Nbcd, Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x4840 => Insn::new(Op::Pea,  Fixed(Long), Ea, Arg::None).src_ea(EaSet::CONTROL);
    0xffc0 0x4ac0 => Insn::new(Op::Tas,  Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x4e80 => Insn::new(Op::Jsr,  SizeSpec::None, Ea, Arg::None).src_ea(EaSet::CONTROL);
    0xffc0 0x4ec0 => Insn::new(Op::Jmp,  SizeSpec::None, Ea, Arg::None).src_ea(EaSet::CONTROL);
    0xff80 0x4880 => Insn::new(Op::Movem, Bit6, RegList, Ea).dst_ea(EaSet::MOVEM_TO_MEM);
    0xff80 0x4c80 => Insn::new(Op::Movem, Bit6, Ea, RegList).src_ea(EaSet::MOVEM_TO_REG);
    0xff00 0x4000 => Insn::new(Op::Negx, Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4200 => Insn::new(Op::Clr,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4400 => Insn::new(Op::Neg,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4600 => Insn::new(Op::Not,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4a00 => Insn::new(Op::Tst,  Bits76, Ea, Arg::None).src_ea(EaSet::DATA_ALT);
    0xf1c0 0x4180 => Insn::new(Op::Chk,  Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf1c0 0x41c0 => Insn::new(Op::Lea,  Fixed(Long), Ea, AnHi).src_ea(EaSet::CONTROL);

    // ---- line 5: ADDQ, SUBQ, Scc, DBcc ----------------------------------
    0xf0f8 0x50c8 => Insn::new(Op::Dbcc, Fixed(Word), DnLo, Disp16);
    0xf0c0 0x50c0 => Insn::new(Op::Scc,  Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xf100 0x5000 => Insn::new(Op::Addq, Bits76, Quick, Ea).dst_ea(EaSet::ALTERABLE);
    0xf100 0x5100 => Insn::new(Op::Subq, Bits76, Quick, Ea).dst_ea(EaSet::ALTERABLE);

    // ---- line 6: branches -----------------------------------------------
    0xff00 0x6000 => Insn::new(Op::Bra, SizeSpec::None, Disp8, Arg::None);
    0xff00 0x6100 => Insn::new(Op::Bsr, SizeSpec::None, Disp8, Arg::None);
    0xf000 0x6000 => Insn::new(Op::Bcc, SizeSpec::None, Disp8, Arg::None);

    // ---- line 7: MOVEQ ---------------------------------------------------
    0xf100 0x7000 => Insn::new(Op::Moveq, Fixed(Long), QuickByte, DnHi);

    // ---- line 8: OR, DIV, SBCD ------------------------------------------
    0xf1f0 0x8100 => Insn::new(Op::Sbcd, Fixed(Byte), RmLo, RmHi);
    0xf1c0 0x80c0 => Insn::new(Op::Divu, Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf1c0 0x81c0 => Insn::new(Op::Divs, Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf100 0x8000 => Insn::new(Op::Or,   Bits76, Ea, DnHi).src_ea(EaSet::DATA);
    0xf100 0x8100 => Insn::new(Op::Or,   Bits76, DnHi, Ea).dst_ea(EaSet::MEM_ALT);

    // ---- line 9: SUB, SUBX, SUBA ----------------------------------------
    0xf0c0 0x90c0 => Insn::new(Op::Suba, Bit8, Ea, AnHi).src_ea(EaSet::ALL);
    0xf130 0x9100 => Insn::new(Op::Subx, Bits76, RmLo, RmHi);
    0xf100 0x9000 => Insn::new(Op::Sub,  Bits76, Ea, DnHi).src_ea(EaSet::ALL);
    0xf100 0x9100 => Insn::new(Op::Sub,  Bits76, DnHi, Ea).dst_ea(EaSet::MEM_ALT);

    // ---- line a: unimplemented, the $A line emulator ---------------------
    0xf000 0xa000 => Insn::new(Op::LineA, SizeSpec::None, Arg::None, Arg::None);

    // ---- line b: CMP, CMPA, CMPM, EOR -----------------------------------
    0xf0c0 0xb0c0 => Insn::new(Op::Cmpa, Bit8, Ea, AnHi).src_ea(EaSet::ALL);
    0xf138 0xb108 => Insn::new(Op::Cmpm, Bits76, PostLo, PostHi);
    0xf100 0xb000 => Insn::new(Op::Cmp,  Bits76, Ea, DnHi).src_ea(EaSet::ALL);
    0xf100 0xb100 => Insn::new(Op::Eor,  Bits76, DnHi, Ea).dst_ea(EaSet::DATA_ALT);

    // ---- line c: AND, MUL, ABCD, EXG -------------------------------------
    0xf1f0 0xc100 => Insn::new(Op::Abcd, Fixed(Byte), RmLo, RmHi);
    0xf1f8 0xc140 => Insn::new(Op::Exg,  Fixed(Long), DnHi, DnLo);
    0xf1f8 0xc148 => Insn::new(Op::Exg,  Fixed(Long), AnHi, AnLo);
    0xf1f8 0xc188 => Insn::new(Op::Exg,  Fixed(Long), DnHi, AnLo);
    0xf1c0 0xc0c0 => Insn::new(Op::Mulu, Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf1c0 0xc1c0 => Insn::new(Op::Muls, Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf100 0xc000 => Insn::new(Op::And,  Bits76, Ea, DnHi).src_ea(EaSet::DATA);
    0xf100 0xc100 => Insn::new(Op::And,  Bits76, DnHi, Ea).dst_ea(EaSet::MEM_ALT);

    // ---- line d: ADD, ADDX, ADDA ----------------------------------------
    0xf0c0 0xd0c0 => Insn::new(Op::Adda, Bit8, Ea, AnHi).src_ea(EaSet::ALL);
    0xf130 0xd100 => Insn::new(Op::Addx, Bits76, RmLo, RmHi);
    0xf100 0xd000 => Insn::new(Op::Add,  Bits76, Ea, DnHi).src_ea(EaSet::ALL);
    0xf100 0xd100 => Insn::new(Op::Add,  Bits76, DnHi, Ea).dst_ea(EaSet::MEM_ALT);

    // ---- line e: shifts and rotates --------------------------------------
    // The memory forms shift one bit of one word and must be matched first:
    // they occupy the bits-7-6 = 11 encoding the register forms leave unused.
    0xffc0 0xe0c0 => Insn::new(Op::Asr,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe1c0 => Insn::new(Op::Asl,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe2c0 => Insn::new(Op::Lsr,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe3c0 => Insn::new(Op::Lsl,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe4c0 => Insn::new(Op::Roxr, Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe5c0 => Insn::new(Op::Roxl, Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe6c0 => Insn::new(Op::Ror,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xffc0 0xe7c0 => Insn::new(Op::Rol,  Fixed(Word), Arg::None, Ea).dst_ea(EaSet::MEM_ALT);
    0xf118 0xe000 => Insn::new(Op::Asr,  Bits76, ShiftCount, DnLo);
    0xf118 0xe100 => Insn::new(Op::Asl,  Bits76, ShiftCount, DnLo);
    0xf118 0xe008 => Insn::new(Op::Lsr,  Bits76, ShiftCount, DnLo);
    0xf118 0xe108 => Insn::new(Op::Lsl,  Bits76, ShiftCount, DnLo);
    0xf118 0xe010 => Insn::new(Op::Roxr, Bits76, ShiftCount, DnLo);
    0xf118 0xe110 => Insn::new(Op::Roxl, Bits76, ShiftCount, DnLo);
    0xf118 0xe018 => Insn::new(Op::Ror,  Bits76, ShiftCount, DnLo);
    0xf118 0xe118 => Insn::new(Op::Rol,  Bits76, ShiftCount, DnLo);

    // ---- line f: unimplemented, the $F line coprocessor escape ------------
    0xf000 0xf000 => Insn::new(Op::LineF, SizeSpec::None, Arg::None, Arg::None);
}

/// Where each top nibble's patterns start in [`TABLE`], and where they end.
///
/// Every pattern constrains bits 15–12, so a scan only ever has to look at the
/// rows for the opcode's own nibble — a handful of comparisons rather than a
/// hundred. Built from `TABLE` itself, so adding a row cannot forget to update
/// it.
static NIBBLE: [(u16, u16); 16] = {
    let mut spans = [(0u16, 0u16); 16];
    let mut i = 0;
    while i < TABLE.len() {
        let n = (TABLE[i].value >> 12) as usize;
        if spans[n].0 == 0 && spans[n].1 == 0 {
            spans[n].0 = i as u16;
        }
        spans[n].1 = i as u16 + 1;
        i += 1;
    }
    spans
};

/// Decode an opcode word into its table row.
///
/// Unassigned encodings, illegal size fields and illegal addressing modes all
/// return [`Insn::ILLEGAL`] — on a 68000 those are the same thing, an
/// illegal-instruction exception through vector 4 — with the sole exception of
/// the `$A` and `$F` lines, which have their own vectors and their own rows.
#[inline]
#[must_use]
pub fn decode(opcode: u16) -> Insn {
    let (start, end) = NIBBLE[(opcode >> 12) as usize];
    let mut i = start as usize;
    while i < end as usize {
        let pattern = &TABLE[i];
        if pattern.matches(opcode) {
            let insn = pattern.insn;
            return if legal(insn, opcode) {
                insn
            } else {
                Insn::ILLEGAL
            };
        }
        i += 1;
    }
    Insn::ILLEGAL
}

/// Whether a matched row's operands are actually encodable for this opcode.
///
/// A `(mask, value)` pattern cannot express "bits 7–6 may not be 11" or
/// "`An` is not a byte operand"; those are the manual's addressing-mode
/// category tables, and they are what separates a real instruction from an
/// illegal one.
fn legal(insn: Insn, opcode: u16) -> bool {
    let Some(size) = insn.size.resolve(opcode) else {
        return false;
    };
    if insn.src.is_ea() && !ea_ok(insn.src, insn.src_modes, size, opcode) {
        return false;
    }
    if insn.dst.is_ea() && !ea_ok(insn.dst, insn.dst_modes, size, opcode) {
        return false;
    }
    // ADDQ/SUBQ reach An, but not as bytes: there is no byte operation on an
    // address register anywhere in the instruction set (M68000PRM, ADDQ).
    if matches!(insn.op, Op::Addq | Op::Subq)
        && size == Size::Byte
        && matches!(Mode::decode(opcode & 0x3f), Some((Mode::AddrReg, _)))
    {
        return false;
    }
    true
}

fn ea_ok(arg: Arg, allowed: EaSet, size: Size, opcode: u16) -> bool {
    let Some((mode, _)) = ea_of(arg, opcode) else {
        return false;
    };
    if !allowed.contains(mode) {
        return false;
    }
    // There is no byte-sized address-register operand.
    !(size == Size::Byte && mode == Mode::AddrReg)
}

/// The effective-address field a given operand slot reads.
///
/// Returns the mode and register number, or `None` when the slot is not an
/// effective address or names no mode. Shared by the interpreter and the
/// disassembler so `MOVE`'s swapped destination halves are decoded in exactly
/// one place.
#[must_use]
pub fn ea_of(arg: Arg, opcode: u16) -> Option<(Mode, u8)> {
    let field = match arg {
        Arg::Ea => opcode & 0x3f,
        Arg::EaDst => ((opcode >> 9) & 7) | ((opcode >> 3) & 0x38),
        _ => return None,
    };
    Mode::decode(field)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_pattern_constrains_the_top_nibble() {
        // The nibble index is what makes decode a short scan; a pattern that
        // left bits 15-12 free would be invisible to it.
        for pattern in TABLE {
            assert_eq!(
                pattern.mask & 0xf000,
                0xf000,
                "pattern {:04x}/{:04x} leaves the top nibble free",
                pattern.mask,
                pattern.value
            );
        }
    }

    #[test]
    fn patterns_are_grouped_by_nibble() {
        // NIBBLE assumes each nibble's rows are contiguous; if they are not,
        // decode silently stops looking part way through.
        let mut seen = [false; 16];
        let mut previous = usize::MAX;
        for pattern in TABLE {
            let n = (pattern.value >> 12) as usize;
            if n != previous {
                assert!(!seen[n], "nibble {n:x} is split into two runs");
                seen[n] = true;
                previous = n;
            }
        }
    }

    #[test]
    fn nibble_spans_cover_the_table() {
        let total: usize = NIBBLE.iter().map(|(a, b)| (b - a) as usize).sum();
        assert_eq!(total, TABLE.len());
    }

    #[test]
    fn known_encodings_decode() {
        assert_eq!(decode(0x4e71).op, Op::Nop);
        assert_eq!(decode(0x4e75).op, Op::Rts);
        assert_eq!(decode(0x4afc).op, Op::Illegal);
        assert_eq!(decode(0x7042).op, Op::Moveq);
        assert_eq!(decode(0x3040).op, Op::Movea);
        assert_eq!(decode(0x3000).op, Op::Move);
        assert_eq!(decode(0x4e40).op, Op::Trap);
        assert_eq!(decode(0xd041).op, Op::Add);
        assert_eq!(decode(0xd0c1).op, Op::Adda);
        assert_eq!(decode(0xd101).op, Op::Addx);
        assert_eq!(decode(0xe000).op, Op::Asr);
        assert_eq!(decode(0xe0d0).op, Op::Asr);
        assert_eq!(decode(0xa000).op, Op::LineA);
        assert_eq!(decode(0xf000).op, Op::LineF);
        assert_eq!(decode(0x0108).op, Op::Movep);
        assert_eq!(decode(0x0100).op, Op::Btst);
        assert_eq!(decode(0x48c0).op, Op::Ext);
        assert_eq!(decode(0x48d0).op, Op::Movem);
        assert_eq!(decode(0x4840).op, Op::Swap);
        assert_eq!(decode(0x4850).op, Op::Pea);
    }

    #[test]
    fn illegal_encodings_are_rejected() {
        // Bits 7-6 = 11 is not a size.
        assert_eq!(decode(0x00c0).op, Op::Illegal);
        // MOVE.B has no address-register source or destination.
        assert_eq!(decode(0x1008).op, Op::Illegal);
        assert_eq!(decode(0x1040).op, Op::Illegal);
        // Mode 7, register 5 names nothing.
        assert_eq!(decode(0x303d).op, Op::Illegal);
        // ADDQ.B to An does not exist.
        assert_eq!(decode(0x5008).op, Op::Illegal);
        // TST does not reach an address register on a 68000.
        assert_eq!(decode(0x4a48).op, Op::Illegal);
    }

    #[test]
    fn privileged_encodings_are_marked() {
        for opcode in [0x027c, 0x46c0, 0x4e70, 0x4e72, 0x4e73, 0x4e60, 0x4e68] {
            assert!(
                decode(opcode).privileged,
                "{opcode:04x} should be privileged"
            );
        }
        assert!(!decode(0x023c).privileged, "ANDI to CCR is not privileged");
    }

    #[test]
    fn the_opcode_map_has_the_shape_the_manual_gives_it() {
        // A tripwire on the whole table: a pattern that starts shadowing
        // another, or a legality rule that stops rejecting something, moves
        // these counts. The numbers themselves are only meaningful as a
        // baseline — what matters is that they do not drift silently.
        let mut legal = 0usize;
        let mut line_a = 0usize;
        let mut line_f = 0usize;
        let mut ops = alloc::collections::BTreeSet::new();
        for opcode in 0..=u16::MAX {
            match decode(opcode).op {
                Op::Illegal => {}
                Op::LineA => line_a += 1,
                Op::LineF => line_f += 1,
                other => {
                    legal += 1;
                    ops.insert(other.mnemonic());
                }
            }
        }
        assert_eq!(line_a, 0x1000, "the whole $A line traps");
        assert_eq!(line_f, 0x1000, "and the whole $F line");
        assert_eq!(legal, 45_815);
        // Every operation in the table is reachable from some encoding.
        for op in Op::ALL {
            // The three that decode to something other than an operation with
            // operands are counted above rather than collected here.
            if matches!(op, Op::Illegal | Op::LineA | Op::LineF) {
                continue;
            }
            assert!(
                ops.contains(op.mnemonic()),
                "{op:?} is in the table but no encoding reaches it"
            );
        }
    }

    #[test]
    fn decode_never_panics() {
        for opcode in 0..=u16::MAX {
            let insn = decode(opcode);
            // A decoded row must resolve a size, or it should have been
            // rejected as illegal.
            if insn.op != Op::Illegal {
                assert!(insn.size.resolve(opcode).is_some(), "{opcode:04x}");
            }
        }
    }
}
