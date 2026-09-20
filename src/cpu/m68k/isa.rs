//! The MC68000, MC68010 and MC68020 instruction sets, described **once**.
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
//! # One table, three processors
//!
//! A row names the processors that implement it ([`Models`]), and
//! [`decode_for`] skips a row the model in hand does not have — so a 68000
//! still sees `$4e7a` as an illegal instruction while a 68010 sees `MOVEC`,
//! and `MOVE from SR` is two rows, one unprivileged for the 68000 and one
//! privileged for everything after it (M68000PRM Appendix A, Table A-1, note
//! 4). The rows a later processor *adds* in an encoding an earlier one left
//! illegal must come before the row that would otherwise claim it and then
//! reject it as illegal: `CAS.L` lives in the `11` size field of `MOVES`,
//! `CHK2` in that of `ORI`, the bit-field instructions in that of the register
//! shifts.
//!
//! Some 68020 instructions carry a word of their own before any effective
//! address extension — `MULS.L`'s register pair, a bit field's offset and
//! width, `MOVEC`'s control register. [`Insn::ext`] counts them, and both the
//! interpreter and the disassembler consume them first.
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
//! *M68000 Family Programmer's Reference Manual* (Motorola M68000PM/AD) for
//! the encodings, the condition-code rules and the addressing-mode legality
//! tables — Section 8's instruction format summary for every encoding, and
//! Appendix A's Table A-1 for which processor has which instruction; the
//! *MC68000 8-/16-/32-Bit Microprocessors User's Manual* (MC68000UM) section 8
//! for instruction timing and section 6 for exception processing; the
//! *MC68020 User's Manual* (MC68020UM) for the 68020. `docs/cpu/m68k.md`
//! records where to find them. No copyleft emulator was consulted.

use core::fmt;

/// Which member of the family a core is.
///
/// A construction property, never a `#[cfg]`: a machine picks its processor
/// in its `.machine` file, and one build has to run an Amiga 500 and an Amiga
/// 1200 side by side.
///
/// Exhaustive on purpose — every place that asks "which processor" should be
/// made to answer again when a 68030 arrives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub enum Model {
    /// The MC68000: 24 address pins, a 16-bit bus, the group-0 frame without a
    /// format word.
    #[default]
    M68000,
    /// The MC68010: the 68000 plus a vector base register, `MOVEC`/`MOVES`,
    /// `RTD`, a privileged `MOVE from SR`, and format words on every stack
    /// frame (MC68000UM §1.3, §6.2.4).
    M68010,
    /// The MC68020: 32-bit addressing, the full extension-word addressing
    /// modes, bit fields, 32-bit multiply and divide, a master stack pointer
    /// and an instruction cache (MC68020UM §1).
    M68020,
    /// The MC68EC020: a 68020 with only 24 address pins (MC68020UM §1:
    /// "the MC68EC020 ... 24-bit address bus"). Same instruction set.
    M68EC020,
    /// The MC68030: the 68020's instruction set without `CALLM`/`RTM`, plus
    /// an on-chip paged memory management unit and its four instructions
    /// (MC68030UM §1.1, §9).
    M68030,
    /// The MC68EC030: a 68030 with **no paged MMU** — the two transparent
    /// translation registers survive as `AC0`/`AC1` and the MMU status
    /// register as `ACUSR`, and `PLOAD`, `PFLUSH` and `PMOVE` to `TC`, `CRP`
    /// or `SRP` are unimplemented F-line instructions (MC68EC030UM §9,
    /// Appendix A). The address bus is the 68030's full 32 bits.
    M68EC030,
}

impl Model {
    /// Every model, in order of introduction.
    pub const ALL: [Model; 6] = [
        Model::M68000,
        Model::M68010,
        Model::M68020,
        Model::M68EC020,
        Model::M68030,
        Model::M68EC030,
    ];

    /// The name the `model` property spells it with.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Model::M68000 => "68000",
            Model::M68010 => "68010",
            Model::M68020 => "68020",
            Model::M68EC020 => "68ec020",
            Model::M68030 => "68030",
            Model::M68EC030 => "68ec030",
        }
    }

    /// The model a `model` property value names.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Model> {
        Model::ALL.into_iter().find(|m| m.name() == name)
    }

    /// Which bits of an address reach the pins.
    ///
    /// 24 on the 68000, the 68010 and the 68EC020; 32 on the full 68020 and
    /// on both 68030 packages — the MC68EC030 keeps the whole address bus and
    /// drops only the MMU (MC68EC030UM §1). Applied to the address *after* it
    /// is computed in 32 bits, which is where the wrap happens on the real
    /// part too.
    #[must_use]
    pub const fn address_mask(self) -> u32 {
        match self {
            Model::M68020 | Model::M68030 | Model::M68EC030 => 0xffff_ffff,
            _ => 0x00ff_ffff,
        }
    }

    /// The 68010's architecture or later: `VBR`, format words, `MOVEC`.
    #[inline]
    #[must_use]
    pub const fn has_010(self) -> bool {
        !matches!(self, Model::M68000)
    }

    /// The 68020's architecture or later: either package of either part.
    #[inline]
    #[must_use]
    pub const fn has_020(self) -> bool {
        matches!(
            self,
            Model::M68020 | Model::M68EC020 | Model::M68030 | Model::M68EC030
        )
    }

    /// The 68030's architecture: either package.
    #[inline]
    #[must_use]
    pub const fn has_030(self) -> bool {
        matches!(self, Model::M68030 | Model::M68EC030)
    }

    /// Whether this part has the paged memory management unit — the one thing
    /// the MC68EC030 leaves out (MC68EC030UM §1.1).
    #[inline]
    #[must_use]
    pub const fn has_mmu(self) -> bool {
        matches!(self, Model::M68030)
    }

    /// Whether a coprocessor may be attached through the F-line interface,
    /// which arrived with the 68020 (MC68020UM §7).
    #[inline]
    #[must_use]
    pub const fn has_coprocessor_interface(self) -> bool {
        self.has_020()
    }

    /// This model's bit in a [`Models`] set.
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Model::M68000 => 1,
            Model::M68010 => 2,
            Model::M68020 | Model::M68EC020 => 4,
            Model::M68030 => 8,
            Model::M68EC030 => 16,
        }
    }
}

impl fmt::Display for Model {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The processors that implement one row of the table.
///
/// The 68EC020 is a 68020 as far as the instruction set is concerned
/// (M68000PRM Appendix A: "All references to the MC68000, MC68020, and
/// MC68030 include references to the corresponding embedded controllers").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Models(pub u8);

impl Models {
    /// Every processor.
    pub const ALL: Models = Models(31);
    /// The 68000 alone — a behaviour the 68010 changed.
    pub const M68000: Models = Models(1);
    /// The 68010 and everything after it.
    pub const FROM_010: Models = Models(30);
    /// The 68020 alone — `CALLM` and `RTM`, which the 68030 dropped
    /// (MC68030UM §1.1: the module support instructions are not implemented).
    pub const M68020: Models = Models(4);
    /// The 68020 and everything after it.
    pub const FROM_020: Models = Models(28);
    /// Both 68030 packages.
    pub const FROM_030: Models = Models(24);
    /// The full MC68030 alone — the encodings that need the paged MMU.
    pub const M68030: Models = Models(8);
    /// The 68000 and the 68010 — a behaviour the 68020 changed.
    pub const UNTIL_010: Models = Models(3);

    /// Whether `model` implements the row.
    #[inline]
    #[must_use]
    pub const fn contains(self, model: Model) -> bool {
        self.0 & model.bit() != 0
    }
}

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
    /// Bits 10–9: `00` byte, `01` word, `10` long — `CHK2` and `CMP2`
    /// (M68000PRM, *CHK2*). `11` is `CALLM`/`RTM`, which have rows of their
    /// own ahead of this one.
    Bits109,
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
            SizeSpec::Bits109 => match (opcode >> 9) & 3 {
                0 => Some(Size::Byte),
                1 => Some(Size::Word),
                2 => Some(Size::Long),
                _ => None,
            },
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
    ///
    /// On a 68020 the same mode field also introduces the *full* extension
    /// word — base and outer displacements, memory indirection, suppressed
    /// base or index — when bit 8 of the first extension word is set
    /// (M68000PRM §2.2.3); a 68000 ignores that bit.
    Index8,
    /// `(xxx).W` — absolute short, sign-extended to 32 bits.
    AbsShort,
    /// `(xxx).L` — absolute long.
    AbsLong,
    /// `(d16,PC)` — program-counter relative.
    PcDisp16,
    /// `(d8,PC,Xn)` — program-counter relative with index, and on a 68020
    /// the full-format PC-relative modes as for [`Mode::Index8`].
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
    ///
    /// For the indexed modes this is the brief format's one word; on a 68020
    /// the first word may introduce more, which [`index_ext_words`] counts.
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
    /// A bit field's source: a data register or a control mode
    /// (M68000PRM, *BFTST*).
    pub const BITFIELD: EaSet = EaSet(Self::CONTROL.0 | Mode::DataReg.bit());
    /// A bit field that is written: a data register or a control alterable
    /// mode (M68000PRM, *BFCHG*).
    pub const BITFIELD_ALT: EaSet = EaSet(Self::CONTROL_ALT.0 | Mode::DataReg.bit());

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
    /// A signed 32-bit displacement in the next two extension words — `LINK.L`
    /// and the 68020's `Bcc.L`.
    Disp32,
    /// `BKPT`'s breakpoint number in bits 2–0.
    Vector3,
    /// `MOVEC`'s control register, in bits 11–0 of its extension word.
    Ctrl,
    /// A general register named by bits 15–12 of the instruction's leading
    /// extension word: `MOVEC`, `MOVES`, `CHK2`, `CMP2`.
    ExtReg,
    /// `TRAPcc`'s optional operand: none, a word or a long, by bits 2–0.
    TrapData,
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
    // ---- the 68010's additions --------------------------------------------
    Bkpt = "BKPT", "breakpoint: an acknowledge cycle, else an illegal-instruction exception";
    MoveFromCcr = "MOVE", "move the condition codes to a destination";
    Movec = "MOVEC", "move to or from a control register (privileged)";
    Moves = "MOVES", "move to or from the address space SFC or DFC names (privileged)";
    Rtd = "RTD", "return and deallocate parameters";
    // ---- the 68020's additions --------------------------------------------
    Bfchg = "BFCHG", "test a bit field and complement it";
    Bfclr = "BFCLR", "test a bit field and clear it";
    Bfexts = "BFEXTS", "extract a bit field, sign-extended";
    Bfextu = "BFEXTU", "extract a bit field, zero-extended";
    Bfffo = "BFFFO", "find the first one in a bit field";
    Bfins = "BFINS", "insert a bit field";
    Bfset = "BFSET", "test a bit field and set it";
    Bftst = "BFTST", "test a bit field";
    Callm = "CALLM", "call a module through its descriptor";
    Cas = "CAS", "compare and swap, indivisibly";
    Cas2 = "CAS2", "compare and swap two operands, indivisibly";
    Cmp2 = "CMP2", "compare a register against a bounds pair (CHK2: and trap)";
    Divl = "DIV", "32-bit divide: DIVS.L, DIVU.L, DIVSL.L, DIVUL.L";
    Extb = "EXTB", "sign-extend a byte to a long";
    Mull = "MUL", "32-bit multiply: MULS.L, MULU.L";
    Pack = "PACK", "pack two unpacked BCD digits, with an adjustment";
    Rtm = "RTM", "return from a module";
    Trapcc = "TRAP", "take a trap if a condition holds";
    Unpk = "UNPK", "unpack a BCD byte into two digits, with an adjustment";
    Pgen = "P", "a memory management instruction: PMOVE, PTEST, PLOAD, PFLUSH";
}

impl Op {
    /// Whether the mnemonic takes a condition-code suffix from bits 11–8.
    #[must_use]
    pub const fn is_conditional(self) -> bool {
        matches!(self, Op::Bcc | Op::Dbcc | Op::Scc | Op::Trapcc)
    }

    /// The mnemonic, for the operations whose name is chosen by their
    /// extension word rather than their opcode.
    ///
    /// `MULS.L` and `MULU.L` share an opcode and differ in bit 11 of the
    /// extension word, and so do `CHK2` and `CMP2`; `DIVSL` is `DIVS.L` with a
    /// 32-bit dividend and a remainder register distinct from the quotient
    /// (M68000PRM, *DIVS*, *MULS*, *CHK2*).
    #[must_use]
    pub fn mnemonic_with(self, ext: u16) -> &'static str {
        let signed = ext & 0x0800 != 0;
        match self {
            Op::Mull => {
                if signed {
                    "MULS"
                } else {
                    "MULU"
                }
            }
            Op::Divl => {
                let quad = ext & 0x0400 != 0;
                let remainder = (ext & 7) != (ext >> 12) & 7;
                match (signed, !quad && remainder) {
                    (true, true) => "DIVSL",
                    (true, false) => "DIVS",
                    (false, true) => "DIVUL",
                    (false, false) => "DIVU",
                }
            }
            Op::Cmp2 => {
                if signed {
                    "CHK2"
                } else {
                    "CMP2"
                }
            }
            // The coprocessor command word carries the operation, exactly as
            // `MULS.L`'s extension word carries its signedness.
            Op::Pgen => pmmu::mnemonic(ext),
            other => other.mnemonic(),
        }
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
    /// Which processors implement the row.
    pub models: Models,
    /// How many extension words the instruction carries *before* any
    /// effective-address extension — `MOVEC`'s register word, `CAS2`'s two,
    /// a bit field's offset and width. `PACK` and `UNPK`, which have no
    /// effective address, count their adjustment word here too.
    pub ext: u8,
    /// Whether the row needs a floating-point coprocessor to exist at all.
    ///
    /// Not a [`Models`] bit, because the FPU is a *property* of the board and
    /// not of the part: a 68020 with no 68881 sees every one of these
    /// encodings as the line-F exception, and a 68020 with one sees an
    /// instruction. [`decode_with`] is where the two part company.
    pub fpu: bool,
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
        models: Models::ALL,
        ext: 0,
        fpu: false,
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
            models: Models::ALL,
            ext: 0,
            fpu: false,
        }
    }

    const fn models(mut self, models: Models) -> Insn {
        self.models = models;
        self
    }

    const fn since_010(self) -> Insn {
        self.models(Models::FROM_010)
    }

    const fn only_020(self) -> Insn {
        self.models(Models::M68020)
    }

    const fn since_020(self) -> Insn {
        self.models(Models::FROM_020)
    }

    const fn since_030(self) -> Insn {
        self.models(Models::FROM_030)
    }

    const fn with_ext(mut self, words: u8) -> Insn {
        self.ext = words;
        self
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
    AnHi, AnLo, BitNumber, Ccr, Ctrl, Disp8, Disp16, Disp32, DnHi, DnLo, Ea, EaDst, ExtReg, Imm,
    MovepEa, PostHi, PostLo, Quick, QuickByte, RegList, RmHi, RmLo, ShiftCount, Sr, TrapData, Usp,
    Vector, Vector3,
};
use Size::{Byte, Long, Word};
use SizeSpec::{Bit6, Bit8, BitOp, Bits76, Bits109, Fixed, Move as MoveSize};

table! {
    // ---- line 0: immediates, static bit operations, MOVEP ----------------
    0xffff 0x003c => Insn::new(Op::OriToCcr,  Fixed(Byte), Imm, Ccr);
    0xffff 0x007c => Insn::new(Op::OriToSr,   Fixed(Word), Imm, Sr).privileged();
    0xffff 0x023c => Insn::new(Op::AndiToCcr, Fixed(Byte), Imm, Ccr);
    0xffff 0x027c => Insn::new(Op::AndiToSr,  Fixed(Word), Imm, Sr).privileged();
    0xffff 0x0a3c => Insn::new(Op::EoriToCcr, Fixed(Byte), Imm, Ccr);
    0xffff 0x0a7c => Insn::new(Op::EoriToSr,  Fixed(Word), Imm, Sr).privileged();
    // The 68020's line-0 additions all live in the `11` size field of an
    // immediate instruction, which rejects it — so they must be matched first
    // (M68000PRM §8, *Instruction Format Summary*). CAS2 is CAS with an
    // immediate "effective address", and is matched before CAS for the same
    // reason.
    0xffff 0x0cfc => Insn::new(Op::Cas2, Fixed(Word), Arg::None, Arg::None).with_ext(2).since_020();
    0xffff 0x0efc => Insn::new(Op::Cas2, Fixed(Long), Arg::None, Arg::None).with_ext(2).since_020();
    0xffc0 0x0ac0 => Insn::new(Op::Cas,  Fixed(Byte), Arg::None, Ea)
                        .dst_ea(EaSet::MEM_ALT).with_ext(1).since_020();
    0xffc0 0x0cc0 => Insn::new(Op::Cas,  Fixed(Word), Arg::None, Ea)
                        .dst_ea(EaSet::MEM_ALT).with_ext(1).since_020();
    0xffc0 0x0ec0 => Insn::new(Op::Cas,  Fixed(Long), Arg::None, Ea)
                        .dst_ea(EaSet::MEM_ALT).with_ext(1).since_020();
    // RTM's register field sits where CALLM's effective address would name a
    // register, which CALLM does not accept.
    0xfff0 0x06c0 => Insn::new(Op::Rtm,   SizeSpec::None, Arg::None, Arg::None).only_020();
    0xffc0 0x06c0 => Insn::new(Op::Callm, SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::CONTROL).with_ext(1).only_020();
    0xf9c0 0x00c0 => Insn::new(Op::Cmp2,  Bits109, Ea, ExtReg)
                        .src_ea(EaSet::CONTROL).with_ext(1).since_020();
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
    // The 68020 lets CMPI compare against a PC-relative operand; the 68000
    // and 68010 do not (M68000PRM, *CMPI*: "PC relative addressing modes do
    // not apply to MC68000").
    0xff00 0x0c00 => Insn::new(Op::Cmpi,      Bits76, Imm, Ea)
                        .dst_ea(EaSet::DATA.without(Mode::Imm)).since_020();
    0xff00 0x0c00 => Insn::new(Op::Cmpi,      Bits76, Imm, Ea)
                        .dst_ea(EaSet::DATA_ALT).models(Models::UNTIL_010);
    // MOVES carries its direction in its extension word, not its opcode.
    0xff00 0x0e00 => Insn::new(Op::Moves,     Bits76, ExtReg, Ea)
                        .dst_ea(EaSet::MEM_ALT).with_ext(1).privileged().since_010();
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
    0xffff 0x4e74 => Insn::new(Op::Rtd,     SizeSpec::None, Disp16, Arg::None).since_010();
    0xffff 0x4e75 => Insn::new(Op::Rts,     SizeSpec::None, Arg::None, Arg::None);
    0xffff 0x4e76 => Insn::new(Op::Trapv,   SizeSpec::None, Arg::None, Arg::None);
    0xffff 0x4e77 => Insn::new(Op::Rtr,     SizeSpec::None, Arg::None, Arg::None);
    // Bit 0 is the direction: clear reads the control register.
    0xffff 0x4e7a => Insn::new(Op::Movec,   Fixed(Long), Ctrl, ExtReg)
                        .with_ext(1).privileged().since_010();
    0xffff 0x4e7b => Insn::new(Op::Movec,   Fixed(Long), ExtReg, Ctrl)
                        .with_ext(1).privileged().since_010();
    0xfff0 0x4e40 => Insn::new(Op::Trap,    SizeSpec::None, Vector, Arg::None);
    0xfff8 0x4e50 => Insn::new(Op::Link,    Fixed(Word), AnLo, Disp16);
    0xfff8 0x4e58 => Insn::new(Op::Unlk,    Fixed(Long), AnLo, Arg::None);
    0xfff8 0x4e60 => Insn::new(Op::MoveUsp, Fixed(Long), AnLo, Usp).privileged();
    0xfff8 0x4e68 => Insn::new(Op::MoveUsp, Fixed(Long), Usp, AnLo).privileged();
    0xfff8 0x4840 => Insn::new(Op::Swap,    Fixed(Word), DnLo, Arg::None);
    // BKPT is PEA's address-register form, which PEA rejects.
    0xfff8 0x4848 => Insn::new(Op::Bkpt,    SizeSpec::None, Vector3, Arg::None).since_010();
    // LINK.L is NBCD's address-register form, and EXTB.L is LEA's
    // data-register form; both reject those.
    0xfff8 0x4808 => Insn::new(Op::Link,    Fixed(Long), AnLo, Disp32).since_020();
    0xfff8 0x49c0 => Insn::new(Op::Extb,    Fixed(Long), DnLo, Arg::None).since_020();
    0xfff8 0x4880 => Insn::new(Op::Ext,     Bit6, DnLo, Arg::None);
    0xfff8 0x48c0 => Insn::new(Op::Ext,     Bit6, DnLo, Arg::None);
    // MOVE from SR is privileged from the 68010 on — the change that let a
    // virtual machine monitor hide the real supervisor state — and MOVE from
    // CCR arrived in its place for user code (M68000PRM Table A-1, note 4).
    0xffc0 0x40c0 => Insn::new(Op::MoveFromSr, Fixed(Word), Sr, Ea)
                        .dst_ea(EaSet::DATA_ALT).models(Models::M68000);
    0xffc0 0x40c0 => Insn::new(Op::MoveFromSr, Fixed(Word), Sr, Ea)
                        .dst_ea(EaSet::DATA_ALT).privileged().since_010();
    0xffc0 0x42c0 => Insn::new(Op::MoveFromCcr, Fixed(Word), Ccr, Ea)
                        .dst_ea(EaSet::DATA_ALT).since_010();
    0xffc0 0x44c0 => Insn::new(Op::MoveToCcr,  Fixed(Word), Ea, Ccr)
                        .src_ea(EaSet::DATA);
    0xffc0 0x46c0 => Insn::new(Op::MoveToSr,   Fixed(Word), Ea, Sr)
                        .src_ea(EaSet::DATA).privileged();
    0xffc0 0x4800 => Insn::new(Op::Nbcd, Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x4840 => Insn::new(Op::Pea,  Fixed(Long), Ea, Arg::None).src_ea(EaSet::CONTROL);
    0xffc0 0x4ac0 => Insn::new(Op::Tas,  Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xffc0 0x4e80 => Insn::new(Op::Jsr,  SizeSpec::None, Ea, Arg::None).src_ea(EaSet::CONTROL);
    0xffc0 0x4ec0 => Insn::new(Op::Jmp,  SizeSpec::None, Ea, Arg::None).src_ea(EaSet::CONTROL);
    // The 32-bit multiply and divide sit below MOVEM's memory-to-register
    // encoding, in space the 68000 left empty.
    0xffc0 0x4c00 => Insn::new(Op::Mull, Fixed(Long), Ea, Arg::None)
                        .src_ea(EaSet::DATA).with_ext(1).since_020();
    0xffc0 0x4c40 => Insn::new(Op::Divl, Fixed(Long), Ea, Arg::None)
                        .src_ea(EaSet::DATA).with_ext(1).since_020();
    0xff80 0x4880 => Insn::new(Op::Movem, Bit6, RegList, Ea).dst_ea(EaSet::MOVEM_TO_MEM);
    0xff80 0x4c80 => Insn::new(Op::Movem, Bit6, Ea, RegList).src_ea(EaSet::MOVEM_TO_REG);
    0xff00 0x4000 => Insn::new(Op::Negx, Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4200 => Insn::new(Op::Clr,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4400 => Insn::new(Op::Neg,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xff00 0x4600 => Insn::new(Op::Not,  Bits76, Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    // TST reaches every mode on a 68020 — an address register as a word or a
    // long, the PC-relative modes and an immediate (M68000PRM, *TST*).
    0xff00 0x4a00 => Insn::new(Op::Tst,  Bits76, Ea, Arg::None).src_ea(EaSet::ALL).since_020();
    0xff00 0x4a00 => Insn::new(Op::Tst,  Bits76, Ea, Arg::None)
                        .src_ea(EaSet::DATA_ALT).models(Models::UNTIL_010);
    0xf1c0 0x4100 => Insn::new(Op::Chk,  Fixed(Long), Ea, DnHi).src_ea(EaSet::DATA).since_020();
    0xf1c0 0x4180 => Insn::new(Op::Chk,  Fixed(Word), Ea, DnHi).src_ea(EaSet::DATA);
    0xf1c0 0x41c0 => Insn::new(Op::Lea,  Fixed(Long), Ea, AnHi).src_ea(EaSet::CONTROL);

    // ---- line 5: ADDQ, SUBQ, Scc, DBcc, TRAPcc --------------------------
    0xf0f8 0x50c8 => Insn::new(Op::Dbcc, Fixed(Word), DnLo, Disp16);
    // TRAPcc is Scc with an immediate or PC-relative "destination", which
    // Scc rejects. Bits 2-0 say how many operand words follow.
    0xf0ff 0x50fa => Insn::new(Op::Trapcc, Fixed(Word), TrapData, Arg::None).since_020();
    0xf0ff 0x50fb => Insn::new(Op::Trapcc, Fixed(Long), TrapData, Arg::None).since_020();
    0xf0ff 0x50fc => Insn::new(Op::Trapcc, SizeSpec::None, Arg::None, Arg::None).since_020();
    0xf0c0 0x50c0 => Insn::new(Op::Scc,  Fixed(Byte), Arg::None, Ea).dst_ea(EaSet::DATA_ALT);
    0xf100 0x5000 => Insn::new(Op::Addq, Bits76, Quick, Ea).dst_ea(EaSet::ALTERABLE);
    0xf100 0x5100 => Insn::new(Op::Subq, Bits76, Quick, Ea).dst_ea(EaSet::ALTERABLE);

    // ---- line 6: branches -----------------------------------------------
    // A displacement byte of $ff means a 32-bit displacement follows on a
    // 68020; on a 68000 it is a branch by -1, to an odd address.
    0xffff 0x60ff => Insn::new(Op::Bra, SizeSpec::None, Disp32, Arg::None).since_020();
    0xffff 0x61ff => Insn::new(Op::Bsr, SizeSpec::None, Disp32, Arg::None).since_020();
    0xf0ff 0x60ff => Insn::new(Op::Bcc, SizeSpec::None, Disp32, Arg::None).since_020();
    0xff00 0x6000 => Insn::new(Op::Bra, SizeSpec::None, Disp8, Arg::None);
    0xff00 0x6100 => Insn::new(Op::Bsr, SizeSpec::None, Disp8, Arg::None);
    0xf000 0x6000 => Insn::new(Op::Bcc, SizeSpec::None, Disp8, Arg::None);

    // ---- line 7: MOVEQ ---------------------------------------------------
    0xf100 0x7000 => Insn::new(Op::Moveq, Fixed(Long), QuickByte, DnHi);

    // ---- line 8: OR, DIV, SBCD, PACK, UNPK ------------------------------
    0xf1f0 0x8100 => Insn::new(Op::Sbcd, Fixed(Byte), RmLo, RmHi);
    // PACK and UNPK are OR's register-destination forms, which OR rejects.
    0xf1f0 0x8140 => Insn::new(Op::Pack, SizeSpec::None, RmLo, RmHi).with_ext(1).since_020();
    0xf1f0 0x8180 => Insn::new(Op::Unpk, SizeSpec::None, RmLo, RmHi).with_ext(1).since_020();
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

    // ---- line e: shifts, rotates and bit fields --------------------------
    // The bit-field instructions occupy the size-11 encodings of the
    // register shifts, so they are matched first. Unsized: the field is
    // whatever the extension word says.
    0xffc0 0xe8c0 => Insn::new(Op::Bftst,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD).with_ext(1).since_020();
    0xffc0 0xe9c0 => Insn::new(Op::Bfextu, SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD).with_ext(1).since_020();
    0xffc0 0xeac0 => Insn::new(Op::Bfchg,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD_ALT).with_ext(1).since_020();
    0xffc0 0xebc0 => Insn::new(Op::Bfexts, SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD).with_ext(1).since_020();
    0xffc0 0xecc0 => Insn::new(Op::Bfclr,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD_ALT).with_ext(1).since_020();
    0xffc0 0xedc0 => Insn::new(Op::Bfffo,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD).with_ext(1).since_020();
    0xffc0 0xeec0 => Insn::new(Op::Bfset,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD_ALT).with_ext(1).since_020();
    0xffc0 0xefc0 => Insn::new(Op::Bfins,  SizeSpec::None, Ea, Arg::None)
                        .src_ea(EaSet::BITFIELD_ALT).with_ext(1).since_020();
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

    // ---- line f: the coprocessor escape ----------------------------------
    // Coprocessor id 0 is the 68030's on-chip memory management unit, which
    // answers only the `000` instruction class — PMOVE, PTEST, PLOAD and
    // PFLUSH, told apart by their command word (`pmmu::decode`). The 68851's
    // other classes (PBcc, PScc, PDBcc, PTRAPcc, PSAVE, PRESTORE) are *not*
    // implemented by a 68030 and stay line F (MC68030UM §9.6). The effective
    // address is left unconstrained here because PFLUSHA has none and encodes
    // `000000` in the field; `pmmu::decode` carries each form's own rule.
    0xffc0 0xf000 => Insn::new(Op::Pgen, SizeSpec::None, Ea, Arg::None)
                        .with_ext(1).privileged().since_030();
    // With no coprocessor present every F-line word takes the line-F
    // exception — except cpSAVE and cpRESTORE, which a 68020 checks for
    // privilege before it tries to talk to any coprocessor, so user code gets
    // a privilege violation instead (MC68020UM §7.5.2.3). Bits 11-9 are the
    // coprocessor id and do not matter to that check.
    0xf1c0 0xf100 => Insn::new(Op::LineF, SizeSpec::None, Arg::None, Arg::None)
                        .privileged().since_020();
    0xf1c0 0xf140 => Insn::new(Op::LineF, SizeSpec::None, Arg::None, Arg::None)
                        .privileged().since_020();
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

/// Decode an opcode word into its table row, for a 68000.
///
/// Unassigned encodings, illegal size fields and illegal addressing modes all
/// return [`Insn::ILLEGAL`] — on a 68000 those are the same thing, an
/// illegal-instruction exception through vector 4 — with the sole exception of
/// the `$A` and `$F` lines, which have their own vectors and their own rows.
#[inline]
#[must_use]
pub fn decode(opcode: u16) -> Insn {
    decode_for(Model::M68000, opcode)
}

/// Which optional coprocessors answer the F line.
///
/// Separate from [`Models`] because a coprocessor is a property of the
/// *board*: the same 68020 is a 68020 with a 68881 and a 68020 without one,
/// and the difference is visible in the opcode map rather than in the part
/// number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Copro {
    /// A 68881 or 68882 answers coprocessor id 1.
    pub fpu: bool,
}

impl Copro {
    /// No coprocessor: every F-line word is the line-F exception.
    pub const NONE: Copro = Copro { fpu: false };
    /// A floating-point coprocessor on id 1.
    pub const FPU: Copro = Copro { fpu: true };
}

/// Decode an opcode word into its table row, for a given processor.
///
/// Rows the processor does not implement are skipped rather than matched, so
/// an encoding a later part added falls through to whatever an earlier part
/// made of it — almost always [`Insn::ILLEGAL`].
#[inline]
#[must_use]
pub fn decode_for(model: Model, opcode: u16) -> Insn {
    decode_with(model, Copro::NONE, opcode)
}

/// Decode an opcode word for a given processor *and* its coprocessors.
///
/// The only rows this reaches that [`decode_for`] does not are the
/// floating-point ones, which exist when a 68881 or 68882 is attached and are
/// the line-F exception when it is not.
#[inline]
#[must_use]
pub fn decode_with(model: Model, copro: Copro, opcode: u16) -> Insn {
    let (start, end) = NIBBLE[(opcode >> 12) as usize];
    let mut i = start as usize;
    while i < end as usize {
        let pattern = &TABLE[i];
        if (copro.fpu || !pattern.insn.fpu)
            && pattern.insn.models.contains(model)
            && pattern.matches(opcode)
        {
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

/// How a 68020 full-format extension word reaches memory, if it does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Indirect {
    /// No memory indirection: the address is base + index + displacement.
    None,
    /// Memory indirect, preindexed: the index is added *before* the pointer
    /// is fetched.
    Pre,
    /// Memory indirect, postindexed: the index is added to the pointer
    /// fetched.
    Post,
}

/// A 68020 full-format extension word, decoded (M68000PRM §2.2.3, Figure 2-2
/// and Table 2-2).
///
/// Everything the effective-address calculation and the disassembler need,
/// and the number of words that follow it — which is the part of an
/// instruction's length a 68000 never had to compute from data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FullExt {
    /// Bit 15: the index is an address register.
    pub index_addr: bool,
    /// Bits 14–12: the index register.
    pub index_reg: u8,
    /// Bit 11: the whole index register, not its sign-extended low word.
    pub index_long: bool,
    /// Bits 10–9: the index is multiplied by 1, 2, 4 or 8.
    pub scale: u8,
    /// Bit 7: the base register is suppressed (reads as zero).
    pub base_suppressed: bool,
    /// Bit 6: the index is suppressed.
    pub index_suppressed: bool,
    /// Bits 5–4: how many words the base displacement occupies, 0, 1 or 2.
    pub bd_words: u8,
    /// Bits 2–0: whether, and how, the address is indirect.
    pub indirect: Indirect,
    /// How many words the outer displacement occupies, 0, 1 or 2.
    pub od_words: u8,
}

impl FullExt {
    /// Decode a full-format word, or `None` for one of the encodings Table
    /// 2-2 marks reserved.
    ///
    /// A reserved encoding is not an addressing mode, so the instruction that
    /// carries one is an illegal instruction. The manual does not say what a
    /// 68020 actually does with one; vector 4 is the reading that cannot
    /// silently compute a wrong address, and it is what this core does.
    #[must_use]
    pub const fn decode(word: u16) -> Option<FullExt> {
        // Bit 3 is defined as zero.
        if word & 0x0008 != 0 {
            return None;
        }
        let bd_words = match (word >> 4) & 3 {
            0 => return None,
            1 => 0,
            2 => 1,
            _ => 2,
        };
        let index_suppressed = word & 0x0040 != 0;
        let iis = word & 7;
        let (indirect, od) = if index_suppressed {
            match iis {
                0 => (Indirect::None, 0),
                1 => (Indirect::Pre, 1),
                2 => (Indirect::Pre, 2),
                3 => (Indirect::Pre, 3),
                _ => return None,
            }
        } else {
            match iis {
                0 => (Indirect::None, 0),
                1 => (Indirect::Pre, 1),
                2 => (Indirect::Pre, 2),
                3 => (Indirect::Pre, 3),
                4 => return None,
                5 => (Indirect::Post, 1),
                6 => (Indirect::Post, 2),
                _ => (Indirect::Post, 3),
            }
        };
        // `od` above is the encoded size, 1 null, 2 word, 3 long.
        let od_words = match od {
            2 => 1,
            3 => 2,
            _ => 0,
        };
        Some(FullExt {
            index_addr: word & 0x8000 != 0,
            index_reg: ((word >> 12) & 7) as u8,
            index_long: word & 0x0800 != 0,
            scale: ((word >> 9) & 3) as u8,
            base_suppressed: word & 0x0080 != 0,
            index_suppressed,
            bd_words,
            indirect,
            od_words,
        })
    }

    /// Words that follow the extension word itself.
    #[must_use]
    pub const fn trailing_words(self) -> u32 {
        self.bd_words as u32 + self.od_words as u32
    }
}

/// Whether an indexed mode's first extension word is a full-format word on
/// this processor.
///
/// Bit 8 selects it, and only a 68020 looks: the 68000 and 68010 ignore bits
/// 10–8 of a brief word altogether (M68000PRM §2.2.3).
#[inline]
#[must_use]
pub const fn is_full_format(model: Model, first: u16) -> bool {
    model.has_020() && first & 0x0100 != 0
}

/// How many words an indexed mode occupies on this processor, given its
/// first extension word — or `None` if that word is a reserved full format.
#[must_use]
pub const fn index_ext_words(model: Model, first: u16) -> Option<u32> {
    if !is_full_format(model, first) {
        return Some(1);
    }
    match FullExt::decode(first) {
        Some(full) => Some(1 + full.trailing_words()),
        None => None,
    }
}

/// A bit field's `{offset:width}` specification, from the instruction's
/// extension word (M68000PRM, *BFTST*, "Instruction Fields").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FieldSpec {
    /// Bit 11: the offset is in a data register rather than immediate.
    pub offset_reg: bool,
    /// Bits 10–6: the immediate offset 0–31, or (bits 8–6) the register.
    pub offset: u8,
    /// Bit 5: the width is in a data register rather than immediate.
    pub width_reg: bool,
    /// Bits 4–0: the immediate width, 0 meaning 32, or (bits 2–0) the
    /// register.
    pub width: u8,
    /// Bits 14–12: the data register `BFEXTU`, `BFEXTS`, `BFFFO` and `BFINS`
    /// name.
    pub reg: u8,
}

impl FieldSpec {
    /// Split an extension word into its fields.
    #[must_use]
    pub const fn decode(word: u16) -> FieldSpec {
        let offset_reg = word & 0x0800 != 0;
        let width_reg = word & 0x0020 != 0;
        FieldSpec {
            offset_reg,
            offset: if offset_reg {
                ((word >> 6) & 7) as u8
            } else {
                ((word >> 6) & 0x1f) as u8
            },
            width_reg,
            width: if width_reg {
                (word & 7) as u8
            } else {
                (word & 0x1f) as u8
            },
            reg: ((word >> 12) & 7) as u8,
        }
    }
}

/// The 68030's memory management command word, described **once**.
///
/// `PMOVE`, `PTEST`, `PLOAD` and `PFLUSH` share the opcode word
/// `1111 000 000 <ea>` and are told apart entirely by this word (M68000PRM
/// §6; MC68030UM §9.7). As with the instruction table above, both the
/// interpreter and the disassembler read the description here, so a listing
/// cannot print an operation the interpreter does not perform.
pub mod pmmu {
    use core::fmt;

    /// Where a function code operand comes from (M68000PRM §6, *PFLUSH*'s
    /// **FC field**).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum FcSource {
        /// `10XXX` — the three bits are the function code.
        Immediate(u8),
        /// `01DDD` — bits 2–0 of a data register.
        DataReg(u8),
        /// `00000` — the source function code register.
        Sfc,
        /// `00001` — the destination function code register.
        Dfc,
    }

    impl FcSource {
        /// Decode a five-bit field, or `None` for an undefined encoding.
        #[must_use]
        pub const fn decode(bits: u16) -> Option<FcSource> {
            let bits = (bits & 0x1f) as u8;
            Some(match bits >> 3 {
                0b10 | 0b11 => FcSource::Immediate(bits & 7),
                0b01 => FcSource::DataReg(bits & 7),
                _ => match bits {
                    0 => FcSource::Sfc,
                    1 => FcSource::Dfc,
                    _ => return None,
                },
            })
        }
    }

    impl fmt::Display for FcSource {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                FcSource::Immediate(fc) => write!(f, "#{fc}"),
                FcSource::DataReg(n) => write!(f, "d{n}"),
                FcSource::Sfc => f.write_str("sfc"),
                FcSource::Dfc => f.write_str("dfc"),
            }
        }
    }

    /// Which register a `PMOVE` names.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum PReg {
        /// The translation control register, a long word. Full 68030 only.
        Tc,
        /// The supervisor root pointer, a quad word. Full 68030 only.
        Srp,
        /// The CPU root pointer, a quad word. Full 68030 only.
        Crp,
        /// Transparent translation register 0 — `AC0` on an EC030. A long
        /// word.
        Tt0,
        /// Transparent translation register 1 — `AC1`. A long word.
        Tt1,
        /// The MMU status register — `ACUSR` on an EC030. A word.
        Mmusr,
    }

    impl PReg {
        /// How many bytes the transfer moves (M68000PRM §6, *PMOVE*: quad for
        /// the root pointers, long for `TC` and the `TTx`, word for `MMUSR`).
        #[must_use]
        pub const fn bytes(self) -> u32 {
            match self {
                PReg::Tc | PReg::Tt0 | PReg::Tt1 => 4,
                PReg::Srp | PReg::Crp => 8,
                PReg::Mmusr => 2,
            }
        }

        /// Whether the register exists only on a part with the paged MMU.
        #[must_use]
        pub const fn needs_mmu(self) -> bool {
            matches!(self, PReg::Tc | PReg::Srp | PReg::Crp)
        }

        /// The assembler name.
        #[must_use]
        pub const fn name(self) -> &'static str {
            match self {
                PReg::Tc => "tc",
                PReg::Srp => "srp",
                PReg::Crp => "crp",
                PReg::Tt0 => "tt0",
                PReg::Tt1 => "tt1",
                PReg::Mmusr => "mmusr",
            }
        }
    }

    impl fmt::Display for PReg {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.write_str(self.name())
        }
    }

    /// What a `PFLUSH` flushes.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum Flush {
        /// `PFLUSHA` — every entry. Mode `001`.
        All,
        /// `PFLUSH fc,mask` — every entry whose function code matches. Mode
        /// `100`.
        ByFc(FcSource, u8),
        /// `PFLUSH fc,mask,<ea>` — the entry for one address in each matching
        /// function code. Mode `110`.
        ByFcAndAddress(FcSource, u8),
    }

    /// What a command word means.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum Class {
        /// `PMOVE` or `PMOVEFD`.
        Move {
            /// Which register.
            reg: PReg,
            /// Whether the register is the *source* — `PMOVE MRn,<ea>`.
            from_reg: bool,
            /// Whether the ATC is left alone (the **FD** bit).
            no_flush: bool,
        },
        /// `PFLUSH`, in one of its three forms.
        Flush(Flush),
        /// `PLOAD`, which walks the tree and writes an ATC entry.
        Load {
            /// Where the function code comes from.
            fc: FcSource,
            /// Whether the walk simulates a read (`PLOADR`) or a write.
            read: bool,
        },
        /// `PTEST`, which walks and reports into `MMUSR`.
        Test {
            /// Where the function code comes from.
            fc: FcSource,
            /// The highest table level to search; zero searches the ATC.
            level: u8,
            /// Whether the walk simulates a read (`PTESTR`) or a write.
            read: bool,
            /// The address register the last descriptor's address goes to.
            areg: Option<u8>,
        },
    }

    /// Decode a command word, or `None` for an encoding the 68030 does not
    /// implement — which is the line-F exception (MC68030UM §9.6).
    #[must_use]
    pub const fn decode(word: u16) -> Option<Class> {
        match word >> 13 {
            // 000 — the transparent translation registers.
            0b000 => {
                if word & 0x00ff != 0 {
                    return None;
                }
                let reg = match (word >> 10) & 7 {
                    0b010 => PReg::Tt0,
                    0b011 => PReg::Tt1,
                    _ => return None,
                };
                Some(Class::Move {
                    reg,
                    from_reg: word & 0x0200 != 0,
                    no_flush: word & 0x0100 != 0,
                })
            }
            // 010 — TC and the two root pointers.
            0b010 => {
                if word & 0x00ff != 0 {
                    return None;
                }
                let reg = match (word >> 10) & 7 {
                    0b000 => PReg::Tc,
                    0b010 => PReg::Srp,
                    0b011 => PReg::Crp,
                    _ => return None,
                };
                Some(Class::Move {
                    reg,
                    from_reg: word & 0x0200 != 0,
                    no_flush: word & 0x0100 != 0,
                })
            }
            // 011 — the MMU status register.
            0b011 => {
                if word & 0x1dff != 0 {
                    return None;
                }
                Some(Class::Move {
                    reg: PReg::Mmusr,
                    from_reg: word & 0x0200 != 0,
                    no_flush: true,
                })
            }
            // 001 — PFLUSH, and PLOAD in the mode field PFLUSH leaves empty.
            0b001 => {
                if word & 0x0100 != 0 {
                    return None;
                }
                let mask = ((word >> 5) & 7) as u8;
                match (word >> 10) & 7 {
                    // PLOAD is the only form in this class that uses bit 9,
                    // which is its read/write flag rather than part of a
                    // mode (M68000PRM §6, *PLOAD*).
                    0b000 => {
                        if mask != 0 {
                            return None;
                        }
                        match FcSource::decode(word) {
                            Some(fc) => Some(Class::Load {
                                fc,
                                read: word & 0x0200 != 0,
                            }),
                            None => None,
                        }
                    }
                    0b001 => {
                        if word & 0x02ff != 0 {
                            return None;
                        }
                        Some(Class::Flush(Flush::All))
                    }
                    0b100 => {
                        if word & 0x0200 != 0 {
                            return None;
                        }
                        match FcSource::decode(word) {
                            Some(fc) => Some(Class::Flush(Flush::ByFc(fc, mask))),
                            None => None,
                        }
                    }
                    0b110 => {
                        if word & 0x0200 != 0 {
                            return None;
                        }
                        match FcSource::decode(word) {
                            Some(fc) => Some(Class::Flush(Flush::ByFcAndAddress(fc, mask))),
                            None => None,
                        }
                    }
                    _ => None,
                }
            }
            // 100 — PTEST, or PLOAD when the level field is zero and the
            // low bits mark it.
            0b100 => {
                let level = ((word >> 10) & 7) as u8;
                let read = word & 0x0200 != 0;
                let areg_bit = word & 0x0100 != 0;
                let areg = ((word >> 5) & 7) as u8;
                if !areg_bit && areg != 0 {
                    return None;
                }
                if level == 0 && areg_bit {
                    // "The instruction takes an F-line exception when the
                    // level field is 0 and the A field is not 0"
                    // (M68000PRM §6, *PTEST*).
                    return None;
                }
                match FcSource::decode(word) {
                    Some(fc) => Some(Class::Test {
                        fc,
                        level,
                        read,
                        areg: if areg_bit { Some(areg) } else { None },
                    }),
                    None => None,
                }
            }
            // 101, 110 and 111 are the 68851's — PVALID, PSAVE and the
            // access-level instructions — and are not implemented by a 68030.
            _ => None,
        }
    }

    /// The mnemonic a command word names, for the disassembler.
    #[must_use]
    pub const fn mnemonic(word: u16) -> &'static str {
        match decode(word) {
            Some(Class::Move { .. }) => "PMOVE",
            Some(Class::Flush(Flush::All)) => "PFLUSHA",
            Some(Class::Flush(_)) => "PFLUSH",
            Some(Class::Load { read, .. }) => {
                if read {
                    "PLOADR"
                } else {
                    "PLOADW"
                }
            }
            Some(Class::Test { read, .. }) => {
                if read {
                    "PTESTR"
                } else {
                    "PTESTW"
                }
            }
            None => "P???",
        }
    }
}

/// `MOVEC`'s control-register codes (M68000PRM, *MOVEC*).
pub mod ctrl {
    /// Source function code.
    pub const SFC: u16 = 0x000;
    /// Destination function code.
    pub const DFC: u16 = 0x001;
    /// Cache control register (68020).
    pub const CACR: u16 = 0x002;
    /// User stack pointer.
    pub const USP: u16 = 0x800;
    /// Vector base register.
    pub const VBR: u16 = 0x801;
    /// Cache address register (68020).
    pub const CAAR: u16 = 0x802;
    /// Master stack pointer (68020).
    pub const MSP: u16 = 0x803;
    /// Interrupt stack pointer (68020).
    pub const ISP: u16 = 0x804;

    /// Whether `model` has the control register `code` — anything else is an
    /// illegal instruction (M68000PRM, *MOVEC*, note 1).
    #[must_use]
    pub const fn exists(model: super::Model, code: u16) -> bool {
        match code {
            SFC | DFC | USP | VBR => model.has_010(),
            CACR | CAAR | MSP | ISP => model.has_020(),
            _ => false,
        }
    }

    /// The assembler name of a control register.
    #[must_use]
    pub const fn name(code: u16) -> Option<&'static str> {
        Some(match code {
            SFC => "SFC",
            DFC => "DFC",
            CACR => "CACR",
            USP => "USP",
            VBR => "VBR",
            CAAR => "CAAR",
            MSP => "MSP",
            ISP => "ISP",
            _ => return None,
        })
    }
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
        // baseline — what matters is that they do not drift silently, and
        // the 68000's in particular is the number it was before the table
        // learned about any other processor.
        let mut reached: alloc::vec::Vec<Op> = alloc::vec::Vec::new();
        for (model, expected, f_line) in [
            (Model::M68000, 45_815usize, 0x1000usize),
            (Model::M68010, 46_002, 0x1000),
            (Model::M68020, 47_419, 0x1000),
            // The 68030 is the 68020 less `CALLM` and `RTM`, which it does
            // not implement — sixteen `RTM` encodings and twenty-eight legal
            // `CALLM` ones (MC68030UM §12.1.3) — plus the sixty-one F-line
            // words on coprocessor id 0 its memory management unit answers.
            // Sixty-four encodings share that opcode and none of them is
            // line F any more; the three whose effective-address field names
            // no mode at all are illegal, and the rest are sorted out by
            // their command word.
            (Model::M68030, 47_436, 0x1000 - 64),
            (Model::M68EC030, 47_436, 0x1000 - 64),
        ] {
            let mut legal = 0usize;
            let mut line_a = 0usize;
            let mut line_f = 0usize;
            for opcode in 0..=u16::MAX {
                match decode_for(model, opcode).op {
                    Op::Illegal => {}
                    Op::LineA => line_a += 1,
                    Op::LineF => line_f += 1,
                    other => {
                        legal += 1;
                        if !reached.contains(&other) {
                            reached.push(other);
                        }
                    }
                }
            }
            assert_eq!(line_a, 0x1000, "{model}: the whole $A line traps");
            assert_eq!(line_f, f_line, "{model}: the $F line");
            assert_eq!(legal, expected, "{model}");
        }
        // Every operation in the table is reachable from some encoding on
        // some processor.
        for op in Op::ALL {
            // The three that decode to something other than an operation with
            // operands are counted above rather than collected here.
            if matches!(op, Op::Illegal | Op::LineA | Op::LineF) {
                continue;
            }
            assert!(
                reached.contains(op),
                "{op:?} is in the table but no encoding reaches it"
            );
        }
    }

    #[test]
    fn the_68ec020_decodes_exactly_as_the_68020_does() {
        for opcode in 0..=u16::MAX {
            assert_eq!(
                decode_for(Model::M68020, opcode),
                decode_for(Model::M68EC020, opcode),
                "{opcode:04x}"
            );
        }
    }

    #[test]
    fn the_68030_is_the_68020_without_callm_and_rtm() {
        // MC68030UM §12.1.3: "the MC68030 does not support the CALLM and RTM
        // instructions of the MC68020. If code is executed on the MC68030
        // using either ... an unimplemented instruction exception is taken."
        // Everything else in the map is identical — the MMU instructions live
        // in the F line, which the 68020 leaves to a coprocessor.
        for opcode in 0..=u16::MAX {
            let twenty = decode_for(Model::M68020, opcode);
            let thirty = decode_for(Model::M68030, opcode);
            if matches!(twenty.op, Op::Callm | Op::Rtm) {
                assert_eq!(thirty.op, Op::Illegal, "{opcode:04x}");
                continue;
            }
            if opcode & 0xffc0 == 0xf000 {
                // The one thing the 68030 adds: coprocessor id 0's general
                // instruction class, which the 68020 leaves to an MC68851 it
                // has no way of knowing is there. The three encodings whose
                // effective-address field names no mode are illegal rather
                // than line F, because the row matched and its operand did
                // not.
                assert_eq!(twenty.op, Op::LineF, "{opcode:04x}");
                assert!(
                    matches!(thirty.op, Op::Pgen | Op::Illegal),
                    "{opcode:04x} is {:?}",
                    thirty.op
                );
                continue;
            }
            assert_eq!(twenty, thirty, "{opcode:04x}");
        }
    }

    #[test]
    fn the_68ec030_decodes_exactly_as_the_68030_does_outside_the_mmu() {
        // The two parts share an instruction set; what the MC68EC030 lacks is
        // the paged MMU itself, which shows up in the command word of an
        // F-line instruction rather than in the opcode map (MC68EC030UM §9.4).
        for opcode in 0..=u16::MAX {
            assert_eq!(
                decode_for(Model::M68030, opcode),
                decode_for(Model::M68EC030, opcode),
                "{opcode:04x}"
            );
        }
    }

    #[test]
    fn a_later_processor_only_ever_adds_to_the_opcode_map() {
        // The 68010 and 68020 are upward compatible (MC68000UM §1.1): every
        // encoding that means something on an earlier part means the same
        // operation on a later one. The exceptions are the ones the manuals
        // list — MOVE from SR becoming privileged, and the 68020 giving a
        // meaning to a Bcc displacement of $ff that the 68000 read as -1.
        for opcode in 0..=u16::MAX {
            let old = decode_for(Model::M68000, opcode);
            let ten = decode_for(Model::M68010, opcode);
            let twenty = decode_for(Model::M68020, opcode);
            if old.op != Op::Illegal {
                assert_eq!(old.op, ten.op, "{opcode:04x} changed on the 68010");
                if old.op != Op::MoveFromSr {
                    assert_eq!(old.privileged, ten.privileged, "{opcode:04x}");
                }
            }
            if ten.op != Op::Illegal {
                assert_eq!(ten.op, twenty.op, "{opcode:04x} changed on the 68020");
                if ten.src != Arg::Disp8 || opcode & 0xff != 0xff {
                    assert_eq!(ten.src, twenty.src, "{opcode:04x}");
                    assert_eq!(ten.dst, twenty.dst, "{opcode:04x}");
                }
            }
        }
        assert!(decode_for(Model::M68010, 0x40c0).privileged);
        assert!(!decode_for(Model::M68000, 0x40c0).privileged);
    }

    #[test]
    fn the_new_encodings_decode_where_the_manual_puts_them() {
        let m20 = |op| decode_for(Model::M68020, op).op;
        let m10 = |op| decode_for(Model::M68010, op).op;
        assert_eq!(m10(0x4e7a), Op::Movec);
        assert_eq!(m10(0x4e7b), Op::Movec);
        assert_eq!(m10(0x0e50), Op::Moves);
        assert_eq!(m10(0x4e74), Op::Rtd);
        assert_eq!(m10(0x42c0), Op::MoveFromCcr);
        assert_eq!(m10(0x4848), Op::Bkpt);
        assert_eq!(m10(0x4c00), Op::Illegal, "no MULS.L on a 68010");
        assert_eq!(m20(0x4c00), Op::Mull);
        assert_eq!(m20(0x4c40), Op::Divl);
        assert_eq!(m20(0xe8c0), Op::Bftst);
        assert_eq!(m20(0xe9d0), Op::Bfextu);
        assert_eq!(m20(0xedc0), Op::Bfffo);
        assert_eq!(m20(0xefd0), Op::Bfins);
        assert_eq!(m20(0x0ad0), Op::Cas);
        assert_eq!(m20(0x0cfc), Op::Cas2);
        assert_eq!(m20(0x00d0), Op::Cmp2);
        assert_eq!(m20(0x04d0), Op::Cmp2);
        assert_eq!(m20(0x06c3), Op::Rtm);
        assert_eq!(m20(0x06d0), Op::Callm);
        assert_eq!(m20(0x8141), Op::Pack);
        assert_eq!(m20(0x8189), Op::Unpk);
        assert_eq!(m20(0x49c0), Op::Extb);
        assert_eq!(m20(0x4808), Op::Link);
        assert_eq!(m20(0x51fa), Op::Trapcc);
        assert_eq!(m20(0x60ff), Op::Bra);
        assert_eq!(m20(0x66ff), Op::Bcc);
        assert_eq!(m20(0x4100), Op::Chk);
        assert_eq!(m20(0x4a48), Op::Tst, "TST.W An");
        assert_eq!(m20(0x4a3c), Op::Tst, "TST.B #imm");
        assert_eq!(m20(0x0c3a), Op::Cmpi, "CMPI.B #,(d16,PC)");
        assert_eq!(decode(0x0c3a).op, Op::Illegal);
        // cpSAVE and cpRESTORE are line F with a privilege check first.
        assert!(decode_for(Model::M68020, 0xf310).privileged);
        assert!(!decode_for(Model::M68020, 0xf210).privileged);
        assert!(!decode(0xf310).privileged);
        // The 68000 still sees none of it.
        for opcode in [
            0x4e7a, 0x0e50, 0x4e74, 0x42c0, 0x4848, 0x4c00, 0xe8c0, 0x0ad0,
        ] {
            assert_eq!(decode(opcode).op, Op::Illegal, "{opcode:04x}");
        }
        // A reserved full-format word is not an addressing mode.
        assert_eq!(FullExt::decode(0x0100), None, "BD SIZE 00 is reserved");
        assert_eq!(FullExt::decode(0x0114), None, "I/IS 100 is reserved");
        assert_eq!(
            FullExt::decode(0x0154),
            None,
            "IS=1 with I/IS 1xx is reserved"
        );
        assert_eq!(index_ext_words(Model::M68000, 0x0134), Some(1));
        assert_eq!(index_ext_words(Model::M68020, 0x0133), Some(1 + 2 + 2));
        assert_eq!(index_ext_words(Model::M68020, 0x0126), Some(1 + 1 + 1));
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
