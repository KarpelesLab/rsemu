//! The instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds one declarative description, [`TABLE`], from which
//! everything else is derived:
//!
//! - the interpreter's decode ([`decode`]) and its per-opcode bus sequence,
//!   which comes from [`Insn::mode`] + [`Insn::access`] rather than from a
//!   cycle-count column;
//! - the disassembler ([`super::disasm`]), which formats from the same row;
//! - `rsemu describe`-style introspection: mnemonics, one-line summaries, and
//!   which encodings are undocumented.
//!
//! # Why there is no cycle column
//!
//! Deliberate. A 6502 cycle *is* a bus access, and the count for
//! `LDA $1234,X` depends on whether the index crosses a page — which is a
//! property of the operand, not of the opcode. Modelling instructions as
//! "N cycles then a result" loses the dummy reads that page-crossing and
//! read-modify-write instructions perform, and those are visible to hardware
//! and load-bearing on the NES (`docs/cpu/6502.md`). [`Access`] says what
//! shape the operand access has; the interpreter turns that into the exact
//! sequence of reads and writes.
//!
//! # Two parts, two tables
//!
//! The NMOS 6502 and the CMOS W65C02S disagree about roughly a third of the
//! matrix, so there are two tables and [`Variant`] picks between them —
//! a construction property, never a `#[cfg]`, because one build of rsemu has to
//! run a NES and a Ben Eater breadboard at the same time. [`TABLE`] is the NMOS
//! part (the Ricoh RP2A03 shares it; what that chip lacks is the BCD adder, not
//! an opcode) and [`CMOS_TABLE`] is the WDC part.
//!
//! # Sources
//!
//! The NMOS opcode matrix, addressing modes and undocumented encodings are from
//! the masswerk 6502 instruction set reference, the NESdev "Obelisk" reference
//! and NESdev's *CPU unofficial opcodes* page (`docs/cpu/6502.md`),
//! cross-checked against `../gones/cpu6502` (ours, MIT, © Mark Karpelès). The
//! CMOS matrix — the new instructions, the new addressing modes, the
//! bit-manipulation group and the lengths and cycle counts of every encoding
//! the NMOS part left undocumented — is from the **W65C02S datasheet** (Western
//! Design Center), section 7 and its opcode matrix.

use core::fmt;

/// Which member of the family this core is.
///
/// A construction property rather than a build flag (`docs/cpu/6502.md`): the
/// three parts differ in their opcode matrix and in a handful of bus and
/// arithmetic behaviours, and one binary has to be able to run all three.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Variant {
    /// The original NMOS MOS 6502, undocumented opcodes and all.
    #[default]
    Nmos6502,
    /// The Ricoh RP2A03/RP2A07 in the NES: the same die with the BCD adder
    /// removed. Identical opcode matrix; see [`Config::decimal`].
    ///
    /// [`Config::decimal`]: super::Config::decimal
    Ricoh2A03,
    /// The WDC W65C02S: the CMOS successor. New instructions, new addressing
    /// modes, no undocumented encodings, and several NMOS bugs fixed.
    Wdc65C02,
}

impl Variant {
    /// Every variant, in the order a `--help` should list them.
    pub const ALL: &'static [Variant] = &[Variant::Nmos6502, Variant::Ricoh2A03, Variant::Wdc65C02];

    /// Whether this is a CMOS part, which is what almost every behavioural
    /// difference keys off.
    #[inline]
    #[must_use]
    pub const fn is_cmos(self) -> bool {
        matches!(self, Variant::Wdc65C02)
    }

    /// The spelling a machine file uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::Nmos6502 => "6502",
            Variant::Ricoh2A03 => "2a03",
            Variant::Wdc65C02 => "65c02",
        }
    }

    /// A one-line description, for `rsemu describe` and the monitor.
    #[must_use]
    pub const fn summary(self) -> &'static str {
        match self {
            Variant::Nmos6502 => "MOS 6502, NMOS, with the undocumented opcodes",
            Variant::Ricoh2A03 => "Ricoh RP2A03 (NES): a 6502 with no BCD adder",
            Variant::Wdc65C02 => "WDC W65C02S, CMOS, with the Rockwell bit instructions",
        }
    }

    /// Look a variant up by the name a machine file writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        Variant::ALL.iter().copied().find(|v| v.name() == name)
    }
}

impl fmt::Display for Variant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How an instruction finds its operand.
///
/// The mode fixes the instruction's length, which is why there is no separate
/// length column: [`Mode::bytes`] is the single answer both the disassembler
/// and the program counter use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// No operand: `INX`, `CLC`, `RTS`.
    Implied,
    /// The accumulator is the operand: `ASL A`.
    Accumulator,
    /// The byte after the opcode is the operand: `LDA #$42`.
    Immediate,
    /// A page-zero address: `LDA $42`.
    ZeroPage,
    /// Page zero indexed by X, wrapping inside page zero: `LDA $42,X`.
    ZeroPageX,
    /// Page zero indexed by Y, wrapping inside page zero: `LDX $42,Y`.
    ZeroPageY,
    /// A full 16-bit address: `LDA $1234`.
    Absolute,
    /// Absolute indexed by X: `LDA $1234,X`.
    AbsoluteX,
    /// Absolute indexed by Y: `LDA $1234,Y`.
    AbsoluteY,
    /// Indirect through a 16-bit pointer — `JMP ($1234)` only, page-wrap bug
    /// included.
    Indirect,
    /// Page-zero pointer pre-indexed by X: `LDA ($42,X)`.
    IndirectX,
    /// Page-zero pointer post-indexed by Y: `LDA ($42),Y`.
    IndirectY,
    /// A signed 8-bit displacement from the *next* instruction: `BNE $c012`.
    Relative,
    /// Page-zero indirect, no index at all: `LDA ($42)`. **CMOS only** — the
    /// addressing mode the NMOS part left out of the ALU group.
    ZeroPageIndirect,
    /// Indirect through a 16-bit pointer indexed by X: `JMP ($1234,X)`.
    /// **CMOS only.**
    AbsoluteIndirectX,
    /// A page-zero address *and* a branch displacement: `BBR0 $42,$c012`.
    /// **CMOS only**, and the only three-byte branch in the family.
    ZeroPageRelative,
    /// One byte, one cycle, no operand and not even a dummy read.
    ///
    /// **CMOS only.** The W65C02S fills columns `$x3` and `$xB` with NOPs that
    /// finish inside their own opcode fetch — the single case in this family
    /// where an instruction is one bus cycle long, and the reason it cannot
    /// share [`Mode::Implied`], which always spends its dummy read.
    Single,
    /// `BRK`'s implied mode, which nevertheless skips a byte.
    ///
    /// `BRK` is documented as a one-byte instruction but pushes `PC + 2`, so
    /// the byte after the opcode is consumed and never looked at. Calling that
    /// a two-byte instruction keeps disassembly aligned with execution, which
    /// is what a monitor needs; the extra byte is not shown because it has no
    /// meaning.
    Break,
}

impl Mode {
    /// Total instruction length in bytes, opcode included.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        match self {
            Mode::Implied | Mode::Accumulator | Mode::Single => 1,
            Mode::Immediate
            | Mode::ZeroPage
            | Mode::ZeroPageX
            | Mode::ZeroPageY
            | Mode::IndirectX
            | Mode::IndirectY
            | Mode::ZeroPageIndirect
            | Mode::Relative
            | Mode::Break => 2,
            Mode::Absolute
            | Mode::AbsoluteX
            | Mode::AbsoluteY
            | Mode::Indirect
            | Mode::AbsoluteIndirectX
            | Mode::ZeroPageRelative => 3,
        }
    }

    /// Whether this mode is indexed by X or Y, and so can cross a page.
    ///
    /// `JMP ($1234,X)` is indexed too, but its own handler resolves it and it
    /// never pays a page-cross cycle, so it is not listed here.
    #[must_use]
    pub const fn is_indexed(self) -> bool {
        matches!(
            self,
            Mode::ZeroPageX
                | Mode::ZeroPageY
                | Mode::AbsoluteX
                | Mode::AbsoluteY
                | Mode::IndirectX
                | Mode::IndirectY
        )
    }

    /// The mode's name as the assembler syntax suggests (`zpg,X`, `abs,Y`).
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            // The one-cycle CMOS NOPs address nothing, exactly like any other
            // implied instruction; only their timing sets them apart.
            Mode::Implied | Mode::Single => "impl",
            Mode::Accumulator => "A",
            Mode::Immediate => "#",
            Mode::ZeroPage => "zpg",
            Mode::ZeroPageX => "zpg,X",
            Mode::ZeroPageY => "zpg,Y",
            Mode::Absolute => "abs",
            Mode::AbsoluteX => "abs,X",
            Mode::AbsoluteY => "abs,Y",
            Mode::Indirect => "ind",
            Mode::IndirectX => "X,ind",
            Mode::IndirectY => "ind,Y",
            Mode::ZeroPageIndirect => "(zpg)",
            Mode::AbsoluteIndirectX => "ind,X",
            Mode::ZeroPageRelative => "zpg,rel",
            Mode::Relative => "rel",
            Mode::Break => "brk",
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What the instruction does to the byte its addressing mode selected.
///
/// This is the *bus shape*, and it is what the cycle count actually depends
/// on. `LDA $1234,X` reads and pays for a page cross only when it happens;
/// `STA $1234,X` writes and always pays, because the dummy read at the
/// unfixed address happens either way; `INC $1234,X` reads, writes the old
/// value back, then writes the new one — three accesses at one address, and
/// the middle one is visible to hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// The operand is not a memory location: implied, accumulator, branch,
    /// jump, stack. The interpreter hand-writes the sequence.
    None,
    /// One read of the operand (immediate included — the operand fetch *is*
    /// the read).
    Read,
    /// One write. The address computation always performs its dummy read.
    Write,
    /// Read, dummy write-back of the old value, write of the new one.
    Modify,
}

/// How well documented an encoding is.
///
/// Undocumented opcodes are in scope from the start because real software
/// depends on them (`docs/cpu/6502.md`); the distinction is kept so a
/// disassembler can flag them and a test can assert the matrix is complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Class {
    /// In the MOS datasheet.
    Documented,
    /// Undocumented but deterministic, and safe to rely on.
    Undocumented,
    /// Undocumented *and* analog: the result depends on a "magic constant"
    /// that varies with temperature and chip series (`ANE`, `LXA`), or on
    /// whether the high-byte AND is dropped (`SHA`, `SHX`, `SHY`, `TAS`).
    ///
    /// Modelled as documented-unstable rather than as a bug: the behaviour
    /// picked here is the one `SingleStepTests/65x02` expects, and the magic
    /// constant is a construction property
    /// ([`Config::magic`](super::Config::magic)).
    Unstable,
}

impl Class {
    /// Whether the encoding appears in the official instruction set.
    #[must_use]
    pub const fn is_documented(self) -> bool {
        matches!(self, Class::Documented)
    }
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
///
/// The mnemonic is the variant name, so the two cannot disagree.
macro_rules! define_ops {
    ($($name:ident = $summary:literal,)*) => {
        /// One operation, independent of how its operand is addressed.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]), so a
        /// disassembler cannot print a name the interpreter does not
        /// implement.
        // Mnemonics are uppercase by universal 6502 convention; renaming them
        // to satisfy the acronym lint would make every reference unreadable.
        #[allow(clippy::upper_case_acronyms)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum Op {
            $(
                #[doc = $summary]
                $name,
            )*
        }

        impl Op {
            /// The assembler mnemonic.
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(Op::$name => stringify!($name),)* }
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
    ADC = "add memory to accumulator with carry",
    ALR = "AND with immediate, then shift the accumulator right",
    ANC = "AND with immediate, then copy bit 7 into carry",
    AND = "AND memory with accumulator",
    ANE = "unstable: (A | magic) AND X AND immediate into A",
    ARR = "AND with immediate, then rotate right through the adder",
    ASL = "shift left one bit",
    BBR0 = "branch if bit 0 of the page-zero byte is clear",
    BBR1 = "branch if bit 1 of the page-zero byte is clear",
    BBR2 = "branch if bit 2 of the page-zero byte is clear",
    BBR3 = "branch if bit 3 of the page-zero byte is clear",
    BBR4 = "branch if bit 4 of the page-zero byte is clear",
    BBR5 = "branch if bit 5 of the page-zero byte is clear",
    BBR6 = "branch if bit 6 of the page-zero byte is clear",
    BBR7 = "branch if bit 7 of the page-zero byte is clear",
    BBS0 = "branch if bit 0 of the page-zero byte is set",
    BBS1 = "branch if bit 1 of the page-zero byte is set",
    BBS2 = "branch if bit 2 of the page-zero byte is set",
    BBS3 = "branch if bit 3 of the page-zero byte is set",
    BBS4 = "branch if bit 4 of the page-zero byte is set",
    BBS5 = "branch if bit 5 of the page-zero byte is set",
    BBS6 = "branch if bit 6 of the page-zero byte is set",
    BBS7 = "branch if bit 7 of the page-zero byte is set",
    BCC = "branch if carry clear",
    BCS = "branch if carry set",
    BEQ = "branch if equal (zero set)",
    BIT = "test bits in memory against the accumulator",
    BMI = "branch if minus (negative set)",
    BNE = "branch if not equal (zero clear)",
    BPL = "branch if plus (negative clear)",
    BRA = "branch always",
    BRK = "force an interrupt through the IRQ vector",
    BVC = "branch if overflow clear",
    BVS = "branch if overflow set",
    CLC = "clear carry",
    CLD = "clear decimal mode",
    CLI = "clear the interrupt disable",
    CLV = "clear overflow",
    CMP = "compare memory with the accumulator",
    CPX = "compare memory with X",
    CPY = "compare memory with Y",
    DCP = "decrement memory, then compare it with the accumulator",
    DEC = "decrement memory",
    DEX = "decrement X",
    DEY = "decrement Y",
    EOR = "exclusive-OR memory with the accumulator",
    INC = "increment memory",
    INX = "increment X",
    INY = "increment Y",
    ISC = "increment memory, then subtract it from the accumulator",
    JAM = "halt the processor until reset",
    JMP = "jump",
    JSR = "jump to subroutine",
    LAS = "AND memory with the stack pointer into A, X and S",
    LAX = "load memory into both the accumulator and X",
    LDA = "load the accumulator",
    LDX = "load X",
    LDY = "load Y",
    LSR = "shift right one bit",
    LXA = "unstable: (A | magic) AND immediate into A and X",
    NOP = "no operation",
    ORA = "OR memory with the accumulator",
    PHA = "push the accumulator",
    PHP = "push the processor status",
    PHX = "push X",
    PHY = "push Y",
    PLA = "pull the accumulator",
    PLP = "pull the processor status",
    PLX = "pull X",
    PLY = "pull Y",
    RLA = "rotate memory left, then AND it with the accumulator",
    RMB0 = "reset bit 0 of the page-zero byte",
    RMB1 = "reset bit 1 of the page-zero byte",
    RMB2 = "reset bit 2 of the page-zero byte",
    RMB3 = "reset bit 3 of the page-zero byte",
    RMB4 = "reset bit 4 of the page-zero byte",
    RMB5 = "reset bit 5 of the page-zero byte",
    RMB6 = "reset bit 6 of the page-zero byte",
    RMB7 = "reset bit 7 of the page-zero byte",
    ROL = "rotate left one bit through carry",
    ROR = "rotate right one bit through carry",
    RRA = "rotate memory right, then add it to the accumulator",
    RTI = "return from interrupt",
    RTS = "return from subroutine",
    SAX = "store A AND X",
    SBC = "subtract memory from the accumulator with borrow",
    SBX = "(A AND X) minus immediate into X, flags as a compare",
    SEC = "set carry",
    SED = "set decimal mode",
    SEI = "set the interrupt disable",
    SHA = "unstable: store A AND X AND (address high + 1)",
    SHX = "unstable: store X AND (address high + 1)",
    SHY = "unstable: store Y AND (address high + 1)",
    SLO = "shift memory left, then OR it with the accumulator",
    SMB0 = "set bit 0 of the page-zero byte",
    SMB1 = "set bit 1 of the page-zero byte",
    SMB2 = "set bit 2 of the page-zero byte",
    SMB3 = "set bit 3 of the page-zero byte",
    SMB4 = "set bit 4 of the page-zero byte",
    SMB5 = "set bit 5 of the page-zero byte",
    SMB6 = "set bit 6 of the page-zero byte",
    SMB7 = "set bit 7 of the page-zero byte",
    SRE = "shift memory right, then exclusive-OR it with the accumulator",
    STA = "store the accumulator",
    STP = "stop the clock until reset",
    STX = "store X",
    STY = "store Y",
    STZ = "store zero",
    TAS = "unstable: A AND X into S, then store it AND (address high + 1)",
    TAX = "transfer the accumulator to X",
    TAY = "transfer the accumulator to Y",
    TRB = "test and reset the bits the accumulator selects",
    TSB = "test and set the bits the accumulator selects",
    TSX = "transfer the stack pointer to X",
    TXA = "transfer X to the accumulator",
    TXS = "transfer X to the stack pointer",
    TYA = "transfer Y to the accumulator",
    USBC = "subtract with borrow (the undocumented $EB encoding)",
    WAI = "wait for an interrupt",
}

/// What one of the Rockwell-derived bit instructions does, and to which bit.
///
/// `RMB`/`SMB`/`BBR`/`BBS` are thirty-two encodings expressing four operations
/// over eight bit positions. Spelling each one out as its own [`Op`] keeps the
/// rule that a variant name *is* its mnemonic — `BBR3` really is what an
/// assembler writes — and this is how the interpreter gets the bit back out
/// without thirty-two match arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BitOp {
    /// `RMB<n>`: clear bit *n* of the page-zero byte.
    Reset(u8),
    /// `SMB<n>`: set bit *n* of the page-zero byte.
    Set(u8),
    /// `BBR<n>`: branch if bit *n* of the page-zero byte is clear.
    BranchClear(u8),
    /// `BBS<n>`: branch if bit *n* of the page-zero byte is set.
    BranchSet(u8),
}

impl BitOp {
    /// The mask the bit index selects.
    #[inline]
    #[must_use]
    pub const fn mask(self) -> u8 {
        let bit = match self {
            BitOp::Reset(b) | BitOp::Set(b) | BitOp::BranchClear(b) | BitOp::BranchSet(b) => b,
        };
        1u8 << bit
    }
}

impl Op {
    /// Which bit instruction this is, if it is one.
    ///
    /// `None` for every operation outside the CMOS bit group, which is all of
    /// the NMOS matrix. Written out rather than generated: an identifier cannot
    /// be assembled from a prefix and a digit without a proc macro, and the
    /// dependency budget is zero (`ROADMAP.md` §0).
    #[must_use]
    pub const fn bit_op(self) -> Option<BitOp> {
        match self {
            Op::RMB0 => Some(BitOp::Reset(0)),
            Op::RMB1 => Some(BitOp::Reset(1)),
            Op::RMB2 => Some(BitOp::Reset(2)),
            Op::RMB3 => Some(BitOp::Reset(3)),
            Op::RMB4 => Some(BitOp::Reset(4)),
            Op::RMB5 => Some(BitOp::Reset(5)),
            Op::RMB6 => Some(BitOp::Reset(6)),
            Op::RMB7 => Some(BitOp::Reset(7)),
            Op::SMB0 => Some(BitOp::Set(0)),
            Op::SMB1 => Some(BitOp::Set(1)),
            Op::SMB2 => Some(BitOp::Set(2)),
            Op::SMB3 => Some(BitOp::Set(3)),
            Op::SMB4 => Some(BitOp::Set(4)),
            Op::SMB5 => Some(BitOp::Set(5)),
            Op::SMB6 => Some(BitOp::Set(6)),
            Op::SMB7 => Some(BitOp::Set(7)),
            Op::BBR0 => Some(BitOp::BranchClear(0)),
            Op::BBR1 => Some(BitOp::BranchClear(1)),
            Op::BBR2 => Some(BitOp::BranchClear(2)),
            Op::BBR3 => Some(BitOp::BranchClear(3)),
            Op::BBR4 => Some(BitOp::BranchClear(4)),
            Op::BBR5 => Some(BitOp::BranchClear(5)),
            Op::BBR6 => Some(BitOp::BranchClear(6)),
            Op::BBR7 => Some(BitOp::BranchClear(7)),
            Op::BBS0 => Some(BitOp::BranchSet(0)),
            Op::BBS1 => Some(BitOp::BranchSet(1)),
            Op::BBS2 => Some(BitOp::BranchSet(2)),
            Op::BBS3 => Some(BitOp::BranchSet(3)),
            Op::BBS4 => Some(BitOp::BranchSet(4)),
            Op::BBS5 => Some(BitOp::BranchSet(5)),
            Op::BBS6 => Some(BitOp::BranchSet(6)),
            Op::BBS7 => Some(BitOp::BranchSet(7)),
            _ => None,
        }
    }

    /// Whether this operation is a conditional or unconditional branch taking a
    /// [`Mode::Relative`] displacement.
    #[must_use]
    pub const fn is_branch(self) -> bool {
        matches!(
            self,
            Op::BPL | Op::BMI | Op::BVC | Op::BVS | Op::BCC | Op::BCS | Op::BNE | Op::BEQ | Op::BRA
        )
    }
}

/// One row of the instruction description: everything known about an opcode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// How it finds its operand, and therefore how long it is.
    pub mode: Mode,
    /// The shape of the operand access, and therefore its bus timing.
    pub access: Access,
    /// Whether this encoding is documented.
    pub class: Class,
}

impl Insn {
    const fn new(op: Op, mode: Mode, access: Access, class: Class) -> Insn {
        Insn {
            op,
            mode,
            access,
            class,
        }
    }

    /// Instruction length in bytes, opcode included.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        self.mode.bytes()
    }
}

/// Build one decode table and its coverage bitmap from a list of rows.
///
/// Every one of the 256 encodings is written out; the `LISTED` companion
/// records which, so a test can prove the matrix is complete rather than
/// trusting that it is. Invoked twice — once per part — because the two
/// matrices are genuinely different documents, not one with exceptions.
macro_rules! isa {
    (
        $(#[$tdoc:meta])* $table:ident,
        $(#[$ldoc:meta])* $listed:ident,
        $($opcode:literal $op:ident $mode:ident $access:ident $class:ident;)*
    ) => {
        $(#[$tdoc])*
        pub static $table: [Insn; 256] = {
            // The array must be initialised before it can be indexed in a
            // const block; every entry is then overwritten below, which
            // the companion bitmap proves.
            let mut t = [Insn::new(Op::JAM, Mode::Implied, Access::None, Class::Undocumented); 256];
            $(t[$opcode as usize] = Insn::new(Op::$op, Mode::$mode, Access::$access, Class::$class);)*
            t
        };

        $(#[$ldoc])*
        pub static $listed: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

isa! {
    /// The NMOS decode table: one row per opcode, and the only description of
    /// the NMOS instruction set in the crate. Shared by the RP2A03, which
    /// differs in its adder rather than its matrix.
    TABLE,
    /// Which opcodes [`TABLE`] actually assigns. Test scaffolding with a real
    /// job: an unassigned entry would silently decode as `JAM`.
    LISTED,

    0x00 BRK Break       None   Documented;
    0x01 ORA IndirectX   Read   Documented;
    0x02 JAM Implied     None   Undocumented;
    0x03 SLO IndirectX   Modify Undocumented;
    0x04 NOP ZeroPage    Read   Undocumented;
    0x05 ORA ZeroPage    Read   Documented;
    0x06 ASL ZeroPage    Modify Documented;
    0x07 SLO ZeroPage    Modify Undocumented;
    0x08 PHP Implied     None   Documented;
    0x09 ORA Immediate   Read   Documented;
    0x0a ASL Accumulator None   Documented;
    0x0b ANC Immediate   Read   Undocumented;
    0x0c NOP Absolute    Read   Undocumented;
    0x0d ORA Absolute    Read   Documented;
    0x0e ASL Absolute    Modify Documented;
    0x0f SLO Absolute    Modify Undocumented;

    0x10 BPL Relative    None   Documented;
    0x11 ORA IndirectY   Read   Documented;
    0x12 JAM Implied     None   Undocumented;
    0x13 SLO IndirectY   Modify Undocumented;
    0x14 NOP ZeroPageX   Read   Undocumented;
    0x15 ORA ZeroPageX   Read   Documented;
    0x16 ASL ZeroPageX   Modify Documented;
    0x17 SLO ZeroPageX   Modify Undocumented;
    0x18 CLC Implied     None   Documented;
    0x19 ORA AbsoluteY   Read   Documented;
    0x1a NOP Implied     None   Undocumented;
    0x1b SLO AbsoluteY   Modify Undocumented;
    0x1c NOP AbsoluteX   Read   Undocumented;
    0x1d ORA AbsoluteX   Read   Documented;
    0x1e ASL AbsoluteX   Modify Documented;
    0x1f SLO AbsoluteX   Modify Undocumented;

    0x20 JSR Absolute    None   Documented;
    0x21 AND IndirectX   Read   Documented;
    0x22 JAM Implied     None   Undocumented;
    0x23 RLA IndirectX   Modify Undocumented;
    0x24 BIT ZeroPage    Read   Documented;
    0x25 AND ZeroPage    Read   Documented;
    0x26 ROL ZeroPage    Modify Documented;
    0x27 RLA ZeroPage    Modify Undocumented;
    0x28 PLP Implied     None   Documented;
    0x29 AND Immediate   Read   Documented;
    0x2a ROL Accumulator None   Documented;
    0x2b ANC Immediate   Read   Undocumented;
    0x2c BIT Absolute    Read   Documented;
    0x2d AND Absolute    Read   Documented;
    0x2e ROL Absolute    Modify Documented;
    0x2f RLA Absolute    Modify Undocumented;

    0x30 BMI Relative    None   Documented;
    0x31 AND IndirectY   Read   Documented;
    0x32 JAM Implied     None   Undocumented;
    0x33 RLA IndirectY   Modify Undocumented;
    0x34 NOP ZeroPageX   Read   Undocumented;
    0x35 AND ZeroPageX   Read   Documented;
    0x36 ROL ZeroPageX   Modify Documented;
    0x37 RLA ZeroPageX   Modify Undocumented;
    0x38 SEC Implied     None   Documented;
    0x39 AND AbsoluteY   Read   Documented;
    0x3a NOP Implied     None   Undocumented;
    0x3b RLA AbsoluteY   Modify Undocumented;
    0x3c NOP AbsoluteX   Read   Undocumented;
    0x3d AND AbsoluteX   Read   Documented;
    0x3e ROL AbsoluteX   Modify Documented;
    0x3f RLA AbsoluteX   Modify Undocumented;

    0x40 RTI Implied     None   Documented;
    0x41 EOR IndirectX   Read   Documented;
    0x42 JAM Implied     None   Undocumented;
    0x43 SRE IndirectX   Modify Undocumented;
    0x44 NOP ZeroPage    Read   Undocumented;
    0x45 EOR ZeroPage    Read   Documented;
    0x46 LSR ZeroPage    Modify Documented;
    0x47 SRE ZeroPage    Modify Undocumented;
    0x48 PHA Implied     None   Documented;
    0x49 EOR Immediate   Read   Documented;
    0x4a LSR Accumulator None   Documented;
    0x4b ALR Immediate   Read   Undocumented;
    0x4c JMP Absolute    None   Documented;
    0x4d EOR Absolute    Read   Documented;
    0x4e LSR Absolute    Modify Documented;
    0x4f SRE Absolute    Modify Undocumented;

    0x50 BVC Relative    None   Documented;
    0x51 EOR IndirectY   Read   Documented;
    0x52 JAM Implied     None   Undocumented;
    0x53 SRE IndirectY   Modify Undocumented;
    0x54 NOP ZeroPageX   Read   Undocumented;
    0x55 EOR ZeroPageX   Read   Documented;
    0x56 LSR ZeroPageX   Modify Documented;
    0x57 SRE ZeroPageX   Modify Undocumented;
    0x58 CLI Implied     None   Documented;
    0x59 EOR AbsoluteY   Read   Documented;
    0x5a NOP Implied     None   Undocumented;
    0x5b SRE AbsoluteY   Modify Undocumented;
    0x5c NOP AbsoluteX   Read   Undocumented;
    0x5d EOR AbsoluteX   Read   Documented;
    0x5e LSR AbsoluteX   Modify Documented;
    0x5f SRE AbsoluteX   Modify Undocumented;

    0x60 RTS Implied     None   Documented;
    0x61 ADC IndirectX   Read   Documented;
    0x62 JAM Implied     None   Undocumented;
    0x63 RRA IndirectX   Modify Undocumented;
    0x64 NOP ZeroPage    Read   Undocumented;
    0x65 ADC ZeroPage    Read   Documented;
    0x66 ROR ZeroPage    Modify Documented;
    0x67 RRA ZeroPage    Modify Undocumented;
    0x68 PLA Implied     None   Documented;
    0x69 ADC Immediate   Read   Documented;
    0x6a ROR Accumulator None   Documented;
    0x6b ARR Immediate   Read   Undocumented;
    0x6c JMP Indirect    None   Documented;
    0x6d ADC Absolute    Read   Documented;
    0x6e ROR Absolute    Modify Documented;
    0x6f RRA Absolute    Modify Undocumented;

    0x70 BVS Relative    None   Documented;
    0x71 ADC IndirectY   Read   Documented;
    0x72 JAM Implied     None   Undocumented;
    0x73 RRA IndirectY   Modify Undocumented;
    0x74 NOP ZeroPageX   Read   Undocumented;
    0x75 ADC ZeroPageX   Read   Documented;
    0x76 ROR ZeroPageX   Modify Documented;
    0x77 RRA ZeroPageX   Modify Undocumented;
    0x78 SEI Implied     None   Documented;
    0x79 ADC AbsoluteY   Read   Documented;
    0x7a NOP Implied     None   Undocumented;
    0x7b RRA AbsoluteY   Modify Undocumented;
    0x7c NOP AbsoluteX   Read   Undocumented;
    0x7d ADC AbsoluteX   Read   Documented;
    0x7e ROR AbsoluteX   Modify Documented;
    0x7f RRA AbsoluteX   Modify Undocumented;

    0x80 NOP Immediate   Read   Undocumented;
    0x81 STA IndirectX   Write  Documented;
    0x82 NOP Immediate   Read   Undocumented;
    0x83 SAX IndirectX   Write  Undocumented;
    0x84 STY ZeroPage    Write  Documented;
    0x85 STA ZeroPage    Write  Documented;
    0x86 STX ZeroPage    Write  Documented;
    0x87 SAX ZeroPage    Write  Undocumented;
    0x88 DEY Implied     None   Documented;
    0x89 NOP Immediate   Read   Undocumented;
    0x8a TXA Implied     None   Documented;
    0x8b ANE Immediate   Read   Unstable;
    0x8c STY Absolute    Write  Documented;
    0x8d STA Absolute    Write  Documented;
    0x8e STX Absolute    Write  Documented;
    0x8f SAX Absolute    Write  Undocumented;

    0x90 BCC Relative    None   Documented;
    0x91 STA IndirectY   Write  Documented;
    0x92 JAM Implied     None   Undocumented;
    0x93 SHA IndirectY   Write  Unstable;
    0x94 STY ZeroPageX   Write  Documented;
    0x95 STA ZeroPageX   Write  Documented;
    0x96 STX ZeroPageY   Write  Documented;
    0x97 SAX ZeroPageY   Write  Undocumented;
    0x98 TYA Implied     None   Documented;
    0x99 STA AbsoluteY   Write  Documented;
    0x9a TXS Implied     None   Documented;
    0x9b TAS AbsoluteY   Write  Unstable;
    0x9c SHY AbsoluteX   Write  Unstable;
    0x9d STA AbsoluteX   Write  Documented;
    0x9e SHX AbsoluteY   Write  Unstable;
    0x9f SHA AbsoluteY   Write  Unstable;

    0xa0 LDY Immediate   Read   Documented;
    0xa1 LDA IndirectX   Read   Documented;
    0xa2 LDX Immediate   Read   Documented;
    0xa3 LAX IndirectX   Read   Undocumented;
    0xa4 LDY ZeroPage    Read   Documented;
    0xa5 LDA ZeroPage    Read   Documented;
    0xa6 LDX ZeroPage    Read   Documented;
    0xa7 LAX ZeroPage    Read   Undocumented;
    0xa8 TAY Implied     None   Documented;
    0xa9 LDA Immediate   Read   Documented;
    0xaa TAX Implied     None   Documented;
    0xab LXA Immediate   Read   Unstable;
    0xac LDY Absolute    Read   Documented;
    0xad LDA Absolute    Read   Documented;
    0xae LDX Absolute    Read   Documented;
    0xaf LAX Absolute    Read   Undocumented;

    0xb0 BCS Relative    None   Documented;
    0xb1 LDA IndirectY   Read   Documented;
    0xb2 JAM Implied     None   Undocumented;
    0xb3 LAX IndirectY   Read   Undocumented;
    0xb4 LDY ZeroPageX   Read   Documented;
    0xb5 LDA ZeroPageX   Read   Documented;
    0xb6 LDX ZeroPageY   Read   Documented;
    0xb7 LAX ZeroPageY   Read   Undocumented;
    0xb8 CLV Implied     None   Documented;
    0xb9 LDA AbsoluteY   Read   Documented;
    0xba TSX Implied     None   Documented;
    0xbb LAS AbsoluteY   Read   Undocumented;
    0xbc LDY AbsoluteX   Read   Documented;
    0xbd LDA AbsoluteX   Read   Documented;
    0xbe LDX AbsoluteY   Read   Documented;
    0xbf LAX AbsoluteY   Read   Undocumented;

    0xc0 CPY Immediate   Read   Documented;
    0xc1 CMP IndirectX   Read   Documented;
    0xc2 NOP Immediate   Read   Undocumented;
    0xc3 DCP IndirectX   Modify Undocumented;
    0xc4 CPY ZeroPage    Read   Documented;
    0xc5 CMP ZeroPage    Read   Documented;
    0xc6 DEC ZeroPage    Modify Documented;
    0xc7 DCP ZeroPage    Modify Undocumented;
    0xc8 INY Implied     None   Documented;
    0xc9 CMP Immediate   Read   Documented;
    0xca DEX Implied     None   Documented;
    0xcb SBX Immediate   Read   Undocumented;
    0xcc CPY Absolute    Read   Documented;
    0xcd CMP Absolute    Read   Documented;
    0xce DEC Absolute    Modify Documented;
    0xcf DCP Absolute    Modify Undocumented;

    0xd0 BNE Relative    None   Documented;
    0xd1 CMP IndirectY   Read   Documented;
    0xd2 JAM Implied     None   Undocumented;
    0xd3 DCP IndirectY   Modify Undocumented;
    0xd4 NOP ZeroPageX   Read   Undocumented;
    0xd5 CMP ZeroPageX   Read   Documented;
    0xd6 DEC ZeroPageX   Modify Documented;
    0xd7 DCP ZeroPageX   Modify Undocumented;
    0xd8 CLD Implied     None   Documented;
    0xd9 CMP AbsoluteY   Read   Documented;
    0xda NOP Implied     None   Undocumented;
    0xdb DCP AbsoluteY   Modify Undocumented;
    0xdc NOP AbsoluteX   Read   Undocumented;
    0xdd CMP AbsoluteX   Read   Documented;
    0xde DEC AbsoluteX   Modify Documented;
    0xdf DCP AbsoluteX   Modify Undocumented;

    0xe0 CPX Immediate   Read   Documented;
    0xe1 SBC IndirectX   Read   Documented;
    0xe2 NOP Immediate   Read   Undocumented;
    0xe3 ISC IndirectX   Modify Undocumented;
    0xe4 CPX ZeroPage    Read   Documented;
    0xe5 SBC ZeroPage    Read   Documented;
    0xe6 INC ZeroPage    Modify Documented;
    0xe7 ISC ZeroPage    Modify Undocumented;
    0xe8 INX Implied     None   Documented;
    0xe9 SBC Immediate   Read   Documented;
    0xea NOP Implied     None   Documented;
    0xeb USBC Immediate  Read   Undocumented;
    0xec CPX Absolute    Read   Documented;
    0xed SBC Absolute    Read   Documented;
    0xee INC Absolute    Modify Documented;
    0xef ISC Absolute    Modify Undocumented;

    0xf0 BEQ Relative    None   Documented;
    0xf1 SBC IndirectY   Read   Documented;
    0xf2 JAM Implied     None   Undocumented;
    0xf3 ISC IndirectY   Modify Undocumented;
    0xf4 NOP ZeroPageX   Read   Undocumented;
    0xf5 SBC ZeroPageX   Read   Documented;
    0xf6 INC ZeroPageX   Modify Documented;
    0xf7 ISC ZeroPageX   Modify Undocumented;
    0xf8 SED Implied     None   Documented;
    0xf9 SBC AbsoluteY   Read   Documented;
    0xfa NOP Implied     None   Undocumented;
    0xfb ISC AbsoluteY   Modify Undocumented;
    0xfc NOP AbsoluteX   Read   Undocumented;
    0xfd SBC AbsoluteX   Read   Documented;
    0xfe INC AbsoluteX   Modify Documented;
    0xff ISC AbsoluteX   Modify Undocumented;
}

isa! {
    /// The CMOS decode table: the W65C02S matrix, from the WDC datasheet.
    ///
    /// Every one of the 256 encodings is defined and documented — the CMOS part
    /// has no undocumented opcodes. What the NMOS matrix left as `JAM` and as
    /// undocumented ALU combinations, this one fills with NOPs of a *specified*
    /// length and cycle count, which is why so many rows carry a mode the NMOS
    /// table never gives `NOP`: `$5c`, `$dc` and `$fc` really are three bytes
    /// long, and `$x3`/`$xB` really do finish in one cycle.
    CMOS_TABLE,
    /// Which opcodes [`CMOS_TABLE`] assigns. See [`LISTED`].
    CMOS_LISTED,

    0x00 BRK Break             None   Documented;
    0x01 ORA IndirectX         Read   Documented;
    0x02 NOP Immediate         Read   Documented;
    0x03 NOP Single            None   Documented;
    0x04 TSB ZeroPage          Modify Documented;
    0x05 ORA ZeroPage          Read   Documented;
    0x06 ASL ZeroPage          Modify Documented;
    0x07 RMB0 ZeroPage         Modify Documented;
    0x08 PHP Implied           None   Documented;
    0x09 ORA Immediate         Read   Documented;
    0x0a ASL Accumulator       None   Documented;
    0x0b NOP Single            None   Documented;
    0x0c TSB Absolute          Modify Documented;
    0x0d ORA Absolute          Read   Documented;
    0x0e ASL Absolute          Modify Documented;
    0x0f BBR0 ZeroPageRelative None   Documented;

    0x10 BPL Relative          None   Documented;
    0x11 ORA IndirectY         Read   Documented;
    0x12 ORA ZeroPageIndirect  Read   Documented;
    0x13 NOP Single            None   Documented;
    0x14 TRB ZeroPage          Modify Documented;
    0x15 ORA ZeroPageX         Read   Documented;
    0x16 ASL ZeroPageX         Modify Documented;
    0x17 RMB1 ZeroPage         Modify Documented;
    0x18 CLC Implied           None   Documented;
    0x19 ORA AbsoluteY         Read   Documented;
    0x1a INC Accumulator       None   Documented;
    0x1b NOP Single            None   Documented;
    0x1c TRB Absolute          Modify Documented;
    0x1d ORA AbsoluteX         Read   Documented;
    0x1e ASL AbsoluteX         Modify Documented;
    0x1f BBR1 ZeroPageRelative None   Documented;

    0x20 JSR Absolute          None   Documented;
    0x21 AND IndirectX         Read   Documented;
    0x22 NOP Immediate         Read   Documented;
    0x23 NOP Single            None   Documented;
    0x24 BIT ZeroPage          Read   Documented;
    0x25 AND ZeroPage          Read   Documented;
    0x26 ROL ZeroPage          Modify Documented;
    0x27 RMB2 ZeroPage         Modify Documented;
    0x28 PLP Implied           None   Documented;
    0x29 AND Immediate         Read   Documented;
    0x2a ROL Accumulator       None   Documented;
    0x2b NOP Single            None   Documented;
    0x2c BIT Absolute          Read   Documented;
    0x2d AND Absolute          Read   Documented;
    0x2e ROL Absolute          Modify Documented;
    0x2f BBR2 ZeroPageRelative None   Documented;

    0x30 BMI Relative          None   Documented;
    0x31 AND IndirectY         Read   Documented;
    0x32 AND ZeroPageIndirect  Read   Documented;
    0x33 NOP Single            None   Documented;
    0x34 BIT ZeroPageX         Read   Documented;
    0x35 AND ZeroPageX         Read   Documented;
    0x36 ROL ZeroPageX         Modify Documented;
    0x37 RMB3 ZeroPage         Modify Documented;
    0x38 SEC Implied           None   Documented;
    0x39 AND AbsoluteY         Read   Documented;
    0x3a DEC Accumulator       None   Documented;
    0x3b NOP Single            None   Documented;
    0x3c BIT AbsoluteX         Read   Documented;
    0x3d AND AbsoluteX         Read   Documented;
    0x3e ROL AbsoluteX         Modify Documented;
    0x3f BBR3 ZeroPageRelative None   Documented;

    0x40 RTI Implied           None   Documented;
    0x41 EOR IndirectX         Read   Documented;
    0x42 NOP Immediate         Read   Documented;
    0x43 NOP Single            None   Documented;
    0x44 NOP ZeroPage          Read   Documented;
    0x45 EOR ZeroPage          Read   Documented;
    0x46 LSR ZeroPage          Modify Documented;
    0x47 RMB4 ZeroPage         Modify Documented;
    0x48 PHA Implied           None   Documented;
    0x49 EOR Immediate         Read   Documented;
    0x4a LSR Accumulator       None   Documented;
    0x4b NOP Single            None   Documented;
    0x4c JMP Absolute          None   Documented;
    0x4d EOR Absolute          Read   Documented;
    0x4e LSR Absolute          Modify Documented;
    0x4f BBR4 ZeroPageRelative None   Documented;

    0x50 BVC Relative          None   Documented;
    0x51 EOR IndirectY         Read   Documented;
    0x52 EOR ZeroPageIndirect  Read   Documented;
    0x53 NOP Single            None   Documented;
    0x54 NOP ZeroPageX         Read   Documented;
    0x55 EOR ZeroPageX         Read   Documented;
    0x56 LSR ZeroPageX         Modify Documented;
    0x57 RMB5 ZeroPage         Modify Documented;
    0x58 CLI Implied           None   Documented;
    0x59 EOR AbsoluteY         Read   Documented;
    0x5a PHY Implied           None   Documented;
    0x5b NOP Single            None   Documented;
    0x5c NOP Absolute          None   Documented;
    0x5d EOR AbsoluteX         Read   Documented;
    0x5e LSR AbsoluteX         Modify Documented;
    0x5f BBR5 ZeroPageRelative None   Documented;

    0x60 RTS Implied           None   Documented;
    0x61 ADC IndirectX         Read   Documented;
    0x62 NOP Immediate         Read   Documented;
    0x63 NOP Single            None   Documented;
    0x64 STZ ZeroPage          Write  Documented;
    0x65 ADC ZeroPage          Read   Documented;
    0x66 ROR ZeroPage          Modify Documented;
    0x67 RMB6 ZeroPage         Modify Documented;
    0x68 PLA Implied           None   Documented;
    0x69 ADC Immediate         Read   Documented;
    0x6a ROR Accumulator       None   Documented;
    0x6b NOP Single            None   Documented;
    0x6c JMP Indirect          None   Documented;
    0x6d ADC Absolute          Read   Documented;
    0x6e ROR Absolute          Modify Documented;
    0x6f BBR6 ZeroPageRelative None   Documented;

    0x70 BVS Relative          None   Documented;
    0x71 ADC IndirectY         Read   Documented;
    0x72 ADC ZeroPageIndirect  Read   Documented;
    0x73 NOP Single            None   Documented;
    0x74 STZ ZeroPageX         Write  Documented;
    0x75 ADC ZeroPageX         Read   Documented;
    0x76 ROR ZeroPageX         Modify Documented;
    0x77 RMB7 ZeroPage         Modify Documented;
    0x78 SEI Implied           None   Documented;
    0x79 ADC AbsoluteY         Read   Documented;
    0x7a PLY Implied           None   Documented;
    0x7b NOP Single            None   Documented;
    0x7c JMP AbsoluteIndirectX None   Documented;
    0x7d ADC AbsoluteX         Read   Documented;
    0x7e ROR AbsoluteX         Modify Documented;
    0x7f BBR7 ZeroPageRelative None   Documented;

    0x80 BRA Relative          None   Documented;
    0x81 STA IndirectX         Write  Documented;
    0x82 NOP Immediate         Read   Documented;
    0x83 NOP Single            None   Documented;
    0x84 STY ZeroPage          Write  Documented;
    0x85 STA ZeroPage          Write  Documented;
    0x86 STX ZeroPage          Write  Documented;
    0x87 SMB0 ZeroPage         Modify Documented;
    0x88 DEY Implied           None   Documented;
    0x89 BIT Immediate         Read   Documented;
    0x8a TXA Implied           None   Documented;
    0x8b NOP Single            None   Documented;
    0x8c STY Absolute          Write  Documented;
    0x8d STA Absolute          Write  Documented;
    0x8e STX Absolute          Write  Documented;
    0x8f BBS0 ZeroPageRelative None   Documented;

    0x90 BCC Relative          None   Documented;
    0x91 STA IndirectY         Write  Documented;
    0x92 STA ZeroPageIndirect  Write  Documented;
    0x93 NOP Single            None   Documented;
    0x94 STY ZeroPageX         Write  Documented;
    0x95 STA ZeroPageX         Write  Documented;
    0x96 STX ZeroPageY         Write  Documented;
    0x97 SMB1 ZeroPage         Modify Documented;
    0x98 TYA Implied           None   Documented;
    0x99 STA AbsoluteY         Write  Documented;
    0x9a TXS Implied           None   Documented;
    0x9b NOP Single            None   Documented;
    0x9c STZ Absolute          Write  Documented;
    0x9d STA AbsoluteX         Write  Documented;
    0x9e STZ AbsoluteX         Write  Documented;
    0x9f BBS1 ZeroPageRelative None   Documented;

    0xa0 LDY Immediate         Read   Documented;
    0xa1 LDA IndirectX         Read   Documented;
    0xa2 LDX Immediate         Read   Documented;
    0xa3 NOP Single            None   Documented;
    0xa4 LDY ZeroPage          Read   Documented;
    0xa5 LDA ZeroPage          Read   Documented;
    0xa6 LDX ZeroPage          Read   Documented;
    0xa7 SMB2 ZeroPage         Modify Documented;
    0xa8 TAY Implied           None   Documented;
    0xa9 LDA Immediate         Read   Documented;
    0xaa TAX Implied           None   Documented;
    0xab NOP Single            None   Documented;
    0xac LDY Absolute          Read   Documented;
    0xad LDA Absolute          Read   Documented;
    0xae LDX Absolute          Read   Documented;
    0xaf BBS2 ZeroPageRelative None   Documented;

    0xb0 BCS Relative          None   Documented;
    0xb1 LDA IndirectY         Read   Documented;
    0xb2 LDA ZeroPageIndirect  Read   Documented;
    0xb3 NOP Single            None   Documented;
    0xb4 LDY ZeroPageX         Read   Documented;
    0xb5 LDA ZeroPageX         Read   Documented;
    0xb6 LDX ZeroPageY         Read   Documented;
    0xb7 SMB3 ZeroPage         Modify Documented;
    0xb8 CLV Implied           None   Documented;
    0xb9 LDA AbsoluteY         Read   Documented;
    0xba TSX Implied           None   Documented;
    0xbb NOP Single            None   Documented;
    0xbc LDY AbsoluteX         Read   Documented;
    0xbd LDA AbsoluteX         Read   Documented;
    0xbe LDX AbsoluteY         Read   Documented;
    0xbf BBS3 ZeroPageRelative None   Documented;

    0xc0 CPY Immediate         Read   Documented;
    0xc1 CMP IndirectX         Read   Documented;
    0xc2 NOP Immediate         Read   Documented;
    0xc3 NOP Single            None   Documented;
    0xc4 CPY ZeroPage          Read   Documented;
    0xc5 CMP ZeroPage          Read   Documented;
    0xc6 DEC ZeroPage          Modify Documented;
    0xc7 SMB4 ZeroPage         Modify Documented;
    0xc8 INY Implied           None   Documented;
    0xc9 CMP Immediate         Read   Documented;
    0xca DEX Implied           None   Documented;
    0xcb WAI Implied           None   Documented;
    0xcc CPY Absolute          Read   Documented;
    0xcd CMP Absolute          Read   Documented;
    0xce DEC Absolute          Modify Documented;
    0xcf BBS4 ZeroPageRelative None   Documented;

    0xd0 BNE Relative          None   Documented;
    0xd1 CMP IndirectY         Read   Documented;
    0xd2 CMP ZeroPageIndirect  Read   Documented;
    0xd3 NOP Single            None   Documented;
    0xd4 NOP ZeroPageX         Read   Documented;
    0xd5 CMP ZeroPageX         Read   Documented;
    0xd6 DEC ZeroPageX         Modify Documented;
    0xd7 SMB5 ZeroPage         Modify Documented;
    0xd8 CLD Implied           None   Documented;
    0xd9 CMP AbsoluteY         Read   Documented;
    0xda PHX Implied           None   Documented;
    0xdb STP Implied           None   Documented;
    0xdc NOP Absolute          None   Documented;
    0xdd CMP AbsoluteX         Read   Documented;
    0xde DEC AbsoluteX         Modify Documented;
    0xdf BBS5 ZeroPageRelative None   Documented;

    0xe0 CPX Immediate         Read   Documented;
    0xe1 SBC IndirectX         Read   Documented;
    0xe2 NOP Immediate         Read   Documented;
    0xe3 NOP Single            None   Documented;
    0xe4 CPX ZeroPage          Read   Documented;
    0xe5 SBC ZeroPage          Read   Documented;
    0xe6 INC ZeroPage          Modify Documented;
    0xe7 SMB6 ZeroPage         Modify Documented;
    0xe8 INX Implied           None   Documented;
    0xe9 SBC Immediate         Read   Documented;
    0xea NOP Implied           None   Documented;
    0xeb NOP Single            None   Documented;
    0xec CPX Absolute          Read   Documented;
    0xed SBC Absolute          Read   Documented;
    0xee INC Absolute          Modify Documented;
    0xef BBS6 ZeroPageRelative None   Documented;

    0xf0 BEQ Relative          None   Documented;
    0xf1 SBC IndirectY         Read   Documented;
    0xf2 SBC ZeroPageIndirect  Read   Documented;
    0xf3 NOP Single            None   Documented;
    0xf4 NOP ZeroPageX         Read   Documented;
    0xf5 SBC ZeroPageX         Read   Documented;
    0xf6 INC ZeroPageX         Modify Documented;
    0xf7 SMB7 ZeroPage         Modify Documented;
    0xf8 SED Implied           None   Documented;
    0xf9 SBC AbsoluteY         Read   Documented;
    0xfa PLX Implied           None   Documented;
    0xfb NOP Single            None   Documented;
    0xfc NOP Absolute          None   Documented;
    0xfd SBC AbsoluteX         Read   Documented;
    0xfe INC AbsoluteX         Modify Documented;
    0xff BBS7 ZeroPageRelative None   Documented;
}

/// Decode one opcode byte as the NMOS part sees it.
///
/// Total: every one of the 256 encodings is defined, undocumented ones
/// included, so decoding never fails. Use [`decode_as`] where the part matters
/// — this is the NMOS matrix, and the CMOS one disagrees about a third of it.
#[inline]
#[must_use]
pub const fn decode(opcode: u8) -> Insn {
    TABLE[opcode as usize]
}

/// Decode one opcode byte as `variant` sees it.
///
/// Also total, for the same reason on both parts: the NMOS matrix defines every
/// encoding because the undocumented ones are in scope, and the CMOS matrix
/// defines every encoding because WDC specified the leftovers as NOPs.
#[inline]
#[must_use]
pub const fn decode_as(variant: Variant, opcode: u8) -> Insn {
    match variant {
        Variant::Wdc65C02 => CMOS_TABLE[opcode as usize],
        _ => TABLE[opcode as usize],
    }
}

/// The whole table one variant decodes with.
#[inline]
#[must_use]
pub const fn table(variant: Variant) -> &'static [Insn; 256] {
    match variant {
        Variant::Wdc65C02 => &CMOS_TABLE,
        _ => &TABLE,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn every_opcode_is_described_exactly_once() {
        // The table initialiser fills with JAM; an omission would decode as a
        // plausible-looking halt rather than fail, which is the worst kind of
        // silence.
        let missing: Vec<usize> = (0..256).filter(|i| !LISTED[*i]).collect();
        assert!(missing.is_empty(), "opcodes not described: {missing:02x?}");
    }

    #[test]
    fn the_documented_matrix_has_the_expected_size() {
        // 151 documented encodings is the number every 6502 reference agrees
        // on; if a row is mistyped as Documented this catches it.
        let documented = TABLE.iter().filter(|i| i.class.is_documented()).count();
        assert_eq!(documented, 151);
        let jams = TABLE.iter().filter(|i| i.op == Op::JAM).count();
        assert_eq!(jams, 12);
    }

    #[test]
    fn mnemonics_match_their_variant_names() {
        assert_eq!(Op::LDA.mnemonic(), "LDA");
        assert_eq!(Op::USBC.mnemonic(), "USBC");
        for op in Op::ALL {
            assert!(!op.summary().is_empty(), "{op:?} has no summary");
            assert!(op.mnemonic().len() >= 3, "{op:?}");
        }
    }

    #[test]
    fn every_declared_operation_is_reachable_from_some_opcode() {
        // An operation nothing decodes to is dead code pretending to be a
        // feature. Either table may reach it — half of these exist only on the
        // CMOS part and half only on the NMOS one.
        for op in Op::ALL {
            assert!(
                TABLE.iter().chain(CMOS_TABLE.iter()).any(|i| i.op == *op),
                "{op:?} is declared but unreachable"
            );
        }
    }

    #[test]
    fn the_cmos_matrix_is_complete_and_entirely_documented() {
        let missing: Vec<usize> = (0..256).filter(|i| !CMOS_LISTED[*i]).collect();
        assert!(missing.is_empty(), "opcodes not described: {missing:02x?}");
        // WDC specifies every leftover encoding as a NOP with a stated length
        // and cycle count, so there is nothing undocumented left to flag.
        for (opcode, insn) in CMOS_TABLE.iter().enumerate() {
            assert!(
                insn.class.is_documented(),
                "{opcode:02x} {:?} is not documented on the CMOS part",
                insn.op
            );
            assert_ne!(insn.op, Op::JAM, "{opcode:02x} still jams");
        }
    }

    #[test]
    fn the_cmos_matrix_fills_its_holes_with_nops_of_three_shapes() {
        // The datasheet's leftovers are not all one byte: columns $x3 and $xB
        // finish inside the opcode fetch, $x2 takes an operand it ignores, and
        // $5c/$dc/$fc are three bytes long.
        let single = (0..=255u8)
            .filter(|o| decode_as(Variant::Wdc65C02, *o).mode == Mode::Single)
            .count();
        assert_eq!(single, 30, "all sixteen of $x3 and $xB, less WAI and STP");
        assert_eq!(decode_as(Variant::Wdc65C02, 0x02).mode, Mode::Immediate);
        assert_eq!(decode_as(Variant::Wdc65C02, 0x44).mode, Mode::ZeroPage);
        assert_eq!(decode_as(Variant::Wdc65C02, 0x54).mode, Mode::ZeroPageX);
        for three in [0x5cu8, 0xdc, 0xfc] {
            let insn = decode_as(Variant::Wdc65C02, three);
            assert_eq!(insn.op, Op::NOP);
            assert_eq!(insn.bytes(), 3, "{three:02x}");
        }
    }

    #[test]
    fn the_bit_group_maps_back_to_its_bit() {
        assert_eq!(Op::RMB0.bit_op(), Some(BitOp::Reset(0)));
        assert_eq!(Op::SMB7.bit_op(), Some(BitOp::Set(7)));
        assert_eq!(Op::BBR3.bit_op(), Some(BitOp::BranchClear(3)));
        assert_eq!(Op::BBS5.bit_op(), Some(BitOp::BranchSet(5)));
        assert_eq!(Op::BBS5.bit_op().unwrap().mask(), 0x20);
        assert_eq!(Op::LDA.bit_op(), None);
        // The encodings are regular: RMB<n> at $x7 for x even, SMB<n> for x
        // odd, BBR/BBS the same way at $xF.
        for bit in 0..8u8 {
            let rmb = 0x07 | (bit << 4);
            let smb = 0x87 | (bit << 4);
            let bbr = 0x0f | (bit << 4);
            let bbs = 0x8f | (bit << 4);
            let of = |o: u8| decode_as(Variant::Wdc65C02, o).op.bit_op();
            assert_eq!(of(rmb), Some(BitOp::Reset(bit)), "{rmb:02x}");
            assert_eq!(of(smb), Some(BitOp::Set(bit)), "{smb:02x}");
            assert_eq!(of(bbr), Some(BitOp::BranchClear(bit)), "{bbr:02x}");
            assert_eq!(of(bbs), Some(BitOp::BranchSet(bit)), "{bbs:02x}");
        }
    }

    #[test]
    fn a_variant_names_itself_the_way_a_machine_file_spells_it() {
        for v in Variant::ALL {
            assert_eq!(Variant::from_name(v.name()), Some(*v));
            assert!(!v.summary().is_empty());
        }
        assert_eq!(Variant::from_name("z80"), None);
        assert!(Variant::Wdc65C02.is_cmos());
        assert!(!Variant::Nmos6502.is_cmos());
        assert!(!Variant::Ricoh2A03.is_cmos());
        // The NES's part is the NMOS matrix; only its adder differs.
        assert_eq!(table(Variant::Ricoh2A03), table(Variant::Nmos6502));
    }

    #[test]
    fn the_cmos_part_replaces_the_nmos_undocumented_encodings() {
        // The three the Wozmon delay loop and every 65C02 assembler depend on.
        assert_eq!(decode_as(Variant::Wdc65C02, 0x3a).op, Op::DEC);
        assert_eq!(decode_as(Variant::Wdc65C02, 0x3a).mode, Mode::Accumulator);
        assert_eq!(decode_as(Variant::Wdc65C02, 0x1a).op, Op::INC);
        assert_eq!(decode_as(Variant::Wdc65C02, 0x80).op, Op::BRA);
        // ... and the same bytes on the NMOS part are still what they were.
        assert_eq!(decode(0x3a).op, Op::NOP);
        assert_eq!(decode(0x1a).op, Op::NOP);
        assert_eq!(decode(0x80).op, Op::NOP);
    }

    #[test]
    fn lengths_follow_the_addressing_mode() {
        assert_eq!(decode(0xea).bytes(), 1); // NOP
        assert_eq!(decode(0xa9).bytes(), 2); // LDA #
        assert_eq!(decode(0xad).bytes(), 3); // LDA abs
        assert_eq!(decode(0x00).bytes(), 2); // BRK skips its signature byte
        assert_eq!(decode(0x6c).bytes(), 3); // JMP (ind)
    }

    #[test]
    fn access_shape_matches_the_instruction_class() {
        assert_eq!(decode(0xbd).access, Access::Read); // LDA abs,X
        assert_eq!(decode(0x9d).access, Access::Write); // STA abs,X
        assert_eq!(decode(0xfe).access, Access::Modify); // INC abs,X
        assert_eq!(decode(0xff).access, Access::Modify); // ISC abs,X
        assert_eq!(decode(0x4c).access, Access::None); // JMP abs
    }

    #[test]
    fn the_unstable_encodings_are_the_documented_six() {
        let unstable: Vec<u8> = (0..=255u8)
            .filter(|o| decode(*o).class == Class::Unstable)
            .collect();
        assert_eq!(unstable, [0x8b, 0x93, 0x9b, 0x9c, 0x9e, 0x9f, 0xab]);
    }
}
