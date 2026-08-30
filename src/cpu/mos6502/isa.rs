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
//! # Sources
//!
//! The opcode matrix, addressing modes and undocumented encodings are from the
//! masswerk 6502 instruction set reference, the NESdev "Obelisk" reference and
//! NESdev's *CPU unofficial opcodes* page (`docs/cpu/6502.md`), cross-checked
//! against `../gones/cpu6502` (ours, MIT, © Mark Karpelès).

use core::fmt;

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
            Mode::Implied | Mode::Accumulator => 1,
            Mode::Immediate
            | Mode::ZeroPage
            | Mode::ZeroPageX
            | Mode::ZeroPageY
            | Mode::IndirectX
            | Mode::IndirectY
            | Mode::Relative
            | Mode::Break => 2,
            Mode::Absolute | Mode::AbsoluteX | Mode::AbsoluteY | Mode::Indirect => 3,
        }
    }

    /// Whether this mode is indexed by X or Y, and so can cross a page.
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
            Mode::Implied => "impl",
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
    BCC = "branch if carry clear",
    BCS = "branch if carry set",
    BEQ = "branch if equal (zero set)",
    BIT = "test bits in memory against the accumulator",
    BMI = "branch if minus (negative set)",
    BNE = "branch if not equal (zero clear)",
    BPL = "branch if plus (negative clear)",
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
    PLA = "pull the accumulator",
    PLP = "pull the processor status",
    RLA = "rotate memory left, then AND it with the accumulator",
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
    SRE = "shift memory right, then exclusive-OR it with the accumulator",
    STA = "store the accumulator",
    STX = "store X",
    STY = "store Y",
    TAS = "unstable: A AND X into S, then store it AND (address high + 1)",
    TAX = "transfer the accumulator to X",
    TAY = "transfer the accumulator to Y",
    TSX = "transfer the stack pointer to X",
    TXA = "transfer X to the accumulator",
    TXS = "transfer X to the stack pointer",
    TYA = "transfer Y to the accumulator",
    USBC = "subtract with borrow (the undocumented $EB encoding)",
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

/// Build [`TABLE`] and [`LISTED`] from one list of rows.
///
/// Every one of the 256 encodings is written out; `LISTED` records which, so a
/// test can prove the matrix is complete rather than trusting that it is.
macro_rules! isa {
    ($($opcode:literal $op:ident $mode:ident $access:ident $class:ident;)*) => {
        /// The decode table: one row per opcode, and the only description of
        /// the instruction set in the crate.
        pub static TABLE: [Insn; 256] = {
            // The array must be initialised before it can be indexed in a
            // const block; every entry is then overwritten below, which
            // `LISTED` proves.
            let mut t = [Insn::new(Op::JAM, Mode::Implied, Access::None, Class::Undocumented); 256];
            $(t[$opcode as usize] = Insn::new(Op::$op, Mode::$mode, Access::$access, Class::$class);)*
            t
        };

        /// Which opcodes the table above actually assigns. Test scaffolding
        /// with a real job: an unassigned entry would silently decode as
        /// `JAM`.
        pub static LISTED: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

isa! {
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

/// Decode one opcode byte.
///
/// Total: every one of the 256 encodings is defined, undocumented ones
/// included, so decoding never fails.
#[inline]
#[must_use]
pub const fn decode(opcode: u8) -> Insn {
    TABLE[opcode as usize]
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
        // feature.
        for op in Op::ALL {
            assert!(
                TABLE.iter().any(|i| i.op == *op),
                "{op:?} is declared but unreachable"
            );
        }
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
