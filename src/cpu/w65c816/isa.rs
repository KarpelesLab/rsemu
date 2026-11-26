//! The 65C816 instruction set, described **once**.
//!
//! The same rule `cpu::mos6502` follows (CLAUDE.md, *CPU cores*): one
//! declarative table, [`TABLE`], from which the interpreter's decode, the
//! disassembler ([`super::disasm`]) and `rsemu describe` all read. A
//! disassembler that names an instruction the interpreter does not implement
//! cannot be written, because there is nowhere else to write it.
//!
//! # No cycle column, and no length column either
//!
//! A 65C816 cycle is a bus cycle, and the count for `LDA $12,X` depends on the
//! low byte of the direct-page register and on the width of the accumulator —
//! properties of the machine, not of the opcode. Worse, so does the
//! *length*: `LDA #` is two bytes with an 8-bit accumulator and three with a
//! 16-bit one, which is the single most famous trap in disassembling 65816
//! code. So [`Mode::operand_bytes`] takes the M and X flags, and the
//! disassembler has to be told them.
//!
//! [`Access`] says what shape the operand access has, and the interpreter
//! turns that into the exact sequence of reads, writes and internal
//! operations the datasheet tabulates.
//!
//! # Sources
//!
//! The opcode matrix and addressing modes are from the **W65C816S datasheet**
//! (Western Design Center), §4 *Addressing Modes* and table 5-4/5-5 *Opcode
//! Matrix*, cross-checked against Eyes & Lichty, *Programming the 65816*,
//! chapter 18 and its instruction pages. No emulator source was consulted.

use core::fmt;

/// How an instruction finds its operand.
///
/// The names follow the datasheet's table 5-1, which is also the order its
/// cycle-by-cycle table 5-7 uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// No operand: `INX`, `CLC`, `RTS`, and every push and pull.
    Implied,
    /// The accumulator is the operand: `ASL A`.
    Accumulator,
    /// An immediate as wide as the accumulator: `LDA #`, `ADC #`, `BIT #`.
    ImmediateM,
    /// An immediate as wide as the index registers: `LDX #`, `CPY #`.
    ImmediateX,
    /// A one-byte immediate whatever the flags say: `REP #`, `SEP #`, and the
    /// signature byte of `BRK`, `COP` and `WDM`.
    Immediate8,
    /// `a`: a 16-bit address in the data bank. `JMP`/`JSR` use it in the
    /// program bank instead, and `PEA` pushes it.
    Absolute,
    /// `a,x`.
    AbsoluteX,
    /// `a,y`.
    AbsoluteY,
    /// `al`: a full 24-bit address.
    AbsoluteLong,
    /// `al,x`.
    AbsoluteLongX,
    /// `(a)`: `JMP ($1234)`, pointer in bank 0.
    AbsoluteIndirect,
    /// `[a]`: `JML [$1234]`, a 24-bit pointer in bank 0.
    AbsoluteIndirectLong,
    /// `(a,x)`: `JMP`/`JSR ($1234,X)`, pointer in the program bank.
    AbsoluteIndexedIndirect,
    /// `d`: an offset into the direct page.
    Direct,
    /// `d,x`.
    DirectX,
    /// `d,y`.
    DirectY,
    /// `(d)`.
    DirectIndirect,
    /// `[d]`: a 24-bit pointer on the direct page.
    DirectIndirectLong,
    /// `(d,x)`.
    DirectIndexedIndirect,
    /// `(d),y`.
    DirectIndirectIndexed,
    /// `[d],y`.
    DirectIndirectLongIndexed,
    /// `d,s`: an offset from the stack pointer.
    StackRelative,
    /// `(d,s),y`.
    StackRelativeIndirectIndexed,
    /// A signed 8-bit displacement from the next instruction.
    Relative,
    /// A signed 16-bit displacement from the next instruction: `BRL`, `PER`.
    RelativeLong,
    /// Two bank bytes: `MVN`/`MVP`. Object code order is destination then
    /// source; assembler order is source then destination.
    BlockMove,
}

impl Mode {
    /// Operand bytes after the opcode, given the M and X flags.
    ///
    /// `m8` and `x8` are true when the accumulator and index registers are
    /// eight bits wide — always so in emulation mode.
    #[must_use]
    pub const fn operand_bytes(self, m8: bool, x8: bool) -> u8 {
        match self {
            Mode::Implied | Mode::Accumulator => 0,
            Mode::ImmediateM => {
                if m8 {
                    1
                } else {
                    2
                }
            }
            Mode::ImmediateX => {
                if x8 {
                    1
                } else {
                    2
                }
            }
            Mode::Immediate8
            | Mode::Direct
            | Mode::DirectX
            | Mode::DirectY
            | Mode::DirectIndirect
            | Mode::DirectIndirectLong
            | Mode::DirectIndexedIndirect
            | Mode::DirectIndirectIndexed
            | Mode::DirectIndirectLongIndexed
            | Mode::StackRelative
            | Mode::StackRelativeIndirectIndexed
            | Mode::Relative => 1,
            Mode::Absolute
            | Mode::AbsoluteX
            | Mode::AbsoluteY
            | Mode::AbsoluteIndirect
            | Mode::AbsoluteIndirectLong
            | Mode::AbsoluteIndexedIndirect
            | Mode::RelativeLong
            | Mode::BlockMove => 2,
            Mode::AbsoluteLong | Mode::AbsoluteLongX => 3,
        }
    }

    /// The datasheet's short name for the mode, for `rsemu describe`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Mode::Implied => "i",
            Mode::Accumulator => "A",
            Mode::ImmediateM | Mode::ImmediateX | Mode::Immediate8 => "#",
            Mode::Absolute => "a",
            Mode::AbsoluteX => "a,x",
            Mode::AbsoluteY => "a,y",
            Mode::AbsoluteLong => "al",
            Mode::AbsoluteLongX => "al,x",
            Mode::AbsoluteIndirect => "(a)",
            Mode::AbsoluteIndirectLong => "[a]",
            Mode::AbsoluteIndexedIndirect => "(a,x)",
            Mode::Direct => "d",
            Mode::DirectX => "d,x",
            Mode::DirectY => "d,y",
            Mode::DirectIndirect => "(d)",
            Mode::DirectIndirectLong => "[d]",
            Mode::DirectIndexedIndirect => "(d,x)",
            Mode::DirectIndirectIndexed => "(d),y",
            Mode::DirectIndirectLongIndexed => "[d],y",
            Mode::StackRelative => "d,s",
            Mode::StackRelativeIndirectIndexed => "(d,s),y",
            Mode::Relative => "r",
            Mode::RelativeLong => "rl",
            Mode::BlockMove => "xyc",
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What the instruction does with the memory operand its mode locates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// No memory operand, or one the instruction handles itself (jumps,
    /// stack operations, block moves).
    None,
    /// Read it: loads, the ALU, compares, `BIT`.
    Read,
    /// Write it: stores.
    Write,
    /// Read it, change it, write it back: shifts, `INC`/`DEC`, `TSB`/`TRB`.
    Modify,
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
macro_rules! define_ops {
    ($($name:ident = $summary:literal,)*) => {
        /// One operation, independent of how its operand is addressed.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]).
        // Mnemonics are uppercase by universal 65xx convention.
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

            /// Every operation, in declaration order.
            pub const ALL: &'static [Op] = &[$(Op::$name,)*];
        }
    };
}

define_ops! {
    ADC = "add memory to the accumulator with carry",
    AND = "AND memory with the accumulator",
    ASL = "shift left one bit",
    BCC = "branch if carry clear",
    BCS = "branch if carry set",
    BEQ = "branch if equal (zero set)",
    BIT = "test bits in memory against the accumulator",
    BMI = "branch if minus (negative set)",
    BNE = "branch if not equal (zero clear)",
    BPL = "branch if plus (negative clear)",
    BRA = "branch always",
    BRK = "software break through the BRK vector",
    BRL = "branch always, long",
    BVC = "branch if overflow clear",
    BVS = "branch if overflow set",
    CLC = "clear carry",
    CLD = "clear decimal mode",
    CLI = "clear the interrupt disable",
    CLV = "clear overflow",
    CMP = "compare memory with the accumulator",
    COP = "co-processor enable: software interrupt through the COP vector",
    CPX = "compare memory with X",
    CPY = "compare memory with Y",
    DEC = "decrement",
    DEX = "decrement X",
    DEY = "decrement Y",
    EOR = "exclusive-OR memory with the accumulator",
    INC = "increment",
    INX = "increment X",
    INY = "increment Y",
    JML = "jump long: load PBR and PC",
    JMP = "jump within the program bank",
    JSL = "jump to subroutine long, pushing PBR",
    JSR = "jump to subroutine within the program bank",
    LDA = "load the accumulator",
    LDX = "load X",
    LDY = "load Y",
    LSR = "shift right one bit",
    MVN = "block move, incrementing X and Y",
    MVP = "block move, decrementing X and Y",
    NOP = "no operation",
    ORA = "OR memory with the accumulator",
    PEA = "push a 16-bit immediate",
    PEI = "push the 16-bit word at a direct-page address",
    PER = "push a PC-relative address",
    PHA = "push the accumulator",
    PHB = "push the data bank register",
    PHD = "push the direct register",
    PHK = "push the program bank register",
    PHP = "push the processor status",
    PHX = "push X",
    PHY = "push Y",
    PLA = "pull the accumulator",
    PLB = "pull the data bank register",
    PLD = "pull the direct register",
    PLP = "pull the processor status",
    PLX = "pull X",
    PLY = "pull Y",
    REP = "reset the status bits the immediate names",
    ROL = "rotate left one bit through carry",
    ROR = "rotate right one bit through carry",
    RTI = "return from interrupt",
    RTL = "return from subroutine long",
    RTS = "return from subroutine",
    SBC = "subtract memory from the accumulator with borrow",
    SEC = "set carry",
    SED = "set decimal mode",
    SEI = "set the interrupt disable",
    SEP = "set the status bits the immediate names",
    STA = "store the accumulator",
    STP = "stop the clock until reset",
    STX = "store X",
    STY = "store Y",
    STZ = "store zero",
    TAX = "transfer the accumulator to X",
    TAY = "transfer the accumulator to Y",
    TCD = "transfer the 16-bit accumulator to the direct register",
    TCS = "transfer the 16-bit accumulator to the stack pointer",
    TDC = "transfer the direct register to the 16-bit accumulator",
    TRB = "test and reset memory bits against the accumulator",
    TSB = "test and set memory bits against the accumulator",
    TSC = "transfer the stack pointer to the 16-bit accumulator",
    TSX = "transfer the stack pointer to X",
    TXA = "transfer X to the accumulator",
    TXS = "transfer X to the stack pointer",
    TXY = "transfer X to Y",
    TYA = "transfer Y to the accumulator",
    TYX = "transfer Y to X",
    WAI = "wait for an interrupt",
    WDM = "reserved for future expansion: a two-byte no-op",
    XBA = "exchange the two bytes of the accumulator",
    XCE = "exchange carry and emulation flags",
}

impl Op {
    /// Whether the operand is as wide as the index registers rather than the
    /// accumulator.
    #[must_use]
    pub const fn index_width(self) -> bool {
        matches!(
            self,
            Op::LDX | Op::LDY | Op::STX | Op::STY | Op::CPX | Op::CPY
        )
    }

    /// Whether this is one of the nine 8-bit conditional and unconditional
    /// branches.
    #[must_use]
    pub const fn is_branch(self) -> bool {
        matches!(
            self,
            Op::BCC | Op::BCS | Op::BEQ | Op::BMI | Op::BNE | Op::BPL | Op::BRA | Op::BVC | Op::BVS
        )
    }
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

/// One decoded opcode: what it does, and how it addresses its operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// The operation.
    pub op: Op,
    /// The addressing mode, which with the M and X flags fixes the length.
    pub mode: Mode,
    /// The operand access the mode's address is used for.
    pub access: Access,
}

impl Insn {
    const fn new(op: Op, mode: Mode, access: Access) -> Insn {
        Insn { op, mode, access }
    }

    /// Instruction length in bytes, opcode included, given the M and X flags.
    #[must_use]
    pub const fn bytes(self, m8: bool, x8: bool) -> u8 {
        1 + self.mode.operand_bytes(m8, x8)
    }
}

/// Build the decode table and its coverage bitmap from a list of rows.
macro_rules! isa {
    ($($opcode:literal $op:ident $mode:ident $access:ident;)*) => {
        /// The decode table: one row per opcode, and the only description of
        /// the 65C816 instruction set in the crate.
        pub static TABLE: [Insn; 256] = {
            let mut t = [Insn::new(Op::NOP, Mode::Implied, Access::None); 256];
            $(t[$opcode as usize] = Insn::new(Op::$op, Mode::$mode, Access::$access);)*
            t
        };

        /// Which opcodes [`TABLE`] actually assigns — all 256, which a test
        /// proves rather than trusts.
        pub static LISTED: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

isa! {
    0x00 BRK Immediate8                   None;
    0x01 ORA DirectIndexedIndirect        Read;
    0x02 COP Immediate8                   None;
    0x03 ORA StackRelative                Read;
    0x04 TSB Direct                       Modify;
    0x05 ORA Direct                       Read;
    0x06 ASL Direct                       Modify;
    0x07 ORA DirectIndirectLong           Read;
    0x08 PHP Implied                      None;
    0x09 ORA ImmediateM                   Read;
    0x0a ASL Accumulator                  None;
    0x0b PHD Implied                      None;
    0x0c TSB Absolute                     Modify;
    0x0d ORA Absolute                     Read;
    0x0e ASL Absolute                     Modify;
    0x0f ORA AbsoluteLong                 Read;

    0x10 BPL Relative                     None;
    0x11 ORA DirectIndirectIndexed        Read;
    0x12 ORA DirectIndirect               Read;
    0x13 ORA StackRelativeIndirectIndexed Read;
    0x14 TRB Direct                       Modify;
    0x15 ORA DirectX                      Read;
    0x16 ASL DirectX                      Modify;
    0x17 ORA DirectIndirectLongIndexed    Read;
    0x18 CLC Implied                      None;
    0x19 ORA AbsoluteY                    Read;
    0x1a INC Accumulator                  None;
    0x1b TCS Implied                      None;
    0x1c TRB Absolute                     Modify;
    0x1d ORA AbsoluteX                    Read;
    0x1e ASL AbsoluteX                    Modify;
    0x1f ORA AbsoluteLongX                Read;

    0x20 JSR Absolute                     None;
    0x21 AND DirectIndexedIndirect        Read;
    0x22 JSL AbsoluteLong                 None;
    0x23 AND StackRelative                Read;
    0x24 BIT Direct                       Read;
    0x25 AND Direct                       Read;
    0x26 ROL Direct                       Modify;
    0x27 AND DirectIndirectLong           Read;
    0x28 PLP Implied                      None;
    0x29 AND ImmediateM                   Read;
    0x2a ROL Accumulator                  None;
    0x2b PLD Implied                      None;
    0x2c BIT Absolute                     Read;
    0x2d AND Absolute                     Read;
    0x2e ROL Absolute                     Modify;
    0x2f AND AbsoluteLong                 Read;

    0x30 BMI Relative                     None;
    0x31 AND DirectIndirectIndexed        Read;
    0x32 AND DirectIndirect               Read;
    0x33 AND StackRelativeIndirectIndexed Read;
    0x34 BIT DirectX                      Read;
    0x35 AND DirectX                      Read;
    0x36 ROL DirectX                      Modify;
    0x37 AND DirectIndirectLongIndexed    Read;
    0x38 SEC Implied                      None;
    0x39 AND AbsoluteY                    Read;
    0x3a DEC Accumulator                  None;
    0x3b TSC Implied                      None;
    0x3c BIT AbsoluteX                    Read;
    0x3d AND AbsoluteX                    Read;
    0x3e ROL AbsoluteX                    Modify;
    0x3f AND AbsoluteLongX                Read;

    0x40 RTI Implied                      None;
    0x41 EOR DirectIndexedIndirect        Read;
    0x42 WDM Immediate8                   None;
    0x43 EOR StackRelative                Read;
    0x44 MVP BlockMove                    None;
    0x45 EOR Direct                       Read;
    0x46 LSR Direct                       Modify;
    0x47 EOR DirectIndirectLong           Read;
    0x48 PHA Implied                      None;
    0x49 EOR ImmediateM                   Read;
    0x4a LSR Accumulator                  None;
    0x4b PHK Implied                      None;
    0x4c JMP Absolute                     None;
    0x4d EOR Absolute                     Read;
    0x4e LSR Absolute                     Modify;
    0x4f EOR AbsoluteLong                 Read;

    0x50 BVC Relative                     None;
    0x51 EOR DirectIndirectIndexed        Read;
    0x52 EOR DirectIndirect               Read;
    0x53 EOR StackRelativeIndirectIndexed Read;
    0x54 MVN BlockMove                    None;
    0x55 EOR DirectX                      Read;
    0x56 LSR DirectX                      Modify;
    0x57 EOR DirectIndirectLongIndexed    Read;
    0x58 CLI Implied                      None;
    0x59 EOR AbsoluteY                    Read;
    0x5a PHY Implied                      None;
    0x5b TCD Implied                      None;
    0x5c JML AbsoluteLong                 None;
    0x5d EOR AbsoluteX                    Read;
    0x5e LSR AbsoluteX                    Modify;
    0x5f EOR AbsoluteLongX                Read;

    0x60 RTS Implied                      None;
    0x61 ADC DirectIndexedIndirect        Read;
    0x62 PER RelativeLong                 None;
    0x63 ADC StackRelative                Read;
    0x64 STZ Direct                       Write;
    0x65 ADC Direct                       Read;
    0x66 ROR Direct                       Modify;
    0x67 ADC DirectIndirectLong           Read;
    0x68 PLA Implied                      None;
    0x69 ADC ImmediateM                   Read;
    0x6a ROR Accumulator                  None;
    0x6b RTL Implied                      None;
    0x6c JMP AbsoluteIndirect             None;
    0x6d ADC Absolute                     Read;
    0x6e ROR Absolute                     Modify;
    0x6f ADC AbsoluteLong                 Read;

    0x70 BVS Relative                     None;
    0x71 ADC DirectIndirectIndexed        Read;
    0x72 ADC DirectIndirect               Read;
    0x73 ADC StackRelativeIndirectIndexed Read;
    0x74 STZ DirectX                      Write;
    0x75 ADC DirectX                      Read;
    0x76 ROR DirectX                      Modify;
    0x77 ADC DirectIndirectLongIndexed    Read;
    0x78 SEI Implied                      None;
    0x79 ADC AbsoluteY                    Read;
    0x7a PLY Implied                      None;
    0x7b TDC Implied                      None;
    0x7c JMP AbsoluteIndexedIndirect      None;
    0x7d ADC AbsoluteX                    Read;
    0x7e ROR AbsoluteX                    Modify;
    0x7f ADC AbsoluteLongX                Read;

    0x80 BRA Relative                     None;
    0x81 STA DirectIndexedIndirect        Write;
    0x82 BRL RelativeLong                 None;
    0x83 STA StackRelative                Write;
    0x84 STY Direct                       Write;
    0x85 STA Direct                       Write;
    0x86 STX Direct                       Write;
    0x87 STA DirectIndirectLong           Write;
    0x88 DEY Implied                      None;
    0x89 BIT ImmediateM                   Read;
    0x8a TXA Implied                      None;
    0x8b PHB Implied                      None;
    0x8c STY Absolute                     Write;
    0x8d STA Absolute                     Write;
    0x8e STX Absolute                     Write;
    0x8f STA AbsoluteLong                 Write;

    0x90 BCC Relative                     None;
    0x91 STA DirectIndirectIndexed        Write;
    0x92 STA DirectIndirect               Write;
    0x93 STA StackRelativeIndirectIndexed Write;
    0x94 STY DirectX                      Write;
    0x95 STA DirectX                      Write;
    0x96 STX DirectY                      Write;
    0x97 STA DirectIndirectLongIndexed    Write;
    0x98 TYA Implied                      None;
    0x99 STA AbsoluteY                    Write;
    0x9a TXS Implied                      None;
    0x9b TXY Implied                      None;
    0x9c STZ Absolute                     Write;
    0x9d STA AbsoluteX                    Write;
    0x9e STZ AbsoluteX                    Write;
    0x9f STA AbsoluteLongX                Write;

    0xa0 LDY ImmediateX                   Read;
    0xa1 LDA DirectIndexedIndirect        Read;
    0xa2 LDX ImmediateX                   Read;
    0xa3 LDA StackRelative                Read;
    0xa4 LDY Direct                       Read;
    0xa5 LDA Direct                       Read;
    0xa6 LDX Direct                       Read;
    0xa7 LDA DirectIndirectLong           Read;
    0xa8 TAY Implied                      None;
    0xa9 LDA ImmediateM                   Read;
    0xaa TAX Implied                      None;
    0xab PLB Implied                      None;
    0xac LDY Absolute                     Read;
    0xad LDA Absolute                     Read;
    0xae LDX Absolute                     Read;
    0xaf LDA AbsoluteLong                 Read;

    0xb0 BCS Relative                     None;
    0xb1 LDA DirectIndirectIndexed        Read;
    0xb2 LDA DirectIndirect               Read;
    0xb3 LDA StackRelativeIndirectIndexed Read;
    0xb4 LDY DirectX                      Read;
    0xb5 LDA DirectX                      Read;
    0xb6 LDX DirectY                      Read;
    0xb7 LDA DirectIndirectLongIndexed    Read;
    0xb8 CLV Implied                      None;
    0xb9 LDA AbsoluteY                    Read;
    0xba TSX Implied                      None;
    0xbb TYX Implied                      None;
    0xbc LDY AbsoluteX                    Read;
    0xbd LDA AbsoluteX                    Read;
    0xbe LDX AbsoluteY                    Read;
    0xbf LDA AbsoluteLongX                Read;

    0xc0 CPY ImmediateX                   Read;
    0xc1 CMP DirectIndexedIndirect        Read;
    0xc2 REP Immediate8                   None;
    0xc3 CMP StackRelative                Read;
    0xc4 CPY Direct                       Read;
    0xc5 CMP Direct                       Read;
    0xc6 DEC Direct                       Modify;
    0xc7 CMP DirectIndirectLong           Read;
    0xc8 INY Implied                      None;
    0xc9 CMP ImmediateM                   Read;
    0xca DEX Implied                      None;
    0xcb WAI Implied                      None;
    0xcc CPY Absolute                     Read;
    0xcd CMP Absolute                     Read;
    0xce DEC Absolute                     Modify;
    0xcf CMP AbsoluteLong                 Read;

    0xd0 BNE Relative                     None;
    0xd1 CMP DirectIndirectIndexed        Read;
    0xd2 CMP DirectIndirect               Read;
    0xd3 CMP StackRelativeIndirectIndexed Read;
    0xd4 PEI DirectIndirect               None;
    0xd5 CMP DirectX                      Read;
    0xd6 DEC DirectX                      Modify;
    0xd7 CMP DirectIndirectLongIndexed    Read;
    0xd8 CLD Implied                      None;
    0xd9 CMP AbsoluteY                    Read;
    0xda PHX Implied                      None;
    0xdb STP Implied                      None;
    0xdc JML AbsoluteIndirectLong         None;
    0xdd CMP AbsoluteX                    Read;
    0xde DEC AbsoluteX                    Modify;
    0xdf CMP AbsoluteLongX                Read;

    0xe0 CPX ImmediateX                   Read;
    0xe1 SBC DirectIndexedIndirect        Read;
    0xe2 SEP Immediate8                   None;
    0xe3 SBC StackRelative                Read;
    0xe4 CPX Direct                       Read;
    0xe5 SBC Direct                       Read;
    0xe6 INC Direct                       Modify;
    0xe7 SBC DirectIndirectLong           Read;
    0xe8 INX Implied                      None;
    0xe9 SBC ImmediateM                   Read;
    0xea NOP Implied                      None;
    0xeb XBA Implied                      None;
    0xec CPX Absolute                     Read;
    0xed SBC Absolute                     Read;
    0xee INC Absolute                     Modify;
    0xef SBC AbsoluteLong                 Read;

    0xf0 BEQ Relative                     None;
    0xf1 SBC DirectIndirectIndexed        Read;
    0xf2 SBC DirectIndirect               Read;
    0xf3 SBC StackRelativeIndirectIndexed Read;
    0xf4 PEA Absolute                     None;
    0xf5 SBC DirectX                      Read;
    0xf6 INC DirectX                      Modify;
    0xf7 SBC DirectIndirectLongIndexed    Read;
    0xf8 SED Implied                      None;
    0xf9 SBC AbsoluteY                    Read;
    0xfa PLX Implied                      None;
    0xfb XCE Implied                      None;
    0xfc JSR AbsoluteIndexedIndirect      None;
    0xfd SBC AbsoluteX                    Read;
    0xfe INC AbsoluteX                    Modify;
    0xff SBC AbsoluteLongX                Read;
}

/// Decode one opcode byte. Total: the 65C816 assigns all 256 encodings.
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
        let missing: Vec<usize> = (0..256).filter(|i| !LISTED[*i]).collect();
        assert!(missing.is_empty(), "opcodes not described: {missing:02x?}");
    }

    #[test]
    fn every_declared_operation_is_reachable() {
        for op in Op::ALL {
            assert!(
                TABLE.iter().any(|i| i.op == *op),
                "{op:?} is declared but no opcode decodes to it"
            );
        }
    }

    #[test]
    fn there_is_exactly_one_nop_and_the_table_has_92_operations() {
        // The initialiser fills with NOP; only $EA may really be one.
        assert_eq!(TABLE.iter().filter(|i| i.op == Op::NOP).count(), 1);
        // The datasheet's instruction count: 92 mnemonics. JML and JSL are
        // written as separate mnemonics here, as Eyes & Lichty do.
        assert_eq!(Op::ALL.len(), 92);
    }

    #[test]
    fn immediate_length_follows_the_flags() {
        let lda = decode(0xa9);
        assert_eq!(lda.bytes(true, true), 2);
        assert_eq!(lda.bytes(false, true), 3);
        let ldx = decode(0xa2);
        assert_eq!(ldx.bytes(false, true), 2);
        assert_eq!(ldx.bytes(true, false), 3);
        // REP is one byte of immediate whatever M and X say.
        assert_eq!(decode(0xc2).bytes(false, false), 2);
        assert_eq!(decode(0x5c).bytes(true, true), 4);
        assert_eq!(decode(0x54).bytes(true, true), 3);
    }

    #[test]
    fn index_width_operations_are_the_six_index_ones() {
        let n = Op::ALL.iter().filter(|o| o.index_width()).count();
        assert_eq!(n, 6);
    }
}
