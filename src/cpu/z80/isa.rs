//! The instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds the whole description of the Z80's five opcode pages, and
//! everything else is derived from it:
//!
//! - the interpreter's decode ([`decode`], [`decode_cb`], [`decode_ed`]) and
//!   its per-instruction bus sequence, which comes from the operand shapes
//!   rather than from a cycle-count column;
//! - the disassembler ([`super::disasm`]), which formats from the same row;
//! - the index-prefix pages, which are **derived** rather than tabulated —
//!   see [`index_substitute`].
//!
//! # Three tables, five pages
//!
//! The Z80 has five opcode pages but only three of them are independent:
//!
//! | Page | Table | How it is reached |
//! | --- | --- | --- |
//! | base | [`BASE`] | no prefix |
//! | `CB` | [`CB`] | the `$cb` prefix |
//! | `ED` | [`ED`] | the `$ed` prefix |
//! | `DD` / `FD` | derived | [`index_substitute`] over [`BASE`] |
//! | `DDCB` / `FDCB` | derived | [`decode_ddcb`] over [`CB`] |
//!
//! Deriving the index pages is not a shortcut, it is what the hardware does:
//! a `$dd` prefix retargets the HL path of the *same* decoder at IX, which is
//! why `LD H,(IX+d)` keeps a real `H` while `LD H,L` becomes `LD IXH,IXL`.
//! Tabulating 1 024 more rows would let the two descriptions disagree.
//!
//! # Why there is no cycle column
//!
//! Deliberate, and the same reason as the 6502's. A Z80 instruction's T-state
//! count is the sum of its M-cycles, and which M-cycles it performs depends on
//! the operand shape and, for conditionals and the block instructions, on the
//! data. `JR cc,e` is 7 or 12 T-states; `LDIR` is 16 or 21. The interpreter
//! charges T-states because it performed an M-cycle, so the count cannot drift
//! from the bus trace (`ROADMAP.md` §6).
//!
//! # Sources
//!
//! The opcode matrix, operand encodings and undocumented pages come from Zilog
//! **UM0080** (the Z80 CPU User Manual), the World of Spectrum Z80 reference,
//! and Sean Young's *Undocumented Z80 Documented* v0.91 for `SLL`, the `IX`/
//! `IY` halves, the duplicate `ED` encodings and the `DDCB` register copies
//! (`docs/cpu/z80-sm83.md`). No copyleft emulator was consulted.

use core::fmt;

/// An 8-bit register operand.
///
/// [`R8::Ixh`] and the other three halves never appear in [`BASE`]: they are
/// produced only by [`index_substitute`], because on hardware they are `H` and
/// `L` seen through an index prefix rather than registers of their own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum R8 {
    /// Accumulator.
    A,
    /// General purpose `B`, and the counter `DJNZ` and the block I/O use.
    B,
    /// General purpose `C`, and the low half of the I/O port address.
    C,
    /// General purpose `D`.
    D,
    /// General purpose `E`.
    E,
    /// High half of `HL`.
    H,
    /// Low half of `HL`.
    L,
    /// The interrupt vector base register.
    I,
    /// The memory refresh counter.
    R,
    /// High half of `IX` — undocumented.
    Ixh,
    /// Low half of `IX` — undocumented.
    Ixl,
    /// High half of `IY` — undocumented.
    Iyh,
    /// Low half of `IY` — undocumented.
    Iyl,
}

impl R8 {
    /// The register's name in assembler syntax.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            R8::A => "A",
            R8::B => "B",
            R8::C => "C",
            R8::D => "D",
            R8::E => "E",
            R8::H => "H",
            R8::L => "L",
            R8::I => "I",
            R8::R => "R",
            R8::Ixh => "IXH",
            R8::Ixl => "IXL",
            R8::Iyh => "IYH",
            R8::Iyl => "IYL",
        }
    }

    /// The eight registers the `r` field of an opcode selects, in encoding
    /// order. Index 6 is `(HL)`, which is memory rather than a register, so
    /// the slot holds `None`.
    pub const ENCODED: [Option<R8>; 8] = [
        Some(R8::B),
        Some(R8::C),
        Some(R8::D),
        Some(R8::E),
        Some(R8::H),
        Some(R8::L),
        None,
        Some(R8::A),
    ];
}

impl fmt::Display for R8 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A 16-bit register or register pair operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum R16 {
    /// The accumulator and flags.
    Af,
    /// `B` and `C`.
    Bc,
    /// `D` and `E`.
    De,
    /// `H` and `L`.
    Hl,
    /// The stack pointer.
    Sp,
    /// Index register `IX`.
    Ix,
    /// Index register `IY`.
    Iy,
    /// The shadow accumulator and flags, which only `EX AF,AF'` names.
    AfAlt,
}

impl R16 {
    /// The pair's name in assembler syntax.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            R16::Af => "AF",
            R16::Bc => "BC",
            R16::De => "DE",
            R16::Hl => "HL",
            R16::Sp => "SP",
            R16::Ix => "IX",
            R16::Iy => "IY",
            R16::AfAlt => "AF'",
        }
    }
}

impl fmt::Display for R16 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which index register a `$dd` or `$fd` prefix selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Index {
    /// The `$dd` prefix.
    Ix,
    /// The `$fd` prefix.
    Iy,
}

impl Index {
    /// The prefix byte that selects this index register.
    #[must_use]
    pub const fn prefix(self) -> u8 {
        match self {
            Index::Ix => 0xdd,
            Index::Iy => 0xfd,
        }
    }

    /// The index register as a 16-bit operand.
    #[must_use]
    pub const fn reg16(self) -> R16 {
        match self {
            Index::Ix => R16::Ix,
            Index::Iy => R16::Iy,
        }
    }

    /// The register that stands in for `H` under this prefix.
    #[must_use]
    pub const fn high(self) -> R8 {
        match self {
            Index::Ix => R8::Ixh,
            Index::Iy => R8::Iyh,
        }
    }

    /// The register that stands in for `L` under this prefix.
    #[must_use]
    pub const fn low(self) -> R8 {
        match self {
            Index::Ix => R8::Ixl,
            Index::Iy => R8::Iyl,
        }
    }
}

/// Where an instruction finds one of its operands.
///
/// The set of operands fixes the instruction's length, which is why there is
/// no length column: [`Insn::operand_bytes`] is the one answer both the
/// disassembler and the program counter use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operand {
    /// The instruction has no operand in this position.
    None,
    /// An 8-bit register.
    Reg(R8),
    /// A 16-bit register or pair.
    Reg16(R16),
    /// Memory addressed by a register pair: `(HL)`, `(BC)`, `(DE)`, `(SP)`.
    Ind(R16),
    /// Memory addressed by an index register plus a signed displacement:
    /// `(IX+d)`. Produced only by [`index_substitute`] and [`decode_ddcb`].
    Idx(R16),
    /// A register used *as* an address without a memory access: the `HL` of
    /// `JP (HL)`, which the assembler spells with parentheses it does not
    /// deserve.
    Ptr(R16),
    /// An immediate byte.
    Imm8,
    /// An immediate word, low byte first.
    Imm16,
    /// Memory at an immediate 16-bit address. Whether one byte or two are
    /// moved is decided by the other operand.
    Abs,
    /// A signed 8-bit displacement from the address of the *next* instruction:
    /// the `e` of `JR` and `DJNZ`.
    Rel,
    /// A bit index, 0 to 7.
    Bit(u8),
    /// A restart target address.
    Rst(u8),
    /// An interrupt mode number, 0 to 2.
    Mode(u8),
    /// The I/O port addressed by `BC` — the whole pair, not just `C`.
    PortC,
    /// The I/O port addressed by an immediate byte, with `A` on the high half
    /// of the address bus.
    PortImm,
    /// The literal zero the undocumented `OUT (C),0` writes.
    Zero,
}

impl Operand {
    /// How many bytes of the instruction stream this operand consumes.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        match self {
            Operand::Imm8 | Operand::Rel | Operand::PortImm | Operand::Idx(_) => 1,
            Operand::Imm16 | Operand::Abs => 2,
            _ => 0,
        }
    }

    /// Whether this operand is a memory access through `HL`, which is the
    /// operand a `$dd`/`$fd` prefix redirects at `(IX+d)`.
    #[must_use]
    pub const fn is_hl_indirect(self) -> bool {
        matches!(self, Operand::Ind(R16::Hl))
    }
}

/// The condition an instruction tests before it does anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Cond {
    /// Unconditional.
    Always,
    /// Zero clear.
    Nz,
    /// Zero set.
    Z,
    /// Carry clear.
    Nc,
    /// Carry set.
    C,
    /// Parity odd — parity/overflow clear.
    Po,
    /// Parity even — parity/overflow set.
    Pe,
    /// Positive — sign clear.
    P,
    /// Minus — sign set.
    M,
}

impl Cond {
    /// The condition's name in assembler syntax, or `""` for
    /// [`Cond::Always`].
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Cond::Always => "",
            Cond::Nz => "NZ",
            Cond::Z => "Z",
            Cond::Nc => "NC",
            Cond::C => "C",
            Cond::Po => "PO",
            Cond::Pe => "PE",
            Cond::P => "P",
            Cond::M => "M",
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How well documented an encoding is.
///
/// Undocumented encodings are in scope from the start because real software
/// depends on them — the `IX` halves, `SLL` and the `DDCB` register copies all
/// appear in shipped Spectrum and MSX code — so the distinction is kept only
/// so a disassembler can flag them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Class {
    /// In Zilog UM0080.
    Documented,
    /// Not in the manual, but deterministic and relied upon.
    Undocumented,
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
        /// One operation, independent of how its operands are addressed.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]), so a
        /// disassembler cannot print a name the interpreter does not
        /// implement.
        // Mnemonics are uppercase by universal Z80 convention; renaming them
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
    ADC = "add with carry",
    ADD = "add",
    AND = "logical AND with the accumulator",
    BIT = "test one bit and set Z from its complement",
    CALL = "push the return address and jump",
    CCF = "complement the carry flag",
    CP = "compare with the accumulator, discarding the result",
    CPD = "compare and decrement HL",
    CPDR = "compare and decrement HL until BC is zero or a match is found",
    CPI = "compare and increment HL",
    CPIR = "compare and increment HL until BC is zero or a match is found",
    CPL = "complement the accumulator",
    DAA = "decimal-adjust the accumulator after a packed-BCD add or subtract",
    DEC = "decrement",
    DI = "disable maskable interrupts",
    DJNZ = "decrement B and jump relative if it is not zero",
    EI = "enable maskable interrupts, effective after the next instruction",
    EX = "exchange two registers",
    EXX = "exchange BC, DE and HL with their shadows",
    HALT = "stop fetching until an interrupt arrives",
    IM = "select the interrupt mode",
    IN = "read an I/O port",
    INC = "increment",
    IND = "read a port into (HL) and decrement HL",
    INDR = "read ports into (HL) downwards until B is zero",
    INI = "read a port into (HL) and increment HL",
    INIR = "read ports into (HL) upwards until B is zero",
    JP = "jump",
    JR = "jump relative",
    LD = "load",
    LDD = "copy (HL) to (DE) and decrement both",
    LDDR = "copy (HL) to (DE) downwards until BC is zero",
    LDI = "copy (HL) to (DE) and increment both",
    LDIR = "copy (HL) to (DE) upwards until BC is zero",
    NEG = "negate the accumulator",
    NOP = "no operation",
    OR = "logical OR with the accumulator",
    OTDR = "write (HL) to a port downwards until B is zero",
    OTIR = "write (HL) to a port upwards until B is zero",
    OUT = "write an I/O port",
    OUTD = "write (HL) to a port and decrement HL",
    OUTI = "write (HL) to a port and increment HL",
    POP = "pop a register pair from the stack",
    PREFIX = "extend the opcode into another page; never executed on its own",
    PUSH = "push a register pair onto the stack",
    RES = "reset one bit",
    RET = "return from a subroutine",
    RETI = "return from a maskable interrupt",
    RETN = "return from a non-maskable interrupt, restoring IFF1 from IFF2",
    RL = "rotate left through carry",
    RLA = "rotate the accumulator left through carry, flags mostly untouched",
    RLC = "rotate left circular",
    RLCA = "rotate the accumulator left circular, flags mostly untouched",
    RLD = "rotate one BCD digit left between A and (HL)",
    RR = "rotate right through carry",
    RRA = "rotate the accumulator right through carry, flags mostly untouched",
    RRC = "rotate right circular",
    RRCA = "rotate the accumulator right circular, flags mostly untouched",
    RRD = "rotate one BCD digit right between A and (HL)",
    RST = "call one of the eight page-zero restart addresses",
    SBC = "subtract with borrow",
    SCF = "set the carry flag",
    SET = "set one bit",
    SLA = "shift left arithmetic",
    SLL = "undocumented: shift left, feeding a one into bit 0",
    SRA = "shift right arithmetic, preserving the sign",
    SRL = "shift right logical",
    SUB = "subtract from the accumulator",
    XOR = "logical exclusive-OR with the accumulator",
}

/// One row of the instruction description: everything known about an encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The destination operand, which for most rows is also the first one
    /// printed.
    pub dst: Operand,
    /// The source operand.
    pub src: Operand,
    /// The condition it tests, if any.
    pub cond: Cond,
    /// The register a `DDCB`/`FDCB` encoding *also* writes its result to.
    ///
    /// Undocumented, and set only by [`decode_ddcb`]: `DD CB d 00` is
    /// `RLC (IX+d)` as far as the manual is concerned, but the low three bits
    /// of the opcode still select a register, and the shifted value lands
    /// there too (Sean Young, *Undocumented Z80 Documented* §6.3).
    pub also: Option<R8>,
    /// Whether this encoding is documented.
    pub class: Class,
}

impl Insn {
    const fn new(op: Op, dst: Operand, src: Operand, cond: Cond, class: Class) -> Insn {
        Insn {
            op,
            dst,
            src,
            cond,
            also: None,
            class,
        }
    }

    /// How many operand bytes follow the opcode.
    ///
    /// Does **not** include prefix bytes, which the caller counted on the way
    /// in, nor the `DDCB` displacement, which sits between the prefix and the
    /// opcode rather than after it.
    #[must_use]
    pub const fn operand_bytes(self) -> u16 {
        self.dst.bytes() + self.src.bytes()
    }

    /// Whether this row is one of the four prefix bytes rather than an
    /// instruction.
    #[must_use]
    pub const fn is_prefix(self) -> bool {
        matches!(self.op, Op::PREFIX)
    }
}

/// Expand one operand token into an [`Operand`].
///
/// The token language is assembler syntax with two additions: `-` is "no
/// operand in this position", and `[HL]` is a register used as an address
/// *without* a memory access, which is what `JP (HL)` really does.
macro_rules! opnd {
    (-) => {
        Operand::None
    };
    (A) => {
        Operand::Reg(R8::A)
    };
    (B) => {
        Operand::Reg(R8::B)
    };
    (C) => {
        Operand::Reg(R8::C)
    };
    (D) => {
        Operand::Reg(R8::D)
    };
    (E) => {
        Operand::Reg(R8::E)
    };
    (H) => {
        Operand::Reg(R8::H)
    };
    (L) => {
        Operand::Reg(R8::L)
    };
    (I) => {
        Operand::Reg(R8::I)
    };
    (R) => {
        Operand::Reg(R8::R)
    };
    (AF) => {
        Operand::Reg16(R16::Af)
    };
    (AF_) => {
        Operand::Reg16(R16::AfAlt)
    };
    (BC) => {
        Operand::Reg16(R16::Bc)
    };
    (DE) => {
        Operand::Reg16(R16::De)
    };
    (HL) => {
        Operand::Reg16(R16::Hl)
    };
    (SP) => {
        Operand::Reg16(R16::Sp)
    };
    ((BC)) => {
        Operand::Ind(R16::Bc)
    };
    ((DE)) => {
        Operand::Ind(R16::De)
    };
    ((HL)) => {
        Operand::Ind(R16::Hl)
    };
    ((SP)) => {
        Operand::Ind(R16::Sp)
    };
    ([HL]) => {
        Operand::Ptr(R16::Hl)
    };
    (n) => {
        Operand::Imm8
    };
    (nn) => {
        Operand::Imm16
    };
    ((nn)) => {
        Operand::Abs
    };
    (e) => {
        Operand::Rel
    };
    ((C)) => {
        Operand::PortC
    };
    ((n)) => {
        Operand::PortImm
    };
    (zero) => {
        Operand::Zero
    };
    (b0) => {
        Operand::Bit(0)
    };
    (b1) => {
        Operand::Bit(1)
    };
    (b2) => {
        Operand::Bit(2)
    };
    (b3) => {
        Operand::Bit(3)
    };
    (b4) => {
        Operand::Bit(4)
    };
    (b5) => {
        Operand::Bit(5)
    };
    (b6) => {
        Operand::Bit(6)
    };
    (b7) => {
        Operand::Bit(7)
    };
    (rst00) => {
        Operand::Rst(0x00)
    };
    (rst08) => {
        Operand::Rst(0x08)
    };
    (rst10) => {
        Operand::Rst(0x10)
    };
    (rst18) => {
        Operand::Rst(0x18)
    };
    (rst20) => {
        Operand::Rst(0x20)
    };
    (rst28) => {
        Operand::Rst(0x28)
    };
    (rst30) => {
        Operand::Rst(0x30)
    };
    (rst38) => {
        Operand::Rst(0x38)
    };
    (im0) => {
        Operand::Mode(0)
    };
    (im1) => {
        Operand::Mode(1)
    };
    (im2) => {
        Operand::Mode(2)
    };
}

/// Expand a condition token.
macro_rules! cond {
    (_) => {
        Cond::Always
    };
    (NZ) => {
        Cond::Nz
    };
    (Z) => {
        Cond::Z
    };
    (NC) => {
        Cond::Nc
    };
    (C) => {
        Cond::C
    };
    (PO) => {
        Cond::Po
    };
    (PE) => {
        Cond::Pe
    };
    (P) => {
        Cond::P
    };
    (M) => {
        Cond::M
    };
}

/// Expand a class token: `d` documented, `u` undocumented.
macro_rules! class {
    (d) => {
        Class::Documented
    };
    (u) => {
        Class::Undocumented
    };
}

/// Build one 256-entry page and the `LISTED` mask that proves it complete.
///
/// Rows the table does not mention decode as an undocumented `NOP`, which is
/// exactly what the `ED` page's holes do on hardware — but the mask records
/// which rows were written, so a test can distinguish "deliberately a NOP"
/// from "forgotten".
macro_rules! isa {
    ($(#[$doc:meta])* $table:ident $listed:ident : $($opcode:literal $op:ident $dst:tt $src:tt $cond:tt $class:ident;)*) => {
        $(#[$doc])*
        pub static $table: [Insn; 256] = {
            let mut t = [Insn::new(
                Op::NOP, Operand::None, Operand::None, Cond::Always, Class::Undocumented,
            ); 256];
            $(t[$opcode as usize] = Insn::new(
                Op::$op, opnd!($dst), opnd!($src), cond!($cond), class!($class),
            );)*
            t
        };

        /// Which opcodes of the page above the table actually assigns.
        ///
        /// Test scaffolding with a real job: an unlisted row decodes as an
        /// undocumented `NOP`, and only this mask says whether that was the
        /// intent.
        pub static $listed: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

isa! {
    /// The unprefixed opcode page: the only description of it in the crate.
    BASE BASE_LISTED:
    0x00 NOP    -    -     _  d;
    0x01 LD     BC   nn    _  d;
    0x02 LD     (BC) A     _  d;
    0x03 INC    BC   -     _  d;
    0x04 INC    B    -     _  d;
    0x05 DEC    B    -     _  d;
    0x06 LD     B    n     _  d;
    0x07 RLCA   -    -     _  d;
    0x08 EX     AF   AF_   _  d;
    0x09 ADD    HL   BC    _  d;
    0x0a LD     A    (BC)  _  d;
    0x0b DEC    BC   -     _  d;
    0x0c INC    C    -     _  d;
    0x0d DEC    C    -     _  d;
    0x0e LD     C    n     _  d;
    0x0f RRCA   -    -     _  d;

    0x10 DJNZ   -    e     _  d;
    0x11 LD     DE   nn    _  d;
    0x12 LD     (DE) A     _  d;
    0x13 INC    DE   -     _  d;
    0x14 INC    D    -     _  d;
    0x15 DEC    D    -     _  d;
    0x16 LD     D    n     _  d;
    0x17 RLA    -    -     _  d;
    0x18 JR     -    e     _  d;
    0x19 ADD    HL   DE    _  d;
    0x1a LD     A    (DE)  _  d;
    0x1b DEC    DE   -     _  d;
    0x1c INC    E    -     _  d;
    0x1d DEC    E    -     _  d;
    0x1e LD     E    n     _  d;
    0x1f RRA    -    -     _  d;

    0x20 JR     -    e     NZ d;
    0x21 LD     HL   nn    _  d;
    0x22 LD     (nn) HL    _  d;
    0x23 INC    HL   -     _  d;
    0x24 INC    H    -     _  d;
    0x25 DEC    H    -     _  d;
    0x26 LD     H    n     _  d;
    0x27 DAA    -    -     _  d;
    0x28 JR     -    e     Z  d;
    0x29 ADD    HL   HL    _  d;
    0x2a LD     HL   (nn)  _  d;
    0x2b DEC    HL   -     _  d;
    0x2c INC    L    -     _  d;
    0x2d DEC    L    -     _  d;
    0x2e LD     L    n     _  d;
    0x2f CPL    -    -     _  d;

    0x30 JR     -    e     NC d;
    0x31 LD     SP   nn    _  d;
    0x32 LD     (nn) A     _  d;
    0x33 INC    SP   -     _  d;
    0x34 INC    (HL) -     _  d;
    0x35 DEC    (HL) -     _  d;
    0x36 LD     (HL) n     _  d;
    0x37 SCF    -    -     _  d;
    0x38 JR     -    e     C  d;
    0x39 ADD    HL   SP    _  d;
    0x3a LD     A    (nn)  _  d;
    0x3b DEC    SP   -     _  d;
    0x3c INC    A    -     _  d;
    0x3d DEC    A    -     _  d;
    0x3e LD     A    n     _  d;
    0x3f CCF    -    -     _  d;

    0x40 LD     B    B     _  d;
    0x41 LD     B    C     _  d;
    0x42 LD     B    D     _  d;
    0x43 LD     B    E     _  d;
    0x44 LD     B    H     _  d;
    0x45 LD     B    L     _  d;
    0x46 LD     B    (HL)  _  d;
    0x47 LD     B    A     _  d;
    0x48 LD     C    B     _  d;
    0x49 LD     C    C     _  d;
    0x4a LD     C    D     _  d;
    0x4b LD     C    E     _  d;
    0x4c LD     C    H     _  d;
    0x4d LD     C    L     _  d;
    0x4e LD     C    (HL)  _  d;
    0x4f LD     C    A     _  d;

    0x50 LD     D    B     _  d;
    0x51 LD     D    C     _  d;
    0x52 LD     D    D     _  d;
    0x53 LD     D    E     _  d;
    0x54 LD     D    H     _  d;
    0x55 LD     D    L     _  d;
    0x56 LD     D    (HL)  _  d;
    0x57 LD     D    A     _  d;
    0x58 LD     E    B     _  d;
    0x59 LD     E    C     _  d;
    0x5a LD     E    D     _  d;
    0x5b LD     E    E     _  d;
    0x5c LD     E    H     _  d;
    0x5d LD     E    L     _  d;
    0x5e LD     E    (HL)  _  d;
    0x5f LD     E    A     _  d;

    0x60 LD     H    B     _  d;
    0x61 LD     H    C     _  d;
    0x62 LD     H    D     _  d;
    0x63 LD     H    E     _  d;
    0x64 LD     H    H     _  d;
    0x65 LD     H    L     _  d;
    0x66 LD     H    (HL)  _  d;
    0x67 LD     H    A     _  d;
    0x68 LD     L    B     _  d;
    0x69 LD     L    C     _  d;
    0x6a LD     L    D     _  d;
    0x6b LD     L    E     _  d;
    0x6c LD     L    H     _  d;
    0x6d LD     L    L     _  d;
    0x6e LD     L    (HL)  _  d;
    0x6f LD     L    A     _  d;

    0x70 LD     (HL) B     _  d;
    0x71 LD     (HL) C     _  d;
    0x72 LD     (HL) D     _  d;
    0x73 LD     (HL) E     _  d;
    0x74 LD     (HL) H     _  d;
    0x75 LD     (HL) L     _  d;
    0x76 HALT   -    -     _  d;
    0x77 LD     (HL) A     _  d;
    0x78 LD     A    B     _  d;
    0x79 LD     A    C     _  d;
    0x7a LD     A    D     _  d;
    0x7b LD     A    E     _  d;
    0x7c LD     A    H     _  d;
    0x7d LD     A    L     _  d;
    0x7e LD     A    (HL)  _  d;
    0x7f LD     A    A     _  d;

    0x80 ADD    A    B     _  d;
    0x81 ADD    A    C     _  d;
    0x82 ADD    A    D     _  d;
    0x83 ADD    A    E     _  d;
    0x84 ADD    A    H     _  d;
    0x85 ADD    A    L     _  d;
    0x86 ADD    A    (HL)  _  d;
    0x87 ADD    A    A     _  d;
    0x88 ADC    A    B     _  d;
    0x89 ADC    A    C     _  d;
    0x8a ADC    A    D     _  d;
    0x8b ADC    A    E     _  d;
    0x8c ADC    A    H     _  d;
    0x8d ADC    A    L     _  d;
    0x8e ADC    A    (HL)  _  d;
    0x8f ADC    A    A     _  d;

    0x90 SUB    -    B     _  d;
    0x91 SUB    -    C     _  d;
    0x92 SUB    -    D     _  d;
    0x93 SUB    -    E     _  d;
    0x94 SUB    -    H     _  d;
    0x95 SUB    -    L     _  d;
    0x96 SUB    -    (HL)  _  d;
    0x97 SUB    -    A     _  d;
    0x98 SBC    A    B     _  d;
    0x99 SBC    A    C     _  d;
    0x9a SBC    A    D     _  d;
    0x9b SBC    A    E     _  d;
    0x9c SBC    A    H     _  d;
    0x9d SBC    A    L     _  d;
    0x9e SBC    A    (HL)  _  d;
    0x9f SBC    A    A     _  d;

    0xa0 AND    -    B     _  d;
    0xa1 AND    -    C     _  d;
    0xa2 AND    -    D     _  d;
    0xa3 AND    -    E     _  d;
    0xa4 AND    -    H     _  d;
    0xa5 AND    -    L     _  d;
    0xa6 AND    -    (HL)  _  d;
    0xa7 AND    -    A     _  d;
    0xa8 XOR    -    B     _  d;
    0xa9 XOR    -    C     _  d;
    0xaa XOR    -    D     _  d;
    0xab XOR    -    E     _  d;
    0xac XOR    -    H     _  d;
    0xad XOR    -    L     _  d;
    0xae XOR    -    (HL)  _  d;
    0xaf XOR    -    A     _  d;

    0xb0 OR     -    B     _  d;
    0xb1 OR     -    C     _  d;
    0xb2 OR     -    D     _  d;
    0xb3 OR     -    E     _  d;
    0xb4 OR     -    H     _  d;
    0xb5 OR     -    L     _  d;
    0xb6 OR     -    (HL)  _  d;
    0xb7 OR     -    A     _  d;
    0xb8 CP     -    B     _  d;
    0xb9 CP     -    C     _  d;
    0xba CP     -    D     _  d;
    0xbb CP     -    E     _  d;
    0xbc CP     -    H     _  d;
    0xbd CP     -    L     _  d;
    0xbe CP     -    (HL)  _  d;
    0xbf CP     -    A     _  d;

    0xc0 RET    -    -     NZ d;
    0xc1 POP    BC   -     _  d;
    0xc2 JP     -    nn    NZ d;
    0xc3 JP     -    nn    _  d;
    0xc4 CALL   -    nn    NZ d;
    0xc5 PUSH   BC   -     _  d;
    0xc6 ADD    A    n     _  d;
    0xc7 RST    -    rst00 _  d;
    0xc8 RET    -    -     Z  d;
    0xc9 RET    -    -     _  d;
    0xca JP     -    nn    Z  d;
    0xcb PREFIX -    -     _  d;
    0xcc CALL   -    nn    Z  d;
    0xcd CALL   -    nn    _  d;
    0xce ADC    A    n     _  d;
    0xcf RST    -    rst08 _  d;

    0xd0 RET    -    -     NC d;
    0xd1 POP    DE   -     _  d;
    0xd2 JP     -    nn    NC d;
    0xd3 OUT    (n)  A     _  d;
    0xd4 CALL   -    nn    NC d;
    0xd5 PUSH   DE   -     _  d;
    0xd6 SUB    -    n     _  d;
    0xd7 RST    -    rst10 _  d;
    0xd8 RET    -    -     C  d;
    0xd9 EXX    -    -     _  d;
    0xda JP     -    nn    C  d;
    0xdb IN     A    (n)   _  d;
    0xdc CALL   -    nn    C  d;
    0xdd PREFIX -    -     _  d;
    0xde SBC    A    n     _  d;
    0xdf RST    -    rst18 _  d;

    0xe0 RET    -    -     PO d;
    0xe1 POP    HL   -     _  d;
    0xe2 JP     -    nn    PO d;
    0xe3 EX     (SP) HL    _  d;
    0xe4 CALL   -    nn    PO d;
    0xe5 PUSH   HL   -     _  d;
    0xe6 AND    -    n     _  d;
    0xe7 RST    -    rst20 _  d;
    0xe8 RET    -    -     PE d;
    0xe9 JP     -    [HL]  _  d;
    0xea JP     -    nn    PE d;
    0xeb EX     DE   HL    _  d;
    0xec CALL   -    nn    PE d;
    0xed PREFIX -    -     _  d;
    0xee XOR    -    n     _  d;
    0xef RST    -    rst28 _  d;

    0xf0 RET    -    -     P  d;
    0xf1 POP    AF   -     _  d;
    0xf2 JP     -    nn    P  d;
    0xf3 DI     -    -     _  d;
    0xf4 CALL   -    nn    P  d;
    0xf5 PUSH   AF   -     _  d;
    0xf6 OR     -    n     _  d;
    0xf7 RST    -    rst30 _  d;
    0xf8 RET    -    -     M  d;
    0xf9 LD     SP   HL    _  d;
    0xfa JP     -    nn    M  d;
    0xfb EI     -    -     _  d;
    0xfc CALL   -    nn    M  d;
    0xfd PREFIX -    -     _  d;
    0xfe CP     -    n     _  d;
    0xff RST    -    rst38 _  d;
}

isa! {
    /// The `$cb` page: rotates, shifts, and the bit operations.
    CB CB_LISTED:
    0x00 RLC B    -    _ d;
    0x01 RLC C    -    _ d;
    0x02 RLC D    -    _ d;
    0x03 RLC E    -    _ d;
    0x04 RLC H    -    _ d;
    0x05 RLC L    -    _ d;
    0x06 RLC (HL) -    _ d;
    0x07 RLC A    -    _ d;
    0x08 RRC B    -    _ d;
    0x09 RRC C    -    _ d;
    0x0a RRC D    -    _ d;
    0x0b RRC E    -    _ d;
    0x0c RRC H    -    _ d;
    0x0d RRC L    -    _ d;
    0x0e RRC (HL) -    _ d;
    0x0f RRC A    -    _ d;

    0x10 RL  B    -    _ d;
    0x11 RL  C    -    _ d;
    0x12 RL  D    -    _ d;
    0x13 RL  E    -    _ d;
    0x14 RL  H    -    _ d;
    0x15 RL  L    -    _ d;
    0x16 RL  (HL) -    _ d;
    0x17 RL  A    -    _ d;
    0x18 RR  B    -    _ d;
    0x19 RR  C    -    _ d;
    0x1a RR  D    -    _ d;
    0x1b RR  E    -    _ d;
    0x1c RR  H    -    _ d;
    0x1d RR  L    -    _ d;
    0x1e RR  (HL) -    _ d;
    0x1f RR  A    -    _ d;

    0x20 SLA B    -    _ d;
    0x21 SLA C    -    _ d;
    0x22 SLA D    -    _ d;
    0x23 SLA E    -    _ d;
    0x24 SLA H    -    _ d;
    0x25 SLA L    -    _ d;
    0x26 SLA (HL) -    _ d;
    0x27 SLA A    -    _ d;
    0x28 SRA B    -    _ d;
    0x29 SRA C    -    _ d;
    0x2a SRA D    -    _ d;
    0x2b SRA E    -    _ d;
    0x2c SRA H    -    _ d;
    0x2d SRA L    -    _ d;
    0x2e SRA (HL) -    _ d;
    0x2f SRA A    -    _ d;

    0x30 SLL B    -    _ u;
    0x31 SLL C    -    _ u;
    0x32 SLL D    -    _ u;
    0x33 SLL E    -    _ u;
    0x34 SLL H    -    _ u;
    0x35 SLL L    -    _ u;
    0x36 SLL (HL) -    _ u;
    0x37 SLL A    -    _ u;
    0x38 SRL B    -    _ d;
    0x39 SRL C    -    _ d;
    0x3a SRL D    -    _ d;
    0x3b SRL E    -    _ d;
    0x3c SRL H    -    _ d;
    0x3d SRL L    -    _ d;
    0x3e SRL (HL) -    _ d;
    0x3f SRL A    -    _ d;

    0x40 BIT b0   B    _ d;
    0x41 BIT b0   C    _ d;
    0x42 BIT b0   D    _ d;
    0x43 BIT b0   E    _ d;
    0x44 BIT b0   H    _ d;
    0x45 BIT b0   L    _ d;
    0x46 BIT b0   (HL) _ d;
    0x47 BIT b0   A    _ d;
    0x48 BIT b1   B    _ d;
    0x49 BIT b1   C    _ d;
    0x4a BIT b1   D    _ d;
    0x4b BIT b1   E    _ d;
    0x4c BIT b1   H    _ d;
    0x4d BIT b1   L    _ d;
    0x4e BIT b1   (HL) _ d;
    0x4f BIT b1   A    _ d;

    0x50 BIT b2   B    _ d;
    0x51 BIT b2   C    _ d;
    0x52 BIT b2   D    _ d;
    0x53 BIT b2   E    _ d;
    0x54 BIT b2   H    _ d;
    0x55 BIT b2   L    _ d;
    0x56 BIT b2   (HL) _ d;
    0x57 BIT b2   A    _ d;
    0x58 BIT b3   B    _ d;
    0x59 BIT b3   C    _ d;
    0x5a BIT b3   D    _ d;
    0x5b BIT b3   E    _ d;
    0x5c BIT b3   H    _ d;
    0x5d BIT b3   L    _ d;
    0x5e BIT b3   (HL) _ d;
    0x5f BIT b3   A    _ d;

    0x60 BIT b4   B    _ d;
    0x61 BIT b4   C    _ d;
    0x62 BIT b4   D    _ d;
    0x63 BIT b4   E    _ d;
    0x64 BIT b4   H    _ d;
    0x65 BIT b4   L    _ d;
    0x66 BIT b4   (HL) _ d;
    0x67 BIT b4   A    _ d;
    0x68 BIT b5   B    _ d;
    0x69 BIT b5   C    _ d;
    0x6a BIT b5   D    _ d;
    0x6b BIT b5   E    _ d;
    0x6c BIT b5   H    _ d;
    0x6d BIT b5   L    _ d;
    0x6e BIT b5   (HL) _ d;
    0x6f BIT b5   A    _ d;

    0x70 BIT b6   B    _ d;
    0x71 BIT b6   C    _ d;
    0x72 BIT b6   D    _ d;
    0x73 BIT b6   E    _ d;
    0x74 BIT b6   H    _ d;
    0x75 BIT b6   L    _ d;
    0x76 BIT b6   (HL) _ d;
    0x77 BIT b6   A    _ d;
    0x78 BIT b7   B    _ d;
    0x79 BIT b7   C    _ d;
    0x7a BIT b7   D    _ d;
    0x7b BIT b7   E    _ d;
    0x7c BIT b7   H    _ d;
    0x7d BIT b7   L    _ d;
    0x7e BIT b7   (HL) _ d;
    0x7f BIT b7   A    _ d;

    0x80 RES b0   B    _ d;
    0x81 RES b0   C    _ d;
    0x82 RES b0   D    _ d;
    0x83 RES b0   E    _ d;
    0x84 RES b0   H    _ d;
    0x85 RES b0   L    _ d;
    0x86 RES b0   (HL) _ d;
    0x87 RES b0   A    _ d;
    0x88 RES b1   B    _ d;
    0x89 RES b1   C    _ d;
    0x8a RES b1   D    _ d;
    0x8b RES b1   E    _ d;
    0x8c RES b1   H    _ d;
    0x8d RES b1   L    _ d;
    0x8e RES b1   (HL) _ d;
    0x8f RES b1   A    _ d;

    0x90 RES b2   B    _ d;
    0x91 RES b2   C    _ d;
    0x92 RES b2   D    _ d;
    0x93 RES b2   E    _ d;
    0x94 RES b2   H    _ d;
    0x95 RES b2   L    _ d;
    0x96 RES b2   (HL) _ d;
    0x97 RES b2   A    _ d;
    0x98 RES b3   B    _ d;
    0x99 RES b3   C    _ d;
    0x9a RES b3   D    _ d;
    0x9b RES b3   E    _ d;
    0x9c RES b3   H    _ d;
    0x9d RES b3   L    _ d;
    0x9e RES b3   (HL) _ d;
    0x9f RES b3   A    _ d;

    0xa0 RES b4   B    _ d;
    0xa1 RES b4   C    _ d;
    0xa2 RES b4   D    _ d;
    0xa3 RES b4   E    _ d;
    0xa4 RES b4   H    _ d;
    0xa5 RES b4   L    _ d;
    0xa6 RES b4   (HL) _ d;
    0xa7 RES b4   A    _ d;
    0xa8 RES b5   B    _ d;
    0xa9 RES b5   C    _ d;
    0xaa RES b5   D    _ d;
    0xab RES b5   E    _ d;
    0xac RES b5   H    _ d;
    0xad RES b5   L    _ d;
    0xae RES b5   (HL) _ d;
    0xaf RES b5   A    _ d;

    0xb0 RES b6   B    _ d;
    0xb1 RES b6   C    _ d;
    0xb2 RES b6   D    _ d;
    0xb3 RES b6   E    _ d;
    0xb4 RES b6   H    _ d;
    0xb5 RES b6   L    _ d;
    0xb6 RES b6   (HL) _ d;
    0xb7 RES b6   A    _ d;
    0xb8 RES b7   B    _ d;
    0xb9 RES b7   C    _ d;
    0xba RES b7   D    _ d;
    0xbb RES b7   E    _ d;
    0xbc RES b7   H    _ d;
    0xbd RES b7   L    _ d;
    0xbe RES b7   (HL) _ d;
    0xbf RES b7   A    _ d;

    0xc0 SET b0   B    _ d;
    0xc1 SET b0   C    _ d;
    0xc2 SET b0   D    _ d;
    0xc3 SET b0   E    _ d;
    0xc4 SET b0   H    _ d;
    0xc5 SET b0   L    _ d;
    0xc6 SET b0   (HL) _ d;
    0xc7 SET b0   A    _ d;
    0xc8 SET b1   B    _ d;
    0xc9 SET b1   C    _ d;
    0xca SET b1   D    _ d;
    0xcb SET b1   E    _ d;
    0xcc SET b1   H    _ d;
    0xcd SET b1   L    _ d;
    0xce SET b1   (HL) _ d;
    0xcf SET b1   A    _ d;

    0xd0 SET b2   B    _ d;
    0xd1 SET b2   C    _ d;
    0xd2 SET b2   D    _ d;
    0xd3 SET b2   E    _ d;
    0xd4 SET b2   H    _ d;
    0xd5 SET b2   L    _ d;
    0xd6 SET b2   (HL) _ d;
    0xd7 SET b2   A    _ d;
    0xd8 SET b3   B    _ d;
    0xd9 SET b3   C    _ d;
    0xda SET b3   D    _ d;
    0xdb SET b3   E    _ d;
    0xdc SET b3   H    _ d;
    0xdd SET b3   L    _ d;
    0xde SET b3   (HL) _ d;
    0xdf SET b3   A    _ d;

    0xe0 SET b4   B    _ d;
    0xe1 SET b4   C    _ d;
    0xe2 SET b4   D    _ d;
    0xe3 SET b4   E    _ d;
    0xe4 SET b4   H    _ d;
    0xe5 SET b4   L    _ d;
    0xe6 SET b4   (HL) _ d;
    0xe7 SET b4   A    _ d;
    0xe8 SET b5   B    _ d;
    0xe9 SET b5   C    _ d;
    0xea SET b5   D    _ d;
    0xeb SET b5   E    _ d;
    0xec SET b5   H    _ d;
    0xed SET b5   L    _ d;
    0xee SET b5   (HL) _ d;
    0xef SET b5   A    _ d;

    0xf0 SET b6   B    _ d;
    0xf1 SET b6   C    _ d;
    0xf2 SET b6   D    _ d;
    0xf3 SET b6   E    _ d;
    0xf4 SET b6   H    _ d;
    0xf5 SET b6   L    _ d;
    0xf6 SET b6   (HL) _ d;
    0xf7 SET b6   A    _ d;
    0xf8 SET b7   B    _ d;
    0xf9 SET b7   C    _ d;
    0xfa SET b7   D    _ d;
    0xfb SET b7   E    _ d;
    0xfc SET b7   H    _ d;
    0xfd SET b7   L    _ d;
    0xfe SET b7   (HL) _ d;
    0xff SET b7   A    _ d;
}

// The `ED` page is mostly holes. Every opcode this table does not mention is a
// two-M1-cycle no-op on hardware, which is exactly what the macro's default
// row says; `ED_LISTED` is what keeps "deliberately a hole" distinguishable
// from "forgotten".
isa! {
    /// The `$ed` page: block moves, block I/O, 16-bit `ADC`/`SBC`, the
    /// interrupt-mode and `I`/`R` transfers, and a great many holes.
    ED ED_LISTED:
    0x40 IN   B    (C)  _ d;
    0x41 OUT  (C)  B    _ d;
    0x42 SBC  HL   BC   _ d;
    0x43 LD   (nn) BC   _ d;
    0x44 NEG  -    -    _ d;
    0x45 RETN -    -    _ d;
    0x46 IM   -    im0  _ d;
    0x47 LD   I    A    _ d;
    0x48 IN   C    (C)  _ d;
    0x49 OUT  (C)  C    _ d;
    0x4a ADC  HL   BC   _ d;
    0x4b LD   BC   (nn) _ d;
    0x4c NEG  -    -    _ u;
    0x4d RETI -    -    _ d;
    0x4e IM   -    im0  _ u;
    0x4f LD   R    A    _ d;

    0x50 IN   D    (C)  _ d;
    0x51 OUT  (C)  D    _ d;
    0x52 SBC  HL   DE   _ d;
    0x53 LD   (nn) DE   _ d;
    0x54 NEG  -    -    _ u;
    0x55 RETN -    -    _ u;
    0x56 IM   -    im1  _ d;
    0x57 LD   A    I    _ d;
    0x58 IN   E    (C)  _ d;
    0x59 OUT  (C)  E    _ d;
    0x5a ADC  HL   DE   _ d;
    0x5b LD   DE   (nn) _ d;
    0x5c NEG  -    -    _ u;
    0x5d RETN -    -    _ u;
    0x5e IM   -    im2  _ d;
    0x5f LD   A    R    _ d;

    0x60 IN   H    (C)  _ d;
    0x61 OUT  (C)  H    _ d;
    0x62 SBC  HL   HL   _ d;
    0x63 LD   (nn) HL   _ u;
    0x64 NEG  -    -    _ u;
    0x65 RETN -    -    _ u;
    0x66 IM   -    im0  _ u;
    0x67 RRD  -    -    _ d;
    0x68 IN   L    (C)  _ d;
    0x69 OUT  (C)  L    _ d;
    0x6a ADC  HL   HL   _ d;
    0x6b LD   HL   (nn) _ u;
    0x6c NEG  -    -    _ u;
    0x6d RETN -    -    _ u;
    0x6e IM   -    im0  _ u;
    0x6f RLD  -    -    _ d;

    0x70 IN   -    (C)  _ u;
    0x71 OUT  (C)  zero _ u;
    0x72 SBC  HL   SP   _ d;
    0x73 LD   (nn) SP   _ d;
    0x74 NEG  -    -    _ u;
    0x75 RETN -    -    _ u;
    0x76 IM   -    im1  _ u;
    0x77 NOP  -    -    _ u;
    0x78 IN   A    (C)  _ d;
    0x79 OUT  (C)  A    _ d;
    0x7a ADC  HL   SP   _ d;
    0x7b LD   SP   (nn) _ d;
    0x7c NEG  -    -    _ u;
    0x7d RETN -    -    _ u;
    0x7e IM   -    im2  _ u;
    0x7f NOP  -    -    _ u;

    0xa0 LDI  -    -    _ d;
    0xa1 CPI  -    -    _ d;
    0xa2 INI  -    -    _ d;
    0xa3 OUTI -    -    _ d;
    0xa8 LDD  -    -    _ d;
    0xa9 CPD  -    -    _ d;
    0xaa IND  -    -    _ d;
    0xab OUTD -    -    _ d;

    0xb0 LDIR -    -    _ d;
    0xb1 CPIR -    -    _ d;
    0xb2 INIR -    -    _ d;
    0xb3 OTIR -    -    _ d;
    0xb8 LDDR -    -    _ d;
    0xb9 CPDR -    -    _ d;
    0xba INDR -    -    _ d;
    0xbb OTDR -    -    _ d;
}

/// The unprefixed row for `opcode`.
#[inline]
#[must_use]
pub fn decode(opcode: u8) -> Insn {
    BASE[opcode as usize]
}

/// The `$cb`-page row for `opcode`.
#[inline]
#[must_use]
pub fn decode_cb(opcode: u8) -> Insn {
    CB[opcode as usize]
}

/// The `$ed`-page row for `opcode`.
///
/// The page's many holes decode as an undocumented `NOP`, which is what the
/// hardware does with them: two M1 cycles and nothing else.
#[inline]
#[must_use]
pub fn decode_ed(opcode: u8) -> Insn {
    ED[opcode as usize]
}

/// Rewrite a base-page row the way a `$dd` or `$fd` prefix does.
///
/// The prefix does not select a different instruction; it retargets the
/// decoder's HL path at an index register. Two rules follow from that, and
/// together they explain every apparent inconsistency in the index pages:
///
/// 1. If the row reaches memory through `(HL)`, *that* operand becomes
///    `(IX+d)` and the other one is left alone — which is why `LD H,(IX+d)`
///    loads the real `H` and there is no encoding for `LD IXH,(IX+d)`.
/// 2. Otherwise `HL`, `H` and `L` become `IX`, `IXH` and `IXL`, which is where
///    the undocumented half registers come from. They are not extra
///    registers, they are `H` and `L` seen through the prefix.
///
/// `EX DE,HL` is the one exception, and it is an exception on hardware too:
/// the exchange does not go through the addressing path the prefix retargets,
/// so `DD EB` is still `EX DE,HL` (Zilog UM0080; Sean Young, *Undocumented Z80
/// Documented* §5.4).
#[must_use]
pub fn index_substitute(insn: Insn, index: Index) -> Insn {
    let immune = insn.op == Op::EX && matches!(insn.dst, Operand::Reg16(R16::De));
    let displaced = !immune && (insn.dst.is_hl_indirect() || insn.src.is_hl_indirect());
    let rewrite = |operand: Operand| {
        if immune {
            return operand;
        }
        match operand {
            Operand::Ind(R16::Hl) => Operand::Idx(index.reg16()),
            // Rule 1: with a displacement in play the register halves stay put.
            Operand::Reg(R8::H) if !displaced => Operand::Reg(index.high()),
            Operand::Reg(R8::L) if !displaced => Operand::Reg(index.low()),
            Operand::Reg16(R16::Hl) => Operand::Reg16(index.reg16()),
            Operand::Ptr(R16::Hl) => Operand::Ptr(index.reg16()),
            other => other,
        }
    };
    let dst = rewrite(insn.dst);
    let src = rewrite(insn.src);
    let half = |o| matches!(o, Operand::Reg(R8::Ixh | R8::Ixl | R8::Iyh | R8::Iyl));
    // A prefix that changed nothing is a redundant byte in front of an
    // ordinary instruction, and a prefix that produced a half register is the
    // undocumented case. Everything else — `(IX+d)`, `ADD IX,rp`, `PUSH IX` —
    // is in UM0080 and keeps the row's class.
    let class = if (dst, src) == (insn.dst, insn.src) || half(dst) || half(src) {
        Class::Undocumented
    } else {
        insn.class
    };
    Insn {
        dst,
        src,
        class,
        ..insn
    }
}

/// The `DDCB`/`FDCB`-page row for `opcode`.
///
/// Every encoding on this page operates on `(IX+d)` whatever its `r` field
/// says — but the field is still decoded, and for `r != 6` the result is
/// *also* written to that register. `BIT` produces no result, so it copies
/// nothing (Sean Young, *Undocumented Z80 Documented* §6.3).
#[must_use]
pub fn decode_ddcb(opcode: u8, index: Index) -> Insn {
    let row = CB[opcode as usize];
    let target = Operand::Idx(index.reg16());
    let copy = if row.op == Op::BIT {
        None
    } else {
        R8::ENCODED[(opcode & 7) as usize]
    };
    let documented = row.class.is_documented() && opcode & 7 == 6;
    Insn {
        // The rotates carry their operand in `dst`; BIT, RES and SET carry the
        // bit index there and the operand in `src`.
        dst: if matches!(row.dst, Operand::Bit(_)) {
            row.dst
        } else {
            target
        },
        src: if matches!(row.dst, Operand::Bit(_)) {
            target
        } else {
            row.src
        },
        also: copy,
        class: if documented {
            Class::Documented
        } else {
            Class::Undocumented
        },
        ..row
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn every_base_and_cb_encoding_is_written_out() {
        // An unassigned row would silently decode as an undocumented NOP,
        // which is right for the ED page's holes and wrong everywhere else.
        assert!(BASE_LISTED.iter().all(|&b| b), "a base-page row is missing");
        assert!(CB_LISTED.iter().all(|&b| b), "a CB-page row is missing");
        assert_eq!(
            ED_LISTED.iter().filter(|&&b| b).count(),
            80,
            "the ED page has 64 rows at $40-$7f plus 16 block instructions"
        );
    }

    #[test]
    fn the_four_prefix_bytes_are_the_only_prefix_rows() {
        let prefixes: Vec<u8> = (0..=255u8).filter(|&o| decode(o).is_prefix()).collect();
        assert_eq!(prefixes, [0xcb, 0xdd, 0xed, 0xfd]);
    }

    #[test]
    fn an_index_prefix_renames_the_halves_but_not_across_a_displacement() {
        // LD H,L -> LD IXH,IXL: no memory operand, so both halves move.
        let ld_h_l = index_substitute(decode(0x65), Index::Ix);
        assert_eq!(ld_h_l.dst, Operand::Reg(R8::Ixh));
        assert_eq!(ld_h_l.src, Operand::Reg(R8::Ixl));

        // LD H,(HL) -> LD H,(IX+d): the displacement wins and H stays H.
        let ld_h_hl = index_substitute(decode(0x66), Index::Ix);
        assert_eq!(ld_h_hl.dst, Operand::Reg(R8::H));
        assert_eq!(ld_h_hl.src, Operand::Idx(R16::Ix));

        // LD (HL),H -> LD (IX+d),H, same reason from the other side.
        let ld_hl_h = index_substitute(decode(0x74), Index::Iy);
        assert_eq!(ld_hl_h.dst, Operand::Idx(R16::Iy));
        assert_eq!(ld_hl_h.src, Operand::Reg(R8::H));
    }

    #[test]
    fn ex_de_hl_ignores_the_prefix_but_ex_sp_hl_does_not() {
        let ex_de = index_substitute(decode(0xeb), Index::Ix);
        assert_eq!(
            (ex_de.op, ex_de.dst, ex_de.src),
            (Op::EX, decode(0xeb).dst, decode(0xeb).src)
        );
        // The prefix byte itself is still a redundant one, so the encoding is
        // undocumented even though the instruction it runs is not.
        assert_eq!(ex_de.class, Class::Undocumented);
        let ex_sp = index_substitute(decode(0xe3), Index::Ix);
        assert_eq!(ex_sp.dst, Operand::Ind(R16::Sp));
        assert_eq!(ex_sp.src, Operand::Reg16(R16::Ix));
    }

    #[test]
    fn the_ddcb_page_copies_its_result_into_the_encoded_register() {
        // DD CB d 00 is RLC (IX+d) *and* LD B,(IX+d)'s result.
        let rlc = decode_ddcb(0x00, Index::Ix);
        assert_eq!(rlc.op, Op::RLC);
        assert_eq!(rlc.dst, Operand::Idx(R16::Ix));
        assert_eq!(rlc.also, Some(R8::B));
        assert_eq!(rlc.class, Class::Undocumented);

        // r == 6 is the documented form and copies nothing.
        let rlc6 = decode_ddcb(0x06, Index::Ix);
        assert_eq!(rlc6.also, None);
        assert_eq!(rlc6.class, Class::Documented);

        // BIT has no result to copy, whatever the r field says.
        let bit = decode_ddcb(0x41, Index::Iy);
        assert_eq!(bit.op, Op::BIT);
        assert_eq!(bit.dst, Operand::Bit(0));
        assert_eq!(bit.src, Operand::Idx(R16::Iy));
        assert_eq!(bit.also, None);
    }

    #[test]
    fn operand_bytes_matches_the_encodings_length() {
        assert_eq!(decode(0x00).operand_bytes(), 0); // NOP
        assert_eq!(decode(0x06).operand_bytes(), 1); // LD B,n
        assert_eq!(decode(0x21).operand_bytes(), 2); // LD HL,nn
        assert_eq!(decode(0x32).operand_bytes(), 2); // LD (nn),A
        assert_eq!(decode(0x18).operand_bytes(), 1); // JR e
        assert_eq!(decode(0xdb).operand_bytes(), 1); // IN A,(n)
        // LD (IX+d),n carries the displacement *and* the immediate.
        assert_eq!(index_substitute(decode(0x36), Index::Ix).operand_bytes(), 2);
    }

    #[test]
    fn mnemonics_are_unique_per_operation() {
        let mut seen: Vec<&str> = Op::ALL.iter().map(|o| o.mnemonic()).collect();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(before, seen.len(), "two operations share a mnemonic");
    }
}
