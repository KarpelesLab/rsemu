//! The SM83 instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds one declarative description, [`BASE`] and [`PREFIXED`], from
//! which everything else is derived:
//!
//! - the interpreter's decode ([`decode`], [`decode_cb`]) *and* its bus
//!   sequence, which falls out of the operands rather than a cycle column;
//! - the disassembler ([`super::disasm`]), which formats from the same row;
//! - introspection: mnemonics, one-line summaries, and which encodings the chip
//!   does not implement at all.
//!
//! # Why there is no cycle column
//!
//! Deliberate, and the same decision the 6502 core made. An SM83 machine cycle
//! is four clocks, and every one of them is either a bus access or a documented
//! internal cycle. So the timing of `LD A,(HL)` is not "2 M-cycles" written in a
//! table — it is the opcode fetch plus the operand read, and the interpreter
//! charges a cycle *because* it made an access. [`Operand`] says what shape the
//! access has; `exec` turns that into the exact sequence. The one thing that
//! cannot be derived is an internal cycle with no access (the `PUSH`
//! predecrement, a taken branch's pipeline reload), and those are written out in
//! the interpreter where the reason for each is visible.
//!
//! The instruction *length* is derived the same way: one byte of opcode, plus
//! the prefix byte if any, plus whatever the operands say they carry
//! ([`Insn::bytes`]). There is no length column either.
//!
//! # What an SM83 is not
//!
//! It is neither a Z80 nor an 8080 (`docs/cpu/z80-sm83.md`). There is no `IX` or
//! `IY`, no alternate register set, no block instructions, no separate I/O
//! space, and no `IN`/`OUT`: the "I/O ports" are ordinary memory at `$FF00`.
//! What it adds is [`Op::LDH`] (the `$FF00`-page accesses), `LD (HL+),A` and its
//! three relatives, [`Op::SWAP`], [`Op::STOP`], and a [`Op::DAA`] whose
//! adjustment is driven by an **N** flag the Z80 spells the same way and uses
//! differently.
//!
//! Eleven opcodes are not implemented by the chip at all — `$D3`, `$DB`, `$DD`,
//! `$E3`, `$E4`, `$EB`, `$EC`, `$ED`, `$F4`, `$FC`, `$FD`. They are not "no
//! operation": executing one hangs the processor until reset, which is why they
//! decode to [`Op::LOCK`] rather than to `NOP`.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0) for the register model and the
//! flag rules, and the [gbdev opcode
//! tables](https://gbdev.io/gb-opcodes/optables/) for the matrix below. Timing
//! at sub-instruction granularity is from Gekkio's *Game Boy: Complete
//! Technical Reference*. No emulator source was consulted.

use core::fmt;

/// One of the seven addressable byte registers.
///
/// `(HL)` is deliberately **not** in here even though the encoding puts it at
/// index 6: it is a memory access, it costs an M-cycle, and folding it into the
/// register enum is how an interpreter ends up with free `(HL)` accesses. It is
/// [`Operand::MemHl`] instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg8 {
    /// `B`.
    B,
    /// `C`.
    C,
    /// `D`.
    D,
    /// `E`.
    E,
    /// `H`.
    H,
    /// `L`.
    L,
    /// The accumulator.
    A,
}

impl Reg8 {
    /// The register's name as an assembler spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg8::B => "B",
            Reg8::C => "C",
            Reg8::D => "D",
            Reg8::E => "E",
            Reg8::H => "H",
            Reg8::L => "L",
            Reg8::A => "A",
        }
    }
}

impl fmt::Display for Reg8 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A 16-bit register, as the encodings group them.
///
/// `AF` appears only in `PUSH`/`POP`, where the encoding's fourth slot holds it
/// instead of `SP`. Keeping both in one enum is what lets `PUSH` and `LD rr,n16`
/// share an operand type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg16 {
    /// `BC`.
    Bc,
    /// `DE`.
    De,
    /// `HL`.
    Hl,
    /// The stack pointer.
    Sp,
    /// The accumulator and flags, as `PUSH`/`POP` see them.
    Af,
}

impl Reg16 {
    /// The register's name as an assembler spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg16::Bc => "BC",
            Reg16::De => "DE",
            Reg16::Hl => "HL",
            Reg16::Sp => "SP",
            Reg16::Af => "AF",
        }
    }
}

impl fmt::Display for Reg16 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A branch condition, as `JR`, `JP`, `CALL` and `RET` take one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Cond {
    /// Zero clear.
    Nz,
    /// Zero set.
    Z,
    /// Carry clear.
    Nc,
    /// Carry set.
    C,
}

impl Cond {
    /// The condition's name as an assembler spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Cond::Nz => "NZ",
            Cond::Z => "Z",
            Cond::Nc => "NC",
            Cond::C => "C",
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Where an instruction's operand lives.
///
/// This is the *bus shape*, and it is what the cycle count actually depends on:
/// [`Operand::Reg`] is free, [`Operand::MemHl`] is one access, and
/// [`Operand::MemImm16`] is two immediate fetches and then an access. The
/// interpreter reads and writes through one pair of functions over this enum, so
/// `LD B,C`, `LD B,(HL)`, `LD (HL),B` and `LD (HL),n8` are one code path with
/// four different bus sequences.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operand {
    /// No operand at all.
    None,
    /// A byte register.
    Reg(Reg8),
    /// `(HL)` — the byte at `HL`. One access.
    MemHl,
    /// A 16-bit register.
    Reg16(Reg16),
    /// `(BC)` or `(DE)`.
    MemReg16(Reg16),
    /// `(HL+)` — the byte at `HL`, after which `HL` is incremented.
    MemHlInc,
    /// `(HL-)` — the byte at `HL`, after which `HL` is decremented.
    MemHlDec,
    /// `(C)` — the byte at `$FF00 + C`. The SM83's answer to an I/O space.
    MemHighC,
    /// An immediate byte, `n8`.
    Imm8,
    /// An immediate word, `n16`, little-endian.
    Imm16,
    /// A signed displacement, `e8`, relative to the *next* instruction.
    Rel8,
    /// `(n16)` — an absolute address.
    MemImm16,
    /// `(n8)` — `$FF00 + n8`, the `LDH` form.
    MemHighImm8,
    /// `SP+e8`, the address `LD HL,SP+e8` and `ADD SP,e8` compute.
    SpRel8,
    /// A branch condition rather than a value.
    Cond(Cond),
    /// A bit index, 0-7, for `BIT`/`RES`/`SET`.
    Bit(u8),
    /// An `RST` vector: `$00`, `$08`, … `$38`.
    Vector(u8),
}

impl Operand {
    /// How many bytes of the instruction stream this operand consumes.
    ///
    /// The single answer both the disassembler and the program counter use;
    /// there is no separate length column ([`Insn::bytes`]).
    #[must_use]
    pub const fn bytes(self) -> u16 {
        match self {
            Operand::Imm8 | Operand::Rel8 | Operand::MemHighImm8 | Operand::SpRel8 => 1,
            Operand::Imm16 | Operand::MemImm16 => 2,
            _ => 0,
        }
    }

    /// Whether reading or writing this operand touches the bus.
    ///
    /// Not used by the interpreter — which knows because it made the access —
    /// but by anything reasoning about an instruction without running it.
    #[must_use]
    pub const fn is_memory(self) -> bool {
        matches!(
            self,
            Operand::MemHl
                | Operand::MemReg16(_)
                | Operand::MemHlInc
                | Operand::MemHlDec
                | Operand::MemHighC
                | Operand::MemImm16
                | Operand::MemHighImm8
        )
    }

    /// Whether the operand is sixteen bits wide.
    ///
    /// `LD (n16),SP` and `LD (n16),A` differ only here, which is why the width
    /// is a property of the operand rather than of the mnemonic.
    #[must_use]
    pub const fn is_wide(self) -> bool {
        matches!(self, Operand::Reg16(_) | Operand::Imm16 | Operand::SpRel8)
    }

    // -- the spellings the table below uses ---------------------------------

    /// `B`.
    pub const B: Operand = Operand::Reg(Reg8::B);
    /// `C`, the register — not the condition and not `(C)`.
    pub const C: Operand = Operand::Reg(Reg8::C);
    /// `D`.
    pub const D: Operand = Operand::Reg(Reg8::D);
    /// `E`.
    pub const E: Operand = Operand::Reg(Reg8::E);
    /// `H`.
    pub const H: Operand = Operand::Reg(Reg8::H);
    /// `L`.
    pub const L: Operand = Operand::Reg(Reg8::L);
    /// `A`.
    pub const A: Operand = Operand::Reg(Reg8::A);
    /// `(HL)`.
    pub const MHL: Operand = Operand::MemHl;
    /// `BC`.
    pub const BC: Operand = Operand::Reg16(Reg16::Bc);
    /// `DE`.
    pub const DE: Operand = Operand::Reg16(Reg16::De);
    /// `HL`.
    pub const HL: Operand = Operand::Reg16(Reg16::Hl);
    /// `SP`.
    pub const SP: Operand = Operand::Reg16(Reg16::Sp);
    /// `AF`.
    pub const AF: Operand = Operand::Reg16(Reg16::Af);
    /// `(BC)`.
    pub const MBC: Operand = Operand::MemReg16(Reg16::Bc);
    /// `(DE)`.
    pub const MDE: Operand = Operand::MemReg16(Reg16::De);
    /// `(HL+)`.
    pub const MHLI: Operand = Operand::MemHlInc;
    /// `(HL-)`.
    pub const MHLD: Operand = Operand::MemHlDec;
    /// `(C)`, i.e. `$FF00 + C`.
    pub const MC: Operand = Operand::MemHighC;
    /// `n8`.
    pub const N8: Operand = Operand::Imm8;
    /// `n16`.
    pub const N16: Operand = Operand::Imm16;
    /// `e8`.
    pub const E8: Operand = Operand::Rel8;
    /// `(n16)`.
    pub const MN16: Operand = Operand::MemImm16;
    /// `(n8)`, i.e. `$FF00 + n8`.
    pub const MN8: Operand = Operand::MemHighImm8;
    /// `SP+e8`.
    pub const SPE8: Operand = Operand::SpRel8;
    /// The `NZ` condition.
    pub const CNZ: Operand = Operand::Cond(Cond::Nz);
    /// The `Z` condition.
    pub const CZ: Operand = Operand::Cond(Cond::Z);
    /// The `NC` condition.
    pub const CNC: Operand = Operand::Cond(Cond::Nc);
    /// The `C` condition.
    pub const CC: Operand = Operand::Cond(Cond::C);
    /// Nothing.
    pub const NONE: Operand = Operand::None;
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
///
/// The mnemonic is the variant name, so the two cannot disagree.
macro_rules! define_ops {
    ($($name:ident = $summary:literal,)*) => {
        /// One operation, independent of how its operands are addressed.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]), so a
        /// disassembler cannot print a name the interpreter does not implement.
        // Mnemonics are uppercase by universal convention; renaming them to
        // satisfy the acronym lint would make every reference unreadable.
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

            /// A one-line description, for a monitor or `rsemu describe`.
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
    ADC = "add with carry into the accumulator",
    ADD = "add into the accumulator, HL or SP",
    AND = "AND into the accumulator",
    BIT = "test one bit, setting Z from its complement",
    CALL = "push the return address and jump",
    CCF = "complement the carry flag",
    CP = "compare with the accumulator, discarding the result",
    CPL = "complement the accumulator",
    DAA = "decimal-adjust the accumulator after an addition or subtraction",
    DEC = "decrement",
    DI = "clear the interrupt master enable",
    EI = "set the interrupt master enable, one instruction late",
    HALT = "stop the clock until an interrupt is pending",
    INC = "increment",
    JP = "jump",
    JR = "jump relative to the next instruction",
    LD = "load",
    LDH = "load through the $FF00 page",
    LOCK = "not an instruction: the chip hangs until reset",
    NOP = "no operation",
    OR = "OR into the accumulator",
    POP = "pop a register pair off the stack",
    PREFIX = "not an instruction: $CB selects the second opcode page",
    PUSH = "push a register pair onto the stack",
    RES = "clear one bit",
    RET = "pop the return address and jump to it",
    RETI = "return and set the interrupt master enable",
    RL = "rotate left through carry",
    RLA = "rotate the accumulator left through carry, clearing Z",
    RLC = "rotate left, carry from bit 7",
    RLCA = "rotate the accumulator left, clearing Z",
    RR = "rotate right through carry",
    RRA = "rotate the accumulator right through carry, clearing Z",
    RRC = "rotate right, carry from bit 0",
    RRCA = "rotate the accumulator right, clearing Z",
    RST = "call one of the eight page-zero vectors",
    SBC = "subtract with borrow from the accumulator",
    SCF = "set the carry flag",
    SET = "set one bit",
    SLA = "shift left, zero into bit 0",
    SRA = "shift right, preserving bit 7",
    SRL = "shift right, zero into bit 7",
    STOP = "stop the clock and the LCD until a button is pressed",
    SUB = "subtract from the accumulator",
    SWAP = "exchange the two nibbles of a byte",
    XOR = "exclusive-OR into the accumulator",
}

/// Whether an encoding is one the chip implements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Class {
    /// A real instruction.
    Documented,
    /// One of the eleven holes in the matrix. Executing it hangs the processor.
    Unimplemented,
}

impl Class {
    /// Whether the chip implements this encoding.
    #[must_use]
    pub const fn is_documented(self) -> bool {
        matches!(self, Class::Documented)
    }
}

/// One row of the instruction description: everything known about an opcode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The destination, or the condition for a conditional branch.
    pub dst: Operand,
    /// The source.
    pub src: Operand,
    /// Whether the chip implements this encoding.
    pub class: Class,
    /// Whether this row is on the `$CB` page.
    pub prefixed: bool,
}

impl Insn {
    const fn new(op: Op, dst: Operand, src: Operand, class: Class) -> Insn {
        Insn {
            op,
            dst,
            src,
            class,
            prefixed: false,
        }
    }

    const fn cb(op: Op, dst: Operand, src: Operand) -> Insn {
        Insn {
            op,
            dst,
            src,
            class: Class::Documented,
            prefixed: true,
        }
    }

    /// Instruction length in bytes, opcode and `$CB` prefix included.
    ///
    /// Derived from the operands, so it cannot disagree with what the
    /// interpreter fetches.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        let prefix = if self.prefixed { 1 } else { 0 };
        1 + prefix + self.dst.bytes() + self.src.bytes()
    }

    /// The branch condition, if this instruction has one.
    #[must_use]
    pub const fn condition(self) -> Option<Cond> {
        match self.dst {
            Operand::Cond(c) => Some(c),
            _ => None,
        }
    }
}

/// Build [`BASE`] and [`LISTED`] from one list of rows.
///
/// All 256 encodings are written out; `LISTED` records which, so a test can
/// prove the matrix is complete rather than trusting that it is.
macro_rules! isa {
    ($($opcode:literal $op:ident $dst:ident $src:ident $class:ident;)*) => {
        /// The first opcode page, and the only description of it in the crate.
        pub static BASE: [Insn; 256] = {
            // The array must be initialised before it can be indexed in a const
            // block; every entry is then overwritten below, which `LISTED`
            // proves.
            let mut t = [Insn::new(
                Op::LOCK, Operand::NONE, Operand::NONE, Class::Unimplemented); 256];
            $(t[$opcode as usize] = Insn::new(
                Op::$op, Operand::$dst, Operand::$src, Class::$class);)*
            t
        };

        /// Which opcodes the table above actually assigns. Test scaffolding with
        /// a real job: an unassigned entry would silently decode as a lock-up.
        pub static LISTED: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

isa! {
    0x00 NOP  NONE NONE Documented;
    0x01 LD   BC   N16  Documented;
    0x02 LD   MBC  A    Documented;
    0x03 INC  BC   NONE Documented;
    0x04 INC  B    NONE Documented;
    0x05 DEC  B    NONE Documented;
    0x06 LD   B    N8   Documented;
    0x07 RLCA NONE NONE Documented;
    0x08 LD   MN16 SP   Documented;
    0x09 ADD  HL   BC   Documented;
    0x0a LD   A    MBC  Documented;
    0x0b DEC  BC   NONE Documented;
    0x0c INC  C    NONE Documented;
    0x0d DEC  C    NONE Documented;
    0x0e LD   C    N8   Documented;
    0x0f RRCA NONE NONE Documented;

    0x10 STOP NONE N8   Documented;
    0x11 LD   DE   N16  Documented;
    0x12 LD   MDE  A    Documented;
    0x13 INC  DE   NONE Documented;
    0x14 INC  D    NONE Documented;
    0x15 DEC  D    NONE Documented;
    0x16 LD   D    N8   Documented;
    0x17 RLA  NONE NONE Documented;
    0x18 JR   NONE E8   Documented;
    0x19 ADD  HL   DE   Documented;
    0x1a LD   A    MDE  Documented;
    0x1b DEC  DE   NONE Documented;
    0x1c INC  E    NONE Documented;
    0x1d DEC  E    NONE Documented;
    0x1e LD   E    N8   Documented;
    0x1f RRA  NONE NONE Documented;

    0x20 JR   CNZ  E8   Documented;
    0x21 LD   HL   N16  Documented;
    0x22 LD   MHLI A    Documented;
    0x23 INC  HL   NONE Documented;
    0x24 INC  H    NONE Documented;
    0x25 DEC  H    NONE Documented;
    0x26 LD   H    N8   Documented;
    0x27 DAA  NONE NONE Documented;
    0x28 JR   CZ   E8   Documented;
    0x29 ADD  HL   HL   Documented;
    0x2a LD   A    MHLI Documented;
    0x2b DEC  HL   NONE Documented;
    0x2c INC  L    NONE Documented;
    0x2d DEC  L    NONE Documented;
    0x2e LD   L    N8   Documented;
    0x2f CPL  NONE NONE Documented;

    0x30 JR   CNC  E8   Documented;
    0x31 LD   SP   N16  Documented;
    0x32 LD   MHLD A    Documented;
    0x33 INC  SP   NONE Documented;
    0x34 INC  MHL  NONE Documented;
    0x35 DEC  MHL  NONE Documented;
    0x36 LD   MHL  N8   Documented;
    0x37 SCF  NONE NONE Documented;
    0x38 JR   CC   E8   Documented;
    0x39 ADD  HL   SP   Documented;
    0x3a LD   A    MHLD Documented;
    0x3b DEC  SP   NONE Documented;
    0x3c INC  A    NONE Documented;
    0x3d DEC  A    NONE Documented;
    0x3e LD   A    N8   Documented;
    0x3f CCF  NONE NONE Documented;

    0x40 LD   B    B    Documented;
    0x41 LD   B    C    Documented;
    0x42 LD   B    D    Documented;
    0x43 LD   B    E    Documented;
    0x44 LD   B    H    Documented;
    0x45 LD   B    L    Documented;
    0x46 LD   B    MHL  Documented;
    0x47 LD   B    A    Documented;
    0x48 LD   C    B    Documented;
    0x49 LD   C    C    Documented;
    0x4a LD   C    D    Documented;
    0x4b LD   C    E    Documented;
    0x4c LD   C    H    Documented;
    0x4d LD   C    L    Documented;
    0x4e LD   C    MHL  Documented;
    0x4f LD   C    A    Documented;

    0x50 LD   D    B    Documented;
    0x51 LD   D    C    Documented;
    0x52 LD   D    D    Documented;
    0x53 LD   D    E    Documented;
    0x54 LD   D    H    Documented;
    0x55 LD   D    L    Documented;
    0x56 LD   D    MHL  Documented;
    0x57 LD   D    A    Documented;
    0x58 LD   E    B    Documented;
    0x59 LD   E    C    Documented;
    0x5a LD   E    D    Documented;
    0x5b LD   E    E    Documented;
    0x5c LD   E    H    Documented;
    0x5d LD   E    L    Documented;
    0x5e LD   E    MHL  Documented;
    0x5f LD   E    A    Documented;

    0x60 LD   H    B    Documented;
    0x61 LD   H    C    Documented;
    0x62 LD   H    D    Documented;
    0x63 LD   H    E    Documented;
    0x64 LD   H    H    Documented;
    0x65 LD   H    L    Documented;
    0x66 LD   H    MHL  Documented;
    0x67 LD   H    A    Documented;
    0x68 LD   L    B    Documented;
    0x69 LD   L    C    Documented;
    0x6a LD   L    D    Documented;
    0x6b LD   L    E    Documented;
    0x6c LD   L    H    Documented;
    0x6d LD   L    L    Documented;
    0x6e LD   L    MHL  Documented;
    0x6f LD   L    A    Documented;

    0x70 LD   MHL  B    Documented;
    0x71 LD   MHL  C    Documented;
    0x72 LD   MHL  D    Documented;
    0x73 LD   MHL  E    Documented;
    0x74 LD   MHL  H    Documented;
    0x75 LD   MHL  L    Documented;
    0x76 HALT NONE NONE Documented;
    0x77 LD   MHL  A    Documented;
    0x78 LD   A    B    Documented;
    0x79 LD   A    C    Documented;
    0x7a LD   A    D    Documented;
    0x7b LD   A    E    Documented;
    0x7c LD   A    H    Documented;
    0x7d LD   A    L    Documented;
    0x7e LD   A    MHL  Documented;
    0x7f LD   A    A    Documented;

    0x80 ADD  A    B    Documented;
    0x81 ADD  A    C    Documented;
    0x82 ADD  A    D    Documented;
    0x83 ADD  A    E    Documented;
    0x84 ADD  A    H    Documented;
    0x85 ADD  A    L    Documented;
    0x86 ADD  A    MHL  Documented;
    0x87 ADD  A    A    Documented;
    0x88 ADC  A    B    Documented;
    0x89 ADC  A    C    Documented;
    0x8a ADC  A    D    Documented;
    0x8b ADC  A    E    Documented;
    0x8c ADC  A    H    Documented;
    0x8d ADC  A    L    Documented;
    0x8e ADC  A    MHL  Documented;
    0x8f ADC  A    A    Documented;

    0x90 SUB  A    B    Documented;
    0x91 SUB  A    C    Documented;
    0x92 SUB  A    D    Documented;
    0x93 SUB  A    E    Documented;
    0x94 SUB  A    H    Documented;
    0x95 SUB  A    L    Documented;
    0x96 SUB  A    MHL  Documented;
    0x97 SUB  A    A    Documented;
    0x98 SBC  A    B    Documented;
    0x99 SBC  A    C    Documented;
    0x9a SBC  A    D    Documented;
    0x9b SBC  A    E    Documented;
    0x9c SBC  A    H    Documented;
    0x9d SBC  A    L    Documented;
    0x9e SBC  A    MHL  Documented;
    0x9f SBC  A    A    Documented;

    0xa0 AND  A    B    Documented;
    0xa1 AND  A    C    Documented;
    0xa2 AND  A    D    Documented;
    0xa3 AND  A    E    Documented;
    0xa4 AND  A    H    Documented;
    0xa5 AND  A    L    Documented;
    0xa6 AND  A    MHL  Documented;
    0xa7 AND  A    A    Documented;
    0xa8 XOR  A    B    Documented;
    0xa9 XOR  A    C    Documented;
    0xaa XOR  A    D    Documented;
    0xab XOR  A    E    Documented;
    0xac XOR  A    H    Documented;
    0xad XOR  A    L    Documented;
    0xae XOR  A    MHL  Documented;
    0xaf XOR  A    A    Documented;

    0xb0 OR   A    B    Documented;
    0xb1 OR   A    C    Documented;
    0xb2 OR   A    D    Documented;
    0xb3 OR   A    E    Documented;
    0xb4 OR   A    H    Documented;
    0xb5 OR   A    L    Documented;
    0xb6 OR   A    MHL  Documented;
    0xb7 OR   A    A    Documented;
    0xb8 CP   A    B    Documented;
    0xb9 CP   A    C    Documented;
    0xba CP   A    D    Documented;
    0xbb CP   A    E    Documented;
    0xbc CP   A    H    Documented;
    0xbd CP   A    L    Documented;
    0xbe CP   A    MHL  Documented;
    0xbf CP   A    A    Documented;

    0xc0 RET  CNZ  NONE Documented;
    0xc1 POP  BC   NONE Documented;
    0xc2 JP   CNZ  N16  Documented;
    0xc3 JP   NONE N16  Documented;
    0xc4 CALL CNZ  N16  Documented;
    0xc5 PUSH BC   NONE Documented;
    0xc6 ADD  A    N8   Documented;
    0xc7 RST  NONE NONE Documented;
    0xc8 RET  CZ   NONE Documented;
    0xc9 RET  NONE NONE Documented;
    0xca JP   CZ   N16  Documented;
    0xcb PREFIX NONE NONE Documented;
    0xcc CALL CZ   N16  Documented;
    0xcd CALL NONE N16  Documented;
    0xce ADC  A    N8   Documented;
    0xcf RST  NONE NONE Documented;

    0xd0 RET  CNC  NONE Documented;
    0xd1 POP  DE   NONE Documented;
    0xd2 JP   CNC  N16  Documented;
    0xd3 LOCK NONE NONE Unimplemented;
    0xd4 CALL CNC  N16  Documented;
    0xd5 PUSH DE   NONE Documented;
    0xd6 SUB  A    N8   Documented;
    0xd7 RST  NONE NONE Documented;
    0xd8 RET  CC   NONE Documented;
    0xd9 RETI NONE NONE Documented;
    0xda JP   CC   N16  Documented;
    0xdb LOCK NONE NONE Unimplemented;
    0xdc CALL CC   N16  Documented;
    0xdd LOCK NONE NONE Unimplemented;
    0xde SBC  A    N8   Documented;
    0xdf RST  NONE NONE Documented;

    0xe0 LDH  MN8  A    Documented;
    0xe1 POP  HL   NONE Documented;
    0xe2 LDH  MC   A    Documented;
    0xe3 LOCK NONE NONE Unimplemented;
    0xe4 LOCK NONE NONE Unimplemented;
    0xe5 PUSH HL   NONE Documented;
    0xe6 AND  A    N8   Documented;
    0xe7 RST  NONE NONE Documented;
    0xe8 ADD  SP   E8   Documented;
    0xe9 JP   NONE HL   Documented;
    0xea LD   MN16 A    Documented;
    0xeb LOCK NONE NONE Unimplemented;
    0xec LOCK NONE NONE Unimplemented;
    0xed LOCK NONE NONE Unimplemented;
    0xee XOR  A    N8   Documented;
    0xef RST  NONE NONE Documented;

    0xf0 LDH  A    MN8  Documented;
    0xf1 POP  AF   NONE Documented;
    0xf2 LDH  A    MC   Documented;
    0xf3 DI   NONE NONE Documented;
    0xf4 LOCK NONE NONE Unimplemented;
    0xf5 PUSH AF   NONE Documented;
    0xf6 OR   A    N8   Documented;
    0xf7 RST  NONE NONE Documented;
    0xf8 LD   HL   SPE8 Documented;
    0xf9 LD   SP   HL   Documented;
    0xfa LD   A    MN16 Documented;
    0xfb EI   NONE NONE Documented;
    0xfc LOCK NONE NONE Unimplemented;
    0xfd LOCK NONE NONE Unimplemented;
    0xfe CP   A    N8   Documented;
    0xff RST  NONE NONE Documented;
}

/// The `RST` vector an opcode encodes: `$C7`, `$CF`, … `$FF` call `$00`, `$08`,
/// … `$38`.
///
/// The one operand the table cannot spell, because it lives in the opcode's own
/// bits rather than beside it. [`decode`] fills it in, so nothing downstream
/// ever sees a `RST` without one.
const fn rst_vector(opcode: u8) -> u8 {
    opcode & 0x38
}

/// The register an encoding's low three bits select, or `(HL)` for 6.
const fn r8_operand(index: u8) -> Operand {
    match index & 7 {
        0 => Operand::B,
        1 => Operand::C,
        2 => Operand::D,
        3 => Operand::E,
        4 => Operand::H,
        5 => Operand::L,
        6 => Operand::MHL,
        _ => Operand::A,
    }
}

/// The `$CB` page, built from its own regularity rather than written out.
///
/// This page has none of the first one's exceptions: it is exactly eight
/// operations over eight operands, then `BIT`, `RES` and `SET` over eight bits
/// and the same eight operands. Writing 256 rows would be 256 chances to typo a
/// bit index, and the structure — which *is* the documentation of the page —
/// would be invisible in the result. So the rows are derived, in a `const`
/// block, from the encoding rule Pan Docs states.
pub static PREFIXED: [Insn; 256] = {
    let mut t = [Insn::cb(Op::RLC, Operand::B, Operand::NONE); 256];
    let mut i = 0usize;
    while i < 256 {
        let opcode = i as u8;
        let target = r8_operand(opcode);
        t[i] = if opcode < 0x40 {
            let op = match opcode >> 3 {
                0 => Op::RLC,
                1 => Op::RRC,
                2 => Op::RL,
                3 => Op::RR,
                4 => Op::SLA,
                5 => Op::SRA,
                6 => Op::SWAP,
                _ => Op::SRL,
            };
            Insn::cb(op, target, Operand::NONE)
        } else {
            let op = match opcode >> 6 {
                1 => Op::BIT,
                2 => Op::RES,
                _ => Op::SET,
            };
            Insn::cb(op, Operand::Bit((opcode >> 3) & 7), target)
        };
        i += 1;
    }
    t
};

/// Decode one first-page opcode.
///
/// Total: every one of the 256 encodings has a row, and the eleven the chip does
/// not implement say so ([`Class::Unimplemented`]) rather than pretending to be
/// `NOP`.
#[inline]
#[must_use]
pub fn decode(opcode: u8) -> Insn {
    let mut insn = BASE[opcode as usize];
    if insn.op == Op::RST {
        insn.dst = Operand::Vector(rst_vector(opcode));
    }
    insn
}

/// Decode one `$CB`-page opcode.
#[inline]
#[must_use]
pub fn decode_cb(opcode: u8) -> Insn {
    PREFIXED[opcode as usize]
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn every_opcode_has_a_row() {
        let missing: Vec<usize> = (0..256).filter(|i| !LISTED[*i]).collect();
        assert!(missing.is_empty(), "unlisted opcodes: {missing:?}");
    }

    #[test]
    fn exactly_eleven_encodings_are_holes() {
        let holes: Vec<u8> = (0..=255u8)
            .filter(|o| !decode(*o).class.is_documented())
            .collect();
        assert_eq!(
            holes,
            [
                0xd3, 0xdb, 0xdd, 0xe3, 0xe4, 0xeb, 0xec, 0xed, 0xf4, 0xfc, 0xfd
            ],
            "the SM83 matrix has eleven holes and these are they"
        );
    }

    #[test]
    fn lengths_are_derived_and_correct() {
        // Spot checks across every length class. If these hold the derivation
        // holds, because nothing else feeds it.
        assert_eq!(decode(0x00).bytes(), 1, "NOP");
        assert_eq!(decode(0x06).bytes(), 2, "LD B,n8");
        assert_eq!(decode(0x01).bytes(), 3, "LD BC,n16");
        assert_eq!(decode(0x08).bytes(), 3, "LD (n16),SP");
        assert_eq!(decode(0xe0).bytes(), 2, "LDH (n8),A");
        assert_eq!(decode(0xe2).bytes(), 1, "LDH (C),A");
        assert_eq!(decode(0xf8).bytes(), 2, "LD HL,SP+e8");
        assert_eq!(decode(0x18).bytes(), 2, "JR e8");
        assert_eq!(decode(0x20).bytes(), 2, "JR NZ,e8");
        assert_eq!(decode(0xc3).bytes(), 3, "JP n16");
        assert_eq!(decode(0xe9).bytes(), 1, "JP HL");
        assert_eq!(decode(0xcd).bytes(), 3, "CALL n16");
        assert_eq!(decode(0xff).bytes(), 1, "RST $38");
        assert_eq!(decode(0x10).bytes(), 2, "STOP");
        assert_eq!(decode_cb(0x00).bytes(), 2, "RLC B");
        assert_eq!(decode_cb(0x7e).bytes(), 2, "BIT 7,(HL)");
    }

    #[test]
    fn the_ld_block_is_a_square_with_one_hole_in_it() {
        for opcode in 0x40..=0x7fu8 {
            let insn = decode(opcode);
            if opcode == 0x76 {
                assert_eq!(insn.op, Op::HALT, "$76 is HALT, not LD (HL),(HL)");
                continue;
            }
            assert_eq!(insn.op, Op::LD, "{opcode:#04x}");
            assert_eq!(insn.dst, r8_operand(opcode >> 3), "{opcode:#04x} dst");
            assert_eq!(insn.src, r8_operand(opcode), "{opcode:#04x} src");
        }
    }

    #[test]
    fn rst_carries_its_vector() {
        for (i, opcode) in [0xc7u8, 0xcf, 0xd7, 0xdf, 0xe7, 0xef, 0xf7, 0xff]
            .into_iter()
            .enumerate()
        {
            let insn = decode(opcode);
            assert_eq!(insn.op, Op::RST);
            assert_eq!(insn.dst, Operand::Vector((i as u8) * 8));
        }
    }

    #[test]
    fn the_cb_page_covers_every_bit_of_every_register() {
        for bit in 0..8u8 {
            for reg in 0..8u8 {
                let base = 0x40 + bit * 8 + reg;
                assert_eq!(decode_cb(base).op, Op::BIT);
                assert_eq!(decode_cb(base).dst, Operand::Bit(bit));
                assert_eq!(decode_cb(base).src, r8_operand(reg));
                assert_eq!(decode_cb(base + 0x40).op, Op::RES);
                assert_eq!(decode_cb(base + 0x80).op, Op::SET);
            }
        }
    }

    #[test]
    fn every_op_appears_somewhere() {
        for op in Op::ALL {
            let in_base = BASE.iter().any(|i| i.op == *op);
            let in_cb = PREFIXED.iter().any(|i| i.op == *op);
            assert!(in_base || in_cb, "{op:?} is declared but never encoded");
        }
    }
}
