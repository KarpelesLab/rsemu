//! The SPC700 instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — so this file holds one declarative description,
//! [`TABLE`], from which the interpreter's decode ([`decode`]), the
//! disassembler ([`super::disasm`]) and the introspection (`rsemu describe`)
//! are all derived.
//!
//! # Shape of the matrix
//!
//! All 256 encodings are instructions; the SPC700 has no holes. Three columns
//! are perfectly regular and are generated rather than written out, because 48
//! hand-typed rows would be 48 chances to typo a bit number and would hide the
//! rule that *is* the documentation:
//!
//! * column 1 — `TCALL n`, `n` = the high nibble;
//! * column 2 — `SET1 dp.b` (even high nibble) / `CLR1 dp.b` (odd), `b` = the
//!   high three bits;
//! * column 3 — `BBS dp.b,rel` / `BBC dp.b,rel`, the same way.
//!
//! The other 208 rows are listed below in the SNESdev wiki's opcode-matrix
//! order.
//!
//! # Why there is no cycle column
//!
//! The same decision the 6502 and SM83 cores made: every SPC700 cycle is a
//! bus read, a bus write, or a documented internal cycle, and the interpreter
//! charges a cycle *because* it did one of those (`exec`). A table of cycle
//! counts would be a second description of the timing that could disagree
//! with the first.
//!
//! # Operand order
//!
//! Assembler syntax is `OP dst,src`, but where both operands carry a byte the
//! encoding puts the **source first** (`MOV dp,#imm` is `8F imm dp`; `ADC
//! dp,dp` is `89 src dst`). The one exception is a relative displacement,
//! which is always the last byte (`BBS dp.b,rel` is `03 dp rel`).
//! [`Insn::byte_order`] states the rule once for both the interpreter's fetch
//! order and the disassembler.
//!
//! # Sources
//!
//! The SNESdev wiki's *SPC-700 instruction set* page and nocash's *fullsnes*
//! (*SNES APU SPC700 CPU Overview* and its opcode tables) for the matrix and
//! the mnemonics. No emulator source of any licence was consulted.

use core::fmt;

/// Where an instruction's operand lives.
///
/// The bus shape: [`Operand::Dp`] is one fetched byte and an access in the
/// direct page, [`Operand::DpIndY`] is a fetched byte, two pointer reads and
/// an access, and so on. The direct page is `$00xx` or `$01xx` according to
/// the `P` flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operand {
    /// No operand.
    None,
    /// The accumulator.
    A,
    /// `X`.
    X,
    /// `Y`.
    Y,
    /// The stack pointer.
    Sp,
    /// The program status word.
    Psw,
    /// `YA`, the 16-bit pair `Y:A`.
    Ya,
    /// The carry flag, as the one-bit instructions name it.
    C,
    /// `#imm`.
    Imm,
    /// `dp`.
    Dp,
    /// `dp+X`, wrapping inside the direct page.
    DpX,
    /// `dp+Y`, wrapping inside the direct page.
    DpY,
    /// `!abs`.
    Abs,
    /// `!abs+X`.
    AbsX,
    /// `!abs+Y`.
    AbsY,
    /// `(X)` — the direct-page byte `X` points at.
    IndX,
    /// `(X)+` — the same, then `X` is incremented.
    IndXInc,
    /// `(Y)` — the direct-page byte `Y` points at.
    IndY,
    /// `[dp+X]` — a pointer in the direct page, indexed before the fetch.
    DpIndX,
    /// `[dp]+Y` — a pointer in the direct page, indexed after the fetch.
    DpIndY,
    /// `[!abs+X]` — `JMP`'s indexed indirect form.
    AbsIndX,
    /// A signed displacement from the next instruction.
    Rel,
    /// `dp.b` — bit `b` of a direct-page byte.
    DpBit(u8),
    /// `m.b` — a 13-bit absolute address and a bit number packed into a word.
    MemBit,
    /// `/m.b` — the complement of that bit.
    MemBitNot,
    /// `u` — `PCALL`'s offset into page `$FF`.
    Upage,
    /// `TCALL`'s table index, 0-15.
    Vector(u8),
}

impl Operand {
    /// How many bytes of the instruction stream this operand consumes.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        match self {
            Operand::Imm
            | Operand::Dp
            | Operand::DpX
            | Operand::DpY
            | Operand::DpIndX
            | Operand::DpIndY
            | Operand::Rel
            | Operand::DpBit(_)
            | Operand::Upage => 1,
            Operand::Abs
            | Operand::AbsX
            | Operand::AbsY
            | Operand::AbsIndX
            | Operand::MemBit
            | Operand::MemBitNot => 2,
            _ => 0,
        }
    }

    /// Whether reading or writing this operand touches memory.
    #[must_use]
    pub const fn is_memory(self) -> bool {
        matches!(
            self,
            Operand::Dp
                | Operand::DpX
                | Operand::DpY
                | Operand::Abs
                | Operand::AbsX
                | Operand::AbsY
                | Operand::IndX
                | Operand::IndXInc
                | Operand::IndY
                | Operand::DpIndX
                | Operand::DpIndY
                | Operand::DpBit(_)
                | Operand::MemBit
                | Operand::MemBitNot
        )
    }

    // -- the spellings the table below uses ---------------------------------

    /// Nothing.
    pub const NONE: Operand = Operand::None;
    /// `#imm`.
    pub const IMM: Operand = Operand::Imm;
    /// `dp`.
    pub const DP: Operand = Operand::Dp;
    /// `dp+X`.
    pub const DPX: Operand = Operand::DpX;
    /// `dp+Y`.
    pub const DPY: Operand = Operand::DpY;
    /// `!abs`.
    pub const ABS: Operand = Operand::Abs;
    /// `!abs+X`.
    pub const ABSX: Operand = Operand::AbsX;
    /// `!abs+Y`.
    pub const ABSY: Operand = Operand::AbsY;
    /// `(X)`.
    pub const INDX: Operand = Operand::IndX;
    /// `(X)+`.
    pub const INDXI: Operand = Operand::IndXInc;
    /// `(Y)`.
    pub const INDY: Operand = Operand::IndY;
    /// `[dp+X]`.
    pub const DPIX: Operand = Operand::DpIndX;
    /// `[dp]+Y`.
    pub const DPIY: Operand = Operand::DpIndY;
    /// `[!abs+X]`.
    pub const ABSIX: Operand = Operand::AbsIndX;
    /// `rel`.
    pub const REL: Operand = Operand::Rel;
    /// `m.b`.
    pub const MB: Operand = Operand::MemBit;
    /// `/m.b`.
    pub const NMB: Operand = Operand::MemBitNot;
    /// `u`.
    pub const UP: Operand = Operand::Upage;
    /// `YA`.
    pub const YA: Operand = Operand::Ya;
    /// `SP`.
    pub const SP: Operand = Operand::Sp;
    /// `PSW`.
    pub const PSW: Operand = Operand::Psw;
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
macro_rules! define_ops {
    ($($name:ident = $summary:literal,)*) => {
        /// One operation, independent of how its operands are addressed.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]).
        // Mnemonics are uppercase by universal convention.
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

            /// Every operation, in declaration order.
            pub const ALL: &'static [Op] = &[$(Op::$name,)*];
        }
    };
}

define_ops! {
    ADC = "add with carry",
    ADDW = "add a direct-page word to YA",
    AND = "bitwise AND",
    AND1 = "AND a memory bit (or its complement) into carry",
    ASL = "shift left, bit 7 into carry",
    BBC = "branch if a direct-page bit is clear",
    BBS = "branch if a direct-page bit is set",
    BCC = "branch if carry clear",
    BCS = "branch if carry set",
    BEQ = "branch if zero set",
    BMI = "branch if negative set",
    BNE = "branch if zero clear",
    BPL = "branch if negative clear",
    BRA = "branch always",
    BRK = "push PC and PSW, set B, clear I, and call the $FFDE vector",
    BVC = "branch if overflow clear",
    BVS = "branch if overflow set",
    CALL = "push the return address and jump",
    CBNE = "compare A with memory and branch if not equal",
    CLR1 = "clear a direct-page bit",
    CLRC = "clear carry",
    CLRP = "clear P: the direct page is $00xx",
    CLRV = "clear overflow and half-carry",
    CMP = "compare, discarding the result",
    CMPW = "compare YA with a direct-page word",
    DAA = "decimal-adjust A after an addition",
    DAS = "decimal-adjust A after a subtraction",
    DBNZ = "decrement and branch if not zero",
    DEC = "decrement",
    DECW = "decrement a direct-page word",
    DI = "clear the interrupt enable flag",
    DIV = "divide YA by X: quotient to A, remainder to Y",
    EI = "set the interrupt enable flag",
    EOR = "bitwise exclusive OR",
    EOR1 = "exclusive-OR a memory bit into carry",
    INC = "increment",
    INCW = "increment a direct-page word",
    JMP = "jump",
    LSR = "shift right, bit 0 into carry",
    MOV = "move",
    MOV1 = "move a memory bit to or from carry",
    MOVW = "move a word between YA and the direct page",
    MUL = "multiply Y by A into YA",
    NOP = "no operation",
    NOT1 = "complement a memory bit",
    NOTC = "complement carry",
    OR = "bitwise OR",
    OR1 = "OR a memory bit (or its complement) into carry",
    PCALL = "call into page $FF",
    POP = "pop a register off the stack",
    PUSH = "push a register onto the stack",
    RET = "pop the return address and jump to it",
    RETI = "pop PSW, then the return address",
    ROL = "rotate left through carry",
    ROR = "rotate right through carry",
    SBC = "subtract with borrow",
    SET1 = "set a direct-page bit",
    SETC = "set carry",
    SETP = "set P: the direct page is $01xx",
    SLEEP = "halt until an interrupt (which the SNES never raises)",
    STOP = "halt the processor until reset",
    SUBW = "subtract a direct-page word from YA",
    TCALL = "call through one of sixteen vectors at $FFC0-$FFDF",
    TCLR1 = "test memory against A, then clear A's bits in it",
    TSET1 = "test memory against A, then set A's bits in it",
    XCN = "exchange the nibbles of A",
}

/// One row of the instruction description.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The destination (or the tested operand, for a branch).
    pub dst: Operand,
    /// The source (or the displacement, for a branch).
    pub src: Operand,
}

impl Insn {
    const fn new(op: Op, dst: Operand, src: Operand) -> Insn {
        Insn { op, dst, src }
    }

    /// Instruction length in bytes, derived from the operands.
    #[must_use]
    pub const fn bytes(self) -> u16 {
        1 + self.dst.bytes() + self.src.bytes()
    }

    /// The two operands in the order their bytes appear in the instruction
    /// stream: source first, except that a displacement is always last.
    #[must_use]
    pub const fn byte_order(self) -> [Operand; 2] {
        if matches!(self.src, Operand::Rel) {
            [self.dst, self.src]
        } else {
            [self.src, self.dst]
        }
    }
}

/// Build the irregular part of [`TABLE`] from one list of rows.
macro_rules! isa {
    ($($opcode:literal $op:ident $dst:ident $src:ident;)*) => {
        /// The opcode matrix, and the only description of it in the crate.
        pub static TABLE: [Insn; 256] = {
            let mut t = [Insn::new(Op::NOP, Operand::NONE, Operand::NONE); 256];
            $(t[$opcode as usize] = Insn::new(Op::$op, Operand::$dst, Operand::$src);)*
            // The three regular columns.
            let mut hi = 0usize;
            while hi < 16 {
                let bit = (hi >> 1) as u8;
                t[(hi << 4) | 1] =
                    Insn::new(Op::TCALL, Operand::Vector(hi as u8), Operand::None);
                t[(hi << 4) | 2] = Insn::new(
                    if hi & 1 == 0 { Op::SET1 } else { Op::CLR1 },
                    Operand::DpBit(bit),
                    Operand::None,
                );
                t[(hi << 4) | 3] = Insn::new(
                    if hi & 1 == 0 { Op::BBS } else { Op::BBC },
                    Operand::DpBit(bit),
                    Operand::Rel,
                );
                hi += 1;
            }
            t
        };

        /// Which opcodes the table assigns, the generated columns included.
        /// Test scaffolding with a real job: an unassigned row would silently
        /// decode as `NOP`.
        pub static LISTED: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            let mut hi = 0usize;
            while hi < 16 {
                t[(hi << 4) | 1] = true;
                t[(hi << 4) | 2] = true;
                t[(hi << 4) | 3] = true;
                hi += 1;
            }
            t
        };
    };
}

isa! {
    0x00 NOP   NONE  NONE;
    0x04 OR    A     DP;
    0x05 OR    A     ABS;
    0x06 OR    A     INDX;
    0x07 OR    A     DPIX;
    0x08 OR    A     IMM;
    0x09 OR    DP    DP;
    0x0a OR1   C     MB;
    0x0b ASL   DP    NONE;
    0x0c ASL   ABS   NONE;
    0x0d PUSH  PSW   NONE;
    0x0e TSET1 ABS   NONE;
    0x0f BRK   NONE  NONE;

    0x10 BPL   NONE  REL;
    0x14 OR    A     DPX;
    0x15 OR    A     ABSX;
    0x16 OR    A     ABSY;
    0x17 OR    A     DPIY;
    0x18 OR    DP    IMM;
    0x19 OR    INDX  INDY;
    0x1a DECW  DP    NONE;
    0x1b ASL   DPX   NONE;
    0x1c ASL   A     NONE;
    0x1d DEC   X     NONE;
    0x1e CMP   X     ABS;
    0x1f JMP   ABSIX NONE;

    0x20 CLRP  NONE  NONE;
    0x24 AND   A     DP;
    0x25 AND   A     ABS;
    0x26 AND   A     INDX;
    0x27 AND   A     DPIX;
    0x28 AND   A     IMM;
    0x29 AND   DP    DP;
    0x2a OR1   C     NMB;
    0x2b ROL   DP    NONE;
    0x2c ROL   ABS   NONE;
    0x2d PUSH  A     NONE;
    0x2e CBNE  DP    REL;
    0x2f BRA   NONE  REL;

    0x30 BMI   NONE  REL;
    0x34 AND   A     DPX;
    0x35 AND   A     ABSX;
    0x36 AND   A     ABSY;
    0x37 AND   A     DPIY;
    0x38 AND   DP    IMM;
    0x39 AND   INDX  INDY;
    0x3a INCW  DP    NONE;
    0x3b ROL   DPX   NONE;
    0x3c ROL   A     NONE;
    0x3d INC   X     NONE;
    0x3e CMP   X     DP;
    0x3f CALL  ABS   NONE;

    0x40 SETP  NONE  NONE;
    0x44 EOR   A     DP;
    0x45 EOR   A     ABS;
    0x46 EOR   A     INDX;
    0x47 EOR   A     DPIX;
    0x48 EOR   A     IMM;
    0x49 EOR   DP    DP;
    0x4a AND1  C     MB;
    0x4b LSR   DP    NONE;
    0x4c LSR   ABS   NONE;
    0x4d PUSH  X     NONE;
    0x4e TCLR1 ABS   NONE;
    0x4f PCALL UP    NONE;

    0x50 BVC   NONE  REL;
    0x54 EOR   A     DPX;
    0x55 EOR   A     ABSX;
    0x56 EOR   A     ABSY;
    0x57 EOR   A     DPIY;
    0x58 EOR   DP    IMM;
    0x59 EOR   INDX  INDY;
    0x5a CMPW  YA    DP;
    0x5b LSR   DPX   NONE;
    0x5c LSR   A     NONE;
    0x5d MOV   X     A;
    0x5e CMP   Y     ABS;
    0x5f JMP   ABS   NONE;

    0x60 CLRC  NONE  NONE;
    0x64 CMP   A     DP;
    0x65 CMP   A     ABS;
    0x66 CMP   A     INDX;
    0x67 CMP   A     DPIX;
    0x68 CMP   A     IMM;
    0x69 CMP   DP    DP;
    0x6a AND1  C     NMB;
    0x6b ROR   DP    NONE;
    0x6c ROR   ABS   NONE;
    0x6d PUSH  Y     NONE;
    0x6e DBNZ  DP    REL;
    0x6f RET   NONE  NONE;

    0x70 BVS   NONE  REL;
    0x74 CMP   A     DPX;
    0x75 CMP   A     ABSX;
    0x76 CMP   A     ABSY;
    0x77 CMP   A     DPIY;
    0x78 CMP   DP    IMM;
    0x79 CMP   INDX  INDY;
    0x7a ADDW  YA    DP;
    0x7b ROR   DPX   NONE;
    0x7c ROR   A     NONE;
    0x7d MOV   A     X;
    0x7e CMP   Y     DP;
    0x7f RETI  NONE  NONE;

    0x80 SETC  NONE  NONE;
    0x84 ADC   A     DP;
    0x85 ADC   A     ABS;
    0x86 ADC   A     INDX;
    0x87 ADC   A     DPIX;
    0x88 ADC   A     IMM;
    0x89 ADC   DP    DP;
    0x8a EOR1  C     MB;
    0x8b DEC   DP    NONE;
    0x8c DEC   ABS   NONE;
    0x8d MOV   Y     IMM;
    0x8e POP   PSW   NONE;
    0x8f MOV   DP    IMM;

    0x90 BCC   NONE  REL;
    0x94 ADC   A     DPX;
    0x95 ADC   A     ABSX;
    0x96 ADC   A     ABSY;
    0x97 ADC   A     DPIY;
    0x98 ADC   DP    IMM;
    0x99 ADC   INDX  INDY;
    0x9a SUBW  YA    DP;
    0x9b DEC   DPX   NONE;
    0x9c DEC   A     NONE;
    0x9d MOV   X     SP;
    0x9e DIV   YA    X;
    0x9f XCN   A     NONE;

    0xa0 EI    NONE  NONE;
    0xa4 SBC   A     DP;
    0xa5 SBC   A     ABS;
    0xa6 SBC   A     INDX;
    0xa7 SBC   A     DPIX;
    0xa8 SBC   A     IMM;
    0xa9 SBC   DP    DP;
    0xaa MOV1  C     MB;
    0xab INC   DP    NONE;
    0xac INC   ABS   NONE;
    0xad CMP   Y     IMM;
    0xae POP   A     NONE;
    0xaf MOV   INDXI A;

    0xb0 BCS   NONE  REL;
    0xb4 SBC   A     DPX;
    0xb5 SBC   A     ABSX;
    0xb6 SBC   A     ABSY;
    0xb7 SBC   A     DPIY;
    0xb8 SBC   DP    IMM;
    0xb9 SBC   INDX  INDY;
    0xba MOVW  YA    DP;
    0xbb INC   DPX   NONE;
    0xbc INC   A     NONE;
    0xbd MOV   SP    X;
    0xbe DAS   A     NONE;
    0xbf MOV   A     INDXI;

    0xc0 DI    NONE  NONE;
    0xc4 MOV   DP    A;
    0xc5 MOV   ABS   A;
    0xc6 MOV   INDX  A;
    0xc7 MOV   DPIX  A;
    0xc8 CMP   X     IMM;
    0xc9 MOV   ABS   X;
    0xca MOV1  MB    C;
    0xcb MOV   DP    Y;
    0xcc MOV   ABS   Y;
    0xcd MOV   X     IMM;
    0xce POP   X     NONE;
    0xcf MUL   YA    NONE;

    0xd0 BNE   NONE  REL;
    0xd4 MOV   DPX   A;
    0xd5 MOV   ABSX  A;
    0xd6 MOV   ABSY  A;
    0xd7 MOV   DPIY  A;
    0xd8 MOV   DP    X;
    0xd9 MOV   DPY   X;
    0xda MOVW  DP    YA;
    0xdb MOV   DPX   Y;
    0xdc DEC   Y     NONE;
    0xdd MOV   A     Y;
    0xde CBNE  DPX   REL;
    0xdf DAA   A     NONE;

    0xe0 CLRV  NONE  NONE;
    0xe4 MOV   A     DP;
    0xe5 MOV   A     ABS;
    0xe6 MOV   A     INDX;
    0xe7 MOV   A     DPIX;
    0xe8 MOV   A     IMM;
    0xe9 MOV   X     ABS;
    0xea NOT1  MB    NONE;
    0xeb MOV   Y     DP;
    0xec MOV   Y     ABS;
    0xed NOTC  NONE  NONE;
    0xee POP   Y     NONE;
    0xef SLEEP NONE  NONE;

    0xf0 BEQ   NONE  REL;
    0xf4 MOV   A     DPX;
    0xf5 MOV   A     ABSX;
    0xf6 MOV   A     ABSY;
    0xf7 MOV   A     DPIY;
    0xf8 MOV   X     DP;
    0xf9 MOV   X     DPY;
    0xfa MOV   DP    DP;
    0xfb MOV   Y     DPX;
    0xfc INC   Y     NONE;
    0xfd MOV   Y     A;
    0xfe DBNZ  Y     REL;
    0xff STOP  NONE  NONE;
}

/// Decode one opcode. Total: every encoding has a row.
#[inline]
#[must_use]
pub fn decode(opcode: u8) -> Insn {
    TABLE[opcode as usize]
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn every_opcode_has_a_row() {
        let missing: Vec<usize> = (0..256).filter(|i| !LISTED[*i]).collect();
        assert!(missing.is_empty(), "unlisted opcodes: {missing:02x?}");
    }

    #[test]
    fn lengths_are_derived_and_correct() {
        assert_eq!(decode(0x00).bytes(), 1, "NOP");
        assert_eq!(decode(0xe8).bytes(), 2, "MOV A,#imm");
        assert_eq!(decode(0xe5).bytes(), 3, "MOV A,!abs");
        assert_eq!(decode(0x8f).bytes(), 3, "MOV dp,#imm");
        assert_eq!(decode(0xfa).bytes(), 3, "MOV dp,dp");
        assert_eq!(decode(0x03).bytes(), 3, "BBS dp.0,rel");
        assert_eq!(decode(0x2e).bytes(), 3, "CBNE dp,rel");
        assert_eq!(decode(0xfe).bytes(), 2, "DBNZ Y,rel");
        assert_eq!(decode(0x1f).bytes(), 3, "JMP [!abs+X]");
        assert_eq!(decode(0x4f).bytes(), 2, "PCALL u");
        assert_eq!(decode(0xaa).bytes(), 3, "MOV1 C,m.b");
        assert_eq!(decode(0x01).bytes(), 1, "TCALL 0");
        assert_eq!(decode(0xaf).bytes(), 1, "MOV (X)+,A");
    }

    #[test]
    fn the_regular_columns_carry_their_numbers() {
        for hi in 0..16u8 {
            assert_eq!(decode(hi << 4 | 1).dst, Operand::Vector(hi));
            let set = decode(hi << 4 | 2);
            assert_eq!(set.op, if hi & 1 == 0 { Op::SET1 } else { Op::CLR1 });
            assert_eq!(set.dst, Operand::DpBit(hi >> 1));
            let br = decode(hi << 4 | 3);
            assert_eq!(br.op, if hi & 1 == 0 { Op::BBS } else { Op::BBC });
            assert_eq!(br.src, Operand::Rel);
        }
    }

    #[test]
    fn source_bytes_come_first_except_a_displacement() {
        assert_eq!(decode(0x8f).byte_order(), [Operand::Imm, Operand::Dp]);
        assert_eq!(decode(0x2e).byte_order(), [Operand::Dp, Operand::Rel]);
    }

    #[test]
    fn every_op_appears_somewhere() {
        for op in Op::ALL {
            assert!(TABLE.iter().any(|i| i.op == *op), "{op:?} is never encoded");
        }
    }
}
