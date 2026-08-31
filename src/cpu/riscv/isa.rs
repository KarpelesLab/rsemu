//! The instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds two declarative descriptions and nothing else derives from
//! anywhere but them:
//!
//! * [`TABLE`] — the 32-bit encodings, one row per instruction, from which the
//!   interpreter's [`decode`] and the disassembler both read.
//! * [`CTABLE`] — the 16-bit `C` encodings, from which [`decode_compressed`]
//!   and [`expand`] both read. Every compressed instruction is defined by the
//!   specification as an alias for one 32-bit instruction, so `expand` is the
//!   whole of the `C` extension's semantics and the interpreter needs no
//!   second implementation of anything.
//!
//! # Why there is no cycle column
//!
//! Deliberate, and for the same reason the 6502 has none: a cycle is charged
//! *because* an access was made, not because a table says an instruction takes
//! four of them. RISC-V does not architecturally define instruction timing at
//! all — that is a property of a particular implementation's pipeline — so a
//! cycle table here would be invention. What the interpreter counts is bus
//! accesses, which is a fact about the machine being modelled.
//!
//! # Decoding
//!
//! Every row is a (mask, match) pair, which is how the specification's opcode
//! maps are actually organised. A linear scan over 200 rows per instruction
//! would be absurd, so [`INDEX`] buckets the rows by the seven-bit opcode
//! field at compile time and [`decode`] scans only that bucket — typically two
//! or three rows. The index is built by a `const fn` from `TABLE` itself, so
//! it cannot fall out of step with it.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA* (CC-BY-4.0)
//! — the RV32I and RV64I base chapters, the "M", "A", "F", "D" and "C"
//! standard extension chapters and their opcode maps — and *Volume II:
//! Privileged Architecture* for `ECALL`/`EBREAK`/`MRET`/`SRET`/`WFI` and
//! `SFENCE.VMA`. Field positions and immediate scrambles are transcribed from
//! the instruction-format figures in Volume I.

use core::fmt;

/// The register width a core is configured for.
///
/// A construction property, never a `#[cfg]`: one build of rsemu has to be
/// able to run an RV64 Linux machine and an RV32 microcontroller, and the
/// difference between their cores is this value (`ROADMAP.md` §6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Xlen {
    /// 32-bit registers and addresses.
    Rv32,
    /// 64-bit registers and addresses.
    Rv64,
}

impl Xlen {
    /// Register width in bits.
    #[inline]
    #[must_use]
    pub const fn bits(self) -> u32 {
        match self {
            Xlen::Rv32 => 32,
            Xlen::Rv64 => 64,
        }
    }

    /// Sign-extend a value from this width to the 64 bits a register is held
    /// in.
    ///
    /// RV32 register values are kept in their sign-extended 64-bit form, which
    /// is what makes the shared interpreter possible: signed comparison,
    /// unsigned comparison and equality all give the same answers on the
    /// widened values as they do on the originals.
    #[inline]
    #[must_use]
    pub const fn sext(self, value: u64) -> u64 {
        match self {
            Xlen::Rv32 => value as u32 as i32 as i64 as u64,
            Xlen::Rv64 => value,
        }
    }

    /// Truncate a value to this width, zero-extending the result.
    ///
    /// This is the *address* rule, and it is not [`Xlen::sext`]: an RV32
    /// hart's address bus is 32 bits wide, so a program counter or an
    /// effective address is the low 32 bits and nothing above them. Register
    /// *values* are held sign-extended, which is a different convention for a
    /// different reason — mixing the two is how an RV32 guest ends up fetching
    /// from `0xffff_ffff_8000_0000`.
    #[inline]
    #[must_use]
    pub const fn trunc(self, value: u64) -> u64 {
        match self {
            Xlen::Rv32 => value & 0xffff_ffff,
            Xlen::Rv64 => value,
        }
    }

    /// The name the `.machine` file and `misa` use: `rv32` or `rv64`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Xlen::Rv32 => "rv32",
            Xlen::Rv64 => "rv64",
        }
    }
}

impl fmt::Display for Xlen {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which standard extension an encoding belongs to.
///
/// Used to gate decoding on the core's configuration — an `F` instruction on a
/// core built without `F` raises an illegal-instruction exception rather than
/// executing — and to build the `misa` CSR.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ext {
    /// The base integer set.
    I,
    /// Integer multiply and divide.
    M,
    /// Atomics: `LR`/`SC` and the `AMO` family.
    A,
    /// Single-precision floating point.
    F,
    /// Double-precision floating point.
    D,
    /// The compressed 16-bit encodings.
    C,
    /// CSR access.
    Zicsr,
    /// Instruction-stream fence.
    Zifencei,
    /// The privileged architecture: traps, returns and fences.
    Priv,
}

impl Ext {
    /// The letter or name as `misa` and the ISA string spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Ext::I => "i",
            Ext::M => "m",
            Ext::A => "a",
            Ext::F => "f",
            Ext::D => "d",
            Ext::C => "c",
            Ext::Zicsr => "zicsr",
            Ext::Zifencei => "zifencei",
            Ext::Priv => "priv",
        }
    }
}

/// Which register widths an encoding exists on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Width {
    /// Both RV32 and RV64.
    Any,
    /// RV32 only — the encoding means something else on RV64.
    Rv32,
    /// RV64 only: the `*W` word instructions and the 64-bit widths.
    Rv64,
}

impl Width {
    /// Whether this encoding exists on `xlen`.
    #[inline]
    #[must_use]
    pub const fn allows(self, xlen: Xlen) -> bool {
        matches!(
            (self, xlen),
            (Width::Any, _) | (Width::Rv32, Xlen::Rv32) | (Width::Rv64, Xlen::Rv64)
        )
    }
}

/// How an instruction's operands are laid out, and therefore how the
/// disassembler prints it.
///
/// The specification's format letters, plus the handful of shapes that print
/// differently even though they share a format — a load prints `rd, imm(rs1)`
/// where an ordinary I-type prints `rd, rs1, imm`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Fmt {
    /// `rd, rs1, rs2`.
    R,
    /// `rd, rs1, imm`.
    I,
    /// `rd, rs1, shamt`.
    Shift,
    /// `rd, imm(rs1)` — a load, or `JALR`.
    Load,
    /// `rs2, imm(rs1)` — a store.
    Store,
    /// `rs1, rs2, target`.
    Branch,
    /// `rd, imm` with a 20-bit upper immediate.
    U,
    /// `rd, target`.
    Jump,
    /// `pred, succ` — `FENCE`'s two ordering sets.
    Fence,
    /// No operands at all.
    None,
    /// `rs1, rs2` — `SFENCE.VMA`'s address and ASID.
    Sfence,
    /// `rd, csr, rs1`.
    Csr,
    /// `rd, csr, uimm`.
    CsrImm,
    /// `rd, (rs1)` — `LR`.
    AmoLoad,
    /// `rd, rs2, (rs1)` — `SC` and every `AMO`.
    Amo,
    /// `fd, imm(rs1)`.
    FpLoad,
    /// `fs2, imm(rs1)`.
    FpStore,
    /// `fd, fs1, fs2` (plus a rounding mode).
    FpR,
    /// `fd, fs1` (plus a rounding mode).
    FpUnary,
    /// `fd, fs1, fs2, fs3` (plus a rounding mode).
    FpR4,
    /// `rd, fs1, fs2` — a comparison, whose result is an integer.
    FpCmp,
    /// `rd, fs1` — a move or convert out of the float registers.
    FpToInt,
    /// `fd, rs1` — a move or convert into the float registers.
    FpFromInt,
}

/// One row of the instruction description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The bits of the encoding that are fixed.
    pub mask: u32,
    /// What those bits must be.
    pub bits: u32,
    /// Operand layout, and therefore the disassembly.
    pub fmt: Fmt,
    /// Which extension it belongs to.
    pub ext: Ext,
    /// Which register widths it exists on.
    pub width: Width,
}

/// Declare the operation enum, its mnemonics, its summaries and the decode
/// table from one list of rows.
///
/// The mnemonic is a literal beside the variant, because RISC-V mnemonics
/// contain dots (`fmadd.s`, `amoswap.w`) and a Rust identifier cannot.
macro_rules! isa {
    ($($mask:literal $bits:literal $op:ident $mn:literal $fmt:ident $ext:ident $width:ident $summary:literal;)*) => {
        /// One operation, independent of how it is encoded.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum Op {
            $(
                #[doc = $summary]
                $op,
            )*
        }

        impl Op {
            /// The assembler mnemonic.
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(Op::$op => $mn,)* }
            }

            /// A one-line description, for `rsemu describe` and the monitor.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(Op::$op => $summary,)* }
            }

            /// Every operation this core implements, in encoding order.
            pub const ALL: &'static [Op] = &[$(Op::$op,)*];
        }

        /// The decode table: the only description of the 32-bit instruction
        /// set in the crate.
        pub static TABLE: &[Insn] = &[
            $(Insn {
                op: Op::$op,
                mask: $mask,
                bits: $bits,
                fmt: Fmt::$fmt,
                ext: Ext::$ext,
                width: Width::$width,
            },)*
        ];
    };
}

isa! {
    // -- LOAD ---------------------------------------------------------------
    0x0000707f 0x00000003 Lb    "lb"    Load  I Any   "load a sign-extended byte";
    0x0000707f 0x00001003 Lh    "lh"    Load  I Any   "load a sign-extended halfword";
    0x0000707f 0x00002003 Lw    "lw"    Load  I Any   "load a sign-extended word";
    0x0000707f 0x00003003 Ld    "ld"    Load  I Rv64  "load a doubleword";
    0x0000707f 0x00004003 Lbu   "lbu"   Load  I Any   "load a zero-extended byte";
    0x0000707f 0x00005003 Lhu   "lhu"   Load  I Any   "load a zero-extended halfword";
    0x0000707f 0x00006003 Lwu   "lwu"   Load  I Rv64  "load a zero-extended word";

    // -- LOAD-FP ------------------------------------------------------------
    0x0000707f 0x00002007 Flw   "flw"   FpLoad F Any  "load a single-precision float, NaN-boxed";
    0x0000707f 0x00003007 Fld   "fld"   FpLoad D Any  "load a double-precision float";

    // -- MISC-MEM -----------------------------------------------------------
    0x0000707f 0x0000000f Fence "fence" Fence I Any        "order memory accesses";
    0x0000707f 0x0000100f FenceI "fence.i" None Zifencei Any "synchronise the instruction stream";

    // -- OP-IMM -------------------------------------------------------------
    0x0000707f 0x00000013 Addi  "addi"  I     I Any   "add a sign-extended immediate";
    0xfc00707f 0x00001013 Slli  "slli"  Shift I Any   "shift left logical by an immediate";
    0x0000707f 0x00002013 Slti  "slti"  I     I Any   "set if less than an immediate, signed";
    0x0000707f 0x00003013 Sltiu "sltiu" I     I Any   "set if less than an immediate, unsigned";
    0x0000707f 0x00004013 Xori  "xori"  I     I Any   "exclusive-OR with an immediate";
    0xfc00707f 0x00005013 Srli  "srli"  Shift I Any   "shift right logical by an immediate";
    0xfc00707f 0x40005013 Srai  "srai"  Shift I Any   "shift right arithmetic by an immediate";
    0x0000707f 0x00006013 Ori   "ori"   I     I Any   "OR with an immediate";
    0x0000707f 0x00007013 Andi  "andi"  I     I Any   "AND with an immediate";

    // -- AUIPC --------------------------------------------------------------
    0x0000007f 0x00000017 Auipc "auipc" U     I Any   "add an upper immediate to the PC";

    // -- OP-IMM-32 ----------------------------------------------------------
    0x0000707f 0x0000001b Addiw "addiw" I     I Rv64  "add an immediate to a word, sign-extended";
    0xfe00707f 0x0000101b Slliw "slliw" Shift I Rv64  "shift a word left logical by an immediate";
    0xfe00707f 0x0000501b Srliw "srliw" Shift I Rv64  "shift a word right logical by an immediate";
    0xfe00707f 0x4000501b Sraiw "sraiw" Shift I Rv64  "shift a word right arithmetic by an immediate";

    // -- STORE --------------------------------------------------------------
    0x0000707f 0x00000023 Sb    "sb"    Store I Any   "store a byte";
    0x0000707f 0x00001023 Sh    "sh"    Store I Any   "store a halfword";
    0x0000707f 0x00002023 Sw    "sw"    Store I Any   "store a word";
    0x0000707f 0x00003023 Sd    "sd"    Store I Rv64  "store a doubleword";

    // -- STORE-FP -----------------------------------------------------------
    0x0000707f 0x00002027 Fsw   "fsw"   FpStore F Any "store a single-precision float";
    0x0000707f 0x00003027 Fsd   "fsd"   FpStore D Any "store a double-precision float";

    // -- AMO ----------------------------------------------------------------
    0xf800707f 0x1000202f LrW      "lr.w"      AmoLoad A Any "load reserved, word";
    0xf800707f 0x1800202f ScW      "sc.w"      Amo     A Any "store conditional, word";
    0xf800707f 0x0800202f AmoswapW "amoswap.w" Amo     A Any "atomic swap, word";
    0xf800707f 0x0000202f AmoaddW  "amoadd.w"  Amo     A Any "atomic add, word";
    0xf800707f 0x2000202f AmoxorW  "amoxor.w"  Amo     A Any "atomic exclusive-OR, word";
    0xf800707f 0x6000202f AmoandW  "amoand.w"  Amo     A Any "atomic AND, word";
    0xf800707f 0x4000202f AmoorW   "amoor.w"   Amo     A Any "atomic OR, word";
    0xf800707f 0x8000202f AmominW  "amomin.w"  Amo     A Any "atomic signed minimum, word";
    0xf800707f 0xa000202f AmomaxW  "amomax.w"  Amo     A Any "atomic signed maximum, word";
    0xf800707f 0xc000202f AmominuW "amominu.w" Amo     A Any "atomic unsigned minimum, word";
    0xf800707f 0xe000202f AmomaxuW "amomaxu.w" Amo     A Any "atomic unsigned maximum, word";
    0xf800707f 0x1000302f LrD      "lr.d"      AmoLoad A Rv64 "load reserved, doubleword";
    0xf800707f 0x1800302f ScD      "sc.d"      Amo     A Rv64 "store conditional, doubleword";
    0xf800707f 0x0800302f AmoswapD "amoswap.d" Amo     A Rv64 "atomic swap, doubleword";
    0xf800707f 0x0000302f AmoaddD  "amoadd.d"  Amo     A Rv64 "atomic add, doubleword";
    0xf800707f 0x2000302f AmoxorD  "amoxor.d"  Amo     A Rv64 "atomic exclusive-OR, doubleword";
    0xf800707f 0x6000302f AmoandD  "amoand.d"  Amo     A Rv64 "atomic AND, doubleword";
    0xf800707f 0x4000302f AmoorD   "amoor.d"   Amo     A Rv64 "atomic OR, doubleword";
    0xf800707f 0x8000302f AmominD  "amomin.d"  Amo     A Rv64 "atomic signed minimum, doubleword";
    0xf800707f 0xa000302f AmomaxD  "amomax.d"  Amo     A Rv64 "atomic signed maximum, doubleword";
    0xf800707f 0xc000302f AmominuD "amominu.d" Amo     A Rv64 "atomic unsigned minimum, doubleword";
    0xf800707f 0xe000302f AmomaxuD "amomaxu.d" Amo     A Rv64 "atomic unsigned maximum, doubleword";

    // -- OP -----------------------------------------------------------------
    0xfe00707f 0x00000033 Add    "add"    R I Any "add";
    0xfe00707f 0x40000033 Sub    "sub"    R I Any "subtract";
    0xfe00707f 0x00001033 Sll    "sll"    R I Any "shift left logical";
    0xfe00707f 0x00002033 Slt    "slt"    R I Any "set if less than, signed";
    0xfe00707f 0x00003033 Sltu   "sltu"   R I Any "set if less than, unsigned";
    0xfe00707f 0x00004033 Xor    "xor"    R I Any "exclusive-OR";
    0xfe00707f 0x00005033 Srl    "srl"    R I Any "shift right logical";
    0xfe00707f 0x40005033 Sra    "sra"    R I Any "shift right arithmetic";
    0xfe00707f 0x00006033 Or     "or"     R I Any "OR";
    0xfe00707f 0x00007033 And    "and"    R I Any "AND";
    0xfe00707f 0x02000033 Mul    "mul"    R M Any "multiply, low half";
    0xfe00707f 0x02001033 Mulh   "mulh"   R M Any "multiply high, signed by signed";
    0xfe00707f 0x02002033 Mulhsu "mulhsu" R M Any "multiply high, signed by unsigned";
    0xfe00707f 0x02003033 Mulhu  "mulhu"  R M Any "multiply high, unsigned by unsigned";
    0xfe00707f 0x02004033 Div    "div"    R M Any "divide, signed";
    0xfe00707f 0x02005033 Divu   "divu"   R M Any "divide, unsigned";
    0xfe00707f 0x02006033 Rem    "rem"    R M Any "remainder, signed";
    0xfe00707f 0x02007033 Remu   "remu"   R M Any "remainder, unsigned";

    // -- LUI ----------------------------------------------------------------
    0x0000007f 0x00000037 Lui   "lui"   U I Any "load an upper immediate";

    // -- OP-32 --------------------------------------------------------------
    0xfe00707f 0x0000003b Addw  "addw"  R I Rv64 "add words, sign-extended";
    0xfe00707f 0x4000003b Subw  "subw"  R I Rv64 "subtract words, sign-extended";
    0xfe00707f 0x0000103b Sllw  "sllw"  R I Rv64 "shift a word left logical";
    0xfe00707f 0x0000503b Srlw  "srlw"  R I Rv64 "shift a word right logical";
    0xfe00707f 0x4000503b Sraw  "sraw"  R I Rv64 "shift a word right arithmetic";
    0xfe00707f 0x0200003b Mulw  "mulw"  R M Rv64 "multiply words, sign-extended";
    0xfe00707f 0x0200403b Divw  "divw"  R M Rv64 "divide words, signed";
    0xfe00707f 0x0200503b Divuw "divuw" R M Rv64 "divide words, unsigned";
    0xfe00707f 0x0200603b Remw  "remw"  R M Rv64 "remainder of words, signed";
    0xfe00707f 0x0200703b Remuw "remuw" R M Rv64 "remainder of words, unsigned";

    // -- fused multiply-add -------------------------------------------------
    0x0600007f 0x00000043 FmaddS  "fmadd.s"  FpR4 F Any "fused multiply-add, single";
    0x0600007f 0x02000043 FmaddD  "fmadd.d"  FpR4 D Any "fused multiply-add, double";
    0x0600007f 0x00000047 FmsubS  "fmsub.s"  FpR4 F Any "fused multiply-subtract, single";
    0x0600007f 0x02000047 FmsubD  "fmsub.d"  FpR4 D Any "fused multiply-subtract, double";
    0x0600007f 0x0000004b FnmsubS "fnmsub.s" FpR4 F Any "negated fused multiply-subtract, single";
    0x0600007f 0x0200004b FnmsubD "fnmsub.d" FpR4 D Any "negated fused multiply-subtract, double";
    0x0600007f 0x0000004f FnmaddS "fnmadd.s" FpR4 F Any "negated fused multiply-add, single";
    0x0600007f 0x0200004f FnmaddD "fnmadd.d" FpR4 D Any "negated fused multiply-add, double";

    // -- OP-FP --------------------------------------------------------------
    0xfe00007f 0x00000053 FaddS  "fadd.s"  FpR F Any "add, single";
    0xfe00007f 0x08000053 FsubS  "fsub.s"  FpR F Any "subtract, single";
    0xfe00007f 0x10000053 FmulS  "fmul.s"  FpR F Any "multiply, single";
    0xfe00007f 0x18000053 FdivS  "fdiv.s"  FpR F Any "divide, single";
    0xfe00007f 0x02000053 FaddD  "fadd.d"  FpR D Any "add, double";
    0xfe00007f 0x0a000053 FsubD  "fsub.d"  FpR D Any "subtract, double";
    0xfe00007f 0x12000053 FmulD  "fmul.d"  FpR D Any "multiply, double";
    0xfe00007f 0x1a000053 FdivD  "fdiv.d"  FpR D Any "divide, double";
    0xfff0007f 0x58000053 FsqrtS "fsqrt.s" FpUnary F Any "square root, single";
    0xfff0007f 0x5a000053 FsqrtD "fsqrt.d" FpUnary D Any "square root, double";
    0xfe00707f 0x20000053 FsgnjS  "fsgnj.s"  FpR F Any "copy with the sign of the second operand, single";
    0xfe00707f 0x20001053 FsgnjnS "fsgnjn.s" FpR F Any "copy with the negated sign of the second operand, single";
    0xfe00707f 0x20002053 FsgnjxS "fsgnjx.s" FpR F Any "copy with the exclusive-OR of the signs, single";
    0xfe00707f 0x22000053 FsgnjD  "fsgnj.d"  FpR D Any "copy with the sign of the second operand, double";
    0xfe00707f 0x22001053 FsgnjnD "fsgnjn.d" FpR D Any "copy with the negated sign of the second operand, double";
    0xfe00707f 0x22002053 FsgnjxD "fsgnjx.d" FpR D Any "copy with the exclusive-OR of the signs, double";
    0xfe00707f 0x28000053 FminS "fmin.s" FpR F Any "minimum, single";
    0xfe00707f 0x28001053 FmaxS "fmax.s" FpR F Any "maximum, single";
    0xfe00707f 0x2a000053 FminD "fmin.d" FpR D Any "minimum, double";
    0xfe00707f 0x2a001053 FmaxD "fmax.d" FpR D Any "maximum, double";
    0xfe00707f 0xa0002053 FeqS  "feq.s"  FpCmp F Any "equal, single (quiet)";
    0xfe00707f 0xa0001053 FltS  "flt.s"  FpCmp F Any "less than, single (signaling)";
    0xfe00707f 0xa0000053 FleS  "fle.s"  FpCmp F Any "less than or equal, single (signaling)";
    0xfe00707f 0xa2002053 FeqD  "feq.d"  FpCmp D Any "equal, double (quiet)";
    0xfe00707f 0xa2001053 FltD  "flt.d"  FpCmp D Any "less than, double (signaling)";
    0xfe00707f 0xa2000053 FleD  "fle.d"  FpCmp D Any "less than or equal, double (signaling)";
    0xfff0007f 0xc0000053 FcvtWS  "fcvt.w.s"  FpToInt F Any  "convert single to a signed word";
    0xfff0007f 0xc0100053 FcvtWuS "fcvt.wu.s" FpToInt F Any  "convert single to an unsigned word";
    0xfff0007f 0xc0200053 FcvtLS  "fcvt.l.s"  FpToInt F Rv64 "convert single to a signed doubleword";
    0xfff0007f 0xc0300053 FcvtLuS "fcvt.lu.s" FpToInt F Rv64 "convert single to an unsigned doubleword";
    0xfff0007f 0xd0000053 FcvtSW  "fcvt.s.w"  FpFromInt F Any  "convert a signed word to single";
    0xfff0007f 0xd0100053 FcvtSWu "fcvt.s.wu" FpFromInt F Any  "convert an unsigned word to single";
    0xfff0007f 0xd0200053 FcvtSL  "fcvt.s.l"  FpFromInt F Rv64 "convert a signed doubleword to single";
    0xfff0007f 0xd0300053 FcvtSLu "fcvt.s.lu" FpFromInt F Rv64 "convert an unsigned doubleword to single";
    0xfff0007f 0xc2000053 FcvtWD  "fcvt.w.d"  FpToInt D Any  "convert double to a signed word";
    0xfff0007f 0xc2100053 FcvtWuD "fcvt.wu.d" FpToInt D Any  "convert double to an unsigned word";
    0xfff0007f 0xc2200053 FcvtLD  "fcvt.l.d"  FpToInt D Rv64 "convert double to a signed doubleword";
    0xfff0007f 0xc2300053 FcvtLuD "fcvt.lu.d" FpToInt D Rv64 "convert double to an unsigned doubleword";
    0xfff0007f 0xd2000053 FcvtDW  "fcvt.d.w"  FpFromInt D Any  "convert a signed word to double";
    0xfff0007f 0xd2100053 FcvtDWu "fcvt.d.wu" FpFromInt D Any  "convert an unsigned word to double";
    0xfff0007f 0xd2200053 FcvtDL  "fcvt.d.l"  FpFromInt D Rv64 "convert a signed doubleword to double";
    0xfff0007f 0xd2300053 FcvtDLu "fcvt.d.lu" FpFromInt D Rv64 "convert an unsigned doubleword to double";
    0xfff0007f 0x40100053 FcvtSD  "fcvt.s.d"  FpUnary D Any "convert double to single";
    0xfff0007f 0x42000053 FcvtDS  "fcvt.d.s"  FpUnary D Any "convert single to double";
    0xfff0707f 0xe0000053 FmvXW   "fmv.x.w"   FpToInt   F Any  "move the raw bits of a single to an integer register";
    0xfff0707f 0xe0001053 FclassS "fclass.s"  FpToInt   F Any  "classify a single";
    0xfff0707f 0xf0000053 FmvWX   "fmv.w.x"   FpFromInt F Any  "move raw bits from an integer register to a single";
    0xfff0707f 0xe2000053 FmvXD   "fmv.x.d"   FpToInt   D Rv64 "move the raw bits of a double to an integer register";
    0xfff0707f 0xe2001053 FclassD "fclass.d"  FpToInt   D Any  "classify a double";
    0xfff0707f 0xf2000053 FmvDX   "fmv.d.x"   FpFromInt D Rv64 "move raw bits from an integer register to a double";

    // -- BRANCH -------------------------------------------------------------
    0x0000707f 0x00000063 Beq  "beq"  Branch I Any "branch if equal";
    0x0000707f 0x00001063 Bne  "bne"  Branch I Any "branch if not equal";
    0x0000707f 0x00004063 Blt  "blt"  Branch I Any "branch if less than, signed";
    0x0000707f 0x00005063 Bge  "bge"  Branch I Any "branch if greater or equal, signed";
    0x0000707f 0x00006063 Bltu "bltu" Branch I Any "branch if less than, unsigned";
    0x0000707f 0x00007063 Bgeu "bgeu" Branch I Any "branch if greater or equal, unsigned";

    // -- JALR / JAL ---------------------------------------------------------
    0x0000707f 0x00000067 Jalr "jalr" Load I Any "jump and link register";
    0x0000007f 0x0000006f Jal  "jal"  Jump I Any "jump and link";

    // -- SYSTEM -------------------------------------------------------------
    0xffffffff 0x00000073 Ecall     "ecall"      None   Priv Any "call the supporting execution environment";
    0xffffffff 0x00100073 Ebreak    "ebreak"     None   Priv Any "return control to the debugger";
    0xffffffff 0x10200073 Sret      "sret"       None   Priv Any "return from a supervisor trap";
    0xffffffff 0x30200073 Mret      "mret"       None   Priv Any "return from a machine trap";
    0xffffffff 0x10500073 Wfi       "wfi"        None   Priv Any "wait for an interrupt";
    0xfe007fff 0x12000073 SfenceVma "sfence.vma" Sfence Priv Any "order address-translation structure updates";
    0x0000707f 0x00001073 Csrrw  "csrrw"  Csr    Zicsr Any "read a CSR and write a register into it";
    0x0000707f 0x00002073 Csrrs  "csrrs"  Csr    Zicsr Any "read a CSR and set the bits a register names";
    0x0000707f 0x00003073 Csrrc  "csrrc"  Csr    Zicsr Any "read a CSR and clear the bits a register names";
    0x0000707f 0x00005073 Csrrwi "csrrwi" CsrImm Zicsr Any "read a CSR and write an immediate into it";
    0x0000707f 0x00006073 Csrrsi "csrrsi" CsrImm Zicsr Any "read a CSR and set the bits an immediate names";
    0x0000707f 0x00007073 Csrrci "csrrci" CsrImm Zicsr Any "read a CSR and clear the bits an immediate names";
}

/// Where each seven-bit opcode's rows start and end in [`TABLE`].
///
/// Built from `TABLE` by a `const fn`, so it is a derived cache in the strict
/// sense — it cannot disagree with the table it indexes, and adding a row
/// needs no second edit.
pub static INDEX: [(u16, u16); 128] = build_index(TABLE);

/// Compute [`INDEX`] at compile time.
const fn build_index(table: &[Insn]) -> [(u16, u16); 128] {
    let mut index = [(0u16, 0u16); 128];
    let mut opcode = 0usize;
    while opcode < 128 {
        let mut first = 0u16;
        let mut last = 0u16;
        let mut found = false;
        let mut i = 0usize;
        while i < table.len() {
            if (table[i].bits & 0x7f) as usize == opcode {
                if !found {
                    first = i as u16;
                    found = true;
                }
                last = i as u16 + 1;
            }
            i += 1;
        }
        index[opcode] = if found { (first, last) } else { (0, 0) };
        opcode += 1;
    }
    index
}

/// Decode a 32-bit instruction word, or `None` if nothing matches.
///
/// Rows whose [`Width`] excludes `xlen` are skipped here rather than rejected
/// later, because on RV32 those encodings are simply not instructions.
#[must_use]
pub fn decode(word: u32, xlen: Xlen) -> Option<&'static Insn> {
    let (first, last) = INDEX[(word & 0x7f) as usize];
    let mut i = first as usize;
    while i < last as usize {
        let insn = &TABLE[i];
        if word & insn.mask == insn.bits && insn.width.allows(xlen) {
            return Some(insn);
        }
        i += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// Field extraction
// ---------------------------------------------------------------------------

/// The destination register field.
#[inline]
#[must_use]
pub const fn rd(word: u32) -> u32 {
    (word >> 7) & 31
}

/// The first source register field.
#[inline]
#[must_use]
pub const fn rs1(word: u32) -> u32 {
    (word >> 15) & 31
}

/// The second source register field.
#[inline]
#[must_use]
pub const fn rs2(word: u32) -> u32 {
    (word >> 20) & 31
}

/// The third source register field, used only by the fused multiply-adds.
#[inline]
#[must_use]
pub const fn rs3(word: u32) -> u32 {
    (word >> 27) & 31
}

/// The `funct3` field, which doubles as the rounding mode on FP instructions.
#[inline]
#[must_use]
pub const fn funct3(word: u32) -> u32 {
    (word >> 12) & 7
}

/// The shift amount: six bits, of which RV32 may only use five.
#[inline]
#[must_use]
pub const fn shamt(word: u32) -> u32 {
    (word >> 20) & 63
}

/// The CSR number.
#[inline]
#[must_use]
pub const fn csr(word: u32) -> u32 {
    word >> 20
}

/// The `aq` (acquire) ordering bit of an atomic.
#[inline]
#[must_use]
pub const fn aq(word: u32) -> bool {
    word & (1 << 26) != 0
}

/// The `rl` (release) ordering bit of an atomic.
#[inline]
#[must_use]
pub const fn rl(word: u32) -> bool {
    word & (1 << 25) != 0
}

/// The I-type immediate, sign-extended.
#[inline]
#[must_use]
pub const fn imm_i(word: u32) -> i64 {
    (word as i32 as i64) >> 20
}

/// The S-type immediate, sign-extended.
#[inline]
#[must_use]
pub const fn imm_s(word: u32) -> i64 {
    (((word & 0xfe00_0000) as i32 as i64) >> 20) | ((word >> 7) & 0x1f) as i64
}

/// The B-type immediate, sign-extended. Always even.
#[inline]
#[must_use]
pub const fn imm_b(word: u32) -> i64 {
    let v = (((word >> 31) & 1) << 12)
        | (((word >> 25) & 0x3f) << 5)
        | (((word >> 8) & 0xf) << 1)
        | (((word >> 7) & 1) << 11);
    (((v << 19) as i32) >> 19) as i64
}

/// The U-type immediate: the top 20 bits, sign-extended from bit 31.
#[inline]
#[must_use]
pub const fn imm_u(word: u32) -> i64 {
    (word & 0xffff_f000) as i32 as i64
}

/// The J-type immediate, sign-extended. Always even.
#[inline]
#[must_use]
pub const fn imm_j(word: u32) -> i64 {
    let v = (((word >> 31) & 1) << 20)
        | (((word >> 21) & 0x3ff) << 1)
        | (((word >> 20) & 1) << 11)
        | (((word >> 12) & 0xff) << 12);
    (((v << 11) as i32) >> 11) as i64
}

/// Whether a halfword begins a 32-bit instruction rather than a compressed
/// one.
///
/// Volume I: an instruction is 16 bits wide unless its two lowest bits are
/// both set. Longer encodings exist in the specification but no ratified
/// extension uses them, so anything else is an illegal instruction here.
#[inline]
#[must_use]
pub const fn is_32bit(half: u16) -> bool {
    half & 3 == 3
}

// ---------------------------------------------------------------------------
// The C extension
// ---------------------------------------------------------------------------

/// One row of the compressed instruction description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CInsn {
    /// Which compressed operation this is.
    pub op: COp,
    /// The bits of the 16-bit encoding that are fixed.
    pub mask: u16,
    /// What those bits must be.
    pub bits: u16,
    /// Which register widths it exists on.
    pub width: Width,
}

/// Declare the compressed operation enum and its table from one list of rows.
macro_rules! rvc {
    ($($mask:literal $bits:literal $op:ident $mn:literal $width:ident $summary:literal;)*) => {
        /// One compressed operation.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum COp {
            $(
                #[doc = $summary]
                $op,
            )*
        }

        impl COp {
            /// The assembler mnemonic.
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(COp::$op => $mn,)* }
            }

            /// A one-line description.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(COp::$op => $summary,)* }
            }

            /// Every compressed operation, in encoding order.
            pub const ALL: &'static [COp] = &[$(COp::$op,)*];
        }

        /// The compressed decode table.
        ///
        /// Order is load-bearing where two rows overlap: `c.addi16sp` is
        /// `c.lui` with `rd = x2`, and `c.jr`/`c.jalr` are `c.mv`/`c.add` with
        /// `rs2 = x0`, so the specific row is written first and [`decode_compressed`]
        /// takes the first match.
        pub static CTABLE: &[CInsn] = &[
            $(CInsn { op: COp::$op, mask: $mask, bits: $bits, width: Width::$width },)*
        ];
    };
}

rvc! {
    // -- quadrant 0 ---------------------------------------------------------
    0xe003 0x0000 CAddi4spn "c.addi4spn" Any  "add a scaled immediate to the stack pointer";
    0xe003 0x2000 CFld      "c.fld"      Any  "load a double";
    0xe003 0x4000 CLw       "c.lw"       Any  "load a word";
    0xe003 0x6000 CFlw      "c.flw"      Rv32 "load a single";
    0xe003 0x6000 CLd       "c.ld"       Rv64 "load a doubleword";
    0xe003 0xa000 CFsd      "c.fsd"      Any  "store a double";
    0xe003 0xc000 CSw       "c.sw"       Any  "store a word";
    0xe003 0xe000 CFsw      "c.fsw"      Rv32 "store a single";
    0xe003 0xe000 CSd       "c.sd"       Rv64 "store a doubleword";

    // -- quadrant 1 ---------------------------------------------------------
    0xe003 0x0001 CAddi     "c.addi"     Any  "add an immediate in place";
    0xe003 0x2001 CJal      "c.jal"      Rv32 "jump and link to x1";
    0xe003 0x2001 CAddiw    "c.addiw"    Rv64 "add an immediate to a word in place";
    0xe003 0x4001 CLi       "c.li"       Any  "load an immediate";
    0xef83 0x6101 CAddi16sp "c.addi16sp" Any  "add a scaled immediate to the stack pointer in place";
    0xe003 0x6001 CLui      "c.lui"      Any  "load an upper immediate";
    0xec03 0x8001 CSrli     "c.srli"     Any  "shift right logical in place";
    0xec03 0x8401 CSrai     "c.srai"     Any  "shift right arithmetic in place";
    0xec03 0x8801 CAndi     "c.andi"     Any  "AND with an immediate in place";
    0xfc63 0x8c01 CSub      "c.sub"      Any  "subtract in place";
    0xfc63 0x8c21 CXor      "c.xor"      Any  "exclusive-OR in place";
    0xfc63 0x8c41 COr       "c.or"       Any  "OR in place";
    0xfc63 0x8c61 CAnd      "c.and"      Any  "AND in place";
    0xfc63 0x9c01 CSubw     "c.subw"     Rv64 "subtract words in place";
    0xfc63 0x9c21 CAddw     "c.addw"     Rv64 "add words in place";
    0xe003 0xa001 CJ        "c.j"        Any  "jump";
    0xe003 0xc001 CBeqz     "c.beqz"     Any  "branch if zero";
    0xe003 0xe001 CBnez     "c.bnez"     Any  "branch if not zero";

    // -- quadrant 2 ---------------------------------------------------------
    0xe003 0x0002 CSlli     "c.slli"     Any  "shift left logical in place";
    0xe003 0x2002 CFldsp    "c.fldsp"    Any  "load a double from the stack";
    0xe003 0x4002 CLwsp     "c.lwsp"     Any  "load a word from the stack";
    0xe003 0x6002 CFlwsp    "c.flwsp"    Rv32 "load a single from the stack";
    0xe003 0x6002 CLdsp     "c.ldsp"     Rv64 "load a doubleword from the stack";
    0xf07f 0x8002 CJr       "c.jr"       Any  "jump to a register";
    0xf003 0x8002 CMv       "c.mv"       Any  "copy a register";
    0xffff 0x9002 CEbreak   "c.ebreak"   Any  "return control to the debugger";
    0xf07f 0x9002 CJalr     "c.jalr"     Any  "jump to a register and link to x1";
    0xf003 0x9002 CAdd      "c.add"      Any  "add in place";
    0xe003 0xa002 CFsdsp    "c.fsdsp"    Any  "store a double to the stack";
    0xe003 0xc002 CSwsp     "c.swsp"     Any  "store a word to the stack";
    0xe003 0xe002 CFswsp    "c.fswsp"    Rv32 "store a single to the stack";
    0xe003 0xe002 CSdsp     "c.sdsp"     Rv64 "store a doubleword to the stack";
}

/// Decode a compressed instruction, or `None` if nothing matches.
#[must_use]
pub fn decode_compressed(half: u16, xlen: Xlen) -> Option<&'static CInsn> {
    // The all-zero halfword is defined as permanently illegal, which is what
    // makes a run into zeroed memory stop rather than execute.
    if half == 0 {
        return None;
    }
    CTABLE
        .iter()
        .find(|c| half & c.mask == c.bits && c.width.allows(xlen))
}

/// A three-bit compressed register field, which names `x8`..`x15`.
#[inline]
const fn creg(field: u16) -> u32 {
    (field & 7) as u32 + 8
}

/// The `rd'`/`rs1'` field at bits 9:7.
#[inline]
const fn crs1(half: u16) -> u32 {
    creg(half >> 7)
}

/// The `rd'`/`rs2'` field at bits 4:2.
#[inline]
const fn crs2(half: u16) -> u32 {
    creg(half >> 2)
}

/// The full five-bit register field at bits 11:7.
#[inline]
const fn cwide_rd(half: u16) -> u32 {
    ((half >> 7) & 31) as u32
}

/// The full five-bit register field at bits 6:2.
#[inline]
const fn cwide_rs2(half: u16) -> u32 {
    ((half >> 2) & 31) as u32
}

/// Sign-extend `value` from bit `bit`.
#[inline]
const fn sext(value: u32, bit: u32) -> i64 {
    let shift = 63 - bit;
    (((value as u64) << shift) as i64) >> shift
}

/// Reassemble an I-type instruction.
const fn make_i(opcode: u32, rd: u32, funct3: u32, rs1: u32, imm: i64) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (((imm as u64) as u32 & 0xfff) << 20)
}

/// Reassemble an S-type instruction.
const fn make_s(opcode: u32, funct3: u32, rs1: u32, rs2: u32, imm: i64) -> u32 {
    let imm = (imm as u64) as u32;
    opcode
        | ((imm & 0x1f) << 7)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x7f) << 25)
}

/// Reassemble an R-type instruction.
const fn make_r(opcode: u32, rd: u32, funct3: u32, rs1: u32, rs2: u32, funct7: u32) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25)
}

/// Reassemble a B-type instruction.
const fn make_b(opcode: u32, funct3: u32, rs1: u32, rs2: u32, imm: i64) -> u32 {
    let imm = (imm as u64) as u32;
    opcode
        | (((imm >> 11) & 1) << 7)
        | (((imm >> 1) & 0xf) << 8)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x3f) << 25)
        | (((imm >> 12) & 1) << 31)
}

/// Reassemble a J-type instruction.
const fn make_j(opcode: u32, rd: u32, imm: i64) -> u32 {
    let imm = (imm as u64) as u32;
    opcode
        | (rd << 7)
        | (((imm >> 12) & 0xff) << 12)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 20) & 1) << 31)
}

/// Reassemble a U-type instruction.
const fn make_u(opcode: u32, rd: u32, imm: i64) -> u32 {
    opcode | (rd << 7) | (((imm as u64) as u32) & 0xffff_f000)
}

/// Expand a compressed instruction into the 32-bit instruction it aliases.
///
/// Volume I, "Compressed Instruction Formats": *every* RVC instruction is
/// defined as expanding to exactly one base instruction, so this function is
/// the complete semantics of the `C` extension. The interpreter therefore has
/// no compressed-instruction code path at all, which is one fewer place for
/// the two to disagree.
///
/// `None` means the encoding is reserved — `c.addi4spn` with a zero immediate,
/// `c.lui` with a zero immediate, `c.jr` with `rs1 = x0`, `c.addiw` with
/// `rd = x0`, a stack load into `x0`, or a shift by 32 or more on RV32 — all of
/// which raise an illegal-instruction exception.
#[must_use]
pub fn expand(half: u16, xlen: Xlen) -> Option<u32> {
    let insn = decode_compressed(half, xlen)?;
    let h = half as u32;
    // Immediate scrambles, transcribed from the RVC format figures. Each one
    // is named for the instruction group that uses it.
    let bit = |n: u32| (h >> n) & 1;
    let bits = |hi: u32, lo: u32| (h >> lo) & ((1 << (hi - lo + 1)) - 1);

    let word = match insn.op {
        COp::CAddi4spn => {
            let imm = (bits(12, 11) << 4) | (bits(10, 7) << 6) | (bit(6) << 2) | (bit(5) << 3);
            if imm == 0 {
                return None;
            }
            make_i(0x13, crs2(half), 0, 2, i64::from(imm))
        }
        COp::CFld => {
            let imm = (bits(12, 10) << 3) | (bits(6, 5) << 6);
            make_i(0x07, crs2(half), 3, crs1(half), i64::from(imm))
        }
        COp::CLw => {
            let imm = (bits(12, 10) << 3) | (bit(6) << 2) | (bit(5) << 6);
            make_i(0x03, crs2(half), 2, crs1(half), i64::from(imm))
        }
        COp::CFlw => {
            let imm = (bits(12, 10) << 3) | (bit(6) << 2) | (bit(5) << 6);
            make_i(0x07, crs2(half), 2, crs1(half), i64::from(imm))
        }
        COp::CLd => {
            let imm = (bits(12, 10) << 3) | (bits(6, 5) << 6);
            make_i(0x03, crs2(half), 3, crs1(half), i64::from(imm))
        }
        COp::CFsd => {
            let imm = (bits(12, 10) << 3) | (bits(6, 5) << 6);
            make_s(0x27, 3, crs1(half), crs2(half), i64::from(imm))
        }
        COp::CSw => {
            let imm = (bits(12, 10) << 3) | (bit(6) << 2) | (bit(5) << 6);
            make_s(0x23, 2, crs1(half), crs2(half), i64::from(imm))
        }
        COp::CFsw => {
            let imm = (bits(12, 10) << 3) | (bit(6) << 2) | (bit(5) << 6);
            make_s(0x27, 2, crs1(half), crs2(half), i64::from(imm))
        }
        COp::CSd => {
            let imm = (bits(12, 10) << 3) | (bits(6, 5) << 6);
            make_s(0x23, 3, crs1(half), crs2(half), i64::from(imm))
        }
        COp::CAddi => {
            let imm = sext((bit(12) << 5) | bits(6, 2), 5);
            let rd = cwide_rd(half);
            make_i(0x13, rd, 0, rd, imm)
        }
        COp::CJal => make_j(0x6f, 1, cj_offset(h)),
        COp::CAddiw => {
            let rd = cwide_rd(half);
            if rd == 0 {
                return None;
            }
            let imm = sext((bit(12) << 5) | bits(6, 2), 5);
            make_i(0x1b, rd, 0, rd, imm)
        }
        COp::CLi => {
            let imm = sext((bit(12) << 5) | bits(6, 2), 5);
            make_i(0x13, cwide_rd(half), 0, 0, imm)
        }
        COp::CAddi16sp => {
            let imm = sext(
                (bit(12) << 9) | (bit(6) << 4) | (bit(5) << 6) | (bits(4, 3) << 7) | (bit(2) << 5),
                9,
            );
            if imm == 0 {
                return None;
            }
            make_i(0x13, 2, 0, 2, imm)
        }
        COp::CLui => {
            let imm = sext((bit(12) << 17) | (bits(6, 2) << 12), 17);
            if imm == 0 {
                return None;
            }
            make_u(0x37, cwide_rd(half), imm)
        }
        COp::CSrli | COp::CSrai => {
            let shamt = (bit(12) << 5) | bits(6, 2);
            if xlen == Xlen::Rv32 && shamt >= 32 {
                return None;
            }
            let rd = crs1(half);
            // SRAI is funct6 = 010000, which is bits 31:26 — so the funct7
            // field the R-type builder takes is 0100000, and shamt[5] is the
            // bit below it.
            let funct7 = if insn.op == COp::CSrai { 0x20 } else { 0 };
            make_r(0x13, rd, 5, rd, shamt & 31, funct7 | (shamt >> 5))
        }
        COp::CAndi => {
            let imm = sext((bit(12) << 5) | bits(6, 2), 5);
            let rd = crs1(half);
            make_i(0x13, rd, 7, rd, imm)
        }
        COp::CSub => make_r(0x33, crs1(half), 0, crs1(half), crs2(half), 0x20),
        COp::CXor => make_r(0x33, crs1(half), 4, crs1(half), crs2(half), 0),
        COp::COr => make_r(0x33, crs1(half), 6, crs1(half), crs2(half), 0),
        COp::CAnd => make_r(0x33, crs1(half), 7, crs1(half), crs2(half), 0),
        COp::CSubw => make_r(0x3b, crs1(half), 0, crs1(half), crs2(half), 0x20),
        COp::CAddw => make_r(0x3b, crs1(half), 0, crs1(half), crs2(half), 0),
        COp::CJ => make_j(0x6f, 0, cj_offset(h)),
        COp::CBeqz | COp::CBnez => {
            let imm = sext(
                (bit(12) << 8)
                    | (bits(11, 10) << 3)
                    | (bits(6, 5) << 6)
                    | (bits(4, 3) << 1)
                    | (bit(2) << 5),
                8,
            );
            let funct3 = if insn.op == COp::CBeqz { 0 } else { 1 };
            make_b(0x63, funct3, crs1(half), 0, imm)
        }
        COp::CSlli => {
            let shamt = (bit(12) << 5) | bits(6, 2);
            if xlen == Xlen::Rv32 && shamt >= 32 {
                return None;
            }
            let rd = cwide_rd(half);
            make_i(0x13, rd, 1, rd, i64::from(shamt))
        }
        COp::CFldsp => {
            let imm = (bit(12) << 5) | (bits(6, 5) << 3) | (bits(4, 2) << 6);
            make_i(0x07, cwide_rd(half), 3, 2, i64::from(imm))
        }
        COp::CLwsp => {
            let rd = cwide_rd(half);
            if rd == 0 {
                return None;
            }
            let imm = (bit(12) << 5) | (bits(6, 4) << 2) | (bits(3, 2) << 6);
            make_i(0x03, rd, 2, 2, i64::from(imm))
        }
        COp::CFlwsp => {
            let imm = (bit(12) << 5) | (bits(6, 4) << 2) | (bits(3, 2) << 6);
            make_i(0x07, cwide_rd(half), 2, 2, i64::from(imm))
        }
        COp::CLdsp => {
            let rd = cwide_rd(half);
            if rd == 0 {
                return None;
            }
            let imm = (bit(12) << 5) | (bits(6, 5) << 3) | (bits(4, 2) << 6);
            make_i(0x03, rd, 3, 2, i64::from(imm))
        }
        COp::CJr => {
            let rs1 = cwide_rd(half);
            if rs1 == 0 {
                return None;
            }
            make_i(0x67, 0, 0, rs1, 0)
        }
        COp::CMv => make_r(0x33, cwide_rd(half), 0, 0, cwide_rs2(half), 0),
        COp::CEbreak => 0x0010_0073,
        COp::CJalr => make_i(0x67, 1, 0, cwide_rd(half), 0),
        COp::CAdd => {
            let rd = cwide_rd(half);
            make_r(0x33, rd, 0, rd, cwide_rs2(half), 0)
        }
        COp::CFsdsp => {
            let imm = (bits(12, 10) << 3) | (bits(9, 7) << 6);
            make_s(0x27, 3, 2, cwide_rs2(half), i64::from(imm))
        }
        COp::CSwsp => {
            let imm = (bits(12, 9) << 2) | (bits(8, 7) << 6);
            make_s(0x23, 2, 2, cwide_rs2(half), i64::from(imm))
        }
        COp::CFswsp => {
            let imm = (bits(12, 9) << 2) | (bits(8, 7) << 6);
            make_s(0x27, 2, 2, cwide_rs2(half), i64::from(imm))
        }
        COp::CSdsp => {
            let imm = (bits(12, 10) << 3) | (bits(9, 7) << 6);
            make_s(0x23, 3, 2, cwide_rs2(half), i64::from(imm))
        }
    };
    Some(word)
}

/// The `CJ` format's scrambled jump offset.
const fn cj_offset(h: u32) -> i64 {
    let v = (((h >> 12) & 1) << 11)
        | (((h >> 11) & 1) << 4)
        | (((h >> 9) & 3) << 8)
        | (((h >> 8) & 1) << 10)
        | (((h >> 7) & 1) << 6)
        | (((h >> 6) & 1) << 7)
        | (((h >> 3) & 7) << 1)
        | (((h >> 2) & 1) << 5);
    sext(v, 11)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn the_index_covers_every_row() {
        // Every row must be reachable through its opcode bucket, or an
        // instruction silently stops decoding.
        for (i, insn) in TABLE.iter().enumerate() {
            let (first, last) = INDEX[(insn.bits & 0x7f) as usize];
            assert!(
                (first as usize..last as usize).contains(&i),
                "{} is outside its bucket",
                insn.op.mnemonic()
            );
        }
    }

    #[test]
    fn every_row_fixes_its_opcode() {
        for insn in TABLE {
            assert_eq!(
                insn.mask & 0x7f,
                0x7f,
                "{} does not fix its opcode field",
                insn.op.mnemonic()
            );
            assert_eq!(
                insn.bits & !insn.mask,
                0,
                "{} has match bits outside its mask",
                insn.op.mnemonic()
            );
        }
    }

    #[test]
    fn no_two_rows_are_ambiguous_at_the_same_width() {
        // Two rows may share bits only if the first one listed is strictly
        // more specific — that is the ordering `decode` relies on.
        for (i, a) in TABLE.iter().enumerate() {
            for b in &TABLE[i + 1..] {
                let common = a.mask & b.mask;
                if a.bits & common != b.bits & common {
                    continue;
                }
                let overlap_width = matches!((a.width, b.width), (Width::Any, _) | (_, Width::Any))
                    || a.width == b.width;
                assert!(
                    !overlap_width || a.mask & !b.mask != 0,
                    "{} and {} are ambiguous",
                    a.op.mnemonic(),
                    b.op.mnemonic()
                );
            }
        }
    }

    #[test]
    fn mnemonics_are_unique() {
        let mut seen: Vec<&str> = Op::ALL.iter().map(|o| o.mnemonic()).collect();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(before, seen.len(), "a mnemonic is used twice");
    }

    #[test]
    fn decodes_the_canonical_encodings() {
        // Hand-assembled from the Volume I format figures.
        let add = 0x00c5_8533; // add a0, a1, a2
        let insn = decode(add, Xlen::Rv64).unwrap();
        assert_eq!(insn.op, Op::Add);
        assert_eq!(rd(add), 10);
        assert_eq!(rs1(add), 11);
        assert_eq!(rs2(add), 12);

        let addi = 0xffb5_0513; // addi a0, a0, -5
        assert_eq!(decode(addi, Xlen::Rv64).unwrap().op, Op::Addi);
        assert_eq!(imm_i(addi), -5);

        let lw = 0x0080_a503; // lw a0, 8(ra)
        assert_eq!(decode(lw, Xlen::Rv64).unwrap().op, Op::Lw);
        assert_eq!(imm_i(lw), 8);

        let sw = 0x00b1_2423; // sw a1, 8(sp)
        assert_eq!(decode(sw, Xlen::Rv64).unwrap().op, Op::Sw);
        assert_eq!(imm_s(sw), 8);

        assert_eq!(decode(0x0000_0073, Xlen::Rv64).unwrap().op, Op::Ecall);
        assert_eq!(decode(0x0010_0073, Xlen::Rv64).unwrap().op, Op::Ebreak);
        assert_eq!(decode(0x3020_0073, Xlen::Rv64).unwrap().op, Op::Mret);
        assert_eq!(decode(0x1020_0073, Xlen::Rv64).unwrap().op, Op::Sret);
        assert_eq!(decode(0x1050_0073, Xlen::Rv64).unwrap().op, Op::Wfi);
        assert_eq!(decode(0x1200_0073, Xlen::Rv64).unwrap().op, Op::SfenceVma);
        // A CSR access and a SYSTEM instruction share an opcode.
        assert_eq!(decode(0x3400_2573, Xlen::Rv64).unwrap().op, Op::Csrrs);
        assert_eq!(csr(0x3400_2573), 0x340);
    }

    #[test]
    fn the_word_instructions_are_rv64_only() {
        assert_eq!(decode(0x0000_001b, Xlen::Rv64).unwrap().op, Op::Addiw);
        assert!(decode(0x0000_001b, Xlen::Rv32).is_none());
        assert_eq!(decode(0x0000_3003, Xlen::Rv64).unwrap().op, Op::Ld);
        assert!(decode(0x0000_3003, Xlen::Rv32).is_none());
    }

    #[test]
    fn branch_and_jump_immediates_are_signed_and_even() {
        // beq a0, a1, -4  => the offset scramble must round-trip.
        let word = make_b(0x63, 0, 10, 11, -4);
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Beq);
        assert_eq!(imm_b(word), -4);
        for offset in [-4096, -2, 0, 2, 4094] {
            assert_eq!(imm_b(make_b(0x63, 0, 1, 2, offset)), offset);
        }
        for offset in [-1_048_576, -2, 0, 2, 1_048_574] {
            assert_eq!(imm_j(make_j(0x6f, 1, offset)), offset);
        }
    }

    #[test]
    fn compressed_expansions_match_their_base_instructions() {
        // c.addi a0, 1  ->  addi a0, a0, 1
        let word = expand(0x0505, Xlen::Rv64).unwrap();
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Addi);
        assert_eq!(rd(word), 10);
        assert_eq!(rs1(word), 10);
        assert_eq!(imm_i(word), 1);

        // c.nop
        assert_eq!(expand(0x0001, Xlen::Rv64).unwrap(), 0x0000_0013);
        // c.ebreak
        assert_eq!(expand(0x9002, Xlen::Rv64).unwrap(), 0x0010_0073);
        // c.jr ra  ->  jalr x0, 0(ra)
        let word = expand(0x8082, Xlen::Rv64).unwrap();
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Jalr);
        assert_eq!(rd(word), 0);
        assert_eq!(rs1(word), 1);
        // c.mv a0, a1  ->  add a0, x0, a1
        let word = expand(0x852e, Xlen::Rv64).unwrap();
        assert_eq!(rd(word), 10);
        assert_eq!(rs1(word), 0);
        assert_eq!(rs2(word), 11);
    }

    #[test]
    fn compressed_shifts_expand_to_the_right_funct6() {
        // c.srai a1, 7 — the encoding that catches a funct6/funct7 mix-up,
        // because c.srli with the same shamt is a valid instruction too.
        let word = expand(0x859d, Xlen::Rv64).unwrap();
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Srai);
        assert_eq!(rd(word), 11);
        assert_eq!(rs1(word), 11);
        assert_eq!(shamt(word), 7);
        // c.srli a1, 7
        let word = expand(0x819d, Xlen::Rv64).unwrap();
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Srli);
        assert_eq!(shamt(word), 7);
        // A shift of 32 or more sets shamt[5], which lives inside funct7.
        let word = expand(0x9081, Xlen::Rv64).unwrap();
        assert_eq!(decode(word, Xlen::Rv64).unwrap().op, Op::Srli);
        assert_eq!(shamt(word), 32);
    }

    #[test]
    fn reserved_compressed_encodings_are_rejected() {
        assert!(expand(0x0000, Xlen::Rv64).is_none(), "the zero halfword");
        assert!(expand(0x8002, Xlen::Rv64).is_none(), "c.jr x0");
        // c.addi4spn with a zero immediate.
        assert!(expand(0x0008, Xlen::Rv64).is_none());
        // c.lui with a zero immediate: rd = a0, imm = 0.
        assert!(expand(0x6501 & !0x107c, Xlen::Rv64).is_none());
    }

    #[test]
    fn the_stack_pointer_forms_scale_their_offsets() {
        // c.addi4spn a0, sp, 8  (nzuimm[3] is bit 5 of the encoding)
        let word = expand(0x0028, Xlen::Rv64).unwrap();
        assert_eq!(rd(word), 10);
        assert_eq!(rs1(word), 2);
        assert_eq!(imm_i(word), 8);
        // The same field one bit over is nzuimm[2] = 4.
        assert_eq!(imm_i(expand(0x0048, Xlen::Rv64).unwrap()), 4);
        // c.addi16sp 16
        let word = expand(0x6141, Xlen::Rv64).unwrap();
        assert_eq!(rd(word), 2);
        assert_eq!(rs1(word), 2);
        assert_eq!(imm_i(word), 16);
    }

    #[test]
    fn quadrant_zero_and_two_disagree_by_width() {
        // 0x6000-family: c.flw on RV32, c.ld on RV64.
        assert_eq!(decode_compressed(0x6108, Xlen::Rv32).unwrap().op, COp::CFlw);
        assert_eq!(decode_compressed(0x6108, Xlen::Rv64).unwrap().op, COp::CLd);
    }

    #[test]
    fn thirty_two_bit_encodings_are_recognised_by_their_low_bits() {
        assert!(is_32bit(0x0033));
        assert!(!is_32bit(0x0001));
        assert!(!is_32bit(0x4000));
    }
}
