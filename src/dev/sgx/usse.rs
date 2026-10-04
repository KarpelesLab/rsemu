//! The USSE2 — the PowerVR SGX543's universal shader engine: instruction
//! decoder, disassembler and interpreter.
//!
//! The SGX543 runs everything programmable on this engine: vertex and pixel
//! shaders, and the GPU's own firmware, the *microkernel*, which is what this
//! module is first meant to run. One instruction is one 64-bit word, stored
//! little-endian as two `u32` — the lower address holds bits 31:0. Bit
//! numbers below are of that 64-bit value. Program counters count
//! instructions (8 bytes), relative to a code base.
//!
//! # Provenance
//!
//! No USSE documentation is public, and none was consulted: no DDK source, no
//! Mesa, no QEMU, no community reverse-engineering. Every field position here
//! was **measured from the guest's own `useasm` encoder, run as a black box**:
//! the encoder entry in the navi's `libGLES_CM.so` was executed in a small ARM
//! interpreter, every input field of every opcode was flipped, and the change
//! in the 64-bit output recorded. Our notes on that measurement (`ENCODING.md`,
//! cited below by section, `§n`) grade every claim:
//!
//! * **exact** — a field position or encoding read off the encoder;
//! * **strong** — consistent across encoder, firmware and the shader
//!   compiler's tables;
//! * **guess** — everything else. Each guess in this file says so in a
//!   comment marked `[G]`, so it can be found and revisited.
//!
//! Names are ours. Where the measurement gave a field but no meaning, the
//! mnemonic carries the encoder's flag number (`f1.28`) rather than an
//! invented name, so that the disassembly matches our reference disassembler
//! and nothing reads as more certain than it is.
//!
//! # Shape
//!
//! `FORMS`, a private static table, is the one
//! declarative description of every instruction form: match bits, mnemonic,
//! predicate and repeat fields, printed flags, operand encodings, and the
//! execution class. [`decode`] walks it to build an [`Insn`]; [`Disasm`]
//! renders an `Insn`; [`Usse::step`] executes one. Nothing about an encoding
//! is written twice.
//!
//! # The interpreter
//!
//! [`Usse`] is one instance's architectural state: the register banks,
//! predicates, link register and PC. Memory and the SGX register file are the
//! caller's, reached through [`UsseBus`] with GPU virtual addresses (the
//! caller translates through the SGX MMU). Execution stops with a [`Stop`]
//! reason — end of program, end of phase, an unimplemented or undecodable
//! instruction, a bus fault, or an exhausted budget. Where a semantic is not
//! known well enough to guess, the interpreter stops with
//! [`Stop::Unimplemented`] rather than inventing behaviour.
//!
//! Loads complete synchronously, so the data-return counters (`DRC`) and the
//! fences that wait on them (`IDF`, `WDF`) have nothing to wait for.

use alloc::string::String;
use core::fmt::{self, Write as _};

use crate::core::error::BusError;
use crate::core::space::MemResult;

// ---------------------------------------------------------------------------
// Bit helpers

/// Bit `n` of a word, as a mask.
const fn b(n: u32) -> u64 {
    1u64 << n
}

/// `width` bits of `w` starting at `lo`.
#[inline]
const fn field(w: u64, lo: u32, width: u32) -> u64 {
    (w >> lo) & ((1u64 << width) - 1)
}

/// Bit `n` of `w`, as a `bool`.
#[inline]
const fn bit(w: u64, n: u32) -> bool {
    (w >> n) & 1 != 0
}

/// Sign-extends a 7-bit field.
const fn sext7(v: u64) -> i64 {
    if v & 0x40 != 0 {
        v as i64 - 0x80
    } else {
        v as i64
    }
}

/// The primary opcode, bits 63:59 (§1).
const OPCODE: u64 = 0x1f << 59;

/// The match bits of a primary opcode.
const fn op(hw: u64) -> u64 {
    hw << 59
}

// ---------------------------------------------------------------------------
// Operands

/// A register bank (§2.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Bank {
    /// Per-instance temporaries, `r`.
    Temp,
    /// Outputs, `o`.
    Output,
    /// Primary attributes, `pa`.
    Primary,
    /// Secondary attributes, `sa` — shared by every instance of a program.
    Secondary,
    /// The special bank: numbers below 64 are the constant table `c`, the
    /// rest special/global registers `g`.
    Special,
    /// The index registers, `i`.
    Index,
}

impl Bank {
    /// The banks an indexed operand's number field selects (§2.3).
    const INDEXABLE: [Bank; 4] = [Bank::Temp, Bank::Output, Bank::Primary, Bank::Secondary];

    fn prefix(self) -> &'static str {
        match self {
            Bank::Temp => "r",
            Bank::Output => "o",
            Bank::Primary => "pa",
            Bank::Secondary => "sa",
            Bank::Special => "g",
            Bank::Index => "i",
        }
    }
}

/// How an immediate prints. Two styles, because the reference disassembler
/// prints an encoded operand in hex and the multiply-add forms' implicit
/// multiplier in decimal; keeping both keeps the listings comparable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImmStyle {
    /// `#0x1c`; values up to 9, and negative ones, in decimal.
    Hex,
    /// `#28`.
    Dec,
}

/// One decoded operand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operand {
    /// No operand.
    None,
    /// A register.
    Reg {
        /// Which bank.
        bank: Bank,
        /// The register number.
        num: u8,
    },
    /// A register addressed through an index register: `bank[i<index> +
    /// offset]` (§2.3: the number field becomes `[6:5]` bank, `[4:0]`
    /// offset).
    Indexed {
        /// Which bank.
        bank: Bank,
        /// The index register, 1 or 2.
        index: u8,
        /// The offset added to it.
        offset: u8,
    },
    /// An immediate. Signed so that the encodings with a sign print as such;
    /// the machine uses the low 32 bits.
    Imm {
        /// The value.
        value: i64,
        /// How it prints.
        style: ImmStyle,
    },
    /// A predicate register, `p0`–`p3`.
    Pred(u8),
    /// A data-return counter.
    Drc(u8),
    /// A destination that is not written back (`_`).
    Discard,
    /// An absolute branch target, in instructions from the code base.
    Abs(u32),
    /// A relative branch target, in instructions from the branch itself.
    Rel(i32),
    /// One of `PHAS`'s selector arguments, printed as the encoder's operand
    /// type and number (`t16:60`) — their meaning is unknown (§7).
    Selector {
        /// The encoder's operand type.
        kind: u8,
        /// The selector value.
        value: u8,
    },
}

/// An operand with its modifiers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Arg {
    /// The operand.
    pub op: Operand,
    /// Negated (the integer multiply-add forms, §4).
    pub neg: bool,
    /// Bitwise-inverted (the bitwise forms' `b43`, §3).
    pub inv: bool,
    /// The high 16 bits rather than the low (`IMAE`'s `b56`, §2.3).
    pub hi: bool,
}

impl Arg {
    const NONE: Arg = Arg::of(Operand::None);

    const fn of(op: Operand) -> Arg {
        Arg {
            op,
            neg: false,
            inv: false,
            hi: false,
        }
    }
}

/// An instruction's predicate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pred {
    /// Unconditional.
    Always,
    /// Executes when `p<n>` is set.
    If(u8),
    /// Executes when `p<n>` is clear.
    IfNot(u8),
    /// The per-instance predicate.
    PerInstance,
}

impl Pred {
    /// The three-bit "long" form (§2.1): `0 none, 1 p0, 2 p1, 3 p2, 4 p3,
    /// 5 !p0, 6 !p1, 7 pN`.
    fn long(v: u64) -> Pred {
        match v {
            0 => Pred::Always,
            1..=4 => Pred::If(v as u8 - 1),
            5 => Pred::IfNot(0),
            6 => Pred::IfNot(1),
            _ => Pred::PerInstance,
        }
    }

    /// The two-bit "short" form (§2.1): `0 none, 1 p0, 2 p1, 3 !p0`.
    fn short(v: u64) -> Pred {
        match v {
            0 => Pred::Always,
            1 => Pred::If(0),
            2 => Pred::If(1),
            _ => Pred::IfNot(0),
        }
    }

    /// The vector forms' two bits (§8, `OPTABLE`): `0 none, 1 p0, 2 !p0,
    /// 3 pN`.
    fn vector(v: u64) -> Pred {
        match v {
            0 => Pred::Always,
            1 => Pred::If(0),
            2 => Pred::IfNot(0),
            _ => Pred::PerInstance,
        }
    }
}

impl fmt::Display for Pred {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Pred::Always => Ok(()),
            Pred::If(n) => write!(f, "p{n}"),
            Pred::IfNot(n) => write!(f, "!p{n}"),
            Pred::PerInstance => f.write_str("pN"),
        }
    }
}

// ---------------------------------------------------------------------------
// Execution classes

/// A bitwise or shift operation (§3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitOp {
    /// `a & b`.
    And,
    /// `a | b`.
    Or,
    /// `a ^ b`.
    Xor,
    /// Shift left.
    Shl,
    /// Logical shift right.
    Shr,
    /// Arithmetic shift right.
    Asr,
    /// Rotate left.
    Rol,
    /// `RLP` — semantics unknown.
    Rlp,
}

/// Which integer multiply-add unit (§4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MadUnit {
    /// Primary opcode 20.
    Hw20,
    /// Primary opcode 21, `IMAE`: a 16-bit half of `src0` times `src1`, plus
    /// a 32-bit `src2` \[strong\].
    Imae,
    /// Primary opcode 26.
    Hw26,
}

/// What an instruction does — the interpreter's dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// `dst = src1 OP src2`, hardware opcodes 10–14.
    Bitwise(BitOp),
    /// Any ALU operation that also writes a predicate (hardware opcode 9).
    Test,
    /// `dst = src0 * src1 + src2`.
    Mad(MadUnit),
    /// `LD`: memory to registers.
    Load,
    /// `ST`: registers to memory.
    Store,
    /// `LDR`: an SGX register to a USSE register.
    Ldr,
    /// `STR`: a USSE register to an SGX register.
    Str,
    /// Load a 32-bit immediate.
    Limm,
    /// Branch to an absolute target.
    Ba,
    /// Branch to a relative target.
    Br,
    /// Return through the link register.
    Lapc,
    /// Set the link register.
    Setl,
    /// Save the link register.
    Savl,
    /// No operation.
    Nop,
    /// Issue a data fence.
    Idf,
    /// Wait for a data-return counter.
    Wdf,
    /// Declare the next phase's entry.
    Phas,
    /// End the phase (useasm opcode 358) \[G\].
    PhaseEnd,
    /// Emit to a fixed-function unit.
    Emit,
    /// Set the repeat increments \[G\].
    Smlsi,
    /// A vector floating-point instruction (not interpreted yet).
    Vector,
    /// Decoded, but with no known semantics.
    Opaque,
}

// ---------------------------------------------------------------------------
// The form table

/// How a predicate is encoded.
#[derive(Clone, Copy, Debug)]
enum PredEnc {
    None,
    Long(u32),
    Short(u32),
    Vector(u32),
}

/// How the repeat count is encoded: `count - 1` in `width` bits at `lo`.
#[derive(Clone, Copy, Debug)]
enum RptEnc {
    None,
    Field(u32, u32),
}

/// How an immediate in a source slot is encoded.
#[derive(Clone, Copy, Debug)]
enum Imm {
    /// The slot's 7 bits, unsigned.
    U7,
    /// The slot's 7 bits, two's complement.
    S7,
    /// The slot's 7 bits, two's complement, printed in decimal.
    Dec,
    /// The bitwise forms' 16-bit immediate with a rotate (§3).
    Wide16,
    /// `LDR`/`STR`'s 18-bit register number (§6.3).
    Wide18,
}

/// A source slot's number field and bank bits (§2.3).
#[derive(Clone, Copy, Debug)]
enum Slot {
    /// `src1`: number `b13:7`, bank `b31:30`, extended bank `b49`.
    S1,
    /// `src2`: number `b6:0`, bank `b29:28`, extended bank `b48`.
    S2,
}

/// A negation bit: which bit, and whether *set* means negated (some forms
/// encode it inverted: the encoder's base word has it set for a positive
/// operand).
#[derive(Clone, Copy, Debug)]
struct Neg(u32, bool);

/// One operand's encoding.
#[derive(Clone, Copy, Debug)]
enum Opd {
    /// The standard destination: number `b27:21`, bank `(b51, b33, b32)`.
    Dst,
    /// The test form's destination: `_` when `b20` is clear.
    DstTest,
    /// The load destinations: number `b27:21`, `b39` primary attribute.
    DstMem,
    /// `src0` of the opcode-20/21 multiply-adds: `b20:14`, `b34` primary
    /// attribute; `b56` selects `IMAE`'s high half.
    Src0Mad { hi: bool },
    /// `src0` of opcode 26: `b20:14`, `b34` primary attribute, `b47` output,
    /// both secondary; `b40` negates.
    Src0Mad26,
    /// A memory base: `b20:14`; `b34` primary attribute, the given bit
    /// output, both secondary.
    Src0Mem(u32),
    /// An `EMIT` register operand: `src0` as [`Opd::Src0Mem`] with `b51`, or
    /// a standard slot, naming a register *pair* — the encoder stores
    /// register `n` as `n / 2` (`OPTABLE`, useasm 162: `n[7:1]`).
    EmitSrc(Option<Slot>),
    /// A standard source slot.
    Src {
        slot: Slot,
        imm: Imm,
        neg: Option<Neg>,
        inv: bool,
    },
    /// The test form's `src1`; the immediate's sign depends on the sub-op.
    TestS1,
    /// The reference disassembler's repeat of `src1` in the multiply-add
    /// test forms.
    TestDup,
    /// The test form's `src2`.
    TestS2,
    /// The test form's predicate destination, `b35:34`.
    PredDst,
    /// A data-return counter, `b32`.
    Drc,
    /// An immediate bit field.
    Field(u32, u32),
    /// `LIMM`'s 32-bit immediate (§5.1).
    LimmImm,
    /// `BA`'s target, `b19:0`.
    AbsTarget,
    /// `BR`'s target, `b19:0` signed.
    RelTarget,
    /// `EMIT`'s 14-bit immediate.
    EmitImm,
    /// `SMLSI`'s four signed increments, one byte each.
    SmlsiInc(u32),
    /// `SMLSI`'s small fields, printed doubled as the encoder takes them.
    SmlsiField(u32),
    /// `PHAS`'s selectors (§7).
    PhasSel(u8),
    /// The vector forms' 6-bit register fields (§8; not decoded further).
    VecReg(u32),
}

/// A flag the disassembler prints when its bits differ from the encoder's
/// base word.
#[derive(Clone, Copy, Debug)]
struct Flag(&'static str, u64);

/// How the mnemonic is formed.
#[derive(Clone, Copy, Debug)]
enum Name {
    Fixed(&'static str),
    /// `ld`/`st` + address space (`a`, `l`, `t`) + size (`b`, `w`, `d`, `q`).
    Mem(&'static str),
    /// The test form: the sub-op's name + `.test`.
    Test,
}

/// One instruction form.
#[derive(Debug)]
struct Form {
    mask: u64,
    value: u64,
    name: Name,
    /// Flag bits the encoder's base word has set: a flag in this mask prints
    /// when the word has it *clear*.
    inverted: u64,
    pred: PredEnc,
    rpt: RptEnc,
    /// The END bit (§2.2).
    end: Option<u32>,
    flags: &'static [Flag],
    ops: &'static [Opd],
    exec: Op,
}

// Flag lists, in the order the reference disassembler prints them (by the
// encoder's flag number, as text). Positions §2.2 and `OPTABLE`.
const F_BITWISE: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("nosched", b(52)),
    Flag("f1.20", b(34)),
    Flag("end", b(54)),
];
const F_TEST_BIT: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("f1.20", b(50)),
    Flag("end", b(54)),
    Flag("f2.0", b(53)),
];
const F_TEST_MAD: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("end", b(54)),
    Flag("f2.0", b(53)),
    Flag("f2.10", b(50)),
];
const F_MAD20: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
    Flag("f2.10", b(42)),
    Flag("f2.15", b(35)),
    Flag("f2.16", b(36)),
];
const F_MAD21: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
    Flag("f2.10", b(42)),
    Flag("f2.15", b(35)),
    Flag("f2.16", b(36)),
    Flag("f2.17", b(37)),
];
const F_MAD26: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
];
const F_LD: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("f1.27", b(40)),
    Flag("f1.28", b(41)),
    Flag("end", b(54)),
    Flag("f1.31", b(53)),
    Flag("f2.2", b(33)),
    Flag("f2.3", b(51)),
    Flag("f2.9", b(35)),
];
const F_ST: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("f1.27", b(40)),
    Flag("f1.28", b(41)),
    Flag("f1.29", b(38)),
    Flag("end", b(54)),
    Flag("f1.31", b(53)),
    Flag("f2.2", b(33)),
    Flag("f2.3", b(51)),
    Flag("f2.9", b(35)),
];
const F_BRANCH: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("link", b(41)),
    Flag("f1.22", b(42)),
    Flag("end", b(43)),
    Flag("f3.14", b(45)),
    Flag("f3.22", b(20)),
    Flag("f3.23", b(21)),
    Flag("f3.4", b(44) | b(55)),
];
const F_LAPC: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("end", b(43)),
    Flag("f3.14", b(45)),
    Flag("f3.4", b(44) | b(55)),
];
const F_END43: &[Flag] = &[Flag("end", b(43))];
const F_NOP: &[Flag] = &[
    Flag("syncs?", b(50)),
    Flag("nosched", b(55)),
    Flag("end", b(43)),
    Flag("f2.26", b(0)),
    Flag("f3.5", b(1)),
];
const F_SKIP_END43: &[Flag] = &[Flag("skipinv", b(55)), Flag("end", b(43))];
const F_LIMM: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
];
const F_EMIT: &[Flag] = &[
    Flag("syncs?", b(50)),
    Flag("end", b(43)),
    Flag("f2.4", b(45)),
    Flag("f2.5", b(44)),
    Flag("f3.20", b(21)),
];
const F_SYNCS51: &[Flag] = &[Flag("syncs?", b(51))];
const F_SMLSI: &[Flag] = &[Flag("end", b(50))];
const F_VEC: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("end", b(43)),
];

// Operand lists.
const S1: Opd = Opd::Src {
    slot: Slot::S1,
    imm: Imm::U7,
    neg: None,
    inv: false,
};
const S2: Opd = Opd::Src {
    slot: Slot::S2,
    imm: Imm::U7,
    neg: None,
    inv: false,
};
const S1_DEC: Opd = Opd::Src {
    slot: Slot::S1,
    imm: Imm::Dec,
    neg: None,
    inv: false,
};
const O_BITWISE: &[Opd] = &[
    Opd::Dst,
    S1,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::Wide16,
        neg: None,
        inv: true,
    },
];
const O_MOV: &[Opd] = &[Opd::Dst, S1];
const O_TEST: &[Opd] = &[
    Opd::DstTest,
    Opd::PredDst,
    Opd::TestS1,
    Opd::TestDup,
    Opd::TestS2,
];
const O_MAD20: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad { hi: false },
    S1_DEC,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::S7,
        neg: Some(Neg(53, true)),
        inv: false,
    },
];
const O_MAD21: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad { hi: true },
    S1_DEC,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::S7,
        neg: None,
        inv: false,
    },
];
const O_MAD26S: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad26,
    S1_DEC,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::S7,
        neg: Some(Neg(39, false)),
        inv: false,
    },
];
const O_MAD26U: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad26,
    S1_DEC,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::U7,
        neg: Some(Neg(39, false)),
        inv: false,
    },
];
const O_LD: &[Opd] = &[Opd::DstMem, Opd::Src0Mem(50), S1, Opd::Drc];
const O_ST: &[Opd] = &[Opd::Src0Mem(50), S1, S2];
const O_LDR: &[Opd] = &[
    Opd::DstMem,
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::Wide18,
        neg: None,
        inv: false,
    },
    Opd::Drc,
];
const O_STR: &[Opd] = &[
    Opd::Src {
        slot: Slot::S2,
        imm: Imm::Wide18,
        neg: None,
        inv: false,
    },
    S1,
];
const O_LIMM: &[Opd] = &[Opd::Dst, Opd::LimmImm];
const O_ABS: &[Opd] = &[Opd::AbsTarget];
const O_REL: &[Opd] = &[Opd::RelTarget];
const O_SETL: &[Opd] = &[S1];
const O_SAVL: &[Opd] = &[Opd::Dst];
const O_IDF: &[Opd] = &[Opd::Drc, Opd::Field(46, 1)];
const O_WDF: &[Opd] = &[Opd::Drc];
const O_EMIT: &[Opd] = &[
    Opd::Field(32, 2),
    Opd::EmitSrc(None),
    Opd::EmitSrc(Some(Slot::S1)),
    Opd::EmitSrc(Some(Slot::S2)),
    Opd::EmitImm,
];
const O_SMLSI: &[Opd] = &[
    Opd::SmlsiInc(24),
    Opd::SmlsiInc(16),
    Opd::SmlsiInc(8),
    Opd::SmlsiInc(0),
    Opd::Field(35, 1),
    Opd::Field(34, 1),
    Opd::Field(33, 1),
    Opd::Field(32, 1),
    Opd::SmlsiField(44),
    Opd::SmlsiField(40),
    Opd::SmlsiField(36),
];
const O_PHAS: &[Opd] = &[
    Opd::Field(0, 20),
    Opd::Field(32, 8),
    Opd::PhasSel(0),
    Opd::PhasSel(1),
    Opd::PhasSel(2),
];
const O_OP205: &[Opd] = &[Opd::Field(4, 3)];
const O_VMAD: &[Opd] = &[
    Opd::VecReg(22),
    Opd::VecReg(12),
    Opd::VecReg(6),
    Opd::VecReg(0),
];

/// Fields used in match masks.
const CLASS: u64 = 7 << 52; // op 31: b54:52 (§7)
const SUB: u64 = 7 << 56; // op 31 classes 1/2: b58:56
const FLOW: u64 = 7 << 38; // op 31 class 0: b40:38

const fn class(c: u64) -> u64 {
    op(31) | c << 52
}

/// The instruction forms. First match wins, so a form that refines another
/// (`mov` within `or`) comes first. Match bits, flags and operand fields are
/// those measured from the encoder (sections cited per family).
static FORMS: &[Form] = &[
    // -- op 31, class 0: flow (§7). b58:56 predicate, b40:38 sub-op.
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0),
        name: Name::Fixed("ba"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_BRANCH,
        ops: O_ABS,
        exec: Op::Ba,
    },
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0) | 1 << 38,
        name: Name::Fixed("br"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_BRANCH,
        ops: O_REL,
        exec: Op::Br,
    },
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0) | 2 << 38,
        name: Name::Fixed("lapc"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_LAPC,
        ops: &[],
        exec: Op::Lapc,
    },
    // §5.2: SETL/SAVL are the encoder's MOV with a link-register operand.
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0) | 3 << 38,
        name: Name::Fixed("setl"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_END43,
        ops: O_SETL,
        exec: Op::Setl,
    },
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0) | 4 << 38,
        name: Name::Fixed("savl"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_END43,
        ops: O_SAVL,
        exec: Op::Savl,
    },
    Form {
        mask: OPCODE | CLASS | FLOW,
        value: class(0) | 5 << 38,
        name: Name::Fixed("nop"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_NOP,
        ops: &[],
        exec: Op::Nop,
    },
    // -- op 31, class 1 (b52): sub-op 2 is SMLSI (§7).
    Form {
        mask: OPCODE | CLASS | SUB,
        value: class(1) | 2 << 56,
        name: Name::Fixed("smlsi"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(50),
        flags: F_SMLSI,
        ops: O_SMLSI,
        exec: Op::Smlsi,
    },
    // -- op 31, class 2 (b53), sub-op b58:56 (§6.3, §6.4, §7).
    Form {
        mask: OPCODE | CLASS | SUB,
        value: class(2),
        name: Name::Fixed("idf"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_SKIP_END43,
        ops: O_IDF,
        exec: Op::Idf,
    },
    Form {
        mask: OPCODE | CLASS | SUB,
        value: class(2) | 1 << 56,
        name: Name::Fixed("wdf"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_END43,
        ops: O_WDF,
        exec: Op::Wdf,
    },
    // EMIT family: the form the microkernel uses has b47 set, b46 clear.
    Form {
        mask: OPCODE | CLASS | SUB | b(47) | b(46),
        value: class(2) | 3 << 56 | b(47),
        name: Name::Fixed("emit162"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_EMIT,
        ops: O_EMIT,
        exec: Op::Emit,
    },
    Form {
        mask: OPCODE | CLASS | SUB,
        value: class(2) | 4 << 56,
        name: Name::Fixed("limm"),
        inverted: 0,
        pred: PredEnc::Long(41),
        rpt: RptEnc::None,
        end: Some(54),
        flags: F_LIMM,
        ops: O_LIMM,
        exec: Op::Limm,
    },
    Form {
        mask: OPCODE | CLASS | SUB | b(0),
        value: class(2) | 5 << 56,
        name: Name::Fixed("op205"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_END43,
        ops: O_OP205,
        exec: Op::Opaque,
    },
    Form {
        mask: OPCODE | CLASS | SUB | b(51),
        value: class(2) | 6 << 56,
        name: Name::Fixed("ldr"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::Field(44, 4),
        end: Some(43),
        flags: F_SKIP_END43,
        ops: O_LDR,
        exec: Op::Ldr,
    },
    Form {
        mask: OPCODE | CLASS | SUB | b(51),
        value: class(2) | 6 << 56 | b(51),
        name: Name::Fixed("str"),
        inverted: 0,
        pred: PredEnc::Short(41),
        rpt: RptEnc::Field(44, 4),
        end: Some(43),
        flags: F_SKIP_END43,
        ops: O_STR,
        exec: Op::Str,
    },
    // -- op 31, class 4 (b54 alone), sub-op 2: PHAS; with b55, opcode 358.
    Form {
        mask: OPCODE | CLASS | SUB | b(55),
        value: class(4) | 2 << 56,
        name: Name::Fixed("phas"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: None,
        flags: F_SYNCS51,
        ops: O_PHAS,
        exec: Op::Phas,
    },
    Form {
        mask: OPCODE | CLASS | SUB | b(55),
        value: class(4) | 2 << 56 | b(55),
        name: Name::Fixed("op358"),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: None,
        flags: F_SYNCS51,
        ops: &[],
        exec: Op::PhaseEnd,
    },
    // -- op 9: the TEST form of any ALU op (§5.3). b19:14 is the ALU sub-op;
    // 0x30–0x37 are the bitwise ones.
    Form {
        mask: OPCODE | 0x30 << 14,
        value: op(9) | 0x30 << 14,
        name: Name::Test,
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(54),
        flags: F_TEST_BIT,
        ops: O_TEST,
        exec: Op::Test,
    },
    Form {
        mask: OPCODE,
        value: op(9),
        name: Name::Test,
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::None,
        end: Some(54),
        flags: F_TEST_MAD,
        ops: O_TEST,
        exec: Op::Test,
    },
    // -- ops 10–14: bitwise and shifts (§3). b35 picks the second member.
    // MOV is `or dst, src, #0` (useasm 163).
    Form {
        mask: OPCODE | b(35) | b(48) | 3 << 28 | 0x7f | 0x7f << 14 | 3 << 36 | 0x1f << 38 | b(43),
        value: op(10) | b(35) | b(48) | 2 << 28,
        name: Name::Fixed("mov"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_BITWISE,
        ops: O_MOV,
        exec: Op::Bitwise(BitOp::Or),
    },
    bitwise(10, false, "and", BitOp::And),
    bitwise(10, true, "or", BitOp::Or),
    Form {
        mask: OPCODE,
        value: op(11),
        name: Name::Fixed("xor"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_BITWISE,
        ops: O_BITWISE,
        exec: Op::Bitwise(BitOp::Xor),
    },
    bitwise(12, false, "shl", BitOp::Shl),
    bitwise(12, true, "rol", BitOp::Rol),
    bitwise(13, false, "shr", BitOp::Shr),
    bitwise(13, true, "asr", BitOp::Asr),
    Form {
        mask: OPCODE,
        value: op(14),
        name: Name::Fixed("rlp"),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_BITWISE,
        ops: O_BITWISE,
        exec: Op::Bitwise(BitOp::Rlp),
    },
    // -- ops 20, 21, 26: integer multiply-add (§4). The variant bit (b43,
    // b41) only selects the name here; what it changes is unknown.
    mad(20, 43, true, "imadd20", O_MAD20, F_MAD20, MadUnit::Hw20),
    mad(20, 43, false, "imadd20b", O_MAD20, F_MAD20, MadUnit::Hw20),
    mad(21, 43, false, "imae", O_MAD21, F_MAD21, MadUnit::Imae),
    mad(21, 43, true, "imae_b", O_MAD21, F_MAD21, MadUnit::Imae),
    // §4: opcode 26 with b52 set never came out of the encoder (6 words in
    // the microkernel); left undecoded rather than guessed.
    mad26(true, "imadd26n", O_MAD26S),
    mad26(false, "imadd26", O_MAD26U),
    // -- ops 29, 30: LD and ST (§6.1, §6.2).
    Form {
        mask: OPCODE,
        value: op(29),
        name: Name::Mem("ld"),
        inverted: b(53),
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_LD,
        ops: O_LD,
        exec: Op::Load,
    },
    Form {
        mask: OPCODE,
        value: op(30),
        name: Name::Mem("st"),
        inverted: b(53),
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_ST,
        ops: O_ST,
        exec: Op::Store,
    },
    // -- vector floating point (§8): decoded to a name only.
    vector(0, b(58), 0, "vmad", O_VMAD),
    vector(0, b(58), b(58), "vmad.b58", O_VMAD),
    vector(1, 0, 0, "vop1", &[]),
    vector(2, 0, 0, "vop2", &[]),
    vector(6, 0, 0, "vcplx", &[]),
    vector(8, 0, 0, "pck", &[]),
];

const fn bitwise(hw: u64, second: bool, name: &'static str, bop: BitOp) -> Form {
    Form {
        mask: OPCODE | b(35),
        value: op(hw) | if second { b(35) } else { 0 },
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 4),
        end: Some(54),
        flags: F_BITWISE,
        ops: O_BITWISE,
        exec: Op::Bitwise(bop),
    }
}

const fn mad(
    hw: u64,
    variant: u32,
    set: bool,
    name: &'static str,
    ops: &'static [Opd],
    flags: &'static [Flag],
    unit: MadUnit,
) -> Form {
    Form {
        mask: OPCODE | b(variant),
        value: op(hw) | if set { b(variant) } else { 0 },
        name: Name::Fixed(name),
        inverted: 0,
        // §2.1: the two-bit predicate at b58:57 on opcodes 20 and 21.
        pred: PredEnc::Short(57),
        rpt: RptEnc::Field(44, 3),
        end: Some(54),
        flags,
        ops,
        exec: Op::Mad(unit),
    }
}

const fn mad26(variant: bool, name: &'static str, ops: &'static [Opd]) -> Form {
    Form {
        mask: OPCODE | b(52) | b(41),
        value: op(26) | if variant { b(41) } else { 0 },
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 3),
        end: Some(54),
        flags: F_MAD26,
        ops,
        exec: Op::Mad(MadUnit::Hw26),
    }
}

const fn vector(
    hw: u64,
    extra_mask: u64,
    extra: u64,
    name: &'static str,
    ops: &'static [Opd],
) -> Form {
    Form {
        mask: OPCODE | extra_mask,
        value: op(hw) | extra,
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::Vector(56),
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_VEC,
        ops,
        exec: Op::Vector,
    }
}

/// The test form's ALU sub-op names (§5.3), by `b19:14`. The multiply-add
/// ones are the encoder's opcodes; their meaning is graded in §4.
fn test_subop_name(sub: u64) -> Option<&'static str> {
    Some(match sub {
        0x16 => "imadd20",
        0x17 => "imadd20n",
        0x18 => "imadd20c",
        0x19 => "imadd20b",
        0x1a => "imadd20bn",
        0x1b => "imadd20bc",
        0x1c => "imae_b",
        0x1d => "imae",
        0x1e => "imadd26n",
        0x1f => "imadd26",
        0x20 => "op235",
        0x21 => "op236",
        0x22 => "op245",
        0x23 => "op246",
        0x24 => "op237",
        0x25 => "op247",
        0x26 => "op238",
        0x27 => "op248",
        0x28 => "op249",
        0x29 => "op250",
        0x30 => "and",
        0x31 => "or",
        0x32 => "xor",
        0x33 => "shl",
        0x34 => "shr",
        0x35 => "rol",
        0x37 => "asr",
        _ => return None,
    })
}

/// Whether a test sub-op is one of the multiply-adds the encoder fronts with
/// an implicit `src1` (useasm 228, 229, 231, 232, 234, 240, 359, 360).
fn test_subop_has_dup(sub: u64) -> bool {
    matches!(sub, 0x16 | 0x17 | 0x19 | 0x1a | 0x1c | 0x1d | 0x1e | 0x1f)
}

/// Whether a test sub-op's immediates are 7-bit unsigned rather than signed
/// (the encoder accepts six magnitude bits for the others; `OPTABLE`).
fn test_subop_unsigned(sub: u64) -> bool {
    sub >= 0x30 || matches!(sub, 0x1d | 0x1f)
}

// ---------------------------------------------------------------------------
// Decoding

/// The most operands any form has.
const MAX_ARGS: usize = 11;

/// A decoded instruction.
#[derive(Clone, Copy, Debug)]
pub struct Insn {
    /// The instruction word.
    pub word: u64,
    /// The predicate.
    pub pred: Pred,
    /// The repeat count, 1–16.
    pub repeat: u8,
    form: &'static Form,
    args: [Arg; MAX_ARGS],
    nargs: u8,
}

impl Insn {
    /// The operands, in disassembly order.
    pub fn args(&self) -> &[Arg] {
        &self.args[..usize::from(self.nargs)]
    }

    /// What the instruction does.
    pub fn op(&self) -> Op {
        self.form.exec
    }

    /// Whether the END flag is set (§2.2).
    pub fn is_end(&self) -> bool {
        self.form.end.is_some_and(|n| bit(self.word, n))
    }

    /// The test form's ALU sub-op, `b19:14` (§5.3).
    fn test_subop(&self) -> u64 {
        field(self.word, 14, 6)
    }
}

/// Decodes one instruction word; `None` when no form matches.
pub fn decode(word: u64) -> Option<Insn> {
    let form = FORMS.iter().find(|f| word & f.mask == f.value)?;
    let pred = match form.pred {
        PredEnc::None => Pred::Always,
        PredEnc::Long(lo) => Pred::long(field(word, lo, 3)),
        PredEnc::Short(lo) => Pred::short(field(word, lo, 2)),
        PredEnc::Vector(lo) => Pred::vector(field(word, lo, 2)),
    };
    let repeat = match form.rpt {
        RptEnc::None => 1,
        RptEnc::Field(lo, width) => field(word, lo, width) as u8 + 1,
    };
    let mut args = [Arg::NONE; MAX_ARGS];
    let mut nargs = 0;
    for opd in form.ops {
        if let Some(arg) = decode_opd(*opd, word) {
            args[nargs] = arg;
            nargs += 1;
        }
    }
    Some(Insn {
        word,
        pred,
        repeat,
        form,
        args,
        nargs: nargs as u8,
    })
}

/// The standard destination (§2.3, "Destination").
fn dst_std(w: u64) -> Operand {
    let num = field(w, 21, 7) as u8;
    match (bit(w, 51), bit(w, 33), bit(w, 32)) {
        (false, false, false) => Operand::Reg {
            bank: Bank::Temp,
            num,
        },
        (false, false, true) => Operand::Reg {
            bank: Bank::Output,
            num,
        },
        (false, true, false) => Operand::Reg {
            bank: Bank::Primary,
            num,
        },
        (true, false, false) => Operand::Reg {
            bank: Bank::Secondary,
            num,
        },
        (true, true, false) => Operand::Reg {
            bank: Bank::Index,
            num,
        },
        (true, false, true) => Operand::Reg {
            bank: Bank::Special,
            num,
        },
        (false, true, true) => indexed(1, num),
        (true, true, true) => indexed(2, num),
    }
}

fn indexed(index: u8, num: u8) -> Operand {
    Operand::Indexed {
        bank: Bank::INDEXABLE[usize::from(num >> 5 & 3)],
        index,
        offset: num & 31,
    }
}

/// A standard source slot (§2.3, "src1"/"src2").
fn src_slot(w: u64, slot: Slot, imm: Imm) -> Operand {
    let (num, bank, ext) = match slot {
        Slot::S1 => (field(w, 7, 7), field(w, 30, 2), bit(w, 49)),
        Slot::S2 => (field(w, 0, 7), field(w, 28, 2), bit(w, 48)),
    };
    if !ext {
        return Operand::Reg {
            bank: Bank::INDEXABLE[bank as usize],
            num: num as u8,
        };
    }
    match bank {
        0 => indexed(1, num as u8),
        1 => Operand::Reg {
            bank: Bank::Special,
            num: num as u8,
        },
        3 => indexed(2, num as u8),
        _ => {
            let (value, style) = match imm {
                Imm::U7 => (num as i64, ImmStyle::Hex),
                Imm::S7 => (sext7(num), ImmStyle::Hex),
                Imm::Dec => (sext7(num), ImmStyle::Dec),
                // §3: imm[6:0] → b6:0, imm[13:7] → b20:14, imm[15:14] →
                // b37:36, rotated left by b42:38 (the encoder rotates a
                // constant right until it fits).
                Imm::Wide16 => {
                    let v = (field(w, 0, 7) | field(w, 14, 7) << 7 | field(w, 36, 2) << 14) as u32;
                    (
                        i64::from(v.rotate_left(field(w, 38, 5) as u32)),
                        ImmStyle::Hex,
                    )
                }
                // §6.3: n[6:0] → b6:0, n[13:7] → b20:14, n[17:14] → b37:34.
                Imm::Wide18 => (
                    (field(w, 0, 7) | field(w, 14, 7) << 7 | field(w, 34, 4) << 14) as i64,
                    ImmStyle::Hex,
                ),
            };
            Operand::Imm { value, style }
        }
    }
}

fn reg(bank: Bank, num: u64) -> Operand {
    Operand::Reg {
        bank,
        num: num as u8,
    }
}

fn hex(value: u64) -> Operand {
    Operand::Imm {
        value: value as i64,
        style: ImmStyle::Hex,
    }
}

fn decode_opd(opd: Opd, w: u64) -> Option<Arg> {
    let sub = field(w, 14, 6);
    Some(match opd {
        Opd::Dst => Arg::of(dst_std(w)),
        // §5.3: with b20 clear the result is not written back. (§5.3 says
        // the "no write-back" flag *sets* b20; `OPTABLE` has it flipping
        // b51, b32 and b20 from a base that has b20 set, so it clears it —
        // and every firmware test that names a register has b20 set.)
        Opd::DstTest if !bit(w, 20) => Arg::of(Operand::Discard),
        Opd::DstTest => Arg::of(dst_std(w)),
        Opd::DstMem => Arg::of(reg(
            if bit(w, 39) {
                Bank::Primary
            } else {
                Bank::Temp
            },
            field(w, 21, 7),
        )),
        Opd::Src0Mad { hi } => Arg {
            hi: hi && bit(w, 56),
            ..Arg::of(reg(
                if bit(w, 34) {
                    Bank::Primary
                } else {
                    Bank::Temp
                },
                field(w, 14, 7),
            ))
        },
        Opd::Src0Mad26 => {
            let bank = match (bit(w, 47), bit(w, 34)) {
                (false, false) => Bank::Temp,
                (true, false) => Bank::Output,
                (false, true) => Bank::Primary,
                (true, true) => Bank::Secondary,
            };
            Arg {
                neg: bit(w, 40),
                ..Arg::of(reg(bank, field(w, 14, 7)))
            }
        }
        Opd::Src0Mem(out) => {
            let bank = match (bit(w, out), bit(w, 34)) {
                (false, false) => Bank::Temp,
                (true, false) => Bank::Output,
                (false, true) => Bank::Primary,
                (true, true) => Bank::Secondary,
            };
            Arg::of(reg(bank, field(w, 14, 7)))
        }
        Opd::EmitSrc(slot) => {
            let op = match slot {
                None => decode_opd(Opd::Src0Mem(51), w)?.op,
                Some(slot) => src_slot(w, slot, Imm::U7),
            };
            Arg::of(match op {
                Operand::Reg { bank, num } => Operand::Reg {
                    bank,
                    num: num.wrapping_mul(2),
                },
                op => op,
            })
        }
        Opd::Src {
            slot,
            imm,
            neg,
            inv,
        } => Arg {
            neg: neg.is_some_and(|Neg(n, set)| bit(w, n) == set),
            inv: inv && bit(w, 43),
            ..Arg::of(src_slot(w, slot, imm))
        },
        Opd::TestS1 => Arg::of(src_slot(
            w,
            Slot::S1,
            if test_subop_unsigned(sub) {
                Imm::U7
            } else {
                Imm::S7
            },
        )),
        Opd::TestDup if test_subop_has_dup(sub) => Arg::of(src_slot(w, Slot::S1, Imm::Dec)),
        Opd::TestDup => return None,
        Opd::TestS2 => Arg::of(src_slot(
            w,
            Slot::S2,
            if test_subop_unsigned(sub) {
                Imm::U7
            } else {
                Imm::S7
            },
        )),
        Opd::PredDst => Arg::of(Operand::Pred(field(w, 34, 2) as u8)),
        Opd::Drc => Arg::of(Operand::Drc(field(w, 32, 1) as u8)),
        Opd::Field(lo, width) => Arg::of(hex(field(w, lo, width))),
        // §5.1: imm[20:0] → b20:0, imm[25:21] → b40:36, imm[31:26] → b49:44.
        Opd::LimmImm => Arg::of(hex(field(w, 0, 21)
            | field(w, 36, 5) << 21
            | field(w, 44, 6) << 26)),
        Opd::AbsTarget => Arg::of(Operand::Abs(field(w, 0, 20) as u32)),
        Opd::RelTarget => {
            let v = field(w, 0, 20) as i32;
            Arg::of(Operand::Rel(if v & 0x8_0000 != 0 {
                v - 0x10_0000
            } else {
                v
            }))
        }
        // §7 / `OPTABLE`: n[5:0] → b27:22, n[11:6] → b40:35, n[13:12] → b55:54.
        Opd::EmitImm => Arg::of(hex(field(w, 22, 6)
            | field(w, 35, 6) << 6
            | field(w, 54, 2) << 12)),
        Opd::SmlsiInc(lo) => Arg::of(Operand::Imm {
            value: i64::from(field(w, lo, 8) as u8 as i8),
            style: ImmStyle::Hex,
        }),
        Opd::SmlsiField(lo) => Arg::of(hex(field(w, lo, 4) << 1)),
        Opd::PhasSel(n) => Arg::of(phas_selector(w, n)),
        Opd::VecReg(lo) => Arg::of(reg(Bank::Temp, field(w, lo, 6))),
    })
}

/// `PHAS`'s three selector arguments (§7): the encoder's int-source types
/// 60–62 from `b44`/`b43`, 63/64/65 as `b42:40` = 1/2/7, 66/67 from `b45`.
/// `b42:40 = 0` never came out of the encoder; it prints as `t14:0`, the
/// reference disassembler's rendering.
fn phas_selector(w: u64, n: u8) -> Operand {
    let (kind, value) = match n {
        0 => (16, 60 ^ (field(w, 44, 1) | field(w, 43, 1) << 1) as u8),
        1 => match field(w, 40, 3) {
            0 => (14, 0),
            1 => (16, 63),
            2 => (16, 64),
            7 => (16, 65),
            v => (15, v as u8),
        },
        _ => (16, 66 + field(w, 45, 1) as u8),
    };
    Operand::Selector { kind, value }
}

// ---------------------------------------------------------------------------
// Disassembly

/// Renders an instruction in the style of our reference disassembler:
/// `[(pred) ]name[.flag…][.rptN] operands`.
#[derive(Clone, Copy, Debug)]
pub struct Disasm<'a> {
    insn: &'a Insn,
    /// The instruction's own address, for relative targets.
    pc: u32,
    /// The code base, for absolute targets.
    code_base: u32,
}

impl<'a> Disasm<'a> {
    /// Prepares `insn`, fetched from GPU address `pc`, for display; absolute
    /// branch targets are shown relative to `code_base`.
    pub fn new(insn: &'a Insn, pc: u32, code_base: u32) -> Self {
        Disasm {
            insn,
            pc,
            code_base,
        }
    }
}

/// Disassembles one word fetched from `pc`. An undecodable word is `.word`.
pub fn disassemble(word: u64, pc: u32, code_base: u32) -> String {
    let mut s = String::new();
    match decode(word) {
        Some(insn) => {
            let _ = write!(s, "{}", Disasm::new(&insn, pc, code_base));
        }
        None => s.push_str(".word"),
    }
    s
}

/// The test field (§5.3): `t0 b42, t1 b43, t2 b40, t3 b41, t4 b39, t5 b36,
/// t6 b37, t7 b38`.
fn test_bits(w: u64) -> u8 {
    const POS: [u32; 8] = [42, 43, 40, 41, 39, 36, 37, 38];
    POS.iter()
        .enumerate()
        .fold(0, |t, (i, &p)| t | (bit(w, p) as u8) << i)
}

fn write_test(f: &mut fmt::Formatter<'_>, t: u8) -> fmt::Result {
    let zero = ["", "z", "nz", "z?3"][usize::from(t >> 2 & 3)];
    let sign = ["", "n", "p", "s?3"][usize::from(t & 3)];
    let join = if t & 0x10 != 0 { "&" } else { "|" };
    f.write_str("t[")?;
    match (zero.is_empty(), sign.is_empty()) {
        (true, true) => f.write_str(if t & 0x10 != 0 { "true" } else { "false" })?,
        (false, true) => f.write_str(zero)?,
        (true, false) => f.write_str(sign)?,
        (false, false) => write!(f, "{zero}{join}{sign}")?,
    }
    if t & 0xe0 != 0 {
        write!(f, ",chan{:x}", t >> 5)?;
    }
    f.write_str("]")
}

fn write_imm(f: &mut fmt::Formatter<'_>, value: i64, style: ImmStyle) -> fmt::Result {
    match style {
        ImmStyle::Hex if value > 9 => write!(f, "#0x{value:x}"),
        _ => write!(f, "#{value}"),
    }
}

impl Disasm<'_> {
    fn write_operand(&self, f: &mut fmt::Formatter<'_>, op: Operand) -> fmt::Result {
        match op {
            Operand::None => Ok(()),
            Operand::Reg {
                bank: Bank::Special,
                num,
            } if num < 64 => write!(f, "c{num}"),
            Operand::Reg { bank, num } => write!(f, "{}{num}", bank.prefix()),
            Operand::Indexed {
                bank,
                index,
                offset,
            } => {
                write!(f, "{}[i{index}+{offset}]", bank.prefix())
            }
            Operand::Imm { value, style } => write_imm(f, value, style),
            Operand::Pred(n) => write!(f, "p{n}"),
            Operand::Drc(n) => write!(f, "drc{n}"),
            Operand::Discard => f.write_str("_"),
            Operand::Abs(n) => write!(
                f,
                "0x{n:x} -> 0x{:08x}",
                self.code_base.wrapping_add(n.wrapping_mul(8))
            ),
            Operand::Rel(n) => write!(
                f,
                "{n:+} -> 0x{:08x}",
                self.pc.wrapping_add((n as u32).wrapping_mul(8))
            ),
            Operand::Selector { kind, value } => write!(f, "t{kind}:{value}"),
        }
    }
}

impl fmt::Display for Disasm<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let insn = self.insn;
        let form = insn.form;
        let w = insn.word;
        if insn.pred != Pred::Always {
            write!(f, "({}) ", insn.pred)?;
        }
        match form.name {
            Name::Fixed(name) => f.write_str(name)?,
            Name::Mem(stem) => {
                let space = if bit(w, 43) {
                    "t"
                } else if bit(w, 42) {
                    "l"
                } else {
                    "a"
                };
                // §6.1: b37:36 = 0 dword, 1 word, 2 byte, 3 qword [G names].
                let size = ["d", "w", "b", "q"][field(w, 36, 2) as usize];
                write!(f, "{stem}{space}{size}")?;
            }
            Name::Test => {
                match test_subop_name(insn.test_subop()) {
                    Some(name) => f.write_str(name)?,
                    None => write!(f, "alu{:#x}", insn.test_subop())?,
                }
                f.write_str(".test.")?;
                write_test(f, test_bits(w))?;
            }
        }
        let differs = w ^ form.inverted;
        let mut shown = 0u64;
        for &Flag(name, mask) in form.flags {
            if differs & mask == mask && mask & !shown != 0 {
                write!(f, ".{name}")?;
                shown |= mask;
            }
        }
        if insn.repeat > 1 {
            write!(f, ".rpt{}", insn.repeat)?;
        }
        f.write_str(" ")?;
        for (i, arg) in insn.args().iter().enumerate() {
            if i > 0 {
                f.write_str(", ")?;
            }
            if arg.neg {
                f.write_str("-")?;
            }
            self.write_operand(f, arg.op)?;
            if arg.inv {
                f.write_str(".inv")?;
            }
            if arg.hi {
                f.write_str(".hi")?;
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// The bus

/// A data access size (§6.1's `b37:36`, names \[G\]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Size {
    /// 8 bits.
    Byte,
    /// 16 bits.
    Word,
    /// 32 bits.
    Dword,
}

impl Size {
    /// The size in bytes.
    pub fn bytes(self) -> u32 {
        match self {
            Size::Byte => 1,
            Size::Word => 2,
            Size::Dword => 4,
        }
    }
}

/// What an `EMIT` sends: a target unit, three source values and an
/// immediate, with the raw word for whatever is not decoded yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Emit {
    /// The target, `b33:32`.
    pub target: u8,
    /// The three source operands' values: each names a register pair (an
    /// immediate has a zero high half).
    pub sources: [[u32; 2]; 3],
    /// The 14-bit immediate.
    pub imm: u32,
    /// The instruction word.
    pub word: u64,
}

/// Everything outside the engine: GPU memory and the SGX register file.
///
/// Addresses are GPU virtual; translating them through the SGX MMU is the
/// caller's job.
pub trait UsseBus {
    /// Loads `size` bytes at `addr`, zero-extended.
    fn load(&mut self, addr: u32, size: Size) -> MemResult<u32>;

    /// Stores the low `size` bytes of `value` at `addr`.
    fn store(&mut self, addr: u32, size: Size, value: u32) -> MemResult;

    /// Fetches the instruction word at `addr` (§1: the lower address holds
    /// bits 31:0).
    fn fetch(&mut self, addr: u32) -> MemResult<u64> {
        let lo = self.load(addr, Size::Dword)?;
        let hi = self.load(addr.wrapping_add(4), Size::Dword)?;
        Ok(u64::from(hi) << 32 | u64::from(lo))
    }

    /// Reads the SGX register at dword index `index` (byte offset `4 *
    /// index`) for `LDR`.
    fn reg_read(&mut self, index: u32) -> u32;

    /// Writes the SGX register at dword index `index` for `STR`.
    fn reg_write(&mut self, index: u32, value: u32);

    /// Takes an `EMIT`. The default refuses it, which stops the engine.
    fn emit(&mut self, emit: &Emit) -> MemResult {
        let _ = emit;
        Err(BusError::Unassigned)
    }
}

// ---------------------------------------------------------------------------
// The interpreter

/// Registers per bank. The temporaries are 128 (§2.3's 7-bit number); the
/// others are sized for the indexed forms' reach, which the 7-bit field does
/// not bound.
pub const BANK_SIZE: usize = 256;

/// Why execution stopped. `pc` is always the instruction index of the
/// instruction concerned, relative to the code base.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// An instruction with END set was executed; `pc` is past it.
    End {
        /// The next instruction.
        pc: u32,
    },
    /// The phase ended (`PHAS` with `b51`, or opcode 358) \[G\]. `next` is the
    /// entry the program declared for its next phase, if any. It is left as
    /// encoded because its base is not settled: the slave microkernel's
    /// `PHAS` targets count from the main image's base, while its branches
    /// count from its own (§7).
    Phase {
        /// The instruction that ended the phase.
        pc: u32,
        /// The next phase's entry, as encoded (an instruction index).
        next: Option<u32>,
    },
    /// An instruction whose semantics are not implemented.
    Unimplemented {
        /// Where.
        pc: u32,
        /// The instruction word.
        word: u64,
        /// What is missing.
        why: &'static str,
    },
    /// No form matched.
    Undecodable {
        /// Where.
        pc: u32,
        /// The instruction word.
        word: u64,
    },
    /// A register outside its bank.
    BadRegister {
        /// Where.
        pc: u32,
        /// The instruction word.
        word: u64,
    },
    /// A memory access faulted.
    Fault {
        /// Where.
        pc: u32,
        /// The GPU address.
        addr: u32,
        /// The error.
        error: BusError,
    },
    /// The instruction budget ran out.
    Budget {
        /// The next instruction.
        pc: u32,
    },
}

/// A register location: bank and index.
#[derive(Clone, Copy, Debug)]
struct Loc(Bank, usize);

/// One USSE instance's architectural state.
pub struct Usse {
    /// The GPU address of instruction 0.
    pub code_base: u32,
    /// The next instruction, in instructions from `code_base`.
    pub pc: u32,
    /// Temporaries.
    pub temp: [u32; BANK_SIZE],
    /// Outputs.
    pub output: [u32; BANK_SIZE],
    /// Primary attributes.
    pub primary: [u32; BANK_SIZE],
    /// Secondary attributes.
    pub secondary: [u32; BANK_SIZE],
    /// Special/global registers, `g64`–`g127`.
    pub special: [u32; 64],
    /// The constant table, `c0`–`c63`. Its contents are not known; the caller
    /// supplies them \[G\].
    pub constants: [u32; 64],
    /// The index registers.
    pub index: [u32; 4],
    /// The predicates `p0`–`p3`.
    pub preds: [bool; 4],
    /// The link register, an instruction index.
    pub link: u32,
    /// The last `SMLSI`'s fields, kept but not yet applied \[G\].
    pub smlsi: [i32; 11],
    /// The entry the program declared for its next phase.
    pub next_phase: Option<u32>,
    /// Instructions retired.
    pub retired: u64,
}

impl fmt::Debug for Usse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Usse")
            .field("code_base", &format_args!("{:#x}", self.code_base))
            .field("pc", &format_args!("{:#x}", self.pc))
            .field("preds", &self.preds)
            .field("link", &self.link)
            .field("index", &self.index)
            .field("next_phase", &self.next_phase)
            .field("retired", &self.retired)
            .finish_non_exhaustive()
    }
}

impl Usse {
    /// A cleared instance about to execute instruction `pc` of the code at
    /// `code_base`.
    pub fn new(code_base: u32, pc: u32) -> Self {
        Usse {
            code_base,
            pc,
            temp: [0; BANK_SIZE],
            output: [0; BANK_SIZE],
            primary: [0; BANK_SIZE],
            secondary: [0; BANK_SIZE],
            special: [0; 64],
            constants: [0; 64],
            index: [0; 4],
            preds: [false; 4],
            link: 0,
            smlsi: [0; 11],
            next_phase: None,
            retired: 0,
        }
    }

    /// The GPU address of instruction `pc`.
    pub fn address_of(&self, pc: u32) -> u32 {
        self.code_base.wrapping_add(pc.wrapping_mul(8))
    }

    /// Runs until something stops it, or `budget` instructions have retired.
    pub fn run(&mut self, bus: &mut dyn UsseBus, budget: u64) -> Stop {
        for _ in 0..budget {
            if let Err(stop) = self.step(bus) {
                return stop;
            }
        }
        Stop::Budget { pc: self.pc }
    }

    /// Executes one instruction.
    pub fn step(&mut self, bus: &mut dyn UsseBus) -> Result<(), Stop> {
        let pc = self.pc;
        let addr = self.address_of(pc);
        let word = bus
            .fetch(addr)
            .map_err(|error| Stop::Fault { pc, addr, error })?;
        let insn = decode(word).ok_or(Stop::Undecodable { pc, word })?;
        let enabled = match insn.pred {
            Pred::Always => true,
            Pred::If(n) => self.preds[usize::from(n)],
            Pred::IfNot(n) => !self.preds[usize::from(n)],
            Pred::PerInstance => {
                return Err(unimpl(pc, word, "per-instance predicate"));
            }
        };
        self.pc = pc.wrapping_add(1);
        if enabled {
            match self.execute(bus, &insn, pc) {
                Ok(()) => {}
                // A phase end has executed: the PC stays past it.
                Err(stop @ Stop::Phase { .. }) => {
                    self.retired = self.retired.wrapping_add(1);
                    return Err(stop);
                }
                // Anything else has not: the PC stays on the instruction.
                Err(stop) => {
                    self.pc = pc;
                    return Err(stop);
                }
            }
        }
        self.retired = self.retired.wrapping_add(1);
        // [G] END on an instruction predicated off is ignored with it.
        if enabled && insn.is_end() {
            return Err(Stop::End { pc: self.pc });
        }
        Ok(())
    }

    fn loc(&self, op: Operand, rep: u8, pc: u32, word: u64) -> Result<Loc, Stop> {
        let bad = Stop::BadRegister { pc, word };
        let rep = usize::from(rep);
        let (bank, n) = match op {
            Operand::Reg { bank, num } => (bank, usize::from(num) + rep),
            // [G] An index register holds a register number.
            Operand::Indexed {
                bank,
                index,
                offset,
            } => (
                bank,
                (self.index[usize::from(index)] as usize)
                    .wrapping_add(usize::from(offset))
                    .wrapping_add(rep),
            ),
            _ => return Err(bad),
        };
        let limit = match bank {
            Bank::Temp => 128,
            Bank::Special => 128,
            Bank::Index => self.index.len(),
            _ => BANK_SIZE,
        };
        if n < limit {
            Ok(Loc(bank, n))
        } else {
            Err(bad)
        }
    }

    fn get(&self, Loc(bank, n): Loc) -> u32 {
        match bank {
            Bank::Temp => self.temp[n],
            Bank::Output => self.output[n],
            Bank::Primary => self.primary[n],
            Bank::Secondary => self.secondary[n],
            Bank::Special if n < 64 => self.constants[n],
            Bank::Special => self.special[n - 64],
            Bank::Index => self.index[n],
        }
    }

    fn set(&mut self, Loc(bank, n): Loc, v: u32, pc: u32, word: u64) -> Result<(), Stop> {
        match bank {
            Bank::Temp => self.temp[n] = v,
            Bank::Output => self.output[n] = v,
            Bank::Primary => self.primary[n] = v,
            Bank::Secondary => self.secondary[n] = v,
            Bank::Special if n < 64 => return Err(Stop::BadRegister { pc, word }),
            Bank::Special => self.special[n - 64] = v,
            Bank::Index => self.index[n] = v,
        }
        Ok(())
    }

    /// An operand's value on repeat iteration `rep`. \[G\] Every register
    /// operand steps by one register per iteration; immediates do not.
    fn read(&self, arg: &Arg, rep: u8, pc: u32, word: u64) -> Result<u32, Stop> {
        let v = match arg.op {
            Operand::Imm { value, .. } => value as u32,
            op => self.get(self.loc(op, rep, pc, word)?),
        };
        let v = if arg.hi { v >> 16 } else { v };
        let v = if arg.inv { !v } else { v };
        Ok(if arg.neg { v.wrapping_neg() } else { v })
    }

    fn write(&mut self, arg: &Arg, rep: u8, v: u32, pc: u32, word: u64) -> Result<(), Stop> {
        match arg.op {
            Operand::Discard => Ok(()),
            op => {
                let loc = self.loc(op, rep, pc, word)?;
                self.set(loc, v, pc, word)
            }
        }
    }

    fn execute(&mut self, bus: &mut dyn UsseBus, insn: &Insn, pc: u32) -> Result<(), Stop> {
        let w = insn.word;
        let a = insn.args();
        match insn.op() {
            Op::Bitwise(bop) => {
                for rep in 0..insn.repeat {
                    let x = self.read(&a[1], rep, pc, w)?;
                    // MOV has no second source: it is `or dst, src, #0`.
                    let y = match a.get(2) {
                        Some(arg) => self.read(arg, rep, pc, w)?,
                        None => 0,
                    };
                    let r = bitwise_op(bop, x, y).ok_or(unimpl(pc, w, "RLP"))?;
                    self.write(&a[0], rep, r, pc, w)?;
                }
            }
            Op::Test => self.test(insn, pc)?,
            Op::Mad(unit) => {
                for rep in 0..insn.repeat {
                    let s0 = self.read(&a[1], rep, pc, w)?;
                    let s1 = self.read(&a[2], rep, pc, w)?;
                    let s2 = self.read(&a[3], rep, pc, w)?;
                    let r = match unit {
                        // [strong] IMAE is 16 × 16 + 32: `hi` already picked
                        // the half; src1 contributes its low 16 bits [G].
                        MadUnit::Imae => (s0 & 0xffff).wrapping_mul(s1 & 0xffff).wrapping_add(s2),
                        // [G] the variant bits (b43, b41) and opcode 20's
                        // b35/b36 change nothing here.
                        MadUnit::Hw20 | MadUnit::Hw26 => s0.wrapping_mul(s1).wrapping_add(s2),
                    };
                    self.write(&a[0], rep, r, pc, w)?;
                }
            }
            Op::Load | Op::Store => self.memory(bus, insn, pc)?,
            Op::Ldr => {
                let index = self.read(&a[1], 0, pc, w)?;
                for rep in 0..insn.repeat {
                    let v = bus.reg_read(index.wrapping_add(u32::from(rep)));
                    self.write(&a[0], rep, v, pc, w)?;
                }
            }
            Op::Str => {
                let index = self.read(&a[0], 0, pc, w)?;
                for rep in 0..insn.repeat {
                    let v = self.read(&a[1], rep, pc, w)?;
                    bus.reg_write(index.wrapping_add(u32::from(rep)), v);
                }
            }
            Op::Limm => {
                let v = self.read(&a[1], 0, pc, w)?;
                self.write(&a[0], 0, v, pc, w)?;
            }
            Op::Ba | Op::Br => {
                // §7: b41 saves the return address [strong].
                if bit(w, 41) {
                    self.link = pc.wrapping_add(1);
                }
                self.pc = match a[0].op {
                    Operand::Abs(n) => n,
                    Operand::Rel(n) => pc.wrapping_add(n as u32),
                    _ => unreachable!("branch forms carry a target"),
                };
            }
            Op::Lapc => self.pc = self.link,
            Op::Setl => self.link = self.read(&a[0], 0, pc, w)?,
            Op::Savl => self.write(&a[0], 0, self.link, pc, w)?,
            Op::Nop | Op::Idf | Op::Wdf => {}
            Op::Phas => {
                let next = field(w, 0, 20) as u32;
                self.next_phase = Some(next);
                // [G] b51 ends the phase here; without it PHAS only declares
                // the next one.
                if bit(w, 51) {
                    return Err(Stop::Phase {
                        pc,
                        next: Some(next),
                    });
                }
            }
            Op::PhaseEnd => {
                return Err(Stop::Phase {
                    pc,
                    next: self.next_phase.take(),
                });
            }
            Op::Emit => {
                let mut sources = [[0; 2]; 3];
                for (s, arg) in sources.iter_mut().zip(&a[1..4]) {
                    *s = match arg.op {
                        Operand::Imm { value, .. } => [value as u32, 0],
                        // The pair's second register is the "next repeat".
                        _ => [self.read(arg, 0, pc, w)?, self.read(arg, 1, pc, w)?],
                    };
                }
                let emit = Emit {
                    target: field(w, 32, 2) as u8,
                    sources,
                    imm: self.read(&a[4], 0, pc, w)?,
                    word: w,
                };
                bus.emit(&emit)
                    .map_err(|error| Stop::Fault { pc, addr: 0, error })?;
            }
            Op::Smlsi => {
                for (s, arg) in self.smlsi.iter_mut().zip(a) {
                    if let Operand::Imm { value, .. } = arg.op {
                        *s = value as i32;
                    }
                }
            }
            Op::Vector => return Err(unimpl(pc, w, "vector floating point")),
            Op::Opaque => return Err(unimpl(pc, w, "unknown semantics")),
        }
        Ok(())
    }

    /// The test form (§5.3): run the ALU op, write the result back unless
    /// discarded, and set the predicate from the test.
    fn test(&mut self, insn: &Insn, pc: u32) -> Result<(), Stop> {
        let w = insn.word;
        let a = insn.args();
        let x = self.read(&a[2], 0, pc, w)?;
        let y = self.read(&a[a.len() - 1], 0, pc, w)?;
        let r = match insn.test_subop() {
            0x30 => x & y,
            0x31 => x | y,
            0x32 => x ^ y,
            0x33 => bitwise_op(BitOp::Shl, x, y).unwrap_or(0),
            0x34 => bitwise_op(BitOp::Shr, x, y).unwrap_or(0),
            0x35 => bitwise_op(BitOp::Rol, x, y).unwrap_or(0),
            0x37 => bitwise_op(BitOp::Asr, x, y).unwrap_or(0),
            // [G] The multiply-adds test with an implied multiplier of 1:
            // `src1 ± src2`, the `n` variants subtracting.
            0x16 | 0x19 | 0x1c | 0x1d | 0x1e | 0x1f => x.wrapping_add(y),
            0x17 | 0x1a => x.wrapping_sub(y),
            _ => return Err(unimpl(pc, w, "test sub-op")),
        };
        let t = test_bits(w);
        if t & 0xe0 != 0 {
            return Err(unimpl(pc, w, "test channel select"));
        }
        // [strong] t[1:0]: 1 negative, 2 non-negative; t[3:2]: 1 zero,
        // 2 non-zero; t4 combines with AND, else OR. An absent test is the
        // combination's identity.
        let sign = match t & 3 {
            0 => None,
            1 => Some((r as i32) < 0),
            2 => Some((r as i32) >= 0),
            _ => return Err(unimpl(pc, w, "test sign mode 3")),
        };
        let zero = match t >> 2 & 3 {
            0 => None,
            1 => Some(r == 0),
            2 => Some(r != 0),
            _ => return Err(unimpl(pc, w, "test zero mode 3")),
        };
        let p = if t & 0x10 != 0 {
            sign.unwrap_or(true) && zero.unwrap_or(true)
        } else {
            sign.unwrap_or(false) || zero.unwrap_or(false)
        };
        self.write(&a[0], 0, r, pc, w)?;
        if let Operand::Pred(n) = a[1].op {
            self.preds[usize::from(n)] = p;
        }
        Ok(())
    }

    /// LD and ST (§6.1, §6.2).
    fn memory(&mut self, bus: &mut dyn UsseBus, insn: &Insn, pc: u32) -> Result<(), Stop> {
        let w = insn.word;
        let a = insn.args();
        if bit(w, 42) || bit(w, 43) {
            return Err(unimpl(pc, w, "load/store address space"));
        }
        let size = match field(w, 36, 2) {
            0 => Size::Dword,
            1 => Size::Word,
            2 => Size::Byte,
            _ => return Err(unimpl(pc, w, "qword load/store")),
        };
        let load = insn.op() == Op::Load;
        let (base, offset) = if load { (&a[1], &a[2]) } else { (&a[0], &a[1]) };
        let base = self.read(base, 0, pc, w)?;
        let offset = self.read(offset, 0, pc, w)?;
        for rep in 0..insn.repeat {
            // [G] The offset counts elements of the access size, and steps by
            // one per repeat; the addressing-mode bits (b40, b41, b53) are
            // not applied.
            let addr = base.wrapping_add(
                offset
                    .wrapping_add(u32::from(rep))
                    .wrapping_mul(size.bytes()),
            );
            if load {
                let v = bus
                    .load(addr, size)
                    .map_err(|error| Stop::Fault { pc, addr, error })?;
                self.write(&a[0], rep, v, pc, w)?;
            } else {
                let v = self.read(&a[2], rep, pc, w)?;
                bus.store(addr, size, v)
                    .map_err(|error| Stop::Fault { pc, addr, error })?;
            }
        }
        Ok(())
    }
}

fn unimpl(pc: u32, word: u64, why: &'static str) -> Stop {
    Stop::Unimplemented { pc, word, why }
}

/// A bitwise or shift operation; `None` for `RLP`, whose semantics are
/// unknown.
fn bitwise_op(op: BitOp, x: u32, y: u32) -> Option<u32> {
    // [G] Shift counts are taken modulo 32.
    let n = y & 31;
    Some(match op {
        BitOp::And => x & y,
        BitOp::Or => x | y,
        BitOp::Xor => x ^ y,
        BitOp::Shl => x << n,
        BitOp::Shr => x >> n,
        BitOp::Asr => ((x as i32) >> n) as u32,
        BitOp::Rol => x.rotate_left(n),
        BitOp::Rlp => return None,
    })
}

#[cfg(test)]
mod tests;
