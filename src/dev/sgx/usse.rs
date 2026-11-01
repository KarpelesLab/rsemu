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
//! in the 64-bit output recorded. Our notes on that measurement
//! (`docs/devices/sgx-usse-encoding.md`, cited below by section, `§n`; it
//! replaces the lost `ENCODING.md` and records the method well enough to
//! repeat it) grade every claim:
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
    /// The vector forms' four 128-bit internal registers, `v0`–`v3` (§8),
    /// numbered here by 32-bit component: `v<k>` is number `4k`.
    Internal,
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
            Bank::Internal => "v",
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
    /// A blend factor (§11): the encoder's selector value — `0` zero, `2`
    /// saturate, `4`/`5`/`6` the colour of `src0`/`src1`/`src2`, `7`/`8`/`9`
    /// their alpha, `24`/`25` unknown — and whether it is complemented
    /// (`1 - f`).
    Factor {
        /// The selector value.
        value: u8,
        /// `1 - f`.
        comp: bool,
    },
    /// A blend operation (§11): the encoder's `18` add, `19` subtract, `22`
    /// and `23` (min and max \[G\]).
    BlendOp(u8),
    /// A named field value (a `pck` format, §8.6).
    Text(&'static str),
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
    /// Which operand slot the field is — [`SLOT_DST`], [`SLOT_SRC0`],
    /// [`SLOT_SRC1`], [`SLOT_SRC2`], or [`SLOT_NONE`] — and so which of the
    /// `SETBASE` offsets applies to it (§9).
    pub slot: u8,
    /// A vector source's swizzle: per result component, the source
    /// component (`0`–`3`), or `4`/`5` for the constants 0.0/1.0 \[G\] (§8).
    pub swz: Option<[u8; 4]>,
    /// A vector source's absolute-value modifier (§8).
    pub abs: bool,
    /// A vector destination's write mask, bit `k` for component `k` (§8).
    pub mask: Option<u8>,
}

/// The destination slot (`b27:21`).
pub const SLOT_DST: u8 = 0;
/// The `src0` slot (`b20:14`).
pub const SLOT_SRC0: u8 = 1;
/// The `src1` slot (`b13:7`).
pub const SLOT_SRC1: u8 = 2;
/// The `src2` slot (`b6:0`).
pub const SLOT_SRC2: u8 = 3;
/// Not a register slot.
pub const SLOT_NONE: u8 = 4;

impl Arg {
    const NONE: Arg = Arg::of(Operand::None);

    const fn of(op: Operand) -> Arg {
        Arg {
            op,
            neg: false,
            inv: false,
            hi: false,
            slot: SLOT_NONE,
            swz: None,
            abs: false,
            mask: None,
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
    /// A predicate code the encoder produces but whose meaning is not
    /// known (the vector forms' `6`, §8).
    Unknown(u8),
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

    /// The three-bit field of vector ops 1–3 and 6–8 (§8): `0 none, 1 p0,
    /// 2 p1, 3 p2, 4 !p0, 5 !p1, 6 ?, 7 pN`.
    fn vector3(v: u64) -> Pred {
        match v {
            0 => Pred::Always,
            1..=3 => Pred::If(v as u8 - 1),
            4 => Pred::IfNot(0),
            5 => Pred::IfNot(1),
            6 => Pred::Unknown(6),
            _ => Pred::PerInstance,
        }
    }

    /// `vmad`'s two bits (§8, `OPTABLE`): `0 none, 1 p0, 2 !p0,
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
            Pred::Unknown(v) => write!(f, "?{v}"),
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

/// Which blend unit form (§11).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlendForm {
    /// Opcode 16: separate colour and alpha factors and operations.
    Split,
    /// Opcode 18: one factor pair, a colour and an alpha operation, and a
    /// byte write mask.
    Masked,
}

/// Which vector form (§8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VecOp {
    /// Op 0, `src0 · src1 + src2` on four F32 components.
    Mad,
    /// Op 1: an F32 two-source operation, sub-op `b14:12`.
    Op1,
    /// Op 3: an operation on a general source and two internal registers,
    /// sub-op `b53:52` (or `b54:52`).
    Op3,
    /// Op 7: a vector move, variant `b47:46`.
    Op7,
    /// Op 8: a lane format conversion.
    Pck,
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
    /// Set the four register-number base offsets (§9) \[strong\].
    SetBase,
    /// A texture sample (op 28, §12): handed to [`UsseBus::sample`].
    Sample,
    /// A per-byte blend (ops 16 and 18, §11).
    Blend(BlendForm),
    /// A vector floating-point instruction this interpreter executes (§8).
    Vec(VecOp),
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
    /// Three bits, vector ops 1–3 and 6–8.
    Vector3(u32),
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
    /// A two's-complement immediate bit field.
    SField(u32, u32),
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
    /// A blend factor: complement bit, code field (low bit, width) and the
    /// code-to-selector table (§11).
    Factor(u32, u32, u32, &'static [u8]),
    /// A blend operation: a 2-bit field of codes 0–3 meaning selectors 18,
    /// 19, 22, 23 (§11).
    BlendOp(u32),
    /// Opcode 17's 1-bit operation: 18 or 19 (§11).
    BlendOp1(u32),
    /// A vector destination (§8): a 6-bit field at `lo` counting 64-bit
    /// registers (`60`–`63` in the temporaries are `v0`–`v3`), the standard
    /// bank bits, and a write mask at `mask`.
    VDst { lo: u32, mask: u32 },
    /// A vector source: a 6-bit field at `lo` with the bank bits of `bank`,
    /// its swizzle, and the negate and absolute-value bits.
    VSrc {
        lo: u32,
        bank: VBank,
        swz: Swz,
        neg: Option<u32>,
        abs: Option<u32>,
    },
    /// An op-3 internal-register source: a 2-bit field and its swizzle.
    VInt {
        lo: u32,
        swz: Swz,
        neg: Option<u32>,
        abs: Option<u32>,
    },
    /// The `pck` destination: the standard destination and a lane mask at
    /// `b37:34`.
    PckDst,
    /// The `pck` source: `src1` with its four per-lane selects.
    PckSrc,
    /// A `pck` format field.
    PckFmt(u32),
}

/// Which bank bits a vector source uses (§8).
#[derive(Clone, Copy, Debug)]
enum VBank {
    /// `b34`: primary attribute, else temporary.
    Src0,
    /// `b31:30` with the extension bit `b49`.
    S1,
    /// `b29:28` with the extension bit `b48`.
    S2,
}

/// How a vector source's swizzle is encoded (§8).
#[derive(Clone, Copy, Debug)]
enum Swz {
    /// An index — its bits listed low first — into a table.
    Table(&'static [u32], &'static [[u8; 4]]),
    /// `vop` `src1`: a 3-bit code per component.
    Comp12,
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
    /// Computed from the word.
    Dyn(fn(u64) -> &'static str),
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
/// The EMIT forms other than 162: the encoder flips only END (b43),
/// flags1.17 (b50) and flags3.20 (b21) (§10).
const F_EMIT_B: &[Flag] = &[
    Flag("syncs?", b(50)),
    Flag("end", b(43)),
    Flag("f3.20", b(21)),
];
const F_SYNCS51: &[Flag] = &[Flag("syncs?", b(51))];
/// Op 28 (§12): the encoder's flags1.0/.2/.3/.4 and flags3.16/.2/.3.
const F_SMP: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("end", b(54)),
    Flag("f1.4", b(53)),
    Flag("f3.16", b(51)),
    Flag("f3.2", b(37)),
    Flag("f3.3", b(38)),
];
/// The blend family's header (§11).
const F_SOP: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
];
const F_BLEND2: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
    Flag("m26", b(20)),
    Flag("am26", b(15)),
    Flag("arev", b(14)),
];
const F_BLEND3: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("end", b(54)),
    Flag("b45", b(45)),
    Flag("b44", b(44)),
    Flag("b46", b(46)),
    Flag("b43", b(43)),
];
/// Op 23: b54 is set mid-program in the corpus, so not END there \[G\].
const F_OP23: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("syncs?", b(50)),
    Flag("b54", b(54)),
];
const F_SMLSI: &[Flag] = &[Flag("end", b(50))];
/// The vector forms (§8). The encoder puts its END flag at b43, but the
/// vertex shaders set b43 on nearly every vector instruction with live code
/// after it, so it is printed as `b43` and does not end the program \[G\].
const F_VEC: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("b43", b(43)),
];
/// Op 3: b52 is part of its sub-op.
const F_VOP3: &[Flag] = &[Flag("skipinv", b(55)), Flag("b43", b(43))];
const F_VMAD: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("b43", b(43)),
    Flag("f16?", b(58)),
];
/// `pck`: b54 (the encoder's END) is set mid-program too; b18 \[G: the
/// normalising scale\], b53/b48/b50 are seen in the corpus only.
const F_PCK: &[Flag] = &[
    Flag("skipinv", b(55)),
    Flag("nosched", b(52)),
    Flag("b54", b(54)),
    Flag("norm", b(18)),
    Flag("b53", b(53)),
    Flag("b50", b(50)),
    Flag("b48", b(48)),
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
        neg: Some(Neg(39, true)),
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
        neg: Some(Neg(39, true)),
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
// §11: the factor code tables, code -> encoder selector value.
const FT_SPLIT_RGB1: &[u8] = &[0, 5, 6, 8, 9, 2, 24];
const FT_SPLIT_RGB2: &[u8] = &[0, 5, 6, 8, 9, 2, 25];
const FT_SPLIT_A1: &[u8] = &[0, 8, 9, 24];
const FT_SPLIT_A2: &[u8] = &[0, 8, 9, 25];
const FT_SHARED: &[u8] = &[0, 2, 5, 8, 4, 7, 6, 9];
const O_BLEND2: &[Opd] = &[
    Opd::Dst,
    S1,
    S2,
    Opd::Factor(56, 38, 3, FT_SPLIT_RGB1),
    Opd::Factor(47, 35, 3, FT_SPLIT_RGB2),
    Opd::BlendOp(18),
    Opd::Factor(43, 52, 2, FT_SPLIT_A1),
    Opd::Factor(34, 41, 2, FT_SPLIT_A2),
    Opd::BlendOp(16),
];
const O_BLENDM: &[Opd] = &[
    Opd::Dst,
    S1,
    S2,
    Opd::Factor(56, 38, 3, FT_SHARED),
    Opd::Factor(47, 35, 3, FT_SHARED),
    Opd::BlendOp(52),
    Opd::BlendOp(41),
    Opd::Field(43, 4),
];
const O_BLEND3: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad { hi: false },
    S1,
    S2,
    Opd::Factor(56, 38, 3, FT_SHARED),
    Opd::Factor(47, 35, 3, FT_SHARED),
    Opd::BlendOp1(41),
    Opd::Field(52, 2),
];
const O_OP23: &[Opd] = &[
    Opd::Dst,
    Opd::Src0Mad { hi: false },
    S1,
    S2,
    Opd::Field(40, 2),
    Opd::Field(38, 2),
    Opd::Field(35, 3),
    Opd::Field(52, 2),
];
const O_SRC3: &[Opd] = &[Opd::Dst, Opd::Src0Mad { hi: false }, S1, S2];
/// §12: destination (b39 primary attribute), coordinates at `src0` and
/// sampler state at `src1` — both register pairs, as EMIT's — and a DRC.
const O_SMP: &[Opd] = &[
    Opd::DstMem,
    Opd::EmitSrc(None),
    Opd::EmitSrc(Some(Slot::S1)),
    Opd::Drc,
];
const O_DST2: &[Opd] = &[Opd::Dst, S1, S2];
/// §9: useasm arguments 0–3 land in b47:36, b35:24, b23:12, b11:0.
const O_FOUR12: &[Opd] = &[
    Opd::Field(36, 12),
    Opd::Field(24, 12),
    Opd::Field(12, 12),
    Opd::Field(0, 12),
];
const O_SETBASE: &[Opd] = O_FOUR12;
const O_FOUR12S: &[Opd] = &[
    Opd::SField(36, 12),
    Opd::SField(24, 12),
    Opd::SField(12, 12),
    Opd::SField(0, 12),
];
const O_OP193: &[Opd] = &[Opd::Field(0, 1), Opd::Field(8, 1)];
const O_VMAD: &[Opd] = &[
    Opd::VDst { lo: 22, mask: 39 },
    Opd::VSrc {
        lo: 12,
        bank: VBank::Src0,
        swz: Swz::Table(&[18, 19, 53], &vector::SW_MAD0),
        neg: Some(37),
        abs: Some(50),
    },
    Opd::VSrc {
        lo: 6,
        bank: VBank::S1,
        swz: Swz::Table(&[20, 21, 44], &vector::SW_MAD1),
        neg: None,
        abs: Some(38),
    },
    Opd::VSrc {
        lo: 0,
        bank: VBank::S2,
        swz: Swz::Table(&[45, 46, 47], &vector::SW_MAD2),
        neg: Some(35),
        abs: Some(36),
    },
];
const O_VOP: &[Opd] = &[
    Opd::VDst { lo: 22, mask: 39 },
    Opd::VSrc {
        lo: 6,
        bank: VBank::S1,
        swz: Swz::Comp12,
        neg: Some(37),
        abs: Some(38),
    },
    Opd::VSrc {
        lo: 0,
        bank: VBank::S2,
        swz: Swz::Table(&[44, 45, 46, 47], &vector::SW16),
        neg: None,
        abs: Some(36),
    },
];
const O_VOP3: &[Opd] = &[
    Opd::VDst { lo: 22, mask: 39 },
    Opd::VSrc {
        lo: 0,
        bank: VBank::S1,
        swz: Swz::Table(&[6, 7, 8, 9, 10], &vector::SW32),
        neg: Some(38),
        abs: Some(37),
    },
    Opd::VInt {
        lo: 28,
        swz: Swz::Table(&[18, 19, 20, 21, 34], &vector::SW32),
        neg: None,
        abs: Some(46),
    },
    Opd::VInt {
        lo: 12,
        swz: Swz::Table(&[14, 15, 16, 17, 54], &vector::SW32),
        neg: Some(36),
        abs: Some(35),
    },
    Opd::Field(47, 2),
];
const O_VOP7: &[Opd] = &[
    Opd::VDst { lo: 18, mask: 24 },
    Opd::VSrc {
        lo: 12,
        bank: VBank::Src0,
        swz: Swz::Table(&[], &[vector::XYZW]),
        neg: None,
        abs: None,
    },
    Opd::VSrc {
        lo: 6,
        bank: VBank::S1,
        swz: Swz::Table(&[35, 36, 37, 38], &vector::SW16),
        neg: None,
        abs: None,
    },
    Opd::VSrc {
        lo: 0,
        bank: VBank::S2,
        swz: Swz::Table(&[], &[vector::XYZW]),
        neg: None,
        abs: None,
    },
    Opd::Field(40, 2),
];
// The destination's format at b40:38, the source's at b43:41 \[strong\]:
// the UI's vertex shader unpacks s16 screen positions into f32 with
// b43:41 = 4 and b40:38 = 6, and the matrix after it needs the floats.
const O_PCK: &[Opd] = &[Opd::PckFmt(38), Opd::PckFmt(41), Opd::PckDst, Opd::PckSrc];

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
    // §9: the rest of class 1. Sub-ops 1, 3 and 4 take four 12-bit fields,
    // sub-op 5 two single bits; only sub-op 3 has known semantics.
    class1(1, "op189", O_FOUR12, Op::Opaque),
    class1(3, "setbase", O_SETBASE, Op::SetBase),
    class1(4, "op192", O_FOUR12S, Op::Opaque),
    class1(5, "op193", O_OP193, Op::Opaque),
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
    // EMIT family (§10): b47:46 pick the form. The microkernel's has b47
    // set, b46 clear (useasm 162); useasm 158 clears both; 159–161 set b46
    // and pick a sub-form with b45:44. Every form carries the same operand
    // fields — the encoder merely leaves some at fixed values (`#0`).
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
    emit(0, "emit158"),
    emit(b(46), "emit159"),
    emit(b(46) | b(44), "emit160"),
    emit(b(46) | b(45), "emit161"),
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
        // [G] The encoder puts END at b43 here, but the microkernel sets it
        // on two back-to-back register writes with live code after them
        // (f2013a40), so on STR it cannot end the program. It still prints.
        end: None,
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
    // -- op 28: texture sampling (§12); b43:42 is useasm 111/112/113.
    Form {
        mask: OPCODE,
        value: op(28),
        name: Name::Dyn(smp_name),
        inverted: 0,
        pred: PredEnc::Long(56),
        rpt: RptEnc::Field(44, 2),
        end: Some(54),
        flags: F_SMP,
        ops: O_SMP,
        exec: Op::Sample,
    },
    // -- ops 16–25: the per-byte blend family (§11).
    sop(
        16,
        "blend2",
        F_BLEND2,
        RptEnc::Field(44, 3),
        O_BLEND2,
        Op::Blend(BlendForm::Split),
    ),
    sop(17, "blend3", F_BLEND3, RptEnc::None, O_BLEND3, Op::Opaque),
    sop(
        18,
        "blend",
        F_SOP,
        RptEnc::None,
        O_BLENDM,
        Op::Blend(BlendForm::Masked),
    ),
    sop(19, "op19", F_SOP, RptEnc::Field(44, 3), O_SRC3, Op::Opaque),
    sop(22, "op22", F_SOP, RptEnc::Field(44, 3), O_DST2, Op::Opaque),
    Form {
        end: None,
        ..sop(23, "op23", F_OP23, RptEnc::Field(44, 2), O_OP23, Op::Opaque)
    },
    sop(24, "op24", F_SOP, RptEnc::Field(44, 3), O_DST2, Op::Opaque),
    sop(25, "op25", F_SOP, RptEnc::Field(44, 3), O_SRC3, Op::Opaque),
    // -- vector floating point (§8): decoded to a name only.
    Form {
        flags: F_VMAD,
        exec: Op::Vec(VecOp::Mad),
        ..vector(0, 0, 0, "vmad", O_VMAD)
    },
    Form {
        name: Name::Dyn(vop1_name),
        pred: PredEnc::Vector3(56),
        exec: Op::Vec(VecOp::Op1),
        ..vector(1, 0, 0, "", O_VOP)
    },
    Form {
        name: Name::Dyn(vop2_name),
        pred: PredEnc::Vector3(56),
        ..vector(2, 0, 0, "", O_VOP)
    },
    Form {
        name: Name::Dyn(vop3_name),
        pred: PredEnc::Vector3(56),
        flags: F_VOP3,
        exec: Op::Vec(VecOp::Op3),
        ..vector(3, 0, 0, "", O_VOP3)
    },
    Form {
        pred: PredEnc::Vector3(56),
        ..vector(6, 0, 0, "vcplx", &[])
    },
    Form {
        name: Name::Dyn(vop7_name),
        pred: PredEnc::Vector3(56),
        rpt: RptEnc::Field(44, 2),
        exec: Op::Vec(VecOp::Op7),
        ..vector(7, 0, 0, "", O_VOP7)
    },
    Form {
        pred: PredEnc::Vector3(56),
        flags: F_PCK,
        exec: Op::Vec(VecOp::Pck),
        ..vector(8, 0, 0, "pck", O_PCK)
    },
];

/// Op 28's variant, `b43:42` (§12): useasm 111, 112, 113 \[G: the
/// dimensionality\].
fn smp_name(w: u64) -> &'static str {
    ["smp1", "smp2", "smp3", "smp.3"][field(w, 42, 2) as usize]
}

/// Op 1's sub-ops, `b14:12` (§8.4): 0 multiply \[strong\], 1 add \[G\],
/// the rest unknown.
fn vop1_name(w: u64) -> &'static str {
    [
        "vmul", "vadd", "vop1.2", "vop1.3", "vop1.4", "vop1.5", "vop1.6", "vop1.7",
    ][field(w, 12, 3) as usize]
}

/// Op 2: op 1's sub-ops on F16 \[strong: it takes a four-component mask on
/// a 64-bit register\].
fn vop2_name(w: u64) -> &'static str {
    [
        "vmul16", "vadd16", "vop2.2", "vop2.3", "vop2.4", "vop2.5", "vop2.6", "vop2.7",
    ][field(w, 12, 3) as usize]
}

/// Op 3's sub-op (§8.3): `b53:52` when it is 2 or 3 (b54 then belongs to
/// a swizzle), else `b54:52`. 3 is the multiply-add \[strong\].
fn vop3_sub(w: u64) -> u64 {
    match field(w, 52, 2) {
        s @ (2 | 3) => s,
        _ => field(w, 52, 3),
    }
}

fn vop3_name(w: u64) -> &'static str {
    match vop3_sub(w) {
        3 => "vmad3",
        2 => "vop3.2",
        4 => "vop3.4",
        5 => "vop3.5",
        _ => "vop3.?",
    }
}

/// Op 7's variant, `b47:46` (§8.5): 0 a move \[strong\].
fn vop7_name(w: u64) -> &'static str {
    ["vmov", "vop7.1", "vop7.2", "vop7.3"][field(w, 46, 2) as usize]
}

/// A blend-family form (§11): the shared header — short predicate b58:57,
/// END b54 — and the given repeat field.
const fn sop(
    hw: u64,
    name: &'static str,
    flags: &'static [Flag],
    rpt: RptEnc,
    ops: &'static [Opd],
    exec: Op,
) -> Form {
    Form {
        mask: OPCODE,
        value: op(hw),
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::Short(57),
        rpt,
        end: Some(54),
        flags,
        ops,
        exec,
    }
}

/// An EMIT form other than 162 (§10): `sub` is its b47:44.
const fn emit(sub: u64, name: &'static str) -> Form {
    Form {
        mask: OPCODE | CLASS | SUB | 0xf << 44,
        value: class(2) | 3 << 56 | sub,
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(43),
        flags: F_EMIT_B,
        ops: O_EMIT,
        exec: Op::Emit,
    }
}

/// A class-1 control form (§9): `sub` is b58:56.
const fn class1(sub: u64, name: &'static str, ops: &'static [Opd], exec: Op) -> Form {
    Form {
        mask: OPCODE | CLASS | SUB,
        value: class(1) | sub << 56,
        name: Name::Fixed(name),
        inverted: 0,
        pred: PredEnc::None,
        rpt: RptEnc::None,
        end: Some(50),
        flags: F_SMLSI,
        ops,
        exec,
    }
}

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
        end: None,
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

    /// Whether the instruction carries the flag the reference disassembler
    /// calls `syncs?` (useasm flags1.17), which ends the phase once it has
    /// executed \[G\]. The evidence is the microkernel's own: of its nine
    /// uses, every one is either a `PHAS`/opcode 358 or the instruction
    /// straight after a `PHAS`, and the code after each is either another
    /// entry point or unreachable.
    pub fn ends_phase(&self) -> bool {
        self.form
            .flags
            .iter()
            .any(|f| f.0 == "syncs?" && self.word & f.1 == f.1)
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
        PredEnc::Vector3(lo) => Pred::vector3(field(word, lo, 3)),
    };
    let repeat = match form.rpt {
        RptEnc::None => 1,
        RptEnc::Field(lo, width) => field(word, lo, width) as u8 + 1,
    };
    let mut args = [Arg::NONE; MAX_ARGS];
    let mut nargs = 0;
    for opd in form.ops {
        if let Some(arg) = decode_opd(*opd, word) {
            args[nargs] = Arg {
                slot: opd_slot(*opd),
                ..arg
            };
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

/// `pck`'s 7-bit fields reach `v0`–`v3` as `r124`–`r127`, as the
/// encoder numbers them (§8.6).
fn internal_alias(op: Operand) -> Operand {
    match op {
        Operand::Reg {
            bank: Bank::Temp,
            num: n @ 124..=127,
        } => reg(Bank::Internal, 4 * u64::from(n - 124)),
        op => op,
    }
}

/// A vector 6-bit register field as a destination (§8): 64-bit registers,
/// the temporaries' `60`–`63` being `v0`–`v3`.
fn vreg_dst(w: u64, f: u64) -> Operand {
    match dst_std(w) {
        Operand::Reg {
            bank: Bank::Temp, ..
        } if f >= 60 => reg(Bank::Internal, 4 * (f - 60)),
        Operand::Reg {
            bank: bank @ (Bank::Temp | Bank::Output | Bank::Primary | Bank::Secondary),
            ..
        } => reg(bank, 2 * f),
        Operand::Reg { bank, .. } => reg(bank, f),
        Operand::Indexed { bank, index, .. } => Operand::Indexed {
            bank,
            index,
            offset: f as u8,
        },
        op => op,
    }
}

/// A vector 6-bit register field as a source (§8).
fn vreg_src(w: u64, f: u64, bank: VBank) -> Operand {
    let (bank, ext) = match bank {
        VBank::Src0 => (if bit(w, 34) { 2 } else { 0 }, false),
        VBank::S1 => (field(w, 30, 2), bit(w, 49)),
        VBank::S2 => (field(w, 28, 2), bit(w, 48)),
    };
    match (ext, bank) {
        (false, 0) if f >= 60 => reg(Bank::Internal, 4 * (f - 60)),
        (false, b) => reg(Bank::INDEXABLE[b as usize], 2 * f),
        (true, 1) => reg(Bank::Special, f),
        (true, 2) => hex(f),
        // [G] Not seen from the encoder.
        (true, b) => indexed(if b == 0 { 1 } else { 2 }, f as u8),
    }
}

/// Decodes a swizzle field.
fn swizzle(w: u64, swz: Swz) -> [u8; 4] {
    match swz {
        Swz::Table(bits, table) => {
            let i = bits
                .iter()
                .enumerate()
                .fold(0usize, |i, (k, &b)| i | usize::from(bit(w, b)) << k);
            table[i % table.len()]
        }
        Swz::Comp12 => {
            const COMP: [[u32; 3]; 4] = [[15, 16, 17], [18, 19, 20], [21, 34, 35], [50, 53, 54]];
            COMP.map(|bits| {
                bits.iter()
                    .enumerate()
                    .fold(0u8, |c, (k, &b)| c | u8::from(bit(w, b)) << k)
            })
        }
    }
}

/// Which register slot an operand encoding occupies (§9).
fn opd_slot(opd: Opd) -> u8 {
    match opd {
        Opd::Dst | Opd::DstTest | Opd::DstMem => SLOT_DST,
        Opd::Src0Mad { .. } | Opd::Src0Mad26 | Opd::Src0Mem(_) | Opd::EmitSrc(None) => SLOT_SRC0,
        Opd::Src { slot: Slot::S1, .. }
        | Opd::TestS1
        | Opd::TestDup
        | Opd::EmitSrc(Some(Slot::S1)) => SLOT_SRC1,
        Opd::Src { slot: Slot::S2, .. } | Opd::TestS2 | Opd::EmitSrc(Some(Slot::S2)) => SLOT_SRC2,
        _ => SLOT_NONE,
    }
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
        Opd::SField(lo, width) => {
            let v = field(w, lo, width) as i64;
            let sign = 1i64 << (width - 1);
            Arg::of(Operand::Imm {
                value: (v ^ sign) - sign,
                style: ImmStyle::Hex,
            })
        }
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
        Opd::Factor(comp, lo, width, table) => Arg::of(Operand::Factor {
            value: table
                .get(field(w, lo, width) as usize)
                .copied()
                .unwrap_or(0xff),
            comp: bit(w, comp),
        }),
        Opd::BlendOp(lo) => Arg::of(Operand::BlendOp([18, 19, 22, 23][field(w, lo, 2) as usize])),
        Opd::BlendOp1(n) => Arg::of(Operand::BlendOp(18 + field(w, n, 1) as u8)),
        Opd::VDst { lo, mask } => Arg {
            mask: Some(field(w, mask, 4) as u8),
            ..Arg::of(vreg_dst(w, field(w, lo, 6)))
        },
        Opd::VSrc {
            lo,
            bank,
            swz,
            neg,
            abs,
        } => Arg {
            swz: Some(swizzle(w, swz)),
            neg: neg.is_some_and(|n| bit(w, n)),
            abs: abs.is_some_and(|n| bit(w, n)),
            ..Arg::of(vreg_src(w, field(w, lo, 6), bank))
        },
        Opd::VInt { lo, swz, neg, abs } => Arg {
            swz: Some(swizzle(w, swz)),
            neg: neg.is_some_and(|n| bit(w, n)),
            abs: abs.is_some_and(|n| bit(w, n)),
            ..Arg::of(reg(Bank::Internal, 4 * field(w, lo, 2)))
        },
        Opd::PckDst => Arg {
            mask: Some(field(w, 34, 4) as u8),
            ..Arg::of(internal_alias(dst_std(w)))
        },
        Opd::PckSrc => Arg {
            swz: Some([
                // [G] With b54 set, b1:0 is not lane 0's select and lane 0
                // reads x. Both corpus uses are `f32 <- f32` copies of a
                // vertex position into `v0` whose b1:0 is 2; only an `xyzw`
                // copy feeds the matrix that follows a position.
                if bit(w, 54) { 0 } else { field(w, 0, 2) as u8 },
                field(w, 16, 2) as u8,
                field(w, 14, 2) as u8,
                field(w, 19, 2) as u8,
            ]),
            ..Arg::of(internal_alias(src_slot(w, Slot::S1, Imm::U7)))
        },
        Opd::PckFmt(lo) => Arg::of(Operand::Text(vector::Fmt::of(field(w, lo, 3)).name())),
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
            Operand::Reg {
                bank: Bank::Internal,
                num,
            } => write!(f, "v{}", num / 4),
            Operand::Reg { bank, num } => write!(f, "{}{num}", bank.prefix()),
            Operand::Text(t) => f.write_str(t),
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
            Operand::Factor { value, comp } => {
                let name = match value {
                    0 if comp => return f.write_str("one"),
                    0 => "zero",
                    2 => "sat",
                    4 => "s0",
                    5 => "s1",
                    6 => "s2",
                    7 => "s0.a",
                    8 => "s1.a",
                    9 => "s2.a",
                    24 => "f24",
                    25 => "f25",
                    _ => "f?",
                };
                write!(f, "{}{name}", if comp { "1-" } else { "" })
            }
            Operand::BlendOp(v) => f.write_str(match v {
                18 => "add",
                19 => "sub",
                22 => "min?",
                23 => "max?",
                _ => "op?",
            }),
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
            Name::Dyn(name) => f.write_str(name(w))?,
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
            if arg.abs {
                f.write_str("|")?;
            }
            self.write_operand(f, arg.op)?;
            if arg.abs {
                f.write_str("|")?;
            }
            if let Some(m) = arg.mask {
                f.write_str(if m == 0 { "._" } else { "." })?;
                for (k, c) in ["x", "y", "z", "w"].iter().enumerate() {
                    if m >> k & 1 != 0 {
                        f.write_str(c)?;
                    }
                }
            }
            if let Some(sw) = arg.swz {
                f.write_str(".")?;
                for c in sw {
                    f.write_str(["x", "y", "z", "w", "0", "1", "?", "?"][usize::from(c & 7)])?;
                }
            }
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
/// immediate, with the raw word for whatever is not decoded yet (§10).
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
    /// The form, `b47:44`: [`EMIT_158`], [`EMIT_159`], [`EMIT_160`],
    /// [`EMIT_161`], or `8`–`11` for the microkernel's useasm 162 (whose
    /// `b45`/`b44` are flags).
    pub form: u8,
    /// Where `src0` resolves to — bank and register number, after `SETBASE`
    /// and index registers — when it names a register. The bursts the
    /// shaders end with (`emit159`/`emit160` from `o0` after filling
    /// `o0..oN`) send the registers from here on \[G\]; how many is not in
    /// the instruction, so the bus is handed the whole engine through
    /// [`UsseBus::emit_from`].
    pub src0_reg: Option<(Bank, u32)>,
}

/// `Emit::form` of useasm 158: four operands and an immediate.
pub const EMIT_158: u8 = 0;
/// `Emit::form` of useasm 159: `b46`.
pub const EMIT_159: u8 = 4;
/// `Emit::form` of useasm 160: `b46 | b44`.
pub const EMIT_160: u8 = 5;
/// `Emit::form` of useasm 161: `b46 | b45`.
pub const EMIT_161: u8 = 6;

/// A texture sample request (op 28, §12). The coordinates and the sampler
/// state are what the instruction's `src0` and `src1` name — four
/// registers from each, since how many the unit reads is not known; the
/// bus decodes them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    /// `b43:42`: the useasm variant (111 + n).
    pub variant: u8,
    /// Four registers from `src0`.
    pub coords: [u32; 4],
    /// Four registers from `src1`.
    pub state: [u32; 4],
    /// The repeat count.
    pub repeat: u8,
    /// The instruction word.
    pub word: u64,
}

/// What a sample returns: up to four words for the destination registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Texel {
    /// The words, from the destination register on.
    pub words: [u32; 4],
    /// How many of them to write.
    pub count: u8,
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

    /// Samples a texture for op 28 (§12). `None`, the default, stops the
    /// engine with [`Stop::Unimplemented`]: texture fetch is the caller's.
    fn sample(&mut self, req: &Sample) -> Option<MemResult<Texel>> {
        let _ = req;
        None
    }

    /// Takes an `EMIT` with the issuing engine's registers in view, for the
    /// forms that send a run of registers rather than their three operands
    /// (§10). The default forwards to [`UsseBus::emit`].
    fn emit_from(&mut self, emit: &Emit, usse: &Usse) -> MemResult {
        let _ = usse;
        self.emit(emit)
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
    /// The phase ended: opcode 358, or any instruction carrying `syncs?`
    /// ([`Insn::ends_phase`]) \[G\]. `next` is the entry the program declared
    /// for its next phase with `PHAS`, if any — `Some(0)` is what the
    /// microkernel declares before ending a task for good, `None` (an opcode
    /// 358 with nothing declared) leaves the choice to the hardware. It is
    /// left as encoded because its base is not settled: the slave
    /// microkernel's `PHAS` targets count from the main image's base, while
    /// its branches count from its own (§7).
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
    /// The vector internal registers `v0`–`v3`, four 32-bit components
    /// each (§8).
    pub internal: [u32; 16],
    /// The predicates `p0`–`p3`.
    pub preds: [bool; 4],
    /// The link register, an instruction index.
    pub link: u32,
    /// The last `SMLSI`'s fields, kept but not yet applied \[G\].
    pub smlsi: [i32; 11],
    /// The register-number offsets `SETBASE` sets, by slot: destination,
    /// `src0`, `src1`, `src2` (§9).
    pub base: [u32; 4],
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
            .field("base", &self.base)
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
            internal: [0; 16],
            preds: [false; 4],
            link: 0,
            smlsi: [0; 11],
            base: [0; 4],
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
            Pred::Unknown(_) => return Err(unimpl(pc, word, "predicate code")),
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
        if enabled && insn.ends_phase() {
            return Err(Stop::Phase {
                pc,
                next: self.next_phase.take(),
            });
        }
        Ok(())
    }

    fn loc(&self, op: Operand, slot: u8, rep: u8, pc: u32, word: u64) -> Result<Loc, Stop> {
        let bad = Stop::BadRegister { pc, word };
        let rep = usize::from(rep);
        let (bank, n) = match op {
            // §9 [strong]: a SETBASE offset moves every directly-numbered
            // register in its slot, temporaries as much as outputs. [G] It
            // leaves the special bank, the index registers and the indexed
            // forms alone.
            Operand::Reg { bank, num } => {
                let base = match bank {
                    Bank::Temp | Bank::Output | Bank::Primary | Bank::Secondary => {
                        self.base.get(usize::from(slot)).copied().unwrap_or(0) as usize
                    }
                    Bank::Special | Bank::Index | Bank::Internal => 0,
                };
                (bank, usize::from(num) + base + rep)
            }
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
            Bank::Internal => self.internal.len(),
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
            Bank::Internal => self.internal[n],
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
            Bank::Internal => self.internal[n] = v,
        }
        Ok(())
    }

    /// An operand's value on repeat iteration `rep`. \[G\] Every register
    /// operand steps by one register per iteration; immediates do not.
    fn read(&self, arg: &Arg, rep: u8, pc: u32, word: u64) -> Result<u32, Stop> {
        let v = match arg.op {
            Operand::Imm { value, .. } => value as u32,
            op => self.get(self.loc(op, arg.slot, rep, pc, word)?),
        };
        let v = if arg.hi { v >> 16 } else { v };
        let v = if arg.inv { !v } else { v };
        Ok(if arg.neg { v.wrapping_neg() } else { v })
    }

    fn write(&mut self, arg: &Arg, rep: u8, v: u32, pc: u32, word: u64) -> Result<(), Stop> {
        match arg.op {
            Operand::Discard => Ok(()),
            op => {
                let loc = self.loc(op, arg.slot, rep, pc, word)?;
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
            // PHAS only declares the next phase; its own `syncs?` bit (b51),
            // like that bit on any instruction, ends the current one (`step`).
            Op::Phas => self.next_phase = Some(field(w, 0, 20) as u32),
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
                let src0_reg = match a[1].op {
                    Operand::Imm { .. } => None,
                    op => {
                        let Loc(bank, n) = self.loc(op, a[1].slot, 0, pc, w)?;
                        Some((bank, n as u32))
                    }
                };
                let emit = Emit {
                    target: field(w, 32, 2) as u8,
                    sources,
                    imm: self.read(&a[4], 0, pc, w)?,
                    word: w,
                    form: field(w, 44, 4) as u8,
                    src0_reg,
                };
                bus.emit_from(&emit, self)
                    .map_err(|error| Stop::Fault { pc, addr: 0, error })?;
            }
            Op::SetBase => {
                for (b, arg) in self.base.iter_mut().zip(a) {
                    if let Operand::Imm { value, .. } = arg.op {
                        *b = value as u32;
                    }
                }
            }
            Op::Smlsi => {
                for (s, arg) in self.smlsi.iter_mut().zip(a) {
                    if let Operand::Imm { value, .. } = arg.op {
                        *s = value as i32;
                    }
                }
            }
            Op::Blend(form) => {
                for rep in 0..insn.repeat {
                    let x = self.read(&a[1], rep, pc, w)?;
                    let y = self.read(&a[2], rep, pc, w)?;
                    let old = match form {
                        BlendForm::Masked => self.read(&a[0], rep, pc, w).unwrap_or(0),
                        BlendForm::Split => 0,
                    };
                    let r = blend(insn, form, x, y, old).ok_or(unimpl(pc, w, "blend selector"))?;
                    self.write(&a[0], rep, r, pc, w)?;
                }
            }
            Op::Vec(v) => self.vector(insn, v, pc)?,
            Op::Sample => {
                let four = |arg: &Arg| -> Result<[u32; 4], Stop> {
                    let mut r = [0; 4];
                    for (i, v) in r.iter_mut().enumerate() {
                        *v = match arg.op {
                            Operand::Imm { value, .. } if i == 0 => value as u32,
                            Operand::Imm { .. } => 0,
                            _ => self.read(arg, i as u8, pc, w)?,
                        };
                    }
                    Ok(r)
                };
                let req = Sample {
                    variant: field(w, 42, 2) as u8,
                    coords: four(&a[1])?,
                    state: four(&a[2])?,
                    repeat: insn.repeat,
                    word: w,
                };
                let texel = bus
                    .sample(&req)
                    .ok_or(unimpl(pc, w, "texture sample"))?
                    .map_err(|error| Stop::Fault { pc, addr: 0, error })?;
                for i in 0..texel.count.min(4) {
                    self.write(&a[0], i, texel.words[usize::from(i)], pc, w)?;
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

    /// Component `c` of a vector source, after its swizzle and modifiers
    /// (§8), as F32 bits.
    fn vsrc(&self, arg: &Arg, c: usize, step: u8, pc: u32, w: u64) -> Result<u32, Stop> {
        let sw = arg.swz.unwrap_or(vector::XYZW)[c];
        let v = match (sw, arg.op) {
            // [G] The encoder's two extra codes as the constants 0 and 1.
            (4, _) => 0,
            (5, _) => 1.0f32.to_bits(),
            (6.., _) => return Err(unimpl(pc, w, "swizzle code")),
            (_, Operand::Imm { .. }) => return Err(unimpl(pc, w, "vector immediate")),
            (sw, op) => self.get(self.loc(op, arg.slot, sw + step, pc, w)?),
        };
        let v = if arg.abs { v & 0x7fff_ffff } else { v };
        Ok(if arg.neg { v ^ 0x8000_0000 } else { v })
    }

    /// Writes component `c` of a vector destination.
    fn vdst(&mut self, arg: &Arg, c: usize, step: u8, v: u32, pc: u32, w: u64) -> Result<(), Stop> {
        let loc = self.loc(arg.op, arg.slot, c as u8 + step, pc, w)?;
        self.set(loc, v, pc, w)
    }

    /// The vector forms (§8). A 64-bit register operand reads its four
    /// components from four consecutive 32-bit registers, and a destination
    /// writes component `k` to the `k`-th; `v0`–`v3` hold four each.
    /// Arithmetic is IEEE single precision, a multiply-add rounding twice
    /// \[G\].
    fn vector(&mut self, insn: &Insn, v: VecOp, pc: u32) -> Result<(), Stop> {
        let w = insn.word;
        let a = insn.args();
        let f = f32::from_bits;
        let mask = a[0].mask.unwrap_or(0xf);
        if v == VecOp::Pck {
            return self.pck(insn, pc);
        }
        let steps = match v {
            VecOp::Op7 => insn.repeat,
            _ => 1,
        };
        for rep in 0..steps {
            // [G] Each repeat moves the registers on by one 64-bit register.
            let step = 2 * rep;
            let mut out = [None; 4];
            for (c, o) in out.iter_mut().enumerate() {
                if mask >> c & 1 == 0 {
                    continue;
                }
                let r = match v {
                    VecOp::Mad if bit(w, 58) => return Err(unimpl(pc, w, "vmad variant b58")),
                    VecOp::Mad => {
                        // b37 negates the product: it is decoded onto src0.
                        let x = f(self.vsrc(&a[1], c, 0, pc, w)?);
                        let y = f(self.vsrc(&a[2], c, 0, pc, w)?);
                        let z = f(self.vsrc(&a[3], c, 0, pc, w)?);
                        x * y + z
                    }
                    VecOp::Op1 => {
                        let x = f(self.vsrc(&a[1], c, 0, pc, w)?);
                        let y = f(self.vsrc(&a[2], c, 0, pc, w)?);
                        match field(w, 12, 3) {
                            0 => x * y,
                            1 => x + y,
                            _ => return Err(unimpl(pc, w, "vector sub-op")),
                        }
                    }
                    VecOp::Op3 => {
                        // [G] Sub-op 2 is taken as a multiply-add too. Its
                        // one corpus use (the UI compositor's vertex shader)
                        // forms the texture coordinate from a constant
                        // (0, 0, 0, 1), `v0.zzww` and `v0`: only `a·b + c`
                        // passes the transformed coordinate through.
                        if !matches!(vop3_sub(w), 2 | 3) {
                            return Err(unimpl(pc, w, "vector sub-op"));
                        }
                        let x = f(self.vsrc(&a[1], c, 0, pc, w)?);
                        let y = f(self.vsrc(&a[2], c, 0, pc, w)?);
                        let z = f(self.vsrc(&a[3], c, 0, pc, w)?);
                        x * y + z
                    }
                    VecOp::Op7 => {
                        if field(w, 46, 2) != 0 {
                            return Err(unimpl(pc, w, "vector move variant"));
                        }
                        f(self.vsrc(&a[2], c, step, pc, w)?)
                    }
                    VecOp::Pck => unreachable!("handled above"),
                };
                *o = Some(r.to_bits());
            }
            // All sources are read before any component is written.
            for (c, o) in out.iter().enumerate() {
                if let Some(r) = *o {
                    self.vdst(&a[0], c, step, r, pc, w)?;
                }
            }
        }
        Ok(())
    }

    /// `pck` (§8.6): each destination lane the mask enables takes the source
    /// lane its select names, converted between the two formats.
    fn pck(&mut self, insn: &Insn, pc: u32) -> Result<(), Stop> {
        use vector::Fmt;
        let w = insn.word;
        let a = insn.args();
        let (dfmt, sfmt) = (Fmt::of(field(w, 38, 3)), Fmt::of(field(w, 41, 3)));
        let norm = bit(w, 18);
        let sel = a[3].swz.unwrap_or(vector::XYZW);
        let mask = a[2].mask.unwrap_or(0);
        let mut vals = [0u32; 4];
        for k in 0..4 {
            if mask >> k & 1 == 0 {
                continue;
            }
            let (word, shift, width) =
                sfmt.lane(u32::from(sel[k]))
                    .ok_or(unimpl(pc, w, "pck format"))?;
            let src = match a[3].op {
                Operand::Imm { value, .. } => value as u32,
                op => self.get(self.loc(op, a[3].slot, word as u8, pc, w)?),
            };
            let lane = if width == 32 {
                src
            } else {
                src >> shift & ((1 << width) - 1)
            };
            vals[k] = dfmt.encode(sfmt.decode(lane, norm), norm);
        }
        for (k, v) in vals.iter().enumerate() {
            if mask >> k & 1 == 0 {
                continue;
            }
            let (word, shift, width) = dfmt.lane(k as u32).ok_or(unimpl(pc, w, "pck format"))?;
            let loc = self.loc(a[2].op, a[2].slot, word as u8, pc, w)?;
            let new = if width == 32 {
                *v
            } else {
                let m = ((1u32 << width) - 1) << shift;
                self.get(loc) & !m | v << shift & m
            };
            self.set(loc, new, pc, w)?;
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
        let (base_arg, offset) = if load { (&a[1], &a[2]) } else { (&a[0], &a[1]) };
        let base = self.read(base_arg, 0, pc, w)?;
        let offset = self.read(offset, 0, pc, w)?;
        // [G] b41 (`f1.28`) writes the base register back. With b53 set
        // (the encoder's default, so `f1.31` does not print) the access is
        // at the base itself and the base then advances by the offset: the
        // microkernel reads a command slot's handler PC with
        // `ldad.f1.28 r4, r0, #1` before the slot's remaining words from
        // the advanced `r0`, and walks lists with `ldad.f1.28 rN, rP, #1`.
        // With b53 clear (`f1.31`) the access is at base + offset as usual
        // and the base advances by the repeat count: its pair-copy loops
        // (`ldad.f1.28.f1.31.rpt2 r5, r3, #0` against an end pointer of
        // `r3 + 8n`, and `stad.f1.28.f1.31.rpt2 r1, #0x52, r5` filling
        // consecutive pairs) never advance the base any other way.
        let (writeback, pre) = (bit(w, 41), !bit(w, 53));
        let index = if writeback && !pre { 0 } else { offset };
        for rep in 0..insn.repeat {
            // [G] The offset counts elements of the access size, and every
            // repeat steps one element; b40 is not applied.
            let addr = base.wrapping_add(
                index
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
        if writeback {
            let step = if pre { u32::from(insn.repeat) } else { offset };
            let next = base.wrapping_add(step.wrapping_mul(size.bytes()));
            self.write(base_arg, 0, next, pc, w)?;
        }
        Ok(())
    }
}

/// One blend factor for byte `k` of `x` (`src1`) against `y` (`src2`), as a
/// 0–255 weight (§11). `None` for the selectors with no known meaning, or
/// that name `src0`, which these forms do not have. \[G\] Byte 3 is alpha.
fn blend_factor(sel: Operand, k: usize, x: [u8; 4], y: [u8; 4]) -> Option<u32> {
    let Operand::Factor { value, comp } = sel else {
        return None;
    };
    let f = match value {
        0 => 0,
        // GL's SRC_ALPHA_SATURATE: min(As, 1 - Ad) on colour, one on alpha.
        2 if k == 3 => 255,
        2 => x[3].min(255 - y[3]),
        5 => x[k],
        6 => y[k],
        8 => x[3],
        9 => y[3],
        _ => return None,
    };
    Some(u32::from(if comp { 255 - f } else { f }))
}

/// A unorm8 product, rounded to nearest \[G\].
fn mul8(a: u8, f: u32) -> u32 {
    (u32::from(a) * f + 127) / 255
}

/// Combines byte `k` (§11): `x·f1 OP y·f2`, saturated to 0–255. \[G\] 22 and
/// 23 are taken as GL's MIN and MAX, which ignore the factors; `rev` swaps
/// the subtraction (opcode 16's `b14`).
fn blend_byte(op: Operand, rev: bool, x: u8, fx: u32, y: u8, fy: u32) -> Option<u8> {
    let (p, q) = (mul8(x, fx), mul8(y, fy));
    let (p, q) = if rev { (q, p) } else { (p, q) };
    Some(match op {
        Operand::BlendOp(18) => (p + q).min(255) as u8,
        Operand::BlendOp(19) => p.saturating_sub(q) as u8,
        Operand::BlendOp(22) => x.min(y),
        Operand::BlendOp(23) => x.max(y),
        _ => return None,
    })
}

/// Opcodes 16 and 18 (§11): per byte, `src1·f1 OP src2·f2`. Opcode 16 has
/// separate alpha factors and operation; opcode 18 shares its factors,
/// applies its alpha operation to byte 3 and writes only the bytes its
/// mask (`b46:43`, `b43 + k` for byte `k` \[G\]) enables, `old` keeping the
/// rest.
fn blend(insn: &Insn, form: BlendForm, x: u32, y: u32, old: u32) -> Option<u32> {
    let a = insn.args();
    let w = insn.word;
    let (xb, yb, ob) = (x.to_le_bytes(), y.to_le_bytes(), old.to_le_bytes());
    let mut out = [0u8; 4];
    for k in 0..4 {
        let alpha = k == 3;
        let (f1, f2, op, rev) = match form {
            BlendForm::Split if alpha => (a[6].op, a[7].op, a[8].op, bit(w, 14)),
            BlendForm::Split => (a[3].op, a[4].op, a[5].op, false),
            BlendForm::Masked => (
                a[3].op,
                a[4].op,
                if alpha { a[6].op } else { a[5].op },
                false,
            ),
        };
        if form == BlendForm::Split && (bit(w, 20) || bit(w, 15)) {
            return None;
        }
        if form == BlendForm::Masked && !bit(w, 43 + k as u32) {
            out[k] = ob[k];
            continue;
        }
        let fx = blend_factor(f1, k, xb, yb)?;
        let fy = blend_factor(f2, k, xb, yb)?;
        out[k] = blend_byte(op, rev, xb[k], fx, yb[k], fy)?;
    }
    Some(u32::from_le_bytes(out))
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

mod vector;

#[cfg(test)]
mod tests;
