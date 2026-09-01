//! The MIPS I instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! [`TABLE`] is the only description of the instruction set in this core, and
//! both [`decode`] and [`disasm`](super::disasm) read from it.
//!
//! # Decoding
//!
//! Every row is a (mask, match) pair, which is how the manual's opcode maps
//! are actually organised: a six-bit primary opcode, and for `SPECIAL` a
//! six-bit function code, for `REGIMM` a five-bit `rt`, and for `COP0` a
//! five-bit `rs`. [`INDEX`] buckets the rows by the primary opcode at compile
//! time and [`decode`] scans only that bucket — at most about twenty rows for
//! `SPECIAL` and one or two for everything else. The index is built by a
//! `const fn` from `TABLE` itself, so it cannot fall out of step with it.
//!
//! Masks are **lenient about fields the hardware ignores**. `ADD` requires
//! `sa == 0` in the assembler's grammar, but the silicon decodes the primary
//! opcode and the function code and nothing else, so an `ADD` encoding with a
//! non-zero `sa` executes as `ADD` here too. Pinning bits the hardware does
//! not look at would invent reserved-instruction exceptions that no R3000 ever
//! raises.
//!
//! # Per-entry requirements, not a version number
//!
//! `ROADMAP.md` §6.1.1: decode is gated **per table entry** against the
//! configured part, and an instruction the part lacks must trap rather than
//! execute. [`Req`] is that gate. It matters immediately, because "R3000A
//! compatible" is a family and not a version: the LSI LR33300 has no TLB at
//! all, so `TLBWI` on one is a reserved instruction and not a slow no-op, and
//! a `COP2` instruction on a part with no GTE raises coprocessor-unusable with
//! `Cause.CE = 2` — which is exactly how a guest probes for one.
//!
//! # Why there is no cycle column
//!
//! Same reason the other cores have none: a cycle is charged *because* a bus
//! access happened, not because a table says an instruction takes four of
//! them. The R3000 is a five-stage pipeline with a one-cycle-per-instruction
//! ideal and stalls that depend on the cache and the memory system, none of
//! which is a property of the ISA. What the interpreter counts is accesses,
//! which is a fact about the machine being modelled (CLAUDE.md, "CPU cores").
//!
//! # Sources
//!
//! Gerry Kane and Joe Heinrich, *MIPS RISC Architecture* (Prentice Hall) —
//! the MIPS I instruction descriptions and the encoding tables in Appendix A —
//! and the *IDT R3051/R3052/R3081 Family Hardware User's Manual* for the
//! processor-control instructions and their operand forms. Field positions are
//! transcribed from the three instruction-format figures (I, J and R type).

use core::fmt;

/// Which extra capability an encoding needs from the configured part.
///
/// The `ROADMAP.md` §6.1.1 gate, and the reason it is an enum on the row
/// rather than a `#[cfg]`: one build of rsemu has to decode for an R3000A
/// *and* for an LR33300 in the same process, and the two do not have the same
/// instruction set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Req {
    /// Present on every MIPS I part.
    Base,
    /// A coprocessor-0 instruction: usable in kernel mode unconditionally, and
    /// in user mode only when `Status.CU0` is set.
    Cop0,
    /// A coprocessor-0 instruction that also needs a **TLB**. Absent on the
    /// LR33300, which has the segment mapping and nothing else.
    Tlb,
    /// A coprocessor-1 instruction (the R3010 floating-point accelerator).
    Cop1,
    /// A coprocessor-2 instruction (on the LR33300, the GTE).
    Cop2,
    /// A coprocessor-3 instruction.
    Cop3,
}

impl Req {
    /// Which coprocessor this requirement names, if any.
    ///
    /// Decides the exception a failed requirement raises: a missing
    /// coprocessor is *coprocessor unusable* with `Cause.CE` naming it, while
    /// a missing TLB is a plain reserved instruction. Reporting the wrong one
    /// breaks feature probing, which is the thing the gate exists to get
    /// right.
    #[must_use]
    pub const fn coprocessor(self) -> Option<u32> {
        match self {
            Req::Cop0 | Req::Tlb => Some(0),
            Req::Cop1 => Some(1),
            Req::Cop2 => Some(2),
            Req::Cop3 => Some(3),
            Req::Base => None,
        }
    }

    /// A short name, for `rsemu describe` and the monitor.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Req::Base => "mips1",
            Req::Cop0 => "cop0",
            Req::Tlb => "tlb",
            Req::Cop1 => "cop1",
            Req::Cop2 => "cop2",
            Req::Cop3 => "cop3",
        }
    }
}

/// How an instruction's operands are laid out, and therefore how the
/// disassembler prints it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Fmt {
    /// `rd, rs, rt` — the three-register arithmetic and logical form.
    R,
    /// `rd, rt, sa` — a shift by a literal amount.
    Shift,
    /// `rd, rt, rs` — a shift by a register. The operand order really is
    /// reversed from [`Fmt::R`]; the shifted value is `rt`.
    ShiftV,
    /// `rt, rs, imm` — the immediate arithmetic and logical form.
    I,
    /// `rt, imm(rs)` — a load or a store.
    Mem,
    /// `rt, imm` — `LUI`, which has no source register.
    Lui,
    /// `rs, rt, target` — a two-register conditional branch.
    Branch,
    /// `rs, target` — a branch against zero.
    BranchZ,
    /// `target` — `J` and `JAL`, whose target is formed from the delay slot's
    /// program counter.
    Jump,
    /// `rs` — `JR`, whose target is a register.
    Rs,
    /// `rs` — `MTHI` and `MTLO`. Prints like [`Fmt::Rs`] and is a separate
    /// shape because it is *not* a control transfer, and the delay-slot
    /// question is answered from this field.
    MoveTo,
    /// `rd, rs` — `JALR`, whose link register defaults to `ra`.
    JumpLink,
    /// `rd` — `MFHI`, `MFLO`.
    Rd,
    /// `rs, rt` — `MULT` and `DIV`, whose results go to `HI` and `LO`.
    HiLo,
    /// `code` — `SYSCALL` and `BREAK`, which carry a 20-bit literal the
    /// handler reads back out of the instruction.
    Code,
    /// `rt, cp0reg` — `MFC0` and `MTC0`, which name a CP0 register by number.
    Cop0Move,
    /// No operands at all — `TLBR`, `TLBWI`, `TLBWR`, `TLBP`, `RFE`.
    None,
    /// `cofun` — a coprocessor operation this core does not implement, printed
    /// as its raw 25-bit function so a listing is still readable.
    CopFun,
    /// `rt, imm(rs)` naming a coprocessor register — `LWCz` and `SWCz`.
    CopMem,
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
    /// What the configured part must have for this to decode.
    pub req: Req,
}

impl Insn {
    /// Whether this instruction transfers control, and therefore has a delay
    /// slot after it.
    ///
    /// Asked by the disassembler, which marks the following line, and by the
    /// interpreter's own test that every branch in the table sets a target.
    #[must_use]
    pub const fn is_branch(self) -> bool {
        matches!(
            self.fmt,
            Fmt::Branch | Fmt::BranchZ | Fmt::Jump | Fmt::Rs | Fmt::JumpLink
        )
    }
}

/// Declare the operation enum, its mnemonics, its summaries and the decode
/// table from one list of rows.
macro_rules! isa {
    ($($mask:literal $bits:literal $op:ident $mn:literal $fmt:ident $req:ident $summary:literal;)*) => {
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

        /// The decode table: the only description of the instruction set in
        /// this core.
        pub static TABLE: &[Insn] = &[
            $(Insn {
                op: Op::$op,
                mask: $mask,
                bits: $bits,
                fmt: Fmt::$fmt,
                req: Req::$req,
            },)*
        ];
    };
}

isa! {
    // -- SPECIAL (opcode 0), selected by the six-bit function code ----------
    0xfc00003f 0x00000000 Sll     "sll"     Shift    Base "shift left logical by a literal";
    0xfc00003f 0x00000002 Srl     "srl"     Shift    Base "shift right logical by a literal";
    0xfc00003f 0x00000003 Sra     "sra"     Shift    Base "shift right arithmetic by a literal";
    0xfc00003f 0x00000004 Sllv    "sllv"    ShiftV   Base "shift left logical by a register";
    0xfc00003f 0x00000006 Srlv    "srlv"    ShiftV   Base "shift right logical by a register";
    0xfc00003f 0x00000007 Srav    "srav"    ShiftV   Base "shift right arithmetic by a register";
    0xfc00003f 0x00000008 Jr      "jr"      Rs       Base "jump to a register, with a delay slot";
    0xfc00003f 0x00000009 Jalr    "jalr"    JumpLink Base "jump to a register and link, with a delay slot";
    0xfc00003f 0x0000000c Syscall "syscall" Code     Base "raise a system-call exception";
    0xfc00003f 0x0000000d Break   "break"   Code     Base "raise a breakpoint exception";
    0xfc00003f 0x00000010 Mfhi    "mfhi"    Rd       Base "move from the HI register";
    0xfc00003f 0x00000011 Mthi    "mthi"    MoveTo   Base "move to the HI register";
    0xfc00003f 0x00000012 Mflo    "mflo"    Rd       Base "move from the LO register";
    0xfc00003f 0x00000013 Mtlo    "mtlo"    MoveTo   Base "move to the LO register";
    0xfc00003f 0x00000018 Mult    "mult"    HiLo     Base "multiply signed, into HI and LO";
    0xfc00003f 0x00000019 Multu   "multu"   HiLo     Base "multiply unsigned, into HI and LO";
    0xfc00003f 0x0000001a Div     "div"     HiLo     Base "divide signed, quotient in LO and remainder in HI";
    0xfc00003f 0x0000001b Divu    "divu"    HiLo     Base "divide unsigned, quotient in LO and remainder in HI";
    0xfc00003f 0x00000020 Add     "add"     R        Base "add, trapping on signed overflow";
    0xfc00003f 0x00000021 Addu    "addu"    R        Base "add, wrapping";
    0xfc00003f 0x00000022 Sub     "sub"     R        Base "subtract, trapping on signed overflow";
    0xfc00003f 0x00000023 Subu    "subu"    R        Base "subtract, wrapping";
    0xfc00003f 0x00000024 And     "and"     R        Base "bitwise AND";
    0xfc00003f 0x00000025 Or      "or"      R        Base "bitwise OR";
    0xfc00003f 0x00000026 Xor     "xor"     R        Base "bitwise exclusive-OR";
    0xfc00003f 0x00000027 Nor     "nor"     R        Base "bitwise NOR";
    0xfc00003f 0x0000002a Slt     "slt"     R        Base "set if less than, signed";
    0xfc00003f 0x0000002b Sltu    "sltu"    R        Base "set if less than, unsigned";

    // -- REGIMM (opcode 1) --------------------------------------------------
    //
    // Which comparison is made comes from **bit 0** of `rt` alone, so all 32
    // encodings are branches and none of them is reserved: `rt = 11101` is a
    // `BGEZ` on a real R3000 rather than an illegal instruction, and pinning
    // all five bits would invent an exception no hardware raises on 28 of the
    // 32 encodings.
    //
    // The **link** is narrower, and this is the asymmetry: `$31` is written
    // only when `rt` is exactly `10000` or `10001`. So the two linking forms
    // are more specific rows than the two plain ones and are listed first —
    // see [`decode`], which takes the first match, and the table's own test,
    // which requires an overlap to be strictly more specific.
    0xfc1f0000 0x04100000 Bltzal  "bltzal"  BranchZ  Base "branch if less than zero and link";
    0xfc1f0000 0x04110000 Bgezal  "bgezal"  BranchZ  Base "branch if greater than or equal to zero and link";
    0xfc010000 0x04000000 Bltz    "bltz"    BranchZ  Base "branch if less than zero";
    0xfc010000 0x04010000 Bgez    "bgez"    BranchZ  Base "branch if greater than or equal to zero";

    // -- primary opcodes ----------------------------------------------------
    0xfc000000 0x08000000 J       "j"       Jump     Base "jump within the 256 MB region, with a delay slot";
    0xfc000000 0x0c000000 Jal     "jal"     Jump     Base "jump and link within the 256 MB region";
    0xfc000000 0x10000000 Beq     "beq"     Branch   Base "branch if equal";
    0xfc000000 0x14000000 Bne     "bne"     Branch   Base "branch if not equal";
    0xfc000000 0x18000000 Blez    "blez"    BranchZ  Base "branch if less than or equal to zero";
    0xfc000000 0x1c000000 Bgtz    "bgtz"    BranchZ  Base "branch if greater than zero";
    0xfc000000 0x20000000 Addi    "addi"    I        Base "add an immediate, trapping on signed overflow";
    0xfc000000 0x24000000 Addiu   "addiu"   I        Base "add a sign-extended immediate, wrapping";
    0xfc000000 0x28000000 Slti    "slti"    I        Base "set if less than an immediate, signed";
    0xfc000000 0x2c000000 Sltiu   "sltiu"   I        Base "set if less than a sign-extended immediate, unsigned";
    0xfc000000 0x30000000 Andi    "andi"    I        Base "AND with a zero-extended immediate";
    0xfc000000 0x34000000 Ori     "ori"     I        Base "OR with a zero-extended immediate";
    0xfc000000 0x38000000 Xori    "xori"    I        Base "exclusive-OR with a zero-extended immediate";
    0xfc000000 0x3c000000 Lui     "lui"     Lui      Base "load an immediate into the upper halfword";

    // -- COP0 ---------------------------------------------------------------
    //
    // `rs` selects the operation: 0 is a move out of a CP0 register, 4 a move
    // in, and `rs` bit 4 marks the processor-control group whose function code
    // names the TLB instructions and `RFE`.
    0xffe007ff 0x40000000 Mfc0    "mfc0"    Cop0Move Cop0 "move from a coprocessor-0 register";
    0xffe007ff 0x40800000 Mtc0    "mtc0"    Cop0Move Cop0 "move to a coprocessor-0 register";
    0xfe00003f 0x42000001 Tlbr    "tlbr"    None     Tlb  "read the TLB entry `Index` names";
    0xfe00003f 0x42000002 Tlbwi   "tlbwi"   None     Tlb  "write the TLB entry `Index` names";
    0xfe00003f 0x42000006 Tlbwr   "tlbwr"   None     Tlb  "write the TLB entry `Random` names";
    0xfe00003f 0x42000008 Tlbp    "tlbp"    None     Tlb  "probe the TLB for the entry `EntryHi` names";
    0xfe00003f 0x42000010 Rfe     "rfe"     None     Cop0 "restore from exception: pop the status stack";

    // -- the coprocessors this core does not implement ----------------------
    //
    // Rows rather than a hole in the map, because a hole would decode as a
    // *reserved instruction* and the architecture says these are *coprocessor
    // unusable* — a distinction a guest probing for a GTE can see, and which
    // `ROADMAP.md` §6.1.1 names as the point of gating per entry.
    0xfc000000 0x44000000 Cop1    "cop1"    CopFun   Cop1 "a coprocessor-1 operation";
    0xfc000000 0x48000000 Cop2    "cop2"    CopFun   Cop2 "a coprocessor-2 operation";
    0xfc000000 0x4c000000 Cop3    "cop3"    CopFun   Cop3 "a coprocessor-3 operation";

    // -- loads and stores ---------------------------------------------------
    0xfc000000 0x80000000 Lb      "lb"      Mem      Base "load a sign-extended byte";
    0xfc000000 0x84000000 Lh      "lh"      Mem      Base "load a sign-extended halfword";
    0xfc000000 0x88000000 Lwl     "lwl"     Mem      Base "load the word bytes on the high side of an unaligned address";
    0xfc000000 0x8c000000 Lw      "lw"      Mem      Base "load a word";
    0xfc000000 0x90000000 Lbu     "lbu"     Mem      Base "load a zero-extended byte";
    0xfc000000 0x94000000 Lhu     "lhu"     Mem      Base "load a zero-extended halfword";
    0xfc000000 0x98000000 Lwr     "lwr"     Mem      Base "load the word bytes on the low side of an unaligned address";
    0xfc000000 0xa0000000 Sb      "sb"      Mem      Base "store a byte";
    0xfc000000 0xa4000000 Sh      "sh"      Mem      Base "store a halfword";
    0xfc000000 0xa8000000 Swl     "swl"     Mem      Base "store the word bytes on the high side of an unaligned address";
    0xfc000000 0xac000000 Sw      "sw"      Mem      Base "store a word";
    0xfc000000 0xb8000000 Swr     "swr"     Mem      Base "store the word bytes on the low side of an unaligned address";

    // -- coprocessor loads and stores ---------------------------------------
    0xfc000000 0xc4000000 Lwc1    "lwc1"    CopMem   Cop1 "load a word into a coprocessor-1 register";
    0xfc000000 0xc8000000 Lwc2    "lwc2"    CopMem   Cop2 "load a word into a coprocessor-2 register";
    0xfc000000 0xcc000000 Lwc3    "lwc3"    CopMem   Cop3 "load a word into a coprocessor-3 register";
    0xfc000000 0xe4000000 Swc1    "swc1"    CopMem   Cop1 "store a word from a coprocessor-1 register";
    0xfc000000 0xe8000000 Swc2    "swc2"    CopMem   Cop2 "store a word from a coprocessor-2 register";
    0xfc000000 0xec000000 Swc3    "swc3"    CopMem   Cop3 "store a word from a coprocessor-3 register";
}

/// Where each six-bit primary opcode's rows start and end in [`TABLE`].
///
/// Built from `TABLE` by a `const fn`, so it is a derived cache in the strict
/// sense — it cannot disagree with the table it indexes, and adding a row
/// needs no second edit.
pub static INDEX: [(u16, u16); 64] = build_index(TABLE);

/// Compute [`INDEX`] at compile time.
const fn build_index(table: &[Insn]) -> [(u16, u16); 64] {
    let mut index = [(0u16, 0u16); 64];
    let mut opcode = 0usize;
    while opcode < 64 {
        let mut first = 0u16;
        let mut last = 0u16;
        let mut found = false;
        let mut i = 0usize;
        while i < table.len() {
            if (table[i].bits >> 26) as usize == opcode {
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

/// Decode an instruction word, or `None` if nothing in the table matches.
///
/// The **first** matching row wins, which is how the one place the opcode map
/// genuinely overlaps is expressed: `BLTZAL` and `BGEZAL` are two specific
/// `rt` values inside the range `BLTZ` and `BGEZ` otherwise cover, so they are
/// listed first. [`TABLE`]'s own test requires any overlap to be strictly more
/// specific than the row it precedes, so the order is a checked property
/// rather than a convention.
///
/// Requirements are **not** checked here: the caller knows the configuration
/// and needs to distinguish "no such instruction" (reserved instruction) from
/// "this part does not have that coprocessor" (coprocessor unusable), which is
/// why [`Insn::req`] comes back rather than being consumed.
#[must_use]
pub fn decode(word: u32) -> Option<&'static Insn> {
    let (first, last) = INDEX[(word >> 26) as usize];
    let mut i = first as usize;
    while i < last as usize {
        let insn = &TABLE[i];
        if word & insn.mask == insn.bits {
            return Some(insn);
        }
        i += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// Field extraction
// ---------------------------------------------------------------------------

/// The primary opcode field, bits 31..26.
#[inline]
#[must_use]
pub const fn opcode(word: u32) -> u32 {
    word >> 26
}

/// The first source register field, bits 25..21.
#[inline]
#[must_use]
pub const fn rs(word: u32) -> u32 {
    (word >> 21) & 31
}

/// The second source register field, bits 20..16.
///
/// Also the destination of every load and of the immediate arithmetic — MIPS
/// spells `rt` as a source or a destination depending on the format, which is
/// the one genuinely confusing thing about the encoding.
#[inline]
#[must_use]
pub const fn rt(word: u32) -> u32 {
    (word >> 16) & 31
}

/// The destination register field of an R-type instruction, bits 15..11.
///
/// Doubles as the coprocessor register number in `MFC0`/`MTC0`.
#[inline]
#[must_use]
pub const fn rd(word: u32) -> u32 {
    (word >> 11) & 31
}

/// The literal shift amount, bits 10..6.
#[inline]
#[must_use]
pub const fn sa(word: u32) -> u32 {
    (word >> 6) & 31
}

/// The function code of a `SPECIAL` or processor-control instruction, bits
/// 5..0.
#[inline]
#[must_use]
pub const fn funct(word: u32) -> u32 {
    word & 63
}

/// The 16-bit immediate, zero-extended.
#[inline]
#[must_use]
pub const fn imm(word: u32) -> u32 {
    word & 0xffff
}

/// The 16-bit immediate, sign-extended to 32 bits.
///
/// `ADDIU` and `SLTIU` both sign-extend despite the `U`: the suffix means
/// "does not trap on overflow" and "compares as unsigned" respectively, never
/// "zero-extends". Getting this wrong is the classic MIPS immediate bug.
#[inline]
#[must_use]
pub const fn simm(word: u32) -> u32 {
    ((word & 0xffff) as u16 as i16) as i32 as u32
}

/// The 26-bit jump target field.
#[inline]
#[must_use]
pub const fn target(word: u32) -> u32 {
    word & 0x03ff_ffff
}

/// The 20-bit literal a `SYSCALL` or `BREAK` carries, bits 25..6.
#[inline]
#[must_use]
pub const fn code(word: u32) -> u32 {
    (word >> 6) & 0x000f_ffff
}

/// The 25-bit coprocessor function field, bits 24..0.
#[inline]
#[must_use]
pub const fn cofun(word: u32) -> u32 {
    word & 0x01ff_ffff
}

/// Where a conditional branch goes.
///
/// The displacement is relative to the **delay slot**, not to the branch, and
/// is a count of words. `delay_pc` is the address of the delay slot, which is
/// the branch's own address plus four.
#[inline]
#[must_use]
pub const fn branch_target(delay_pc: u32, word: u32) -> u32 {
    delay_pc.wrapping_add(simm(word) << 2)
}

/// Where a `J` or `JAL` goes.
///
/// The top four bits come from the **delay slot's** program counter, which is
/// why a jump at the very end of a 256 MB region lands in the next one.
#[inline]
#[must_use]
pub const fn jump_target(delay_pc: u32, word: u32) -> u32 {
    (delay_pc & 0xf000_0000) | (target(word) << 2)
}

/// The o32 ABI names of the general registers, in numeric order.
///
/// These are what a disassembler, gdb and the monitor print. `$30` has two
/// names — `s8` and `fp` — and the ABI table lists `s8` first when it is being
/// used as a saved register, so that is the one printed.
pub const REG_NAMES: [&str; 32] = [
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
    "t7", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "t8", "t9", "k0", "k1", "gp", "sp", "s8",
    "ra",
];

/// Look a general register up by ABI or `$N` name.
#[must_use]
pub fn reg_by_name(name: &str) -> Option<u32> {
    let name = name.strip_prefix('$').unwrap_or(name);
    if let Ok(n) = name.parse::<u32>()
        && n < 32
    {
        return Some(n);
    }
    if name == "fp" {
        return Some(30);
    }
    REG_NAMES.iter().position(|n| *n == name).map(|i| i as u32)
}

/// The byte order the core is wired for.
///
/// A **pin** on a real R3000, not a build option: the same die is sold
/// big-endian and little-endian, and the choice changes what `LWL`, `LWR`,
/// `SWL` and `SWR` mean as well as how a halfword sits in a word. So it is a
/// construction property, like every other axis of the family
/// (`ROADMAP.md` §6.1.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Endian {
    /// The low byte of a word lives at the lowest address.
    Little,
    /// The high byte of a word lives at the lowest address.
    Big,
}

impl Endian {
    /// Whether this is big-endian.
    #[inline]
    #[must_use]
    pub const fn is_big(self) -> bool {
        matches!(self, Endian::Big)
    }

    /// The name a `.machine` file uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Endian::Little => "little",
            Endian::Big => "big",
        }
    }
}

impl fmt::Display for Endian {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// The unaligned-transfer byte tables
// ---------------------------------------------------------------------------
//
// Kane & Heinrich state `LWL`/`LWR`/`SWL`/`SWR` as byte tables indexed by the
// low two bits of the effective address, with a separate table per byte order.
// Transcribing four tables and then indexing them is more code than the shifts
// they describe, and gets the same answers — but only if the shift is derived
// rather than guessed, so both are derived here and the tests check every one
// of the eight (four instructions x two byte orders) against the manual's
// tables spelled out as literals.

/// The shift the *left* half of an unaligned transfer uses.
///
/// `LWL` takes the byte at the effective address into the most significant
/// byte of the register and works towards the word boundary; which direction
/// that is in memory depends on the byte order, and that is the whole of the
/// endianness dependence.
#[inline]
#[must_use]
pub const fn unaligned_left_shift(addr: u32, endian: Endian) -> u32 {
    let byte = addr & 3;
    8 * if endian.is_big() { byte } else { 3 - byte }
}

/// The shift the *right* half of an unaligned transfer uses.
#[inline]
#[must_use]
pub const fn unaligned_right_shift(addr: u32, endian: Endian) -> u32 {
    let byte = addr & 3;
    8 * if endian.is_big() { 3 - byte } else { byte }
}

/// Merge an aligned word into a register the way `LWL` does.
#[inline]
#[must_use]
pub const fn lwl(old: u32, word: u32, addr: u32, endian: Endian) -> u32 {
    let k = unaligned_left_shift(addr, endian);
    // A shift of 32 is undefined in Rust and unreachable here: `k` is 0, 8, 16
    // or 24, so `!0 >> k` and `word << k` are always well defined. The keep
    // mask is the bits *below* the shifted-in field.
    let keep = if k == 0 { 0 } else { (1u32 << k) - 1 };
    (word << k) | (old & keep)
}

/// Merge an aligned word into a register the way `LWR` does.
#[inline]
#[must_use]
pub const fn lwr(old: u32, word: u32, addr: u32, endian: Endian) -> u32 {
    let k = unaligned_right_shift(addr, endian);
    (word >> k) | (old & !(u32::MAX >> k))
}

/// Merge a register into an aligned word the way `SWL` does.
#[inline]
#[must_use]
pub const fn swl(word: u32, value: u32, addr: u32, endian: Endian) -> u32 {
    let k = unaligned_left_shift(addr, endian);
    (word & !(u32::MAX >> k)) | (value >> k)
}

/// Merge a register into an aligned word the way `SWR` does.
#[inline]
#[must_use]
pub const fn swr(word: u32, value: u32, addr: u32, endian: Endian) -> u32 {
    let k = unaligned_right_shift(addr, endian);
    (word & !(u32::MAX << k)) | (value << k)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn every_row_is_reachable_through_the_index() {
        for insn in TABLE {
            let found = decode(insn.bits).expect("its own encoding must decode");
            assert_eq!(
                found.op,
                insn.op,
                "{} decoded as {}",
                insn.op.mnemonic(),
                found.op.mnemonic()
            );
        }
    }

    #[test]
    fn an_overlapping_row_is_always_the_more_specific_one_and_comes_first() {
        // Two rows overlap when every bit both of them pin agrees, and the
        // first one wins. That is a silent bug — the second instruction never
        // happens — *unless* the earlier row is strictly more specific, which
        // is exactly how `BLTZAL` and `BGEZAL` sit inside `BLTZ` and `BGEZ`.
        // So the rule is not "no overlaps": it is "an overlap must narrow".
        for (i, a) in TABLE.iter().enumerate() {
            for b in &TABLE[i + 1..] {
                let common = a.mask & b.mask;
                if a.bits & common != b.bits & common {
                    continue;
                }
                assert!(
                    a.mask & b.mask == b.mask && a.mask != b.mask,
                    "{} and {} overlap without the first being more specific",
                    a.op.mnemonic(),
                    b.op.mnemonic()
                );
            }
        }
    }

    #[test]
    fn every_regimm_encoding_is_a_branch_and_only_two_of_them_link() {
        // All 32 `rt` values decode; the comparison comes from bit 0 and the
        // link from the whole field being 10000 or 10001. A model that pinned
        // all five bits would raise a reserved instruction on 28 of them, and
        // one that took the link from bit 4 alone would write `$31` on 16.
        for rt in 0..32u32 {
            let word = 0x0400_0000 | (rt << 16);
            let insn = decode(word).unwrap_or_else(|| panic!("rt = {rt:05b} did not decode"));
            let expected = match rt {
                0x10 => Op::Bltzal,
                0x11 => Op::Bgezal,
                _ if rt & 1 == 0 => Op::Bltz,
                _ => Op::Bgez,
            };
            assert_eq!(insn.op, expected, "rt = {rt:05b}");
        }
    }

    #[test]
    fn every_row_pins_the_bits_its_own_encoding_sets() {
        for insn in TABLE {
            assert_eq!(
                insn.bits & !insn.mask,
                0,
                "{} sets a bit outside its own mask",
                insn.op.mnemonic()
            );
        }
    }

    #[test]
    fn mnemonics_are_unique() {
        let mut seen: Vec<&str> = TABLE.iter().map(|i| i.op.mnemonic()).collect();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(before, seen.len(), "two rows share a mnemonic");
    }

    #[test]
    fn the_all_zero_word_is_a_nop() {
        // `sll $zero, $zero, 0`. Every MIPS assembler emits this for `nop`, so
        // a table that failed to decode it would fail on the most common
        // instruction in any real program.
        let insn = decode(0).expect("the zero word decodes");
        assert_eq!(insn.op, Op::Sll);
    }

    #[test]
    fn immediates_sign_extend_where_the_manual_says_they_do() {
        // `addiu $t0, $t1, -1`
        let word = 0x2400_0000 | (9 << 21) | (8 << 16) | 0xffff;
        assert_eq!(simm(word), 0xffff_ffff);
        assert_eq!(imm(word), 0xffff);
    }

    #[test]
    fn a_jump_takes_its_high_bits_from_the_delay_slot() {
        // A `j` at 0x0fff_fffc has its delay slot at 0x1000_0000, so the
        // target is formed in the *next* 256 MB region.
        let word = 0x0800_0000 | 0x0000_0100;
        assert_eq!(jump_target(0x1000_0000, word), 0x1000_0400);
        assert_eq!(jump_target(0x0fff_fffc, word), 0x0000_0400);
    }

    #[test]
    fn a_branch_displacement_is_relative_to_the_delay_slot() {
        // `beq $zero, $zero, -1` at 0x1000: the delay slot is 0x1004 and the
        // target is 0x1004 - 4 = 0x1000, the branch itself.
        let word = 0x1000_0000 | 0xffff;
        assert_eq!(branch_target(0x1004, word), 0x1000);
    }

    // -- the unaligned tables ----------------------------------------------

    /// Model the manual's byte tables directly.
    ///
    /// `LWL` takes the byte at the effective address into the register's most
    /// significant byte and then walks **towards the most significant end of
    /// the word in memory**, which is the low address on a big-endian part and
    /// the high address on a little-endian one. `LWR` starts at the same byte
    /// and fills the register's least significant end, walking the other way.
    ///
    /// Written from that definition rather than from a shift, so agreeing with
    /// [`lwl`] means the shift was derived correctly rather than that the same
    /// mistake was made twice.
    fn table_transfer(old: u32, mem: [u8; 4], addr: u32, endian: Endian, left: bool) -> u32 {
        let mut out = old.to_be_bytes(); // out[0] is the register's MSB
        let step: isize = match (left, endian.is_big()) {
            (true, true) | (false, false) => 1,
            _ => -1,
        };
        let mut reg: isize = if left { 0 } else { 3 };
        let mut at = (addr & 3) as isize;
        while (0..4).contains(&at) {
            out[reg as usize] = mem[at as usize];
            reg += if left { 1 } else { -1 };
            at += step;
        }
        u32::from_be_bytes(out)
    }

    /// The bytes of an aligned word as the configured byte order stores them,
    /// indexed by offset within the word.
    fn word_bytes(word: u32, endian: Endian) -> [u8; 4] {
        if endian.is_big() {
            word.to_be_bytes()
        } else {
            word.to_le_bytes()
        }
    }

    /// The inverse of [`word_bytes`].
    fn bytes_word(bytes: [u8; 4], endian: Endian) -> u32 {
        if endian.is_big() {
            u32::from_be_bytes(bytes)
        } else {
            u32::from_le_bytes(bytes)
        }
    }

    #[test]
    fn the_unaligned_loads_match_the_manuals_byte_tables() {
        let old = 0xaabb_ccdd;
        let word = 0x0123_4567;
        for endian in [Endian::Big, Endian::Little] {
            let mem = word_bytes(word, endian);
            for byte in 0..4u32 {
                let addr = 0x1000 + byte;
                assert_eq!(
                    lwl(old, word, addr, endian),
                    table_transfer(old, mem, addr, endian, true),
                    "lwl {endian} at +{byte}"
                );
                assert_eq!(
                    lwr(old, word, addr, endian),
                    table_transfer(old, mem, addr, endian, false),
                    "lwr {endian} at +{byte}"
                );
            }
        }
    }

    #[test]
    fn a_store_and_the_matching_load_round_trip_the_register() {
        // The two halves of each pair describe the same set of bytes from
        // opposite ends, so storing a register through one and loading it back
        // through the same one has to be the identity — whatever was in memory
        // first, at every offset, in both byte orders.
        for endian in [Endian::Big, Endian::Little] {
            for byte in 0..4u32 {
                let addr = 0x2000 + byte;
                for value in [0x1122_3344u32, 0, u32::MAX] {
                    for mem in [0x5566_7788u32, 0, u32::MAX] {
                        assert_eq!(
                            lwl(value, swl(mem, value, addr, endian), addr, endian),
                            value,
                            "swl/lwl at +{byte}, {endian}"
                        );
                        assert_eq!(
                            lwr(value, swr(mem, value, addr, endian), addr, endian),
                            value,
                            "swr/lwr at +{byte}, {endian}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn each_half_covers_exactly_the_bytes_the_manual_gives_it() {
        // Storing all-ones into all-zeroes makes the covered byte set visible,
        // and the counts are the manual's: the left half runs from the
        // effective address to the most significant end of the word, and the
        // right half from the same address to the least significant end — so
        // the two counts always sum to five, overlapping in the byte they
        // share.
        for endian in [Endian::Big, Endian::Little] {
            for byte in 0..4u32 {
                let addr = 0x2000 + byte;
                let covered = |w: u32| w.to_be_bytes().iter().filter(|b| **b != 0).count();
                let left = covered(swl(0, u32::MAX, addr, endian));
                let right = covered(swr(0, u32::MAX, addr, endian));
                let (want_l, want_r) = if endian.is_big() {
                    (4 - byte as usize, byte as usize + 1)
                } else {
                    (byte as usize + 1, 4 - byte as usize)
                };
                assert_eq!(left, want_l, "swl at +{byte}, {endian}");
                assert_eq!(right, want_r, "swr at +{byte}, {endian}");
            }
        }
    }

    #[test]
    fn the_store_pair_writes_an_unaligned_word_across_two_aligned_ones() {
        // The compiler idiom for an unaligned 32-bit store, and the mirror of
        // the load-pair test: `swl` at one end of the four bytes and `swr` at
        // the other, which between them touch exactly those four and nothing
        // else. Memory is modelled as bytes so "nothing else" is checkable.
        for endian in [Endian::Big, Endian::Little] {
            for k in 0..4u32 {
                let mut mem = [0xffu8; 8];
                let base = 0x1000 + k;
                let (left_at, right_at) = if endian.is_big() {
                    (base, base + 3)
                } else {
                    (base + 3, base)
                };
                let read = |mem: &[u8; 8], addr: u32| -> u32 {
                    let off = ((addr & !3) - 0x1000) as usize;
                    bytes_word([mem[off], mem[off + 1], mem[off + 2], mem[off + 3]], endian)
                };
                let write = |mem: &mut [u8; 8], addr: u32, w: u32| {
                    let off = ((addr & !3) - 0x1000) as usize;
                    mem[off..off + 4].copy_from_slice(&word_bytes(w, endian));
                };
                let value = 0x1122_3344u32;
                let w = read(&mem, left_at);
                write(&mut mem, left_at, swl(w, value, left_at, endian));
                let w = read(&mem, right_at);
                write(&mut mem, right_at, swr(w, value, right_at, endian));

                let k = k as usize;
                let got = bytes_word([mem[k], mem[k + 1], mem[k + 2], mem[k + 3]], endian);
                assert_eq!(got, value, "unaligned store at +{k}, {endian}");
                for (i, b) in mem.iter().enumerate() {
                    if i < k || i > k + 3 {
                        assert_eq!(*b, 0xff, "byte {i} was written and should not have been");
                    }
                }
            }
        }
    }

    #[test]
    fn the_load_pair_reads_an_unaligned_word_out_of_two_aligned_ones() {
        // The idiom every MIPS compiler emits for an unaligned 32-bit load:
        // `lwl rt, k(base)` then `lwr rt, k+3(base)` on a big-endian part, and
        // the two addresses swapped on a little-endian one. Model eight bytes
        // of memory and check the register against the four bytes really there.
        let mem: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];
        for endian in [Endian::Big, Endian::Little] {
            let w0 = bytes_word([mem[0], mem[1], mem[2], mem[3]], endian);
            let w1 = bytes_word([mem[4], mem[5], mem[6], mem[7]], endian);
            let word_at = |addr: u32| if addr < 0x1004 { w0 } else { w1 };
            for k in 0..4u32 {
                let base = 0x1000 + k;
                let (left_at, right_at) = if endian.is_big() {
                    (base, base + 3)
                } else {
                    (base + 3, base)
                };
                let mut rt = 0xdead_beefu32;
                rt = lwl(rt, word_at(left_at), left_at, endian);
                rt = lwr(rt, word_at(right_at), right_at, endian);
                let want = bytes_word(
                    [
                        mem[k as usize],
                        mem[k as usize + 1],
                        mem[k as usize + 2],
                        mem[k as usize + 3],
                    ],
                    endian,
                );
                assert_eq!(rt, want, "unaligned load at +{k}, {endian}");
            }
        }
    }

    #[test]
    fn a_transfer_at_the_word_boundary_moves_the_whole_word() {
        assert_eq!(lwl(0, 0x1234_5678, 0x100, Endian::Big), 0x1234_5678);
        assert_eq!(lwl(0, 0x1234_5678, 0x103, Endian::Little), 0x1234_5678);
        assert_eq!(lwr(0, 0x1234_5678, 0x103, Endian::Big), 0x1234_5678);
        assert_eq!(lwr(0, 0x1234_5678, 0x100, Endian::Little), 0x1234_5678);
    }

    #[test]
    fn a_transfer_at_the_far_end_moves_exactly_one_byte() {
        // Big-endian `lwl` at +3 reads the word's low byte into the register's
        // high byte and leaves the other three alone.
        assert_eq!(
            lwl(0x0000_00ff, 0x1234_5678, 0x103, Endian::Big),
            0x7800_00ff
        );
        assert_eq!(
            lwl(0x0000_00ff, 0x1234_5678, 0x100, Endian::Little),
            0x7800_00ff
        );
        // And the store side: big-endian `swr` at +0 writes only the byte at
        // the word boundary, which is the word's high byte.
        assert_eq!(
            swr(0xaabb_ccdd, 0x1122_3344, 0x100, Endian::Big),
            0x44bb_ccdd
        );
        assert_eq!(
            swr(0xaabb_ccdd, 0x1122_3344, 0x103, Endian::Little),
            0x44bb_ccdd
        );
    }

    #[test]
    fn register_names_round_trip() {
        for (i, name) in REG_NAMES.iter().enumerate() {
            assert_eq!(reg_by_name(name), Some(i as u32));
        }
        assert_eq!(reg_by_name("$0"), Some(0));
        assert_eq!(reg_by_name("$31"), Some(31));
        assert_eq!(reg_by_name("fp"), Some(30));
        assert_eq!(reg_by_name("$32"), None);
        assert_eq!(reg_by_name("nonsense"), None);
    }

    #[test]
    fn a_requirement_names_the_coprocessor_that_would_be_unusable() {
        assert_eq!(Req::Base.coprocessor(), None);
        assert_eq!(Req::Cop0.coprocessor(), Some(0));
        assert_eq!(Req::Tlb.coprocessor(), Some(0));
        assert_eq!(Req::Cop2.coprocessor(), Some(2));
    }
}
