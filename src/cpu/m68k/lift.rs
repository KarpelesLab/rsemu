//! The m68k frontend: guest instructions lifted into [`ir::Block`](crate::ir::Block)s.
//!
//! The **fourth** frontend for the translation IR, after RISC-V, x86 and A64,
//! and the first for a core whose interpreter is bus-accurate to the cycle.
//! CLAUDE.md, "CPU cores", is the whole brief: *the interpreter is the oracle*,
//! and what this file emits is differentially tested against `exec.rs` forever
//! ([`differential`](super::differential)).
//!
//! # What is lifted, and what is not
//!
//! The subset is the **MC68000 integer core**, and only on an
//! [`Model::M68000`]: moves, the arithmetic and logic groups with their full
//! condition codes including **X**, the shifts and rotates at a *static* count,
//! `Bcc`/`BRA`/`DBcc`/`Scc`/`JMP`/`RTS`, `MOVEM` into registers, `LEA`,
//! `UNLK`, `EXG`, `EXT`, `SWAP`, `MOVEQ`, `TST`, `CLR`, `NOT`, `NEG`, `NEGX`,
//! `ADDX`, `SUBX`, `CMPM`, the bit instructions, `MOVE from SR`,
//! `MOVE to CCR`, the immediate-to-`CCR` trio, and `NOP`, across the twelve
//! addressing modes.
//!
//! Everything else ends the block with a terminator that hands the PC back to
//! the interpreter, which then executes that one instruction itself. The
//! *reasons* an encoding is declined are four, and they are worth separating
//! because only the first is a gap:
//!
//! 1. **Not written yet**: `MULU`/`MULS`/`DIVU`/`DIVS` (whose cycle counts are
//!    data-dependent — `exec::divu_cycles` derives them from the microcode's
//!    loop shape, which is a helper call rather than a lowering), the BCD
//!    group, `TAS`, `MOVEP`, `CHK`, a shift by a *register* count, and every
//!    privileged or exception-raising instruction (`TRAP`, `TRAPV`, `RTE`,
//!    `RTR`, `STOP`, `RESET`, `MOVE to SR`, `MOVE USP`, `ILLEGAL`, line A,
//!    line F).
//! 2. **Restartability** — see "A fault is handled by restarting the
//!    instruction" below. An instruction that commits **more than one store**
//!    is declined, which removes `JSR`, `BSR`, `PEA`, `LINK`, `MOVEM` *to*
//!    memory, and every long-sized *memory* destination (`MOVE.L <ea>,(An)`,
//!    `ADD.L Dn,(An)`, `ADDX.L -(Ay),-(Ax)`), because a 68000 writes a long
//!    as two word bus cycles.
//! 3. **Another processor.** Only a 68000 is lifted. A 68010's shorter `CLR`,
//!    a 68020's per-instruction timing table, its misaligned operands, its
//!    full-format extension words and its deferred prefetch faults are each a
//!    different frontend, and [`lift`] refuses the model outright rather than
//!    silently mis-timing it.
//! 4. **The core is not in a liftable state** — a pending reset, a pending
//!    interrupt, `STOP`, a halt, an `RTE` replay in flight, or **T** set in
//!    `SR`. Those are `super::engine`'s to check, not this file's.
//!
//! # Ticks: what is static here and what the host charges
//!
//! `ROADMAP.md` §9 and [`ir`](crate::ir)'s decision 2 make the tick count a
//! hashed *output*, so a block that charges 7 where the interpreter charged 8
//! is a state-hash failure rather than a rounding error. A 68000 has no
//! per-instruction cycle table at all (`exec.rs`, "One bus access is four
//! cycles"), which splits its time into exactly two kinds:
//!
//! | Site | Count | Static? |
//! | --- | --- | --- |
//! | a data read or write | 4 per byte or word bus cycle | **no** — an odd address is an address error and costs nothing |
//! | an instruction fetch (a prefetch queue *slide* or a refill) | 4 | **no** — a fetch from an odd address is an address error too, and a refill's target is a run-time value |
//! | microcode idle time (`Exec::internal`) | whatever MC68000UM §8 says | **yes** for this subset |
//!
//! So this file emits [`Opcode::CHARGE`] for the microcode idle cycles alone,
//! at exactly the points `exec.rs` calls `internal()`, and **every bus cycle
//! — data and fetch alike — charges itself through the host**. That is
//! CLAUDE.md's "cycle accounting is per-access, driven through the bus"
//! surviving into a lifted block rather than being replaced by a table.
//! [`InsnStart::ticks`] is therefore the static column, the two add up, and
//! the sum is one of the columns [`differential`](super::differential)
//! compares.
//!
//! ## The prefetch fetches are *performed*, not merely charged
//!
//! The other three frontends in this tree — `cpu::riscv::lift`,
//! `cpu::x86::lift`, `cpu::arm::a64::lift` — charge an instruction fetch and
//! never make the access. This one makes it: `Lifter::fetch` emits an
//! [`Opcode::LD`] with [`AccessKind::Fetch`] wherever `exec.rs` slides or
//! refills the queue. Three reasons, and none of them is tidiness:
//!
//! * A 68000's fetches are ordinary bus cycles **interleaved with its operand
//!   cycles**, so a device mapped over the code sees them in a defined order.
//! * **A fetch from an odd address is an address error**, and a `JMP (A0)` or
//!   an `RTS` to an odd target takes it at the refill. The target is a
//!   run-time value, so nothing at lift time could have declined it and a
//!   charge could not have raised it.
//! * The words come back as values, which is what lets the prefetch queue be
//!   modelled rather than reconstructed (below).
//!
//! # The prefetch queue, which is why this is not a RISC-V lifter
//!
//! `State::prefetch` is architectural state, and its invariant is one line
//! (`exec.rs`): `prefetch[0]` is the word at `pc`, and `prefetch[1]` is the
//! word at `pc + 2`. It is why `MOVE.W <mem>,($xxxxxxxx).L` writes its operand
//! *before* its last instruction fetch while the same move from a register
//! does not, and why the program counter an address-error frame pushes is not
//! the address of the faulting instruction.
//!
//! So it gets two slots ([`PREFETCH0`], [`PREFETCH1`]), and the words in them
//! are the words the block's own fetches returned — a slide rebinds
//! `prefetch[0]` to what `prefetch[1]` held and `prefetch[1]` to the word just
//! fetched, exactly as `Exec::slide` does. A block that never slides binds
//! neither and the host's own copies stay the truth.
//!
//! What this file does **not** model is `pc` moving once per slide *within* an
//! instruction, which is visible in a group-0 frame. It does not have to: a
//! fault restarts the instruction on the interpreter, which does.
//!
//! # A fault is handled by restarting the instruction
//!
//! A 68000's mid-instruction fault is architecturally visible in registers the
//! instruction has *already* changed: `MOVE.W (A0)+,(A1)+` whose write faults
//! leaves `A0` advanced, and the fourteen-byte group-0 frame it pushes carries
//! a program counter that depends on how far the prefetch got. The IR's only
//! channel from a block to guest state is [`InsnStart::live`], which publishes
//! a mapping taken **at a boundary** — so a write that lands *between*
//! boundaries cannot be expressed. [`Opcode::GET_SLOT`] says so in as many
//! words: *"there is no `set_slot`"*.
//!
//! The answer here is not to approximate it. `super::engine` unwinds the
//! ticks the partial instruction charged — every one of them, the host's
//! per-access four included — and hands that **one** guest instruction to the
//! interpreter, which redoes it from its first word and faults where and how
//! the hardware does, with the right registers, the right frame and the right
//! cycle count.
//!
//! That is exact only while nothing the instruction did before the fault can
//! be seen twice. Register updates cannot: the boundary published them
//! *un*-updated and the interpreter recomputes them. A **committed store**
//! can, so an instruction that makes more than one of them is not lifted at
//! all (reason 2 above), and the host asserts it never sees a second store in
//! one guest instruction. What would remove the restriction is an IR op —
//! `set_slot`, the dual of [`Opcode::GET_SLOT`] — and it is not added here
//! because the restriction is cheap, checkable, and costs a *documented* list
//! of encodings rather than a silent approximation.
//!
//! # Guest state: the slot numbering
//!
//! | Slot | State |
//! | --- | --- |
//! | `0..=7` | the data registers `D0`..`D7` ([`d_slot`]) |
//! | `8..=15` | the address registers `A0`..`A7` ([`a_slot`]) |
//! | `16` | the status register ([`SR`]) |
//! | `17` | the program counter ([`PC`]), bound only at an exit boundary |
//! | `18`, `19` | the prefetch queue ([`PREFETCH0`], [`PREFETCH1`]) |
//!
//! Every slot is 32 bits ([`Type::I32`]) — `SR` in its low sixteen — because
//! every m68k register is, `A7` included, and because a write to an address
//! register is 32 bits wide whatever the instruction's size says (M68000PRM
//! §1.2, quoted at `Lifter::write_a`).
//!
//! **Only the condition codes of `SR` are ever written**, never **S**, **T**
//! or the interrupt mask: the instructions that change those are outside the
//! subset. `super::engine` merges back `flags::CCR` alone, so the stack
//! pointer bank cannot move under a lifted block.
//!
//! ## The X flag needs nothing from the IR that is not already there
//!
//! **X** is bit 4 of `SR` and is *not* a copy of **C** — `CMP` writes carry
//! and leaves the extend of an `ADDX` chain alone (M68000PRM, *CMP*) — so it
//! is computed as its own one-bit temporary and deposited at bit 4, beside the
//! four temporaries for N, Z, V and C. The awkward rules fall out the same
//! way: `ADDX`'s **Z** is `Z_new & Z_old`, a loop-carried dependency that a
//! deferred-flags design could not express and an ordinary temporary
//! expresses for free (`src/ir/mod.rs`, decision 1, which names this exact
//! case).
//!
//! # Reading and writing guest registers
//!
//! As in every frontend here: **a write is a rebinding** (nothing is emitted;
//! the slot maps to the result temporary and the next boundary records it) and
//! **a read is [`Opcode::GET_SLOT`]**. A register the block has computed
//! stays in a temporary across the whole block.
//!
//! # Where the block ends
//!
//! * At a transfer of control ([`Stop::Transfer`]) — a branch, `JMP`, `RTS`,
//!   or a `DBcc`. Nothing is merged across one: superblocks are a later pass,
//!   and a 68000 branch target is *not* bounded by the block's window.
//! * At the [`WINDOW`] boundary ([`Stop::Window`]), so a store into the page a
//!   block was lifted from can be noticed at a block boundary and so the
//!   instruction words a block read stay one invalidation unit.
//! * At an encoding outside the subset ([`Stop::Unsupported`]) or bytes that
//!   could not be read ([`Stop::Unreadable`]) — the lifter never invents an
//!   encoding.
//! * At [`MAX_INSNS`] ([`Stop::Limit`]).
//!
//! # Sources
//!
//! *M68000 Family Programmer's Reference Manual* (M68000PM/AD) for every
//! instruction's operation and condition-code rules, and the *MC68000
//! 8-/16-/32-Bit Microprocessors User's Manual* (MC68000UM) §8 for the
//! instruction timing this file's charges mirror. Every non-obvious rule is
//! quoted, not merely cited, next to the code that implements it. The
//! *second* oracle is `exec.rs` itself, and where the two are both consulted
//! the comment says so. No emulator source of any licence was opened for any
//! part of this file (`ROADMAP.md` §1); Musashi, Cyclone, WinUAE, vAmiga,
//! Hatari and Basilisk II were not read.

use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Align, Block, BlockBuilder, Cond as IrCond, Const, Endian, InsnStart, MemOp,
    MemSpace, Opcode, RegSlot, Sign, Temp, Type,
};

use super::disasm;
use super::flags;
use super::isa::{Arg, Cond, Copro, Insn, Mode, Model, Op, Size, decode_with, ea_of};

// ---------------------------------------------------------------------------
// The slot numbering
// ---------------------------------------------------------------------------

/// The slot holding data register `D`*n*.
///
/// # Panics
///
/// Never: `n` is masked to three bits, because every caller derives it from a
/// three-bit opcode field.
#[inline]
#[must_use]
pub const fn d_slot(n: u32) -> RegSlot {
    RegSlot((n & 7) as u16)
}

/// The slot holding address register `A`*n*.
///
/// `A7` is whichever stack pointer the current privilege state selects, which
/// is what `State::a[7]` holds — the bank swap lives in `State::set_sr` and
/// nothing in the lifted subset can cause one.
#[inline]
#[must_use]
pub const fn a_slot(n: u32) -> RegSlot {
    RegSlot(8 + (n & 7) as u16)
}

/// The slot holding the status register.
///
/// Thirty-two bits wide with the register in the low sixteen, so every slot in
/// this frontend has one type. Only `flags::CCR` is ever written — see the
/// module docs.
pub const SR: RegSlot = RegSlot(16);

/// The slot holding the program counter.
///
/// Bound only at a block's **exit** boundary: at every other boundary the PC
/// is [`InsnStart::pc`], and a temporary for it would be a second source of
/// truth.
pub const PC: RegSlot = RegSlot(17);

/// The slot holding `State::prefetch[0]` — the word at [`PC`].
///
/// The prefetch queue is architectural state, not a cache: it is why
/// `MOVE.W <mem>,($xxxxxxxx).L` writes its operand before its last
/// instruction fetch, and why the program counter an address-error frame
/// pushes is not the address of the faulting instruction (`exec.rs`). So it
/// gets slots, and the words in them come from the fetch accesses themselves
/// rather than from a re-read at block exit — which is the difference between
/// modelling the queue and approximating it. A block that never slides never
/// binds these, and the host's own copies stay the truth.
pub const PREFETCH0: RegSlot = RegSlot(18);

/// The slot holding `State::prefetch[1]` — the word at [`PC`] `+ 2`.
pub const PREFETCH1: RegSlot = RegSlot(19);

/// One past the highest slot this frontend numbers.
pub const SLOT_COUNT: u16 = 20;

// ---------------------------------------------------------------------------
// Inputs and outputs
// ---------------------------------------------------------------------------

/// Where the lifter reads guest instruction words.
///
/// Words rather than bytes because a 68000 fetches in words and charges per
/// word, and because [`Insn`] is decoded from one. Implemented for every
/// `FnMut(u32) -> Option<u16>`, so a caller can pass a closure over an address
/// space or over a slice. `None` means "cannot be read here" and ends the
/// block ([`Stop::Unreadable`]).
pub trait InsnSource {
    /// The word at guest address `addr`, or `None` if it is unreadable.
    fn word(&mut self, addr: u32) -> Option<u16>;
}

impl<F: FnMut(u32) -> Option<u16>> InsnSource for F {
    #[inline]
    fn word(&mut self, addr: u32) -> Option<u16> {
        self(addr)
    }
}

/// Why a block stopped where it did.
///
/// Reported rather than inferred: "the block is short" has five causes here
/// and only one of them is a gap in the subset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// An encoding outside the subset. It was not lifted; the block's exit PC
    /// is its address, so the interpreter executes it next.
    Unsupported,
    /// A transfer of control this block does not follow.
    Transfer,
    /// The next instruction would leave the [`WINDOW`] the block started in.
    Window,
    /// The caller's instruction limit ([`MAX_INSNS`]).
    Limit,
    /// The instruction words could not be read.
    Unreadable,
}

/// A lifted block, and what is true about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lifted {
    /// The block. Always ends in a terminator and always passes
    /// [`verify`](crate::ir::verify).
    pub block: Block,
    /// Why lifting stopped.
    pub stop: Stop,
    /// How many guest instructions were lifted. Zero is legal and means the
    /// block's first instruction was outside the subset — the block is then
    /// just an exit boundary and a terminator.
    pub insns: usize,
    /// The address one past the last instruction lifted, which is the block's
    /// exit PC when nothing transferred control.
    pub end_pc: u32,
}

/// How many guest instructions [`lift`] will take by default.
///
/// A 68000 instruction is one to five words and the interpreter's step is one
/// instruction, so this is also the bound on how long a safe point can be
/// delayed (`ROADMAP.md` §4.7): thirty-two instructions is a few hundred guest
/// cycles, so a stop is prompt.
pub const MAX_INSNS: usize = 32;

/// The address window a block is confined to.
///
/// Not an MMU page — a 68000 has no MMU — but the same job in two directions:
/// it is the unit `super::engine` invalidates a translation by when the
/// guest writes into it, and it bounds how far ahead of the guest the lifter
/// reads. Four kilobytes because that is the granularity every other
/// translation cache in this tree uses.
pub const WINDOW: u32 = 4096;

/// The mask that selects an address's offset within its [`WINDOW`].
pub const WINDOW_MASK: u32 = WINDOW - 1;

/// The block cache key beside the entry PC.
///
/// [`Block::key`] is every configuration bit this lift depends on. Only the
/// model is in it, and that is a statement rather than an economy: this
/// frontend refuses every model but [`Model::M68000`], and a 68000's decode,
/// timing and operand rules do not depend on any *state* — not on **S**
/// (which decides only which address space an access reaches, and is the
/// host's business at access time), not on the condition codes (a `Bcc` is
/// lifted as a computed exit rather than a predicted one), and not on a
/// translation generation, because there is nothing to translate.
#[must_use]
pub fn key(model: Model) -> u64 {
    model as u64
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Lift the guest instructions at `entry_pc` into a translation block.
///
/// Reads at most `max_insns` instructions, never leaves `entry_pc`'s
/// [`WINDOW`], and always produces a well-formed block — including when
/// nothing could be lifted, in which case `Lifted::insns` is zero and the
/// block is an exit boundary and a terminator.
///
/// `src` must read the same bytes the interpreter's fetch would. There is no
/// translation to get wrong on a 68000, but there *is* a read-ahead: this
/// reads up to `max_insns` instructions the guest has not asked for, so a
/// caller reads through `MemAttrs::DEBUG` (CLAUDE.md, "a debugger read must
/// not pop a FIFO") and the engine's invalidation is what keeps the answer
/// fresh.
///
/// # Errors
///
/// [`Error::Unimplemented`] for any model but [`Model::M68000`] — see the
/// module docs, reason 3. Refused rather than approximated: a 68020's time
/// comes from a table this file does not read, and lifting one as a 68000
/// would be wrong on every instruction rather than on a few.
pub fn lift<S: InsnSource>(
    model: Model,
    entry_pc: u32,
    src: &mut S,
    max_insns: usize,
) -> Result<Lifted> {
    if model != Model::M68000 {
        return Err(Error::Unimplemented(
            "the m68k IR frontend is MC68000 only: a 68010's shorter CLR, a 68020's \
             per-instruction timing table and its misaligned operands are each a different \
             lowering",
        ));
    }
    // An odd program counter is an address error on the *fetch*, which is the
    // one fetch fault a 68000 takes before any bus cycle (MC68000UM §6.3.9).
    // The interpreter raises it; this refuses the block so it does.
    if entry_pc & 1 != 0 {
        return Ok(empty(model, entry_pc, Stop::Unsupported));
    }

    let mut lf = Lifter::new(model, entry_pc);
    let window = lf.window;
    let mut pc = entry_pc;
    let mut insns = 0usize;

    let stop = loop {
        if insns >= max_insns {
            break Stop::Limit;
        }
        if pc & !WINDOW_MASK != window {
            break Stop::Window;
        }
        // The instruction's length comes from the disassembler, which reads
        // the same declarative rows decode does — so there is one description
        // of the instruction set and not a second (CLAUDE.md, "CPU cores").
        let mut words = [0u16; disasm::MAX_EXT_WORDS + 1];
        let Some(opcode) = src.word(pc) else {
            break Stop::Unreadable;
        };
        words[0] = opcode;
        let mut have = 1usize;
        let len = loop {
            let d = disasm::disassemble_for(model, pc, &words[..have]);
            let len = u32::from(d.len);
            let need = (len as usize).div_ceil(2);
            if need <= have {
                break len;
            }
            let at = pc.wrapping_add(2 * have as u32);
            if at & !WINDOW_MASK != window {
                return Ok(lf.close(pc, insns, Stop::Window));
            }
            let Some(w) = src.word(at) else {
                return Ok(lf.close(pc, insns, Stop::Unreadable));
            };
            words[have] = w;
            have += 1;
            if have > disasm::MAX_EXT_WORDS {
                break u32::from(disasm::disassemble_for(model, pc, &words[..have]).len);
            }
        };
        let next_pc = pc.wrapping_add(len);
        if next_pc.wrapping_sub(1) & !WINDOW_MASK != window {
            // The instruction's own last word is outside the window, so its
            // bytes are not all in one invalidation unit.
            break Stop::Window;
        }

        match lf.insn(&words[..have], pc, next_pc) {
            Flow::Rejected => break Stop::Unsupported,
            Flow::Continue => {
                insns += 1;
                pc = next_pc;
            }
            Flow::Transfer => {
                insns += 1;
                pc = next_pc;
                break Stop::Transfer;
            }
        }
    };

    Ok(lf.close(pc, insns, stop))
}

/// A well-formed block that lifts nothing.
fn empty(model: Model, entry_pc: u32, stop: Stop) -> Lifted {
    let lf = Lifter::new(model, entry_pc);
    lf.close(entry_pc, 0, stop)
}

// ---------------------------------------------------------------------------
// The plan: what an encoding means, decided before anything is emitted
// ---------------------------------------------------------------------------

/// What lifting one instruction will emit.
///
/// Every encoding is classified — and every precondition checked — *before* a
/// single op is emitted, so the emitter is total and a rejected instruction
/// leaves no debris in the block (`cpu::riscv::lift` splits it the same way
/// and for the same reason).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Plan {
    /// `MOVE`/`MOVEA`.
    Move,
    /// `MOVEQ`.
    Moveq,
    /// A two-operand arithmetic or logical operation.
    Binary(BinKind),
    /// `CMP`/`CMPI`.
    Compare,
    /// `CMPM`.
    Cmpm,
    /// `ADDA`/`SUBA`.
    Adda { add: bool },
    /// `CMPA`.
    Cmpa,
    /// `ADDX`/`SUBX`.
    Addx { add: bool },
    /// `CLR`/`NOT`/`NEG`/`NEGX`.
    Unary,
    /// `TST`.
    Tst,
    /// `EXT`.
    Ext,
    /// `SWAP`.
    Swap,
    /// `EXG`.
    Exg,
    /// `LEA`.
    Lea,
    /// `UNLK`.
    Unlk,
    /// `MOVE from SR` or `MOVE from CCR`, and `MOVE to CCR`.
    MoveCcr { from_sr: bool },
    /// `ANDI`/`ORI`/`EORI` to `CCR`.
    CcrImm(BinKind),
    /// `BTST`/`BCHG`/`BCLR`/`BSET`.
    Bit,
    /// A shift or rotate by a count known at lift time.
    Shift { count: u32, memory: bool },
    /// `MOVEM` into registers.
    MovemLoad,
    /// `NOP`.
    Nop,
    /// `Bcc`/`BRA` with an 8-bit or 16-bit displacement.
    Branch {
        target: u32,
        always: bool,
        word: bool,
    },
    /// `DBcc`.
    Dbcc { target: u32 },
    /// `Scc`.
    Scc,
    /// `JMP`.
    Jmp,
    /// `RTS`.
    Rts,
}

/// Which two-operand arithmetic or logical operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BinKind {
    Add,
    Sub,
    And,
    Or,
    Eor,
}

/// How many *store* bus cycles an effective address of `size` costs.
///
/// A 68000 has a 16-bit data bus, so "there is no such thing as a 32-bit bus
/// cycle; a device watching the bus sees two" (`exec.rs`,
/// [`Exec::read_long`]). That is why a long memory destination is two stores
/// and so declined — see the module docs, reason 2.
const fn store_cycles(mode: Mode, size: Size) -> u32 {
    if !mode.is_memory() {
        return 0;
    }
    match size {
        Size::Long => 2,
        _ => 1,
    }
}

/// Decide what an encoding means, or decline it.
#[allow(clippy::too_many_lines)]
fn classify(insn: Insn, opcode: u16, size: Size, pc: u32, words: &[u16]) -> Option<Plan> {
    // A privileged encoding raises a privilege violation in user state and
    // executes in supervisor state, so lifting one would need the block keyed
    // on **S**. None of them is in the subset anyway.
    if insn.privileged {
        return None;
    }
    let src_mode = ea_of(insn.src, opcode).map(|(m, _)| m);
    let dst_mode = ea_of(insn.dst, opcode).map(|(m, _)| m);

    match insn.op {
        Op::Nop => Some(Plan::Nop),
        Op::Move | Op::Movea => {
            // A `MOVE` to a long memory destination is two stores.
            let stores = dst_mode.map_or(0, |m| store_cycles(m, size));
            if stores > 1 {
                return None;
            }
            Some(Plan::Move)
        }
        Op::Moveq => Some(Plan::Moveq),
        Op::Add
        | Op::Addi
        | Op::Addq
        | Op::Sub
        | Op::Subi
        | Op::Subq
        | Op::And
        | Op::Andi
        | Op::Or
        | Op::Ori
        | Op::Eor
        | Op::Eori => {
            // A read-modify-write into a long memory destination is two word
            // stores with the destination's own read in front of them, so a
            // fault on the second has already committed the first.
            if dst_mode.map_or(0, |m| store_cycles(m, size)) > 1 {
                return None;
            }
            Some(Plan::Binary(match insn.op {
                Op::Add | Op::Addi | Op::Addq => BinKind::Add,
                Op::Sub | Op::Subi | Op::Subq => BinKind::Sub,
                Op::And | Op::Andi => BinKind::And,
                Op::Or | Op::Ori => BinKind::Or,
                _ => BinKind::Eor,
            }))
        }
        Op::Cmp | Op::Cmpi => Some(Plan::Compare),
        Op::Cmpm => Some(Plan::Cmpm),
        Op::Adda => Some(Plan::Adda { add: true }),
        Op::Suba => Some(Plan::Adda { add: false }),
        Op::Cmpa => Some(Plan::Cmpa),
        Op::Addx | Op::Subx => {
            // The memory form is `-(Ay),-(Ax)`, and a long one writes two
            // words with a prefetch between them.
            if opcode & 0x0008 != 0 && size == Size::Long {
                return None;
            }
            Some(Plan::Addx {
                add: insn.op == Op::Addx,
            })
        }
        Op::Neg | Op::Negx | Op::Not | Op::Clr => {
            if dst_mode.map_or(0, |m| store_cycles(m, size)) > 1 {
                return None;
            }
            Some(Plan::Unary)
        }
        Op::Tst => Some(Plan::Tst),
        Op::Ext => Some(Plan::Ext),
        Op::Swap => Some(Plan::Swap),
        Op::Exg => Some(Plan::Exg),
        Op::Lea => Some(Plan::Lea),
        Op::Unlk => Some(Plan::Unlk),
        Op::MoveFromSr => Some(Plan::MoveCcr { from_sr: true }),
        Op::MoveToCcr => Some(Plan::MoveCcr { from_sr: false }),
        Op::AndiToCcr => Some(Plan::CcrImm(BinKind::And)),
        Op::OriToCcr => Some(Plan::CcrImm(BinKind::Or)),
        Op::EoriToCcr => Some(Plan::CcrImm(BinKind::Eor)),
        Op::Btst | Op::Bchg | Op::Bclr | Op::Bset => {
            // `BTST` never writes; the other three write a byte to memory or a
            // long to a register, so neither is two stores.
            //
            // A *dynamic* bit number into a data register is declined: the
            // write costs "two more, and BCLR two more again (MC68000UM Table
            // 8-7)" and which of the two applies turns on whether the reduced
            // bit number is at least sixteen — a run-time value, and
            // [`Opcode::CHARGE`] carries an immediate.
            if insn.op != Op::Btst
                && insn.src != Arg::BitNumber
                && matches!(dst_mode, Some(Mode::DataReg))
            {
                return None;
            }
            Some(Plan::Bit)
        }
        Op::Asl | Op::Asr | Op::Lsl | Op::Lsr | Op::Rol | Op::Ror | Op::Roxl | Op::Roxr => {
            if insn.dst == Arg::Ea {
                // "The memory form shifts one bit of one word" (`exec.rs`), so
                // it is one word store however the operand is addressed.
                if dst_mode.map_or(0, |m| store_cycles(m, Size::Word)) > 1 {
                    return None;
                }
                return Some(Plan::Shift {
                    count: 1,
                    memory: true,
                });
            }
            if opcode & 0x0020 != 0 {
                // A count in `Dn` is a run-time value the flag rules depend on
                // in six different ways (`exec::shift`), and the interpreter's
                // per-bit loop makes the cycle count depend on it too. Not
                // lifted; see the module docs, reason 1.
                return None;
            }
            let q = (opcode >> 9) & 7;
            Some(Plan::Shift {
                count: if q == 0 { 8 } else { u32::from(q) },
                memory: false,
            })
        }
        Op::Movem => {
            if insn.dst == Arg::Ea {
                // `MOVEM` to memory is one store per register, and a register
                // list is never empty in practice; declined wholesale rather
                // than for a mask of one, because the mask is an extension
                // word and the *shape* is what is being decided here.
                return None;
            }
            Some(Plan::MovemLoad)
        }
        Op::Bra | Op::Bcc => {
            let byte = opcode as i8;
            // The base is the address of the word after the opcode
            // (`exec::op_branch`).
            let base = pc.wrapping_add(2);
            if byte == 0 {
                let word = *words.get(1)?;
                Some(Plan::Branch {
                    target: base.wrapping_add(i32::from(word as i16) as u32),
                    always: insn.op == Op::Bra,
                    word: true,
                })
            } else {
                Some(Plan::Branch {
                    target: base.wrapping_add(i32::from(byte) as u32),
                    always: insn.op == Op::Bra,
                    word: false,
                })
            }
        }
        Op::Dbcc => {
            let base = pc.wrapping_add(2);
            let word = *words.get(1)?;
            Some(Plan::Dbcc {
                target: base.wrapping_add(i32::from(word as i16) as u32),
            })
        }
        Op::Scc => {
            if dst_mode.map_or(0, |m| store_cycles(m, Size::Byte)) > 1 {
                return None;
            }
            // "Two extra cycles when the byte is set, which is the one place a
            // 68000's timing depends on a condition (MC68000UM Table 8-11)" —
            // and only for a *register* destination. So `Scc Dn` is lifted
            // only for `ST` and `SF`, whose condition is a decode constant;
            // every memory form is lifted, because a memory `Scc` costs the
            // same either way.
            let cc = (opcode >> 8) & 0xf;
            if matches!(dst_mode, Some(Mode::DataReg)) && cc > 1 {
                return None;
            }
            Some(Plan::Scc)
        }
        Op::Jmp => Some(Plan::Jmp),
        Op::Rts => Some(Plan::Rts),
        _ => {
            let _ = src_mode;
            None
        }
    }
}

// ---------------------------------------------------------------------------
// The lifter
// ---------------------------------------------------------------------------

/// What lifting one instruction did, and whether lifting goes on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    /// Nothing was emitted; the instruction is outside the subset.
    Rejected,
    /// Lifted; carry on at the program-order successor.
    Continue,
    /// Lifted, and it transferred control somewhere this block does not
    /// follow, so the block ends.
    Transfer,
}

/// A resolved operand: where a value is, not what it is.
///
/// The same split `exec::Loc` makes, and for the same reason: resolving
/// separately from reading is what makes `ADDQ #1,(A0)+` increment `A0` once
/// rather than twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Loc {
    /// Data register `n`.
    D(u32),
    /// Address register `n`.
    A(u32),
    /// A memory location at this address.
    Mem(Temp),
    /// A value with no home: an immediate, or a field out of the opcode.
    Value(Temp),
    /// A memory location whose contents have already been read.
    ///
    /// `ADDX`/`SUBX` fetch their destination as part of the predecrement walk
    /// that computes its address, so re-reading it would put a bus cycle on
    /// the wire that hardware does not (`exec::Loc::Prefetched`).
    Prefetched(Temp, Temp),
}

/// One translation in progress.
struct Lifter {
    model: Model,
    /// The window the entry PC is in. No instruction outside it is lifted.
    window: u32,
    b: BlockBuilder,
    /// Which temporary holds each data register, where one does.
    d: [Option<Temp>; 8],
    /// Which temporary holds each address register, where one does.
    a: [Option<Temp>; 8],
    /// Which temporary holds `SR`, where one does.
    sr: Option<Temp>,
    /// Which temporaries hold the two prefetch words, where any do.
    p: [Option<Temp>; 2],
    /// Ticks charged so far, counted from block entry.
    ticks: u64,
    /// Ticks the *current* instruction has charged, so a deferred slide can be
    /// paid at the right point.
    deferred_slides: u32,
    /// Stores this guest instruction has emitted. The restartability budget is
    /// one; see the module docs.
    stores: u32,
    /// The instruction words of the instruction being lifted.
    words: Vec<u16>,
    /// How many of them have been consumed as extension words.
    consumed: usize,
    /// Whether the `MOVE` in progress took its source from memory.
    source_was_memory: bool,
    /// A `MOVE` destination's postincrement, owed once its write lands.
    deferred_postincrement: Option<(u32, u32)>,
    /// The temporary holding the exit PC, once a transfer has set one.
    pc_out: Option<Temp>,
    /// The exit PC where it is a constant, for the exit boundary's `pc` field.
    static_exit: Option<u32>,
    /// Whether the last instruction lifted closed the block itself, with one
    /// exit boundary per path.
    ///
    /// [`Lifter::close`] must then add nothing: its own boundary would come
    /// *after* the paths' and carry the pre-path tick column, which is lower
    /// than theirs — and the verifier rejects a boundary whose retired ticks
    /// went backwards. That is not a rule to work around: the column is what
    /// a mid-block fault reconstructs the cycle counter from.
    closed: bool,
    /// The address of the instruction being lifted — `Exec::pc0`.
    pc0: u32,
    /// The modelled `State::pc`: the address of `prefetch[0]`.
    ///
    /// It moves once per slide, which is what makes the program counter a
    /// group-0 exception frame pushes correct — and what makes a slide's fetch
    /// address `pc + 4` rather than a count of extension words.
    pc_model: u32,
}

impl Lifter {
    fn new(model: Model, entry_pc: u32) -> Lifter {
        Lifter {
            model,
            window: entry_pc & !WINDOW_MASK,
            b: BlockBuilder::new(u64::from(entry_pc), key(model)),
            d: [None; 8],
            a: [None; 8],
            sr: None,
            p: [None; 2],
            ticks: 0,
            deferred_slides: 0,
            stores: 0,
            words: Vec::new(),
            consumed: 0,
            source_was_memory: false,
            deferred_postincrement: None,
            pc_out: None,
            static_exit: None,
            closed: false,
            pc0: entry_pc,
            pc_model: entry_pc,
        }
    }

    // -- the clock ------------------------------------------------------

    /// Charge internal (non-bus) cycles, where `Exec::internal` charges them.
    fn internal(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        self.b.charge(u64::from(cycles));
        self.ticks += u64::from(cycles);
    }

    /// One instruction-fetch bus cycle at `addr`.
    ///
    /// A real [`Opcode::LD`] with [`AccessKind::Fetch`], not a bare charge.
    /// That is the one place this frontend is *more* faithful than the other
    /// three in the tree, and it is not decoration:
    ///
    /// * a 68000's fetches are ordinary bus cycles interleaved with its
    ///   operand cycles, so a device mapped over the code sees them;
    /// * **a fetch from an odd address is an address error** — the one fetch
    ///   fault a 68000 takes before any bus cycle (`exec::read_word_fc`) — and
    ///   a `JMP (A0)` or an `RTS` to an odd target raises it at the refill.
    ///   A charge cannot raise it, and the target is a run-time value, so
    ///   nothing at lift time could have declined it either;
    /// * the value is discarded, because the engine rebuilds `State::prefetch`
    ///   from the guest's own bytes when the block stops. It is the bus cycle
    ///   and the fault that are wanted, which is exactly what
    ///   [`MemOp::volatile`] protects from dead-code elimination.
    ///
    /// The four cycles are charged by the *host*, per access, as every other
    /// access here is.
    fn fetch(&mut self, addr: u32) -> Temp {
        let at = self.konst(addr);
        self.fetch_at(at)
    }

    /// The same, at an address only known at run time.
    fn fetch_at(&mut self, addr: Temp) -> Temp {
        let mem = MemOp {
            size: Width::U16,
            sign: Sign::Unsigned,
            space: MemSpace::MEM,
            seg: None,
            endian: Endian::Big,
            align: Align::Fault,
            kind: AccessKind::Fetch,
            volatile: true,
        };
        self.b.load(Type::I32, addr, mem)
    }

    /// Slide the queue one word: fetch from `pc + 4` and advance `pc` by two.
    ///
    /// "This is the only place `pc` moves during an instruction, which is what
    /// makes the value an address-error frame pushes correct" (`exec::slide`).
    fn slide(&mut self) {
        let at = self.pc_model.wrapping_add(4);
        let word = self.fetch(at);
        // "Executing an instruction *slides* the queue once per instruction
        // word: shift `prefetch[1]` down, read a fresh word from `pc + 4`, and
        // advance `pc` by two" (`exec.rs`). Both halves are rebindings.
        let ahead = self.read_prefetch(1);
        self.p[0] = Some(ahead);
        self.p[1] = Some(word);
        self.pc_model = self.pc_model.wrapping_add(2);
    }

    /// Take the next extension word, charging `delay` internal cycles between
    /// reading it and the refill it causes.
    ///
    /// `Exec::ext`: "The delay is where an indexed mode's two internal cycles
    /// go: the manual's timing puts them before the refill, and a bus trace
    /// shows them there (MC68000UM Table 8-1)."
    fn ext(&mut self, delay: u32) -> u16 {
        let word = self.take_word();
        self.internal(delay);
        self.slide();
        word
    }

    /// Take the next extension word but leave its refill for [`Lifter::settle`].
    fn ext_deferred(&mut self) -> u16 {
        self.deferred_slides += 1;
        self.take_word()
    }

    /// Pay off any deferred refill, then perform the instruction's final
    /// slide, which is what leaves the next opcode in `prefetch[0]`.
    fn settle(&mut self) {
        while self.deferred_slides > 0 {
            self.deferred_slides -= 1;
            self.slide();
        }
        self.slide();
    }

    /// Reload both prefetch words from `target`, with `gap` internal cycles
    /// between the two fetches (`Exec::refill`).
    ///
    /// "The program counter moves to `target - 4` *before* the first fetch and
    /// steps as each word lands, because the queue invariant is that `pc`
    /// addresses `prefetch[0]` and neither word has arrived yet. That is not
    /// bookkeeping for its own sake: a branch to an odd address takes an
    /// address error here, and the frame it pushes carries exactly this value."
    fn refill(&mut self, target: u32, gap: u32) {
        self.deferred_slides = 0;
        self.pc_model = target.wrapping_sub(4);
        let first = self.fetch(target);
        self.pc_model = target.wrapping_sub(2);
        self.internal(gap);
        let second = self.fetch(target.wrapping_add(2));
        self.pc_model = target;
        self.p[0] = Some(first);
        self.p[1] = Some(second);
    }

    /// A refill whose target is only known at run time.
    ///
    /// The address is a temporary rather than a constant, which is the whole
    /// difference: `JMP (A0)` and `RTS` can land on an odd address and this is
    /// where they find out. `pc_model` is left at the *pre-refill* value,
    /// because nothing after a computed transfer reads it — the block ends.
    fn refill_at(&mut self, target: Temp, gap: u32) {
        self.deferred_slides = 0;
        let first = self.fetch_at(target);
        self.internal(gap);
        let at = self.offset(target, 2);
        let second = self.fetch_at(at);
        self.p[0] = Some(first);
        self.p[1] = Some(second);
    }

    /// The next unconsumed instruction word.
    fn take_word(&mut self) -> u16 {
        let word = self.words.get(self.consumed).copied().unwrap_or(0);
        self.consumed += 1;
        word
    }

    // -- constants and guest state --------------------------------------

    /// Materialize a 32-bit constant.
    fn konst(&mut self, value: u32) -> Temp {
        self.b.imm(Type::I32, Const::Int(u128::from(value)))
    }

    /// A one-bit constant.
    fn kbit(&mut self, value: bool) -> Temp {
        self.b.imm(Type::I1, Const::Int(u128::from(value)))
    }

    /// Read data register `n` into a temporary.
    fn read_d(&mut self, n: u32) -> Temp {
        let n = (n & 7) as usize;
        match self.d[n] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I32, d_slot(n as u32));
                self.d[n] = Some(t);
                t
            }
        }
    }

    /// Read address register `n` into a temporary.
    fn read_a(&mut self, n: u32) -> Temp {
        let n = (n & 7) as usize;
        match self.a[n] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I32, a_slot(n as u32));
                self.a[n] = Some(t);
                t
            }
        }
    }

    /// Read a prefetch word into a temporary.
    fn read_prefetch(&mut self, which: usize) -> Temp {
        match self.p[which] {
            Some(t) => t,
            None => {
                let slot = if which == 0 { PREFETCH0 } else { PREFETCH1 };
                let t = self.b.get_slot(Type::I32, slot);
                self.p[which] = Some(t);
                t
            }
        }
    }

    /// Read `SR` into a temporary.
    fn read_sr(&mut self) -> Temp {
        match self.sr {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I32, SR);
                self.sr = Some(t);
                t
            }
        }
    }

    /// Bind data register `n` to a temporary. A write is a rebinding.
    fn write_d(&mut self, n: u32, t: Temp) {
        self.d[(n & 7) as usize] = Some(t);
    }

    /// Bind address register `n` to a temporary.
    ///
    /// "Every write to an address register is 32 bits wide, whatever the
    /// instruction's size says (M68000PRM §1.2)" — `exec::write_loc`. So there
    /// is no merge here, ever, and a caller that has a narrow value widens it
    /// first.
    fn write_a(&mut self, n: u32, t: Temp) {
        self.a[(n & 7) as usize] = Some(t);
    }

    /// Bind `SR` to a temporary.
    fn write_sr(&mut self, t: Temp) {
        self.sr = Some(t);
    }

    /// The register slots a temporary currently shadows, in slot order.
    ///
    /// Slot order rather than binding order: `ROADMAP.md` §0's determinism
    /// rule reaches the IR too, and this vector is hashed by anything that
    /// hashes a block.
    fn live_regs(&self) -> Vec<(RegSlot, Temp)> {
        let mut live = Vec::new();
        for (n, t) in self.d.iter().enumerate() {
            if let Some(t) = t {
                live.push((d_slot(n as u32), *t));
            }
        }
        for (n, t) in self.a.iter().enumerate() {
            if let Some(t) = t {
                live.push((a_slot(n as u32), *t));
            }
        }
        if let Some(t) = self.sr {
            live.push((SR, t));
        }
        if let Some(t) = self.p[0] {
            live.push((PREFETCH0, t));
        }
        if let Some(t) = self.p[1] {
            live.push((PREFETCH1, t));
        }
        live
    }

    // -- arithmetic helpers ---------------------------------------------

    /// Mask a value to `size`.
    fn masked(&mut self, value: Temp, size: Size) -> Temp {
        if size == Size::Long {
            return value;
        }
        let m = self.konst(size.mask());
        self.b.binary(Opcode::AND, Type::I32, value, m)
    }

    /// Replace the low `size` bits of `old` with `value` (`exec::merge`).
    fn merge(&mut self, old: Temp, value: Temp, size: Size) -> Temp {
        if size == Size::Long {
            return value;
        }
        let keep = self.konst(!size.mask());
        let hi = self.b.binary(Opcode::AND, Type::I32, old, keep);
        let lo = self.masked(value, size);
        self.b.binary(Opcode::OR, Type::I32, hi, lo)
    }

    /// Sign-extend the low `size` bits of `value` into all 32.
    fn sign_extend(&mut self, value: Temp, size: Size) -> Temp {
        match size {
            Size::Long => value,
            Size::Word => {
                let sh = self.konst(16);
                let up = self.b.binary(Opcode::SHL, Type::I32, value, sh);
                self.b.binary(Opcode::SAR, Type::I32, up, sh)
            }
            Size::Byte => {
                let sh = self.konst(24);
                let up = self.b.binary(Opcode::SHL, Type::I32, value, sh);
                self.b.binary(Opcode::SAR, Type::I32, up, sh)
            }
        }
    }

    /// Whether bit `bit` of `value` is set, as a one-bit temporary.
    fn bit_of(&mut self, value: Temp, bit: u32) -> Temp {
        let mask = self.konst(1u32 << bit);
        let and = self.b.binary(Opcode::AND, Type::I32, value, mask);
        let zero = self.konst(0);
        self.b.setcond(IrCond::Ne, Type::I32, and, zero)
    }

    /// Whether the low `size` bits of `value` are all zero.
    fn is_zero(&mut self, value: Temp, size: Size) -> Temp {
        let v = self.masked(value, size);
        let zero = self.konst(0);
        self.b.setcond(IrCond::Eq, Type::I32, v, zero)
    }

    /// Whether the sign bit of `value` at `size` is set.
    fn is_negative(&mut self, value: Temp, size: Size) -> Temp {
        let sign = match size {
            Size::Byte => 7,
            Size::Word => 15,
            Size::Long => 31,
        };
        self.bit_of(value, sign)
    }

    /// The condition codes each of the five flags currently holds.
    fn ccr_bits(&mut self) -> Ccr {
        let sr = self.read_sr();
        Ccr {
            x: self.bit_of(sr, 4),
            n: self.bit_of(sr, 3),
            z: self.bit_of(sr, 2),
            v: self.bit_of(sr, 1),
            c: self.bit_of(sr, 0),
        }
    }

    /// Write five condition-code bits back into `SR`, leaving every other bit
    /// of the register alone.
    fn set_ccr(&mut self, ccr: Ccr) {
        let sr = self.read_sr();
        let keep = self.konst(u32::from(!flags::CCR));
        let mut acc = self.b.binary(Opcode::AND, Type::I32, sr, keep);
        for (bit, flag) in [(4, ccr.x), (3, ccr.n), (2, ccr.z), (1, ccr.v), (0, ccr.c)] {
            let wide = self.b.unary(Opcode::EXT_Z, Type::I32, flag);
            let shifted = if bit == 0 {
                wide
            } else {
                let sh = self.konst(bit);
                self.b.binary(Opcode::SHL, Type::I32, wide, sh)
            };
            acc = self.b.binary(Opcode::OR, Type::I32, acc, shifted);
        }
        self.write_sr(acc);
    }

    /// `N` and `Z` from a result, `V` and `C` cleared, `X` untouched — the
    /// logical and move rule, "the only regular one in the instruction set"
    /// (`exec::set_logic_flags`).
    fn set_logic_flags(&mut self, value: Temp, size: Size) {
        let x = self.ccr_bits().x;
        let n = self.is_negative(value, size);
        let z = self.is_zero(value, size);
        let off = self.kbit(false);
        self.set_ccr(Ccr {
            x,
            n,
            z,
            v: off,
            c: off,
        });
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for an addition.
    ///
    /// `exec::set_add_flags` computes carry as `(sm && dm) || (!rm && (sm ||
    /// dm))` and overflow as `(sm && dm && !rm) || (!sm && !dm && rm)`, from
    /// the sign bits of source, destination and result — the adder's own
    /// formulation, which is what makes it right at every width. The same
    /// expressions are emitted here rather than an `ADDC` carry-out, because
    /// the operands are sub-register slices of 32-bit temporaries and a carry
    /// out of bit 31 is not a carry out of bit 7.
    fn add_flags(&mut self, src: Temp, dst: Temp, result: Temp, size: Size) -> (Temp, Temp) {
        let sm = self.is_negative(src, size);
        let dm = self.is_negative(dst, size);
        let rm = self.is_negative(result, size);
        let sm_and_dm = self.b.binary(Opcode::AND, Type::I1, sm, dm);
        let sm_or_dm = self.b.binary(Opcode::OR, Type::I1, sm, dm);
        let not_rm = self.b.unary(Opcode::NOT, Type::I1, rm);
        let second = self.b.binary(Opcode::AND, Type::I1, not_rm, sm_or_dm);
        let carry = self.b.binary(Opcode::OR, Type::I1, sm_and_dm, second);
        let both_neg_pos = self.b.binary(Opcode::AND, Type::I1, sm_and_dm, not_rm);
        let not_sm = self.b.unary(Opcode::NOT, Type::I1, sm);
        let not_dm = self.b.unary(Opcode::NOT, Type::I1, dm);
        let both_pos = self.b.binary(Opcode::AND, Type::I1, not_sm, not_dm);
        let both_pos_neg = self.b.binary(Opcode::AND, Type::I1, both_pos, rm);
        let overflow = self
            .b
            .binary(Opcode::OR, Type::I1, both_neg_pos, both_pos_neg);
        (carry, overflow)
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for a subtraction.
    ///
    /// `exec::set_sub_flags`: borrow is `(sm && !dm) || (rm && (sm || !dm))`
    /// and overflow is `(!sm && dm && !rm) || (sm && !dm && rm)`.
    fn sub_flags(&mut self, src: Temp, dst: Temp, result: Temp, size: Size) -> (Temp, Temp) {
        let sm = self.is_negative(src, size);
        let dm = self.is_negative(dst, size);
        let rm = self.is_negative(result, size);
        let not_dm = self.b.unary(Opcode::NOT, Type::I1, dm);
        let not_sm = self.b.unary(Opcode::NOT, Type::I1, sm);
        let not_rm = self.b.unary(Opcode::NOT, Type::I1, rm);
        let first = self.b.binary(Opcode::AND, Type::I1, sm, not_dm);
        let or = self.b.binary(Opcode::OR, Type::I1, sm, not_dm);
        let second = self.b.binary(Opcode::AND, Type::I1, rm, or);
        let borrow = self.b.binary(Opcode::OR, Type::I1, first, second);
        let a1 = self.b.binary(Opcode::AND, Type::I1, not_sm, dm);
        let a2 = self.b.binary(Opcode::AND, Type::I1, a1, not_rm);
        let b1 = self.b.binary(Opcode::AND, Type::I1, sm, not_dm);
        let b2 = self.b.binary(Opcode::AND, Type::I1, b1, rm);
        let overflow = self.b.binary(Opcode::OR, Type::I1, a2, b2);
        (borrow, overflow)
    }

    /// Evaluate one of the sixteen condition codes against `SR`.
    ///
    /// M68000PRM §3.2, *Condition Tests*, written out exactly as
    /// `State::test` writes it out: "`GT` is `N·V·Z̄ + N̄·V̄·Z̄`, and any
    /// 'simplification' of that is where the bugs live."
    fn test(&mut self, cond: Cond) -> Temp {
        let f = self.ccr_bits();
        let not = |lf: &mut Lifter, t: Temp| lf.b.unary(Opcode::NOT, Type::I1, t);
        match cond.0 {
            0x0 => self.kbit(true),
            0x1 => self.kbit(false),
            0x2 => {
                let nc = not(self, f.c);
                let nz = not(self, f.z);
                self.b.binary(Opcode::AND, Type::I1, nc, nz)
            }
            0x3 => self.b.binary(Opcode::OR, Type::I1, f.c, f.z),
            0x4 => not(self, f.c),
            0x5 => f.c,
            0x6 => not(self, f.z),
            0x7 => f.z,
            0x8 => not(self, f.v),
            0x9 => f.v,
            0xa => not(self, f.n),
            0xb => f.n,
            0xc => {
                let ne = self.b.binary(Opcode::XOR, Type::I1, f.n, f.v);
                not(self, ne)
            }
            0xd => self.b.binary(Opcode::XOR, Type::I1, f.n, f.v),
            0xe => {
                let ne = self.b.binary(Opcode::XOR, Type::I1, f.n, f.v);
                let eq = not(self, ne);
                let nz = not(self, f.z);
                self.b.binary(Opcode::AND, Type::I1, nz, eq)
            }
            _ => {
                let ne = self.b.binary(Opcode::XOR, Type::I1, f.n, f.v);
                self.b.binary(Opcode::OR, Type::I1, f.z, ne)
            }
        }
    }

    // -- memory ---------------------------------------------------------

    /// The access descriptor for a data access of `size`.
    ///
    /// [`Align::Fault`] on a word, because on a 68000 and a 68010 "an odd
    /// address is an address error" (`exec::read_word_fc`) — the access is not
    /// split and not performed, and it costs no cycles. The host is what turns
    /// that into vector 3 rather than vector 2.
    ///
    /// Big-endian, always: a 68000 has no byte-order control.
    /// [`MemOp::volatile`] on every access, because each one spends four
    /// cycles and can fault, both guest-visible, so dead-code elimination may
    /// not remove one whose value is discarded — `CLR.W (A0)` on a 68000 reads
    /// its destination and the read is a real bus cycle a device can see
    /// (MC68000UM Table 8-6).
    fn mem_op(size: Size, kind: AccessKind, sign: Sign) -> MemOp {
        MemOp {
            size: match size {
                Size::Byte => Width::U8,
                _ => Width::U16,
            },
            sign,
            space: MemSpace::MEM,
            seg: None,
            endian: Endian::Big,
            align: match size {
                Size::Byte => Align::None,
                _ => Align::Fault,
            },
            kind,
            volatile: true,
        }
    }

    /// One byte or word read.
    fn read_bus(&mut self, addr: Temp, size: Size, sign: Sign) -> Temp {
        let mem = Self::mem_op(size, AccessKind::Load, sign);
        self.b.load(Type::I32, addr, mem)
    }

    /// One byte or word write.
    ///
    /// The restartability budget is **one store per guest instruction** and
    /// this is where it is spent, so this is where it is checked: an
    /// instruction that commits two of them cannot be handed back to the
    /// interpreter after the first, and `classify` is supposed to have
    /// declined it. A debug build says which encoding got through rather than
    /// leaving `engine`'s host to discover it at run time.
    fn write_bus(&mut self, addr: Temp, value: Temp, size: Size) {
        let mem = Self::mem_op(size, AccessKind::Store, Sign::Unsigned);
        self.b.store(Type::I32, addr, value, mem);
        self.stores += 1;
        debug_assert!(
            self.stores <= 1,
            "opcode {:04x} at {:#x} would commit {} stores; `classify` must decline it \
             (see the module docs, \"A fault is handled by restarting the instruction\")",
            self.words.first().copied().unwrap_or(0),
            self.pc0,
            self.stores
        );
    }

    /// `addr + delta`, computed in the guest's width.
    ///
    /// Thirty-two bits then widened, never the other way round: CLAUDE.md's
    /// "guest addresses are computed in the guest's width, then widened", and
    /// on a 68000 the wrap at 24 bits is the host's business rather than this
    /// arithmetic's.
    fn offset(&mut self, addr: Temp, delta: u32) -> Temp {
        if delta == 0 {
            return addr;
        }
        let k = self.konst(delta);
        self.b.binary(Opcode::ADD, Type::I32, addr, k)
    }

    /// A long read: "two word accesses, high word first" (`exec::read_long`).
    fn read_long(&mut self, addr: Temp) -> Temp {
        let hi = self.read_bus(addr, Size::Word, Sign::Unsigned);
        let at = self.offset(addr, 2);
        let lo = self.read_bus(at, Size::Word, Sign::Unsigned);
        let sh = self.konst(16);
        let up = self.b.binary(Opcode::SHL, Type::I32, hi, sh);
        self.b.binary(Opcode::OR, Type::I32, up, lo)
    }

    /// Read a resolved operand (`exec::read_loc`).
    fn read_loc(&mut self, loc: Loc, size: Size) -> Temp {
        match loc {
            Loc::D(n) => {
                let v = self.read_d(n);
                self.masked(v, size)
            }
            Loc::A(n) => {
                let v = self.read_a(n);
                self.masked(v, size)
            }
            Loc::Value(t) | Loc::Prefetched(_, t) => self.masked(t, size),
            Loc::Mem(addr) => match size {
                Size::Byte => self.read_bus(addr, Size::Byte, Sign::Unsigned),
                Size::Word => self.read_bus(addr, Size::Word, Sign::Unsigned),
                Size::Long => self.read_long(addr),
            },
        }
    }

    /// Write a resolved operand (`exec::write_loc`).
    fn write_loc(&mut self, loc: Loc, size: Size, value: Temp) {
        match loc {
            Loc::D(n) => {
                let old = self.read_d(n);
                let merged = self.merge(old, value, size);
                self.write_d(n, merged);
            }
            Loc::A(n) => self.write_a(n, value),
            Loc::Value(_) => {}
            Loc::Mem(addr) | Loc::Prefetched(addr, _) => match size {
                Size::Byte => self.write_bus(addr, value, Size::Byte),
                Size::Word => self.write_bus(addr, value, Size::Word),
                Size::Long => {
                    // Two word writes, high word first. Only reached for a
                    // register destination in this subset — `classify`
                    // declines a long memory destination — and kept honest
                    // here so a future widening cannot silently emit one word.
                    let sh = self.konst(16);
                    let hi = self.b.binary(Opcode::SHR, Type::I32, value, sh);
                    self.write_bus(addr, hi, Size::Word);
                    let at = self.offset(addr, 2);
                    self.write_bus(at, value, Size::Word);
                }
            },
        }
    }

    // -- addressing modes -----------------------------------------------

    /// How far an autoincrement mode steps (`exec::step`).
    ///
    /// "A byte access through `A7` steps by two, because the 68000 keeps the
    /// stack pointer even; there is no such rule for `A0`-`A6` (M68000PRM
    /// §1.2)."
    const fn step(size: Size, reg: u32) -> u32 {
        match size {
            Size::Byte if reg == 7 => 2,
            other => other.bytes(),
        }
    }

    /// Resolve an operand slot (`exec::resolve`).
    fn resolve(&mut self, arg: Arg, opcode: u16, size: Size) -> Option<Loc> {
        match arg {
            Arg::None => {
                let z = self.konst(0);
                Some(Loc::Value(z))
            }
            Arg::DnHi => Some(Loc::D(u32::from((opcode >> 9) & 7))),
            Arg::DnLo => Some(Loc::D(u32::from(opcode & 7))),
            Arg::AnHi => Some(Loc::A(u32::from((opcode >> 9) & 7))),
            Arg::AnLo => Some(Loc::A(u32::from(opcode & 7))),
            Arg::Quick => {
                let q = (opcode >> 9) & 7;
                let v = if q == 0 { 8 } else { u32::from(q) };
                let t = self.konst(v);
                Some(Loc::Value(t))
            }
            Arg::QuickByte => {
                let v = i32::from(opcode as i8) as u32;
                let t = self.konst(v);
                Some(Loc::Value(t))
            }
            Arg::Imm => {
                let v = self.immediate(size);
                Some(Loc::Value(v))
            }
            Arg::Sr => {
                let sr = self.read_sr();
                let t = self.masked(sr, Size::Word);
                Some(Loc::Value(t))
            }
            Arg::Ccr => {
                let sr = self.read_sr();
                let m = self.konst(u32::from(flags::CCR));
                let t = self.b.binary(Opcode::AND, Type::I32, sr, m);
                Some(Loc::Value(t))
            }
            Arg::Ea | Arg::EaDst => self.resolve_ea(arg, opcode, size, Extra::Operand),
            _ => None,
        }
    }

    /// An immediate operand out of the instruction stream.
    fn immediate(&mut self, size: Size) -> Temp {
        let v = match size {
            Size::Byte => u32::from(self.ext(0) & 0xff),
            Size::Word => u32::from(self.ext(0)),
            Size::Long => {
                let hi = self.ext(0);
                let lo = self.ext(0);
                (u32::from(hi) << 16) | u32::from(lo)
            }
        };
        self.konst(v)
    }

    /// Resolve an effective address (`exec::resolve_ea`).
    #[allow(clippy::too_many_lines)]
    fn resolve_ea(&mut self, arg: Arg, opcode: u16, size: Size, extra: Extra) -> Option<Loc> {
        let (mode, reg) = ea_of(arg, opcode)?;
        let reg = u32::from(reg);
        match mode {
            Mode::DataReg => Some(Loc::D(reg)),
            Mode::AddrReg => Some(Loc::A(reg)),
            Mode::Indirect => {
                let a = self.read_a(reg);
                Some(Loc::Mem(a))
            }
            Mode::PostInc => {
                let addr = self.read_a(reg);
                let by = Self::step(size, reg);
                if extra == Extra::MoveDest {
                    // "nothing has touched the address register by the time
                    // the write is attempted" (`exec::resolve_ea`).
                    self.deferred_postincrement = Some((reg, by));
                } else {
                    let next = self.offset(addr, by);
                    self.write_a(reg, next);
                }
                Some(Loc::Mem(addr))
            }
            Mode::PreDec => {
                // "Two internal cycles for the decrement, before the access —
                // except as a `MOVE` destination, where the decrement overlaps
                // the prefetch that a move performs before its write and costs
                // nothing (MC68000UM Table 8-5 against Table 8-1)."
                if extra != Extra::MoveDest {
                    self.internal(2);
                }
                let base = self.read_a(reg);
                let by = Self::step(size, reg);
                let addr = self.offset(base, by.wrapping_neg());
                if extra != Extra::MoveDest {
                    self.write_a(reg, addr);
                }
                Some(Loc::Mem(addr))
            }
            Mode::Disp16 => {
                let disp = i32::from(self.ext(0) as i16) as u32;
                let a = self.read_a(reg);
                let addr = self.offset(a, disp);
                Some(Loc::Mem(addr))
            }
            Mode::Index8 => {
                let word = self.ext(extra.index_delay());
                let a = self.read_a(reg);
                let addr = self.index_address(a, word);
                Some(Loc::Mem(addr))
            }
            Mode::AbsShort => {
                let v = i32::from(self.ext(0) as i16) as u32;
                let t = self.konst(v);
                Some(Loc::Mem(t))
            }
            Mode::AbsLong => {
                let hi = self.ext(0);
                // "A `MOVE` to an absolute long address writes its operand
                // *before* the last of its own instruction fetches — but only
                // when the source came out of memory" (`exec::resolve_ea`).
                let defer = extra == Extra::MoveDest && self.source_was_memory;
                let lo = if defer {
                    self.ext_deferred()
                } else {
                    self.ext(0)
                };
                let t = self.konst((u32::from(hi) << 16) | u32::from(lo));
                Some(Loc::Mem(t))
            }
            Mode::PcDisp16 => {
                // "The base is the address of the extension word itself, which
                // is `pc + 2` while the word is still in the queue."
                let base = self.pc_model.wrapping_add(2);
                let disp = i32::from(self.ext(0) as i16) as u32;
                let t = self.konst(base.wrapping_add(disp));
                Some(Loc::Mem(t))
            }
            Mode::PcIndex8 => {
                let base = self.pc_model.wrapping_add(2);
                let word = self.ext(extra.index_delay());
                let b = self.konst(base);
                let addr = self.index_address(b, word);
                Some(Loc::Mem(addr))
            }
            Mode::Imm => {
                let v = self.immediate(size);
                Some(Loc::Value(v))
            }
        }
    }

    /// Apply a brief extension word to a base address (`exec::index_address`).
    ///
    /// "Bit 15 selects the data or address file, bits 14-12 the register, bit
    /// 11 whether the index is the sign-extended low word or the whole
    /// register, and bits 7-0 are a signed displacement. Bits 10-8 are the
    /// 68020's scale and full-format bits and are ignored here, which is what
    /// a 68000 does with them (M68000PRM §2.1)."
    fn index_address(&mut self, base: Temp, ext: u16) -> Temp {
        let reg = u32::from((ext >> 12) & 7);
        let value = if ext & 0x8000 != 0 {
            self.read_a(reg)
        } else {
            self.read_d(reg)
        };
        let index = if ext & 0x0800 != 0 {
            value
        } else {
            self.sign_extend(value, Size::Word)
        };
        let sum = self.b.binary(Opcode::ADD, Type::I32, base, index);
        let disp = i32::from(ext as i8) as u32;
        self.offset(sum, disp)
    }

    // -- one instruction ------------------------------------------------

    /// Lift one instruction.
    fn insn(&mut self, words: &[u16], pc: u32, next_pc: u32) -> Flow {
        let opcode = words[0];
        // The same decode the interpreter runs, with no coprocessor: a 68000
        // has none, so every F-line word is the line-F exception and is
        // outside the subset anyway.
        let insn = decode_with(self.model, Copro::NONE, opcode);
        let Some(size) = insn.size.resolve(opcode) else {
            return Flow::Rejected;
        };
        let Some(plan) = classify(insn, opcode, size, pc, words) else {
            return Flow::Rejected;
        };

        // Everything below this line emits, so the boundary opens here.
        let live = self.live_regs();
        self.b.insn_start(InsnStart {
            pc: u64::from(pc),
            next_pc: u64::from(next_pc),
            ticks: self.ticks,
            live,
        });

        self.words.clear();
        self.words.extend_from_slice(words);
        self.consumed = 1;
        self.pc0 = pc;
        self.pc_model = pc;
        self.deferred_slides = 0;
        self.stores = 0;
        self.source_was_memory = false;
        self.deferred_postincrement = None;

        self.emit(plan, insn, opcode, size, pc, next_pc)
    }

    #[allow(clippy::too_many_lines)]
    fn emit(
        &mut self,
        plan: Plan,
        insn: Insn,
        opcode: u16,
        size: Size,
        pc: u32,
        next_pc: u32,
    ) -> Flow {
        let hi = u32::from((opcode >> 9) & 7);
        let lo = u32::from(opcode & 7);
        match plan {
            Plan::Nop => {
                self.settle();
                Flow::Continue
            }
            Plan::Move => self.op_move(insn, opcode, size),
            Plan::Moveq => {
                let value = self.konst(i32::from(opcode as i8) as u32);
                self.write_d(hi, value);
                self.set_logic_flags(value, Size::Long);
                self.settle();
                Flow::Continue
            }
            Plan::Binary(kind) => self.op_binary(insn, opcode, size, kind),
            Plan::Compare => self.op_compare(insn, opcode, size),
            Plan::Cmpm => self.op_cmpm(opcode, size),
            Plan::Adda { add } => self.op_adda(opcode, size, add),
            Plan::Cmpa => self.op_cmpa(opcode, size),
            Plan::Addx { add } => self.op_addx(opcode, size, add),
            Plan::Unary => self.op_unary(insn, opcode, size),
            Plan::Tst => {
                let Some(src) = self.resolve(Arg::Ea, opcode, size) else {
                    return Flow::Rejected;
                };
                let v = self.read_loc(src, size);
                self.set_logic_flags(v, size);
                self.settle();
                Flow::Continue
            }
            Plan::Ext => {
                let old = self.read_d(lo);
                let value = if size == Size::Long {
                    self.sign_extend(old, Size::Word)
                } else {
                    let byte = self.sign_extend(old, Size::Byte);
                    self.merge(old, byte, Size::Word)
                };
                self.write_d(lo, value);
                self.set_logic_flags(value, size);
                self.settle();
                Flow::Continue
            }
            Plan::Swap => {
                let old = self.read_d(lo);
                let sh = self.konst(16);
                let value = self.b.binary(Opcode::ROTL, Type::I32, old, sh);
                self.write_d(lo, value);
                self.set_logic_flags(value, Size::Long);
                self.settle();
                Flow::Continue
            }
            Plan::Exg => {
                match (insn.src, insn.dst) {
                    (Arg::DnHi, Arg::DnLo) => {
                        let a = self.read_d(hi);
                        let b = self.read_d(lo);
                        self.write_d(hi, b);
                        self.write_d(lo, a);
                    }
                    (Arg::AnHi, Arg::AnLo) => {
                        let a = self.read_a(hi);
                        let b = self.read_a(lo);
                        self.write_a(hi, b);
                        self.write_a(lo, a);
                    }
                    _ => {
                        let d = self.read_d(hi);
                        let a = self.read_a(lo);
                        self.write_d(hi, a);
                        self.write_a(lo, d);
                    }
                }
                self.internal(2);
                self.settle();
                Flow::Continue
            }
            Plan::Lea => {
                let Some(Loc::Mem(addr)) =
                    self.resolve_ea(Arg::Ea, opcode, Size::Long, Extra::Control)
                else {
                    return Flow::Rejected;
                };
                self.write_a(hi, addr);
                self.settle();
                Flow::Continue
            }
            Plan::Unlk => {
                // A long read and two register writes, no store. "The stack
                // pointer is restored first and the register second, so
                // `UNLK A7` ends up holding the popped value rather than the
                // frame pointer plus four" (`exec.rs`, from M68000PRM,
                // *UNLK*).
                let frame = self.read_a(lo);
                let saved = self.read_long(frame);
                let next = self.offset(frame, 4);
                self.write_a(7, next);
                self.write_a(lo, saved);
                self.settle();
                Flow::Continue
            }
            Plan::MoveCcr { from_sr } => self.op_move_ccr(insn, opcode, from_sr),
            Plan::CcrImm(kind) => {
                let imm = self.immediate(Size::Byte);
                let sr = self.read_sr();
                let op = match kind {
                    BinKind::And => Opcode::AND,
                    BinKind::Or => Opcode::OR,
                    _ => Opcode::XOR,
                };
                // Only the condition codes take part; `ANDI #0,CCR` must not
                // clear **S** (M68000PRM, *ANDI to CCR*: "Source ∧ CCR → CCR").
                let widened = if kind == BinKind::And {
                    let keep = self.konst(u32::from(!flags::CCR));
                    self.b.binary(Opcode::OR, Type::I32, imm, keep)
                } else {
                    let m = self.konst(u32::from(flags::CCR));
                    self.b.binary(Opcode::AND, Type::I32, imm, m)
                };
                let value = self.b.binary(op, Type::I32, sr, widened);
                self.write_sr(value);
                // `internal(8)`, then `idle_fetch()` — one word read at
                // `pc + 2` that is thrown away, a real bus cycle charged four
                // — then the final slide. Twenty cycles in all, which is what
                // MC68000UM Table 8-6 publishes for `ORI to CCR`.
                self.internal(8);
                // `Exec::idle_fetch`: one word read at `pc + 2`, thrown away,
                // and the queue does *not* slide for it.
                let idle = self.pc_model.wrapping_add(2);
                self.fetch(idle);
                self.settle();
                Flow::Continue
            }
            Plan::Bit => self.op_bit(insn, opcode, size),
            Plan::Shift { count, memory } => self.op_shift(insn, opcode, size, count, memory),
            Plan::MovemLoad => self.op_movem_load(opcode, size),
            Plan::Branch {
                target,
                always,
                word,
            } => self.op_branch(opcode, target, always, word, next_pc),
            Plan::Dbcc { target } => self.op_dbcc(opcode, target, next_pc),
            Plan::Scc => self.op_scc(opcode),
            Plan::Jmp => self.op_jmp(opcode, pc),
            Plan::Rts => {
                let sp = self.read_a(7);
                let target = self.read_long(sp);
                let next = self.offset(sp, 4);
                self.write_a(7, next);
                self.refill_at(target, 0);
                self.pc_out = Some(target);
                Flow::Transfer
            }
        }
    }

    // -- instruction bodies ---------------------------------------------

    fn op_move(&mut self, insn: Insn, opcode: u16, size: Size) -> Flow {
        self.source_was_memory = ea_of(insn.src, opcode).is_some_and(|(mode, _)| mode.is_memory());
        let Some(src) = self.resolve(insn.src, opcode, size) else {
            return Flow::Rejected;
        };
        let value = self.read_loc(src, size);
        if insn.op == Op::Movea {
            // "MOVEA.W sign-extends into the whole register."
            let value = if size == Size::Word {
                self.sign_extend(value, Size::Word)
            } else {
                value
            };
            self.write_a(u32::from((opcode >> 9) & 7), value);
            self.settle();
            return Flow::Continue;
        }
        let Some(dst) = self.resolve_ea(insn.dst, opcode, size, Extra::MoveDest) else {
            return Flow::Rejected;
        };
        self.set_logic_flags(value, size);
        if let Some((Mode::PreDec, reg)) = ea_of(insn.dst, opcode) {
            // "A predecrement destination is the one `MOVE` that prefetches
            // before it writes, and its long form puts the low word out
            // first." A long one is two stores and `classify` declined it, so
            // only the byte and word forms reach here.
            let reg = u32::from(reg);
            self.settle();
            let base = self.read_a(reg);
            let addr = self.offset(base, Self::step(size, reg).wrapping_neg());
            self.write_a(reg, addr);
            self.write_loc(Loc::Mem(addr), size, value);
        } else {
            self.write_loc(dst, size, value);
            self.settle();
        }
        if let Some((reg, by)) = self.deferred_postincrement.take() {
            let base = self.read_a(reg);
            let next = self.offset(base, by);
            self.write_a(reg, next);
        }
        Flow::Continue
    }

    fn op_binary(&mut self, insn: Insn, opcode: u16, size: Size, kind: BinKind) -> Flow {
        let Some(src) = self.resolve(insn.src, opcode, size) else {
            return Flow::Rejected;
        };
        let src_value = self.read_loc(src, size);
        let Some(dst) = self.resolve(insn.dst, opcode, size) else {
            return Flow::Rejected;
        };
        if let Loc::A(n) = dst
            && matches!(insn.op, Op::Addq | Op::Subq)
        {
            // "ADDQ/SUBQ on an address register is a full 32-bit add that
            // touches no flags at all (M68000PRM, ADDQ)."
            let base = self.read_a(n);
            let value = match kind {
                BinKind::Add => self.b.binary(Opcode::ADD, Type::I32, base, src_value),
                _ => self.b.binary(Opcode::SUB, Type::I32, base, src_value),
            };
            self.write_a(n, value);
            // "A long quick add to an address register is two cycles cheaper
            // than a word one: the word form has to sign-extend first."
            self.internal(if size == Size::Long { 2 } else { 4 });
            self.settle();
            return Flow::Continue;
        }
        let dst_value = self.read_loc(dst, size);
        let result = match kind {
            BinKind::Add => {
                let raw = self.b.binary(Opcode::ADD, Type::I32, dst_value, src_value);
                let r = self.masked(raw, size);
                let (c, v) = self.add_flags(src_value, dst_value, r, size);
                let n = self.is_negative(r, size);
                let z = self.is_zero(r, size);
                self.set_ccr(Ccr { x: c, n, z, v, c });
                r
            }
            BinKind::Sub => {
                let raw = self.b.binary(Opcode::SUB, Type::I32, dst_value, src_value);
                let r = self.masked(raw, size);
                let (c, v) = self.sub_flags(src_value, dst_value, r, size);
                let n = self.is_negative(r, size);
                let z = self.is_zero(r, size);
                self.set_ccr(Ccr { x: c, n, z, v, c });
                r
            }
            BinKind::And | BinKind::Or | BinKind::Eor => {
                let op = match kind {
                    BinKind::And => Opcode::AND,
                    BinKind::Or => Opcode::OR,
                    _ => Opcode::XOR,
                };
                let raw = self.b.binary(op, Type::I32, dst_value, src_value);
                let r = self.masked(raw, size);
                self.set_logic_flags(r, size);
                r
            }
        };
        self.arith_internal(insn, opcode, size, dst);
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result);
            self.settle();
        } else {
            // "A memory destination is written after the final prefetch on a
            // 68000; the write is the last bus cycle of a read-modify-write
            // only for TAS (MC68000UM Table 8-5)."
            self.settle();
            self.write_back(dst, size, result);
        }
        Flow::Continue
    }

    /// "The long-operand penalty the manual marks with a double dagger. A
    /// 32-bit ALU operation into a data register costs two extra cycles, and
    /// four when the source needed no bus cycle of its own — a register or an
    /// immediate (MC68000UM Table 8-5, note **)."
    fn arith_internal(&mut self, insn: Insn, opcode: u16, size: Size, dst: Loc) {
        if size != Size::Long || !matches!(dst, Loc::D(_)) {
            return;
        }
        let cheap_source = match insn.src {
            Arg::Imm | Arg::Quick | Arg::DnHi => true,
            Arg::Ea => matches!(
                ea_of(Arg::Ea, opcode),
                Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
            ),
            _ => false,
        };
        self.internal(if cheap_source { 4 } else { 2 });
    }

    /// Commit the result of a read-modify-write (`exec::write_back`).
    ///
    /// Identical to [`Lifter::write_loc`] except for a long memory
    /// destination, which goes out low word first — and `classify` declines
    /// those, so the two coincide here. Kept as its own method so the
    /// distinction is where the interpreter puts it.
    fn write_back(&mut self, loc: Loc, size: Size, value: Temp) {
        if let (Loc::Mem(addr) | Loc::Prefetched(addr, _), Size::Long) = (loc, size) {
            let at = self.offset(addr, 2);
            self.write_bus(at, value, Size::Word);
            let sh = self.konst(16);
            let hi = self.b.binary(Opcode::SHR, Type::I32, value, sh);
            self.write_bus(addr, hi, Size::Word);
            return;
        }
        self.write_loc(loc, size, value);
    }

    fn op_compare(&mut self, insn: Insn, opcode: u16, size: Size) -> Flow {
        let Some(src) = self.resolve(insn.src, opcode, size) else {
            return Flow::Rejected;
        };
        let src_value = self.read_loc(src, size);
        let Some(dst) = self.resolve(insn.dst, opcode, size) else {
            return Flow::Rejected;
        };
        let dst_value = self.read_loc(dst, size);
        let raw = self.b.binary(Opcode::SUB, Type::I32, dst_value, src_value);
        let r = self.masked(raw, size);
        // "CMP leaves X alone: it is a test, not an arithmetic step."
        let x = self.ccr_bits().x;
        let (c, v) = self.sub_flags(src_value, dst_value, r, size);
        let n = self.is_negative(r, size);
        let z = self.is_zero(r, size);
        self.set_ccr(Ccr { x, n, z, v, c });
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        self.settle();
        Flow::Continue
    }

    fn op_cmpm(&mut self, opcode: u16, size: Size) -> Flow {
        let y = u32::from(opcode & 7);
        let x = u32::from((opcode >> 9) & 7);
        let src_addr = self.read_a(y);
        let next_y = self.offset(src_addr, Self::step(size, y));
        self.write_a(y, next_y);
        let src_value = self.read_loc(Loc::Mem(src_addr), size);
        let dst_addr = self.read_a(x);
        let next_x = self.offset(dst_addr, Self::step(size, x));
        self.write_a(x, next_x);
        let dst_value = self.read_loc(Loc::Mem(dst_addr), size);
        let raw = self.b.binary(Opcode::SUB, Type::I32, dst_value, src_value);
        let r = self.masked(raw, size);
        let xf = self.ccr_bits().x;
        let (c, v) = self.sub_flags(src_value, dst_value, r, size);
        let n = self.is_negative(r, size);
        let z = self.is_zero(r, size);
        self.set_ccr(Ccr { x: xf, n, z, v, c });
        self.settle();
        Flow::Continue
    }

    fn op_adda(&mut self, opcode: u16, size: Size, add: bool) -> Flow {
        let Some(src) = self.resolve(Arg::Ea, opcode, size) else {
            return Flow::Rejected;
        };
        let raw = self.read_loc(src, size);
        // "A word source is sign-extended to 32 bits before the add; the
        // operation itself is always long (M68000PRM, ADDA)."
        let value = if size == Size::Word {
            self.sign_extend(raw, Size::Word)
        } else {
            raw
        };
        let n = u32::from((opcode >> 9) & 7);
        let base = self.read_a(n);
        let result = if add {
            self.b.binary(Opcode::ADD, Type::I32, base, value)
        } else {
            self.b.binary(Opcode::SUB, Type::I32, base, value)
        };
        self.write_a(n, result);
        let cheap = matches!(
            ea_of(Arg::Ea, opcode),
            Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
        );
        self.internal(if size == Size::Word || cheap { 4 } else { 2 });
        self.settle();
        Flow::Continue
    }

    fn op_cmpa(&mut self, opcode: u16, size: Size) -> Flow {
        let Some(src) = self.resolve(Arg::Ea, opcode, size) else {
            return Flow::Rejected;
        };
        let raw = self.read_loc(src, size);
        let value = if size == Size::Word {
            self.sign_extend(raw, Size::Word)
        } else {
            raw
        };
        let dst_value = self.read_a(u32::from((opcode >> 9) & 7));
        let r = self.b.binary(Opcode::SUB, Type::I32, dst_value, value);
        let x = self.ccr_bits().x;
        let (c, v) = self.sub_flags(value, dst_value, r, Size::Long);
        let n = self.is_negative(r, Size::Long);
        let z = self.is_zero(r, Size::Long);
        self.set_ccr(Ccr { x, n, z, v, c });
        self.internal(2);
        self.settle();
        Flow::Continue
    }

    fn op_addx(&mut self, opcode: u16, size: Size, add: bool) -> Flow {
        let f = self.ccr_bits();
        let x_in = f.x;
        let was_zero = f.z;
        let memory = opcode & 0x0008 != 0;
        let (src_value, dst) = if memory {
            let y = u32::from(opcode & 7);
            let xr = u32::from((opcode >> 9) & 7);
            // "One pair of internal cycles for the whole instruction, not one
            // per predecrement: the second address calculation overlaps the
            // first operand's fetch (MC68000UM Table 8-8)."
            self.internal(2);
            let src_value = self.read_predecrement(y, size);
            let dst_value = self.read_predecrement(xr, size);
            let addr = self.read_a(xr);
            (src_value, Loc::Prefetched(addr, dst_value))
        } else {
            let raw = self.read_d(u32::from(opcode & 7));
            let src_value = self.masked(raw, size);
            (src_value, Loc::D(u32::from((opcode >> 9) & 7)))
        };
        let dst_value = self.read_loc(dst, size);
        let x_wide = self.b.unary(Opcode::EXT_Z, Type::I32, x_in);
        let result = if add {
            let a = self.b.binary(Opcode::ADD, Type::I32, dst_value, src_value);
            let b = self.b.binary(Opcode::ADD, Type::I32, a, x_wide);
            self.masked(b, size)
        } else {
            let a = self.b.binary(Opcode::SUB, Type::I32, dst_value, src_value);
            let b = self.b.binary(Opcode::SUB, Type::I32, a, x_wide);
            self.masked(b, size)
        };
        let (c, v) = if add {
            self.add_flags(src_value, dst_value, result, size)
        } else {
            self.sub_flags(src_value, dst_value, result, size)
        };
        let n = self.is_negative(result, size);
        // "Z is only ever *cleared* by an extended operation: a multi-precision
        // sum is zero only if every step of it was, so a zero result leaves Z
        // exactly as the previous step left it (M68000PRM, ADDX)." That is the
        // loop-carried dependency `src/ir/mod.rs`'s decision 1 names, and it
        // is one `and` of two ordinary temporaries.
        let now_zero = self.is_zero(result, size);
        let z = self.b.binary(Opcode::AND, Type::I1, now_zero, was_zero);
        self.set_ccr(Ccr { x: c, n, z, v, c });
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(4);
        }
        if let Loc::D(_) = dst {
            self.write_loc(dst, size, result);
            self.settle();
            return Flow::Continue;
        }
        // A long extended result is two stores with the prefetch between them,
        // and `classify` declined it; only byte and word reach here.
        self.settle();
        self.write_back(dst, size, result);
        Flow::Continue
    }

    /// "`-(An)` as `ADDX`, `SUBX`, `ABCD` and `SBCD` perform it: **one word at
    /// a time**" (`exec::read_predecrement`). A long operand is two separate
    /// steps, which is why the low half comes off the bus first.
    fn read_predecrement(&mut self, reg: u32, size: Size) -> Temp {
        if size != Size::Long {
            let base = self.read_a(reg);
            let addr = self.offset(base, Self::step(size, reg).wrapping_neg());
            self.write_a(reg, addr);
            return self.read_loc(Loc::Mem(addr), size);
        }
        let base = self.read_a(reg);
        let low_at = self.offset(base, 2u32.wrapping_neg());
        self.write_a(reg, low_at);
        let low = self.read_bus(low_at, Size::Word, Sign::Unsigned);
        let high_at = self.offset(low_at, 2u32.wrapping_neg());
        self.write_a(reg, high_at);
        let high = self.read_bus(high_at, Size::Word, Sign::Unsigned);
        let sh = self.konst(16);
        let up = self.b.binary(Opcode::SHL, Type::I32, high, sh);
        self.b.binary(Opcode::OR, Type::I32, up, low)
    }

    fn op_unary(&mut self, insn: Insn, opcode: u16, size: Size) -> Flow {
        let Some(dst) = self.resolve(insn.dst, opcode, size) else {
            return Flow::Rejected;
        };
        // "CLR still reads its destination on a 68000 — the read is a real bus
        // cycle and a device can see it (MC68000UM Table 8-6, and the reason
        // CLR is not usable on a read-sensitive register)."
        let value = self.read_loc(dst, size);
        let result = match insn.op {
            Op::Clr => {
                let x = self.ccr_bits().x;
                let off = self.kbit(false);
                let on = self.kbit(true);
                self.set_ccr(Ccr {
                    x,
                    n: off,
                    z: on,
                    v: off,
                    c: off,
                });
                self.konst(0)
            }
            Op::Not => {
                let raw = self.b.unary(Opcode::NOT, Type::I32, value);
                let r = self.masked(raw, size);
                self.set_logic_flags(r, size);
                r
            }
            Op::Neg => {
                let zero = self.konst(0);
                let raw = self.b.binary(Opcode::SUB, Type::I32, zero, value);
                let r = self.masked(raw, size);
                let (c, v) = self.sub_flags(value, zero, r, size);
                let n = self.is_negative(r, size);
                let z = self.is_zero(r, size);
                self.set_ccr(Ccr { x: c, n, z, v, c });
                r
            }
            // NEGX
            _ => {
                let f = self.ccr_bits();
                let x_wide = self.b.unary(Opcode::EXT_Z, Type::I32, f.x);
                let zero = self.konst(0);
                let a = self.b.binary(Opcode::SUB, Type::I32, zero, value);
                let raw = self.b.binary(Opcode::SUB, Type::I32, a, x_wide);
                let r = self.masked(raw, size);
                let (c, v) = self.sub_flags(value, zero, r, size);
                let n = self.is_negative(r, size);
                let now_zero = self.is_zero(r, size);
                let z = self.b.binary(Opcode::AND, Type::I1, now_zero, f.z);
                self.set_ccr(Ccr { x: c, n, z, v, c });
                r
            }
        };
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result);
            self.settle();
        } else {
            self.settle();
            self.write_back(dst, size, result);
        }
        Flow::Continue
    }

    fn op_move_ccr(&mut self, insn: Insn, opcode: u16, from_sr: bool) -> Flow {
        if from_sr {
            // `MOVE from SR` is unprivileged on a 68000 and moves the whole
            // register; `MOVE from CCR` is a 68010 addition and no 68000
            // encoding reaches it.
            let Some(dst) = self.resolve_ea(Arg::Ea, opcode, Size::Word, Extra::Operand) else {
                return Flow::Rejected;
            };
            let sr = self.read_sr();
            let value = self.masked(sr, Size::Word);
            // "The 68000 reads the destination first, which is why MOVE from
            // SR is a read-modify-write and the 68010 replaced it (MC68000UM
            // Table 8-6)." A register destination's read costs nothing and
            // two internal cycles are charged instead.
            let _ = self.read_loc(dst, Size::Word);
            if matches!(dst, Loc::D(_)) {
                self.internal(2);
                self.write_loc(dst, Size::Word, value);
                self.settle();
            } else {
                self.settle();
                self.write_loc(dst, Size::Word, value);
            }
            return Flow::Continue;
        }
        // `MOVE to CCR` takes a word source and writes the low five bits.
        let Some(src) = self.resolve(insn.src, opcode, Size::Word) else {
            return Flow::Rejected;
        };
        let value = self.read_loc(src, Size::Word);
        let sr = self.read_sr();
        let keep = self.konst(u32::from(!flags::CCR));
        let hi = self.b.binary(Opcode::AND, Type::I32, sr, keep);
        let m = self.konst(u32::from(flags::CCR));
        let lo = self.b.binary(Opcode::AND, Type::I32, value, m);
        let merged = self.b.binary(Opcode::OR, Type::I32, hi, lo);
        self.write_sr(merged);
        // "Both forms *reload* the queue rather than sliding it. Writing SR
        // can change the privilege state, and the word already in the queue
        // was fetched with the old function code — so the 68000 fetches it
        // again, which is visible on FC0-FC2 as well as in the cycle count"
        // (`exec::op_move_to_sr`). The reload's target is `pc + 2`, which is
        // this instruction's own successor, so the block may go on.
        self.internal(4);
        let next = self.pc_model.wrapping_add(2);
        self.refill(next, 0);
        Flow::Continue
    }

    fn op_bit(&mut self, insn: Insn, opcode: u16, size: Size) -> Flow {
        // The bit number is one word of immediate when the source is a static
        // bit number, and a data register otherwise. `Arg::BitNumber` is
        // "*always* one word, while the bit instructions' operand size is long
        // when they address a data register" (`isa::Arg`).
        let static_bit = match insn.src {
            Arg::BitNumber => {
                let w = self.ext(0);
                Some(u32::from(w & 0xff))
            }
            _ => None,
        };
        let number = match static_bit {
            Some(v) => self.konst(v),
            None => self.read_d(u32::from((opcode >> 9) & 7)),
        };
        let Some(dst) = self.resolve(insn.dst, opcode, size) else {
            return Flow::Rejected;
        };
        // "Long when the destination is a data register, byte otherwise"
        // (`isa::SizeSpec::BitOp`), and the bit number is reduced modulo the
        // operand's width — M68000PRM, *BTST*: "if a data register is the
        // destination, then the bit numbering is modulo 32 ... if a memory
        // location is the destination, a byte is read ... and the bit
        // numbering is modulo 8".
        let width = if size == Size::Long { 31 } else { 7 };
        let wm = self.konst(width);
        let bitno = self.b.binary(Opcode::AND, Type::I32, number, wm);
        let one = self.konst(1);
        let mask = self.b.binary(Opcode::SHL, Type::I32, one, bitno);
        let value = self.read_loc(dst, size);
        let and = self.b.binary(Opcode::AND, Type::I32, value, mask);
        let zero = self.konst(0);
        let z = self.b.setcond(IrCond::Eq, Type::I32, and, zero);
        // Only **Z** is written: "Z — set if the bit tested is zero, cleared
        // otherwise. X, N, V, C — not affected" (M68000PRM, *BTST*).
        let f = self.ccr_bits();
        self.set_ccr(Ccr {
            x: f.x,
            n: f.n,
            z,
            v: f.v,
            c: f.c,
        });
        if insn.op == Op::Btst {
            // "A long test costs two more, and so does one whose operand is an
            // immediate: the bit number has to be reduced modulo the operand
            // size either way, and nothing else is happening" (`exec::op_bit`,
            // from MC68000UM Table 8-9).
            if size == Size::Long || matches!(ea_of(insn.dst, opcode), Some((Mode::Imm, _))) {
                self.internal(2);
            }
            self.settle();
            return Flow::Continue;
        }
        let result = match insn.op {
            Op::Bset => self.b.binary(Opcode::OR, Type::I32, value, mask),
            Op::Bclr => self.b.binary(Opcode::ANDC, Type::I32, value, mask),
            _ => self.b.binary(Opcode::XOR, Type::I32, value, mask),
        };
        if matches!(dst, Loc::D(_)) {
            // "A long bit operation on a register costs two more, and BCLR two
            // more again (MC68000UM Table 8-7)" — and the "two more" turns on
            // the *reduced bit number*, which is why `classify` declines a
            // dynamic bit number into a register: that charge would have to be
            // conditional, and `Opcode::CHARGE` carries an immediate.
            let bit = static_bit.map_or(0, |v| v & width);
            self.internal(if bit >= 16 { 4 } else { 2 });
            if insn.op == Op::Bclr {
                self.internal(2);
            }
            self.write_loc(dst, size, result);
            self.settle();
        } else {
            self.settle();
            self.write_back(dst, size, result);
        }
        Flow::Continue
    }

    fn op_shift(&mut self, insn: Insn, opcode: u16, size: Size, count: u32, memory: bool) -> Flow {
        if memory {
            // "The memory form shifts one bit of one word."
            let Some(dst) = self.resolve_ea(Arg::Ea, opcode, Size::Word, Extra::Operand) else {
                return Flow::Rejected;
            };
            let value = self.read_loc(dst, Size::Word);
            let result = self.shift(insn.op, value, 1, Size::Word);
            self.settle();
            self.write_back(dst, Size::Word, result);
            return Flow::Continue;
        }
        let n = u32::from(opcode & 7);
        let old = self.read_d(n);
        let value = self.masked(old, size);
        let result = self.shift(insn.op, value, count, size);
        let merged = self.merge(old, result, size);
        self.write_d(n, merged);
        // "Two cycles per bit, on top of the two (word) or four (long) the
        // instruction costs before it starts (MC68000UM Table 8-12)."
        self.internal(if size == Size::Long { 4 } else { 2 } + 2 * count);
        self.settle();
        Flow::Continue
    }

    /// One shift or rotate by a count known at lift time, setting the flags
    /// the manual gives it.
    ///
    /// `exec::shift` is the second oracle and the loop is unrolled here: the
    /// count is a decode constant between 1 and 8, so every per-bit rule —
    /// `ASL`'s "V is set if the sign bit changed at *any* point in the shift,
    /// not just at the end (M68000PRM, ASL)" among them — becomes a chain of
    /// ordinary temporaries rather than a run-time loop. The `count > bits`
    /// case cannot arise for a static count of at most eight, so the
    /// interpreter's `exhausted` rule is unreachable here and the shift by a
    /// register count — which is where it *does* arise — is declined by
    /// `classify`.
    fn shift(&mut self, op: Op, value: Temp, count: u32, size: Size) -> Temp {
        let sign = match size {
            Size::Byte => 7u32,
            Size::Word => 15,
            Size::Long => 31,
        };
        let bits = sign + 1;
        debug_assert!(count >= 1 && count <= bits, "a static shift count is 1..=8");
        let mut result = value;
        let mut carry = self.kbit(false);
        let mut overflow = self.kbit(false);
        let one = self.konst(1);
        match op {
            Op::Asl | Op::Lsl => {
                for _ in 0..count {
                    carry = self.bit_of(result, sign);
                    let shifted = self.b.binary(Opcode::SHL, Type::I32, result, one);
                    let next = self.masked(shifted, size);
                    if op == Op::Asl {
                        let changed = self.b.binary(Opcode::XOR, Type::I32, next, result);
                        let flipped = self.bit_of(changed, sign);
                        overflow = self.b.binary(Opcode::OR, Type::I1, overflow, flipped);
                    }
                    result = next;
                }
            }
            Op::Asr => {
                for _ in 0..count {
                    carry = self.bit_of(result, 0);
                    // An arithmetic right shift of the *operand's* width, not
                    // of 32 bits: the value is already masked, so the sign is
                    // re-injected explicitly.
                    let logical = self.b.binary(Opcode::SHR, Type::I32, result, one);
                    let sm = self.konst(1u32 << sign);
                    let keep = self.b.binary(Opcode::AND, Type::I32, result, sm);
                    result = self.b.binary(Opcode::OR, Type::I32, logical, keep);
                }
            }
            Op::Lsr => {
                for _ in 0..count {
                    carry = self.bit_of(result, 0);
                    result = self.b.binary(Opcode::SHR, Type::I32, result, one);
                }
            }
            Op::Rol => {
                for _ in 0..count {
                    carry = self.bit_of(result, sign);
                    let shifted = self.b.binary(Opcode::SHL, Type::I32, result, one);
                    let masked = self.masked(shifted, size);
                    let bit = self.b.unary(Opcode::EXT_Z, Type::I32, carry);
                    result = self.b.binary(Opcode::OR, Type::I32, masked, bit);
                }
            }
            Op::Ror => {
                for _ in 0..count {
                    carry = self.bit_of(result, 0);
                    let shifted = self.b.binary(Opcode::SHR, Type::I32, result, one);
                    let bit = self.b.unary(Opcode::EXT_Z, Type::I32, carry);
                    let sh = self.konst(sign);
                    let top = self.b.binary(Opcode::SHL, Type::I32, bit, sh);
                    result = self.b.binary(Opcode::OR, Type::I32, shifted, top);
                }
            }
            // ROXL and ROXR: the (N+1)-bit rotate through **X**, which is the
            // reason `Opcode::ROTLC`/`ROTRC` exist (`src/ir/mod.rs`, decision
            // 6, which names `ROXL`/`ROXR` among the six ISAs that need it).
            // They are not used here because the IR's rotate-through-carry is
            // a rotate of the *type's* width and these rotate through the
            // operand's, which is eight or sixteen bits as often as
            // thirty-two.
            Op::Roxl => {
                let mut x = self.ccr_bits().x;
                for _ in 0..count {
                    carry = self.bit_of(result, sign);
                    let shifted = self.b.binary(Opcode::SHL, Type::I32, result, one);
                    let masked = self.masked(shifted, size);
                    let bit = self.b.unary(Opcode::EXT_Z, Type::I32, x);
                    result = self.b.binary(Opcode::OR, Type::I32, masked, bit);
                    x = carry;
                }
                let n = self.is_negative(result, size);
                let z = self.is_zero(result, size);
                let off = self.kbit(false);
                self.set_ccr(Ccr {
                    x,
                    n,
                    z,
                    v: off,
                    c: carry,
                });
                return result;
            }
            _ => {
                let mut x = self.ccr_bits().x;
                for _ in 0..count {
                    carry = self.bit_of(result, 0);
                    let shifted = self.b.binary(Opcode::SHR, Type::I32, result, one);
                    let bit = self.b.unary(Opcode::EXT_Z, Type::I32, x);
                    let sh = self.konst(sign);
                    let top = self.b.binary(Opcode::SHL, Type::I32, bit, sh);
                    result = self.b.binary(Opcode::OR, Type::I32, shifted, top);
                    x = carry;
                }
                let n = self.is_negative(result, size);
                let z = self.is_zero(result, size);
                let off = self.kbit(false);
                self.set_ccr(Ccr {
                    x,
                    n,
                    z,
                    v: off,
                    c: carry,
                });
                return result;
            }
        }
        let n = self.is_negative(result, size);
        let z = self.is_zero(result, size);
        let off = self.kbit(false);
        let v = if op == Op::Asl { overflow } else { off };
        // "A plain rotate does not touch X — only the shifts and the two
        // rotate-through-extend forms do... Getting this wrong quietly breaks
        // every multi-precision routine that rotates a mask between `ADDX`
        // steps."
        let x = if matches!(op, Op::Asl | Op::Asr | Op::Lsl | Op::Lsr) {
            carry
        } else {
            self.ccr_bits().x
        };
        self.set_ccr(Ccr {
            x,
            n,
            z,
            v,
            c: carry,
        });
        result
    }

    fn op_movem_load(&mut self, opcode: u16, size: Size) -> Flow {
        let mask = self.ext(0);
        let Some((mode, reg)) = ea_of(Arg::Ea, opcode) else {
            return Flow::Rejected;
        };
        let reg = u32::from(reg);
        let long = size == Size::Long;
        let walking = mode == Mode::PostInc;
        let mut addr = if walking {
            self.read_a(reg)
        } else {
            let Some(Loc::Mem(addr)) = self.resolve_ea(Arg::Ea, opcode, size, Extra::Operand)
            else {
                return Flow::Rejected;
            };
            addr
        };
        let stride = if long { 4 } else { 2 };
        for bit in 0..16u32 {
            if mask & (1 << bit) == 0 {
                continue;
            }
            if walking {
                let next = self.offset(addr, 2);
                self.write_a(reg, next);
            }
            let high = self.read_bus(addr, Size::Word, Sign::Unsigned);
            let value = if long {
                if walking {
                    let next = self.offset(addr, 4);
                    self.write_a(reg, next);
                }
                let at = self.offset(addr, 2);
                let low = self.read_bus(at, Size::Word, Sign::Unsigned);
                let sh = self.konst(16);
                let up = self.b.binary(Opcode::SHL, Type::I32, high, sh);
                self.b.binary(Opcode::OR, Type::I32, up, low)
            } else {
                // "A word `MOVEM` sign-extends into the whole register"
                // (M68000PRM, *MOVEM*: "word transfers to either address or
                // data registers are sign extended to 32 bits").
                self.sign_extend(high, Size::Word)
            };
            addr = self.offset(addr, stride);
            self.set_register(bit, value);
        }
        // "One word past the end, read and discarded. It is a real bus cycle,
        // and a MOVEM that ends at the top of a mapped region can fault on it."
        if walking {
            let next = self.offset(addr, 2);
            self.write_a(reg, next);
        }
        let _ = self.read_bus(addr, Size::Word, Sign::Unsigned);
        if walking {
            self.write_a(reg, addr);
        }
        self.settle();
        Flow::Continue
    }

    /// One of the sixteen registers a `MOVEM` mask names: `D0`-`D7` then
    /// `A0`-`A7`.
    fn set_register(&mut self, index: u32, value: Temp) {
        if index < 8 {
            self.write_d(index, value);
        } else {
            self.write_a(index - 8, value);
        }
    }

    /// A precise side exit: do `work`, then leave the block for `exit_pc`
    /// with `live` as the outgoing register map.
    ///
    /// The shape `cpu::riscv::lift`'s `side_exit` uses, for the same three
    /// reasons: the boundary records stay in program order so
    /// [`InsnStart::ticks`] stays monotonic and the verifier's check on it
    /// keeps working; every [`Opcode::BRCOND`] stays a *forward* branch, which
    /// is what `ir::pass`'s single backward liveness walk is built on; and the
    /// exit's live map is taken exactly where the path diverges, which is what
    /// makes leaving through it architecturally precise.
    ///
    /// `work`'s charges are **path-local**: `self.ticks` and `self.pc_model`
    /// are restored afterwards, so a later path's static column counts its own
    /// cycles rather than this one's. That is what lets `DBcc`'s three paths
    /// — 10, 12 and 14 cycles — live in one block, and it is why the paths are
    /// emitted in increasing-cost order: the verifier refuses a boundary whose
    /// tick column is lower than its predecessor's.
    fn path(
        &mut self,
        work: impl FnOnce(&mut Lifter),
        exit_pc: u32,
        overrides: &[(RegSlot, Temp)],
    ) {
        let ticks = self.ticks;
        let pc_model = self.pc_model;
        let queue = self.p;
        work(self);
        // Taken *after* `work`, so the prefetch words this path fetched are the
        // ones it publishes. Every other slot is unchanged by a path, because
        // a path only slides, charges and leaves.
        let mut live = self.live_regs();
        for &(slot, temp) in overrides {
            match live.iter_mut().find(|(s, _)| *s == slot) {
                Some(entry) => entry.1 = temp,
                None => live.push((slot, temp)),
            }
        }
        live.sort_by_key(|(slot, _)| slot.0);
        let target = self.konst(exit_pc);
        live.push((PC, target));
        self.b.insn_start(InsnStart {
            pc: u64::from(exit_pc),
            next_pc: u64::from(exit_pc),
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        self.ticks = ticks;
        self.pc_model = pc_model;
        self.p = queue;
    }

    /// Branch forward over what follows when `when` holds — a
    /// [`Opcode::BRCOND`] on a one-bit selector, whose target is patched once
    /// the skipped sequence exists. Returns the instruction index to patch.
    fn brcond(&mut self, when: Temp) -> usize {
        self.b
            .emit_raw(Opcode::BRCOND, Type::I1, None, None, &[when], None, None, 0)
    }

    /// Patch the branch at `over` to land at the next instruction emitted.
    fn land(&mut self, over: usize) {
        let at = self.b.next_index() as u32;
        self.b.patch_aux(over, at);
    }

    /// `Bcc`, `BRA`, and their word-displacement forms.
    ///
    /// Every path leaves the block, so the two sides are two *exits* rather
    /// than a rejoin, and each spends its own cycles. The sequences are
    /// `exec::op_branch`'s and they reproduce MC68000UM Table 8-14's published
    /// times: a taken branch is `internal(2)` plus a two-word refill — ten
    /// cycles — and a not-taken one is `internal(4)` plus the fetches it still
    /// makes, which is eight for the byte form and twelve for the word form,
    /// because "a word displacement that is not taken still costs the fetch".
    fn op_branch(
        &mut self,
        opcode: u16,
        target: u32,
        always: bool,
        word: bool,
        next_pc: u32,
    ) -> Flow {
        if always {
            // `BRA`/`BRA.W` has one path. The word form reads its displacement
            // out of the queue with no refill of its own, exactly as a jump
            // does, so it costs what the byte form costs.
            self.internal(2);
            self.refill(target, 0);
            self.static_exit = Some(target);
            return Flow::Transfer;
        }
        let taken = self.test(Cond::from_opcode(opcode));
        self.closed = true;
        // The taken path's static column is 2 and the not-taken path's is 4,
        // whichever displacement form this is, so taken goes first — and the
        // branch that skips it is therefore on the *negation*.
        let not_taken = self.b.unary(Opcode::NOT, Type::I1, taken);
        let over = self.brcond(not_taken);
        self.path(
            |lf| {
                lf.internal(2);
                lf.refill(target, 0);
            },
            target,
            &[],
        );
        self.land(over);
        self.path(
            move |lf| {
                lf.internal(4);
                if word {
                    // "A word displacement that is not taken still costs the
                    // fetch": the word is consumed and the queue refilled.
                    let _ = lf.take_word();
                    lf.slide();
                }
                lf.settle();
            },
            next_pc,
            &[],
        );
        Flow::Transfer
    }

    /// `DBcc`: three paths, three cycle counts, and the counter written on two
    /// of them.
    ///
    /// `exec::op_dbcc`, whose numbers are MC68000UM Table 8-14's: the
    /// condition true is 12, the condition false with the counter expired is
    /// 14, and the condition false with the loop continuing is 10.
    ///
    /// The counter is computed **unconditionally** — it is pure arithmetic
    /// with no side effect, so computing it on a path that does not use it
    /// costs nothing observable — and each exit's live map then names either
    /// the old temporary or the new one. That is how the path which must *not*
    /// write `Dn` avoids writing it, without a conditional rebinding the IR
    /// has no way to express.
    fn op_dbcc(&mut self, opcode: u16, target: u32, next_pc: u32) -> Flow {
        let n = u32::from(opcode & 7);
        self.closed = true;
        let cond = self.test(Cond::from_opcode(opcode));
        let old = self.read_d(n);
        let one = self.konst(1);
        let dec = self.b.binary(Opcode::SUB, Type::I32, old, one);
        let counter = self.merge(old, dec, Size::Word);
        // "if counter == 0xffff" — the word counter *after* the decrement, so
        // a loop entered with zero runs 65 536 times rather than not at all.
        let low = self.masked(counter, Size::Word);
        let all_ones = self.konst(0xffff);
        let expired = self.b.setcond(IrCond::Eq, Type::I32, low, all_ones);
        let not_cond = self.b.unary(Opcode::NOT, Type::I1, cond);
        let not_expired = self.b.unary(Opcode::NOT, Type::I1, expired);
        let loops = self.b.binary(Opcode::AND, Type::I1, not_cond, not_expired);

        // The counter is written on two of the three paths, and `read_d`
        // above bound `Dn` to its *old* temporary — so the default live map is
        // the one the condition-true path wants, and the other two override
        // the one slot.
        let wrote = [(d_slot(n), counter)];

        // In increasing static-column order: the loop (2), the condition true
        // (4), the counter expired (6). Each branch skips the path below it on
        // the *negation* of that path's own predicate.
        let not_loops = self.b.unary(Opcode::NOT, Type::I1, loops);
        let over_loop = self.brcond(not_loops);
        // The loop is taken: the displacement comes out of the queue with no
        // refill of its own, exactly as a branch's does.
        self.path(
            |lf| {
                lf.internal(2);
                lf.refill(target, 0);
            },
            target,
            &wrote,
        );
        self.land(over_loop);
        // Not the loop, so either the condition held or the counter expired.
        let over_true = self.brcond(not_cond);
        // The condition is true: the loop is over and the counter is left
        // alone, which is why this path's map is `base`.
        self.path(
            |lf| {
                lf.internal(4);
                let _ = lf.take_word();
                lf.slide();
                lf.settle();
            },
            next_pc,
            &[],
        );
        self.land(over_true);
        // The counter expired: the instruction falls through, writes the
        // counter, and pays for its two fetches.
        self.path(
            |lf| {
                lf.internal(6);
                let _ = lf.take_word();
                lf.slide();
                lf.settle();
            },
            next_pc,
            &wrote,
        );
        Flow::Transfer
    }

    fn op_scc(&mut self, opcode: u16) -> Flow {
        let set = self.test(Cond::from_opcode(opcode));
        let Some(dst) = self.resolve_ea(Arg::Ea, opcode, Size::Byte, Extra::Operand) else {
            return Flow::Rejected;
        };
        let ones = self.konst(0xff);
        let zero = self.konst(0);
        let value = self.b.emit(Opcode::MOVCOND, Type::I32, &[set, ones, zero]);
        if !matches!(dst, Loc::D(_)) {
            // "A memory destination is read before it is written, exactly as
            // CLR reads one: the 68000 has no write-only bus cycle, and a
            // read-sensitive register notices."
            let _ = self.read_loc(dst, Size::Byte);
            self.settle();
            self.write_back(dst, Size::Byte, value);
            return Flow::Continue;
        }
        // "Two extra cycles when the byte is set, which is the one place a
        // 68000's timing depends on a condition (MC68000UM Table 8-11)."
        // `classify` only lets `ST` and `SF` reach here with a register
        // destination, so the condition is a decode constant and the charge
        // stays an immediate; every other condition into a register falls back
        // rather than guessing which of the two counts to emit.
        if opcode >> 8 & 0xf == 0x0 {
            self.internal(2);
        }
        self.write_loc(dst, Size::Byte, value);
        self.settle();
        Flow::Continue
    }

    fn op_jmp(&mut self, opcode: u16, pc: u32) -> Flow {
        let Some((mode, reg)) = ea_of(Arg::Ea, opcode) else {
            return Flow::Rejected;
        };
        let reg = u32::from(reg);
        // `Exec::jump_target`: "The **last extension word is taken straight
        // out of the prefetch queue with no refill**: the queue is about to be
        // reloaded from the target, so paying for a fetch that will be thrown
        // away would be a wasted bus cycle and the 68000 does not make one —
        // which is why `JMP (d16,An)` is ten cycles and not fourteen."
        let target = match mode {
            Mode::Indirect => self.read_a(reg),
            Mode::Disp16 => {
                let queued = self.words.get(1).copied().unwrap_or(0);
                self.internal(2);
                let a = self.read_a(reg);
                self.offset(a, i32::from(queued as i16) as u32)
            }
            Mode::Index8 => {
                let queued = self.words.get(1).copied().unwrap_or(0);
                self.internal(6);
                let a = self.read_a(reg);
                self.index_address(a, queued)
            }
            Mode::AbsShort => {
                let queued = self.words.get(1).copied().unwrap_or(0);
                self.internal(2);
                self.konst(i32::from(queued as i16) as u32)
            }
            Mode::AbsLong => {
                // "Two words, and only the first of them is worth a refill."
                let hi = self.ext(0);
                let lo = self.words.get(2).copied().unwrap_or(0);
                self.konst((u32::from(hi) << 16) | u32::from(lo))
            }
            Mode::PcDisp16 => {
                let queued = self.words.get(1).copied().unwrap_or(0);
                self.internal(2);
                let base = pc.wrapping_add(2);
                self.konst(base.wrapping_add(i32::from(queued as i16) as u32))
            }
            Mode::PcIndex8 => {
                let queued = self.words.get(1).copied().unwrap_or(0);
                self.internal(6);
                let base = self.konst(pc.wrapping_add(2));
                self.index_address(base, queued)
            }
            _ => return Flow::Rejected,
        };
        self.refill_at(target, 0);
        self.pc_out = Some(target);
        Flow::Transfer
    }

    // -- closing --------------------------------------------------------

    /// Close the block: the exit boundary, then the terminator.
    ///
    /// The exit boundary begins no guest instruction. It carries the outgoing
    /// register map and the [`PC`] slot, which is the only thing that tells
    /// the engine where to resume; its `pc` field is the exit PC where that is
    /// a constant, and the program-order continuation otherwise.
    fn close(mut self, program_order_pc: u32, insns: usize, stop: Stop) -> Lifted {
        if self.closed {
            // Every path of the last instruction lifted ended in its own exit
            // boundary and terminator, so the block is already well formed and
            // already ends in one.
            return Lifted {
                block: self.b.finish(),
                stop,
                insns,
                end_pc: program_order_pc,
            };
        }
        let at = self.static_exit.unwrap_or(program_order_pc);
        let pc = match self.pc_out {
            Some(t) => t,
            None => self.konst(at),
        };
        let mut live = self.live_regs();
        live.push((PC, pc));
        self.b.insn_start(InsnStart {
            pc: u64::from(at),
            next_pc: u64::from(at),
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        Lifted {
            block: self.b.finish(),
            stop,
            insns,
            end_pc: at,
        }
    }
}

/// The five condition-code bits, as one-bit temporaries.
#[derive(Debug, Clone, Copy)]
struct Ccr {
    x: Temp,
    n: Temp,
    z: Temp,
    v: Temp,
    c: Temp,
}

/// Which microcode cycles an effective-address calculation spends, which
/// depends on what the instruction is doing with it (`exec::ExtraCycles`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Extra {
    /// An ordinary operand.
    Operand,
    /// A `MOVE` destination, whose predecrement costs nothing and whose
    /// postincrement is owed until the write lands.
    MoveDest,
    /// An address the instruction only wants the value of — `LEA`.
    Control,
}

impl Extra {
    /// The internal cycles an indexed mode spends before its refill.
    const fn index_delay(self) -> u32 {
        match self {
            // `LEA (d8,An,Xn)` spends two more than an operand fetch does
            // (MC68000UM Table 8-1 against Table 8-13).
            Extra::Control => 4,
            _ => 2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::verify;
    use alloc::vec;

    /// A source over a slice of words loaded at [`BASE`].
    struct Words<'a>(&'a [u16]);

    impl InsnSource for Words<'_> {
        fn word(&mut self, addr: u32) -> Option<u16> {
            let off = addr.checked_sub(BASE)? / 2;
            self.0.get(off as usize).copied()
        }
    }

    /// Where these tests lift from: inside a [`WINDOW`], so a block that ends
    /// at the window bound does so because a test asked it to.
    const BASE: u32 = 0x1000;

    fn lift_at(words: &[u16], max: usize) -> Lifted {
        let mut src = Words(words);
        lift(Model::M68000, BASE, &mut src, max).expect("a 68000 lifts")
    }

    fn lifted(words: &[u16]) -> Lifted {
        lift_at(words, MAX_INSNS)
    }

    /// The opcode mnemonics of a block, in order.
    fn ops(block: &Block) -> Vec<&'static str> {
        block.insts().iter().map(|i| i.op.name()).collect()
    }

    /// The sum of a block's [`Opcode::CHARGE`] immediates.
    fn static_ticks(block: &Block) -> u64 {
        block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::CHARGE)
            .map(|i| i.imm.map_or(0, |c| c.bits() as u64))
            .sum()
    }

    /// How many bus accesses a block makes, by kind.
    fn accesses(block: &Block) -> (usize, usize, usize) {
        let mut fetches = 0;
        let mut loads = 0;
        let mut stores = 0;
        for inst in block.insts() {
            match (inst.op, inst.mem.map(|m| m.kind)) {
                (Opcode::LD, Some(AccessKind::Fetch)) => fetches += 1,
                (Opcode::LD, Some(AccessKind::Load)) => loads += 1,
                (Opcode::ST, _) => stores += 1,
                _ => {}
            }
        }
        (fetches, loads, stores)
    }

    #[test]
    fn a_lifted_block_verifies_and_ends_in_a_terminator() {
        // MOVE.L D0,D1 then `MULU.W D1,D0`, which is outside the subset.
        let l = lifted(&[0x2200, 0xc0c1]);
        verify(&l.block).expect("the frontend produces well-formed blocks");
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Unsupported);
        assert_eq!(l.block.insts().last().map(|i| i.op), Some(Opcode::EXIT_TB));
    }

    #[test]
    fn a_block_whose_first_instruction_is_outside_the_subset_is_still_wellformed() {
        // MULU.W D1,D0 — data-dependent cycles, not lifted.
        let l = lifted(&[0xc0c1]);
        assert_eq!(l.insns, 0);
        verify(&l.block).expect("a block that lifts nothing is still a block");
        assert_eq!(l.end_pc, BASE);
    }

    #[test]
    fn an_odd_entry_pc_is_refused_rather_than_lifted() {
        // An odd program counter is an address error on the *fetch*, and the
        // entry word could not have been read at lift time either.
        let mut src = Words(&[0x4e71]);
        let l = lift(Model::M68000, BASE + 1, &mut src, MAX_INSNS).expect("a 68000");
        assert_eq!(l.insns, 0);
        verify(&l.block).expect("well formed");
    }

    #[test]
    fn a_model_the_frontend_does_not_lift_is_refused_outright() {
        for model in [
            Model::M68010,
            Model::M68020,
            Model::M68EC020,
            Model::M68030,
            Model::M68040,
        ] {
            let mut src = Words(&[0x4e71]);
            let err = lift(model, BASE, &mut src, MAX_INSNS)
                .expect_err("only a 68000 has an IR frontend");
            assert!(
                alloc::format!("{err}").contains("MC68000 only"),
                "{err} for {model}"
            );
        }
    }

    #[test]
    fn a_block_never_leaves_the_window_it_started_in() {
        // Sixty-four `NOP`s from two words before the window bound: only the
        // first one fits.
        let program = vec![0x4e71; 64];
        let mut src = Words(&program);
        let at = BASE + WINDOW - 2;
        // The source is relative to `BASE`, so shift it: a reader that returns
        // a `NOP` for every address in the window.
        struct Nops;
        impl InsnSource for Nops {
            fn word(&mut self, _addr: u32) -> Option<u16> {
                Some(0x4e71)
            }
        }
        let _ = &mut src;
        let mut nops = Nops;
        let l = lift(Model::M68000, at, &mut nops, MAX_INSNS).expect("a 68000");
        assert_eq!(l.insns, 1, "one word fits before the bound");
        assert_eq!(l.stop, Stop::Window);
    }

    #[test]
    fn unreadable_bytes_end_the_block_rather_than_inventing_an_encoding() {
        // Two `NOP`s and then nothing: `Words` answers `None` past the slice.
        let l = lifted(&[0x4e71, 0x4e71]);
        assert_eq!(l.insns, 2);
        assert_eq!(l.stop, Stop::Unreadable);
    }

    #[test]
    fn the_instruction_limit_ends_a_block() {
        let program = vec![0x4e71; 40];
        let l = lift_at(&program, 8);
        assert_eq!(l.insns, 8);
        assert_eq!(l.stop, Stop::Limit);
        assert_eq!(l.end_pc, BASE + 16);
    }

    #[test]
    fn a_nop_is_one_fetch_and_nothing_else() {
        // `NOP` is four cycles, which is its final prefetch and nothing more
        // (MC68000UM Table 8-4).
        let l = lifted(&[0x4e71, 0xc0c1]);
        let (fetches, loads, stores) = accesses(&l.block);
        assert_eq!((fetches, loads, stores), (1, 0, 0));
        assert_eq!(static_ticks(&l.block), 0, "no internal cycles at all");
    }

    #[test]
    fn a_register_move_makes_one_fetch_and_no_data_access() {
        // MOVE.L D0,D1 is four cycles: one prefetch.
        let l = lifted(&[0x2200, 0xc0c1]);
        assert_eq!(accesses(&l.block), (1, 0, 0));
        assert_eq!(static_ticks(&l.block), 0);
    }

    #[test]
    fn a_memory_to_memory_move_reads_once_writes_once_and_fetches_once() {
        // MOVE.W (A2)+,(A3)+ — the case the IR's memory-to-memory question is
        // about. Twelve cycles: a read, a write and a prefetch.
        let l = lifted(&[0x36da, 0xc0c1]);
        assert_eq!(accesses(&l.block), (1, 1, 1));
        assert_eq!(static_ticks(&l.block), 0);
        // and the accesses come out in the order the bus sees them. A `MOVE`
        // with a memory destination writes its operand and *then* prefetches
        // — `exec::op_move`'s `write_loc` before its `settle` — which is the
        // opposite of the read-modify-write order two lines of `op_binary`
        // away, and is why the order is asserted rather than assumed.
        let order: Vec<&'static str> = l
            .block
            .insts()
            .iter()
            .filter(|i| i.mem.is_some())
            .map(|i| match i.mem.map(|m| m.kind) {
                Some(AccessKind::Fetch) => "fetch",
                Some(AccessKind::Load) => "load",
                _ => "store",
            })
            .collect();
        assert_eq!(order, vec!["load", "store", "fetch"], "{}", l.block);
    }

    #[test]
    fn a_long_operand_is_two_word_bus_cycles_high_word_first() {
        // "A 68000 has a 16-bit data bus, so there is no such thing as a
        // 32-bit bus cycle; a device watching the bus sees two."
        let l = lifted(&[0xd092, 0xc0c1]); // ADD.L (A2),D0
        let (_, loads, _) = accesses(&l.block);
        assert_eq!(loads, 2, "a long read is two word reads: {}", l.block);
        for inst in l.block.insts() {
            if let Some(mem) = inst.mem {
                assert_eq!(mem.size, Width::U16, "no access is wider than a word");
            }
        }
    }

    #[test]
    fn a_word_access_carries_the_alignment_rule_and_a_byte_access_does_not() {
        let l = lifted(&[0xd052, 0xc0c1]); // ADD.W (A2),D0
        let word = l
            .block
            .insts()
            .iter()
            .find(|i| i.mem.is_some_and(|m| m.kind == AccessKind::Load))
            .and_then(|i| i.mem)
            .expect("there is a load");
        assert_eq!(word.align, Align::Fault, "an odd word is an address error");
        assert_eq!(word.endian, Endian::Big);
        assert!(word.volatile, "a bus cycle survives dead-code elimination");

        let l = lifted(&[0xd012, 0xc0c1]); // ADD.B (A2),D0
        let byte = l
            .block
            .insts()
            .iter()
            .find(|i| i.mem.is_some_and(|m| m.kind == AccessKind::Load))
            .and_then(|i| i.mem)
            .expect("there is a load");
        assert_eq!(byte.align, Align::None, "a byte has no alignment rule");
        assert_eq!(byte.size, Width::U8);
    }

    #[test]
    fn a_fetch_carries_the_alignment_rule_too() {
        // The one fault a charge could not have raised.
        let l = lifted(&[0x4e71, 0xc0c1]);
        let fetch = l
            .block
            .insts()
            .iter()
            .find(|i| i.mem.is_some_and(|m| m.kind == AccessKind::Fetch))
            .and_then(|i| i.mem)
            .expect("there is a fetch");
        assert_eq!(fetch.align, Align::Fault);
        assert!(fetch.volatile);
    }

    #[test]
    fn every_instruction_that_commits_two_stores_is_declined() {
        // The restartability rule, as the encodings it costs.
        let declined: &[(&[u16], &str)] = &[
            (&[0x2092], "MOVE.L (A2),(A0)"),
            (&[0x2192], "MOVE.L (A2),-(A0)"),
            (&[0xd192], "ADD.L D0,(A2)"),
            (&[0xd78a], "ADDX.L -(A2),-(A3)"),
            (&[0x4892, 0x0001], "MOVEM.W D0,(A2)"),
            (&[0x4e92], "JSR (A2)"),
            (&[0x6102], "BSR.B"),
            (&[0x4852], "PEA (A2)"),
            (&[0x4e52, 0xfff0], "LINK A2,#-16"),
        ];
        for (words, what) in declined {
            let l = lifted(words);
            assert_eq!(l.insns, 0, "{what} must not be lifted:\n{}", l.block);
        }
    }

    #[test]
    fn the_things_that_are_lifted_are_lifted() {
        // The other half of the same claim: a documented subset done exactly
        // beats a broad one done approximately, but it still has to be there.
        let taken: &[(&[u16], &str)] = &[
            (&[0x2200], "MOVE.L D0,D1"),
            (&[0x36da], "MOVE.W (A2)+,(A3)+"),
            (&[0x7001], "MOVEQ #1,D0"),
            (&[0xd041], "ADD.W D1,D0"),
            (&[0x9081], "SUB.L D1,D0"),
            (&[0xd141], "ADDX.W D1,D0"),
            (&[0xb108], "CMPM.B (A0)+,(A0)+"),
            (&[0x4252], "CLR.W (A2)"),
            (&[0x4a80], "TST.L D0"),
            (&[0x4880], "EXT.W D0"),
            (&[0x4840], "SWAP D0"),
            (&[0xc141], "EXG D0,D1"),
            (&[0x41d2], "LEA (A2),A0"),
            (&[0x4e5a], "UNLK A2"),
            (&[0x0800, 0x0003], "BTST #3,D0"),
            (&[0xe348], "LSL.W #1,D0"),
            (&[0x4c92, 0x0003], "MOVEM.W (A2),D0-D1"),
            (&[0x6602], "BNE.B +2"),
            (&[0x50c8, 0x0002], "DBT D0,+2"),
            (&[0x50c0], "ST D0"),
            (&[0x4ed2], "JMP (A2)"),
            (&[0x4e75], "RTS"),
            (&[0x4e71], "NOP"),
            (&[0x40c0], "MOVE SR,D0"),
            (&[0x44c0], "MOVE D0,CCR"),
            (&[0x003c, 0x0005], "ORI #5,CCR"),
        ];
        for (words, what) in taken {
            let l = lifted(words);
            assert_eq!(l.insns, 1, "{what} is in the subset:\n{}", l.block);
            verify(&l.block).unwrap_or_else(|e| panic!("{what}: {e}\n{}", l.block));
        }
    }

    #[test]
    fn a_branch_becomes_two_exits_and_the_cheaper_one_comes_first() {
        // Every path leaves the block, so the two sides are two boundaries;
        // and [`InsnStart::ticks`] must not run backwards across them, which
        // is what decides the order.
        let l = lifted(&[0x6604, 0x4e71, 0x4e71]); // BNE.B +4
        verify(&l.block).expect("well formed");
        let marks: Vec<(u64, u64)> = l.block.marks().iter().map(|m| (m.pc, m.ticks)).collect();
        assert_eq!(
            marks,
            vec![
                (u64::from(BASE), 0),
                (u64::from(BASE) + 6, 2),
                (u64::from(BASE) + 2, 4),
            ],
            "taken (2) before not taken (4)"
        );
        assert_eq!(
            l.block
                .insts()
                .iter()
                .filter(|i| i.op == Opcode::EXIT_TB)
                .count(),
            2
        );
        // Every branch inside a block is *forward*, which `ir::pass`'s single
        // backward liveness walk depends on.
        for (i, inst) in l.block.insts().iter().enumerate() {
            if inst.op == Opcode::BRCOND {
                assert!(inst.aux as usize > i, "a brcond must go forward");
            }
        }
    }

    #[test]
    fn dbcc_becomes_three_exits_in_increasing_cost_order() {
        let l = lifted(&[0x56c8, 0xfffe]); // DBNE D0,-2
        verify(&l.block).expect("well formed");
        let ticks: Vec<u64> = l.block.marks().iter().map(|m| m.ticks).collect();
        assert_eq!(ticks, vec![0, 2, 4, 6], "{}", l.block);
        assert_eq!(
            l.block
                .insts()
                .iter()
                .filter(|i| i.op == Opcode::EXIT_TB)
                .count(),
            3
        );
    }

    #[test]
    fn the_tick_column_never_runs_backwards_and_the_verifier_agrees() {
        // A long straight-line block through most of the subset.
        let program = vec![
            0x7001, 0xd041, 0x4a80, 0xe348, 0x4840, 0x2200, 0x4e71, 0x4252, 0x0800, 0x0003,
        ];
        let l = lifted(&program);
        verify(&l.block).expect("well formed");
        let mut last = 0;
        for mark in l.block.marks() {
            assert!(mark.ticks >= last, "{}", l.block);
            last = mark.ticks;
        }
    }

    #[test]
    fn a_boundary_names_only_temporaries_that_already_exist() {
        let program = vec![0x7001, 0xd041, 0x36da, 0x4252, 0x4e71];
        let l = lifted(&program);
        for mark in l.block.marks() {
            for (_, temp) in &mark.live {
                assert!(
                    l.block.type_of(*temp).is_some(),
                    "{temp} is named live and was never allocated"
                );
            }
        }
    }

    #[test]
    fn a_slot_a_boundary_shadows_stays_shadowed_at_every_later_boundary() {
        // [`InsnStart::live`]'s stated invariant, asserted per frontend
        // because the verifier cannot see the branch graph.
        let program = vec![0x7001, 0x7202, 0xd041, 0x36da, 0x4e71];
        let l = lifted(&program);
        let mut seen: Vec<RegSlot> = Vec::new();
        for mark in l.block.marks() {
            for slot in &seen {
                assert!(
                    mark.live.iter().any(|(s, _)| s == slot) || *slot == PC,
                    "slot {slot:?} was shadowed and then dropped at pc {:#x}\n{}",
                    mark.pc,
                    l.block
                );
            }
            for (slot, _) in &mark.live {
                if !seen.contains(slot) {
                    seen.push(*slot);
                }
            }
        }
    }

    #[test]
    fn a_boundarys_live_map_is_in_slot_order() {
        // `ROADMAP.md` §0's determinism rule reaches the IR: this vector is
        // hashed by anything that hashes a block.
        let program = vec![0x7001, 0x7202, 0x36da, 0x6604, 0x4e71, 0x4e71];
        let l = lifted(&program);
        for mark in l.block.marks() {
            let slots: Vec<u16> = mark.live.iter().map(|(s, _)| s.0).collect();
            let mut sorted = slots.clone();
            sorted.sort_unstable();
            // The exit boundaries append `PC` last, which is the highest slot
            // a boundary can name after the queue — so compare the prefix.
            let body = &slots[..slots.len().saturating_sub(1)];
            let mut body_sorted = body.to_vec();
            body_sorted.sort_unstable();
            assert_eq!(body, &body_sorted[..], "live maps are in slot order");
            let _ = sorted;
        }
    }

    #[test]
    fn a_shift_by_a_static_count_unrolls_and_a_register_count_is_declined() {
        // LSL.W #8,D0 unrolls into eight steps rather than one shift, because
        // `ASL`'s overflow rule and the carry are per-bit.
        let l = lifted(&[0xe148, 0xc0c1]);
        assert_eq!(l.insns, 1);
        let shifts = ops(&l.block).iter().filter(|o| **o == "shl").count();
        assert!(shifts >= 8, "eight steps, one per bit: {}", l.block);
        // LSL.W D1,D0 — a run-time count — is not lifted at all.
        assert_eq!(lifted(&[0xe368]).insns, 0);
    }

    #[test]
    fn the_cache_key_separates_the_models_it_would_lift_differently() {
        // Only the model is in the key, and that is a claim rather than an
        // economy: nothing else this frontend depends on is guest state.
        assert_ne!(key(Model::M68000), key(Model::M68010));
        assert_eq!(key(Model::M68000), 0);
    }

    #[test]
    fn the_window_bound_is_a_power_of_two_and_its_mask_selects_it() {
        assert!(WINDOW.is_power_of_two());
        assert_eq!(WINDOW_MASK, WINDOW - 1);
    }

    #[test]
    fn dead_code_elimination_preserves_a_lifted_block() {
        // The flags are computed as ordinary temporaries and most of them are
        // read by nothing, which is exactly what `ir`'s decision 1 leans on
        // DCE to recover — so the pass has to accept what this emits.
        let program = vec![0x7001, 0xd041, 0x36da, 0x4252, 0x4e71];
        let l = lifted(&program);
        let pruned = crate::ir::eliminate_dead_code(&l.block);
        verify(&pruned).expect("elimination keeps a block well formed");
        assert!(
            pruned.insts().len() <= l.block.insts().len(),
            "a pass may only remove"
        );
        // Nothing with an effect goes away.
        let before = accesses(&l.block);
        let after = accesses(&pruned);
        assert_eq!(before, after, "no bus cycle may be eliminated");
        assert_eq!(static_ticks(&l.block), static_ticks(&pruned));
    }
}
