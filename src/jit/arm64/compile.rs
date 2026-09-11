//! The code generator: one [`Block`] in, one function's worth of A64 out.
//!
//! Safe Rust. It produces a `Vec<u8>` and a table of [`MemOp`] descriptors;
//! `buf` is what makes those bytes executable and
//! `rt` is what enters them. Like [`emit`](super::emit) it is
//! **not** gated to an aarch64 host, because lowering an IR block to bytes is
//! arithmetic: it runs, and is tested, anywhere. What needs the host is
//! executing the result.
//!
//! # The machine this compiles to
//!
//! The same one [`jit::x86::compile`](mod@crate::jit::x86::compile) targets, in
//! every respect that is not the instruction set — and that is deliberate,
//! because the parts that are *not* per-host are the parts that were expensive
//! to get right:
//!
//! * **Temporaries live where [`linear_scan`] puts them.** The allocator is
//!   [`ir::linear_scan`](crate::ir::linear_scan), shared, because everything
//!   it decides is a property of the block. What a backend contributes is its
//!   two register banks — `SAVED` and `VOLATILE` — and its `calls_inside`
//!   map.
//! * **Precise state at a fault is a write-through**, not a side table: a
//!   temporary an [`InsnStart`](crate::ir::InsnStart) names goes to its host
//!   register *and* to its frame slot, once, at its definition. The exception
//!   path reads the frame. `jit::x86::compile`'s "Precise state at a fault"
//!   has the measurement that chose this, and none of it is x86-specific.
//! * **The bookkeeping is deferred, not batched.** [`Opcode::CHARGE`] and
//!   [`Opcode::INSN_START`] emit no instruction at all; they are replayed by
//!   [`abi`](super::abi)'s `flush_thunk`, one call per *region*, and a region
//!   is bounded so that a static range is exactly what ran. `plan` is that
//!   rule, and it is the x86 backend's rule transcribed rather than a new one.
//!
//! Values are held **canonically masked to their type**, exactly as
//! `ir::Interp` holds them: an `i32` temporary never carries bits above 32 and
//! an `i1` never carries bits above 1. Every op assumes canonical inputs and
//! masks its own output exactly once.
//!
//! # The register assignment
//!
//! | register | holds |
//! | --- | --- |
//! | `x19` | the `Ctx` |
//! | `x20` | the temporary frame |
//! | `x21` | the thunk table |
//! | `x0`–`x3` | the arguments of a thunk call |
//! | `x9` `x10` `x11` `x12` | scratch |
//! | `x16` | the thunk address a `BLR` jumps to |
//! | `x22`–`x28` | allocated to intervals that cross a call |
//! | `x4`–`x7`, `x13`–`x15` | allocated to intervals that do not |
//!
//! **Ten registers to the allocator against x86-64's seven, and seven of them
//! callee-saved against three.** That is the one place where this host is
//! plainly better than the other, and it is the number to watch when the
//! throughput question is eventually answered: the x86 backend's own docs say
//! the ceiling on what its allocator can do is that System V has six
//! callee-saved registers of which it must spend three, while AAPCS64 has ten
//! (`x19`–`x28`) of which this spends three. A block whose live set forced a
//! spill there may not force one here.
//!
//! `x18` is **never touched**: AAPCS64 §6.1.1 makes it the platform register,
//! reserved by the platform ABI, and a code generator that used it would work
//! on Linux and corrupt a thread on a platform that gives it a meaning.
//!
//! # What compiles, and what does not
//!
//! [`compiles`] is the list. A block containing anything else is **refused**
//! and runs on the interpreter, which is `ROADMAP.md` §9's own answer for an
//! unsupported host applied to an unsupported *block*, and it is what makes a
//! partial backend a correct one. A [`Refusal`] always names what stopped it.
//!
//! Refused today, and each for a stated reason:
//!
//! * **the atomics and the exclusives** — a guest atomic has to reach the
//!   host's, and `IrHost::rmw` is that seam. On this host the question is
//!   sharper than on x86, not softer: A64's `LDXR`/`STXR` pair has a
//!   *bounded* retry requirement, and inlining one means deciding in generated
//!   code what happens when the exclusive monitor is lost. `ROADMAP.md` §9.1's
//!   sixth mechanism, and not this round's.
//! * **`popcount`** — A64's population count is `CNT`, which is a SIMD
//!   instruction (DDI 0487 C7.2, *CNT (vector)*), and the base A64 integer set
//!   has none. The SWAR sequence the x86 backend open-codes would work, but no
//!   frontend this backend can be differentially tested against emits
//!   `popcount` on a machine that also runs on this host — `cpu::x86::lift`
//!   does, and an x86 guest under an aarch64 host is a combination nothing in
//!   the tree exercises yet. Shipping code generation the harnesses cannot
//!   reach is the thing `jit::x86::compiles` refuses to do, and this follows it.
//! * **`mulu2`/`muls2`** — `UMULH`/`SMULH` make these two instructions, and
//!   they are the first thing to add once the differential runs on a real
//!   runner. They are out of this round for the same reason: nothing has
//!   executed them.
//! * **`rotlc`/`rotrc`, the divides, `addc`/`subb`, `mulhsu`, `call_helper`,
//!   `phi`** — as in the x86 backend, for its reasons.
//!
//! # The software TLB, inlined
//!
//! `ROADMAP.md` §9.1's first mechanism, and it is here: the probe below is
//! a null check, an alignment test, an index, a tag compare and an add, over
//! the *host's own* [`FastSet`], for a load and for a store. Two things about
//! it are shorter than the x86 sequence and both are the instruction set
//! rather than a cleverness: `CBZ`/`CBNZ` branch on a register without
//! touching flags, so the three "is it zero" tests cost one instruction each
//! instead of two; and `UBFX` extracts the alignment bits without a mask
//! constant.
//!
//! What the host still owes is unchanged, because it is a property of the
//! seam and not of the host code: [`FastMem::note_fast_load`] pays the tick,
//! and [`FastMem::note_fast_store`] pays the tick, the `RamStore`'s own dirty
//! bitmap and the guest-physical dirty log.
//!
//! [`FastMem::note_fast_load`]: crate::jit::FastMem::note_fast_load
//! [`FastMem::note_fast_store`]: crate::jit::FastMem::note_fast_store
//! [`FastSet`]: crate::jit::FastSet

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::value::Width;
use crate::ir::{
    Allocation, Block, CallSites, Cond, Home, Inst, Liveness, MemOp, MemSpace, Opcode, RegBanks,
    Sign, Temp, Type, bitfield_parts, linear_scan,
};
use crate::jit::PAGE_MASK;
use crate::jit::tlb::FastSet;

use super::abi::{Event, off, status, vt};
use super::emit::{Asm, Cond as Cc, Fixup, Logic, Reg, ShiftOp};

/// Why a block was not compiled.
///
/// Never an error: the IR interpreter is always the fallback
/// (`ROADMAP.md` §9, "Backends"), so a refusal costs speed on that block and
/// nothing else. Every variant names something specific, because "the JIT did
/// not take it" with no reason attached is how a backend's coverage silently
/// rots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Refusal {
    /// An opcode this backend does not lower. See [`compiles`].
    Op(Opcode),
    /// A type this backend does not hold in a host register: anything wider
    /// than 64 bits, and both float types.
    Type(Type),
    /// The block is shaped in a way the compiler will not take: a branch that
    /// is not forward, a missing terminator, an operand count that does not
    /// match the op, a bitfield outside its type, a frame slot or a branch
    /// displacement no A64 encoding reaches.
    Shape(&'static str),
    /// The code buffer had no room, even after being reset.
    CodeBufferFull,
}

impl core::fmt::Display for Refusal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::Op(op) => write!(f, "no lowering for `{op}`"),
            Refusal::Type(ty) => write!(f, "no host register holds an `{ty}`"),
            Refusal::Shape(what) => write!(f, "{what}"),
            Refusal::CodeBufferFull => f.write_str("the code buffer is full"),
        }
    }
}

/// Whether this backend lowers `op`.
///
/// Everything `cpu::riscv::lift` emits, plus most of what `cpu::arm::a64::lift`
/// does, plus the neighbours that cost nothing once their family is in. The
/// module docs say what is missing and why each one is missing.
#[must_use]
pub fn compiles(op: Opcode) -> bool {
    matches!(
        op,
        Opcode::MOV
            | Opcode::GET_SLOT
            | Opcode::EXT_S
            | Opcode::EXT_Z
            | Opcode::TRUNC
            | Opcode::BSWAP
            | Opcode::DEPOSIT
            | Opcode::EXTRACT
            | Opcode::ADD
            | Opcode::SUB
            | Opcode::MUL
            | Opcode::NEG
            | Opcode::AND
            | Opcode::OR
            | Opcode::XOR
            | Opcode::NOT
            | Opcode::ANDC
            | Opcode::SHL
            | Opcode::SHR
            | Opcode::SAR
            | Opcode::ROTL
            | Opcode::ROTR
            | Opcode::CLZ
            | Opcode::CTZ
            | Opcode::SETCOND
            | Opcode::MOVCOND
            | Opcode::BRCOND
            | Opcode::LD
            | Opcode::ST
            | Opcode::FENCE
            | Opcode::GOTO_TB
            | Opcode::EXIT_TB
            | Opcode::LOOKUP_AND_GOTO
            | Opcode::CHARGE
            | Opcode::INSN_START
    )
}

/// One block's worth of host code, and the descriptors it points at.
///
/// The [`MemOp`] table is a `Box<[MemOp]>` whose element addresses are baked
/// into the code as immediates, so it must not move once compilation has
/// finished. A `Box`'s allocation does not move when the `Box` does, which is
/// why it is a box and not a `Vec` field.
#[derive(Debug)]
pub struct Compiled {
    code: Vec<u8>,
    /// Never read from Rust after compilation — and that is the point. Its
    /// element *addresses* are immediates in the code above, so what this
    /// field does is keep the allocation alive for as long as the code that
    /// points into it.
    #[allow(dead_code)]
    mems: Box<[MemOp]>,
    /// The block's deferred bookkeeping, in instruction order.
    events: Box<[Event]>,
    /// Where the register allocator put every temporary.
    alloc: Allocation,
    offset: u64,
    /// Where this block's **chain entry** sits, relative to its own first
    /// byte: immediately past the prologue.
    ///
    /// A block entered by a call runs the prologue and gets a host frame; a
    /// block entered by a *link* inherits the frame its predecessor is
    /// standing in, so it must start here instead. See
    /// [`Compiler::epilogue`]'s chain pad.
    chain: u64,
}

impl Compiled {
    /// The machine code.
    #[inline]
    #[must_use]
    pub fn code(&self) -> &[u8] {
        &self.code
    }

    /// Where it was placed in the code buffer.
    #[inline]
    #[must_use]
    pub fn offset(&self) -> u64 {
        self.offset
    }

    /// Where its chain entry was placed in the code buffer.
    ///
    /// The address a linked predecessor jumps to. Absolute within the buffer,
    /// like [`Compiled::offset`], because that is the form the block cache
    /// carries and the chain thunk adds a base to.
    #[inline]
    #[must_use]
    pub fn chain_entry(&self) -> u64 {
        self.offset + self.chain
    }

    /// The bookkeeping a flush replays, which the runtime hands generated code
    /// a range into.
    #[inline]
    #[must_use]
    pub fn events(&self) -> &[Event] {
        &self.events
    }

    /// The same block, recorded as living at `offset`.
    #[must_use]
    pub fn at(mut self, offset: u64) -> Compiled {
        self.offset = offset;
        // The code is copied into the buffer, and nothing reads it again.
        self.code = Vec::new();
        self
    }

    /// Where the allocator put `temp`.
    #[inline]
    #[must_use]
    pub fn home(&self, temp: Temp) -> Home {
        self.alloc.home(temp)
    }

    /// Whether this code writes `temp`'s value into the frame, which is the
    /// only place Rust can read it from after the run.
    #[inline]
    #[must_use]
    pub fn frame_backed(&self, temp: Temp) -> bool {
        self.alloc.frame_backed(temp)
    }

    /// How many temporaries the allocator kept in host registers.
    #[must_use]
    pub fn in_registers(&self) -> usize {
        self.alloc.in_registers()
    }

    /// How many descriptors it carries, for tests.
    #[cfg(test)]
    #[must_use]
    pub fn mem_count(&self) -> usize {
        self.mems.len()
    }
}

/// Where a compiled block keeps its temporaries.
///
/// [`Regs::Frame`] is the **control** a differential runs against — the same
/// block, the same host, the same everything, with the allocator switched
/// off — which is what turns "the two engines agree" into "the allocator did
/// not change the answer".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Regs {
    /// Every temporary lives in the frame: the backend without the allocator.
    Frame,
    /// Linear scan over the block's live intervals — [`linear_scan`].
    #[default]
    Scan,
}

/// Compile `block`, or say why not.
///
/// # Errors
///
/// [`Refusal`], naming the op, type or shape that stopped it.
pub fn compile(block: &Block) -> Result<Compiled, Refusal> {
    compile_with(block, Regs::default())
}

/// The same, with the register allocator switched.
///
/// # Errors
///
/// [`Refusal`], naming the op, type or shape that stopped it.
pub fn compile_with(block: &Block, regs: Regs) -> Result<Compiled, Refusal> {
    Compiler::new(block, regs)?.run()
}

/// The context pointer, for the body of a block.
const CTX: Reg = Reg::X19;
/// The temporary frame's base.
const FRAME: Reg = Reg::X20;
/// The thunk table's base.
const VTAB: Reg = Reg::X21;
/// The accumulator — this backend's `rax`.
const A: Reg = Reg::X9;
/// The second scratch.
const B: Reg = Reg::X10;
/// The third scratch.
const C: Reg = Reg::X11;
/// The fourth scratch, and the one the TLB probe leaves a host address in.
const D: Reg = Reg::X12;
/// Where a thunk's address is loaded before the `BLR`.
///
/// `x16` is IP0, which AAPCS64 §6.1.1 sets aside for exactly this: a scratch a
/// call sequence may use, corruptible across the call, and never live over
/// one.
const IP: Reg = Reg::X16;

/// The host registers a temporary may live in that survive a call.
///
/// `x19`–`x28` are callee-saved (AAPCS64 §6.1.1); three of them are the
/// context, the frame and the thunk table, and the remaining seven are these.
pub(super) const SAVED: [u8; 7] = [22, 23, 24, 25, 26, 27, 28];

/// The registers a thunk call may destroy, which the allocator gives only to
/// intervals that do not span one.
///
/// `x0`–`x7` are argument and result registers and `x9`–`x15` are corruptible;
/// this backend spends `x0`–`x3` on call arguments and `x9`–`x12` on fixed
/// scratch, which leaves these ten minus the three it does not need. `x8` (the
/// indirect result register) and `x17` (IP1) are left alone: neither is needed
/// and both are easier to reason about unused.
pub(super) const VOLATILE: [u8; 7] = [4, 5, 6, 7, 13, 14, 15];

/// A register number back into a [`Reg`].
///
/// Only the numbers [`SAVED`] and [`VOLATILE`] contain can reach here, and an
/// allocation naming anything else is a bug in this file rather than in a
/// guest — so the fallback is the accumulator, which every lowering already
/// treats as destroyed, rather than a panic in generated-code emission.
const fn reg_of(n: u8) -> Reg {
    match n {
        4 | 5 | 6 | 7 | 13 | 14 | 15 | 22 | 23 | 24 | 25 | 26 | 27 | 28 => Reg(n),
        _ => A,
    }
}

/// Whether this backend's lowering of `op` calls into the host *after* reading
/// its operands and *before* writing its results.
///
/// One of the two inputs [`linear_scan`] cannot check for itself. `charge` and
/// `insn_start` are not here, because neither emits an instruction: their call
/// is the region's flush, which is [`CallSites::before`]. [`Opcode::FENCE`] is
/// not here either, because it emits an instruction and not a call — it still
/// starts a region, so the allocator hears about it through the same array.
const fn calls_inside(op: Opcode) -> bool {
    matches!(op, Opcode::LD | Opcode::ST | Opcode::GET_SLOT)
}

/// The deferred bookkeeping of one block, and where each region of it is
/// replayed.
#[derive(Debug)]
struct Plan {
    /// Every [`Opcode::CHARGE`] and [`Opcode::INSN_START`] the block contains,
    /// in instruction order.
    events: Vec<Event>,
    /// `at[i]` is the half-open range of [`Plan::events`] a flush emitted in
    /// the gap ahead of instruction `i` replays, and `None` when no flush is
    /// emitted there.
    at: Vec<Option<(u32, u32)>>,
}

impl Plan {
    /// Whether replaying `lo .. hi` can end the block.
    ///
    /// It can only where the range holds a boundary a run may *leave* at, and
    /// `flush_thunk` leaves at no other kind of event and at no exit boundary.
    /// A flush over a range of nothing but charges is therefore known to
    /// answer zero, and the `CBNZ` after it would be dead code.
    fn can_stop(&self, lo: u32, hi: u32) -> bool {
        self.events[lo as usize..hi as usize]
            .iter()
            .any(|e| matches!(e, Event::Boundary { exit: false, .. }))
    }
}

/// Collect a block's bookkeeping and decide where to replay it.
///
/// The rule, which is `jit::x86::compile`'s rule and is written out in full
/// there: an instruction is a **region boundary** when it can let the host
/// observe something ([`calls_inside`] or [`Opcode::FENCE`]), when it is a
/// terminator, when it is a `brcond`, or when a `brcond` targets it. At a
/// boundary the pending range is flushed if it holds anything, and a new
/// region starts at that instruction whether or not anything was flushed. So
/// no branch and no branch target lies strictly inside a region, every path
/// that reaches a flush entered its region at the top, and the events in
/// `[region, here)` are exactly the ones that ran.
///
/// # Errors
///
/// [`Refusal`] for a charge with no tick count or a boundary marker that
/// points at no record — the interpreter reports such a block as an error, and
/// the two engines have to agree about that too.
fn plan(block: &Block) -> Result<Plan, Refusal> {
    let insts = block.insts();
    let n = insts.len();
    let mut target = vec![false; n];
    for inst in insts {
        if inst.op == Opcode::BRCOND
            && let Some(slot) = target.get_mut(inst.aux as usize)
        {
            *slot = true;
        }
    }
    let mut events: Vec<Event> = Vec::new();
    let mut at = vec![None; n];
    let mut region = 0u32;
    for (i, inst) in insts.iter().enumerate() {
        let op = inst.op;
        if target[i]
            || calls_inside(op)
            || op == Opcode::FENCE
            || op == Opcode::BRCOND
            || op.is_terminator()
        {
            let here = events.len() as u32;
            if here > region {
                at[i] = Some((region, here));
            }
            region = here;
        }
        match op {
            Opcode::CHARGE => {
                let ticks = inst
                    .imm
                    .ok_or(Refusal::Shape("a charge needs a tick count"))?
                    .bits() as u64;
                // Fuse into the boundary just ahead of it where the three
                // conditions `jit::x86::compile::plan` writes out hold: a
                // non-zero count, no flush point in between
                // (`events.len() > region`, which is what a `brcond` targeting
                // the charge breaks), and a boundary whose slot is still free.
                let open = events.len() > region as usize;
                match events.last_mut() {
                    Some(Event::Boundary { ticks: slot, .. })
                        if open && ticks != 0 && *slot == 0 =>
                    {
                        *slot = ticks;
                    }
                    _ => events.push(Event::Charge(ticks)),
                }
            }
            Opcode::INSN_START => {
                block
                    .marks()
                    .get(inst.aux as usize)
                    .ok_or(Refusal::Shape("the boundary marker points at no record"))?;
                events.push(Event::Boundary {
                    mark: inst.aux,
                    // Read here because the replay cannot: it is handed a
                    // range of events and never sees an instruction index.
                    exit: insts.get(i + 1).is_some_and(|next| next.op.is_terminator()),
                    // Filled in by the charge that follows, if one does.
                    ticks: 0,
                });
            }
            _ => {}
        }
    }
    Ok(Plan { events, at })
}

/// The `mem` table, collected before anything is emitted so its addresses are
/// final.
fn descriptors(block: &Block) -> Box<[MemOp]> {
    let mut out = Vec::new();
    for inst in block.insts() {
        if matches!(inst.op, Opcode::LD | Opcode::ST)
            && let Some(mem) = inst.mem
        {
            out.push(mem);
        }
    }
    out.into_boxed_slice()
}

struct Compiler<'a> {
    block: &'a Block,
    asm: Asm,
    mems: Box<[MemOp]>,
    /// Where every temporary lives.
    alloc: Allocation,
    /// The next descriptor to hand out, in the order [`descriptors`] collected.
    next_mem: usize,
    /// Where each IR instruction's code begins — **after** any flush emitted
    /// ahead of it, because that is where a branch has to land.
    starts: Vec<usize>,
    /// The deferred bookkeeping and where it is replayed, from [`plan`].
    plan: Plan,
    /// Branches whose target is an IR instruction index.
    branches: Vec<(Fixup, usize)>,
    /// Jumps to the epilogue.
    exits: Vec<Fixup>,
    /// Jumps to the pad that leaves with [`status::SPENT`], one per flush.
    spent: Vec<Fixup>,
    /// Where the chain entry ended up — set once, by [`Compiler::run`].
    chain: u64,
    /// Set when an offset had no encoding. Checked once, at the end of
    /// [`Compiler::run`]: an address this backend cannot form is a refusal,
    /// and the interpreter runs the block.
    unreachable_offset: bool,
}

/// The stack frame this backend reserves.
///
/// 128 bytes: 32 of scratch and 96 for the twelve registers the prologue
/// saves. Sixteen-byte aligned because AAPCS64 §6.2.  requires `SP` to be
/// 16-byte aligned at every instruction that uses it as a base register, and a
/// misaligned `SP` on this architecture is a fault rather than a slowdown.
const FRAME_SIZE: u64 = 128;

/// Where a load thunk leaves its value: the first scratch word.
const OUT: u64 = 0;

/// Where the inlined access parks a value across the call that follows it.
///
/// The load parks the loaded word across `note_fast_load` and the store parks
/// the guest address across `note_fast_store`. Both could live in a
/// callee-saved register instead — this host has seven spare — but that is
/// seven registers the *allocator* would not get, and the x86 backend's
/// measurement of the same trade says the allocator wants them more than the
/// fast path wants two stack accesses.
const PARKED: u64 = 8;

/// The first byte of the saved-register area.
const SAVE_AT: u64 = 32;

impl<'a> Compiler<'a> {
    fn new(block: &'a Block, regs: Regs) -> Result<Compiler<'a>, Refusal> {
        // A temporary's frame slot is reached with A64's unsigned scaled
        // 12-bit immediate, so the last one has to be inside 4095 words.
        if block.temp_count() > 4095 {
            return Err(Refusal::Shape("the block has too many temporaries"));
        }
        // Every instruction index reaches generated code as an immediate — a
        // fault's `at`, a flush's range.
        if u32::try_from(block.insts().len()).is_err() {
            return Err(Refusal::Shape("the block is too long"));
        }
        // Straight-line SSA, checked here and not only in `ir::verify`,
        // because `rt::Engine::run` keeps one temporary frame across every
        // block it runs and does not clear it: a frame slot read before this
        // block wrote it would be an earlier block's value, where the
        // interpreter would answer zero.
        let mut defined = vec![false; block.temp_count()];
        for i in 0..block.insts().len() {
            if block
                .srcs(i)
                .iter()
                .any(|t| !defined.get(t.index()).copied().unwrap_or(false))
            {
                return Err(Refusal::Shape("a temporary is read before it is assigned"));
            }
            let inst = &block.insts()[i];
            for dst in [inst.dst, inst.dst2].into_iter().flatten() {
                if let Some(slot) = defined.get_mut(dst.index()) {
                    *slot = true;
                }
            }
        }
        let plan = plan(block)?;
        let alloc = match regs {
            Regs::Frame => Allocation::none(block),
            Regs::Scan => {
                let live = Liveness::compute(block);
                let inside: Vec<bool> = block.insts().iter().map(|i| calls_inside(i.op)).collect();
                let before: Vec<bool> = plan.at.iter().map(Option::is_some).collect();
                linear_scan(
                    block,
                    &live,
                    &RegBanks {
                        saved: &SAVED,
                        volatile: &VOLATILE,
                    },
                    &CallSites {
                        inside: &inside,
                        before: &before,
                    },
                )
            }
        };
        Ok(Compiler {
            block,
            asm: Asm::new(),
            mems: descriptors(block),
            alloc,
            next_mem: 0,
            starts: Vec::with_capacity(block.insts().len()),
            plan,
            branches: Vec::new(),
            exits: Vec::new(),
            spent: Vec::new(),
            chain: 0,
            unreachable_offset: false,
        })
    }

    fn run(mut self) -> Result<Compiled, Refusal> {
        let insts = self.block.insts();
        if !insts.last().is_some_and(|i| i.op.is_terminator()) {
            return Err(Refusal::Shape("the block does not end in a terminator"));
        }
        self.prologue();
        // Past the prologue, which a block reached by a link must not run
        // again: it is standing in the frame its predecessor built, and a
        // second set of saves would grow the host stack once per guest branch.
        self.chain = self.asm.here() as u64;
        for (at, inst) in insts.iter().enumerate() {
            // Before the position a branch lands on, so the taken path skips
            // the range it did not execute.
            if let Some((lo, hi)) = self.plan.at[at] {
                self.flush(lo, hi);
            }
            self.starts.push(self.asm.here());
            self.inst(at, inst)?;
        }
        self.epilogue();
        for (fixup, target) in core::mem::take(&mut self.branches) {
            let at = self.starts[target];
            self.asm.bind_to(fixup, at);
        }
        if self.unreachable_offset {
            return Err(Refusal::Shape(
                "an operand is past what one A64 offset reaches",
            ));
        }
        if self.asm.overflowed() {
            // A64 branches are 14, 19 or 26 bits of *instructions*, so this
            // needs a block whose lowering is tens of thousands of
            // instructions long. Refused rather than truncated: a branch that
            // wrapped its field would land in the middle of another block.
            return Err(Refusal::Shape("a branch does not reach inside this block"));
        }
        Ok(Compiled {
            code: self.asm.finish(),
            mems: self.mems,
            events: self.plan.events.into_boxed_slice(),
            alloc: self.alloc,
            offset: 0,
            chain: self.chain,
        })
    }

    // ---- frame ---------------------------------------------------------

    fn prologue(&mut self) {
        self.asm.sub_imm(64, Reg::SP, Reg::SP, FRAME_SIZE as u32);
        // `x19`–`x28`, then the frame pointer and the link register. `X30` is
        // the one that is not optional: every `BLR` below overwrites it.
        self.asm.stp(Reg::X19, Reg::X20, Reg::SP, SAVE_AT);
        self.asm.stp(Reg::X21, Reg(22), Reg::SP, SAVE_AT + 16);
        self.asm.stp(Reg(23), Reg(24), Reg::SP, SAVE_AT + 32);
        self.asm.stp(Reg(25), Reg(26), Reg::SP, SAVE_AT + 48);
        self.asm.stp(Reg(27), Reg(28), Reg::SP, SAVE_AT + 64);
        self.asm.stp(Reg::X29, Reg::X30, Reg::SP, SAVE_AT + 80);
        self.asm.mov(64, CTX, Reg::X0);
        self.load_ctx(FRAME, off::TEMPS);
        self.load_ctx(VTAB, off::VT);
    }

    fn epilogue(&mut self) {
        // The pad every flush's "the allowance is spent" edge lands on. One
        // per block rather than one per flush, and it falls straight through
        // into the epilogue below. Emitted only where something jumps here.
        if !self.spent.is_empty() {
            for f in core::mem::take(&mut self.spent) {
                self.asm.bind(f);
            }
            self.asm.mov_imm(Reg::X0, status::SPENT);
        }
        for f in core::mem::take(&mut self.exits) {
            self.asm.bind(f);
        }
        self.chain_pad();
        self.asm.ldp(Reg::X19, Reg::X20, Reg::SP, SAVE_AT);
        self.asm.ldp(Reg::X21, Reg(22), Reg::SP, SAVE_AT + 16);
        self.asm.ldp(Reg(23), Reg(24), Reg::SP, SAVE_AT + 32);
        self.asm.ldp(Reg(25), Reg(26), Reg::SP, SAVE_AT + 48);
        self.asm.ldp(Reg(27), Reg(28), Reg::SP, SAVE_AT + 64);
        self.asm.ldp(Reg::X29, Reg::X30, Reg::SP, SAVE_AT + 80);
        self.asm.add_imm(64, Reg::SP, Reg::SP, FRAME_SIZE as u32);
        self.asm.ret();
    }

    /// The direct link: ask the dispatcher for the successor's code and
    /// branch to it, or fall through into the frame teardown below.
    ///
    /// Every way out of a block arrives here with its [`status`] in `x0` — the
    /// three terminators, the spent pad above, and the fault sequence — and
    /// the reason it is *every* way out rather than the two that can chain is
    /// that the thunk is what closes the block off: it publishes the exit
    /// boundary, drains the guest's stores against the block cache and counts
    /// what retired.
    ///
    /// `BLR` overwrites `X30`, which the prologue has already saved and the
    /// epilogue reloads; the link itself is `BR`, which leaves it alone,
    /// because the successor's own epilogue is what eventually returns.
    ///
    /// [`Ctx::chain`](super::abi::Ctx::chain) being zero is a caller that has
    /// not enabled linking, and then the whole of it is a store, a load, a
    /// taken `CBZ` and a load: four instructions once per block exit.
    fn chain_pad(&mut self) {
        self.store_ctx(Reg::X0, off::OUT_STATUS);
        self.load_ctx(IP, off::CHAIN);
        let off_ = self.asm.cbz(64, IP, true);
        self.ctx_to_arg();
        self.asm.blr(IP);
        let end = self.asm.cbz(64, Reg::X0, true);
        // The successor's chain entry. `x19`, `x20`, `x21` and `sp` are
        // already what its body expects — the same context, the same temporary
        // frame, the same thunk table and the same frame depth — which is the
        // whole of why a link is a branch and not a call. The thunk guarantees
        // the frame is long enough for the block it names; it cannot grow one,
        // because growing it would move it under the code now running.
        self.asm.br(Reg::X0);
        self.asm.bind(off_);
        self.asm.bind(end);
        self.load_ctx(Reg::X0, off::OUT_STATUS);
    }

    /// Leave the block with `code` in `x0`.
    fn leave(&mut self, code: u64) {
        self.asm.mov_imm(Reg::X0, code);
        let f = self.asm.b();
        self.exits.push(f);
    }

    // ---- context and frame accesses -------------------------------------

    /// `LDR dst, [x19, #field]`.
    fn load_ctx(&mut self, dst: Reg, field: u64) {
        if !self.asm.ldr(dst, CTX, field) {
            self.unreachable_offset = true;
        }
    }

    /// `STR src, [x19, #field]`.
    fn store_ctx(&mut self, src: Reg, field: u64) {
        if !self.asm.str(src, CTX, field) {
            self.unreachable_offset = true;
        }
    }

    /// Write a constant into a context field, through the accumulator.
    fn store_ctx_imm(&mut self, field: u64, value: u64) {
        self.asm.mov_imm(A, value);
        self.store_ctx(A, field);
    }

    /// Add one to a `u64` context field.
    ///
    /// Three instructions where x86 has one: A64 has no read-modify-write
    /// against memory outside the atomics, and an atomic here would be both
    /// wrong (this counter is per-execution and single-threaded) and slower.
    fn bump(&mut self, field: u64) {
        self.load_ctx(A, field);
        self.asm.add_imm(64, A, A, 1);
        self.store_ctx(A, field);
    }

    // ---- operands ------------------------------------------------------

    fn frame_slot(temp: Temp) -> u64 {
        (temp.index() as u64) * 8
    }

    /// The host register `temp` lives in, if it lives in one.
    fn home(&self, temp: Temp) -> Option<Reg> {
        match self.alloc.home(temp) {
            Home::Reg(n) => Some(reg_of(n)),
            Home::Frame => None,
        }
    }

    /// Get `temp` into `reg`, which the caller may then destroy.
    fn load_temp(&mut self, reg: Reg, temp: Temp) {
        match self.home(temp) {
            Some(home) => {
                if home != reg {
                    self.asm.mov(64, reg, home);
                }
            }
            None => {
                let at = Self::frame_slot(temp);
                if !self.asm.ldr(reg, FRAME, at) {
                    self.unreachable_offset = true;
                }
            }
        }
    }

    /// A **read-only** operand: the register holding `temp`, without copying
    /// it when it already lives in one.
    ///
    /// The caller promises not to write the register this returns. Every use
    /// of it below is an instruction that only reads that operand, and getting
    /// it wrong corrupts a temporary that is still live.
    fn operand(&mut self, scratch: Reg, temp: Temp) -> Reg {
        match self.home(temp) {
            Some(home) => home,
            None => {
                self.load_temp(scratch, temp);
                scratch
            }
        }
    }

    /// The register an op should compute into: the destination's own home
    /// where there is one, and the accumulator otherwise.
    ///
    /// `blocked` is the operands still read **after** the accumulator is first
    /// written. An operand read only *into* the accumulator is deliberately
    /// not blocked: a register is handed to the destination only out of the
    /// free set, so `home(dst) == home(a)` is the statement that `a` is dead
    /// after this instruction.
    fn acc(&self, inst: &Inst, blocked: &[Temp]) -> Reg {
        let Some(dst) = inst.dst else { return A };
        let Some(home) = self.home(dst) else {
            return A;
        };
        if blocked.iter().any(|t| self.home(*t) == Some(home)) {
            A
        } else {
            home
        }
    }

    fn store_temp(&mut self, temp: Temp, reg: Reg) {
        let at = Self::frame_slot(temp);
        if !self.asm.str(reg, FRAME, at) {
            self.unreachable_offset = true;
        }
    }

    /// Put `reg`'s value where `temp` lives — and, when the frame is also its
    /// home, in both places.
    ///
    /// The write-through is `ROADMAP.md` §9's precise-exception requirement in
    /// this backend, exactly as in the x86 one: a temporary an `InsnStart`
    /// names is read by the exception path, out of the frame, from outside the
    /// generated code.
    fn commit(&mut self, temp: Temp, reg: Reg) {
        if let Some(home) = self.home(temp) {
            if home != reg {
                self.asm.mov(64, home, reg);
            }
            if self.alloc.frame_backed(temp) {
                self.store_temp(temp, reg);
            }
        } else {
            self.store_temp(temp, reg);
        }
    }

    /// Canonicalise `reg` to `ty`, as `Interp::set` does.
    ///
    /// The 32-bit case is free of a mask constant: every A64 instruction with
    /// a `W` destination clears the upper half of the register (DDI 0487
    /// §C1.2.5), so a 32-bit `MOV` *is* the truncation.
    fn mask(&mut self, reg: Reg, ty: Type) {
        match ty {
            Type::I1 => self.asm.ubfx(64, reg, reg, 0, 1),
            Type::I32 => self.asm.mov(32, reg, reg),
            Type::I64 => {}
            // Refused at `check_type`, so this is unreachable rather than
            // wrong; masking nothing is still the conservative answer.
            _ => {}
        }
    }

    /// Sign-extend the low `bits` of `reg` through the whole register.
    ///
    /// One `SBFX` — the `SBFM` alias — where x86 needs a left shift and an
    /// arithmetic right shift.
    fn sext(&mut self, reg: Reg, bits: u32) {
        if bits < 64 {
            self.asm.sbfx(64, reg, reg, 0, bits);
        }
    }

    /// Write `reg` into the instruction's destination.
    ///
    /// `ir::Interp` applies **two** masks here and this reproduces both: the
    /// arithmetic ops mask their result to the *instruction's* width, and every
    /// write then masks to the *destination temporary's*.
    ///
    /// A destination **wider** than the instruction is refused, because there
    /// the two orders differ. Nothing in the tree emits one.
    ///
    /// `reg` is masked **in place**, so it must be a scratch register and
    /// never one [`Compiler::operand`] handed back.
    fn write(&mut self, inst: &Inst, reg: Reg) -> Result<(), Refusal> {
        let dst = inst
            .dst
            .ok_or(Refusal::Shape("this op must have a destination"))?;
        let dst_ty = self
            .block
            .type_of(dst)
            .ok_or(Refusal::Shape("the destination was never allocated"))?;
        check_type(dst_ty)?;
        if dst_ty.bits() > inst.ty.bits() {
            return Err(Refusal::Shape(
                "this op's destination is wider than the op's own type",
            ));
        }
        self.mask(reg, inst.ty);
        if dst_ty != inst.ty {
            self.mask(reg, dst_ty);
        }
        self.commit(dst, reg);
        Ok(())
    }

    fn src(&self, at: usize, i: usize) -> Result<Temp, Refusal> {
        self.block
            .srcs(at)
            .get(i)
            .copied()
            .ok_or(Refusal::Shape("too few source operands"))
    }

    /// The same, refusing an operand whose type is not the instruction's.
    ///
    /// The IR does not require the two to agree and `ir::Interp` then computes
    /// on the *operand's* width. For the ops where that changes the answer —
    /// the rotates, the bit counts, `bswap` — reproducing it would mean
    /// carrying the interpreter's accident into generated code, and doing
    /// anything else would be a silent divergence. So it is refused.
    fn src_typed(&self, at: usize, i: usize, ty: Type) -> Result<Temp, Refusal> {
        let temp = self.src(at, i)?;
        if self.block.type_of(temp) == Some(ty) {
            Ok(temp)
        } else {
            Err(Refusal::Shape(
                "this op's operand is not the op's own type, and the interpreter \
                 would compute at the operand's width",
            ))
        }
    }

    // ---- calls ---------------------------------------------------------

    /// `LDR x16, [x21, #slot]` then `BLR x16`, with the context already in
    /// `x0`.
    ///
    /// Two instructions rather than x86's one `call [r14 + disp]`, because A64
    /// has no call through memory: a `BLR` takes a register.
    fn call(&mut self, slot: u64) {
        self.load_vt(slot);
        self.asm.blr(IP);
    }

    fn load_vt(&mut self, slot: u64) {
        if !self.asm.ldr(IP, VTAB, slot) {
            self.unreachable_offset = true;
        }
    }

    fn ctx_to_arg(&mut self) {
        self.asm.mov(64, Reg::X0, CTX);
    }

    /// Replay events `lo .. hi`: the region's charges and boundaries.
    fn flush(&mut self, lo: u32, hi: u32) {
        self.ctx_to_arg();
        self.asm.mov_imm(Reg::X1, u64::from(lo));
        self.asm.mov_imm(Reg::X2, u64::from(hi));
        self.call(vt::FLUSH);
        // A non-zero answer means the replay stopped at a boundary because
        // `IrHost::spent` said the tick allowance was gone, so the block
        // leaves at that boundary rather than running on to its terminator.
        // **One** instruction here against x86's two, because `CBNZ` needs no
        // flag-setting compare ahead of it.
        //
        // Emitted only where the answer can be non-zero: `flush_thunk` stops
        // at a boundary and nowhere else, and never at an exit boundary.
        if self.plan.can_stop(lo, hi) {
            let f = self.asm.cbz(64, Reg::X0, false);
            self.spent.push(f);
        }
    }

    /// The sequence a faulting access jumps to: record where and why, and
    /// leave. `x0` holds the error code on entry.
    ///
    /// No flush: this is only ever emitted inside a load or a store, which is
    /// a region boundary, so everything up to the faulting access has already
    /// been replayed — which is what makes the fault's reported tick count the
    /// interpreter's.
    fn fault(&mut self, at: usize) -> Result<(), Refusal> {
        let at = u64::try_from(at).map_err(|_| Refusal::Shape("the block is too long"))?;
        self.store_ctx(Reg::X0, off::FAULT_ERROR);
        self.store_ctx_imm(off::FAULT_AT, at);
        self.leave(status::FAULT);
        Ok(())
    }

    // ---- the opcodes ---------------------------------------------------

    fn inst(&mut self, at: usize, inst: &Inst) -> Result<(), Refusal> {
        let op = inst.op;
        if !compiles(op) {
            return Err(Refusal::Op(op));
        }
        check_type(inst.ty)?;
        let w = inst.ty.bits();

        match op {
            Opcode::MOV => {
                let acc = self.acc(inst, &[]);
                match (self.block.srcs(at).first().copied(), inst.imm) {
                    (Some(s), _) => self.load_temp(acc, s),
                    (None, Some(c)) => self.asm.mov_imm(acc, c.bits() as u64),
                    (None, None) => {
                        return Err(Refusal::Shape("a mov needs a source or an immediate"));
                    }
                }
                self.write(inst, acc)?;
            }
            Opcode::GET_SLOT => {
                self.ctx_to_arg();
                self.asm.mov_imm(Reg::X1, u64::from(inst.aux & 0xffff));
                self.call(vt::GET_SLOT);
                self.write(inst, Reg::X0)?;
            }
            Opcode::EXT_S => {
                let s = self.src(at, 0)?;
                let from = self
                    .block
                    .type_of(s)
                    .ok_or(Refusal::Shape("the source was never allocated"))?;
                check_type(from)?;
                let acc = self.acc(inst, &[]);
                self.load_temp(acc, s);
                self.sext(acc, from.bits());
                self.write(inst, acc)?;
            }
            Opcode::EXT_Z | Opcode::TRUNC => {
                let s = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                self.load_temp(acc, s);
                self.write(inst, acc)?;
            }
            Opcode::BSWAP => {
                // One instruction, whatever the lane, because A64 has exactly
                // the three reversals the IR's lane widths name: `REV16`,
                // `REV32` and `REV` (DDI 0487 C6.2). The x86 backend open-codes
                // a swap cascade for a lane narrower than the type; this host
                // needs none, and that is the clearest single example of what
                // "the same IR, a different backend" buys.
                if w != 32 && w != 64 {
                    return Err(Refusal::Shape("a byte reversal is 32 or 64 bits wide"));
                }
                let lane = match inst.imm {
                    Some(c) => u32::try_from(c.bits())
                        .map_err(|_| Refusal::Shape("the lane width is absurd"))?,
                    None => w,
                };
                if !matches!(lane, 8 | 16 | 32 | 64) || lane > w || !w.is_multiple_of(lane) {
                    return Err(Refusal::Shape(
                        "a bswap lane is 8, 16, 32 or 64 bits and divides the type",
                    ));
                }
                let s = self.src_typed(at, 0, inst.ty)?;
                self.load_temp(A, s);
                if lane > 8 {
                    // A lane of eight bits reverses nothing.
                    self.asm.rev(w, lane, A, A);
                }
                self.write(inst, A)?;
            }
            Opcode::DEPOSIT => {
                let (pos, len) = bitfield_parts(inst.aux);
                if field_mask(pos, len, w).is_none() {
                    return Err(Refusal::Shape("the bitfield leaves the type"));
                }
                let into = self.src(at, 0)?;
                let what = self.src(at, 1)?;
                self.load_temp(A, into);
                self.load_temp(B, what);
                // `BFI` — one instruction for the IR's whole `deposit`: it
                // takes the low `len` bits of the source, places them at
                // `pos`, and leaves every other bit of the destination alone
                // (DDI 0487 C6.2, *BFI*). The x86 lowering is six.
                self.asm.bfi(64, A, B, pos, len);
                self.write(inst, A)?;
            }
            Opcode::EXTRACT => {
                let (pos, len) = bitfield_parts(inst.aux);
                if field_mask(pos, len, w).is_none() {
                    return Err(Refusal::Shape("the bitfield leaves the type"));
                }
                let s = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                let from = self.operand(A, s);
                self.asm.ubfx(64, acc, from, pos, len);
                self.write(inst, acc)?;
            }
            Opcode::ADD | Opcode::SUB | Opcode::AND | Opcode::OR | Opcode::XOR | Opcode::ANDC => {
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let acc = self.acc(inst, &[b]);
                let lhs = self.operand(A, a);
                let rhs = self.operand(B, b);
                // Always at 64 bits, with `write` masking to the type
                // afterwards. A 32-bit `add` of two values held zero-extended
                // agrees with the 64-bit one in its low half, and the mask is
                // owed either way — so the width of the arithmetic is a free
                // choice and the wider one avoids a second rule to check.
                match op {
                    Opcode::ADD => self.asm.add(64, acc, lhs, rhs),
                    Opcode::SUB => self.asm.sub(64, acc, lhs, rhs),
                    Opcode::AND => self.asm.logic(64, Logic::And, acc, lhs, rhs),
                    Opcode::OR => self.asm.logic(64, Logic::Orr, acc, lhs, rhs),
                    Opcode::XOR => self.asm.logic(64, Logic::Eor, acc, lhs, rhs),
                    // `BIC` is `a AND NOT b` in one instruction, which is
                    // exactly the IR's `andc`; x86 needs a `not` and an `and`
                    // and a third register to hold the inverse.
                    _ => self.asm.logic(64, Logic::Bic, acc, lhs, rhs),
                }
                self.write(inst, acc)?;
            }
            Opcode::MUL => {
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let acc = self.acc(inst, &[b]);
                let lhs = self.operand(A, a);
                let rhs = self.operand(B, b);
                self.asm.mul(64, acc, lhs, rhs);
                self.write(inst, acc)?;
            }
            Opcode::NEG => {
                let a = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                let s = self.operand(A, a);
                self.asm.neg(64, acc, s);
                self.write(inst, acc)?;
            }
            Opcode::NOT => {
                let a = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                let s = self.operand(A, a);
                self.asm.not(64, acc, s);
                self.write(inst, acc)?;
            }
            Opcode::SHL | Opcode::SHR | Opcode::SAR => self.shift(at, inst, w)?,
            Opcode::ROTL | Opcode::ROTR => {
                if w != 32 && w != 64 {
                    return Err(Refusal::Shape("a rotate is 32 or 64 bits wide"));
                }
                let a = self.src_typed(at, 0, inst.ty)?;
                let b = self.src(at, 1)?;
                self.load_temp(A, a);
                self.load_temp(B, b);
                if op == Opcode::ROTL {
                    // A64 rotates right and not left, so a left rotate by `n`
                    // is a right rotate by `-n`: `RORV` takes its amount
                    // modulo the datasize (DDI 0487 C6.2, *RORV*), and
                    // `(-n) MOD w` is `(w - n MOD w) MOD w`, which is the
                    // rotate the IR asks for including at `n == 0`.
                    self.asm.neg(64, B, B);
                }
                self.asm.shift(w, ShiftOp::Ror, A, A, B);
                self.write(inst, A)?;
            }
            Opcode::CLZ | Opcode::CTZ => {
                if w != 32 && w != 64 {
                    return Err(Refusal::Shape("a bit count is 32 or 64 bits wide"));
                }
                let a = self.src_typed(at, 0, inst.ty)?;
                let acc = self.acc(inst, &[]);
                let s = self.operand(A, a);
                if op == Opcode::CTZ {
                    // Trailing zeros are leading zeros of the reversed word
                    // (DDI 0487 C6.2, *RBIT*), and `CLZ` answers the width for
                    // a zero input — which is exactly what the IR asks for in
                    // both directions.
                    self.asm.rbit(w, acc, s);
                    self.asm.clz(w, acc, acc);
                } else {
                    self.asm.clz(w, acc, s);
                }
                self.write(inst, acc)?;
            }
            Opcode::SETCOND => {
                let cond = inst
                    .cond
                    .ok_or(Refusal::Shape("a comparison needs a condition"))?;
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let cc = self.compare(cond, w, a, b);
                self.asm.cset(64, A, cc);
                self.write(inst, A)?;
            }
            Opcode::MOVCOND => self.movcond(at, inst, w)?,
            Opcode::BRCOND => self.brcond(at, inst, w)?,
            Opcode::LD => self.load(at, inst)?,
            Opcode::ST => self.store(at, inst)?,
            // A guest barrier, inline: one instruction and no call.
            //
            // **`DMB ISH`, not nothing and not `DMB ISHST`.** `IrHost::fence`'s
            // contract is a `SeqCst` host fence, chosen there because it is the
            // only ordering that also forbids **store-then-load** — and on this
            // architecture, unlike x86-64, that is not the only reordering at
            // stake: A64 is weakly ordered, so store-store and load-load are
            // reorderable too and the barrier is doing work that `MFENCE` was
            // partly getting for free from x86-TSO. The full inner-shareable
            // `DMB` is what a `SeqCst` fence compiles to here.
            //
            // `DSB` would also be correct and is strictly stronger: it waits
            // for completion rather than ordering (DDI 0487 §B2.3), which a
            // guest barrier does not need and which costs. The choice is
            // `DMB`, and `DSB` appears in this backend only in
            // [`buf`](super::buf), where completion really is the requirement.
            //
            // The store to `committed` is `Interp`'s, in `Interp`'s order: a
            // fence is an outward act other observers may already have seen, so
            // a `Retry` after it has nothing left to restart from. [`plan`]
            // makes this a region boundary so that the store lands *after* the
            // bookkeeping it follows has been replayed.
            Opcode::FENCE => {
                self.store_ctx_imm(off::COMMITTED, 1);
                self.asm.dmb_ish();
            }
            // Both of these emit **nothing**. Everything they do is an
            // [`Event`] in the region's flush.
            Opcode::CHARGE | Opcode::INSN_START => {}
            Opcode::GOTO_TB => {
                let pc = inst
                    .imm
                    .ok_or(Refusal::Shape("a goto_tb needs its successor's PC"))?
                    .bits() as u64;
                self.store_ctx_imm(off::OUT_PC, pc);
                self.leave(status::GOTO);
            }
            Opcode::EXIT_TB => self.leave(status::EXIT),
            Opcode::LOOKUP_AND_GOTO => {
                let s = self.src(at, 0)?;
                self.load_temp(A, s);
                self.store_ctx(A, off::OUT_PC);
                self.leave(status::LOOKUP);
            }
            other => return Err(Refusal::Op(other)),
        }
        Ok(())
    }

    /// `CMP` the two operands and return the condition to branch on.
    ///
    /// Signed comparisons at a width below 64 need the values sign-extended
    /// first: temporaries are held zero-extended, so `-1` as an `i32` is
    /// `0xffff_ffff` and would compare *above* zero rather than below it.
    fn compare(&mut self, cond: Cond, w: u32, a: Temp, b: Temp) -> Cc {
        let signed = matches!(cond, Cond::LtS | Cond::LeS | Cond::GtS | Cond::GeS);
        let (lhs, rhs) = if signed {
            // A signed comparison rewrites both operands, so neither may be
            // the register a live temporary is sitting in.
            self.load_temp(A, a);
            self.load_temp(B, b);
            self.sext(A, w);
            self.sext(B, w);
            (A, B)
        } else {
            let lhs = self.operand(A, a);
            let rhs = self.operand(B, b);
            (lhs, rhs)
        };
        self.asm.cmp(64, lhs, rhs);
        cc_of(cond)
    }

    fn movcond(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let srcs = self.block.srcs(at);
        match (inst.cond, srcs.len()) {
            (Some(cond), 4) => {
                let (a, b) = (self.src(at, 0)?, self.src(at, 1)?);
                let (t, f) = (self.src(at, 2)?, self.src(at, 3)?);
                let cc = self.compare(cond, w, a, b);
                // Only the `S` forms of A64's arithmetic write `NZCV`
                // (DDI 0487 §C6.2), so a `MOV` or an `LDR` between the compare
                // and the select cannot disturb the flags — which is what lets
                // the two candidates be fetched here.
                self.load_temp(A, t);
                let other = self.operand(B, f);
                self.asm.csel(64, A, A, other, cc);
            }
            (_, 3) => {
                let sel = self.src(at, 0)?;
                let (t, f) = (self.src(at, 1)?, self.src(at, 2)?);
                let bit = self.operand(C, sel);
                // The selector's low bit, tested without assuming the rest of
                // the register is zero — the same defensive mask the x86
                // backend's `test bit, 1` is.
                self.asm.ubfx(64, C, bit, 0, 1);
                self.asm.cmp_imm(64, C, 0);
                self.load_temp(A, t);
                let other = self.operand(B, f);
                // Non-zero means the selector's low bit was set, so take `t`.
                self.asm.csel(64, A, A, other, Cc::Ne);
            }
            _ => {
                return Err(Refusal::Shape(
                    "a movcond takes a selector and two values, or a condition and four",
                ));
            }
        }
        self.write(inst, A)
    }

    fn brcond(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let target = inst.aux as usize;
        // Forward only, and inside the block. `Liveness` is a single backward
        // walk that is exact for forward control flow and silently wrong for a
        // loop, the verifier enforces it, and a compiled backward branch would
        // additionally have no step limit to stop it.
        if target <= at || target >= self.block.insts().len() {
            return Err(Refusal::Shape(
                "a brcond branches forward, inside the block",
            ));
        }
        let f = match (inst.cond, self.block.srcs(at).len()) {
            (Some(cond), 2) => {
                let (a, b) = (self.src(at, 0)?, self.src(at, 1)?);
                let cc = self.compare(cond, w, a, b);
                self.asm.b_cond(cc)
            }
            (_, 1) => {
                let sel = self.src(at, 0)?;
                let bit = self.operand(A, sel);
                // `TBNZ` tests one bit and branches on it in a single
                // instruction, with no flags at all (DDI 0487 C6.2, *TBNZ*).
                // It reaches ±32 KiB, which `Asm::bind_to` checks; a block
                // whose lowering is longer than that is refused.
                self.asm.tbz(bit, 0, false)
            }
            _ => {
                return Err(Refusal::Shape(
                    "a brcond takes a selector, or a condition and two values",
                ));
            }
        };
        self.branches.push((f, target));
        Ok(())
    }

    /// A shift, with the out-of-range case selected rather than branched to.
    ///
    /// [`Opcode::SHL`], [`Opcode::SHR`] and [`Opcode::SAR`] are undefined in
    /// the IR when the amount reaches the type's width, and `ir::Interp` — the
    /// oracle — deliberately takes the *mathematical* answer rather than the
    /// mask-the-count behaviour both x86-64 and A64 give for free, so that a
    /// frontend which forgot its guard diverges instead of being quietly
    /// rescued. Agreeing with the oracle is the requirement.
    ///
    /// Where the x86 backend emits a compare and two branches, this emits a
    /// compare and a `CSEL`: the shifted value and the saturated one are both
    /// computed and one is chosen. That is branchless, which on a host whose
    /// out-of-range case is *never* taken by a real frontend is the right
    /// shape — a never-taken branch still occupies a predictor entry.
    fn shift(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let a = self.src(at, 0)?;
        let b = self.src(at, 1)?;
        self.load_temp(A, a);
        self.load_temp(B, b);
        let arithmetic = inst.op == Opcode::SAR;
        let (sh, out_of_range) = if arithmetic {
            // The value is held zero-extended, so an arithmetic shift has to
            // see the sign bit where the host expects it — and the answer past
            // the width is that sign, all the way down.
            self.sext(A, w);
            self.asm.asr_imm(64, C, A, 63);
            (ShiftOp::Asr, C)
        } else {
            let sh = if inst.op == Opcode::SHL {
                ShiftOp::Lsl
            } else {
                ShiftOp::Lsr
            };
            (sh, Reg::ZR)
        };
        self.asm.cmp_imm(64, B, w);
        self.asm.shift(64, sh, A, A, B);
        // Unsigned lower: the amount is a canonical zero-extended value, so an
        // `i32` amount of four billion is above 64 here rather than negative.
        self.asm.csel(64, A, A, out_of_range, Cc::Lo);
        self.write(inst, A)
    }

    // ---- memory --------------------------------------------------------

    /// The address of the next descriptor, which generated code holds as an
    /// immediate.
    fn next_descriptor(&mut self) -> Result<u64, Refusal> {
        let mem = self
            .mems
            .get(self.next_mem)
            .ok_or(Refusal::Shape("a memory op with no descriptor"))?;
        self.next_mem += 1;
        Ok(core::ptr::from_ref(mem) as u64)
    }

    /// Whether the inlined TLB probe is a correct answer for this access.
    ///
    /// Every condition is a case where the fast path and
    /// [`Tlb::read`](crate::jit::Tlb::read) would not agree, rather than a
    /// case where the fast path would merely be slower: a **segmented** access
    /// is translated before it reaches the TLB, a **separate I/O space** is not
    /// what the TLB fronts, and a width that is not a whole power of two up to
    /// eight has no single host access.
    ///
    /// Alignment and page containment are checked at run time, not here,
    /// because they are properties of the address rather than of the access.
    fn inlinable(mem: &MemOp) -> bool {
        mem.space == MemSpace::MEM
            && mem.seg.is_none()
            && matches!(mem.size, Width::U8 | Width::U16 | Width::U32 | Width::U64)
    }

    /// The inlined software-TLB probe, shared by a load and a store.
    ///
    /// `ROADMAP.md` §9.1's first mechanism: mask, compare, add. On entry `x9`
    /// holds the guest address; on the fall-through `x12` holds the host
    /// address of that guest byte and `x9` still holds the guest address.
    /// Every way of not being a hit — no plan, a misaligned address, the wrong
    /// page, the wrong world, a page with no host address — lands in the
    /// returned fixups, which the caller binds to its slow path.
    ///
    /// `base`, `mask` and `tag_bits` name the `Ctx` fields
    /// of the set to probe, which is what makes one sequence serve two sets: a
    /// load reads the set admitted on read permission and a store the one
    /// admitted on write permission.
    fn probe(&mut self, base: u64, mask: u64, tag_bits: u64, bytes: u64) -> Vec<Fixup> {
        let mut slow: Vec<Fixup> = Vec::new();
        let page_shift = PAGE_MASK.trailing_ones();
        // The set. `CBZ` is a branch on a register with no compare ahead of
        // it, which is the shape of most of this sequence.
        self.load_ctx(C, base);
        slow.push(self.asm.cbz(64, C, true));
        if bytes > 1 {
            // Natural alignment. It is also what makes the page-crossing check
            // unnecessary: an aligned access of at most eight bytes cannot
            // span two 4 KiB pages.
            let low = bytes.trailing_zeros();
            self.asm.ubfx(64, B, A, 0, low);
            slow.push(self.asm.cbz(64, B, false));
        }
        // index = (addr >> page_shift) & mask, scaled by the entry stride
        self.asm.lsr_imm(64, B, A, page_shift);
        self.load_ctx(D, mask);
        self.asm.logic(64, Logic::And, B, B, D);
        self.asm.lsl_imm(64, B, B, FastSet::STRIDE.trailing_zeros());
        self.asm.add(64, B, B, C);
        // tag = (addr & !PAGE_MASK) | context | valid. Two shifts rather than
        // a mask constant: A64 has no arithmetic against a 64-bit immediate,
        // so the alternative is a `MOVN` and an `AND` — the same two
        // instructions plus a register to hold the mask, on a sequence that
        // is already using three.
        self.asm.lsr_imm(64, D, A, page_shift);
        self.asm.lsl_imm(64, D, D, page_shift);
        self.load_ctx(C, tag_bits);
        self.asm.logic(64, Logic::Orr, D, D, C);
        if !self.asm.ldr(C, B, FastSet::TAG) {
            self.unreachable_offset = true;
        }
        self.asm.cmp(64, D, C);
        slow.push(self.asm.b_cond(Cc::Ne));
        // The host addend, zero when this page has no inline path.
        if !self.asm.ldr(D, B, FastSet::HOST) {
            self.unreachable_offset = true;
        }
        slow.push(self.asm.cbz(64, D, true));
        self.asm.add(64, D, D, A);
        slow
    }

    fn load(&mut self, at: usize, inst: &Inst) -> Result<(), Refusal> {
        let mem = inst
            .mem
            .ok_or(Refusal::Shape("a memory op needs a MemOp descriptor"))?;
        let addr = self.src(at, 0)?;
        let descriptor = self.next_descriptor()?;
        let bytes = mem.size.bytes();

        // A volatile load is a bus cycle whose occurrence the guest can
        // observe even when its value is discarded, so it commits — and it
        // commits *before* the access, because whether the fault that access
        // may take is restartable depends on it.
        if mem.volatile {
            self.store_ctx_imm(off::COMMITTED, 1);
        }

        let mut slow: Vec<Fixup> = Vec::new();
        let mut joined: Option<Fixup> = None;
        if Self::inlinable(&mem) {
            self.load_temp(A, addr);
            slow = self.probe(off::TLB_BASE, off::TLB_MASK, off::TAG_BITS, bytes);
            if !self.asm.load_zx(A, D, 0, bytes) {
                self.unreachable_offset = true;
            }
            // Park the value across the call rather than in a callee-saved
            // register, which the allocator owns; see `PARKED`.
            if !self.asm.str(A, Reg::SP, PARKED) {
                self.unreachable_offset = true;
            }
            // The tick the host's own path would have charged for this access.
            self.ctx_to_arg();
            self.call(vt::FAST_TICK);
            self.bump(off::FAST_HITS);
            if !self.asm.ldr(A, Reg::SP, PARKED) {
                self.unreachable_offset = true;
            }
            joined = Some(self.asm.b());
        }

        for f in slow {
            self.asm.bind(f);
        }
        self.ctx_to_arg();
        self.asm.mov_imm(Reg::X1, descriptor);
        self.load_temp(Reg::X2, addr);
        // The out-parameter: the scratch word the prologue reserved. `ADD` of
        // a zero immediate is A64's `MOV` from `SP`, which the ordinary
        // register `MOV` cannot express — `ORR` reads register 31 as the zero
        // register, not as `SP` (DDI 0487 C6.2, *MOV (to/from SP)*).
        self.asm
            .add_imm(64, Reg::X3, Reg::SP, u32::try_from(OUT).unwrap_or(0));
        self.call(vt::LOAD);
        let ok = self.asm.cbz(64, Reg::X0, true);
        self.fault(at)?;
        self.asm.bind(ok);
        if !self.asm.ldr(A, Reg::SP, OUT) {
            self.unreachable_offset = true;
        }

        if let Some(f) = joined {
            self.asm.bind(f);
        }
        if mem.sign == Sign::Signed {
            self.sext(A, mem.size.bits());
        }
        self.write(inst, A)
    }

    /// A store: the same inlined probe a load uses, over the store set, plus
    /// one call that pays what moving the bytes did not.
    ///
    /// Three of the four differences between a store and a load were settled
    /// at *fill* time, in the host — write permission is a different bit and
    /// therefore a different table, the architecture's first-write bookkeeping
    /// was done by the walk that filled the entry, and a protection check that
    /// may differ within one page was asked before the page was admitted. The
    /// fourth cannot be settled in advance and is [`vt::FAST_STORE`]: the
    /// `RamStore`'s own dirty bitmap, the guest-physical dirty log the
    /// dispatcher drains for self-modifying code, and whatever else the core
    /// owes a store. One call, after the bytes have landed, which is
    /// unobservable — nothing runs in between and the fast path cannot fault.
    fn store(&mut self, at: usize, inst: &Inst) -> Result<(), Refusal> {
        let mem = inst
            .mem
            .ok_or(Refusal::Shape("a memory op needs a MemOp descriptor"))?;
        let addr = self.src(at, 0)?;
        let value = self.src(at, 1)?;
        let descriptor = self.next_descriptor()?;
        let bytes = mem.size.bytes();
        let mask = mem.size.mask();

        // A store commits before it is attempted, because whether the fault it
        // may take is restartable depends on whether anything has been seen.
        self.store_ctx_imm(off::COMMITTED, 1);

        let mut slow: Vec<Fixup> = Vec::new();
        let mut joined: Option<Fixup> = None;
        if Self::inlinable(&mem) {
            self.load_temp(A, addr);
            slow = self.probe(off::ST_BASE, off::ST_MASK, off::ST_TAG, bytes);
            // The guest address is the thunk's second argument and `x9` is
            // about to hold the value, so it goes to the stack slot the
            // prologue reserved.
            if !self.asm.str(A, Reg::SP, PARKED) {
                self.unreachable_offset = true;
            }
            self.load_temp(A, value);
            // The bytes, and only the bytes: `store_trunc` writes the width
            // the guest asked for, so a neighbouring guest byte another device
            // is reading is never disturbed — and the high bits of the value
            // are discarded by the instruction rather than by a mask ahead of
            // it. The slow path *does* mask, because there the value is handed
            // to `IrHost::store`, whose contract says it arrives already
            // truncated.
            if !self.asm.store_trunc(A, D, 0, bytes) {
                self.unreachable_offset = true;
            }
            self.ctx_to_arg();
            if !self.asm.ldr(Reg::X1, Reg::SP, PARKED) {
                self.unreachable_offset = true;
            }
            self.asm.mov_imm(Reg::X2, bytes);
            self.call(vt::FAST_STORE);
            self.bump(off::FAST_WRITES);
            joined = Some(self.asm.b());
        }

        for f in slow {
            self.asm.bind(f);
        }
        self.ctx_to_arg();
        self.asm.mov_imm(Reg::X1, descriptor);
        self.load_temp(Reg::X2, addr);
        self.load_temp(Reg::X3, value);
        if mask != u64::MAX {
            self.asm.mov_imm(A, mask);
            self.asm.logic(64, Logic::And, Reg::X3, Reg::X3, A);
        }
        self.call(vt::STORE);
        let ok = self.asm.cbz(64, Reg::X0, true);
        self.fault(at)?;
        self.asm.bind(ok);
        if let Some(f) = joined {
            self.asm.bind(f);
        }
        Ok(())
    }
}

/// The condition a comparison branches on.
const fn cc_of(cond: Cond) -> Cc {
    match cond {
        Cond::Eq => Cc::Eq,
        Cond::Ne => Cc::Ne,
        Cond::LtS => Cc::Lt,
        Cond::LeS => Cc::Le,
        Cond::GtS => Cc::Gt,
        Cond::GeS => Cc::Ge,
        Cond::LtU => Cc::Lo,
        Cond::LeU => Cc::Ls,
        Cond::GtU => Cc::Hi,
        Cond::GeU => Cc::Hs,
    }
}

/// The mask of a `len`-bit field at `pos`, or `None` if it leaves the type.
///
/// The mask itself is never emitted on this host — `BFI` and `UBFX` take the
/// position and the length directly — but the *check* is the same one, and it
/// is what stops a bitfield instruction being encoded with an `imms` outside
/// its register.
const fn field_mask(pos: u32, len: u32, width: u32) -> Option<u64> {
    if len == 0 || pos + len > width || width > 64 {
        return None;
    }
    let ones = if len >= 64 {
        u64::MAX
    } else {
        (1u64 << len) - 1
    };
    Some(ones << pos)
}

/// Refuse a type no host register holds.
const fn check_type(ty: Type) -> Result<(), Refusal> {
    match ty {
        Type::I1 | Type::I32 | Type::I64 => Ok(()),
        other => Err(Refusal::Type(other)),
    }
}
