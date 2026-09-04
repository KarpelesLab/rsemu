//! The code generator: one [`Block`] in, one function's worth of x86-64 out.
//!
//! Safe Rust. It produces a `Vec<u8>` and a table of [`MemOp`] descriptors;
//! [`buf`](super::buf) is what makes those bytes executable and
//! [`rt`](super::rt) is what enters them.
//!
//! # The machine this compiles to
//!
//! **Temporaries live where [`linear_scan`] puts them**, which is
//! `ROADMAP.md` §9's pipeline finished: *"register allocation (linear scan) →
//! host backend"*. A temporary with a host register is read and written there
//! and never touches memory; one the allocator could not place keeps its slot
//! in the `u64` frame the [`Ctx`](super::rt::Ctx) points at, and is loaded and
//! stored exactly as the whole backend used to work. [`Regs::Frame`] switches
//! the allocator off and is that earlier backend, kept runnable as the control
//! every differential and both benchmarks compare against.
//!
//! ## Precise state at a fault, which is where the allocator gets interesting
//!
//! §9: *"when a load faults halfway through a translated block, the guest must
//! observe exactly the architectural state its ISA specifies"*, and the design
//! it names for that is a side table keyed by host code offset, so the runtime
//! can materialize architectural state from wherever the allocator left it.
//!
//! **This backend does not need one, and the reason is a measurement rather
//! than a preference.** A temporary an [`InsnStart`](crate::ir::InsnStart)
//! names is *written through*: it goes to its host register and to its frame
//! slot, once, at its definition, and stays right for the rest of the block
//! because the IR is SSA. The exception path then reads the frame exactly as
//! it always did — `rt`'s `publish` is unchanged — and reconstructs the same
//! state the interpreter would have had. Everything the boundaries do **not**
//! name is dead the moment its last reader has run, and never reaches memory
//! at all.
//!
//! The alternative was tried on paper and is worse *here*, which is the part
//! worth writing down. A side table lets the write-through go, but only if the
//! value is still in a register when the fault happens — so every temporary a
//! boundary names has to stay live until the boundary's extent ends, not until
//! its last reader. On `cpu::riscv::lift` a boundary's live map names **every
//! shadowed guest register**: a trace of the benchmark's `alu-loop` has 85
//! temporaries, 57 of them named at some boundary, over three callee-saved
//! host registers. Extending 57 intervals to overlap would spill all but three
//! of them, which is the write-through's cost paid as a spill plus a reload.
//! The table becomes the right trade when a frontend names few registers per
//! boundary or a host has thirty-two of them; it is not this frontend on this
//! host, and [`Allocation::frame_backed`] is where the decision is stated.
//!
//! ## Deferred bookkeeping, which is where the guest instructions went
//!
//! A lifted RV64I trace is mostly bookkeeping: 130 of its 215 IR instructions
//! were call sites, because every guest instruction is an
//! [`Opcode::INSN_START`] and an [`Opcode::CHARGE`] and both used to be a
//! thunk call with a fistful of context stores around it. Neither emits a
//! single byte now. They are **replayed** instead, by
//! [`rt`](super::rt)'s `flush_thunk` — one call per *region* — and a region is
//! usually the whole block.
//!
//! What makes a static range the right answer is a property of the block
//! rather than an assumption about it. A flush is emitted ahead of every
//! instruction whose lowering can let the host observe anything — a load, a
//! store, a slot read — ahead of every terminator, ahead of every
//! [`Opcode::BRCOND`], and ahead of every instruction a `brcond` targets. Each
//! of those also **starts** a region. So no branch and no branch target lies
//! strictly inside a region, every path that reaches a flush entered its
//! region at the top, and the events in `[region, here)` are exactly the ones
//! that ran. A branch lands *after* the flush at its target — that is
//! what `Compiler::starts` records — so the taken path arrives with nothing
//! pending and the fall-through arrives having replayed the range the branch
//! skipped.
//!
//! Nothing is batched and nothing is reordered: the replay makes the same
//! calls, with the same arguments, in the same order `ir::Interp` makes them,
//! and it writes the same context fields — [`Ctx::ticks`](super::rt::Ctx),
//! `retired`, `boundaries`, `boundary_pc`, `mark`, `committed`, `published`.
//! A fault therefore still reports the interpreter's exact tick count: the
//! faulting access is a call site, so its region was replayed *before* the
//! access was attempted, and `retired` is the charged count as of the last
//! boundary the run actually passed.
//!
//! The three things generated code still writes itself are `committed`, which
//! a store and a volatile load set after their flush, and the two counters
//! `fast_hits` and `fast_writes`. That is the whole of a guest instruction's
//! per-instruction cost now: nothing.
//!
//! ### What it costs, which is in the allocator
//!
//! A flush is a call in the *gap* ahead of an instruction, so a value whose
//! **last use** is that instruction's operand — a branch condition, a load's
//! address — needs a callee-saved register where rule 2's "strictly between"
//! deliberately let it keep a volatile one, and there are three of those.
//!
//! `benches/jit_dispatch.rs` never pays it: RV64I traces gained 1.11–1.50×,
//! and the allocator's own margin over [`Regs::Frame`] went **up**, because
//! the bookkeeping it used to be measured through is gone. Where it could
//! bite is `benches/x86_dispatch.rs`'s `branchy` and `load-heavy`, which put a
//! `brcond` or an access at nearly every guest instruction. Measured twice
//! against the same baseline, those two came out at 0.94–0.95× and at
//! 1.02–1.07×, which is the machine's spread rather than a number: call them
//! flat, and note that `alu-loop`, `memcpy` and `chain` gained 1.06–1.26× in
//! both runs. The mechanism is real even where the measurement is not, so it
//! is written down rather than assumed away.
//!
//! Moving a `brcond`'s flush onto the branch's **taken edge**, as an
//! out-of-line pad, would take the gap call off the condition's interval and
//! is the obvious next thing to try. It is not obviously a win, which is why
//! it is a measurement and not a patch: in a superblock the taken edge is the
//! one that stays in the trace, so the pad would land on the hot path.
//!
//! Values are held **canonically masked to their type**, exactly as
//! `ir::Interp` holds them: an `i32` temporary never carries bits above 32 and
//! an `i1` never carries bits above 1. Every op therefore assumes canonical
//! inputs and masks its own output exactly once, which is the same contract
//! the oracle keeps and the reason the two can be compared value for value.
//!
//! # The register assignment
//!
//! | register | holds |
//! | --- | --- |
//! | `rbx` | the [`Ctx`](super::rt::Ctx) |
//! | `r12` | the temporary frame |
//! | `r14` | the thunk table |
//! | `rax` `rcx` `rdx` `rsi` `rdi` | scratch |
//! | `rbp` `r13` `r15` | allocated to intervals that cross a call |
//! | `r8` `r9` `r10` `r11` | allocated to intervals that do not |
//!
//! The three pinned ones are callee-saved, so a thunk call cannot lose them.
//! `r15` used to hold the TLB's load set for the whole block and `r13` used to
//! be the scratch that survived a call; both now go to the allocator, because
//! three callee-saved registers instead of one is the difference between an
//! allocation that can hold a guest register across a boundary and one that
//! cannot. What that cost is written down where it is paid: the inlined probe
//! reloads its set from the context, and parks a value across the call that
//! follows it — the loaded word across `note_fast_load`, the guest address
//! across `note_fast_store`.
//!
//! Seven registers is the ceiling, and it is worth knowing why it is so low.
//! Generated code still calls into the host at every `get_slot`, on every
//! access, and at each region's flush, so anything that outlives one of those
//! needs a callee-saved register, and System V has six of which two are
//! already the context and the frame. The ceiling has not moved; what moved is
//! how many calls a block has, which the deferred bookkeeping cut from two per
//! guest instruction to roughly one per block.
//!
//! # Out-of-range shifts
//!
//! [`Opcode::SHL`], [`Opcode::SHR`] and [`Opcode::SAR`] are undefined in the
//! IR when the amount reaches the type's width, and `ir::Interp` — the oracle
//! — deliberately takes the *mathematical* answer rather than the
//! mask-the-count behaviour x86-64 gives for free, so that a frontend which
//! forgot its guard diverges instead of being quietly rescued. This backend
//! therefore emits the compare and the branch rather than taking the free
//! behaviour: agreeing with the oracle is the requirement, and "the host does
//! it differently for nothing" is precisely the divergence the interpreter's
//! choice exists to expose.
//!
//! # Baseline instructions only
//!
//! No `popcnt`, no `lzcnt`, no `tzcnt`, no BMI — see [`emit`](super::emit).
//! Population count is the SWAR sequence and the zero counts are `BSR`/`BSF`
//! with an explicit zero case, so the generated code runs on any x86-64.

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

use super::emit::{Alu, Asm, Cc, Fixup, Reg, Shift};
use super::rt::{Event, off, status, vt};

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
    /// than 64 bits, and both float types — tier-1 floating point is a helper
    /// call, and helper calls are refused too.
    Type(Type),
    /// The block is shaped in a way the compiler will not take: a branch that
    /// is not forward, a missing terminator, an operand count that does not
    /// match the op, a bitfield outside its type.
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
/// The union of what the RISC-V and x86 frontends emit, plus the neighbours
/// that cost nothing once their family is in. What is *not* here, and why:
///
/// * **The atomics and `fence`** — a guest atomic has to reach the host's
///   atomic, and `IrHost::rmw` is the seam that does it. Inlining one means
///   deciding the host memory model in generated code, which is
///   `ROADMAP.md` §9.1's sixth mechanism and not this one.
/// * **`call_helper`** — arbitrary Rust, and a barrier for the register
///   mapping (`ir`'s decision 4). Cheap to add later; nothing emits one yet.
/// * **`phi`** — cannot be executed as defined, in this backend or the
///   interpreter (`ir`'s "Known gaps").
/// * **`div`/`rem`, `addc`/`subb`, `mulhsu`** — no frontend in the tree emits
///   one. Lowering an op nothing exercises would mean shipping untested code
///   generation, which is worse than shipping none: the differential harnesses
///   cannot reach it.
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
            | Opcode::MULU2
            | Opcode::MULS2
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
            | Opcode::ROTLC
            | Opcode::ROTRC
            | Opcode::CLZ
            | Opcode::CTZ
            | Opcode::POPCOUNT
            | Opcode::SETCOND
            | Opcode::MOVCOND
            | Opcode::BRCOND
            | Opcode::LD
            | Opcode::ST
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
/// why it is a box and not a `Vec` field — a `Vec` that were ever pushed to
/// would reallocate under the generated code.
#[derive(Debug)]
pub struct Compiled {
    code: Vec<u8>,
    /// Never read from Rust after compilation — and that is the point. Its
    /// element *addresses* are immediates in the code above, so what this
    /// field does is keep the allocation alive for as long as the code that
    /// points into it. Dropping it would leave the generated `mov rsi, imm64`
    /// naming freed memory.
    #[allow(dead_code)]
    mems: Box<[MemOp]>,
    /// The block's deferred bookkeeping, in instruction order.
    ///
    /// A `Box<[Event]>` for the same reason [`Compiled::mems`] is one: the
    /// runtime is handed a pointer into it and a range, and a `Vec` that were
    /// ever pushed to would move the allocation under a running block.
    events: Box<[Event]>,
    /// Where the register allocator put every temporary.
    ///
    /// Carried forward because the *runtime* needs it and cannot re-derive it:
    /// after a run, a temporary that lived only in a host register is gone,
    /// and reading its frame slot would hand back the zero the frame was
    /// cleared to. Every temporary a boundary names is
    /// [`frame_backed`](Allocation::frame_backed) by construction — that is
    /// what keeps `ROADMAP.md` §9's precise exceptions exact.
    alloc: Allocation,
    offset: u64,
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
    ///
    /// Exposed so that a test can check the *invariant* rather than only its
    /// consequences: a value that outlives a call and sits in a register the
    /// callee may destroy is wrong even on the runs where the callee happens
    /// not to destroy it, and that is not a property a differential can be
    /// relied on to show — see
    /// `no_value_that_outlives_a_call_is_left_where_a_call_can_destroy_it`.
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
/// A switch rather than a constant for two reasons, and neither is taste.
/// [`Regs::Frame`] is the **control** every differential in this backend runs
/// against — the same block, the same host, the same everything, with the
/// allocator switched off — which is what turns "the two engines agree" into
/// "the allocator did not change the answer". And it is the column
/// `benches/jit_dispatch.rs` and `benches/x86_dispatch.rs` compare against,
/// because a register allocator whose win is asserted rather than measured is
/// a register allocator nobody can tell has regressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Regs {
    /// Every temporary lives in the frame: the backend as it stood before the
    /// allocator, kept runnable.
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

/// The host registers a temporary may live in, by [`Reg`] number.
///
/// Three saved and four volatile out of sixteen, and the arithmetic is worth
/// stating because it is the whole ceiling on what the allocator can do here.
/// System V gives six callee-saved registers; `rbx` holds the context and
/// `r12` the frame, `r14` holds the thunk table because a call happens two or
/// three times per guest instruction and reloading it each time costs more
/// than the register is worth, and the remaining three are these. `rax`,
/// `rcx`, `rdx`, `rsi` and `rdi` stay fixed scratch: the lowerings for
/// `popcount`, `bswap`, `deposit` and the inlined TLB probe each need three or
/// four registers they may destroy, and `rcx` is the only register a variable
/// shift can take its count from.
pub(super) const SAVED: [u8; 3] = [Reg::Rbp as u8, Reg::R13 as u8, Reg::R15 as u8];

/// The registers a thunk call may destroy, which the allocator gives only to
/// intervals that do not span one.
pub(super) const VOLATILE: [u8; 4] = [Reg::R8 as u8, Reg::R9 as u8, Reg::R10 as u8, Reg::R11 as u8];

/// A register number back into a [`Reg`].
///
/// Only the numbers [`SAVED`] and [`VOLATILE`] contain can reach here, and an
/// allocation naming anything else is a bug in this file rather than in a
/// guest — so the fallback is `rax`, which every lowering already treats as
/// destroyed, rather than a panic in generated-code emission.
const fn reg_of(n: u8) -> Reg {
    match n {
        5 => Reg::Rbp,
        8 => Reg::R8,
        9 => Reg::R9,
        10 => Reg::R10,
        11 => Reg::R11,
        13 => Reg::R13,
        15 => Reg::R15,
        _ => Reg::Rax,
    }
}

/// Whether this backend's lowering of `op` calls into the host *after*
/// reading its operands and *before* writing its results.
///
/// One of the two inputs [`linear_scan`] cannot check for itself, so it is one
/// expression that can be read against the lowerings — and
/// `the_call_map_the_allocator_is_given_matches_what_the_lowerings_emit`
/// asserts it against the emitted bytes rather than against this list.
///
/// `charge` and `insn_start` are **not** here any more, and that is not an
/// omission: neither emits an instruction. Their call is the region's flush,
/// which is [`CallSites::before`] instead, because it runs ahead of the
/// instruction it is attached to rather than inside it.
const fn calls_inside(op: Opcode) -> bool {
    matches!(
        op,
        // Every access has a slow path with a call on it, whether or not the
        // fast path is taken at run time, and the fast path has its own call
        // to `note_fast_load`.
        Opcode::LD | Opcode::ST | Opcode::GET_SLOT
    )
}

/// The deferred bookkeeping of one block, and where each region of it is
/// replayed.
#[derive(Debug)]
struct Plan {
    /// Every [`Opcode::CHARGE`] and [`Opcode::INSN_START`] the block contains,
    /// in instruction order — the array [`Event`] documents.
    events: Vec<Event>,
    /// `at[i]` is the half-open range of [`Plan::events`] a flush emitted in
    /// the gap ahead of instruction `i` replays, and `None` when no flush is
    /// emitted there — the common case, because a region with nothing in it
    /// has nothing to replay.
    at: Vec<Option<(u32, u32)>>,
}

/// Collect a block's bookkeeping and decide where to replay it.
///
/// See the module docs for why a static range is exactly what ran. The rule is
/// one pass: an instruction is a **region boundary** when it can let the host
/// observe something ([`calls_inside`]), when it is a terminator, when it is a
/// `brcond`, or when a `brcond` targets it. At a boundary the pending range is
/// flushed if it holds anything, and a new region starts at that instruction
/// whether or not anything was flushed — dropping an empty range loses
/// nothing, and it is what keeps the two sides of a branch agreeing about
/// where their region began.
///
/// # Errors
///
/// [`Refusal`] for a charge with no tick count or a boundary marker that
/// points at no record. Refused here rather than replayed as a charge of zero
/// or a boundary that never happens: the interpreter reports such a block as
/// an error, and the two engines have to agree about that too.
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
        if target[i] || calls_inside(op) || op == Opcode::BRCOND || op.is_terminator() {
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
                events.push(Event::Charge(ticks));
            }
            Opcode::INSN_START => {
                block
                    .marks()
                    .get(inst.aux as usize)
                    .ok_or(Refusal::Shape("the boundary marker points at no record"))?;
                events.push(Event::Boundary(inst.aux));
            }
            _ => {}
        }
    }
    // One event per instruction at most, and `Compiler::new` has already
    // refused a block whose instruction count does not fit in an `i32`, so
    // every index a flush carries is a `u32` that fits.
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
}

/// The stack this backend reserves below its saved registers.
///
/// Forty bytes: eight for a load thunk's out-parameter, eight for
/// [`PARKED`], and twenty-four of padding that keeps `rsp` sixteen-byte
/// aligned at every `call`, which the System V AMD64 ABI requires and a
/// violation of which shows up as a fault inside some unrelated library's SSE
/// prologue rather than here.
const FRAME: i32 = 0x28;

/// Where the inlined load parks its value across `note_fast_load`.
///
/// It used to be `r13`, and `r13` is now a register the allocator hands out —
/// three callee-saved registers instead of two is a third more of the only
/// bank that can hold a value across a call, and the fast path pays two stack
/// accesses for it on a path that already has a dozen instructions.
const PARKED: i32 = 8;

impl<'a> Compiler<'a> {
    fn new(block: &'a Block, regs: Regs) -> Result<Compiler<'a>, Refusal> {
        // A temporary's frame slot is reached with a 32-bit displacement.
        if block
            .temp_count()
            .checked_mul(8)
            .is_none_or(|n| i32::try_from(n).is_err())
        {
            return Err(Refusal::Shape("the block has too many temporaries"));
        }
        // Every instruction index reaches generated code as an immediate —
        // a fault's `at`, a flush's range — so the whole block has to fit in
        // one.
        if i32::try_from(block.insts().len()).is_err() {
            return Err(Refusal::Shape("the block is too long"));
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
        })
    }

    fn run(mut self) -> Result<Compiled, Refusal> {
        let insts = self.block.insts();
        if !insts.last().is_some_and(|i| i.op.is_terminator()) {
            return Err(Refusal::Shape("the block does not end in a terminator"));
        }
        self.prologue();
        for (at, inst) in insts.iter().enumerate() {
            // Before the position a branch lands on, so the taken path skips
            // the range it did not execute — see the module docs.
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
        Ok(Compiled {
            code: self.asm.finish(),
            mems: self.mems,
            events: self.plan.events.into_boxed_slice(),
            alloc: self.alloc,
            offset: 0,
        })
    }

    // ---- frame ---------------------------------------------------------

    fn prologue(&mut self) {
        for r in [Reg::Rbx, Reg::Rbp, Reg::R12, Reg::R13, Reg::R14, Reg::R15] {
            self.asm.push(r);
        }
        self.asm.alu_ri(Alu::Sub, Reg::Rsp, FRAME);
        self.asm.mov_rr(Reg::Rbx, Reg::Rdi);
        self.asm.mov_rm(Reg::R12, Reg::Rbx, off::TEMPS);
        self.asm.mov_rm(Reg::R14, Reg::Rbx, off::VT);
        // `r15` used to hold the TLB's load set for the whole block and now
        // belongs to the allocator; the inlined probe loads it from the
        // context instead, which is one `mov` on a path with a dozen.
    }

    fn epilogue(&mut self) {
        for f in core::mem::take(&mut self.exits) {
            self.asm.bind(f);
        }
        self.asm.alu_ri(Alu::Add, Reg::Rsp, FRAME);
        for r in [Reg::R15, Reg::R14, Reg::R13, Reg::R12, Reg::Rbp, Reg::Rbx] {
            self.asm.pop(r);
        }
        self.asm.ret();
    }

    /// Leave the block with `code` in `rax`.
    fn leave(&mut self, code: u64) {
        self.asm.mov_ri(Reg::Rax, code);
        let f = self.asm.jmp();
        self.exits.push(f);
    }

    // ---- operands ------------------------------------------------------

    fn frame(temp: Temp) -> i32 {
        (temp.index() as i32) * 8
    }

    /// The host register `temp` lives in, if it lives in one.
    fn home(&self, temp: Temp) -> Option<Reg> {
        match self.alloc.home(temp) {
            Home::Reg(n) => Some(reg_of(n)),
            Home::Frame => None,
        }
    }

    /// Get `temp` into `reg`, which the caller may then destroy.
    ///
    /// A register-to-register move where the allocator gave it a home and a
    /// load out of the frame where it did not. The move is what the allocator
    /// bought: it is renamed away on any out-of-order host, where the load is
    /// an address computation and an L1 access even on a hit.
    fn load_temp(&mut self, reg: Reg, temp: Temp) {
        match self.home(temp) {
            Some(home) => {
                if home != reg {
                    self.asm.mov_rr(reg, home);
                }
            }
            None => {
                let at = Self::frame(temp);
                self.asm.mov_rm(reg, Reg::R12, at);
            }
        }
    }

    /// A **read-only** operand: the register holding `temp`, without copying it
    /// when it already lives in one.
    ///
    /// The caller promises not to write the register this returns, which is
    /// why it is a separate method from [`Compiler::load_temp`] rather than an
    /// optimisation inside it. Every use of it below is an instruction that
    /// only reads its second operand — the `r/m` side of an `add`, the source
    /// of a `cmov`, the value a `test` looks at — and getting that wrong
    /// corrupts a temporary that is still live, which is exactly the class of
    /// bug that shows up three instructions later as a wrong guest register.
    fn operand(&mut self, scratch: Reg, temp: Temp) -> Reg {
        match self.home(temp) {
            Some(home) => home,
            None => {
                self.load_temp(scratch, temp);
                scratch
            }
        }
    }

    /// The register an op should compute into.
    ///
    /// The destination's own home where there is one, which turns the common
    /// three-instruction shape — fetch, operate, put back — into two, or into
    /// one where the destination inherited an operand's register. `rax`
    /// otherwise, which is the pre-allocator behaviour.
    ///
    /// `blocked` is the operands still read **after** the accumulator is first
    /// written: the right-hand side of a two-operand sequence. An operand read
    /// only *into* the accumulator is deliberately not blocked, and the reason
    /// is a property of the allocator rather than luck. A register is handed to
    /// the destination only out of the free set, and an operand's register is
    /// free at this instruction exactly when that operand's interval ended
    /// here — so `home(dst) == home(a)` *is* the statement that `a` is dead
    /// after this instruction, and writing over it is writing over a value
    /// nothing will read again.
    fn acc(&self, inst: &Inst, blocked: &[Temp]) -> Reg {
        let Some(dst) = inst.dst else { return Reg::Rax };
        let Some(home) = self.home(dst) else {
            return Reg::Rax;
        };
        if blocked.iter().any(|t| self.home(*t) == Some(home)) {
            Reg::Rax
        } else {
            home
        }
    }

    fn store_temp(&mut self, temp: Temp, reg: Reg) {
        let at = Self::frame(temp);
        self.asm.mov_mr(Reg::R12, at, reg);
    }

    /// Put `reg`'s value where `temp` lives — and, when the frame is also its
    /// home, in both places.
    ///
    /// The write-through is `ROADMAP.md` §9's precise-exception requirement in
    /// this backend. A temporary an [`InsnStart`](crate::ir::InsnStart) names
    /// is read by the *exception path*, out of the frame, from outside the
    /// generated code — so it reaches the frame at its definition, once, and
    /// stays right for the rest of the block because the IR is SSA. Everything
    /// else is dead the moment its last reader has run and never touches
    /// memory at all.
    fn commit(&mut self, temp: Temp, reg: Reg) {
        if let Some(home) = self.home(temp) {
            if home != reg {
                self.asm.mov_rr(home, reg);
            }
            if self.alloc.frame_backed(temp) {
                self.store_temp(temp, reg);
            }
        } else {
            self.store_temp(temp, reg);
        }
    }

    /// Canonicalise `reg` to `ty`, as `Interp::set` does.
    fn mask(&mut self, reg: Reg, ty: Type) {
        match ty {
            Type::I1 => self.asm.alu_ri(Alu::And, reg, 1),
            Type::I32 => self.asm.mov_rr32(reg, reg),
            Type::I64 => {}
            // Refused at `check_type`, so this is unreachable rather than
            // wrong; masking nothing is still the conservative answer.
            _ => {}
        }
    }

    /// Sign-extend the low `bits` of `reg` through the whole register.
    fn sext(&mut self, reg: Reg, bits: u32) {
        if bits < 64 {
            let shift = (64 - bits) as u8;
            self.asm.shift_ri(Shift::Shl, reg, shift);
            self.asm.shift_ri(Shift::Sar, reg, shift);
        }
    }

    /// Write `reg` into the instruction's destination.
    ///
    /// `ir::Interp` applies **two** masks here and this reproduces both: the
    /// arithmetic ops mask their result to the *instruction's* width, and every
    /// write then masks to the *destination temporary's*. Usually the two are
    /// the same type — `BlockBuilder::emit` allocates the destination with the
    /// instruction's type — but not always, and the exception is not exotic:
    /// [`Opcode::SETCOND`]'s type is the type it *compared* and its destination
    /// is one bit, which the verifier insists on.
    ///
    /// A destination **wider** than the instruction is refused, because there
    /// the two orders differ: the ops that do not mask to the instruction's
    /// width would keep bits this backend has already dropped. Nothing in the
    /// tree emits one, and refusing costs a block on the interpreter.
    ///
    /// `reg` is masked **in place**, so it must be a scratch register and
    /// never one [`Compiler::operand`] handed back — that one is some live
    /// temporary's home.
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
    /// **The IR does not require the two to agree**, and the verifier does not
    /// check it — so an `i32` `rotl` over an `i64` operand is a legal block, and
    /// `ir::Interp` then computes on the operand's *own* width and masks the
    /// result at the instruction's. For the ops where the two answers differ —
    /// the rotates, the bit counts, `bswap`, and the widening multiplies —
    /// reproducing that would mean carrying the interpreter's accident into
    /// generated code, and doing anything else would be a silent divergence.
    /// So it is refused, and the interpreter runs the block.
    ///
    /// Neither frontend in the tree emits one: `cpu::riscv::lift` truncates to
    /// `i32` before every word-width operation, and `cpu::x86::lift` is `i64`
    /// and `i1` throughout. The narrower ops — `add`, `and`, `shl` and their
    /// neighbours — are unaffected either way, because masking the result is
    /// the whole difference, so they are not checked.
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

    /// `call [r14 + slot]`, with the context already in `rdi`.
    fn call(&mut self, slot: i32) {
        self.asm.call_m(Reg::R14, slot);
    }

    fn ctx_to_rdi(&mut self) {
        self.asm.mov_rr(Reg::Rdi, Reg::Rbx);
    }

    /// Add one to a `u64` context field, in one instruction and without
    /// destroying a register.
    fn bump(&mut self, field: i32) {
        self.asm.alu_mi(Alu::Add, Reg::Rbx, field, 1);
    }

    /// Replay events `lo .. hi`: the region's charges and boundaries.
    ///
    /// See the module docs for why the range is exactly what ran, and
    /// [`rt`](super::rt)'s `flush_thunk` for what it does with it.
    fn flush(&mut self, lo: u32, hi: u32) {
        self.ctx_to_rdi();
        self.asm.mov_ri32(Reg::Rsi, lo);
        self.asm.mov_ri32(Reg::Rdx, hi);
        self.call(vt::FLUSH);
    }

    /// The sequence a faulting access jumps to: record where and why, and
    /// leave. `rax` holds the error code on entry.
    ///
    /// No flush: this is only ever emitted inside a load or a store, which is
    /// a region boundary, so everything up to the faulting access has already
    /// been replayed — which is exactly what makes the fault's reported tick
    /// count the interpreter's.
    fn fault(&mut self, at: usize) -> Result<(), Refusal> {
        let at = i32::try_from(at).map_err(|_| Refusal::Shape("the block is too long"))?;
        self.asm.mov_mr(Reg::Rbx, off::FAULT_ERROR, Reg::Rax);
        self.asm.mov_mi(Reg::Rbx, off::FAULT_AT, at);
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
                    (None, Some(c)) => self.asm.mov_ri(acc, c.bits() as u64),
                    (None, None) => {
                        return Err(Refusal::Shape("a mov needs a source or an immediate"));
                    }
                }
                self.write(inst, acc)?;
            }
            Opcode::GET_SLOT => {
                self.ctx_to_rdi();
                self.asm.mov_ri32(Reg::Rsi, inst.aux & 0xffff);
                self.call(vt::GET_SLOT);
                self.write(inst, Reg::Rax)?;
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
                self.load_temp(Reg::Rax, s);
                if lane == w && (w == 32 || w == 64) {
                    // The whole-type case, which is one instruction.
                    if w == 64 {
                        self.asm.bswap64(Reg::Rax);
                    } else {
                        self.asm.bswap32(Reg::Rax);
                    }
                } else {
                    // A lane narrower than the type: `BSWAP` cannot express it,
                    // so it is the swap cascade — bytes within halfwords, then
                    // halfwords within words, then words — stopped at the lane
                    // width. x86's `BSWAP r16` is documented as undefined, and
                    // ARM's `REV16` is exactly this shape, which is why the op
                    // carries a lane width at all.
                    //
                    // The masks span the whole register rather than the type's
                    // width, which is correct because a value is held
                    // canonically masked: the bits above `w` are zero going in
                    // and are masked off again coming out.
                    for (shift, mask) in [
                        (8u8, 0x00ff_00ff_00ff_00ffu64),
                        (16, 0x0000_ffff_0000_ffff),
                        (32, 0x0000_0000_ffff_ffff),
                    ] {
                        if lane <= u32::from(shift) {
                            break;
                        }
                        self.asm.mov_rr(Reg::Rcx, Reg::Rax);
                        self.asm.mov_ri(Reg::Rdx, mask);
                        self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rdx);
                        self.asm.shift_ri(Shift::Shl, Reg::Rcx, shift);
                        self.asm.shift_ri(Shift::Shr, Reg::Rax, shift);
                        self.asm.alu_rr(Alu::And, Reg::Rax, Reg::Rdx);
                        self.asm.alu_rr(Alu::Or, Reg::Rax, Reg::Rcx);
                    }
                }
                self.write(inst, Reg::Rax)?;
            }
            Opcode::DEPOSIT => {
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or(Refusal::Shape("the bitfield leaves the type"))?;
                let into = self.src(at, 0)?;
                let what = self.src(at, 1)?;
                self.load_temp(Reg::Rax, into);
                self.asm.mov_ri(Reg::Rcx, !field);
                self.asm.alu_rr(Alu::And, Reg::Rax, Reg::Rcx);
                self.load_temp(Reg::Rcx, what);
                if pos > 0 {
                    self.asm.shift_ri(Shift::Shl, Reg::Rcx, pos as u8);
                }
                self.asm.mov_ri(Reg::Rdx, field);
                self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rdx);
                self.asm.alu_rr(Alu::Or, Reg::Rax, Reg::Rcx);
                self.write(inst, Reg::Rax)?;
            }
            Opcode::EXTRACT => {
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or(Refusal::Shape("the bitfield leaves the type"))?;
                let s = self.src(at, 0)?;
                self.load_temp(Reg::Rax, s);
                self.asm.mov_ri(Reg::Rcx, field);
                self.asm.alu_rr(Alu::And, Reg::Rax, Reg::Rcx);
                if pos > 0 {
                    self.asm.shift_ri(Shift::Shr, Reg::Rax, pos as u8);
                }
                self.write(inst, Reg::Rax)?;
            }
            Opcode::ADD | Opcode::SUB | Opcode::AND | Opcode::OR | Opcode::XOR => {
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let acc = self.acc(inst, &[b]);
                self.load_temp(acc, a);
                let rhs = self.operand(Reg::Rcx, b);
                let alu = match op {
                    Opcode::ADD => Alu::Add,
                    Opcode::SUB => Alu::Sub,
                    Opcode::AND => Alu::And,
                    Opcode::OR => Alu::Or,
                    _ => Alu::Xor,
                };
                self.asm.alu_rr(alu, acc, rhs);
                self.write(inst, acc)?;
            }
            Opcode::ANDC => {
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                // `b` is read after the accumulator is first written, so the
                // accumulator may not be the register `b` lives in.
                let acc = self.acc(inst, &[b]);
                self.load_temp(acc, a);
                self.load_temp(Reg::Rcx, b);
                self.asm.not(Reg::Rcx);
                self.asm.alu_rr(Alu::And, acc, Reg::Rcx);
                self.write(inst, acc)?;
            }
            Opcode::MUL => {
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let acc = self.acc(inst, &[b]);
                self.load_temp(acc, a);
                let rhs = self.operand(Reg::Rcx, b);
                self.asm.imul_rr(acc, rhs);
                self.write(inst, acc)?;
            }
            Opcode::NEG => {
                let a = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                self.load_temp(acc, a);
                self.asm.neg(acc);
                self.write(inst, acc)?;
            }
            Opcode::NOT => {
                let a = self.src(at, 0)?;
                let acc = self.acc(inst, &[]);
                self.load_temp(acc, a);
                self.asm.not(acc);
                self.write(inst, acc)?;
            }
            Opcode::MULU2 | Opcode::MULS2 => self.widening_multiply(at, inst, w)?,
            Opcode::SHL | Opcode::SHR | Opcode::SAR => self.shift(at, inst, w)?,
            Opcode::ROTL | Opcode::ROTR => {
                if w != 32 && w != 64 {
                    return Err(Refusal::Shape("a rotate is 32 or 64 bits wide"));
                }
                let a = self.src_typed(at, 0, inst.ty)?;
                let b = self.src(at, 1)?;
                self.load_temp(Reg::Rax, a);
                self.load_temp(Reg::Rcx, b);
                let sh = if op == Opcode::ROTL {
                    Shift::Rol
                } else {
                    Shift::Ror
                };
                // `cl` is masked to six bits at 64 and five at 32, which is
                // exactly the amount modulo the width the IR asks for.
                if w == 64 {
                    self.asm.shift_rcl(sh, Reg::Rax);
                } else {
                    self.asm.shift_rcl32(sh, Reg::Rax);
                }
                self.write(inst, Reg::Rax)?;
            }
            Opcode::ROTLC | Opcode::ROTRC => self.rotate_through_carry(at, inst, w)?,
            Opcode::CLZ | Opcode::CTZ => self.count_zeros(at, inst, w)?,
            Opcode::POPCOUNT => {
                let a = self.src_typed(at, 0, inst.ty)?;
                self.load_temp(Reg::Rax, a);
                self.popcount();
                self.write(inst, Reg::Rax)?;
            }
            Opcode::SETCOND => {
                let cond = inst
                    .cond
                    .ok_or(Refusal::Shape("a comparison needs a condition"))?;
                let a = self.src(at, 0)?;
                let b = self.src(at, 1)?;
                let cc = self.compare(cond, w, a, b);
                self.asm.setcc(cc, Reg::Rax);
                self.write(inst, Reg::Rax)?;
            }
            Opcode::MOVCOND => self.movcond(at, inst, w)?,
            Opcode::BRCOND => self.brcond(at, inst, w)?,
            Opcode::LD => self.load(at, inst)?,
            Opcode::ST => self.store(at, inst)?,
            // Both of these emit **nothing**. Everything they do is an
            // [`Event`] in the region's flush — see the module docs — and
            // [`plan`] is where a malformed one is refused.
            Opcode::CHARGE | Opcode::INSN_START => {}
            Opcode::GOTO_TB => {
                let pc = inst
                    .imm
                    .ok_or(Refusal::Shape("a goto_tb needs its successor's PC"))?
                    .bits() as u64;
                self.asm.mov_ri(Reg::Rax, pc);
                self.asm.mov_mr(Reg::Rbx, off::OUT_PC, Reg::Rax);
                self.leave(status::GOTO);
            }
            Opcode::EXIT_TB => self.leave(status::EXIT),
            Opcode::LOOKUP_AND_GOTO => {
                let s = self.src(at, 0)?;
                self.load_temp(Reg::Rax, s);
                self.asm.mov_mr(Reg::Rbx, off::OUT_PC, Reg::Rax);
                self.leave(status::LOOKUP);
            }
            other => return Err(Refusal::Op(other)),
        }
        Ok(())
    }

    /// `cmp` the two operands and return the condition to branch on.
    ///
    /// Signed comparisons at a width below 64 need the values sign-extended
    /// first: temporaries are held zero-extended, so `-1` as an `i32` is
    /// `0xffff_ffff` and would compare *above* zero rather than below it.
    fn compare(&mut self, cond: Cond, w: u32, a: Temp, b: Temp) -> Cc {
        let signed = matches!(cond, Cond::LtS | Cond::LeS | Cond::GtS | Cond::GeS);
        self.load_temp(Reg::Rax, a);
        // A signed comparison rewrites both operands, so neither may be the
        // register a live temporary is sitting in; an unsigned one only reads
        // the right-hand side.
        let rhs = if signed {
            self.load_temp(Reg::Rcx, b);
            self.sext(Reg::Rax, w);
            self.sext(Reg::Rcx, w);
            Reg::Rcx
        } else {
            self.operand(Reg::Rcx, b)
        };
        self.asm.alu_rr(Alu::Cmp, Reg::Rax, rhs);
        cc_of(cond)
    }

    fn movcond(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let srcs = self.block.srcs(at);
        match (inst.cond, srcs.len()) {
            (Some(cond), 4) => {
                let (a, b) = (self.src(at, 0)?, self.src(at, 1)?);
                let (t, f) = (self.src(at, 2)?, self.src(at, 3)?);
                let cc = self.compare(cond, w, a, b);
                // `mov` does not touch the flags, so the two candidates can be
                // fetched between the compare and the conditional move.
                self.load_temp(Reg::Rax, t);
                let other = self.operand(Reg::Rcx, f);
                self.asm.cmovcc(invert(cc), Reg::Rax, other);
            }
            (_, 3) => {
                let sel = self.src(at, 0)?;
                let (t, f) = (self.src(at, 1)?, self.src(at, 2)?);
                let bit = self.operand(Reg::Rdx, sel);
                self.asm.test_ri32(bit, 1);
                self.load_temp(Reg::Rax, t);
                let other = self.operand(Reg::Rcx, f);
                // Zero means the selector's low bit was clear, so take `f`.
                self.asm.cmovcc(Cc::E, Reg::Rax, other);
            }
            _ => {
                return Err(Refusal::Shape(
                    "a movcond takes a selector and two values, or a condition and four",
                ));
            }
        }
        self.write(inst, Reg::Rax)
    }

    fn brcond(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let target = inst.aux as usize;
        // Forward only, and inside the block. `Liveness` is a single backward
        // walk that is exact for forward control flow and silently wrong for a
        // loop, the verifier enforces it, and a compiled backward branch would
        // additionally have no step limit to stop it (`ir::Interp`'s does).
        if target <= at || target >= self.block.insts().len() {
            return Err(Refusal::Shape(
                "a brcond branches forward, inside the block",
            ));
        }
        let cc = match (inst.cond, self.block.srcs(at).len()) {
            (Some(cond), 2) => {
                let (a, b) = (self.src(at, 0)?, self.src(at, 1)?);
                self.compare(cond, w, a, b)
            }
            (_, 1) => {
                let sel = self.src(at, 0)?;
                let bit = self.operand(Reg::Rax, sel);
                self.asm.test_ri32(bit, 1);
                Cc::Ne
            }
            _ => {
                return Err(Refusal::Shape(
                    "a brcond takes a selector, or a condition and two values",
                ));
            }
        };
        let f = self.asm.jcc(cc);
        self.branches.push((f, target));
        Ok(())
    }

    fn shift(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let a = self.src(at, 0)?;
        let b = self.src(at, 1)?;
        self.load_temp(Reg::Rax, a);
        self.load_temp(Reg::Rcx, b);
        let arithmetic = inst.op == Opcode::SAR;
        if arithmetic {
            // The value is held zero-extended, so an arithmetic shift has to
            // see the sign bit where the host expects it.
            self.sext(Reg::Rax, w);
        }
        // Out of range is undefined in the IR and the oracle takes the
        // mathematical answer; see the module docs for why the free
        // mask-the-count behaviour is not used.
        self.asm.alu_ri(Alu::Cmp, Reg::Rcx, w as i32);
        let out_of_range = self.asm.jcc(Cc::Ae);
        let sh = match inst.op {
            Opcode::SHL => Shift::Shl,
            Opcode::SHR => Shift::Shr,
            _ => Shift::Sar,
        };
        self.asm.shift_rcl(sh, Reg::Rax);
        let done = self.asm.jmp();
        self.asm.bind(out_of_range);
        if arithmetic {
            self.asm.shift_ri(Shift::Sar, Reg::Rax, 63);
        } else {
            self.asm.mov_ri(Reg::Rax, 0);
        }
        self.asm.bind(done);
        self.write(inst, Reg::Rax)
    }

    fn rotate_through_carry(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let a = self.src(at, 0)?;
        let carry_in = self.src(at, 1)?;
        let carry_out = inst
            .dst2
            .ok_or(Refusal::Shape("a carry op must produce its carry out"))?;
        let carry_ty = self
            .block
            .type_of(carry_out)
            .ok_or(Refusal::Shape("the carry out was never allocated"))?;
        self.load_temp(Reg::Rax, a);
        self.load_temp(Reg::Rcx, carry_in);
        self.asm.alu_ri(Alu::And, Reg::Rcx, 1);
        // A third scratch register, because the value, the carry in and the
        // carry out are all live at once. It used to be `r13`, which the
        // allocator now hands out.
        self.asm.mov_rr(Reg::Rsi, Reg::Rax);
        if inst.op == Opcode::ROTLC {
            self.asm.shift_ri(Shift::Shl, Reg::Rax, 1);
            self.asm.alu_rr(Alu::Or, Reg::Rax, Reg::Rcx);
            self.asm.shift_ri(Shift::Shr, Reg::Rsi, (w - 1) as u8);
        } else {
            self.asm.shift_ri(Shift::Shr, Reg::Rax, 1);
            if w > 1 {
                self.asm.shift_ri(Shift::Shl, Reg::Rcx, (w - 1) as u8);
            }
            self.asm.alu_rr(Alu::Or, Reg::Rax, Reg::Rcx);
        }
        self.asm.alu_ri(Alu::And, Reg::Rsi, 1);
        self.write(inst, Reg::Rax)?;
        self.mask(Reg::Rsi, carry_ty);
        self.commit(carry_out, Reg::Rsi);
        Ok(())
    }

    fn count_zeros(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let a = self.src_typed(at, 0, inst.ty)?;
        self.load_temp(Reg::Rax, a);
        // The width is the answer for a zero input, in both directions:
        // `CLZ` counts within the type and `CTZ` saturates at it.
        self.asm.mov_ri(Reg::Rdx, u64::from(w));
        self.asm.test_rr(Reg::Rax, Reg::Rax);
        let zero = self.asm.jcc(Cc::E);
        if inst.op == Opcode::CLZ {
            // `bsr` gives the index of the highest set bit, so the count of
            // leading zeros within `w` bits is `w - 1 - index`.
            self.asm.bsr(Reg::Rcx, Reg::Rax);
            self.asm.mov_ri(Reg::Rdx, u64::from(w - 1));
            self.asm.alu_rr(Alu::Sub, Reg::Rdx, Reg::Rcx);
        } else {
            self.asm.bsf(Reg::Rdx, Reg::Rax);
        }
        self.asm.bind(zero);
        self.asm.mov_rr(Reg::Rax, Reg::Rdx);
        self.write(inst, Reg::Rax)
    }

    /// Population count of `rax`, in `rax`.
    ///
    /// The classic SWAR sequence — pairs, nibbles, bytes, then one multiply to
    /// sum the bytes — rather than `POPCNT`, which is an extension a host may
    /// not have. The input is already masked to its type, so counting the
    /// whole 64-bit register counts exactly the type's bits.
    fn popcount(&mut self) {
        self.asm.mov_rr(Reg::Rcx, Reg::Rax);
        self.asm.shift_ri(Shift::Shr, Reg::Rcx, 1);
        self.asm.mov_ri(Reg::Rdx, 0x5555_5555_5555_5555);
        self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rdx);
        self.asm.alu_rr(Alu::Sub, Reg::Rax, Reg::Rcx);

        self.asm.mov_ri(Reg::Rdx, 0x3333_3333_3333_3333);
        self.asm.mov_rr(Reg::Rcx, Reg::Rax);
        self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rdx);
        self.asm.shift_ri(Shift::Shr, Reg::Rax, 2);
        self.asm.alu_rr(Alu::And, Reg::Rax, Reg::Rdx);
        self.asm.alu_rr(Alu::Add, Reg::Rax, Reg::Rcx);

        self.asm.mov_rr(Reg::Rcx, Reg::Rax);
        self.asm.shift_ri(Shift::Shr, Reg::Rcx, 4);
        self.asm.alu_rr(Alu::Add, Reg::Rax, Reg::Rcx);
        self.asm.mov_ri(Reg::Rdx, 0x0f0f_0f0f_0f0f_0f0f);
        self.asm.alu_rr(Alu::And, Reg::Rax, Reg::Rdx);

        self.asm.mov_ri(Reg::Rdx, 0x0101_0101_0101_0101);
        self.asm.imul_rr(Reg::Rax, Reg::Rdx);
        self.asm.shift_ri(Shift::Shr, Reg::Rax, 56);
    }

    fn widening_multiply(&mut self, at: usize, inst: &Inst, w: u32) -> Result<(), Refusal> {
        let a = self.src_typed(at, 0, inst.ty)?;
        let b = self.src_typed(at, 1, inst.ty)?;
        let high = inst
            .dst2
            .ok_or(Refusal::Shape("a widening multiply produces its high half"))?;
        let high_ty = self
            .block
            .type_of(high)
            .ok_or(Refusal::Shape("the high half was never allocated"))?;
        let signed = inst.op == Opcode::MULS2;
        self.load_temp(Reg::Rax, a);
        self.load_temp(Reg::Rcx, b);
        if w == 64 {
            // The one-operand form: `rdx:rax` is the whole product.
            if signed {
                self.asm.imul1(Reg::Rcx);
            } else {
                self.asm.mul(Reg::Rcx);
            }
            self.asm.mov_rr(Reg::Rsi, Reg::Rdx);
        } else {
            // Below 64 bits the whole product fits in one register, so the
            // high half is a shift. A signed product needs both operands
            // sign-extended first; the result's bits `w..2w` are then the same
            // whether the product is read as 64 or 128 bits wide.
            if signed {
                self.sext(Reg::Rax, w);
                self.sext(Reg::Rcx, w);
            }
            self.asm.imul_rr(Reg::Rax, Reg::Rcx);
            self.asm.mov_rr(Reg::Rsi, Reg::Rax);
            self.asm.shift_ri(Shift::Shr, Reg::Rsi, w as u8);
        }
        self.write(inst, Reg::Rax)?;
        // The interpreter masks the high half twice as well: once to the
        // instruction's width, where it takes it out of the double-width
        // product, and once to the temporary it lands in.
        self.mask(Reg::Rsi, inst.ty);
        self.mask(Reg::Rsi, high_ty);
        self.commit(high, Reg::Rsi);
        Ok(())
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
    /// case where the fast path would merely be slower:
    ///
    /// * A **segmented** access is translated before it reaches the TLB (x86's
    ///   `mem_load` adds the segment base and checks the limit), and that
    ///   translation is the frontend's, not this backend's.
    /// * A **separate I/O space** is not what the TLB fronts.
    /// * A width that is not a whole power of two up to eight has no single
    ///   host access.
    ///
    /// A store's own extra conditions are not here, because none of them is a
    /// property of the [`MemOp`]: they are properties of the *entry*, and they
    /// were settled when it was filled. See [`Compiler::store`].
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
    /// `ROADMAP.md` §9.1's first mechanism: mask, compare, add. On entry
    /// `Rax` holds the guest address; on the fall-through `Rdx` holds the host
    /// address of that guest byte and `Rax` still holds the guest address.
    /// Every way of not being a hit — no plan, a misaligned address, the wrong
    /// page, the wrong world, a page with no host address — lands in the
    /// returned fixups, which the caller binds to its slow path.
    ///
    /// `base`, `mask` and `tag` name the [`Ctx`](super::rt::Ctx) fields of the
    /// set to probe, which is what makes one sequence serve two sets: a load
    /// reads the set admitted on read permission and a store the one admitted
    /// on write permission, and nothing else about the probe differs.
    fn probe(&mut self, base: i32, mask: i32, tag_bits: i32, bytes: u64) -> Vec<Fixup> {
        let mut slow: Vec<Fixup> = Vec::new();
        // The set, which used to sit in `r15` for the whole block and is now
        // one of the three registers the allocator has to work with.
        self.asm.mov_rm(Reg::Rsi, Reg::Rbx, base);
        self.asm.test_rr(Reg::Rsi, Reg::Rsi);
        slow.push(self.asm.jcc(Cc::E));
        if bytes > 1 {
            // Natural alignment. It is also what makes the page-crossing check
            // unnecessary: an aligned access of at most eight bytes cannot
            // span two 4 KiB pages.
            self.asm
                .test_ri32(Reg::Rax, i32::try_from(bytes - 1).unwrap_or(7));
            slow.push(self.asm.jcc(Cc::Ne));
        }
        // index = (addr >> 12) & mask, scaled by the entry stride
        self.asm.mov_rr(Reg::Rcx, Reg::Rax);
        self.asm.shift_ri(Shift::Shr, Reg::Rcx, 12);
        self.asm.mov_rm(Reg::Rdx, Reg::Rbx, mask);
        self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rdx);
        self.asm
            .shift_ri(Shift::Shl, Reg::Rcx, FastSet::STRIDE.trailing_zeros() as u8);
        self.asm.alu_rr(Alu::Add, Reg::Rcx, Reg::Rsi);
        // tag = (addr & !PAGE_MASK) | context | valid
        self.asm.mov_rr(Reg::Rdx, Reg::Rax);
        self.asm.mov_ri(Reg::Rsi, !PAGE_MASK);
        self.asm.alu_rr(Alu::And, Reg::Rdx, Reg::Rsi);
        self.asm.mov_rm(Reg::Rsi, Reg::Rbx, tag_bits);
        self.asm.alu_rr(Alu::Or, Reg::Rdx, Reg::Rsi);
        let tag = i32::try_from(FastSet::TAG).unwrap_or(0);
        self.asm.alu_rm(Alu::Cmp, Reg::Rdx, Reg::Rcx, tag);
        slow.push(self.asm.jcc(Cc::Ne));
        // The host addend, zero when this page has no inline path.
        let host = i32::try_from(FastSet::HOST).unwrap_or(8);
        self.asm.mov_rm(Reg::Rdx, Reg::Rcx, host);
        self.asm.test_rr(Reg::Rdx, Reg::Rdx);
        slow.push(self.asm.jcc(Cc::E));
        self.asm.alu_rr(Alu::Add, Reg::Rdx, Reg::Rax);
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
            self.asm.mov_mi(Reg::Rbx, off::COMMITTED, 1);
        }

        let mut slow: Vec<Fixup> = Vec::new();
        let mut joined: Option<Fixup> = None;
        if Self::inlinable(&mem) {
            // `ROADMAP.md` §9.1's first mechanism, inlined: mask, compare,
            // add, load. Everything that is not a hit on plain little-endian
            // RAM branches out to the host's own path, which is the one that
            // fills the entry.
            self.load_temp(Reg::Rax, addr);
            slow = self.probe(off::TLB_BASE, off::TLB_MASK, off::TAG_BITS, bytes);
            self.asm.load_zx(Reg::Rax, Reg::Rdx, 0, bytes);
            // Park the value across the call rather than in a callee-saved
            // register, which the allocator now owns; see `PARKED`.
            self.asm.mov_mr(Reg::Rsp, PARKED, Reg::Rax);
            // The tick the host's own path would have charged for this access.
            self.ctx_to_rdi();
            self.call(vt::FAST_TICK);
            self.bump(off::FAST_HITS);
            self.asm.mov_rm(Reg::Rax, Reg::Rsp, PARKED);
            joined = Some(self.asm.jmp());
        }

        for f in slow {
            self.asm.bind(f);
        }
        self.ctx_to_rdi();
        self.asm.mov_ri(Reg::Rsi, descriptor);
        self.load_temp(Reg::Rdx, addr);
        // The out-parameter: the eight bytes of frame the prologue reserved.
        self.asm.mov_rr(Reg::Rcx, Reg::Rsp);
        self.call(vt::LOAD);
        self.asm.test_rr(Reg::Rax, Reg::Rax);
        let ok = self.asm.jcc(Cc::E);
        self.fault(at)?;
        self.asm.bind(ok);
        self.asm.mov_rm(Reg::Rax, Reg::Rsp, 0);

        if let Some(f) = joined {
            self.asm.bind(f);
        }
        if mem.sign == Sign::Signed {
            self.sext(Reg::Rax, mem.size.bits());
        }
        self.write(inst, Reg::Rax)
    }

    /// A store: the same inlined probe a load uses, over the store set, plus
    /// one call that pays what moving the bytes did not.
    ///
    /// A store is harder than a load and the differences are worth naming,
    /// because every one of them is silent when it is wrong. Three of the four
    /// are settled at *fill* time, in the host, which is why none of them
    /// appears here:
    ///
    /// * **Write permission is a different bit from read permission**, so the
    ///   store set is a different table — [`off::ST_BASE`], not
    ///   [`off::TLB_BASE`] — and an entry in it was admitted on
    ///   [`Perms::WRITE`](crate::core::space::Perms::WRITE) and on the guest
    ///   MMU's own write permission.
    /// * **The architecture's first-write bookkeeping** — RISC-V's PTE dirty
    ///   bit — was done by the walk that filled the entry, because a store
    ///   entry only exists because a walk *for a store* succeeded.
    /// * **A protection check that may differ within one page** (PMP) was asked
    ///   as `pmp_page_uniform` rather than `pmp_allows` before the page was
    ///   admitted at all.
    ///
    /// The fourth cannot be settled in advance, and it is
    /// [`vt::FAST_STORE`]: the [`RamStore`](crate::core::space::RamStore)'s own
    /// dirty bitmap, which a host pointer does not mark and which is the only
    /// record a framebuffer refresh or a live snapshot has; the guest-physical
    /// dirty log the dispatcher drains to invalidate translations of a page
    /// the guest has just rewritten (`ROADMAP.md` §9.1's third mechanism, and
    /// Linux really does write its own code, in module loading and in
    /// alternatives patching); and whatever else the core owes a store, such as
    /// RISC-V's reservation. One call, after the bytes have landed, which is
    /// unobservable — nothing runs in between and the fast path cannot fault.
    ///
    /// So `RamStore::host_ptr` stays read-only *as a Rust API*, and what
    /// changed is not that rule but that there is now a place where the two
    /// bits it exists to protect are paid. A backend that wrote through it and
    /// did not call this would be exactly the bug that documentation warns
    /// about.
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
        self.asm.mov_mi(Reg::Rbx, off::COMMITTED, 1);

        let mut slow: Vec<Fixup> = Vec::new();
        let mut joined: Option<Fixup> = None;
        if Self::inlinable(&mem) {
            self.load_temp(Reg::Rax, addr);
            slow = self.probe(off::ST_BASE, off::ST_MASK, off::ST_TAG, bytes);
            // The guest address is the thunk's first argument and `rax` is
            // about to hold the value, so it goes to the stack slot the
            // prologue reserved rather than to a callee-saved register the
            // allocator now owns; see `PARKED`.
            self.asm.mov_mr(Reg::Rsp, PARKED, Reg::Rax);
            self.load_temp(Reg::Rax, value);
            // The bytes, and only the bytes: `store_trunc` writes the width the
            // guest asked for, so a neighbouring guest byte another device is
            // reading is never disturbed — and the high bits of the value are
            // discarded by the instruction rather than by a mask ahead of it.
            //
            // The slow path *does* mask, and the asymmetry is not an oversight:
            // there the value is handed to
            // [`IrHost::store`](crate::ir::IrHost::store), whose contract says
            // it arrives already truncated to the access width, and a host is
            // entitled to rely on that. Here the only consumer is the `mov`.
            self.asm.store_trunc(Reg::Rdx, 0, Reg::Rax, bytes);
            self.ctx_to_rdi();
            self.asm.mov_rm(Reg::Rsi, Reg::Rsp, PARKED);
            self.asm.mov_ri(Reg::Rdx, bytes);
            self.call(vt::FAST_STORE);
            self.bump(off::FAST_WRITES);
            joined = Some(self.asm.jmp());
        }

        for f in slow {
            self.asm.bind(f);
        }
        self.ctx_to_rdi();
        self.asm.mov_ri(Reg::Rsi, descriptor);
        self.load_temp(Reg::Rdx, addr);
        self.load_temp(Reg::Rcx, value);
        if mask != u64::MAX {
            // `rax` rather than a callee-saved register: it is not an argument
            // register, this sequence makes no call before the `and`, and
            // `r13` now belongs to the allocator.
            self.asm.mov_ri(Reg::Rax, mask);
            self.asm.alu_rr(Alu::And, Reg::Rcx, Reg::Rax);
        }
        self.call(vt::STORE);
        self.asm.test_rr(Reg::Rax, Reg::Rax);
        let ok = self.asm.jcc(Cc::E);
        self.fault(at)?;
        self.asm.bind(ok);
        if let Some(f) = joined {
            self.asm.bind(f);
        }
        Ok(())
    }
}

/// The condition code a comparison branches on.
const fn cc_of(cond: Cond) -> Cc {
    match cond {
        Cond::Eq => Cc::E,
        Cond::Ne => Cc::Ne,
        Cond::LtS => Cc::L,
        Cond::LeS => Cc::Le,
        Cond::GtS => Cc::G,
        Cond::GeS => Cc::Ge,
        Cond::LtU => Cc::B,
        Cond::LeU => Cc::Be,
        Cond::GtU => Cc::A,
        Cond::GeU => Cc::Ae,
    }
}

/// The condition that holds exactly when `cc` does not.
const fn invert(cc: Cc) -> Cc {
    match cc {
        Cc::E => Cc::Ne,
        Cc::Ne => Cc::E,
        Cc::L => Cc::Ge,
        Cc::Ge => Cc::L,
        Cc::Le => Cc::G,
        Cc::G => Cc::Le,
        Cc::B => Cc::Ae,
        Cc::Ae => Cc::B,
        Cc::Be => Cc::A,
        Cc::A => Cc::Be,
    }
}

/// The mask of a `len`-bit field at `pos`, or `None` if it leaves the type.
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
        // `i128` needs a register pair and the widening multiplies produce
        // their high half separately, so nothing in the tree asks for one;
        // the float types exist to be carried into a helper call, and helper
        // calls are refused.
        other => Err(Refusal::Type(other)),
    }
}
