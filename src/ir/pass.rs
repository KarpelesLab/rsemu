//! Liveness, and the dead-code elimination it pays for.
//!
//! The module docs' first decision — flags are ordinary temporaries — states
//! its own cost plainly: *"this design is strictly worse than eager packing
//! until liveness and DCE exist."* This is that pass pair, and the debt it
//! settles is concrete. x86 computes `PF` as a popcount on nearly every ALU
//! instruction and reads it almost never; the Z80 computes `P/V` and the
//! undocumented `XF`/`YF` on nearly every one and reads them less often still;
//! ARM's cores already return `(result, n, z, c, v)` from every ALU helper and
//! let the caller decide whether to commit. Lifted literally, all of those
//! become temporaries nothing consumes, and every one of them would be real
//! host instructions in the translation. Removing them is what makes the
//! flags-as-temporaries decision cheaper than eager packing rather than dearer.
//!
//! # What may be removed, and what may not
//!
//! The pass is conservative in four separate ways, and every one of them is a
//! defect that would not show up as a crash:
//!
//! * **[`Opcode::has_side_effect`] is absolute.** Stores, atomics, fences,
//!   helper calls, [`Opcode::CHARGE`] and [`Opcode::INSN_START`] stay whatever
//!   the liveness says. A charge is a *hashed output* rather than a budget
//!   (module docs, decision 2), so an eliminated one is a state-hash mismatch
//!   against the interpreter, not a lost optimisation.
//! * **A load is eliminable only when [`MemOp::volatile`] is false.** That flag
//!   exists for the reads whose value is discarded but whose bus cycle is not:
//!   the 6502's dummy reads — the internal cycle `PLA`, `RTS`, `RTI` and `JSR`
//!   spend — and its index fix-up read, which on the NMOS part lands on the
//!   *unfixed* address, which is why `STA $20ff,X` touches `$2000`-page
//!   hardware. Eliminating one changes what the guest's hardware sees, which
//!   no amount of "nothing reads the value" makes safe.
//! * **A temporary named in any [`InsnStart::live`] mapping is live by
//!   definition.** That mapping is what a mid-block fault reconstructs
//!   architectural state from, and nothing in the block *consumes* it — it is
//!   read by the exception path, from outside the IR. Treating "no consumer"
//!   as "dead" here deletes the value a fault was going to report, and every
//!   test that does not fault still passes.
//! * **An instruction with no result is never removed.** [`Opcode::BRCOND`]
//!   has no destination and no side-effect flag, because its effect *is* the
//!   branch; a rule phrased as "no `dst` means dead" would quietly delete
//!   control flow. So elimination requires at least one result and every
//!   result dead — which is also the exact wording of the rule: remove what
//!   nothing consumes, and only that.
//!
//! # Why one backward pass is enough
//!
//! Textbook liveness is a fixpoint over a control-flow graph, and the interval
//! form below is the one a linear-scan allocator wants (Poletto and Sarkar,
//! *Linear scan register allocation*, ACM TOPLAS 21(5), 1999). A translation
//! block needs neither the graph nor the fixpoint: it is SSA in a single
//! linear order, every definition precedes its uses, and the only control flow
//! inside it is forward — a [`Opcode::BRCOND`] over instructions ahead of it.
//! Walking the instructions in reverse and unioning every later use therefore
//! computes a *superset* of the true live set at each point, which is the safe
//! direction: it keeps too much, never too little. A block with a backward
//! branch in it would need the fixpoint, and no frontend can build one today —
//! the IR has no label-defining op.

use crate::ir::block::{Block, InsnStart, RegSlot};
use crate::ir::op::{MemOp, Opcode};
use crate::ir::types::Temp;
use alloc::vec;
use alloc::vec::Vec;

/// What a block does with one temporary.
///
/// Keyed by [`Temp`] number in a flat [`Vec`] rather than held in a map: a
/// block's temporaries are `0..temp_count()` with no holes, so an index-keyed
/// vector is both the fastest shape and the only *deterministic* one
/// (CLAUDE.md, "Determinism" — no hashed iteration order in anything that
/// decides guest-visible state, and register assignment decides it).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TempLife {
    /// The index of the instruction that assigns this temporary.
    ///
    /// `None` for one that was allocated and never assigned — which
    /// [`BlockBuilder::temp`](crate::ir::BlockBuilder::temp) permits, and
    /// which dead-code elimination leaves behind wherever it removed a
    /// definition.
    pub def: Option<u32>,
    /// The index of the last instruction that reads it, if any.
    ///
    /// A boundary marker counts as a read: see [`TempLife::at_boundary`].
    pub last_use: Option<u32>,
    /// How many operand slots across the block name it.
    ///
    /// A count rather than a flag because a spill or rematerialization
    /// heuristic wants the number, and it is free to keep here.
    pub uses: u32,
    /// Whether an [`InsnStart`] names it as live guest state.
    ///
    /// Liveness for the exception path rather than for a consumer inside the
    /// block, and the reason this is its own field: `uses == 0` with this set
    /// is exactly the value dead-code elimination must not touch.
    pub at_boundary: bool,
}

impl TempLife {
    /// Whether anything at all needs this temporary's value.
    #[inline]
    #[must_use]
    pub const fn is_live(self) -> bool {
        self.uses > 0 || self.at_boundary
    }

    /// The instruction range this temporary occupies, as `(def, last_use)`,
    /// both ends inclusive.
    ///
    /// `None` when it was never assigned. A temporary with a definition and no
    /// reader yields `(def, def)` rather than nothing: it is still assigned,
    /// and an allocator handed a block that has not been through dead-code
    /// elimination still owes it a register for that one instruction.
    #[must_use]
    pub fn interval(self) -> Option<(u32, u32)> {
        let def = self.def?;
        Some((def, self.last_use.unwrap_or(def).max(def)))
    }
}

/// Liveness for every temporary in a block.
///
/// Syntactic: it counts the operand slots that name a temporary, whether or
/// not the instruction holding that slot is itself dead. That is deliberate —
/// it describes the block *as it stands*, which is what a register allocator
/// needs. Run [`eliminate_dead_code`] first and the two agree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Liveness {
    lives: Vec<TempLife>,
}

impl Liveness {
    /// Compute liveness for `block`.
    #[must_use]
    pub fn compute(block: &Block) -> Liveness {
        let mut lives = vec![TempLife::default(); block.temp_count()];

        for (i, inst) in block.insts().iter().enumerate() {
            let at = i as u32;

            // Reads before writes, because an instruction reads its operands
            // before it assigns its results. An SSA block cannot name the same
            // temporary on both sides anyway; the order is for the reader.
            for src in block.srcs(i) {
                if let Some(life) = lives.get_mut(src.index()) {
                    life.uses += 1;
                    life.last_use = Some(at);
                }
            }
            for dst in [inst.dst, inst.dst2].into_iter().flatten() {
                if let Some(life) = lives.get_mut(dst.index())
                    && life.def.is_none()
                {
                    life.def = Some(at);
                }
            }

            // A boundary extends a temporary's range to the marker even though
            // no operand slot names it, because the exception path
            // materializes architectural state from exactly there.
            if inst.op == Opcode::INSN_START
                && let Some(mark) = block.marks().get(inst.aux as usize)
            {
                for (_, temp) in &mark.live {
                    if let Some(life) = lives.get_mut(temp.index()) {
                        life.at_boundary = true;
                        life.last_use = Some(match life.last_use {
                            Some(prev) => prev.max(at),
                            None => at,
                        });
                    }
                }
            }
        }

        Liveness { lives }
    }

    /// What the block does with `temp`, or `None` if it was never allocated.
    #[inline]
    #[must_use]
    pub fn life(&self, temp: Temp) -> Option<TempLife> {
        self.lives.get(temp.index()).copied()
    }

    /// Whether anything needs `temp`'s value. False for an unknown temporary.
    #[inline]
    #[must_use]
    pub fn is_live(&self, temp: Temp) -> bool {
        self.life(temp).is_some_and(TempLife::is_live)
    }

    /// Every temporary, in numbering order, with what the block does with it.
    pub fn iter(&self) -> impl Iterator<Item = (Temp, TempLife)> + '_ {
        self.lives
            .iter()
            .enumerate()
            .map(|(i, life)| (Temp(i as u32), *life))
    }

    /// Every assigned temporary's live interval, ordered by start point.
    ///
    /// The input a linear-scan allocator takes: intervals sorted by increasing
    /// start. Ties break on temporary number, so the order is total and the
    /// allocation that follows from it is reproducible — the same block must
    /// always translate to the same host code, or a state hash stops being an
    /// identity.
    #[must_use]
    pub fn intervals(&self) -> Vec<(Temp, u32, u32)> {
        let mut out: Vec<(Temp, u32, u32)> = self
            .iter()
            .filter_map(|(temp, life)| life.interval().map(|(lo, hi)| (temp, lo, hi)))
            .collect();
        // Nearly sorted already — definitions come in instruction order — but
        // only nearly, because a second result is allocated before the first
        // on the ops that have one. Sort rather than assume.
        out.sort_by_key(|(temp, lo, _)| (*lo, *temp));
        out
    }
}

/// Whether an instruction's *effect* obliges it to stay, whatever consumes it.
///
/// Split out from [`eliminate_dead_code`] because it is the rule that has to
/// be right, and a rule that reads as one expression can be checked by eye.
fn must_keep(op: Opcode, mem: Option<MemOp>, has_result: bool) -> bool {
    // A terminator is the block's exit and a result-free instruction is all
    // effect (BRCOND); neither is expressible as a dead value.
    !has_result
        || op.is_terminator()
        || op.has_side_effect()
        // Only a load reaches here carrying a descriptor — a store is already
        // a side effect — and a volatile one is a bus cycle the guest's
        // hardware observes even though the value is thrown away.
        || mem.is_some_and(|m| m.volatile)
}

/// Remove the instructions whose results nothing consumes.
///
/// Returns a new block; the input is untouched. Temporary numbering and the
/// boundary records survive unchanged, so a [`Temp`] taken from the input
/// still names the same value in the output, and the result passes
/// [`verify`](crate::ir::verify) whenever the input did.
///
/// One backward walk suffices, chains included — the popcount feeding the mask
/// feeding the parity flag all go together — because a definition always
/// precedes its uses in the block's linear order, so by the time the walk
/// reaches an instruction it has already decided every instruction that could
/// have consumed it. See the module docs for what is never removed.
#[must_use]
pub fn eliminate_dead_code(block: &Block) -> Block {
    let insts = block.insts();
    let mut needed = vec![false; block.temp_count()];

    // Seed: every temporary any boundary names is live before the walk starts.
    // Seeded rather than discovered, because the consumer is the exception
    // path and it is not an instruction in this block.
    for mark in block.marks() {
        seed_boundary(mark, &mut needed);
    }

    let mut keep = vec![false; insts.len()];
    for (i, inst) in insts.iter().enumerate().rev() {
        let results = [inst.dst, inst.dst2];
        let has_result = results.iter().any(Option::is_some);
        // A destination outside the type table cannot be looked up; a block
        // holding one is malformed and the verifier says so by name. Call it
        // live, so a pass run ahead of the verifier — the fuzz target does
        // exactly that — deletes nothing on the strength of a number it could
        // not resolve.
        let result_live = results
            .into_iter()
            .flatten()
            .any(|t| needed.get(t.index()).copied().unwrap_or(true));

        if !must_keep(inst.op, inst.mem, has_result) && !result_live {
            continue;
        }
        keep[i] = true;
        for src in block.srcs(i) {
            if let Some(slot) = needed.get_mut(src.index()) {
                *slot = true;
            }
        }
    }

    block.retain(&keep)
}

/// Mark every temporary a boundary names as needed.
fn seed_boundary(mark: &InsnStart, needed: &mut [bool]) {
    for (_, temp) in &mark.live {
        if let Some(slot) = needed.get_mut(temp.index()) {
            *slot = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Hoisting slot reads
// ---------------------------------------------------------------------------

/// Move every [`Opcode::GET_SLOT`] as early in the block as it may legally go.
///
/// # What this is for, and the measurement that asked for it
///
/// A backend that defers [`Opcode::CHARGE`] and [`Opcode::INSN_START`] — which
/// is what `jit::x86` does, and what the portable interpreter's own laziness
/// amounts to — has to **replay** the deferred bookkeeping before anything the
/// host can observe. A slot read is one of those things: `IrHost::read_slot`
/// is a call into the guest's register file, so every charge and every
/// boundary since the last replay has to have happened first.
///
/// The frontends in this tree emit, per guest instruction, an `insn_start`,
/// then a `charge`, then a `get_slot` for each guest register the instruction
/// reads for the first time in this block, then the arithmetic. That ordering
/// puts a replay point one or two instructions after every boundary, so the
/// deferral buys almost nothing: the replay happens per *guest instruction*
/// rather than per region, which is the cost it exists to avoid.
///
/// Measured on `machines/arm64-virt.machine` booting Linux 6.12.94 `arm64`
/// under `engine = "jit-host"`, twenty guest seconds, host instructions by
/// callgrind: **4.64 replays per block against 3.38 slot reads per block**, on
/// blocks that retire 6.44 guest instructions each. The replay was 19.3% of
/// the whole profile and the slot-read thunk another 7.8%; the code the JIT
/// generated was 7.7%. `benches/a64_linux_boot.rs` is where those numbers come
/// from, and its docs say how to take them again.
///
/// A `get_slot` moved above its own instruction's boundary joins the region
/// its predecessors are already in, and a run of them collapses to one replay
/// point — or to none, where the whole run reaches the top of the block and
/// there is nothing before it to replay.
///
/// # What it bought, and the half of it that was not predicted
///
/// The same twenty seconds, the same binary but for this pass, the same
/// 154 233 793 guest instructions retired in the same 23 935 619 blocks and
/// the same state hash — so the two columns are the same guest work, and every
/// number below is host cost alone.
///
/// | | before | after |
/// | --- | --- | --- |
/// | **host instructions** | **53 930 067 136** | **51 375 584 369** (−4.74%) |
/// | replays (`flush_thunk`) | 111 176 345 calls, 10.42 G | 73 205 074 calls, 9.89 G |
/// | slot reads (`get_slot_thunk`) | 80 988 884 calls, 4.21 G | 81 018 450 calls, **1.90 G** |
/// | the generated code itself | 4.13 G | 3.89 G |
/// | this pass | — | 0.33 G |
/// | `Cpu::advance` | 12 646 625 187 | 12 646 625 221 |
///
/// **The replay is the smaller half of the saving.** A third of the calls went
/// away and only 0.53 G with them, because what a replay costs is mostly the
/// events in it and hoisting moves events between regions rather than removing
/// them; the calls it saves are worth about 14 host instructions each.
///
/// **The slot read is the larger half, and it is not what was aimed at.** Its
/// thunk got 55% cheaper *per call* — 52.0 host instructions to 23.4 — because
/// a read has to ask whether the pending boundary's mapping binds the slot it
/// is about to read, and a read hoisted above every boundary in its region
/// finds no pending boundary and answers in three instructions. That is 2.31 G
/// of the 2.55 G. It is recorded here because the reasoning that produced this
/// pass would not have predicted it, and the next person estimating a change
/// like this one should know that the estimate was right about the direction
/// and wrong about the mechanism.
///
/// The extra 29 566 slot reads are the cost side, and they are the reason
/// rule 2 exists: a read hoisted above a side exit runs on a path that was
/// leaving the block. Bounding it to *within* the region — never above a
/// `brcond` — held it to 0.04% of the reads rather than to whatever a
/// superblock's exit rate happens to be.
///
/// # What it is worth on the other two frontends
///
/// The pass is frontend-agnostic and all three of them call it. It landed on
/// A64 alone, and the other two were **measured separately rather than assumed
/// to follow** — the frontends differ in how many slot reads they emit and
/// where, and it shows:
///
/// | frontend | workload | host instructions | | insns/block then |
/// | --- | --- | --- | --- | --- |
/// | `cpu::arm::a64::lift` | `arm64-virt`, a Debian arm64 `Image` | 53 930 067 136 → 51 375 584 369 | **−4.74%** | 6.44 |
/// | `cpu::x86::lift` | `pc64`, a stock `bzImage` | 28 567 216 644 → 27 832 228 449 | **−2.57%** | 4.79 |
/// | `cpu::riscv::lift` | `riscv-virt`, OpenSBI `fw_jump` | 66 624 299 149 → 66 138 469 427 | **−0.73%** | 3.18 |
///
/// Twenty guest seconds each, `engine = "jit-host"`, under callgrind with
/// `--cache-sim=no`. The first row is `benches/a64_linux_boot.rs` and the
/// other two are `rsemu run … --for 20s --trace cpu`, which prints the block
/// count the last column is derived from. Every pair reached the same
/// `Machine::state_hash`, so the two host-instruction columns are the same
/// guest work and the difference is host cost alone.
///
/// **The spread is block length**, and that is worth writing down because it
/// is what a fourth frontend should estimate from. What this removes is a
/// replay point per *guest instruction*, and how much of that it can remove is
/// bounded by how many boundaries a read is able to move above — which is
/// bounded by how many boundaries a block has at all. The three workloads
/// order by instructions per block and the saving orders with them. A frontend
/// whose blocks are two instructions long has almost nothing here — and a
/// frontend whose blocks get *longer* has more, which is why the last column
/// says what a block was at the time rather than what it is now.
///
/// # Why it is safe, stated as five rules
///
/// A slot read is *pure*: it writes a temporary and touches nothing else. So
/// the only questions are what it reads and when it is reached, and each rule
/// below answers one of them.
///
/// 1. **Never above an instruction a branch targets.** A block is
///    straight-line SSA with forward branches, so an instruction after a
///    branch target is reachable without executing what precedes that target.
///    Moving a definition above one leaves the taken path reading a temporary
///    nothing assigned.
/// 2. **Never above a [`Opcode::BRCOND`] or a terminator.** Not for the value
///    — the value is the same — but for the *count*: a superblock's side exit
///    is an inline sequence the trace branches over, so an instruction moved
///    above one runs on a path that was about to leave the block, and the
///    saved replay is paid back as a call the guest did not need.
/// 3. **Never above a [`Opcode::CALL_HELPER`].** A helper is arbitrary Rust
///    that may write the guest's registers (`ir`'s decision 4), so the slot's
///    value is not the same on both sides of one.
/// 4. **Never at all when an earlier boundary binds the same slot.** Reading a
///    slot some boundary's [`InsnStart::live`] mapping names is what makes a
///    backend *publish*, and a publish at an earlier boundary writes an
///    earlier mapping. That is a different sequence of writes to guest state
///    even where it ends in the same place, so the read stays where the
///    frontend put it. A frontend that keeps the mapping invariant never emits
///    such a read anyway — it has the temporary — which is why this rule costs
///    nothing and is checked rather than assumed.
/// 5. **Everything else is crossable, and crossing it is the point.** A
///    boundary that binds other slots, a charge, and every arithmetic
///    instruction: none of them can be observed by a slot read and none of them
///    reads its result.
///
/// The relative order of two hoisted reads is their original one, and every
/// choice is made from instruction indices, so the same block always produces
/// the same block (`ROADMAP.md` §0).
///
/// Returns a new block; the input is untouched. Temporary numbering and the
/// boundary records survive unchanged, and the result passes
/// [`verify`](crate::ir::verify) whenever the input did.
#[must_use]
pub fn hoist_slot_reads(block: &Block) -> Block {
    let insts = block.insts();
    let n = insts.len();

    // Which instructions a branch can land on. Rule 1.
    let mut targeted = vec![false; n];
    for inst in insts {
        if inst.op == Opcode::BRCOND
            && let Some(slot) = targeted.get_mut(inst.aux as usize)
        {
            *slot = true;
        }
    }

    // The slots some boundary has already bound, in first-bound order. Rule 4.
    // A flat `Vec` rather than a set: a block binds a handful of slots, and a
    // linear scan over a handful beats a hash nobody may iterate anyway
    // (`ROADMAP.md` §0 — no hashed order in anything that decides guest-visible
    // state, and where a slot read happens decides it).
    let mut bound: Vec<RegSlot> = Vec::new();

    // `(where it lands, where it came from)` for every read that moves.
    //
    // One flat vector rather than a bucket per instruction, and it needs no
    // sort: the floor only ever moves forward and the scan only ever moves
    // forward, so the pairs come out already ordered by `(lands, came from)`,
    // which is exactly the order they have to be emitted in. It is also *not*
    // expressible as a sort key over the instructions, because the answer has
    // to be stable under a second run — a read already sitting at its landing
    // point must be left there, and a key that cannot tell "at the floor" from
    // "hoisted to the floor" swaps two of them every time the pass runs again.
    let mut hoists: Vec<(u32, u32)> = Vec::new();
    let mut hoisted = vec![false; n];
    // The lowest index nothing may move above: one past the last barrier.
    let mut floor = 0usize;
    let mut moved = false;

    for (i, inst) in insts.iter().enumerate() {
        if targeted[i]
            || inst.op == Opcode::BRCOND
            || inst.op == Opcode::CALL_HELPER
            || inst.op.is_terminator()
        {
            floor = i + 1;
        }
        if inst.op == Opcode::INSN_START {
            if let Some(mark) = block.marks().get(inst.aux as usize) {
                for (slot, _) in &mark.live {
                    if !bound.contains(slot) {
                        bound.push(*slot);
                    }
                }
            }
            continue;
        }
        if inst.op != Opcode::GET_SLOT {
            continue;
        }
        // `aux` is the slot, and a verified block takes it from nowhere else.
        let slot = RegSlot(inst.aux as u16);
        if targeted[i] || bound.contains(&slot) {
            continue;
        }
        if i <= floor {
            // Already where it would be moved to. Nothing to do, and the next
            // read to arrive belongs *after* it rather than in front of it —
            // which is the whole of what makes a second pass a no-op.
            floor = i + 1;
            continue;
        }
        hoists.push((floor as u32, i as u32));
        hoisted[i] = true;
        moved = true;
    }

    if !moved {
        return block.clone();
    }

    let mut order: Vec<usize> = Vec::with_capacity(n);
    let mut next = 0usize;
    for (i, &moved_here) in hoisted.iter().enumerate() {
        while let Some(&(lands, from)) = hoists.get(next)
            && lands as usize == i
        {
            order.push(from as usize);
            next += 1;
        }
        if !moved_here {
            order.push(i);
        }
    }
    block.reorder(&order)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::value::Width;
    use crate::ir::block::{BlockBuilder, RegSlot};
    use crate::ir::op::Cond;
    use crate::ir::types::{Const, Type};
    use crate::ir::verify;

    fn mark(pc: u64, ticks: u64, live: &[(RegSlot, Temp)]) -> InsnStart {
        InsnStart {
            pc,
            next_pc: pc + 2,
            ticks,
            live: live.to_vec(),
        }
    }

    fn count(block: &Block, op: Opcode) -> usize {
        block.insts().iter().filter(|i| i.op == op).count()
    }

    /// The x86 `PF` / Z80 `P/V` shape: an `XOR` whose *result* is stored, and
    /// a parity flag computed off it by popcount that nothing reads.
    ///
    /// `name_parity_live` is the single difference between the two blocks:
    /// whether the second guest instruction boundary names the parity flag as
    /// live guest state. That is the whole test — the flag is dead in exactly
    /// one of them, and nothing in the IR's *shape* says which.
    fn parity_block(name_parity_live: bool) -> Block {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(4);

        let a = b.imm(Type::I32, Const::Int(0x5a));
        let v = b.imm(Type::I32, Const::Int(0x0f));
        let result = b.binary(Opcode::XOR, Type::I32, a, v);

        // PF: set when the number of one bits is even, which is a popcount and
        // a test of its low bit.
        let ones = b.unary(Opcode::POPCOUNT, Type::I32, result);
        let one = b.imm(Type::I32, Const::Int(1));
        let odd = b.binary(Opcode::AND, Type::I32, ones, one);
        let zero = b.imm(Type::I32, Const::Int(0));
        let parity = b.setcond(Cond::Eq, Type::I32, odd, zero);

        // The result itself is architectural: it goes to memory.
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        b.store(Type::I32, addr, result, MemOp::store(Width::U32));

        let live: Vec<(RegSlot, Temp)> = if name_parity_live {
            vec![(RegSlot(0), parity)]
        } else {
            Vec::new()
        };
        b.insn_start(mark(0x1002, 4, &live));
        b.charge(4);
        b.exit_tb();
        b.finish()
    }

    #[test]
    fn a_parity_flag_nothing_reads_is_removed() {
        let block = parity_block(false);
        verify(&block).expect("the input block is well formed");
        let before = block.insts().len();

        let out = eliminate_dead_code(&block);
        verify(&out).expect("dead-code elimination must not break the block");

        // The whole parity chain — the popcount, the constant 1, the mask, the
        // constant 0 and the comparison — goes transitively, in one backward
        // pass.
        assert_eq!(count(&out, Opcode::POPCOUNT), 0, "{out}");
        assert_eq!(count(&out, Opcode::SETCOND), 0, "{out}");
        assert_eq!(count(&out, Opcode::AND), 0, "{out}");
        assert_eq!(out.insts().len(), before - 5, "{out}");

        // What must survive: the value that is stored, the store, both
        // charges, both boundaries and the terminator.
        assert_eq!(count(&out, Opcode::XOR), 1);
        assert_eq!(count(&out, Opcode::ST), 1);
        assert_eq!(count(&out, Opcode::CHARGE), 2);
        assert_eq!(count(&out, Opcode::INSN_START), 2);
        assert_eq!(count(&out, Opcode::EXIT_TB), 1);
        assert_eq!(out.marks(), block.marks(), "the records must be untouched");
    }

    #[test]
    fn the_same_flag_named_live_at_a_boundary_stays() {
        // The subtle bug this pass could have: nothing in the block consumes
        // the parity flag in *either* version. The only difference is the
        // boundary mapping, which is read from outside the IR by the exception
        // path — so a liveness that counts only operand slots deletes the
        // value a mid-block fault was going to report, and no test that does
        // not fault notices.
        let block = parity_block(true);
        verify(&block).expect("the input block is well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("dead-code elimination must not break the block");

        assert_eq!(out.insts(), block.insts(), "nothing here is dead:\n{out}");
        assert_eq!(count(&out, Opcode::POPCOUNT), 1);
        assert_eq!(count(&out, Opcode::SETCOND), 1);
    }

    #[test]
    fn a_volatile_load_survives_and_a_plain_one_does_not() {
        // The 6502 dummy read: the value is discarded by construction, and the
        // bus cycle is the entire point of the access.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let addr = b.imm(Type::I64, Const::Int(0x20ff));
        let mut dummy = MemOp::load(Width::U8);
        dummy.volatile = true;
        let _ = b.load(Type::I32, addr, dummy);
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U8));
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("still well formed");
        assert_eq!(count(&out, Opcode::LD), 1, "{out}");
        let ld = out
            .insts()
            .iter()
            .find(|i| i.op == Opcode::LD)
            .expect("the volatile load is the one that stayed");
        assert!(ld.mem.expect("a load carries its descriptor").volatile);
    }

    #[test]
    fn effects_and_control_flow_are_never_removed() {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let flag = b.imm(Type::I1, Const::Int(1));
        // No result, not a terminator, not flagged as a side effect: the case
        // a "no dst means dead" rule would delete, taking the control flow
        // with it. Its target is patched once the instructions it jumps over
        // exist, because a branch is forward or the verifier rejects it.
        let branch = b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[flag], None, None, 0);
        // A helper's return value is unused here, and a helper may do
        // anything — a mode change, a device access, a trap.
        let _ = b.emit(Opcode::CALL_HELPER, Type::I64, &[]);
        // An atomic read-modify-write, likewise: the memory changed.
        let addr = b.imm(Type::I64, Const::Int(0x40));
        let one = b.imm(Type::I64, Const::Int(1));
        let _ = b.emit(Opcode::FETCH_ADD, Type::I64, &[addr, one]);
        b.patch_aux(branch, b.next_index() as u32);
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("still well formed");
        assert_eq!(out.insts(), block.insts(), "nothing here may go:\n{out}");
    }

    #[test]
    fn eliminating_dead_code_repoints_a_forward_branch_at_what_is_left() {
        // A `brcond`'s target is an *instruction index*, so dropping anything
        // ahead of it slides the target — a superblock's side exit would land
        // in the middle of the trace it was meant to skip. Latent until
        // traces: the first frontend emitted no branch at all.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let flag = b.imm(Type::I1, Const::Int(0));
        let branch = b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[flag], None, None, 0);
        // Three dead instructions between the branch and its target, all of
        // which elimination removes.
        let a = b.imm(Type::I64, Const::Int(3));
        let n = b.unary(Opcode::NEG, Type::I64, a);
        let _ = b.unary(Opcode::NOT, Type::I64, n);
        b.patch_aux(branch, b.next_index() as u32);
        b.charge(2);
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("still well formed");
        let (at, brcond) = out
            .insts()
            .iter()
            .enumerate()
            .find(|(_, i)| i.op == Opcode::BRCOND)
            .expect("the branch stayed");
        // It still jumps to the charge, which is now three instructions
        // earlier than it was.
        assert_eq!(
            out.insts()[brcond.aux as usize].op,
            Opcode::CHARGE,
            "the branch lost its target:\n{out}"
        );
        assert_eq!(brcond.aux as usize, at + 1, "{out}");
    }

    #[test]
    fn a_dead_chain_goes_all_the_way_down() {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let a = b.imm(Type::I64, Const::Int(3));
        let x = b.unary(Opcode::NEG, Type::I64, a);
        let y = b.unary(Opcode::NOT, Type::I64, x);
        let _ = b.binary(Opcode::MUL, Type::I64, y, y);
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("still well formed");
        assert_eq!(out.insts().len(), 3, "only the frame survives:\n{out}");
        assert_eq!(count(&out, Opcode::MOV), 0);
    }

    #[test]
    fn eliminating_twice_changes_nothing_the_first_pass_left() {
        let block = parity_block(false);
        let once = eliminate_dead_code(&block);
        let twice = eliminate_dead_code(&once);
        assert_eq!(once.insts(), twice.insts(), "the pass is not idempotent");
    }

    #[test]
    fn liveness_reports_uses_and_intervals() {
        let block = parity_block(true);
        let live = Liveness::compute(&block);

        // t0 and t1 are the XOR's operands, defined at 2 and 3 and read by the
        // XOR at 4.
        let a = live.life(Temp(0)).expect("t0 is allocated");
        assert_eq!(a.def, Some(2));
        assert_eq!(a.uses, 1);
        assert_eq!(a.interval(), Some((2, 4)));
        assert!(!a.at_boundary);

        // The XOR's result is read twice: by the popcount and by the store.
        let result = live.life(Temp(2)).expect("t2 is allocated");
        assert_eq!(result.uses, 2);
        assert!(result.is_live());

        // The parity flag has no consumer at all and is live anyway, its range
        // reaching the boundary that names it.
        let parity = live.life(Temp(7)).expect("t7 is the parity flag");
        assert_eq!(parity.uses, 0);
        assert!(parity.at_boundary);
        assert!(parity.is_live());
        let (def, end) = parity.interval().expect("it is assigned");
        assert!(end > def, "the range must reach the boundary");

        // Intervals come back in start order, which is what linear scan wants.
        let intervals = live.intervals();
        assert!(
            intervals.windows(2).all(|w| w[0].1 <= w[1].1),
            "{intervals:?}"
        );
        assert_eq!(intervals.len(), block.temp_count());
    }

    #[test]
    fn liveness_after_elimination_has_no_dead_temporaries_left() {
        let out = eliminate_dead_code(&parity_block(false));
        let live = Liveness::compute(&out);
        for (temp, life) in live.iter() {
            assert!(
                life.def.is_none() || life.is_live(),
                "{temp} is still assigned and still dead:\n{out}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Hoisting slot reads
    // -----------------------------------------------------------------------

    const X0: RegSlot = RegSlot(0);
    const X1: RegSlot = RegSlot(1);
    const X2: RegSlot = RegSlot(2);

    /// Where the slot reads ended up, as `(instruction index, slot)`.
    fn slot_reads(block: &Block) -> Vec<(usize, u32)> {
        block
            .insts()
            .iter()
            .enumerate()
            .filter(|(_, i)| i.op == Opcode::GET_SLOT)
            .map(|(n, i)| (n, i.aux))
            .collect()
    }

    /// Everything a block can be observed to do, in order.
    ///
    /// The oracle a reordering has to leave alone: the outcome, the guest
    /// state, the ticks, and the exact sequence of host calls. Anything a
    /// backend can see is in here; anything else is not observable.
    #[derive(Debug, PartialEq, Eq)]
    struct Trace {
        state: Vec<(RegSlot, u128)>,
        ticks: u64,
        log: Vec<alloc::string::String>,
    }

    impl Trace {
        fn new(slots: &[(RegSlot, u64)]) -> Trace {
            Trace {
                state: slots.iter().map(|&(s, v)| (s, u128::from(v))).collect(),
                ticks: 0,
                log: Vec::new(),
            }
        }
    }

    impl crate::ir::IrHost for Trace {
        fn read_slot(&mut self, slot: RegSlot) -> u128 {
            let v = self
                .state
                .iter()
                .find(|(s, _)| *s == slot)
                .map_or(0, |(_, v)| *v);
            self.log.push(alloc::format!("read {} = {v}", slot.0));
            v
        }
        fn write_slot(&mut self, slot: RegSlot, value: u128) {
            self.log.push(alloc::format!("write {} = {value}", slot.0));
            match self.state.iter_mut().find(|(s, _)| *s == slot) {
                Some(entry) => entry.1 = value,
                None => self.state.push((slot, value)),
            }
        }
        fn charge(&mut self, ticks: u64) {
            self.log.push(alloc::format!("charge {ticks}"));
            self.ticks += ticks;
        }
        fn insn_start(&mut self, mark: &InsnStart) {
            self.log.push(alloc::format!("boundary {:#x}", mark.pc));
        }
        // No block here touches memory: the pass moves slot reads, and a slot
        // read is not an access. A block that did would be a different test.
        fn load(&mut self, _mem: &MemOp, _addr: u64) -> crate::core::space::MemResult<u64> {
            crate::core::space::MemResult::Ok(0)
        }
        fn store(
            &mut self,
            _mem: &MemOp,
            _addr: u64,
            _value: u64,
        ) -> crate::core::space::MemResult {
            crate::core::space::MemResult::Ok(())
        }
    }

    /// The log with the slot reads taken out: everything the guest can tell
    /// apart, in order.
    fn effects(t: &Trace) -> Vec<&str> {
        t.log
            .iter()
            .map(alloc::string::String::as_str)
            .filter(|line| !line.starts_with("read "))
            .collect()
    }

    /// The slot reads and what they returned, in order.
    fn reads(t: &Trace) -> Vec<&str> {
        t.log
            .iter()
            .map(alloc::string::String::as_str)
            .filter(|line| line.starts_with("read "))
            .collect()
    }

    fn observe(block: &Block, slots: &[(RegSlot, u64)]) -> (crate::ir::Outcome, Trace) {
        let mut host = Trace::new(slots);
        let out = crate::ir::Interp::new()
            .run(block, &mut host)
            .expect("the block runs");
        (out, host)
    }

    /// Two guest instructions, each reading a register for the first time.
    ///
    /// The shape every frontend in this tree emits and the one the pass exists
    /// for: `insn_start`, `charge`, `get_slot`, arithmetic, repeat. Neither
    /// slot is bound by an earlier boundary, so both reads may travel.
    fn two_reads() -> Block {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let x0 = b.get_slot(Type::I64, X0);
        let one = b.imm(Type::I64, Const::Int(1));
        let sum = b.binary(Opcode::ADD, Type::I64, x0, one);

        b.insn_start(mark(0x1002, 1, &[(X2, sum)]));
        b.charge(1);
        let x1 = b.get_slot(Type::I64, X1);
        let both = b.binary(Opcode::ADD, Type::I64, sum, x1);

        b.insn_start(mark(0x1004, 2, &[(X2, both)]));
        b.exit_tb();
        b.finish()
    }

    #[test]
    fn a_slot_read_travels_to_the_top_of_the_block() {
        let block = two_reads();
        verify(&block).expect("the input block is well formed");
        assert_eq!(slot_reads(&block), vec![(2, 0), (7, 1)]);

        let out = hoist_slot_reads(&block);
        verify(&out).expect("hoisting must not break the block");

        // Both reads are now ahead of the block's first boundary, so a backend
        // that defers its bookkeeping has nothing at all to replay before
        // either of them — which is the whole saving.
        assert_eq!(slot_reads(&out), vec![(0, 0), (1, 1)], "{out}");
        assert_eq!(out.insts().len(), block.insts().len());
    }

    #[test]
    fn hoisting_changes_nothing_the_interpreter_can_see() {
        let slots = [(X0, 7), (X1, 30)];
        let (want_out, want) = observe(&two_reads(), &slots);
        let (got_out, got) = observe(&hoist_slot_reads(&two_reads()), &slots);
        assert_eq!(want_out, got_out);
        assert_eq!(want.state, got.state, "the guest's registers differ");
        assert_eq!(want.ticks, got.ticks, "the tick count differs");
        // Not merely the same end state: the same sequence of everything the
        // guest can tell apart, which is the claim `ROADMAP.md` §0 makes about
        // two engines and the one a reordering is most likely to break. The
        // slot *reads* are what moved and are therefore filtered out of the
        // sequence — and checked separately, because moving one that returned
        // a different value is exactly the defect this pass could have.
        assert_eq!(
            effects(&want),
            effects(&got),
            "the host saw a different sequence"
        );
        assert_eq!(
            reads(&want),
            reads(&got),
            "a moved read returned something else"
        );
    }

    #[test]
    fn a_read_of_a_slot_an_earlier_boundary_binds_stays_put() {
        // Rule 4. `X0` is bound by the first boundary, so reading it is what
        // makes a backend publish, and a read moved above that boundary would
        // publish a different mapping.
        let mut b = BlockBuilder::new(0x1000, 0);
        let seed = b.imm(Type::I64, Const::Int(9));
        b.insn_start(mark(0x1000, 0, &[(X0, seed)]));
        b.charge(1);
        let again = b.get_slot(Type::I64, X0);
        b.insn_start(mark(0x1002, 1, &[(X0, again)]));
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = hoist_slot_reads(&block);
        assert_eq!(slot_reads(&out), slot_reads(&block), "{out}");
    }

    #[test]
    fn a_read_below_a_branch_stays_below_it() {
        // Rules 1 and 2 together, in the shape a superblock's side exit has: a
        // `brcond` that jumps over an inline exit sequence, and a read after
        // the target. Above the branch it would run on a path that was leaving
        // the block; above the target it would leave the taken path reading a
        // temporary nothing assigned.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let cond = b.imm(Type::I1, Const::Int(1));
        let branch = b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[cond],
            None,
            None,
            0,
        );
        b.exit_tb();
        let target = b.next_index();
        b.patch_aux(branch, target as u32);
        let x1 = b.get_slot(Type::I64, X1);
        b.insn_start(mark(0x1002, 1, &[(X2, x1)]));
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = hoist_slot_reads(&block);
        verify(&out).expect("hoisting must not break the block");
        assert_eq!(slot_reads(&out), slot_reads(&block), "{out}");
    }

    #[test]
    fn a_read_after_a_branch_target_hoists_only_as_far_as_the_target() {
        // The same block with a guest instruction after the target, so there
        // *is* somewhere legal to go: the read may join the region the target
        // opened, and no further.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        let cond = b.imm(Type::I1, Const::Int(1));
        let branch = b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[cond],
            None,
            None,
            0,
        );
        b.exit_tb();
        let target = b.next_index();
        b.patch_aux(branch, target as u32);
        b.insn_start(mark(0x1002, 1, &[]));
        b.charge(1);
        let x1 = b.get_slot(Type::I64, X1);
        b.insn_start(mark(0x1004, 2, &[(X2, x1)]));
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = hoist_slot_reads(&block);
        verify(&out).expect("hoisting must not break the block");
        // One past the target: the target instruction is a landing site
        // nothing may move above (rule 1), so the read joins the region it
        // opened rather than displacing it, and the charge below it is what
        // the read no longer has to wait for.
        assert_eq!(slot_reads(&out), vec![(target + 1, 1)], "{out}");
        // And the branch still names the instruction it used to.
        let landed = out.insts()[branch].aux as usize;
        assert_eq!(landed, target, "the branch was repointed wrongly:\n{out}");
        assert_eq!(out.insts()[landed].op, Opcode::INSN_START, "{out}");
    }

    #[test]
    fn a_read_below_a_helper_call_stays_below_it() {
        // Rule 3: a helper may write the guest's registers, so the value on
        // the two sides of one is not the same value.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0, &[]));
        b.charge(1);
        b.emit_raw(
            Opcode::CALL_HELPER,
            Type::I64,
            None,
            None,
            &[],
            None,
            None,
            0,
        );
        let x1 = b.get_slot(Type::I64, X1);
        b.insn_start(mark(0x1002, 1, &[(X2, x1)]));
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = hoist_slot_reads(&block);
        assert_eq!(slot_reads(&out), slot_reads(&block), "{out}");
    }

    #[test]
    fn a_block_with_nothing_to_hoist_is_returned_unchanged() {
        let block = parity_block(false);
        assert_eq!(hoist_slot_reads(&block), block);
    }

    #[test]
    fn hoisting_is_idempotent_and_deterministic() {
        let once = hoist_slot_reads(&two_reads());
        let twice = hoist_slot_reads(&once);
        assert_eq!(once, twice, "a second pass moved something");
        assert_eq!(once, hoist_slot_reads(&two_reads()), "not deterministic");
    }

    #[test]
    fn hoisting_composes_with_dead_code_elimination_either_way_round() {
        // The two passes are independent — one drops instructions and repoints
        // branches, the other moves them and repoints branches — and a
        // frontend may run them in either order. They must agree, or the block
        // a backend sees depends on a pipeline order nothing writes down.
        let block = two_reads();
        let a = eliminate_dead_code(&hoist_slot_reads(&block));
        let b = hoist_slot_reads(&eliminate_dead_code(&block));
        verify(&a).expect("well formed");
        verify(&b).expect("well formed");
        let slots = [(X0, 7), (X1, 30)];
        assert_eq!(observe(&a, &slots).1, observe(&b, &slots).1);
    }
}
