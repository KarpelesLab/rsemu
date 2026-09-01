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

use crate::ir::block::{Block, InsnStart};
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
        // with it.
        b.emit_void(Opcode::BRCOND, Type::I1, &[flag]);
        // A helper's return value is unused here, and a helper may do
        // anything — a mode change, a device access, a trap.
        let _ = b.emit(Opcode::CALL_HELPER, Type::I64, &[]);
        // An atomic read-modify-write, likewise: the memory changed.
        let addr = b.imm(Type::I64, Const::Int(0x40));
        let one = b.imm(Type::I64, Const::Int(1));
        let _ = b.emit(Opcode::FETCH_ADD, Type::I64, &[addr, one]);
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");

        let out = eliminate_dead_code(&block);
        verify(&out).expect("still well formed");
        assert_eq!(out.insts(), block.insts(), "nothing here may go:\n{out}");
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
}
