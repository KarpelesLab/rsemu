//! Linear-scan register allocation over a block's live intervals.
//!
//! `ROADMAP.md` §9's pipeline ends *"→ register allocation (linear scan) →
//! host backend"*, and this is that step. It is here rather than in a backend
//! because everything it decides is a property of the **block**: which
//! temporaries are live at once, which of them a branch can jump over, and
//! which have to survive a call. What a backend contributes is two lists of
//! register numbers and a flag per instruction, and it gets back one home per
//! temporary. A second host backend re-uses the algorithm instead of writing
//! it again.
//!
//! [`Liveness`] already computes the intervals, in the order this wants them
//! (Poletto and Sarkar, *Linear scan register allocation*, ACM TOPLAS 21(5),
//! 1999, §4: intervals sorted by increasing start point, an active list sorted
//! by increasing end point). Nothing here recomputes them.
//!
//! # Three rules that are not in the textbook, and why each is here
//!
//! ## 1. A temporary a branch can jump over stays in the frame
//!
//! A block is straight-line SSA **with forward branches** (the module docs,
//! decision 7), so a [`Opcode::BRCOND`] at `i` targeting `t` skips
//! `i+1 ..= t-1`. A temporary defined in that window and read at or after `t`
//! is *undefined* on the taken path — [`Interp`](crate::ir::Interp) reads the
//! zero its frame was initialized with, and a host register would hold
//! whatever the last temporary to own it left behind. Those are different
//! numbers, and no test that does not take the branch can tell.
//!
//! So such a temporary is pinned to the frame. It is the one place where "the
//! interval is a contiguous range" stops being the whole truth about a block,
//! and it is checked rather than assumed:
//! `a_definition_a_branch_jumps_over_never_reaches_a_register`.
//!
//! ## 2. Two register classes, because a call is not a barrier for all of them
//!
//! Generated code calls into the host at every [`Opcode::CHARGE`], every
//! [`Opcode::INSN_START`], every [`Opcode::GET_SLOT`] and on the slow path of
//! every access — which on the frontends in this tree is two or three calls
//! per *guest instruction*. A caller-saved host register is therefore useless
//! to anything that outlives one, and a callee-saved one is scarce: the System
//! V AMD64 ABI has six, and the x86-64 backend has already spent two of them
//! on the context and the frame.
//!
//! Splitting the bank is what makes both usable. An interval that crosses a
//! call gets a saved register or nothing; one that does not prefers a volatile
//! register, precisely so the saved ones stay free for the intervals that have
//! no alternative.
//!
//! **A call at either end of an interval does not cross it.** Every lowering
//! reads its operands before its call and writes its result after — a
//! `get_slot` has no operands at all, a load reads its address into an
//! argument register first — so only a call *strictly between* the definition
//! and the last use can clobber the value. Treating the endpoints as crossings
//! costs a register on nearly every interval in a block, which on three saved
//! registers is most of the allocation.
//!
//! **A call in the *gap* before an instruction is the other case, and it is
//! not the same one.** A backend may emit a call that runs before the
//! instruction it belongs to has read anything — the x86-64 backend replays
//! its deferred tick bookkeeping that way, once per region rather than once
//! per guest instruction. Such a call clobbers an interval that is merely
//! *read* at that instruction, which the rule above deliberately lets keep a
//! volatile register. So [`CallSites`] carries the two separately and
//! [`crosses`] tests `start < gap <= end` for one and `start < call < end` for
//! the other. Collapsing them into one array is safe in exactly one direction
//! and wrong in the other, which is why they are not one array.
//!
//! ## 3. What "spilled" means here, and what it does not
//!
//! A spilled temporary lives in its frame slot for the **whole** block rather
//! than being written out at one point and reloaded at another. That is what
//! makes the decision expressible as one [`Home`] per temporary, and it is
//! also what keeps the backend's precise-exception story intact: the frame is
//! the only thing the exception path can read, so "sometimes in the frame" is
//! a state the runtime would have to be able to name. The cost is a worse
//! allocation than a splitting allocator would find; the benefit is that
//! nothing outside this file has to know when a value moved.
//!
//! The spill heuristic is the textbook one — when no register is free, the
//! interval with the furthest end point loses, current interval included —
//! restricted to the class the current interval can actually use.
//!
//! # Determinism
//!
//! Every choice is made from index-ordered data: intervals come back sorted by
//! `(start, temp)`, the free set is a bitmask and the lowest-numbered register
//! wins, and ties in the spill heuristic break on the lower temporary number.
//! The same block always allocates the same way, which `ROADMAP.md` §0 needs
//! for a state hash to be an identity rather than a coincidence.

use crate::ir::block::Block;
use crate::ir::op::Opcode;
use crate::ir::pass::Liveness;
use crate::ir::types::Temp;
use alloc::vec;
use alloc::vec::Vec;

/// The largest register number an allocation can name.
///
/// Sixteen, because the free set is a `u16` bitmask and every host this crate
/// targets has sixteen or thirty-two general registers — a backend with more
/// passes the sixteen it is willing to give away.
pub const MAX_REGS: u8 = 16;

/// Where a temporary lives, for the whole block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Home {
    /// Its slot in the temporary frame.
    ///
    /// Also where the exception path reads it, so a temporary named at an
    /// [`InsnStart`](crate::ir::InsnStart) boundary must reach here whatever
    /// else it does — see [`Allocation::frame_backed`].
    Frame,
    /// A host register, in the backend's own numbering.
    Reg(u8),
}

/// The registers a backend is willing to hand out.
///
/// Two lists rather than one, because a call into the host clobbers one of
/// them and not the other. A register in neither list is one the backend has
/// reserved — a context pointer, a fixed scratch — and the allocator never
/// names it.
#[derive(Debug, Clone, Copy)]
pub struct RegBanks<'a> {
    /// Registers whose value survives a call into the host.
    pub saved: &'a [u8],
    /// Registers a call into the host may clobber.
    pub volatile: &'a [u8],
}

/// Where a backend's lowering calls into the host, per instruction.
///
/// Two arrays because a call's *position within* an instruction's lowering
/// changes which intervals it can destroy — see rule 2 in the module docs. A
/// short or missing entry is read as *no call*, so a backend that miscounts
/// loses an allocation rather than correctness; it is also the one input this
/// module cannot check for itself, which is why the x86-64 backend derives
/// both arrays from one expression each and asserts them against the bytes it
/// emitted.
#[derive(Debug, Clone, Copy, Default)]
pub struct CallSites<'a> {
    /// `inside[i]`: instruction `i`'s lowering calls out *after* reading its
    /// operands and *before* writing its results.
    pub inside: &'a [bool],
    /// `before[i]`: a call runs in the gap ahead of instruction `i`, before it
    /// has read anything at all.
    pub before: &'a [bool],
}

/// One block's assignment: a [`Home`] per temporary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Allocation {
    homes: Vec<Home>,
    boundary: Vec<bool>,
}

impl Allocation {
    /// Where `temp` lives. [`Home::Frame`] for a temporary this block never
    /// allocated, which is the conservative answer.
    #[inline]
    #[must_use]
    pub fn home(&self, temp: Temp) -> Home {
        self.homes.get(temp.index()).copied().unwrap_or(Home::Frame)
    }

    /// Whether `temp`'s frame slot holds its value.
    ///
    /// True for a spilled temporary, and true for one that lives in a register
    /// but is **also** written through to the frame because an
    /// [`InsnStart`](crate::ir::InsnStart) names it as live guest state. That
    /// second case is the whole of `ROADMAP.md` §9's precise-exception
    /// requirement in this design: the exception path materializes
    /// architectural state out of the frame, so the values it will ask for are
    /// the ones the frame is guaranteed to hold. A backend must emit the
    /// write-through wherever this is true; `jit::x86::tests`' `agree_under`
    /// is what fails if one stops, on every block its differential generates
    /// rather than only on the ones that fault.
    ///
    /// A temporary that is neither is dead as soon as its last reader has run,
    /// and nothing outside the block can observe it.
    #[inline]
    #[must_use]
    pub fn frame_backed(&self, temp: Temp) -> bool {
        matches!(self.home(temp), Home::Frame)
            || self.boundary.get(temp.index()).copied().unwrap_or(false)
    }

    /// How many temporaries got a register.
    #[must_use]
    pub fn in_registers(&self) -> usize {
        self.homes
            .iter()
            .filter(|h| !matches!(h, Home::Frame))
            .count()
    }

    /// How many stayed in the frame, definitions and holes alike.
    #[must_use]
    pub fn in_frame(&self) -> usize {
        self.homes.len() - self.in_registers()
    }

    /// An allocation that gives nothing a register — the pre-allocator
    /// backend, kept as the control every differential runs against.
    #[must_use]
    pub fn none(block: &Block) -> Allocation {
        Allocation {
            homes: vec![Home::Frame; block.temp_count()],
            boundary: vec![true; block.temp_count()],
        }
    }
}

/// Assign every temporary a home.
///
/// `sites` says where the backend's lowerings call into the host — see
/// [`CallSites`], and rule 2 in the module docs for why one array would not
/// have been enough.
#[must_use]
pub fn linear_scan(
    block: &Block,
    live: &Liveness,
    banks: &RegBanks<'_>,
    sites: &CallSites<'_>,
) -> Allocation {
    let n = block.temp_count();
    let mut alloc = Allocation {
        homes: vec![Home::Frame; n],
        boundary: vec![false; n],
    };
    for (temp, life) in live.iter() {
        if let Some(slot) = alloc.boundary.get_mut(temp.index()) {
            *slot = life.at_boundary;
        }
    }

    let pinned = pinned(block, live, n);
    let inside = prefix(sites.inside, block.insts().len());
    let gaps = prefix(sites.before, block.insts().len());

    let saved = mask_of(banks.saved);
    let volatile = mask_of(banks.volatile);
    let mut free = saved | volatile;
    let mut active: Vec<Live> = Vec::new();

    for (temp, start, end) in live.intervals() {
        let idx = temp.index();
        if idx >= n {
            continue;
        }
        // Expire. An interval that ends where this one starts may share the
        // register: every lowering reads its operands before it writes its
        // results, so the outgoing value is gone by the time the new one
        // lands.
        //
        // **Unless it also *starts* there**, which is the one case where that
        // reasoning fails and it is not a corner: the three ops with a second
        // result — the widening multiplies and the rotates through carry —
        // define both at the same instruction, and a first result nothing
        // reads has the zero-length interval `[i, i]`. Expiring it here would
        // hand its register straight to the second result, and the lowering
        // then writes the carry over the value. Nothing *reads* that value, so
        // it is unobservable today; it is also one boundary mapping away from
        // being a wrong guest register, and a rule that is only safe because
        // of what the consumer happens to do is not a rule.
        active.retain(|a| {
            if a.end <= start && a.start < start {
                free |= 1 << a.reg;
                false
            } else {
                true
            }
        });
        if pinned[idx] {
            continue;
        }

        // A value that outlives a call has one bank; one that does not prefers
        // the other, so the scarce bank stays free for the values with no
        // alternative.
        let crosses = crosses(&inside, &gaps, start, end);
        let class = if crosses { saved } else { saved | volatile };
        let order = if crosses {
            [saved, 0]
        } else {
            [volatile, saved]
        };

        let mut got = None;
        for want in order {
            if want != 0
                && let Some(r) = take(&mut free, want)
            {
                got = Some(r);
                break;
            }
        }
        match got {
            Some(reg) => {
                alloc.homes[idx] = Home::Reg(reg);
                active.push(Live {
                    start,
                    end,
                    temp: idx,
                    reg,
                });
            }
            None => {
                // Nothing free in a class this interval can use. The furthest
                // ending active interval loses — unless that is this one, in
                // which case it stays in the frame and every register keeps
                // the longer-lived value it already holds.
                //
                // The class filter is **provably redundant** and is kept
                // because the rule it states is the one that would still be
                // right if the bank order above changed. The proof, recorded
                // because a mutation that removes it survives every test and
                // that is a fact about the algorithm rather than about the
                // corpus: suppose this interval `[s, e]` crosses a call at
                // `c`, so it needs a saved register, and suppose the victim is
                // an active interval `[s', e']` in a volatile one. It is
                // active, so `s' <= s`; it is worth stealing, so `e' > e`; and
                // it is in a volatile register, so no call lies in `(s', e')`.
                // But `s' <= s < c < e < e'` puts `c` there. There is no such
                // victim.
                let victim = active
                    .iter()
                    .enumerate()
                    .filter(|(_, a)| class & (1 << a.reg) != 0)
                    .max_by_key(|(_, a)| (a.end, core::cmp::Reverse(a.temp)))
                    .map(|(at, _)| at);
                if let Some(at) = victim
                    && active[at].end > end
                {
                    alloc.homes[active[at].temp] = Home::Frame;
                    let reg = active[at].reg;
                    active[at] = Live {
                        start,
                        end,
                        temp: idx,
                        reg,
                    };
                    alloc.homes[idx] = Home::Reg(reg);
                }
            }
        }
    }

    alloc
}

/// One interval currently holding a register.
#[derive(Debug, Clone, Copy)]
struct Live {
    start: u32,
    end: u32,
    temp: usize,
    reg: u8,
}

/// The bitmask of a register list, ignoring anything out of range.
fn mask_of(regs: &[u8]) -> u16 {
    regs.iter()
        .filter(|r| **r < MAX_REGS)
        .fold(0u16, |m, r| m | (1u16 << r))
}

/// Take the lowest-numbered free register in `want`.
fn take(free: &mut u16, want: u16) -> Option<u8> {
    let avail = *free & want;
    if avail == 0 {
        return None;
    }
    let reg = avail.trailing_zeros() as u8;
    *free &= !(1u16 << reg);
    Some(reg)
}

/// `out[i]` is how many of `calls[..i]` are true.
fn prefix(calls: &[bool], len: usize) -> Vec<u32> {
    let mut out = vec![0u32; len + 1];
    for i in 0..len {
        out[i + 1] = out[i] + u32::from(calls.get(i).copied().unwrap_or(false));
    }
    out
}

/// Whether any call can destroy a value held from `start` to `end`.
///
/// Two windows, and the asymmetry is the whole point (module docs, rule 2): a
/// call *within* an instruction runs between that instruction's reads and its
/// writes, so only `start < i < end` reaches the value; a call in the *gap*
/// ahead of an instruction runs before its reads, so `start < i <= end` does.
fn crosses(inside: &[u32], gaps: &[u32], start: u32, end: u32) -> bool {
    any(inside, start + 1, end) || any(gaps, start + 1, end + 1)
}

/// Whether `counts` records anything in `[lo, hi)`, where `counts[i]` is the
/// number of marked instructions below `i`.
fn any(counts: &[u32], lo: u32, hi: u32) -> bool {
    if counts.is_empty() || hi <= lo {
        return false;
    }
    let top = counts.len() - 1;
    let hi = (hi as usize).min(top);
    let lo = (lo as usize).min(hi);
    counts[hi] > counts[lo]
}

/// The temporaries that must stay in the frame whatever the intervals say.
///
/// Two cases, and the second is not rule 1:
///
/// * a definition a forward branch can jump over — see rule 1 in the module
///   docs;
/// * a temporary **read before it is assigned**, which
///   [`verify`](crate::ir::verify) rejects but which nothing obliges a caller
///   to have run: `compile` is public and the dispatcher does not verify. The
///   interpreter reads the zero its frame was initialized with; a register
///   would hold the previous owner's value. Pinning makes a malformed block
///   translate to something that agrees with the oracle rather than to
///   something that disagrees.
///
/// Quadratic in branches times temporaries, which is nothing on a translation
/// block and is worth the plainness: the alternative is an interval tree for a
/// loop that runs a handful of times.
fn pinned(block: &Block, live: &Liveness, n: usize) -> Vec<bool> {
    let mut pinned = vec![false; n];

    let mut defined = vec![false; n];
    for (i, inst) in block.insts().iter().enumerate() {
        for src in block.srcs(i) {
            if let Some(false) = defined.get(src.index()).copied()
                && let Some(slot) = pinned.get_mut(src.index())
            {
                *slot = true;
            }
        }
        for dst in [inst.dst, inst.dst2].into_iter().flatten() {
            if let Some(slot) = defined.get_mut(dst.index()) {
                *slot = true;
            }
        }
    }

    for (i, inst) in block.insts().iter().enumerate() {
        if inst.op != Opcode::BRCOND {
            continue;
        }
        let target = inst.aux as usize;
        if target <= i {
            continue;
        }
        for (t, slot) in pinned.iter_mut().enumerate().take(n) {
            let Some(life) = live.life(Temp(t as u32)) else {
                continue;
            };
            let Some(def) = life.def else { continue };
            let last = life.last_use.unwrap_or(def) as usize;
            let def = def as usize;
            if def > i && def < target && last >= target {
                *slot = true;
            }
        }
    }
    pinned
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::block::{BlockBuilder, InsnStart, RegSlot};
    use crate::ir::op::Opcode;
    use crate::ir::types::{Const, Type};
    use alloc::vec;

    const SAVED: [u8; 2] = [0, 1];
    const VOLATILE: [u8; 2] = [2, 3];

    fn banks() -> RegBanks<'static> {
        RegBanks {
            saved: &SAVED,
            volatile: &VOLATILE,
        }
    }

    fn mark(pc: u64, live: &[(RegSlot, Temp)]) -> InsnStart {
        InsnStart {
            pc,
            next_pc: pc + 4,
            ticks: 0,
            live: live.to_vec(),
        }
    }

    fn run(block: &Block, calls: &[bool]) -> Allocation {
        run_with(block, calls, &[])
    }

    fn run_with(block: &Block, inside: &[bool], before: &[bool]) -> Allocation {
        let live = Liveness::compute(block);
        linear_scan(block, &live, &banks(), &CallSites { inside, before })
    }

    #[test]
    fn a_short_chain_gets_registers_and_reuses_them() {
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let x = b.imm(Type::I64, Const::Int(1));
        let y = b.unary(Opcode::NEG, Type::I64, x);
        let z = b.unary(Opcode::NOT, Type::I64, y);
        b.insn_start(mark(4, &[(RegSlot(0), z)]));
        b.exit_tb();
        let block = b.finish();
        let calls = vec![true, false, false, false, true, false];
        let alloc = run(&block, &calls);

        // No call falls between any definition and its use, so all three take
        // the volatile bank — and there are only two of those, so the third
        // reuses the first's register.
        assert!(matches!(alloc.home(x), Home::Reg(_)), "{alloc:?}");
        assert!(matches!(alloc.home(y), Home::Reg(_)));
        assert!(matches!(alloc.home(z), Home::Reg(_)));
        assert_eq!(alloc.home(x), alloc.home(z), "the register was not reused");

        // and `z` is named at a boundary, so its frame slot is written too.
        assert!(alloc.frame_backed(z));
        assert!(!alloc.frame_backed(x));
    }

    #[test]
    fn a_value_that_outlives_a_call_takes_a_saved_register() {
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let x = b.imm(Type::I64, Const::Int(1));
        // A call between the definition and the use.
        b.charge(1);
        let y = b.unary(Opcode::NEG, Type::I64, x);
        b.insn_start(mark(4, &[(RegSlot(0), y)]));
        b.exit_tb();
        let block = b.finish();
        let calls = vec![true, false, true, false, true, false];
        let alloc = run(&block, &calls);

        let Home::Reg(r) = alloc.home(x) else {
            panic!("x should have a register: {alloc:?}");
        };
        assert!(SAVED.contains(&r), "a value that crosses a call took r{r}");
    }

    #[test]
    fn a_definition_a_branch_jumps_over_never_reaches_a_register() {
        // The rule the textbook does not have: on the taken path the register
        // holds the previous owner's value and the frame holds zero, and the
        // interpreter reads zero.
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let sel = b.imm(Type::I1, Const::Int(1));
        let branch = b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[sel], None, None, 0);
        let skipped = b.imm(Type::I64, Const::Int(7));
        b.patch_aux(branch, b.next_index() as u32);
        // Read after the branch target: undefined when the branch was taken.
        let _ = b.unary(Opcode::NEG, Type::I64, skipped);
        b.insn_start(mark(4, &[]));
        b.exit_tb();
        let block = b.finish();
        let alloc = run(&block, &vec![false; block.insts().len()]);
        assert_eq!(alloc.home(skipped), Home::Frame, "{alloc:?}");
        // The selector is defined before the branch, so it is unaffected.
        assert!(matches!(alloc.home(sel), Home::Reg(_)));
    }

    #[test]
    fn running_out_of_registers_spills_the_longest_lived() {
        // Five values all live at once over four registers: the one whose last
        // use is furthest away loses, which is the textbook rule.
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let mut temps = Vec::new();
        for i in 0..5u128 {
            temps.push(b.imm(Type::I64, Const::Int(i)));
        }
        // Consume them in reverse, so the first-defined is the last-used.
        let mut acc = temps[4];
        for t in temps.iter().rev().skip(1) {
            acc = b.binary(Opcode::ADD, Type::I64, acc, *t);
        }
        b.insn_start(mark(4, &[(RegSlot(0), acc)]));
        b.exit_tb();
        let block = b.finish();
        let alloc = run(&block, &vec![false; block.insts().len()]);

        let spilled: Vec<Temp> = temps
            .iter()
            .copied()
            .filter(|t| alloc.home(*t) == Home::Frame)
            .collect();
        assert_eq!(spilled, vec![temps[0]], "{alloc:?}");
        assert_eq!(alloc.in_registers() + alloc.in_frame(), block.temp_count());
    }

    #[test]
    fn two_results_of_one_instruction_never_share_a_register() {
        // A widening multiply whose low half nothing reads: its interval is
        // the single point where both results are defined, and expiring it
        // there would give the high half the low half's register — which the
        // backend then writes the high half into after it has already written
        // the low one.
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let x = b.imm(Type::I64, Const::Int(7));
        let y = b.imm(Type::I64, Const::Int(9));
        let lo = b.temp(Type::I64);
        let hi = b.temp(Type::I64);
        b.emit_raw(
            Opcode::MULU2,
            Type::I64,
            Some(lo),
            Some(hi),
            &[x, y],
            None,
            None,
            0,
        );
        b.insn_start(mark(4, &[(RegSlot(0), hi)]));
        b.exit_tb();
        let block = b.finish();
        let alloc = run(&block, &vec![false; block.insts().len()]);
        assert!(matches!(alloc.home(hi), Home::Reg(_)), "{alloc:?}");
        assert_ne!(
            alloc.home(lo),
            alloc.home(hi),
            "both results of one instruction took the same register: {alloc:?}"
        );
    }

    #[test]
    fn a_value_read_at_an_instruction_a_call_runs_ahead_of_takes_a_saved_register() {
        // The asymmetry rule 2 states, in the one shape that separates the two
        // arrays: `x` is defined at one instruction and *read* at the next, and
        // a call runs in the gap between them. Recorded as `inside` the reader
        // it would not cross — an operand is read before its instruction's own
        // call — so the volatile bank would look safe and the callee would
        // destroy the value. Recorded as `before`, it crosses.
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let x = b.imm(Type::I64, Const::Int(1));
        let y = b.unary(Opcode::NEG, Type::I64, x);
        b.insn_start(mark(4, &[(RegSlot(0), y)]));
        b.exit_tb();
        let block = b.finish();
        let n = block.insts().len();

        // The reader is instruction 2. As an `inside` call it does not reach
        // `x`, whose interval ends there.
        let mut inside = vec![false; n];
        inside[2] = true;
        let Home::Reg(r) = run_with(&block, &inside, &[]).home(x) else {
            panic!("x should have a register");
        };
        assert!(VOLATILE.contains(&r), "r{r} was not the volatile bank");

        // The same call in the gap ahead of instruction 2 does reach it.
        let mut before = vec![false; n];
        before[2] = true;
        let Home::Reg(r) = run_with(&block, &[], &before).home(x) else {
            panic!("x should still have a register");
        };
        assert!(SAVED.contains(&r), "a value a gap call crosses took r{r}");
    }

    #[test]
    fn the_allocation_is_the_same_every_time() {
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let mut acc = b.imm(Type::I64, Const::Int(1));
        for i in 0..12u128 {
            let k = b.imm(Type::I64, Const::Int(i));
            acc = b.binary(Opcode::ADD, Type::I64, acc, k);
        }
        b.insn_start(mark(4, &[(RegSlot(0), acc)]));
        b.exit_tb();
        let block = b.finish();
        let calls = vec![false; block.insts().len()];
        assert_eq!(run(&block, &calls), run(&block, &calls));
    }

    #[test]
    fn the_control_allocation_gives_nothing_a_register() {
        let mut b = BlockBuilder::new(0, 0);
        b.insn_start(mark(0, &[]));
        let x = b.imm(Type::I64, Const::Int(1));
        b.insn_start(mark(4, &[(RegSlot(0), x)]));
        b.exit_tb();
        let block = b.finish();
        let alloc = Allocation::none(&block);
        assert_eq!(alloc.in_registers(), 0);
        for t in 0..block.temp_count() {
            assert!(alloc.frame_backed(Temp(t as u32)));
        }
    }
}
