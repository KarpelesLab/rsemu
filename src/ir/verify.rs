//! The IR verifier.
//!
//! Runs between a frontend and a backend, and rejects a malformed block before
//! anything lowers it. That ordering is the point: a miscompile found in
//! generated host code costs a day, and the same defect found here names the
//! instruction.
//!
//! The rules it enforces are the ones the rest of the design depends on — SSA,
//! type agreement, and the two invariants that make precise exceptions and
//! exact tick accounting possible at all (see the module docs).

use crate::core::error::{Error, Result};
use crate::ir::block::Block;
use crate::ir::op::Opcode;
use crate::ir::types::{Temp, Type};
use alloc::format;
use alloc::vec;

/// Check a block, or say what is wrong with it.
///
/// # Errors
///
/// [`Error::Ir`] naming the offending instruction by index.
pub fn verify(block: &Block) -> Result<()> {
    let mut defined = vec![false; block.temp_count()];
    let mut seen_insn_start = false;
    let insts = block.insts();

    for (i, inst) in insts.iter().enumerate() {
        let at = |what: &str| Error::Ir(format!("instruction {i} ({}): {what}", inst.op));

        // Every operand must already have been assigned. Straight-line SSA, so
        // "dominates" is simply "earlier".
        for src in block.srcs(i) {
            match defined.get(src.index()) {
                Some(true) => {}
                Some(false) => {
                    return Err(at(&format!("{src} is used before it is assigned")));
                }
                None => return Err(at(&format!("{src} was never allocated in this block"))),
            }
        }

        // Assigned exactly once.
        for dst in [inst.dst, inst.dst2].into_iter().flatten() {
            let slot = defined
                .get_mut(dst.index())
                .ok_or_else(|| at(&format!("{dst} was never allocated in this block")))?;
            if *slot {
                return Err(at(&format!("{dst} is assigned twice; the IR is SSA")));
            }
            *slot = true;
        }

        // A branch is forward, and lands inside the block.
        //
        // Both halves are load-bearing rather than tidiness. `pass`'s liveness
        // is one backward walk, which computes the true live set only because
        // the sole control flow inside a block is a jump *ahead*; a backward
        // one would need a fixpoint and would silently get a wrong answer
        // instead of an error. And a target past the end is a block that runs
        // off into nothing — [`Interp`](crate::ir::Interp) reports it, but a
        // host backend would have emitted a jump to a label it never placed.
        if inst.op == Opcode::BRCOND {
            let target = inst.aux as usize;
            if target <= i {
                return Err(at(
                    "a brcond must branch forward; a loop inside a block needs a fixpoint                      liveness this IR does not have",
                ));
            }
            if target >= insts.len() {
                return Err(at("the branch target is outside the block"));
            }
        }

        // Only the memory ops carry an access descriptor, and they must.
        let is_mem = matches!(inst.op, Opcode::LD | Opcode::ST);
        if is_mem && inst.mem.is_none() {
            return Err(at("a memory op needs a MemOp descriptor"));
        }
        if !is_mem && inst.mem.is_some() {
            return Err(at("only a memory op may carry a MemOp descriptor"));
        }

        match inst.op {
            // A boundary marker must point at a real record, and ticks may
            // never run backwards: they are counted from block entry and are
            // what a mid-block fault reconstructs the cycle counter from.
            Opcode::INSN_START => {
                let mark = block
                    .marks()
                    .get(inst.aux as usize)
                    .ok_or_else(|| at("the boundary marker points at no record"))?;
                if let Some(prev) = block.marks().get(inst.aux.wrapping_sub(1) as usize)
                    && inst.aux > 0
                    && mark.ticks < prev.ticks
                {
                    return Err(at("retired ticks went backwards across a boundary"));
                }
                for (_, temp) in &mark.live {
                    if !defined.get(temp.index()).copied().unwrap_or(false) {
                        return Err(at(&format!(
                            "{temp} is named live at a boundary but is not assigned yet"
                        )));
                    }
                }
                seen_insn_start = true;
            }
            // A charge outside a guest instruction is a tick nothing can
            // attribute, and the tick count is a hashed output.
            Opcode::CHARGE => {
                if !seen_insn_start {
                    return Err(at("a charge before the first guest instruction boundary"));
                }
                if inst.imm.is_none() {
                    return Err(at("a charge needs a tick count"));
                }
            }
            // A slot read takes its slot from `aux` and nothing else: an
            // operand would mean the slot were computed, and a slot is a
            // decode constant on every frontend.
            Opcode::GET_SLOT => {
                if inst.dst.is_none() {
                    return Err(at("a slot read must produce a value"));
                }
                if !block.srcs(i).is_empty() {
                    return Err(at("a slot read takes its slot from aux, not an operand"));
                }
            }
            // The condition ops produce one bit, whatever they compared.
            Opcode::SETCOND => {
                if inst.cond.is_none() {
                    return Err(at("a comparison needs a condition"));
                }
                expect_type(block, inst.dst, Type::I1).map_err(|e| at(&e))?;
            }
            // The width-sensitive ops: their value operands are the
            // instruction's own type, or the answer is not defined.
            //
            // The IR does **not** require an instruction's type to match its
            // operands' in general, and most of the time that is harmless —
            // `add.i32` over an `i64` operand computes at 64 bits and masks to
            // 32, which is the same answer. For these ops it is not the same
            // answer, because the width *is* an operand: a rotate wraps at it,
            // a bit count counts within it, a `bswap` lane divides it, a
            // widening multiply's high half starts at it.
            //
            // [`Interp`](crate::ir::Interp) computes at the operand's width and
            // masks at the instruction's, which for `clz.i32` over an `i64`
            // reaches `a.leading_zeros() - (128 - 32)` and **underflows** — a
            // debug-build panic on a block this verifier used to accept. The
            // interpreter now saturates rather than panicking, and this is the
            // rule that means it never has to: the shape is rejected here, one
            // level above, where the error can name the instruction.
            //
            // `jit::x86`'s `Compiler::src_typed` refuses exactly this list and
            // says the same thing, and neither frontend in the tree emits one.
            Opcode::CLZ
            | Opcode::CTZ
            | Opcode::POPCOUNT
            | Opcode::BSWAP
            | Opcode::ROTL
            | Opcode::ROTR
            | Opcode::MULU2
            | Opcode::MULS2 => {
                // The rotates take their amount from a second operand, which
                // is a count rather than a value and is reduced modulo the
                // width whatever its type. Only the value operands are checked.
                let values = if matches!(inst.op, Opcode::MULU2 | Opcode::MULS2) {
                    2
                } else {
                    1
                };
                for k in 0..values {
                    let src = block.srcs(i).get(k).copied();
                    if src.is_some() {
                        expect_type(block, src, inst.ty).map_err(|e| at(&e))?;
                    }
                }
            }
            // Carry in and carry out are one bit each.
            Opcode::ADDC | Opcode::SUBB | Opcode::ROTLC | Opcode::ROTRC => {
                let srcs = block.srcs(i);
                let carry_in = srcs
                    .last()
                    .copied()
                    .ok_or_else(|| at("a carry op needs a carry in"))?;
                expect_type(block, Some(carry_in), Type::I1).map_err(|e| at(&e))?;
                if inst.dst2.is_none() {
                    return Err(at("a carry op must produce its carry out"));
                }
                expect_type(block, inst.dst2, Type::I1).map_err(|e| at(&e))?;
            }
            _ => {}
        }
    }

    match insts.last() {
        Some(last) if last.op.is_terminator() => Ok(()),
        Some(_) => Err(Error::Ir(format!(
            "block {:#x} does not end in a terminator",
            block.entry_pc
        ))),
        None => Err(Error::Ir(format!("block {:#x} is empty", block.entry_pc))),
    }
}

fn expect_type(
    block: &Block,
    temp: Option<Temp>,
    want: Type,
) -> core::result::Result<(), alloc::string::String> {
    let Some(temp) = temp else {
        return Err(format!("expected a {want} operand and there is none"));
    };
    match block.type_of(temp) {
        Some(got) if got == want => Ok(()),
        Some(got) => Err(format!("{temp} is {got} where {want} is required")),
        None => Err(format!("{temp} was never allocated in this block")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::block::{BlockBuilder, InsnStart};
    use crate::ir::types::Const;
    use alloc::vec::Vec;

    fn mark(pc: u64, ticks: u64) -> InsnStart {
        InsnStart {
            pc,
            next_pc: pc + 4,
            ticks,
            live: Vec::new(),
        }
    }

    fn wellformed() -> BlockBuilder {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        b.charge(1);
        b
    }

    #[test]
    fn a_wellformed_block_verifies() {
        let mut b = wellformed();
        let x = b.imm(Type::I64, Const::Int(7));
        let _ = b.binary(Opcode::ADD, Type::I64, x, x);
        b.exit_tb();
        verify(&b.finish()).expect("this block is well formed");
    }

    #[test]
    fn a_block_without_a_terminator_is_rejected() {
        let mut b = wellformed();
        let _ = b.imm(Type::I64, Const::Int(1));
        let err = verify(&b.finish()).expect_err("no terminator");
        assert!(format!("{err}").contains("does not end in a terminator"));
    }

    #[test]
    fn an_empty_block_is_rejected() {
        let err = verify(&BlockBuilder::new(0x20, 0).finish()).expect_err("empty");
        assert!(format!("{err}").contains("is empty"));
    }

    #[test]
    fn a_charge_before_any_boundary_is_rejected() {
        // The tick count is hashed, so a tick that belongs to no guest
        // instruction cannot be reconstructed at a fault.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.charge(1);
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("unattributed charge");
        assert!(format!("{err}").contains("before the first guest instruction boundary"));
    }

    #[test]
    fn ticks_may_not_run_backwards_across_a_boundary() {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 5));
        b.insn_start(mark(0x1004, 2));
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("ticks went backwards");
        assert!(format!("{err}").contains("ticks went backwards"));
    }

    #[test]
    fn equal_ticks_across_a_boundary_are_fine() {
        // Equal is not backwards: an instruction that charges nothing is
        // ordinary, and the counter simply does not move.
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 3));
        b.insn_start(mark(0x1004, 3));
        b.exit_tb();
        verify(&b.finish()).expect("equal ticks are legal");
    }

    #[test]
    fn using_a_temporary_before_it_is_assigned_is_rejected() {
        let mut b = wellformed();
        let later = b.temp(Type::I64);
        let _ = b.binary(Opcode::ADD, Type::I64, later, later);
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("use before def");
        assert!(format!("{err}").contains("used before it is assigned"));
    }

    #[test]
    fn a_carry_in_must_be_one_bit() {
        let mut b = wellformed();
        let a = b.imm(Type::I32, Const::Int(1));
        let wide_carry = b.imm(Type::I32, Const::Int(1));
        let _ = b.addc(Opcode::ADDC, Type::I32, a, a, wide_carry);
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("carry must be i1");
        assert!(format!("{err}").contains("is i32 where i1 is required"));
    }

    #[test]
    fn a_block_may_hold_several_terminators_because_a_superblock_has_side_exits() {
        // The rule used to be "a terminator is the last instruction", and it
        // was wrong the moment traces landed: a superblock merges across a
        // branch and leaves the other side as an inline exit sequence, which
        // ends in a terminator with the rest of the trace after it
        // (`ROADMAP.md` §9, mechanism 4).
        let mut b = wellformed();
        let cond = b.imm(Type::I1, Const::Int(1));
        let over = b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[cond],
            None,
            None,
            0,
        );
        b.insn_start(mark(0x1004, 1));
        b.exit_tb();
        b.patch_aux(over, b.next_index() as u32);
        b.insn_start(mark(0x1008, 1));
        b.exit_tb();
        verify(&b.finish()).expect("two exits are legal");
    }

    #[test]
    fn a_bit_count_over_an_operand_of_another_width_is_rejected() {
        // The hole this rule closes: `Interp` counts leading zeros in a `u128`
        // and subtracts `128 - w`, so `clz.i32` over an `i64` operand holding
        // anything above `2^32` underflowed and panicked in a debug build — on
        // a block the verifier used to accept. The width is an *operand* of
        // these ops, not just the width their result is masked to.
        let mut b = wellformed();
        let wide = b.imm(Type::I64, Const::Int(0x1_0000_0000));
        let narrow = b.temp(Type::I32);
        b.emit_raw(
            Opcode::CLZ,
            Type::I32,
            Some(narrow),
            None,
            &[wide],
            None,
            None,
            0,
        );
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("the widths disagree");
        assert!(
            format!("{err}").contains("is i64 where i32 is required"),
            "{err}"
        );
    }

    #[test]
    fn a_rotate_takes_its_amount_from_an_operand_of_any_width() {
        // The counterpart, and the reason the rule is stated per operand
        // rather than per instruction: a rotate's *value* decides the width,
        // its *amount* is reduced modulo that width whatever type it has, and
        // `cpu::x86::lift` really does hand a count of a different type.
        let mut b = wellformed();
        let value = b.imm(Type::I32, Const::Int(0x1234));
        let amount = b.imm(Type::I64, Const::Int(3));
        let _ = b.binary(Opcode::ROTL, Type::I32, value, amount);
        b.exit_tb();
        verify(&b.finish()).expect("only the value operand carries the width");
    }

    #[test]
    fn a_widening_multiply_checks_both_of_its_value_operands() {
        let mut b = wellformed();
        let a = b.imm(Type::I32, Const::Int(3));
        let wide = b.imm(Type::I64, Const::Int(5));
        let lo = b.temp(Type::I32);
        let hi = b.temp(Type::I32);
        b.emit_raw(
            Opcode::MULU2,
            Type::I32,
            Some(lo),
            Some(hi),
            &[a, wide],
            None,
            None,
            0,
        );
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("the second operand is wider");
        assert!(
            format!("{err}").contains("is i64 where i32 is required"),
            "{err}"
        );
    }

    #[test]
    fn a_backward_branch_inside_a_block_is_rejected() {
        // `pass`'s liveness is a single backward walk, which is exact only for
        // forward control flow. A block with a loop in it would get a wrong
        // live set rather than an error, so the verifier refuses one.
        let mut b = wellformed();
        let cond = b.imm(Type::I1, Const::Int(1));
        let at = b.next_index() as u32;
        let _ = b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[cond],
            None,
            None,
            at,
        );
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("a backward branch");
        assert!(format!("{err}").contains("must branch forward"), "{err}");
    }

    #[test]
    fn a_branch_off_the_end_of_the_block_is_rejected() {
        let mut b = wellformed();
        let cond = b.imm(Type::I1, Const::Int(1));
        let _ = b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[cond],
            None,
            None,
            99,
        );
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("a branch off the end");
        assert!(format!("{err}").contains("outside the block"), "{err}");
    }
}
