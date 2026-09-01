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

        // A terminator ends the block and nothing follows it.
        if inst.op.is_terminator() && i + 1 != insts.len() {
            return Err(at("a terminator must be the last instruction in a block"));
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
            // The condition ops produce one bit, whatever they compared.
            Opcode::SETCOND => {
                if inst.cond.is_none() {
                    return Err(at("a comparison needs a condition"));
                }
                expect_type(block, inst.dst, Type::I1).map_err(|e| at(&e))?;
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
    fn a_terminator_in_the_middle_is_rejected() {
        let mut b = wellformed();
        b.exit_tb();
        let _ = b.imm(Type::I64, Const::Int(1));
        b.exit_tb();
        let err = verify(&b.finish()).expect_err("early terminator");
        assert!(format!("{err}").contains("must be the last instruction"));
    }
}
