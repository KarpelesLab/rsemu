//! Translation blocks: instructions, guest instruction boundaries, and the
//! builder a frontend drives.

use crate::ir::op::{Cond, MemOp, Opcode};
use crate::ir::types::{Const, Temp, Type};
use alloc::vec::Vec;
use core::fmt;

/// A slot in a guest's architectural state, numbered by that guest's frontend.
///
/// Deliberately opaque here. "The guest register file" is larger than the
/// register struct on five of our nine cores, and an [`InsnStart`] that named
/// only the obvious registers would be unable to reconstruct architectural
/// state at a fault: the Z80's `WZ` is read out by `BIT n,(HL)` and its `Q` by
/// `SCF`/`CCF`, MIPS's `in_delay` decides whether `EPC` names the branch or
/// the delay slot, the 6502's open-bus latches feed `MemAttrs` on every
/// access, and every flag is a slot too (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct RegSlot(pub u16);

/// A guest instruction boundary.
///
/// `ROADMAP.md` §9's precise-exception design: at every boundary the IR
/// records enough to materialize architectural state from a host code offset,
/// so a load that faults halfway through a translated block delivers its
/// exception at exactly the PC the ISA specifies, with exactly the registers
/// that had retired.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InsnStart {
    /// The guest PC of this instruction.
    pub pc: u64,
    /// The guest PC of the next instruction.
    ///
    /// A `next_pc` rather than a length, because `Exit::len` is derived as the
    /// difference and on x86 an instruction's length is not a static property
    /// of its opcode.
    pub next_pc: u64,
    /// Guest ticks retired at this boundary, counted from block entry.
    ///
    /// The column that makes an exact tick count reconstructible at a fault.
    /// A core's cycle counter is in its snapshot and the state hash is taken
    /// over that snapshot, so a JIT that cannot reproduce this number at a
    /// mid-block fault produces a different hash from the interpreter at the
    /// same instruction — which is precisely what phase 5's "save/restore
    /// across an engine switch" gate tests.
    pub ticks: u64,
    /// Which temporary currently holds each live piece of guest state.
    pub live: Vec<(RegSlot, Temp)>,
}

/// One IR instruction.
///
/// Source operands live in the block's flat operand array rather than in a
/// `Vec` per instruction — a per-instruction allocation in a translator is a
/// cost paid on every block. Reach them with [`Block::srcs`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Inst {
    /// The operation.
    pub op: Opcode,
    /// The result type, or for a store or branch the type operated on.
    pub ty: Type,
    /// The result, where there is one.
    pub dst: Option<Temp>,
    /// A second result.
    ///
    /// Three ops need one: the carry out of [`Opcode::ADDC`]/[`Opcode::SUBB`]
    /// and the rotates-through-carry, the high half of a widening multiply,
    /// and a helper call — every soft-float entry point returns a value *and*
    /// its accrued exception flags, which is the tier-1 floating-point plan.
    pub dst2: Option<Temp>,
    /// An immediate.
    pub imm: Option<Const>,
    /// The access descriptor, for [`Opcode::LD`] and [`Opcode::ST`].
    pub mem: Option<MemOp>,
    /// The comparison, for the condition ops.
    pub cond: Option<Cond>,
    /// Opcode-specific payload: an [`InsnStart`] index, a helper id, a label.
    pub aux: u32,
    src_start: u32,
    src_len: u32,
}

/// A translation block: straight-line SSA over typed temporaries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Block {
    /// The guest PC this block starts at.
    pub entry_pc: u64,
    /// The rest of the translation-block cache key.
    ///
    /// `ROADMAP.md` §9 keys the cache on `(guest PC, relevant CPU flags)`.
    /// One of those flags is easy to forget and belongs here: the **exit
    /// mask**. `ExitMask::USER` decides whether an `ecall` vectors into the
    /// guest or leaves the core — two entirely different code sequences from
    /// identical guest bytes — and it can be changed while the core runs, and
    /// is deliberately not snapshotted.
    pub key: u64,
    insts: Vec<Inst>,
    operands: Vec<Temp>,
    marks: Vec<InsnStart>,
    types: Vec<Type>,
}

impl Block {
    /// The block's instructions, in order.
    #[inline]
    #[must_use]
    pub fn insts(&self) -> &[Inst] {
        &self.insts
    }

    /// The source operands of the instruction at `index`.
    #[must_use]
    pub fn srcs(&self, index: usize) -> &[Temp] {
        let inst = &self.insts[index];
        let start = inst.src_start as usize;
        &self.operands[start..start + inst.src_len as usize]
    }

    /// The guest instruction boundaries, in order.
    #[inline]
    #[must_use]
    pub fn marks(&self) -> &[InsnStart] {
        &self.marks
    }

    /// The type of a temporary, or `None` if it was never allocated here.
    #[inline]
    #[must_use]
    pub fn type_of(&self, temp: Temp) -> Option<Type> {
        self.types.get(temp.index()).copied()
    }

    /// How many temporaries this block allocated.
    #[inline]
    #[must_use]
    pub fn temp_count(&self) -> usize {
        self.types.len()
    }
}

impl fmt::Display for Block {
    /// A textual dump, which is what a differential failure gets reported as.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "block {:#x} key {:#x}", self.entry_pc, self.key)?;
        for (i, inst) in self.insts.iter().enumerate() {
            match inst.dst {
                Some(d) => write!(f, "  {d} = {}", inst.op)?,
                None => write!(f, "  {}", inst.op)?,
            }
            if let Some(d2) = inst.dst2 {
                write!(f, " -> {d2}")?;
            }
            write!(f, ".{}", inst.ty)?;
            for src in self.srcs(i) {
                write!(f, " {src}")?;
            }
            if let Some(imm) = inst.imm {
                write!(f, " {imm}")?;
            }
            if inst.op == Opcode::INSN_START
                && let Some(mark) = self.marks.get(inst.aux as usize)
            {
                write!(f, " pc={:#x} ticks={}", mark.pc, mark.ticks)?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

/// Builds a [`Block`]. One per translation; a frontend drives it.
#[derive(Debug)]
pub struct BlockBuilder {
    block: Block,
}

impl BlockBuilder {
    /// A builder for a block entered at `entry_pc` under cache key `key`.
    #[must_use]
    pub fn new(entry_pc: u64, key: u64) -> BlockBuilder {
        BlockBuilder {
            block: Block {
                entry_pc,
                key,
                insts: Vec::new(),
                operands: Vec::new(),
                marks: Vec::new(),
                types: Vec::new(),
            },
        }
    }

    /// Allocate a temporary.
    pub fn temp(&mut self, ty: Type) -> Temp {
        let t = Temp(self.block.types.len() as u32);
        self.block.types.push(ty);
        t
    }

    /// Append an instruction, allocating a result of type `ty`.
    pub fn emit(&mut self, op: Opcode, ty: Type, srcs: &[Temp]) -> Temp {
        let dst = self.temp(ty);
        self.push(op, ty, Some(dst), None, srcs, None, None, 0);
        dst
    }

    /// Append an instruction that produces no result.
    pub fn emit_void(&mut self, op: Opcode, ty: Type, srcs: &[Temp]) {
        self.push(op, ty, None, None, srcs, None, None, 0);
    }

    /// Materialize an immediate.
    pub fn imm(&mut self, ty: Type, value: Const) -> Temp {
        let dst = self.temp(ty);
        self.push(Opcode::MOV, ty, Some(dst), None, &[], Some(value), None, 0);
        dst
    }

    /// A binary operation.
    pub fn binary(&mut self, op: Opcode, ty: Type, a: Temp, b: Temp) -> Temp {
        self.emit(op, ty, &[a, b])
    }

    /// A unary operation.
    pub fn unary(&mut self, op: Opcode, ty: Type, a: Temp) -> Temp {
        self.emit(op, ty, &[a])
    }

    /// Add or subtract with a carry in, yielding `(result, carry_out)`.
    ///
    /// The shape the flag machines need: the 6502's only add is `ADC`, and
    /// ARM expresses its whole ALU this way — `SUB` is `add(a, !b, true)`.
    pub fn addc(&mut self, op: Opcode, ty: Type, a: Temp, b: Temp, carry: Temp) -> (Temp, Temp) {
        let dst = self.temp(ty);
        let carry_out = self.temp(Type::I1);
        self.push(
            op,
            ty,
            Some(dst),
            Some(carry_out),
            &[a, b, carry],
            None,
            None,
            0,
        );
        (dst, carry_out)
    }

    /// Compare two values into a one-bit temporary.
    pub fn setcond(&mut self, cond: Cond, ty: Type, a: Temp, b: Temp) -> Temp {
        let dst = self.temp(Type::I1);
        self.push(
            Opcode::SETCOND,
            ty,
            Some(dst),
            None,
            &[a, b],
            None,
            Some(cond),
            0,
        );
        dst
    }

    /// A load.
    pub fn load(&mut self, ty: Type, addr: Temp, mem: MemOp) -> Temp {
        let dst = self.temp(ty);
        let inst = self.push(Opcode::LD, ty, Some(dst), None, &[addr], None, None, 0);
        self.block.insts[inst].mem = Some(mem);
        dst
    }

    /// A store.
    pub fn store(&mut self, ty: Type, addr: Temp, value: Temp, mem: MemOp) {
        let inst = self.push(Opcode::ST, ty, None, None, &[addr, value], None, None, 0);
        self.block.insts[inst].mem = Some(mem);
    }

    /// Charge guest ticks.
    ///
    /// Emitted where the interpreter charges, because the count is hashed
    /// output rather than a budget (see the module docs).
    pub fn charge(&mut self, ticks: u64) {
        self.push(
            Opcode::CHARGE,
            Type::I64,
            None,
            None,
            &[],
            Some(Const::Int(ticks as u128)),
            None,
            0,
        );
    }

    /// Open a guest instruction boundary.
    pub fn insn_start(&mut self, mark: InsnStart) {
        let index = self.block.marks.len() as u32;
        self.block.marks.push(mark);
        self.push(
            Opcode::INSN_START,
            Type::I64,
            None,
            None,
            &[],
            None,
            None,
            index,
        );
    }

    /// Leave the block, returning to the dispatcher.
    pub fn exit_tb(&mut self) {
        self.push(Opcode::EXIT_TB, Type::I64, None, None, &[], None, None, 0);
    }

    /// Finish, yielding the block.
    #[must_use]
    pub fn finish(self) -> Block {
        self.block
    }

    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        op: Opcode,
        ty: Type,
        dst: Option<Temp>,
        dst2: Option<Temp>,
        srcs: &[Temp],
        imm: Option<Const>,
        cond: Option<Cond>,
        aux: u32,
    ) -> usize {
        let src_start = self.block.operands.len() as u32;
        self.block.operands.extend_from_slice(srcs);
        self.block.insts.push(Inst {
            op,
            ty,
            dst,
            dst2,
            imm,
            mem: None,
            cond,
            aux,
            src_start,
            src_len: srcs.len() as u32,
        });
        self.block.insts.len() - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::value::Width;
    use crate::ir::op::MemOp;
    use alloc::vec;

    fn mark(pc: u64, ticks: u64) -> InsnStart {
        InsnStart {
            pc,
            next_pc: pc + 4,
            ticks,
            live: Vec::new(),
        }
    }

    #[test]
    fn a_block_keeps_its_operands_flat_and_reachable() {
        let mut b = BlockBuilder::new(0x8000_0000, 0);
        b.insn_start(mark(0x8000_0000, 0));
        let a = b.imm(Type::I64, Const::Int(1));
        let c = b.imm(Type::I64, Const::Int(2));
        let sum = b.binary(Opcode::ADD, Type::I64, a, c);
        b.exit_tb();
        let block = b.finish();

        assert_eq!(block.temp_count(), 3);
        assert_eq!(block.type_of(sum), Some(Type::I64));
        // The add's operands come back in order, out of the shared array.
        let add = block
            .insts()
            .iter()
            .position(|i| i.op == Opcode::ADD)
            .expect("the add is in the block");
        assert_eq!(block.srcs(add), &[a, c]);
        // and the immediate moves carry no sources at all.
        assert!(block.srcs(1).is_empty());
    }

    #[test]
    fn carry_producing_ops_get_a_second_result_of_one_bit() {
        let mut b = BlockBuilder::new(0, 0);
        let x = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let y = b.imm(Type::I32, Const::Int(1));
        let carry_in = b.imm(Type::I1, Const::Int(0));
        let (sum, carry_out) = b.addc(Opcode::ADDC, Type::I32, x, y, carry_in);
        let block = b.finish();

        assert_eq!(block.type_of(sum), Some(Type::I32));
        assert_eq!(block.type_of(carry_out), Some(Type::I1));
    }

    #[test]
    fn a_boundary_marker_points_at_its_record() {
        let mut b = BlockBuilder::new(0x100, 0);
        b.insn_start(mark(0x100, 0));
        b.charge(1);
        b.insn_start(mark(0x104, 1));
        b.charge(1);
        let block = b.finish();

        assert_eq!(block.marks().len(), 2);
        assert_eq!(block.marks()[1].pc, 0x104);
        // Ticks are cumulative from block entry, which is what makes them
        // reconstructible at a mid-block fault.
        assert_eq!(block.marks()[1].ticks, 1);
        let starts: Vec<u32> = block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::INSN_START)
            .map(|i| i.aux)
            .collect();
        assert_eq!(starts, vec![0, 1]);
    }

    #[test]
    fn a_load_carries_its_descriptor() {
        let mut b = BlockBuilder::new(0, 0);
        let addr = b.imm(Type::I64, Const::Int(0x2002));
        let mut mem = MemOp::load(Width::U8);
        mem.volatile = true;
        let value = b.load(Type::I32, addr, mem);
        let block = b.finish();

        let ld = block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::LD)
            .expect("the load is in the block");
        assert_eq!(ld.dst, Some(value));
        assert!(ld.mem.expect("a load has a descriptor").volatile);
    }

    #[test]
    fn a_block_dumps_itself_for_a_differential_report() {
        let mut b = BlockBuilder::new(0x80, 0x1);
        b.insn_start(mark(0x80, 0));
        let a = b.imm(Type::I32, Const::Int(5));
        let _ = b.unary(Opcode::NEG, Type::I32, a);
        let text = alloc::format!("{}", b.finish());
        assert!(text.starts_with("block 0x80 key 0x1\n"), "{text}");
        assert!(text.contains("insn_start"), "{text}");
        assert!(text.contains("t1 = neg.i32 t0"), "{text}");
    }
}
