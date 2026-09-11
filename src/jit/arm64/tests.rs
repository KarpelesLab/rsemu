//! What can be tested without an A64 machine, and what cannot.
//!
//! This backend was written on an x86-64 host, which cannot execute a single
//! instruction it emits. That is the whole reason the module is split the way
//! it is, and the whole reason this file is in two halves:
//!
//! * **Everything above `mod executed`** runs anywhere the feature is on. It
//!   asserts the *code*, not its effect: the encodings against DDI 0487 (in
//!   [`emit`](super::emit)'s own tests), the prologue and epilogue against the
//!   frame this backend claims to build, the refusal seam against the ops it
//!   says it does not lower, the register allocator's central invariant against
//!   the emitted call sites, and the guest barrier against the one word it must
//!   be. None of it needs a processor.
//! * **`mod executed`** is the differential against the oracle —
//!   `jit::x86::tests`' harness, adapted — and it can only run on an aarch64
//!   Linux runner. On every other host it is not compiled at all.
//!
//! The split is not a compromise, but it is a *limitation*, and it should be
//! read as one: until the aarch64 job runs `mod executed`, the strongest claim
//! this backend has is "it emits the instructions the manual says it does".
//! Agreement with the interpreter is asserted by code that has never run.

use alloc::vec;
use alloc::vec::Vec;

use crate::core::value::Width;
use crate::ir::{
    Block, BlockBuilder, Const, Home, InsnStart, Liveness, MemOp, Opcode, RegSlot, Type,
};

use super::compile::{Refusal, Regs, compile, compile_with, compiles};

/// Where the test machine's RAM lives.
const BASE: u64 = 0x2000_0000;

/// The words of a compiled block.
fn words(code: &[u8]) -> Vec<u32> {
    assert_eq!(code.len() % 4, 0, "A64 instructions are four bytes each");
    code.as_chunks::<4>()
        .0
        .iter()
        .map(|c| u32::from_le_bytes(*c))
        .collect()
}

/// The smallest well-formed block: one boundary, one charge, one terminator.
fn tiny() -> Block {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let x = b.imm(Type::I64, Const::Int(7));
    let y = b.imm(Type::I64, Const::Int(9));
    let _ = b.binary(Opcode::ADD, Type::I64, x, y);
    b.exit_tb();
    b.finish()
}

#[test]
fn a_block_is_wrapped_in_the_frame_this_backend_claims_to_build() {
    // The prologue and the epilogue are the two sequences no differential can
    // isolate: a mistake in either shows up as a corrupted *caller*, which is
    // a crash somewhere else entirely. So they are asserted as words.
    let code = compile(&tiny()).expect("a tiny block compiles");
    let w = words(code.code());
    // `sub sp, sp, #128`.
    assert_eq!(w[0], 0xd102_03ff, "the frame is opened first");
    // `stp x19, x20, [sp, #32]` — the first of six pairs.
    assert_eq!(w[1], 0xa902_53f3);
    // …and the last is `stp x29, x30, [sp, #112]`, because `x30` is what
    // every `BLR` in the body destroys.
    assert_eq!(w[6], 0xa907_7bfd);
    // `mov x19, x0`: the context, out of the argument register.
    assert_eq!(w[7], 0xaa00_03f3);
    // The tail: `add sp, sp, #128` then `ret`.
    let n = w.len();
    assert_eq!(w[n - 2], 0x9102_03ff);
    assert_eq!(w[n - 1], 0xd65f_03c0);
    // And every callee-saved register the prologue saved is restored.
    assert_eq!(w[n - 8], 0xa942_53f3, "ldp x19, x20, [sp, #32]");
    assert_eq!(w[n - 3], 0xa947_7bfd, "ldp x29, x30, [sp, #112]");
}

#[test]
fn every_op_the_backend_claims_it_lowers_has_a_lowering() {
    // `compiles` is a list, and a list can say yes to something the `match` in
    // `Compiler::inst` falls off the end of — which would be a `Refusal::Op`
    // from inside the compile rather than before it. Every op that reaches
    // this backend at all is one of these, so walking them is walking the
    // contract.
    for op in [
        Opcode::MOV,
        Opcode::GET_SLOT,
        Opcode::EXT_S,
        Opcode::EXT_Z,
        Opcode::TRUNC,
        Opcode::BSWAP,
        Opcode::DEPOSIT,
        Opcode::EXTRACT,
        Opcode::ADD,
        Opcode::SUB,
        Opcode::MUL,
        Opcode::NEG,
        Opcode::AND,
        Opcode::OR,
        Opcode::XOR,
        Opcode::NOT,
        Opcode::ANDC,
        Opcode::SHL,
        Opcode::SHR,
        Opcode::SAR,
        Opcode::ROTL,
        Opcode::ROTR,
        Opcode::CLZ,
        Opcode::CTZ,
        Opcode::POPCOUNT,
        Opcode::MULU2,
        Opcode::MULS2,
        Opcode::SETCOND,
        Opcode::MOVCOND,
        Opcode::BRCOND,
        Opcode::LD,
        Opcode::ST,
        Opcode::FENCE,
        Opcode::GOTO_TB,
        Opcode::EXIT_TB,
        Opcode::LOOKUP_AND_GOTO,
        Opcode::CHARGE,
        Opcode::INSN_START,
    ] {
        assert!(compiles(op), "`{op}` is in the list");
    }
    // And the ones the module docs say are refused really are, because a list
    // that quietly grew would be a backend shipping code generation nothing
    // has executed.
    for op in [
        Opcode::ROTLC,
        Opcode::ROTRC,
        Opcode::DIV_U,
        Opcode::DIV_S,
        Opcode::LD_EXCL,
        Opcode::ST_EXCL,
        Opcode::CALL_HELPER,
        Opcode::PHI,
    ] {
        assert!(!compiles(op), "`{op}` is refused");
    }
}

#[test]
fn an_op_the_backend_does_not_lower_is_refused_and_not_miscompiled() {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    let x = b.imm(Type::I64, Const::Int(0xff));
    let y = b.imm(Type::I64, Const::Int(3));
    let _ = b.binary(Opcode::DIV_U, Type::I64, x, y);
    b.exit_tb();
    let block = b.finish();
    assert_eq!(compile(&block).err(), Some(Refusal::Op(Opcode::DIV_U)));
}

/// The one lowering in this backend that leaves the integer unit.
///
/// Asserted as words for the reason every encoding here is: the functional
/// differential runs on one CI runner and nowhere else, so on every other host
/// this is the only thing between a typo in a `Rn` field and a `popcount` that
/// counts the wrong register. And it is asserted as a *sequence*, because the
/// four instructions are only correct in this order — `CNT` counts each byte
/// separately and it is `ADDV` that makes the eight per-byte counts one
/// number.
#[test]
fn a_popcount_is_the_four_vector_instructions_in_order() {
    fn code_for(ty: Type) -> Vec<u32> {
        let mut b = BlockBuilder::new(BASE, 0);
        b.insn_start(InsnStart {
            pc: BASE,
            next_pc: BASE + 4,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let x = b.imm(ty, Const::Int(0xff));
        let _ = b.unary(Opcode::POPCOUNT, ty, x);
        b.exit_tb();
        words(compile(&b.finish()).expect("popcount compiles").code())
    }
    for ty in [Type::I64, Type::I32, Type::I1] {
        let code = code_for(ty);
        // `fmov d16, Xn` / `cnt v16.8b, v16.8b` / `addv b16, v16.8b` /
        // `fmov Xd, d16`. The register fields of the two `FMOV`s vary with
        // where the allocator put things, so the two fixed words are matched
        // exactly and the moves by their opcode.
        let at = code
            .iter()
            .position(|w| *w == 0x0e20_5a10)
            .unwrap_or_else(|| panic!("`cnt v16.8b, v16.8b` is in the block for {ty:?}"));
        assert!(at > 0, "a `CNT` must have an `FMOV` feeding it");
        assert_eq!(
            code[at - 1] & 0xffff_fc1f,
            0x9e67_0010,
            "`fmov d16, Xn` comes first"
        );
        assert_eq!(code[at + 1], 0x0e31_ba10, "`addv b16, v16.8b` comes next");
        assert_eq!(
            code[at + 2] & 0xffff_ffe0,
            0x9e66_0200,
            "`fmov Xd, d16` takes the answer back"
        );
        // No SWAR left over: the four instructions are the whole lowering, so
        // none of the magic constants can be in the block.
        for magic in [0x5555u16, 0x3333, 0x0f0f, 0x0101] {
            assert!(
                !code.iter().any(|w| (w >> 5) & 0xffff == u32::from(magic)),
                "the vector lowering materialises no mask constant"
            );
        }
    }
}

/// `mulu2`/`muls2` at 64 bits: the high half is one instruction, not a call
/// and not a shift.
#[test]
fn a_widening_multiply_is_a_mulh_and_a_mul() {
    fn code_for(op: Opcode, ty: Type) -> Vec<u32> {
        let mut b = BlockBuilder::new(BASE, 0);
        b.insn_start(InsnStart {
            pc: BASE,
            next_pc: BASE + 4,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let x = b.imm(ty, Const::Int(0xdead_beef));
        let y = b.imm(ty, Const::Int(0x1234_5678));
        let low = b.temp(ty);
        let high = b.temp(ty);
        b.emit_raw(op, ty, Some(low), Some(high), &[x, y], None, None, 0);
        b.exit_tb();
        words(
            compile(&b.finish())
                .expect("a widening multiply compiles")
                .code(),
        )
    }
    // `umulh x11, x9, x10` and `smulh x11, x9, x10`: the accumulator and the
    // second scratch in, the third scratch out.
    let unsigned = code_for(Opcode::MULU2, Type::I64);
    assert!(
        unsigned.contains(&0x9bca_7d2b),
        "`umulh x11, x9, x10` is the unsigned high half"
    );
    let signed = code_for(Opcode::MULS2, Type::I64);
    assert!(
        signed.contains(&0x9b4a_7d2b),
        "`smulh x11, x9, x10` is the signed one"
    );
    assert!(
        !signed.contains(&0x9bca_7d2b),
        "and the two are not the same instruction"
    );
    // Below 64 bits the product fits in one register, so there is no `MULH` at
    // all — the high half is a shift, and a `SMULH` there would be a wrong
    // answer rather than a slow one.
    let narrow = code_for(Opcode::MULU2, Type::I32);
    assert!(
        !narrow.iter().any(|w| w & 0xff60_7c00 == 0x9b40_7c00),
        "a 32-bit widening multiply needs no high-half instruction"
    );
}

#[test]
fn a_backward_branch_is_refused_because_nothing_would_stop_it() {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    let x = b.imm(Type::I1, Const::Int(1));
    let at = b.emit_raw(Opcode::BRCOND, Type::I64, None, None, &[x], None, None, 0);
    b.patch_aux(at, 0);
    b.exit_tb();
    let block = b.finish();
    assert!(matches!(compile(&block), Err(Refusal::Shape(_))));
}

#[test]
fn a_block_with_more_temporaries_than_one_offset_reaches_is_refused() {
    // A64's only load offset is an unsigned 12-bit immediate scaled by the
    // access size, so the frame reaches 4095 temporaries and not one more.
    // x86-64 has a `disp32` and never meets this wall; refusing is what keeps
    // the difference from being a wrong address.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    for i in 0..4100u64 {
        let _ = b.imm(Type::I64, Const::Int(u128::from(i)));
    }
    b.exit_tb();
    let block = b.finish();
    assert!(block.temp_count() > 4095);
    assert!(matches!(compile(&block), Err(Refusal::Shape(_))));
}

/// A block whose only unusual member is `fence`, with a store before it and a
/// load after it — the store-buffer shape, which is the one a barrier exists
/// to constrain.
fn a_block_with_a_barrier(fence: bool) -> Block {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let addr = b.imm(Type::I64, Const::Int(u128::from(BASE)));
    let value = b.imm(Type::I64, Const::Int(0x1234));
    b.store(Type::I64, addr, value, MemOp::store(Width::U64));
    if fence {
        b.emit_raw(Opcode::FENCE, Type::I64, None, None, &[], None, None, 0);
    }
    let other = b.imm(Type::I64, Const::Int(u128::from(BASE + 8)));
    let _ = b.load(Type::I64, other, MemOp::load(Width::U64));
    b.exit_tb();
    b.finish()
}

#[test]
fn a_guest_barrier_is_one_dmb_ish_and_not_a_call() {
    // The lowering that cannot be checked by running anything on an x86-64
    // host and that is *most* worth checking: `DMB ISHST` and `DMB ISHLD`
    // differ from `DMB ISH` in one nibble, are both weaker, and would pass
    // every functional test on a machine that happens not to reorder. So the
    // word is asserted, and its absence from the unfenced block with it.
    const DMB_ISH: u32 = 0xd503_3bbf;
    let with = compile(&a_block_with_a_barrier(true)).expect("it compiles");
    let without = compile(&a_block_with_a_barrier(false)).expect("it compiles");
    let count = |b: &[u8]| words(b).into_iter().filter(|w| *w == DMB_ISH).count();
    assert_eq!(count(with.code()), 1, "exactly one barrier, inline");
    assert_eq!(count(without.code()), 0, "and none without a fence");
    // Not a `DSB`, which would also be correct and is strictly stronger.
    assert_eq!(
        words(with.code())
            .into_iter()
            .filter(|w| *w == 0xd503_3b9f)
            .count(),
        0,
        "a guest fence orders; it does not wait for completion"
    );
}

#[test]
fn a_guest_instructions_bookkeeping_costs_no_code_at_all() {
    // `charge` and `insn_start` are replayed by the flush thunk, so adding
    // more of them to a block must not add a single instruction. The x86
    // backend asserts the same thing in bytes; here it is words, which is the
    // same statement on a fixed-width instruction set.
    fn size(insns: u64) -> usize {
        let mut b = BlockBuilder::new(BASE, 0);
        for i in 0..insns {
            b.insn_start(InsnStart {
                pc: BASE + i * 4,
                next_pc: BASE + i * 4 + 4,
                ticks: i,
                live: Vec::new(),
            });
            b.charge(1);
        }
        b.exit_tb();
        compile(&b.finish()).expect("it compiles").code().len()
    }
    assert_eq!(size(1), size(64), "a guest instruction costs nothing");
}

/// Whether the backend's lowering of `op` calls into the host *after* reading
/// its operands and before writing its results.
///
/// Written out here rather than read off `compile::calls_inside`, and the
/// duplication is the point: a single shared constant would make the test and
/// the compiler agree with a wrong answer.
fn a_call_site(op: Opcode) -> bool {
    matches!(op, Opcode::LD | Opcode::ST | Opcode::GET_SLOT)
}

/// Where the backend replays a region's deferred bookkeeping, worked out again
/// rather than read off `compile::plan`.
///
/// Deliberately the *other* formulation: `plan` carries the region start
/// forward in one pass, and this searches backwards for it per instruction.
fn a_flush_before(block: &Block) -> Vec<bool> {
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
    let boundary = |i: usize| {
        let op = insts[i].op;
        target[i]
            || a_call_site(op)
            || op == Opcode::FENCE
            || op == Opcode::BRCOND
            || op.is_terminator()
    };
    let mut out = vec![false; n];
    for i in 0..n {
        if !boundary(i) {
            continue;
        }
        // The region this instruction closes starts at the previous boundary,
        // and is flushed only if it holds an event.
        let mut start = 0;
        for j in (0..i).rev() {
            if boundary(j) {
                start = j;
                break;
            }
        }
        out[i] = insts[start..i]
            .iter()
            .any(|inst| matches!(inst.op, Opcode::CHARGE | Opcode::INSN_START));
    }
    out
}

#[test]
fn no_value_that_outlives_a_call_is_left_where_a_call_can_destroy_it() {
    // The register allocator's central invariant, asserted on a block with
    // more live values than there are registers. It is host-agnostic — the
    // allocator is shared — but the *banks* are this backend's, and a bank
    // that named a volatile register as saved would be a value silently
    // destroyed by the first thunk call.
    let block = a_block_that_holds_many_values_across_calls();
    let compiled = compile_with(&block, Regs::Scan).expect("it compiles");
    let live = Liveness::compute(&block);
    let flushes = a_flush_before(&block);
    let volatile = super::compile::VOLATILE;
    let intervals = live.intervals();
    for (i, inst) in block.insts().iter().enumerate() {
        if !a_call_site(inst.op) && !flushes[i] {
            continue;
        }
        let i = i as u32;
        for (temp, from, to) in &intervals {
            // A value defined before this instruction and read after it spans
            // the call this instruction makes.
            let spans = if a_call_site(inst.op) {
                *from < i && *to > i
            } else {
                // A flush runs in the *gap* ahead of the instruction, so a
                // value read by the instruction itself also spans it.
                *from < i && *to >= i
            };
            if !spans {
                continue;
            }
            if let Home::Reg(n) = compiled.home(*temp) {
                assert!(
                    !volatile.contains(&n),
                    "{temp} spans the call at {i} and lives in x{n}, which a call destroys"
                );
            }
        }
    }
}

/// A block with more simultaneously live values than the allocator has
/// registers, and calls in the middle of them.
fn a_block_that_holds_many_values_across_calls() -> Block {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let mut held = Vec::new();
    for i in 0..24u64 {
        held.push(b.imm(Type::I64, Const::Int(u128::from(i * 0x1111 + 1))));
    }
    // A slot read is a call, and it sits in the middle of every one of those
    // intervals.
    for slot in 0..4u16 {
        let _ = b.get_slot(Type::I64, RegSlot(slot));
        b.charge(1);
    }
    // Now read them all again, so every one of them is live across the calls.
    let mut sum = held[0];
    for t in &held[1..] {
        sum = b.binary(Opcode::ADD, Type::I64, sum, *t);
    }
    b.emit_raw(
        Opcode::LOOKUP_AND_GOTO,
        Type::I64,
        None,
        None,
        &[sum],
        None,
        None,
        0,
    );
    b.finish()
}

#[test]
fn the_allocator_actually_places_values_in_registers() {
    // The banks are handed to `linear_scan` by number, and a bank that was
    // empty or wrong would produce a correct backend with every temporary in
    // the frame — slower, and silently so.
    let block = a_block_that_holds_many_values_across_calls();
    let compiled = compile_with(&block, Regs::Scan).expect("it compiles");
    assert!(
        compiled.in_registers() > 4,
        "the allocator placed only {} of {} temporaries",
        compiled.in_registers(),
        block.temp_count()
    );
    let control = compile_with(&block, Regs::Frame).expect("it compiles");
    assert_eq!(control.in_registers(), 0, "the control keeps none");
    assert!(
        compiled.code().len() < control.code().len(),
        "the allocated block is not smaller than the control"
    );
}

#[test]
fn a_memory_op_carries_exactly_one_descriptor() {
    let block = a_block_with_a_barrier(true);
    let compiled = compile(&block).expect("it compiles");
    assert_eq!(compiled.mem_count(), 2, "one load and one store");
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
mod executed;
