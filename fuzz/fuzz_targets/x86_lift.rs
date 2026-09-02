#![no_main]
//! The x86 IR frontend, against the x86 interpreter.
//!
//! `riscv_lift` asks the same question for the first frontend; this target
//! asks it for the second, where the two things RISC-V could not exercise
//! live: **flags**, six of which nearly every arithmetic instruction writes
//! and almost nothing reads, and **self-modifying code**, which x86 makes
//! architectural rather than optional.
//!
//! CLAUDE.md, "CPU cores", is the rule being enforced:
//!
//! > Each core ships an interpreter first; the IR frontend comes later and is
//! > differentially tested against the interpreter forever. **The interpreter
//! > is the oracle.**
//!
//! The comparison itself is `cpu::x86::differential`, shared with the offline
//! test in `tests/x86_lift_differential.rs` so that a case found here can be
//! pasted straight into a regression. It runs one program through both engines
//! and compares the eight general registers, `EIP`, **`EFLAGS` whole**, the
//! tick count, the block's own static tick column, guest memory byte for byte,
//! and whether the two agreed about faulting — and when they faulted, the
//! architectural state at the fault. A divergence in any of them is a frontend
//! bug, or, much more interestingly, an interpreter bug, and then the manual
//! decides which.
//!
//! # Input encoding
//!
//! A four-byte header, then five bytes per instruction:
//!
//! ```text
//!   header  pp ll rr xx    policies, program length, initial registers, spare
//!   insn    ff dd dd dd dd form selector, then the fields
//! ```
//!
//! `form` and `fields` go straight to `differential::synthesize`, which turns
//! any pair of numbers into an encoding *inside the lifted subset*. That bias
//! is the whole point: a uniformly random byte is almost never the start of one
//! of the hundred-odd encodings this frontend lifts, so an unbiased target
//! would spend its budget proving that `lift` stops cleanly at an unsupported
//! opcode — which the unit tests already assert — and would never compare two
//! engines at all.
//!
//! Decoded by hand rather than through `arbitrary`'s derive, for the reason
//! `state_roundtrip` gives: a dependency bump must not reinterpret every
//! committed seed.
//!
//! # Twelve frontends
//!
//! The header's first byte picks a [`Shape`], a [`Smc`] policy and a [`Flags`]
//! policy, and whether the run goes through `compare` or through
//! `compare_cached`. All of them are separate frontends in everything that
//! matters — every one is in the block's cache key, and each emits different IR
//! from the same bytes.

use libfuzzer_sys::fuzz_target;

use rsemu::cpu::x86::differential::{Case, compare, compare_cached, synthesize};
use rsemu::cpu::x86::lift::{Flags, Shape, Smc};

/// Bytes per synthesized instruction in the input encoding.
const STRIDE: usize = 5;
/// The header, ahead of the instruction stream.
const HEADER: usize = 4;
/// The most instructions one case holds.
///
/// Well past the point where a longer program finds anything a shorter one does
/// not: a block is bounded by `lift::MAX_INSNS` anyway, and a case's program
/// has to fit in the first page.
const MAX_INSNS: usize = 40;

fuzz_target!(|data: &[u8]| {
    if data.len() < HEADER + STRIDE {
        return;
    }
    let policies = data[0];
    let shape = match policies % 3 {
        0 => Shape::BasicBlock,
        1 => Shape::Extended,
        _ => Shape::Trace,
    };
    let smc = if policies & 0x04 == 0 {
        Smc::EndBlock
    } else {
        Smc::Guard
    };
    let flags = if policies & 0x08 == 0 {
        Flags::Eager
    } else {
        Flags::Elide
    };
    let cached = policies & 0x10 != 0;

    let available = (data.len() - HEADER) / STRIDE;
    let want = 1 + usize::from(data[1]) % MAX_INSNS;
    let count = want.min(available);

    let mut program = Vec::new();
    for i in 0..count {
        let at = HEADER + i * STRIDE;
        let form = u32::from(data[at]);
        let fields = u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
        let insn = synthesize(form, fields);
        // A case's program lives in the first page, which the harness asserts
        // rather than tolerates.
        if program.len() + insn.len() >= 4000 {
            break;
        }
        program.extend_from_slice(&insn);
    }
    if program.is_empty() {
        return;
    }
    // `HLT` is outside the subset, so a run that falls off the end of the
    // program stops cleanly instead of executing the data window.
    program.push(0xf4);

    let mut case = Case::seeded(program).with_shape(shape).with_smc(smc).with_flags(flags);
    // A third of the register file comes from the input, so a generated program
    // meets values it did not compute — the boundary conditions of every flag
    // live at the edges of a register, not in the middle.
    case.regs[5] = u32::from(data[2]) << 24 | u32::from(data[3]);
    case.regs[6] = case.regs[5].rotate_left(11);
    case.regs[7] = !case.regs[5];

    let outcome = if cached {
        compare_cached(&case, 24)
    } else {
        compare(&case)
    };
    if let Err(divergence) = outcome {
        panic!("{divergence}");
    }
});
