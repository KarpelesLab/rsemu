#![no_main]
//! The AArch64 IR frontend, against the AArch64 interpreter.
//!
//! `ir_verify` fuzzes the *verifier* — it asks whether a malformed block is
//! rejected rather than miscompiled. This target asks the other question, the
//! one no amount of verification answers: given guest bytes, does the block
//! this frontend built **mean the same thing** the interpreter does?
//!
//! CLAUDE.md, "CPU cores", is the rule being enforced:
//!
//! > Each core ships an interpreter first; the IR frontend comes later and is
//! > differentially tested against the interpreter forever. **The interpreter
//! > is the oracle.**
//!
//! The comparison itself is `cpu::arm::a64::differential::compare`, shared
//! with the offline sweep in `tests/a64_lift_differential.rs` so that a case
//! found here can be pasted straight into a regression. It runs one program
//! through both engines and compares every general register, the **stack
//! pointer**, all four `PSTATE` flags, the PC, the tick count, the block's own
//! static tick column, guest memory, and whether the two agreed about
//! faulting.
//!
//! # What this frontend adds to what the other two fuzz
//!
//! The **flags**, and they are the reason this target is not redundant with
//! `x86_lift`. A64 has no flag-elision policy to get wrong, but it has the
//! opposite hazard: `CMP` *is* `SUBS`, so the carry rule — set when there was
//! **no** borrow — is on the path of every conditional branch in every
//! program, and a lifter that inverted it would still pass any test that only
//! read registers. And the **stack pointer** is a slot rather than a register
//! here, spelled by the same encoding that spells `XZR` elsewhere, which is
//! the operand rule DDI 0487 C1.2.5 exists for and which nothing else in the
//! tree exercises.
//!
//! # Input encoding
//!
//! A four-byte header, then five bytes per instruction:
//!
//! ```text
//!   header  cc ll ss xx    core, program length, policy, and a spare
//!   insn    ff dd dd dd dd form selector, then the fields
//! ```
//!
//! `form` and `fields` go straight to `differential::synthesize`, which turns
//! any pair of numbers into an encoding *inside the lifted subset*. That bias
//! is the whole point: a uniformly random 32-bit word is almost never one of
//! the encodings this frontend lifts, so an unbiased target would spend its
//! budget proving that `lift` stops cleanly at an unsupported opcode — which
//! the unit tests already assert — and would never compare two engines at all.
//! The generator still reaches encodings the architecture leaves `UNDEFINED`,
//! which come back as `Verdict::Nothing` and are counted rather than rejected;
//! that half is under test too, because the frontend must reject exactly what
//! the interpreter rejects.
//!
//! Decoded by hand rather than through `arbitrary`'s derive, for the reason
//! `state_roundtrip` gives: a dependency bump must not reinterpret every
//! committed seed.
//!
//! # The worlds
//!
//! The header's first byte picks the part — a Cortex-A53, which has no
//! `FEAT_LSE`, against a Neoverse N1, which does, so the same bytes are an
//! atomic on one and `UNDEFINED` on the other and the two end a block in
//! different places. Its third byte picks `SCTLR_EL1.A`, which decides whether
//! an unaligned access splits into bytes or faults — a different `Align` in
//! every memory op, a different cache key and a different tick count on every
//! access — and the block shape, because all three must agree with the
//! interpreter and a disagreement between two of them is a frontend bug
//! wherever it shows up.

use libfuzzer_sys::fuzz_target;

use rsemu::cpu::arm::a64::Config;
use rsemu::cpu::arm::a64::differential::{Case, compare, compare_cached, synthesize};
use rsemu::cpu::arm::a64::lift::Shape;

/// How many instructions one input may describe.
///
/// A block is bounded by `lift::MAX_INSNS` at 64, so a little more than that
/// is enough to reach the limit without the oracle's per-case cost growing for
/// nothing.
const MAX_INSNS: usize = 80;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let cfg = if data[0] % 2 == 0 {
        Config::cortex_a53()
    } else {
        Config::neoverse_n1()
    };
    let want = 1 + (data[1] as usize) % MAX_INSNS;
    let policy = data[2];
    let shape = match policy % 3 {
        0 => Shape::BasicBlock,
        1 => Shape::Extended,
        _ => Shape::Trace,
    };

    let body = &data[4..];
    let mut program = Vec::with_capacity(want);
    for n in 0..want {
        // Total by construction: a truncated input yields zeros and a shorter
        // program rather than a rejected one, so every corpus entry is
        // productive from its first byte.
        let at = n * 5;
        let byte = |k: usize| body.get(at + k).copied().unwrap_or(0);
        let fields = u32::from_le_bytes([byte(1), byte(2), byte(3), byte(4)]);
        program.push(synthesize(u32::from(byte(0)), fields));
    }

    let mut case = Case::seeded(program).with_config(cfg).with_shape(shape);
    if policy & 0x80 != 0 {
        case = case.strict();
    }
    if let Err(divergence) = compare(&case) {
        panic!("the lifter and the interpreter disagree:\n{divergence}");
    }
    // The same program again, through the translation runtime: many blocks
    // rather than one, served from a cache, chained exit to exit, and
    // invalidated by the guest's own stores. `compare` is blind to all of that
    // — a single block is never served twice — so a corpus entry that is
    // interesting for the frontend is interesting for the runtime too, and it
    // costs one more run of bytes the fuzzer has already found.
    if let Err(divergence) = compare_cached(&case, 32) {
        panic!("the cached path and the interpreter disagree:\n{divergence}");
    }
});
