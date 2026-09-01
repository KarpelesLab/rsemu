#![no_main]
//! The RISC-V IR frontend, against the RISC-V interpreter.
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
//! The comparison itself is `cpu::riscv::differential::compare`, shared with
//! the offline test in `tests/riscv_lift_differential.rs` so that a case found
//! here can be pasted straight into a regression. It runs one program through
//! both engines and compares the integer register file, the PC, the tick
//! count, the block's own static tick column, guest memory, and whether the
//! two agreed about faulting. A divergence in any of them is a frontend bug —
//! or, much more interestingly, an interpreter bug, and then the manual
//! decides which.
//!
//! # Input encoding
//!
//! A four-byte header, then five bytes per instruction:
//!
//! ```text
//!   header  cc ll xx xx    core, program length, and a spare
//!   insn    ff dd dd dd dd form selector, then the fields
//! ```
//!
//! `form` and `fields` go straight to
//! `differential::synthesize`, which turns any pair of numbers into an
//! encoding *inside the lifted subset*. That bias is the whole point: a
//! uniformly random 32-bit word is almost never one of the ninety-odd
//! encodings this frontend lifts, so an unbiased target would spend its budget
//! proving that `lift` stops cleanly at an unsupported opcode — which the unit
//! tests already assert — and would never compare two engines at all.
//!
//! The generator is still able to produce a block that lifts nothing: the
//! selector reaches `JALR` on a core without `C`, and shift and branch forms
//! reach their out-of-subset cases. Those come back as
//! `Verdict::Nothing` and are counted rather than rejected.
//!
//! Decoded by hand rather than through `arbitrary`'s derive, for the reason
//! `state_roundtrip` gives: a dependency bump must not reinterpret every
//! committed seed.
//!
//! # The three cores
//!
//! The header's first byte picks between a bare RV64I hart, one that traps
//! misaligned accesses, and one with `C`. All three are separate frontends in
//! everything that matters — the misalignment policy is in every memory op's
//! `Align` and in the block's cache key, and `C` decides both whether a
//! 16-bit halfword is an instruction and whether `JALR` is in the subset at
//! all.

use libfuzzer_sys::fuzz_target;

use rsemu::cpu::riscv::Config;
use rsemu::cpu::riscv::csr::Extensions;
use rsemu::cpu::riscv::differential::{Case, compare, compare_cached, synthesize};

/// How many instructions one input may describe.
///
/// A block is bounded by `lift::MAX_INSNS` at 64, so a little more than that
/// is enough to reach the limit without the oracle's per-case cost growing for
/// nothing.
const MAX_INSNS: usize = 80;

/// The three cores, by header byte.
fn core(selector: u8) -> Config {
    match selector % 3 {
        0 => Config::rv64i(),
        1 => {
            let mut cfg = Config::rv64i();
            cfg.misaligned = false;
            cfg
        }
        _ => {
            let mut cfg = Config::rv64i();
            cfg.ext = Extensions {
                c: true,
                ..Extensions::I
            };
            cfg
        }
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let cfg = core(data[0]);
    let want = 1 + (data[1] as usize) % MAX_INSNS;

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

    let case = Case::seeded(program).with_config(cfg);
    if let Err(divergence) = compare(&case) {
        panic!("the lifter and the interpreter disagree:\n{divergence}");
    }
    // The same program again, through the translation runtime: many blocks
    // rather than one, served from a cache, chained exit to exit, invalidated
    // by the guest's own stores, and every access resolved through a software
    // TLB. `compare` is blind to all of that — a single block is never served
    // twice — so a corpus entry that is interesting for the frontend is
    // interesting for the runtime too, and it costs one more run of bytes the
    // fuzzer has already found.
    if let Err(divergence) = compare_cached(&case, 32) {
        panic!("the cached path and the interpreter disagree:\n{divergence}");
    }
});
