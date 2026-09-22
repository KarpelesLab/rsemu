//! The m68k IR frontend against the m68k interpreter, at length.
//!
//! `cpu::m68k::differential`'s own tests take a slice of each sweep so
//! `cargo test` stays quick; this is where the whole of them run. CLAUDE.md,
//! "CPU cores": *the interpreter is the oracle*, and a frontend is tested
//! against it **forever** — so the corpus that matters is not a list of cases
//! somebody thought of, it is every encoding there is.
//!
//! Three sweeps:
//!
//! 1. **Every sixteen-bit opcode word**, with extension words that make an
//!    encoding which wants them well formed — 65 536 cases. A few hundred of
//!    those words lift and the rest exercise the fallback, and nothing is
//!    skipped: an encoding the frontend declines still has to leave the guest
//!    in the state the interpreter leaves it in, and an illegal one is
//!    compared through its exception.
//! 2. **The same, with a second set of extension words**, because a mode's
//!    displacement decides where an access lands and therefore whether it
//!    faults — and **the same again in user state**, where a privileged
//!    encoding is a privilege violation rather than an instruction and an
//!    exception switches stack-pointer banks on the way in.
//! 3. **A seeded pseudo-random stream** of multi-instruction programs, which
//!    is where a *sequence* is tested: a register the previous instruction
//!    left in a temporary, an `ADDX` chain reading the **X** its predecessor
//!    wrote, a branch into the middle of a block.
//!
//! Every result is reported as a **rate** — "N cases, 0 disagreements" —
//! because a probabilistic test reported as "green" is not a measurement.

#![cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]

use rsemu::cpu::m68k::Engine;
use rsemu::cpu::m68k::differential::{opcode_sweep, opcode_sweep_in, sweep};

/// The translated engines this file sweeps, every sweep, both of them.
///
/// `jit-host` is not a *different* engine from `jit` but a different **mix**:
/// a block its code generator refuses runs on the portable backend beside it,
/// so the two share an executor and differ in which instructions reach the
/// other one. Sweeping only `jit` would leave the generated code unswept, and
/// sweeping only `jit-host` would leave whatever it refuses unswept on a host
/// that has a backend at all. On a build or a host with none, the two are the
/// same run twice, which is what `ROADMAP.md` §9's fallback is supposed to
/// look like.
const ENGINES: [Engine; 2] = [Engine::Jit, Engine::JitHost];

#[test]
fn every_opcode_word_agrees_with_the_interpreter() {
    for engine in ENGINES {
        let (cases, found) = opcode_sweep(engine, 1, &[0x0010, 0x0000, 0x2400]);
        assert_eq!(cases, 65_536, "the whole decode space must have run");
        assert!(
            found.is_none(),
            "{engine:?}: {cases} cases, 1 disagreement:\n{}",
            found.unwrap()
        );
        println!("{engine:?}: {cases} cases, 0 disagreements");
    }
}

#[test]
fn every_opcode_word_agrees_with_a_second_set_of_extension_words() {
    // A negative displacement and an index register that is long rather than
    // word, so the modes that compute an address land somewhere else — and
    // often outside the case's RAM, which is the fault path.
    for engine in ENGINES {
        let (cases, found) = opcode_sweep(engine, 1, &[0xf80c, 0xffff, 0xfffe]);
        assert_eq!(cases, 65_536, "the whole decode space must have run");
        assert!(
            found.is_none(),
            "{engine:?}: {cases} cases, 1 disagreement:\n{}",
            found.unwrap()
        );
        println!("{engine:?}: {cases} cases, 0 disagreements");
    }
}

#[test]
fn every_opcode_word_agrees_in_user_state_too() {
    // The same decode space with **S** clear. Nothing in the lifted subset is
    // privileged, so this is a claim about the *fallback*: a privileged
    // encoding is a privilege violation here rather than an instruction, `A7`
    // is the user stack pointer, and an exception switches banks on the way
    // in. Every one of those has to come out the same on both engines.
    for engine in ENGINES {
        let (cases, found) = opcode_sweep_in(
            engine,
            1,
            &[0x0010, 0x0000, 0x2400],
            rsemu::cpu::m68k::flags::IPL,
        );
        assert_eq!(cases, 65_536, "the whole decode space must have run");
        assert!(
            found.is_none(),
            "{engine:?}: {cases} cases, 1 disagreement:\n{}",
            found.unwrap()
        );
        println!("{engine:?}: {cases} cases, 0 disagreements");
    }
}

/// How many seeds and how many programs per seed the random sweep runs.
///
/// The defaults are what `cargo test` can afford. They are also **smaller than
/// the size that found the last defect**: the prefetch-queue bug fixed in
/// `d3d8b1c4` was green over 6,000 cases and failed over 60,000, so a sweep of
/// this size passing is evidence of very little on its own. The nightly
/// `long-run` workflow raises both, which is where a sweep long enough to mean
/// something belongs — not in a gate every commit has to wait for.
fn sweep_size() -> (u64, usize) {
    let get = |name: &str, default: u64| {
        std::env::var(name)
            .ok()
            .and_then(|v| v.parse::<u64>().ok())
            .filter(|&v| v > 0)
            .unwrap_or(default)
    };
    (
        get("RSEMU_M68K_DIFF_SEEDS", 8),
        get("RSEMU_M68K_DIFF_PROGRAMS", 750) as usize,
    )
}

#[test]
fn a_long_seeded_random_sweep_agrees_with_the_interpreter() {
    // Several seeds rather than one, so a failure names the seed that found it
    // and so "the sweep passed" is not a claim about one arbitrary sequence.
    let (seeds, programs) = sweep_size();
    for engine in ENGINES {
        let mut total = 0usize;
        for seed in 1..=seeds {
            let (cases, found) = sweep(
                engine,
                seed.wrapping_mul(0x9e37_79b9_7f4a_7c15),
                programs,
                6,
            );
            total += cases;
            assert!(
                found.is_none(),
                "{engine:?}, seed {seed}: {cases} cases, 1 disagreement:\n{}",
                found.unwrap()
            );
        }
        println!("{engine:?}: {total} cases over {seeds} seeds, 0 disagreements");
    }
}
