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

#![cfg(feature = "cpu-m68k-lift")]

use rsemu::cpu::m68k::differential::{opcode_sweep, opcode_sweep_in, sweep};

#[test]
fn every_opcode_word_agrees_with_the_interpreter() {
    let (cases, found) = opcode_sweep(1, &[0x0010, 0x0000, 0x2400]);
    assert_eq!(cases, 65_536, "the whole decode space must have run");
    assert!(
        found.is_none(),
        "{cases} cases, 1 disagreement:\n{}",
        found.unwrap()
    );
    println!("{cases} cases, 0 disagreements");
}

#[test]
fn every_opcode_word_agrees_with_a_second_set_of_extension_words() {
    // A negative displacement and an index register that is long rather than
    // word, so the modes that compute an address land somewhere else — and
    // often outside the case's RAM, which is the fault path.
    let (cases, found) = opcode_sweep(1, &[0xf80c, 0xffff, 0xfffe]);
    assert_eq!(cases, 65_536, "the whole decode space must have run");
    assert!(
        found.is_none(),
        "{cases} cases, 1 disagreement:\n{}",
        found.unwrap()
    );
    println!("{cases} cases, 0 disagreements");
}

#[test]
fn every_opcode_word_agrees_in_user_state_too() {
    // The same decode space with **S** clear. Nothing in the lifted subset is
    // privileged, so this is a claim about the *fallback*: a privileged
    // encoding is a privilege violation here rather than an instruction, `A7`
    // is the user stack pointer, and an exception switches banks on the way
    // in. Every one of those has to come out the same on both engines.
    let (cases, found) =
        opcode_sweep_in(1, &[0x0010, 0x0000, 0x2400], rsemu::cpu::m68k::flags::IPL);
    assert_eq!(cases, 65_536, "the whole decode space must have run");
    assert!(
        found.is_none(),
        "{cases} cases, 1 disagreement:\n{}",
        found.unwrap()
    );
    println!("{cases} cases, 0 disagreements");
}

#[test]
fn a_long_seeded_random_sweep_agrees_with_the_interpreter() {
    // Eight seeds so a failure names one, and so "the sweep passed" is not a
    // claim about one arbitrary sequence.
    let mut total = 0usize;
    for seed in 1..=8u64 {
        let (cases, found) = sweep(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15), 750, 6);
        total += cases;
        assert!(
            found.is_none(),
            "seed {seed}: {cases} cases, 1 disagreement:\n{}",
            found.unwrap()
        );
    }
    println!("{total} cases over 8 seeds, 0 disagreements");
}
