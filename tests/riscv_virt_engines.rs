//! `riscv-virt` under both execution engines, and the claim that they are the
//! same machine.
//!
//! `ROADMAP.md` §0: *"a bit-identical state hash … across the interpreter and
//! the JIT for the same guest"*. Every test in `src/jit` and every differential
//! harness in `src/cpu/riscv` asserts something one level below that — a block,
//! a register, a tick — because until `engine = "jit"` reached the dispatcher
//! there was no machine to hash. There is now, and this is the assertion the
//! roadmap actually wrote down:
//!
//! * The same board, the same firmware, the same number of quanta, under
//!   `engine = "interp"`, `engine = "jit"` and `engine = "jit-host"`, hashes to
//!   the same number at every checkpoint — not at the end only, because a
//!   divergence that shows up at the last checkpoint is a different bug from
//!   one that shows up at the first.
//! * A snapshot taken under one engine restores under the other and carries on
//!   to the same hash. That is half of phase 7's gate, and it is a property of
//!   the snapshot rather than of the engines: nothing engine-specific is in it.
//!
//! The firmware is a bare RV64I loop rather than a kernel, for the reason
//! `tests/workload` gives: it runs in CI on every commit, so it has to be cheap
//! and it has to need no downloaded image. Every instruction in it is inside
//! the lifted subset except the `jalr` that closes the loop, so the JIT really
//! does translate, cache and re-serve blocks here — asserted, not assumed.

#![cfg(all(
    feature = "machine-riscv-virt",
    feature = "cpu-riscv-lift",
    feature = "jit"
))]

use rsemu::machine::{Machine, catalog};

/// An RV64I loop for the `virt` board's firmware slot, loaded at `0x8000_0000`.
///
/// The same program `tests/workload/mod.rs` boots this board with: add, store,
/// load and two shifts per iteration, closed by a `jalr` through a register the
/// first two instructions compute. Source: *The RISC-V Instruction Set Manual,
/// Volume I*, chapter 2.
const PROGRAM: [u32; 12] = [
    0x0000_0f17, // auipc t5, 0        t5 = 0x80000000
    0x014f_0f13, // addi  t5, t5, 20   t5 = loop
    0x0000_1397, // auipc t2, 1        t2 = 0x80001008, a scratch word in DRAM
    0x0000_0293, // addi  t0, x0, 0
    0x0010_0313, // addi  t1, x0, 1
    0x0062_82b3, // loop: add t0, t0, t1
    0x0053_b023, // sd    t0, 0(t2)
    0x0003_be03, // ld    t3, 0(t2)
    0x003e_1e93, // slli  t4, t3, 3
    0x003e_de93, // srli  t4, t4, 3
    0x01d2_82b3, // add   t0, t0, t4
    0x000f_0067, // jalr  x0, 0(t5)
];

/// Build `riscv-virt` with [`PROGRAM`] as its firmware and `engine` as its
/// hart's execution engine.
///
/// 16 MiB of DRAM rather than the board's 128: `state_hash` walks every byte of
/// RAM and this test takes one every few quanta, so sizing the board to the
/// workload is what makes the comparison cheap enough to run on every commit.
fn board(engine: &str, tag: &str) -> Machine {
    let firmware: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
    let entry = catalog::machine("riscv-virt").expect("this build ships riscv-virt");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options
        .realize
        .media
        .insert("firmware", firmware.as_slice());
    for slot in ["flash0", "flash1", "disk", "initrd"] {
        options.realize.media.insert(slot, &[][..]);
    }
    options
        .resolve
        .params
        .push((String::from("ram"), String::from("16M")));
    options
        .resolve
        .params
        .push((String::from("engine"), String::from(engine)));
    // Two of these run in one process, so they must not share a console port or
    // a power signal with each other.
    options
        .resolve
        .params
        .push((String::from("console"), format!("test.engines.{tag}")));
    options
        .resolve
        .params
        .push((String::from("power"), format!("test.engines.{tag}")));
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("riscv-virt does not build with engine={engine}: {e}"))
}

/// Run `quanta` quanta, hashing every `every`.
fn hashes(machine: &mut Machine, quanta: usize, every: usize) -> Vec<u64> {
    let mut out = Vec::new();
    for n in 1..=quanta {
        machine.run_quantum().expect("the machine advances");
        if n % every == 0 {
            out.push(
                machine
                    .state_hash()
                    .expect("a deterministic machine hashes"),
            );
        }
    }
    out
}

#[test]
fn every_engine_hashes_to_the_same_machine_at_every_checkpoint() {
    let mut interp = board("interp", "hash.interp");
    let want = hashes(&mut interp, 40, 4);
    assert_eq!(
        want.len(),
        10,
        "the checkpoint arithmetic changed and the test stopped checking"
    );
    for engine in ["jit", "jit-host"] {
        let mut board = board(engine, &format!("hash.{engine}"));
        let got = hashes(&mut board, 40, 4);
        for (n, (want, got)) in want.iter().zip(&got).enumerate() {
            assert_eq!(
                want, got,
                "checkpoint {n}: `engine = \"interp\"` hashes to {want:#018x} and \
                 `engine = \"{engine}\"` to {got:#018x}. A cache hit, a cache miss, an \
                 interpreted run and a compiled run must be indistinguishable to the \
                 guest, including cycle counts (ROADMAP.md §0)"
            );
        }
    }
}

#[test]
fn the_engine_property_is_read_rather_than_accepted_and_ignored() {
    // The comparison above passes trivially if `engine` reaches nothing, which
    // is exactly the state this work found the crate in — so the property has
    // to be shown to be *load-bearing*. Two halves: a value nothing implements
    // is refused, and the two values that are implemented build different
    // harts. (That the JIT board really executes translated blocks is asserted
    // one level down, where the count is reachable:
    // `cpu::riscv::engine::tests::a_translated_hart_and_an_interpreted_one_agree_on_every_column`.)
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let mut options = catalog::build_options().expect("catalog");
    let firmware: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
    options
        .realize
        .media
        .insert("firmware", firmware.as_slice());
    for slot in ["flash0", "flash1", "disk", "initrd"] {
        options.realize.media.insert(slot, &[][..]);
    }
    options
        .resolve
        .params
        .push((String::from("engine"), String::from("tier1")));
    let registry = catalog::registry().expect("a registry");
    let err = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("an engine nothing implements must be refused, not ignored");
    let text = err.to_string();
    assert!(
        text.contains("engine") && text.contains("interp") && text.contains("jit"),
        "the refusal has to name the property and what it will take: {text}"
    );

    // And the two that are implemented reach different backends, which the
    // hashes above cannot show because they are supposed to be equal.
    let mut plain = board("jit", "prop.jit");
    let mut host = board("jit-host", "prop.host");
    for _ in 0..8 {
        plain.run_quantum().expect("advances");
        host.run_quantum().expect("advances");
    }
    assert_eq!(
        plain.state_hash().expect("hashes"),
        host.state_hash().expect("hashes"),
        "the two JIT engines are the same guest and a different backend"
    );
}

#[test]
fn a_snapshot_crosses_from_one_engine_to_the_other_and_carries_on_the_same() {
    // Phase 7's gate, minus the accelerator: a snapshot is a property of the
    // guest, not of the engine that produced it, so it must move both ways and
    // the run afterwards must be the same run.
    let mut interp = board("interp", "snap.interp");
    let mut jit = board("jit-host", "snap.jit");
    for _ in 0..12 {
        interp.run_quantum().expect("advances");
    }
    let taken = interp.save().expect("the machine snapshots");

    jit.load(&taken).expect("the JIT board takes it");
    assert_eq!(
        jit.state_hash().expect("hashes"),
        interp.state_hash().expect("hashes"),
        "restoring an interpreter's snapshot into a JIT board did not reproduce it"
    );

    let want = hashes(&mut interp, 20, 5);
    let got = hashes(&mut jit, 20, 5);
    assert_eq!(want, got, "the two diverged after the restore");

    // And the other direction, which is the one that finds derived state a
    // restore forgot to invalidate: the JIT board's block cache was filled from
    // the *old* RAM, and the snapshot it takes must carry none of it.
    let taken = jit.save().expect("the JIT board snapshots");
    let mut fresh = board("interp", "snap.fresh");
    fresh.load(&taken).expect("the interpreter board takes it");
    assert_eq!(
        fresh.state_hash().expect("hashes"),
        jit.state_hash().expect("hashes"),
    );
    assert_eq!(hashes(&mut fresh, 20, 5), hashes(&mut jit, 20, 5));
}
