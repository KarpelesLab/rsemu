//! One AArch64 board under all three execution engines, and the claim that
//! they are the same machine.
//!
//! `ROADMAP.md` §0: *"a bit-identical state hash … across the interpreter and
//! the JIT for the same guest"*. `src/cpu/arm/a64/differential.rs` asserts
//! something one level below that — a block, a register, a flag, a tick — and
//! `src/cpu/arm/a64/engine.rs`'s own tests assert it for a bare core; until
//! `engine = "jit"` reached the dispatcher there was no *machine* to hash.
//! There is now, and this is the assertion the roadmap actually wrote down. It
//! is `tests/riscv_virt_engines.rs` and `tests/x86_engines.rs` for the other
//! two cores, deliberately in the same shape:
//!
//! * the same board, the same guest, the same number of quanta, under
//!   `engine = "interp"`, `engine = "jit"` and `engine = "jit-host"`, hashes to
//!   the same number at **every** checkpoint — not at the end only, because a
//!   divergence that shows up at the last checkpoint is a different bug from
//!   one that shows up at the first;
//! * a snapshot taken under one engine restores under the other and carries on
//!   to the same hash, which is a property of the snapshot rather than of the
//!   engines: nothing engine-specific is in one.
//!
//! # Why the board is built here rather than taken from `machines/`
//!
//! `machines/arm64-virt.machine` writes `engine = "interp"` as a literal
//! rather than as a `param`, so there is nothing for a test to override, and
//! it wants a kernel image besides. `machines/a64-mini.machine` wants a
//! firmware. So this file builds the smallest machine that is entirely inside
//! the lifted subset: a clock, RAM and a core — which is also what makes the
//! comparison about the engines and nothing else.
//!
//! The guest is a loop rather than a program: an add, a store, a load, an
//! exclusive-or and a direct backward branch, with its scratch word on a
//! **different page** from its code so the store does not invalidate the block
//! it sits in. Every instruction is inside the lifted subset and the back edge
//! is a direct `B`, so the JIT really does translate, cache, chain and
//! re-serve blocks here — asserted through `Cpu::jit_stats`, not assumed.

#![cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::arm::a64::Cpu;
use rsemu::machine::{Machine, build};

/// The smallest AArch64 machine there is: a clock, RAM and a core.
const SOURCE: &str = r#"
machine "a64-engines" {
  param engine = "interp"
  param ram = 1M
  osc cpu = 100000000 Hz
  space mem { width = 64 }
  object cpu0 "cpu.arm.a64" {
    clock  = cpu
    space  = mem
    engine = engine
    cpu    = "cortex-a53"
    reset  = 0x00000000
  }
  object dram "ram" { size = ram }
  map mem 0x00000000 size ram = dram
}
"#;

/// Where the guest program is written, which is the board's reset vector.
const PROGRAM: u64 = 0;

/// A counting loop whose scratch word is on a different page from its code.
///
/// Every instruction is inside the lifted subset — `MOVZ`, `ADD` immediate, a
/// store, a load, a shifted-register `EOR` and a direct `B` — so a translated
/// core translates all of it, and the store ends each block, so the loop is a
/// **chain** of short blocks rather than one long one.
const LOOP: [u32; 7] = [
    0xd282_0007, // movz x7, #0x1000     ; the scratch page
    0xd280_0005, // movz x5, #0
    0x9100_04a5, // add  x5, x5, #1      ; the loop starts here
    0xf900_00e5, // str  x5, [x7]
    0xf940_00e6, // ldr  x6, [x7]
    0xca05_0108, // eor  x8, x8, x5
    0x17ff_fffc, // b    .-16
];

/// Build the board on `engine`, with the program in RAM.
///
/// `tag` names the machine, because two of them live in one process.
fn board(engine: &str, tag: &str) -> (Machine, Arc<Cpu>) {
    let cpus: Arc<Captured<Cpu>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cpus);
    let mut bindings = rsemu::machine::catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.arm.a64", move |props| {
        let cpu = Arc::new(Cpu::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings)
        .with_param("engine", engine);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = build(&format!("a64-engines.{tag}"), SOURCE, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not build with engine={engine}: {e}"));
    let cpu = cpus.take().expect("the binding captured the core");

    // `build` realizes **and resets**, and a cold reset zeroes RAM, so the
    // program goes in afterwards.
    let space = cpu.space().expect("the core has its space");
    for (i, word) in LOOP.iter().enumerate() {
        space
            .write(
                PROGRAM + 4 * i as u64,
                Width::U32,
                u64::from(*word),
                MemAttrs::DEFAULT,
            )
            .expect("the program fits in RAM");
    }
    (machine, cpu)
}

/// Run `quanta` quanta, taking a state hash every `every` of them.
fn hashes(machine: &mut Machine, quanta: usize, every: usize) -> Vec<u64> {
    let mut out = Vec::new();
    for n in 0..quanta {
        machine.run_quantum().expect("the machine runs");
        if (n + 1) % every == 0 {
            out.push(machine.state_hash().expect("the machine hashes"));
        }
    }
    out
}

#[test]
fn every_engine_hashes_to_the_same_machine_at_every_checkpoint() {
    let (mut interp, _) = board("interp", "interp");
    let want = hashes(&mut interp, 40, 4);
    // A guard on the checkpoint arithmetic rather than on the guest: a change
    // that quietly took ten checkpoints down to one would still pass below.
    assert_eq!(want.len(), 10);

    for engine in ["jit", "jit-host"] {
        let (mut machine, cpu) = board(engine, engine);
        let got = hashes(&mut machine, 40, 4);
        for (n, (a, b)) in want.iter().zip(&got).enumerate() {
            assert_eq!(
                a, b,
                "checkpoint {n} under engine={engine}: {a:#018x} interpreted, {b:#018x} \
                 translated. ROADMAP.md §0 requires a bit-identical state hash across \
                 the interpreter and the JIT for the same guest"
            );
        }
        let stats = cpu.jit_stats().expect("a translated core has statistics");
        assert!(
            stats.blocks > 0,
            "engine={engine} executed no translated block at all, so the \
             comparison above compared two interpreters"
        );
        assert!(
            stats.retired > stats.interpreted,
            "engine={engine} retired {} instructions inside blocks against {} \
             interpreted, which is not a translated run",
            stats.retired,
            stats.interpreted
        );
        if engine == "jit" {
            assert_eq!(
                stats.compiled, 0,
                "the portable backend must compile nothing; `jit-host` is what asks \
                 for the code generator"
            );
        }
    }
}

#[test]
fn the_host_code_generator_actually_compiles_where_the_build_has_one() {
    let (mut machine, cpu) = board("jit-host", "hostcode");
    for _ in 0..40 {
        machine.run_quantum().expect("the machine runs");
    }
    let stats = cpu.jit_stats().expect("a translated core has statistics");
    assert!(stats.blocks > 0, "no block ran");
    let compiled = stats.compiled;
    if cfg!(all(
        feature = "jit-x86",
        target_os = "linux",
        target_arch = "x86_64"
    )) {
        assert!(
            compiled > 0,
            "this build has a host code generator and it compiled nothing"
        );
    } else {
        assert_eq!(
            compiled, 0,
            "this build has no host code generator, so `jit-host` is `jit`"
        );
    }
}

#[test]
fn the_engine_property_is_read_rather_than_accepted_and_ignored() {
    // A value nothing implements must be refused by name, not silently
    // interpreted — an engine that is not the one you asked for is a
    // measurement that quietly means nothing.
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"))
        .with_param("engine", "tier1");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let err = build("a64-engines.bad", SOURCE, &registry, &options)
        .expect_err("`engine = \"tier1\"` names no engine");
    let text = format!("{err}");
    assert!(text.contains("engine"), "{text}");
    assert!(text.contains("interp"), "{text}");
    assert!(text.contains("jit"), "{text}");
}

#[test]
fn a_snapshot_crosses_from_one_engine_to_the_other_and_carries_on_the_same() {
    let (mut interp, _) = board("interp", "snap.interp");
    let (mut jit, _) = board("jit", "snap.jit");

    for _ in 0..12 {
        interp.run_quantum().expect("the machine runs");
    }
    let taken = interp.save().expect("the machine saves");
    jit.load(&taken).expect("the snapshot restores");
    assert_eq!(
        interp.state_hash().expect("hashes"),
        jit.state_hash().expect("hashes"),
        "a snapshot taken under the interpreter must restore under the JIT to \
         the same state"
    );
    assert_eq!(hashes(&mut interp, 20, 5), hashes(&mut jit, 20, 5));

    // And the other direction, which is the one that catches derived state —
    // a block cache filled from the old RAM — leaking into a snapshot.
    let taken = jit.save().expect("the machine saves");
    interp.load(&taken).expect("the snapshot restores");
    assert_eq!(
        interp.state_hash().expect("hashes"),
        jit.state_hash().expect("hashes")
    );
    assert_eq!(hashes(&mut jit, 20, 5), hashes(&mut interp, 20, 5));
}
