//! The `m68k-mini` board, end to end.
//!
//! A unit test can say "the interpreter executed `MOVE.L`". This says something
//! stronger: an MC68000 **named in a `.machine` file** is handed an address
//! space by the machine layer, fetches its stack pointer and program counter
//! out of the two longwords at address zero, runs a program from ROM, and the
//! bytes it wrote are in RAM afterwards — in the right byte order.
//!
//! Byte order is the half of this that a unit test with a hand-built space does
//! not exercise. rsemu carries endianness per region, so it is the *board* that
//! decides whether a 68000 sees words the right way round, and a board that got
//! it wrong would still realize and would still run — into the weeds.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-m68k-mini`.

#![cfg(feature = "machine-m68k-mini")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the board's RAM starts, with the file's default `ram-base`.
const RAM: u64 = 0x0010_0000;

/// A firmware image, hand-assembled from the MC68000 user manual's instruction
/// formats (MC68000UM, §"Instruction Set Details").
///
/// ```text
///   000000: 0020 0000    dc.l  $00200000   ; the reset supervisor stack pointer
///   000004: 0000 0400    dc.l  $00000400   ; the reset program counter
///   ...
///   000400: 203c 1234 5678   move.l #$12345678, d0
///   000406: 21c0 0000        move.l d0, ($0000).w   ; -- see below
///   00040a: 23c0 0010 0000   move.l d0, ($00100000).l
///   000410: 60fe             bra    *
/// ```
///
/// The `move.l d0,($0000).w` is not in the image — a word-sized absolute
/// address would sign-extend and land back on the vector table. The long form
/// at `$00040a` is the one that matters, and `$12345678` is chosen because
/// every byte of it is different: if the board put the 68000 on a
/// little-endian map, the longword in RAM comes back as `$78563412` and this
/// test says so.
fn firmware() -> Vec<u8> {
    let mut image = vec![0u8; 0x0412];
    // The reset vectors, big-endian, which is how a 68000 reads them.
    image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes()); // SSP
    image[4..8].copy_from_slice(&0x0000_0400u32.to_be_bytes()); // PC
    let code: &[u16] = &[
        0x203c, 0x1234, 0x5678, // move.l #$12345678, d0
        0x23c0, 0x0010, 0x0000, // move.l d0, ($00100000).l
        0x60fe, // bra *
    ];
    for (i, word) in code.iter().enumerate() {
        let at = 0x0400 + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// Build the board out of the catalog with the firmware in its `firmware` slot.
fn boot() -> Machine {
    boot_on("interp")
}

/// The same board with `engine` as the core's execution engine.
fn boot_on(engine: &str) -> Machine {
    let entry = catalog::machine("m68k-mini").expect("this build ships m68k-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("engine"), String::from(engine)));
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize with engine={engine}: {e}"),
    }
}

/// Read a big-endian longword out of the guest's memory space.
fn peek_long(m: &Machine, addr: u64) -> u32 {
    let space = m.space("mem").expect("the memory space");
    let mut value = 0u32;
    for i in 0..4 {
        let byte = space
            .read(addr + i, Width::U8, MemAttrs::DEFAULT)
            .expect("a mapped byte") as u32;
        value = (value << 8) | byte;
    }
    value
}

#[test]
fn the_board_realizes_with_the_core_bound_to_its_space() {
    let m = boot();
    assert_eq!(m.name(), "m68k-mini");
    for path in ["cpu", "boot", "dram"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The two longwords the processor fetches out of reset are where the image
    // put them, and are readable in the order it wrote them.
    assert_eq!(peek_long(&m, 0), 0x0020_0000);
    assert_eq!(peek_long(&m, 4), 0x0000_0400);
}

#[test]
fn the_firmware_runs_and_its_longword_reaches_ram_the_right_way_round() {
    let mut m = boot();
    // A millisecond of virtual time at 8 MHz is 8,000 cycles and the program is
    // under fifty, reset sequence included. A span rather than an instruction
    // count because the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    assert_eq!(
        peek_long(&m, RAM),
        0x1234_5678,
        "either the `MOVE.L` never ran, or the board put a big-endian core on \
         a little-endian map and every word is swapped"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert_eq!(peek_long(&other, RAM), 0x1234_5678);
}

/// `ROADMAP.md` §0's non-negotiable, on this board: *a bit-identical state hash
/// across the interpreter and the translated engine for the same guest*.
///
/// `cpu::m68k::differential` compares two cores column by column over every
/// opcode word there is, which is the stronger test of the *frontend*. This is
/// the stronger test of the **machine**: one hash over every register, every
/// byte of RAM and every device's state, taken at a checkpoint, with the whole
/// board in it. A cache hit, a cache miss, a lifted instruction and an
/// interpreted one all have to come out as the same number.
#[test]
#[cfg(feature = "cpu-m68k-lift")]
fn both_engines_hash_to_the_same_machine_at_every_checkpoint() {
    let mut interp = boot_on("interp");
    let mut ir = boot_on("ir");
    for n in 1..=24 {
        interp
            .run_quantum()
            .expect("the interpreted machine advances");
        ir.run_quantum().expect("the translated machine advances");
        let want = interp.state_hash().expect("a deterministic machine hashes");
        let got = ir.state_hash().expect("a deterministic machine hashes");
        assert_eq!(
            want, got,
            "checkpoint {n}: `engine = \"interp\"` hashes to {want:#018x} and \
             `engine = \"ir\"` to {got:#018x}"
        );
    }
    // And the program really ran, so this is a comparison of a machine that
    // did something rather than of two idle boards.
    assert_eq!(peek_long(&ir, RAM), 0x1234_5678);
}

/// The `engine` property is *read* rather than accepted and ignored.
///
/// The comparison above passes trivially if `engine` reaches nothing, so the
/// negative is what makes it mean something: an engine no build implements is
/// refused, with a message.
#[test]
fn an_engine_nothing_implements_is_refused_rather_than_ignored() {
    let entry = catalog::machine("m68k-mini").expect("this build ships m68k-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("engine"), String::from("jit-host")));
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("an engine nothing implements must be refused, not ignored");
}

/// And a build that *cannot* run the translated engine says so rather than
/// interpreting quietly.
///
/// The other half of the same rule: "an engine that silently is not the one
/// you asked for is how a JIT stays unmeasured for a year"
/// (`cpu::riscv::Engine::Jit`, which learned it the hard way).
#[test]
#[cfg(not(feature = "cpu-m68k-lift"))]
fn a_build_without_the_frontend_refuses_the_engine_it_cannot_run() {
    let entry = catalog::machine("m68k-mini").expect("this build ships m68k-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("engine"), String::from("ir")));
    let registry = catalog::registry().expect("a registry");
    let err = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("`ir` needs `cpu-m68k-lift`");
    let text = format!("{err}");
    assert!(text.contains("cpu-m68k-lift"), "{text}");
}
