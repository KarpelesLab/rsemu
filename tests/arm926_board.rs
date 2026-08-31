//! The `arm926` board, end to end.
//!
//! A unit test can say "the interpreter executed `STR`". This says something
//! stronger: an ARMv5TE core **named in a `.machine` file** is handed an
//! address space by the machine layer, resets to the vector, runs a program out
//! of a boot ROM, writes into DRAM and into the peripheral aperture, and the
//! bytes are there afterwards.
//!
//! That is the thing that did not exist before: `cpu.arm` had a `DeviceClass`
//! and could be constructed, but no `Instance` impl, no `bind` and no `schema`,
//! so no machine file could give it a space or wire a line to it.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-arm926`.

#![cfg(feature = "machine-arm926")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the board's peripheral aperture is, with the file's default `periph`.
const PERIPH: u64 = 0xf000_0000;

/// Where the board's DRAM starts, with the file's default `ram-base`.
const DRAM: u64 = 0x0200_0000;

/// A firmware image: eight ARM instructions, hand-assembled.
///
/// Written out as words with their mnemonics rather than assembled by anything,
/// because the crate has no assembler and a table of eight encodings is
/// checkable by hand against the ARM ARM (DDI 0100, A3.4 data processing, A5.2
/// load/store, A4.1.5 branch).
///
/// ```text
///   0x00: mov r0, #0xf0000000     ; the peripheral aperture
///   0x04: mov r1, #0x02000000     ; DRAM
///   0x08: mov r2, #42
///   0x0c: str r2, [r1]            ; 42 -> DRAM
///   0x10: ldr r3, [r1]            ; and back out again
///   0x14: add r3, r3, #1          ; 43
///   0x18: str r3, [r0]            ; 43 -> the peripheral window
///   0x1c: b   .                   ; park
/// ```
///
/// The load and the store are what make this a *board* test rather than a
/// decode test: they only work if the machine layer mapped ROM, DRAM and the
/// aperture into one space and handed that space to the core.
const FIRMWARE: [u32; 8] = [
    0xe3a0_04f0,
    0xe3a0_1402,
    0xe3a0_202a,
    0xe581_2000,
    0xe591_3000,
    0xe283_3001,
    0xe580_3000,
    0xeaff_fffe,
];

/// The firmware as bytes, little-endian, which is the byte order the board's
/// `big-endian = false` selects.
fn firmware() -> Vec<u8> {
    FIRMWARE.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Build the board out of the catalog with the firmware in its `firmware` slot.
fn boot() -> Machine {
    let entry = catalog::machine("arm926").expect("this build ships arm926");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read one word of the guest's memory space.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

#[test]
fn the_board_realizes_with_the_core_bound_to_its_space() {
    let m = boot();
    assert_eq!(m.name(), "arm926");
    for path in ["cpu", "boot", "dram", "regs"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The firmware landed at the reset vector, which is where the core will
    // fetch its first instruction from.
    assert_eq!(peek(&m, 0x0000_0000), u64::from(FIRMWARE[0]));
}

#[test]
fn the_firmware_runs_and_reaches_dram_and_the_peripheral_window() {
    let mut m = boot();

    // Eight instructions and a reset sequence, with room to spare: a millisecond
    // of virtual time at 200 MHz is 200,000 ticks. A span rather than an
    // instruction count because the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    assert_eq!(
        peek(&m, DRAM),
        42,
        "the `STR` into DRAM did not reach the mapped RAM"
    );
    assert_eq!(
        peek(&m, PERIPH),
        43,
        "the `STR` into the peripheral aperture did not reach it"
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

    // And the restored machine keeps running from where the other one was,
    // rather than from a reset it silently took on the way in.
    assert_eq!(peek(&other, PERIPH), 43);
}
