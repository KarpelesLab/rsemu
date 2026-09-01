//! The `spi-flash` board, end to end.
//!
//! A unit test can say "the register accepted the write". This says something
//! much stronger: a RISC-V program, running on the emulated hart,
//!
//! 1. reads a Winbond W25Q's JEDEC identifier through an emulated STM32F4 SPI
//!    peripheral,
//! 2. programs a payload into that flash through an emulated OCTOSPI's
//!    indirect-write mode, one `DR` byte at a time,
//! 3. puts the OCTOSPI into memory-mapped mode and **jumps into the window**,
//! 4. and the payload runs — every instruction of it fetched back out of the
//!    flash as an `0Bh` frame clocked down `bus::spi`.
//!
//! The sentinel in RAM can only appear if all four of those happened. Nothing
//! copies the payload anywhere: the only place those bytes exist after step 2
//! is the flash model's array.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-spi-flash`.

#![cfg(feature = "machine-spi-flash")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::dev::stm32::demo::{PAYLOAD_BYTES, RAM, SENTINEL, SPI_FLASH_DEMO, WINDOW, payload};
use rsemu::machine::{Machine, catalog};

/// Build the board and boot the demo firmware.
fn boot() -> Machine {
    let entry = catalog::machine("spi-flash").expect("this build ships spi-flash");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", SPI_FLASH_DEMO);
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options).expect("it realizes")
}

/// Read `len` bytes of the machine's address space, as a debugger would.
fn peek(machine: &Machine, at: u64, len: usize) -> Vec<u8> {
    let space = machine.space("mem").expect("the board has one space");
    let mut out = vec![0u8; len];
    space
        .read_bytes(at, &mut out, MemAttrs::DEBUG)
        .expect("inside the map");
    out
}

/// Run until the payload has left its mark, or give up.
fn run_until_sentinel(machine: &mut Machine) -> u64 {
    let mut elapsed = 0u64;
    // A condition rather than a fixed span: what bounds how far a hart gets
    // per millisecond is the scheduler's quantum budget, and a hard-coded
    // number would fail the day that default changed.
    while elapsed < 2_000_000_000 {
        let word = u32::from_le_bytes(peek(machine, u64::from(RAM) + 4, 4).try_into().unwrap());
        if word == SENTINEL {
            return elapsed;
        }
        machine
            .run_for(GlobalTime::from_nanos(5_000_000))
            .expect("it runs");
        elapsed += 5_000_000;
    }
    panic!("the payload never ran within {elapsed} ns of virtual time");
}

#[test]
fn the_guest_reads_the_flashs_jedec_id_through_the_stm32_spi() {
    let mut machine = boot();
    run_until_sentinel(&mut machine);
    // `EFh` is Winbond (JEP106 bank 1), `40h` the W25Q ordering option whose
    // `QE` is fixed set, and `14h` the capacity byte of a 1 MiB part — the
    // base-two logarithm of the density, `2^20`.
    assert_eq!(peek(&machine, u64::from(RAM), 3), [0xef, 0x40, 0x14]);
}

#[test]
fn the_guest_programs_the_flash_and_then_executes_out_of_the_window() {
    let mut machine = boot();
    run_until_sentinel(&mut machine);

    // The sentinel is in RAM, which is only reachable by executing the payload
    // — and the payload exists nowhere but the flash.
    let word = u32::from_le_bytes(peek(&machine, u64::from(RAM) + 4, 4).try_into().unwrap());
    assert_eq!(word, SENTINEL);

    // And the window really does answer with what was programmed. This read
    // goes through the OCTOSPI, down the SPI bus, into the flash's `0Bh`
    // handler and back — the same path every one of those instruction fetches
    // took.
    let bytes = {
        let space = machine.space("mem").expect("one space");
        let mut out = vec![0u8; PAYLOAD_BYTES as usize];
        space
            .read_bytes(u64::from(WINDOW), &mut out, MemAttrs::DEFAULT)
            .expect("the window answers");
        out
    };
    assert_eq!(bytes, payload(), "the flash holds what the guest wrote");
}

#[test]
fn the_window_refuses_a_debug_read_rather_than_disturbing_the_bus() {
    let mut machine = boot();
    run_until_sentinel(&mut machine);
    // Reaching the flash means clocking a frame, which moves another device's
    // command state machine. There is no side-effect-free route through a bus,
    // so `MemAttrs::debug` is honoured by refusing.
    let space = machine.space("mem").expect("one space");
    let mut out = [0u8; 4];
    assert!(
        space
            .read_bytes(u64::from(WINDOW), &mut out, MemAttrs::DEBUG)
            .is_err(),
        "a debug read of the aperture must not clock a frame"
    );
}

#[test]
fn the_mapping_refuses_a_write_because_the_decode_says_read_only() {
    let mut machine = boot();
    run_until_sentinel(&mut machine);
    // `perms = "r-x"` in the machine file. The `w` is genuinely enforced by
    // `core::space`, unlike the `x`, which is carried and not yet checked.
    let space = machine.space("mem").expect("one space");
    assert!(
        space
            .write_bytes(u64::from(WINDOW), &[0u8; 4], MemAttrs::DEFAULT)
            .is_err(),
        "a read-only mapping refuses a store"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_the_same_state_hash() {
    let mut machine = boot();
    run_until_sentinel(&mut machine);
    let before = machine.state_hash().expect("a hash");
    let bytes = machine.save().expect("it snapshots");

    let mut other = boot();
    other.load(&bytes).expect("it restores");
    assert_eq!(other.state_hash().expect("a hash"), before);
    // And the restored machine keeps running out of the window.
    other
        .run_for(GlobalTime::from_nanos(5_000_000))
        .expect("it runs");
}
