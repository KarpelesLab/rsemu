//! A save state survives a chunk-version bump — proved by running both
//! machines forward.
//!
//! `ROADMAP.md` §4.5: "a version field with no migration mechanism is
//! decoration", and the test it asks for is "a *cross-version* load from a
//! committed fixture — a round-trip test never exercises it". This file is that
//! test at the machine level, on a whole shipped board, through
//! [`Machine::load`] — the same call `rsemu_load` makes for the browser build's
//! save-state button.
//!
//! # Why the fixture is built rather than committed
//!
//! A committed blob would be a megabyte of flash array and a hostage to every
//! unrelated change on the board — a new device, a moved region, a different
//! demo firmware — none of which has anything to do with migration, all of
//! which would rewrite it. What is committed instead is the *rule* that makes
//! one derivable: `flash.spinor` v1 is a byte-for-byte **prefix** of v2, so the
//! v1 snapshot an older build would have written is this build's snapshot with
//! the thirteen appended shifter bytes removed. `the_v1_tail_is_the_shifter_at_rest`
//! checks that claim against the bytes rather than asserting it, and it is the
//! test that fails first if v2's layout ever stops being an append.
//!
//! # Why both machines run afterwards
//!
//! `save -> load -> save` byte equality and `Machine::state_hash` are both
//! functions of the fields `save` *writes*. A field `save` omits cannot make
//! either fail, however wrong the restored machine is — and a migration that
//! produced plausible-looking bytes and a broken machine would pass every one
//! of them. Only restoring into a board that has been reset and then running
//! **both** forward can see it.

#![cfg(feature = "machine-spi-flash")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::state::{Migrations, StateReader, StateWriter};
use rsemu::machine::{Machine, catalog};

/// The board's instance path for the serial flash.
const FLASH: &str = "nor";

/// The trailing fields `flash.spinor` v2 added: rx, tx, count, and four bools.
const SHIFTER_BYTES: usize = 4 + 4 + 1 + 4;

/// Long enough for the demo firmware to be part way through talking to the
/// flash, which is the only state worth migrating.
const SPAN: GlobalTime = GlobalTime::from_nanos(2_000_000);

/// The `spi-flash` board with its demo firmware, exactly as `tests/spi_flash.rs`
/// boots it.
fn boot() -> Machine {
    let entry = catalog::machine("spi-flash").expect("this build ships spi-flash");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options
        .realize
        .media
        .insert("firmware", rsemu::dev::stm32::demo::SPI_FLASH_DEMO);
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options).expect("it realizes")
}

/// The `nor` chunk of a snapshot, as stored.
fn flash_chunk(bytes: &[u8]) -> (u32, Vec<u8>) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let (_, version, data) = reader.load_raw(FLASH).expect("the board has a flash");
    (version, data.to_vec())
}

/// Rewrite a snapshot with its `flash.spinor` chunk as an older build wrote it.
///
/// Every other chunk is copied verbatim, which is what makes this a snapshot
/// from a *mixed* build in the only way that matters: one class is behind.
fn downgrade_flash_to_v1(bytes: &[u8]) -> Vec<u8> {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let mut w = StateWriter::new(reader.shape().clone());
    for info in reader.chunks() {
        let (class, version, data) = reader.load_raw(info.path).expect("a listed chunk");
        let (version, data) = if info.path == FLASH {
            assert_eq!(
                version, 2,
                "this file knows how to undo v2 and nothing else"
            );
            (1, &data[..data.len() - SHIFTER_BYTES])
        } else {
            (version, data)
        };
        w.raw_chunk(info.path, class, version, data)
            .expect("one chunk per path");
    }
    w.to_vec().expect("a snapshot with an older flash in it")
}

#[test]
fn the_v1_tail_is_the_shifter_at_rest() {
    // The premise the fixture is built on. `flash.spinor` v2 appends the seven
    // shifter fields and changes nothing before them, so truncating a v2 chunk
    // is not an approximation of v1's output — it *is* v1's output, provided
    // the bytes being dropped are the ones a v1 part would have had anyway.
    //
    // On this board the flash is reached through `link = "transactional"`, so
    // its `SlavePins` shifter is never clocked at all: a controller hands the
    // part whole words. That is exactly the case v1 could describe, and the
    // assertion below is what would catch v2 growing a field somewhere other
    // than the end.
    let mut machine = boot();
    machine.run_for(SPAN).expect("it runs");
    let (version, chunk) = flash_chunk(&machine.save().expect("it snapshots"));
    assert_eq!(version, 2, "this build ships flash.spinor v2");
    assert_eq!(
        &chunk[chunk.len() - SHIFTER_BYTES..],
        &[0u8; SHIFTER_BYTES],
        "the transactional link never touches the shifter, so every one of \
         these is its power-on value — rx and tx zero, no bits counted, not \
         selected, SCK at idle, MOSI low, nothing preloaded"
    );
}

#[test]
fn a_v1_save_state_loads_and_the_board_runs_on() {
    let (mut original, mut restored) = (boot(), boot());
    original.run_for(SPAN).expect("it runs");
    let current = original.save().expect("it snapshots");
    let old = downgrade_flash_to_v1(&current);
    assert!(old.len() < current.len(), "v1 is the shorter encoding");

    // The real load path: `Machine::load`, which reaches for this build's own
    // migration table without being asked. Nothing here registers anything.
    restored
        .load(&old)
        .expect("a v1 save state loads into a v2 build");

    // Re-saving is the weak check, and it is still worth making: the migrated
    // machine must be at *this* build's version and describe the same state.
    let again = restored.save().expect("and snapshots again");
    assert_eq!(flash_chunk(&again).0, 2, "restored at the current version");
    assert_eq!(
        again, current,
        "the migrated board re-saves what it restored"
    );

    // The strong check. Both boards run the same span from here; a field the
    // migration failed to establish shows up as a divergence and nowhere else.
    original.run_for(SPAN).expect("the original runs on");
    restored.run_for(SPAN).expect("the migrated one runs on");
    assert_eq!(
        restored.save().expect("b saves"),
        original.save().expect("a saves"),
        "the migrated board diverged from the one it was copied from"
    );
}

#[test]
fn a_v1_save_state_without_the_table_is_refused_by_name() {
    // What every save state in the wild would get if the table were empty, and
    // what made this worth fixing: a clean refusal naming the gap, and no way
    // forward. `load_with` uses the table it is given and nothing else, which
    // is what makes this expressible at all.
    let (mut original, mut restored) = (boot(), boot());
    original.run_for(SPAN).expect("it runs");
    let old = downgrade_flash_to_v1(&original.save().expect("it snapshots"));

    let e = restored
        .load_with(&old, &Migrations::new())
        .expect_err("an empty table cannot upgrade a v1 chunk")
        .to_string();
    assert!(e.contains("flash.spinor"), "{e}");
    assert!(e.contains("v1"), "{e}");
    assert!(e.contains("none registered"), "{e}");
}

#[test]
fn a_snapshot_from_a_newer_build_is_refused_rather_than_guessed_at() {
    // Snapshots move forwards only. A chunk claiming a version this build does
    // not have cannot be read by ignoring the difference, and the error has to
    // say so rather than truncating something.
    let mut machine = boot();
    machine.run_for(SPAN).expect("it runs");
    let current = machine.save().expect("it snapshots");

    let reader = StateReader::new(&current).expect("a snapshot");
    let mut w = StateWriter::new(reader.shape().clone());
    for info in reader.chunks() {
        let (class, version, data) = reader.load_raw(info.path).expect("a listed chunk");
        let version = if info.path == FLASH {
            version + 1
        } else {
            version
        };
        w.raw_chunk(info.path, class, version, data)
            .expect("one chunk per path");
    }
    let future = w.to_vec().expect("a snapshot from the future");

    let e = boot()
        .load(&future)
        .expect_err("a v3 chunk means nothing to a v2 build")
        .to_string();
    assert!(e.contains("never downgraded"), "{e}");
}
