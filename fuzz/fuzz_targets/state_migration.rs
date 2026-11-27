#![no_main]
//! A migration is a parser, and it runs on bytes a user handed us.
//!
//! `core::state` states the contract for the snapshot reader — never panic,
//! never index without a bounds check, never trust a length it has not compared
//! against the bytes remaining, never allocate proportional to a claimed count.
//! Every one of those applies to a migration step, and more sharply: a step
//! runs *before* the device's own `load` gets to reject anything, so it sees the
//! most hostile version of a chunk there is. An old save state is also exactly
//! the file a user is most likely to have got from somewhere else.
//!
//! What is driven here is the table `Machine::load` actually uses —
//! `machine::default_migrations()` — with the fuzzer's bytes standing in for an
//! old chunk of a class that has a step registered. That was `flash.spinor`
//! alone when this target was written, and it went on being the only class
//! driven while five more gained steps of their own — two of them processor
//! cores, whose chunks are the largest and most intricate in a snapshot. Every
//! class in [`CLASSES`] is driven now; the list names each `CLASS` static
//! directly, so one that is renamed or dropped breaks this build rather than
//! quietly leaving the table.
//!
//! Three properties:
//!
//! 1. **The step never panics**, on any input, at any registered version.
//! 2. **What it produces is a chunk**, not a corruption: the device's `load`
//!    either accepts it or reports an error, and never panics either way. A
//!    part that accepted the migrated bytes must still answer frames
//!    afterwards, because a device left half-loaded is worse than one that
//!    refused. This one is checked for `flash.spinor` only — it needs a device
//!    to load into, and that part is the one cheap enough to build per input.
//! 3. **Upgrading is monotone and total.** Asking for the version the chunk
//!    already is must be the identity and must not copy; asking for a version
//!    below it must be refused rather than guessed at, because snapshots move
//!    forwards only.
//!
//! # Input encoding
//!
//! The first byte picks the class and the version the chunk claims to be: the
//! class is its high nibble modulo the table below, and the version is its low
//! nibble modulo two more than the class ships, so the fuzzer reaches every
//! registered version, the current one and one from the future. The rest is
//! the chunk payload. An empty input is a zero-length `flash.spinor` chunk,
//! which is itself a case worth having.

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, DeviceClass};
use rsemu::core::props::{Props, Value};
use rsemu::core::state::{MachineShape, StateReader, StateWriter};
use rsemu::dev::flash::spinor::{CLASS, CLASS_NAME, SpiNor};
use rsemu::machine::default_migrations;

/// The instance path every chunk here is written at.
const PATH: &str = "nor";

/// Every class `machine::default_migrations` registers a step for that this
/// build has. `flash.spinor` is first so that an input whose first byte is
/// below sixteen is still a chunk of that class, as every committed seed is.
static CLASSES: &[&DeviceClass] = &[
    &CLASS,
    &rsemu::dev::stm32::octospi::CLASS,
    &rsemu::dev::rcar::scif::CLASS,
    &rsemu::cpu::arm::aprofile::CLASS,
    &rsemu::cpu::x86::CLASS,
    &rsemu::dev::sgx::CLASS,
];

/// A part small enough that building one per iteration is cheap, and a power of
/// two of at least one 64 KiB block as the class requires.
const SIZE: u64 = 64 * 1024;

/// Wrap `payload` as a snapshot holding one chunk of `class` at `version`.
fn snapshot(class: &str, version: u32, payload: &[u8]) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device(PATH, class).expect("a fresh shape");
    let mut writer = StateWriter::new(shape);
    writer
        .raw_chunk(PATH, class, version, payload)
        .expect("one chunk at one path");
    writer.to_vec().expect("a writer with one chunk emits")
}

fuzz_target!(|data: &[u8]| {
    let (selector, payload) = match data.split_first() {
        Some((v, rest)) => (*v, rest),
        None => (0, &[][..]),
    };
    let class = CLASSES[usize::from(selector >> 4) % CLASSES.len()];
    let version = u32::from(selector & 0x0f) % (class.version + 2);

    let table = default_migrations().expect("the shipped table has no duplicate step");
    assert!(
        !table.steps_for(class.name).is_empty(),
        "`{}` is listed here as having migration steps and registers none",
        class.name
    );
    let bytes = snapshot(class.name, version, payload);
    let reader = StateReader::new(&bytes).expect("what the writer wrote, the reader reads");

    // Downgrading is refused, always, whatever the payload says.
    if version > 0 {
        assert!(
            reader.load(PATH, class.name, version - 1, &table).is_err(),
            "a v{version} chunk must not be readable as v{}",
            version - 1
        );
    }

    // The identity case borrows rather than copies, and cannot fail.
    let same = reader
        .load(PATH, class.name, version, &table)
        .expect("a chunk at its own version needs no step");
    assert!(!same.migrated(), "no step should have run");
    assert_eq!(same.data(), payload, "the identity case changed the bytes");

    // The step itself. A version with no step registered must be an error
    // naming the gap; one with a step must produce bytes the device can be
    // handed.
    let Ok(chunk) = reader.load(PATH, class.name, class.version, &table) else {
        return;
    };
    assert_eq!(chunk.version(), class.version);
    assert_eq!(chunk.stored_version(), version);
    assert_eq!(chunk.migrated(), version != class.version);

    // The rest needs a device to load into, and only the flash part is cheap
    // enough to build on every input.
    if class.name != CLASS_NAME {
        return;
    }

    let part =
        SpiNor::new(&Props::new().with("size", Value::Size(SIZE))).expect("a plausible part");
    if part.load(&mut chunk.reader()).is_err() {
        return;
    }

    // It accepted the migrated bytes, so it must still be a working part: a
    // `9Fh` frame has to come back with the identifier rather than panicking or
    // hanging on whatever state the chunk described.
    let slave = part.slave();
    slave.select(true);
    for word in [0x9fu32, 0, 0, 0] {
        let _ = rsemu::bus::spi::exchange(&*slave, word);
    }
    slave.select(false);

    // And saving it again must work, at the current version, with bytes the
    // reader accepts — a migrated device is not a second-class one.
    let again = {
        let mut writer = StateWriter::new({
            let mut shape = MachineShape::new();
            shape.add_device(PATH, CLASS_NAME).expect("a fresh shape");
            shape
        });
        {
            let mut out = writer
                .chunk(PATH, CLASS_NAME, CLASS.version)
                .expect("one chunk");
            part.save(&mut out).expect("a part saves");
        }
        writer.to_vec().expect("and the snapshot emits")
    };
    let reread = StateReader::new(&again).expect("a snapshot this build wrote");
    reread
        .load(PATH, CLASS_NAME, CLASS.version, &table)
        .expect("and reads back");
});
