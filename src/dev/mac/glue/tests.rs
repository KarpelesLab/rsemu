//! The address decoder's own tests: which side answers, and the one rule the
//! ROM's memory sizing depends on.

use super::*;
use crate::core::props::{Link, Value};
use crate::core::space::{RamStore, RomStore, RomWrite};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::{Endian, Width};
use alloc::vec;

const RAM_LEN: u64 = 0x10_0000;
const ROM_LEN: u64 = 0x2_0000;

fn props() -> Props {
    Props::new()
        .with("rom", Value::Link(Link::new("rom").unwrap()))
        .with("ram", Value::Link(Link::new("dram").unwrap()))
}

/// A decoder with a ROM whose first bytes are recognisable and a blank memory
/// behind it, both big-endian as a 68000 stores them.
fn wired() -> (Glue, Arc<RamStore>) {
    let glue = Glue::new(&props()).expect("the two links are enough");
    let mut image = vec![0u8; ROM_LEN as usize];
    image[..4].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);
    let rom: RegionRef = Arc::new(
        Region::rom("rom", Arc::new(RomStore::new(image)), RomWrite::Ignore)
            .with_endian(Endian::Big),
    );
    let store = Arc::new(RamStore::new(RAM_LEN));
    let ram: RegionRef = Arc::new(Region::ram("dram", Arc::clone(&store)).with_endian(Endian::Big));
    glue.attach(&rom, &ram, 24).expect("both map");
    (glue, store)
}

fn read(glue: &Glue, region: &str, offset: u64) -> MemResult<[u8; 4]> {
    let mut out = [0u8; 4];
    let window = if region == LOW_REGION {
        &glue.low
    } else {
        &glue.high
    };
    window.read(offset, &mut out, MemAttrs::DEFAULT)?;
    Ok(out)
}

/// Drop the overlay the way the VIA's `PA4` does.
fn clear_overlay(glue: &Glue) {
    let pin = glue.sink(OVERLAY_PIN, &[WireId::new(1)]).expect("the pin");
    pin.sink.set_level(WireId::new(1), 0, Level::Low);
}

/// Out of reset the ROM answers at zero, which is the only reason a 68000 can
/// find a reset vector on a machine whose memory holds nothing.
#[test]
fn the_rom_answers_at_zero_until_the_overlay_is_cleared() {
    let (glue, _ram) = wired();
    assert!(glue.overlaid());
    assert_eq!(
        read(&glue, LOW_REGION, 0).unwrap(),
        [0xde, 0xad, 0xbe, 0xef]
    );
    // And it repeats every 128 KiB, because the socket has no pin above its
    // own top address line.
    assert_eq!(
        read(&glue, LOW_REGION, ROM_LEN).unwrap(),
        [0xde, 0xad, 0xbe, 0xef]
    );
    assert_eq!(
        read(&glue, LOW_REGION, 0x30_0000).unwrap(),
        [0xde, 0xad, 0xbe, 0xef]
    );
}

/// `PA4` going low swaps memory in underneath, and the vector table is then
/// somewhere an exception handler can be written.
#[test]
fn clearing_the_overlay_puts_memory_at_zero() {
    let (glue, ram) = wired();
    ram.write_at(0, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    clear_overlay(&glue);
    assert!(!glue.overlaid());
    assert_eq!(
        read(&glue, LOW_REGION, 0).unwrap(),
        [0x11, 0x22, 0x33, 0x44]
    );
}

/// **Main memory does not repeat.** This is the rule the ROM's memory sizing
/// depends on: a 1 MiB machine must *not* answer at `$100000`, or it is taken
/// for a 4 MiB one — which is exactly what happened while this decoder folded
/// the address, and the ROM then looped in its memory test for ever.
#[test]
fn memory_answers_where_it_is_and_the_rest_of_the_window_floats() {
    let (glue, ram) = wired();
    ram.write_at(0, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    clear_overlay(&glue);
    assert!(
        read(&glue, LOW_REGION, RAM_LEN - 4).is_ok(),
        "the last word"
    );
    assert_ne!(
        read(&glue, LOW_REGION, RAM_LEN).unwrap(),
        [0x11, 0x22, 0x33, 0x44],
        "the first address above the installed memory must not alias it"
    );
}

/// The `$600000` window is memory only while the overlay is up, and nothing
/// afterwards.
#[test]
fn the_high_window_is_memory_only_while_the_overlay_is_asserted() {
    let (glue, ram) = wired();
    ram.write_at(0, &[0xaa, 0xbb, 0xcc, 0xdd]).unwrap();
    assert_eq!(
        read(&glue, HIGH_REGION, 0).unwrap(),
        [0xaa, 0xbb, 0xcc, 0xdd],
        "the overlay's memory window"
    );
    clear_overlay(&glue);
    assert_eq!(read(&glue, HIGH_REGION, 0), Err(BusError::Unassigned));
}

/// A write while the ROM is at zero goes to the ROM, which ignores it — and
/// the memory underneath is untouched, because the decoder is pointing
/// somewhere else entirely.
#[test]
fn a_write_reaches_whichever_side_is_decoded() {
    let (glue, ram) = wired();
    glue.low
        .write(0, &[1, 2, 3, 4], MemAttrs::DEFAULT)
        .expect("the ROM swallows it");
    let mut out = [0u8; 4];
    ram.read_at(0, &mut out).unwrap();
    assert_eq!(out, [0, 0, 0, 0]);

    clear_overlay(&glue);
    glue.low
        .write(0, &[1, 2, 3, 4], MemAttrs::DEFAULT)
        .expect("memory takes it");
    ram.read_at(0, &mut out).unwrap();
    assert_eq!(out, [1, 2, 3, 4]);
}

/// Both kinds of reset put the ROM back at zero: the processor is about to
/// fetch a reset vector, and a board whose overlay did not come back would
/// reset into empty memory.
#[test]
fn reset_restores_the_overlay() {
    let (glue, _ram) = wired();
    clear_overlay(&glue);
    assert!(!glue.overlaid());
    glue.reset(ResetKind::Warm);
    assert!(glue.overlaid());
}

/// Invariant 6: the pin level round-trips, because a restore does not re-run
/// the wire graph.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let (saved, _ram) = wired();
    clear_overlay(&saved);

    let image = |glue: &Glue| -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("glue", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("glue", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(glue, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    };
    let first = image(&saved);

    let (restored, ram) = wired();
    ram.write_at(0, &[0x55; 4]).unwrap();
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("glue", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(image(&restored), first, "the same decoder, bit for bit");
    assert!(!restored.overlaid(), "it came back pointing at memory");
    assert_eq!(read(&restored, LOW_REGION, 0).unwrap(), [0x55; 4]);
}

/// A debug read travels through unchanged, because a decoder has no state to
/// disturb and the memory behind it makes its own decision.
#[test]
fn a_debug_read_reaches_the_same_place() {
    let (glue, _ram) = wired();
    let mut out = [0u8; 4];
    glue.low.read(0, &mut out, MemAttrs::DEBUG).unwrap();
    assert_eq!(out, [0xde, 0xad, 0xbe, 0xef]);
}

/// The decoder imposes no width of its own: whatever is behind it decides.
#[test]
fn the_decoder_accepts_whatever_the_memory_behind_it_does() {
    let (glue, _ram) = wired();
    let c = glue.low.constraints();
    assert_eq!(c.min, Width::U8);
    assert!(c.max >= Width::U32);
}

/// Both links are required, and a memory object with no bytes is refused
/// rather than mapped.
#[test]
fn properties_are_checked() {
    assert!(Glue::new(&Props::new()).is_err(), "`rom` is required");
    let err = Glue::new(&props().with("size", Value::Size(1)))
        .expect_err("a property this class does not know")
        .to_string();
    assert!(err.contains("size"), "{err}");

    let glue = Glue::new(&props()).unwrap();
    let empty: RegionRef = Arc::new(Region::ram("none", Arc::new(RamStore::new(0))));
    let rom: RegionRef = Arc::new(Region::rom(
        "rom",
        Arc::new(RomStore::zeroed(ROM_LEN)),
        RomWrite::Ignore,
    ));
    let err = glue
        .attach(&rom, &empty, 24)
        .expect_err("no bytes")
        .to_string();
    assert!(err.contains("no bytes"), "{err}");
}

/// A ROM that cannot repeat evenly through the window is refused, rather than
/// answering at some addresses and not others.
#[test]
fn a_rom_that_cannot_repeat_evenly_is_refused() {
    let glue = Glue::new(&props()).unwrap();
    let odd: RegionRef = Arc::new(Region::rom(
        "rom",
        Arc::new(RomStore::zeroed(0x3_0000)),
        RomWrite::Ignore,
    ));
    let ram: RegionRef = Arc::new(Region::ram("dram", Arc::new(RamStore::new(RAM_LEN))));
    let err = glue.attach(&odd, &ram, 24).expect_err("not a power of two");
    assert!(err.to_string().contains("repeat evenly"), "{err}");
}

/// The class registers and names the regions and the pin a machine file may
/// write.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);
    let glue = Glue::new(&props()).unwrap();
    assert!(glue.region("").is_some());
    assert!(glue.region(LOW_REGION).is_some());
    assert!(glue.region(HIGH_REGION).is_some());
    assert!(glue.region("rom").is_none());
    assert!(glue.sink(OVERLAY_PIN, &[]).is_some());
    assert!(glue.sink("ovl", &[]).is_none(), "it is called `overlay`");
    assert!(schema().port_named(OVERLAY_PIN).is_some());
}
