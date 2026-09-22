//! The address decoder's own tests: which side answers, and how both the ROM
//! and main memory repeat through the window their select decodes.

use super::*;
use crate::core::props::{Link, Value};
use crate::core::space::{RamStore, RomStore, RomWrite};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::Mutex;
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
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

/// **Main memory repeats through the whole window**, because a board with less
/// than four megabytes on it leaves `A20`/`A21` out of the DRAM's decode.
///
/// The ROM's boot screen depends on it: the Plus ROM draws the insert-disk
/// icon through a pointer near the top of the four-megabyte window, and only
/// the fold puts that on the middle of a 1 MiB machine's screen buffer.
/// `src/dev/mac/glue.rs` has the argument and `tests/mac_plus.rs` the picture.
#[test]
fn memory_repeats_through_its_whole_window() {
    let (glue, ram) = wired();
    ram.write_at(0, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    clear_overlay(&glue);
    assert!(
        read(&glue, LOW_REGION, RAM_LEN - 4).is_ok(),
        "the last word"
    );
    for copy in [RAM_LEN, 2 * RAM_LEN, 3 * RAM_LEN] {
        assert_eq!(
            read(&glue, LOW_REGION, copy).unwrap(),
            [0x11, 0x22, 0x33, 0x44],
            "the copy at {copy:#x}"
        );
    }
    // And the fold is by the installed size, so the top of the window is the
    // top of memory — which is the property the screen buffer's address needs.
    ram.write_at(RAM_LEN - 4, &[0x55, 0x66, 0x77, 0x88])
        .unwrap();
    assert_eq!(
        read(&glue, LOW_REGION, LOW_WINDOW - 4).unwrap(),
        [0x55, 0x66, 0x77, 0x88],
        "the last word of the window is the last word of memory"
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

/// A **latching** overlay is cleared once and cannot be put back, and that is
/// a measurement rather than a convenience: a Macintosh Classic ROM drives
/// `PA4` high again five and a half virtual seconds into startup, long after
/// it has put its own exception vector table at address zero.
///
/// A reset still brings the overlay back, because the processor is about to
/// fetch a reset vector out of whatever answers there.
#[test]
fn a_latching_overlay_does_not_come_back_on_a_rising_edge() {
    let glue = Glue::new(&props().with("overlay", Value::Str("latching".into())))
        .expect("a mode this class knows");
    assert_eq!(glue.mode(), Mode::Latching);
    let rom: RegionRef = Arc::new(Region::rom(
        "rom",
        Arc::new(RomStore::zeroed(ROM_LEN)),
        RomWrite::Ignore,
    ));
    let ram: RegionRef = Arc::new(Region::ram("dram", Arc::new(RamStore::new(RAM_LEN))));
    glue.attach(&rom, &ram, 24).expect("both map");

    let pin = glue.sink(OVERLAY_PIN, &[WireId::new(1)]).expect("the pin");
    assert!(glue.overlaid(), "asserted at power-on, as the Plus's is");
    pin.sink.set_level(WireId::new(1), 0, Level::Low);
    assert!(!glue.overlaid(), "software cleared it");
    pin.sink.set_level(WireId::new(1), 0, Level::High);
    assert!(!glue.overlaid(), "and cannot put it back");
    glue.reset(ResetKind::Cold);
    assert!(glue.overlaid(), "but a reset can");

    // The default is the Plus's, where the pin *is* the decode.
    let level = Glue::new(&props()).expect("the default");
    assert_eq!(level.mode(), Mode::Level);
    let err = Glue::new(&props().with("overlay", Value::Str("sometimes".into())))
        .expect_err("a mode this class does not know")
        .to_string();
    assert!(err.contains("latching"), "{err}");
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
    for pin in [VIA_IRQ_PIN, SCC_IRQ_PIN] {
        assert!(glue.sink(pin, &[]).is_some(), "{pin}");
        assert!(schema().port_named(pin).is_some(), "{pin}");
    }
    for pin in IPL_PINS {
        assert!(schema().port_named(pin).is_some(), "{pin}");
    }
    let err = Device::connect(&glue, "ipl2", dummy_source())
        .expect_err("the third IPL pin is the programmer's switch, not ours")
        .to_string();
    assert!(err.contains("`ipl1`"), "{err}");
}

/// A wire source that drives nothing, for the pin-name check above.
fn dummy_source() -> WireSource {
    let wire = crate::core::wire::Wire::builder()
        .source(WireId::new(9))
        .build_shared();
    WireSource::new(wire, WireId::new(9))
}

/// **The two interrupt sources are priority-encoded, not summed.**
///
/// Level 3 is what Apple's ROM answers with a bare `RTE`, so a board that
/// presented it would livelock the first time a mouse moved during a vertical
/// blanking interrupt. The module docs have the measurement; this is the
/// arithmetic.
#[test]
fn both_interrupt_sources_at_once_is_the_higher_level_and_never_three() {
    let glue = Glue::new(&props()).unwrap();
    assert_eq!(glue.interrupt_level(), 0, "nothing is asking");
    glue.set_interrupt(false, true);
    assert_eq!(glue.interrupt_level(), 1, "the VIA alone");
    glue.set_interrupt(true, true);
    assert_eq!(glue.interrupt_level(), 2, "the SCC wins");
    glue.set_interrupt(false, false);
    assert_eq!(glue.interrupt_level(), 2, "the SCC alone");
    // And the VIA's request is not lost while the SCC has the bus: put it back
    // and take the SCC away, and the level falls to 1 rather than to 0.
    glue.set_interrupt(false, true);
    glue.set_interrupt(true, false);
    assert_eq!(glue.interrupt_level(), 1);
    glue.set_interrupt(false, false);
    assert_eq!(glue.interrupt_level(), 0);
}

/// The level arrives on the pins as a two-bit number, which is what a 68000's
/// `IPL` inputs are.
#[test]
fn the_level_reaches_the_pins_as_its_own_bits() {
    use crate::core::wire::{Pull, Wire};
    let glue = Glue::new(&props()).unwrap();
    let seen = Arc::new(Mutex::new([false; 2]));

    #[derive(Debug)]
    struct Bit {
        seen: Arc<Mutex<[bool; 2]>>,
        bit: usize,
    }
    impl WireSink for Bit {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.seen.lock()[self.bit] = level.is_high();
        }
    }

    for (bit, name) in IPL_PINS.iter().enumerate() {
        let id = WireId::new(bit as u64 + 1);
        let wire = Wire::builder()
            .source(id)
            .resolved(Pull::Down)
            .sink(
                Arc::new(Bit {
                    seen: Arc::clone(&seen),
                    bit,
                }),
                0,
            )
            .build_shared();
        Device::connect(&glue, name, WireSource::new(wire, id)).expect("an output");
        Device::announce(&glue, name);
    }
    assert_eq!(*seen.lock(), [false, false]);
    glue.set_interrupt(false, true);
    assert_eq!(*seen.lock(), [true, false], "level 1");
    glue.set_interrupt(true, true);
    assert_eq!(*seen.lock(), [false, true], "level 2, and IPL0 let go");
    glue.set_interrupt(true, false);
    glue.set_interrupt(false, false);
    assert_eq!(*seen.lock(), [false, false]);
}
