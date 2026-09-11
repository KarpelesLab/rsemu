//! `st.dbgmcu`'s unit tests.

use super::*;
use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::AtomicU32;
use crate::core::wire::{Wire, WireIdAllocator, WireSink};

/// An F407's, with the reset-value identifier.
fn dbgmcu() -> Dbgmcu {
    Dbgmcu::with_id(0x413, 0x1000)
}

fn peek(d: &Dbgmcu, offset: u64) -> u32 {
    let mut word = [0u8; 4];
    d.regs
        .read(offset, &mut word, MemAttrs::DEFAULT)
        .expect("a word read is legal");
    u32::from_le_bytes(word)
}

fn poke(d: &Dbgmcu, offset: u64, value: u32) {
    d.regs
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write is legal");
}

/// Follows the level of one freeze pin.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
    edges: AtomicU32,
}

impl WireSink for Probe {
    fn set_level(&self, _src: crate::core::wire::WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
        self.edges.fetch_add(1, Ordering::Relaxed);
    }
}

impl Probe {
    fn is_high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }
}

/// Wire `port` to a probe.
fn watch(d: &Dbgmcu, port: &str) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(d, port, WireSource::new(wire, id)).expect("a freeze pin");
    probe
}

/// A wire with one source, so a pin has something to drive.
fn dummy_source() -> WireSource {
    let id = crate::core::wire::WireId::new(9);
    WireSource::new(Wire::builder().source(id).build_shared(), id)
}

#[test]
fn idcode_is_read_only_and_carries_both_halves() {
    let d = dbgmcu();
    // "DEV_ID … 0x413 for STM32F405xx/07xx and STM32F415xx/17xx"; REV_ID is
    // the silicon revision in the top half (RM0090 §38.6.1).
    assert_eq!(peek(&d, 0x00), 0x1000_0413);
    poke(&d, 0x00, 0xffff_ffff);
    assert_eq!(peek(&d, 0x00), 0x1000_0413, "IDCODE is read-only");
    assert_eq!(d.idcode(), 0x1000_0413);
}

#[test]
fn cr_keeps_the_five_bits_the_manual_names_and_drops_the_rest() {
    let d = dbgmcu();
    assert_eq!(peek(&d, 0x04), 0, "reset value 0x0000 0000");
    poke(&d, 0x04, 0xffff_ffff);
    // DBG_SLEEP 0, DBG_STOP 1, DBG_STANDBY 2, TRACE_IOEN 5, TRACE_MODE[7:6]
    // (RM0090 §38.16.2). Everything else is reserved.
    assert_eq!(peek(&d, 0x04), 0x0000_00e7);
    poke(&d, 0x04, 0b011);
    assert_eq!(peek(&d, 0x04), 0b011, "DBG_SLEEP and DBG_STOP");
}

#[test]
fn the_freeze_words_read_back_only_the_bits_the_part_has() {
    let d = dbgmcu();
    poke(&d, 0x08, 0xffff_ffff);
    assert_eq!(peek(&d, 0x08), 0x06e0_1dff, "APB1_FZ, RM0090 §38.16.3");
    poke(&d, 0x0c, 0xffff_ffff);
    assert_eq!(peek(&d, 0x0c), 0x0007_0003, "APB2_FZ, RM0090 §38.16.4");
    // Bit 9 of APB1_FZ is reserved and sits between TIM14 and RTC, which is
    // the off-by-one a hand-written mask gets wrong.
    poke(&d, 0x08, 1 << 9);
    assert_eq!(peek(&d, 0x08), 0);
}

#[test]
fn a_freeze_pin_needs_the_bit_and_the_halt_together() {
    let d = dbgmcu();
    let iwdg = watch(&d, "iwdg");
    assert!(!iwdg.is_high(), "nothing is halted and nothing is set");

    // The bit alone freezes nothing: the chip is running.
    poke(&d, 0x08, 1 << 12);
    assert!(!iwdg.is_high(), "DBG_IWDG_STOP on a running core");
    assert!(!d.freezing("iwdg"));

    // The halt alone freezes nothing either.
    let bare = dbgmcu();
    let other = watch(&bare, "iwdg");
    Device::debug_halt(&bare, true);
    assert!(!other.is_high(), "halted, but the bit is clear");

    // Together.
    Device::debug_halt(&d, true);
    assert!(iwdg.is_high());
    assert!(d.freezing("iwdg"));

    // And releasing either lets it go.
    Device::debug_halt(&d, false);
    assert!(!iwdg.is_high());
    Device::debug_halt(&d, true);
    assert!(iwdg.is_high());
    poke(&d, 0x08, 0);
    assert!(!iwdg.is_high());
}

#[test]
fn each_bit_drives_its_own_pin_and_no_other() {
    let d = dbgmcu();
    let wwdg = watch(&d, "wwdg");
    let iwdg = watch(&d, "iwdg");
    let tim1 = watch(&d, "tim1");
    d.set_halted(true);

    // DBG_WWDG_STOP is bit 11 and DBG_IWDG_STOP is bit 12; swapping the two
    // is the mistake that would otherwise pass every other test here.
    poke(&d, 0x08, 1 << 11);
    assert!(wwdg.is_high() && !iwdg.is_high());
    poke(&d, 0x08, 1 << 12);
    assert!(iwdg.is_high() && !wwdg.is_high());

    // APB2's TIM1 is bit 0 of the *other* word, so setting APB1 bit 0 (TIM2)
    // must not move it.
    poke(&d, 0x08, 1);
    assert!(!tim1.is_high());
    poke(&d, 0x0c, 1);
    assert!(tim1.is_high());
}

#[test]
fn a_system_reset_does_not_clear_it_and_a_power_on_does() {
    let d = dbgmcu();
    let iwdg = watch(&d, "iwdg");
    poke(&d, 0x08, 1 << 12);
    poke(&d, 0x04, 0b111);
    d.set_halted(true);
    assert!(iwdg.is_high());

    // "This register is asynchronously reset by the POR … and not by the
    // system reset" (RM0090 §38.16.2). A watchdog reset the debugger is
    // stepping through must not throw away the bit that stops the next one.
    Device::reset(&d, ResetKind::Warm);
    assert_eq!(peek(&d, 0x08), 1 << 12);
    assert_eq!(peek(&d, 0x04), 0b111);
    assert!(iwdg.is_high());

    Device::reset(&d, ResetKind::Cold);
    assert_eq!(peek(&d, 0x08), 0);
    assert_eq!(peek(&d, 0x04), 0);
    assert!(!iwdg.is_high());
    assert!(d.halted(), "a power-on is not what un-halts a debugger");
}

#[test]
fn a_debug_read_is_the_same_read_and_a_debug_write_is_refused() {
    let d = dbgmcu();
    poke(&d, 0x08, 1 << 12);
    let mut word = [0u8; 4];
    d.regs
        .read(0x00, &mut word, MemAttrs::DEBUG)
        .expect("a debugger reads IDCODE first of all");
    assert_eq!(u32::from_le_bytes(word), 0x1000_0413);
    d.regs.read(0x08, &mut word, MemAttrs::DEBUG).unwrap();
    assert_eq!(u32::from_le_bytes(word), 1 << 12, "and nothing moved");

    assert_eq!(
        d.regs.write(0x08, &0u32.to_le_bytes(), MemAttrs::DEBUG),
        Err(BusError::BadAccess),
        "a debug write would stop a watchdog the guest is relying on"
    );
    assert_eq!(peek(&d, 0x08), 1 << 12);
}

#[test]
fn only_a_full_word_is_a_legal_access() {
    let d = dbgmcu();
    let mut byte = [0u8; 1];
    assert_eq!(
        d.regs.read(0x08, &mut byte, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        d.regs.constraints(),
        AccessConstraints::word(Width::U32, Endian::Little)
    );
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = dbgmcu();
    poke(&saved, 0x04, 0b111);
    poke(&saved, 0x08, (1 << 12) | (1 << 11) | 1);
    poke(&saved, 0x0c, 0x0007_0003);

    let mut shape = MachineShape::new();
    shape.add_device("dbgmcu", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dbgmcu", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = dbgmcu();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dbgmcu", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let offsets = [0x00, 0x04, 0x08, 0x0c];
    let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
    let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
    assert_eq!(before, after);

    // The halt is deliberately not in the chunk: it belongs to whatever is
    // debugging, and a snapshot restored by a plain run is not halted.
    saved.set_halted(true);
    assert!(!restored.halted());
}

#[test]
fn a_corrupt_snapshot_is_refused() {
    let saved = dbgmcu();
    saved.regs.state.lock().words[APB1_FZ] = 1 << 9; // a reserved bit
    let mut shape = MachineShape::new();
    shape.add_device("dbgmcu", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dbgmcu", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dbgmcu", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&dbgmcu(), &mut chunk.reader()).is_err());
}

#[test]
fn the_identifier_is_a_property_and_a_bad_one_is_refused() {
    let d = Dbgmcu::new(&Props::new()).unwrap();
    assert_eq!(d.idcode(), 0x1000_0413, "an F407 by default");

    let props = Props::new()
        .with("dev-id", Value::from(0x415u64))
        .with("rev-id", Value::from(0x1001u64));
    assert_eq!(Dbgmcu::new(&props).unwrap().idcode(), 0x1001_0415);

    assert!(Dbgmcu::new(&Props::new().with("dev-id", Value::from(0x1234u64))).is_err());
    assert!(Dbgmcu::new(&Props::new().with("rev-id", Value::from(0x1_0000u64))).is_err());
    assert!(Dbgmcu::new(&Props::new().with("devid", Value::from(1u64))).is_err());
}

#[test]
fn the_class_is_registrable_and_constructs_through_the_registry() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);
}

#[test]
fn the_schema_and_the_device_agree_about_pins_and_regions() {
    let d = dbgmcu();
    let schema = schema();
    for bit in FREEZE {
        assert!(
            schema.port_named(bit.pin).is_some(),
            "the schema is missing `{}`",
            bit.pin
        );
        assert!(
            Device::connect(&d, bit.pin, dummy_source()).is_ok(),
            "the device refuses `{}`",
            bit.pin
        );
    }
    assert!(schema.port_named("iwdg").is_some());
    assert!(
        schema.port_named("halted").is_none(),
        "a halt is not a wire"
    );
    assert!(Device::connect(&d, "sdio", dummy_source()).is_err());
    assert!(Device::region(&d, "").is_some());
    assert!(Device::region(&d, "regs").is_some());
    assert!(Device::region(&d, "idcode").is_none());
}
