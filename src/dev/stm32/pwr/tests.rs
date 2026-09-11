//! `st.pwr`, register by register.
//!
//! As in [`rcc`](super::super::rcc), the device is lazily advanced and the
//! tests drive [`Device::advance_to`] themselves rather than standing a
//! scheduler up.

use super::*;

use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::space::MemAttrs;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireIdAllocator};
use alloc::vec::Vec;

/// How many ticks the tests give the regulator.
const DELAY: u64 = 8;

fn pwr(variant: Variant) -> Pwr {
    Pwr::with_config(variant, DELAY)
}

fn peek(pwr: &Pwr, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    pwr.regs
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a word read");
    u32::from_le_bytes(buf)
}

fn peek_debug(pwr: &Pwr, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    pwr.regs
        .read(offset, &mut buf, MemAttrs::DEBUG)
        .expect("a debug word read");
    u32::from_le_bytes(buf)
}

fn poke(pwr: &Pwr, offset: u64, value: u32) {
    pwr.regs
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write");
}

fn tick(pwr: &Pwr, ticks: u64) {
    let now = Device::current_tick(pwr);
    Device::advance_to(pwr, now + ticks);
}

#[test]
fn dbp_is_clear_out_of_reset_and_drives_its_wire() {
    let pwr = pwr(Variant::F4);
    assert!(!pwr.dbp());

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn crate::core::wire::WireSink>, 0)
        .build_shared();
    Device::connect(&pwr, DBP_PIN, WireSource::new(wire, id)).expect("dbp");
    assert!(!sink.is_high(), "and announces the level it is already at");

    poke(&pwr, CR, peek(&pwr, CR) | (1 << 8));
    assert!(pwr.dbp());
    assert!(sink.is_high());

    poke(&pwr, CR, peek(&pwr, CR) & !(1 << 8));
    assert!(!sink.is_high());
}

#[test]
fn one_pin_is_all_a_pwr_drives() {
    let pwr = pwr(Variant::L4);
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    assert!(Device::connect(&pwr, "wkup1", WireSource::new(wire, id)).is_err());
}

#[test]
fn an_f4_reports_the_regulator_as_ready_and_an_l4_as_changing() {
    // The polarity trap: `CSR.VOSRDY` goes *set* when the F4's regulator has
    // arrived, and `SR2.VOSF` goes *clear* when the L4's has. Firmware waits
    // on opposite edges and a model that averaged them would hang one family.
    let f4 = pwr(Variant::F42x);
    assert_eq!(peek(&f4, F4_CSR) & F4_CSR_VOSRDY, F4_CSR_VOSRDY);
    // `VOS` is [15:14] on an F42x: move it from 11 to 10.
    poke(&f4, CR, (peek(&f4, CR) & !(0b11 << 14)) | (0b10 << 14));
    assert_eq!(peek(&f4, F4_CSR) & F4_CSR_VOSRDY, 0, "the regulator moved");
    tick(&f4, DELAY);
    assert_eq!(peek(&f4, F4_CSR) & F4_CSR_VOSRDY, F4_CSR_VOSRDY);

    let l4 = pwr(Variant::L4);
    assert_eq!(peek(&l4, L4_SR2) & L4_SR2_VOSF, 0, "ready at reset");
    // `VOS` is [10:9] on an L4: range 1 to range 2.
    poke(&l4, CR, (peek(&l4, CR) & !(0b11 << 9)) | (0b10 << 9));
    assert_eq!(peek(&l4, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF, "changing");
    tick(&l4, DELAY - 1);
    assert_eq!(peek(&l4, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF);
    tick(&l4, 1);
    assert_eq!(peek(&l4, L4_SR2) & L4_SR2_VOSF, 0, "and arrived");
}

#[test]
fn an_f405_has_no_over_drive_and_an_f429_does() {
    // On an F405/407 `VOS` is the single bit 14, so writing the F42x's
    // two-bit field changes nothing above it and `ODEN` is reserved.
    let f405 = pwr(Variant::F4);
    assert_eq!(peek(&f405, CR), 0x0000_4000);
    poke(&f405, CR, peek(&f405, CR) | F4_CR_ODEN);
    tick(&f405, DELAY);
    assert_eq!(
        peek(&f405, F4_CSR) & F4_CSR_ODRDY,
        0,
        "an F405 answers no over-drive"
    );

    let f429 = pwr(Variant::F42x);
    assert_eq!(peek(&f429, CR), 0x0000_c000);
    poke(&f429, CR, peek(&f429, CR) | F4_CR_ODEN);
    assert_eq!(peek(&f429, F4_CSR) & F4_CSR_ODRDY, 0);
    tick(&f429, DELAY);
    assert_eq!(peek(&f429, F4_CSR) & F4_CSR_ODRDY, F4_CSR_ODRDY);
    poke(&f429, CR, peek(&f429, CR) | F4_CR_ODSWEN);
    tick(&f429, DELAY);
    assert_eq!(peek(&f429, F4_CSR) & F4_CSR_ODSWRDY, F4_CSR_ODSWRDY);
    // Switched off again, the ready bits go with it.
    poke(&f429, CR, peek(&f429, CR) & !(F4_CR_ODEN | F4_CR_ODSWEN));
    assert_eq!(peek(&f429, F4_CSR) & (F4_CSR_ODRDY | F4_CSR_ODSWRDY), 0);
}

#[test]
fn the_l4_clear_register_is_write_only() {
    let l4 = pwr(Variant::L4);
    // `SR1` is read-only, so plant a flag the way the hardware would.
    {
        let mut state = l4.regs.state.lock();
        *state.word_mut(L4_SR1) = 0x0000_0101;
    }
    assert_eq!(peek(&l4, L4_SR1), 0x0000_0101);
    poke(&l4, L4_SR1, 0);
    assert_eq!(
        peek(&l4, L4_SR1),
        0x0000_0101,
        "a write to SR1 does nothing"
    );
    poke(&l4, L4_SCR, 0x0000_0101);
    assert_eq!(peek(&l4, L4_SR1), 0, "SCR is what clears it");
    assert_eq!(peek(&l4, L4_SCR), 0, "and SCR itself reads as zero");
}

#[test]
fn the_l4plus_has_cr5_and_the_l4_does_not() {
    assert_eq!(pwr(Variant::L4).variant().register_bytes(), 0x60);
    let plus = pwr(Variant::L4Plus);
    assert_eq!(plus.variant().register_bytes(), 0x84);
    // "R1MODE: … 1: Range 1 normal mode" is the reset state.
    assert_eq!(peek(&plus, L4_CR5), 0x0000_0100);
    poke(&plus, L4_CR5, 0);
    assert_eq!(peek(&plus, L4_CR5), 0, "the boost mode is selectable");
}

#[test]
fn a_debug_access_changes_nothing() {
    let pwr = pwr(Variant::L4);
    poke(&pwr, CR, (peek(&pwr, CR) & !(0b11 << 9)) | (0b10 << 9));
    assert_eq!(peek_debug(&pwr, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF);
    assert_eq!(
        Device::current_tick(&pwr),
        0,
        "a debug read advanced nothing"
    );
    assert_eq!(
        pwr.regs.write(CR, &0u32.to_le_bytes(), MemAttrs::DEBUG),
        Err(BusError::BadAccess),
        "a debug write would unlock the backup domain"
    );
}

#[test]
fn only_a_full_word_is_a_legal_access() {
    let pwr = pwr(Variant::F4);
    let mut byte = [0u8; 1];
    assert_eq!(
        pwr.regs.read(CR, &mut byte, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        pwr.regs.constraints(),
        AccessConstraints::word(Width::U32, Endian::Little)
    );
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = pwr(Variant::F42x);
    poke(&saved, CR, peek(&saved, CR) | (1 << 8) | F4_CR_ODEN);
    poke(
        &saved,
        CR,
        (peek(&saved, CR) & !(0b11 << 14)) | (0b01 << 14),
    );
    // Saved with both the regulator and the over-drive still in transition.
    assert_eq!(peek(&saved, F4_CSR) & (F4_CSR_VOSRDY | F4_CSR_ODRDY), 0);

    let mut shape = MachineShape::new();
    shape.add_device("pwr", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("pwr", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = pwr(Variant::F42x);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("pwr", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let words = Variant::F42x.register_bytes() / 4;
    let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(before, after);
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored),
        "what was still pending came across"
    );
    assert!(restored.dbp(), "and so did DBP");

    tick(&restored, DELAY);
    assert_eq!(
        peek(&restored, F4_CSR) & (F4_CSR_VOSRDY | F4_CSR_ODRDY),
        F4_CSR_VOSRDY | F4_CSR_ODRDY,
        "the transition completed on the other side"
    );
}

#[test]
fn a_property_this_class_does_not_know_is_a_typo() {
    assert_eq!(Pwr::new(&Props::new()).unwrap().variant(), Variant::F4);
    let props = Props::new().with("variant", Value::from("l4plus"));
    assert_eq!(Pwr::new(&props).unwrap().variant(), Variant::L4Plus);
    assert!(Pwr::new(&Props::new().with("variant", Value::from("h7"))).is_err());
    assert!(Pwr::new(&Props::new().with("ready_delay", Value::from(4u64))).is_err());
}

#[test]
fn the_class_is_registrable_and_agrees_with_its_schema() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);

    let schema = schema();
    assert_eq!(
        schema.port_named(DBP_PIN).map(|p| p.dir),
        Some(PortDir::Out)
    );
    assert!(schema.port_named("vosrdy").is_none());

    let pwr = pwr(Variant::F4);
    assert!(Device::region(&pwr, "").is_some());
    assert!(Device::region(&pwr, "regs").is_some());
    assert!(Device::region(&pwr, "pins").is_none());
    assert!(Device::is_lazy(&pwr));
}

/// Somewhere for a driven level to land.
#[derive(Debug, Default)]
struct Probe {
    high: crate::core::sync::AtomicU32,
}

impl Probe {
    fn is_high(&self) -> bool {
        self.high.load(Ordering::Relaxed) != 0
    }
}

impl crate::core::wire::WireSink for Probe {
    fn set_level(&self, _src: crate::core::wire::WireId, _line: u32, level: Level) {
        self.high
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}
