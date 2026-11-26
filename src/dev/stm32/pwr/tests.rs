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

// ---------------------------------------------------------------------------
// The clock gate (`RCC_APB1ENR.PWREN`, `RCC_APB1RSTR.PWRRST`)
// ---------------------------------------------------------------------------

/// Wire one of the two gate inputs up, and hand back the pin and its source.
///
/// Drawing the `enable` wire is itself the assertion that RCC owns this clock,
/// so a controller comes back gated **off** from this call.
fn gate_pin(pwr: &Pwr, port: &str) -> (SinkPin, crate::core::wire::WireId) {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin = Device::sink(pwr, port, &[id]).expect("a gate pin a PWR has");
    (pin, id)
}

/// Connect `dbp` to a probe and hand it back.
fn watch_dbp(pwr: &Pwr) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn crate::core::wire::WireSink>, 0)
        .build_shared();
    Device::connect(pwr, DBP_PIN, WireSource::new(wire, id)).expect("dbp");
    sink
}

#[test]
fn a_gated_pwr_reads_as_zero_and_drops_every_write() {
    // GitHub issue #14, and the sequence this peripheral exists for: a
    // `SystemClock_Config` that forgets `RCC_APB1ENR.PWREN` finds a `VOS` that
    // will not take and a `VOSRDY` that never moves.
    let pwr = pwr(Variant::F42x);
    let cr = peek(&pwr, CR);
    assert_eq!(peek(&pwr, F4_CSR) & F4_CSR_VOSRDY, F4_CSR_VOSRDY);

    let (enable, id) = gate_pin(&pwr, gate::ENABLE_PIN);
    assert_eq!(peek(&pwr, CR), 0, "the whole block reads as zero");
    assert_eq!(peek(&pwr, F4_CSR), 0, "`VOSRDY` included");
    assert_eq!(
        peek_debug(&pwr, F4_CSR),
        0,
        "and so does a debug read, because that is what the bus returns"
    );

    // Not effective, and not a bus fault: an STM32's bridge answers.
    poke(&pwr, CR, (cr & !(0b11 << 14)) | (0b10 << 14));
    tick(&pwr, DELAY * 4);

    enable.sink.set_level(id, 0, Level::High);
    // Removing a clock is not a reset — `PWRRST` is what resets — so the
    // register file is exactly where it was, `VOS` never moved and the
    // regulator was never asked to.
    assert_eq!(peek(&pwr, CR), cr);
    assert_eq!(peek(&pwr, F4_CSR) & F4_CSR_VOSRDY, F4_CSR_VOSRDY);
}

#[test]
fn an_unwired_enable_leaves_the_pwr_clocked() {
    // Every machine file written before the gate draws no `enable`, and none of
    // them may go deaf because this landed.
    let pwr = pwr(Variant::L4);
    poke(&pwr, CR, (peek(&pwr, CR) & !(0b11 << 9)) | (0b10 << 9));
    assert_eq!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF);
    tick(&pwr, DELAY);
    assert_eq!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, 0);
}

#[test]
fn a_gated_regulator_transition_waits_rather_than_catching_up() {
    // The half a register model alone would get wrong. A transition in flight
    // when the clock goes has `ready-delay` ticks left, and it still has them
    // when the clock comes back — it does not finish the instant it returns,
    // and it does not finish while it is away either.
    let pwr = pwr(Variant::L4);
    let (enable, id) = gate_pin(&pwr, gate::ENABLE_PIN);
    enable.sink.set_level(id, 0, Level::High);

    poke(&pwr, CR, (peek(&pwr, CR) & !(0b11 << 9)) | (0b10 << 9));
    tick(&pwr, DELAY - 2);
    assert_eq!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF, "two to go");

    enable.sink.set_level(id, 0, Level::Low);
    // A hundred ticks with no clock is a hundred ticks the regulator did not
    // count, so the two it had left are still two.
    tick(&pwr, 100);
    enable.sink.set_level(id, 0, Level::High);
    assert_eq!(
        peek(&pwr, L4_SR2) & L4_SR2_VOSF,
        L4_SR2_VOSF,
        "it did not catch up on the way back"
    );
    tick(&pwr, 1);
    assert_eq!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, L4_SR2_VOSF);
    tick(&pwr, 1);
    assert_eq!(
        peek(&pwr, L4_SR2) & L4_SR2_VOSF,
        0,
        "and arrives on its own"
    );
}

#[test]
fn a_gated_pwr_holds_the_dbp_level_it_had() {
    // The one output this device drives is a **latched configuration bit**, not
    // something combinational the block computes each cycle. A guest cannot
    // move it while the clock is off — the write to `CR` is not effective — so
    // the honest answer is that it does not move. Driving it low would claim
    // that gating `APB1` re-arms the backup domain's write protection, which
    // is in the `VSWITCH` domain and is not gated by `APB1` at all.
    let pwr = pwr(Variant::F4);
    let dbp = watch_dbp(&pwr);
    poke(&pwr, CR, peek(&pwr, CR) | (1 << 8));
    assert!(dbp.is_high());

    let (enable, id) = gate_pin(&pwr, gate::ENABLE_PIN);
    poke(&pwr, CR, peek(&pwr, CR) & !(1 << 8));
    assert!(
        dbp.is_high(),
        "the write was not effective, so nothing moved"
    );
    assert!(pwr.dbp(), "and `CR.DBP` is still set behind the bridge");

    enable.sink.set_level(id, 0, Level::High);
    assert!(
        dbp.is_high(),
        "and it is still set when the clock comes back"
    );
    poke(&pwr, CR, peek(&pwr, CR) & !(1 << 8));
    assert!(!dbp.is_high(), "now the guest can close it again");
}

#[test]
fn the_reset_pin_puts_the_register_file_back_and_closes_the_backup_domain() {
    // `RCC_APB1RSTR.PWRRST`, as a level: the block is deaf while it stands and
    // comes back at its reset values when it is let go — and `CR`'s reset value
    // has `DBP` clear, which makes the reset line the one thing that *does*
    // move this output.
    let pwr = pwr(Variant::F42x);
    let dbp = watch_dbp(&pwr);
    poke(&pwr, CR, peek(&pwr, CR) | (1 << 8) | F4_CR_ODEN);
    assert!(dbp.is_high());

    let (reset, id) = gate_pin(&pwr, gate::RESET_PIN);
    // An `enable` nobody drew stays clocked, so `reset` alone is what shuts the
    // block here.
    reset.sink.set_level(id, 0, Level::High);
    assert!(!dbp.is_high(), "the reset line closed the backup domain");
    assert_eq!(peek(&pwr, CR), 0, "held in reset, the block is deaf");
    poke(&pwr, CR, 1 << 8);
    assert!(
        !dbp.is_high(),
        "and a write while it stands is not effective"
    );

    reset.sink.set_level(id, 0, Level::Low);
    assert_eq!(
        peek(&pwr, CR),
        Variant::F42x.cr_reset(),
        "and what it kept is the reset value"
    );
    assert_eq!(
        peek(&pwr, F4_CSR) & (F4_CSR_ODRDY | F4_CSR_ODSWRDY),
        0,
        "the over-drive it had armed went with it"
    );
    assert_eq!(
        Device::next_event_tick(&pwr),
        None,
        "and so did the deadline it was counting to"
    );
}

#[test]
fn a_reset_level_that_did_not_move_is_not_an_edge() {
    let pwr = pwr(Variant::F4);
    let (reset, id) = gate_pin(&pwr, gate::RESET_PIN);
    reset.sink.set_level(id, 0, Level::High);
    reset.sink.set_level(id, 0, Level::Low);
    poke(&pwr, CR, peek(&pwr, CR) | (1 << 8));
    reset.sink.set_level(id, 0, Level::Low);
    assert!(pwr.dbp(), "a low that was already low resets nothing");
}

// ---------------------------------------------------------------------------
// The U5 (RM0456 §10; ST CMSIS `stm32u585xx.h`, `PWR_TypeDef`)
// ---------------------------------------------------------------------------

/// `HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)`, as the
/// device sees it: `VOS = 11` with `BOOSTEN`, then a spin on `VOSRDY` and
/// another on `ACTVOSRDY`. Returns how many ticks the two spins took.
fn u5_scale1(pwr: &Pwr) -> u64 {
    let vosr = peek(pwr, U5_VOSR);
    poke(
        pwr,
        U5_VOSR,
        (vosr & !(U5_VOS | U5_VOSR_BOOSTEN)) | U5_VOS | U5_VOSR_BOOSTEN,
    );
    let mut ticks = 0;
    while peek(pwr, U5_VOSR) & U5_VOSR_VOSRDY == 0 {
        tick(pwr, 1);
        ticks += 1;
        assert!(ticks < 1000, "VOSRDY never came");
    }
    while peek(pwr, U5_SVMSR) & U5_SVMSR_ACTVOSRDY == 0 {
        tick(pwr, 1);
        ticks += 1;
        assert!(ticks < 1000, "ACTVOSRDY never came");
    }
    ticks
}

#[test]
fn a_u5_comes_out_of_reset_in_range_4_on_the_ldo() {
    let pwr = pwr(Variant::U5);
    assert_eq!(peek(&pwr, U5_VOSR), 0x0000_8000, "VOS = 00, VOSRDY");
    assert_eq!(
        peek(&pwr, U5_SVMSR),
        0x0000_8000,
        "ACTVOS = 00, ACTVOSRDY, LDO"
    );
    assert!(!pwr.dbp());
}

#[test]
fn the_u5_voltage_scaling_sequence_terminates() {
    let pwr = pwr(Variant::U5);
    let ticks = u5_scale1(&pwr);
    assert_eq!(ticks, DELAY, "both flags arrive together after the delay");
    assert_eq!(peek(&pwr, U5_SVMSR) & U5_VOS, U5_VOS, "ACTVOS = range 1");
    // `HAL_RCC_ClockConfig` then waits for the booster.
    assert_ne!(peek(&pwr, U5_VOSR) & U5_VOSR_BOOSTRDY, 0);

    // `HAL_RCC_OscConfig` drops `BOOSTEN` around a PLL1 reprogram: BOOSTRDY
    // goes with it at once, and comes back after the delay.
    poke(&pwr, U5_VOSR, peek(&pwr, U5_VOSR) & !U5_VOSR_BOOSTEN);
    assert_eq!(peek(&pwr, U5_VOSR) & U5_VOSR_BOOSTRDY, 0);
    poke(&pwr, U5_VOSR, peek(&pwr, U5_VOSR) | U5_VOSR_BOOSTEN);
    assert_eq!(peek(&pwr, U5_VOSR) & U5_VOSR_BOOSTRDY, 0);
    tick(&pwr, DELAY);
    assert_ne!(peek(&pwr, U5_VOSR) & U5_VOSR_BOOSTRDY, 0);

    // The guest cannot write the ready bits itself.
    poke(&pwr, U5_VOSR, U5_VOS);
    assert_eq!(peek(&pwr, U5_VOSR), U5_VOS | U5_VOSR_VOSRDY);
    poke(&pwr, U5_SVMSR, 0);
    assert_ne!(peek(&pwr, U5_SVMSR) & U5_SVMSR_ACTVOSRDY, 0);
}

#[test]
fn the_u5_smps_switch_is_reported_by_regs() {
    let pwr = pwr(Variant::U5);
    poke(&pwr, U5_CR3, U5_CR3_REGSEL);
    assert_eq!(peek(&pwr, U5_SVMSR) & U5_SVMSR_REGS, 0, "not yet");
    tick(&pwr, DELAY);
    assert_ne!(peek(&pwr, U5_SVMSR) & U5_SVMSR_REGS, 0, "on the SMPS");
    poke(&pwr, U5_CR3, 0);
    tick(&pwr, DELAY);
    assert_eq!(peek(&pwr, U5_SVMSR) & U5_SVMSR_REGS, 0, "back on the LDO");
}

#[test]
fn the_u5_dbp_is_its_own_register_and_drives_the_same_wire() {
    let pwr = pwr(Variant::U5);
    let probe = watch_dbp(&pwr);
    poke(&pwr, CR, 1 << 8);
    assert!(!probe.is_high(), "CR1 bit 8 is SRAM1PD on a U5, not DBP");
    poke(&pwr, U5_DBPR, 1);
    assert_eq!(peek(&pwr, U5_DBPR), 1, "the HAL's read-back spin ends");
    assert!(probe.is_high());
    pwr.set_dbp(false);
    assert!(!probe.is_high());
}

#[test]
fn the_u5_status_and_clear_registers() {
    let pwr = pwr(Variant::U5);
    // A supply monitor answers with its supply present.
    poke(&pwr, U5_SVMCR, (1 << 24) | (1 << 28));
    assert_eq!(peek(&pwr, U5_SVMSR) & U5_MONITORS, 1 << 24, "VDDUSBRDY");
    // `UCPD_DBDIS` and the pulls are storage.
    poke(&pwr, 0x2c, 1);
    assert_eq!(peek(&pwr, 0x2c), 1);
    poke(&pwr, 0x94, 0xffff);
    assert_eq!(peek(&pwr, 0x94), 0xffff);
    // `SR.CSSF` clears STOPF/SBF, `WUSCR` clears WUSR and reads zero.
    pwr.regs.state.lock().words[(U5_SR / 4) as usize] = 0b110;
    pwr.regs.state.lock().words[(U5_WUSR / 4) as usize] = 0b101;
    poke(&pwr, U5_SR, 1);
    assert_eq!(peek(&pwr, U5_SR), 0);
    poke(&pwr, U5_WUSCR, 0b001);
    assert_eq!(peek(&pwr, U5_WUSR), 0b100);
    assert_eq!(peek(&pwr, U5_WUSCR), 0);
}

#[test]
fn a_gated_u5_regulator_waits_for_ahb3en_pwren() {
    let pwr = pwr(Variant::U5);
    let (enable, id) = gate_pin(&pwr, gate::ENABLE_PIN);
    // Gated: the write does not take.
    poke(&pwr, U5_VOSR, U5_VOS);
    assert_eq!(peek(&pwr, U5_VOSR), 0);
    enable.sink.set_level(id, enable.line, Level::High);
    assert_eq!(peek(&pwr, U5_VOSR) & U5_VOS, 0, "the write was dropped");
    assert_eq!(u5_scale1(&pwr), DELAY);
}

#[test]
fn a_u5_snapshot_round_trips_to_identical_state() {
    let saved = pwr(Variant::U5);
    poke(&saved, U5_DBPR, 1);
    poke(&saved, U5_CR3, U5_CR3_REGSEL);
    poke(&saved, U5_VOSR, U5_VOS | U5_VOSR_BOOSTEN);
    tick(&saved, DELAY / 2);
    assert_eq!(peek(&saved, U5_VOSR) & U5_VOSR_VOSRDY, 0);

    let mut shape = MachineShape::new();
    shape.add_device("pwr", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("pwr", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let restored = pwr(Variant::U5);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("pwr", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let words = Variant::U5.register_bytes() / 4;
    let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(before, after);
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored)
    );
    assert!(restored.dbp());
    tick(&restored, DELAY);
    assert_eq!(
        peek(&restored, U5_VOSR) & (U5_VOSR_VOSRDY | U5_VOSR_BOOSTRDY),
        U5_VOSR_VOSRDY | U5_VOSR_BOOSTRDY
    );
    assert_ne!(peek(&restored, U5_SVMSR) & U5_SVMSR_REGS, 0);
    let props = Props::new().with("variant", Value::from("u5"));
    assert_eq!(Pwr::new(&props).unwrap().variant(), Variant::U5);
}

// ---------------------------------------------------------------------------
// The WB (RM0434; ST CMSIS `stm32wb55xx.h`, `PWR_TypeDef`; reset values from
// ST's `STM32WB55_CM4.svd`)
// ---------------------------------------------------------------------------

/// Connect `c2boot` to a probe and hand it back.
fn watch_c2boot(pwr: &Pwr) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn crate::core::wire::WireSink>, 0)
        .build_shared();
    Device::connect(pwr, C2BOOT_PIN, WireSource::new(wire, id)).expect("c2boot");
    sink
}

/// A WB's `C2CR1`.
const WB_C2CR1: u64 = 0x80;

#[test]
fn a_wb_comes_out_of_reset_as_the_svd_says() {
    let pwr = pwr(Variant::WB);
    for (name, reg, value) in [
        ("CR1", CR, 0x0000_0200),
        ("CR2", L4_CR2, 0),
        ("CR3", L4_CR3, 0x0000_8000),
        ("CR4", WB_CR4, 0),
        ("SR1", L4_SR1, 0),
        ("SR2", L4_SR2, WB_SR2_SMPSBF),
        ("SCR", L4_SCR, 0),
        ("CR5", WB_CR5, 0x0000_4270),
        ("C2CR1", WB_C2CR1, 0),
        ("C2CR3", WB_C2CR3, 0x0000_8000),
        ("EXTSCR", WB_EXTSCR, 0),
    ] {
        assert_eq!(peek(&pwr, reg), value, "{name}");
    }
    assert!(!pwr.dbp());
    assert!(!pwr.c2boot());
    assert_eq!(
        Device::region(&pwr, "").map(|r| r.len()),
        Some(0x8c),
        "through EXTSCR"
    );
}

#[test]
fn the_wb_voltage_scaling_sequence_terminates() {
    // `HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2)`, then
    // back to range 1: write `VOS`, spin while `VOSF` is set.
    let pwr = pwr(Variant::WB);
    for vos in [0b10, 0b01] {
        poke(&pwr, CR, (peek(&pwr, CR) & !(0b11 << 9)) | (vos << 9));
        assert_ne!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, 0, "changing");
        let mut spins = 0;
        while peek(&pwr, L4_SR2) & L4_SR2_VOSF != 0 {
            tick(&pwr, 1);
            spins += 1;
            assert!(spins < 100, "VOSF never cleared");
        }
        assert_eq!(spins, DELAY);
        assert_eq!((peek(&pwr, CR) >> 9) & 0b11, vos);
    }
    // `SR2` is the hardware's.
    poke(&pwr, L4_SR2, 0xffff_ffff);
    assert_eq!(peek(&pwr, L4_SR2), WB_SR2_SMPSBF);
}

#[test]
fn the_wb_low_power_run_regulator_answers_reglpf() {
    let pwr = pwr(Variant::WB);
    // `HAL_PWREx_EnableLowPowerRunMode`: `LPR` set, nobody waits, but the
    // flag gets there.
    poke(&pwr, CR, peek(&pwr, CR) | WB_CR1_LPR);
    assert_eq!(peek(&pwr, L4_SR2) & WB_SR2_REGLPF, 0);
    tick(&pwr, DELAY);
    assert_ne!(peek(&pwr, L4_SR2) & WB_SR2_REGLPF, 0);
    // `HAL_PWREx_DisableLowPowerRunMode`: `LPR` clear, spin on `REGLPF`.
    poke(&pwr, CR, peek(&pwr, CR) & !WB_CR1_LPR);
    let mut spins = 0;
    while peek(&pwr, L4_SR2) & WB_SR2_REGLPF != 0 {
        tick(&pwr, 1);
        spins += 1;
        assert!(spins < 100, "REGLPF never cleared");
    }
    // The voltage scaling flag was never involved.
    assert_eq!(peek(&pwr, L4_SR2) & L4_SR2_VOSF, 0);
}

#[test]
fn the_wb_smps_mode_is_reported_by_sr2() {
    let pwr = pwr(Variant::WB);
    // `LL_PWR_SMPS_SetMode(LL_PWR_SMPS_STEP_DOWN)`.
    poke(&pwr, WB_CR5, peek(&pwr, WB_CR5) | WB_CR5_SMPSEN);
    assert_eq!(peek(&pwr, L4_SR2) & 0b11, WB_SR2_SMPSBF, "not there yet");
    tick(&pwr, DELAY);
    assert_eq!(peek(&pwr, L4_SR2) & 0b11, WB_SR2_SMPSF, "stepping down");
    // The trims are storage and move no flag.
    poke(&pwr, WB_CR5, WB_CR5_SMPSEN | 0x0000_0127);
    assert_eq!(peek(&pwr, WB_CR5), WB_CR5_SMPSEN | 0x0000_0127);
    assert_eq!(Device::next_event_tick(&pwr), None);
    // And back to bypass.
    poke(&pwr, WB_CR5, 0x0000_4270);
    tick(&pwr, DELAY);
    assert_eq!(peek(&pwr, L4_SR2) & 0b11, WB_SR2_SMPSBF);
}

#[test]
fn the_wb_dbp_drives_the_wire_the_rcc_reads() {
    let pwr = pwr(Variant::WB);
    let sink = watch_dbp(&pwr);
    // `HAL_PWR_EnableBkUpAccess`, then the WB RCC HAL's own spin on
    // `PWR->CR1 & DBP`.
    poke(&pwr, CR, peek(&pwr, CR) | (1 << 8));
    assert_ne!(peek(&pwr, CR) & (1 << 8), 0);
    assert!(pwr.dbp());
    assert!(sink.is_high());
    poke(&pwr, CR, peek(&pwr, CR) & !(1 << 8));
    assert!(!sink.is_high());
}

#[test]
fn c2boot_follows_cr4_and_nothing_else_drives_it() {
    let pwr = pwr(Variant::WB);
    let boot = watch_c2boot(&pwr);
    assert!(!boot.is_high(), "CPU2 is held out of reset");
    // The rest of `CR4` is storage and does not boot anything.
    poke(&pwr, WB_CR4, 0x31f);
    assert!(!boot.is_high());
    assert_eq!(peek(&pwr, WB_CR4), 0x31f);
    // `HAL_PWREx_ReleaseCore(PWR_CORE_CPU2)`.
    poke(&pwr, WB_CR4, peek(&pwr, WB_CR4) | WB_CR4_C2BOOT);
    assert!(boot.is_high());
    assert!(pwr.c2boot());
    // `HAL_PWREx_HoldCore` clears it. The level falls with the bit; that a
    // booted CPU2 keeps running is its model's to honour — it takes the edge.
    poke(&pwr, WB_CR4, peek(&pwr, WB_CR4) & !WB_CR4_C2BOOT);
    assert!(!boot.is_high());
    assert_eq!(peek(&pwr, WB_CR4), 0x31f);
    // A system reset holds CPU2 again.
    poke(&pwr, WB_CR4, WB_CR4_C2BOOT);
    assert!(boot.is_high());
    Device::reset(&pwr, ResetKind::Warm);
    assert!(!boot.is_high());
    assert_eq!(peek(&pwr, WB_CR4), 0);
}

#[test]
fn only_a_wb_has_a_c2boot_pin() {
    for variant in [Variant::F4, Variant::L4, Variant::U5] {
        let pwr = pwr(variant);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(
            Device::connect(&pwr, C2BOOT_PIN, WireSource::new(wire, id)).is_err(),
            "{variant:?}"
        );
        assert!(!pwr.c2boot());
    }
    let pwr = pwr(Variant::WB);
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    assert!(Device::connect(&pwr, "c3boot", WireSource::new(wire, id)).is_err());
    assert_eq!(
        schema().port_named(C2BOOT_PIN).map(|p| p.dir),
        Some(PortDir::Out)
    );
}

#[test]
fn the_wb_status_and_clear_registers() {
    let pwr = pwr(Variant::WB);
    // Nothing raises these, so plant them as the hardware would.
    {
        let mut state = pwr.regs.state.lock();
        *state.word_mut(L4_SR1) = 0xff9f;
        *state.word_mut(WB_EXTSCR) = 0xef00;
    }
    // `SR1` is read-only.
    poke(&pwr, L4_SR1, 0);
    assert_eq!(peek(&pwr, L4_SR1), 0xff9f);
    // `SCR` clears what it names, and `WUFI` (15) is not among them.
    poke(&pwr, L4_SCR, 0x0003);
    assert_eq!(peek(&pwr, L4_SR1), 0xff9c);
    poke(&pwr, L4_SCR, 0xffff_ffff);
    assert_eq!(peek(&pwr, L4_SR1), 1 << 15);
    assert_eq!(peek(&pwr, L4_SCR), 0, "write-only");

    // `EXTSCR`: C1CSSF, C2CSSF and CCRPF each clear their own.
    poke(&pwr, WB_EXTSCR, 0xffff_0000);
    assert_eq!(peek(&pwr, WB_EXTSCR), 0xef00, "the flags are read-only");
    poke(&pwr, WB_EXTSCR, 1);
    assert_eq!(peek(&pwr, WB_EXTSCR), 0xec00, "C1SBF and C1STOPF");
    poke(&pwr, WB_EXTSCR, 2);
    assert_eq!(peek(&pwr, WB_EXTSCR), 0xe000, "C2SBF and C2STOPF");
    poke(&pwr, WB_EXTSCR, 4);
    assert_eq!(peek(&pwr, WB_EXTSCR), 0xc000, "CRPF; C1DS/C2DS stay");

    // CPU2's low-power settings are storage.
    poke(&pwr, WB_C2CR1, 0xc034);
    poke(&pwr, WB_C2CR3, 0x961f);
    assert_eq!(peek(&pwr, WB_C2CR1), 0xc034);
    assert_eq!(peek(&pwr, WB_C2CR3), 0x961f);
    // And so are the pulls, port H included.
    poke(&pwr, 0x58, 0x3);
    assert_eq!(peek(&pwr, 0x58), 0x3);
}

#[test]
fn a_wb_snapshot_round_trips_to_identical_state() {
    let saved = pwr(Variant::WB);
    poke(
        &saved,
        CR,
        (peek(&saved, CR) | (1 << 8) | WB_CR1_LPR) & !(0b11 << 9) | (0b10 << 9),
    );
    poke(&saved, WB_CR4, WB_CR4_C2BOOT | 1);
    poke(&saved, WB_CR5, peek(&saved, WB_CR5) | WB_CR5_SMPSEN);
    poke(&saved, WB_C2CR1, 0x4);
    tick(&saved, DELAY / 2);
    // Mid-transition: VOSF, REGLPF and the SMPS flags all pending.
    assert_ne!(peek(&saved, L4_SR2) & L4_SR2_VOSF, 0);

    let mut shape = MachineShape::new();
    shape.add_device("pwr", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("pwr", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let restored = pwr(Variant::WB);
    let boot = watch_c2boot(&restored);
    assert!(!boot.is_high());
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("pwr", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let words = Variant::WB.register_bytes() / 4;
    let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(before, after);
    let held = *saved.regs.state.lock();
    assert_eq!(held, *restored.regs.state.lock(), "deadlines and all");
    assert!(restored.dbp());
    assert!(boot.is_high(), "the load republished c2boot");
    tick(&restored, DELAY);
    let sr2 = peek(&restored, L4_SR2);
    assert_eq!(sr2 & L4_SR2_VOSF, 0);
    assert_ne!(sr2 & WB_SR2_REGLPF, 0);
    assert_eq!(sr2 & 0b11, WB_SR2_SMPSF);
    let props = Props::new().with("variant", Value::from("wb"));
    assert_eq!(Pwr::new(&props).unwrap().variant(), Variant::WB);
    let variant = schema();
    let variant = variant
        .props
        .iter()
        .find(|p| p.name == "variant")
        .expect("variant");
    assert!(variant.values.iter().any(|v| v == "wb"));
}

#[test]
fn smpsen_follows_cr5_at_once_for_the_rcc() {
    // RM0434 §6.4.9: the RCC's `SMPSSWS` is "no clock" whenever
    // `PWR_CR5.SMPSEN` is clear, so the bit itself — not the SMPS flags that
    // follow it a regulator transition later — is what the wire carries.
    let pwr = pwr(Variant::WB);
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn crate::core::wire::WireSink>, 0)
        .build_shared();
    Device::connect(&pwr, SMPSEN_PIN, WireSource::new(wire, id)).expect("smpsen");
    assert!(!sink.is_high(), "SMPSEN resets clear (RM0434 §5.6.8)");
    poke(&pwr, WB_CR5, peek(&pwr, WB_CR5) | WB_CR5_SMPSEN);
    assert!(sink.is_high(), "at once");
    assert_eq!(peek(&pwr, L4_SR2) & 0b11, WB_SR2_SMPSBF, "the flags lag");
    Device::reset(&pwr, ResetKind::Warm);
    assert!(!sink.is_high());

    // A WB's pin only.
    let other = Pwr::with_config(Variant::L4, DELAY);
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    assert!(Device::connect(&other, SMPSEN_PIN, WireSource::new(wire, id)).is_err());
    assert_eq!(
        schema().port_named(SMPSEN_PIN).map(|p| p.dir),
        Some(PortDir::Out)
    );
}
