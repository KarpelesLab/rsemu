//! `st.rcc`, register by register.
//!
//! The device is lazily advanced, so nothing here has a scheduler: a test
//! drives [`Device::advance_to`] itself, which is exactly what the scheduler
//! would do when a guest access syncs the device. That keeps the tests
//! deterministic and independent of the machine layer.

use super::*;

use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::space::MemAttrs;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireIdAllocator};

// -- F4 register offsets, as a test writes them ------------------------------

const CR: u64 = 0x00;
const PLLCFGR: u64 = 0x04;
const CFGR: u64 = 0x08;
const AHB1RSTR: u64 = 0x10;
const APB1ENR: u64 = 0x40;
const BDCR: u64 = 0x70;
const CSR: u64 = 0x74;

const CR_HSION: u32 = 1 << 0;
const CR_HSIRDY: u32 = 1 << 1;
const CR_HSEON: u32 = 1 << 16;
const CR_HSERDY: u32 = 1 << 17;
const CR_PLLON: u32 = 1 << 24;
const CR_PLLRDY: u32 = 1 << 25;

/// How many ticks the tests give an oscillator.
const DELAY: u64 = 16;

fn f4() -> Rcc {
    Rcc::with_config(Variant::F4, Frequencies::default(), DELAY)
}

fn l4() -> Rcc {
    Rcc::with_config(Variant::L4, Frequencies::default(), DELAY)
}

fn peek(rcc: &Rcc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    rcc.regs
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a word read");
    u32::from_le_bytes(buf)
}

fn peek_debug(rcc: &Rcc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    rcc.regs
        .read(offset, &mut buf, MemAttrs::DEBUG)
        .expect("a debug word read");
    u32::from_le_bytes(buf)
}

fn poke(rcc: &Rcc, offset: u64, value: u32) {
    rcc.regs
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write");
}

/// Run the device forward `ticks` ticks, as a scheduler catch-up would.
fn tick(rcc: &Rcc, ticks: u64) {
    let now = Device::current_tick(rcc);
    Device::advance_to(rcc, now + ticks);
}

/// A wire with one source, so a pin has something to drive.
fn probe() -> (Arc<Wire>, WireSource, Arc<LevelProbe>) {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(LevelProbe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
        .build_shared();
    (Arc::clone(&wire), WireSource::new(wire, id), sink)
}

/// Somewhere for a driven level to land.
#[derive(Debug, Default)]
struct LevelProbe {
    high: AtomicU32,
}

impl LevelProbe {
    fn is_high(&self) -> bool {
        self.high.load(Ordering::Relaxed) != 0
    }
}

impl WireSink for LevelProbe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.high
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Ready bits
// ---------------------------------------------------------------------------

#[test]
fn hserdy_follows_hseon_after_the_startup_delay() {
    let rcc = f4();
    // Out of reset the part runs from the HSI and says so.
    assert_eq!(
        peek(&rcc, CR) & (CR_HSION | CR_HSIRDY),
        CR_HSION | CR_HSIRDY
    );
    assert_eq!(peek(&rcc, CR) & CR_HSERDY, 0);

    poke(&rcc, CR, peek(&rcc, CR) | CR_HSEON);
    assert_eq!(
        peek(&rcc, CR) & CR_HSERDY,
        0,
        "a crystal is not ready on the very next read"
    );
    tick(&rcc, DELAY - 1);
    assert_eq!(peek(&rcc, CR) & CR_HSERDY, 0, "nor one tick early");
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, CR) & CR_HSERDY, CR_HSERDY, "and ready on time");

    // "Cleared by hardware when the oscillator is switched off."
    poke(&rcc, CR, peek(&rcc, CR) & !CR_HSEON);
    assert_eq!(peek(&rcc, CR) & CR_HSERDY, 0);
}

#[test]
fn a_ready_bit_cannot_be_set_by_the_guest() {
    let rcc = f4();
    poke(&rcc, CR, !CR_HSEON);
    assert_eq!(
        peek(&rcc, CR) & CR_HSERDY,
        0,
        "HSERDY is the hardware's answer, not storage"
    );
    // The writable bits did land.
    assert_eq!(peek(&rcc, CR) & (1 << 19), 1 << 19, "CSSON is writable");
}

#[test]
fn the_l4_hsi_ready_bit_is_two_places_above_its_enable() {
    // RM0351 §6.4.1: HSION is bit 8 and HSIRDY is bit 10, because bit 9 is
    // HSIKERON. Getting this wrong makes every L4 startup hang.
    let rcc = l4();
    poke(&rcc, CR, peek(&rcc, CR) | (1 << 8));
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, CR) & (1 << 10), 1 << 10);
    assert_eq!(peek(&rcc, CR) & (1 << 9), 0, "HSIKERON was not asked for");
}

// ---------------------------------------------------------------------------
// The switch
// ---------------------------------------------------------------------------

#[test]
fn sws_does_not_switch_to_a_pll_that_is_not_ready() {
    let rcc = f4();
    // SW = 10 is the PLL on an F4, and the PLL has not been started.
    poke(&rcc, CFGR, 0b10);
    tick(&rcc, 8);
    assert_eq!(
        peek(&rcc, CFGR) & 0b11,
        0,
        "the write is ignored, as on hardware"
    );
    assert_eq!(peek(&rcc, CFGR) & 0b1100, 0, "so SWS never moves");

    // Start it, and the same write is now accepted.
    poke(&rcc, CR, peek(&rcc, CR) | CR_PLLON);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, CR) & CR_PLLRDY, CR_PLLRDY);
    poke(&rcc, CFGR, 0b10);
    assert_eq!(peek(&rcc, CFGR) & 0b11, 0b10, "SW took");
    assert_eq!(peek(&rcc, CFGR) & 0b1100, 0, "SWS is one tick behind");
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, CFGR) & 0b1100, 0b10 << 2, "and then it agrees");
}

#[test]
fn the_vendor_clock_config_sequence_terminates() {
    // The `SystemInit`/`SystemClock_Config` shape, as a bounded loop: every
    // spin below is the one a RAM window makes infinite.
    let rcc = f4();
    let mut budget = 10_000;
    let mut spin = |f: &mut dyn FnMut() -> bool| {
        while !f() {
            tick(&rcc, 1);
            budget -= 1;
            assert!(budget > 0, "the spin did not terminate");
        }
    };

    poke(&rcc, CR, peek(&rcc, CR) | CR_HSEON);
    spin(&mut || peek(&rcc, CR) & CR_HSERDY != 0);

    // HSE 8 MHz, M = 8, N = 336, P = /2, Q = 7 — the Discovery board's 168 MHz
    // (RM0090 §7.3.2, and the numbers `machines/stm32f407.machine` writes).
    poke(&rcc, PLLCFGR, (1 << 22) | (7 << 24) | (336 << 6) | 8);
    poke(&rcc, CR, peek(&rcc, CR) | CR_PLLON);
    spin(&mut || peek(&rcc, CR) & CR_PLLRDY != 0);

    // HPRE = /1, PPRE1 = /4, PPRE2 = /2, SW = PLL.
    poke(&rcc, CFGR, (5 << 10) | (4 << 13) | 0b10);
    spin(&mut || peek(&rcc, CFGR) & 0b1100 == 0b10 << 2);

    let clocks = rcc.clocks();
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(168_000_000),
        "8 MHz / 8 * 336 / 2"
    );
    assert_eq!(
        clocks.rate(ClockOutput::HCLK),
        Rational::integer(168_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::PCLK1),
        Rational::integer(42_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::PCLK2),
        Rational::integer(84_000_000)
    );
    // "If APBx prescaler is 1, timer clock = PCLKx, else 2 × PCLKx."
    assert_eq!(
        clocks.rate(ClockOutput::TIMCLK1),
        Rational::integer(84_000_000)
    );
    // VCO / Q = 336 MHz / 7 = 48 MHz, which is what the USB core wants.
    assert_eq!(
        clocks.rate(ClockOutput::PLL48),
        Rational::integer(48_000_000)
    );
}

#[test]
fn the_l4_pll_output_frequency_is_what_the_registers_say() {
    // MSI at 4 MHz, M = 1, N = 40, R = /2 → 80 MHz, which is the L4's maximum
    // and the configuration every L4 example uses (RM0351 §6.4.4).
    let rcc = l4();
    let clocks = rcc.clocks();
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(4_000_000),
        "an L4 boots on a 4 MHz MSI"
    );

    // PLLSRC = MSI, PLLM = 0 (÷1), PLLN = 40, PLLR = 00 (÷2), PLLREN.
    poke(&rcc, 0x0c, (1 << 24) | (40 << 8) | 1);
    poke(&rcc, CR, peek(&rcc, CR) | CR_PLLON);
    tick(&rcc, DELAY);
    // SW = 11 is the PLL on an L4.
    poke(&rcc, CFGR, 0b11);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, CFGR) & 0b1100, 0b11 << 2);
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(80_000_000)
    );
}

#[test]
fn a_prescaler_change_moves_the_wire_without_touching_the_source() {
    let rcc = f4();
    let clocks = rcc.clocks();
    let before = clocks.generation();
    // HPRE = /4 (field 9) on the HSI the part boots from.
    poke(&rcc, CFGR, 9 << 4);
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(16_000_000),
        "the source did not move"
    );
    assert_eq!(clocks.rate(ClockOutput::HCLK), Rational::integer(4_000_000));
    assert!(clocks.generation() > before, "and a consumer can notice");
}

// ---------------------------------------------------------------------------
// The gate wires
// ---------------------------------------------------------------------------

#[test]
fn a_peripheral_enable_bit_is_a_wire() {
    let rcc = f4();
    let (_wire, source, probe) = probe();
    // `APB1ENR` bit 17 is USART2EN (RM0090 §7.3.13).
    Device::connect(&rcc, "apb1en17", source).expect("apb1en17");
    assert!(!probe.is_high(), "gated out of reset");

    poke(&rcc, APB1ENR, 1 << 17);
    assert!(probe.is_high(), "the peripheral has its clock");
    assert_eq!(rcc.gate("apb1en", 17), Some(true));

    poke(&rcc, APB1ENR, 0);
    assert!(!probe.is_high());
    assert_eq!(rcc.gate("apb1en", 17), Some(false));
}

#[test]
fn a_peripheral_reset_bit_is_a_separate_wire() {
    let rcc = f4();
    let (_wire, source, probe) = probe();
    // `AHB1RSTR` bit 0 is GPIOARST.
    Device::connect(&rcc, "ahb1rst0", source).expect("ahb1rst0");
    poke(&rcc, AHB1RSTR, 1);
    assert!(probe.is_high(), "the reset line is pulled");
    poke(&rcc, AHB1RSTR, 0);
    assert!(!probe.is_high(), "and released");
}

#[test]
fn a_bank_this_variant_does_not_have_is_refused() {
    let rcc = f4();
    let (_w, source, _p) = probe();
    // `apb1enb` is the L4's `APB1ENR2`; an F4 has one APB1 enable word.
    assert!(Device::connect(&rcc, "apb1enb0", source).is_err());
    let (_w, source, _p) = probe();
    assert!(Device::connect(&rcc, "apb1en32", source).is_err());
    let (_w, source, _p) = probe();
    assert!(Device::connect(&rcc, "nonsense", source).is_err());

    let rcc = l4();
    let (_w, source, probe) = probe();
    Device::connect(&rcc, "apb1enb0", source).expect("the L4 has APB1ENR2");
    // `APB1ENR2` is at +0x5c on an L4.
    poke(&rcc, 0x5c, 1);
    assert!(probe.is_high());
}

#[test]
fn the_bank_pin_names_cannot_collide() {
    // `apb1en21` is `APB1ENR` bit 21 and nothing else. Were the L4's second
    // word called `apb1en2`, this spelling would be two pins.
    let layout = Variant::L4.layout();
    assert_eq!(
        parse_bank_pin(layout, "apb1en21"),
        bank_key(layout, "apb1en", 21)
    );
    assert_eq!(
        parse_bank_pin(layout, "apb1enb1"),
        bank_key(layout, "apb1enb", 1)
    );
    assert_ne!(
        parse_bank_pin(layout, "apb1en21"),
        parse_bank_pin(layout, "apb1enb1")
    );
}

// ---------------------------------------------------------------------------
// The backup domain
// ---------------------------------------------------------------------------

#[test]
fn bdcr_is_read_only_until_pwr_dbp_is_set() {
    let rcc = f4();
    rcc.set_dbp(false);
    // RTCSEL = LSE, RTCEN, LSEON.
    poke(&rcc, BDCR, (1 << 15) | (1 << 8) | 1);
    assert_eq!(peek(&rcc, BDCR), 0, "every bit of the write was dropped");

    rcc.set_dbp(true);
    poke(&rcc, BDCR, (1 << 15) | (1 << 8) | 1);
    assert_eq!(
        peek(&rcc, BDCR) & ((1 << 15) | (1 << 8) | 1),
        (1 << 15) | (1 << 8) | 1
    );
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, BDCR) & 0b10, 0b10, "LSERDY came up");
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_768),
        "and the RTC has its clock"
    );
}

#[test]
fn an_unwired_dbp_leaves_the_backup_domain_open() {
    // A board with no `st.pwr` has nothing modelling the protection. A domain
    // that could never be opened would be a board bug wearing a device bug's
    // clothes, so it is open and the module documentation says so.
    let rcc = f4();
    poke(&rcc, BDCR, 1);
    assert_eq!(peek(&rcc, BDCR) & 1, 1);
}

#[test]
fn bdrst_clears_the_backup_domain() {
    let rcc = f4();
    rcc.set_dbp(true);
    poke(&rcc, BDCR, (1 << 15) | (1 << 8) | 1);
    tick(&rcc, DELAY);
    assert_ne!(peek(&rcc, BDCR) & 0b10, 0);

    let (_w, source, probe) = probe();
    Device::connect(&rcc, RTCEN_PIN, source).expect("rtcen");
    assert!(probe.is_high(), "RTCEN is a wire the RTC can watch");

    poke(&rcc, BDCR, 1 << 16);
    assert_eq!(peek(&rcc, BDCR), 1 << 16, "nothing else survives BDRST");
    assert!(!probe.is_high());
    assert_eq!(rcc.clocks().rate(ClockOutput::RTCCLK), Rational::integer(0));
}

#[test]
fn a_warm_reset_keeps_the_backup_domain_and_a_cold_one_does_not() {
    let rcc = f4();
    rcc.set_dbp(true);
    poke(&rcc, BDCR, (1 << 8) | 1);
    Device::reset(&rcc, ResetKind::Warm);
    assert_eq!(peek(&rcc, BDCR) & 1, 1, "battery-backed state survives");
    Device::reset(&rcc, ResetKind::Cold);
    assert_eq!(peek(&rcc, BDCR), 0, "a power-on does not");
}

// ---------------------------------------------------------------------------
// Reset causes
// ---------------------------------------------------------------------------

#[test]
fn an_iwdg_reset_sets_iwdgrstf_and_rmvf_clears_it() {
    let rcc = f4();
    // A power-on leaves BOR, pin and POR set (RM0090 §7.3.21).
    assert_eq!(peek(&rcc, CSR) & 0xff00_0000, 0x0e00_0000);

    let src = WireId::new(1);
    let pin = Device::sink(&rcc, "iwdgrst", &[src]).expect("iwdgrst");
    assert_eq!(pin.line, 29);
    pin.sink.set_level(src, 29, Level::High);
    assert_eq!(peek(&rcc, CSR) & (1 << 29), 1 << 29);

    // "RMVF: remove reset flag … cleared by software by writing 1", and the
    // bit itself never reads back.
    poke(&rcc, CSR, 1 << 24);
    assert_eq!(peek(&rcc, CSR) & 0xff00_0000, 0);
    assert_eq!(peek(&rcc, CSR) & (1 << 24), 0);
}

#[test]
fn the_l4_puts_its_reset_flags_and_rmvf_somewhere_else() {
    let rcc = l4();
    assert!(
        Device::sink(&rcc, "porrst", &[WireId::new(1)]).is_none(),
        "an L4 has no PORRSTF"
    );
    let src = WireId::new(1);
    let pin = Device::sink(&rcc, "oblrst", &[src]).expect("oblrst");
    assert_eq!(pin.line, 25, "OBLRSTF is bit 25 on an L4");
    pin.sink.set_level(src, 25, Level::High);
    assert_eq!(peek(&rcc, 0x94) & (1 << 25), 1 << 25);
    poke(&rcc, 0x94, 1 << 23);
    assert_eq!(peek(&rcc, 0x94) & 0xff00_0000, 0);
}

// ---------------------------------------------------------------------------
// The framework contract
// ---------------------------------------------------------------------------

#[test]
fn a_debug_access_changes_nothing() {
    let rcc = f4();
    poke(&rcc, CR, peek(&rcc, CR) | CR_HSEON);
    // Invariant 5: a debug read does not advance the device, so it cannot make
    // a ready bit come true that a guest read has not yet earned.
    assert_eq!(peek_debug(&rcc, CR) & CR_HSERDY, 0);
    assert_eq!(Device::current_tick(&rcc), 0);

    // And a debug write is refused rather than starting an oscillator.
    assert_eq!(
        rcc.regs.write(CR, &0u32.to_le_bytes(), MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
    assert_eq!(peek(&rcc, CR) & CR_HSEON, CR_HSEON);
}

#[test]
fn only_a_full_word_is_a_legal_access() {
    let rcc = f4();
    let mut byte = [0u8; 1];
    assert_eq!(
        rcc.regs.read(CR, &mut byte, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        rcc.regs.constraints(),
        AccessConstraints::word(Width::U32, Endian::Little)
    );
}

#[test]
fn the_next_event_is_always_in_the_future() {
    let rcc = f4();
    assert_eq!(Device::next_event_tick(&rcc), None);
    poke(&rcc, CR, peek(&rcc, CR) | CR_HSEON);
    assert_eq!(Device::next_event_tick(&rcc), Some(DELAY));
    tick(&rcc, DELAY);
    assert_eq!(
        Device::next_event_tick(&rcc),
        None,
        "a deadline that has passed is not an event"
    );
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = f4();
    saved.set_dbp(true);
    poke(&saved, CR, peek(&saved, CR) | CR_HSEON);
    tick(&saved, DELAY);
    poke(&saved, PLLCFGR, (1 << 22) | (7 << 24) | (336 << 6) | 8);
    poke(&saved, CR, peek(&saved, CR) | CR_PLLON);
    tick(&saved, DELAY);
    poke(&saved, CFGR, (5 << 10) | (4 << 13) | 0b10);
    poke(&saved, APB1ENR, 1 << 17);
    poke(&saved, BDCR, (1 << 15) | (1 << 8) | 1);
    // Saved mid-transition on purpose: the LSE's deadline and the pending
    // `SWS` switch are both architectural, and a snapshot that dropped them
    // would come back with a spin that never ends.
    assert_eq!(peek(&saved, CFGR) & 0b1100, 0);

    let mut shape = MachineShape::new();
    shape.add_device("rcc", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("rcc", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = f4();
    restored.set_dbp(true);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("rcc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let words = Variant::F4.register_bytes() / 4;
    let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(before, after, "every register came across");
    assert_eq!(
        Device::current_tick(&saved),
        Device::current_tick(&restored)
    );
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored),
        "and so did what is still pending"
    );
    assert_eq!(
        saved.clocks().rate(ClockOutput::SYSCLK),
        restored.clocks().rate(ClockOutput::SYSCLK),
        "the rate table is derived, and it was derived again"
    );

    // The pending work really does complete on the other side.
    tick(&restored, DELAY);
    assert_eq!(peek(&restored, CFGR) & 0b1100, 0b10 << 2);
    assert_eq!(peek(&restored, BDCR) & 0b10, 0b10);
    assert_eq!(
        restored.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(168_000_000)
    );
}

#[test]
fn a_property_this_class_does_not_know_is_a_typo() {
    let props = Props::new().with("variant", Value::from("l4"));
    assert_eq!(Rcc::new(&props).unwrap().variant(), Variant::L4);
    assert_eq!(Rcc::new(&Props::new()).unwrap().variant(), Variant::F4);
    assert!(Rcc::new(&Props::new().with("variant", Value::from("h7"))).is_err());
    assert!(Rcc::new(&Props::new().with("readydelay", Value::from(4u64))).is_err());

    // A board with a 25 MHz can gets 25 MHz arithmetic.
    let rcc = Rcc::new(&Props::new().with("hse", Value::from(25_000_000u64))).unwrap();
    poke(&rcc, CR, peek(&rcc, CR) | CR_HSEON);
    tick(&rcc, DELAY);
    poke(&rcc, CFGR, 1);
    tick(&rcc, 1);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(25_000_000)
    );
}

#[test]
fn the_class_is_registrable_and_agrees_with_its_schema() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);

    let schema = schema();
    assert_eq!(schema.port_named(DBP_PIN).map(|p| p.dir), Some(PortDir::In));
    assert_eq!(
        schema.port_named(RTCEN_PIN).map(|p| p.dir),
        Some(PortDir::Out)
    );
    for pin in ["ahb1en0", "apb2en31", "apb1rstb7", "iwdgrst"] {
        assert!(schema.port_named(pin).is_some(), "{pin} is declared");
    }
    assert!(schema.port_named("apb2en32").is_none());

    let rcc = f4();
    assert!(Device::region(&rcc, "").is_some());
    assert!(Device::region(&rcc, "regs").is_some());
    assert!(Device::region(&rcc, "fifo").is_none());
    assert!(Device::export(&rcc, ExportId::CLOCK_TREE).is_some());
    assert!(Device::export(&rcc, ExportId::TIMEBASE).is_none());
    assert!(Device::is_lazy(&rcc));
}

#[test]
fn the_export_is_the_handle_a_consumer_downcasts_to() {
    let rcc = f4();
    let export = Device::export(&rcc, ExportId::CLOCK_TREE).expect("clock tree");
    let handle = Arc::clone(export.opaque().expect("opaque"))
        .downcast::<Clocks>()
        .expect("a Clocks");
    assert_eq!(
        handle.rate(ClockOutput::SYSCLK),
        Rational::integer(16_000_000),
        "the HSI the part boots on"
    );
    assert!(Arc::ptr_eq(&handle, &rcc.clocks()));
}

#[cfg(feature = "dev-stm32-pwr")]
#[test]
fn pwr_dbp_reaches_the_rcc_over_a_wire() {
    // The coupling the issue asks for, wired the way a board would wire it:
    // `wire pwr.dbp -> rcc.dbp`. Neither device holds a lock across the hop —
    // PWR drives the level after its critical section and the RCC's sink
    // stores it in an atomic — so the ranked lock order is never tested.
    use crate::dev::stm32::pwr::{self, Pwr};

    let rcc = f4();
    let pwr = Pwr::with_config(pwr::Variant::F4, 8);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Device::sink(&rcc, DBP_PIN, &[id]).expect("rcc.dbp");
    let wire = Wire::builder()
        .source(id)
        .sink_weak(Arc::downgrade(&sink.sink), sink.line)
        .build_shared();
    Device::connect(&pwr, pwr::DBP_PIN, WireSource::new(wire, id)).expect("pwr.dbp");

    poke(&rcc, BDCR, 1);
    assert_eq!(peek(&rcc, BDCR), 0, "protected while DBP is clear");

    // `PWR_CR.DBP` is bit 8 on both families, and this takes the write path
    // firmware takes, wire included.
    pwr.set_dbp(true);
    assert!(pwr.dbp());

    poke(&rcc, BDCR, 1);
    assert_eq!(peek(&rcc, BDCR) & 1, 1, "and open once DBP arrives");
}
