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
const CIR: u64 = 0x0c;
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
// The clock interrupt
// ---------------------------------------------------------------------------

#[test]
fn a_ready_bit_raises_the_interrupt_only_when_its_enable_is_set() {
    // RM0090 §7.3.5: "Set by hardware when the HSE clock becomes stable and
    // HSERDYIE is set." The enable is part of the setting condition, not a
    // mask over an already-set flag.
    let rcc = f4();
    let (_wire, source, probe) = probe();
    Device::connect(&rcc, IRQ_PIN, source).expect("an RCC drives `irq`");
    assert!(!probe.is_high(), "nothing has happened yet");

    poke(&rcc, CR, CR_HSION | CR_HSEON);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, CR) & CR_HSERDY, CR_HSERDY, "the crystal arrived");
    assert_eq!(peek(&rcc, CIR), 0, "and nobody asked to be told");
    assert!(!probe.is_high());

    // HSERDYIE is bit 3 + 8; arm it and start the PLL instead.
    poke(&rcc, CIR, 1 << (4 + 8));
    poke(&rcc, CR, CR_HSION | CR_HSEON | CR_PLLON);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, CR) & CR_PLLRDY, CR_PLLRDY);
    assert_eq!(peek(&rcc, CIR) & 0xff, 1 << 4, "PLLRDYF, and only it");
    assert!(probe.is_high(), "and the line is up");
}

#[test]
fn a_cir_flag_is_cleared_by_the_bit_sixteen_places_above_it() {
    let rcc = f4();
    let (_wire, source, probe) = probe();
    Device::connect(&rcc, IRQ_PIN, source).expect("`irq`");
    poke(&rcc, CIR, 1 << (4 + 8));
    poke(&rcc, CR, CR_HSION | CR_PLLON);
    tick(&rcc, DELAY);
    assert!(probe.is_high());

    // A write of one to PLLRDYC (bit 20) clears PLLRDYF (bit 4); the flags
    // themselves are read-only, so writing one there does nothing at all.
    poke(&rcc, CIR, (1 << (4 + 8)) | (1 << 4));
    assert_eq!(
        peek(&rcc, CIR) & 0xff,
        1 << 4,
        "a flag is not guest-writable"
    );
    assert!(probe.is_high());

    poke(&rcc, CIR, (1 << (4 + 8)) | (1 << (4 + 16)));
    assert_eq!(peek(&rcc, CIR) & 0xff, 0, "cleared");
    assert_eq!(
        peek(&rcc, CIR),
        1 << (4 + 8),
        "and the clear bit reads zero"
    );
    assert!(!probe.is_high(), "so the line goes down");
}

#[test]
fn the_l4_splits_the_same_logic_into_three_registers() {
    // RM0351 §6.4.5–§6.4.7: `CIER` at 0x18, `CIFR` at 0x1c (read-only) and
    // `CICR` at 0x20 (write one to clear, reads zero). The bit positions are
    // the L4's own — `MSIRDYF` is bit 2 and `PLLRDYF` is bit 5, not the F4's 4.
    const CIER: u64 = 0x18;
    const CIFR: u64 = 0x1c;
    const CICR: u64 = 0x20;
    let rcc = l4();
    let (_wire, source, probe) = probe();
    Device::connect(&rcc, IRQ_PIN, source).expect("`irq`");

    poke(&rcc, CIER, 1 << 5);
    poke(&rcc, CR, (1 << 0) | (1 << 24));
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, CIFR), 1 << 5, "PLLRDYF");
    assert!(probe.is_high());

    poke(&rcc, CIFR, 0);
    assert_eq!(peek(&rcc, CIFR), 1 << 5, "`CIFR` is read-only");
    poke(&rcc, CICR, 1 << 5);
    assert_eq!(peek(&rcc, CIFR), 0);
    assert_eq!(peek(&rcc, CICR), 0, "`CICR` holds nothing of its own");
    assert!(!probe.is_high());
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

// -- the clock-control seam ---------------------------------------------------

/// An instance wired to four fabricated domains, bypassing the machine layer:
/// `bind` is what fills this in on a real board, and its only input is a
/// `DomainId` per named output.
fn driving() -> (Rcc, Arc<ClockControl>) {
    let rcc = Rcc::with_outputs(
        Variant::F4,
        Frequencies::default(),
        DELAY,
        alloc::vec![
            (ClockOutput::SYSCLK, String::from("sysclk")),
            (ClockOutput::HCLK, String::from("hclk")),
            (ClockOutput::PCLK1, String::from("pclk1")),
            (ClockOutput::PCLK2, String::from("pclk2")),
            (ClockOutput::TIMCLK1, String::from("timclk1")),
            (ClockOutput::TIMCLK2, String::from("timclk2")),
        ],
    );
    let mut forest = crate::core::clock::ClockForest::new();
    let hse = forest
        .add_oscillator("hse", Rational::integer(8_000_000))
        .expect("a crystal");
    let sys = forest.add_domain("sysclk", hse, 2, 1).expect("hse * 2");
    let hclk = forest.add_domain("hclk", sys, 1, 1).expect("sysclk");
    let pclk1 = forest.add_domain("pclk1", hclk, 1, 4).expect("hclk / 4");
    let pclk2 = forest.add_domain("pclk2", hclk, 1, 2).expect("hclk / 2");
    let tim1 = forest
        .add_domain("timclk1", pclk1, 2, 1)
        .expect("pclk1 * 2");
    let tim2 = forest
        .add_domain("timclk2", pclk2, 2, 1)
        .expect("pclk2 * 2");
    *rcc.regs.domains.lock() = alloc::vec![
        (ClockOutput::SYSCLK, sys),
        (ClockOutput::HCLK, hclk),
        (ClockOutput::PCLK1, pclk1),
        (ClockOutput::PCLK2, pclk2),
        (ClockOutput::TIMCLK1, tim1),
        (ClockOutput::TIMCLK2, tim2),
    ];
    let control = Arc::new(ClockControl::new());
    Device::attach_clock_control(&rcc, Arc::clone(&control));
    (rcc, control)
}

#[test]
fn attaching_the_seam_asks_for_the_reset_tree_at_once() {
    // A part comes out of reset on HSI with every prescaler at one, and a
    // machine file's declared ratios are not that. Asking only on the next
    // register write would leave the domains wrong until the guest happened to
    // touch `CFGR`.
    let (_rcc, control) = driving();
    let batch = control.take();
    assert_eq!(batch.len(), 6, "every named output is asked for");
    // 16 MHz off an 8 MHz reference, then unity all the way down: HSI, no
    // prescaling. Domain ids are 1..4 in creation order.
    assert_eq!(batch[0].ratio(), Rational::integer(2));
    for r in &batch[1..] {
        assert_eq!(r.ratio(), Rational::ONE);
    }
}

#[test]
fn the_pll_and_the_prescalers_become_exact_ratios_of_their_parents() {
    // RM0090 §7.3.2's own worked example: HSE 8 MHz, M=8, N=336, P=2. The
    // device already computed 168 MHz; what this asserts is what it asks the
    // forest for, which is a ratio and never a frequency.
    let (rcc, control) = driving();
    let _ = control.take();

    poke(&rcc, CR, CR_HSION | CR_HSEON);
    rcc.advance_to(DELAY + 1);
    poke(&rcc, PLLCFGR, 8 | (336 << 6) | (1 << 22));
    poke(&rcc, CR, CR_HSION | CR_HSEON | CR_PLLON);
    rcc.advance_to(2 * DELAY + 2);
    // HPRE 1, PPRE1 4, PPRE2 2, SW = PLL.
    poke(&rcc, CFGR, (5 << 10) | (4 << 13) | 2);
    rcc.advance_to(2 * DELAY + 4);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(168_000_000)
    );

    let batch = control.take();
    let of = |out: ClockOutput| {
        let domain = rcc
            .regs
            .domains
            .lock()
            .iter()
            .find(|(o, _)| *o == out)
            .map(|(_, d)| *d)
            .expect("a driven output");
        batch
            .iter()
            .find(|r| r.domain == domain)
            .expect("a request for it")
            .ratio()
    };
    // SYSCLK against the crystal, HCLK against SYSCLK, and both APB clocks
    // against HCLK — never against each other, and never against absolute time.
    assert_eq!(of(ClockOutput::SYSCLK), Rational::integer(21));
    assert_eq!(of(ClockOutput::HCLK), Rational::ONE);
    assert_eq!(of(ClockOutput::PCLK1), Rational::new(1, 4).unwrap());
    assert_eq!(of(ClockOutput::PCLK2), Rational::new(1, 2).unwrap());
    // "If the APB prescaler is 1 the timer clock is PCLKx, else 2 x PCLKx"
    // (RM0090 7.2). Both prescalers are above one here, so both timer clocks
    // are their bus clock doubled — and each is rated against *its own* bus.
    assert_eq!(of(ClockOutput::TIMCLK1), Rational::integer(2));
    assert_eq!(of(ClockOutput::TIMCLK2), Rational::integer(2));
}

#[test]
fn an_output_nobody_named_is_driven_by_nothing() {
    // Every board in `machines/` is in this state, and none of them may change
    // because the seam exists.
    let rcc = f4();
    let control = Arc::new(ClockControl::new());
    Device::attach_clock_control(&rcc, Arc::clone(&control));
    poke(&rcc, CFGR, 7 << 10);
    rcc.advance_to(DELAY + 1);
    assert!(
        !control.is_pending(),
        "an unwired controller drives nothing"
    );
}

// ---------------------------------------------------------------------------
// The U5 (RM0456 §11; ST CMSIS `stm32u585xx.h`, `RCC_TypeDef`)
// ---------------------------------------------------------------------------

mod u5 {
    use super::*;

    pub(super) const CR: u64 = 0x00;
    pub(super) const ICSCR1: u64 = 0x08;
    pub(super) const CFGR1: u64 = 0x1c;
    pub(super) const CFGR2: u64 = 0x20;
    pub(super) const CFGR3: u64 = 0x24;
    pub(super) const PLL1CFGR: u64 = 0x28;
    pub(super) const PLL1DIVR: u64 = 0x34;
    pub(super) const PLL1FRACR: u64 = 0x38;
    pub(super) const CIER: u64 = 0x50;
    pub(super) const CIFR: u64 = 0x54;
    pub(super) const CICR: u64 = 0x58;
    pub(super) const AHB2ENR2: u64 = 0x90;
    pub(super) const AHB3ENR: u64 = 0x94;
    pub(super) const APB3ENR: u64 = 0xa8;
    pub(super) const APB3RSTR: u64 = 0x80;
    pub(super) const BDCR: u64 = 0xf0;
    pub(super) const CSR: u64 = 0xf4;

    pub(super) const CR_MSISRDY: u32 = 1 << 2;
    pub(super) const CR_MSIKON: u32 = 1 << 4;
    pub(super) const CR_HSION: u32 = 1 << 8;
    pub(super) const CR_HSIRDY: u32 = 1 << 10;
    pub(super) const CR_HSI48ON: u32 = 1 << 12;
    pub(super) const CR_HSI48RDY: u32 = 1 << 13;
    pub(super) const CR_HSEON: u32 = 1 << 16;
    pub(super) const CR_HSERDY: u32 = 1 << 17;
    pub(super) const CR_PLL1ON: u32 = 1 << 24;
    pub(super) const CR_PLL1RDY: u32 = 1 << 25;
    pub(super) const CR_PLL2ON: u32 = 1 << 26;
    pub(super) const CR_PLL2RDY: u32 = 1 << 27;

    pub(super) fn rcc() -> Rcc {
        Rcc::with_config(
            Variant::U5,
            Frequencies {
                // The B-U585I-IOT02A's 16 MHz can.
                hse: 16_000_000,
                ..Frequencies::default()
            },
            DELAY,
        )
    }

    /// `PLL1DIVR` from the HAL's *real* factors, each stored as value − 1.
    pub(super) fn divr(n: u32, p: u32, q: u32, r: u32) -> u32 {
        (n - 1) | ((p - 1) << 9) | ((q - 1) << 16) | ((r - 1) << 24)
    }

    /// Run `SystemClock_Config` as CubeMX writes it for a U585 at 160 MHz:
    /// MSIS at 4 MHz into PLL1 with M = 1, N = 80, R = 2.
    pub(super) fn clock_config(rcc: &Rcc) {
        // PLL1SRC = MSIS, PLL1RGE = 4–8 MHz, M = 1, PLL1MBOOST = ÷1, REN|QEN.
        poke(rcc, PLL1CFGR, 1 | (1 << 2) | (1 << 17) | (1 << 18));
        poke(rcc, PLL1DIVR, divr(80, 2, 2, 2));
        poke(rcc, CR, peek(rcc, CR) | CR_PLL1ON);
        let mut spins = 0;
        while peek(rcc, CR) & CR_PLL1RDY == 0 {
            tick(rcc, 1);
            spins += 1;
            assert!(spins < 1000, "PLL1RDY never came");
        }
        // HPRE = PPRE1 = PPRE2 = ÷1, PPRE3 = ÷2.
        poke(rcc, CFGR2, 0);
        poke(rcc, CFGR3, 4 << 4);
        poke(rcc, CFGR1, (peek(rcc, CFGR1) & !0b11) | 0b11);
        while peek(rcc, CFGR1) & 0b1100 != 0b1100 {
            tick(rcc, 1);
            spins += 1;
            assert!(spins < 1000, "SWS never followed SW");
        }
    }
}

#[test]
fn a_u5_boots_on_the_4mhz_msis() {
    let rcc = u5::rcc();
    assert_eq!(
        peek(&rcc, u5::CR),
        0x0000_0035,
        "MSIS and MSIK on and ready"
    );
    assert_eq!(peek(&rcc, u5::CFGR1) & 0b1111, 0, "SW = SWS = MSIS");
    assert_eq!(peek(&rcc, u5::PLL1DIVR), 0x0101_0280);
    let clocks = rcc.clocks();
    for out in [
        ClockOutput::SYSCLK,
        ClockOutput::HCLK,
        ClockOutput::PCLK1,
        ClockOutput::PCLK2,
        ClockOutput::PCLK3,
    ] {
        assert_eq!(clocks.rate(out), Rational::integer(4_000_000), "{out}");
    }
}

#[test]
fn the_u5_msis_range_follows_msirgsel() {
    let rcc = u5::rcc();
    // With MSIRGSEL clear the range is `CSR.MSISSRANGE`; writing a new
    // `ICSCR1.MSISRANGE` does nothing until MSIRGSEL says to look there.
    poke(&rcc, u5::ICSCR1, 0x0400_0000);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(4_000_000)
    );
    // Range 0 is 48 MHz.
    poke(&rcc, u5::ICSCR1, 1 << 23);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(48_000_000)
    );
    // Range 6 is 4 MHz ÷ 3, exactly, not ST's rounded 1 330 000.
    poke(&rcc, u5::ICSCR1, (1 << 23) | (6 << 28));
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(4_000_000).checked_scale(1, 3).unwrap()
    );
}

#[test]
fn the_u5_vendor_clock_config_reaches_160mhz() {
    let rcc = u5::rcc();
    u5::clock_config(&rcc);
    let clocks = rcc.clocks();
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(160_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::HCLK),
        Rational::integer(160_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::PCLK1),
        Rational::integer(160_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::PCLK3),
        Rational::integer(80_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::TIMCLK1),
        Rational::integer(160_000_000)
    );
    // PLL1Q = 320 MHz / 2.
    assert_eq!(
        clocks.rate(ClockOutput::PLL48),
        Rational::integer(160_000_000)
    );

    // And APB1 at ÷4 gives the timers twice that.
    poke(&rcc, u5::CFGR2, 5 << 4);
    assert_eq!(
        clocks.rate(ClockOutput::PCLK1),
        Rational::integer(40_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::TIMCLK1),
        Rational::integer(80_000_000)
    );
}

#[test]
fn the_u5_pll1_takes_its_fraction_only_behind_fracen() {
    let rcc = u5::rcc();
    poke(&rcc, u5::CR, peek(&rcc, u5::CR) | u5::CR_HSEON);
    tick(&rcc, DELAY);
    assert_ne!(peek(&rcc, u5::CR) & u5::CR_HSERDY, 0);
    // HSE 16 MHz, M = 2, N = 20 + 4096/8192, R = 1: VCO = 8 × 20.5 = 164.
    poke(&rcc, u5::PLL1DIVR, u5::divr(20, 2, 2, 1));
    poke(&rcc, u5::PLL1FRACR, 4096 << 3);
    poke(&rcc, u5::PLL1CFGR, 3 | (1 << 8) | (1 << 18));
    poke(&rcc, u5::CR, peek(&rcc, u5::CR) | u5::CR_PLL1ON);
    tick(&rcc, DELAY);
    poke(&rcc, u5::CFGR1, 0b11);
    tick(&rcc, 1);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(160_000_000),
        "FRACEN clear: the fraction is ignored"
    );
    poke(&rcc, u5::PLL1CFGR, 3 | (1 << 4) | (1 << 8) | (1 << 18));
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(164_000_000)
    );
    // PLL1MBOOST divides the booster's clock, not the VCO's input.
    poke(
        &rcc,
        u5::PLL1CFGR,
        3 | (1 << 4) | (1 << 8) | (3 << 12) | (1 << 18),
    );
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(164_000_000)
    );
}

#[test]
fn a_u5_switch_to_a_pll_that_is_not_ready_is_refused() {
    let rcc = u5::rcc();
    poke(&rcc, u5::CFGR1, 0b11);
    tick(&rcc, 2);
    assert_eq!(peek(&rcc, u5::CFGR1) & 0b1111, 0, "SW refused, SWS unmoved");
    poke(&rcc, u5::CR, peek(&rcc, u5::CR) | u5::CR_HSION);
    tick(&rcc, DELAY);
    assert_ne!(peek(&rcc, u5::CR) & u5::CR_HSIRDY, 0);
    poke(&rcc, u5::CFGR1, 0b01);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, u5::CFGR1) & 0b1111, 0b0101);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(16_000_000)
    );
}

#[test]
fn every_u5_oscillator_the_hal_polls_comes_ready() {
    let rcc = u5::rcc();
    rcc.set_dbp(true);
    poke(
        &rcc,
        u5::CR,
        peek(&rcc, u5::CR) | u5::CR_HSI48ON | u5::CR_PLL2ON | (1 << 14) | (1 << 28),
    );
    poke(&rcc, u5::BDCR, 1 | (1 << 7) | (1 << 26));
    tick(&rcc, DELAY);
    let cr = peek(&rcc, u5::CR);
    for (name, bit) in [
        ("HSI48RDY", u5::CR_HSI48RDY),
        ("PLL2RDY", u5::CR_PLL2RDY),
        ("SHSIRDY", 1 << 15),
        ("PLL3RDY", 1 << 29),
        ("MSISRDY", u5::CR_MSISRDY),
    ] {
        assert_ne!(cr & bit, 0, "{name}");
    }
    let bdcr = peek(&rcc, u5::BDCR);
    assert_ne!(bdcr & (1 << 1), 0, "LSERDY");
    assert_ne!(bdcr & (1 << 11), 0, "LSESYSRDY");
    assert_ne!(bdcr & (1 << 27), 0, "LSIRDY — in BDCR on a U5");

    // And each drops the moment its enable does.
    poke(&rcc, u5::CR, cr & !(u5::CR_MSIKON | u5::CR_PLL2ON));
    assert_eq!(peek(&rcc, u5::CR) & ((1 << 5) | u5::CR_PLL2RDY), 0);
}

#[test]
fn the_u5_lsi_is_behind_the_backup_domain_protection() {
    let rcc = u5::rcc();
    rcc.set_dbp(false);
    poke(&rcc, u5::BDCR, 1 << 26);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, u5::BDCR), 0, "dropped while DBP is clear");
    rcc.set_dbp(true);
    // LSI ÷ 128 to the RTC.
    poke(&rcc, u5::BDCR, (1 << 26) | (1 << 28) | (2 << 8) | (1 << 15));
    tick(&rcc, DELAY);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_000).checked_scale(1, 128).unwrap()
    );
}

#[test]
fn the_u5_clock_interrupt_has_msik_at_eleven() {
    let rcc = u5::rcc();
    let (_wire, source, probe) = probe();
    Device::connect(&rcc, IRQ_PIN, source).unwrap();
    poke(&rcc, u5::CIER, 1 << 11);
    poke(&rcc, u5::CR, peek(&rcc, u5::CR) & !u5::CR_MSIKON);
    poke(&rcc, u5::CR, peek(&rcc, u5::CR) | u5::CR_MSIKON);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, u5::CIFR), 1 << 11, "MSIKRDYF");
    assert!(probe.is_high());
    poke(&rcc, u5::CIFR, 0);
    assert_eq!(peek(&rcc, u5::CIFR), 1 << 11, "CIFR is read-only");
    poke(&rcc, u5::CICR, 1 << 11);
    assert_eq!(peek(&rcc, u5::CIFR), 0);
    assert!(!probe.is_high());
}

#[test]
fn the_u5_gate_banks_are_where_the_header_puts_them() {
    let rcc = u5::rcc();
    // `ahb3en2` is `AHB3ENR.PWREN`: the wire a U5 board runs to `st.pwr`.
    for (pin, reg, bit) in [
        ("ahb3en2", u5::AHB3ENR, 2),
        ("ahb2enb0", u5::AHB2ENR2, 0),
        ("apb3en6", u5::APB3ENR, 6),
        ("apb3rst6", u5::APB3RSTR, 6),
        ("apb1en17", 0x9c, 17),
        ("apb1enb1", 0xa0, 1),
        ("ahb2en0", 0x8c, 0),
        ("apb2en14", 0xa4, 14),
        ("ahb1en1", 0x88, 1),
    ] {
        let (_w, source, probe) = probe();
        Device::connect(&rcc, pin, source).unwrap_or_else(|e| panic!("{pin}: {e}"));
        assert!(!probe.is_high(), "{pin} starts low");
        poke(&rcc, reg, 1 << bit);
        assert!(probe.is_high(), "{pin} follows its bit");
        poke(&rcc, reg, 0);
    }
    // The new prefixes are refused by the families that do not have them.
    let (_w, source, _p) = probe();
    assert!(Device::connect(&l4(), "apb3en0", source).is_err());
    let (_w, source, _p) = probe();
    assert!(Device::connect(&f4(), "ahb2enb0", source).is_err());
    let layout = Variant::U5.layout();
    assert_ne!(
        parse_bank_pin(layout, "ahb2en1"),
        parse_bank_pin(layout, "ahb2enb1")
    );
    let schema = schema();
    for pin in ["ahb2enb31", "apb3en0", "ahb2rstb3", "apb3rst31"] {
        assert!(schema.port_named(pin).is_some(), "{pin} is declared");
    }
}

#[test]
fn the_u5_reset_flags_and_rmvf() {
    let rcc = u5::rcc();
    assert_eq!(
        peek(&rcc, u5::CSR),
        0x0c00_4400,
        "PIN + BOR, MSI ranges 4 MHz"
    );
    assert!(Device::sink(&rcc, "fwrst", &[WireId::new(1)]).is_none());
    let src = WireId::new(1);
    let pin = Device::sink(&rcc, "iwdgrst", &[src]).expect("iwdgrst");
    pin.sink.set_level(src, pin.line, Level::High);
    assert_ne!(peek(&rcc, u5::CSR) & (1 << 29), 0);
    poke(&rcc, u5::CSR, peek(&rcc, u5::CSR) | (1 << 23));
    assert_eq!(peek(&rcc, u5::CSR) & 0xff80_0000, 0);
    assert_eq!(peek(&rcc, u5::CSR) & 0xffff, 0x4400, "the ranges stay");
}

#[test]
fn a_u5_snapshot_round_trips_to_identical_state() {
    let saved = u5::rcc();
    saved.set_dbp(true);
    u5::clock_config(&saved);
    poke(&saved, u5::BDCR, 1 | (1 << 26));
    poke(&saved, u5::CR, peek(&saved, u5::CR) | u5::CR_HSI48ON);
    // Mid-transition: three ready deadlines pending.
    assert_eq!(peek(&saved, u5::CR) & u5::CR_HSI48RDY, 0);

    let mut shape = MachineShape::new();
    shape.add_device("rcc", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("rcc", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = u5::rcc();
    restored.set_dbp(true);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("rcc", CLASS_NAME, STATE_VERSION, &Migrations::new())
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
    assert_eq!(
        restored.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(160_000_000)
    );
    tick(&restored, DELAY);
    assert_ne!(peek(&restored, u5::CR) & u5::CR_HSI48RDY, 0);
    assert_eq!(
        peek(&restored, u5::BDCR) & ((1 << 1) | (1 << 27)),
        (1 << 1) | (1 << 27)
    );
}

#[test]
fn the_u5_variant_is_a_property() {
    let props = Props::new()
        .with("variant", Value::from("u5"))
        .with("pclk3", Value::from("pclk3"));
    let rcc = Rcc::new(&props).unwrap();
    assert_eq!(rcc.variant(), Variant::U5);
    assert_eq!(
        Device::region(&rcc, "").map(|r| r.len()),
        Some(0x118),
        "through PRIVCFGR"
    );
}

// ---------------------------------------------------------------------------
// The WB (RM0434; ST CMSIS `stm32wb55xx.h`, `RCC_TypeDef`; reset values from
// ST's `STM32WB55_CM4.svd`)
// ---------------------------------------------------------------------------

mod wb {
    use super::*;

    pub(super) const CR: u64 = 0x00;
    pub(super) const CFGR: u64 = 0x08;
    pub(super) const PLLCFGR: u64 = 0x0c;
    pub(super) const CIER: u64 = 0x18;
    pub(super) const CIFR: u64 = 0x1c;
    pub(super) const CICR: u64 = 0x20;
    pub(super) const SMPSCR: u64 = 0x24;
    pub(super) const APB3RSTR: u64 = 0x44;
    pub(super) const AHB3ENR: u64 = 0x50;
    pub(super) const BDCR: u64 = 0x90;
    pub(super) const CSR: u64 = 0x94;
    pub(super) const CRRCR: u64 = 0x98;
    pub(super) const HSECR: u64 = 0x9c;
    pub(super) const EXTCFGR: u64 = 0x108;
    pub(super) const C2AHB3ENR: u64 = 0x150;

    pub(super) const CR_MSIRDY: u32 = 1 << 1;
    pub(super) const CR_HSION: u32 = 1 << 8;
    pub(super) const CR_HSIKERON: u32 = 1 << 9;
    pub(super) const CR_HSIRDY: u32 = 1 << 10;
    pub(super) const CR_HSIKERDY: u32 = 1 << 12;
    pub(super) const CR_HSEON: u32 = 1 << 16;
    pub(super) const CR_HSERDY: u32 = 1 << 17;
    pub(super) const CR_HSEPRE: u32 = 1 << 20;
    pub(super) const CR_PLLON: u32 = 1 << 24;
    pub(super) const CR_PLLRDY: u32 = 1 << 25;
    pub(super) const CR_PLLSAI1ON: u32 = 1 << 26;
    pub(super) const CR_PLLSAI1RDY: u32 = 1 << 27;

    pub(super) const HPREF: u32 = 1 << 16;
    pub(super) const PPRE1F: u32 = 1 << 17;
    pub(super) const PPRE2F: u32 = 1 << 18;
    pub(super) const SHDHPREF: u32 = 1 << 16;
    pub(super) const C2HPREF: u32 = 1 << 17;

    pub(super) fn rcc() -> Rcc {
        Rcc::with_config(
            Variant::WB,
            Frequencies {
                // The P-NUCLEO-WB55's 32 MHz radio crystal.
                hse: 32_000_000,
                ..Frequencies::default()
            },
            DELAY,
        )
    }

    /// `PLLCFGR` from the HAL's *real* factors: M, Q and R stored as
    /// value − 1, N as itself, with `PLLQEN` and `PLLREN` set.
    pub(super) fn pllcfgr(src: u32, m: u32, n: u32, q: u32, r: u32) -> u32 {
        src | ((m - 1) << 4) | (n << 8) | (1 << 24) | ((q - 1) << 25) | (1 << 28) | ((r - 1) << 29)
    }

    /// Spin on `reg & mask == want` as a HAL wait loop does, one tick a poll.
    pub(super) fn wait(rcc: &Rcc, reg: u64, mask: u32, want: u32, what: &str) {
        let mut spins = 0;
        while peek(rcc, reg) & mask != want {
            tick(rcc, 1);
            spins += 1;
            assert!(spins < 1000, "{what} never came");
        }
    }

    /// `HAL_RCC_ClockConfig`'s order: each prescaler written and its flag
    /// waited on, then `SW` and a wait on `SWS`.
    pub(super) fn clock_config(rcc: &Rcc, hpre: u32, c2hpre: u32, shdhpre: u32, sw: u32) {
        poke(rcc, CFGR, (peek(rcc, CFGR) & !(0xf << 4)) | (hpre << 4));
        wait(rcc, CFGR, HPREF, HPREF, "HPREF");
        poke(
            rcc,
            EXTCFGR,
            (peek(rcc, EXTCFGR) & !(0xf << 4)) | (c2hpre << 4),
        );
        wait(rcc, EXTCFGR, C2HPREF, C2HPREF, "C2HPREF");
        poke(rcc, EXTCFGR, (peek(rcc, EXTCFGR) & !0xf) | shdhpre);
        wait(rcc, EXTCFGR, SHDHPREF, SHDHPREF, "SHDHPREF");
        poke(rcc, CFGR, peek(rcc, CFGR) & !(0x3f << 8));
        wait(rcc, CFGR, PPRE1F | PPRE2F, PPRE1F | PPRE2F, "PPRExF");
        poke(rcc, CFGR, (peek(rcc, CFGR) & !0b11) | sw);
        wait(rcc, CFGR, 0b1100, sw << 2, "SWS");
    }

    /// Oscillators on, waited for as `HAL_RCC_OscConfig` waits.
    pub(super) fn start(rcc: &Rcc, on: u32, rdy: u32) {
        poke(rcc, CR, peek(rcc, CR) | on);
        wait(rcc, CR, rdy, rdy, "a ready bit");
    }
}

#[test]
fn a_wb_boots_on_the_4mhz_msi() {
    let rcc = wb::rcc();
    for (name, reg, value) in [
        ("CR", wb::CR, 0x0000_0063),
        ("ICSCR", 0x04, 0x4000_0000),
        ("CFGR", wb::CFGR, 0x0007_0000),
        ("PLLCFGR", wb::PLLCFGR, 0x2204_0100),
        ("PLLSAI1CFGR", 0x10, 0x2204_0100),
        ("SMPSCR", wb::SMPSCR, 0x0000_0301),
        ("AHB3ENR", wb::AHB3ENR, 0x0208_0000),
        ("APB1ENR1", 0x58, 0x0000_0400),
        ("AHB1SMENR", 0x68, 0x0001_1207),
        ("APB1SMENR1", 0x78, 0x85a0_4e01),
        ("CSR", wb::CSR, 0x0c00_0000),
        ("HSECR", wb::HSECR, 0x0000_0030),
        ("EXTCFGR", wb::EXTCFGR, 0x0003_0000),
        ("C2AHB3ENR", wb::C2AHB3ENR, 0x0208_0000),
        ("C2APB1ENR1", 0x158, 0x0000_0400),
        ("C2AHB3SMENR", 0x170, 0x0307_0000),
        ("C2APB3SMENR", 0x184, 0x0000_0003),
        // RM0434 over the SVD, both times.
        ("AHB1ENR", 0x48, 0),
        ("C2AHB1SMENR", 0x168, 0x0001_1007),
        ("AHB2SMENR", 0x6c, 0x0001_209f),
        ("AHB3SMENR", 0x70, 0x0307_0100),
        ("APB1SMENR2", 0x7c, 0x0000_0021),
        ("APB2SMENR", 0x80, 0x0026_5800),
        ("C2AHB2SMENR", 0x16c, 0x0001_209f),
        ("C2APB1SMENR1", 0x178, 0x85a0_4601),
        ("C2APB1SMENR2", 0x17c, 0x0000_0021),
        ("C2APB2SMENR", 0x180, 0x0026_5800),
        ("C2APB3ENR", 0x164, 0),
    ] {
        assert_eq!(peek(&rcc, reg), value, "{name}");
    }
    let clocks = rcc.clocks();
    for out in [
        ClockOutput::SYSCLK,
        ClockOutput::HCLK,
        ClockOutput::HCLK2,
        ClockOutput::HCLK4,
        ClockOutput::PCLK1,
        ClockOutput::PCLK2,
        ClockOutput::TIMCLK1,
        ClockOutput::TIMCLK2,
    ] {
        assert_eq!(clocks.rate(out), Rational::integer(4_000_000), "{out}");
    }
    assert!(clocks.rate(ClockOutput::PCLK3).is_zero(), "no APB3 clock");
    assert!(clocks.rate(ClockOutput::PLL48).is_zero());
    // The two new outputs exist only on a WB.
    assert!(l4().clocks().rate(ClockOutput::HCLK2).is_zero());
    assert!(f4().clocks().rate(ClockOutput::HCLK4).is_zero());
}

#[test]
fn the_wb_msi_range_is_always_cr_s() {
    let rcc = wb::rcc();
    // No MSIRGSEL to set: range 11 in `CR` is 48 MHz at once.
    poke(&rcc, wb::CR, (peek(&rcc, wb::CR) & !0xf0) | (11 << 4));
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(48_000_000)
    );
    // 12…15 are "not allowed (hardware write protection)" (RM0434 §6.4.1):
    // the write's other bits land and the range does not move.
    poke(
        &rcc,
        wb::CR,
        (peek(&rcc, wb::CR) & !0xf0) | (12 << 4) | wb::CR_HSION,
    );
    assert_eq!((peek(&rcc, wb::CR) >> 4) & 0xf, 11);
    assert_ne!(peek(&rcc, wb::CR) & wb::CR_HSION, 0);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(48_000_000)
    );
}

#[test]
fn every_wb_prescaler_flag_drops_on_a_change_and_comes_back() {
    let rcc = wb::rcc();
    let flags = |rcc: &Rcc| peek(rcc, wb::CFGR) & (0b111 << 16);
    // HPRE ÷2: `HPREF` and only it goes clear, then comes back.
    poke(&rcc, wb::CFGR, 0x8 << 4);
    assert_eq!(flags(&rcc), wb::PPRE1F | wb::PPRE2F);
    tick(&rcc, 1);
    assert_eq!(flags(&rcc), 0b111 << 16);
    // PPRE1 and PPRE2 at once.
    poke(&rcc, wb::CFGR, (0x8 << 4) | (4 << 8) | (5 << 11));
    assert_eq!(flags(&rcc), wb::HPREF);
    tick(&rcc, 1);
    assert_eq!(flags(&rcc), 0b111 << 16);
    // Writing the fields as they stand is not a change.
    poke(&rcc, wb::CFGR, (0x8 << 4) | (4 << 8) | (5 << 11));
    assert_eq!(flags(&rcc), 0b111 << 16);
    // The flags are the hardware's: `CLEAR_REG(RCC->CFGR)` cannot clear
    // them, it can only move every prescaler back and so clear all three for
    // a tick.
    poke(&rcc, wb::CFGR, 0);
    assert_eq!(flags(&rcc), 0);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, wb::CFGR), 0b111 << 16);

    // `EXTCFGR`: C2HPRE clears `C2HPREF`, SHDHPRE clears `SHDHPREF`.
    poke(&rcc, wb::EXTCFGR, 0x8 << 4);
    assert_eq!(peek(&rcc, wb::EXTCFGR), wb::SHDHPREF | (0x8 << 4));
    assert_eq!(
        Device::next_event_tick(&rcc),
        Some(Device::current_tick(&rcc) + 1),
        "the flag's return is a scheduled event"
    );
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, wb::EXTCFGR), 0x0003_0000 | (0x8 << 4));
    poke(&rcc, wb::EXTCFGR, (0x8 << 4) | 0x9);
    assert_eq!(peek(&rcc, wb::EXTCFGR) & (0b11 << 16), wb::C2HPREF);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, wb::EXTCFGR) & (0b11 << 16), 0b11 << 16);
    // `RFCSS` is the hardware's too.
    poke(&rcc, wb::EXTCFGR, peek(&rcc, wb::EXTCFGR) | (1 << 20));
    assert_eq!(peek(&rcc, wb::EXTCFGR) & (1 << 20), 0);
}

#[test]
fn the_wb_ahb_prescalers_have_the_wb_table() {
    let rcc = wb::rcc();
    // MSI range 11, 48 MHz, so every divisor in the table divides evenly.
    poke(&rcc, wb::CR, (peek(&rcc, wb::CR) & !0xf0) | (11 << 4));
    for (field, div) in [
        (0b0000, 1),
        (0b0001, 3),
        (0b0010, 5),
        (0b0011, 1),
        (0b0100, 1),
        (0b0101, 6),
        (0b0110, 10),
        (0b0111, 32),
        (0b1000, 2),
        (0b1001, 4),
        (0b1011, 16),
        (0b1111, 512),
    ] {
        let want = Rational::integer(48_000_000).checked_scale(1, div).unwrap();
        poke(&rcc, wb::CFGR, field << 4);
        poke(&rcc, wb::EXTCFGR, (field << 4) | field);
        let clocks = rcc.clocks();
        assert_eq!(clocks.rate(ClockOutput::HCLK), want, "HPRE {field:04b}");
        assert_eq!(clocks.rate(ClockOutput::HCLK2), want, "C2HPRE {field:04b}");
        assert_eq!(clocks.rate(ClockOutput::HCLK4), want, "SHDHPRE {field:04b}");
        assert_eq!(
            clocks.rate(ClockOutput::SYSCLK),
            Rational::integer(48_000_000)
        );
    }
    // The three are independent: CPU2 and the flash do not follow HPRE.
    poke(&rcc, wb::CFGR, 0x8 << 4);
    poke(&rcc, wb::EXTCFGR, (0x9 << 4) | 0x1);
    let clocks = rcc.clocks();
    assert_eq!(
        clocks.rate(ClockOutput::HCLK),
        Rational::integer(24_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::HCLK2),
        Rational::integer(12_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::HCLK4),
        Rational::integer(16_000_000)
    );
    // And the APB clocks hang off HCLK1.
    assert_eq!(
        clocks.rate(ClockOutput::PCLK1),
        Rational::integer(24_000_000)
    );
}

#[test]
fn the_wb_vendor_clock_config_reaches_64mhz_from_the_hse() {
    // CubeMX's P-NUCLEO-WB55 default: HSE 32 MHz into the PLL with M = 2,
    // N = 8, R = 2 (VCO 128 MHz), Q = 2, CPU2 at ÷2.
    let rcc = wb::rcc();
    wb::start(&rcc, wb::CR_HSEON, wb::CR_HSERDY);
    poke(&rcc, wb::PLLCFGR, wb::pllcfgr(3, 2, 8, 2, 2));
    wb::start(&rcc, wb::CR_PLLON, wb::CR_PLLRDY);
    wb::clock_config(&rcc, 0, 0x8, 0, 0b11);
    let clocks = rcc.clocks();
    for (out, hz) in [
        (ClockOutput::SYSCLK, 64_000_000),
        (ClockOutput::HCLK, 64_000_000),
        (ClockOutput::HCLK2, 32_000_000),
        (ClockOutput::HCLK4, 64_000_000),
        (ClockOutput::PCLK1, 64_000_000),
        (ClockOutput::PCLK2, 64_000_000),
        (ClockOutput::TIMCLK1, 64_000_000),
        (ClockOutput::TIMCLK2, 64_000_000),
        (ClockOutput::PLL48, 64_000_000),
    ] {
        assert_eq!(clocks.rate(out), Rational::integer(hz), "{out}");
    }
    // APB1 at ÷4 doubles the timers.
    poke(&rcc, wb::CFGR, peek(&rcc, wb::CFGR) | (5 << 8));
    assert_eq!(
        clocks.rate(ClockOutput::PCLK1),
        Rational::integer(16_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::TIMCLK1),
        Rational::integer(32_000_000)
    );
}

#[test]
fn the_wb_pll_reaches_64mhz_from_the_msi() {
    let rcc = wb::rcc();
    // MSI 4 MHz, M = 1, N = 32, R = 2.
    poke(&rcc, wb::PLLCFGR, wb::pllcfgr(1, 1, 32, 4, 2));
    wb::start(&rcc, wb::CR_PLLON, wb::CR_PLLRDY);
    wb::clock_config(&rcc, 0, 0x8, 0, 0b11);
    let clocks = rcc.clocks();
    assert_eq!(
        clocks.rate(ClockOutput::SYSCLK),
        Rational::integer(64_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::HCLK2),
        Rational::integer(32_000_000)
    );
    assert_eq!(
        clocks.rate(ClockOutput::PLL48),
        Rational::integer(32_000_000),
        "Q = 4 off a 128 MHz VCO"
    );
    // Without `PLLREN` there is no system output.
    poke(&rcc, wb::PLLCFGR, wb::pllcfgr(1, 1, 32, 4, 2) & !(1 << 28));
    assert!(clocks.rate(ClockOutput::SYSCLK).is_zero());
}

#[test]
fn hsepre_halves_the_crystal_into_sysclk_and_the_pll() {
    let rcc = wb::rcc();
    rcc.set_dbp(true);
    wb::start(&rcc, wb::CR_HSEON | wb::CR_HSEPRE, wb::CR_HSERDY);
    poke(&rcc, wb::CFGR, 0b10);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, wb::CFGR) & 0b1100, 0b1000);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(16_000_000)
    );
    // The PLL sees 16 MHz too: M = 1, N = 8, R = 2 is 64 MHz, not 128.
    poke(&rcc, wb::PLLCFGR, wb::pllcfgr(3, 1, 8, 2, 2));
    wb::start(&rcc, wb::CR_PLLON, wb::CR_PLLRDY);
    poke(&rcc, wb::CFGR, 0b11);
    tick(&rcc, 1);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(64_000_000)
    );
    // The RTC's HSE ÷ 32 takes the undivided crystal.
    poke(&rcc, wb::BDCR, (3 << 8) | (1 << 15));
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(1_000_000)
    );
}

#[test]
fn a_wb_switch_to_a_source_that_is_not_ready_is_refused() {
    let rcc = wb::rcc();
    poke(&rcc, wb::CFGR, 0b11);
    tick(&rcc, 2);
    assert_eq!(peek(&rcc, wb::CFGR) & 0b1111, 0, "SW refused, SWS unmoved");
    poke(&rcc, wb::CFGR, 0b10);
    tick(&rcc, 2);
    assert_eq!(peek(&rcc, wb::CFGR) & 0b1111, 0, "no HSE either");
    wb::start(&rcc, wb::CR_HSION, wb::CR_HSIRDY);
    poke(&rcc, wb::CFGR, 0b01);
    tick(&rcc, 1);
    assert_eq!(peek(&rcc, wb::CFGR) & 0b1111, 0b0101);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::SYSCLK),
        Rational::integer(16_000_000)
    );
}

#[test]
fn every_wb_oscillator_the_hal_polls_comes_ready() {
    let rcc = wb::rcc();
    rcc.set_dbp(true);
    poke(
        &rcc,
        wb::CR,
        peek(&rcc, wb::CR) | wb::CR_HSION | wb::CR_HSIKERON | wb::CR_PLLSAI1ON,
    );
    poke(&rcc, wb::BDCR, 1);
    poke(&rcc, wb::CSR, (1 << 0) | (1 << 2));
    poke(&rcc, wb::CRRCR, 1);
    tick(&rcc, DELAY);
    let cr = peek(&rcc, wb::CR);
    for (name, bit) in [
        ("MSIRDY", wb::CR_MSIRDY),
        ("HSIRDY", wb::CR_HSIRDY),
        ("HSIKERDY", wb::CR_HSIKERDY),
        ("PLLSAI1RDY", wb::CR_PLLSAI1RDY),
    ] {
        assert_ne!(cr & bit, 0, "{name}");
    }
    assert_ne!(peek(&rcc, wb::BDCR) & (1 << 1), 0, "LSERDY");
    let csr = peek(&rcc, wb::CSR);
    assert_ne!(csr & (1 << 1), 0, "LSI1RDY");
    assert_ne!(csr & (1 << 3), 0, "LSI2RDY");
    assert_ne!(peek(&rcc, wb::CRRCR) & (1 << 1), 0, "HSI48RDY");

    // `LSI2ON` is "enable and selection" (RM0434 §6.4.31): with both on,
    // the LSI is LSI2.
    poke(&rcc, wb::BDCR, 1 | (2 << 8) | (1 << 15));
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_000)
    );
    poke(&rcc, wb::CSR, 1 << 2);
    assert_eq!(peek(&rcc, wb::CSR) & 0b1111, 0b1100, "LSI1 off, LSI2 on");
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_000)
    );
    // LSI1 running and LSI2 selected but not yet ready: no LSI at all.
    poke(&rcc, wb::CSR, 1);
    tick(&rcc, DELAY);
    poke(&rcc, wb::CSR, 1 | (1 << 2));
    assert_eq!(peek(&rcc, wb::CSR) & 0b1111, 0b0111);
    assert!(rcc.clocks().rate(ClockOutput::RTCCLK).is_zero());
    tick(&rcc, DELAY);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_000)
    );
    // And with LSI2 deselected, LSI1 is the LSI again.
    poke(&rcc, wb::CSR, 1);
    assert_eq!(
        rcc.clocks().rate(ClockOutput::RTCCLK),
        Rational::integer(32_000)
    );

    // And each drops the moment its enable does.
    poke(&rcc, wb::CSR, 0);
    assert_eq!(peek(&rcc, wb::CSR) & 0b1111, 0);
    poke(&rcc, wb::CRRCR, 0);
    assert_eq!(peek(&rcc, wb::CRRCR), 0);
    poke(&rcc, wb::CR, peek(&rcc, wb::CR) & !wb::CR_HSIKERON);
    assert_eq!(peek(&rcc, wb::CR) & wb::CR_HSIKERDY, 0);
    assert!(rcc.clocks().rate(ClockOutput::RTCCLK).is_zero());
}

#[test]
fn the_wb_clock_interrupt_has_hsi48_at_ten_and_lsi2_at_eleven() {
    let rcc = wb::rcc();
    let (_wire, source, probe) = probe();
    Device::connect(&rcc, IRQ_PIN, source).unwrap();
    poke(&rcc, wb::CIER, (1 << 10) | (1 << 11));
    poke(&rcc, wb::CSR, 1 << 2);
    poke(&rcc, wb::CRRCR, 1);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, wb::CIFR), (1 << 10) | (1 << 11));
    assert!(probe.is_high());
    poke(&rcc, wb::CIFR, 0);
    assert_eq!(peek(&rcc, wb::CIFR), (1 << 10) | (1 << 11), "read-only");
    poke(&rcc, wb::CICR, 1 << 11);
    assert_eq!(peek(&rcc, wb::CIFR), 1 << 10);
    assert!(probe.is_high());
    poke(&rcc, wb::CICR, 1 << 10);
    assert_eq!(peek(&rcc, wb::CIFR), 0);
    assert!(!probe.is_high());

    // LSI1 and the PLL keep the L4's positions, 0 and 5.
    poke(&rcc, wb::CIER, 1 | (1 << 5));
    poke(&rcc, wb::CSR, 1);
    poke(&rcc, wb::PLLCFGR, wb::pllcfgr(1, 1, 32, 2, 2));
    poke(&rcc, wb::CR, peek(&rcc, wb::CR) | wb::CR_PLLON);
    tick(&rcc, DELAY);
    assert_eq!(peek(&rcc, wb::CIFR), 1 | (1 << 5));
}

#[test]
fn the_wb_gate_banks_are_where_the_header_puts_them() {
    let rcc = wb::rcc();
    for (bank, reg, bit) in [
        ("ahb1en", 0x48, 0),
        ("ahb2en", 0x4c, 0),
        ("ahb3en", wb::AHB3ENR, 20),
        ("apb1en", 0x58, 21),
        ("apb1enb", 0x5c, 0),
        ("apb2en", 0x60, 14),
        ("ahb1rst", 0x28, 0),
        ("ahb2rst", 0x2c, 0),
        ("ahb3rst", 0x30, 20),
        ("apb1rst", 0x38, 21),
        ("apb1rstb", 0x3c, 0),
        ("apb2rst", 0x40, 14),
        ("apb3rst", wb::APB3RSTR, 0),
        ("c2ahb1en", 0x148, 9),
        ("c2ahb2en", 0x14c, 0),
        ("c2ahb3en", wb::C2AHB3ENR, 20),
        ("c2apb1en", 0x158, 21),
        ("c2apb1enb", 0x15c, 0),
        ("c2apb2en", 0x160, 14),
        ("c2apb3en", 0x164, 0),
    ] {
        let pin = format!("{bank}{bit}");
        let (_w, source, probe) = probe();
        Device::connect(&rcc, &pin, source).unwrap_or_else(|e| panic!("{pin}: {e}"));
        let reset = peek(&rcc, reg);
        poke(&rcc, reg, 0);
        assert!(!probe.is_high(), "{pin} low with its bit clear");
        assert_eq!(rcc.gate(bank, bit), Some(false));
        poke(&rcc, reg, 1 << bit);
        assert!(probe.is_high(), "{pin} follows its bit");
        assert_eq!(rcc.gate(bank, bit), Some(true));
        poke(&rcc, reg, reset);
    }

    // HSEM and FLASH are clocked out of reset, for both CPUs, and so is the
    // RTC's APB interface; IPCC is not.
    for pin in [
        "ahb3en19",
        "ahb3en25",
        "c2ahb3en19",
        "c2ahb3en25",
        "apb1en10",
    ] {
        let (_w, source, probe) = probe();
        Device::connect(&rcc, pin, source).unwrap();
        assert!(probe.is_high(), "{pin} is on out of reset");
    }
    let (_w, source, ipcc) = probe();
    Device::connect(&rcc, "ahb3en20", source).unwrap();
    assert!(!ipcc.is_high(), "IPCC is not");

    // CPU2's banks are a WB's, and nobody else has them.
    for other in [f4(), l4(), u5::rcc()] {
        let (_w, source, _p) = probe();
        assert!(Device::connect(&other, "c2ahb3en19", source).is_err());
    }
    // A WB has no APB3 enable word and no second AHB2 word.
    for pin in ["apb3en0", "ahb2enb0"] {
        let (_w, source, _p) = probe();
        assert!(Device::connect(&rcc, pin, source).is_err(), "{pin}");
    }
    let layout = Variant::WB.layout();
    assert_ne!(
        parse_bank_pin(layout, "c2ahb1en3"),
        parse_bank_pin(layout, "ahb1en3")
    );
    assert_ne!(
        parse_bank_pin(layout, "c2apb1en21"),
        parse_bank_pin(layout, "c2apb1enb1")
    );
    let schema = schema();
    for pin in ["c2ahb1en0", "c2ahb3en31", "c2apb1enb0", "c2apb3en1"] {
        assert!(schema.port_named(pin).is_some(), "{pin} is declared");
    }
}

#[test]
fn the_wb_smps_clock_follows_smpsen_the_hse_and_smpssel() {
    // RM0434 §6.4.9, rule by rule.
    let rcc = wb::rcc();
    // "Whenever the SMPS step-down converter is disabled in PWR_CR5.SMPSEN,
    // no clock is used" — and nothing drives `smpsen` yet.
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x301);
    poke(&rcc, wb::SMPSCR, 0x10);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x310, "SMPSDIV kept, SWS = none");

    let src = WireId::new(1);
    let pin = Device::sink(&rcc, SMPSEN_PIN, &[src]).expect("a WB has `smpsen`");
    pin.sink.set_level(src, pin.line, Level::High);
    // HSI selected while the HSI is off: no clock.
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x310);
    wb::start(&rcc, wb::CR_HSION, wb::CR_HSIRDY);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x010, "HSI16");
    // MSI is running from reset.
    poke(&rcc, wb::SMPSCR, 0x01);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x101, "MSI");
    // The status is the hardware's.
    poke(&rcc, wb::SMPSCR, 0x201);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x101);
    // The reserved selection is no clock.
    poke(&rcc, wb::SMPSCR, 0x03);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x303);
    // "Whenever the HSE is active the HSE is used regardless of the settings
    // in SMPSSEL."
    wb::start(&rcc, wb::CR_HSEON, wb::CR_HSERDY);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x203);
    poke(&rcc, wb::SMPSCR, 0x01);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x201);
    // The SMPS switched off again: no clock, whatever is running.
    pin.sink.set_level(src, pin.line, Level::Low);
    assert_eq!(peek(&rcc, wb::SMPSCR), 0x301);

    // `smpsen` is a WB's pin.
    assert!(Device::sink(&l4(), SMPSEN_PIN, &[src]).is_none());
    assert_eq!(
        schema().port_named(SMPSEN_PIN).map(|p| p.dir),
        Some(PortDir::In)
    );
}

#[test]
fn the_wb_hsecr_takes_the_key_for_one_write() {
    let rcc = wb::rcc();
    poke(&rcc, wb::HSECR, 0x2a << 8);
    assert_eq!(peek(&rcc, wb::HSECR), 0x30, "locked: dropped");
    poke(&rcc, wb::HSECR, 0xcafe_cafe);
    assert_eq!(peek(&rcc, wb::HSECR), 0x31, "UNLOCKED");
    // `LL_RCC_HSE_SetCapacitorTuning`'s read-modify-write.
    poke(
        &rcc,
        wb::HSECR,
        (peek(&rcc, wb::HSECR) & !0x3f00) | (0x2a << 8),
    );
    assert_eq!(peek(&rcc, wb::HSECR), (0x2a << 8) | 0x30, "landed, locked");
    poke(&rcc, wb::HSECR, 0);
    assert_eq!(peek(&rcc, wb::HSECR), (0x2a << 8) | 0x30, "locked again");
}

#[test]
fn the_wb_csr_holds_the_radio_reset_and_wakeup_clock() {
    let rcc = wb::rcc();
    // `RFWKPSEL` = HSE ÷ 1024 and `LSI2TRIM` are storage.
    poke(&rcc, wb::CSR, (0b11 << 14) | (0x5 << 8));
    assert_eq!(peek(&rcc, wb::CSR), 0x0c00_0000 | (0b11 << 14) | (0x5 << 8));
    // `RFRSTS` cannot be written; it is `APB3RSTR.RFRST`.
    poke(&rcc, wb::CSR, peek(&rcc, wb::CSR) | (1 << 16));
    assert_eq!(peek(&rcc, wb::CSR) & (1 << 16), 0);
    poke(&rcc, wb::APB3RSTR, 1);
    assert_ne!(peek(&rcc, wb::CSR) & (1 << 16), 0, "the radio is held");
    poke(&rcc, wb::APB3RSTR, 0);
    assert_eq!(peek(&rcc, wb::CSR) & (1 << 16), 0, "and released");
}

#[test]
fn the_wb_reset_flags_and_rmvf() {
    let rcc = wb::rcc();
    assert!(Device::sink(&rcc, "fwrst", &[WireId::new(1)]).is_none());
    assert!(Device::sink(&rcc, "porrst", &[WireId::new(1)]).is_none());
    let src = WireId::new(1);
    for (pin, bit) in [
        ("oblrst", 25),
        ("sftrst", 28),
        ("iwdgrst", 29),
        ("wwdgrst", 30),
        ("lpwrrst", 31),
    ] {
        let sink = Device::sink(&rcc, pin, &[src]).expect("a reset cause");
        sink.sink.set_level(src, sink.line, Level::High);
        assert_ne!(peek(&rcc, wb::CSR) & (1 << bit), 0, "{pin}");
    }
    poke(
        &rcc,
        wb::CSR,
        peek(&rcc, wb::CSR) | (1 << 23) | (0b01 << 14),
    );
    assert_eq!(
        peek(&rcc, wb::CSR),
        0b01 << 14,
        "every flag gone, RFWKPSEL kept"
    );
}

#[test]
fn a_wb_snapshot_round_trips_to_identical_state() {
    let saved = wb::rcc();
    saved.set_dbp(true);
    wb::start(&saved, wb::CR_HSEON, wb::CR_HSERDY);
    poke(&saved, wb::PLLCFGR, wb::pllcfgr(3, 2, 8, 2, 2));
    wb::start(&saved, wb::CR_PLLON, wb::CR_PLLRDY);
    wb::clock_config(&saved, 0, 0x8, 0, 0b11);
    poke(&saved, 0x15c, 1);
    poke(&saved, wb::HSECR, 0xcafe_cafe);
    // Mid-transition: LSI2 and HSI48 are starting and C2HPREF is clear.
    poke(&saved, wb::CSR, 1 << 2);
    poke(&saved, wb::CRRCR, 1);
    poke(&saved, wb::EXTCFGR, 0x9 << 4);
    assert_eq!(peek(&saved, wb::EXTCFGR) & wb::C2HPREF, 0);

    let mut shape = MachineShape::new();
    shape.add_device("rcc", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("rcc", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = wb::rcc();
    restored.set_dbp(true);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("rcc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    let words = Variant::WB.register_bytes() / 4;
    let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(before, after);
    let held = *saved.regs.state.lock();
    assert_eq!(held, *restored.regs.state.lock(), "deadlines and all");
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored)
    );
    for out in [ClockOutput::SYSCLK, ClockOutput::HCLK2, ClockOutput::HCLK4] {
        assert_eq!(
            saved.clocks().rate(out),
            restored.clocks().rate(out),
            "{out}"
        );
    }
    assert_eq!(
        restored.clocks().rate(ClockOutput::HCLK2),
        Rational::integer(16_000_000)
    );
    // The pending work completes on the other side, and the key survived.
    tick(&restored, DELAY);
    assert_ne!(peek(&restored, wb::EXTCFGR) & wb::C2HPREF, 0);
    assert_ne!(peek(&restored, wb::CSR) & (1 << 3), 0);
    assert_ne!(peek(&restored, wb::CRRCR) & (1 << 1), 0);
    poke(&restored, wb::HSECR, 0x2a << 8);
    assert_eq!(peek(&restored, wb::HSECR), 0x2a << 8);
}

#[test]
fn the_wb_drives_cpu2_s_clock_domain_against_sysclk() {
    let rcc = Rcc::with_outputs(
        Variant::WB,
        Frequencies {
            hse: 32_000_000,
            ..Frequencies::default()
        },
        DELAY,
        alloc::vec![
            (ClockOutput::SYSCLK, String::from("sysclk")),
            (ClockOutput::HCLK2, String::from("hclk2")),
            (ClockOutput::HCLK4, String::from("hclk4")),
        ],
    );
    let mut forest = crate::core::clock::ClockForest::new();
    let hse = forest
        .add_oscillator("hse", Rational::integer(32_000_000))
        .unwrap();
    let sys = forest.add_domain("sysclk", hse, 1, 8).unwrap();
    let c2 = forest.add_domain("hclk2", sys, 1, 1).unwrap();
    let shd = forest.add_domain("hclk4", sys, 1, 1).unwrap();
    *rcc.regs.domains.lock() = alloc::vec![
        (ClockOutput::SYSCLK, sys),
        (ClockOutput::HCLK2, c2),
        (ClockOutput::HCLK4, shd),
    ];
    let control = Arc::new(ClockControl::new());
    Device::attach_clock_control(&rcc, Arc::clone(&control));
    let batch = control.take();
    assert_eq!(batch.len(), 3);
    assert_eq!(batch[0].ratio(), Rational::new(1, 8).unwrap(), "4 MHz MSI");

    poke(&rcc, wb::EXTCFGR, (0x8 << 4) | 0x1);
    let batch = control.take();
    let of = |domain| {
        batch
            .iter()
            .find(|r| r.domain == domain)
            .expect("a request")
            .ratio()
    };
    assert_eq!(of(c2), Rational::new(1, 2).unwrap(), "CPU2: SYSCLK ÷ 2");
    assert_eq!(of(shd), Rational::new(1, 3).unwrap(), "the flash: ÷ 3");
}

#[test]
fn the_wb_variant_is_a_property() {
    let props = Props::new()
        .with("variant", Value::from("wb"))
        .with("hse", Value::from(32_000_000u64))
        .with("hclk2", Value::from("cpu2clk"))
        .with("hclk4", Value::from("flashclk"));
    let rcc = Rcc::new(&props).unwrap();
    assert_eq!(rcc.variant(), Variant::WB);
    assert_eq!(
        Device::region(&rcc, "").map(|r| r.len()),
        Some(0x188),
        "through C2APB3SMENR"
    );
    assert_eq!(rcc.regs.named.len(), 2);
}
