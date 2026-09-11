//! `st.rcc` driving real clock domains — the clock-control seam, end to end.
//!
//! GitHub issue #14 asks for one thing this file is the whole of: *"the device
//! owns a clock output the machine can wire as `sysclk`/`hclk`/`pclk1`/`pclk2`,
//! so peripheral clocks follow the tree … prescaler changes take effect on the
//! wire"*, with the named test being "HSE 8 MHz, M=8, N=336, P=2 → **168 MHz on
//! the sysclk wire**".
//!
//! Every other `st.rcc` test asserts what the device *computes*. This one
//! asserts what the **scheduler** ends up believing, which until the seam
//! landed was a different and unchanging number.
//!
//! The machine is written inline rather than added to `machines/`: it is a
//! statement about the seam, not a board anybody runs, and the exact lines a
//! real board would need are in `src/dev/stm32/rcc.rs`'s module documentation.
//!
//! Source: ST **RM0090** rev 21 §7.2 (the clock tree) and §7.3 (`CR`,
//! `PLLCFGR`, `CFGR`). No emulator source of any licence was consulted.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::{GlobalTime, Rational};
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::machine::DeviceEntry;
use rsemu::machine::{Machine, build, catalog};

/// Where the test maps RCC, which is where an F4 has it (RM0090 Table 1).
const RCC: u64 = 0x4002_3800;

const CR: u64 = RCC;
const PLLCFGR: u64 = RCC + 0x04;
const CFGR: u64 = RCC + 0x08;

const CR_HSEON: u64 = 1 << 16;
const CR_HSERDY: u64 = 1 << 17;
const CR_PLLON: u64 = 1 << 24;
const CR_PLLRDY: u64 = 1 << 25;

/// A crystal, the four nodes of RM0090 §7.2's tree, and the controller that
/// drives them.
///
/// `sysclk` starts at `hse * 2` because that is what an F4 comes out of reset
/// on: HSI, 16 MHz. A board with one declared high-speed oscillator writes the
/// internal RC as an exact ratio of the external one — see `drive_domains` for
/// why, and for what that does and does not model.
const SRC: &str = r#"
machine "clock-tree" {
  osc hse = 8000000 Hz

  space mem { width = 32 }

  object sysclk "clock" { clock = hse * 2 }
  object hclk   "clock" { clock = sysclk }
  object pclk1  "clock" { clock = hclk / 4 }
  object pclk2  "clock" { clock = hclk / 2 }

  object rcc "st.rcc" {
    clock   = hse
    variant = "f4"
    hse     = 8000000
    sysclk  = "sysclk"
    hclk    = "hclk"
    pclk1   = "pclk1"
    pclk2   = "pclk2"
  }

  map mem 0x40023800 size 0x90 = rcc
}
"#;

fn machine() -> Machine {
    let registry = catalog::registry().expect("the catalog registers");
    let options = catalog::build_options().expect("the catalog binds");
    match build("clock-tree.machine", SRC, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("{e}"),
    }
}

fn poke(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("mem")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .expect("a word write");
}

fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("mem")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a word read")
}

fn hz(m: &Machine, object: &str) -> Rational {
    let domain = m
        .device(object)
        .and_then(DeviceEntry::domain)
        .expect("a clock node has a domain");
    m.clocks()
        .domain_frequency(domain)
        .expect("a rate for every domain")
}

fn ticks(m: &Machine, object: &str) -> u64 {
    let domain = m
        .device(object)
        .and_then(DeviceEntry::domain)
        .expect("a clock node has a domain");
    m.clocks()
        .ticks(domain)
        .expect("a counter for every domain")
}

/// Ten microseconds — long enough for the default sixteen-tick ready delay of
/// an 8 MHz can, short enough to run in no time at all.
fn a_moment(m: &mut Machine) {
    m.run_for(GlobalTime::from_nanos(10_000)).expect("runs");
}

#[test]
fn the_pll_output_frequency_is_what_the_registers_say() {
    let mut m = machine();

    // Out of reset the part is on HSI at 16 MHz with every prescaler at one,
    // so the whole tree is 16 MHz — **whatever the machine file declared**.
    // That is the seam doing its job: the rates now come from `CFGR`, and the
    // `/4` and `/2` written on the nodes below are only the shape of the tree.
    assert_eq!(hz(&m, "sysclk"), Rational::integer(16_000_000));
    assert_eq!(hz(&m, "hclk"), Rational::integer(16_000_000));
    assert_eq!(hz(&m, "pclk1"), Rational::integer(16_000_000));
    assert_eq!(hz(&m, "pclk2"), Rational::integer(16_000_000));

    // `SystemInit`, by hand: start the crystal and wait for it.
    poke(&m, CR, peek(&m, CR) | CR_HSEON);
    a_moment(&mut m);
    assert_ne!(peek(&m, CR) & CR_HSERDY, 0, "HSERDY never came back");

    // M=8, N=336, P=2, Q=7, source HSE — RM0090 §7.3.2's own worked example,
    // and what every F4 Discovery board does.
    poke(
        &m,
        PLLCFGR,
        // PLLP is `(P/2) - 1`, so P = 2 is the zero this spells out.
        8 | (336 << 6) | (1 << 22) | (7 << 24),
    );
    poke(&m, CR, peek(&m, CR) | CR_PLLON);
    a_moment(&mut m);
    assert_ne!(peek(&m, CR) & CR_PLLRDY, 0, "PLLRDY never came back");

    // HPRE 1, PPRE1 4, PPRE2 2, SW = PLL. The prescalers go in with the
    // switch, which is what a vendor `SystemClock_Config` does.
    poke(&m, CFGR, (5 << 10) | (4 << 13) | 2);
    a_moment(&mut m);
    assert_eq!(peek(&m, CFGR) & 0xc, 2 << 2, "SWS never followed SW");
    // `SWS` moved during the catch-up that `peek` triggered, so the request it
    // produced is waiting for a boundary. One more round is where it lands —
    // which on a real board is the next thing the guest's poll loop does
    // anyway. See `Scheduler::apply_clock_requests`.
    a_moment(&mut m);

    // The point of the whole exercise: 168 MHz on the sysclk wire, and the
    // buses at the prescaled rates, as exact rationals rather than as a number
    // the device merely reports.
    assert_eq!(hz(&m, "sysclk"), Rational::integer(168_000_000));
    assert_eq!(hz(&m, "hclk"), Rational::integer(168_000_000));
    assert_eq!(hz(&m, "pclk1"), Rational::integer(42_000_000));
    assert_eq!(hz(&m, "pclk2"), Rational::integer(84_000_000));
}

#[test]
fn a_prescaler_change_takes_effect_on_the_wire() {
    // The other half of the issue's sentence. `PPRE1` alone, with nothing else
    // touched: the APB1 clock has to move and `SYSCLK` has to stay put.
    let mut m = machine();
    a_moment(&mut m);
    assert_eq!(hz(&m, "pclk1"), Rational::integer(16_000_000));

    // PPRE1 = /16 (RM0090 §7.3.3: 0b111).
    poke(&m, CFGR, 7 << 10);
    a_moment(&mut m);
    assert_eq!(hz(&m, "sysclk"), Rational::integer(16_000_000));
    assert_eq!(hz(&m, "pclk1"), Rational::integer(1_000_000));
    assert_eq!(hz(&m, "pclk2"), Rational::integer(16_000_000));

    // And the ratio inside the tree is exact from there on: sixteen HCLK ticks
    // per PCLK1 tick, counted rather than converted through absolute time.
    let (h0, p0) = (ticks(&m, "hclk"), ticks(&m, "pclk1"));
    m.run_for(GlobalTime::from_nanos(1_000_000)).expect("runs");
    let (h1, p1) = (ticks(&m, "hclk"), ticks(&m, "pclk1"));
    // A tree stops on a tick boundary, so a millisecond of it is a millisecond
    // rounded down — never up, and never more than one tick short.
    assert!((15_999..=16_000).contains(&(h1 - h0)), "{}", h1 - h0);
    assert!((999..=1_000).contains(&(p1 - p0)), "{}", p1 - p0);
    // The ratio itself is exact, and that is the claim that matters: it is
    // integer arithmetic over the divisors and never goes near absolute time.
    let hclk = m.device("hclk").and_then(DeviceEntry::domain).unwrap();
    let pclk1 = m.device("pclk1").and_then(DeviceEntry::domain).unwrap();
    assert_eq!(
        m.clocks().convert_ticks(hclk, pclk1, 16_000).unwrap(),
        1_000
    );
}

#[test]
fn a_counter_is_continuous_across_the_change() {
    // The rule `ClockControl` promises, at machine level: reprogramming the
    // tree neither loses a tick nor invents one.
    let mut m = machine();
    m.run_for(GlobalTime::from_nanos(1_000_000)).expect("runs");
    let before = (ticks(&m, "sysclk"), ticks(&m, "pclk1"));
    assert_eq!(before, (16_000, 16_000));

    // Switch to the crystal itself — 8 MHz, half what it was running at.
    poke(&m, CR, peek(&m, CR) | CR_HSEON);
    a_moment(&mut m);
    poke(&m, CFGR, 1);
    a_moment(&mut m);
    assert_eq!(hz(&m, "sysclk"), Rational::integer(8_000_000));

    let after = (ticks(&m, "sysclk"), ticks(&m, "pclk1"));
    assert!(
        after.0 >= before.0 && after.1 >= before.1,
        "a counter may not go backwards across a re-rating: {before:?} -> {after:?}"
    );
    // Twenty microseconds of it at 16 MHz, then the rest at 8: whatever the
    // split, the counter is monotone and the *next* millisecond is counted at
    // the new rate exactly.
    let mark = (ticks(&m, "sysclk"), ticks(&m, "pclk1"));
    m.run_for(GlobalTime::from_nanos(1_000_000)).expect("runs");
    assert_eq!(ticks(&m, "sysclk") - mark.0, 8_000);
    assert_eq!(ticks(&m, "pclk1") - mark.1, 8_000);
}

#[test]
fn a_board_that_names_no_outputs_drives_nothing() {
    // Every board in `machines/` is this one until it is rewritten, and none of
    // them may change behaviour because the seam exists.
    const QUIET: &str = r#"
machine "quiet" {
  osc hse = 8000000 Hz
  space mem { width = 32 }
  object sysclk "clock" { clock = hse * 21 }
  object rcc "st.rcc" { clock = hse, variant = "f4", hse = 8000000 }
  map mem 0x40023800 size 0x90 = rcc
}
"#;
    let registry = catalog::registry().expect("the catalog registers");
    let options = catalog::build_options().expect("the catalog binds");
    let mut m = build("quiet.machine", QUIET, &registry, &options).expect("builds");

    assert_eq!(hz(&m, "sysclk"), Rational::integer(168_000_000));
    poke(&m, CR, peek(&m, CR) | CR_HSEON);
    a_moment(&mut m);
    poke(&m, CFGR, 1);
    a_moment(&mut m);
    assert_eq!(
        hz(&m, "sysclk"),
        Rational::integer(168_000_000),
        "an output nobody named must leave the machine file's rate alone"
    );
}
