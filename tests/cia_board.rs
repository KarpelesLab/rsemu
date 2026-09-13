//! Two MOS 8520 CIAs on a board: the part reached over a real bus, on a real
//! clock, with real wires between them.
//!
//! `src/dev/mos/cia.rs` has the unit tests, and they drive the device directly.
//! What they cannot prove is that any of it survives the machine layer — that
//! the `clock = clk / 10` in the machine file is the clock the timers actually
//! count, that a `map` of sixteen bytes reaches the sixteen registers, that
//! `wire cia_a.pc -> cia_b.flag` carries a one-cycle strobe with its polarity
//! intact, and that a port pin declared bidirectional resolves against the
//! pull-up on the other end. That is what this file is for.
//!
//! `machines/tests/cia-pair.machine` is the board and is deliberately not in the
//! catalog; it also records, at length, the two things an Amiga board will have
//! to do that this one does not (byte lanes, and a 256-byte register stride).
//!
//! The register numbers below are the chip's own, and the bit constants are
//! from the 6526 and 8520 data sheets rather than from the device's private
//! ones, so that a rename inside the model cannot quietly rewrite what is being
//! asserted here.

#![cfg(all(feature = "cpu-m68k", feature = "dev-mos8520"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::mos::Cia;
use rsemu::machine::{BuildOptions, Machine, catalog};

/// The board.
const BOARD: &str = include_str!("../machines/tests/cia-pair.machine");

/// Where CIA-A's sixteen registers are, as the machine file maps them.
const CIA_A: u64 = 0x00a0_0000;
/// And CIA-B's.
const CIA_B: u64 = 0x00a1_0000;

/// `PRA`, `PRB` and the two direction registers.
const PRA: u64 = 0x0;
const PRB: u64 = 0x1;
const DDRA: u64 = 0x2;
/// Timer A's latch, low byte first.
const TA_LO: u64 = 0x4;
const TA_HI: u64 = 0x5;
/// The interrupt control register.
const ICR: u64 = 0xd;
/// Timer A's control register.
const CRA: u64 = 0xe;

/// `CRA0`: run.
const CR_START: u8 = 0x01;
/// `ICR0`: timer A underflowed.
const ICR_TA: u8 = 0x01;
/// `ICR4`: a negative edge on `/FLAG`.
const ICR_FLAG: u8 = 0x10;

/// The E clock: the board's 7.09379 MHz crystal divided by ten, which is what
/// an Amiga gives its CIAs and what `clock = clk / 10` means.
const E_CLOCK: u64 = 709_379;

/// A firmware image that parks the core: a reset vector pair and `BRA .`.
///
/// The CIAs are driven over the bus rather than by a program — what is under
/// test is a board, not an assembler — but virtual time has to pass for the
/// timers to count, and a core fetching out of an all-zero image would spend
/// that time taking exception vectors instead of idling.
fn parked() -> Vec<u8> {
    const ENTRY: u32 = 0x400;
    let mut image = vec![0u8; 0x402];
    // The two longwords a 68000 fetches out of reset: the supervisor stack
    // pointer (the top of this board's RAM) and the program counter.
    image[0x00..0x04].copy_from_slice(&0x0011_0000u32.to_be_bytes());
    image[0x04..0x08].copy_from_slice(&ENTRY.to_be_bytes());
    // `BRA .` — a branch to itself, M68000PRM.
    image[0x400..0x402].copy_from_slice(&0x60feu16.to_be_bytes());
    image
}

/// Build the board, and hand back the two CIAs its bindings captured.
fn boot(tag: &str) -> (Machine, Arc<Cia>, Arc<Cia>) {
    let cias: Arc<Captured<Cia>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cias);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("mos.8520", move |props| {
        let cia = Arc::new(Cia::new(props)?);
        kept.push(&cia);
        Ok(cia)
    });
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(bindings);
    options.realize.media.insert("firmware", parked());
    let registry = catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build(tag, BOARD, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let all = cias.all();
    assert_eq!(all.len(), 2, "the board has two CIAs");
    // Objects are constructed in declaration order, so this is `cia_a` then
    // `cia_b` — the order the machine file writes them in.
    (machine, Arc::clone(&all[0]), Arc::clone(&all[1]))
}

fn load(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte") as u8
}

fn store(m: &Machine, addr: u64, value: u8) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped byte");
}

#[test]
fn each_chip_answers_at_its_own_sixteen_addresses() {
    let (m, _a, _b) = boot("cia-pair.map");
    store(&m, CIA_A + DDRA, 0xf0);
    store(&m, CIA_B + DDRA, 0x0f);
    assert_eq!(load(&m, CIA_A + DDRA), 0xf0);
    assert_eq!(
        load(&m, CIA_B + DDRA),
        0x0f,
        "two chips, two register files"
    );

    // Sixteen registers and no seventeenth: the chip decodes four lines, so a
    // board that maps more than sixteen bytes is mapping a hole.
    store(&m, CIA_A + PRA, 0xa5);
    assert_eq!(load(&m, CIA_A + PRA), 0xa5 | 0x0f, "outputs, then pull-ups");
}

#[test]
fn the_timers_count_the_e_clock_the_machine_file_gives_them() {
    // The claim is about the *board*: `clock = clk / 10` on a 7.09379 MHz
    // crystal is 709379 Hz, and a timer with a latch of N underflows every
    // N + 1 of those.
    let (mut m, a, b) = boot("cia-pair.clock");

    // A latch of 35467 is a period of 35468 E ticks, which is 50 ms to five
    // figures. 65535 is the most a single timer can hold, and at this clock
    // that is 92 ms — which is the whole reason the chained mode exists.
    let latch: u16 = 35_467;
    store(&m, CIA_A + TA_LO, latch as u8);
    store(&m, CIA_A + TA_HI, (latch >> 8) as u8);
    store(&m, CIA_A + CRA, CR_START);
    assert_eq!(load(&m, CIA_A + ICR) & ICR_TA, 0);

    m.run_for(GlobalTime::from_nanos(25_000_000)).expect("runs");
    assert_eq!(
        load(&m, CIA_A + ICR) & ICR_TA,
        0,
        "half the period is not the period"
    );
    let half = a.ticks();
    assert_eq!(half, 17_734, "25 ms of a 709379 Hz clock, rounded down");
    assert_eq!(b.ticks(), half, "and both chips are on the same crystal");

    m.run_for(GlobalTime::from_nanos(25_000_000)).expect("runs");
    assert_eq!(
        load(&m, CIA_A + ICR) & ICR_TA,
        ICR_TA,
        "and the whole period is"
    );
    assert_eq!(a.ticks(), 35_468, "and 50 ms is one whole period of it");
    assert_eq!(
        a.ticks(),
        E_CLOCK / 20,
        "which is the rate said arithmetically"
    );
}

#[test]
fn the_pc_strobe_crosses_the_wire_into_the_other_chips_flag() {
    // `wire cia_a.pc -> cia_b.flag`. Both pins keep their true polarity, so the
    // one-cycle low pulse a PRB access makes is a negative edge at the other
    // end and sets ICR4 there — the parallel-port handshake, wired as the
    // silicon is.
    let (mut m, _a, _b) = boot("cia-pair.pc");
    assert_eq!(load(&m, CIA_B + ICR) & ICR_FLAG, 0, "nothing has happened");

    let _ = load(&m, CIA_A + PRB);
    assert_eq!(
        load(&m, CIA_B + ICR) & ICR_FLAG,
        ICR_FLAG,
        "a read of PRB strobed /PC, and /FLAG saw the edge"
    );
    // The read above cleared it, and nothing re-raises it on its own.
    m.run_for(GlobalTime::from_nanos(1_000_000)).expect("runs");
    assert_eq!(load(&m, CIA_B + ICR) & ICR_FLAG, 0);

    // A write strobes it too, and the strobe is one cycle, so a second access
    // is a second edge rather than one long low.
    store(&m, CIA_A + PRB, 0x00);
    m.run_for(GlobalTime::from_nanos(1_000_000)).expect("runs");
    store(&m, CIA_A + PRB, 0x00);
    assert_eq!(load(&m, CIA_B + ICR) & ICR_FLAG, ICR_FLAG);

    // And the other direction is not wired: CIA-B's /PC reaches nothing.
    let _ = load(&m, CIA_B + PRB);
    assert_eq!(load(&m, CIA_A + ICR) & ICR_FLAG, 0);
}

#[test]
fn a_port_pin_drives_its_neighbour_and_lets_go_when_it_is_an_input() {
    // `wire cia_a.pa0 -> cia_b.pa1 { pull = "up" }`. The pull-up is the one both
    // chips have on every port pin, and it is what an input pin presents; an
    // output pin overrides it.
    let (m, _a, _b) = boot("cia-pair.port");
    assert_eq!(
        load(&m, CIA_B + PRA) & 0x02,
        0x02,
        "two inputs, one pull-up"
    );

    store(&m, CIA_A + DDRA, 0x01); // pa0 an output
    store(&m, CIA_A + PRA, 0x00);
    assert_eq!(
        load(&m, CIA_B + PRA) & 0x02,
        0x00,
        "an output stage beats the resistor"
    );
    store(&m, CIA_A + PRA, 0x01);
    assert_eq!(load(&m, CIA_B + PRA) & 0x02, 0x02);

    store(&m, CIA_A + PRA, 0x00);
    assert_eq!(load(&m, CIA_B + PRA) & 0x02, 0x00);
    store(&m, CIA_A + DDRA, 0x00); // and back to an input
    assert_eq!(
        load(&m, CIA_B + PRA) & 0x02,
        0x02,
        "the net floats back up to the pull-up"
    );
}

#[test]
fn the_tod_pin_belongs_to_the_board_and_is_not_the_e_clock() {
    // Nothing on this board drives TOD, which is the point: on an Amiga the two
    // chips get *different* rates from the display and neither is the timer
    // clock. Time passing must therefore not advance either counter.
    let (mut m, a, b) = boot("cia-pair.tod");
    m.run_for(GlobalTime::from_nanos(100_000_000))
        .expect("runs");
    assert!(a.ticks() > 0, "the timers counted");
    assert_eq!(a.tod(), 0, "and the TOD counter did not");
    assert_eq!(b.tod(), 0);

    // 50 vertical blanks into one and 313 line ticks into the other: a PAL
    // frame, as the two pins would see it.
    for _ in 0..50 {
        a.tod_pulse();
    }
    for _ in 0..313 {
        b.tod_pulse();
    }
    assert_eq!(a.tod(), 50);
    assert_eq!(b.tod(), 313, "two pins, two rates, one board");
}

#[test]
fn the_board_runs_deterministically_and_survives_a_snapshot() {
    // The machine-level regression `CLAUDE.md` asks every board for: the same
    // run reaches the same state, and a snapshot taken part way through is that
    // state rather than an approximation of it.
    let span = GlobalTime::from_nanos(20_000_000);

    let (mut one, _, _) = boot("cia-pair.det.1");
    let (mut two, _, _) = boot("cia-pair.det.2");
    for m in [&mut one, &mut two] {
        // Timer A free-running, timer B counting its underflows, both
        // interrupts armed: every moving part of the chip at once.
        store(m, CIA_A + TA_LO, 0xff);
        store(m, CIA_A + TA_HI, 0x03);
        store(m, CIA_A + 0x6, 0x05);
        store(m, CIA_A + 0x7, 0x00);
        store(m, CIA_A + CRA, CR_START);
        store(m, CIA_A + 0xf, CR_START | 0x40);
        store(m, CIA_A + ICR, 0x80 | 0x03);
        m.run_for(span).expect("runs");
    }
    let hash = one.state_hash().expect("hashable");
    assert_eq!(hash, two.state_hash().expect("hashable"), "deterministic");

    let bytes = one.save().expect("saveable");
    one.run_for(span).expect("runs");
    let later = one.state_hash().expect("hashable");
    assert_ne!(later, hash, "the machine moved");

    one.load(&bytes).expect("loadable");
    assert_eq!(one.state_hash().expect("hashable"), hash, "and came back");
    one.run_for(span).expect("runs");
    assert_eq!(
        one.state_hash().expect("hashable"),
        later,
        "and ran the same way the second time"
    );
}
