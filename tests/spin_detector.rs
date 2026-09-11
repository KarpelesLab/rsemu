//! The spin detector on a whole board (`core::spin`).
//!
//! The unit tests in `cpu::arm::v7m` say the interpreter's hooks fire. This
//! says the thing the feature actually exists for: firmware polling a word that
//! never changes, on a *machine*, with the run stopping and naming the loop
//! instead of hanging until somebody's patience runs out.
//!
//! `stm32f407` because it is a real part with a real memory map — flash at
//! `0x0800_0000`, SRAM at `0x2000_0000`, a vector table the core reads at reset
//! — so the report has a genuine address and a genuine region name in it, and
//! nothing is arranged for the test's convenience.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::spin;
use rsemu::core::trace::EventKind;
use rsemu::machine::{Machine, catalog};

/// Where the part's flash is.
const FLASH: u32 = 0x0800_0000;
/// Where the poll loop's `LDR` lives, once the vector table is out of the way.
const LOOP_PC: u32 = FLASH + 0x0a;
/// The SRAM word the firmware waits on. Nothing on this board ever writes it.
const POLLED: u32 = 0x2000_0100;

/// Firmware: set up a pointer, then wait forever for a word that never sets.
///
/// ```text
///   0x08000000  .word 0x20010000     ; initial SP
///   0x08000004  .word 0x08000009     ; reset vector, Thumb
///   0x08000008  ldr  r1, [pc, #4]    ; r1 = 0x20000100
///   0x0800000a  ldr  r0, [r1]        ; <- the loop
///   0x0800000c  cmp  r0, #0
///   0x0800000e  beq  0x0800000a
///   0x08000010  .word 0x20000100
/// ```
///
/// A `cmp` and a conditional branch rather than a bare `b`, because this is
/// meant to be the thing a compiler emits for `while (!(*flag)) ;` rather than
/// the shortest encoding that reproduces the symptom.
fn firmware() -> Vec<u8> {
    let mut image = Vec::new();
    image.extend_from_slice(&0x2001_0000u32.to_le_bytes()); // SP
    image.extend_from_slice(&(FLASH + 9).to_le_bytes()); // reset vector
    for half in [
        0x4901u16, // ldr r1, [pc, #4]
        0x6808,    // ldr r0, [r1]
        0x2800,    // cmp r0, #0
        0xd0fc,    // beq .-6
    ] {
        image.extend_from_slice(&half.to_le_bytes());
    }
    image.extend_from_slice(&POLLED.to_le_bytes());
    image
}

/// The board, with the poll loop in its flash.
fn board() -> Machine {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// A millisecond of virtual time — at 16 MHz off the HSI that is sixteen
/// thousand cycles, thousands of times round the loop.
fn a_millisecond() -> GlobalTime {
    GlobalTime::from_nanos(1_000_000)
}

#[test]
fn run_for_with_a_budget_returns_the_loop_as_an_error() {
    let mut m = board();
    let detector = m.arm_spin_detector(1000);
    detector.set_stops_the_run(true);

    let err = m
        .run_for(a_millisecond())
        .expect_err("the firmware is stuck and the run was told to say so");

    let rsemu::Error::Spin(event) = &err else {
        panic!("the wrong error came back: {err}");
    };
    assert_eq!(event.kind, EventKind::SPIN);
    assert_eq!(event.cpu, 0, "the board declares one processor");
    assert_eq!(u64::from(LOOP_PC), event.pc);
    assert_eq!(u64::from(POLLED), event.addr);
    assert_eq!(event.value, 0);
    assert_eq!(event.count, 1000);
    assert!(
        event.region.is_some(),
        "the report should name what answered at {POLLED:#x}"
    );

    // And the whole point: the sentence says where to look.
    let text = err.to_string();
    assert!(text.contains("cpu0"), "{text}");
    assert!(text.contains("0x0800000a"), "{text}");
    assert!(text.contains("0x20000100"), "{text}");

    // The error did not consume the finding: the detector still holds it, so a
    // caller that wants the structured form after catching the error has it.
    assert_eq!(detector.events(), vec![event.clone()]);

    // And resuming is not refused by the same finding a second time. The loop
    // is as stuck as it was, but its streak is past the threshold and will not
    // trip again until the value changes, so there is nothing new to report.
    m.run_for(a_millisecond())
        .expect("the same loop does not report twice");
}

#[test]
fn without_the_budget_the_run_completes_and_the_finding_is_still_recorded() {
    // The default: report, do not interfere. This is the mode that is meant to
    // be safe to leave on.
    let mut m = board();
    let detector = m.arm_spin_detector(1000);
    m.run_for(a_millisecond()).expect("the run is not stopped");
    let events = detector.events();
    assert_eq!(
        events.len(),
        1,
        "one line for a loop, not one per iteration"
    );
    assert_eq!(events[0].pc, u64::from(LOOP_PC));
}

#[test]
fn the_detector_does_not_change_the_state_hash() {
    // The determinism gate. A diagnostic that perturbed the guest would be
    // worse than no diagnostic: the hang it was hunting would move.
    let mut plain = board();
    plain.run_for(a_millisecond()).expect("runs");
    let expected = plain.state_hash().expect("a deterministic machine");

    let mut watched = board();
    let detector = watched.arm_spin_detector(1000);
    watched.run_for(a_millisecond()).expect("runs");
    assert_eq!(
        watched.state_hash().expect("a deterministic machine"),
        expected,
        "arming the detector moved the machine"
    );
    assert!(!detector.events().is_empty(), "it really was watching");

    // A second armed run reaches the same state *and* reports the same thing,
    // which is the other half of determinism: two identical runs report
    // identically.
    let mut again = board();
    let second = again.arm_spin_detector(1000);
    again.run_for(a_millisecond()).expect("runs");
    assert_eq!(again.state_hash().expect("deterministic"), expected);
    assert_eq!(second.events(), detector.events());
}

#[test]
fn disarming_puts_the_board_back_where_it_was() {
    let mut m = board();
    let detector = m.arm_spin_detector(1000);
    assert!(m.spin_detector().is_some());
    m.run_for(a_millisecond()).expect("runs");
    assert!(!detector.events().is_empty());

    m.disarm_spin_detector();
    assert!(m.spin_detector().is_none());
    assert!(detector.events().is_empty(), "disarming forgets as well");
    m.run_for(a_millisecond()).expect("runs");
    assert!(detector.events().is_empty(), "and nothing reports after it");
}

#[test]
fn the_default_threshold_is_what_the_documentation_says() {
    // Named, because a caller that does not want to think about the number
    // should be able to write this and get a sensible one.
    let mut m = board();
    let detector = m.arm_spin_detector(spin::DEFAULT_THRESHOLD);
    assert_eq!(detector.threshold(), 10_000);
    // Ten milliseconds, not one: the loop is six cycles and the part comes out
    // of reset on the 16 MHz HSI, so ten thousand iterations is about four
    // milliseconds of virtual time. Which is the point of the default — it is
    // long enough that nothing ordinary reaches it.
    m.run_for(GlobalTime::from_nanos(10_000_000)).expect("runs");
    assert_eq!(detector.events()[0].count, 10_000);
}
