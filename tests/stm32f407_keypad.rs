//! Firmware scanning the `stm32f407` board's keypad matrix over `st.gpio`.
//!
//! The other keypad tests drive wires directly. This one is the end of the
//! chain the device exists for: a **host keypress** reaches a **guest program**
//! that scans the matrix the way firmware does, through eight real GPIO pins on
//! eight nets that resolve themselves.
//!
//! It is worth having because every link in that chain is a different kind of
//! thing and each can be wrong on its own:
//!
//! * `PUPDR` has to hold an input pin high with nothing driving it. This is the
//!   defect that made the whole change necessary — a row pin with an internal
//!   pull-up used to read **low** when unconnected, so scanning firmware saw
//!   every key held at once and there was no way for a machine file to say
//!   otherwise.
//! * `OTYPER` has to make a row that is *not* being scanned let go of the line
//!   rather than drive it high, or the port fights the keypad on every
//!   unselected row.
//! * `IDR` has to report the **pin** and not the port's own intention, which is
//!   what reading a column back amounts to.
//! * The keypad has to short a row net to a column net while, and only while, a
//!   key is down.
//!
//! The program is straight-line rather than a loop: four row strobes with an
//! `IDR` sample after each, stored where the test can read them. Unrolled
//! because a hand-assembled scan loop is a test of the assembler in this file
//! rather than of the board, and because four separate samples say *which* row
//! strobe found the key — a loop that stopped at the first hit could not
//! distinguish "found on row 1" from "found on every row", which is exactly the
//! failure the pull-up defect produced.
//!
//! Sources: ST **RM0090** rev 21 §8.3.10 (the output stage and the pull
//! resistors), §8.4.1–§8.4.6 (the register map), and the ARMv7-M Architecture
//! Reference Manual, ARM **DDI 0403**, A7.7 for the six Thumb-2 encodings. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-stm32f407", feature = "dev-keypad-matrix"))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::hosts::HostObjects;
use rsemu::core::record::Recorder;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::keypad::{DEFAULT_KEYPAD_PORT, Keys, keys};
use rsemu::machine::{Machine, catalog};

/// `GPIOE`'s base: AHB1 + four GPIO strides (RM0090 Table 1).
const GPIOE: u32 = 0x4002_1000;
/// Where SRAM1 starts, and where the four samples land.
const SRAM1: u64 = 0x2000_0000;

/// The initial stack pointer: the top of SRAM2.
const STACK: u32 = 0x2002_0000;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;

// ---------------------------------------------------------------------------
// The same very small Thumb-2 assembler the other `stm32f407` tests use
// ---------------------------------------------------------------------------
//
// Copied rather than shared because a `tests/` file is its own crate. Each
// encoding is checkable by hand against ARM DDI 0403 A7.7.

/// `MOVW Rd, #imm16` — encoding T3, A7.7.76.
fn movw(d: u16, imm16: u16) -> [u16; 2] {
    let i = (imm16 >> 11) & 1;
    let imm4 = (imm16 >> 12) & 0xf;
    let imm3 = (imm16 >> 8) & 7;
    let imm8 = imm16 & 0xff;
    [0xf240 | (i << 10) | imm4, (imm3 << 12) | (d << 8) | imm8]
}

/// `MOVT Rd, #imm16` — encoding T1, A7.7.79. The same field layout.
fn movt(d: u16, imm16: u16) -> [u16; 2] {
    let [a, b] = movw(d, imm16);
    [a | 0x0080, b]
}

/// `STR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.158.
fn str_imm(t: u16, n: u16, off: u16) -> u16 {
    assert!(
        off.is_multiple_of(4) && off / 4 < 32,
        "STR T1 cannot reach {off:#x}"
    );
    0x6000 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.42. The same fields as `STR`.
fn ldr_imm(t: u16, n: u16, off: u16) -> u16 {
    assert!(
        off.is_multiple_of(4) && off / 4 < 32,
        "LDR T1 cannot reach {off:#x}"
    );
    0x6800 | ((off / 4) << 6) | (n << 3) | t
}

/// `MOVS Rd, #imm8` — encoding T1, A7.7.76.
fn movs(d: u16, imm8: u16) -> u16 {
    0x2000 | (d << 8) | imm8
}

/// Load a 32-bit constant into a low register: `MOVW` then `MOVT`.
fn load32(d: u16, value: u32) -> Vec<u16> {
    let mut out = Vec::new();
    out.extend(movw(d, value as u16));
    out.extend(movt(d, (value >> 16) as u16));
    out
}

/// `MODER`: PE0–PE3 general-purpose output (`01`), PE4–PE7 input (`00`).
const MODER: u32 = 0x0000_0055;
/// `OTYPER`: PE0–PE3 open-drain, so an unselected row **lets go** of the line
/// instead of driving it high and fighting the keypad (RM0090 §8.3.10).
const OTYPER: u32 = 0x0000_000f;
/// `PUPDR`: a pull-up (`01`) on all eight pins. On the rows it is what holds an
/// unselected line high; on the columns it is the whole reason an unpressed
/// column reads as a one.
const PUPDR: u32 = 0x0000_5555;

/// The firmware: configure the port, then strobe each row low in turn and
/// sample `IDR` after each.
///
/// ```text
///   0x000: .word 0x20020000        ; initial SP
///   0x004: .word entry|1           ; reset vector
///
///   entry: r0 = GPIOE
///          MODER  = 0x55           ; PE0-3 out, PE4-7 in
///          OTYPER = 0x0f           ; the rows are open-drain
///          PUPDR  = 0x5555         ; pull-ups on all eight
///          ODR    = 0x0f           ; every row released
///          ODR = 0x0e ; r4 = IDR   ; strobe PE0 low, sample
///          ODR = 0x0d ; r5 = IDR
///          ODR = 0x0b ; r6 = IDR
///          ODR = 0x07 ; r7 = IDR
///          ODR    = 0x0f           ; and let go again
///          r1 = SRAM1
///          SRAM1[0..16] = r4, r5, r6, r7
///          b .
/// ```
fn firmware() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    main.extend(load32(0, GPIOE));
    main.extend(load32(1, MODER));
    main.push(str_imm(1, 0, 0x00));
    main.extend(load32(1, OTYPER));
    main.push(str_imm(1, 0, 0x04));
    main.extend(load32(1, PUPDR));
    main.push(str_imm(1, 0, 0x0c));

    // One strobe per row, into a register apiece. `0x0f` with row n cleared.
    for (row, sample) in [0u16, 1, 2, 3].into_iter().zip([4u16, 5, 6, 7]) {
        main.push(movs(1, 0x0f & !(1 << row)));
        main.push(str_imm(1, 0, 0x14)); // ODR
        main.push(ldr_imm(sample, 0, 0x10)); // IDR
    }
    main.push(movs(1, 0x0f));
    main.push(str_imm(1, 0, 0x14));

    main.extend(load32(1, SRAM1 as u32));
    for (i, sample) in [4u16, 5, 6, 7].into_iter().enumerate() {
        main.push(str_imm(sample, 1, (i * 4) as u16));
    }
    main.push(0xe7fe); // b .

    let mut image = vec![0u8; 0x300];
    fn word(image: &mut [u8], at: usize, value: u32) {
        image[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }
    word(&mut image, 0x00, STACK);
    word(&mut image, 0x04, ENTRY | 1);
    let at = ENTRY as usize;
    assert!(at + main.len() * 2 <= image.len(), "the program overran");
    for (i, half) in main.iter().enumerate() {
        image[at + i * 2..at + i * 2 + 2].copy_from_slice(&half.to_le_bytes());
    }
    image
}

/// Build the shipped board with `hosts` and the scan program in its firmware
/// slot.
fn boot(hosts: &Arc<HostObjects>) -> Machine {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options.realize.hosts = Arc::clone(hosts);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Long enough for a few hundred instructions at 16 MHz from the HSI.
const RUN: GlobalTime = GlobalTime::from_nanos(2_000_000);

/// The four `IDR` samples the program left in SRAM, masked to the columns.
fn scan(machine: &Machine) -> [u64; 4] {
    let space = machine.space("mem").expect("the memory space");
    let mut out = [0u64; 4];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = space
            .read(SRAM1 + (i * 4) as u64, Width::U32, MemAttrs::DEBUG)
            .expect("a mapped word")
            & 0xf0;
    }
    out
}

/// The host keypad this board's `keypad` object opened.
fn pad(hosts: &Arc<HostObjects>) -> Arc<Keys> {
    keys::open(hosts, DEFAULT_KEYPAD_PORT).expect("the board's keypad port")
}

/// Every column high: `PE4`–`PE7` all ones.
const ALL_UP: u64 = 0xf0;

#[test]
fn firmware_scanning_the_matrix_finds_the_key_the_host_pressed() {
    let hosts = Arc::new(HostObjects::new());
    let mut machine = boot(&hosts);
    machine.reset(rsemu::core::device::ResetKind::Cold);
    // `"6"` is row 1, column 2 in this board's layout.
    assert!(pad(&hosts).press("6", true), "the board names its keys");
    machine.run_for(RUN).expect("the program runs");

    let rows = scan(&machine);
    assert_eq!(
        rows[1],
        ALL_UP & !(1 << (4 + 2)),
        "strobing row 1 must pull column 2 low and leave the other three alone"
    );
    for row in [0usize, 2, 3] {
        assert_eq!(
            rows[row], ALL_UP,
            "row {row} holds no pressed key, so its strobe reaches no column"
        );
    }
}

#[test]
fn an_untouched_keypad_reads_as_sixteen_keys_up() {
    // The defect the tri-state model exists for, at board level: before
    // `PUPDR` decided anything, an input pin with nothing on it read **low**,
    // so this assertion would have failed on all four rows and firmware would
    // have believed every key was held.
    let hosts = Arc::new(HostObjects::new());
    let mut machine = boot(&hosts);
    machine.reset(rsemu::core::device::ResetKind::Cold);
    machine.run_for(RUN).expect("the program runs");
    assert_eq!(scan(&machine), [ALL_UP; 4]);
}

#[test]
fn two_keys_in_one_column_are_each_found_in_their_own_row_strobe() {
    let hosts = Arc::new(HostObjects::new());
    let mut machine = boot(&hosts);
    machine.reset(rsemu::core::device::ResetKind::Cold);
    // `"1"` is (0, 0) and `"4"` is (1, 0): one column, two rows.
    let pad = pad(&hosts);
    assert!(pad.press("1", true));
    assert!(pad.press("4", true));
    machine.run_for(RUN).expect("the program runs");

    let rows = scan(&machine);
    let col0 = ALL_UP & !(1 << 4);
    assert_eq!(rows[0], col0, "row 0's strobe finds `1`");
    assert_eq!(rows[1], col0, "row 1's strobe finds `4`");
    assert_eq!(rows[2], ALL_UP);
    assert_eq!(rows[3], ALL_UP);
}

#[test]
fn three_corners_of_a_rectangle_ghost_the_fourth_because_this_pad_has_no_diodes() {
    // Not a modelling wart: a membrane keypad with no series diodes really does
    // this, and firmware that scans one has to live with it. `1`, `2` and `4`
    // are (0,0), (0,1) and (1,0) — so strobing row 1 finds `4` on column 0 as
    // it should, and *also* column 1, through 4 → col0 → 1 → row0 → 2 → col1.
    // `5` at (1,1) is not pressed.
    let hosts = Arc::new(HostObjects::new());
    let mut machine = boot(&hosts);
    machine.reset(rsemu::core::device::ResetKind::Cold);
    let pad = pad(&hosts);
    for key in ["1", "2", "4"] {
        assert!(pad.press(key, true), "`{key}` is on this pad");
    }
    machine.run_for(RUN).expect("the program runs");

    let rows = scan(&machine);
    assert_eq!(
        rows[1],
        ALL_UP & !(1 << 4) & !(1 << 5),
        "column 1 is the ghost: no key at (1,1) is down"
    );
    assert_eq!(rows[2], ALL_UP);
    assert_eq!(rows[3], ALL_UP);
}

#[test]
fn a_recorded_keypress_replays_to_the_same_scan_and_the_same_state_hash() {
    // The same contract `tests/joypad_replay.rs` holds the console pads to: a
    // press that crossed the seam is in the log, and the log alone reproduces
    // the run.
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    let pad = keys::open(&hosts, DEFAULT_KEYPAD_PORT).expect("open before the build");
    recorder
        .register(keys::channel(DEFAULT_KEYPAD_PORT), keys::sink(&pad))
        .expect("the keypad's door");
    let mut machine = boot(&hosts);
    machine
        .set_recorder(Arc::clone(&recorder))
        .expect("a deterministic board takes a recorder");
    machine.reset(rsemu::core::device::ResetKind::Cold);

    // Key index 6 is (1, 2), held. Posted rather than pressed directly, so it
    // travels the recorded path.
    recorder
        .post(&keys::channel(DEFAULT_KEYPAD_PORT), &[6, 1])
        .expect("a registered channel");
    machine.run_for(RUN).expect("the program runs");
    let recorded = machine.state_hash().expect("a deterministic board");
    let recorded_scan = scan(&machine);
    let recorded_at = machine.now();
    assert_eq!(recorded_scan[1], ALL_UP & !(1 << 6));
    assert_eq!(recorder.log().len(), 1, "one press, one logged event");

    // A fresh board, driven by nothing but the log.
    let log =
        rsemu::core::record::InputLog::decode(&recorder.log().encode().expect("the log encodes"))
            .expect("and decodes");
    assert_eq!(log.events()[0].channel.to_string(), "keypad:keypad");
    let replay = Arc::new(Recorder::replaying(log));
    let hosts = Arc::new(HostObjects::new());
    let pad = keys::open(&hosts, DEFAULT_KEYPAD_PORT).expect("open before the build");
    replay
        .register(keys::channel(DEFAULT_KEYPAD_PORT), keys::sink(&pad))
        .expect("the keypad's door");
    let mut replayed = boot(&hosts);
    replayed
        .set_recorder(Arc::clone(&replay))
        .expect("a replaying recorder");
    replayed.reset(rsemu::core::device::ResetKind::Cold);
    replayed.run_for(RUN).expect("the program runs");

    assert_eq!(replayed.now(), recorded_at);
    assert_eq!(scan(&replayed), recorded_scan);
    assert_eq!(
        replayed.state_hash().expect("a deterministic board"),
        recorded,
        "the whole machine, not just the pins"
    );
    assert_eq!(replay.cursor(), 1);
}
