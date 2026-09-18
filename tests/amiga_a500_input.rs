//! An A500's keyboard and mouse, through the record/replay seam, read by a
//! guest program on `machines/amiga-a500.machine`.
//!
//! What a person does reaches the machine the way `rsemu run --vnc` delivers
//! it: `host::input` events — keysyms and absolute pointer positions — posted on
//! a frontend channel, fanned out by a `Feed` to `AmigaKeyboardSink` and
//! `AmigaMouseSink`, and from there into the keyboard's and mouse's host
//! objects. So the translation, the keyboard's protocol, the 8520's shift
//! register, Denise's counters and CIA-A's port are all on the path, and the
//! guest is what reads the result.
//!
//! The claims, in the order `tests/joypad_replay.rs` makes them:
//!
//! 1. **Keys arrive as raw keycodes through the handshake.** The guest polls
//!    CIA-A's `ICR` for the serial flag, stores each `SDR` byte and pulses
//!    `KDAT` low by turning the serial port to output and back — chapter 8's
//!    "pulsing the SP line low then high". The keyboard sends nothing further
//!    until it has, so a run of bytes in order *is* the handshake working. A
//!    whole line delivered in one poll arrives whole, rather than overflowing
//!    the keyboard's ten-code buffer.
//! 2. **Motion counts on `JOY0DAT` and a click shows on `PA6`.** The guest
//!    samples both; the counters move by the deltas the pointer moved, spread
//!    over time rather than written at once, and the button waits for them.
//! 3. **A recording replays bit for bit**, out through the file format and
//!    back, to the same state hash at the same instant.
//! 4. **A board whose keyboard and mouse have no channel refuses to build.**
//!
//! The ROM is hand-assembled from the MC68000 user's manual's instruction
//! formats; register addresses and bits are the *Amiga Hardware Reference
//! Manual*'s (Appendix A, Appendix F, chapter 8). No emulator source was
//! consulted, and nothing from any Amiga ROM is in this file.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::HostObjects;
use rsemu::core::record::{Channel, InputLog, Recorder};
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::amiga::keyboard::{self, code, keys};
use rsemu::dev::amiga::mouse::mice;
use rsemu::host::input::amiga::{AmigaKeyboardSink, AmigaMouseSink};
use rsemu::host::input::{self, Feed, InputEvent, Keysym};
use rsemu::machine::{BuildOptions, Machine, catalog};

/// One slice of a run: a millisecond, a byte and a bit of keyboard traffic.
const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

/// Where the guest counts the bytes it has received, and stores them after.
const COUNT: u64 = 0x1000;
const BYTES: u64 = 0x1002;
/// Where it leaves its last samples of `JOY0DAT` and CIA-A's `PRA`.
const JOY0: u64 = 0x1100;
const PRA: u64 = 0x1102;
/// Paula's `POTGOR`, which the test reads itself.
const POTGOR: u64 = 0xDF_F016;

/// The guest, at `$F8000C`.
///
/// ```text
///   F8000C  move.b #$01,$BFE201     CIA-A DDRA: PA0 an output, PA6 an input
///   F80014  move.b #$00,$BFE001     OVL low
///   F8001C  move.w #2000,d3         wait ~3 ms: the keyboard's first sync bit
///   F80020  dbra   d3,*
///   F80024  bra.s  HS               answer it
///   F80026  L: move.w $DFF00A,$1100 JOY0DAT
///   F8002E  move.b $BFE001,$1102    PRA
///   F80036  move.b $BFED01,d0       ICR, which a read clears
///   F8003C  btst   #3,d0            SP: a byte has arrived
///   F80040  beq.s  L
///   F80042  move.b $BFEC01,d1       SDR
///   F80048  move.w $1000,d2
///   F8004C  lea    $1002,a0
///   F80050  move.b d1,0(a0,d2.w)
///   F80054  addq.w #1,$1000
///   F80058  HS: ori.b #$40,$BFEE01  CRA: SPMODE, output — KDAT low
///   F80060  move.w #100,d3          ~140 us, past the manual's 85
///   F80064  dbra   d3,*
///   F80068  andi.b #$BF,$BFEE01     back to input — KDAT released
///   F80070  bra.s  L
/// ```
#[rustfmt::skip]
const PROGRAM: [u16; 50] = [
    0x13fc, 0x0001, 0x00bf, 0xe201,
    0x13fc, 0x0000, 0x00bf, 0xe001,
    0x363c, 2000,
    0x51cb, 0xfffe,
    0x6032,
    0x31f9, 0x00df, 0xf00a, 0x1100,
    0x11f9, 0x00bf, 0xe001, 0x1102,
    0x1039, 0x00bf, 0xed01,
    0x0800, 0x0003,
    0x67e4,
    0x1239, 0x00bf, 0xec01,
    0x3438, 0x1000,
    0x41f8, 0x1002,
    0x1181, 0x2000,
    0x5278, 0x1000,
    0x0039, 0x0040, 0x00bf, 0xee01,
    0x363c, 100,
    0x51cb, 0xfffe,
    0x0239, 0x00bf, 0x00bf, 0xee01,
];

/// The program's last word, which `PROGRAM` has no room for.
const BRA_L: u16 = 0x60b4;

fn rom() -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x00F8_000Cu32.to_be_bytes());
    for (i, word) in PROGRAM.iter().chain([BRA_L].iter()).enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

fn options(hosts: &Arc<HostObjects>) -> BuildOptions {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom());
    // DF0 names a media slot; an empty one is an empty drive (the PC floppy
    // precedent, `tests/pc_at_ide.rs`). The front ends bind it for a user.
    options.realize.media.insert("df0", Vec::new());
    options.realize.hosts = Arc::clone(hosts);
    options
}

fn build(hosts: &Arc<HostObjects>) -> rsemu::Result<Machine> {
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build("amiga-a500", entry.source, &registry, &options(hosts))
}

/// The frontend's channel.
fn channel() -> Channel {
    input::channel("amiga-test")
}

/// The board, with a frontend feed wired to its keyboard and mouse and
/// registered with `recorder`, and the device doors the build opened
/// registered too.
fn board(recorder: &Arc<Recorder>) -> Machine {
    let hosts = Arc::new(HostObjects::new());
    let mut machine = build(&hosts).unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let feed = Arc::new(Feed::new());
    feed.attach(Arc::new(
        AmigaKeyboardSink::open(&hosts).expect("the board has a keyboard"),
    ));
    feed.attach(Arc::new(
        AmigaMouseSink::open(&hosts).expect("the board has a mouse"),
    ));
    recorder
        .register(channel(), input::sink(&feed))
        .expect("a fresh recorder takes channels");
    for name in keys::names(&hosts) {
        let kb = keys::get(&hosts, &name).unwrap().unwrap();
        recorder
            .register(keys::channel(&name), keys::sink(&kb))
            .unwrap();
    }
    for name in mice::names(&hosts) {
        let m = mice::get(&hosts, &name).unwrap().unwrap();
        recorder
            .register(mice::channel(&name), mice::sink(&m))
            .unwrap();
    }
    machine
        .set_recorder(Arc::clone(recorder))
        .expect("the board runs deterministically");
    machine.reset(ResetKind::Cold);
    machine
}

fn peek(m: &Machine, addr: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEBUG)
        .expect("a mapped address")
}

/// The raw codes the guest has received, decoded as a keyboard driver would.
fn received(m: &Machine) -> Vec<u8> {
    let n = peek(m, COUNT, Width::U16);
    (0..n)
        .map(|i| keyboard::code_of(peek(m, BYTES + i, Width::U8) as u8))
        .collect()
}

fn key(keysym: Keysym, down: bool) -> [u8; input::EVENT_BYTES] {
    InputEvent::Key { keysym, down }.encode()
}

fn pointer(x: u32, y: u32, buttons: u8) -> [u8; input::EVENT_BYTES] {
    InputEvent::Pointer { x, y, buttons }.encode()
}

/// The slice the pointer moves on.
const MOVE: usize = 45;

/// What happens before each slice.
///
/// The keyboard synchronises first: the guest answers its first sync bit after
/// a delay loop, and `$FD` and `$FE` are through by slice 8. A key moved before
/// then is folded into the start-up stream, which is a different test. The six
/// key movements enter the keyboard five milliseconds apart
/// (`keyboard::MOVEMENT_TICKS`), so the last is taken at slice 35.
fn script() -> Vec<Vec<u8>> {
    let mut s = vec![Vec::new(); MOVE + 11];
    // `b` pressed and released in one poll — two events, one post.
    s[10] = [
        key(Keysym::from_ascii(b'b'), true),
        key(Keysym::from_ascii(b'b'), false),
    ]
    .concat();
    // `!` from a client that does not send its shift.
    s[13] = key(Keysym::from_ascii(b'!'), true).to_vec();
    s[14] = key(Keysym::from_ascii(b'!'), false).to_vec();
    // The pointer is somewhere; then it moves right 40 and up 20 framebuffer
    // pixels — 40 counts right, 20 up, a count a pixel — and in the same poll
    // the left and right buttons go down where it stopped.
    s[MOVE - 1] = pointer(100, 100, 0).to_vec();
    s[MOVE] = [pointer(140, 80, 0), pointer(140, 80, 0b101)].concat();
    s
}

/// Run `machine`, posting `script[i]` before slice `i` (nothing, replaying).
fn drive(machine: &mut Machine, recorder: &Recorder, script: &[Vec<u8>]) -> Vec<(u16, u8)> {
    let mut samples = Vec::with_capacity(script.len());
    for post in script {
        if !post.is_empty() {
            recorder
                .post(&channel(), post)
                .expect("a registered channel");
        }
        machine.run_for(SLICE).expect("a deterministic run");
        samples.push((
            peek(machine, JOY0, Width::U16) as u16,
            peek(machine, PRA, Width::U8) as u8,
        ));
    }
    samples
}

#[test]
fn keys_reach_the_guest_as_raw_codes_one_handshake_at_a_time() {
    let recorder = Arc::new(Recorder::recording());
    let mut m = board(&recorder);
    let script = script();
    drive(&mut m, &recorder, &script[..MOVE]);
    assert_eq!(
        received(&m),
        [
            code::POWER_UP_STREAM,
            code::END_POWER_UP_STREAM,
            0x35,
            0x35 | code::KEY_UP,
            0x60,
            0x01,
            0x01 | code::KEY_UP,
            0x60 | code::KEY_UP,
        ],
        "$FD and $FE after the first handshake; B down and up; ! with its shift"
    );
}

/// A whole line typed in one poll — a VNC client's paste, or a frontend a
/// slice behind — reaches the guest whole and in order.
///
/// Before the keyboard kept a backlog of host movements this lost the twelfth
/// movement and every one after it — one on the line and ten in the type-ahead
/// buffer is all the keyboard holds — and the twelfth here is `,` *up*. On
/// Workbench the shell went on repeating a key nobody was holding. The
/// keyboard now takes a host movement every `keyboard::MOVEMENT_TICKS`; the
/// ten-code buffer is still there for a guest that stops answering.
#[test]
fn a_line_typed_in_one_poll_arrives_whole_and_nothing_is_left_down() {
    use rsemu::host::input::amiga::rawkey;
    let recorder = Arc::new(Recorder::recording());
    let mut m = board(&recorder);
    let text = b"hello, amiga 500";
    let mut s = vec![Vec::new(); 20 + 2 * text.len() * 5 + 10];
    s[10] = text
        .iter()
        .flat_map(|c| {
            [
                key(Keysym::from_ascii(*c), true),
                key(Keysym::from_ascii(*c), false),
            ]
        })
        .flatten()
        .collect();
    drive(&mut m, &recorder, &s);

    let mut want = vec![code::POWER_UP_STREAM, code::END_POWER_UP_STREAM];
    for c in text {
        let k = rawkey(Keysym::from_ascii(*c)).expect("a US key").code;
        want.extend([k, k | code::KEY_UP]);
    }
    let got = received(&m);
    assert!(
        !got.contains(&code::BUFFER_OVERFLOW),
        "no movement was lost: {got:02x?}"
    );
    assert_eq!(
        got, want,
        "every movement, in the order it was typed: each down has its up"
    );
}

#[test]
fn motion_counts_on_joy0dat_and_the_click_waits_for_it_on_pa6() {
    let recorder = Arc::new(Recorder::recording());
    let mut m = board(&recorder);
    let samples = drive(&mut m, &recorder, &script());

    let (joy_before, pra_before) = samples[MOVE - 1];
    assert_eq!(joy_before, 0, "nothing has moved yet");
    assert_eq!(pra_before & 0x40, 0x40, "PA6 high: switch open");

    // Forty counts at five a millisecond take eight: part of the way there
    // two slices in, and neither counter ever jumps.
    let x = |joy: u16| (joy & 0xff) as u8;
    let y = |joy: u16| (joy >> 8) as u8;
    let (mid, pra_mid) = samples[MOVE + 1];
    assert!(x(mid) > 0 && x(mid) < 40, "X on its way: {mid:#06x}");
    assert_eq!(pra_mid & 0x40, 0x40, "the click has not landed mid-move");
    for pair in samples.windows(2) {
        let dx = x(pair[1].0).wrapping_sub(x(pair[0].0)) as i8;
        assert!(
            (0..=6).contains(&dx),
            "a millisecond is five counts, give or take where the slice fell: {dx}"
        );
    }

    let (joy, pra) = *samples.last().expect("samples");
    assert_eq!(x(joy), 40, "right 40: JOY0DAT {joy:#06x}");
    assert_eq!(y(joy), (-20i8) as u8, "up 20 wraps below zero: {joy:#06x}");
    assert_eq!(pra & 0x40, 0, "the left button closed PA6");
    let potgor = peek(&m, POTGOR, Width::U16) as u16;
    assert_eq!(
        potgor & 0x0400,
        0,
        "and the right one pin 9: POTGOR {potgor:#06x}"
    );
    assert_eq!(potgor & 0x0100, 0x0100, "the middle one is up");
}

#[test]
fn a_recorded_session_replays_to_the_same_state_hash() {
    let recorder = Arc::new(Recorder::recording());
    let mut machine = board(&recorder);
    let samples = drive(&mut machine, &recorder, &script());
    let recorded = machine.state_hash().expect("deterministic mode hashes");
    let recorded_at = machine.now();
    let recorded_bytes = received(&machine);

    let bytes = recorder.log().encode().expect("a recording encodes");
    let log = InputLog::decode(&bytes).expect("and decodes");
    assert_eq!(log.events().len(), 5, "five posts");

    let replay = Arc::new(Recorder::replaying(log));
    let mut replayed = board(&replay);
    let quiet = vec![Vec::new(); script().len()];
    let replayed_samples = drive(&mut replayed, &replay, &quiet);

    assert_eq!(replayed.now(), recorded_at, "the same instant");
    assert_eq!(
        replayed.state_hash().expect("deterministic mode hashes"),
        recorded,
        "the same machine, bit for bit"
    );
    assert_eq!(received(&replayed), recorded_bytes);
    assert_eq!(
        replayed_samples, samples,
        "sampled the same on the same slices"
    );
    assert_eq!(replay.cursor(), 5, "every recorded post was delivered");

    // And a run where nobody typed or moved is a different machine.
    let silent = Arc::new(Recorder::recording());
    let mut still = board(&silent);
    drive(&mut still, &silent, &quiet);
    assert_ne!(still.state_hash().unwrap(), recorded);
    assert_eq!(
        received(&still),
        [code::POWER_UP_STREAM, code::END_POWER_UP_STREAM]
    );
}

#[test]
fn a_board_whose_keyboard_and_mouse_have_no_channel_refuses_to_build() {
    // Paula's serial port is a door too, and it is not what this test is
    // about, so its channel is registered before the table is sealed.
    use rsemu::host::chardev::ports;
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    let serial = ports::open(&hosts, "serial").expect("a port");
    recorder
        .register(ports::channel("serial"), ports::sink(&serial))
        .expect("a fresh recorder");
    hosts
        .seal(Arc::clone(&recorder))
        .expect("a table with one door, registered");
    let err = build(&hosts).expect_err("the keyboard has no channel");
    let text = format!("{err}");
    assert!(
        text.contains("amiga-keyboard:keyboard"),
        "the refusal names the input that bypassed the seam: {text}"
    );

    // With the keyboard's channel registered, it is the mouse's turn.
    let hosts = Arc::new(HostObjects::new());
    let recorder = Arc::new(Recorder::recording());
    let serial = ports::open(&hosts, "serial").expect("a port");
    recorder
        .register(ports::channel("serial"), ports::sink(&serial))
        .unwrap();
    let kb = keys::open(&hosts, keyboard::DEFAULT_KEYBOARD_PORT).unwrap();
    recorder
        .register(
            keys::channel(keyboard::DEFAULT_KEYBOARD_PORT),
            keys::sink(&kb),
        )
        .unwrap();
    hosts.seal(Arc::clone(&recorder)).unwrap();
    let err = build(&hosts).expect_err("the mouse has no channel");
    assert!(format!("{err}").contains("amiga-mouse:mouse"), "{err}");
}

#[test]
fn a_build_sealed_against_a_recorder_wires_both_doors_itself() {
    // What `rsemu run --record-input` does: the recorder goes in before the
    // build, and realize registers every door the devices opened.
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    let mut options = options(&hosts);
    options.realize.recorder = Some(Arc::clone(&recorder));
    let entry = catalog::machine("amiga-a500").unwrap();
    let registry = catalog::registry().unwrap();
    rsemu::machine::build("amiga-a500", entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("a sealed A500 builds: {e}"));
    assert!(recorder.knows(&keys::channel(keyboard::DEFAULT_KEYBOARD_PORT)));
    assert!(recorder.knows(&mice::channel(rsemu::dev::amiga::mouse::DEFAULT_MOUSE_PORT)));
}
