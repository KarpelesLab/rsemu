//! Buttons, through the record/replay seam and nowhere else
//! (`ROADMAP.md` §4.5; `CLAUDE.md`, determinism).
//!
//! The Game Boy's joypad and the Master System's I/O chip both used to carry
//! the same paragraph in their documentation: *"that seam does not exist yet,
//! so `set_pressed` is the interim door"*. It exists, and this file is what
//! makes the conversion checkable rather than merely written down.
//!
//! Both boards are inline: a core, a ROM, a kilobyte of RAM and the input
//! device, and nothing else. They model no product, which is the point — what
//! is being asserted is the *seam*, and a board with a picture on it would only
//! add ways for the assertion to be about something else.
//!
//! Three claims per console, in the order that makes them mean anything:
//!
//! 1. **Pressing changes the run.** A guest that polls its pad reaches a
//!    different state when somebody pressed something. Without this, "record
//!    and replay agree" would be satisfied by a seam that dropped every press.
//! 2. **A recording replays bit for bit.** The recording goes out through the
//!    file format and comes back, and a fresh machine driven only by it reaches
//!    the same state hash at the same instant.
//! 3. **A board whose pad has no channel refuses to build.** The seal, on the
//!    host object the device opens by name.

#![cfg(any(
    all(feature = "cpu-sm83", feature = "dev-gb"),
    all(feature = "cpu-z80", feature = "dev-sms")
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::HostObjects;
use rsemu::core::record::{Channel, InputLog, Recorder};
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, catalog};

/// How long each slice of a run is.
///
/// A millisecond: dozens of loop iterations on either console, and short enough
/// that a press lands in the middle of the run rather than at one end of it.
const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

/// Where each guest program leaves what it saw: the buttons at `+0`, the loop
/// count it was on at `+1`, and the counter itself at `+2`.
const SEEN: u64 = 0xc000;

/// Build a board from source, against a host-object table the caller keeps.
fn build(name: &str, source: &str, rom: Vec<u8>, hosts: &Arc<HostObjects>) -> Machine {
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.media.insert("firmware", rom);
    options.realize.hosts = Arc::clone(hosts);
    let registry = catalog::registry().expect("this build's registry");
    match rsemu::machine::build(name, source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("{name} does not realize: {e}"),
    }
}

/// One byte of the memory space, without disturbing anything.
fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped byte") as u8
}

/// Run `machine` in slices, posting `presses[i]` before slice `i`.
///
/// Posting before a slice rather than at some wall-clock moment is what keeps
/// the *test* deterministic, and it changes nothing about what is proved: the
/// recorder does not know when the host called it, only which round boundary
/// the machine drained it on.
fn drive(
    machine: &mut Machine,
    recorder: &Recorder,
    channel: &Channel,
    presses: &[&[u8]],
) -> (u64, [u8; 3]) {
    for press in presses {
        if !press.is_empty() {
            recorder.post(channel, press).expect("a registered channel");
        }
        machine.run_for(SLICE).expect("a deterministic run");
    }
    let seen = [
        peek(machine, SEEN),
        peek(machine, SEEN + 1),
        peek(machine, SEEN + 2),
    ];
    (
        machine.state_hash().expect("deterministic mode hashes"),
        seen,
    )
}

/// A slice with nothing posted, for the silent control and for a replay.
const QUIET: &[u8] = b"";

// ---------------------------------------------------------------------------
// The Game Boy
// ---------------------------------------------------------------------------

#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
mod gameboy {
    use super::*;
    use rsemu::dev::gb::joypad::{DEFAULT_PAD_PORT, pads};

    /// An SM83, a ROM at the reset vector, work RAM, and the joypad at `$FF00`.
    const BOARD: &str = r#"
    machine "gb-pad" {
      osc master = 4194304 Hz
      space mem { width = 16, unassigned = read-as-ones }
      object cpu "cpu.sm83" { clock = master / 4, space = mem, post-boot = true }
      object boot "rom" { size = 32K, image = "firmware" }
      object wram "ram" { size = 8K }
      object pad "gb.joypad" {}
      map mem 0x0000 size 0x8000 = boot
      map mem 0xc000 size 0x2000 = wram
      map mem 0xff00 size 0x0001 = pad
    }
    "#;

    /// The guest program, at `$0100` — where a post-boot SM83 starts.
    ///
    /// ```text
    ///   0100: 3e 20        ld   a, $20      ; bit 4 low: select the d-pad row
    ///   0102: e0 00        ldh  ($00), a
    ///   0104: 21 02 c0     ld   hl, $c002   ; the iteration counter
    ///   0107: 34      L:   inc  (hl)
    ///   0108: f0 00        ldh  a, ($00)
    ///   010a: 2f           cpl              ; the register is active low
    ///   010b: e6 0f        and  $0f
    ///   010d: 28 f8        jr   z, L        ; nothing held: go round again
    ///   010f: ea 00 c0     ld   ($c000), a  ; which buttons
    ///   0112: 7e           ld   a, (hl)
    ///   0113: ea 01 c0     ld   ($c001), a  ; and how long it took to see them
    ///   0116: 18 fe        jr   $
    /// ```
    ///
    /// The iteration count is what makes the assertion about *when*: a press
    /// delivered one round boundary later is a different number in `$C001`.
    const PROGRAM: [u8; 25] = [
        0x3e, 0x20, 0xe0, 0x00, 0x21, 0x02, 0xc0, 0x34, 0xf0, 0x00, 0x2f, 0xe6, 0x0f, 0x28, 0xf8,
        0xea, 0x00, 0xc0, 0x7e, 0xea, 0x01, 0xc0, 0x18, 0xfe, 0x00,
    ];

    fn rom() -> Vec<u8> {
        let mut image = vec![0u8; 0x8000];
        image[0x0100..0x0100 + PROGRAM.len()].copy_from_slice(&PROGRAM);
        image
    }

    /// The board, with its joypad registered as a record/replay channel.
    ///
    /// Three lines, and they are the whole conversion: the pad is a host object
    /// opened by name, `pads::channel` is that name as a channel, and
    /// `pads::sink` is what a payload does once the machine has decided when.
    fn board(recorder: &Arc<Recorder>) -> Machine {
        let hosts = Arc::new(HostObjects::new());
        let pad = pads::open(&hosts, DEFAULT_PAD_PORT).expect("a pad before the build");
        recorder
            .register(pads::channel(DEFAULT_PAD_PORT), pads::sink(&pad))
            .expect("a fresh recorder takes channels");
        let mut machine = build("gb-pad.machine", BOARD, rom(), &hosts);
        machine
            .set_recorder(Arc::clone(recorder))
            .expect("the board runs deterministically");
        machine.reset(ResetKind::Cold);
        machine
    }

    /// Right and Down held, then everything released: bits 0 and 3 of the mask
    /// `GbPad::set_buttons` takes.
    const HELD: &[u8] = &[0b0000_1001];
    const RELEASED: &[u8] = &[0b0000_0000];

    fn script() -> Vec<&'static [u8]> {
        vec![QUIET, QUIET, HELD, QUIET, RELEASED, QUIET]
    }

    #[test]
    fn pressing_a_button_changes_where_the_run_ends_up() {
        let recorder = Arc::new(Recorder::recording());
        let mut quiet = board(&recorder);
        let (silent, silent_seen) = drive(
            &mut quiet,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &[QUIET; 6],
        );
        assert_eq!(silent_seen[0], 0, "nobody pressed anything");

        let recorder = Arc::new(Recorder::recording());
        let mut pressed = board(&recorder);
        let (held, held_seen) = drive(
            &mut pressed,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &script(),
        );
        assert_eq!(
            held_seen[0], 0b0000_1001,
            "the guest read Right and Down off $FF00"
        );
        assert_ne!(silent, held, "a press is guest-visible");
        assert_eq!(recorder.log().len(), 2, "two posts, two logged events");
    }

    #[test]
    fn a_recorded_session_replays_to_the_same_state_hash() {
        let recorder = Arc::new(Recorder::recording());
        let mut machine = board(&recorder);
        let (recorded, recorded_seen) = drive(
            &mut machine,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &script(),
        );
        let recorded_at = machine.now();

        // Out through the file format and back, so what is replayed is what a
        // `.trace` on disk would hold rather than a live object.
        let bytes = recorder.log().encode().expect("a recording encodes");
        let log = InputLog::decode(&bytes).expect("and decodes");
        assert_eq!(log.events()[0].channel.to_string(), "pad:gb-joypad");

        let replay = Arc::new(Recorder::replaying(log));
        let mut replayed = board(&replay);
        let (hash, seen) = drive(
            &mut replayed,
            &replay,
            &pads::channel(DEFAULT_PAD_PORT),
            &[QUIET; 6],
        );

        assert_eq!(replayed.now(), recorded_at, "the same instant");
        assert_eq!(hash, recorded, "the same machine, bit for bit");
        assert_eq!(
            seen, recorded_seen,
            "the guest saw the same buttons on the same iteration"
        );
        assert_eq!(replay.cursor(), 2, "every recorded press was delivered");
    }

    #[test]
    fn a_board_whose_joypad_has_no_channel_refuses_to_build() {
        let recorder = Arc::new(Recorder::recording());
        let hosts = Arc::new(HostObjects::new());
        hosts.seal(Arc::clone(&recorder)).expect("an empty table");

        let mut options = BuildOptions::new()
            .with_classes(catalog::classes())
            .with_bindings(catalog::bindings().expect("this build's bindings"));
        options.realize.media.insert("firmware", rom());
        options.realize.hosts = hosts;
        let registry = catalog::registry().expect("this build's registry");
        let err = rsemu::machine::build("gb-pad.machine", BOARD, &registry, &options)
            .expect_err("the joypad has no channel");
        let text = format!("{err}");
        assert!(
            text.contains("pad:gb-joypad"),
            "the refusal names the input that bypassed the seam: {text}"
        );
        assert!(text.contains("replay"), "and says why it matters: {text}");
    }
}

// ---------------------------------------------------------------------------
// The Master System
// ---------------------------------------------------------------------------

#[cfg(all(feature = "cpu-z80", feature = "dev-sms"))]
mod mastersystem {
    use super::*;
    use rsemu::core::record::InputSink;
    use rsemu::dev::sms::io::{DEFAULT_PAD_PORT, pads};

    /// A Z80, a ROM at the reset vector, work RAM, and the I/O chip's pad
    /// aperture at `$DC`.
    const BOARD: &str = r#"
    machine "sms-pad" {
      osc clk = 3579545 Hz
      space mem  { width = 16, unassigned = open-bus }
      space port { width = 16, unassigned = open-bus }
      object cpu "cpu.z80" {
        clock        = clk
        space        = mem
        iospace      = "port"
        engine       = "interp"
        floating-bus = 0xff
      }
      object boot "rom" { size = 16K, image = "firmware" }
      object wram "ram" { size = 8K }
      object io "sms.io" { region = "export" }
      map mem  0x0000 size 16K   = boot
      map mem  0xc000 size 8K    = wram
      map port 0x00dc size 0x02  = io.pads
    }
    "#;

    /// The guest program, at the Z80's reset vector.
    ///
    /// ```text
    ///   0000: 01 dc 00     ld   bc, $00dc   ; IN A,(C) puts B on A8-A15, so the
    ///   0003: 21 02 c0     ld   hl, $c002   ; board's full decode is honoured
    ///   0006: 34      L:   inc  (hl)
    ///   0007: ed 78        in   a, (c)
    ///   0009: 2f           cpl              ; every pad line is active low
    ///   000a: e6 3f        and  $3f
    ///   000c: 28 f8        jr   z, L
    ///   000e: 32 00 c0     ld   ($c000), a  ; which buttons
    ///   0011: 7e           ld   a, (hl)
    ///   0012: 32 01 c0     ld   ($c001), a  ; and how long it took
    ///   0015: 18 fe        jr   $
    /// ```
    const PROGRAM: [u8; 23] = [
        0x01, 0xdc, 0x00, 0x21, 0x02, 0xc0, 0x34, 0xed, 0x78, 0x2f, 0xe6, 0x3f, 0x28, 0xf8, 0x32,
        0x00, 0xc0, 0x7e, 0x32, 0x01, 0xc0, 0x18, 0xfe,
    ];

    fn rom() -> Vec<u8> {
        let mut image = vec![0u8; 0x4000];
        image[..PROGRAM.len()].copy_from_slice(&PROGRAM);
        image
    }

    fn board(recorder: &Arc<Recorder>) -> Machine {
        let hosts = Arc::new(HostObjects::new());
        let port = pads::open(&hosts, DEFAULT_PAD_PORT).expect("a pad port before the build");
        recorder
            .register(pads::channel(DEFAULT_PAD_PORT), pads::sink(&port))
            .expect("a fresh recorder takes channels");
        let mut machine = build("sms-pad.machine", BOARD, rom(), &hosts);
        machine
            .set_recorder(Arc::clone(recorder))
            .expect("the board runs deterministically");
        machine.reset(ResetKind::Cold);
        machine
    }

    /// Up and button 2 on port A: bits 0 and 5 of the mask
    /// `SmsPads::set_buttons` takes. The second byte is port B and the third
    /// the console's own buttons, both untouched.
    const HELD: &[u8] = &[0b0010_0001, 0, 0];
    const RELEASED: &[u8] = &[0, 0, 0];

    fn script() -> Vec<&'static [u8]> {
        vec![QUIET, QUIET, HELD, QUIET, RELEASED, QUIET]
    }

    #[test]
    fn pressing_a_button_changes_where_the_run_ends_up() {
        let recorder = Arc::new(Recorder::recording());
        let mut quiet = board(&recorder);
        let (silent, silent_seen) = drive(
            &mut quiet,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &[QUIET; 6],
        );
        assert_eq!(silent_seen[0], 0, "nobody pressed anything");

        let recorder = Arc::new(Recorder::recording());
        let mut pressed = board(&recorder);
        let (held, held_seen) = drive(
            &mut pressed,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &script(),
        );
        assert_eq!(
            held_seen[0], 0b0010_0001,
            "the guest read Up and button 2 off $DC"
        );
        assert_ne!(silent, held, "a press is guest-visible");
        assert_eq!(recorder.log().len(), 2, "two posts, two logged events");
    }

    #[test]
    fn a_recorded_session_replays_to_the_same_state_hash() {
        let recorder = Arc::new(Recorder::recording());
        let mut machine = board(&recorder);
        let (recorded, recorded_seen) = drive(
            &mut machine,
            &recorder,
            &pads::channel(DEFAULT_PAD_PORT),
            &script(),
        );
        let recorded_at = machine.now();

        let bytes = recorder.log().encode().expect("a recording encodes");
        let log = InputLog::decode(&bytes).expect("and decodes");
        assert_eq!(log.events()[0].channel.to_string(), "pad:sms-pads");

        let replay = Arc::new(Recorder::replaying(log));
        let mut replayed = board(&replay);
        let (hash, seen) = drive(
            &mut replayed,
            &replay,
            &pads::channel(DEFAULT_PAD_PORT),
            &[QUIET; 6],
        );

        assert_eq!(replayed.now(), recorded_at, "the same instant");
        assert_eq!(hash, recorded, "the same machine, bit for bit");
        assert_eq!(
            seen, recorded_seen,
            "the guest saw the same buttons on the same iteration"
        );
        assert_eq!(replay.cursor(), 2, "every recorded press was delivered");
    }

    #[test]
    fn a_board_whose_pads_have_no_channel_refuses_to_build() {
        let recorder = Arc::new(Recorder::recording());
        let hosts = Arc::new(HostObjects::new());
        hosts.seal(Arc::clone(&recorder)).expect("an empty table");

        let mut options = BuildOptions::new()
            .with_classes(catalog::classes())
            .with_bindings(catalog::bindings().expect("this build's bindings"));
        options.realize.media.insert("firmware", rom());
        options.realize.hosts = hosts;
        let registry = catalog::registry().expect("this build's registry");
        let err = rsemu::machine::build("sms-pad.machine", BOARD, &registry, &options)
            .expect_err("the pads have no channel");
        let text = format!("{err}");
        assert!(
            text.contains("pad:sms-pads"),
            "the refusal names the input that bypassed the seam: {text}"
        );
    }

    /// The console's own buttons ride the same channel, and Pause is an edge
    /// the guest cannot miss: a payload with the bit set and one without is a
    /// press, exactly as a thumb produces.
    #[test]
    fn the_pause_button_travels_through_the_seam_too() {
        let recorder = Arc::new(Recorder::recording());
        let hosts = Arc::new(HostObjects::new());
        let port = pads::open(&hosts, DEFAULT_PAD_PORT).expect("a pad port");
        recorder
            .register(pads::channel(DEFAULT_PAD_PORT), pads::sink(&port))
            .expect("a fresh recorder takes channels");
        let sink: Arc<dyn InputSink> = pads::sink(&port);

        sink.deliver(&[0, 0, pads::PAUSE]);
        assert!(port.buttons(0) == 0, "no pad line moved");
        sink.deliver(&[0, 0, 0]);
        // What is asserted here is the encoding rather than the NMI, which
        // `tests/sms_board.rs` already takes on a real board: the third byte is
        // the console's buttons and nothing else touches the pads.
        assert_eq!(pads::RECORD_BYTES, 3);
        assert_eq!(pads::RESET, 0x02);
    }
}
