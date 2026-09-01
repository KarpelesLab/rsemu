//! Input reaching a guest, and doing it without breaking determinism.
//!
//! Two claims, and they are separate:
//!
//! 1. **A key pressed in a VNC client is executed by the guest.** A real 8086
//!    polls the 8042's status register, reads its data port and stores the scan
//!    code into RAM. Nothing here reaches into the controller: the key goes in
//!    as an X11 keysym at the top of the frontend's path — the same
//!    [`InputEvent`] a `KeyEvent` message (RFC 6143 §7.5.4) turns into — and
//!    comes out as a byte the guest's own `IN AL, 60h` fetched.
//!
//! 2. **A recorded session replays to an identical state hash.** The same
//!    machine is built twice: once with the events delivered live and recorded,
//!    once with the recording as the only input. The two runs' state hashes are
//!    compared.
//!
//! # What is and is not proved about determinism
//!
//! What is proved is the thing this frontend is responsible for: an input event
//! is applied at a virtual instant, that instant is recorded, and replaying at
//! the same instants produces a bit-identical machine. That is `CLAUDE.md`'s
//! rule for a non-deterministic input crossing into the machine.
//!
//! What is **not** proved here is a *whole-session* replay across a process
//! boundary, because the general record/replay seam does not exist yet: the
//! log this frontend writes is its own file, its cursor is not part of a
//! snapshot, and nothing else non-deterministic in the machine is captured
//! alongside it. `src/host/vnc/mod.rs` lists the five things the general seam
//! has to offer for this file's log to be deleted in its favour.

#![cfg(feature = "vnc")]

use rsemu::core::clock::GlobalTime;
use rsemu::host::input::{InputEvent, InputLog, Keysym};

// A build with `vnc` but no keyboard and no pad still runs the log's own
// round-trip test below, and must not carry imports for the guests it does not
// have — the feature sweep builds exactly that combination with warnings denied.
#[cfg(any(all(feature = "cpu-x86", feature = "dev-pc"), feature = "dev-nes-io"))]
use rsemu::host::input::InputSink;

/// How much virtual time each turn of the test's loop advances.
///
/// A millisecond: long enough for the 8042's own clock domain to tick many
/// times, short enough that the whole test is a few dozen turns.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

// ---------------------------------------------------------------------------
// a guest that reads the keyboard
// ---------------------------------------------------------------------------

/// An 8086, a megabyte of RAM, a boot ROM and an 8042.
///
/// The smallest board on which a *guest instruction* can observe a keystroke.
/// Inline rather than in `machines/`, because it models no product and exists
/// only for this file.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
const X86_KBD: &str = r#"
machine "x86-kbd" {
  osc xtal = 14318180 Hz

  space mem  { width = 20, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }

  object cpu "cpu.x86" {
    clock   = xtal / 3
    space   = mem
    iospace = "port"
    model   = "8086"
    engine  = "interp"
  }

  object dram "ram" { size = 0xf0000 }
  object boot "rom" { size = 64K, image = "firmware" }

  # The keyboard controller, on its own divided clock: a byte between the
  # keyboard and the host takes about a millisecond of serial time, and the
  # scheduler is the only thing that decides when that byte lands.
  object kbc "pc.kbc" { clock = xtal / 12000, port = "keyboard" }

  map mem  0x00000 size 0xf0000 = dram
  map mem  0xf0000 size 64K     = boot
  map port 0x0060  size 0x0001  = kbc.data
  map port 0x0064  size 0x0001  = kbc.cmd
}
"#;

/// Where the guest stores the scan code it read.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
const SENTINEL_ADDR: u64 = 0x0600;

/// The whole guest program, and it lives at the reset vector.
///
/// An 8086 resets to `CS:IP = F000:FFF0`, which is linear `0xFFFF0` — the last
/// sixteen bytes of the address space (Intel 8086 Family User's Manual, "Reset
/// and Initialization"). Thirteen of them are enough, so there is no boot jump
/// and no code in RAM:
///
/// ```text
///   E4 64        in    al, 64h      ; the 8042 status register
///   A8 01        test  al, 1        ; output buffer full?
///   74 FA        jz    -6           ; no: go round again
///   E4 60        in    al, 60h      ; yes: take the byte
///   A2 00 06     mov   [0600h], al  ; DS is 0 at reset, so this is linear
///   EB FE        jmp   $            ; and stop
/// ```
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
const PROGRAM: [u8; 13] = [
    0xe4, 0x64, 0xa8, 0x01, 0x74, 0xfa, 0xe4, 0x60, 0xa2, 0x00, 0x06, 0xeb, 0xfe,
];

/// A 64 KiB ROM image with [`PROGRAM`] at the reset vector.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
fn rom() -> Vec<u8> {
    let mut image = vec![0xffu8; 0x1_0000];
    image[0xfff0..0xfff0 + PROGRAM.len()].copy_from_slice(&PROGRAM);
    image
}

/// Build the board, and hand back the keyboard's character port.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
fn a_pc() -> (
    rsemu::machine::Machine,
    std::sync::Arc<rsemu::host::chardev::CharPort>,
) {
    let image = rom();
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    options.realize.media.insert("firmware", image.as_slice());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build("x86-kbd.machine", X86_KBD, &registry, &options)
        .expect("a board with a keyboard on it");
    let port = rsemu::host::chardev::ports::open(&options.realize.hosts, "keyboard")
        .expect("the 8042 opened its port");
    (machine, port)
}

/// One byte of the guest's memory, without side effects.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
fn peek(machine: &rsemu::machine::Machine, addr: u64) -> u8 {
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    machine
        .space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped byte") as u8
}

#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
#[test]
fn a_key_pressed_in_a_client_is_read_by_the_guest() {
    let (mut machine, port) = a_pc();
    let keyboard = rsemu::host::input::KeyboardSink::new(port);

    // Let the guest reach its polling loop first, so what is asserted is a
    // keystroke arriving at a running machine rather than one that happened to
    // be queued before it started.
    machine.run_until(SLICE).expect("the guest starts");
    assert_eq!(peek(&machine, SENTINEL_ADDR), 0, "nothing stored yet");

    // Exactly what `VncServer::poll` hands back for a KeyEvent with
    // `down-flag = 1` and `key = 0x61` (RFC 6143 §7.5.4).
    keyboard.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'a'),
        down: true,
    });

    let mut at = machine.now();
    for _ in 0..64 {
        at = at.saturating_add(SLICE);
        machine.run_until(at).expect("the guest runs");
        if peek(&machine, SENTINEL_ADDR) != 0 {
            break;
        }
    }

    // Set 2's `A` is 0x1C. The controller's command byte is clear out of reset,
    // so translation to set 1 is off and the guest sees the keyboard's own code
    // — which `src/dev/pc/kbc.rs` asserts separately and this test relies on.
    assert_eq!(
        peek(&machine, SENTINEL_ADDR),
        0x1c,
        "the guest executed IN AL, 60h and got the `a` key's make code"
    );
}

#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
#[test]
fn a_key_the_keyboard_does_not_have_reaches_no_one() {
    let (mut machine, port) = a_pc();
    let keyboard = rsemu::host::input::KeyboardSink::new(port);
    machine.run_until(SLICE).expect("the guest starts");
    // A keysym with no position on a 101-key board. Sending the guest a scan
    // code for it would be inventing a keypress.
    keyboard.deliver(InputEvent::Key {
        keysym: Keysym(0xfe03),
        down: true,
    });
    let mut at = machine.now();
    for _ in 0..8 {
        at = at.saturating_add(SLICE);
        machine.run_until(at).expect("the guest runs");
    }
    assert_eq!(peek(&machine, SENTINEL_ADDR), 0);
}

// ---------------------------------------------------------------------------
// a NES pad
// ---------------------------------------------------------------------------

/// The other shape of input device: level rather than edge.
///
/// A board with the two controller ports on it and nothing else, so the read is
/// the guest's own — through the address space, at `$4016`, with the strobe and
/// the shift register the console really has.
#[cfg(feature = "dev-nes-io")]
const NES_PADS: &str = r#"
machine "nes-pads" {
  osc master = 21477272 Hz
  space cpubus { width = 16, unassigned = open-bus }
  object ports "nes.ports" { clock = master / 12, pads = "nes-pads" }
  map cpubus 0x4016 size 0x0001 = ports.port1
  map cpubus 0x4017 size 0x0001 = ports.port2
}
"#;

#[cfg(feature = "dev-nes-io")]
#[test]
fn a_key_held_in_a_client_holds_a_button_on_a_controller() {
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::dev::nes::input::{Pad, buttons, pads};
    use rsemu::host::input::PadSink;
    use std::sync::Arc;

    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build("nes-pads.machine", NES_PADS, &registry, &options)
        .expect("a board with two controller ports");
    let pad: Arc<Pad> =
        pads::open(&options.realize.hosts, "nes-pads").expect("the ports opened their pad seam");
    let sink = PadSink::new(Arc::clone(&pad), 0);

    // What a VNC client sends when someone holds the right arrow and X.
    sink.deliver(InputEvent::Key {
        keysym: Keysym::RIGHT,
        down: true,
    });
    sink.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'x'),
        down: true,
    });
    assert_eq!(pad.get(0), buttons::RIGHT | buttons::A);

    // And the console reads it: strobe the latch through $4016, then shift the
    // eight bits out in the order the hardware sends them, A first.
    let bus = machine.space("cpubus").expect("the CPU bus");
    bus.write(0x4016, Width::U8, 1, MemAttrs::DEFAULT)
        .expect("the strobe is a mapped write");
    bus.write(0x4016, Width::U8, 0, MemAttrs::DEFAULT)
        .expect("and so is dropping it");
    let mut shifted = 0u8;
    for _ in 0..8 {
        let bit = bus
            .read(0x4016, Width::U8, MemAttrs::DEFAULT)
            .expect("a mapped read") as u8;
        shifted = (shifted << 1) | (bit & 1);
    }
    assert_eq!(
        shifted,
        buttons::RIGHT | buttons::A,
        "the guest's own read of $4016 sees what the client is holding"
    );

    // Releasing is the same path in reverse, and the pad is level: what the
    // next strobe latches is whatever is held at that moment.
    sink.deliver(InputEvent::Key {
        keysym: Keysym::RIGHT,
        down: false,
    });
    assert_eq!(pad.get(0), buttons::A);
}

// ---------------------------------------------------------------------------
// determinism
// ---------------------------------------------------------------------------

/// A recording, and the replay of it, reach the same state.
///
/// The machine is the PC above, because a guest that *executes* on what it was
/// typed is a much stronger subject than one that merely stores it: a divergence
/// of one scheduler tick changes which instruction the byte lands in the middle
/// of, and the state hash covers the CPU's registers as well as its RAM.
#[cfg(all(feature = "cpu-x86", feature = "dev-pc"))]
#[test]
fn a_recorded_session_replays_to_an_identical_state_hash() {
    // -- the recording ------------------------------------------------------
    //
    // The events stand in for what a client sent; what matters is that they are
    // delivered at a slice boundary and that the instant is recorded with them.
    let script = [
        (3u32, Keysym::from_ascii(b'a'), true),
        (5, Keysym::from_ascii(b'a'), false),
        (9, Keysym::SHIFT_L, true),
        (11, Keysym::from_ascii(b'b'), true),
    ];

    let (mut machine, port) = a_pc();
    let keyboard = rsemu::host::input::KeyboardSink::new(port);
    let mut log = InputLog::new();
    for turn in 0..24u32 {
        // Step 3 of the session loop: deliver at `machine.now()`, and record
        // the instant that was.
        for (at, keysym, down) in &script {
            if *at == turn {
                let event = InputEvent::Key {
                    keysym: *keysym,
                    down: *down,
                };
                log.push(machine.now(), event);
                keyboard.deliver(event);
            }
        }
        let deadline = machine.now().saturating_add(SLICE);
        machine.run_until(deadline).expect("the guest runs");
    }
    let recorded = machine.state_hash().expect("a hash of the recorded run");
    assert_eq!(log.len(), script.len());
    assert!(log.is_ordered());

    // The log survives being written out and read back, which is what a bug
    // report attaching one relies on.
    let bytes = log.encode();
    let log = InputLog::decode(&bytes).expect("our own bytes");

    // -- the replay ---------------------------------------------------------
    let (mut replayed, port) = a_pc();
    let keyboard = rsemu::host::input::KeyboardSink::new(port);
    let mut replay = rsemu::host::input::Replay::new(log);
    for _ in 0..24 {
        for entry in replay.due(replayed.now()) {
            keyboard.deliver(entry.event);
        }
        let deadline = replayed.now().saturating_add(SLICE);
        replayed.run_until(deadline).expect("the guest runs");
    }
    assert!(replay.is_finished(), "every event was replayed");

    assert_eq!(
        replayed.state_hash().expect("a hash of the replayed run"),
        recorded,
        "a replayed session is the same machine, bit for bit"
    );

    // And the claim is not vacuous: a run given *no* input reaches a different
    // state, so the hash really is sensitive to what was typed.
    let (mut silent, _port) = a_pc();
    for _ in 0..24 {
        let deadline = silent.now().saturating_add(SLICE);
        silent.run_until(deadline).expect("the guest runs");
    }
    assert_ne!(
        silent.state_hash().expect("a hash of the silent run"),
        recorded,
        "a machine nobody typed at is not the same machine"
    );
}

/// An event replayed a fraction of a nanosecond late is a different run, so the
/// log stores `GlobalTime`'s raw units rather than a rounded nanosecond count.
#[test]
fn a_logs_instants_survive_a_round_trip_exactly() {
    // A raw instant that is deliberately not a whole number of nanoseconds:
    // `GlobalTime` counts 2⁻⁶⁴-second units, and a machine's `now()` lands on
    // one of them, not on a nanosecond boundary.
    let odd = GlobalTime::from_raw(12_345_678_901_234_567_890);
    assert_ne!(
        GlobalTime::from_nanos(odd.as_nanos()),
        odd,
        "the instant this test is about is one nanoseconds cannot hold"
    );
    let mut log = InputLog::new();
    log.push(
        odd,
        InputEvent::Key {
            keysym: Keysym::RETURN,
            down: true,
        },
    );
    let back = InputLog::decode(&log.encode()).expect("our own bytes");
    assert_eq!(back.events()[0].at, odd);
}
