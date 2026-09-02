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
//! # What is proved about determinism, and by what
//!
//! The frontend no longer keeps a log of its own: an event is posted to the
//! machine's [`Recorder`](rsemu::core::record::Recorder) and the *machine*
//! decides which scheduling-round boundary it lands on. So what is asserted
//! below is a whole-session recording in the general format — the one
//! `rsemu replay` reads, carrying the machine's shape, that a bug report
//! attaches — rather than a private file with a private cursor.
//!
//! Three assertions, and the first is what makes the other two mean anything:
//! a run nobody typed at reaches a *different* state hash, the replay reaches
//! the *same* one, and the guest executed on the bytes either way.

#![cfg(feature = "vnc")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::record::InputLog;
use rsemu::host::input::{self, Feed, InputEvent, Keysym};

// A build with `vnc` but no keyboard and no pad still runs the payload tests
// below, and must not carry imports for the guests it does not have — the
// feature sweep builds exactly that combination with warnings denied.
#[cfg(any(
    all(feature = "cpu-x86", feature = "dev-pc"),
    feature = "dev-nes-io",
    all(feature = "cpu-sm83", feature = "dev-gb"),
    all(feature = "cpu-z80", feature = "dev-sms")
))]
use rsemu::host::input::InputSink;
use std::sync::Arc;

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
    use rsemu::host::input::{PadSink, Pads};
    use std::sync::Arc;

    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build("nes-pads.machine", NES_PADS, &registry, &options)
        .expect("a board with two controller ports");
    let pad: Arc<Pad> =
        pads::open(&options.realize.hosts, "nes-pads").expect("the ports opened their pad seam");
    let sink = PadSink::new(Pads::Nes(Arc::clone(&pad)), 0);

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
// the other two consoles
// ---------------------------------------------------------------------------

/// Advance `machine` a slice at a time until the guest's sentinel reads `want`,
/// and return whatever it says at the end.
///
/// A bounded loop rather than one slice, for the same reason the 8086 above gets
/// sixty-four turns: the press lands at the next scheduling-round boundary and
/// the guest reaches its next poll a few instructions after that, so "one slice"
/// would be asserting a coincidence about the quantum rather than about input.
#[cfg(any(
    all(feature = "cpu-sm83", feature = "dev-gb"),
    all(feature = "cpu-z80", feature = "dev-sms")
))]
fn poll_until(
    machine: &mut rsemu::machine::Machine,
    step: GlobalTime,
    peek: &dyn Fn(&rsemu::machine::Machine) -> u8,
    want: u8,
) -> u8 {
    for _ in 0..64 {
        let deadline = machine.now().saturating_add(step);
        machine.run_until(deadline).expect("the guest runs");
        if peek(machine) == want {
            break;
        }
    }
    peek(machine)
}

/// The defect these two tests exist for.
///
/// A Game Boy and a Master System file their controller port under the *same*
/// host kind a NES does — `pad` — so the frontend loop that listed that kind
/// and then asked for a NES-typed pad got their names and failed the downcast
/// in silence. A picture, and no buttons. So what is exercised here is
/// [`PadSink::open`], which is the discovery that replaced it: the sink is
/// found the way `rsemu --vnc` finds it, and nothing in the test names a
/// concrete pad type.
///
/// And what asserts it worked is the **guest**, polling its own port in a loop,
/// exactly as the 8086 above executes `IN AL, 60h`. A pad the host can read
/// back is not evidence; a guest instruction that saw the press is.
#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
const GB_PAD: &str = r#"
machine "gb-pad" {
  osc master = 4194304 Hz
  space cpubus { width = 16, unassigned = read-as-ones }
  object cpu "cpu.sm83" { clock = master / 4, space = cpubus, post-boot = true }
  object cart "rom" { size = 32K, image = "cart" }
  object wram "ram" { size = 8K }
  object pad "gb.joypad" { pad = "gb-joypad" }
  map cpubus 0x0000 size 32K     = cart
  map cpubus 0xc000 size 8K      = wram
  map cpubus 0xff00 size 0x0001  = pad
}
"#;

/// The guest program, at `$0100` — where an SM83 with `post-boot = true` starts.
///
/// ```text
///   3E 10        ld   a, $10      ; P14 high, P15 low: select the action row
///   E0 00        ldh  ($00), a    ; $FF00
///   F0 00        ldh  a, ($00)    ; and read it straight back
///   E6 0F        and  $0f         ; the four button lines, active low
///   EA 00 C0     ld   ($C000), a  ; where the test looks
///   18 F3        jr   -13         ; round again, forever
/// ```
///
/// A loop rather than a one-shot: a pad is a *level*, so the interesting claim
/// is that the guest's next poll sees the press — which means it has to keep
/// polling. Pan Docs, "Joypad Input", for the register.
#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
const GB_PROGRAM: [u8; 13] = [
    0x3e, 0x10, 0xe0, 0x00, 0xf0, 0x00, 0xe6, 0x0f, 0xea, 0x00, 0xc0, 0x18, 0xf3,
];

/// The board, and the host-object table it was built against.
///
/// The table is what a frontend is handed after the build; every pad in this
/// file is found in it by kind and name, never by reaching into the machine.
#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
fn a_game_boy() -> (
    rsemu::machine::Machine,
    std::sync::Arc<rsemu::core::hosts::HostObjects>,
) {
    let mut cart = vec![0xffu8; 0x8000];
    cart[0x0100..0x0100 + GB_PROGRAM.len()].copy_from_slice(&GB_PROGRAM);

    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    options.realize.media.insert("cart", cart.as_slice());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build("gb-pad.machine", GB_PAD, &registry, &options)
        .expect("a board with a joypad on it");
    let hosts = Arc::clone(&options.realize.hosts);
    (machine, hosts)
}

/// The low nibble of `$FF00` as the guest last stored it.
#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
fn gb_peek(machine: &rsemu::machine::Machine) -> u8 {
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    machine
        .space("cpubus")
        .expect("the CPU bus")
        .read(0xc000, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped byte") as u8
}

#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
#[test]
fn a_key_held_in_a_client_is_polled_by_a_game_boy() {
    use rsemu::host::input::PadSink;

    let (mut machine, hosts) = a_game_boy();

    // Exactly what `rsemu --vnc` does, and the whole point: no console-specific
    // type is named here.
    let sink = PadSink::open(&hosts, 0).expect("the joypad opened its pad port");

    let peek = gb_peek;
    let step = GlobalTime::from_nanos(1_000_000);
    machine.run_until(step).expect("the guest starts");
    assert_eq!(peek(&machine), 0x0f, "four lines high: nobody is pressing");

    // What a VNC client sends when someone holds X — the A button, by the host
    // layout every emulator has used since the 1990s.
    sink.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'x'),
        down: true,
    });
    assert_eq!(
        poll_until(&mut machine, step, &peek, 0x0e),
        0x0e,
        "the guest's own read of $FF00 pulled A low"
    );

    // Level, not edge: releasing is visible at the guest's next poll and
    // nowhere else.
    sink.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'x'),
        down: false,
    });
    assert_eq!(poll_until(&mut machine, step, &peek, 0x0f), 0x0f);
}

/// The Master System's controller ports, on the Z80's separate I/O space.
#[cfg(all(feature = "cpu-z80", feature = "dev-sms"))]
const SMS_PAD: &str = r#"
machine "sms-pad" {
  osc master = 10738635 Hz
  space mem  { width = 16, unassigned = open-bus }
  space port { width = 16, unassigned = open-bus }
  object cpu "cpu.z80" { clock = master / 3, space = mem, iospace = "port" }
  object cart "rom" { size = 16K, image = "cart" }
  object wram "ram" { size = 8K }
  object io "sms.io" { region = "export", pads = "sms-pads" }
  map mem  0x0000 size 16K     = cart
  map mem  0xc000 size 8K      = wram
  map port 0x00dc size 0x0002  = io.pads
}
"#;

/// The guest program, at the Z80's reset vector.
///
/// ```text
///   01 DC 00     ld   bc, $00DC   ; B is A8-A15 on an `IN r,(C)`
///   ED 78        in   a, (c)      ; so the port address is exactly $00DC
///   32 00 C0     ld   ($C000), a
///   18 F6        jr   -10         ; round again, forever
/// ```
///
/// `IN A,(n)` would have put the accumulator on the high address byte, which is
/// why the real console mirrors its ports across all 256 pages; `IN r,(C)` lets
/// this board map two bytes and mean it.
#[cfg(all(feature = "cpu-z80", feature = "dev-sms"))]
const SMS_PROGRAM: [u8; 10] = [0x01, 0xdc, 0x00, 0xed, 0x78, 0x32, 0x00, 0xc0, 0x18, 0xf6];

#[cfg(all(feature = "cpu-z80", feature = "dev-sms"))]
#[test]
fn a_key_held_in_a_client_is_polled_by_a_master_system() {
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::host::input::PadSink;

    let mut cart = vec![0xffu8; 0x4000];
    cart[..SMS_PROGRAM.len()].copy_from_slice(&SMS_PROGRAM);

    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    options.realize.media.insert("cart", cart.as_slice());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = rsemu::machine::build("sms-pad.machine", SMS_PAD, &registry, &options)
        .expect("a board with two controller ports");

    let sink = PadSink::open(&options.realize.hosts, 0).expect("the I/O chip opened its pad port");

    let peek = |m: &rsemu::machine::Machine| {
        m.space("mem")
            .expect("the memory space")
            .read(0xc000, Width::U8, MemAttrs::DEBUG)
            .expect("a mapped byte") as u8
    };

    let step = GlobalTime::from_nanos(1_000_000);
    machine.run_until(step).expect("the guest starts");
    assert_eq!(
        peek(&machine),
        0xff,
        "every line pulled up: nobody is pressing"
    );

    sink.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'x'),
        down: true,
    });
    assert_eq!(
        poll_until(&mut machine, step, &peek, 0xef),
        0xef,
        "the guest's own IN A,(C) saw TL low — the console's button 1"
    );

    // Start is not on this pad at all: it is the Pause switch on the console,
    // driving /NMI. So it must not appear at $DC — and the button that is held
    // must still be the only line down.
    sink.deliver(InputEvent::Key {
        keysym: Keysym::RETURN,
        down: true,
    });
    assert_eq!(
        poll_until(&mut machine, step, &peek, 0xef),
        0xef,
        "and nothing else moved"
    );
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
mod determinism {
    use super::*;
    use rsemu::core::record::Recorder;

    /// A PC whose keyboard is the far end of the `input:vnc` channel.
    ///
    /// The whole wiring a frontend does: a `Feed` holding the sinks, registered
    /// with the recorder under the channel's name, and the recorder attached to
    /// the machine.
    fn a_recorded_pc(recorder: &Arc<Recorder>) -> rsemu::machine::Machine {
        let (mut machine, port) = a_pc();
        let feed = Arc::new(Feed::new());
        feed.attach(Arc::new(rsemu::host::input::KeyboardSink::new(port)));
        recorder
            .register(input::channel(input::DEFAULT_STREAM), input::sink(&feed))
            .expect("a fresh recorder takes channels");
        machine
            .set_recorder(Arc::clone(recorder))
            .expect("a deterministic machine records");
        machine
    }

    /// What a client sent, as `(turn, keysym, down)`.
    ///
    /// Two of them land on the same turn on purpose: two keys in one poll is
    /// the common case, and they are one payload — which is the tie-break at
    /// equal instants that the seam has to have.
    const SCRIPT: [(u32, Keysym, bool); 4] = [
        (3, Keysym::from_ascii(b'a'), true),
        (5, Keysym::from_ascii(b'a'), false),
        (9, Keysym::SHIFT_L, true),
        (9, Keysym::from_ascii(b'b'), true),
    ];

    /// Run 24 turns, posting the script's events before the turn they belong
    /// to. Returns the final state hash.
    fn drive(machine: &mut rsemu::machine::Machine, recorder: &Recorder, typing: bool) -> u64 {
        let channel = input::channel(input::DEFAULT_STREAM);
        for turn in 0..24u32 {
            if typing {
                // Everything for this turn in one payload, exactly as one
                // `VncServer::poll` hands back a batch.
                let mut payload = Vec::new();
                for (at, keysym, down) in &SCRIPT {
                    if *at == turn {
                        payload.extend_from_slice(
                            &InputEvent::Key {
                                keysym: *keysym,
                                down: *down,
                            }
                            .encode(),
                        );
                    }
                }
                if !payload.is_empty() {
                    recorder
                        .post(&channel, &payload)
                        .expect("a registered channel");
                }
            }
            let deadline = machine.now().saturating_add(SLICE);
            machine.run_until(deadline).expect("the guest runs");
        }
        machine.state_hash().expect("a hash of a deterministic run")
    }

    #[test]
    fn typing_at_the_machine_changes_where_it_ends_up() {
        // The control. Without it, "record and replay agree" would be satisfied
        // by a seam that dropped every keystroke on the floor.
        let quiet_recorder = Arc::new(Recorder::recording());
        let mut quiet = a_recorded_pc(&quiet_recorder);
        let silent = drive(&mut quiet, &quiet_recorder, false);
        assert_eq!(peek(&quiet, SENTINEL_ADDR), 0, "nobody typed");

        let recorder = Arc::new(Recorder::recording());
        let mut machine = a_recorded_pc(&recorder);
        let typed = drive(&mut machine, &recorder, true);
        assert_eq!(
            peek(&machine, SENTINEL_ADDR),
            0x1c,
            "the guest executed IN AL, 60h on the `a` key's make code"
        );
        assert_ne!(
            silent, typed,
            "a machine nobody typed at is not the same machine"
        );
    }

    #[test]
    fn a_recorded_session_replays_to_an_identical_state_hash() {
        let recorder = Arc::new(Recorder::recording());
        let mut machine = a_recorded_pc(&recorder);
        let recorded = drive(&mut machine, &recorder, true);
        let recorded_at = machine.now();

        // Three posts, because two of the four events shared a turn.
        let log = recorder.log();
        assert_eq!(log.len(), 3);
        assert_eq!(log.events()[0].channel.to_string(), "input:vnc");
        assert!(
            log.events().windows(2).all(|w| w[0].at <= w[1].at),
            "a recording is delivery-ordered"
        );

        // Out through the file format and back: this is what a bug report
        // attaches, and it carries the machine's shape.
        let bytes = log.encode().expect("a recording encodes");
        let log = InputLog::decode(&bytes).expect("our own bytes");

        let replay = Arc::new(Recorder::replaying(log));
        let mut replayed = a_recorded_pc(&replay);
        let hash = drive(&mut replayed, &replay, false);

        assert_eq!(replayed.now(), recorded_at, "the same instant");
        assert_eq!(
            hash, recorded,
            "a replayed session is the same machine, bit for bit"
        );
        assert_eq!(
            peek(&replayed, SENTINEL_ADDR),
            0x1c,
            "and the guest executed on the replayed bytes"
        );
        assert_eq!(replay.cursor(), 3, "every recorded payload was delivered");
    }
}

/// The same two claims, for a console's pad rather than a PC's keyboard.
///
/// `tests/joypad_replay.rs` already proves a Game Boy replays bit for bit
/// through **its own** channel, `pad:gb-joypad`. This is the other door: a VNC
/// client's keysyms on `input:vnc`, translated to the console's matrix by
/// [`PadSink`](rsemu::host::input::PadSink) at delivery time. A translation that
/// happened at *post* time instead would record the wrong thing and replay
/// differently, and this is what would notice.
#[cfg(all(feature = "cpu-sm83", feature = "dev-gb"))]
mod console_determinism {
    use super::*;
    use rsemu::core::record::Recorder;
    use rsemu::host::input::PadSink;

    /// A millisecond a turn, twenty-four turns — the same cadence the PC above
    /// is driven at.
    const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

    /// A Game Boy whose joypad is the far end of the `input:vnc` channel.
    fn a_recorded_game_boy(recorder: &Arc<Recorder>) -> rsemu::machine::Machine {
        let (mut machine, hosts) = a_game_boy();
        let feed = Arc::new(Feed::new());
        feed.attach(Arc::new(
            PadSink::open(&hosts, 0).expect("the joypad opened its pad port"),
        ));
        recorder
            .register(input::channel(input::DEFAULT_STREAM), input::sink(&feed))
            .expect("a fresh recorder takes channels");
        machine
            .set_recorder(Arc::clone(recorder))
            .expect("a deterministic machine records");
        machine
    }

    /// What a client sent, as `(turn, keysym, down)`.
    const SCRIPT: [(u32, Keysym, bool); 4] = [
        (3, Keysym::from_ascii(b'x'), true),
        (5, Keysym::LEFT, true),
        (9, Keysym::from_ascii(b'x'), false),
        (9, Keysym::from_ascii(b'z'), true),
    ];

    fn drive(machine: &mut rsemu::machine::Machine, recorder: &Recorder, pressing: bool) -> u64 {
        let channel = input::channel(input::DEFAULT_STREAM);
        for turn in 0..24u32 {
            if pressing {
                let mut payload = Vec::new();
                for (at, keysym, down) in &SCRIPT {
                    if *at == turn {
                        payload.extend_from_slice(
                            &InputEvent::Key {
                                keysym: *keysym,
                                down: *down,
                            }
                            .encode(),
                        );
                    }
                }
                if !payload.is_empty() {
                    recorder
                        .post(&channel, &payload)
                        .expect("a registered channel");
                }
            }
            let deadline = machine.now().saturating_add(SLICE);
            machine.run_until(deadline).expect("the guest runs");
        }
        machine.state_hash().expect("a hash of a deterministic run")
    }

    #[test]
    fn pressing_a_button_changes_where_a_game_boy_ends_up() {
        let quiet_recorder = Arc::new(Recorder::recording());
        let mut quiet = a_recorded_game_boy(&quiet_recorder);
        let silent = drive(&mut quiet, &quiet_recorder, false);
        assert_eq!(gb_peek(&quiet), 0x0f, "nobody pressed anything");

        let recorder = Arc::new(Recorder::recording());
        let mut machine = a_recorded_game_boy(&recorder);
        let pressed = drive(&mut machine, &recorder, true);
        // A and Left were pressed, A released, B pressed. The action row is the
        // one selected, so what the guest last stored is B down and A up.
        assert_eq!(gb_peek(&machine), 0x0d, "the guest polled B down");
        assert_ne!(silent, pressed);
    }

    #[test]
    fn a_recorded_console_session_replays_to_an_identical_state_hash() {
        let recorder = Arc::new(Recorder::recording());
        let mut machine = a_recorded_game_boy(&recorder);
        let recorded = drive(&mut machine, &recorder, true);
        let recorded_at = machine.now();

        let bytes = recorder.log().encode().expect("a recording encodes");
        let log = InputLog::decode(&bytes).expect("our own bytes");
        assert_eq!(log.len(), 3, "two of the four events shared a turn");
        assert_eq!(log.events()[0].channel.to_string(), "input:vnc");

        let replay = Arc::new(Recorder::replaying(log));
        let mut replayed = a_recorded_game_boy(&replay);
        let hash = drive(&mut replayed, &replay, false);

        assert_eq!(replayed.now(), recorded_at, "the same instant");
        assert_eq!(hash, recorded, "the same machine, bit for bit");
        assert_eq!(gb_peek(&replayed), 0x0d, "and the guest polled the replay");
        assert_eq!(replay.cursor(), 3, "every recorded payload was delivered");
    }
}

/// An event replayed a fraction of a nanosecond late is a different run, so the
/// recording stores `GlobalTime`'s raw units rather than a rounded nanosecond
/// count — end to end, through the encoder the VNC path actually uses.
#[test]
fn a_recordings_instants_survive_a_round_trip_exactly() {
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
    log.push(rsemu::core::record::InputEvent {
        at: odd,
        channel: input::channel(input::DEFAULT_STREAM),
        payload: InputEvent::Key {
            keysym: Keysym::RETURN,
            down: true,
        }
        .encode()
        .to_vec(),
    })
    .expect("an empty log takes anything");
    let back = InputLog::decode(&log.encode().expect("encodes")).expect("our own bytes");
    assert_eq!(back.events()[0].at, odd);
    assert_eq!(back.events()[0].at.raw(), odd.raw());
}

/// The payload the channel carries is this module's business and nobody
/// else's: twelve bytes an event at a time, so a batch is an array of them.
#[test]
fn a_batch_of_events_is_one_payload() {
    let feed = Arc::new(Feed::new());
    let mut payload = Vec::new();
    for down in [true, false] {
        payload.extend_from_slice(
            &InputEvent::Key {
                keysym: Keysym::from_ascii(b'q'),
                down,
            }
            .encode(),
        );
    }
    assert_eq!(payload.len(), 2 * rsemu::host::input::EVENT_BYTES);
    // Nothing attached: the feed is where the seam puts a payload, and a
    // machine with no keyboard must not be a panic.
    rsemu::core::record::InputSink::deliver(&*feed, &payload);
    assert!(feed.is_empty());
}
