//! An RF reader session, through the record/replay seam and nowhere else
//! (`ROADMAP.md` §4.5; `CLAUDE.md`, determinism).
//!
//! `tests/keypad_replay.rs` makes this argument for a keypad and this file
//! makes it for the ST25DV's **reader door**, in the same three claims and the
//! same order:
//!
//! 1. **Tapping the tag changes the run.** A board reaches a different state
//!    when a reader wrote to it. Without this, "record and replay agree" would
//!    be satisfied by a door that dropped every command.
//! 2. **A recording replays bit for bit.** It goes out through the file format
//!    and comes back, and a fresh machine driven only by it reaches the same
//!    state hash at the same instant — *and* produces the same RF responses,
//!    which is the half a keypad has no analogue of. A response is machine
//!    output, not host input, so it is not in the log; it comes back only
//!    because the machine it is derived from came back.
//! 3. **A board whose reader has no channel refuses to build.** The seal, on
//!    the host object the device opens by name.
//!
//! # The board
//!
//! Inline, and deliberately not a product. There is **no processor**: an I²C
//! bus is a host object either end of the build may hold, so this file plays
//! the bus master itself and what is asserted stays the seam rather than some
//! firmware's idea of a driver. `machines/stm32f407.machine` carries the same
//! tag on a board that does have a core.
//!
//! A `wire.level-to-edge` watches `GPO`, so the pin's level is architectural
//! state that lands in the snapshot the hash is taken over — which is what
//! makes claim 1 about the *conductor* rather than about a buffer the tag
//! happens to save.

#![cfg(feature = "dev-st25dv")]

use std::sync::Arc;

use rsemu::bus::i2c::{Ack, Address, Direction, I2cBus, buses};
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::HostObjects;
use rsemu::core::record::{Channel, InputLog, Recorder};
use rsemu::core::wire::Level;
use rsemu::dev::st25dv::rf;
use rsemu::machine::{BuildOptions, Machine, catalog};

/// An ST25DV04K on a bus this file drives, with its `GPO` watched.
const BOARD: &str = r#"
machine "st25dv" {
  osc clk = 1000000 Hz
  space mem { width = 16, unassigned = read-as-ones }
  object wram "ram" { size = 1K }
  object tag "st.st25dv" {
    clock   = clk
    density = "4K"
    gpo     = "open-drain"
    bus     = "nfc-bus"
    reader  = "nfc"
  }
  object watch "wire.level-to-edge" { edge = "both" }
  map mem 0x0000 size 1K = wram
  wire tag.gpo -> watch.in { pull = "up" }
}
"#;

/// The reader door's name, which is also the channel's.
const READER: &str = "nfc";

/// The I²C bus's name, a rendezvous rather than a door: nothing
/// non-deterministic crosses at it, so a sealed table passes it.
const BUS: &str = "nfc-bus";

/// The tag's user-memory address, `1010 0 1 1` (DS10925 Table 88, `E2 = 0`).
const USER: u8 = 0x53;

/// How long each slice of a run is.
const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

/// A slice with nothing posted, for the silent control and for a replay.
const QUIET: &[u8] = b"";

/// Build the board against a host-object table the caller keeps.
fn build(hosts: &Arc<HostObjects>) -> Machine {
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.hosts = Arc::clone(hosts);
    let registry = catalog::registry().expect("this build's registry");
    match rsemu::machine::build("st25dv.machine", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the ST25DV board does not realize: {e}"),
    }
}

/// The board, with its reader registered as a record/replay channel.
///
/// Three lines, and they are the whole conversion: the reader is a host object
/// opened by name, `rf::channel` is that name as a channel, and `rf::sink` is
/// what a payload does once the machine has decided when.
fn board(recorder: &Arc<Recorder>) -> (Machine, Arc<rf::Reader>, Arc<I2cBus>) {
    let hosts = Arc::new(HostObjects::new());
    let reader = rf::open(&hosts, READER).expect("a reader before the build");
    recorder
        .register(rf::channel(READER), rf::sink(&reader))
        .expect("a fresh recorder takes channels");
    let bus = buses::open(&hosts, BUS).expect("a bus before the build");
    let mut machine = build(&hosts);
    machine
        .set_recorder(Arc::clone(recorder))
        .expect("the board runs deterministically");
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, reader, bus)
}

/// What `GPO` is actually sitting at: the net the tag's `gpo` pin drives,
/// asked for its own resolution.
fn gpo(machine: &Machine) -> Level {
    let net = machine
        .nets()
        .iter()
        .find(|net| {
            net.sources().iter().any(|pin| {
                pin.port == "gpo" && machine.devices()[pin.device].path().ends_with("tag")
            })
        })
        .expect("the tag drives GPO");
    net.wire().resolve_net()
}

/// A sequential write over I²C: START, device select, the 16-bit address, the
/// data, STOP (DS10925 §6.4).
fn i2c_write(bus: &I2cBus, at: u16, data: &[u8]) {
    assert_eq!(
        bus.start(Address::Seven(USER), Direction::Write),
        Ack::Ack,
        "the tag did not answer its own address"
    );
    assert_eq!(bus.write((at >> 8) as u8), Ack::Ack);
    assert_eq!(bus.write(at as u8), Ack::Ack);
    for byte in data {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    bus.stop();
}

/// A random read (§6.5.1).
fn i2c_read(bus: &I2cBus, at: u16, count: usize) -> Vec<u8> {
    assert_eq!(bus.start(Address::Seven(USER), Direction::Write), Ack::Ack);
    assert_eq!(bus.write((at >> 8) as u8), Ack::Ack);
    assert_eq!(bus.write(at as u8), Ack::Ack);
    assert_eq!(bus.start(Address::Seven(USER), Direction::Read), Ack::Ack);
    let mut out = Vec::new();
    for i in 0..count {
        let last = i + 1 == count;
        out.push(bus.read(if last { Ack::Nack } else { Ack::Ack }));
    }
    bus.stop();
    out
}

/// The reader session this file records: a tap, a pair of blocks written from
/// RF, and the field going away again.
fn script() -> Vec<Vec<u8>> {
    vec![
        QUIET.to_vec(),
        rf::Command::encode_all(&[rf::Command::Field(true), rf::Command::Inventory]),
        QUIET.to_vec(),
        rf::Command::encode_all(&[rf::Command::WriteBlocks {
            block: 4,
            data: &[0xde, 0xad, 0xbe, 0xef],
        }]),
        QUIET.to_vec(),
        rf::Command::encode_all(&[
            rf::Command::ReadBlocks { block: 4, count: 1 },
            rf::Command::Field(false),
        ]),
        QUIET.to_vec(),
    ]
}

/// Run `machine` in slices, posting `session[i]` before slice `i`.
///
/// Posting before a slice rather than at some wall-clock moment is what keeps
/// the *test* deterministic, and it changes nothing about what is proved: the
/// recorder does not know when the host called it, only which round boundary
/// the machine drained it on.
fn drive(
    machine: &mut Machine,
    recorder: &Recorder,
    channel: &Channel,
    reader: &rf::Reader,
    session: &[Vec<u8>],
) -> (u64, Vec<Level>, Vec<Vec<u8>>) {
    let mut seen = Vec::with_capacity(session.len());
    let mut replies = Vec::new();
    for payload in session {
        if !payload.is_empty() {
            recorder
                .post(channel, payload)
                .expect("a registered channel");
        }
        machine.run_for(SLICE).expect("a deterministic run");
        seen.push(gpo(machine));
        replies.extend(reader.drain());
    }
    (
        machine.state_hash().expect("deterministic mode hashes"),
        seen,
        replies,
    )
}

#[test]
fn a_reader_writing_blocks_changes_where_the_run_ends_up() {
    let quiet = Arc::new(Recorder::recording());
    let (mut machine, reader, bus) = board(&quiet);
    // The I²C host puts something in user memory first, so both runs start from
    // the same non-trivial place.
    i2c_write(&bus, 0x0000, &[1, 2, 3, 4]);
    machine.run_for(SLICE).expect("the write cycle");
    let (silent, silent_gpo, silent_replies) = drive(
        &mut machine,
        &quiet,
        &rf::channel(READER),
        &reader,
        &vec![QUIET.to_vec(); script().len()],
    );
    assert!(
        silent_gpo.iter().all(|l| l.is_high()),
        "nobody tapped it, so the pull-up had GPO the whole time"
    );
    assert!(silent_replies.is_empty(), "and the tag said nothing");

    let loud = Arc::new(Recorder::recording());
    let (mut machine, reader, bus) = board(&loud);
    i2c_write(&bus, 0x0000, &[1, 2, 3, 4]);
    machine.run_for(SLICE).expect("the write cycle");
    let (tapped, tapped_gpo, replies) = drive(
        &mut machine,
        &loud,
        &rf::channel(READER),
        &reader,
        &script(),
    );

    // The factory GPO has FIELD_CHANGE_EN and GPO_EN set (Table 26), so the
    // carrier arriving pulls the open-drain pin down for IT_TIME. IT_TIME = 3
    // out of the factory, which Eq. (1) makes 188 µs — well inside one slice,
    // so the level is back up by the time the slice ends. The *state* it left
    // behind is what differs.
    assert_ne!(
        silent, tapped,
        "the reader wrote a block and raised interrupt status; the silent run did neither"
    );
    assert_eq!(
        tapped_gpo, silent_gpo,
        "a pulse is shorter than a slice, so the sampled level is not what changed"
    );

    // Four responses: the inventory, the block write, the block read, and
    // nothing at all for either field event.
    assert_eq!(replies.len(), 3, "{replies:?}");
    assert_eq!(replies[0][0], 0x00, "the inventory succeeded");
    assert_eq!(replies[0].len(), 10, "DSFID and eight UID bytes");
    assert_eq!(replies[1], vec![0x00]);
    assert_eq!(replies[2], vec![0x00, 0xde, 0xad, 0xbe, 0xef]);

    // And the I²C side sees what the reader wrote, which is the dual interface
    // doing the one thing it exists for — once tW has elapsed. An RF block
    // write costs the same internal write cycle an I²C one does, and until it
    // ends the part NACKs its own address on the wire (§6.4.3), so a host that
    // asked now would be told to wait.
    assert_eq!(
        bus.start(Address::Seven(USER), Direction::Write),
        Ack::Nack,
        "the RF write cycle is still running"
    );
    bus.stop();
    for _ in 0..5 {
        machine.run_for(SLICE).expect("tW is 5 ms at 1 MHz");
    }
    assert_eq!(i2c_read(&bus, 0x0010, 4), vec![0xde, 0xad, 0xbe, 0xef]);
    assert_eq!(loud.log().len(), 3, "three posts, three logged events");
}

#[test]
fn a_recorded_session_replays_to_the_same_state_hash_and_the_same_responses() {
    let recorder = Arc::new(Recorder::recording());
    let (mut machine, reader, bus) = board(&recorder);
    i2c_write(&bus, 0x0000, &[1, 2, 3, 4]);
    machine.run_for(SLICE).expect("the write cycle");
    let (recorded, recorded_gpo, recorded_replies) = drive(
        &mut machine,
        &recorder,
        &rf::channel(READER),
        &reader,
        &script(),
    );
    let recorded_at = machine.now();

    // Out through the file format and back, so what is replayed is what a
    // `.trace` on disk would hold rather than a live object.
    let bytes = recorder.log().encode().expect("a recording encodes");
    let log = InputLog::decode(&bytes).expect("and decodes");
    assert_eq!(log.events()[0].channel.to_string(), "st25dv-rf:nfc");
    assert_eq!(
        log.events()[0].payload,
        rf::Command::encode_all(&[rf::Command::Field(true), rf::Command::Inventory]),
        "the recording is readable as what it is: the tag's own command codes"
    );

    let replay = Arc::new(Recorder::replaying(log));
    let (mut replayed, reader, bus) = board(&replay);
    i2c_write(&bus, 0x0000, &[1, 2, 3, 4]);
    replayed.run_for(SLICE).expect("the write cycle");
    let (hash, seen, replies) = drive(
        &mut replayed,
        &replay,
        &rf::channel(READER),
        &reader,
        &vec![QUIET.to_vec(); script().len()],
    );

    assert_eq!(replayed.now(), recorded_at, "the same instant");
    assert_eq!(hash, recorded, "the same machine, bit for bit");
    assert_eq!(seen, recorded_gpo, "GPO moved on the same slices");
    assert_eq!(
        replies, recorded_replies,
        "and the tag answered the replayed reader exactly as it answered the live one — \
         which is why a response never has to be recorded"
    );
    assert_eq!(replay.cursor(), 3, "every recorded command was delivered");
}

#[test]
fn a_board_whose_reader_has_no_channel_refuses_to_build() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    hosts.seal(Arc::clone(&recorder)).expect("an empty table");

    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.hosts = hosts;
    let registry = catalog::registry().expect("this build's registry");
    let err = rsemu::machine::build("st25dv.machine", BOARD, &registry, &options)
        .expect_err("the reader has no channel");
    let text = format!("{err}");
    assert!(
        text.contains("st25dv-rf:nfc"),
        "the refusal names the input that bypassed the seam: {text}"
    );
    assert!(text.contains("replay"), "and says why it matters: {text}");
}

#[test]
fn sealing_during_realize_wires_the_reader_rather_than_refusing_the_board() {
    // The other half of the seal's contract, and the half a board actually
    // takes: `HostKind::door` carries a sink factory, so a caller who never
    // declared the channel gets a *recorded* reader instead of an error.
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.hosts = Arc::clone(&hosts);
    options.realize.recorder = Some(Arc::clone(&recorder));
    let registry = catalog::registry().expect("this build's registry");
    rsemu::machine::build("st25dv.machine", BOARD, &registry, &options)
        .expect("the seal wires a door it knows how to feed");

    assert!(hosts.is_sealed());
    assert!(
        recorder.knows(&rf::channel(READER)),
        "the reader the tag opened is on a channel nobody had to name"
    );
}
