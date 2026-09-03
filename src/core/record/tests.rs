//! The seam's own tests: delivery order, the log's canonical form, the parser's
//! refusals, and the host-object seal.

use super::*;
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::state::MachineShape;
use alloc::vec;

/// A sink that keeps everything, so a test can assert on order as well as
/// content.
#[derive(Debug, Default)]
struct Recording {
    seen: Mutex<Vec<Vec<u8>>>,
    rewinds: Mutex<usize>,
}

impl Recording {
    fn new() -> Arc<Recording> {
        Arc::new(Recording {
            seen: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            rewinds: Mutex::with_rank(LockRank::LEAF, 0),
        })
    }

    fn payloads(&self) -> Vec<Vec<u8>> {
        self.seen.lock().clone()
    }
}

impl InputSink for Recording {
    fn deliver(&self, payload: &[u8]) {
        self.seen.lock().push(payload.to_vec());
    }

    fn on_rewind(&self) {
        *self.rewinds.lock() += 1;
    }
}

const CHARDEV: HostKind = HostKind::new("chardev");

fn console() -> Channel {
    Channel::new(CHARDEV, "console")
}

fn t(nanos: u64) -> GlobalTime {
    GlobalTime::from_nanos(nanos)
}

// ---------------------------------------------------------------------------
// delivery
// ---------------------------------------------------------------------------

#[test]
fn nothing_is_delivered_until_a_round_boundary() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();

    recorder.post(&console(), b"a").unwrap();
    recorder.post(&console(), b"b").unwrap();
    assert!(
        sink.payloads().is_empty(),
        "posting is queueing: the machine decides when"
    );

    assert_eq!(recorder.deliver(t(1_000)).unwrap(), 2);
    assert_eq!(sink.payloads(), vec![b"a".to_vec(), b"b".to_vec()]);
}

#[test]
fn everything_delivered_in_one_round_carries_that_rounds_instant() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();

    recorder.post(&console(), b"x").unwrap();
    recorder.deliver(t(5_000)).unwrap();
    recorder.post(&console(), b"y").unwrap();
    recorder.deliver(t(9_000)).unwrap();

    let log = recorder.log();
    assert_eq!(log.len(), 2);
    assert_eq!(log.events()[0].at, t(5_000));
    assert_eq!(log.events()[1].at, t(9_000));
}

#[test]
fn a_replay_delivers_the_log_and_discards_the_host() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    recorder.post(&console(), b"one").unwrap();
    recorder.deliver(t(1_000)).unwrap();
    recorder.post(&console(), b"two").unwrap();
    recorder.deliver(t(3_000)).unwrap();
    let log = recorder.log();

    let replayed = Recording::new();
    let replay = Recorder::replaying(log);
    replay
        .register(console(), Arc::clone(&replayed) as Arc<dyn InputSink>)
        .unwrap();

    // A host pumping live input at a replay is answered honestly.
    assert!(
        !replay.post(&console(), b"live").unwrap(),
        "a replay does not take live input"
    );

    // Nothing is due before the first event's instant.
    assert_eq!(replay.deliver(t(500)).unwrap(), 0);
    assert_eq!(replay.deliver(t(1_000)).unwrap(), 1);
    assert_eq!(replay.deliver(t(2_000)).unwrap(), 0);
    assert_eq!(replay.deliver(t(3_000)).unwrap(), 1);
    assert_eq!(replayed.payloads(), sink.payloads());
}

#[test]
fn a_round_that_skips_past_several_events_delivers_all_of_them_in_order() {
    // A replay whose rounds do not land exactly on the recorded instants must
    // still deliver everything that became due, and in log order. `at <= now`
    // is what makes that true; `at == now` would silently drop events.
    let mut log = InputLog::new();
    for (i, at) in [1_000u64, 1_500, 2_000].iter().enumerate() {
        log.push(InputEvent {
            at: t(*at),
            channel: console(),
            payload: vec![i as u8],
        })
        .unwrap();
    }

    let sink = Recording::new();
    let replay = Recorder::replaying(log);
    replay
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    assert_eq!(replay.deliver(t(9_000)).unwrap(), 3);
    assert_eq!(sink.payloads(), vec![vec![0], vec![1], vec![2]]);
}

#[test]
fn an_unregistered_channel_is_refused_rather_than_dropped() {
    let recorder = Recorder::recording();
    let err = recorder
        .post(&Channel::new(CHARDEV, "nobody"), b"hi")
        .unwrap_err();
    assert!(
        matches!(&err, Error::Config { at, .. } if at == "chardev:nobody"),
        "the error names the channel: {err}"
    );
}

#[test]
fn an_oversized_payload_is_refused_on_the_way_in() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    let huge = vec![0u8; MAX_PAYLOAD + 1];
    assert!(recorder.post(&console(), &huge).is_err());
}

#[test]
fn a_replay_skips_a_channel_it_has_no_object_for_but_still_advances() {
    let mut log = InputLog::new();
    log.push(InputEvent {
        at: t(1_000),
        channel: Channel::new(CHARDEV, "absent"),
        payload: vec![1],
    })
    .unwrap();
    log.push(InputEvent {
        at: t(2_000),
        channel: console(),
        payload: vec![2],
    })
    .unwrap();

    let sink = Recording::new();
    let replay = Recorder::replaying(log);
    replay
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    replay.deliver(t(3_000)).unwrap();
    assert_eq!(replay.cursor(), 2, "the cursor passed both");
    assert_eq!(sink.payloads(), vec![vec![2]]);
}

// ---------------------------------------------------------------------------
// rewind
// ---------------------------------------------------------------------------

#[test]
fn a_rewind_moves_the_cursor_back_and_tells_every_sink() {
    let mut log = InputLog::new();
    for at in [1_000u64, 2_000, 3_000] {
        log.push(InputEvent {
            at: t(at),
            channel: console(),
            payload: vec![(at / 1_000) as u8],
        })
        .unwrap();
    }

    let sink = Recording::new();
    let replay = Recorder::replaying(log);
    replay
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    replay.deliver(t(3_000)).unwrap();
    assert_eq!(replay.cursor(), 3);

    replay.rewind_to(t(2_000));
    assert_eq!(
        replay.cursor(),
        1,
        "back to the first event at or after 2µs"
    );
    assert_eq!(
        *sink.rewinds.lock(),
        1,
        "the sink was told to drop its queue"
    );

    replay.deliver(t(3_000)).unwrap();
    assert_eq!(
        sink.payloads(),
        vec![vec![1], vec![2], vec![3], vec![2], vec![3]],
        "the events after the target were delivered a second time"
    );
}

#[test]
fn a_rewind_while_recording_truncates_the_future() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    recorder.post(&console(), b"early").unwrap();
    recorder.deliver(t(1_000)).unwrap();
    recorder.post(&console(), b"late").unwrap();
    recorder.deliver(t(5_000)).unwrap();
    assert_eq!(recorder.log().len(), 2);

    recorder.rewind_to(t(3_000));
    let log = recorder.log();
    assert_eq!(log.len(), 1, "the branch that was rewound past is gone");
    assert_eq!(log.events()[0].payload, b"early".to_vec());
}

#[test]
fn pending_input_is_dropped_by_a_rewind() {
    let sink = Recording::new();
    let recorder = Recorder::recording();
    recorder
        .register(console(), Arc::clone(&sink) as Arc<dyn InputSink>)
        .unwrap();
    recorder.post(&console(), b"unheard").unwrap();
    recorder.rewind_to(GlobalTime::ZERO);
    assert_eq!(recorder.deliver(t(1_000)).unwrap(), 0);
    assert!(sink.payloads().is_empty());
}

// ---------------------------------------------------------------------------
// the log as a file format
// ---------------------------------------------------------------------------

fn sample_log() -> InputLog {
    let mut shape = MachineShape::new();
    shape.add_device("/cpu", "cpu.demo").unwrap();
    shape.add_region("bus", "ram", 0, 0x1000);
    let mut log = InputLog::for_shape(shape);
    log.push(InputEvent {
        at: t(1_000),
        channel: console(),
        payload: b"hello".to_vec(),
    })
    .unwrap();
    log.push(InputEvent {
        at: t(1_000),
        channel: Channel::new(HostKind::new("pad"), "player1"),
        payload: vec![0x80],
    })
    .unwrap();
    log.push(InputEvent {
        at: t(4_000),
        channel: console(),
        payload: Vec::new(),
    })
    .unwrap();
    log
}

#[test]
fn a_recording_round_trips() {
    let log = sample_log();
    let bytes = log.encode().unwrap();
    let back = InputLog::decode(&bytes).unwrap();
    assert_eq!(back, log);
    assert_eq!(
        back.encode().unwrap(),
        bytes,
        "re-encoding what was decoded is byte-identical"
    );
}

#[test]
fn a_recording_carries_the_machine_shape() {
    let bytes = sample_log().encode().unwrap();
    let back = InputLog::decode(&bytes).unwrap();
    let mut other = MachineShape::new();
    other.add_device("/cpu", "cpu.other").unwrap();
    assert!(
        !back.shape().diff(&other).is_empty(),
        "replaying into another board is a diff, not a boolean"
    );
}

#[test]
fn the_parser_refuses_what_it_did_not_write() {
    let good = sample_log().encode().unwrap();

    assert!(InputLog::decode(b"").is_err(), "empty");
    assert!(
        InputLog::decode(b"RSEMUSNP").is_err(),
        "a snapshot, not a recording"
    );

    let mut version = good.clone();
    version[8] = 0xff;
    assert!(
        InputLog::decode(&version).is_err(),
        "a future format version"
    );

    let mut trailing = good.clone();
    trailing.push(0);
    assert!(InputLog::decode(&trailing).is_err(), "trailing bytes");

    for cut in 0..good.len() {
        // Truncation at every offset: an error, never a panic.
        let _ = InputLog::decode(&good[..cut]);
    }
}

#[test]
fn out_of_order_events_are_refused_on_the_wire_and_in_memory() {
    let mut log = InputLog::new();
    log.push(InputEvent {
        at: t(5_000),
        channel: console(),
        payload: vec![1],
    })
    .unwrap();
    assert!(
        log.push(InputEvent {
            at: t(1_000),
            channel: console(),
            payload: vec![2],
        })
        .is_err(),
        "a recording is delivery-ordered"
    );

    // And the same on decode: hand-build a body with descending instants.
    let mut bytes: Vec<u8> = Vec::new();
    bytes.write_all(b"RSEMURPL").unwrap();
    bytes.write_u32(LOG_FORMAT_VERSION).unwrap();
    MachineShape::new().encode_into(&mut bytes).unwrap();
    for at in [5_000u64, 1_000] {
        bytes.write_u8(TAG_EVENT).unwrap();
        bytes.write_u128(t(at).raw()).unwrap();
        bytes.write_str("chardev").unwrap();
        bytes.write_str("console").unwrap();
        bytes.write_bytes(&[0]).unwrap();
    }
    bytes.write_u8(TAG_END).unwrap();
    assert!(InputLog::decode(&bytes).is_err());
}

#[test]
fn index_at_finds_the_rewind_cursor() {
    let log = sample_log();
    assert_eq!(log.index_at(GlobalTime::ZERO), 0);
    assert_eq!(log.index_at(t(1_000)), 0, "at or after, so the pair at 1µs");
    assert_eq!(log.index_at(t(2_000)), 2);
    assert_eq!(log.index_at(t(9_000)), 3);
}

// ---------------------------------------------------------------------------
// the seal: the enforcement made visible
// ---------------------------------------------------------------------------

#[test]
fn an_open_table_lets_anything_through() {
    let hosts = HostObjects::new();
    assert!(!hosts.is_sealed());
    hosts.open(CHARDEV, "console", || 7u32).unwrap();
}

#[test]
fn a_sealed_table_refuses_a_host_object_the_recorder_does_not_know() {
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console(), Arc::new(NullSink) as Arc<dyn InputSink>)
        .unwrap();

    let hosts = HostObjects::new();
    hosts.seal(Arc::clone(&recorder)).unwrap();
    assert!(hosts.is_sealed());

    // The registered channel is fine.
    hosts.open(CHARDEV, "console", || 1u32).unwrap();

    // A second one — the next device that quietly grows an input — is not.
    let err = hosts.open(CHARDEV, "console2", || 2u32).unwrap_err();
    assert!(
        matches!(&err, Error::Config { at, .. } if at == "chardev:console2"),
        "the refusal names the channel that bypassed the seam: {err}"
    );

    // And so is a different kind under a registered name.
    assert!(
        hosts
            .open(HostKind::new("pad"), "console", || 3u32)
            .is_err()
    );
}

#[test]
fn sealing_a_populated_table_checks_what_is_already_open() {
    let hosts = HostObjects::new();
    hosts.open(CHARDEV, "console", || 1u32).unwrap();
    hosts
        .open(HostKind::new("pad"), "player1", || 2u32)
        .unwrap();

    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console(), Arc::new(NullSink) as Arc<dyn InputSink>)
        .unwrap();
    let err = hosts.seal(Arc::clone(&recorder)).unwrap_err();
    assert!(
        matches!(&err, Error::Config { at, .. } if at == "pad:player1"),
        "sealing late still names what was missed: {err}"
    );
    assert!(!hosts.is_sealed(), "a refused seal leaves the table open");
}

/// A table seal does *not* close the channel list; the first delivery does.
///
/// It used to, and that made the mechanism unusable on the one path that
/// matters: a frontend cannot exist before the machine it draws, so `--vnc
/// --record-input` registers `input:vnc` after the build — which is after
/// `machine::realize` has sealed the table. Nothing is lost by waiting, because
/// [`Recorder::register`]'s own reason for refusing is that a late channel
/// "would silently have missed everything before it", and before the first
/// round boundary there is nothing to have missed.
#[test]
fn the_channel_list_closes_at_the_first_delivery_not_at_the_seal() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = HostObjects::new();
    hosts.seal(Arc::clone(&recorder)).unwrap();
    assert!(!recorder.is_sealed(), "a sealed table is not a sealed list");

    recorder
        .register(console(), Arc::new(NullSink) as Arc<dyn InputSink>)
        .expect("a frontend attached after the build still gets its channel");

    recorder.deliver(t(1_000)).unwrap();
    assert!(recorder.is_sealed(), "the first round boundary closes it");
    assert!(
        recorder
            .register(
                Channel::new(CHARDEV, "late"),
                Arc::new(NullSink) as Arc<dyn InputSink>
            )
            .is_err(),
        "and after that a channel would have missed everything already delivered"
    );
}

/// The seal's reach, now that a kind says what it is.
///
/// [`HostObjects`] files three unrelated things under one `(kind, name)` space,
/// and until [`HostKind`] carried a role the seal checked all three alike: a
/// `pci-bus` is not non-deterministic, cannot be a channel and has nothing to
/// record, yet a sealed table refused it exactly as it refused an undeclared
/// keyboard. That is why nothing in `src/` sealed anything — sealing any board
/// with a PCI or USB bus in it failed on an object that was never an input.
///
/// The three, and what the seal now does with each:
///
/// * a **door** — host input as `(instant, payload)`. Refused unless declared,
///   which is the whole point;
/// * a **rendezvous** — how two devices inside one build find each other.
///   Ignored;
/// * **pulled** — `medium`, where host bytes really do cross but the guest asks
///   for a sector rather than receiving one, so no `(instant, payload)` log
///   could describe it. Ignored, and still a hole.
#[test]
fn the_seal_tells_a_door_from_a_rendezvous() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = HostObjects::new();
    hosts.seal(Arc::clone(&recorder)).unwrap();

    // A door. Refusing this is the point of the seal: an input the recorder
    // does not know about would be missing from every recording.
    assert!(
        hosts.open(CHARDEV, "console", || 1u32).is_err(),
        "an undeclared input is what the seal exists to catch"
    );

    // A rendezvous. Refusing this caught nothing and cost the seal every board
    // above the smallest.
    for rendezvous in [
        "pci-bus",
        "usb-bus",
        "i2c-bus",
        "spi-bus",
        "ata-bay",
        "sd-slot",
        "floppy-drive",
        "apic-bus",
        "signal",
        "riscv.dt",
        "capture",
    ] {
        hosts
            .open(HostKind::rendezvous(rendezvous), "0", || 1u32)
            .unwrap_or_else(|e| {
                panic!("`{rendezvous}` is how two ends of one build meet, not an input: {e}")
            });
    }

    // And the third kind: host bytes really do cross at a `medium`, but the
    // guest pulls them a sector at a time, so there is no channel to declare
    // and demanding one would mean no board with a disk could be sealed.
    hosts
        .open(HostKind::pulled("medium"), "hd0", || 1u32)
        .expect("a drive's image has no `(instant, payload)` shape to declare");
}

/// A door whose module ships a `sink()` wires itself; one that does not is
/// refused, and says which two functions to write.
///
/// This is the pair that decides whether any of this was worth doing.
/// `core::record` lists four things a new device must do to be covered and
/// says step 2 — ship `channel()` and `sink()` beside `open()` — is the one
/// that gets skipped invisibly. Under a sealed table it is now the one that
/// stops the build.
#[test]
fn a_door_that_can_feed_itself_is_wired_and_one_that_cannot_is_refused() {
    /// The ten lines a door's module owes the seam, as a `HostKind` carries
    /// them.
    fn feed(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        let thing = Arc::clone(object).downcast::<Recording>().ok()?;
        Some(thing as Arc<dyn InputSink>)
    }
    const WIRED: HostKind = HostKind::door("test.wired", feed);
    const BARE: HostKind = HostKind::new("test.bare");

    // A door that knows where its payloads go is registered by the seal, so a
    // caller that never heard of it still records it.
    let hosts = HostObjects::new();
    let sink = hosts.open(WIRED, "console", Recording::default).unwrap();
    let recorder = Arc::new(Recorder::recording());
    hosts.seal(Arc::clone(&recorder)).unwrap();

    let channel = Channel::new(WIRED, "console");
    assert!(recorder.knows(&channel), "the seal wired it");
    recorder.post(&channel, b"typed").unwrap();
    recorder.deliver(t(1_000)).unwrap();
    assert_eq!(
        sink.payloads(),
        [b"typed".to_vec()],
        "and it reaches the object"
    );

    // A door whose module never shipped the pair is where the build stops.
    let hosts = HostObjects::new();
    hosts.open(BARE, "thermometer", || 1u32).unwrap();
    let err = hosts
        .seal(Arc::new(Recorder::recording()))
        .expect_err("nothing can say where this one's payloads go");
    assert!(
        matches!(&err, Error::Config { at, .. } if at == "test.bare:thermometer"),
        "the refusal names the object: {err}"
    );
    let text = alloc::format!("{err}");
    assert!(
        text.contains("`channel()`") && text.contains("`sink()`"),
        "and says which two functions to write: {text}"
    );
}

#[test]
fn unsealing_gives_the_table_back() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = HostObjects::new();
    hosts.seal(Arc::clone(&recorder)).unwrap();
    assert!(hosts.open(CHARDEV, "anything", || 1u32).is_err());
    hosts.unseal();
    hosts.open(CHARDEV, "anything", || 1u32).unwrap();
}

#[test]
fn a_recorder_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Recorder>();
    assert_send_sync::<InputLog>();
    assert_send_sync::<Channel>();
}
