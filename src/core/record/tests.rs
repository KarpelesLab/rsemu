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

#[test]
fn a_sealed_recorder_takes_no_more_channels() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = HostObjects::new();
    hosts.seal(Arc::clone(&recorder)).unwrap();
    assert!(recorder.is_sealed());
    assert!(
        recorder
            .register(console(), Arc::new(NullSink) as Arc<dyn InputSink>)
            .is_err(),
        "the two lists cannot drift after the seal"
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
