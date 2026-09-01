#![no_main]
//! A recording is a file a user hands us, so its reader is a parser on
//! untrusted input.
//!
//! `rsemu record session.trace` writes one and `rsemu replay session.trace`
//! reads it back on a different host, possibly from a bug report, possibly
//! produced by an older build. `core::record` therefore states the same
//! contract `core::state` does — never panic, never trust a length it has not
//! compared against the bytes remaining, never allocate against a claimed
//! count — and this target holds it to it.
//!
//! Three properties, in increasing strength:
//!
//! 1. **No panic.** `InputLog::decode` over arbitrary bytes, then every
//!    accessor on whatever came back.
//! 2. **Self-consistency.** A recording that parsed must agree with itself: its
//!    events are in non-descending instant order (the module's whole ordering
//!    rule), `index_at` agrees with a linear scan over them, no payload exceeds
//!    the format's limit, and the shape it carries diffs empty against itself.
//! 3. **Canonical form.** A recording has exactly one valid encoding, so
//!    re-encoding what was decoded must reproduce the fuzzer's own bytes. That
//!    is the property replay rests on: two spellings of one recording would
//!    mean two byte strings that are the same session, and "the same recording
//!    produces the same run" would stop being checkable by comparison.
//!
//! The decoded log is also *replayed* into a recorder against a counting sink,
//! because the delivery path reads attacker-chosen instants and channel names
//! out of the log and must terminate on every one of them — a cursor that could
//! fail to advance would be an infinite loop inside a run, which shows up here
//! as a timeout.

use libfuzzer_sys::fuzz_target;
use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::record::{InputLog, InputSink, MAX_PAYLOAD, NullSink, Recorder};

fuzz_target!(|data: &[u8]| {
    // Anything that fails to parse has already proved the only thing this
    // target can prove about it: it did not panic.
    let Ok(log) = InputLog::decode(data) else {
        return;
    };

    // A shape is its own twin, exactly as in the snapshot reader.
    let shape = log.shape().clone();
    let diff = shape.diff(log.shape());
    assert!(
        diff.is_empty(),
        "a recording's shape differs from itself: {diff}"
    );

    let events = log.events();
    assert_eq!(events.len(), log.len());
    assert_eq!(events.is_empty(), log.is_empty());
    assert_eq!(log.last_instant(), events.last().map(|e| e.at));

    for pair in events.windows(2) {
        assert!(
            pair[0].at <= pair[1].at,
            "a recording is delivery-ordered, got {} then {}",
            pair[0].at.raw(),
            pair[1].at.raw()
        );
    }

    for event in events {
        assert!(
            event.payload.len() <= MAX_PAYLOAD,
            "the decoder accepted a {}-byte payload",
            event.payload.len()
        );
        // Every one of these strings came out of the input.
        let _ = event.channel.to_string();
        let _ = event.channel.kind();
        let _ = event.channel.name();

        // The rewind cursor must agree with a linear scan, on instants the
        // fuzzer chose rather than ones a writer produced.
        let expected = events.iter().position(|e| e.at >= event.at).unwrap_or(0);
        assert_eq!(
            log.index_at(event.at),
            expected,
            "index_at disagrees with a scan at {}",
            event.at.raw()
        );
    }
    assert_eq!(log.index_at(GlobalTime::ZERO), 0);
    assert_eq!(log.index_at(GlobalTime::MAX), events.len());

    // Replay it. Every event must be delivered exactly once and the cursor must
    // reach the end, whatever instants the input chose.
    let recorder = Recorder::replaying(log.clone());
    let known: Option<rsemu::core::record::Channel> = events.first().map(|e| e.channel.clone());
    if let Some(channel) = known {
        recorder
            .register(channel, Arc::new(NullSink) as Arc<dyn InputSink>)
            .expect("an unsealed recorder takes channels");
    }
    // Delivering at MAX drains everything; delivering again must be a no-op,
    // which is what proves the cursor is monotone rather than merely advanced.
    let first = recorder.deliver(GlobalTime::MAX).expect("delivery");
    assert_eq!(recorder.cursor(), events.len(), "the cursor reached the end");
    let second = recorder.deliver(GlobalTime::MAX).expect("delivery");
    assert_eq!(second, 0, "a drained log delivers nothing more");
    assert!(first <= events.len());

    // A rewind to the beginning must put it back exactly, and a second drain
    // must deliver the same count.
    recorder.rewind_to(GlobalTime::ZERO);
    assert_eq!(recorder.cursor(), 0);
    assert_eq!(
        recorder.deliver(GlobalTime::MAX).expect("delivery"),
        first,
        "a rewound replay delivers the same events again"
    );

    // Canonical form: re-encoding what was decoded reproduces the input.
    let reencoded = log.encode().expect("a decoded log is within the format limits");
    assert!(
        reencoded == data,
        "a recording has exactly one valid encoding, but decoding {} bytes and \
         re-encoding produced {} different bytes",
        data.len(),
        reencoded.len()
    );
});
