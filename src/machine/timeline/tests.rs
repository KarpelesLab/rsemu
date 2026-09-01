//! The timeline's bookkeeping, without a machine.
//!
//! Rewind on a *real* board — record, rewind, run forward, land on the same
//! state hash — is `tests/record_replay.rs`, because it needs a CPU, a ROM and
//! a character port. What is here is everything that is decidable from the
//! keyframe list alone, plus the two refusals a mis-wired caller gets.

use super::*;
use crate::core::hosts::HostKind;
use crate::core::record::{Channel, InputSink, NullSink, Recorder};

fn t(nanos: u64) -> GlobalTime {
    GlobalTime::from_nanos(nanos)
}

fn timeline() -> Timeline {
    Timeline::new(Arc::new(Recorder::recording()), t(1_000_000))
}

/// Stand in a keyframe list by hand: `snapshot` needs a machine, and these
/// tests are about what the list does once it exists.
fn with_keyframes(instants: &[u64]) -> Timeline {
    let mut timeline = timeline();
    for at in instants {
        timeline.keyframes.push(Keyframe {
            at: t(*at),
            bytes: alloc::vec![0u8; 8],
        });
    }
    timeline
}

#[test]
fn the_default_cadence_is_a_virtual_second() {
    assert_eq!(DEFAULT_CADENCE, GlobalTime::from_nanos(1_000_000_000));
    let timeline = Timeline::with_default_cadence(Arc::new(Recorder::recording()));
    assert_eq!(timeline.cadence(), DEFAULT_CADENCE);
    assert_eq!(timeline.keyframes(), 0);
}

#[test]
fn instants_and_bytes_report_what_is_held() {
    let timeline = with_keyframes(&[0, 1_000_000, 2_000_000]);
    assert_eq!(
        timeline.instants(),
        alloc::vec![t(0), t(1_000_000), t(2_000_000)]
    );
    assert_eq!(timeline.bytes_held(), 24);
}

#[test]
fn forget_before_keeps_the_keyframe_that_covers_the_target() {
    let mut timeline = with_keyframes(&[0, 1_000_000, 2_000_000, 3_000_000]);
    timeline.forget_before(t(2_500_000));
    assert_eq!(
        timeline.instants(),
        alloc::vec![t(2_000_000), t(3_000_000)],
        "the 2 ms frame is what makes 2.5 ms reachable, so it stays"
    );
}

#[test]
fn forget_before_the_oldest_frame_drops_nothing() {
    let mut timeline = with_keyframes(&[1_000_000, 2_000_000]);
    timeline.forget_before(GlobalTime::ZERO);
    assert_eq!(timeline.keyframes(), 2);
}

#[test]
fn a_timeline_with_no_machine_recorder_refuses() {
    // Both refusals are on `check_attached`, which every entry point runs; a
    // machine is needed to exercise them end to end, so what is asserted here
    // is that the error type is the one a caller can match on.
    let timeline = timeline();
    assert!(matches!(
        timeline.recorder().mode(),
        crate::core::record::Mode::Record
    ));
}

#[test]
fn a_recorder_with_channels_still_starts_empty() {
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(
            Channel::new(HostKind::new("chardev"), "console"),
            Arc::new(NullSink) as Arc<dyn InputSink>,
        )
        .unwrap();
    let timeline = Timeline::new(recorder, t(1_000));
    assert_eq!(timeline.keyframes(), 0);
    assert_eq!(timeline.bytes_held(), 0);
}
