//! A keypad's presses, through the record/replay seam and nowhere else
//! (`ROADMAP.md` §4.5; `CLAUDE.md`, determinism).
//!
//! `tests/joypad_replay.rs` makes this argument for the Game Boy and the Master
//! System and this file makes it for `keypad.matrix`, in the same three claims
//! and the same order:
//!
//! 1. **Pressing changes the run.** A board reaches a different state when
//!    somebody held a key down. Without this, "record and replay agree" would
//!    be satisfied by a seam that dropped every press.
//! 2. **A recording replays bit for bit.** It goes out through the file format
//!    and comes back, and a fresh machine driven only by it reaches the same
//!    state hash at the same instant, with every recorded event delivered.
//! 3. **A board whose keypad has no channel refuses to build.** The seal, on
//!    the host object the device opens by name.
//!
//! # The board
//!
//! Inline, and deliberately not a product: what is being asserted is the seam,
//! and a board with a firmware on it would only add ways for the assertion to
//! be about something else. There is **no processor** — a keypad is not
//! addressable, so nothing here needs one. A pair of inverters stands in for
//! the GPIO port a real scan would use: the first idles high (the realize
//! sweep's own example, `ROADMAP.md` §4.3), so the second holds row 1 low, and
//! row 1 stays selected for the whole run. A `wire.level-to-edge` watches the
//! column, so the column's level is architectural state that lands in the
//! snapshot the hash is taken over — which is what makes claim 1 about the
//! *conductor* rather than about a bitmap the device happens to save.

#![cfg(feature = "dev-keypad-matrix")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::HostObjects;
use rsemu::core::record::{Channel, InputLog, Recorder};
use rsemu::core::wire::Level;
use rsemu::dev::keypad::{DEFAULT_KEYPAD_PORT, keys};
use rsemu::machine::{BuildOptions, Machine, catalog};

/// A four-by-three keypad with row 1 held low and column 2 watched.
const BOARD: &str = r#"
machine "keypad" {
  osc clk = 1000000 Hz
  space mem { width = 16, unassigned = read-as-ones }
  object wram "ram" { size = 1K }
  object hi   "wire.not" {}
  object sel  "wire.not" {}
  object pad  "keypad.matrix" {
    rows   = 4
    cols   = 3
    layout = "1,2,3,4,5,6,7,8,9,*,0,#"
  }
  object scan "wire.level-to-edge" { edge = "both" }
  map mem 0x0000 size 1K = wram
  wire hi.out   -> sel.in
  wire sel.out  -> pad.row1
  wire pad.col2 -> scan.in { pull = "up" }
}
"#;

/// The key at row 1, column 2 — `"6"` on the keypad above, index `1*3 + 2`.
const KEY_SIX: u8 = 5;

/// The key at row 2, column 2 — `"9"`, on a row this board never selects.
const KEY_NINE: u8 = 8;

/// One movement, in [`keys::RECORD_BYTES`] bytes: press `6`.
const PRESS_SIX: &[u8] = &[KEY_SIX, 1];

/// Two movements in one post — release `6` and press `9` — because a host that
/// batches what happened between two round boundaries posts them together, and
/// the sink has to apply them in order.
const SWAP: &[u8] = &[KEY_SIX, 0, KEY_NINE, 1];

/// A slice with nothing posted, for the silent control and for a replay.
const QUIET: &[u8] = b"";

/// How long each slice of a run is.
const SLICE: GlobalTime = GlobalTime::from_nanos(1_000_000);

/// Build the board against a host-object table the caller keeps.
fn build(hosts: &Arc<HostObjects>) -> Machine {
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.hosts = Arc::clone(hosts);
    let registry = catalog::registry().expect("this build's registry");
    match rsemu::machine::build("keypad.machine", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the keypad board does not realize: {e}"),
    }
}

/// The board, with its keypad registered as a record/replay channel.
///
/// Three lines, and they are the whole conversion: the keypad is a host object
/// opened by name, `keys::channel` is that name as a channel, and `keys::sink`
/// is what a payload does once the machine has decided when.
fn board(recorder: &Arc<Recorder>) -> Machine {
    let hosts = Arc::new(HostObjects::new());
    let keypad = keys::open(&hosts, DEFAULT_KEYPAD_PORT).expect("a keypad before the build");
    recorder
        .register(keys::channel(DEFAULT_KEYPAD_PORT), keys::sink(&keypad))
        .expect("a fresh recorder takes channels");
    let mut machine = build(&hosts);
    machine
        .set_recorder(Arc::clone(recorder))
        .expect("the board runs deterministically");
    machine.reset(ResetKind::Cold);
    machine.sweep();
    machine
}

/// What column 2 is actually sitting at: the net the keypad's `col2` pin
/// drives, asked for its own resolution.
fn column_two(machine: &Machine) -> Level {
    let net = machine
        .nets()
        .iter()
        .find(|net| {
            net.sources().iter().any(|pin| {
                pin.port == "col2" && machine.devices()[pin.device].path().ends_with("pad")
            })
        })
        .expect("the keypad drives column 2");
    net.wire().resolve_net()
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
) -> (u64, Vec<Level>) {
    let mut seen = Vec::with_capacity(presses.len());
    for press in presses {
        if !press.is_empty() {
            recorder.post(channel, press).expect("a registered channel");
        }
        machine.run_for(SLICE).expect("a deterministic run");
        seen.push(column_two(machine));
    }
    (
        machine.state_hash().expect("deterministic mode hashes"),
        seen,
    )
}

fn script() -> Vec<&'static [u8]> {
    vec![QUIET, QUIET, PRESS_SIX, QUIET, SWAP, QUIET]
}

#[test]
fn pressing_a_key_pulls_the_column_down_and_changes_where_the_run_ends_up() {
    let recorder = Arc::new(Recorder::recording());
    let mut quiet = board(&recorder);
    let (silent, silent_seen) = drive(
        &mut quiet,
        &recorder,
        &keys::channel(DEFAULT_KEYPAD_PORT),
        &[QUIET; 6],
    );
    assert!(
        silent_seen.iter().all(|l| l.is_high()),
        "nobody pressed anything, so the pull-up had the column the whole time"
    );

    let recorder = Arc::new(Recorder::recording());
    let mut pressed = board(&recorder);
    let (held, held_seen) = drive(
        &mut pressed,
        &recorder,
        &keys::channel(DEFAULT_KEYPAD_PORT),
        &script(),
    );
    assert_eq!(
        held_seen,
        vec![
            Level::High,
            Level::High,
            Level::Low,
            Level::Low,
            Level::High,
            Level::High
        ],
        "the column follows the key: row 1 is held low, and `6` is at (1,2)"
    );
    assert_ne!(
        silent, held,
        "the run ends with `9` held, which the silent one does not"
    );
    assert_eq!(recorder.log().len(), 2, "two posts, two logged events");
}

#[test]
fn a_recorded_session_replays_to_the_same_state_hash() {
    let recorder = Arc::new(Recorder::recording());
    let mut machine = board(&recorder);
    let (recorded, recorded_seen) = drive(
        &mut machine,
        &recorder,
        &keys::channel(DEFAULT_KEYPAD_PORT),
        &script(),
    );
    let recorded_at = machine.now();

    // Out through the file format and back, so what is replayed is what a
    // `.trace` on disk would hold rather than a live object.
    let bytes = recorder.log().encode().expect("a recording encodes");
    let log = InputLog::decode(&bytes).expect("and decodes");
    assert_eq!(log.events()[0].channel.to_string(), "keypad:keypad");
    assert_eq!(log.events()[0].payload.len(), keys::RECORD_BYTES);
    assert_eq!(
        log.events()[1].payload.len(),
        2 * keys::RECORD_BYTES,
        "two movements travelled in one post"
    );

    let replay = Arc::new(Recorder::replaying(log));
    let mut replayed = board(&replay);
    let (hash, seen) = drive(
        &mut replayed,
        &replay,
        &keys::channel(DEFAULT_KEYPAD_PORT),
        &[QUIET; 6],
    );

    assert_eq!(replayed.now(), recorded_at, "the same instant");
    assert_eq!(hash, recorded, "the same machine, bit for bit");
    assert_eq!(
        seen, recorded_seen,
        "the column moved on the same slices it moved on when recorded"
    );
    assert_eq!(replay.cursor(), 2, "every recorded movement was delivered");
}

#[test]
fn a_board_whose_keypad_has_no_channel_refuses_to_build() {
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    hosts.seal(Arc::clone(&recorder)).expect("an empty table");

    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.hosts = hosts;
    let registry = catalog::registry().expect("this build's registry");
    let err = rsemu::machine::build("keypad.machine", BOARD, &registry, &options)
        .expect_err("the keypad has no channel");
    let text = format!("{err}");
    assert!(
        text.contains("keypad:keypad"),
        "the refusal names the input that bypassed the seam: {text}"
    );
    assert!(text.contains("replay"), "and says why it matters: {text}");
}
