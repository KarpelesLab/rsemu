//! Record a session on a real board, replay it to an identical state hash, and
//! rewind (`ROADMAP.md` §4.5, and phase 9's gate).
//!
//! The board is the Apple 1: the smallest machine rsemu ships, and — more to
//! the point — the smallest one with **real non-deterministic input**. Its
//! whole I/O is one MC6821 with a keyboard on one port, so what a person types
//! is the entire difference between two runs. RSMON, rsemu's own monitor, sits
//! in the ROM socket, so the test needs no image of unclear provenance.
//!
//! Five things are proved here, and the first is the one that makes the other
//! four mean anything:
//!
//! 1. **The input matters.** A run with keystrokes and a run without reach
//!    different state hashes. Without this the rest is a test that two empty
//!    runs agree, which they always did.
//! 2. **A replay is bit-identical.** A fresh machine, driven only by the
//!    recording, reaches the same hash at the same instant, and the guest
//!    prints the same bytes.
//! 3. **Rewind lands on the hash it left.** Run, snapshot on a cadence, go back
//!    to an earlier instant, run forward again, arrive at the same number.
//! 4. **A device cannot bypass the seam.** A sealed host-object table refuses
//!    to build a board whose input the recorder has no channel for, naming it.
//! 5. **A frozen recording replays to a pinned state.** The bytes and the
//!    resulting hash are constants in this file, so the comparison crosses
//!    runs, builds and machines rather than staying inside one process.
//!
//! Everything needs a machine, so the whole file is gated on `machine-apple1`.
//!
//! # What the fifth one can and cannot prove
//!
//! Phase 9's gate is *"a recorded session replayed bit-identically on a
//! different host"*, and 1–4 do not test it however green they are: each
//! records and replays inside one process, so a host that disagreed with every
//! other host would still pass all four as long as it disagreed with itself
//! consistently. Nothing here compared a result against anything that came from
//! outside the run.
//!
//! Pinning the artefact is what converts an existing but unexploited CI matrix
//! into the gate. `cargo test --all-features` already runs on `ubuntu-latest`,
//! `macos-latest` and `windows-latest` — three operating systems, two
//! instruction sets, three linkers — and against a constant in the source those
//! three become three hosts replaying one recording. This has also been run by
//! hand on `i686-unknown-linux-gnu`, where `usize` is four bytes; the hash is
//! the same, which is a stronger statement than a second 64-bit host would have
//! made.
//!
//! What is still **not** proved, and cannot be from one machine: a big-endian
//! host (none is buildable-and-runnable here), a host with a different
//! floating-point environment reaching a guest that uses one, and any wasm
//! target — CI *builds* all three wasm targets and runs none of them, because
//! there is no wasm test runner in this repository. A `wasm32-wasip1` job under
//! `wasmtime` would add the most: a genuinely different execution environment
//! with a 32-bit address space, and this test would need no changes to be the
//! thing it ran.

#![cfg(all(feature = "machine-apple1", feature = "std"))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::{HostKind, HostObjects};
use rsemu::core::record::{Channel, FnSink, InputSink, NullSink, Recorder};
use rsemu::core::sched::ThreadingMode;
use rsemu::host::chardev::{CharPort, ports};
use rsemu::machine::{Machine, Timeline, catalog};

/// The port name `machines/apple1.machine` defaults its `console` param to.
const CONSOLE: &str = "console";

/// How long each run is. Long enough for RSMON to print its banner and echo
/// what is typed at it, short enough to keep the test quick.
const SPAN: GlobalTime = GlobalTime::from_nanos(400_000_000);

fn console_channel() -> Channel {
    Channel::new(ports::KIND, CONSOLE)
}

/// A sink that feeds a character port, and drops what it is holding on a
/// rewind.
///
/// The whole adapter between `core::record` and `host::chardev`: two closures.
/// The rewind half matters — bytes queued at the rewind target are re-delivered
/// from the log, so a port that kept them would hand the guest each one twice.
fn port_sink(port: &Arc<CharPort>) -> Arc<dyn InputSink> {
    let feeding = Arc::clone(port);
    let clearing = Arc::clone(port);
    Arc::new(
        FnSink::new("chardev:console", move |bytes: &[u8]| {
            feeding.feed(bytes);
        })
        .on_rewind(move || clearing.clear()),
    )
}

/// An Apple 1 with RSMON in its ROM socket, plus the host-object table its
/// devices opened.
fn apple1(mode: ThreadingMode) -> (Machine, Arc<HostObjects>) {
    apple1_with_hosts(mode, Arc::new(HostObjects::new()))
}

fn apple1_with_hosts(mode: ThreadingMode, hosts: Arc<HostObjects>) -> (Machine, Arc<HostObjects>) {
    let entry = catalog::machine("apple1").expect("this build ships apple1");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.mode = mode;
    options
        .realize
        .media
        .insert("rom", rsemu::dev::apple1::RSMON.as_slice());
    options.realize.hosts = Arc::clone(&hosts);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the board realizes");
    (machine, hosts)
}

/// What the guest printed, as it would appear on a terminal.
fn output(port: &CharPort) -> Vec<u8> {
    port.drain()
}

/// Run a machine in `chunks` slices of `SPAN`, posting `keys[i]` before slice
/// `i`. Returns the final state hash and everything the guest printed.
///
/// Posting before a slice rather than at some wall-clock moment is what keeps
/// the *test* deterministic. It changes nothing about what is being proved:
/// the recorder does not know when the host called it, only which round
/// boundary the machine drained it on.
fn drive(
    machine: &mut Machine,
    port: &CharPort,
    recorder: &Recorder,
    keys: &[&[u8]],
) -> (u64, Vec<u8>) {
    let channel = console_channel();
    let mut printed = Vec::new();
    for key in keys {
        if !key.is_empty() {
            recorder.post(&channel, key).expect("a registered channel");
        }
        machine.run_for(SPAN).expect("a deterministic run");
        printed.extend_from_slice(&output(port));
    }
    (
        machine.state_hash().expect("deterministic mode hashes"),
        printed,
    )
}

/// The keystrokes typed at the recorded session: a memory dump command, then
/// two more lines. RSMON echoes what it is given, so every one of them moves
/// guest state.
const TYPED: [&[u8]; 5] = [b"", b"0\r", b"E000.E00F\r", b"", b"FF\r"];

// ---------------------------------------------------------------------------
// 1. the input matters
// ---------------------------------------------------------------------------

#[test]
fn typing_at_the_machine_changes_where_it_ends_up() {
    // The control. Without this, "record and replay agree" would be satisfied
    // by a seam that dropped every keystroke on the floor.
    let (mut quiet, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    quiet
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic");
    quiet.reset(ResetKind::Cold);
    let (silent_hash, silent_output) =
        drive(&mut quiet, &port, &recorder, &[b"", b"", b"", b"", b""]);

    let (mut typed, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    typed
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic");
    typed.reset(ResetKind::Cold);
    let (typed_hash, typed_output) = drive(&mut typed, &port, &recorder, &TYPED);

    assert_ne!(
        silent_hash, typed_hash,
        "the keystrokes reached the guest, so the two runs are different machines"
    );
    assert_ne!(
        silent_output, typed_output,
        "and the guest printed something different because of them"
    );
    assert_eq!(
        recorder.log().len(),
        TYPED.iter().filter(|k| !k.is_empty()).count(),
        "every keystroke is one logged event"
    );
}

// ---------------------------------------------------------------------------
// 2. a replay is bit-identical
// ---------------------------------------------------------------------------

#[test]
fn a_recorded_session_replays_to_the_same_state_hash() {
    // Record.
    let (mut machine, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    machine
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic threading records");
    machine.reset(ResetKind::Cold);
    let (recorded_hash, recorded_output) = drive(&mut machine, &port, &recorder, &TYPED);
    let recorded_at = machine.now();

    // The recording goes through the file format on the way, so what is
    // replayed is what a `.trace` on disk would hold rather than a live object.
    let bytes = recorder.log().encode().expect("a recording encodes");
    let log = rsemu::core::record::InputLog::decode(&bytes).expect("and decodes");
    assert!(!log.is_empty(), "there is something to replay");

    // Replay, on a machine that shares nothing with the first: its own host
    // objects, its own port, its own devices.
    let (mut replayed, hosts) = apple1(ThreadingMode::Deterministic);
    let replay_port = ports::open(&hosts, CONSOLE).expect("the console port");
    let replay = Arc::new(Recorder::replaying(log));
    replay
        .register(console_channel(), port_sink(&replay_port))
        .expect("a fresh recorder takes channels");
    replayed
        .set_recorder(Arc::clone(&replay))
        .expect("deterministic threading replays");
    replayed.reset(ResetKind::Cold);

    // No keystrokes this time: everything comes out of the log. A `post` here
    // would be discarded, and the seam says so.
    let (replayed_hash, replayed_output) = drive(
        &mut replayed,
        &replay_port,
        &replay,
        &[b"", b"", b"", b"", b""],
    );

    assert_eq!(replayed.now(), recorded_at, "the same instant");
    assert_eq!(
        replayed_hash, recorded_hash,
        "a replayed session is the same machine, bit for bit"
    );
    assert_eq!(
        replayed_output, recorded_output,
        "and it printed the same thing"
    );
    assert_eq!(
        replay.cursor(),
        TYPED.iter().filter(|k| !k.is_empty()).count(),
        "every recorded event was delivered"
    );
}

#[test]
fn a_replay_refuses_live_input() {
    let log = rsemu::core::record::InputLog::new();
    let replay = Recorder::replaying(log);
    replay
        .register(console_channel(), Arc::new(NullSink) as Arc<dyn InputSink>)
        .expect("a fresh recorder takes channels");
    assert!(
        !replay.post(&console_channel(), b"x").expect("registered"),
        "a replay that also took live input would not be a replay"
    );
}

// ---------------------------------------------------------------------------
// 3. rewind
// ---------------------------------------------------------------------------

#[test]
fn a_rewind_lands_on_the_hash_it_left() {
    // Record first, so the rewind has real input to re-deliver on the way
    // forward. A rewind over an empty log proves only that `Machine::load`
    // works, which `core::state` already proves.
    let (mut machine, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    machine
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic");
    machine.reset(ResetKind::Cold);
    drive(&mut machine, &port, &recorder, &TYPED);
    let log = recorder.log();
    assert!(!log.is_empty());

    // Now replay it under a timeline that keeps a snapshot every 100 ms.
    let (mut replayed, hosts) = apple1(ThreadingMode::Deterministic);
    let replay_port = ports::open(&hosts, CONSOLE).expect("the console port");
    let replay = Arc::new(Recorder::replaying(log));
    replay
        .register(console_channel(), port_sink(&replay_port))
        .expect("a fresh recorder takes channels");
    replayed
        .set_recorder(Arc::clone(&replay))
        .expect("deterministic");
    replayed.reset(ResetKind::Cold);

    let cadence = GlobalTime::from_nanos(100_000_000);
    let mut timeline = Timeline::new(Arc::clone(&replay), cadence);

    let middle = GlobalTime::from_nanos(800_000_000);
    let end = GlobalTime::from_nanos(1_600_000_000);

    timeline.run_until(&mut replayed, middle).expect("a run");
    let at_middle = replayed.now();
    let hash_middle = replayed.state_hash().expect("a hash");
    let cursor_middle = replay.cursor();
    let _ = output(&replay_port);

    timeline.run_until(&mut replayed, end).expect("a run");
    let at_end = replayed.now();
    let hash_end = replayed.state_hash().expect("a hash");
    let output_second_half = output(&replay_port);
    assert_ne!(
        hash_middle, hash_end,
        "the machine went somewhere between the two instants"
    );
    assert!(timeline.keyframes() > 1, "the cadence took snapshots");

    // Back.
    let landed = timeline
        .rewind_to(&mut replayed, at_middle)
        .expect("a rewind");
    assert_eq!(landed, at_middle, "a rewind to a boundary lands on it");
    assert_eq!(
        replayed.state_hash().expect("a hash"),
        hash_middle,
        "the machine is where it was"
    );
    assert_eq!(
        replay.cursor(),
        cursor_middle,
        "and so is the recording's cursor"
    );
    let _ = output(&replay_port);

    // Forward again, over the same stretch of log.
    timeline.run_until(&mut replayed, end).expect("a re-run");
    assert_eq!(replayed.now(), at_end);
    assert_eq!(
        replayed.state_hash().expect("a hash"),
        hash_end,
        "re-running the same stretch reaches the same state"
    );
    assert_eq!(
        output(&replay_port),
        output_second_half,
        "and the guest printed the same thing on the way — which is exactly the \
         host-side state a rewind cannot un-emit"
    );
}

#[test]
fn a_timeline_needs_the_machines_own_recorder() {
    let (mut machine, _hosts) = apple1(ThreadingMode::Deterministic);
    let mine = Arc::new(Recorder::recording());
    let theirs = Arc::new(Recorder::recording());
    machine
        .set_recorder(Arc::clone(&mine))
        .expect("deterministic");

    let mut wrong = Timeline::new(theirs, GlobalTime::from_nanos(1_000_000));
    assert!(
        wrong.run_for(&mut machine, SPAN).is_err(),
        "a timeline pointed at another machine's recorder is a wiring bug, not a \
         determinism bug, and says so"
    );

    machine.take_recorder();
    let mut orphan = Timeline::new(mine, GlobalTime::from_nanos(1_000_000));
    assert!(orphan.run_for(&mut machine, SPAN).is_err());
}

#[test]
fn a_rewind_past_the_oldest_snapshot_is_refused() {
    let (mut machine, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    machine
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic");
    machine.reset(ResetKind::Cold);

    let mut timeline = Timeline::new(recorder, GlobalTime::from_nanos(100_000_000));
    timeline
        .run_until(&mut machine, GlobalTime::from_nanos(500_000_000))
        .expect("a run");
    timeline.forget_before(GlobalTime::from_nanos(300_000_000));

    let err = timeline
        .rewind_to(&mut machine, GlobalTime::from_nanos(50_000_000))
        .expect_err("nothing that old is held any more");
    assert!(
        format!("{err}").contains("oldest snapshot"),
        "the refusal says why: {err}"
    );
}

// ---------------------------------------------------------------------------
// 4. the enforcement, made visible
// ---------------------------------------------------------------------------

#[test]
fn a_board_whose_input_has_no_channel_refuses_to_build() {
    // The test that fails if a device bypasses the seam. The Apple 1's PIA
    // opens `chardev:console` from its own `new(props)`; a recorder that has
    // not registered that channel would have recorded a session missing every
    // keystroke, so the build refuses instead.
    let recorder = Arc::new(Recorder::recording());
    let hosts = Arc::new(HostObjects::new());
    hosts
        .seal(Arc::clone(&recorder))
        .expect("an empty table seals");

    let entry = catalog::machine("apple1").expect("this build ships apple1");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options
        .realize
        .media
        .insert("rom", rsemu::dev::apple1::RSMON.as_slice());
    options.realize.hosts = Arc::clone(&hosts);
    let registry = catalog::registry().expect("a registry");

    let err = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("the board's keyboard has no channel");
    let text = format!("{err}");
    assert!(
        text.contains("chardev:console"),
        "the refusal names the input that bypassed the seam: {text}"
    );
    assert!(text.contains("replay"), "and says why it matters: {text}");
}

#[test]
fn the_same_board_builds_once_the_channel_is_declared() {
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), Arc::new(NullSink) as Arc<dyn InputSink>)
        .expect("a fresh recorder takes channels");
    let hosts = Arc::new(HostObjects::new());
    hosts
        .seal(Arc::clone(&recorder))
        .expect("an empty table seals");

    let (mut machine, _) = apple1_with_hosts(ThreadingMode::Deterministic, hosts);
    machine.reset(ResetKind::Cold);
    machine.run_for(SPAN).expect("a run");
}

#[test]
fn an_unknown_host_kind_is_refused_too() {
    // Not a device this tree has, and that is the point: the check is on the
    // table rather than on a list of known device classes, so the *next* device
    // to grow an input is refused by code that predates it.
    let recorder = Arc::new(Recorder::recording());
    let hosts = HostObjects::new();
    hosts
        .seal(Arc::clone(&recorder))
        .expect("an empty table seals");
    assert!(
        hosts.open(HostKind::new("net"), "eth0", || 0u32).is_err(),
        "a NIC's port is an input like any other"
    );
}

// ---------------------------------------------------------------------------
// 5. the other host
// ---------------------------------------------------------------------------

/// A recording of exactly the session above, frozen as bytes.
///
/// This is the artefact, not a re-derivation of it: 471 bytes that were
/// produced by a run on some machine on some day and have been in the source
/// ever since. Replaying it exercises `InputLog::decode` on a file rather than
/// `InputLog::clone` on a live object, and — the point — it removes the
/// recording run from the replay's causal chain entirely. Every other test here
/// records and replays inside one process, so each would pass on a host that
/// disagreed with every other host, as long as it disagreed with itself
/// consistently.
///
/// It embeds the Apple 1's `MachineShape`, so a change to the board makes this
/// fail with a shape diff. That is the correct failure — a recording of another
/// machine — and regenerating it is the fix: record `TYPED` and print
/// `recorder.log().encode()`.
#[rustfmt::skip]
const FROZEN_SESSION: &[u8] = &[
    0x52, 0x53, 0x45, 0x4d, 0x55, 0x52, 0x50, 0x4c, 0x01, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x63, 0x70, 0x75, 0x0b, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x63, 0x70, 0x75, 0x2e, 0x6d, 0x6f, 0x73, 0x36, 0x35,
    0x30, 0x32, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x69,
    0x61, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x70, 0x70,
    0x6c, 0x65, 0x31, 0x2e, 0x70, 0x69, 0x61, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x72, 0x6f, 0x6d, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x31, 0x2e, 0x72, 0x6f, 0x6d,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x72, 0x61, 0x6d,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x61, 0x6d, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x63, 0x70, 0x75, 0x62, 0x75, 0x73, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x61, 0x6d,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x70, 0x75, 0x62,
    0x75, 0x73, 0x10, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x6d, 0x69, 0x72, 0x72, 0x6f, 0x72, 0x28, 0x70, 0x69, 0x61,
    0x29, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x70, 0x75,
    0x62, 0x75, 0x73, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x6d, 0x6f, 0x6e, 0x69, 0x74, 0x6f, 0x72, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x63, 0x68, 0x61, 0x72, 0x64, 0x65, 0x76, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x6f, 0x6e, 0x73, 0x6f, 0x6c,
    0x65, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x0d, 0x01,
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x63, 0x68, 0x61, 0x72, 0x64, 0x65, 0x76, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x63, 0x6f, 0x6e, 0x73, 0x6f, 0x6c, 0x65, 0x0a, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x30, 0x30, 0x30, 0x2e, 0x45,
    0x30, 0x30, 0x46, 0x0d, 0x01, 0x98, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99,
    0x99, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x68, 0x61, 0x72, 0x64, 0x65, 0x76,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x6f, 0x6e, 0x73,
    0x6f, 0x6c, 0x65, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46,
    0x46, 0x0d, 0x00,
];

/// The state hash the board reaches after replaying [`FROZEN_SESSION`].
///
/// The whole cross-host gate is this constant. A hash compared against another
/// hash computed in the same process proves reproducibility *on this host*; a
/// hash compared against a number in the source proves it against every host
/// that has ever run this test. CI runs `cargo test --all-features` on
/// `ubuntu-latest`, `macos-latest` and `windows-latest` — three operating
/// systems and two instruction sets — so the second host is already there and
/// was simply never asked. It has also been run by hand on a 32-bit target
/// (`cargo test --target i686-unknown-linux-gnu`), which is the interesting
/// direction: `usize` is four bytes there, and a snapshot that had leaked one
/// would not produce this number.
const FROZEN_HASH: u64 = 0xe2c6_ef17_b01e_2bb7;

/// The instant the replay ends at, in raw 2⁻⁶⁴-second units.
///
/// Pinned raw rather than in nanoseconds because it is not a whole number of
/// them: five spans of 400 ms is two virtual seconds less two units, and
/// `as_nanos` rounds that away.
const FROZEN_END: u128 = 36_893_488_147_419_103_230;

/// What the guest prints while [`FROZEN_SESSION`] replays.
///
/// The other half of the artefact, and the half a state hash cannot cover: a
/// machine can reach the right state having said the wrong things on the way.
const FROZEN_OUTPUT: &[u8] =
    b"RSMON\r>0\r0000: 00 00 00 00 00 00 00 00\r>E000.E00F\rE00F: E0 E0 E0 E0 \
      E0 E0 E0 E0\r>FF\r17FF: 17 17 17 17 ";

#[test]
fn a_recording_out_of_a_bug_report_replays_to_a_pinned_state() {
    let log = rsemu::core::record::InputLog::decode(FROZEN_SESSION)
        .expect("the frozen recording still parses");

    let (mut machine, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let replay = Arc::new(Recorder::replaying(log));
    replay
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    machine
        .set_recorder(Arc::clone(&replay))
        .expect("the recording is of this board");
    machine.reset(ResetKind::Cold);

    let (hash, printed) = drive(&mut machine, &port, &replay, &[b"", b"", b"", b"", b""]);

    assert_eq!(
        machine.now().raw(),
        FROZEN_END,
        "the replay ended somewhere else"
    );
    assert_eq!(
        printed, FROZEN_OUTPUT,
        "the guest said something different than it said when this was recorded"
    );
    assert_eq!(
        hash, FROZEN_HASH,
        "this build reached a different machine than the one in the source. \
         Either determinism broke, or this host disagrees with the one that \
         produced the constant — and which of those it is, is exactly what \
         phase 9's gate asks. A deliberate change to the Apple 1, the 6502 or \
         a snapshot format moves this number legitimately: re-record `TYPED` \
         and update FROZEN_SESSION, FROZEN_HASH and FROZEN_OUTPUT together"
    );
}

#[test]
fn this_build_still_produces_the_frozen_recording() {
    // The other direction, and what keeps the frozen artefact from rotting
    // into a fossil nothing writes any more: recording the same session must
    // reproduce those bytes exactly. It also pins the encoder itself as
    // host-invariant — no `usize`, no pointer, no map iteration order.
    let (mut machine, hosts) = apple1(ThreadingMode::Deterministic);
    let port = ports::open(&hosts, CONSOLE).expect("the console port");
    let recorder = Arc::new(Recorder::recording());
    recorder
        .register(console_channel(), port_sink(&port))
        .expect("a fresh recorder takes channels");
    machine
        .set_recorder(Arc::clone(&recorder))
        .expect("deterministic");
    machine.reset(ResetKind::Cold);
    drive(&mut machine, &port, &recorder, &TYPED);

    let encoded = recorder.log().encode().expect("a recording encodes");
    assert_eq!(
        encoded.len(),
        FROZEN_SESSION.len(),
        "the recording changed length: {} bytes now, {} frozen",
        encoded.len(),
        FROZEN_SESSION.len()
    );
    assert!(
        encoded == FROZEN_SESSION,
        "this build writes a different recording of the same session"
    );
}

#[test]
fn a_recording_of_another_board_is_refused_with_a_diff() {
    // What the shape in a recording is *for*. It has been written into every
    // recording since the format existed and nothing read it back, so a
    // recording of one machine replayed into another delivered its input to
    // whatever device answered to the same channel name — a silently different
    // run, which is the one outcome a replay must never produce.
    let mut shape = rsemu::core::state::MachineShape::new();
    shape
        .add_device("cpu", "cpu.z80")
        .expect("a shape takes a device");
    let mut log = rsemu::core::record::InputLog::for_shape(shape);
    log.push(rsemu::core::record::InputEvent {
        at: GlobalTime::from_nanos(1_000_000),
        channel: console_channel(),
        payload: b"x".to_vec(),
    })
    .expect("an empty log takes an event");

    let (mut machine, _hosts) = apple1(ThreadingMode::Deterministic);
    let err = machine
        .set_recorder(Arc::new(Recorder::replaying(log)))
        .expect_err("that recording is not of this board");
    let text = format!("{err}");
    assert!(
        text.contains("cpu.z80"),
        "the refusal is a diff naming what moved, not a boolean: {text}"
    );

    // A log built by hand carries no shape at all, and unknown provenance is
    // not a mismatch: a unit test and a fuzz case must still be replayable.
    let bare = rsemu::core::record::InputLog::new();
    machine
        .set_recorder(Arc::new(Recorder::replaying(bare)))
        .expect("an unshaped log is unknown provenance, not a wrong board");
}

// ---------------------------------------------------------------------------
// parallel
// ---------------------------------------------------------------------------

#[test]
fn a_parallel_machine_refuses_a_recorder() {
    let (mut machine, _hosts) = apple1(ThreadingMode::Parallel);
    let err = machine
        .set_recorder(Arc::new(Recorder::recording()))
        .expect_err("a parallel run cannot be replayed");
    let text = format!("{err}");
    assert!(
        text.contains("inside a round"),
        "the refusal says what is not reproducible, not merely that something is not: {text}"
    );
}
