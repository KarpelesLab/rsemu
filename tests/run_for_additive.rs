//! Is [`Machine::run_for`] additive?
//!
//! `ROADMAP.md` §11.6 claims a browser session and a native session reach the
//! same state hash. The browser advances **frame by frame**; `rsemu run`
//! advances **one span**. That claim is therefore only true if running for a
//! span and running for the same span in pieces reach the same state — which is
//! what this file measures rather than assumes.
//!
//! It was not always true. A round used to end at
//! `min(now + quantum, limit, next_event)`, so an intermediate deadline
//! truncated one — an extra scheduling boundary, which advanced the round-robin
//! cursor an extra time and handed every runnable a budget the unsliced run
//! never handed out. Both effects were permanent. A round now ends at an
//! instant that depends on virtual time and the machine's state and on nothing
//! else, and a deadline inside one declines the round instead of splitting it
//! (`core::sched::Scheduler::run_quantum_until`).

// Every machine named below is asked for by name, so all three have to be in
// the build. Without this the file fails under any narrower feature set with
// "a known workload" rather than skipping, which is not a measurement.
#![cfg(all(
    feature = "machine-nes",
    feature = "machine-gameboy",
    feature = "machine-apple1"
))]

mod workload;

use rsemu::core::clock::GlobalTime;

/// About 100 ms, rounded down to a raw span every split below divides exactly.
///
/// Exactly, because the pieces have to sum to the whole or the two runs have
/// different *deadlines* and no scheduler could reconcile them: `GlobalTime`
/// counts 2⁻⁶⁴ seconds and rounds down, so `2 × from_nanos(50 ms)` is one unit
/// short of `from_nanos(100 ms)`, and `now` is architectural state. That is
/// arithmetic, not scheduling, and an earlier version of this file measured it
/// by accident and reported it as non-additivity.
const SPAN: GlobalTime =
    GlobalTime::from_raw(GlobalTime::from_nanos(100_000_000).raw() / SPLIT_LCM * SPLIT_LCM);

/// The least common multiple of every split in [`SPLITS`].
const SPLIT_LCM: u128 = 10;

/// How the span is cut up. One piece is the unsliced run.
const SPLITS: [u32; 3] = [1, 2, 10];

/// Run one workload for [`SPAN`] in `pieces` equal steps and return its hash.
///
/// `None` when this build has no such workload, which is an ordinary
/// `--no-default-features` build rather than a failure.
fn hash_after(name: &str, pieces: u32) -> Option<u64> {
    let w = workload::all().into_iter().find(|w| w.name == name)?;
    let mut booted = w.boot();
    let step = GlobalTime::from_raw(SPAN.raw() / u128::from(pieces));
    for _ in 0..pieces {
        booted.machine.run_for(step).expect("the machine runs");
    }
    Some(booted.machine.state_hash().expect("it hashes"))
}

/// The same span, taken whole and taken in pieces, reaches the same state.
#[test]
fn one_span_and_many_pieces_reach_the_same_state() {
    // Every workload this build has, rather than a named list. A hard-coded
    // list silently stops covering the workload added after it was written —
    // which is exactly what happened here: `riscv-virt` was missing, and it
    // turned out to be the only one of the four that a shifted round-robin
    // cursor was not the whole story for.
    let names: Vec<&'static str> = workload::all().into_iter().map(|w| w.name).collect();
    if names.is_empty() {
        eprintln!("no machine features enabled; nothing to compare");
        return;
    }

    for name in &names {
        let whole = hash_after(name, 1).expect("a workload this build has");
        let hashes: Vec<(u32, u64)> = SPLITS
            .iter()
            .map(|p| (*p, hash_after(name, *p).expect("the same workload again")))
            .collect();
        let shown: Vec<String> = hashes
            .iter()
            .map(|(p, h)| format!("{p}:{h:#018x}"))
            .collect();
        println!("{name}: {}", shown.join("  "));

        for (pieces, hash) in hashes {
            assert_eq!(
                whole, hash,
                "{name}: {pieces} pieces reach a different state from one span; \
                 `run_for` is not additive"
            );
        }
    }
}

/// Splitting the *same* way twice must agree.
///
/// That is determinism rather than additivity, and it is the half that must
/// never regress: a fix that made every split agree by making the run depend on
/// something new would pass the test above and fail this one.
#[test]
fn the_same_split_twice_reaches_the_same_state() {
    for w in workload::all() {
        assert_eq!(
            hash_after(w.name, 2),
            hash_after(w.name, 2),
            "{}: the same split must be reproducible",
            w.name
        );
    }
}
