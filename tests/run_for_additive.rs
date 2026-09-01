//! Is [`Machine::run_for`] additive?
//!
//! `ROADMAP.md` §11.6 claims a browser session and a native session reach the
//! same state hash. The browser advances **frame by frame**; `rsemu run`
//! advances **one span**. That claim is therefore only true if running for a
//! span and running for the same span in pieces reach the same state — which is
//! what this file measures rather than assumes.
//!
//! It is not obviously true. `Scheduler::run_quantum_deterministic` picks
//! `target = min(now + quantum, limit, next_event)`, so a deadline that falls
//! mid-quantum *truncates* that quantum. Every intermediate deadline is
//! therefore an extra scheduling boundary that the single span does not have,
//! and a boundary changes how far each runnable gets before the next one runs.

mod workload;

use rsemu::core::clock::GlobalTime;

/// Run one workload for `span` in `pieces` equal steps and return its hash.
///
/// `None` when this build has no such workload, which is an ordinary
/// `--no-default-features` build rather than a failure.
fn hash_after(name: &str, span_ns: u64, pieces: u32) -> Option<u64> {
    let w = workload::all().into_iter().find(|w| w.name == name)?;
    let mut booted = w.boot();
    let step = span_ns / u64::from(pieces);
    for _ in 0..pieces {
        booted
            .machine
            .run_for(GlobalTime::from_nanos(step))
            .expect("the machine runs");
    }
    Some(booted.machine.state_hash().expect("it hashes"))
}

/// The same span, taken whole and taken in pieces.
///
/// If this ever passes for every split, `run_for` is additive and §11.6's
/// claim holds. Today it does not, and the assertion below records *which*
/// way round that is so a future change to the scheduler is noticed.
#[test]
fn one_span_and_many_pieces_are_compared_honestly() {
    const SPAN: u64 = 100_000_000; // 100 ms

    // Every workload this build has, rather than a named list. A hard-coded
    // list silently stops covering the workload added after it was written —
    // which is exactly what happened here: `riscv-virt` was missing, and the
    // guard below caught it in the one feature set where it was the *only*
    // workload.
    let names: Vec<&'static str> = workload::all().into_iter().map(|w| w.name).collect();
    if names.is_empty() {
        eprintln!("no machine features enabled; nothing to compare");
        return;
    }

    let mut non_additive = 0usize;
    for name in &names {
        let whole = hash_after(name, SPAN, 1).expect("a workload this build has");
        let halves = hash_after(name, SPAN, 2).expect("the same workload again");
        let tenths = hash_after(name, SPAN, 10).expect("the same workload again");
        println!(
            "{name}: whole {whole:#018x}  halves {halves:#018x}  tenths {tenths:#018x}  \
             additive={}",
            whole == halves && whole == tenths
        );

        // Splitting the *same* way twice must agree. That is determinism, and
        // it is the part that must never regress.
        assert_eq!(
            Some(halves),
            hash_after(name, SPAN, 2),
            "{name}: the same split must be reproducible"
        );

        // Whether the split *shape* matters is workload-dependent, and that is
        // itself a finding. On `nes-ntsc`, `gameboy` and `apple1` two pieces
        // and ten reach the same hash; on `riscv-virt` they do not. An earlier
        // version of this file asserted they always agree, generalising from
        // the three that did — so it is recorded here and not asserted.
        if halves != tenths {
            println!("  ({name}: the split shape matters here, not just its existence)");
        }

        if whole != halves {
            non_additive += 1;
        }
    }

    // A characterisation of today's behaviour, not an endorsement of it.
    // `run_for` is **not** additive: an intermediate deadline truncates a
    // quantum, and that is a scheduling boundary the single span never had.
    //
    // **When this assertion starts failing, that is good news**: it means
    // `run_for` became additive and §11.6's promise that a browser session and
    // a native session reach the same state hash became true rather than
    // aspirational. Delete it and fix the roadmap.
    //
    // Counted rather than asserted per workload, because a workload with
    // nothing scheduled has no boundary to be moved and would be additive for
    // an uninteresting reason.
    assert!(
        non_additive > 0,
        "every workload is now additive — run_for may have been fixed; see the note above"
    );
}
