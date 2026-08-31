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
fn hash_after(name: &str, span_ns: u64, pieces: u32) -> u64 {
    let w = workload::all()
        .into_iter()
        .find(|w| w.name == name)
        .expect("a known workload");
    let mut booted = w.boot();
    let step = span_ns / u64::from(pieces);
    for _ in 0..pieces {
        booted
            .machine
            .run_for(GlobalTime::from_nanos(step))
            .expect("the machine runs");
    }
    booted.machine.state_hash().expect("it hashes")
}

/// The same span, taken whole and taken in pieces.
///
/// If this ever passes for every split, `run_for` is additive and §11.6's
/// claim holds. Today it does not, and the assertion below records *which*
/// way round that is so a future change to the scheduler is noticed.
#[test]
fn one_span_and_many_pieces_are_compared_honestly() {
    const SPAN: u64 = 100_000_000; // 100 ms
    for name in ["nes-ntsc", "gameboy", "apple1"] {
        let whole = hash_after(name, SPAN, 1);
        let halves = hash_after(name, SPAN, 2);
        let tenths = hash_after(name, SPAN, 10);
        println!(
            "{name}: whole {whole:#018x}  halves {halves:#018x}  tenths {tenths:#018x}  \
             additive={}",
            whole == halves && whole == tenths
        );
        // Splitting the *same* way twice must agree. That is determinism, and
        // it is the part that must never regress.
        assert_eq!(
            halves,
            hash_after(name, SPAN, 2),
            "{name}: the same split must be reproducible"
        );

        // A characterisation of today's behaviour, not an endorsement of it.
        // `run_for` is **not** additive: one span and a split span reach
        // different states, because an intermediate deadline truncates a
        // quantum and that is a scheduling boundary the single span never had.
        //
        // Note what the numbers say, though: two pieces and ten pieces agree
        // with each other. It is not that more boundaries drift further — it
        // is that *any* intermediate deadline differs from none, and past that
        // the split shape stops mattering.
        //
        // **When this assertion starts failing, that is good news**: it means
        // `run_for` became additive, and §11.6's promise that a browser session
        // and a native session reach the same state hash became true rather
        // than aspirational. Delete it and fix the roadmap.
        assert_ne!(
            whole, halves,
            "{name}: run_for has become additive — see the note above"
        );
        assert_eq!(
            halves, tenths,
            "{name}: the split shape is not supposed to matter, only its existence"
        );
    }
}
