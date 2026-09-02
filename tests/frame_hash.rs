//! The headless frame-hash regression (`ROADMAP.md` §12).
//!
//! > Machine-level regression: run a machine deterministically for N virtual
//! > seconds and assert the final state hash plus periodic framebuffer hashes.
//! > Cheap, brutal, catches nearly everything.
//!
//! This is that. Every workload in [`workload`] is built from a generated image,
//! advanced a fixed number of *virtual* frames, and hashed — the whole machine
//! through [`Machine::state_hash`], and the picture through [`Surface::hash`],
//! which is the same FNV-1a so the two are comparable artefacts. The expected
//! values live in `tests/goldens/frame-hashes.txt` and are regenerated with
//! `RSEMU_BLESS_FRAME_HASHES=1`.
//!
//! It needs no corpus and no download, so it runs on every commit in CI rather
//! than behind `RSEMU_CONFORMANCE` — which is the point of it. `benches/` is
//! where the same workloads get a stopwatch; this file is what makes any
//! optimisation there safe to believe.
//!
//! # What a failure here means
//!
//! A changed hash is a changed guest-visible behaviour. That may be a fix — the
//! ledger only ever shrinks — but it is never noise: nothing in the run reads
//! the host clock, the environment, or a map with an unstable iteration order.
//! Confirm the new behaviour is the behaviour you meant, re-bless, and say in
//! the commit message which device moved.
//!
//! [`Machine::state_hash`]: rsemu::machine::Machine::state_hash
//! [`Surface::hash`]: rsemu::host::display::Surface::hash

mod workload;

use std::collections::BTreeMap;

use workload::{Golden, Workload};

/// Set to re-record `tests/goldens/frame-hashes.txt` instead of asserting.
const BLESS: &str = "RSEMU_BLESS_FRAME_HASHES";

/// Run every workload this build has and compare against the golden file.
#[test]
fn the_committed_workloads_hash_to_their_recorded_values() {
    assert!(
        workload::nes_rom_override().is_none(),
        "{} is set, so the NES workload is running a cartridge this repository \
         has never seen. The recorded hashes describe the generated ROM and \
         cannot apply. Unset it to run the regression; it is meant for \
         `cargo bench`, where it measures the gate as written.",
        workload::NES_ROM_ENV
    );

    let workloads = workload::all();
    if workloads.is_empty() {
        // A build with no machine feature. Not a failure: it is a build with
        // nothing to regress, and the feature sweep runs one machine at a time.
        eprintln!("no machine features enabled; nothing to run");
        return;
    }

    let mut fresh: BTreeMap<String, Vec<Golden>> = BTreeMap::new();
    for w in &workloads {
        fresh.insert(w.name.to_owned(), record(w));
    }

    if std::env::var(BLESS).is_ok_and(|v| v != "0") {
        workload::bless(&fresh);
        return;
    }

    let expected = workload::goldens();
    let mut wrong = Vec::new();
    for w in &workloads {
        let got = &fresh[w.name];
        let Some(want) = expected.get(w.name) else {
            wrong.push(format!("`{}` has no recorded hashes at all", w.name));
            continue;
        };
        if got.len() != want.len() {
            wrong.push(format!(
                "`{}`: recorded {} checkpoints, this run produced {}",
                w.name,
                want.len(),
                got.len()
            ));
            continue;
        }
        for (got, want) in got.iter().zip(want) {
            if got != want {
                wrong.push(format!(
                    "`{}` after {} frames:\n     state  want {:#018x}  got {:#018x}\n     frame  want {}  got {}",
                    w.name,
                    got.frame,
                    want.state,
                    got.state,
                    show(want.frame_hash),
                    show(got.frame_hash),
                ));
            }
        }
    }

    assert!(
        wrong.is_empty(),
        "{} workload checkpoint(s) changed:\n\n{}\n\n\
         A hash here is guest-visible behaviour. If the change was intended, \
         re-record with\n\n    {BLESS}=1 cargo test --all-features --test frame_hash\n\n\
         and name the device that moved in the commit message.",
        wrong.len(),
        wrong.join("\n")
    );
}

/// The same workload, run twice in one process, produces the same hashes.
///
/// The golden file cannot show this: it would pass just as well if the run were
/// reproducible only because the whole thing were constant. This is the
/// property the benchmark depends on — the emulated work is identical run to
/// run and only the stopwatch varies — so it is asserted directly.
#[test]
fn a_workload_is_reproducible_within_one_process() {
    for w in workload::all() {
        // A quarter of the regression's length: reproducibility either holds
        // from the first checkpoint or it does not hold at all.
        let frames = (w.frames / 4).max(1);
        let first = hashes(&w, frames);
        let second = hashes(&w, frames);
        assert!(!first.is_empty(), "`{}` produced no hashes", w.name);
        assert_eq!(
            first, second,
            "`{}` produced different hashes on two identical runs",
            w.name
        );
    }
}

/// A machine with a display draws something, and keeps drawing something new.
///
/// A blank screen has a perfectly stable hash, so the golden file alone would
/// happily bless a ROM that stopped rendering on the day someone broke the
/// fetch path. This is the guard against that: the picture has to be busy, and
/// consecutive frames have to differ.
#[test]
fn a_machine_with_a_picture_actually_draws_one() {
    for w in workload::all() {
        // Boot once to ask whether there is anything to look at, and stop there
        // if not: running a display-less machine to completion to discover it
        // has no display is most of this test's cost for none of its value.
        if w.boot().capture.is_none() {
            continue;
        }

        let mut colours = 0usize;
        let mut hashes: Vec<u64> = Vec::new();
        let mut has_display = false;
        // Sampled at the checkpoint cadence rather than every frame: capturing
        // a surface and counting its colours costs more than emulating the
        // frame that produced it, and a picture that is busy at frame 30 and
        // frame 60 is a picture.
        w.run(w.frames, |frame, booted| {
            if let Some(capture) = booted.capture.as_mut() {
                has_display = true;
                if frame % w.checkpoint_every == 0 {
                    colours = colours.max(capture.distinct_colours());
                    hashes.push(capture.hash());
                }
            }
            true
        });
        if !has_display {
            continue;
        }
        assert!(
            colours >= w.min_colours,
            "`{}` drew a frame with only {colours} distinct colours, fewer than \
             the {} its hardware can produce; a blank picture would pass the \
             hash check and mean nothing",
            w.name,
            w.min_colours
        );
        let mut distinct = hashes.clone();
        distinct.sort_unstable();
        distinct.dedup();
        assert!(
            distinct.len() > 1,
            "`{}` drew {} identical frames in a row; the guest is meant to \
             scroll, so the picture must change",
            w.name,
            hashes.len()
        );
    }
}

/// A snapshot taken mid-run resumes to the same place the uninterrupted run
/// reaches.
///
/// The frame-hash regression pins *a* trajectory; this pins that the trajectory
/// is a property of the machine's recorded state rather than of how it was
/// reached, which is what every later optimisation — a block cache, a JIT, a
/// rewind buffer — is going to lean on. Run half, save, run the rest, remember
/// where you landed; then take a machine that has only ever been reset, hand it
/// the snapshot, run the same rest, and land in the same place.
#[test]
fn a_snapshot_taken_mid_run_resumes_to_the_same_hashes() {
    for w in workload::all() {
        // A quarter of the regression's length on each leg. Long enough that
        // every device has state worth carrying across the snapshot, short
        // enough that the whole file still runs on every commit.
        let half = (w.frames / 4).max(1);

        let mut booted = w.boot();
        booted.step_many(half);
        let snapshot = booted.machine.save().expect("the machine saves");
        booted.step_many(half);
        let straight_through = booted.machine.state_hash().expect("a state hash");
        drop(booted);

        let mut resumed = w.boot();
        resumed.machine.load(&snapshot).expect("the snapshot loads");
        resumed.step_many(half);
        assert_eq!(
            straight_through,
            resumed.machine.state_hash().expect("a state hash"),
            "`{}`: resuming from a snapshot half way through did not reach the \
             same state as running straight through",
            w.name
        );
    }
}

// ---------------------------------------------------------------------------

/// Run a workload to its recorded length, sampling at its checkpoint interval
/// and always at the end.
fn record(w: &Workload) -> Vec<Golden> {
    let mut out = Vec::new();
    w.run(w.frames, |frame, booted| {
        if frame % w.checkpoint_every == 0 || frame == w.frames {
            out.push(Golden {
                frame,
                state: booted.machine.state_hash().expect("a state hash"),
                frame_hash: booted.capture.as_mut().map(workload::Capture::hash),
            });
        }
        true
    });
    out
}

/// State hashes at the workload's checkpoint cadence, for the reproducibility
/// check.
///
/// Not every frame: `state_hash` walks the whole machine, guest RAM included,
/// so hashing per frame would make this test cost more than every emulated
/// instruction in it — and it would not catch anything the cadence misses,
/// since two runs that diverge stay diverged.
fn hashes(w: &Workload, frames: u32) -> Vec<u64> {
    let every = w.checkpoint_every.min(frames).max(1);
    let mut out = Vec::new();
    w.run(frames, |frame, booted| {
        if frame % every == 0 || frame == frames {
            out.push(booted.machine.state_hash().expect("a state hash"));
        }
        true
    });
    out
}

fn show(hash: Option<u64>) -> String {
    match hash {
        Some(h) => format!("{h:#018x}"),
        None => String::from("-"),
    }
}
