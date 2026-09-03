#![no_main]
//! The *second* record/replay seam's file, held to the same contract as the
//! first (`fuzz_targets/record_log.rs`).
//!
//! `usermode::journal` answers a level-3 guest's syscalls from a log, and that
//! log is a file a consumer hands us — out of a bug report, from an older
//! build, possibly from a different program entirely. Until recently it had no
//! magic and no version, which meant `Journal::load` could not even tell it was
//! looking at the wrong file; now it is framed like a recording and so it earns
//! a target like a recording's.
//!
//! Three properties, in increasing strength:
//!
//! 1. **No panic.** `Journal::decode` over arbitrary bytes, then every
//!    accessor, then a full replay against whatever came back.
//! 2. **Self-consistency.** The counts agree, and the snapshot form
//!    (`save`/`load`, which carries a cursor the recording does not) round
//!    trips through the recording form it embeds.
//! 3. **Canonical form.** A journal recording has exactly one valid encoding,
//!    so re-encoding what was decoded must reproduce the fuzzer's own bytes —
//!    the property that makes "the same recording produces the same run"
//!    checkable by comparison rather than by argument.
//!
//! The replay itself is driven too, because that path compares attacker-chosen
//! tags and instants and must terminate on every one of them: a divergence is
//! an `Err`, never a hang and never a wrong answer handed back to a guest.

use libfuzzer_sys::fuzz_target;

use rsemu::core::clock::GlobalTime;
use rsemu::core::state::{SliceSource, Source};
use rsemu::usermode::{Answer, Journal, JournalMode, Tag};

fuzz_target!(|data: &[u8]| {
    // Anything that fails to parse has already proved the only thing this
    // target can prove about it: it did not panic.
    let Ok(journal) = Journal::decode(data) else {
        return;
    };

    let len = journal.len();
    assert_eq!(journal.is_empty(), len == 0);
    assert_eq!(
        journal.mode(),
        JournalMode::Live,
        "a decoded recording is a log, not a running replay"
    );
    assert_eq!(
        journal.remaining(),
        len,
        "and nothing in it has been replayed yet"
    );

    // Canonical form: re-encoding what was decoded reproduces the input.
    let reencoded = journal.encode().expect("a decoded journal re-encodes");
    assert!(
        reencoded == data,
        "a journal recording has exactly one valid encoding, but decoding {} \
         bytes and re-encoding produced {} different bytes",
        data.len(),
        reencoded.len()
    );

    // The snapshot form embeds the recording whole, so a round trip through it
    // must reproduce both the log and the cursor.
    let mut snapshot: Vec<u8> = Vec::new();
    journal.save(&mut snapshot).expect("a journal saves");
    let restored = Journal::new();
    let mut source = SliceSource::new(&snapshot);
    restored
        .load(&mut source)
        .expect("what save wrote, load reads");
    assert_eq!(source.remaining(), 0, "and reads all of it");
    assert_eq!(restored.len(), len);
    assert_eq!(
        restored.encode().expect("re-encodes"),
        reencoded,
        "the snapshot form carries the recording form unchanged"
    );

    // Replay every answer back out. The questions are the ones the recording
    // itself claims were asked, so a well-formed log answers all of them and a
    // log the fuzzer bent answers none — and either way this terminates.
    journal.set_mode(JournalMode::Replay);
    let mut served = 0usize;
    for _ in 0..len {
        // The tag and instant of the entry about to be served are not exposed,
        // so ask with a fixed pair: a match serves, a mismatch diverges, and
        // both must be an ordinary return rather than a panic.
        match journal.ask(GlobalTime::ZERO, Tag(0), Answer::default) {
            Ok(_) => served += 1,
            Err(_) => break,
        }
    }
    assert!(served <= len, "replay handed out more than it holds");
    assert_eq!(journal.remaining(), len - served);

    // Rewinding a replay is `set_mode` on this seam, and it must put the
    // cursor back exactly.
    journal.set_mode(JournalMode::Replay);
    assert_eq!(journal.remaining(), len, "a replay rewinds to the start");

    // Past the end is an error, never a panic and never a stale answer.
    journal.clear();
    assert!(journal.is_empty());
    journal.set_mode(JournalMode::Replay);
    assert!(
        journal.ask(GlobalTime::MAX, Tag(u32::MAX), Answer::default).is_err(),
        "an exhausted log says so"
    );
});
