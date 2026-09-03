//! The record/replay seam: the one door non-determinism comes through.
//!
//! `ROADMAP.md` §0 says any non-deterministic input crossing into the machine
//! goes through the record/replay seam or it is a determinism bug, and phase 5b
//! says a syscall's result *is* exactly such an input. At level 1 that rule is
//! easy to hold because the inputs are few and obvious — a key, a packet, a
//! host clock read. At level 3 it is the hard part of the whole design, because
//! the guest's every interaction with the outside world is a syscall and a
//! syscall's answer is whatever the host felt like saying.
//!
//! # The funnel
//!
//! Every value that is **not a function of guest state** is obtained by calling
//! [`Journal::ask`] with a closure that would compute it. What the journal does
//! with the closure is the mode:
//!
//! | Mode | The closure | The answer |
//! | --- | --- | --- |
//! | [`Live`](JournalMode::Live) | runs | is returned, and forgotten |
//! | [`Record`](JournalMode::Record) | runs | is returned, and written down |
//! | [`Replay`](JournalMode::Replay) | **never runs** | comes from the log |
//!
//! That is the whole mechanism, and its value is in what it makes checkable: a
//! consumer that reaches the host anywhere else has a bug that shows up as a
//! replay that diverges, rather than as a mystery a year later. In replay a
//! mismatched tag or a mismatched instant is reported as
//! [`Error::State`] immediately — an early,
//! loud failure at the point the two runs first differed, which is the only
//! useful place to fail.
//!
//! # What deliberately does *not* come through here
//!
//! **Time.** A level-3 run's clock is [`GuestClock`](super::GuestClock), which
//! is a function of executed ticks, so a clock read is already deterministic
//! and there is nothing to record. Eliminating a non-deterministic input beats
//! journalling it: the log stays small, and a replay does not go wrong when
//! the consumer adds a `clock_gettime` fast path.
//!
//! **The schedule.** With [`ThreadSet`](super::ThreadSet) the interleaving is a
//! function of the program too. This is not incidental — a journal of answers
//! is replayable only if the *questions* arrive in the same order, so the
//! deterministic scheduler is a precondition for this module working at all.
//!
//! # Shape of an answer
//!
//! An [`Answer`] is a scalar and some bytes, because that is what every
//! external result actually is: a `read` is a count and a buffer, a
//! `getrandom` is a count and a buffer, a `stat` is zero and a structure, a
//! `connect` is a return code and nothing. The consumer decides what the
//! scalar means — it does not have to be an errno, and this module does not
//! know that Linux exists.
//!
//! # Why this is not [`core::record`](crate::core::record), reviewed
//!
//! Two modules in one tree both call themselves "the record/replay seam", and
//! that is worth being sure about rather than merely comfortable with. The
//! review was done; the split stands; here is the argument from this side, and
//! `core::record` carries the other half.
//!
//! Neither subsumes the other, because **each one's key is the other's
//! check**:
//!
//! | | `core::record` | here |
//! | --- | --- | --- |
//! | key | `(instant, channel)` | position in the sequence |
//! | the other field | a channel with no sink is skipped, and the run is still faithful | a tag that does not match is a divergence, reported at once |
//! | who initiates | the host, whenever it likes | the guest, mid-instruction |
//! | delivery | pushed into a sink at a round boundary | returned to the caller at the call site |
//! | a payload's size | bounded — a keystroke, a frame | whatever the guest asked to `read` |
//! | the cursor | derived: seek the log by instant | architectural: nothing else names the position |
//!
//! The tempting unification is that `Recorder::deliver` is itself a pull — the
//! machine asks "what arrived?" once per round — so input could be journalled
//! as one question per round boundary. It could, and it would be worse in three
//! measurable ways. A recorded Apple 1 session is three events and would become
//! one entry per scheduling round, millions of them, unless empty answers were
//! elided — which is instant-keying, reintroduced. A round delivers *N*
//! payloads across *M* channels, so the channel dimension would have to be
//! re-encoded inside an answer's bytes. And a journal cannot skip a question:
//! `rsemu replay` on a headless host with no terminal attached is a thing
//! people do, and `core::record` supports it precisely because a missing sink
//! costs nothing.
//!
//! The converse is worse still. A syscall answer keyed on an instant needs two
//! syscalls never to share one, which no clock design promises; and a recorder
//! has no way to *return* a value to a blocked caller, only to push into a
//! sink.
//!
//! So the mechanisms stay two. What is **not** allowed to stay two is the file
//! format, and that is what changed at this review:
//! [`Journal::encode`] now writes a magic, a version and tagged entries exactly
//! as [`InputLog::encode`](crate::core::record::InputLog::encode) does, and
//! [`Journal::decode`] is held to the same parser contract with a fuzz target
//! of its own. A session that has both ends up with two sections of one
//! vocabulary rather than two file formats, which was always the stated goal
//! and until now was only a claim.
//!
//! One thing a recording carries and this does not: a
//! [`MachineShape`](crate::core::state::MachineShape). A recording needs one
//! because a wrong board *silently accepts* input on a channel whose name
//! happens to match. A journal needs none because a wrong program diverges at
//! the first question whose tag or instant differs, which is a better check
//! than a fingerprint and comes for free.

use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::error::{Error, Result};
use crate::core::state::{Sink, SliceSource, Source};
use crate::core::sync::{self, LockRank};

/// Magic at the start of every journal recording.
///
/// Deliberately distinct from `core::record`'s `RSEMURPL`: the two seams write
/// two sections of one vocabulary, not two spellings of one section, and a
/// reader handed the wrong one should say so rather than mis-parse it.
const MAGIC: [u8; 8] = *b"RSEMUJRN";

/// The journal recording's container format version.
///
/// Independent of both `core::state`'s `FORMAT_VERSION` and `core::record`'s
/// [`LOG_FORMAT_VERSION`](crate::core::record::LOG_FORMAT_VERSION), because the
/// three framings change for three different reasons.
pub const JOURNAL_FORMAT_VERSION: u32 = 1;

/// Tag byte introducing one more answer.
const TAG_ANSWER: u8 = 0x01;

/// Tag byte marking the end of the answer list.
const TAG_END: u8 = 0x00;

/// What a journal does with the answers that pass through it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JournalMode {
    /// Ask the host and keep nothing. The default, and what an ordinary run
    /// does.
    #[default]
    Live,
    /// Ask the host and write the answer down.
    Record,
    /// Do not ask the host; take the answer from what was written down.
    Replay,
}

/// What kind of question was asked.
///
/// Consumer-defined: rsemu does not know what a syscall is, so it cannot name
/// the questions. A Linux-shaped consumer will put its syscall number here; a
/// different one will put something else. All this module does with a tag is
/// **check it** — a replay whose tags do not line up has diverged, and saying
/// so is worth more than the tag's meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[repr(transparent)]
pub struct Tag(pub u32);

/// One answer from outside the machine.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Answer {
    /// The scalar part — a count, a status, a handle. Meaning is the
    /// consumer's.
    pub value: u64,
    /// The bytes part, if the question had one: what a `read` read, what a
    /// random source produced, what a directory listing said.
    pub bytes: Vec<u8>,
}

impl Answer {
    /// An answer that is only a scalar.
    #[must_use]
    pub const fn value(value: u64) -> Answer {
        Answer {
            value,
            bytes: Vec::new(),
        }
    }

    /// An answer that is a scalar and some bytes.
    #[must_use]
    pub const fn with_bytes(value: u64, bytes: Vec<u8>) -> Answer {
        Answer { value, bytes }
    }
}

/// One logged answer, with what it has to match on replay.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    /// The virtual instant the question was asked at. Recorded because a
    /// divergence usually shows up here first — the same question asked a
    /// million ticks late means the guest took a different path.
    at: u128,
    tag: u32,
    answer: Answer,
}

/// The log, and where replay has got to.
#[derive(Debug, Default)]
struct Inner {
    mode: JournalMode,
    entries: Vec<Entry>,
    /// The next entry replay will hand out.
    cursor: usize,
}

/// The record/replay log for a level-3 run.
///
/// # Locking
///
/// One [`sync::Mutex`] at [`LockRank::LEAF`] — the rank nothing nests under —
/// because a journal is asked a question from inside whatever the consumer was
/// already doing, and must never be the reason two of its locks have to be
/// ordered. It is the last thing acquired and the first released.
#[derive(Debug, Default)]
pub struct Journal {
    inner: sync::Mutex<Inner>,
}

impl Journal {
    /// A journal in [`JournalMode::Live`]: nothing is recorded.
    #[must_use]
    pub fn new() -> Journal {
        Journal::with_mode(JournalMode::Live)
    }

    /// A journal in `mode`.
    #[must_use]
    pub fn with_mode(mode: JournalMode) -> Journal {
        Journal {
            inner: sync::Mutex::with_rank(
                LockRank::LEAF,
                Inner {
                    mode,
                    entries: Vec::new(),
                    cursor: 0,
                },
            ),
        }
    }

    /// What this journal is doing.
    #[must_use]
    pub fn mode(&self) -> JournalMode {
        self.inner.lock().mode
    }

    /// Change what it does.
    ///
    /// Switching to [`JournalMode::Replay`] rewinds the cursor, so a recorded
    /// log can be replayed without a round trip through bytes.
    pub fn set_mode(&self, mode: JournalMode) {
        let mut inner = self.inner.lock();
        inner.mode = mode;
        if mode == JournalMode::Replay {
            inner.cursor = 0;
        }
    }

    /// How many answers have been recorded.
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.lock().entries.len()
    }

    /// Whether nothing has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.lock().entries.is_empty()
    }

    /// How many recorded answers replay has not reached yet.
    #[must_use]
    pub fn remaining(&self) -> usize {
        let inner = self.inner.lock();
        inner.entries.len().saturating_sub(inner.cursor)
    }

    /// Forget everything and rewind.
    pub fn clear(&self) {
        let mut inner = self.inner.lock();
        inner.entries.clear();
        inner.cursor = 0;
    }

    /// Obtain an external answer — the only sanctioned way to.
    ///
    /// `at` is the virtual instant the question is asked at, which the caller
    /// takes from [`GuestClock::now`](super::GuestClock::now); `tag` says what
    /// kind of question it is. In [`Replay`](JournalMode::Replay) the closure
    /// is **not called**, which is the property that makes replay work with no
    /// host at all — a recorded run replays in a browser, on a different
    /// operating system, with the files deleted.
    ///
    /// # Errors
    ///
    /// In [`Replay`](JournalMode::Replay): [`Error::State`] if the log has run
    /// out, or if the next entry's tag or instant does not match. That is a
    /// divergence, and it is reported where it happened rather than allowed to
    /// become wrong output later.
    pub fn ask<F>(&self, at: GlobalTime, tag: Tag, f: F) -> Result<Answer>
    where
        F: FnOnce() -> Answer,
    {
        // The mode is read, and the lock released, before the closure runs:
        // the closure is host I/O and may take as long as it likes, and
        // nothing outward happens under this lock (§4.7).
        let mode = self.inner.lock().mode;
        match mode {
            JournalMode::Live => Ok(f()),
            JournalMode::Record => {
                let answer = f();
                let mut inner = self.inner.lock();
                inner.entries.push(Entry {
                    at: at.raw(),
                    tag: tag.0,
                    answer: answer.clone(),
                });
                Ok(answer)
            }
            JournalMode::Replay => {
                let mut inner = self.inner.lock();
                let cursor = inner.cursor;
                let Some(entry) = inner.entries.get(cursor).cloned() else {
                    return Err(diverged(alloc::format!(
                        "the recording ended after {cursor} answer(s), but the guest asked \
                         a {tag:?} at {}ns",
                        at.as_nanos()
                    )));
                };
                if entry.tag != tag.0 {
                    return Err(diverged(alloc::format!(
                        "answer {cursor} was recorded for tag {} and the guest asked tag {}",
                        entry.tag,
                        tag.0
                    )));
                }
                if entry.at != at.raw() {
                    return Err(diverged(alloc::format!(
                        "answer {cursor} was recorded at {}ns and the guest asked at {}ns",
                        GlobalTime::from_raw(entry.at).as_nanos(),
                        at.as_nanos()
                    )));
                }
                inner.cursor = cursor + 1;
                Ok(entry.answer)
            }
        }
    }

    /// Encode the recording: the trace file `rsemu record` would produce and
    /// `rsemu replay` would take (§2's binary surface).
    ///
    /// Framed exactly as [`InputLog::encode`](crate::core::record::InputLog::encode)
    /// is — magic, a format version of its own, tagged entries, an end tag —
    /// so the two seams write two *sections* rather than two file formats. See
    /// the module documentation for what is deliberately different.
    ///
    /// Neither the mode nor the cursor is written. Both are what you are
    /// *doing* with a recording rather than part of one, and a cursor baked
    /// into a distributable file would make a replay start wherever the
    /// recorder happened to stop. [`Journal::save`] is the other form, for a
    /// consumer snapshotting a run in progress.
    ///
    /// # Errors
    ///
    /// Only if a sink write fails, which for the `Vec<u8>` used here it cannot.
    pub fn encode(&self) -> Result<Vec<u8>> {
        let inner = self.inner.lock();
        let mut out: Vec<u8> = Vec::new();
        out.write_all(&MAGIC)?;
        out.write_u32(JOURNAL_FORMAT_VERSION)?;
        for entry in &inner.entries {
            out.write_u8(TAG_ANSWER)?;
            out.write_u128(entry.at)?;
            out.write_u32(entry.tag)?;
            out.write_u64(entry.answer.value)?;
            out.write_bytes(&entry.answer.bytes)?;
        }
        out.write_u8(TAG_END)?;
        Ok(out)
    }

    /// Decode a recording into a fresh journal, in [`JournalMode::Live`] with
    /// the cursor at the start.
    ///
    /// A parser on untrusted input, held to `core::state`'s contract: it never
    /// panics, never trusts a length it has not compared against the bytes
    /// remaining, never allocates against a claimed count, and rejects anything
    /// that is not the one canonical form [`Journal::encode`] writes.
    ///
    /// It does **not** check that instants are non-descending, and that is a
    /// difference from a recording rather than an omission: the key here is the
    /// *order the questions arrive in*, the instant is a check against it, and
    /// nothing in the design says two threads must ask in clock order.
    ///
    /// # Errors
    ///
    /// [`Error::State`] naming what was expected: a bad magic, an unsupported
    /// format version, an unknown tag, or trailing bytes.
    pub fn decode(bytes: &[u8]) -> Result<Journal> {
        let mut src = SliceSource::new(bytes);
        let magic = src.take(MAGIC.len())?;
        if magic != MAGIC {
            return Err(Error::State(alloc::format!(
                "not a journal recording: magic {magic:02x?}, expected {MAGIC:02x?}"
            )));
        }
        let format = src.read_u32()?;
        if format != JOURNAL_FORMAT_VERSION {
            return Err(Error::State(alloc::format!(
                "journal format version {format} (this build reads {JOURNAL_FORMAT_VERSION})"
            )));
        }
        let mut entries = Vec::new();
        loop {
            match src.read_u8()? {
                TAG_END => break,
                TAG_ANSWER => {
                    let at = src.read_u128()?;
                    let tag = src.read_u32()?;
                    let value = src.read_u64()?;
                    // Borrowed and bounds-checked against the input before it
                    // is copied, so a claimed length cannot become a large
                    // allocation. There is no size *limit* on an answer, and
                    // that too is a difference from a recording: a keystroke
                    // and an Ethernet frame have a natural ceiling, and what a
                    // guest asked to `read` does not.
                    let bytes = src.read_bytes()?.to_vec();
                    entries.push(Entry {
                        at,
                        tag,
                        answer: Answer { value, bytes },
                    });
                }
                tag => {
                    return Err(Error::State(alloc::format!(
                        "unknown tag 0x{tag:02x} in a journal recording (expected 0x00 or 0x01)"
                    )));
                }
            }
        }
        if src.remaining() != 0 {
            return Err(Error::State(alloc::format!(
                "{} trailing byte(s) after the end of a journal recording",
                src.remaining()
            )));
        }
        let journal = Journal::new();
        journal.inner.lock().entries = entries;
        Ok(journal)
    }

    /// Write the log **and where replay has got to**, for a consumer
    /// snapshotting a run in progress.
    ///
    /// The other half of the split [`Journal::encode`] describes: this is state
    /// rather than a file, so it carries the cursor. It embeds the encoded
    /// recording whole, the way a recording embeds a machine shape, so the two
    /// forms cannot drift apart.
    ///
    /// Note what is *not* possible here and is in
    /// [`core::record`](crate::core::record): re-deriving the cursor on load.
    /// A recorder seeks its log by the restored instant, because an instant is
    /// a key there. Here the key is sequence position, two answers may share an
    /// instant, and nothing in the guest's state names the position — so the
    /// cursor is genuinely architectural for this seam and derived for that
    /// one.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    pub fn save<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        let cursor = self.inner.lock().cursor as u64;
        let encoded = self.encode()?;
        sink.write_u64(cursor)?;
        sink.write_bytes(&encoded)
    }

    /// Read a log and its cursor back, replacing whatever this one holds.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the encoding is malformed or the cursor is past the
    /// end of the log.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, source: &mut S) -> Result<()> {
        let cursor = source.read_u64()? as usize;
        let encoded = source.read_bytes()?;
        let decoded = Journal::decode(encoded)?;
        let entries = core::mem::take(&mut decoded.inner.lock().entries);
        if cursor > entries.len() {
            return Err(Error::State(alloc::format!(
                "a replay cursor of {cursor} is past the {} recorded answer(s)",
                entries.len()
            )));
        }
        let mut inner = self.inner.lock();
        inner.entries = entries;
        inner.cursor = cursor;
        Ok(())
    }
}

/// A replay divergence, phrased so the reader knows it is a determinism bug
/// and not a corrupt file.
fn diverged(detail: alloc::string::String) -> Error {
    Error::State(alloc::format!(
        "replay diverged from the recording: {detail}"
    ))
}
