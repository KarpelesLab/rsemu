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

use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::error::{Error, Result};
use crate::core::state::{Sink, Source};
use crate::core::sync::{self, LockRank};

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

    /// Write the log.
    ///
    /// This is the trace file `rsemu record` would produce, and the input
    /// `rsemu replay` would take (§2's binary surface). The mode is not
    /// written: it is what you are *doing* with a log, not part of one.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    pub fn save<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        let inner = self.inner.lock();
        sink.write_u64(inner.cursor as u64)?;
        sink.write_seq_len(inner.entries.len() as u64)?;
        for entry in &inner.entries {
            sink.write_u128(entry.at)?;
            sink.write_u32(entry.tag)?;
            sink.write_u64(entry.answer.value)?;
            sink.write_bytes(&entry.answer.bytes)?;
        }
        Ok(())
    }

    /// Read a log back, replacing whatever this one holds.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the encoding is malformed or the cursor is past the
    /// end of the log.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, source: &mut S) -> Result<()> {
        let cursor = source.read_u64()? as usize;
        let count = source.read_seq_len(36)?;
        let mut entries = Vec::new();
        for _ in 0..count {
            let at = source.read_u128()?;
            let tag = source.read_u32()?;
            let value = source.read_u64()?;
            let bytes = source.read_bytes()?.to_vec();
            entries.push(Entry {
                at,
                tag,
                answer: Answer { value, bytes },
            });
        }
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
