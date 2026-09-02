//! Rewind: periodic snapshots plus replay from the nearest one
//! (`ROADMAP.md` §4.5, and phase 9's third gate item).
//!
//! Rewind is not a separate mechanism. It is
//! [`Machine::load`](crate::machine::Machine::load) and
//! [`core::record`](crate::core::record) used together, and this module is the
//! bookkeeping that makes the pair usable: keep a snapshot every so often,
//! keep the input log, and to reach an earlier instant restore the newest
//! snapshot at or before it and replay forward.
//!
//! ```text
//!   snapshots   S0        S1        S2        S3
//!               │         │         │         │
//!   timeline ───●─────────●─────────●─────────●──────────►  now
//!                              ▲
//!                   rewind_to(t) restores S1 and replays the
//!                   log from S1's instant up to t
//! ```
//!
//! # Cadence is a memory/latency trade and nothing else
//!
//! The snapshot interval decides two numbers and no others: how much memory a
//! session costs (one snapshot per interval), and how long a rewind takes (up
//! to one interval of re-execution). [`DEFAULT_CADENCE`] is one virtual second,
//! which on the boards in this tree is a handful of milliseconds of host time
//! to replay and a snapshot small enough to keep hundreds of. A machine with
//! gigabytes of RAM wants a different answer and will want §4.5's page-indexed
//! incremental encoding first — this module deliberately has no opinion about
//! what is inside a snapshot, only about when one is taken.
//!
//! A snapshot is taken **on a round boundary**, because that is where the
//! machine is a complete architectural state and where inputs are delivered.
//! Taking one mid-round would restore to an instant no input log entry can be
//! aligned against.
//!
//! # The replay cursor is derived, not saved
//!
//! The piece that makes a rewind land where it says it does, and it is
//! deliberately *not* in the snapshot.
//! [`Machine::load`](crate::machine::Machine::load) calls
//! [`Recorder::rewind_to`](crate::core::record::Recorder::rewind_to), which
//! seeks the log to the restored instant by binary search. A cursor kept in the
//! snapshot would be a second copy of a number the log already implies, and
//! `CLAUDE.md` is explicit that derived state is rebuilt rather than saved; a
//! cursor kept nowhere would restart the recording from the beginning and hand
//! the guest every keystroke of the run a second time. Seeking is the third
//! option, and it is the one that also works for a debugger loading a snapshot
//! with no timeline in sight.
//!
//! # What a rewind does to host state that cannot be rewound
//!
//! The interesting half, and it is not solvable — only decidable. Three kinds
//! of host state exist on the other side of the seam, and the timeline treats
//! them differently on purpose:
//!
//! * **Queued input the guest has not consumed yet.** Rewindable, and rewound:
//!   [`Recorder::rewind_to`](crate::core::record::Recorder::rewind_to) calls
//!   [`InputSink::on_rewind`](crate::core::record::InputSink::on_rewind) on
//!   every channel so a port drops what it is holding, and the log re-delivers
//!   the same bytes on the way forward. Without this the bytes arrive twice.
//!
//! * **Output the guest has already emitted.** *Not* rewindable, and the
//!   timeline says so rather than pretending: characters printed to a terminal,
//!   bytes written to a socket, samples already in a sound card's ring have
//!   left. A rewound machine will emit them again, and the host sees them
//!   twice. That is the correct behaviour for a debugger — the guest really did
//!   do it twice — and the wrong behaviour for anything that treats the output
//!   as a side effect on the world. A frontend that cares suppresses output
//!   between the rewind target and the point it rewound from; it has the two
//!   instants, and the machine does not have the frontend's policy.
//!
//! * **Host handles: an open file, a socket, a disk image.** Neither rewound
//!   nor rewindable here. A machine snapshot taken while a write-back cache
//!   holds dirty blocks and restored without a matching disk snapshot restores
//!   to a corrupt guest filesystem — §4.5 states that as the *atomicity rule*
//!   and settles it in favour of "storage is snapshotted with the machine or
//!   not at all". Until a block backend can snapshot itself, [`Timeline`]
//!   rewinds the machine and leaves the backing store where it was, which is
//!   sound for a read-only image and unsound for a writable one. The honest
//!   statement is that this is a limitation of the *storage* layer rather than
//!   of rewind, and it moves when §7.1's cache-flush contract lands.
//!
//! # Example
//!
//! ```no_run
//! # use std::sync::Arc;
//! # use rsemu::core::clock::GlobalTime;
//! # use rsemu::core::record::Recorder;
//! # use rsemu::machine::{Machine, Timeline};
//! # fn demo(machine: &mut Machine) -> rsemu::Result<()> {
//! let recorder = Arc::new(Recorder::recording());
//! machine.set_recorder(Arc::clone(&recorder))?;
//!
//! let mut timeline = Timeline::new(recorder, GlobalTime::from_nanos(1_000_000));
//! timeline.run_for(machine, GlobalTime::from_nanos(50_000_000))?;
//!
//! let landed = timeline.rewind_to(machine, GlobalTime::from_nanos(20_000_000))?;
//! assert!(landed <= GlobalTime::from_nanos(20_000_000));
//! # Ok(())
//! # }
//! ```

use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::error::{Error, Result};
use crate::core::record::Recorder;
use crate::machine::machine::Machine;

/// The default interval between snapshots: one virtual second.
///
/// See the module docs for what the number buys. It is a `GlobalTime` rather
/// than a count of rounds because a round is not a fixed length of virtual
/// time — a quantum interrupted by an event is shorter — and a cadence that
/// drifted with event density would make rewind latency unpredictable for no
/// reason.
pub const DEFAULT_CADENCE: GlobalTime = GlobalTime::from_nanos(1_000_000_000);

/// One snapshot and the instant it was taken at.
#[derive(Debug, Clone)]
struct Keyframe {
    at: GlobalTime,
    bytes: Vec<u8>,
}

/// A machine's history: periodic snapshots beside the input log.
///
/// Holds the recorder rather than the machine, so a caller keeps ownership of
/// the machine and can do anything else with it between calls. Every method
/// that needs the machine takes it, which also makes it impossible to point a
/// timeline at a machine other than the one whose recorder it holds without
/// noticing — the shape check on the snapshot catches it.
#[derive(Debug)]
pub struct Timeline {
    recorder: Arc<Recorder>,
    cadence: GlobalTime,
    keyframes: Vec<Keyframe>,
    /// The next instant at or after which a snapshot is due.
    due: GlobalTime,
}

impl Timeline {
    /// A timeline over `recorder`, snapshotting every `cadence` of virtual
    /// time.
    ///
    /// A zero cadence means "snapshot at every round boundary", which is
    /// legitimate for a short deterministic test and ruinous for anything else.
    #[must_use]
    pub fn new(recorder: Arc<Recorder>, cadence: GlobalTime) -> Timeline {
        Timeline {
            recorder,
            cadence,
            keyframes: Vec::new(),
            due: GlobalTime::ZERO,
        }
    }

    /// A timeline at [`DEFAULT_CADENCE`].
    #[must_use]
    pub fn with_default_cadence(recorder: Arc<Recorder>) -> Timeline {
        Timeline::new(recorder, DEFAULT_CADENCE)
    }

    /// The recorder this timeline drives.
    #[must_use]
    pub fn recorder(&self) -> &Arc<Recorder> {
        &self.recorder
    }

    /// The snapshot interval.
    #[must_use]
    pub fn cadence(&self) -> GlobalTime {
        self.cadence
    }

    /// How many snapshots are held.
    #[must_use]
    pub fn keyframes(&self) -> usize {
        self.keyframes.len()
    }

    /// The instants snapshots were taken at, oldest first.
    #[must_use]
    pub fn instants(&self) -> Vec<GlobalTime> {
        self.keyframes.iter().map(|k| k.at).collect()
    }

    /// The total size of the snapshots held, in bytes.
    ///
    /// The number a cadence is chosen against.
    #[must_use]
    pub fn bytes_held(&self) -> usize {
        self.keyframes.iter().map(|k| k.bytes.len()).sum()
    }

    /// Take a snapshot now, whatever the cadence says.
    ///
    /// Call it before the first run so a rewind to the beginning has somewhere
    /// to land; [`Timeline::run_until`] does that for you when the timeline is
    /// empty.
    ///
    /// # Errors
    ///
    /// Whatever [`Machine::save`] reports.
    pub fn snapshot(&mut self, machine: &Machine) -> Result<()> {
        let at = machine.now();
        let bytes = machine.save()?;
        // Replacing rather than appending when the instant repeats: a caller
        // that snapshots twice without running must not grow the history, and a
        // zero cadence would otherwise do exactly that every round.
        match self.keyframes.last_mut() {
            Some(last) if last.at == at => last.bytes = bytes,
            _ => self.keyframes.push(Keyframe { at, bytes }),
        }
        self.due = at.saturating_add(self.cadence);
        Ok(())
    }

    /// The smallest interval the cadence can actually be honoured at.
    ///
    /// A snapshot is taken between rounds, so a cadence finer than a round is
    /// not a finer cadence — it is a request that cannot be met, and asking for
    /// one must not make the loop stand still. The scheduler's own quantum is
    /// the floor, and a raw unit is the floor of *that* for a machine whose
    /// quantum is degenerate.
    fn step(&self, machine: &Machine) -> GlobalTime {
        let quantum = machine.scheduler().config().quantum;
        self.cadence.max(quantum).max(GlobalTime::from_raw(1))
    }

    /// Run `machine` to `deadline`, taking snapshots on the cadence.
    ///
    /// The run is broken at the cadence points and nowhere else, so the
    /// trajectory is the one
    /// [`Machine::run_until`](crate::machine::Machine::run_until) would have
    /// taken on its own: that call is additive (§11.6), a snapshot is taken
    /// *between* rounds, and a snapshot observes nothing.
    ///
    /// A round that will not fit before the next cadence point is not split —
    /// splitting one is exactly what costs `run_for` its additivity — so a
    /// snapshot can land a round late. The cadence is a budget, not a
    /// guarantee, and saying so here is cheaper than a caller discovering it
    /// from a keyframe list with a gap in it.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the machine has no recorder or a different one from
    /// this timeline's; otherwise whatever a round or a save reports.
    pub fn run_until(&mut self, machine: &mut Machine, deadline: GlobalTime) -> Result<()> {
        self.check_attached(machine)?;
        if self.keyframes.is_empty() {
            self.snapshot(machine)?;
        }
        let step = self.step(machine);
        while machine.now() < deadline {
            let before = machine.now();
            // Always strictly beyond `before`, so the loop cannot stand still
            // even when the cadence point is already behind us.
            let target = deadline.min(self.due.max(before.saturating_add(step)));
            machine.run_until(target)?;
            if machine.now() <= before {
                if target >= deadline {
                    // The next round would overrun the caller's deadline.
                    // `Machine::run_until` declines it rather than splitting
                    // it, so a run legitimately ends with time left over; the
                    // next call executes it and nothing is lost.
                    break;
                }
                // It would overrun the cadence point instead. Push the point
                // out and take the round: a late snapshot beats a split round.
                self.due = target.saturating_add(step);
                continue;
            }
            if machine.now() >= self.due {
                self.snapshot(machine)?;
            }
        }
        Ok(())
    }

    /// [`Timeline::run_until`], relative to where the machine is now.
    ///
    /// # Errors
    ///
    /// As [`Timeline::run_until`].
    pub fn run_for(&mut self, machine: &mut Machine, span: GlobalTime) -> Result<()> {
        let deadline = machine.now().saturating_add(span);
        self.run_until(machine, deadline)
    }

    /// Rewind to `at`, returning the instant actually reached.
    ///
    /// Restores the newest snapshot at or before `at`, rewinds the recorder to
    /// that snapshot's instant, and replays forward. The instant returned is
    /// the machine's own after the replay, which is at or a little before `at`
    /// for the same reason [`Machine::run_until`](crate::machine::Machine::run_until)
    /// can stop short: a round that would overrun the target is declined rather
    /// than split.
    ///
    /// **The recorder must be replaying** for the forward run to reproduce the
    /// original. A recording recorder is allowed and does something different
    /// and deliberate: it truncates its log at `at`, so the run forward from
    /// there records a *new* future. That is what an interactive rewind wants —
    /// go back, do something else — and it is why
    /// [`Recorder::rewind_to`](crate::core::record::Recorder::rewind_to)
    /// truncates rather than keeping a branch. rsemu has one timeline, not a
    /// tree of them.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if there is no snapshot at or before `at` — a rewind
    /// target older than the oldest keyframe cannot be reached and says so —
    /// or if the machine has a different recorder; otherwise whatever the load
    /// or the forward run reports.
    pub fn rewind_to(&mut self, machine: &mut Machine, at: GlobalTime) -> Result<GlobalTime> {
        self.check_attached(machine)?;
        let index = self
            .keyframes
            .partition_point(|k| k.at <= at)
            .checked_sub(1)
            .ok_or_else(|| Error::Config {
                at: machine.name().to_string(),
                message: match self.keyframes.first() {
                    Some(first) => alloc::format!(
                        "cannot rewind to {}: the oldest snapshot is at {}",
                        at.raw(),
                        first.at.raw()
                    ),
                    None => alloc::string::String::from(
                        "cannot rewind: this timeline holds no snapshots",
                    ),
                },
            })?;

        let keyframe = self.keyframes[index].clone();
        // `Machine::load` rewinds the seam itself, to the instant it restored —
        // it has to, so that a debugger loading a snapshot without a timeline
        // is not left delivering against an instant the machine has left. That
        // is exactly `keyframe.at`, so there is nothing to do here afterwards.
        machine.load(&keyframe.bytes)?;
        debug_assert_eq!(machine.now(), keyframe.at);
        // Everything after the restored keyframe is a future that no longer
        // exists. Keeping it would let a later rewind land on a snapshot of a
        // run that has been replaced.
        self.keyframes.truncate(index + 1);
        self.due = keyframe.at.saturating_add(self.cadence);

        if at > keyframe.at {
            self.run_until(machine, at)?;
        }
        Ok(machine.now())
    }

    /// Drop every snapshot older than `at`, keeping the one that covers it.
    ///
    /// The bounded-history knob: an interactive session that offers ten seconds
    /// of rewind calls this with `now - 10s` after each run. The keyframe *at
    /// or before* `at` is kept, because dropping it would make `at` itself
    /// unreachable.
    pub fn forget_before(&mut self, at: GlobalTime) {
        let keep = self.keyframes.partition_point(|k| k.at <= at);
        let drop = keep.saturating_sub(1);
        if drop > 0 {
            self.keyframes.drain(..drop);
        }
    }

    /// Refuse a machine that is not driven by this timeline's recorder.
    ///
    /// Cheap and worth it: a timeline pointed at the wrong machine would take
    /// snapshots of one and rewind the input of another, and the failure would
    /// look like a determinism bug rather than a wiring mistake.
    fn check_attached(&self, machine: &Machine) -> Result<()> {
        match machine.recorder() {
            Some(attached) if Arc::ptr_eq(attached, &self.recorder) => Ok(()),
            Some(_) => Err(Error::Config {
                at: machine.name().to_string(),
                message: alloc::string::String::from(
                    "this machine is driven by a different recorder than the timeline holds",
                ),
            }),
            None => Err(Error::Config {
                at: machine.name().to_string(),
                message: alloc::string::String::from(
                    "a timeline needs the machine's recorder attached: call \
                     `Machine::set_recorder` first",
                ),
            }),
        }
    }
}

#[cfg(test)]
mod tests;
