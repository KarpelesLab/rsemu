//! The scheduling contract for guest threads.
//!
//! `ROADMAP.md` phase 5b asks for *"a scheduling contract for guest threads, so
//! §4.2's rules about who owns time still hold when the scheduled thing is a
//! thread rather than a CPU"*. This is that contract, and it is deliberately
//! thin: rsemu says **when** threads run and **how long**, and says nothing
//! about what a thread *is*. Whether two threads share a memory map, what a
//! process is, what `clone` copies and what a signal does are all the
//! consumer's, per §2.1.
//!
//! # What the contract actually is
//!
//! 1. A guest thread is an [`ExitingCore`]. Nothing more is required of it.
//! 2. It is run for a **quantum measured in ticks**, never in host time. That
//!    is the whole determinism argument: a wall-clock quantum preempts at an
//!    instruction that depends on how fast the host was that afternoon, and a
//!    tick quantum preempts at the same instruction every time.
//! 3. Virtual time advances by exactly what threads consumed
//!    ([`GuestClock`]), so a consumer's clock is a function of execution.
//! 4. When nothing is runnable but something is asleep, **time jumps to the
//!    earliest deadline**. A level-3 run never sleeps on the host — there is
//!    nothing to wait for that is not itself virtual.
//!
//! # A pull loop, not a callback
//!
//! [`ThreadSet::run_next`] returns a [`Stop`] and the consumer decides what it
//! means. There is no `trait Kernel` here and there will not be one: rsemu
//! does not know what services a syscall, and a callback would put the
//! consumer's kernel inside rsemu's lock discipline for no benefit. The
//! consumer's run loop is the outer loop, exactly as it is today in the crate
//! this exists for.
//!
//! # Determinism, precisely
//!
//! With one [`ThreadSet`] driving every thread, the interleaving is a pure
//! function of the program: the thread order is an id order (a `BTreeMap`, not
//! a hash), the quantum is a tick count, and the clock is virtual. That is
//! what makes [`super::journal`] able to replay a run at all — a journal of
//! external answers is only replayable if the *questions* come back in the
//! same order.
//!
//! Running threads on several host workers is a different mode and is not this
//! type's job; §4.7 already says parallel guest execution is not reproducible
//! and that determinism is a property of the mode rather than of the thread
//! count.

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::error::{Error, Result};
use crate::core::exec::{Exit, ExitingCore};
use crate::core::sched::Budget;
use crate::core::state::{Sink, Source};
use crate::core::sync::{self, LockRank};

use super::clock::GuestClock;

/// How many ticks a thread runs before the next one gets a turn, unless the
/// consumer says otherwise.
///
/// Ten thousand bus accesses: long enough that the per-quantum bookkeeping is
/// noise, short enough that a thread spinning on a futex word does not starve
/// the thread that will set it.
pub const DEFAULT_QUANTUM: u64 = 10_000;

/// A guest thread's identity within one [`ThreadSet`].
///
/// Ids are handed out in order and **never reused**, so a stale id names
/// nothing rather than naming whoever came next. They are not process ids,
/// thread ids or anything else the guest can see — the consumer maps its own
/// numbering onto these.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct ThreadId(pub u32);

/// Whether a thread wants to run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadState {
    /// Ready to run whenever its turn comes.
    Runnable,
    /// Not runnable until the consumer says so, or — if there is a deadline —
    /// until virtual time reaches it.
    ///
    /// The deadline is the *only* thing the framework knows about why a thread
    /// is blocked. What it is waiting for, and whether the wait can be
    /// interrupted, are the consumer's business.
    Blocked {
        /// When to make it runnable again, or `None` to wait for an explicit
        /// [`ThreadSet::wake`].
        until: Option<GlobalTime>,
    },
}

/// One thread, and how it stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stop {
    /// Which thread ran.
    pub thread: ThreadId,
    /// Ticks it consumed.
    pub consumed: u64,
    /// Virtual time after it ran, which is when the exit happened.
    pub at: GlobalTime,
    /// Why it stopped, or `None` if its quantum simply ran out.
    ///
    /// `None` is not an event: the consumer loops and the thread runs again
    /// when its turn comes round. It exists so a run loop can count quanta,
    /// poll a cancellation flag, or hand control back to a browser event loop.
    pub exit: Option<Exit>,
}

/// One thread's entry.
#[derive(Debug)]
struct Entry {
    core: Arc<dyn ExitingCore>,
    state: ThreadState,
}

/// The table, and where the round robin has got to.
#[derive(Debug)]
struct Inner {
    threads: BTreeMap<ThreadId, Entry>,
    next_id: u32,
    /// The id to resume scanning from. Held rather than an index because ids
    /// are stable and positions are not.
    cursor: ThreadId,
    quantum: u64,
}

/// The guest threads of a level-3 run, and the clock they advance.
///
/// # Locking
///
/// One [`sync::Mutex`] at [`LockRank::SCHED`], **released before a thread
/// runs** — the re-entrancy contract of §4.7 in its simplest form. A guest
/// thread executing is an outward call of the most emphatic kind: it reaches
/// the address space, and through a consumer's syscall handling it may come
/// back here to wake a sibling.
#[derive(Debug)]
pub struct ThreadSet {
    clock: Arc<GuestClock>,
    inner: sync::Mutex<Inner>,
}

impl ThreadSet {
    /// An empty set sharing `clock`, with the default quantum.
    #[must_use]
    pub fn new(clock: Arc<GuestClock>) -> ThreadSet {
        ThreadSet {
            clock,
            inner: sync::Mutex::with_rank(
                LockRank::SCHED,
                Inner {
                    threads: BTreeMap::new(),
                    next_id: 1,
                    cursor: ThreadId(0),
                    quantum: DEFAULT_QUANTUM,
                },
            ),
        }
    }

    /// The clock these threads advance.
    #[must_use]
    pub fn clock(&self) -> &Arc<GuestClock> {
        &self.clock
    }

    /// How many ticks a thread runs before the next one gets a turn.
    #[must_use]
    pub fn quantum(&self) -> u64 {
        self.inner.lock().quantum
    }

    /// Set the quantum.
    ///
    /// A quantum of zero would make no progress, so it is clamped to one tick.
    pub fn set_quantum(&self, ticks: u64) {
        self.inner.lock().quantum = ticks.max(1);
    }

    /// Add a runnable thread and name it.
    pub fn insert(&self, core: Arc<dyn ExitingCore>) -> ThreadId {
        let mut inner = self.inner.lock();
        let id = ThreadId(inner.next_id);
        inner.next_id = inner.next_id.wrapping_add(1);
        inner.threads.insert(
            id,
            Entry {
                core,
                state: ThreadState::Runnable,
            },
        );
        id
    }

    /// Remove a thread. Returns whether there was one.
    ///
    /// A thread that has exited is *removed*, not parked: the framework has no
    /// concept of a zombie, because a zombie is a thing a parent can wait for
    /// and waiting is the consumer's.
    pub fn remove(&self, id: ThreadId) -> bool {
        self.inner.lock().threads.remove(&id).is_some()
    }

    /// How many threads are in the set.
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.lock().threads.len()
    }

    /// Whether the set is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.lock().threads.is_empty()
    }

    /// Every thread, in id order.
    #[must_use]
    pub fn ids(&self) -> Vec<ThreadId> {
        self.inner.lock().threads.keys().copied().collect()
    }

    /// The core behind a thread, so a consumer can read its registers.
    ///
    /// This is how a syscall's number and arguments are reached: the consumer
    /// downcasts or, more usefully, kept its own `Arc` to the concrete core
    /// when it inserted it. rsemu does not know which register carries a
    /// syscall number and deliberately does not offer to guess
    /// ([`ExitingCore`]).
    #[must_use]
    pub fn core(&self, id: ThreadId) -> Option<Arc<dyn ExitingCore>> {
        self.inner
            .lock()
            .threads
            .get(&id)
            .map(|e| Arc::clone(&e.core))
    }

    /// What a thread is doing.
    #[must_use]
    pub fn state(&self, id: ThreadId) -> Option<ThreadState> {
        self.inner.lock().threads.get(&id).map(|e| e.state)
    }

    /// Stop running a thread until something wakes it.
    ///
    /// Returns whether there was such a thread. `until` is a virtual instant —
    /// [`GuestClock::now`] plus however long — and `None` means "only an
    /// explicit [`wake`](ThreadSet::wake)".
    pub fn block(&self, id: ThreadId, until: Option<GlobalTime>) -> bool {
        let mut inner = self.inner.lock();
        match inner.threads.get_mut(&id) {
            Some(entry) => {
                entry.state = ThreadState::Blocked { until };
                true
            }
            None => false,
        }
    }

    /// Make a thread runnable again. Returns whether there was such a thread.
    pub fn wake(&self, id: ThreadId) -> bool {
        let mut inner = self.inner.lock();
        match inner.threads.get_mut(&id) {
            Some(entry) => {
                entry.state = ThreadState::Runnable;
                true
            }
            None => false,
        }
    }

    /// Run the next runnable thread for one quantum.
    ///
    /// Returns `None` when there is nothing to run and nothing that will ever
    /// become runnable on its own — no threads at all, or every one of them
    /// blocked with no deadline. That is a real condition a consumer must
    /// handle (it is a deadlock, or the program is over), and reporting it is
    /// better than spinning.
    ///
    /// When every thread is asleep with a deadline, virtual time **jumps** to
    /// the earliest one and that thread runs. There is no host sleep anywhere
    /// in this crate.
    pub fn run_next(&self) -> Option<Stop> {
        let (id, core, quantum) = self.pick()?;
        // The lock is released before the guest runs. See the type's docs.
        let run = core.run_to_exit(Budget::of(quantum));
        self.clock.advance(run.consumed.ticks);
        Some(Stop {
            thread: id,
            consumed: run.consumed.ticks,
            at: self.clock.now(),
            exit: run.exit,
        })
    }

    /// Choose the next thread, waking sleepers and jumping the clock if that
    /// is what it takes.
    fn pick(&self) -> Option<(ThreadId, Arc<dyn ExitingCore>, u64)> {
        for _ in 0..2 {
            if let Some(picked) = self.pick_runnable() {
                return Some(picked);
            }
            // Nothing runnable. If anything is sleeping, virtual time is
            // allowed to skip to the moment it wakes — the scheduler owns
            // time, so there is nothing to wait *for* (`ROADMAP.md` §4.2).
            let earliest = {
                let inner = self.inner.lock();
                inner
                    .threads
                    .values()
                    .filter_map(|e| match e.state {
                        ThreadState::Blocked { until } => until,
                        ThreadState::Runnable => None,
                    })
                    .min()?
            };
            self.clock.advance_to(earliest);
            let now = self.clock.now();
            let mut inner = self.inner.lock();
            for entry in inner.threads.values_mut() {
                if let ThreadState::Blocked { until: Some(when) } = entry.state
                    && when <= now
                {
                    entry.state = ThreadState::Runnable;
                }
            }
        }
        None
    }

    /// The next runnable thread at or after the cursor, wrapping once.
    fn pick_runnable(&self) -> Option<(ThreadId, Arc<dyn ExitingCore>, u64)> {
        let mut inner = self.inner.lock();
        let cursor = inner.cursor;
        let id = inner
            .threads
            .range(cursor..)
            .chain(inner.threads.range(..cursor))
            .find(|(_, entry)| entry.state == ThreadState::Runnable)
            .map(|(id, _)| *id)?;
        // Resume the next scan *after* this thread, so a runnable thread never
        // starves one behind it.
        inner.cursor = ThreadId(id.0.wrapping_add(1));
        let core = Arc::clone(&inner.threads.get(&id)?.core);
        Some((id, core, inner.quantum))
    }

    /// Write the schedule: which threads exist, what they are doing, and whose
    /// turn is next.
    ///
    /// The **cores are not written**, because a core is a
    /// [`Device`](crate::core::Device) with a `save` of its own and the
    /// consumer owns where that goes. What is here is the part only this type
    /// knows: the run states, the deadlines and the round-robin cursor —
    /// exactly the "scheduler is architectural state" rule of §4.5, applied to
    /// a scheduler whose runnables are threads.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    pub fn save<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        let inner = self.inner.lock();
        sink.write_u32(inner.next_id)?;
        sink.write_u32(inner.cursor.0)?;
        sink.write_u64(inner.quantum)?;
        sink.write_seq_len(inner.threads.len() as u64)?;
        for (id, entry) in &inner.threads {
            sink.write_u32(id.0)?;
            match entry.state {
                ThreadState::Runnable => sink.write_u8(0)?,
                ThreadState::Blocked { until: None } => sink.write_u8(1)?,
                ThreadState::Blocked { until: Some(at) } => {
                    sink.write_u8(2)?;
                    sink.write_u128(at.raw())?;
                }
            }
        }
        Ok(())
    }

    /// Read a schedule back onto the threads already inserted.
    ///
    /// The cores must be back first: a `ThreadSet` cannot conjure an
    /// [`ExitingCore`], so a restore is *insert every thread, then load*. That
    /// asymmetry is deliberate and is the honest one — the alternative is a
    /// factory callback that would have to know every core class.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the encoding is malformed, or names a thread that
    /// has not been inserted.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, source: &mut S) -> Result<()> {
        let next_id = source.read_u32()?;
        let cursor = source.read_u32()?;
        let quantum = source.read_u64()?;
        let count = source.read_seq_len(5)?;
        let mut states = BTreeMap::new();
        for _ in 0..count {
            let id = ThreadId(source.read_u32()?);
            let state = match source.read_u8()? {
                0 => ThreadState::Runnable,
                1 => ThreadState::Blocked { until: None },
                2 => ThreadState::Blocked {
                    until: Some(GlobalTime::from_raw(source.read_u128()?)),
                },
                other => {
                    return Err(Error::State(alloc::format!(
                        "unknown thread state {other} in a schedule"
                    )));
                }
            };
            states.insert(id, state);
        }
        let mut inner = self.inner.lock();
        for id in states.keys() {
            if !inner.threads.contains_key(id) {
                return Err(Error::State(alloc::format!(
                    "the schedule names thread {} but it has not been inserted",
                    id.0
                )));
            }
        }
        inner.threads.retain(|id, _| states.contains_key(id));
        for (id, state) in states {
            if let Some(entry) = inner.threads.get_mut(&id) {
                entry.state = state;
            }
        }
        inner.next_id = next_id;
        inner.cursor = ThreadId(cursor);
        inner.quantum = quantum;
        Ok(())
    }
}
