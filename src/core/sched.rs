//! Event queue, execution budgets, threading modes (`ROADMAP.md` §4.2).
//!
//! The scheduler owns time. A device never sleeps, never reads a wall clock and
//! never spawns anything to tick itself: it registers an event, or it declares
//! itself lazily advanced and gets caught up before it is touched.
//!
//! # The four pieces
//!
//! * **The event queue** is a hierarchical timing wheel for the dense near term
//!   plus a binary heap for the far future. Events carry a monotonically
//!   increasing sequence number, so two events at the same instant always fire
//!   in the order they were posted — ties break deterministically or the whole
//!   determinism claim is void.
//! * **Execution budgets.** A CPU is never "stepped one instruction". It is
//!   handed a [`Budget`] — *run until virtual time T, or N ticks, whichever
//!   comes first* — and reports back what it [`Consumed`]. That is what lets
//!   JIT block execution and per-access cycle accounting coexist: the block runs
//!   to its natural end and the tick count is the truth afterwards.
//! * **Rounds end where the machine says, not where the caller does.** One
//!   round of the round-robin runs to its *natural target*: the next point of
//!   an absolute quantum grid, the next queued event, or the next event a
//!   lazily-advanced device has of its own — whichever is first, and all three are functions of
//!   virtual time and machine state alone. A caller's deadline that falls
//!   *inside* a round does not shorten it; the round simply does not start, and
//!   runs whole when the caller asks for more time. That is what makes
//!   [`Machine::run_for`](crate::machine::Machine::run_for) additive (§11.6),
//!   and it is worth the one thing it costs: a run can return with up to one
//!   round of virtual time elapsed and not yet executed. Nothing is lost —
//!   budgets come from each tree's absolute position, so the next round hands
//!   out the ticks — but a caller that needs execution to track a fine deadline
//!   wants a shorter [`SchedulerConfig::quantum`], or
//!   [`Scheduler::step_quantum_until`], which is the debugger's.
//! * **Sync-on-access.** The queue handles *scheduled* behaviour, but not
//!   *sampled* behaviour: a 6502 reads `$2002` at an arbitrary cycle and the PPU
//!   has to be at exactly that dot, sprite-0 and vblank race included. So a
//!   device may register as a [`LazyDevice`]: it holds its own tick and gets
//!   [`LazyDevice::advance_to`] before any access is dispatched to it. Without
//!   this a 10 000-tick budget makes every status read thousands of cycles
//!   stale, and the split-screen status bar in nearly every NES game is wrong.
//!   Catch-up is bounded by the device's own next scheduled event, so it never
//!   simulates past a point where its behaviour would change, and a debug access
//!   ([`AccessKind::Debug`]) advances nothing at all. The trigger is a
//!   [`LazyHandle`] — see below.
//! * **Snapshots.** The scheduler is architectural state, not a cache
//!   (`ROADMAP.md` §4.5): [`Scheduler::snapshot`] and [`Scheduler::restore`]
//!   carry the pending events, virtual time, the tie-break counter and the
//!   round-robin cursor across a save/load, so a restored timer is the same
//!   number of ticks from firing as the saved one was.
//! * **Threading modes and rate control**, selected per machine.
//!   [`ThreadingMode::Deterministic`] round-robins every runnable on one
//!   thread and is the mode whose state hash is a golden.
//!   [`ThreadingMode::Parallel`] gives each runnable a job on the
//!   `core::sync` task pool and joins them at the round's boundary — that join
//!   is §4.2's rendezvous barrier — and gives up reproducibility for it,
//!   which is why [`Machine::state_hash`](crate::machine::Machine::state_hash)
//!   refuses in it. [`ThreadingMode::Accel`] is that same round with its
//!   *clock* replaced: virtual time is read off the injected [`HostClock`]
//!   rather than derived from what the runnables reported, because an engine
//!   whose guest runs on host silicon has no tick count to report. That is
//!   §4.2's "the scheduler becomes a deadline service", and it is what lets a
//!   stock kernel's delay loops mean something under acceleration.
//! * **Safe points.** [`SafePoint`] is §4.7's stop-the-world protocol: a
//!   generation counter and a per-runnable [`ExitFlag`], checked at block
//!   boundaries, never a host signal. [`Scheduler::stop_the_world`] raises it
//!   and waits out the pool, which is what a snapshot or a retopology needs.
//!
//! # Catch-up and the lock ladder
//!
//! Sync-on-access has to fire from inside `MemOps::read`, which takes `&self`
//! and runs with the bus's own lock held, well below the loop that owns the
//! scheduler. That rules out reaching back for a scheduler-ranked lock:
//! [`LockRank::SCHED`](crate::core::sync::LockRank::SCHED) is *above*
//! [`LockRank::BUS`](crate::core::sync::LockRank::BUS), so an access that
//! acquired one would invert the ladder, and two CPUs doing it on two buses is a
//! deadlock rather than a style violation.
//!
//! That deadlock is no longer hypothetical: under
//! [`ThreadingMode::Parallel`] two CPUs really are inside their own
//! [`LockRank::BUS`](crate::core::sync::LockRank::BUS)-ranked session locks at
//! the same instant, which is the first time the ladder has had to hold against
//! anything but one thread's own nesting.
//!
//! So catch-up never takes a scheduler lock. Each lazily-advanced device sits in
//! its own slot behind a leaf-ranked lock that is held across a move and nothing
//! else — the device is taken *out* of the slot, the guard is dropped, and only
//! then is [`LazyDevice::advance_to`] called, so the device is free to touch its
//! own bus and its own state while nothing is held. A [`LazyHandle`] is a shared
//! reference to one such slot, handed to the access path when the machine is
//! built. The one thing the slot needs from the clock forest — where the
//! device's domain has got to — is published into it every time the scheduler
//! advances virtual time.
//!
//! # What this module may not do
//!
//! Nothing here names `std::thread`, `std::sync`, or the host clock
//! (`ROADMAP.md` §15, invariant 4). Rate control genuinely needs wall time, so
//! it takes a [`HostClock`] **injected** at construction and implemented above
//! the `std` line. That keeps the `no_std` and wasm builds compiling and keeps
//! the clock mockable, which is what makes deterministic replay testable at all.
//! [`ThreadingMode::Accel`] uses the same injected clock for a second purpose
//! — as the *source* of elapsed time rather than as a throttle on it — and
//! that is the whole of its access to the host: one trait, one method, one
//! implementation, in `host/`.
//!
//! There is also no floating point, here or anywhere it reaches. Pacing
//! arithmetic is integer nanoseconds and fixed-point [`GlobalTime`].
//!
//! # Where virtual time sits relative to a tree
//!
//! [`Scheduler::now`] is the front of virtual time. An individual clock tree may
//! sit a little behind it — by strictly less than one tick of whichever domain
//! drives it — because of two rules that are both worth more than the
//! discrepancy:
//!
//! * A runnable is never let past a scheduled event. Stopping short is a
//!   rounding error; running past one is an interrupt handled too late.
//! * A tree only ever advances by whole ticks of a domain that drives it.
//!   Dragging a tree to an arbitrary instant mid-cycle would permanently shift
//!   that domain's phase against its own crystal, which is a far worse lie than
//!   a fractional cycle of lag.
//!
//! So an event posted for NES PPU dot 82181 fires at exactly the instant of that
//! dot, at which point the PPU's counter reads 82179 — the CPU cycle containing
//! that dot has not finished. A device delivering its own event advances itself
//! to the tick it asked for; it knows that tick, having scheduled it.

use alloc::boxed::Box;
use alloc::collections::{BTreeSet, BinaryHeap};
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cmp::{Ordering, Reverse};
use core::fmt;

use crate::core::clock::{ClockError, ClockForest, DomainId, GlobalTime, OscillatorId};
use crate::core::sync::{AtomicBool, AtomicU64, Handle, Mutex, Ordering as AtomicOrdering, Pool};

// ---------------------------------------------------------------------------
// errors
// ---------------------------------------------------------------------------

/// Everything the scheduler refuses to do.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum SchedError {
    /// The clock forest rejected an operation.
    Clock(ClockError),
    /// The handle does not belong to this scheduler.
    UnknownRunnable(RunnableId),
    /// The handle does not belong to this scheduler.
    UnknownLazyDevice(LazyId),
    /// A runnable reported consuming more than it was given.
    ///
    /// Always a bug in the runnable, and always fatal: a CPU that overruns its
    /// budget has already executed past an event that should have interrupted
    /// it, and no later correction can put that back.
    BudgetExceeded {
        /// Who overran.
        runnable: RunnableId,
        /// What it was allowed.
        budget: u64,
        /// What it claimed.
        consumed: u64,
    },
    /// A lazily-advanced device went backwards.
    NonMonotonicDevice {
        /// Which device.
        device: LazyId,
        /// Where it was.
        from: u64,
        /// Where it claimed to be afterwards.
        to: u64,
    },
    /// A lazily-advanced device is already being advanced further up the stack.
    ///
    /// Catch-up takes the device out of its slot for the duration of
    /// [`LazyDevice::advance_to`], precisely so that no lock is held across
    /// that call (`ROADMAP.md` §4.7's re-entrancy contract). A second catch-up
    /// reaching the same device while the first is still running — a device
    /// that reads its own registers as it simulates — therefore finds the slot
    /// empty. Reporting it beats both alternatives: recursing would need two
    /// mutable borrows of one device, and waiting would be a deadlock.
    ///
    /// Under [`ThreadingMode::Deterministic`] one thread runs everything, so
    /// this can only mean re-entrancy, and it is reported immediately.
    /// [`ThreadingMode::Parallel`] also reaches the same emptiness when two
    /// CPUs touch one device inside one quantum, which is contention rather
    /// than a bug — so there it waits out a bounded spin first and this is what
    /// is left when the wait expires.
    LazyDeviceBusy(LazyId),
    /// Rate control, or [`ThreadingMode::Accel`], needs a host clock and none
    /// was injected.
    ///
    /// A policy failure for the first and a fatal one for the second:
    /// `Accel` has no other source of elapsed time, and the alternative to
    /// saying so would be a machine whose clocks run at a rate set by the
    /// quantum.
    NoHostClock,
    /// A snapshot could not be restored into this scheduler.
    InvalidSnapshot(&'static str),
}

impl fmt::Display for SchedError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SchedError::Clock(e) => write!(f, "clock: {e}"),
            SchedError::UnknownRunnable(id) => write!(f, "no runnable #{}", id.0),
            SchedError::UnknownLazyDevice(id) => write!(f, "no lazy device #{}", id.0),
            SchedError::BudgetExceeded {
                runnable,
                budget,
                consumed,
            } => write!(
                f,
                "runnable #{} consumed {consumed} ticks of a {budget}-tick budget",
                runnable.0
            ),
            SchedError::NonMonotonicDevice { device, from, to } => write!(
                f,
                "lazy device #{} went backwards, from tick {from} to {to}",
                device.0
            ),
            SchedError::LazyDeviceBusy(id) => write!(
                f,
                "lazy device #{} is already being advanced further up the stack",
                id.0
            ),
            SchedError::NoHostClock => f.write_str(
                "no host clock was injected: rate control and `accel` threading both need one",
            ),
            SchedError::InvalidSnapshot(why) => write!(f, "invalid scheduler snapshot: {why}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for SchedError {}

impl From<ClockError> for SchedError {
    fn from(e: ClockError) -> Self {
        SchedError::Clock(e)
    }
}

impl From<SchedError> for crate::core::Error {
    /// Scheduler failures surface as configuration errors.
    ///
    /// As with [`ClockError`], `core::Error` has no dedicated variant yet; it is
    /// `#[non_exhaustive]` and one belongs there.
    fn from(e: SchedError) -> Self {
        use alloc::string::ToString;
        crate::core::Error::Config {
            at: String::from("scheduler"),
            message: e.to_string(),
        }
    }
}

/// Shorthand for a fallible scheduler operation.
pub type SchedResult<T> = core::result::Result<T, SchedError>;

// ---------------------------------------------------------------------------
// events
// ---------------------------------------------------------------------------

/// A handle to a queued event, usable to cancel it.
///
/// The value is the event's sequence number, which is also what breaks ties
/// between events at the same instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EventId(u64);

impl EventId {
    /// The raw sequence number.
    #[inline]
    pub const fn seq(self) -> u64 {
        self.0
    }

    /// Rebuilds a handle from a sequence number.
    ///
    /// For snapshot restore, and for nothing else: an event's identity *is* its
    /// tie-break, so a queue rebuilt from a snapshot has to carry the numbers it
    /// was saved with or two events at one instant swap places
    /// (`ROADMAP.md` §4.5). Minting a number here for a *fresh* event would
    /// collide with the queue's own counter; that is what
    /// [`EventQueue::schedule`] is for.
    #[inline]
    pub const fn from_seq(seq: u64) -> EventId {
        EventId(seq)
    }
}

/// Who an event is for.
///
/// An opaque handle the machine layer maps back to a device. The core stays
/// free of device types (`ROADMAP.md` §15, invariant 1), so this is deliberately
/// just a number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct EventTarget(pub u32);

/// A queued event.
///
/// Ordering is `(time, seq)` and nothing else, which is what makes the fire
/// order a pure function of the posting order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    /// When it fires.
    pub time: GlobalTime,
    /// Its identity, and its tie-break.
    pub id: EventId,
    /// Who it is for.
    pub target: EventTarget,
    /// An opaque value handed back to the target — a timer index, a channel
    /// number, whatever the device put there.
    pub token: u64,
}

impl Ord for Event {
    fn cmp(&self, other: &Event) -> Ordering {
        self.time
            .cmp(&other.time)
            .then_with(|| self.id.0.cmp(&other.id.0))
    }
}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Event) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Levels in the timing wheel. Four levels of 256 slots span 2³² granules.
const WHEEL_LEVELS: usize = 4;
/// Bits of granule index consumed per wheel level.
const WHEEL_SLOT_BITS: u32 = 8;
/// Slots per level.
const WHEEL_SLOTS: usize = 1 << WHEEL_SLOT_BITS;

/// The default granule, in [`GlobalTime`] bits: 2⁻³² s, about 233 ps.
///
/// Four levels of 256 slots then cover the next second of virtual time, which
/// is far more than any real machine queues densely, and everything beyond goes
/// to the far heap.
pub const DEFAULT_GRANULE_SHIFT: u32 = 32;

/// Where an event belongs right now.
enum Placement {
    /// At or before the current granule: it goes to the due heap, which filters
    /// on exact time.
    Due,
    /// Level and slot of the near wheel.
    Near(usize, usize),
    /// Beyond the wheel's span.
    Far,
}

/// The event queue: a hierarchical timing wheel plus a far-future heap.
///
/// # Why both
///
/// Emulated machines queue almost everything within the next few thousand
/// ticks — the next scanline, the next serial bit — where a wheel gives O(1)
/// insertion and expiry. But some events are genuinely distant (a watchdog, a
/// disk seek, an RTC alarm), and a wheel wide enough for those wastes memory on
/// slots nothing ever lands in. The heap takes those, and the wheel pulls them
/// back in as they come within range.
///
/// Advancing is bounded work regardless of how far time jumps: a level's index
/// can only move through 256 slots before it has swept the whole level, so a
/// jump of a second costs the same as a jump of a millisecond.
#[derive(Debug)]
pub struct EventQueue {
    now: GlobalTime,
    now_granule: u128,
    granule_shift: u32,
    /// `WHEEL_LEVELS × WHEEL_SLOTS` buckets, level-major.
    near: Vec<Vec<Event>>,
    /// Entries per level, so an empty level is skipped without scanning it.
    level_len: [usize; WHEEL_LEVELS],
    far: BinaryHeap<Reverse<Event>>,
    /// Events at or before `now_granule`, ordered; the exact-time filter is
    /// applied when popping.
    due: BinaryHeap<Reverse<Event>>,
    cancelled: BTreeSet<u64>,
    next_seq: u64,
}

impl Default for EventQueue {
    fn default() -> Self {
        EventQueue::new(DEFAULT_GRANULE_SHIFT)
    }
}

impl EventQueue {
    /// An empty queue whose wheel granule is `2^granule_shift` units of
    /// [`GlobalTime`] — that is, `2^(granule_shift − 64)` seconds.
    ///
    /// The shift is clamped to at most 96 so the granule arithmetic stays
    /// meaningful.
    pub fn new(granule_shift: u32) -> EventQueue {
        let mut near = Vec::with_capacity(WHEEL_LEVELS * WHEEL_SLOTS);
        near.resize_with(WHEEL_LEVELS * WHEEL_SLOTS, Vec::new);
        EventQueue {
            now: GlobalTime::ZERO,
            now_granule: 0,
            granule_shift: granule_shift.min(96),
            near,
            level_len: [0; WHEEL_LEVELS],
            far: BinaryHeap::new(),
            due: BinaryHeap::new(),
            cancelled: BTreeSet::new(),
            next_seq: 0,
        }
    }

    /// The queue's current position.
    #[inline]
    pub const fn now(&self) -> GlobalTime {
        self.now
    }

    /// Posts an event and returns its handle.
    ///
    /// An event whose time is already past is not dropped: it fires at the next
    /// [`EventQueue::pop_due`], still in sequence order. Losing it silently
    /// would turn a one-tick scheduling slip into a missing interrupt.
    pub fn schedule(&mut self, time: GlobalTime, target: EventTarget, token: u64) -> EventId {
        let id = EventId(self.next_seq);
        self.next_seq += 1;
        self.push_entry(Event {
            time,
            id,
            target,
            token,
        });
        id
    }

    /// Cancels an event.
    ///
    /// The entry is tombstoned rather than hunted down: a queue that has to find
    /// an arbitrary element is a queue that cannot be a wheel. The memory is
    /// reclaimed when the event's instant is reached.
    pub fn cancel(&mut self, id: EventId) {
        self.cancelled.insert(id.0);
    }

    /// The earliest instant at which anything could fire, if anything is queued.
    ///
    /// Exact, not a hint: the levels of the wheel are ordered, so the first
    /// non-empty slot at the lowest non-empty level holds the earliest entries.
    pub fn next_deadline(&mut self) -> Option<GlobalTime> {
        self.purge_cancelled_due();
        if let Some(Reverse(e)) = self.due.peek() {
            return Some(e.time);
        }
        for level in 0..WHEEL_LEVELS {
            if self.level_len[level] == 0 {
                continue;
            }
            let shift = WHEEL_SLOT_BITS * level as u32;
            let base = self.now_granule >> shift;
            for step in 1..WHEEL_SLOTS as u128 {
                let slot = ((base + step) & (WHEEL_SLOTS as u128 - 1)) as usize;
                let bucket = &self.near[level * WHEEL_SLOTS + slot];
                let earliest = bucket
                    .iter()
                    .filter(|e| !self.cancelled.contains(&e.id.0))
                    .map(|e| e.time)
                    .min();
                if earliest.is_some() {
                    return earliest;
                }
            }
        }
        self.far.peek().map(|Reverse(e)| e.time)
    }

    /// Moves the queue's position forward, cascading the wheel.
    ///
    /// Never moves backwards: virtual time is monotone by definition.
    pub fn advance_to(&mut self, to: GlobalTime) {
        if to <= self.now {
            return;
        }
        let old_granule = self.now_granule;
        let new_granule = to.raw() >> self.granule_shift;
        self.now = to;
        if new_granule == old_granule {
            return;
        }
        self.now_granule = new_granule;

        // Level 0 first: everything in the granules just passed is due. Doing
        // this before the cascades is what keeps the two from colliding —
        // anything a cascade re-places is, by construction, strictly ahead of
        // the range swept here.
        let steps = (new_granule - old_granule).min(WHEEL_SLOTS as u128);
        for step in 1..=steps {
            let slot = ((old_granule + step) & (WHEEL_SLOTS as u128 - 1)) as usize;
            self.level_len[0] -= self.near[slot].len();
            let drained = core::mem::take(&mut self.near[slot]);
            for e in drained {
                self.due.push(Reverse(e));
            }
        }

        // Then the upper levels, each entry re-placed against the new position.
        for level in 1..WHEEL_LEVELS {
            let shift = WHEEL_SLOT_BITS * level as u32;
            let old_index = old_granule >> shift;
            let new_index = new_granule >> shift;
            if old_index == new_index {
                continue;
            }
            let steps = (new_index - old_index).min(WHEEL_SLOTS as u128);
            for step in 1..=steps {
                let slot = ((old_index + step) & (WHEEL_SLOTS as u128 - 1)) as usize;
                let idx = level * WHEEL_SLOTS + slot;
                self.level_len[level] -= self.near[idx].len();
                let drained = core::mem::take(&mut self.near[idx]);
                for e in drained {
                    self.push_entry(e);
                }
            }
        }

        // Finally pull anything that has come within the wheel's span. The heap
        // is ordered, so the first entry that does not qualify ends the sweep.
        while let Some(Reverse(top)) = self.far.peek() {
            if matches!(self.placement(top.time), Placement::Far) {
                break;
            }
            let Reverse(e) = self.far.pop().expect("just peeked");
            self.push_entry(e);
        }
    }

    /// Advances to `now` and returns the next event due at or before it, in
    /// `(time, sequence)` order.
    pub fn pop_due(&mut self, now: GlobalTime) -> Option<Event> {
        self.advance_to(now);
        loop {
            let due_now = matches!(self.due.peek(), Some(Reverse(e)) if e.time <= now);
            if !due_now {
                return None;
            }
            let Reverse(e) = self.due.pop().expect("just peeked");
            if self.cancelled.remove(&e.id.0) {
                continue;
            }
            return Some(e);
        }
    }

    /// The sequence number the next posted event will carry.
    ///
    /// Snapshot state, not a diagnostic. The number is the tie-break, so a
    /// restored queue that started counting again from zero would order events
    /// posted after the restore *before* the ones it restored — and two events
    /// at the same instant would fire in the wrong order for the rest of the
    /// run (`ROADMAP.md` §4.5).
    #[inline]
    pub const fn next_seq(&self) -> u64 {
        self.next_seq
    }

    /// Every live event, in the exact order it will fire.
    ///
    /// The queue is a wheel plus two heaps, so its internal layout is a
    /// function of the history of `advance_to` calls rather than of the events
    /// alone; enumerating it in `(time, sequence)` order — the same total order
    /// [`EventQueue::pop_due`] uses — is what makes the output a function of
    /// the queue's *contents* and therefore reproducible.
    ///
    /// Cancelled entries are omitted. A tombstone is bookkeeping for a queue
    /// that cannot delete from the middle of a wheel, not architectural state:
    /// an event that will never fire has no observable consequence, and
    /// cancelling its id again after a restore is harmless.
    pub fn events(&self) -> Vec<Event> {
        let mut out = Vec::with_capacity(self.len());
        let live = |e: &Event| !self.cancelled.contains(&e.id.0);
        out.extend(
            self.due
                .iter()
                .map(|Reverse(e)| e)
                .filter(|e| live(e))
                .cloned(),
        );
        out.extend(self.near.iter().flatten().filter(|e| live(e)).cloned());
        out.extend(
            self.far
                .iter()
                .map(|Reverse(e)| e)
                .filter(|e| live(e))
                .cloned(),
        );
        // Ids are unique, so `(time, seq)` is a total order and the sort needs
        // no stability to be deterministic.
        out.sort_unstable();
        out
    }

    /// Replaces the queue's whole contents and position.
    ///
    /// The inverse of [`EventQueue::events`] plus [`EventQueue::next_seq`]:
    /// restoring both is what makes a save/load round-trip fire the same events
    /// at the same instants in the same order. Re-deriving the queue by asking
    /// devices to re-register instead would lose sub-tick phase, and every
    /// timer would then fail its own round-trip test (`ROADMAP.md` §4.5).
    ///
    /// Events whose instant is already past are kept, not dropped: they fire at
    /// the next [`EventQueue::pop_due`], exactly as they would have without the
    /// save.
    ///
    /// # Errors
    ///
    /// [`SchedError::InvalidSnapshot`] if two events share a sequence number,
    /// or if any is at or above `next_seq` — either would let a later event
    /// win a tie against an earlier one.
    pub fn restore(&mut self, now: GlobalTime, next_seq: u64, events: &[Event]) -> SchedResult<()> {
        let mut seen = BTreeSet::new();
        for e in events {
            if e.id.0 >= next_seq {
                return Err(SchedError::InvalidSnapshot(
                    "an event's sequence number is not below the next sequence number",
                ));
            }
            if !seen.insert(e.id.0) {
                return Err(SchedError::InvalidSnapshot(
                    "two events share a sequence number",
                ));
            }
        }

        for bucket in &mut self.near {
            bucket.clear();
        }
        self.level_len = [0; WHEEL_LEVELS];
        self.far.clear();
        self.due.clear();
        self.cancelled.clear();
        self.now = now;
        self.now_granule = now.raw() >> self.granule_shift;
        self.next_seq = next_seq;
        for e in events {
            self.push_entry(e.clone());
        }
        Ok(())
    }

    /// The number of queued events, cancelled-but-not-yet-reached ones included.
    pub fn len(&self) -> usize {
        let near: usize = self.level_len.iter().sum();
        near + self.far.len() + self.due.len()
    }

    /// Whether anything at all is queued.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn purge_cancelled_due(&mut self) {
        while let Some(Reverse(e)) = self.due.peek() {
            if !self.cancelled.contains(&e.id.0) {
                return;
            }
            let id = e.id.0;
            self.due.pop();
            self.cancelled.remove(&id);
        }
    }

    /// Which bucket an instant belongs in, relative to the current position.
    ///
    /// A level holds entries whose *block index at that level* is 1..=255 ahead
    /// of the current one. Choosing by block index rather than by raw distance
    /// is what guarantees no two blocks ever share a slot, and it makes the
    /// levels strictly ordered in time — which is what lets
    /// [`EventQueue::next_deadline`] stop at the first non-empty level.
    fn placement(&self, time: GlobalTime) -> Placement {
        if time <= self.now {
            return Placement::Due;
        }
        let granule = time.raw() >> self.granule_shift;
        for level in 0..WHEEL_LEVELS {
            let shift = WHEEL_SLOT_BITS * level as u32;
            let delta = (granule >> shift) - (self.now_granule >> shift);
            if delta == 0 {
                return Placement::Due;
            }
            if delta < WHEEL_SLOTS as u128 {
                let slot = ((granule >> shift) & (WHEEL_SLOTS as u128 - 1)) as usize;
                return Placement::Near(level, slot);
            }
        }
        Placement::Far
    }

    fn push_entry(&mut self, e: Event) {
        match self.placement(e.time) {
            Placement::Due => self.due.push(Reverse(e)),
            Placement::Near(level, slot) => {
                self.near[level * WHEEL_SLOTS + slot].push(e);
                self.level_len[level] += 1;
            }
            Placement::Far => self.far.push(Reverse(e)),
        }
    }
}

// ---------------------------------------------------------------------------
// runnables and lazily-advanced devices
// ---------------------------------------------------------------------------

/// A handle to a registered [`Runnable`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RunnableId(u32);

impl RunnableId {
    /// The handle's index.
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A handle to a registered [`LazyDevice`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LazyId(u32);

impl LazyId {
    /// The handle's index.
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// What a runnable is allowed to do before it must return.
///
/// Both limits apply; whichever binds first wins. `ticks` is expressed in the
/// runnable's own clock domain and is derived exactly from `until`, so a
/// runnable may work in either currency without converting between them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Budget {
    /// The virtual instant not to run past.
    pub until: GlobalTime,
    /// The maximum number of ticks of the runnable's own domain to consume.
    pub ticks: u64,
}

impl Budget {
    /// A budget of `ticks` with no deadline.
    ///
    /// The level-3 form (`ROADMAP.md` §2.1): a guest thread's quantum is a
    /// count of executed ticks, because that is the currency that is the same
    /// on every host. There is no virtual instant to stop at, because a level-3
    /// run has no devices to keep an appointment with — so `until` is
    /// [`GlobalTime::MAX`] and `ticks` is the only limit that binds.
    #[must_use]
    pub const fn of(ticks: u64) -> Budget {
        Budget {
            until: GlobalTime::MAX,
            ticks,
        }
    }
}

/// What a runnable actually did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Consumed {
    /// Ticks of the runnable's own domain that were consumed.
    ///
    /// Never more than the budget: overrunning is a fatal error, because the
    /// overrun has already executed past an event that should have stopped it.
    /// Fewer is fine and normal — a halted CPU consumes nothing.
    pub ticks: u64,
}

impl Consumed {
    /// A run that consumed `ticks`.
    #[inline]
    pub const fn new(ticks: u64) -> Consumed {
        Consumed { ticks }
    }
}

// ---------------------------------------------------------------------------
// safe points and stop-the-world
// ---------------------------------------------------------------------------

/// The stop-the-world protocol: a generation counter and a per-runnable exit
/// flag (`ROADMAP.md` §4.7).
///
/// A TLB shootdown, a memory-topology change, a snapshot and a reset all need
/// every runnable quiescent. `ROADMAP.md` §4.7 fixes the mechanism and rules
/// out the obvious alternative in the same sentence: *"a generation counter
/// plus a per-CPU exit flag checked at translation-block boundaries — never a
/// host signal, because wasm has none and signals are miserable on Windows."*
///
/// # How the two halves fit
///
/// * [`SafePoint::request`] raises the world flag and bumps the generation. A
///   runnable sees [`ExitFlag::raised`] at its next block boundary, returns
///   what it consumed, and the scheduler collects it at the quantum's
///   rendezvous — which under [`ThreadingMode::Parallel`] is the join at the
///   end of [`Scheduler::run_quantum`] and under
///   [`ThreadingMode::Deterministic`] is simply the return of the round.
/// * The **generation** is what makes a stop distinguishable from the previous
///   one. A cache keyed on it — a TLB, a decoded page table, a host pointer —
///   is invalidated by the number changing rather than by a flag that has been
///   raised and lowered while nobody was looking.
///
/// A runnable that ignores its flag is not incorrect: it stops at the quantum
/// boundary instead of at the next block, so the stop takes up to one quantum
/// rather than up to one block. Honouring it is a latency optimisation with a
/// correctness consequence only for a machine whose quantum is long.
///
/// The cores that consult it today are the ones the machine layer hands a
/// [`TickCursor`] to and that keep it: **MOS 6502, SM83 and RISC-V**. A core
/// opts in by implementing
/// [`Device::attach_cursor`](crate::core::device::Device::attach_cursor) — the
/// hook is already called for every runnable device — and asking
/// [`TickCursor::exit_requested`] where it would stop anyway.
///
/// # Determinism
///
/// Nothing raises a flag during an ordinary run, so a
/// [`ThreadingMode::Deterministic`] machine that nobody stops executes exactly
/// the sequence it executed before this existed. That is deliberate: the mode
/// whose state hash is a golden must not acquire a new way to diverge.
#[derive(Debug, Clone, Default)]
pub struct SafePoint {
    world: Arc<World>,
}

/// What every runnable on one machine shares.
#[derive(Debug, Default)]
struct World {
    /// Bumped by every request, so a stop is distinguishable from the one
    /// before it.
    generation: AtomicU64,
    /// Whether a stop is outstanding right now.
    stop: AtomicBool,
}

impl SafePoint {
    /// A protocol with nothing requested.
    #[must_use]
    pub fn new() -> SafePoint {
        SafePoint::default()
    }

    /// Ask every runnable to unwind at its next block boundary.
    ///
    /// Returns the generation this request carries. Idempotent: a second
    /// request while one is outstanding still bumps the generation, because a
    /// second reason to stop is a second reason to invalidate.
    pub fn request(&self) -> u64 {
        let generation = self
            .world
            .generation
            .fetch_add(1, AtomicOrdering::AcqRel)
            .wrapping_add(1);
        self.world.stop.store(true, AtomicOrdering::Release);
        generation
    }

    /// Let the world run again.
    pub fn release(&self) {
        self.world.stop.store(false, AtomicOrdering::Release);
    }

    /// Whether a stop is outstanding.
    #[inline]
    #[must_use]
    pub fn stop_requested(&self) -> bool {
        self.world.stop.load(AtomicOrdering::Acquire)
    }

    /// The generation of the most recent request.
    ///
    /// Starts at zero and only ever rises, so a cache that remembers the value
    /// it was built under knows it is stale by comparing rather than by being
    /// told.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.world.generation.load(AtomicOrdering::Acquire)
    }

    /// A fresh per-runnable flag on this protocol.
    #[must_use]
    pub fn flag(&self) -> ExitFlag {
        ExitFlag {
            world: Arc::clone(&self.world),
            mine: Arc::new(AtomicBool::new(false)),
        }
    }
}

/// One runnable's exit flag: *stop at your next block boundary*.
///
/// Handed to a core through its [`TickCursor`], which every runnable device
/// already receives, so honouring it costs a core one relaxed load per block
/// and no change to any signature. See [`SafePoint`] for the protocol.
#[derive(Debug, Clone)]
pub struct ExitFlag {
    world: Arc<World>,
    mine: Arc<AtomicBool>,
}

impl Default for ExitFlag {
    /// A flag on a protocol of its own — which is what a standalone
    /// [`TickCursor`] wants, and what a test fixture gets.
    fn default() -> ExitFlag {
        SafePoint::new().flag()
    }
}

impl ExitFlag {
    /// Whether this runnable should unwind to the scheduler now.
    ///
    /// Two relaxed loads, deliberately. The ordering that matters is the
    /// quantum's rendezvous — the pool join, or the return of the round — which
    /// is a real happens-before edge; this load only has to become visible
    /// *eventually*, and paying for an acquire at every block boundary to make
    /// it visible a few hundred cycles sooner would be paying in the one place
    /// the machine cannot afford it.
    #[inline]
    #[must_use]
    pub fn raised(&self) -> bool {
        self.world.stop.load(AtomicOrdering::Relaxed) || self.mine.load(AtomicOrdering::Relaxed)
    }

    /// Ask this one runnable to unwind, leaving the rest running.
    ///
    /// What a debugger's single-step wants, and what a device that needs one
    /// particular CPU out of the way wants. A world stop is
    /// [`SafePoint::request`].
    pub fn raise(&self) {
        self.mine.store(true, AtomicOrdering::Release);
    }

    /// Clear this runnable's own flag. Does not clear a world stop.
    pub fn clear(&self) {
        self.mine.store(false, AtomicOrdering::Release);
    }

    /// The generation of the most recent world stop.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.world.generation.load(AtomicOrdering::Acquire)
    }

    /// The protocol this flag belongs to.
    #[must_use]
    pub fn safe_point(&self) -> SafePoint {
        SafePoint {
            world: Arc::clone(&self.world),
        }
    }
}

/// The world held stopped, released when this is dropped.
///
/// Returned by [`Scheduler::stop_the_world`]. While it is alive every runnable
/// that honours its [`ExitFlag`] declines to start another block, and the task
/// pool has been quiesced — so a snapshot, a remap or a reset taken here sees a
/// machine nobody is executing.
#[derive(Debug)]
pub struct StopGuard {
    safe: SafePoint,
    generation: u64,
}

impl StopGuard {
    /// The generation this stop carries.
    #[inline]
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }
}

impl Drop for StopGuard {
    fn drop(&mut self) {
        self.safe.release();
    }
}

/// Where a runnable has got to **inside** the quantum it is running.
///
/// A [`Runnable`] reports what it consumed only when it returns, so for the
/// length of one `run` call the clock forest still stands where the quantum
/// began. That is fine for a device nobody is looking at and wrong for one that
/// is sampled: a 6502 reading `$2002` on the ninth cycle of a budget needs the
/// PPU on the dot that cycle really is, not on the dot the quantum started at.
///
/// So a core publishes its own cycle counter here as it runs, and the scheduler
/// converts that into every lazily-advanced device's domain through the
/// oscillator tree they share -- exact integer arithmetic, never absolute time
/// (`ROADMAP.md` 4.2). This is the "letting a runnable report progress *as* it
/// runs" that [`LazyHandle`] names as the proper resolution of its own
/// staleness.
///
/// Publishing is optional: a core that never touches its cursor leaves catch-up
/// exactly where it was, bounded by the quantum.
#[derive(Debug, Clone, Default)]
pub struct TickCursor {
    inner: Arc<CursorInner>,
}

/// What a cursor shares between the runnable and the scheduler.
#[derive(Debug, Default)]
struct CursorInner {
    /// The runnable's own tick counter.
    ticks: AtomicU64,
    /// The first tick at which some lazily-advanced device has an event of its
    /// own, in the *runnable's* ticks. `u64::MAX` when none has one.
    deadline: AtomicU64,
    /// The slots to catch up when that tick arrives.
    slots: Mutex<Option<Arc<[Arc<LazySlot>]>>>,
    /// *Unwind at your next block boundary* (`ROADMAP.md` §4.7). Carried here
    /// rather than on [`Budget`] because every runnable device is already
    /// handed a cursor and no core's signature has to change to consult one.
    exit: ExitFlag,
}

impl TickCursor {
    /// A fresh cursor at zero, on a safe-point protocol of its own.
    #[must_use]
    pub fn new() -> TickCursor {
        TickCursor::with_exit(ExitFlag::default())
    }

    /// A fresh cursor at zero, sharing `exit`'s protocol.
    ///
    /// What [`Scheduler::add_runnable`] builds, so every runnable on one
    /// machine sees the same world stop.
    #[must_use]
    pub fn with_exit(exit: ExitFlag) -> TickCursor {
        TickCursor {
            inner: Arc::new(CursorInner {
                ticks: AtomicU64::new(0),
                deadline: AtomicU64::new(u64::MAX),
                slots: Mutex::new(None),
                exit,
            }),
        }
    }

    /// Whether this runnable has been asked to unwind to the scheduler.
    ///
    /// The block-boundary check of `ROADMAP.md` §4.7's safe-point protocol. A
    /// core consults it where it would naturally stop anyway — between
    /// instructions, at the end of a translation block — and returns what it
    /// has consumed so far. Returning less than the budget is always legal
    /// ([`Runnable::run`]), so this needs no new contract.
    ///
    /// Nothing raises it during an ordinary run, so a core that checks it is
    /// bit-identical to one that does not until somebody actually stops the
    /// world.
    #[inline]
    #[must_use]
    pub fn exit_requested(&self) -> bool {
        self.inner.exit.raised()
    }

    /// This runnable's exit flag, for a caller that wants to stop exactly one.
    #[must_use]
    pub fn exit_flag(&self) -> ExitFlag {
        self.inner.exit.clone()
    }

    /// Publish the runnable's own tick counter.
    ///
    /// Monotonic and free-running -- it is the core's ticks-since-power-on, not
    /// an offset into the budget, so nothing has to be reset between quanta and
    /// a core carrying cycle debt still reports the truth.
    ///
    /// **This is also where a lazily-advanced device's own event lands.** A
    /// vblank NMI is caused by nothing the core did, so nothing on the access
    /// path will ever ask for it; if the core is running a long stretch that
    /// touches no PPU register the flag would otherwise not be raised until the
    /// quantum ended, tens of cycles late. So the cursor knows the next tick at
    /// which some device has an event, and crossing it catches every one of
    /// them up right here — inside the cycle, before the core samples its pins.
    #[inline]
    pub fn set(&self, ticks: u64) {
        self.inner.ticks.store(ticks, AtomicOrdering::Relaxed);
        if ticks >= self.inner.deadline.load(AtomicOrdering::Relaxed) {
            self.reach(ticks);
        }
    }

    /// What was last published.
    #[inline]
    #[must_use]
    pub fn get(&self) -> u64 {
        self.inner.ticks.load(AtomicOrdering::Relaxed)
    }

    /// A device's event tick has arrived: catch every device up and work out
    /// where the next one is.
    ///
    /// Cold on purpose. It runs a handful of times per scanline on a NES, and
    /// the common path is one relaxed load.
    #[cold]
    fn reach(&self, ticks: u64) {
        // Cloned out: the slot list is fixed after realize, and holding a leaf
        // lock across `LazySlot::sync` — which takes another leaf — is the
        // order violation `core::sync` exists to catch.
        let slots = self.inner.slots.lock().clone();
        let Some(slots) = slots else {
            self.inner.deadline.store(u64::MAX, AtomicOrdering::Relaxed);
            return;
        };
        let mut next = u64::MAX;
        for slot in slots.iter() {
            let _ = slot.sync(slot.id, None, AccessKind::Guest);
            if let Some(at) = slot.cursor_deadline(ticks) {
                next = next.min(at.max(ticks + 1));
            }
        }
        self.inner.deadline.store(next, AtomicOrdering::Relaxed);
    }

    /// Point the cursor at the devices it should keep in step, and recompute
    /// the first tick at which one of them has something to do.
    fn watch(&self, slots: Option<Arc<[Arc<LazySlot>]>>) {
        let mut next = u64::MAX;
        if let Some(slots) = &slots {
            let now = self.get();
            for slot in slots.iter() {
                if let Some(at) = slot.cursor_deadline(now) {
                    next = next.min(at.max(now));
                }
            }
        }
        *self.inner.slots.lock() = slots;
        self.inner.deadline.store(next, AtomicOrdering::Relaxed);
    }
}

/// A lazy slot's view of the runnable that is executing right now.
///
/// Armed by the scheduler immediately before a `run` call and disarmed after
/// it, so it exists only while there is a live position to convert. The ratio
/// is in oscillator units of the tree both domains hang off -- an intra-tree
/// relationship, which is exact.
#[derive(Debug, Clone)]
struct Live {
    cursor: TickCursor,
    /// The runnable's tick counter when the run call began.
    base_cursor: u64,
    /// This slot's domain position at that same instant.
    base_tick: u64,
    /// Tree units per tick of the *runnable's* domain.
    mul: u64,
    /// Tree units per tick of *this slot's* domain.
    div: u64,
}

impl Live {
    /// Where this slot's domain stands, given what the runnable has published.
    fn present(&self) -> u64 {
        let elapsed = self.cursor.get().saturating_sub(self.base_cursor);
        // `elapsed * mul` converts ticks to tree units; dividing by this
        // domain's units-per-tick lands in its ticks. Both factors come from
        // one oscillator, so there is no rounding to accumulate.
        let units = elapsed.saturating_mul(self.mul);
        self.base_tick.saturating_add(units / self.div)
    }
}

/// Something the scheduler gives execution budgets to: a CPU, a DMA engine, a
/// coprocessor.
///
/// `Send + Sync` from the first commit, because retrofitting it later is a
/// rewrite (`ROADMAP.md` §0, §4.7).
pub trait Runnable: Send + Sync {
    /// Runs until the budget is exhausted and reports what was consumed.
    ///
    /// Returning less than the budget is legitimate — a halt, a wait-for-
    /// interrupt, a natural block boundary, or a raised
    /// [`ExitFlag`]. Returning more is a bug and the scheduler treats it as
    /// one.
    ///
    /// # How many times this is called
    ///
    /// **Once per runnable per round, in every threading mode.** A runnable may
    /// therefore do per-call work — a UART that pumps its port once — without
    /// that number depending on how a caller sliced the run.
    /// [`ThreadingMode::Parallel`] submits one job per runnable and joins them
    /// at the round's end; it does not split a round into passes, which is the
    /// thing that used to make the count caller-dependent (§11.6).
    ///
    /// What it does not promise is *isolation*: under
    /// [`ThreadingMode::Parallel`] this runs while other runnables are running
    /// and while guest accesses to the same device are in flight. See
    /// [`Device::run`](crate::core::device::Device::run) for what that means
    /// for a device that is a runnable.
    fn run(&mut self, budget: Budget) -> Consumed;
}

/// A device that is advanced only when somebody looks at it.
///
/// The PPU is the motivating case: it is far cheaper to run it in bursts than
/// dot by dot, but a CPU read of a status register has to see the state at
/// exactly that dot. So the device keeps its own tick, and the access path
/// catches it up before dispatching an access to it — through a
/// [`LazyHandle`], which is reachable from a `&self` memory operation, or
/// through [`Scheduler::sync_for_access`] where the scheduler itself is in
/// hand.
pub trait LazyDevice: Send + Sync {
    /// The tick, in the device's own clock domain, that it has simulated up to.
    fn current_tick(&self) -> u64;

    /// Simulates forward to `tick`. Never called with a tick in the past.
    fn advance_to(&mut self, tick: u64);

    /// The device's own next internal event, if it has one.
    ///
    /// Catch-up never crosses it: past that tick the device's behaviour changes,
    /// and simulating through it in one step would compute the wrong answer.
    /// `None` means "nothing pending", and catch-up runs to the present.
    fn next_event_tick(&self) -> Option<u64> {
        None
    }

    /// Whether this device must be caught up on every tick of the runnable that
    /// is executing, rather than only at its own next event.
    ///
    /// See [`crate::core::device::Device::sampled_every_cycle`], which is where
    /// this is documented and where a device declares it.
    fn sampled_every_cycle(&self) -> bool {
        false
    }
}

/// Why a device is being accessed.
///
/// A debug access must not change anything — not a FIFO, not a status bit, and
/// not the clock (`ROADMAP.md` §15, invariant 5). Mapping
/// [`MemAttrs::debug`](crate::core::space::MemAttrs) onto this is the access
/// path's job: the scheduler takes the distinction directly rather than
/// depending on the address space, so that `core::sched` stays independent of
/// `core::space`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccessKind {
    /// A guest access. The device is caught up first.
    #[default]
    Guest,
    /// A debugger or monitor access. Nothing is advanced.
    Debug,
}

// ---------------------------------------------------------------------------
// threading modes and rate control
// ---------------------------------------------------------------------------

/// How guest execution is spread over host threads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum ThreadingMode {
    /// One host thread, round-robin over runnables with a fixed quantum.
    ///
    /// Required for record/replay and the regression suite, and the only mode
    /// implemented here.
    #[default]
    Deterministic,
    /// A thread per CPU with a rendezvous barrier per quantum.
    ///
    /// Fast, **non-deterministic**, and the intended default for interactive
    /// use. One job per runnable goes to the `core::sync` task pool at the
    /// start of a round and the round ends when every one of them has been
    /// joined — that join *is* the rendezvous barrier, and it is also the
    /// happens-before edge that makes the next round's bookkeeping see
    /// everything the last round's runnables wrote.
    ///
    /// What it gives up is stated rather than hedged: two CPUs that observe
    /// each other through memory observe each other at host-timing-dependent
    /// instants, so the state hash is **not** reproducible.
    /// [`ThreadingMode::is_deterministic`] is false here, and
    /// [`Machine::state_hash`](crate::machine::Machine::state_hash) refuses
    /// rather than returning a number a regression suite would then bless.
    ///
    /// Everything that is *not* given up: virtual time, the event queue, the
    /// per-tree tick counters and every budget still come from the same
    /// absolute grid the deterministic mode uses, so a machine does not run
    /// *faster* in guest time, only in host time.
    ///
    /// On a backend with no threads — the no-threads browser build, bare metal
    /// — the pool runs jobs inline and this degenerates to submission order.
    /// That is a supported configuration and not a fallback (§11.3): it is
    /// slower than [`ThreadingMode::Deterministic`] by one allocation per
    /// runnable per round and otherwise identical.
    ///
    /// # What it is actually worth, measured
    ///
    /// A round costs a dispatch per runnable — a queue push, a wake and a wait
    /// — of the order of a microsecond, and that is paid whether or not there
    /// is anything to overlap. So the mode is a **loss** below a work-per-round
    /// threshold and a win above it, and the threshold is not small.
    ///
    /// Measured on a 32-core x86-64 Linux host, `--release`, with runnables
    /// doing genuinely serial work of about 1.5 ns per tick, as a factor
    /// against [`ThreadingMode::Deterministic`]:
    ///
    /// | ticks/round | 2 runnables | 4 | 8 |
    /// | --- | --- | --- | --- |
    /// | 1 000 | 0.2–0.5× | 0.8× | 0.8–1.6× |
    /// | 10 000 (the default cap) | 0.83× | 1.5× | 2.5× |
    /// | 100 000 | 1.0–1.5× | 1.8–2.6× | 3.5–3.7× |
    /// | 1 000 000 | 1.5× | 2.5–3.3× | 3.7× |
    ///
    /// Two conclusions worth stating plainly rather than burying:
    ///
    /// * **Two CPUs at the default [`SchedulerConfig::max_ticks_per_quantum`]
    ///   are slower in this mode than in the deterministic one.** A machine
    ///   with two cores wants a larger cap before it asks for parallelism.
    /// * The speedup saturates near 4× however many runnables there are,
    ///   because the barrier is per round and the round is only as short as its
    ///   slowest runnable. This is a rendezvous design, not a free-running one,
    ///   and §4.2 chose it deliberately.
    ///
    /// `machines/tests/heterogeneous.machine` — a 100 MHz RISC-V hart and a
    /// 1 MHz 6502 — measures **1.05–1.10×**, which is what its work ratio
    /// allows: the 6502 is about a tenth of the round's work, so Amdahl caps
    /// the board at roughly 1.1× and the implementation reaches it.
    Parallel,
    /// CPUs run in hardware and virtual time is slaved to the host clock
    /// (§4.2).
    ///
    /// Structurally [`ThreadingMode::Parallel`] — one job per runnable, the
    /// same rendezvous at the round's end, the same non-reproducibility — with
    /// **one** difference, which is the whole mode: a round's elapsed virtual
    /// time is read off the injected [`HostClock`] instead of being computed
    /// from what the runnables reported.
    ///
    /// # Why an accelerated board needs it and a parallel one does not
    ///
    /// A runnable's [`Consumed`] is a count of guest ticks, and an accelerated
    /// core has none to give: a vCPU inside `KVM_RUN` executes an unknown
    /// number of instructions in a knowable amount of *host* time. So it
    /// returns its whole budget, which makes the board's clocks advance by one
    /// quantum per round no matter how long the round took. Under `Parallel`
    /// that is not a rounding error, it is a wall:
    ///
    /// * a guest that spins without exiting holds the round, and every clock
    ///   on the board stops for as long as it spins — Linux's `hpet_counting()`
    ///   reads the HPET counter twice inside one round and finds it unmoved,
    ///   and `timer_irq_works()` waits out an `mdelay()` that no tick can
    ///   interrupt, which panics `check_timer()`;
    /// * a guest that calibrates the host's time-stamp counter against a board
    ///   timer is measuring a real clock against a fictional one, and concludes
    ///   it is on a 176 THz processor.
    ///
    /// Here the wall measures the round, so both come out right, and
    /// `machines/q35-linux.machine` boots on its own command line.
    ///
    /// # What it requires, and what it gives up on top of `Parallel`
    ///
    /// * **A host clock must be injected** —
    ///   [`Machine::set_host_clock`](crate::machine::Machine::set_host_clock),
    ///   [`MonotonicClock`](crate::host::clock::MonotonicClock) — or every
    ///   round fails with [`SchedError::NoHostClock`]. There is no fallback,
    ///   because the fallback would be a guess.
    /// * **A caller's deadline no longer declines a round.** Under `Parallel`
    ///   a deadline falling inside a round defers the whole round, which is
    ///   what makes [`Machine::run_for`](crate::machine::Machine::run_for)
    ///   additive (§11.6). That property is not available once rounds are cut
    ///   by the host clock, and declining would mean not entering the guest at
    ///   all, so the deadline bounds the budgets instead.
    /// * **Reported tick counts stop moving the clock.** An interpreted
    ///   runnable sharing an accelerated board still gets a budget from its
    ///   own tree's absolute position, but its tree is then dragged to the
    ///   wall like everyone else's — so it executes at whatever rate the host
    ///   can manage rather than at its declared frequency. A board with an
    ///   interpreted core whose timing matters is a board to run in one of the
    ///   other two modes.
    Accel,
}

impl ThreadingMode {
    /// Whether a run in this mode is bit-reproducible.
    ///
    /// True only for [`ThreadingMode::Deterministic`]. `ROADMAP.md` §0 makes
    /// determinism a property of *the mode*, not of the thread count, and §4.2
    /// says parallel execution is non-deterministic in as many words — so this
    /// is the predicate everything that depends on reproducibility asks, rather
    /// than each caller re-deciding what "deterministic enough" means.
    #[inline]
    #[must_use]
    pub const fn is_deterministic(self) -> bool {
        matches!(self, ThreadingMode::Deterministic)
    }
}

impl fmt::Display for ThreadingMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ThreadingMode::Deterministic => "deterministic",
            ThreadingMode::Parallel => "parallel",
            ThreadingMode::Accel => "accel",
        })
    }
}

/// How fast virtual time is allowed to run against wall time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum RateControl {
    /// As fast as the host can manage. The default, and the only one that needs
    /// no host clock.
    #[default]
    Unbounded,
    /// Throttled to wall time.
    Realtime {
        /// How far behind wall time the machine may fall before the debt is
        /// written off instead of chased.
        ///
        /// Without a limit, a host that stalls for a second leaves the guest
        /// owing a second of catch-up, which it then runs at maximum speed —
        /// audio breaks up and input lags. Forgiving the debt is the honest
        /// behaviour, and it is a policy the machine states rather than one the
        /// scheduler invents.
        max_catchup_nanos: u64,
    },
    /// A fixed fraction of wall time: `num/den`, so `1/2` is half speed for
    /// debugging and `2/1` is double.
    FixedRatio {
        /// Numerator of the virtual-to-real rate.
        num: u64,
        /// Denominator of the virtual-to-real rate. Must not be zero.
        den: u64,
    },
}

/// The host's monotonic clock, injected rather than named.
///
/// Nothing under `core/` may read the host clock directly (`ROADMAP.md` §15,
/// invariant 4): a real implementation lives above the `std` line, in `host/`.
/// Injecting it is also what makes rate control testable — a test hands in a
/// clock it controls and the result is deterministic.
pub trait HostClock: Send + Sync {
    /// Nanoseconds since some fixed, arbitrary origin. Must be monotonic.
    fn monotonic_nanos(&self) -> u64;
}

/// What the rate controller wants the caller to do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pace {
    /// Keep running.
    Run,
    /// Virtual time is ahead of the wall by this many nanoseconds.
    ///
    /// The scheduler does not sleep — it cannot, without naming a host facility
    /// — so it says how long and the host loop decides how to wait.
    Wait {
        /// How far ahead the machine is.
        nanos: u64,
    },
}

/// Throttles virtual time against wall time, in integers only.
///
/// Deliberately a pure function of `(host nanos, virtual instant)` plus its own
/// origin: the same inputs always produce the same decision, and the decision
/// never touches guest state.
#[derive(Debug, Clone)]
pub struct RateController {
    control: RateControl,
    origin_host: u64,
    origin_virtual: GlobalTime,
}

impl RateController {
    /// A controller with the given policy, not yet anchored.
    pub fn new(control: RateControl) -> RateController {
        RateController {
            control,
            origin_host: 0,
            origin_virtual: GlobalTime::ZERO,
        }
    }

    /// The policy in force.
    #[inline]
    pub const fn control(&self) -> RateControl {
        self.control
    }

    /// Replaces the policy and re-anchors.
    pub fn set_control(&mut self, control: RateControl, host_nanos: u64, now: GlobalTime) {
        self.control = control;
        self.reset(host_nanos, now);
    }

    /// Anchors the controller: from here, virtual and wall time are level.
    pub fn reset(&mut self, host_nanos: u64, now: GlobalTime) {
        self.origin_host = host_nanos;
        self.origin_virtual = now;
    }

    /// Decides whether to keep running.
    ///
    /// Integer nanoseconds throughout; the ratio is applied as a `u128` product
    /// so a long run cannot overflow into a wrong decision.
    pub fn pace(&mut self, host_nanos: u64, now: GlobalTime) -> Pace {
        let virtual_ns = now.saturating_sub(self.origin_virtual).as_nanos();
        let host_ns = host_nanos.saturating_sub(self.origin_host);
        let allowance = match self.control {
            RateControl::Unbounded => return Pace::Run,
            RateControl::Realtime { max_catchup_nanos } => {
                if host_ns.saturating_sub(virtual_ns) > max_catchup_nanos {
                    // Too far behind to chase: write the debt off rather than
                    // sprint through it.
                    self.origin_host = host_nanos;
                    self.origin_virtual = now;
                    return Pace::Run;
                }
                host_ns
            }
            RateControl::FixedRatio { num, den } => {
                if den == 0 {
                    return Pace::Run;
                }
                let scaled = (host_ns as u128) * (num as u128) / (den as u128);
                u64::try_from(scaled).unwrap_or(u64::MAX)
            }
        };
        if virtual_ns > allowance {
            Pace::Wait {
                nanos: virtual_ns - allowance,
            }
        } else {
            Pace::Run
        }
    }
}

// ---------------------------------------------------------------------------
// the scheduler
// ---------------------------------------------------------------------------

/// How a [`Scheduler`] is set up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchedulerConfig {
    /// Threading mode.
    pub mode: ThreadingMode,
    /// How many worker threads [`ThreadingMode::Parallel`] asks the task pool
    /// for.
    ///
    /// Thread count is a machine property, never something a device decides
    /// (`ROADMAP.md` §4.7, "jobs, not threads"). Zero means *run jobs inline*,
    /// which is what a backend with no threads does anyway, and is a real
    /// answer rather than a missing one: a parallel machine on the no-threads
    /// browser build still runs.
    ///
    /// Ignored entirely by [`ThreadingMode::Deterministic`], which creates no
    /// pool at all.
    pub workers: usize,
    /// Rate control policy.
    pub rate: RateControl,
    /// The span of virtual time one round of the round-robin covers.
    ///
    /// Shorter means finer interleaving between runnables and more scheduler
    /// overhead; it does not affect correctness, because catch-up makes every
    /// access exact regardless.
    ///
    /// It *is* the grid a round ends on, though: rounds end on whole multiples
    /// of it counted from the origin, so a deadline that is a whole number of
    /// quanta lands exactly on a boundary and leaves nothing deferred. See
    /// [`Scheduler::run_quantum_until`].
    pub quantum: GlobalTime,
    /// A hard cap on ticks handed out in one budget, whatever the quantum works
    /// out to.
    pub max_ticks_per_quantum: u64,
    /// Wheel granularity, in [`GlobalTime`] bits.
    pub granule_shift: u32,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        SchedulerConfig {
            mode: ThreadingMode::Deterministic,
            workers: 0,
            rate: RateControl::Unbounded,
            quantum: DEFAULT_QUANTUM,
            max_ticks_per_quantum: 10_000,
            granule_shift: DEFAULT_GRANULE_SHIFT,
        }
    }
}

/// One millisecond: short enough that a machine feels responsive, long enough
/// that scheduling is not the bottleneck.
pub const DEFAULT_QUANTUM: GlobalTime = GlobalTime::from_nanos(1_000_000);

struct RunnableSlot {
    domain: DomainId,
    inner: Option<Box<dyn Runnable>>,
    /// The runnable's live position, if it publishes one.
    cursor: TickCursor,
}

impl fmt::Debug for RunnableSlot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RunnableSlot")
            .field("domain", &self.domain)
            .field("registered", &self.inner.is_some())
            .field("cursor", &self.cursor.get())
            .finish()
    }
}

/// What a lazy slot protects: the device, and where its domain has got to.
struct LazyState {
    device: Option<Box<dyn LazyDevice>>,
    /// The executing runnable's live position, while one is executing.
    live: Option<Live>,
    /// The tick of the slot's domain the scheduler last published.
    ///
    /// Catch-up reached from inside a memory access cannot read the clock
    /// forest — the forest belongs to whoever is driving the run loop — so the
    /// scheduler pushes each domain's position here whenever it advances time.
    present: u64,
}

/// One registered lazily-advanced device.
///
/// Behind an `Arc` so a [`LazyHandle`] can reach it from an access path that has
/// no route back to the scheduler, and behind a [`Mutex`] at the default
/// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF) because **nothing is
/// ever acquired while it is held**: catch-up takes the device *out* of the
/// slot, drops the guard, and only then calls
/// [`LazyDevice::advance_to`] — which is free to touch its own bus, its own
/// state lock, or a wire. A leaf that is only ever held across a `take` and a
/// put-back nests under every rank in the ladder, which is exactly what an
/// access already holding `BUS` needs.
struct LazySlot {
    /// This slot's own handle, so it can report which device an error is about
    /// from a path that was not given one.
    id: LazyId,
    domain: DomainId,
    state: Mutex<LazyState>,
    /// Whether finding the slot empty may mean *another thread is advancing
    /// it* rather than *I re-entered my own catch-up*.
    ///
    /// Set only under [`ThreadingMode::Parallel`] on a backend that really has
    /// threads. Under [`ThreadingMode::Deterministic`] an empty slot can only
    /// be re-entrancy, which is what
    /// [`SchedError::LazyDeviceBusy`] has always said — so this stays false
    /// there and that path keeps its exact behaviour, error and all.
    contended: AtomicBool,
}

/// How many spins a contended catch-up gives another thread before it decides
/// the emptiness is re-entrancy after all.
///
/// Generous rather than tight: the thread it is waiting for is inside
/// [`LazyDevice::advance_to`], which on a PPU is a burst of real work. Too
/// small and an honest contention becomes a spurious error under load; too
/// large and a genuine re-entrancy bug takes a visible pause before it is
/// reported. It only ever costs the pause in code that was already wrong.
const LAZY_CONTENDED_SPINS: u32 = 1 << 16;

impl fmt::Debug for LazySlot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("LazySlot");
        s.field("domain", &self.domain);
        match self.state.try_lock() {
            Some(state) => s
                .field("registered", &state.device.is_some())
                .field("present", &state.present)
                .finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl LazySlot {
    /// Records where the slot's domain has got to.
    fn publish(&self, present: u64) {
        self.state.lock().present = present;
    }

    /// Brings the device up to date, from `present` or from the last published
    /// position.
    ///
    /// The critical section covers reading the device's position and taking
    /// ownership of it, and stops there. `advance_to` runs with the device held
    /// exclusively by value and no lock held at all, which is `ROADMAP.md`
    /// §4.7's re-entrancy contract satisfied by construction rather than by
    /// good intentions.
    fn sync(&self, id: LazyId, present: Option<u64>, kind: AccessKind) -> SchedResult<u64> {
        let mut spins = 0u32;
        let (mut device, from, target) = loop {
            let mut state = self.state.lock();
            if let Some(p) = present {
                state.present = p;
            }
            // The executing runnable's live position, where there is one, is
            // ahead of what the forest has been told: the quantum has not ended
            // yet. It is the honest target -- see [`TickCursor`].
            let target = match &state.live {
                Some(live) => state.present.max(live.present()),
                None => state.present,
            };
            let Some(device) = state.device.as_ref() else {
                drop(state);
                self.wait_for_the_other_thread(id, &mut spins)?;
                continue;
            };
            let from = device.current_tick();
            if kind == AccessKind::Debug {
                return Ok(from);
            }
            if target <= from {
                return Ok(from);
            }
            let device = state.device.take().expect("borrowed successfully above");
            break (device, from, target);
        };
        // Never *through* the device's own next event in one step: beyond that
        // tick its behaviour changes. So walk to it, let it happen, and ask
        // again -- a target several events away still arrives, which a single
        // clamped step would not.
        let mut to = from;
        loop {
            let stop = target.min(device.next_event_tick().unwrap_or(u64::MAX));
            if stop <= to {
                break;
            }
            device.advance_to(stop);
            let reached = device.current_tick();
            if reached <= to {
                // No progress: an event tick that is not in the future, which
                // `Device::next_event_tick` forbids. Stop rather than spin.
                to = reached;
                break;
            }
            to = reached;
        }
        self.state.lock().device = Some(device);
        if to < from {
            return Err(SchedError::NonMonotonicDevice {
                device: id,
                from,
                to,
            });
        }
        Ok(to)
    }

    /// An empty slot: either somebody else is inside this device's
    /// `advance_to`, or this thread is.
    ///
    /// Under [`ThreadingMode::Deterministic`] there is only one thread, so an
    /// empty slot is unambiguously re-entrancy and reporting it is right —
    /// recursing would need two mutable borrows of one device and waiting would
    /// be a deadlock. Under [`ThreadingMode::Parallel`] the same emptiness is
    /// ordinarily *contention*: two CPUs reached one PPU inside the same
    /// quantum, which is not a bug and must not be reported as one.
    ///
    /// Spinning rather than blocking, because there is nothing to block on: the
    /// seam has no condition variable (`core::sync` says why), the holder is
    /// running device code with no lock held, and the wait is bounded by one
    /// `advance_to`. The bound then keeps a genuine re-entrancy bug from
    /// hanging: past it the emptiness is reported exactly as it always was.
    fn wait_for_the_other_thread(&self, id: LazyId, spins: &mut u32) -> SchedResult<()> {
        if !self.contended.load(AtomicOrdering::Relaxed) || *spins >= LAZY_CONTENDED_SPINS {
            return Err(SchedError::LazyDeviceBusy(id));
        }
        *spins += 1;
        core::hint::spin_loop();
        Ok(())
    }

    /// Arm the live view of the runnable that is about to execute.
    fn arm(&self, live: Live) {
        self.state.lock().live = Some(live);
    }

    /// The device's next event, expressed in the *running runnable's* ticks.
    ///
    /// `None` when the device has no event, or when no runnable is executing
    /// and there is therefore nothing to express it in.
    fn cursor_deadline(&self, now: u64) -> Option<u64> {
        let state = self.state.lock();
        let live = state.live.as_ref()?;
        let device = state.device.as_ref()?;
        if device.sampled_every_cycle() {
            // Nothing to convert: this one is looked at every cycle, so the
            // next cycle is the deadline.
            return Some(now + 1);
        }
        let event = device.next_event_tick()?;
        let ahead = event.saturating_sub(live.base_tick);
        // Round up: the runnable tick that *reaches* the event is the first one
        // whose converted position is at or past it.
        let units = ahead.saturating_mul(live.div);
        Some(live.base_cursor + units.div_ceil(live.mul))
    }

    /// Drop it again, so a sync between quanta uses the published position.
    fn disarm(&self) {
        self.state.lock().live = None;
    }

    /// Puts the device on a specific tick of its own domain.
    fn sync_to_tick(&self, id: LazyId, tick: u64) -> SchedResult<u64> {
        let mut spins = 0u32;
        let (mut device, from) = loop {
            let mut state = self.state.lock();
            let Some(device) = state.device.as_ref() else {
                drop(state);
                self.wait_for_the_other_thread(id, &mut spins)?;
                continue;
            };
            let from = device.current_tick();
            if tick <= from {
                return Ok(from);
            }
            let device = state.device.take().expect("borrowed successfully above");
            break (device, from);
        };
        device.advance_to(tick);
        let to = device.current_tick();
        self.state.lock().device = Some(device);
        if to < from {
            return Err(SchedError::NonMonotonicDevice {
                device: id,
                from,
                to,
            });
        }
        Ok(to)
    }

    /// The device's own next event, if it has one and it is registered.
    ///
    /// Like [`LazySlot::current_tick`] this asks the device under the slot's
    /// leaf lock, which is why [`LazyDevice::next_event_tick`] may not take one
    /// of its own.
    fn next_event_tick(&self) -> Option<u64> {
        let state = self.state.lock();
        state.device.as_ref().and_then(|d| d.next_event_tick())
    }

    /// Where the device has simulated up to, advancing nothing.
    fn current_tick(&self, id: LazyId) -> SchedResult<u64> {
        let state = self.state.lock();
        state
            .device
            .as_ref()
            .map(|d| d.current_tick())
            .ok_or(SchedError::LazyDeviceBusy(id))
    }
}

/// A shared handle to one lazily-advanced device: sync-on-access from a path
/// that cannot reach the scheduler.
///
/// This is what makes `ROADMAP.md` §4.2's sync-on-access implementable. The
/// path that must trigger catch-up is `MemOps::read`, which takes `&self` and
/// runs with the bus's own lock held, several frames below the run loop that
/// owns the scheduler. A handle is cloned to the mapping when the machine is
/// realized, and thereafter the access path calls [`LazyHandle::sync`] with no
/// borrow of, and no lock shared with, the scheduler.
///
/// # Lock order
///
/// [`LockRank::SCHED`](crate::core::sync::LockRank::SCHED) sits **above**
/// [`LockRank::BUS`](crate::core::sync::LockRank::BUS): a bus access that
/// reached back for a scheduler-ranked lock would invert the ladder, and two
/// CPUs doing it on different buses is a textbook deadlock. So nothing on this
/// path takes one. The only lock involved is the slot's own leaf, held across a
/// move and nothing else.
///
/// # What it is not
///
/// The tick it catches up to is the one the scheduler last published, which it
/// does every time it advances virtual time. Within a quantum a runnable's own
/// progress is not yet in the clock forest — the forest is advanced from the
/// runnable's report, after it returns — so a handle used from inside a
/// runnable's execution sees that runnable's position at the start of the
/// quantum. Bounding the quantum by the next event is what keeps that honest;
/// resolving it properly means letting a runnable report progress *as* it runs,
/// which is a change to [`Runnable`] and not to this type.
#[derive(Debug, Clone)]
pub struct LazyHandle {
    id: LazyId,
    slot: Arc<LazySlot>,
}

impl LazyHandle {
    /// The device's handle in its scheduler.
    #[inline]
    pub const fn id(&self) -> LazyId {
        self.id
    }

    /// The clock domain the device is counted in.
    #[inline]
    pub fn domain(&self) -> DomainId {
        self.slot.domain
    }

    /// Brings the device up to date before an access, and returns the tick it
    /// is now at.
    ///
    /// [`AccessKind::Debug`] advances nothing — a debugger read must not move a
    /// device's clock any more than it may pop a FIFO (`ROADMAP.md` §15,
    /// invariant 5).
    ///
    /// # Errors
    ///
    /// [`SchedError::LazyDeviceBusy`] if catch-up for this device is already
    /// running further up the stack, or [`SchedError::NonMonotonicDevice`] if
    /// the device reports going backwards.
    pub fn sync(&self, kind: AccessKind) -> SchedResult<u64> {
        self.slot.sync(self.id, None, kind)
    }

    /// Advances the device to a specific tick of its own domain.
    ///
    /// What an event dispatcher calls when delivering a device its own
    /// scheduled event; see [`Scheduler::sync_to_tick`].
    ///
    /// # Errors
    ///
    /// As [`LazyHandle::sync`].
    pub fn sync_to_tick(&self, tick: u64) -> SchedResult<u64> {
        self.slot.sync_to_tick(self.id, tick)
    }

    /// The tick the device has simulated up to, advancing nothing.
    ///
    /// # Errors
    ///
    /// [`SchedError::LazyDeviceBusy`] if catch-up is running further up the
    /// stack, in which case the device's position is in flight and there is no
    /// answer to give.
    pub fn current_tick(&self) -> SchedResult<u64> {
        self.slot.current_tick(self.id)
    }

    /// The tick of the device's domain the scheduler last published — the
    /// target the next [`LazyHandle::sync`] will aim for.
    pub fn present_tick(&self) -> u64 {
        self.slot.state.lock().present
    }
}

/// Everything about a [`Scheduler`] that a snapshot has to carry
/// (`ROADMAP.md` §4.5).
///
/// The scheduler *is* architectural state. Re-deriving the queue after a load
/// by asking devices to re-register their events loses sub-tick phase — a timer
/// that was 40 cycles from firing comes back a whole period from firing — and
/// every timer then fails its own round-trip test. So the queue is enumerated
/// and rebuilt verbatim, sequence numbers included.
///
/// # What is here, and why each piece
///
/// * `now` — the front of virtual time. Without it a restored machine starts at
///   instant zero and every absolute deadline in the queue is already past.
/// * `events` — the pending events, in fire order.
/// * `next_seq` — the tie-break counter. Events posted after a restore must
///   lose ties against events restored from before it, which they only do if
///   the counter continues rather than restarts.
/// * `cursor` — where the round-robin resumes. It decides which CPU runs first
///   in the next quantum, so two machines that differ only in this diverge.
///
/// What is deliberately absent is the clock forest, which the layer above saves
/// (its tick counters are the authoritative time state and are shared with
/// devices), and the rate controller, which is anchored to a host clock and is
/// therefore host state rather than guest state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchedulerSnapshot {
    /// The virtual instant the scheduler was at.
    pub now: GlobalTime,
    /// The sequence number the next posted event will carry.
    pub next_seq: u64,
    /// Where the round-robin resumes, as an index into the runnables.
    pub cursor: usize,
    /// Every pending event, in the order it will fire.
    pub events: Vec<Event>,
}

/// `t` as a whole number of nanoseconds, when it is exactly one.
///
/// [`GlobalTime::from_nanos`] rounds down and so does
/// [`GlobalTime::as_nanos`], so the round trip can land a nanosecond low —
/// `from_nanos(1_000_000).as_nanos()` is 999 999. Both candidates are tried,
/// and the answer is the one that converts back to exactly `t`.
fn whole_nanos(t: GlobalTime) -> Option<u64> {
    let floor = t.as_nanos();
    [floor, floor.saturating_add(1)]
        .into_iter()
        .find(|n| *n != 0 && GlobalTime::from_nanos(*n) == t)
}

/// Whether a round may be cut short by the caller's deadline.
///
/// [`Cut::No`] is what a run loop wants: a deadline that falls inside a round
/// declines the round rather than splitting it, which is what makes
/// [`Machine::run_for`](crate::machine::Machine::run_for) additive (§11.6).
/// [`Cut::Yes`] is the debugger's, and is not additive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Cut {
    /// Run the round whole or not at all.
    No,
    /// Run whatever fits before the deadline.
    Yes,
}

/// Where a round's elapsed virtual time comes from.
///
/// The single difference between [`ThreadingMode::Parallel`] and
/// [`ThreadingMode::Accel`], and the whole of §4.2's *"virtual time is slaved
/// to the host clock and the scheduler becomes a deadline service"*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    /// The machine's own grid: a round ends at its natural target and every
    /// tree advances by the ticks its runnable reported. What every mode but
    /// one does, and the only thing a deterministic run can do.
    Emulated,
    /// The host clock: a round ends wherever the wall says it ended, and every
    /// tree is dragged to that instant regardless of what anybody reported.
    ///
    /// For an engine whose progress is **not measurable in guest ticks**. A
    /// vCPU inside `KVM_RUN` executes an unknown number of instructions in a
    /// known amount of host time, so the host time is the only honest input —
    /// and a report of "the whole budget", which is what an accelerated core
    /// has to return, is fiction that would make the board's clocks run at a
    /// rate set by the quantum rather than by anything real.
    Host,
}

/// One runnable's job in flight: which slot it came out of, and the handle that
/// gives the box back with what it consumed.
///
/// Named because the round has to keep the box *and* its index together — the
/// scheduler owns the runnables and the job owns one for the length of the
/// round, so the index is what puts it back.
type Dispatched = (usize, Handle<(Box<dyn Runnable>, Consumed)>);

/// What one round of the round-robin did.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct QuantumReport {
    /// Where virtual time started.
    pub from: GlobalTime,
    /// Where it ended.
    pub to: GlobalTime,
    /// Ticks consumed, per runnable, in registration order.
    pub consumed: Vec<(RunnableId, u64)>,
    /// Events that came due, in `(time, sequence)` order.
    pub fired: Vec<Event>,
}

/// The machine's scheduler: virtual time, the event queue, and execution
/// budgets.
///
/// It owns the [`ClockForest`], because time and the things that consume it
/// cannot be kept consistent from two places.
pub struct Scheduler {
    forest: ClockForest,
    queue: EventQueue,
    now: GlobalTime,
    config: SchedulerConfig,
    runnables: Vec<RunnableSlot>,
    lazy: Vec<Arc<LazySlot>>,
    /// Where the round-robin starts next round, so no runnable is permanently
    /// first.
    cursor: usize,
    /// The quantum as a whole number of nanoseconds, when it is one.
    ///
    /// The grid a round ends on is counted in these — see
    /// [`Scheduler::next_grid_point`] for why. Derived from the config, which
    /// is fixed at construction, so it is computed once rather than per round.
    quantum_nanos: Option<u64>,
    /// [`Scheduler::lazy`] as one shared slice, so arming a cursor does not
    /// allocate. Rebuilt when a device is registered, which happens at realize
    /// and nowhere else.
    lazy_snapshot: Option<Arc<[Arc<LazySlot>]>>,
    /// Per runnable, the lazy slots on its *own* oscillator tree — what
    /// [`ThreadingMode::Parallel`] arms, since two runnables cannot share one
    /// slot's live view. `None` until the first parallel round builds it, and
    /// dropped whenever the registration set changes.
    tree_slots: Option<Vec<Arc<[Arc<LazySlot>]>>>,
    /// The task pool [`ThreadingMode::Parallel`] submits to.
    ///
    /// Built at construction from [`SchedulerConfig::workers`], or handed in by
    /// an embedder with [`Scheduler::set_pool`] — which is the browser's case
    /// and the reason the seam exposes a pool rather than `spawn`
    /// (`ROADMAP.md` §4.7, §11.2).
    pool: Option<Arc<Pool>>,
    /// The stop-the-world protocol every runnable's [`ExitFlag`] hangs off.
    safe: SafePoint,
    rate: RateController,
    host_clock: Option<Box<dyn HostClock>>,
    /// Where virtual time was last pinned to the host clock, for
    /// [`ThreadingMode::Accel`]: the host reading and the virtual instant that
    /// were level at that moment.
    ///
    /// Every later instant is computed from this pair rather than accumulated
    /// round by round, so the two clocks cannot drift apart by more than one
    /// rounding of [`GlobalTime::from_nanos`] however long the machine runs.
    /// `None` until the first accelerated round, and cleared by
    /// [`Scheduler::restore`] — a restored machine is at whatever instant its
    /// snapshot says, and pinning it to a host reading taken before the load
    /// would make the first round jump.
    accel_anchor: Option<(u64, GlobalTime)>,
}

impl fmt::Debug for Scheduler {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Scheduler")
            .field("now", &self.now)
            .field("config", &self.config)
            .field("runnables", &self.runnables)
            .field("lazy", &self.lazy)
            .field("queued", &self.queue.len())
            .field("host_clock", &self.host_clock.is_some())
            .finish()
    }
}

impl Scheduler {
    /// A scheduler driving `forest`.
    pub fn new(forest: ClockForest, config: SchedulerConfig) -> Scheduler {
        let queue = EventQueue::new(config.granule_shift);
        let rate = RateController::new(config.rate);
        let quantum_nanos = whole_nanos(config.quantum);
        // Only the modes that dispatch have anything to submit, and a pool of
        // workers nobody uses is a pool of threads nobody uses.
        let pool = matches!(config.mode, ThreadingMode::Parallel | ThreadingMode::Accel)
            .then(|| Arc::new(Pool::new(config.workers)));
        Scheduler {
            forest,
            queue,
            now: GlobalTime::ZERO,
            config,
            runnables: Vec::new(),
            lazy: Vec::new(),
            cursor: 0,
            quantum_nanos,
            lazy_snapshot: None,
            tree_slots: None,
            pool,
            safe: SafePoint::new(),
            rate,
            host_clock: None,
            accel_anchor: None,
        }
    }

    /// Use `pool` instead of one of this scheduler's own.
    ///
    /// The embedder's entry point. A browser cannot create a Web Worker
    /// synchronously from arbitrary code, so the page builds the pool up front
    /// and hands it in; the same call lets a host share one pool between two
    /// machines rather than doubling its thread count (`ROADMAP.md` §4.7).
    pub fn set_pool(&mut self, pool: Arc<Pool>) {
        self.pool = Some(pool);
        self.republish_contention();
    }

    /// The task pool, if this scheduler has one.
    #[inline]
    #[must_use]
    pub fn pool(&self) -> Option<&Arc<Pool>> {
        self.pool.as_ref()
    }

    /// Whether an empty lazy slot can mean *another thread has it*.
    fn contended(&self) -> bool {
        matches!(
            self.config.mode,
            ThreadingMode::Parallel | ThreadingMode::Accel
        ) && self.pool.as_ref().is_some_and(|p| p.workers() > 0)
    }

    /// Re-derive [`LazySlot::contended`] after anything that can change the
    /// answer: a new pool, a new device.
    fn republish_contention(&self) {
        let contended = self.contended();
        for slot in &self.lazy {
            slot.contended.store(contended, AtomicOrdering::Relaxed);
        }
    }

    /// The stop-the-world protocol (`ROADMAP.md` §4.7).
    ///
    /// Clone it to anything that may need to stop the machine — a host thread,
    /// a device that remaps memory from inside its own write path. See
    /// [`SafePoint`].
    #[inline]
    #[must_use]
    pub fn safe_point(&self) -> SafePoint {
        self.safe.clone()
    }

    /// One runnable's exit flag.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownRunnable`] if the handle is not from this
    /// scheduler.
    pub fn exit_flag(&self, id: RunnableId) -> SchedResult<ExitFlag> {
        self.runnables
            .get(id.index())
            .map(|slot| slot.cursor.exit_flag())
            .ok_or(SchedError::UnknownRunnable(id))
    }

    /// Stop the world and hold it stopped until the guard is dropped.
    ///
    /// Raises every runnable's exit flag, bumps the generation, and waits for
    /// the task pool to go idle — the barrier of `ROADMAP.md` §4.7. What comes
    /// back is a machine nobody is executing: safe to snapshot, to retopologise,
    /// to reset.
    ///
    /// Calling it between rounds is the ordinary case and costs the quiesce.
    /// Calling it from another thread while a parallel round is in flight is
    /// the interesting one: the runnables unwind at their next block boundary
    /// and the round's own join is the rendezvous.
    ///
    /// It does **not** take the scheduler's borrow away from whoever is driving
    /// the run loop; a caller that wants to stop a machine another thread is
    /// running holds a [`SafePoint`] rather than a `&Scheduler`.
    #[must_use = "the world runs again when the guard is dropped"]
    pub fn stop_the_world(&self) -> StopGuard {
        let generation = self.safe.request();
        if let Some(pool) = self.pool.as_ref() {
            pool.quiesce();
        }
        StopGuard {
            safe: self.safe.clone(),
            generation,
        }
    }

    /// Injects the host clock used by rate control.
    ///
    /// Without one, [`RateControl::Unbounded`] still works and every other
    /// policy reports [`SchedError::NoHostClock`] rather than silently running
    /// unthrottled.
    pub fn set_host_clock(&mut self, clock: Box<dyn HostClock>) {
        self.host_clock = Some(clock);
    }

    /// The clock forest.
    #[inline]
    pub fn forest(&self) -> &ClockForest {
        &self.forest
    }

    /// The clock forest, mutably — for building the machine and for guest
    /// writes that re-rate a domain.
    #[inline]
    pub fn forest_mut(&mut self) -> &mut ClockForest {
        &mut self.forest
    }

    /// The event queue.
    #[inline]
    pub fn queue(&self) -> &EventQueue {
        &self.queue
    }

    /// The current virtual instant.
    #[inline]
    pub const fn now(&self) -> GlobalTime {
        self.now
    }

    /// The configuration in force.
    #[inline]
    pub fn config(&self) -> &SchedulerConfig {
        &self.config
    }

    /// Registers something to hand budgets to, running in `domain`.
    pub fn add_runnable(&mut self, domain: DomainId, runnable: Box<dyn Runnable>) -> RunnableId {
        let id = RunnableId(self.runnables.len() as u32);
        self.runnables.push(RunnableSlot {
            domain,
            inner: Some(runnable),
            cursor: TickCursor::with_exit(self.safe.flag()),
        });
        self.tree_slots = None;
        id
    }

    /// Registers a lazily-advanced device clocked by `domain`.
    pub fn add_lazy_device(&mut self, domain: DomainId, device: Box<dyn LazyDevice>) -> LazyId {
        let id = LazyId(self.lazy.len() as u32);
        // An unknown domain is reported by the first sync rather than here,
        // which keeps this call infallible; zero is the honest starting
        // position either way, since nothing has been simulated yet.
        let present = self.forest.ticks(domain).unwrap_or(0);
        let contended = self.contended();
        self.lazy.push(Arc::new(LazySlot {
            id,
            domain,
            state: Mutex::new(LazyState {
                device: Some(device),
                live: None,
                present,
            }),
            contended: AtomicBool::new(contended),
        }));
        self.lazy_snapshot = None;
        self.tree_slots = None;
        id
    }

    /// A shared handle to a registered lazily-advanced device.
    ///
    /// The machine layer clones one of these onto every mapping that routes to
    /// the device, so `MemOps::read` can catch it up through `&self` without a
    /// route back to the scheduler and without taking a scheduler-ranked lock.
    /// See [`LazyHandle`] for why that matters.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownLazyDevice`] if the handle is not from this
    /// scheduler.
    pub fn lazy_handle(&self, id: LazyId) -> SchedResult<LazyHandle> {
        self.lazy
            .get(id.index())
            .map(|slot| LazyHandle {
                id,
                slot: Arc::clone(slot),
            })
            .ok_or(SchedError::UnknownLazyDevice(id))
    }

    /// The live-position cursor of a registered runnable.
    ///
    /// A core publishes its own tick counter into this as it executes, so that
    /// a lazily-advanced device sampled from inside the core's `run` call is
    /// caught up to the tick the access really happened on rather than to the
    /// start of the quantum. See [`TickCursor`].
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownRunnable`] if the handle is not from this
    /// scheduler.
    pub fn runnable_cursor(&self, id: RunnableId) -> SchedResult<TickCursor> {
        self.runnables
            .get(id.index())
            .map(|slot| slot.cursor.clone())
            .ok_or(SchedError::UnknownRunnable(id))
    }

    /// The clock domain a lazily-advanced device is registered in.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownLazyDevice`] if the handle is not from this
    /// scheduler.
    pub fn lazy_domain(&self, id: LazyId) -> SchedResult<DomainId> {
        self.lazy
            .get(id.index())
            .map(|slot| slot.domain)
            .ok_or(SchedError::UnknownLazyDevice(id))
    }

    /// The clock domain a runnable is registered in.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownRunnable`] if the handle is not from this scheduler.
    pub fn runnable_domain(&self, id: RunnableId) -> SchedResult<DomainId> {
        self.runnables
            .get(id.index())
            .map(|s| s.domain)
            .ok_or(SchedError::UnknownRunnable(id))
    }

    // -- scheduling ---------------------------------------------------------

    /// Posts an event at an absolute virtual instant.
    pub fn schedule_at(&mut self, time: GlobalTime, target: EventTarget, token: u64) -> EventId {
        self.queue.schedule(time, target, token)
    }

    /// Posts an event at a given tick of a clock domain.
    ///
    /// The tick is converted to the timeline through that domain's own tree, so
    /// the deadline lands exactly where the device means it to — the NES PPU
    /// asks for "dot 241×341" and gets that dot, not a rounded neighbourhood of
    /// it.
    ///
    /// # Errors
    ///
    /// [`SchedError::Clock`] if the domain is unknown or the conversion
    /// overflows.
    pub fn schedule_at_tick(
        &mut self,
        domain: DomainId,
        tick: u64,
        target: EventTarget,
        token: u64,
    ) -> SchedResult<EventId> {
        let time = self.forest.global_time_of_tick(domain, tick)?;
        Ok(self.queue.schedule(time, target, token))
    }

    /// Posts an event `ticks` ticks of `domain` from that domain's current
    /// position.
    ///
    /// # Errors
    ///
    /// [`SchedError::Clock`] if the domain is unknown or the conversion
    /// overflows.
    pub fn schedule_after_ticks(
        &mut self,
        domain: DomainId,
        ticks: u64,
        target: EventTarget,
        token: u64,
    ) -> SchedResult<EventId> {
        let at = self.forest.ticks(domain)?.saturating_add(ticks);
        self.schedule_at_tick(domain, at, target, token)
    }

    /// Cancels a posted event.
    pub fn cancel(&mut self, id: EventId) {
        self.queue.cancel(id);
    }

    // -- catch-up -----------------------------------------------------------

    /// Brings a lazily-advanced device up to date before an access, and returns
    /// the tick it is now at.
    ///
    /// The target is computed **inside the device's own clock tree** — the
    /// domain's tick count at the tree's current position — so a PPU catching up
    /// to its CPU never goes near absolute time (`ROADMAP.md` §15, invariant 2).
    /// It is then clamped to the device's own next event, so catch-up never
    /// simulates past a point where the device's behaviour would change.
    ///
    /// [`AccessKind::Debug`] advances nothing.
    ///
    /// Takes `&self`, not `&mut self`: the caller is `MemOps::read`, which has
    /// a shared borrow and is several frames below whoever owns the scheduler.
    /// A device reached from inside a *running* quantum has no route back here
    /// at all and uses a [`LazyHandle`] instead; this method is the same
    /// operation for a caller that does hold the scheduler — a monitor, a
    /// dispatcher between quanta, a test — and it reads the forest directly, so
    /// it is exact even if virtual time moved since the last publish.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownLazyDevice`], [`SchedError::Clock`],
    /// [`SchedError::LazyDeviceBusy`], or [`SchedError::NonMonotonicDevice`] if
    /// the device reports going backwards.
    pub fn sync_for_access(&self, id: LazyId, kind: AccessKind) -> SchedResult<u64> {
        let slot = self
            .lazy
            .get(id.index())
            .ok_or(SchedError::UnknownLazyDevice(id))?;
        let present = self.forest.ticks(slot.domain)?;
        slot.sync(id, Some(present), kind)
    }

    /// Advances a lazily-advanced device to a specific tick of its own domain.
    ///
    /// This is what an event dispatcher calls when delivering a device its own
    /// scheduled event: the device asked to be at that tick, and this puts it
    /// there. It may be up to one tick of the tree's driving domain ahead of the
    /// domain's own counter, for the reason set out in the module documentation
    /// — the CPU stopped at the cycle boundary before the event's instant.
    ///
    /// Going backwards is refused rather than obeyed.
    ///
    /// # Errors
    ///
    /// [`SchedError::UnknownLazyDevice`], [`SchedError::LazyDeviceBusy`], or
    /// [`SchedError::NonMonotonicDevice`] if the device reports going
    /// backwards.
    pub fn sync_to_tick(&self, id: LazyId, tick: u64) -> SchedResult<u64> {
        self.lazy
            .get(id.index())
            .ok_or(SchedError::UnknownLazyDevice(id))?
            .sync_to_tick(id, tick)
    }

    /// Catches every lazily-advanced device up to the present.
    ///
    /// The other half of sync-on-access, and the half without which a mapped
    /// PPU is worse than no PPU: a device nobody reads still has to reach the
    /// dot it is standing on, or it never raises the NMI that the game is
    /// waiting for. A run loop calls this at every quantum boundary.
    ///
    /// Each device is advanced repeatedly until it reaches the present or stops
    /// making progress, because a single [`LazyHandle::sync`] stops at the
    /// device's own next event and a quantum may contain several of them.
    ///
    /// # Errors
    ///
    /// [`SchedError::Clock`] for a domain the forest does not know,
    /// [`SchedError::LazyDeviceBusy`] if catch-up is already running further up
    /// the stack, or [`SchedError::NonMonotonicDevice`].
    pub fn sync_lazy_devices(&self) -> SchedResult<()> {
        for (index, slot) in self.lazy.iter().enumerate() {
            let id = LazyId(index as u32);
            let present = self.forest.ticks(slot.domain)?;
            let mut last = None;
            loop {
                let at = slot.sync(id, Some(present), AccessKind::Guest)?;
                if at >= present || Some(at) == last {
                    break;
                }
                last = Some(at);
            }
        }
        Ok(())
    }

    /// The earliest instant at which some lazily-advanced device's own next
    /// event falls, if any device has one.
    ///
    /// What a run loop bounds its next quantum by. Without it a CPU handed a
    /// 10 000-cycle budget runs thousands of cycles past the dot the PPU raised
    /// vblank on, and the NMI lands that late — the scheduled half of §4.2,
    /// where [`LazyHandle::sync`] is the sampled half.
    ///
    /// A device whose next event has already gone by is not reported: the
    /// caller cannot un-run the cycles that passed it, and clamping a quantum
    /// to an instant that is not in the future would stall the machine instead.
    pub fn lazy_deadline(&self) -> Option<GlobalTime> {
        let mut best: Option<GlobalTime> = None;
        for slot in &self.lazy {
            let Some(tick) = slot.next_event_tick() else {
                continue;
            };
            let Ok(at) = self.forest.global_time_of_tick(slot.domain, tick) else {
                continue;
            };
            if at <= self.now {
                continue;
            }
            if best.is_none_or(|b| at < b) {
                best = Some(at);
            }
        }
        best
    }

    /// Publishes every lazy device's domain position, for the handles.
    ///
    /// Called after each advance of virtual time. Cheap — there are as many
    /// slots as there are lazily-advanced devices, which is a handful — and
    /// recomputed rather than tracked incrementally, because a guest write that
    /// re-rates or gates a domain moves its tick counter without anything
    /// having advanced.
    fn publish_lazy_positions(&self) {
        for slot in &self.lazy {
            if let Ok(present) = self.forest.ticks(slot.domain) {
                slot.publish(present);
            }
        }
    }

    // -- running ------------------------------------------------------------

    /// Runs one quantum.
    ///
    /// # Errors
    ///
    /// [`SchedError::BudgetExceeded`] if a runnable overran,
    /// [`SchedError::NoHostClock`] under [`ThreadingMode::Accel`] with no
    /// clock injected, or [`SchedError::Clock`].
    pub fn run_quantum(&mut self) -> SchedResult<QuantumReport> {
        self.run_quantum_until(GlobalTime::MAX)
    }

    /// Runs one quantum, but never past `limit`.
    ///
    /// A round ends at its *natural target*: the next point of the quantum
    /// grid, the next queued event, or the next event a lazily-advanced device
    /// has of its own — an instant that depends on virtual time and the
    /// machine's own state and on nothing else.
    /// If `limit` falls *before* that instant the round is not started: virtual
    /// time moves to `limit` with nothing executed, and the round runs whole
    /// when the caller asks for more time.
    ///
    /// That is what makes running for a span and running for the same span in
    /// pieces reach the same state (§11.6). A deadline is an arbitrary instant
    /// chosen by whoever is driving the machine — a frame in a browser, a span
    /// on a command line — and letting it cut a round short would hand every
    /// runnable a budget the unsliced run never handed out, permanently.
    ///
    /// The price is that a caller whose deadlines are finer than the machine's
    /// own boundaries gets its work in bursts rather than a little at a time.
    /// Nothing is lost — budgets come from each tree's absolute position, so a
    /// deferred tick is handed out by the round that owns it — but a caller
    /// that needs execution to track a fine deadline should shorten
    /// [`SchedulerConfig::quantum`], which is what it is for.
    ///
    /// # Errors
    ///
    /// As [`Scheduler::run_quantum`].
    pub fn run_quantum_until(&mut self, limit: GlobalTime) -> SchedResult<QuantumReport> {
        self.run_quantum_bounded(limit, Cut::No)
    }

    /// Runs one quantum, cutting it short at `limit` rather than declining it.
    ///
    /// **Not additive, and that is the point.** A debugger stepping one CPU
    /// cycle at a time cannot wait for a round to end — that is thousands of
    /// cycles, and every breakpoint between here and there would be stepped
    /// over. So this hands out the fragment of a round that fits before
    /// `limit`, which is exactly the scheduling boundary
    /// [`Scheduler::run_quantum_until`] refuses to create.
    ///
    /// Use it for stepping and for nothing else. A run loop that reaches for it
    /// gives up §11.6: two sessions that stop at different instants stop being
    /// comparable, permanently.
    ///
    /// # Errors
    ///
    /// As [`Scheduler::run_quantum`].
    pub fn step_quantum_until(&mut self, limit: GlobalTime) -> SchedResult<QuantumReport> {
        self.run_quantum_bounded(limit, Cut::Yes)
    }

    fn run_quantum_bounded(&mut self, limit: GlobalTime, cut: Cut) -> SchedResult<QuantumReport> {
        match self.config.mode {
            ThreadingMode::Deterministic => self.run_quantum_deterministic(limit, cut),
            ThreadingMode::Parallel => self.run_quantum_dispatched(limit, cut, Source::Emulated),
            ThreadingMode::Accel => self.run_quantum_dispatched(limit, cut, Source::Host),
        }
    }

    /// Runs quanta until virtual time reaches `deadline`.
    ///
    /// # Errors
    ///
    /// As [`Scheduler::run_quantum`].
    pub fn run_until(&mut self, deadline: GlobalTime) -> SchedResult<()> {
        while self.now < deadline {
            let before = self.now;
            let report = self.run_quantum_until(deadline)?;
            if self.now <= before && report.fired.is_empty() {
                // Nothing moved and nothing fired: jump to the deadline rather
                // than spin on a machine with no runnables and no events. A
                // quantum that stood still *because* an event was due at this
                // very instant is progress, and must not end the loop.
                self.advance_idle_to(deadline)?;
                return Ok(());
            }
        }
        Ok(())
    }

    /// The next instant a round would naturally end at, ignoring the caller.
    ///
    /// Three candidates, and every one of them is an *absolute* instant rather
    /// than an offset from wherever the last round happened to stop:
    ///
    /// * the next point of the quantum grid — a multiple of
    ///   [`SchedulerConfig::quantum`], not `now + quantum`;
    /// * the next queued event, because a CPU that executes through its own NMI
    ///   has already got the answer wrong;
    /// * the next instant a lazily-advanced device has an event of its own, so
    ///   the PPU reaches the dot it raises vblank on even while the CPU is busy
    ///   elsewhere (§4.2, the scheduled half of sync-on-access).
    ///
    /// Being a pure function of virtual time and machine state — never of how
    /// the caller sliced the run — is the whole point. See
    /// [`Scheduler::run_quantum_until`] for what it buys.
    fn natural_target(&mut self) -> GlobalTime {
        let mut target = self.next_grid_point();
        if let Some(deadline) = self.queue.next_deadline()
            && deadline < target
        {
            target = deadline;
        }
        if let Some(at) = self.lazy_deadline()
            && at < target
        {
            target = at;
        }
        // An event already in the past pulls the target below `now`; running
        // backwards is worse than firing it late, so the round stands still and
        // the tail of `run_quantum_deterministic` pops it.
        target.max(self.now)
    }

    /// The first multiple of the quantum strictly after `now`.
    ///
    /// A grid anchored at the origin rather than at `now` is what makes an
    /// interrupted run resume on the boundaries it would have used anyway: two
    /// instants in the same cell have the same next boundary, so a caller's
    /// deadline landing mid-cell cannot shift every later one.
    ///
    /// Counted in **nanoseconds** rather than in raw 2⁻⁶⁴-second units,
    /// whenever the quantum is a whole number of them, because that is the unit
    /// callers name deadlines in. A nanosecond is not a dyadic fraction of a
    /// second, so `k` raw quanta drift below `k` quanta-worth of nanoseconds by
    /// up to `k` units — enough that a run of one virtual second would stop a
    /// hair before the second and leave the cycle beginning there for the next
    /// call. Counting the grid the way the caller counts the deadline puts the
    /// two on the same points: [`GlobalTime::from_nanos`] rounds once, the same
    /// way, on both sides.
    ///
    /// A zero quantum returns `now`, which stalls the machine — deliberately,
    /// because [`Machine::run_until`](crate::machine::Machine::run_until)
    /// reports that as the configuration error it is rather than spinning.
    fn next_grid_point(&self) -> GlobalTime {
        if let Some(nanos) = self.quantum_nanos {
            let here = self.now.as_nanos() / nanos;
            // At most twice: `from_nanos` rounds down, so the boundary that
            // `now`'s own nanosecond count names can land at or before `now`
            // itself. The one after it cannot, a quantum being a whole
            // nanosecond or more.
            for cell in [here.saturating_add(1), here.saturating_add(2)] {
                let at = GlobalTime::from_nanos(cell.saturating_mul(nanos));
                if at > self.now {
                    return at;
                }
            }
        }
        let quantum = self.config.quantum.raw();
        if quantum == 0 {
            return self.now;
        }
        let cell = self.now.raw() / quantum;
        GlobalTime::from_raw(cell.saturating_add(1).saturating_mul(quantum))
    }

    fn run_quantum_deterministic(
        &mut self,
        limit: GlobalTime,
        cut: Cut,
    ) -> SchedResult<QuantumReport> {
        let from = self.now;
        let natural = self.natural_target();
        let target = match cut {
            // A debugger's fragment. See [`Scheduler::step_quantum_until`] for
            // why this exists and why nothing else may use it.
            Cut::Yes => natural.min(limit),
            Cut::No => natural,
        };
        if target > limit {
            return self.decline_round(from, limit);
        }

        let mut consumed = Vec::with_capacity(self.runnables.len());
        let count = self.runnables.len();
        for i in 0..count {
            let index = (self.cursor + i) % count;
            let id = RunnableId(index as u32);
            let domain = self.runnables[index].domain;
            let allowed = self.ticks_until(domain, target)?;
            let allowed = allowed.min(self.config.max_ticks_per_quantum);
            if allowed == 0 {
                consumed.push((id, 0));
                continue;
            }
            let budget = Budget {
                until: target,
                ticks: allowed,
            };
            let Some(mut runnable) = self.runnables[index].inner.take() else {
                consumed.push((id, 0));
                continue;
            };
            // Everything sampled while this runnable executes must see where it
            // has got to, not where the quantum began (see [`TickCursor`]).
            let cursor = self.runnables[index].cursor.clone();
            self.arm_live_cursors(domain, &cursor);
            let used = runnable.run(budget);
            self.disarm_live_cursors(&cursor);
            self.runnables[index].inner = Some(runnable);
            if used.ticks > allowed {
                return Err(SchedError::BudgetExceeded {
                    runnable: id,
                    budget: allowed,
                    consumed: used.ticks,
                });
            }
            if used.ticks > 0 {
                self.forest.advance_domain(domain, used.ticks)?;
            }
            consumed.push((id, used.ticks));
        }
        if count > 0 {
            self.cursor = (self.cursor + 1) % count;
        }

        self.close_round(from, target, consumed)
    }

    /// A round the caller's deadline falls inside: virtual time moves to the
    /// deadline, nothing executes, and the round runs whole when the caller
    /// asks for more time.
    ///
    /// Running the fragment instead — which is what this did, and what made
    /// `run_for` non-additive — hands every runnable a budget the unsliced run
    /// never handed out, and then hands out the remainder in a second pass. Two
    /// runnables that observe each other diverge there and never converge
    /// again. `riscv-virt` is the measured case: its 16550 pumps its port once
    /// per call, so an extra pass is an extra character. Rotating the
    /// round-robin only on a completed round fixes the ordering but not that.
    ///
    /// Nothing is lost by waiting. Budgets come from each tree's absolute
    /// position (see [`Scheduler::ticks_until`]), so the ticks this defers are
    /// handed out by the round that ends up owning them. No event can fall in
    /// the skipped interval either: one at or before `limit` would have been
    /// the natural target.
    fn decline_round(&mut self, from: GlobalTime, limit: GlobalTime) -> SchedResult<QuantumReport> {
        self.advance_idle_to(limit)?;
        let mut fired = Vec::new();
        while let Some(e) = self.queue.pop_due(self.now) {
            fired.push(e);
        }
        Ok(QuantumReport {
            from,
            to: self.now,
            consumed: Vec::new(),
            fired,
        })
    }

    /// The tail every round shares: passive crystals, virtual time, the
    /// positions lazily-advanced devices are caught up to, and the events that
    /// became due.
    fn close_round(
        &mut self,
        from: GlobalTime,
        target: GlobalTime,
        consumed: Vec<(RunnableId, u64)>,
    ) -> SchedResult<QuantumReport> {
        // Trees nothing drives — a bare RTC crystal — still have to reach the
        // present, and the only way there is through absolute time. This is a
        // legitimate cross-tree conversion: there is no intra-tree alternative.
        self.advance_undriven_trees(target)?;

        self.now = target;
        // Before the events are popped, so a handler reached through a handle
        // sees the position the event fired at rather than the previous one.
        self.publish_lazy_positions();
        let mut fired = Vec::new();
        while let Some(e) = self.queue.pop_due(self.now) {
            fired.push(e);
        }
        Ok(QuantumReport {
            from,
            to: self.now,
            consumed,
            fired,
        })
    }

    /// One round with a thread per runnable and a rendezvous barrier at the end
    /// (`ROADMAP.md` §4.2's `parallel`) — **and, with `source` set to
    /// [`Source::Host`], §4.2's `accel`.**
    ///
    /// The two modes are one function because they differ in exactly one
    /// thing: where the round's elapsed virtual time comes from. Everything
    /// else — the reservation of a shared tree's units, the submission, the
    /// barrier, the overrun check, the cursor arming — is the same code, and
    /// writing it twice is how the two would drift apart. [`Source`] is the
    /// difference, stated once.
    ///
    /// # What is the same as the deterministic round, and why
    ///
    /// Everything except *where the work runs*. The round's target is the same
    /// natural target — the quantum grid, the next queued event, the next event
    /// a lazily-advanced device has of its own — and every budget still comes
    /// from its tree's absolute position through
    /// [`Scheduler::ticks_until`]. So a parallel machine executes the same
    /// number of guest ticks in the same number of rounds as a deterministic
    /// one; what differs is only the order in which two runnables' memory
    /// effects interleave, which is exactly the thing §4.2 says this mode gives
    /// up and nothing more.
    ///
    /// A deadline inside a round still declines the round, so
    /// [`Machine::run_for`](crate::machine::Machine::run_for) keeps its shape
    /// here too — not its state hash, which no mode can promise once two CPUs
    /// race, but the property that the *set of rounds* does not depend on how
    /// the caller sliced the run.
    ///
    /// # The barrier
    ///
    /// `submit` for every runnable **but one**, that one on this thread, then
    /// `join` for each submitted job. The joins are the barrier: nothing is put
    /// back in its slot, no domain is advanced and no event is popped until
    /// every job has finished, and `join` is a happens-before edge, so the
    /// bookkeeping below reads everything the jobs wrote. Submitting in
    /// registration order and joining in registration order also means the
    /// *bookkeeping* is deterministic given the consumed counts — the
    /// non-determinism is confined to what those counts are and to what the
    /// guests saw of each other.
    ///
    /// Keeping one runnable here is not a micro-optimisation, it is what
    /// decides whether the mode is worth using on a two-CPU machine. A round
    /// costs roughly a couple of microseconds per *dispatched* job — a queue
    /// push, a wake, and a wait — against which a round's actual work must be
    /// measured. Two CPUs at the default
    /// [`SchedulerConfig::max_ticks_per_quantum`] of 10 000 is well inside the
    /// region where two dispatches cost more than the second core saves;
    /// one dispatch and a driver thread that works instead of blocking roughly
    /// halves that overhead. It is still a real cost, and
    /// [`ThreadingMode::Parallel`]'s own documentation says where the crossover
    /// lies rather than claiming there is not one.
    ///
    /// # The round-robin cursor
    ///
    /// Not rotated: there is no "first" runnable to rotate away from. It is
    /// still carried through a snapshot, so a machine saved in parallel mode
    /// and restored in deterministic mode resumes its round-robin where the
    /// last deterministic round left it rather than at zero.
    fn run_quantum_dispatched(
        &mut self,
        limit: GlobalTime,
        cut: Cut,
        source: Source,
    ) -> SchedResult<QuantumReport> {
        let from = self.now;
        // Not computed under `Source::Host`: it is unused there, and it walks
        // every lazy slot on a path that now runs once per guest exit.
        let natural = match source {
            Source::Emulated => self.natural_target(),
            Source::Host => from,
        };
        let target = match (source, cut) {
            // Under acceleration the target is **only** an allowance: it sizes
            // the budgets and nothing else, because where the round ends is
            // read off the host clock afterwards. So it is a whole quantum
            // ahead of `now`, and neither the natural target nor the caller's
            // deadline may pull it in.
            //
            // Both would pull it all the way to `now`, and that is fatal
            // rather than conservative. An event already due clamps
            // [`Scheduler::natural_target`] to the present; a budget of zero
            // ticks takes the runnable out of the round entirely; and a round
            // that runs nothing still advances virtual time to the wall, so
            // the device whose event was late reschedules into the past again
            // and the machine never enters the guest a second time. The same
            // goes for a deadline falling inside a round: declining it is what
            // makes [`Machine::run_for`](crate::machine::Machine::run_for)
            // additive (§11.6), and additivity is not on offer once rounds are
            // cut by the wall — while *not entering the guest* is a machine
            // that has stopped. The caller's deadline is honoured by the run
            // loop above, which compares it against a `now` that is the wall.
            // A zero quantum leaves this round with nothing to hand out. The
            // other modes report that through their caller — a round that
            // moves no virtual time is what
            // [`Machine::run_until`](crate::machine::Machine::run_until) turns
            // into "the scheduler quantum is zero" — and this one must reach
            // the same place rather than spin the guest on an empty budget for
            // ever, so it declines to move the clock too.
            (Source::Host, _) if self.config.quantum.raw() == 0 => {
                return self.close_round(from, from, Vec::new());
            }
            (Source::Host, _) => from.saturating_add(self.config.quantum),
            (Source::Emulated, Cut::Yes) => natural.min(limit),
            (Source::Emulated, Cut::No) => natural,
        };
        if source == Source::Emulated && target > limit {
            return self.decline_round(from, limit);
        }

        let count = self.runnables.len();
        let mut allowed = Vec::with_capacity(count);
        // Units of each tree already promised to an earlier runnable. See
        // [`Scheduler::ticks_until_after`] for why a tree has to be shared out
        // rather than handed to everyone whole.
        let mut reserved: Vec<(OscillatorId, u64)> = Vec::new();
        for index in 0..count {
            let domain = self.runnables[index].domain;
            let osc = self.forest.root_of(domain).ok();
            let taken = osc
                .and_then(|osc| reserved.iter().find(|(o, _)| *o == osc))
                .map_or(0, |(_, units)| *units);
            let ticks = self
                .ticks_until_after(domain, target, taken)?
                .min(self.config.max_ticks_per_quantum);
            allowed.push(ticks);
            if let (Some(osc), Ok(per_tick)) =
                (osc, self.forest.domain(domain).map(|d| d.units_per_tick()))
            {
                let units = ticks.saturating_mul(per_tick);
                match reserved.iter_mut().find(|(o, _)| *o == osc) {
                    Some((_, sum)) => *sum = sum.saturating_add(units),
                    None => reserved.push((osc, units)),
                }
            }
        }

        // The host reading that opens the round. Taken before anything runs so
        // that the span it measures is exactly the span the guests were given.
        let opened = match source {
            Source::Host => {
                let at = self.host_nanos()?;
                // Anchored *here* rather than at the close, or the very first
                // round would measure its own length as zero and virtual time
                // would start one round behind the wall for ever.
                if self.accel_anchor.is_none() {
                    self.accel_anchor = Some((at, self.now));
                }
                Some(at)
            }
            Source::Emulated => None,
        };

        self.arm_parallel_cursors();

        let mut used_by: Vec<Option<Consumed>> = alloc::vec![None; count];

        // Which runnables actually have work. The last of them stays on this
        // thread: the driver would otherwise submit every job and then block,
        // which wastes a core and pays a queue round trip for a runnable that
        // is already here. One less dispatch per round is the difference
        // between a two-CPU machine being faster in this mode and being
        // slower — see the module docs on what the barrier costs.
        let mut work: Vec<usize> = allowed
            .iter()
            .zip(&self.runnables)
            .enumerate()
            .filter(|(_, (ticks, slot))| **ticks > 0 && slot.inner.is_some())
            .map(|(index, _)| index)
            .collect();
        let here = work.pop();

        // Cloned out of `self` so the submission loop can borrow the runnables
        // mutably while it holds the pool.
        let pool = self.pool.clone();
        let mut handles: Vec<Dispatched> = Vec::with_capacity(work.len());
        for index in work {
            let budget = Budget {
                until: target,
                ticks: allowed[index],
            };
            let mut runnable = self.runnables[index]
                .inner
                .take()
                .expect("checked just above");
            match pool.as_ref() {
                // The job owns the runnable for the length of the round, which
                // is what makes `&mut self` on `Runnable::run` reachable from a
                // worker thread at all: the box moves there and comes back.
                Some(pool) => handles.push((
                    index,
                    pool.submit(move || {
                        let used = runnable.run(budget);
                        (runnable, used)
                    }),
                )),
                // No pool at all: run it here, in registration order. A
                // parallel machine on a backend with no threads is still a
                // machine that runs (§11.3).
                None => {
                    let used = runnable.run(budget);
                    self.runnables[index].inner = Some(runnable);
                    used_by[index] = Some(used);
                }
            }
        }

        // This thread's share, running alongside everything submitted above.
        if let Some(index) = here {
            let budget = Budget {
                until: target,
                ticks: allowed[index],
            };
            let mut runnable = self.runnables[index]
                .inner
                .take()
                .expect("checked just above");
            let used = runnable.run(budget);
            self.runnables[index].inner = Some(runnable);
            used_by[index] = Some(used);
        }

        // The rendezvous.
        for (index, handle) in handles {
            let (runnable, used) = handle.join();
            self.runnables[index].inner = Some(runnable);
            used_by[index] = Some(used);
        }
        self.disarm_parallel_cursors();

        let mut consumed = Vec::with_capacity(count);
        for index in 0..count {
            let used = used_by[index].unwrap_or_default();
            match source {
                Source::Emulated => self.record_consumption(index, allowed[index], used)?,
                // The overrun check still applies — a runnable that claims more
                // than it was given is a bug in either mode — but the count is
                // not what moves the clock here. See [`Source::Host`].
                Source::Host => self.check_consumption(index, allowed[index], used)?,
            }
            consumed.push((RunnableId(index as u32), used.ticks));
        }

        match opened {
            Some(opened) => self.close_round_slaved(from, opened, consumed),
            None => self.close_round(from, target, consumed),
        }
    }

    /// Check a runnable's report and advance its domain by what it consumed.
    ///
    /// Split out because the parallel round has to do it after the barrier
    /// rather than immediately after the call, and doing it twice by hand is
    /// how the two rounds would drift apart.
    fn record_consumption(
        &mut self,
        index: usize,
        allowed: u64,
        used: Consumed,
    ) -> SchedResult<()> {
        self.check_consumption(index, allowed, used)?;
        if used.ticks > 0 {
            self.forest
                .advance_domain(self.runnables[index].domain, used.ticks)?;
        }
        Ok(())
    }

    /// The half of [`Scheduler::record_consumption`] that is a *check* rather
    /// than a clock advance.
    ///
    /// [`Source::Host`] keeps this and drops the other half: a runnable that
    /// claims more than it was given is a bug whichever clock is in charge,
    /// but under acceleration the count it returns is not the thing that moves
    /// time.
    fn check_consumption(&self, index: usize, allowed: u64, used: Consumed) -> SchedResult<()> {
        if used.ticks > allowed {
            return Err(SchedError::BudgetExceeded {
                runnable: RunnableId(index as u32),
                budget: allowed,
                consumed: used.ticks,
            });
        }
        Ok(())
    }

    /// The injected host clock's reading.
    ///
    /// # Errors
    ///
    /// [`SchedError::NoHostClock`] if none was injected. Under
    /// [`ThreadingMode::Accel`] that is fatal rather than a degraded mode:
    /// there is no other source of elapsed time, and a machine that guessed
    /// would be the very thing this mode exists to stop.
    fn host_nanos(&self) -> SchedResult<u64> {
        Ok(self
            .host_clock
            .as_ref()
            .ok_or(SchedError::NoHostClock)?
            .monotonic_nanos())
    }

    /// Where virtual time stands, given a reading of the host clock.
    ///
    /// Computed from a single anchor rather than accumulated per round, so
    /// nothing drifts: the answer is always
    /// `anchor_virtual + (host − anchor_host)`, scaled by the rate policy.
    /// [`RateControl::FixedRatio`] is honoured — a machine asked to run at
    /// half speed does, because its clocks are told half as much time passed —
    /// and every other policy is one-to-one, which is what makes a guest's own
    /// calibration of the host's time-stamp counter against a board timer come
    /// out right.
    fn slaved_now(&mut self, host_nanos: u64) -> GlobalTime {
        let (origin_host, origin_at) = match self.accel_anchor {
            Some(anchor) => anchor,
            None => {
                let anchor = (host_nanos, self.now);
                self.accel_anchor = Some(anchor);
                anchor
            }
        };
        let host_ns = host_nanos.saturating_sub(origin_host);
        let virtual_ns = match self.rate.control() {
            RateControl::FixedRatio { num, den } if den != 0 => {
                u64::try_from((host_ns as u128) * (num as u128) / (den as u128)).unwrap_or(u64::MAX)
            }
            _ => host_ns,
        };
        origin_at.saturating_add(GlobalTime::from_nanos(virtual_ns))
    }

    /// The tail of an accelerated round: virtual time is whatever the host
    /// clock says, and every tree is dragged there.
    ///
    /// Three things differ from [`Scheduler::close_round`], and each is the
    /// mode's definition rather than an approximation:
    ///
    /// * **The target comes from the wall**, read after the guests ran rather
    ///   than computed before them. Nothing else can measure what a vCPU did.
    /// * **Every active tree is advanced absolutely**, not only the undriven
    ///   ones. A tree whose runnable is a hypervisor client has no honest tick
    ///   count to advance by, so its position comes from the same place as
    ///   everyone else's. This is a cross-tree conversion, and it is the one
    ///   §4.2 sanctions: there is no intra-tree alternative when the driving
    ///   engine cannot count.
    /// * **The round is never empty.** A host clock whose resolution swallowed
    ///   the round would leave `now` where it was, and a run loop reads that as
    ///   *virtual time did not advance* and stops. One nanosecond is below
    ///   anything a guest can observe and is bounded by the anchor, which the
    ///   next round measures from unchanged.
    fn close_round_slaved(
        &mut self,
        from: GlobalTime,
        opened: u64,
        consumed: Vec<(RunnableId, u64)>,
    ) -> SchedResult<QuantumReport> {
        let closed = self.host_nanos()?;
        let target = self
            .slaved_now(closed.max(opened))
            .max(from.saturating_add(GlobalTime::from_nanos(1)));
        let oscillators: Vec<OscillatorId> = self.forest.oscillators().collect();
        for osc in oscillators {
            if !self.forest.is_active(osc)? {
                continue;
            }
            // A no-op for a tree already past `target`, which is how a
            // runnable that *can* count keeps whatever it counted.
            self.forest.advance_to_global(osc, target)?;
        }
        self.now = target;
        self.publish_lazy_positions();
        let mut fired = Vec::new();
        while let Some(e) = self.queue.pop_due(self.now) {
            fired.push(e);
        }
        Ok(QuantumReport {
            from,
            to: self.now,
            consumed,
            fired,
        })
    }

    /// Arm each runnable's cursor over the lazily-advanced devices on *its own*
    /// oscillator tree, for a round in which every runnable executes at once.
    ///
    /// Two differences from [`Scheduler::arm_live_cursors`], both forced:
    ///
    /// * A slot holds **one** live view, so a tree with two runnables on it has
    ///   no honest answer to "where has the executing runnable got to". Such a
    ///   tree is left unarmed and its devices are caught up to the position the
    ///   scheduler last published — which is what every device had before
    ///   [`TickCursor`] existed, and is bounded by the round rather than wrong.
    /// * A cursor watches only the slots on its own tree. In the deterministic
    ///   round one cursor watches every slot, which costs a cross-tree
    ///   catch-up to a published position and nothing else; here it would put
    ///   two threads into one slot for no benefit at all, since a slot on
    ///   another tree has no live view to convert against anyway.
    fn arm_parallel_cursors(&mut self) {
        self.build_tree_slots();
        let Some(trees) = self.tree_slots.clone() else {
            return;
        };
        // A tree driven by more than one runnable has no single live position.
        let mut runnables_on: Vec<(OscillatorId, usize)> = Vec::new();
        for slot in &self.runnables {
            if let Ok(osc) = self.forest.root_of(slot.domain) {
                match runnables_on.iter_mut().find(|(o, _)| *o == osc) {
                    Some((_, n)) => *n += 1,
                    None => runnables_on.push((osc, 1)),
                }
            }
        }
        for (index, slots) in trees.iter().enumerate() {
            let domain = self.runnables[index].domain;
            let cursor = self.runnables[index].cursor.clone();
            let (Ok(osc), Ok(mul), Ok(base_cursor)) = (
                self.forest.root_of(domain),
                self.forest.domain(domain).map(|d| d.units_per_tick()),
                self.forest.ticks(domain),
            ) else {
                cursor.watch(None);
                continue;
            };
            if !runnables_on.iter().any(|(o, n)| *o == osc && *n == 1) {
                cursor.watch(None);
                continue;
            }
            for slot in slots.iter() {
                let (Ok(div), Ok(base_tick)) = (
                    self.forest.domain(slot.domain).map(|d| d.units_per_tick()),
                    self.forest.ticks(slot.domain),
                ) else {
                    continue;
                };
                if div == 0 {
                    continue;
                }
                slot.arm(Live {
                    cursor: cursor.clone(),
                    base_cursor,
                    base_tick,
                    mul,
                    div,
                });
            }
            cursor.watch(Some(Arc::clone(slots)));
        }
    }

    /// Drop every live view and every watch a parallel round installed.
    fn disarm_parallel_cursors(&self) {
        for slot in &self.runnables {
            slot.cursor.watch(None);
        }
        for slot in &self.lazy {
            slot.disarm();
        }
    }

    /// Per runnable, the lazy slots that share its oscillator tree.
    ///
    /// Derived state keyed on the registration set, which changes at realize
    /// and nowhere else.
    fn build_tree_slots(&mut self) {
        if self.tree_slots.is_some() {
            return;
        }
        let mut trees = Vec::with_capacity(self.runnables.len());
        for slot in &self.runnables {
            let osc = self.forest.root_of(slot.domain);
            let mine: Arc<[Arc<LazySlot>]> = self
                .lazy
                .iter()
                .filter(|lazy| osc.is_ok() && self.forest.root_of(lazy.domain) == osc)
                .cloned()
                .collect();
            trees.push(mine);
        }
        self.tree_slots = Some(trees);
    }

    /// Point every lazily-advanced device on `domain`'s own oscillator tree at
    /// the cursor of the runnable that is about to execute.
    ///
    /// Only devices on the same tree: a ratio between two trees is not exact,
    /// and routing an intra-quantum position through absolute time would throw
    /// away the exactness the oscillator forest exists to preserve
    /// (`ROADMAP.md` 4.2). A device on another tree keeps the published
    /// position, which is what it had before this existed.
    fn arm_live_cursors(&mut self, domain: DomainId, cursor: &TickCursor) {
        if self.lazy_snapshot.is_none() {
            self.lazy_snapshot = Some(self.lazy.iter().cloned().collect());
        }
        let (Ok(osc), Ok(mul)) = (
            self.forest.root_of(domain),
            self.forest.domain(domain).map(|d| d.units_per_tick()),
        ) else {
            return;
        };
        // The forest's position for the runnable's own domain, **not** what the
        // cursor currently reads. A core that overran its last budget has
        // already executed cycles the forest has not been told about and
        // carries them as debt; its cursor is ahead by exactly that much. Using
        // the cursor here would cancel the debt out and leave every lazy device
        // three dots per owed cycle behind — and, because the debt varies from
        // quantum to quantum, behind by a different amount each time.
        let Ok(base_cursor) = self.forest.ticks(domain) else {
            return;
        };
        for slot in &self.lazy {
            if self.forest.root_of(slot.domain) != Ok(osc) {
                continue;
            }
            let (Ok(div), Ok(base_tick)) = (
                self.forest.domain(slot.domain).map(|d| d.units_per_tick()),
                self.forest.ticks(slot.domain),
            ) else {
                continue;
            };
            if div == 0 {
                continue;
            }
            slot.arm(Live {
                cursor: cursor.clone(),
                base_cursor,
                base_tick,
                mul,
                div,
            });
        }
        // Armed, so every slot can now say where its next event falls in the
        // runnable's own ticks — which is what the cursor needs in order to
        // catch them up from inside a cycle.
        cursor.watch(self.lazy_snapshot.clone());
    }

    /// Drop every live view. Between runnables the published position is the
    /// only honest one.
    fn disarm_live_cursors(&self, cursor: &TickCursor) {
        cursor.watch(None);
        for slot in &self.lazy {
            slot.disarm();
        }
    }

    /// Moves an idle machine forward without running anything.
    fn advance_idle_to(&mut self, to: GlobalTime) -> SchedResult<()> {
        if to <= self.now {
            return Ok(());
        }
        self.advance_undriven_trees(to)?;
        self.now = to;
        self.publish_lazy_positions();
        Ok(())
    }

    /// Advances every tree that no runnable drives, so a machine's passive
    /// crystals keep time.
    ///
    /// Recomputed each quantum rather than cached, because reparenting can move
    /// a domain between trees at runtime. When the topology generation counter
    /// exists (`ROADMAP.md` §15, invariant 3) this becomes derived state keyed
    /// on it, like every other cache.
    fn advance_undriven_trees(&mut self, to: GlobalTime) -> SchedResult<()> {
        let mut driven: Vec<bool> = alloc::vec![false; self.forest.domain_count()];
        // Indexed by oscillator, but sized by domains: a forest never has more
        // oscillators than domains, and this avoids a second count.
        for slot in &self.runnables {
            if let Ok(osc) = self.forest.root_of(slot.domain) {
                driven[osc.index()] = true;
            }
        }
        let oscillators: Vec<OscillatorId> = self.forest.oscillators().collect();
        for osc in oscillators {
            if driven[osc.index()] || !self.forest.is_active(osc)? {
                continue;
            }
            self.forest.advance_to_global(osc, to)?;
        }
        Ok(())
    }

    /// How many ticks of `domain` fit between its tree's current position and
    /// `target`.
    ///
    /// Recomputed from the absolute target every quantum rather than carried
    /// forward, so the rounding in the cross-tree step is bounded by one tick
    /// and cannot accumulate.
    fn ticks_until(&self, domain: DomainId, target: GlobalTime) -> SchedResult<u64> {
        self.ticks_until_after(domain, target, 0)
    }

    /// As [`Scheduler::ticks_until`], with `reserved` units of the tree already
    /// promised to somebody else.
    ///
    /// A tree has **one** unit counter and every domain on it is a divider of
    /// that counter, so two runnables on one tree do not advance independently
    /// — advancing either moves both. The deterministic round gets this right
    /// by accident of ordering: it advances each runnable's domain before
    /// computing the next one's budget, so the second runnable sees the
    /// position the first left.
    ///
    /// A parallel round hands every budget out before anything runs, so it has
    /// to reserve instead: each runnable on a tree is given the span its
    /// predecessors could not have used. The two agree exactly whenever every
    /// runnable consumes what it was given, which is the ordinary case; where
    /// one under-consumes, the deterministic round can hand the slack to a
    /// later runnable and the parallel round cannot, because it has already
    /// started them all.
    ///
    /// Note what this is *not* a workaround for: a board with two CPUs on one
    /// crystal is outside the clock model in **both** modes, since one counter
    /// cannot say that one of the two halted. Two CPUs want two oscillators —
    /// which is also what the hardware has (§4.2, "as many roots as the real
    /// board has crystals"). The reservation exists so that the degenerate
    /// configuration behaves the same in both modes rather than diverging
    /// silently.
    fn ticks_until_after(
        &self,
        domain: DomainId,
        target: GlobalTime,
        reserved: u64,
    ) -> SchedResult<u64> {
        if self.forest.is_gated(domain)? {
            return Ok(0);
        }
        let osc = self.forest.root_of(domain)?;
        let here = self.forest.unit_position(osc)?.saturating_add(reserved);
        let there = self.forest.units_at_global(osc, target)?;
        if there <= here {
            return Ok(0);
        }
        let per_tick = self.forest.domain(domain)?.units_per_tick();
        Ok((there - here) / per_tick)
    }

    // -- rate control -------------------------------------------------------

    /// Asks the rate controller whether to keep running.
    ///
    /// The only method that touches the injected host clock, and it never
    /// changes guest state: pacing decides *when* the host loop continues, not
    /// what the machine computes.
    ///
    /// # Errors
    ///
    /// [`SchedError::NoHostClock`] if the policy needs wall time and no clock
    /// was injected.
    pub fn pace(&mut self) -> SchedResult<Pace> {
        if matches!(self.rate.control(), RateControl::Unbounded) {
            return Ok(Pace::Run);
        }
        let clock = self.host_clock.as_ref().ok_or(SchedError::NoHostClock)?;
        let host_nanos = clock.monotonic_nanos();
        Ok(self.rate.pace(host_nanos, self.now))
    }

    /// The rate controller, for policy changes at runtime.
    #[inline]
    pub fn rate_controller_mut(&mut self) -> &mut RateController {
        &mut self.rate
    }

    // -- snapshots ----------------------------------------------------------

    /// Everything a snapshot has to carry about this scheduler (§4.5).
    ///
    /// See [`SchedulerSnapshot`] for what is in it and what is deliberately
    /// not.
    pub fn snapshot(&self) -> SchedulerSnapshot {
        SchedulerSnapshot {
            now: self.now,
            next_seq: self.queue.next_seq(),
            cursor: self.cursor,
            events: self.queue.events(),
        }
    }

    /// Restores what [`Scheduler::snapshot`] returned.
    ///
    /// The queue is replaced wholesale, virtual time is set to the saved
    /// instant, and the tie-break counter resumes where it left off — so the
    /// restored machine fires exactly the events the saved one would have, at
    /// exactly the same instants, in exactly the same order.
    ///
    /// Rate control re-anchors if a host clock is present: virtual time has
    /// just jumped, and an anchor from before the jump would have the machine
    /// either sprint or stall for however far it moved. Pacing is not guest
    /// state, so this is a re-anchoring rather than a restore.
    ///
    /// # Errors
    ///
    /// [`SchedError::InvalidSnapshot`] if the round-robin cursor does not name
    /// a registered runnable, or if the event set is not internally consistent
    /// — see [`EventQueue::restore`].
    pub fn restore(&mut self, snapshot: &SchedulerSnapshot) -> SchedResult<()> {
        let count = self.runnables.len();
        if (count == 0 && snapshot.cursor != 0) || (count > 0 && snapshot.cursor >= count) {
            return Err(SchedError::InvalidSnapshot(
                "the round-robin cursor does not name a registered runnable",
            ));
        }
        self.queue
            .restore(snapshot.now, snapshot.next_seq, &snapshot.events)?;
        self.now = snapshot.now;
        self.cursor = snapshot.cursor;
        self.publish_lazy_positions();
        // A restored machine is at whatever instant its snapshot says. Keeping
        // the old anchor would make the first accelerated round jump by the
        // difference between the two, in whichever direction.
        self.accel_anchor = None;
        if let Some(clock) = self.host_clock.as_ref() {
            let host_nanos = clock.monotonic_nanos();
            self.rate.reset(host_nanos, self.now);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::clock::Rational;

    fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn scheduler_pieces_are_send_and_sync() {
        // Threading is a configuration, never a retrofit (`ROADMAP.md` §0).
        assert_send_sync::<Scheduler>();
        assert_send_sync::<EventQueue>();
        assert_send_sync::<Event>();
        assert_send_sync::<RateController>();
    }

    // -- event queue --------------------------------------------------------

    fn t(ns: u64) -> GlobalTime {
        GlobalTime::from_nanos(ns)
    }

    fn drain(q: &mut EventQueue, now: GlobalTime) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        while let Some(e) = q.pop_due(now) {
            out.push((e.token, e.id.seq()));
        }
        out
    }

    #[test]
    fn events_fire_in_time_order_and_ties_break_by_sequence() {
        let mut q = EventQueue::default();
        // Posted out of order, and three of them at the very same instant.
        q.schedule(t(300), EventTarget(0), 30);
        let a = q.schedule(t(100), EventTarget(0), 10);
        let b = q.schedule(t(100), EventTarget(0), 11);
        let c = q.schedule(t(100), EventTarget(0), 12);
        q.schedule(t(200), EventTarget(0), 20);
        assert!(a.seq() < b.seq() && b.seq() < c.seq());

        let tokens: Vec<u64> = drain(&mut q, t(1_000))
            .iter()
            .map(|(tok, _)| *tok)
            .collect();
        assert_eq!(tokens, alloc::vec![10, 11, 12, 20, 30]);
    }

    #[test]
    fn ordering_is_identical_however_time_is_stepped() {
        // Determinism is not "the same answer if you ask the same way": the fire
        // order must not depend on how the caller chopped up the advance, or a
        // replay that pauses in a different place diverges.
        let build = || {
            let mut q = EventQueue::default();
            let mut rng = 0x1234_5678u64;
            for i in 0..500u64 {
                rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                // Spread across every level of the wheel and into the far heap.
                let when = t((rng >> 40) % 2_000_000_000);
                q.schedule(when, EventTarget((i % 7) as u32), i);
            }
            q
        };

        let mut one = build();
        let all = drain(&mut one, t(4_000_000_000));
        assert_eq!(all.len(), 500);

        let mut stepped = build();
        let mut piecewise = Vec::new();
        for step in 1..=4_000u64 {
            piecewise.extend(drain(&mut stepped, t(step * 1_000_000)));
        }
        assert_eq!(all, piecewise);

        // Sequence numbers are the tie-break, so the whole history is sorted by
        // (time, seq) — which is exactly the claim.
        let mut sorted = all.clone();
        sorted.sort_by_key(|(_, seq)| *seq);
        let mut by_time = all.clone();
        by_time.sort_by_key(|(_, seq)| *seq);
        assert_eq!(sorted, by_time);
    }

    #[test]
    fn far_future_events_come_back_through_the_wheel() {
        let mut q = EventQueue::default();
        // Well past the wheel's one-second span, so this starts in the heap.
        let far = t(60_000_000_000);
        q.schedule(far, EventTarget(1), 99);
        q.schedule(t(1_000), EventTarget(1), 1);

        assert_eq!(q.next_deadline(), Some(t(1_000)));
        assert_eq!(drain(&mut q, t(2_000)), alloc::vec![(1, 1)]);
        assert!(drain(&mut q, t(30_000_000_000)).is_empty());
        assert_eq!(q.next_deadline(), Some(far));
        assert_eq!(drain(&mut q, far), alloc::vec![(99, 0)]);
        assert!(q.is_empty());
    }

    #[test]
    fn a_huge_jump_expires_everything_it_passes() {
        // The wheel's cost is bounded by its slot count, not by the size of the
        // jump: this crosses every level and the heap in one call.
        let mut q = EventQueue::default();
        for i in 0..1_000u64 {
            q.schedule(t(i * 977 + 1), EventTarget(0), i);
        }
        let fired = drain(&mut q, t(10_000_000_000));
        assert_eq!(fired.len(), 1_000);
        for (i, (token, _)) in fired.iter().enumerate() {
            assert_eq!(*token, i as u64);
        }
        assert!(q.is_empty());
    }

    #[test]
    fn cancelled_events_never_fire() {
        let mut q = EventQueue::default();
        let a = q.schedule(t(100), EventTarget(0), 1);
        q.schedule(t(200), EventTarget(0), 2);
        let c = q.schedule(t(50_000_000_000), EventTarget(0), 3);
        q.cancel(a);
        q.cancel(c);
        assert_eq!(q.next_deadline(), Some(t(200)));
        assert_eq!(drain(&mut q, t(60_000_000_000)), alloc::vec![(2, 1)]);
    }

    #[test]
    fn an_event_in_the_past_still_fires() {
        // Dropping it would turn a one-tick scheduling slip into a lost
        // interrupt, which is unrecoverable and nearly undiagnosable.
        let mut q = EventQueue::default();
        q.advance_to(t(1_000));
        q.schedule(t(10), EventTarget(0), 7);
        assert_eq!(q.next_deadline(), Some(t(10)));
        assert_eq!(drain(&mut q, t(1_000)), alloc::vec![(7, 0)]);
    }

    #[test]
    fn next_deadline_is_exact_at_every_level_of_the_wheel() {
        for ns in [1u64, 500, 100_000, 900_000_000, 5_000_000_000] {
            let mut q = EventQueue::default();
            q.schedule(t(ns), EventTarget(0), 0);
            assert_eq!(q.next_deadline(), Some(t(ns)), "at {ns} ns");
            let just_before = t(ns).saturating_sub(GlobalTime::from_raw(1));
            assert!(drain(&mut q, just_before).is_empty(), "at {ns} ns");
            assert_eq!(q.next_deadline(), Some(t(ns)), "at {ns} ns");
            assert_eq!(drain(&mut q, t(ns)).len(), 1, "at {ns} ns");
        }
    }

    // -- budgets and the deterministic loop ---------------------------------

    /// A stand-in CPU. It uses its whole budget unless it has halted, and
    /// remembers what it was handed.
    #[derive(Debug, Default)]
    struct Cpu {
        budgets: Vec<u64>,
        halt_after: Option<u64>,
        total: u64,
    }

    impl Runnable for Cpu {
        fn run(&mut self, budget: Budget) -> Consumed {
            self.budgets.push(budget.ticks);
            let take = match self.halt_after {
                Some(limit) if self.total + budget.ticks > limit => {
                    limit.saturating_sub(self.total)
                }
                _ => budget.ticks,
            };
            self.total += take;
            Consumed::new(take)
        }
    }

    /// A runnable that lies about what it consumed.
    #[derive(Debug)]
    struct Liar;
    impl Runnable for Liar {
        fn run(&mut self, budget: Budget) -> Consumed {
            Consumed::new(budget.ticks + 1)
        }
    }

    fn nes_scheduler() -> (Scheduler, DomainId, DomainId) {
        let mut forest = ClockForest::new();
        let master = forest
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
        let ppu = forest.add_domain("ppu", master, 1, 4).unwrap();
        let sched = Scheduler::new(forest, SchedulerConfig::default());
        (sched, cpu, ppu)
    }

    #[test]
    fn a_budget_is_bounded_by_both_time_and_ticks() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        let id = sched.add_runnable(cpu, Box::new(Cpu::default()));
        assert_eq!(sched.runnable_domain(id).unwrap(), cpu);

        let report = sched.run_quantum().unwrap();
        // The default cap is 10 000 ticks and a 1 ms quantum is about 1 790 NES
        // CPU cycles, so time binds first.
        let (_, used) = report.consumed[0];
        assert!((1_700..1_800).contains(&used), "{used}");
        assert_eq!(sched.forest().ticks(cpu).unwrap(), used);
        // And the PPU followed exactly, without anyone converting through time.
        assert_eq!(sched.forest().ticks(ppu).unwrap(), used * 3);

        // Shrink the tick cap and the other limit binds instead.
        sched.config.max_ticks_per_quantum = 100;
        let report = sched.run_quantum().unwrap();
        assert_eq!(report.consumed[0].1, 100);
    }

    #[test]
    fn under_consumption_is_normal_and_self_correcting() {
        let (mut sched, cpu, _ppu) = nes_scheduler();
        sched.add_runnable(
            cpu,
            Box::new(Cpu {
                halt_after: Some(500),
                ..Cpu::default()
            }),
        );

        // The CPU halts after 500 ticks. Virtual time keeps moving anyway,
        // because a halted CPU does not stop the crystal.
        for _ in 0..5 {
            sched.run_quantum().unwrap();
        }
        assert_eq!(sched.forest().ticks(cpu).unwrap(), 500);
        assert!(sched.now() > GlobalTime::ZERO);
    }

    #[test]
    fn overrunning_a_budget_is_a_hard_error() {
        let (mut sched, cpu, _ppu) = nes_scheduler();
        let id = sched.add_runnable(cpu, Box::new(Liar));
        match sched.run_quantum() {
            Err(SchedError::BudgetExceeded {
                runnable,
                budget,
                consumed,
            }) => {
                assert_eq!(runnable, id);
                assert_eq!(consumed, budget + 1);
            }
            other => panic!("expected BudgetExceeded, got {other:?}"),
        }
    }

    #[test]
    fn the_round_robin_rotates_deterministically() {
        let mut forest = ClockForest::new();
        let root = forest
            .add_oscillator("xtal", Rational::integer(1_000_000))
            .unwrap();
        let a = forest.add_domain("a", root, 1, 1).unwrap();
        let b = forest.add_domain("b", root, 1, 1).unwrap();
        let c = forest.add_domain("c", root, 1, 1).unwrap();

        let mut sched = Scheduler::new(forest, SchedulerConfig::default());
        sched.add_runnable(a, Box::new(Cpu::default()));
        sched.add_runnable(b, Box::new(Cpu::default()));
        sched.add_runnable(c, Box::new(Cpu::default()));

        // No runnable is permanently first, and which one is first is a pure
        // function of the round number.
        let mut order = Vec::new();
        for _ in 0..5 {
            let report = sched.run_quantum().unwrap();
            order.push(
                report
                    .consumed
                    .iter()
                    .map(|(id, _)| id.index())
                    .collect::<Vec<_>>(),
            );
        }
        assert_eq!(
            order,
            alloc::vec![
                alloc::vec![0, 1, 2],
                alloc::vec![1, 2, 0],
                alloc::vec![2, 0, 1],
                alloc::vec![0, 1, 2],
                alloc::vec![1, 2, 0],
            ]
        );
    }

    // -- the accel mode -----------------------------------------------------

    /// A host clock a test moves by hand, so an accelerated round's outcome is
    /// a fact rather than a race.
    #[derive(Debug, Clone, Default)]
    struct StepClock(Arc<AtomicU64>);

    impl StepClock {
        fn advance(&self, nanos: u64) {
            self.0.fetch_add(nanos, AtomicOrdering::Relaxed);
        }
    }

    impl HostClock for StepClock {
        fn monotonic_nanos(&self) -> u64 {
            self.0.load(AtomicOrdering::Relaxed)
        }
    }

    /// A runnable that moves the host clock while it "executes", which is what
    /// a vCPU inside `KVM_RUN` does.
    #[derive(Debug)]
    struct Spinner {
        clock: StepClock,
        nanos: u64,
    }

    impl Runnable for Spinner {
        fn run(&mut self, budget: Budget) -> Consumed {
            self.clock.advance(self.nanos);
            // What an accelerated core must report: the whole budget, because
            // it cannot count guest ticks. Under `Emulated` that is what makes
            // the board's clocks run at a rate set by the quantum.
            Consumed::new(budget.ticks)
        }
    }

    fn accel_scheduler(clock: &StepClock) -> (Scheduler, DomainId) {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Accel, 0);
        sched.set_host_clock(Box::new(clock.clone()));
        (sched, cpu)
    }

    /// Without a clock there is no elapsed time, and guessing one is the whole
    /// defect this mode exists to fix.
    #[test]
    fn accel_says_so_rather_than_inventing_a_clock() {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Accel, 0);
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        assert_eq!(sched.run_quantum().unwrap_err(), SchedError::NoHostClock);
    }

    /// The mode's definition: a round is as long as the host clock says, and
    /// **not** as long as the runnable claimed.
    #[test]
    fn accel_takes_a_rounds_length_from_the_host_clock() {
        let clock = StepClock::default();
        let (mut sched, cpu) = accel_scheduler(&clock);
        sched.add_runnable(
            cpu,
            Box::new(Spinner {
                clock: clock.clone(),
                nanos: 4_000_000,
            }),
        );
        let report = sched.run_quantum().unwrap();
        // Four milliseconds of wall, against a 1 ms quantum whose budget the
        // runnable claimed in full. Under `Parallel` this round would have been
        // exactly one quantum long however long the host took, which is the
        // fact `hpet_counting()` and `timer_irq_works()` both trip over.
        let span = report.to.as_nanos() - report.from.as_nanos();
        assert!(
            (3_999_990..=4_000_010).contains(&span),
            "{span} ns of virtual time for 4 ms of host time"
        );
        // And the board's own clocks went with it, whole ticks of their own
        // domain: the NES CPU is 1.789773 MHz, so four milliseconds is about
        // 7 159 cycles rather than the 1 789 a 1 ms quantum would have given.
        let ticks = sched.forest().ticks(cpu).unwrap();
        assert!((7_100..7_200).contains(&ticks), "{ticks}");
    }

    /// Ten rounds of a millisecond each land on ten milliseconds, not on ten
    /// milliseconds plus ten roundings: every instant is computed from one
    /// anchor.
    #[test]
    fn accel_does_not_drift_across_rounds() {
        let clock = StepClock::default();
        let (mut sched, cpu) = accel_scheduler(&clock);
        sched.add_runnable(
            cpu,
            Box::new(Spinner {
                clock: clock.clone(),
                nanos: 1_000_000,
            }),
        );
        for _ in 0..10 {
            sched.run_quantum().unwrap();
        }
        assert_eq!(sched.now(), GlobalTime::from_nanos(10_000_000));
    }

    /// A round the host clock did not notice still moves virtual time, because
    /// a run loop reads *no advance* as a stalled machine and stops.
    #[test]
    fn accel_never_reports_a_round_of_no_length() {
        let clock = StepClock::default();
        let (mut sched, cpu) = accel_scheduler(&clock);
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let first = sched.run_quantum().unwrap();
        assert!(first.to > first.from);
        let second = sched.run_quantum().unwrap();
        assert!(second.to > second.from);
    }

    /// A zero quantum reaches the same place it does in the other modes: a
    /// round that moves nothing, which the run loop above reports as the
    /// configuration error it is. Spinning the guest on an empty budget for
    /// ever would be the alternative.
    #[test]
    fn accel_declines_a_zero_quantum_rather_than_spinning() {
        let clock = StepClock::default();
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Accel, 0);
        sched.config.quantum = GlobalTime::ZERO;
        sched.set_host_clock(Box::new(clock.clone()));
        sched.add_runnable(
            cpu,
            Box::new(Spinner {
                clock: clock.clone(),
                nanos: 1_000_000,
            }),
        );
        let report = sched.run_quantum().unwrap();
        assert_eq!(report.to, report.from);
        assert!(report.consumed.is_empty(), "nothing may have run");
    }

    /// A deadline inside a round bounds the budgets instead of declining the
    /// round: a guest that is never entered is a machine that has stopped.
    #[test]
    fn accel_runs_the_round_a_deadline_falls_inside() {
        let clock = StepClock::default();
        let (mut sched, cpu) = accel_scheduler(&clock);
        sched.add_runnable(
            cpu,
            Box::new(Spinner {
                clock: clock.clone(),
                nanos: 2_000_000,
            }),
        );
        // A tenth of the quantum. `Parallel` would decline this round outright.
        let report = sched
            .run_quantum_until(GlobalTime::from_nanos(100_000))
            .unwrap();
        assert_eq!(report.consumed.len(), 1);
        assert!(report.consumed[0].1 > 0, "the runnable never ran");
        // Two milliseconds, less the one unit `as_nanos` floors away.
        assert!(report.to.as_nanos() >= 1_999_999, "{:?}", report.to);
    }

    /// `fixed-ratio` still means what it says: half speed is half as much
    /// virtual time for the same wall.
    #[test]
    fn accel_honours_a_fixed_ratio() {
        let clock = StepClock::default();
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Accel, 0);
        sched.config.rate = RateControl::FixedRatio { num: 1, den: 2 };
        sched.rate_controller_mut().set_control(
            RateControl::FixedRatio { num: 1, den: 2 },
            0,
            GlobalTime::ZERO,
        );
        sched.set_host_clock(Box::new(clock.clone()));
        sched.add_runnable(
            cpu,
            Box::new(Spinner {
                clock: clock.clone(),
                nanos: 4_000_000,
            }),
        );
        sched.run_quantum().unwrap();
        assert_eq!(sched.now(), GlobalTime::from_nanos(2_000_000));
    }

    // -- the parallel mode --------------------------------------------------

    /// The same forest as [`nes_scheduler`], in whichever mode is asked for.
    ///
    /// `workers` goes to the pool. Zero is the honest configuration on a
    /// backend with no threads and is also what the equivalence tests want:
    /// they are about the *bookkeeping* being the same, and a worker thread
    /// would only add a source of variance to a workload that has none.
    fn nes_scheduler_in(mode: ThreadingMode, workers: usize) -> (Scheduler, DomainId, DomainId) {
        let mut forest = ClockForest::new();
        let master = forest
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
        let ppu = forest.add_domain("ppu", master, 1, 4).unwrap();
        let config = SchedulerConfig {
            mode,
            workers,
            ..SchedulerConfig::default()
        };
        (Scheduler::new(forest, config), cpu, ppu)
    }

    /// A workload whose result cannot depend on the interleaving: three cores
    /// that consume everything they are given and observe nothing.
    ///
    /// That is the point. The claim under test is that *the scheduler's own*
    /// arithmetic — budgets, tick counters, virtual time, the event queue — is
    /// the same in both modes, so the workload must contribute no variance of
    /// its own or the test would be measuring the guests instead.
    fn three_cores(sched: &mut Scheduler, domains: &[DomainId]) {
        for domain in domains {
            sched.add_runnable(*domain, Box::new(Cpu::default()));
        }
    }

    #[test]
    fn a_parallel_round_moves_time_exactly_as_a_deterministic_one_does() {
        let mut ends = Vec::new();
        for mode in [ThreadingMode::Deterministic, ThreadingMode::Parallel] {
            let (mut sched, cpu, ppu) = nes_scheduler_in(mode, 0);
            three_cores(&mut sched, &[cpu, ppu]);
            sched.schedule_at(t(400_000), EventTarget(7), 11);
            let mut fired = Vec::new();
            for _ in 0..8 {
                let report = sched.run_quantum().unwrap();
                fired.extend(report.fired.iter().map(|e| (e.time, e.token)));
            }
            ends.push((
                sched.now(),
                sched.forest().ticks(cpu).unwrap(),
                sched.forest().ticks(ppu).unwrap(),
                fired,
            ));
        }
        assert_eq!(
            ends[0], ends[1],
            "the parallel round hands out the same budgets and keeps the same time"
        );
    }

    #[test]
    fn a_parallel_round_reports_every_runnable_in_registration_order() {
        let (mut sched, cpu, ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        three_cores(&mut sched, &[cpu, ppu, cpu]);
        // No rotation: there is no "first" runnable in a round where every one
        // of them starts at once, so the order is the registration order, every
        // round, and the round-robin cursor stays where it was.
        for _ in 0..3 {
            let report = sched.run_quantum().unwrap();
            let order: Vec<usize> = report.consumed.iter().map(|(id, _)| id.index()).collect();
            assert_eq!(order, alloc::vec![0, 1, 2]);
        }
    }

    #[test]
    fn a_parallel_round_still_declines_a_round_its_deadline_falls_inside() {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        // Well inside the 1 ms grid: the round does not start, virtual time
        // moves to the deadline, and nothing executes. That is what keeps
        // `Machine::run_for`'s shape (§11.6) in this mode too.
        let report = sched.run_quantum_until(t(1_000)).unwrap();
        assert_eq!(report.to, t(1_000));
        assert!(report.consumed.is_empty());
        assert_eq!(sched.forest().ticks(cpu).unwrap(), 0);
    }

    #[test]
    fn a_parallel_round_still_refuses_a_runnable_that_overran() {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        sched.add_runnable(cpu, Box::new(Liar));
        assert!(matches!(
            sched.run_quantum().unwrap_err(),
            SchedError::BudgetExceeded { .. }
        ));
    }

    // -- the safe point -----------------------------------------------------

    /// A core that consults its exit flag between "instructions".
    ///
    /// One tick per instruction, so "how many did it manage" is exactly "how
    /// many instructions before it was told to stop".
    #[derive(Debug)]
    struct Stoppable {
        cursor: Arc<Mutex<Option<TickCursor>>>,
        /// Raise the world stop once this many ticks have gone by, so the test
        /// can stop the world *from inside* a round rather than between them.
        stop_at: Option<u64>,
        safe: SafePoint,
    }

    impl Runnable for Stoppable {
        fn run(&mut self, budget: Budget) -> Consumed {
            let cursor = self.cursor.lock().clone();
            let mut used = 0;
            while used < budget.ticks {
                used += 1;
                if let Some(cursor) = cursor.as_ref() {
                    cursor.set(used);
                    if self.stop_at == Some(used) {
                        self.safe.request();
                    }
                    if cursor.exit_requested() {
                        break;
                    }
                }
            }
            Consumed::new(used)
        }
    }

    /// Register a [`Stoppable`] and hand it the cursor the scheduler made for
    /// it.
    fn stoppable(
        sched: &mut Scheduler,
        domain: DomainId,
        stop_at: Option<u64>,
    ) -> (RunnableId, Arc<Mutex<Option<TickCursor>>>) {
        let slot = Arc::new(Mutex::new(None));
        let safe = sched.safe_point();
        let id = sched.add_runnable(
            domain,
            Box::new(Stoppable {
                cursor: Arc::clone(&slot),
                stop_at,
                safe,
            }),
        );
        *slot.lock() = Some(sched.runnable_cursor(id).unwrap());
        (id, slot)
    }

    #[test]
    fn nothing_raises_an_exit_flag_in_an_ordinary_run() {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Deterministic, 0);
        let (id, _) = stoppable(&mut sched, cpu, None);
        let report = sched.run_quantum().unwrap();
        let (_, used) = report.consumed[0];
        assert!(used > 0);
        assert_eq!(sched.forest().ticks(cpu).unwrap(), used);
        assert!(!sched.exit_flag(id).unwrap().raised());
        assert_eq!(sched.safe_point().generation(), 0);
    }

    #[test]
    fn a_world_stop_unwinds_every_runnable_at_its_next_block_boundary() {
        let (mut sched, cpu, ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        // The first runnable stops the world on its fourth tick; the second
        // one, which runs afterwards, must see the flag from its very first.
        stoppable(&mut sched, cpu, Some(4));
        stoppable(&mut sched, ppu, None);
        let report = sched.run_quantum().unwrap();
        assert_eq!(
            report.consumed[0].1, 4,
            "the requester stopped where it asked"
        );
        assert_eq!(
            report.consumed[1].1, 1,
            "the second unwound at its first boundary"
        );
        assert_eq!(sched.safe_point().generation(), 1);
        assert!(sched.safe_point().stop_requested());
    }

    #[test]
    fn a_stop_guard_holds_the_world_and_lets_it_go_again() {
        let (mut sched, cpu, _ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        stoppable(&mut sched, cpu, None);
        {
            let guard = sched.stop_the_world();
            assert_eq!(guard.generation(), 1);
            assert!(sched.safe_point().stop_requested());
            // A round under the guard executes the minimum a runnable can
            // notice the flag in and no more.
            let report = sched.run_quantum().unwrap();
            assert_eq!(report.consumed[0].1, 1);
        }
        assert!(!sched.safe_point().stop_requested());
        let report = sched.run_quantum().unwrap();
        assert!(report.consumed[0].1 > 1, "the world runs again");
        // The generation only ever rises, so a cache keyed on it knows it is
        // stale even though the flag has been raised and lowered since.
        assert_eq!(sched.safe_point().generation(), 1);
        assert_eq!(sched.stop_the_world().generation(), 2);
    }

    #[test]
    fn an_exit_flag_can_stop_one_runnable_and_leave_the_rest() {
        let (mut sched, cpu, ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        let (a, _) = stoppable(&mut sched, cpu, None);
        stoppable(&mut sched, ppu, None);
        sched.exit_flag(a).unwrap().raise();
        let report = sched.run_quantum().unwrap();
        assert_eq!(report.consumed[0].1, 1, "the one that was asked");
        assert!(report.consumed[1].1 > 1, "and only that one");
        assert!(
            !sched.safe_point().stop_requested(),
            "no world stop happened"
        );
    }

    #[test]
    fn two_runnables_on_one_tree_share_its_counter_in_both_modes() {
        // Written down because it is a **finding**, not a feature. A tree has
        // one unit counter and every domain on it is a divider of that counter,
        // so two runnables on one oscillator do not advance independently:
        // whatever the first one consumes has already moved the second one's
        // clock. The deterministic round gets that right by accident of
        // ordering — it advances each domain before computing the next budget —
        // and the second runnable is left with almost nothing.
        //
        // Neither mode can do better without a per-domain counter, which is a
        // change to `core::clock` and not to this file. What both modes *must*
        // do is agree, and a parallel round that handed every runnable the
        // whole span would advance the tree twice over. Hence the reservation
        // in `ticks_until_after`, and hence this test.
        //
        // The design answer for a board that wants two CPUs is two
        // oscillators; `machines/tests/heterogeneous.machine` says so at
        // length, and it is what the hardware has.
        let mut reports = Vec::new();
        for mode in [ThreadingMode::Deterministic, ThreadingMode::Parallel] {
            let (mut sched, cpu, ppu) = nes_scheduler_in(mode, 0);
            sched.add_runnable(cpu, Box::new(Cpu::default()));
            sched.add_runnable(ppu, Box::new(Cpu::default()));
            let report = sched.run_quantum().unwrap();
            let by_id = |id: usize| {
                report
                    .consumed
                    .iter()
                    .find(|(r, _)| r.index() == id)
                    .map(|(_, n)| *n)
                    .unwrap()
            };
            reports.push((by_id(0), by_id(1), sched.forest().ticks(cpu).unwrap()));
        }
        assert_eq!(
            reports[0], reports[1],
            "the two modes hand out the same ticks"
        );
        let (first, second, _) = reports[0];
        assert!(
            first > 1_000,
            "the first runnable got the tree's whole span"
        );
        assert!(
            second < first / 100,
            "the second one is left with what the first did not use ({second} against {first})"
        );
    }

    #[test]
    fn a_parallel_round_leaves_a_shared_tree_on_the_published_position() {
        // Two runnables on one oscillator tree: no single live position exists,
        // so nothing is armed and catch-up falls back to what the scheduler
        // last published. The alternative — arming one of the two arbitrarily —
        // would make a device's dot depend on which runnable happened to be
        // registered first.
        let (mut sched, cpu, ppu) = nes_scheduler_in(ThreadingMode::Parallel, 0);
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.add_runnable(ppu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        sched.run_quantum().unwrap();
        sched.sync_lazy_devices().unwrap();
        // Caught up to the quantum's own boundary, exactly as a device on
        // another tree always was.
        assert_eq!(
            sched.sync_for_access(dev, AccessKind::Guest).unwrap(),
            sched.forest().ticks(ppu).unwrap()
        );
    }

    #[test]
    fn a_quantum_never_runs_past_a_scheduled_event() {
        let (mut sched, cpu, _ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        // Half a microsecond in, far inside the default 1 ms quantum.
        sched.schedule_at(t(500), EventTarget(3), 42);
        let report = sched.run_quantum().unwrap();
        assert_eq!(report.to, t(500));
        assert_eq!(report.fired.len(), 1);
        assert_eq!(report.fired[0].token, 42);
        assert_eq!(report.fired[0].target, EventTarget(3));
    }

    #[test]
    fn events_can_be_scheduled_in_domain_ticks() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        // NES vblank: scanline 241, dot 0, counted in PPU dots from reset.
        let dot = 241 * 341;
        let deadline = sched.forest().global_time_of_tick(ppu, dot).unwrap();
        sched.schedule_at_tick(ppu, dot, EventTarget(1), 0).unwrap();

        // Run until it fires, and check the machine stopped exactly there rather
        // than somewhere in its neighbourhood.
        let mut fired_at = None;
        for _ in 0..100 {
            let report = sched.run_quantum().unwrap();
            if let Some(e) = report.fired.first() {
                fired_at = Some((report.to, e.token));
                break;
            }
        }
        assert_eq!(fired_at, Some((deadline, 0)));

        // The event fires at exactly the instant of that dot. The PPU's counter
        // is at, or just short of, the dot itself: the deadline falls two thirds
        // of the way through a CPU cycle, and the CPU is not stopped mid-cycle.
        // Short by less than one driving tick — never past it. See the module
        // documentation.
        let at = sched.forest().ticks(ppu).unwrap();
        assert!((dot - 3..=dot).contains(&at), "{at} vs {dot}");
        assert_eq!(sched.forest().ticks(cpu).unwrap() * 3, at);
    }

    // -- catch-up -----------------------------------------------------------

    /// A stand-in PPU: it remembers the dot it has been advanced to and can
    /// declare an internal event it must not be simulated past.
    #[derive(Debug, Default)]
    struct Ppu {
        tick: u64,
        next_event: Option<u64>,
        advances: u32,
    }

    impl LazyDevice for Ppu {
        fn current_tick(&self) -> u64 {
            self.tick
        }
        fn advance_to(&mut self, tick: u64) {
            assert!(tick >= self.tick, "advance_to must never go backwards");
            self.tick = tick;
            self.advances += 1;
        }
        fn next_event_tick(&self) -> Option<u64> {
            self.next_event
        }
    }

    #[test]
    fn catch_up_puts_a_lazy_device_exactly_where_the_access_is() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));

        sched.run_quantum().unwrap();
        let cpu_ticks = sched.forest().ticks(cpu).unwrap();
        assert!(cpu_ticks > 1_000);

        // The device has not moved at all yet — that is the point of laziness.
        assert_eq!(sched.sync_for_access(dev, AccessKind::Debug).unwrap(), 0);

        // A guest access drags it to exactly the current dot: three per CPU
        // cycle, arrived at without a single absolute-time conversion. This is
        // what makes a `$2002` read see the right vblank flag.
        let at = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
        assert_eq!(at, cpu_ticks * 3);
    }

    /// A core that publishes its position and samples a lazy device mid-run.
    ///
    /// The shape of a 6502 reading `$2002`: the read happens on cycle `at` of
    /// the budget, thousands of cycles before the quantum ends, and the answer
    /// has to describe *that* cycle.
    #[derive(Debug)]
    struct SamplingCpu {
        /// Handed over after registration, exactly as the machine layer does it.
        cursor: Arc<Mutex<Option<TickCursor>>>,
        slot: Arc<LazySlot>,
        /// Which cycle of the run to sample on.
        at: u64,
        /// The device tick the sample saw.
        saw: Arc<AtomicU64>,
        ticks: u64,
    }

    impl Runnable for SamplingCpu {
        fn run(&mut self, budget: Budget) -> Consumed {
            let cursor = self.cursor.lock().clone();
            for _ in 0..budget.ticks {
                self.ticks += 1;
                if let Some(cursor) = &cursor {
                    cursor.set(self.ticks);
                }
                if self.ticks == self.at {
                    let at = self
                        .slot
                        .sync(LazyId(0), None, AccessKind::Guest)
                        .expect("the device is registered");
                    self.saw.store(at, AtomicOrdering::Relaxed);
                }
            }
            Consumed::new(budget.ticks)
        }
    }

    fn sampling_cpu(sched: &mut Scheduler, cpu: DomainId, dev: LazyId, at: u64) -> Arc<AtomicU64> {
        let saw = Arc::new(AtomicU64::new(u64::MAX));
        let cursor = Arc::new(Mutex::new(None));
        let id = sched.add_runnable(
            cpu,
            Box::new(SamplingCpu {
                cursor: Arc::clone(&cursor),
                slot: Arc::clone(&sched.lazy[dev.index()]),
                at,
                saw: Arc::clone(&saw),
                ticks: 0,
            }),
        );
        *cursor.lock() = Some(sched.runnable_cursor(id).expect("just registered"));
        saw
    }

    #[test]
    fn a_published_position_makes_catch_up_dot_exact_inside_a_quantum() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let saw = sampling_cpu(&mut sched, cpu, dev, 40);

        sched.run_quantum().unwrap();
        // Three dots per CPU cycle, sampled on cycle 40 — not at the start of
        // the quantum (0) and not at its end (thousands of dots later). This is
        // the whole point of `TickCursor`.
        assert_eq!(saw.load(AtomicOrdering::Relaxed), 120);
    }

    #[test]
    fn a_core_that_publishes_nothing_still_sees_the_quantums_position() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let saw = Arc::new(AtomicU64::new(u64::MAX));
        sched.add_runnable(
            cpu,
            Box::new(SamplingCpu {
                // Never given one: publishing is optional.
                cursor: Arc::new(Mutex::new(None)),
                slot: Arc::clone(&sched.lazy[dev.index()]),
                at: 40,
                saw: Arc::clone(&saw),
                ticks: 0,
            }),
        );
        sched.run_quantum().unwrap();
        assert_eq!(
            saw.load(AtomicOrdering::Relaxed),
            0,
            "with nothing published the device stands where the quantum began"
        );
    }

    #[test]
    fn catch_up_stops_at_the_devices_own_next_event() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(
            ppu,
            Box::new(Ppu {
                next_event: Some(100),
                ..Ppu::default()
            }),
        );
        // Two rounds, because a round is now bounded by the device's own next
        // event: the first stops at dot 100 and the second, with that deadline
        // behind it, runs a whole quantum. (This stand-in never moves its
        // event; a real device advances it, which is why
        // `Device::next_event_tick` documents that it must.)
        sched.run_quantum().unwrap();
        sched.run_quantum().unwrap();
        // Thousands of dots have passed, but the device may not be simulated
        // past dot 100, where its own behaviour changes.
        assert!(sched.forest().ticks(ppu).unwrap() > 5_000);
        assert_eq!(sched.sync_for_access(dev, AccessKind::Guest).unwrap(), 100);
    }

    #[test]
    fn a_debug_access_advances_nothing() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        sched.run_quantum().unwrap();
        for _ in 0..10 {
            assert_eq!(sched.sync_for_access(dev, AccessKind::Debug).unwrap(), 0);
        }
        assert!(sched.sync_for_access(dev, AccessKind::Guest).unwrap() > 0);
    }

    #[test]
    fn catch_up_is_idempotent_and_monotone() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let mut last = 0;
        for _ in 0..20 {
            sched.run_quantum().unwrap();
            let a = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
            let b = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
            assert_eq!(a, b, "a second sync with no time passing must be a no-op");
            assert!(a >= last);
            last = a;
        }
        assert_eq!(last, sched.forest().ticks(cpu).unwrap() * 3);
    }

    #[test]
    fn a_device_can_be_put_on_its_own_event_tick() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let dot = 241 * 341;
        sched.schedule_at_tick(ppu, dot, EventTarget(1), 0).unwrap();
        for _ in 0..100 {
            if !sched.run_quantum().unwrap().fired.is_empty() {
                break;
            }
        }
        // Catch-up alone stops just short, because the CPU cycle containing that
        // dot has not finished. Delivering the event puts the device exactly on
        // the dot it asked for.
        let caught_up = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
        assert!(caught_up < dot && caught_up >= dot - 3);
        assert_eq!(sched.sync_to_tick(dev, dot).unwrap(), dot);
        // And it never goes backwards.
        assert_eq!(sched.sync_to_tick(dev, dot - 10).unwrap(), dot);
    }

    // -- catch-up from an access path ---------------------------------------

    /// A CPU that reads a lazily-advanced device from inside its own execution
    /// — an MMIO read in miniature. It holds a [`LazyHandle`] and nothing else:
    /// no borrow of the scheduler, which is what the real path cannot have.
    struct SyncingCpu {
        handle: LazyHandle,
        seen: Arc<Mutex<Vec<u64>>>,
    }

    impl Runnable for SyncingCpu {
        fn run(&mut self, budget: Budget) -> Consumed {
            let at = self.handle.sync(AccessKind::Guest).expect("catch-up");
            self.seen.lock().push(at);
            Consumed::new(budget.ticks)
        }
    }

    #[test]
    fn a_device_is_caught_up_from_inside_a_running_cpu() {
        // The whole point of §4.2's sync-on-access: the trigger is a memory
        // access several frames below the run loop, with no way back to the
        // scheduler. A handle is that way.
        let (mut sched, cpu, ppu) = nes_scheduler();
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let handle = sched.lazy_handle(dev).expect("a handle");
        let seen = Arc::new(Mutex::new(Vec::new()));
        sched.add_runnable(
            cpu,
            Box::new(SyncingCpu {
                handle,
                seen: Arc::clone(&seen),
            }),
        );

        sched.run_quantum().unwrap();
        let after_one = sched.forest().ticks(cpu).unwrap();
        assert!(after_one > 1_000);
        sched.run_quantum().unwrap();

        let seen = seen.lock().clone();
        assert_eq!(seen[0], 0, "nothing has run before the first quantum");
        // The read in the second quantum sees the dot the CPU had reached, at
        // three dots per cycle, arrived at without one absolute-time
        // conversion. A runnable's progress *within* the quantum it is
        // currently in is not in the clock forest yet — the forest is advanced
        // from its report, after it returns — so this is the position at the
        // quantum boundary. See `LazyHandle`.
        assert_eq!(seen[1], after_one * 3);
    }

    /// A device with something observable to be wrong about: a flag that goes
    /// up at a known dot, which a stale device would report the wrong side of.
    #[derive(Debug)]
    struct FlagPpu {
        tick: u64,
        flag_at: u64,
        flag: Arc<Mutex<bool>>,
    }

    impl LazyDevice for FlagPpu {
        fn current_tick(&self) -> u64 {
            self.tick
        }
        fn advance_to(&mut self, tick: u64) {
            self.tick = tick;
            if tick >= self.flag_at {
                *self.flag.lock() = true;
            }
        }
    }

    #[test]
    fn an_access_reads_the_value_the_device_had_at_that_very_tick() {
        // One quantum is about 1 790 NES CPU cycles, so 5 370 dots.
        for (flag_at, expected) in [(100u64, true), (1_000_000u64, false)] {
            let (mut sched, cpu, ppu) = nes_scheduler();
            let flag = Arc::new(Mutex::new(false));
            let dev = sched.add_lazy_device(
                ppu,
                Box::new(FlagPpu {
                    tick: 0,
                    flag_at,
                    flag: Arc::clone(&flag),
                }),
            );
            let handle = sched.lazy_handle(dev).expect("a handle");
            sched.add_runnable(cpu, Box::new(Cpu::default()));
            sched.run_quantum().unwrap();

            // Stale until somebody looks: that is what makes it cheap.
            assert!(!*flag.lock(), "at {flag_at}");
            handle.sync(AccessKind::Guest).expect("catch-up");
            assert_eq!(*flag.lock(), expected, "at {flag_at}");
        }
    }

    #[test]
    fn catch_up_takes_nothing_a_bus_access_may_not_nest_under() {
        use crate::core::sync::{self, LockRank};

        let (mut sched, cpu, ppu) = nes_scheduler();
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let handle = sched.lazy_handle(dev).expect("a handle");
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.run_quantum().unwrap();
        let dot = sched.forest().ticks(ppu).unwrap();

        // An MMIO read holds the bus fabric's lock. `LockRank::SCHED` is above
        // `LockRank::BUS`, so reaching back for the scheduler from here is a
        // ladder inversion — and a deadlock the moment two CPUs on two buses do
        // it at once.
        let _bus = LockRank::BUS.enter();
        assert_eq!(
            sync::violates_lock_order(LockRank::SCHED),
            cfg!(debug_assertions),
            "the inversion this design exists to avoid"
        );
        // Catch-up does not take it. In a debug build the ladder is live, so
        // anything at or below `BUS` would panic here rather than pass.
        assert_eq!(handle.sync(AccessKind::Guest).unwrap(), dot);
    }

    #[test]
    fn a_device_nobody_reads_is_still_caught_up_at_the_quantum_boundary() {
        // The other half of sync-on-access, and the reason a bound-but-never-
        // advanced PPU is worse than no PPU: a game whose main loop spins on a
        // flag its NMI handler sets never touches a PPU register, so nothing
        // would ever drag the chip to the dot that raises vblank.
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let handle = sched.lazy_handle(dev).expect("a handle");

        sched.run_quantum().unwrap();
        assert_eq!(handle.current_tick().unwrap(), 0, "nothing looked at it");

        sched.sync_lazy_devices().unwrap();
        let dot = sched.forest().ticks(ppu).unwrap();
        assert_eq!(handle.current_tick().unwrap(), dot);
        assert_eq!(dot, sched.forest().ticks(cpu).unwrap() * 3);
    }

    #[test]
    fn catch_up_crosses_a_run_of_internal_events_one_at_a_time() {
        // A single `sync` stops at the device's own next event. A quantum may
        // contain many of them — a PPU stopping at every scanline crosses 15 in
        // a millisecond — so reaching the present takes a loop, and
        // `sync_lazy_devices` is where it lives.
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let dev = sched.add_lazy_device(
            ppu,
            Box::new(Ppu {
                // Never more than 100 dots at a time.
                next_event: Some(100),
                ..Ppu::default()
            }),
        );
        let handle = sched.lazy_handle(dev).expect("a handle");
        // Twice, for the reason in `catch_up_stops_at_the_devices_own_next_event`:
        // the first round ends *on* dot 100 and the second runs past it.
        sched.run_quantum().unwrap();
        sched.run_quantum().unwrap();

        // One sync alone stops at the declared event and goes no further.
        assert_eq!(handle.sync(AccessKind::Guest).unwrap(), 100);
        // The scheduler's own pass reaches the present anyway. (This stand-in
        // device never moves its event, so the loop's second guard — no
        // progress — is what ends it; a real device advances its event, which
        // is why `Device::next_event_tick` documents that it must.)
        sched.sync_lazy_devices().unwrap();
        assert_eq!(handle.current_tick().unwrap(), 100);
    }

    #[test]
    fn a_quantum_can_be_bounded_by_a_lazy_devices_own_event() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));

        // Nothing lazy: nothing to bound a quantum by.
        assert_eq!(sched.lazy_deadline(), None);

        // A device with no event of its own likewise reports none.
        let plain = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        assert_eq!(sched.lazy_deadline(), None);
        let _ = plain;

        // One with an event names the instant that dot falls on, which is
        // exactly where a run loop must stop the CPU: past it the NMI has been
        // raised and the CPU has already run through it.
        let dot = 4_000u64;
        sched.add_lazy_device(
            ppu,
            Box::new(Ppu {
                next_event: Some(dot),
                ..Ppu::default()
            }),
        );
        let at = sched.lazy_deadline().expect("a deadline");
        assert_eq!(at, sched.forest().global_time_of_tick(ppu, dot).unwrap());

        // Running to it leaves the CPU one dot-worth of rounding short of the
        // event and never past it, so catch-up lands the device *on* the dot.
        sched.run_until(at).unwrap();
        sched.sync_lazy_devices().unwrap();
        assert!(sched.forest().ticks(ppu).unwrap() <= dot);

        // And an event already behind virtual time is not reported: clamping a
        // quantum to an instant the machine is standing on would stall it.
        while sched.lazy_deadline().is_some() {
            let at = sched.lazy_deadline().expect("checked");
            if at <= sched.now() {
                break;
            }
            sched.run_until(at).unwrap();
            sched.run_quantum().unwrap();
        }
        assert_eq!(sched.lazy_deadline(), None);
    }

    /// A device that reads its own registers as it simulates — the one way a
    /// catch-up can re-enter itself.
    #[derive(Debug)]
    struct SelfReadingPpu {
        tick: u64,
        me: Arc<Mutex<Option<LazyHandle>>>,
        saw: Arc<Mutex<Option<SchedError>>>,
    }

    impl LazyDevice for SelfReadingPpu {
        fn current_tick(&self) -> u64 {
            self.tick
        }
        fn advance_to(&mut self, tick: u64) {
            let me = self.me.lock().clone();
            if let Some(handle) = me {
                *self.saw.lock() = handle.sync(AccessKind::Guest).err();
            }
            self.tick = tick;
        }
    }

    #[test]
    fn a_re_entrant_catch_up_is_reported_rather_than_deadlocked() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        let me = Arc::new(Mutex::new(None));
        let saw = Arc::new(Mutex::new(None));
        let dev = sched.add_lazy_device(
            ppu,
            Box::new(SelfReadingPpu {
                tick: 0,
                me: Arc::clone(&me),
                saw: Arc::clone(&saw),
            }),
        );
        *me.lock() = Some(sched.lazy_handle(dev).expect("a handle"));
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.run_quantum().unwrap();

        // The outer catch-up succeeds; the inner one finds the device in flight
        // and says so. Waiting would be a deadlock and recursing would need two
        // mutable borrows of one device, so this is the only honest answer.
        assert!(sched.sync_for_access(dev, AccessKind::Guest).unwrap() > 0);
        assert_eq!(*saw.lock(), Some(SchedError::LazyDeviceBusy(dev)));
    }

    #[test]
    fn a_handle_and_the_scheduler_reach_the_same_device() {
        let (mut sched, cpu, ppu) = nes_scheduler();
        let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
        let handle = sched.lazy_handle(dev).expect("a handle");
        assert_eq!(handle.id(), dev);
        assert_eq!(handle.domain(), ppu);
        assert_eq!(sched.lazy_domain(dev).unwrap(), ppu);

        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.run_quantum().unwrap();
        let through_the_scheduler = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
        assert_eq!(handle.current_tick().unwrap(), through_the_scheduler);
        assert_eq!(handle.present_tick(), through_the_scheduler);
        // And a second sync through either route is a no-op.
        assert_eq!(
            handle.sync(AccessKind::Guest).unwrap(),
            through_the_scheduler
        );
    }

    #[test]
    fn unknown_handles_are_errors_not_panics() {
        let (sched, _cpu, _ppu) = nes_scheduler();
        let bogus_device = LazyId(7);
        assert_eq!(
            sched
                .sync_for_access(bogus_device, AccessKind::Guest)
                .unwrap_err(),
            SchedError::UnknownLazyDevice(bogus_device)
        );
        let bogus_runnable = RunnableId(7);
        assert_eq!(
            sched.runnable_domain(bogus_runnable).unwrap_err(),
            SchedError::UnknownRunnable(bogus_runnable)
        );
    }

    // -- undriven trees -----------------------------------------------------

    #[test]
    fn a_crystal_nothing_drives_still_keeps_time() {
        let mut forest = ClockForest::new();
        let master = forest
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
        let rtc = forest
            .add_oscillator("rtc", Rational::integer(32_768))
            .unwrap();
        let seconds = forest.add_domain("seconds", rtc, 1, 32_768).unwrap();

        let mut sched = Scheduler::new(forest, SchedulerConfig::default());
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.run_until(t(2_000_000_000)).unwrap();

        // Two seconds of virtual time: the RTC has ticked twice, on its own
        // crystal, through the one cross-tree conversion that is legitimate.
        assert_eq!(sched.forest().ticks(seconds).unwrap(), 2);
    }

    #[test]
    fn an_event_due_at_the_current_instant_does_not_end_the_run() {
        // A quantum can legitimately advance no time at all, when an event is
        // due at this very instant. Treating that as "the machine is idle" would
        // silently stop the run at the first such event.
        let (mut sched, cpu, _ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        sched.schedule_at(GlobalTime::ZERO, EventTarget(0), 1);
        // Two milliseconds rather than two microseconds: a round runs only when
        // the deadline reaches its boundary, so a deadline inside the first
        // quantum would leave the CPU untouched for a reason that has nothing
        // to do with what this test is about (see `run_quantum_until`).
        sched.run_until(t(2_000_000)).unwrap();
        assert_eq!(sched.now(), t(2_000_000));
        assert!(sched.forest().ticks(cpu).unwrap() > 0);
    }

    /// A quantum that is not a whole number of nanoseconds still has a grid,
    /// counted in raw units — the fallback in `next_grid_point`.
    #[test]
    fn a_sub_nanosecond_quantum_still_has_an_absolute_grid() {
        assert_eq!(
            whole_nanos(GlobalTime::from_nanos(1_000_000)),
            Some(1_000_000)
        );
        // 2⁻²⁰ s is 953.674… ns, which is not a whole number of them.
        let quantum = GlobalTime::from_raw(1 << 44);
        assert_eq!(whole_nanos(quantum), None);

        let mut forest = ClockForest::new();
        let root = forest
            .add_oscillator("xtal", Rational::integer(1_000_000))
            .unwrap();
        let domain = forest.add_domain("d", root, 1, 1).unwrap();
        let config = SchedulerConfig {
            quantum,
            ..SchedulerConfig::default()
        };
        let mut sched = Scheduler::new(forest, config);
        sched.add_runnable(domain, Box::new(Cpu::default()));
        for k in 1..=4u128 {
            let report = sched.run_quantum().unwrap();
            assert_eq!(report.to, GlobalTime::from_raw(k << 44), "round {k}");
        }
    }

    #[test]
    fn an_idle_machine_does_not_spin_and_lands_exactly() {
        let mut forest = ClockForest::new();
        let root = forest
            .add_oscillator("xtal", Rational::integer(1_000))
            .unwrap();
        let _ = forest.add_domain("d", root, 1, 1).unwrap();
        let mut sched = Scheduler::new(forest, SchedulerConfig::default());
        sched.run_until(t(5_000_000_000)).unwrap();
        assert_eq!(sched.now(), t(5_000_000_000));
    }

    // -- snapshots ----------------------------------------------------------

    #[test]
    fn a_queue_round_trips_through_enumeration_and_restore() {
        let mut q = EventQueue::default();
        let mut rng = 0xfeed_face_u64;
        for i in 0..300u64 {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            // Every level of the wheel, the far heap, and a pile of ties.
            let when = t(((rng >> 41) % 2_000_000_000) / 1_000 * 1_000);
            q.schedule(when, EventTarget((i % 5) as u32), i);
        }
        let cancelled = q.schedule(t(10), EventTarget(9), 999);
        q.cancel(cancelled);
        // Part way through, so the wheel has cascaded and the layout is no
        // longer the one insertion produced.
        q.advance_to(t(400_000_000));

        let events = q.events();
        let next_seq = q.next_seq();
        assert!(
            events.iter().all(|e| e.token != 999),
            "a cancelled event is not state"
        );
        assert!(events.windows(2).all(|w| w[0] < w[1]), "in fire order");

        let mut restored = EventQueue::new(DEFAULT_GRANULE_SHIFT);
        restored.restore(q.now(), next_seq, &events).unwrap();
        assert_eq!(restored.now(), q.now());
        assert_eq!(restored.next_seq(), next_seq);
        assert_eq!(restored.next_deadline(), q.next_deadline());

        // The claim that matters: what is left fires identically.
        let a = drain(&mut q, t(4_000_000_000));
        let b = drain(&mut restored, t(4_000_000_000));
        assert!(!a.is_empty());
        assert_eq!(a, b);
    }

    #[test]
    fn an_event_already_due_survives_a_restore_and_still_fires() {
        // Restoring must not quietly drop what a save caught in flight.
        let mut q = EventQueue::default();
        q.advance_to(t(1_000));
        q.schedule(t(10), EventTarget(0), 7);
        let events = q.events();
        let mut restored = EventQueue::default();
        restored.restore(t(1_000), q.next_seq(), &events).unwrap();
        assert_eq!(drain(&mut restored, t(1_000)), alloc::vec![(7, 0)]);
    }

    #[test]
    fn an_inconsistent_event_set_is_refused_rather_than_loaded() {
        let mut q = EventQueue::default();
        let event = |seq: u64| Event {
            time: t(100),
            id: EventId::from_seq(seq),
            target: EventTarget(0),
            token: seq,
        };
        // A sequence number the counter has not reached: the next event posted
        // would collide with it and the two would tie on identity.
        assert_eq!(
            q.restore(t(0), 3, &[event(3)]).unwrap_err(),
            SchedError::InvalidSnapshot(
                "an event's sequence number is not below the next sequence number"
            )
        );
        assert_eq!(
            q.restore(t(0), 9, &[event(1), event(1)]).unwrap_err(),
            SchedError::InvalidSnapshot("two events share a sequence number")
        );
    }

    #[test]
    fn a_saved_scheduler_fires_the_same_events_at_the_same_instants() {
        let mut saved = nes_scheduler().0;
        let (_, cpu, ppu) = nes_scheduler();
        saved.add_runnable(cpu, Box::new(Cpu::default()));
        for i in 0..40u64 {
            saved
                .schedule_after_ticks(ppu, 700 + i * 41, EventTarget(2), i)
                .unwrap();
        }

        // Run part way, so the queue is mid-flight rather than pristine.
        for _ in 0..6 {
            saved.run_quantum().unwrap();
        }
        let snapshot = saved.snapshot();
        assert!(!snapshot.events.is_empty(), "events still pending");

        // The layer above saves the clock forest separately — its tick counters
        // are the authoritative time state — so the restore starts from those
        // and adds the scheduler's own.
        let mut restored = Scheduler::new(saved.forest().clone(), SchedulerConfig::default());
        restored.add_runnable(cpu, Box::new(Cpu::default()));
        assert_eq!(restored.now(), GlobalTime::ZERO);
        restored.restore(&snapshot).unwrap();
        assert_eq!(restored.now(), saved.now());

        let history = |sched: &mut Scheduler| {
            let mut out = Vec::new();
            for _ in 0..40 {
                let report = sched.run_quantum().unwrap();
                for e in report.fired {
                    out.push((e.time.raw(), e.id.seq(), e.token));
                }
            }
            out
        };
        let a = history(&mut saved);
        let b = history(&mut restored);
        assert!(
            a.len() > 20,
            "the run must actually fire things: {}",
            a.len()
        );
        assert_eq!(a, b);
        assert_eq!(saved.now(), restored.now());
    }

    #[test]
    fn ties_still_break_by_sequence_after_a_restore() {
        let (mut sched, _cpu, _ppu) = nes_scheduler();
        sched.schedule_at(t(1_000), EventTarget(0), 10);
        sched.schedule_at(t(1_000), EventTarget(0), 11);
        let snapshot = sched.snapshot();

        let mut restored = Scheduler::new(sched.forest().clone(), SchedulerConfig::default());
        restored.restore(&snapshot).unwrap();
        // An event posted after the restore is *later* than both, and must lose
        // the tie to them. It only does if the sequence counter carried over.
        restored.schedule_at(t(1_000), EventTarget(0), 12);

        let report = restored.run_quantum().unwrap();
        let tokens: Vec<u64> = report.fired.iter().map(|e| e.token).collect();
        assert_eq!(tokens, alloc::vec![10, 11, 12]);
    }

    #[test]
    fn the_round_robin_resumes_where_it_stopped() {
        let forest = || {
            let mut f = ClockForest::new();
            let root = f
                .add_oscillator("xtal", Rational::integer(1_000_000))
                .unwrap();
            let a = f.add_domain("a", root, 1, 1).unwrap();
            let b = f.add_domain("b", root, 1, 1).unwrap();
            let c = f.add_domain("c", root, 1, 1).unwrap();
            (f, a, b, c)
        };
        let (f, a, b, c) = forest();
        let mut sched = Scheduler::new(f, SchedulerConfig::default());
        for domain in [a, b, c] {
            sched.add_runnable(domain, Box::new(Cpu::default()));
        }
        sched.run_quantum().unwrap();
        let snapshot = sched.snapshot();
        assert_eq!(snapshot.cursor, 1);

        let mut restored = Scheduler::new(sched.forest().clone(), SchedulerConfig::default());
        for domain in [a, b, c] {
            restored.add_runnable(domain, Box::new(Cpu::default()));
        }
        restored.restore(&snapshot).unwrap();

        // Which runnable goes first is guest-visible the moment two of them
        // touch the same device, so it is state, not scheduling policy.
        let order = |sched: &mut Scheduler| {
            sched
                .run_quantum()
                .unwrap()
                .consumed
                .iter()
                .map(|(id, _)| id.index())
                .collect::<Vec<_>>()
        };
        let from_the_saved = order(&mut sched);
        let from_the_restored = order(&mut restored);
        assert_eq!(from_the_restored, alloc::vec![1, 2, 0]);
        assert_eq!(from_the_saved, from_the_restored);
    }

    #[test]
    fn a_snapshot_that_does_not_fit_this_machine_is_refused() {
        let (mut sched, cpu, _ppu) = nes_scheduler();
        sched.add_runnable(cpu, Box::new(Cpu::default()));
        let mut snapshot = sched.snapshot();
        snapshot.cursor = 4;
        assert_eq!(
            sched.restore(&snapshot).unwrap_err(),
            SchedError::InvalidSnapshot(
                "the round-robin cursor does not name a registered runnable"
            )
        );
        // And a machine with nothing to run has nowhere for a cursor to point.
        let (mut empty, _cpu, _ppu) = nes_scheduler();
        let mut snapshot = empty.snapshot();
        snapshot.cursor = 1;
        assert!(empty.restore(&snapshot).is_err());
    }

    // -- rate control -------------------------------------------------------

    /// The injected clock is what makes rate control testable at all: a test
    /// hands in a clock it controls, and the outcome stops being a race.
    #[derive(Debug)]
    struct FakeClock(u64);
    impl HostClock for FakeClock {
        fn monotonic_nanos(&self) -> u64 {
            self.0
        }
    }

    /// Both nanosecond conversions floor, so a pacing figure may land one unit
    /// low. Asserting to the nanosecond would be asserting a precision the
    /// fixed-point timeline does not claim.
    fn assert_wait(pace: Pace, nanos: u64) {
        match pace {
            Pace::Wait { nanos: got } => {
                assert!(got.abs_diff(nanos) <= 2, "expected ~{nanos} ns, got {got}");
            }
            Pace::Run => panic!("expected a wait of ~{nanos} ns, got Run"),
        }
    }

    #[test]
    fn unbounded_never_waits_and_needs_no_clock() {
        let (mut sched, _cpu, _ppu) = nes_scheduler();
        assert_eq!(sched.pace().unwrap(), Pace::Run);
    }

    #[test]
    fn realtime_throttling_is_integer_only() {
        let mut rc = RateController::new(RateControl::Realtime {
            max_catchup_nanos: 100_000_000,
        });
        rc.reset(0, GlobalTime::ZERO);
        // Virtual time has run a millisecond; the wall has not moved.
        assert_wait(rc.pace(0, t(1_000_000)), 1_000_000);
        // The wall catches up.
        assert_eq!(rc.pace(1_000_000, t(1_000_000)), Pace::Run);
        // The host stalls for a second: the debt is written off, not chased at
        // full speed, which is what keeps audio and input sane after a hitch.
        assert_eq!(rc.pace(1_001_000_000, t(1_000_000)), Pace::Run);
        assert_wait(rc.pace(1_001_000_000, t(1_100_000)), 100_000);
    }

    #[test]
    fn fixed_ratio_scales_the_allowance() {
        let mut rc = RateController::new(RateControl::FixedRatio { num: 1, den: 2 });
        rc.reset(0, GlobalTime::ZERO);
        // Half speed: after 1 ms of wall time, 500 µs of virtual time is due.
        assert_eq!(rc.pace(1_000_000, t(400_000)), Pace::Run);
        assert_wait(rc.pace(1_000_000, t(600_000)), 100_000);

        let mut rc = RateController::new(RateControl::FixedRatio { num: 2, den: 1 });
        rc.reset(0, GlobalTime::ZERO);
        assert_eq!(rc.pace(1_000_000, t(1_900_000)), Pace::Run);
        assert_wait(rc.pace(1_000_000, t(2_100_000)), 100_000);
    }

    #[test]
    fn rate_control_without_a_clock_is_refused() {
        let (mut sched, _cpu, _ppu) = nes_scheduler();
        sched.rate_controller_mut().set_control(
            RateControl::Realtime {
                max_catchup_nanos: 0,
            },
            0,
            GlobalTime::ZERO,
        );
        // Silently running unthrottled would be a rate control that does not
        // control the rate.
        assert_eq!(sched.pace().unwrap_err(), SchedError::NoHostClock);

        sched.set_host_clock(Box::new(FakeClock(0)));
        assert!(matches!(
            sched.pace().unwrap(),
            Pace::Run | Pace::Wait { .. }
        ));
    }

    #[test]
    fn the_whole_loop_is_reproducible_run_to_run() {
        // The regression suite's basic claim: identical inputs, identical
        // history, with no wall clock anywhere in the path.
        let history = || {
            let (mut sched, cpu, ppu) = nes_scheduler();
            sched.add_runnable(cpu, Box::new(Cpu::default()));
            let dev = sched.add_lazy_device(ppu, Box::new(Ppu::default()));
            for i in 0..40u64 {
                sched
                    .schedule_after_ticks(ppu, 700 + i * 13, EventTarget(2), i)
                    .unwrap();
            }
            let mut out: Vec<(u128, u64)> = Vec::new();
            for _ in 0..50 {
                let report = sched.run_quantum().unwrap();
                out.push((report.to.raw(), report.consumed[0].1));
                for e in report.fired {
                    out.push((e.time.raw(), e.token));
                }
                let at = sched.sync_for_access(dev, AccessKind::Guest).unwrap();
                out.push((0, at));
            }
            out
        };
        let a = history();
        assert!(a.len() > 100);
        assert_eq!(a, history());
    }
}
