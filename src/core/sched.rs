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
//! * **Threading modes and rate control**, selected per machine. Only
//!   [`ThreadingMode::Deterministic`] is implemented here; the others are
//!   named, have their extension points marked, and return an error rather than
//!   pretending.
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
use crate::core::sync::Mutex;

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
    /// Under [`ThreadingMode::Deterministic`] — the only mode implemented — one
    /// thread runs everything, so this can only mean re-entrancy. A parallel
    /// mode would also reach it when two CPUs touch one device at the same
    /// instant, which wants that mode's rendezvous rather than a spin here.
    LazyDeviceBusy(LazyId),
    /// The threading mode is recognised but not implemented in this build.
    ModeUnimplemented(ThreadingMode),
    /// Rate control needs a host clock and none was injected.
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
            SchedError::ModeUnimplemented(m) => {
                write!(f, "threading mode `{m}` is not implemented in this build")
            }
            SchedError::NoHostClock => f.write_str("rate control needs an injected host clock"),
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

/// Something the scheduler gives execution budgets to: a CPU, a DMA engine, a
/// coprocessor.
///
/// `Send + Sync` from the first commit, because retrofitting it later is a
/// rewrite (`ROADMAP.md` §0, §4.7).
pub trait Runnable: Send + Sync {
    /// Runs until the budget is exhausted and reports what was consumed.
    ///
    /// Returning less than the budget is legitimate — a halt, a wait-for-
    /// interrupt, a natural block boundary. Returning more is a bug and the
    /// scheduler treats it as one.
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
    /// Fast, non-deterministic, the intended default for interactive use. Not
    /// implemented: it needs the `core::sync` task pool and barrier, which is a
    /// separate seam (`ROADMAP.md` §4.7).
    Parallel,
    /// CPUs run in hardware and virtual time is slaved to the host clock.
    ///
    /// The scheduler becomes a deadline service. Not implemented: it needs the
    /// acceleration backends (`ROADMAP.md` §10).
    Accel,
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
    /// Threading mode. Only [`ThreadingMode::Deterministic`] runs here.
    pub mode: ThreadingMode,
    /// Rate control policy.
    pub rate: RateControl,
    /// The span of virtual time one round of the round-robin covers.
    ///
    /// Shorter means finer interleaving between runnables and more scheduler
    /// overhead; it does not affect correctness, because catch-up makes every
    /// access exact regardless.
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
            rate: RateControl::Unbounded,
            // One millisecond: short enough that a machine feels responsive,
            // long enough that scheduling is not the bottleneck.
            quantum: GlobalTime::from_nanos(1_000_000),
            max_ticks_per_quantum: 10_000,
            granule_shift: DEFAULT_GRANULE_SHIFT,
        }
    }
}

struct RunnableSlot {
    domain: DomainId,
    inner: Option<Box<dyn Runnable>>,
}

impl fmt::Debug for RunnableSlot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RunnableSlot")
            .field("domain", &self.domain)
            .field("registered", &self.inner.is_some())
            .finish()
    }
}

/// What a lazy slot protects: the device, and where its domain has got to.
struct LazyState {
    device: Option<Box<dyn LazyDevice>>,
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
    domain: DomainId,
    state: Mutex<LazyState>,
}

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
        let (mut device, from, target) = {
            let mut state = self.state.lock();
            if let Some(p) = present {
                state.present = p;
            }
            let target = state.present;
            let device = state
                .device
                .as_ref()
                .ok_or(SchedError::LazyDeviceBusy(id))?;
            let from = device.current_tick();
            if kind == AccessKind::Debug {
                return Ok(from);
            }
            // Never past the device's own next event: beyond that tick its
            // behaviour changes, and simulating through it in one step would
            // compute the wrong answer.
            let target = target.min(device.next_event_tick().unwrap_or(u64::MAX));
            if target <= from {
                return Ok(from);
            }
            let device = state.device.take().expect("borrowed successfully above");
            (device, from, target)
        };
        device.advance_to(target);
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

    /// Puts the device on a specific tick of its own domain.
    fn sync_to_tick(&self, id: LazyId, tick: u64) -> SchedResult<u64> {
        let (mut device, from) = {
            let mut state = self.state.lock();
            let device = state
                .device
                .as_ref()
                .ok_or(SchedError::LazyDeviceBusy(id))?;
            let from = device.current_tick();
            if tick <= from {
                return Ok(from);
            }
            let device = state.device.take().expect("borrowed successfully above");
            (device, from)
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
    rate: RateController,
    host_clock: Option<Box<dyn HostClock>>,
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
        Scheduler {
            forest,
            queue,
            now: GlobalTime::ZERO,
            config,
            runnables: Vec::new(),
            lazy: Vec::new(),
            cursor: 0,
            rate,
            host_clock: None,
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
        });
        id
    }

    /// Registers a lazily-advanced device clocked by `domain`.
    pub fn add_lazy_device(&mut self, domain: DomainId, device: Box<dyn LazyDevice>) -> LazyId {
        let id = LazyId(self.lazy.len() as u32);
        // An unknown domain is reported by the first sync rather than here,
        // which keeps this call infallible; zero is the honest starting
        // position either way, since nothing has been simulated yet.
        let present = self.forest.ticks(domain).unwrap_or(0);
        self.lazy.push(Arc::new(LazySlot {
            domain,
            state: Mutex::new(LazyState {
                device: Some(device),
                present,
            }),
        }));
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
    /// [`SchedError::ModeUnimplemented`] for a mode this build does not
    /// implement, [`SchedError::BudgetExceeded`] if a runnable overran, or
    /// [`SchedError::Clock`].
    pub fn run_quantum(&mut self) -> SchedResult<QuantumReport> {
        self.run_quantum_until(GlobalTime::MAX)
    }

    /// Runs one quantum, but never past `limit`.
    ///
    /// # Errors
    ///
    /// As [`Scheduler::run_quantum`].
    pub fn run_quantum_until(&mut self, limit: GlobalTime) -> SchedResult<QuantumReport> {
        match self.config.mode {
            ThreadingMode::Deterministic => self.run_quantum_deterministic(limit),
            // Extension point: `parallel` submits one job per runnable to the
            // `core::sync` task pool and joins on a barrier at the quantum
            // boundary; `accel` replaces the target computation below with a
            // host-clock deadline and lets the hardware run. Both need seams
            // that do not exist yet, and guessing at them here would be worse
            // than saying so.
            mode => Err(SchedError::ModeUnimplemented(mode)),
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

    fn run_quantum_deterministic(&mut self, limit: GlobalTime) -> SchedResult<QuantumReport> {
        let from = self.now;
        let mut target = self.now.saturating_add(self.config.quantum).min(limit);
        if let Some(deadline) = self.queue.next_deadline()
            && deadline < target
        {
            // Never run past an event: a CPU that executes through its own NMI
            // is a CPU that has already got the answer wrong.
            target = deadline.max(self.now);
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
            let used = runnable.run(budget);
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
        if self.forest.is_gated(domain)? {
            return Ok(0);
        }
        let osc = self.forest.root_of(domain)?;
        let here = self.forest.unit_position(osc)?;
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

    #[test]
    fn parallel_and_accel_refuse_rather_than_pretend() {
        for mode in [ThreadingMode::Parallel, ThreadingMode::Accel] {
            let (mut sched, cpu, _ppu) = nes_scheduler();
            sched.config.mode = mode;
            sched.add_runnable(cpu, Box::new(Cpu::default()));
            assert_eq!(
                sched.run_quantum().unwrap_err(),
                SchedError::ModeUnimplemented(mode)
            );
        }
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
        sched.run_until(t(2_000)).unwrap();
        assert_eq!(sched.now(), t(2_000));
        assert!(sched.forest().ticks(cpu).unwrap() > 0);
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
