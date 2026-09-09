//! Structured tracing and profiling output: what a run *did*, in numbers a
//! script can read (`ROADMAP.md` §4's `trace.rs`, phase 9's
//! "tracing/profiling output").
//!
//! # Why this exists, and why it is counters rather than an event stream
//!
//! The tree has needed this repeatedly and improvised it every time. Three
//! recent examples, each a private patch that was reverted afterwards:
//!
//! * `Host::new` instrumented by hand to print `(pc, cycles, edge, pending)`
//!   every round, to find the one line in two guest seconds where `pending`
//!   was true;
//! * block entries, chained entries, translations and invalidations counted by
//!   hand to prove a workload really stressed the seams it claimed —
//!   `docs/testing/long-run.md` records "982 618 of 1 220 450 block entries in
//!   a guest second are chained" and "106 058 translations thrown away against
//!   106 089 made" as *prose*, because there was no way to print them;
//! * per-function host-instruction counts, reached for through callgrind,
//!   which cannot see guest-level structure at all.
//!
//! Every one of those is a **count**. Not one of them is a stream. That is the
//! whole design: an event record per block entry is hundreds of millions of
//! records for a single run, which is a cost no hot path can carry and an
//! output nobody reads; a *total* answers the same question in eight bytes.
//! Where a distribution rather than a total is wanted — how long a round ran —
//! this keeps a power-of-two histogram, which is still counters.
//!
//! So the shape here is: a fixed array of [`Counter`]s grouped into
//! [`Channel`]s, a [`Table`] they are folded into, and a plain two-column text
//! rendering of that table. No event log, no ring buffer, no timestamps.
//!
//! # Determinism
//!
//! **A trace must not change what the guest does or when.** Three rules keep
//! that true, and they are structural rather than aspirational:
//!
//! 1. Nothing here reads a clock. A counter that wants guest time is *handed*
//!    the span the caller already had — see [`quantum`] — so the trace never
//!    introduces a time source, host or guest. `CLAUDE.md`'s "no wall-clock
//!    reads outside `host/`" is satisfied by there being nothing to read.
//! 2. Nothing here is readable by the guest. These counters are not device
//!    state, are in no snapshot chunk, and are not in
//!    `Machine::state_hash`.
//! 3. Nothing here allocates or locks on a path a guest can feel. A counter is
//!    a relaxed `fetch_add` on a static; the [`Table`] that allocates is built
//!    once, after the run.
//!
//! `tests/cli_trace.rs` asserts the consequence directly: the same workload
//! with tracing on and off reaches the identical state hash, and the trace of a
//! deterministic run is itself byte-identical between runs.
//!
//! # Cost
//!
//! Without the `trace` feature every function here has an empty body and the
//! call sites vanish, so a default build carries **zero** instructions for this
//! module — which is what makes it safe to put a hook on a hot path at all.
//! With the feature compiled in, a disabled channel costs one relaxed load of a
//! static and a not-taken branch; an enabled one costs that plus a relaxed
//! `fetch_add`. `docs/testing/tracing.md` has the measured numbers.
//!
//! # The counters are process-global
//!
//! Deliberately, and it is the one property of this module that surprises. A
//! hook belongs wherever the event happens — inside `jit::Dispatcher::run`,
//! inside a lifted block's store path — and in none of those places is there a
//! `Machine` in hand to attribute the count to. A per-machine table would be a
//! table only the outermost hooks could reach, which is the opposite of what
//! this is for. The consequence is that a process running two machines gets
//! their sum; [`reset`] exists for a caller that wants them apart, and the
//! `rsemu` binary runs one machine per process anyway.
//!
//! Counters collected *from* a machine rather than pushed into a static — the
//! per-CPU translation statistics, the per-domain tick totals — have no such
//! problem, and [`crate::host::trace`] gathers those straight off the
//! [`Machine`](crate::machine::Machine).

use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

#[cfg(feature = "trace")]
use crate::core::sync::{AtomicU32, AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Channels
// ---------------------------------------------------------------------------

/// A family of related counters, and the unit `--trace` names.
///
/// A `#[repr(transparent)]` newtype with `pub const` variants rather than an
/// enum, per `CLAUDE.md`'s extensible-enumeration rule: channels are added as
/// subsystems grow hooks, and a downstream `match` must not break when one is.
///
/// The numeric value is a **bit index**, not an ordinal: [`enable`] sets bit
/// `Channel::0` of a mask, so there are thirty-two channels available.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Channel(pub u16);

impl Channel {
    /// The scheduler: how many rounds a run took, how long they were, and what
    /// came due in them.
    ///
    /// One of the two channels with a *live* hook — [`Channel::MMIO`] is the
    /// other, and everything else is read off the machine once the run has
    /// ended.
    pub const SCHED: Channel = Channel(0);

    /// Per-processor execution statistics: blocks entered, how many by
    /// following a patched exit, translations made and thrown away, and guest
    /// instructions retired inside a block against interpreted one at a time.
    ///
    /// Collected from each core's own `jit_stats`, which already existed and
    /// had no way out of the process — `docs/platforms/pc64.md` and
    /// `docs/platforms/arm64-virt.md` quote these numbers, read by hand.
    pub const CPU: Channel = Channel(1);

    /// Per-clock-domain tick totals: how many cycles of its own oscillator each
    /// clocked device was advanced.
    pub const CLOCK: Channel = Channel(2);

    /// Per-region MMIO accesses: how many reads and how many writes each
    /// device aperture answered.
    ///
    /// The **second** channel with a live hook, and the only one on a path a
    /// guest access takes — three lines in `core::space::flat`, one per
    /// dispatch arm that ends in a `MemOps` call. It costs the RAM path nothing
    /// at all, because the RAM path is a different arm of the same `match`;
    /// `docs/testing/tracing.md` has the callgrind and wall-clock measurements
    /// that decided it was safe to place.
    pub const MMIO: Channel = Channel(3);

    /// Every channel this build knows, in the order `--trace all` reports them.
    pub const ALL: &'static [Channel] =
        &[Channel::SCHED, Channel::CPU, Channel::CLOCK, Channel::MMIO];

    /// The name `--trace` spells this channel with.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Channel::SCHED => "sched",
            Channel::CPU => "cpu",
            Channel::CLOCK => "clock",
            Channel::MMIO => "mmio",
            _ => "unknown",
        }
    }

    /// The channel `--trace` named, if this build has one by that name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Channel> {
        Channel::ALL.iter().copied().find(|c| c.name() == name)
    }

    /// One line saying what this channel reports, for `--help` and for the
    /// error a misspelling gets.
    #[must_use]
    pub fn summary(self) -> &'static str {
        match self {
            Channel::SCHED => "scheduler rounds: how many, how long, what came due in them",
            Channel::CPU => "per-processor: blocks, chaining, translations, retired vs interpreted",
            Channel::CLOCK => "per-clock-domain tick totals",
            Channel::MMIO => "per-region MMIO: reads and writes each device aperture answered",
            _ => "",
        }
    }
}

// ---------------------------------------------------------------------------
// Counters
// ---------------------------------------------------------------------------

/// One counter slot, by index into this module's static array.
///
/// Same newtype rule as [`Channel`], and for the same reason: the set grows
/// with the hooks.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Counter(pub u16);

/// How many counter slots the static array has.
///
/// Fixed, and checked by a test against [`Counter::NAMES`], because a map
/// lookup on a hot path would be exactly the cost this module exists to avoid.
/// Raising it is nearly free: the array is zero-initialised static data.
pub const SLOTS: usize = 48;

/// How many buckets a power-of-two histogram keeps.
///
/// A `u64` has sixty-four bit positions, but the quantities here are
/// nanoseconds of a round and counts of ticks, both far below `2^32`; thirty-two
/// buckets covers four billion of anything, and a value above the last bucket
/// is counted *in* it rather than dropped.
pub const BUCKETS: u16 = 32;

impl Counter {
    /// Scheduler rounds run.
    pub const QUANTA: Counter = Counter(0);

    /// Rounds in which no runnable was given a budget: a boundary `run_until`
    /// declined because the deadline fell inside the round, or a machine with
    /// nothing runnable at all.
    ///
    /// The two cannot be told apart from a `QuantumReport` alone — the
    /// scheduler has no "why this round ended" type yet, and
    /// `docs/testing/tracing.md` specifies the one it should grow. Named for
    /// what is observable rather than for what is wanted.
    pub const QUANTA_IDLE: Counter = Counter(1);

    /// Rounds that advanced virtual time by nothing.
    pub const QUANTA_EMPTY: Counter = Counter(2);

    /// Events dispatched out of scheduler rounds.
    pub const EVENTS: Counter = Counter(3);

    /// Runnable budgets issued — one per runnable per round, so
    /// `budgets / quanta` is the machine's runnable count.
    pub const BUDGETS: Counter = Counter(4);

    /// Ticks consumed by runnables, summed over every domain.
    ///
    /// A sum *across* clock domains, so it is a volume of work rather than a
    /// time: two domains at different frequencies each contribute their own
    /// ticks. `CLAUDE.md`'s ban on routing an intra-tree relationship through
    /// absolute time is why this is not converted to seconds here.
    pub const TICKS: Counter = Counter(5);

    /// Total virtual nanoseconds the traced rounds covered.
    pub const SPAN_NS: Counter = Counter(6);

    /// The base of a [`BUCKETS`]-wide power-of-two histogram of how long each
    /// round was, in nanoseconds.
    pub const SPAN_LOG2: Counter = Counter(16);

    /// Every named slot, in index order. `""` is a slot nothing uses yet.
    ///
    /// The rendering side reads this, so a counter with no name here is a
    /// counter that never reaches a file — which a test asserts cannot happen
    /// silently.
    pub const NAMES: [&'static str; SLOTS] = {
        let mut names = [""; SLOTS];
        names[0] = "sched.quanta";
        names[1] = "sched.quanta.idle";
        names[2] = "sched.quanta.empty";
        names[3] = "sched.events";
        names[4] = "sched.budgets";
        names[5] = "sched.ticks";
        names[6] = "sched.span-ns";
        // 8..16 is reserved for the *reason* each round ended — allowance
        // spent, timer edge, lazy deadline, declined boundary, exit flag —
        // which needs a type `core::sched` has not grown yet.
        // `docs/testing/tracing.md` specifies it, down to the slot arithmetic.
        // 16..48 is the histogram, rendered by `Table::collect` from the index
        // rather than by a constant string each: thirty-two names would say
        // what the index already says.
        names
    };
}

/// The enabled-channel mask. Bit `Channel::0` is that channel.
#[cfg(feature = "trace")]
static ENABLED: AtomicU32 = AtomicU32::new(0);

/// The counters themselves.
///
/// `Relaxed` throughout: a counter is not a synchronisation edge and must never
/// become one. Under `parallel` threading two runnables may add to one slot
/// from two host threads and the total is still exact — `fetch_add` is atomic —
/// while the interleaving is not observable, because the output is a total.
#[cfg(feature = "trace")]
#[allow(clippy::declare_interior_mutable_const)]
static COUNTERS: [AtomicU64; SLOTS] = {
    const ZERO: AtomicU64 = AtomicU64::new(0);
    [ZERO; SLOTS]
};

/// Whether `ch` is being traced.
///
/// The gate every hook is written against. Without the `trace` feature it
/// folds to `false` at compile time, so the hook and everything it guards is
/// deleted.
#[must_use]
#[inline]
pub fn on(ch: Channel) -> bool {
    #[cfg(feature = "trace")]
    {
        ENABLED.load(Ordering::Relaxed) & (1u32 << (ch.0 & 31)) != 0
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = ch;
        false
    }
}

/// Whether anything at all is being traced.
#[must_use]
#[inline]
pub fn any() -> bool {
    #[cfg(feature = "trace")]
    {
        ENABLED.load(Ordering::Relaxed) != 0
    }
    #[cfg(not(feature = "trace"))]
    {
        false
    }
}

/// Start counting `ch`.
///
/// A build without the `trace` feature counts nothing and says nothing; the
/// layer that has to *refuse* rather than ignore is the command line, and
/// [`crate::host::trace`] is where that refusal lives.
pub fn enable(ch: Channel) {
    #[cfg(feature = "trace")]
    {
        ENABLED.fetch_or(1u32 << (ch.0 & 31), Ordering::Relaxed);
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = ch;
    }
}

/// Stop counting everything and zero every counter.
///
/// For a process that runs more than one machine and wants them apart. Not
/// atomic as a whole — a counter another thread increments while this runs may
/// survive — which is honest for a facility whose entire cost model is
/// "relaxed and unsynchronised".
///
/// The MMIO *names* are deliberately kept: a flat view built before this call
/// still holds the ids they were interned under, and forgetting them would
/// leave those leaves counting into rows nothing could name.
pub fn reset() {
    #[cfg(feature = "trace")]
    {
        ENABLED.store(0, Ordering::Relaxed);
        for slot in &COUNTERS {
            slot.store(0, Ordering::Relaxed);
        }
        for slot in &MMIO_COUNTERS {
            slot.store(0, Ordering::Relaxed);
        }
    }
}

/// Add `n` to a counter, if its channel is on.
///
/// The channel is a parameter rather than a property of the counter so that a
/// hook can test once and then add several counters: one load instead of five.
#[inline]
pub fn add(ch: Channel, c: Counter, n: u64) {
    #[cfg(feature = "trace")]
    {
        if on(ch) {
            raw(c, n);
        }
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (ch, c, n);
    }
}

/// Add to a counter whose channel the caller has already tested.
#[inline]
pub fn raw(c: Counter, n: u64) {
    #[cfg(feature = "trace")]
    {
        if let Some(slot) = COUNTERS.get(c.0 as usize) {
            slot.fetch_add(n, Ordering::Relaxed);
        }
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (c, n);
    }
}

/// Count `value` into the power-of-two histogram based at `base`.
///
/// Bucket *k* holds the values needing *k* bits, so bucket 0 is zero, bucket 1
/// is one, bucket 11 is `1024..2047`, and a bucket's label is a magnitude a
/// reader can convert in their head. Values at or above `2^BUCKETS` land in the
/// last bucket rather than being dropped: an out-of-range measurement is
/// exactly the one worth seeing.
#[inline]
pub fn hist(ch: Channel, base: Counter, value: u64) {
    #[cfg(feature = "trace")]
    {
        if on(ch) {
            let bits = u64::BITS - value.leading_zeros();
            let bucket = u16::try_from(bits).unwrap_or(BUCKETS).min(BUCKETS - 1);
            raw(Counter(base.0 + bucket), 1);
        }
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (ch, base, value);
    }
}

/// Read a counter, for a test that wants to assert one.
#[must_use]
pub fn get(c: Counter) -> u64 {
    #[cfg(feature = "trace")]
    {
        COUNTERS
            .get(c.0 as usize)
            .map_or(0, |s| s.load(Ordering::Relaxed))
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = c;
        0
    }
}

// ---------------------------------------------------------------------------
// Per-region MMIO
// ---------------------------------------------------------------------------

/// How many MMIO regions the [`Channel::MMIO`] counters cover.
///
/// A second array rather than more slots in the named-counter one: this is
/// indexed by a *region*, so its size is a property of the machine rather than
/// of the counter set, and a board with a hundred apertures would otherwise
/// push [`SLOTS`] past anything [`Counter::NAMES`] can describe. Two counters
/// per region — reads and writes — so this is `2 × 256` of zero-initialised
/// static data, four kilobytes that a build without the `trace` feature does
/// not have at all.
///
/// A process with more MMIO regions than this counts its first `MMIO_REGIONS`
/// and reports the rest as one overflow header line rather than silently
/// folding them into region zero; [`crate::host::trace`] is what writes that
/// line. The largest board in the tree interns a few dozen.
pub const MMIO_REGIONS: usize = 256;

/// The per-region MMIO counters, indexed `region * 2 + write`.
#[cfg(feature = "trace")]
#[allow(clippy::declare_interior_mutable_const)]
static MMIO_COUNTERS: [AtomicU64; MMIO_REGIONS * 2] = {
    const ZERO: AtomicU64 = AtomicU64::new(0);
    [ZERO; MMIO_REGIONS * 2]
};

/// The names behind the ids, and the identity each id was handed out for.
///
/// [`Global`](crate::core::sync::Global) rather than a `Mutex`, per
/// `core::sync`'s rule for a table that lives in a `static`, at
/// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF): it is taken at
/// *flatten* time, underneath the topology write that is rebuilding the view,
/// and never on an access path.
#[cfg(feature = "trace")]
static MMIO_NAMES: crate::core::sync::Global<MmioNames> =
    crate::core::sync::Global::new(MmioNames {
        names: Vec::new(),
        by_key: BTreeMap::new(),
        dropped: 0,
    });

/// The interning table [`mmio_intern`] keeps.
#[cfg(feature = "trace")]
#[derive(Debug)]
struct MmioNames {
    names: Vec<String>,
    by_key: BTreeMap<usize, u16>,
    dropped: u32,
}

/// Give an MMIO aperture a dense id, or return the one it already has.
///
/// `key` is an identity token the caller owns — `core::space::flat` passes the
/// address of the `MemOps` the aperture dispatches to — and `name` is what the
/// region is called. The identity is
/// **not** the mapping: one device mapped into two address spaces, or
/// re-flattened after a BAR moved, keeps its id and therefore its running
/// total. That is the property a per-view index could not have, and it is why
/// this table is here rather than in the flat view: a view is derived state
/// that a retopology throws away, and a count that a `mov` to a BAR resets is
/// worse than no count.
///
/// Returns [`u16::MAX`] once the table is full, which is out of range of the
/// counter array, so an aperture past the cap is uncounted rather than
/// misattributed. Costs a lock and a map lookup, once per aperture per
/// flatten, and nothing at all without the `trace` feature — in which case it
/// interns nothing and every aperture gets `u16::MAX`.
pub fn mmio_intern(key: usize, name: &str) -> u16 {
    #[cfg(feature = "trace")]
    {
        let mut table = MMIO_NAMES.lock();
        if let Some(id) = table.by_key.get(&key) {
            return *id;
        }
        let Ok(id) = u16::try_from(table.names.len()) else {
            table.dropped = table.dropped.saturating_add(1);
            return u16::MAX;
        };
        if usize::from(id) >= MMIO_REGIONS {
            table.dropped = table.dropped.saturating_add(1);
            return u16::MAX;
        }
        // Region names are *class* names much of the time — two PL011s are
        // both `arm.pl011` — so a repeat is suffixed rather than allowed to
        // collide, which would merge two devices into one row of the trace.
        let mut unique = String::from(name);
        let mut seq = 1u32;
        while table.names.contains(&unique) {
            unique = alloc::format!("{name}#{seq}");
            seq += 1;
        }
        table.names.push(unique);
        table.by_key.insert(key, id);
        id
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (key, name);
        u16::MAX
    }
}

/// Every interned aperture's name, by id, plus how many did not fit.
///
/// A snapshot the collector renders from; empty without the `trace` feature.
#[must_use]
pub fn mmio_regions() -> (Vec<String>, u32) {
    #[cfg(feature = "trace")]
    {
        let table = MMIO_NAMES.lock();
        (table.names.clone(), table.dropped)
    }
    #[cfg(not(feature = "trace"))]
    {
        (Vec::new(), 0)
    }
}

/// One MMIO access to region `id` happened; `write` says which direction.
///
/// **The hook on a per-access path**, and the only one in this module: three
/// call sites, all of them the `FlatTarget::Io` arm of a `match` whose other
/// arms are RAM and ROM. Those arms are what a guest's ordinary loads and
/// stores take, and they are untouched — which is the measurement
/// `docs/testing/tracing.md` records, and the reason this was safe to place at
/// all.
///
/// The identity is the dense index [`mmio_intern`] handed out when the view was
/// flattened — `core::space::RegionId` — so this is an array subscript and not
/// a lookup: a name would need a map, and a map on a dispatch path is exactly
/// the cost this module exists to avoid.
#[inline]
pub fn mmio(id: u16, write: bool) {
    #[cfg(feature = "trace")]
    {
        if on(Channel::MMIO)
            && let Some(slot) = MMIO_COUNTERS.get(usize::from(id) * 2 + usize::from(write))
        {
            slot.fetch_add(1, Ordering::Relaxed);
        }
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (id, write);
    }
}

/// Read one per-region MMIO counter, for the collector and for a test.
///
/// Zero for a region index this build has no slot for, which is the same
/// answer a region nothing touched gives — [`mmio_regions`] is what tells the
/// two apart, because it knows how many apertures were interned.
#[must_use]
pub fn mmio_get(id: u16, write: bool) -> u64 {
    #[cfg(feature = "trace")]
    {
        MMIO_COUNTERS
            .get(usize::from(id) * 2 + usize::from(write))
            .map_or(0, |s| s.load(Ordering::Relaxed))
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (id, write);
        0
    }
}

// ---------------------------------------------------------------------------
// The one live hook
// ---------------------------------------------------------------------------

/// One scheduler round happened: it covered `span_ns` of virtual time, issued
/// `budgets` runnable budgets that consumed `ticks` between them, and
/// dispatched `events`.
///
/// One of the **two** hooks this module asks another file to place, and it is
/// one line at the end of `Machine::advance_to`; [`mmio`] is the other. Nothing
/// else a trace reports needs a hook at all — it is read off the machine when
/// the run ends, which is why this subsystem costs the block-entry path nothing
/// whatever.
///
/// It takes the span rather than reading a clock, which is what makes the
/// determinism rule structural: there is no time source in here to perturb.
#[inline]
pub fn quantum(span_ns: u64, budgets: u64, ticks: u64, events: u64) {
    #[cfg(feature = "trace")]
    {
        if on(Channel::SCHED) {
            raw(Counter::QUANTA, 1);
            raw(Counter::SPAN_NS, span_ns);
            raw(Counter::EVENTS, events);
            raw(Counter::BUDGETS, budgets);
            raw(Counter::TICKS, ticks);
            if budgets == 0 {
                raw(Counter::QUANTA_IDLE, 1);
            }
            if span_ns == 0 {
                raw(Counter::QUANTA_EMPTY, 1);
            }
            hist(Channel::SCHED, Counter::SPAN_LOG2, span_ns);
        }
    }
    #[cfg(not(feature = "trace"))]
    {
        let _ = (span_ns, budgets, ticks, events);
    }
}

/// One scheduler round happened, as the scheduler itself described it.
///
/// **This is the whole of the scheduler hook**, and the one line another file
/// is asked to add:
/// `crate::core::trace::quantum_report(&report);` immediately after a round's
/// events have been dispatched. It is written against
/// [`QuantumReport`](crate::core::sched::QuantumReport) rather than against
/// four numbers so that the call site is a line rather than a paragraph, and it
/// returns before touching the report at all when the channel is off — so a
/// build with the feature compiled in and `sched` disabled pays one relaxed
/// load, and a build without the feature pays nothing whatever.
#[inline]
pub fn quantum_report(report: &crate::core::sched::QuantumReport) {
    if !on(Channel::SCHED) {
        return;
    }
    let span = report.to.as_nanos().saturating_sub(report.from.as_nanos());
    let ticks = report.consumed.iter().map(|(_, used)| *used).sum();
    quantum(
        span,
        report.consumed.len() as u64,
        ticks,
        report.fired.len() as u64,
    );
}

// ---------------------------------------------------------------------------
// The table and its rendering
// ---------------------------------------------------------------------------

/// A trace, as name-to-number rows plus a header of things that are not
/// numbers.
///
/// A `BTreeMap` rather than a hash map, per `CLAUDE.md`'s determinism rule: the
/// rendered order is the sorted order, so two runs of one workload produce
/// byte-identical files and `diff` says something useful.
#[derive(Debug, Default, Clone)]
pub struct Table {
    rows: BTreeMap<String, u64>,
    notes: Vec<(String, String)>,
}

impl Table {
    /// An empty table.
    #[must_use]
    pub fn new() -> Table {
        Table::default()
    }

    /// Set a row, replacing whatever was there.
    pub fn set(&mut self, name: &str, value: u64) {
        self.rows.insert(name.to_string(), value);
    }

    /// Add to a row, creating it at zero first.
    pub fn add(&mut self, name: &str, value: u64) {
        *self.rows.entry(name.to_string()).or_insert(0) += value;
    }

    /// Record a header line: something about the run that is not a count.
    ///
    /// Order is insertion order, because a header is read top to bottom by a
    /// person rather than looked up by a script.
    pub fn note(&mut self, key: &str, value: &str) {
        self.notes.push((key.to_string(), value.to_string()));
    }

    /// Read a row back, for a test.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<u64> {
        self.rows.get(name).copied()
    }

    /// How many rows there are.
    #[must_use]
    pub fn len(&self) -> usize {
        self.rows.len()
    }

    /// Whether the table has no rows.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// Every row, in sorted order.
    ///
    /// So that a collector can total up rows it did not write — which is how
    /// [`crate::host::trace`] sums per-core counters into machine-wide ones
    /// without a second accumulator that could disagree with the table.
    pub fn rows(&self) -> impl Iterator<Item = (&str, u64)> + '_ {
        self.rows
            .iter()
            .map(|(name, value)| (name.as_str(), *value))
    }

    /// Fold this build's live counters for `ch` into the table.
    ///
    /// A named counter is written even when it is zero, because "no declined
    /// boundaries" is an answer and a missing row is not. Histogram buckets are
    /// the exception: an empty bucket is noise, and thirty-two mostly-zero rows
    /// would bury the six that carry the distribution.
    pub fn collect(&mut self, ch: Channel) {
        if ch != Channel::SCHED {
            return;
        }
        for (index, name) in Counter::NAMES.iter().enumerate() {
            if !name.is_empty() {
                let value = get(Counter(u16::try_from(index).unwrap_or(0)));
                self.set(name, value);
            }
        }
        for bucket in 0..BUCKETS {
            let value = get(Counter(Counter::SPAN_LOG2.0 + bucket));
            if value != 0 {
                let mut name = String::from("sched.span-ns.log2.");
                // Zero-padded so the sorted order is the numeric order: without
                // it `log2.10` sorts before `log2.9` and the distribution reads
                // backwards in the middle.
                if bucket < 10 {
                    name.push('0');
                }
                name.push_str(&itoa(u64::from(bucket)));
                self.set(&name, value);
            }
        }
    }

    /// The whole table as text: a `#` header, then one `name value` row per
    /// line, sorted.
    ///
    /// The format is deliberately the dullest thing that works. A person reads
    /// it as a table; a script reads a row with
    /// `awk '$1=="sched.quanta"{print $2}'`; `diff` of two of them is the answer
    /// to "what did that change do". It carries no dependency, because the
    /// policy has room for none, and nothing about it needs a parser: there is
    /// no nesting, no quoting and no escaping, because every name is an
    /// identifier this crate chose and every value is a `u64`.
    ///
    /// There is no timestamp and no wall-clock figure anywhere in it, and that
    /// is a rule rather than an omission: **the trace of a deterministic run
    /// must itself be deterministic**, so that comparing two traces byte for
    /// byte is a real regression test. Host time is callgrind's and `perf`'s
    /// business; this file answers questions about the *guest's* structure.
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::new();
        out.push_str("# rsemu-trace 1\n");
        for (key, value) in &self.notes {
            out.push_str("# ");
            out.push_str(key);
            for _ in key.len()..16 {
                out.push(' ');
            }
            out.push_str(value);
            out.push('\n');
        }
        let width = self.rows.keys().map(String::len).max().unwrap_or(0).max(24);
        for (name, value) in &self.rows {
            out.push_str(name);
            for _ in name.len()..=width {
                out.push(' ');
            }
            // Right-aligned in a fixed field, so a column of numbers lines up
            // for a reader without `column -t`.
            let text = itoa(*value);
            for _ in text.len()..14 {
                out.push(' ');
            }
            out.push_str(&text);
            out.push('\n');
        }
        out
    }
}

/// `u64` as decimal, without `std`'s formatting machinery in the way.
///
/// `alloc::format!` would do it, and this module is `alloc`. It is written out
/// because the rendering path is also the one a `no_std` embedder calls to get
/// a trace out of a browser, and a formatter's monomorphised machinery is a
/// large thing to pull in for twenty digits.
fn itoa(mut value: u64) -> String {
    if value == 0 {
        return String::from("0");
    }
    let mut digits = [0u8; 20];
    let mut at = digits.len();
    while value != 0 {
        at -= 1;
        digits[at] = b'0' + u8::try_from(value % 10).unwrap_or(0);
        value /= 10;
    }
    // Every byte written above is an ASCII digit, so this cannot fail.
    String::from_utf8(digits[at..].to_vec()).unwrap_or_default()
}

#[cfg(test)]
mod tests;
