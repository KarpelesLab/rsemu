//! Wires: single-bit signal lines between devices (`ROADMAP.md` §4.3).
//!
//! A wire is the generic mechanism behind interrupt requests, GPIO pins, reset
//! lines, DMA request/acknowledge pairs and card-detect switches. The core has
//! no concept of an "interrupt": an i8259, an APIC, a GIC or the NES NMI line
//! are ordinary devices that happen to own [`WireSink`]s and [`WireSource`]s.
//!
//! # Why `set_level` carries the source
//!
//! ```text
//! apu.irq  ─┐
//!           ├─► cpu.irq     (wired-OR: the CPU sees IRQ while *either* asserts)
//! cart.irq ─┘
//! ```
//!
//! [`WireSink::set_level`] takes the asserting source's [`WireId`], and that
//! parameter is the whole design. Without it, a sink told "low" cannot know
//! whether the *other* driver is still asserting, so it drops a line that must
//! stay high — the classic shared-interrupt bug, and one that is unfixable
//! after the fact because the information was never passed. rsemu makes the
//! sink track which sources assert; [`FanIn`] is that bookkeeping, ready made,
//! and [`WireOr`] / [`WireAnd`] are it wrapped in a device so a machine
//! description can also make fan-in explicit.
//!
//! # Level and edge
//!
//! Wires carry levels. Edge semantics come from [`LevelToEdge`], a device that
//! *remembers the previous level* rather than a flag hidden inside a consumer,
//! so the remembered level lands in a snapshot like any other architectural
//! state (`ROADMAP.md` §4.5, invariant 6). It emits a transient pulse on its
//! output wire; [`EdgeLatch`] is the matching consumer.
//!
//! # Re-entrancy and cycles
//!
//! A sink notified of a level change may drive another wire from inside
//! `set_level` — an interrupt controller recomputing its output is exactly
//! that — so propagation is inherently re-entrant. Per CLAUDE.md's re-entrancy
//! contract, a [`Wire`] holds no lock across the outward call: its state is
//! atomic and is updated before delivery starts.
//!
//! Delivery is *iterative, not recursive*. A wire that is already delivering
//! records the new level, marks it pending and returns immediately; the
//! outermost delivery re-runs until the wire is quiescent. A cycle therefore
//! costs stack depth proportional to the length of the cycle rather than to the
//! number of times it goes round, and a genuine combinational loop (a
//! [`WireNot`] feeding its own input) terminates after [`Wire::SETTLE_LIMIT`]
//! passes with [`Wire::unsettled`] incremented, instead of overflowing the
//! stack. Such a loop is a machine-description error the resolver is expected
//! to reject; the runtime merely refuses to hang. Stack depth for an *acyclic*
//! graph is bounded by its depth, which is a property of the machine file.
//!
//! # Construction and ownership
//!
//! Sources and sinks are fixed when the wire is built, matching two-phase
//! device construction (`ROADMAP.md` §4.4): the resolver knows the whole graph,
//! so it knows every driver of every net. That is what lets the per-source
//! state be a plain array of atomics — no lock, no allocation while the machine
//! runs, and `Send + Sync` with no `unsafe` anywhere. The realize order is
//! therefore: construct devices, build the wires (naming each device as a
//! sink), then hand each device the [`WireSource`] it drives. Re-plugging a
//! wire (hot-plug) builds a new one and swaps it in.
//!
//! A device that both drives and listens — an interrupt controller with a
//! request line out and an acknowledge line in — closes an `Arc` cycle through
//! its wires, which would leak on teardown. [`WireBuilder::sink_weak`] is the
//! break: the machine owns the devices, the wire only refers to them.
//!
//! # Notes for later phases
//!
//! `core::device`, `core::state` and `core::sync` are still stubs, so the
//! combinators here are plain structs rather than `impl Device`, and snapshots
//! are exchanged as `Vec<(WireId, Level)>` rather than through a `StateWriter`.
//! Both are mechanical to retrofit; the state each object holds is already
//! separated from its diagnostics. Nothing here names `std::sync` — the only
//! primitives used are `core::sync::atomic`, which every target has
//! (invariant 4).

use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering::SeqCst};

/// The state of a signal line.
///
/// Deliberately not `bool`: half the lines in a real machine are active-low,
/// and `Level::Low` at a call site says which end of the wire is meant where
/// `false` would not.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// Deasserted for an active-high line, asserted for an active-low one.
    #[default]
    Low,
    /// Asserted for an active-high line, deasserted for an active-low one.
    High,
}

impl Level {
    /// `High` for `true`, `Low` for `false`.
    #[inline]
    pub const fn from_bool(b: bool) -> Level {
        if b { Level::High } else { Level::Low }
    }

    /// `true` for `High`.
    #[inline]
    pub const fn as_bool(self) -> bool {
        matches!(self, Level::High)
    }

    /// Whether this is [`Level::High`].
    #[inline]
    pub const fn is_high(self) -> bool {
        matches!(self, Level::High)
    }

    /// Whether this is [`Level::Low`].
    #[inline]
    pub const fn is_low(self) -> bool {
        matches!(self, Level::Low)
    }

    /// The opposite level.
    #[inline]
    pub const fn inverted(self) -> Level {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }
}

impl From<bool> for Level {
    #[inline]
    fn from(b: bool) -> Level {
        Level::from_bool(b)
    }
}

impl From<Level> for bool {
    #[inline]
    fn from(l: Level) -> bool {
        l.as_bool()
    }
}

/// A transition of a signal line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Edge {
    /// Low to high.
    Rising,
    /// High to low.
    Falling,
}

impl Edge {
    /// The edge described by a transition, or `None` if the level did not move.
    #[inline]
    pub const fn between(from: Level, to: Level) -> Option<Edge> {
        match (from, to) {
            (Level::Low, Level::High) => Some(Edge::Rising),
            (Level::High, Level::Low) => Some(Edge::Falling),
            _ => None,
        }
    }
}

/// Which edges a detector reacts to.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum EdgeTrigger {
    /// Low-to-high only. The common interrupt case.
    #[default]
    Rising,
    /// High-to-low only, as for an active-low line.
    Falling,
    /// Either transition.
    Both,
}

impl EdgeTrigger {
    /// Whether `edge` fires this trigger.
    #[inline]
    pub const fn matches(self, edge: Edge) -> bool {
        matches!(
            (self, edge),
            (EdgeTrigger::Both, _)
                | (EdgeTrigger::Rising, Edge::Rising)
                | (EdgeTrigger::Falling, Edge::Falling)
        )
    }
}

/// How several drivers of one line combine.
///
/// Named after the electrical arrangement it models: `Or` is the wired-OR of
/// active-high drivers, `And` the wired-AND of open-drain drivers sharing a
/// pull-up. With no drivers at all the line sits at [`Resolve::idle`] — low for
/// `Or`, high for `And`, which is the pull-up.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Resolve {
    /// The line is high while *any* source drives it high.
    #[default]
    Or,
    /// The line is high only while *every* source drives it high.
    And,
}

impl Resolve {
    /// The level of a line with no sources.
    #[inline]
    pub const fn idle(self) -> Level {
        match self {
            Resolve::Or => Level::Low,
            Resolve::And => Level::High,
        }
    }
}

/// The identity of a wire source.
///
/// Every driver of a line has one, and it travels with each level change so a
/// sink can tell its drivers apart. Ids are assigned by the machine resolver —
/// [`WireIdAllocator`] is the counter it uses — and are unique within a
/// machine, not globally.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WireId(
    /// The raw identifier. Public so a machine builder can assign ids from its
    /// own numbering and so ids are usable in `const` context.
    pub u64,
);

impl WireId {
    /// The reserved "no source" id, for a level with no meaningful origin: a
    /// test stimulus, a reset default.
    pub const NONE: WireId = WireId(0);

    /// An id with the given raw value.
    #[inline]
    pub const fn new(raw: u64) -> WireId {
        WireId(raw)
    }

    /// The raw value.
    #[inline]
    pub const fn raw(self) -> u64 {
        self.0
    }
}

/// A monotonic [`WireId`] counter.
///
/// One allocator per machine, never a global: ids must be reproducible for a
/// given machine description, and a process-wide counter would make them depend
/// on how many machines were built earlier (the determinism rule in CLAUDE.md).
#[derive(Debug)]
pub struct WireIdAllocator {
    next: AtomicUsize,
}

impl WireIdAllocator {
    /// A fresh allocator. The first id handed out is `WireId(1)`;
    /// [`WireId::NONE`] is never allocated.
    pub fn new() -> Self {
        WireIdAllocator {
            next: AtomicUsize::new(1),
        }
    }

    /// The next unused id.
    pub fn alloc(&self) -> WireId {
        WireId(self.next.fetch_add(1, SeqCst) as u64)
    }
}

impl Default for WireIdAllocator {
    fn default() -> Self {
        // Not derivable: a derived `Default` would start at zero and hand out
        // `WireId::NONE`.
        WireIdAllocator::new()
    }
}

/// Something that can be told a signal line changed.
///
/// `src` names the driver, `line` is the sink's own input pin number (chosen
/// when the sink was connected, so one device can host many inputs), and
/// `level` is that driver's new level — **not** the resolved level of the net.
/// A sink with more than one driver resolves them itself, normally by keeping a
/// [`FanIn`].
///
/// Implementations must be cheap and must not block: `set_level` runs inside
/// the caller's propagation. Driving another wire from within is allowed and
/// expected; see the module docs on re-entrancy.
pub trait WireSink: Send + Sync {
    /// Called when `src` changes the level it drives on this sink's `line`.
    fn set_level(&self, src: WireId, line: u32, level: Level);
}

/// Per-source level state: the bookkeeping that makes wired-OR possible.
///
/// A sink with several drivers keeps one of these, updates it from
/// [`WireSink::set_level`], and asks [`FanIn::resolve`] for the level of the
/// net. When the APU deasserts its IRQ while the cartridge still asserts,
/// `resolve(Resolve::Or)` still answers `High`.
///
/// The source list is fixed at construction, so updates are wait-free and
/// allocation-free, and the whole thing is `Send + Sync` with no lock.
#[derive(Debug)]
pub struct FanIn {
    /// Sorted and deduplicated, so lookup is a binary search and iteration
    /// order is deterministic.
    sources: Box<[WireId]>,
    levels: Box<[AtomicBool]>,
}

impl FanIn {
    /// Track these sources, all initially [`Level::Low`].
    ///
    /// Duplicates are collapsed; order does not matter.
    pub fn new(sources: &[WireId]) -> Self {
        let mut ids: Vec<WireId> = sources.to_vec();
        ids.sort_unstable();
        ids.dedup();
        let levels: Vec<AtomicBool> = ids.iter().map(|_| AtomicBool::new(false)).collect();
        FanIn {
            sources: ids.into_boxed_slice(),
            levels: levels.into_boxed_slice(),
        }
    }

    /// The tracked sources, in ascending id order.
    #[inline]
    pub fn sources(&self) -> &[WireId] {
        &self.sources
    }

    /// Whether `src` is one of the tracked sources.
    #[inline]
    pub fn contains(&self, src: WireId) -> bool {
        self.index_of(src).is_some()
    }

    #[inline]
    fn index_of(&self, src: WireId) -> Option<usize> {
        self.sources.binary_search(&src).ok()
    }

    #[inline]
    fn level_at(&self, i: usize) -> Level {
        Level::from_bool(self.levels[i].load(SeqCst))
    }

    #[inline]
    fn set_at(&self, i: usize, level: Level) -> bool {
        self.levels[i].swap(level.as_bool(), SeqCst) != level.as_bool()
    }

    /// Record `src`'s level.
    ///
    /// Returns whether that changed anything: `false` both for a repeat of the
    /// level already recorded and for a source this `FanIn` does not track. Use
    /// [`FanIn::contains`] when the two need telling apart — an untracked
    /// source means the machine graph was built wrong.
    #[inline]
    pub fn set(&self, src: WireId, level: Level) -> bool {
        match self.index_of(src) {
            Some(i) => self.set_at(i, level),
            None => false,
        }
    }

    /// The level last recorded for `src`, or `None` if it is not tracked.
    #[inline]
    pub fn level_of(&self, src: WireId) -> Option<Level> {
        self.index_of(src).map(|i| self.level_at(i))
    }

    /// Whether any source is currently high.
    pub fn any_high(&self) -> bool {
        self.levels.iter().any(|l| l.load(SeqCst))
    }

    /// Whether every source is currently high. Vacuously true with no sources.
    pub fn all_high(&self) -> bool {
        self.levels.iter().all(|l| l.load(SeqCst))
    }

    /// The level of the net under the given resolution.
    #[inline]
    pub fn resolve(&self, mode: Resolve) -> Level {
        if self.sources.is_empty() {
            return mode.idle();
        }
        match mode {
            Resolve::Or => Level::from_bool(self.any_high()),
            Resolve::And => Level::from_bool(self.all_high()),
        }
    }

    /// Drive every source low, as a reset does.
    ///
    /// Records only: nothing is propagated, because a reset propagates through
    /// the devices' own reset paths.
    pub fn clear(&self) {
        for l in self.levels.iter() {
            l.store(false, SeqCst);
        }
    }

    /// The architectural state: every source and its level, in id order.
    pub fn snapshot(&self) -> Vec<(WireId, Level)> {
        self.sources
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, self.level_at(i)))
            .collect()
    }

    /// Restore state produced by [`FanIn::snapshot`].
    ///
    /// Entries for sources this `FanIn` does not track are ignored: a snapshot
    /// taken from a differently shaped machine is diagnosed by the snapshot
    /// layer (`ROADMAP.md` §4.5), not here. Nothing is propagated — on load,
    /// every device restores its own state.
    pub fn restore(&self, state: &[(WireId, Level)]) {
        for (id, level) in state {
            if let Some(i) = self.index_of(*id) {
                self.levels[i].store(level.as_bool(), SeqCst);
            }
        }
    }
}

/// How a wire holds a connected sink.
enum SinkRef {
    Strong(Arc<dyn WireSink>),
    /// Non-owning, for breaking the `Arc` cycle a device that both drives and
    /// listens would otherwise create.
    Weak(Weak<dyn WireSink>),
}

impl SinkRef {
    /// Run `f` against the sink, or nothing if a weak sink has been dropped.
    #[inline]
    fn with(&self, f: impl FnOnce(&dyn WireSink)) {
        match self {
            SinkRef::Strong(s) => f(&**s),
            SinkRef::Weak(w) => {
                if let Some(s) = w.upgrade() {
                    f(&*s);
                }
            }
        }
    }
}

/// One connected sink and the input pin it was connected to.
struct SinkPort {
    sink: SinkRef,
    line: u32,
}

/// A net: per-source level state plus fan-out to every connected sink.
///
/// Sources drive it with [`Wire::set`], or through a [`WireSource`] handle, and
/// every change is delivered to every sink as `set_level(src, line, level)` —
/// the driver's level, unresolved, so the sink can do its own fan-in. The wire
/// keeps the per-source levels so a repeated `set` costs nothing, so
/// [`Wire::refresh`] can re-announce state after a snapshot load, and so the
/// levels themselves can be snapshotted.
///
/// Built through [`Wire::builder`]; sources and sinks are fixed thereafter.
pub struct Wire {
    inputs: FanIn,
    /// One flag per source, set when its level moved and has not been
    /// delivered. This is what makes a re-entrant `set` cost an iteration
    /// rather than a stack frame.
    pending: Box<[AtomicBool]>,
    sinks: Box<[SinkPort]>,
    /// Held by whichever call is currently delivering. Not a lock over data: it
    /// protects no state, so nothing is held across the outward call.
    delivering: AtomicBool,
    unsettled: AtomicUsize,
}

impl Wire {
    /// How many delivery passes one outermost [`Wire::set`] runs before
    /// declaring the graph unsettled.
    ///
    /// Reached only by a combinational loop, which is a machine-description
    /// error; a legitimate graph settles in one pass, or in as many passes as
    /// its feedback path needs to reach a stable state.
    pub const SETTLE_LIMIT: u32 = 64;

    /// Start building a wire.
    pub fn builder() -> WireBuilder {
        WireBuilder::new()
    }

    /// Drive `src`'s level, delivering the change to every sink.
    ///
    /// Returns whether the level moved. `false` means either that `src` already
    /// drove this level — nothing is delivered, so a device may call this every
    /// cycle — or that `src` is not a source of this wire, for which see
    /// [`FanIn::set`].
    ///
    /// Delivery has finished when this returns, *except* when the call is
    /// re-entrant (a sink driving the wire that is notifying it) or when
    /// another thread is mid-delivery; then the change is handed to that
    /// delivery, which picks it up before it finishes. Level semantics make
    /// that indistinguishable from outside: the wire converges on the levels
    /// last written.
    pub fn set(&self, src: WireId, level: Level) -> bool {
        let Some(i) = self.inputs.index_of(src) else {
            return false;
        };
        if !self.inputs.set_at(i, level) {
            return false;
        }
        self.pending[i].store(true, SeqCst);
        self.deliver();
        true
    }

    /// Re-announce every source's current level to every sink.
    ///
    /// Wanted after a snapshot load, and after a reset that changed levels
    /// without propagating. Delivery is idempotent, so this is always safe.
    pub fn refresh(&self) {
        for p in self.pending.iter() {
            p.store(true, SeqCst);
        }
        self.deliver();
    }

    /// Deliver pending changes until the wire is quiescent.
    ///
    /// The `delivering` flag is what keeps re-entrancy iterative: a nested or
    /// concurrent `set` has already recorded its level and marked it pending,
    /// so it can return and let this loop pick the change up.
    fn deliver(&self) {
        if self.delivering.swap(true, SeqCst) {
            // Someone else — possibly this very call stack — is delivering and
            // will observe what we just marked pending.
            return;
        }
        loop {
            let mut passes: u32 = 0;
            loop {
                let mut moved = false;
                for (i, src) in self.inputs.sources.iter().enumerate() {
                    // Take the flag first, then read the level: a racing writer
                    // that beats us to the level sets the flag again, so the
                    // worst case is one redundant, idempotent delivery.
                    if self.pending[i].swap(false, SeqCst) {
                        moved = true;
                        let level = self.inputs.level_at(i);
                        for port in self.sinks.iter() {
                            port.sink.with(|s| s.set_level(*src, port.line, level));
                        }
                    }
                }
                if !moved {
                    break;
                }
                passes += 1;
                if passes >= Self::SETTLE_LIMIT {
                    // A combinational loop. Stop rather than spin: the recorded
                    // levels stay correct, the sinks may be stale, and
                    // `unsettled()` says so. The next external change starts a
                    // fresh, equally bounded attempt.
                    self.unsettled.fetch_add(1, SeqCst);
                    for p in self.pending.iter() {
                        p.store(false, SeqCst);
                    }
                    self.delivering.store(false, SeqCst);
                    return;
                }
            }
            self.delivering.store(false, SeqCst);
            // A change made between the last scan and the release above would
            // have seen the flag set and returned, so re-check before leaving.
            if !self.pending.iter().any(|p| p.load(SeqCst)) {
                return;
            }
            if self.delivering.swap(true, SeqCst) {
                return;
            }
        }
    }

    /// The per-source levels, for a sink or a snapshot to inspect.
    #[inline]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The sources that may drive this wire, in ascending id order.
    #[inline]
    pub fn sources(&self) -> &[WireId] {
        self.inputs.sources()
    }

    /// The level `src` is driving, or `None` if it is not a source here.
    #[inline]
    pub fn level_of(&self, src: WireId) -> Option<Level> {
        self.inputs.level_of(src)
    }

    /// The level of the net under the given resolution.
    #[inline]
    pub fn resolve(&self, mode: Resolve) -> Level {
        self.inputs.resolve(mode)
    }

    /// How many sinks are connected.
    #[inline]
    pub fn sink_count(&self) -> usize {
        self.sinks.len()
    }

    /// How many deliveries have given up at [`Wire::SETTLE_LIMIT`].
    ///
    /// A diagnostic, not architectural state: it is not snapshotted, and a
    /// non-zero value means the machine description contains a combinational
    /// loop through this wire.
    pub fn unsettled(&self) -> usize {
        self.unsettled.load(SeqCst)
    }

    /// The architectural state: every source and its level.
    pub fn snapshot(&self) -> Vec<(WireId, Level)> {
        self.inputs.snapshot()
    }

    /// Restore state from [`Wire::snapshot`] without delivering anything.
    ///
    /// Call [`Wire::refresh`] afterwards if the sinks do not restore their own
    /// input state.
    pub fn restore(&self, state: &[(WireId, Level)]) {
        self.inputs.restore(state);
    }
}

impl fmt::Debug for Wire {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Manual: a sink is a trait object and is not `Debug`.
        f.debug_struct("Wire")
            .field("inputs", &self.inputs)
            .field("sinks", &self.sinks.len())
            .field("unsettled", &self.unsettled.load(SeqCst))
            .finish()
    }
}

/// Builder for a [`Wire`].
///
/// Every driver of the net is declared before it runs, which is what the
/// machine resolver knows and what lets the wire's state be lock-free.
#[derive(Default)]
pub struct WireBuilder {
    sources: Vec<WireId>,
    sinks: Vec<SinkPort>,
}

impl WireBuilder {
    /// An empty builder: no sources, no sinks.
    pub fn new() -> Self {
        WireBuilder::default()
    }

    /// Declare a driver of this net.
    #[must_use]
    pub fn source(mut self, src: WireId) -> Self {
        self.sources.push(src);
        self
    }

    /// Declare several drivers.
    #[must_use]
    pub fn sources(mut self, srcs: &[WireId]) -> Self {
        self.sources.extend_from_slice(srcs);
        self
    }

    /// Connect a sink, delivering to its input pin `line`.
    ///
    /// Sinks are notified in connection order, which keeps a machine's
    /// behaviour reproducible.
    #[must_use]
    pub fn sink(mut self, sink: Arc<dyn WireSink>, line: u32) -> Self {
        self.sinks.push(SinkPort {
            sink: SinkRef::Strong(sink),
            line,
        });
        self
    }

    /// Connect a sink without owning it.
    ///
    /// For the case the module docs describe: a device that drives a wire whose
    /// own sink chain leads back to it would otherwise form an `Arc` cycle and
    /// leak. The machine holds the strong reference; a sink dropped before the
    /// wire is simply skipped.
    #[must_use]
    pub fn sink_weak(mut self, sink: Weak<dyn WireSink>, line: u32) -> Self {
        self.sinks.push(SinkPort {
            sink: SinkRef::Weak(sink),
            line,
        });
        self
    }

    /// Finish. Every source starts [`Level::Low`] and nothing is delivered;
    /// devices announce their reset levels themselves.
    pub fn build(self) -> Wire {
        let inputs = FanIn::new(&self.sources);
        let pending: Vec<AtomicBool> = inputs
            .sources()
            .iter()
            .map(|_| AtomicBool::new(false))
            .collect();
        Wire {
            inputs,
            pending: pending.into_boxed_slice(),
            sinks: self.sinks.into_boxed_slice(),
            delivering: AtomicBool::new(false),
            unsettled: AtomicUsize::new(0),
        }
    }

    /// Finish, wrapped for sharing between the devices at both ends.
    pub fn build_shared(self) -> Arc<Wire> {
        Arc::new(self.build())
    }
}

impl fmt::Debug for WireBuilder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WireBuilder")
            .field("sources", &self.sources)
            .field("sinks", &self.sinks.len())
            .finish()
    }
}

/// An output port: a wire plus the id this device drives it with.
///
/// What a device holds in order to assert a line. Bundling the two means a
/// device never has to remember to pass its own id, which is the sort of
/// mistake that produces a line nobody can deassert.
#[derive(Debug, Clone)]
pub struct WireSource {
    wire: Arc<Wire>,
    id: WireId,
}

impl WireSource {
    /// Bind `id`, which must be one of the wire's declared sources, to `wire`.
    pub fn new(wire: Arc<Wire>, id: WireId) -> Self {
        WireSource { wire, id }
    }

    /// The id this port drives with.
    #[inline]
    pub fn id(&self) -> WireId {
        self.id
    }

    /// The wire this port drives.
    #[inline]
    pub fn wire(&self) -> &Arc<Wire> {
        &self.wire
    }

    /// Drive `level`. Returns whether it changed, as [`Wire::set`] does.
    #[inline]
    pub fn set(&self, level: Level) -> bool {
        self.wire.set(self.id, level)
    }

    /// Drive high.
    #[inline]
    pub fn raise(&self) -> bool {
        self.set(Level::High)
    }

    /// Drive low.
    #[inline]
    pub fn lower(&self) -> bool {
        self.set(Level::Low)
    }

    /// The level this port is currently driving.
    #[inline]
    pub fn level(&self) -> Level {
        self.wire.level_of(self.id).unwrap_or(Level::Low)
    }

    /// Drive `active`, then immediately return to the opposite level.
    ///
    /// The transient an edge-triggered input latches. Both transitions are
    /// delivered to every sink, so a sink that only tracks levels sees the line
    /// end where it started — which is the point: a pulse means something only
    /// to something that latches it, such as [`EdgeLatch`]. A pulse driven onto
    /// a wire inside a propagation cycle can be coalesced away, so do not put
    /// an edge output in one.
    pub fn pulse(&self, active: Level) {
        self.set(active);
        self.set(active.inverted());
    }
}

/// `wire.split`: one input, many outputs.
///
/// A [`Wire`] already fans out to many sinks, so a split is not needed to
/// deliver to several places; it exists because a machine description often
/// wants the fan-out to be a named node — to renumber lines, to insert a
/// combinator on one branch only, or simply so the graph reads clearly.
///
/// Several drivers on the input are resolved (wired-OR by default) before being
/// forwarded, so a split never re-exports the bug it is meant to be neutral
/// about.
#[derive(Debug)]
pub struct WireSplit {
    inputs: FanIn,
    mode: Resolve,
    outs: Box<[WireSource]>,
}

impl WireSplit {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.split";

    /// A split with wired-OR input resolution.
    pub fn new(sources: &[WireId], outs: Vec<WireSource>) -> Self {
        Self::with_resolve(sources, Resolve::Or, outs)
    }

    /// A split with an explicit input resolution.
    pub fn with_resolve(sources: &[WireId], mode: Resolve, outs: Vec<WireSource>) -> Self {
        WireSplit {
            inputs: FanIn::new(sources),
            mode,
            outs: outs.into_boxed_slice(),
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(self.mode)
    }

    /// Drive the outputs with the level the current inputs imply.
    ///
    /// A combinational device's output is a function of its inputs, but a wire
    /// only carries what has been driven onto it, and a freshly built or
    /// freshly loaded machine has driven nothing. Reset and snapshot-load call
    /// this — in topological order, so a chain of gates converges in one sweep
    /// — the same job [`Wire::refresh`] does for a net.
    pub fn announce(&self) {
        let out = self.level();
        for o in self.outs.iter() {
            o.set(out);
        }
    }
}

impl WireSink for WireSplit {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            let out = self.inputs.resolve(self.mode);
            for o in self.outs.iter() {
                o.set(out);
            }
        }
    }
}

/// `wire.or`: the explicit wired-OR combiner.
///
/// Its output is high while any input source is high. This is the device the
/// DSL's implicit fan-in expands into (`ROADMAP.md` §4.3, §5) — the same
/// resolution a [`FanIn`]-keeping sink performs, packaged so a machine file can
/// name it.
#[derive(Debug)]
pub struct WireOr {
    inputs: FanIn,
    out: WireSource,
}

impl WireOr {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.or";

    /// Combine `sources` onto `out`.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        WireOr {
            inputs: FanIn::new(sources),
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(Resolve::Or)
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`].
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireOr {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(Resolve::Or));
        }
    }
}

/// `wire.and`: the wired-AND combiner.
///
/// Output high only while every input is high — an open-drain bus with a
/// pull-up, and how an "everyone is ready" line is built. With no inputs it
/// reads high, matching the pull-up.
#[derive(Debug)]
pub struct WireAnd {
    inputs: FanIn,
    out: WireSource,
}

impl WireAnd {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.and";

    /// Combine `sources` onto `out`.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        WireAnd {
            inputs: FanIn::new(sources),
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(Resolve::And)
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`].
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireAnd {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(Resolve::And));
        }
    }
}

/// `wire.not`: inversion, for converting between active-high and active-low.
///
/// Inputs are resolved first (wired-OR by default, which makes a multi-input
/// `not` a NOR gate), then inverted.
#[derive(Debug)]
pub struct WireNot {
    inputs: FanIn,
    mode: Resolve,
    out: WireSource,
}

impl WireNot {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.not";

    /// An inverter with wired-OR input resolution.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        Self::with_resolve(sources, Resolve::Or, out)
    }

    /// An inverter with an explicit input resolution.
    pub fn with_resolve(sources: &[WireId], mode: Resolve, out: WireSource) -> Self {
        WireNot {
            inputs: FanIn::new(sources),
            mode,
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(self.mode).inverted()
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`]. An inverter especially needs this: its idle
    /// output is high, which is *not* where an undriven wire sits.
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireNot {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(self.mode).inverted());
        }
    }
}

/// The snapshottable state of a [`LevelToEdge`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EdgeState {
    /// Each input source and the level it is driving.
    pub inputs: Vec<(WireId, Level)>,
    /// The resolved input level the detector last acted on. This is the field
    /// that makes edge detection a device rather than a flag: restore it and
    /// the detector resumes without inventing an edge that never happened.
    pub last: Level,
}

/// `wire.level-to-edge`: an edge detector with state.
///
/// Holds the previous resolved input level and emits a pulse on its output
/// ([`WireSource::pulse`]) for every transition its [`EdgeTrigger`] selects.
/// Because that previous level is a field rather than a flag hidden in a
/// consumer, it round-trips through a snapshot: a detector restored
/// mid-assertion does not manufacture a spurious interrupt, which is exactly
/// what "just keep a bool somewhere" produces.
#[derive(Debug)]
pub struct LevelToEdge {
    inputs: FanIn,
    mode: Resolve,
    trigger: EdgeTrigger,
    /// The last resolved input level. Architectural state.
    last: AtomicBool,
    /// Pulse polarity: the level the output is driven to on an edge.
    active: Level,
    out: WireSource,
    /// Diagnostic only, hence absent from [`EdgeState`].
    edges: AtomicUsize,
}

impl LevelToEdge {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.level-to-edge";

    /// A detector with wired-OR inputs, emitting a high-going pulse.
    pub fn new(sources: &[WireId], trigger: EdgeTrigger, out: WireSource) -> Self {
        Self::with_options(sources, Resolve::Or, trigger, Level::High, out)
    }

    /// A detector with every knob spelled out: input resolution, which edges
    /// fire, and the polarity of the output pulse.
    pub fn with_options(
        sources: &[WireId],
        mode: Resolve,
        trigger: EdgeTrigger,
        active: Level,
        out: WireSource,
    ) -> Self {
        let inputs = FanIn::new(sources);
        let last = inputs.resolve(mode);
        LevelToEdge {
            inputs,
            mode,
            trigger,
            last: AtomicBool::new(last.as_bool()),
            active,
            out,
            edges: AtomicUsize::new(0),
        }
    }

    /// The per-source input state.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The resolved input level the detector last acted on.
    pub fn last_level(&self) -> Level {
        Level::from_bool(self.last.load(SeqCst))
    }

    /// How many pulses have been emitted. A diagnostic, not architectural
    /// state, so it is deliberately absent from [`EdgeState`].
    pub fn edge_count(&self) -> usize {
        self.edges.load(SeqCst)
    }

    /// The state to write to a snapshot.
    pub fn snapshot(&self) -> EdgeState {
        EdgeState {
            inputs: self.inputs.snapshot(),
            last: self.last_level(),
        }
    }

    /// Restore snapshotted state. Emits nothing.
    pub fn restore(&self, state: &EdgeState) {
        self.inputs.restore(&state.inputs);
        self.last.store(state.last.as_bool(), SeqCst);
    }
}

impl WireSink for LevelToEdge {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if !self.inputs.set(src, level) {
            return;
        }
        let now = self.inputs.resolve(self.mode);
        let before = Level::from_bool(self.last.swap(now.as_bool(), SeqCst));
        let Some(edge) = Edge::between(before, now) else {
            return;
        };
        if self.trigger.matches(edge) {
            self.edges.fetch_add(1, SeqCst);
            self.out.pulse(self.active);
        }
    }
}

/// The consumer half of edge semantics: latches a pulse until it is taken.
///
/// An edge-triggered input means something only if something remembers the
/// transient, and hardware does exactly this — a flip-flop the handler clears.
/// Interrupt controllers will embed the same behaviour; this is it on its own,
/// snapshottable, and useful as the far end of a [`LevelToEdge`].
#[derive(Debug)]
pub struct EdgeLatch {
    /// The level that counts as a pulse.
    active: Level,
    pending: AtomicBool,
    /// Diagnostic only.
    seen: AtomicUsize,
}

impl EdgeLatch {
    /// A latch that triggers on `active`.
    pub fn new(active: Level) -> Self {
        EdgeLatch {
            active,
            pending: AtomicBool::new(false),
            seen: AtomicUsize::new(0),
        }
    }

    /// Whether a pulse is latched.
    pub fn peek(&self) -> bool {
        self.pending.load(SeqCst)
    }

    /// Take the latched pulse, clearing it.
    pub fn take(&self) -> bool {
        self.pending.swap(false, SeqCst)
    }

    /// Clear without reporting, as a reset does.
    pub fn clear(&self) {
        self.pending.store(false, SeqCst);
    }

    /// How many pulses have been latched. Diagnostic, not architectural state.
    pub fn count(&self) -> usize {
        self.seen.load(SeqCst)
    }

    /// The latched flag, for a snapshot.
    pub fn snapshot(&self) -> bool {
        self.peek()
    }

    /// Restore the latched flag.
    pub fn restore(&self, pending: bool) {
        self.pending.store(pending, SeqCst);
    }
}

impl WireSink for EdgeLatch {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        if level == self.active {
            self.pending.store(true, SeqCst);
            self.seen.fetch_add(1, SeqCst);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    const A: WireId = WireId::new(1);
    const B: WireId = WireId::new(2);
    const C: WireId = WireId::new(3);
    const GATE: WireId = WireId::new(100);
    const GATE2: WireId = WireId::new(101);

    fn stage_id(n: u64) -> WireId {
        WireId::new(200 + n)
    }

    /// A sink that does what §4.3 says a sink must do: track which sources are
    /// asserting and resolve them itself.
    #[derive(Debug)]
    struct Irq {
        inputs: FanIn,
        asserted: AtomicBool,
        changes: AtomicUsize,
        calls: AtomicUsize,
        last_line: AtomicUsize,
    }

    impl Irq {
        fn new(sources: &[WireId]) -> Arc<Self> {
            Arc::new(Irq {
                inputs: FanIn::new(sources),
                asserted: AtomicBool::new(false),
                changes: AtomicUsize::new(0),
                calls: AtomicUsize::new(0),
                last_line: AtomicUsize::new(0),
            })
        }
        fn level(&self) -> Level {
            Level::from_bool(self.asserted.load(SeqCst))
        }
        fn changes(&self) -> usize {
            self.changes.load(SeqCst)
        }
    }

    impl WireSink for Irq {
        fn set_level(&self, src: WireId, line: u32, level: Level) {
            self.calls.fetch_add(1, SeqCst);
            self.last_line.store(line as usize, SeqCst);
            if self.inputs.set(src, level) {
                let now = self.inputs.resolve(Resolve::Or).as_bool();
                if self.asserted.swap(now, SeqCst) != now {
                    self.changes.fetch_add(1, SeqCst);
                }
            }
        }
    }

    /// A sink that only remembers the last level it was told, ignoring `src`.
    /// Fine on a single-source net, which is all it is used for here.
    #[derive(Debug)]
    struct Probe {
        level: AtomicBool,
        calls: AtomicUsize,
    }

    impl Probe {
        fn new() -> Arc<Self> {
            Arc::new(Probe {
                level: AtomicBool::new(false),
                calls: AtomicUsize::new(0),
            })
        }
        fn level(&self) -> Level {
            Level::from_bool(self.level.load(SeqCst))
        }
        fn calls(&self) -> usize {
            self.calls.load(SeqCst)
        }
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level.store(level.as_bool(), SeqCst);
            self.calls.fetch_add(1, SeqCst);
        }
    }

    /// A gate that reaches its target through a `Weak`, so a feedback loop can
    /// be built at all: the strong-reference wiring in this module is
    /// deliberately acyclic.
    #[derive(Debug)]
    struct WeakGate {
        inputs: FanIn,
        target: Weak<Wire>,
        id: WireId,
        invert: bool,
    }

    impl WireSink for WeakGate {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            if !self.inputs.set(src, level) {
                return;
            }
            let out = self.inputs.resolve(Resolve::Or);
            let out = if self.invert { out.inverted() } else { out };
            if let Some(wire) = self.target.upgrade() {
                wire.set(self.id, out);
            }
        }
    }

    fn is_send_sync<T: Send + Sync>() {}

    #[test]
    fn core_types_are_send_and_sync() {
        // Threading is a configuration, not a later port (ROADMAP.md §0).
        is_send_sync::<Wire>();
        is_send_sync::<FanIn>();
        is_send_sync::<WireSource>();
        is_send_sync::<WireSplit>();
        is_send_sync::<WireOr>();
        is_send_sync::<WireAnd>();
        is_send_sync::<WireNot>();
        is_send_sync::<LevelToEdge>();
        is_send_sync::<EdgeLatch>();
        is_send_sync::<WireIdAllocator>();
    }

    // ---- the bug this design exists to prevent ----------------------------

    #[test]
    fn wired_or_holds_the_line_when_one_source_deasserts() {
        // The APU deasserts its IRQ while the cartridge is still asserting. A
        // sink that only knew "someone said low" would drop the line here.
        let cpu = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(cpu.clone(), 0)
            .build();

        assert_eq!(cpu.level(), Level::Low);

        wire.set(A, Level::High);
        assert_eq!(cpu.level(), Level::High);

        wire.set(B, Level::High);
        assert_eq!(cpu.level(), Level::High);

        wire.set(A, Level::Low);
        assert_eq!(
            cpu.level(),
            Level::High,
            "the line must stay high while another source asserts"
        );
        assert_eq!(wire.resolve(Resolve::Or), Level::High);

        // Only when the last source deasserts does the line drop.
        wire.set(B, Level::Low);
        assert_eq!(cpu.level(), Level::Low);

        // Two transitions of the resolved line, not four.
        assert_eq!(cpu.changes(), 2);
    }

    #[test]
    fn wired_or_through_an_explicit_or_device() {
        // The same again, with the fan-in made explicit as the DSL's resolver
        // would expand it.
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let gate = Arc::new(WireOr::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(gate.clone(), 0)
            .build();

        net.set(A, Level::High);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::High);
        assert_eq!(gate.level(), Level::High);
        net.set(B, Level::Low);
        assert_eq!(probe.level(), Level::Low);
    }

    #[test]
    fn a_source_that_was_never_declared_changes_nothing() {
        // The resolver rejects such a graph; the runtime must not corrupt the
        // net's state on the way to that diagnosis.
        let cpu = Irq::new(&[A]);
        let wire = Wire::builder().source(A).sink(cpu.clone(), 0).build();
        wire.set(A, Level::High);
        assert!(!wire.set(C, Level::Low));
        assert_eq!(cpu.level(), Level::High);
        assert_eq!(wire.level_of(C), None);
        assert!(!wire.inputs().contains(C));
    }

    #[test]
    fn repeating_a_level_delivers_nothing() {
        // Devices assert unconditionally every cycle; that must be free.
        let probe = Probe::new();
        let wire = Wire::builder().source(A).sink(probe.clone(), 0).build();
        assert!(wire.set(A, Level::High));
        assert!(!wire.set(A, Level::High));
        assert!(!wire.set(A, Level::High));
        assert_eq!(probe.calls(), 1);
    }

    // ---- fan-in and fan-out ------------------------------------------------

    #[test]
    fn wired_and_needs_every_source() {
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let gate = Arc::new(WireAnd::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(gate.clone(), 0)
            .build();

        net.set(A, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(gate.level(), Level::Low);
    }

    #[test]
    fn an_and_with_no_sources_reads_as_a_pull_up() {
        let idle = FanIn::new(&[]);
        assert_eq!(idle.resolve(Resolve::And), Level::High);
        assert_eq!(idle.resolve(Resolve::Or), Level::Low);
    }

    #[test]
    fn one_wire_fans_out_to_several_sinks() {
        let a = Probe::new();
        let b = Probe::new();
        let c = Irq::new(&[A]);
        let wire = Wire::builder()
            .source(A)
            .sink(a.clone(), 0)
            .sink(b.clone(), 7)
            .sink(c.clone(), 3)
            .build();
        assert_eq!(wire.sink_count(), 3);

        wire.set(A, Level::High);
        assert_eq!(a.level(), Level::High);
        assert_eq!(b.level(), Level::High);
        assert_eq!(c.level(), Level::High);
        assert_eq!(b.calls(), 1);
        assert_eq!(c.last_line.load(SeqCst), 3, "each sink gets its own line");

        wire.set(A, Level::Low);
        assert_eq!(a.level(), Level::Low);
        assert_eq!(b.level(), Level::Low);
        assert_eq!(c.level(), Level::Low);
    }

    #[test]
    fn a_weak_sink_is_skipped_once_it_is_dropped() {
        let probe = Probe::new();
        let weak: Weak<dyn WireSink> = Arc::downgrade(&(probe.clone() as Arc<dyn WireSink>));
        let wire = Wire::builder().source(A).sink_weak(weak, 0).build();
        wire.set(A, Level::High);
        assert_eq!(probe.level(), Level::High);

        drop(probe);
        // The sink is gone; delivery must be a no-op rather than a panic.
        assert!(wire.set(A, Level::Low));
    }

    #[test]
    fn split_forwards_to_every_output() {
        let x = Probe::new();
        let y = Probe::new();
        let out_x = Wire::builder()
            .source(GATE)
            .sink(x.clone(), 0)
            .build_shared();
        let out_y = Wire::builder()
            .source(GATE)
            .sink(y.clone(), 0)
            .build_shared();
        let split = Arc::new(WireSplit::new(
            &[A],
            vec![WireSource::new(out_x, GATE), WireSource::new(out_y, GATE)],
        ));
        let net = Wire::builder().source(A).sink(split.clone(), 0).build();

        net.set(A, Level::High);
        assert_eq!(x.level(), Level::High);
        assert_eq!(y.level(), Level::High);
        assert_eq!(split.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(x.level(), Level::Low);
        assert_eq!(y.level(), Level::Low);
    }

    #[test]
    fn not_inverts_and_nors_multiple_inputs() {
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let inv = Arc::new(WireNot::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(inv.clone(), 0)
            .build();

        // Nothing has been driven yet, so the probe has seen nothing; the
        // inverter's own view is correct from the start, and announcing it
        // puts the idle high onto the wire.
        assert_eq!(inv.level(), Level::High);
        assert_eq!(probe.level(), Level::Low);
        inv.announce();
        assert_eq!(probe.level(), Level::High);

        net.set(A, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::Low, "NOR: B still asserts");
        net.set(B, Level::Low);
        assert_eq!(probe.level(), Level::High);
        assert_eq!(inv.level(), Level::High);
    }

    // ---- edges -------------------------------------------------------------

    fn edge_rig(trigger: EdgeTrigger) -> (Wire, Arc<LevelToEdge>, Arc<EdgeLatch>) {
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(latch.clone(), 0)
            .build_shared();
        let det = Arc::new(LevelToEdge::new(&[A], trigger, WireSource::new(out, GATE)));
        let net = Wire::builder().source(A).sink(det.clone(), 0).build();
        (net, det, latch)
    }

    #[test]
    fn rising_edges_only() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Rising);

        net.set(A, Level::High);
        assert!(latch.take(), "a rising edge is latched");
        assert!(!latch.take(), "and taking it clears the latch");
        assert_eq!(det.edge_count(), 1);

        net.set(A, Level::Low);
        assert!(
            !latch.peek(),
            "a falling edge does not fire a rising trigger"
        );
        assert_eq!(det.edge_count(), 1);

        net.set(A, Level::High);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);
        assert_eq!(latch.count(), 2);
    }

    #[test]
    fn falling_edges_only() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Falling);

        net.set(A, Level::High);
        assert!(!latch.peek());
        assert_eq!(det.last_level(), Level::High);
        net.set(A, Level::Low);
        assert!(latch.take(), "a falling edge is latched");
        assert_eq!(det.edge_count(), 1);
    }

    #[test]
    fn both_edges() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Both);
        net.set(A, Level::High);
        assert!(latch.take());
        net.set(A, Level::Low);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);
    }

    #[test]
    fn an_edge_detector_resolves_its_inputs_before_detecting() {
        // Two sources on one detector: a second source asserting while the
        // first already does is not a new edge, and the first deasserting while
        // the second holds is not a falling edge either.
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(latch.clone(), 0)
            .build_shared();
        let det = Arc::new(LevelToEdge::new(
            &[A, B],
            EdgeTrigger::Both,
            WireSource::new(out, GATE),
        ));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(det.clone(), 0)
            .build();

        net.set(A, Level::High);
        assert_eq!(det.edge_count(), 1);
        net.set(B, Level::High);
        assert_eq!(det.edge_count(), 1);
        net.set(A, Level::Low);
        assert_eq!(det.edge_count(), 1);
        net.set(B, Level::Low);
        assert_eq!(det.edge_count(), 2);
        assert!(latch.take());
    }

    #[test]
    fn a_pulse_leaves_the_line_where_it_started() {
        // The transient means something only to something that latches it.
        let probe = Probe::new();
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .sink(latch.clone(), 0)
            .build_shared();
        WireSource::new(out, GATE).pulse(Level::High);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(probe.calls(), 2, "both transitions are delivered");
        assert!(latch.take());
    }

    // ---- depth, re-entrancy, cycles ---------------------------------------

    #[test]
    fn propagation_survives_a_deep_chain() {
        // Sixteen inverters in series: an even number, so the far end follows
        // the near end. Also the stack-depth check.
        const STAGES: u64 = 16;
        let probe = Probe::new();

        // Built from the far end backwards, so each stage can be handed the
        // wire it drives.
        let mut wire = Wire::builder()
            .source(stage_id(STAGES))
            .sink(probe.clone(), 0)
            .build_shared();
        let mut stages: Vec<Arc<WireNot>> = Vec::new();
        for stage in (0..STAGES).rev() {
            let inv = Arc::new(WireNot::new(
                &[stage_id(stage)],
                WireSource::new(wire, stage_id(stage + 1)),
            ));
            stages.push(inv.clone());
            wire = Wire::builder()
                .source(stage_id(stage))
                .sink(inv, 0)
                .build_shared();
        }
        // Bring the chain up as a reset would: announce in topological order,
        // otherwise every stage's undriven output disagrees with its input.
        for inv in stages.iter().rev() {
            inv.announce();
        }
        assert_eq!(probe.level(), Level::Low, "an even chain follows its input");

        wire.set(stage_id(0), Level::High);
        assert_eq!(probe.level(), Level::High);
        wire.set(stage_id(0), Level::Low);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(wire.unsettled(), 0);
    }

    /// A sink that mirrors source `A` onto source `B` of the very wire that is
    /// notifying it: re-entrancy that settles, because its own change is
    /// idempotent.
    #[derive(Debug)]
    struct Mirror {
        target: Weak<Wire>,
        calls: AtomicUsize,
    }

    impl WireSink for Mirror {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            self.calls.fetch_add(1, SeqCst);
            if src == A
                && let Some(wire) = self.target.upgrade()
            {
                wire.set(B, level);
            }
        }
    }

    #[test]
    fn a_sink_may_drive_the_wire_that_is_notifying_it() {
        let mut sink: Option<Arc<Mirror>> = None;
        let wire = Arc::new_cyclic(|me: &Weak<Wire>| {
            let m = Arc::new(Mirror {
                target: me.clone(),
                calls: AtomicUsize::new(0),
            });
            sink = Some(m.clone());
            Wire::builder().sources(&[A, B]).sink(m, 0).build()
        });
        let sink = sink.expect("built");

        wire.set(A, Level::High);
        assert_eq!(wire.level_of(B), Some(Level::High), "B mirrored A");
        assert_eq!(
            sink.calls.load(SeqCst),
            2,
            "the re-entrant change is delivered by the outer pass, once"
        );
        assert_eq!(
            wire.unsettled(),
            0,
            "a settling feedback path is not a loop"
        );

        wire.set(A, Level::Low);
        assert_eq!(wire.level_of(B), Some(Level::Low));
        assert_eq!(wire.unsettled(), 0);
    }

    #[test]
    fn a_combinational_loop_is_bounded_rather_than_fatal() {
        // An inverter wired back into its own input oscillates forever in
        // hardware and would recurse forever in a naive implementation. Here it
        // costs SETTLE_LIMIT passes and a diagnostic.
        let wire = Arc::new_cyclic(|me: &Weak<Wire>| {
            let gate = Arc::new(WeakGate {
                inputs: FanIn::new(&[A, GATE]),
                target: me.clone(),
                id: GATE,
                invert: true,
            });
            Wire::builder().sources(&[A, GATE]).sink(gate, 0).build()
        });

        // A high: the NOR output is low, which is where it already sits, so the
        // loop is stable.
        wire.set(A, Level::High);
        assert_eq!(wire.unsettled(), 0);

        // A low: the output must flip, which flips the input, which flips the
        // output — the classic ring oscillator.
        wire.set(A, Level::Low);
        assert!(
            wire.unsettled() > 0,
            "an oscillating loop must be reported, not hang or overflow"
        );
        // The wire is still usable afterwards; state is not corrupted.
        assert!(wire.level_of(GATE).is_some());
    }

    #[test]
    fn a_two_wire_loop_is_also_bounded() {
        // The cycle runs through two wires, so no single wire's guard catches
        // the re-entry on its first pass. The outer wire's settle limit still
        // ends it, and the stack stays two wires deep.
        let mut inner: Option<Arc<Wire>> = None;
        let w1 = Arc::new_cyclic(|w1: &Weak<Wire>| {
            // Non-inverting relay from w2 back onto w1's GATE2 source.
            let relay = Arc::new(WeakGate {
                inputs: FanIn::new(&[GATE]),
                target: w1.clone(),
                id: GATE2,
                invert: false,
            });
            let w2 = Wire::builder().source(GATE).sink(relay, 0).build_shared();
            inner = Some(w2.clone());
            // One inversion in the loop, so it cannot settle.
            let inv = Arc::new(WireNot::new(&[A, GATE2], WireSource::new(w2, GATE)));
            Wire::builder().sources(&[A, GATE2]).sink(inv, 0).build()
        });
        assert!(inner.is_some());

        w1.set(A, Level::High);
        w1.set(A, Level::Low);
        assert!(w1.unsettled() > 0, "an odd-inversion loop must be bounded");
    }

    // ---- state -------------------------------------------------------------

    #[test]
    fn wire_state_round_trips() {
        let cpu = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(cpu.clone(), 0)
            .build();
        wire.set(A, Level::High);
        let saved = wire.snapshot();
        assert_eq!(saved, vec![(A, Level::High), (B, Level::Low)]);

        wire.set(A, Level::Low);
        wire.set(B, Level::High);
        assert_ne!(wire.snapshot(), saved);

        wire.restore(&saved);
        assert_eq!(wire.snapshot(), saved);
        // Restore is silent, so the sink is stale until refreshed: that is the
        // contract, because on a real load the sink restores itself.
        assert_eq!(cpu.inputs.level_of(B), Some(Level::High));
        wire.refresh();
        assert_eq!(cpu.inputs.level_of(A), Some(Level::High));
        assert_eq!(cpu.inputs.level_of(B), Some(Level::Low));
        assert_eq!(cpu.level(), Level::High);
    }

    #[test]
    fn edge_detector_state_round_trips_without_inventing_an_edge() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Rising);
        net.set(A, Level::High);
        assert!(latch.take());
        let saved = det.snapshot();
        assert_eq!(saved.last, Level::High);

        // Later in the timeline the line drops and rises again.
        net.set(A, Level::Low);
        net.set(A, Level::High);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);

        // Loading the snapshot back must emit nothing, and must not re-arm the
        // detector for a level that was already high.
        det.restore(&saved);
        latch.clear();
        assert_eq!(det.snapshot(), saved);
        assert!(!latch.peek(), "restoring state emits nothing");
        net.set(A, Level::High);
        assert!(!latch.peek(), "already high: no edge");
    }

    #[test]
    fn latch_state_round_trips() {
        let latch = EdgeLatch::new(Level::High);
        latch.set_level(A, 0, Level::High);
        assert!(latch.snapshot());
        latch.clear();
        latch.restore(true);
        assert!(latch.take());
        assert!(!latch.peek());
    }

    #[test]
    fn fan_in_tracks_and_resolves() {
        let f = FanIn::new(&[B, A, A]);
        assert_eq!(f.sources(), &[A, B], "sorted and deduplicated");
        assert!(f.set(A, Level::High));
        assert!(!f.set(A, Level::High));
        assert_eq!(f.level_of(A), Some(Level::High));
        assert_eq!(f.resolve(Resolve::Or), Level::High);
        assert_eq!(f.resolve(Resolve::And), Level::Low);
        assert!(f.set(B, Level::High));
        assert!(f.all_high());
        assert_eq!(f.resolve(Resolve::And), Level::High);
        f.clear();
        assert!(!f.any_high());
        assert_eq!(f.resolve(Resolve::Or), Level::Low);
        assert!(!f.set(C, Level::High), "an untracked source is ignored");
    }

    #[test]
    fn ids_are_allocated_per_machine_and_never_zero() {
        let alloc = WireIdAllocator::new();
        let first = alloc.alloc();
        let second = alloc.alloc();
        assert_ne!(first, WireId::NONE);
        assert_ne!(first, second);
        assert_eq!(first.raw(), 1);
        // A second machine numbers its wires exactly the same way.
        let other = WireIdAllocator::default();
        assert_eq!(other.alloc(), first);
    }

    #[test]
    fn level_and_edge_helpers() {
        assert_eq!(Level::default(), Level::Low);
        assert_eq!(Level::High.inverted(), Level::Low);
        assert!(Level::from_bool(true).is_high());
        assert!(Level::from_bool(false).is_low());
        assert!(bool::from(Level::High));
        assert_eq!(Level::from(true), Level::High);
        assert_eq!(Edge::between(Level::Low, Level::High), Some(Edge::Rising));
        assert_eq!(Edge::between(Level::High, Level::Low), Some(Edge::Falling));
        assert_eq!(Edge::between(Level::High, Level::High), None);
        assert!(EdgeTrigger::Both.matches(Edge::Falling));
        assert!(!EdgeTrigger::Rising.matches(Edge::Falling));
        assert_eq!(Resolve::And.idle(), Level::High);
        assert_eq!(Resolve::default(), Resolve::Or);
    }

    #[test]
    fn source_port_drives_and_reads_back() {
        let probe = Probe::new();
        let wire = Wire::builder()
            .source(A)
            .sink(probe.clone(), 0)
            .build_shared();
        let port = WireSource::new(wire.clone(), A);
        assert_eq!(port.id(), A);
        assert_eq!(port.level(), Level::Low);
        assert!(port.raise());
        assert_eq!(port.level(), Level::High);
        assert_eq!(probe.level(), Level::High);
        assert!(port.lower());
        assert_eq!(port.level(), Level::Low);
        assert!(Arc::ptr_eq(port.wire(), &wire));
    }
}
