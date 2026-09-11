//! Catching a processor that is spinning on a load whose value never changes.
//!
//! # The failure this exists for
//!
//! The most common way a board fails during bring-up is silently. Firmware
//! polls a status bit — a `DRDY` on a peripheral that is not modelled, a PLL
//! lock that needs a clock nobody wired, a `CYCCNT` that reads zero — and the
//! bit never sets, so the loop never ends. rsemu already reports the
//! neighbouring failures: an access to an address nothing is mapped at goes
//! through [`UnassignedPolicy::LOG`](crate::core::space::UnassignedPolicy), and
//! a machine that cannot advance virtual time at all is refused by
//! `Machine::run_until`. Neither says anything here, because the address *is*
//! mapped, something *does* answer, and virtual time advances perfectly well.
//! The symptom is that the run never ends, and finding the loop means
//! attaching gdb and reading the program counter.
//!
//! So: watch the loads. When one processor loads the same address from the same
//! instruction and gets the same value, over and over, with no store of its own
//! and no interrupt in between, it is not making progress and it is worth
//! saying so.
//!
//! # What "no progress" is approximated by, and why that approximation
//!
//! A processor's *architectural* progress is its whole register file, and
//! comparing that per instruction is not a diagnostic, it is a second emulator.
//! The approximation here is the one that costs a comparison of three words:
//!
//! * the same **(pc, address, value)** on consecutive loads — the instruction
//!   is re-executing and the memory it reads has not moved;
//! * broken by a **store** from the same processor, because a loop that writes
//!   anything is a loop that could be driving the thing it is waiting for (a
//!   kick loop, a memcpy, a counter in RAM). Stores are only ever a *reset*;
//!   they are never reported, because a repeated store is usually correct;
//! * broken by **interrupt entry**, because a loop waiting on a handler that is
//!   legitimately slow is not stuck.
//!
//! What it therefore does **not** catch, stated so nobody discovers it the hard
//! way:
//!
//! * A poll loop with **two loads in its body** — `ldr`/`ldr`/`tst` — never
//!   builds a streak, because consecutive loads differ. One slot was chosen
//!   over *n* because one slot is three comparisons against registers the core
//!   already has, and because the overwhelmingly common shape a compiler emits
//!   for `while (!(REG & BIT))` is a single load.
//! * A spin whose interrupts keep arriving — a SysTick that fires every
//!   thousand cycles resets the streak every thousand cycles, so a threshold
//!   above that never trips. That is the documented consequence of the
//!   interrupt rule above, and the fix when it bites is a lower threshold.
//! * Loads issued by a **translated block**. The hooks are in the interpreters;
//!   a lifted block reaches memory through the IR host and does not call them.
//!   Every core in this crate interprets by default, so this is a limitation of
//!   `engine = "jit"` rather than of an ordinary run.
//!
//! # Cost
//!
//! Disarmed, a hook is **one load of a `u64` field the core already has in
//! cache and a branch that is never taken** — [`Watch::load`] is `#[inline]`
//! and everything behind the test is `#[cold]`, so the fall-through is what the
//! branch predictor and the code layout are optimised for. There is no atomic
//! on the per-load path at all: the threshold is *copied* into the [`Watch`]
//! when the detector is attached and re-read once per scheduler round, never
//! per access.
//!
//! Measured, callgrind, `--release`, on a Cortex-M4 firmware that does nothing
//! but poll — one watched load every six guest cycles, which is as dense as a
//! load path gets — against the same tree with the call sites compiled out:
//! **+1.67 host instructions per guest load disarmed, +8.67 armed**. The armed
//! figure is the `#[cold]` call plus the comparisons; inlining [`Watch::load`]'s
//! body would take some of it back and would cost the disarmed path its layout,
//! which is the wrong trade for a diagnostic that is off almost always.
//! `docs/testing/tracing.md` has the table, the denominators and how to
//! reproduce it.
//!
//! **Run-time arming rather than a Cargo feature**, unlike
//! [`crate::core::trace`]'s counters. The person this exists for is bringing a
//! board up and does not yet know they will need it, so a build they have to
//! redo is a build they will not have; and `core/` is never feature-gated in
//! any case. Under two instructions per load is what makes that affordable.
//!
//! # Determinism
//!
//! Nothing here is visible to the guest and nothing here reads a clock. A
//! [`Watch`] is not architectural state: it is not written to a snapshot, it is
//! not in `Machine::state_hash`, and a restore does not carry one. The streak
//! is counted in *loads*, not in time, so two identical runs count identically.
//!
//! Reports are collected per [`Detector`] rather than in a static, which is the
//! property that makes them reproducible: two machines in one process — or two
//! tests in one test binary — have two detectors and cannot contaminate each
//! other. [`Detector::events`] additionally returns its events stably sorted by
//! processor, so even [`ThreadingMode::Parallel`], where two cores genuinely
//! run at once, produces one order.
//!
//! [`ThreadingMode::Parallel`]: crate::core::sched::ThreadingMode

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, Mapping, Region, RequesterId};
use crate::core::sync::{AtomicBool, AtomicU32, LockRank, Mutex, Ordering};
use crate::core::trace::{Event, EventKind};

/// A streak length that no ordinary loop reaches and a stuck one reaches
/// quickly.
///
/// Ten thousand *loads*, not ten thousand cycles: on a 168 MHz Cortex-M4
/// spinning in a six-cycle loop that is about a third of a millisecond of guest
/// time, and on a 1 MHz 6502 it is nearer sixty. Counting loads rather than time
/// is what makes it deterministic (see the module docs), and the consequence is
/// that the delay before a report is a property of the board. Lower it when a
/// slow part makes the wait tiresome, and raise it if a legitimate loop on some
/// board turns out to read the same word ten thousand times running.
pub const DEFAULT_THRESHOLD: u64 = 10_000;

/// How many events one detector keeps.
///
/// A cap rather than an unbounded `Vec`, because this is a diagnostic that runs
/// while something is already wrong: a firmware that polls two dozen dead
/// peripherals in turn would otherwise grow a report per streak forever.
/// Overflow is counted by [`Detector::dropped`] rather than silently discarded
/// — a report that quietly stops being complete is worse than no report.
pub const MAX_EVENTS: usize = 64;

/// How deep the region walk that names an address will go.
///
/// The same bound `core::space::flat` uses, and for the same reason: a
/// container tree is shallow in practice and this is recursion on a small wasm
/// stack.
const MAX_DEPTH: u32 = 64;

// ---------------------------------------------------------------------------
// The detector
// ---------------------------------------------------------------------------

/// The armed state a machine shares with its processors, and where their
/// findings go.
///
/// One per machine, held behind an [`Arc`] that each core keeps beside its
/// address space — which is the right place for it for the reason
/// [`Device::export`](crate::core::device::Device::export) gives about handles
/// generally: it is wiring, not guest state, so it survives a reset and is
/// never serialized.
#[derive(Debug)]
pub struct Detector {
    /// The streak length that trips a report, or zero for disarmed.
    ///
    /// `u32` rather than `u64` deliberately: [`AtomicU64`] is not available on
    /// every target `core::sync` supports, `core/` is never feature-gated, and
    /// a threshold above four billion loads is not a threshold anybody wants.
    ///
    /// [`AtomicU64`]: crate::core::sync::AtomicU64
    threshold: AtomicU32,
    /// Whether a report should end the run rather than only be recorded.
    stop: AtomicBool,
    /// What has been found so far.
    log: Mutex<Log>,
}

/// The events one detector has collected.
#[derive(Debug)]
struct Log {
    events: Vec<Event>,
    dropped: u64,
}

impl Detector {
    /// A detector armed at `threshold` consecutive loads.
    ///
    /// A threshold of zero is disarmed, which is what
    /// [`Detector::disarm`] produces and what every core sees until a machine
    /// hands one out.
    #[must_use]
    pub fn new(threshold: u64) -> Arc<Detector> {
        Arc::new(Detector {
            threshold: AtomicU32::new(clamp(threshold)),
            stop: AtomicBool::new(false),
            // LEAF, and it has to be: a core emits while holding its own
            // execution lock, which is ranked BUS.
            log: Mutex::with_rank(
                LockRank::LEAF,
                Log {
                    events: Vec::new(),
                    dropped: 0,
                },
            ),
        })
    }

    /// The streak length that trips a report; zero when disarmed.
    #[must_use]
    pub fn threshold(&self) -> u64 {
        u64::from(self.threshold.load(Ordering::Relaxed))
    }

    /// Change the threshold.
    ///
    /// A core picks this up at the start of its next scheduler round rather
    /// than at the next load — the per-load path deliberately holds a copy and
    /// touches no atomic. A value above [`u32::MAX`] is clamped to it.
    pub fn set_threshold(&self, threshold: u64) {
        self.threshold.store(clamp(threshold), Ordering::Relaxed);
    }

    /// Stop watching. Equivalent to a threshold of zero.
    pub fn disarm(&self) {
        self.threshold.store(0, Ordering::Relaxed);
    }

    /// Whether anything is being watched.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.threshold.load(Ordering::Relaxed) != 0
    }

    /// Whether a report should end the run — the issue's *budget mode*.
    ///
    /// With this set, `Machine::run_until` and everything built on it stop at
    /// the end of the round a report landed in and return
    /// [`Error::Spin`](crate::core::Error::Spin). That is what turns a CI test
    /// that would have hung into one that fails in seconds, naming the loop.
    #[must_use]
    pub fn stops_the_run(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }

    /// Set whether a report ends the run.
    pub fn set_stops_the_run(&self, yes: bool) {
        self.stop.store(yes, Ordering::Relaxed);
    }

    /// Record one finding.
    ///
    /// Called by a [`Watch`] and by nothing else, while the emitting core holds
    /// its own execution lock — hence [`LockRank::LEAF`] on the log.
    fn emit(&self, event: Event) {
        let mut log = self.log.lock();
        if log.events.len() >= MAX_EVENTS {
            log.dropped = log.dropped.saturating_add(1);
            return;
        }
        log.events.push(event);
    }

    /// Everything found so far, stably sorted by processor.
    ///
    /// The sort is what makes this deterministic under
    /// [`ThreadingMode::Parallel`](crate::core::sched::ThreadingMode): each
    /// core appends its own findings in its own deterministic order, and
    /// grouping by core is the only thing an interleaving could have disturbed.
    #[must_use]
    pub fn events(&self) -> Vec<Event> {
        let mut events = self.log.lock().events.clone();
        events.sort_by_key(|e| e.cpu);
        events
    }

    /// How many findings did not fit in [`MAX_EVENTS`].
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.log.lock().dropped
    }

    /// Forget everything found so far, leaving the arming alone.
    ///
    /// What a caller does between runs, and what `Machine::run_until` does
    /// after reporting one, so the next call is not refused by the same event.
    pub fn clear(&self) {
        let mut log = self.log.lock();
        log.events.clear();
        log.dropped = 0;
    }
}

/// Fit a threshold into the atomic that holds it.
fn clamp(threshold: u64) -> u32 {
    u32::try_from(threshold).unwrap_or(u32::MAX)
}

// ---------------------------------------------------------------------------
// The per-processor watch
// ---------------------------------------------------------------------------

/// One processor's view of the detector: the streak it is building, and the
/// hooks its interpreter calls.
///
/// Lives beside the core's address space rather than inside its architectural
/// state — it is derived, it is not serialized, and a snapshot restore must not
/// drop the detector the machine attached.
#[derive(Debug, Clone, Default)]
pub struct Watch {
    /// The detector to report to, when there is one.
    detector: Option<Arc<Detector>>,
    /// A copy of [`Detector::threshold`]. **This is the hot-path guard**: zero
    /// means disarmed, and reading it costs no atomic.
    threshold: u64,
    /// Which processor this is, in the report.
    cpu: u32,
    /// How many consecutive loads have matched. Zero means no streak — which is
    /// what a store or an interrupt leaves behind, in one store instruction.
    streak: u64,
    /// The instruction the streak is at.
    pc: u64,
    /// The address it is loading.
    addr: u64,
    /// The value it keeps getting.
    value: u64,
}

impl Watch {
    /// A disarmed watch for the processor `cpu` identifies.
    #[must_use]
    pub fn new(cpu: RequesterId) -> Watch {
        Watch {
            cpu: cpu.0,
            ..Watch::default()
        }
    }

    /// Attach, replace or remove the detector this processor reports to, and
    /// tell it which processor it is.
    ///
    /// Takes effect immediately: the threshold is copied here and now, so a
    /// caller that arms a detector and then runs does not have to think about
    /// when the core will notice.
    pub fn attach(&mut self, cpu: u32, detector: Option<Arc<Detector>>) {
        self.cpu = cpu;
        self.detector = detector;
        self.streak = 0;
        self.refresh();
    }

    /// Re-read the detector's threshold.
    ///
    /// Called once per scheduler round by the core, which is the granularity a
    /// change in arming takes effect at — deliberately, because the alternative
    /// is an atomic load per guest load.
    pub fn refresh(&mut self) {
        self.threshold = self.detector.as_ref().map_or(0, |d| d.threshold());
    }

    /// The detector this processor reports to.
    #[must_use]
    pub fn detector(&self) -> Option<&Arc<Detector>> {
        self.detector.as_ref()
    }

    /// Whether this watch is armed.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.threshold != 0
    }

    /// The interpreter loaded `value` from `addr` while executing the
    /// instruction at `pc`.
    ///
    /// `phys` is the same address after whatever translation the core does,
    /// and is used only to *name* the region in a report; a core with no MMU
    /// passes `addr` twice. Both are taken because the number a reader wants to
    /// see is the virtual one — it is what is in the listing — while the number
    /// the topology can resolve is the physical one.
    ///
    /// **The hot path.** Disarmed, this is a field load and a not-taken branch;
    /// the work is `#[cold]` so that is what the code layout favours.
    #[inline]
    pub fn load(&mut self, space: &AddressSpace, pc: u64, addr: u64, phys: u64, value: u64) {
        if self.threshold == 0 {
            return;
        }
        self.watched(space, pc, addr, phys, value);
    }

    /// The armed half of [`Watch::load`], out of line.
    #[cold]
    #[inline(never)]
    fn watched(&mut self, space: &AddressSpace, pc: u64, addr: u64, phys: u64, value: u64) {
        if self.streak != 0 && pc == self.pc && addr == self.addr && value == self.value {
            self.streak += 1;
            // `==` and not `>=`, which is the whole of "emit once": past the
            // threshold the streak keeps counting and never trips again, and
            // only a change of value — or a store, or an interrupt — starts a
            // streak that can.
            if self.streak == self.threshold {
                self.report(space, phys);
            }
            return;
        }
        self.pc = pc;
        self.addr = addr;
        self.value = value;
        self.streak = 1;
    }

    /// Build and file the event for a streak that has just reached its
    /// threshold.
    #[cold]
    #[inline(never)]
    fn report(&mut self, space: &AddressSpace, phys: u64) {
        let Some(detector) = &self.detector else {
            return;
        };
        detector.emit(Event {
            kind: EventKind::SPIN,
            cpu: self.cpu,
            pc: self.pc,
            addr: self.addr,
            value: self.value,
            count: self.streak,
            region: region_at(space, phys),
        });
    }

    /// This processor stored something. Ends any streak.
    ///
    /// Stores are never reported — a repeated store to a repeated address is
    /// usually a kick loop and usually correct — they only reset.
    #[inline]
    pub fn store(&mut self) {
        if self.threshold != 0 {
            self.streak = 0;
        }
    }

    /// This processor took an interrupt or an exception. Ends any streak.
    #[inline]
    pub fn interrupt(&mut self) {
        if self.threshold != 0 {
            self.streak = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Naming an address
// ---------------------------------------------------------------------------

/// What is mapped at `addr`, as deep a name as the topology can give.
///
/// Done once, when a streak trips, and never on the access path: it takes the
/// space's read guard and walks the mapping tree, which is a cost a report can
/// afford and a load cannot.
///
/// Returns `None` for an address nothing covers, and for a space whose
/// topology is being rewritten — [`AddressSpace::try_view`] rather than
/// `view`, because a diagnostic that panics is not a diagnostic.
fn region_at(space: &AddressSpace, addr: u64) -> Option<String> {
    let view = space.try_view()?;
    let mut best: Option<&Mapping> = None;
    for (_, mapping) in view.mappings() {
        if addr < mapping.base || addr >= mapping.end() {
            continue;
        }
        if best.is_none_or(|b| mapping.priority >= b.priority) {
            best = Some(mapping);
        }
    }
    let mapping = best?;
    Some(leaf_name(&mapping.region, addr - mapping.base, 0))
}

/// The name of the deepest region covering `offset` within `region`.
///
/// A container reports its innermost child and an alias reports its target, so
/// a peripheral inside a SoC container is named `rcc` rather than `periph`.
/// Ties between equal-priority children go to the later one, matching the
/// flattener; it is a label, not a dispatch decision, so an exact tie-break is
/// not worth a second implementation of the flattener's rules.
fn leaf_name(region: &Region, offset: u64, depth: u32) -> String {
    if depth < MAX_DEPTH {
        if let Some(container) = region.as_container() {
            let mut best: Option<&Mapping> = None;
            for child in container.children() {
                if offset < child.base || offset >= child.end() {
                    continue;
                }
                if best.is_none_or(|b| child.priority >= b.priority) {
                    best = Some(child);
                }
            }
            if let Some(child) = best {
                return leaf_name(&child.region, offset - child.base, depth + 1);
            }
        }
        if let Some(alias) = region.as_alias() {
            return leaf_name(
                alias.target(),
                offset.wrapping_add(alias.offset()),
                depth + 1,
            );
        }
    }
    region.name().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{Mapping as SpaceMapping, RamStore, Region};
    use alloc::format;
    use alloc::vec;

    /// A space with `ram` at zero and a two-level container above it, so the
    /// naming walk has something to walk.
    fn space() -> AddressSpace {
        let space = AddressSpace::new("mem", 32);
        let ram = Region::ram("ram", Arc::new(RamStore::new(0x1000)));
        space.topology().map(ram, 0).expect("maps");
        let inner = Region::ram("rcc", Arc::new(RamStore::new(0x400)));
        let periph = Region::container("periph", 0x1000, vec![SpaceMapping::new(inner, 0x800)]);
        space.topology().map(periph, 0x4000_0000).expect("maps");
        space
    }

    fn armed(threshold: u64) -> (Watch, Arc<Detector>) {
        let detector = Detector::new(threshold);
        let mut watch = Watch::new(RequesterId(3));
        watch.attach(3, Some(Arc::clone(&detector)));
        (watch, detector)
    }

    #[test]
    fn a_streak_trips_exactly_at_the_threshold_and_once() {
        let space = space();
        let (mut watch, detector) = armed(4);
        for _ in 0..100 {
            watch.load(&space, 0x100, 0x40, 0x40, 0);
        }
        let events = detector.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].count, 4, "the count is the threshold, exactly");
        assert_eq!(events[0].cpu, 3, "the index it was attached with");
        assert_eq!(events[0].kind, EventKind::SPIN);
    }

    #[test]
    fn a_change_of_pc_address_or_value_starts_a_new_streak() {
        let space = space();
        for (pc, addr, value) in [
            (0x104u64, 0x40u64, 0u64),
            (0x100, 0x44, 0),
            (0x100, 0x40, 1),
        ] {
            let (mut watch, detector) = armed(4);
            for _ in 0..3 {
                watch.load(&space, 0x100, 0x40, 0x40, 0);
            }
            // The fourth load would have tripped it; this one differs, so the
            // streak restarts and four more are needed.
            watch.load(&space, pc, addr, addr, value);
            assert!(detector.events().is_empty());
            for _ in 0..3 {
                watch.load(&space, pc, addr, addr, value);
            }
            assert_eq!(detector.events().len(), 1);
        }
    }

    #[test]
    fn a_store_or_an_interrupt_ends_the_streak() {
        let space = space();
        for reset in [Watch::store as fn(&mut Watch), Watch::interrupt] {
            let (mut watch, detector) = armed(4);
            for _ in 0..1000 {
                watch.load(&space, 0x100, 0x40, 0x40, 0);
                reset(&mut watch);
            }
            assert!(detector.events().is_empty());
        }
    }

    #[test]
    fn a_disarmed_watch_counts_nothing() {
        let space = space();
        let mut watch = Watch::default();
        assert!(!watch.is_armed());
        for _ in 0..1000 {
            watch.load(&space, 0x100, 0x40, 0x40, 0);
        }
        // And a detector at zero is the same thing said the other way.
        let (mut watch, detector) = armed(0);
        assert!(!watch.is_armed());
        for _ in 0..1000 {
            watch.load(&space, 0x100, 0x40, 0x40, 0);
        }
        assert!(detector.events().is_empty());
    }

    #[test]
    fn a_threshold_change_is_picked_up_at_the_next_refresh_and_not_before() {
        let space = space();
        let (mut watch, detector) = armed(0);
        detector.set_threshold(4);
        for _ in 0..1000 {
            watch.load(&space, 0x100, 0x40, 0x40, 0);
        }
        assert!(detector.events().is_empty(), "the copy is still zero");
        watch.refresh();
        for _ in 0..4 {
            watch.load(&space, 0x100, 0x40, 0x40, 0);
        }
        assert_eq!(detector.events().len(), 1);
    }

    #[test]
    fn a_report_names_the_deepest_region_covering_the_address() {
        let space = space();
        let (mut watch, detector) = armed(2);
        watch.load(&space, 0x100, 0x4000_0800, 0x4000_0800, 0);
        watch.load(&space, 0x100, 0x4000_0800, 0x4000_0800, 0);
        assert_eq!(detector.events()[0].region.as_deref(), Some("rcc"));

        // And an address nothing covers says nothing rather than guessing.
        let (mut watch, detector) = armed(2);
        watch.load(&space, 0x100, 0xdead_0000, 0xdead_0000, 0);
        watch.load(&space, 0x100, 0xdead_0000, 0xdead_0000, 0);
        assert_eq!(detector.events()[0].region, None);
    }

    #[test]
    fn events_are_capped_and_the_overflow_is_counted() {
        let space = space();
        let (mut watch, detector) = armed(2);
        for n in 0..(MAX_EVENTS as u64 + 10) {
            watch.load(&space, 0x100, 0x40, 0x40, n);
            watch.load(&space, 0x100, 0x40, 0x40, n);
        }
        assert_eq!(detector.events().len(), MAX_EVENTS);
        assert_eq!(detector.dropped(), 10);
    }

    #[test]
    fn events_come_back_grouped_by_processor_whatever_order_they_arrived_in() {
        // The determinism property: two cores interleaving under parallel
        // threading must still produce one reading order.
        let space = space();
        let detector = Detector::new(2);
        let mut cpus: Vec<Watch> = (0..3)
            .map(|n| {
                let mut w = Watch::new(RequesterId(0));
                w.attach(2 - n, Some(Arc::clone(&detector)));
                w
            })
            .collect();
        for _ in 0..2 {
            for w in &mut cpus {
                w.load(&space, 0x100, 0x40, 0x40, 0);
            }
        }
        let order: Vec<u32> = detector.events().iter().map(|e| e.cpu).collect();
        assert_eq!(order, vec![0, 1, 2]);
    }

    #[test]
    fn clearing_forgets_the_findings_and_leaves_the_arming_alone() {
        let space = space();
        let (mut watch, detector) = armed(2);
        watch.load(&space, 0x100, 0x40, 0x40, 0);
        watch.load(&space, 0x100, 0x40, 0x40, 0);
        assert_eq!(detector.events().len(), 1);
        detector.clear();
        assert!(detector.events().is_empty());
        assert_eq!(detector.dropped(), 0);
        assert!(detector.is_armed());
    }

    #[test]
    fn a_threshold_wider_than_the_counter_is_clamped_rather_than_wrapped() {
        // The atomic is a `u32` because not every target `core::sync` supports
        // has a 64-bit one. Wrapping would turn an absurd threshold into a
        // small one, which is the failure worth refusing.
        let detector = Detector::new(u64::from(u32::MAX) + 1000);
        assert_eq!(detector.threshold(), u64::from(u32::MAX));
    }

    #[test]
    fn an_event_renders_as_a_sentence() {
        let event = Event {
            kind: EventKind::SPIN,
            cpu: 0,
            pc: 0x0800_1234,
            addr: 0x4002_3800,
            value: 0x83,
            count: 100_000,
            region: Some(String::from("rcc")),
        };
        assert_eq!(
            format!("{event}"),
            "cpu0 spinning at 0x08001234 reading 0x40023800 (rcc) = 0x00000083 \
             for 100000 iterations"
        );
        // The same finding with nothing to name it drops the parenthesis
        // rather than printing an empty one.
        let event = Event {
            region: None,
            ..event
        };
        assert!(!format!("{event}").contains("()"));
    }
}
