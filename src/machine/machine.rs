//! The assembled machine: what [`realize`](mod@crate::machine::realize) produces
//! and what a run loop drives (`ROADMAP.md` §4).
//!
//! A [`Machine`] owns the four things a running machine is made of — the
//! address spaces (§4.1), the clock forest and scheduler (§4.2), the wire nets
//! (§4.3) and the device instances (§4.4) — plus the one piece of bookkeeping
//! that ties them to a snapshot: a **stable instance path** per device, which
//! is the chunk key §4.5 keys state by.
//!
//! Everything here is `no_std + alloc` and nothing names `std::sync`,
//! `std::thread` or the host clock: rate control takes a [`HostClock`]
//! injected from above the `std` line (invariant 4).
//!
//! [`HostClock`]: crate::core::sched::HostClock
//!
//! # The run loop
//!
//! ```text
//! run_quantum ─► Scheduler::run_quantum ─► Runnable::run per CPU (budgeted)
//!                                       └► events that came due
//!                    ─► Instance::event per fired event
//!                    ─► Deferred::drain  after every handler
//! ```
//!
//! Only [`ThreadingMode::Deterministic`](crate::core::sched::ThreadingMode) is
//! driven here, which is the mode §4.2 requires for record/replay and for the
//! regression suite. The other two are a `core::sched` concern and report
//! themselves unimplemented.
//!
//! Events are dispatched **after** the quantum that made them due rather than
//! from inside it, because `Scheduler::run_quantum` collects them into its
//! report. That is not a loss of precision: a quantum never runs past the next
//! deadline, so the machine is standing exactly at the event's instant when the
//! handler runs.
//!
//! # Snapshots
//!
//! [`Machine::save`] writes one chunk per device keyed by instance path, plus
//! three chunks of machine-level state: [`CLOCK_PATH`] for the oscillator
//! forest, [`SCHED_PATH`] for virtual time and the event queue, and
//! [`WIRE_PATH`] for the levels every wire source is driving. All three begin
//! with `/`, which no object name can, so they can never collide with a device.
//!
//! The scheduler chunk is there because §4.5 says the scheduler *is*
//! architectural state: the pending events, the front of virtual time and the
//! tie-break sequence counter all have to survive a load, or a restored timer
//! comes back a whole period from firing instead of the forty cycles it was
//! actually at. It is written after the clocks and read back after them too,
//! since the positions it republishes to lazily-advanced devices are derived
//! from the forest's tick counters.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::{ClockForest, DomainId, GlobalTime};
use crate::core::device::{Deferred, Device, DeviceClass, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::sched::{
    Budget, Consumed, Event, EventId, EventTarget, HostClock, QuantumReport, Runnable, RunnableId,
    Scheduler, SchedulerSnapshot,
};
use crate::core::space::{AddressSpace, RequesterId};
use crate::core::state::{MachineShape, Migrations, Sink, Source, StateReader, StateWriter};
use crate::core::wire::{Level, Wire, WireId};
use crate::machine::realize::Instance;

/// The snapshot chunk holding the oscillator forest's tick counters.
pub const CLOCK_PATH: &str = "/clock";

/// The class name recorded on the [`CLOCK_PATH`] chunk.
pub const CLOCK_CLASS: &str = "machine.clock";

/// The snapshot chunk holding every wire source's level.
pub const WIRE_PATH: &str = "/wires";

/// The class name recorded on the [`WIRE_PATH`] chunk.
pub const WIRE_CLASS: &str = "machine.wires";

/// The snapshot chunk holding the scheduler: virtual time and the event queue.
pub const SCHED_PATH: &str = "/sched";

/// The class name recorded on the [`SCHED_PATH`] chunk.
pub const SCHED_CLASS: &str = "machine.sched";

/// The version of the machine-level chunks written by this build.
pub const MACHINE_STATE_VERSION: u32 = 1;

/// One address space, with the name the machine description gave it.
///
/// The space is behind an `Arc` because devices that initiate accesses need to
/// hold their own view of it (§4.4's `Initiator`). That is also why the
/// topology is fixed once realize has finished: `AddressSpace::map` takes
/// `&mut self`, and an `Arc` with clones outstanding can never be borrowed
/// mutably again.
#[derive(Debug)]
pub struct SpaceEntry {
    name: String,
    space: Arc<AddressSpace>,
}

impl SpaceEntry {
    /// The space's name, as the `space` statement spelled it.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The space itself.
    pub fn space(&self) -> &Arc<AddressSpace> {
        &self.space
    }
}

/// One device instance and everything the machine knows about it.
#[derive(Debug)]
pub struct DeviceEntry {
    pub(crate) path: String,
    pub(crate) class: &'static DeviceClass,
    pub(crate) device: Arc<dyn Device>,
    pub(crate) instance: Option<Arc<dyn Instance>>,
    pub(crate) domain: Option<DomainId>,
    pub(crate) space: Option<usize>,
    pub(crate) requester: RequesterId,
    pub(crate) runnable: Option<RunnableId>,
}

impl DeviceEntry {
    /// The instance path — the snapshot chunk key (§4.5), stable for the life
    /// of the machine.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// The class this instance was built from.
    pub fn class(&self) -> &'static DeviceClass {
        self.class
    }

    /// The device.
    pub fn device(&self) -> &Arc<dyn Device> {
        &self.device
    }

    /// The device's machine-layer view, when its class is bound (see
    /// [`Bindings`](crate::machine::realize::Bindings)).
    pub fn instance(&self) -> Option<&Arc<dyn Instance>> {
        self.instance.as_ref()
    }

    /// The clock domain it runs in, if it declared one.
    pub fn domain(&self) -> Option<DomainId> {
        self.domain
    }

    /// The address space it declared, as an index into [`Machine::spaces`].
    pub fn space_index(&self) -> Option<usize> {
        self.space
    }

    /// Its requester id, as it appears in `MemAttrs` for accesses it initiates.
    pub fn requester(&self) -> RequesterId {
        self.requester
    }

    /// Its scheduler handle, if it takes execution budgets.
    pub fn runnable(&self) -> Option<RunnableId> {
        self.runnable
    }
}

/// One wire net: a set of pins that are the same piece of copper.
#[derive(Debug)]
pub struct Net {
    pub(crate) wire: Arc<Wire>,
    pub(crate) sources: Vec<PinRef>,
}

impl Net {
    /// The net itself.
    pub fn wire(&self) -> &Arc<Wire> {
        &self.wire
    }

    /// The pins driving it, in the order their ids were allocated.
    pub fn sources(&self) -> &[PinRef] {
        &self.sources
    }
}

/// One end of a wire: a device and one of its pins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinRef {
    /// Index into [`Machine::devices`].
    pub device: usize,
    /// The pin's name, as the device knows it.
    pub port: String,
    /// The id this pin drives the net with, for a source pin.
    pub id: WireId,
}

/// The `Runnable` the scheduler sees, wrapping the `Instance` the machine owns.
///
/// The shim exists because the two halves disagree about mutability: §4.6's
/// `Cpu::run` takes `&self` (a device is shared — it is `Send + Sync` with
/// interior mutability), while [`Runnable::run`] takes `&mut self` and the
/// scheduler takes ownership of the box. Forwarding through an `Arc` is the
/// only way to satisfy both without the machine giving up ownership of its own
/// device.
pub(crate) struct RunAdapter {
    inner: Arc<dyn Instance>,
}

impl RunAdapter {
    /// Wrap `inner` so the scheduler can hand it budgets.
    pub(crate) fn new(inner: Arc<dyn Instance>) -> RunAdapter {
        RunAdapter { inner }
    }
}

impl Runnable for RunAdapter {
    fn run(&mut self, budget: Budget) -> Consumed {
        self.inner.run(budget)
    }
}

/// A realized machine: spaces, clocks, wires and devices, ready to run.
///
/// Built by [`realize`](crate::machine::realize::realize). Nothing observable
/// happens before that call returns, so a description that fails half way
/// leaves no half-wired machine behind (§4.4).
#[derive(Debug)]
pub struct Machine {
    name: String,
    spaces: Vec<SpaceEntry>,
    sched: Scheduler,
    devices: Vec<DeviceEntry>,
    by_path: BTreeMap<String, usize>,
    nets: Vec<Net>,
    sweep: Vec<PinRef>,
    shape: MachineShape,
    deferred: Deferred,
}

/// The parts a realizer hands to [`Machine::assemble`].
///
/// A struct rather than eight positional arguments: every one of them is a
/// `Vec` or a name, and swapping two at a call site would compile.
#[derive(Debug)]
pub(crate) struct MachineParts {
    pub(crate) name: String,
    pub(crate) spaces: Vec<(String, Arc<AddressSpace>)>,
    pub(crate) sched: Scheduler,
    pub(crate) devices: Vec<DeviceEntry>,
    pub(crate) nets: Vec<Net>,
    pub(crate) sweep: Vec<PinRef>,
    pub(crate) shape: MachineShape,
    pub(crate) deferred: Deferred,
}

impl Machine {
    /// Assemble a machine from parts. Called by the realizer, and by nothing
    /// else — every field has an invariant the realizer establishes.
    pub(crate) fn assemble(parts: MachineParts) -> Machine {
        let by_path = parts
            .devices
            .iter()
            .enumerate()
            .map(|(i, d)| (d.path.clone(), i))
            .collect();
        Machine {
            name: parts.name,
            spaces: parts
                .spaces
                .into_iter()
                .map(|(name, space)| SpaceEntry { name, space })
                .collect(),
            sched: parts.sched,
            devices: parts.devices,
            by_path,
            nets: parts.nets,
            sweep: parts.sweep,
            shape: parts.shape,
            deferred: parts.deferred,
        }
    }

    /// The machine's name, as `machine "nes"` wrote it.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Every address space, in declaration order.
    pub fn spaces(&self) -> &[SpaceEntry] {
        &self.spaces
    }

    /// One address space by name.
    pub fn space(&self, name: &str) -> Option<&Arc<AddressSpace>> {
        self.spaces
            .iter()
            .find(|s| s.name == name)
            .map(SpaceEntry::space)
    }

    /// Every device instance, in declaration order — which is also reset order.
    pub fn devices(&self) -> &[DeviceEntry] {
        &self.devices
    }

    /// One device by instance path.
    pub fn device(&self, path: &str) -> Option<&DeviceEntry> {
        self.by_path.get(path).and_then(|i| self.devices.get(*i))
    }

    /// The index of the device at `path`, for scheduling events against it.
    pub fn device_index(&self, path: &str) -> Option<usize> {
        self.by_path.get(path).copied()
    }

    /// Every wire net, in the order the realizer built them.
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }

    /// The scheduler, which owns virtual time and the clock forest.
    pub fn scheduler(&self) -> &Scheduler {
        &self.sched
    }

    /// The scheduler, mutably — for posting events and changing rate control.
    pub fn scheduler_mut(&mut self) -> &mut Scheduler {
        &mut self.sched
    }

    /// The oscillator forest (§4.2).
    pub fn clocks(&self) -> &ClockForest {
        self.sched.forest()
    }

    /// The current virtual instant.
    pub fn now(&self) -> GlobalTime {
        self.sched.now()
    }

    /// The machine's structural fingerprint, which a snapshot is checked
    /// against (§4.5).
    pub fn shape(&self) -> &MachineShape {
        &self.shape
    }

    /// Inject the host's monotonic clock, for rate control.
    ///
    /// Nothing below `host/` may read a wall clock (invariant 4), so the
    /// machine is handed one rather than reaching for it.
    pub fn set_host_clock(&mut self, clock: Box<dyn HostClock>) {
        self.sched.set_host_clock(clock);
    }

    // -----------------------------------------------------------------
    // reset and the realize sweep
    // -----------------------------------------------------------------

    /// Reset every device, then re-announce every wire source (§4.3).
    ///
    /// Devices are reset in **declaration order** — the order the machine file
    /// names them — because a reset order that depends on a hash or on the
    /// wiring is a reset order that changes between runs, and §0 does not allow
    /// that. A device that must be reset after another says so by being
    /// declared after it.
    ///
    /// The deferred queue is drained after each device, so an action a reset
    /// handler pushes runs before the next device is touched, in the order it
    /// was pushed.
    pub fn reset(&mut self, kind: ResetKind) {
        for i in 0..self.devices.len() {
            let device = Arc::clone(&self.devices[i].device);
            device.reset(kind);
            self.deferred.drain();
        }
        self.sweep();
    }

    /// The realize sweep: walk wire sources in topological order and announce
    /// the level each drives (§4.3).
    ///
    /// Without it a freshly realized — or freshly restored — machine is
    /// inconsistent: an undriven wire sits low, which contradicts an inverter
    /// whose output idles high, and the interrupt line comes up wrong on some
    /// machines and only on some paths. The order is
    /// [`realize_order`](crate::machine::validate::realize_order)'s, so a
    /// source announces before anything that forwards its level.
    pub fn sweep(&mut self) {
        for pin in &self.sweep {
            if let Some(instance) = self.devices[pin.device].instance.as_ref() {
                instance.announce(&pin.port);
            }
        }
        self.deferred.drain();
    }

    // -----------------------------------------------------------------
    // running
    // -----------------------------------------------------------------

    /// Run one scheduler quantum and dispatch whatever came due.
    ///
    /// # Errors
    ///
    /// Whatever the scheduler refuses — an overrun budget, an unimplemented
    /// threading mode — or an event addressed to a device that does not exist.
    pub fn run_quantum(&mut self) -> Result<QuantumReport> {
        let report = self.sched.run_quantum()?;
        self.dispatch(&report)?;
        Ok(report)
    }

    /// Run until virtual time reaches `deadline`.
    ///
    /// The loop is here rather than in `Scheduler::run_until` because that one
    /// discards the per-quantum report, and the report is where fired events
    /// are: driving it from above is what keeps them from being dropped.
    ///
    /// # Errors
    ///
    /// As [`Machine::run_quantum`], plus a machine whose configuration cannot
    /// advance virtual time at all.
    pub fn run_until(&mut self, deadline: GlobalTime) -> Result<()> {
        while self.sched.now() < deadline {
            let before = self.sched.now();
            let report = self.sched.run_quantum_until(deadline)?;
            self.dispatch(&report)?;
            if self.sched.now() <= before {
                // A quantum always ends at `min(now + quantum, deadline, next
                // deadline)`, and events due at `now` have already been popped,
                // so this can only mean a zero quantum. Reporting it is the
                // only honest option: spinning would hang, and jumping to the
                // deadline through `Scheduler::run_until` would fire events
                // into a report nobody reads.
                return Err(Error::Config {
                    at: self.name.clone(),
                    message: "virtual time did not advance: the scheduler quantum is zero"
                        .to_string(),
                });
            }
        }
        Ok(())
    }

    /// Run for `span` of virtual time from wherever the machine is now.
    ///
    /// # Errors
    ///
    /// As [`Machine::run_quantum`].
    pub fn run_for(&mut self, span: GlobalTime) -> Result<()> {
        let deadline = self.sched.now().saturating_add(span);
        self.run_until(deadline)
    }

    /// Post an event for the device at `path`, `ticks` of its own clock domain
    /// from now.
    ///
    /// `token` is handed back to the device untouched: it is a timer index, a
    /// channel number, whatever the device put there.
    ///
    /// # Errors
    ///
    /// If no device is at `path`, if it has no clock domain, or if the clock
    /// conversion overflows.
    pub fn schedule_after_ticks(&mut self, path: &str, ticks: u64, token: u64) -> Result<EventId> {
        let index = self.device_index(path).ok_or_else(|| Error::Config {
            at: path.to_string(),
            message: "no device at this instance path".to_string(),
        })?;
        let domain = self.devices[index].domain.ok_or_else(|| Error::Config {
            at: path.to_string(),
            message: "cannot post an event for a device with no clock domain".to_string(),
        })?;
        let target = EventTarget(u32::try_from(index).unwrap_or(u32::MAX));
        Ok(self
            .sched
            .schedule_after_ticks(domain, ticks, target, token)?)
    }

    /// Deliver every event in `report` to its device, draining the deferred
    /// queue after each handler.
    ///
    /// The drain is per handler, not per quantum: an action a device defers is
    /// meant to run *after that handler returns* and before anything else
    /// observes the machine (§4.7's re-entrancy contract). Batching them to the
    /// end of the quantum would reorder them against the next event.
    fn dispatch(&mut self, report: &QuantumReport) -> Result<()> {
        for event in &report.fired {
            let index = event.target.0 as usize;
            let Some(instance) = self
                .devices
                .get(index)
                .map(|d| d.instance.clone())
                .ok_or_else(|| Error::Config {
                    at: self.name.clone(),
                    message: format!(
                        "event {} is addressed to device {index}, which does not exist",
                        event.id.seq()
                    ),
                })?
            else {
                // A device with no machine-layer view cannot have posted an
                // event, so this is a stale target rather than a lost handler.
                continue;
            };
            instance.event(event.token, &mut self.deferred);
            self.deferred.drain();
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // snapshots
    // -----------------------------------------------------------------

    /// Serialize the whole machine: one chunk per device, keyed by instance
    /// path, plus the clock forest, the scheduler and the wire levels (§4.5).
    ///
    /// # Errors
    ///
    /// Whatever a device's `save` reports, or a duplicate instance path.
    pub fn save(&self) -> Result<Vec<u8>> {
        let mut w = StateWriter::new(self.shape.clone());
        for entry in &self.devices {
            let mut chunk = w.chunk(&entry.path, entry.class.name, entry.class.version)?;
            entry.device.save(&mut chunk)?;
        }
        {
            let mut chunk = w.chunk(CLOCK_PATH, CLOCK_CLASS, MACHINE_STATE_VERSION)?;
            save_clocks(self.sched.forest(), &mut chunk)?;
        }
        {
            let mut chunk = w.chunk(SCHED_PATH, SCHED_CLASS, MACHINE_STATE_VERSION)?;
            save_sched(&self.sched, &mut chunk)?;
        }
        {
            let mut chunk = w.chunk(WIRE_PATH, WIRE_CLASS, MACHINE_STATE_VERSION)?;
            save_wires(&self.nets, &mut chunk)?;
        }
        w.to_vec()
    }

    /// Restore what [`Machine::save`] wrote, with no class migrations.
    ///
    /// # Errors
    ///
    /// As [`Machine::load_with`].
    pub fn load(&mut self, bytes: &[u8]) -> Result<()> {
        self.load_with(bytes, &Migrations::new())
    }

    /// Restore a snapshot, migrating device chunks through `migrations`.
    ///
    /// The machine's shape is checked first, so a snapshot taken from a
    /// differently-shaped machine fails with a diff naming what moved rather
    /// than by loading nonsense into the wrong device (§4.5).
    ///
    /// The realize sweep runs afterwards: a restored machine is as inconsistent
    /// as a fresh one until every gate drives what its inputs imply.
    ///
    /// # Errors
    ///
    /// A shape mismatch, a missing or mis-classed chunk, a migration hole, or
    /// whatever a device's `load` reports.
    pub fn load_with(&mut self, bytes: &[u8], migrations: &Migrations) -> Result<()> {
        let reader = StateReader::new(bytes)?;
        reader.check_shape(&self.shape)?;
        for entry in &self.devices {
            let chunk = reader.load(
                &entry.path,
                entry.class.name,
                entry.class.version,
                migrations,
            )?;
            let mut r = chunk.reader();
            entry.device.load(&mut r)?;
        }
        let clocks = reader.load(CLOCK_PATH, CLOCK_CLASS, MACHINE_STATE_VERSION, migrations)?;
        load_clocks(self.sched.forest_mut(), &mut clocks.reader())?;
        // After the clocks: the scheduler's restore republishes every lazily
        // advanced device's domain position, which is only right once the tick
        // counters those positions come from are back.
        let sched = reader.load(SCHED_PATH, SCHED_CLASS, MACHINE_STATE_VERSION, migrations)?;
        load_sched(&mut self.sched, &mut sched.reader())?;
        let wires = reader.load(WIRE_PATH, WIRE_CLASS, MACHINE_STATE_VERSION, migrations)?;
        load_wires(&self.nets, &mut wires.reader())?;
        self.deferred.drain();
        self.sweep();
        Ok(())
    }

    /// A hash of the machine's serialized state.
    ///
    /// The regression method of §0 in one call: run deterministically for N
    /// virtual units and compare this number. It is a hash of [`Machine::save`]
    /// output, which `core::state` guarantees is byte-identical for identical
    /// state, so equal hashes mean equal state and not merely equal-looking
    /// state.
    ///
    /// # Errors
    ///
    /// As [`Machine::save`].
    pub fn state_hash(&self) -> Result<u64> {
        Ok(fnv1a(&self.save()?))
    }
}

/// FNV-1a over the snapshot bytes.
///
/// Not a cryptographic hash and not meant to be: `purecrypto`'s BLAKE3 is the
/// integrity seam (§4.5), and this is a test and regression comparison that has
/// to work in a dependency-free `no_std` build.
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Write every oscillator's unit position and every domain's tick counter.
///
/// The tick counters are the authoritative architectural state (§4.2); the
/// global timeline is derived from them and is recomputed on restore.
///
/// Both sequences come from the forest's own enumeration, in creation order, so
/// the writer and the reader agree without either of them having to have kept a
/// list of handles.
fn save_clocks(forest: &ClockForest, sink: &mut impl Sink) -> Result<()> {
    let oscillators: Vec<_> = forest.oscillators().collect();
    sink.write_seq_len(oscillators.len() as u64)?;
    for osc in oscillators {
        sink.write_u64(forest.unit_position(osc)?)?;
    }
    let domains: Vec<_> = forest.domains().collect();
    sink.write_seq_len(domains.len() as u64)?;
    for id in domains {
        sink.write_u64(forest.ticks(id)?)?;
    }
    Ok(())
}

/// Restore what [`save_clocks`] wrote.
fn load_clocks<'a>(forest: &mut ClockForest, src: &mut impl Source<'a>) -> Result<()> {
    let domains: Vec<_> = forest.domains().collect();
    let count = src.read_seq_len(8)? as usize;
    let oscillators: Vec<_> = forest.oscillators().collect();
    if count != oscillators.len() {
        return Err(Error::State(format!(
            "snapshot has {count} oscillators, this machine has {}",
            oscillators.len()
        )));
    }
    // Unit positions first: §4.2's tick counters are anchored to them, so
    // restoring in the other order rebases every counter onto the old front.
    for osc in oscillators {
        forest.restore_unit_position(osc, src.read_u64()?)?;
    }
    let count = src.read_seq_len(8)? as usize;
    if count != domains.len() {
        return Err(Error::State(format!(
            "snapshot has {count} clock domains, this machine has {}",
            domains.len()
        )));
    }
    for id in domains {
        forest.restore_ticks(id, src.read_u64()?)?;
    }
    Ok(())
}

/// Write the scheduler's own architectural state (§4.5).
///
/// Virtual time, every pending event in fire order, the tie-break sequence
/// counter and the round-robin cursor. Re-deriving the queue by asking devices
/// to re-register would lose sub-tick phase — a timer 40 cycles from firing
/// would come back a whole period from firing — so the queue is written
/// verbatim.
fn save_sched(sched: &Scheduler, sink: &mut impl Sink) -> Result<()> {
    let snapshot = sched.snapshot();
    sink.write_u128(snapshot.now.raw())?;
    sink.write_u64(snapshot.next_seq)?;
    sink.write_u64(snapshot.cursor as u64)?;
    sink.write_seq_len(snapshot.events.len() as u64)?;
    for event in &snapshot.events {
        sink.write_u128(event.time.raw())?;
        sink.write_u64(event.id.seq())?;
        sink.write_u32(event.target.0)?;
        sink.write_u64(event.token)?;
    }
    Ok(())
}

/// Restore what [`save_sched`] wrote.
fn load_sched<'a>(sched: &mut Scheduler, src: &mut impl Source<'a>) -> Result<()> {
    let now = GlobalTime::from_raw(src.read_u128()?);
    let next_seq = src.read_u64()?;
    let cursor = usize::try_from(src.read_u64()?)
        .map_err(|_| Error::State(String::from("scheduler cursor does not fit this host")))?;
    // Sixteen bytes of instant, eight of sequence, four of target, eight of
    // token: an event cannot encode in fewer, so a corrupt count is caught
    // before anything is reserved.
    let count = src.read_seq_len(36)? as usize;
    let mut events = Vec::with_capacity(count.min(src.remaining()));
    for _ in 0..count {
        events.push(Event {
            time: GlobalTime::from_raw(src.read_u128()?),
            id: EventId::from_seq(src.read_u64()?),
            target: EventTarget(src.read_u32()?),
            token: src.read_u64()?,
        });
    }
    sched.restore(&SchedulerSnapshot {
        now,
        next_seq,
        cursor,
        events,
    })?;
    Ok(())
}

/// Write each net's per-source levels.
fn save_wires(nets: &[Net], sink: &mut impl Sink) -> Result<()> {
    sink.write_seq_len(nets.len() as u64)?;
    for net in nets {
        let levels = net.wire.snapshot();
        sink.write_seq_len(levels.len() as u64)?;
        for (id, level) in levels {
            sink.write_u64(id.raw())?;
            sink.write_u8(u8::from(level.is_high()))?;
        }
    }
    Ok(())
}

/// Restore what [`save_wires`] wrote.
fn load_wires<'a>(nets: &[Net], src: &mut impl Source<'a>) -> Result<()> {
    // A net encodes at least its own source count.
    let count = src.read_seq_len(8)? as usize;
    if count != nets.len() {
        return Err(Error::State(format!(
            "snapshot has {count} wire nets, this machine has {}",
            nets.len()
        )));
    }
    for net in nets {
        // Nine bytes per source: a `u64` id and a level byte.
        let sources = src.read_seq_len(9)? as usize;
        let mut levels = Vec::with_capacity(sources.min(src.remaining()));
        for _ in 0..sources {
            let id = WireId::new(src.read_u64()?);
            let level = match src.read_u8()? {
                0 => Level::Low,
                1 => Level::High,
                other => {
                    return Err(Error::State(format!("wire level {other} is not 0 or 1")));
                }
            };
            levels.push((id, level));
        }
        net.wire.restore(&levels);
    }
    // Restoring a level does not *deliver* it, and a sink's own fan-in is
    // derived state that nothing else rebuilds — so a re-announce is what
    // makes the sinks agree with the wires again. It has to come after every
    // net is restored: a sink that re-drives its output would otherwise write
    // over a net whose saved levels had not been put back yet.
    for net in nets {
        net.wire.refresh();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_machine_chunk_paths_cannot_collide_with_a_device() {
        // Object names come from the resolver's identifier grammar, which has
        // no `/`. That is the whole reason the machine's own chunks are spelled
        // with one.
        assert!(CLOCK_PATH.starts_with('/'));
        assert!(WIRE_PATH.starts_with('/'));
    }

    #[test]
    fn the_state_hash_is_a_function_of_the_bytes() {
        assert_eq!(fnv1a(b"abc"), fnv1a(b"abc"));
        assert_ne!(fnv1a(b"abc"), fnv1a(b"abd"));
        // An empty snapshot still hashes to the FNV offset basis rather than 0,
        // so "no state" and "hash not computed" are distinguishable.
        assert_eq!(fnv1a(b""), 0xcbf2_9ce4_8422_2325);
    }
}
