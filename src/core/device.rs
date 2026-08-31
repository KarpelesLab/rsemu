//! The device trait, its lifecycle, and the re-entrancy contract.
//!
//! A device is anything a machine is built from: RAM, a CPU, an interrupt
//! controller, a whole chipset. `ROADMAP.md` §4.4 fixes the shape; this module
//! is where the pieces built separately — [`space`](crate::core::space),
//! [`wire`](crate::core::wire), [`clock`](crate::core::clock),
//! [`props`](crate::core::props), [`state`](crate::core::state) — meet.
//!
//! # Two-phase construction
//!
//! `DeviceClass::construct` validates properties and allocates. [`Device::realize`]
//! performs every outward action: mapping regions, connecting wires, attaching
//! to buses. **Nothing observable happens before realize**, which is what lets a
//! resolver build the whole graph and then fail cleanly without having half-wired
//! a machine.
//!
//! # The re-entrancy contract
//!
//! Device methods take `&self`, so state lives behind interior mutability. The
//! naive rule — "never hold a lock across a call into another device" — is
//! unimplementable: an MMIO write to a DMA controller's GO register *must* issue
//! reads while the handler runs, and a PCI config write that moves a BAR remaps
//! memory from inside the device's own write path. Forbidding that forbids the
//! NES.
//!
//! The rule instead: mutate your own state in a short critical section, release
//! it, and *then* act outward — or push the action onto a [`Deferred`] queue and
//! let the caller run it once your handler has returned. See [`Deferred`].

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed, LazyHandle};
use crate::core::space::{RegionRef, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::wire::{IntAck, WireId, WireSink, WireSource};

/// How deep a reset goes.
///
/// Distinct kinds because devices genuinely differ: an RTC keeps time across a
/// warm reset but not a cold one, and a bus reset touches only what hangs off
/// that bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ResetKind {
    /// Power-on. Every register returns to its documented reset value.
    Cold,
    /// A reset line pulse. Battery-backed and always-on state survives.
    Warm,
    /// A bus-level reset affecting only devices on that bus.
    Bus,
}

/// One declared property of a device class.
///
/// The class declares these once; the registry uses them for `rsemu describe`,
/// and the machine-description validator uses them to reject a typo'd property
/// before the device is ever constructed.
#[derive(Debug, Clone, Copy)]
pub struct PropertySpec {
    /// Property name as it appears in a `.machine` file.
    pub name: &'static str,
    /// What kind of value it takes.
    pub kind: ValueKind,
    /// Whether omitting it is an error.
    pub required: bool,
    /// One line explaining what it does, for `rsemu describe`.
    pub summary: &'static str,
}

/// The static description of a kind of device, plus its constructor.
///
/// Held by value in a `static`, so a class costs nothing at runtime and the
/// registry is a list of references rather than an allocation per class.
pub struct DeviceClass {
    /// Dotted name used in machine files and by the registry (`pci.nvme`).
    pub name: &'static str,
    /// Snapshot version for this class's chunk encoding.
    ///
    /// Bump when `save`/`load` change shape, and register a migration step
    /// (`state::Migrations`) — a version bump with no migration makes every
    /// existing snapshot unloadable.
    pub version: u32,
    /// One line for `rsemu devices`.
    pub summary: &'static str,
    /// Every property this class accepts.
    pub properties: &'static [PropertySpec],
    /// Validate properties and allocate. Performs no outward action.
    pub construct: fn(&Props) -> Result<Box<dyn Device>>,
}

impl fmt::Debug for DeviceClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The `construct` fn pointer is not Debug and would be noise anyway.
        f.debug_struct("DeviceClass")
            .field("name", &self.name)
            .field("version", &self.version)
            .field("properties", &self.properties.len())
            .finish()
    }
}

/// A device's input pin: the sink to deliver to, and the line the device knows
/// it by.
///
/// The `Arc` is the device's own — **a net holds only a weak reference to its
/// sinks**, because the machine owns devices and a wire merely refers to them.
/// That is `ROADMAP.md` §4.3's weak edge, and it is what stops an IRQ/ack loop
/// leaking.
pub struct SinkPin {
    /// The sink to deliver to.
    pub sink: Arc<dyn WireSink>,
    /// Which of the device's own input lines this is.
    pub line: u32,
}

impl fmt::Debug for SinkPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `WireSink` is a behaviour, not data; the line number is the part
        // worth printing.
        f.debug_struct("SinkPin").field("line", &self.line).finish()
    }
}

/// Anything a machine is built from.
///
/// `Send + Sync` from the first commit rather than once threading "is needed":
/// retrofitting it is a rewrite, and the threading mode is a machine property
/// (`ROADMAP.md` §0).
pub trait Device: Send + Sync + fmt::Debug {
    /// The class this device is an instance of.
    fn class(&self) -> &'static DeviceClass;

    /// Perform every outward action: map regions, connect wires, attach to buses.
    ///
    /// Nothing observable may happen before this is called.
    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()>;

    /// Undo [`realize`](Device::realize): unmap, disconnect, cancel pending work.
    ///
    /// Defaults to doing nothing, which is correct for a device that maps and
    /// wires nothing. A device that *does* must override it or hot-unplug leaks
    /// a mapping (`ROADMAP.md` §4.4).
    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    /// Return to a documented reset state.
    fn reset(&self, kind: ResetKind);

    /// Serialize architectural state — never derived caches (invariant 3).
    ///
    /// Defaults to writing nothing, for a genuinely stateless device. Anything
    /// with state must override this *and* [`load`](Device::load), and ship the
    /// round-trip test that proves they agree (invariant 6).
    fn save(&self, _w: &mut ChunkWriter<'_>) -> Result<()> {
        Ok(())
    }

    /// Restore what [`save`](Device::save) wrote.
    fn load(&self, _r: &mut ChunkReader<'_>) -> Result<()> {
        Ok(())
    }

    // ---------------------------------------------------------------------
    // How this device connects to the rest of the machine.
    //
    // These live on `Device` rather than in a second trait beside it. The
    // machine layer originally had to invent one, because there is no route
    // from a `dyn Device` to another trait object without `Any` in the
    // supertrait chain — and the cost was that every class had to be
    // registered twice, once for construction and once for connection, with
    // nothing keeping the two tables in step. Every method below is defaulted,
    // so a device implements only what it actually has.
    // ---------------------------------------------------------------------

    /// A region a `map` statement may name.
    ///
    /// `""` is the device's whole aperture, which is what
    /// `map cpubus 0 size 2K = wram` asks for.
    fn region(&self, _name: &str) -> Option<RegionRef> {
        None
    }

    /// The sink for input pin `port`, and the line the device knows it by.
    ///
    /// `sources` is every id that will drive this pin's net. §4.3 requires a
    /// sink to track *which* sources are asserting — that is what makes
    /// wired-OR correct when one source deasserts — and a `FanIn` is told its
    /// sources at construction, while a device is constructed long before any
    /// `WireId` exists. This call is the only moment both are known.
    fn sink(&self, _port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        None
    }

    /// Take the output port for pin `port`.
    ///
    /// Called once per net the pin drives; a pin driving two nets is handed two
    /// sources and must drive both.
    ///
    /// # Errors
    ///
    /// If the device drives no such pin.
    fn connect(&self, port: &str, _source: WireSource) -> Result<()> {
        Err(Error::Config {
            at: port.to_string(),
            message: "this device drives no such pin".to_string(),
        })
    }

    /// The acknowledge handler this device offers on output pin `port`.
    ///
    /// **The device must own the `Arc`**, exactly as it owns the one behind a
    /// [`SinkPin`]: what reaches the sink is a `Weak`, so a handler built on
    /// the fly would arrive already dead.
    ///
    /// An interrupt controller returns one here; everything else returns
    /// `None`, which is the answer for a line that carries a level and nothing
    /// more. The realizer asks every driver on a net and hands what it gets to
    /// that net's sinks — see [`IntAck`] for why the vector travels with the
    /// wire rather than through a device handle.
    fn int_ack(&self, _port: &str) -> Option<Arc<dyn IntAck>> {
        None
    }

    /// Told what answers the acknowledge cycle on input pin `port`.
    ///
    /// A CPU with a vectored interrupt input keeps this and calls
    /// [`IntAck::acknowledge`] when it takes the interrupt. The reference is
    /// **weak**: the machine owns both devices, and a CPU that kept its
    /// controller alive would close a cycle nothing could drop.
    ///
    /// Called once per driver on the net, before reset and before the realize
    /// sweep.
    fn attach_int_ack(&self, _port: &str, _ack: Weak<dyn IntAck>) {}

    /// Announce the level `port` idles at — the realize sweep (§4.3).
    ///
    /// A device whose outputs idle low may ignore this, since a fresh net is
    /// already low. An inverter, or anything whose output is a function of its
    /// inputs, must drive here or the machine comes up inconsistent.
    fn announce(&self, _port: &str) {}

    /// Whether this device forwards a level within one instant, with no state.
    ///
    /// Feeds the realize ordering: a cycle of *combinational* devices has no
    /// topological order and is rejected, while a cycle through a stateful one
    /// is an ordinary handshake (§4.3). A device is sequential until it says
    /// otherwise, which is the safe default — claiming to be combinational
    /// when you are not turns a legitimate handshake into a rejected machine.
    fn combinational(&self) -> bool {
        false
    }

    /// Whether the scheduler should hand this device execution budgets.
    ///
    /// True for a CPU, a DMA engine, a coprocessor. Such a device needs a clock
    /// domain, and realize refuses one without.
    fn is_runnable(&self) -> bool {
        false
    }

    /// Run until the budget is exhausted, and report what was consumed (§4.2).
    ///
    /// Takes `&self`: a device is shared, and holds its state behind interior
    /// mutability.
    fn run(&self, _budget: Budget) -> Consumed {
        Consumed::default()
    }

    /// An event this device posted has come due.
    ///
    /// Outward actions — driving a wire, starting a burst — go on `deferred`,
    /// which the caller drains the moment this returns (§4.7).
    fn event(&self, _token: u64, _deferred: &mut Deferred) {}

    // ---------------------------------------------------------------------
    // Lazily advanced devices — sync-on-access (`ROADMAP.md` §4.2).
    //
    // The queue handles *scheduled* behaviour; it cannot handle *sampled*
    // behaviour. A 6502 reads `$2002` at an arbitrary cycle and the PPU has to
    // be at exactly that dot. So a device may declare that it holds its own
    // tick and is caught up before it is touched, and the machine layer
    // registers it with the scheduler on its clock domain.
    //
    // Every method here takes `&self`, like the rest of the surface: a device
    // is shared and its state lives behind interior mutability. The scheduler's
    // `LazyDevice` takes `&mut self` instead, and the machine layer adapts
    // between the two — which is also where the `&mut` exclusivity that makes
    // `advance_to` non-re-entrant comes from.
    //
    // # What an implementation must guarantee
    //
    // [`current_tick`](Device::current_tick) and
    // [`next_event_tick`](Device::next_event_tick) are called with the
    // scheduler's slot lock held, at
    // [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF) — the rank nothing
    // nests under. **Neither may take a lock**: publish the two numbers into
    // atomics as the device advances. `advance_to` is called with no lock held
    // at all, so it is free to take its own and to reach its own bus.
    // ---------------------------------------------------------------------

    /// Whether this device advances only when somebody looks at it.
    ///
    /// A device that says yes needs a clock domain, and realize refuses one
    /// without — its tick is counted in that domain and catch-up has no target
    /// otherwise.
    fn is_lazy(&self) -> bool {
        false
    }

    /// The tick, in the device's own clock domain, that it has simulated up to.
    ///
    /// Must not take a lock — see the note above.
    fn current_tick(&self) -> u64 {
        0
    }

    /// Simulate forward until [`current_tick`](Device::current_tick) reaches
    /// `tick`.
    ///
    /// Never called with a tick in the past, and never with any lock held, so
    /// an implementation may take its own state lock and reach its own bus.
    /// Running backwards is a no-op, not an error.
    fn advance_to(&self, tick: u64) {
        let _ = tick;
    }

    /// The device's own next internal event, if it has one.
    ///
    /// Catch-up never crosses it: past that tick the device's behaviour
    /// changes, and simulating through it in one step would compute the wrong
    /// answer. It is also what the machine layer bounds a quantum by, so that a
    /// CPU is not let run thousands of cycles past the dot an NMI was raised
    /// on. `None` means "nothing pending" and catch-up runs to the present.
    ///
    /// **Must be strictly greater than [`current_tick`](Device::current_tick)**
    /// or catch-up makes no progress and the device stalls where it stands.
    /// Must not take a lock — see the note above.
    fn next_event_tick(&self) -> Option<u64> {
        None
    }

    /// Told the handle that catches this device up.
    ///
    /// This is how sync-on-access reaches the code that answers an access:
    /// `MemOps::read` takes `&self` and runs several frames below whoever owns
    /// the scheduler, so the device keeps the handle and calls
    /// [`LazyHandle::sync`] at the top of its own read and write paths. Called
    /// once, by the machine layer, after the device is registered.
    ///
    /// A device that is not lazy never gets one.
    fn attach_lazy(&self, handle: LazyHandle) {
        let _ = handle;
    }
}

/// A device that performs its own accesses: DMA engines, bus masters, host
/// controllers, and every CPU.
///
/// A device that can only *respond* cannot model NES OAM DMA, an 8237, a PCI
/// bus master, virtio descriptor fetch, or a USB controller walking transfer
/// rings — which is most devices from the PC phase onward, and two in the NES.
/// The requester id travels in `MemAttrs` so an IOMMU can translate it.
pub trait Initiator {
    /// This initiator's identity, as it appears in `MemAttrs::requester`.
    fn requester(&self) -> RequesterId;
}

/// An action a device wants performed *after* its handler returns.
///
/// The payload is deliberately a closure rather than an enum of known actions:
/// the core must not know what devices exist (invariant 1), and a DMA burst, a
/// wire change and a remap have nothing in common but their timing.
type Action = Box<dyn FnOnce() + Send>;

/// The deferred-action queue that makes the re-entrancy contract workable.
///
/// A handler that needs to act outward — start a DMA burst, drop an interrupt
/// line, remap a BAR — pushes the action here instead of doing it inline. The
/// caller drains the queue once the handler has returned and its critical
/// section is released.
///
/// This is what turns re-entrancy from an accident into a decision. Without it,
/// the same code deadlocks under the `native-std` sync backend and panics under
/// `single`, which is a miserable way to discover a design problem.
#[derive(Default)]
pub struct Deferred {
    actions: Vec<Action>,
    /// Set while [`drain`](Deferred::drain) is running, so an action that
    /// defers further work is queued rather than recursing.
    draining: bool,
}

impl fmt::Debug for Deferred {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Deferred")
            .field("pending", &self.actions.len())
            .field("draining", &self.draining)
            .finish()
    }
}

impl Deferred {
    /// An empty queue.
    pub fn new() -> Deferred {
        Deferred {
            actions: Vec::new(),
            draining: false,
        }
    }

    /// Queue an action to run once the current handler has returned.
    pub fn push(&mut self, action: impl FnOnce() + Send + 'static) {
        self.actions.push(Box::new(action));
    }

    /// Whether anything is queued.
    pub fn is_empty(&self) -> bool {
        self.actions.is_empty()
    }

    /// How many actions are queued.
    pub fn len(&self) -> usize {
        self.actions.len()
    }

    /// Run every queued action, in the order they were pushed.
    ///
    /// Actions queued *by* an action run in the same drain, after the ones
    /// already present — the queue is drained iteratively rather than
    /// recursively, so a device that defers work from a deferred action costs
    /// no stack. Returns the number of actions run.
    ///
    /// Re-entering `drain` is a no-op returning zero, which keeps a nested call
    /// from running actions out of order.
    pub fn drain(&mut self) -> usize {
        if self.draining {
            return 0;
        }
        self.draining = true;
        let mut ran = 0;
        // Not a `for` over `actions`: an action may push more, and those must
        // run in this drain rather than being stranded until the next one.
        while !self.actions.is_empty() {
            let batch = core::mem::take(&mut self.actions);
            for action in batch {
                action();
                ran += 1;
            }
        }
        self.draining = false;
        ran
    }
}

/// What a device is handed during [`Device::realize`].
///
/// Holds the instance's identity and its deferred queue. Access to address
/// spaces, the clock forest and wires is added as the machine assembly layer
/// lands; realize is where those connections are made, so this is the type that
/// grows.
#[derive(Debug)]
pub struct RealizeCtx<'a> {
    path: &'a str,
    requester: RequesterId,
    deferred: &'a mut Deferred,
}

impl<'a> RealizeCtx<'a> {
    /// Build a context for the instance at `path`.
    pub fn new(path: &'a str, requester: RequesterId, deferred: &'a mut Deferred) -> Self {
        RealizeCtx {
            path,
            requester,
            deferred,
        }
    }

    /// The instance path of the device being realized (`cpu`, `pci.0.nvme`).
    ///
    /// This is the snapshot chunk key (`ROADMAP.md` §4.5), so it is stable for
    /// the life of the machine.
    pub fn path(&self) -> &str {
        self.path
    }

    /// This device's requester id, for accesses it initiates.
    pub fn requester(&self) -> RequesterId {
        self.requester
    }

    /// Queue an action to run after realize completes.
    pub fn defer(&mut self, action: impl FnOnce() + Send + 'static) {
        self.deferred.push(action);
    }

    /// A realize-time failure that names the instance.
    ///
    /// "cannot map at 0x2000" is unactionable in a machine with forty devices;
    /// the path is what makes it a bug report.
    pub fn error(&self, message: impl Into<String>) -> Error {
        Error::Config {
            at: String::from(self.path),
            message: message.into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::sync::Arc;
    use core::sync::atomic::{AtomicU32, Ordering};

    #[test]
    fn deferred_actions_run_in_push_order() {
        let log = Arc::new(AtomicU32::new(0));
        let mut q = Deferred::new();
        for i in 1..=3u32 {
            let log = Arc::clone(&log);
            // Base-10 shifting records order, not just occurrence.
            q.push(move || {
                log.store(log.load(Ordering::Relaxed) * 10 + i, Ordering::Relaxed);
            });
        }
        assert_eq!(q.len(), 3);
        assert_eq!(q.drain(), 3);
        assert_eq!(log.load(Ordering::Relaxed), 123);
        assert!(q.is_empty());
    }

    #[test]
    fn an_action_may_queue_more_work_without_recursing() {
        // The case the contract exists for: a deferred DMA completion raising an
        // interrupt, which defers again. It must run in this drain, not the next.
        let count = Arc::new(AtomicU32::new(0));
        let mut q = Deferred::new();
        let c = Arc::clone(&count);
        q.push(move || {
            c.fetch_add(1, Ordering::Relaxed);
        });
        assert_eq!(q.drain(), 1);
        assert_eq!(count.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn draining_an_empty_queue_is_free() {
        let mut q = Deferred::new();
        assert_eq!(q.drain(), 0);
        assert!(q.is_empty());
    }

    #[test]
    fn realize_errors_name_the_instance() {
        let mut q = Deferred::new();
        let ctx = RealizeCtx::new("pci.0.nvme", RequesterId(7), &mut q);
        assert_eq!(ctx.path(), "pci.0.nvme");
        assert_eq!(ctx.requester(), RequesterId(7));
        let e = ctx.error("cannot map at 0x2000").to_string();
        assert!(e.contains("pci.0.nvme"), "{e}");
        assert!(e.contains("cannot map"), "{e}");
    }

    #[test]
    fn a_context_can_defer_during_realize() {
        let ran = Arc::new(AtomicU32::new(0));
        let mut q = Deferred::new();
        {
            let mut ctx = RealizeCtx::new("cpu", RequesterId::ANONYMOUS, &mut q);
            let r = Arc::clone(&ran);
            ctx.defer(move || {
                r.fetch_add(1, Ordering::Relaxed);
            });
            // Nothing has happened yet — that is the whole point of deferring.
            assert_eq!(ran.load(Ordering::Relaxed), 0);
        }
        q.drain();
        assert_eq!(ran.load(Ordering::Relaxed), 1);
    }
}
