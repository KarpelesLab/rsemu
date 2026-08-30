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
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RequesterId;
use crate::core::state::{ChunkReader, ChunkWriter};

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
