//! Where a guest's `PSCI_SYSTEM_OFF` lands.
//!
//! A device with **no registers and no address**. On a RISC-V `virt` board the
//! equivalent is `riscv.syscon`, a magic value written to a word of MMIO; on
//! an AArch64 board there is no such word, because the architecture already
//! has a firmware call for it. `SYSTEM_OFF` and `SYSTEM_RESET` are `SMC`
//! instructions, they are serviced inside `cpu.arm.a64`, and what leaves the
//! core is a **wire** — which is the only mechanism the framework has for "a
//! device did something and another device must know".
//!
//! ```text
//!   wire cpu.poweroff -> pwr.off      ; SYSTEM_OFF
//!   wire cpu.reboot   -> pwr.reboot   ; SYSTEM_RESET
//!   wire pwr.reset    -> cpu.reset    ; and what a reboot actually does
//! ```
//!
//! So this object is the *board's* answer to a request the core relayed: one
//! board stops the process, another pulses reset, a third could cut power to
//! half of itself. Putting that decision in the core would have made every
//! AArch64 machine agree about it.
//!
//! # How the host hears about it
//!
//! Through a **named signal**, the same seam as
//! [`chardev::ports`](crate::host::chardev::ports) and for the same reason: a
//! machine file can hand a device a name but not a host object. The machine
//! file writes `signal = "power"`, the host calls [`signals::open`] with the
//! same name, and the two meet. A machine that names no signal still works —
//! it just has nobody listening.
//!
//! # Why this is not `dev::riscv::syscon`'s `Signal`
//!
//! It is the same idea and very nearly the same forty lines, and one copy
//! would be better. The other one lives under `dev/riscv/`, reachable only
//! from a build with `dev-riscv` on, and an AArch64 board that had to link a
//! PLIC and a CLINT to be able to switch itself off would contradict the
//! crate-shape rule. It is filed under its own [`HostKind`] — `power`, not
//! `signal` — deliberately: a kind's identity is its *name alone*, so two
//! modules sharing a name must agree about the type stored under it, and these
//! two do not. `src/dev/fdt.rs` and `src/dev/power.rs` are the same one-commit
//! hoist, and `docs/platforms/arm64-virt.md` records both.
//!
//! [`HostKind`]: crate::core::hosts::HostKind

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.power";

/// What a guest asked the machine to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// Stop, successfully — `PSCI_SYSTEM_OFF`.
    Poweroff,
    /// Start again from the reset vector — `PSCI_SYSTEM_RESET`.
    Reboot,
}

/// A place a guest's power request lands, shared by name with the host.
#[derive(Debug, Default)]
pub struct Signal {
    pending: Mutex<Option<Request>>,
}

impl Signal {
    /// A signal with nothing pending.
    #[must_use]
    pub fn new() -> Signal {
        Signal {
            pending: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// What the guest asked for, if anything, leaving it in place.
    #[must_use]
    pub fn peek(&self) -> Option<Request> {
        *self.pending.lock()
    }

    /// What the guest asked for, clearing it.
    pub fn take(&self) -> Option<Request> {
        self.pending.lock().take()
    }

    /// Record a request. The *first* one wins: a machine that has already been
    /// told to power off is not then told to reboot by its own shutdown path.
    pub fn raise(&self, request: Request) {
        let mut pending = self.pending.lock();
        if pending.is_none() {
            *pending = Some(request);
        }
    }

    /// Forget any pending request.
    pub fn clear(&self) {
        *self.pending.lock() = None;
    }
}

/// The build's named power signals.
pub mod signals {
    use super::Signal;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;

    /// The kind a power signal is filed under in a build's [`HostObjects`].
    ///
    /// A *rendezvous*: two ends of one build finding each other. Nothing
    /// non-deterministic crosses in here, so a recorded build does not need a
    /// channel for it (`core::hosts`).
    pub const KIND: HostKind = HostKind::rendezvous("power");

    /// The power signal `name` refers to in `hosts`, creating it on first
    /// mention.
    ///
    /// The **host** side of the rendezvous: called before the host starts
    /// watching for a poweroff, or after the build to pick up what the device
    /// opened.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Signal>> {
        hosts.open(KIND, name, Signal::new)
    }

    /// The same signal, from a device's `new(props)` — acquiring a host object
    /// is allocation, and [`core::hosts`](crate::core::hosts) argues why.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Signal>> {
        props.host(KIND, name, Signal::new)
    }

    /// The power signal called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Signal>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }
}

/// The half of [`Power`] a pin can hold, so a pin does not have to borrow the
/// device it belongs to.
#[derive(Debug)]
struct Core {
    signal: Arc<Signal>,
    /// The reset output, at [`LockRank::LEAF`].
    out: Mutex<Option<WireSource>>,
}

impl Core {
    /// Act on a request. Called from a wire callback, with nothing else held.
    fn request(&self, what: Request) {
        self.signal.raise(what);
        if what == Request::Reboot {
            // A pulse, not a level: a reset line that stayed asserted would
            // hold the core in reset forever.
            let out = self.out.lock().clone();
            if let Some(out) = out {
                out.set(Level::High);
                out.set(Level::Low);
            }
        }
    }
}

/// The board's power controller.
#[derive(Debug)]
pub struct Power {
    core: Arc<Core>,
    signal_name: String,
    /// The sinks handed out by [`Device::sink`], kept alive here — a net holds
    /// only a weak reference to a sink.
    pins: Mutex<Vec<Arc<RequestPin>>>,
}

impl Power {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Power> {
        let mut r = props.reader();
        let name = r.or("signal", String::from("power"))?;
        r.finish()?;
        Ok(Power::with_signal(signals::attach(props, &name)?, name))
    }

    /// Build one against a signal the caller already has.
    #[must_use]
    pub fn with_signal(signal: Arc<Signal>, signal_name: String) -> Power {
        Power {
            core: Arc::new(Core {
                signal,
                out: Mutex::with_rank(LockRank::LEAF, None),
            }),
            signal_name,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The signal a guest's requests land on.
    #[must_use]
    pub fn signal(&self) -> &Arc<Signal> {
        &self.core.signal
    }

    /// The name the signal was opened under.
    #[must_use]
    pub fn signal_name(&self) -> &str {
        &self.signal_name
    }
}

/// One of the power controller's inputs, as something a wire can drive.
#[derive(Debug)]
pub struct RequestPin {
    owner: Arc<Core>,
    what: Request,
    inputs: FanIn,
}

impl WireSink for RequestPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // On the rising edge only: the core pulses the line for one request,
        // and a level that stayed high must not mean "power off repeatedly".
        if self.inputs.resolve(Resolve::Or).is_high() {
            self.owner.request(self.what);
        }
    }
}

/// The `arm.power` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "where PSCI SYSTEM_OFF and SYSTEM_RESET land: a named host signal and a reset pin",
    properties: &[PropertySpec {
        name: "signal",
        kind: ValueKind::Str,
        required: false,
        summary: "the named host signal a request lands on (default \"power\")",
    }],
    construct: |props| Ok(Box::new(Power::new(props)?)),
};

impl Device for Power {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A reset clears a pending request: the machine has been restarted, so
        // whatever it was asked to do has happened.
        self.core.signal.clear();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let what = match port {
            "off" => Request::Poweroff,
            "reboot" => Request::Reboot,
            _ => return None,
        };
        let pin = Arc::new(RequestPin {
            owner: Arc::clone(&self.core),
            what,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "reset" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a power controller drives one pin, `reset`"),
            });
        }
        *self.core.out.lock() = Some(source);
        Ok(())
    }

    // No `save`/`load`: a pending request is a message to the host, not guest
    // state, and a snapshot that carried one would power the machine off again
    // when it was restored.
}

impl Instance for Power {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Power::new(props)?)))
}

/// What the validator should know about `arm.power`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("signal", ValueKind::Str))
        .port("off", PortDir::In)
        .port("reboot", PortDir::In)
        .port("reset", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    fn power() -> Power {
        Power::with_signal(Arc::new(Signal::new()), String::from("test"))
    }

    /// Drive `port` high, the way a wire from the core would.
    fn pulse(p: &Power, port: &str) {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let pin = p.sink(port, &[id]).expect("the controller has that pin");
        pin.sink.set_level(id, 0, Level::High);
        pin.sink.set_level(id, 0, Level::Low);
    }

    #[test]
    fn a_poweroff_request_reaches_the_signal() {
        let p = power();
        assert_eq!(p.signal().peek(), None);
        pulse(&p, "off");
        assert_eq!(p.signal().peek(), Some(Request::Poweroff));
    }

    #[test]
    fn the_first_request_wins() {
        // A shutdown path that asks for a reboot after asking to power off
        // must not get one.
        let p = power();
        pulse(&p, "off");
        pulse(&p, "reboot");
        assert_eq!(p.signal().peek(), Some(Request::Poweroff));
    }

    #[derive(Debug, Default)]
    struct Probe {
        pulses: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            if level.is_high() {
                self.pulses.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    #[test]
    fn a_reboot_pulses_the_reset_line() {
        let p = power();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        p.connect("reset", WireSource::new(wire, id))
            .expect("a power controller drives reset");
        pulse(&p, "reboot");
        assert_eq!(probe.pulses.load(Ordering::Relaxed), 1);
        assert_eq!(p.signal().peek(), Some(Request::Reboot));
    }

    #[test]
    fn a_pin_this_device_does_not_have_is_refused() {
        let p = power();
        let ids = WireIdAllocator::new();
        assert!(p.sink("sleep", &[ids.alloc()]).is_none());
        assert!(
            p.connect("off", {
                let id = ids.alloc();
                let wire = Wire::builder().source(id).build_shared();
                WireSource::new(wire, id)
            })
            .is_err()
        );
    }

    #[test]
    fn a_reset_clears_a_pending_request() {
        let p = power();
        pulse(&p, "off");
        p.reset(ResetKind::Cold);
        assert_eq!(p.signal().peek(), None);
    }
}
