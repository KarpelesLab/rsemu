//! An external watchdog kicked by a pin.
//!
//! Many boards keep the SoC alive only while it proves it is: a supervisor
//! chip or a companion microcontroller watches one GPIO, expects it to toggle
//! every so often, and pulls the SoC's reset when it stops. The Alphard navi
//! is one (its sub-processor watches a GPIO the kernel toggles every few tens
//! of milliseconds, and the kernel's own restart path simply stops toggling
//! and spins); a hardware supervisor such as a TPS3823-class part is the same
//! device with the timeout set by a capacitor.
//!
//! # Behaviour
//!
//! * `kick` is the watched pin. **Any edge** restarts the countdown; a level
//!   that holds is not a kick, which is exactly why a crashed CPU with the pin
//!   stuck high still gets reset.
//! * The watchdog **arms on the first edge** it sees. Before that it waits
//!   forever: a board's boot ROM and early firmware may not kick at all, and a
//!   supervisor that fired during them would make the board unbootable.
//! * When `timeout` ticks of its clock pass with no edge, `reset` pulses
//!   (high, then low) and the watchdog disarms until the next edge — the
//!   rebooted firmware re-arms it by kicking.
//! * With `scope = "machine"` the expiry also resets **every device**, warm
//!   ([`MachineReset`](crate::core::device::MachineReset)): what a supervisor
//!   that holds a whole SoC in reset does, where a pin reaches only a core.
//! * With `log = "port"` every expiry is reported, one line, on that
//!   character port: a board that reboots has several ways to, and this says
//!   which one it was.
//!
//! The countdown is lazily advanced (`ROADMAP.md` §4.2): the device schedules
//! only its deadline and costs nothing between edges.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "watchdog.pin";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The watched input.
pub const KICK_PIN: &str = "kick";
/// The reset output, pulsed on expiry.
pub const RESET_PIN: &str = "reset";

/// No deadline.
const NEVER: u64 = u64::MAX;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The last level seen on `kick`, so a repeated level is not an edge.
    level: bool,
    /// The tick the reset fires at, or [`NEVER`] while disarmed.
    deadline: u64,
    /// How many times it has fired, for a test or a monitor.
    fired: u64,
}

struct Shared {
    timeout: u64,
    state: Mutex<State>,
    tick: AtomicU64,
    deadline: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
    out: Mutex<Option<WireSource>>,
    /// With `scope = "machine"`, the machine's reset request: the whole board
    /// resets, not only whatever `reset` is wired to.
    machine: Mutex<Option<Arc<crate::core::device::MachineReset>>>,
    whole_machine: bool,
    /// Where an expiry is reported, if anywhere.
    log: Option<Arc<dyn crate::host::chardev::CharDevice>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("timeout", &self.timeout)
            .field("tick", &self.tick.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Shared {
    fn pulse(&self) {
        let machine = self.machine.lock().clone();
        if let Some(machine) = machine {
            machine.request();
        }
        let out = self.out.lock().clone();
        if let Some(out) = out {
            out.set(Level::High);
            out.set(Level::Low);
        }
    }

    fn advance_to(&self, target: u64) {
        let fire = {
            let mut s = self.state.lock();
            let now = self.tick.load(Ordering::Relaxed);
            let fire = target > now && s.deadline != NEVER && target >= s.deadline;
            let at = s.deadline;
            if fire {
                s.deadline = NEVER;
                s.fired += 1;
            }
            self.tick.store(target.max(now), Ordering::Relaxed);
            self.deadline.store(s.deadline, Ordering::Relaxed);
            fire.then_some((at, s.fired))
        };
        if let Some((at, n)) = fire {
            if let Some(log) = &self.log {
                let line = alloc::format!(
                    "watchdog: no kick for {} ticks, reset at tick {at} (expiry {n})\n",
                    self.timeout
                );
                log.write(line.as_bytes());
            }
            self.pulse();
        }
    }

    fn kick(&self, high: bool) {
        // Catch the clock up first, so the new deadline counts from now and
        // not from wherever this device last looked.
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
        let mut s = self.state.lock();
        if s.level == high {
            return;
        }
        s.level = high;
        let now = self.tick.load(Ordering::Relaxed);
        s.deadline = now.saturating_add(self.timeout);
        self.deadline.store(s.deadline, Ordering::Relaxed);
    }
}

struct KickPin {
    shared: Arc<Shared>,
}

impl fmt::Debug for KickPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("KickPin")
    }
}

impl WireSink for KickPin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.shared.kick(level.is_high());
    }
}

/// A pin-kicked external watchdog.
#[derive(Debug)]
pub struct PinWatchdog {
    shared: Arc<Shared>,
    pins: Mutex<Vec<Arc<KickPin>>>,
}

impl PinWatchdog {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing or malformed `timeout`, or a property
    /// this class does not know.
    pub fn new(props: &Props) -> Result<PinWatchdog> {
        let mut r = props.reader();
        let timeout = r.or_range("timeout", 0u64, 1..=u64::MAX / 2)?;
        let scope = r.or_enum("scope", "pin", &["pin", "machine"])?;
        let log = r.optional_str("log")?.map(ToString::to_string);
        r.finish()?;
        let log = match log {
            Some(name) => Some(crate::host::chardev::ports::attach(props, &name)?
                as Arc<dyn crate::host::chardev::CharDevice>),
            None => None,
        };
        let mut w = PinWatchdog::with_timeout(timeout);
        if let Some(shared) = Arc::get_mut(&mut w.shared) {
            shared.whole_machine = scope == "machine";
            shared.log = log;
        }
        Ok(w)
    }

    /// A watchdog that fires `timeout` ticks after the last edge.
    #[must_use]
    pub fn with_timeout(timeout: u64) -> PinWatchdog {
        PinWatchdog {
            shared: Arc::new(Shared {
                timeout,
                state: Mutex::with_rank(
                    LockRank::DEVICE,
                    State {
                        level: false,
                        deadline: NEVER,
                        fired: 0,
                    },
                ),
                tick: AtomicU64::new(0),
                deadline: AtomicU64::new(NEVER),
                lazy: Mutex::with_rank(LockRank::LEAF, None),
                out: Mutex::with_rank(LockRank::LEAF, None),
                machine: Mutex::with_rank(LockRank::LEAF, None),
                whole_machine: false,
                log: None,
            }),
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// How many times the reset has fired.
    #[must_use]
    pub fn fired(&self) -> u64 {
        self.shared.state.lock().fired
    }
}

/// The `watchdog.pin` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an external watchdog: pulses `reset` when `kick` stops toggling for `timeout` ticks",
    properties: &[
        PropertySpec {
            name: "timeout",
            kind: ValueKind::Uint,
            required: true,
            summary: "ticks of the device's clock allowed between two edges on `kick`",
        },
        PropertySpec {
            name: "scope",
            kind: ValueKind::Str,
            required: false,
            summary: "\"pin\" (the default): pulse `reset`; \"machine\": also reset every device, warm",
        },
        PropertySpec {
            name: "log",
            kind: ValueKind::Str,
            required: false,
            summary: "a character port each expiry is reported on, one line",
        },
    ],
    construct: |props| Ok(Box::new(PinWatchdog::new(props)?)),
};

impl Device for PinWatchdog {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if self.shared.whole_machine {
            *self.shared.machine.lock() = Some(ctx.machine_reset()?);
        }
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // A cold start is a power-on: disarmed, as the supervisor is before
        // the first kick. A warm reset is usually this device's own doing,
        // and it disarmed itself when it fired.
        if kind == ResetKind::Cold {
            let mut s = self.shared.state.lock();
            s.deadline = NEVER;
            s.level = false;
            self.shared.deadline.store(NEVER, Ordering::Relaxed);
        }
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<crate::core::device::SinkPin> {
        if port != KICK_PIN {
            return None;
        }
        let pin = Arc::new(KickPin {
            shared: Arc::clone(&self.shared),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != RESET_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a pin watchdog drives one pin, `reset`"),
            });
        }
        *self.shared.out.lock() = Some(source);
        Ok(())
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let d = self.shared.deadline.load(Ordering::Relaxed);
        (d != NEVER).then_some(d)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.shared.state.lock();
        w.write_bool(s.level)?;
        w.write_u64(s.deadline)?;
        w.write_u64(s.fired)?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let s = State {
            level: r.read_bool()?,
            deadline: r.read_u64()?,
            fired: r.read_u64()?,
        };
        let tick = r.read_u64()?;
        *self.shared.state.lock() = s;
        self.shared.tick.store(tick, Ordering::Relaxed);
        self.shared.deadline.store(s.deadline, Ordering::Relaxed);
        Ok(())
    }
}

impl Instance for PinWatchdog {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(PinWatchdog::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("timeout", ValueKind::Uint).required())
        .prop(PropSchema::new("scope", ValueKind::Str).values(&["pin", "machine"]))
        .prop(PropSchema::new("log", ValueKind::Str))
        .port(KICK_PIN, PortDir::In)
        .port(RESET_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_expiry_is_reported_on_the_log_port() {
        let port = Arc::new(crate::host::chardev::CharPort::new());
        let mut w = PinWatchdog::with_timeout(100);
        Arc::get_mut(&mut w.shared).unwrap().log =
            Some(Arc::clone(&port) as Arc<dyn crate::host::chardev::CharDevice>);
        w.shared.kick(true);
        w.shared.advance_to(99);
        assert!(port.drain().is_empty());
        w.shared.advance_to(100);
        let line = String::from_utf8(port.drain()).unwrap();
        assert_eq!(
            line,
            "watchdog: no kick for 100 ticks, reset at tick 100 (expiry 1)\n"
        );
    }

    #[test]
    fn it_waits_for_the_first_kick_then_fires_when_the_kicks_stop() {
        let w = PinWatchdog::with_timeout(100);
        w.shared.advance_to(10_000);
        assert_eq!(w.fired(), 0, "disarmed until kicked");
        w.shared.kick(true);
        w.shared.advance_to(10_050);
        w.shared.kick(false);
        w.shared.advance_to(10_140);
        assert_eq!(w.fired(), 0, "kicked in time");
        w.shared.advance_to(10_150);
        assert_eq!(w.fired(), 1, "100 ticks after the last edge");
        w.shared.advance_to(20_000);
        assert_eq!(w.fired(), 1, "disarmed again after firing");
    }

    #[test]
    fn a_held_level_is_not_a_kick() {
        let w = PinWatchdog::with_timeout(100);
        w.shared.kick(true);
        w.shared.advance_to(90);
        w.shared.kick(true);
        w.shared.advance_to(100);
        assert_eq!(w.fired(), 1);
    }

    #[test]
    fn the_deadline_is_the_next_event() {
        let w = PinWatchdog::with_timeout(100);
        assert_eq!(w.next_event_tick(), None);
        w.shared.advance_to(5);
        w.shared.kick(true);
        assert_eq!(w.next_event_tick(), Some(105));
    }
}
