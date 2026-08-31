//! The system controller: how a guest turns the machine off.
//!
//! A one-register device. The guest writes a magic value and the machine stops,
//! restarts, or stops with an exit code. Trivial hardware, and the only way a
//! headless test ever ends on purpose rather than on a timeout.
//!
//! # The register
//!
//! One 32-bit write-only register at offset 0. The low half is the command and
//! the high half is a payload:
//!
//! ```text
//!   0x0000_5555   pass:  stop the machine, successfully
//!   0xcccc_3333   fail:  stop the machine, reporting 0xcccc
//!   0x0000_7777   reset: pulse the reset line
//! ```
//!
//! These are the values RISC-V boards conventionally use, and the device tree
//! this board generates names it `syscon` with `syscon-poweroff` and
//! `syscon-reboot` nodes pointing at it — which is how Linux finds it without
//! any board-specific code.
//!
//! # How the host hears about it
//!
//! Through a **named signal**, the same seam as
//! [`chardev::ports`](crate::host::chardev::ports) and for the same reason: a
//! machine file can hand a device a name but not a host object. The machine
//! file writes `signal = "power"`, the host calls [`signals::open`] with the
//! same name, and the two meet. A machine that names no signal still works —
//! it just has nobody listening.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

use super::dt::{DtSource, NodeKind, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "riscv.syscon";

/// How much address space the register occupies.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// The command that stops the machine successfully.
pub const CMD_PASS: u16 = 0x5555;
/// The command that stops the machine with the payload as an exit code.
pub const CMD_FAIL: u16 = 0x3333;
/// The command that pulses the reset line.
pub const CMD_RESET: u16 = 0x7777;

/// What a guest asked the machine to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// Stop, successfully.
    Poweroff,
    /// Stop, reporting this code.
    Fail(u16),
    /// Start again from the reset vector.
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

/// The process-wide table of named power signals.
///
/// See the module docs for why a name is the only thing that can travel from a
/// machine file into a device constructor, and
/// [`chardev`](crate::host::chardev) for the precedent.
pub mod signals {
    use super::Signal;
    use alloc::collections::BTreeMap;
    use alloc::string::{String, ToString};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::sync::{LockRank, Mutex};

    /// Name to signal, in name order rather than hash order (`CLAUDE.md`,
    /// determinism).
    static TABLE: Mutex<BTreeMap<String, Arc<Signal>>> =
        Mutex::with_rank(LockRank::LEAF, BTreeMap::new());

    /// The signal called `name`, creating it if this is the first mention.
    #[must_use]
    pub fn open(name: &str) -> Arc<Signal> {
        let mut table = TABLE.lock();
        if let Some(signal) = table.get(name) {
            return Arc::clone(signal);
        }
        let signal = Arc::new(Signal::new());
        table.insert(name.to_string(), Arc::clone(&signal));
        signal
    }

    /// The signal called `name`, if it has been opened.
    #[must_use]
    pub fn get(name: &str) -> Option<Arc<Signal>> {
        TABLE.lock().get(name).map(Arc::clone)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(name: &str) -> bool {
        TABLE.lock().remove(name).is_some()
    }

    /// Every open signal's name, in order.
    #[must_use]
    pub fn names() -> Vec<String> {
        TABLE.lock().keys().cloned().collect()
    }
}

/// The register, as something an address space can dispatch to.
struct Registers {
    signal: Arc<Signal>,
    signal_name: String,
    /// The reset output, at [`LockRank::LEAF`].
    out: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("signal", &self.signal_name)
            .field("pending", &self.signal.peek())
            .finish()
    }
}

/// The system controller.
#[derive(Debug)]
pub struct Syscon {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Syscon {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, or if one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Syscon> {
        let mut r = props.reader();
        let name = r.or("signal", String::from("power"))?;
        r.finish()?;
        Ok(Syscon::with_signal(signals::open(&name), name))
    }

    /// Build one against a signal the caller already holds.
    #[must_use]
    pub fn with_signal(signal: Arc<Signal>, signal_name: String) -> Syscon {
        let regs = Arc::new(Registers {
            signal,
            signal_name,
            out: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "riscv.syscon",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        super::dt::publish(&region, Arc::downgrade(&regs) as Weak<dyn DtSource>);
        Syscon { regs, region }
    }

    /// The signal a guest's requests land on.
    #[must_use]
    pub fn signal(&self) -> &Arc<Signal> {
        &self.regs.signal
    }

    /// The name the signal was opened under.
    #[must_use]
    pub fn signal_name(&self) -> &str {
        &self.regs.signal_name
    }
}

impl MemOps for Registers {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        // Write-only. Reads-as-zero rather than a fault: a `syscon` regmap
        // driver reads before it writes.
        dst.fill(0);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || offset != 0 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write here would stop the machine somebody is debugging.
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let command = value as u16;
        let payload = (value >> 16) as u16;
        match command {
            CMD_PASS => self.signal.raise(Request::Poweroff),
            CMD_FAIL => self.signal.raise(Request::Fail(payload)),
            CMD_RESET => {
                self.signal.raise(Request::Reboot);
                // A reset is a pulse, not a level: the line goes high and comes
                // straight back, which is what a `wire.level-to-edge` would
                // otherwise have to be inserted to produce.
                let out = self.out.lock().clone();
                if let Some(out) = out {
                    out.pulse(Level::High);
                }
            }
            // Anything else is a value this controller does not implement, and
            // ignoring it is what hardware does with an unrecognised command.
            _ => {}
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        NodeSpec {
            kind: NodeKind::Syscon {
                poweroff: u32::from(CMD_PASS),
                reboot: u32::from(CMD_RESET),
            },
            name: "test",
            // `syscon` last, because that is the generic binding the poweroff
            // and reboot nodes look for.
            compatible: &["sifive,test1", "sifive,test0", "syscon"],
            cells: alloc::vec![("reg-io-width", alloc::vec![4])],
            strings: alloc::vec![],
            irq_wire: None,
        }
    }
}

/// The `riscv.syscon` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "system controller: a guest writes a magic value to power off, fail, or reboot",
    properties: &[PropertySpec {
        name: "signal",
        kind: ValueKind::Str,
        required: false,
        summary: "the named signal a request lands on (default \"power\")",
    }],
    construct: |props| Ok(Box::new(Syscon::new(props)?)),
};

impl Device for Syscon {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // A cold start clears a request left over from the run that asked for
        // this reset; a warm one does not, or a reboot would cancel itself.
        if kind == ResetKind::Cold {
            self.regs.signal.clear();
        }
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "reset" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a system controller drives one pin, `reset`"),
            });
        }
        *self.regs.out.lock() = Some(source);
        Ok(())
    }

    // No `save`/`load`: a pending power request belongs to the host session
    // rather than to the machine, and the register itself holds nothing.
}

impl Instance for Syscon {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Syscon::new(props)?)))
}

/// What the validator should know about `riscv.syscon`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("signal", ValueKind::Str))
        .region("")
        .region("regs")
        .port("reset", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn syscon() -> Syscon {
        Syscon::with_signal(Arc::new(Signal::new()), "test".to_string())
    }

    fn poke(s: &Syscon, value: u32) {
        s.regs
            .write(0, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    #[test]
    fn the_magic_values_are_the_three_requests() {
        let s = syscon();
        poke(&s, u32::from(CMD_PASS));
        assert_eq!(s.signal().take(), Some(Request::Poweroff));

        poke(&s, u32::from(CMD_RESET));
        assert_eq!(s.signal().take(), Some(Request::Reboot));

        poke(&s, (0xbeefu32 << 16) | u32::from(CMD_FAIL));
        assert_eq!(s.signal().take(), Some(Request::Fail(0xbeef)));
    }

    #[test]
    fn an_unrecognised_command_does_nothing() {
        let s = syscon();
        poke(&s, 0x1234);
        assert_eq!(s.signal().peek(), None);
    }

    #[test]
    fn the_first_request_wins() {
        // A shutdown path that writes the register twice must not have its
        // reason overwritten by its own second write.
        let s = syscon();
        poke(&s, (7u32 << 16) | u32::from(CMD_FAIL));
        poke(&s, u32::from(CMD_PASS));
        assert_eq!(s.signal().peek(), Some(Request::Fail(7)));
    }

    #[test]
    fn a_debug_write_is_refused_and_a_read_is_zero() {
        let s = syscon();
        assert!(
            s.regs
                .write(0, &u32::from(CMD_PASS).to_le_bytes(), MemAttrs::DEBUG)
                .is_err()
        );
        assert_eq!(s.signal().peek(), None);
        let mut bytes = [0xffu8; 4];
        s.regs.read(0, &mut bytes, MemAttrs::DEBUG).unwrap();
        assert_eq!(bytes, [0; 4]);
    }

    #[test]
    fn only_an_aligned_word_at_offset_zero_is_a_command() {
        let s = syscon();
        assert!(s.regs.write(4, &[0u8; 4], MemAttrs::DEFAULT).is_err());
        assert!(s.regs.write(0, &[0u8; 2], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_name_reaches_the_same_signal_from_both_ends() {
        let device_end = signals::open("test.syscon.shared");
        let host_end = signals::open("test.syscon.shared");
        device_end.raise(Request::Poweroff);
        assert_eq!(host_end.take(), Some(Request::Poweroff));
        assert!(signals::names().iter().any(|n| n == "test.syscon.shared"));
        assert!(signals::close("test.syscon.shared"));
        assert!(signals::get("test.syscon.shared").is_none());
    }

    #[test]
    fn a_cold_reset_clears_a_pending_request_and_a_warm_one_does_not() {
        let s = syscon();
        poke(&s, u32::from(CMD_RESET));
        s.reset(ResetKind::Warm);
        assert_eq!(
            s.signal().peek(),
            Some(Request::Reboot),
            "the reboot stands"
        );
        s.reset(ResetKind::Cold);
        assert_eq!(s.signal().peek(), None);
    }
}
