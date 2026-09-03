//! The xHCI controller as a **PCI function**: a Type 00h header, a base address
//! register and `INTA#`.
//!
//! # Why this file exists
//!
//! Every USB host controller in this tree was MMIO-attached only. That is right
//! for the parts they were written from — a dwc2 is an SoC block, a ChipIdea is
//! an SoC block, and `machines/xhci-mini.machine` maps the register block at an
//! address the board chose — and it is wrong for every PC. A PC guest does not
//! find a host controller at a known address: it enumerates the bus, looks for
//! a function whose class code is `0C0330h`, sizes that function's base address
//! register, places the register block wherever it likes, and takes the
//! interrupt off whichever router input the swizzle landed the pin on. Nothing
//! here could answer any of that, so no board in `machines/` had a display and
//! a USB port at once — which is exactly what
//! [`host::input::mouse::capture`](crate::host::input::mouse::capture) reports
//! when it says a machine with a pointer does not exist yet.
//!
//! # The split, and what the transport actually contributes
//!
//! [`Xhci`] is the controller as the specification defines it and knows nothing
//! about PCI; this file is the configuration face and knows nothing about
//! rings. It is the same division [`crate::dev::nvme`] draws between `ctrl.rs`
//! and its function, and it is falsifiable the same way: there is no TRB, no
//! context and no doorbell below, and `xhci.rs` names no configuration offset
//! and no [`Bdf`].
//!
//! The transport contributes exactly four things:
//!
//! * **The window.** BAR0 is a 64-bit non-prefetchable memory register
//!   (xHCI 1.2 §5.2, *PCI Local Bus Specification* Rev 2.1 §6.2.5.1). 64-bit
//!   because the specification names it that way, and because a controller
//!   placed above 4 GiB is the case a 32-bit-only model silently fails.
//! * **`COMMAND[2]`**, Bus Master Enable. A function that may not master the
//!   bus fetches **nothing** — not a command TRB, not a Device Context, not an
//!   Event Ring Segment Table entry — which is [`Xhci::set_master`], and which
//!   is asserted rather than assumed (`tests/xhci_pci.rs`).
//! * **`COMMAND[10]`**, Interrupt Disable (Rev 3.0 §6.2.2). Set, and the
//!   function drives no `INTx#` — while `STATUS[3]`, Interrupt Status
//!   (§6.2.3), still reports the condition, because the two are deliberately
//!   different questions.
//! * **The pin.** [`Intx`] publishes `INTA#` onto the fabric's shared,
//!   level-sensitive, open-drain net (Rev 2.1 §2.2.6) *and* brings the same pin
//!   out to the board as an ordinary wire, for a machine with no interrupt
//!   router.
//!
//! # How the engine's interrupt reaches the pin
//!
//! Through a [`Wire`], and that is not indirection for its own sake. [`Xhci`]
//! drives one level-triggered output and has to keep working on a board with no
//! PCI in it at all, so it cannot own an [`Intx`] the way
//! [`crate::dev::ahci`]'s HBA does — `dev-usb-xhci` would then require
//! `bus-pci`, and `xhci-mini` would link a whole fabric to wire one pin. A
//! `Wire` is the generic mechanism the core already has for "this device's
//! output is that device's input" (`CLAUDE.md`: generic first, specific
//! second), it holds no rank-checked lock across the delivery, and it puts the
//! masking exactly where the Command register is:
//!
//! ```text
//!   USBCMD.INTE & IMAN.IE & IMAN.IP ──► Wire ──► Function ──► Intx ─┬─► the PCI net, swizzled
//!            (xHCI 1.2 §4.17.3)                     │               └─► `irq`, off the card edge
//!                                                   └── COMMAND[10] masks; STATUS[3] does not
//! ```
//!
//! The sink is held **weakly** by the wire, because the function holds the
//! engine and the engine holds the wire: a strong edge would close a cycle
//! nothing collects, which is the leak [`crate::bus::pci::ConfigPorts`] records
//! having been found twice by LeakSanitizer.
//!
//! # Acknowledging an interrupt does not cost a fourth write
//!
//! `xhci.rs` documents the three the specification fixes the order of —
//! `USBSTS.EINT`, then `ERDP` with `EHB`, then `IMAN.IP`. A PCI attachment adds
//! two register *bits* around them and **neither joins that sequence**:
//! `COMMAND[10]` gates the pin without touching any of the state above, and
//! `STATUS[3]` is read-only and reports the condition whatever the gate says. A
//! driver that masked with `COMMAND[10]` and expected `STATUS[3]` to follow
//! would poll forever, so `tests/xhci_pci.rs` asserts both directions.
//!
//! # `MemAttrs::debug`
//!
//! A configuration **read** has no side effects: `STATUS[3]` is derived from
//! the level the engine published into an atomic, so reading it acknowledges
//! nothing. A configuration **write** is refused outright, for the reason
//! [`crate::dev::nvme`]'s is — it would move the register block out from under
//! a driver mid-transfer, or switch off the bus mastering a transfer in flight
//! depends on. The register block behind the BAR keeps its own rules, which are
//! `xhci.rs`'s.
//!
//! # Sources
//!
//! The **xHCI 1.2c specification** (Intel, document 868295) §5.2, its PCI
//! configuration registers, for the base address register and the class code;
//! the ***PCI Local Bus Specification* Rev 2.1** §2.2.6 for the `INTx#` pins,
//! §6.1 and §6.2 for the Type 00h header, §6.2.2 for the Command register,
//! §6.2.4 for the Interrupt Pin register and §6.2.5.1 for base addresses and
//! the sizing read-back; **Rev 3.0** §6.2.2 for Interrupt Disable and §6.2.3
//! for Interrupt Status; and the *PCI Code and ID Assignment Specification* for
//! the encoding of `0C0330h` — base class `0Ch` serial bus controller,
//! sub-class `03h` USB, programming interface `30h` XHCI. No emulator source
//! and no operating system's xHCI driver was consulted (`ROADMAP.md` §1).

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use core::fmt;

use super::{MAX_SLOTS, Params, REGISTER_BYTES, STATE_VERSION, Xhci, register_region};

use crate::bus::pci::{Bar, Bars, Bdf, ConfigSpace, Intx, IntxPin, PciBus, PciFunction, config};
use crate::bus::usb::{MAX_PORTS, UsbBus};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{AddressSpace, MemAttrs, Perms};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, Wire, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "usb.xhci-pci";

/// How much address space the base address register claims.
///
/// The register block is [`REGISTER_BYTES`] and a BAR window is a power of two
/// (Rev 2.1 §6.2.5.1), so this is the next one up: 16 KiB. What the extra
/// 4 KiB decodes is what §5.5 says of every reserved dword in the runtime
/// register file — zero on a read, nothing on a write.
pub const BAR_BYTES: u64 = REGISTER_BYTES.next_power_of_two();

/// `COMMAND[10]`, Interrupt Disable (Rev 3.0 §6.2.2). Set, and the function
/// drives no `INTx#`.
const COMMAND_INTX_OFF: u16 = 0x0400;

/// Which bits of the Command register this function implements.
///
/// Rev 2.1 §6.2.2 lets a function hardwire to zero any bit it does not
/// implement. This one decodes no I/O space, so bit 0 reads back zero however
/// hard firmware writes it; the three that matter are Memory Space, Bus Master
/// and — from Rev 3.0 §6.2.2 — Interrupt Disable at bit 10.
const COMMAND_IMPLEMENTED: u16 = config::COMMAND_MEMORY | config::COMMAND_MASTER | COMMAND_INTX_OFF;

/// `STATUS[3]`, Interrupt Status (Rev 3.0 §6.2.3): the function's own interrupt
/// condition, whatever [`COMMAND_INTX_OFF`] says about emitting it.
const STATUS_INTERRUPT: u8 = 0x08;

/// Base class `0Ch`: a serial bus controller.
const CLASS_SERIAL: u8 = 0x0c;
/// Sub-class `03h` under [`CLASS_SERIAL`]: a USB controller.
const SUBCLASS_USB: u8 = 0x03;
/// Programming interface `30h`: an xHCI controller. Together that is class code
/// `0C0330h`, which is what an operating system enumerates for.
const PROGIF_XHCI: u8 = 0x30;

/// The one source on the private net between the engine and this function.
///
/// Any id would do — the net has exactly one driver and one sink — and it is
/// named rather than left implicit so a `Debug` dump says which pin it is.
const IRQ_SOURCE: WireId = WireId::new(1);

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output, taken **off the card edge**.
    ///
    /// The same `INTA#` this function drives onto its bus's interrupt net, for
    /// a board with no interrupt router: a machine with a south bridge collects
    /// the net and leaves this unwired, and a machine without one wires it
    /// straight to an interrupt controller. See
    /// [`Intx`](crate::bus::pci::Intx).
    ///
    /// Level-triggered, and the AND of `USBCMD.INTE`, `IMAN.IE` and `IMAN.IP`
    /// (xHCI 1.2 §4.17.3) with `COMMAND[10]` clear.
    pub const IRQ: &str = "irq";
}

// ---------------------------------------------------------------------------
// the configuration face
// ---------------------------------------------------------------------------

/// The registers a configuration cycle reaches, and the BAR that carries the
/// controller.
///
/// Separate from [`XhciPci`] because a [`PciFunction`] has to be reachable as
/// an `Arc<dyn PciFunction>` while [`Device::realize`] only ever has `&self` —
/// the same shape [`crate::dev::nvme`] and
/// [`pc::vgapci`](crate::dev::pc::vgapci) use for the same reason. It is also
/// the [`WireSink`] the engine's interrupt output lands on.
struct Function {
    /// The 256 bytes that are not base address registers. At
    /// [`LockRank::DEVICE`], released before anything outward.
    config: Mutex<ConfigSpace>,
    /// The base address registers, which own `0x10`-`0x27` and `0x30`-`0x33`.
    bars: Bars,
    /// `INTA#`, on the fabric and on the card edge.
    intx: Intx,
    /// The level the *engine* is driving, before `COMMAND[10]`. This is what
    /// `STATUS[3]` reports.
    raised: AtomicBool,
    /// `COMMAND[10]`, Interrupt Disable.
    intx_off: AtomicBool,
    xhci: Arc<Xhci>,
}

impl fmt::Debug for Function {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Function");
        match self.config.try_lock() {
            Some(c) => s.field(
                "command",
                &(u16::from(c.byte(config::COMMAND)) | u16::from(c.byte(config::COMMAND + 1)) << 8),
            ),
            None => s.field("command", &"<in use>"),
        };
        s.field("bars", &self.bars)
            .field("intx", &self.intx)
            .finish_non_exhaustive()
    }
}

impl Function {
    /// The header this controller hardwires (Rev 2.1 §6.1).
    fn fresh_config(vendor: u16, device: u16, revision: u8) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(vendor), 2);
        c.hardwire(config::DEVICE_ID, u32::from(device), 2);
        // §6.2.2: every enable bit clear out of reset, so the function decodes
        // nothing and masters nothing until firmware has finished sizing.
        c.hardwire(config::COMMAND, 0x0000, 2);
        // §6.2.3: DEVSEL# timing 01b (medium) in bits 10:9.
        c.hardwire(config::STATUS, 0x0200, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        c.hardwire(config::CLASS_CODE, u32::from(PROGIF_XHCI), 1);
        c.hardwire(config::CLASS_CODE + 1, u32::from(SUBCLASS_USB), 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(CLASS_SERIAL), 1);
        // §6.2.1: header type 00h, single function.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);
        // §6.2.4: this function drives `INTA#`.
        c.hardwire(config::INTERRUPT_PIN, u32::from(IntxPin::A.0), 1);

        c.allow(config::COMMAND, 2);
        c.allow(config::CACHE_LINE_SIZE, 1);
        c.allow(config::LATENCY_TIMER, 1);
        // Which interrupt controller input firmware routed the pin to. Firmware
        // writes it; hardware reads it back and ignores it.
        c.allow(config::INTERRUPT_LINE, 1);
        c
    }

    /// The Command register as it stands.
    fn command(&self) -> u16 {
        let c = self.config.lock();
        u16::from(c.byte(config::COMMAND)) | u16::from(c.byte(config::COMMAND + 1)) << 8
    }

    /// Push the two Command bits the engine and the pin care about across.
    ///
    /// Called with no lock held: [`Function::drive_intx`] drives a wire and a
    /// fabric net.
    fn apply_command(&self, command: u16) {
        self.xhci.set_master(command & config::COMMAND_MASTER != 0);
        self.intx_off
            .store(command & COMMAND_INTX_OFF != 0, Ordering::Relaxed);
        self.drive_intx();
    }

    /// Put the pin where the engine's level and `COMMAND[10]` say between them.
    fn drive_intx(&self) {
        let asserted =
            self.raised.load(Ordering::Relaxed) && !self.intx_off.load(Ordering::Relaxed);
        self.intx.set(Level::from_bool(asserted));
    }
}

/// The engine's one interrupt output, arriving.
impl WireSink for Function {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.raised.store(level.is_high(), Ordering::Relaxed);
        self.drive_intx();
    }
}

impl PciFunction for Function {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: nothing here has a side effect. `STATUS[3]` below
        // is *read* from the level the engine published into an atomic, and
        // reading a level acknowledges nothing — which is exactly what the
        // debug rule asks of a status bit.
        self.config.lock().read(offset, dst);
        self.bars.config_read(offset, dst);
        if self.raised.load(Ordering::Relaxed) {
            for (i, slot) in dst.iter_mut().enumerate() {
                if offset.saturating_add(i as u16) == config::STATUS {
                    *slot |= STATUS_INTERRUPT;
                }
            }
        }
        // A retopology that could not happen when it was asked for gets its
        // next chance here, for the reason `pc::pmc` and `dev::nvme` both give:
        // leaving the machine's memory map disagreeing with its own registers
        // is worse than a debugger's read having an invisible effect on neither.
        if self.bars.is_stale() {
            self.bars.sync(self.command(), false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here would move the register block out from under
            // the guest, or switch off the bus mastering a transfer in flight
            // is relying on. `ConfigPorts` refuses one before it reaches here;
            // this is the second lock on the same door.
            return;
        }
        let bars_moved = self.bars.config_write(offset, src);
        let (command_moved, command) = {
            let mut c = self.config.lock();
            let moved = c.write(offset, src);
            // Rev 2.1 §6.2.2: an unimplemented Command bit is hardwired to
            // zero, so a write that sets one reads back as zero.
            let raw =
                u16::from(c.byte(config::COMMAND)) | u16::from(c.byte(config::COMMAND + 1)) << 8;
            let kept = raw & COMMAND_IMPLEMENTED;
            if kept != raw {
                c.set_byte(config::COMMAND, kept as u8);
                c.set_byte(config::COMMAND + 1, (kept >> 8) as u8);
            }
            (
                moved
                    && offset < config::COMMAND + 2
                    && offset.saturating_add(src.len() as u16) > config::COMMAND,
                kept,
            )
        };
        // The configuration lock is released before either of these: one drives
        // a wire and the other retopologises an address space.
        if command_moved {
            self.apply_command(command);
        }
        if bars_moved || command_moved || self.bars.is_stale() {
            self.bars.sync(command, false);
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An xHCI host controller as a PCI function.
#[derive(Debug)]
pub struct XhciPci {
    regs: Arc<Function>,
    xhci: Arc<Xhci>,
    bus: Arc<PciBus>,
    at: Bdf,
    vendor: u16,
    device: u16,
    revision: u8,
}

impl XhciPci {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `bus` — the **PCI** fabric this function is on. Defaults to `pci0`,
    ///   which is what every other function in this tree defaults to.
    /// * `usb-bus` — the named [`UsbBus`] this controller is the root of.
    ///   Required, and a separate property from `bus` because the two really
    ///   are two buses: this object is a function on one and the root of the
    ///   other.
    /// * `device`, `function` — where on the fabric it answers.
    /// * `vendor-id`, `device-id`, `revision` — the identification a driver
    ///   matches on.
    /// * `ports`, `slots`, `microframe` — [`Params`], as `usb.xhci` takes them.
    ///
    /// Allocation and validation only; nothing is announced onto either bus
    /// until [`realize`](Device::realize).
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property, [`Error::Config`]
    /// for a value outside its range, for a USB bus already smaller than the
    /// port count asked for, or for a name already open as another kind of bus.
    pub fn new(props: &Props) -> Result<XhciPci> {
        let mut r = props.reader();
        let pci_name = r.or_str("bus", "pci0")?.to_string();
        let usb_name = r.require_str("usb-bus")?.to_string();
        let device_no = r.or_range("device", 5u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let function_no = r.or_range(
            "function",
            0u64,
            0..=u64::from(crate::bus::pci::MAX_FUNCTION),
        )?;
        let vendor = r.or_range("vendor-id", 0x1234u64, 0..=0xffff)?;
        let device = r.or_range("device-id", 0x1e31u64, 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let ports = r.or_range("ports", 1u64, 1..=MAX_PORTS as u64)?;
        let slots = r.or_range("slots", 8u64, 1..=MAX_SLOTS as u64)?;
        let microframe = r.or_range("microframe", 7500u64, 1..=u64::from(u32::MAX))?;
        r.finish()?;

        let usb = crate::bus::usb::buses::attach(props, &usb_name, ports as u8)?;
        if usb.port_count() < ports as u8 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "the USB bus `{usb_name}` already has {} ports and this controller asked for \
                     {ports}; the first object to name a bus fixes its size",
                    usb.port_count()
                ),
            });
        }
        let pci = crate::bus::pci::buses::attach(props, &pci_name)?;
        let at = Bdf::new(0, device_no as u8, function_no as u8)?;
        XhciPci::with_buses(
            pci,
            at,
            usb,
            Params {
                ports: ports as u8,
                slots: slots as u8,
                microframe_ticks: microframe,
            },
            vendor as u16,
            device as u16,
            revision as u8,
        )
    }

    /// The same function, built from handles a test or a Rust caller already
    /// has.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the BAR table refuses the window, which would be a
    /// bug in this file rather than anything a caller can cause.
    pub fn with_buses(
        pci: Arc<PciBus>,
        at: Bdf,
        usb: Arc<UsbBus>,
        params: Params,
        vendor: u16,
        device: u16,
        revision: u8,
    ) -> Result<XhciPci> {
        let xhci = Arc::new(Xhci::new(usb, params));
        let region = register_region(&xhci, "xhci.bar0", BAR_BYTES);
        // xHCI 1.2 §5.2: the register block is a memory window at BAR0, and it
        // is 64-bit capable. Non-prefetchable, because reading a doorbell or a
        // write-1-to-clear status register is not free of consequence and a
        // prefetchable window says it is (Rev 2.1 §6.2.5.1).
        let bars =
            Bars::new().with(0, Bar::memory(BAR_BYTES).wide().decoding(region, Perms::RW))?;
        let regs = Arc::new(Function {
            config: Mutex::with_rank(
                LockRank::DEVICE,
                Function::fresh_config(vendor, device, revision),
            ),
            bars,
            intx: Intx::new(IntxPin::A),
            raised: AtomicBool::new(false),
            intx_off: AtomicBool::new(false),
            xhci: Arc::clone(&xhci),
        });
        // The private net between the engine's one output and this function's
        // masking. Weak on the sink side: the function holds the engine and the
        // engine holds this net, so a strong edge would close a cycle.
        let wire = Wire::builder()
            .source(IRQ_SOURCE)
            .sink_weak(Arc::downgrade(&regs) as Weak<dyn WireSink>, 0)
            .build_shared();
        xhci.connect_irq(WireSource::new(wire, IRQ_SOURCE));
        Ok(XhciPci {
            regs,
            xhci,
            bus: pci,
            at,
            vendor,
            device,
            revision,
        })
    }

    /// Where this function sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The engine behind the register block.
    #[must_use]
    pub fn xhci(&self) -> &Arc<Xhci> {
        &self.xhci
    }

    /// The base address registers, for a test that wants to see where the
    /// window went.
    #[must_use]
    pub fn bars(&self) -> &Bars {
        &self.regs.bars
    }

    /// The Command register as it stands.
    #[must_use]
    pub fn command(&self) -> u16 {
        self.regs.command()
    }

    /// The `INTA#` pin this function drives.
    #[must_use]
    pub fn intx(&self) -> &Intx {
        &self.regs.intx
    }

    /// Put this function's window into `space` and tell the engine which space
    /// its rings live in. **Retopology.**
    ///
    /// What [`Instance::bind`] does, reachable directly so a unit test can
    /// assemble a controller without a machine.
    ///
    /// # Errors
    ///
    /// Whatever the space refuses: a window that does not fit, or a nesting
    /// depth this space will not take.
    pub fn attach_space(
        &self,
        space: &Arc<AddressSpace>,
        requester: crate::core::space::RequesterId,
    ) -> Result<()> {
        self.xhci.attach_space(space, requester);
        self.regs.bars.install(space, self.regs.command())
    }
}

impl Device for XhciPci {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: announcing itself onto the fabric. Nothing
        // observable happened before this (`CLAUDE.md`, two-phase construction).
        self.bus
            .attach(self.at, Arc::clone(&self.regs) as Arc<dyn PciFunction>)?;
        // And plugging `INTA#` in, which can only happen now: a function that
        // is not on the fabric has no device number, and the device number is
        // what decides which of the bus's four interrupt nets the pin reaches
        // (`bus::pci::swizzle`).
        self.regs.intx.plug(&self.bus, self.at);
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // `PCIRST#` clears the configuration registers and the controller with
        // them: a driver that comes back expects a halted controller with no
        // slots and a function that decodes nothing, which is what a cold
        // machine has.
        *self.regs.config.lock() = Function::fresh_config(self.vendor, self.device, self.revision);
        self.regs.bars.reset();
        self.xhci.reset(kind);
        // The Command register is zero now, so this is what clears Bus Master
        // Enable and drops the pin.
        self.regs.apply_command(0);
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.bars.sync(0, true);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(self.regs.config.lock().bytes())?;
        for value in self.regs.bars.latches() {
            w.write_u32(value)?;
        }
        self.xhci.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let config: &[u8] = r.read_bytes()?;
        let mut latches = [0u32; Bars::COUNT as usize];
        for slot in &mut latches {
            *slot = r.read_u32()?;
        }
        {
            let mut c = self.regs.config.lock();
            *c = Function::fresh_config(self.vendor, self.device, self.revision);
            c.restore(config);
        }
        self.regs.bars.set_latches(&latches);
        self.xhci.load(r)?;
        // Where the window is, whether the engine may fetch, and whether the
        // pin is gated are all functions of the Command register, so all three
        // are re-derived rather than saved (`CLAUDE.md`: derived state is never
        // serialized).
        let command = self.regs.command();
        self.regs.apply_command(command);
        self.regs.bars.sync(command, true);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "an xHCI PCI function drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.regs.intx.connect(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.drive_intx();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes, for the reason `usb.xhci` is: `MFINDEX` runs on its own and a guest
    /// that polls it has to see the answer at the cycle it polled.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.xhci.ticks()
    }

    fn advance_to(&self, tick: u64) {
        self.xhci.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.xhci.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.xhci.attach_lazy(handle);
    }
}

impl Instance for XhciPci {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an xHCI controller masters the bus its rings and contexts live on, and places \
                 its register block with a base address register: add `space = mem` to the object",
            ),
        })?;
        self.attach_space(space, ctx.requester())
    }
}

/// The `usb.xhci-pci` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an xHCI USB host controller as a PCI function: class code 0C0330h, the register \
              block behind a 64-bit memory base address register, and INTA# onto the fabric",
    properties: PROPERTIES,
    construct: |props| Ok(Box::new(XhciPci::new(props)?)),
};

/// The properties [`CLASS`] accepts.
static PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "bus",
        kind: ValueKind::Str,
        required: false,
        summary: "the PCI fabric this function is on (default `pci0`)",
    },
    PropertySpec {
        name: "usb-bus",
        kind: ValueKind::Str,
        required: true,
        summary: "the named USB bus this controller is the root of",
    },
    PropertySpec {
        name: "device",
        kind: ValueKind::Uint,
        required: false,
        summary: "the device number it answers at on bus 0 (default 5)",
    },
    PropertySpec {
        name: "function",
        kind: ValueKind::Uint,
        required: false,
        summary: "the function number, 0-7 (default 0)",
    },
    PropertySpec {
        name: "vendor-id",
        kind: ValueKind::Uint,
        required: false,
        summary: "the vendor identification (default 0x1234)",
    },
    PropertySpec {
        name: "device-id",
        kind: ValueKind::Uint,
        required: false,
        summary: "the device identification (default 0x1e31)",
    },
    PropertySpec {
        name: "revision",
        kind: ValueKind::Uint,
        required: false,
        summary: "the revision identification byte (default 0)",
    },
    PropertySpec {
        name: "ports",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many root ports, 1 to 15 (default 1)",
    },
    PropertySpec {
        name: "slots",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many device slots, 1 to 31 (default 8)",
    },
    PropertySpec {
        name: "microframe",
        kind: ValueKind::Uint,
        required: false,
        summary: "clock-domain ticks in one 125 us microframe (default 7500, exact at 60 MHz)",
    },
];

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(XhciPci::new(props)?)))
}

/// What the validator should know about `usb.xhci-pci`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("usb-bus", ValueKind::Str).required())
        .prop(
            PropSchema::new("device", ValueKind::Uint)
                .range(0, u64::from(crate::bus::pci::MAX_DEVICE)),
        )
        .prop(
            PropSchema::new("function", ValueKind::Uint)
                .range(0, u64::from(crate::bus::pci::MAX_FUNCTION)),
        )
        .prop(PropSchema::new("vendor-id", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("device-id", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, MAX_PORTS as u64))
        .prop(PropSchema::new("slots", ValueKind::Uint).range(1, MAX_SLOTS as u64))
        .prop(PropSchema::new("microframe", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .port(pin::IRQ, PortDir::Out)
}
