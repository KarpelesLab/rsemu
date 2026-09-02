//! AHCI: a Serial ATA host bus adapter as a PCI function.
//!
//! # What this device is
//!
//! The second **bus master** in this tree, after [`nvme`](crate::dev::nvme),
//! and the one that makes a real ATA drive reachable the way a modern machine
//! reaches one. A driver builds a command list, a received-FIS area and a
//! scatter/gather table in its own memory, writes one bit into `PxCI`, and the
//! adapter fetches all of it, hands the command to the drive, moves the data,
//! writes the drive's answer back and raises `INTA#`.
//!
//! # The split, and the seam it needed
//!
//! [`hba`] is AHCI and Serial ATA; this file is PCI. That is the same division
//! [`nvme`](crate::dev::nvme) draws, and it is falsifiable the same way:
//! `hba.rs` contains no configuration space offset and no [`Bdf`], and this
//! file contains no AHCI register offset and no FIS.
//!
//! There is a second split, and it is the one this device was blocked on. An
//! AHCI port carries an **ATA command**, and the command set already existed —
//! on the far side of eight 8-bit ports, reachable only by writing them in the
//! right order. A Serial ATA adapter has no ports to write. So
//! [`ata::disk::taskfile`](crate::dev::ata::disk::taskfile) came first: the
//! same command block as a struct, loaded into the same registers, dispatched
//! by the same `AtaDisk::command`. **There is one command set underneath both
//! adapters**, and deleting the taskfile module leaves the AT's IDE channel
//! working exactly as it did.
//!
//! ```text
//!   pc::ide  ─ eight ports ──┐
//!                            ├──► AtaDisk ──► Medium
//!   ahci     ─ a taskfile ───┘
//! ```
//!
//! # PIO and DMA
//!
//! Both, because the drive says which and the adapter does not guess. A PIO
//! command is announced to the driver by a PIO Setup FIS before each data block
//! and ends by latching that FIS's ending status — `PxIS.PSS`. A DMA command
//! moves its data and ends with a Register - Device to Host FIS — `PxIS.DHRS`.
//! Getting that backwards is the kind of thing that works with one driver and
//! hangs another, which is why [`Phase`](crate::dev::ata::Phase) carries it out
//! of the drive rather than the adapter deriving it from an opcode.
//!
//! A drive only answers the DMA command family when its `dma` property is set,
//! and the default is off — an AT-class IDE cable has no bus master on it, and
//! a drive that advertised DMA there would be inviting a driver to program an
//! engine that is not there. `machines/ahci-mini.machine` sets it, and says so.
//!
//! # Sources
//!
//! * **Serial ATA AHCI Specification, Revision 1.3.1** (Intel) — §2.1 for the
//!   PCI header, in particular `ABAR` at offset `24h` (base address register
//!   five) and the class code `010601h`; the rest is cited in [`hba`].
//! * The *PCI Local Bus Specification* Rev 2.1 §6.1/§6.2 for the Type 00h
//!   header, §6.2.5.1 for the base address registers, and Rev 3.0 §6.2.2 for
//!   the Interrupt Disable bit.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).

pub mod hba;

#[cfg(test)]
mod tests;

pub use hba::{AHCI_RANK, Hba, MAX_PORTS, REGISTER_LEN};

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::pci::{Bar, Bars, Bdf, ConfigSpace, PciBus, PciFunction, buses, config};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, Perms, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::WireSource;
use crate::dev::ata::bays;
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "ahci.hba";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The prefix a port's drive bay is named with when a machine file says
/// nothing: port *n* looks in `sata`*n*.
pub const DEFAULT_BAY_PREFIX: &str = "sata";

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output: `INTA#`, level triggered, asserted while `GHC.IE`
    /// is set and any implemented port has an enabled interrupt pending.
    ///
    /// The **same pin** the adapter drives onto its bus's interrupt net, taken
    /// off the card edge instead: a board with an interrupt router collects the
    /// net and leaves this unwired, and a board without one — the `ahci-mini`
    /// case — wires it straight to an interrupt controller. See
    /// [`Intx`](crate::bus::pci::Intx).
    pub const IRQ: &str = "irq";
}

/// Which bits of the Command register this function implements.
///
/// Rev 2.1 §6.2.2 lets a function hardwire to zero any bit it does not
/// implement. This one decodes no I/O space — `CAP.SAM` is set, so there is no
/// legacy task-file interface to decode one for — so bit 0 reads back zero
/// however hard firmware writes it.
const COMMAND_IMPLEMENTED: u16 = config::COMMAND_MEMORY | config::COMMAND_MASTER | COMMAND_INTX_OFF;

/// `COMMAND[10]`, Interrupt Disable (Rev 3.0 §6.2.2).
const COMMAND_INTX_OFF: u16 = 0x0400;

/// `STATUS[3]`, Interrupt Status (Rev 3.0 §6.2.3).
const STATUS_INTERRUPT: u8 = 0x08;

/// Base class `01h`: a mass storage controller (AHCI §2.1.5).
const CLASS_STORAGE: u8 = 0x01;
/// Sub-class `06h`: a Serial ATA controller.
const SUBCLASS_SATA: u8 = 0x06;
/// Programming interface `01h` under sub-class `06h`: an AHCI HBA with a major
/// revision of one. Together, class code `010601h`.
const PROGIF_AHCI: u8 = 0x01;

/// Which base address register carries the register block.
///
/// **Five**, and that is not a choice: AHCI §2.1.11 puts `ABAR` at
/// configuration offset `24h`, which is base address register five. The first
/// five are where a controller that also implements the legacy SFF-8038i
/// interface puts its task-file windows; this one does not, and leaves them
/// unimplemented rather than decoding something it has no registers for.
const ABAR: u8 = 5;

// ---------------------------------------------------------------------------
// the configuration face
// ---------------------------------------------------------------------------

/// The registers a configuration cycle reaches, and the BAR that carries the
/// adapter.
///
/// Separate from [`Ahci`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` while [`Device::realize`] only ever has `&self`.
struct Function {
    /// The 256 bytes that are not base address registers, at
    /// [`LockRank::DEVICE`] — released before anything outward, including
    /// before the adapter's own state lock, which ranks below it.
    config: Mutex<ConfigSpace>,
    bars: Bars,
    hba: Arc<Hba>,
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
        s.field("bars", &self.bars).finish()
    }
}

impl Function {
    /// The header this adapter hardwires (Rev 2.1 §6.1, AHCI §2.1).
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
        c.hardwire(config::CLASS_CODE, u32::from(PROGIF_AHCI), 1);
        c.hardwire(config::CLASS_CODE + 1, u32::from(SUBCLASS_SATA), 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(CLASS_STORAGE), 1);
        // §6.2.1: header type 00h, single function.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);
        // §6.2.4: this function drives `INTA#`.
        c.hardwire(config::INTERRUPT_PIN, 0x01, 1);

        c.allow(config::COMMAND, 2);
        c.allow(config::CACHE_LINE_SIZE, 1);
        c.allow(config::LATENCY_TIMER, 1);
        c.allow(config::INTERRUPT_LINE, 1);
        c
    }

    /// The Command register as it stands.
    fn command(&self) -> u16 {
        let c = self.config.lock();
        u16::from(c.byte(config::COMMAND)) | u16::from(c.byte(config::COMMAND + 1)) << 8
    }

    /// Push the two Command bits the adapter cares about across to it.
    ///
    /// Called with no lock held: `set_intx_disabled` drives the interrupt wire.
    fn apply_command(&self, command: u16) {
        self.hba.set_master(command & config::COMMAND_MASTER != 0);
        self.hba.set_intx_disabled(command & COMMAND_INTX_OFF != 0);
    }
}

impl PciFunction for Function {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: a configuration read of this function has no side
        // effects. The Interrupt Status bit below is read from the adapter's
        // published level, which is an atomic load.
        self.config.lock().read(offset, dst);
        self.bars.config_read(offset, dst);
        if self.hba.interrupt_pending() {
            for (i, slot) in dst.iter_mut().enumerate() {
                if offset.saturating_add(i as u16) == config::STATUS {
                    *slot |= STATUS_INTERRUPT;
                }
            }
        }
        // A retopology that could not happen when it was asked for gets its
        // next chance here, for the reason `pc::pmc`, `pc::vgapci` and `nvme`
        // all give: leaving the machine's memory map disagreeing with its own
        // registers is worse than a debugger's read having an invisible effect
        // on neither.
        if self.bars.is_stale() {
            self.bars.sync(self.command(), false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here would move `ABAR` under the guest's feet, or
            // switch off the bus mastering a command is relying on.
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

/// A Serial ATA host bus adapter on a PCI bus.
#[derive(Debug)]
pub struct Ahci {
    regs: Arc<Function>,
    hba: Arc<Hba>,
    bus: Arc<PciBus>,
    at: Bdf,
    vendor: u16,
    device: u16,
    revision: u8,
}

impl Ahci {
    /// Validate `props` and build the adapter.
    ///
    /// Allocation and validation only: the fabric handle and the drive bays are
    /// acquired here because acquiring a host object *is* allocation
    /// ([`core::hosts`](crate::core::hosts)), and nothing is announced onto the
    /// bus until [`realize`](Device::realize).
    ///
    /// # Where the drives come from
    ///
    /// The same rendezvous the AT's IDE channel uses: a named
    /// [`Bay`](crate::dev::ata::bays::Bay) per port. Port *n* looks in
    /// `<bays><n>` — `sata0`, `sata1`, … by default — and an `ata.disk` object
    /// in the same machine names the same bay. An empty bay is an empty bay:
    /// `PxSSTS` reports no device and a driver skips the port, which is what an
    /// unpopulated connector does.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know or a value
    /// outside its range, and [`Error::Config`] if the `bus` name is already
    /// open as something else.
    pub fn new(props: &Props) -> Result<Ahci> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device_no = r.or_range("device", 5u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let function_no = r.or_range(
            "function",
            0u64,
            0..=u64::from(crate::bus::pci::MAX_FUNCTION),
        )?;
        let vendor = r.or_range("vendor-id", 0x1234u64, 0..=0xffff)?;
        let device = r.or_range("device-id", 0x2922u64, 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let ports = r.or_range("ports", 1u64, 1..=MAX_PORTS as u64)?;
        let prefix = r.or_str("bays", DEFAULT_BAY_PREFIX)?.to_string();
        r.finish()?;

        let mut table: Vec<(String, Arc<bays::Bay>)> = Vec::new();
        for index in 0..ports {
            let name = format!("{prefix}{index}");
            let bay = bays::attach(props, &name)?;
            table.push((name, bay));
        }

        let bus = buses::attach(props, &bus_name)?;
        let at = Bdf::new(0, device_no as u8, function_no as u8)?;
        Ahci::with_bus(bus, at, vendor as u16, device as u16, revision as u8, table)
    }

    /// The same adapter, built from a fabric handle and bays a test or a Rust
    /// caller already has.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the BAR table refuses the window, which would be a
    /// bug in this file rather than anything a caller can cause.
    pub fn with_bus(
        bus: Arc<PciBus>,
        at: Bdf,
        vendor: u16,
        device: u16,
        revision: u8,
        bays: Vec<(String, Arc<bays::Bay>)>,
    ) -> Result<Ahci> {
        let hba = Arc::new(Hba::new(bays));
        let region: RegionRef = Arc::new(Region::io(
            "ahci.abar",
            REGISTER_LEN,
            Arc::clone(&hba) as Arc<dyn crate::core::space::MemOps>,
        ));
        // AHCI §2.1.11: `ABAR` is a 32-bit, non-prefetchable memory window, and
        // it lives at base address register five. Not 64-bit: the specification
        // makes bits 02:01 read-only zero, which *is* the statement that this
        // range can be mapped anywhere in 32-bit address space and nowhere else.
        let bars = Bars::new().with(ABAR, Bar::memory(REGISTER_LEN).decoding(region, Perms::RW))?;
        Ok(Ahci {
            regs: Arc::new(Function {
                config: Mutex::with_rank(
                    LockRank::DEVICE,
                    Function::fresh_config(vendor, device, revision),
                ),
                bars,
                hba: Arc::clone(&hba),
            }),
            hba,
            bus,
            at,
            vendor,
            device,
            revision,
        })
    }

    /// Where this adapter sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The engine behind the register block.
    #[must_use]
    pub fn hba(&self) -> &Arc<Hba> {
        &self.hba
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

    /// Put this function's window into `space` and tell the adapter which space
    /// its command lists live in. **Retopology.**
    ///
    /// # Errors
    ///
    /// Whatever the space refuses.
    pub fn attach_space(
        &self,
        space: &Arc<AddressSpace>,
        requester: crate::core::space::RequesterId,
    ) -> Result<()> {
        self.hba.attach_space(space, requester);
        self.regs.bars.install(space, self.regs.command())
    }
}

/// The `ahci.hba` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Serial ATA host bus adapter as a PCI function: the AHCI 1.3.1 register block, \
              the command list, the received-FIS area and a PRDT-walking bus master over ATA \
              drives in named bays",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric this adapter is on (default `pci0`)",
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
            summary: "the device identification (default 0x2922)",
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
            summary: "how many Serial ATA ports it implements, 1 to 8 (default 1)",
        },
        PropertySpec {
            name: "bays",
            kind: ValueKind::Str,
            required: false,
            summary: "the drive bay name prefix: port n looks in `<bays>n` (default `sata`)",
        },
    ],
    construct: |props| Ok(Box::new(Ahci::new(props)?)),
};

impl Device for Ahci {
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
        self.hba.intx().plug(&self.bus, self.at);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // `PCIRST#` clears the configuration registers and the adapter with
        // them: a driver that comes back expects `GHC.IE` clear, every port
        // stopped and `PxTFD` holding the device's signature, which is what a
        // cold machine has. The drives reset themselves — they are separate
        // objects and the machine resets each of them.
        *self.regs.config.lock() = Function::fresh_config(self.vendor, self.device, self.revision);
        self.regs.bars.reset();
        self.hba.reset();
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.bars.sync(self.regs.command(), true);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(self.regs.config.lock().bytes())?;
        for value in self.regs.bars.latches() {
            w.write_u32(value)?;
        }
        self.hba.save(w)
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
        self.hba.load(r)?;
        // Where the window is and what the adapter may do are both functions of
        // the Command register, so both are re-derived rather than saved
        // (`CLAUDE.md`: derived state is never serialized).
        let command = self.regs.command();
        self.regs.apply_command(command);
        self.regs.bars.sync(command, true);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: format!("an AHCI adapter drives `{}` and nothing else", pin::IRQ),
            });
        }
        self.hba.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.hba.refresh_irq();
    }
}

impl Instance for Ahci {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an AHCI adapter masters the memory its command lists live in, and places its \
                 register block with a base address register: add `space = mem` to the object",
            ),
        })?;
        self.attach_space(space, ctx.requester())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ahci::new(props)?)))
}

/// What the validator should know about `ahci.hba`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
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
        .prop(PropSchema::new("bays", ValueKind::Str))
        .port(pin::IRQ, PortDir::Out)
}
