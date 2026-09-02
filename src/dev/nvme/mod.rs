//! NVM Express: a PCI function that reads and writes guest memory on its own.
//!
//! # What is new about this device
//!
//! Everything else in this tree that moves bytes for a guest is either
//! programmed I/O — the guest reads a data port a word at a time, as
//! `dev::ata` does — or third-party DMA, where a separate
//! controller ([`pc::dma`](crate::dev::pc::dma)) does the moving. An NVMe
//! controller is neither. It is a **bus master**: the driver builds a
//! submission queue, a completion queue and a list of Physical Region Pages in
//! its *own* memory, writes one 32-bit doorbell, and the controller reads all
//! of that out of guest RAM itself, moves the data, writes a completion back,
//! and raises an interrupt.
//!
//! ```text
//!   guest RAM                                    the controller
//!   ---------                                    --------------
//!   SQ entry (64 bytes) ─────── doorbell ──────►  fetch
//!     opcode, NSID, PRP1, PRP2, SLBA, NLB           │
//!   PRP list  ◄──────────────────────────────────── walk
//!   data pages ◄──────── read ── write ───────────► the medium
//!   CQ entry (16 bytes) ◄──────────────────────── post + phase tag
//!                        ◄──── INTx# ──────────── the wire
//! ```
//!
//! Two consequences shape this module, and both are argued where they bite:
//!
//! * **Every pointer comes from the guest**, so every walk is bounded and every
//!   fault is a status code rather than a panic ([`ctrl`]'s `prp_chunks`, and
//!   `fuzz/fuzz_targets/nvme_mmio.rs`).
//! * **A guest can aim a PRP entry at this controller's own doorbells**, so the
//!   engine is re-entrant by design and never holds a lock across a
//!   guest-memory access ([`ctrl`]'s module documentation).
//!
//! # The split
//!
//! [`ctrl::Controller`] is NVM Express and knows nothing about PCI;
//! [`Nvme`] is the PCI function and knows nothing about queues. That is the
//! same division `docs/buses/storage.md` argues for the ATA drive and its
//! channel, and it is falsifiable the same way: `ctrl.rs` contains no
//! configuration space offset and no `Bdf`, and this file contains no NVMe
//! opcode and no queue.
//!
//! The transport contributes exactly four things, and all four arrive through
//! setters on the controller: the address space it masters, its `INTx#` output,
//! `COMMAND[2]` (Bus Master Enable — a function that may not master the bus
//! fetches no commands), and `COMMAND[10]` (Interrupt Disable).
//!
//! # The namespace is a `Medium`
//!
//! The bytes come from [`dev::medium::Medium`](crate::dev::medium::Medium), the same
//! seam an ATA drive's platter uses, so `--drive nvme0=disk.qcow2` works here
//! for exactly the reason it works there and **no image format is parsed in
//! this module** (`ROADMAP.md` §7.1). A `no_std` build gets a
//! [`RamStore`]; a `dev-blk` build gets a host
//! file through `fstool`.
//!
//! # What is deliberately not here
//!
//! * **MSI and MSI-X.** `src/bus/pci` has no capability list yet, so there is
//!   nothing to advertise them in. The controller is pin-based, which NVMe
//!   permits (§7.5.1.1), and its completion queues carry the interrupt vector a
//!   driver programmed so that adding MSI-X later changes the transport rather
//!   than the engine.
//! * **Scatter Gather Lists.** `Identify Controller`'s `SGLS` reports zero and
//!   a command with `PSDT` set is refused with Invalid Field, which is how a
//!   driver is supposed to find out. PRPs are what a PCIe controller has to
//!   support anyway.
//! * **More than one namespace**, formats other than a flat LBA one, metadata,
//!   end-to-end protection, reservations, and the optional command sets. Each
//!   is absent rather than half-present: `Identify` reports what is here.
//!
//! # Sources
//!
//! The **NVM Express Base Specification, Revision 1.4** (freely published at
//! <https://nvmexpress.org/specifications/>) for everything above the
//! transport, and the *PCI Local Bus Specification* Rev 2.1 §6.1/§6.2 for the
//! Type 00h header, §6.2.5.1 for the base address registers and Rev 3.0 §6.2.2
//! for the Interrupt Disable bit. The class code `010802h` is NVMe §2.1's, and
//! the *PCI Code and ID Assignment Specification*'s. No emulator source was
//! consulted (`CLAUDE.md`, provenance).

pub mod ctrl;

#[cfg(test)]
mod tests;

pub use ctrl::{Controller, MAX_IO_QUEUES, NVME_RANK, Namespace, Params, REGISTER_LEN};

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::pci::{Bar, Bars, Bdf, ConfigSpace, PciBus, PciFunction, buses, config};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, Perms, RamStore, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::WireSource;
use crate::dev::medium::{self, Medium, Snapshot};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "nvme.controller";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The media slot a controller looks for when its machine file names none.
pub const DEFAULT_SLOT: &str = "nvme0";

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output: `INTA#`, level-triggered, asserted while any
    /// completion queue holds an entry the host has not acknowledged (NVMe
    /// §7.5.1.1).
    pub const IRQ: &str = "irq";
}

/// Which bits of the Command register this function implements.
///
/// Rev 2.1 §6.2.2 lets a function hardwire to zero any bit it does not
/// implement. This one decodes no I/O space, so bit 0 reads back zero however
/// hard firmware writes it; the three that matter are Memory Space, Bus Master
/// and — from Rev 3.0 §6.2.2 — Interrupt Disable at bit 10.
const COMMAND_IMPLEMENTED: u16 = config::COMMAND_MEMORY | config::COMMAND_MASTER | COMMAND_INTX_OFF;

/// `COMMAND[10]`, Interrupt Disable (*PCI Local Bus Specification* Rev 3.0
/// §6.2.2). Set, and the function drives no `INTx#`.
const COMMAND_INTX_OFF: u16 = 0x0400;

/// `STATUS[3]`, Interrupt Status (Rev 3.0 §6.2.3): the function's own interrupt
/// state, whatever [`COMMAND_INTX_OFF`] says about emitting it.
const STATUS_INTERRUPT: u8 = 0x08;

/// Base class `01h`: a mass storage controller.
const CLASS_STORAGE: u8 = 0x01;
/// Sub-class `08h` under [`CLASS_STORAGE`]: a non-volatile memory controller.
const SUBCLASS_NVM: u8 = 0x08;
/// Programming interface `02h`: an NVM Express I/O controller. Together that
/// is class code `010802h`, which is what a driver enumerates for (NVMe §2.1).
const PROGIF_NVME: u8 = 0x02;

// ---------------------------------------------------------------------------
// the configuration face
// ---------------------------------------------------------------------------

/// The registers a configuration cycle reaches, and the BAR that carries the
/// controller.
///
/// Separate from [`Nvme`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` while [`Device::realize`] only ever has `&self` — the
/// same shape [`pc::vgapci`](crate::dev::pc::vgapci) uses for the same reason.
struct Function {
    /// The 256 bytes that are not base address registers. At
    /// [`LockRank::DEVICE`], released before anything outward — including
    /// before the controller's own state lock, which ranks below it.
    config: Mutex<ConfigSpace>,
    /// The base address registers, which own `0x10`-`0x27` and `0x30`-`0x33`.
    bars: Bars,
    ctrl: Arc<Controller>,
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
        c.hardwire(config::CLASS_CODE, u32::from(PROGIF_NVME), 1);
        c.hardwire(config::CLASS_CODE + 1, u32::from(SUBCLASS_NVM), 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(CLASS_STORAGE), 1);
        // §6.2.1: header type 00h, single function.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);
        // §6.2.4: this function drives `INTA#`.
        c.hardwire(config::INTERRUPT_PIN, 0x01, 1);

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

    /// Push the two Command bits the controller cares about across to it.
    ///
    /// Called with no lock held: `set_intx_disabled` drives the interrupt wire.
    fn apply_command(&self, command: u16) {
        self.ctrl.set_master(command & config::COMMAND_MASTER != 0);
        self.ctrl.set_intx_disabled(command & COMMAND_INTX_OFF != 0);
    }
}

impl PciFunction for Function {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: a configuration read of this function has no side
        // effects. The Interrupt Status bit below is *read* from the
        // controller's published level, which is an atomic load — reading it
        // cannot acknowledge anything, which is exactly what the debug rule
        // asks of a status bit.
        self.config.lock().read(offset, dst);
        self.bars.config_read(offset, dst);
        if self.ctrl.interrupt_pending() {
            for (i, slot) in dst.iter_mut().enumerate() {
                if offset.saturating_add(i as u16) == config::STATUS {
                    *slot |= STATUS_INTERRUPT;
                }
            }
        }
        // A retopology that could not happen when it was asked for gets its
        // next chance here, for the reason `pc::pmc` and `pc::vgapci` both
        // give: leaving the machine's memory map disagreeing with its own
        // registers is worse than a debugger's read having an invisible effect
        // on neither.
        if self.bars.is_stale() {
            self.bars.sync(self.command(), false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here would move a BAR under the guest's feet, or
            // switch off the bus mastering a command is relying on.
            // `ConfigPorts` refuses one before it reaches here; this is the
            // second lock on the same door.
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

/// An NVM Express controller on a PCI bus, with one namespace behind it.
#[derive(Debug)]
pub struct Nvme {
    regs: Arc<Function>,
    ctrl: Arc<Controller>,
    bus: Arc<PciBus>,
    at: Bdf,
    vendor: u16,
    device: u16,
    revision: u8,
}

impl Nvme {
    /// Validate `props` and build the controller.
    ///
    /// Allocation and validation only. The fabric handle and the media slot are
    /// acquired here because acquiring a host object *is* allocation
    /// ([`core::hosts`](crate::core::hosts)), and nothing is announced onto the
    /// bus until [`realize`](Device::realize).
    ///
    /// # Where the bytes come from
    ///
    /// The same two places an `ata.disk`'s do, and
    /// the machine file names neither directly. It names a **media slot**
    /// (`image = "nvme0"`), and the run decides what is behind that name: a
    /// [`Medium`] the host installed — what
    /// `rsemu run … --drive nvme0=disk.qcow2` does — wins and brings its own
    /// capacity, and otherwise the media table's bytes are copied into a
    /// [`RamStore`] of `size` bytes.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know or a value
    /// outside its range; [`Error::Config`] if the `bus` name is already open
    /// as something else, if the capacity is zero or is not a whole number of
    /// logical blocks, or if a bound image does not fit.
    pub fn new(props: &Props) -> Result<Nvme> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device_no = r.or_range("device", 4u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let function_no = r.or_range(
            "function",
            0u64,
            0..=u64::from(crate::bus::pci::MAX_FUNCTION),
        )?;
        let vendor = r.or_range("vendor-id", 0x1234u64, 0..=0xffff)?;
        let device = r.or_range("device-id", 0x1122u64, 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let size = r.or_size("size", 0)?;
        let block = r.or_range("block", 512u64, 512..=4096)?;
        let read_only = r.or("readonly", false)?;
        let queues = r.or_range("queues", 4u64, 1..=u64::from(MAX_IO_QUEUES))?;
        let serial = r.or_str("serial", "RSEMU0000000000000001")?.to_string();
        let model = r.or_str("model", "RSEMU NVME CONTROLLER")?.to_string();
        let firmware = r.or_str("firmware", "1.0")?.to_string();
        let media = r.optional_media("image")?;
        let slot = media.map(crate::core::props::Media::name);
        let image = media.map(crate::core::props::Media::to_bytes);
        r.finish()?;

        if !block.is_power_of_two() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!("a logical block is a power of two, and {block} is not"),
            });
        }
        let lba_shift = block.trailing_zeros();

        // A medium the *host* installed, under the media slot's name if there
        // is one. It wins over the media table: a run that said
        // `--drive nvme0=disk.qcow2` meant it.
        let supplied = match props.hosts() {
            Some(hosts) => {
                let name = slot.unwrap_or(DEFAULT_SLOT);
                medium::get(hosts, name)?.and_then(|slot| slot.take())
            }
            None => None,
        };
        let bytes = match (&supplied, size, image.as_ref()) {
            (Some(medium), _, _) => medium.capacity(),
            (None, 0, Some(image)) => image.len() as u64,
            (None, size, _) => size,
        };
        if bytes == 0 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "a controller with no namespace has nothing to do: give it `size`, an `image` \
                     with bytes behind it, or a medium installed under its media slot",
                ),
            });
        }
        let media: Arc<dyn Medium> = match supplied {
            // The media table is ignored rather than layered on top: a host
            // that named an image file did not also mean "and stamp these bytes
            // over the front of it".
            Some(medium) => medium,
            None => {
                let store = RamStore::new(bytes);
                if let Some(image) = image {
                    if image.len() as u64 > bytes {
                        return Err(Error::Config {
                            at: String::from(CLASS_NAME),
                            message: alloc::format!(
                                "the bound image is {} byte(s) and the namespace holds {bytes}",
                                image.len()
                            ),
                        });
                    }
                    RamStore::write_at(&store, 0, &image).map_err(|e| Error::Config {
                        at: String::from(CLASS_NAME),
                        message: alloc::format!("the bound image did not fit: {e}"),
                    })?;
                }
                Arc::new(store)
            }
        };

        let ns = Namespace::new(media, lba_shift, read_only)?;
        let bus = buses::attach(props, &bus_name)?;
        let at = Bdf::new(0, device_no as u8, function_no as u8)?;
        Nvme::with_bus(
            bus,
            at,
            vendor as u16,
            device as u16,
            revision as u8,
            ns,
            Params {
                vendor: vendor as u16,
                subsystem_vendor: vendor as u16,
                serial,
                model,
                firmware,
                io_queues: queues as u16,
            },
        )
    }

    /// The same controller, built from a fabric handle and a namespace a test
    /// or a Rust caller already has.
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
        ns: Namespace,
        params: Params,
    ) -> Result<Nvme> {
        let ctrl = Arc::new(Controller::new(ns, params));
        let region: RegionRef = Arc::new(Region::io(
            "nvme.regs",
            REGISTER_LEN,
            Arc::clone(&ctrl) as Arc<dyn crate::core::space::MemOps>,
        ));
        // NVMe §2.1.10: the register block is `MLBAR`/`MUBAR`, a 64-bit
        // non-prefetchable memory window at BAR0. 64-bit because that is what
        // the specification names, and because a controller placed above 4 GiB
        // is the case a 32-bit-only model would silently fail at.
        let bars = Bars::new().with(
            0,
            Bar::memory(REGISTER_LEN).wide().decoding(region, Perms::RW),
        )?;
        Ok(Nvme {
            regs: Arc::new(Function {
                config: Mutex::with_rank(
                    LockRank::DEVICE,
                    Function::fresh_config(vendor, device, revision),
                ),
                bars,
                ctrl: Arc::clone(&ctrl),
            }),
            ctrl,
            bus,
            at,
            vendor,
            device,
            revision,
        })
    }

    /// Where this controller sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The engine behind the register block.
    #[must_use]
    pub fn controller(&self) -> &Arc<Controller> {
        &self.ctrl
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

    /// Put this function's window into `space` and tell the controller which
    /// space its queues live in. **Retopology.**
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
        self.ctrl.attach_space(space, requester);
        self.regs.bars.install(space, self.regs.command())
    }
}

/// The `nvme.controller` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an NVM Express controller as a PCI function: the NVMe 1.4 register block, the \
              admin and NVM command sets, and a PRP-walking bus master over one namespace",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric this controller is on (default `pci0`)",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device number it answers at on bus 0 (default 4)",
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
            summary: "the device identification (default 0x1122)",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the revision identification byte (default 0)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot the namespace is bound to; a host medium under that name wins",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the namespace holds, when no image says",
        },
        PropertySpec {
            name: "block",
            kind: ValueKind::Uint,
            required: false,
            summary: "bytes in a logical block: 512, 1024, 2048 or 4096 (default 512)",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "refuse every write, and say so in Identify Namespace's NSATTR",
        },
        PropertySpec {
            name: "queues",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many I/O queue pairs the controller allocates (default 4)",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Str,
            required: false,
            summary: "the serial number Identify Controller reports",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "the model number Identify Controller reports",
        },
        PropertySpec {
            name: "firmware",
            kind: ValueKind::Str,
            required: false,
            summary: "the firmware revision Identify Controller reports",
        },
    ],
    construct: |props| Ok(Box::new(Nvme::new(props)?)),
};

impl Device for Nvme {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: announcing itself onto the fabric. Nothing
        // observable happened before this (`CLAUDE.md`, two-phase construction).
        self.bus
            .attach(self.at, Arc::clone(&self.regs) as Arc<dyn PciFunction>)
    }

    fn reset(&self, _kind: ResetKind) {
        // `PCIRST#` clears the configuration registers and the controller with
        // them: a driver that comes back expects to find `CSTS.RDY` clear and
        // no queues, which is exactly what a cold machine has.
        *self.regs.config.lock() = Function::fresh_config(self.vendor, self.device, self.revision);
        self.regs.bars.reset();
        self.ctrl.reset();
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.bars.sync(self.regs.command(), true);
    }

    fn flush(&self) -> Result<()> {
        // What a shutdown notification through `CC.SHN` would have done, and
        // for the same reason a drive's does: the guest is not obliged to ask,
        // and the write it made still happened.
        self.ctrl.namespace().flush()
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The namespace first, on the terms the medium itself sets — the same
        // three policies an ATA drive's platter offers, and for the same
        // reasons ([`Snapshot`](crate::dev::medium::Snapshot)). A `RamStore`
        // captures; a `dev::blk::Image` records which image it is and flushes
        // it, so what is on disk matches the moment the snapshot was taken.
        let ns = self.ctrl.namespace();
        match ns.snapshot() {
            Snapshot::Capture => w.write_bytes(&ns.contents()?)?,
            Snapshot::Reference => {
                ns.flush()?;
                w.write_bytes(ns.describe().as_bytes())?;
            }
            Snapshot::Refuse => {
                return Err(Error::State(alloc::format!(
                    "this namespace's medium ({}) refuses to be snapshotted",
                    ns.describe()
                )));
            }
        }
        w.write_bytes(self.regs.config.lock().bytes())?;
        for value in self.regs.bars.latches() {
            w.write_u32(value)?;
        }
        self.ctrl.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ns = self.ctrl.namespace();
        let bytes: &[u8] = r.read_bytes()?;
        match ns.snapshot() {
            Snapshot::Capture => {
                let held = ns.blocks() * ns.lba_bytes();
                if bytes.len() as u64 != held {
                    return Err(Error::State(alloc::format!(
                        "the snapshot holds a namespace of {} byte(s), this one holds {held}",
                        bytes.len()
                    )));
                }
                ns.restore(bytes)?;
            }
            Snapshot::Reference => {
                // The bytes are still in the image file; what the chunk holds
                // is *which* image, and the check is that it is still that one.
                let want = ns.describe();
                let got = core::str::from_utf8(bytes).unwrap_or("");
                if got != want {
                    return Err(Error::State(alloc::format!(
                        "this snapshot was taken of `{got}` and this namespace is `{want}`"
                    )));
                }
            }
            Snapshot::Refuse => {
                return Err(Error::State(alloc::format!(
                    "this namespace's medium ({}) refuses to be snapshotted",
                    ns.describe()
                )));
            }
        }
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
        self.ctrl.load(r)?;
        // Where the window is and what the controller may do are both functions
        // of the Command register, so both are re-derived rather than saved
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
                message: alloc::format!(
                    "an NVMe controller drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.ctrl.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.ctrl.refresh_irq();
    }
}

impl Instance for Nvme {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an NVMe controller masters the memory its queues live in, and places its \
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Nvme::new(props)?)))
}

/// What the validator should know about `nvme.controller`.
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
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("block", ValueKind::Uint).range(512, 4096))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("queues", ValueKind::Uint).range(1, u64::from(MAX_IO_QUEUES)))
        .prop(PropSchema::new("serial", ValueKind::Str))
        .prop(PropSchema::new("model", ValueKind::Str))
        .prop(PropSchema::new("firmware", ValueKind::Str))
        .port(pin::IRQ, PortDir::Out)
}
