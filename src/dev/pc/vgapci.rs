//! The PCI half of a display adapter: the Type 00h header that makes a firmware
//! call it the console, and the expansion ROM its video BIOS arrives on.
//!
//! # Why this exists, and why it is a separate object from [`video`](super::video)
//!
//! A firmware written for a PCI machine does not find video by scanning memory.
//! It enumerates the bus, looks for a function whose **class code is `030000`**
//! — base class 03 display, sub-class 00 VGA-compatible, programming interface
//! 00 (*PCI Local Bus Specification* Rev 2.1 Appendix D) — and gets that
//! function's option ROM off its **expansion ROM base address register**
//! (§6.2.5.2). On a 440FX board it *cannot* do anything else: the first thing it
//! does to the chipset is set the PAM registers for `0xc0000`-`0xdffff` to
//! read/write, which turns the whole ISA option-ROM window into blank DRAM
//! (`docs/platforms/pc-at.md`). Whatever is in a legacy ROM socket there is
//! invisible from that moment on, by design — that window is the firmware's
//! scratch pad for the ROMs it copies *into* it.
//!
//! So this is the object that answers "is there a VGA on this bus". It is
//! separate from [`video`](super::video), which is the CRTC, the character
//! buffer and the character generator, because the two answer different
//! questions and the division is testable: the guest reaches `pc.video` through
//! the board's own decode of `0x3c0`-`0x3df` and `0xb8000`, exactly as it
//! reaches an ISA card, and it reaches this through configuration space. One
//! card in a machine file is two objects for the same reason a drive and its
//! channel are ([`ide`](super::ide)) — and, unlike that one, this division costs
//! nothing in fidelity, because a PCI VGA's legacy decode is not addressed
//! through its BARs either.
//!
//! What that division *does* simplify away is `COMMAND[0]`/`COMMAND[1]` gating
//! the legacy `0x3c0`/`0xa0000` decode, and the VGA palette snoop bit. On a real
//! card those turn the legacy aperture off; here the board's `map` statements
//! hold it open unconditionally, which is what an ISA card does and what every
//! firmware assumes anyway. Nothing yet asks for the other behaviour.
//!
//! # What the firmware does with it
//!
//! 1. Reads the class code, decides this is the console.
//! 2. Sizes the expansion ROM register — writes all ones, reads back the size
//!    mask (§6.2.5.1's read-back rule, which §6.2.5.2's register also obeys).
//! 3. Writes a base into it, sets its enable bit **and** `COMMAND[1]`, because
//!    the memory-space bit has precedence over the ROM enable (§6.2.5.2).
//! 4. Reads the image out of the window it just created, copies it into the
//!    RAM it made at `0xc0000`, checks the `0x55 0xaa` signature and the
//!    checksum, and far-calls offset 3 of the copy.
//!
//! Steps 2 and 3 are what [`Bars`] is for; step 4 is why this device carries an
//! image at all.
//!
//! # The image, and what has to be in it
//!
//! The `image` media slot takes the same video BIOS the board's legacy socket
//! takes, and the window is the image rounded up to a power of two, at least the
//! 2 KiB §6.2.5.2's address field can express. **An empty slot means no
//! expansion ROM register at all** — it reads back as zero, which is Rev 2.1
//! §6.2.5.1's own way of saying "not implemented" — because a card with no ROM
//! fitted is an ordinary card and offering firmware a window full of erased
//! bytes would be worse than offering none.
//!
//! A firmware that loads a ROM off a BAR may insist on the **PCI Data
//! Structure** the PCI Firmware Specification puts at the offset named by the
//! two bytes at `0x18` of the image: the signature `PCIR`, and inside it the
//! vendor and device ID the image is for. That is a property of the *image*, not
//! of this device — but it is why `vendor-id` and `device-id` are properties
//! rather than hardwired. A video BIOS built for an ISA card has no `PCIR` and
//! no firmware will take it off a BAR; one built for a PCI card names the ids it
//! expects, and the machine file has to agree with it.
//!
//! # Sources
//!
//! *PCI Local Bus Specification, Revision 2.1* — §6.1 and §6.2 for the Type 00h
//! header, §6.2.2 for the Command register, §6.2.5.1 for base addresses and the
//! sizing read-back, §6.2.5.2 for the expansion ROM register, Appendix D for the
//! class codes. No emulator source was consulted (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::pci::{Bar, Bars, Bdf, ConfigSpace, PciBus, PciFunction, buses, config};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, Region, RegionRef, RomStore, RomWrite};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.vga-pci";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The smallest expansion ROM window §6.2.5.2's address field can express.
const MIN_ROM_LEN: u64 = 2048;

/// The largest one this class will allocate.
///
/// Not a hardware limit — a sanity bound, so that a mistyped image is an error
/// naming the property rather than an allocation that takes the host down. A
/// video BIOS is tens of kilobytes; 16 MiB is four hundred times that.
const MAX_ROM_LEN: u64 = 16 * 1024 * 1024;

/// What an unprogrammed cell of the ROM window reads as.
///
/// `0xff`, an erased EPROM byte, for the same reason
/// [`rom`](super::rom)'s socket pads with it: firmware that walks past the end
/// of the image must not find a second `0x55 0xaa` there.
const ERASED: u8 = 0xff;

/// Which bits of the Command register this function implements.
///
/// Rev 2.1 §6.2.2 lets a function hardwire to zero any bit it does not
/// implement, and this one implements the three that decide whether it decodes
/// anything — I/O space, memory space and bus master — plus the VGA palette
/// snoop at bit 5, which firmware sets on a display adapter and which is
/// latched here and drives nothing (see the module docs). Everything else reads
/// back zero.
const COMMAND_IMPLEMENTED: u16 =
    config::COMMAND_IO | config::COMMAND_MEMORY | config::COMMAND_MASTER | 0x0020;

/// The registers a configuration cycle reaches, and the ROM behind the BAR.
///
/// Separate from [`VgaPci`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` while [`Device::realize`] only ever has `&self` — the
/// same shape [`pmc`](super::pmc) uses for the same reason.
struct Registers {
    /// The 256 bytes of configuration space that are not base address
    /// registers. At [`LockRank::DEVICE`], released before anything outward.
    config: Mutex<ConfigSpace>,
    /// The base address registers, which own `0x10`-`0x27` and `0x30`-`0x33`.
    /// Its own locks are all [`LockRank::LEAF`] and it never holds one across
    /// a call.
    bars: Bars,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
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

impl Registers {
    /// The header this card hardwires (Rev 2.1 §6.1).
    fn fresh_config(vendor: u16, device: u16, revision: u8) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(vendor), 2);
        c.hardwire(config::DEVICE_ID, u32::from(device), 2);
        // §6.2.2: every enable bit clear out of reset, so the function decodes
        // nothing until firmware has finished sizing its windows.
        c.hardwire(config::COMMAND, 0x0000, 2);
        // §6.2.3: DEVSEL# timing 01b (medium) in bits 10:9, and nothing else to
        // report.
        c.hardwire(config::STATUS, 0x0200, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        // 030000h: programming interface 00, sub-class 00 (VGA-compatible),
        // base class 03 (display). This is the number the firmware looks for.
        c.hardwire(config::CLASS_CODE, 0x00, 1);
        c.hardwire(config::CLASS_CODE + 1, u32::from(config::SUBCLASS_VGA), 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(config::CLASS_DISPLAY), 1);
        // §6.2.1: header type 00h, single function.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);
        // §6.2.4: no interrupt pin. A VGA that raised one would have to be
        // wired to a south bridge, and this board has none.
        c.hardwire(config::INTERRUPT_PIN, 0x00, 1);

        c.allow(config::COMMAND, 2);
        c.allow(config::CACHE_LINE_SIZE, 1);
        c.allow(config::LATENCY_TIMER, 1);
        // Firmware writes which interrupt controller input it routed the pin
        // to. There is no pin, and it writes it anyway.
        c.allow(config::INTERRUPT_LINE, 1);
        c
    }

    /// The Command register as it stands.
    fn command(&self) -> u16 {
        let c = self.config.lock();
        u16::from(c.byte(config::COMMAND)) | u16::from(c.byte(config::COMMAND + 1)) << 8
    }
}

impl PciFunction for Registers {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: a configuration read of this function has no side
        // effects — no status bit a read clears, no FIFO to pop — so a
        // debugger's window may poll it freely. The sizing protocol is a
        // *write* followed by a read, and the write is what is refused.
        self.config.lock().read(offset, dst);
        // The BAR registers overwrite their own bytes and leave the rest.
        self.bars.config_read(offset, dst);
        // A retopology that could not happen when it was asked for gets its
        // next chance here, for the reason [`pmc`](super::pmc) gives: leaving
        // the machine's memory map disagreeing with its own registers is worse
        // than a debugger's read having an invisible effect on neither.
        if self.bars.is_stale() {
            self.bars.sync(self.command(), false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here would move a BAR under the guest's feet, which
            // is exactly what `MemAttrs::debug` exists to forbid.
            // `ConfigPorts` refuses one before it reaches here; this is the
            // second lock on the same door, for a caller that reaches the
            // function directly.
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
        // Only when something that decides a window actually moved. Firmware
        // rewrites the same base constantly while it sizes, and re-flattening
        // an address space for a write that changed nothing is pure cost.
        if bars_moved || command_moved || self.bars.is_stale() {
            self.bars.sync(command, false);
        }
    }
}

/// A PCI display adapter's configuration face, with its video BIOS behind the
/// expansion ROM base address register.
#[derive(Debug)]
pub struct VgaPci {
    regs: Arc<Registers>,
    bus: Arc<PciBus>,
    at: Bdf,
    vendor: u16,
    device: u16,
    revision: u8,
    /// The bytes behind the expansion ROM window, for a test that wants to see
    /// what the firmware will read. `None` where no image was bound.
    rom: Option<Arc<RomStore>>,
}

impl VgaPci {
    /// Validate `props` and build the device.
    ///
    /// Allocation and validation only: the fabric handle is acquired here
    /// because acquiring a host object *is* allocation
    /// ([`core::hosts`](crate::core::hosts)), and nothing is announced onto it
    /// until [`realize`](Device::realize).
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, a device or
    /// function number off the bus, an identifier that does not fit sixteen
    /// bits, or an image larger than 16 MiB; [`Error::Config`] if the
    /// `bus` name is already open as something else.
    pub fn new(props: &Props) -> Result<VgaPci> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device_no = r.or_range("device", 2u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let function_no = r.or_range(
            "function",
            0u64,
            0..=u64::from(crate::bus::pci::MAX_FUNCTION),
        )?;
        let vendor = r.or_range("vendor-id", 0x1234u64, 0..=0xffff)?;
        let device = r.or_range("device-id", 0x1111u64, 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let image = r.require_media("image")?.to_bytes();
        r.finish()?;
        let bus = buses::attach(props, &bus_name)?;
        let at = Bdf::new(0, device_no as u8, function_no as u8)?;
        VgaPci::with_bus(
            bus,
            at,
            vendor as u16,
            device as u16,
            revision as u8,
            &image,
        )
    }

    /// The same card, built from a fabric handle a test already has.
    ///
    /// An empty `image` fits no expansion ROM register at all — see the module
    /// docs.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the image is larger than 16 MiB;
    /// [`Error::Config`] if the BAR table refuses the window, which would be a
    /// bug in this file rather than anything a caller can cause.
    pub fn with_bus(
        bus: Arc<PciBus>,
        at: Bdf,
        vendor: u16,
        device: u16,
        revision: u8,
        image: &[u8],
    ) -> Result<VgaPci> {
        let mut bars = Bars::new();
        let mut rom = None;
        if !image.is_empty() {
            let len = image.len() as u64;
            if len > MAX_ROM_LEN {
                return Err(Error::Property(alloc::format!(
                    "property `image`: an expansion ROM here holds at most {MAX_ROM_LEN} bytes \
                     and this image is {len}"
                )));
            }
            // §6.2.5.1: a window is a power of two, and §6.2.5.2's address
            // field starts at bit 11, so the smallest one expressible is 2 KiB.
            let window = len.next_power_of_two().max(MIN_ROM_LEN);
            let mut bytes = alloc::vec![ERASED; window as usize];
            bytes[..image.len()].copy_from_slice(image);
            let store = Arc::new(RomStore::new(bytes));
            let region: RegionRef = Arc::new(Region::rom(
                "pc.vga-pci.rom",
                Arc::clone(&store),
                // A write to a ROM is swallowed, not faulted, exactly as the
                // legacy socket's is: firmware writes all ones to a BAR while
                // sizing, and some of it writes to the window too.
                RomWrite::Ignore,
            ));
            bars = bars.with(
                Bars::ROM,
                Bar::rom(window).decoding(region, crate::core::space::Perms::RX),
            )?;
            rom = Some(store);
        }
        Ok(VgaPci {
            regs: Arc::new(Registers {
                config: Mutex::with_rank(
                    LockRank::DEVICE,
                    Registers::fresh_config(vendor, device, revision),
                ),
                bars,
            }),
            bus,
            at,
            vendor,
            device,
            revision,
            rom,
        })
    }

    /// Where this card sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The bytes behind the expansion ROM window, or `None` where no image was
    /// bound.
    #[must_use]
    pub fn rom(&self) -> Option<&Arc<RomStore>> {
        self.rom.as_ref()
    }

    /// The base address registers, for a test that wants to see where a window
    /// went.
    #[must_use]
    pub fn bars(&self) -> &Bars {
        &self.regs.bars
    }

    /// The Command register as it stands.
    #[must_use]
    pub fn command(&self) -> u16 {
        self.regs.command()
    }

    /// Put this card's windows into `space`. **Retopology.**
    ///
    /// What [`Instance::bind`] does, reachable directly so a unit test can
    /// assemble a card without a machine.
    ///
    /// # Errors
    ///
    /// Whatever the space refuses: a window that does not fit, or a nesting
    /// depth this space will not take.
    pub fn attach_space(&self, space: &Arc<AddressSpace>) -> Result<()> {
        self.regs.bars.install(space, self.regs.command())
    }
}

/// The `pc.vga-pci` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a PCI display adapter's configuration header and its video BIOS's expansion ROM",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric this card is on (default `pci0`)",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device number it answers at on bus 0 (default 2)",
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
            summary: "the vendor identification the video BIOS expects to find (default 0x1234)",
        },
        PropertySpec {
            name: "device-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device identification the video BIOS expects to find (default 0x1111)",
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
            required: true,
            summary: "the media slot the video BIOS is bound to; empty fits no expansion ROM",
        },
    ],
    construct: |props| Ok(Box::new(VgaPci::new(props)?)),
};

impl Device for VgaPci {
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
        // `PCIRST#` clears the configuration registers, and rsemu's warm reset
        // stands in for it. Both the enable bits and the bases go, so the card
        // decodes nothing again — which is the state firmware expects to find
        // when it starts enumerating.
        *self.regs.config.lock() = Registers::fresh_config(self.vendor, self.device, self.revision);
        self.regs.bars.reset();
        // The ROM image is the machine's, not the run's, so a cold reset has
        // nothing extra to do: there is no writable store here to clear.
        //
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.bars.sync(self.regs.command(), true);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(self.regs.config.lock().bytes())?;
        for value in self.regs.bars.latches() {
            w.write_u32(value)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let config: &[u8] = r.read_bytes()?;
        let mut latches = [0u32; Bars::COUNT as usize];
        for slot in &mut latches {
            *slot = r.read_u32()?;
        }
        {
            let mut c = self.regs.config.lock();
            *c = Registers::fresh_config(self.vendor, self.device, self.revision);
            c.restore(config);
        }
        self.regs.bars.set_latches(&latches);
        // Where the windows are is a function of the registers, so it is
        // rebuilt rather than saved (`CLAUDE.md`: derived state is never
        // serialized).
        self.regs.bars.sync(self.regs.command(), true);
        Ok(())
    }
}

impl Instance for VgaPci {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a card's base address registers place windows in a memory space, so it needs \
                 the space they are placed in: add `space = mem` to the object that declares it",
            ),
        })?;
        self.attach_space(space)
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(VgaPci::new(props)?)))
}

/// What the validator should know about `pc.vga-pci`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::PropSchema;
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
        .prop(PropSchema::new("image", ValueKind::Media).required())
}

#[cfg(test)]
mod tests;
