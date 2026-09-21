//! A PCI IDE controller in **native mode**: the function whose ports are
//! wherever its base address registers say they are.
//!
//! # Why this device exists, and what it is not
//!
//! [`super::ide`] is one IDE channel's address decode, and on a 1984-lineage
//! board that decode is *fixed*: `0x1f0` and `0x3f6` for the primary channel,
//! `0x170` and `0x376` for the secondary, because that is what the board's
//! address decoder was wired for. Every PC firmware and every DOS driver knows
//! those four numbers by heart, and a controller that answers there is in what
//! the PCI class code calls **compatibility mode**.
//!
//! A PCI IDE controller may instead be in **native mode**, where the four
//! windows are named by four base address registers and firmware puts them
//! wherever it likes. That is the whole difference this file models, and it is
//! worth being precise about how little else changes:
//!
//! * **There is no ATA in this file.** Not one command opcode, not one status
//!   bit. It does not even know how many drives are on the cable. The channel
//!   is [`super::ide`]'s and the drives are `crate::dev::ata`'s, exactly as in
//!   compatibility mode; this function *supplies the same two regions to base
//!   address registers instead of to a `map` statement*.
//! * **There is no bus-master DMA**, so the programming interface byte does not
//!   claim any (bit 7 clear) and there is no fifth base address register. A
//!   driver that reads the class code learns that, and falls back to PIO, which
//!   is what it would do against a real controller with the bit clear.
//! * **The interrupt is still a board wire.** A real native-mode part routes
//!   each channel's `INTRQ` onto its `INTA#` pin, and firmware reads the
//!   Interrupt Line register to find which controller input that pin reaches.
//!   `pc-at` has no south bridge and therefore no `PIRQ` router to route it
//!   *to* (`src/fw/pcbios/pci.rs` says the same thing about `B10Eh`), so the
//!   machine file wires each channel's own `irq` port to the 8259A as it always
//!   did, and this function's Interrupt Pin register reads zero — Rev 2.1
//!   §6.2.4's own encoding for "this function does not use an interrupt pin".
//!   That is a statement about this board, not a shortcut: a function claiming
//!   a pin that reaches nothing would be worse than one claiming none.
//!
//! # The four registers
//!
//! *PCI IDE Controller Specification*, revision 1.0, and the class code
//! encoding in *PCI Local Bus Specification* Rev 2.1 Appendix D:
//!
//! ```text
//!   BAR0   8 bytes of I/O   primary   command block   (data .. status)
//!   BAR1   4 bytes of I/O   primary   control block   at offset 2
//!   BAR2   8 bytes of I/O   secondary command block
//!   BAR3   4 bytes of I/O   secondary control block   at offset 2
//! ```
//!
//! The control-block window is **four** bytes wide and exactly **one** of them
//! is decoded, at offset 2: the Device Control / Alternate Status register. So
//! a driver computes `BAR1 + 2` and finds the same register a compatibility-mode
//! driver finds at `0x3f6`. [`Bar::at_offset`] is how that is said here, and it
//! is why this file needs nothing from [`super::ide`] that a compatibility-mode
//! board does not already take.
//!
//! A channel the machine file did not name gets **no** base address register at
//! all rather than an empty one: Rev 2.1 §6.2.5.1 makes an unimplemented
//! register read as zero, which is how firmware knows to stop looking, and a
//! sizeable register behind which nothing answers is precisely the board bug
//! [`Bars::install`] refuses.
//!
//! ## The programming interface byte
//!
//! Appendix D gives base class `01h` (mass storage), sub-class `01h` (IDE), and
//! a programming interface byte whose bits are:
//!
//! ```text
//!   bit 0  primary channel is in native mode
//!   bit 1  primary channel's mode is programmable
//!   bit 2  secondary channel is in native mode
//!   bit 3  secondary channel's mode is programmable
//!   bit 7  the controller is bus-master capable
//! ```
//!
//! This part is native on both channels and switchable on neither, and has no
//! bus-master engine, so the byte is `05h` — and a channel that is not fitted
//! still reads its mode bit set, because the mode is a property of the
//! controller's decode rather than of what is plugged into it.
//!
//! # Why an I/O base address register is the interesting part
//!
//! Because moving one moves a window in the space the configuration write that
//! moves it is travelling through. `src/bus/pci/bar.rs` carries that argument in
//! full; the short form is that the try-lock cannot succeed, the retry cannot be
//! another configuration cycle, and what places the window is the host bridge's
//! scheduler drain a moment later. This device is the first thing in the tree
//! with an I/O BAR, and it is why that drain now exists on a 440FX board
//! ([`super::pmc`]) and not only on a q35.
//!
//! # Sources
//!
//! * *PCI Local Bus Specification*, Revision 2.1 — §6.1 and §6.2 for the header,
//!   §6.2.2 for `COMMAND[0]`, §6.2.4 for the interrupt registers, §6.2.5.1 for
//!   the base address registers, Appendix D for the class code.
//! * *PCI IDE Controller Specification*, revision 1.0 — which base address
//!   register is which channel's, and that the control block's window is four
//!   bytes with the register at offset 2.
//! * T13's ATA/ATAPI-6 for what is behind those windows, which is
//!   [`super::ide`]'s business and not this file's.
//!
//! No emulator source was consulted for any of it (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::pci::{Bar, BarSpaces, Bars, Bdf, ConfigSpace, PciBus, PciFunction, buses, config};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.ide-pci";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How wide a command-block window is: the eight ports of ATA/ATAPI-6's command
/// block, which is also the smallest power of two that holds them.
const COMMAND_WINDOW: u64 = 8;

/// How wide a control-block window is.
///
/// Four, not one. The *PCI IDE Controller Specification* gives the control
/// block a four-byte I/O window and decodes one byte of it; §6.2.5.1's minimum
/// for an I/O window is four anyway, so there was never a one-byte register to
/// declare.
const CONTROL_WINDOW: u64 = 4;

/// Where in the control-block window the one decoded byte is.
const CONTROL_OFFSET: u64 = 2;

/// Base class 01h: a mass storage controller (Appendix D).
const CLASS_STORAGE: u8 = 0x01;

/// Sub-class 01h: an IDE controller.
const SUBCLASS_IDE: u8 = 0x01;

/// The programming interface byte: native on both channels, switchable on
/// neither, no bus master. See the module docs for the bit assignments.
const PROG_IF_NATIVE: u8 = 0x05;

/// Which bits of the Command register this function implements.
///
/// Rev 2.1 §6.2.2 lets a function hardwire to zero any bit it does not
/// implement. This one decodes I/O space and nothing else — there is no memory
/// window and no bus-master engine — so `COMMAND[0]` is the only bit that does
/// anything, and it is the only one that reads back.
const COMMAND_IMPLEMENTED: u16 = config::COMMAND_IO;

/// What the Interrupt Line register reads out of reset.
///
/// Rev 2.1 §6.2.4: 255 means "unknown, or no connection". Which is the truth on
/// a board with no `PIRQ` router — see the module docs.
const INTERRUPT_LINE_NONE: u8 = 0xff;

/// One channel's pair of registers.
struct ChannelBars {
    /// The base address register the command block answers at.
    command: u8,
    /// The base address register the control block answers at.
    control: u8,
}

/// The two channels, in register order. A Type 00h header has six base address
/// registers and this part uses four of them; 4 and 5 read as zero, which is
/// how firmware learns there is no bus-master window here.
const CHANNELS: [ChannelBars; 2] = [
    // `primary`
    ChannelBars {
        command: 0,
        control: 1,
    },
    // `secondary`
    ChannelBars {
        command: 2,
        control: 3,
    },
];

/// The registers a configuration cycle reaches.
///
/// Separate from [`IdePci`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` while `Device::realize` only ever has `&self` — the
/// same shape [`super::vgapci`] uses for the same reason.
struct Registers {
    /// The 256 bytes that are not base address registers. At
    /// [`LockRank::DEVICE`], released before anything outward.
    config: Mutex<ConfigSpace>,
    /// The base address registers, which own `0x10`-`0x27`. Its own locks are
    /// all [`LockRank::LEAF`] and it never holds one across a call.
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
    /// The header this part hardwires (Rev 2.1 §6.1, Appendix D).
    fn fresh_config(vendor: u16, device: u16, revision: u8) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(vendor), 2);
        c.hardwire(config::DEVICE_ID, u32::from(device), 2);
        // §6.2.2: every enable bit clear out of reset, so nothing decodes until
        // firmware has finished sizing the windows.
        c.hardwire(config::COMMAND, 0x0000, 2);
        // §6.2.3: DEVSEL# timing 01b (medium) in bits 10:9, and nothing to
        // report besides.
        c.hardwire(config::STATUS, 0x0200, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        c.hardwire(config::CLASS_CODE, u32::from(PROG_IF_NATIVE), 1);
        c.hardwire(config::CLASS_CODE + 1, u32::from(SUBCLASS_IDE), 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(CLASS_STORAGE), 1);
        // §6.2.1: header type 00h, single function.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);
        // §6.2.4: no interrupt pin, because there is nothing on this board for
        // one to reach. The module docs argue it.
        c.hardwire(config::INTERRUPT_PIN, 0x00, 1);
        c.hardwire(config::INTERRUPT_LINE, u32::from(INTERRUPT_LINE_NONE), 1);

        c.allow(config::COMMAND, 2);
        c.allow(config::CACHE_LINE_SIZE, 1);
        c.allow(config::LATENCY_TIMER, 1);
        // Firmware writes which controller input it routed the pin to. There is
        // no pin, and firmware writes it anyway, so the byte is writable and
        // connected to nothing.
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
        // effects. The sizing protocol is a *write* followed by a read, and the
        // write is what a debugger is refused.
        self.config.lock().read(offset, dst);
        self.bars.config_read(offset, dst);
        if self.bars.is_stale() {
            self.bars.sync(self.command(), false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here would move a window under the guest's feet,
            // which is exactly what `MemAttrs::debug` exists to forbid.
            return;
        }
        let bars_moved = self.bars.config_write(offset, src);
        let (command_moved, command) = {
            let mut c = self.config.lock();
            let moved = c.write(offset, src);
            // §6.2.2: an unimplemented Command bit is hardwired to zero, so a
            // write that sets one reads back as zero.
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
        if bars_moved || command_moved || self.bars.is_stale() {
            // **Always** the try-lock, and here it can never succeed on a board
            // whose configuration cycles are port cycles: this write is itself
            // an `OUT` to `0xcfc`, travelling through the very space the window
            // is in. So this call reliably fails, sets the stale flag, and
            // returns — and the host bridge's scheduler drain is what actually
            // places the window. `src/bus/pci/bar.rs` has the argument.
            self.bars.sync(command, false);
        }
    }

    fn retopology_owed(&self) -> bool {
        self.bars.is_stale()
    }

    fn settle(&self) {
        // Still the try-lock: this runs from `Device::advance_to` with no
        // access of *this* thread's in flight, but a sibling thread's access
        // can hold the space and blocking there would invert the ladder.
        self.bars.sync(self.command(), false);
    }
}

/// A PCI IDE controller's configuration face.
#[derive(Debug)]
pub struct IdePci {
    regs: Arc<Registers>,
    bus: Arc<PciBus>,
    at: Bdf,
    vendor: u16,
    device: u16,
    revision: u8,
    /// The name of the address space the windows go in. `space =` is structural
    /// and there is one of it, and this function's windows are all in the
    /// *other* space, so it is named — the convention `crate::cpu::x86`'s
    /// `iospace` property established.
    iospace: String,
    /// Which sibling object each channel is, if the machine file named one.
    /// Resolved at [`Instance::bind`], because that is the first moment two
    /// independently constructed objects can be introduced.
    channels: Mutex<[Option<String>; 2]>,
}

impl IdePci {
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
    /// bits, or neither channel named — a controller with no channel decodes
    /// nothing and is a machine-description mistake rather than a configuration.
    pub fn new(props: &Props) -> Result<IdePci> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device_no = r.or_range("device", 1u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let function_no = r.or_range(
            "function",
            0u64,
            0..=u64::from(crate::bus::pci::MAX_FUNCTION),
        )?;
        let vendor = r.or_range("vendor-id", 0x1234u64, 0..=0xffff)?;
        let device = r.or_range("device-id", 0x1230u64, 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let iospace = r.or_str("iospace", "port")?.to_string();
        let channels = [
            r.optional_link("primary")?
                .map(|l| String::from(l.as_str())),
            r.optional_link("secondary")?
                .map(|l| String::from(l.as_str())),
        ];
        r.finish()?;
        if channels.iter().all(Option::is_none) {
            return Err(Error::Property(String::from(
                "`pc.ide-pci` decodes the channels it is given and has none: name at least one \
                 `pc.ide` object as `primary` or `secondary`",
            )));
        }
        let bus = buses::attach(props, &bus_name)?;
        let at = Bdf::new(0, device_no as u8, function_no as u8)?;
        let fitted = [channels[0].is_some(), channels[1].is_some()];
        let mut card = IdePci::with_bus(
            bus,
            at,
            vendor as u16,
            device as u16,
            revision as u8,
            fitted,
        )?;
        card.iospace = iospace;
        *card.channels.lock() = channels;
        Ok(card)
    }

    /// The same controller, built from a fabric handle a test already has.
    ///
    /// `fitted` says which of the two channels exists; a channel that does not
    /// gets no base address register, because §6.2.5.1 makes an unimplemented
    /// register read as zero and that is how firmware stops looking.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the register table refuses a window, which would be
    /// a bug in this file rather than anything a caller can cause.
    pub fn with_bus(
        bus: Arc<PciBus>,
        at: Bdf,
        vendor: u16,
        device: u16,
        revision: u8,
        fitted: [bool; 2],
    ) -> Result<IdePci> {
        let mut bars = Bars::new();
        for (channel, present) in CHANNELS.iter().zip(fitted.iter()) {
            if !present {
                continue;
            }
            bars = bars.with(channel.command, Bar::io(COMMAND_WINDOW))?;
            bars = bars.with(
                channel.control,
                Bar::io(CONTROL_WINDOW).at_offset(CONTROL_OFFSET),
            )?;
        }
        Ok(IdePci {
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
            iospace: String::from("port"),
            channels: Mutex::with_rank(LockRank::LEAF, [None, None]),
        })
    }

    /// Where this controller sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
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

    /// Put `region` behind one channel's command block.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if that channel was not declared, or the region does
    /// not fit its window.
    pub fn attach_command(&self, channel: usize, region: RegionRef) -> Result<()> {
        let which = CHANNELS.get(channel).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: String::from("an IDE controller has two channels, 0 and 1"),
        })?;
        self.regs.bars.supply(which.command, region)
    }

    /// Put `region` behind one channel's control block.
    ///
    /// # Errors
    ///
    /// As [`attach_command`](IdePci::attach_command).
    pub fn attach_control(&self, channel: usize, region: RegionRef) -> Result<()> {
        let which = CHANNELS.get(channel).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: String::from("an IDE controller has two channels, 0 and 1"),
        })?;
        self.regs.bars.supply(which.control, region)
    }

    /// Put this controller's windows into `space`. **Retopology.**
    ///
    /// What [`Instance::bind`] does, reachable directly so a unit test can
    /// assemble one without a machine.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a declared register has no region to decode, which
    /// is a machine description that named a channel and did not supply it.
    pub fn attach_space(&self, space: &Arc<AddressSpace>) -> Result<()> {
        self.regs
            .bars
            .install(&BarSpaces::new().io(space), self.regs.command())
    }
}

/// The `pc.ide-pci` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a PCI IDE controller in native mode: each channel's ports wherever its BAR says",
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
            summary: "the device number it answers at on bus 0 (default 1)",
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
            summary: "the vendor identification it hardwires (default 0x1234)",
        },
        PropertySpec {
            name: "device-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device identification it hardwires (default 0x1230)",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the revision identification byte (default 0)",
        },
        PropertySpec {
            name: "iospace",
            kind: ValueKind::Str,
            required: false,
            summary: "the I/O address space the windows are placed in (default `port`)",
        },
        PropertySpec {
            name: "primary",
            kind: ValueKind::Link,
            required: false,
            summary: "the `pc.ide` channel BAR0 and BAR1 decode",
        },
        PropertySpec {
            name: "secondary",
            kind: ValueKind::Link,
            required: false,
            summary: "the `pc.ide` channel BAR2 and BAR3 decode",
        },
    ],
    construct: |props| Ok(Box::new(IdePci::new(props)?)),
};

impl Device for IdePci {
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
        // `PCIRST#` clears the configuration registers. Both the enable bit and
        // the bases go, so the controller decodes nothing again — which is the
        // state firmware expects to find when it starts enumerating, and the
        // reason a warm reset does not leave a guest's ports where the previous
        // guest put them.
        *self.regs.config.lock() = Registers::fresh_config(self.vendor, self.device, self.revision);
        self.regs.bars.reset();
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight, which is the one place the blocking guard is
        // legal.
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
        // serialized). Blocking, because a load runs with nothing in flight.
        self.regs.bars.sync(self.regs.command(), true);
        Ok(())
    }
}

impl Instance for IdePci {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx
            .space_named(&self.iospace)
            .ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: alloc::format!(
                    "`iospace = \"{}\"` names no address space in this machine; a native-mode \
                     IDE controller places its windows in the processor's I/O space",
                    self.iospace
                ),
            })?;
        let named = self.channels.lock().clone();
        for (index, path) in named.iter().enumerate() {
            let Some(path) = path else { continue };
            // `cmd` and `ctl` are the two regions `pc.ide` publishes, and they
            // are the same two a compatibility-mode board reaches with a `map`
            // statement. The controller adds the *address*, which is the whole
            // of what native mode means.
            self.attach_command(index, ctx.region(path, "cmd")?)?;
            self.attach_control(index, ctx.region(path, "ctl")?)?;
        }
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(IdePci::new(props)?)))
}

/// What the validator should know about `pc.ide-pci`.
#[must_use]
pub fn schema() -> ClassSchema {
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
        .prop(PropSchema::new("iospace", ValueKind::Str))
        .prop(PropSchema::new("primary", ValueKind::Link))
        .prop(PropSchema::new("secondary", ValueKind::Link))
}

#[cfg(test)]
mod tests;
