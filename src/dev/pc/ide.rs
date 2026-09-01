//! The AT's IDE interface: one channel, its two register blocks and its
//! interrupt.
//!
//! **This is address decode and nothing else**, which is the honest model of
//! what an AT-class board contributes to a hard disk. "IDE" means *integrated
//! drive electronics*: the controller that was a card in a PC/XT moved onto the
//! drive in 1986, and what stayed behind is a pair of chip selects, a
//! bidirectional buffer on sixteen data lines, and one wire to an 8259A. The
//! drive itself is [`crate::dev::ata::disk`], it is a separate object in a
//! machine description, and it does not know this file exists.
//!
//! The falsifiable form of the split, which is the point of stating it:
//!
//! * **There is not one ATA command opcode in this file.** No `0xec`, no
//!   `IDENTIFY`, no `READ SECTORS`.
//! * **There is not one `IDENTIFY` word index, and not one status- or
//!   error-register bit.** This file cannot tell you what `BSY` is worth.
//! * Symmetrically, `src/dev/ata/disk.rs` contains no I/O port address and no
//!   register offset.
//!
//! What is left is the table in [`register_at`] — eight offsets to eight
//! register *names* — and the rules of a cable with two drives on it.
//!
//! # The two register blocks
//!
//! An AT decodes a channel at two disjoint places, which is why this device
//! publishes two regions rather than one:
//!
//! ```text
//!   command block   0x1f0-0x1f7 (primary) / 0x170-0x177 (secondary)
//!     +0  data (16 bit)            +4  LBA mid   / cylinder low
//!     +1  features / error         +5  LBA high  / cylinder high
//!     +2  sector count             +6  device: DEV, LBA, head
//!     +3  LBA low   / sector       +7  command / status
//!
//!   control block   0x3f6 (primary) / 0x376 (secondary)
//!     +0  device control (write) / alternate status (read)
//! ```
//!
//! The primary channel's control port is **0x3f6**, which is inside the range
//! the floppy adapter is usually drawn as owning. That is not a mistake in
//! either device: on a real AT the diskette adapter decodes `0x3f0-0x3f5` and
//! `0x3f7`, and `0x3f6` belongs to the fixed-disk adapter. `0x3f7` is shared —
//! the floppy drives bit 7 and the hard disk adapter bits 6:0 — and
//! `machines/pc-at.machine` splits the floppy's window in two around `0x3f6`
//! for exactly this reason.
//!
//! # `MemAttrs::debug` is load-bearing here
//!
//! Two of these sixteen accesses have side effects a debugger must not cause,
//! and the hardware itself provides the escape hatch for one of them:
//!
//! * Reading **status** at command-block offset 7 clears the pending interrupt.
//! * Reading **data** at offset 0 advances the sector buffer.
//!
//! Both are suppressed under `attrs.debug`, and passed through to the drive as a
//! flag rather than decided here (`ROADMAP.md` §15, invariant 5). The
//! **alternate status** register in the control block is the same eight bits
//! with no interrupt acknowledge attached — it exists on real hardware so that a
//! driver can look without acknowledging, which makes it debug-safe by
//! construction and means the control block needs no special case at all.
//!
//! A debug **write** is refused outright: a write to the command register starts
//! a command and a write to device control resets the drives, and neither can be
//! made harmless.
//!
//! # Two drives on one cable
//!
//! A write to any command block register goes to **both** drives; each decides
//! whether it is being addressed by comparing the Device register's DEV bit with
//! the position it is jumpered to. A read is answered by whichever drive says it
//! is selected. Three cases, and the third is how a driver probes:
//!
//! * the selected drive is there — it answers;
//! * the selected position is empty but the other is occupied — **zero**,
//!   because the drive that is there answers for the one that is not, and a
//!   status register of zero is what tells a driver "nothing here";
//! * both empty — ones, because nothing is driving the bus at all and an ISA
//!   bus with nothing driving it reads as ones.
//!
//! # Interrupts
//!
//! One output pin, `irq`, driven by the selected drive's `INTRQ` — gated by its
//! own nIEN bit, which is the drive's business and not this file's. On a PC/AT
//! the primary channel lands on IRQ 14 and the secondary on IRQ 15, both on the
//! slave 8259A; which one is the machine file's business and not this file's
//! either.
//!
//! # Sources
//!
//! T13's ATA/ATAPI-6 for the register blocks and the cable's rules, the *IBM
//! Personal Computer AT Technical Reference* for the fixed-disk adapter's port
//! and interrupt assignments, and Ralf Brown's Interrupt List for the `0x3f6` /
//! `0x3f7` split with the diskette adapter. **No emulator source was consulted**
//! (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::ata::bays::{self, Bay};
use crate::dev::ata::disk::{AtaDisk, Reg};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.ide";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the command block answers: eight ports.
pub const COMMAND_WINDOW_LEN: u64 = 8;

/// How much address space the control block answers: one port.
pub const CONTROL_WINDOW_LEN: u64 = 1;

/// The bay name device 0 gets when a machine description does not say.
pub const DEFAULT_MASTER_BAY: &str = "ata0";

/// The bay name device 1 gets when a machine description does not say.
pub const DEFAULT_SLAVE_BAY: &str = "ata1";

/// What one offset in the command block selects.
///
/// The entire ATA content of this file. Eight offsets to eight names; the
/// names' meanings live on the drive, where the silicon that implements them
/// lives.
#[must_use]
pub fn register_at(offset: u64) -> Reg {
    match offset & 7 {
        0 => Reg::Data,
        1 => Reg::Feature,
        2 => Reg::SectorCount,
        3 => Reg::LbaLow,
        4 => Reg::LbaMid,
        5 => Reg::LbaHigh,
        6 => Reg::Device,
        _ => Reg::Command,
    }
}

/// Where the adapter's interrupt cell sits in the ranked lock order.
///
/// [`LockRank::LEAF`], so the wire can be driven with nothing else held — the
/// re-entrancy contract every chip in [`crate::dev::pc`] follows.
const PIN_RANK: LockRank = LockRank::LEAF;

// ---------------------------------------------------------------------------
// The channel
// ---------------------------------------------------------------------------

/// One IDE channel: two drive bays and an interrupt line.
///
/// It has no registers of its own, which is why this device has no `save`/
/// `load`: every bit a guest can observe through these ports lives on a drive,
/// and the drives are separate objects that snapshot themselves.
struct Channel {
    bays: [Arc<Bay>; 2],
    names: [String; 2],
    irq_out: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Channel")
            .field("master", &self.names[0])
            .field("slave", &self.names[1])
            .field("master_present", &self.bays[0].is_occupied())
            .field("slave_present", &self.bays[1].is_occupied())
            .finish()
    }
}

impl Channel {
    /// Both drives, looked up and the bay locks released — nothing outward
    /// happens while a bay is held.
    fn drives(&self) -> [Option<Arc<AtaDisk>>; 2] {
        [self.bays[0].drive(), self.bays[1].drive()]
    }

    /// The drive that answers a read, if any.
    fn answering(drives: &[Option<Arc<AtaDisk>>; 2]) -> Option<&Arc<AtaDisk>> {
        drives.iter().flatten().find(|drive| drive.is_selected())
    }

    /// Read one command block register.
    fn read_reg(&self, reg: Reg, debug: bool) -> u16 {
        let drives = self.drives();
        let value = match Channel::answering(&drives) {
            Some(drive) => drive.read_reg(reg, debug),
            None => nobody_home(&drives),
        };
        drop(drives);
        if !debug {
            self.refresh();
        }
        value
    }

    /// Write one command block register, to every drive on the cable.
    fn write_reg(&self, reg: Reg, value: u16) {
        for drive in self.drives().iter().flatten() {
            drive.write_reg(reg, value);
        }
        self.refresh();
    }

    /// The alternate status register: the same eight bits as status, with no
    /// interrupt acknowledge attached.
    fn read_alt_status(&self) -> u8 {
        let drives = self.drives();
        match Channel::answering(&drives) {
            Some(drive) => drive.read_alt_status(),
            None => nobody_home(&drives) as u8,
        }
    }

    /// Write the device control register, to every drive on the cable.
    fn write_control(&self, value: u8) {
        for drive in self.drives().iter().flatten() {
            drive.write_device_control(value);
        }
        self.refresh();
    }

    /// Recompute the interrupt output, with no drive or bay lock held.
    fn refresh(&self) {
        let drives = self.drives();
        let level = Channel::answering(&drives).is_some_and(|drive| drive.irq_asserted());
        drop(drives);
        let pin = self.irq_out.lock().clone();
        if let Some(pin) = pin {
            pin.set(Level::from_bool(level));
        }
    }
}

/// What the bus reads when the selected position is empty.
///
/// Zero if the *other* position is occupied, because the drive that is there
/// answers for the one that is not and a status register reading zero is what
/// tells a driver there is nothing at that address; ones if the cable is empty
/// altogether, because then nothing is driving it and the ISA bus's pull-ups
/// win.
fn nobody_home(drives: &[Option<Arc<AtaDisk>>; 2]) -> u16 {
    if drives.iter().any(Option::is_some) {
        0x0000
    } else {
        0xffff
    }
}

// ---------------------------------------------------------------------------
// The two apertures
// ---------------------------------------------------------------------------

/// The command block: eight ports, one of which is sixteen bits wide.
#[derive(Debug)]
struct CommandBlock(Arc<Channel>);

impl MemOps for CommandBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let reg = register_at(offset);
        match (reg, dst.len()) {
            // The data port, a word at a time — `insw`, which is how every
            // driver moves a sector.
            (Reg::Data, 2) => {
                let word = self.0.read_reg(Reg::Data, attrs.debug);
                dst.copy_from_slice(&word.to_le_bytes());
                Ok(())
            }
            // Two words, for an adapter on a 32-bit local bus. Not what an
            // AT's cable does, and free to allow.
            (Reg::Data, 4) => {
                let lo = self.0.read_reg(Reg::Data, attrs.debug);
                let hi = self.0.read_reg(Reg::Data, attrs.debug);
                dst[..2].copy_from_slice(&lo.to_le_bytes());
                dst[2..].copy_from_slice(&hi.to_le_bytes());
                Ok(())
            }
            // A byte from the data port still shifts a whole word out of the
            // drive, because the drive has no idea how wide the host's cycle
            // was; the high half is lost in the adapter's buffer, exactly as it
            // is on the board.
            (Reg::Data, 1) => {
                dst[0] = self.0.read_reg(Reg::Data, attrs.debug) as u8;
                Ok(())
            }
            (reg, 1) => {
                dst[0] = self.0.read_reg(reg, attrs.debug) as u8;
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A write to the command register starts a command and a write to
            // the data register fills a sector buffer. Neither can be made
            // harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let reg = register_at(offset);
        match (reg, src.len()) {
            (Reg::Data, 2) => {
                self.0
                    .write_reg(Reg::Data, u16::from_le_bytes([src[0], src[1]]));
                Ok(())
            }
            (Reg::Data, 4) => {
                self.0
                    .write_reg(Reg::Data, u16::from_le_bytes([src[0], src[1]]));
                self.0
                    .write_reg(Reg::Data, u16::from_le_bytes([src[2], src[3]]));
                Ok(())
            }
            (reg, 1) => {
                self.0.write_reg(reg, u16::from(src[0]));
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, word, and — for an adapter that is not on a 16-bit ISA bus —
        // doubleword. Alignment is not required, because the register file is
        // eight byte-wide ports and only offset zero accepts anything wider;
        // `read` and `write` reject the combinations that are not real.
        AccessConstraints::IO.with_widths(Width::U8, Width::U32)
    }
}

/// The control block: one port, and the reason a driver can look at the status
/// without acknowledging an interrupt.
#[derive(Debug)]
struct ControlBlock(Arc<Channel>);

impl MemOps for ControlBlock {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // No `attrs.debug` case, and that is a claim rather than an omission:
        // the alternate status register has no side effect to suppress. It is
        // the register the hardware provides *because* reading status
        // acknowledges.
        *byte = self.0.read_alt_status();
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // SRST lives in this register. A debug write would reset both
            // drives.
            return Err(BusError::BadAccess);
        }
        self.0.write_control(*value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// One IDE channel as a PC/AT board wires it.
#[derive(Debug)]
pub struct Ide {
    channel: Arc<Channel>,
    command: RegionRef,
    control: RegionRef,
}

impl Ide {
    /// Validate `props` and build the adapter.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or one this class
    /// does not know was given.
    pub fn new(props: &Props) -> Result<Ide> {
        let mut r = props.reader();
        let master = r.or_str("master", DEFAULT_MASTER_BAY)?.to_string();
        let slave = r.or_str("slave", DEFAULT_SLAVE_BAY)?.to_string();
        r.finish()?;
        if master == slave {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "`master` and `slave` are two positions on one cable and cannot both be \
                     `{master}`"
                ),
            });
        }
        // Opening a bay is allocation, not an outward action: it creates the
        // socket if the drive has not been constructed yet, which is the whole
        // point of a rendezvous that does not depend on declaration order.
        let bays = [bays::attach(props, &master)?, bays::attach(props, &slave)?];
        Ok(Ide::with_bays(bays, [master, slave]))
    }

    /// Build one around bays the caller already has.
    #[must_use]
    pub fn with_bays(bays: [Arc<Bay>; 2], names: [String; 2]) -> Ide {
        let channel = Arc::new(Channel {
            bays,
            names,
            irq_out: Mutex::with_rank(PIN_RANK, None),
        });
        let command: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            COMMAND_WINDOW_LEN,
            Arc::new(CommandBlock(Arc::clone(&channel))) as Arc<dyn MemOps>,
        ));
        let control: RegionRef = Arc::new(Region::io(
            "pc.ide.ctl",
            CONTROL_WINDOW_LEN,
            Arc::new(ControlBlock(Arc::clone(&channel))) as Arc<dyn MemOps>,
        ));
        Ide {
            channel,
            command,
            control,
        }
    }

    /// The drive in one of the two positions, if there is one.
    #[must_use]
    pub fn drive(&self, position: crate::dev::ata::Position) -> Option<Arc<AtaDisk>> {
        let index = usize::from(position == crate::dev::ata::Position::Device1);
        self.channel.bays[index].drive()
    }

    /// The bay names this channel was given, master first.
    #[must_use]
    pub fn bay_names(&self) -> [&str; 2] {
        [&self.channel.names[0], &self.channel.names[1]]
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        let drives = self.channel.drives();
        Channel::answering(&drives).is_some_and(|drive| drive.irq_asserted())
    }
}

/// The `pc.ide` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "one AT IDE channel: the command and control blocks, master/slave selection, INTRQ",
    properties: &[
        PropertySpec {
            name: "master",
            kind: ValueKind::Str,
            required: false,
            summary: "the drive bay device 0 is fitted in (default `ata0`)",
        },
        PropertySpec {
            name: "slave",
            kind: ValueKind::Str,
            required: false,
            summary: "the drive bay device 1 is fitted in (default `ata1`)",
        },
    ],
    construct: |props| Ok(Box::new(Ide::new(props)?)),
};

impl Device for Ide {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` places both regions and the realizer hands
        // the wire over, both after every device has been constructed.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The adapter has nothing to reset — every bit is on a drive, and the
        // drives are their own devices, which `Machine::reset` visits itself.
        // What is left is to re-drive the pin, because a wire keeps the level
        // it was last given and a drive that has just been reset is no longer
        // asking for an interrupt.
        //
        // `machines/pc-at.machine` declares the drives *before* the channels so
        // that this runs after they have gone quiet; a board that got the order
        // wrong would carry a stale level until its first port access, which
        // refreshes too.
        self.channel.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" | "cmd" => Some(Arc::clone(&self.command)),
            "ctl" => Some(Arc::clone(&self.control)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an IDE channel drives one pin, `irq`"),
            });
        }
        *self.channel.irq_out.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == "irq" {
            self.channel.refresh();
        }
    }
}

impl Instance for Ide {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ide::new(props)?)))
}

/// What the validator should know about `pc.ide`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("master", ValueKind::Str))
        .prop(PropSchema::new("slave", ValueKind::Str))
        .region("regs")
        .region("cmd")
        .region("ctl")
        .port("irq", PortDir::Out)
}

#[cfg(test)]
mod tests;
