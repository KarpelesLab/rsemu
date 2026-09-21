//! The A4000's IDE port: its address decode, its byte swap and its interrupt
//! register.
//!
//! One class, `amiga.ide`. The A4000 has no Gayle — it has Fat Gary, Ramsey
//! and a handful of buffers — and what stands between the processor and an AT
//! bus drive is a chip select, three address lines and an interrupt input,
//! exactly as on the A600. Where it differs from
//! [`crate::dev::amiga::gayle`] is *where*, and that difference is the whole
//! reason this is a separate file rather than a property on that one:
//!
//! | | A600 / A1200 (Gayle) | A4000 |
//! | --- | --- | --- |
//! | command block | `$DA2000` (and `$DA0000`) | `$00DD2020` |
//! | control block | `$DA3000` (and `$DA1000`) | `$00DD3020` |
//! | interrupt | Gayle's change/enable registers at `$DA9000`/`$DAA000` | one register at `$00DD3020` |
//! | also on the chip | the PCMCIA slot, the identification register, the overlay | nothing |
//!
//! So a Gayle with a moved base would still carry a credit-card slot the
//! A4000 does not have and an interrupt-enable register its Kickstart never
//! writes, and the interrupt would never be delivered. The decode itself is
//! shared, and is reproduced here rather than imported so that an A4000 build
//! links no PCMCIA model; `gayle.rs` carries the long form of the argument for
//! it.
//!
//! # Where the registers are, and how that was established
//!
//! **Black-box.** Commodore's A4000 documentation the author has access to
//! does not print an address for the port, so what is here is what the ROM
//! does. Booting `amiga-os-310-a4000.rom` on this board with the whole of
//! `$00D00000`–`$00DBFFFF`, `$00DD0000`–`$00DDFFFF` and `$00E80000`–
//! `$00EFFFFF` answered by a recorder and nothing else, the ROM finishes
//! autoconfig (34 byte reads, at every even address from `$00E80000` to
//! `$00E80042`, finding nothing) and then makes exactly three accesses, four
//! times over:
//!
//! ```text
//!   W.B  $00DD203A   ; Device/Head — select device 0, then device 1
//!   R.B  $00DD2032   ; Cylinder Low
//!   R.B  $00DD203E   ; Status
//! ```
//!
//! That is ATA-1 §9.1's drive-present probe, and it fixes the geometry
//! completely. The three offsets from `$00DD2020` are `$12`, `$1A` and `$1E` —
//! `4n + 2` for *n* = 4, 6 and 7, which are Cylinder Low, Device/Head and
//! Status in ATA-1 §7's register order. So:
//!
//! * **`A4`–`A2` are `DA2`–`DA0`**, the drive's three address lines: register
//!   *n* is four bytes on from register *n*−1, the same wiring the A600's
//!   schematic has (`gayle.rs`, and the Gayle specification §7.3).
//! * **`A12` is the chip select**: `CS1FX-` for the command block below it,
//!   `CS3FX-` for the control block above, again as on the A600.
//! * **`A1` is not decoded, and the data bus is byte-swapped.** A register's
//!   four-byte slot answers at all four of its addresses, and which half of
//!   the sixteen data lines a byte access takes is `A0`: even is `D15`–`D8`,
//!   odd is `D7`–`D0`. The eight-bit registers are on the drive's `DD7`–`DD0`
//!   and reach the processor on `D15`–`D8`, which is why the ROM reads them at
//!   the *even* addresses `$…32`, `$…3A` and `$…3E` — and why it reads the
//!   sixteen-bit data register as a word at `$00DD2020`, four bytes lower in
//!   the same slot. The A600's sheet 12 carries the same swap in capitals
//!   ("WARNING: BYTE SWAPPED") and for the same reason: a sector then lands in
//!   memory in the order a PC wrote it, which is why one HDF boots on either
//!   machine. Read back with the swap the wrong way round, the Rigid Disk
//!   Block's `RDSK` would be `DRKS` and nothing would mount.
//!
//! What the region models, then, is the two 4 KiB pages `A12` picks between,
//! mapped at `$00DD2000`. Inside each, the drive answers only where `A5` is
//! high and `A11`–`A6` are low — `$x020`–`$x03F` — with register *n*'s slot at
//! `$x020 + 4n`:
//!
//! ```text
//!   $00DD2020         data, a word:  DD7-DD0 first, then DD15-DD8
//!   $00DD202A         sector count   $00DD2032   cylinder low
//!   $00DD203A         device/head    $00DD203E   status, and command on a write
//!   $00DD3020         the interrupt register, read as a word: bit 15
//!   $00DD303A         alternate status, and device control on a write
//! ```
//!
//! The `+$20` is `A5`, part of the board's select rather than of the drive's
//! decode, which is why the `map` statement's address is the page and not the
//! task file.
//!
//! # The interrupt register, and why it is not a latch
//!
//! The drive's `INTRQ` reaches the processor as `INT2` — Paula's `PORTS`,
//! level 2 — the same level Gayle's IDE interrupt arrives on and the level
//! `scsi.device` puts its handler at. Software has to be able to tell that it
//! was *this* port that interrupted before it touches anybody's drive, and
//! what it reads to find out is `$00DD3020`: the control block's register-0
//! slot, which ATA-1 §7.2 leaves to no drive at all.
//!
//! **It is the line, not a latch in front of it**, and that was established by
//! experiment rather than assumed. Modelled as a latch that only a write
//! clears, the boot stops dead: the ROM issues `RECALIBRATE`, reads
//! `$00DD3020` as `$8000`, reads Status — which releases `INTRQ` (ATA-1 §9.5)
//! — and reads `$00DD3020` again, forever, two hundred thousand times in six
//! seconds, because it never writes there at all. Made the line itself, the
//! same ROM reads `$8000` once, reads Status, goes on to `IDENTIFY DEVICE` and
//! boots Workbench. So: **bit 15 of the word — bit 7 of the even byte — is
//! `INTRQ`**, every other bit floats, and a write does nothing.
//! `docs/platforms/amiga.md`, "A4000", has both traces.
//!
//! # `MemAttrs::debug`
//!
//! As in [`crate::dev::amiga::gayle`] and [`crate::dev::pc::ide`]: a debug
//! read of Status or Data is passed to the drive as a flag and neither
//! acknowledges nor advances anything, Alternate Status and the interrupt
//! register have no side effect to begin with, and a debug write is refused.
//!
//! No Amiga emulator source, no AROS source and no Kickstart disassembly was
//! consulted (`ROADMAP.md` §1); the listing above is rsemu's own recorder
//! printing the bus cycles a running machine made.

use alloc::boxed::Box;
use alloc::format;
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

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.ide";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The region a `map` statement places at `$00DD2000`: both chip selects.
pub const IDE_REGION: &str = "ide";

/// How much the region decodes: the two 4 KiB pages `A12` picks between.
pub const IDE_WINDOW_LEN: u64 = 0x2000;

/// Where the task file sits inside each page: `A5` high, `A11`–`A6` low.
const TASK_FILE: u64 = 0x020;

/// The page bits the board's select looks at: `A11`–`A5`.
const SELECT_MASK: u64 = 0xfe0;

/// The `INT2` output: open collector onto the net CIA-A's `/IRQ` is on.
pub const INT2_PIN: &str = "int2";

/// The bay device 0 is fitted in when a machine file does not say.
pub const DEFAULT_MASTER_BAY: &str = "ata0";

/// The bay device 1 is fitted in when a machine file does not say.
pub const DEFAULT_SLAVE_BAY: &str = "ata1";

/// Bit 7 of the interrupt register — bit 15 of the word the ROM reads there:
/// the drive's `INTRQ`.
pub const INT_PENDING: u8 = 0x80;

/// Which chip select an offset in the window asserts: `A12`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Select {
    /// The drive's `CS1FX-`: the command block, `$00DD2020`-`$00DD203F`.
    Command,
    /// The drive's `CS3FX-`: the control block, `$00DD3020`-`$00DD303F`.
    Control,
}

/// The chip select and the three drive address lines (`A4`–`A2`) an offset in
/// the window decodes to, or `None` where the board selects nothing.
#[must_use]
#[inline]
pub const fn ide_decode(offset: u64) -> Option<(Select, u8)> {
    let select = if offset & 0x1000 == 0 {
        Select::Command
    } else {
        Select::Control
    };
    let page = offset & 0xfff;
    if page & SELECT_MASK != TASK_FILE {
        return None;
    }
    Some((select, ((page >> 2) & 7) as u8))
}

/// What `DA2`–`DA0` select in the command block (ATA-1 §7, register order).
#[must_use]
pub const fn command_register(da: u8) -> Reg {
    match da & 7 {
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

/// `DA2`–`DA0` of the control block's Device Control / Alternate Status.
pub const CONTROL_REGISTER: u8 = 6;

/// `DA2`–`DA0` of the board's own interrupt register: the control block's
/// register-0 slot, which no drive drives.
pub const INT_REGISTER: u8 = 0;

// ---------------------------------------------------------------------------
// the port
// ---------------------------------------------------------------------------

/// The shared half of the device: the cable and the pin.
struct Port {
    bays: [Arc<Bay>; 2],
    names: [String; 2],
    int2: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Port")
            .field("master", &self.names[0])
            .field("slave", &self.names[1])
            .finish_non_exhaustive()
    }
}

impl Port {
    /// Both drives, looked up with the bay locks released.
    fn drives(&self) -> [Option<Arc<AtaDisk>>; 2] {
        [self.bays[0].drive(), self.bays[1].drive()]
    }

    /// The drive that answers a read, if any.
    fn answering(drives: &[Option<Arc<AtaDisk>>; 2]) -> Option<&Arc<AtaDisk>> {
        drives.iter().flatten().find(|drive| drive.is_selected())
    }

    /// The drive's `INTRQ`, as the cable carries it.
    fn intrq(&self) -> bool {
        let drives = self.drives();
        Port::answering(&drives).is_some_and(|drive| drive.irq_asserted())
    }

    /// Take a new view of `INTRQ` and drive the pin from it.
    ///
    /// The drive is asked with nothing of this device's held, and the pin is
    /// driven with nothing held at all — mutate, release, then call outward.
    fn refresh(&self) {
        let level = self.intrq();
        let source = self.int2.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(level));
        }
    }

    // -- the cable -----------------------------------------------------------

    fn read_command(&self, reg: Reg, debug: bool) -> Option<u16> {
        let drives = self.drives();
        match Port::answering(&drives) {
            Some(drive) => Some(drive.read_reg(reg, debug)),
            // The selected position is empty but the other is not: the drive
            // that is there answers for it with zeroes (ATA-1 §5.2.2).
            None if drives.iter().any(Option::is_some) => Some(0),
            // An empty cable: nothing drives the bus at all.
            None => None,
        }
    }

    fn write_command(&self, reg: Reg, value: u16) {
        for drive in self.drives().iter().flatten() {
            drive.write_reg(reg, value);
        }
    }

    fn read_alt_status(&self) -> Option<u8> {
        let drives = self.drives();
        match Port::answering(&drives) {
            Some(drive) => Some(drive.read_alt_status()),
            None if drives.iter().any(Option::is_some) => Some(0),
            None => None,
        }
    }

    fn write_control(&self, value: u8) {
        for drive in self.drives().iter().flatten() {
            drive.write_device_control(value);
        }
    }
}

/// `$00DD2000`–`$00DD3FFF`: the drive, and the board's interrupt register.
#[derive(Debug)]
struct IdeWindow(Arc<Port>);

impl MemOps for IdeWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let port = &self.0;
        let float = attrs.bus;
        // The two halves of the sixteen data lines, as the processor sees
        // them: `D15`–`D8` on an even byte, `D7`–`D0` on an odd one. The port
        // is byte-swapped, so `D15`–`D8` is the drive's `DD7`–`DD0`.
        let (even, odd) = match ide_decode(offset) {
            None => (float, float),
            Some((Select::Command, da)) if command_register(da) == Reg::Data => {
                // One -IOR, one word out of the sector buffer.
                let word = port.read_command(Reg::Data, attrs.debug);
                word.map_or((float, float), |w| (w as u8, (w >> 8) as u8))
            }
            Some((Select::Command, da)) => {
                let byte = port.read_command(command_register(da), attrs.debug);
                (byte.map_or(float, |b| b as u8), float)
            }
            Some((Select::Control, CONTROL_REGISTER)) => {
                (port.read_alt_status().unwrap_or(float), float)
            }
            Some((Select::Control, INT_REGISTER)) => {
                // The board's own, on the same lane the drive's eight-bit
                // registers are on: the ROM reads a *word* here and looks at
                // bit 15, which is bit 7 of the even byte.
                let set = port.intrq();
                (
                    (if set { INT_PENDING } else { 0 }) | (float & !INT_PENDING),
                    float,
                )
            }
            // ATA-1 §7.2 leaves the rest of the control block to the drive's
            // own Drive Address register, which this drive has none of.
            Some((Select::Control, _)) => (float, float),
        };
        match (dst.len(), offset & 1) {
            (2, 0) => {
                dst[0] = even;
                dst[1] = odd;
            }
            (1, 0) => dst[0] = even,
            (1, _) => dst[0] = odd,
            _ => return Err(BusError::BadAccess),
        }
        if !attrs.debug {
            port.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A write to Command starts a command, one to Data fills a sector
            // buffer and one to Device Control can reset the drives. None of
            // them can be made harmless (`ROADMAP.md` 15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let port = &self.0;
        // What is on `D15..D8` and `D7..D0`. A byte write drives the same byte
        // on both halves of the bus.
        let (high, low) = match (src.len(), offset & 1) {
            (2, 0) => (src[0], src[1]),
            (1, _) => (src[0], src[0]),
            _ => return Err(BusError::BadAccess),
        };
        // Byte-swapped: `D15..D8` is `DD7..DD0`.
        let word = u16::from(high) | u16::from(low) << 8;
        match ide_decode(offset) {
            Some((Select::Command, da)) => port.write_command(command_register(da), word),
            Some((Select::Control, CONTROL_REGISTER)) => port.write_control(high),
            // The interrupt register is the drive's line; nothing a write can
            // change, and the ROM never writes one.
            Some((Select::Control, _)) | None => {}
        }
        port.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A byte or a word, either address, big-endian: the drive's sixteen
        // data lines sit on the low half of the 32-bit bus, so a longword
        // access is two cycles on two addresses, as on the board.
        let mut c = AccessConstraints::word(Width::U16, Endian::Big);
        c.min = Width::U8;
        c.natural_alignment = false;
        c
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The A4000's IDE port.
#[derive(Debug)]
pub struct AmigaIde {
    port: Arc<Port>,
    ide: RegionRef,
}

impl AmigaIde {
    /// Validate `props` and build the port.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown;
    /// [`Error::Config`] if `master` and `slave` name one bay.
    pub fn new(props: &Props) -> Result<AmigaIde> {
        let mut r = props.reader();
        let master = r.or_str("master", DEFAULT_MASTER_BAY)?.to_string();
        let slave = r.or_str("slave", DEFAULT_SLAVE_BAY)?.to_string();
        r.finish()?;
        if master == slave {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "`master` and `slave` are two positions on one cable and cannot both be \
                     `{master}`"
                ),
            });
        }
        // Opening a bay is allocation, not an outward action (`pc.ide`).
        let bays = [bays::attach(props, &master)?, bays::attach(props, &slave)?];
        Ok(AmigaIde::with_bays(bays, [master, slave]))
    }

    /// Build one around bays the caller already has.
    #[must_use]
    pub fn with_bays(bays: [Arc<Bay>; 2], names: [String; 2]) -> AmigaIde {
        let port = Arc::new(Port {
            bays,
            names,
            int2: Mutex::with_rank(LockRank::LEAF, None),
        });
        AmigaIde {
            ide: Arc::new(Region::io(
                format!("{CLASS_NAME}.{IDE_REGION}"),
                IDE_WINDOW_LEN,
                Arc::new(IdeWindow(Arc::clone(&port))),
            )),
            port,
        }
    }

    /// The drive in one of the two positions, if there is one.
    #[must_use]
    pub fn drive(&self, position: crate::dev::ata::Position) -> Option<Arc<AtaDisk>> {
        let index = usize::from(position == crate::dev::ata::Position::Device1);
        self.port.bays[index].drive()
    }

    /// Whether the port is pulling `INT2` — the drive's `INTRQ`, which is all
    /// the interrupt register is.
    #[must_use]
    pub fn interrupt(&self) -> bool {
        self.port.intrq()
    }
}

/// The `amiga.ide` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the A4000's IDE port: the chip selects and byte swap in front of an `ata.disk`, \
              and the interrupt register at $00DD3020",
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
    construct: |props| Ok(Box::new(AmigaIde::new(props)?)),
};

impl Device for AmigaIde {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the `map` statement places the region and the wire
        // graph brings the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The drives are their own devices, declared first, so the line
        // observed here is the one they now drive.
        self.port.refresh();
    }

    // No `save`/`load`: this port holds no state of its own. Everything a
    // snapshot has to carry — the task file, the sector buffer, `INTRQ` — is
    // the `ata.disk`'s, and the interrupt register is that drive's line seen
    // through a buffer. A pin level restored from here would be derived state
    // serialized twice (`CLAUDE.md`, devices).

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | IDE_REGION => Some(Arc::clone(&self.ide)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != INT2_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("the A4000's IDE port drives one pin: `int2`"),
            });
        }
        *self.port.int2.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == INT2_PIN {
            self.port.refresh();
        }
    }
}

/// Nothing to bind: the region is placed by a `map` statement and the pin by
/// the wire graph.
impl Instance for AmigaIde {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(AmigaIde::new(props)?)))
}

/// What the validator should know about `amiga.ide`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("master", ValueKind::Str))
        .prop(PropSchema::new("slave", ValueKind::Str))
        .region("")
        .region(IDE_REGION)
        .port(INT2_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
