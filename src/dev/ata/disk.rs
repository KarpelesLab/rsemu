//! An ATA hard disk: the command block, the command set and the media.
//!
//! **This models the drive, not a host adapter.** Nothing here knows what
//! `0x1f0` is, and nothing here has a register *offset* — [`Reg`] is an
//! enumeration of the eight names ATA gives the command block, and an adapter
//! is what turns a chip select and three address lines into one of them.
//!
//! ```text
//!   adapter ──► write_reg(Reg, u16) ─────────────────► every drive on the cable
//!           ◄── read_reg(Reg, debug) ──u16────────────  the *selected* drive
//!
//!           ──► write_device_control(u8) ────────────► every drive (nIEN, SRST)
//!           ◄── read_alt_status() ──u8────────────────  no side effect, ever
//!           ◄── irq_asserted() ──bool────────────────   INTRQ
//! ```
//!
//! Five calls. Everything above that line — which port answers, whether the
//! data register is sixteen or thirty-two bits wide on this bus, which 8259A
//! input the interrupt lands on, what an absent drive leaves on the bus — is
//! the adapter's business, because on real silicon it is the adapter's business
//! too.
//!
//! # Why the split is drawn exactly here
//!
//! "IDE" stands for *integrated drive electronics*, and it is a description of
//! where the parts are: the controller that was a card in a PC/XT moved onto
//! the drive, and what stayed on the motherboard is a buffer and a decoder.
//! ATA is the cable between them. So the honest seam is the cable, and the
//! cable carries a register selection, sixteen data lines, a reset and an
//! interrupt — which is exactly the list above.
//!
//! Two consequences that show the line is in the right place:
//!
//! * **Device selection is the drive's business, not the adapter's.** A write
//!   to the Device register goes to *both* drives; each compares the DEV bit
//!   with the position it is jumpered to and decides whether it is being spoken
//!   to. [`AtaDisk::is_selected`] is how the adapter finds out who answers, and
//!   it never has to know which bit that was.
//! * The same drive would hang off a CompactFlash socket, a PCMCIA adapter or a
//!   PCI IDE controller with no change, because none of those change the cable.
//!
//! # ATAPI is out of scope
//!
//! There is no packet interface here. `IDENTIFY PACKET DEVICE` is aborted,
//! which is the specified behaviour of a device that is not a packet device and
//! is how a driver distinguishes the two. A half-built CD-ROM that answered
//! `IDENTIFY PACKET DEVICE` and then failed on the first `PACKET` would be
//! worse than an honest refusal.
//!
//! # Time
//!
//! **Deliberately zero**, the same choice [`crate::dev::pc::fdc`] and
//! [`crate::dev::flash::cfi`] make, and the reasoning has to be spelled out
//! because this is the device where it is most likely to matter.
//!
//! A command completes inside the `write_reg` that delivers its opcode. `BSY`
//! is therefore never observed set by anything except a driver that is holding
//! `SRST` asserted — which is a state the *host* put the drive in, so it is
//! observable and is modelled. The consequences are precise:
//!
//! * `while (status & BSY) ;` terminates on its first read. Correct.
//! * `while (!(status & DRQ)) ;` terminates on its first read, because the
//!   sector was fetched before the `OUT` that asked for it returned. Correct.
//! * A driver that waits for `INTRQ` gets it — asserted, again, before that
//!   `OUT` returned. Because `INTRQ` is a level that stays up until the Status
//!   register is read, an edge-triggered ISA interrupt input still sees an
//!   edge. Correct.
//! * A driver that *times* a seek measures zero. Nothing in a PC's boot path
//!   does that.
//!
//! The order in which `BSY` and `DRQ` change is still modelled exactly, because
//! that order is what firmware polls on and getting it wrong is what makes a
//! model work with one driver and hang another. In particular the two PIO
//! protocols differ in a way that is easy to get wrong and is asserted in the
//! tests: a **read** asserts `INTRQ` at the *start* of every block including the
//! first, and a **write** asserts it at the *end* of every block, so the first
//! block of a write is announced by `DRQ` alone and a driver that waits for an
//! interrupt before writing it deadlocks on real hardware too.
//!
//! Making the durations real is the other correct choice: this device would
//! take a clock domain and post its completion as a scheduler event
//! (`ROADMAP.md` §4.2). Nothing above [`AtaDisk`]'s five methods would change,
//! because a host adapter already asks rather than assumes.
//!
//! # The backing store
//!
//! A [`RamStore`] filled from a media slot, not an `fstool::BlockDevice`. The
//! same argument [`crate::dev::sd::card`] and `dev-flash-cfi` make: the contents
//! are a flat image, byte addressed, and reaching for a disk-image crate would
//! drag `std` into a `no_std` device. `docs/buses/storage.md` names the variant
//! that does not make that trade — a `dev/blk/ata` under the documented `std`
//! exception, for a large or sparse image — and it would reuse this whole
//! protocol half and replace only [`AtaDisk::read_media`] and
//! [`AtaDisk::write_media`].
//!
//! # Sources
//!
//! * **T13, *AT Attachment with Packet Interface - 6* (ATA/ATAPI-6,
//!   T13/1410D)** — the register file and its two-deep 48-bit FIFOs, the status
//!   and error bit assignments, the PIO data-in and data-out protocols, the
//!   command descriptions for `IDENTIFY DEVICE`, `READ SECTOR(S)`,
//!   `WRITE SECTOR(S)`, `READ MULTIPLE`, `WRITE MULTIPLE`, `SET MULTIPLE MODE`,
//!   `INITIALIZE DEVICE PARAMETERS`, `READ VERIFY SECTOR(S)`, `SEEK`,
//!   `EXECUTE DEVICE DIAGNOSTIC`, `FLUSH CACHE`, `SET FEATURES` and
//!   `READ NATIVE MAX ADDRESS`, the 48-bit Address feature set, and the
//!   IDENTIFY DEVICE data table.
//! * *IBM Personal Computer AT Technical Reference* (1984) and Ralf Brown's
//!   Interrupt List for the `INT 13h` side of the CHS translation.
//!
//! **No emulator source was consulted and no operating system's ATA driver was
//! opened** (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "ata.disk";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes an ATA logical sector holds.
///
/// 512 everywhere here. Long logical sectors exist in ATA-7 and later and no
/// PC firmware of the era can address one.
pub const SECTOR: u64 = 512;

/// The largest LBA the 28-bit addressing scheme can name, plus one.
pub const LBA28_LIMIT: u64 = 1 << 28;

/// The largest LBA the 48-bit addressing scheme can name, plus one.
pub const LBA48_LIMIT: u64 = 1 << 48;

/// The largest cylinder count words 1 and 54 of `IDENTIFY DEVICE` can express.
pub const MAX_IDENTIFY_CYLINDERS: u64 = 16383;

// ---------------------------------------------------------------------------
// The Status register
// ---------------------------------------------------------------------------

/// Busy: the device owns the command block and every other bit is meaningless.
pub const ST_BSY: u8 = 0x80;
/// Device ready.
pub const ST_DRDY: u8 = 0x40;
/// Device fault.
pub const ST_DF: u8 = 0x20;
/// Device seek complete. Obsolete, and every driver still looks at it.
pub const ST_DSC: u8 = 0x10;
/// Data request: the data register is ready to be emptied or filled.
pub const ST_DRQ: u8 = 0x08;
/// An error occurred; the Error register says which.
pub const ST_ERR: u8 = 0x01;

/// The resting status of a drive with nothing to do.
const ST_IDLE: u8 = ST_DRDY | ST_DSC;

// ---------------------------------------------------------------------------
// The Error register
// ---------------------------------------------------------------------------

/// Uncorrectable data error.
pub const ERR_UNC: u8 = 0x40;
/// The requested sector's ID field was not found — an out-of-range address.
pub const ERR_IDNF: u8 = 0x10;
/// Command aborted.
pub const ERR_ABRT: u8 = 0x04;
/// Track 0 was not found.
pub const ERR_TK0NF: u8 = 0x02;

/// What the Error register holds after a passing power-on diagnostic.
const DIAGNOSTIC_PASSED: u8 = 0x01;

// ---------------------------------------------------------------------------
// The Device register
// ---------------------------------------------------------------------------

/// Which of the two drives on the cable is being addressed.
pub const DEV_SELECT: u8 = 0x10;
/// Addressing is LBA rather than CHS.
pub const DEV_LBA: u8 = 0x40;
/// The head number, or LBA bits 27:24.
pub const DEV_HEAD: u8 = 0x0f;
/// Bits 7 and 5 are obsolete and read back as ones.
const DEV_OBSOLETE: u8 = 0xa0;

// ---------------------------------------------------------------------------
// The Device Control register
// ---------------------------------------------------------------------------

/// High order byte: read back the previous content of the 48-bit FIFOs.
pub const CTL_HOB: u8 = 0x80;
/// Software reset.
pub const CTL_SRST: u8 = 0x04;
/// Interrupt disable: the drive shall not assert `INTRQ`.
pub const CTL_NIEN: u8 = 0x02;

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// The command opcodes this drive answers.
///
/// Named rather than inlined because the dispatch below reads as prose with
/// them and as a soup of magic numbers without.
pub mod cmd {
    /// `NOP`. Specified to be aborted, which is not the same as ignored.
    pub const NOP: u8 = 0x00;
    /// `RECALIBRATE`, 0x10-0x1f. Obsolete since ATA-4; firmware still sends it.
    pub const RECALIBRATE: u8 = 0x10;
    /// `READ SECTOR(S)`, with retries.
    pub const READ_SECTORS: u8 = 0x20;
    /// `READ SECTOR(S)`, without retries. The same command here.
    pub const READ_SECTORS_NORETRY: u8 = 0x21;
    /// `READ SECTOR(S) EXT`: 48-bit addressing.
    pub const READ_SECTORS_EXT: u8 = 0x24;
    /// `READ NATIVE MAX ADDRESS EXT`.
    pub const READ_NATIVE_MAX_EXT: u8 = 0x27;
    /// `READ MULTIPLE EXT`.
    pub const READ_MULTIPLE_EXT: u8 = 0x29;
    /// `WRITE SECTOR(S)`.
    pub const WRITE_SECTORS: u8 = 0x30;
    /// `WRITE SECTOR(S)`, without retries.
    pub const WRITE_SECTORS_NORETRY: u8 = 0x31;
    /// `WRITE SECTOR(S) EXT`.
    pub const WRITE_SECTORS_EXT: u8 = 0x34;
    /// `WRITE MULTIPLE EXT`.
    pub const WRITE_MULTIPLE_EXT: u8 = 0x39;
    /// `READ VERIFY SECTOR(S)`.
    pub const VERIFY_SECTORS: u8 = 0x40;
    /// `READ VERIFY SECTOR(S)`, without retries.
    pub const VERIFY_SECTORS_NORETRY: u8 = 0x41;
    /// `READ VERIFY SECTOR(S) EXT`.
    pub const VERIFY_SECTORS_EXT: u8 = 0x42;
    /// `SEEK`, 0x70-0x7f.
    pub const SEEK: u8 = 0x70;
    /// `EXECUTE DEVICE DIAGNOSTIC`.
    pub const DIAGNOSTIC: u8 = 0x90;
    /// `INITIALIZE DEVICE PARAMETERS`: the host declares the CHS translation.
    pub const INIT_DEVICE_PARAMS: u8 = 0x91;
    /// `IDENTIFY PACKET DEVICE`. Aborted: this is not a packet device.
    pub const IDENTIFY_PACKET: u8 = 0xa1;
    /// `READ MULTIPLE`.
    pub const READ_MULTIPLE: u8 = 0xc4;
    /// `WRITE MULTIPLE`.
    pub const WRITE_MULTIPLE: u8 = 0xc5;
    /// `SET MULTIPLE MODE`.
    pub const SET_MULTIPLE: u8 = 0xc6;
    /// `STANDBY IMMEDIATE`.
    pub const STANDBY_IMMEDIATE: u8 = 0xe0;
    /// `IDLE IMMEDIATE`.
    pub const IDLE_IMMEDIATE: u8 = 0xe1;
    /// `STANDBY`.
    pub const STANDBY: u8 = 0xe2;
    /// `IDLE`.
    pub const IDLE: u8 = 0xe3;
    /// `CHECK POWER MODE`.
    pub const CHECK_POWER_MODE: u8 = 0xe5;
    /// `SLEEP`.
    pub const SLEEP: u8 = 0xe6;
    /// `FLUSH CACHE`.
    pub const FLUSH_CACHE: u8 = 0xe7;
    /// `FLUSH CACHE EXT`.
    pub const FLUSH_CACHE_EXT: u8 = 0xea;
    /// `IDENTIFY DEVICE`.
    pub const IDENTIFY: u8 = 0xec;
    /// `SET FEATURES`.
    pub const SET_FEATURES: u8 = 0xef;
    /// `READ NATIVE MAX ADDRESS`.
    pub const READ_NATIVE_MAX: u8 = 0xf8;
}

// ---------------------------------------------------------------------------
// The seam
// ---------------------------------------------------------------------------

/// Which position on the cable a drive is jumpered to.
///
/// The names ATA uses. "Master" and "slave" are the names the connectors and
/// every BIOS setup screen use, and a machine description accepts both.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Position {
    /// Device 0, the master.
    #[default]
    Device0,
    /// Device 1, the slave.
    Device1,
}

impl Position {
    /// The value of the Device register's DEV bit that selects this position.
    #[must_use]
    fn dev_bit(self) -> bool {
        self == Position::Device1
    }

    /// The name a machine description writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Position::Device0 => "master",
            Position::Device1 => "slave",
        }
    }
}

/// One of the eight command block registers, by name.
///
/// **There is no number here, and that is the whole point of this type.** Two
/// of the eight have different meanings in each direction, which is a property
/// of the register and not of the adapter, so each carries both names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reg {
    /// The 16-bit data port: the sector buffer, one word at a time.
    Data,
    /// Features on a write, Error on a read.
    Feature,
    /// Sector Count: how many sectors a transfer moves.
    SectorCount,
    /// LBA bits 7:0, or the CHS sector number (which is one-based).
    LbaLow,
    /// LBA bits 15:8, or the low byte of the CHS cylinder.
    LbaMid,
    /// LBA bits 23:16, or the high byte of the CHS cylinder.
    LbaHigh,
    /// Drive select, addressing mode, and the head number or LBA bits 27:24.
    Device,
    /// Command on a write, Status on a read.
    Command,
}

// ---------------------------------------------------------------------------
// Geometry and addressing
// ---------------------------------------------------------------------------

/// A cylinders/heads/sectors translation.
///
/// Two of these exist at once and confusing them is the classic failure this
/// device exists to get right: the **default** translation, which
/// `IDENTIFY DEVICE` reports in words 1, 3 and 6 and which is what a BIOS reads
/// before it has said anything, and the **current** translation, which
/// `INITIALIZE DEVICE PARAMETERS` sets, which words 54, 55 and 56 report, and
/// which is the one every CHS command is decoded against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    /// Cylinders.
    pub cylinders: u16,
    /// Heads per cylinder. One to sixteen: the Device register has four bits.
    pub heads: u8,
    /// Sectors per track. Numbered from **one**, not zero.
    pub sectors: u8,
}

impl Geometry {
    /// How many sectors this translation can name.
    #[must_use]
    pub fn addressable(&self) -> u64 {
        u64::from(self.cylinders) * u64::from(self.heads) * u64::from(self.sectors)
    }

    /// Whether it names anything at all.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.cylinders > 0 && self.heads > 0 && self.sectors > 0
    }
}

/// An address as the command block carries it.
///
/// The three forms exist because a BIOS speaks the first, an operating system
/// speaks the second, and a large disk needs the third — and a model that
/// translated between them wrongly would read one sector down one path and a
/// different sector down the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    /// Cylinder, head and one-based sector.
    Chs {
        /// Cylinder.
        cylinder: u16,
        /// Head.
        head: u8,
        /// Sector, counted from one.
        sector: u8,
    },
    /// A 28-bit logical block address.
    Lba28(u32),
    /// A 48-bit logical block address.
    Lba48(u64),
}

impl Address {
    /// The logical block this names, under `geometry`.
    ///
    /// `None` when a CHS triple is not one `geometry` can express — a sector of
    /// zero, or a head or sector beyond the current translation. Real hardware
    /// answers `IDNF` to those, and so does this drive.
    ///
    /// The formula is the only one there is, and writing it once here rather
    /// than at each of its four call sites is why this type exists:
    ///
    /// ```text
    ///   LBA = (cylinder x heads + head) x sectors_per_track + (sector - 1)
    /// ```
    #[must_use]
    pub fn to_lba(self, geometry: &Geometry) -> Option<u64> {
        match self {
            Address::Chs {
                cylinder,
                head,
                sector,
            } => {
                if !geometry.is_valid()
                    || sector == 0
                    || sector > geometry.sectors
                    || head >= geometry.heads
                {
                    return None;
                }
                let heads = u64::from(geometry.heads);
                let spt = u64::from(geometry.sectors);
                Some((u64::from(cylinder) * heads + u64::from(head)) * spt + u64::from(sector) - 1)
            }
            Address::Lba28(lba) => Some(u64::from(lba)),
            Address::Lba48(lba) => Some(lba),
        }
    }

    /// The CHS triple naming `lba` under `geometry`, if it has one.
    #[must_use]
    pub fn from_lba(lba: u64, geometry: &Geometry) -> Option<Address> {
        if !geometry.is_valid() {
            return None;
        }
        let heads = u64::from(geometry.heads);
        let spt = u64::from(geometry.sectors);
        let sector = lba % spt + 1;
        let track = lba / spt;
        let head = track % heads;
        let cylinder = track / heads;
        if cylinder > u64::from(u16::MAX) {
            return None;
        }
        Some(Address::Chs {
            cylinder: cylinder as u16,
            head: head as u8,
            sector: sector as u8,
        })
    }
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

/// Everything about a drive that does not change while it is running.
#[derive(Debug, Clone)]
pub struct Identity {
    /// How many 512-byte sectors it holds.
    pub sectors: u64,
    /// The default CHS translation, reported in words 1, 3 and 6.
    pub geometry: Geometry,
    /// The model string, `IDENTIFY` words 27-46. Forty characters.
    pub model: String,
    /// The serial number, words 10-19. Twenty characters.
    pub serial: String,
    /// The firmware revision, words 23-26. Eight characters.
    pub firmware: String,
    /// Whether the medium refuses writes.
    pub read_only: bool,
    /// Whether the 48-bit Address feature set is supported.
    pub lba48: bool,
    /// The largest block `SET MULTIPLE MODE` will accept, in sectors.
    pub max_multiple: u8,
}

impl Identity {
    /// Assemble an identity for a drive of `sectors` sectors.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the capacity is zero, if it cannot be addressed at
    /// all with the addressing modes enabled, or if `max_multiple` is not a
    /// power of two.
    pub fn new(
        sectors: u64,
        geometry: Geometry,
        lba48: bool,
        max_multiple: u8,
    ) -> Result<Identity> {
        if sectors == 0 {
            return Err(config(String::from("a drive holds at least one sector")));
        }
        if !lba48 && sectors > LBA28_LIMIT {
            return Err(config(format!(
                "{sectors} sector(s) needs 48-bit addressing, which this drive has turned off"
            )));
        }
        if sectors > LBA48_LIMIT {
            return Err(config(format!(
                "{sectors} sector(s) is more than 48-bit addressing can name"
            )));
        }
        if !geometry.is_valid() || geometry.heads > 16 {
            return Err(config(format!(
                "{}/{}/{} is not a translation the Device register can express",
                geometry.cylinders, geometry.heads, geometry.sectors
            )));
        }
        if max_multiple == 0 || !max_multiple.is_power_of_two() {
            return Err(config(format!(
                "`multiple` is a power of two block size in sectors, and {max_multiple} is not one"
            )));
        }
        Ok(Identity {
            sectors,
            geometry,
            model: String::from("RSEMU HARDDISK"),
            serial: String::from("RSEMU00000000000001"),
            firmware: String::from("1.0"),
            read_only: false,
            lba48,
            max_multiple,
        })
    }

    /// How many bytes the drive holds.
    #[must_use]
    pub fn capacity(&self) -> u64 {
        self.sectors * SECTOR
    }
}

/// The CHS translation a drive of `sectors` sectors comes out of the factory
/// with.
///
/// Sixteen heads and sixty-three sectors per track is what every PC BIOS of the
/// era assumes and what every drive of the era therefore reported; the cylinder
/// count follows and is capped at 16383, because `IDENTIFY DEVICE`'s word 1
/// cannot express more and a drive that claimed to would be lying to firmware
/// that believed it.
///
/// A drive too small for 16x63 gets the largest translation that fits, and one
/// too large simply has sectors that CHS cannot reach — which is a property of
/// CHS, not a defect here, and is the reason LBA exists.
#[must_use]
pub fn default_geometry(sectors: u64) -> Geometry {
    let mut heads: u64 = 16;
    let mut spt: u64 = 63;
    while heads > 1 && sectors < heads * spt {
        heads /= 2;
    }
    while spt > 1 && sectors < heads * spt {
        spt -= 1;
    }
    let cylinders = (sectors / (heads * spt)).clamp(1, MAX_IDENTIFY_CYLINDERS);
    Geometry {
        cylinders: cylinders as u16,
        heads: heads as u8,
        sectors: spt as u8,
    }
}

// ---------------------------------------------------------------------------
// The 48-bit register FIFOs
// ---------------------------------------------------------------------------

/// One command block register that is two bytes deep.
///
/// The 48-bit Address feature set does not add registers; it makes six of the
/// existing ones a two-entry FIFO. Writing pushes, and the Device Control
/// register's HOB bit chooses which entry a read sees. A 28-bit command simply
/// never looks at `previous`, which is why the same struct serves both.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Fifo {
    current: u8,
    previous: u8,
}

impl Fifo {
    fn write(&mut self, value: u8) {
        self.previous = self.current;
        self.current = value;
    }

    fn read(&self, hob: bool) -> u8 {
        if hob { self.previous } else { self.current }
    }

    fn load(&mut self, current: u8, previous: u8) {
        self.current = current;
        self.previous = previous;
    }
}

// ---------------------------------------------------------------------------
// A transfer in progress
// ---------------------------------------------------------------------------

/// How completion is to be reported in the command block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Cylinder/head/sector, against the current translation.
    Chs,
    /// 28-bit LBA.
    Lba28,
    /// 48-bit LBA.
    Lba48,
}

/// A multi-sector PIO transfer, part way through.
///
/// **This is state**, which is why it is in the snapshot: a guest that took a
/// save state between the 200th and 201st word of a sector and restored it must
/// carry on at the 201st.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Transfer {
    /// Host to device. A read is `false`.
    out: bool,
    /// The next sector to touch on the medium.
    next: u64,
    /// Sectors still to move, not counting any already in the buffer.
    left: u64,
    /// Sectors per DRQ block: one, or the `SET MULTIPLE MODE` count.
    block: u32,
    /// How the completion address is written back.
    mode: Mode,
    /// The last sector actually moved, for that write-back.
    last: u64,
}

// ---------------------------------------------------------------------------
// The drive
// ---------------------------------------------------------------------------

/// Everything that changes.
#[derive(Debug)]
struct Volatile {
    /// Whether this drive is the one the Device register's DEV bit names.
    selected: bool,
    /// The Device register, as last written by anyone.
    device: u8,
    error: u8,
    status: u8,
    /// The Device Control register: HOB, SRST and nIEN.
    control: u8,
    features: Fifo,
    count: Fifo,
    lba_low: Fifo,
    lba_mid: Fifo,
    lba_high: Fifo,
    /// `INTRQ`, before nIEN gates it.
    irq: bool,
    /// The host is holding `SRST` asserted.
    in_reset: bool,
    /// The `SET MULTIPLE MODE` block size; zero means the mode is off.
    multiple: u8,
    /// The current CHS translation, which `INITIALIZE DEVICE PARAMETERS` moves.
    current: Geometry,
    /// The sector buffer.
    buf: Vec<u8>,
    /// How far through it the host has got.
    pos: usize,
    xfer: Option<Transfer>,
}

impl Volatile {
    fn power_on(position: Position, geometry: Geometry) -> Volatile {
        let mut state = Volatile {
            // Device 0 is selected out of a reset, because the Device register
            // is zero and its DEV bit with it.
            selected: position == Position::Device0,
            device: 0,
            error: DIAGNOSTIC_PASSED,
            status: ST_IDLE,
            control: 0,
            features: Fifo::default(),
            count: Fifo::default(),
            lba_low: Fifo::default(),
            lba_mid: Fifo::default(),
            lba_high: Fifo::default(),
            irq: false,
            in_reset: false,
            multiple: 0,
            current: geometry,
            buf: Vec::new(),
            pos: 0,
            xfer: None,
        };
        // A power-on leaves the same signature a reset does, and it has to:
        // the *first* thing a driver reads is those five registers, and a drive
        // that answered zeroes to a cold probe would be indistinguishable from
        // an empty cable position.
        state.signature();
        state
    }

    fn hob(&self) -> bool {
        self.control & CTL_HOB != 0
    }

    /// The signature a reset leaves in the command block: an ATA device answers
    /// 0x00 / 0x00 / 0x00 / 0x01 / 0x01, which is how a driver tells it from a
    /// packet device (whose signature is 0xeb14 in the two cylinder bytes).
    fn signature(&mut self) {
        self.error = DIAGNOSTIC_PASSED;
        self.count.load(1, 0);
        self.lba_low.load(1, 0);
        self.lba_mid.load(0, 0);
        self.lba_high.load(0, 0);
        self.device &= DEV_SELECT;
        self.status = ST_IDLE;
        self.buf.clear();
        self.pos = 0;
        self.xfer = None;
    }
}

/// An ATA hard disk.
///
/// Construct it with [`AtaDisk::new`] from machine-description properties, or
/// with [`AtaDisk::with_identity`] directly.
pub struct AtaDisk {
    id: Identity,
    position: Position,
    media: Arc<RamStore>,
    state: Mutex<Volatile>,
}

impl fmt::Debug for AtaDisk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AtaDisk")
            .field("sectors", &self.id.sectors)
            .field("geometry", &self.id.geometry)
            .field("position", &self.position)
            .field("read_only", &self.id.read_only)
            .finish_non_exhaustive()
    }
}

impl AtaDisk {
    /// Validate `props` and allocate the drive, or report an empty bay.
    ///
    /// `None` is a cable position with nothing plugged into it, which is what a
    /// machine file describes by giving neither a `size` nor an `image`. It is
    /// not an error: a PC with no hard disk is an ordinary PC, and the
    /// alternative would be a machine description that needed an `if`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind;
    /// [`Error::Config`] if the geometry or the capacity is not one a drive can
    /// have, or the bound image does not fit.
    pub fn new(props: &Props) -> Result<Option<AtaDisk>> {
        let mut r = props.reader();
        let size = r.or_size("size", 0)?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        let read_only = r.or("readonly", false)?;
        let lba48 = r.or("lba48", true)?;
        let max_multiple = r.or_range("multiple", 16u64, 1..=128)? as u8;
        let position = match r.or_enum("position", "master", &["master", "slave"])? {
            "slave" => Position::Device1,
            _ => Position::Device0,
        };
        let cylinders = r.optional::<u64>("cylinders")?;
        let heads = r.optional::<u64>("heads")?;
        let sectors = r.optional::<u64>("sectors")?;
        let model = r.or_str("model", "RSEMU HARDDISK")?.to_string();
        let serial = r.or_str("serial", "RSEMU0000000000000001")?.to_string();
        let firmware = r.or_str("firmware", "1.0")?.to_string();
        // Read by `DiskDevice`, which owns the rendezvous; touched here so the
        // reader does not report it as unknown.
        let _ = r.optional_str("bay")?;
        r.finish()?;

        // A capacity from `size`, or from the image if there is one and no
        // `size`. Zero either way is an empty bay — including an image slot
        // bound to no bytes, which is how a front end says "there is no disk"
        // without the machine description needing an `if`.
        let bytes = match (size, image.as_ref()) {
            (0, Some(image)) => image.len() as u64,
            (size, _) => size,
        };
        if bytes == 0 {
            return Ok(None);
        }
        if !bytes.is_multiple_of(SECTOR) {
            return Err(config(format!(
                "a drive holds a whole number of {SECTOR}-byte sectors, and {bytes} bytes is not \
                 a whole number of them"
            )));
        }
        let total = bytes / SECTOR;

        let geometry = match (cylinders, heads, sectors) {
            (None, None, None) => default_geometry(total),
            (Some(c), Some(h), Some(s)) => {
                if c == 0
                    || c > u64::from(u16::MAX)
                    || !(1..=16).contains(&h)
                    || !(1..=255).contains(&s)
                {
                    return Err(config(format!(
                        "{c}/{h}/{s} is not a translation an ATA drive can report"
                    )));
                }
                Geometry {
                    cylinders: c as u16,
                    heads: h as u8,
                    sectors: s as u8,
                }
            }
            _ => {
                return Err(config(String::from(
                    "`cylinders`, `heads` and `sectors` come as a set: give all three or none",
                )));
            }
        };

        let mut id = Identity::new(total, geometry, lba48, max_multiple)?;
        id.read_only = read_only;
        id.model = model;
        id.serial = serial;
        id.firmware = firmware;

        let disk = AtaDisk::with_identity(id, position)?;
        if let Some(image) = image {
            if image.len() as u64 > bytes {
                return Err(config(format!(
                    "the bound image is {} byte(s) and the drive holds {bytes}",
                    image.len()
                )));
            }
            disk.load_image(0, &image)?;
        }
        Ok(Some(disk))
    }

    /// Build a drive from an identity the caller already has.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the capacity does not fit in this host's memory.
    pub fn with_identity(id: Identity, position: Position) -> Result<AtaDisk> {
        let bytes = id.capacity();
        if usize::try_from(bytes).is_err() {
            return Err(config(format!(
                "a drive of {bytes} byte(s) is larger than this host's address space"
            )));
        }
        let geometry = id.geometry;
        Ok(AtaDisk {
            id,
            position,
            media: Arc::new(RamStore::new(bytes)),
            state: Mutex::with_rank(LockRank::DEVICE, Volatile::power_on(position, geometry)),
        })
    }

    /// The drive's fixed identity.
    #[must_use]
    pub fn identity(&self) -> &Identity {
        &self.id
    }

    /// Which cable position it is jumpered to.
    #[must_use]
    pub fn position(&self) -> Position {
        self.position
    }

    /// Whether the Device register's DEV bit currently names this drive.
    ///
    /// The adapter's only question. Has no side effect: safe from a debugger.
    #[must_use]
    pub fn is_selected(&self) -> bool {
        self.state.lock().selected
    }

    /// Whether `INTRQ` is asserted — the drive's interrupt is pending *and*
    /// nIEN is not holding it off.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        let state = self.state.lock();
        state.irq && state.control & CTL_NIEN == 0
    }

    /// The Status register **without** clearing the pending interrupt.
    ///
    /// This is the Alternate Status register at the control block, and the
    /// reason it exists on real hardware: a driver — and a debugger — needs to
    /// look at the status without acknowledging anything. `MemAttrs::debug`
    /// (`ROADMAP.md` §15, invariant 5) is the same requirement one level up, and
    /// this method is how the adapter satisfies it for both.
    #[must_use]
    pub fn read_alt_status(&self) -> u8 {
        self.state.lock().status
    }

    /// The current CHS translation: what `INITIALIZE DEVICE PARAMETERS` last
    /// set, or the drive's default.
    #[must_use]
    pub fn current_geometry(&self) -> Geometry {
        self.state.lock().current
    }

    /// The `SET MULTIPLE MODE` block size, in sectors. Zero means the mode is
    /// off and `READ MULTIPLE` will abort.
    #[must_use]
    pub fn multiple(&self) -> u8 {
        self.state.lock().multiple
    }

    // -- the medium --------------------------------------------------------

    /// Read from the medium, ignoring the protocol entirely.
    ///
    /// The debugger's and the host's door, not the guest's — and the one a
    /// test uses to check that what the guest wrote reached the *image* rather
    /// than only the drive's own buffer.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the drive.
    pub fn read_media(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        self.media
            .read_at(offset, dst)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this drive")))
    }

    /// Write to the medium, ignoring the protocol entirely.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the drive.
    pub fn write_media(&self, offset: u64, src: &[u8]) -> Result<()> {
        self.media
            .write_at(offset, src)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this drive")))
    }

    /// Put `bytes` on the medium at `offset`. The image loader's door.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image runs off the end of the drive.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.media.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit on a drive of {}",
                bytes.len(),
                self.id.capacity()
            ))
        })
    }

    /// The whole medium as a fresh vector, for a snapshot or a test.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; self.id.capacity() as usize];
        // Cannot fail: the length is the store's own.
        let _ = self.media.read_at(0, &mut out);
        out
    }

    // -- power -------------------------------------------------------------

    /// A power-on or hardware reset.
    ///
    /// The contents survive, which is the whole point of the device; everything
    /// else goes back to the factory, including the CHS translation and the
    /// `SET MULTIPLE MODE` block size. A **software** reset preserves both,
    /// which is why [`AtaDisk::write_device_control`] does not call this.
    ///
    /// There is no clock domain here and therefore no tick to rewind — the trap
    /// a lazily-advanced device falls into when `Machine::reset` does not rewind
    /// its clock domain does not apply, and could not, because this device never
    /// asks what time it is.
    pub fn power_on_reset(&self) {
        let mut state = self.state.lock();
        *state = Volatile::power_on(self.position, self.id.geometry);
    }

    // -- the cable ---------------------------------------------------------

    /// Write one command block register.
    ///
    /// **Every drive on the cable sees every write**, which is why this takes no
    /// "is it for me" argument: a write to [`Reg::Device`] is how selection
    /// moves, so it is latched unconditionally, and every other write is ignored
    /// by a drive that is not selected. An adapter therefore broadcasts and
    /// does not have to know which bit means what.
    pub fn write_reg(&self, reg: Reg, value: u16) {
        let mut state = self.state.lock();
        let byte = value as u8;
        if reg == Reg::Device {
            state.device = byte;
            state.selected = (byte & DEV_SELECT != 0) == self.position.dev_bit();
            return;
        }
        if !state.selected {
            return;
        }
        // While the host holds SRST asserted the drive is busy and the command
        // block is the drive's, not the host's.
        if state.in_reset {
            return;
        }
        match reg {
            Reg::Data => self.write_data(&mut state, value),
            Reg::Feature => state.features.write(byte),
            Reg::SectorCount => state.count.write(byte),
            Reg::LbaLow => state.lba_low.write(byte),
            Reg::LbaMid => state.lba_mid.write(byte),
            Reg::LbaHigh => state.lba_high.write(byte),
            Reg::Command => self.command(&mut state, byte),
            Reg::Device => unreachable!("handled above"),
        }
    }

    /// Read one command block register.
    ///
    /// `debug` suppresses every side effect, and this device has two of them:
    /// reading the Status register clears the pending interrupt, and reading the
    /// Data register advances the sector buffer. A debugger must do neither
    /// (`ROADMAP.md` §15, invariant 5).
    pub fn read_reg(&self, reg: Reg, debug: bool) -> u16 {
        let mut state = self.state.lock();
        let hob = state.hob();
        match reg {
            Reg::Data => self.read_data(&mut state, debug),
            Reg::Feature => u16::from(state.error),
            Reg::SectorCount => u16::from(state.count.read(hob)),
            Reg::LbaLow => u16::from(state.lba_low.read(hob)),
            Reg::LbaMid => u16::from(state.lba_mid.read(hob)),
            Reg::LbaHigh => u16::from(state.lba_high.read(hob)),
            Reg::Device => u16::from(state.device | DEV_OBSOLETE),
            Reg::Command => {
                if !debug {
                    state.irq = false;
                }
                u16::from(state.status)
            }
        }
    }

    /// Write the Device Control register, which lives in the control block and
    /// which **every** drive on the cable sees.
    ///
    /// Three bits matter: HOB chooses which half of a 48-bit register a read
    /// returns, nIEN gates `INTRQ`, and SRST is a software reset that is held
    /// rather than pulsed — the drive is busy while it is asserted and takes the
    /// reset when it is released, which is the sequence a driver performs.
    pub fn write_device_control(&self, value: u8) {
        let mut state = self.state.lock();
        let was = state.control;
        state.control = value;
        match (was & CTL_SRST != 0, value & CTL_SRST != 0) {
            (false, true) => {
                // Asserted: the drive owns the command block from here.
                state.in_reset = true;
                state.status = ST_BSY;
                state.irq = false;
            }
            (true, false) => {
                // Released: take the reset. A software reset does **not**
                // assert INTRQ, and it preserves the current CHS translation
                // and the SET MULTIPLE MODE block size — which is the whole
                // difference between it and a power cycle.
                state.in_reset = false;
                state.signature();
                state.irq = false;
            }
            _ => {}
        }
    }

    // -- the data register -------------------------------------------------

    fn read_data(&self, state: &mut Volatile, debug: bool) -> u16 {
        // Nothing to hand over outside a data phase. The standard leaves the
        // result undefined; zero is what a drain loop expects to stop on.
        if state.status & ST_DRQ == 0 {
            return 0;
        }
        let at = state.pos;
        if at >= state.buf.len() {
            return 0;
        }
        let lo = u16::from(state.buf[at]);
        let hi = u16::from(state.buf.get(at + 1).copied().unwrap_or(0));
        let word = lo | (hi << 8);
        if debug {
            return word;
        }
        state.pos = at + 2;
        if state.pos >= state.buf.len() {
            self.block_consumed(state);
        }
        word
    }

    fn write_data(&self, state: &mut Volatile, value: u16) {
        if state.status & ST_DRQ == 0 {
            return;
        }
        let Some(xfer) = state.xfer.as_ref() else {
            return;
        };
        if !xfer.out {
            return;
        }
        let at = state.pos;
        if at >= state.buf.len() {
            return;
        }
        state.buf[at] = value as u8;
        if at + 1 < state.buf.len() {
            state.buf[at + 1] = (value >> 8) as u8;
        }
        state.pos = at + 2;
        if state.pos >= state.buf.len() {
            self.block_filled(state);
        }
    }

    /// The host has emptied a block the drive filled.
    fn block_consumed(&self, state: &mut Volatile) {
        state.status &= !ST_DRQ;
        let Some(xfer) = state.xfer.clone() else {
            // `IDENTIFY DEVICE` and friends: one block and no transfer.
            state.buf.clear();
            state.pos = 0;
            return;
        };
        if xfer.left == 0 {
            self.complete(state, xfer.last, xfer.mode);
            return;
        }
        self.fill_block(state);
    }

    /// The host has filled a block the drive is to write.
    fn block_filled(&self, state: &mut Volatile) {
        let Some(mut xfer) = state.xfer.clone() else {
            return;
        };
        let count = (state.buf.len() as u64) / SECTOR;
        let ok = self
            .media
            .write_at(xfer.next * SECTOR, &state.buf[..])
            .is_ok();
        state.status &= !ST_DRQ;
        if !ok {
            self.fail(state, ERR_IDNF, xfer.next, xfer.mode);
            return;
        }
        xfer.last = xfer.next + count - 1;
        xfer.next += count;
        state.xfer = Some(xfer.clone());
        if xfer.left == 0 {
            self.complete(state, xfer.last, xfer.mode);
        } else {
            self.open_block(state);
        }
        // A PIO data-out block is acknowledged at its *end*, which is why this
        // is here and not in `open_block`: the first block of a write gets DRQ
        // and no interrupt.
        state.irq = true;
    }

    /// Load the next block of a read into the buffer and raise DRQ and INTRQ.
    fn fill_block(&self, state: &mut Volatile) {
        let Some(mut xfer) = state.xfer.clone() else {
            return;
        };
        let count = u64::from(xfer.block).min(xfer.left);
        let mut buf = alloc::vec![0u8; (count * SECTOR) as usize];
        if self.media.read_at(xfer.next * SECTOR, &mut buf).is_err() {
            self.fail(state, ERR_IDNF, xfer.next, xfer.mode);
            return;
        }
        xfer.last = xfer.next + count - 1;
        xfer.next += count;
        xfer.left -= count;
        state.xfer = Some(xfer);
        state.buf = buf;
        state.pos = 0;
        state.status = ST_IDLE | ST_DRQ;
        // A PIO data-in block is announced at its *start*.
        state.irq = true;
    }

    /// Open the next block of a write: DRQ, and deliberately no interrupt.
    fn open_block(&self, state: &mut Volatile) {
        let Some(mut xfer) = state.xfer.clone() else {
            return;
        };
        let count = u64::from(xfer.block).min(xfer.left);
        xfer.left -= count;
        state.xfer = Some(xfer);
        state.buf = alloc::vec![0u8; (count * SECTOR) as usize];
        state.pos = 0;
        state.status = ST_IDLE | ST_DRQ;
    }

    /// One block of data with no transfer behind it: `IDENTIFY DEVICE`.
    fn one_block(&self, state: &mut Volatile, bytes: Vec<u8>) {
        state.xfer = None;
        state.buf = bytes;
        state.pos = 0;
        state.status = ST_IDLE | ST_DRQ;
        state.error = 0;
        state.irq = true;
    }

    // -- completion --------------------------------------------------------

    fn complete(&self, state: &mut Volatile, last: u64, mode: Mode) {
        state.status = ST_IDLE;
        state.error = 0;
        state.xfer = None;
        state.buf.clear();
        state.pos = 0;
        state.count.load(0, 0);
        self.store_address(state, last, mode);
        state.irq = true;
    }

    fn fail(&self, state: &mut Volatile, error: u8, at: u64, mode: Mode) {
        state.status = ST_IDLE | ST_ERR;
        state.error = error;
        state.xfer = None;
        state.buf.clear();
        state.pos = 0;
        self.store_address(state, at, mode);
        state.irq = true;
    }

    /// Abort a command that never got as far as an address.
    fn abort(&self, state: &mut Volatile) {
        state.status = ST_IDLE | ST_ERR;
        state.error = ERR_ABRT;
        state.xfer = None;
        state.buf.clear();
        state.pos = 0;
        state.irq = true;
    }

    fn succeed(&self, state: &mut Volatile) {
        state.status = ST_IDLE;
        state.error = 0;
        state.irq = true;
    }

    /// Put `lba` back in the command block the way `mode` names addresses.
    ///
    /// ATA has the drive leave the address of the last sector transferred — or
    /// the sector in error — in the registers when a command ends, and a BIOS
    /// that chains reads believes it.
    fn store_address(&self, state: &mut Volatile, lba: u64, mode: Mode) {
        match mode {
            Mode::Chs => {
                if let Some(Address::Chs {
                    cylinder,
                    head,
                    sector,
                }) = Address::from_lba(lba, &state.current)
                {
                    state.lba_low.load(sector, 0);
                    state.lba_mid.load(cylinder as u8, 0);
                    state.lba_high.load((cylinder >> 8) as u8, 0);
                    state.device = (state.device & !DEV_HEAD) | (head & DEV_HEAD);
                }
            }
            Mode::Lba28 => {
                state.lba_low.load(lba as u8, 0);
                state.lba_mid.load((lba >> 8) as u8, 0);
                state.lba_high.load((lba >> 16) as u8, 0);
                state.device = (state.device & !DEV_HEAD) | ((lba >> 24) as u8 & DEV_HEAD);
            }
            Mode::Lba48 => {
                state.lba_low.load(lba as u8, (lba >> 24) as u8);
                state.lba_mid.load((lba >> 8) as u8, (lba >> 32) as u8);
                state.lba_high.load((lba >> 16) as u8, (lba >> 40) as u8);
            }
        }
    }

    // -- addressing --------------------------------------------------------

    /// How this command names its address, given whether it is an EXT command.
    fn mode_of(state: &Volatile, ext: bool) -> Mode {
        if ext {
            Mode::Lba48
        } else if state.device & DEV_LBA != 0 {
            Mode::Lba28
        } else {
            Mode::Chs
        }
    }

    /// The address the command block currently holds, under `mode`.
    fn address(state: &Volatile, mode: Mode) -> Address {
        match mode {
            Mode::Chs => Address::Chs {
                cylinder: u16::from(state.lba_mid.current)
                    | (u16::from(state.lba_high.current) << 8),
                head: state.device & DEV_HEAD,
                sector: state.lba_low.current,
            },
            Mode::Lba28 => Address::Lba28(
                u32::from(state.lba_low.current)
                    | (u32::from(state.lba_mid.current) << 8)
                    | (u32::from(state.lba_high.current) << 16)
                    | (u32::from(state.device & DEV_HEAD) << 24),
            ),
            Mode::Lba48 => Address::Lba48(
                u64::from(state.lba_low.current)
                    | (u64::from(state.lba_mid.current) << 8)
                    | (u64::from(state.lba_high.current) << 16)
                    | (u64::from(state.lba_low.previous) << 24)
                    | (u64::from(state.lba_mid.previous) << 32)
                    | (u64::from(state.lba_high.previous) << 40),
            ),
        }
    }

    /// How many sectors the Sector Count register is asking for.
    ///
    /// Zero means the maximum in both schemes, which is 256 for a 28-bit
    /// command and 65536 for a 48-bit one — the one place the two differ by
    /// more than a width.
    fn count_of(state: &Volatile, mode: Mode) -> u64 {
        if mode == Mode::Lba48 {
            let n = u64::from(state.count.current) | (u64::from(state.count.previous) << 8);
            if n == 0 { 65536 } else { n }
        } else {
            let n = u64::from(state.count.current);
            if n == 0 { 256 } else { n }
        }
    }

    // -- the command set ---------------------------------------------------

    fn command(&self, state: &mut Volatile, opcode: u8) {
        // Writing the Command register negates INTRQ. A driver that relied on
        // the previous command's interrupt still being up would be relying on
        // hardware not doing this.
        state.irq = false;
        state.status = ST_IDLE;
        state.error = 0;
        state.xfer = None;
        state.buf.clear();
        state.pos = 0;

        // RECALIBRATE and SEEK are ranges rather than single opcodes: the low
        // four bits were a step rate on a part old enough to have one.
        let family = opcode & 0xf0;
        match opcode {
            cmd::IDENTIFY => {
                let block = self.identify_block(state);
                self.one_block(state, block);
            }
            cmd::IDENTIFY_PACKET => {
                // Not a packet device. Aborting is how a driver finds out, and
                // is the only honest answer while ATAPI is out of scope.
                self.abort(state);
            }
            cmd::READ_SECTORS | cmd::READ_SECTORS_NORETRY => self.transfer(state, false, false, 1),
            cmd::READ_SECTORS_EXT => self.transfer(state, false, true, 1),
            cmd::WRITE_SECTORS | cmd::WRITE_SECTORS_NORETRY => self.transfer(state, true, false, 1),
            cmd::WRITE_SECTORS_EXT => self.transfer(state, true, true, 1),
            cmd::READ_MULTIPLE => self.multiple_transfer(state, false, false),
            cmd::READ_MULTIPLE_EXT => self.multiple_transfer(state, false, true),
            cmd::WRITE_MULTIPLE => self.multiple_transfer(state, true, false),
            cmd::WRITE_MULTIPLE_EXT => self.multiple_transfer(state, true, true),
            cmd::VERIFY_SECTORS | cmd::VERIFY_SECTORS_NORETRY => self.verify(state, false),
            cmd::VERIFY_SECTORS_EXT => self.verify(state, true),
            cmd::SET_MULTIPLE => self.set_multiple(state),
            cmd::INIT_DEVICE_PARAMS => self.init_device_params(state),
            cmd::DIAGNOSTIC => {
                state.signature();
                state.irq = true;
            }
            cmd::FLUSH_CACHE | cmd::FLUSH_CACHE_EXT => {
                // Every write reached the medium inside the call that carried
                // it, so there is nothing to flush and success is the truth.
                self.succeed(state);
            }
            cmd::SET_FEATURES => self.set_features(state),
            cmd::READ_NATIVE_MAX => {
                let last = self.id.sectors - 1;
                self.succeed(state);
                self.store_address(state, last, Mode::Lba28);
                state.device |= DEV_LBA;
            }
            cmd::READ_NATIVE_MAX_EXT => {
                let last = self.id.sectors - 1;
                self.succeed(state);
                self.store_address(state, last, Mode::Lba48);
                state.device |= DEV_LBA;
            }
            cmd::STANDBY_IMMEDIATE
            | cmd::IDLE_IMMEDIATE
            | cmd::STANDBY
            | cmd::IDLE
            | cmd::SLEEP => self.succeed(state),
            cmd::CHECK_POWER_MODE => {
                // 0xff is "active or idle", which this drive always is.
                state.count.load(0xff, 0);
                self.succeed(state);
            }
            cmd::NOP => {
                // Specified to be aborted. Not a no-op, whatever the name says.
                self.abort(state);
            }
            _ if family == cmd::RECALIBRATE => {
                // The heads are at track zero, because there are no heads.
                self.succeed(state);
                let mode = Self::mode_of(state, false);
                self.store_address(state, 0, mode);
            }
            _ if family == cmd::SEEK => self.seek(state),
            _ => self.abort(state),
        }
    }

    /// `READ SECTOR(S)`, `WRITE SECTOR(S)` and their EXT forms.
    fn transfer(&self, state: &mut Volatile, out: bool, ext: bool, block: u32) {
        if ext && !self.id.lba48 {
            self.abort(state);
            return;
        }
        if out && self.id.read_only {
            // A write-protected medium. ATA reports this as an aborted command;
            // the write-protect bit in the Error register belongs to ATAPI.
            self.abort(state);
            return;
        }
        let mode = Self::mode_of(state, ext);
        let count = Self::count_of(state, mode);
        let Some(lba) = Self::address(state, mode).to_lba(&state.current) else {
            // A CHS triple the current translation cannot name.
            self.no_such_address(state, ERR_IDNF);
            return;
        };
        if lba >= self.id.sectors || count > self.id.sectors - lba {
            self.fail(state, ERR_IDNF, lba.min(self.id.sectors - 1), mode);
            return;
        }
        state.xfer = Some(Transfer {
            out,
            next: lba,
            left: count,
            block,
            mode,
            last: lba,
        });
        if out {
            self.open_block(state);
        } else {
            self.fill_block(state);
        }
    }

    /// `READ MULTIPLE` / `WRITE MULTIPLE` and their EXT forms.
    ///
    /// The only difference from the single-sector commands is the block size,
    /// and that the mode has to have been turned on first: a drive on which
    /// `SET MULTIPLE MODE` has not run aborts these.
    fn multiple_transfer(&self, state: &mut Volatile, out: bool, ext: bool) {
        let block = state.multiple;
        if block == 0 {
            self.abort(state);
            return;
        }
        self.transfer(state, out, ext, u32::from(block));
    }

    fn verify(&self, state: &mut Volatile, ext: bool) {
        if ext && !self.id.lba48 {
            self.abort(state);
            return;
        }
        let mode = Self::mode_of(state, ext);
        let count = Self::count_of(state, mode);
        let Some(lba) = Self::address(state, mode).to_lba(&state.current) else {
            self.no_such_address(state, ERR_IDNF);
            return;
        };
        if lba >= self.id.sectors || count > self.id.sectors - lba {
            self.fail(state, ERR_IDNF, lba.min(self.id.sectors - 1), mode);
            return;
        }
        let last = lba + count - 1;
        self.complete(state, last, mode);
    }

    fn seek(&self, state: &mut Volatile) {
        let mode = Self::mode_of(state, false);
        let Some(lba) = Self::address(state, mode).to_lba(&state.current) else {
            self.no_such_address(state, ERR_IDNF);
            return;
        };
        if lba >= self.id.sectors {
            self.fail(state, ERR_IDNF, self.id.sectors - 1, mode);
            return;
        }
        self.succeed(state);
    }

    fn set_multiple(&self, state: &mut Volatile) {
        let requested = state.count.current;
        // A power of two, at least one, no larger than word 47 advertises.
        // Zero is rejected rather than treated as "disable", because the
        // standard's own text has moved on that point between revisions and an
        // abort is the answer a driver copes with either way.
        if requested == 0 || !requested.is_power_of_two() || requested > self.id.max_multiple {
            self.abort(state);
            return;
        }
        state.multiple = requested;
        self.succeed(state);
    }

    fn init_device_params(&self, state: &mut Volatile) {
        let sectors = state.count.current;
        let heads = (state.device & DEV_HEAD) + 1;
        if sectors == 0 {
            self.abort(state);
            return;
        }
        // The host declares the translation and the drive believes it; the
        // cylinder count is what is left. This is exactly how a BIOS makes the
        // geometry it computed and the geometry the drive decodes against agree
        // — and the reason `IDENTIFY` words 54-56 report *this*, not the
        // default.
        let per_cylinder = u64::from(heads) * u64::from(sectors);
        let cylinders = (self.id.sectors / per_cylinder).min(u64::from(u16::MAX));
        state.current = Geometry {
            cylinders: cylinders as u16,
            heads,
            sectors,
        };
        self.succeed(state);
    }

    fn set_features(&self, state: &mut Volatile) {
        // The subcommands a PIO-only drive can honestly answer. Everything
        // else aborts, which is what a device that does not implement a
        // feature is required to do.
        const SET_TRANSFER_MODE: u8 = 0x03;
        const ENABLE_WRITE_CACHE: u8 = 0x02;
        const DISABLE_WRITE_CACHE: u8 = 0x82;
        const DISABLE_READ_LOOKAHEAD: u8 = 0x55;
        const ENABLE_READ_LOOKAHEAD: u8 = 0xaa;
        const DISABLE_REVERT_ON_POWER_UP: u8 = 0x66;
        const ENABLE_REVERT_ON_POWER_UP: u8 = 0xcc;
        match state.features.current {
            SET_TRANSFER_MODE => {
                // Only PIO modes exist here: the transfer mode value's top
                // three bits are the class, and anything that is not PIO
                // (0b000) or PIO flow control (0b001) is a DMA mode this drive
                // does not have.
                let mode = state.count.current;
                if mode >> 3 <= 1 {
                    self.succeed(state);
                } else {
                    self.abort(state);
                }
            }
            ENABLE_WRITE_CACHE
            | DISABLE_WRITE_CACHE
            | DISABLE_READ_LOOKAHEAD
            | ENABLE_READ_LOOKAHEAD
            | DISABLE_REVERT_ON_POWER_UP
            | ENABLE_REVERT_ON_POWER_UP => self.succeed(state),
            _ => self.abort(state),
        }
    }

    /// End a command that never got as far as a valid address.
    ///
    /// `IDNF` rather than `ABRT`: the command was one this drive has, and the
    /// address was one the current translation cannot name, which is what the
    /// Error register's "ID not found" bit is for.
    fn no_such_address(&self, state: &mut Volatile, error: u8) {
        state.status = ST_IDLE | ST_ERR;
        state.error = error;
        state.xfer = None;
        state.buf.clear();
        state.pos = 0;
        state.irq = true;
    }

    // -- IDENTIFY DEVICE ---------------------------------------------------

    /// The 256-word `IDENTIFY DEVICE` response, as 512 bytes in transfer order.
    ///
    /// Word *n* occupies bytes 2n and 2n+1 little-endian, and an ASCII field
    /// holds its **first** character in the *high* byte of each word — which is
    /// why the strings below are byte-swapped in pairs, and why a model that
    /// forgets shows a driver "IRDAHSD KSI".
    fn identify_block(&self, state: &Volatile) -> Vec<u8> {
        let mut w = [0u16; 256];
        let id = &self.id;

        // Word 0: an ATA device (bit 15 clear), not removable.
        w[0] = 0x0040;
        // Words 1, 3, 6: the *default* CHS translation.
        w[1] = id.geometry.cylinders;
        w[3] = u16::from(id.geometry.heads);
        w[6] = u16::from(id.geometry.sectors);
        put_string(&mut w[10..20], &id.serial);
        put_string(&mut w[23..27], &id.firmware);
        put_string(&mut w[27..47], &id.model);
        // Word 47: the largest READ/WRITE MULTIPLE block, with the constant
        // 0x80 the standard puts in the high byte.
        w[47] = 0x8000 | u16::from(id.max_multiple);
        // Word 49: LBA supported (bit 9), IORDY supported (bit 11) and may be
        // disabled (bit 10). DMA (bit 8) is deliberately clear — this drive
        // moves data through the data register and nothing else.
        w[49] = (1 << 9) | (1 << 10) | (1 << 11);
        // Word 50: bit 14 is set to distinguish the word from a zero one.
        w[50] = 0x4000;
        // Word 51: PIO data transfer cycle timing mode 2, in the high byte.
        w[51] = 0x0200;
        // Word 53: words 54-58 are valid (bit 0), and 64-70 (bit 1).
        w[53] = 0x0003;
        // Words 54-58: the *current* translation, and how much it addresses.
        // This is the pair a BIOS and this drive have to agree on.
        w[54] = state.current.cylinders;
        w[55] = u16::from(state.current.heads);
        w[56] = u16::from(state.current.sectors);
        let chs_capacity = state.current.addressable().min(u64::from(u32::MAX));
        w[57] = chs_capacity as u16;
        w[58] = (chs_capacity >> 16) as u16;
        // Word 59: the current multiple setting, valid only with bit 8 set.
        w[59] = if state.multiple == 0 {
            0
        } else {
            0x0100 | u16::from(state.multiple)
        };
        // Words 60-61: how many sectors 28-bit addressing reaches.
        let lba28 = id.sectors.min(LBA28_LIMIT - 1);
        w[60] = lba28 as u16;
        w[61] = (lba28 >> 16) as u16;
        // Word 64: PIO modes 3 and 4 supported.
        w[64] = 0x0003;
        // Words 67-68: minimum PIO cycle time, with and without IORDY, in ns.
        w[67] = 120;
        w[68] = 120;
        // Word 80: ATA/ATAPI-4, -5 and -6.
        w[80] = (1 << 4) | (1 << 5) | (1 << 6);
        // Words 83 and 86: the 48-bit Address feature set, supported and
        // enabled. Bit 14 set and bit 15 clear is what makes word 83 valid.
        w[83] = 0x4000 | if id.lba48 { 1 << 10 } else { 0 };
        w[84] = 0x4000;
        w[86] = if id.lba48 { 1 << 10 } else { 0 };
        w[87] = 0x4000;
        if id.lba48 {
            // Words 100-103: the whole capacity, 48 bits of it.
            w[100] = id.sectors as u16;
            w[101] = (id.sectors >> 16) as u16;
            w[102] = (id.sectors >> 32) as u16;
            w[103] = (id.sectors >> 48) as u16;
        }

        let mut out = alloc::vec![0u8; 512];
        for (i, word) in w.iter().enumerate() {
            out[i * 2] = *word as u8;
            out[i * 2 + 1] = (*word >> 8) as u8;
        }
        // Word 255: the signature 0xa5 in the low byte and a checksum in the
        // high one, chosen so that the 512 bytes sum to zero modulo 256.
        out[510] = 0xa5;
        let sum: u8 = out[..511].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        out[511] = 0u8.wrapping_sub(sum);
        out
    }

    // -- snapshots ---------------------------------------------------------

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The medium, exactly as `sd.card` and `dev-flash-cfi` save theirs: a
        // drive's contents are guest-visible state, and a snapshot that
        // restored to different bytes would be a snapshot of a different
        // machine.
        w.write_bytes(&self.contents())?;
        let state = self.state.lock();
        w.write_bool(state.selected)?;
        w.write_u8(state.device)?;
        w.write_u8(state.error)?;
        w.write_u8(state.status)?;
        w.write_u8(state.control)?;
        for fifo in [
            state.features,
            state.count,
            state.lba_low,
            state.lba_mid,
            state.lba_high,
        ] {
            w.write_u8(fifo.current)?;
            w.write_u8(fifo.previous)?;
        }
        w.write_bool(state.irq)?;
        w.write_bool(state.in_reset)?;
        w.write_u8(state.multiple)?;
        w.write_u16(state.current.cylinders)?;
        w.write_u8(state.current.heads)?;
        w.write_u8(state.current.sectors)?;
        // A transfer part way through its sector buffer *is* state: the buffer
        // and the position in it both have to come back, or a guest that
        // snapshotted mid-sector resumes reading somebody else's data.
        w.write_bytes(&state.buf)?;
        w.write_u64(state.pos as u64)?;
        match &state.xfer {
            None => w.write_bool(false)?,
            Some(x) => {
                w.write_bool(true)?;
                w.write_bool(x.out)?;
                w.write_u64(x.next)?;
                w.write_u64(x.left)?;
                w.write_u32(x.block)?;
                w.write_u8(match x.mode {
                    Mode::Chs => 0,
                    Mode::Lba28 => 1,
                    Mode::Lba48 => 2,
                })?;
                w.write_u64(x.last)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        if bytes.len() as u64 != self.id.capacity() {
            return Err(Error::State(format!(
                "the snapshot holds a drive of {} byte(s), this one holds {}",
                bytes.len(),
                self.id.capacity()
            )));
        }
        self.media
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the drive refused the snapshot")))?;
        let selected = r.read_bool()?;
        let device = r.read_u8()?;
        let error = r.read_u8()?;
        let status = r.read_u8()?;
        let control = r.read_u8()?;
        let mut fifos = [Fifo::default(); 5];
        for fifo in &mut fifos {
            fifo.current = r.read_u8()?;
            fifo.previous = r.read_u8()?;
        }
        let irq = r.read_bool()?;
        let in_reset = r.read_bool()?;
        let multiple = r.read_u8()?;
        let current = Geometry {
            cylinders: r.read_u16()?,
            heads: r.read_u8()?,
            sectors: r.read_u8()?,
        };
        let buf = r.read_bytes()?.to_vec();
        let pos = r.read_u64()?;
        if pos > buf.len() as u64 {
            return Err(Error::State(format!(
                "a snapshot buffer position of {pos} is past the {} byte(s) it holds",
                buf.len()
            )));
        }
        let xfer = if r.read_bool()? {
            let out = r.read_bool()?;
            let next = r.read_u64()?;
            let left = r.read_u64()?;
            let block = r.read_u32()?;
            let mode = match r.read_u8()? {
                0 => Mode::Chs,
                1 => Mode::Lba28,
                2 => Mode::Lba48,
                other => {
                    return Err(Error::State(format!(
                        "{other} is not an addressing mode this drive has"
                    )));
                }
            };
            let last = r.read_u64()?;
            if block == 0 || next > self.id.sectors || left > self.id.sectors {
                return Err(Error::State(format!(
                    "a snapshot transfer of {left} sector(s) from {next} is not one this drive \
                     could have started"
                )));
            }
            Some(Transfer {
                out,
                next,
                left,
                block,
                mode,
                last,
            })
        } else {
            None
        };
        let mut state = self.state.lock();
        state.selected = selected;
        state.device = device;
        state.error = error;
        state.status = status;
        state.control = control;
        state.features = fifos[0];
        state.count = fifos[1];
        state.lba_low = fifos[2];
        state.lba_mid = fifos[3];
        state.lba_high = fifos[4];
        state.irq = irq;
        state.in_reset = in_reset;
        state.multiple = multiple;
        state.current = current;
        state.buf = buf;
        state.pos = pos as usize;
        state.xfer = xfer;
        Ok(())
    }
}

/// Put `text` into an ATA ASCII field, space padded and byte-swapped in pairs.
fn put_string(words: &mut [u16], text: &str) {
    let bytes = text.as_bytes();
    for (i, word) in words.iter_mut().enumerate() {
        let hi = bytes.get(i * 2).copied().unwrap_or(b' ');
        let lo = bytes.get(i * 2 + 1).copied().unwrap_or(b' ');
        // Non-ASCII would be a machine file's doing; a space is a safer thing
        // to hand firmware than a byte it will print as line noise.
        let hi = if hi.is_ascii_graphic() || hi == b' ' {
            hi
        } else {
            b' '
        };
        let lo = if lo.is_ascii_graphic() || lo == b' ' {
            lo
        } else {
            b' '
        };
        *word = (u16::from(hi) << 8) | u16::from(lo);
    }
}

fn config(message: String) -> Error {
    Error::Config {
        at: String::from(CLASS_NAME),
        message,
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The `ata.disk` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an ATA hard disk: the command block, the command set and CHS/LBA addressing",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the drive holds; absent or zero takes the image's length, \
                      and with no image that is an empty bay",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the initial contents; the rest reads zero",
        },
        PropertySpec {
            name: "bay",
            kind: ValueKind::Str,
            required: false,
            summary: "the named drive bay this drive is fitted in (default `ata0`)",
        },
        PropertySpec {
            name: "position",
            kind: ValueKind::Str,
            required: false,
            summary: "`master` (device 0, the default) or `slave` (device 1)",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "write protect the medium: a write command aborts",
        },
        PropertySpec {
            name: "lba48",
            kind: ValueKind::Bool,
            required: false,
            summary: "advertise and accept the 48-bit Address feature set (default true)",
        },
        PropertySpec {
            name: "multiple",
            kind: ValueKind::Uint,
            required: false,
            summary: "the largest READ/WRITE MULTIPLE block, in sectors; a power of two",
        },
        PropertySpec {
            name: "cylinders",
            kind: ValueKind::Uint,
            required: false,
            summary: "the default CHS translation's cylinders; give all three or none",
        },
        PropertySpec {
            name: "heads",
            kind: ValueKind::Uint,
            required: false,
            summary: "the default CHS translation's heads, 1 to 16",
        },
        PropertySpec {
            name: "sectors",
            kind: ValueKind::Uint,
            required: false,
            summary: "the default CHS translation's sectors per track, 1 to 255",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "the IDENTIFY model string, forty characters",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Str,
            required: false,
            summary: "the IDENTIFY serial number; a constant, because a run must be reproducible",
        },
        PropertySpec {
            name: "firmware",
            kind: ValueKind::Str,
            required: false,
            summary: "the IDENTIFY firmware revision, eight characters",
        },
    ],
    construct: |props| Ok(Box::new(DiskDevice::new(props)?)),
};

/// An [`AtaDisk`] as a machine-description object.
///
/// The drive is not a memory-mapped device: it has no region, no pins and no
/// clock, because on real hardware it is on the far side of a cable from
/// anything that has those. What this wrapper adds is the two-phase
/// construction contract and the rendezvous — it fits the drive into a named
/// bay at construction, and a host adapter in the same build picks it up.
#[derive(Debug)]
pub struct DiskDevice {
    drive: Option<Arc<AtaDisk>>,
    bay: String,
}

impl DiskDevice {
    /// Validate `props`, build the drive if there is one, and fit it.
    ///
    /// # Errors
    ///
    /// As [`AtaDisk::new`], plus [`Error::Config`] if the named bay already
    /// holds a drive.
    pub fn new(props: &Props) -> Result<DiskDevice> {
        let bay = props
            .get("bay")
            .and_then(crate::core::props::Value::as_str)
            .unwrap_or(super::DEFAULT_BAY)
            .to_string();
        let Some(disk) = AtaDisk::new(props)? else {
            // An empty bay. The bay is still opened, so that an adapter naming
            // it finds a socket rather than nothing at all — which is the
            // difference between "no disk" and "a typo in a bay name".
            super::bays::attach(props, &bay)?;
            return Ok(DiskDevice { drive: None, bay });
        };
        let drive = Arc::new(disk);
        let holder = super::bays::attach(props, &bay)?;
        holder.fit(Arc::clone(&drive)).map_err(|_| {
            config(format!(
                "two drives were fitted in the bay called `{bay}`; give one of them another `bay`"
            ))
        })?;
        Ok(DiskDevice {
            drive: Some(drive),
            bay,
        })
    }

    /// The drive behind this object, if the bay is not empty.
    #[must_use]
    pub fn drive(&self) -> Option<&Arc<AtaDisk>> {
        self.drive.as_ref()
    }

    /// The bay it was fitted in.
    #[must_use]
    pub fn bay(&self) -> &str {
        &self.bay
    }
}

impl Device for DiskDevice {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. The bay was claimed at construction, which is
        // allocation rather than an observable action — the same argument
        // `core::hosts` makes for every other rendezvous.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. A board reset cycles the drive's power, which resets the
        // protocol state and leaves the contents alone — the distinction NOR
        // flash and an SD card both draw, and for the same reason.
        if let Some(drive) = &self.drive {
            drive.power_on_reset();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        match &self.drive {
            None => w.write_bool(false),
            Some(drive) => {
                w.write_bool(true)?;
                drive.save(w)
            }
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let occupied = r.read_bool()?;
        match (&self.drive, occupied) {
            (Some(drive), true) => drive.load(r),
            (None, false) => Ok(()),
            (Some(_), false) => Err(Error::State(String::from(
                "the snapshot has an empty bay and this machine has a drive in it",
            ))),
            (None, true) => Err(Error::State(String::from(
                "the snapshot has a drive and this machine's bay is empty",
            ))),
        }
    }
}

impl Instance for DiskDevice {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(DiskDevice::new(props)?)))
}

/// What the validator should know about `ata.disk`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("bay", ValueKind::Str))
        .prop(PropSchema::new("position", ValueKind::Str).values(&["master", "slave"]))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("lba48", ValueKind::Bool))
        .prop(PropSchema::new("multiple", ValueKind::Uint).range(1, 128))
        .prop(PropSchema::new("cylinders", ValueKind::Uint).range(1, 65535))
        .prop(PropSchema::new("heads", ValueKind::Uint).range(1, 16))
        .prop(PropSchema::new("sectors", ValueKind::Uint).range(1, 255))
        .prop(PropSchema::new("model", ValueKind::Str))
        .prop(PropSchema::new("serial", ValueKind::Str))
        .prop(PropSchema::new("firmware", ValueKind::Str))
}

#[cfg(test)]
mod tests;
