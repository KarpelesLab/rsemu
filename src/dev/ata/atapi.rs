//! An ATAPI CD-ROM: the packet interface, and the command set that rides it.
//!
//! **This models the drive, not a host adapter**, on exactly the terms
//! [`super::disk`] states: nothing here knows what `0x170` is, and nothing here
//! has a register *offset*. It reaches a cable through
//! [`AtaDevice`](super::AtaDevice), the same six calls a hard disk answers, and
//! [`crate::dev::pc::ide`] cannot tell which of the two it is talking to —
//! which is the truth about the ribbon cable and the reason ATAPI exists at
//! all.
//!
//! # What ATAPI is, and why it is a separate file
//!
//! A hard disk's command set is *addresses*: the host puts an LBA in the
//! registers and writes `READ SECTOR(S)`. A CD-ROM's is not, because a CD-ROM
//! is a SCSI peripheral that happened to be sold on an IDE cable in 1994. So
//! T13 defined one ATA command, `PACKET`, whose entire job is to carry a
//! twelve-byte SCSI **command descriptor block** through the data register; the
//! meaning of those twelve bytes is SFF-8020i's and later MMC's, and the ATA
//! layer neither knows nor cares what they say.
//!
//! That is why this is a sibling of `disk.rs` and not a flag on it. The two
//! share the register file — [`Reg`](super::Reg) is the same eight names, the
//! reset signature lands in the same four registers, `BSY` and `DRQ` are the
//! same two bits — and they share **not one line of command dispatch**. Delete
//! this file and `ata.disk` is unchanged; delete `disk.rs` and this one loses
//! six constants and a string formatter.
//!
//! ```text
//!   pc.ide ──► write_reg(Reg::Command, 0xa0)  ──► PACKET: open the CDB phase
//!          ──► write_reg(Reg::Data, …) x6     ──► twelve bytes of SCSI CDB
//!                                                 └─► READ(10), INQUIRY, …
//!          ◄── read_reg(Reg::Data) x n        ◄── the data the CDB asked for
//! ```
//!
//! # The handshake, precisely
//!
//! ATA/ATAPI-6 §9.10 (PACKET command protocol) and §8.21 (`PACKET`). The order
//! of the bits is the whole protocol, and a model that gets it nearly right
//! works with one driver and hangs the next:
//!
//! 1. The host writes the Features register (bit 0 `DMA`, bit 1 `OVL`), the
//!    **byte count limit** into the two Byte Count registers — which are the
//!    LBA Mid and LBA High registers wearing their packet names — and then
//!    `PACKET` into the Command register.
//! 2. The device raises `DRQ` with `BSY` cleared and sets the Interrupt Reason
//!    register (the Sector Count register's packet name) to `C/D = 1`,
//!    `I/O = 0`: *give me a command packet*. It does **not** interrupt, because
//!    `IDENTIFY PACKET DEVICE` word 0 bits 6:5 report microprocessor DRQ and a
//!    driver that read that word polls.
//! 3. The host writes six words. The device clears `DRQ`, works, and comes back
//!    either with a data block — `C/D = 0`, `I/O = 1`, the **actual** byte count
//!    in the Byte Count registers, `DRQ` set, `INTRQ` asserted — or with
//!    completion.
//! 4. Completion is `C/D = 1`, `I/O = 1`, `DRQ` clear, `INTRQ` asserted. Note
//!    that this happens after the *last* data block as well, which is the one
//!    place the packet protocol differs from §9.5's PIO data-in: an ATA read
//!    has no completion interrupt and an ATAPI one does. Counted, which is how
//!    the tests state it: a packet data-in command of *n* blocks interrupts
//!    *n + 1* times.
//!
//! A failure is `CHECK CONDITION`: `CHK` in the Status register, the sense key
//! in the Error register's top four bits, and the detail waiting for the
//! `REQUEST SENSE` the driver now has to issue.
//!
//! # Status bits mean different things here
//!
//! Three of the eight are renamed for a packet device, which is why this file
//! defines its own names for them rather than importing `disk`'s:
//!
//! | bit | non-packet | packet |
//! |-----|------------|--------|
//! | 6   | `DRDY`     | **always zero** — ATA/ATAPI-6 §7.15.6.3 |
//! | 5   | `DF`       | `DMRD`, DMA ready |
//! | 4   | `DSC`      | `SERV`, service request |
//! | 0   | `ERR`      | `CHK`, check condition |
//!
//! `DRDY` being permanently clear is what makes a packet device's Status
//! register read `0x00` at rest, and together with the `0xEB14` signature in
//! the two Byte Count registers it is how a driver tells the two kinds of
//! device apart before it has issued a single command.
//!
//! # The disc
//!
//! A [`Medium`], 2048 bytes to the logical block, and the drive never writes to
//! it — there is no `WRITE(10)` and no `MODE SELECT` in the command set below,
//! so the read-only-ness is structural rather than a flag that could be got
//! wrong. An empty [`medium::MediumSlot`] or an unbound media slot is a drive
//! with **no disc in it**, which is an ordinary CD-ROM drive and answers
//! `NOT READY`/`MEDIUM NOT PRESENT` rather than failing to exist.
//!
//! **2352-byte raw images are refused, not guessed at.** A `.bin`/`.cue` pair
//! carries the sync pattern, the header and the error-correction codes as well
//! as the user data, so its user area is at a stride this device does not
//! model; audio tracks have no user data at all and no `READ(10)` reaches them.
//! A file whose length is a multiple of 2352 and not of 2048 is therefore
//! rejected at construction with a message that says which format it looks like
//! — the alternative, reading it as though the sectors were 2048 bytes, would
//! hand a guest sixteen bytes of sync pattern where its boot record should be
//! and no error anywhere.
//!
//! # Sources
//!
//! * **T13, *AT Attachment with Packet Interface - 6* (ATA/ATAPI-6,
//!   T13/1410D)** — §7.15 (the Status and Error registers as a packet device
//!   uses them), §8.16 (`IDENTIFY PACKET DEVICE` and its data table), §8.21
//!   (`PACKET`), §8.7 (`DEVICE RESET`), §9.1 (the reset signature) and §9.10
//!   (the packet command protocol).
//! * **SFF-8020i**, *ATA Packet Interface for CD-ROMs*, revision 2.6 — the
//!   packet command set: `TEST UNIT READY`, `REQUEST SENSE`, `INQUIRY`,
//!   `START/STOP UNIT`, `PREVENT/ALLOW MEDIUM REMOVAL`, `READ CD-ROM CAPACITY`,
//!   `READ(10)`, `READ(12)`, `SEEK(10)`, `READ TOC`, `MODE SENSE(6)` and
//!   `MODE SENSE(10)`, the mode pages, and the fixed-format sense data.
//! * **MMC** (SCSI Multi-Media Commands) for mode page 2Ah.
//!
//! **No emulator source of any licence was consulted, and no operating
//! system's ATAPI or SCSI driver was opened** (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::AtaDevice;
use super::disk::{
    AtaDisk, CTL_NIEN, CTL_SRST, DEV_OBSOLETE, DEV_SELECT, ERR_ABRT, Position, Reg, ST_BSY, ST_DRQ,
    put_string,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source as _};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::medium::{self, Medium, Snapshot};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "ata.cdrom";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes one logical block of a data CD holds.
///
/// 2048, which is Mode 1 and Mode 2 Form 1 user data (ISO 9660's own unit).
pub const BLOCK: u64 = 2048;

/// How many bytes one *raw* CD sector holds, sync pattern and ECC included.
///
/// Named only so that [`AtapiDrive::new`] can recognise a raw image and refuse
/// it by name rather than reading it as though it were cooked.
pub const RAW_BLOCK: u64 = 2352;

/// How many bytes the command descriptor block of an ATAPI CD-ROM is.
///
/// Twelve. `IDENTIFY PACKET DEVICE` word 0 bits 1:0 report it, and sixteen is
/// the other legal value; this drive reports and accepts twelve, which is what
/// every CD-ROM of the period did.
pub const PACKET_BYTES: usize = 12;

// ---------------------------------------------------------------------------
// The Status register, as a packet device uses it
// ---------------------------------------------------------------------------

/// DMA ready. Bit 5, which a non-packet device calls `DF`.
pub const ST_DMRD: u8 = 0x20;
/// Service request. Bit 4, which a non-packet device calls `DSC`.
pub const ST_SERV: u8 = 0x10;
/// Check condition: the command failed and `REQUEST SENSE` says why.
///
/// Bit 0, which a non-packet device calls `ERR`.
pub const ST_CHK: u8 = 0x01;

// ---------------------------------------------------------------------------
// The Interrupt Reason register (the Sector Count register's packet name)
// ---------------------------------------------------------------------------

/// Command/data: set while the register block holds a command packet rather
/// than data.
pub const IR_CD: u8 = 0x01;
/// Input/output: set when the transfer is device to host.
pub const IR_IO: u8 = 0x02;
/// Release: the device has released the bus. Never set here — this drive does
/// not implement the overlapped feature set.
pub const IR_REL: u8 = 0x04;

// ---------------------------------------------------------------------------
// The signature
// ---------------------------------------------------------------------------

/// What a packet device leaves in the LBA Mid register after a reset.
pub const SIGNATURE_MID: u8 = 0x14;
/// What a packet device leaves in the LBA High register after a reset.
pub const SIGNATURE_HIGH: u8 = 0xeb;

// ---------------------------------------------------------------------------
// ATA commands
// ---------------------------------------------------------------------------

/// The opcodes this drive answers in the **Command** register.
///
/// Seven, of which two exist to be refused. Everything a CD-ROM actually does
/// arrives inside a `PACKET`.
pub mod cmd {
    /// `NOP`. Specified to be aborted.
    pub const NOP: u8 = 0x00;
    /// `DEVICE RESET`: reset the packet device without touching the cable.
    pub const DEVICE_RESET: u8 = 0x08;
    /// `EXECUTE DEVICE DIAGNOSTIC`.
    pub const DIAGNOSTIC: u8 = 0x90;
    /// `PACKET`: carry a command descriptor block through the data register.
    pub const PACKET: u8 = 0xa0;
    /// `IDENTIFY PACKET DEVICE`.
    pub const IDENTIFY_PACKET: u8 = 0xa1;
    /// `IDENTIFY DEVICE`. Aborted, and the signature restored with it.
    pub const IDENTIFY: u8 = 0xec;
    /// `SET FEATURES`.
    pub const SET_FEATURES: u8 = 0xef;
}

// ---------------------------------------------------------------------------
// Packet commands
// ---------------------------------------------------------------------------

/// The command descriptor block opcodes this drive answers.
///
/// SFF-8020i's mandatory set plus the three a boot and an install reach for.
/// Everything else is `ILLEGAL REQUEST`/`INVALID COMMAND OPERATION CODE`,
/// which is how a driver discovers what a drive has.
pub mod packet {
    /// `TEST UNIT READY`.
    pub const TEST_UNIT_READY: u8 = 0x00;
    /// `REQUEST SENSE`.
    pub const REQUEST_SENSE: u8 = 0x03;
    /// `INQUIRY`.
    pub const INQUIRY: u8 = 0x12;
    /// `MODE SENSE(6)`.
    pub const MODE_SENSE_6: u8 = 0x1a;
    /// `START/STOP UNIT`.
    pub const START_STOP_UNIT: u8 = 0x1b;
    /// `PREVENT/ALLOW MEDIUM REMOVAL`.
    pub const PREVENT_ALLOW: u8 = 0x1e;
    /// `READ CD-ROM CAPACITY`.
    pub const READ_CAPACITY: u8 = 0x25;
    /// `READ(10)`.
    pub const READ_10: u8 = 0x28;
    /// `SEEK(10)`.
    pub const SEEK_10: u8 = 0x2b;
    /// `READ TOC/PMA/ATIP`.
    pub const READ_TOC: u8 = 0x43;
    /// `MODE SENSE(10)`.
    pub const MODE_SENSE_10: u8 = 0x5a;
    /// `READ(12)`.
    pub const READ_12: u8 = 0xa8;
}

/// The sense keys this drive reports. SFF-8020i §9.4.
pub mod sense_key {
    /// Nothing to report.
    pub const NO_SENSE: u8 = 0x00;
    /// The drive cannot do it yet: no disc, mostly.
    pub const NOT_READY: u8 = 0x02;
    /// The disc could not be read.
    pub const MEDIUM_ERROR: u8 = 0x03;
    /// The command descriptor block asked for something impossible.
    pub const ILLEGAL_REQUEST: u8 = 0x05;
    /// Something changed under the host: a reset, or a new disc.
    pub const UNIT_ATTENTION: u8 = 0x06;
}

/// The additional sense codes this drive reports, as `(ASC, ASCQ)` pairs.
pub mod asc {
    /// No additional information.
    pub const NONE: (u8, u8) = (0x00, 0x00);
    /// Unrecovered read error: the host's storage said no.
    pub const UNRECOVERED_READ: (u8, u8) = (0x11, 0x00);
    /// Invalid command operation code.
    pub const INVALID_COMMAND: (u8, u8) = (0x20, 0x00);
    /// Logical block address out of range.
    pub const LBA_OUT_OF_RANGE: (u8, u8) = (0x21, 0x00);
    /// Invalid field in the command descriptor block.
    pub const INVALID_FIELD: (u8, u8) = (0x24, 0x00);
    /// Not ready to ready transition: the disc may have changed.
    pub const MEDIUM_CHANGED: (u8, u8) = (0x28, 0x00);
    /// Power on, reset or bus device reset occurred.
    pub const RESET_OCCURRED: (u8, u8) = (0x29, 0x00);
    /// Medium removal prevented.
    pub const REMOVAL_PREVENTED: (u8, u8) = (0x53, 0x02);
    /// Medium not present.
    pub const MEDIUM_NOT_PRESENT: (u8, u8) = (0x3a, 0x00);
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

/// Everything about a drive that does not change while it is running.
#[derive(Debug, Clone)]
pub struct Identity {
    /// The model string, `IDENTIFY PACKET DEVICE` words 27-46.
    pub model: String,
    /// The serial number, words 10-19.
    pub serial: String,
    /// The firmware revision, words 23-26.
    pub firmware: String,
    /// The eight-character vendor identification `INQUIRY` reports.
    pub vendor: String,
    /// Whether the DMA bit of the Features register is accepted.
    ///
    /// Off by default, and the default is the honest one for a drive on a plain
    /// AT-class IDE cable: nothing on that board moves bytes for the drive.
    pub dma: bool,
}

impl Identity {
    /// The factory settings.
    #[must_use]
    pub fn new() -> Identity {
        Identity {
            model: String::from("RSEMU CD-ROM"),
            serial: String::from("RSEMU00000000000001"),
            firmware: String::from("1.0"),
            vendor: String::from("RSEMU"),
            dma: false,
        }
    }
}

impl Default for Identity {
    fn default() -> Identity {
        Identity::new()
    }
}

// ---------------------------------------------------------------------------
// Sense
// ---------------------------------------------------------------------------

/// What the next `REQUEST SENSE` will report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Sense {
    key: u8,
    asc: u8,
    ascq: u8,
}

impl Sense {
    const NONE: Sense = Sense {
        key: sense_key::NO_SENSE,
        asc: 0,
        ascq: 0,
    };

    fn new(key: u8, (asc, ascq): (u8, u8)) -> Sense {
        Sense { key, asc, ascq }
    }
}

// ---------------------------------------------------------------------------
// A transfer in progress
// ---------------------------------------------------------------------------

/// Where the bytes of a data-in phase come from.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Origin {
    /// A response the drive built in one go: small, and every one of them is.
    Built(Vec<u8>),
    /// The disc, from this byte offset onwards.
    Disc(u64),
}

/// A packet command's data-in phase, part way through.
///
/// **This is state**, which is why it is in the snapshot: a guest suspended
/// between the 200th and 201st byte of a `READ(10)` block must carry on at the
/// 201st.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Xfer {
    /// Bytes not yet placed in a DRQ block. The block the host is draining now
    /// is *not* counted here.
    left: u64,
    /// How many bytes have already been taken out of `from`.
    taken: u64,
    /// Where the rest comes from.
    from: Origin,
}

/// Which half of the packet protocol the drive is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// No `DRQ`, nothing owed.
    Idle,
    /// `DRQ` is up and the drive is waiting for a command descriptor block.
    Cdb,
    /// `DRQ` is up over a block of data the host is to read.
    DataIn,
    /// `DRQ` is up over one block that is not a packet command's — the
    /// `IDENTIFY PACKET DEVICE` response, which is an ordinary PIO data-in.
    Identify,
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
    /// The Error register: sense key in 7:4, then `ABRT`, `EOM`, `ILI`.
    error: u8,
    status: u8,
    /// The Device Control register: SRST and nIEN.
    control: u8,
    /// The Features register: bit 0 DMA, bit 1 OVL.
    features: u8,
    /// The Interrupt Reason register.
    reason: u8,
    /// The two Byte Count registers, low in the bottom half.
    byte_count: u16,
    /// The LBA Low register, which a packet device does not use for anything
    /// but its share of the reset signature.
    lba_low: u8,
    /// `INTRQ`, before nIEN gates it.
    irq: bool,
    /// The host is holding `SRST` asserted.
    in_reset: bool,
    /// What the next `REQUEST SENSE` reports.
    sense: Sense,
    /// A condition that must be reported before any other command runs.
    ///
    /// The SCSI unit-attention rule (SFF-8020i §9.3): a reset or a disc change
    /// is reported once, to the first command that is neither `INQUIRY` nor
    /// `REQUEST SENSE`, and that command fails so that the host knows to start
    /// again.
    attention: Option<Sense>,
    /// `PREVENT/ALLOW MEDIUM REMOVAL` has locked the tray.
    locked: bool,
    /// The buffer under `DRQ`, and how far through it the host has got.
    buf: Vec<u8>,
    pos: usize,
    stage: Stage,
    xfer: Option<Xfer>,
}

impl Volatile {
    fn power_on(position: Position) -> Volatile {
        let mut state = Volatile {
            selected: position == Position::Device0,
            device: 0,
            error: 0,
            status: 0,
            control: 0,
            features: 0,
            reason: 0,
            byte_count: 0,
            lba_low: 0,
            irq: false,
            in_reset: false,
            sense: Sense::NONE,
            attention: Some(Sense::new(sense_key::UNIT_ATTENTION, asc::RESET_OCCURRED)),
            locked: false,
            buf: Vec::new(),
            pos: 0,
            stage: Stage::Idle,
            xfer: None,
        };
        state.signature();
        state
    }

    /// The signature a reset leaves in the command block.
    ///
    /// ATA/ATAPI-6 §9.1: a packet device answers 0x01 / 0x01 / 0x14 / 0xEB in
    /// the Sector Count, LBA Low, LBA Mid and LBA High registers, with the
    /// Status register **zero** — `DRDY` is not a bit a packet device has
    /// (§7.15.6.3), so there is nothing else in it. That pair, the `0xEB14`
    /// and the zero, is the whole of how a driver tells a CD-ROM from a hard
    /// disk before issuing a command.
    fn signature(&mut self) {
        self.error = 0;
        self.reason = 1;
        self.lba_low = 1;
        self.byte_count = (u16::from(SIGNATURE_HIGH) << 8) | u16::from(SIGNATURE_MID);
        self.status = 0;
        self.device &= DEV_SELECT;
        self.buf.clear();
        self.pos = 0;
        self.stage = Stage::Idle;
        self.xfer = None;
    }
}

/// An ATAPI CD-ROM drive.
///
/// Construct it with [`AtapiDrive::new`] from machine-description properties,
/// or with [`AtapiDrive::with_disc`] directly.
pub struct AtapiDrive {
    id: Identity,
    position: Position,
    /// The disc, if there is one in the drive. `None` is an empty tray.
    disc: Option<Arc<dyn Medium>>,
    /// How many 2048-byte logical blocks the disc holds.
    blocks: u64,
    state: Mutex<Volatile>,
}

impl fmt::Debug for AtapiDrive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AtapiDrive")
            .field("position", &self.position)
            .field("blocks", &self.blocks)
            .field("disc", &self.disc)
            .finish_non_exhaustive()
    }
}

impl AtapiDrive {
    /// Validate `props` and build the drive.
    ///
    /// Unlike [`AtaDisk::new`](super::AtaDisk::new) this never reports "no
    /// device": a CD-ROM drive with nothing in it is an ordinary CD-ROM drive,
    /// and the guest is meant to find it and be told `MEDIUM NOT PRESENT`.
    /// What is optional is the *disc*.
    ///
    /// # Where the bytes come from
    ///
    /// The same two sources `ata.disk` has, chosen the same way: a
    /// [`MediumSlot`](medium::MediumSlot) a host filled wins — that is what
    /// `rsemu run pc-at --drive cdrom=disc.iso` installs — and otherwise the
    /// media table's bytes go into a [`RamStore`].
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind;
    /// [`Error::Config`] if the disc's length is not a whole number of
    /// 2048-byte logical blocks — including the case where it is a whole number
    /// of 2352-byte *raw* sectors, which is reported as the raw image it is.
    pub fn new(props: &Props) -> Result<AtapiDrive> {
        let mut r = props.reader();
        let media = r.optional_media("image")?;
        let slot = media.map(crate::core::props::Media::name);
        let image = media.map(crate::core::props::Media::to_bytes);
        let position = match r.or_enum("position", "master", &["master", "slave"])? {
            "slave" => Position::Device1,
            _ => Position::Device0,
        };
        let dma = r.or("dma", false)?;
        let model = r.or_str("model", "RSEMU CD-ROM")?.to_string();
        let serial = r.or_str("serial", "RSEMU0000000000000001")?.to_string();
        let firmware = r.or_str("firmware", "1.0")?.to_string();
        let vendor = r.or_str("vendor", "RSEMU")?.to_string();
        let bay = r.optional_str("bay")?;
        r.finish()?;

        let supplied = match props.hosts() {
            Some(hosts) => {
                let name = slot.unwrap_or_else(|| bay.unwrap_or(super::DEFAULT_BAY));
                medium::get(hosts, name)?.and_then(|slot| slot.take())
            }
            None => None,
        };

        let mut id = Identity::new();
        id.model = model;
        id.serial = serial;
        id.firmware = firmware;
        id.vendor = vendor;
        id.dma = dma;

        let disc: Option<Arc<dyn Medium>> = match (supplied, image) {
            (Some(medium), _) => Some(medium),
            // An image slot bound to no bytes is how a front end says "there is
            // no disc in the drive" without the machine description needing an
            // `if`.
            (None, Some(bytes)) if !bytes.is_empty() => {
                let store = RamStore::new(bytes.len() as u64);
                store.write_at(0, &bytes).map_err(|e| {
                    config(format!("the disc image does not fit in host memory: {e:?}"))
                })?;
                Some(Arc::new(store))
            }
            _ => None,
        };
        AtapiDrive::with_disc(id, position, disc)
    }

    /// Build a drive around a disc the caller already has, or around none.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the medium's length is not a whole number of
    /// [`BLOCK`]-byte logical blocks.
    pub fn with_disc(
        id: Identity,
        position: Position,
        disc: Option<Arc<dyn Medium>>,
    ) -> Result<AtapiDrive> {
        let blocks = match &disc {
            None => 0,
            Some(medium) => {
                let bytes = medium.capacity();
                if bytes == 0 {
                    return Err(config(String::from(
                        "a disc of zero bytes is an empty drive; leave the media slot unbound",
                    )));
                }
                if !bytes.is_multiple_of(BLOCK) {
                    // The common way to get here is a raw `.bin` rip, and
                    // saying so is worth the branch: the failure mode of
                    // reading one as though it were cooked is sixteen bytes of
                    // sync pattern where the boot record should be, with no
                    // error anywhere.
                    if bytes.is_multiple_of(RAW_BLOCK) {
                        return Err(config(format!(
                            "{bytes} bytes is {} raw {RAW_BLOCK}-byte CD sectors; this drive \
                             reads {BLOCK}-byte logical blocks and does not model the sync \
                             pattern, the header or the error-correction codes a raw image \
                             carries. Convert it to a 2048-byte-per-sector image first",
                            bytes / RAW_BLOCK
                        )));
                    }
                    return Err(config(format!(
                        "a disc holds a whole number of {BLOCK}-byte logical blocks, and \
                         {bytes} bytes is not a whole number of them"
                    )));
                }
                bytes / BLOCK
            }
        };
        Ok(AtapiDrive {
            id,
            position,
            disc,
            blocks,
            state: Mutex::with_rank(LockRank::DEVICE, Volatile::power_on(position)),
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

    /// How many 2048-byte logical blocks the disc holds; zero with no disc.
    #[must_use]
    pub fn blocks(&self) -> u64 {
        self.blocks
    }

    /// The disc, for a host that wants to look at it directly.
    #[must_use]
    pub fn disc(&self) -> Option<&Arc<dyn Medium>> {
        self.disc.as_ref()
    }

    // -- the cable ---------------------------------------------------------

    fn do_write_reg(&self, reg: Reg, value: u16) {
        let mut state = self.state.lock();
        let byte = value as u8;
        if reg == Reg::Device {
            state.device = byte;
            state.selected = (byte & DEV_SELECT != 0) == (self.position == Position::Device1);
            return;
        }
        if !state.selected || state.in_reset {
            return;
        }
        match reg {
            Reg::Data => self.write_data(&mut state, value),
            Reg::Feature => state.features = byte,
            // The Sector Count register is the Interrupt Reason register and
            // the device owns it; the overlapped feature set would take a tag
            // out of bits 7:3, and this drive does not implement it.
            Reg::SectorCount => {}
            Reg::LbaLow => state.lba_low = byte,
            Reg::LbaMid => state.byte_count = (state.byte_count & 0xff00) | u16::from(byte),
            Reg::LbaHigh => {
                state.byte_count = (state.byte_count & 0x00ff) | (u16::from(byte) << 8);
            }
            Reg::Command => self.command(&mut state, byte),
            Reg::Device => unreachable!("handled above"),
        }
    }

    fn do_read_reg(&self, reg: Reg, debug: bool) -> u16 {
        let mut state = self.state.lock();
        match reg {
            Reg::Data => self.read_data(&mut state, debug),
            Reg::Feature => u16::from(state.error),
            Reg::SectorCount => u16::from(state.reason),
            Reg::LbaLow => u16::from(state.lba_low),
            Reg::LbaMid => u16::from(state.byte_count as u8),
            Reg::LbaHigh => u16::from((state.byte_count >> 8) as u8),
            Reg::Device => u16::from(state.device | DEV_OBSOLETE),
            Reg::Command => {
                if !debug {
                    state.irq = false;
                }
                u16::from(state.status)
            }
        }
    }

    fn do_write_device_control(&self, value: u8) {
        let mut state = self.state.lock();
        let was = state.control;
        state.control = value;
        match (was & CTL_SRST != 0, value & CTL_SRST != 0) {
            (false, true) => {
                state.in_reset = true;
                state.status = ST_BSY;
                state.irq = false;
            }
            (true, false) => {
                // A software reset leaves the signature and does not interrupt.
                // It is also a reason to raise a unit attention, because the
                // host has just thrown away whatever the drive was doing.
                state.in_reset = false;
                state.signature();
                state.sense = Sense::NONE;
                state.attention = Some(Sense::new(sense_key::UNIT_ATTENTION, asc::RESET_OCCURRED));
                state.irq = false;
            }
            _ => {}
        }
    }

    // -- the data register -------------------------------------------------

    fn read_data(&self, state: &mut Volatile, debug: bool) -> u16 {
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
        if state.status & ST_DRQ == 0 || state.stage != Stage::Cdb {
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
            let cdb = core::mem::take(&mut state.buf);
            state.pos = 0;
            state.status &= !ST_DRQ;
            state.stage = Stage::Idle;
            self.execute(state, &cdb);
        }
    }

    /// The host has emptied a block the drive filled.
    fn block_consumed(&self, state: &mut Volatile) {
        state.status &= !ST_DRQ;
        state.buf.clear();
        state.pos = 0;
        match state.stage {
            Stage::Identify => {
                // An ordinary PIO data-in command, which is what
                // `IDENTIFY PACKET DEVICE` is: ATA/ATAPI-6 §9.5 DPIOI1:DI1,
                // no interrupt on this transition.
                state.stage = Stage::Idle;
                state.status = 0;
            }
            Stage::DataIn => self.next_block(state),
            _ => state.stage = Stage::Idle,
        }
    }

    /// Open the next data-in block, or complete the command.
    fn next_block(&self, state: &mut Volatile) {
        let Some(xfer) = state.xfer.as_ref() else {
            self.good(state);
            return;
        };
        if xfer.left == 0 {
            self.good(state);
            return;
        }
        // ATA/ATAPI-6 §8.21.5: the byte count limit is the most the device may
        // move in one DRQ block, and an odd value is rounded down because the
        // data register is sixteen bits wide. Zero is a reserved value; reading
        // it as the full 65536 is the lenient choice and costs nothing, because
        // no host deliberately asks for a limit of no bytes, while aborting
        // would turn a driver's oversight into an unexplained hang.
        let limit = if state.byte_count == 0 {
            65536u64
        } else {
            u64::from(state.byte_count & !1)
        };
        let limit = limit.max(2);
        let n = core::cmp::min(limit, xfer.left);
        let mut buf = alloc::vec![0u8; n as usize];
        let taken = xfer.taken;
        let ok = match &xfer.from {
            Origin::Built(bytes) => {
                let at = taken as usize;
                buf.copy_from_slice(&bytes[at..at + n as usize]);
                true
            }
            Origin::Disc(base) => match &self.disc {
                Some(disc) => disc.read_at(base + taken, &mut buf).is_ok(),
                None => false,
            },
        };
        if !ok {
            self.check(state, sense_key::MEDIUM_ERROR, asc::UNRECOVERED_READ);
            return;
        }
        let xfer = state.xfer.as_mut().expect("checked above");
        xfer.taken += n;
        xfer.left -= n;
        state.buf = buf;
        state.pos = 0;
        state.byte_count = n as u16;
        state.reason = IR_IO;
        state.status = ST_DRQ;
        state.stage = Stage::DataIn;
        state.irq = true;
    }

    // -- completion --------------------------------------------------------

    /// End a packet command successfully.
    ///
    /// ATA/ATAPI-6 §9.10: `C/D` and `I/O` both set, `DRQ` clear, and `INTRQ`
    /// asserted — *including* after the last block of a data-in command, which
    /// is the one place the packet protocol differs from §9.5's PIO data-in.
    fn good(&self, state: &mut Volatile) {
        state.status = 0;
        state.error = 0;
        state.reason = IR_CD | IR_IO;
        state.buf.clear();
        state.pos = 0;
        state.stage = Stage::Idle;
        state.xfer = None;
        state.irq = true;
    }

    /// End a packet command with `CHECK CONDITION`.
    fn check(&self, state: &mut Volatile, key: u8, asc: (u8, u8)) {
        state.sense = Sense::new(key, asc);
        state.status = ST_CHK;
        state.error = key << 4;
        state.reason = IR_CD | IR_IO;
        state.buf.clear();
        state.pos = 0;
        state.stage = Stage::Idle;
        state.xfer = None;
        state.irq = true;
    }

    /// Abort an **ATA** command — one that never became a packet.
    fn abort(&self, state: &mut Volatile) {
        state.status = ST_CHK;
        state.error = (sense_key::ILLEGAL_REQUEST << 4) | ERR_ABRT;
        state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_COMMAND);
        state.reason = IR_CD | IR_IO;
        state.buf.clear();
        state.pos = 0;
        state.stage = Stage::Idle;
        state.xfer = None;
        state.irq = true;
    }

    fn succeed(&self, state: &mut Volatile) {
        state.status = 0;
        state.error = 0;
        state.irq = true;
    }

    // -- the ATA command set -----------------------------------------------

    fn command(&self, state: &mut Volatile, opcode: u8) {
        state.irq = false;
        state.status = 0;
        state.error = 0;
        state.buf.clear();
        state.pos = 0;
        state.stage = Stage::Idle;
        state.xfer = None;
        match opcode {
            cmd::PACKET => self.start_packet(state),
            cmd::IDENTIFY_PACKET => {
                let block = self.identify_block();
                state.buf = block;
                state.pos = 0;
                state.stage = Stage::Identify;
                state.status = ST_DRQ;
                state.byte_count = 512;
                state.reason = IR_IO;
                state.irq = true;
            }
            cmd::IDENTIFY => {
                // ATA/ATAPI-6 §8.15: a packet device aborts `IDENTIFY DEVICE`
                // and **restores the signature** with it, so that a host which
                // probed the wrong way round still learns what this is.
                self.abort(state);
                let status = state.status;
                let error = state.error;
                state.signature();
                state.status = status;
                state.error = error;
            }
            cmd::DEVICE_RESET => {
                // §8.7: reset the packet device. The signature comes back and
                // INTRQ is **not** asserted.
                state.signature();
                state.sense = Sense::NONE;
                state.attention = Some(Sense::new(sense_key::UNIT_ATTENTION, asc::RESET_OCCURRED));
                state.locked = false;
                state.irq = false;
            }
            cmd::DIAGNOSTIC => {
                state.signature();
                // The diagnostic code, which is the one thing the Error
                // register holds after this command rather than a sense key.
                state.error = 0x01;
                state.irq = true;
            }
            cmd::SET_FEATURES => self.set_features(state),
            // Specified to be aborted, whatever the name says.
            cmd::NOP => self.abort(state),
            _ => self.abort(state),
        }
    }

    fn set_features(&self, state: &mut Volatile) {
        const SET_TRANSFER_MODE: u8 = 0x03;
        if state.features != SET_TRANSFER_MODE {
            self.abort(state);
            return;
        }
        // The transfer mode itself lives in the Sector Count register, which
        // this device does not latch: for a packet device that register is the
        // Interrupt Reason register and the device owns it, so there is nothing
        // to inspect. Accepting the subcommand is therefore the only answer
        // available, and it is the right one — every mode a host can ask this
        // drive for is one it can honour, because the bytes move through the
        // data register whatever the mode says.
        self.succeed(state);
    }

    /// `PACKET`: open the command-packet phase.
    fn start_packet(&self, state: &mut Volatile) {
        // Bit 1 is OVL, the overlapped feature set, which this drive does not
        // implement; bit 0 is DMA, which it implements only where a host
        // adapter can move the bytes.
        if state.features & 0x02 != 0 || (state.features & 0x01 != 0 && !self.id.dma) {
            self.abort(state);
            return;
        }
        state.buf = alloc::vec![0u8; PACKET_BYTES];
        state.pos = 0;
        state.stage = Stage::Cdb;
        state.reason = IR_CD;
        state.status = ST_DRQ;
        // No interrupt: `IDENTIFY PACKET DEVICE` word 0 bits 6:5 report
        // microprocessor DRQ, so a driver polls for this one.
        state.irq = false;
    }

    // -- the packet command set --------------------------------------------

    fn execute(&self, state: &mut Volatile, cdb: &[u8]) {
        let opcode = cdb[0];
        // SFF-8020i §9.3: a pending unit attention is reported to the first
        // command that is neither `INQUIRY` nor `REQUEST SENSE`, and that
        // command does not run.
        if opcode != packet::INQUIRY
            && opcode != packet::REQUEST_SENSE
            && let Some(pending) = state.attention.take()
        {
            self.check(state, pending.key, (pending.asc, pending.ascq));
            return;
        }
        match opcode {
            packet::TEST_UNIT_READY => {
                if self.disc.is_none() {
                    self.check(state, sense_key::NOT_READY, asc::MEDIUM_NOT_PRESENT);
                } else {
                    state.sense = Sense::NONE;
                    self.good(state);
                }
            }
            packet::REQUEST_SENSE => {
                let alloc_len = u64::from(cdb[4]);
                let sense = state.sense;
                state.sense = Sense::NONE;
                self.data_in(state, request_sense(sense), alloc_len);
            }
            packet::INQUIRY => {
                if cdb[1] & 0x01 != 0 {
                    // The vital product data pages. Not implemented, and
                    // saying so is what lets a host fall back.
                    self.check(state, sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD);
                    return;
                }
                let alloc_len = u64::from(cdb[4]);
                self.data_in(state, self.inquiry(), alloc_len);
            }
            packet::MODE_SENSE_6 => {
                let alloc_len = u64::from(cdb[4]);
                self.mode_sense(state, cdb[2], alloc_len, false);
            }
            packet::MODE_SENSE_10 => {
                let alloc_len = be16(&cdb[7..9]);
                self.mode_sense(state, cdb[2], alloc_len, true);
            }
            packet::START_STOP_UNIT => {
                let eject = cdb[4] & 0x02 != 0 && cdb[4] & 0x01 == 0;
                if eject && state.locked {
                    self.check(state, sense_key::ILLEGAL_REQUEST, asc::REMOVAL_PREVENTED);
                } else {
                    // Spinning up, spinning down and opening a tray that is not
                    // modelled are all a no-op with a good status: what is in
                    // the drive is the *host's* business, and a guest that
                    // could eject a disc the host cannot put back would be a
                    // worse model, not a better one.
                    self.good(state);
                }
            }
            packet::PREVENT_ALLOW => {
                state.locked = cdb[4] & 0x01 != 0;
                self.good(state);
            }
            packet::READ_CAPACITY => {
                if self.disc.is_none() {
                    self.check(state, sense_key::NOT_READY, asc::MEDIUM_NOT_PRESENT);
                    return;
                }
                let mut out = Vec::with_capacity(8);
                // The address of the **last** block, not the count.
                out.extend_from_slice(&((self.blocks - 1) as u32).to_be_bytes());
                out.extend_from_slice(&(BLOCK as u32).to_be_bytes());
                self.data_in(state, out, 8);
            }
            packet::READ_10 => {
                let lba = u64::from(be32(&cdb[2..6]));
                let blocks = be16(&cdb[7..9]);
                self.read(state, lba, blocks);
            }
            packet::READ_12 => {
                let lba = u64::from(be32(&cdb[2..6]));
                let blocks = u64::from(be32(&cdb[6..10]));
                self.read(state, lba, blocks);
            }
            packet::SEEK_10 => {
                if self.disc.is_none() {
                    self.check(state, sense_key::NOT_READY, asc::MEDIUM_NOT_PRESENT);
                    return;
                }
                let lba = u64::from(be32(&cdb[2..6]));
                if lba >= self.blocks {
                    self.check(state, sense_key::ILLEGAL_REQUEST, asc::LBA_OUT_OF_RANGE);
                } else {
                    self.good(state);
                }
            }
            packet::READ_TOC => {
                let msf = cdb[1] & 0x02 != 0;
                let format = cdb[2] & 0x0f;
                let start = cdb[6];
                let alloc_len = be16(&cdb[7..9]);
                self.read_toc(state, msf, format, start, alloc_len);
            }
            _ => self.check(state, sense_key::ILLEGAL_REQUEST, asc::INVALID_COMMAND),
        }
    }

    /// Start a data-in phase over a response the drive has already built.
    ///
    /// `alloc_len` is the command descriptor block's allocation length, and the
    /// rule is SCSI's: the device transfers the lesser of what it has and what
    /// was asked for, and an allocation length of zero is a successful command
    /// that moves nothing.
    fn data_in(&self, state: &mut Volatile, mut bytes: Vec<u8>, alloc_len: u64) {
        let n = core::cmp::min(bytes.len() as u64, alloc_len);
        if n == 0 {
            self.good(state);
            return;
        }
        bytes.truncate(n as usize);
        state.xfer = Some(Xfer {
            left: n,
            taken: 0,
            from: Origin::Built(bytes),
        });
        state.sense = Sense::NONE;
        self.next_block(state);
    }

    /// `READ(10)` and `READ(12)`.
    fn read(&self, state: &mut Volatile, lba: u64, blocks: u64) {
        if self.disc.is_none() {
            self.check(state, sense_key::NOT_READY, asc::MEDIUM_NOT_PRESENT);
            return;
        }
        if blocks == 0 {
            // Not an error: a transfer length of zero is a successful command
            // that reads nothing.
            self.good(state);
            return;
        }
        if lba >= self.blocks || blocks > self.blocks - lba {
            self.check(state, sense_key::ILLEGAL_REQUEST, asc::LBA_OUT_OF_RANGE);
            return;
        }
        state.xfer = Some(Xfer {
            left: blocks * BLOCK,
            taken: 0,
            from: Origin::Disc(lba * BLOCK),
        });
        state.sense = Sense::NONE;
        self.next_block(state);
    }

    /// `MODE SENSE(6)` and `MODE SENSE(10)`, which differ only in their header.
    fn mode_sense(&self, state: &mut Volatile, page_control: u8, alloc_len: u64, long: bool) {
        let page = page_control & 0x3f;
        let mut pages: Vec<u8> = Vec::new();
        let all = page == 0x3f;
        if all || page == 0x01 {
            pages.extend_from_slice(&read_error_recovery_page());
        }
        if all || page == 0x0d {
            pages.extend_from_slice(&cd_parameters_page());
        }
        if all || page == 0x2a {
            pages.extend_from_slice(&capabilities_page());
        }
        if pages.is_empty() {
            self.check(state, sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD);
            return;
        }
        let mut out: Vec<u8> = Vec::new();
        // Byte 2 of either header is the device-specific parameter, whose bit 7
        // is WP: a CD-ROM is write protected by construction.
        if long {
            let len = (pages.len() + 6) as u16;
            out.extend_from_slice(&len.to_be_bytes());
            out.push(0x00); // medium type
            out.push(0x80); // write protected
            out.extend_from_slice(&[0, 0, 0, 0]); // reserved, no block descriptors
        } else {
            out.push((pages.len() + 3) as u8);
            out.push(0x00);
            out.push(0x80);
            out.push(0x00);
        }
        out.extend_from_slice(&pages);
        self.data_in(state, out, alloc_len);
    }

    /// `READ TOC/PMA/ATIP`, formats 0 and 1.
    ///
    /// A disc this device models has exactly one track: a data track, number 1,
    /// starting at block 0, in one session. Formats 2 and above describe
    /// multi-session and raw sub-channel layouts that a single-track ISO image
    /// does not have, and are refused rather than invented.
    fn read_toc(&self, state: &mut Volatile, msf: bool, format: u8, start: u8, alloc_len: u64) {
        if self.disc.is_none() {
            self.check(state, sense_key::NOT_READY, asc::MEDIUM_NOT_PRESENT);
            return;
        }
        let lead_out = self.blocks;
        match format {
            0 => {
                // A track number above the last one, and above the lead-out's
                // 0xAA, is an invalid field.
                if start > 1 && start != 0xaa {
                    self.check(state, sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD);
                    return;
                }
                let mut body: Vec<u8> = Vec::new();
                if start <= 1 {
                    body.extend_from_slice(&track_descriptor(1, 0, msf));
                }
                body.extend_from_slice(&track_descriptor(0xaa, lead_out, msf));
                let mut out: Vec<u8> = Vec::new();
                out.extend_from_slice(&((body.len() + 2) as u16).to_be_bytes());
                out.push(1); // first track
                out.push(1); // last track
                out.extend_from_slice(&body);
                self.data_in(state, out, alloc_len);
            }
            1 => {
                // Multi-session information: one session, whose first track is
                // track 1 at block 0.
                let mut out: Vec<u8> = Vec::new();
                out.extend_from_slice(&10u16.to_be_bytes());
                out.push(1); // first session
                out.push(1); // last session
                out.extend_from_slice(&track_descriptor(1, 0, msf));
                self.data_in(state, out, alloc_len);
            }
            _ => self.check(state, sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD),
        }
    }

    /// The 36-byte standard `INQUIRY` response. SFF-8020i §10.4.
    fn inquiry(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; 36];
        // Peripheral qualifier 000b, peripheral device type 05h: a CD-ROM.
        out[0] = 0x05;
        // RMB: the medium is removable.
        out[1] = 0x80;
        // ATAPI version 0, response data format 2 — which is what says the
        // eight-byte vendor and sixteen-byte product fields below are where
        // SCSI-2 puts them.
        out[3] = 0x32;
        // Additional length: everything after this byte.
        out[4] = 31;
        ascii_field(&mut out[8..16], &self.id.vendor);
        ascii_field(&mut out[16..32], &self.id.model);
        ascii_field(&mut out[32..36], &self.id.firmware);
        out
    }

    // -- IDENTIFY PACKET DEVICE --------------------------------------------

    /// The 256-word `IDENTIFY PACKET DEVICE` response, in transfer order.
    ///
    /// ATA/ATAPI-6 §8.16. Word 0 is the one that matters most and is the one a
    /// model is most likely to get wrong, so it is spelled out field by field.
    fn identify_block(&self) -> Vec<u8> {
        let mut w = [0u16; 256];
        let id = &self.id;

        // Word 0, bit by bit:
        //   15:14 = 10b   this is an ATAPI device
        //   12:8  = 5     the command packet set is the CD-ROM one
        //   7     = 1     the medium is removable
        //   6:5   = 00b   microprocessor DRQ: the host polls for the packet
        //                 phase rather than waiting for an interrupt. The
        //                 slowest of the three promises, which is the safe one
        //                 to make: a device faster than it claims breaks
        //                 nothing, and one that claimed interrupt DRQ and then
        //                 did not interrupt would hang its driver.
        //   1:0   = 00b   a twelve-byte command packet
        w[0] = 0x8000 | (5 << 8) | 0x0080;
        put_string(&mut w[10..20], &id.serial);
        put_string(&mut w[23..27], &id.firmware);
        put_string(&mut w[27..47], &id.model);
        // Word 49: LBA supported, IORDY supported and may be disabled, and DMA
        // only where a host adapter can move the bytes.
        w[49] = (1 << 9) | (1 << 10) | (1 << 11) | if id.dma { 1 << 8 } else { 0 };
        w[50] = 0x4000;
        // Word 53: words 64-70 are valid, and word 88 when there is a DMA mode
        // to report. Bit 0 covers words 54-58, the current CHS translation,
        // which a packet device does not have.
        w[53] = 0x0002 | if id.dma { 0x0004 } else { 0 };
        if id.dma {
            w[63] = 0x0007 | (1 << (8 + 2));
            w[88] = 0x003f | (1 << (8 + 5));
        }
        // Word 64: PIO modes 3 and 4.
        w[64] = 0x0003;
        // Words 67-68: minimum PIO cycle time, with and without IORDY, in ns.
        w[67] = 120;
        w[68] = 120;
        // Words 71-72: how long after `PACKET` the device releases the bus, in
        // nanoseconds. Zero: it never does, because it has no overlapped
        // feature set to release it for.
        w[71] = 0;
        w[72] = 0;
        // Word 80: ATA/ATAPI-4, -5 and -6.
        w[80] = (1 << 4) | (1 << 5) | (1 << 6);
        // Word 82: the command sets supported — `NOP`, `DEVICE RESET` and the
        // `PACKET` feature set. Word 85 is the same list, enabled.
        w[82] = (1 << 14) | (1 << 9) | (1 << 4);
        w[83] = 0x4000;
        w[84] = 0x4000;
        w[85] = (1 << 14) | (1 << 9) | (1 << 4);
        w[86] = 0;
        w[87] = 0x4000;

        let mut out = alloc::vec![0u8; 512];
        for (i, word) in w.iter().enumerate() {
            out[i * 2] = *word as u8;
            out[i * 2 + 1] = (*word >> 8) as u8;
        }
        out[510] = 0xa5;
        let sum: u8 = out[..511].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        out[511] = 0u8.wrapping_sub(sum);
        out
    }

    // -- power -------------------------------------------------------------

    /// A power-on or hardware reset.
    ///
    /// The disc survives — it is in the tray, not in the electronics — and
    /// everything else goes back to the factory, including the tray lock.
    pub fn power_on(&self) {
        let mut state = self.state.lock();
        *state = Volatile::power_on(self.position);
    }

    // -- snapshots ---------------------------------------------------------

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        match &self.disc {
            None => w.write_bool(false)?,
            Some(disc) => {
                w.write_bool(true)?;
                match disc.snapshot() {
                    Snapshot::Capture => {
                        let mut bytes = alloc::vec![0u8; disc.capacity() as usize];
                        disc.read_at(0, &mut bytes)
                            .map_err(|e| medium::error_at(0, e))?;
                        w.write_bytes(&bytes)?;
                    }
                    Snapshot::Reference => w.write_bytes(disc.describe().as_bytes())?,
                    Snapshot::Refuse => {
                        return Err(Error::State(format!(
                            "this drive's disc ({}) refuses to be snapshotted",
                            disc.describe()
                        )));
                    }
                }
            }
        }
        let state = self.state.lock();
        w.write_bool(state.selected)?;
        w.write_u8(state.device)?;
        w.write_u8(state.error)?;
        w.write_u8(state.status)?;
        w.write_u8(state.control)?;
        w.write_u8(state.features)?;
        w.write_u8(state.reason)?;
        w.write_u16(state.byte_count)?;
        w.write_u8(state.lba_low)?;
        w.write_bool(state.irq)?;
        w.write_bool(state.in_reset)?;
        w.write_u8(state.sense.key)?;
        w.write_u8(state.sense.asc)?;
        w.write_u8(state.sense.ascq)?;
        match state.attention {
            None => w.write_bool(false)?,
            Some(pending) => {
                w.write_bool(true)?;
                w.write_u8(pending.key)?;
                w.write_u8(pending.asc)?;
                w.write_u8(pending.ascq)?;
            }
        }
        w.write_bool(state.locked)?;
        w.write_bytes(&state.buf)?;
        w.write_u64(state.pos as u64)?;
        w.write_u8(match state.stage {
            Stage::Idle => 0,
            Stage::Cdb => 1,
            Stage::DataIn => 2,
            Stage::Identify => 3,
        })?;
        match &state.xfer {
            None => w.write_bool(false)?,
            Some(x) => {
                w.write_bool(true)?;
                w.write_u64(x.left)?;
                w.write_u64(x.taken)?;
                match &x.from {
                    Origin::Built(bytes) => {
                        w.write_bool(false)?;
                        w.write_bytes(bytes)?;
                    }
                    Origin::Disc(at) => {
                        w.write_bool(true)?;
                        w.write_u64(*at)?;
                    }
                }
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let had_disc = r.read_bool()?;
        match (&self.disc, had_disc) {
            (Some(disc), true) => {
                let bytes: &[u8] = r.read_bytes()?;
                match disc.snapshot() {
                    Snapshot::Capture => {
                        if bytes.len() as u64 != disc.capacity() {
                            return Err(Error::State(format!(
                                "the snapshot holds a disc of {} byte(s), this one holds {}",
                                bytes.len(),
                                disc.capacity()
                            )));
                        }
                        disc.write_at(0, bytes).map_err(|e| {
                            Error::State(format!("the disc refused the snapshot: {e:?}"))
                        })?;
                    }
                    Snapshot::Reference => {
                        let want = disc.describe();
                        if bytes != want.as_bytes() {
                            return Err(Error::State(format!(
                                "the snapshot references a different disc: it names `{}` and \
                                 this drive holds `{want}`",
                                String::from_utf8_lossy(&bytes[..bytes.len().min(120)])
                            )));
                        }
                    }
                    Snapshot::Refuse => {
                        return Err(Error::State(format!(
                            "this drive's disc ({}) refuses to be snapshotted",
                            disc.describe()
                        )));
                    }
                }
            }
            (None, false) => {}
            (Some(_), false) => {
                return Err(Error::State(String::from(
                    "the snapshot has an empty drive and this one has a disc in it",
                )));
            }
            (None, true) => {
                return Err(Error::State(String::from(
                    "the snapshot has a disc and this drive is empty",
                )));
            }
        }
        let selected = r.read_bool()?;
        let device = r.read_u8()?;
        let error = r.read_u8()?;
        let status = r.read_u8()?;
        let control = r.read_u8()?;
        let features = r.read_u8()?;
        let reason = r.read_u8()?;
        let byte_count = r.read_u16()?;
        let lba_low = r.read_u8()?;
        let irq = r.read_bool()?;
        let in_reset = r.read_bool()?;
        let sense = Sense {
            key: r.read_u8()?,
            asc: r.read_u8()?,
            ascq: r.read_u8()?,
        };
        let attention = if r.read_bool()? {
            Some(Sense {
                key: r.read_u8()?,
                asc: r.read_u8()?,
                ascq: r.read_u8()?,
            })
        } else {
            None
        };
        let locked = r.read_bool()?;
        let buf = r.read_bytes()?.to_vec();
        let pos = r.read_u64()?;
        if pos > buf.len() as u64 {
            return Err(Error::State(format!(
                "a snapshot buffer position of {pos} is past the {} byte(s) it holds",
                buf.len()
            )));
        }
        let stage = match r.read_u8()? {
            0 => Stage::Idle,
            1 => Stage::Cdb,
            2 => Stage::DataIn,
            3 => Stage::Identify,
            other => {
                return Err(Error::State(format!(
                    "{other} is not a packet protocol stage this drive has"
                )));
            }
        };
        let xfer = if r.read_bool()? {
            let left = r.read_u64()?;
            let taken = r.read_u64()?;
            let from = if r.read_bool()? {
                Origin::Disc(r.read_u64()?)
            } else {
                Origin::Built(r.read_bytes()?.to_vec())
            };
            if let Origin::Built(bytes) = &from
                && taken + left > bytes.len() as u64
            {
                return Err(Error::State(format!(
                    "a snapshot transfer of {left} byte(s) from {taken} does not fit the {} \
                     byte(s) of response it carries",
                    bytes.len()
                )));
            }
            Some(Xfer { left, taken, from })
        } else {
            None
        };
        let mut state = self.state.lock();
        state.selected = selected;
        state.device = device;
        state.error = error;
        state.status = status;
        state.control = control;
        state.features = features;
        state.reason = reason;
        state.byte_count = byte_count;
        state.lba_low = lba_low;
        state.irq = irq;
        state.in_reset = in_reset;
        state.sense = sense;
        state.attention = attention;
        state.locked = locked;
        state.buf = buf;
        state.pos = pos as usize;
        state.stage = stage;
        state.xfer = xfer;
        Ok(())
    }
}

impl AtaDevice for AtapiDrive {
    fn is_selected(&self) -> bool {
        self.state.lock().selected
    }

    fn write_reg(&self, reg: Reg, value: u16) {
        self.do_write_reg(reg, value);
    }

    fn read_reg(&self, reg: Reg, debug: bool) -> u16 {
        self.do_read_reg(reg, debug)
    }

    fn write_device_control(&self, value: u8) {
        self.do_write_device_control(value);
    }

    fn read_alt_status(&self) -> u8 {
        self.state.lock().status
    }

    fn irq_asserted(&self) -> bool {
        let state = self.state.lock();
        state.irq && state.control & CTL_NIEN == 0
    }

    fn power_on_reset(&self) {
        self.power_on();
    }

    fn as_disk(self: Arc<Self>) -> Option<Arc<AtaDisk>> {
        // A packet device is not a hard disk, and the two callers that ask this
        // question cannot drive one. Saying so is the point.
        None
    }
}

// ---------------------------------------------------------------------------
// Response builders
// ---------------------------------------------------------------------------

/// The 18-byte fixed-format sense data. SFF-8020i §9.4.
fn request_sense(sense: Sense) -> Vec<u8> {
    let mut out = alloc::vec![0u8; 18];
    // Response code 70h: a current error, in the fixed format. Bit 7 is VALID,
    // which says the information field holds a block address; it does not.
    out[0] = 0x70;
    out[2] = sense.key & 0x0f;
    // Additional sense length: the ten bytes after this one.
    out[7] = 10;
    out[12] = sense.asc;
    out[13] = sense.ascq;
    out
}

/// Mode page 01h, read error recovery parameters. Twelve bytes.
fn read_error_recovery_page() -> [u8; 12] {
    let mut page = [0u8; 12];
    page[0] = 0x01;
    page[1] = 0x0a;
    // Every recovery parameter is zero: this drive has no mechanism to retry
    // with, so the honest report is "no retries, no correction span".
    page
}

/// Mode page 0Dh, CD-ROM parameters. Eight bytes.
fn cd_parameters_page() -> [u8; 8] {
    let mut page = [0u8; 8];
    page[0] = 0x0d;
    page[1] = 0x06;
    // The two constants that make an MSF address mean something, and the reason
    // this page is the one a driver reads before it asks for a TOC in MSF.
    page[5] = 60; // seconds per minute
    page[7] = 75; // frames per second
    page
}

/// Mode page 2Ah, CD capabilities and mechanical status. MMC.
///
/// Twenty-two bytes: the MMC-1 form, which is the one a drive of this vintage
/// reports. Every capability this drive does not have reads as zero, which is
/// what "does not have" means here — there is no audio play, no CD-R, no
/// changer and no tray, and a page that claimed otherwise would invite a host
/// to ask for something that would then fail.
fn capabilities_page() -> [u8; 22] {
    let mut page = [0u8; 22];
    page[0] = 0x2a;
    page[1] = 0x14;
    // Byte 4 bit 0: the drive reads CD-DA — no. Bit 5, Mode 2 Form 1: yes,
    // because that is how an ISO 9660 image's 2048-byte blocks are carried.
    page[4] = 0x20;
    // Byte 6 bit 0: locking supported; bit 1: the lock's current state is
    // reported elsewhere; bit 3: the medium is ejectable by the host, which it
    // is not here.
    page[6] = 0x01;
    // Bytes 8-9 and 14-15: maximum and current read speed in kilobytes per
    // second. 176 KB/s is single speed and this drive has no speed at all —
    // every transfer completes inside the port write that asked for it — so
    // what it reports is the slowest thing that is not a lie about being
    // broken.
    page[8] = 0x00;
    page[9] = 0xb0;
    page[14] = 0x00;
    page[15] = 0xb0;
    page
}

/// One eight-byte TOC track descriptor.
fn track_descriptor(track: u8, lba: u64, msf: bool) -> [u8; 8] {
    let mut out = [0u8; 8];
    // ADR 1 (the address is a position), control 4 (a data track, no
    // pre-emphasis, digital copy prohibited).
    out[1] = 0x14;
    out[2] = track;
    if msf {
        let (m, s, f) = to_msf(lba);
        out[5] = m;
        out[6] = s;
        out[7] = f;
    } else {
        out[4..8].copy_from_slice(&(lba as u32).to_be_bytes());
    }
    out
}

/// Turn a logical block address into minutes, seconds and frames.
///
/// The Red Book's two-second lead-in is the 150 frames added here, and it is
/// why `READ TOC` in MSF reports 00:02:00 for a track that starts at block 0.
fn to_msf(lba: u64) -> (u8, u8, u8) {
    let total = lba + 150;
    let frame = total % 75;
    let seconds = (total / 75) % 60;
    let minutes = total / (75 * 60);
    (minutes as u8, seconds as u8, frame as u8)
}

/// Lay `text` into a SCSI ASCII field: space padded, not NUL terminated, and
/// not byte-swapped — which is the other half of why `IDENTIFY` and `INQUIRY`
/// strings cannot share a formatter.
fn ascii_field(dst: &mut [u8], text: &str) {
    for (slot, byte) in dst.iter_mut().zip(
        text.bytes()
            .map(|b| if b.is_ascii_graphic() { b } else { b' ' })
            .chain(core::iter::repeat(b' ')),
    ) {
        *slot = byte;
    }
}

fn be16(bytes: &[u8]) -> u64 {
    u64::from(u16::from_be_bytes([bytes[0], bytes[1]]))
}

fn be32(bytes: &[u8]) -> u32 {
    u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
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

/// The `ata.cdrom` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an ATAPI CD-ROM: the packet interface and the MMC command set a boot needs",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the disc; unbound or empty is a drive with no \
                      disc in it",
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
            name: "dma",
            kind: ValueKind::Bool,
            required: false,
            summary: "accept the Features register's DMA bit on a PACKET command (default \
                      false); a bus-mastering host adapter wants it",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "the IDENTIFY model string, also INQUIRY's product identification",
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
            summary: "the IDENTIFY firmware revision, also INQUIRY's product revision",
        },
        PropertySpec {
            name: "vendor",
            kind: ValueKind::Str,
            required: false,
            summary: "INQUIRY's eight-character vendor identification",
        },
    ],
    construct: |props| Ok(Box::new(CdromDevice::new(props)?)),
};

/// An [`AtapiDrive`] as a machine-description object.
///
/// The same wrapper `ata.disk` has and for the same reason: the drive is not a
/// memory-mapped device, so what this adds is the two-phase construction
/// contract and the bay rendezvous.
#[derive(Debug)]
pub struct CdromDevice {
    drive: Arc<AtapiDrive>,
    bay: String,
}

impl CdromDevice {
    /// Validate `props`, build the drive, and fit it.
    ///
    /// # Errors
    ///
    /// As [`AtapiDrive::new`], plus [`Error::Config`] if the named bay already
    /// holds something.
    pub fn new(props: &Props) -> Result<CdromDevice> {
        let bay = props
            .get("bay")
            .and_then(crate::core::props::Value::as_str)
            .unwrap_or(super::DEFAULT_BAY)
            .to_string();
        let drive = Arc::new(AtapiDrive::new(props)?);
        let holder = super::bays::attach(props, &bay)?;
        holder
            .fit_device(Arc::clone(&drive) as Arc<dyn AtaDevice>)
            .map_err(|_| {
                config(format!(
                    "two devices were fitted in the bay called `{bay}`; give one of them \
                     another `bay`"
                ))
            })?;
        Ok(CdromDevice { drive, bay })
    }

    /// The drive behind this object.
    #[must_use]
    pub fn drive(&self) -> &Arc<AtapiDrive> {
        &self.drive
    }

    /// The bay it was fitted in.
    #[must_use]
    pub fn bay(&self) -> &str {
        &self.bay
    }
}

impl Device for CdromDevice {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        self.drive.power_on();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.drive.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.drive.load(r)
    }
}

impl Instance for CdromDevice {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(CdromDevice::new(props)?)))
}

/// What the validator should know about `ata.cdrom`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("bay", ValueKind::Str))
        .prop(PropSchema::new("position", ValueKind::Str).values(&["master", "slave"]))
        .prop(PropSchema::new("dma", ValueKind::Bool))
        .prop(PropSchema::new("model", ValueKind::Str))
        .prop(PropSchema::new("serial", ValueKind::Str))
        .prop(PropSchema::new("firmware", ValueKind::Str))
        .prop(PropSchema::new("vendor", ValueKind::Str))
}

#[cfg(test)]
mod tests;
