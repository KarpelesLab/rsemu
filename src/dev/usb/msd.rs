//! A USB mass storage device: Bulk-Only Transport over a SCSI command set,
//! with a real [`Medium`] behind it.
//!
//! # Why this device and not another controller
//!
//! Everything else that hangs off [`crate::bus::usb`] is a *controller* or a
//! toy. This is the first USB device model whose answers come from somewhere —
//! the same [`Medium`] an ATA drive, an AHCI port and an NVMe namespace read
//! and write, so `--drive usb0=disk.qcow2` reaches a guest over USB for the
//! same reason and through the same seam it reaches one over SATA. That is the
//! whole point of it: USB stops being a bus with a mouse on it.
//!
//! It needs **no new host controller**. Bulk-Only Transport is, as its name
//! says, two bulk endpoints and the default pipe — and [`crate::dev::usb::ehci`]
//! already walks bulk queue heads, keeps data toggles, honours short packets
//! and turns a `STALL` into a halted qTD. So a guest driver reaches this device
//! through a controller that was finished before it existed, which is the test
//! of whether the fabric's transaction seam was the right shape.
//!
//! # The protocol, in three phases
//!
//! ```text
//!   host                                         device
//!   ────                                         ──────
//!   OUT ep2: CBW, exactly 31 bytes   ──────►   parse, dispatch to SCSI
//!   IN  ep1: data … (or OUT ep2)     ◄─────►   the medium, a packet at a time
//!   IN  ep1: CSW, exactly 13 bytes   ◄──────   tag, residue, status
//! ```
//!
//! That is BOT §5.1, §5.2 and §5.3. The device never buffers a transfer: a
//! `READ (10)` of 65,535 blocks moves **one packet at a time straight off the
//! medium**, so the memory a command costs is `wMaxPacketSize` and not what the
//! guest asked for. See *Bounding what the guest asked for*, below.
//!
//! # The thirteen cases are implemented, not approximated
//!
//! BOT §6.7 tabulates every combination of what the host said it wanted
//! (`dCBWDataTransferLength` and the direction bit) against what the device
//! turned out to intend, and says for each one what the residue is, whether the
//! status is *Command Passed*, *Command Failed* or *Phase Error*, and which
//! pipe gets stalled. Six of the thirteen are error paths a driver only reaches
//! when it has miscomputed something, and they are exactly the ones an emulator
//! is tempted to skip — so all thirteen live in **one** `match`, with the
//! specification's own case numbers written beside them, and `msd/tests.rs`
//! walks every one.
//!
//! # Two stalls that outlive a class reset
//!
//! BOT §3.1 is explicit that a *Bulk-Only Mass Storage Reset* "shall preserve
//! the value of its bulk data toggle bits and endpoint STALL conditions", and
//! §5.3.4 makes Reset Recovery the class reset **followed by** a
//! `CLEAR_FEATURE(ENDPOINT_HALT)` on each bulk pipe. Modelling that honestly is
//! why [`crate::bus::usb::Function`] grew [`Function::halt_cleared`]: this
//! device halts a pipe itself, as a protocol signal, and only the host's
//! `CLEAR_FEATURE` may let it go.
//!
//! # Bounding what the guest asked for
//!
//! `CLAUDE.md` and the fuzz target next door are about a bus master walking a
//! structure the guest built; this device masters nothing and walks nothing.
//! Its untrusted input is a **byte stream on a bulk endpoint**, and the same
//! discipline applies to it:
//!
//! * A CBW is one 31-byte packet or it is not a CBW (§5.1, §6.2.1), so there is
//!   no reassembly buffer to grow and no length field that decides an
//!   allocation.
//! * `dCBWDataTransferLength` is a `u32` the guest chose and **nothing is ever
//!   sized from it**. It is a counter that decrements.
//! * A `READ`/`WRITE` transfer length is likewise never an allocation: the
//!   payload is a *cursor* into the medium rather than a buffer, and the work one
//!   transaction can cause is one packet.
//! * Everything a command *does* buffer — `INQUIRY` data, sense data, a
//!   capacity — is bounded by [`MAX_PARAMETER_BYTES`], and every allocation
//!   length is clamped to it before it reaches a `Vec`.
//! * Every logical block address is checked in `u64` with
//!   [`u64::checked_add`] against the medium's block count, so a
//!   `0xffff_ffff_ffff_ffff` LBA is `LOGICAL BLOCK ADDRESS OUT OF RANGE` rather
//!   than a wrap into block zero.
//!
//! # Sources
//!
//! * **Universal Serial Bus Mass Storage Class, Bulk-Only Transport, Revision
//!   1.0** (usb.org, free download) — §3.1 the class reset, §3.2 `Get Max LUN`,
//!   §4.3-§4.4 the interface and endpoint descriptors, §5.1 the CBW, §5.2 the
//!   CSW, §5.3 the three phases and Reset Recovery, §6.2 a valid and meaningful
//!   CBW, §6.6 the error classes, §6.7 the thirteen cases.
//! * **Universal Serial Bus Mass Storage Class, Specification Overview,
//!   Revision 1.4** — `bInterfaceSubClass` 06h, the SCSI transparent command
//!   set, and `bInterfaceProtocol` 50h, Bulk-Only.
//! * **USB 2.0** §5.8 (bulk transfers, and that they do not exist at low
//!   speed), §9.4, §9.6.
//! * **Seagate SCSI Commands Reference Manual, 100293068 Rev. J, October
//!   2016** — a vendor document that reproduces the SPC-5 and SBC-4 command
//!   formats: §2.4 sense data and its table 27, §3.6 and table 59 `INQUIRY`,
//!   §3.11 `MODE SENSE (6)`, §3.16 and table 97 `READ (10)`, §3.22 and table
//!   120 `READ CAPACITY (10)`, §3.23 `READ CAPACITY (16)`, §3.37
//!   `REQUEST SENSE`, §3.49 `START STOP UNIT`, §3.51 `SYNCHRONIZE CACHE (10)`,
//!   §3.53 `TEST UNIT READY`, §3.60 `WRITE (10)`.
//!
//! No emulator source and no operating system's mass-storage or SCSI driver was
//! opened (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{
    Completion, ConfigurationDescriptor, Descriptors, DeviceDescriptor, Direction,
    EndpointDescriptor, Function, InterfaceDescriptor, Peripheral, Recipient, RequestKind,
    SetupPacket, Speed, TransferType, UsbDevice, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::medium::{self as medium, Medium, Snapshot};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.storage";

/// The media slot a machine description binds when it names none.
const DEFAULT_SLOT: &str = "usb0";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// What the descriptors say (BOT §4)
// ---------------------------------------------------------------------------

/// `bInterfaceClass` 08h: mass storage (BOT §4.3, table 4.5).
pub const CLASS_MASS_STORAGE: u8 = 0x08;

/// `bInterfaceSubClass` 06h: the SCSI transparent command set.
///
/// From the *Mass Storage Class Specification Overview*, which is the document
/// BOT §4.3 defers the subclass codes to. 06h means "the CBWCB is a SCSI
/// command descriptor block and nothing has been reduced out of it", which is
/// what every USB disk made since about 2001 reports.
pub const SUBCLASS_SCSI: u8 = 0x06;

/// `bInterfaceProtocol` 50h: Bulk-Only Transport (BOT §4.3, table 4.5).
pub const PROTOCOL_BULK_ONLY: u8 = 0x50;

/// The bulk IN endpoint's number. `bEndpointAddress` is this with bit 7 set.
pub const ENDPOINT_IN: u8 = 1;
/// The bulk OUT endpoint's number.
pub const ENDPOINT_OUT: u8 = 2;

/// The interface number the class requests are addressed to (BOT §3.1, §3.2).
const INTERFACE: u16 = 0;

/// The class-specific requests of BOT §3.
pub mod class_request {
    /// §3.1. `bmRequestType` 00100001b, `wLength` 0. Readies the device for the
    /// next CBW and **preserves the endpoint stall conditions**.
    pub const BULK_ONLY_RESET: u8 = 0xff;
    /// §3.2. `bmRequestType` 10100001b, `wLength` 1. Returns the highest LUN.
    pub const GET_MAX_LUN: u8 = 0xfe;
}

// ---------------------------------------------------------------------------
// The wrappers (BOT §5.1, §5.2)
// ---------------------------------------------------------------------------

/// How many bytes a Command Block Wrapper is. Exactly this, or it is not one
/// (BOT §5.1, §6.2.1).
pub const CBW_BYTES: usize = 31;

/// How many bytes a Command Status Wrapper is (BOT §5.2).
pub const CSW_BYTES: usize = 13;

/// `dCBWSignature`, little-endian, spelling `USBC` (BOT §5.1).
pub const CBW_SIGNATURE: u32 = 0x4342_5355;

/// `dCSWSignature`, spelling `USBS` (BOT §5.2).
pub const CSW_SIGNATURE: u32 = 0x5342_5355;

/// `bmCBWFlags` bit 7: set means Data-In, device to host (BOT §5.1).
const CBW_FLAG_IN: u8 = 0x80;

/// The `bCSWStatus` values of BOT §5.2, table 5.3.
pub mod status {
    /// The command completed.
    pub const PASSED: u8 = 0x00;
    /// The command was understood and failed; sense data says why.
    pub const FAILED: u8 = 0x01;
    /// Host and device disagree about the data phase. The host must perform a
    /// Reset Recovery (§5.3.3.1).
    pub const PHASE_ERROR: u8 = 0x02;
}

/// The largest command block a CBW can carry (BOT §5.1: `CBWCB` is 16 bytes).
pub const CDB_BYTES: usize = 16;

/// A decoded Command Block Wrapper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cbw {
    /// `dCBWTag`, echoed into the CSW and used for nothing else (§6.2).
    pub tag: u32,
    /// `dCBWDataTransferLength`: how many bytes the host expects to move.
    pub data_length: u32,
    /// `bmCBWFlags`.
    pub flags: u8,
    /// `bCBWLUN`, four bits.
    pub lun: u8,
    /// `bCBWCBLength`, 1 to 16.
    pub cb_length: u8,
    /// `CBWCB`, all sixteen bytes; only the first `cb_length` are the command.
    pub cb: [u8; CDB_BYTES],
}

impl Cbw {
    /// Decode the thirty-one bytes, little-endian (§5.1).
    #[must_use]
    pub fn decode(bytes: &[u8; CBW_BYTES]) -> Cbw {
        let mut cb = [0u8; CDB_BYTES];
        cb.copy_from_slice(&bytes[15..31]);
        Cbw {
            tag: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            data_length: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            flags: bytes[12],
            lun: bytes[13],
            cb_length: bytes[14],
            cb,
        }
    }

    /// Which way the host expects the data phase to go.
    ///
    /// `None` when `dCBWDataTransferLength` is zero, because §5.1 says the
    /// device shall then *ignore* the direction bit — a rule worth encoding in
    /// the type rather than remembering at each use.
    #[must_use]
    pub fn direction(&self) -> Option<Direction> {
        if self.data_length == 0 {
            None
        } else if self.flags & CBW_FLAG_IN != 0 {
            Some(Direction::In)
        } else {
            Some(Direction::Out)
        }
    }

    /// The command block, as long as `bCBWCBLength` says it is.
    #[must_use]
    pub fn command(&self) -> &[u8] {
        &self.cb[..usize::from(self.cb_length).min(CDB_BYTES)]
    }
}

/// Encode a Command Status Wrapper (§5.2).
#[must_use]
fn encode_csw(tag: u32, residue: u32, status: u8) -> [u8; CSW_BYTES] {
    let signature = CSW_SIGNATURE.to_le_bytes();
    let tag = tag.to_le_bytes();
    let residue = residue.to_le_bytes();
    [
        signature[0],
        signature[1],
        signature[2],
        signature[3],
        tag[0],
        tag[1],
        tag[2],
        tag[3],
        residue[0],
        residue[1],
        residue[2],
        residue[3],
        status,
    ]
}

// ---------------------------------------------------------------------------
// The SCSI command set
// ---------------------------------------------------------------------------

/// The operation codes this device answers, with the section of the Seagate
/// *SCSI Commands Reference Manual* (Rev. J) that defines each.
pub mod opcode {
    /// §3.53.
    pub const TEST_UNIT_READY: u8 = 0x00;
    /// §3.37.
    pub const REQUEST_SENSE: u8 = 0x03;
    /// §3.15.
    pub const READ_6: u8 = 0x08;
    /// §3.59.
    pub const WRITE_6: u8 = 0x0a;
    /// §3.6.
    pub const INQUIRY: u8 = 0x12;
    /// §3.11.
    pub const MODE_SENSE_6: u8 = 0x1a;
    /// §3.49.
    pub const START_STOP_UNIT: u8 = 0x1b;
    /// SPC-5. Accepted and ignored: nothing here has a door to lock.
    pub const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1e;
    /// §3.22.
    pub const READ_CAPACITY_10: u8 = 0x25;
    /// §3.16.
    pub const READ_10: u8 = 0x28;
    /// §3.60.
    pub const WRITE_10: u8 = 0x2a;
    /// §3.55. `BYTCHK` zero, so it is a bounds check and nothing else.
    pub const VERIFY_10: u8 = 0x2f;
    /// §3.51.
    pub const SYNCHRONIZE_CACHE_10: u8 = 0x35;
    /// §3.12.
    pub const MODE_SENSE_10: u8 = 0x5a;
    /// §3.18.
    pub const READ_16: u8 = 0x88;
    /// §3.62.
    pub const WRITE_16: u8 = 0x8a;
    /// §3.52.
    pub const SYNCHRONIZE_CACHE_16: u8 = 0x91;
    /// §3.23, reached through `SERVICE ACTION IN (16)` with service action 10h.
    pub const SERVICE_ACTION_IN_16: u8 = 0x9e;
    /// §3.33.
    pub const REPORT_LUNS: u8 = 0xa0;
    /// §3.17.
    pub const READ_12: u8 = 0xa8;
    /// §3.61.
    pub const WRITE_12: u8 = 0xaa;
}

/// The service action of `SERVICE ACTION IN (16)` that is `READ CAPACITY (16)`.
const SERVICE_ACTION_READ_CAPACITY_16: u8 = 0x10;

/// The sense keys of the Seagate manual's §2.4 (SPC-5 table 28).
pub mod sense_key {
    /// No specific sense information.
    pub const NO_SENSE: u8 = 0x00;
    /// The medium is there and the bytes could not be moved.
    pub const MEDIUM_ERROR: u8 = 0x03;
    /// A non-recoverable hardware failure.
    pub const HARDWARE_ERROR: u8 = 0x04;
    /// The command block was wrong.
    pub const ILLEGAL_REQUEST: u8 = 0x05;
    /// The medium refuses writes.
    pub const DATA_PROTECT: u8 = 0x07;
}

/// The additional sense codes this device reports, as `(ASC, ASCQ)`.
pub mod asc {
    /// An unrecovered read error.
    pub const UNRECOVERED_READ_ERROR: (u8, u8) = (0x11, 0x00);
    /// A write fault.
    pub const WRITE_FAULT: (u8, u8) = (0x03, 0x00);
    /// The operation code is not one this device implements.
    pub const INVALID_COMMAND: (u8, u8) = (0x20, 0x00);
    /// The logical block address is past the end of the medium.
    pub const LBA_OUT_OF_RANGE: (u8, u8) = (0x21, 0x00);
    /// A field of the command block holds a value this device rejects.
    pub const INVALID_FIELD_IN_CDB: (u8, u8) = (0x24, 0x00);
    /// The LUN is not one this device has.
    pub const LUN_NOT_SUPPORTED: (u8, u8) = (0x25, 0x00);
    /// The medium is write protected.
    pub const WRITE_PROTECTED: (u8, u8) = (0x27, 0x00);
}

/// How many bytes of fixed-format sense data `REQUEST SENSE` returns.
///
/// Eighteen: the eight bytes before `ADDITIONAL SENSE LENGTH` plus the ten that
/// field then declares (Seagate §2.4, table 27).
pub const SENSE_BYTES: usize = 18;

/// The largest parameter response this device will ever build.
///
/// Every buffered payload — `INQUIRY` data, sense data, a capacity, a mode
/// page, a LUN report — fits far inside this; the constant exists so that an
/// allocation length a *guest* chose is clamped before it reaches an allocator,
/// which is the same rule the bus masters in this tree apply to a length field
/// in guest memory.
pub const MAX_PARAMETER_BYTES: u64 = 4096;

/// The fixed-format sense data a `REQUEST SENSE` returns (Seagate §2.4, table
/// 27, response code 70h — a current error).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Sense {
    key: u8,
    asc: u8,
    ascq: u8,
}

impl Sense {
    const fn new(key: u8, code: (u8, u8)) -> Sense {
        Sense {
            key,
            asc: code.0,
            ascq: code.1,
        }
    }

    /// The eighteen bytes of table 27.
    fn encode(self) -> [u8; SENSE_BYTES] {
        let mut out = [0u8; SENSE_BYTES];
        // Response code 70h, current error; `VALID` clear, because this device
        // never has an `INFORMATION` field worth reporting.
        out[0] = 0x70;
        out[2] = self.key & 0x0f;
        // `ADDITIONAL SENSE LENGTH (n - 7)`: ten more bytes follow.
        out[7] = (SENSE_BYTES - 8) as u8;
        out[12] = self.asc;
        out[13] = self.ascq;
        out
    }
}

// ---------------------------------------------------------------------------
// The transfer in flight
// ---------------------------------------------------------------------------

/// Which of BOT §5.3's three phases the device is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// Waiting for a CBW on the bulk-out endpoint (§5.3.1).
    #[default]
    Command,
    /// Sending the data phase on the bulk-in endpoint (§5.3.2).
    DataIn,
    /// Taking the data phase on the bulk-out endpoint (§5.3.2).
    DataOut,
    /// The CSW is what the next bulk-in gets (§5.3.3).
    Status,
}

impl Phase {
    const fn code(self) -> u8 {
        match self {
            Phase::Command => 0,
            Phase::DataIn => 1,
            Phase::DataOut => 2,
            Phase::Status => 3,
        }
    }

    const fn from_code(code: u8) -> Phase {
        match code {
            1 => Phase::DataIn,
            2 => Phase::DataOut,
            3 => Phase::Status,
            _ => Phase::Command,
        }
    }
}

/// Where the data phase's bytes come from, or go.
///
/// **Never the whole transfer.** A `READ (10)` for 65,535 blocks is
/// [`Payload::Blocks`] — a cursor — so the command costs one packet of memory
/// however many blocks the guest asked for. The only variant that allocates is
/// [`Payload::Buffer`], and everything that produces one is a parameter
/// response bounded by [`MAX_PARAMETER_BYTES`].
#[derive(Debug, Clone, PartialEq, Eq)]
enum Payload {
    /// The command moves no data of its own.
    None,
    /// Bytes the command produced, and how many of them have gone.
    Buffer { bytes: Vec<u8>, offset: u64 },
    /// A byte offset into the medium, walked a packet at a time.
    Blocks { offset: u64 },
}

impl Payload {
    const fn code(&self) -> u8 {
        match self {
            Payload::None => 0,
            Payload::Buffer { .. } => 1,
            Payload::Blocks { .. } => 2,
        }
    }
}

/// Everything the device remembers between transactions.
#[derive(Debug, Clone, PartialEq, Eq)]
struct MsdState {
    phase: Phase,
    /// `dCBWTag` of the command in flight, echoed into the CSW (§5.2).
    tag: u32,
    /// `dCBWDataTransferLength` of the command in flight.
    host_length: u32,
    /// How many of those bytes the host has still to move, in either
    /// direction. Counts fill and discarded bytes as well as relevant ones,
    /// because it is the host's expectation and not the device's intent.
    host_remaining: u64,
    /// How many bytes of the device's own intent are still to move.
    device_remaining: u64,
    /// How many bytes were *relevant* — the number `dCSWDataResidue` is
    /// computed from (§5.2).
    moved: u64,
    /// The `bCSWStatus` the CSW will carry.
    status: u8,
    payload: Payload,
    /// How much of the thirteen-byte CSW has gone out.
    csw_sent: u8,
    /// The bulk-in pipe is halted by the *device*, as a protocol signal
    /// (§5.3.2, §6.7). Only a `CLEAR_FEATURE` clears it (§3.1).
    stall_in: bool,
    /// The bulk-out pipe likewise.
    stall_out: bool,
    /// §6.6.1: a CBW that was not valid wedges the device until a full Reset
    /// Recovery, which *starts* with the class reset. Separate from the two
    /// stall latches on purpose — clearing the halts alone must not resume it.
    wedged: bool,
    /// What the next `REQUEST SENSE` reports.
    sense: Sense,
}

impl Default for MsdState {
    fn default() -> MsdState {
        MsdState {
            phase: Phase::Command,
            tag: 0,
            host_length: 0,
            host_remaining: 0,
            device_remaining: 0,
            moved: 0,
            status: status::PASSED,
            payload: Payload::None,
            csw_sent: 0,
            stall_in: false,
            stall_out: false,
            wedged: false,
            sense: Sense::default(),
        }
    }
}

/// What one SCSI command turned out to intend.
///
/// The device half of BOT §6.7's matrix: `Dn`, `Di` or `Do`, and how much.
#[derive(Debug)]
struct Intent {
    /// `None` is `Dn`.
    direction: Option<Direction>,
    /// How many bytes. Zero exactly when `direction` is `None`.
    length: u64,
    payload: Payload,
    /// `bCSWStatus` if the data phase completes: passed or failed.
    status: u8,
}

impl Intent {
    /// `Dn`, and the command succeeded.
    const fn none() -> Intent {
        Intent {
            direction: None,
            length: 0,
            payload: Payload::None,
            status: status::PASSED,
        }
    }

    /// `Dn`, and the command failed; the caller has already set the sense.
    const fn failed() -> Intent {
        Intent {
            direction: None,
            length: 0,
            payload: Payload::None,
            status: status::FAILED,
        }
    }

    /// `Di`, from a buffer the command produced.
    fn parameter(bytes: Vec<u8>, allocation: u64) -> Intent {
        let mut bytes = bytes;
        // SCSI's allocation length is the host's cap on a parameter response
        // and the device transfers the smaller of the two (Seagate §2.2.6).
        // Clamping here is also what keeps `MAX_PARAMETER_BYTES` meaningful.
        let take = allocation.min(MAX_PARAMETER_BYTES).min(bytes.len() as u64);
        bytes.truncate(take as usize);
        if bytes.is_empty() {
            // An allocation length of zero is legal and means "send nothing"
            // (Seagate SCSI Commands Reference Manual §2.2.6), so the device
            // intends `Dn` and BOT §6.7's case (1) applies rather than case
            // (2). Getting this wrong turns a legal probe into a phase error.
            return Intent::none();
        }
        Intent {
            direction: Some(Direction::In),
            length: bytes.len() as u64,
            payload: Payload::Buffer { bytes, offset: 0 },
            status: status::PASSED,
        }
    }

    /// `Di` or `Do`, straight off the medium.
    const fn blocks(direction: Direction, offset: u64, length: u64) -> Intent {
        Intent {
            direction: Some(direction),
            length,
            payload: Payload::Blocks { offset },
            status: status::PASSED,
        }
    }
}

// ---------------------------------------------------------------------------
// The function
// ---------------------------------------------------------------------------

/// The class-specific half of the device: descriptors, the two class requests,
/// the BOT state machine and the SCSI command set.
struct StorageFunction {
    descriptors: Descriptors,
    speed: Speed,
    /// `wMaxPacketSize` of both bulk endpoints, which BOT §4.4 leaves to the
    /// speed and USB 2.0 §5.8.3 then fixes at 512 for high speed.
    max_packet: u16,
    /// The bytes.
    media: Arc<dyn Medium>,
    /// Bytes in a logical block.
    block: u64,
    /// How many of them there are.
    blocks: u64,
    /// Whether this drive refuses writes, whatever the medium would allow.
    read_only: bool,
    /// The `RMB` bit of the standard `INQUIRY` data (Seagate §3.6.2, table 59).
    removable: bool,
    /// `T10 VENDOR IDENTIFICATION`, eight bytes, space padded.
    vendor_id: [u8; 8],
    /// `PRODUCT IDENTIFICATION`, sixteen bytes.
    product_id: [u8; 16],
    /// `PRODUCT REVISION LEVEL`, four bytes.
    revision: [u8; 4],
    /// The serial number, for `INQUIRY` page 80h and the string descriptor.
    serial: String,
    state: Mutex<MsdState>,
}

impl fmt::Debug for StorageFunction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("StorageFunction");
        s.field("blocks", &self.blocks);
        s.field("block", &self.block);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// Pad `text` into a fixed-width, space-filled SCSI identification field.
///
/// Space padding rather than NUL: Seagate §3.6.2 and SPC-5 make these fields
/// ASCII left-aligned and space filled, and a host that prints them without
/// trimming shows the difference.
fn pad<const N: usize>(text: &str) -> [u8; N] {
    let mut out = [b' '; N];
    for (slot, byte) in out.iter_mut().zip(text.bytes()) {
        // Anything outside printable ASCII would make the field unprintable,
        // and a machine file is allowed to be wrong about that.
        *slot = if (0x20..0x7f).contains(&byte) {
            byte
        } else {
            b'?'
        };
    }
    out
}

impl StorageFunction {
    fn new(
        media: Arc<dyn Medium>,
        block: u64,
        read_only: bool,
        removable: bool,
        speed: Speed,
        ids: (u16, u16),
        strings: (&str, &str, &str, &str),
    ) -> StorageFunction {
        let (vendor, product) = ids;
        let (vendor_id, product_id, revision, serial) = strings;
        // Bulk endpoints are 512 bytes at high speed and 64 at full speed
        // (USB 2.0 §5.8.3). Low speed has no bulk endpoints at all, which
        // `UsbStorage::new` refuses before this is reached.
        let max_packet: u16 = if speed == Speed::High { 512 } else { 64 };
        let blocks = media.capacity() / block;

        let device = DeviceDescriptor {
            usb: 0x0200,
            // BOT §4.1: "The device shall specify the device class and subclass
            // codes in the interface descriptor, and not in the device
            // descriptor."
            class: 0,
            subclass: 0,
            protocol: 0,
            max_packet0: speed.max_control_packet() as u8,
            vendor,
            product,
            device: 0x0100,
            manufacturer: 0,
            product_name: 0,
            serial: 0,
            configurations: 1,
        };

        let interface = InterfaceDescriptor {
            number: INTERFACE as u8,
            alternate: 0,
            endpoints: 2,
            class: CLASS_MASS_STORAGE,
            subclass: SUBCLASS_SCSI,
            protocol: PROTOCOL_BULK_ONLY,
            name: 0,
        };
        let bulk_in = EndpointDescriptor {
            address: ENDPOINT_IN | Direction::BIT,
            attributes: TransferType::Bulk.attribute_bits(),
            max_packet,
            // §4.4.1: "Does not apply to Bulk endpoints."
            interval: 0,
        };
        let bulk_out = EndpointDescriptor {
            address: ENDPOINT_OUT,
            attributes: TransferType::Bulk.attribute_bits(),
            max_packet,
            interval: 0,
        };

        let mut body = Vec::new();
        body.extend_from_slice(&interface.encode());
        body.extend_from_slice(&bulk_in.encode());
        body.extend_from_slice(&bulk_out.encode());

        let mut device = device;
        let mut descriptors = Descriptors::new();
        descriptors.add_configuration(
            &ConfigurationDescriptor {
                interfaces: 1,
                value: 1,
                name: 0,
                attributes: 0,
                // 200 mA in the 2 mA units the field counts in, which is what a
                // bus-powered disk asks for.
                max_power: 100,
            },
            &body,
        );
        // BOT §4 requires a serial number string, and §4.1.1 requires the
        // device descriptor to index it. The index is rebuilt here rather than
        // guessed: `add_string` creates the language list first and returns
        // what a descriptor should refer to.
        device.serial = descriptors.add_string(serial);
        descriptors.set_device(&device);
        if speed == Speed::High {
            descriptors.set_qualifier(&device, 0);
        }

        StorageFunction {
            descriptors,
            speed,
            max_packet,
            media,
            block,
            blocks,
            read_only,
            removable,
            vendor_id: pad(vendor_id),
            product_id: pad(product_id),
            revision: pad(revision),
            serial: serial.to_string(),
            state: Mutex::with_rank(LockRank::DEVICE, MsdState::default()),
        }
    }

    /// Whether this drive refuses writes — its own flag, or the medium's.
    fn write_protected(&self) -> bool {
        self.read_only || self.media.is_read_only()
    }

    // -- the BOT state machine ----------------------------------------------

    /// A CBW arrived. Validate it (§6.2.1), run the command, and settle the
    /// thirteen cases (§6.7).
    fn begin(&self, state: &mut MsdState, packet: &[u8]) -> Completion {
        // §6.2.1: a valid CBW was received after a CSW or a reset, is exactly
        // thirty-one bytes, and carries the signature. The length test is also
        // what makes reassembly unnecessary — see the module docs.
        if packet.len() != CBW_BYTES {
            return self.wedge(state);
        }
        let mut bytes = [0u8; CBW_BYTES];
        bytes.copy_from_slice(packet);
        if u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) != CBW_SIGNATURE {
            return self.wedge(state);
        }
        let cbw = Cbw::decode(&bytes);

        state.tag = cbw.tag;
        state.host_length = cbw.data_length;
        state.host_remaining = u64::from(cbw.data_length);
        state.moved = 0;
        state.csw_sent = 0;

        // §6.2.2: a *meaningful* CBW has no reserved bit set (`bmCBWFlags`
        // 6:0 and `bCBWLUN` 7:4, which §5.1 says the host sets to zero), names
        // a LUN this device has, and carries a command block length the command
        // set allows. §6.4 leaves the response to a CBW that is valid but *not*
        // meaningful undefined; failing the command with sense data is the
        // useful answer, because a driver can see it — a stall would leave it
        // guessing which of the four error classes it had hit.
        let reserved_flags = cbw.flags & !CBW_FLAG_IN != 0;
        let bad_lun = cbw.lun != 0;
        let bad_length = cbw.cb_length == 0 || usize::from(cbw.cb_length) > CDB_BYTES;
        let intent = if reserved_flags || bad_length {
            state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD_IN_CDB);
            Intent::failed()
        } else if bad_lun {
            state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::LUN_NOT_SUPPORTED);
            Intent::failed()
        } else {
            self.execute(state, cbw.command())
        };

        self.settle(state, cbw.direction(), intent);
        Completion::ack(packet.len() as u64)
    }

    /// §6.6.1: the CBW was not valid, so both pipes stall and stay stalled
    /// until a Reset Recovery.
    fn wedge(&self, state: &mut MsdState) -> Completion {
        state.wedged = true;
        state.stall_in = true;
        state.stall_out = true;
        state.phase = Phase::Command;
        state.payload = Payload::None;
        state.device_remaining = 0;
        state.host_remaining = 0;
        Completion::stall()
    }

    /// The thirteen cases of BOT §6.7, once, with the specification's own case
    /// numbers.
    ///
    /// `host` is `Hn`/`Hi`/`Ho` and `intent` is `Dn`/`Di`/`Do`; between them
    /// they decide the phase the device enters, the residue the CSW carries,
    /// the status it carries, and which pipe — if any — is halted.
    fn settle(&self, state: &mut MsdState, host: Option<Direction>, intent: Intent) {
        state.status = intent.status;
        state.device_remaining = intent.length;
        state.payload = intent.payload;

        match (host, intent.direction) {
            // (1) Hn = Dn. The overwhelmingly common no-data command.
            (None, None) => self.to_status(state),
            // (2) Hn < Di and (3) Hn < Do. The host left no room at all for a
            // data phase the device needs, and cannot be told how much it
            // missed: §6.7.1 has it ignore the residue and reset.
            (None, Some(_)) => {
                state.device_remaining = 0;
                state.payload = Payload::None;
                state.status = status::PHASE_ERROR;
                self.to_status(state);
            }
            // (4) Hi > Dn. The host is waiting for bytes there are none of.
            // §6.7.2 lets the device pad or stall; stalling is the honest one,
            // because padding would hand a driver bytes that mean nothing.
            (Some(Direction::In), None) => {
                state.stall_in = true;
                self.to_status(state);
            }
            // (5) Hi > Di and (6) Hi = Di. The thin diagonal, plus the short
            // read every `INQUIRY` with a generous allocation length produces.
            (Some(Direction::In), Some(Direction::In))
                if intent.length <= u64::from(state.host_length) =>
            {
                state.phase = Phase::DataIn;
            }
            // (7) Hi < Di. The device has more to say than the host will hear.
            // §6.7.2: transfer less than indicated, stall the bulk-in pipe, and
            // report a phase error.
            (Some(Direction::In), Some(Direction::In)) => {
                state.device_remaining = 0;
                state.payload = Payload::None;
                state.status = status::PHASE_ERROR;
                state.stall_in = true;
                self.to_status(state);
            }
            // (8) Hi <> Do. The host is listening and the device wants to be
            // told. Same treatment as (7).
            (Some(Direction::In), Some(Direction::Out)) => {
                state.device_remaining = 0;
                state.payload = Payload::None;
                state.status = status::PHASE_ERROR;
                state.stall_in = true;
                self.to_status(state);
            }
            // (9) Ho > Dn, (11) Ho > Do and (12) Ho = Do. §6.7.3 allows the
            // device to *accept a total of* `dCBWDataTransferLength` rather
            // than stall, and accepting is what this device does: there is no
            // desynchronisation risk on an OUT pipe, and a driver that
            // over-sends gets its residue instead of a halt to clear.
            (Some(Direction::Out), None) => state.phase = Phase::DataOut,
            (Some(Direction::Out), Some(Direction::Out))
                if intent.length <= u64::from(state.host_length) =>
            {
                state.phase = Phase::DataOut;
            }
            // (10) Ho <> Di and (13) Ho < Do. The device cannot be satisfied by
            // what the host is sending, so the bytes are drained and the status
            // is a phase error (§6.7.3).
            (Some(Direction::Out), Some(_)) => {
                state.device_remaining = 0;
                state.payload = Payload::None;
                state.status = status::PHASE_ERROR;
                state.phase = Phase::DataOut;
            }
        }
    }

    /// Move to the status phase, dropping whatever is left of the data phase.
    fn to_status(&self, state: &mut MsdState) {
        state.phase = Phase::Status;
        state.device_remaining = 0;
        state.payload = Payload::None;
        state.csw_sent = 0;
    }

    /// `dCSWDataResidue` (§5.2): what the host expected less what was relevant.
    fn residue(state: &MsdState) -> u32 {
        u64::from(state.host_length).saturating_sub(state.moved) as u32
    }

    /// The medium refused mid-transfer. §5.3.2 lets the device end the command
    /// by stalling the pipe in use; the sense says which failure it was.
    fn fail_transfer(&self, state: &mut MsdState, error: BusError, writing: bool) {
        state.sense = match error {
            BusError::Protected => Sense::new(sense_key::DATA_PROTECT, asc::WRITE_PROTECTED),
            BusError::BadAccess => Sense::new(sense_key::ILLEGAL_REQUEST, asc::LBA_OUT_OF_RANGE),
            _ if writing => Sense::new(sense_key::MEDIUM_ERROR, asc::WRITE_FAULT),
            _ => Sense::new(sense_key::MEDIUM_ERROR, asc::UNRECOVERED_READ_ERROR),
        };
        state.status = status::FAILED;
        if writing {
            state.stall_out = true;
        } else {
            state.stall_in = true;
        }
        self.to_status(state);
    }

    // -- the command set ----------------------------------------------------

    /// Run one command block and say what it intends.
    fn execute(&self, state: &mut MsdState, cdb: &[u8]) -> Intent {
        // A zero-length command block was rejected as not meaningful before
        // this was reached, so `cdb[0]` exists; the guard keeps that true
        // whatever a later caller does.
        let Some(&op) = cdb.first() else {
            state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD_IN_CDB);
            return Intent::failed();
        };
        // SPC: sense data is preserved until `REQUEST SENSE` reads it or
        // another command arrives, so every other command clears it here —
        // once, rather than in each arm that could forget.
        if op != opcode::REQUEST_SENSE {
            state.sense = Sense::default();
        }
        match op {
            opcode::TEST_UNIT_READY => Intent::none(),
            opcode::REQUEST_SENSE => {
                // §3.37: reading the sense data clears it. The allocation
                // length is one byte, at offset 4.
                let allocation = u64::from(cdb.get(4).copied().unwrap_or(0));
                let bytes = state.sense.encode().to_vec();
                state.sense = Sense::default();
                Intent::parameter(bytes, allocation)
            }
            opcode::INQUIRY => self.inquiry(state, cdb),
            opcode::MODE_SENSE_6 => {
                let allocation = u64::from(cdb.get(4).copied().unwrap_or(0));
                Intent::parameter(self.mode_parameter_header_6(), allocation)
            }
            opcode::MODE_SENSE_10 => {
                let allocation = u64::from(be16(cdb, 7));
                Intent::parameter(self.mode_parameter_header_10(), allocation)
            }
            opcode::START_STOP_UNIT | opcode::PREVENT_ALLOW_MEDIUM_REMOVAL => Intent::none(),
            opcode::READ_CAPACITY_10 => {
                let mut out = Vec::with_capacity(8);
                // §3.22.2, table 120: the *last* logical block address, so an
                // empty medium is not representable and a full 32-bit one
                // saturates at 0xffffffff to say "ask READ CAPACITY (16)".
                let last = self.blocks.saturating_sub(1).min(u64::from(u32::MAX));
                out.extend_from_slice(&(last as u32).to_be_bytes());
                out.extend_from_slice(&(self.block as u32).to_be_bytes());
                Intent::parameter(out, 8)
            }
            opcode::SERVICE_ACTION_IN_16
                if cdb.get(1).copied().unwrap_or(0) & 0x1f == SERVICE_ACTION_READ_CAPACITY_16 =>
            {
                // §3.23: thirty-two bytes, of which this device fills the first
                // twelve and leaves the protection and thin-provisioning fields
                // zero, which is what a drive with neither reports.
                let mut out = alloc::vec![0u8; 32];
                let last = self.blocks.saturating_sub(1);
                out[..8].copy_from_slice(&last.to_be_bytes());
                out[8..12].copy_from_slice(&(self.block as u32).to_be_bytes());
                let allocation = u64::from(be32(cdb, 10));
                Intent::parameter(out, allocation)
            }
            opcode::REPORT_LUNS => {
                // §3.33: an eight-byte header whose first field is the byte
                // length of the list, then one eight-byte entry per LUN. This
                // device has exactly LUN 0.
                let mut out = alloc::vec![0u8; 16];
                out[..4].copy_from_slice(&8u32.to_be_bytes());
                let allocation = u64::from(be32(cdb, 6));
                Intent::parameter(out, allocation)
            }
            opcode::READ_6 | opcode::WRITE_6 => {
                // §3.15, §3.59: a 21-bit address and an eight-bit length in
                // which zero means 256 blocks.
                let lba = u64::from(be24(cdb, 1) & 0x001f_ffff);
                let count = match cdb.get(4).copied().unwrap_or(0) {
                    0 => 256,
                    n => u64::from(n),
                };
                self.transfer(state, op == opcode::READ_6, lba, count)
            }
            opcode::READ_10 | opcode::WRITE_10 | opcode::VERIFY_10 => {
                let lba = u64::from(be32(cdb, 2));
                let count = u64::from(be16(cdb, 7));
                if op == opcode::VERIFY_10 {
                    // §3.55 with `BYTCHK` clear: verify that the blocks can be
                    // read, which for a modelled medium is a bounds check.
                    return match self.bounds(state, lba, count) {
                        Some(_) => Intent::none(),
                        None => Intent::failed(),
                    };
                }
                self.transfer(state, op == opcode::READ_10, lba, count)
            }
            opcode::READ_12 | opcode::WRITE_12 => {
                let lba = u64::from(be32(cdb, 2));
                let count = u64::from(be32(cdb, 6));
                self.transfer(state, op == opcode::READ_12, lba, count)
            }
            opcode::READ_16 | opcode::WRITE_16 => {
                let lba = be64(cdb, 2);
                let count = u64::from(be32(cdb, 10));
                self.transfer(state, op == opcode::READ_16, lba, count)
            }
            opcode::SYNCHRONIZE_CACHE_10 | opcode::SYNCHRONIZE_CACHE_16 => {
                match self.media.flush() {
                    Ok(()) => Intent::none(),
                    Err(_) => {
                        state.sense = Sense::new(sense_key::MEDIUM_ERROR, asc::WRITE_FAULT);
                        Intent::failed()
                    }
                }
            }
            _ => {
                state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_COMMAND);
                Intent::failed()
            }
        }
    }

    /// The standard `INQUIRY` data of Seagate §3.6.2, table 59, and the two
    /// vital product data pages a host asks for.
    fn inquiry(&self, state: &mut MsdState, cdb: &[u8]) -> Intent {
        let evpd = cdb.get(1).copied().unwrap_or(0) & 0x01 != 0;
        let page = cdb.get(2).copied().unwrap_or(0);
        let allocation = u64::from(be16(cdb, 3));
        if evpd {
            let bytes = match page {
                // Page 00h: the supported pages, in ascending order.
                0x00 => alloc::vec![0x00, 0x00, 0x00, 0x02, 0x00, 0x80],
                // Page 80h: the unit serial number.
                0x80 => {
                    let serial = self.serial.as_bytes();
                    let len = serial.len().min(usize::from(u8::MAX));
                    let mut out = alloc::vec![0x00, 0x80, 0x00, len as u8];
                    out.extend_from_slice(&serial[..len]);
                    out
                }
                _ => {
                    state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD_IN_CDB);
                    return Intent::failed();
                }
            };
            return Intent::parameter(bytes, allocation);
        }
        if page != 0 {
            // §3.6: with `EVPD` clear the `PAGE CODE` field shall be zero.
            state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::INVALID_FIELD_IN_CDB);
            return Intent::failed();
        }

        let mut out = alloc::vec![0u8; 36];
        // Byte 0: PERIPHERAL QUALIFIER 000b, PERIPHERAL DEVICE TYPE 00h — a
        // direct-access block device, connected.
        out[0] = 0x00;
        // Byte 1: RMB.
        out[1] = if self.removable { 0x80 } else { 0x00 };
        // Byte 2: VERSION. 05h is SPC-3, the version every host understands and
        // the oldest one that is not "obsolete" to a modern driver.
        out[2] = 0x05;
        // Byte 3: RESPONSE DATA FORMAT 2, which is the only legal value; HISUP
        // and NORMACA clear.
        out[3] = 0x02;
        // Byte 4: ADDITIONAL LENGTH (n - 4). Thirty-six bytes total.
        out[4] = (36 - 5) as u8;
        out[8..16].copy_from_slice(&self.vendor_id);
        out[16..32].copy_from_slice(&self.product_id);
        out[32..36].copy_from_slice(&self.revision);
        Intent::parameter(out, allocation)
    }

    /// The four-byte mode parameter header of `MODE SENSE (6)` (Seagate §3.11).
    ///
    /// No pages follow it. A host issues `MODE SENSE` on a USB disk to read one
    /// bit — `WP`, in the device-specific parameter — and this device has no
    /// page whose contents would not be an invention.
    fn mode_parameter_header_6(&self) -> Vec<u8> {
        alloc::vec![
            // MODE DATA LENGTH: the bytes that follow this one.
            3,
            // MEDIUM TYPE: 00h, the default.
            0,
            // DEVICE-SPECIFIC PARAMETER: bit 7 is WP for a block device.
            if self.write_protected() { 0x80 } else { 0x00 },
            // BLOCK DESCRIPTOR LENGTH: none.
            0,
        ]
    }

    /// The eight-byte header of `MODE SENSE (10)` (Seagate §3.12).
    fn mode_parameter_header_10(&self) -> Vec<u8> {
        alloc::vec![
            0,
            6,
            0,
            if self.write_protected() { 0x80 } else { 0x00 },
            0,
            0,
            0,
            0,
        ]
    }

    /// Turn an LBA and a block count into a byte range on the medium, or set
    /// the sense and return `None`.
    ///
    /// **The bounds check the whole device rests on.** Everything is `u64` and
    /// the addition is checked, so an LBA of `u64::MAX` is out of range rather
    /// than a wrap to the front of the disk.
    fn bounds(&self, state: &mut MsdState, lba: u64, count: u64) -> Option<(u64, u64)> {
        let range = lba
            .checked_add(count)
            .filter(|end| *end <= self.blocks)
            .and_then(|_| Some((lba.checked_mul(self.block)?, count.checked_mul(self.block)?)));
        if range.is_none() {
            state.sense = Sense::new(sense_key::ILLEGAL_REQUEST, asc::LBA_OUT_OF_RANGE);
        }
        range
    }

    /// `READ` or `WRITE`: bounds-check, then hand back a cursor.
    fn transfer(&self, state: &mut MsdState, reading: bool, lba: u64, count: u64) -> Intent {
        let Some((offset, length)) = self.bounds(state, lba, count) else {
            return Intent::failed();
        };
        if !reading && self.write_protected() {
            state.sense = Sense::new(sense_key::DATA_PROTECT, asc::WRITE_PROTECTED);
            return Intent::failed();
        }
        if length == 0 {
            // A transfer length of zero is not an error (Seagate §2.2.4: "a
            // TRANSFER LENGTH field set to zero specifies that no logical
            // blocks shall be transferred").
            return Intent::none();
        }
        let direction = if reading {
            Direction::In
        } else {
            Direction::Out
        };
        Intent::blocks(direction, offset, length)
    }

    // -- the endpoints ------------------------------------------------------

    /// One bulk-in transaction: a data-phase packet, or the CSW.
    ///
    /// `consume` is false for the debug path, where nothing may move
    /// (`ROADMAP.md` §15, invariant 5).
    fn bulk_in(&self, dst: &mut [u8], consume: bool) -> Completion {
        let mut state = self.state.lock();
        if state.wedged || state.stall_in {
            // Re-arm, so that a `CLEAR_FEATURE` alone cannot resume a device
            // §6.6.1 says stays stalled until a Reset Recovery.
            if state.wedged {
                state.stall_in = true;
            }
            return Completion::stall();
        }
        match state.phase {
            // §3.3: the host may ask for data or a CSW before the CBW it
            // belongs to. There is nothing to give, and `NAK` is what a device
            // with nothing to give answers (USB 2.0 §8.4.5).
            Phase::Command | Phase::DataOut => Completion::nak(),
            Phase::DataIn => {
                let want = state
                    .device_remaining
                    .min(state.host_remaining)
                    .min(dst.len() as u64) as usize;
                if want == 0 {
                    // The data phase is over; the CSW is next.
                    if consume {
                        self.to_status(&mut state);
                    }
                    return Completion::ack(0);
                }
                let moved = match &state.payload {
                    Payload::Buffer { bytes, offset } => {
                        let from = (*offset).min(bytes.len() as u64) as usize;
                        let n = want.min(bytes.len() - from);
                        dst[..n].copy_from_slice(&bytes[from..from + n]);
                        n
                    }
                    Payload::Blocks { offset } => {
                        let at = *offset;
                        if let Err(e) = self.media.read_at(at, &mut dst[..want]) {
                            if consume {
                                self.fail_transfer(&mut state, e, false);
                            }
                            return Completion::stall();
                        }
                        want
                    }
                    Payload::None => 0,
                };
                if !consume {
                    return Completion::ack(moved as u64);
                }
                match &mut state.payload {
                    Payload::Buffer { offset, .. } => *offset += moved as u64,
                    Payload::Blocks { offset } => *offset += moved as u64,
                    Payload::None => {}
                }
                state.device_remaining -= moved as u64;
                state.host_remaining = state.host_remaining.saturating_sub(moved as u64);
                state.moved += moved as u64;
                if state.device_remaining == 0 {
                    // §6.7.2 cases (5) and (6): the short packet this produces
                    // is what tells the host the data phase ended, and the
                    // residue in the CSW says by how much.
                    self.to_status(&mut state);
                }
                Completion::ack(moved as u64)
            }
            Phase::Status => {
                let csw = encode_csw(state.tag, Self::residue(&state), state.status);
                let from = usize::from(state.csw_sent).min(CSW_BYTES);
                let n = (CSW_BYTES - from).min(dst.len());
                dst[..n].copy_from_slice(&csw[from..from + n]);
                if consume {
                    state.csw_sent = (from + n) as u8;
                    if state.csw_sent as usize >= CSW_BYTES {
                        // Ready for the next CBW (§6.2.1).
                        let carry = state.sense;
                        let stalls = (state.stall_in, state.stall_out);
                        *state = MsdState {
                            sense: carry,
                            stall_in: stalls.0,
                            stall_out: stalls.1,
                            ..MsdState::default()
                        };
                    }
                }
                Completion::ack(n as u64)
            }
        }
    }

    /// One bulk-out transaction: a CBW, or a data-phase packet.
    fn bulk_out(&self, src: &[u8]) -> Completion {
        let mut state = self.state.lock();
        if state.wedged || state.stall_out {
            if state.wedged {
                state.stall_out = true;
            }
            return Completion::stall();
        }
        match state.phase {
            Phase::Command => self.begin(&mut state, src),
            Phase::DataOut => {
                let take = state.host_remaining.min(src.len() as u64) as usize;
                if take == 0 {
                    return Completion::ack(0);
                }
                // Only the part the device actually wants is *relevant*
                // (§5.2); the rest is accepted and dropped, which is what
                // §6.7.3 permits for cases (9), (10), (11) and (13).
                let relevant = state.device_remaining.min(take as u64) as usize;
                if relevant > 0 {
                    if let Payload::Blocks { offset } = &state.payload {
                        let at = *offset;
                        if let Err(e) = self.media.write_at(at, &src[..relevant]) {
                            self.fail_transfer(&mut state, e, true);
                            return Completion::stall();
                        }
                        if let Payload::Blocks { offset } = &mut state.payload {
                            *offset += relevant as u64;
                        }
                    }
                    state.device_remaining -= relevant as u64;
                    state.moved += relevant as u64;
                }
                state.host_remaining -= take as u64;
                if state.host_remaining == 0 {
                    self.to_status(&mut state);
                }
                Completion::ack(take as u64)
            }
            // The host is sending during a phase that has nothing to take.
            // §3.3 forbids it; `NAK` is the answer that neither loses data nor
            // invents an error the specification does not define.
            Phase::DataIn | Phase::Status => Completion::nak(),
        }
    }

    /// BOT §3.1: ready the device for the next CBW, **without** touching the
    /// endpoint stall conditions, which the specification says survive.
    fn class_reset(&self) {
        let mut state = self.state.lock();
        let carry = (state.stall_in, state.stall_out, state.sense);
        *state = MsdState {
            stall_in: carry.0,
            stall_out: carry.1,
            sense: carry.2,
            ..MsdState::default()
        };
    }
}

/// Read a big-endian field out of a command block, or zero if it is short.
///
/// A `CBWCB` is sixteen bytes whatever `bCBWCBLength` said, so these never read
/// out of bounds in practice; they are written this way so that a caller that
/// passes a truncated block gets zeros rather than a panic.
fn be16(cdb: &[u8], at: usize) -> u16 {
    let hi = cdb.get(at).copied().unwrap_or(0);
    let lo = cdb.get(at + 1).copied().unwrap_or(0);
    u16::from_be_bytes([hi, lo])
}

fn be24(cdb: &[u8], at: usize) -> u32 {
    (u32::from(cdb.get(at).copied().unwrap_or(0)) << 16)
        | (u32::from(cdb.get(at + 1).copied().unwrap_or(0)) << 8)
        | u32::from(cdb.get(at + 2).copied().unwrap_or(0))
}

fn be32(cdb: &[u8], at: usize) -> u32 {
    let mut out = 0u32;
    for i in 0..4 {
        out = (out << 8) | u32::from(cdb.get(at + i).copied().unwrap_or(0));
    }
    out
}

fn be64(cdb: &[u8], at: usize) -> u64 {
    let mut out = 0u64;
    for i in 0..8 {
        out = (out << 8) | u64::from(cdb.get(at + i).copied().unwrap_or(0));
    }
    out
}

impl Function for StorageFunction {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        self.speed
    }

    fn reset(&self) {
        // A *bus* reset, which is not the class reset: USB 2.0 §9.1.1.3 puts
        // the device back in the Default state entirely, stalls included.
        *self.state.lock() = MsdState::default();
    }

    fn configure(&self, _value: u8) -> bool {
        // §9.4.7 clears every halt, and a device that has just been
        // (re)configured has no transfer in flight.
        *self.state.lock() = MsdState::default();
        true
    }

    fn control_in(&self, setup: SetupPacket) -> Option<Vec<u8>> {
        if setup.kind() != RequestKind::Class
            || setup.recipient() != Recipient::Interface
            || setup.index != INTERFACE
        {
            return None;
        }
        match setup.request {
            // §3.2: one byte, the highest LUN. This device has one logical
            // unit, so the answer is zero — and answering rather than stalling
            // is what the specification prefers, even though it permits a
            // device with a single LUN to stall.
            class_request::GET_MAX_LUN => Some(alloc::vec![0]),
            _ => None,
        }
    }

    fn control_out(&self, setup: SetupPacket, _data: &[u8]) -> bool {
        if setup.kind() != RequestKind::Class
            || setup.recipient() != Recipient::Interface
            || setup.index != INTERFACE
        {
            return false;
        }
        match setup.request {
            class_request::BULK_ONLY_RESET if setup.value == 0 && setup.length == 0 => {
                self.class_reset();
                true
            }
            _ => false,
        }
    }

    fn halt_cleared(&self, endpoint: u8) {
        let mut state = self.state.lock();
        let number = endpoint & 0x0f;
        if endpoint & Direction::BIT != 0 {
            if number == ENDPOINT_IN {
                state.stall_in = false;
            }
        } else if number == ENDPOINT_OUT {
            state.stall_out = false;
        }
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT_IN {
            return Completion::stall();
        }
        self.bulk_in(dst, true)
    }

    fn endpoint_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        if endpoint != ENDPOINT_OUT {
            return Completion::stall();
        }
        self.bulk_out(src)
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT_IN {
            return Completion::stall();
        }
        // The debug path. It reads the medium — which has no side effects —
        // but advances no cursor, pops no CSW, clears no sense and moves no
        // phase, so a monitor showing what is queued cannot break the transfer
        // it is looking at.
        self.bulk_in(dst, false)
    }
}

// ---------------------------------------------------------------------------
// The machine object
// ---------------------------------------------------------------------------

/// A USB mass storage device: one LUN, one medium, Bulk-Only Transport.
#[derive(Debug)]
pub struct UsbStorage {
    peripheral: Arc<Peripheral>,
    function: Arc<StorageFunction>,
    /// What a snapshot should do about the bytes. The medium's policy, asked
    /// once at construction so that `save` cannot disagree with `load`.
    snapshot: Snapshot,
}

impl UsbStorage {
    /// Validate `props` and build the device.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`crate::bus::usb::UsbBus`] to plug into. Required.
    /// * `port` — which port of it. Defaults to 0.
    /// * `image` — the **media slot** the bytes come from. A [`Medium`] the
    ///   host installed under that name — what `rsemu run … --drive usb0=disk.img`
    ///   does — wins and brings its own capacity; otherwise the media table's
    ///   bytes are copied into a [`RamStore`] of `size` bytes. Defaults to
    ///   `usb0`.
    /// * `size` — how big the disk is when no medium was installed.
    /// * `block` — bytes in a logical block, a power of two from 512 to 4096.
    ///   Defaults to 512.
    /// * `readonly` — refuse writes whatever the medium allows. Defaults to
    ///   false.
    /// * `removable` — the `RMB` bit of the `INQUIRY` data. Defaults to true,
    ///   which is what a USB stick reports.
    /// * `vendor`, `product` — `idVendor` and `idProduct`.
    /// * `vendor-id`, `product-id`, `revision` — the three ASCII fields of the
    ///   standard `INQUIRY` data.
    /// * `serial` — the serial number string. BOT §4.1.1 asks for at least
    ///   twelve characters from `0-9A-F`, and the default obeys that.
    /// * `speed` — `high` (the default) or `full`. **Not `low`**: USB 2.0 §5.8
    ///   defines no bulk transfers at low speed, so a low-speed mass storage
    ///   device could not carry its own protocol.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing
    /// property; [`Error::Config`] for a low speed, a
    /// block size that is not a power of two, a capacity that is zero or is not
    /// a whole number of blocks, an image that does not fit, or a port that is
    /// taken.
    pub fn new(props: &Props) -> Result<UsbStorage> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let port = r.or_range("port", 0u64, 0..=u64::from(u8::MAX))?;
        let size = r.or_size("size", 0)?;
        let block = r.or_range("block", 512u64, 512..=4096)?;
        let read_only = r.or("readonly", false)?;
        let removable = r.or("removable", true)?;
        let vendor = r.or_range("vendor", 0x0781u64, 0..=u64::from(u16::MAX))?;
        let product = r.or_range("product", 0x5567u64, 0..=u64::from(u16::MAX))?;
        let vendor_id = r.or_str("vendor-id", "RSEMU")?.to_string();
        let product_id = r.or_str("product-id", "USB DISK")?.to_string();
        let revision = r.or_str("revision", "1.00")?.to_string();
        let serial = r.or_str("serial", "0123456789AB")?.to_string();
        let spelling = r.or_str("speed", Speed::High.name())?.to_string();
        let media = r.optional_media("image")?;
        let slot = media.map(crate::core::props::Media::name);
        let image = media.map(crate::core::props::Media::to_bytes);
        r.finish()?;

        let speed = Speed::from_name(&spelling).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: alloc::format!("`speed` is one of {:?}, not `{spelling}`", Speed::NAMES),
        })?;
        if speed == Speed::Low {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "USB 2.0 §5.8 defines no bulk transfers at low speed, and Bulk-Only \
                     Transport is two bulk endpoints: a low-speed mass storage device cannot \
                     exist. Use `full` or `high`",
                ),
            });
        }
        if !block.is_power_of_two() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!("a logical block is a power of two, and {block} is not"),
            });
        }

        // A medium the *host* installed wins over the media table, exactly as
        // it does for an NVMe namespace: a run that said `--drive usb0=disk.img`
        // meant it.
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
                    "a disk with no bytes is not a disk: give it `size`, an `image` with bytes \
                     behind it, or a medium installed under its media slot",
                ),
            });
        }
        if !bytes.is_multiple_of(block) {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "{bytes} byte(s) is not a whole number of {block}-byte logical blocks"
                ),
            });
        }

        let media: Arc<dyn Medium> = match supplied {
            Some(medium) => medium,
            None => {
                let store = RamStore::new(bytes);
                if let Some(image) = image {
                    if image.len() as u64 > bytes {
                        return Err(Error::Config {
                            at: String::from(CLASS_NAME),
                            message: alloc::format!(
                                "the bound image is {} byte(s) and the disk holds {bytes}",
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

        let disk = UsbStorage::with_medium(
            media,
            block,
            read_only,
            removable,
            speed,
            (vendor as u16, product as u16),
            (&vendor_id, &product_id, &revision, &serial),
        );

        // Opening the table entry creates nothing anybody can see; the bus is
        // sized by whichever object names it first, which is the controller in
        // every machine description that makes sense.
        let bus = buses::attach(props, &bus_name, port as u8 + 1)?;
        bus.attach(port as u8, disk.device())?;
        Ok(disk)
    }

    /// A disk on a medium the caller already holds, plugged into nothing.
    ///
    /// For a test, or an embedder that owns its own bus and attaches the device
    /// itself with [`UsbStorage::device`].
    #[must_use]
    pub fn with_medium(
        media: Arc<dyn Medium>,
        block: u64,
        read_only: bool,
        removable: bool,
        speed: Speed,
        ids: (u16, u16),
        strings: (&str, &str, &str, &str),
    ) -> UsbStorage {
        let snapshot = media.snapshot();
        let function = Arc::new(StorageFunction::new(
            media, block, read_only, removable, speed, ids, strings,
        ));
        let peripheral = Arc::new(Peripheral::new(Arc::clone(&function) as Arc<dyn Function>));
        UsbStorage {
            peripheral,
            function,
            snapshot,
        }
    }

    /// A disk of `bytes` bytes in host memory, at high speed. The shortest way
    /// to one for a test.
    #[must_use]
    pub fn in_memory(bytes: u64) -> UsbStorage {
        UsbStorage::with_medium(
            Arc::new(RamStore::new(bytes)),
            512,
            false,
            true,
            Speed::High,
            (0x0781, 0x5567),
            ("RSEMU", "USB DISK", "1.00", "0123456789AB"),
        )
    }

    /// The disk as the fabric sees it, for
    /// [`UsbBus::attach`](crate::bus::usb::UsbBus::attach).
    #[must_use]
    pub fn device(&self) -> Arc<dyn UsbDevice> {
        Arc::clone(&self.peripheral) as Arc<dyn UsbDevice>
    }

    /// The address the host has given it, or zero before enumeration.
    #[must_use]
    pub fn address(&self) -> crate::bus::usb::DeviceAddress {
        self.peripheral.address()
    }

    /// The configuration the host selected, or zero.
    #[must_use]
    pub fn configuration(&self) -> u8 {
        self.peripheral.endpoint0().configuration()
    }

    /// The medium behind the disk — what a test asserts against.
    #[must_use]
    pub fn medium(&self) -> &Arc<dyn Medium> {
        &self.function.media
    }

    /// Bytes in a logical block.
    #[must_use]
    pub fn block_size(&self) -> u64 {
        self.function.block
    }

    /// How many logical blocks the disk holds.
    #[must_use]
    pub fn block_count(&self) -> u64 {
        self.function.blocks
    }

    /// `wMaxPacketSize` of the two bulk endpoints — the number a host driver
    /// has to read out of the endpoint descriptors, exposed so a test does not
    /// have to.
    #[must_use]
    pub fn max_packet(&self) -> u16 {
        self.function.max_packet
    }

    /// Encode everything the BOT state machine remembers.
    fn save_state<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        self.peripheral.endpoint0().save(w)?;
        let state = self.function.state.lock();
        w.write_u8(state.phase.code())?;
        w.write_u32(state.tag)?;
        w.write_u32(state.host_length)?;
        w.write_u64(state.host_remaining)?;
        w.write_u64(state.device_remaining)?;
        w.write_u64(state.moved)?;
        w.write_u8(state.status)?;
        w.write_u8(state.csw_sent)?;
        w.write_bool(state.stall_in)?;
        w.write_bool(state.stall_out)?;
        w.write_bool(state.wedged)?;
        w.write_u8(state.sense.key)?;
        w.write_u8(state.sense.asc)?;
        w.write_u8(state.sense.ascq)?;
        w.write_u8(state.payload.code())?;
        match &state.payload {
            Payload::None => Ok(()),
            Payload::Buffer { bytes, offset } => {
                w.write_bytes(bytes)?;
                w.write_u64(*offset)
            }
            Payload::Blocks { offset } => w.write_u64(*offset),
        }
    }

    /// Restore what [`UsbStorage::save_state`] wrote.
    fn load_state<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        self.peripheral.endpoint0().load(r)?;
        let phase = Phase::from_code(r.read_u8()?);
        let tag = r.read_u32()?;
        let host_length = r.read_u32()?;
        let host_remaining = r.read_u64()?;
        let device_remaining = r.read_u64()?;
        let moved = r.read_u64()?;
        let status = r.read_u8()?;
        let csw_sent = r.read_u8()?;
        let stall_in = r.read_bool()?;
        let stall_out = r.read_bool()?;
        let wedged = r.read_bool()?;
        let sense = Sense {
            key: r.read_u8()?,
            asc: r.read_u8()?,
            ascq: r.read_u8()?,
        };
        let payload = match r.read_u8()? {
            1 => {
                let bytes = r.read_bytes()?.to_vec();
                let offset = r.read_u64()?;
                if offset > bytes.len() as u64 {
                    return Err(Error::State(alloc::format!(
                        "usb.storage: a data phase {offset} byte(s) into a {}-byte parameter \
                         response",
                        bytes.len()
                    )));
                }
                Payload::Buffer { bytes, offset }
            }
            2 => Payload::Blocks {
                offset: r.read_u64()?,
            },
            _ => Payload::None,
        };
        if usize::from(csw_sent) > CSW_BYTES {
            return Err(Error::State(alloc::format!(
                "usb.storage: {csw_sent} byte(s) of a {CSW_BYTES}-byte command status wrapper"
            )));
        }
        *self.function.state.lock() = MsdState {
            phase,
            tag,
            host_length,
            host_remaining,
            device_remaining,
            moved,
            status,
            payload,
            csw_sent,
            stall_in,
            stall_out,
            wedged,
            sense,
        };
        Ok(())
    }
}

impl Device for UsbStorage {
    fn class(&self) -> &'static DeviceClass {
        &STORAGE_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the disk plugged itself into the named bus at
        // construction, which is the rendezvous table and not an observable
        // action.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        self.peripheral.bus_reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The medium's own policy first, on the same terms an NVMe namespace
        // and an AHCI port take it: a file-backed disk is *referenced* and a
        // RAM one is captured, because writing sixteen gigabytes into a
        // snapshot chunk is not on offer (`dev::ata::Snapshot`).
        match self.snapshot {
            Snapshot::Refuse => {
                return Err(Error::State(alloc::format!(
                    "usb.storage: this disk's medium ({}) refuses to be snapshotted",
                    self.function.media.describe()
                )));
            }
            Snapshot::Reference => {
                w.write_u8(1)?;
                w.write_str(&self.function.media.describe())?;
                self.function.media.flush().map_err(|e| {
                    Error::State(alloc::format!(
                        "usb.storage: the medium would not flush before a snapshot: {e}"
                    ))
                })?;
            }
            Snapshot::Capture => {
                w.write_u8(0)?;
                let held = self.function.blocks * self.function.block;
                w.write_u64(held)?;
                let mut block = alloc::vec![0u8; self.function.block as usize];
                let mut at = 0u64;
                while at < held {
                    self.function.media.read_at(at, &mut block).map_err(|e| {
                        Error::State(alloc::format!(
                            "usb.storage: the medium would not read for a snapshot: {e}"
                        ))
                    })?;
                    w.write_all(&block)?;
                    at += self.function.block;
                }
            }
        }
        self.save_state(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        match r.read_u8()? {
            0 => {
                let held = r.read_u64()?;
                let want = self.function.blocks * self.function.block;
                if held != want {
                    return Err(Error::State(alloc::format!(
                        "usb.storage: a snapshot of {held} byte(s) into a {want}-byte disk"
                    )));
                }
                let mut at = 0u64;
                let step = self.function.block as usize;
                while at < held {
                    let chunk = r.take(step)?;
                    self.function.media.write_at(at, chunk).map_err(|e| {
                        Error::State(alloc::format!(
                            "usb.storage: the medium would not take a snapshot's bytes: {e}"
                        ))
                    })?;
                    at += self.function.block;
                }
            }
            _ => {
                let described = r.read_str()?;
                let now = self.function.media.describe();
                if described != now {
                    return Err(Error::State(alloc::format!(
                        "usb.storage: this snapshot references `{described}` and the disk is \
                         backed by `{now}`"
                    )));
                }
            }
        }
        self.load_state(r)
    }
}

impl Instance for UsbStorage {}

/// The `usb.storage` device class.
pub static STORAGE_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a USB mass storage device: Bulk-Only Transport and a SCSI command set over two \
              bulk endpoints, reading and writing a real medium",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the named USB bus to plug into",
        },
        PropertySpec {
            name: "port",
            kind: ValueKind::Uint,
            required: false,
            summary: "which port of that bus (default 0)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot the disk is bound to; a host medium under that name wins",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how big the disk is when no medium was installed",
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
            summary: "refuse writes whatever the medium allows (default false)",
        },
        PropertySpec {
            name: "removable",
            kind: ValueKind::Bool,
            required: false,
            summary: "the RMB bit of the INQUIRY data (default true)",
        },
        PropertySpec {
            name: "vendor",
            kind: ValueKind::Uint,
            required: false,
            summary: "idVendor, as the device descriptor reports it",
        },
        PropertySpec {
            name: "product",
            kind: ValueKind::Uint,
            required: false,
            summary: "idProduct",
        },
        PropertySpec {
            name: "vendor-id",
            kind: ValueKind::Str,
            required: false,
            summary: "T10 VENDOR IDENTIFICATION in the INQUIRY data, eight characters",
        },
        PropertySpec {
            name: "product-id",
            kind: ValueKind::Str,
            required: false,
            summary: "PRODUCT IDENTIFICATION in the INQUIRY data, sixteen characters",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Str,
            required: false,
            summary: "PRODUCT REVISION LEVEL in the INQUIRY data, four characters",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Str,
            required: false,
            summary: "the serial number string; BOT 4.1.1 asks for twelve or more of 0-9A-F",
        },
        PropertySpec {
            name: "speed",
            kind: ValueKind::Str,
            required: false,
            summary: "how fast it signals: `high` (default) or `full`; never `low`",
        },
    ],
    construct: |props| Ok(Box::new(UsbStorage::new(props)?)),
};

/// Add [`STORAGE_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&STORAGE_CLASS)
}

/// Bind [`STORAGE_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(UsbStorage::new(props)?)))
}

/// What the validator should know about `usb.storage`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("port", ValueKind::Uint).range(0, u64::from(u8::MAX)))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("block", ValueKind::Uint).range(512, 4096))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("removable", ValueKind::Bool))
        .prop(PropSchema::new("vendor", ValueKind::Uint).range(0, u64::from(u16::MAX)))
        .prop(PropSchema::new("product", ValueKind::Uint).range(0, u64::from(u16::MAX)))
        .prop(PropSchema::new("vendor-id", ValueKind::Str))
        .prop(PropSchema::new("product-id", ValueKind::Str))
        .prop(PropSchema::new("revision", ValueKind::Str))
        .prop(PropSchema::new("serial", ValueKind::Str))
        .prop(PropSchema::new("speed", ValueKind::Str).values(&["full", "high"]))
}

/// The wire format is the specification's, and these are the numbers that say
/// so. A change to any of them is a change to the protocol, not a refactor.
const _: () = {
    assert!(CBW_BYTES == 31);
    assert!(CSW_BYTES == 13);
    assert!(CDB_BYTES == 16);
    assert!(SENSE_BYTES == 18);
    assert!(CBW_SIGNATURE == 0x4342_5355);
    assert!(CSW_SIGNATURE == 0x5342_5355);
};

/// Helper for a snapshot round-trip test, which needs a plain byte vector
/// rather than a [`ChunkWriter`].
#[cfg(test)]
impl UsbStorage {
    fn save_to(&self, out: &mut Vec<u8>) -> Result<()> {
        // Only the RAM path is exercised: a test's medium is a `RamStore`.
        out.write_u8(0)?;
        let held = self.function.blocks * self.function.block;
        out.write_u64(held)?;
        let mut block = alloc::vec![0u8; self.function.block as usize];
        let mut at = 0u64;
        while at < held {
            self.function
                .media
                .read_at(at, &mut block)
                .expect("a RamStore reads");
            out.write_all(&block)?;
            at += self.function.block;
        }
        self.save_state(out)
    }

    fn load_from(&self, bytes: &[u8]) -> Result<()> {
        let mut r = ChunkReader::new(bytes);
        self.load(&mut r)
    }
}

#[cfg(test)]
mod tests;
