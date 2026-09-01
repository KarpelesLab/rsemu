//! An SD memory card: the protocol, the state machine and the registers.
//!
//! **This models the card, not a controller.** Nothing here knows what an
//! `SDMMC_CMDR` is. A card speaks a command/response protocol over a set of
//! wires, and *which* wires — the four-bit SD bus, the one-bit SD bus, or SPI
//! — is the transport's business. So the seam this file exposes is the one the
//! SD Physical Layer Simplified Specification itself draws:
//!
//! ```text
//!   controller ──► command(index, arg) ──────────────► card
//!              ◄── Reply::{None, Short, Long} ────────
//!
//!              ──► write_data(bytes) ───────────────►   (host → card)
//!              ◄── read_data(bytes)  ───────────────    (card → host)
//! ```
//!
//! Four calls. Everything a controller does — echoing the command index into a
//! `RESPCMD` register, deciding that no response means "timeout" rather than
//! "this command has none", pacing the bytes through a FIFO or a DMA engine —
//! is above that line, because on real silicon it is above that line too.
//!
//! # Why the split is drawn exactly here
//!
//! An SPI-mode card and an SD-mode card are **the same die**. The command set,
//! the register contents, the state machine and the addressing are identical;
//! what differs is the framing (`0x40 | index`, then four argument bytes, then
//! a CRC byte, then a one-byte R1 instead of a 48-bit response) and the fact
//! that SPI has no bus addressing, so `CMD2`/`CMD3`/`CMD7` do not exist and an
//! initialised card is simply *there*. [`BusMode`] is that difference and it is
//! the only place in this file the transport is mentioned. An SPI controller
//! wanting a card implements `bus::spi::SpiSlave` over one of these and
//! synthesises its one-byte R1 from the card status a [`Reply::Short`] carries
//! — it does not need a second card model, and if it did, this split would be
//! in the wrong place.
//!
//! # Time
//!
//! **Deliberately zero**, the same choice [`crate::dev::flash::cfi`] makes and
//! for a weaker version of the same reason. A real card's programming time is
//! real: after a `CMD24` it pulls DAT0 low for anywhere between a hundred
//! microseconds and a quarter of a second, and firmware polls `CMD13` or the
//! busy line waiting for it. Here a block is programmed inside the call that
//! delivers its last byte, so [`SdCard::is_busy`] never reports busy and a
//! polling loop terminates on its first iteration.
//!
//! Unlike a NOR part, a card *does* have a clock — the controller supplies it —
//! so this could be modelled, and [`SdCard::is_busy`] exists so that a
//! controller asks rather than assumes. What stops it being modelled today is
//! that the duration is not a fact: `TAAC`/`NSAC`/`R2W_FACTOR` in the CSD are
//! the card's *declared* figures and every real part beats them by an order of
//! magnitude, so a number here would be invented rather than sourced. When a
//! guest turns up that can tell the difference, this device grows a clock
//! domain and the busy window becomes a scheduler event; nothing above it has
//! to change, because the question is already asked through a method.
//!
//! # The backing store
//!
//! A [`RamStore`] filled from a media slot, not an `fstool::BlockDevice`. The
//! same argument `dev-flash-cfi` makes: the contents are a flat image, byte
//! addressed, and reaching for a disk-image crate would drag `std` into a
//! `no_std` device for nothing. A card backed by a real, large, sparse image is
//! a `dev/blk/sd` variant under the documented `std` exception — it would reuse
//! this whole protocol half and replace only [`SdCard::read_media`] and
//! [`SdCard::write_media`].
//!
//! # Sources
//!
//! * **SD Association, *Physical Layer Simplified Specification*** — the free,
//!   publicly downloadable specification named in `docs/buses/storage.md`.
//!   Specifically: §4.2 (card identification and the `ACMD41` handshake), §4.3
//!   (data transfer) and its Table 4-35 (the state transition table), §4.3.10
//!   (`CMD6` switch function), §4.5 (`CRC7`), §4.7.4 (the command
//!   descriptions), §4.9 (the response formats R1/R1b/R2/R3/R6/R7), §4.10.1
//!   (the card status), §4.10.2 (the SD status), §5.1 (`OCR`), §5.2 (`CID`),
//!   §5.3.2 and §5.3.3 (`CSD` versions 1.0 and 2.0), and §5.6 (`SCR`).
//!
//! No emulator source of any licence was consulted, and none of this came from
//! an operating system's MMC subsystem.

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
pub const CLASS_NAME: &str = "sd.card";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The block a card reads and writes in, in bytes.
///
/// 512 everywhere in this file. `READ_BL_LEN` and `WRITE_BL_LEN` in the CSD say
/// nine, and a high-capacity card's block length is *fixed* at 512 (§5.3.3) —
/// only a standard-capacity card lets `CMD16` move the read length, and only
/// downwards.
pub const BLOCK: u64 = 512;

/// The largest card the standard-capacity encoding can describe: 2 GiB (§5.3.2).
pub const MAX_STANDARD_CAPACITY: u64 = 2 * 1024 * 1024 * 1024;

/// The granularity of the high-capacity `C_SIZE` field: 512 KiB (§5.3.3).
pub const HIGH_CAPACITY_UNIT: u64 = 512 * 1024;

// ---------------------------------------------------------------------------
// The card status (§4.10.1, Table 4-42)
// ---------------------------------------------------------------------------

/// The command's argument was out of the card's address range.
pub const OUT_OF_RANGE: u32 = 1 << 31;
/// A misaligned address that did not match the block length.
pub const ADDRESS_ERROR: u32 = 1 << 30;
/// The transferred block length is not allowed for this card.
pub const BLOCK_LEN_ERROR: u32 = 1 << 29;
/// An erase sequence was cleared before executing.
pub const ERASE_SEQ_ERROR: u32 = 1 << 28;
/// An invalid selection of write blocks for erase.
pub const ERASE_PARAM: u32 = 1 << 27;
/// The host attempted to write to a protected block.
pub const WP_VIOLATION: u32 = 1 << 26;
/// The command was not legal for the card's current state.
pub const ILLEGAL_COMMAND: u32 = 1 << 22;
/// A generic card error: something in the sticky set is set.
pub const CARD_ERROR: u32 = 1 << 19;
/// The card will accept data. Always set here: see the module note on time.
pub const READY_FOR_DATA: u32 = 1 << 8;
/// The card expects the next command to be an application command.
pub const APP_CMD: u32 = 1 << 5;
/// The sequence of an authentication process was wrong.
pub const AKE_SEQ_ERROR: u32 = 1 << 3;

/// Where the state machine's position sits in the card status.
const STATE_SHIFT: u32 = 9;

/// The error bits that are cleared once they have been reported, which is
/// every one this model raises. §4.10.1's "clear condition" column calls them
/// type C: "clear by read".
const CLEAR_ON_READ: u32 = OUT_OF_RANGE
    | ADDRESS_ERROR
    | BLOCK_LEN_ERROR
    | ERASE_SEQ_ERROR
    | ERASE_PARAM
    | WP_VIOLATION
    | ILLEGAL_COMMAND
    | AKE_SEQ_ERROR;

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// The command indices this card answers.
///
/// Named rather than inlined because the state table below reads as prose with
/// them and as a soup of magic numbers without.
pub mod cmd {
    /// `GO_IDLE_STATE`: reset to the idle state.
    pub const GO_IDLE_STATE: u8 = 0;
    /// `ALL_SEND_CID`: every card on the bus sends its CID.
    pub const ALL_SEND_CID: u8 = 2;
    /// `SEND_RELATIVE_ADDR`: the card publishes a new bus address.
    pub const SEND_RELATIVE_ADDR: u8 = 3;
    /// `SWITCH_FUNC`: query or change the access mode and the other groups.
    pub const SWITCH_FUNC: u8 = 6;
    /// `SELECT/DESELECT_CARD`.
    pub const SELECT_CARD: u8 = 7;
    /// `SEND_IF_COND`: the version-2 interface-condition handshake.
    pub const SEND_IF_COND: u8 = 8;
    /// `SEND_CSD`.
    pub const SEND_CSD: u8 = 9;
    /// `SEND_CID`.
    pub const SEND_CID: u8 = 10;
    /// `STOP_TRANSMISSION`.
    pub const STOP_TRANSMISSION: u8 = 12;
    /// `SEND_STATUS`.
    pub const SEND_STATUS: u8 = 13;
    /// `GO_INACTIVE_STATE`.
    pub const GO_INACTIVE_STATE: u8 = 15;
    /// `SET_BLOCKLEN`.
    pub const SET_BLOCKLEN: u8 = 16;
    /// `READ_SINGLE_BLOCK`.
    pub const READ_SINGLE_BLOCK: u8 = 17;
    /// `READ_MULTIPLE_BLOCK`.
    pub const READ_MULTIPLE_BLOCK: u8 = 18;
    /// `SET_BLOCK_COUNT`: how many blocks the next multiple transfer moves.
    pub const SET_BLOCK_COUNT: u8 = 23;
    /// `WRITE_BLOCK`.
    pub const WRITE_BLOCK: u8 = 24;
    /// `WRITE_MULTIPLE_BLOCK`.
    pub const WRITE_MULTIPLE_BLOCK: u8 = 25;
    /// `ERASE_WR_BLK_START`.
    pub const ERASE_WR_BLK_START: u8 = 32;
    /// `ERASE_WR_BLK_END`.
    pub const ERASE_WR_BLK_END: u8 = 33;
    /// `ERASE`.
    pub const ERASE: u8 = 38;
    /// `APP_CMD`: the next command is an application command.
    pub const APP_CMD: u8 = 55;

    /// `SET_BUS_WIDTH` (application command).
    pub const A_SET_BUS_WIDTH: u8 = 6;
    /// `SD_STATUS` (application command).
    pub const A_SD_STATUS: u8 = 13;
    /// `SD_SEND_OP_COND` (application command): the initialisation handshake.
    pub const A_SD_SEND_OP_COND: u8 = 41;
    /// `SEND_SCR` (application command).
    pub const A_SEND_SCR: u8 = 51;
}

// ---------------------------------------------------------------------------
// The state machine (§4.3, Table 4-35)
// ---------------------------------------------------------------------------

/// Where the card is in its state machine.
///
/// A real `enum`: there are exactly nine, the encoding is fixed by the
/// specification's `CURRENT_STATE` field, and exhaustiveness is what makes the
/// transition table below checkable (`CLAUDE.md`, "Type conventions").
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
#[repr(u8)]
pub enum Phase {
    /// Powered up, waiting for `ACMD41` to finish the voltage handshake.
    #[default]
    Idle = 0,
    /// The handshake is done and the card is waiting for `CMD2`.
    Ready = 1,
    /// The CID has been sent; waiting for `CMD3` to be given an address.
    Identification = 2,
    /// Addressed, but not selected. The resting state of an idle card.
    Standby = 3,
    /// Selected. Commands that move data are legal here and nowhere else.
    Transfer = 4,
    /// Sending data to the host.
    SendingData = 5,
    /// Receiving data from the host.
    ReceiveData = 6,
    /// Programming a received block.
    Programming = 7,
    /// Disconnected while programming.
    Disconnect = 8,
    /// Switched off the bus by `CMD15`, answering nothing until power cycles.
    ///
    /// Not one of the specification's `CURRENT_STATE` codes, because a card in
    /// this state does not respond at all and so never reports one.
    Inactive = 15,
}

impl Phase {
    /// The `CURRENT_STATE` code this phase reports in a card status.
    #[must_use]
    pub fn code(self) -> u32 {
        u32::from(self as u8) & 0xf
    }

    fn from_code(code: u8) -> Result<Phase> {
        Ok(match code {
            0 => Phase::Idle,
            1 => Phase::Ready,
            2 => Phase::Identification,
            3 => Phase::Standby,
            4 => Phase::Transfer,
            5 => Phase::SendingData,
            6 => Phase::ReceiveData,
            7 => Phase::Programming,
            8 => Phase::Disconnect,
            15 => Phase::Inactive,
            other => {
                return Err(Error::State(format!("{other} is not an SD card state")));
            }
        })
    }
}

/// Which transport the card is behind.
///
/// The *only* place this file knows there is more than one. See the module
/// documentation for why it is one field and not a second card model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BusMode {
    /// The native SD bus: `CMD2`/`CMD3` publish an address, `CMD7` selects.
    #[default]
    Sd,
    /// SPI. There is no bus addressing, so a card that has finished `ACMD41`
    /// is already in [`Phase::Transfer`] and the addressing commands never
    /// happen (Physical Layer §7.2).
    Spi,
}

// ---------------------------------------------------------------------------
// Responses
// ---------------------------------------------------------------------------

/// What came back on the CMD line.
///
/// The response *formats* — R1, R1b, R2, R3, R6, R7 — collapse to three shapes
/// on the wire, and the shape is all a controller can see. Which format a given
/// command uses is a property of the command, known to both ends in advance;
/// encoding it here would be modelling the driver's knowledge rather than the
/// card's behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reply {
    /// Nothing was driven on CMD.
    ///
    /// Two situations that are genuinely one situation on the wire: a broadcast
    /// command with no response (`CMD0`, `CMD15`), and a card that was not
    /// addressed, or was inactive, and stayed quiet. A controller tells them
    /// apart from *its own* `WAITRESP` field — silence when it expected nothing
    /// is a command sent, silence when it expected a response is a timeout —
    /// which is exactly what the silicon does.
    None,
    /// A 48-bit response: R1, R1b, R3, R6 or R7.
    Short {
        /// The command index echoed in bits 45:40, which a controller latches
        /// into its `RESPCMD` register.
        ///
        /// `0x3f` for R2 and R3, whose corresponding field is all ones because
        /// they carry no index (§4.9.3, §4.9.4).
        index: u8,
        /// The 32-bit payload: a card status for R1 and R1b, the OCR for R3,
        /// the published address and an abbreviated status for R6, the echoed
        /// interface condition for R7.
        value: u32,
        /// R1b: the card holds DAT0 low afterwards until it is done.
        ///
        /// Always paired with [`SdCard::is_busy`], which in this model answers
        /// `false` immediately — see the module note on time.
        busy: bool,
    },
    /// A 136-bit response, R2: the CID or the CSD.
    ///
    /// The four words are the register's bits `[127:96]`, `[95:64]`,
    /// `[63:32]`, `[31:0]`, **including the trailing CRC7 and end bit**, which
    /// is what a controller's four response registers hold and what a driver
    /// that checks the CSD's CRC reads back.
    Long([u32; 4]),
}

/// What the card did with a data-phase request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
pub enum Data {
    /// Every byte was supplied, or accepted.
    Moved,
    /// The card has nothing more to give, or will take nothing more: the
    /// transfer finished, was never started, or was refused.
    ///
    /// Silence on the DAT lines, which a controller reports as a data timeout.
    Ended,
}

// ---------------------------------------------------------------------------
// The transfer in flight
// ---------------------------------------------------------------------------

/// A data transfer a command set up and the transport is now moving.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Transfer {
    /// Card to host.
    to_host: bool,
    /// A fixed payload — the `CMD6` switch status, the `ACMD13` SD status, the
    /// `ACMD51` SCR — rather than the array.
    payload: Option<Vec<u8>>,
    /// Byte address in the array of the block being moved.
    addr: u64,
    /// Bytes of the current block already moved.
    done: u32,
    /// The block length for this transfer.
    len: u32,
    /// Open ended: `CMD18` or `CMD25`, which run until `CMD12` or until a
    /// `CMD23` count runs out.
    multiple: bool,
    /// Blocks still to go, when `CMD23` fixed a count.
    left: Option<u32>,
    /// The block being assembled on a write. A card programs whole blocks, so
    /// a partial one is held here and never reaches the array.
    buf: Vec<u8>,
}

impl Transfer {
    fn payload(bytes: Vec<u8>) -> Transfer {
        let len = bytes.len() as u32;
        Transfer {
            to_host: true,
            payload: Some(bytes),
            addr: 0,
            done: 0,
            len,
            multiple: false,
            left: None,
            buf: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// The card
// ---------------------------------------------------------------------------

/// Everything that changes.
#[derive(Debug)]
struct Volatile {
    phase: Phase,
    /// The published relative card address. Zero until `CMD3`.
    rca: u16,
    /// What the next `CMD3` will publish.
    next_rca: u16,
    /// The read block length `CMD16` set.
    block_len: u32,
    /// The previous command was `CMD55`.
    app_cmd: bool,
    /// Error bits waiting to be reported in the next card status.
    sticky: u32,
    /// 1 or 4, from `ACMD6`.
    bus_width: u8,
    /// The `CMD6` group-1 access mode the host selected.
    access_mode: u8,
    /// A `CMD23` block count that has not been consumed yet.
    block_count: Option<u32>,
    /// The `CMD32` end of the erase range, as the card holds it: a block
    /// address on a high-capacity card, a byte address otherwise.
    erase_start: Option<u32>,
    /// The `CMD33` end of the same range.
    erase_end: Option<u32>,
    /// The transfer the last data command set up.
    transfer: Option<Transfer>,
}

impl Volatile {
    fn power_on(next_rca: u16) -> Volatile {
        Volatile {
            phase: Phase::Idle,
            rca: 0,
            next_rca,
            block_len: BLOCK as u32,
            app_cmd: false,
            sticky: 0,
            bus_width: 1,
            access_mode: 0,
            block_count: None,
            erase_start: None,
            erase_end: None,
            transfer: None,
        }
    }
}

/// The card's fixed identity and geometry: everything a power cycle does not
/// change.
#[derive(Debug, Clone)]
pub struct Identity {
    /// How many bytes the card holds.
    pub capacity: u64,
    /// SDHC/SDXC block addressing rather than SDSC byte addressing.
    pub high_capacity: bool,
    /// The card is mechanically write protected.
    pub read_only: bool,
    /// The CID register, most significant byte first.
    pub cid: [u8; 16],
    /// The CSD register, most significant byte first.
    pub csd: [u8; 16],
    /// The SCR register, most significant byte first.
    pub scr: [u8; 8],
}

impl Identity {
    /// Assemble the three registers a card is identified by.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the capacity cannot be expressed in the CSD the
    /// addressing mode implies.
    pub fn new(
        capacity: u64,
        high_capacity: bool,
        read_only: bool,
        text: IdentityText<'_>,
    ) -> Result<Identity> {
        if capacity == 0 || !capacity.is_multiple_of(BLOCK) {
            return Err(config(format!(
                "a card holds a whole number of {BLOCK}-byte blocks, and {capacity} is not one"
            )));
        }
        let csd = if high_capacity {
            csd_v2(capacity, read_only)?
        } else {
            csd_v1(capacity, read_only)?
        };
        Ok(Identity {
            capacity,
            high_capacity,
            read_only,
            cid: cid(text),
            csd,
            scr: scr(),
        })
    }

    /// How many 512-byte blocks the card holds.
    #[must_use]
    pub fn blocks(&self) -> u64 {
        self.capacity / BLOCK
    }
}

/// An SD memory card.
///
/// Construct it with [`SdCard::new`] from machine-description properties, or
/// with [`SdCard::with_identity`] directly.
pub struct SdCard {
    id: Identity,
    mode: BusMode,
    media: Arc<RamStore>,
    state: Mutex<Volatile>,
    /// The first address `CMD3` publishes, so a reset is reproducible.
    first_rca: u16,
}

impl fmt::Debug for SdCard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SdCard")
            .field("capacity", &self.id.capacity)
            .field("high_capacity", &self.id.high_capacity)
            .field("read_only", &self.id.read_only)
            .field("mode", &self.mode)
            .finish_non_exhaustive()
    }
}

impl SdCard {
    /// Validate `props` and allocate the card.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind;
    /// [`Error::Config`] if the capacity cannot be expressed in a CSD, or the
    /// bound image does not fit.
    pub fn new(props: &Props) -> Result<SdCard> {
        let mut r = props.reader();
        let capacity = r.require_size("size")?;
        let high_capacity = r.or("high-capacity", capacity > MAX_STANDARD_CAPACITY)?;
        let read_only = r.or("readonly", false)?;
        let manufacturer = r.or_range("manufacturer", 0x03u64, 0..=0xff)? as u8;
        let oem = r.or_str("oem", "RE")?.to_string();
        let product = r.or_str("product", "RSEMU")?.to_string();
        let revision = r.or_range("revision", 0x10u64, 0..=0xff)? as u8;
        let serial = r.or_range("serial", 1u64, 0..=0xffff_ffff)? as u32;
        let year = r.or_range("year", 2024u64, 2000..=2255)? as u16;
        let month = r.or_range("month", 1u64, 1..=12)? as u8;
        let rca = r.or_range("rca", 1u64, 1..=0xffff)? as u16;
        let mode = match r.or_enum("mode", "sd", &["sd", "spi"])? {
            "spi" => BusMode::Spi,
            _ => BusMode::Sd,
        };
        // Read by `CardDevice`, which owns the rendezvous; touched here so the
        // reader does not report it as unknown.
        let _ = r.optional_str("slot")?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        r.finish()?;

        let id = Identity::new(
            capacity,
            high_capacity,
            read_only,
            IdentityText {
                manufacturer,
                oem: &oem,
                product: &product,
                revision,
                serial,
                year,
                month,
            },
        )?;
        let card = SdCard::with_identity(id, mode, rca)?;
        if let Some(image) = image {
            if image.len() as u64 > capacity {
                return Err(config(format!(
                    "the bound image is {} byte(s) and the card holds {capacity}",
                    image.len()
                )));
            }
            card.load_image(0, &image)?;
        }
        Ok(card)
    }

    /// Build a card from an identity that has already been assembled.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the capacity does not fit in this host's memory.
    pub fn with_identity(id: Identity, mode: BusMode, first_rca: u16) -> Result<SdCard> {
        if usize::try_from(id.capacity).is_err() {
            return Err(config(format!(
                "a card of {} byte(s) is larger than this host's address space",
                id.capacity
            )));
        }
        let media = Arc::new(RamStore::new(id.capacity));
        Ok(SdCard {
            id,
            mode,
            media,
            state: Mutex::with_rank(LockRank::DEVICE, Volatile::power_on(first_rca)),
            first_rca,
        })
    }

    /// The card's fixed identity: capacity, addressing, CID, CSD and SCR.
    #[must_use]
    pub fn identity(&self) -> &Identity {
        &self.id
    }

    /// Which transport this card is behind.
    #[must_use]
    pub fn bus_mode(&self) -> BusMode {
        self.mode
    }

    /// Where the state machine currently is.
    ///
    /// Has no side effect: safe to call from a debugger.
    #[must_use]
    pub fn phase(&self) -> Phase {
        self.state.lock().phase
    }

    /// The address `CMD3` published, or zero.
    #[must_use]
    pub fn rca(&self) -> u16 {
        self.state.lock().rca
    }

    /// The data bus width `ACMD6` selected: 1 or 4.
    #[must_use]
    pub fn bus_width(&self) -> u8 {
        self.state.lock().bus_width
    }

    /// The card status **without** clearing anything.
    ///
    /// The debug-safe twin of what a `CMD13` response carries. §4.10.1 makes
    /// most error bits "clear by read", so the ordinary path clears them and a
    /// debugger or a test must not — the `MemAttrs::debug` rule (`ROADMAP.md`
    /// §15, invariant 5) applied one level below a register block.
    #[must_use]
    pub fn peek_status(&self) -> u32 {
        let state = self.state.lock();
        Self::status_of(&state)
    }

    /// Whether the card is holding DAT0 low.
    ///
    /// Always `false`: programming completes inside the call that delivers a
    /// block's last byte. The method exists so that a controller *asks* rather
    /// than assumes, which is what makes the busy window addable later without
    /// touching anything above this line. See the module note on time.
    #[must_use]
    pub fn is_busy(&self) -> bool {
        false
    }

    // -- the array ---------------------------------------------------------

    /// Read from the card's array, ignoring the protocol entirely.
    ///
    /// The debugger's and the host's door, not the guest's.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the card.
    pub fn read_media(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        self.media
            .read_at(offset, dst)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this card")))
    }

    /// Write to the card's array, ignoring the protocol entirely.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the card.
    pub fn write_media(&self, offset: u64, src: &[u8]) -> Result<()> {
        self.media
            .write_at(offset, src)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this card")))
    }

    /// Put `bytes` into the array at `offset`. The image loader's door.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image runs off the end of the card.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.media.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit in a card of {}",
                bytes.len(),
                self.id.capacity
            ))
        })
    }

    /// The whole array as a fresh vector, for a snapshot or a test.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; self.id.capacity as usize];
        // Cannot fail: the length is the store's own.
        let _ = self.media.read_at(0, &mut out);
        out
    }

    // -- power -------------------------------------------------------------

    /// Power the card down and up again.
    ///
    /// The **contents survive**, which is the whole point of the device; the
    /// state machine goes back to [`Phase::Idle`] and the published address,
    /// the bus width and the block length go back to their power-on values.
    /// This is what a controller's `POWER` register does, and what a board
    /// reset does to a card that is soldered to it.
    pub fn power_cycle(&self) {
        *self.state.lock() = Volatile::power_on(self.first_rca);
    }

    // -- the command path --------------------------------------------------

    /// Send one command and take the response.
    ///
    /// `index` is the six-bit command index and `arg` its 32-bit argument.
    /// Application commands — the ones written `ACMDn` — are just the command
    /// that follows a `CMD55`, and are recognised as such here rather than
    /// through a separate entry point, because on the wire they are not one.
    pub fn command(&self, index: u8, arg: u32) -> Reply {
        let mut state = self.state.lock();
        if state.phase == Phase::Inactive {
            // Off the bus until power cycles (§4.3). Even CMD0 is ignored.
            return Reply::None;
        }
        let app = state.app_cmd;
        state.app_cmd = false;
        if app {
            self.app_command(&mut state, index, arg)
        } else {
            self.basic_command(&mut state, index, arg)
        }
    }

    /// Take the next bytes of a card-to-host transfer.
    pub fn read_data(&self, dst: &mut [u8]) -> Data {
        let mut state = self.state.lock();
        let mut at = 0usize;
        while at < dst.len() {
            match state.transfer.as_ref() {
                Some(t) if t.to_host => {}
                _ => return Data::Ended,
            }
            if state.transfer.as_ref().is_some_and(|t| t.done == t.len)
                && !Self::next_block(&mut state)
            {
                return Data::Ended;
            }
            let Some(t) = state.transfer.as_mut() else {
                return Data::Ended;
            };
            let run = ((t.len - t.done) as usize).min(dst.len() - at);
            let ok = match &t.payload {
                Some(bytes) => {
                    let from = t.done as usize;
                    dst[at..at + run].copy_from_slice(&bytes[from..from + run]);
                    true
                }
                None => self
                    .media
                    .read_at(t.addr + u64::from(t.done), &mut dst[at..at + run])
                    .is_ok(),
            };
            if !ok {
                state.sticky |= OUT_OF_RANGE;
                state.transfer = None;
                state.phase = Phase::Transfer;
                return Data::Ended;
            }
            let t = state.transfer.as_mut().expect("still in flight");
            t.done += run as u32;
            at += run;
        }
        // A single-block read is over the moment its last byte leaves. A
        // multiple one waits for CMD12 or for its CMD23 count, and finding that
        // out is `next_block`'s job on the following call.
        Self::settle_after_read(&mut state);
        Data::Moved
    }

    /// Give the card the next bytes of a host-to-card transfer.
    pub fn write_data(&self, src: &[u8]) -> Data {
        let mut state = self.state.lock();
        let mut at = 0usize;
        while at < src.len() {
            match state.transfer.as_ref() {
                Some(t) if !t.to_host => {}
                _ => return Data::Ended,
            }
            if state.transfer.as_ref().is_some_and(|t| t.done == t.len)
                && !Self::next_block(&mut state)
            {
                return Data::Ended;
            }
            let Some(t) = state.transfer.as_mut() else {
                return Data::Ended;
            };
            let run = ((t.len as usize) - t.buf.len()).min(src.len() - at);
            t.buf.extend_from_slice(&src[at..at + run]);
            at += run;
            if t.buf.len() as u32 == t.len {
                let addr = t.addr;
                let block = core::mem::take(&mut t.buf);
                t.done = t.len;
                if !self.program(&mut state, addr, &block) {
                    return Data::Ended;
                }
                Self::settle_after_write(&mut state);
            }
        }
        Data::Moved
    }

    /// End whatever transfer is in flight, as a controller aborting its data
    /// path does.
    ///
    /// A partially received block is **dropped**, not programmed: a card
    /// programs whole blocks, and a half-written one never reaches the array.
    pub fn abort(&self) {
        let mut state = self.state.lock();
        state.transfer = None;
        if matches!(
            state.phase,
            Phase::SendingData | Phase::ReceiveData | Phase::Programming
        ) {
            state.phase = Phase::Transfer;
        }
    }

    // -- the state machine -------------------------------------------------

    fn basic_command(&self, state: &mut Volatile, index: u8, arg: u32) -> Reply {
        match index {
            cmd::GO_IDLE_STATE => {
                let rca = state.next_rca;
                *state = Volatile::power_on(rca);
                Reply::None
            }
            cmd::ALL_SEND_CID => {
                if state.phase != Phase::Ready {
                    return Self::illegal(state, index);
                }
                state.phase = Phase::Identification;
                Reply::Long(words_of(&self.id.cid))
            }
            cmd::SEND_RELATIVE_ADDR => {
                if !matches!(state.phase, Phase::Identification | Phase::Standby) {
                    return Self::illegal(state, index);
                }
                let status = Self::take_status(state);
                state.rca = state.next_rca;
                state.next_rca = state.next_rca.wrapping_add(1).max(1);
                state.phase = Phase::Standby;
                Reply::Short {
                    index,
                    value: (u32::from(state.rca) << 16) | r6_status(status),
                    busy: false,
                }
            }
            cmd::SWITCH_FUNC => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                let payload = Self::switch_status(state, arg);
                let status = Self::take_status(state);
                state.transfer = Some(Transfer::payload(payload));
                state.phase = Phase::SendingData;
                Reply::Short {
                    index,
                    value: status,
                    busy: false,
                }
            }
            cmd::SELECT_CARD => {
                let target = (arg >> 16) as u16;
                if target != state.rca || state.rca == 0 {
                    // Deselected. A card being deselected drives nothing
                    // (§4.7.4): the response belongs to whichever card was
                    // addressed, and that is not this one.
                    if matches!(state.phase, Phase::Transfer | Phase::Programming) {
                        state.phase = Phase::Standby;
                    }
                    return Reply::None;
                }
                let status = Self::take_status(state);
                match state.phase {
                    Phase::Standby => state.phase = Phase::Transfer,
                    Phase::Disconnect => state.phase = Phase::Programming,
                    _ => {}
                }
                Reply::Short {
                    index,
                    value: status,
                    busy: true,
                }
            }
            cmd::SEND_IF_COND => {
                if state.phase != Phase::Idle {
                    return Self::illegal(state, index);
                }
                // §4.3.13: the card echoes the check pattern only if it can
                // work at the supplied voltage. VHS 0001b is 2.7-3.6 V, which
                // is the only one this card accepts; anything else and it stays
                // quiet, which is how a host learns the card is unusable.
                let vhs = (arg >> 8) & 0xf;
                if vhs != 0x1 {
                    return Reply::None;
                }
                Reply::Short {
                    index,
                    value: arg & 0xfff,
                    busy: false,
                }
            }
            cmd::SEND_CSD | cmd::SEND_CID => {
                let target = (arg >> 16) as u16;
                if state.phase != Phase::Standby || target != state.rca {
                    return Reply::None;
                }
                let register = if index == cmd::SEND_CSD {
                    &self.id.csd
                } else {
                    &self.id.cid
                };
                Reply::Long(words_of(register))
            }
            cmd::STOP_TRANSMISSION => {
                if !matches!(
                    state.phase,
                    Phase::SendingData | Phase::ReceiveData | Phase::Programming
                ) {
                    return Self::illegal(state, index);
                }
                let status = Self::take_status(state);
                // A partial block in the receive buffer is discarded, as it is
                // on real silicon: the card programs blocks, not bytes.
                state.transfer = None;
                state.phase = Phase::Transfer;
                Reply::Short {
                    index,
                    value: status,
                    busy: true,
                }
            }
            cmd::SEND_STATUS => {
                let target = (arg >> 16) as u16;
                if self.mode == BusMode::Sd && (target != state.rca || state.rca == 0) {
                    return Reply::None;
                }
                Reply::Short {
                    index,
                    value: Self::take_status(state),
                    busy: false,
                }
            }
            cmd::GO_INACTIVE_STATE => {
                let target = (arg >> 16) as u16;
                if target == state.rca && state.rca != 0 {
                    state.phase = Phase::Inactive;
                }
                Reply::None
            }
            cmd::SET_BLOCKLEN => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                // §5.3.3: a high-capacity card's block length is fixed at 512
                // and CMD16 may only confirm it. A standard-capacity card has
                // READ_BL_PARTIAL set, so a shorter read length is legal — and
                // WRITE_BL_PARTIAL is clear, so a shorter *write* is not, which
                // `start_media_transfer` enforces where it belongs.
                let ok = if self.id.high_capacity {
                    arg == BLOCK as u32
                } else {
                    arg >= 1 && u64::from(arg) <= BLOCK
                };
                if ok {
                    state.block_len = arg;
                } else {
                    state.sticky |= BLOCK_LEN_ERROR;
                }
                Reply::Short {
                    index,
                    value: Self::take_status(state),
                    busy: false,
                }
            }
            cmd::READ_SINGLE_BLOCK | cmd::READ_MULTIPLE_BLOCK => {
                self.start_media_transfer(state, index, arg, true)
            }
            cmd::SET_BLOCK_COUNT => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                state.block_count = Some(arg);
                Reply::Short {
                    index,
                    value: Self::take_status(state),
                    busy: false,
                }
            }
            cmd::WRITE_BLOCK | cmd::WRITE_MULTIPLE_BLOCK => {
                self.start_media_transfer(state, index, arg, false)
            }
            cmd::ERASE_WR_BLK_START | cmd::ERASE_WR_BLK_END => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                if index == cmd::ERASE_WR_BLK_START {
                    state.erase_start = Some(arg);
                    state.erase_end = None;
                } else if state.erase_start.is_some() {
                    state.erase_end = Some(arg);
                } else {
                    state.sticky |= ERASE_SEQ_ERROR;
                }
                Reply::Short {
                    index,
                    value: Self::take_status(state),
                    busy: false,
                }
            }
            cmd::ERASE => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                self.erase(state);
                Reply::Short {
                    index,
                    value: Self::take_status(state),
                    busy: true,
                }
            }
            cmd::APP_CMD => {
                let target = (arg >> 16) as u16;
                let addressed = match self.mode {
                    // Before CMD3 the card has no address and CMD55 carries
                    // zero; afterwards it must be addressed to us.
                    BusMode::Sd if state.rca != 0 => target == state.rca,
                    BusMode::Sd => target == 0,
                    BusMode::Spi => true,
                };
                if !addressed {
                    return Reply::None;
                }
                state.app_cmd = true;
                let value = Self::take_status(state) | APP_CMD;
                Reply::Short {
                    index,
                    value,
                    busy: false,
                }
            }
            _ => Self::illegal(state, index),
        }
    }

    fn app_command(&self, state: &mut Volatile, index: u8, arg: u32) -> Reply {
        match index {
            cmd::A_SD_SEND_OP_COND => {
                if state.phase != Phase::Idle {
                    return Self::illegal(state, index);
                }
                // §4.2.3. A zero voltage window is an *inquiry*: the host is
                // asking what the card wants, and the card must not begin
                // initialising.
                let window = arg & 0x00ff_8000;
                if window == 0 {
                    return Reply::Short {
                        index: 0x3f,
                        value: self.ocr(false),
                        busy: false,
                    };
                }
                // HCS, bit 30. A host that does not set it cannot address a
                // high-capacity card, and §4.2.3 says such a card rejects the
                // host by going inactive rather than pretending.
                if self.id.high_capacity && arg & (1 << 30) == 0 {
                    state.phase = Phase::Inactive;
                    return Reply::None;
                }
                // Ready on the first poll: the initialisation window is time,
                // and this model has none. A host's `while (!(ocr & BUSY))`
                // loop terminates on its first iteration.
                state.phase = match self.mode {
                    BusMode::Sd => Phase::Ready,
                    // SPI has no bus addressing, so there is no CMD2/CMD3/CMD7
                    // sequence to walk and an initialised card is simply
                    // available (Physical Layer §7.2.1).
                    BusMode::Spi => Phase::Transfer,
                };
                Reply::Short {
                    index: 0x3f,
                    value: self.ocr(true),
                    busy: false,
                }
            }
            cmd::A_SET_BUS_WIDTH => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                match arg & 0x3 {
                    0b00 => state.bus_width = 1,
                    0b10 => state.bus_width = 4,
                    _ => state.sticky |= OUT_OF_RANGE,
                }
                Reply::Short {
                    index,
                    value: Self::take_status(state) | APP_CMD,
                    busy: false,
                }
            }
            cmd::A_SD_STATUS => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                let payload = Self::sd_status(state);
                let value = Self::take_status(state) | APP_CMD;
                state.transfer = Some(Transfer::payload(payload));
                state.phase = Phase::SendingData;
                Reply::Short {
                    index,
                    value,
                    busy: false,
                }
            }
            cmd::A_SEND_SCR => {
                if state.phase != Phase::Transfer {
                    return Self::illegal(state, index);
                }
                let value = Self::take_status(state) | APP_CMD;
                state.transfer = Some(Transfer::payload(self.id.scr.to_vec()));
                state.phase = Phase::SendingData;
                Reply::Short {
                    index,
                    value,
                    busy: false,
                }
            }
            // Not an application command this card knows. §4.3.9: the card
            // treats it as the ordinary command of the same index.
            _ => self.basic_command(state, index, arg),
        }
    }

    /// Set up a `CMD17`/`CMD18`/`CMD24`/`CMD25` transfer.
    fn start_media_transfer(
        &self,
        state: &mut Volatile,
        index: u8,
        arg: u32,
        to_host: bool,
    ) -> Reply {
        if state.phase != Phase::Transfer {
            return Self::illegal(state, index);
        }
        let multiple = index == cmd::READ_MULTIPLE_BLOCK || index == cmd::WRITE_MULTIPLE_BLOCK;
        // The difference the specification spends §4.3.14 on: a high-capacity
        // card's argument counts 512-byte blocks, a standard-capacity card's
        // counts bytes. Getting this backwards reads the right data from the
        // wrong place for the first 512 sectors and then quietly diverges.
        let addr = if self.id.high_capacity {
            u64::from(arg) * BLOCK
        } else {
            u64::from(arg)
        };
        // WRITE_BL_PARTIAL is clear (§5.3.2), so a write is always a whole
        // block; READ_BL_PARTIAL is set, so a shorter read is legal.
        let len = if to_host {
            state.block_len
        } else {
            BLOCK as u32
        };
        let refuse = |state: &mut Volatile, bit: u32| -> Reply {
            state.sticky |= bit;
            Reply::Short {
                index,
                value: Self::take_status(state),
                busy: false,
            }
        };
        if !to_host && u64::from(state.block_len) != BLOCK {
            return refuse(state, BLOCK_LEN_ERROR);
        }
        if addr >= self.id.capacity || addr + u64::from(len) > self.id.capacity {
            return refuse(state, OUT_OF_RANGE);
        }
        // READ_BLK_MISALIGN and WRITE_BLK_MISALIGN are both clear, so a
        // transfer may not straddle a physical block boundary.
        if addr / BLOCK != (addr + u64::from(len) - 1) / BLOCK {
            return refuse(state, ADDRESS_ERROR);
        }
        if !to_host && self.id.read_only {
            return refuse(state, WP_VIOLATION);
        }
        // `left` counts the blocks *after* this one, so a CMD23 of two means
        // one more once the first has moved. A count of zero is not a
        // zero-block transfer, it is no count at all.
        let left = if multiple {
            state.block_count.take().filter(|n| *n != 0).map(|n| n - 1)
        } else {
            state.block_count = None;
            None
        };
        let value = Self::take_status(state);
        state.transfer = Some(Transfer {
            to_host,
            payload: None,
            addr,
            done: 0,
            len,
            multiple,
            left,
            buf: Vec::new(),
        });
        state.phase = if to_host {
            Phase::SendingData
        } else {
            Phase::ReceiveData
        };
        Reply::Short {
            index,
            value,
            busy: false,
        }
    }

    /// Move to the next block of an open-ended transfer, or end it.
    ///
    /// Returns whether there is a block to move.
    fn next_block(state: &mut Volatile) -> bool {
        let Some(t) = state.transfer.as_mut() else {
            return false;
        };
        let more = t.multiple && t.payload.is_none() && t.left != Some(0);
        if !more {
            state.transfer = None;
            state.phase = Phase::Transfer;
            return false;
        }
        t.addr += u64::from(t.len);
        t.done = 0;
        t.buf.clear();
        if let Some(left) = t.left.as_mut() {
            *left -= 1;
        }
        true
    }

    /// A read that has just delivered its last requested byte.
    fn settle_after_read(state: &mut Volatile) {
        let Some(t) = state.transfer.as_ref() else {
            return;
        };
        if t.done < t.len {
            return;
        }
        // A single block, or the last block of a counted multiple, is the end
        // of the transfer and the card returns to `tran` without a CMD12.
        if !t.multiple || t.left == Some(0) {
            state.transfer = None;
            state.phase = Phase::Transfer;
        }
    }

    /// A write whose block has just been programmed.
    fn settle_after_write(state: &mut Volatile) {
        let Some(t) = state.transfer.as_ref() else {
            return;
        };
        if !t.multiple || t.left == Some(0) {
            state.transfer = None;
            // Programming is instantaneous, so `prg` is passed through rather
            // than rested in. See the module note on time.
            state.phase = Phase::Transfer;
        }
    }

    /// Program one whole block. Returns whether the transfer may continue.
    fn program(&self, state: &mut Volatile, addr: u64, block: &[u8]) -> bool {
        let fail = |state: &mut Volatile, bit: u32| {
            state.sticky |= bit;
            state.transfer = None;
            state.phase = Phase::Transfer;
            false
        };
        if self.id.read_only {
            return fail(state, WP_VIOLATION);
        }
        if self.media.write_at(addr, block).is_err() {
            return fail(state, OUT_OF_RANGE);
        }
        true
    }

    fn erase(&self, state: &mut Volatile) {
        let (Some(start), Some(end)) = (state.erase_start, state.erase_end) else {
            state.sticky |= ERASE_SEQ_ERROR;
            return;
        };
        state.erase_start = None;
        state.erase_end = None;
        if self.id.read_only {
            state.sticky |= WP_VIOLATION;
            return;
        }
        let unit = if self.id.high_capacity { BLOCK } else { 1 };
        let from = u64::from(start) * unit;
        let to = u64::from(end) * unit + BLOCK;
        if from > to || to > self.id.capacity {
            state.sticky |= ERASE_PARAM;
            return;
        }
        // SCR's DATA_STAT_AFTER_ERASE is clear, so erased blocks read as zero.
        if self.media.fill(from, to - from, 0).is_err() {
            state.sticky |= ERASE_PARAM;
        }
    }

    fn illegal(state: &mut Volatile, index: u8) -> Reply {
        state.sticky |= ILLEGAL_COMMAND;
        let value = Self::take_status(state);
        Reply::Short {
            index,
            value,
            busy: false,
        }
    }

    /// The card status as it stands, with nothing cleared.
    fn status_of(state: &Volatile) -> u32 {
        let mut status = state.sticky | (state.phase.code() << STATE_SHIFT) | READY_FOR_DATA;
        if state.sticky & CLEAR_ON_READ != 0 {
            status |= CARD_ERROR;
        }
        if state.app_cmd {
            status |= APP_CMD;
        }
        status
    }

    /// The card status for a response, clearing what §4.10.1 says a report
    /// clears.
    fn take_status(state: &mut Volatile) -> u32 {
        let status = Self::status_of(state);
        state.sticky &= !CLEAR_ON_READ;
        status
    }

    fn ocr(&self, ready: bool) -> u32 {
        // §5.1. Bits 23:15 are the 2.7-3.6 V window this card works in; bit 31
        // says the power-up sequence has finished and bit 30 — only meaningful
        // once it has — is the card capacity status.
        let mut ocr = 0x00ff_8000;
        if ready {
            ocr |= 1 << 31;
            if self.id.high_capacity {
                ocr |= 1 << 30;
            }
        }
        ocr
    }

    /// The 64-byte `CMD6` switch-function status (§4.3.10.4).
    fn switch_status(state: &mut Volatile, arg: u32) -> Vec<u8> {
        let mut out = alloc::vec![0u8; 64];
        // Maximum current at 3.3 V, in mA.
        out[0] = 0x00;
        out[1] = 0x64;
        // Support bitmaps, groups 6 down to 1, two bytes each. Only group 1
        // (access mode) offers anything beyond the default: bit 0 is the
        // 25 MHz default and bit 1 is 50 MHz high speed.
        for group in 0..6usize {
            let support: u16 = if group == 5 { 0x0003 } else { 0x0001 };
            out[2 + group * 2] = (support >> 8) as u8;
            out[3 + group * 2] = support as u8;
        }
        // The function selected in each group, one nibble each, groups 6..1.
        let switching = arg & (1 << 31) != 0;
        let mut selected = [0xfu8; 6];
        for (group, slot) in selected.iter_mut().enumerate() {
            let want = ((arg >> (group * 4)) & 0xf) as u8;
            let support: u16 = if group == 0 { 0x0003 } else { 0x0001 };
            *slot = if want == 0xf {
                // "no change": the card reports what is already in force.
                if group == 0 { state.access_mode } else { 0 }
            } else if support & (1u16 << want) != 0 {
                if switching && group == 0 {
                    state.access_mode = want;
                }
                want
            } else {
                // Not supported: §4.3.10.4 says the card answers 0xf.
                0xf
            };
        }
        out[14] = (selected[5] << 4) | selected[4];
        out[15] = (selected[3] << 4) | selected[2];
        out[16] = (selected[1] << 4) | selected[0];
        // Data structure version 1: the busy-status fields below are present.
        out[17] = 0x01;
        out
    }

    /// The 64-byte `ACMD13` SD status (§4.10.2).
    fn sd_status(state: &Volatile) -> Vec<u8> {
        let mut out = alloc::vec![0u8; 64];
        // DAT_BUS_WIDTH, bits 511:510: 00 is one bit, 10 is four.
        out[0] = if state.bus_width == 4 { 0b10 << 6 } else { 0 };
        // SD_CARD_TYPE, bits 495:480: zero is a regular read/write card.
        // SPEED_CLASS, bits 447:440: 4 is class 10.
        out[8] = 0x04;
        // PERFORMANCE_MOVE, bits 439:432, in MB/s. 0xff means "always enough".
        out[9] = 0xff;
        // AU_SIZE, bits 431:428: 9 is 4 MiB.
        out[10] = 0x90;
        out
    }

    // -- snapshots ---------------------------------------------------------

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The array, exactly as `dev-flash-cfi` saves its own: a card's
        // contents are guest-visible state, and a snapshot that restored to
        // different bytes would be a snapshot of a different machine.
        w.write_bytes(&self.contents())?;
        let state = self.state.lock();
        w.write_u8(state.phase as u8)?;
        w.write_u16(state.rca)?;
        w.write_u16(state.next_rca)?;
        w.write_u32(state.block_len)?;
        w.write_bool(state.app_cmd)?;
        w.write_u32(state.sticky)?;
        w.write_u8(state.bus_width)?;
        w.write_u8(state.access_mode)?;
        write_option_u32(w, state.block_count)?;
        write_option_u32(w, state.erase_start)?;
        write_option_u32(w, state.erase_end)?;
        match &state.transfer {
            None => w.write_bool(false)?,
            Some(t) => {
                w.write_bool(true)?;
                w.write_bool(t.to_host)?;
                match &t.payload {
                    Some(bytes) => {
                        w.write_bool(true)?;
                        w.write_bytes(bytes)?;
                    }
                    None => w.write_bool(false)?,
                }
                w.write_u64(t.addr)?;
                w.write_u32(t.done)?;
                w.write_u32(t.len)?;
                w.write_bool(t.multiple)?;
                write_option_u32(w, t.left)?;
                w.write_bytes(&t.buf)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        if bytes.len() as u64 != self.id.capacity {
            return Err(Error::State(format!(
                "the snapshot holds a card of {} byte(s), this one holds {}",
                bytes.len(),
                self.id.capacity
            )));
        }
        self.media
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the card refused the snapshot")))?;
        let phase = Phase::from_code(r.read_u8()?)?;
        let rca = r.read_u16()?;
        let next_rca = r.read_u16()?;
        let block_len = r.read_u32()?;
        let app_cmd = r.read_bool()?;
        let sticky = r.read_u32()?;
        let bus_width = r.read_u8()?;
        let access_mode = r.read_u8()?;
        let block_count = read_option_u32(r)?;
        let erase_start = read_option_u32(r)?;
        let erase_end = read_option_u32(r)?;
        let transfer = if r.read_bool()? {
            let to_host = r.read_bool()?;
            let payload = if r.read_bool()? {
                Some(r.read_bytes()?.to_vec())
            } else {
                None
            };
            let addr = r.read_u64()?;
            let done = r.read_u32()?;
            let len = r.read_u32()?;
            let multiple = r.read_bool()?;
            let left = read_option_u32(r)?;
            let buf = r.read_bytes()?.to_vec();
            if len == 0 || u64::from(len) > BLOCK || done > len || buf.len() as u32 > len {
                return Err(Error::State(format!(
                    "a snapshot transfer of {done}/{len} byte(s) is not one this card can hold"
                )));
            }
            Some(Transfer {
                to_host,
                payload,
                addr,
                done,
                len,
                multiple,
                left,
                buf,
            })
        } else {
            None
        };
        if block_len == 0 || u64::from(block_len) > BLOCK {
            return Err(Error::State(format!(
                "{block_len} is not a block length an SD card can hold"
            )));
        }
        if bus_width != 1 && bus_width != 4 {
            return Err(Error::State(format!(
                "{bus_width} is not an SD data bus width"
            )));
        }
        *self.state.lock() = Volatile {
            phase,
            rca,
            next_rca,
            block_len,
            app_cmd,
            sticky,
            bus_width,
            access_mode,
            block_count,
            erase_start,
            erase_end,
            transfer,
        };
        Ok(())
    }
}

fn write_option_u32(w: &mut ChunkWriter<'_>, value: Option<u32>) -> Result<()> {
    match value {
        Some(v) => {
            w.write_bool(true)?;
            w.write_u32(v)
        }
        None => w.write_bool(false),
    }
}

fn read_option_u32(r: &mut ChunkReader<'_>) -> Result<Option<u32>> {
    if r.read_bool()? {
        Ok(Some(r.read_u32()?))
    } else {
        Ok(None)
    }
}

/// The abbreviated status an R6 carries: card status bits 23, 22 and 19, then
/// bits 12:0 (§4.9.4).
fn r6_status(status: u32) -> u32 {
    (((status >> 23) & 1) << 15)
        | (((status >> 22) & 1) << 14)
        | (((status >> 19) & 1) << 13)
        | (status & 0x1fff)
}

/// A 128-bit register as the four words a long response is read out of, most
/// significant first.
fn words_of(register: &[u8; 16]) -> [u32; 4] {
    let mut out = [0u32; 4];
    for (i, word) in out.iter_mut().enumerate() {
        let b = &register[i * 4..i * 4 + 4];
        *word = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    }
    out
}

/// CRC7 over `bytes`: the polynomial `x^7 + x^3 + 1` the specification uses for
/// commands, responses and the CID/CSD registers (§4.5).
#[must_use]
pub fn crc7(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for byte in bytes {
        let mut b = *byte;
        for _ in 0..8 {
            let bit = (b & 0x80) >> 7;
            b <<= 1;
            let top = (crc >> 6) & 1;
            crc = (crc << 1) & 0x7f;
            if top ^ bit != 0 {
                crc ^= 0x09;
            }
        }
    }
    crc & 0x7f
}

// ---------------------------------------------------------------------------
// The registers
// ---------------------------------------------------------------------------

/// The text and numbers that go into a CID.
#[derive(Debug, Clone, Copy)]
pub struct IdentityText<'a> {
    /// JEDEC manufacturer identifier, `MID`.
    pub manufacturer: u8,
    /// Two ASCII characters, `OID`. Padded or truncated to two.
    pub oem: &'a str,
    /// Five ASCII characters, `PNM`. Padded or truncated to five.
    pub product: &'a str,
    /// Product revision, `PRV`.
    pub revision: u8,
    /// Serial number, `PSN`.
    pub serial: u32,
    /// Manufacturing year, 2000 to 2255.
    pub year: u16,
    /// Manufacturing month, 1 to 12.
    pub month: u8,
}

/// Build a CID (§5.2).
fn cid(text: IdentityText<'_>) -> [u8; 16] {
    let mut cid = [0u8; 16];
    cid[0] = text.manufacturer;
    fixed_ascii(&mut cid[1..3], text.oem);
    fixed_ascii(&mut cid[3..8], text.product);
    cid[8] = text.revision;
    cid[9..13].copy_from_slice(&text.serial.to_be_bytes());
    // MDT occupies bits 19:8: eight bits of year offset from 2000, then four
    // of month. Bits 23:20 are reserved and stay zero.
    let year = text.year.saturating_sub(2000) as u8;
    cid[13] = year >> 4;
    cid[14] = ((year & 0x0f) << 4) | (text.month & 0x0f);
    cid[15] = (crc7(&cid[..15]) << 1) | 1;
    cid
}

/// Build a version 1.0 CSD, the standard-capacity one (§5.3.2).
fn csd_v1(capacity: u64, read_only: bool) -> Result<[u8; 16]> {
    if capacity > MAX_STANDARD_CAPACITY {
        return Err(config(format!(
            "{capacity} byte(s) is past the {MAX_STANDARD_CAPACITY} a standard-capacity CSD can \
             describe; set `high-capacity = true`"
        )));
    }
    // capacity = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN, with
    // READ_BL_LEN fixed at 9 here. C_SIZE is twelve bits and C_SIZE_MULT three,
    // so the smallest multiplier that brings the block count inside 4096 is the
    // one to use.
    let blocks = capacity / BLOCK;
    let mut mult = 0u32;
    while mult < 8 && blocks >> (mult + 2) > 4096 {
        mult += 1;
    }
    let unit = 1u64 << (mult + 2);
    if mult == 8 || !blocks.is_multiple_of(unit) || blocks / unit == 0 {
        return Err(config(format!(
            "{capacity} byte(s) is not a size a standard-capacity CSD can express exactly; a \
             multiple of {} would be",
            unit * BLOCK
        )));
    }
    let c_size = (blocks / unit - 1) as u32;
    let mut csd = [0u8; 16];
    csd[0] = 0x00; // CSD_STRUCTURE = 0, version 1.0
    csd[1] = 0x26; // TAAC: 1.5 ms
    csd[2] = 0x00; // NSAC
    csd[3] = 0x32; // TRAN_SPEED: 25 MHz
    csd[4] = 0x5b; // CCC 0x5b5: classes 0, 2, 4, 5, 7, 8 and 10
    csd[5] = 0x59; // the rest of CCC, then READ_BL_LEN = 9
    // READ_BL_PARTIAL is set — mandatory for a standard-capacity card — and
    // both misalign bits and DSR_IMP are clear.
    csd[6] = 0x80 | ((c_size >> 10) & 0x03) as u8;
    csd[7] = ((c_size >> 2) & 0xff) as u8;
    // C_SIZE's low two bits, then the two read-current fields, both 0b111.
    csd[8] = (((c_size & 0x3) as u8) << 6) | 0x3f;
    // The two write-current fields, then C_SIZE_MULT's top two bits.
    csd[9] = 0xfc | ((mult >> 1) & 0x3) as u8;
    // C_SIZE_MULT's low bit, ERASE_BLK_EN, then SECTOR_SIZE 0x7f.
    csd[10] = (((mult & 1) as u8) << 7) | 0x40 | (0x7f >> 1);
    csd[11] = 0x80; // SECTOR_SIZE's low bit, then WP_GRP_SIZE = 0
    csd[12] = 0x0a; // R2W_FACTOR = 2, then WRITE_BL_LEN = 9
    csd[13] = 0x40; // WRITE_BL_LEN's low bit; WRITE_BL_PARTIAL clear
    csd[14] = if read_only { 0x20 } else { 0x00 };
    csd[15] = (crc7(&csd[..15]) << 1) | 1;
    Ok(csd)
}

/// Build a version 2.0 CSD, the high-capacity one (§5.3.3).
fn csd_v2(capacity: u64, read_only: bool) -> Result<[u8; 16]> {
    if !capacity.is_multiple_of(HIGH_CAPACITY_UNIT) {
        return Err(config(format!(
            "a high-capacity card's C_SIZE counts {HIGH_CAPACITY_UNIT}-byte units, and \
             {capacity} is not a multiple of one"
        )));
    }
    let units = capacity / HIGH_CAPACITY_UNIT;
    if units - 1 > 0x3f_ffff {
        return Err(config(format!(
            "{capacity} byte(s) is past what a version 2.0 CSD's 22-bit C_SIZE can describe"
        )));
    }
    let c_size = (units - 1) as u32;
    let mut csd = [0u8; 16];
    csd[0] = 0x40; // CSD_STRUCTURE = 1, version 2.0
    csd[1] = 0x0e; // TAAC is fixed at 0x0e in version 2.0
    csd[2] = 0x00; // NSAC is fixed at zero
    csd[3] = 0x32; // TRAN_SPEED: 25 MHz
    csd[4] = 0x5b; // CCC 0x5b5
    csd[5] = 0x59; // READ_BL_LEN is fixed at 9
    csd[6] = 0x00; // READ_BL_PARTIAL, both misalign bits and DSR_IMP are clear
    csd[7] = ((c_size >> 16) & 0x3f) as u8;
    csd[8] = ((c_size >> 8) & 0xff) as u8;
    csd[9] = (c_size & 0xff) as u8;
    csd[10] = 0x40 | (0x7f >> 1); // ERASE_BLK_EN, then SECTOR_SIZE 0x7f
    csd[11] = 0x80; // SECTOR_SIZE's low bit, then WP_GRP_SIZE = 0
    csd[12] = 0x0a; // R2W_FACTOR = 2, then WRITE_BL_LEN = 9
    csd[13] = 0x40; // WRITE_BL_PARTIAL clear
    csd[14] = if read_only { 0x20 } else { 0x00 };
    csd[15] = (crc7(&csd[..15]) << 1) | 1;
    Ok(csd)
}

/// Build the SCR (§5.6).
fn scr() -> [u8; 8] {
    let mut scr = [0u8; 8];
    // SCR_STRUCTURE = 0, SD_SPEC = 2: the Physical Layer 2.00 command set.
    scr[0] = 0x02;
    // DATA_STAT_AFTER_ERASE clear (erased blocks read zero), SD_SECURITY = 0,
    // SD_BUS_WIDTHS = 0b0101: one bit and four bits.
    scr[1] = 0x05;
    // SD_SPEC3 set, so the version is at least 3.0x.
    scr[2] = 0x80;
    // CMD_SUPPORT bit 1: CMD23, set block count. This card has no CMD20 speed
    // class control and no CMD48/49 or CMD58/59.
    scr[3] = 0x02;
    scr
}

/// Copy `text` into `out` as ASCII, padding with spaces and truncating.
fn fixed_ascii(out: &mut [u8], text: &str) {
    out.fill(b' ');
    for (slot, byte) in out.iter_mut().zip(text.bytes()) {
        *slot = if byte.is_ascii_graphic() { byte } else { b'?' };
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

/// The `sd.card` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an SD memory card: the command set, the state machine and the registers",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes the card holds, as in `size = 64M`",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the initial contents; the rest reads zero",
        },
        PropertySpec {
            name: "slot",
            kind: ValueKind::Str,
            required: false,
            summary: "the named card slot this card sits in (default `sd0`)",
        },
        PropertySpec {
            name: "high-capacity",
            kind: ValueKind::Bool,
            required: false,
            summary: "SDHC block addressing rather than SDSC byte addressing (default: by size)",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "the mechanical write-protect tab: a write fails with WP_VIOLATION",
        },
        PropertySpec {
            name: "mode",
            kind: ValueKind::Str,
            required: false,
            summary: "`sd` (the default) or `spi`, which has no bus addressing",
        },
        PropertySpec {
            name: "manufacturer",
            kind: ValueKind::Uint,
            required: false,
            summary: "the CID's MID field",
        },
        PropertySpec {
            name: "oem",
            kind: ValueKind::Str,
            required: false,
            summary: "the CID's two-character OID field",
        },
        PropertySpec {
            name: "product",
            kind: ValueKind::Str,
            required: false,
            summary: "the CID's five-character PNM field",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the CID's PRV field",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Uint,
            required: false,
            summary: "the CID's PSN field; a constant, because a run must be reproducible",
        },
        PropertySpec {
            name: "year",
            kind: ValueKind::Uint,
            required: false,
            summary: "the CID's manufacturing year, 2000 to 2255",
        },
        PropertySpec {
            name: "month",
            kind: ValueKind::Uint,
            required: false,
            summary: "the CID's manufacturing month, 1 to 12",
        },
        PropertySpec {
            name: "rca",
            kind: ValueKind::Uint,
            required: false,
            summary: "the address the first CMD3 publishes (default 1)",
        },
    ],
    construct: |props| Ok(Box::new(CardDevice::new(props)?)),
};

/// An [`SdCard`] as a machine-description object.
///
/// The card itself is not a memory-mapped device: it has no region, no pins and
/// no clock. What this wrapper adds is the two-phase construction contract and
/// the rendezvous — it puts the card in a named slot at construction, and a
/// controller in the same build picks it up from there.
#[derive(Debug)]
pub struct CardDevice {
    card: Arc<SdCard>,
    slot: String,
}

impl CardDevice {
    /// Validate `props`, build the card, and put it in its slot.
    ///
    /// # Errors
    ///
    /// As [`SdCard::new`], plus [`Error::Config`] if the named slot already
    /// holds a card.
    pub fn new(props: &Props) -> Result<CardDevice> {
        let slot = props
            .get("slot")
            .and_then(crate::core::props::Value::as_str)
            .unwrap_or(super::DEFAULT_SLOT)
            .to_string();
        let card = Arc::new(SdCard::new(props)?);
        let holder = super::slots::attach(props, &slot)?;
        holder.insert(Arc::clone(&card)).map_err(|_| {
            config(format!(
                "two cards were put in the slot called `{slot}`; give one of them another `slot`"
            ))
        })?;
        Ok(CardDevice { card, slot })
    }

    /// The card behind this object.
    #[must_use]
    pub fn card(&self) -> &Arc<SdCard> {
        &self.card
    }

    /// The slot it was put in.
    #[must_use]
    pub fn slot(&self) -> &str {
        &self.slot
    }
}

impl Device for CardDevice {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. The slot was claimed at construction, which is
        // allocation rather than an observable action — the same argument
        // `core::hosts` makes for every other rendezvous.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. A board reset cycles the card's power, which resets the
        // protocol state and leaves the contents alone — the same distinction
        // NOR flash draws, and for the same reason.
        self.card.power_cycle();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.card.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.card.load(r)
    }
}

impl Instance for CardDevice {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(CardDevice::new(props)?)))
}

/// What the validator should know about `sd.card`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("slot", ValueKind::Str))
        .prop(PropSchema::new("high-capacity", ValueKind::Bool))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("mode", ValueKind::Str).values(&["sd", "spi"]))
        .prop(PropSchema::new("manufacturer", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("oem", ValueKind::Str))
        .prop(PropSchema::new("product", ValueKind::Str))
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("serial", ValueKind::Uint).range(0, 0xffff_ffff))
        .prop(PropSchema::new("year", ValueKind::Uint).range(2000, 2255))
        .prop(PropSchema::new("month", ValueKind::Uint).range(1, 12))
        .prop(PropSchema::new("rca", ValueKind::Uint).range(1, 0xffff))
}

#[cfg(test)]
mod tests;
