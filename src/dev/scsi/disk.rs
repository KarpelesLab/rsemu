//! A SCSI direct-access device: a hard disk on the cable.
//!
//! The target half of the split [`super`] argues for. This file is a phase
//! machine and a command set and **contains no controller register** — it does
//! not know what a WD33C93A is, and the same object would hang off an NCR
//! 53C94 or an Adaptec card unchanged.
//!
//! # What it implements
//!
//! The direct-access command set an operating system of 1990 actually issues
//! (X3.131-1994 §8 for the commands every device type has, §9 for this one's):
//!
//! | Opcode | Command | Clause |
//! | --- | --- | --- |
//! | `00` | `TEST UNIT READY` | §8.2.16 |
//! | `03` | `REQUEST SENSE`, fixed format | §8.2.14 |
//! | `08` | `READ(6)` | §9.2.5 |
//! | `0A` | `WRITE(6)` | §9.2.16 |
//! | `0B` | `SEEK(6)` | §9.2.14 |
//! | `12` | `INQUIRY`, standard data and pages `00`/`80` | §8.2.5 |
//! | `15` | `MODE SELECT(6)` — parsed and ignored | §8.2.8 |
//! | `16`, `17` | `RESERVE`, `RELEASE` | §9.2.12, §9.2.11 |
//! | `1A` | `MODE SENSE(6)`, pages `01`, `03`, `04`, `3F` | §8.2.10, §9.3.3 |
//! | `1B` | `START STOP UNIT` | §9.2.15 |
//! | `1D` | `SEND DIAGNOSTIC` | §8.2.15 |
//! | `1E` | `PREVENT ALLOW MEDIUM REMOVAL` | §9.2.4 |
//! | `25` | `READ CAPACITY` | §9.2.7 |
//! | `28` | `READ(10)` | §9.2.6 |
//! | `2A` | `WRITE(10)` | §9.2.17 |
//! | `2B` | `SEEK(10)` | §9.2.14 |
//! | `2F` | `VERIFY(10)`, `BytChk` = 0 | §9.2.19 |
//! | `35` | `SYNCHRONIZE CACHE` | §9.2.18 |
//! | `37` | `READ DEFECT DATA(10)`, an empty list | §9.2.8 |
//! | `55` | `MODE SELECT(10)` — parsed and ignored | §8.2.9 |
//! | `5A` | `MODE SENSE(10)` | §8.2.11 |
//!
//! Anything else is `CHECK CONDITION` with `ILLEGAL REQUEST` / *invalid command
//! operation code*, which is what a real drive answers and what lets a driver
//! probe without the model having to guess.
//!
//! # What it does not
//!
//! `FORMAT UNIT`, linked commands, tagged queueing, synchronous transfer
//! negotiation (the `SYNCHRONOUS DATA TRANSFER REQUEST` extended message),
//! `COPY`, and disconnection. The first four are refused loudly; the last is
//! [`super`]'s subject.
//!
//! # The medium
//!
//! [`dev::medium`](crate::dev::medium), the same seam `ata.disk` and an NVMe
//! namespace use, so `--media hd0=disk.hdf` and `--drive hd0=disk.qcow2` both
//! work on a SCSI drive with nothing here changing.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::{Bus, Phase, Target, buses, cdb_len, message, sense, status};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::medium::{self, Medium, Snapshot};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "scsi.disk";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The only block size this drive reports. A SCSI drive may have others; every
/// Amiga Rigid Disk Block ever written assumes this one, and `MODE SELECT`'s
/// block descriptor is accepted and ignored rather than obeyed.
pub const BLOCK: u64 = 512;

/// How many blocks a data phase moves between medium accesses.
///
/// A `READ(10)` may ask for 65,535 blocks — 32 MiB — and allocating that in one
/// go to answer one command would be absurd. The buffer is refilled this many
/// blocks at a time, with the state lock released across the medium access.
const CHUNK: u64 = 64;

/// Fixed-format sense data is eighteen bytes (X3.131-1994 §8.2.14, Table 65).
const SENSE_LEN: usize = 18;

// ---------------------------------------------------------------------------
// the target's state
// ---------------------------------------------------------------------------

/// What `REQUEST SENSE` will report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Sense {
    key: u8,
    asc: u8,
    ascq: u8,
    /// The `INFORMATION` field: the offending block, when there is one.
    info: u32,
    /// Whether `INFORMATION` is meaningful — the `VALID` bit.
    valid: bool,
}

impl Sense {
    const NONE: Sense = Sense {
        key: sense::NO_SENSE,
        asc: 0,
        ascq: 0,
        info: 0,
        valid: false,
    };

    const fn new(key: u8, asc: u8) -> Sense {
        Sense {
            key,
            asc,
            ascq: 0,
            info: 0,
            valid: false,
        }
    }
}

/// Which direction the data phase, if any, goes and how much is left of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Transfer {
    /// The next block on the medium.
    lba: u64,
    /// Blocks still to move after whatever is in the buffer.
    left: u64,
}

/// Everything one target holds.
#[derive(Debug)]
struct State {
    phase: Phase,
    /// The logical unit the `IDENTIFY` message named.
    lun: u8,
    /// The command descriptor block being received, or the last one received.
    cdb: [u8; 12],
    /// How many of its bytes have arrived.
    filled: usize,
    /// How many it will have when it is complete; zero before the first byte.
    want: usize,
    /// The bytes of the current data phase — filled for `DATA IN`, filling for
    /// `DATA OUT`.
    buffer: Vec<u8>,
    /// How much of `buffer` has been moved.
    cursor: usize,
    /// Blocks still to come after `buffer`, if this is a disk transfer.
    transfer: Transfer,
    /// The status byte the `STATUS` phase will deliver.
    status: u8,
    /// Sense data for the next `REQUEST SENSE`.
    sense: Sense,
}

impl State {
    fn idle() -> State {
        State {
            phase: Phase::BusFree,
            lun: 0,
            cdb: [0; 12],
            filled: 0,
            want: 0,
            buffer: Vec::new(),
            cursor: 0,
            transfer: Transfer::default(),
            status: status::GOOD,
            sense: Sense::NONE,
        }
    }

    /// Everything a connection owns, forgotten. The sense data survives: it
    /// belongs to the logical unit, not to the connection, which is the whole
    /// reason `REQUEST SENSE` can be a separate command (§8.2.14).
    fn disconnect(&mut self) {
        self.phase = Phase::BusFree;
        self.filled = 0;
        self.want = 0;
        self.buffer = Vec::new();
        self.cursor = 0;
        self.transfer = Transfer::default();
    }

    /// End the command with `status`, and no more data.
    fn finish(&mut self, code: u8) {
        self.status = code;
        self.buffer = Vec::new();
        self.cursor = 0;
        self.transfer = Transfer::default();
        self.phase = Phase::Status;
    }

    /// End the command badly, recording why.
    fn fail(&mut self, sense: Sense) {
        self.sense = sense;
        self.finish(status::CHECK_CONDITION);
    }

    /// Deliver `data`, truncated to the allocation length the command asked
    /// for, and then a `GOOD` status.
    fn answer(&mut self, mut data: Vec<u8>, allocation: usize) {
        data.truncate(allocation);
        self.status = status::GOOD;
        self.cursor = 0;
        self.transfer = Transfer::default();
        if data.is_empty() {
            self.buffer = data;
            self.phase = Phase::Status;
        } else {
            self.buffer = data;
            self.phase = Phase::DataIn;
        }
    }
}

// ---------------------------------------------------------------------------
// the drive
// ---------------------------------------------------------------------------

/// A SCSI hard disk.
pub struct ScsiDisk {
    media: Arc<dyn Medium>,
    /// How many 512-byte blocks the medium holds.
    blocks: u64,
    read_only: bool,
    vendor: String,
    product: String,
    revision: String,
    serial: String,
    state: Mutex<State>,
}

impl fmt::Debug for ScsiDisk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ScsiDisk")
            .field("blocks", &self.blocks)
            .field("read_only", &self.read_only)
            .field("product", &self.product)
            .field("phase", &self.state.lock().phase)
            .finish_non_exhaustive()
    }
}

impl ScsiDisk {
    /// A drive over `media`, whose capacity it takes.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the medium does not hold a whole number of blocks,
    /// or holds none.
    pub fn with_medium(media: Arc<dyn Medium>) -> Result<ScsiDisk> {
        let bytes = media.capacity();
        if bytes == 0 || !bytes.is_multiple_of(BLOCK) {
            return Err(config(format!(
                "a SCSI disk holds a whole number of {BLOCK}-byte blocks, and {bytes} bytes is \
                 not a whole number of them"
            )));
        }
        let read_only = media.is_read_only();
        Ok(ScsiDisk {
            media,
            blocks: bytes / BLOCK,
            read_only,
            vendor: String::from("RSEMU"),
            product: String::from("SCSI HARDDISK"),
            revision: String::from("1.0"),
            serial: String::from("RSEMU0000000000000001"),
            state: Mutex::with_rank(LockRank::DEVICE, State::idle()),
        })
    }

    /// How many blocks it holds.
    #[must_use]
    pub const fn blocks(&self) -> u64 {
        self.blocks
    }

    /// Whether it refuses writes.
    #[must_use]
    pub const fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// Put `bytes` on the medium at `offset` — how a media slot's contents get
    /// onto a drive that was built from one.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if they do not fit.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.media.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit on a drive of {}",
                bytes.len(),
                self.blocks * BLOCK
            ))
        })
    }

    /// The whole medium as a fresh vector, for a [`Snapshot::Capture`] chunk or
    /// a test.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the medium could not be read.
    pub fn contents(&self) -> Result<Vec<u8>> {
        let mut out = alloc::vec![0u8; (self.blocks * BLOCK) as usize];
        self.media
            .read_at(0, &mut out)
            .map_err(|e| medium::error_at(0, e))?;
        Ok(out)
    }

    /// Make every write durable.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the host refused.
    pub fn flush_media(&self) -> Result<()> {
        self.media.flush().map_err(|e| medium::error_at(0, e))
    }

    /// The reported geometry, derived from the capacity.
    ///
    /// SCSI has no geometry in its addressing — a block number is a block
    /// number — but `MODE SENSE` page `04` reports one and some drivers read
    /// it. Thirty-two blocks a track is the figure every Amiga hard-disk tool
    /// of the period defaults to; heads are doubled until the cylinder count
    /// fits the three bytes the page has for it.
    #[must_use]
    pub fn geometry(&self) -> (u32, u8, u8) {
        let sectors: u64 = 32;
        let mut heads: u64 = 1;
        while heads < 16 && self.blocks.div_ceil(sectors * heads) > 0x00ff_ffff {
            heads *= 2;
        }
        let cylinders = self.blocks.div_ceil(sectors * heads).min(0x00ff_ffff);
        (cylinders as u32, heads as u8, sectors as u8)
    }

    // -- the command set ----------------------------------------------------

    /// Run the command descriptor block now in `state.cdb`, leaving the target
    /// in whatever phase comes next.
    ///
    /// Called with the state lock held and **no medium access inside it**: a
    /// data phase records where it will read from and the first chunk is
    /// fetched by [`ScsiDisk::refill`] with the lock released.
    fn execute(&self, state: &mut State) {
        let cdb = state.cdb;
        let opcode = cdb[0];
        // §8.2.5: `INQUIRY` must answer for an unsupported logical unit rather
        // than fail, so a driver can tell "no such unit" from "no such target".
        if state.lun != 0 && opcode != 0x12 && opcode != 0x03 {
            state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_LUN));
            return;
        }
        match opcode {
            // TEST UNIT READY (§8.2.16)
            0x00 => state.finish(status::GOOD),
            // REQUEST SENSE (§8.2.14) — clears the sense it reports.
            0x03 => {
                let sense = state.sense;
                state.sense = Sense::NONE;
                let allocation = cdb[4] as usize;
                state.answer(fixed_sense(&sense), allocation);
            }
            // READ(6) (§9.2.5): a 21-bit block address, and a count of zero
            // means 256 blocks.
            0x08 => {
                let lba = u64::from(u32::from_be_bytes([0, cdb[1] & 0x1f, cdb[2], cdb[3]]));
                let count = if cdb[4] == 0 { 256 } else { u64::from(cdb[4]) };
                self.begin_read(state, lba, count);
            }
            // WRITE(6) (§9.2.16)
            0x0a => {
                let lba = u64::from(u32::from_be_bytes([0, cdb[1] & 0x1f, cdb[2], cdb[3]]));
                let count = if cdb[4] == 0 { 256 } else { u64::from(cdb[4]) };
                self.begin_write(state, lba, count);
            }
            // SEEK(6) / SEEK(10) (§9.2.14): the heads move, nothing transfers.
            0x0b | 0x2b => state.finish(status::GOOD),
            // INQUIRY (§8.2.5)
            0x12 => {
                let allocation = cdb[4] as usize;
                let evpd = cdb[1] & 1 == 1;
                match (evpd, cdb[2]) {
                    (false, 0) => {
                        let lun = state.lun;
                        state.answer(self.inquiry(lun), allocation);
                    }
                    (false, _) => {
                        state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_FIELD));
                    }
                    // Page 00: which vital product data pages exist.
                    (true, 0x00) => {
                        state.answer(alloc::vec![0x00, 0x00, 0x00, 0x02, 0x00, 0x80], allocation)
                    }
                    // Page 80: the unit serial number.
                    (true, 0x80) => {
                        let serial = self.serial.as_bytes();
                        let mut page = alloc::vec![0x00, 0x80, 0x00, serial.len() as u8];
                        page.extend_from_slice(serial);
                        state.answer(page, allocation);
                    }
                    (true, _) => {
                        state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_FIELD));
                    }
                }
            }
            // MODE SELECT(6) / (10) (§8.2.8, §8.2.9): the parameter list is
            // received and discarded. Every field it can carry — block size,
            // error recovery, caching — is one this drive has no second value
            // for, so obeying it and ignoring it are the same drive.
            0x15 => {
                let len = cdb[4] as usize;
                self.begin_data_out(state, len);
            }
            0x55 => {
                let len = usize::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                self.begin_data_out(state, len);
            }
            // RESERVE / RELEASE (§9.2.12, §9.2.11): one initiator, so a
            // reservation is always granted and always free to release.
            0x16 | 0x17 => state.finish(status::GOOD),
            // MODE SENSE(6) (§8.2.10)
            0x1a => {
                let allocation = cdb[4] as usize;
                let dbd = cdb[1] & 0x08 != 0;
                match self.mode_pages(cdb[2] & 0x3f) {
                    Some(pages) => state.answer(self.mode_sense6(dbd, &pages), allocation),
                    None => {
                        state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_FIELD));
                    }
                }
            }
            // START STOP UNIT (§9.2.15): the motor is always up.
            0x1b => state.finish(status::GOOD),
            // SEND DIAGNOSTIC (§8.2.15): the self-test passes.
            0x1d => state.finish(status::GOOD),
            // PREVENT ALLOW MEDIUM REMOVAL (§9.2.4): the medium is not
            // removable, so the lock is a no-op that succeeds.
            0x1e => state.finish(status::GOOD),
            // READ CAPACITY (§9.2.7): the *last* block's number, not the count.
            0x25 => {
                let last = (self.blocks - 1).min(u64::from(u32::MAX)) as u32;
                let mut data = Vec::with_capacity(8);
                data.extend_from_slice(&last.to_be_bytes());
                data.extend_from_slice(&(BLOCK as u32).to_be_bytes());
                state.answer(data, 8);
            }
            // READ(10) (§9.2.6)
            0x28 => {
                let lba = u64::from(u32::from_be_bytes([cdb[2], cdb[3], cdb[4], cdb[5]]));
                let count = u64::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                self.begin_read(state, lba, count);
            }
            // WRITE(10) (§9.2.17)
            0x2a => {
                let lba = u64::from(u32::from_be_bytes([cdb[2], cdb[3], cdb[4], cdb[5]]));
                let count = u64::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                self.begin_write(state, lba, count);
            }
            // VERIFY(10) (§9.2.19) with `BytChk` = 0: check the blocks are
            // readable, which on a medium that cannot have a bad block means
            // checking they are in range. `BytChk` = 1 would need a data-out
            // phase to compare against and is refused.
            0x2f => {
                if cdb[1] & 0x02 != 0 {
                    state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_FIELD));
                    return;
                }
                let lba = u64::from(u32::from_be_bytes([cdb[2], cdb[3], cdb[4], cdb[5]]));
                let count = u64::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                match self.range(lba, count) {
                    Ok(()) => state.finish(status::GOOD),
                    Err(s) => state.fail(s),
                }
            }
            // SYNCHRONIZE CACHE (§9.2.18): there is no cache in front of the
            // medium, so this is the medium's own flush.
            0x35 => match self.media.flush() {
                Ok(()) => state.finish(status::GOOD),
                Err(_) => state.fail(Sense::new(sense::MEDIUM_ERROR, 0x0c)),
            },
            // READ DEFECT DATA(10) (§9.2.8): a four-byte header and no list.
            // A drive with no defects is a truthful answer, and a driver that
            // asks is usually deciding whether to warn about one.
            0x37 => {
                let allocation = usize::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                let list_format = cdb[2] & 0x07;
                state.answer(
                    alloc::vec![0x00, 0x10 | list_format, 0x00, 0x00],
                    allocation,
                );
            }
            // MODE SENSE(10) (§8.2.11)
            0x5a => {
                let allocation = usize::from(u16::from_be_bytes([cdb[7], cdb[8]]));
                let dbd = cdb[1] & 0x08 != 0;
                match self.mode_pages(cdb[2] & 0x3f) {
                    Some(pages) => state.answer(self.mode_sense10(dbd, &pages), allocation),
                    None => {
                        state.fail(Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_INVALID_FIELD));
                    }
                }
            }
            _ => state.fail(Sense::new(
                sense::ILLEGAL_REQUEST,
                sense::ASC_INVALID_COMMAND,
            )),
        }
    }

    /// Whether `count` blocks from `lba` are on this medium (§9.2.6's
    /// out-of-range condition).
    fn range(&self, lba: u64, count: u64) -> core::result::Result<(), Sense> {
        if lba.checked_add(count).is_none_or(|end| end > self.blocks) {
            let mut s = Sense::new(sense::ILLEGAL_REQUEST, sense::ASC_LBA_OUT_OF_RANGE);
            s.info = lba as u32;
            s.valid = true;
            return Err(s);
        }
        Ok(())
    }

    fn begin_read(&self, state: &mut State, lba: u64, count: u64) {
        if let Err(s) = self.range(lba, count) {
            state.fail(s);
            return;
        }
        if count == 0 {
            // §9.2.6: a transfer length of zero is not an error and moves no
            // data.
            state.finish(status::GOOD);
            return;
        }
        state.status = status::GOOD;
        state.buffer = Vec::new();
        state.cursor = 0;
        state.transfer = Transfer { lba, left: count };
        state.phase = Phase::DataIn;
    }

    fn begin_write(&self, state: &mut State, lba: u64, count: u64) {
        if self.read_only {
            state.fail(Sense::new(sense::DATA_PROTECT, sense::ASC_WRITE_PROTECTED));
            return;
        }
        if let Err(s) = self.range(lba, count) {
            state.fail(s);
            return;
        }
        if count == 0 {
            state.finish(status::GOOD);
            return;
        }
        state.status = status::GOOD;
        state.buffer = Vec::new();
        state.cursor = 0;
        state.transfer = Transfer { lba, left: count };
        state.phase = Phase::DataOut;
    }

    /// A data-out phase of exactly `len` bytes whose contents are discarded.
    fn begin_data_out(&self, state: &mut State, len: usize) {
        if len == 0 {
            state.finish(status::GOOD);
            return;
        }
        state.status = status::GOOD;
        state.buffer = Vec::new();
        state.cursor = 0;
        // `left` of zero with a non-empty expectation is the discard case: the
        // buffer fills to `len` and then the command ends.
        state.transfer = Transfer { lba: 0, left: 0 };
        state.buffer.reserve(len);
        state.cursor = len;
        state.phase = Phase::DataOut;
    }

    // -- the data each command answers with ---------------------------------

    /// Standard `INQUIRY` data (§8.2.5, Table 45).
    fn inquiry(&self, lun: u8) -> Vec<u8> {
        let mut data = alloc::vec![0u8; 36];
        // Peripheral qualifier and device type. §8.2.5: a logical unit the
        // target does not support is qualifier 3, type 1F — "no device here,
        // and this target will never have one".
        data[0] = if lun == 0 { 0x00 } else { 0x7f };
        // Not removable.
        data[1] = 0x00;
        // ANSI-approved version 2: SCSI-2.
        data[2] = 0x02;
        // Response data format 2, which is what version 2 requires.
        data[3] = 0x02;
        // Additional length: everything after this byte.
        data[4] = 31;
        data[5] = 0x00;
        data[6] = 0x00;
        // No relative addressing, no wide, no synchronous, no linking, no
        // queueing — every one of them a thing this target does not do, and
        // §8.2.5.1 is explicit that these bits are claims rather than hopes.
        data[7] = 0x00;
        put_ascii(&mut data[8..16], &self.vendor);
        put_ascii(&mut data[16..32], &self.product);
        put_ascii(&mut data[32..36], &self.revision);
        data
    }

    /// The mode pages `page` selects, or `None` if it selects none.
    ///
    /// `3F` is "every page" (§8.2.10), and the pages come back in ascending
    /// page-code order, which the clause requires.
    fn mode_pages(&self, page: u8) -> Option<Vec<u8>> {
        let mut out = Vec::new();
        let all = page == 0x3f;
        if all || page == 0x01 {
            out.extend_from_slice(&self.page_error_recovery());
        }
        if all || page == 0x03 {
            out.extend_from_slice(&self.page_format());
        }
        if all || page == 0x04 {
            out.extend_from_slice(&self.page_geometry());
        }
        // Page code `00` is "vendor specific (does not require the page
        // format)" (§8.2.10, Table 90). This drive has no vendor-specific page,
        // so what it returns is the header and the block descriptor with
        // nothing after them — a complete mode parameter list (§8.3.3) rather
        // than an error, which is what a host asking for four bytes is after:
        // the medium type and the write-protect bit in the header.
        //
        // Refusing it is what stops an A4000T booting. Commodore's 53C710
        // `scsi.device` issues `1A 00 00 00 04 00` while it configures a unit,
        // and an `ILLEGAL REQUEST` there sends it round the whole bus scan
        // again — seven times, and then the unit is left half-configured and
        // AmigaDOS asks for the volume back.
        if out.is_empty() && page != 0x00 {
            return None;
        }
        Some(out)
    }

    /// Page `01`, read-write error recovery (§9.3.3.6, Table 116).
    fn page_error_recovery(&self) -> [u8; 12] {
        let mut page = [0u8; 12];
        page[0] = 0x01;
        page[1] = 0x0a;
        // No automatic reallocation, no transfer of a block that failed: a
        // medium with no failure modes has nothing to recover from.
        page[2] = 0x00;
        // One retry, which is one more than will ever be needed.
        page[3] = 0x01;
        page
    }

    /// Page `03`, format device (§9.3.3.1, Table 110).
    fn page_format(&self) -> [u8; 24] {
        let (_, heads, sectors) = self.geometry();
        let mut page = [0u8; 24];
        page[0] = 0x03;
        page[1] = 0x16;
        // Tracks per zone: all of them, which is one zone.
        page[2..4].copy_from_slice(&u16::from(heads).to_be_bytes());
        // Sectors per track, and bytes per physical sector.
        page[10..12].copy_from_slice(&u16::from(sectors).to_be_bytes());
        page[12..14].copy_from_slice(&(BLOCK as u16).to_be_bytes());
        // Interleave 1: consecutive blocks are consecutive.
        page[14..16].copy_from_slice(&1u16.to_be_bytes());
        // Hard-sectored, not removable.
        page[20] = 0x40;
        page
    }

    /// Page `04`, rigid disk drive geometry (§9.3.3.7, Table 117).
    fn page_geometry(&self) -> [u8; 24] {
        let (cylinders, heads, _) = self.geometry();
        let mut page = [0u8; 24];
        page[0] = 0x04;
        page[1] = 0x16;
        page[2..5].copy_from_slice(&cylinders.to_be_bytes()[1..4]);
        page[5] = heads;
        // No write precompensation and no reduced write current: both are
        // "starting cylinder" fields, and zero means from the first cylinder.
        // Landing zone: past the last cylinder, which is where a park goes.
        page[12..15].copy_from_slice(&cylinders.to_be_bytes()[1..4]);
        // 3600 rpm, the figure a drive of this era reports.
        page[20..22].copy_from_slice(&3600u16.to_be_bytes());
        page
    }

    /// The eight-byte block descriptor a `MODE SENSE` carries unless `DBD`
    /// asked it not to (§8.3.3, Table 91).
    fn block_descriptor(&self) -> [u8; 8] {
        let mut d = [0u8; 8];
        // Density code 0: the default for a direct-access device.
        d[0] = 0x00;
        let blocks = u32::try_from(self.blocks)
            .unwrap_or(0x00ff_ffff)
            .min(0x00ff_ffff);
        d[1..4].copy_from_slice(&blocks.to_be_bytes()[1..4]);
        d[5..8].copy_from_slice(&(BLOCK as u32).to_be_bytes()[1..4]);
        d
    }

    /// A `MODE SENSE(6)` reply: a four-byte header, a block descriptor, pages.
    fn mode_sense6(&self, dbd: bool, pages: &[u8]) -> Vec<u8> {
        let descriptor = if dbd { 0 } else { 8 };
        let mut out = Vec::with_capacity(4 + descriptor + pages.len());
        out.push(0);
        out.push(0x00);
        out.push(if self.read_only { 0x80 } else { 0x00 });
        out.push(descriptor as u8);
        if !dbd {
            out.extend_from_slice(&self.block_descriptor());
        }
        out.extend_from_slice(pages);
        // The length byte counts everything after itself (§8.3.3).
        out[0] = (out.len() - 1) as u8;
        out
    }

    /// A `MODE SENSE(10)` reply: the same, with a two-byte length and an
    /// eight-byte header (§8.3.3, Table 92).
    fn mode_sense10(&self, dbd: bool, pages: &[u8]) -> Vec<u8> {
        let descriptor = if dbd { 0u16 } else { 8 };
        let mut out = Vec::with_capacity(8 + usize::from(descriptor) + pages.len());
        out.extend_from_slice(&[0, 0]);
        out.push(0x00);
        out.push(if self.read_only { 0x80 } else { 0x00 });
        out.extend_from_slice(&[0, 0]);
        out.extend_from_slice(&descriptor.to_be_bytes());
        if !dbd {
            out.extend_from_slice(&self.block_descriptor());
        }
        out.extend_from_slice(pages);
        let len = (out.len() - 2) as u16;
        out[0..2].copy_from_slice(&len.to_be_bytes());
        out
    }

    // -- the data phases ----------------------------------------------------

    /// Fetch the next chunk of a `DATA IN` transfer, if there is one and the
    /// buffer is empty.
    ///
    /// Called with **no lock held**: it takes the state lock to decide, drops
    /// it across the medium access, and takes it again to store the result.
    /// That is the re-entrancy contract, and it is what keeps a `READ(10)` of
    /// 32 MiB from allocating 32 MiB.
    fn refill(&self) {
        let (lba, count) = {
            let state = self.state.lock();
            if state.cursor < state.buffer.len() || state.transfer.left == 0 {
                return;
            }
            (state.transfer.lba, state.transfer.left.min(CHUNK))
        };
        let mut chunk = alloc::vec![0u8; (count * BLOCK) as usize];
        let result = self.media.read_at(lba * BLOCK, &mut chunk);
        let mut state = self.state.lock();
        // Another access may have moved on while the lock was down; only this
        // transfer's own continuation is stored.
        if state.transfer.lba != lba || state.transfer.left < count {
            return;
        }
        match result {
            Ok(()) => {
                state.buffer = chunk;
                state.cursor = 0;
                state.transfer.lba += count;
                state.transfer.left -= count;
            }
            Err(_) => {
                let mut s = Sense::new(sense::MEDIUM_ERROR, 0x11);
                s.info = lba as u32;
                s.valid = true;
                state.fail(s);
            }
        }
    }

    /// Put a full chunk of a `DATA OUT` transfer on the medium.
    ///
    /// The same shape as [`ScsiDisk::refill`], in the other direction.
    fn drain(&self) {
        let (lba, chunk) = {
            let mut state = self.state.lock();
            if state.transfer.left == 0 || state.buffer.is_empty() {
                return;
            }
            let whole = (state.buffer.len() as u64 / BLOCK).min(state.transfer.left);
            if whole == 0 {
                return;
            }
            let bytes = (whole * BLOCK) as usize;
            let chunk: Vec<u8> = state.buffer.drain(..bytes).collect();
            let lba = state.transfer.lba;
            state.transfer.lba += whole;
            state.transfer.left -= whole;
            (lba, chunk)
        };
        if let Err(_e) = self.media.write_at(lba * BLOCK, &chunk) {
            let mut state = self.state.lock();
            let mut s = Sense::new(sense::MEDIUM_ERROR, 0x0c);
            s.info = lba as u32;
            s.valid = true;
            state.fail(s);
        }
    }
}

impl Target for ScsiDisk {
    fn select(&self, atn: bool) -> bool {
        let mut state = self.state.lock();
        state.disconnect();
        state.lun = 0;
        state.phase = if atn {
            Phase::MessageOut
        } else {
            Phase::Command
        };
        true
    }

    fn phase(&self) -> Phase {
        self.state.lock().phase
    }

    fn read(&self, dst: &mut [u8]) -> usize {
        if dst.is_empty() {
            return 0;
        }
        // A `DATA IN` phase may need the next chunk off the medium, which is an
        // access made with no lock held.
        if self.state.lock().phase == Phase::DataIn {
            self.refill();
        }
        let mut state = self.state.lock();
        match state.phase {
            Phase::DataIn => {
                let available = state.buffer.len().saturating_sub(state.cursor);
                let n = available.min(dst.len());
                dst[..n].copy_from_slice(&state.buffer[state.cursor..state.cursor + n]);
                state.cursor += n;
                if state.cursor >= state.buffer.len() && state.transfer.left == 0 {
                    state.buffer = Vec::new();
                    state.cursor = 0;
                    state.phase = Phase::Status;
                }
                n
            }
            Phase::Status => {
                dst[0] = state.status;
                state.phase = Phase::MessageIn;
                1
            }
            Phase::MessageIn => {
                dst[0] = message::COMMAND_COMPLETE;
                state.disconnect();
                1
            }
            // An input read in an output phase moves nothing. The initiator has
            // misread the phase signals, which on a real bus is exactly what
            // happens: the target is not driving the data bus.
            _ => 0,
        }
    }

    fn write(&self, src: &[u8]) -> usize {
        let mut taken = 0;
        while taken < src.len() {
            let mut state = self.state.lock();
            match state.phase {
                // §6.6.7: one `IDENTIFY`, and then the target asks for the
                // command. A message this target does not understand is
                // accepted and ignored rather than rejected, because rejecting
                // it needs an `ATN` handshake nothing here drives.
                Phase::MessageOut => {
                    let byte = src[taken];
                    taken += 1;
                    if byte & message::IDENTIFY != 0 {
                        state.lun = byte & message::LUN_MASK;
                    }
                    state.phase = Phase::Command;
                }
                Phase::Command => {
                    let byte = src[taken];
                    taken += 1;
                    if state.filled == 0 {
                        // §7.1: the group code in the first byte says how long
                        // the block is. A reserved or vendor group is answered
                        // as a six-byte command, which is what a drive that
                        // cannot recognise it will do with the bytes it gets.
                        state.want = cdb_len(byte).unwrap_or(6);
                        state.cdb = [0; 12];
                    }
                    let at = state.filled;
                    if at < state.cdb.len() {
                        state.cdb[at] = byte;
                    }
                    state.filled += 1;
                    if state.filled >= state.want {
                        self.execute(&mut state);
                        state.filled = 0;
                        state.want = 0;
                    }
                }
                Phase::DataOut => {
                    // The discard case: `cursor` holds how many bytes are still
                    // expected and nothing is written anywhere.
                    if state.transfer.left == 0 {
                        let n = state.cursor.min(src.len() - taken);
                        state.cursor -= n;
                        taken += n;
                        if state.cursor == 0 {
                            state.finish(status::GOOD);
                        }
                        continue;
                    }
                    let want = (state.transfer.left * BLOCK) as usize;
                    let room = want.saturating_sub(state.buffer.len());
                    let n = room.min(src.len() - taken);
                    if n == 0 {
                        break;
                    }
                    state.buffer.extend_from_slice(&src[taken..taken + n]);
                    taken += n;
                    let full = state.buffer.len() as u64 >= BLOCK;
                    drop(state);
                    if full {
                        self.drain();
                    }
                    let mut state = self.state.lock();
                    if state.phase == Phase::DataOut
                        && state.transfer.left == 0
                        && state.buffer.is_empty()
                    {
                        state.finish(status::GOOD);
                    }
                }
                // Nothing is listening in an input phase.
                _ => break,
            }
        }
        taken
    }

    fn release(&self) {
        self.state.lock().disconnect();
    }

    fn bus_reset(&self) {
        let mut state = self.state.lock();
        state.disconnect();
        // §5.2.2: the next command from every initiator gets a unit attention.
        state.sense = Sense::new(sense::UNIT_ATTENTION, sense::ASC_RESET);
        state.status = status::GOOD;
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The medium first, on its own terms — the argument is
        // `dev::medium::Snapshot`'s, and `ata.disk` makes the same call.
        match self.media.snapshot() {
            Snapshot::Capture => w.write_bytes(&self.contents()?)?,
            Snapshot::Reference => {
                self.media.flush().map_err(|e| medium::error_at(0, e))?;
                w.write_bytes(self.media.describe().as_bytes())?;
            }
            Snapshot::Refuse => {
                return Err(Error::State(format!(
                    "this drive's medium ({}) refuses to be snapshotted",
                    self.media.describe()
                )));
            }
        }
        let state = self.state.lock();
        w.write_u8(phase_code(state.phase))?;
        w.write_u8(state.lun)?;
        w.write_all(&state.cdb)?;
        w.write_u64(state.filled as u64)?;
        w.write_u64(state.want as u64)?;
        w.write_bytes(&state.buffer)?;
        w.write_u64(state.cursor as u64)?;
        w.write_u64(state.transfer.lba)?;
        w.write_u64(state.transfer.left)?;
        w.write_u8(state.status)?;
        w.write_u8(state.sense.key)?;
        w.write_u8(state.sense.asc)?;
        w.write_u8(state.sense.ascq)?;
        w.write_u32(state.sense.info)?;
        w.write_bool(state.sense.valid)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        match self.media.snapshot() {
            Snapshot::Capture => {
                let bytes = r.read_bytes()?;
                if bytes.len() as u64 != self.blocks * BLOCK {
                    return Err(Error::State(format!(
                        "the snapshot holds {} byte(s) and this drive is {}",
                        bytes.len(),
                        self.blocks * BLOCK
                    )));
                }
                self.media
                    .write_at(0, bytes)
                    .map_err(|e| medium::error_at(0, e))?;
            }
            Snapshot::Reference => {
                let described = core::str::from_utf8(r.read_bytes()?).map_err(|e| {
                    Error::State(format!("the medium's identity is not UTF-8: {e}"))
                })?;
                if described != self.media.describe() {
                    return Err(Error::State(format!(
                        "the snapshot references `{described}` and this drive is `{}`",
                        self.media.describe()
                    )));
                }
            }
            Snapshot::Refuse => {
                return Err(Error::State(String::from(
                    "this drive's medium refuses to be snapshotted, so there is nothing to \
                     restore from",
                )));
            }
        }
        let phase = phase_from(r.read_u8()?)?;
        let lun = r.read_u8()?;
        let mut cdb = [0u8; 12];
        cdb.copy_from_slice(r.take(12)?);
        let filled = r.read_u64()? as usize;
        let want = r.read_u64()? as usize;
        let buffer = r.read_bytes()?.to_vec();
        let cursor = r.read_u64()? as usize;
        let lba = r.read_u64()?;
        let left = r.read_u64()?;
        let code = r.read_u8()?;
        let sense = Sense {
            key: r.read_u8()?,
            asc: r.read_u8()?,
            ascq: r.read_u8()?,
            info: r.read_u32()?,
            valid: r.read_bool()?,
        };
        let mut state = self.state.lock();
        *state = State {
            phase,
            lun: lun & message::LUN_MASK,
            cdb,
            filled: filled.min(12),
            want: want.min(12),
            buffer,
            cursor,
            transfer: Transfer { lba, left },
            status: code,
            sense,
        };
        Ok(())
    }
}

/// A phase as one snapshot byte. Not [`Phase::mci`]: bus free has no `MCI`
/// value, and a snapshot needs every state to have a code.
const fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::BusFree => 0,
        Phase::DataOut => 1,
        Phase::DataIn => 2,
        Phase::Command => 3,
        Phase::Status => 4,
        Phase::MessageOut => 5,
        Phase::MessageIn => 6,
    }
}

fn phase_from(code: u8) -> Result<Phase> {
    match code {
        0 => Ok(Phase::BusFree),
        1 => Ok(Phase::DataOut),
        2 => Ok(Phase::DataIn),
        3 => Ok(Phase::Command),
        4 => Ok(Phase::Status),
        5 => Ok(Phase::MessageOut),
        6 => Ok(Phase::MessageIn),
        other => Err(Error::State(format!("{other} is not a SCSI bus phase"))),
    }
}

/// Fixed-format sense data (X3.131-1994 §8.2.14, Table 65).
fn fixed_sense(s: &Sense) -> Vec<u8> {
    let mut data = alloc::vec![0u8; SENSE_LEN];
    // Error code 70: a current error, in the fixed format.
    data[0] = 0x70 | if s.valid { 0x80 } else { 0x00 };
    data[2] = s.key & 0x0f;
    data[3..7].copy_from_slice(&s.info.to_be_bytes());
    // Additional sense length: everything after this byte.
    data[7] = (SENSE_LEN - 8) as u8;
    data[12] = s.asc;
    data[13] = s.ascq;
    data
}

/// Lay `text` into an ASCII field, space padded, as SCSI fields are (§8.2.5.1).
fn put_ascii(field: &mut [u8], text: &str) {
    for (slot, byte) in field.iter_mut().zip(
        text.bytes()
            .map(|b| {
                if b.is_ascii_graphic() || b == b' ' {
                    b
                } else {
                    b' '
                }
            })
            .chain(core::iter::repeat(b' ')),
    ) {
        *slot = byte;
    }
}

fn config(message: String) -> Error {
    Error::Config {
        at: String::from(CLASS_NAME),
        message,
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The `scsi.disk` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a SCSI direct-access device: the command set, the phase sequence and a medium, at \
              one address on a named SCSI bus",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot whose bytes fill the drive; no bytes is no drive",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "the capacity when no image says, in bytes (a multiple of 512)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the SCSI bus this target is on (default `scsi0`)",
        },
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "this target's SCSI bus address, 0 to 7 (default 0)",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the drive refuses writes",
        },
        PropertySpec {
            name: "vendor",
            kind: ValueKind::Str,
            required: false,
            summary: "the eight-character vendor field of INQUIRY data",
        },
        PropertySpec {
            name: "product",
            kind: ValueKind::Str,
            required: false,
            summary: "the sixteen-character product field of INQUIRY data",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Str,
            required: false,
            summary: "the four-character revision field of INQUIRY data",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Str,
            required: false,
            summary: "the unit serial number, INQUIRY vital product data page 80",
        },
    ],
    construct: |props| Ok(Box::new(DiskDevice::new(props)?)),
};

/// `scsi.disk`: a drive, and the bus address it answers at.
#[derive(Debug)]
pub struct DiskDevice {
    drive: Option<Arc<ScsiDisk>>,
    bus: Arc<Bus>,
    id: u8,
}

impl DiskDevice {
    /// Validate `props`, allocate the drive and take its place on the bus.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind;
    /// [`Error::Config`] if the capacity is not one a drive can have, or the
    /// bus address is taken.
    pub fn new(props: &Props) -> Result<DiskDevice> {
        let mut r = props.reader();
        let size = r.or_size("size", 0)?;
        let media = r.optional_media("image")?;
        let slot = media.map(crate::core::props::Media::name);
        let image = media.map(crate::core::props::Media::to_bytes);
        let bus_name = r.or_str("bus", super::DEFAULT_BUS)?.to_string();
        let id = r.or_range("id", 0u64, 0..=7)? as u8;
        let read_only = r.or("readonly", false)?;
        let vendor = r.or_str("vendor", "RSEMU")?.to_string();
        let product = r.or_str("product", "SCSI HARDDISK")?.to_string();
        let revision = r.or_str("revision", "1.0")?.to_string();
        let serial = r.or_str("serial", "RSEMU0000000000000001")?.to_string();
        r.finish()?;

        // Opening the bus is allocation, not an outward action — the argument
        // `dev::ata::bays` makes for a drive bay.
        let bus = buses::attach(props, &bus_name)?;

        // A medium the host installed wins over the media table, for the reason
        // `ata.disk` gives: a run that said `--drive hd0=disk.qcow2` meant it.
        let supplied = match props.hosts() {
            Some(hosts) => {
                let name = slot.unwrap_or(&bus_name);
                medium::get(hosts, name)?.and_then(|slot| slot.take())
            }
            None => None,
        };
        let bytes = match (&supplied, size, image.as_ref()) {
            (Some(m), _, _) => m.capacity(),
            (None, 0, Some(image)) => image.len() as u64,
            (None, size, _) => size,
        };
        // No bytes anywhere is an empty address on the cable, which is a
        // machine without that drive rather than a misconfiguration.
        if bytes == 0 {
            return Ok(DiskDevice {
                drive: None,
                bus,
                id,
            });
        }

        let media: Arc<dyn Medium> = match supplied {
            Some(m) => m,
            None => Arc::new(RamStore::new(bytes)),
        };
        let mut disk = ScsiDisk::with_medium(media)?;
        disk.read_only = read_only || disk.read_only;
        disk.vendor = vendor;
        disk.product = product;
        disk.revision = revision;
        disk.serial = serial;
        let disk = Arc::new(disk);
        if let Some(image) = image {
            if image.len() as u64 > bytes {
                return Err(config(format!(
                    "the bound image is {} byte(s) and the drive holds {bytes}",
                    image.len()
                )));
            }
            disk.load_image(0, &image)?;
        }
        let target: Arc<dyn Target> = Arc::clone(&disk) as Arc<dyn Target>;
        if bus.fit(id, target).is_err() {
            return Err(config(format!(
                "SCSI bus `{bus_name}` already has a target at address {id}"
            )));
        }
        Ok(DiskDevice {
            drive: Some(disk),
            bus,
            id,
        })
    }

    /// The drive, if this address is occupied.
    #[must_use]
    pub fn drive(&self) -> Option<Arc<ScsiDisk>> {
        self.drive.clone()
    }

    /// The bus it is on.
    #[must_use]
    pub fn bus(&self) -> Arc<Bus> {
        Arc::clone(&self.bus)
    }
}

impl Device for DiskDevice {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the bus address was claimed at construction, which
        // is allocation rather than an observable action.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. A board reset cycles the drive's power: the protocol
        // state goes, the contents stay — the distinction `ata.disk`, NOR flash
        // and an SD card all draw.
        if let Some(drive) = &self.drive {
            drive.release();
            let mut state = drive.state.lock();
            state.sense = Sense::NONE;
            state.status = status::GOOD;
            state.lun = 0;
        }
    }

    fn flush(&self) -> Result<()> {
        match &self.drive {
            Some(drive) => drive.flush_media(),
            None => Ok(()),
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u8(self.id)?;
        match &self.drive {
            None => w.write_bool(false),
            Some(drive) => {
                w.write_bool(true)?;
                drive.save(w)
            }
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let id = r.read_u8()?;
        if id != self.id {
            return Err(Error::State(format!(
                "the snapshot's target is at address {id} and this one is at {}",
                self.id
            )));
        }
        let occupied = r.read_bool()?;
        match (&self.drive, occupied) {
            (Some(drive), true) => drive.load(r),
            (None, false) => Ok(()),
            (Some(_), false) => Err(Error::State(String::from(
                "the snapshot has an empty bus address and this machine has a drive at it",
            ))),
            (None, true) => Err(Error::State(String::from(
                "the snapshot has a drive and this machine's bus address is empty",
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

/// What the validator should know about `scsi.disk`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("id", ValueKind::Uint))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("vendor", ValueKind::Str))
        .prop(PropSchema::new("product", ValueKind::Str))
        .prop(PropSchema::new("revision", ValueKind::Str))
        .prop(PropSchema::new("serial", ValueKind::Str))
}
