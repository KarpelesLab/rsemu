//! The AMD/Fujitsu standard command set (`0x0002`), as a Spansion S29JL064J
//! speaks it.
//!
//! # Source
//!
//! The **Infineon (Cypress/Spansion) S29JL064J** datasheet, document
//! 002-00856 Rev. \*J: §8.9 and the autoselect code table (manufacturer,
//! the three-word device ID, sector protection, the Secured Silicon
//! indicator), §8.11 and Table 7 (`WP#` protects sectors 0, 1, 140 and 141),
//! §8.13 and Table 5 (the 256-byte Secured Silicon Region at the bottom of the
//! array), §9 and Tables 8-11 (the CFI query and the AMD primary extended
//! table), §10 and Table 12 (every command sequence, with its word- and
//! byte-mode addresses), and §11 (the status bits a driver polls). JEDEC
//! JESD68.01 for the query's common layout, which `cfi.rs` already cites.
//!
//! No emulator source and no driver source of any licence was consulted.
//!
//! # Addresses
//!
//! Every command cycle is decoded by its **device word address**: the bus
//! offset divided by the bus width, which on the usual board — one x16 part
//! whose A0 is the CPU's A1 (`width = 2, interleave = 1`) — is the byte offset
//! over two. The unlock cycles are at word `0x555` and `0x2aa`, and only
//! A10-A0 take part ("address bits A21-A11 are don't cares for unlock and
//! command cycles", Table 12 note 15), so a bank address above them changes
//! nothing. A part wired **x8** (`BYTE#` low, a device width of one byte) is
//! addressed in bytes with A-1 as its lowest line, and its table gives the same
//! cycles at `0xaaa` and `0x555` and the query at byte `0xaa`: one bit to the
//! left of the word figures. So an x8 part's byte address is shifted right
//! once and every table below is written once, in words.
//!
//! # Time, and what a driver polls
//!
//! As for the Intel set (`cfi.rs`, "Time"): a program or an erase completes in
//! the bus cycle that issues it. The AMD set has no status register to report
//! that through — a driver polls the array itself, and reads **DQ7** (data
//! polling: the complement of the programmed bit until the program is done,
//! §11.1) or **DQ6** (toggle bit: it flips on every read while busy, §11.3).
//! Both see a finished operation here on the first read: DQ7 is already the
//! true data, and two consecutive reads return the same byte, so DQ6 does not
//! toggle. That is the whole of the status protocol this part needs to honour,
//! and it is why there is no status mode: a read after a command is a read of
//! the array. An erase suspend (`0xb0`) and resume (`0x30`) are accepted and do
//! nothing, because there is never an erase in progress to suspend.
//!
//! # What is not modelled
//!
//! * **Banks as separate state machines.** The part has four banks and can
//!   read one while another programs (§8.3, "simultaneous operation"). With
//!   zero-time operations nothing is ever busy to be read around, so the whole
//!   part is one state machine; `banks` exists for the query's bank fields.
//! * **Autoselect scoped to a bank.** Autoselect entered at a bank address
//!   answers in that bank only, on silicon; here it answers at every address
//!   until the reset command, which is what a driver that reads the codes and
//!   then writes `0xf0` observes either way.
//! * **In-system sector protection.** The protect and unprotect algorithms use
//!   a high voltage on `RESET#` (§8.10). Sectors are protected by the machine
//!   file (`locked`, `readonly`, `write-protect`), and autoselect reports it.
//! * **ACC.** `WP#/ACC` at `VHH` only makes programming faster.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use super::{Array, Chip, Geometry, Mode, Pending, QUERY_REGIONS, config, log2};
use crate::core::error::{Error, Result};
use crate::core::props::{Media, Reader, Value};
use crate::core::sync::Ordering;

/// AMD's JEDEC manufacturer identifier (JEP106 bank 1, `0x01`), which Spansion
/// parts report (S29JL064J autoselect table: "Manufacturer ID: 01h").
pub const AMD_MANUFACTURER: u16 = 0x0001;

/// The Secured Silicon Region's size (S29JL064J §8.13, Table 5).
pub const SECSI_BYTES: usize = 256;

/// The first unlock cycle's word address, and the address the third cycle of
/// every sequence goes to.
const UNLOCK_1: u64 = 0x555;
/// The second unlock cycle's word address.
const UNLOCK_2: u64 = 0x2aa;
/// A10-A0: the address lines a command cycle decodes (Table 12, note 15).
const UNLOCK_MASK: u64 = 0x7ff;
/// The word address the CFI query command is written to (§9).
const QUERY_ADDR: u64 = 0x55;

/// The command bytes (Table 12).
const CMD_UNLOCK_1: u8 = 0xaa;
const CMD_UNLOCK_2: u8 = 0x55;
const CMD_RESET: u8 = 0xf0;
const CMD_AUTOSELECT: u8 = 0x90;
const CMD_PROGRAM: u8 = 0xa0;
const CMD_ERASE_SETUP: u8 = 0x80;
const CMD_CHIP_ERASE: u8 = 0x10;
const CMD_SECTOR_ERASE: u8 = 0x30;
const CMD_BYPASS: u8 = 0x20;
const CMD_SECSI_ENTRY: u8 = 0x88;
const CMD_EXIT: u8 = 0x00;
const CMD_QUERY: u8 = 0x98;
const CMD_SUSPEND: u8 = 0xb0;
/// Erase resume shares its byte with the sector erase command.
const CMD_RESUME: u8 = 0x30;

/// How the Secured Silicon Region was locked, which is what autoselect word 3
/// reports and whether it can still be written (S29JL064J §8.13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SecSiLock {
    /// Neither: the region is extra flash, programmable and erasable.
    #[default]
    None,
    /// Locked by the customer: read-only from now on.
    Customer,
    /// Locked at the factory, usually around a serial number: read-only.
    Factory,
}

impl SecSiLock {
    /// The Secured Silicon indicator autoselect returns: `81h` factory locked,
    /// `41h` customer locked, `01h` neither (Table 12, note 20).
    #[must_use]
    pub fn indicator(self) -> u16 {
        match self {
            SecSiLock::None => 0x01,
            SecSiLock::Customer => 0x41,
            SecSiLock::Factory => 0x81,
        }
    }

    fn parse(text: &str) -> Result<SecSiLock> {
        match text {
            "none" => Ok(SecSiLock::None),
            "customer" => Ok(SecSiLock::Customer),
            "factory" => Ok(SecSiLock::Factory),
            other => Err(Error::Property(format!(
                "`secsi-lock = \"{other}\"`: the region is \"none\", \"customer\" or \"factory\" \
                 locked"
            ))),
        }
    }
}

/// What distinguishes one AMD-set part from another beyond its geometry.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AmdOptions {
    /// The second and third device-ID words (autoselect `0x0e` and `0x0f`),
    /// for a part whose first device word is `0x7e`, meaning "read on".
    pub device_ext: Option<[u16; 2]>,
    /// Sectors per bank, lowest bank first — the query's bank organisation
    /// (`0x57`) and per-bank counts (`0x58` on). Empty for a part that
    /// reports none.
    pub banks: Vec<u64>,
    /// The query's top/bottom boot flag (`0x4f`), or `None` to derive it
    /// from where the small sectors are.
    pub boot_flag: Option<u8>,
    /// The Secured Silicon Region as the *bus* sees it at the bottom of the
    /// window — one region per part, laid out across the lanes the way the
    /// array is. Shorter than the window is fine; the rest reads erased.
    pub secsi: Vec<u8>,
    /// How the region is locked.
    pub secsi_lock: SecSiLock,
    /// `WP#` held low: the two outermost sectors at each end of the array
    /// refuse program and erase (§8.11, Table 7).
    pub write_protect: bool,
}

impl AmdOptions {
    /// Check the options against the geometry they describe.
    pub(super) fn check(&self, geom: &Geometry) -> Result<()> {
        if !self.banks.is_empty() {
            let total: u64 = self.banks.iter().sum();
            if total != geom.block_count() {
                return Err(config(format!(
                    "`banks` adds up to {total} sector(s) and the geometry has {}",
                    geom.block_count()
                )));
            }
            if self.banks.len() > 0xff || self.banks.iter().any(|b| *b == 0 || *b > 0xff) {
                return Err(config(String::from(
                    "the query holds a bank count and each bank's sector count in one byte",
                )));
            }
        }
        let window = SECSI_BYTES as u64 * geom.interleave();
        if self.secsi.len() as u64 > window {
            return Err(config(format!(
                "a Secured Silicon image of {} byte(s), and {} part(s) of {SECSI_BYTES} bytes \
                 each hold {window}",
                self.secsi.len(),
                geom.interleave()
            )));
        }
        Ok(())
    }

    /// Lane `lane`'s own 256 bytes out of the bus-order image.
    pub(super) fn secsi_for_lane(&self, geom: &Geometry, lane: usize, lanes: usize) -> Vec<u8> {
        let mut out = alloc::vec![0xffu8; SECSI_BYTES];
        let dw = geom.device_width();
        let bw = geom.bus_width();
        for (o, byte) in self.secsi.iter().enumerate() {
            let o = o as u64;
            if ((o / dw) % lanes as u64) as usize != lane {
                continue;
            }
            let at = ((o / bw) * dw + (o % bw) % dw) as usize;
            if let Some(slot) = out.get_mut(at) {
                *slot = *byte;
            }
        }
        out
    }
}

/// The AMD-only properties, as a machine file wrote them — read before the
/// geometry exists, turned into [`AmdOptions`] after.
#[derive(Debug, Default)]
pub(super) struct AmdProps<'a> {
    device_ext: Option<&'a [Value]>,
    banks: Option<&'a [Value]>,
    boot_flag: Option<u64>,
    secsi: Option<&'a Media>,
    secsi_lock: Option<&'a str>,
    write_protect: Option<bool>,
}

impl<'a> AmdProps<'a> {
    pub(super) fn read(r: &mut Reader<'a>) -> Result<AmdProps<'a>> {
        Ok(AmdProps {
            device_ext: r.optional_list("device-ext")?,
            banks: r.optional_list("banks")?,
            boot_flag: r.optional::<u64>("boot-flag")?,
            secsi: r.optional_media("secsi")?,
            secsi_lock: r.optional_str("secsi-lock")?,
            write_protect: r.optional::<bool>("write-protect")?,
        })
    }

    /// The first AMD-only property that was given, for the error an Intel
    /// part gives about it.
    pub(super) fn first_given(&self) -> Option<&'static str> {
        [
            ("device-ext", self.device_ext.is_some()),
            ("banks", self.banks.is_some()),
            ("boot-flag", self.boot_flag.is_some()),
            ("secsi", self.secsi.is_some()),
            ("secsi-lock", self.secsi_lock.is_some()),
            ("write-protect", self.write_protect.is_some()),
        ]
        .into_iter()
        .find_map(|(name, given)| given.then_some(name))
    }

    pub(super) fn into_options(self) -> Result<AmdOptions> {
        let number = |prop: &str, value: &Value| -> Result<u64> {
            match value {
                Value::Uint(n) | Value::Size(n) | Value::Addr(n) => Ok(*n),
                other => Err(Error::Property(format!(
                    "`{prop}` holds numbers, not {other}"
                ))),
            }
        };
        let device_ext = match self.device_ext {
            None => None,
            Some([a, b]) => {
                let a = number("device-ext", a)?;
                let b = number("device-ext", b)?;
                if a > 0xffff || b > 0xffff {
                    return Err(Error::Property(String::from(
                        "`device-ext` holds two 16-bit device-ID words",
                    )));
                }
                Some([a as u16, b as u16])
            }
            Some(other) => {
                return Err(Error::Property(format!(
                    "`device-ext` is the two device-ID words autoselect answers at 0x0e and \
                     0x0f, and {} value(s) were given",
                    other.len()
                )));
            }
        };
        let banks = match self.banks {
            None => Vec::new(),
            Some(list) => list
                .iter()
                .map(|v| number("banks", v))
                .collect::<Result<Vec<u64>>>()?,
        };
        let boot_flag = match self.boot_flag {
            None => None,
            Some(flag) => Some(u8::try_from(flag).map_err(|_| {
                Error::Property(String::from("`boot-flag` is one byte of the CFI query"))
            })?),
        };
        Ok(AmdOptions {
            device_ext,
            banks,
            boot_flag,
            secsi: self.secsi.map(|m| m.bytes().to_vec()).unwrap_or_default(),
            secsi_lock: self
                .secsi_lock
                .map_or(Ok(SecSiLock::None), SecSiLock::parse)?,
            write_protect: self.write_protect.unwrap_or(false),
        })
    }
}

// ---------------------------------------------------------------------------
// the state machine
// ---------------------------------------------------------------------------

impl Array {
    /// The AMD options; only ever called on an AMD part.
    fn amd_options(&self) -> &AmdOptions {
        self.amd
            .as_ref()
            .expect("the AMD state machine runs only on a part built with AMD options")
    }

    /// The device word address a bus offset reaches (see the module docs).
    pub(super) fn amd_word(&self, offset: u64) -> u64 {
        let device = offset / self.geom.bus_width();
        if self.geom.device_width() == 1 {
            device >> 1
        } else {
            device
        }
    }

    /// Which byte of a part's Secured Silicon Region a bus offset reaches, if
    /// it is inside the region (Table 5: device bytes `0x00`-`0xff`).
    pub(super) fn secsi_index(&self, offset: u64) -> Option<usize> {
        let dw = self.geom.device_width();
        let bw = self.geom.bus_width();
        let at = (offset / bw) * dw + (offset % bw) % dw;
        (at < SECSI_BYTES as u64).then_some(at as usize)
    }

    /// Whether sector `block` refuses program and erase: protected in the
    /// part, or one of the four `WP#` guards while `WP#` is low (§8.11).
    fn amd_protected(&self, chip: &Chip, block: u64) -> bool {
        let n = self.geom.block_count();
        let guarded = self.amd_options().write_protect && (block < 2 || block + 2 >= n);
        guarded
            || usize::try_from(block)
                .ok()
                .and_then(|b| chip.locked.get(b).copied())
                .unwrap_or(true)
    }

    /// What autoselect answers at `offset` (§8.9's table; Table 12).
    ///
    /// The low eight bits of the word address select the code — the bank
    /// address above them is the bank's own business and changes nothing here
    /// (see "What is not modelled").
    pub(super) fn amd_identifier(&self, chip: &Chip, offset: u64) -> u16 {
        let options = self.amd_options();
        match self.amd_word(offset) & 0xff {
            0x00 => self.manufacturer,
            0x01 => self.device_id,
            0x02 => self.geom.block_at(offset).map_or(0, |(block, _, _)| {
                u16::from(self.amd_protected(chip, block))
            }),
            0x03 => options.secsi_lock.indicator(),
            0x0e => options.device_ext.map_or(0, |ext| ext[0]),
            0x0f => options.device_ext.map_or(0, |ext| ext[1]),
            _ => 0,
        }
    }

    /// One bus cycle into this part's state machine.
    pub(super) fn amd_command(&self, chip: &mut Chip, offset: u64, value: u16) {
        // DQ15-DQ8 are don't care in a command cycle (Table 12, note 14).
        let cmd = value as u8;
        let at = self.amd_word(offset) & UNLOCK_MASK;
        match core::mem::replace(&mut chip.pending, Pending::None) {
            Pending::Unlock1 if cmd == CMD_UNLOCK_2 && at == UNLOCK_2 => {
                chip.pending = Pending::Unlock2;
            }
            Pending::Unlock2 if at == UNLOCK_1 => self.amd_third(chip, cmd),
            Pending::AmdProgram => self.amd_program(chip, offset, value),
            Pending::EraseSetup if cmd == CMD_UNLOCK_1 && at == UNLOCK_1 => {
                chip.pending = Pending::EraseUnlock1;
            }
            Pending::EraseUnlock1 if cmd == CMD_UNLOCK_2 && at == UNLOCK_2 => {
                chip.pending = Pending::EraseUnlock2;
            }
            Pending::EraseUnlock2 if cmd == CMD_CHIP_ERASE && at == UNLOCK_1 => {
                self.amd_erase_chip(chip, offset);
            }
            // The sixth cycle, and — within the sector-erase time-out — each
            // further sector's single cycle (§10.7). The time-out never
            // expires here, so every `SA/30` that follows lands; anything
            // else ends the run.
            Pending::EraseUnlock2 | Pending::EraseMore if cmd == CMD_SECTOR_ERASE => {
                self.amd_erase_sector(chip, offset);
                chip.pending = Pending::EraseMore;
            }
            Pending::AmdExit if cmd == CMD_EXIT => {
                // The fourth cycle of the Secured Silicon exit, or the second
                // of the unlock-bypass reset (Table 12): either way, back to
                // the array.
                chip.bypass = false;
                chip.secsi_mode = false;
                chip.mode = Mode::Array;
            }
            // A broken sequence. "Writing incorrect address and data values
            // or writing them in the improper sequence may place the device in
            // an unknown state" (§10); the known state chosen here is to take
            // the write as the start of a new one, so the reset command and a
            // fresh unlock both work from anywhere.
            _ => self.amd_first(chip, offset, value),
        }
    }

    /// A write with no sequence in progress.
    fn amd_first(&self, chip: &mut Chip, offset: u64, value: u16) {
        let cmd = value as u8;
        let at = self.amd_word(offset);
        if chip.bypass {
            // "During the unlock bypass mode, only the Unlock Bypass Program
            // and Unlock Bypass Reset commands are valid" (§10.5.1).
            match cmd {
                CMD_PROGRAM => chip.pending = Pending::AmdProgram,
                CMD_AUTOSELECT => chip.pending = Pending::AmdExit,
                _ => {}
            }
            return;
        }
        match cmd {
            // Reset: back to the array — or, in the Secured Silicon Region,
            // back to reading *that*, which the mode flag keeps (§10.2).
            CMD_RESET => chip.mode = Mode::Array,
            CMD_UNLOCK_1 if at & UNLOCK_MASK == UNLOCK_1 => chip.pending = Pending::Unlock1,
            // The query is valid from the array and from autoselect (§9).
            // A8 and up are not decoded, so a driver that writes it at 0x555
            // reaches it too.
            CMD_QUERY if at & 0xff == QUERY_ADDR && matches!(chip.mode, Mode::Array | Mode::Id) => {
                chip.mode = Mode::Cfi;
            }
            // Nothing is ever erasing, so there is nothing to suspend or
            // resume; both are accepted and change nothing.
            CMD_SUSPEND | CMD_RESUME => {}
            _ => {}
        }
    }

    /// The third cycle of an unlocked sequence, at `0x555`.
    fn amd_third(&self, chip: &mut Chip, cmd: u8) {
        match cmd {
            CMD_AUTOSELECT => {
                chip.mode = Mode::Id;
                // A `0x00` next is the Secured Silicon exit's fourth cycle.
                chip.pending = Pending::AmdExit;
            }
            CMD_PROGRAM => chip.pending = Pending::AmdProgram,
            CMD_ERASE_SETUP => chip.pending = Pending::EraseSetup,
            // "The ACC function and unlock bypass modes are not available when
            // the Secured Silicon Region is enabled" (§10.4).
            CMD_BYPASS => chip.bypass = !chip.secsi_mode,
            CMD_SECSI_ENTRY => {
                chip.secsi_mode = true;
                chip.mode = Mode::Array;
            }
            // `0xf0` here is a reset like any other; so is anything the table
            // does not list.
            _ => chip.mode = Mode::Array,
        }
    }

    /// Program one device word: bits only go from one to zero (§10.5). A
    /// protected sector is left alone, which the part reports only by DQ7
    /// polling briefly and then returning to read mode.
    fn amd_program(&self, chip: &mut Chip, offset: u64, value: u16) {
        // "When the Embedded Program algorithm is complete, that bank then
        // returns to the read mode" (§10.5).
        chip.mode = Mode::Array;
        let dw = self.geom.device_width();
        if chip.secsi_mode
            && let Some(first) = self.secsi_index(offset)
        {
            if self.amd_options().secsi_lock == SecSiLock::None {
                for i in 0..dw as usize {
                    if let Some(byte) = chip.secsi.get_mut(first + i) {
                        *byte &= (value >> (8 * i)) as u8;
                    }
                }
            }
            return;
        }
        let Some((block, _, _)) = self.geom.block_at(offset) else {
            return;
        };
        if self.amd_protected(chip, block) {
            return;
        }
        self.dirty.store(true, Ordering::Relaxed);
        for i in 0..dw {
            let at = offset + i;
            if let Ok(old) = self.array.read_u8(at) {
                let _ = self.array.write_u8(at, old & (value >> (8 * i)) as u8);
            }
        }
    }

    /// Erase one sector's worth of this part's lanes back to ones.
    fn erase_lanes(&self, base: u64, size: u64, offset: u64) {
        let dw = self.geom.device_width();
        let stride = self.geom.bus_width();
        let mut at = base + offset % stride;
        while at < base + size {
            let _ = self.array.fill(at, dw, 0xff);
            at += stride;
        }
    }

    /// Sector erase (§10.7): the sector `offset` falls in, unless it is
    /// protected. In the Secured Silicon Region, sector 0's address erases the
    /// region instead, when it is not locked.
    fn amd_erase_sector(&self, chip: &mut Chip, offset: u64) {
        let Some((block, base, size)) = self.geom.block_at(offset) else {
            return;
        };
        if chip.secsi_mode && block == 0 {
            if self.amd_options().secsi_lock == SecSiLock::None {
                chip.secsi.fill(0xff);
            }
            return;
        }
        if self.amd_protected(chip, block) {
            return;
        }
        self.dirty.store(true, Ordering::Relaxed);
        self.erase_lanes(base, size, offset);
    }

    /// Chip erase (§10.6): every sector that is not protected.
    fn amd_erase_chip(&self, chip: &mut Chip, offset: u64) {
        let mut block = 0u64;
        let mut base = 0u64;
        for region in self.geom.regions() {
            for _ in 0..region.count {
                if !self.amd_protected(chip, block) {
                    self.dirty.store(true, Ordering::Relaxed);
                    self.erase_lanes(base, region.size, offset);
                }
                block += 1;
                base += region.size;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the query
// ---------------------------------------------------------------------------

/// Where the small sectors are, as the query's boot flag says it (Table 11,
/// `0x4f`): uniform, bottom, top, or both.
fn derived_boot_flag(geom: &Geometry) -> u8 {
    let regions = geom.regions();
    let largest = regions.iter().map(|r| r.size).max().unwrap_or(0);
    let bottom = regions.first().is_some_and(|r| r.size < largest);
    let top = regions.last().is_some_and(|r| r.size < largest);
    match (bottom, top) {
        (true, true) => 0x04,
        (true, false) => 0x02,
        (false, true) => 0x03,
        (false, false) => 0x00,
    }
}

/// The query structure of **one** AMD-set part (S29JL064J Tables 8-11).
///
/// The system-interface fields — voltages, typical and maximum timeouts, the
/// ACC supply — are the S29JL064J's own. A driver reads them only to size its
/// timeouts, which are never approached here (module docs, "Time").
pub(super) fn build_query(geom: &Geometry, options: &AmdOptions) -> Vec<u8> {
    let regions = geom.regions().len();
    // The primary extended table is at 0x40 on the S29JL064J (Table 8), which
    // leaves room for four region descriptors; a part with more pushes it on.
    let extended = (QUERY_REGIONS + regions * 4).max(0x40);
    let banks = options.banks.len();
    let mut q = alloc::vec![0u8; extended + 0x18 + banks];

    // Table 8: the identification string.
    q[0x10] = b'Q';
    q[0x11] = b'R';
    q[0x12] = b'Y';
    // Primary command set: AMD/Fujitsu standard.
    q[0x13] = 0x02;
    q[0x14] = 0x00;
    q[0x15] = extended as u8;
    q[0x16] = (extended >> 8) as u8;

    // Table 9: the system interface.
    q[0x1b] = 0x27; // Vcc min 2.7 V
    q[0x1c] = 0x36; // Vcc max 3.6 V
    q[0x1f] = 0x03; // typical word program 8 us
    q[0x20] = 0x00; // no buffered program
    q[0x21] = 0x09; // typical sector erase 512 ms
    q[0x22] = 0x0f; // typical chip erase 32 s
    q[0x23] = 0x04; // max word program 16x
    q[0x25] = 0x04; // max sector erase 16x

    // Table 10: the geometry, per part.
    q[0x27] = log2(geom.size() / geom.interleave());
    q[0x28] = 0x02; // x8/x16
    q[0x29] = 0x00;
    q[0x2c] = regions as u8;
    for (i, region) in geom.regions().iter().enumerate() {
        let at = QUERY_REGIONS + i * 4;
        let count = region.count - 1;
        let size = region.size / geom.interleave() / 256;
        q[at] = count as u8;
        q[at + 1] = (count >> 8) as u8;
        q[at + 2] = size as u8;
        q[at + 3] = (size >> 8) as u8;
    }

    // Table 11: the AMD primary extended table.
    let e = extended;
    q[e] = b'P';
    q[e + 1] = b'R';
    q[e + 2] = b'I';
    q[e + 3] = b'1';
    q[e + 4] = b'3';
    q[e + 5] = 0x0c; // unlock required; 0.11 um floating gate
    q[e + 6] = 0x02; // erase suspend: read and write
    q[e + 7] = 0x01; // sector protect: one sector per group
    q[e + 8] = 0x01; // temporary sector unprotect supported
    q[e + 9] = 0x04; // protect scheme: 29LV800 mode
    // Simultaneous operation: the number of sectors outside bank 1.
    q[e + 0x0a] = options
        .banks
        .first()
        .map_or(0, |first| (geom.block_count() - first).min(0xff) as u8);
    q[e + 0x0b] = 0x00; // no burst mode
    q[e + 0x0c] = 0x00; // no page mode
    q[e + 0x0d] = 0x85; // ACC min 8.5 V
    q[e + 0x0e] = 0x95; // ACC max 9.5 V
    q[e + 0x0f] = options.boot_flag.unwrap_or_else(|| derived_boot_flag(geom));
    q[e + 0x10] = 0x00; // no program suspend
    q[e + 0x17] = banks as u8;
    for (i, sectors) in options.banks.iter().enumerate() {
        q[e + 0x18 + i] = *sectors as u8;
    }
    q
}

#[cfg(test)]
mod tests;
