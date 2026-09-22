//! MFM: the bit stream on a 1.44 MB high-density disk, and the IBM track
//! layout a Macintosh with a SWIM writes on one.
//!
//! An 800K Apple disk is zoned GCR ([`super::gcr`]) — constant *linear*
//! density, so the spindle changes speed as the head steps. A 1.44 MB disk is
//! the IBM format instead: **constant angular velocity**, 300 revolutions a
//! minute from cylinder 0 to cylinder 79, eighteen 512-byte sectors on every
//! one of the 160 tracks.
//!
//! ```text
//!   80 cylinders x 2 heads x 18 sectors x 512 bytes = 1,474,560 bytes
//! ```
//!
//! # Cells, and why this module's clock is 1 MHz where the IWM's is 500 kHz
//!
//! MFM spends **two bit cells on every data bit**: a clock cell and then a data
//! cell. The data cell carries the bit; the clock cell carries a pulse only
//! where two zeros meet, which is what keeps a run of zeros self-clocking. So a
//! 500 kbit/s data rate — the rate every high-density 3.5-inch drive runs at —
//! is a **1 MHz cell rate**, and one tick of this module's clock is one cell,
//! exactly as one tick of the IWM's is one GCR cell.
//!
//! At 300 rpm that is
//!
//! ```text
//!   1,000,000 cells/s / (300/60 rev/s) = 200,000 cells a revolution
//! ```
//!
//! which is [`CELLS_PER_REVOLUTION`], and 12,500 bytes of MFM on a track
//! against the 11,990 the eighteen sectors and their gaps need. The slack is
//! the trailing gap.
//!
//! # The two marks, derived rather than quoted
//!
//! An MFM address mark is a byte whose **clock pulse is deliberately missing**,
//! so it cannot occur anywhere in ordinary data and a controller can
//! resynchronise on it. There are two, and this module generates both from the
//! encoding rule rather than from a table, so the numbers are arithmetic anyone
//! can check:
//!
//! ```text
//!   $A1 = 1010_0001, encoded normally   -> 01 00 01 00 10 10 10 01 = $44A9
//!         with the clock pulse between data bits 4 and 5 suppressed
//!                                       -> 01 00 01 00 10 00 10 01 = $4489
//!
//!   $C2 = 1100_0010, encoded normally   -> 01 01 00 10 10 10 01 00 = $52A4
//!         with the clock pulse between data bits 3 and 4 suppressed
//!                                       -> 01 01 00 10 00 10 01 00 = $5224
//! ```
//!
//! `$4489` prefixes an ID or a data field, three of them in a row; `$5224`
//! prefixes the index mark once a revolution. [`sync_cells`] is the generator
//! and `tests.rs` asserts both values against the hand arithmetic above, which
//! is what makes them a derivation and not a recollection.
//!
//! # The track
//!
//! ```text
//!   80 x $4E          gap 4a
//!   12 x $00          sync
//!    3 x $4489        the index mark's missing-clock $C2... (see below)
//!        $FC          the index address mark
//!   50 x $4E          gap 1
//!   for each of 18 sectors:
//!     12 x $00        sync
//!      3 x $4489      the ID mark's $A1
//!          $FE        the ID address mark
//!          C H R N    cylinder, head, sector, and 2 for "512 bytes"
//!          CRC        over $A1 $A1 $A1 $FE C H R N
//!     22 x $4E        gap 2
//!     12 x $00        sync
//!      3 x $4489      the data mark's $A1
//!          $FB        the data address mark
//!    512 x data
//!          CRC        over $A1 $A1 $A1 $FB and the data
//!     84 x $4E        gap 3
//!   $4E to the end    gap 4b
//! ```
//!
//! The index mark's three sync bytes are `$C2` rather than `$A1`, so they carry
//! `$5224`. Nothing in the read path here looks for it — a controller finds
//! sectors by their own marks — but a formatter writes it and a disk without
//! one is not the format.
//!
//! The CRC is **CRC-16/CCITT**: the polynomial `x^16 + x^12 + x^5 + 1`
//! (`$1021`), seeded with `$FFFF`, no reflection and no final xor, and it
//! covers the three sync bytes as *data* (`$A1`, not `$4489`) together with the
//! address mark and the field. [`crc16`] is it.
//!
//! No emulator source was consulted (`ROADMAP.md` §1, `CLAUDE.md`).

use alloc::vec::Vec;

pub use super::gcr::Track;

/// How many cylinders a 1.44 MB disk has.
pub const CYLINDERS: usize = 80;
/// The highest cylinder a mechanism steps to.
pub const MAX_CYLINDER: u8 = 79;
/// How many heads.
pub const SIDES: usize = 2;
/// How many sectors on every track — the same number on every one of them,
/// which is the whole difference from Apple's zoned GCR.
pub const SECTORS: usize = 18;
/// How many bytes of data in a sector.
pub const DATA_BYTES: usize = 512;
/// How many bytes a whole 1.44 MB disk holds.
pub const BYTES: usize = CYLINDERS * SIDES * SECTORS * DATA_BYTES;

/// How many MFM cells go past the head in one revolution: a 1 MHz cell rate at
/// 300 revolutions a minute. See the module docs.
pub const CELLS_PER_REVOLUTION: usize = 200_000;

/// How many revolutions a minute a high-density mechanism turns at — the same
/// on every cylinder, unlike an 800K one.
pub const RPM: u32 = 300;

/// The `N` field of an ID: 2 means 512 bytes, as `128 << N`.
pub const SIZE_CODE: u8 = 2;

/// The index address mark that follows the three `$C2` sync bytes.
pub const IAM: u8 = 0xfc;
/// The ID address mark that follows the three `$A1`s.
pub const IDAM: u8 = 0xfe;
/// The data address mark that follows the three `$A1`s.
pub const DAM: u8 = 0xfb;
/// A deleted-data address mark. Recognised on a read; never written here.
pub const DDAM: u8 = 0xf8;

/// The sync byte an ID or data field is prefixed with, three times over.
pub const SYNC_A1: u8 = 0xa1;
/// And the one an index mark is.
pub const SYNC_C2: u8 = 0xc2;

/// The gap byte a formatter fills with.
pub const GAP_BYTE: u8 = 0x4e;

/// Gap 4a, before the index mark.
pub const GAP4A: usize = 80;
/// Gap 1, between the index mark and the first sector.
pub const GAP1: usize = 50;
/// Gap 2, between an ID field and its data field.
pub const GAP2: usize = 22;
/// Gap 3, between one sector and the next. The number a formatter leaves for
/// eighteen sectors at 500 kbit/s.
pub const GAP3: usize = 84;
/// How many zero bytes precede a mark.
pub const SYNC_BYTES: usize = 12;

/// CRC-16/CCITT over `bytes`, continuing from `seed`.
///
/// `x^16 + x^12 + x^5 + 1`, most significant bit first, no reflection and no
/// final xor. A field's CRC is seeded with `$FFFF` and covers the three sync
/// bytes as data — `$A1 $A1 $A1` — then the address mark, then the field.
#[must_use]
pub fn crc16(seed: u16, bytes: &[u8]) -> u16 {
    let mut crc = seed;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            // Shifting a one out of the top means the polynomial divides in.
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The sixteen cells of one MFM byte: a clock cell and a data cell per data
/// bit, most significant bit first.
///
/// `prev` is the data bit before this byte — a clock pulse goes only where two
/// zeros meet — and `suppress` names a data-bit position (0 for the most
/// significant) whose clock pulse is left out, which is what makes an address
/// mark an address mark.
///
/// The rule, and the whole of it: `clock = !prev && !bit`.
#[must_use]
pub fn cells(byte: u8, prev: bool, suppress: Option<u32>) -> u16 {
    let mut out = 0u16;
    let mut prev = prev;
    for i in 0..8u32 {
        let bit = byte & (0x80 >> i) != 0;
        let mut clock = !prev && !bit;
        if suppress == Some(i) {
            clock = false;
        }
        out = (out << 2) | (u16::from(clock) << 1) | u16::from(bit);
        prev = bit;
    }
    out
}

/// The sixteen cells of one of the two sync bytes, with its clock pulse
/// missing: `$4489` for [`SYNC_A1`] and `$5224` for [`SYNC_C2`].
///
/// Which pulse is left out is the format's, and the module docs show the
/// arithmetic: between data bits 4 and 5 for `$A1`, between 3 and 4 for `$C2`.
/// A pair's clock cell is the one *before* its data bit, so those are the
/// clock cells of positions 5 and 4.
///
/// Any other byte gets its ordinary encoding, because only those two are marks.
#[must_use]
pub fn sync_cells(byte: u8) -> u16 {
    match byte {
        SYNC_A1 => cells(SYNC_A1, false, Some(5)),
        SYNC_C2 => cells(SYNC_C2, false, Some(4)),
        other => cells(other, false, None),
    }
}

/// One sector as it sits on the medium.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sector {
    /// The `C` of the ID field.
    pub cylinder: u8,
    /// Its `H`.
    pub head: u8,
    /// Its `R`, **one-based**: an IBM track's sectors are numbered 1 to 18.
    pub sector: u8,
    /// Its `N`; 2 for the 512-byte sectors a 1.44 MB disk carries.
    pub size_code: u8,
    /// The data field.
    pub data: Vec<u8>,
    /// Whether the data field carried [`DDAM`] rather than [`DAM`].
    pub deleted: bool,
}

impl Sector {
    /// A 512-byte sector at `(cylinder, head, sector)`, `sector` one-based.
    #[must_use]
    pub fn new(cylinder: u8, head: u8, sector: u8, data: &[u8]) -> Sector {
        Sector {
            cylinder,
            head,
            sector,
            size_code: SIZE_CODE,
            data: data.to_vec(),
            deleted: false,
        }
    }

    /// The four bytes of the ID field: `C H R N`.
    #[must_use]
    pub fn id(&self) -> [u8; 4] {
        [self.cylinder, self.head, self.sector, self.size_code]
    }
}

/// Appends MFM to a [`Track`], keeping the last data bit so the next byte's
/// clock cell is right.
#[derive(Debug)]
struct Writer<'a> {
    track: &'a mut Track,
    prev: bool,
}

impl Writer<'_> {
    /// One ordinary byte.
    fn byte(&mut self, byte: u8) {
        self.push(cells(byte, self.prev, None));
        self.prev = byte & 1 != 0;
    }

    /// `n` of them.
    fn fill(&mut self, byte: u8, n: usize) {
        for _ in 0..n {
            self.byte(byte);
        }
    }

    /// One of the two sync bytes, clock pulse and all.
    fn sync(&mut self, byte: u8) {
        self.push(sync_cells(byte));
        self.prev = byte & 1 != 0;
    }

    fn push(&mut self, cells: u16) {
        for i in (0..16).rev() {
            self.track.push_bit(cells & (1 << i) != 0);
        }
    }
}

/// Lay one whole track down: gaps, index mark, and `sectors` in the order
/// given.
///
/// The track is padded with gap bytes to [`CELLS_PER_REVOLUTION`], which is
/// what makes the spindle turn at 300 rpm when one tick of the controller's
/// clock is one cell — the same arithmetic `gcr::SECTOR_CELLS` does for an 800K
/// disk, and for the same reason: **how fast the disk turns is the track's
/// length, not a number the computer writes**.
#[must_use]
pub fn encode_track(sectors: &[Sector]) -> Track {
    let mut track = Track::new();
    {
        let mut w = Writer {
            track: &mut track,
            prev: false,
        };
        w.fill(GAP_BYTE, GAP4A);
        w.fill(0x00, SYNC_BYTES);
        for _ in 0..3 {
            w.sync(SYNC_C2);
        }
        w.byte(IAM);
        w.fill(GAP_BYTE, GAP1);
        for sector in sectors {
            w.fill(0x00, SYNC_BYTES);
            for _ in 0..3 {
                w.sync(SYNC_A1);
            }
            w.byte(IDAM);
            let id = sector.id();
            for &b in &id {
                w.byte(b);
            }
            let mut head = [SYNC_A1, SYNC_A1, SYNC_A1, IDAM, 0, 0, 0, 0];
            head[4..].copy_from_slice(&id);
            let crc = crc16(0xffff, &head);
            w.byte((crc >> 8) as u8);
            w.byte(crc as u8);

            w.fill(GAP_BYTE, GAP2);
            w.fill(0x00, SYNC_BYTES);
            for _ in 0..3 {
                w.sync(SYNC_A1);
            }
            let mark = if sector.deleted { DDAM } else { DAM };
            w.byte(mark);
            for &b in &sector.data {
                w.byte(b);
            }
            let crc = crc16(
                crc16(0xffff, &[SYNC_A1, SYNC_A1, SYNC_A1, mark]),
                &sector.data,
            );
            w.byte((crc >> 8) as u8);
            w.byte(crc as u8);
            w.fill(GAP_BYTE, GAP3);
        }
        // Gap 4b: whatever is left of the revolution. A cell is half a data
        // bit, so a byte is sixteen of them.
        while w.track.len() + 16 <= CELLS_PER_REVOLUTION {
            w.byte(GAP_BYTE);
        }
        while w.track.len() < CELLS_PER_REVOLUTION {
            w.track.push_bit(false);
        }
    }
    track
}

/// Which sectors a controller would find on `track`, and the ones whose CRC
/// did not hold.
///
/// This is the read path a test uses as the encoder's oracle. It is **not**
/// what [`super::swim`] does: the chip decodes a cell at a time as the medium
/// goes past, where this walks a whole revolution at once.
#[must_use]
pub fn decode_track(track: &Track) -> (Vec<Sector>, Vec<Bad>) {
    let mut found = Vec::new();
    let mut bad = Vec::new();
    if track.is_empty() {
        return (found, bad);
    }
    // One and a bit revolutions, so a field that straddles the index is found.
    let cells = track.len();
    let a1 = sync_cells(SYNC_A1);
    let mut window = 0u16;
    let mut at = 0usize;
    let mut marks: Vec<usize> = Vec::new();
    while at < cells + 16 {
        window = (window << 1) | u16::from(track.bit(at));
        at += 1;
        if at >= 16 && window == a1 {
            marks.push(at);
        }
    }
    // Three `$A1`s in a row is a mark; the byte after the third says which.
    let mut i = 0;
    let mut pending: Option<Sector> = None;
    while i < marks.len() {
        // The third of a run: the two before it are 16 and 32 cells earlier.
        let third = marks[i];
        let run = i + 2 < marks.len() && marks[i + 1] == third + 16 && marks[i + 2] == third + 32;
        if !run {
            i += 1;
            continue;
        }
        let after = marks[i + 2];
        i += 3;
        let Some(mark) = read_byte(track, after) else {
            continue;
        };
        match mark {
            IDAM => {
                let mut id = [0u8; 4];
                for (n, slot) in id.iter_mut().enumerate() {
                    let Some(b) = read_byte(track, after + 16 * (n + 1)) else {
                        break;
                    };
                    *slot = b;
                }
                let stored = match (
                    read_byte(track, after + 16 * 5),
                    read_byte(track, after + 16 * 6),
                ) {
                    (Some(hi), Some(lo)) => u16::from(hi) << 8 | u16::from(lo),
                    _ => continue,
                };
                let mut head = [SYNC_A1, SYNC_A1, SYNC_A1, IDAM, 0, 0, 0, 0];
                head[4..].copy_from_slice(&id);
                if crc16(0xffff, &head) != stored {
                    bad.push(Bad::Id { at: after, stored });
                    continue;
                }
                pending = Some(Sector {
                    cylinder: id[0],
                    head: id[1],
                    sector: id[2],
                    size_code: id[3],
                    data: Vec::new(),
                    deleted: false,
                });
            }
            DAM | DDAM => {
                let Some(mut sector) = pending.take() else {
                    continue;
                };
                let len = DATA_BYTES.min(128usize << u32::from(sector.size_code.min(6)));
                let mut data = Vec::with_capacity(len);
                for n in 0..len {
                    let Some(b) = read_byte(track, after + 16 * (n + 1)) else {
                        break;
                    };
                    data.push(b);
                }
                if data.len() != len {
                    continue;
                }
                let stored = match (
                    read_byte(track, after + 16 * (len + 1)),
                    read_byte(track, after + 16 * (len + 2)),
                ) {
                    (Some(hi), Some(lo)) => u16::from(hi) << 8 | u16::from(lo),
                    _ => continue,
                };
                let seed = crc16(0xffff, &[SYNC_A1, SYNC_A1, SYNC_A1, mark]);
                if crc16(seed, &data) != stored {
                    bad.push(Bad::Data {
                        sector: sector.sector,
                        stored,
                    });
                    continue;
                }
                sector.deleted = mark == DDAM;
                sector.data = data;
                found.push(sector);
            }
            _ => {}
        }
    }
    (found, bad)
}

/// A field whose CRC did not hold, named so a failure says which.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bad {
    /// An ID field, at the cell its mark ended on.
    Id {
        /// Where on the track.
        at: usize,
        /// The CRC the medium carried.
        stored: u16,
    },
    /// A data field, named by the sector its ID claimed.
    Data {
        /// Which sector.
        sector: u8,
        /// The CRC the medium carried.
        stored: u16,
    },
}

/// The byte whose sixteen cells start at `at`: the odd cells are the data bits
/// and the even ones are clock.
fn read_byte(track: &Track, at: usize) -> Option<u8> {
    if track.is_empty() {
        return None;
    }
    let mut byte = 0u8;
    for n in 0..8 {
        byte = (byte << 1) | u8::from(track.bit(at + n * 2 + 1));
    }
    Some(byte)
}

/// Where block `n` of a 1.44 MB image sits: cylinder, head, and the
/// **one-based** sector number an IBM ID field carries.
///
/// Sectors run across a track, then across the two heads, then outward — the
/// same order as [`super::disk`]'s GCR mapping, and the order a PC's BIOS and a
/// Macintosh's driver both assume.
#[must_use]
pub fn place(block: usize) -> Option<(u8, u8, u8)> {
    if block >= CYLINDERS * SIDES * SECTORS {
        return None;
    }
    let sector = block % SECTORS;
    let head = (block / SECTORS) % SIDES;
    let cylinder = block / (SECTORS * SIDES);
    Some((cylinder as u8, head as u8, sector as u8 + 1))
}

/// The block number of `(cylinder, head, sector)`, `sector` one-based.
#[must_use]
pub fn block_of(cylinder: u8, head: u8, sector: u8) -> Option<usize> {
    if usize::from(cylinder) >= CYLINDERS
        || usize::from(head) >= SIDES
        || sector == 0
        || usize::from(sector) > SECTORS
    {
        return None;
    }
    Some((usize::from(cylinder) * SIDES + usize::from(head)) * SECTORS + usize::from(sector - 1))
}

#[cfg(test)]
mod tests;
