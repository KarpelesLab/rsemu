//! AmigaDOS floppy tracks: an ADF's sectors as the MFM a drive head presents,
//! and back.
//!
//! An ADF is a **sector image** — 80 cylinders × 2 sides × 11 sectors × 512
//! bytes, 901 120 bytes, cylinder 0 side 0 sector 0 first. The Amiga never reads
//! a sector: `trackdisk.device` has Paula DMA a whole raw track into chip RAM
//! and finds the sectors in it in software. So a disk image becomes a disk by
//! *encoding* each track into the cells the head would pass, and a track the
//! guest wrote becomes sectors again by decoding it. Both directions are here,
//! and neither knows anything about the drive, the controller or a filesystem.
//!
//! # The track
//!
//! From the *Amiga ROM Kernel Reference Manual: Devices* (Commodore-Amiga, 3rd
//! edition), Appendix C, "Commodore-Amiga Disk Format" and "MFM Track
//! Encoding":
//!
//! > Nulls written as a gap, then 11 or 22 sectors of data. No gaps written
//! > between sectors.
//!
//! and per sector, before encoding:
//!
//! ```text
//!   2 bytes   $00                    MFM $AAAA each
//!   2 bytes   $A1 with a missing     MFM $4489 each, "standard sync byte"
//!             clock pulse
//!   1 byte    format                 $FF, "Amiga 1.0 format"      \ one longword
//!   1 byte    track number                                        |  for encoding
//!   1 byte    sector number                                       |
//!   1 byte    sectors until end of write                          /
//!  16 bytes   OS recovery info       a block of 16 for encoding
//!   4 bytes   header checksum        a longword
//!   4 bytes   data-area checksum     a longword
//! 512 bytes   data                   a block of 512
//! ```
//!
//! 544 bytes, 1088 of MFM. "Sectors until end of write" counts down from 11 on
//! the first sector after the gap to 1 on the last, which is the manual's own
//! worked example.
//!
//! ## The encoding
//!
//! "When the data is encoded, the odd bits are encoded first, then the even
//! bits of the block": a block of `n` bytes becomes `n` bytes' worth of MFM
//! carrying bits 7, 5, 3, 1 of every byte, then `n` more carrying bits 6, 4, 2,
//! 0. Each data bit is two cells, a clock cell and then the bit:
//!
//! ```text
//!   1 -> 01
//!   0 -> 10    if following a 0
//!   0 -> 00    if following a 1
//! ```
//!
//! That rule is applied across field boundaries too — a clock depends on the
//! previous *data* bit wherever it was — so a sector whose data ends in a one
//! is followed by `$2AAA`, not `$AAAA`. The manual's `$AAAA` is the encoding
//! after a zero, which is what the gap leaves behind. The sync words are the one
//! place the rule is broken on purpose ("A1 without a clock pulse"), and the
//! manual's own value pins down the cell order: `$4489` read as clock/data
//! pairs is `01 00 01 00 10 00 10 01`, data `1010 0001`, `$A1`, so data
//! occupies the `$5555` cells of a word and clocks the `$AAAA` cells.
//!
//! ## The checksums
//!
//! **Appendix C names the two checksum fields and does not give their
//! arithmetic**, and no other Commodore document found says it either — not the
//! RKRM *Libraries and Devices* (1.3) and not the AmigaOS wiki's copy of the
//! trackdisk chapter. What is used here is an exclusive-or of 32-bit chunks,
//! taken over the region **as it lies on the disk**, data cells only:
//!
//! ```text
//!   sum = 0
//!   for each MFM longword w in the region:  sum ^= w
//!   sum &= $5555_5555
//! ```
//!
//! The header checksum covers the ten MFM longwords from the format byte to the
//! end of the recovery info; the data checksum covers the 256 of the data. In
//! decoded terms every source longword `L` contributes its odd bits
//! `(L >> 1) & $5555_5555` and its even bits `L & $5555_5555`, which is
//! [`checksum`]. The stored checksum is then encoded like any other longword,
//! so its odd half is always zero.
//!
//! The *form* — an XOR over 32-bit chunks — is how the sector checksum is
//! described publicly (techtravels.org, "re-examining XOR data checksum used on
//! amiga floppies", 2010: "the 32-bit XOR checksum based on 32-bit chunks at a
//! time", in a hardware project, not an emulator). Which chunks, and the mask,
//! were **established against Kickstart itself**, run black-box on this board
//! from the user's own media: 2.04 boots a Workbench disk encoded exactly this
//! way, and refuses the same disk with one data cell flipped in every data
//! checksum, or in every header checksum (`tests/amiga_adf.rs`, behind
//! `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_ADF_DIR`). No emulator source or
//! output, GPL or otherwise, was consulted; see `docs/platforms/amiga.md`.
//!
//! # What the track looks like from the index
//!
//! [`encode_track`] writes the gap first and the eleven sectors after it, the
//! order of the manual's "first-ever write of the track". A 300 rpm revolution
//! at two microseconds a cell is the drive's [`TRACK_BYTES`] of MFM; eleven
//! sectors take 11 968 of those, and the gap is the other 532.

use alloc::vec;
use alloc::vec::Vec;

use super::floppy::{TRACK_BYTES, TRACK_CELLS, TRACKS};

/// Bytes in a sector.
pub const SECTOR_BYTES: usize = 512;

/// Sectors on a double-density track.
pub const SECTORS: usize = 11;

/// Bytes of data a track holds.
pub const TRACK_DATA: usize = SECTOR_BYTES * SECTORS;

/// The length of a double-density ADF: every track's sectors back to back.
pub const ADF_BYTES: usize = TRACK_DATA * TRACKS;

/// The length of a high-density one, 22 sectors a track. Recognised only to
/// be refused by name: the drive on an A500 is double density.
pub const ADF_HD_BYTES: usize = ADF_BYTES * 2;

/// The sync word, "MFM encoded A1 without a clock pulse".
pub const SYNC: u16 = 0x4489;

/// The format byte, "Amiga 1.0 format".
pub const FORMAT: u8 = 0xff;

/// The OS recovery info ("sector label") area.
pub const LABEL_BYTES: usize = 16;

/// A whole encoded sector: 544 bytes of source, doubled.
pub const SECTOR_MFM_BYTES: usize = 2 * (4 + 4 + LABEL_BYTES + 4 + 4 + SECTOR_BYTES);

/// The gap, in MFM bytes: what a revolution holds beyond eleven sectors.
pub const GAP_BYTES: usize = TRACK_BYTES - SECTORS * SECTOR_MFM_BYTES;

/// The data cells of an MFM longword.
const DATA_CELLS: u32 = 0x5555_5555;

/// The checksum of a region of source bytes, as [`encode_track`] stores it.
///
/// `region` is read as big-endian longwords; its length must be a multiple of
/// four, which both regions a sector has are. See the module docs for what this
/// is and where it came from.
///
/// # Panics
///
/// If `region.len()` is not a multiple of four.
#[must_use]
pub fn checksum(region: &[u8]) -> u32 {
    assert!(
        region.len().is_multiple_of(4),
        "a checksum region is whole longwords"
    );
    let mut sum = 0u32;
    for chunk in region.as_chunks::<4>().0 {
        let long = u32::from_be_bytes(*chunk);
        // The odd bits as the odd half's data cells hold them, and the even
        // bits as the even half's do.
        sum ^= (long >> 1) ^ long;
    }
    sum & DATA_CELLS
}

// ---------------------------------------------------------------------------
// encoding
// ---------------------------------------------------------------------------

/// Cells, most significant first, with the previous data bit remembered for
/// the clock rule.
struct Encoder {
    out: Vec<u8>,
    cells: usize,
    last: bool,
}

impl Encoder {
    fn new() -> Encoder {
        Encoder {
            out: vec![0; TRACK_BYTES],
            cells: 0,
            last: false,
        }
    }

    fn cell(&mut self, on: bool) {
        if on {
            self.out[self.cells / 8] |= 0x80 >> (self.cells % 8);
        }
        self.cells += 1;
    }

    /// One data bit and the clock in front of it.
    fn bit(&mut self, bit: bool) {
        self.cell(!self.last && !bit);
        self.cell(bit);
        self.last = bit;
    }

    /// Sixteen cells exactly as given: the sync word, which breaks the rule.
    fn raw(&mut self, word: u16) {
        for i in (0..16).rev() {
            self.cell(word >> i & 1 != 0);
        }
        self.last = word & 1 != 0;
    }

    /// A block: every byte's odd bits, then every byte's even bits.
    fn block(&mut self, bytes: &[u8]) {
        for shift in [7, 6] {
            for byte in bytes {
                for i in (0..4).map(|k| shift - 2 * k) {
                    self.bit(byte >> i & 1 != 0);
                }
            }
        }
    }
}

/// Encode one track's sectors as the MFM a head passes over, from the index.
///
/// `track` is the track number the headers carry (cylinder × 2 + side) and
/// `data` the track's [`TRACK_DATA`] bytes, sector 0 first. The result is
/// [`TRACK_BYTES`] long: the gap, then sectors 0 to 10.
///
/// # Panics
///
/// If `data` is not [`TRACK_DATA`] bytes.
#[must_use]
pub fn encode_track(track: u8, data: &[u8]) -> Vec<u8> {
    assert_eq!(data.len(), TRACK_DATA, "a track is eleven sectors");
    let mut e = Encoder::new();
    for _ in 0..GAP_BYTES * 4 {
        e.bit(false);
    }
    for (sector, bytes) in data.as_chunks::<SECTOR_BYTES>().0.iter().enumerate() {
        let info = [FORMAT, track, sector as u8, (SECTORS - sector) as u8];
        let label = [0u8; LABEL_BYTES];
        let mut header = [0u8; 4 + LABEL_BYTES];
        header[..4].copy_from_slice(&info);
        header[4..].copy_from_slice(&label);

        e.block(&[0, 0]);
        e.raw(SYNC);
        e.raw(SYNC);
        e.block(&info);
        e.block(&label);
        e.block(&checksum(&header).to_be_bytes());
        e.block(&checksum(bytes).to_be_bytes());
        e.block(bytes);
    }
    debug_assert_eq!(e.cells as u64, TRACK_CELLS, "a track is one revolution");
    e.out
}

// ---------------------------------------------------------------------------
// decoding
// ---------------------------------------------------------------------------

/// One sector found on a track, whether or not it checked out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sector {
    /// The format byte.
    pub format: u8,
    /// The track number in the header.
    pub track: u8,
    /// The sector number in the header.
    pub sector: u8,
    /// Sectors until the end of the write.
    pub until_gap: u8,
    /// The OS recovery info.
    pub label: [u8; LABEL_BYTES],
    /// Whether the header agreed with its checksum.
    pub header_ok: bool,
    /// Whether the data agreed with its checksum.
    pub data_ok: bool,
    /// The data.
    pub data: Vec<u8>,
    /// The cell the first sync word starts at.
    pub at: u64,
}

impl Sector {
    /// Whether this is an AmigaDOS sector for `track` whose header and data
    /// both check out.
    #[must_use]
    pub fn good_for(&self, track: u8) -> bool {
        self.header_ok
            && self.data_ok
            && self.format == FORMAT
            && self.track == track
            && usize::from(self.sector) < SECTORS
    }
}

/// Cells read round a track, which has no end.
struct Cells<'a> {
    mfm: &'a [u8],
}

impl Cells<'_> {
    fn len(&self) -> u64 {
        self.mfm.len() as u64 * 8
    }

    fn cell(&self, at: u64) -> u32 {
        let at = at % self.len();
        u32::from(self.mfm[(at / 8) as usize] >> (7 - at % 8) & 1)
    }

    fn word(&self, at: u64) -> u16 {
        (0..16).fold(0, |w, i| (w << 1) | self.cell(at + i) as u16)
    }

    /// Decode a block of `out.len()` bytes starting at cell `at`: the odd half
    /// then the even half.
    fn block(&self, at: u64, out: &mut [u8]) {
        out.fill(0);
        let n = out.len() as u64;
        for (half, shift) in [(0u64, 7u32), (1, 6)] {
            for (i, byte) in out.iter_mut().enumerate() {
                for k in 0..4u64 {
                    // Each byte is four data bits, eight cells; the data cell
                    // is the second of each pair.
                    let cell = at + half * 8 * n + i as u64 * 8 + 2 * k + 1;
                    *byte |= (self.cell(cell) as u8) << (shift - 2 * k as u32);
                }
            }
        }
    }
}

/// Every sector on an MFM track, in the order they pass the head from the
/// index.
///
/// The track is read as a loop, so a sector the index falls in the middle of is
/// found whole — a guest's write starts wherever the disk happened to be. A
/// sync mark whose header does not check out is reported with `header_ok`
/// false and scanning carries on one cell later; a sector whose header does is
/// skipped over whole.
#[must_use]
pub fn scan_track(mfm: &[u8]) -> Vec<Sector> {
    let cells = Cells { mfm };
    let total = cells.len();
    let mut found = Vec::new();
    let mut at = 0u64;
    while at < total {
        if cells.word(at) != SYNC {
            at += 1;
            continue;
        }
        // Past every sync word in a row: the manual writes two, and the first
        // sync matched may be either of them.
        let mut body = at + 16;
        while cells.word(body) == SYNC {
            body += 16;
        }
        let mut info = [0u8; 4];
        let mut label = [0u8; LABEL_BYTES];
        let mut sums = [0u8; 8];
        cells.block(body, &mut info);
        cells.block(body + 64, &mut label);
        cells.block(body + 64 + 256, &mut sums[..4]);
        cells.block(body + 64 + 256 + 64, &mut sums[4..]);
        let mut header = [0u8; 4 + LABEL_BYTES];
        header[..4].copy_from_slice(&info);
        header[4..].copy_from_slice(&label);
        let header_sum = u32::from_be_bytes([sums[0], sums[1], sums[2], sums[3]]);
        let data_sum = u32::from_be_bytes([sums[4], sums[5], sums[6], sums[7]]);
        let header_ok = checksum(&header) == header_sum;
        let mut data = vec![0u8; SECTOR_BYTES];
        let data_at = body + 64 + 256 + 128;
        cells.block(data_at, &mut data);
        let data_ok = checksum(&data) == data_sum;
        found.push(Sector {
            format: info[0],
            track: info[1],
            sector: info[2],
            until_gap: info[3],
            label,
            header_ok,
            data_ok,
            data,
            at,
        });
        at = if header_ok {
            data_at + 16 * SECTOR_BYTES as u64
        } else {
            at + 1
        };
    }
    found
}

/// The good sectors of track `track`, by sector number.
///
/// The first good copy of each sector wins. A sector that is missing, fails a
/// checksum, is not in the AmigaDOS format or names another track is `None`.
#[must_use]
pub fn decode_track(mfm: &[u8], track: u8) -> Vec<Option<Vec<u8>>> {
    let mut out = vec![None; SECTORS];
    for sector in scan_track(mfm) {
        if sector.good_for(track) {
            let slot = &mut out[usize::from(sector.sector)];
            if slot.is_none() {
                *slot = Some(sector.data);
            }
        }
    }
    out
}

/// Where a track's sectors start in an ADF.
#[must_use]
pub const fn track_offset(track: usize) -> usize {
    track * TRACK_DATA
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Sector data that is different in every byte of every sector, so a
    /// sector decoded into the wrong place cannot pass.
    fn pattern(track: u8) -> Vec<u8> {
        (0..TRACK_DATA)
            .map(|i| (i as u8).wrapping_mul(31) ^ (i >> 9) as u8 ^ track)
            .collect()
    }

    #[test]
    fn the_layout_adds_up_to_one_revolution() {
        assert_eq!(SECTOR_MFM_BYTES, 1088);
        assert_eq!(GAP_BYTES, 532);
        assert_eq!(ADF_BYTES, 901_120);
        assert_eq!(encode_track(0, &pattern(0)).len(), TRACK_BYTES);
    }

    #[test]
    fn every_sector_starts_with_the_manuals_aaaa_4489_4489() {
        let mfm = encode_track(7, &pattern(7));
        let at = GAP_BYTES;
        assert_eq!(
            &mfm[at..at + 8],
            &[0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x89, 0x44, 0x89]
        );
        // The gap is zeroes after zeroes.
        assert!(mfm[..GAP_BYTES].iter().all(|b| *b == 0xaa));
    }

    #[test]
    fn a_track_round_trips_through_its_own_checksums() {
        for track in [0u8, 1, 80, 159] {
            let data = pattern(track);
            let mfm = encode_track(track, &data);
            let sectors = scan_track(&mfm);
            assert_eq!(sectors.len(), SECTORS, "track {track}");
            for (i, s) in sectors.iter().enumerate() {
                assert!(s.good_for(track), "track {track} sector {i}: {s:?}");
                assert_eq!(usize::from(s.sector), i);
                assert_eq!(
                    usize::from(s.until_gap),
                    SECTORS - i,
                    "the manual's countdown"
                );
                assert_eq!(s.data, data[i * SECTOR_BYTES..(i + 1) * SECTOR_BYTES]);
            }
            let decoded = decode_track(&mfm, track);
            let joined: Vec<u8> = decoded.into_iter().flat_map(Option::unwrap).collect();
            assert_eq!(joined, data);
        }
    }

    #[test]
    fn the_clock_rule_holds_everywhere_but_the_sync_words() {
        // MFM never has two adjacent ones, and never more than three zeroes in
        // a row — except inside $4489, whose missing clock makes four.
        let mfm = encode_track(3, &pattern(3));
        let cells = Cells { mfm: &mfm };
        let syncs: Vec<u64> = scan_track(&mfm).iter().map(|s| s.at).collect();
        let in_sync = |at: u64| syncs.iter().any(|s| at >= *s && at < *s + 32);
        let mut zeroes = 0;
        for at in 0..cells.len() {
            let c = cells.cell(at);
            assert!(
                !(c == 1 && cells.cell(at + 1) == 1),
                "adjacent ones at {at}"
            );
            zeroes = if c == 0 { zeroes + 1 } else { 0 };
            if !in_sync(at) {
                assert!(zeroes <= 3, "a run of {zeroes} zeroes at {at}");
            }
        }
    }

    #[test]
    fn a_data_block_decodes_to_the_manuals_odd_then_even_split() {
        // $80000001: bit 31 is odd, bit 0 even. The odd half carries bit 31 in
        // its first data cell; the even half carries bit 0 in its last.
        let mut e = Encoder::new();
        e.block(&0x8000_0001u32.to_be_bytes());
        let odd = u32::from_be_bytes([e.out[0], e.out[1], e.out[2], e.out[3]]);
        let even = u32::from_be_bytes([e.out[4], e.out[5], e.out[6], e.out[7]]);
        assert_eq!(odd & DATA_CELLS, 0x4000_0000);
        assert_eq!(even & DATA_CELLS, 0x0000_0001);
        // And the checksum is those data cells XORed.
        assert_eq!(checksum(&0x8000_0001u32.to_be_bytes()), 0x4000_0001);
    }

    #[test]
    fn the_checksum_is_the_xor_of_the_encoded_longwords_data_cells() {
        // The definition in the module docs, computed the long way: encode the
        // region, XOR every MFM longword, mask the clocks off.
        let data = pattern(9);
        let region = &data[..SECTOR_BYTES];
        let mut e = Encoder::new();
        e.block(region);
        let mut sum = 0u32;
        for chunk in e.out[..2 * SECTOR_BYTES].as_chunks::<4>().0 {
            sum ^= u32::from_be_bytes(*chunk);
        }
        assert_eq!(sum & DATA_CELLS, checksum(region));
    }

    #[test]
    fn one_flipped_cell_fails_the_sector_it_is_in_and_no_other() {
        let data = pattern(2);
        let mut mfm = encode_track(2, &data);
        // A data cell in sector 4's data area.
        let sector4 = GAP_BYTES + 4 * SECTOR_MFM_BYTES;
        mfm[sector4 + 8 + 56 + 100] ^= 0x01;
        let decoded = decode_track(&mfm, 2);
        for (i, s) in decoded.iter().enumerate() {
            assert_eq!(s.is_none(), i == 4, "sector {i}");
        }
        // A flipped header cell fails the header instead.
        let mut mfm = encode_track(2, &data);
        mfm[sector4 + 8 + 3] ^= 0x04;
        assert!(decode_track(&mfm, 2)[4].is_none());
    }

    #[test]
    fn a_track_written_from_anywhere_is_found_across_the_index() {
        // Rotate the track so the index falls inside sector 6: a write that
        // began at a random point of the revolution.
        let data = pattern(5);
        let mfm = encode_track(5, &data);
        let cut = GAP_BYTES + 6 * SECTOR_MFM_BYTES + 300;
        let rotated: Vec<u8> = mfm[cut..].iter().chain(&mfm[..cut]).copied().collect();
        let decoded = decode_track(&rotated, 5);
        assert!(decoded.iter().all(Option::is_some));
        assert_eq!(decoded[6].as_deref(), Some(&data[6 * 512..7 * 512]));
    }

    #[test]
    fn a_sector_for_another_track_is_not_this_tracks() {
        let mfm = encode_track(4, &pattern(4));
        assert!(decode_track(&mfm, 5).iter().all(Option::is_none));
    }
}
