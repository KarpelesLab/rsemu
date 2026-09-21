//! Apple's 6-and-2 group-code recording: the bit stream an 800K drive's head
//! delivers, and the 524-byte sectors in it.
//!
//! This is the format, not a device. [`Track`] builds the bits for one
//! cylinder and side out of its sectors; [`decode_track`] reads them back, so
//! the two are each other's test. `mac.iwm` shifts a [`Track`] past its read
//! head and `mac.disk` says which sectors go on it.
//!
//! # Sources
//!
//! * *Guide to the Macintosh Family Hardware*, 2nd edition, chapter 9, for
//!   what GCR is and why: "all Apple II and Macintosh disk drives used
//!   group-code recording (GCR) with NRZI (non-return-to-zero, inverted)
//!   encoding … A transition always occurs for a one in the data stream. No
//!   transition occurs for a zero … Groups of three or more zeros would be
//!   difficult to distinguish using this encoding, so the disk drivers use GCR
//!   to format the data in such a way that more than two adjacent zeros don't
//!   occur. **With GCR formatting, each group of three data bytes is formatted
//!   as four 8-bit patterns.**" The same chapter gives the drive's data rate
//!   and says the IWM "converts between the NRZI serial data used to
//!   communicate with disk drives and the 8-bit parallel data used to
//!   communicate with the CPU".
//! * **US patent 4,564,941**, "Error detection system", Apple Computer Inc.,
//!   filed 8 December 1983, granted 14 January 1986, for the three-byte
//!   interleaved checksum: the rotation of `C3` ("the contents of check-sum C3
//!   are shifted one bit to the left … and the most significant bit is rotated
//!   to the least significant bit position"), the carry chain ("check-sum C1 is
//!   then set equal to check-sum C1 plus the contents of byte A (plus any carry
//!   c)"), and the scrambling of the data with it ("byte A … exclusive-ORed
//!   with the contents of check-sum C3", "byte B of the record is then
//!   exclusive-ORed with the contents of check-sum C1").
//! * Published descriptions of the on-disk layout — the self-sync run, the
//!   `$D5 $AA $96` address prologue and its five fields, the `$D5 $AA $AD` data
//!   prologue, the `$DE $AA` epilogues, and the five speed zones of 12, 11, 10,
//!   9 and 8 sectors — which are format facts and are asserted against each
//!   other in this module's tests.
//!
//! **No emulator source was consulted** (`ROADMAP.md` §1, `CLAUDE.md`). The
//! two places a description named an emulator's routine as its own source were
//! not followed.
//!
//! # The sixty-four disk bytes are derived, not transcribed
//!
//! A 6-and-2 "disk byte" carries six bits of payload in eight bits on the
//! medium, and which sixty-four of the 256 patterns qualify follows from what
//! NRZI can read back and from the need for a byte to be findable in a bit
//! stream with no other framing:
//!
//! 1. **Bit 7 is one.** That is what the shifter looks for: it holds at zero
//!    until a one arrives and latches when the one reaches the top, which is
//!    how eight bits are picked out of a stream that has no byte boundaries in
//!    it.
//! 2. **No more than two zeros in a row**, which is the Guide's own rule above.
//! 3. **No more than one *pair* of zeros.**
//! 4. **At least two ones in a row somewhere below bit 7.**
//!
//! Those four leave exactly sixty-four patterns — [`DISK_BYTES`] is generated
//! from them and this module's tests assert the count — and they leave out
//! `$D5` and `$AA` precisely because those two alternate. That is not a
//! coincidence: it
//! is why the two marks a track is searched for can be those two bytes and can
//! never be mistaken for payload.

use alloc::vec;
use alloc::vec::Vec;

/// How many bytes a Macintosh sector holds: 512 of data and twelve of tag.
pub const SECTOR_BYTES: usize = 524;
/// How many of those twelve come first: the tag the file system keeps.
pub const TAG_BYTES: usize = 12;
/// And the rest, which is the block a file system reads and writes.
pub const DATA_BYTES: usize = 512;

/// How many disk bytes [`SECTOR_BYTES`] becomes: three bytes to four, rounded
/// up, because 524 is not a multiple of three.
pub const SECTOR_NIBBLES: usize = 699;

/// The highest cylinder an 800K mechanism steps to.
pub const MAX_TRACK: u8 = 79;
/// How many cylinders one has.
pub const TRACKS: usize = 80;

/// The address field's prologue.
pub const ADDRESS_MARK: [u8; 3] = [0xd5, 0xaa, 0x96];
/// The data field's.
pub const DATA_MARK: [u8; 3] = [0xd5, 0xaa, 0xad];
/// What ends both fields.
pub const EPILOGUE: [u8; 2] = [0xde, 0xaa];

/// The format byte of a double-sided Macintosh disk, 800K.
pub const FORMAT_800K: u8 = 0x22;
/// The format byte of a single-sided one, 400K.
pub const FORMAT_400K: u8 = 0x02;

/// How many self-sync bytes come before an address field.
const SYNC_BEFORE_ADDRESS: usize = 36;
/// How many come between the address field and the data field.
const SYNC_BEFORE_DATA: usize = 5;

/// How many sectors each of the five speed zones holds, outermost first.
///
/// A 3.5-inch Apple disk turns slower the further out the head is, so that
/// every track carries the same bits per inch and the outer ones carry more
/// sectors. Sixteen cylinders a zone, five zones, eighty cylinders:
/// `(12 + 11 + 10 + 9 + 8) * 16 = 800` sectors a side, which is where 400K and
/// 800K come from.
pub const ZONE_SECTORS: [u8; 5] = [12, 11, 10, 9, 8];

/// How many cylinders one speed zone covers.
pub const ZONE_TRACKS: u8 = 16;

/// How many sectors cylinder `track` holds.
#[must_use]
pub fn sectors_on(track: u8) -> u8 {
    ZONE_SECTORS[usize::from(track.min(MAX_TRACK) / ZONE_TRACKS)]
}

/// How many sectors one side of a disk holds: 800.
#[must_use]
pub fn sectors_per_side() -> usize {
    (0..TRACKS).map(|t| usize::from(sectors_on(t as u8))).sum()
}

// ---------------------------------------------------------------------------
// the sixty-four disk bytes
// ---------------------------------------------------------------------------

/// Whether `code` is one of the sixty-four patterns a 6-and-2 payload byte may
/// take. The four rules are in this module's docs.
const fn is_disk_byte(code: u8) -> bool {
    if code & 0x80 == 0 {
        return false;
    }
    let mut i = 7;
    let mut zeros = 0u8;
    let mut pairs = 0u8;
    let mut ones = 0u8;
    let mut ones_pair = false;
    while i > 0 {
        i -= 1;
        if code & (1 << i) == 0 {
            if zeros == 1 {
                pairs += 1;
            }
            zeros += 1;
            if zeros > 2 {
                return false;
            }
            ones = 0;
        } else {
            zeros = 0;
            ones += 1;
            if ones >= 2 {
                ones_pair = true;
            }
        }
    }
    pairs <= 1 && ones_pair
}

/// The sixty-four disk bytes, in ascending order: index by a six-bit payload.
pub static DISK_BYTES: [u8; 64] = build_disk_bytes();

const fn build_disk_bytes() -> [u8; 64] {
    let mut table = [0u8; 64];
    let mut code = 0u16;
    let mut n = 0usize;
    while code < 256 {
        if is_disk_byte(code as u8) {
            // A table that is not exactly sixty-four long is a broken rule, and
            // the `tests` module asserts it; here the index simply stops.
            if n < 64 {
                table[n] = code as u8;
                n += 1;
            }
        }
        code += 1;
    }
    table
}

/// The six-bit payload `code` carries, or `None` if it is not a disk byte.
#[must_use]
pub fn payload_of(code: u8) -> Option<u8> {
    DISK_BYTES.iter().position(|&c| c == code).map(|i| i as u8)
}

// ---------------------------------------------------------------------------
// the checksum
// ---------------------------------------------------------------------------

/// The three-byte interleaved checksum of patent 4,564,941, and the data
/// scrambled with it.
///
/// Both come out of one pass, because the patent's scheme is one pass: each
/// byte is added into one of the three sums and then written out exclusive-ORed
/// with a *different* one, so that a single bad byte damages the sums and the
/// bytes after it rather than only itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Checksum {
    /// `C1`, `C2`, `C3`.
    pub sums: [u8; 3],
}

impl Checksum {
    /// Scramble `sector` and give back the bytes to write and the checksum.
    ///
    /// The patent's order, per group of three bytes A, B, C:
    ///
    /// * rotate `C3` left one place, the bit that comes off the top becoming
    ///   the carry into `C1`;
    /// * `C1 += A + carry`, and A is written as `A ^ C3`;
    /// * `C2 += B + carry`, and B is written as `B ^ C1`;
    /// * `C3 += C + carry`, and C is written as `C ^ C2`.
    ///
    /// A trailing group of one or two bytes stops where it runs out, which is
    /// what a 524-byte sector needs: 524 is 174 groups and two bytes over.
    #[must_use]
    pub fn scramble(sector: &[u8]) -> (Vec<u8>, Checksum) {
        let mut out = Vec::with_capacity(sector.len());
        let (mut c1, mut c2, mut c3) = (0u8, 0u8, 0u8);
        for group in sector.chunks(3) {
            // "The contents of check-sum C3 are shifted one bit to the left …
            // and the most significant bit is rotated to the least significant
            // bit position."
            let mut carry = u16::from(c3 >> 7);
            c3 = c3.rotate_left(1);

            let sum = u16::from(c1) + u16::from(group[0]) + carry;
            c1 = sum as u8;
            carry = sum >> 8;
            out.push(group[0] ^ c3);

            if group.len() > 1 {
                let sum = u16::from(c2) + u16::from(group[1]) + carry;
                c2 = sum as u8;
                carry = sum >> 8;
                out.push(group[1] ^ c1);
            }
            if group.len() > 2 {
                let sum = u16::from(c3) + u16::from(group[2]) + carry;
                c3 = sum as u8;
                out.push(group[2] ^ c2);
            }
        }
        (out, Checksum { sums: [c1, c2, c3] })
    }

    /// The other direction: unscramble what was read and give back the checksum
    /// it should have had.
    ///
    /// "The retrieved quantity A⊕C3 is exclusive-ORed with the contents of
    /// check-sum C3 … thereby resulting in a true retrieval of the contents of
    /// byte A." Which is the same pass with the exclusive-OR before the
    /// addition instead of after it.
    #[must_use]
    pub fn unscramble(raw: &[u8]) -> (Vec<u8>, Checksum) {
        let mut out = Vec::with_capacity(raw.len());
        let (mut c1, mut c2, mut c3) = (0u8, 0u8, 0u8);
        for group in raw.chunks(3) {
            let mut carry = u16::from(c3 >> 7);
            c3 = c3.rotate_left(1);

            let a = group[0] ^ c3;
            let sum = u16::from(c1) + u16::from(a) + carry;
            c1 = sum as u8;
            carry = sum >> 8;
            out.push(a);

            if group.len() > 1 {
                let b = group[1] ^ c1;
                let sum = u16::from(c2) + u16::from(b) + carry;
                c2 = sum as u8;
                carry = sum >> 8;
                out.push(b);
            }
            if group.len() > 2 {
                let c = group[2] ^ c2;
                let sum = u16::from(c3) + u16::from(c) + carry;
                c3 = sum as u8;
                out.push(c);
            }
        }
        (out, Checksum { sums: [c1, c2, c3] })
    }

    /// The four six-bit values the checksum is written as: the top two bits of
    /// each sum together, then the low six of each.
    #[must_use]
    pub fn nibbles(&self) -> [u8; 4] {
        let [c1, c2, c3] = self.sums;
        [
            ((c1 >> 6) << 4) | ((c2 >> 6) << 2) | (c3 >> 6),
            c1 & 0x3f,
            c2 & 0x3f,
            c3 & 0x3f,
        ]
    }

    /// The same undone.
    #[must_use]
    pub fn from_nibbles(n: [u8; 4]) -> Checksum {
        Checksum {
            sums: [
                (((n[0] >> 4) & 3) << 6) | n[1],
                (((n[0] >> 2) & 3) << 6) | n[2],
                ((n[0] & 3) << 6) | n[3],
            ],
        }
    }
}

// ---------------------------------------------------------------------------
// nibblization
// ---------------------------------------------------------------------------

/// Turn bytes into six-bit values, three to four.
///
/// The Guide: "each group of three data bytes is formatted as four 8-bit
/// patterns". The four are the top two bits of each of the three, packed into
/// one value, and then the low six of each — so nothing is lost and no value
/// is wider than six bits. A trailing group of one or two bytes yields two or
/// three values, which is how 524 bytes become
/// [`SECTOR_NIBBLES`] rather than 700.
#[must_use]
pub fn nibblize(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len().div_ceil(3) * 4);
    for group in bytes.chunks(3) {
        let a = group[0];
        let b = group.get(1).copied().unwrap_or(0);
        let c = group.get(2).copied().unwrap_or(0);
        out.push(((a >> 6) << 4) | ((b >> 6) << 2) | (c >> 6));
        out.push(a & 0x3f);
        if group.len() > 1 {
            out.push(b & 0x3f);
        }
        if group.len() > 2 {
            out.push(c & 0x3f);
        }
    }
    out
}

/// The same undone. A run that is not a whole number of groups stops where it
/// runs out, the way [`nibblize`] left it.
#[must_use]
pub fn denibblize(nibbles: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(nibbles.len() / 4 * 3 + 2);
    for group in nibbles.chunks(4) {
        let top = group[0];
        for (i, &low) in group[1..].iter().enumerate() {
            let high = (top >> (4 - 2 * i)) & 3;
            out.push((high << 6) | (low & 0x3f));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// a track
// ---------------------------------------------------------------------------

/// One cylinder and side as a stream of bits, which is what a head delivers.
///
/// Bits rather than bytes because the stream has no byte boundaries in it: the
/// shifter finds them by waiting for a one to reach the top of its register,
/// and a self-sync byte — eight ones and two zeros, ten bits where a payload
/// byte is eight — is what lets it. Modelling the track as bytes would hand
/// the shifter a framing it has to earn.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Track {
    bits: Vec<u8>,
    len: usize,
}

impl Track {
    /// An empty track.
    #[must_use]
    pub fn new() -> Track {
        Track::default()
    }

    /// How many bit cells go past the head in one revolution.
    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether there is nothing on it — an unformatted cylinder.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Bit `n` of the revolution, which wraps: a disk goes round.
    #[must_use]
    pub fn bit(&self, n: usize) -> bool {
        if self.len == 0 {
            return false;
        }
        let n = n % self.len;
        self.bits[n / 8] & (0x80 >> (n % 8)) != 0
    }

    /// Append one bit.
    pub fn push_bit(&mut self, bit: bool) {
        if self.len.is_multiple_of(8) {
            self.bits.push(0);
        }
        if bit {
            let n = self.len;
            self.bits[n / 8] |= 0x80 >> (n % 8);
        }
        self.len += 1;
    }

    /// Append the eight bits of `byte`, most significant first.
    pub fn push_byte(&mut self, byte: u8) {
        for i in (0..8).rev() {
            self.push_bit(byte & (1 << i) != 0);
        }
    }

    /// Append `n` self-sync bytes.
    ///
    /// A self-sync byte is `$FF` in **ten** bit cells rather than eight: eight
    /// ones and two zeros. It is the whole trick of the format — a shifter that
    /// latches when a one reaches the top of its register comes out of a run of
    /// these aligned to them however it went in, because every ten bits it has
    /// thrown away two.
    pub fn push_sync(&mut self, n: usize) {
        for _ in 0..n {
            self.push_byte(0xff);
            self.push_bit(false);
            self.push_bit(false);
        }
    }

    /// Append one six-bit value as its disk byte.
    pub fn push_nibble(&mut self, value: u8) {
        self.push_byte(DISK_BYTES[usize::from(value & 0x3f)]);
    }

    /// Lay one whole sector down: the gap, the address field, the gap and the
    /// data field.
    pub fn push_sector(&mut self, sector: &Sector) {
        self.push_sync(SYNC_BEFORE_ADDRESS);
        for byte in ADDRESS_MARK {
            self.push_byte(byte);
        }
        let side_high = (u8::from(sector.side) << 5) | (sector.track >> 6);
        let track_low = sector.track & 0x3f;
        self.push_nibble(track_low);
        self.push_nibble(sector.sector);
        self.push_nibble(side_high);
        self.push_nibble(sector.format);
        self.push_nibble(track_low ^ sector.sector ^ side_high ^ sector.format);
        for byte in EPILOGUE {
            self.push_byte(byte);
        }
        self.push_byte(0xff);

        self.push_sync(SYNC_BEFORE_DATA);
        for byte in DATA_MARK {
            self.push_byte(byte);
        }
        self.push_nibble(sector.sector);
        let (scrambled, sum) = Checksum::scramble(&sector.bytes);
        for value in nibblize(&scrambled) {
            self.push_nibble(value);
        }
        for value in sum.nibbles() {
            self.push_nibble(value);
        }
        for byte in EPILOGUE {
            self.push_byte(byte);
        }
        self.push_byte(0xff);
    }
}

/// One sector, as it goes onto a track or comes off it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sector {
    /// The cylinder, 0 to [`MAX_TRACK`].
    pub track: u8,
    /// Which head, `false` for the first.
    pub side: bool,
    /// Which sector of the cylinder.
    pub sector: u8,
    /// The format byte: [`FORMAT_800K`] or [`FORMAT_400K`].
    pub format: u8,
    /// Its 524 bytes: twelve of tag and then 512 of data.
    pub bytes: Vec<u8>,
}

impl Sector {
    /// A sector holding `data` with `tag` in front of it, padded or trimmed to
    /// the twelve and 512 the format has room for.
    #[must_use]
    pub fn new(track: u8, side: bool, sector: u8, format: u8, tag: &[u8], data: &[u8]) -> Sector {
        let mut bytes = vec![0u8; SECTOR_BYTES];
        let n = tag.len().min(TAG_BYTES);
        bytes[..n].copy_from_slice(&tag[..n]);
        let n = data.len().min(DATA_BYTES);
        bytes[TAG_BYTES..TAG_BYTES + n].copy_from_slice(&data[..n]);
        Sector {
            track,
            side,
            sector,
            format,
            bytes,
        }
    }

    /// The twelve tag bytes.
    #[must_use]
    pub fn tag(&self) -> &[u8] {
        &self.bytes[..TAG_BYTES]
    }

    /// The 512 bytes a file system sees.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.bytes[TAG_BYTES..]
    }
}

/// Build the whole of one cylinder and side.
///
/// The sectors are laid down in the order given; a real formatter interleaves
/// them, and nothing here or in the drive depends on which order they are in,
/// because a sector is found by reading its address field rather than by
/// counting.
#[must_use]
pub fn encode_track(sectors: &[Sector]) -> Track {
    let mut track = Track::new();
    for sector in sectors {
        track.push_sector(sector);
    }
    track
}

// ---------------------------------------------------------------------------
// reading it back
// ---------------------------------------------------------------------------

/// Why a sector on a track could not be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BadSector {
    /// A field's five address bytes did not check.
    AddressChecksum,
    /// A data field was not a whole one before the track came round again.
    Truncated,
    /// A byte on the medium was not one of the sixty-four.
    NotADiskByte,
    /// The 24-bit checksum on the data did not match.
    DataChecksum,
}

/// Shift a whole revolution past a head and give back every sector on it.
///
/// This is [`encode_track`]'s inverse and its test: what a real drive's head
/// and a real IWM would make of the bits, without either of them.
///
/// # Errors
///
/// Nothing: a field that does not read is left out and reported in the second
/// half of the pair, so one damaged sector does not hide the rest.
#[must_use]
pub fn decode_track(track: &Track) -> (Vec<Sector>, Vec<BadSector>) {
    let mut sectors = Vec::new();
    let mut bad = Vec::new();
    if track.is_empty() {
        return (sectors, bad);
    }
    // Two revolutions, so that a field straddling the index is seen whole, and
    // then anything found in the second is a repeat of the first.
    let bytes = shift_bytes(track, 2);
    let mut i = 0usize;
    let mut address: Option<(u8, bool, u8, u8)> = None;
    while i + 3 <= bytes.len() {
        let window = &bytes[i..];
        if window.starts_with(&ADDRESS_MARK) {
            let Some(fields) = window.get(3..8) else {
                break;
            };
            let Some(v) = fields
                .iter()
                .map(|&b| payload_of(b))
                .collect::<Option<Vec<u8>>>()
            else {
                bad.push(BadSector::NotADiskByte);
                i += 3;
                continue;
            };
            let (track_low, sector, side_high, format, sum) = (v[0], v[1], v[2], v[3], v[4]);
            if (track_low ^ sector ^ side_high ^ format) & 0x3f != sum {
                bad.push(BadSector::AddressChecksum);
                address = None;
            } else {
                let number = track_low | ((side_high & 1) << 6);
                address = Some((number, side_high & 0x20 != 0, sector, format));
            }
            i += 8;
            continue;
        }
        if window.starts_with(&DATA_MARK) {
            let Some((number, side, sector, format)) = address.take() else {
                i += 3;
                continue;
            };
            let end = 3 + 1 + SECTOR_NIBBLES + 4;
            let Some(field) = window.get(3..end) else {
                bad.push(BadSector::Truncated);
                break;
            };
            let Some(v) = field
                .iter()
                .map(|&b| payload_of(b))
                .collect::<Option<Vec<u8>>>()
            else {
                bad.push(BadSector::NotADiskByte);
                i += 3;
                continue;
            };
            let scrambled = denibblize(&v[1..1 + SECTOR_NIBBLES]);
            let (bytes, sum) = Checksum::unscramble(&scrambled[..SECTOR_BYTES]);
            let want = Checksum::from_nibbles([
                v[1 + SECTOR_NIBBLES],
                v[2 + SECTOR_NIBBLES],
                v[3 + SECTOR_NIBBLES],
                v[4 + SECTOR_NIBBLES],
            ]);
            if sum != want {
                bad.push(BadSector::DataChecksum);
            } else if !sectors.iter().any(|s: &Sector| s.sector == sector) {
                sectors.push(Sector {
                    track: number,
                    side,
                    sector,
                    format,
                    bytes,
                });
            }
            i += end;
            continue;
        }
        i += 1;
    }
    (sectors, bad)
}

/// Shift `revolutions` of `track` past a head and give back the bytes the
/// shifter latched.
///
/// The IWM's own rule, and the only one there is: the register shifts left and
/// a byte is complete when a one reaches bit 7. Leading zeros are therefore
/// skipped rather than counted, which is what makes a self-sync run
/// self-synchronising.
#[must_use]
pub fn shift_bytes(track: &Track, revolutions: usize) -> Vec<u8> {
    let mut out = Vec::new();
    let mut sr = 0u8;
    for n in 0..track.len() * revolutions {
        sr = (sr << 1) | u8::from(track.bit(n));
        if sr & 0x80 != 0 {
            out.push(sr);
            sr = 0;
        }
    }
    out
}

#[cfg(test)]
mod tests;
