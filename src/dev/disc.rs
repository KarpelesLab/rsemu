//! What a **CD** is, underneath whatever bus reaches it.
//!
//! [`medium`] answers "where do the bytes come from"; this
//! module answers "what are those bytes". They are different questions and a
//! CD is the one place in the tree where the second has a non-trivial answer:
//! a hard disk's sector *is* its bytes, and a CD's is 2352 bytes of frame with
//! 2048 bytes of user data buried in it at an offset that depends on the mode.
//!
//! # Why this is its own module and not part of `medium`
//!
//! `dev::medium` is the seam **every** block device stores bytes behind — an
//! NVMe namespace, a `virtio.blk`, an `ata.disk`, a CFI flash part. A
//! `riscv-virt` build has one and has never heard of a compact disc, and
//! putting ECMA-130's frame layout in that file would compile the Red Book
//! into it. So `dev-disc` is its own feature, it implies `dev-medium`, and the
//! two files divide as the two questions do.
//!
//! # Why this is not part of either drive
//!
//! Two devices in this tree hold a CD and they share no bus, no register file
//! and no command set:
//!
//! * [`ata::atapi`](crate::dev::ata::atapi) — a packet device on an ATA cable
//!   or a Serial ATA port, answering SFF-8020i command descriptor blocks;
//! * [`amiga::cdrom`](crate::dev::amiga::cdrom) — the CD32's mechanism, reached
//!   through Akiko's message ring and answering nothing resembling SCSI.
//!
//! Both need the same four things and neither may depend on the other, so the
//! four things live here: **sector layout**, **user-data extraction**, **MSF
//! conversion** and **table-of-contents synthesis**. Nothing in this file names
//! a register, an opcode, an interrupt or a bus.
//!
//! # Two file layouts, told apart by looking
//!
//! A CD's physical unit is a 2352-byte **frame**: 12 bytes of sync, 4 of
//! header, then 2336 of the mode's own arrangement (ECMA-130, *Data interchange
//! on read-only 120 mm optical data disks (CD-ROM)*, §14). What a file system
//! wants out of it is the 2048 bytes of user data a Mode 1 or Mode 2 Form 1
//! sector carries. Two file layouts follow and [`Layout`] is which:
//!
//! * [`Layout::UserData`] — 2048 bytes a sector, the user data and nothing
//!   else. This is what an ISO 9660 image is: ECMA-119 describes the *file
//!   system* in 2048-byte logical sectors and is silent about frames.
//! * [`Layout::Frames`] — 2352 bytes a sector, whole frames, the user data
//!   lifted out of each: bytes 16..2064 for Mode 1 and 24..2072 for Mode 2
//!   Form 1, whose eight-byte sub-header sits between the header and the data
//!   (ECMA-130 §§14.2-14.3).
//!
//! **Which one a file is, is decided by the sync pattern rather than by the
//! extension or by arithmetic.** A frame begins with `00`, ten `FF` and `00`
//! (§14.1), and a file whose first twelve bytes are that is frames. A size
//! cannot settle it: 2048 and 2352 share a factor of 16, so an image of
//! 301 056 bytes divides evenly by both, and the sync pattern is definitive
//! where a modulus is a guess.
//!
//! # What is *not* modelled
//!
//! No audio track, no `.cue` sheet, no subchannel and so no CD+G, no
//! multi-session, and no Mode 2 Form 2 (2324-byte) data. A disc here is **one
//! data track**, which is what both callers actually have: a CD32 master and a
//! bootable ISO are each one data track in one session. The other formats are
//! not done rather than half done, which is why [`Toc`] is a synthesis with a
//! constructor rather than a parser with a fallback.
//!
//! # Sources
//!
//! * **ECMA-130**, *Data interchange on read-only 120 mm optical data disks
//!   (CD-ROM)*, 2nd edition, June 1996 — §14 the sector layouts and the sync
//!   pattern, §20 the lead-in, §22.3.1 the track control field.
//! * **ECMA-119**, *Volume and file structure of CD-ROM for information
//!   interchange* (ISO 9660) — §6.1.2, the 2048-byte logical sector.
//!
//! **No emulator source of any licence was consulted** (`CLAUDE.md`,
//! provenance).

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::dev::medium::{self, Medium};

// ---------------------------------------------------------------------------
// the numbers
// ---------------------------------------------------------------------------

/// A whole frame, sync and parity and all (ECMA-130 §14).
pub const FRAME_BYTES: u64 = 2352;

/// The user data a Mode 1 or Mode 2 Form 1 sector carries (ECMA-130 §14.2,
/// §14.3; ECMA-119 §6.1.2's logical sector).
pub const USER_BYTES: u64 = 2048;

/// Where the user data starts in a Mode 1 frame: past sync and header.
const MODE1_AT: usize = 16;

/// Where it starts in a Mode 2 Form 1 frame: past the eight-byte sub-header as
/// well.
const MODE2_AT: usize = 24;

/// The byte of a frame's header that says which mode it is (ECMA-130 §14.2).
const MODE_BYTE: usize = 15;

/// The mode byte of a Mode 2 sector.
const MODE_2: u8 = 2;

/// A frame's twelve-byte sync pattern (ECMA-130 §14.1).
pub const SYNC: [u8; 12] = [
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
];

/// The lead-in every disc begins with, in frames: two seconds (ECMA-130 §20).
///
/// Why [`msf`] of block zero is 00:02:00 and not 00:00:00, and the single
/// number that a model gets wrong more often than anything else about a CD.
pub const LEAD_IN_FRAMES: u64 = 150;

/// Frames a second at single speed (ECMA-130 §13): 75.
pub const FRAMES_PER_SECOND: u64 = 75;

/// Seconds a minute, which an MSF address counts in.
pub const SECONDS_PER_MINUTE: u64 = 60;

/// The `ADR` a table-of-contents entry carries when its address is a position
/// (ECMA-130 §22.3.1): 1.
pub const ADR_POSITION: u8 = 1;

/// The control nibble of a data track: `$4`, "data track, digital copy
/// prohibited" (ECMA-130 §22.3.1).
pub const CONTROL_DATA_TRACK: u8 = 0x4;

/// The track number a table of contents reserves for the lead-out
/// (SFF-8020i's `READ TOC`, and ECMA-130 §22.3.3's lead-out area).
pub const LEAD_OUT_TRACK: u8 = 0xaa;

// ---------------------------------------------------------------------------
// layout
// ---------------------------------------------------------------------------

/// How the sectors of an image are laid out in the file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layout {
    /// 2048 bytes a sector: user data only, which is what an ISO image is.
    UserData,
    /// 2352 bytes a sector: whole frames, user data lifted out of each.
    Frames,
}

impl Layout {
    /// How many bytes one sector takes in the file.
    #[must_use]
    pub const fn stride(self) -> u64 {
        match self {
            Layout::UserData => USER_BYTES,
            Layout::Frames => FRAME_BYTES,
        }
    }

    /// Which layout `head` — the first bytes of the image — is in.
    ///
    /// The sync pattern decides it; see the module documentation for why a size
    /// cannot.
    #[must_use]
    pub fn of(head: &[u8]) -> Layout {
        if head.len() >= SYNC.len() && head[..SYNC.len()] == SYNC {
            Layout::Frames
        } else {
            Layout::UserData
        }
    }

    /// The name a diagnostic gives it.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Layout::UserData => "user data",
            Layout::Frames => "raw frames",
        }
    }
}

// ---------------------------------------------------------------------------
// addresses
// ---------------------------------------------------------------------------

/// A logical block as the minute, second and frame a drive reports.
///
/// Block zero is two seconds in, because the lead-in is 150 frames long
/// (ECMA-130 §20). This is the arithmetic and not a command set: `READ TOC` in
/// MSF form and the CD32's own position report are the same sum, which is the
/// whole reason it is here rather than in either of them.
#[must_use]
pub fn msf(block: u64) -> (u8, u8, u8) {
    let f = block + LEAD_IN_FRAMES;
    (
        (f / (FRAMES_PER_SECOND * SECONDS_PER_MINUTE)) as u8,
        (f / FRAMES_PER_SECOND % SECONDS_PER_MINUTE) as u8,
        (f % FRAMES_PER_SECOND) as u8,
    )
}

// ---------------------------------------------------------------------------
// the table of contents
// ---------------------------------------------------------------------------

/// A disc's table of contents, as this module synthesises it.
///
/// Synthesised, because a bare image carries none: one track, number 1, a data
/// track, starting at logical block 0, with the lead-out at the block after the
/// last.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Toc {
    /// The first track number; always 1 here.
    pub first: u8,
    /// The last track number; always 1 here.
    pub last: u8,
    /// The track's control nibble: [`CONTROL_DATA_TRACK`].
    pub control: u8,
    /// Where the track starts, in logical blocks.
    pub start: u64,
    /// Where the lead-out starts: the block after the last.
    pub lead_out: u64,
}

impl Toc {
    /// The table of contents of a disc holding one data track of `sectors`
    /// logical blocks.
    #[must_use]
    pub const fn one_data_track(sectors: u64) -> Toc {
        Toc {
            first: 1,
            last: 1,
            control: CONTROL_DATA_TRACK,
            start: 0,
            lead_out: sectors,
        }
    }

    /// The `ADR`/control byte a descriptor carries: `ADR` in 7:4, control in
    /// 3:0 (ECMA-130 §22.3.1).
    #[must_use]
    pub const fn adr_control(&self) -> u8 {
        (ADR_POSITION << 4) | (self.control & 0x0f)
    }
}

// ---------------------------------------------------------------------------
// the disc
// ---------------------------------------------------------------------------

/// A disc: a [`Medium`], the layout its sectors are in, and how many there are.
///
/// Cheap to clone-by-reference — the medium is behind an `Arc` — and immutable:
/// a disc does not move, a *head* does, and where the head is belongs to
/// whichever drive is holding this.
#[derive(Debug)]
pub struct Disc {
    bytes: Arc<dyn Medium>,
    layout: Layout,
    sectors: u64,
}

impl Disc {
    /// The disc `bytes` holds, in whichever layout it is in.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming `at` if the image is empty or is not a whole
    /// number of sectors in the layout the sync pattern says it is in.
    pub fn open(at: &str, bytes: Arc<dyn Medium>) -> Result<Disc> {
        let (layout, capacity) = probe(&bytes)?;
        let stride = layout.stride();
        if capacity == 0 || !capacity.is_multiple_of(stride) {
            return Err(config(
                at,
                format!(
                    "a disc image of {capacity} bytes is not a whole number of {stride}-byte \
                     sectors"
                ),
            ));
        }
        Ok(Disc {
            bytes,
            layout,
            sectors: capacity / stride,
        })
    }

    /// The disc `bytes` holds, which must be 2048-byte user data.
    ///
    /// For a drive that models [`USER_BYTES`]-byte logical blocks and nothing
    /// else — no sync pattern, no header, no error-correction codes and no
    /// audio track. **A raw image is refused by name rather than guessed at**,
    /// because the failure mode of reading one as though it were cooked is
    /// sixteen bytes of sync pattern where the boot record should be and no
    /// error anywhere.
    ///
    /// Two things give a raw image away and both are checked, because neither
    /// alone is enough: the sync pattern, which is definitive but which a file
    /// truncated before its first frame does not have, and a length that is a
    /// whole number of 2352-byte sectors and not of 2048-byte ones.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming `at`.
    pub fn open_user_data(at: &str, bytes: Arc<dyn Medium>) -> Result<Disc> {
        let (layout, capacity) = probe(&bytes)?;
        if capacity == 0 {
            return Err(config(
                at,
                String::from(
                    "a disc of zero bytes is an empty drive; leave the media slot unbound",
                ),
            ));
        }
        if layout == Layout::Frames {
            return Err(config(at, raw_message(capacity)));
        }
        if !capacity.is_multiple_of(USER_BYTES) {
            if capacity.is_multiple_of(FRAME_BYTES) {
                return Err(config(at, raw_message(capacity)));
            }
            return Err(config(
                at,
                format!(
                    "a disc holds a whole number of {USER_BYTES}-byte logical blocks, and \
                     {capacity} bytes is not a whole number of them"
                ),
            ));
        }
        Ok(Disc {
            bytes,
            layout: Layout::UserData,
            sectors: capacity / USER_BYTES,
        })
    }

    /// How many logical blocks the disc holds.
    #[must_use]
    pub fn sectors(&self) -> u64 {
        self.sectors
    }

    /// How its sectors are laid out in the image.
    #[must_use]
    pub fn layout(&self) -> Layout {
        self.layout
    }

    /// The storage underneath, for a caller that has to snapshot or describe it.
    #[must_use]
    pub fn medium(&self) -> &Arc<dyn Medium> {
        &self.bytes
    }

    /// How many bytes of user data the whole disc holds.
    #[must_use]
    pub fn user_bytes(&self) -> u64 {
        self.sectors * USER_BYTES
    }

    /// The disc's table of contents.
    #[must_use]
    pub fn toc(&self) -> Toc {
        Toc::one_data_track(self.sectors)
    }

    /// The [`USER_BYTES`] bytes of user data logical block `lba` carries.
    ///
    /// # Errors
    ///
    /// [`Error::State`] past the last block, or when the image cannot be read.
    pub fn read_block(
        &self,
        at: &str,
        lba: u64,
        dst: &mut [u8; USER_BYTES as usize],
    ) -> Result<()> {
        if lba >= self.sectors {
            return Err(Error::State(format!(
                "{at}: block {lba} is past the disc's {} blocks",
                self.sectors
            )));
        }
        self.read_user_at(lba * USER_BYTES, dst)
    }

    /// Fill `dst` from `offset` bytes into the disc's **user data**, whatever
    /// the frame layout underneath.
    ///
    /// The address a caller does arithmetic in is always the user-data one —
    /// `lba * 2048 + n` — because that is the address ISO 9660 and every
    /// command set use. Turning it into a file offset is this method's whole
    /// job, and it is why a drive that streams a multi-block transfer never
    /// learns what a frame is.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs past the disc or the medium refuses
    /// it.
    pub fn read_user_at(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        if dst.is_empty() {
            return Ok(());
        }
        let end = offset.saturating_add(dst.len() as u64);
        if end > self.user_bytes() {
            return Err(Error::State(format!(
                "a read of {} byte(s) at {offset} runs past the disc's {} bytes of user data",
                dst.len(),
                self.user_bytes()
            )));
        }
        match self.layout {
            Layout::UserData => self
                .bytes
                .read_at(offset, dst)
                .map_err(|e| medium::error_at(offset, e)),
            Layout::Frames => {
                // One frame at a time, starting part way into the first and
                // ending part way through the last: the caller's offset is a
                // user-data one and a frame's user data is not where the frame
                // is.
                let mut done: u64 = 0;
                let mut frame = [0u8; FRAME_BYTES as usize];
                while done < dst.len() as u64 {
                    let at = offset + done;
                    let lba = at / USER_BYTES;
                    let within = (at % USER_BYTES) as usize;
                    let file_at = lba * FRAME_BYTES;
                    self.bytes
                        .read_at(file_at, &mut frame)
                        .map_err(|e| medium::error_at(file_at, e))?;
                    let from = user_data_at(&frame);
                    let take =
                        core::cmp::min(USER_BYTES as usize - within, dst.len() - done as usize);
                    dst[done as usize..done as usize + take]
                        .copy_from_slice(&frame[from + within..from + within + take]);
                    done += take as u64;
                }
                Ok(())
            }
        }
    }
}

/// Where the user data starts in `frame`.
///
/// Mode 2's sub-header pushes it eight bytes along (ECMA-130 §14.3); any other
/// mode byte is read as Mode 1, which is what a drive handed a frame it cannot
/// classify does with it.
fn user_data_at(frame: &[u8; FRAME_BYTES as usize]) -> usize {
    if frame[MODE_BYTE] == MODE_2 {
        MODE2_AT
    } else {
        MODE1_AT
    }
}

/// The layout and capacity of `bytes`, by looking at its first twelve.
fn probe(bytes: &Arc<dyn Medium>) -> Result<(Layout, u64)> {
    let capacity = bytes.capacity();
    let mut head = [0u8; SYNC.len()];
    if capacity >= SYNC.len() as u64 {
        bytes
            .read_at(0, &mut head)
            .map_err(|e| medium::error_at(0, e))?;
    }
    Ok((Layout::of(&head), capacity))
}

/// The diagnostic a raw image gets, which names the format rather than the
/// arithmetic.
fn raw_message(capacity: u64) -> String {
    format!(
        "{capacity} bytes is {} raw {FRAME_BYTES}-byte CD sectors; this drive reads \
         {USER_BYTES}-byte logical blocks and does not model the sync pattern, the header or \
         the error-correction codes a raw image carries. Convert it to a \
         {USER_BYTES}-byte-per-sector image first",
        capacity.div_ceil(FRAME_BYTES)
    )
}

fn config(at: &str, message: String) -> Error {
    Error::Config {
        at: String::from(at),
        message,
    }
}

// ---------------------------------------------------------------------------
// fixtures
// ---------------------------------------------------------------------------

/// A disc image of `sectors` 2352-byte Mode 1 frames, each carrying a pattern
/// derived from its own index, for a test that needs whole frames.
#[must_use]
#[doc(hidden)]
pub fn mode1_image(sectors: u64) -> Vec<u8> {
    let mut out = vec![0u8; (sectors * FRAME_BYTES) as usize];
    for lba in 0..sectors {
        let at = (lba * FRAME_BYTES) as usize;
        out[at..at + SYNC.len()].copy_from_slice(&SYNC);
        let (m, s, f) = msf(lba);
        out[at + 12] = m;
        out[at + 13] = s;
        out[at + 14] = f;
        out[at + MODE_BYTE] = 1;
        for i in 0..USER_BYTES as usize {
            out[at + MODE1_AT + i] = (lba as u8).wrapping_add(i as u8);
        }
    }
    out
}

/// The same content as [`mode1_image`], as an ISO image: user data only.
#[must_use]
#[doc(hidden)]
pub fn iso_image(sectors: u64) -> Vec<u8> {
    let mut out = vec![0u8; (sectors * USER_BYTES) as usize];
    for lba in 0..sectors {
        let at = (lba * USER_BYTES) as usize;
        for i in 0..USER_BYTES as usize {
            out[at + i] = (lba as u8).wrapping_add(i as u8);
        }
    }
    out
}

/// A flat buffer holding `bytes`, as a medium.
#[cfg(test)]
fn store(bytes: &[u8]) -> Arc<dyn Medium> {
    let ram = crate::core::space::RamStore::new(bytes.len() as u64);
    // A fresh store nobody else has a handle on; the write cannot fail.
    let _ = ram.write_at(0, bytes);
    Arc::new(ram) as Arc<dyn Medium>
}

#[cfg(test)]
mod tests {
    use super::*;

    use alloc::string::ToString;

    const AT: &str = "test.disc";

    #[test]
    fn a_sync_pattern_says_frames_and_anything_else_says_user_data() {
        assert_eq!(Layout::of(&SYNC), Layout::Frames);
        assert_eq!(Layout::of(&[0u8; 12]), Layout::UserData);
        // Short of twelve bytes there is no pattern to match, whatever the
        // bytes are.
        assert_eq!(Layout::of(&SYNC[..11]), Layout::UserData);
        assert_eq!(Layout::UserData.stride(), 2048);
        assert_eq!(Layout::Frames.stride(), 2352);
    }

    /// 2048 and 2352 share a factor of 16, so 128 frames is also 147 logical
    /// sectors: the case a modulus cannot decide and the sync pattern can.
    #[test]
    fn a_size_alone_cannot_tell_the_two_layouts_apart() {
        assert_eq!(FRAME_BYTES * 128, USER_BYTES * 147);
        let raw = store(&mode1_image(128));
        let disc = Disc::open(AT, raw).expect("frames open");
        assert_eq!(disc.layout(), Layout::Frames);
        assert_eq!(disc.sectors(), 128);
    }

    /// And a drive that only reads 2048-byte blocks must still refuse it, which
    /// the modulus would have let through.
    #[test]
    fn a_raw_image_whose_length_divides_by_2048_is_still_refused() {
        let raw = store(&mode1_image(128));
        let text = Disc::open_user_data(AT, raw)
            .expect_err("a raw image is not a cooked one")
            .to_string();
        assert!(text.contains("2352"), "{text}");
    }

    #[test]
    fn a_raw_image_is_refused_by_name_on_its_length_alone() {
        // No sync pattern — a rip whose first frame was lost — so only the
        // length is left to say what it is.
        let raw = store(&vec![0u8; (FRAME_BYTES * 10) as usize]);
        let text = Disc::open_user_data(AT, raw)
            .expect_err("a raw image is not a cooked one")
            .to_string();
        assert!(text.contains("2352"), "{text}");
        assert!(text.contains("2048"), "{text}");
    }

    #[test]
    fn a_length_that_is_neither_is_refused_as_arithmetic() {
        let odd = store(&vec![0u8; 2047]);
        let text = Disc::open_user_data(AT, odd)
            .expect_err("not a whole number of blocks")
            .to_string();
        assert!(text.contains("whole number"), "{text}");
        assert!(!text.contains("2352"), "{text}");
    }

    #[test]
    fn an_empty_image_is_refused_by_both_doors() {
        assert!(Disc::open(AT, store(&[])).is_err());
        assert!(Disc::open_user_data(AT, store(&[])).is_err());
    }

    /// The two layouts of the same disc hand a caller the same bytes, which is
    /// the whole point of lifting the user data out of a frame.
    #[test]
    fn the_same_disc_reads_the_same_either_way() {
        let iso = Disc::open(AT, store(&iso_image(6))).expect("an ISO opens");
        let raw = Disc::open(AT, store(&mode1_image(6))).expect("frames open");
        assert_eq!(iso.layout(), Layout::UserData);
        assert_eq!(raw.layout(), Layout::Frames);
        for lba in 0..6 {
            let mut a = [0u8; USER_BYTES as usize];
            let mut b = [0u8; USER_BYTES as usize];
            iso.read_block(AT, lba, &mut a).expect("the ISO reads");
            raw.read_block(AT, lba, &mut b).expect("the frames read");
            assert_eq!(a, b, "block {lba}");
            // And it is what the fixture put there, checked against the rule
            // rather than against the other image.
            for (i, byte) in a.iter().enumerate() {
                assert_eq!(*byte, (lba as u8).wrapping_add(i as u8));
            }
        }
    }

    /// Mode 2 Form 1's sub-header pushes the user data eight bytes along.
    #[test]
    fn mode_2_form_1_user_data_starts_eight_bytes_later() {
        let mut image = mode1_image(1);
        image[MODE_BYTE] = MODE_2;
        image[MODE2_AT] = 0x5a;
        let disc = Disc::open(AT, store(&image)).expect("frames open");
        let mut buf = [0u8; USER_BYTES as usize];
        disc.read_block(AT, 0, &mut buf).expect("it reads");
        assert_eq!(buf[0], 0x5a);
    }

    /// A streamed read is addressed in user-data bytes and may start and end
    /// inside a sector, which is what a packet command's byte-count limit does
    /// to a multi-block transfer.
    #[test]
    fn a_streamed_read_crosses_sectors_in_either_layout() {
        for image in [iso_image(4), mode1_image(4)] {
            let disc = Disc::open(AT, store(&image)).expect("it opens");
            let mut got = vec![0u8; 3000];
            disc.read_user_at(USER_BYTES - 100, &mut got)
                .expect("it reads");
            for (i, byte) in got.iter().enumerate() {
                let at = USER_BYTES - 100 + i as u64;
                let lba = at / USER_BYTES;
                let within = at % USER_BYTES;
                assert_eq!(
                    *byte,
                    (lba as u8).wrapping_add(within as u8),
                    "{:?} byte {i}",
                    disc.layout()
                );
            }
        }
    }

    #[test]
    fn a_read_past_the_disc_is_refused_rather_than_short() {
        let disc = Disc::open(AT, store(&iso_image(3))).expect("an ISO opens");
        let mut buf = [0u8; USER_BYTES as usize];
        assert!(disc.read_block(AT, 2, &mut buf).is_ok());
        assert!(disc.read_block(AT, 3, &mut buf).is_err());
        let mut one = [0u8; 1];
        assert!(disc.read_user_at(disc.user_bytes() - 1, &mut one).is_ok());
        assert!(disc.read_user_at(disc.user_bytes(), &mut one).is_err());
        // An empty read is not an access and cannot run off anything.
        assert!(disc.read_user_at(u64::MAX, &mut []).is_ok());
    }

    /// The lead-in is two seconds, so block zero is 00:02:00 and block 75 is
    /// 00:03:00 (ECMA-130 §20).
    #[test]
    fn a_block_is_reported_two_seconds_in() {
        assert_eq!(msf(0), (0, 2, 0));
        assert_eq!(msf(74), (0, 2, 74));
        assert_eq!(msf(75), (0, 3, 0));
        assert_eq!(msf(75 * 60 - 151), (0, 59, 74));
        assert_eq!(msf(75 * 60 - 150), (1, 0, 0));
    }

    #[test]
    fn the_table_of_contents_is_one_data_track() {
        let disc = Disc::open(AT, store(&iso_image(10))).expect("an ISO opens");
        assert_eq!(
            disc.toc(),
            Toc {
                first: 1,
                last: 1,
                control: CONTROL_DATA_TRACK,
                start: 0,
                lead_out: 10,
            }
        );
        // ADR 1, control 4: the byte both command sets put in a descriptor.
        assert_eq!(disc.toc().adr_control(), 0x14);
    }
}
