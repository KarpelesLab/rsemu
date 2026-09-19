//! The CD32's CD-ROM drive: the disc, the frame layout its sectors are in,
//! and a table of contents.
//!
//! One class, `amiga.cd`. It is the *mechanism* — a disc, its sectors and
//! where a track starts — and not a controller: [`super::akiko`] is the
//! CD32's controller and is what a guest talks to. The split is `ata.disk`'s
//! and `pc.ide`'s and it holds for the same reason: this file contains no
//! register offset and no interrupt, and Akiko contains no sector layout.
//!
//! The Developer Notes (*Amiga CD32 Developer Notes*, Revision 3,
//! Commodore-Amiga Inc.) describe the drive in one line each — "Top loading
//! double speed CD-ROM drive", 300 KB/s, and "Create an ISO-9660 image file
//! suitable for making the gold (master) disc" — and say nothing about its
//! interface. So the *disc* is modelled here, from the disc standards, and
//! the interface is where [`super::akiko`] stops.
//!
//! # What a disc image is, and which ones this reads
//!
//! A CD's physical unit is a 2352-byte **frame** (ECMA-130, *Data interchange
//! on read-only 120 mm optical data disks (CD-ROM)*, §14: 12 bytes of sync,
//! 4 of header, then 2336 of the mode's own arrangement). What a file system
//! actually wants out of it is the 2048 bytes of user data a Mode 1 or Mode 2
//! Form 1 sector carries. Two file layouts follow, and this class reads both:
//!
//! * **2048 bytes a sector** — the user data and nothing else, which is what
//!   an ISO 9660 image is (ECMA-119 describes the *file system* in those
//!   2048-byte logical sectors and is silent about frames, which is exactly
//!   why the layout exists). This is what a CD32 master is delivered as and
//!   what the Developer Notes tell an author to produce.
//! * **2352 bytes a sector** — whole frames. The user data is lifted out:
//!   bytes 16–2063 for Mode 1, and 24–2071 for Mode 2 Form 1, whose eight-byte
//!   sub-header sits between the header and the data (ECMA-130 §§14.2–14.3).
//!
//! **Which one a file is, is decided by looking rather than by the extension
//! or by arithmetic.** A frame begins with the sync pattern `00` then ten
//! `FF` then `00` (§14.1), and a file whose first twelve bytes are that is
//! frames; anything else is user data. Sizes cannot settle it on their own —
//! 2048 and 2352 share a factor of 16, so an image of 301 056 bytes divides
//! evenly by both — and the sync pattern is definitive where a modulus is a
//! guess.
//!
//! Nothing else is read. **No audio track**, no `.cue` sheet, no subchannel
//! and so no CD+G, no multi-session and no Mode 2 Form 2 (2324-byte) data:
//! a disc here is one data track. The Developer Notes list "ISO-9660 CD-ROM,
//! Audio CD, CD+G" as the drive's formats and the other two are simply not
//! done, rather than half done.
//!
//! # The table of contents
//!
//! Synthesised, because a bare image carries none: **one track**, number 1,
//! a data track (control `$4`, "data track, digital copy prohibited" —
//! ECMA-130 §22.3.1), starting at logical block 0, with the lead-out at the
//! block after the last. [`Toc`] is that, and [`msf`] converts a logical
//! block to the minute/second/frame a drive reports, which is the block plus
//! the 150-frame (two-second) lead-in every disc begins with (§20).
//!
//! # Speed
//!
//! "Double speed", 150 frames a second doubled, is recorded in
//! [`FRAMES_PER_SECOND`] and used by nothing: no command reaches this drive
//! (see [`super::akiko`]), so there is no transfer whose duration it could
//! set. It is here because a rate that is a fact about the machine belongs
//! written down next to the machine, and because the first command that ever
//! arrives will want it.
//!
//! # `MemAttrs::debug`
//!
//! This class has no register window at all, so there is nothing for a debug
//! access to be careful of. Its only guest-visible surface is through
//! [`super::akiko`], which has its own rule.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::medium::{self, Medium};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.cd";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// A whole frame, sync and parity and all (ECMA-130 §14).
pub const FRAME_BYTES: u64 = 2352;

/// The user data a Mode 1 or Mode 2 Form 1 sector carries.
pub const USER_BYTES: u64 = 2048;

/// Where the user data starts in a Mode 1 frame: past sync and header.
const MODE1_AT: usize = 16;

/// Where it starts in a Mode 2 Form 1 frame: past the eight-byte sub-header
/// as well.
const MODE2_AT: usize = 24;

/// The byte of a frame's header that says which mode it is (§14.2).
const MODE_BYTE: usize = 15;

/// A frame's twelve-byte sync pattern (§14.1).
pub const SYNC: [u8; 12] = [
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
];

/// The lead-in every disc begins with, in frames: two seconds (§20).
pub const LEAD_IN_FRAMES: u64 = 150;

/// Frames a second at single speed; the CD32's drive is twice this.
pub const FRAMES_PER_SECOND: u64 = 75;

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
    pub fn stride(self) -> u64 {
        match self {
            Layout::UserData => USER_BYTES,
            Layout::Frames => FRAME_BYTES,
        }
    }

    /// Which layout `head` — the first bytes of the image — is in.
    ///
    /// The sync pattern decides it; see the module documentation for why a
    /// size cannot.
    #[must_use]
    pub fn of(head: &[u8]) -> Layout {
        if head.len() >= SYNC.len() && head[..SYNC.len()] == SYNC {
            Layout::Frames
        } else {
            Layout::UserData
        }
    }
}

/// A disc's table of contents, as this class synthesises it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Toc {
    /// The first track number; always 1 here.
    pub first: u8,
    /// The last track number; always 1 here.
    pub last: u8,
    /// The track's control nibble: `$4`, a data track.
    pub control: u8,
    /// Where the track starts, in logical blocks.
    pub start: u64,
    /// Where the lead-out starts: the block after the last.
    pub lead_out: u64,
}

/// A logical block as the minute, second and frame a drive reports.
///
/// Block zero is two seconds in, because the lead-in is 150 frames long
/// (ECMA-130 §20).
#[must_use]
pub fn msf(block: u64) -> (u8, u8, u8) {
    let f = block + LEAD_IN_FRAMES;
    (
        (f / (FRAMES_PER_SECOND * 60)) as u8,
        (f / FRAMES_PER_SECOND % 60) as u8,
        (f % FRAMES_PER_SECOND) as u8,
    )
}

// ---------------------------------------------------------------------------
// the disc
// ---------------------------------------------------------------------------

/// A disc in the tray.
#[derive(Debug)]
struct Disc {
    bytes: Arc<dyn Medium>,
    layout: Layout,
    sectors: u64,
}

impl Disc {
    /// The user data of logical block `lba`.
    fn read(&self, lba: u64, dst: &mut [u8; USER_BYTES as usize]) -> Result<()> {
        if lba >= self.sectors {
            return Err(Error::State(format!(
                "amiga.cd: block {lba} is past the disc's {} blocks",
                self.sectors
            )));
        }
        let at = lba * self.layout.stride();
        match self.layout {
            Layout::UserData => self
                .bytes
                .read_at(at, dst)
                .map_err(|e| medium::error_at(at, e)),
            Layout::Frames => {
                let mut frame = [0u8; FRAME_BYTES as usize];
                self.bytes
                    .read_at(at, &mut frame)
                    .map_err(|e| medium::error_at(at, e))?;
                // Mode 2's sub-header pushes the user data eight bytes along;
                // any other mode byte is read as Mode 1, which is what a
                // drive handed a frame it cannot classify does with it.
                let from = if frame[MODE_BYTE] == 2 {
                    MODE2_AT
                } else {
                    MODE1_AT
                };
                dst.copy_from_slice(&frame[from..from + USER_BYTES as usize]);
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the seam Akiko holds
// ---------------------------------------------------------------------------

/// What a controller can ask the mechanism.
///
/// [`ExportId::CD_DRIVE`] carries it. Cloning it is cloning the handle, not
/// the disc.
#[derive(Debug, Clone)]
pub struct DrivePort {
    inner: Arc<Inner>,
}

impl DrivePort {
    /// Whether there is a disc in the tray.
    #[must_use]
    pub fn has_disc(&self) -> bool {
        self.inner.disc.is_some()
    }

    /// How many logical blocks that disc has, or zero with an empty tray.
    #[must_use]
    pub fn sectors(&self) -> u64 {
        self.inner.disc.as_ref().map_or(0, |d| d.sectors)
    }

    /// How its sectors are laid out in the image, or `None` with no disc.
    #[must_use]
    pub fn layout(&self) -> Option<Layout> {
        self.inner.disc.as_ref().map(|d| d.layout)
    }

    /// The disc's table of contents, or `None` with no disc.
    #[must_use]
    pub fn toc(&self) -> Option<Toc> {
        self.inner.disc.as_ref().map(|d| Toc {
            first: 1,
            last: 1,
            control: 0x4,
            start: 0,
            lead_out: d.sectors,
        })
    }

    /// Read logical block `lba`'s 2048 bytes of user data, and leave the head
    /// there.
    ///
    /// # Errors
    ///
    /// [`Error::State`] with no disc in the tray, past the last block, or
    /// when the image cannot be read.
    pub fn read(&self, lba: u64, dst: &mut [u8; USER_BYTES as usize]) -> Result<()> {
        let Some(disc) = self.inner.disc.as_ref() else {
            return Err(Error::State(String::from("amiga.cd: no disc in the tray")));
        };
        disc.read(lba, dst)?;
        *self.inner.at.lock() = lba;
        Ok(())
    }

    /// Where the head is: the block last read, zero out of reset.
    #[must_use]
    pub fn at(&self) -> u64 {
        *self.inner.at.lock()
    }
}

/// The drive's shared insides.
#[derive(Debug)]
struct Inner {
    disc: Option<Disc>,
    /// The block last read. A real mechanism's head is somewhere, and this is
    /// the only thing about the drive that moves.
    at: Mutex<u64>,
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The CD32's CD-ROM drive.
#[derive(Debug)]
pub struct CdRom {
    inner: Arc<Inner>,
}

impl CdRom {
    /// Validate `props` and load the tray.
    ///
    /// An `image` slot with no bytes — which is what a front end binds when
    /// nobody names a disc — is an **empty tray**, not an error. That is a
    /// CD32 at its boot screen.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property nothing here accepts, and
    /// [`Error::Config`] for an image whose length is not a whole number of
    /// sectors in the layout it is in.
    pub fn new(props: &Props) -> Result<CdRom> {
        let mut r = props.reader();
        let media = r.optional_media("image")?;
        r.finish()?;

        // A medium the host installed under the slot's name wins over the
        // bytes in the media table, exactly as `ata.disk` has it: a run that
        // named a file meant that file.
        let supplied = match (props.hosts(), media.as_ref()) {
            (Some(hosts), Some(m)) => medium::get(hosts, m.name())?.and_then(|slot| slot.take()),
            _ => None,
        };
        let medium: Option<Arc<dyn Medium>> = match (supplied, media.as_ref()) {
            (Some(m), _) => Some(m),
            (None, Some(m)) if !m.is_empty() => Some(store(m.bytes())),
            _ => None,
        };
        CdRom::holding(medium)
    }

    /// The drive with `medium` in the tray, or empty with `None`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image is not a whole number of sectors.
    pub fn holding(medium: Option<Arc<dyn Medium>>) -> Result<CdRom> {
        let disc = match medium {
            None => None,
            Some(bytes) => {
                let mut head = [0u8; SYNC.len()];
                let capacity = bytes.capacity();
                if capacity >= SYNC.len() as u64 {
                    bytes
                        .read_at(0, &mut head)
                        .map_err(|e| medium::error_at(0, e))?;
                }
                let layout = Layout::of(&head);
                let stride = layout.stride();
                if capacity == 0 || capacity % stride != 0 {
                    return Err(Error::Config {
                        at: String::from(CLASS_NAME),
                        message: format!(
                            "a disc image of {capacity} bytes is not a whole number of \
                             {stride}-byte sectors"
                        ),
                    });
                }
                Some(Disc {
                    bytes,
                    layout,
                    sectors: capacity / stride,
                })
            }
        };
        Ok(CdRom {
            inner: Arc::new(Inner {
                disc,
                at: Mutex::with_rank(LockRank::DEVICE, 0),
            }),
        })
    }

    /// The handle a controller holds.
    #[must_use]
    pub fn port(&self) -> DrivePort {
        DrivePort {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl Device for CdRom {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the controller holds the port, and a disc is not a
        // region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A drive recalibrates to the start of the disc; the disc stays in it.
        *self.inner.at.lock() = 0;
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::CD_DRIVE)
            .then(|| Export::Opaque(Arc::new(self.port()) as Arc<dyn core::any::Any + Send + Sync>))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(*self.inner.at.lock())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let at = r.read_u64()?;
        let sectors = self.inner.disc.as_ref().map_or(0, |d| d.sectors);
        if at != 0 && at >= sectors {
            return Err(Error::State(format!(
                "amiga.cd: a head at block {at} on a disc of {sectors} blocks"
            )));
        }
        *self.inner.at.lock() = at;
        Ok(())
    }
}

impl Instance for CdRom {}

/// The `amiga.cd` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the CD32's double-speed CD-ROM drive: a disc of 2048-byte user data or 2352-byte \
              frames, and a one-track table of contents",
    properties: &[PropertySpec {
        name: "image",
        kind: ValueKind::Media,
        required: false,
        summary: "the disc: an ISO 9660 image, or raw frames. No bytes is an empty tray",
    }],
    construct: |props| Ok(Box::new(CdRom::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(CdRom::new(props)?)))
}

/// What the validator should know about `amiga.cd`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME).prop(PropSchema::new("image", ValueKind::Media))
}

/// A flat buffer holding `bytes`, as a medium.
fn store(bytes: &[u8]) -> Arc<dyn Medium> {
    let ram = RamStore::new(bytes.len() as u64);
    // A fresh store nobody else has a handle on; the write cannot fail.
    let _ = ram.write_at(0, bytes);
    Arc::new(ram) as Arc<dyn Medium>
}

/// A disc image of `sectors` 2352-byte Mode 1 frames, each carrying `fill` of
/// its own index, for a test that needs whole frames rather than user data.
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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn disc(bytes: Vec<u8>) -> Arc<dyn Medium> {
        super::store(&bytes)
    }

    fn snapshot(d: &CdRom) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("cd0", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("cd0", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(d, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn an_empty_tray_is_not_an_error_and_has_no_disc() {
        let drive = CdRom::holding(None).expect("an empty tray realizes");
        let port = drive.port();
        assert!(!port.has_disc());
        assert_eq!(port.sectors(), 0);
        assert_eq!(port.layout(), None);
        assert_eq!(port.toc(), None);
        let mut buf = [0u8; USER_BYTES as usize];
        assert!(port.read(0, &mut buf).is_err());
    }

    #[test]
    fn an_iso_image_is_user_data_and_a_frame_image_is_frames() {
        let iso = CdRom::holding(Some(disc(iso_image(4)))).expect("an ISO realizes");
        assert_eq!(iso.port().layout(), Some(Layout::UserData));
        assert_eq!(iso.port().sectors(), 4);

        let raw = CdRom::holding(Some(disc(mode1_image(4)))).expect("frames realize");
        assert_eq!(raw.port().layout(), Some(Layout::Frames));
        assert_eq!(raw.port().sectors(), 4);
    }

    /// The two layouts of the same disc hand the guest the same bytes, which
    /// is the whole point of lifting the user data out of a frame.
    #[test]
    fn the_same_disc_reads_the_same_either_way() {
        let iso = CdRom::holding(Some(disc(iso_image(6)))).expect("an ISO realizes");
        let raw = CdRom::holding(Some(disc(mode1_image(6)))).expect("frames realize");
        for lba in 0..6 {
            let mut a = [0u8; USER_BYTES as usize];
            let mut b = [0u8; USER_BYTES as usize];
            iso.port().read(lba, &mut a).expect("the ISO reads");
            raw.port().read(lba, &mut b).expect("the frames read");
            assert_eq!(a, b, "block {lba}");
            // And it is the content the fixture put there, checked against
            // the rule rather than against the other image.
            for (i, byte) in a.iter().enumerate() {
                assert_eq!(*byte, (lba as u8).wrapping_add(i as u8));
            }
        }
    }

    /// Mode 2 Form 1's sub-header pushes the user data eight bytes along.
    #[test]
    fn mode_2_form_1_user_data_starts_eight_bytes_later() {
        let mut image = mode1_image(1);
        image[MODE_BYTE] = 2;
        // Put a recognisable byte where Mode 2's data begins.
        image[MODE2_AT] = 0x5A;
        let drive = CdRom::holding(Some(disc(image))).expect("frames realize");
        let mut buf = [0u8; USER_BYTES as usize];
        drive.port().read(0, &mut buf).expect("it reads");
        assert_eq!(buf[0], 0x5A);
    }

    #[test]
    fn a_disc_that_is_not_whole_sectors_is_refused() {
        assert!(CdRom::holding(Some(disc(vec![0u8; 2047]))).is_err());
        assert!(CdRom::holding(Some(disc(vec![0u8; 0]))).is_err());
        let mut short = mode1_image(2);
        short.truncate(short.len() - 1);
        assert!(CdRom::holding(Some(disc(short))).is_err());
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
        let drive = CdRom::holding(Some(disc(iso_image(10)))).expect("an ISO realizes");
        let toc = drive.port().toc().expect("a disc has one");
        assert_eq!(
            toc,
            Toc {
                first: 1,
                last: 1,
                control: 0x4,
                start: 0,
                lead_out: 10,
            }
        );
    }

    #[test]
    fn reading_past_the_last_block_is_refused() {
        let drive = CdRom::holding(Some(disc(iso_image(3)))).expect("an ISO realizes");
        let mut buf = [0u8; USER_BYTES as usize];
        assert!(drive.port().read(2, &mut buf).is_ok());
        assert!(drive.port().read(3, &mut buf).is_err());
    }

    #[test]
    fn the_head_position_survives_a_save_and_a_load() {
        let drive = CdRom::holding(Some(disc(iso_image(8)))).expect("an ISO realizes");
        let mut buf = [0u8; USER_BYTES as usize];
        drive.port().read(5, &mut buf).expect("it reads");
        assert_eq!(drive.port().at(), 5);

        let bytes = snapshot(&drive);
        let fresh = CdRom::holding(Some(disc(iso_image(8)))).expect("an ISO realizes");
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("cd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&fresh, &mut chunk.reader()).unwrap();
        assert_eq!(fresh.port().at(), 5);
        assert_eq!(snapshot(&fresh), bytes);
    }

    #[test]
    fn a_head_past_the_disc_is_refused_on_load() {
        let far = CdRom::holding(Some(disc(iso_image(128)))).expect("an ISO realizes");
        let mut buf = [0u8; USER_BYTES as usize];
        far.port().read(99, &mut buf).expect("it reads");
        let bytes = snapshot(&far);

        let small = CdRom::holding(Some(disc(iso_image(4)))).expect("an ISO realizes");
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("cd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&small, &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_reset_puts_the_head_back_and_leaves_the_disc_in() {
        let drive = CdRom::holding(Some(disc(iso_image(8)))).expect("an ISO realizes");
        let mut buf = [0u8; USER_BYTES as usize];
        drive.port().read(6, &mut buf).expect("it reads");
        drive.reset(ResetKind::Cold);
        assert_eq!(drive.port().at(), 0);
        assert!(drive.port().has_disc());
    }
}
