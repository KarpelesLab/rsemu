//! The CD32's CD-ROM drive: the disc in the tray and where the head is.
//!
//! One class, `amiga.cd`. It is the *mechanism* — a drive holding a disc — and
//! not a controller: [`super::akiko`] is the CD32's controller and is what a
//! guest talks to. The split is `ata.disk`'s and `pc.ide`'s and it holds for
//! the same reason: this file contains no register offset and no interrupt,
//! and Akiko contains no sector layout.
//!
//! The Developer Notes (*Amiga CD32 Developer Notes*, Revision 3,
//! Commodore-Amiga Inc.) describe the drive in one line each — "Top loading
//! double speed CD-ROM drive", 300 KB/s, and "Create an ISO-9660 image file
//! suitable for making the gold (master) disc" — and say nothing about its
//! interface. So the *disc* is modelled from the disc standards and the
//! interface is where [`super::akiko`] stops.
//!
//! # What a disc is, and which images this reads
//!
//! [`crate::dev::disc`], and not one line of it is here. Sector layout — 2048
//! bytes of user data or 2352-byte raw frames, told apart by ECMA-130 §14.1's
//! sync pattern — Mode 1 and Mode 2 Form 1 user-data extraction, the
//! minute/second/frame conversion with §20's 150-frame lead-in, and the
//! synthesised one-data-track table of contents are all bus-neutral facts
//! about a compact disc, and an ATAPI drive on a completely different cable
//! needs every one of them. That module's documentation argues each; this file
//! adds the *drive*: a tray, a head position, and the two-phase construction
//! and snapshot contract a machine object owes.
//!
//! **No audio track**, no `.cue` sheet, no subchannel and so no CD+G, no
//! multi-session and no Mode 2 Form 2 data. The Developer Notes list
//! "ISO-9660 CD-ROM, Audio CD, CD+G" as the drive's formats and the other two
//! are simply not done, rather than half done.
//!
//! # Speed
//!
//! "Double speed", 150 frames a second doubled, is recorded in
//! [`FRAMES_PER_SECOND`](crate::dev::disc::FRAMES_PER_SECOND) and used by
//! nothing: no command reaches this drive (see [`super::akiko`]), so there is
//! no transfer whose duration it could set. It is written down next to the
//! machine because the first command that ever arrives will want it.
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

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::disc::{Disc, Layout, Toc, USER_BYTES};
use crate::dev::medium::{self, Medium};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.cd";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

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
        self.inner.disc.as_ref().map_or(0, Disc::sectors)
    }

    /// How its sectors are laid out in the image, or `None` with no disc.
    #[must_use]
    pub fn layout(&self) -> Option<Layout> {
        self.inner.disc.as_ref().map(Disc::layout)
    }

    /// The disc's table of contents, or `None` with no disc.
    #[must_use]
    pub fn toc(&self) -> Option<Toc> {
        self.inner.disc.as_ref().map(Disc::toc)
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
        disc.read_block(CLASS_NAME, lba, dst)?;
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
            // Both layouts, because a CD32 master is delivered as an ISO and a
            // rip of one is raw frames, and `dev::disc` reads either. Which it
            // is, is decided by the sync pattern rather than by the length —
            // see that module for why a modulus cannot settle it.
            Some(bytes) => Some(Disc::open(CLASS_NAME, bytes)?),
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
        let sectors = self.inner.disc.as_ref().map_or(0, Disc::sectors);
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

#[cfg(test)]
mod tests {
    use super::*;

    use alloc::vec;
    use alloc::vec::Vec;

    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::dev::disc::{iso_image, mode1_image};

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

    #[test]
    fn a_disc_that_is_not_whole_sectors_is_refused() {
        assert!(CdRom::holding(Some(disc(vec![0u8; 2047]))).is_err());
        assert!(CdRom::holding(Some(disc(vec![0u8; 0]))).is_err());
        let mut short = mode1_image(2);
        short.truncate(short.len() - 1);
        assert!(CdRom::holding(Some(disc(short))).is_err());
    }

    #[test]
    fn the_table_of_contents_is_one_data_track() {
        let drive = CdRom::holding(Some(disc(iso_image(10)))).expect("an ISO realizes");
        let toc = drive.port().toc().expect("a disc has one");
        assert_eq!(toc, Toc::one_data_track(10));
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
