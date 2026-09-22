//! The disk in the drive: a 400K or 800K image, and the cylinders it becomes.
//!
//! Two containers are understood, because those are the two a Macintosh floppy
//! image comes in.
//!
//! * A **raw** image: the blocks end to end, 409,600 bytes for a 400K disk and
//!   819,200 for an 800K one, with no tags.
//! * A **DiskCopy 4.2** container: an 84-byte header and then the same bytes,
//!   optionally followed by twelve tag bytes a block.
//!
//! # The DiskCopy 4.2 header
//!
//! | offset | size | field |
//! | --- | --- | --- |
//! | `$00` | 1 | the length of the Pascal name that follows |
//! | `$01` | 63 | the name, padded with zeros |
//! | `$40` | 4 | `dataSize`, big-endian, of the block at `$54` |
//! | `$44` | 4 | `tagSize`, of the block after it |
//! | `$48` | 4 | `dataChecksum` |
//! | `$4C` | 4 | `tagChecksum` |
//! | `$50` | 1 | `diskFormat` |
//! | `$51` | 1 | `formatByte` |
//! | `$52` | 2 | `$0100`, "if it is not `$01 $00`, it is not a DC42 format file" |
//!
//! `diskFormat` is 0 for 400K GCR, 1 for 800K GCR, 2 for 720K MFM and **3 for
//! 1440K MFM**. Both checksums are the same arithmetic: a 32-bit accumulator
//! starting at zero, and for each big-endian 16-bit word of the section, add the
//! word and then rotate the accumulator right one bit. The first twelve bytes
//! of the tag section are skipped in its checksum, because they belong to the
//! boot block and DiskCopy did not include them.
//!
//! # Why a 1.44 MB image is refused by name
//!
//! Because a Macintosh Plus cannot read one, and no amount of work on this
//! board changes that. A Plus has an **IWM** and an 800K double-density drive.
//! 1.44 MB needs the **SWIM** controller and high-density media, both of which
//! arrived with the Macintosh SE FDHD in 1989 — three years after this machine,
//! and the Guide's chapter 9 lists the two interfaces separately for exactly
//! that reason. A board that read a 1.44 MB disk would not be a Plus.
//!
//! So [`Disk::from_image`] recognises the format, says what it is and why the
//! drive cannot take it, and refuses. The alternative — letting it through and
//! delivering nonsense to the head — is a bug report about the emulator for
//! something that is a fact about the hardware.
//!
//! # Where a block sits
//!
//! Block zero is cylinder 0, side 0, sector 0; blocks run up through the
//! sectors of a cylinder, then through the sides of it, then outward. A
//! single-sided disk has one side and half the blocks. The cylinder's sector
//! count comes from [`gcr::sectors_on`], so the zones are what decide where a
//! block lands rather than a table.
//!
//! No emulator source was consulted (`ROADMAP.md` §1).

use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use super::gcr::{self, DATA_BYTES, MAX_TRACK, Sector, TAG_BYTES, TRACKS, Track};
use super::mfm;
use crate::core::error::{Error, Result};

/// How the bits on a disk are written, which decides which controller can read
/// it and how fast the spindle turns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Density {
    /// Apple's zoned 6-and-2 GCR: a 400K or an 800K disk, five speed zones,
    /// 500 kHz cells. An IWM and a SWIM can both read one ([`super::gcr`]).
    Gcr,
    /// IBM MFM at 500 kbit/s: a 1.44 MB disk, 300 rpm on every cylinder,
    /// 1 MHz cells. **Only a SWIM** ([`super::mfm`]).
    Mfm,
}

/// Which controller is asking for an image, because that is what decides
/// whether a 1.44 MB one can be read at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reader {
    /// A Macintosh Plus's IWM and its 800K double-density mechanism. A 1.44 MB
    /// image is refused **by name** — see the module docs.
    Iwm,
    /// A Macintosh Classic's SWIM and its SuperDrive, which reads all three.
    Swim,
}

/// How long a DiskCopy 4.2 header is.
pub const DC42_HEADER: usize = 84;
/// The word at `$52` that says a file is one.
pub const DC42_MAGIC: u16 = 0x0100;

/// `diskFormat`: 400K GCR, single-sided.
pub const DISK_FORMAT_400K: u8 = 0;
/// `diskFormat`: 800K GCR, double-sided.
pub const DISK_FORMAT_800K: u8 = 1;
/// `diskFormat`: 720K MFM. Not a drive a Plus has.
pub const DISK_FORMAT_720K: u8 = 2;
/// `diskFormat`: 1440K MFM. Not a drive a Plus has — see the module docs.
pub const DISK_FORMAT_1440K: u8 = 3;

/// How many bytes of data a 400K disk holds.
pub const BYTES_400K: usize = 409_600;
/// And an 800K one.
pub const BYTES_800K: usize = 819_200;
/// And a 1.44 MB one, which is here only so that the refusal can name it.
pub const BYTES_1440K: usize = 1_474_560;

/// A disk in a drive.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Disk {
    /// How its bits are written.
    density: Density,
    /// One side or two.
    sides: u8,
    /// The address field's format byte: [`gcr::FORMAT_800K`] or
    /// [`gcr::FORMAT_400K`].
    format: u8,
    /// 512 bytes a block, in block order.
    data: Vec<u8>,
    /// Twelve bytes a block, in the same order. Zero-filled when the container
    /// carried none.
    tags: Vec<u8>,
    /// Whether the write-protect tab is over the hole.
    write_protect: bool,
    /// What the container called itself, for a message.
    name: String,
}

impl Disk {
    /// Read an image: a DiskCopy 4.2 container or a raw one.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] with a message that says what the image is, when it is
    /// a size or a format this drive cannot take — a 1.44 MB image most of all,
    /// for the reason in the module docs.
    pub fn from_image(bytes: &[u8]) -> Result<Disk> {
        Disk::from_image_for(bytes, Reader::Iwm)
    }

    /// The same, for a controller that says which it is.
    ///
    /// [`Reader::Swim`] is what lets a **1.44 MB** image through, and nothing
    /// else does: the refusal [`Reader::Iwm`] gets is a fact about a Macintosh
    /// Plus's hardware rather than a limitation of this model, and turning it
    /// into a silent success would be worse than the error.
    ///
    /// # Errors
    ///
    /// As [`Disk::from_image`].
    pub fn from_image_for(bytes: &[u8], reader: Reader) -> Result<Disk> {
        match Dc42::parse(bytes)? {
            Some(dc42) => Disk::from_dc42(bytes, &dc42, reader),
            None => Disk::from_raw(bytes, reader),
        }
    }

    fn refuse(what: &str, why: &str) -> Error {
        Error::Config {
            at: String::from("mac.disk"),
            message: format!("{what}; {why}"),
        }
    }

    /// The message every "this drive cannot take that" refusal ends with.
    fn plus_has_an_iwm() -> String {
        String::from(
            "a Macintosh Plus has an IWM and an 800K double-density drive, and 1.44 MB needs \
             the SWIM controller and high-density media that arrived with the Macintosh SE \
             FDHD in 1989. Give it a 400K or 800K image instead",
        )
    }

    fn from_dc42(bytes: &[u8], dc42: &Dc42, reader: Reader) -> Result<Disk> {
        let sides = match dc42.disk_format {
            DISK_FORMAT_400K => 1,
            DISK_FORMAT_800K => 2,
            DISK_FORMAT_1440K if reader == Reader::Swim => {
                let start = DC42_HEADER;
                let data = &bytes[start..start + dc42.data_size as usize];
                return Disk::build_mfm(data, dc42.name.clone());
            }
            DISK_FORMAT_1440K => {
                return Err(Disk::refuse(
                    &format!(
                        "`{}` is a DiskCopy 4.2 container of a 1.44 MB disk (diskFormat 3, \
                         dataSize {})",
                        dc42.name, dc42.data_size
                    ),
                    &Disk::plus_has_an_iwm(),
                ));
            }
            DISK_FORMAT_720K => {
                return Err(Disk::refuse(
                    &format!(
                        "`{}` is a DiskCopy 4.2 container of a 720K MFM disk (diskFormat 2)",
                        dc42.name
                    ),
                    &Disk::plus_has_an_iwm(),
                ));
            }
            other => {
                return Err(Disk::refuse(
                    &format!("`{}` has diskFormat {other}", dc42.name),
                    "which is not one of 0 (400K GCR), 1 (800K GCR), 2 (720K MFM) or 3 (1440K \
                     MFM)",
                ));
            }
        };
        let start = DC42_HEADER;
        let data = &bytes[start..start + dc42.data_size as usize];
        let tags = &bytes[start + dc42.data_size as usize..][..dc42.tag_size as usize];
        Disk::build(sides, data, tags, dc42.name.clone())
    }

    fn from_raw(bytes: &[u8], reader: Reader) -> Result<Disk> {
        let sides = match bytes.len() {
            BYTES_400K => 1,
            BYTES_800K => 2,
            BYTES_1440K if reader == Reader::Swim => {
                return Disk::build_mfm(bytes, String::new());
            }
            BYTES_1440K => {
                return Err(Disk::refuse(
                    "this image is 1,474,560 bytes, which is a 1.44 MB disk",
                    &Disk::plus_has_an_iwm(),
                ));
            }
            other => {
                return Err(Disk::refuse(
                    &format!("this image is {other} bytes"),
                    "a raw Macintosh floppy image is 409,600 bytes (400K) or 819,200 (800K). A \
                     DiskCopy 4.2 container is recognised by its own header",
                ));
            }
        };
        Disk::build(sides, bytes, &[], String::new())
    }

    /// A 1.44 MB disk: two sides, eighteen sectors a track, MFM.
    ///
    /// There are no tags: the twelve bytes an Apple GCR sector carries beside
    /// its data have nowhere to live in an IBM sector, which is exactly why a
    /// DiskCopy container of one has `tagSize = 0`.
    fn build_mfm(data: &[u8], name: String) -> Result<Disk> {
        if data.len() != mfm::BYTES {
            return Err(Disk::refuse(
                &format!(
                    "the data fork is {} bytes and a 1.44 MB disk holds {}",
                    data.len(),
                    mfm::BYTES
                ),
                "the header and the file disagree",
            ));
        }
        Ok(Disk {
            density: Density::Mfm,
            sides: 2,
            format: gcr::FORMAT_800K,
            data: data.to_vec(),
            tags: Vec::new(),
            write_protect: false,
            name,
        })
    }

    fn build(sides: u8, data: &[u8], tags: &[u8], name: String) -> Result<Disk> {
        let blocks = gcr::sectors_per_side() * usize::from(sides);
        if data.len() != blocks * DATA_BYTES {
            return Err(Disk::refuse(
                &format!(
                    "the data fork is {} bytes and a {}-sided disk holds {}",
                    data.len(),
                    sides,
                    blocks * DATA_BYTES
                ),
                "the header and the file disagree",
            ));
        }
        let mut tag_bytes = vec![0u8; blocks * TAG_BYTES];
        let n = tags.len().min(tag_bytes.len());
        tag_bytes[..n].copy_from_slice(&tags[..n]);
        Ok(Disk {
            density: Density::Gcr,
            sides,
            format: if sides == 2 {
                gcr::FORMAT_800K
            } else {
                gcr::FORMAT_400K
            },
            data: data.to_vec(),
            tags: tag_bytes,
            write_protect: false,
            name,
        })
    }

    /// A blank formatted disk of `sides` sides, for a test or a `format`.
    #[must_use]
    pub fn blank(sides: u8) -> Disk {
        let sides = sides.clamp(1, 2);
        let blocks = gcr::sectors_per_side() * usize::from(sides);
        Disk {
            density: Density::Gcr,
            sides,
            format: if sides == 2 {
                gcr::FORMAT_800K
            } else {
                gcr::FORMAT_400K
            },
            data: vec![0; blocks * DATA_BYTES],
            tags: vec![0; blocks * TAG_BYTES],
            write_protect: false,
            name: String::new(),
        }
    }

    /// A blank formatted 1.44 MB disk, for a test or a `format`.
    #[must_use]
    pub fn blank_mfm() -> Disk {
        Disk {
            density: Density::Mfm,
            sides: 2,
            format: gcr::FORMAT_800K,
            data: vec![0; mfm::BYTES],
            tags: Vec::new(),
            write_protect: false,
            name: String::new(),
        }
    }

    /// How its bits are written, which is what decides which controller can
    /// read it and how long a cylinder is.
    #[must_use]
    pub fn density(&self) -> Density {
        self.density
    }

    /// How many 512-byte blocks it holds: 800 a side.
    #[must_use]
    pub fn blocks(&self) -> usize {
        self.data.len() / DATA_BYTES
    }

    /// One side or two.
    #[must_use]
    pub fn sides(&self) -> u8 {
        self.sides
    }

    /// What the container called itself, or an empty string.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Whether the tab is over the hole.
    #[must_use]
    pub fn write_protected(&self) -> bool {
        self.write_protect
    }

    /// Move the tab.
    pub fn set_write_protected(&mut self, protect: bool) {
        self.write_protect = protect;
    }

    /// The 512 bytes of block `n`, if it has one.
    #[must_use]
    pub fn block(&self, n: usize) -> Option<&[u8]> {
        self.data.get(n * DATA_BYTES..(n + 1) * DATA_BYTES)
    }

    /// Which block sits at `sector` of `track` on `side`, if any does.
    ///
    /// The running total over the zones, which is why the zone table is the
    /// only place a sector count is written down.
    #[must_use]
    pub fn block_of(&self, track: u8, side: bool, sector: u8) -> Option<usize> {
        if self.density == Density::Mfm {
            return mfm::block_of(track, u8::from(side), sector + 1);
        }
        if track > MAX_TRACK || (side && self.sides < 2) {
            return None;
        }
        let per = usize::from(gcr::sectors_on(track));
        if usize::from(sector) >= per {
            return None;
        }
        let mut block = 0usize;
        for t in 0..track {
            block += usize::from(gcr::sectors_on(t)) * usize::from(self.sides);
        }
        if side {
            block += per;
        }
        Some(block + usize::from(sector))
    }

    /// Build the bit stream of one cylinder and side.
    ///
    /// Derived state: never snapshotted, rebuilt whenever the head moves.
    #[must_use]
    pub fn track(&self, track: u8, side: bool) -> Track {
        if self.density == Density::Mfm {
            return self.mfm_track(track, u8::from(side));
        }
        if track > MAX_TRACK || (side && self.sides < 2) {
            return Track::new();
        }
        let sectors: Vec<Sector> = (0..gcr::sectors_on(track))
            .filter_map(|s| {
                let block = self.block_of(track, side, s)?;
                Some(Sector::new(
                    track,
                    side,
                    s,
                    self.format,
                    &self.tags[block * TAG_BYTES..(block + 1) * TAG_BYTES],
                    &self.data[block * DATA_BYTES..(block + 1) * DATA_BYTES],
                ))
            })
            .collect();
        gcr::encode_track(&sectors)
    }

    /// Build the MFM bit stream of one cylinder and head of a 1.44 MB disk.
    ///
    /// Derived state: never snapshotted, rebuilt whenever the head moves. The
    /// sector numbers an ID field carries are **one-based**, which is the IBM
    /// format's and not Apple's.
    #[must_use]
    pub fn mfm_track(&self, cylinder: u8, head: u8) -> Track {
        if self.density != Density::Mfm
            || usize::from(cylinder) >= mfm::CYLINDERS
            || usize::from(head) >= mfm::SIDES
        {
            return Track::new();
        }
        let sectors: Vec<mfm::Sector> = (1..=mfm::SECTORS as u8)
            .filter_map(|s| {
                let block = mfm::block_of(cylinder, head, s)?;
                let data = self
                    .data
                    .get(block * DATA_BYTES..(block + 1) * DATA_BYTES)?;
                Some(mfm::Sector::new(cylinder, head, s, data))
            })
            .collect();
        mfm::encode_track(&sectors)
    }
}

/// A DiskCopy 4.2 header as it is on disk.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dc42 {
    /// The Pascal name, trimmed.
    pub name: String,
    /// `dataSize`, of the block at `$54`.
    pub data_size: u32,
    /// `tagSize`, of the block after it.
    pub tag_size: u32,
    /// `dataChecksum` as stored.
    pub data_checksum: u32,
    /// `tagChecksum` as stored.
    pub tag_checksum: u32,
    /// `diskFormat`.
    pub disk_format: u8,
    /// `formatByte`: `$02` for a Macintosh 400K, `$22` for an 800K.
    pub format_byte: u8,
}

impl Dc42 {
    /// Parse the header, or `None` if `bytes` is not a DiskCopy 4.2 container.
    ///
    /// The magic alone is not enough to be sure — two bytes match by accident
    /// often enough — so the lengths have to agree with the file as well, which
    /// is the check that tells a container from a raw image whose 83rd and 84th
    /// bytes happen to be `$01 $00`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] when the header *is* one and its fields do not fit the
    /// file, which is a damaged container rather than a raw image.
    pub fn parse(bytes: &[u8]) -> Result<Option<Dc42>> {
        if bytes.len() < DC42_HEADER {
            return Ok(None);
        }
        let be32 = |at: usize| {
            u32::from_be_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        if u16::from_be_bytes([bytes[0x52], bytes[0x53]]) != DC42_MAGIC {
            return Ok(None);
        }
        let data_size = be32(0x40);
        let tag_size = be32(0x44);
        let total = DC42_HEADER as u64 + u64::from(data_size) + u64::from(tag_size);
        if total != bytes.len() as u64 {
            // The magic is there, so this is meant to be a container; a length
            // that does not add up is a truncated file rather than a raw image
            // and saying so is more use than treating it as one.
            return Err(Error::Config {
                at: String::from("mac.disk"),
                message: format!(
                    "a DiskCopy 4.2 header says {DC42_HEADER} + {data_size} + {tag_size} = \
                     {total} bytes and the file is {}",
                    bytes.len()
                ),
            });
        }
        let len = usize::from(bytes[0]).min(63);
        let name = bytes[1..1 + len]
            .iter()
            .map(|&c| {
                if (0x20..0x7f).contains(&c) {
                    c as char
                } else {
                    '.'
                }
            })
            .collect();
        Ok(Some(Dc42 {
            name,
            data_size,
            tag_size,
            data_checksum: be32(0x48),
            tag_checksum: be32(0x4c),
            disk_format: bytes[0x50],
            format_byte: bytes[0x51],
        }))
    }

    /// DiskCopy's checksum over `section`: a 32-bit accumulator, and for each
    /// big-endian 16-bit word, add it and then rotate the accumulator right one
    /// bit.
    ///
    /// An odd trailing byte is not part of any word and is ignored, which is
    /// what the arithmetic says; a Macintosh disk image never has one.
    #[must_use]
    pub fn checksum(section: &[u8]) -> u32 {
        let mut acc = 0u32;
        for word in section.as_chunks::<2>().0 {
            acc = acc.wrapping_add(u32::from(u16::from_be_bytes([word[0], word[1]])));
            acc = acc.rotate_right(1);
        }
        acc
    }

    /// The checksum of the tag section, which skips its first twelve bytes:
    /// they belong to the boot block and DiskCopy left them out.
    #[must_use]
    pub fn tag_section_checksum(tags: &[u8]) -> u32 {
        if tags.len() <= TAG_BYTES {
            return 0;
        }
        Dc42::checksum(&tags[TAG_BYTES..])
    }

    /// How many blocks the container claims.
    #[must_use]
    pub fn blocks(&self) -> usize {
        self.data_size as usize / DATA_BYTES
    }

    /// A one-line description, for a message or a listing.
    #[must_use]
    pub fn describe(&self) -> String {
        let what = match self.disk_format {
            DISK_FORMAT_400K => "400K GCR, single-sided",
            DISK_FORMAT_800K => "800K GCR, double-sided",
            DISK_FORMAT_720K => "720K MFM",
            DISK_FORMAT_1440K => "1.44 MB MFM",
            _ => "an unknown format",
        };
        format!(
            "`{}`: {what} (diskFormat {}, formatByte ${:02x}), {} bytes of data and {} of tag",
            self.name, self.disk_format, self.format_byte, self.data_size, self.tag_size
        )
    }
}

/// How many cylinders a Macintosh floppy has, re-exported so a caller does not
/// have to reach into [`gcr`] for it.
pub const CYLINDERS: usize = TRACKS;

#[cfg(test)]
mod tests;
