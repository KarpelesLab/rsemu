//! A minimal ISO 9660 image, built from nothing.
//!
//! Shared by [`tests/pc_at_cdrom.rs`](../pc_at_cdrom.rs), which drives a CD-ROM
//! through the ATAPI packet interface, and
//! [`tests/pc_at_eltorito.rs`](../pc_at_eltorito.rs), which boots one.
//!
//! **Nothing is vendored.** The disc every test here reads is assembled by
//! these functions and written into a temporary directory, which is the same
//! rule the rest of the tree follows for fixtures: a `.iso` in the repository
//! would be a binary blob with a licence nobody checked, and one fetched from
//! the network would make `cargo test` need one.
//!
//! # What is built
//!
//! Enough ISO 9660 for the structure to be real rather than a byte pattern that
//! happens to satisfy the one field a test looks at:
//!
//! ```text
//!   block  0-15  the system area, zero
//!   block  16    the primary volume descriptor
//!   block  17    the boot record volume descriptor   (El Torito only)
//!   block  18    the volume descriptor set terminator
//!   block  19    the root directory: `.`, `..` and one file
//!   block  20    the boot catalog                    (El Torito only)
//!   block  21    the file's contents
//!   block  22    the boot image                      (El Torito only)
//! ```
//!
//! # Sources
//!
//! * **ISO 9660:1988** — §8.4 (the primary volume descriptor's field layout),
//!   §8.2 (the boot record's), §9.1 (a directory record) and §7.2/§7.3 (the
//!   "both-endian" numerical forms, which carry every integer twice so that a
//!   reader of either endianness can take the half it understands).
//! * **"El Torito" Bootable CD-ROM Format Specification, Version 1.0**
//!   (Phoenix Technologies and IBM, 25 January 1995) — the boot record volume
//!   descriptor's identifier and catalog pointer, and the boot catalog's
//!   validation and initial/default entries.

#![allow(dead_code)]
// Two test binaries include this module and each uses a different half of it;
// `pub` here means "visible to the binary", which is what `pub(crate)` spells
// inside one.

/// How many bytes one logical block holds.
pub(crate) const BLOCK: usize = 2048;

/// Where the volume descriptor set starts. ISO 9660 §6.2.1: the first sixteen
/// blocks are the system area and the standard says nothing about them.
pub(crate) const PVD_BLOCK: u32 = 16;
/// Where [`bootable_disc`] puts the boot record volume descriptor.
pub(crate) const BOOT_RECORD_BLOCK: u32 = 17;
/// Where the volume descriptor set terminator goes.
pub(crate) const TERMINATOR_BLOCK: u32 = 18;
/// Where the root directory's one extent goes.
pub(crate) const ROOT_BLOCK: u32 = 19;
/// Where [`bootable_disc`] puts the boot catalog.
pub(crate) const CATALOG_BLOCK: u32 = 20;
/// Where the one file's contents go.
pub(crate) const FILE_BLOCK: u32 = 21;
/// Where [`bootable_disc`] puts the boot image.
pub(crate) const BOOT_IMAGE_BLOCK: u32 = 22;

/// The one file's name, in the form ISO 9660 §7.5 gives a file identifier:
/// upper case, eight-and-three, and a `;1` version suffix.
pub(crate) const FILE_NAME: &str = "HELLO.TXT;1";

/// The volume identifier the images carry, so that a test reading the primary
/// volume descriptor has something recognisable to assert on.
pub(crate) const VOLUME_ID: &str = "RSEMU_TEST";

/// A both-endian 32-bit number: ISO 9660 §7.3.3, little-endian then
/// big-endian, because a reader takes whichever half it understands.
fn both32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
    out.extend_from_slice(&value.to_be_bytes());
}

/// A both-endian 16-bit number, §7.2.3.
fn both16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
    out.extend_from_slice(&value.to_be_bytes());
}

/// `text`, space padded to `len` bytes — an `a-characters` or `d-characters`
/// field, §7.4.
fn padded(out: &mut Vec<u8>, text: &str, len: usize) {
    let bytes = text.as_bytes();
    let n = bytes.len().min(len);
    out.extend_from_slice(&bytes[..n]);
    out.extend(std::iter::repeat_n(b' ', len - n));
}

/// A seventeen-byte date-and-time field, §8.4.26. All zeroes with a zero
/// offset is the specified way of saying "not specified", which is the honest
/// thing for an image whose contents must be byte-identical on every host and
/// in every year (`CLAUDE.md`, determinism).
fn no_date(out: &mut Vec<u8>) {
    out.extend_from_slice(b"0000000000000000");
    out.push(0);
}

/// One directory record, §9.1.
///
/// `name` is the file identifier; an empty name is the `.` entry and a
/// one-byte `\x01` is `..`, which is how ISO 9660 spells the two of them.
fn directory_record(extent: u32, length: u32, directory: bool, name: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let len = 33 + name.len() + usize::from(name.len().is_multiple_of(2));
    out.push(len as u8);
    out.push(0); // extended attribute record length
    both32(&mut out, extent);
    both32(&mut out, length);
    // The recording date and time, §9.1.5: seven bytes, and zero throughout for
    // the reason `no_date` gives.
    out.extend_from_slice(&[0u8; 7]);
    out.push(if directory { 0x02 } else { 0x00 }); // file flags
    out.push(0); // file unit size
    out.push(0); // interleave gap size
    both16(&mut out, 1); // volume sequence number
    out.push(name.len() as u8);
    out.extend_from_slice(name);
    // §9.1.12: a record's length is even, so a name of even length is followed
    // by one padding byte.
    while out.len() < len {
        out.push(0);
    }
    out
}

/// The primary volume descriptor, §8.4.
fn primary_volume_descriptor(blocks: u32, root_extent: u32, root_length: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(BLOCK);
    out.push(1); // volume descriptor type: primary
    out.extend_from_slice(b"CD001"); // standard identifier
    out.push(1); // volume descriptor version
    out.push(0); // unused
    padded(&mut out, "", 32); // system identifier
    padded(&mut out, VOLUME_ID, 32); // volume identifier
    out.extend_from_slice(&[0u8; 8]); // unused
    both32(&mut out, blocks); // volume space size, in logical blocks
    out.extend_from_slice(&[0u8; 32]); // unused
    both16(&mut out, 1); // volume set size
    both16(&mut out, 1); // volume sequence number
    both16(&mut out, BLOCK as u16); // logical block size
    both32(&mut out, 0); // path table size
    out.extend_from_slice(&0u32.to_le_bytes()); // type-L path table
    out.extend_from_slice(&0u32.to_le_bytes()); // optional type-L path table
    out.extend_from_slice(&0u32.to_be_bytes()); // type-M path table
    out.extend_from_slice(&0u32.to_be_bytes()); // optional type-M path table
    out.extend_from_slice(&directory_record(root_extent, root_length, true, &[0u8]));
    padded(&mut out, "", 128); // volume set identifier
    padded(&mut out, "RSEMU", 128); // publisher identifier
    padded(&mut out, "RSEMU", 128); // data preparer identifier
    padded(&mut out, "RSEMU", 128); // application identifier
    padded(&mut out, "", 37); // copyright file identifier
    padded(&mut out, "", 37); // abstract file identifier
    padded(&mut out, "", 37); // bibliographic file identifier
    no_date(&mut out); // volume creation
    no_date(&mut out); // volume modification
    no_date(&mut out); // volume expiration
    no_date(&mut out); // volume effective
    out.push(1); // file structure version
    out.push(0); // reserved
    out.resize(BLOCK, 0);
    out
}

/// The volume descriptor set terminator, §8.3.
fn terminator() -> Vec<u8> {
    let mut out = Vec::with_capacity(BLOCK);
    out.push(0xff);
    out.extend_from_slice(b"CD001");
    out.push(1);
    out.resize(BLOCK, 0);
    out
}

/// The root directory's extent: `.`, `..` and one file.
fn root_directory(file_length: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(BLOCK);
    out.extend_from_slice(&directory_record(ROOT_BLOCK, BLOCK as u32, true, &[0x00]));
    out.extend_from_slice(&directory_record(ROOT_BLOCK, BLOCK as u32, true, &[0x01]));
    out.extend_from_slice(&directory_record(
        FILE_BLOCK,
        file_length,
        false,
        FILE_NAME.as_bytes(),
    ));
    out.resize(BLOCK, 0);
    out
}

/// A plain data disc: one file, whose contents are `contents`.
///
/// The file must fit in one logical block, which is all the directory record
/// above describes and all any test here needs.
#[must_use]
pub(crate) fn data_disc(contents: &[u8]) -> Vec<u8> {
    assert!(contents.len() <= BLOCK, "the one file fits in one block");
    let blocks = (FILE_BLOCK + 1) as usize;
    let mut out = vec![0u8; blocks * BLOCK];
    put(
        &mut out,
        PVD_BLOCK,
        &primary_volume_descriptor(blocks as u32, ROOT_BLOCK, BLOCK as u32),
    );
    put(&mut out, TERMINATOR_BLOCK, &terminator());
    put(&mut out, ROOT_BLOCK, &root_directory(contents.len() as u32));
    put(&mut out, FILE_BLOCK, contents);
    out
}

/// What kind of boot the catalog's initial/default entry describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Emulation {
    /// Media type 0: the image is loaded at a load segment and entered.
    None {
        /// The segment the image is loaded at. Zero means 07C0h.
        load_segment: u16,
        /// How many 512-byte virtual sectors of it to load.
        sectors: u16,
    },
    /// Media type 2: the image is a 1.44 MB diskette and becomes drive 00h.
    Diskette144,
    /// Media type 4, which rsemu's firmware declines — here so that a test can
    /// prove it declines rather than loading something it cannot service.
    HardDisk,
}

impl Emulation {
    fn media_type(self) -> u8 {
        match self {
            Emulation::None { .. } => 0,
            Emulation::Diskette144 => 2,
            Emulation::HardDisk => 4,
        }
    }
}

/// A bootable disc: the data disc above plus an El Torito boot record, a boot
/// catalog and `image` as the boot image.
///
/// `bootable` is the initial/default entry's boot indicator: `true` writes
/// `0x88` and `false` writes `0x00`, which a test uses to check that a catalog
/// describing an image nobody is meant to boot is declined.
#[must_use]
pub(crate) fn bootable_disc(
    contents: &[u8],
    image: &[u8],
    emulation: Emulation,
    bootable: bool,
) -> Vec<u8> {
    let image_blocks = image.len().div_ceil(BLOCK).max(1);
    let blocks = BOOT_IMAGE_BLOCK as usize + image_blocks;
    let mut out = vec![0u8; blocks * BLOCK];
    put(
        &mut out,
        PVD_BLOCK,
        &primary_volume_descriptor(blocks as u32, ROOT_BLOCK, BLOCK as u32),
    );
    put(&mut out, BOOT_RECORD_BLOCK, &boot_record(CATALOG_BLOCK));
    put(&mut out, TERMINATOR_BLOCK, &terminator());
    put(&mut out, ROOT_BLOCK, &root_directory(contents.len() as u32));
    put(&mut out, CATALOG_BLOCK, &boot_catalog(emulation, bootable));
    put(&mut out, FILE_BLOCK, contents);
    put(&mut out, BOOT_IMAGE_BLOCK, image);
    out
}

/// The boot record volume descriptor, El Torito §"Boot Record".
fn boot_record(catalog: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(BLOCK);
    out.push(0); // volume descriptor type: a boot record
    out.extend_from_slice(b"CD001");
    out.push(1); // version
    padded(&mut out, "", 0);
    // The boot system identifier, thirty-two bytes, and the string that makes
    // this boot record El Torito's rather than somebody else's.
    let mut id = b"EL TORITO SPECIFICATION".to_vec();
    id.resize(32, 0);
    out.extend_from_slice(&id);
    out.extend_from_slice(&[0u8; 32]); // boot identifier, unused
    out.extend_from_slice(&catalog.to_le_bytes()); // the boot catalog's block
    out.resize(BLOCK, 0);
    out
}

/// The boot catalog: a validation entry and an initial/default entry.
fn boot_catalog(emulation: Emulation, bootable: bool) -> Vec<u8> {
    let mut out = Vec::with_capacity(BLOCK);

    // -- the validation entry, El Torito §"Validation Entry" -----------------
    let mut validation = Vec::with_capacity(32);
    validation.push(0x01); // header ID
    validation.push(0x00); // platform ID: 80x86
    validation.extend_from_slice(&[0, 0]); // reserved
    let mut id = b"RSEMU".to_vec();
    id.resize(24, 0);
    validation.extend_from_slice(&id);
    validation.extend_from_slice(&[0, 0]); // the checksum, filled in below
    validation.push(0x55);
    validation.push(0xaa);
    // "The sum of all the words in the entry, including the checksum, must be
    // zero" — so the checksum is the two's complement of the rest.
    let sum = validation
        .chunks(2)
        .map(|w| u16::from_le_bytes([w[0], w[1]]))
        .fold(0u16, u16::wrapping_add);
    let checksum = 0u16.wrapping_sub(sum);
    validation[28..30].copy_from_slice(&checksum.to_le_bytes());
    out.extend_from_slice(&validation);

    // -- the initial/default entry -------------------------------------------
    let (load_segment, sectors) = match emulation {
        Emulation::None {
            load_segment,
            sectors,
        } => (load_segment, sectors),
        // A diskette emulation's load segment and sector count are ignored:
        // the specification fixes them at 0000:7C00 and one virtual sector.
        _ => (0, 1),
    };
    out.push(if bootable { 0x88 } else { 0x00 });
    out.push(emulation.media_type());
    out.extend_from_slice(&load_segment.to_le_bytes());
    out.push(0); // system type, out of the image's partition table
    out.push(0); // unused
    out.extend_from_slice(&sectors.to_le_bytes());
    out.extend_from_slice(&BOOT_IMAGE_BLOCK.to_le_bytes());
    out.extend_from_slice(&[0u8; 20]); // unused

    out.resize(BLOCK, 0);
    out
}

fn put(image: &mut [u8], block: u32, bytes: &[u8]) {
    let at = block as usize * BLOCK;
    image[at..at + bytes.len()].copy_from_slice(bytes);
}

/// Write `image` into a fresh temporary directory and hand back the path.
///
/// The directory is not cleaned up by this function: a test that wants it gone
/// removes it, and one that fails leaves the disc behind to be looked at.
#[must_use]
pub(crate) fn write_temp(name: &str, image: &[u8]) -> std::path::PathBuf {
    let mut dir = std::env::temp_dir();
    dir.push(format!("rsemu-iso-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("a temporary directory");
    let path = dir.join(format!("{name}.iso"));
    std::fs::write(&path, image).expect("the disc image");
    path
}
