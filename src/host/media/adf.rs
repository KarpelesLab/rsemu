//! Amiga ADF disk images, from a file or straight out of an Amiga Forever disc.
//!
//! An ADF is AmigaDOS's sectors back to back — 80 cylinders × 2 sides × 11
//! sectors × 512 bytes, 901 120 bytes — and nothing else: no header, no
//! magic, no checksum of its own. So there is little to *decode* here. What
//! this source adds over a bare path is the one thing a user of Amiga Forever
//! actually needs, which is not having to extract anything:
//!
//! ```text
//!   adf:<file>                     an .adf, checked for its length
//!   adf:<dvd.iso>,disk=<name>      out of an Amiga Forever disc image
//!   adf:<dvd.iso>                  fails, and lists the disks the disc holds
//! ```
//!
//! The bytes then go into a media slot like any other, and `amiga.floppy`
//! encodes them into the MFM tracks a drive head presents
//! (`dev::amiga::adf`). A disk read this way is a **copy**:
//! the guest's writes live in the session and never reach the file, which for
//! an image inside an ISO is the only thing possible and for a user's single
//! copy of a Workbench disk is the only safe default. `--drive df0=disk.adf`
//! is the spelling that writes through; `docs/platforms/amiga.md` has the
//! whole argument.
//!
//! # What the note says
//!
//! Whether the disk will boot, because that is the question a user who picked
//! one out of a list of 28 has. From the *Amiga ROM Kernel Reference Manual:
//! Devices* (Commodore-Amiga, 3rd edition), Appendix C, "Floppy Boot Process":
//! the first two sectors are the boot block, "the first three longwords come
//! from the include file `devices/bootblock.h`. The type must be `BBID_DOS`;
//! the checksum must be correct (an additive carry wraparound sum of
//! `0xffffffff`)". The fourth byte of the type is printed as it is rather than
//! interpreted.
//!
//! # Where the disc keeps them
//!
//! ```text
//!   /Amiga Files/Shared/adf/<name>.adf
//! ```
//!
//! beside the ROMs [`kickstart`](super::kickstart) reads from, on the disc and
//! in an installed copy alike. `disk=` resolves there, with `.adf` optional; a
//! name with a `/` in it is a whole path into the volume.
//!
//! # Provenance
//!
//! The Commodore manual above for the boot block, ECMA-119 for where `CD001`
//! sits, and `fstool`'s ISO 9660 reader for the rest. No emulator source was
//! consulted.

use alloc::borrow::ToOwned;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::PathBuf;

use crate::core::error::{Error, Result};

/// A double-density ADF: 1760 sectors of 512 bytes.
pub const ADF_BYTES: usize = 80 * 2 * 11 * 512;

/// A high-density one, 22 sectors a track. Named so it can be refused by name.
pub const ADF_HD_BYTES: usize = 2 * ADF_BYTES;

/// The directory an Amiga Forever disc keeps its disk images in.
pub const ISO_ADF_DIR: &str = "/Amiga Files/Shared/adf";

/// A disk image, read and checked.
#[derive(Debug, Clone)]
pub struct Disk {
    /// Where it came from, for messages.
    pub origin: String,
    /// The sectors, which is what goes in the media slot.
    pub bytes: Vec<u8>,
    /// The boot block: the fourth byte of its `DOS` type and whether its
    /// checksum holds, or `None` if it has no `DOS` type at all.
    pub boot: Option<(u8, bool)>,
}

impl Disk {
    /// One line about the disk, for the run's own log.
    #[must_use]
    pub fn describe(&self) -> String {
        let boot = match self.boot {
            Some((flags, true)) => {
                format!("a bootable `DOS\\{flags}` boot block, checksum verified")
            }
            Some((flags, false)) => format!(
                "a `DOS\\{flags}` boot block whose checksum does not hold, so Kickstart will not \
                 boot it"
            ),
            None => "no DOS boot block, so it is a data disk".to_owned(),
        };
        format!("ADF, {} KiB, {boot}", self.bytes.len() / 1024)
    }
}

/// Check an ADF's length and read its boot block.
///
/// # Errors
///
/// [`Error::Config`] if the length is not a double-density ADF's.
pub fn parse(origin: &str, bytes: Vec<u8>) -> Result<Disk> {
    if bytes.len() != ADF_BYTES {
        let why = if bytes.len() == ADF_HD_BYTES {
            "a high-density ADF, and an A500's drive is double density".to_owned()
        } else {
            format!("not an ADF, which is {ADF_BYTES} bytes: 80 cylinders, 2 sides, 11 sectors")
        };
        return Err(config(origin, format!("is {} bytes, {why}", bytes.len())));
    }
    let boot = boot_block(&bytes);
    Ok(Disk {
        origin: origin.to_owned(),
        bytes,
        boot,
    })
}

/// The boot block's `DOS` type byte and whether its checksum holds.
///
/// The sum is over the two boot sectors as big-endian longwords, the carry out
/// of bit 31 added back in, and a good block sums to `$FFFF_FFFF` — the manual's
/// "additive carry wraparound sum".
#[must_use]
pub fn boot_block(image: &[u8]) -> Option<(u8, bool)> {
    if image.len() < 1024 || &image[..3] != b"DOS" {
        return None;
    }
    let mut sum = 0u32;
    for chunk in image[..1024].as_chunks::<4>().0 {
        let long = u32::from_be_bytes(*chunk);
        let (next, carried) = sum.overflowing_add(long);
        sum = next.wrapping_add(u32::from(carried));
    }
    Some((image[3], sum == u32::MAX))
}

/// Everything after `adf:` on a media specification.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Spec {
    source: PathBuf,
    disk: Option<String>,
}

impl Spec {
    fn parse(spec: &str) -> Result<Spec> {
        // Source first, options after, as `kickstart:` and `--drive` have it.
        let mut parts = spec.split(',');
        let source = parts.next().unwrap_or("");
        if source.is_empty() {
            return Err(config(
                "adf",
                "needs a file: `adf:<disk.adf>`, or `adf:<dvd.iso>,disk=<name>`".to_owned(),
            ));
        }
        let mut out = Spec {
            source: PathBuf::from(source),
            disk: None,
        };
        for opt in parts {
            match opt.split_once('=') {
                Some(("disk", name)) => out.disk = Some(name.to_owned()),
                _ => {
                    return Err(config(
                        source,
                        format!("`{opt}` is not an adf option; the one there is is `disk=`"),
                    ));
                }
            }
        }
        Ok(out)
    }
}

/// Read a disk the way the user's own media holds it.
///
/// `spec` is everything after `adf:`. Nothing is copied or written: the file,
/// or the disc, is read where it is.
///
/// # Errors
///
/// [`Error::Config`] for a malformed specification, a file that cannot be read,
/// a disc that is not an Amiga Forever one or does not hold the disk named, or
/// bytes that are not a double-density ADF.
pub fn open(spec: &str) -> Result<Disk> {
    let spec = Spec::parse(spec)?;
    if super::is_iso(&spec.source)? {
        return from_iso(&spec);
    }
    let origin = spec.source.display().to_string();
    if spec.disk.is_some() {
        return Err(config(
            &origin,
            "is not an ISO 9660 disc image, so `disk=` has nothing to look inside. Drop it and \
             name the ADF directly."
                .to_owned(),
        ));
    }
    let bytes =
        std::fs::read(&spec.source).map_err(|e| config(&origin, format!("cannot be read: {e}")))?;
    parse(&origin, bytes)
}

fn from_iso(spec: &Spec) -> Result<Disk> {
    use std::io::Read;

    let at = spec.source.display().to_string();
    let mut dev = fstool::block::open_image_read_only(&spec.source)
        .map_err(|e| config(&at, format!("cannot be opened as a disc image: {e}")))?;
    let iso = fstool::fs::iso9660::Iso9660::open(&mut *dev)
        .map_err(|e| config(&at, format!("is not a readable ISO 9660 volume: {e}")))?;
    let volume = iso.volume_id().trim().to_string();
    let entries = iso.list_path(&mut *dev, ISO_ADF_DIR).map_err(|_| {
        config(
            &at,
            format!(
                "is an ISO 9660 volume named `{volume}`, but it has no `{ISO_ADF_DIR}` \
                 directory, so it is not an Amiga Forever disc. That is where Amiga Forever \
                 keeps its ADF images."
            ),
        )
    })?;

    let Some(name) = spec.disk.as_deref() else {
        return Err(config(
            &at,
            format!(
                "is an Amiga Forever disc (`{volume}`); say which disk to take from it with \
                 `,disk=<name>`. It holds {}.",
                disk_list(&entries)
            ),
        ));
    };
    let path = iso_path(name, &entries).ok_or_else(|| {
        config(
            &at,
            format!(
                "holds no disk called `{name}`; it holds {}.",
                disk_list(&entries)
            ),
        )
    })?;
    let origin = format!("{at}:{path}");
    let mut bytes = Vec::new();
    iso.open_file_reader(&mut *dev, &path)
        .and_then(|mut r| r.read_to_end(&mut bytes).map_err(fstool::Error::from))
        .map_err(|e| config(&origin, format!("cannot be read out of the disc: {e}")))?;
    parse(&origin, bytes)
}

/// Resolve `disk=<name>` against the disc's ADF directory.
fn iso_path(name: &str, entries: &[fstool::fs::DirEntry]) -> Option<String> {
    if name.contains('/') {
        return Some(name.to_owned());
    }
    let found = entries
        .iter()
        .find(|e| e.name == name)
        .or_else(|| entries.iter().find(|e| e.name == format!("{name}.adf")))?;
    Some(format!("{ISO_ADF_DIR}/{}", found.name))
}

/// The disk names on the disc, for an error that says what to ask for.
fn disk_list(entries: &[fstool::fs::DirEntry]) -> String {
    let mut names: Vec<&str> = entries
        .iter()
        .map(|e| e.name.as_str())
        .filter(|n| n.ends_with(".adf"))
        .collect();
    names.sort_unstable();
    if names.is_empty() {
        return "no `.adf` files at all".to_owned();
    }
    names.join(", ")
}

fn config(at: &str, message: String) -> Error {
    Error::Config {
        at: at.to_owned(),
        message,
    }
}

#[cfg(test)]
mod tests {
    //! Built images only. The two tests that want the user's files read them in
    //! place behind `RSEMU_AMIGA_ADF_DIR` and `RSEMU_AMIGA_FOREVER_ISO`, and
    //! skip saying so when unset. An ADF from Amiga Forever is Cloanto's to
    //! distribute and is never copied here.

    use super::*;
    use alloc::vec;

    /// A blank ADF with a DOS boot block whose checksum is solved for.
    fn bootable(flags: u8) -> Vec<u8> {
        let mut image = vec![0u8; ADF_BYTES];
        image[..4].copy_from_slice(&[b'D', b'O', b'S', flags]);
        image[8..12].copy_from_slice(&880u32.to_be_bytes());
        image[12..16].copy_from_slice(&0x4e75_0000u32.to_be_bytes());
        let sum = {
            let mut sum = 0u32;
            for chunk in image[..1024].as_chunks::<4>().0 {
                let (n, c) = sum.overflowing_add(u32::from_be_bytes(*chunk));
                sum = n.wrapping_add(u32::from(c));
            }
            sum
        };
        image[4..8].copy_from_slice(&(!sum).to_be_bytes());
        image
    }

    struct Scratch(PathBuf);

    impl Scratch {
        fn new(tag: &str) -> Scratch {
            let dir = std::env::temp_dir().join(format!("rsemu-adf-{tag}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).expect("a scratch directory");
            Scratch(dir)
        }

        fn write(&self, name: &str, bytes: &[u8]) -> PathBuf {
            let path = self.0.join(name);
            std::fs::write(&path, bytes).expect("writes");
            path
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn a_solved_boot_block_verifies_and_one_flipped_bit_does_not() {
        let mut image = bootable(1);
        let disk = parse("t", image.clone()).expect("an ADF");
        assert_eq!(disk.boot, Some((1, true)));
        assert!(
            disk.describe().contains("bootable `DOS\\1`"),
            "{}",
            disk.describe()
        );
        image[600] ^= 0x10;
        assert_eq!(parse("t", image).unwrap().boot, Some((1, false)));
        let blank = parse("t", vec![0; ADF_BYTES]).unwrap();
        assert_eq!(blank.boot, None);
        assert!(blank.describe().contains("data disk"));
    }

    #[test]
    fn a_length_that_is_not_an_adf_says_what_one_is() {
        let Error::Config { message, .. } = parse("t", vec![0; 1000]).unwrap_err() else {
            panic!("a config error");
        };
        assert!(message.contains("901120"), "{message}");
        let Error::Config { message, .. } = parse("t", vec![0; ADF_HD_BYTES]).unwrap_err() else {
            panic!("a config error");
        };
        assert!(message.contains("high-density"), "{message}");
    }

    #[test]
    fn a_file_is_read_in_place_and_disk_equals_needs_a_disc() {
        let scratch = Scratch::new("file");
        let path = scratch.write("wb.adf", &bootable(0));
        let disk = open(&path.display().to_string()).expect("reads");
        assert_eq!(disk.bytes.len(), ADF_BYTES);
        let spec = format!("{},disk=workbench", path.display());
        let Error::Config { message, .. } = open(&spec).unwrap_err() else {
            panic!("a config error");
        };
        assert!(message.contains("ISO 9660"), "{message}");
    }

    #[test]
    fn a_specification_is_a_source_then_options() {
        assert_eq!(
            Spec::parse("/d.iso,disk=wb").unwrap(),
            Spec {
                source: PathBuf::from("/d.iso"),
                disk: Some("wb".to_owned())
            }
        );
        assert!(Spec::parse("").is_err());
        let Error::Config { message, .. } = Spec::parse("/d.iso,rom=x").unwrap_err() else {
            panic!("a config error");
        };
        assert!(message.contains("disk="), "{message}");
    }

    #[test]
    fn the_users_adf_directory_reads() {
        let Ok(dir) = std::env::var("RSEMU_AMIGA_ADF_DIR") else {
            println!(
                "adf: set RSEMU_AMIGA_ADF_DIR to an Amiga Forever `Shared/adf` directory to read \
                 real disks in place. Nothing is copied."
            );
            return;
        };
        let mut read = 0;
        for entry in std::fs::read_dir(&dir).expect("a directory").flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "adf") {
                let disk = open(&path.display().to_string())
                    .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
                println!("adf: {}: {}", path.display(), disk.describe());
                read += 1;
            }
        }
        assert!(read > 0, "no .adf in {dir}");
    }

    #[test]
    fn a_disk_comes_straight_out_of_the_users_disc_image() {
        let Ok(iso) = std::env::var("RSEMU_AMIGA_FOREVER_ISO") else {
            println!(
                "adf: set RSEMU_AMIGA_FOREVER_ISO to an Amiga Forever DVD image to check the ISO \
                 9660 path. The disc is read in place."
            );
            return;
        };
        let Error::Config { message, .. } = open(&iso).unwrap_err() else {
            panic!("a disc with no `disk=` should say which disks it holds");
        };
        assert!(message.contains("disk="), "{message}");
        assert!(message.contains(".adf"), "{message}");

        let spec = format!("{iso},disk=amiga-os-310-workbench");
        let disk = open(&spec).unwrap_or_else(|e| panic!("{spec}: {e}"));
        assert_eq!(disk.bytes.len(), ADF_BYTES);
        assert!(
            disk.boot.is_some_and(|(_, ok)| ok),
            "a Workbench disk boots: {}",
            disk.describe()
        );

        let missing = format!("{iso},disk=not-a-real-disk");
        let Error::Config { message, .. } = open(&missing).unwrap_err() else {
            panic!("a config error");
        };
        assert!(message.contains("not-a-real-disk"), "{message}");
    }
}
