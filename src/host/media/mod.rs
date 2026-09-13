//! Where the bytes in a media slot come from.
//!
//! A machine description names a **slot** — `image = "firmware"` — and whoever
//! realizes the machine binds bytes to it. `--media firmware=<file>` is how the
//! command line does that, and for almost everything the answer is "read the
//! file", which is what [`read`] does when the specification is a plain path.
//!
//! Some media do not arrive as a flat file, though, and the difference is worth
//! naming rather than hiding. A user's Kickstart ROM is inside an
//! `AMIROMTYPE1` wrapper that needs a key they own, or inside a 1.7 GB disc
//! image that must not be extracted by hand first. Reaching those is *host*
//! work: opening files, finding a key beside one, walking a filesystem. None of
//! it may happen below `host/`, and none of it changes what a device sees,
//! which is still `{name, bytes}`.
//!
//! So a media specification may carry a **scheme**:
//!
//! ```text
//!   --media firmware=/path/to/fw.bin                      a file
//!   --media firmware=kickstart:/path/to/kick.rom          a decoded Kickstart
//!   --media firmware=kickstart:/path/to/dvd.iso,rom=<name>
//!   --media df0=adf:/path/to/disk.adf                     a checked ADF
//!   --media df0=adf:/path/to/dvd.iso,disk=<name>
//! ```
//!
//! The scheme list is closed and short — anything that is not one of these
//! names is a path, so `C:\roms\fw.bin` is a file and not a scheme called `C`.
//! Adding one is a deliberate act, which is the point: a media source is a
//! place bytes can come from, and there should be few enough of them to read in
//! one sitting.

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::error::{Error, Result};

#[cfg(feature = "media-kickstart")]
#[cfg_attr(docsrs, doc(cfg(feature = "media-kickstart")))]
pub mod kickstart;

#[cfg(feature = "media-adf")]
#[cfg_attr(docsrs, doc(cfg(feature = "media-adf")))]
pub mod adf;

/// The prefix that selects the Kickstart source.
const KICKSTART: &str = "kickstart:";

/// The prefix that selects the ADF source.
const ADF: &str = "adf:";

/// Bytes for a media slot, and anything worth telling the user about them.
#[derive(Debug, Clone)]
pub struct Loaded {
    /// What to bind to the slot.
    pub bytes: Vec<u8>,
    /// One line about what was decoded, when the source has something to say.
    /// `None` for a plain file, which has nothing to report that the path did
    /// not already say.
    pub note: Option<String>,
}

/// Resolve a media specification to bytes.
///
/// A bare path is read as a file, which is what it has always meant. A
/// recognised scheme is handed to its source.
///
/// # Errors
///
/// [`Error::Config`] if the file cannot be read, if a scheme's specification is
/// malformed, or if a source refuses what it found — a Kickstart that fails its
/// own checksum, for instance.
pub fn read(spec: &str) -> Result<Loaded> {
    if let Some(rest) = spec.strip_prefix(KICKSTART) {
        return read_kickstart(rest);
    }
    if let Some(rest) = spec.strip_prefix(ADF) {
        return read_adf(rest);
    }
    let bytes = std::fs::read(spec).map_err(|e| Error::Config {
        at: spec.to_string(),
        message: alloc::format!("cannot be read: {e}"),
    })?;
    Ok(Loaded { bytes, note: None })
}

#[cfg(feature = "media-kickstart")]
fn read_kickstart(rest: &str) -> Result<Loaded> {
    let image = kickstart::open(rest)?;
    let note = Some(image.describe());
    Ok(Loaded {
        bytes: image.bytes,
        note,
    })
}

// Answering "this build cannot do that" beats letting the path fall through to
// `std::fs::read`, which would report that no file is named `kickstart:…` —
// true, unhelpful, and the wrong thing to go and check. Same reasoning as
// `Error::UnknownClass`, which says "missing Cargo feature" rather than
// "unknown".
#[cfg(not(feature = "media-kickstart"))]
fn read_kickstart(rest: &str) -> Result<Loaded> {
    Err(Error::Config {
        at: rest.to_string(),
        message: String::from(
            "this build has no `kickstart` media source; rebuild with the `media-kickstart` \
             feature to read an Amiga Kickstart ROM",
        ),
    })
}

#[cfg(feature = "media-adf")]
fn read_adf(rest: &str) -> Result<Loaded> {
    let disk = adf::open(rest)?;
    let note = Some(disk.describe());
    Ok(Loaded {
        bytes: disk.bytes,
        note,
    })
}

#[cfg(not(feature = "media-adf"))]
fn read_adf(rest: &str) -> Result<Loaded> {
    Err(Error::Config {
        at: rest.to_string(),
        message: String::from(
            "this build has no `adf` media source; rebuild with the `media-adf` feature to read \
             an Amiga disk image this way (a plain path to an .adf works without it)",
        ),
    })
}

/// Where the ISO 9660 standard identifier sits: the volume descriptor set
/// begins at logical sector 16 of 2048 bytes, and `CD001` is at offset 1 of a
/// descriptor (ECMA-119 §6.7.1, §8.1.2).
#[cfg(any(feature = "media-kickstart", feature = "media-adf"))]
const ISO_MAGIC_OFFSET: u64 = 16 * 2048 + 1;

/// Whether `path` carries the ISO 9660 standard identifier.
///
/// A cheap two-syscall probe rather than a format guess from the extension: an
/// Amiga Forever disc image is 1.7 GB and must not be read into memory to find
/// out what it is, and a ROM or an ADF must not be handed to a filesystem
/// reader. Shared by every source that can look inside a disc.
#[cfg(any(feature = "media-kickstart", feature = "media-adf"))]
fn is_iso(path: &std::path::Path) -> Result<bool> {
    use std::io::{Read, Seek, SeekFrom};

    let mut file = std::fs::File::open(path).map_err(|e| Error::Config {
        at: path.display().to_string(),
        message: alloc::format!("cannot be opened: {e}"),
    })?;
    if file.seek(SeekFrom::Start(ISO_MAGIC_OFFSET)).is_err() {
        return Ok(false);
    }
    let mut magic = [0u8; 5];
    match file.read_exact(&mut magic) {
        // Short of 32 KiB is every ROM there is, and no ISO; an ADF is longer,
        // and its sector 16 is not a volume descriptor.
        Ok(()) => Ok(&magic == b"CD001"),
        Err(_) => Ok(false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_path_is_a_file_and_not_a_scheme() {
        // Reading fails, but it fails as a *path*, which is the whole claim:
        // a Windows drive letter must not be mistaken for a media source.
        let e = read("C:\\roms\\fw.bin").unwrap_err();
        let Error::Config { at, message } = e else {
            panic!("expected a config error");
        };
        assert_eq!(at, "C:\\roms\\fw.bin");
        assert!(message.contains("cannot be read"), "{message}");
    }

    #[test]
    fn a_missing_file_names_itself() {
        let e = read("/nonexistent/rsemu/media-test.bin").unwrap_err();
        let Error::Config { at, .. } = e else {
            panic!("expected a config error");
        };
        assert_eq!(at, "/nonexistent/rsemu/media-test.bin");
    }

    #[cfg(not(feature = "media-adf"))]
    #[test]
    fn an_adf_scheme_this_build_lacks_says_which_feature_it_wants() {
        let e = read("adf:/some/disk.adf").unwrap_err();
        let Error::Config { message, .. } = e else {
            panic!("expected a config error");
        };
        assert!(message.contains("media-adf"), "{message}");
    }

    #[cfg(not(feature = "media-kickstart"))]
    #[test]
    fn a_scheme_this_build_lacks_says_which_feature_it_wants() {
        let e = read("kickstart:/some/kick.rom").unwrap_err();
        let Error::Config { message, .. } = e else {
            panic!("expected a config error");
        };
        assert!(message.contains("media-kickstart"), "{message}");
    }
}
