//! Amiga Kickstart ROM images: the header a Commodore ROM carries, the
//! checksum it carries at the other end, Cloanto's keyed `AMIROMTYPE1`
//! wrapper, and the three places a user's own copy actually lives.
//!
//! # Why this is a media decode and not a loader device
//!
//! [`dfuse`](crate::dev::dfuse) is the other half of this argument, and it goes
//! the other way. A DfuSe file is a *list* of pieces, each with an absolute
//! address written inside the file, which is "a shape no media slot can
//! express" — [`Media`](crate::core::props::Media) is `{name, bytes}` and no
//! address travels with it — so the thing that understands DfuSe has to be a
//! device that writes into an [`AddressSpace`](crate::core::space::AddressSpace).
//!
//! A Kickstart is the opposite. Once the wrapper is off it is one flat image of
//! 256 KiB or 512 KiB and **the file carries no addresses at all**. Everything
//! about *where* it goes is a property of the board, not of the container:
//!
//! * Kickstart sits at `$F8_0000` (512 KiB parts) or `$FC_0000` (256 KiB), which
//!   is a decode in Gary, not a field in the file.
//! * It is *also* visible at address 0 out of reset, because the `OVL` bit in
//!   CIA-A's port A comes up set and the address decoder mirrors ROM over Chip
//!   RAM until the first thing Kickstart does is clear it. That mirror is a wire
//!   and a decode — exactly the kind of thing `map` and `core::wire` exist for —
//!   and it is emphatically not something a file format could describe, since
//!   the same bytes appear at two addresses at once.
//!
//! So a loader device would buy nothing and cost plenty: it would have to be
//! realized, snapshotted, and reset, and it would put the mirror's policy inside
//! a *format parser*, where it cannot see the CIA. The container is a container.
//! It decodes to bytes on the host side, the bytes go into a media slot, and an
//! Amiga board maps that slot twice and wires `OVL` to the second mapping —
//! which is a board's job and stays a board's job.
//!
//! The second reason is smaller and just as decisive: the `AMIROMTYPE1` key is a
//! *second file*, supplied by the user, that is not guest-visible in any way. A
//! device would need a media slot for it, so `rom.key` would become part of the
//! machine description of every Amiga. It is not part of the machine. It is part
//! of how this host reads that file.
//!
//! # The image
//!
//! Big-endian throughout — it is a 68000 part. Source: the *Amiga Hardware
//! Reference Manual* (Commodore-Amiga, 3rd ed.) for the address map, the `OVL`
//! overlay and the interrupt-acknowledge read; the layout below was then
//! confirmed field by field against 42 images.
//!
//! ```text
//!   header
//!     0   u16  ROM identification word, high byte $11
//!     2   u16  $4EF9, a 68000 JMP.L — the first instruction the CPU fetches
//!     4   u32  its operand: the absolute entry point
//!     8   u16  }  $0000 / $FFFF depending on vintage; not load-bearing
//!    10   u16  }
//!    12   u16  version   \  $FFFF on 1.0 and 1.1, which predate the convention
//!    14   u16  revision  /  40.68 is Kickstart 3.1 for the A1200
//!
//!   footer, the last 24 bytes
//!    -24  u32  checksum
//!    -20  u32  the size of the image, in bytes
//!    -16  8xu16  $0018..$001F
//! ```
//!
//! Those last eight words are the vector numbers the processor reads back during
//! an interrupt-acknowledge cycle: `$19`..`$1F` are the 68000's autovectors for
//! levels 1 to 7, and the ROM's top sixteen bytes are where the Amiga's decode
//! puts them. 1.x images carry `$0000` in the first slot where 2.x and later
//! carry `$0018`. Nothing here reads them; they are described so that the 24
//! bytes are accounted for rather than mysterious.
//!
//! ## The identification word does not encode the size
//!
//! `$1111` on a 256 KiB image and `$1114` on a 512 KiB one is the *common* case
//! and it is easy to mistake for an encoding. It is not one, and two images in
//! Cloanto's own set say so: Kickstart 36.16 for the A3000 is 512 KiB and
//! carries `$1111`, and the A570 extended ROM is 256 KiB and carries `$1114`.
//! So [`Image`] checks the **high byte only** and takes the size from the footer
//! and from the file. Deriving a size from the low byte would reject two real
//! ROMs to enforce a rule that does not exist.
//!
//! ## The checksum
//!
//! A 32-bit **one's-complement sum with end-around carry** over every big-endian
//! longword in the image, the footer's checksum longword included. A good image
//! sums to `$FFFF_FFFF`; the stored longword is whatever makes that true. This
//! is the same arithmetic as an IP header checksum (RFC 1071) widened to 32
//! bits, which is what makes it cheap in 68000 code: `ADD.L` then `ADDX.L #0`.
//!
//! See [`checksum`]. Verified against every footer-carrying image in the set.
//!
//! ## When there is no footer to check
//!
//! Not everything shipped as a `.rom` is a Kickstart. An A1000 bootstrap ROM, a
//! CDTV extended ROM, an A590 controller ROM and a Picasso IV board ROM all have
//! `$FFFF_FFFF` or a stray value where the size longword should be, and no
//! checksum anywhere. The discriminator that works on all 42 images is
//! **the size longword at `-20` equals the length of the image**: every image
//! where that holds also verifies, and every image where it does not carries no
//! checksum at all. So that is the rule — see [`Checksum`].
//!
//! A checksum that is *present and wrong* is an error, never a warning. That is
//! the whole point of checking: a board that boots a half-downloaded Kickstart
//! does not fail, it misbehaves, and it misbehaves somewhere else.
//!
//! # `AMIROMTYPE1`
//!
//! Cloanto's keyed container, as described in their own published note on the
//! format and in the `ReadMe` that ships beside `rom.key`: eleven bytes of ASCII
//! `AMIROMTYPE1`, then the ROM image XORed byte for byte with `rom.key`, the key
//! repeating from the start whenever it runs out. It is an obfuscation and a
//! licence check, not cryptography, and it is its own inverse.
//!
//! The key belongs to whoever bought it. rsemu never carries one, never caches a
//! decoded image, and never copies either out of where the user put them — it
//! reads them in place, every run. See [`open`].
//!
//! # Where the user's copy lives
//!
//! [`open`] takes the three forms an Amiga Forever installation actually
//! presents, and picks between them by looking at the file rather than at its
//! name:
//!
//! ```text
//!   kickstart:<file>                       a .rom, plain or keyed
//!   kickstart:<file>,key=<file>            with the key somewhere else
//!   kickstart:<dvd.iso>,rom=<name>         out of the DVD image, unextracted
//! ```

use alloc::borrow::ToOwned;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::{Path, PathBuf};

use crate::core::error::{Error, Result};

// ---------------------------------------------------------------------------
// The format
// ---------------------------------------------------------------------------

/// The eleven ASCII bytes a keyed image begins with.
pub const KEYED_MAGIC: &[u8; 11] = b"AMIROMTYPE1";

/// The high byte of the ROM identification word, on every Commodore Kickstart
/// and on AROS.
pub const ID_HIGH: u8 = 0x11;

/// The 68000 `JMP.L` opcode the identification word is followed by.
const JMP_L: u16 = 0x4EF9;

/// How many bytes of footer there are: checksum, size, and the eight vector
/// words.
const FOOTER: usize = 24;

/// What verifying an image's own checksum came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Checksum {
    /// The image carries a footer and it verifies. Holds the stored longword.
    Verified(u32),
    /// The image carries no checksum footer — the size longword at `-20` is not
    /// the length of the image — so there was nothing to verify. Every
    /// Kickstart proper has one; the bootstrap and expansion ROMs that share
    /// the `.rom` extension do not.
    Absent,
}

impl Checksum {
    /// Whether an actual checksum was found and agreed.
    #[must_use]
    pub fn verified(self) -> bool {
        matches!(self, Checksum::Verified(_))
    }
}

/// A decoded ROM image: the plain bytes, and what its own header said about
/// them.
#[derive(Debug, Clone)]
pub struct Image {
    /// Where it came from, for messages. Not a path the guest can see.
    pub origin: String,
    /// The plain image, wrapper removed. This is what goes in the media slot.
    pub bytes: Vec<u8>,
    /// The identification word at offset 0. High byte [`ID_HIGH`]; the low byte
    /// is *not* a size (see the module docs).
    pub id: u16,
    /// The absolute address the `JMP.L` at offset 2 jumps to.
    pub entry: u32,
    /// `version.revision` at offsets 12 and 14, when the image carries them.
    /// `None` for 1.0 and 1.1, which store `$FFFF` there.
    pub version: Option<(u16, u16)>,
    /// Whether the image agreed with its own checksum.
    pub checksum: Checksum,
    /// Whether the file was wrapped in [`KEYED_MAGIC`].
    pub keyed: bool,
}

impl Image {
    /// One line naming what was loaded, for the run's own log.
    #[must_use]
    pub fn describe(&self) -> String {
        let version = match self.version {
            Some((v, r)) => format!("Kickstart {v}.{r}"),
            None => "a Kickstart of no stated version".to_owned(),
        };
        let check = match self.checksum {
            Checksum::Verified(_) => "checksum verified",
            Checksum::Absent => "no checksum footer",
        };
        let wrapper = if self.keyed { "keyed, " } else { "" };
        format!(
            "{version}, {} KiB, {wrapper}{check}",
            self.bytes.len() / 1024
        )
    }
}

/// The one's-complement sum with end-around carry over `bytes`, read as
/// big-endian longwords.
///
/// A whole Kickstart — footer included — sums to `u32::MAX`. A trailing run of
/// fewer than four bytes is not reachable on a real image (every size is a
/// power of two) and is ignored rather than zero-padded, so this never depends
/// on a padding convention nobody documented.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u32 {
    let mut sum: u32 = 0;
    for word in bytes.as_chunks::<4>().0 {
        let value = u32::from_be_bytes(*word);
        // The end-around carry, which is what makes this a one's-complement sum
        // rather than a modular one: the carry out of bit 31 comes back in at
        // bit 0. `overflowing_add` gives both halves in one operation.
        let (next, carried) = sum.overflowing_add(value);
        sum = next.wrapping_add(u32::from(carried));
    }
    sum
}

/// Whether `file` is wrapped in Cloanto's keyed container.
#[must_use]
pub fn is_keyed(file: &[u8]) -> bool {
    file.len() > KEYED_MAGIC.len() && &file[..KEYED_MAGIC.len()] == KEYED_MAGIC
}

/// Strip [`KEYED_MAGIC`] and XOR what follows with `key`, cycling.
///
/// # Errors
///
/// [`Error::Config`] if `file` is not keyed, or if `key` is empty — an empty key
/// would be an identity transform that quietly produced garbage.
pub fn decrypt(origin: &str, file: &[u8], key: &[u8]) -> Result<Vec<u8>> {
    if !is_keyed(file) {
        return Err(config(
            origin,
            "is not an AMIROMTYPE1 image, so there is nothing to decode with a key".to_owned(),
        ));
    }
    if key.is_empty() {
        return Err(config(
            origin,
            "was given an empty rom.key; a key of no bytes cannot decode anything".to_owned(),
        ));
    }
    Ok(file[KEYED_MAGIC.len()..]
        .iter()
        .zip(key.iter().cycle())
        .map(|(b, k)| b ^ k)
        .collect())
}

/// Decode a `.rom` file: unwrap it if it is keyed, then read and check the
/// image.
///
/// `key` is consulted only when the file is actually keyed, so a plain image
/// loads whether or not a key was found.
///
/// # Errors
///
/// [`Error::Config`] when the file is keyed and no key was supplied, when it is
/// too short to be a ROM, when the identification word or the `JMP.L` is wrong,
/// or — the one this function exists for — when the image carries a checksum and
/// disagrees with it.
pub fn decode(origin: &str, file: &[u8], key: Option<&[u8]>) -> Result<Image> {
    let keyed = is_keyed(file);
    let bytes = if keyed {
        let Some(key) = key else {
            return Err(config(
                origin,
                "is an AMIROMTYPE1 image and needs the `rom.key` it was encoded with. Amiga \
                 Forever keeps that file beside the ROMs; point at it with `,key=<file>` if it \
                 is somewhere else."
                    .to_owned(),
            ));
        };
        decrypt(origin, file, key)?
    } else {
        file.to_vec()
    };
    parse(origin, bytes, keyed)
}

/// Read the header and verify the footer of an already-plain image.
///
/// # Errors
///
/// [`Error::Config`] as [`decode`] describes.
pub fn parse(origin: &str, bytes: Vec<u8>, keyed: bool) -> Result<Image> {
    if bytes.len() < FOOTER || !bytes.len().is_multiple_of(4) {
        return Err(config(
            origin,
            format!(
                "is {} bytes, which is not a ROM image: a Kickstart is a whole number of \
                 longwords and at least {FOOTER} bytes long",
                bytes.len()
            ),
        ));
    }
    let id = be16(&bytes, 0);
    if (id >> 8) as u8 != ID_HIGH {
        return Err(config(
            origin,
            format!(
                "starts with the word ${id:04X}, not a ROM identification word (${ID_HIGH:02X}xx). \
                 A keyed image decoded with the wrong `rom.key` looks exactly like this."
            ),
        ));
    }
    let jmp = be16(&bytes, 2);
    if jmp != JMP_L {
        return Err(config(
            origin,
            format!(
                "has ${jmp:04X} where a ROM has the JMP.L (${JMP_L:04X}) the 68000 fetches out \
                 of reset"
            ),
        ));
    }
    let entry = be32(&bytes, 4);
    let version = match (be16(&bytes, 12), be16(&bytes, 14)) {
        (0xFFFF, _) | (_, 0xFFFF) => None,
        (v, r) => Some((v, r)),
    };

    // The footer is present exactly when the size longword agrees with the
    // file. See the module docs: that is the discriminator that works across
    // the whole of Cloanto's set, where the identification word alone does not.
    let stored_size = be32(&bytes, bytes.len() - 20);
    let verdict = if u64::from(stored_size) == bytes.len() as u64 {
        let sum = checksum(&bytes);
        if sum != u32::MAX {
            let stored = be32(&bytes, bytes.len() - FOOTER);
            return Err(config(
                origin,
                format!(
                    "fails its own checksum: the image carries ${stored:08X} and sums to \
                     ${sum:08X} where a good one sums to $FFFFFFFF. The file is damaged, \
                     truncated, or was decoded with the wrong `rom.key`."
                ),
            ));
        }
        Checksum::Verified(be32(&bytes, bytes.len() - FOOTER))
    } else {
        Checksum::Absent
    };

    Ok(Image {
        origin: origin.to_owned(),
        bytes,
        id,
        entry,
        version,
        checksum: verdict,
        keyed,
    })
}

fn be16(bytes: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([bytes[at], bytes[at + 1]])
}

fn be32(bytes: &[u8], at: usize) -> u32 {
    u32::from_be_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

fn config(at: &str, message: String) -> Error {
    Error::Config {
        at: at.to_owned(),
        message,
    }
}

// ---------------------------------------------------------------------------
// Where the user's copy lives
// ---------------------------------------------------------------------------

/// The directory an Amiga Forever DVD keeps its ROMs and `rom.key` in.
///
/// Both the disc and an installed copy use this layout, which is why the same
/// `rom=<name>` works against either.
pub const ISO_ROM_DIR: &str = "/Amiga Files/Shared/rom";

/// The name of the key file, beside the ROMs, on the disc and on disk alike.
pub const KEY_NAME: &str = "rom.key";

/// Everything after `kickstart:` on a media specification.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Spec {
    /// The file to read: a `.rom`, or an ISO 9660 image to find one inside.
    source: PathBuf,
    /// `rom=<name>`: which ROM, when `source` is a disc image.
    rom: Option<String>,
    /// `key=<file>`: where `rom.key` is, when it is not beside the ROM.
    key: Option<PathBuf>,
}

impl Spec {
    fn parse(spec: &str) -> Result<Spec> {
        // The source comes first and options follow it, exactly as `--drive`
        // does — so a comma inside a filename is ambiguous only for a name that
        // also ends in something spelled `,rom=` or `,key=`.
        let mut parts = spec.split(',');
        let source = parts.next().unwrap_or("");
        if source.is_empty() {
            return Err(config(
                "kickstart",
                "needs a file: `kickstart:<rom>`, or `kickstart:<dvd.iso>,rom=<name>`".to_owned(),
            ));
        }
        let mut out = Spec {
            source: PathBuf::from(source),
            rom: None,
            key: None,
        };
        for opt in parts {
            match opt.split_once('=') {
                Some(("rom", name)) => out.rom = Some(name.to_owned()),
                Some(("key", path)) => out.key = Some(PathBuf::from(path)),
                _ => {
                    return Err(config(
                        source,
                        format!("`{opt}` is not a kickstart option; they are `rom=` and `key=`"),
                    ));
                }
            }
        }
        Ok(out)
    }
}

/// Read a Kickstart the way the user's own media holds it.
///
/// `spec` is everything after `kickstart:` on the command line:
///
/// ```text
///   <file>                  a .rom; keyed images are decoded with the
///                           `rom.key` sitting beside them
///   <file>,key=<file>       ... with the key elsewhere
///   <dvd.iso>,rom=<name>    straight out of an Amiga Forever disc image,
///                           nothing extracted
/// ```
///
/// Nothing is copied, cached or written. The user's files are read where they
/// are, on every run.
///
/// # Errors
///
/// [`Error::Config`] for a malformed specification, a file that cannot be read,
/// a disc image that is not an Amiga Forever one, a keyed image with no key, or
/// an image that fails its own checksum.
pub fn open(spec: &str) -> Result<Image> {
    let spec = Spec::parse(spec)?;
    if super::is_iso(&spec.source)? {
        return from_iso(&spec);
    }
    if spec.rom.is_some() {
        return Err(config(
            &spec.source.display().to_string(),
            "is not an ISO 9660 disc image, so `rom=` has nothing to look inside. Drop it and \
             name the ROM file directly."
                .to_owned(),
        ));
    }
    let origin = spec.source.display().to_string();
    let file = read_file(&spec.source)?;
    let key = if is_keyed(&file) {
        Some(find_key(&spec)?)
    } else {
        None
    };
    decode(&origin, &file, key.as_deref())
}

/// The key for a plain-file source: `key=` if it was given, else `rom.key`
/// beside the ROM.
fn find_key(spec: &Spec) -> Result<Vec<u8>> {
    if let Some(path) = &spec.key {
        return read_file(path);
    }
    let beside = spec
        .source
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(KEY_NAME);
    read_file(&beside).map_err(|_| {
        config(
            &spec.source.display().to_string(),
            format!(
                "is an AMIROMTYPE1 image, and there is no `{}` beside it to decode it with. \
                 Amiga Forever keeps `{KEY_NAME}` in the same directory as the ROMs; name it \
                 with `,key=<file>` if yours is elsewhere.",
                beside.display()
            ),
        )
    })
}

fn read_file(path: &Path) -> Result<Vec<u8>> {
    std::fs::read(path)
        .map_err(|e| config(&path.display().to_string(), format!("cannot be read: {e}")))
}

// ---------------------------------------------------------------------------
// Out of the disc image
// ---------------------------------------------------------------------------

/// Pull the ROM — and its key — straight out of an Amiga Forever disc image.
///
/// `fstool`'s ISO 9660 reader does the filesystem; this decides what counts as
/// an Amiga Forever disc and what to say when it is some other ISO.
fn from_iso(spec: &Spec) -> Result<Image> {
    use std::io::Read;

    let at = spec.source.display().to_string();
    let mut dev = fstool::block::open_image_read_only(&spec.source)
        .map_err(|e| config(&at, format!("cannot be opened as a disc image: {e}")))?;
    let iso = fstool::fs::iso9660::Iso9660::open(&mut *dev)
        .map_err(|e| config(&at, format!("is not a readable ISO 9660 volume: {e}")))?;
    let volume = iso.volume_id().trim().to_string();

    // The ROM directory is the authority on whether this is the right disc, and
    // the volume id is the diagnostic. Checking the id alone would reject a
    // future disc that reorganised nothing, and checking neither would report
    // "no such file" for a Debian image.
    let entries = iso.list_path(&mut *dev, ISO_ROM_DIR).map_err(|_| {
        config(
            &at,
            format!(
                "is an ISO 9660 volume named `{volume}`, but it has no `{ISO_ROM_DIR}` \
                 directory, so it is not an Amiga Forever disc. That is where Amiga Forever \
                 keeps its Kickstart images and `{KEY_NAME}`."
            ),
        )
    })?;

    let Some(name) = spec.rom.as_deref() else {
        return Err(config(
            &at,
            format!(
                "is an Amiga Forever disc (`{volume}`); say which ROM to take from it with \
                 `,rom=<name>`. It holds {}.",
                rom_list(&entries)
            ),
        ));
    };

    let path = iso_path(name, &entries).ok_or_else(|| {
        config(
            &at,
            format!(
                "holds no ROM called `{name}`; it holds {}.",
                rom_list(&entries)
            ),
        )
    })?;
    let origin = format!("{at}:{path}");

    let mut file = Vec::new();
    iso.open_file_reader(&mut *dev, &path)
        .and_then(|mut r| r.read_to_end(&mut file).map_err(fstool::Error::from))
        .map_err(|e| config(&origin, format!("cannot be read out of the disc: {e}")))?;

    // The key rides on the same disc, in the same directory. A user who has the
    // ISO has the key, so `key=` is an override here rather than a requirement.
    let key = if is_keyed(&file) {
        Some(match &spec.key {
            Some(path) => read_file(path)?,
            None => {
                let key_path = format!("{ISO_ROM_DIR}/{KEY_NAME}");
                let mut key = Vec::new();
                iso.open_file_reader(&mut *dev, &key_path)
                    .and_then(|mut r| r.read_to_end(&mut key).map_err(fstool::Error::from))
                    .map_err(|e| {
                        config(
                            &at,
                            format!(
                                "holds a keyed `{name}` but no readable `{key_path}` to decode \
                                 it with: {e}"
                            ),
                        )
                    })?;
                key
            }
        })
    } else {
        None
    };

    decode(&origin, &file, key.as_deref())
}

/// Resolve `rom=<name>` against the disc's ROM directory.
///
/// A name with a `/` in it is taken as a whole path into the volume, so an
/// unusual disc is still reachable. Otherwise it is a file in [`ISO_ROM_DIR`],
/// with `.rom` appended when that is what it takes to find it — `rom=aros` and
/// `rom=aros-20250422.rom` should both work.
fn iso_path(name: &str, entries: &[fstool::fs::DirEntry]) -> Option<String> {
    if name.contains('/') {
        return Some(name.to_owned());
    }
    let found = entries
        .iter()
        .find(|e| e.name == name)
        .or_else(|| entries.iter().find(|e| e.name == format!("{name}.rom")))?;
    Some(format!("{ISO_ROM_DIR}/{}", found.name))
}

/// The ROM names on the disc, for an error that tells the user what to ask for.
fn rom_list(entries: &[fstool::fs::DirEntry]) -> String {
    let mut names: Vec<&str> = entries
        .iter()
        .map(|e| e.name.as_str())
        .filter(|n| n.ends_with(".rom"))
        .collect();
    names.sort_unstable();
    if names.is_empty() {
        return "no `.rom` files at all".to_owned();
    }
    // Generous on purpose. This message is reached by a user who does not know
    // the names — that is the whole reason they are here — so truncating to a
    // tidy handful answers the wrong question. The cap exists only so that an
    // ISO with ten thousand `.rom` files cannot fill a terminal.
    const SHOWN: usize = 64;
    let more = names.len().saturating_sub(SHOWN);
    names.truncate(SHOWN);
    let head = names.join(", ");
    if more == 0 {
        head
    } else {
        format!("{head}, and {more} more")
    }
}

#[cfg(test)]
mod tests;
