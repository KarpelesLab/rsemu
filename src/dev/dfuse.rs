//! DfuSe (`.dfu`) firmware images: the container that carries a load address
//! for every piece of a firmware, and the loader device that honours it.
//!
//! A `.bin` is bytes and nothing else, so a machine file has to say where they
//! go. STMicroelectronics' **DfuSe** container — the file `dfu-util -D` and
//! STM32CubeProgrammer both take — says it *inside the file*: a firmware is a
//! list of **elements**, each with its own absolute address, grouped into
//! **targets** (one per alternate setting of the device's DFU interface, which
//! is how a part offers "internal flash" and "option bytes" as separate things
//! to write).
//!
//! That is a shape no media slot can express.
//! [`Media`](crate::core::props::Media) is `{name, bytes}`, `MediaTable` binds
//! one flat blob per slot, and no address travels with it. So the piece that
//! understands the container is a **device**, on the pattern `arm.loader`,
//! `riscv.loader` and `x86.linuxboot` already set: it takes the media slot,
//! parses it, and writes each element into an [`AddressSpace`] at the address
//! the file names. Nothing in the media pipeline, the snapshot format or the
//! CLI had to learn a new type.
//!
//! # The format (ST **UM0391**, the DfuSe file format specification)
//!
//! ```text
//!   prefix, 11 bytes
//!     0   szSignature[5]   "DfuSe"
//!     5   bVersion         0x01
//!     6   DFUImageSize     u32 LE, the whole file bar the 16-byte suffix
//!    10   bTargets         how many target prefixes follow
//!
//!   target prefix, 274 bytes, x bTargets
//!     0   szSignature[6]   "Target"
//!     6   bAlternateSetting
//!     7   bTargetNamed     u32 LE, zero when szTargetName is meaningless
//!    11   szTargetName[255]
//!   266   dwTargetSize     u32 LE, the size of this target's element block
//!   270   dwNbElements     u32 LE
//!
//!   element, x dwNbElements
//!     0   dwElementAddress u32 LE
//!     4   dwElementSize    u32 LE
//!     8   data
//!
//!   suffix, 16 bytes (DFU 1.1 section 6.2, which UM0391 reuses verbatim)
//!     0   bcdDevice        u16 LE
//!     2   idProduct        u16 LE
//!     4   idVendor         u16 LE
//!     6   bcdDFU           u16 LE, 0x011A for a DfuSe file
//!     8   ucDfuSignature   'U' 'F' 'D' (55 46 44)
//!    11   bLength          16
//!    12   dwCRC            u32 LE
//! ```
//!
//! `dwCRC` is a CRC-32 over **every byte of the file except those last four**:
//! reflected, polynomial `0xEDB88320`, initialised to `0xFFFFFFFF` and — the
//! part that catches every first implementation — **not** inverted at the end.
//! It is therefore the ones' complement of the familiar zlib CRC-32:
//! `crc32(b"123456789")` is `0x340BC6D9` here where zlib says `0xCBF43926`.
//!
//! **Provenance.** The layout above is from ST's own document UM0391 and from
//! the USB-IF's DFU 1.1 class specification, both of which describe the file a
//! device is flashed with rather than anybody's implementation of a flasher. No
//! GPL source was read (`CLAUDE.md`, `ROADMAP.md` §1); `dfu-util` in particular
//! is GPL-2.0 and is off limits as source however convenient its output is as a
//! black-box fixture.
//!
//! # What is validated, and why all of it
//!
//! A `.dfu` is untrusted input — it arrives from a vendor's download page — and
//! this is the shape of parser a fuzzer finds bugs in, so nothing is taken on
//! trust:
//!
//! * the prefix signature and `bVersion`;
//! * `DFUImageSize` against the real length, so a truncated download is named
//!   as truncated rather than loading a short first element and stopping;
//! * every target and element header against the bytes that are actually there,
//!   with **the offset it ran out at** in the message;
//! * `dwNbElements` against the room its target's block has, so a header
//!   claiming four billion elements does not reserve four billion descriptors
//!   before the first bounds check refuses it;
//! * `dwTargetSize` against the element block it describes;
//! * the suffix's `ucDfuSignature`, `bLength` and `bcdDFU`;
//! * `dwCRC`, unless `verify-crc = false` says to load a file with a broken
//!   suffix anyway;
//! * that no two elements of the selected target overlap, and that none wraps
//!   past the end of the address space;
//! * that something is actually mapped where each element lands, which the bus
//!   would not have told us — a space whose unassigned policy is anything but
//!   `Fault` accepts a write into a hole and drops it.
//!
//! Every failure is an [`Error::Config`] naming a number. None of them is a
//! panic, a truncation, or a silently short load.
//!
//! # When the elements are written
//!
//! At **bind**, so a firmware aimed at an address the board does not answer
//! fails the build rather than producing a machine that runs zeroes; and again
//! on a **cold** reset, which is the power cycle that puts the part back to how
//! it arrived. Not on a warm reset: a DfuSe image is usually flash, an STM32
//! firmware that reprograms itself and then asks the NVIC for `SYSRESETREQ` is
//! doing an ordinary thing, and reverting its work there would be a bug that
//! looked like the guest's. `arm.loader` reloads on every reset because its
//! image is a kernel in volatile DRAM; this one cannot assume that.
//!
//! The writes carry [`MemAttrs::debug`], which is not a hedge: an element that
//! lands in `st.flash` *is* a programmer driving the flash interface before the
//! core ever runs, and that device documents its debug write as exactly "the
//! loader's door" — a direct poke that moves no status bit and starts no
//! operation. A guest-attributed store there would be refused by a locked flash
//! controller and quietly do nothing, which is the failure this file exists to
//! avoid.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RequesterId, SpaceView};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "dfu.loader";

/// The five bytes a DfuSe file starts with.
pub const MAGIC: &[u8; 5] = b"DfuSe";

/// The six bytes a target prefix starts with.
pub const TARGET_MAGIC: &[u8; 6] = b"Target";

/// How long the file prefix is.
pub const PREFIX_LEN: u64 = 11;

/// How long one target prefix is.
pub const TARGET_PREFIX_LEN: u64 = 274;

/// How long one element's header is: address and size.
pub const ELEMENT_HEADER_LEN: u64 = 8;

/// How long the DFU suffix is, and what its `bLength` must say.
pub const SUFFIX_LEN: u64 = 16;

/// The only `bVersion` UM0391 defines.
pub const VERSION: u8 = 0x01;

/// The `bcdDFU` a DfuSe file carries: DFU 1.1 with ST's extension.
pub const BCD_DFU: u16 = 0x011a;

/// `ucDfuSignature`, in the order the bytes appear in the file.
pub const SUFFIX_SIGNATURE: [u8; 3] = *b"UFD";

// ---------------------------------------------------------------------------
// The CRC
// ---------------------------------------------------------------------------

/// The reflected CRC-32 table for polynomial `0xEDB88320`.
const fn crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut c = i as u32;
        let mut bit = 0;
        while bit < 8 {
            c = if c & 1 != 0 {
                0xedb8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

/// Built once at compile time: 1 KiB of `.rodata` against eight shifts a byte.
static CRC_TABLE: [u32; 256] = crc_table();

/// The CRC a DFU suffix carries over `bytes`.
///
/// Reflected, polynomial `0xEDB88320`, initialised to `0xFFFFFFFF`, and **not**
/// complemented at the end — see the module documentation, because that last
/// clause is the whole difference from every other CRC-32 in circulation.
#[must_use]
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut c: u32 = 0xffff_ffff;
    for &b in bytes {
        c = CRC_TABLE[((c ^ u32::from(b)) & 0xff) as usize] ^ (c >> 8);
    }
    c
}

// ---------------------------------------------------------------------------
// The parsed file
// ---------------------------------------------------------------------------

/// One element: a run of bytes and the guest address it belongs at.
///
/// The data is kept as an `(offset, len)` pair into the file rather than as a
/// copy, so parsing a 1 MiB firmware allocates nothing but the descriptors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Element {
    /// Where the first byte goes in the guest's address space.
    pub addr: u64,
    /// Where the data starts in the file.
    pub offset: u64,
    /// How many bytes there are.
    pub len: u64,
}

impl Element {
    /// One past the last guest address this element writes.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the element runs off the top of the address space,
    /// which a `u32` address and a `u32` size cannot actually manage but which
    /// a hostile file is entitled to try.
    pub fn end(&self) -> Result<u64> {
        self.addr.checked_add(self.len).ok_or_else(|| {
            bad(format!(
                "an element of {} byte(s) at {:#x} runs off the end of the address space",
                self.len, self.addr
            ))
        })
    }

    /// The element's bytes, out of the file it was parsed from.
    ///
    /// Empty for an [`Element`] handed a file it did not come from: a caller
    /// error, caught here rather than read out of bounds.
    #[must_use]
    pub fn data<'a>(&self, file: &'a [u8]) -> &'a [u8] {
        let start = self.offset as usize;
        let end = start.saturating_add(self.len as usize);
        file.get(start..end).unwrap_or(&[])
    }
}

/// One target: an alternate setting of the device's DFU interface, its name,
/// and the elements written through it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Target {
    /// `bAlternateSetting` — the number `dfu-util -a` takes.
    pub alt: u8,
    /// `szTargetName`, when `bTargetNamed` said it means something.
    pub name: Option<String>,
    /// The elements, in file order.
    pub elements: Vec<Element>,
}

/// The DFU suffix: what device the file was built for, and its CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Suffix {
    /// `bcdDevice`, or `0xFFFF` for "any".
    pub bcd_device: u16,
    /// `idProduct`, or `0xFFFF` for "any".
    pub id_product: u16,
    /// `idVendor`, or `0xFFFF` for "any".
    pub id_vendor: u16,
    /// `bcdDFU`, which is [`BCD_DFU`] in a well-formed file.
    pub bcd_dfu: u16,
    /// `dwCRC`, as the file states it.
    pub crc: u32,
}

/// A whole parsed DfuSe file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    /// The targets, in file order.
    pub targets: Vec<Target>,
    /// The suffix.
    pub suffix: Suffix,
}

/// An [`Error::Config`] attributed to this class.
fn bad(message: String) -> Error {
    Error::Config {
        at: CLASS_NAME.to_string(),
        message,
    }
}

/// The message a short file gets: what was being read, where, and how much
/// there was. Every truncation failure goes through here so they all name the
/// offset they ran out at.
fn short(what: &str, at: u64, want: u64, have: u64) -> Error {
    bad(format!(
        "truncated DfuSe file: {what} needs {want} byte(s) at offset {at:#x} and the file is \
         {have} byte(s) long"
    ))
}

/// Check that `len` bytes exist at `at`, then hand them over.
fn take<'a>(file: &'a [u8], at: u64, len: u64, what: &str) -> Result<&'a [u8]> {
    let have = file.len() as u64;
    match at.checked_add(len) {
        // `at` and `end` are both at most `have`, which came from a `usize`, so
        // both casts back are exact on every host this builds for.
        Some(end) if end <= have => Ok(&file[at as usize..end as usize]),
        _ => Err(short(what, at, len, have)),
    }
}

/// A little-endian `u32` at `at` within an already-bounded field.
fn le32(field: &[u8], at: usize) -> u64 {
    u64::from(u32::from_le_bytes([
        field[at],
        field[at + 1],
        field[at + 2],
        field[at + 3],
    ]))
}

/// A little-endian `u16` at `at` within an already-bounded field.
fn le16(field: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([field[at], field[at + 1]])
}

impl Image {
    /// Whether `file` even claims to be a DfuSe container.
    ///
    /// The prefix signature and a plausible length, and nothing else: a cheap
    /// sniff for a caller choosing between formats, not a substitute for
    /// [`parse`](Image::parse).
    #[must_use]
    pub fn looks_like(file: &[u8]) -> bool {
        file.len() as u64 >= PREFIX_LEN + SUFFIX_LEN && file.starts_with(MAGIC)
    }

    /// Parse `file`.
    ///
    /// `verify_crc` checks `dwCRC`; passing `false` accepts a file whose suffix
    /// is wrong, which is what `verify-crc = false` on the device is for. Every
    /// other check is unconditional — a bad CRC is a damaged download, whereas
    /// a target prefix pointing past the end of the file is not something a
    /// caller can usefully opt out of.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming the offset for a truncated file, the field for
    /// a wrong signature or version, and both numbers for a size that does not
    /// agree with the bytes.
    pub fn parse(file: &[u8], verify_crc: bool) -> Result<Image> {
        let have = file.len() as u64;
        if have < PREFIX_LEN + SUFFIX_LEN {
            return Err(short(
                "a DfuSe file's prefix and suffix",
                0,
                PREFIX_LEN + SUFFIX_LEN,
                have,
            ));
        }
        let prefix = take(file, 0, PREFIX_LEN, "the prefix")?;
        if &prefix[..5] != MAGIC {
            return Err(bad(format!(
                "this is not a DfuSe file: it starts with {:02x?} and a DfuSe file starts with \
                 \"DfuSe\" ({MAGIC:02x?})",
                &prefix[..5]
            )));
        }
        if prefix[5] != VERSION {
            return Err(bad(format!(
                "DfuSe prefix bVersion is {} and UM0391 defines only {VERSION}",
                prefix[5]
            )));
        }
        let image_size = le32(prefix, 6);
        let body_end = have - SUFFIX_LEN;
        if image_size != body_end {
            return Err(bad(format!(
                "DFUImageSize says {image_size} byte(s) before the suffix and the file has \
                 {body_end} ({have} less the {SUFFIX_LEN}-byte suffix): the download is \
                 truncated, or the prefix was rewritten"
            )));
        }
        let targets_count = prefix[10];

        // The suffix before the targets, because it is the cheapest way to find
        // out that the file is damaged at all.
        let suffix = Self::parse_suffix(file, body_end, verify_crc)?;

        let mut at = PREFIX_LEN;
        let mut targets = Vec::with_capacity(targets_count as usize);
        for index in 0..targets_count {
            let (target, next) = Self::parse_target(file, at, body_end, index)?;
            targets.push(target);
            at = next;
        }
        if at != body_end {
            return Err(bad(format!(
                "{targets_count} target(s) account for the file up to offset {at:#x} and the \
                 suffix starts at {body_end:#x}: {} byte(s) between them belong to nothing",
                body_end - at
            )));
        }
        Ok(Image { targets, suffix })
    }

    /// The 16 bytes at `at`, and the CRC over everything before `at + 12`.
    fn parse_suffix(file: &[u8], at: u64, verify_crc: bool) -> Result<Suffix> {
        let s = take(file, at, SUFFIX_LEN, "the DFU suffix")?;
        if s[8..11] != SUFFIX_SIGNATURE {
            return Err(bad(format!(
                "the DFU suffix signature is {:02x?} and it has to be {SUFFIX_SIGNATURE:02x?} \
                 (\"UFD\"): this file has no DFU suffix, or whatever wrote it put those three \
                 bytes down the other way round",
                &s[8..11]
            )));
        }
        if u64::from(s[11]) != SUFFIX_LEN {
            return Err(bad(format!(
                "the DFU suffix declares bLength = {} and this loader reads the \
                 {SUFFIX_LEN}-byte suffix of DFU 1.1; a longer one would put the CRC somewhere \
                 else",
                s[11]
            )));
        }
        let bcd_dfu = le16(s, 6);
        if bcd_dfu != BCD_DFU {
            return Err(bad(format!(
                "bcdDFU is {bcd_dfu:#06x} and a DfuSe file has {BCD_DFU:#06x}: this is a plain \
                 DFU 1.0/1.1 binary with a suffix, which carries no load address at all and has \
                 to be loaded as a raw image at an address the machine file states"
            )));
        }
        let stated = le32(s, 12) as u32;
        if verify_crc {
            let computed = crc32(&file[..(at + 12) as usize]);
            if computed != stated {
                return Err(bad(format!(
                    "the DFU suffix CRC is {stated:#010x} and the file's bytes compute to \
                     {computed:#010x}: the image is damaged. `verify-crc = false` loads it \
                     anyway"
                )));
            }
        }
        Ok(Suffix {
            bcd_device: le16(s, 0),
            id_product: le16(s, 2),
            id_vendor: le16(s, 4),
            bcd_dfu,
            crc: stated,
        })
    }

    /// One target prefix at `at` and its elements; returns where the next
    /// target starts.
    fn parse_target(file: &[u8], at: u64, body_end: u64, index: u8) -> Result<(Target, u64)> {
        let header = take(file, at, TARGET_PREFIX_LEN, "a target prefix")?;
        if &header[..6] != TARGET_MAGIC {
            return Err(bad(format!(
                "target {index} at offset {at:#x} starts with {:02x?} and a target prefix starts \
                 with \"Target\"",
                &header[..6]
            )));
        }
        let alt = header[6];
        let named = le32(header, 7) != 0;
        let name = named.then(|| {
            let raw = &header[11..266];
            let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
            String::from_utf8_lossy(&raw[..end]).into_owned()
        });
        let target_size = le32(header, 266);
        let count = le32(header, 270);

        let block = at + TARGET_PREFIX_LEN;
        if block + target_size > body_end {
            return Err(bad(format!(
                "target {index} (alt {alt}) says its elements are {target_size} byte(s) from \
                 offset {block:#x}, which ends past the suffix at {body_end:#x}"
            )));
        }
        // `dwNbElements` is a `u32` out of an untrusted file. Each element costs
        // at least its header, so that is the real ceiling and reserving for
        // anything more would be allocating on a liar's say-so.
        let ceiling = target_size / ELEMENT_HEADER_LEN;
        if count > ceiling {
            return Err(bad(format!(
                "target {index} (alt {alt}) declares {count} element(s) and its {target_size} \
                 byte(s) have room for at most {ceiling}"
            )));
        }
        let mut elements = Vec::with_capacity(count as usize);
        let mut cursor = block;
        for n in 0..count {
            let head = take(file, cursor, ELEMENT_HEADER_LEN, "an element header")?;
            let addr = le32(head, 0);
            let len = le32(head, 4);
            let data = cursor + ELEMENT_HEADER_LEN;
            if data + len > block + target_size {
                return Err(bad(format!(
                    "element {n} of target {index} (alt {alt}) is {len} byte(s) at file offset \
                     {data:#x} and its target's block ends at {:#x}",
                    block + target_size
                )));
            }
            elements.push(Element {
                addr,
                offset: data,
                len,
            });
            cursor = data + len;
        }
        if cursor != block + target_size {
            return Err(bad(format!(
                "target {index} (alt {alt}) declares dwTargetSize = {target_size} and its \
                 {count} element(s) account for {}",
                cursor - block
            )));
        }
        Ok((
            Target {
                alt,
                name,
                elements,
            },
            cursor,
        ))
    }

    /// The target `alt` and/or `name` select.
    ///
    /// With neither given, a file carrying exactly one target yields it; a file
    /// carrying several is a configuration error that lists them, because
    /// taking the first would silently write option bytes as though they were
    /// code on any part whose alt 0 is not what the machine wanted.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming every target the file does have.
    pub fn select(&self, alt: Option<u8>, name: Option<&str>) -> Result<&Target> {
        let matching = |t: &&Target| {
            alt.is_none_or(|a| t.alt == a)
                && name.is_none_or(|n| t.name.as_deref().is_some_and(|have| have == n))
        };
        let mut found = self.targets.iter().filter(matching);
        let first = found.next();
        match (first, found.next()) {
            (Some(t), None) => Ok(t),
            (Some(_), Some(_)) => Err(bad(format!(
                "more than one target matches (alt = {alt:?}, target = {name:?}); this file has \
                 {}",
                self.describe_targets()
            ))),
            (None, _) if alt.is_none() && name.is_none() => Err(bad(format!(
                "this DfuSe file carries {} target(s) and the machine file did not say which to \
                 load: add `alt = N` or `target = \"...\"`. It has {}",
                self.targets.len(),
                self.describe_targets()
            ))),
            (None, _) => Err(bad(format!(
                "no target matches (alt = {alt:?}, target = {name:?}); this file has {}",
                self.describe_targets()
            ))),
        }
    }

    /// `alt 0 "Internal Flash", alt 1 "Option Bytes"`, for an error message.
    fn describe_targets(&self) -> String {
        if self.targets.is_empty() {
            return String::from("no targets at all");
        }
        let mut out = String::new();
        for (i, t) in self.targets.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            match &t.name {
                Some(n) => out.push_str(&format!("alt {} {n:?}", t.alt)),
                None => out.push_str(&format!("alt {} (unnamed)", t.alt)),
            }
        }
        out
    }
}

/// Refuse a target whose elements write the same address twice.
///
/// # Errors
///
/// [`Error::Config`] naming both elements and what they collide over.
fn check_disjoint(elements: &[Element]) -> Result<()> {
    // Quadratic, over a list that is single digits in every real file and is
    // bounded by `dwTargetSize / 8` in a hostile one. Sorting would need an
    // index permutation to keep the file-order numbering the message uses.
    for (i, a) in elements.iter().enumerate() {
        let a_end = a.end()?;
        for (j, b) in elements.iter().enumerate().skip(i + 1) {
            let b_end = b.end()?;
            if a.addr < b_end && b.addr < a_end {
                return Err(bad(format!(
                    "elements {i} ({} byte(s) at {:#x}) and {j} ({} byte(s) at {:#x}) overlap; a \
                     DfuSe file that writes one address twice does not say which value wins",
                    a.len, a.addr, b.len, b.addr
                )));
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// What the loader learned when the machine bound it.
#[derive(Debug, Default)]
struct Binding {
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// What the last load failed with, if it did. `reset` cannot return an
    /// error, so it is kept here and surfaced by [`Loader::last_error`].
    error: Option<String>,
    /// How many times the elements have been written in, for tests.
    loads: u64,
}

/// A DfuSe image, and the elements it puts into memory.
#[derive(Debug)]
pub struct Loader {
    file: Arc<[u8]>,
    /// The selected target's elements, in file order. Empty for an unbound slot.
    elements: Vec<Element>,
    alt: u8,
    target: Option<String>,
    suffix: Option<Suffix>,
    space_name: Option<String>,
    binding: Mutex<Binding>,
}

/// How much is written per bus transaction, so a failure names an address near
/// the one that failed rather than the start of a megabyte.
const CHUNK: usize = 4096;

impl Loader {
    /// Validate `props`, parse the image and pick the target.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `image` media slot is missing or a property
    /// this class does not know was given, and [`Error::Config`] for anything
    /// wrong with the file: see [`Image::parse`] and [`Image::select`].
    pub fn new(props: &Props) -> Result<Loader> {
        let mut r = props.reader();
        let file = r.require_media("image")?.to_bytes();
        let space_name = r.optional_str("space")?.map(ToString::to_string);
        let alt = r.optional::<u8>("alt")?;
        let target = r.optional_str("target")?.map(ToString::to_string);
        let verify_crc = r.or::<bool>("verify-crc", true)?;
        r.finish()?;

        // An unbound slot is empty bytes, and that is not an error: it is what
        // lets one machine file carry a firmware slot a bare-metal run leaves
        // empty. `arm.loader` makes the same allowance for the same reason.
        if file.is_empty() {
            return Ok(Loader {
                file,
                elements: Vec::new(),
                alt: alt.unwrap_or(0),
                target,
                suffix: None,
                space_name,
                binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
            });
        }
        Self::assemble(file, alt, target.as_deref(), verify_crc, space_name)
    }

    /// Build one from a file directly, for a test or an embedder.
    ///
    /// # Errors
    ///
    /// As [`Loader::new`], less the property errors.
    pub fn from_file(
        file: impl Into<Arc<[u8]>>,
        alt: Option<u8>,
        target: Option<&str>,
        verify_crc: bool,
    ) -> Result<Loader> {
        Self::assemble(file.into(), alt, target, verify_crc, None)
    }

    /// Parse, select and check; the half [`Loader::new`] and
    /// [`Loader::from_file`] share.
    fn assemble(
        file: Arc<[u8]>,
        alt: Option<u8>,
        target: Option<&str>,
        verify_crc: bool,
        space_name: Option<String>,
    ) -> Result<Loader> {
        let image = Image::parse(&file, verify_crc)?;
        let chosen = image.select(alt, target)?;
        check_disjoint(&chosen.elements)?;
        Ok(Loader {
            elements: chosen.elements.clone(),
            alt: chosen.alt,
            target: chosen.name.clone(),
            suffix: Some(image.suffix),
            file,
            space_name,
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        })
    }

    /// The elements that will be written, in file order.
    #[must_use]
    pub fn elements(&self) -> &[Element] {
        &self.elements
    }

    /// The bytes of `element`, which must be one of [`Loader::elements`].
    #[must_use]
    pub fn data(&self, element: &Element) -> &[u8] {
        element.data(&self.file)
    }

    /// Which alternate setting was selected.
    #[must_use]
    pub fn alt(&self) -> u8 {
        self.alt
    }

    /// The selected target's name, if the file named it.
    #[must_use]
    pub fn target(&self) -> Option<&str> {
        self.target.as_deref()
    }

    /// The file's DFU suffix, for diagnostics: which part it was built for.
    #[must_use]
    pub fn suffix(&self) -> Option<Suffix> {
        self.suffix
    }

    /// One line naming the device the file was built for and what it writes.
    ///
    /// `idVendor`/`idProduct` are the fields that say *this firmware is not for
    /// this board*, and nothing in the machine layer can check that for us — a
    /// `.machine` file carries no USB IDs — so they are reported rather than
    /// enforced, here and in every placement error.
    #[must_use]
    pub fn summary(&self) -> String {
        let Some(s) = self.suffix else {
            return String::from("no image bound");
        };
        let bytes: u64 = self.elements.iter().map(|e| e.len).sum();
        format!(
            "DfuSe {:04x}:{:04x} rev {:04x}, alt {} {}, {} element(s), {bytes} byte(s)",
            s.id_vendor,
            s.id_product,
            s.bcd_device,
            self.alt,
            match &self.target {
                Some(n) => format!("{n:?}"),
                None => String::from("(unnamed)"),
            },
            self.elements.len(),
        )
    }

    /// What the last load failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.binding.lock().error.clone()
    }

    /// How many times the elements have been written into memory.
    #[must_use]
    pub fn loads(&self) -> u64 {
        self.binding.lock().loads
    }

    /// Whether there is nothing to write.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Write every element into `space` now.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming the element and the address, for an element the
    /// space has no memory at — which is the ordinary way a firmware built for
    /// a different part fails here.
    pub fn load_into(&self, space: &AddressSpace, requester: RequesterId) -> Result<()> {
        if self.elements.is_empty() {
            return Ok(());
        }
        // One view for the whole load rather than one per chunk: the topology
        // cannot change underneath a loader that holds it, and `locate` needs a
        // view anyway.
        let view = space.try_view().ok_or_else(|| {
            bad(String::from(
                "the address space's topology is being changed; a loader runs at bind and at \
                 reset, when nothing else should hold it",
            ))
        })?;
        // See the module docs: this is the programmer's door, not the guest's.
        let attrs = MemAttrs::DEBUG.with_requester(requester);
        for (n, element) in self.elements.iter().enumerate() {
            let mut at = element.addr;
            // `end` is checked here as well as in `check_disjoint` so that
            // `load_into` is sound on a `Loader` built any other way.
            let end = element.end()?;
            for piece in self.data(element).chunks(CHUNK) {
                self.ensure_mapped(&view, n, element, at, space)?;
                Self::write_span(&view, at, piece, attrs)
                    .map_err(|e| self.placement_error(n, element, at, space, &e))?;
                at += piece.len() as u64;
            }
            // The last byte too: a chunk-aligned probe alone would miss a
            // region that ends part way through the final chunk.
            if end > element.addr {
                self.ensure_mapped(&view, n, element, end - 1, space)?;
            }
        }
        Ok(())
    }

    /// Write one chunk, falling back to single legal-width stores when the
    /// region refuses a burst.
    ///
    /// An image region is not always memory. `st.flash`'s programming window is
    /// [`AccessConstraints::IO`](crate::core::space::AccessConstraints::IO) —
    /// any width up to a double word, but **no bursts**, because a block copy
    /// into flash is not a transfer the part can perform and the dispatcher
    /// refuses it rather than hand the controller a 4 KiB "register write". So
    /// a burst that comes back [`BusError::BadAccess`] is retried as the
    /// naturally-aligned 8-, 4-, 2- and 1-byte stores the region does accept,
    /// which is how a programmer writes flash anyway.
    ///
    /// The retry may rewrite bytes a partially-completed burst already put
    /// down, in the one case where a chunk straddles a region boundary and the
    /// far side refuses. That is harmless: the same bytes go to the same
    /// addresses, and every region an image lands in takes an idempotent write.
    fn write_span(view: &SpaceView<'_>, at: u64, bytes: &[u8], attrs: MemAttrs) -> MemResult {
        match view.write_bytes(at, bytes, attrs) {
            Err(BusError::BadAccess) => {}
            other => return other,
        }
        let total = bytes.len() as u64;
        let mut off = 0;
        while off < total {
            let a = at + off;
            // The widest legal width that both fits and lands naturally
            // aligned; `AccessConstraints::check_bulk` accepts exactly those.
            let mut n = 8;
            while n > 1 && (n > total - off || !a.is_multiple_of(n)) {
                n /= 2;
            }
            view.write_bytes(a, &bytes[off as usize..(off + n) as usize], attrs)?;
            off += n;
        }
        Ok(())
    }

    /// Refuse an element the space has nothing mapped at.
    ///
    /// The bus cannot answer this for us: an [`AddressSpace`] whose unassigned
    /// policy is anything but `Fault` — which is most boards, because open bus
    /// is what real hardware does — takes a write into a hole and drops it. A
    /// firmware loaded into a hole would then run as zeroes with no complaint
    /// anywhere, which is precisely the outcome this device exists to prevent.
    fn ensure_mapped(
        &self,
        view: &SpaceView<'_>,
        n: usize,
        element: &Element,
        at: u64,
        space: &AddressSpace,
    ) -> Result<()> {
        if view.locate(at).is_some() {
            return Ok(());
        }
        Err(bad(format!(
            "element {n} of this DfuSe image is {} byte(s) at {:#x} and nothing is mapped at \
             {at:#x} in space `{}`. Either the board has no writable image region there, or the \
             firmware was built for a different part — the file says {}",
            element.len,
            element.addr,
            space.name(),
            self.summary()
        )))
    }

    /// The message for an element a mapped region nonetheless refused.
    fn placement_error(
        &self,
        n: usize,
        element: &Element,
        at: u64,
        space: &AddressSpace,
        cause: &impl core::fmt::Display,
    ) -> Error {
        bad(format!(
            "element {n} of this DfuSe image is {} byte(s) at {:#x} and space `{}` refused the \
             write at {at:#x}: {cause}. The file says {}",
            element.len,
            element.addr,
            space.name(),
            self.summary()
        ))
    }
}

/// The `dfu.loader` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "writes a DfuSe (.dfu, UM0391) image into guest memory, each element at the address \
              the file gives it",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: true,
            summary: "the .dfu file, as the name of a media slot (`image = \"firmware\"`)",
        },
        PropertySpec {
            name: "space",
            kind: ValueKind::Str,
            required: false,
            summary: "which address space to write into, if not the one the object declares",
        },
        PropertySpec {
            name: "alt",
            kind: ValueKind::Uint,
            required: false,
            summary: "which bAlternateSetting to load, for a file with several targets",
        },
        PropertySpec {
            name: "target",
            kind: ValueKind::Str,
            required: false,
            summary: "which target name to load, for a file with several targets",
        },
        PropertySpec {
            name: "verify-crc",
            kind: ValueKind::Bool,
            required: false,
            summary: "check the DFU suffix CRC before loading (default true)",
        },
    ],
    construct: |props| Ok(Box::new(Loader::new(props)?)),
};

impl Device for Loader {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the elements go in at bind and at cold reset. See
        // the module docs for why not on a warm one.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        if kind != ResetKind::Cold {
            return;
        }
        let (space, requester) = {
            let binding = self.binding.lock();
            (binding.space.clone(), binding.requester)
        };
        let Some(space) = space else {
            return;
        };
        let result = self.load_into(&space, requester);
        let mut binding = self.binding.lock();
        binding.loads += 1;
        binding.error = result.err().map(|e| e.to_string());
    }

    // No `save`/`load`: the image is the media the caller bound, and a snapshot
    // that carried it would store the firmware in every save state
    // (`ROADMAP.md` §4.5). What the elements wrote is in the memory they were
    // written into, and whatever owns that memory saves it.
}

impl Instance for Loader {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = match &self.space_name {
            Some(name) => ctx.space_named(name).ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: format!("no address space named `{name}`"),
            })?,
            None => ctx.space().ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: String::from(
                    "a loader needs an address space to write into (`space = mem`)",
                ),
            })?,
        };
        // Fail here rather than at reset, where nothing could report it: a
        // machine whose firmware does not fit should not build at all.
        self.load_into(space, ctx.requester())?;
        let mut binding = self.binding.lock();
        binding.space = Some(Arc::clone(space));
        binding.requester = ctx.requester();
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Loader::new(props)?)))
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// What the validator should know about `dfu.loader`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media).required())
        .prop(PropSchema::new("space", ValueKind::Str))
        .prop(PropSchema::new("alt", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("target", ValueKind::Str))
        .prop(PropSchema::new("verify-crc", ValueKind::Bool))
}

// ---------------------------------------------------------------------------
// Building a file, which the tests and the fuzz target both need
// ---------------------------------------------------------------------------

/// One target as [`build`] takes it: its alternate setting, its name if it has
/// one, and `(address, bytes)` for each of its elements.
pub type TargetSpec<'a> = (u8, Option<&'a str>, &'a [(u32, &'a [u8])]);

/// Emit a DfuSe file, for tests and for anything that needs a fixture.
///
/// `CLAUDE.md` forbids vendoring a corpus, and a DfuSe file is trivial to
/// write, so the fixtures are synthesised rather than committed. Public because
/// the fuzz target and the integration tests are separate crates and both need
/// a *valid* file to start mutating from — an entirely random byte string is
/// rejected by the first signature check and never reaches the interesting half
/// of the parser.
///
/// The CRC is computed, so what comes out is valid by construction; a test that
/// wants a broken file breaks one of these afterwards.
#[must_use]
pub fn build(vendor: u16, product: u16, targets: &[TargetSpec<'_>]) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(MAGIC);
    out.push(VERSION);
    out.extend_from_slice(&0u32.to_le_bytes()); // DFUImageSize, patched below
    out.push(targets.len() as u8);

    for (alt, name, elements) in targets {
        let mut block = Vec::new();
        for (addr, data) in *elements {
            block.extend_from_slice(&addr.to_le_bytes());
            block.extend_from_slice(&(data.len() as u32).to_le_bytes());
            block.extend_from_slice(data);
        }
        out.extend_from_slice(TARGET_MAGIC);
        out.push(*alt);
        out.extend_from_slice(&u32::from(name.is_some()).to_le_bytes());
        let mut field = [0u8; 255];
        if let Some(n) = name {
            let bytes = n.as_bytes();
            let take = bytes.len().min(254);
            field[..take].copy_from_slice(&bytes[..take]);
        }
        out.extend_from_slice(&field);
        out.extend_from_slice(&(block.len() as u32).to_le_bytes());
        out.extend_from_slice(&(elements.len() as u32).to_le_bytes());
        out.extend_from_slice(&block);
    }

    let image_size = out.len() as u32;
    out[6..10].copy_from_slice(&image_size.to_le_bytes());

    out.extend_from_slice(&0xffffu16.to_le_bytes()); // bcdDevice
    out.extend_from_slice(&product.to_le_bytes());
    out.extend_from_slice(&vendor.to_le_bytes());
    out.extend_from_slice(&BCD_DFU.to_le_bytes());
    out.extend_from_slice(&SUFFIX_SIGNATURE);
    out.push(SUFFIX_LEN as u8);
    let crc = crc32(&out);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::core::value::Width;

    /// A file with one named target and whatever elements are asked for.
    fn one_target(elements: &[(u32, &[u8])]) -> Vec<u8> {
        build(0x0483, 0xdf11, &[(0, Some("Internal Flash"), elements)])
    }

    fn space_with_ram(base: u64, len: u64) -> Arc<AddressSpace> {
        let space = AddressSpace::new("mem", 32);
        let ram = Arc::new(RamStore::new(len));
        space
            .topology()
            .map(Region::ram("ram", ram), base)
            .expect("a fresh space");
        Arc::new(space)
    }

    fn peek(space: &AddressSpace, addr: u64) -> u8 {
        space
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .expect("mapped") as u8
    }

    #[test]
    fn the_dfu_crc_convention_is_the_uninverted_one() {
        // The check value every CRC-32 implementation states is 0xCBF43926;
        // DFU's suffix omits the final complement, so it is the other one. An
        // implementation that gets this wrong rejects every real file.
        assert_eq!(crc32(b"123456789"), 0x340b_c6d9);
        assert_eq!(crc32(b"123456789"), !0xcbf4_3926u32);
        assert_eq!(crc32(&[]), 0xffff_ffff);
    }

    #[test]
    fn a_built_file_round_trips_through_the_parser() {
        let file = one_target(&[(0x0800_0000, b"AAAA"), (0x0800_8000, b"BBBB")]);
        let image = Image::parse(&file, true).expect("we built it");
        assert_eq!(image.suffix.id_vendor, 0x0483);
        assert_eq!(image.suffix.id_product, 0xdf11);
        assert_eq!(image.suffix.bcd_dfu, BCD_DFU);
        assert_eq!(image.targets.len(), 1);
        let t = &image.targets[0];
        assert_eq!(t.alt, 0);
        assert_eq!(t.name.as_deref(), Some("Internal Flash"));
        assert_eq!(t.elements.len(), 2);
        assert_eq!(t.elements[0].addr, 0x0800_0000);
        assert_eq!(t.elements[0].data(&file), b"AAAA");
        assert_eq!(t.elements[1].addr, 0x0800_8000);
        assert_eq!(t.elements[1].data(&file), b"BBBB");
        assert!(Image::looks_like(&file));
    }

    #[test]
    fn the_elements_land_at_their_stated_addresses_and_not_between_them() {
        let file = one_target(&[(0x0800_0000, b"AAAA"), (0x0800_0010, b"BBBB")]);
        let space = space_with_ram(0x0800_0000, 0x1000);
        // Not zeroed: erased flash is all ones, and a test that cannot tell a
        // gap from a zero byte proves less.
        for a in 0..0x1000 {
            space
                .write(0x0800_0000 + a, Width::U8, 0xff, MemAttrs::DEFAULT)
                .expect("mapped");
        }
        let loader = Loader::from_file(file, None, None, true).expect("a valid file");
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("both elements fit");
        assert_eq!(peek(&space, 0x0800_0000), b'A');
        assert_eq!(peek(&space, 0x0800_0003), b'A');
        assert_eq!(peek(&space, 0x0800_0004), 0xff, "the gap is untouched");
        assert_eq!(peek(&space, 0x0800_000f), 0xff, "and still is here");
        assert_eq!(peek(&space, 0x0800_0010), b'B');
        assert_eq!(peek(&space, 0x0800_0013), b'B');
    }

    #[test]
    fn a_bad_suffix_crc_is_rejected_unless_verification_is_off() {
        let mut file = one_target(&[(0x2000_0000, b"xyz")]);
        let last = file.len() - 1;
        file[last] ^= 0xff;
        let e = Image::parse(&file, true)
            .expect_err("the CRC no longer matches")
            .to_string();
        assert!(e.contains("CRC"), "{e}");
        assert!(e.contains("verify-crc"), "it has to say the way out: {e}");
        Image::parse(&file, false).expect("verification off loads it anyway");
    }

    #[test]
    fn a_bad_signature_or_bcddfu_is_rejected() {
        let good = one_target(&[(0x2000_0000, b"xyz")]);

        let mut file = good.clone();
        file[0] = b'X';
        let e = Image::parse(&file, false)
            .expect_err("not DfuSe")
            .to_string();
        assert!(e.contains("DfuSe"), "{e}");

        let mut file = good.clone();
        file[5] = 2;
        let e = Image::parse(&file, false)
            .expect_err("bVersion")
            .to_string();
        assert!(e.contains("bVersion"), "{e}");

        // ucDfuSignature, eight bytes into the suffix.
        let mut file = good.clone();
        let sig = file.len() - 8;
        file[sig] = b'D';
        let e = Image::parse(&file, false).expect_err("no UFD").to_string();
        assert!(e.contains("UFD"), "{e}");

        // bLength.
        let mut file = good.clone();
        let blen = file.len() - 5;
        file[blen] = 20;
        let e = Image::parse(&file, false).expect_err("bLength").to_string();
        assert!(e.contains("bLength"), "{e}");

        // bcdDFU: a plain DFU 1.1 file with a suffix and no DfuSe body.
        let mut file = good.clone();
        let bcd = file.len() - 10;
        file[bcd..bcd + 2].copy_from_slice(&0x0110u16.to_le_bytes());
        let e = Image::parse(&file, false).expect_err("bcdDFU").to_string();
        assert!(e.contains("bcdDFU"), "{e}");
        assert!(e.contains("raw image"), "it says what to do instead: {e}");

        // The target prefix's own signature.
        let mut file = good.clone();
        file[PREFIX_LEN as usize] = b'X';
        let e = Image::parse(&file, false)
            .expect_err("not Target")
            .to_string();
        assert!(e.contains("Target"), "{e}");
    }

    #[test]
    fn the_selected_alternate_setting_is_the_one_loaded() {
        let file = build(
            0x0483,
            0xdf11,
            &[
                (0, Some("Internal Flash"), &[(0x0800_0000, &b"CODE"[..])]),
                (1, Some("Option Bytes"), &[(0x1fff_c000, &b"OPTS"[..])]),
            ],
        );
        let image = Image::parse(&file, true).expect("valid");

        assert_eq!(image.select(Some(1), None).expect("alt 1").alt, 1);
        assert_eq!(
            image
                .select(None, Some("Internal Flash"))
                .expect("by name")
                .alt,
            0
        );

        // Two targets and no selector is an error that lists them, rather than
        // silently writing option bytes as if they were code.
        let e = image.select(None, None).expect_err("ambiguous").to_string();
        assert!(e.contains("alt = N"), "{e}");
        assert!(
            e.contains("Internal Flash") && e.contains("Option Bytes"),
            "{e}"
        );

        let e = image
            .select(Some(7), None)
            .expect_err("no such")
            .to_string();
        assert!(e.contains("no target matches"), "{e}");

        let loader = Loader::from_file(file, Some(1), None, true).expect("alt 1");
        assert_eq!(loader.elements().len(), 1);
        assert_eq!(loader.elements()[0].addr, 0x1fff_c000);
        assert_eq!(loader.target(), Some("Option Bytes"));
    }

    #[test]
    fn an_element_outside_every_image_region_names_the_address_in_the_error() {
        let file = one_target(&[(0x0800_0000, b"AAAA"), (0x9999_0000, b"BBBB")]);
        // Open bus, which is what a real board does and what makes the write
        // into the hole succeed at the bus level. The loader must not be fooled.
        let space = space_with_ram(0x0800_0000, 0x1000);
        let loader = Loader::from_file(file, None, None, true).expect("valid");
        let e = loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect_err("nothing is mapped at 0x99990000")
            .to_string();
        assert!(e.contains("99990000"), "{e}");
        assert!(e.contains("element 1"), "{e}");
        assert!(e.contains("0483:df11"), "and which part it is for: {e}");
    }

    #[test]
    fn an_element_that_runs_off_the_end_of_its_region_is_refused() {
        // Mapped at the start and not at the end: the chunk-aligned probe alone
        // would miss this, which is why the last byte is probed too.
        let file = one_target(&[(0x0800_0ff0, &[0xaa; 0x20])]);
        let space = space_with_ram(0x0800_0000, 0x1000);
        let loader = Loader::from_file(file, None, None, true).expect("valid");
        let e = loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect_err("it runs past the top of the RAM")
            .to_string();
        assert!(e.contains("element 0"), "{e}");
    }

    #[test]
    fn a_truncated_file_is_rejected_with_the_offset_it_ran_out_at() {
        let file = one_target(&[(0x0800_0000, &[0x5au8; 512][..])]);

        // Short of even a prefix and a suffix.
        let e = Image::parse(&file[..8], false)
            .expect_err("far too short")
            .to_string();
        assert!(e.contains("truncated"), "{e}");

        // Truncated in the middle: DFUImageSize no longer matches, which is the
        // check that catches a half-finished download.
        let e = Image::parse(&file[..file.len() - 64], false)
            .expect_err("short")
            .to_string();
        assert!(e.contains("DFUImageSize"), "{e}");
        assert!(e.contains("truncated"), "{e}");

        // A prefix that promises a target the file does not contain. The
        // DFUImageSize check has to be satisfied for the target walk to be
        // reached at all, so this one is built rather than sliced.
        let mut file = one_target(&[(0x0800_0000, b"AA")]);
        file[10] = 2;
        let e = Image::parse(&file, false)
            .expect_err("one target's worth of bytes, two promised")
            .to_string();
        assert!(e.contains("truncated"), "{e}");
        assert!(e.contains("a target prefix"), "{e}");
    }

    #[test]
    fn a_target_that_lies_about_its_size_is_refused() {
        let mut file = one_target(&[(0x0800_0000, b"AAAA")]);
        // dwTargetSize, 266 bytes into the target prefix.
        let at = PREFIX_LEN as usize + 266;
        file[at..at + 4].copy_from_slice(&0x1000u32.to_le_bytes());
        let e = Image::parse(&file, false).expect_err("too big").to_string();
        assert!(e.contains("past the suffix"), "{e}");

        // And a count that its block has no room for: the allocation guard.
        let mut file = one_target(&[(0x0800_0000, b"AAAA")]);
        let at = PREFIX_LEN as usize + 270;
        file[at..at + 4].copy_from_slice(&0xffff_ffffu32.to_le_bytes());
        let e = Image::parse(&file, false)
            .expect_err("too many")
            .to_string();
        assert!(e.contains("room for at most"), "{e}");
    }

    #[test]
    fn overlapping_elements_are_refused_naming_both() {
        let file = one_target(&[(0x0800_0000, &[0u8; 16][..]), (0x0800_0008, b"XXXX")]);
        // The container itself is well formed; it is the placement that is not.
        Image::parse(&file, true).expect("a valid container");
        let e = Loader::from_file(file, None, None, true)
            .expect_err("they overlap")
            .to_string();
        assert!(e.contains("overlap"), "{e}");
        assert!(e.contains("8000000") && e.contains("8000008"), "{e}");
    }

    #[test]
    fn an_unnamed_target_parses_and_says_so() {
        let file = build(0x0483, 0xdf11, &[(3, None, &[(0x2000_0000, &b"hi"[..])])]);
        let image = Image::parse(&file, true).expect("valid");
        assert_eq!(image.targets[0].name, None);
        assert!(image.describe_targets().contains("alt 3 (unnamed)"));
    }

    #[test]
    fn an_empty_media_slot_loads_nothing_and_is_not_an_error() {
        let loader = Loader::new(
            &Props::new().with("image", crate::core::props::Media::new("firmware", &[][..])),
        )
        .expect("an unbound slot is empty, not wrong");
        assert!(loader.is_empty());
        assert_eq!(loader.summary(), "no image bound");
        let space = space_with_ram(0, 0x100);
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("nothing to do");
    }

    #[test]
    fn a_media_slot_is_required() {
        let e = Loader::new(&Props::new())
            .expect_err("no image")
            .to_string();
        assert!(e.contains("image") && e.contains("media"), "{e}");
    }

    #[test]
    fn a_cold_reset_writes_the_elements_in_again_and_a_warm_one_does_not() {
        // A DfuSe image is usually flash, and firmware that reprograms itself
        // and then asks for SYSRESETREQ must not find its work reverted.
        let file = one_target(&[(0x0800_0000, b"AAAA")]);
        let space = space_with_ram(0x0800_0000, 0x1000);
        let loader = Loader::from_file(file, None, None, true).expect("valid");
        loader.binding.lock().space = Some(Arc::clone(&space));

        space
            .write(0x0800_0000, Width::U8, 0x00, MemAttrs::DEFAULT)
            .expect("mapped");
        loader.reset(ResetKind::Warm);
        assert_eq!(
            peek(&space, 0x0800_0000),
            0x00,
            "a warm reset reloads nothing"
        );
        assert_eq!(loader.loads(), 0);

        loader.reset(ResetKind::Cold);
        assert_eq!(peek(&space, 0x0800_0000), b'A');
        assert_eq!(loader.loads(), 1);
        assert_eq!(loader.last_error(), None);
    }

    #[test]
    fn the_class_declares_every_property_the_reader_asks_for() {
        // The reader refuses an unknown property, and the validator refuses one
        // the schema omits; this catches the two lists drifting apart.
        let schema = schema();
        for spec in CLASS.properties {
            assert!(
                schema.props.iter().any(|p| p.name == spec.name),
                "`{}` is in the class and not in the schema",
                spec.name
            );
        }
        assert_eq!(schema.props.len(), CLASS.properties.len());
    }
}
