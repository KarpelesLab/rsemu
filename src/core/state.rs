//! Versioned snapshots: the save-state reader and writer (`ROADMAP.md` §4.5).
//!
//! Snapshots are built in phase 1 rather than bolted on later because
//! record/replay, rewind and the whole regression method are defined in terms
//! of them: a machine that cannot be serialised cannot be replayed, and a
//! device with no round-trip test is a device with an undiscovered missing
//! field (`ROADMAP.md` §15, invariant 6).
//!
//! # The format
//!
//! A snapshot is a header followed by one chunk per device instance, keyed by
//! `(instance path, class name, class version)`, followed by an end marker.
//!
//! ```text
//! magic          8 bytes  "RSEMUSNP"
//! format         u32      FORMAT_VERSION
//! codec          u16      0 = stored; the compcol seam (see below)
//! integrity      u16      0 = none;   the purecrypto seam (see below)
//! shape                   the structural fingerprint, see MachineShape
//! chunk*                  tag 0x01, path, class, u32 version, blob payload
//! end                     tag 0x00
//! ```
//!
//! Everything is **little-endian on the wire**, on every host, for every width.
//! That is a format decision, not a host property: a snapshot written by a
//! big-endian host must load on a little-endian one, and guest byte order is a
//! separate matter carried by [`crate::core::value::Endian`] at the device
//! level. Integers are fixed width (no varints): a fixed encoding has exactly
//! one representation for a value, which is what makes byte-identical output
//! provable rather than hoped for.
//!
//! # Determinism
//!
//! The same state must produce byte-identical output every time, because the
//! project's regression method is "hash the state and compare"
//! (`ROADMAP.md` §0). Concretely:
//!
//! - Chunks are emitted sorted by `(instance path, class)`, whatever order the
//!   devices were saved in. [`StateWriter`] buffers and sorts rather than
//!   streaming, so save order is not observable in the output.
//! - Every collection in the header is a `BTreeMap`/`BTreeSet`. No `HashMap`
//!   appears in this module, and none may be introduced.
//! - There is no padding, no alignment, no reserved-but-uninitialised field,
//!   and no timestamp. Nothing in the output comes from anywhere but the state.
//!
//! The reader enforces the same canonical form it writes — ascending chunk
//! keys, ascending header sets, no trailing bytes — so a snapshot has exactly
//! one valid encoding, and "re-encode what you decoded" is byte-identical.
//!
//! # Machine identity is a diff, not a boolean
//!
//! Loading a snapshot into a differently-shaped machine fails with a diff
//! naming what differs ([`ShapeDiff`]), never a crash and never a bare `false`.
//! The identity is a *structural fingerprint* — device classes at instance
//! paths, region layout, feature set, guest architectures — and deliberately
//! **not** a hash of the config text: a text hash gives a boolean where a diff
//! is wanted, and invalidates every snapshot on the planet when someone edits
//! a comment or passes `-p ram=8G` (`ROADMAP.md` §4.5).
//!
//! # Migration
//!
//! A version field with no migration mechanism is decoration. A class registers
//! upgrade functions `vN -> vN+1` in a [`Migrations`] table; loading an older
//! chunk runs the chain from the version on the wire up to the version this
//! build understands. Missing a step is an error that names the gap and the
//! steps that *are* registered. A newer-than-supported chunk is refused —
//! snapshots move forwards only.
//!
//! # Seams left open on purpose
//!
//! - **Compression** (`compcol`, zstd) and **integrity/encryption**
//!   (`purecrypto`, BLAKE3) are out of scope here and would be feature-gated
//!   dependencies. The header carries `codec` and `integrity` fields that this
//!   build writes as zero and refuses to read as anything else, so an old
//!   reader rejects a compressed snapshot with a clear message instead of
//!   parsing garbage. That is the whole seam; see [`Codec`] and [`Integrity`].
//! - **Guest RAM** is the bulk of a real snapshot and wants a page-indexed,
//!   dirty-log-driven incremental encoding. That is a decision for the RAM
//!   device's chunk *contents*, layered on the byte-blob primitive here; this
//!   module deliberately has no opinion about what is inside a chunk.
//! - **The scheduler is architectural state** and is easy to forget: its event
//!   queue, per-domain tick counters, residual accumulators and tie-break
//!   sequence counter are saved as an ordinary chunk under its own instance
//!   path, exactly like a device.
//!
//! # Untrusted input
//!
//! A snapshot is a file a user can hand us, so the reader is a parser on
//! untrusted input: it never panics, never indexes without a bounds check,
//! never trusts a length field it has not compared against the bytes actually
//! remaining, and never allocates proportional to a claimed count. Every
//! failure is [`Error::State`] with a message that says what was expected.
//!
//! # Example
//!
//! ```
//! use rsemu::core::state::{MachineShape, Migrations, Sink, Source, StateReader, StateWriter};
//!
//! let mut shape = MachineShape::new();
//! shape.add_device("/cpu0", "cpu.demo").unwrap();
//!
//! let mut w = StateWriter::new(shape.clone());
//! {
//!     let mut c = w.chunk("/cpu0", "cpu.demo", 1).unwrap();
//!     c.write_u16(0xfffc).unwrap();
//! }
//! let bytes = w.to_vec().unwrap();
//!
//! let r = StateReader::new(&bytes).unwrap();
//! r.check_shape(&shape).unwrap();
//! let chunk = r.load("/cpu0", "cpu.demo", 1, &Migrations::new()).unwrap();
//! let mut cr = chunk.reader();
//! assert_eq!(cr.read_u16().unwrap(), 0xfffc);
//! cr.end().unwrap();
//! ```

use alloc::borrow::Cow;
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};

/// Magic at the start of every snapshot.
const MAGIC: [u8; 8] = *b"RSEMUSNP";

/// The snapshot container format version.
///
/// This is the version of the *container* — the header and chunk framing — and
/// changes only when that framing changes. It is unrelated to a device class
/// version, which versions the bytes *inside* one chunk and is what
/// [`Migrations`] upgrades.
pub const FORMAT_VERSION: u32 = 1;

/// Tag byte introducing one more chunk.
const TAG_CHUNK: u8 = 0x01;

/// Tag byte marking the end of the chunk list.
const TAG_END: u8 = 0x00;

/// The largest diff rendered before the rest is summarised.
///
/// A shape mismatch on a large machine can differ in hundreds of places, and a
/// wall of text is as unhelpful as a boolean.
const MAX_DIFF_LINES: usize = 32;

/// How chunk payloads are encoded.
///
/// The seam for `compcol` (`ROADMAP.md` §4.5, "Layering"). This build writes
/// and accepts [`Codec::STORED`] only; a snapshot naming any other codec is
/// refused with a message saying so, rather than parsed as garbage.
///
/// An extensible-enumeration newtype per `CLAUDE.md` ("Type conventions"),
/// because codecs are added by later phases and exhaustive matching is not
/// wanted.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Codec(pub u16);

impl Codec {
    /// No compression: chunk payloads are stored verbatim.
    pub const STORED: Codec = Codec(0);
    /// Reserved for `compcol`'s zstd encoder. Not implemented in this build.
    pub const ZSTD: Codec = Codec(1);
}

/// What integrity protection covers the snapshot.
///
/// The seam for `purecrypto` (BLAKE3 digests, optional encryption). As with
/// [`Codec`], this build writes and accepts [`Integrity::NONE`] only.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Integrity(pub u16);

impl Integrity {
    /// No digest and no encryption.
    pub const NONE: Integrity = Integrity(0);
    /// Reserved for a BLAKE3 digest over the body. Not implemented here.
    pub const BLAKE3: Integrity = Integrity(1);
}

/// Build an [`Error::State`] from a formatted message.
///
/// Every failure in this module goes through here so the error variant is
/// decided in one place.
fn state_error(message: String) -> Error {
    Error::State(message)
}

/// The error for "the input ended before the value did".
fn truncated(need: usize, have: usize) -> Error {
    state_error(format!(
        "truncated snapshot: needed {need} more byte(s), {have} remain"
    ))
}

/// Convert a wire length to a host `usize`, refusing what this host cannot
/// address.
///
/// Lengths are `u64` on the wire (`CLAUDE.md`: sizes are `u64`, never
/// `usize`), so a 32-bit host reading a 64-bit host's snapshot fails cleanly
/// rather than truncating.
fn wire_len(len: u64) -> Result<usize> {
    usize::try_from(len).map_err(|_| {
        state_error(format!(
            "length {len} does not fit in this host's address space"
        ))
    })
}

// ---------------------------------------------------------------------------
// Sink
// ---------------------------------------------------------------------------

/// Generate the little-endian integer writers as provided trait methods.
macro_rules! sink_int_methods {
    ($($name:ident: $t:ty;)*) => {
        $(
            #[doc = concat!("Write a `", stringify!($t), "` in little-endian order.")]
            fn $name(&mut self, value: $t) -> Result<()> {
                self.write_all(&value.to_le_bytes())
            }
        )*
    };
}

/// Somewhere encoded bytes can be put.
///
/// Deliberately **not** `std::io::Write`: the emulation core is `no_std +
/// alloc` (`ROADMAP.md` §0), so the snapshot writer defines the minimum it
/// needs — one method that appends bytes and cannot partially succeed.
///
/// Implementors provide [`Sink::write_all`] only. Every other method is a
/// provided encoder built on it and **must not be overridden**: the wire format
/// is the format, not a per-sink decision.
pub trait Sink {
    /// Append every byte of `bytes`, or fail.
    ///
    /// There is no short-write case on purpose. A partially written snapshot is
    /// not a state a caller could usefully recover from.
    fn write_all(&mut self, bytes: &[u8]) -> Result<()>;

    sink_int_methods! {
        write_u8: u8;
        write_u16: u16;
        write_u32: u32;
        write_u64: u64;
        write_u128: u128;
        write_i8: i8;
        write_i16: i16;
        write_i32: i32;
        write_i64: i64;
        write_i128: i128;
    }

    /// Write a bool as one canonical byte: `0` or `1`.
    ///
    /// The reader rejects every other value, so `bool` has a single encoding
    /// and a round-trip cannot smuggle a payload through a spare bit.
    fn write_bool(&mut self, value: bool) -> Result<()> {
        self.write_u8(u8::from(value))
    }

    /// Write a length-prefixed byte array: `u64` length, then the bytes.
    fn write_bytes(&mut self, bytes: &[u8]) -> Result<()> {
        self.write_u64(bytes.len() as u64)?;
        self.write_all(bytes)
    }

    /// Write a length-prefixed UTF-8 string.
    ///
    /// The length is in bytes, not characters, and the reader validates UTF-8.
    fn write_str(&mut self, value: &str) -> Result<()> {
        self.write_bytes(value.as_bytes())
    }

    /// Write the element count of a sequence, to be followed by the elements.
    ///
    /// Pairs with [`Source::read_seq_len`]. Separated from the elements because
    /// element encoding is the caller's business; this only fixes how the count
    /// is framed.
    fn write_seq_len(&mut self, count: u64) -> Result<()> {
        self.write_u64(count)
    }
}

impl Sink for Vec<u8> {
    fn write_all(&mut self, bytes: &[u8]) -> Result<()> {
        self.extend_from_slice(bytes);
        Ok(())
    }
}

impl<S: Sink + ?Sized> Sink for &mut S {
    fn write_all(&mut self, bytes: &[u8]) -> Result<()> {
        (**self).write_all(bytes)
    }
}

// ---------------------------------------------------------------------------
// Source
// ---------------------------------------------------------------------------

/// Generate the little-endian integer readers as provided trait methods.
macro_rules! source_int_methods {
    ($($name:ident: $t:ty;)*) => {
        $(
            #[doc = concat!("Read a little-endian `", stringify!($t), "`.")]
            fn $name(&mut self) -> Result<$t> {
                const N: usize = core::mem::size_of::<$t>();
                let bytes = self.take(N)?;
                // `take` promises exactly N bytes; the conversion is checked
                // anyway so a buggy `Source` impl is an error, not a panic.
                let array = <[u8; N]>::try_from(bytes).map_err(|_| truncated(N, bytes.len()))?;
                Ok(<$t>::from_le_bytes(array))
            }
        )*
    };
}

/// A place encoded bytes come from.
///
/// The mirror of [`Sink`], and equally not `std::io::Read`. The lifetime is on
/// the trait rather than on the method so [`Source::take`] can hand back a
/// borrow of the *input* rather than of the source: decoding a chunk or a
/// string is then a sub-slice, not a copy, which matters when the chunk is a
/// gigabyte of guest RAM.
///
/// Implementors provide [`Source::take`] and [`Source::remaining`]. Everything
/// else is a provided decoder and must not be overridden.
pub trait Source<'a> {
    /// Consume exactly `len` bytes and return them.
    ///
    /// Must fail with [`Error::State`] rather than panic when fewer than `len`
    /// bytes remain — the whole parser's panic-freedom rests on this one
    /// method.
    fn take(&mut self, len: usize) -> Result<&'a [u8]>;

    /// How many bytes are still unread.
    ///
    /// Used to bound allocations against the input actually present, so a
    /// corrupt count field cannot make us reserve gigabytes.
    fn remaining(&self) -> usize;

    source_int_methods! {
        read_u8: u8;
        read_u16: u16;
        read_u32: u32;
        read_u64: u64;
        read_u128: u128;
        read_i8: i8;
        read_i16: i16;
        read_i32: i32;
        read_i64: i64;
        read_i128: i128;
    }

    /// Read a bool, rejecting any byte that is not `0` or `1`.
    fn read_bool(&mut self) -> Result<bool> {
        match self.read_u8()? {
            0 => Ok(false),
            1 => Ok(true),
            other => Err(state_error(format!(
                "invalid bool byte 0x{other:02x} (expected 0x00 or 0x01)"
            ))),
        }
    }

    /// Read a length-prefixed byte array, borrowed from the input.
    fn read_bytes(&mut self) -> Result<&'a [u8]> {
        let len = wire_len(self.read_u64()?)?;
        self.take(len)
    }

    /// Read a length-prefixed UTF-8 string, borrowed from the input.
    ///
    /// Invalid UTF-8 is an error, not a lossy replacement: an instance path
    /// that silently changed under us would defeat the point of keying chunks
    /// by path.
    fn read_str(&mut self) -> Result<&'a str> {
        let bytes = self.read_bytes()?;
        core::str::from_utf8(bytes)
            .map_err(|e| state_error(format!("invalid UTF-8 in snapshot string: {e}")))
    }

    /// Read a length-prefixed string as an owned [`String`].
    fn read_string(&mut self) -> Result<String> {
        self.read_str().map(ToString::to_string)
    }

    /// Read the element count of a sequence, bounded by the bytes available.
    ///
    /// `min_element_bytes` is the smallest number of bytes one element can
    /// possibly occupy — at least 1, and larger when the element type has a
    /// fixed minimum. Any count whose elements could not fit in the remaining
    /// input is rejected *before* the caller allocates, which is what stops a
    /// corrupt `u64` count from becoming an out-of-memory abort.
    fn read_seq_len(&mut self, min_element_bytes: u64) -> Result<u64> {
        let count = self.read_u64()?;
        let min = min_element_bytes.max(1);
        let floor = count.checked_mul(min).ok_or_else(|| {
            state_error(format!(
                "sequence of {count} element(s) overflows any possible encoding"
            ))
        })?;
        let have = self.remaining() as u64;
        if floor > have {
            return Err(state_error(format!(
                "sequence claims {count} element(s) needing at least {floor} byte(s), \
                 but only {have} remain"
            )));
        }
        Ok(count)
    }
}

impl<'a, S: Source<'a> + ?Sized> Source<'a> for &mut S {
    fn take(&mut self, len: usize) -> Result<&'a [u8]> {
        (**self).take(len)
    }

    fn remaining(&self) -> usize {
        (**self).remaining()
    }
}

/// A [`Source`] over a byte slice: the only one this build needs.
///
/// Kept separate from the readers so that a future streaming source (a
/// decompressor, a memory-mapped file) is a new implementation rather than a
/// change to the parser.
#[derive(Debug, Clone)]
pub struct SliceSource<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> SliceSource<'a> {
    /// Start reading at the beginning of `data`.
    pub const fn new(data: &'a [u8]) -> Self {
        SliceSource { data, pos: 0 }
    }

    /// How many bytes have been consumed so far.
    pub const fn position(&self) -> usize {
        self.pos
    }
}

impl<'a> Source<'a> for SliceSource<'a> {
    fn take(&mut self, len: usize) -> Result<&'a [u8]> {
        let end = self
            .pos
            .checked_add(len)
            .ok_or_else(|| truncated(len, self.remaining()))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or_else(|| truncated(len, self.remaining()))?;
        self.pos = end;
        Ok(slice)
    }

    fn remaining(&self) -> usize {
        // Saturating rather than `-`: `pos` never exceeds `data.len()`, but a
        // parser is not the place to prove an invariant with a subtraction that
        // would panic in debug builds if it ever broke.
        self.data.len().saturating_sub(self.pos)
    }
}

// ---------------------------------------------------------------------------
// Machine shape: the structural fingerprint
// ---------------------------------------------------------------------------

/// One mapped region, as it contributes to the machine's identity.
///
/// Region *layout* is part of the fingerprint because a snapshot restored into
/// a machine whose RAM sits somewhere else is not the same machine, however
/// identical the device list looks. Region *contents* are not here — they live
/// in the owning device's chunk.
///
/// Field order is the sort order (`space`, `base`, `size`, `name`), which is
/// what makes the encoded set deterministic.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RegionShape {
    /// Which address space the region is in (`"mem"`, `"io"`, a bus name).
    pub space: String,
    /// Where it starts, in that space.
    pub base: u64,
    /// How many bytes it covers.
    pub size: u64,
    /// The region's name, so the diff can point at something a human named.
    pub name: String,
}

impl fmt::Display for RegionShape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{} @ {:#x}+{:#x}",
            self.space, self.name, self.base, self.size
        )
    }
}

/// The structural fingerprint of a machine.
///
/// This is what a snapshot is checked against, and it is deliberately a
/// *structure* rather than a hash (`ROADMAP.md` §4.5): it can be diffed, so a
/// mismatch tells the user which device moved rather than only that something
/// did. Editing a comment in a `.machine` file, or renaming a parameter, does
/// not change it; adding a device or moving a region does.
///
/// It holds:
///
/// - **devices**: instance path → class name. The path is the identity; the
///   class is what must still be there.
/// - **regions**: the layout described above.
/// - **features**: the build's feature set, so "this snapshot needs a build
///   with `dev-nvme`" is answerable.
/// - **arches**: the guest architectures present, for the same reason.
///
/// Class *versions* are deliberately absent: a version difference is not a
/// shape difference, it is a migration ([`Migrations`]).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MachineShape {
    devices: BTreeMap<String, String>,
    regions: BTreeSet<RegionShape>,
    features: BTreeSet<String>,
    arches: BTreeSet<String>,
}

impl MachineShape {
    /// An empty shape: no devices, no regions, no features.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a device instance at `path` of class `class`.
    ///
    /// Two devices cannot share an instance path — that would make the chunk
    /// key ambiguous — so a duplicate is an error rather than an overwrite.
    pub fn add_device(&mut self, path: &str, class: &str) -> Result<()> {
        if let Some(existing) = self.devices.get(path) {
            return Err(state_error(format!(
                "duplicate instance path `{path}`: already registered as class `{existing}`"
            )));
        }
        self.devices.insert(path.to_string(), class.to_string());
        Ok(())
    }

    /// Record a mapped region.
    ///
    /// Adding the identical region twice is a no-op: the shape is a set, and a
    /// machine builder that walks its topology twice must not produce a
    /// different fingerprint than one that walks it once.
    pub fn add_region(&mut self, space: &str, name: &str, base: u64, size: u64) {
        self.regions.insert(RegionShape {
            space: space.to_string(),
            base,
            size,
            name: name.to_string(),
        });
    }

    /// Record a build feature this machine depends on.
    pub fn add_feature(&mut self, feature: &str) {
        self.features.insert(feature.to_string());
    }

    /// Record a guest architecture present in this machine.
    pub fn add_arch(&mut self, arch: &str) {
        self.arches.insert(arch.to_string());
    }

    /// The device instances, as instance path → class name, in path order.
    pub fn devices(&self) -> &BTreeMap<String, String> {
        &self.devices
    }

    /// The class registered at `path`, if any.
    pub fn device_class(&self, path: &str) -> Option<&str> {
        self.devices.get(path).map(String::as_str)
    }

    /// The mapped regions, in sort order.
    pub fn regions(&self) -> &BTreeSet<RegionShape> {
        &self.regions
    }

    /// The feature set.
    pub fn features(&self) -> &BTreeSet<String> {
        &self.features
    }

    /// The guest architectures.
    pub fn arches(&self) -> &BTreeSet<String> {
        &self.arches
    }

    /// Whether this shape describes nothing at all.
    pub fn is_empty(&self) -> bool {
        self.devices.is_empty()
            && self.regions.is_empty()
            && self.features.is_empty()
            && self.arches.is_empty()
    }

    /// Compare a snapshot's shape (`self`) against a machine's (`machine`).
    ///
    /// The direction matters for the wording of the result: `self` is what the
    /// snapshot says, `machine` is what is actually here.
    pub fn diff(&self, machine: &MachineShape) -> ShapeDiff {
        let mut differences = Vec::new();

        for (path, class) in &self.devices {
            match machine.devices.get(path) {
                None => differences.push(ShapeDifference::DeviceMissing {
                    path: path.clone(),
                    class: class.clone(),
                }),
                Some(other) if other != class => {
                    differences.push(ShapeDifference::DeviceClassChanged {
                        path: path.clone(),
                        snapshot_class: class.clone(),
                        machine_class: other.clone(),
                    });
                }
                Some(_) => {}
            }
        }
        for (path, class) in &machine.devices {
            if !self.devices.contains_key(path) {
                differences.push(ShapeDifference::DeviceUnexpected {
                    path: path.clone(),
                    class: class.clone(),
                });
            }
        }

        // Regions: report a move as a move. Reporting "region X is gone" plus
        // "region X appeared" for a base address that shifted by one page is
        // technically true and practically useless.
        let only_snapshot: Vec<&RegionShape> = self.regions.difference(&machine.regions).collect();
        let only_machine: Vec<&RegionShape> = machine.regions.difference(&self.regions).collect();
        let mut paired = alloc::vec![false; only_machine.len()];
        for snap in &only_snapshot {
            let mate = only_machine
                .iter()
                .enumerate()
                .find(|(i, m)| !paired[*i] && m.space == snap.space && m.name == snap.name);
            match mate {
                Some((i, m)) => {
                    paired[i] = true;
                    differences.push(ShapeDifference::RegionMoved {
                        snapshot: (*snap).clone(),
                        machine: (*m).clone(),
                    });
                }
                None => differences.push(ShapeDifference::RegionMissing((*snap).clone())),
            }
        }
        for (i, m) in only_machine.iter().enumerate() {
            if !paired[i] {
                differences.push(ShapeDifference::RegionUnexpected((*m).clone()));
            }
        }

        for feature in self.features.difference(&machine.features) {
            differences.push(ShapeDifference::FeatureMissing(feature.clone()));
        }
        for feature in machine.features.difference(&self.features) {
            differences.push(ShapeDifference::FeatureUnexpected(feature.clone()));
        }
        for arch in self.arches.difference(&machine.arches) {
            differences.push(ShapeDifference::ArchMissing(arch.clone()));
        }
        for arch in machine.arches.difference(&self.arches) {
            differences.push(ShapeDifference::ArchUnexpected(arch.clone()));
        }

        ShapeDiff { differences }
    }

    /// Encode the shape into `sink`.
    ///
    /// Public because the shape is the identity check for more than one file
    /// format: a snapshot carries it in its header, and a
    /// [recording](crate::core::record::InputLog) carries it so that replaying
    /// into a differently-shaped machine fails with a [`ShapeDiff`] rather than
    /// by injecting input into whatever device answers to that name.
    ///
    /// # Errors
    ///
    /// Whatever `sink` reports.
    pub fn encode_into<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        sink.write_seq_len(self.devices.len() as u64)?;
        for (path, class) in &self.devices {
            sink.write_str(path)?;
            sink.write_str(class)?;
        }
        sink.write_seq_len(self.regions.len() as u64)?;
        for region in &self.regions {
            sink.write_str(&region.space)?;
            sink.write_u64(region.base)?;
            sink.write_u64(region.size)?;
            sink.write_str(&region.name)?;
        }
        write_string_set(sink, &self.features)?;
        write_string_set(sink, &self.arches)
    }

    /// Decode a shape, rejecting any non-canonical ordering.
    ///
    /// Ordering is enforced rather than merely sorted-on-load so that a
    /// snapshot has exactly one valid encoding; that is what lets "decode then
    /// re-encode" be byte-identical, which the determinism tests rely on.
    ///
    /// The mirror of [`MachineShape::encode_into`], and public for the same
    /// reason.
    ///
    /// # Errors
    ///
    /// [`Error::State`] for a truncated, out-of-order or non-UTF-8 encoding.
    pub fn decode_from<'a, S: Source<'a> + ?Sized>(src: &mut S) -> Result<Self> {
        // A device entry is two length-prefixed strings: 16 bytes minimum.
        let device_count = src.read_seq_len(16)?;
        let mut devices = BTreeMap::new();
        let mut previous: Option<&str> = None;
        for _ in 0..device_count {
            let path = src.read_str()?;
            let class = src.read_str()?;
            if let Some(prev) = previous
                && path <= prev
            {
                return Err(state_error(format!(
                    "device entries out of order: `{path}` after `{prev}` \
                     (paths must be unique and ascending)"
                )));
            }
            previous = Some(path);
            devices.insert(path.to_string(), class.to_string());
        }

        // A region entry is two strings plus two u64s: 32 bytes minimum.
        let region_count = src.read_seq_len(32)?;
        let mut regions: BTreeSet<RegionShape> = BTreeSet::new();
        let mut previous_region: Option<RegionShape> = None;
        for _ in 0..region_count {
            let space = src.read_string()?;
            let base = src.read_u64()?;
            let size = src.read_u64()?;
            let name = src.read_string()?;
            let region = RegionShape {
                space,
                base,
                size,
                name,
            };
            if let Some(prev) = &previous_region
                && region <= *prev
            {
                return Err(state_error(format!(
                    "region entries out of order: {region} after {prev}"
                )));
            }
            previous_region = Some(region.clone());
            regions.insert(region);
        }

        let features = read_string_set(src, "feature")?;
        let arches = read_string_set(src, "arch")?;

        Ok(MachineShape {
            devices,
            regions,
            features,
            arches,
        })
    }
}

/// Write a set of strings in ascending order.
fn write_string_set<S: Sink + ?Sized>(sink: &mut S, set: &BTreeSet<String>) -> Result<()> {
    sink.write_seq_len(set.len() as u64)?;
    for item in set {
        sink.write_str(item)?;
    }
    Ok(())
}

/// Read a set of strings, rejecting duplicates and out-of-order entries.
fn read_string_set<'a, S: Source<'a> + ?Sized>(
    src: &mut S,
    what: &str,
) -> Result<BTreeSet<String>> {
    let count = src.read_seq_len(8)?;
    let mut set = BTreeSet::new();
    let mut previous: Option<&str> = None;
    for _ in 0..count {
        let item = src.read_str()?;
        if let Some(prev) = previous
            && item <= prev
        {
            return Err(state_error(format!(
                "{what} entries out of order: `{item}` after `{prev}` \
                 (must be unique and ascending)"
            )));
        }
        previous = Some(item);
        set.insert(item.to_string());
    }
    Ok(set)
}

/// One way a snapshot's machine differs from the machine it is loading into.
///
/// Non-exhaustive because later phases will find more that is worth naming
/// (bus topology, CPU count), and adding a variant must not break callers.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ShapeDifference {
    /// The snapshot has a device this machine does not.
    DeviceMissing {
        /// The instance path.
        path: String,
        /// The class the snapshot has there.
        class: String,
    },
    /// This machine has a device the snapshot does not.
    DeviceUnexpected {
        /// The instance path.
        path: String,
        /// The class this machine has there.
        class: String,
    },
    /// Both have a device at this path, but of different classes.
    DeviceClassChanged {
        /// The instance path.
        path: String,
        /// What the snapshot says is there.
        snapshot_class: String,
        /// What is actually there.
        machine_class: String,
    },
    /// The snapshot has a region this machine does not.
    RegionMissing(RegionShape),
    /// This machine has a region the snapshot does not.
    RegionUnexpected(RegionShape),
    /// The same named region sits at a different address or has a different
    /// size.
    RegionMoved {
        /// Where the snapshot says it is.
        snapshot: RegionShape,
        /// Where it actually is.
        machine: RegionShape,
    },
    /// The snapshot needs a build feature this build does not have.
    FeatureMissing(String),
    /// This build has a feature the snapshot was not taken with.
    FeatureUnexpected(String),
    /// The snapshot names a guest architecture this machine does not have.
    ArchMissing(String),
    /// This machine has a guest architecture the snapshot does not name.
    ArchUnexpected(String),
}

impl fmt::Display for ShapeDifference {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShapeDifference::DeviceMissing { path, class } => write!(
                f,
                "- device `{path}` (class `{class}`) is in the snapshot but not in this machine"
            ),
            ShapeDifference::DeviceUnexpected { path, class } => write!(
                f,
                "+ device `{path}` (class `{class}`) is in this machine but not in the snapshot"
            ),
            ShapeDifference::DeviceClassChanged {
                path,
                snapshot_class,
                machine_class,
            } => write!(
                f,
                "! device `{path}` is class `{machine_class}` here \
                 but `{snapshot_class}` in the snapshot"
            ),
            ShapeDifference::RegionMissing(r) => {
                write!(f, "- region {r} is in the snapshot but not in this machine")
            }
            ShapeDifference::RegionUnexpected(r) => {
                write!(f, "+ region {r} is in this machine but not in the snapshot")
            }
            ShapeDifference::RegionMoved { snapshot, machine } => write!(
                f,
                "! region {}/{} is at {:#x}+{:#x} here but {:#x}+{:#x} in the snapshot",
                machine.space,
                machine.name,
                machine.base,
                machine.size,
                snapshot.base,
                snapshot.size
            ),
            ShapeDifference::FeatureMissing(feat) => write!(
                f,
                "- the snapshot needs feature `{feat}`, which this build does not have"
            ),
            ShapeDifference::FeatureUnexpected(feat) => write!(
                f,
                "+ this build has feature `{feat}`, which the snapshot was not taken with"
            ),
            ShapeDifference::ArchMissing(arch) => write!(
                f,
                "- the snapshot has guest architecture `{arch}`, absent from this machine"
            ),
            ShapeDifference::ArchUnexpected(arch) => write!(
                f,
                "+ this machine has guest architecture `{arch}`, absent from the snapshot"
            ),
        }
    }
}

/// The full set of differences between a snapshot's machine and this one.
///
/// This is what `ROADMAP.md` §4.5 means by "fails with a diff, not a crash": it
/// is inspectable ([`ShapeDiff::differences`]) for tooling and printable for
/// humans.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ShapeDiff {
    differences: Vec<ShapeDifference>,
}

impl ShapeDiff {
    /// Whether the two shapes matched.
    pub fn is_empty(&self) -> bool {
        self.differences.is_empty()
    }

    /// How many differences were found.
    pub fn len(&self) -> usize {
        self.differences.len()
    }

    /// The differences, in a deterministic order: devices, then regions, then
    /// features, then architectures.
    pub fn differences(&self) -> &[ShapeDifference] {
        &self.differences
    }
}

impl fmt::Display for ShapeDiff {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.differences.is_empty() {
            return f.write_str("machine shapes match");
        }
        write!(
            f,
            "snapshot does not match this machine ({} difference(s)):",
            self.differences.len()
        )?;
        for difference in self.differences.iter().take(MAX_DIFF_LINES) {
            write!(f, "\n  {difference}")?;
        }
        if self.differences.len() > MAX_DIFF_LINES {
            write!(
                f,
                "\n  ... and {} more",
                self.differences.len() - MAX_DIFF_LINES
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

/// One buffered chunk, before the snapshot is emitted.
#[derive(Debug)]
struct PendingChunk {
    class: String,
    version: u32,
    data: Vec<u8>,
}

/// Builds a snapshot.
///
/// Chunks are buffered and emitted in instance-path order by
/// [`StateWriter::write_to`], so the order devices happen to be walked in is
/// not observable in the output. That costs one copy of the state in memory and
/// buys byte-identical snapshots, which is the trade `ROADMAP.md` §0 asks for.
/// (A streaming writer would have to fix the order at the call site, and every
/// future parallel save would silently break determinism.)
///
/// The writer is not consumed by writing, so the same state can be emitted
/// twice and compared — which is exactly what the determinism test does.
#[derive(Debug)]
pub struct StateWriter {
    shape: MachineShape,
    codec: Codec,
    integrity: Integrity,
    chunks: BTreeMap<String, PendingChunk>,
}

impl StateWriter {
    /// Start a snapshot of a machine with this shape.
    pub fn new(shape: MachineShape) -> Self {
        StateWriter {
            shape,
            codec: Codec::STORED,
            integrity: Integrity::NONE,
            chunks: BTreeMap::new(),
        }
    }

    /// The shape this snapshot records.
    pub fn shape(&self) -> &MachineShape {
        &self.shape
    }

    /// How many chunks have been added.
    pub fn len(&self) -> usize {
        self.chunks.len()
    }

    /// Whether no chunk has been added yet.
    pub fn is_empty(&self) -> bool {
        self.chunks.is_empty()
    }

    /// Begin the chunk for the device instance at `path`.
    ///
    /// `version` is the class's state version — the number whose upgrade path
    /// [`Migrations`] describes. Bump it in the same commit that changes what
    /// the device writes, and register the `vN -> vN+1` function then, not
    /// later.
    ///
    /// One chunk per instance path: a second chunk for the same path is an
    /// error, because the path is the key a loader looks state up by and a
    /// duplicate would make "which one is mine" unanswerable.
    ///
    /// The returned writer borrows the snapshot, so chunks are necessarily
    /// written one at a time. Bytes are kept as they are written; there is no
    /// commit step to forget.
    pub fn chunk(&mut self, path: &str, class: &str, version: u32) -> Result<ChunkWriter<'_>> {
        if let Some(existing) = self.chunks.get(path) {
            return Err(state_error(format!(
                "duplicate chunk for instance path `{path}` \
                 (already written as class `{}` v{})",
                existing.class, existing.version
            )));
        }
        if path.is_empty() {
            return Err(state_error(
                "a chunk needs a non-empty instance path".to_string(),
            ));
        }
        let entry = self.chunks.entry(path.to_string()).or_insert(PendingChunk {
            class: class.to_string(),
            version,
            data: Vec::new(),
        });
        Ok(ChunkWriter {
            buffer: &mut entry.data,
        })
    }

    /// Add a chunk whose payload is already encoded.
    ///
    /// The escape hatch for a caller holding bytes rather than a device — a
    /// migration tool, a test fixture, a chunk copied verbatim from another
    /// snapshot.
    pub fn raw_chunk(&mut self, path: &str, class: &str, version: u32, data: &[u8]) -> Result<()> {
        let mut writer = self.chunk(path, class, version)?;
        writer.write_all(data)
    }

    /// Emit the whole snapshot into `sink`.
    ///
    /// Takes `&self`: writing does not consume the writer, so the identical
    /// state can be emitted more than once and the results compared byte for
    /// byte.
    pub fn write_to<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<()> {
        sink.write_all(&MAGIC)?;
        sink.write_u32(FORMAT_VERSION)?;
        sink.write_u16(self.codec.0)?;
        sink.write_u16(self.integrity.0)?;
        self.shape.encode_into(sink)?;
        // BTreeMap iteration is ascending by instance path: the canonical order
        // the reader checks for.
        for (path, chunk) in &self.chunks {
            sink.write_u8(TAG_CHUNK)?;
            sink.write_str(path)?;
            sink.write_str(&chunk.class)?;
            sink.write_u32(chunk.version)?;
            sink.write_bytes(&chunk.data)?;
        }
        sink.write_u8(TAG_END)
    }

    /// Emit the whole snapshot into a fresh [`Vec`].
    pub fn to_vec(&self) -> Result<Vec<u8>> {
        let mut out = Vec::new();
        self.write_to(&mut out)?;
        Ok(out)
    }
}

/// Encodes one device's state into its chunk.
///
/// Every [`Sink`] method is available here; a device's `save` is a straight
/// sequence of `write_*` calls in a fixed order, matched field for field by its
/// `load`. Only *architectural* state belongs in it — a TLB, a flat view, a
/// translation block or a host pointer is derived and must be rebuilt on load
/// (`ROADMAP.md` §15, invariant 3).
#[derive(Debug)]
pub struct ChunkWriter<'a> {
    buffer: &'a mut Vec<u8>,
}

impl ChunkWriter<'_> {
    /// How many bytes have been written to this chunk so far.
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Whether nothing has been written to this chunk yet.
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

impl Sink for ChunkWriter<'_> {
    fn write_all(&mut self, bytes: &[u8]) -> Result<()> {
        self.buffer.extend_from_slice(bytes);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Migration
// ---------------------------------------------------------------------------

/// One step of a class's upgrade chain: `vN` bytes in, `vN+1` bytes out.
///
/// A plain `fn` pointer rather than a boxed closure: migrations are static
/// facts about a class, the table has to be `Send + Sync` like everything else
/// in the core, and a `fn` keeps [`Migrations`] `Debug` and cheap to clone.
///
/// The step reads the old encoding with the same [`Source`] primitives the old
/// `load` used, and writes the new one with the same [`Sink`] primitives the
/// new `save` uses — so a migration is written by copying the two functions and
/// editing between them.
pub type MigrateFn = fn(&mut ChunkReader<'_>, &mut Vec<u8>) -> Result<()>;

/// The registered `vN -> vN+1` upgrade functions, per class.
///
/// A version field with no migration mechanism is decoration (`ROADMAP.md`
/// §4.5). This is the mechanism: a class registers each step it has ever
/// needed, and loading an old chunk walks the chain up to the version this
/// build understands. Steps are single-version hops on purpose — writing
/// `v1 -> v4` directly means writing `v2 -> v4` and `v3 -> v4` too, and one of
/// them will be the one that rots.
#[derive(Debug, Clone, Default)]
pub struct Migrations {
    /// class → (from-version → step producing from-version + 1).
    steps: BTreeMap<String, BTreeMap<u32, MigrateFn>>,
}

impl Migrations {
    /// An empty table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register the step that turns a `from_version` chunk of `class` into a
    /// `from_version + 1` one.
    ///
    /// Registering the same step twice is an error rather than an overwrite:
    /// two crates disagreeing about how v3 becomes v4 is a bug that must not
    /// resolve itself by load order.
    pub fn register(&mut self, class: &str, from_version: u32, step: MigrateFn) -> Result<()> {
        if from_version == u32::MAX {
            return Err(state_error(format!(
                "class `{class}`: version {from_version} has no successor"
            )));
        }
        let per_class = self.steps.entry(class.to_string()).or_default();
        if per_class.contains_key(&from_version) {
            return Err(state_error(format!(
                "class `{class}`: a migration from v{from_version} is already registered"
            )));
        }
        per_class.insert(from_version, step);
        Ok(())
    }

    /// The versions `class` can migrate *from*, ascending.
    pub fn steps_for(&self, class: &str) -> Vec<u32> {
        self.steps
            .get(class)
            .map(|m| m.keys().copied().collect())
            .unwrap_or_default()
    }

    /// Run the chain that turns `from_version` bytes into `to_version` bytes.
    ///
    /// Returns the input untouched when the versions already agree, so the
    /// common case does not copy. Fails, naming the gap, when a step is
    /// missing; fails when `from_version` is the newer of the two, because a
    /// snapshot from a future build cannot be understood by guessing.
    pub fn upgrade<'a>(
        &self,
        class: &str,
        from_version: u32,
        to_version: u32,
        data: Cow<'a, [u8]>,
    ) -> Result<Cow<'a, [u8]>> {
        if from_version == to_version {
            return Ok(data);
        }
        if from_version > to_version {
            return Err(state_error(format!(
                "class `{class}`: snapshot chunk is v{from_version} but this build understands \
                 v{to_version}; snapshots can be upgraded, never downgraded"
            )));
        }

        let mut current = data;
        let mut version = from_version;
        while version < to_version {
            let step = self.steps.get(class).and_then(|m| m.get(&version)).copied();
            let Some(step) = step else {
                let known = self.steps_for(class);
                let known = if known.is_empty() {
                    "none registered".to_string()
                } else {
                    let mut s = String::new();
                    for (i, v) in known.iter().enumerate() {
                        if i > 0 {
                            s.push_str(", ");
                        }
                        s.push_str(&format!("v{v}->v{}", v + 1));
                    }
                    s
                };
                return Err(state_error(format!(
                    "class `{class}`: no migration from v{version} to v{} \
                     (upgrading a v{from_version} chunk to v{to_version}; registered: {known})",
                    version + 1
                )));
            };
            let migrated = {
                let mut reader = ChunkReader::new(&current);
                let mut out = Vec::new();
                step(&mut reader, &mut out)?;
                out
            };
            current = Cow::Owned(migrated);
            version += 1;
        }
        Ok(current)
    }
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

/// A chunk as it sits in the input, before any migration.
#[derive(Debug, Clone, Copy)]
struct RawChunk<'a> {
    path: &'a str,
    class: &'a str,
    version: u32,
    data: &'a [u8],
}

/// What a snapshot says about one chunk, without decoding it.
///
/// Enough for `rsemu` to list a snapshot's contents, and for a loader to decide
/// what it is about to do before doing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkInfo<'a> {
    /// The device instance path this chunk belongs to.
    pub path: &'a str,
    /// The device class that wrote it.
    pub class: &'a str,
    /// The class state version it was written at.
    pub version: u32,
    /// The payload size in bytes.
    pub len: usize,
}

/// A parsed snapshot, ready to be loaded chunk by chunk.
///
/// Parsing is up-front and total: by the time this exists, the header is valid,
/// every chunk is framed correctly and in canonical order, and no length field
/// points past the end of the input. What is *inside* a chunk is the owning
/// device's business and is not touched here.
#[derive(Debug, Clone)]
pub struct StateReader<'a> {
    codec: Codec,
    integrity: Integrity,
    shape: MachineShape,
    chunks: Vec<RawChunk<'a>>,
}

impl<'a> StateReader<'a> {
    /// Parse a snapshot from a byte slice.
    pub fn new(bytes: &'a [u8]) -> Result<Self> {
        let mut source = SliceSource::new(bytes);
        Self::from_source(&mut source)
    }

    /// Parse a snapshot from any [`Source`].
    ///
    /// Trailing bytes after the end marker are an error: a snapshot has exactly
    /// one encoding, and "there is more here than I understand" is a corrupt
    /// file, not a feature.
    pub fn from_source<S: Source<'a> + ?Sized>(src: &mut S) -> Result<Self> {
        let magic = src.take(MAGIC.len())?;
        if magic != MAGIC {
            return Err(state_error(format!(
                "not a snapshot: expected magic {:?}, found {:?}",
                core::str::from_utf8(&MAGIC).unwrap_or("RSEMUSNP"),
                DisplayBytes(magic)
            )));
        }
        let format = src.read_u32()?;
        if format != FORMAT_VERSION {
            return Err(state_error(format!(
                "snapshot container format v{format}, but this build reads v{FORMAT_VERSION}"
            )));
        }
        let codec = Codec(src.read_u16()?);
        if codec != Codec::STORED {
            return Err(state_error(format!(
                "snapshot uses codec {} (compression is the compcol seam and is not \
                 implemented in this build)",
                codec.0
            )));
        }
        let integrity = Integrity(src.read_u16()?);
        if integrity != Integrity::NONE {
            return Err(state_error(format!(
                "snapshot uses integrity scheme {} (digests and encryption are the \
                 purecrypto seam and are not implemented in this build)",
                integrity.0
            )));
        }

        let shape = MachineShape::decode_from(src)?;

        let mut chunks: Vec<RawChunk<'a>> = Vec::new();
        loop {
            match src.read_u8()? {
                TAG_END => break,
                TAG_CHUNK => {
                    let path = src.read_str()?;
                    let class = src.read_str()?;
                    let version = src.read_u32()?;
                    let data = src.read_bytes()?;
                    if let Some(previous) = chunks.last()
                        && path <= previous.path
                    {
                        return Err(state_error(format!(
                            "chunks out of order: `{path}` after `{}` \
                             (instance paths must be unique and ascending)",
                            previous.path
                        )));
                    }
                    chunks.push(RawChunk {
                        path,
                        class,
                        version,
                        data,
                    });
                }
                other => {
                    return Err(state_error(format!(
                        "unknown chunk tag 0x{other:02x} (expected 0x{TAG_CHUNK:02x} \
                         for a chunk or 0x{TAG_END:02x} for the end)"
                    )));
                }
            }
        }
        let left = src.remaining();
        if left != 0 {
            return Err(state_error(format!(
                "{left} trailing byte(s) after the end of the snapshot"
            )));
        }

        Ok(StateReader {
            codec,
            integrity,
            shape,
            chunks,
        })
    }

    /// The shape of the machine this snapshot was taken from.
    pub fn shape(&self) -> &MachineShape {
        &self.shape
    }

    /// The payload codec named in the header (always [`Codec::STORED`] here).
    pub fn codec(&self) -> Codec {
        self.codec
    }

    /// The integrity scheme named in the header (always [`Integrity::NONE`]).
    pub fn integrity(&self) -> Integrity {
        self.integrity
    }

    /// Every chunk's key, in instance-path order.
    pub fn chunks(&self) -> impl Iterator<Item = ChunkInfo<'a>> + '_ {
        self.chunks.iter().map(|c| ChunkInfo {
            path: c.path,
            class: c.class,
            version: c.version,
            len: c.data.len(),
        })
    }

    /// What this snapshot's machine and `machine` disagree about.
    pub fn diff_shape(&self, machine: &MachineShape) -> ShapeDiff {
        self.shape.diff(machine)
    }

    /// Fail unless this snapshot's machine matches `machine`.
    ///
    /// The error is the rendered [`ShapeDiff`] — the thing §4.5 asks for
    /// instead of a boolean. Call it before loading anything: half a machine
    /// restored from a snapshot of a different machine is worse than a refusal.
    pub fn check_shape(&self, machine: &MachineShape) -> Result<()> {
        let diff = self.diff_shape(machine);
        if diff.is_empty() {
            Ok(())
        } else {
            Err(state_error(diff.to_string()))
        }
    }

    /// The chunk at `path`, if there is one.
    pub fn find(&self, path: &str) -> Option<ChunkInfo<'a>> {
        self.raw(path).map(|c| ChunkInfo {
            path: c.path,
            class: c.class,
            version: c.version,
            len: c.data.len(),
        })
    }

    fn raw(&self, path: &str) -> Option<&RawChunk<'a>> {
        // Chunks are sorted by path, so this is a binary search rather than a
        // scan: a phase-6 PC has hundreds of instances and load is O(n) calls.
        self.chunks
            .binary_search_by(|c| c.path.cmp(path))
            .ok()
            .and_then(|i| self.chunks.get(i))
    }

    /// Load the chunk for `path`, migrating it to `version` if needed.
    ///
    /// Fails when there is no chunk at that path, when the chunk was written by
    /// a different class, when the chunk is newer than this build understands,
    /// or when the migration chain has a hole — each with a message naming what
    /// is wrong.
    pub fn load(
        &self,
        path: &str,
        class: &str,
        version: u32,
        migrations: &Migrations,
    ) -> Result<LoadedChunk<'a>> {
        let Some(chunk) = self.raw(path) else {
            return Err(state_error(format!(
                "snapshot has no chunk for instance path `{path}` ({})",
                self.summarise_paths()
            )));
        };
        if chunk.class != class {
            return Err(state_error(format!(
                "instance `{path}` is class `{class}` here but the snapshot has \
                 class `{}` there",
                chunk.class
            )));
        }
        let data = migrations.upgrade(
            class,
            chunk.version,
            version,
            Cow::Borrowed::<'a, [u8]>(chunk.data),
        )?;
        Ok(LoadedChunk {
            path: chunk.path,
            class: chunk.class,
            stored_version: chunk.version,
            version,
            data,
        })
    }

    /// The chunk payload exactly as stored, with no migration.
    ///
    /// For tooling that copies or inspects chunks. A device should use
    /// [`StateReader::load`].
    pub fn load_raw(&self, path: &str) -> Result<(&'a str, u32, &'a [u8])> {
        let Some(chunk) = self.raw(path) else {
            return Err(state_error(format!(
                "snapshot has no chunk for instance path `{path}` ({})",
                self.summarise_paths()
            )));
        };
        Ok((chunk.class, chunk.version, chunk.data))
    }

    /// A short list of the paths that *are* present, for error messages.
    fn summarise_paths(&self) -> String {
        if self.chunks.is_empty() {
            return "the snapshot has no chunks at all".to_string();
        }
        let shown = 8usize.min(self.chunks.len());
        let mut s = String::from("present: ");
        for (i, chunk) in self.chunks.iter().take(shown).enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push('`');
            s.push_str(chunk.path);
            s.push('`');
        }
        if self.chunks.len() > shown {
            s.push_str(&format!(" and {} more", self.chunks.len() - shown));
        }
        s
    }
}

/// Render bytes for an error message without assuming they are text.
struct DisplayBytes<'a>(&'a [u8]);

impl fmt::Debug for DisplayBytes<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("0x")?;
        for byte in self.0 {
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// One chunk, migrated to the version the caller asked for.
///
/// Holds a [`Cow`]: an up-to-date chunk borrows the snapshot bytes and costs
/// nothing, while a migrated one owns what the chain produced.
#[derive(Debug, Clone)]
pub struct LoadedChunk<'a> {
    path: &'a str,
    class: &'a str,
    stored_version: u32,
    version: u32,
    data: Cow<'a, [u8]>,
}

impl<'a> LoadedChunk<'a> {
    /// The instance path this chunk belongs to.
    pub fn path(&self) -> &'a str {
        self.path
    }

    /// The class that wrote it.
    pub fn class(&self) -> &'a str {
        self.class
    }

    /// The version the payload is now at — the one that was asked for.
    pub fn version(&self) -> u32 {
        self.version
    }

    /// The version it was written at, before migration.
    pub fn stored_version(&self) -> u32 {
        self.stored_version
    }

    /// Whether a migration chain ran.
    pub fn migrated(&self) -> bool {
        self.stored_version != self.version
    }

    /// The payload bytes.
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// A reader over the payload.
    pub fn reader(&self) -> ChunkReader<'_> {
        ChunkReader::new(&self.data)
    }

    /// The payload, owned.
    pub fn into_data(self) -> Vec<u8> {
        self.data.into_owned()
    }
}

/// Decodes one device's state from its chunk.
///
/// The mirror of [`ChunkWriter`]: the same `read_*` calls in the same order.
/// Finish with [`ChunkReader::end`], which fails if the device read fewer
/// fields than were written — the cheapest possible detector for the classic
/// "someone added a field to `save` and forgot `load`" bug that invariant 6
/// exists to catch.
#[derive(Debug, Clone)]
pub struct ChunkReader<'a> {
    source: SliceSource<'a>,
}

impl<'a> ChunkReader<'a> {
    /// Read a chunk payload from `data`.
    pub const fn new(data: &'a [u8]) -> Self {
        ChunkReader {
            source: SliceSource::new(data),
        }
    }

    /// How many payload bytes have been consumed.
    pub const fn position(&self) -> usize {
        self.source.position()
    }

    /// Assert the whole chunk was consumed.
    ///
    /// A device that stops early has almost certainly forgotten a field, and
    /// finding out here beats finding out when the guest misbehaves a million
    /// cycles later.
    pub fn end(self) -> Result<()> {
        let left = self.source.remaining();
        if left == 0 {
            Ok(())
        } else {
            Err(state_error(format!(
                "{left} unread byte(s) left in chunk: the loader read fewer fields \
                 than the saver wrote"
            )))
        }
    }
}

impl<'a> Source<'a> for ChunkReader<'a> {
    fn take(&mut self, len: usize) -> Result<&'a [u8]> {
        self.source.take(len)
    }

    fn remaining(&self) -> usize {
        self.source.remaining()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// A machine shape used by most tests: two devices, one region.
    fn demo_shape() -> MachineShape {
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.demo").unwrap();
        shape.add_device("/mem/ram", "mem.ram").unwrap();
        shape.add_region("mem", "ram", 0x0000, 0x0800);
        shape.add_feature("std");
        shape.add_arch("demo16");
        shape
    }

    /// A snapshot with two chunks, written in whatever order `reversed` says.
    fn demo_snapshot(reversed: bool) -> Vec<u8> {
        let mut w = StateWriter::new(demo_shape());
        let order: [&str; 2] = if reversed {
            ["/mem/ram", "/cpu0"]
        } else {
            ["/cpu0", "/mem/ram"]
        };
        for path in order {
            match path {
                "/cpu0" => {
                    let mut c = w.chunk("/cpu0", "cpu.demo", 1).unwrap();
                    c.write_u16(0xfffc).unwrap();
                    c.write_u8(0b0010_0100).unwrap();
                }
                _ => {
                    let mut c = w.chunk("/mem/ram", "mem.ram", 1).unwrap();
                    c.write_bytes(&[0xa5; 16]).unwrap();
                }
            }
        }
        w.to_vec().unwrap()
    }

    // -- primitives ---------------------------------------------------------

    #[test]
    fn every_primitive_round_trips() {
        let mut buf: Vec<u8> = Vec::new();

        buf.write_u8(u8::MAX).unwrap();
        buf.write_u16(u16::MAX).unwrap();
        buf.write_u32(u32::MAX).unwrap();
        buf.write_u64(u64::MAX).unwrap();
        buf.write_u128(u128::MAX).unwrap();
        buf.write_i8(i8::MIN).unwrap();
        buf.write_i16(i16::MIN).unwrap();
        buf.write_i32(i32::MIN).unwrap();
        buf.write_i64(i64::MIN).unwrap();
        buf.write_i128(i128::MIN).unwrap();
        buf.write_u8(0).unwrap();
        buf.write_i32(-1).unwrap();
        buf.write_i64(0x0123_4567_89ab_cdef).unwrap();
        buf.write_bool(true).unwrap();
        buf.write_bool(false).unwrap();
        buf.write_bytes(&[]).unwrap();
        buf.write_bytes(&[0xde, 0xad, 0xbe, 0xef]).unwrap();
        buf.write_str("").unwrap();
        buf.write_str("/pci@0/nvme@1 — ünïcode ✓").unwrap();
        buf.write_seq_len(3).unwrap();
        for i in 0..3u32 {
            buf.write_u32(i).unwrap();
        }

        let mut r = SliceSource::new(&buf);
        assert_eq!(r.read_u8().unwrap(), u8::MAX);
        assert_eq!(r.read_u16().unwrap(), u16::MAX);
        assert_eq!(r.read_u32().unwrap(), u32::MAX);
        assert_eq!(r.read_u64().unwrap(), u64::MAX);
        assert_eq!(r.read_u128().unwrap(), u128::MAX);
        assert_eq!(r.read_i8().unwrap(), i8::MIN);
        assert_eq!(r.read_i16().unwrap(), i16::MIN);
        assert_eq!(r.read_i32().unwrap(), i32::MIN);
        assert_eq!(r.read_i64().unwrap(), i64::MIN);
        assert_eq!(r.read_i128().unwrap(), i128::MIN);
        assert_eq!(r.read_u8().unwrap(), 0);
        assert_eq!(r.read_i32().unwrap(), -1);
        assert_eq!(r.read_i64().unwrap(), 0x0123_4567_89ab_cdef);
        assert!(r.read_bool().unwrap());
        assert!(!r.read_bool().unwrap());
        assert_eq!(r.read_bytes().unwrap(), &[] as &[u8]);
        assert_eq!(r.read_bytes().unwrap(), &[0xde, 0xad, 0xbe, 0xef]);
        assert_eq!(r.read_str().unwrap(), "");
        assert_eq!(r.read_str().unwrap(), "/pci@0/nvme@1 — ünïcode ✓");
        assert_eq!(r.read_seq_len(4).unwrap(), 3);
        for i in 0..3u32 {
            assert_eq!(r.read_u32().unwrap(), i);
        }
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn integers_are_little_endian_whatever_the_host_is() {
        // The wire order is a format decision, not a host property: a snapshot
        // has to move between hosts of either endianness.
        let mut buf: Vec<u8> = Vec::new();
        buf.write_u32(0x1234_5678).unwrap();
        buf.write_i16(-2).unwrap();
        assert_eq!(buf, vec![0x78, 0x56, 0x34, 0x12, 0xfe, 0xff]);
    }

    #[test]
    fn a_bool_has_exactly_one_encoding() {
        let mut buf: Vec<u8> = Vec::new();
        buf.write_bool(true).unwrap();
        assert_eq!(buf, vec![1]);
        // Anything else is corruption, not a truthy value.
        for byte in [2u8, 0x80, 0xff] {
            let bytes = [byte];
            let err = SliceSource::new(&bytes).read_bool().unwrap_err();
            assert!(matches!(err, Error::State(_)), "{err:?}");
        }
    }

    #[test]
    fn a_sequence_count_is_bounded_by_the_bytes_present() {
        // The anti-OOM rule: a corrupt count must be refused before anything
        // allocates in proportion to it.
        let mut bytes: Vec<u8> = Vec::new();
        bytes.write_u64(u64::MAX).unwrap();
        let err = SliceSource::new(&bytes).read_seq_len(1).unwrap_err();
        assert!(matches!(err, Error::State(_)));

        let mut bytes: Vec<u8> = Vec::new();
        bytes.write_u64(1 << 62).unwrap();
        // 2^62 * 8 overflows u64: caught as an impossible encoding.
        let err = SliceSource::new(&bytes).read_seq_len(8).unwrap_err();
        assert!(matches!(err, Error::State(_)));
    }

    #[test]
    fn a_short_read_is_an_error_not_a_panic() {
        let bytes = [1u8, 2, 3];
        let mut src = SliceSource::new(&bytes);
        assert!(src.read_u64().is_err());
        assert_eq!(src.remaining(), 3, "a failed read consumes nothing");
        assert!(src.take(usize::MAX).is_err());
    }

    #[test]
    fn invalid_utf8_in_a_string_is_rejected() {
        let mut bytes: Vec<u8> = Vec::new();
        bytes.write_bytes(&[0xff, 0xfe]).unwrap();
        let err = SliceSource::new(&bytes).read_str().unwrap_err();
        assert!(matches!(err, Error::State(_)));
    }

    // -- determinism --------------------------------------------------------

    #[test]
    fn repeated_writes_are_byte_identical() {
        let mut w = StateWriter::new(demo_shape());
        w.chunk("/cpu0", "cpu.demo", 1)
            .unwrap()
            .write_u64(0x0123_4567_89ab_cdef)
            .unwrap();
        assert_eq!(w.to_vec().unwrap(), w.to_vec().unwrap());
    }

    #[test]
    fn save_order_is_not_observable_in_the_output() {
        // The whole point of buffering and sorting: a parallel or reordered
        // save must not produce a different snapshot of the same state.
        assert_eq!(demo_snapshot(false), demo_snapshot(true));
    }

    #[test]
    fn shape_building_order_is_not_observable_either() {
        let mut a = MachineShape::new();
        a.add_device("/b", "class.b").unwrap();
        a.add_device("/a", "class.a").unwrap();
        a.add_region("mem", "rom", 0x8000, 0x8000);
        a.add_region("mem", "ram", 0x0000, 0x0800);
        a.add_feature("z");
        a.add_feature("a");

        let mut b = MachineShape::new();
        b.add_device("/a", "class.a").unwrap();
        b.add_device("/b", "class.b").unwrap();
        b.add_region("mem", "ram", 0x0000, 0x0800);
        b.add_region("mem", "rom", 0x8000, 0x8000);
        b.add_feature("a");
        b.add_feature("z");
        // Adding the same region twice must not change the fingerprint.
        b.add_region("mem", "ram", 0x0000, 0x0800);

        assert_eq!(a, b);
        assert_eq!(
            StateWriter::new(a).to_vec().unwrap(),
            StateWriter::new(b).to_vec().unwrap()
        );
    }

    #[test]
    fn decoding_and_re_encoding_reproduces_the_bytes() {
        // The canonical-form claim: exactly one valid encoding per snapshot.
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        let mut w = StateWriter::new(r.shape().clone());
        for info in r.chunks() {
            let (class, version, data) = r.load_raw(info.path).unwrap();
            w.raw_chunk(info.path, class, version, data).unwrap();
        }
        assert_eq!(w.to_vec().unwrap(), bytes);
    }

    /// A committed fixture: a v1 snapshot, byte for byte.
    ///
    /// This pins the wire format. If a change to this module alters it, this
    /// test fails and the change becomes a deliberate format decision — with a
    /// [`FORMAT_VERSION`] bump — rather than a silent break of every snapshot
    /// anyone has saved. It is also the fixture the cross-version load test
    /// reads, per `ROADMAP.md` §4.5: a round-trip test can never exercise
    /// migration, because it only ever writes what it can already read.
    const FIXTURE_V1: &[u8] = &[
        0x52, 0x53, 0x45, 0x4d, 0x55, 0x53, 0x4e, 0x50, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x2f, 0x63, 0x70, 0x75, 0x30, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x63, 0x70, 0x75, 0x2e, 0x64, 0x65, 0x6d, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x63, 0x70, 0x75,
        0x30, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x70, 0x75, 0x2e, 0x64, 0x65,
        0x6d, 0x6f, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc,
        0xff, 0x24, 0x00,
    ];

    /// The state the fixture holds, written the way v1 of `cpu.demo` wrote it:
    /// a 16-bit PC and a flags byte.
    fn write_fixture_v1() -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.demo").unwrap();
        let mut w = StateWriter::new(shape);
        let mut c = w.chunk("/cpu0", "cpu.demo", 1).unwrap();
        c.write_u16(0xfffc).unwrap();
        c.write_u8(0x24).unwrap();
        w.to_vec().unwrap()
    }

    #[test]
    fn the_wire_format_matches_its_committed_fixture() {
        assert_eq!(write_fixture_v1(), FIXTURE_V1);
    }

    // -- shape checking -----------------------------------------------------

    #[test]
    fn a_matching_shape_loads() {
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        r.check_shape(&demo_shape()).unwrap();
        assert!(r.diff_shape(&demo_shape()).is_empty());
    }

    #[test]
    fn a_mismatched_shape_fails_with_a_diff_that_names_what_differs() {
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();

        // A machine that differs in every way the diff can describe.
        let mut machine = MachineShape::new();
        machine.add_device("/cpu0", "cpu.other").unwrap(); // class changed
        machine.add_device("/ppu", "nes.ppu").unwrap(); // extra device
        machine.add_region("mem", "ram", 0x2000, 0x0800); // region moved
        machine.add_region("io", "ports", 0x00, 0x100); // extra region
        machine.add_arch("demo16");
        // `/mem/ram` is missing, and feature `std` is missing.

        let diff = r.diff_shape(&machine);
        assert_eq!(diff.len(), 6, "{diff}");
        assert!(
            diff.differences()
                .contains(&ShapeDifference::DeviceMissing {
                    path: "/mem/ram".to_string(),
                    class: "mem.ram".to_string(),
                })
        );
        assert!(
            diff.differences()
                .contains(&ShapeDifference::DeviceClassChanged {
                    path: "/cpu0".to_string(),
                    snapshot_class: "cpu.demo".to_string(),
                    machine_class: "cpu.other".to_string(),
                })
        );

        let err = r.check_shape(&machine).unwrap_err();
        let Error::State(text) = err else {
            panic!("wrong error variant");
        };
        // The message has to name the things, not just count them.
        assert!(text.contains("/mem/ram"), "{text}");
        assert!(text.contains("/ppu"), "{text}");
        assert!(text.contains("cpu.other"), "{text}");
        assert!(text.contains("mem/ram"), "{text}");
        assert!(text.contains("0x2000"), "{text}");
        assert!(text.contains("std"), "{text}");
        assert!(text.contains("io/ports"), "{text}");
    }

    #[test]
    fn a_moved_region_reads_as_a_move_not_a_disappearance() {
        let mut snapshot = MachineShape::new();
        snapshot.add_region("mem", "ram", 0x0000, 0x0800);
        let mut machine = MachineShape::new();
        machine.add_region("mem", "ram", 0x1000, 0x0800);

        let diff = snapshot.diff(&machine);
        assert_eq!(diff.len(), 1);
        assert!(matches!(
            diff.differences()[0],
            ShapeDifference::RegionMoved { .. }
        ));
    }

    #[test]
    fn a_long_diff_is_summarised_rather_than_dumped() {
        let mut machine = MachineShape::new();
        for i in 0..(MAX_DIFF_LINES + 10) {
            machine.add_device(&format!("/dev{i:03}"), "d").unwrap();
        }
        let diff = MachineShape::new().diff(&machine);
        let text = diff.to_string();
        assert_eq!(diff.len(), MAX_DIFF_LINES + 10);
        assert!(text.contains("and 10 more"), "{text}");
        assert_eq!(text.lines().count(), MAX_DIFF_LINES + 2);
    }

    #[test]
    fn a_duplicate_instance_path_is_refused_by_the_shape() {
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.demo").unwrap();
        assert!(shape.add_device("/cpu0", "cpu.other").is_err());
    }

    // -- chunks -------------------------------------------------------------

    #[test]
    fn chunks_round_trip_by_path() {
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        let keys: Vec<(&str, &str, u32)> =
            r.chunks().map(|c| (c.path, c.class, c.version)).collect();
        assert_eq!(
            keys,
            vec![("/cpu0", "cpu.demo", 1), ("/mem/ram", "mem.ram", 1)]
        );

        let chunk = r.load("/cpu0", "cpu.demo", 1, &Migrations::new()).unwrap();
        assert!(!chunk.migrated());
        assert!(matches!(chunk.data, Cow::Borrowed(_)), "no needless copy");
        let mut cr = chunk.reader();
        assert_eq!(cr.read_u16().unwrap(), 0xfffc);
        assert_eq!(cr.read_u8().unwrap(), 0b0010_0100);
        cr.end().unwrap();
    }

    #[test]
    fn a_loader_that_stops_early_is_caught() {
        // Invariant 6's cheapest detector: a field added to `save` and
        // forgotten in `load` leaves bytes behind.
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        let chunk = r.load("/cpu0", "cpu.demo", 1, &Migrations::new()).unwrap();
        let mut cr = chunk.reader();
        assert_eq!(cr.read_u16().unwrap(), 0xfffc);
        let err = cr.end().unwrap_err();
        assert!(matches!(err, Error::State(_)));
    }

    #[test]
    fn a_second_chunk_for_one_path_is_refused() {
        let mut w = StateWriter::new(demo_shape());
        w.chunk("/cpu0", "cpu.demo", 1).unwrap();
        assert!(w.chunk("/cpu0", "cpu.demo", 1).is_err());
        assert!(w.chunk("", "cpu.demo", 1).is_err());
    }

    #[test]
    fn a_missing_chunk_says_what_is_there_instead() {
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        let err = r
            .load("/apu", "apu.demo", 1, &Migrations::new())
            .unwrap_err();
        let Error::State(text) = err else {
            panic!("wrong error variant");
        };
        assert!(text.contains("/apu"), "{text}");
        assert!(text.contains("/cpu0"), "{text}");
    }

    #[test]
    fn a_chunk_written_by_a_different_class_is_refused() {
        let bytes = demo_snapshot(false);
        let r = StateReader::new(&bytes).unwrap();
        let err = r
            .load("/cpu0", "cpu.mos6502", 1, &Migrations::new())
            .unwrap_err();
        let Error::State(text) = err else {
            panic!("wrong error variant");
        };
        assert!(text.contains("cpu.mos6502"), "{text}");
        assert!(text.contains("cpu.demo"), "{text}");
    }

    #[test]
    fn an_empty_snapshot_round_trips() {
        let w = StateWriter::new(MachineShape::new());
        assert!(w.is_empty());
        let bytes = w.to_vec().unwrap();
        let r = StateReader::new(&bytes).unwrap();
        assert_eq!(r.chunks().count(), 0);
        assert!(r.shape().is_empty());
        r.check_shape(&MachineShape::new()).unwrap();
    }

    // -- migration ----------------------------------------------------------

    /// `cpu.demo` v1 -> v2: a cycle counter was added; old snapshots start at 0.
    fn demo_v1_to_v2(src: &mut ChunkReader<'_>, dst: &mut Vec<u8>) -> Result<()> {
        let pc = src.read_u16()?;
        let flags = src.read_u8()?;
        dst.write_u16(pc)?;
        dst.write_u8(flags)?;
        dst.write_u32(0)?;
        Ok(())
    }

    /// `cpu.demo` v2 -> v3: the PC was widened to 32 bits.
    fn demo_v2_to_v3(src: &mut ChunkReader<'_>, dst: &mut Vec<u8>) -> Result<()> {
        let pc = src.read_u16()?;
        let flags = src.read_u8()?;
        let cycles = src.read_u32()?;
        dst.write_u32(u32::from(pc))?;
        dst.write_u8(flags)?;
        dst.write_u32(cycles)?;
        Ok(())
    }

    fn demo_migrations() -> Migrations {
        let mut m = Migrations::new();
        m.register("cpu.demo", 1, demo_v1_to_v2).unwrap();
        m.register("cpu.demo", 2, demo_v2_to_v3).unwrap();
        m
    }

    #[test]
    fn a_v1_fixture_loads_into_a_v2_build() {
        // The cross-version load §4.5 asks for, from the committed fixture.
        let r = StateReader::new(FIXTURE_V1).unwrap();
        let mut migrations = Migrations::new();
        migrations.register("cpu.demo", 1, demo_v1_to_v2).unwrap();

        let chunk = r.load("/cpu0", "cpu.demo", 2, &migrations).unwrap();
        assert!(chunk.migrated());
        assert_eq!(chunk.stored_version(), 1);
        assert_eq!(chunk.version(), 2);

        let mut cr = chunk.reader();
        assert_eq!(cr.read_u16().unwrap(), 0xfffc);
        assert_eq!(cr.read_u8().unwrap(), 0x24);
        assert_eq!(cr.read_u32().unwrap(), 0, "new field defaults");
        cr.end().unwrap();
    }

    #[test]
    fn a_v1_fixture_walks_the_whole_chain_to_v3() {
        let r = StateReader::new(FIXTURE_V1).unwrap();
        let chunk = r.load("/cpu0", "cpu.demo", 3, &demo_migrations()).unwrap();
        assert_eq!((chunk.stored_version(), chunk.version()), (1, 3));
        let mut cr = chunk.reader();
        assert_eq!(cr.read_u32().unwrap(), 0x0000_fffc, "PC widened");
        assert_eq!(cr.read_u8().unwrap(), 0x24);
        assert_eq!(cr.read_u32().unwrap(), 0);
        cr.end().unwrap();
    }

    #[test]
    fn a_hole_in_the_chain_names_the_gap_and_the_steps_that_exist() {
        let r = StateReader::new(FIXTURE_V1).unwrap();
        let mut migrations = Migrations::new();
        migrations.register("cpu.demo", 1, demo_v1_to_v2).unwrap();
        // No v2 -> v3 step registered.
        let err = r.load("/cpu0", "cpu.demo", 3, &migrations).unwrap_err();
        let Error::State(text) = err else {
            panic!("wrong error variant");
        };
        assert!(text.contains("v2 to v3"), "{text}");
        assert!(text.contains("v1->v2"), "{text}");
        assert!(text.contains("cpu.demo"), "{text}");
    }

    #[test]
    fn a_chunk_from_a_newer_build_is_refused_rather_than_guessed_at() {
        let r = StateReader::new(FIXTURE_V1).unwrap();
        let err = r
            .load("/cpu0", "cpu.demo", 0, &demo_migrations())
            .unwrap_err();
        let Error::State(text) = err else {
            panic!("wrong error variant");
        };
        assert!(text.contains("never downgraded"), "{text}");
    }

    #[test]
    fn a_migration_step_cannot_be_registered_twice() {
        let mut m = Migrations::new();
        m.register("cpu.demo", 1, demo_v1_to_v2).unwrap();
        assert!(m.register("cpu.demo", 1, demo_v1_to_v2).is_err());
        assert!(m.register("cpu.demo", u32::MAX, demo_v1_to_v2).is_err());
        assert_eq!(m.steps_for("cpu.demo"), vec![1]);
        assert!(m.steps_for("cpu.unknown").is_empty());
    }

    #[test]
    fn a_migration_that_hits_truncated_input_fails_instead_of_panicking() {
        // Migrations are parsers too: they run on bytes from the same file.
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.demo").unwrap();
        let mut w = StateWriter::new(shape);
        w.raw_chunk("/cpu0", "cpu.demo", 1, &[0x01]).unwrap(); // half a u16
        let bytes = w.to_vec().unwrap();
        let r = StateReader::new(&bytes).unwrap();
        let err = r.load("/cpu0", "cpu.demo", 2, &demo_migrations());
        assert!(matches!(err, Err(Error::State(_))));
    }

    // -- hostile input ------------------------------------------------------

    #[test]
    fn every_truncation_is_rejected_without_panicking() {
        // Fuzz-shaped: feed the parser every prefix of a valid snapshot. None
        // of them is a valid snapshot, and none may panic.
        let bytes = demo_snapshot(false);
        for len in 0..bytes.len() {
            let result = StateReader::new(&bytes[..len]);
            assert!(
                result.is_err(),
                "prefix of {len} byte(s) parsed as a snapshot"
            );
        }
        assert!(StateReader::new(&bytes).is_ok());
    }

    #[test]
    fn every_single_byte_corruption_is_survived() {
        // Some corruptions land in a payload or a name and stay valid; the
        // requirement is not that they fail, it is that nothing panics and that
        // whatever comes back can be walked without panicking either.
        let original = demo_snapshot(false);
        for i in 0..original.len() {
            for mask in [0x01u8, 0x40, 0x80, 0xff] {
                let mut bytes = original.clone();
                bytes[i] ^= mask;
                let Ok(reader) = StateReader::new(&bytes) else {
                    continue;
                };
                // Walk everything a caller would: keys, raw payloads, loads,
                // a shape diff, and a read off the front of each chunk.
                let _ = reader.diff_shape(&demo_shape()).to_string();
                for info in reader.chunks() {
                    let _ = reader.load_raw(info.path).unwrap();
                    let loaded = reader
                        .load(info.path, info.class, info.version, &Migrations::new())
                        .unwrap();
                    let mut cr = loaded.reader();
                    let _ = cr.read_u64();
                    let _ = cr.read_str();
                    let _ = cr.read_bool();
                }
            }
        }
    }

    #[test]
    fn truncations_of_a_corrupt_snapshot_are_survived_too() {
        // Truncation and corruption combined, which is what a half-written file
        // on a full disk actually looks like.
        let original = demo_snapshot(false);
        for i in (0..original.len()).step_by(3) {
            let mut bytes = original.clone();
            bytes[i] ^= 0xff;
            for len in 0..bytes.len() {
                let _ = StateReader::new(&bytes[..len]);
            }
        }
    }

    #[test]
    fn arbitrary_bytes_are_never_mistaken_for_a_snapshot() {
        assert!(StateReader::new(&[]).is_err());
        assert!(StateReader::new(&[0u8; 64]).is_err());
        assert!(StateReader::new(&[0xffu8; 4096]).is_err());
        let mut counting = Vec::new();
        for i in 0..1024u32 {
            counting.push((i % 251) as u8);
        }
        assert!(StateReader::new(&counting).is_err());
    }

    /// Hand-build a header so the tests can produce inputs the writer cannot.
    fn handcrafted_header(format: u32, codec: u16, integrity: u16) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.write_all(&MAGIC).unwrap();
        bytes.write_u32(format).unwrap();
        bytes.write_u16(codec).unwrap();
        bytes.write_u16(integrity).unwrap();
        // An empty shape: no devices, regions, features or arches.
        for _ in 0..4 {
            bytes.write_seq_len(0).unwrap();
        }
        bytes
    }

    #[test]
    fn the_header_is_checked_field_by_field() {
        let mut wrong_magic = handcrafted_header(FORMAT_VERSION, 0, 0);
        wrong_magic[0] = b'Q';
        wrong_magic.write_u8(TAG_END).unwrap();
        let Err(Error::State(text)) = StateReader::new(&wrong_magic) else {
            panic!("bad magic accepted");
        };
        assert!(text.contains("not a snapshot"), "{text}");

        let mut future = handcrafted_header(FORMAT_VERSION + 1, 0, 0);
        future.write_u8(TAG_END).unwrap();
        let Err(Error::State(text)) = StateReader::new(&future) else {
            panic!("future format accepted");
        };
        assert!(text.contains("container format"), "{text}");

        // The compcol seam: an old reader must say so, not parse garbage.
        let mut compressed = handcrafted_header(FORMAT_VERSION, Codec::ZSTD.0, 0);
        compressed.write_u8(TAG_END).unwrap();
        let Err(Error::State(text)) = StateReader::new(&compressed) else {
            panic!("unknown codec accepted");
        };
        assert!(text.contains("compcol"), "{text}");

        // The purecrypto seam, likewise.
        let mut signed = handcrafted_header(FORMAT_VERSION, 0, Integrity::BLAKE3.0);
        signed.write_u8(TAG_END).unwrap();
        let Err(Error::State(text)) = StateReader::new(&signed) else {
            panic!("unknown integrity scheme accepted");
        };
        assert!(text.contains("purecrypto"), "{text}");
    }

    #[test]
    fn non_canonical_encodings_are_rejected() {
        // Chunks out of path order.
        let mut bytes = handcrafted_header(FORMAT_VERSION, 0, 0);
        for path in ["/b", "/a"] {
            bytes.write_u8(TAG_CHUNK).unwrap();
            bytes.write_str(path).unwrap();
            bytes.write_str("c").unwrap();
            bytes.write_u32(1).unwrap();
            bytes.write_bytes(&[]).unwrap();
        }
        bytes.write_u8(TAG_END).unwrap();
        let Err(Error::State(text)) = StateReader::new(&bytes) else {
            panic!("out-of-order chunks accepted");
        };
        assert!(text.contains("out of order"), "{text}");

        // The same path twice is the same rule: ascending means strictly.
        let mut bytes = handcrafted_header(FORMAT_VERSION, 0, 0);
        for _ in 0..2 {
            bytes.write_u8(TAG_CHUNK).unwrap();
            bytes.write_str("/a").unwrap();
            bytes.write_str("c").unwrap();
            bytes.write_u32(1).unwrap();
            bytes.write_bytes(&[]).unwrap();
        }
        bytes.write_u8(TAG_END).unwrap();
        assert!(StateReader::new(&bytes).is_err());

        // An unknown framing tag.
        let mut bytes = handcrafted_header(FORMAT_VERSION, 0, 0);
        bytes.write_u8(0x7f).unwrap();
        let Err(Error::State(text)) = StateReader::new(&bytes) else {
            panic!("unknown tag accepted");
        };
        assert!(text.contains("unknown chunk tag"), "{text}");

        // Trailing bytes after the end marker.
        let mut bytes = handcrafted_header(FORMAT_VERSION, 0, 0);
        bytes.write_u8(TAG_END).unwrap();
        bytes.write_all(b"junk").unwrap();
        let Err(Error::State(text)) = StateReader::new(&bytes) else {
            panic!("trailing bytes accepted");
        };
        assert!(text.contains("trailing"), "{text}");
    }

    #[test]
    fn out_of_order_header_entries_are_rejected() {
        let mut bytes = Vec::new();
        bytes.write_all(&MAGIC).unwrap();
        bytes.write_u32(FORMAT_VERSION).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_seq_len(2).unwrap();
        bytes.write_str("/b").unwrap();
        bytes.write_str("c").unwrap();
        bytes.write_str("/a").unwrap();
        bytes.write_str("c").unwrap();
        let Err(Error::State(text)) = StateReader::new(&bytes) else {
            panic!("out-of-order devices accepted");
        };
        assert!(text.contains("out of order"), "{text}");

        // Features, same rule.
        let mut bytes = Vec::new();
        bytes.write_all(&MAGIC).unwrap();
        bytes.write_u32(FORMAT_VERSION).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_seq_len(0).unwrap();
        bytes.write_seq_len(0).unwrap();
        bytes.write_seq_len(2).unwrap();
        bytes.write_str("z").unwrap();
        bytes.write_str("a").unwrap();
        assert!(StateReader::new(&bytes).is_err());
    }

    #[test]
    fn an_absurd_length_field_is_refused_before_anything_allocates() {
        let mut bytes = handcrafted_header(FORMAT_VERSION, 0, 0);
        bytes.write_u8(TAG_CHUNK).unwrap();
        bytes.write_str("/a").unwrap();
        bytes.write_str("c").unwrap();
        bytes.write_u32(1).unwrap();
        bytes.write_u64(u64::MAX).unwrap(); // payload length
        assert!(StateReader::new(&bytes).is_err());

        // A device count no input could satisfy.
        let mut bytes = Vec::new();
        bytes.write_all(&MAGIC).unwrap();
        bytes.write_u32(FORMAT_VERSION).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_u16(0).unwrap();
        bytes.write_u64(u64::MAX / 2).unwrap();
        assert!(StateReader::new(&bytes).is_err());
    }

    #[test]
    fn the_slice_source_reports_progress_honestly() {
        let bytes = [0u8; 8];
        let mut src = SliceSource::new(&bytes);
        assert_eq!((src.position(), src.remaining()), (0, 8));
        src.read_u32().unwrap();
        assert_eq!((src.position(), src.remaining()), (4, 4));
        assert!(src.read_u64().is_err());
        assert_eq!((src.position(), src.remaining()), (4, 4));
    }

    #[test]
    fn a_sink_reference_is_a_sink() {
        // So a caller can keep its buffer and still pass it to `write_to`.
        let mut buf: Vec<u8> = Vec::new();
        let w = StateWriter::new(MachineShape::new());
        w.write_to(&mut &mut buf).unwrap();
        assert_eq!(buf, w.to_vec().unwrap());
    }

    #[test]
    fn a_chunk_writer_reports_its_size() {
        let mut w = StateWriter::new(MachineShape::new());
        let mut c = w.chunk("/a", "c", 1).unwrap();
        assert!(c.is_empty());
        c.write_u32(0).unwrap();
        assert_eq!(c.len(), 4);
        assert_eq!(w.len(), 1);
    }

    #[test]
    fn errors_are_all_the_state_variant() {
        // §4.5 errors go through `Error::State`, so a caller can match on one
        // thing rather than guess.
        let bytes = demo_snapshot(false);
        let candidates = [
            StateReader::new(&bytes[..4]).err(),
            StateReader::new(&bytes)
                .unwrap()
                .check_shape(&MachineShape::new())
                .err(),
            StateReader::new(&bytes)
                .unwrap()
                .load("/nope", "c", 1, &Migrations::new())
                .err(),
            MachineShape::new()
                .add_device("", "")
                .ok()
                .and(None::<Error>),
        ];
        for candidate in candidates.into_iter().flatten() {
            assert!(matches!(candidate, Error::State(_)), "{candidate:?}");
        }
    }
}
