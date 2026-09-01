//! A disk image file as a drive's [`Medium`].
//!
//! The whole of the adapter. `fstool` opens the file and understands the
//! format; this decides what a guest is told when the host says no, and what a
//! snapshot of a file means.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use core::fmt;
use std::path::{Path, PathBuf};

use fstool::block::{BlockDevice, CreateOpts, FileBackend, Qcow2Backend};

use crate::core::error::{BusError, Result};
use crate::core::space::MemResult;
use crate::core::sync::{LockRank, Mutex};
use crate::dev::ata::{Medium, Snapshot};

use super::{config_error, media_error};

/// How to open an image.
///
/// `Default` is what `--drive hd0=disk.qcow2` means: open the existing file
/// read-write, let `fstool` pick the backend from the file's own contents, and
/// snapshot by reference.
#[derive(Debug, Clone)]
pub struct ImageOptions {
    /// Open the file `O_RDONLY` and refuse every guest write. The drive
    /// advertises itself write protected, so a guest finds out from `IDENTIFY`
    /// rather than from a failed command.
    pub read_only: bool,
    /// What a machine snapshot does about the bytes. See [`Snapshot`];
    /// [`Snapshot::Reference`] is the default and the only one that is honest
    /// about a large image.
    pub snapshot: Snapshot,
    /// Create the image instead of opening it, with this capacity in bytes.
    ///
    /// The backend follows the extension, as `fstool` defines it: `.qcow2` /
    /// `.qcow` / `.q2` make a qcow2, anything else a sparse raw file. An
    /// existing file at the path is replaced.
    pub create: Option<u64>,
    /// The passphrase for an encrypted container — a LUKS volume, or a qcow2
    /// with either `crypt_method`. Ignored for anything unencrypted.
    pub password: Option<String>,
    /// The cluster size a newly created qcow2 allocates in, in bytes. A power
    /// of two, at least 512; zero takes `fstool`'s default of 64 KiB, which is
    /// what `qemu-img` uses.
    ///
    /// It is the granularity of allocate-on-write, so it trades metadata size
    /// against how much a one-sector write costs. Ignored for a raw image,
    /// which has no metadata to size.
    pub cluster: u32,
}

impl Default for ImageOptions {
    fn default() -> ImageOptions {
        ImageOptions {
            read_only: false,
            snapshot: Snapshot::Reference,
            create: None,
            password: None,
            cluster: 0,
        }
    }
}

impl ImageOptions {
    /// Defaults.
    #[must_use]
    pub fn new() -> ImageOptions {
        ImageOptions::default()
    }

    /// Open read-only.
    #[must_use]
    pub fn read_only(mut self, yes: bool) -> ImageOptions {
        self.read_only = yes;
        self
    }

    /// Choose the snapshot policy.
    #[must_use]
    pub fn snapshot(mut self, policy: Snapshot) -> ImageOptions {
        self.snapshot = policy;
        self
    }

    /// Create an image of `bytes` rather than opening one.
    #[must_use]
    pub fn create(mut self, bytes: u64) -> ImageOptions {
        self.create = Some(bytes);
        self
    }

    /// Supply a passphrase for an encrypted container.
    #[must_use]
    pub fn password(mut self, password: impl Into<String>) -> ImageOptions {
        self.password = Some(password.into());
        self
    }

    /// Set the cluster size a newly created qcow2 allocates in.
    #[must_use]
    pub fn cluster(mut self, bytes: u32) -> ImageOptions {
        self.cluster = bytes;
        self
    }
}

/// A host disk image behind a drive.
///
/// Shareable (`Send + Sync`) and used through `&self`, which is what the device
/// contract needs and what `fstool::BlockDevice` — `Send`, not `Sync`, and
/// `&mut self` throughout — does not give. The lock is what bridges the two.
pub struct Image {
    /// What it is, for a diagnostic and for a [`Snapshot::Reference`] chunk.
    describe: String,
    capacity: u64,
    read_only: bool,
    policy: Snapshot,
    /// [`LockRank::LEAF`]: the drive holds its own [`LockRank::DEVICE`] state
    /// lock across a sector transfer, so this has to nest *under* that, and
    /// nothing is ever locked while it is held. The critical section is one
    /// positional read or write with no outward call in it.
    device: Mutex<Box<dyn BlockDevice>>,
}

impl fmt::Debug for Image {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Image")
            .field("describe", &self.describe)
            .field("capacity", &self.capacity)
            .field("read_only", &self.read_only)
            .field("snapshot", &self.policy)
            .finish_non_exhaustive()
    }
}

impl Image {
    /// Open (or create) the image at `path`.
    ///
    /// **This is the outward action, and it happens before any device exists.**
    /// A run opens the image and installs it with
    /// [`blk::install`](super::install); the drive's `new(props)` then takes it
    /// out of a slot, which is rendezvous rather than I/O. Two-phase
    /// construction is not bent to make a file-backed drive work.
    ///
    /// # Errors
    ///
    /// [`Error::Config`](crate::core::error::Error::Config) if the file cannot
    /// be opened, the format is one `fstool` refuses (an encrypted image with
    /// no passphrase, a qcow2 whose backing file is missing), the image is
    /// empty, or its capacity is not a whole number of 512-byte sectors — which
    /// is not a drive an ATA host could address.
    pub fn open(path: &Path, opts: &ImageOptions) -> Result<Image> {
        let shown = path.display().to_string();
        let device = open_device(path, opts).map_err(|e| config_error(&shown, &e))?;
        // The canonical path, so that two names for one file compare equal in a
        // snapshot reference. A path that cannot be canonicalised (it was just
        // created, on a filesystem that dislikes it) falls back to what was
        // typed, which is still stable for the life of the run.
        let resolved = std::fs::canonicalize(path).unwrap_or_else(|_| PathBuf::from(path));
        let describe = alloc::format!(
            "{} {} {}",
            format_of(path),
            resolved.display(),
            device.total_size()
        );
        Image::wrap(describe, device, opts)
    }

    /// Build an image over a block device the caller already has.
    ///
    /// For a test that wants `fstool`'s `MemoryBackend`, its `CrashInject`
    /// fault injector, or a `SlicedBackend` over a partition — and for the fuzz
    /// target, which must not touch the filesystem on every iteration.
    /// `describe` is the identity a [`Snapshot::Reference`] chunk records, so it
    /// has to name this device and no other.
    ///
    /// # Errors
    ///
    /// As [`Image::open`], for the capacity checks.
    pub fn from_device(
        describe: impl Into<String>,
        device: Box<dyn BlockDevice>,
        opts: &ImageOptions,
    ) -> Result<Image> {
        Image::wrap(describe.into(), device, opts)
    }

    fn wrap(describe: String, device: Box<dyn BlockDevice>, opts: &ImageOptions) -> Result<Image> {
        let capacity = device.total_size();
        if capacity == 0 {
            return Err(config_error(
                &describe,
                &fstool::Error::InvalidArgument(
                    "the image is empty; a drive holds at least one sector".to_string(),
                ),
            ));
        }
        if !capacity.is_multiple_of(crate::dev::ata::disk::SECTOR) {
            return Err(config_error(
                &describe,
                &fstool::Error::InvalidArgument(alloc::format!(
                    "{capacity} bytes is not a whole number of 512-byte sectors"
                )),
            ));
        }
        Ok(Image {
            describe,
            capacity,
            read_only: opts.read_only,
            policy: opts.snapshot,
            device: Mutex::with_rank(LockRank::LEAF, device),
        })
    }

    /// What the image is: format, resolved path and capacity.
    #[must_use]
    pub fn describe(&self) -> &str {
        &self.describe
    }

    /// The advisory logical sector size the backend reports.
    #[must_use]
    pub fn block_size(&self) -> u32 {
        self.device.lock().block_size()
    }

    /// The range check, in `u64` and before the offset ever becomes a host
    /// `usize` — a 64-bit guest on a 32-bit host still has 64-bit sectors.
    fn bounds(&self, offset: u64, len: u64) -> MemResult {
        let end = offset.checked_add(len).ok_or(BusError::BadAccess)?;
        if end > self.capacity {
            // Not on this medium: an address the drive should not have asked
            // for, or an image that shrank under it.
            return Err(BusError::BadAccess);
        }
        Ok(())
    }
}

impl Medium for Image {
    fn capacity(&self) -> u64 {
        self.capacity
    }

    fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        self.bounds(offset, dst.len() as u64)?;
        if dst.is_empty() {
            return Ok(());
        }
        self.device
            .lock()
            .read_at(offset, dst)
            // Past the bounds check, an `OutOfBounds` from the backend is a
            // corrupt image rather than a sector the drive does not have.
            .map_err(|e| media_error(&e))
    }

    fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        if self.read_only {
            return Err(BusError::Protected);
        }
        self.bounds(offset, src.len() as u64)?;
        if src.is_empty() {
            return Ok(());
        }
        self.device
            .lock()
            .write_at(offset, src)
            .map_err(|e| media_error(&e))
    }

    fn flush(&self) -> MemResult {
        if self.read_only {
            // Nothing was written, so there is nothing to make durable, and
            // reporting a failure would make a guest's barrier look broken.
            return Ok(());
        }
        self.device.lock().sync().map_err(|e| media_error(&e))
    }

    fn is_read_only(&self) -> bool {
        self.read_only
    }

    fn snapshot(&self) -> Snapshot {
        self.policy
    }

    fn describe(&self) -> String {
        self.describe.clone()
    }
}

/// Open the backend `fstool` picks for this file, or create one.
fn open_device(path: &Path, opts: &ImageOptions) -> fstool::Result<Box<dyn BlockDevice>> {
    if let Some(size) = opts.create {
        if opts.read_only {
            return Err(fstool::Error::InvalidArgument(
                "an image cannot be both created and read-only".to_string(),
            ));
        }
        let mut create = CreateOpts::default();
        if opts.cluster != 0 {
            if !opts.cluster.is_power_of_two() || opts.cluster < 512 {
                return Err(fstool::Error::InvalidArgument(alloc::format!(
                    "a qcow2 cluster is a power of two of at least 512 bytes, not {}",
                    opts.cluster
                )));
            }
            create.cluster_size = opts.cluster;
        }
        return fstool::block::create_image(path, size, &create);
    }
    let password = opts.password.as_deref();
    if opts.read_only {
        fstool::block::open_image_read_only_with_password(path, password)
    } else {
        fstool::block::open_image_with_password(path, password)
    }
}

/// The name of the container format, for the identity string.
///
/// A label, not a parse: every one of these is `fstool`'s own probe, and the
/// backend that actually reads the file is chosen by `fstool` from the same
/// probes. Anything unrecognised is a flat image, which is what a raw file is.
fn format_of(path: &Path) -> &'static str {
    if Qcow2Backend::probe(path).unwrap_or(false) {
        return "qcow2";
    }
    if fstool::block::dmg::probe(path).unwrap_or(false) {
        return "dmg";
    }
    if fstool::block::diskcopy::probe(path).unwrap_or(false) {
        return "diskcopy42";
    }
    "raw"
}

/// A sparse raw image of `bytes` at `path`, replacing whatever is there.
///
/// `fstool`'s `FileBackend::create` does this with `set_len`, so the file is a
/// hole: a 16 GiB drive costs no disk until the guest writes to it. Exposed
/// because "make me a blank disk" is what a test and a first run both want, and
/// because it is the sparse answer the media-slot path cannot give.
///
/// # Errors
///
/// [`Error::Config`](crate::core::error::Error::Config) if the file cannot be
/// created.
pub fn create_raw(path: &Path, bytes: u64) -> Result<()> {
    FileBackend::create(path, bytes)
        .map(|_| ())
        .map_err(|e| config_error(&path.display().to_string(), &e))
}

#[cfg(test)]
mod tests;
