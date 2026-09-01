//! Real disk images: a guest drive that is a *file* rather than a buffer.
//!
//! One of the two documented `std` exceptions to the `no_std` rule
//! (`CLAUDE.md`), because [`fstool`](https://github.com/KarpelesLab/fstool) is a
//! `std` crate. Everything here is behind the `dev-blk` feature, so
//! `--no-default-features` builds the emulation core without it and the drive
//! keeps working on its media slot.
//!
//! # What this is, and what it deliberately is not
//!
//! It is an **adapter**, about a hundred lines of one. `fstool` already owns
//! the layer below: `BlockDevice` (`Read + Write + Seek + Send`, positional
//! `read_at`/`write_at`, `total_size`, `sync`, `zero_range`), a sparse
//! file backend, an in-memory backend, a sub-range view, crash injection, and
//! the image formats — **qcow2** (v2 and v3, read/write, allocate-on-write,
//! compressed clusters, backing files, encryption), UDIF **DMG**, DiskCopy 4.2
//! and **LUKS**. `ROADMAP.md` §7.1 is explicit that emulated controllers sit on
//! `fstool::BlockDevice` "rather than on a parallel rsemu invention", so no
//! image format is parsed in this module and none should ever be.
//!
//! What is here is the *impedance match* between that trait and
//! [`ata::Medium`](crate::dev::ata::Medium):
//!
//! ```text
//!   AtaDisk ──► Medium::read_at(&self, u64, &mut [u8]) -> MemResult
//!                      │  Mutex (core::sync)      map fstool::Error -> BusError
//!                      ▼
//!               fstool::BlockDevice::read_at(&mut self, u64, &mut [u8])
//!                      │
//!                      ▼  FileBackend / Qcow2Backend / DmgBackend / LuksBackend
//!                    the host file
//! ```
//!
//! Three things that adapter has to get right, and they are the reason it
//! exists at all rather than the drive holding a `BlockDevice` directly:
//!
//! * **`&mut self` to `&self`.** `fstool::BlockDevice` is `Send` and *not*
//!   `Sync`, and its methods take `&mut self`. A device is `Send + Sync` with
//!   synchronous methods from the first commit (`CLAUDE.md`), so the device owns
//!   its image behind a lock rather than sharing it. The lock is
//!   [`core::sync::Mutex`](crate::core::sync), never `std::sync` — nothing
//!   under `dev/` may name that, `std` gate or no `std` gate.
//! * **`std::io::Error` to [`BusError`].** A short
//!   read, a torn write, an image that shrank and a full filesystem are
//!   different failures and the guest is told so: see [`bus_error`].
//! * **Snapshots.** A file-backed drive references its image rather than
//!   copying it — [`Snapshot`](crate::dev::ata::Snapshot) has the argument.
//!
//! # Time
//!
//! **A host read takes zero guest time**, exactly as it does with a `RamStore`,
//! and that is a determinism requirement rather than an omission: if the
//! duration of a `pread` reached the guest's timeline, two runs of the same
//! machine would diverge on how warm the host's page cache was. The drive
//! models no I/O delay at all (`dev::ata::disk`, "Time"); when it grows one it
//! will come from a clock domain and a scheduler event, and the host's actual
//! latency will still not be it.
//!
//! Nothing here reads the wall clock, sleeps, or spawns a thread.
//!
//! # How a run reaches it
//!
//! Through the media slot the machine file already names, not through a new
//! property:
//!
//! ```text
//!   machine file:   object hd0 "ata.disk" { image = "hd0", bay = "ide0-master" }
//!   the run:        rsemu run pc-at --bios bios.bin --drive hd0=disk.qcow2
//!                   └─► blk::install(hosts, "hd0", Image::open(…)?)
//!   construction:   ata.disk finds a Medium waiting under "hd0" and uses it
//! ```
//!
//! `--hd0 disk.img` still binds the media slot to bytes and still copies them
//! into RAM. **Both work**; they are different contracts and neither is a
//! degraded version of the other. A machine file never holds a host path,
//! because a machine file is portable data describing a board.
//!
//! # Provenance
//!
//! No image format is implemented here, so there is no on-disk structure to
//! cite. The formats come from `fstool` (Karpelès Lab, MIT), which is a
//! permitted first-party dependency by name in `CLAUDE.md`. **No QEMU source
//! was opened for this module, its docs included.**

use alloc::string::String;
use alloc::sync::Arc;

use crate::core::error::{BusError, Error, Result};
use crate::core::hosts::HostObjects;
use crate::dev::ata::Medium;

mod image;

pub use image::{Image, ImageOptions, create_raw};

/// Turn an `fstool` failure into the bus error that describes it.
///
/// The mapping the guest ultimately sees as an ATA error code, so it is written
/// down once — [`ata::medium`](crate::dev::ata::medium) has the other half of
/// the table:
///
/// | `fstool::Error` | [`BusError`] | Why |
/// | --- | --- | --- |
/// | `OutOfBounds` | `BadAccess` | there is no such sector on this medium |
/// | `Io(PermissionDenied)`, `Immutable` | `Protected` | the medium is there and refuses this direction |
/// | `Io(WouldBlock)`, `Io(Interrupted)` | `Retry` | nothing has happened yet, so retrying is legal |
/// | `Io(UnexpectedEof)` | `Unassigned` | a short read: the sector exists and the bytes did not arrive |
/// | anything else | `Unassigned` | an uncorrectable data error |
///
/// `Retry` is only produced for the two `io::ErrorKind`s that are defined to
/// mean "no progress was made". Anything that might have partially written is
/// `Unassigned` instead, because the dispatcher's rule — a retry must not
/// re-run a half-completed access — applies just as much to a disk sector as to
/// a memory region.
///
/// # `OutOfBounds` means two different things, and only one of them is here
///
/// [`Image`] range-checks every access against its own capacity *before* the
/// backend sees it, so an `OutOfBounds` that comes back from a read the drive
/// was allowed to make is not "no such sector" — the sector is on the drive.
/// It is a **corrupt image**: an L2 entry pointing past the end of the file, a
/// truncated qcow2, a backing chain that lost its base. That is an
/// uncorrectable data error, and [`Image`] maps it that way rather than telling
/// a guest its own geometry is wrong. This function is the table for a caller
/// that has *not* already bounds-checked.
#[must_use]
pub fn bus_error(e: &fstool::Error) -> BusError {
    match e {
        fstool::Error::OutOfBounds { .. } => BusError::BadAccess,
        fstool::Error::Immutable { .. } => BusError::Protected,
        fstool::Error::Io(io) => match io.kind() {
            std::io::ErrorKind::PermissionDenied => BusError::Protected,
            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted => BusError::Retry,
            _ => BusError::Unassigned,
        },
        _ => BusError::Unassigned,
    }
}

/// [`bus_error`], for a caller that has already range-checked the access.
///
/// The only difference is `OutOfBounds`, which can no longer mean "off the end
/// of the drive" and therefore means the image's own metadata disagrees with
/// its size: an uncorrectable data error. See [`bus_error`]'s last section.
pub(crate) fn media_error(e: &fstool::Error) -> BusError {
    match bus_error(e) {
        BusError::BadAccess => BusError::Unassigned,
        other => other,
    }
}

/// Turn an `fstool` failure into a configuration error naming `path`.
///
/// For the open path, where there is no guest to tell and a person to tell
/// instead.
pub(crate) fn config_error(path: &str, e: &fstool::Error) -> Error {
    Error::Config {
        at: String::from("dev.blk"),
        message: alloc::format!("{path}: {e}"),
    }
}

/// Install `image` as the medium for the drive that names media slot `slot`.
///
/// What `rsemu run … --drive hd0=disk.qcow2` calls. The drive picks it up when
/// it is constructed; nothing about the machine description changes.
///
/// # Errors
///
/// [`Error::Config`] if another kind of host object already holds that name, or
/// if a medium is already waiting there — two drives writing one image file is
/// data loss rather than a configuration.
pub fn install(hosts: &HostObjects, slot: &str, image: Arc<Image>) -> Result<()> {
    let medium: Arc<dyn Medium> = image;
    if crate::dev::ata::medium::install(hosts, slot, medium)? {
        return Ok(());
    }
    Err(Error::Config {
        at: String::from("dev.blk"),
        message: alloc::format!("media slot `{slot}` already has an image bound to it"),
    })
}
