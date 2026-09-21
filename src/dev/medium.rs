//! What a drive's platter actually is.
//!
//! A storage device models a register file, a command set and a handshake; it
//! does not model *storage*. The bytes come from a [`Medium`], and every device
//! in the tree that has any — `ata.disk`, an NVMe namespace, `virtio.blk` —
//! reads and writes through this one trait, which is why `--drive` works the
//! same way on all of them.
//!
//! It lives here rather than under any one of them because it belongs to none:
//! a `riscv-virt` build has a virtio disk and no ATA command set anywhere, and
//! `dev-medium` is the feature that says so.
//!
//! There are two implementations:
//!
//! * [`RamStore`] — a flat buffer, filled from a media slot. `no_std`, no
//!   dependency, and the whole capacity costs host memory.
//! * [`dev::blk::Image`](crate::dev::blk) — a host file through
//!   `fstool::BlockDevice`, so sparse raw, qcow2, DMG and LUKS all work and a
//!   16 GiB drive costs 16 GiB of *disk*. `std`, and one of the two documented
//!   exceptions to the `no_std` rule (`CLAUDE.md`).
//!
//! The trait is not in `dev/blk` because `dev/blk` is `std` and its callers are
//! not: the seam has to be nameable from the side that cannot see `std`. It is
//! deliberately *not* a parallel invention of
//! `fstool::BlockDevice` — it is narrower (`&self`, no `Seek`, no `Read`) so
//! that the RAM implementation stays lock-free, and `dev/blk` adapts one to the
//! other in about thirty lines.
//!
//! # Errors are a three-way answer, not an `Option`
//!
//! Everything here returns [`MemResult`], and *which* error comes back is part
//! of the contract, because the device turns it into a status its guest can
//! act on — an ATA error bit, an NVMe status code, a virtio `S_IOERR`:
//!
//! | Error | Means |
//! | --- | --- |
//! | [`BusError::BadAccess`] | the range is not on this medium — off the end, or the image shrank |
//! | [`BusError::Unassigned`] | the medium is there and the bytes could not be moved: a host I/O error, a short read, a torn write |
//! | [`BusError::Protected`] | the medium is write protected |
//! | [`BusError::Retry`] | busy, and **nothing has happened yet** |
//!
//! `dev::ata::disk::error_bit` is that translation for ATA, and it is written
//! down once so the two read paths and the write path cannot disagree.
//!
//! A silent `0xff` and a bare `None` are both forbidden (`CLAUDE.md`).
//!
//! # Snapshots
//!
//! [`Snapshot`] is the policy, and it exists because "what does a snapshot of a
//! file-backed disk mean" has no single right answer — see its documentation.
//!
//! # Two doors, and they are not the same door
//!
//! [`MediumSlot`] is a **construction-time** hand-off: a run installs bytes
//! under a slot name, the drive that names that slot takes them out as it is
//! built, and the slot reads empty for the rest of the machine's life. That is
//! deliberate — two drives sharing one host file is data loss — and it is also
//! why nothing could ask a *running* machine what was in its drives.
//!
//! [`Removable`] is the other door, and it is open for as long as the machine
//! is. A drive with a door publishes [`MediaPort`] as
//! [`ExportId::REMOVABLE_MEDIA`], and [`attached`] walks a whole machine
//! collecting them, so `rsemu monitor`'s `media`, `insert` and `eject` work on
//! a PC's diskette drive, an Amiga's DF0, an ATAPI tray, a CD32's tray and an
//! SD socket without naming any of those five types. The three bespoke host
//! objects that preceded it (`dev::pc::fdc::drives::Drive`,
//! `dev::amiga::floppy::Floppy`, `dev::sd::slots::Slot`) still exist and still
//! do what they did; what they no longer do is *be the only route*.

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

use crate::core::device::{Export, ExportId};
use crate::core::error::{BusError, Error, Result};
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::props::Props;
use crate::core::space::{MemResult, RamStore};
use crate::core::sync::{LockRank, Mutex};

/// A drive's storage: capacity, bytes by offset, and durability.
///
/// `&self` throughout — a medium is shared behind an `Arc` and the drive
/// already holds its own state lock while it reads, so a `&mut self` seam would
/// either duplicate that lock or force one on [`RamStore`], which needs none.
pub trait Medium: Send + Sync + fmt::Debug {
    /// How many bytes the medium holds. Fixed for the life of the drive.
    fn capacity(&self) -> u64;

    /// Fill `dst` from `offset`.
    ///
    /// # Errors
    ///
    /// As the module documentation's table.
    fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult;

    /// Put `src` at `offset`.
    ///
    /// # Errors
    ///
    /// As the module documentation's table.
    fn write_at(&self, offset: u64, src: &[u8]) -> MemResult;

    /// Make every write so far durable — what `FLUSH CACHE` asks for.
    ///
    /// The default is success, which is the truth for a medium with no cache in
    /// front of it: [`RamStore`] took the write in the call that carried it and
    /// there is nowhere for it to be pending.
    ///
    /// # Errors
    ///
    /// [`BusError::Unassigned`] if the host refused to make the writes durable.
    fn flush(&self) -> MemResult {
        Ok(())
    }

    /// Whether the medium itself refuses writes, whatever the drive was
    /// configured with: a read-only image file, a device opened `O_RDONLY`.
    fn is_read_only(&self) -> bool {
        false
    }

    /// What a machine snapshot should do about these bytes.
    fn snapshot(&self) -> Snapshot {
        Snapshot::Capture
    }

    /// A stable one-line identity, for a [`Snapshot::Reference`] chunk and for
    /// diagnostics: format, path and capacity, or whatever names *this* medium.
    ///
    /// Must not vary between two calls on one medium, and must differ between
    /// two media a snapshot has no business being swapped between.
    fn describe(&self) -> String {
        String::new()
    }
}

/// What a machine snapshot does about a drive's contents.
///
/// Three positions, and every one is defensible for some drive, which is why
/// this is a policy rather than a decision taken once in the code:
///
/// * [`Capture`](Snapshot::Capture) — the bytes go into the chunk. A complete,
///   self-contained snapshot: restore it anywhere and the machine is the
///   machine. It costs the whole capacity per snapshot, which is fine for the
///   8 MiB drive a test builds and absurd for 16 GiB.
/// * [`Reference`](Snapshot::Reference) — the chunk records what the medium
///   *is* ([`Medium::describe`]) plus the drive's protocol state; the bytes stay
///   in the image file, which `save` flushes so that what is on disk is
///   consistent with the moment the snapshot was taken. Restoring checks the
///   identity still matches and then trusts the file. This is an **external**
///   snapshot in the usual sense: the image is outside it, so a guest that has
///   written to the image since is a difference the snapshot cannot see.
///   Copy-on-write overlays are what close that gap, and they are `fstool` work
///   (`ROADMAP.md` §7.1) rather than rsemu-on-top work.
/// * [`Refuse`](Snapshot::Refuse) — `save` fails, loudly. For a drive backed by
///   something a snapshot has no business either capturing or referencing: a
///   whole host block device, a network target.
///
/// The default for a RAM medium is `Capture` and for a file-backed one is
/// `Reference`, which is the promise each can actually keep. What is *not* on
/// offer is silently writing sixteen gigabytes into a snapshot chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Snapshot {
    /// Write the whole medium into the snapshot chunk.
    Capture,
    /// Write the medium's identity, flush it, and leave the bytes where they
    /// are.
    Reference,
    /// Refuse to snapshot a machine holding this medium.
    Refuse,
}

impl Snapshot {
    /// The name a machine description or a command line writes.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Snapshot::Capture => "capture",
            Snapshot::Reference => "reference",
            Snapshot::Refuse => "refuse",
        }
    }

    /// The policy that name refers to, or `None` if it names none of them.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Snapshot> {
        match name {
            "capture" => Some(Snapshot::Capture),
            "reference" => Some(Snapshot::Reference),
            "refuse" => Some(Snapshot::Refuse),
            _ => None,
        }
    }
}

impl fmt::Display for Snapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A flat buffer is the default medium, and it captures.
impl Medium for RamStore {
    fn capacity(&self) -> u64 {
        self.len()
    }

    fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        RamStore::read_at(self, offset, dst)
    }

    fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        RamStore::write_at(self, offset, src)
    }
}

// ---------------------------------------------------------------------------
// the rendezvous
// ---------------------------------------------------------------------------

/// The kind a drive medium is filed under in a build's [`HostObjects`].
///
/// [`pulled`](HostKind::pulled), which is the only use of that word in the
/// tree and is a statement about a hole rather than about safety. Host bytes
/// really do cross into a machine here — a drive's image is host state — but
/// the guest asks for a sector when it wants one instead of receiving it at an
/// instant, so no `(instant, payload)` log describes it and a sealed
/// host-object table has nothing to demand. What this needs is an identity
/// check on the image, which is [`Snapshot::Reference`] and a weak one;
/// [`core::record`](crate::core::record)'s table lists it as uncovered.
pub const KIND: HostKind = HostKind::pulled("medium");

/// Where a medium slot's lock sits in the ranked order.
///
/// Beside `dev::ata::bays::BAY_RANK` and for the same reason: it is taken
/// alone, once, during construction.
pub const MEDIUM_RANK: LockRank = LockRank::new(0x4c41);

/// A medium a *host* supplies, waiting for the drive that will use it.
///
/// The other half of `--drive hd0=disk.qcow2`. A machine file names a media
/// slot (`image = "hd0"`) and never a host path, because a machine file is
/// portable data describing a board; whether that slot is a blob in RAM or a
/// file on the host is a property of the **run**. So the run installs a
/// [`Medium`] under the slot's name, `ata.disk` looks for one as it is
/// constructed, and neither the machine file nor the IDE adapter changes.
///
/// Holds at most one medium and hands it over exactly once: two drives naming
/// one slot would otherwise share a file and corrupt it.
pub struct MediumSlot {
    /// [`MEDIUM_RANK`]: taken alone, during construction.
    medium: Mutex<Option<Arc<dyn Medium>>>,
}

impl fmt::Debug for MediumSlot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MediumSlot")
            .field("occupied", &self.medium.lock().is_some())
            .finish()
    }
}

impl Default for MediumSlot {
    fn default() -> MediumSlot {
        MediumSlot::new()
    }
}

impl MediumSlot {
    /// An empty slot.
    #[must_use]
    pub fn new() -> MediumSlot {
        MediumSlot {
            medium: Mutex::with_rank(MEDIUM_RANK, None),
        }
    }

    /// A slot already holding `medium`.
    #[must_use]
    pub fn holding(medium: Arc<dyn Medium>) -> MediumSlot {
        MediumSlot {
            medium: Mutex::with_rank(MEDIUM_RANK, Some(medium)),
        }
    }

    /// Put a medium in, reporting whether the slot was empty.
    pub fn fit(&self, medium: Arc<dyn Medium>) -> bool {
        let mut held = self.medium.lock();
        if held.is_some() {
            return false;
        }
        *held = Some(medium);
        true
    }

    /// Take the medium out, leaving the slot empty.
    ///
    /// Taking rather than cloning is deliberate: a medium is usually a host
    /// file, and two drives writing one file is data loss rather than a
    /// feature.
    #[must_use]
    pub fn take(&self) -> Option<Arc<dyn Medium>> {
        self.medium.lock().take()
    }

    /// Whether something is waiting here.
    #[must_use]
    pub fn is_occupied(&self) -> bool {
        self.medium.lock().is_some()
    }
}

/// The slot called `name` in this build, creating it on first mention.
///
/// # Errors
///
/// [`Error::Config`] if another kind of host object already holds that name.
pub fn attach(props: &Props, name: &str) -> Result<Arc<MediumSlot>> {
    props.host(KIND, name, MediumSlot::new)
}

/// The slot called `name`, if one has been opened.
///
/// # Errors
///
/// As [`attach`].
pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<MediumSlot>>> {
    hosts.get(KIND, name)
}

/// Install `medium` under `name` for a drive to pick up.
///
/// What `rsemu run … --drive hd0=disk.qcow2` calls, and what a Rust caller
/// assembling a machine calls. `false` means a medium was already waiting
/// there and this one was not fitted.
///
/// # Errors
///
/// As [`attach`].
pub fn install(hosts: &HostObjects, name: &str, medium: Arc<dyn Medium>) -> Result<bool> {
    let slot = hosts.open(KIND, name, MediumSlot::new)?;
    Ok(slot.fit(medium))
}

/// Every open medium-slot name, in name order.
#[must_use]
pub fn names(hosts: &HostObjects) -> alloc::vec::Vec<String> {
    hosts.names(KIND)
}

// ---------------------------------------------------------------------------
// the door: changing a medium while the machine runs
// ---------------------------------------------------------------------------

/// What one removable bay is holding.
///
/// A *description* rather than the medium: a host that lists what is in a
/// machine's drives has no business getting a handle it could read or write
/// behind the drive's back, and `Medium::describe` is already the weak
/// identity string [`Snapshot::Reference`] compares.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MediumInfo {
    /// [`Medium::describe`], or whatever the drive calls what it has —
    /// non-empty, because a bay that cannot say what is in it is no better
    /// than one that says nothing at all.
    pub describe: String,
    /// How many bytes it holds, as the *drive* counts them: a floppy's image
    /// length, a disc's user data, a card's capacity.
    pub capacity: u64,
    /// Whether the guest is refused writes to it — a floppy's tab, an image
    /// opened `O_RDONLY`, a disc.
    pub write_protected: bool,
}

/// One bay a medium can be put into and taken out of.
///
/// # Not [`ata::bays::Bay`](crate::dev::ata::bays)
///
/// That one is a *cable position*: which ATA device answers as device 0 on a
/// channel, decided once when the machine is built. This one is a **door** —
/// a diskette drive's slot, a tray, a card socket — and the whole reason it
/// exists is that it can be opened after realize. The two never appear in the
/// same sentence, but they do appear in the same tree, so: a bay here is
/// something a person puts a disk into.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MediaBay {
    /// The bay's device-local name, stable for the life of the machine and
    /// unique within its device: `"0"` for a one-drive diskette controller,
    /// `"df0"`, `"tray"`, `"card"`.
    pub name: String,
    /// One line naming what the bay physically is, for a listing.
    pub summary: String,
    /// What is in it, or `None` for an empty one.
    pub medium: Option<MediumInfo>,
}

/// A device whose medium can be changed while the machine runs.
///
/// Three questions and two commands, which is the whole of what a front end
/// needs and deliberately less than what any one drive can do: a host does
/// not seek, does not spin up a motor and does not lock a tray.
///
/// # The guest has to notice
///
/// **An implementation of [`insert`](Removable::insert) or
/// [`eject`](Removable::eject) that does not raise its part's own
/// disk-change signal is a defect, not an omission.** Every removable
/// mechanism ever built has one, because a filesystem the guest has mounted
/// is cached in the guest's RAM and a swap it does not see corrupts the next
/// write:
///
/// | Part | What it raises |
/// | --- | --- |
/// | Intel 82077AA / µPD765 | the digital input register's bit 7, `DSKCHG`, cleared by a step pulse with a diskette in the drive (*IBM PC/AT Technical Reference*) |
/// | Amiga internal drive | `CHNG*` on the drive connector, pulled low until the head is stepped, read at CIA-A `PRA` bit 2 |
/// | ATAPI CD-ROM | a pending UNIT ATTENTION with additional sense `28h 00h`, *NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED* (SFF-8020i §9.3) |
/// | SD card | nothing of its own — the *card* has no such line; a fresh card comes up in the idle state and the host's old RCA stops answering, which is what a real hot swap does |
///
/// # When it may be called
///
/// **At a scheduling-round boundary, with the machine stopped**, which is
/// where the monitor console calls it from. That is not a locking
/// requirement — the implementations take their own state lock and are safe
/// from anywhere — it is a *determinism* one, and
/// [`core::record`](crate::core::record) has the argument: a swap is an
/// instant plus an action, and an action that arrives from a host thread
/// while the guest runs lands at an instant nothing recorded. Between rounds
/// there is no such ambiguity, because the instant is the one the session's
/// previous command stopped at.
///
/// # Re-entrancy
///
/// An implementation mutates its own state in a short critical section and
/// releases the lock before any outward call — a wire change, a controller
/// notification — exactly as `CLAUDE.md` requires of every device method.
pub trait Removable: Send + Sync + fmt::Debug {
    /// Every bay this device has, in a stable order.
    ///
    /// A drive with nothing in it still has its bay: an empty one is a fact
    /// about the machine and `None` would hide it.
    fn bays(&self) -> Vec<MediaBay>;

    /// Put `medium` in bay `name`, raising the part's disk-change signal.
    ///
    /// Replaces whatever was there — a swap is one action on real hardware
    /// too, and making a caller eject first would leave a window in which the
    /// guest sees an empty drive that never physically existed.
    ///
    /// `write_protect` is the host's say: the tab, `--drive …,ro`, `insert …
    /// ro`. A medium that is [`read-only`](Medium::is_read_only) of its own
    /// accord is write protected whatever this says.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if there is no such bay, or if the medium is not
    /// something this drive can hold — a diskette image of the wrong length,
    /// a disc whose sectors do not divide.
    fn insert(&self, name: &str, medium: Arc<dyn Medium>, write_protect: bool) -> Result<()>;

    /// Take whatever is in bay `name` out, raising the part's disk-change
    /// signal.
    ///
    /// Ejecting an empty bay is not an error: it is the state the caller
    /// asked for, and a front end that has to look first has a race.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if there is no such bay.
    fn eject(&self, name: &str) -> Result<()>;
}

/// The handle [`ExportId::REMOVABLE_MEDIA`] carries.
///
/// A concrete newtype because [`Export::Opaque`] transports
/// `Arc<dyn Any + Send + Sync>` and there is no route from that to a
/// `dyn Removable`; this is the same shape `DrivePort`, `PaulaPort` and
/// `ControllerPort` already use. Cloning it is cloning the handle, not the
/// medium.
#[derive(Debug, Clone)]
pub struct MediaPort {
    inner: Arc<dyn Removable>,
}

impl MediaPort {
    /// The port for `inner`.
    #[must_use]
    pub fn new(inner: Arc<dyn Removable>) -> MediaPort {
        MediaPort { inner }
    }

    /// Every bay, as [`Removable::bays`].
    #[must_use]
    pub fn bays(&self) -> Vec<MediaBay> {
        self.inner.bays()
    }

    /// One bay by name, or `None` if this device has no such bay.
    #[must_use]
    pub fn bay(&self, name: &str) -> Option<MediaBay> {
        self.inner.bays().into_iter().find(|b| b.name == name)
    }

    /// As [`Removable::insert`].
    ///
    /// # Errors
    ///
    /// As [`Removable::insert`].
    pub fn insert(&self, name: &str, medium: Arc<dyn Medium>, write_protect: bool) -> Result<()> {
        self.inner.insert(name, medium, write_protect)
    }

    /// As [`Removable::eject`].
    ///
    /// # Errors
    ///
    /// As [`Removable::eject`].
    pub fn eject(&self, name: &str) -> Result<()> {
        self.inner.eject(name)
    }

    /// This port as the handle a device publishes.
    ///
    /// The one line every implementor's `Device::export` needs, written once
    /// so that eleven devices cannot spell the cast eleven ways.
    #[must_use]
    pub fn export(&self) -> Export {
        Export::Opaque(Arc::new(self.clone()) as Arc<dyn Any + Send + Sync>)
    }
}

/// A device that has removable bays, and its instance path.
#[derive(Debug, Clone)]
pub struct Attached {
    /// The instance path, as `devices` prints it.
    pub path: String,
    /// Its door.
    pub port: MediaPort,
}

/// Every device in `machine` that publishes [`ExportId::REMOVABLE_MEDIA`], in
/// declaration order.
///
/// The whole of what a front end needs to ask a machine it did not build
/// "what is in your drives?". A build with no removable device answers with an
/// empty list rather than an error, because a NES genuinely has no drives.
#[must_use]
pub fn attached(machine: &crate::machine::Machine) -> Vec<Attached> {
    machine
        .devices()
        .iter()
        .filter_map(|entry| {
            let export = entry.device().export(ExportId::REMOVABLE_MEDIA)?;
            let port = export.opaque()?.clone().downcast::<MediaPort>().ok()?;
            Some(Attached {
                path: entry.path().to_string(),
                port: MediaPort::clone(&port),
            })
        })
        .collect()
}

/// The device at `path` and its door, or a message saying which of the two
/// things went wrong.
///
/// # Errors
///
/// [`Error::Config`] naming `path` when no device is there, or when the one
/// that is has no removable bay.
pub fn attached_at(machine: &crate::machine::Machine, path: &str) -> Result<MediaPort> {
    let Some(entry) = machine.device(path) else {
        return Err(Error::Config {
            at: path.to_string(),
            message: "no device at that path".to_string(),
        });
    };
    entry
        .device()
        .export(ExportId::REMOVABLE_MEDIA)
        .and_then(|e| e.opaque()?.clone().downcast::<MediaPort>().ok())
        .map(|p| MediaPort::clone(&p))
        .ok_or_else(|| Error::Config {
            at: path.to_string(),
            message: alloc::format!("`{}` has no removable media", entry.class().name),
        })
}

/// Read a whole medium into a buffer, for a drive that decodes its image
/// rather than reading it a sector at a time.
///
/// A diskette is MFM-encoded track by track and a 1.44 MiB image is four
/// megabytes of cells; there is nothing to stream. A disc is the other case
/// and keeps its `Arc<dyn Medium>`.
///
/// # Errors
///
/// [`Error::Config`] naming `at` if the medium refuses the read, or if it is
/// larger than a host buffer could hold on this target.
pub fn slurp(at: &str, medium: &Arc<dyn Medium>) -> Result<alloc::vec::Vec<u8>> {
    let capacity = medium.capacity();
    let len = usize::try_from(capacity).map_err(|_| Error::Config {
        at: at.to_string(),
        message: alloc::format!("{capacity} bytes is more than this host can hold in memory"),
    })?;
    let mut bytes = alloc::vec![0u8; len];
    if len != 0 {
        medium.read_at(0, &mut bytes).map_err(|e| Error::Config {
            at: at.to_string(),
            message: alloc::format!("cannot be read: {e}"),
        })?;
    }
    Ok(bytes)
}

/// A flat buffer holding `bytes`, as a medium.
///
/// What a host has after reading a file, and what four drives in this tree
/// each used to write out for themselves.
#[must_use]
pub fn from_bytes(bytes: &[u8]) -> Arc<dyn Medium> {
    let store = RamStore::new(bytes.len() as u64);
    // A fresh store nobody else has a handle on; the write cannot fail.
    let _ = RamStore::write_at(&store, 0, bytes);
    Arc::new(store) as Arc<dyn Medium>
}

/// A medium error, as a diagnostic for a host-side caller.
///
/// What a device says when a medium refuses something *outside* a guest
/// command — filling a namespace at construction, flushing at realize — where
/// there is no status register to report it in and the offset is the only
/// thing that identifies which access failed.
#[must_use]
pub fn error_at(offset: u64, e: BusError) -> Error {
    Error::State(alloc::format!("{offset:#x}: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn a_ram_store_is_a_capturing_medium() {
        let store = RamStore::new(1024);
        assert_eq!(Medium::capacity(&store), 1024);
        assert_eq!(store.snapshot(), Snapshot::Capture);
        assert!(!store.is_read_only());
        assert!(store.describe().is_empty());
        assert!(Medium::write_at(&store, 512, &[1, 2, 3]).is_ok());
        let mut got = vec![0u8; 3];
        assert!(Medium::read_at(&store, 512, &mut got).is_ok());
        assert_eq!(got, vec![1, 2, 3]);
        assert!(store.flush().is_ok());
    }

    #[test]
    fn a_read_past_the_end_is_bad_access_not_a_silent_zero() {
        let store = RamStore::new(512);
        let mut got = vec![0u8; 8];
        assert_eq!(
            Medium::read_at(&store, 510, &mut got),
            Err(BusError::BadAccess)
        );
    }

    #[test]
    fn a_slot_hands_its_medium_over_exactly_once() {
        let store: Arc<dyn Medium> = Arc::new(RamStore::new(512));
        let slot = MediumSlot::new();
        assert!(!slot.is_occupied());
        assert!(slot.fit(Arc::clone(&store)));
        assert!(slot.is_occupied());
        assert!(!slot.fit(Arc::clone(&store)));
        assert!(slot.take().is_some());
        assert!(slot.take().is_none());
    }

    #[test]
    fn a_policy_round_trips_through_its_name() {
        for policy in [Snapshot::Capture, Snapshot::Reference, Snapshot::Refuse] {
            assert_eq!(Snapshot::from_name(policy.as_str()), Some(policy));
        }
        assert_eq!(Snapshot::from_name("maybe"), None);
    }

    #[test]
    fn a_host_installs_a_medium_under_a_slot_name() {
        let hosts = HostObjects::new();
        let store: Arc<dyn Medium> = Arc::new(RamStore::new(512));
        assert!(install(&hosts, "hd0", Arc::clone(&store)).expect("installed"));
        assert!(!install(&hosts, "hd0", store).expect("a second refused"));
        assert_eq!(names(&hosts), vec![String::from("hd0")]);
        let slot = get(&hosts, "hd0").expect("no type clash").expect("a slot");
        assert!(slot.take().is_some());
        assert!(get(&hosts, "hd1").expect("no type clash").is_none());
    }

    /// A drive with two bays, which nothing in the tree has yet and which the
    /// seam has to support anyway: the naming rule a front end uses falls
    /// apart the moment a device has more than one, and a test is cheaper than
    /// finding that out from the first two-drive controller.
    #[derive(Debug)]
    struct TwoBay {
        held: Mutex<[Option<Arc<dyn Medium>>; 2]>,
        protected: Mutex<[bool; 2]>,
    }

    impl TwoBay {
        fn new() -> Arc<TwoBay> {
            Arc::new(TwoBay {
                held: Mutex::with_rank(MEDIUM_RANK, [const { None }; 2]),
                protected: Mutex::with_rank(LockRank::LEAF, [false; 2]),
            })
        }

        fn index(name: &str) -> Result<usize> {
            match name {
                "a" => Ok(0),
                "b" => Ok(1),
                _ => Err(Error::Config {
                    at: name.to_string(),
                    message: String::from("two bays, `a` and `b`"),
                }),
            }
        }
    }

    impl Removable for TwoBay {
        fn bays(&self) -> alloc::vec::Vec<MediaBay> {
            let held = self.held.lock();
            let protected = *self.protected.lock();
            ["a", "b"]
                .iter()
                .enumerate()
                .map(|(i, name)| MediaBay {
                    name: String::from(*name),
                    summary: alloc::format!("bay {name}"),
                    medium: held[i].as_ref().map(|m| MediumInfo {
                        describe: m.describe(),
                        capacity: m.capacity(),
                        write_protected: protected[i] || m.is_read_only(),
                    }),
                })
                .collect()
        }

        fn insert(&self, name: &str, medium: Arc<dyn Medium>, write_protect: bool) -> Result<()> {
            let i = TwoBay::index(name)?;
            self.held.lock()[i] = Some(medium);
            self.protected.lock()[i] = write_protect;
            Ok(())
        }

        fn eject(&self, name: &str) -> Result<()> {
            let i = TwoBay::index(name)?;
            self.held.lock()[i] = None;
            self.protected.lock()[i] = false;
            Ok(())
        }
    }

    #[test]
    fn a_port_answers_three_questions_and_takes_two_commands() {
        let port = MediaPort::new(TwoBay::new() as Arc<dyn Removable>);
        assert_eq!(port.bays().len(), 2);
        assert!(port.bays().iter().all(|b| b.medium.is_none()));
        assert!(port.bay("a").is_some());
        assert!(port.bay("c").is_none(), "a bay nothing has");

        port.insert("b", from_bytes(&[1, 2, 3, 4]), true)
            .expect("bay b");
        let b = port.bay("b").expect("bay b").medium.expect("held");
        assert_eq!(b.capacity, 4);
        assert!(b.write_protected, "the host said so");
        assert!(port.bay("a").expect("bay a").medium.is_none(), "not both");

        port.eject("b").expect("bay b");
        assert!(port.bay("b").expect("bay b").medium.is_none());
        // Ejecting an empty bay is the state the caller asked for, not an
        // error: a front end that has to look first has a race.
        port.eject("b").expect("still fine");

        assert!(port.insert("c", from_bytes(&[0]), false).is_err());
        assert!(port.eject("c").is_err());
    }

    #[test]
    fn a_port_travels_as_the_export_a_host_downcasts() {
        // The handle is `Opaque` because it names `Medium`, which lives behind
        // `dev-medium`, and `core/` is never feature-gated. So the cast has to
        // come back, and this is the assertion that it does.
        let port = MediaPort::new(TwoBay::new() as Arc<dyn Removable>);
        let export = port.export();
        let back = export
            .opaque()
            .expect("an opaque handle")
            .clone()
            .downcast::<MediaPort>()
            .expect("and it is a MediaPort");
        back.insert("a", from_bytes(&[7; 16]), false)
            .expect("through the handle");
        assert_eq!(
            port.bay("a").expect("bay a").medium.expect("held").capacity,
            16,
            "the handle and the port are one drive"
        );
    }

    #[test]
    fn slurping_a_medium_reads_all_of_it_and_names_what_refused() {
        let store: Arc<dyn Medium> = Arc::new(RamStore::new(64));
        Medium::write_at(&*store, 0, &[9u8; 64]).expect("in range");
        assert_eq!(slurp("x", &store).expect("it reads"), alloc::vec![9u8; 64]);

        // An empty medium is empty rather than an error — a drive with a
        // zero-length image is a drive with nothing in it.
        let empty: Arc<dyn Medium> = Arc::new(RamStore::new(0));
        assert!(slurp("x", &empty).expect("nothing to read").is_empty());
    }
}
