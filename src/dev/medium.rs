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

use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

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
pub const KIND: HostKind = HostKind::new("medium");

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
}
