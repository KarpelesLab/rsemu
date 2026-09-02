//! ATA: the drive, independent of whatever is talking to it.
//!
//! [`disk`] holds the whole of it — the command block registers, the command
//! set, the busy/DRQ handshake, CHS and LBA addressing and the 256-word
//! `IDENTIFY DEVICE` response. That module's own documentation argues where the
//! line between a drive and a host adapter falls and why it falls there; the
//! short version is that "IDE" means the controller is *on the drive*, so what
//! is left on the motherboard is address decode, and address decode is nearly
//! all [`crate::dev::pc::ide`] contains.
//!
//! The falsifiable form of that claim, which is the point of stating it:
//!
//! * `src/dev/ata/disk.rs` contains **no I/O port address and no register
//!   offset**. A register is named ([`Reg`]), never numbered.
//! * `src/dev/pc/ide.rs` contains **no ATA command opcode, no `IDENTIFY` word
//!   index and no status- or error-register bit**. It knows eight register
//!   names, two chip selects and an interrupt line.
//!
//! If either grep starts returning hits, the split has rotted.
//!
//! # Two front doors, one command set
//!
//! Eight ports written in the right order is the right model of a ribbon cable
//! and the wrong model of a Serial ATA link, which carries the whole command
//! block at once in a structure with no ordering and no register offset in it.
//! [`disk::taskfile`] is the second door: a [`Taskfile`] of six named fields,
//! loaded into the very same command block registers a port write would have
//! left, dispatched by the very same `AtaDisk::command`, with its data phase
//! running the identical busy/DRQ handshake in bulk.
//!
//! The falsifiable form of *that* claim: **delete [`disk::taskfile`] and
//! [`crate::dev::pc::ide`] is unchanged**; delete `AtaDisk::command` and both
//! adapters stop working. `dev/ahci` is the second caller and it did not need a
//! line of `pc/ide` to change.
//!
//! **ATAPI is out of scope.** There is no packet interface here, no SCSI
//! command descriptor block and no CD-ROM: `IDENTIFY PACKET DEVICE` is aborted,
//! which is exactly what a non-packet device does and exactly how a driver
//! finds out. Half a CD-ROM would be worse than none.
//!
//! # Finding each other
//!
//! A drive and its host adapter are separate objects in a machine description,
//! and there is no `core::bus` yet, so they meet through [`bays`] — a named
//! drive bay in the build's [`HostObjects`](crate::core::hosts::HostObjects),
//! the same rendezvous pattern [`crate::dev::sd::slots`], `bus::spi::buses` and
//! `host::chardev::ports` use. Both ends name the same bay (`bay = "ata0"`), and
//! whichever is constructed first creates it. An empty bay is an empty bay: the
//! adapter finds nothing, the command block reads back as zero, and a driver
//! concludes there is no drive there — which is what an unpopulated cable
//! position does.
//!
//! # What the platter is
//!
//! [`medium`] is the seam between the protocol and the storage. A drive built
//! from a media slot gets a `RamStore` and costs its whole capacity in host
//! memory; one whose slot a host filled with a [`Medium`] gets that instead —
//! `dev/blk` supplies a host file through `fstool`, so sparse raw, qcow2, DMG and
//! LUKS images all work and nothing above [`AtaDisk`]'s five methods changes.
//! **Both paths are supported**: the media slot is what keeps this device
//! `no_std`, and the file is what keeps a 16 GiB drive out of RAM.
//!
//! # Sources
//!
//! The **AT Attachment with Packet Interface** standards from T13 — ATA/ATAPI-6
//! (T13/1410D) for the command set and the register file, and its 48-bit
//! Address feature set — and the *IBM Personal Computer AT Technical Reference*
//! for the board's side of it. Clause and command names are cited on the items
//! they justify.
//!
//! **No emulator source of any licence was consulted, and no operating
//! system's ATA driver was opened** (`CLAUDE.md`, provenance).

pub mod disk;
pub mod medium;

pub use disk::taskfile::{Phase, Registers, Taskfile};
pub use disk::{Address, AtaDisk, Geometry, Identity, Position, Reg};
pub use medium::{Medium, MediumSlot, Snapshot};

/// The bay name a drive and an adapter get when neither says.
pub const DEFAULT_BAY: &str = "ata0";

/// Named drive bays: how a drive and its host adapter find each other.
///
/// A [`Bay`](bays::Bay) is the cable position, not the drive. It exists whether
/// or not something is in it, because that is the honest model of a ribbon
/// cable with one connector unused — and because the adapter is usually
/// constructed before the drive.
pub mod bays {
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use core::fmt;

    use super::disk::AtaDisk;
    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::sync::{LockRank, Mutex};

    /// The kind a drive bay is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::new("ata-bay");

    /// Where a drive bay's lock sits in the ranked order.
    ///
    /// An adapter looks the drive up *before* it touches anything else and
    /// releases the bay immediately, so this rank sits above the drive's own
    /// state and below the CPU's bus session. The whole ladder one `IN` travels:
    ///
    /// ```text
    ///   CPU session              (BUS 0x4000)
    ///     → the drive bay        (0x4c40, here)
    ///       → the drive's state  (DEVICE 0x5000)
    ///         → the adapter's interrupt wire (LEAF)
    /// ```
    ///
    /// A distinct number from [`crate::dev::sd::slots::SLOT_RANK`], which costs
    /// nothing: no machine holds both at once, and picking a distinct number
    /// means a board that someday does gets a deterministic order rather than a
    /// deadlock.
    pub const BAY_RANK: LockRank = LockRank::new(0x4c40);

    /// One position on a cable.
    ///
    /// Holds at most one drive. `Mutex` rather than an atomic because the
    /// contents are an `Arc` and this is a cold path — a drive is fitted once,
    /// during construction, and looked at once per register access afterwards.
    pub struct Bay {
        drive: Mutex<Option<Arc<AtaDisk>>>,
    }

    impl fmt::Debug for Bay {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Bay")
                .field("occupied", &self.drive.lock().is_some())
                .finish()
        }
    }

    impl Bay {
        /// An empty cable position.
        #[must_use]
        pub fn new() -> Bay {
            Bay {
                drive: Mutex::with_rank(BAY_RANK, None),
            }
        }

        /// Fit `drive`, if the bay is empty.
        ///
        /// # Errors
        ///
        /// The drive back, unchanged, if something is already fitted. The
        /// caller has the names and makes the message.
        pub fn fit(&self, drive: Arc<AtaDisk>) -> core::result::Result<(), Arc<AtaDisk>> {
            let mut bay = self.drive.lock();
            if bay.is_some() {
                return Err(drive);
            }
            *bay = Some(drive);
            Ok(())
        }

        /// Take the drive out, if there is one.
        pub fn remove(&self) -> Option<Arc<AtaDisk>> {
            self.drive.lock().take()
        }

        /// The drive in the bay, if any.
        #[must_use]
        pub fn drive(&self) -> Option<Arc<AtaDisk>> {
            self.drive.lock().clone()
        }

        /// Whether there is a drive in it.
        #[must_use]
        pub fn is_occupied(&self) -> bool {
            self.drive.lock().is_some()
        }
    }

    impl Default for Bay {
        fn default() -> Bay {
            Bay::new()
        }
    }

    /// The bay `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous: called before a build to fit a
    /// drive, or after one to take it out.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name, which is a collision between two host modules rather
    /// than anything a machine file can cause.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Bay>> {
        hosts.open(KIND, name, Bay::new)
    }

    /// The bay `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs to
    /// no build gets a private bay, so a device a unit test constructed
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Bay>> {
        props.host(KIND, name, Bay::new)
    }

    /// The bay called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Bay>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open bay name, in name order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }
}

/// Add every `ata` class to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::error::Result<()> {
    disk::register(registry)
}

/// Bind every `ata` class into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::error::Result<()> {
    disk::bind(bindings)
}

/// What the validator should know about the `ata` classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![disk::schema()]
}
