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
//! * `src/dev/ata/disk.rs` and `src/dev/ata/atapi.rs` contain **no I/O port
//!   address and no register offset**. A register is named ([`Reg`]), never
//!   numbered.
//! * `src/dev/pc/ide.rs` contains **no ATA command opcode, no SCSI one, no
//!   `IDENTIFY` word index and no status- or error-register bit**. It knows
//!   eight register names, two chip selects and an interrupt line.
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
//! [`TaskfileDevice`] is that door written as a trait, because a Serial ATA
//! port carries a *packet* device as readily as a hard disk and an engine typed
//! on [`AtaDisk`] could only ever drive one of them. Both implementations load
//! the struct into the same command block registers and run the same dispatch,
//! so the trait adds a second **caller** and not a second command set.
//!
//! # Two command sets, one cable
//!
//! [`disk`] is a non-packet device and [`atapi`] is a packet one, and the two
//! share the cable and **nothing else**. What they share is [`AtaDevice`] —
//! the six calls above, written down as a trait so that a [`bays::Bay`] is a
//! cable position rather than a hard-disk holder. What they do not share is a
//! single command opcode: `ata.disk` decodes `READ SECTOR(S)` against a CHS or
//! LBA address in its own registers, and `ata.cdrom` decodes nothing at all
//! until a twelve-byte SCSI command descriptor block has arrived through the
//! data register.
//!
//! The falsifiable form: **`disk.rs` and `atapi.rs` share no command dispatch
//! and no `Volatile`**, and every name `atapi.rs` imports from `disk` belongs
//! to the *register file* rather than to either command set — [`Reg`],
//! [`Position`], the Device, Device Control and Error bits, the two Status bits
//! whose meaning is the same on both kinds of device, and `put_string`, which
//! is how ATA lays an ASCII field into a word array whatever the device is.
//! ([`AtaDisk`] comes with them as the return type of [`AtaDevice::as_disk`],
//! and is never constructed or called there.) A driver tells the two apart by
//! the reset signature, which is the mechanism ATA/ATAPI-6 §9.1 provides for
//! exactly this and which both devices leave.
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
//! [`dev::medium`](crate::dev::medium) is the seam between the protocol and the
//! storage, and it is not this module's: an NVMe namespace and a `virtio.blk`
//! store their bytes behind the same trait, so it lives one level up under its
//! own `dev-medium` feature. A drive built from a media slot gets a `RamStore`
//! and costs its whole capacity in host memory; one whose slot a host filled
//! with a [`Medium`](crate::dev::medium::Medium) gets that instead —
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

#[cfg(feature = "dev-ata-atapi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-ata-atapi")))]
pub mod atapi;
pub mod disk;

#[cfg(feature = "dev-ata-atapi")]
pub use atapi::{AtapiDrive, CdromDevice};
pub use disk::taskfile::{Phase, Registers, Taskfile, TaskfileDevice};
pub use disk::{Address, AtaDisk, Geometry, Identity, Position, Reg};

/// What a host adapter can say to whatever is plugged into a cable position.
///
/// The ribbon cable, as a trait. `disk`'s module documentation argues that the
/// honest seam between a drive and a host adapter is the cable, and lists the
/// six calls it carries; this is that list, and it exists because there is now
/// more than one kind of thing on the far end of it. A [`crate::dev::pc::ide`]
/// channel drives an `ata.disk` and an `ata.cdrom` through the same code
/// because on a real board it drives them through the same eight ports.
///
/// **There is no command here.** Every method names a register or a signal;
/// what a device does when the Command register is written is the device's own
/// business, which is the whole content of the split.
pub trait AtaDevice: Send + Sync + core::fmt::Debug {
    /// Whether the Device register's `DEV` bit currently names this device.
    fn is_selected(&self) -> bool;

    /// Write one command block register. **Every device on the cable sees
    /// every write** — selection is decided by the device, not the adapter.
    fn write_reg(&self, reg: Reg, value: u16);

    /// Read one command block register. `debug` suppresses every side effect.
    fn read_reg(&self, reg: Reg, debug: bool) -> u16;

    /// Write the Device Control register, which every device on the cable
    /// sees: `HOB`, `nIEN` and `SRST`.
    fn write_device_control(&self, value: u8);

    /// The Status register **without** clearing the pending interrupt — the
    /// Alternate Status register, and what makes a debugger read safe.
    fn read_alt_status(&self) -> u8;

    /// Whether `INTRQ` is asserted: an interrupt is pending *and* `nIEN` is not
    /// holding it off.
    fn irq_asserted(&self) -> bool;

    /// A power-on or hardware reset. The medium survives; the protocol state
    /// does not.
    fn power_on_reset(&self);

    /// The non-packet drive this is, if it is one.
    ///
    /// Not a downcast in disguise and not an escape hatch for an adapter: the
    /// callers are the ones that genuinely need an `AtaDisk` and cannot work
    /// with anything else — `dev/ahci`, whose command engine speaks the
    /// taskfile seam, and `dev/amiga/gayle`, whose board never had a CD-ROM on
    /// its IDE port. A packet device takes the default and answers `None`,
    /// which those two read as an empty bay, which is the truthful answer to
    /// "is there a hard disk here".
    fn as_disk(self: alloc::sync::Arc<Self>) -> Option<alloc::sync::Arc<AtaDisk>> {
        None
    }

    /// This device as something that answers a whole command block at once.
    ///
    /// The **second** door, and the one a Serial ATA adapter speaks: see
    /// [`TaskfileDevice`]. Required rather than defaulted, because a default of
    /// `None` would let a device that simply forgot to write this line read as
    /// an empty port on every AHCI board in the tree, and an empty port is
    /// indistinguishable from a working one that has nothing in it.
    fn as_taskfile(self: alloc::sync::Arc<Self>) -> alloc::sync::Arc<dyn TaskfileDevice>;
}

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
    use super::{AtaDevice, TaskfileDevice};
    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::sync::{LockRank, Mutex};

    /// The kind a drive bay is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::rendezvous("ata-bay");

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
    /// Holds at most one device, of either kind: a bay is a *connector*, and a
    /// connector does not care whether the thing in it answers `READ
    /// SECTOR(S)` or `PACKET`. `Mutex` rather than an atomic because the
    /// contents are an `Arc` and this is a cold path — a device is fitted once,
    /// during construction, and looked at once per register access afterwards.
    pub struct Bay {
        device: Mutex<Option<Arc<dyn AtaDevice>>>,
    }

    impl fmt::Debug for Bay {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Bay")
                .field("occupied", &self.device.lock().is_some())
                .finish()
        }
    }

    impl Bay {
        /// An empty cable position.
        #[must_use]
        pub fn new() -> Bay {
            Bay {
                device: Mutex::with_rank(BAY_RANK, None),
            }
        }

        /// Fit `drive`, if the bay is empty.
        ///
        /// # Errors
        ///
        /// The drive back, unchanged, if something is already fitted. The
        /// caller has the names and makes the message.
        pub fn fit(&self, drive: Arc<AtaDisk>) -> core::result::Result<(), Arc<AtaDisk>> {
            self.fit_device(drive)
                .map_err(|back| back.as_disk().expect("it went in as a disk"))
        }

        /// Fit anything that speaks the cable, if the bay is empty.
        ///
        /// # Errors
        ///
        /// The device back, unchanged, if something is already fitted.
        pub fn fit_device(
            &self,
            device: Arc<dyn AtaDevice>,
        ) -> core::result::Result<(), Arc<dyn AtaDevice>> {
            let mut bay = self.device.lock();
            if bay.is_some() {
                return Err(device);
            }
            *bay = Some(device);
            Ok(())
        }

        /// Take whatever is in the bay out, if there is anything.
        pub fn remove(&self) -> Option<Arc<dyn AtaDevice>> {
            self.device.lock().take()
        }

        /// The **hard disk** in the bay, if what is in it is one.
        ///
        /// `None` for an empty bay *and* for a bay with a packet device in it,
        /// which is the truthful answer to a caller that can only drive a
        /// non-packet drive. [`Bay::device`] is the question a host adapter
        /// asks.
        #[must_use]
        pub fn drive(&self) -> Option<Arc<AtaDisk>> {
            self.device.lock().clone()?.as_disk()
        }

        /// Whatever is in the bay, if anything.
        #[must_use]
        pub fn device(&self) -> Option<Arc<dyn AtaDevice>> {
            self.device.lock().clone()
        }

        /// Whatever is in the bay, as the taskfile door a Serial ATA adapter
        /// speaks.
        ///
        /// Unlike [`Bay::drive`] this does **not** narrow to a hard disk: a
        /// packet device answers a taskfile too, and `dev/ahci` drives both
        /// through it.
        #[must_use]
        pub fn taskfile(&self) -> Option<Arc<dyn TaskfileDevice>> {
            Some(self.device.lock().clone()?.as_taskfile())
        }

        /// Whether there is anything in it.
        #[must_use]
        pub fn is_occupied(&self) -> bool {
            self.device.lock().is_some()
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
    disk::register(registry)?;
    #[cfg(feature = "dev-ata-atapi")]
    atapi::register(registry)?;
    Ok(())
}

/// Bind every `ata` class into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::error::Result<()> {
    disk::bind(bindings)?;
    #[cfg(feature = "dev-ata-atapi")]
    atapi::bind(bindings)?;
    Ok(())
}

/// What the validator should know about the `ata` classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    #[allow(unused_mut)]
    let mut out = alloc::vec![disk::schema()];
    #[cfg(feature = "dev-ata-atapi")]
    out.push(atapi::schema());
    out
}
