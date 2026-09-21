//! SCSI: the bus, and whatever is hanging off it.
//!
//! The same split `dev/ata` draws, drawn again for a bus that genuinely has
//! one. On ATA the controller is *on the drive*, so an "adapter" is address
//! decode; on SCSI the initiator and the target are separate parts speaking a
//! defined protocol over a shared cable, and the cable is where the seam
//! belongs.
//!
//! * [`Target`] is the cable, as a trait: select me, tell me which phase you
//!   are requesting, move these bytes, let go. **There is no command opcode in
//!   it** — what a target does when it has received a command descriptor block
//!   is the target's own business.
//! * [`Bus`] is the cable itself: eight addresses, each either empty or
//!   holding a target. It is a rendezvous, exactly like
//!   [`dev::ata::bays`](crate::dev::ata::bays) — a controller and a target are
//!   separate objects in a machine description and meet under a name
//!   (`bus = "scsi0"`), whichever is constructed first creating it.
//! * [`disk`] is a direct-access block device: the one target this feature
//!   ships. Adding a CD-ROM later is a **new target**, not a new controller.
//!
//! The falsifiable form of the split, which is the point of writing it down:
//!
//! * `src/dev/scsi/mod.rs` and `src/dev/scsi/disk.rs` have **no register
//!   offset and no controller register name** in their code — no `SASR`, no
//!   `SCMD`, no auxiliary status.
//! * `src/dev/wd33c93.rs` has **no SCSI command** in its code — no `INQUIRY`,
//!   no `READ(10)`, no sense key, no mode page.
//!
//! If either grep starts returning hits outside a comment, the split has
//! rotted.
//!
//! # The phases, and who drives them
//!
//! A SCSI bus is a state machine with one state variable that both ends can
//! see: the *information transfer phase*, encoded by three signals — `MSG`,
//! `C/D` and `I/O` (SCSI-2 X3.131-1994 §5.1). The **target** drives all three
//! and therefore chooses the phase; the initiator reads them and moves bytes in
//! whichever direction `I/O` says. [`Phase`] is those three signals plus the
//! two states in which no information moves at all, `BUS FREE` and the
//! target's own idea of "I have nothing more to say".
//!
//! A whole command is then:
//!
//! ```text
//!   BUS FREE → (arbitration, selection) → MESSAGE OUT   IDENTIFY
//!                                       → COMMAND       the CDB
//!                                       → DATA IN/OUT   if there is any
//!                                       → STATUS        one byte
//!                                       → MESSAGE IN    COMMAND COMPLETE
//!                                       → BUS FREE
//! ```
//!
//! and that sequence, phase for phase, is what
//! [`Wd33c93`](crate::dev::wd33c93::Wd33c93) walks.
//!
//! # Disconnection is not modelled
//!
//! A target here never disconnects: it completes the whole command in the
//! connection the initiator opened. That is a permitted target, not a
//! shortcut — disconnection is optional in SCSI-2 (§6.6.10; a target need not
//! implement it, and an initiator that grants `DiscPriv` in its IDENTIFY
//! message is granting permission rather than issuing a requirement). What it
//! costs is that `RESELECTION` is never exercised, so the controller's
//! reselection paths are written from the datasheet and tested against a
//! synthetic target rather than against this one. Said here rather than left
//! to be discovered.
//!
//! # Sources
//!
//! *Small Computer System Interface-2*, X3.131-1994 — §5 the bus phases, §6
//! the messages, §7 the command structure and the status byte, §8 the commands
//! every device type implements, §9 the direct-access device (`INQUIRY`, `READ
//! CAPACITY`, `READ`, `WRITE`, `MODE SENSE` and its pages). SCSI-1 (X3.131-1986)
//! for the six-byte commands an Amiga of 1990 actually issues. Clause numbers
//! are cited on the items they justify.
//!
//! **No emulator source of any licence was consulted, and no operating
//! system's SCSI driver was opened** (`CLAUDE.md`, provenance).

pub mod disk;

pub use disk::{DiskDevice, ScsiDisk};

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::Result;
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::props::Props;
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{LockRank, Mutex};

/// How many addresses a narrow SCSI bus has (X3.131-1994 §5.1.3.2: one data
/// line per device, eight on an 8-bit bus).
pub const IDS: usize = 8;

/// The bus name a controller and a target get when neither says.
pub const DEFAULT_BUS: &str = "scsi0";

/// An information transfer phase, as the three phase signals encode it.
///
/// A real enum rather than the `#[repr(transparent)]` newtype the house style
/// uses for extensible enumerations, because this one genuinely is not
/// extensible: `MSG`, `C/D` and `I/O` are three wires, and eight combinations
/// is all there will ever be. Two of the eight are reserved by the standard and
/// no device may request them, which is why they are absent here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No device is using the bus (§5.1.1). Nothing to transfer.
    BusFree,
    /// `MSG`=0 `C/D`=0 `I/O`=0: bytes flow initiator → target (§5.1.5.1).
    DataOut,
    /// `MSG`=0 `C/D`=0 `I/O`=1: bytes flow target → initiator (§5.1.5.2).
    DataIn,
    /// `MSG`=0 `C/D`=1 `I/O`=0: the command descriptor block (§5.1.5.3).
    Command,
    /// `MSG`=0 `C/D`=1 `I/O`=1: one status byte (§5.1.5.4).
    Status,
    /// `MSG`=1 `C/D`=1 `I/O`=0: a message to the target (§5.1.5.6).
    MessageOut,
    /// `MSG`=1 `C/D`=1 `I/O`=1: a message from the target (§5.1.5.5).
    MessageIn,
}

impl Phase {
    /// The three phase signals, `MSG` `C/D` `I/O`, as a three-bit field —
    /// which is exactly the `MCI` field a WD33C93A reports in its SCSI Status
    /// register. `None` for [`Phase::BusFree`], where nothing is asserted.
    #[must_use]
    pub const fn mci(self) -> Option<u8> {
        match self {
            Phase::BusFree => None,
            Phase::DataOut => Some(0b000),
            Phase::DataIn => Some(0b001),
            Phase::Command => Some(0b010),
            Phase::Status => Some(0b011),
            Phase::MessageOut => Some(0b110),
            Phase::MessageIn => Some(0b111),
        }
    }

    /// Whether bytes move target → initiator in this phase — the `I/O` signal.
    #[must_use]
    pub const fn is_input(self) -> bool {
        matches!(self, Phase::DataIn | Phase::Status | Phase::MessageIn)
    }
}

/// Status byte values a target returns (X3.131-1994 §7.2.3, Table 27).
pub mod status {
    /// The command completed without error.
    pub const GOOD: u8 = 0x00;
    /// Sense data is available; the initiator should issue `REQUEST SENSE`.
    pub const CHECK_CONDITION: u8 = 0x02;
    /// The logical unit is busy and could not accept the command.
    pub const BUSY: u8 = 0x08;
}

/// Message codes this bus uses (X3.131-1994 §6.6, Table 15).
pub mod message {
    /// The target has completed the command and is about to free the bus.
    pub const COMMAND_COMPLETE: u8 = 0x00;
    /// The initiator refuses the message just received.
    pub const MESSAGE_REJECT: u8 = 0x07;
    /// No message to send, sent when the target asks anyway.
    pub const NO_OPERATION: u8 = 0x08;
    /// The `IDENTIFY` message's fixed high bit (§6.6.7).
    pub const IDENTIFY: u8 = 0x80;
    /// `IDENTIFY`'s "you may disconnect" grant.
    pub const DISC_PRIV: u8 = 0x40;
    /// The logical unit number an `IDENTIFY` carries.
    pub const LUN_MASK: u8 = 0x07;
}

/// Sense keys (X3.131-1994 §7.2.14, Table 69).
pub mod sense {
    /// No sense data to report.
    pub const NO_SENSE: u8 = 0x00;
    /// The medium or the drive failed.
    pub const MEDIUM_ERROR: u8 = 0x03;
    /// A non-recoverable hardware failure.
    pub const HARDWARE_ERROR: u8 = 0x04;
    /// The command, or a field in it, is not one this device supports.
    pub const ILLEGAL_REQUEST: u8 = 0x05;
    /// Something changed — a reset, a medium change.
    pub const UNIT_ATTENTION: u8 = 0x06;
    /// The medium refuses to be written.
    pub const DATA_PROTECT: u8 = 0x07;

    /// Additional sense code: invalid command operation code (§7.2.14).
    pub const ASC_INVALID_COMMAND: u8 = 0x20;
    /// Additional sense code: logical block address out of range.
    pub const ASC_LBA_OUT_OF_RANGE: u8 = 0x21;
    /// Additional sense code: invalid field in the command descriptor block.
    pub const ASC_INVALID_FIELD: u8 = 0x24;
    /// Additional sense code: the logical unit is not supported.
    pub const ASC_INVALID_LUN: u8 = 0x25;
    /// Additional sense code: a power-on or bus-device reset has occurred.
    pub const ASC_RESET: u8 = 0x29;
    /// Additional sense code: write protected.
    pub const ASC_WRITE_PROTECTED: u8 = 0x27;
}

/// How long a command descriptor block is, from the group code in its first
/// byte (X3.131-1994 §7.1, Table 20).
///
/// Group 3 and group 4 are reserved and group 6 and 7 are vendor specific; both
/// come back as `None`, which a controller turns into whatever its own
/// specification says about an unknown group.
#[must_use]
pub const fn cdb_len(opcode: u8) -> Option<usize> {
    match opcode >> 5 {
        0 => Some(6),
        1 | 2 => Some(10),
        5 => Some(12),
        _ => None,
    }
}

/// What an initiator can say to whatever is at one address on the cable.
///
/// The cable, as a trait, and deliberately narrow: seven calls, none of which
/// names a command. A [`Bus`] address is a *connector*, and a connector does
/// not care whether the thing at the far end answers `READ(10)` or `READ TOC`.
///
/// Every method takes `&self`: a target sits behind an `Arc` on a bus several
/// objects can see, and holds whatever lock it needs for as long as it needs
/// it and no longer.
pub trait Target: Send + Sync + fmt::Debug {
    /// Respond to a selection, or not.
    ///
    /// `atn` is whether the initiator asserted `ATN` during selection, which is
    /// the initiator saying it has a message to send (§5.1.3.3). `true` here is
    /// the target asserting `BSY`; `false` is nobody home, which the initiator
    /// sees as a selection timeout.
    ///
    /// After a `true` the target is in a phase: [`Phase::MessageOut`] if `atn`,
    /// [`Phase::Command`] otherwise.
    fn select(&self, atn: bool) -> bool;

    /// The phase the target is requesting now.
    ///
    /// [`Phase::BusFree`] once the target has released the bus, which is how an
    /// initiator learns the operation is over.
    fn phase(&self) -> Phase;

    /// Move bytes target → initiator, returning how many moved.
    ///
    /// Short is not an error: a `DATA IN` phase ends when the target has no
    /// more to give, and the initiator finding that out is the mechanism by
    /// which a phase ends. Zero in an output phase is the honest answer, and
    /// the caller has made a mistake it can see.
    fn read(&self, dst: &mut [u8]) -> usize;

    /// Move bytes initiator → target, returning how many the target took.
    fn write(&self, src: &[u8]) -> usize;

    /// The initiator has let go of the bus: `BUS FREE`.
    ///
    /// A target that had more to say forgets it. This is the abort path as well
    /// as the tidy one — there is no difference from the target's side, which
    /// is what §5.1.1 says about an unexpected bus free.
    fn release(&self);

    /// `RST` on the cable: everything forgets everything, and the next command
    /// gets a `UNIT ATTENTION` (§5.2.2).
    fn bus_reset(&self);

    /// Write this target's state into a snapshot.
    ///
    /// # Errors
    ///
    /// Whatever the sink reports.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()>;

    /// Read this target's state back.
    ///
    /// # Errors
    ///
    /// [`crate::Error::State`] if the chunk does not describe this target.
    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()>;
}

// ---------------------------------------------------------------------------
// the bus
// ---------------------------------------------------------------------------

/// Where a SCSI bus's lock sits in the ranked order.
///
/// A controller looks a target up *before* it touches the target and releases
/// the bus immediately, so this rank sits above the target's own state and
/// below the CPU's bus session — the ladder one register write travels:
///
/// ```text
///   CPU session                (BUS 0x4000)
///     → the SCSI bus           (0x4c60, here)
///       → the target's state   (DEVICE 0x5000)
///         → the controller's interrupt wire (LEAF)
/// ```
///
/// A distinct number from [`crate::dev::ata::bays::BAY_RANK`] for the reason
/// that one gives: no board holds both today, and distinct numbers mean a board
/// that someday does gets a deterministic order rather than a deadlock.
pub const BUS_RANK: LockRank = LockRank::new(0x4c60);

/// The kind a SCSI bus is filed under in a build's [`HostObjects`].
pub const KIND: HostKind = HostKind::rendezvous("scsi-bus");

/// One cable: eight addresses, each empty or holding a target.
///
/// `Mutex` rather than an array of atomics because the contents are `Arc`s and
/// this is a cold path — a target is fitted once, during construction, and
/// looked at once per selection afterwards.
pub struct Bus {
    targets: Mutex<[Option<Arc<dyn Target>>; IDS]>,
}

impl fmt::Debug for Bus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let occupied: Vec<usize> = self
            .targets
            .lock()
            .iter()
            .enumerate()
            .filter_map(|(id, t)| t.as_ref().map(|_| id))
            .collect();
        f.debug_struct("Bus").field("ids", &occupied).finish()
    }
}

impl Bus {
    /// An empty cable.
    #[must_use]
    pub fn new() -> Bus {
        Bus {
            targets: Mutex::with_rank(BUS_RANK, [const { None }; IDS]),
        }
    }

    /// Fit `target` at `id`, if that address is free.
    ///
    /// # Errors
    ///
    /// The target back, unchanged, if the address is taken or out of range. The
    /// caller has the names and makes the message.
    pub fn fit(
        &self,
        id: u8,
        target: Arc<dyn Target>,
    ) -> core::result::Result<(), Arc<dyn Target>> {
        let mut bus = self.targets.lock();
        let Some(place) = bus.get_mut(usize::from(id)) else {
            return Err(target);
        };
        if place.is_some() {
            return Err(target);
        }
        *place = Some(target);
        Ok(())
    }

    /// Whatever is at `id`, if anything.
    #[must_use]
    pub fn target(&self, id: u8) -> Option<Arc<dyn Target>> {
        self.targets.lock().get(usize::from(id))?.clone()
    }

    /// Take whatever is at `id` off the bus.
    pub fn remove(&self, id: u8) -> Option<Arc<dyn Target>> {
        self.targets.lock().get_mut(usize::from(id))?.take()
    }

    /// Every occupied address, ascending.
    #[must_use]
    pub fn occupied(&self) -> Vec<u8> {
        self.targets
            .lock()
            .iter()
            .enumerate()
            .filter_map(|(id, t)| t.as_ref().map(|_| id as u8))
            .collect()
    }

    /// `RST`: tell every target on the cable, with the bus lock released
    /// before any of them is called.
    pub fn reset(&self) {
        let targets: Vec<Arc<dyn Target>> = self.targets.lock().iter().flatten().cloned().collect();
        for target in targets {
            target.bus_reset();
        }
    }
}

impl Default for Bus {
    fn default() -> Bus {
        Bus::new()
    }
}

/// Named SCSI buses: how a controller and its targets find each other.
pub mod buses {
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use super::{Bus, KIND};
    use crate::core::error::Result;
    use crate::core::hosts::HostObjects;
    use crate::core::props::Props;

    /// The bus `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Bus>> {
        hosts.open(KIND, name, Bus::new)
    }

    /// The bus `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`. A `Props` belonging to no
    /// build gets a private bus, so a device a unit test constructed directly
    /// still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Bus>> {
        props.host(KIND, name, Bus::new)
    }

    /// The bus called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Bus>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open bus name, in name order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }
}

/// The bus `name` refers to in `hosts`, creating it on first mention.
///
/// A shorthand for [`buses::open`], kept because `dev/ata` has the same pair.
///
/// # Errors
///
/// As [`buses::open`].
pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Bus>> {
    buses::open(hosts, name)
}

/// The bus a device's properties name, creating it on first mention.
///
/// # Errors
///
/// As [`buses::attach`].
pub fn attach(props: &Props, name: &str) -> Result<Arc<Bus>> {
    buses::attach(props, name)
}

/// Every open bus name, in name order.
#[must_use]
pub fn names(hosts: &HostObjects) -> Vec<String> {
    buses::names(hosts)
}

/// Add every `scsi` class to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    disk::register(registry)
}

/// Bind every `scsi` class into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    disk::bind(bindings)
}

/// What the validator should know about the `scsi` classes.
#[must_use]
pub fn schemas() -> Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![disk::schema()]
}

#[cfg(test)]
mod tests;
