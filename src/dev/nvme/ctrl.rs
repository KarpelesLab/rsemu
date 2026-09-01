//! The controller itself: the register block, the queues a driver builds in
//! guest RAM, the PRP walker, and the command set.
//!
//! Split from [`super`] the same way [`Hcd`](crate::dev::usb::ehci::Hcd) is
//! split from its register map: everything here is about NVM Express and
//! nothing here is about PCI. The transport contributes four things and they
//! all arrive through setters — the address space this controller masters, the
//! interrupt output, whether the function is allowed to master the bus
//! (`COMMAND[2]`), and whether its `INTx#` emission is disabled
//! (`COMMAND[10]`).
//!
//! # Sources
//!
//! **NVM Express Base Specification, Revision 1.4** (freely published at
//! <https://nvmexpress.org/specifications/>), and the **NVM Command Set** it
//! carries in that revision:
//!
//! * §3.1 — the controller registers: `CAP`, `VS`, `INTMS`/`INTMC`, `CC`,
//!   `CSTS`, `AQA`, `ASQ`, `ACQ`, and the doorbell stride.
//! * §3.1.5 — Controller Reset: what `CC.EN` 1→0 means.
//! * §4.1 — submission and completion queue head/tail arithmetic, and what an
//!   invalid doorbell value does.
//! * §4.2 — the 64-byte submission queue entry, and §4.6 the 16-byte
//!   completion queue entry with its phase tag.
//! * §4.3 — the Physical Region Page (PRP) entry and the PRP List.
//! * §5 — the admin command set: Create/Delete I/O Queue, Identify, Set/Get
//!   Features, Get Log Page, Abort, Asynchronous Event Request.
//! * §6 — the NVM command set: Flush, Write, Read, Write Zeroes.
//! * §7.5.1.1 — pin-based interrupt behaviour, which is the only kind this
//!   build has wires for.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance); in particular
//! the Linux `nvme` driver is GPLv2 and was not opened.
//!
//! # The shape of the thing
//!
//! ```text
//!   BAR0 ──► registers 0x00-0x3f ──► CC, CSTS, AQA, ASQ, ACQ
//!            doorbells at 0x1000 ──► SQyTDBL, CQyHDBL
//!
//!   guest RAM:  SQ ──► 64-byte command ──► PRP1/PRP2 ──► the data pages
//!               CQ ◄── 16-byte completion with a phase tag
//! ```
//!
//! Everything except the register block lives in **guest memory**, and this
//! controller reads and writes it as a bus master with its own
//! [`RequesterId`]. That is the whole difference between this device and every
//! programmed-I/O device in the tree.
//!
//! # Locks, and the order they go in
//!
//! One state lock at [`NVME_RANK`], which sits *below* [`LockRank::DEVICE`] —
//! so the PCI function's configuration lock may be taken and released before
//! this one, never the other way round — and *above* [`LockRank::WIRE`], so the
//! interrupt output can be driven after it is released. It is never held across
//! a guest-memory access, a medium access, or a wire change: [`Controller::run`]
//! takes it to pick the next command and to advance a queue pointer, releases
//! it, and only then does the outward thing. That is `CLAUDE.md`'s re-entrancy
//! contract written as code, and here it is load-bearing rather than
//! decorative, because a guest may aim a PRP entry at this controller's own
//! register block.
//!
//! # Re-entrancy: a doorbell reached from inside a doorbell
//!
//! A command's PRP list is guest memory, and guest memory is an address space
//! that also contains this controller's own BAR. So a driver — by accident or
//! on purpose — can make a data transfer land on `SQ0TDBL`, and the write
//! handler is then re-entered from inside itself. A `busy` flag is the
//! answer, and it is the same answer [`Wire`](crate::core::wire::Wire) gives
//! for a re-entrant level change: **the work is iterative, not recursive**. A
//! re-entrant doorbell write records the new tail and returns; the outermost
//! [`Controller::run`] re-reads every tail after each command and picks the new
//! work up. Recursion depth is one, whatever the guest builds.
//!
//! # Time
//!
//! A command completes inside the doorbell write that submitted it, so this is
//! a controller with zero service time. Two consequences, both deliberate:
//!
//! * **No host latency reaches the guest's timeline** (`docs/buses/storage.md`:
//!   "a host read takes zero guest time"), so two runs of the same machine
//!   agree whatever the host's page cache was doing.
//! * **There is no in-flight state to snapshot.** Everything that outlives a
//!   command — the queues, the entries, the data — is in guest memory, which
//!   the RAM device saves. The controller's own durable state is the register
//!   file and the queue descriptors below, and a snapshot is never taken with
//!   half a command executed. This is the same argument
//!   [`Hcd::save`](crate::dev::usb::ehci::Hcd) makes for a microframe.
//!
//! A service latency, when it comes, will be a clock domain and a scheduler
//! event (`ROADMAP.md` §4.7) — and the host's actual I/O time will still not be
//! it.

use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{BusError, Result};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, RequesterId,
};
use crate::core::state::{Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU32, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::ata::{Medium, Snapshot};

// ---------------------------------------------------------------------------
// shape
// ---------------------------------------------------------------------------

/// Where this controller's state lock sits in the ranked order.
///
/// Below [`LockRank::DEVICE`] and above [`LockRank::WIRE`]. The ladder one
/// doorbell write travels:
///
/// ```text
///   CPU session                       (BUS 0x4000)
///     → the PCI function's config     (DEVICE 0x5000)   — configuration only
///       → the controller's state      (0x5a00, here)
///         → the medium                (LEAF)
///           → the interrupt output    (WIRE 0x6000, taken after 0x5a00 is released)
/// ```
///
/// It cannot be [`LockRank::BUS`] for the reason `core::space` states and
/// `src/bus/pci/mod.rs` repeats: *"A CPU holds a `BUS`-ranked lock across the
/// accesses it issues"*, and every access to this register block arrives from
/// inside one.
pub const NVME_RANK: LockRank = LockRank::new(0x5a00);

/// How much memory the register block decodes: 8 KiB.
///
/// The registers occupy `0x00`-`0x3f` and the doorbells start at `0x1000`
/// (§3.1). A power of two, because a BAR window is one (*PCI Local Bus
/// Specification* Rev 2.1 §6.2.5.1).
pub const REGISTER_LEN: u64 = 0x2000;

/// Where the doorbell registers start (§3.1).
const DOORBELL_BASE: u64 = 0x1000;

/// Bytes between one doorbell and the next: `2^(2 + CAP.DSTRD)` with
/// `CAP.DSTRD` zero (§3.1.1).
const DOORBELL_STRIDE: u64 = 4;

/// How many I/O queue pairs this controller supports at most.
///
/// Not a specification limit — the specification's is 65534 — but the size of
/// the doorbell array this model allocates. A machine file may ask for fewer.
pub const MAX_IO_QUEUES: u16 = 8;

/// Queue identifiers 0 (admin) through [`MAX_IO_QUEUES`].
const QUEUE_SLOTS: usize = MAX_IO_QUEUES as usize + 1;

/// Bytes in a submission queue entry (§4.2).
const SQE_LEN: u64 = 64;

/// Bytes in a completion queue entry (§4.6).
const CQE_LEN: u64 = 16;

/// The largest I/O queue this controller will create, in entries.
///
/// Reported as `CAP.MQES` minus one, which is how §3.1.1 spells a zero-based
/// maximum.
const MAX_QUEUE_ENTRIES: u32 = 1024;

/// The largest admin queue, in entries. §3.1.8 caps `AQA` at 4096.
const MAX_ADMIN_ENTRIES: u32 = 4096;

/// `CAP.TO`, the worst-case time to `CSTS.RDY` in 500 ms units (§3.1.1).
const CAP_TIMEOUT: u32 = 1;

/// Maximum Data Transfer Size, in `2^(12 + CAP.MPSMIN)` units (§5.15.2.2).
///
/// Ten, so 4 MiB, which is larger than a real part usually reports and is
/// chosen for a reason: with 4 KiB pages a single PRP List page describes 2 MiB,
/// so a controller whose limit is under that **can never chain one list to the
/// next** and §4.3's chaining rule becomes code no test and no fuzzer can
/// reach. A limit above it makes the chain reachable, and
/// `a_prp_list_that_points_at_itself_terminates` reaches it.
///
/// It costs nothing in memory: a data transfer is moved a PRP entry at a time
/// straight between the medium and guest memory, so the largest allocation any
/// command causes is [`MAX_STRUCTURE`], not this.
const MDTS: u8 = 10;

/// What [`MDTS`] means in bytes, with `CAP.MPSMIN` zero: 4 MiB.
const MAX_TRANSFER: u64 = 4096 << MDTS;

/// The largest data structure a command that *builds* one may ask for.
///
/// `Identify` is 4 KiB and the longest log page here is a few hundred bytes, so
/// a request past this is a driver asking for something that does not exist —
/// and unlike a data transfer, this one is a host allocation, which is why it
/// has a bound of its own well below [`MAX_TRANSFER`].
const MAX_STRUCTURE: u64 = 64 * 1024;

/// How much of a Write Zeroes is issued to the medium at a time.
const ZERO_CHUNK: u64 = 64 * 1024;

/// How many PRP List pages one command's chain may occupy before this
/// controller calls it a data transfer error.
///
/// A list's last entry may point at another list (§4.3), so a guest can close a
/// ring. [`MAX_TRANSFER`] over the smallest page size needs one list; sixty-four
/// is far past anything legitimate and is a bound rather than a behaviour.
const MAX_PRP_LISTS: u32 = 64;

/// How many commands one entry into [`Controller::run`] executes before it
/// stands down.
///
/// The queues can hold at most this many commands between them, so a
/// legitimate driver can never reach it — reaching it means the controller's
/// own data transfers have been ringing its doorbells, which is what a guest
/// that points a PRP entry at `SQyTDBL` does, and it is the one way a doorbell
/// write could otherwise become unbounded work inside a single guest
/// instruction. Whatever is left is picked up by the next doorbell write,
/// exactly as a command left behind by a full completion queue is.
const MAX_COMMANDS_PER_RUN: u32 = QUEUE_SLOTS as u32 * MAX_ADMIN_ENTRIES;

/// The version this controller reports in `VS`: 1.4.0 (§3.1.2 — major in bits
/// 31:16, minor in 15:8, tertiary in 7:0).
const VERSION: u32 = 0x0001_0400;

// -- register offsets (§3.1) ------------------------------------------------

/// Controller Capabilities. 64 bits, read-only.
const REG_CAP: u64 = 0x00;
/// Version. 32 bits, read-only.
const REG_VS: u64 = 0x08;
/// Interrupt Mask Set. 32 bits, write-1-to-set.
const REG_INTMS: u64 = 0x0c;
/// Interrupt Mask Clear. 32 bits, write-1-to-clear.
const REG_INTMC: u64 = 0x10;
/// Controller Configuration. 32 bits.
const REG_CC: u64 = 0x14;
/// Controller Status. 32 bits.
const REG_CSTS: u64 = 0x1c;
/// NVM Subsystem Reset. 32 bits, write-only.
const REG_NSSR: u64 = 0x20;
/// Admin Queue Attributes. 32 bits.
const REG_AQA: u64 = 0x24;
/// Admin Submission Queue Base Address. 64 bits.
const REG_ASQ: u64 = 0x28;
/// Admin Completion Queue Base Address. 64 bits.
const REG_ACQ: u64 = 0x30;

// -- CC (§3.1.5) ------------------------------------------------------------

/// `CC.EN`: the controller may process commands.
const CC_EN: u32 = 1 << 0;
/// `CC.CSS`, bits 6:4: which command set.
const CC_CSS_SHIFT: u32 = 4;
/// `CC.MPS`, bits 11:7: the host memory page size is `2^(12 + MPS)`.
const CC_MPS_SHIFT: u32 = 7;
/// `CC.SHN`, bits 16:15: shutdown notification.
const CC_SHN_SHIFT: u32 = 15;
/// `CC.IOSQES`, bits 20:17: `2^n` bytes per submission queue entry.
const CC_IOSQES_SHIFT: u32 = 17;
/// `CC.IOCQES`, bits 24:21: `2^n` bytes per completion queue entry.
const CC_IOCQES_SHIFT: u32 = 21;
/// Which bits of `CC` this controller implements; §3.1.5 hardwires the rest.
const CC_MASK: u32 = 0x01ff_f8f1;

// -- CSTS (§3.1.6) ----------------------------------------------------------

/// `CSTS.RDY`: the controller is ready for commands.
const CSTS_RDY: u32 = 1 << 0;
/// `CSTS.CFS`: controller fatal status. Nothing but a reset clears it.
const CSTS_CFS: u32 = 1 << 1;
/// `CSTS.SHST`, bits 3:2: shutdown status. `10b` is "shutdown complete".
const CSTS_SHST_COMPLETE: u32 = 0b10 << 2;
/// The mask of `CSTS.SHST`.
const CSTS_SHST_MASK: u32 = 0b11 << 2;

// -- status codes (§4.6.1, Figures 126-128) ---------------------------------

/// Successful completion.
const ST_SUCCESS: u16 = 0x0000;
/// Generic: Invalid Command Opcode.
const ST_INVALID_OPCODE: u16 = 0x0001;
/// Generic: Invalid Field in Command.
const ST_INVALID_FIELD: u16 = 0x0002;
/// Generic: Data Transfer Error — the controller could not move the bytes.
const ST_DATA_TRANSFER: u16 = 0x0004;
/// Generic: Invalid Namespace or Format.
const ST_INVALID_NAMESPACE: u16 = 0x000b;
/// Generic: PRP Offset Invalid.
const ST_PRP_OFFSET: u16 = 0x0013;
/// Generic: LBA Out of Range.
const ST_LBA_RANGE: u16 = 0x0080;
/// Command specific: Completion Queue Invalid.
const ST_CQ_INVALID: u16 = 0x0100;
/// Command specific: Invalid Queue Identifier.
const ST_INVALID_QID: u16 = 0x0101;
/// Command specific: Invalid Queue Size.
const ST_INVALID_QSIZE: u16 = 0x0102;
/// Command specific: Asynchronous Event Request Limit Exceeded.
const ST_AER_LIMIT: u16 = 0x0105;
/// Command specific: Invalid Interrupt Vector.
const ST_INVALID_VECTOR: u16 = 0x0108;
/// Command specific: Invalid Log Page.
const ST_INVALID_LOG_PAGE: u16 = 0x0109;
/// Command specific: Invalid Queue Deletion — a completion queue still has
/// submission queues associated with it.
const ST_INVALID_QUEUE_DELETION: u16 = 0x010c;
/// Media and data integrity: Write Fault.
const ST_WRITE_FAULT: u16 = 0x0280;
/// Media and data integrity: Unrecovered Read Error.
const ST_UNRECOVERED_READ: u16 = 0x0281;

// -- opcodes ----------------------------------------------------------------

/// Admin: Delete I/O Submission Queue (§5.6).
const ADMIN_DELETE_SQ: u8 = 0x00;
/// Admin: Create I/O Submission Queue (§5.4).
const ADMIN_CREATE_SQ: u8 = 0x01;
/// Admin: Get Log Page (§5.14).
const ADMIN_GET_LOG_PAGE: u8 = 0x02;
/// Admin: Delete I/O Completion Queue (§5.5).
const ADMIN_DELETE_CQ: u8 = 0x04;
/// Admin: Create I/O Completion Queue (§5.3).
const ADMIN_CREATE_CQ: u8 = 0x05;
/// Admin: Identify (§5.15).
const ADMIN_IDENTIFY: u8 = 0x06;
/// Admin: Abort (§5.1).
const ADMIN_ABORT: u8 = 0x08;
/// Admin: Set Features (§5.21).
const ADMIN_SET_FEATURES: u8 = 0x09;
/// Admin: Get Features (§5.15 in 1.3, §5.21's companion).
const ADMIN_GET_FEATURES: u8 = 0x0a;
/// Admin: Asynchronous Event Request (§5.2).
const ADMIN_ASYNC_EVENT: u8 = 0x0c;

/// NVM: Flush (§6.8).
const NVM_FLUSH: u8 = 0x00;
/// NVM: Write (§6.15).
const NVM_WRITE: u8 = 0x01;
/// NVM: Read (§6.9).
const NVM_READ: u8 = 0x02;
/// NVM: Write Zeroes (§6.16).
const NVM_WRITE_ZEROES: u8 = 0x08;

/// Feature identifier 07h, Number of Queues (§5.21.1.7).
const FEATURE_NUM_QUEUES: u8 = 0x07;
/// Feature identifier 0Bh, Asynchronous Event Configuration.
const FEATURE_ASYNC_CONFIG: u8 = 0x0b;

/// How many Asynchronous Event Requests may be outstanding: `AERL + 1`, and
/// `AERL` is reported as zero (§5.15.2.2's zero-based field).
const AER_LIMIT: u32 = 1;

/// Bytes in an Identify data structure (§5.15).
const IDENTIFY_LEN: u64 = 4096;

// ---------------------------------------------------------------------------
// the namespace
// ---------------------------------------------------------------------------

/// The one namespace this controller presents, and the bytes behind it.
///
/// The medium is [`dev::ata::Medium`](crate::dev::ata::Medium) — the seam a
/// [`RamStore`](crate::core::space::RamStore) and a
/// [`blk::Image`](crate::dev::blk) both satisfy — so an NVMe namespace is a
/// qcow2 file for exactly the same reason an ATA drive is, and neither module
/// parses an image format (`ROADMAP.md` §7.1).
pub struct Namespace {
    media: Arc<dyn Medium>,
    /// `2^lba_shift` bytes per logical block: the `LBADS` of LBA Format 0
    /// (§5.15.2.1).
    lba_shift: u32,
    blocks: u64,
    read_only: bool,
}

impl fmt::Debug for Namespace {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Namespace")
            .field("blocks", &self.blocks)
            .field("lba_bytes", &self.lba_bytes())
            .field("read_only", &self.read_only)
            .finish()
    }
}

impl Namespace {
    /// A namespace on `media`, in blocks of `2^lba_shift` bytes.
    ///
    /// # Errors
    ///
    /// [`Error::Config`](crate::Error::Config) if the block size is not one
    /// this model supports, or if the medium does not hold a whole number of
    /// blocks — an `Identify Namespace` reporting a capacity the medium cannot
    /// serve is a lie a guest finds out about at the last sector.
    pub fn new(media: Arc<dyn Medium>, lba_shift: u32, read_only: bool) -> Result<Namespace> {
        if !(9..=12).contains(&lba_shift) {
            return Err(crate::core::error::Error::Config {
                at: String::from("nvme.namespace"),
                message: alloc::format!(
                    "a logical block is 512, 1024, 2048 or 4096 bytes, not 2^{lba_shift}"
                ),
            });
        }
        let bytes = media.capacity();
        let lba = 1u64 << lba_shift;
        if bytes == 0 || !bytes.is_multiple_of(lba) {
            return Err(crate::core::error::Error::Config {
                at: String::from("nvme.namespace"),
                message: alloc::format!(
                    "a namespace holds a whole number of {lba}-byte blocks, and {bytes} bytes is \
                     not a whole number of them"
                ),
            });
        }
        Ok(Namespace {
            read_only: read_only || media.is_read_only(),
            media,
            lba_shift,
            blocks: bytes / lba,
        })
    }

    /// Bytes in one logical block.
    #[must_use]
    pub fn lba_bytes(&self) -> u64 {
        1 << self.lba_shift
    }

    /// How many logical blocks the namespace holds.
    #[must_use]
    pub fn blocks(&self) -> u64 {
        self.blocks
    }

    /// The medium behind it, for a host that wants to check what a guest wrote
    /// without going back through the controller.
    #[must_use]
    pub fn medium(&self) -> &Arc<dyn Medium> {
        &self.media
    }

    /// Whether writes are refused.
    #[must_use]
    pub fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// Every byte on the medium, for a [`Snapshot::Capture`] chunk.
    ///
    /// # Errors
    ///
    /// [`Error::State`](crate::Error::State) if the medium could not be read.
    pub fn contents(&self) -> Result<Vec<u8>> {
        let mut out = vec![0u8; self.media.capacity() as usize];
        self.media
            .read_at(0, &mut out)
            .map_err(|e| crate::core::error::Error::State(alloc::format!("nvme namespace: {e}")))?;
        Ok(out)
    }

    /// What a machine snapshot should do about these bytes.
    #[must_use]
    pub fn snapshot(&self) -> Snapshot {
        self.media.snapshot()
    }

    /// The medium's own one-line identity, for a [`Snapshot::Reference`] chunk.
    #[must_use]
    pub fn describe(&self) -> String {
        self.media.describe()
    }

    /// Make every write durable — what a `Flush` command and a snapshot both
    /// ask for.
    ///
    /// # Errors
    ///
    /// [`Error::State`](crate::Error::State) if the host refused.
    pub fn flush(&self) -> Result<()> {
        self.media
            .flush()
            .map_err(|e| crate::core::error::Error::State(alloc::format!("nvme namespace: {e}")))
    }

    /// Put `bytes` back on the medium, restoring a captured snapshot.
    ///
    /// # Errors
    ///
    /// [`Error::State`](crate::Error::State) if the medium refused them.
    pub fn restore(&self, bytes: &[u8]) -> Result<()> {
        self.media
            .write_at(0, bytes)
            .map_err(|e| crate::core::error::Error::State(alloc::format!("nvme namespace: {e}")))
    }

    /// The byte range `count` blocks at `slba` covers, or `None` if it runs off
    /// the end of the namespace.
    ///
    /// In `u64` throughout and checked before anything becomes a host index:
    /// this is where the 64-bit-guest-on-a-32-bit-host rule bites
    /// (`docs/buses/storage.md`).
    fn range(&self, slba: u64, count: u64) -> Option<(u64, u64)> {
        let end = slba.checked_add(count)?;
        if end > self.blocks {
            return None;
        }
        Some((slba << self.lba_shift, count << self.lba_shift))
    }
}

// ---------------------------------------------------------------------------
// queues
// ---------------------------------------------------------------------------

/// One submission queue: where it is, how big, and how far each end has got.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct SubQueue {
    base: u64,
    entries: u32,
    head: u32,
    tail: u32,
    cqid: u16,
}

/// One completion queue, plus the phase tag the driver spins on (§4.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct CompQueue {
    base: u64,
    entries: u32,
    head: u32,
    tail: u32,
    /// The phase bit the *next* entry written carries. Starts set, because a
    /// driver zeroes the queue and waits for a 1 (§4.6).
    phase: bool,
    vector: u16,
    interrupts: bool,
}

impl CompQueue {
    /// Whether one more entry fits. §4.1: the queue is full when the tail is
    /// one behind the head.
    fn has_room(&self) -> bool {
        self.entries > 0 && (self.tail + 1) % self.entries != self.head
    }

    /// Whether the host has entries it has not acknowledged.
    fn pending(&self) -> bool {
        self.head != self.tail
    }
}

/// Everything a snapshot has to carry.
#[derive(Debug, Clone, Copy)]
struct State {
    cc: u32,
    csts: u32,
    intms: u32,
    aqa: u32,
    asq: u64,
    acq: u64,
    /// Outstanding Asynchronous Event Requests. They are never completed —
    /// nothing generates an event — which is what real silicon does with them.
    aer: u32,
    sq: [Option<SubQueue>; QUEUE_SLOTS],
    cq: [Option<CompQueue>; QUEUE_SLOTS],
}

impl State {
    /// The reset state: no queues, nothing enabled (§3.1.5).
    const fn new() -> State {
        State {
            cc: 0,
            csts: 0,
            intms: 0,
            aqa: 0,
            asq: 0,
            acq: 0,
            aer: 0,
            sq: [None; QUEUE_SLOTS],
            cq: [None; QUEUE_SLOTS],
        }
    }

    /// The host memory page size `CC.MPS` currently names (§3.1.5).
    fn page(&self) -> u64 {
        1u64 << (12 + ((self.cc >> CC_MPS_SHIFT) & 0xf))
    }

    /// Forget every queue and clear `CSTS.RDY`: a Controller Reset (§3.1.5).
    ///
    /// `AQA`, `ASQ` and `ACQ` are left alone. A driver rewrites them before
    /// every enable, so no conforming one can tell the difference, and keeping
    /// them makes a disable/enable cycle without a rewrite work rather than
    /// fail silently.
    fn controller_reset(&mut self) {
        self.sq = [None; QUEUE_SLOTS];
        self.cq = [None; QUEUE_SLOTS];
        self.aer = 0;
        self.csts &= !(CSTS_RDY | CSTS_SHST_MASK);
    }
}

// ---------------------------------------------------------------------------
// the controller
// ---------------------------------------------------------------------------

/// What the transport hands the controller once the machine is wired.
struct Link {
    /// The space this controller masters.
    ///
    /// `Weak`, like every bus master's handle: the machine owns the space, and
    /// a device that kept its own space alive would close a cycle nothing could
    /// drop.
    space: Option<Weak<AddressSpace>>,
    irq: Option<WireSource>,
}

impl fmt::Debug for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Link")
            .field("space", &self.space.is_some())
            .field("irq", &self.irq.is_some())
            .finish()
    }
}

/// How a controller was configured by its machine description.
#[derive(Debug, Clone)]
pub struct Params {
    /// The PCI vendor identification, which `Identify Controller` repeats.
    pub vendor: u16,
    /// The PCI device identification, which nothing repeats but which is here
    /// for symmetry with the subsystem vendor id.
    pub subsystem_vendor: u16,
    /// The serial number, 20 ASCII bytes in `Identify Controller` (§5.15.2.2).
    pub serial: String,
    /// The model number, 40 ASCII bytes.
    pub model: String,
    /// The firmware revision, 8 ASCII bytes.
    pub firmware: String,
    /// How many I/O queue pairs the controller allocates, 1-[`MAX_IO_QUEUES`].
    pub io_queues: u16,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            vendor: 0,
            subsystem_vendor: 0,
            serial: String::from("RSEMU0000000000000001"),
            model: String::from("RSEMU NVME CONTROLLER"),
            firmware: String::from("1.0"),
            io_queues: 4,
        }
    }
}

/// An NVM Express controller: the register block, the queues and the commands.
pub struct Controller {
    ns: Namespace,
    params: Params,
    /// [`NVME_RANK`]. Never held across a guest-memory access, a medium access
    /// or a wire change.
    state: Mutex<State>,
    /// The transport's contributions, at [`LockRank::WIRE`]: cloned out and the
    /// guard dropped before any of them is used.
    link: Mutex<Link>,
    /// The identity this controller's own accesses carry, so that a bus fault
    /// names the master rather than the CPU that rang the doorbell.
    requester: AtomicU32,
    /// The level the interrupt output is being held at, so the PCI Status
    /// register's Interrupt Status bit is free to read.
    irq_level: AtomicU32,
    /// `COMMAND[2]`, Bus Master Enable. A function that may not master the bus
    /// does not fetch commands (*PCI Local Bus Specification* Rev 2.1 §6.2.2).
    master: AtomicBool,
    /// `COMMAND[10]`, Interrupt Disable (*PCI Local Bus Specification* Rev 3.0
    /// §6.2.2). Set, and the function drives no `INTx#` — but the Status
    /// register's Interrupt Status bit still reports the internal state.
    intx_disabled: AtomicBool,
    /// Whether [`Controller::run`] is already on the stack. See the module
    /// documentation: a PRP entry may address this controller's own doorbells.
    busy: AtomicBool,
}

impl fmt::Debug for Controller {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Controller");
        s.field("namespace", &self.ns);
        match self.state.try_lock() {
            Some(state) => s
                .field("cc", &state.cc)
                .field("csts", &state.csts)
                .finish_non_exhaustive(),
            None => s.field("state", &"<in use>").finish_non_exhaustive(),
        }
    }
}

impl Controller {
    /// A controller presenting `ns`, configured by `params`.
    #[must_use]
    pub fn new(ns: Namespace, params: Params) -> Controller {
        let params = Params {
            io_queues: params.io_queues.clamp(1, MAX_IO_QUEUES),
            ..params
        };
        Controller {
            ns,
            params,
            state: Mutex::with_rank(NVME_RANK, State::new()),
            link: Mutex::with_rank(
                LockRank::WIRE,
                Link {
                    space: None,
                    irq: None,
                },
            ),
            requester: AtomicU32::new(RequesterId::ANONYMOUS.0),
            irq_level: AtomicU32::new(0),
            master: AtomicBool::new(false),
            intx_disabled: AtomicBool::new(false),
            busy: AtomicBool::new(false),
        }
    }

    /// The namespace it presents.
    #[must_use]
    pub fn namespace(&self) -> &Namespace {
        &self.ns
    }

    /// How it was configured.
    #[must_use]
    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Give the controller the address space its queues live in, and the
    /// identity its own accesses carry.
    pub fn attach_space(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        let mut link = self.link.lock();
        link.space = Some(Arc::downgrade(space));
        drop(link);
        self.requester.store(requester.0, Ordering::Relaxed);
    }

    /// Connect the `INTx#` output.
    pub fn connect_irq(&self, source: WireSource) {
        self.link.lock().irq = Some(source);
        self.refresh_irq();
    }

    /// Whether the function may master the bus (`COMMAND[2]`).
    pub fn set_master(&self, enabled: bool) {
        self.master.store(enabled, Ordering::Relaxed);
    }

    /// Whether the function's `INTx#` emission is disabled (`COMMAND[10]`).
    pub fn set_intx_disabled(&self, disabled: bool) {
        self.intx_disabled.store(disabled, Ordering::Relaxed);
        self.refresh_irq();
    }

    /// Whether the controller has an interrupt condition, whatever
    /// `COMMAND[10]` says about emitting it.
    ///
    /// This is what the PCI Status register's Interrupt Status bit reports
    /// (Rev 3.0 §6.2.3), and reading it must not disturb anything — which is
    /// why it is an atomic rather than a walk of the queues.
    #[must_use]
    pub fn interrupt_pending(&self) -> bool {
        self.irq_level.load(Ordering::Relaxed) != 0
    }

    /// The level the `INTx#` output is being driven to.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.interrupt_pending() && !self.intx_disabled.load(Ordering::Relaxed))
    }

    /// The space this controller masters, if it still exists.
    fn space(&self) -> Option<Arc<AddressSpace>> {
        // Cloned out and the guard dropped before the caller touches it: the
        // space's own topology lock ranks above everything here, so holding
        // this across an access would be a ladder violation as well as a
        // re-entrancy one.
        self.link.lock().space.as_ref().and_then(Weak::upgrade)
    }

    /// The attributes this controller's own accesses carry.
    fn attrs(&self) -> MemAttrs {
        MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)))
    }
}

/// Assemble a little-endian `u32` from a slice, which is how every NVMe
/// structure is defined (§1.5: "all values are in little endian").
fn le32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

/// Assemble a little-endian `u64`.
fn le64(bytes: &[u8]) -> u64 {
    let mut out = [0u8; 8];
    out.copy_from_slice(&bytes[..8]);
    u64::from_le_bytes(out)
}

/// Copy `text` into `dst` as the space-padded ASCII an NVMe identify string is
/// (§1.5: strings are left justified and padded with spaces).
fn ascii(dst: &mut [u8], text: &str) {
    dst.fill(b' ');
    for (slot, byte) in dst.iter_mut().zip(text.bytes()) {
        // A non-ASCII byte would make the field something no driver can print.
        *slot = if byte.is_ascii_graphic() || byte == b' ' {
            byte
        } else {
            b' '
        };
    }
}

// ---------------------------------------------------------------------------
// the command
// ---------------------------------------------------------------------------

/// A submission queue entry, as the fields this controller reads (§4.2).
#[derive(Debug, Clone, Copy)]
struct Command {
    opcode: u8,
    /// `CDW0[15:14]`, PRP or SGL for Data Transfer. Non-zero means SGLs, which
    /// this controller reports it does not support (`Identify Controller`'s
    /// `SGLS` is zero, §5.15.2.2).
    psdt: u8,
    cid: u16,
    nsid: u32,
    prp1: u64,
    prp2: u64,
    cdw10: u32,
    cdw11: u32,
    cdw12: u32,
}

impl Command {
    /// Decode the fields this controller uses out of the 64 bytes.
    fn parse(raw: &[u8; SQE_LEN as usize]) -> Command {
        let cdw0 = le32(&raw[0..4]);
        Command {
            opcode: cdw0 as u8,
            psdt: ((cdw0 >> 14) & 0x3) as u8,
            cid: (cdw0 >> 16) as u16,
            nsid: le32(&raw[4..8]),
            prp1: le64(&raw[24..32]),
            prp2: le64(&raw[32..40]),
            cdw10: le32(&raw[40..44]),
            cdw11: le32(&raw[44..48]),
            cdw12: le32(&raw[48..52]),
        }
    }
}

/// One entry picked off a submission queue, ready to run.
#[derive(Debug, Clone, Copy)]
struct Job {
    sqid: u16,
    cqid: u16,
    /// Where the 64-byte entry is in guest memory.
    addr: u64,
    /// The submission queue head *after* consuming it, which is what the
    /// completion reports back (§4.6, `SQHD`).
    sqhd: u32,
}

/// What a register write asks for once the state lock is released.
///
/// Every one of these is an outward action — a wire change, a medium flush, or
/// a walk of guest memory — and none of them may happen under the lock
/// (`CLAUDE.md`, re-entrancy).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum After {
    /// Refresh the interrupt output and nothing else.
    Irq,
    /// The host asked for a shutdown: make the medium durable, then say so.
    Shutdown,
    /// Commands may be waiting.
    Run,
}

// ---------------------------------------------------------------------------
// registers, and the engine behind the doorbells
// ---------------------------------------------------------------------------

impl Controller {
    /// Re-derive the `INTx#` level from the completion queues and drive it.
    ///
    /// §7.5.1.1: with pin-based interrupts the controller asserts the pin while
    /// any completion queue with interrupts enabled holds an entry the host has
    /// not acknowledged and whose vector is not masked, and deasserts it once
    /// the host's completion queue head doorbell writes have caught up. That is
    /// a *level*, which is why this is recomputed rather than pulsed.
    pub fn refresh_irq(&self) {
        let pending = {
            let state = self.state.lock();
            state.csts & CSTS_RDY != 0
                && state.cq.iter().flatten().any(|cq| {
                    cq.interrupts && cq.pending() && state.intms & (1 << (cq.vector & 31)) == 0
                })
        };
        self.irq_level.store(u32::from(pending), Ordering::Relaxed);
        let level = self.irq_level();
        // The state lock is released before the wire is touched: an observer
        // may drive another wire from inside `set`, and the ladder runs
        // `NVME_RANK` -> `WIRE`, never back.
        let out = self.link.lock().irq.clone();
        if let Some(out) = out {
            out.set(level);
        }
    }

    /// Back to the power-on state: `PCIRST#`, or the machine's reset.
    pub fn reset(&self) {
        *self.state.lock() = State::new();
        self.busy.store(false, Ordering::Relaxed);
        self.master.store(false, Ordering::Relaxed);
        self.intx_disabled.store(false, Ordering::Relaxed);
        self.refresh_irq();
    }

    /// `CAP`, the capabilities this model actually has (§3.1.1).
    fn cap(&self) -> u64 {
        // MQES is zero-based; CQR is set because only physically contiguous
        // queues are supported, which is also why Create I/O Queue refuses
        // `PC` clear; AMS is zero, round robin only; TO is in 500 ms units.
        let low = u64::from(MAX_QUEUE_ENTRIES - 1) | (1 << 16) | (u64::from(CAP_TIMEOUT) << 24);
        // In the upper dword: DSTRD 0 (a 4-byte stride), CSS bit 0 at CAP[37]
        // for the NVM command set, MPSMIN 0 (4 KiB) and MPSMAX 4 (64 KiB).
        let high = (1u64 << 5) | (4 << 20);
        low | (high << 32)
    }

    /// The 32 bits a read of the register at `offset` sees.
    ///
    /// Side-effect free at every offset, which is what makes
    /// [`MemAttrs::debug`] cheap on the read side: §3.1 has no read-to-clear
    /// register, and the doorbells are write-only — a read of one is undefined
    /// and this model answers zero rather than handing the driver's own tail
    /// back to it.
    fn read_dword(&self, offset: u64) -> u32 {
        match offset {
            REG_CAP => self.cap() as u32,
            0x04 => (self.cap() >> 32) as u32,
            REG_VS => VERSION,
            // Both mask registers read back the mask itself (§3.1.3, §3.1.4).
            REG_INTMS | REG_INTMC => self.state.lock().intms,
            REG_CC => self.state.lock().cc,
            REG_CSTS => self.state.lock().csts,
            REG_AQA => self.state.lock().aqa,
            REG_ASQ => self.state.lock().asq as u32,
            0x2c => (self.state.lock().asq >> 32) as u32,
            REG_ACQ => self.state.lock().acq as u32,
            0x34 => (self.state.lock().acq >> 32) as u32,
            // Write-only (§3.1.9), and `CAP.NSSRS` is clear anyway, so there is
            // no subsystem reset to ask for.
            REG_NSSR => 0,
            // Everything else here is reserved, and reads as zero.
            _ => 0,
        }
    }

    /// Take a 32-bit register write.
    fn write_dword(&self, offset: u64, value: u32) {
        if offset >= DOORBELL_BASE {
            self.doorbell(offset, value);
            return;
        }
        let after = {
            let mut state = self.state.lock();
            match offset {
                REG_INTMS => {
                    state.intms |= value;
                    After::Irq
                }
                REG_INTMC => {
                    state.intms &= !value;
                    After::Irq
                }
                REG_CC => Controller::write_cc(&mut state, value),
                // §3.1.8: the admin queue registers are written while the
                // controller is disabled. A write with `CC.EN` set is
                // undefined, and ignoring it is the answer that cannot corrupt
                // a running controller.
                REG_AQA if state.cc & CC_EN == 0 => {
                    state.aqa = value & 0x0fff_0fff;
                    After::Irq
                }
                REG_ASQ if state.cc & CC_EN == 0 => {
                    state.asq = (state.asq & 0xffff_ffff_0000_0000) | u64::from(value);
                    After::Irq
                }
                0x2c if state.cc & CC_EN == 0 => {
                    state.asq = (state.asq & 0xffff_ffff) | (u64::from(value) << 32);
                    After::Irq
                }
                REG_ACQ if state.cc & CC_EN == 0 => {
                    state.acq = (state.acq & 0xffff_ffff_0000_0000) | u64::from(value);
                    After::Irq
                }
                0x34 if state.cc & CC_EN == 0 => {
                    state.acq = (state.acq & 0xffff_ffff) | (u64::from(value) << 32);
                    After::Irq
                }
                // `CSTS` is read-only here — the one write-1-to-clear bit in it
                // is `NSSRO`, and `CAP.NSSRS` says this subsystem has no reset
                // to report. `NSSR` itself is ignored for the same reason
                // (§3.1.9). Everything else is reserved.
                _ => After::Irq,
            }
        };
        match after {
            After::Irq => self.refresh_irq(),
            After::Run => {
                self.run();
                self.refresh_irq();
            }
            After::Shutdown => self.shutdown(),
        }
    }

    /// `CC` (§3.1.5), which is where a controller starts and stops.
    fn write_cc(state: &mut State, value: u32) -> After {
        let value = value & CC_MASK;
        let shn = (value >> CC_SHN_SHIFT) & 0x3;
        let enabled = state.cc & CC_EN != 0;
        let wanted = value & CC_EN != 0;
        match (enabled, wanted) {
            (false, true) => {
                state.cc = value;
                Controller::enable(state);
                After::Irq
            }
            (true, false) => {
                // A Controller Reset. Every queue goes, and `CSTS.RDY` follows.
                state.cc = value;
                state.controller_reset();
                After::Irq
            }
            (true, true) => {
                // Only the shutdown notification may move while the controller
                // is enabled; §3.1.5 leaves the rest undefined, and this model
                // keeps what it had rather than reconfiguring underneath a
                // driver that is mid-flight.
                const SHN_MASK: u32 = 0x3 << CC_SHN_SHIFT;
                state.cc = (state.cc & !SHN_MASK) | (value & SHN_MASK);
                if shn != 0 {
                    After::Shutdown
                } else {
                    After::Irq
                }
            }
            (false, false) => {
                state.cc = value;
                After::Irq
            }
        }
    }

    /// `CC.EN` went from 0 to 1: check the configuration and come ready.
    ///
    /// §3.1.5 is explicit that a controller which cannot accept the
    /// configuration must **not** set `CSTS.RDY`, and §3.1.6 gives it
    /// `CSTS.CFS` to say why. Coming ready anyway and failing later would leave
    /// a driver waiting on a queue the controller never built.
    fn enable(state: &mut State) {
        let iosqes = (state.cc >> CC_IOSQES_SHIFT) & 0xf;
        let iocqes = (state.cc >> CC_IOCQES_SHIFT) & 0xf;
        let mps = (state.cc >> CC_MPS_SHIFT) & 0xf;
        let css = (state.cc >> CC_CSS_SHIFT) & 0x7;
        // `ASQS` and `ACQS` are zero-based, and §3.1.8 makes 2 the minimum
        // queue size — so a stored zero means one entry, which is illegal.
        let asqs = (state.aqa & 0xfff) + 1;
        let acqs = ((state.aqa >> 16) & 0xfff) + 1;
        let page = 1u64 << (12 + mps);
        let ok = iosqes == 6
            && iocqes == 4
            && css == 0
            && mps <= 4
            && asqs >= 2
            && acqs >= 2
            && asqs <= MAX_ADMIN_ENTRIES
            && acqs <= MAX_ADMIN_ENTRIES
            && state.asq != 0
            && state.acq != 0
            && state.asq.is_multiple_of(page)
            && state.acq.is_multiple_of(page);
        if !ok {
            state.csts |= CSTS_CFS;
            return;
        }
        state.sq[0] = Some(SubQueue {
            base: state.asq,
            entries: asqs,
            head: 0,
            tail: 0,
            cqid: 0,
        });
        state.cq[0] = Some(CompQueue {
            base: state.acq,
            entries: acqs,
            head: 0,
            tail: 0,
            // §4.6: a driver zeroes the queue and waits for the phase tag to
            // become 1, so the first entry the controller writes carries 1.
            phase: true,
            vector: 0,
            // §5.3: the admin completion queue's interrupts are always enabled.
            interrupts: true,
        });
        state.csts |= CSTS_RDY;
    }

    /// The host set `CC.SHN`: flush, then report the shutdown complete.
    ///
    /// The flush is the point. §3.1.5's shutdown is how a host says "I am about
    /// to lose power", and a controller that acknowledged it without making its
    /// writes durable would be lying about the one thing the notification is
    /// for.
    fn shutdown(&self) {
        let _ = self.ns.media.flush();
        {
            let mut state = self.state.lock();
            state.csts = (state.csts & !CSTS_SHST_MASK) | CSTS_SHST_COMPLETE;
        }
        self.refresh_irq();
    }

    /// A doorbell write (§3.1.11, §3.1.12).
    fn doorbell(&self, offset: u64, value: u32) {
        let byte = offset - DOORBELL_BASE;
        if !byte.is_multiple_of(DOORBELL_STRIDE) {
            // Not on a stride boundary, so not a doorbell.
            return;
        }
        let index = byte / DOORBELL_STRIDE;
        let qid = (index / 2) as usize;
        let is_cq = index % 2 == 1;
        if qid >= QUEUE_SLOTS {
            return;
        }
        let after = {
            let mut state = self.state.lock();
            if state.csts & CSTS_RDY == 0 {
                // Nothing is running, so there is no doorbell to ring.
                return;
            }
            let slot = if is_cq {
                state.cq[qid].map(|cq| cq.entries)
            } else {
                state.sq[qid].map(|sq| sq.entries)
            };
            match slot {
                // §4.1: an invalid doorbell value is a fatal condition, and
                // `CSTS.CFS` is how the controller says so.
                Some(entries) if value >= entries => {
                    state.csts |= CSTS_CFS;
                    After::Irq
                }
                Some(_) if is_cq => {
                    if let Some(cq) = state.cq[qid].as_mut() {
                        cq.head = value;
                    }
                    // `Run`, not just `Irq`: making room in a completion queue
                    // is what *releases* a command [`Controller::pick`] left on
                    // its submission queue because there was nowhere to put the
                    // answer. A model that only ran on a submission doorbell
                    // would strand that command until the driver happened to
                    // submit another one.
                    After::Run
                }
                Some(_) => {
                    if let Some(sq) = state.sq[qid].as_mut() {
                        sq.tail = value;
                    }
                    After::Run
                }
                // A doorbell for a queue that does not exist is ignored: there
                // is nothing to corrupt, and a driver probing for one is not a
                // fatal event.
                None => return,
            }
        };
        match after {
            After::Run => {
                self.run();
                self.refresh_irq();
            }
            _ => self.refresh_irq(),
        }
    }

    /// Whether any submission queue holds work whose completion queue has room.
    fn has_work(&self) -> bool {
        let state = self.state.lock();
        Controller::pick(&state).is_some()
    }

    /// The lowest-numbered queue with a command ready and somewhere to put the
    /// completion.
    ///
    /// Fixed priority, admin first, rather than the round robin §4.11 permits:
    /// arbitration is a scheduling policy and a deterministic one is what a
    /// reproducible machine needs (`CLAUDE.md`, determinism). A driver with one
    /// queue cannot tell the difference, and one with several sees an order
    /// rather than a race.
    fn pick(state: &State) -> Option<(usize, SubQueue)> {
        if state.csts & (CSTS_RDY | CSTS_CFS) != CSTS_RDY {
            return None;
        }
        for (qid, slot) in state.sq.iter().enumerate() {
            let Some(sq) = *slot else { continue };
            if sq.head == sq.tail || sq.entries == 0 {
                continue;
            }
            // A full completion queue is back pressure, not an error: leave the
            // command where it is and come back when the host's head doorbell
            // has made room.
            if let Some(cq) = state.cq.get(usize::from(sq.cqid)).copied().flatten()
                && cq.has_room()
            {
                return Some((qid, sq));
            }
        }
        None
    }

    /// Consume one entry, advancing the submission queue head.
    fn next_job(&self) -> Option<Job> {
        let mut state = self.state.lock();
        let (qid, sq) = Controller::pick(&state)?;
        let addr = sq.base + u64::from(sq.head) * SQE_LEN;
        let head = (sq.head + 1) % sq.entries;
        if let Some(slot) = state.sq[qid].as_mut() {
            slot.head = head;
        }
        Some(Job {
            sqid: qid as u16,
            cqid: sq.cqid,
            addr,
            sqhd: head,
        })
    }

    /// Run every command the driver has made available.
    ///
    /// Iterative, not recursive: see the module documentation. A doorbell rung
    /// from inside one of this controller's own guest-memory accesses records
    /// its tail and returns, and the loop below picks the work up.
    pub fn run(&self) {
        // Rev 2.1 §6.2.2: a function that has not been granted Bus Master
        // Enable does not generate accesses of its own, so it cannot fetch a
        // command. A driver sets the bit before it rings a doorbell; one that
        // forgot sees nothing happen, which is what the hardware does too.
        if !self.master.load(Ordering::Relaxed) {
            return;
        }
        if self.busy.swap(true, Ordering::AcqRel) {
            return;
        }
        let mut budget = MAX_COMMANDS_PER_RUN;
        loop {
            while let Some(job) = self.next_job() {
                self.execute(&job);
                budget -= 1;
                if budget == 0 {
                    break;
                }
            }
            self.busy.store(false, Ordering::Release);
            // Another master may have rung a doorbell between the last check
            // and this release. Re-check, and take the flag back only if there
            // is something to do.
            if budget == 0 || !self.has_work() || self.busy.swap(true, Ordering::AcqRel) {
                break;
            }
        }
    }

    /// Fetch one command, run it, and post its completion.
    fn execute(&self, job: &Job) {
        let Some(space) = self.space() else {
            self.fatal();
            return;
        };
        let mut raw = [0u8; SQE_LEN as usize];
        if space.read_bytes(job.addr, &mut raw, self.attrs()).is_err() {
            // The controller cannot read its own work queue. §3.1.6's
            // Controller Fatal Status is exactly this condition, and stopping
            // is the honest answer: inventing commands out of bus faults is not.
            self.fatal();
            return;
        }
        let cmd = Command::parse(&raw);
        let done = if job.sqid == 0 {
            self.admin(&space, &cmd)
        } else {
            self.nvm(&space, &cmd)
        };
        // `None` is a command held outstanding rather than one that failed.
        if let Some((status, dw0)) = done {
            self.complete(&space, job, cmd.cid, status, dw0);
        }
    }

    /// Write one completion queue entry and advance the tail (§4.6).
    fn complete(&self, space: &AddressSpace, job: &Job, cid: u16, status: u16, dw0: u32) {
        let placed = {
            let mut state = self.state.lock();
            // `get_mut`, not an index: every route to a queue identifier
            // checks it — Create I/O Submission Queue, and the snapshot loader
            // — but this is a value that reached us from guest memory by way of
            // a queue descriptor, and a device that walks guest data should not
            // have a panic on any path at all.
            match state
                .cq
                .get_mut(usize::from(job.cqid))
                .and_then(Option::as_mut)
            {
                Some(cq) if cq.has_room() => {
                    let addr = cq.base + u64::from(cq.tail) * CQE_LEN;
                    let phase = cq.phase;
                    cq.tail += 1;
                    if cq.tail == cq.entries {
                        // §4.6: the phase tag inverts every time the tail wraps,
                        // which is how a driver knows an entry is new without
                        // the controller writing anything else.
                        cq.tail = 0;
                        cq.phase = !cq.phase;
                    }
                    Some((addr, phase))
                }
                _ => None,
            }
        };
        let Some((addr, phase)) = placed else {
            return;
        };
        let mut entry = [0u8; CQE_LEN as usize];
        entry[0..4].copy_from_slice(&dw0.to_le_bytes());
        // DW2: which submission queue, and how far its head has got.
        let dw2 = u32::from(job.sqhd as u16) | (u32::from(job.sqid) << 16);
        entry[8..12].copy_from_slice(&dw2.to_le_bytes());
        // DW3: the command identifier, the phase tag, and the status field.
        let dw3 = u32::from(cid) | (u32::from(phase) << 16) | (u32::from(status) << 17);
        entry[12..16].copy_from_slice(&dw3.to_le_bytes());
        if space.write_bytes(addr, &entry, self.attrs()).is_err() {
            self.fatal();
        }
    }

    /// The controller cannot continue (§3.1.6, `CSTS.CFS`).
    fn fatal(&self) {
        self.state.lock().csts |= CSTS_CFS;
    }
}

// ---------------------------------------------------------------------------
// physical region pages (§4.3)
// ---------------------------------------------------------------------------

impl Controller {
    /// Resolve a command's `PRP1`/`PRP2` into the guest ranges a transfer of
    /// `len` bytes covers, or the status code that says why it cannot.
    ///
    /// §4.3, and the three cases it defines:
    ///
    /// * the transfer fits in the page `PRP1` points into — `PRP2` is unused;
    /// * it needs one more page — `PRP2` *is* that page, and is page aligned;
    /// * it needs more — `PRP2` points at a **PRP List**, whose entries are
    ///   page aligned, and whose last entry points at the next list when there
    ///   is still more than one page to go.
    ///
    /// Every pointer here comes from guest memory, so a list may point at
    /// itself. [`MAX_PRP_LISTS`] is the bound that makes the walk terminate
    /// whatever the guest built — the property
    /// `fuzz/fuzz_targets/nvme_mmio.rs` exists to check.
    fn prp_chunks(
        &self,
        space: &AddressSpace,
        prp1: u64,
        prp2: u64,
        len: u64,
        page: u64,
    ) -> core::result::Result<Vec<(u64, u64)>, u16> {
        let mut out: Vec<(u64, u64)> = Vec::new();
        if len == 0 {
            return Ok(out);
        }
        // §4.3: a PRP entry for the NVM command set has a memory page offset
        // that is dword aligned, and only `PRP1` may have one at all.
        let offset = prp1 & (page - 1);
        if !offset.is_multiple_of(4) {
            return Err(ST_PRP_OFFSET);
        }
        let first = core::cmp::min(len, page - offset);
        out.push((prp1, first));
        let mut remaining = len - first;
        if remaining == 0 {
            return Ok(out);
        }
        if !prp2.is_multiple_of(page) {
            return Err(ST_PRP_OFFSET);
        }
        if remaining <= page {
            out.push((prp2, remaining));
            return Ok(out);
        }
        let per_page = page / 8;
        let mut list = prp2;
        let mut lists = 0u32;
        loop {
            lists += 1;
            if lists > MAX_PRP_LISTS {
                // A ring, or a chain longer than any real command needs.
                return Err(ST_DATA_TRANSFER);
            }
            let mut chained = false;
            for i in 0..per_page {
                let mut raw = [0u8; 8];
                if space
                    .read_bytes(list + i * 8, &mut raw, self.attrs())
                    .is_err()
                {
                    return Err(ST_DATA_TRANSFER);
                }
                let entry = u64::from_le_bytes(raw);
                if !entry.is_multiple_of(page) {
                    return Err(ST_PRP_OFFSET);
                }
                // §4.3: the last entry of a full list is a pointer to the next
                // list, unless what is left fits in that one page.
                if i + 1 == per_page && remaining > page {
                    list = entry;
                    chained = true;
                    break;
                }
                let take = core::cmp::min(remaining, page);
                out.push((entry, take));
                remaining -= take;
                if remaining == 0 {
                    return Ok(out);
                }
            }
            if !chained {
                // Unreachable by construction — the last slot either chains or
                // carries the last page — but a `loop` that trusted that would
                // spin on a page size this model did not expect.
                return Err(ST_DATA_TRANSFER);
            }
        }
    }

    /// Move `data` out to the guest, through the ranges `PRP1`/`PRP2` name.
    fn scatter(&self, space: &AddressSpace, cmd: &Command, data: &[u8], page: u64) -> u16 {
        let chunks = match self.prp_chunks(space, cmd.prp1, cmd.prp2, data.len() as u64, page) {
            Ok(chunks) => chunks,
            Err(status) => return status,
        };
        let mut at = 0usize;
        for (addr, len) in chunks {
            let n = len as usize;
            if space
                .write_bytes(addr, &data[at..at + n], self.attrs())
                .is_err()
            {
                return ST_DATA_TRANSFER;
            }
            at += n;
        }
        ST_SUCCESS
    }

    /// Move `len` bytes between the medium at `offset` and the guest ranges
    /// `PRP1`/`PRP2` name.
    ///
    /// One PRP entry at a time, with no whole-transfer buffer in between: a
    /// command may name 4 MiB ([`MDTS`]) and the largest thing this allocates is
    /// one page. The chunk list itself is computed first, so a malformed chain
    /// fails before a single byte moves.
    fn transfer(
        &self,
        space: &AddressSpace,
        cmd: &Command,
        offset: u64,
        len: u64,
        write: bool,
    ) -> u16 {
        let chunks = match self.prp_chunks(space, cmd.prp1, cmd.prp2, len, self.page()) {
            Ok(chunks) => chunks,
            Err(status) => return status,
        };
        let mut at = offset;
        for (addr, chunk) in chunks {
            let mut buf = vec![0u8; chunk as usize];
            if write {
                if space.read_bytes(addr, &mut buf, self.attrs()).is_err() {
                    return ST_DATA_TRANSFER;
                }
                if self.ns.media.write_at(at, &buf).is_err() {
                    // §4.6.1's media and data integrity errors are where a
                    // write that did not reach the medium belongs.
                    return ST_WRITE_FAULT;
                }
            } else {
                if self.ns.media.read_at(at, &mut buf).is_err() {
                    return ST_UNRECOVERED_READ;
                }
                if space.write_bytes(addr, &buf, self.attrs()).is_err() {
                    return ST_DATA_TRANSFER;
                }
            }
            at += chunk;
        }
        ST_SUCCESS
    }

    /// The page size `CC.MPS` currently names.
    fn page(&self) -> u64 {
        self.state.lock().page()
    }
}

// ---------------------------------------------------------------------------
// the admin command set (§5)
// ---------------------------------------------------------------------------

impl Controller {
    /// Run one admin command. `None` holds it outstanding.
    fn admin(&self, space: &AddressSpace, cmd: &Command) -> Option<(u16, u32)> {
        Some(match cmd.opcode {
            ADMIN_CREATE_CQ => self.create_cq(cmd),
            ADMIN_CREATE_SQ => self.create_sq(cmd),
            ADMIN_DELETE_CQ => self.delete_cq(cmd),
            ADMIN_DELETE_SQ => self.delete_sq(cmd),
            ADMIN_IDENTIFY => self.identify(space, cmd),
            ADMIN_SET_FEATURES => self.set_features(cmd),
            ADMIN_GET_FEATURES => self.get_features(cmd),
            ADMIN_GET_LOG_PAGE => self.get_log_page(space, cmd),
            // §5.1: bit 0 of the completion's DW0 set means the command was
            // *not* aborted. Nothing here is ever in flight long enough to
            // abort, so that is always the honest answer.
            ADMIN_ABORT => (ST_SUCCESS, 1),
            ADMIN_ASYNC_EVENT => return self.async_event(),
            _ => (ST_INVALID_OPCODE, 0),
        })
    }

    /// Create I/O Completion Queue (§5.3).
    fn create_cq(&self, cmd: &Command) -> (u16, u32) {
        let qid = (cmd.cdw10 & 0xffff) as usize;
        let entries = ((cmd.cdw10 >> 16) & 0xffff) + 1;
        let contiguous = cmd.cdw11 & 1 != 0;
        let interrupts = cmd.cdw11 & 2 != 0;
        let vector = (cmd.cdw11 >> 16) as u16;
        if qid == 0 || qid > usize::from(self.params.io_queues) {
            return (ST_INVALID_QID, 0);
        }
        if !(2..=MAX_QUEUE_ENTRIES).contains(&entries) {
            return (ST_INVALID_QSIZE, 0);
        }
        if !contiguous {
            // `CAP.CQR` is set, so a queue that is not physically contiguous is
            // not something this controller ever offered (§3.1.1).
            return (ST_INVALID_FIELD, 0);
        }
        // This build has pin-based interrupts only, so a vector is a mask bit
        // in `INTMS` and there are 32 of them.
        if vector >= 32 {
            return (ST_INVALID_VECTOR, 0);
        }
        let page = self.page();
        if !cmd.prp1.is_multiple_of(page) || cmd.prp1 == 0 {
            return (ST_PRP_OFFSET, 0);
        }
        let mut state = self.state.lock();
        if state.cq[qid].is_some() {
            return (ST_INVALID_QID, 0);
        }
        state.cq[qid] = Some(CompQueue {
            base: cmd.prp1,
            entries,
            head: 0,
            tail: 0,
            phase: true,
            vector,
            interrupts,
        });
        (ST_SUCCESS, 0)
    }

    /// Create I/O Submission Queue (§5.4).
    fn create_sq(&self, cmd: &Command) -> (u16, u32) {
        let qid = (cmd.cdw10 & 0xffff) as usize;
        let entries = ((cmd.cdw10 >> 16) & 0xffff) + 1;
        let contiguous = cmd.cdw11 & 1 != 0;
        let cqid = (cmd.cdw11 >> 16) as u16;
        if qid == 0 || qid > usize::from(self.params.io_queues) {
            return (ST_INVALID_QID, 0);
        }
        if !(2..=MAX_QUEUE_ENTRIES).contains(&entries) {
            return (ST_INVALID_QSIZE, 0);
        }
        if !contiguous {
            return (ST_INVALID_FIELD, 0);
        }
        let page = self.page();
        if !cmd.prp1.is_multiple_of(page) || cmd.prp1 == 0 {
            return (ST_PRP_OFFSET, 0);
        }
        let mut state = self.state.lock();
        // §5.4: the completion queue named has to exist, and saying so with the
        // specification's own status code is how a driver finds its own bug.
        if usize::from(cqid) >= QUEUE_SLOTS || state.cq[usize::from(cqid)].is_none() {
            return (ST_CQ_INVALID, 0);
        }
        if state.sq[qid].is_some() {
            return (ST_INVALID_QID, 0);
        }
        state.sq[qid] = Some(SubQueue {
            base: cmd.prp1,
            entries,
            head: 0,
            tail: 0,
            cqid,
        });
        (ST_SUCCESS, 0)
    }

    /// Delete I/O Submission Queue (§5.6).
    fn delete_sq(&self, cmd: &Command) -> (u16, u32) {
        let qid = (cmd.cdw10 & 0xffff) as usize;
        if qid == 0 || qid >= QUEUE_SLOTS {
            return (ST_INVALID_QID, 0);
        }
        let mut state = self.state.lock();
        if state.sq[qid].take().is_none() {
            return (ST_INVALID_QID, 0);
        }
        (ST_SUCCESS, 0)
    }

    /// Delete I/O Completion Queue (§5.5).
    fn delete_cq(&self, cmd: &Command) -> (u16, u32) {
        let qid = (cmd.cdw10 & 0xffff) as usize;
        if qid == 0 || qid >= QUEUE_SLOTS {
            return (ST_INVALID_QID, 0);
        }
        let mut state = self.state.lock();
        if state.cq[qid].is_none() {
            return (ST_INVALID_QID, 0);
        }
        // §5.5: deleting a completion queue that still has submission queues
        // associated with it is an error, not a way to strand them.
        if state
            .sq
            .iter()
            .flatten()
            .any(|sq| usize::from(sq.cqid) == qid)
        {
            return (ST_INVALID_QUEUE_DELETION, 0);
        }
        state.cq[qid] = None;
        (ST_SUCCESS, 0)
    }

    /// Identify (§5.15).
    fn identify(&self, space: &AddressSpace, cmd: &Command) -> (u16, u32) {
        if cmd.psdt != 0 {
            return (ST_INVALID_FIELD, 0);
        }
        let data = match cmd.cdw10 & 0xff {
            // CNS 00h: the namespace this command names.
            0x00 => {
                if cmd.nsid != 1 {
                    return (ST_INVALID_NAMESPACE, 0);
                }
                self.identify_namespace()
            }
            // CNS 01h: the controller.
            0x01 => self.identify_controller(),
            // CNS 02h: the active namespace identifiers greater than `NSID`.
            0x02 => {
                let mut list = vec![0u8; IDENTIFY_LEN as usize];
                if cmd.nsid < 1 {
                    list[0..4].copy_from_slice(&1u32.to_le_bytes());
                }
                list
            }
            _ => return (ST_INVALID_FIELD, 0),
        };
        (self.scatter(space, cmd, &data, self.page()), 0)
    }

    /// Set Features (§5.21).
    ///
    /// The Save bit is ignored rather than refused: every feature here is
    /// volatile, `Identify Controller`'s `ONCS` says so, and a driver that sets
    /// it gets the same answer it would from a controller with no non-volatile
    /// feature storage.
    fn set_features(&self, cmd: &Command) -> (u16, u32) {
        match (cmd.cdw10 & 0xff) as u8 {
            // §5.21.1.7: the host asks for a number of queues and the
            // controller answers with the number it *allocated*, which may be
            // fewer. Both fields are zero-based.
            FEATURE_NUM_QUEUES => (ST_SUCCESS, self.allocated_queues()),
            FEATURE_ASYNC_CONFIG => (ST_SUCCESS, 0),
            _ => (ST_INVALID_FIELD, 0),
        }
    }

    /// Get Features, the companion of §5.21.
    fn get_features(&self, cmd: &Command) -> (u16, u32) {
        match (cmd.cdw10 & 0xff) as u8 {
            FEATURE_NUM_QUEUES => (ST_SUCCESS, self.allocated_queues()),
            FEATURE_ASYNC_CONFIG => (ST_SUCCESS, 0),
            _ => (ST_INVALID_FIELD, 0),
        }
    }

    /// The zero-based submission and completion queue counts §5.21.1.7 reports.
    fn allocated_queues(&self) -> u32 {
        let n = u32::from(self.params.io_queues - 1);
        n | (n << 16)
    }

    /// Get Log Page (§5.14).
    ///
    /// The three mandatory pages are here and every one of them reads as
    /// zeroes, which is the truth: nothing in this model records an error, a
    /// temperature or a firmware slot. A log page it does not have is an
    /// Invalid Log Page rather than a page of plausible fiction.
    fn get_log_page(&self, space: &AddressSpace, cmd: &Command) -> (u16, u32) {
        if cmd.psdt != 0 {
            return (ST_INVALID_FIELD, 0);
        }
        let lid = (cmd.cdw10 & 0xff) as u8;
        // NUMD is zero-based and split across two dwords (§5.14).
        let numd = u64::from((cmd.cdw10 >> 16) & 0xffff) | (u64::from(cmd.cdw11 & 0xffff) << 16);
        let len = (numd + 1) * 4;
        if !matches!(lid, 0x01..=0x03) {
            return (ST_INVALID_LOG_PAGE, 0);
        }
        if len > MAX_STRUCTURE {
            return (ST_INVALID_FIELD, 0);
        }
        let data = vec![0u8; len as usize];
        (self.scatter(space, cmd, &data, self.page()), 0)
    }

    /// Asynchronous Event Request (§5.2).
    ///
    /// Held outstanding for ever, because that is what a controller with
    /// nothing to report does: the command occupies one of the `AERL + 1`
    /// slots and completes when an event happens. Nothing in this model
    /// generates one, so nothing completes it — and a driver that submits more
    /// than the limit gets the specification's own status code rather than a
    /// completion it did not ask for.
    fn async_event(&self) -> Option<(u16, u32)> {
        let mut state = self.state.lock();
        if state.aer >= AER_LIMIT {
            return Some((ST_AER_LIMIT, 0));
        }
        state.aer += 1;
        None
    }

    /// The 4096-byte `Identify Controller` structure (§5.15.2.2).
    fn identify_controller(&self) -> Vec<u8> {
        let mut d = vec![0u8; IDENTIFY_LEN as usize];
        d[0..2].copy_from_slice(&self.params.vendor.to_le_bytes());
        d[2..4].copy_from_slice(&self.params.subsystem_vendor.to_le_bytes());
        ascii(&mut d[4..24], &self.params.serial);
        ascii(&mut d[24..64], &self.params.model);
        ascii(&mut d[64..72], &self.params.firmware);
        // MDTS: the largest data transfer, in `2^(12 + CAP.MPSMIN)` units.
        d[77] = MDTS;
        d[80..84].copy_from_slice(&VERSION.to_le_bytes());
        // ACL, the abort command limit, zero-based; AERL, likewise.
        d[258] = 3;
        d[259] = (AER_LIMIT - 1) as u8;
        // SQES and CQES: the minimum and maximum entry sizes, as powers of two
        // in the low and high nibbles. 64 bytes and 16 bytes, both fixed.
        d[512] = 0x66;
        d[513] = 0x44;
        // MAXCMD: how many commands may be outstanding at once. One, because a
        // command completes inside the doorbell write that submitted it.
        d[514..516].copy_from_slice(&1u16.to_le_bytes());
        // NN: one namespace.
        d[516..520].copy_from_slice(&1u32.to_le_bytes());
        // ONCS bit 3: Write Zeroes is supported. Nothing else optional is.
        d[520..522].copy_from_slice(&0x0008u16.to_le_bytes());
        // VWC bit 0: a volatile write cache is present, so `Flush` means
        // something — the medium's own `flush`, which is what makes it true.
        d[525] = 1;
        d
    }

    /// The 4096-byte `Identify Namespace` structure (§5.15.2.1).
    fn identify_namespace(&self) -> Vec<u8> {
        let mut d = vec![0u8; IDENTIFY_LEN as usize];
        let blocks = self.ns.blocks();
        // NSZE, NCAP and NUSE: the size, the capacity and how much is
        // allocated. A flat medium is fully allocated, so all three are equal.
        d[0..8].copy_from_slice(&blocks.to_le_bytes());
        d[8..16].copy_from_slice(&blocks.to_le_bytes());
        d[16..24].copy_from_slice(&blocks.to_le_bytes());
        // NLBAF: one LBA format, zero-based. FLBAS: format 0 is in use.
        d[25] = 0;
        d[26] = 0;
        // NSATTR bit 0: the namespace is write protected. A medium that refuses
        // writes says so here rather than only at the first failed one.
        d[99] = u8::from(self.ns.is_read_only());
        // LBA Format 0: metadata size 0, and the block size as a power of two
        // in bits 23:16.
        let lbaf0 = (self.ns.lba_shift) << 16;
        d[128..132].copy_from_slice(&lbaf0.to_le_bytes());
        d
    }
}

// ---------------------------------------------------------------------------
// the NVM command set (§6)
// ---------------------------------------------------------------------------

impl Controller {
    /// Run one I/O command.
    fn nvm(&self, space: &AddressSpace, cmd: &Command) -> Option<(u16, u32)> {
        // §6: every NVM command names a namespace, and this controller has one.
        // `FFFFFFFFh` means "all of them", which Flush accepts and a data
        // command does not.
        let all = cmd.nsid == 0xffff_ffff;
        if cmd.nsid != 1 && !(all && cmd.opcode == NVM_FLUSH) {
            return Some((ST_INVALID_NAMESPACE, 0));
        }
        Some(match cmd.opcode {
            NVM_FLUSH => match self.ns.media.flush() {
                Ok(()) => (ST_SUCCESS, 0),
                Err(_) => (ST_WRITE_FAULT, 0),
            },
            NVM_READ => self.read(space, cmd),
            NVM_WRITE => self.write(space, cmd),
            NVM_WRITE_ZEROES => self.write_zeroes(cmd),
            _ => (ST_INVALID_OPCODE, 0),
        })
    }

    /// The starting block and the count a Read, Write or Write Zeroes names.
    ///
    /// `SLBA` is 64 bits across CDW10 and CDW11, and `NLB` is the zero-based
    /// block count in CDW12's low half (§6.9, §6.15).
    fn extent(cmd: &Command) -> (u64, u64) {
        let slba = u64::from(cmd.cdw10) | (u64::from(cmd.cdw11) << 32);
        let blocks = u64::from(cmd.cdw12 & 0xffff) + 1;
        (slba, blocks)
    }

    /// Read (§6.9): medium to guest memory.
    fn read(&self, space: &AddressSpace, cmd: &Command) -> (u16, u32) {
        if cmd.psdt != 0 {
            return (ST_INVALID_FIELD, 0);
        }
        let (slba, blocks) = Controller::extent(cmd);
        let Some((offset, len)) = self.ns.range(slba, blocks) else {
            return (ST_LBA_RANGE, 0);
        };
        if len > MAX_TRANSFER {
            // Larger than `MDTS` said, which is an Invalid Field rather than a
            // transfer the controller quietly truncates (§5.15.2.2).
            return (ST_INVALID_FIELD, 0);
        }
        (self.transfer(space, cmd, offset, len, false), 0)
    }

    /// Write (§6.15): guest memory to medium.
    fn write(&self, space: &AddressSpace, cmd: &Command) -> (u16, u32) {
        if cmd.psdt != 0 {
            return (ST_INVALID_FIELD, 0);
        }
        let (slba, blocks) = Controller::extent(cmd);
        let Some((offset, len)) = self.ns.range(slba, blocks) else {
            return (ST_LBA_RANGE, 0);
        };
        if len > MAX_TRANSFER {
            return (ST_INVALID_FIELD, 0);
        }
        if self.ns.is_read_only() {
            return (ST_WRITE_FAULT, 0);
        }
        (self.transfer(space, cmd, offset, len, true), 0)
    }

    /// Write Zeroes (§6.16): no data transfer at all, which is the point of it.
    fn write_zeroes(&self, cmd: &Command) -> (u16, u32) {
        let (slba, blocks) = Controller::extent(cmd);
        let Some((offset, len)) = self.ns.range(slba, blocks) else {
            return (ST_LBA_RANGE, 0);
        };
        if self.ns.is_read_only() {
            return (ST_WRITE_FAULT, 0);
        }
        // In bounded pieces, because `NLB` may name far more than `MDTS` — the
        // limit is on *data transfer*, and this command transfers none.
        let zeroes = vec![0u8; ZERO_CHUNK as usize];
        let mut at = 0u64;
        while at < len {
            let n = core::cmp::min(ZERO_CHUNK, len - at);
            if self
                .ns
                .media
                .write_at(offset + at, &zeroes[..n as usize])
                .is_err()
            {
                return (ST_WRITE_FAULT, 0);
            }
            at += n;
        }
        (ST_SUCCESS, 0)
    }
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

impl Controller {
    /// Serialize the register file and the queue descriptors.
    ///
    /// Nothing else needs saving: the queues themselves, the commands in them
    /// and the data are all in guest memory, and a command never spans a
    /// snapshot because it completes inside the doorbell write that submitted
    /// it (see the module documentation).
    ///
    /// `master` and `intx_disabled` are **not** here. They are the PCI Command
    /// register, which the function saves, and re-deriving them on load is what
    /// keeps derived state out of a snapshot (`CLAUDE.md`).
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let state = *self.state.lock();
        w.write_u32(state.cc)?;
        w.write_u32(state.csts)?;
        w.write_u32(state.intms)?;
        w.write_u32(state.aqa)?;
        w.write_u64(state.asq)?;
        w.write_u64(state.acq)?;
        w.write_u32(state.aer)?;
        for slot in &state.sq {
            match slot {
                Some(sq) => {
                    w.write_bool(true)?;
                    w.write_u64(sq.base)?;
                    w.write_u32(sq.entries)?;
                    w.write_u32(sq.head)?;
                    w.write_u32(sq.tail)?;
                    w.write_u16(sq.cqid)?;
                }
                None => w.write_bool(false)?,
            }
        }
        for slot in &state.cq {
            match slot {
                Some(cq) => {
                    w.write_bool(true)?;
                    w.write_u64(cq.base)?;
                    w.write_u32(cq.entries)?;
                    w.write_u32(cq.head)?;
                    w.write_u32(cq.tail)?;
                    w.write_u16(cq.vector)?;
                    w.write_bool(cq.phase)?;
                    w.write_bool(cq.interrupts)?;
                }
                None => w.write_bool(false)?,
            }
        }
        Ok(())
    }

    /// Restore what [`Controller::save`] wrote.
    ///
    /// Every field is checked rather than trusted. A queue with zero entries
    /// would divide by zero the first time a doorbell moved its head, and a
    /// snapshot is an untrusted parser surface like any other file
    /// (`CLAUDE.md`, testing) — `fuzz/fuzz_targets/nvme_mmio.rs` feeds this one
    /// arbitrary bytes on purpose.
    ///
    /// # Errors
    ///
    /// [`Error::State`](crate::Error::State) for a chunk that describes a
    /// controller this model could not have been in.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let mut state = State::new();
        state.cc = r.read_u32()? & CC_MASK;
        state.csts = r.read_u32()? & 0x3f;
        state.intms = r.read_u32()?;
        state.aqa = r.read_u32()? & 0x0fff_0fff;
        state.asq = r.read_u64()?;
        state.acq = r.read_u64()?;
        state.aer = r.read_u32()?.min(AER_LIMIT);
        let bad = |what: &str| crate::core::error::Error::State(alloc::format!("nvme: {what}"));
        for slot in &mut state.sq {
            if !r.read_bool()? {
                continue;
            }
            let sq = SubQueue {
                base: r.read_u64()?,
                entries: r.read_u32()?,
                head: r.read_u32()?,
                tail: r.read_u32()?,
                cqid: r.read_u16()?,
            };
            if !(2..=MAX_ADMIN_ENTRIES).contains(&sq.entries)
                || sq.head >= sq.entries
                || sq.tail >= sq.entries
                || usize::from(sq.cqid) >= QUEUE_SLOTS
            {
                return Err(bad("a submission queue no controller could have built"));
            }
            *slot = Some(sq);
        }
        for slot in &mut state.cq {
            if !r.read_bool()? {
                continue;
            }
            let cq = CompQueue {
                base: r.read_u64()?,
                entries: r.read_u32()?,
                head: r.read_u32()?,
                tail: r.read_u32()?,
                vector: r.read_u16()?,
                phase: r.read_bool()?,
                interrupts: r.read_bool()?,
            };
            if !(2..=MAX_ADMIN_ENTRIES).contains(&cq.entries)
                || cq.head >= cq.entries
                || cq.tail >= cq.entries
                || cq.vector >= 32
            {
                return Err(bad("a completion queue no controller could have built"));
            }
            *slot = Some(cq);
        }
        // A submission queue whose completion queue went missing would post
        // into nothing.
        for sq in state.sq.iter().flatten() {
            if state.cq[usize::from(sq.cqid)].is_none() {
                return Err(bad(
                    "a submission queue names a completion queue that is not there",
                ));
            }
        }
        *self.state.lock() = state;
        self.refresh_irq();
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// the register block, as something an address space dispatches to
// ---------------------------------------------------------------------------

impl MemOps for Controller {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // No `debug` branch, and that is a claim rather than an omission: §3.1
        // has no read-to-clear register, the doorbells are write-only, and
        // nothing here advances a queue or acknowledges an interrupt. Every
        // side effect this device has is on the write side, and those are
        // refused outright below.
        if offset.saturating_add(dst.len() as u64) > REGISTER_LEN {
            return Err(BusError::BadAccess);
        }
        match dst.len() {
            4 => {
                if !offset.is_multiple_of(4) {
                    return Err(BusError::BadAccess);
                }
                dst.copy_from_slice(&self.read_dword(offset).to_le_bytes());
            }
            8 => {
                if !offset.is_multiple_of(8) {
                    return Err(BusError::BadAccess);
                }
                let value = u64::from(self.read_dword(offset))
                    | (u64::from(self.read_dword(offset + 4)) << 32);
                dst.copy_from_slice(&value.to_le_bytes());
            }
            // §3.1: the controller registers are 32- or 64-bit quantities. A
            // byte access is not a register access, and answering one would be
            // inventing behaviour.
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // There is no harmless write here. `CC.EN` starts and stops the
            // controller, a doorbell submits commands and moves a queue head,
            // and `INTMS` masks an interrupt the guest is waiting for. A
            // debugger that could do any of those would be a debugger that
            // changed the guest (`CLAUDE.md`, devices).
            return Err(BusError::BadAccess);
        }
        if offset.saturating_add(src.len() as u64) > REGISTER_LEN {
            return Err(BusError::BadAccess);
        }
        match src.len() {
            4 => {
                if !offset.is_multiple_of(4) {
                    return Err(BusError::BadAccess);
                }
                self.write_dword(offset, le32(src));
            }
            8 => {
                if !offset.is_multiple_of(8) {
                    return Err(BusError::BadAccess);
                }
                // §3.1: a 64-bit register may be written as one 64-bit access or
                // as two 32-bit ones, low half first. Both land here as the
                // same two writes, which is what keeps `ASQ` from being half
                // updated by one path and whole by the other.
                let value = le64(src);
                self.write_dword(offset, value as u32);
                self.write_dword(offset + 4, (value >> 32) as u32);
            }
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U32, Width::U64)
            .with_natural_alignment(true)
            .with_endian(Endian::Little)
    }
}
