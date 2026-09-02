//! The host bus adapter: the register file, the ports, the command list, the
//! received-FIS area and the PRDT walker.
//!
//! Split from [`super`] the way [`Controller`](crate::dev::nvme::Controller) is
//! split from its PCI function: everything here is AHCI and Serial ATA, and
//! nothing here is PCI. The transport contributes four things and all four
//! arrive through setters — the address space this adapter masters, its
//! `INTx#` output, `COMMAND[2]` (Bus Master Enable) and `COMMAND[10]`
//! (Interrupt Disable).
//!
//! # What an AHCI HBA actually is
//!
//! A **data movement engine**. Software builds, in its own memory, a command
//! list of up to 32 headers; each header points at a command table holding the
//! Register - Host to Device FIS that *is* the ATA command, followed by a
//! Physical Region Descriptor Table — the scatter/gather list. Software writes
//! one bit into `PxCI`, and the HBA fetches all of that itself, hands the
//! taskfile to the drive, moves the data between the drive and the addresses
//! the PRDT names, writes the drive's answering FIS into the received-FIS area,
//! and raises `INTx#`.
//!
//! ```text
//!   guest memory                                the HBA
//!   ------------                                -------
//!   command list ──────── PxCI ───────────────►  fetch header (32 bytes)
//!     CFL, W, PRDTL, CTBA                          │
//!   command table  ◄──────────────────────────── fetch
//!     CFIS (H2D Register FIS)  ─────────────────►  AtaDisk::taskfile_start
//!     PRDT[0..PRDTL]  ◄──────────────────────────  walk
//!   data pages ◄────────── read ── write ──────►  the medium
//!   received FIS area ◄──── D2H / PIO Setup ────  post
//!                     ◄──── INTA# ──────────────  the wire
//!   command header DW1 ◄─── PRDBC ──────────────  bytes moved
//! ```
//!
//! Everything except the register block is in **guest memory**, and this
//! adapter reads and writes it as a bus master with its own [`RequesterId`].
//!
//! # There is one command set, and it is not here
//!
//! Not a single ATA opcode appears in this file. A command reaches the drive as
//! a [`Taskfile`] — six named fields — and comes back as
//! [`Registers`]; the decode, the addressing, the media access and the
//! busy/DRQ handshake are all [`crate::dev::ata::disk`]'s, shared byte for byte
//! with the AT's IDE channel. What this file contributes is the *transport*: a
//! FIS is a byte layout, a PRD is a byte layout, and byte layouts belong to the
//! adapter that receives them, exactly as a port number belongs to the adapter
//! that decodes it.
//!
//! # Locks, and the order they go in
//!
//! One state lock at [`AHCI_RANK`], which sits *below* [`LockRank::DEVICE`] —
//! so the PCI function's configuration lock may be taken and released before
//! this one, never the other way round — and *above* [`LockRank::WIRE`], so the
//! interrupt output is driven after it is released.
//!
//! ```text
//!   CPU session                        (BUS 0x4000)
//!     → the PCI function's config      (DEVICE 0x5000)  — configuration only
//!     → the drive bay                  (0x4c40)         — looked up, released
//!       → the drive's state            (DEVICE 0x5000)
//!     → the HBA's own state            (0x5a10, here)
//!       → the interrupt output         (WIRE 0x6000, after 0x5a10 is released)
//! ```
//!
//! Note that the drive's lock is at `DEVICE`, which is *above* this one: the
//! HBA's state lock is therefore **never** held across a call into the drive,
//! and it is never held across a guest-memory access or a wire change either.
//! It cannot be [`LockRank::BUS`] for the reason `core::space` states — *"a CPU
//! holds a `BUS`-ranked lock across the accesses it issues"* — and every access
//! to this register block arrives from inside one.
//!
//! # Re-entrancy: a doorbell reached from inside a doorbell
//!
//! The PRDT is guest memory and guest memory is an address space that also
//! contains this adapter's own ABAR. A driver can therefore aim a data block at
//! `PxCI`, and the write handler is re-entered from inside itself. The answer
//! is the one [`Wire`](crate::core::wire::Wire) and
//! [`nvme`](crate::dev::nvme) both give: **the work is iterative, not
//! recursive**. A re-entrant write to `PxCI` records the bits and returns; the
//! outermost [`Hba::run`] re-reads every port's `PxCI` after each command and
//! picks the new work up. Recursion depth is one, whatever the guest builds.
//!
//! # Every walk is bounded
//!
//! Three of them, and each bound is argued where it is applied:
//!
//! * the **PRDT** is up to 65,535 entries by specification, and this model
//!   additionally stops a single command at `MAX_TRANSFER` bytes;
//! * a **data phase** is capped at `MAX_BLOCKS` blocks, so a drive that
//!   somehow never finished could not spin the engine;
//! * one entry into [`Hba::run`] executes at most `MAX_COMMANDS_PER_RUN`
//!   commands — a bound a legitimate driver cannot reach, because reaching it
//!   means the data *was* the doorbell.
//!
//! `fuzz/fuzz_targets/ahci_mmio.rs` drives arbitrary bytes through all three.
//!
//! # Time
//!
//! A command completes inside the `PxCI` write that issued it, so this is an
//! adapter with zero service time — the same choice
//! [`nvme`](crate::dev::nvme) makes and for the same two reasons: no host I/O
//! latency reaches the guest's timeline, and there is no in-flight state to
//! snapshot, because everything that outlives a command is in guest memory.
//!
//! # What is deliberately absent
//!
//! * **Native Command Queuing.** `CAP.SNCQ` is zero and `PxSACT` is a register
//!   a driver is told not to use. NCQ needs a device that reorders, and this
//!   drive has no queue at all.
//! * **Port multipliers**, **FIS-based switching**, **command completion
//!   coalescing**, **enclosure management**, **staggered spin-up**, **cold
//!   presence detect** and **aggressive link power management**. Each is
//!   reported unsupported in `CAP`/`CAP2` rather than half-present, which is
//!   how a driver is supposed to find out.
//! * **MSI and MSI-X**, because `src/bus/pci` has no capability list yet. The
//!   adapter is pin-based, which AHCI permits.
//! * **ATAPI.** The `A` bit in a command header is accepted and ignored: the
//!   drive behind the port is not a packet device and aborts the command, which
//!   is exactly how a driver finds out.
//!
//! # Sources
//!
//! * **Serial ATA Advanced Host Controller Interface (AHCI), Revision 1.3.1**
//!   (Intel, 2011) — §2.1 the PCI header and `ABAR` at offset `24h`, §3.1 the
//!   generic host control registers, §3.3 the port registers and their offsets,
//!   §4.2.1 the Received FIS structure, §4.2.2 the command list and its command
//!   header, §4.2.3 the command table and the Physical Region Descriptor Table,
//!   §5.4.1 when `PRDBC` is updated, §5.5 the software rules, §6.1 the error
//!   types and §10.4 reset. Intel's own PDF answers `403` to anything that is
//!   not a browser; the copy read was the Internet Archive's of
//!   `intel.com/content/dam/www/public/us/en/documents/technical-specifications/serial-ata-ahci-spec-rev1-3-1.pdf`.
//! * **Serial ATA: High Speed Serialized AT Attachment, Revision 1.0** —
//!   §8.5.2 the Register - Host to Device FIS layout, §8.5.3 the
//!   Register - Device to Host FIS, §8.5.8 the PIO Setup FIS. AHCI §4.2.3.1
//!   defers to it for the FIS formats and does not repeat them.
//! * T13's ATA/ATAPI-6 for the command block itself, through
//!   [`crate::dev::ata`].
//!
//! **No emulator source was consulted and no operating system's AHCI or libata
//! driver was opened** (`CLAUDE.md`, provenance).

use alloc::string::String;
use alloc::sync::{Arc, Weak};
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
use crate::dev::ata::bays::Bay;
use crate::dev::ata::disk::{CTL_SRST, ST_BSY, ST_DRQ, ST_ERR};
use crate::dev::ata::{AtaDisk, Phase, Registers, Taskfile};

// ---------------------------------------------------------------------------
// shape
// ---------------------------------------------------------------------------

/// Where this adapter's state lock sits in the ranked order.
///
/// Beside [`crate::dev::nvme::NVME_RANK`] and for the same reasons: below
/// [`LockRank::DEVICE`] so a PCI configuration cycle's lock is taken and
/// released before it, and above [`LockRank::WIRE`] so the interrupt output is
/// driven after it is released. A distinct number from NVMe's because a board
/// may carry both, and a deterministic order is better than a coin toss.
pub const AHCI_RANK: LockRank = LockRank::new(0x5a10);

/// How many ports this model implements at most.
///
/// Not a specification limit — AHCI's is 32 — but the size of the port array
/// this model allocates and the reason [`REGISTER_LEN`] is what it is. A
/// machine file may ask for fewer.
pub const MAX_PORTS: usize = 8;

/// How much memory the register block decodes: 2 KiB.
///
/// §3: the generic host control registers occupy `00h`-`2Bh` and port *n*'s
/// registers start at `100h + n * 80h`, so eight ports reach `500h`. A power of
/// two, because a base address register window is one.
pub const REGISTER_LEN: u64 = 0x800;

/// Where port 0's registers start (§3.3).
const PORT_BASE: u64 = 0x100;

/// Bytes between one port's registers and the next (§3.3).
const PORT_STRIDE: u64 = 0x80;

/// How many command slots a port has (§3.1.1, `CAP.NCS`).
const COMMAND_SLOTS: u32 = 32;

/// Bytes in one command header (§4.2.2).
const HEADER_LEN: u64 = 32;

/// Bytes in the whole command list: 32 headers (§4.2.2).
const COMMAND_LIST_LEN: u64 = HEADER_LEN * COMMAND_SLOTS as u64;

/// Bytes in the received FIS structure (§4.2.1).
const RECEIVED_FIS_LEN: u64 = 256;

/// Where the PRDT starts inside a command table (§4.2.3): after the 64-byte
/// command FIS, the 16-byte ATAPI command and 48 reserved bytes.
const PRDT_OFFSET: u64 = 0x80;

/// Bytes in one Physical Region Descriptor (§4.2.3.3): four dwords.
const PRD_LEN: u64 = 16;

/// The largest `PRDTL` §4.2.2 allows.
const MAX_PRDT: u32 = 65535;

/// The most one command will move.
///
/// 32 MiB, which is what a 48-bit `READ DMA EXT` of 65,536 sectors asks for —
/// the largest transfer the command set can name, so a legitimate driver never
/// meets this bound and a guest that built a ring of PRDs does.
const MAX_TRANSFER: u64 = 65536 * 512;

/// The most data blocks one command's data phase will run.
///
/// One more than the sectors a 48-bit command can name. The drive cannot
/// produce more; a bound here is what makes that a fact of this loop rather
/// than a fact of the drive.
const MAX_BLOCKS: u64 = 65537;

/// How many commands one entry into [`Hba::run`] executes.
///
/// Every port's whole command list twice over. A driver cannot reach it —
/// reaching it means a data transfer was writing `PxCI`, which is the
/// re-entrancy case this bound exists for.
const MAX_COMMANDS_PER_RUN: u32 = MAX_PORTS as u32 * COMMAND_SLOTS * 2;

/// Bytes moved between the drive and guest memory at a time.
///
/// One sector. The drive hands out at most one `DRQ` block per call and a PRD
/// may be any length, so this is only the size of the staging buffer that sits
/// between them — small, because it is a host allocation on a data path.
const CHUNK: usize = 512;

// ---------------------------------------------------------------------------
// the registers (§3.1, §3.3)
// ---------------------------------------------------------------------------

/// `VS`, the AHCI revision this adapter implements: 1.3.1 (§3.1.5.6).
const VERSION: u32 = 0x0001_0301;

const REG_CAP: u64 = 0x00;
const REG_GHC: u64 = 0x04;
const REG_IS: u64 = 0x08;
const REG_PI: u64 = 0x0c;
const REG_VS: u64 = 0x10;
const REG_CAP2: u64 = 0x24;

/// `GHC.AE`, AHCI Enable. Read-only one here, because `CAP.SAM` is one: this
/// adapter has no legacy task-file interface to switch to (§3.1.2).
const GHC_AE: u32 = 1 << 31;
/// `GHC.IE`, the global interrupt enable (§3.1.2).
const GHC_IE: u32 = 1 << 1;
/// `GHC.HR`, HBA Reset (§3.1.2). Write-1, and hardware clears it.
const GHC_HR: u32 = 1 << 0;

const PORT_CLB: u64 = 0x00;
const PORT_CLBU: u64 = 0x04;
const PORT_FB: u64 = 0x08;
const PORT_FBU: u64 = 0x0c;
const PORT_IS: u64 = 0x10;
const PORT_IE: u64 = 0x14;
const PORT_CMD: u64 = 0x18;
const PORT_TFD: u64 = 0x20;
const PORT_SIG: u64 = 0x24;
const PORT_SSTS: u64 = 0x28;
const PORT_SCTL: u64 = 0x2c;
const PORT_SERR: u64 = 0x30;
const PORT_SACT: u64 = 0x34;
const PORT_CI: u64 = 0x38;

/// `PxIS.DHRS`: a D2H Register FIS arrived with its `I` bit set (§3.3.5).
const IS_DHRS: u32 = 1 << 0;
/// `PxIS.PSS`: a PIO Setup FIS arrived with its `I` bit set and its data moved.
const IS_PSS: u32 = 1 << 1;
/// `PxIS.DPS`: a PRD with its `I` bit set finished (§5.4.2).
const IS_DPS: u32 = 1 << 5;
/// `PxIS.PCS`: the connect status changed. Reflects `PxSERR.DIAG.X`, read-only.
const IS_PCS: u32 = 1 << 6;
/// `PxIS.OFS`: more bytes arrived than the PRD table had room for (§6.1.5).
const IS_OFS: u32 = 1 << 24;
/// `PxIS.IFS`: a fatal error on the interface stopped the transfer (§6.1.2).
const IS_IFS: u32 = 1 << 27;
/// `PxIS.HBFS`: a host bus error the adapter cannot recover from (§6.1.1).
const IS_HBFS: u32 = 1 << 29;
/// `PxIS.TFES`: the device's status came back with its error bit set (§6.1.4).
const IS_TFES: u32 = 1 << 30;

/// The four status bits §6.2.2 calls fatal: the ones that put a port in
/// `ERR:Fatal` and clear `PxCMD.CR`. `PxIS.HBDS` is not among the ones this
/// model can raise — there is no parity on a host bus that does not exist.
const FATAL: u32 = IS_HBFS | IS_IFS | IS_TFES;

/// Which `PxIS` bits software clears by writing one (§3.3.5).
///
/// `PCS` and `PRCS` are not among them: they reflect `PxSERR.DIAG.X` and
/// `PxSERR.DIAG.N` and are cleared by clearing those, which is the trap a
/// driver falls into if a model gets it wrong.
const IS_WRITE_CLEAR: u32 = 0xfd80_00af;

/// Which `PxIE` bits this adapter implements — the mirror of the `PxIS` bits it
/// can set, plus the read-only reflections a driver may still enable.
const IE_IMPLEMENTED: u32 = 0xfdc0_00ff;

/// `PxCMD.ST`, Start: the adapter may process the command list (§3.3.7).
const CMD_ST: u32 = 1 << 0;
/// `PxCMD.SUD`, Spin-Up Device. Read-only one: staggered spin-up is not
/// supported, and §3.3.7 makes the bit read-only one in that case.
const CMD_SUD: u32 = 1 << 1;
/// `PxCMD.POD`, Power On Device. Read-only one, as §3.3.7 requires when cold
/// presence detect is not supported.
const CMD_POD: u32 = 1 << 2;
/// `PxCMD.CLO`, Command List Override: clear `BSY` and `DRQ` so that a software
/// reset can be sent to a device that is stuck (§3.3.7).
const CMD_CLO: u32 = 1 << 3;
/// `PxCMD.FRE`, FIS Receive Enable (§3.3.7).
const CMD_FRE: u32 = 1 << 4;
/// `PxCMD.FR`, FIS Receive Running (§3.3.7). Read-only to software; the
/// adapter raises it when `FRE` is set and lowers it when `FRE` is cleared.
const CMD_FR: u32 = 1 << 14;
/// `PxCMD.CR`, Command List Running (§3.3.7). Read-only to software, and **not**
/// simply a copy of `ST`: §6.2.2 has a fatal error clear `CR` while leaving `ST`
/// alone, and software restarts the port by writing `ST` one-to-zero — which is
/// what resets `PxCI` — and then back to one. A model that made `CR` follow `ST`
/// would leave a driver's recovery sequence with nothing to wait on.
const CMD_CR: u32 = 1 << 15;
/// `PxCMD.ATAPI` and `PxCMD.DLAE`: which commands drive the activity LED. There
/// is no LED, so they are storage and nothing else (§3.3.7).
const CMD_WRITABLE: u32 = CMD_ST | CMD_CLO | CMD_FRE | (1 << 24) | (1 << 25);
/// `PxCMD.CCS`, the slot the adapter is issuing from (§3.3.7).
const CMD_CCS_SHIFT: u32 = 8;
const CMD_CCS_MASK: u32 = 0x1f << CMD_CCS_SHIFT;

/// `PxSERR.DIAG.X`, Exchanged: device presence changed (§3.3.12).
const SERR_DIAG_X: u32 = 1 << 26;

/// `PxSSTS`/`PxSCTL`'s `DET` field (§3.3.10, §3.3.11).
const DET_MASK: u32 = 0xf;
/// `DET` 1h: perform the interface initialisation sequence — `COMRESET`.
const DET_INIT: u32 = 1;
/// `DET` 4h: the interface is disabled and the Phy is offline.
const DET_OFFLINE: u32 = 4;

/// `PxSSTS` for a port with a device on it: `DET` 3h (presence and
/// communication), `SPD` 3h (Gen 3) and `IPM` 1h (active).
const SSTS_READY: u32 = 0x0000_0133;

/// `PxTFD` out of reset (§3.3.8): `STS` `7Fh`, which is what a port with
/// nothing on it keeps reporting.
const TFD_RESET: u32 = 0x0000_007f;

/// `PxSIG` before any device has answered (§3.3.9).
const SIG_UNKNOWN: u32 = 0xffff_ffff;

// ---------------------------------------------------------------------------
// FIS layouts (Serial ATA 1.0 §8.5)
// ---------------------------------------------------------------------------

/// Register - Host to Device (§8.5.2). Five dwords.
const FIS_H2D: u8 = 0x27;
/// Register - Device to Host (§8.5.3). Five dwords.
const FIS_D2H: u8 = 0x34;
/// PIO Setup - Device to Host (§8.5.8). Five dwords.
const FIS_PIO_SETUP: u8 = 0x5f;

/// Bytes in a Register FIS, in either direction.
const REGISTER_FIS_LEN: usize = 20;

/// Where the PIO Setup FIS is posted in the received FIS structure (§4.2.1).
const PSFIS_AT: u64 = 0x20;
/// Where the D2H Register FIS is posted (§4.2.1).
const RFIS_AT: u64 = 0x40;

/// The `C` bit of a Register - Host to Device FIS: set when the transfer was
/// caused by a write to the Command register rather than to Device Control.
const H2D_C: u8 = 1 << 7;

/// The `I` bit of a Register - Device to Host or PIO Setup FIS: the device's
/// interrupt line.
const D2H_I: u8 = 1 << 6;

/// The `D` bit of a PIO Setup FIS: set when the device is writing host memory.
const PIO_D: u8 = 1 << 5;

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// One port's register file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PortState {
    /// `PxCLB`/`PxCLBU`, already 1 KiB aligned (§3.3.1).
    clb: u64,
    /// `PxFB`/`PxFBU`, already 256-byte aligned (§3.3.3).
    fb: u64,
    is: u32,
    ie: u32,
    cmd: u32,
    tfd: u32,
    sctl: u32,
    serr: u32,
    sact: u32,
    ci: u32,
    /// `PxSIG`, latched from the first D2H Register FIS after a reset (§3.3.9).
    ///
    /// Latched rather than read live, and that is the whole content of the
    /// register: the same four command block registers hold the address a
    /// command left behind, so a model that answered `PxSIG` out of the drive's
    /// *current* taskfile would have it change under a driver every time a read
    /// finished.
    sig: u32,
    /// The Device Control register the port last sent the device.
    ///
    /// Genuine adapter state rather than a copy of the drive's: Serial ATA
    /// §8.5.2.2 has the host adapter transmit a Register - Host to Device FIS
    /// *on a change* of the Control register, so an adapter has to remember
    /// what it last sent. Here it is what says whether a control FIS released
    /// `SRST` or asserted it, which decides whether the device answers.
    ctl: u8,
    /// Command slots that were issued while the device could not answer.
    ///
    /// A software reset's first FIS asserts `SRST` and the device is
    /// deliberately silent afterwards (§10.4.1), so nothing clears `PxCI` for
    /// that slot unless the command header's `C` bit asked the adapter to.
    /// A slot the adapter has run and cannot complete is parked rather than
    /// re-run, which is what keeps [`Hba::run`] making progress.
    parked: u32,
}

impl PortState {
    const fn new() -> PortState {
        PortState {
            clb: 0,
            fb: 0,
            is: 0,
            ie: 0,
            cmd: CMD_SUD | CMD_POD,
            tfd: TFD_RESET,
            sctl: 0,
            serr: 0,
            sact: 0,
            ci: 0,
            sig: SIG_UNKNOWN,
            ctl: 0,
            parked: 0,
        }
    }

    /// Whether the command list engine is running — `CR`, not `ST`.
    const fn running(&self) -> bool {
        self.cmd & CMD_CR != 0
    }

    /// Whether received FISes may be posted to memory (§3.3.7).
    const fn receiving(&self) -> bool {
        self.cmd & CMD_FR != 0
    }
}

/// The whole register file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    ghc: u32,
    is: u32,
    ports: [PortState; MAX_PORTS],
}

impl State {
    const fn new() -> State {
        State {
            // `CAP.SAM` is one, so `GHC.AE` is read-only one (§3.1.2).
            ghc: GHC_AE,
            is: 0,
            ports: [PortState::new(); MAX_PORTS],
        }
    }
}

/// What the transport hands the adapter, at [`LockRank::WIRE`].
struct Link {
    space: Option<Weak<AddressSpace>>,
    irq: Option<WireSource>,
}

impl fmt::Debug for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Link")
            .field("space", &self.space.as_ref().map(|s| s.strong_count()))
            .field("irq", &self.irq.is_some())
            .finish()
    }
}

/// One implemented port: a drive bay and the name it was given.
#[derive(Debug)]
struct Port {
    bay: Arc<Bay>,
    name: String,
}

/// A Serial ATA host bus adapter.
pub struct Hba {
    ports: Vec<Port>,
    /// [`AHCI_RANK`]. Never held across a guest-memory access, a call into a
    /// drive, or a wire change.
    state: Mutex<State>,
    /// The transport's contributions, at [`LockRank::WIRE`]: cloned out and the
    /// guard dropped before any of them is used.
    link: Mutex<Link>,
    /// The identity this adapter's own accesses carry, so a bus fault names the
    /// master rather than the CPU that wrote `PxCI`.
    requester: AtomicU32,
    /// The level the interrupt output is being held at, so the PCI Status
    /// register's Interrupt Status bit is free to read.
    irq_level: AtomicU32,
    /// `COMMAND[2]`, Bus Master Enable.
    master: AtomicBool,
    /// `COMMAND[10]`, Interrupt Disable.
    intx_disabled: AtomicBool,
    /// Whether [`Hba::run`] is already on the stack.
    busy: AtomicBool,
}

impl fmt::Debug for Hba {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Hba");
        s.field("ports", &self.ports.len());
        match self.state.try_lock() {
            Some(state) => s.field("ghc", &state.ghc).field("is", &state.is),
            None => s.field("state", &"<in use>"),
        };
        s.finish_non_exhaustive()
    }
}

/// What a register write asks for once the state lock is released.
///
/// Every one of these is outward — a wire change, a walk of guest memory, or a
/// call into a drive — and none may happen under the lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum After {
    /// Refresh the interrupt output and nothing else.
    Irq,
    /// Commands may be waiting.
    Run,
    /// `GHC.HR`: reset the whole adapter.
    HbaReset,
    /// `PxSCTL.DET` moved: run the interface initialisation sequence.
    Comreset(usize),
}

// ---------------------------------------------------------------------------
// construction and the transport's four contributions
// ---------------------------------------------------------------------------

impl Hba {
    /// An adapter with one port per named bay.
    ///
    /// Allocation only: the bays were opened by the caller, which is
    /// `core::hosts`' rendezvous and therefore allocation too.
    #[must_use]
    pub fn new(bays: Vec<(String, Arc<Bay>)>) -> Hba {
        let ports = bays
            .into_iter()
            .take(MAX_PORTS)
            .map(|(name, bay)| Port { bay, name })
            .collect();
        Hba {
            ports,
            state: Mutex::with_rank(AHCI_RANK, State::new()),
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

    /// How many ports it implements.
    #[must_use]
    pub fn ports(&self) -> usize {
        self.ports.len()
    }

    /// The bay name port `index` was given.
    #[must_use]
    pub fn bay_name(&self, index: usize) -> Option<&str> {
        self.ports.get(index).map(|p| p.name.as_str())
    }

    /// Give the adapter the address space its command lists live in, and the
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

    /// Whether the adapter has an interrupt condition, whatever `COMMAND[10]`
    /// says about emitting it.
    ///
    /// What the PCI Status register's Interrupt Status bit reports; reading it
    /// disturbs nothing, which is why it is an atomic rather than a walk.
    #[must_use]
    pub fn interrupt_pending(&self) -> bool {
        self.irq_level.load(Ordering::Relaxed) != 0
    }

    /// The level the `INTx#` output is being driven to.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.interrupt_pending() && !self.intx_disabled.load(Ordering::Relaxed))
    }

    /// The space this adapter masters, if it still exists.
    fn space(&self) -> Option<Arc<AddressSpace>> {
        self.link.lock().space.as_ref().and_then(Weak::upgrade)
    }

    /// The attributes this adapter's own accesses carry.
    fn attrs(&self) -> MemAttrs {
        MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)))
    }

    /// The drive on port `index`, with the bay lock released before anything
    /// outward happens.
    fn drive(&self, index: usize) -> Option<Arc<AtaDisk>> {
        self.ports.get(index).and_then(|p| p.bay.drive())
    }

    /// Whether port `index` has a device on it.
    fn occupied(&self, index: usize) -> bool {
        self.ports.get(index).is_some_and(|p| p.bay.is_occupied())
    }
}

// ---------------------------------------------------------------------------
// interrupts and reset
// ---------------------------------------------------------------------------

impl Hba {
    /// Re-derive the `INTx#` level and drive it.
    ///
    /// §3.1.3: `IS.IPS[p]` says port *p* has an interrupt pending, and the pin
    /// follows it while `GHC.IE` is set. `IS` is write-1-to-clear, and a port
    /// whose `PxIS & PxIE` is still non-zero sets its bit again — which is why
    /// §5.5.3 has software clear `PxIS` **before** `IS` and why a driver that
    /// does it the other way round never sees the line drop. Modelling that
    /// order is the point.
    pub fn refresh_irq(&self) {
        let pending = {
            let mut state = self.state.lock();
            let mut is = state.is;
            for (i, port) in state.ports.iter().enumerate() {
                if port.is & port.ie != 0 {
                    is |= 1 << i;
                }
            }
            state.is = is;
            state.ghc & GHC_IE != 0 && is != 0
        };
        self.irq_level.store(u32::from(pending), Ordering::Relaxed);
        let level = self.irq_level();
        let out = self.link.lock().irq.clone();
        if let Some(out) = out {
            out.set(level);
        }
    }

    /// Back to the power-on state: `PCIRST#` or the machine's reset.
    ///
    /// Bus mastering and the interrupt disable go with it, because they are the
    /// PCI Command register and `PCIRST#` clears that too — the function's own
    /// `reset` puts a fresh configuration space back at the same moment, so the
    /// two stay in step.
    pub fn reset(&self) {
        self.reset_registers();
        self.master.store(false, Ordering::Relaxed);
        self.intx_disabled.store(false, Ordering::Relaxed);
        self.refresh_irq();
    }

    /// `GHC.HR` (§3.1.2, §10.4.3): an **internal** reset of the adapter.
    ///
    /// The state machines and the memory-mapped register file go back to their
    /// reset values; the PCI configuration space does not, because an HBA reset
    /// is not `PCIRST#`. A model that cleared Bus Master Enable here would leave
    /// the adapter unable to fetch anything after the driver's own reset
    /// sequence — which is the first thing a driver does.
    ///
    /// `busy` is deliberately not touched either. It says whether [`Hba::run`]
    /// is on the stack, and this can be reached *from* `run` by way of a data
    /// block landing on `GHC`; clearing it there would let the next re-entrant
    /// write recurse.
    fn hba_reset(&self) {
        self.reset_registers();
        self.refresh_irq();
    }

    /// The memory-mapped register file, back to its reset values.
    fn reset_registers(&self) {
        // §10.4.3: a reset re-initialises every port via `COMRESET`, and a
        // `COMRESET` is a hardware reset of the device — so the drives are
        // reset here rather than left as they were. Without it an adapter that
        // reset after a command would report that command's leftover taskfile
        // as the port's signature, and the ordering between this device's reset
        // and the drive's own would become guest-visible.
        for index in 0..self.ports.len() {
            if let Some(drive) = self.drive(index) {
                drive.power_on_reset();
            }
        }
        // The drives are read *before* the state lock is taken: a bay and a
        // drive both rank above this lock, and taking them the other way round
        // would be a ladder violation.
        //
        // §3.3.8 and §3.3.9: `PxTFD` and `PxSIG` take their values from the
        // first FIS a device sends, which for a device that has just come out
        // of reset is its signature — so they are asked for it rather than
        // filled in from a table here. A model that hard-coded what an ATA
        // drive answers would be keeping a second copy of a fact that lives on
        // the drive. An empty port receives no FIS and keeps `7Fh` and
        // `FFFFFFFFh`.
        let answered: Vec<Option<Registers>> = (0..self.ports.len())
            .map(|index| self.drive(index).map(|d| d.taskfile_registers()))
            .collect();
        let mut state = self.state.lock();
        *state = State::new();
        for (port, regs) in state.ports.iter_mut().zip(answered) {
            if let Some(regs) = regs {
                port.tfd = tfd_of(&regs);
                port.sig = signature_of(&regs);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// register reads
// ---------------------------------------------------------------------------

impl Hba {
    /// `CAP`, the capabilities this model actually has (§3.1.1).
    fn cap(&self) -> u32 {
        let np = (self.ports.len().max(1) - 1) as u32;
        // S64A: 64-bit data structures are supported, which is what makes
        // `PxCLBU`, `PxFBU` and a PRD's `DBAU` writable.
        (1 << 31)
            // SCLO: `PxCMD.CLO` works, so a driver can get a stuck device into
            // a state where a software reset can be sent.
            | (1 << 24)
            // ISS: Gen 3, 6 Gbps. A number with no consequence in a model with
            // no link, and reporting `0h` would mean "reserved".
            | (3 << 20)
            // SAM: AHCI only. There is no legacy task-file interface behind
            // this function, which is also why `GHC.AE` is read-only one.
            | (1 << 18)
            // PMD: multiple DRQ block PIO transfers, which §3.1.1 says an
            // AHCI 1.2 or later adapter shall report.
            | (1 << 15)
            // NCS, zero based.
            | ((COMMAND_SLOTS - 1) << 8)
            // NP, zero based.
            | np
    }

    /// `PI`, which ports are available to software (§3.1.4).
    fn pi(&self) -> u32 {
        // Contiguous from zero: this model has no reason to leave a hole, and
        // §3.1.4's whole purpose is to let a *platform* leave one.
        if self.ports.is_empty() {
            0
        } else {
            (1u32 << self.ports.len()) - 1
        }
    }

    /// `PxSSTS` for port `index` (§3.3.10).
    ///
    /// Derived rather than stored: it is a function of what is on the port and
    /// what `PxSCTL.DET` has been set to, and storing a copy would be storing
    /// derived state (`CLAUDE.md`).
    fn ssts(&self, index: usize, sctl: u32) -> u32 {
        if !self.occupied(index) {
            return 0;
        }
        match sctl & DET_MASK {
            // A `COMRESET` is being transmitted: presence is detected but the
            // Phy has not finished, which is `DET` 1h.
            DET_INIT => 1,
            // The interface is disabled and the Phy is offline.
            DET_OFFLINE => DET_OFFLINE,
            _ => SSTS_READY,
        }
    }

    /// The 32 bits a read of the register at `offset` sees.
    ///
    /// Side-effect free at every offset, which is what makes [`MemAttrs::debug`]
    /// free on the read side: §3.1 and §3.3 have no read-to-clear register, and
    /// nothing here advances a command slot or acknowledges a completion.
    fn read_dword(&self, offset: u64) -> u32 {
        if offset >= PORT_BASE {
            let index = ((offset - PORT_BASE) / PORT_STRIDE) as usize;
            let within = (offset - PORT_BASE) % PORT_STRIDE;
            if index >= self.ports.len() {
                // §3: every register not defined reads zero. Software is told
                // not to touch an unimplemented port, and this is what it finds
                // if it does anyway.
                return 0;
            }
            // `PxSSTS` and `PxSIG` are the two that need the bay rather than
            // the state lock, so they are answered before it is taken.
            if within == PORT_SSTS {
                let sctl = self.state.lock().ports[index].sctl;
                return self.ssts(index, sctl);
            }
            let port = self.state.lock().ports[index];
            return match within {
                PORT_CLB => port.clb as u32,
                PORT_CLBU => (port.clb >> 32) as u32,
                PORT_FB => port.fb as u32,
                PORT_FBU => (port.fb >> 32) as u32,
                PORT_IS => port.is,
                PORT_IE => port.ie,
                PORT_CMD => port.cmd,
                PORT_TFD => port.tfd,
                PORT_SIG => port.sig,
                PORT_SCTL => port.sctl,
                PORT_SERR => port.serr,
                PORT_SACT => port.sact,
                PORT_CI => port.ci,
                // `PxSNTF`, `PxFBS`, `PxDEVSLP` and the vendor-specific window:
                // unimplemented, and §3 says an unimplemented register reads
                // zero rather than answering something a driver might believe.
                _ => 0,
            };
        }
        match offset {
            REG_CAP => self.cap(),
            REG_GHC => self.state.lock().ghc,
            REG_IS => self.state.lock().is,
            REG_PI => self.pi(),
            REG_VS => VERSION,
            // `CCC_CTL`, `CCC_PORTS`, `EM_LOC`, `EM_CTL`, `CAP2` and `BOHC`.
            // Command completion coalescing, enclosure management and the
            // BIOS/OS handoff are all reported unsupported in `CAP`/`CAP2`, and
            // §3.1 has their registers read zero when they are.
            REG_CAP2 => 0,
            _ => 0,
        }
    }
}

// ---------------------------------------------------------------------------
// register writes
// ---------------------------------------------------------------------------

impl Hba {
    fn write_dword(&self, offset: u64, value: u32) {
        let after = if offset >= PORT_BASE {
            let index = ((offset - PORT_BASE) / PORT_STRIDE) as usize;
            let within = (offset - PORT_BASE) % PORT_STRIDE;
            if index >= self.ports.len() {
                return;
            }
            let mut state = self.state.lock();
            Hba::write_port(&mut state.ports[index], index, within, value)
        } else {
            let mut state = self.state.lock();
            match offset {
                REG_GHC => {
                    if value & GHC_HR != 0 {
                        After::HbaReset
                    } else {
                        // `AE` is read-only one and `HR` reads back zero once
                        // the reset has happened, so `IE` is the only bit a
                        // write can move.
                        state.ghc = GHC_AE | (value & GHC_IE);
                        After::Irq
                    }
                }
                REG_IS => {
                    state.is &= !value;
                    After::Irq
                }
                // `CAP`, `PI` and `VS` are read-only; `PI` in particular is
                // loaded by the platform and this model *is* the platform.
                // Everything else in the generic block belongs to a feature
                // `CAP` reports as absent.
                _ => After::Irq,
            }
        };
        match after {
            After::Irq => self.refresh_irq(),
            After::Run => {
                self.run();
                self.refresh_irq();
            }
            After::HbaReset => self.hba_reset(),
            After::Comreset(index) => {
                self.comreset(index);
                self.refresh_irq();
            }
        }
    }

    /// One port register write. Returns what has to happen once the lock is
    /// released.
    fn write_port(port: &mut PortState, index: usize, within: u64, value: u32) -> After {
        match within {
            // §3.3.1: bits 09:00 are read-only, so the command list is 1 KiB
            // aligned whatever software writes.
            PORT_CLB => {
                port.clb = (port.clb & 0xffff_ffff_0000_0000) | u64::from(value & !0x3ff);
                After::Irq
            }
            PORT_CLBU => {
                port.clb = (port.clb & 0xffff_ffff) | (u64::from(value) << 32);
                After::Irq
            }
            // §3.3.3: bits 07:00 are read-only — 256-byte alignment.
            PORT_FB => {
                port.fb = (port.fb & 0xffff_ffff_0000_0000) | u64::from(value & !0xff);
                After::Irq
            }
            PORT_FBU => {
                port.fb = (port.fb & 0xffff_ffff) | (u64::from(value) << 32);
                After::Irq
            }
            PORT_IS => {
                port.is &= !(value & IS_WRITE_CLEAR);
                After::Irq
            }
            PORT_IE => {
                port.ie = value & IE_IMPLEMENTED;
                After::Irq
            }
            PORT_CMD => {
                let was_started = port.cmd & CMD_ST != 0;
                // `CR` and `FR` are in the kept half: they are the adapter's to
                // move, not software's.
                let keep = port.cmd & !CMD_WRITABLE;
                port.cmd = keep | (value & CMD_WRITABLE) | CMD_SUD | CMD_POD;
                if port.cmd & CMD_FRE != 0 {
                    port.cmd |= CMD_FR;
                } else {
                    port.cmd &= !CMD_FR;
                }
                if value & CMD_CLO != 0 {
                    // §3.3.7: setting `CLO` clears `BSY` and `DRQ` so a software
                    // reset can be transmitted, and hardware clears the bit once
                    // it has. The device's own busy state is not touched — this
                    // is the *adapter's* copy of the taskfile, which is what
                    // §3.3.8 says `PxTFD` is.
                    port.tfd &= !u32::from(ST_BSY | ST_DRQ);
                    port.cmd &= !CMD_CLO;
                }
                let started = port.cmd & CMD_ST != 0;
                if was_started && !started {
                    // §3.3.14 and §3.3.13: `PxCI` and `PxSACT` are cleared when
                    // software writes `ST` from one to zero, which is also how
                    // §6.2.2's recovery gets rid of the command that failed.
                    port.cmd &= !(CMD_CR | CMD_CCS_MASK);
                    port.ci = 0;
                    port.sact = 0;
                    port.parked = 0;
                } else if !was_started && started {
                    // §3.3.7: after `ST` goes zero to one the highest priority
                    // slot to issue from next is slot 0.
                    port.cmd = (port.cmd | CMD_CR) & !CMD_CCS_MASK;
                }
                if port.running() {
                    After::Run
                } else {
                    After::Irq
                }
            }
            PORT_SCTL => {
                let before = port.sctl & DET_MASK;
                port.sctl = value;
                if before != value & DET_MASK {
                    After::Comreset(index)
                } else {
                    After::Irq
                }
            }
            PORT_SERR => {
                port.serr &= !value;
                // §3.3.5: `PxIS.PCS` reflects `PxSERR.DIAG.X` and is only
                // cleared when that is. The same holds for `UFS` and `DIAG.F`,
                // which this model never sets.
                if port.serr & SERR_DIAG_X == 0 {
                    port.is &= !IS_PCS;
                }
                After::Irq
            }
            // §3.3.13: write-1-to-set. Native command queuing is not supported
            // (`CAP.SNCQ` is zero) so nothing here acts on it, but the register
            // still reads back what a driver put in it rather than lying.
            PORT_SACT => {
                port.sact |= value;
                After::Irq
            }
            // §3.3.14: write-1-to-set, and "bits in this field shall only be
            // set to '1' by software when `PxCMD.ST` is set to '1'" — `ST`,
            // which is software's, rather than `CR`, which is the adapter's. A
            // port in `ERR:Fatal` has `ST` set and `CR` clear, so a slot issued
            // there does latch and simply never runs, which is what the
            // hardware leaves for §6.2.2's recovery to find in `PxCI`.
            PORT_CI => {
                if port.cmd & CMD_ST != 0 {
                    port.ci |= value;
                    if port.running() {
                        return After::Run;
                    }
                }
                After::Irq
            }
            // `PxTFD`, `PxSIG` and `PxSSTS` are read-only; `PxSNTF`, `PxFBS`
            // and `PxDEVSLP` belong to features `CAP` reports as absent.
            _ => After::Irq,
        }
    }

    /// `PxSCTL.DET` moved: the interface initialisation sequence (§3.3.11).
    ///
    /// A `COMRESET` is a hardware reset of the device, which is exactly
    /// [`AtaDisk::power_on_reset`] — the contents survive and everything else
    /// goes back to the factory. When `DET` returns to zero the link comes back
    /// up and the device sends its signature in a D2H Register FIS, which is
    /// how a driver learns what is on the port.
    fn comreset(&self, index: usize) {
        let det = self.state.lock().ports[index].sctl & DET_MASK;
        let drive = self.drive(index);
        if det == DET_INIT {
            // The lock is released before the drive is touched: the drive's own
            // lock ranks above this one.
            if let Some(drive) = &drive {
                drive.power_on_reset();
            }
            let mut state = self.state.lock();
            let port = &mut state.ports[index];
            port.ci = 0;
            port.sact = 0;
            port.parked = 0;
            port.ctl = 0;
            port.tfd = if drive.is_some() {
                u32::from(ST_BSY)
            } else {
                TFD_RESET
            };
            return;
        }
        if det != 0 {
            return;
        }
        // Communication re-established. §3.3.12: a change in device presence
        // sets `PxSERR.DIAG.X`, which `PxIS.PCS` reflects.
        let Some(drive) = drive else {
            let mut state = self.state.lock();
            state.ports[index].tfd = TFD_RESET;
            state.ports[index].sig = SIG_UNKNOWN;
            return;
        };
        let regs = drive.taskfile_registers();
        let irq = drive.taskfile_acknowledge();
        let (fb, receiving) = {
            let state = self.state.lock();
            (state.ports[index].fb, state.ports[index].receiving())
        };
        if receiving {
            self.post_fis(fb + RFIS_AT, &d2h_fis(&regs, false));
        }
        let _ = irq;
        let mut state = self.state.lock();
        let port = &mut state.ports[index];
        port.tfd = tfd_of(&regs);
        port.sig = signature_of(&regs);
        port.serr |= SERR_DIAG_X;
        port.is |= IS_PCS;
    }
}

// ---------------------------------------------------------------------------
// FIS assembly (Serial ATA 1.0 §8.5)
// ---------------------------------------------------------------------------

/// `PxTFD` from a command block: error in 15:8, status in 7:0 (§3.3.8).
fn tfd_of(regs: &Registers) -> u32 {
    (u32::from(regs.error) << 8) | u32::from(regs.status)
}

/// `PxSIG` from a command block (§3.3.9): LBA high in 31:24, LBA mid in 23:16,
/// LBA low in 15:8 and the sector count in 7:0.
///
/// This is the register a driver reads to find out *what* is on the port — an
/// ATA device leaves `00000101h` after a reset and a packet device leaves
/// `EB140101h`, and telling the two apart is the whole purpose of it. The
/// numbers are the device's, not this file's.
fn signature_of(regs: &Registers) -> u32 {
    (((regs.lba >> 16) as u32 & 0xff) << 24)
        | (((regs.lba >> 8) as u32 & 0xff) << 16)
        | ((regs.lba as u32 & 0xff) << 8)
        | u32::from(regs.count as u8)
}

/// A Register - Device to Host FIS (§8.5.3).
fn d2h_fis(regs: &Registers, interrupt: bool) -> [u8; REGISTER_FIS_LEN] {
    let mut fis = [0u8; REGISTER_FIS_LEN];
    fis[0] = FIS_D2H;
    fis[1] = if interrupt { D2H_I } else { 0 };
    fis[2] = regs.status;
    fis[3] = regs.error;
    fis[4] = regs.lba as u8;
    fis[5] = (regs.lba >> 8) as u8;
    fis[6] = (regs.lba >> 16) as u8;
    fis[7] = regs.device;
    fis[8] = (regs.lba >> 24) as u8;
    fis[9] = (regs.lba >> 32) as u8;
    fis[10] = (regs.lba >> 40) as u8;
    fis[12] = regs.count as u8;
    fis[13] = (regs.count >> 8) as u8;
    fis
}

/// A PIO Setup - Device to Host FIS (§8.5.8).
///
/// It carries **both** status values, which is the whole reason it exists: the
/// one the host is to see while the block moves, and the one to latch when the
/// last byte has gone. A model that reported only the second would show a
/// driver a finished command in the middle of one.
fn pio_setup_fis(
    regs: &Registers,
    status_before: u8,
    end_status: u8,
    device_to_host: bool,
    count: u16,
    interrupt: bool,
) -> [u8; REGISTER_FIS_LEN] {
    let mut fis = [0u8; REGISTER_FIS_LEN];
    fis[0] = FIS_PIO_SETUP;
    fis[1] = if interrupt { D2H_I } else { 0 } | if device_to_host { PIO_D } else { 0 };
    fis[2] = status_before;
    fis[3] = regs.error;
    fis[4] = regs.lba as u8;
    fis[5] = (regs.lba >> 8) as u8;
    fis[6] = (regs.lba >> 16) as u8;
    fis[7] = regs.device;
    fis[8] = (regs.lba >> 24) as u8;
    fis[9] = (regs.lba >> 32) as u8;
    fis[10] = (regs.lba >> 40) as u8;
    fis[12] = regs.count as u8;
    fis[13] = (regs.count >> 8) as u8;
    fis[15] = end_status;
    fis[16] = count as u8;
    fis[17] = (count >> 8) as u8;
    fis
}

/// A little-endian dword out of a slice.
fn le32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

// ---------------------------------------------------------------------------
// the engine
// ---------------------------------------------------------------------------

/// One command picked off a port's command list, ready to run.
#[derive(Debug, Clone, Copy)]
struct Job {
    port: usize,
    slot: u32,
    clb: u64,
    fb: u64,
    receiving: bool,
}

/// One command header, as the fields this adapter reads (§4.2.2).
#[derive(Debug, Clone, Copy)]
struct Header {
    /// Command FIS Length, in dwords. Two to sixteen; anything else is illegal.
    cfl: u64,
    /// `PRDTL`: how many entries the scatter/gather list holds.
    prdtl: u32,
    /// `C`: clear `BSY` and the command's `PxCI` bit once the FIS has gone.
    clear_busy: bool,
    /// The command table's base, already 128-byte aligned.
    ctba: u64,
}

/// A cursor over one command's Physical Region Descriptor Table (§4.2.3.3).
///
/// Loads one descriptor at a time rather than the whole table: `PRDTL` may be
/// 65,535, each entry may name 4 MiB, and a model that gathered the list first
/// would allocate a megabyte before moving a byte.
#[derive(Debug)]
struct Prdt {
    base: u64,
    len: u32,
    next: u32,
    addr: u64,
    left: u64,
    /// Whether the descriptor currently open asked for an interrupt when it
    /// finishes (§5.4.2).
    interrupt: bool,
    /// Whether any descriptor with that bit set has finished.
    fired: bool,
}

impl Prdt {
    fn new(base: u64, len: u32) -> Prdt {
        Prdt {
            base,
            len: len.min(MAX_PRDT),
            next: 0,
            addr: 0,
            left: 0,
            interrupt: false,
            fired: false,
        }
    }

    /// Where the next `want` bytes go, and how many of them this descriptor
    /// takes. `None` when the table is exhausted — which is an overflow if the
    /// device still has data (§6.1.5) and an ordinary short transfer if it does
    /// not.
    fn take(&mut self, hba: &Hba, space: &AddressSpace, want: usize) -> Option<(u64, usize)> {
        while self.left == 0 {
            if self.next >= self.len {
                return None;
            }
            let at = self.base + u64::from(self.next) * PRD_LEN;
            let mut raw = [0u8; PRD_LEN as usize];
            if space.read_bytes(at, &mut raw, hba.attrs()).is_err() {
                return None;
            }
            self.next += 1;
            // §4.2.3.3: bit 0 of `DBA` is reserved, so a data block is word
            // aligned; `DBC` is a zero-based byte count in bits 21:0 whose bit
            // 0 must be one, which is the same statement about even lengths.
            let dw3 = le32(&raw[12..16]);
            self.addr = (u64::from(le32(&raw[0..4])) & !1) | (u64::from(le32(&raw[4..8])) << 32);
            self.left = u64::from(dw3 & 0x003f_ffff) + 1;
            self.interrupt = dw3 & (1 << 31) != 0;
        }
        let n = core::cmp::min(self.left, want as u64) as usize;
        Some((self.addr, n))
    }

    /// Advance past `n` bytes just moved.
    fn advance(&mut self, n: usize) {
        self.addr += n as u64;
        self.left -= n as u64;
        if self.left == 0 && self.interrupt {
            self.fired = true;
        }
    }
}

impl Hba {
    /// Whether any running port has a command waiting.
    fn has_work(&self) -> bool {
        let state = self.state.lock();
        state
            .ports
            .iter()
            .take(self.ports.len())
            .any(|p| p.running() && p.ci & !p.parked != 0)
    }

    /// The next command to run, with `PxCMD.CCS` moved to its slot.
    fn next_job(&self) -> Option<Job> {
        let mut state = self.state.lock();
        for index in 0..self.ports.len() {
            let port = &mut state.ports[index];
            if !port.running() {
                continue;
            }
            let live = port.ci & !port.parked;
            if live == 0 {
                continue;
            }
            let slot = live.trailing_zeros();
            port.cmd = (port.cmd & !CMD_CCS_MASK) | ((slot << CMD_CCS_SHIFT) & CMD_CCS_MASK);
            return Some(Job {
                port: index,
                slot,
                clb: port.clb,
                fb: port.fb,
                receiving: port.receiving(),
            });
        }
        None
    }

    /// Run every command the driver has made available.
    ///
    /// Iterative, not recursive: a `PxCI` write reached from inside one of this
    /// adapter's own guest-memory accesses records the bits and returns, and the
    /// loop below picks the work up. See the module documentation.
    pub fn run(&self) {
        // *PCI Local Bus Specification* Rev 2.1 §6.2.2: a function that has not
        // been granted Bus Master Enable generates no accesses of its own, so it
        // cannot fetch a command header. A driver sets the bit before it writes
        // `PxCI`; one that forgot sees nothing happen, as on the hardware.
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
            // Another master may have written `PxCI` between the last check and
            // this release. Re-check, and take the flag back only if there is
            // something to do.
            if budget == 0 || !self.has_work() || self.busy.swap(true, Ordering::AcqRel) {
                break;
            }
        }
    }

    /// Stop a port because something it cannot recover from happened.
    ///
    /// §6.2.2: a fatal error — `HBFS`, `HBDS`, `IFS` or `TFES` — puts the port
    /// in `ERR:Fatal` and **clears `PxCMD.CR`**, deliberately not `PxCMD.ST`.
    /// `PxCI` is left alone so software can read which command was outstanding,
    /// and `PxCMD.CCS` keeps the slot the adapter was on. The port is restarted
    /// by writing `ST` one-to-zero, which resets `PxCI`, and then back to one.
    ///
    /// Clearing `CR` is also what makes the engine terminate:
    /// [`Hba::next_job`] tests `CR` and will not offer the port again.
    fn port_fatal(&self, index: usize, bit: u32) {
        let mut state = self.state.lock();
        let port = &mut state.ports[index];
        port.is |= bit;
        port.cmd &= !CMD_CR;
    }

    /// Write a FIS into the received-FIS area, if the port is accepting them.
    fn post_fis(&self, at: u64, fis: &[u8]) -> bool {
        let Some(space) = self.space() else {
            return false;
        };
        space.write_bytes(at, fis, self.attrs()).is_ok()
    }

    /// Fetch, run and complete one command.
    fn execute(&self, job: &Job) {
        let Some(space) = self.space() else {
            self.port_fatal(job.port, IS_HBFS);
            return;
        };
        // §4.2.2: the command list is 1 KiB and holds 32 headers, so a slot is
        // in range by construction — but the base came from the guest, and a
        // header that cannot be read is a host bus error (§6.1.1) rather than a
        // command invented out of a fault.
        let at = job.clb + u64::from(job.slot) * HEADER_LEN;
        debug_assert!(u64::from(job.slot) * HEADER_LEN < COMMAND_LIST_LEN);
        let mut raw = [0u8; HEADER_LEN as usize];
        if space.read_bytes(at, &mut raw, self.attrs()).is_err() {
            self.port_fatal(job.port, IS_HBFS);
            return;
        }
        let dw0 = le32(&raw[0..4]);
        let header = Header {
            cfl: u64::from(dw0 & 0x1f),
            prdtl: dw0 >> 16,
            clear_busy: dw0 & (1 << 10) != 0,
            // §4.2.2: bits 06:00 of `CTBA` are reserved — 128-byte alignment.
            ctba: (u64::from(le32(&raw[8..12])) & !0x7f) | (u64::from(le32(&raw[12..16])) << 32),
        };
        // §4.2.2: "A length of 0 or 1 is illegal. The maximum value allowed is
        // 10h." An illegal one is a FIS the adapter cannot send, which §6.1.2
        // makes an interface error.
        if !(2..=16).contains(&header.cfl) {
            self.port_fatal(job.port, IS_IFS);
            return;
        }
        let mut cfis = [0u8; 64];
        let len = (header.cfl * 4) as usize;
        if space
            .read_bytes(header.ctba, &mut cfis[..len], self.attrs())
            .is_err()
        {
            self.port_fatal(job.port, IS_HBFS);
            return;
        }
        if cfis[0] != FIS_H2D {
            // §4.2.3.1: for data transfer operations the command FIS is the
            // H2D Register FIS. Anything else is a FIS this adapter cannot
            // transmit.
            self.port_fatal(job.port, IS_IFS);
            return;
        }
        let Some(drive) = self.drive(job.port) else {
            // A running port with nothing on it. There is no device to answer,
            // which is what §6.1.2's interface error covers.
            self.port_fatal(job.port, IS_IFS);
            return;
        };
        if cfis[1] & H2D_C == 0 {
            self.control_fis(job, &drive, &header, cfis[15]);
        } else {
            self.command_fis(job, &space, &drive, &header, &cfis);
        }
    }

    /// A Register - Host to Device FIS with `C` clear: a Device Control write.
    ///
    /// Serial ATA §8.5.2.3: the device acts on the *Control* register rather
    /// than starting a command. AHCI §10.4.1 uses exactly two of these for a
    /// software reset — one asserting `SRST` and one releasing it — and the
    /// asymmetry is the whole subtlety: while `SRST` is asserted the device is
    /// busy and answers nothing, so only the command header's `C` bit can clear
    /// `PxCI` for that slot.
    fn control_fis(&self, job: &Job, drive: &Arc<AtaDisk>, header: &Header, control: u8) {
        let previous = self.state.lock().ports[job.port].ctl;
        drive.write_device_control(control);
        let released = previous & CTL_SRST != 0 && control & CTL_SRST == 0;
        let regs = drive.taskfile_registers();
        let interrupt = drive.taskfile_acknowledge();
        if released && job.receiving {
            // The device comes out of reset and reports its signature. §10.4.1:
            // this D2H clears `BSY` for the slot, which is what clears `PxCI`.
            self.post_fis(job.fb + RFIS_AT, &d2h_fis(&regs, interrupt));
        }
        let mut state = self.state.lock();
        let port = &mut state.ports[job.port];
        port.ctl = control;
        port.tfd = tfd_of(&regs);
        if released {
            // §3.3.9: `PxSIG` is updated once after a reset sequence, and this
            // is that sequence's end.
            port.sig = signature_of(&regs);
            // Every slot the reset sequence parked belongs to that sequence and
            // is answered by the same FIS.
            port.ci &= !port.parked;
            port.parked = 0;
            port.ci &= !(1 << job.slot);
            if interrupt {
                port.is |= IS_DHRS;
            }
        } else if header.clear_busy {
            port.tfd &= !u32::from(ST_BSY | ST_DRQ);
            port.ci &= !(1 << job.slot);
        } else {
            // Issued, sent, and the device is silent. The slot stays
            // outstanding — which is what the hardware does — and is parked so
            // that the engine moves on to the next one instead of re-sending it.
            port.parked |= 1 << job.slot;
        }
    }

    /// A Register - Host to Device FIS with `C` set: an ATA command.
    fn command_fis(
        &self,
        job: &Job,
        space: &AddressSpace,
        drive: &Arc<AtaDisk>,
        header: &Header,
        cfis: &[u8; 64],
    ) {
        // Serial ATA §8.5.2, Figure 58. Six named fields out of twenty bytes:
        // this is the only place in the tree that knows where they sit.
        let tf = Taskfile {
            command: cfis[2],
            feature: u16::from(cfis[3]) | (u16::from(cfis[11]) << 8),
            count: u16::from(cfis[12]) | (u16::from(cfis[13]) << 8),
            lba: u64::from(cfis[4])
                | (u64::from(cfis[5]) << 8)
                | (u64::from(cfis[6]) << 16)
                | (u64::from(cfis[8]) << 24)
                | (u64::from(cfis[9]) << 32)
                | (u64::from(cfis[10]) << 40),
            device: cfis[7],
        };
        {
            // §3.3.8: the adapter sets `BSY` when it transmits the command FIS.
            let mut state = self.state.lock();
            state.ports[job.port].tfd |= u32::from(ST_BSY);
        }
        let mut phase = drive.taskfile_start(&tf);
        let mut prdt = Prdt::new(header.ctba + PRDT_OFFSET, header.prdtl);
        let mut scratch = [0u8; CHUNK];
        let mut moved: u64 = 0;
        let mut blocks: u64 = 0;
        let mut pio = false;
        let mut trouble: u32 = 0;

        while let Phase::Data { out, dma, block } = phase {
            blocks += 1;
            if blocks > MAX_BLOCKS || moved.saturating_add(block) > MAX_TRANSFER {
                // More than the command set can name. A guest that got here
                // built something a drive cannot have produced.
                trouble |= IS_OFS;
                break;
            }
            let before = drive.taskfile_registers().status;
            let mut left = block;
            while left > 0 {
                let want = core::cmp::min(left as usize, CHUNK);
                let Some((addr, n)) = prdt.take(self, space, want) else {
                    // §6.1.5: the device has more to move than the PRD table
                    // has room for.
                    trouble |= IS_OFS;
                    break;
                };
                let did = if out {
                    if space
                        .read_bytes(addr, &mut scratch[..n], self.attrs())
                        .is_err()
                    {
                        trouble |= IS_HBFS;
                        break;
                    }
                    drive.taskfile_write(&scratch[..n])
                } else {
                    let got = drive.taskfile_read(&mut scratch[..n]) as usize;
                    if space
                        .write_bytes(addr, &scratch[..got], self.attrs())
                        .is_err()
                    {
                        trouble |= IS_HBFS;
                        break;
                    }
                    got as u64
                };
                if did == 0 {
                    // The drive said it had a block and then moved nothing.
                    // Unreachable by construction — `Phase::Data` means `DRQ`
                    // with a non-empty block — and a loop that trusted that
                    // would spin.
                    trouble |= IS_IFS;
                    break;
                }
                prdt.advance(did as usize);
                moved += did;
                left -= did;
            }
            if trouble != 0 {
                break;
            }
            let regs = drive.taskfile_registers();
            if !dma {
                // §5.6.3: a PIO command's data phase is announced by a PIO
                // Setup FIS carrying the status to show while the block moves
                // and the status to latch when it has.
                pio = true;
                let interrupt = drive.taskfile_acknowledge();
                if job.receiving {
                    self.post_fis(
                        job.fb + PSFIS_AT,
                        &pio_setup_fis(&regs, before, regs.status, !out, block as u16, interrupt),
                    );
                }
                let mut state = self.state.lock();
                let port = &mut state.ports[job.port];
                port.tfd = tfd_of(&regs);
                if interrupt {
                    port.is |= IS_PSS;
                }
            }
            phase = drive.taskfile_phase();
        }

        // §6.1.5 distinguishes the two directions of an overflow, and the
        // difference matters more than it looks.
        //
        // On a **read** the adapter "shall make a best effort to continue": the
        // device has bytes and there is nowhere to put them, so they are read
        // and discarded, the command finishes, and `PRDBC` says how many
        // actually landed. The port keeps running.
        //
        // On a **write** the adapter has run out of data to give a device that
        // is still asking, and §6.1.5 is explicit that this is fatal — "a
        // COMRESET is required by software to clean up from this serious
        // error", and an HBA is even allowed to hang. What must *not* happen is
        // this model's obvious shortcut: feeding the drive zeroes to get the
        // command over with would write them to the medium, which is the one
        // outcome worse than stopping. So the drive is left holding its block,
        // `PxTFD` reports the `DRQ` that says so, and §6.2.2.1's recovery — see
        // `PxTFD.STS.DRQ`, issue a COMRESET — is what a driver does about it.
        let mut write_overflow = false;
        if trouble & IS_OFS != 0 {
            match drive.taskfile_phase() {
                Phase::Data { out: false, .. } => self.discard(drive),
                Phase::Data { out: true, .. } => write_overflow = true,
                Phase::Done => {}
            }
        }

        let regs = drive.taskfile_registers();
        let interrupt = drive.taskfile_acknowledge();
        // §5.4.1: `PRDBC` is the byte count that actually transferred, and
        // software reads it to find a short transfer.
        let mut prdbc = [0u8; 4];
        prdbc.copy_from_slice(&(moved as u32).to_le_bytes());
        let bc_at = job.clb + u64::from(job.slot) * HEADER_LEN + 4;
        if space.write_bytes(bc_at, &prdbc, self.attrs()).is_err() {
            trouble |= IS_HBFS;
        }
        if !pio && job.receiving {
            // §5.6.2: a DMA or non-data command ends with a D2H Register FIS.
            // A PIO one does not — its last PIO Setup FIS carried the ending
            // status, which is why `PxIS.PSS` rather than `PxIS.DHRS` is what
            // a driver waits on there.
            self.post_fis(job.fb + RFIS_AT, &d2h_fis(&regs, interrupt));
        }

        let failed = regs.status & ST_ERR != 0;
        let mut raised = trouble;
        if failed {
            // §6.1.4: the status came back with `ERR` set.
            raised |= IS_TFES;
        }
        let mut state = self.state.lock();
        let port = &mut state.ports[job.port];
        port.tfd = tfd_of(&regs);
        if prdt.fired {
            port.is |= IS_DPS;
        }
        if !pio && interrupt {
            port.is |= IS_DHRS;
        }
        port.is |= raised;
        if raised & FATAL != 0 || write_overflow {
            // §6.2.2: `HBFS`, `HBDS`, `IFS` and `TFES` are the fatal ones, and
            // §6.1.5 puts a write overflow with them. The adapter stops issuing
            // and `PxCI` keeps the failing slot, so software can see which
            // command it was. A *read* overflow is explicitly not fatal —
            // "the HBA continues to operate" — so a short PRD table on a read
            // reports itself and the command still completes.
            port.cmd &= !CMD_CR;
        } else {
            port.ci &= !(1 << job.slot);
        }
    }

    /// Read out and throw away a data-in phase whose bytes have nowhere to go.
    ///
    /// §6.1.5's "best effort to continue" on a read overflow, and the only
    /// direction it is safe in: there is no equivalent for a write, because the
    /// only thing an adapter with no data could hand the drive is data it made
    /// up, and the drive would put it on the medium.
    ///
    /// Bounded by `MAX_BLOCKS` for the reason every walk here is bounded: the
    /// drive is a sibling device and this loop must terminate on its own terms.
    fn discard(&self, drive: &Arc<AtaDisk>) {
        let mut scratch = [0u8; CHUNK];
        for _ in 0..MAX_BLOCKS {
            match drive.taskfile_phase() {
                Phase::Data { out: false, .. } => {
                    if drive.taskfile_read(&mut scratch) == 0 {
                        return;
                    }
                }
                _ => return,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

impl Hba {
    /// Serialize the register file.
    ///
    /// Nothing else needs saving: the command lists, the received-FIS areas and
    /// the data are all in guest memory, and a command never spans a snapshot
    /// because it completes inside the `PxCI` write that issued it. `master` and
    /// `intx_disabled` are the PCI Command register, which the function saves,
    /// and re-deriving them on load is what keeps derived state out of a
    /// snapshot (`CLAUDE.md`).
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let state = *self.state.lock();
        w.write_u32(state.ghc)?;
        w.write_u32(state.is)?;
        w.write_u64(self.ports.len() as u64)?;
        for port in state.ports.iter().take(self.ports.len()) {
            w.write_u64(port.clb)?;
            w.write_u64(port.fb)?;
            w.write_u32(port.is)?;
            w.write_u32(port.ie)?;
            w.write_u32(port.cmd)?;
            w.write_u32(port.tfd)?;
            w.write_u32(port.sctl)?;
            w.write_u32(port.serr)?;
            w.write_u32(port.sact)?;
            w.write_u32(port.ci)?;
            w.write_u32(port.sig)?;
            w.write_u32(port.parked)?;
            w.write_u8(port.ctl)?;
        }
        Ok(())
    }

    /// Restore what [`Hba::save`] wrote.
    ///
    /// # Errors
    ///
    /// [`crate::Error::State`] if the chunk describes an adapter with a
    /// different number of ports, and whatever the source refuses.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let ghc = r.read_u32()?;
        let is = r.read_u32()?;
        let count = r.read_u64()?;
        if count != self.ports.len() as u64 {
            return Err(crate::core::error::Error::State(alloc::format!(
                "the snapshot has {count} port(s) and this adapter has {}",
                self.ports.len()
            )));
        }
        let mut ports = [PortState::new(); MAX_PORTS];
        for port in ports.iter_mut().take(self.ports.len()) {
            port.clb = r.read_u64()?;
            port.fb = r.read_u64()?;
            port.is = r.read_u32()?;
            port.ie = r.read_u32()?;
            port.cmd = r.read_u32()?;
            port.tfd = r.read_u32()?;
            port.sctl = r.read_u32()?;
            port.serr = r.read_u32()?;
            port.sact = r.read_u32()?;
            port.ci = r.read_u32()?;
            port.sig = r.read_u32()?;
            port.parked = r.read_u32()?;
            port.ctl = r.read_u8()?;
            // Only the bits that exist: `SUD` and `POD` are read-only ones and
            // everything outside this mask is reserved, so a hostile chunk
            // cannot leave the register file describing a state the hardware
            // cannot be in. `FR` follows `FRE`, which is the one invariant a
            // saved `PxCMD` could otherwise arrive violating.
            port.cmd = (port.cmd & (CMD_WRITABLE | CMD_CR | CMD_FR)) | CMD_SUD | CMD_POD;
            if port.cmd & CMD_FRE == 0 {
                port.cmd &= !CMD_FR;
            }
        }
        {
            let mut state = self.state.lock();
            state.ghc = GHC_AE | (ghc & GHC_IE);
            state.is = is;
            state.ports = ports;
        }
        self.refresh_irq();
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// the memory-mapped face
// ---------------------------------------------------------------------------

impl MemOps for Hba {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // No `debug` branch, and that is a claim rather than an omission: §3.1
        // and §3.3 have no read-to-clear register, `PxIS` and `IS` are cleared
        // by *writing* ones, and nothing here advances a command slot or
        // acknowledges a completion. Every side effect this device has is on the
        // write side, and those are refused outright below.
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
                // §3: "64-bit access must not cross an 8-byte alignment
                // boundary", which is the same statement as natural alignment.
                if !offset.is_multiple_of(8) {
                    return Err(BusError::BadAccess);
                }
                let value = u64::from(self.read_dword(offset))
                    | (u64::from(self.read_dword(offset + 4)) << 32);
                dst.copy_from_slice(&value.to_le_bytes());
            }
            // §3: the registers are 32- or 64-bit quantities. A byte access is
            // not a register access, and answering one would be invention.
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // There is no harmless write here. `PxCI` runs a command, `PxCMD.ST`
            // starts and stops the engine, `GHC.HR` resets the adapter, and
            // `PxIS` and `IS` are write-1-to-clear — a debugger that touched any
            // of them would be a debugger that changed the guest.
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
                // A 64-bit register may be written as one 64-bit access or as
                // two 32-bit ones, low half first. Both land here as the same
                // two writes, which is what keeps `PxCLB` from being half
                // updated by one path and whole by the other.
                self.write_dword(offset, le32(&src[0..4]));
                self.write_dword(offset + 4, le32(&src[4..8]));
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

/// The bytes one port's command list occupies in guest memory (§4.2.2), and the
/// bytes its received-FIS structure occupies (§4.2.1).
///
/// Exported because a machine description does not allocate them — the *driver*
/// does — and a test that builds one wants the sizes from here rather than from
/// a comment.
#[must_use]
pub fn structure_sizes() -> (u64, u64) {
    (COMMAND_LIST_LEN, RECEIVED_FIS_LEN)
}

/// Where port `index`'s registers sit inside the register block (§3.3).
#[must_use]
pub fn port_offset(index: usize) -> u64 {
    PORT_BASE + index as u64 * PORT_STRIDE
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod probe {
    //! A place for the constants above to be asserted against each other.
    use super::*;

    #[test]
    fn the_register_window_holds_every_port() {
        assert!(port_offset(MAX_PORTS - 1) + PORT_STRIDE <= REGISTER_LEN);
        assert!(REGISTER_LEN.is_power_of_two());
    }

    #[test]
    fn the_command_table_layout_matches_the_specification() {
        // §4.2.3: 64 bytes of command FIS, 16 of ATAPI command, 48 reserved.
        assert_eq!(PRDT_OFFSET, 64 + 16 + 48);
        assert_eq!(COMMAND_LIST_LEN, 1024);
        assert_eq!(RECEIVED_FIS_LEN, 256);
    }

    #[test]
    fn a_vector_of_zero_ports_still_reports_something_legal() {
        let hba = Hba::new(Vec::new());
        assert_eq!(hba.pi(), 0);
        // `CAP.NP` is zero based and its minimum is one port, so an adapter
        // with none still says `0h` rather than underflowing.
        assert_eq!(hba.cap() & 0x1f, 0);
    }
}
