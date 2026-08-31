//! A generic EHCI host controller: the register file, and the schedule walker.
//!
//! # What an EHCI controller actually is
//!
//! Almost none of EHCI is the register block. The register block is thirty-odd
//! bytes of capability and operational registers; the *controller* is the thing
//! that reads **linked lists the driver built in guest RAM** and executes them.
//! A driver hands over two roots — `ASYNCLISTADDR` and `PERIODICLISTBASE` —
//! sets `USBCMD.RS`, and from then on the controller is a bus master walking
//! queue heads and queue element transfer descriptors of its own accord, moving
//! bytes between guest memory and USB devices, and raising an interrupt when a
//! descriptor with `IOC` set retires.
//!
//! ```text
//!   ASYNCLISTADDR ──► QH ──► QH ──► QH ──┐   circular; the H-bit QH is the head
//!                     │                  │
//!                     └──────────────────┘
//!                     │ overlay (a qTD inside the QH)
//!                     ▼
//!            CurrentqTD ──► qTD ──► qTD ──► T
//!                            │
//!                            └─► five 4 KiB buffer pages
//!
//!   PERIODICLISTBASE ──► [1024 frame-list entries]
//!                          │ FRINDEX[12:3] selects one per frame
//!                          ▼
//!                        QH (interrupt) ──► QH ──► T
//!                        µFrame S-mask picks which of the eight microframes
//! ```
//!
//! # The walk is guest-controlled, so it is bounded
//!
//! Every pointer above comes from guest memory. A queue head may link to
//! itself, a qTD may point at itself, and a frame-list entry may close a
//! circle — a driver bug does that by accident and a hostile guest does it on
//! purpose. So every loop here has a hard bound ([`MAX_ASYNC_QH`],
//! [`MAX_PERIODIC_NODES`], [`MAX_QTD_ADVANCE`], [`MAX_PACKETS`]), and reaching
//! one simply ends the microframe: the controller comes back to it on the next
//! one, which is what real silicon does too since it only ever has one
//! microframe's worth of time. `fuzz/fuzz_targets/usb_ehci.rs` drives arbitrary
//! bytes through the register block and arbitrary structures through the
//! schedule for exactly this reason.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). This is a *lazily advanced* device
//! (`ROADMAP.md` §4.2): it holds its own tick, publishes the tick its next
//! microframe falls on, and is caught up before any register access and at
//! every microframe boundary by the machine's quantum limit. It never sleeps,
//! never reads a host clock, and never spawns anything.
//!
//! A microframe is 125 µs and a frame is 1 ms — §4.2 calls a USB frame timer
//! exactly the awkward-rate case the oscillator forest exists for. It is only
//! awkward if you insist on deriving it from seconds: put the controller on the
//! 60 MHz clock a USB 2.0 PHY actually runs at and a microframe is **exactly**
//! 7500 ticks and a frame exactly 60 000, with no residue and no float. That is
//! what [`Params::microframe_ticks`] is, and a machine file that puts the
//! controller on a domain where the number is not exact has said something
//! false about its board.
//!
//! # `MemAttrs::debug`
//!
//! Two traps, and both are tested:
//!
//! * `USBSTS` is **write-1-to-clear**. A debugger write would acknowledge an
//!   interrupt the guest has not seen, so a debug write is refused outright
//!   ([`BusError::BadAccess`]) rather than made harmless — there is no harmless
//!   version of "start the controller" either.
//! * A debug **read** must not advance the schedule. Reads sync the device with
//!   [`AccessKind::Debug`], which advances nothing, so reading `FRINDEX` from a
//!   monitor does not move the frame counter or execute a transfer.
//!
//! # What is modelled, and what is not
//!
//! * **Asynchronous schedule** (control and bulk): queue heads, queue element
//!   transfer descriptors, the transfer overlay, short-packet handling with the
//!   alternate-next pointer, data toggles, `IOC`, halt on `STALL`, and the
//!   Interrupt-on-Async-Advance doorbell. Yes.
//! * **Periodic schedule** (interrupt): the frame list, queue heads with an
//!   `µFrame S-mask`. Yes.
//! * **Isochronous**: `iTD` and `siTD` nodes are *skipped* — the walker follows
//!   their link pointers and executes nothing. A frame list containing them
//!   still works; the isochronous data does not move. Said plainly rather than
//!   half-implemented.
//! * **Split transactions**: not modelled, and there is nothing to split to,
//!   because there is no hub device (`crate::bus::usb`).
//! * **Full and low speed**: EHCI cannot drive them (EHCI 1.0 §4.2). This
//!   controller does the honest thing and **hands the port to a companion** by
//!   setting `PORTSC.Port Owner`, exactly as §4.2.2 requires, so a full-speed
//!   device attached to a bare EHCI *disappears* instead of silently
//!   enumerating on a bus that could not have carried it.
//! * **64-bit addressing**: `HCCPARAMS` reports 32-bit, so `CTRLDSSEGMENT`
//!   stores and reads back but is not folded into an address.
//! * **`USBSTS.Reclamation`** (bit 13) reads zero always. It is the
//!   controller's own idle-detection state, read-only and informational; a
//!   constant zero means "not reclaiming", which is always a safe thing for a
//!   driver to see, and is more honest than a bit that is right half the time.
//!
//! # Sources
//!
//! The **Enhanced Host Controller Interface Specification for Universal Serial
//! Bus, revision 1.0** (Intel, freely available) — §2.2 capability registers,
//! §2.3 operational registers, §3.5 the queue element transfer descriptor, §3.6
//! the queue head, §4.8 the asynchronous schedule, §4.10 the transfer overlay
//! state machine — and **USB 2.0** for everything above the controller. No
//! emulator source was consulted (`ROADMAP.md` §1); in particular the Linux
//! kernel's USB stack is GPLv2 and was not opened.

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};

use core::fmt;

use crate::bus::usb::{
    Completion, DeviceAddress, HCD_RANK, MAX_PORTS, SetupPacket, Speed, Status, UsbBus, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.ehci";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub(crate) const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Capability registers (EHCI 1.0 §2.2)
// ---------------------------------------------------------------------------

/// How far past the capability registers the operational ones start, for a
/// controller built with the standard layout.
///
/// The value is entirely the implementation's choice — it exists *because* it
/// varies, which is what lets [`crate::dev::usb::chipidea`] put the operational
/// registers at `+0x140` without changing anything else.
pub const DEFAULT_CAPLENGTH: u8 = 0x20;

/// `HCIVERSION` (§2.2.2): BCD, and 1.0 is the only revision there is.
pub const HCIVERSION: u16 = 0x0100;

/// How much address space a standard-layout controller's register block takes.
pub const REGISTER_BYTES: u64 = 0x100;

// ---------------------------------------------------------------------------
// USBCMD (§2.3.1)
// ---------------------------------------------------------------------------

/// Run/Stop.
const CMD_RS: u32 = 1 << 0;
/// Host controller reset. Self-clearing.
const CMD_HCRESET: u32 = 1 << 1;
/// Frame list size, bits 3:2. Only meaningful with `HCCPARAMS.PFLF`.
const CMD_FLS_SHIFT: u32 = 2;
/// Periodic schedule enable.
const CMD_PSE: u32 = 1 << 4;
/// Asynchronous schedule enable.
const CMD_ASE: u32 = 1 << 5;
/// Interrupt on async advance doorbell.
const CMD_IAAD: u32 = 1 << 6;
/// Light host controller reset. Optional, and this one does not have it.
const CMD_LHCR: u32 = 1 << 7;
/// Everything software may set. Reserved bits read back zero.
const CMD_MASK: u32 = CMD_RS
    | CMD_HCRESET
    | (0x3 << CMD_FLS_SHIFT)
    | CMD_PSE
    | CMD_ASE
    | CMD_IAAD
    | CMD_LHCR
    | (0x3 << 8)
    | (1 << 11)
    | (0xff << 16);

/// `USBCMD` out of reset (§2.3.1): interrupt threshold of eight microframes,
/// everything else clear.
const CMD_RESET_VALUE: u32 = 0x0008_0000;

// ---------------------------------------------------------------------------
// USBSTS (§2.3.2) and USBINTR (§2.3.3)
// ---------------------------------------------------------------------------

/// A transfer with `IOC` retired, or a short packet completed one.
pub const STS_USBINT: u32 = 1 << 0;
/// A transfer completed with an error.
pub const STS_USBERRINT: u32 = 1 << 1;
/// A port changed connect, enable or overcurrent state.
pub const STS_PORT_CHANGE: u32 = 1 << 2;
/// `FRINDEX` rolled over the top of the frame list.
pub const STS_FLR: u32 = 1 << 3;
/// A DMA access faulted.
pub const STS_HSE: u32 = 1 << 4;
/// The async-advance doorbell has been answered.
pub const STS_IAA: u32 = 1 << 5;
/// The controller is not running.
pub const STS_HCHALTED: u32 = 1 << 12;
/// Reclamation. Read-only, and this controller reads it zero — see the module
/// docs.
pub const STS_RECLAMATION: u32 = 1 << 13;
/// The periodic schedule is running.
pub const STS_PSS: u32 = 1 << 14;
/// The asynchronous schedule is running.
pub const STS_ASS: u32 = 1 << 15;

/// The write-1-to-clear half of `USBSTS`, and the half `USBINTR` enables.
///
/// **This is the `MemAttrs::debug` trap**: a debug write here would
/// acknowledge an interrupt on the guest's behalf.
pub const STS_W1C: u32 = STS_USBINT | STS_USBERRINT | STS_PORT_CHANGE | STS_FLR | STS_HSE | STS_IAA;

// ---------------------------------------------------------------------------
// PORTSC (§2.3.9)
// ---------------------------------------------------------------------------

/// Something is plugged in.
pub const PORT_CCS: u32 = 1 << 0;
/// The connect status changed. Write-1-to-clear.
pub const PORT_CSC: u32 = 1 << 1;
/// The port is enabled.
pub const PORT_PE: u32 = 1 << 2;
/// The enable state changed. Write-1-to-clear.
pub const PORT_PEC: u32 = 1 << 3;
/// Overcurrent. Never asserted here: a modelled bus has no current.
const PORT_OCA: u32 = 1 << 4;
/// Overcurrent changed. Write-1-to-clear.
const PORT_OCC: u32 = 1 << 5;
/// Force port resume.
const PORT_FPR: u32 = 1 << 6;
/// The port is suspended.
const PORT_SUSPEND: u32 = 1 << 7;
/// Drive a bus reset. **Software-cleared** (§2.3.9): the driver sets it, waits
/// its own 50 ms, and clears it, and the port is enabled when it does.
pub const PORT_RESET: u32 = 1 << 8;
/// Line status, bits 11:10. `01` is the K state, which is how a low-speed
/// device announces itself before any reset.
const PORT_LS_SHIFT: u32 = 10;
/// Port power. `HCSPARAMS.PPC` is zero here, so this reads one always.
pub const PORT_PP: u32 = 1 << 12;
/// The port belongs to a companion controller, not to this one.
pub const PORT_OWNER: u32 = 1 << 13;
/// The write-1-to-clear bits of `PORTSC`.
const PORT_W1C: u32 = PORT_CSC | PORT_PEC | PORT_OCC;
/// The bits software may set directly.
const PORT_WRITABLE: u32 =
    PORT_PE | PORT_FPR | PORT_SUSPEND | PORT_RESET | PORT_OWNER | (0x3 << 14);

// ---------------------------------------------------------------------------
// Link pointers, queue heads and qTDs (§3.5, §3.6)
// ---------------------------------------------------------------------------

/// A link pointer's terminate bit: nothing follows.
const LINK_T: u32 = 1 << 0;
/// A frame-list link pointer's type field, bits 2:1.
const LINK_TYP_SHIFT: u32 = 1;
/// `Typ = 01b`: the node is a queue head.
const TYP_QH: u32 = 1;
/// A link pointer's address bits.
const LINK_ADDR: u32 = !0x1f;

/// `Endpoint Characteristics` (QH dword 1): the device address, bits 6:0.
const EPCHAR_ADDR: u32 = 0x7f;
/// Inactivate on next transaction, bit 7.
const EPCHAR_I: u32 = 1 << 7;
/// Endpoint number, bits 11:8.
const EPCHAR_EP_SHIFT: u32 = 8;
/// Endpoint speed, bits 13:12. `10b` is high speed.
const EPCHAR_EPS_SHIFT: u32 = 12;
/// Data toggle control: take the toggle from the qTD rather than the QH.
const EPCHAR_DTC: u32 = 1 << 14;
/// Head of the reclamation list, bit 15.
const EPCHAR_H: u32 = 1 << 15;
/// Maximum packet length, bits 26:16.
const EPCHAR_MPS_SHIFT: u32 = 16;
/// …eleven bits of it.
const EPCHAR_MPS_MASK: u32 = 0x7ff;

/// qTD token: the transfer is still to be done.
const TOKEN_ACTIVE: u32 = 1 << 7;
/// The endpoint stalled, or an error retired the transfer.
const TOKEN_HALTED: u32 = 1 << 6;
/// The buffer ran out before the byte count did.
const TOKEN_DBE: u32 = 1 << 5;
/// The device sent more than `wMaxPacketSize`.
const TOKEN_BABBLE: u32 = 1 << 4;
/// A transaction error: no handshake, a timeout, a CRC failure.
const TOKEN_XACTERR: u32 = 1 << 3;
/// The status byte, bits 7:0.
const TOKEN_STATUS: u32 = 0xff;
/// PID code, bits 9:8: `00` OUT, `01` IN, `10` SETUP.
const TOKEN_PID_SHIFT: u32 = 8;
/// Interrupt on complete, bit 15.
const TOKEN_IOC: u32 = 1 << 15;
/// Current page, bits 14:12.
const TOKEN_CPAGE_SHIFT: u32 = 12;
/// Total bytes to transfer, bits 30:16.
const TOKEN_BYTES_SHIFT: u32 = 16;
/// …fifteen bits of it, so 20 480 bytes at most.
const TOKEN_BYTES_MASK: u32 = 0x7fff;
/// Data toggle, bit 31.
const TOKEN_TOGGLE: u32 = 1 << 31;

/// `PID Code = 00b`.
const PID_OUT: u32 = 0;
/// `PID Code = 01b`.
const PID_IN: u32 = 1;
/// `PID Code = 10b`.
const PID_SETUP: u32 = 2;

/// A buffer page is 4 KiB, and a qTD has five of them (§3.5.4).
const PAGE_SIZE: u32 = 4096;
/// …five.
const BUFFER_PAGES: usize = 5;

/// A queue head is twelve dwords (§3.6): three of its own, the current qTD
/// pointer, and an eight-dword transfer overlay.
const QH_DWORDS: usize = 12;
/// Where the overlay starts inside a queue head.
const QH_OVERLAY: usize = 4;
/// A qTD is eight dwords (§3.5).
const QTD_DWORDS: usize = 8;

// ---------------------------------------------------------------------------
// Bounds on a guest-controlled walk
// ---------------------------------------------------------------------------

/// How many queue heads one microframe's asynchronous traversal visits.
///
/// The async list is circular by construction, so "walk until the end" has no
/// end; a bound is the only correct way to write this loop.
pub const MAX_ASYNC_QH: usize = 64;

/// How many nodes one frame-list entry's chain is followed for.
pub const MAX_PERIODIC_NODES: usize = 64;

/// How many queue element descriptors one queue head retires per microframe.
///
/// Also what stops a qTD whose next pointer is itself: the fetch counts, so a
/// self-loop costs a bounded number of guest reads and then the microframe
/// ends.
pub const MAX_QTD_ADVANCE: usize = 32;

/// How many packets one transfer descriptor moves per microframe.
///
/// A qTD carries at most 20 480 bytes, which is 2 560 packets at an eight-byte
/// maximum packet size. Stopping short simply leaves the descriptor active for
/// the next microframe, which is both bounded and what the hardware does.
pub const MAX_PACKETS: usize = 1024;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// The parts of a controller a board gets to choose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Params {
    /// How many root ports. 1 to 15 — `HCSPARAMS.N_PORTS` is four bits.
    pub ports: u8,
    /// How many clock-domain ticks one 125 µs microframe takes.
    ///
    /// A property rather than a constant because it belongs to the *board's*
    /// clock tree: at the 60 MHz a USB 2.0 PHY runs at it is exactly 7500, and
    /// a machine file that puts the controller on some other domain has to say
    /// what the number is there. Never a float, never seconds
    /// (`CLAUDE.md`, determinism).
    pub microframe_ticks: u64,
    /// Where the operational registers start, as `CAPLENGTH` reports.
    pub caplength: u8,
    /// Whether this controller has a device role at all — `USBMODE` and the
    /// rest of [`crate::dev::usb::chipidea`]'s additions.
    ///
    /// `false` for a plain EHCI, which is a host and nothing else, and whose
    /// schedule therefore always runs.
    pub dual_role: bool,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            ports: 1,
            // 60 MHz / 8000 microframes a second. Exact, which is the point.
            microframe_ticks: 7500,
            caplength: DEFAULT_CAPLENGTH,
            dual_role: false,
        }
    }
}

/// `USBMODE.CM` (ChipIdea): the controller is idle.
pub const MODE_IDLE: u32 = 0;
/// `USBMODE.CM`: the controller is a device — the *guest* is the peripheral.
pub const MODE_DEVICE: u32 = 2;
/// `USBMODE.CM`: the controller is a host.
pub const MODE_HOST: u32 = 3;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Regs {
    /// Domain ticks simulated. The authoritative copy; an atomic mirrors it.
    ticks: u64,
    usbcmd: u32,
    usbsts: u32,
    usbintr: u32,
    frindex: u32,
    ctrldssegment: u32,
    periodic_base: u32,
    async_addr: u32,
    configflag: u32,
    portsc: [u32; MAX_PORTS],
    // The dual-role additions. Present in the struct always so the snapshot
    // encoding does not depend on a construction property — a saved state must
    // load into the class it was written by, whatever that class was built
    // with.
    usbmode: u32,
    otgsc: u32,
    burstsize: u32,
    txfilltuning: u32,
}

impl Regs {
    fn reset(ports: u8, dual_role: bool) -> Regs {
        let mut regs = Regs {
            ticks: 0,
            usbcmd: CMD_RESET_VALUE,
            usbsts: STS_HCHALTED,
            usbintr: 0,
            frindex: 0,
            ctrldssegment: 0,
            periodic_base: 0,
            async_addr: 0,
            configflag: 0,
            portsc: [0; MAX_PORTS],
            // A dual-role part comes up idle and the firmware chooses; a plain
            // EHCI has no choice to make.
            usbmode: if dual_role { MODE_IDLE } else { MODE_HOST },
            otgsc: 0,
            burstsize: 0x0000_1010,
            txfilltuning: 0,
        };
        for port in regs.portsc.iter_mut().take(usize::from(ports)) {
            // `HCSPARAMS.PPC` is zero, so the ports are always powered; and
            // `CONFIGFLAG` resets clear, which per §4.2 means every port
            // belongs to a companion controller until the driver claims it.
            *port = PORT_PP | PORT_OWNER;
        }
        regs
    }

    /// Whether the host schedule may run.
    fn running(&self, dual_role: bool) -> bool {
        self.usbcmd & CMD_RS != 0
            && self.usbsts & STS_HCHALTED == 0
            && (!dual_role || self.usbmode & 0x3 == MODE_HOST)
    }

    /// How many entries the periodic frame list has (§2.3.1, `USBCMD.FLS`).
    ///
    /// `HCCPARAMS.PFLF` is reported as one, so a driver may shrink the list —
    /// which some embedded drivers do to save memory, and which changes where
    /// `FRINDEX` wraps.
    fn frame_list_size(&self) -> u32 {
        match (self.usbcmd >> CMD_FLS_SHIFT) & 0x3 {
            0 => 1024,
            1 => 512,
            2 => 256,
            // `11b` is reserved; treating it as the default is the only
            // defined thing to do with a reserved encoding.
            _ => 1024,
        }
    }
}

/// A generic EHCI host controller: the register file and the schedule walker.
///
/// The **engine**, deliberately separate from any register map: an
/// [`EhciController`] is this plus the standard layout, and a
/// [`ChipIdea`](crate::dev::usb::chipidea::ChipIdea) is this plus a different
/// one. Everything below the register offsets is here, once.
pub struct Hcd {
    bus: Arc<UsbBus>,
    params: Params,
    regs: Mutex<Regs>,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `Regs::ticks`.
    ticks: AtomicU64,
    /// The tick the next microframe falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The space this controller masters.
    ///
    /// `Weak`, like every bus master's handle: the machine owns the space, and
    /// a device that kept its own space alive would close a cycle nothing could
    /// drop.
    space: Mutex<Option<Weak<AddressSpace>>>,
    requester: AtomicU32,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The level the interrupt output is being held at, so a debug read is
    /// free.
    irq_level: AtomicU32,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// "Nothing scheduled", as [`Hcd::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

impl fmt::Debug for Hcd {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Hcd");
        s.field("ports", &self.params.ports);
        match self.regs.try_lock() {
            Some(regs) => s.field("regs", &*regs).finish_non_exhaustive(),
            None => s.field("regs", &"<in use>").finish_non_exhaustive(),
        }
    }
}

/// What a register write asks for once the register lock is released.
///
/// The re-entrancy contract (`core::device`): decide under the lock, release,
/// *then* act outward. Every one of these is an outward action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// Nothing but the interrupt refresh every write ends with.
    Nothing,
    /// `USBCMD.HCRESET`: put everything back and start again.
    Reset,
    /// A port moved: reset a device, hand a port over, change an enable.
    Port(u8),
    /// `CONFIGFLAG` moved: every port changes hands at once.
    AllPorts,
}

impl Hcd {
    /// A controller on `bus`, configured by `params`.
    #[must_use]
    pub fn new(bus: Arc<UsbBus>, params: Params) -> Hcd {
        let params = Params {
            ports: params.ports.clamp(1, MAX_PORTS as u8),
            microframe_ticks: params.microframe_ticks.max(1),
            ..params
        };
        Hcd {
            bus,
            params,
            regs: Mutex::with_rank(HCD_RANK, Regs::reset(params.ports, params.dual_role)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            space: Mutex::with_rank(LockRank::WIRE, None),
            requester: AtomicU32::new(RequesterId::ANONYMOUS.0),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            irq_level: AtomicU32::new(0),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        }
    }

    /// How this controller was configured.
    #[must_use]
    pub fn params(&self) -> Params {
        self.params
    }

    /// The bus it drives.
    #[must_use]
    pub fn bus(&self) -> &Arc<UsbBus> {
        &self.bus
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// The level the interrupt output is being driven to.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.irq_level.load(Ordering::Relaxed) != 0)
    }

    /// `USBSTS`, for a test that wants to see what the guest would.
    #[must_use]
    pub fn status(&self) -> u32 {
        self.regs.lock().usbsts
    }

    /// `PORTSC` for port `port`, or zero for a port that does not exist.
    #[must_use]
    pub fn portsc(&self, port: u8) -> u32 {
        self.regs
            .lock()
            .portsc
            .get(usize::from(port))
            .copied()
            .unwrap_or(0)
    }

    /// Give the controller the address space its DMA traverses, and the
    /// identity its accesses carry.
    pub fn attach_space(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        *self.space.lock() = Some(Arc::downgrade(space));
        self.requester.store(requester.0, Ordering::Relaxed);
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.irq.lock() = Some(source);
        self.refresh_irq();
    }

    /// Told the handle that catches this device up.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.lazy.lock() = Some(handle);
    }

    /// The space this controller masters, if it still exists.
    fn space(&self) -> Option<Arc<AddressSpace>> {
        // Cloned out and the guard dropped before the caller does anything
        // with it: the space lock ranks *above* the fabric, so holding it
        // across a transaction would be a ladder violation as well as a
        // re-entrancy one.
        self.space.lock().as_ref().and_then(Weak::upgrade)
    }

    /// The attributes this controller's own accesses carry.
    fn attrs(&self) -> MemAttrs {
        MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)))
    }

    /// Bring the controller up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5). Public
    /// because a register map is allowed to live in another module — that is
    /// the whole point of the split — and every one of them has to do this
    /// first.
    pub fn sync_for(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up for this device is already running further
        // up the stack. The access still has to be answered.
        let _ = handle.sync(kind);
    }

    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, regs: &Regs) {
        self.ticks.store(regs.ticks, Ordering::Relaxed);
        let next = if regs.running(self.params.dual_role) {
            // Microframes land on multiples of the period, so they are a
            // property of the clock rather than of when the guest happened to
            // set RS — which is what a free-running PHY does, and what makes
            // this reproducible across a save and restore.
            let mf = self.params.microframe_ticks;
            (regs.ticks / mf + 1).saturating_mul(mf)
        } else {
            NO_EVENT
        };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// The tick the next microframe falls on, if the controller is running.
    #[must_use]
    pub fn next_event_tick(&self) -> Option<u64> {
        match self.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    /// Re-derive the interrupt output from `USBSTS` and `USBINTR` and drive it.
    ///
    /// Called with no lock of ours held (§2.3.2: the interrupt is the AND of
    /// the status and the enable, over the six write-1-to-clear bits).
    pub fn refresh_irq(&self) {
        let asserted = {
            let regs = self.regs.lock();
            regs.usbsts & regs.usbintr & STS_W1C != 0
        };
        self.irq_level.store(u32::from(asserted), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(Level::from_bool(asserted));
        }
    }

    // -----------------------------------------------------------------
    // The register file
    // -----------------------------------------------------------------

    /// Read a capability register (§2.2). Offsets 0x00, 0x04, 0x08, 0x0c.
    ///
    /// Read-only, side-effect free, and answerable without the schedule being
    /// caught up — which is why it takes no attributes.
    #[must_use]
    pub fn read_cap(&self, offset: u64) -> u32 {
        match offset {
            // CAPLENGTH in bits 7:0, HCIVERSION in bits 31:16 (§2.2.1-2.2.2).
            0x00 => u32::from(self.params.caplength) | (u32::from(HCIVERSION) << 16),
            // HCSPARAMS (§2.2.3). N_PORTS in bits 3:0; PPC clear, because the
            // ports are always powered; no companions, no port routing rules;
            // one port per companion is reported as zero of each.
            0x04 => u32::from(self.params.ports),
            // HCCPARAMS (§2.2.4). Bit 0 clear: 32-bit addressing only, so
            // CTRLDSSEGMENT is stored and ignored. Bit 1 set: the frame list
            // size is programmable. Bit 2 clear: no async schedule park. Bits
            // 15:8 zero: no extended capabilities, so no BIOS handoff.
            0x08 => 0x0000_0002,
            // HCSP-PORTROUTE (§2.2.5). Zero: no companion routing table.
            0x0c => 0,
            _ => 0,
        }
    }

    /// Read an operational register (§2.3), by offset from `CAPLENGTH`.
    ///
    /// Side-effect free at every offset — EHCI has no read-to-clear register —
    /// so `debug` changes nothing here. The debug rule bites on the *write*
    /// side and on catch-up, both of which the caller handles.
    #[must_use]
    pub fn read_op(&self, offset: u64) -> u32 {
        let regs = self.regs.lock();
        match offset {
            0x00 => regs.usbcmd,
            0x04 => regs.usbsts,
            0x08 => regs.usbintr,
            0x0c => regs.frindex,
            0x10 => regs.ctrldssegment,
            0x14 => regs.periodic_base,
            0x18 => regs.async_addr,
            0x40 => regs.configflag,
            _ => {
                if let Some(port) = Hcd::port_at(offset) {
                    return regs.portsc.get(usize::from(port)).copied().unwrap_or(0);
                }
                0
            }
        }
    }

    /// Which port `PORTSC` offset `offset` names, if it names one.
    ///
    /// Offsets are relative to the operational base, so this is the same
    /// answer whichever register map asked.
    #[must_use]
    pub fn port_at(offset: u64) -> Option<u8> {
        if !(0x44..0x44 + 4 * MAX_PORTS as u64).contains(&offset) || !offset.is_multiple_of(4) {
            return None;
        }
        Some(((offset - 0x44) / 4) as u8)
    }

    /// Write an operational register, reporting what has to happen once the
    /// lock is released.
    pub fn write_op(&self, offset: u64, value: u32) -> After {
        let mut regs = self.regs.lock();
        match offset {
            0x00 => {
                if value & CMD_HCRESET != 0 {
                    // Self-clearing, and it takes everything with it.
                    return After::Reset;
                }
                let was_running = regs.usbcmd & CMD_RS != 0;
                regs.usbcmd = value & CMD_MASK & !CMD_HCRESET;
                let running = regs.usbcmd & CMD_RS != 0;
                if running != was_running {
                    if running {
                        regs.usbsts &= !STS_HCHALTED;
                    } else {
                        // A real controller halts at the end of the current
                        // transaction. This one executes a whole transaction
                        // inside one microframe and never straddles a write,
                        // so "the end of the current transaction" is now.
                        regs.usbsts |= STS_HCHALTED;
                    }
                }
                Hcd::sync_schedule_status(&mut regs);
                self.publish(&regs);
                After::Nothing
            }
            0x04 => {
                // Write-1-to-clear, and *only* those bits. The read-only half
                // — `HCHalted`, `Reclamation`, and the two schedule-status
                // bits — is the controller's to set, and a write to it is
                // discarded rather than obeyed.
                regs.usbsts &= !(value & STS_W1C);
                After::Nothing
            }
            0x08 => {
                regs.usbintr = value & STS_W1C;
                After::Nothing
            }
            0x0c => {
                // §2.3.4: writable only while the controller is halted. A
                // write while it runs is ignored rather than faulted — the
                // register is there, the guest's write is simply lost, which
                // is what the hardware does.
                if regs.usbsts & STS_HCHALTED != 0 {
                    regs.frindex = value & 0x3fff;
                    self.publish(&regs);
                }
                After::Nothing
            }
            0x10 => {
                regs.ctrldssegment = value;
                After::Nothing
            }
            // §2.3.6: 4 KiB aligned. §2.3.7: 32-byte aligned.
            0x14 => {
                regs.periodic_base = value & !0xfff;
                After::Nothing
            }
            0x18 => {
                regs.async_addr = value & LINK_ADDR;
                After::Nothing
            }
            0x40 => {
                let was = regs.configflag & 1;
                regs.configflag = value & 1;
                if regs.configflag != was {
                    // §4.2.2: `CONFIGFLAG` is how the driver takes the ports
                    // away from the companion controllers, and gives them back.
                    // Nothing else clears `Port Owner` wholesale — software may
                    // still release one port at a time by writing the bit, and
                    // `settle_port` re-releases anything this controller cannot
                    // drive.
                    let configured = regs.configflag != 0;
                    for port in regs.portsc.iter_mut().take(usize::from(self.params.ports)) {
                        if configured {
                            *port &= !PORT_OWNER;
                        } else {
                            *port = (*port | PORT_OWNER) & !PORT_PE;
                        }
                    }
                }
                After::AllPorts
            }
            _ => {
                let Some(port) = Hcd::port_at(offset) else {
                    return After::Nothing;
                };
                if usize::from(port) >= usize::from(self.params.ports) {
                    return After::Nothing;
                }
                let index = usize::from(port);
                let old = regs.portsc[index];
                // Write-1-to-clear the change bits, then take the writable
                // ones. Everything else keeps the controller's value.
                let mut new = old & !(value & PORT_W1C);
                new = (new & !PORT_WRITABLE) | (value & PORT_WRITABLE);
                // Overcurrent is never asserted here.
                new &= !PORT_OCA;
                regs.portsc[index] = new;
                After::Port(port)
            }
        }
    }

    /// Whether the periodic and asynchronous schedules are reported as running.
    ///
    /// §2.3.2 lets the controller take a while over these, and software polls
    /// them before it touches either list. Following `USBCMD` immediately is
    /// legal and is what makes both directions of that poll terminate.
    fn sync_schedule_status(regs: &mut Regs) {
        let halted = regs.usbsts & STS_HCHALTED != 0;
        regs.usbsts &= !(STS_PSS | STS_ASS);
        if !halted {
            if regs.usbcmd & CMD_PSE != 0 {
                regs.usbsts |= STS_PSS;
            }
            if regs.usbcmd & CMD_ASE != 0 {
                regs.usbsts |= STS_ASS;
            }
        }
    }

    /// Read a dual-role register: `USBMODE`, `OTGSC` and the tuning knobs.
    ///
    /// Here rather than in [`crate::dev::usb::chipidea`] because they are
    /// *state*, and state belongs with the engine that snapshots it; what the
    /// variant owns is where in its aperture they appear.
    #[must_use]
    pub fn read_extra(&self, which: Extra) -> u32 {
        let regs = self.regs.lock();
        match which {
            Extra::UsbMode => regs.usbmode,
            Extra::Otgsc => regs.otgsc,
            Extra::BurstSize => regs.burstsize,
            Extra::TxFillTuning => regs.txfilltuning,
        }
    }

    /// Write a dual-role register.
    pub fn write_extra(&self, which: Extra, value: u32) -> After {
        let mut regs = self.regs.lock();
        match which {
            Extra::UsbMode => {
                // §USBMODE: CM is write-once after a reset on real ChipIdea
                // parts. Modelled as write-once, because firmware writes it
                // once and a driver that wrote it twice would be relying on
                // behaviour the part does not have.
                if regs.usbmode & 0x3 == MODE_IDLE {
                    regs.usbmode = value & 0x1f;
                } else {
                    regs.usbmode = (regs.usbmode & 0x3) | (value & 0x1c);
                }
                Hcd::sync_schedule_status(&mut regs);
                self.publish(&regs);
            }
            Extra::Otgsc => {
                // The status and interrupt-status halves are write-1-to-clear;
                // the control and interrupt-enable halves are read/write.
                // Nothing here generates an OTG event, so this is storage that
                // reads back — said plainly rather than pretending otherwise.
                regs.otgsc = value & 0x7f7f_7f7f;
            }
            Extra::BurstSize => regs.burstsize = value & 0x0000_ffff,
            Extra::TxFillTuning => regs.txfilltuning = value & 0x003f_3fff,
        }
        After::Nothing
    }

    /// Perform whatever a register write asked for, with no lock held.
    pub fn act(&self, after: After) {
        match after {
            After::Nothing => {}
            After::Reset => self.controller_reset(),
            After::Port(port) => self.settle_port(port),
            After::AllPorts => {
                for port in 0..self.params.ports {
                    self.settle_port(port);
                }
            }
        }
        self.refresh_irq();
    }

    /// `USBCMD.HCRESET` (§2.3.1): everything back to its reset value, the
    /// schedules stopped, the ports disabled.
    fn controller_reset(&self) {
        {
            let mut regs = self.regs.lock();
            let ticks = regs.ticks;
            *regs = Regs {
                ticks,
                // A reset does not un-choose the role: on a dual-role part
                // `USBMODE` survives `HCRESET`, which is why firmware writes it
                // first and resets afterwards.
                usbmode: regs.usbmode,
                ..Regs::reset(self.params.ports, self.params.dual_role)
            };
            self.publish(&regs);
        }
        for port in 0..self.params.ports {
            self.bus.set_enabled(port, false);
        }
        for port in 0..self.params.ports {
            self.settle_port(port);
        }
    }

    /// Bring one port's `PORTSC`, the fabric and the device behind it into
    /// agreement.
    ///
    /// The one place the "EHCI is high-speed only" rule is enforced, and the
    /// place a port changes hands.
    fn settle_port(&self, port: u8) {
        let index = usize::from(port);
        if index >= usize::from(self.params.ports) {
            return;
        }

        // What the port looks like from the fabric's side. Read outside our
        // own lock: `speed` is a call into the device.
        let connected = self.bus.connected(port);
        let speed = self.bus.speed(port);
        let plugged_changed = self.bus.take_change(port);

        // Decide under the lock, act outside it. There is no `Enable` here on
        // purpose: the only thing that ever enables a port is the end of a
        // reset ([`Hcd::finish_port_reset`]), which is also where the port may
        // change hands instead.
        enum Act {
            Nothing,
            Disable,
        }
        let act = {
            let mut regs = self.regs.lock();
            let mut sc = regs.portsc[index];
            let configured = regs.configflag & 1 != 0;

            if plugged_changed {
                sc |= PORT_CSC;
                regs.usbsts |= STS_PORT_CHANGE;
            }
            sc = (sc & !PORT_CCS) | if connected { PORT_CCS } else { 0 };
            sc |= PORT_PP;

            // §4.2.2: with `CONFIGFLAG` clear every port belongs to the
            // companion controllers, and the driver sets it once it is ready to
            // own them.
            if !configured {
                sc |= PORT_OWNER;
            }

            // A low-speed device is detected *before* any reset, from the line
            // state, and released immediately (§4.2.2). Line status `01b` is
            // the K state that says so, and it is worth putting in the register
            // because a driver reads it to make the same decision.
            sc &= !(0x3 << PORT_LS_SHIFT);
            if connected && speed == Some(Speed::Low) {
                sc |= 0x1 << PORT_LS_SHIFT;
                if configured {
                    sc |= PORT_OWNER;
                }
            }

            let mut act = Act::Nothing;
            if !connected {
                if sc & PORT_PE != 0 {
                    sc |= PORT_PEC;
                }
                sc &= !(PORT_PE | PORT_RESET | PORT_SUSPEND);
                act = Act::Disable;
            } else if sc & PORT_RESET != 0 {
                // Reset is in progress: the port is disabled while it is
                // asserted, and the device sees the reset when software
                // releases it.
                sc &= !PORT_PE;
                act = Act::Disable;
            } else if sc & PORT_OWNER != 0 {
                sc &= !PORT_PE;
                act = Act::Disable;
            }
            regs.portsc[index] = sc;
            act
        };

        match act {
            Act::Nothing => {}
            Act::Disable => self.bus.set_enabled(port, false),
        }
    }

    /// Software released `PORT_RESET`: drive the reset and decide who keeps the
    /// port.
    ///
    /// Called from the register write path, outside the lock — by whichever
    /// register map noticed the release, which is why it is public.
    pub fn finish_reset(&self, port: u8) {
        let index = usize::from(port);
        self.bus.reset_port(port);
        let speed = self.bus.speed(port);
        let keep = {
            let mut regs = self.regs.lock();
            let mut sc = regs.portsc[index];
            sc &= !PORT_RESET;
            let keep = match speed {
                // A high-speed device is ours.
                Some(Speed::High) => {
                    if sc & PORT_PE == 0 {
                        sc |= PORT_PEC;
                    }
                    sc |= PORT_PE;
                    true
                }
                // §4.2.2: anything else is handed to a companion controller.
                // The port stops being ours — connect status and all — which
                // is why a full-speed device attached to a bare EHCI *vanishes*
                // rather than half-working. There is no companion in this tree,
                // so the honest outcome is that the device is unreachable, and
                // that is exactly what a board with one EHCI and no OHCI does.
                Some(_) => {
                    sc &= !(PORT_PE | PORT_CCS);
                    sc |= PORT_OWNER | PORT_CSC;
                    regs.usbsts |= STS_PORT_CHANGE;
                    false
                }
                None => {
                    sc &= !PORT_PE;
                    false
                }
            };
            regs.portsc[index] = sc;
            keep
        };
        self.bus.set_enabled(port, keep);
    }

    // -----------------------------------------------------------------
    // Time and the schedule
    // -----------------------------------------------------------------

    /// Simulate forward to `target` domain ticks.
    ///
    /// Runs with **no lock held across an outward call**: each microframe
    /// decides what to do under the register lock, releases it, then walks the
    /// schedule — which reaches guest memory and the USB fabric
    /// (`core::device`, the re-entrancy contract).
    pub fn advance_to(&self, target: u64) {
        loop {
            {
                let mut regs = self.regs.lock();
                if !regs.running(self.params.dual_role) {
                    regs.ticks = regs.ticks.max(target);
                    self.publish(&regs);
                    return;
                }
                let mf = self.params.microframe_ticks;
                let next = (regs.ticks / mf + 1).saturating_mul(mf);
                if next > target {
                    regs.ticks = regs.ticks.max(target);
                    self.publish(&regs);
                    return;
                }
                regs.ticks = next;
                self.publish(&regs);
            }
            self.microframe();
        }
    }

    /// One 125 µs microframe.
    fn microframe(&self) {
        // Ports first: a device plugged in between microframes has to reach
        // `PORTSC` before anything else looks at the bus.
        if self.bus.any_change() {
            for port in 0..self.params.ports {
                self.settle_port(port);
            }
        }

        let (frindex, periodic, asynchronous, periodic_base, async_addr, doorbell) = {
            let mut regs = self.regs.lock();
            let size = regs.frame_list_size();
            let next = (regs.frindex + 1) & 0x3fff;
            // The frame list index is `FRINDEX[N:3]`, so it wraps every
            // `size * 8` microframes (§2.3.4).
            if (next / 8).is_multiple_of(size) && (regs.frindex / 8) % size == size - 1 {
                regs.usbsts |= STS_FLR;
            }
            regs.frindex = next;
            Hcd::sync_schedule_status(&mut regs);
            (
                regs.frindex,
                regs.usbcmd & CMD_PSE != 0,
                regs.usbcmd & CMD_ASE != 0,
                regs.periodic_base,
                regs.async_addr,
                regs.usbcmd & CMD_IAAD != 0,
            )
        };

        let Some(space) = self.space() else {
            // No space: nothing to walk. A machine that gave this controller
            // no `space =` is refused at bind time, so this is only reachable
            // from a test holding the engine directly.
            self.refresh_irq();
            return;
        };

        if periodic && periodic_base != 0 {
            self.walk_periodic(&space, frindex, periodic_base);
        }
        if asynchronous && async_addr != 0 {
            self.walk_async(&space, async_addr);
        }
        if doorbell && asynchronous {
            // §4.8.2: the doorbell is answered once the controller has been
            // all the way round the async list, which it just has.
            let mut regs = self.regs.lock();
            regs.usbcmd &= !CMD_IAAD;
            regs.usbsts |= STS_IAA;
        }
        self.refresh_irq();
    }

    /// One frame-list entry's chain, for the microframe `frindex` names.
    fn walk_periodic(&self, space: &AddressSpace, frindex: u32, base: u32) {
        let size = {
            let regs = self.regs.lock();
            regs.frame_list_size()
        };
        let index = (frindex / 8) % size;
        let microframe = frindex % 8;
        let Some(mut link) = self.read32(space, base.wrapping_add(index * 4)) else {
            self.host_system_error();
            return;
        };

        for _ in 0..MAX_PERIODIC_NODES {
            if link & LINK_T != 0 {
                return;
            }
            let node = link & LINK_ADDR;
            if node == 0 {
                return;
            }
            let typ = (link >> LINK_TYP_SHIFT) & 0x3;
            let next = match self.read32(space, node) {
                Some(next) => next,
                None => {
                    self.host_system_error();
                    return;
                }
            };
            if typ == TYP_QH {
                // §3.6.3: the S-mask says which of the eight microframes this
                // endpoint is serviced in.
                let smask = match self.read32(space, node.wrapping_add(8)) {
                    Some(caps) => caps & 0xff,
                    None => {
                        self.host_system_error();
                        return;
                    }
                };
                if smask & (1 << microframe) != 0 {
                    self.execute_qh(space, node);
                }
            }
            // Isochronous nodes are followed and not executed — see the module
            // docs. Their link pointer is dword 0 like a queue head's, which is
            // why walking past them costs nothing.
            link = next;
        }
    }

    /// The asynchronous list, once round.
    fn walk_async(&self, space: &AddressSpace, head: u32) {
        let mut node = head & LINK_ADDR;
        for step in 0..MAX_ASYNC_QH {
            if node == 0 {
                return;
            }
            self.execute_qh(space, node);
            let Some(link) = self.read32(space, node) else {
                self.host_system_error();
                return;
            };
            if link & LINK_T != 0 {
                return;
            }
            let next = link & LINK_ADDR;
            // The list is circular by construction, so returning to where we
            // started is the *end* of a traversal rather than an error.
            if next == head & LINK_ADDR {
                return;
            }
            node = next;
            let _ = step;
        }
        // Ran out of budget. The rest of the list is walked next microframe,
        // which is what a controller with a finite microframe does anyway.
    }

    /// Execute whatever one queue head has outstanding.
    fn execute_qh(&self, space: &AddressSpace, addr: u32) {
        let mut qh = [0u32; QH_DWORDS];
        for (i, slot) in qh.iter_mut().enumerate() {
            let Some(value) = self.read32(space, addr.wrapping_add((i * 4) as u32)) else {
                self.host_system_error();
                return;
            };
            *slot = value;
        }

        let epchar = qh[1];
        let device = DeviceAddress((epchar & EPCHAR_ADDR) as u8);
        let endpoint = ((epchar >> EPCHAR_EP_SHIFT) & 0xf) as u8;
        // A queue head whose maximum packet size is zero is a driver bug; one
        // packet of one byte is the only forward progress available, and it is
        // better than dividing by zero.
        let mps = ((epchar >> EPCHAR_MPS_SHIFT) & EPCHAR_MPS_MASK).max(1);
        let dtc = epchar & EPCHAR_DTC != 0;
        // `H` (head of the reclamation list) and `EPS` (endpoint speed) are
        // read and not acted on: the first drives `USBSTS.Reclamation`, which
        // this controller reads zero, and the second only matters when there is
        // a transaction translator to route through — and there is no hub.
        let _ = (epchar & EPCHAR_H, (epchar >> EPCHAR_EPS_SHIFT) & 0x3);

        let mut dirty = false;
        let mut advanced = 0usize;
        loop {
            // §4.10.8: a halted queue head stops dead. The controller does not
            // advance past the descriptor that halted it, and nothing moves on
            // this endpoint until software clears the condition — which is
            // exactly what makes a `STALL` visible to a driver instead of being
            // stepped over on the next microframe.
            if qh[QH_OVERLAY + 2] & TOKEN_HALTED != 0 {
                break;
            }
            if qh[QH_OVERLAY + 2] & TOKEN_ACTIVE == 0 {
                if advanced >= MAX_QTD_ADVANCE {
                    break;
                }
                // §3.6.2: `I` asks the controller to stop here when the queue
                // is not already mid-transfer.
                if epchar & EPCHAR_I != 0 {
                    break;
                }
                // §4.10.2: advance the queue through the overlay's Next qTD
                // Pointer. A queue head the driver has only just built may
                // instead carry the first descriptor in `CurrentqTD`, so that
                // is tried once, which costs one guest read and accepts both
                // conventions.
                let candidate = if qh[QH_OVERLAY] & LINK_T == 0 {
                    qh[QH_OVERLAY] & LINK_ADDR
                } else if advanced == 0 {
                    qh[3] & LINK_ADDR
                } else {
                    0
                };
                if candidate == 0 {
                    break;
                }
                let mut qtd = [0u32; QTD_DWORDS];
                let mut faulted = false;
                for (i, slot) in qtd.iter_mut().enumerate() {
                    match self.read32(space, candidate.wrapping_add((i * 4) as u32)) {
                        Some(value) => *slot = value,
                        None => {
                            faulted = true;
                            break;
                        }
                    }
                }
                if faulted {
                    self.host_system_error();
                    break;
                }
                if qtd[2] & TOKEN_ACTIVE == 0 {
                    break;
                }
                // §3.6.3: with `DTC` clear the toggle lives in the queue head
                // and survives the descriptor it came from.
                let toggle = if dtc {
                    qtd[2] & TOKEN_TOGGLE
                } else {
                    qh[QH_OVERLAY + 2] & TOKEN_TOGGLE
                };
                qh[3] = candidate;
                qh[QH_OVERLAY] = qtd[0];
                qh[QH_OVERLAY + 1] = qtd[1];
                qh[QH_OVERLAY + 2] = (qtd[2] & !TOKEN_TOGGLE) | toggle;
                qh[QH_OVERLAY + 3..QH_DWORDS].copy_from_slice(&qtd[3..QTD_DWORDS]);
                advanced += 1;
            }

            // The overlay always moves from here on, whatever the outcome, so
            // the queue head has to be written back.
            let outcome = self.run_transfer(space, &mut qh, device, endpoint, mps);
            dirty = true;
            if qh[QH_OVERLAY + 2] & TOKEN_ACTIVE != 0 {
                // A `NAK`, or the packet budget ran out. Either way the
                // descriptor stays active and this queue head is done for the
                // microframe.
                break;
            }
            // Retired. §4.10.8: the overlay's status and buffer offset are
            // written back into the descriptor the driver owns.
            let current = qh[3] & LINK_ADDR;
            if current != 0 {
                let _ = self.write32(space, current.wrapping_add(8), qh[QH_OVERLAY + 2]);
                let _ = self.write32(space, current.wrapping_add(12), qh[QH_OVERLAY + 3]);
            }
            if outcome.halted {
                break;
            }
            if outcome.short_packet && qh[QH_OVERLAY + 1] & LINK_T == 0 {
                // §4.10.6: a short packet on an `IN` sends the queue to the
                // alternate descriptor, which is how a driver gets told the
                // transfer ended early without losing the rest of the chain.
                qh[QH_OVERLAY] = qh[QH_OVERLAY + 1];
            }
        }

        if dirty {
            for (i, value) in qh.iter().enumerate().skip(3) {
                if self
                    .write32(space, addr.wrapping_add((i * 4) as u32), *value)
                    .is_none()
                {
                    self.host_system_error();
                    return;
                }
            }
        }
    }

    /// Move as much of the overlay's transfer as one microframe allows.
    fn run_transfer(
        &self,
        space: &AddressSpace,
        qh: &mut [u32; QH_DWORDS],
        device: DeviceAddress,
        endpoint: u8,
        mps: u32,
    ) -> Outcome {
        let mut token = qh[QH_OVERLAY + 2];
        let pid = (token >> TOKEN_PID_SHIFT) & 0x3;
        let mut total = (token >> TOKEN_BYTES_SHIFT) & TOKEN_BYTES_MASK;
        let mut page = ((token >> TOKEN_CPAGE_SHIFT) & 0x7) as usize;
        let mut offset = qh[QH_OVERLAY + 3] & (PAGE_SIZE - 1);
        let mut toggle = token & TOKEN_TOGGLE != 0;
        let ioc = token & TOKEN_IOC != 0;

        let mut outcome = Outcome::default();
        let mut status = 0u32;
        let mut retire = false;

        for packet in 0..MAX_PACKETS {
            // A descriptor whose byte count is zero still describes **one**
            // transaction — a zero-length packet — and that is what every
            // control transfer's status stage is (USB 2.0 §8.5.3). Retiring it
            // without going to the bus would mean `SET_ADDRESS` never took
            // effect, because the device applies it when the status stage
            // completes and nothing would have told it that one had.
            if total == 0 && packet > 0 {
                retire = true;
                break;
            }
            let want = mps.min(total);
            let completion = match pid {
                PID_SETUP => {
                    // A setup packet is eight bytes, always (USB 2.0 §9.3). A
                    // descriptor claiming fewer is malformed, and inventing the
                    // missing bytes would hand the device a request nobody
                    // wrote.
                    if total < SetupPacket::SIZE as u32 {
                        status |= TOKEN_XACTERR | TOKEN_HALTED;
                        retire = true;
                        break;
                    }
                    let mut raw = [0u8; 8];
                    let n = 8usize;
                    let Some(_) = self.buffer_read(space, qh, page, offset, &mut raw[..n]) else {
                        status |= TOKEN_DBE | TOKEN_HALTED;
                        retire = true;
                        break;
                    };
                    let status_code = self.bus.setup(device, endpoint, SetupPacket::decode(&raw));
                    // §8.6.1: the data stage after a `SETUP` is always `DATA1`.
                    if status_code == Status::Ack {
                        toggle = true;
                    }
                    Completion {
                        status: status_code,
                        len: n as u64,
                    }
                }
                PID_IN => {
                    let mut buf = alloc::vec![0u8; want as usize];
                    let completion = self.bus.read(device, endpoint, &mut buf);
                    if completion.status == Status::Ack {
                        let n = (completion.len as u32).min(want) as usize;
                        if self
                            .buffer_write(space, qh, page, offset, &buf[..n])
                            .is_none()
                        {
                            status |= TOKEN_DBE | TOKEN_HALTED;
                            retire = true;
                            break;
                        }
                        if (n as u32) < want {
                            outcome.short_packet = true;
                        }
                        Completion::ack(n as u64)
                    } else {
                        completion
                    }
                }
                PID_OUT => {
                    let mut buf = alloc::vec![0u8; want as usize];
                    if self
                        .buffer_read(space, qh, page, offset, &mut buf)
                        .is_none()
                    {
                        status |= TOKEN_DBE | TOKEN_HALTED;
                        retire = true;
                        break;
                    }
                    self.bus.write(device, endpoint, &buf)
                }
                // `11b` is reserved. A descriptor carrying it is malformed and
                // the transfer is retired with an error rather than guessed at.
                _ => {
                    status |= TOKEN_XACTERR | TOKEN_HALTED;
                    retire = true;
                    break;
                }
            };

            match completion.status {
                Status::Ack => {
                    let moved = (completion.len as u32).min(total);
                    let Some((next_page, next_offset)) = Hcd::advance(page, offset, moved) else {
                        status |= TOKEN_DBE | TOKEN_HALTED;
                        retire = true;
                        break;
                    };
                    page = next_page;
                    offset = next_offset;
                    total -= moved;
                    if pid != PID_SETUP {
                        toggle = !toggle;
                    }
                    if outcome.short_packet || total == 0 {
                        retire = true;
                        break;
                    }
                }
                Status::Nak => {
                    // Not an error. The descriptor stays active and the host
                    // tries again next microframe (§8.4.5).
                    break;
                }
                Status::Stall => {
                    status |= TOKEN_HALTED;
                    retire = true;
                    break;
                }
                Status::Babble => {
                    status |= TOKEN_BABBLE | TOKEN_HALTED;
                    retire = true;
                    break;
                }
                Status::NoDevice | Status::Error => {
                    status |= TOKEN_XACTERR | TOKEN_HALTED;
                    retire = true;
                    break;
                }
            }
        }

        outcome.halted = status & TOKEN_HALTED != 0;

        token &= !(TOKEN_STATUS
            | (TOKEN_BYTES_MASK << TOKEN_BYTES_SHIFT)
            | (0x7 << TOKEN_CPAGE_SHIFT)
            | TOKEN_TOGGLE);
        token |= status;
        token |= (total & TOKEN_BYTES_MASK) << TOKEN_BYTES_SHIFT;
        token |= ((page as u32) & 0x7) << TOKEN_CPAGE_SHIFT;
        if toggle {
            token |= TOKEN_TOGGLE;
        }
        if !retire {
            token |= TOKEN_ACTIVE;
        }
        qh[QH_OVERLAY + 2] = token;
        // §3.5.4: the running offset lives in the low twelve bits of buffer
        // pointer zero, whichever page is current.
        qh[QH_OVERLAY + 3] = (qh[QH_OVERLAY + 3] & !(PAGE_SIZE - 1)) | (offset & (PAGE_SIZE - 1));

        if retire {
            let mut regs = self.regs.lock();
            if ioc {
                regs.usbsts |= STS_USBINT;
            }
            if outcome.halted {
                regs.usbsts |= STS_USBERRINT;
            }
        }
        outcome
    }

    /// Where `moved` bytes on from `(page, offset)` lands, or `None` past the
    /// end of the five buffer pages.
    fn advance(page: usize, offset: u32, moved: u32) -> Option<(usize, u32)> {
        let mut page = page;
        let mut offset = offset.checked_add(moved)?;
        while offset >= PAGE_SIZE {
            offset -= PAGE_SIZE;
            page += 1;
            if page >= BUFFER_PAGES {
                // Landing exactly on the end is legal; needing a byte past it
                // is the data buffer error §3.5.4 describes.
                return (offset == 0 && page == BUFFER_PAGES)
                    .then_some((BUFFER_PAGES - 1, PAGE_SIZE));
            }
        }
        Some((page, offset))
    }

    /// The guest address of `(page, offset)` in a qTD's buffer list.
    fn buffer_address(qh: &[u32; QH_DWORDS], page: usize, offset: u32) -> Option<u32> {
        if page >= BUFFER_PAGES || offset >= PAGE_SIZE {
            return None;
        }
        Some((qh[QH_OVERLAY + 3 + page] & !(PAGE_SIZE - 1)).wrapping_add(offset))
    }

    /// Copy `dst.len()` bytes out of the qTD's buffer, crossing pages as
    /// needed. `None` if the buffer runs out first.
    fn buffer_read(
        &self,
        space: &AddressSpace,
        qh: &[u32; QH_DWORDS],
        page: usize,
        offset: u32,
        dst: &mut [u8],
    ) -> Option<()> {
        let mut page = page;
        let mut offset = offset;
        let mut done = 0usize;
        while done < dst.len() {
            let addr = Hcd::buffer_address(qh, page, offset)?;
            let room = (PAGE_SIZE - offset) as usize;
            let n = room.min(dst.len() - done);
            space
                .read_bytes(u64::from(addr), &mut dst[done..done + n], self.attrs())
                .ok()?;
            done += n;
            let (next_page, next_offset) = Hcd::advance(page, offset, n as u32)?;
            page = next_page;
            offset = next_offset;
            if offset == PAGE_SIZE {
                // The end of the last page. Anything more is out of buffer.
                return (done == dst.len()).then_some(());
            }
        }
        Some(())
    }

    /// Copy `src` into the qTD's buffer, crossing pages as needed.
    fn buffer_write(
        &self,
        space: &AddressSpace,
        qh: &[u32; QH_DWORDS],
        page: usize,
        offset: u32,
        src: &[u8],
    ) -> Option<()> {
        let mut page = page;
        let mut offset = offset;
        let mut done = 0usize;
        while done < src.len() {
            let addr = Hcd::buffer_address(qh, page, offset)?;
            let room = (PAGE_SIZE - offset) as usize;
            let n = room.min(src.len() - done);
            space
                .write_bytes(u64::from(addr), &src[done..done + n], self.attrs())
                .ok()?;
            done += n;
            let (next_page, next_offset) = Hcd::advance(page, offset, n as u32)?;
            page = next_page;
            offset = next_offset;
            if offset == PAGE_SIZE {
                return (done == src.len()).then_some(());
            }
        }
        Some(())
    }

    /// One dword of guest memory, or `None` for a bus fault.
    fn read32(&self, space: &AddressSpace, addr: u32) -> Option<u32> {
        space
            .read(u64::from(addr), Width::U32, self.attrs())
            .ok()
            .map(|value| value as u32)
    }

    /// Store one dword of guest memory, or `None` for a bus fault.
    fn write32(&self, space: &AddressSpace, addr: u32, value: u32) -> Option<()> {
        space
            .write(u64::from(addr), Width::U32, u64::from(value), self.attrs())
            .ok()
    }

    /// A DMA access faulted (§2.3.2, `Host System Error`).
    ///
    /// The controller stops: continuing to walk a list it cannot read would
    /// invent transfers out of bus faults.
    fn host_system_error(&self) {
        let mut regs = self.regs.lock();
        regs.usbsts |= STS_HSE | STS_HCHALTED;
        regs.usbcmd &= !CMD_RS;
        Hcd::sync_schedule_status(&mut regs);
        self.publish(&regs);
    }

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    /// Return to the documented reset state.
    pub fn reset(&self, _kind: ResetKind) {
        {
            let mut regs = self.regs.lock();
            let ticks = regs.ticks;
            *regs = Regs {
                ticks,
                ..Regs::reset(self.params.ports, self.params.dual_role)
            };
            self.publish(&regs);
        }
        for port in 0..self.params.ports {
            self.bus.set_enabled(port, false);
        }
        for port in 0..self.params.ports {
            self.settle_port(port);
        }
        self.refresh_irq();
    }

    /// Serialize the register file.
    ///
    /// # What a snapshot mid-transfer means
    ///
    /// Nothing special, and that is by construction. A transaction is executed
    /// to completion inside one microframe, and everything that outlives it —
    /// the queue head, the overlay, the descriptor's token — lives in **guest
    /// memory**, which the RAM device saves. The controller's own durable
    /// state is exactly the registers below, so a snapshot is never taken with
    /// half a transaction in flight and there is no hidden walker position to
    /// restore.
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let regs = *self.regs.lock();
        w.write_u64(regs.ticks)?;
        w.write_u32(regs.usbcmd)?;
        w.write_u32(regs.usbsts)?;
        w.write_u32(regs.usbintr)?;
        w.write_u32(regs.frindex)?;
        w.write_u32(regs.ctrldssegment)?;
        w.write_u32(regs.periodic_base)?;
        w.write_u32(regs.async_addr)?;
        w.write_u32(regs.configflag)?;
        w.write_u32(regs.usbmode)?;
        w.write_u32(regs.otgsc)?;
        w.write_u32(regs.burstsize)?;
        w.write_u32(regs.txfilltuning)?;
        w.write_seq_len(MAX_PORTS as u64)?;
        for port in regs.portsc {
            w.write_u32(port)?;
        }
        Ok(())
    }

    /// Restore what [`save`](Hcd::save) wrote.
    ///
    /// # Errors
    ///
    /// [`Error::State`] for a truncated or malformed chunk.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let mut regs = Regs {
            ticks: r.read_u64()?,
            usbcmd: r.read_u32()?,
            usbsts: r.read_u32()?,
            usbintr: r.read_u32()?,
            frindex: r.read_u32()?,
            ctrldssegment: r.read_u32()?,
            periodic_base: r.read_u32()?,
            async_addr: r.read_u32()?,
            configflag: r.read_u32()?,
            usbmode: r.read_u32()?,
            otgsc: r.read_u32()?,
            burstsize: r.read_u32()?,
            txfilltuning: r.read_u32()?,
            portsc: [0; MAX_PORTS],
        };
        let count = r.read_seq_len(4)?;
        if count != MAX_PORTS as u64 {
            return Err(Error::State(alloc::format!(
                "usb.ehci: a snapshot with {count} ports, not {MAX_PORTS}"
            )));
        }
        for port in &mut regs.portsc {
            *port = r.read_u32()?;
        }
        {
            let mut slot = self.regs.lock();
            *slot = regs;
            self.publish(&slot);
        }
        // The fabric's enable bits are derived state and are never serialized
        // (`ROADMAP.md` §4.5): they are re-derived from `PORTSC` here.
        for port in 0..self.params.ports {
            let enabled = self.portsc(port) & (PORT_PE | PORT_OWNER) == PORT_PE;
            self.bus.set_enabled(port, enabled);
        }
        self.refresh_irq();
        Ok(())
    }
}

/// Which of the dual-role registers a variant is asking for.
///
/// An enum rather than an offset, because the offsets are the *variant's* and
/// the state is the engine's. See [`Hcd::read_extra`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Extra {
    /// `USBMODE`: the controller role.
    UsbMode,
    /// `OTGSC`: on-the-go status and control.
    Otgsc,
    /// `BURSTSIZE`: the AHB burst length the DMA engine uses.
    BurstSize,
    /// `TXFILLTUNING`: how far ahead the transmit FIFO is filled.
    TxFillTuning,
}

/// What executing one transfer descriptor did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Outcome {
    /// An `IN` returned fewer bytes than the packet size.
    short_packet: bool,
    /// The descriptor retired with its `Halted` bit set.
    halted: bool,
}

// ---------------------------------------------------------------------------
// The standard-layout device
// ---------------------------------------------------------------------------

/// A generic EHCI host controller as a machine object.
///
/// The register block is EHCI's own layout: capability registers at zero,
/// operational registers at `CAPLENGTH`. A SoC that puts them somewhere else
/// wraps [`Hcd`] instead — see [`crate::dev::usb::chipidea`], which is what
/// that looks like.
#[derive(Debug)]
pub struct EhciController {
    hcd: Arc<Hcd>,
    region: RegionRef,
}

impl EhciController {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this controller is the root of.
    ///   Required.
    /// * `ports` — how many root ports, 1 to 15. Defaults to 1.
    /// * `microframe` — how many clock-domain ticks one 125 µs microframe
    ///   takes. Defaults to 7500, which is the number at the 60 MHz a USB 2.0
    ///   PHY runs at.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for a port count or a microframe outside its range.
    pub fn new(props: &Props) -> Result<EhciController> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let ports = r.or_range("ports", 1u64, 1..=MAX_PORTS as u64)?;
        let microframe = r.or_range("microframe", 7500u64, 1..=u64::from(u32::MAX))?;
        r.finish()?;

        let bus = buses::open(&bus_name, ports as u8);
        if bus.port_count() < ports as u8 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "the USB bus `{bus_name}` already has {} ports and this controller asked for \
                     {ports}; the first object to name a bus fixes its size",
                    bus.port_count()
                ),
            });
        }
        Ok(EhciController::with_bus(
            bus,
            Params {
                ports: ports as u8,
                microframe_ticks: microframe,
                caplength: DEFAULT_CAPLENGTH,
                dual_role: false,
            },
        ))
    }

    /// A controller on a bus the caller already holds.
    ///
    /// The way to build one without going through the named table — an
    /// embedder that owns its own [`UsbBus`], or a test that wants a bus
    /// nothing else can reach.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>, params: Params) -> EhciController {
        let hcd = Arc::new(Hcd::new(bus, params));
        let port = Arc::new(EhciPort {
            hcd: Arc::clone(&hcd),
        });
        let region = Arc::new(Region::io("ehci", REGISTER_BYTES, port as Arc<dyn MemOps>));
        EhciController { hcd, region }
    }

    /// The engine underneath.
    #[must_use]
    pub fn hcd(&self) -> &Arc<Hcd> {
        &self.hcd
    }
}

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output. Level-triggered, and the AND of `USBSTS` with
    /// `USBINTR` (EHCI 1.0 §2.3.2).
    pub const IRQ: &str = "irq";
}

/// The standard EHCI register block, as something an address space dispatches
/// to.
struct EhciPort {
    hcd: Arc<Hcd>,
}

impl fmt::Debug for EhciPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EhciPort").finish_non_exhaustive()
    }
}

impl MemOps for EhciPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.hcd.sync_for(attrs);
        let caplength = u64::from(self.hcd.params.caplength);
        let value = if offset < caplength {
            self.hcd.read_cap(offset & !0x3)
        } else {
            self.hcd.read_op((offset - caplength) & !0x3)
        };
        narrow_read(offset, value, dst)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // `USBSTS` is write-1-to-clear and `USBCMD` starts the controller.
            // Neither has a harmless version, so a debug write is refused
            // outright (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let caplength = u64::from(self.hcd.params.caplength);
        if offset < caplength {
            // The capability registers are read-only (§2.2).
            return Ok(());
        }
        let Some(value) = word_write(src) else {
            return Err(BusError::BadAccess);
        };
        self.hcd.sync_for(attrs);
        let op = offset - caplength;
        let reset = Hcd::port_at(op)
            .map(|port| (port, self.hcd.portsc(port)))
            .filter(|(_, sc)| sc & PORT_RESET != 0);
        let after = self.hcd.write_op(op, value);
        self.hcd.act(after);
        // Software releasing `PORT_RESET` is what actually drives the reset
        // (§2.3.9), and it is the moment the controller decides whether the
        // device is one it can talk to at all.
        if let Some((port, _)) = reset
            && self.hcd.portsc(port) & PORT_RESET == 0
        {
            self.hcd.finish_reset(port);
            self.hcd.refresh_irq();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Reads may be 8, 16 or 32 bits: `CAPLENGTH` is a byte and
        // `HCIVERSION` a halfword, and every driver reads them that way.
        // Writes are checked separately, because §2.3 requires the operational
        // registers to be written as dwords.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

/// Answer a 1-, 2- or 4-byte read out of the dword at `offset & !3`.
pub(crate) fn narrow_read(offset: u64, value: u32, dst: &mut [u8]) -> MemResult {
    let bytes = value.to_le_bytes();
    let lane = (offset & 0x3) as usize;
    match dst.len() {
        1 | 2 | 4 => {
            if lane + dst.len() > 4 {
                return Err(BusError::BadAccess);
            }
            dst.copy_from_slice(&bytes[lane..lane + dst.len()]);
            Ok(())
        }
        _ => Err(BusError::BadAccess),
    }
}

/// The dword a register write carries, or `None` for a width §2.3 forbids.
pub(crate) fn word_write(src: &[u8]) -> Option<u32> {
    (src.len() == 4).then(|| u32::from_le_bytes([src[0], src[1], src[2], src[3]]))
}

impl Device for EhciController {
    fn class(&self) -> &'static DeviceClass {
        &EHCI_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and a `wire`
        // statement connects the interrupt.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        self.hcd.reset(kind);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.hcd.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.hcd.load(r)
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "an EHCI controller drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.hcd.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.hcd.refresh_irq();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. The schedule runs on its own, once per microframe, and a guest that
    /// polls `USBSTS` has to see the answer at the cycle it polled.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.hcd.ticks()
    }

    fn advance_to(&self, tick: u64) {
        self.hcd.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.hcd.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.hcd.attach_lazy(handle);
    }
}

impl Instance for EhciController {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an EHCI controller masters the bus its queue heads live on: add `space = mem` \
                 to the object",
            ),
        })?;
        self.hcd.attach_space(space, ctx.requester());
        Ok(())
    }
}

/// The `usb.ehci` device class.
pub static EHCI_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a generic EHCI USB 2.0 host controller: the EHCI 1.0 register file and the \
              QH/qTD schedule walker, DMA-reading its work out of guest RAM",
    properties: EHCI_PROPERTIES,
    construct: |props| Ok(Box::new(EhciController::new(props)?)),
};

/// The properties [`EHCI_CLASS`] accepts. Shared with the variant, which
/// accepts the same ones.
pub(crate) static EHCI_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "bus",
        kind: ValueKind::Str,
        required: true,
        summary: "the named USB bus this controller is the root of",
    },
    PropertySpec {
        name: "ports",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many root ports, 1 to 15 (default 1)",
    },
    PropertySpec {
        name: "microframe",
        kind: ValueKind::Uint,
        required: false,
        summary: "clock-domain ticks in one 125 us microframe (default 7500, exact at 60 MHz)",
    },
];

/// Add [`EHCI_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&EHCI_CLASS)
}

/// Bind [`EHCI_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| {
        Ok(Arc::new(EhciController::new(props)?))
    })
}

/// What the validator should know about `usb.ehci`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, MAX_PORTS as u64))
        .prop(PropSchema::new("microframe", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
}
