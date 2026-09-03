//! An xHCI host controller: the register file, the rings, and the contexts.
//!
//! # What an xHCI controller actually is
//!
//! Almost none of xHCI is the register block either — but where
//! [`ehci`](crate::dev::usb::ehci) hands the controller two *linked lists*, xHCI
//! hands it **rings**, and the shape is much closer to
//! [`nvme`](crate::dev::nvme) and [`ahci`](crate::dev::ahci) than to any earlier
//! USB controller. A driver builds:
//!
//! ```text
//!   DCBAAP ──► Device Context Base Address Array
//!                 [0] scratchpad     [1] ──► Device Context (slot 1)
//!                                             ├ Slot Context     (DCI 0)
//!                                             ├ EP Context 0     (DCI 1) ──► Transfer Ring
//!                                             ├ EP Context 1 OUT (DCI 2) ──► Transfer Ring
//!                                             └ …                (DCI 3-31)
//!
//!   CRCR   ──► Command Ring   ── Enable Slot, Address Device, Configure Endpoint …
//!
//!   ERSTBA ──► Event Ring Segment Table ──► Event Ring   ◄── the xHC is the producer
//!   ERDP   ──► where software has read to
//!
//!   DBOFF  ──► Doorbell Array   [0] = command ring, [slot] = endpoint DCI
//! ```
//!
//! and then rings a doorbell. Everything after that is the controller reading
//! guest memory of its own accord.
//!
//! # The cycle bit is the ownership protocol
//!
//! A ring has no head and tail register. Each TRB carries a **Cycle bit**
//! (xHCI 1.2 §4.9), and the producer's Cycle State says which value means
//! "mine". The consumer walks forward until the Cycle bit disagrees, and that
//! is the end of the queue; a **Link TRB** (§6.4.4.1) closes the ring back to
//! its start and toggles the cycle state when its `TC` flag is set. So a ring is
//! a *cycle by construction* — which is exactly the hazard `CLAUDE.md` names,
//! spelled with TRBs instead of queue heads.
//!
//! # Bounding what the guest built
//!
//! Every pointer above comes from guest memory, and every one of them can point
//! back at something already visited:
//!
//! * a Link TRB can point at itself, so consecutive links are bounded by
//!   [`MAX_LINK_HOPS`];
//! * a ring can be entirely Link TRBs, which the same bound catches;
//! * one Transfer Descriptor can chain arbitrarily many TRBs, bounded by
//!   [`MAX_TRBS_PER_TD`];
//! * one doorbell can make arbitrarily many TDs runnable, and one command can
//!   make another runnable, so **one `run()` executes at most
//!   [`MAX_WORK_ITEMS`] work items** and leaves the rest pending;
//! * a single TRB can name a 64 KiB transfer, bounded by [`MAX_PACKETS`]
//!   packets per visit.
//!
//! **And a doorbell rung from inside this controller's own DMA is iterative,
//! not recursive.** A guest may point a Normal TRB's data buffer straight at the
//! doorbell array — four bytes of disk data are a doorbell write — and the
//! `busy` flag turns that into "record the work and return", exactly as
//! [`crate::dev::nvme`] does for a PRP entry aimed at `SQyTDBL`. The outermost
//! `run()` picks the work up.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). This is a lazily advanced device
//! (`ROADMAP.md` §4.2) that holds its own tick and publishes the tick its next
//! microframe falls on. A microframe is 125 µs, which is exactly 7500 ticks of
//! the 60 MHz a USB 2.0 PHY runs at — no float, no residue, and the same
//! argument [`crate::dev::usb::ehci`] makes.
//!
//! What actually happens on a microframe is small, because a doorbell does its
//! work inside the write that rang it (as NVMe does): `MFINDEX` advances
//! (§5.5.1), the ports are polled for changes, the interrupt moderation counter
//! counts down, and any endpoint that answered `NAK` is retried.
//!
//! `IMODC` is specified in 250 ns units (§5.5.2.2) and a microframe is 125 µs,
//! so the counter falls by exactly **500** per microframe. Integer, exact, and
//! never seconds.
//!
//! # Interrupts, and what acknowledging one costs
//!
//! §4.17.2 defines the moderation scheme and §4.17.3 the pin behaviour. An
//! event is posted; if the ring is non-empty, `IMAN.IE` is set, `ERDP.EHB` is
//! clear and `IMODC` has reached zero, then `IMAN.IP` is set, `ERDP.EHB` is set,
//! and `IMODC` is reloaded from `IMODI`. The line is the AND of `USBCMD.INTE`,
//! `IMAN.IE` and `IMAN.IP`, and **only software writing one to `IMAN.IP` drops
//! it**.
//!
//! So acknowledging is three writes in an order the specification fixes:
//!
//! 1. `USBSTS.EINT` — §5.4.2 bit 3: *"Software that uses EINT shall clear it
//!    prior to clearing any IP flags"*, because an `IP` `0`→`1` transition
//!    between the two would be lost.
//! 2. `ERDP` with `EHB` set — §5.5.2.3.3: `EHB` is RW1C and is cleared *by
//!    writing the Dequeue Pointer register*, which is also how software says how
//!    far it has read (§4.9.4).
//! 3. `IMAN.IP` — §4.17.3: *"Once the INTx# signal is asserted, it remains
//!    asserted until the device driver clears the Interrupt Pending (IP) flag."*
//!
//! `tests/usb_xhci.rs` counts the traps a guest takes, because that is what
//! makes the order visible: completing an interrupt-controller claim before
//! step 3 leaves the level asserted and doubles the count.
//!
//! # `MemAttrs::debug`
//!
//! * A debug **read** advances nothing: reads sync with [`AccessKind::Debug`],
//!   and no register in this block is read-to-clear (`CRCR`'s pointer and its
//!   `RCS`/`CS`/`CA` flags read as zero by §5.4.5 whoever asks).
//! * A debug **write** is refused outright ([`BusError::BadAccess`]). There is
//!   no harmless version of a doorbell, and `USBSTS`, `IMAN.IP`, `ERDP.EHB` and
//!   every `PORTSC` change bit are write-1-to-clear: a debugger that touched one
//!   would consume a TRB, advance a dequeue pointer, or acknowledge an interrupt
//!   the guest has not seen.
//!
//! # What is modelled, and what is not
//!
//! * **USB 2.0 root ports only.** One xHCI Supported Protocol Capability
//!   (§7.2), `USB ` major 2 minor 0, covering every port. Low, full and high
//!   speed are all reachable — unlike EHCI, an xHCI root hub drives them
//!   itself, so nothing is handed to a companion and nothing vanishes.
//!   SuperSpeed is *not* modelled, because [`crate::bus::usb`] has no
//!   SuperSpeed and saying otherwise would be a lie a driver could catch.
//! * **One interrupter.** `HCSPARAMS1.MaxIntrs` = 1, wired to one pin. MSI/MSI-X
//!   belong to a PCI function and this controller is bus-agnostic.
//! * **32-byte contexts.** `HCCPARAMS1.CSZ` = 0 (§5.3.6).
//! * **No streams.** `MaxPStreams` in an endpoint context is required to be
//!   zero; a non-zero value is answered with a *Parameter Error* rather than
//!   half-implemented.
//! * **No isochronous transfers, no scratchpad buffers, no save/restore
//!   state.** `USBCMD.CSS`/`CRS` read back zero and do nothing, which is what
//!   §5.4.1 says a controller that does not implement them shall do.
//! * **The transfer-ring position lives in guest memory.** After every doorbell
//!   the endpoint's TR Dequeue Pointer and `DCS` are written back into the
//!   Output Endpoint Context. §6.2.3 leaves that field *undefined* while the
//!   endpoint is Running, so writing through is legal — and it means this
//!   controller carries no hidden per-endpoint position across a snapshot.
//! * **A `NAK` rewinds to the start of the Transfer Descriptor** and the TD is
//!   retried on the next microframe. Partial progress inside a TD is therefore
//!   discarded, which is unreachable for every device in this tree — a mass
//!   storage device and a HID mouse each answer a whole packet or `NAK` before
//!   any byte moves — and is said plainly rather than papered over.
//!
//! # Sources
//!
//! The **eXtensible Host Controller Interface for Universal Serial Bus (xHCI)
//! Requirements Specification, Revision 1.2c** (Intel, document 868295, October
//! 2025), read directly: §4.5 device slots and the Device Context Index, §4.6
//! the commands, §4.9 ring operation, §4.17 interrupters, §4.19 root hub ports,
//! §5.3 the capability registers, §5.4 the operational registers, §5.5 the
//! runtime registers, §5.6 the doorbell array, §6.2 the contexts, §6.4 the TRBs,
//! §6.5 the Event Ring Segment Table, §7.2 the Supported Protocol capability.
//! **USB 2.0** for everything above the controller.
//!
//! No emulator source was consulted (`ROADMAP.md` §1): not QEMU, not Bochs, not
//! VirtualBox, and no operating system's xHCI driver — Linux's `xhci-hcd` is
//! GPLv2 and was not opened. Two web searches for the specification returned
//! links into emulator trees; they were not followed and their domains were
//! excluded from the search.

#[cfg(feature = "dev-usb-xhci-pci")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-xhci-pci")))]
pub mod pci;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};

use core::fmt;

use crate::bus::usb::{
    Completion, DeviceAddress, HCD_RANK, MAX_PORTS, SetupPacket, Speed, Status, UsbBus, buses, host,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.xhci";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub(crate) const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// The register map (§5.3, §5.4, §5.5, §5.6, §7)
// ---------------------------------------------------------------------------

/// `CAPLENGTH` (§5.3.1): where the operational registers start.
///
/// Implementation-defined by the specification, and 0x40 here rather than the
/// 0x20 the capability registers actually occupy so that the Supported Protocol
/// extended capability (§7.2) fits between them at [`offset::XECP`].
pub const CAPLENGTH: u8 = 0x40;

/// `HCIVERSION` (§5.3.2), BCD.
///
/// 1.0.0, because nothing this model implements is optional above xHCI 1.0 —
/// declaring 1.2 would promise `ETE`, `CEM` and the rest of §5.4.1's later bits.
pub const HCIVERSION: u16 = 0x0100;

/// Where the Supported Protocol extended capability sits (§7, §7.2).
const XECP_OFFSET: u64 = 0x20;

/// Where the operational registers start.
const OP_BASE: u64 = CAPLENGTH as u64;

/// Where the per-port register sets start, relative to the operational base
/// (§5.4, Table 5-18).
const PORT_BASE: u64 = 0x400;

/// Bytes in one port register set (§5.4, Table 5-19): `PORTSC`, `PORTPMSC`,
/// `PORTLI`, `PORTHLPMC`.
const PORT_STRIDE: u64 = 0x10;

/// `DBOFF` (§5.3.7): where the doorbell array sits.
const DB_BASE: u64 = 0x1000;

/// `RTSOFF` (§5.3.8): where the runtime registers sit.
const RT_BASE: u64 = 0x2000;

/// Where interrupter 0's register set sits, relative to [`RT_BASE`]
/// (§5.5, Table 5-35).
const IR0_BASE: u64 = 0x20;

/// How much address space the whole register block takes.
///
/// Enough for the runtime registers and their one interrupter, rounded to a
/// page so a machine file can map it without a hole.
pub const REGISTER_BYTES: u64 = 0x3000;

// -- USBCMD (§5.4.1, Table 5-20) --------------------------------------------

/// Run/Stop.
pub const CMD_RS: u32 = 1 << 0;
/// Host Controller Reset. Self-clearing.
pub const CMD_HCRST: u32 = 1 << 1;
/// Interrupter Enable: the master gate on every interrupter's output.
pub const CMD_INTE: u32 = 1 << 2;
/// Host System Error Enable.
pub const CMD_HSEE: u32 = 1 << 3;
/// Enable Wrap Event: post an MFINDEX Wrap Event when `MFINDEX` rolls over.
pub const CMD_EWE: u32 = 1 << 10;
/// Enable U3 MFINDEX Stop.
const CMD_EU3S: u32 = 1 << 11;
/// Everything software may set. `LHCRST` is absent because `HCCPARAMS1.LHRC` is
/// zero; `CSS` and `CRS` always read zero (§5.4.1).
const CMD_MASK: u32 = CMD_RS | CMD_INTE | CMD_HSEE | CMD_EWE | CMD_EU3S;

// -- USBSTS (§5.4.2, Table 5-21) --------------------------------------------

/// The controller is halted. Read-only, and the inverse of `USBCMD.RS`.
pub const STS_HCH: u32 = 1 << 0;
/// Host System Error: a DMA access faulted.
pub const STS_HSE: u32 = 1 << 2;
/// Event Interrupt: some interrupter's `IP` went from zero to one.
pub const STS_EINT: u32 = 1 << 3;
/// Port Change Detect: some port's change bit went from zero to one.
pub const STS_PCD: u32 = 1 << 4;
/// Save/Restore Error. Never set here — neither operation is implemented.
const STS_SRE: u32 = 1 << 10;
/// Controller Not Ready. Always zero: this controller is ready the instant it
/// exists, and §5.4.2 says the flag is cleared when it is.
const STS_CNR: u32 = 1 << 11;
/// Host Controller Error (§5.4.2 bit 12): an internal error software must reset
/// and reinitialise out of (§4.24.1).
///
/// **This model never sets it**, and that is the honest answer rather than a
/// gap: every failure it can actually have is a guest-memory access that
/// faulted, which §5.4.2 calls Host System Error and which [`STS_HSE`] reports.
/// Inventing an internal error would be a state a driver could not diagnose.
/// It is read as part of "the controller is executing" so that a snapshot
/// written by a later version which *does* set it still halts this one.
pub const STS_HCE: u32 = 1 << 12;
/// The write-1-to-clear half of `USBSTS`.
pub const STS_W1C: u32 = STS_HSE | STS_EINT | STS_PCD | STS_SRE;

// -- CRCR (§5.4.5, Table 5-24) ----------------------------------------------

/// Ring Cycle State: the cycle state the first fetched command TRB carries.
const CRCR_RCS: u64 = 1 << 0;
/// Command Stop. Write-1-to-set, reads zero.
const CRCR_CS: u64 = 1 << 1;
/// Command Abort. Write-1-to-set, reads zero.
const CRCR_CA: u64 = 1 << 2;
/// Command Ring Running. Read-only.
const CRCR_CRR: u64 = 1 << 3;
/// The command ring is 64-byte aligned, so the low six bits are not address.
const CRCR_PTR: u64 = !0x3f;

// -- CONFIG (§5.4.7, Table 5-26) --------------------------------------------

/// Max Device Slots Enabled, bits 7:0.
const CONFIG_SLOTS: u32 = 0xff;
/// U3 Entry Enable.
const CONFIG_U3E: u32 = 1 << 8;
/// Configuration Information Enable.
const CONFIG_CIE: u32 = 1 << 9;

// -- PORTSC (§5.4.8, Table 5-27) --------------------------------------------

/// Current Connect Status.
pub const PORT_CCS: u32 = 1 << 0;
/// Port Enabled/Disabled. Set only by the controller; write-1-to-*clear*.
pub const PORT_PED: u32 = 1 << 1;
/// Over-current Active. Never asserted: a modelled bus has no current.
const PORT_OCA: u32 = 1 << 3;
/// Port Reset. Write-1-to-set; the controller clears it when reset completes.
pub const PORT_PR: u32 = 1 << 4;
/// Port Link State, bits 8:5.
const PORT_PLS_SHIFT: u32 = 5;
/// …four bits of it.
const PORT_PLS_MASK: u32 = 0xf;
/// Port Power.
pub const PORT_PP: u32 = 1 << 9;
/// Port Speed, bits 13:10 — a Protocol Speed ID (§7.2.1).
pub const PORT_SPEED_SHIFT: u32 = 10;
/// …four bits of it.
pub const PORT_SPEED_MASK: u32 = 0xf;
/// Port Indicator Control, bits 15:14.
const PORT_PIC_SHIFT: u32 = 14;
/// Port Link State Write Strobe: without it a write to `PLS` is ignored.
const PORT_LWS: u32 = 1 << 16;
/// Connect Status Change.
pub const PORT_CSC: u32 = 1 << 17;
/// Port Enabled/Disabled Change.
pub const PORT_PEC: u32 = 1 << 18;
/// Over-current Change.
const PORT_OCC: u32 = 1 << 20;
/// Port Reset Change.
pub const PORT_PRC: u32 = 1 << 21;
/// Port Link State Change.
const PORT_PLC: u32 = 1 << 22;
/// The change bits of a USB2 protocol port. `WRC` and `CEC` are USB3-only and
/// are `RsvdZ` here (§5.4.8).
const PORT_CHANGE: u32 = PORT_CSC | PORT_PEC | PORT_OCC | PORT_PRC | PORT_PLC;
/// Wake-on-connect / disconnect / over-current enables, bits 27:25. Stored and
/// read back; nothing in this tree wakes.
const PORT_WAKE: u32 = 0x7 << 25;

/// `PLS` value `0`: the link is in U0 — running.
const PLS_U0: u32 = 0;
/// `PLS` value `3`: U3, the suspended state.
const PLS_U3: u32 = 3;
/// `PLS` value `5`: RxDetect, and the reset default (§5.4.8).
const PLS_RXDETECT: u32 = 5;
/// `PLS` value `7`: Polling — a USB2 port with something attached that has not
/// been reset yet (§4.19.1.1).
const PLS_POLLING: u32 = 7;

// -- Interrupter registers (§5.5.2) -----------------------------------------

/// Interrupt Pending. Write-1-to-clear, and the thing that drives the pin.
pub const IMAN_IP: u32 = 1 << 0;
/// Interrupt Enable.
pub const IMAN_IE: u32 = 1 << 1;
/// Dequeue ERST Segment Index, bits 2:0 — a hint, stored and ignored.
const ERDP_DESI: u64 = 0x7;
/// Event Handler Busy. Write-1-to-clear.
pub const ERDP_EHB: u64 = 1 << 3;
/// The Event Ring Dequeue Pointer proper, bits 63:4.
const ERDP_PTR: u64 = !0xf;

/// How many 250 ns `IMODC` ticks one 125 µs microframe is (§5.5.2.2).
///
/// Exactly 500. Integer, and never derived from seconds.
const IMOD_TICKS_PER_MICROFRAME: u32 = 500;

/// `IMODI` out of reset (§5.5.2.2): 4000, which is 1 ms.
const IMOD_RESET: u32 = 4000;

// ---------------------------------------------------------------------------
// TRBs (§6.4)
// ---------------------------------------------------------------------------

/// A TRB is sixteen bytes, always (§6.4).
const TRB_BYTES: u64 = 16;

/// Where the type field sits in a TRB's fourth dword.
const TRB_TYPE_SHIFT: u32 = 10;
/// …six bits of it.
const TRB_TYPE_MASK: u32 = 0x3f;
/// The Cycle bit (§4.9).
const TRB_CYCLE: u32 = 1 << 0;
/// Evaluate Next TRB. Read and not acted on: this controller saves no endpoint
/// state between TRBs, so there is nothing for it to defer.
const TRB_ENT: u32 = 1 << 1;
/// Toggle Cycle, on a Link TRB (§6.4.4.1).
const TRB_TC: u32 = 1 << 1;
/// Interrupt-on Short Packet.
const TRB_ISP: u32 = 1 << 2;
/// Chain: this TRB and the next are one Transfer Descriptor.
const TRB_CH: u32 = 1 << 4;
/// Interrupt On Completion.
const TRB_IOC: u32 = 1 << 5;
/// Immediate Data: the parameter component is data, not a pointer.
const TRB_IDT: u32 = 1 << 6;
/// Block Event Interrupt: post the event, do not assert the interrupt.
const TRB_BEI: u32 = 1 << 9;
/// Direction, on a Data Stage or Status Stage TRB (§6.4.1.2.2, §6.4.1.2.3).
const TRB_DIR: u32 = 1 << 16;
/// Block Set Address Request, on an Address Device Command (§6.4.3.4).
const TRB_BSR: u32 = 1 << 9;
/// Deconfigure, on a Configure Endpoint Command (§6.4.3.5).
const TRB_DC: u32 = 1 << 9;

/// TRB type identifiers (§6.4.6, Table 6-91).
pub mod trb {
    /// Bulk and interrupt data, and the data stage of a control transfer.
    pub const NORMAL: u32 = 1;
    /// The eight bytes of a `SETUP` packet, carried immediately.
    pub const SETUP_STAGE: u32 = 2;
    /// The data stage of a control transfer.
    pub const DATA_STAGE: u32 = 3;
    /// The zero-length status stage of a control transfer.
    pub const STATUS_STAGE: u32 = 4;
    /// Isochronous data. Recognised and refused — see the module docs.
    pub const ISOCH: u32 = 5;
    /// The ring's own continuation.
    pub const LINK: u32 = 6;
    /// Software-defined event data.
    pub const EVENT_DATA: u32 = 7;
    /// A transfer-ring no-op.
    pub const NO_OP: u32 = 8;
    /// Allocate a device slot.
    pub const ENABLE_SLOT: u32 = 9;
    /// Release one.
    pub const DISABLE_SLOT: u32 = 10;
    /// Address the device in a slot.
    pub const ADDRESS_DEVICE: u32 = 11;
    /// Add and drop endpoints.
    pub const CONFIGURE_ENDPOINT: u32 = 12;
    /// Re-evaluate selected context fields.
    pub const EVALUATE_CONTEXT: u32 = 13;
    /// Take an endpoint out of the Halted state.
    pub const RESET_ENDPOINT: u32 = 14;
    /// Stop an endpoint.
    pub const STOP_ENDPOINT: u32 = 15;
    /// Move an endpoint's transfer-ring dequeue pointer.
    pub const SET_TR_DEQUEUE: u32 = 16;
    /// Put a slot back in the Default state.
    pub const RESET_DEVICE: u32 = 17;
    /// A command-ring no-op.
    pub const NO_OP_COMMAND: u32 = 23;
    /// A transfer completed, short-packeted or failed.
    pub const TRANSFER_EVENT: u32 = 32;
    /// A command completed.
    pub const COMMAND_COMPLETION_EVENT: u32 = 33;
    /// A port's change bits went from all-clear to not.
    pub const PORT_STATUS_CHANGE_EVENT: u32 = 34;
    /// The controller itself has something to report.
    pub const HOST_CONTROLLER_EVENT: u32 = 37;
    /// `MFINDEX` rolled over, and `USBCMD.EWE` asked to hear about it.
    pub const MFINDEX_WRAP_EVENT: u32 = 39;
}

/// TRB completion codes (§6.4.5, Table 6-90).
pub mod code {
    /// The operation succeeded.
    pub const SUCCESS: u32 = 1;
    /// The controller could not keep up. Never reported here.
    pub const DATA_BUFFER_ERROR: u32 = 2;
    /// The device sent more than `wMaxPacketSize`.
    pub const BABBLE: u32 = 3;
    /// No valid response from the device.
    pub const USB_TRANSACTION_ERROR: u32 = 4;
    /// A TRB parameter is out of range or invalid.
    pub const TRB_ERROR: u32 = 5;
    /// The endpoint stalled.
    pub const STALL_ERROR: u32 = 6;
    /// No slots left.
    pub const NO_SLOTS_AVAILABLE: u32 = 9;
    /// A command named a slot that is not enabled.
    pub const SLOT_NOT_ENABLED: u32 = 11;
    /// A doorbell was rung for an endpoint that is not enabled.
    pub const ENDPOINT_NOT_ENABLED: u32 = 12;
    /// Fewer bytes arrived than the TRB asked for.
    pub const SHORT_PACKET: u32 = 13;
    /// A context parameter is invalid.
    pub const PARAMETER_ERROR: u32 = 17;
    /// A command asked for an illegal state transition.
    pub const CONTEXT_STATE_ERROR: u32 = 19;
    /// The event ring had no room.
    pub const EVENT_RING_FULL: u32 = 21;
    /// A command was stopped by `CRCR.CS` or `CRCR.CA`.
    pub const COMMAND_RING_STOPPED: u32 = 24;
}

// ---------------------------------------------------------------------------
// Contexts (§6.2)
// ---------------------------------------------------------------------------

/// Bytes in one context entry.
///
/// Thirty-two, because `HCCPARAMS1.CSZ` is reported as zero (§5.3.6, §6.2.2).
const CONTEXT_BYTES: u64 = 32;

/// Dwords in one context entry.
const CONTEXT_DWORDS: usize = 8;

/// The largest Device Context Index there is (§4.5.1): the Slot Context plus
/// thirty-one endpoints.
const MAX_DCI: u32 = 31;

/// Slot Context dword 0: Context Entries, bits 31:27 (§6.2.2, Table 6-4).
const SLOT_ENTRIES_SHIFT: u32 = 27;
/// Slot Context dword 0: Speed, bits 23:20.
const SLOT_SPEED_SHIFT: u32 = 20;
/// Slot Context dword 1: Root Hub Port Number, bits 23:16 (Table 6-5).
const SLOT_PORT_SHIFT: u32 = 16;
/// Slot Context dword 3: Slot State, bits 31:27 (Table 6-7).
const SLOT_STATE_SHIFT: u32 = 27;

/// Slot State `0`: Disabled or Enabled — the two are indistinguishable in the
/// Output Slot Context, which is why [`Regs::slot_enabled`] exists.
const SLOT_STATE_ENABLED: u32 = 0;
/// Slot State `1`: Default.
const SLOT_STATE_DEFAULT: u32 = 1;
/// Slot State `2`: Addressed.
const SLOT_STATE_ADDRESSED: u32 = 2;
/// Slot State `3`: Configured.
const SLOT_STATE_CONFIGURED: u32 = 3;

/// Endpoint Context dword 0: Endpoint State, bits 2:0 (§6.2.3, Table 6-8).
const EP_STATE_MASK: u32 = 0x7;
/// Endpoint Context dword 0: Max Primary Streams, bits 14:10.
const EP_MAXPSTREAMS_SHIFT: u32 = 10;
/// Endpoint Context dword 1: Endpoint Type, bits 5:3 (Table 6-9).
const EP_TYPE_SHIFT: u32 = 3;
/// Endpoint Context dword 1: Max Packet Size, bits 31:16.
const EP_MPS_SHIFT: u32 = 16;
/// Endpoint Context dword 2: Dequeue Cycle State, bit 0 (Table 6-10).
const EP_DCS: u32 = 1 << 0;

/// Endpoint State `0`: Disabled.
const EP_STATE_DISABLED: u32 = 0;
/// Endpoint State `1`: Running.
const EP_STATE_RUNNING: u32 = 1;
/// Endpoint State `2`: Halted.
const EP_STATE_HALTED: u32 = 2;
/// Endpoint State `3`: Stopped.
const EP_STATE_STOPPED: u32 = 3;
/// Endpoint State `4`: Error.
const EP_STATE_ERROR: u32 = 4;

/// Endpoint Type `0`: the context is not valid (§6.2.3, Table 6-9).
const EP_TYPE_INVALID: u32 = 0;
/// Endpoint Type `4`: bidirectional control.
const EP_TYPE_CONTROL: u32 = 4;

// ---------------------------------------------------------------------------
// Bounds on a guest-controlled walk
// ---------------------------------------------------------------------------

/// The largest Slot ID this model can allocate.
///
/// Thirty-one rather than the 255 §5.3.3 allows, because the enabled-slot
/// bitmap is a `u32` whose bit *n* is slot *n* — and Slot ID zero is not a slot
/// (§5.6), so bit 0 is never used and thirty-one is the ceiling. A synthetic
/// board needs one slot; nothing in this tree needs thirty.
///
/// **Thirty-two was wrong**, and `fuzz/fuzz_targets/usb_xhci.rs` found it on its
/// first seeded run: a Reset Endpoint Command naming slot 32 passed the range
/// check and then shifted a `u32` by thirty-two, which is a panic in a debug
/// build and a wrap in a release one. Every use now goes through one private
/// helper that returns the bit or `None`, which cannot express the mistake.
pub const MAX_SLOTS: usize = 31;

/// The bit slot `slot` takes in the enabled-slot bitmap, or `None` if it is not
/// a Slot ID this controller can have.
///
/// A function rather than a shift written out at each site: a Slot ID reaching
/// these paths came out of a TRB the guest wrote, and `1 << 32` is undefined
/// behaviour in C and a panic here. See [`MAX_SLOTS`].
const fn slot_bit(slot: u8) -> Option<u32> {
    if slot == 0 || slot as usize > MAX_SLOTS {
        None
    } else {
        Some(1u32 << slot)
    }
}

/// How many Event Ring Segment Table entries this model supports.
///
/// `HCSPARAMS2.ERST Max` is reported as 4, and 2⁴ is sixteen (§5.3.4).
pub const MAX_ERST_ENTRIES: u32 = 16;

/// …and the `ERST Max` field that says so.
const ERST_MAX: u32 = 4;

/// The smallest and largest an Event Ring segment may be (§6.5, Table 6-95).
const ERST_MIN_SEGMENT: u32 = 16;
/// …and the largest.
const ERST_MAX_SEGMENT: u32 = 4096;

/// How many consecutive Link TRBs one fetch will follow.
///
/// A Link TRB pointing at itself is a ring with no end; so is a ring made
/// entirely of Link TRBs. This is what makes both of them terminate.
pub const MAX_LINK_HOPS: usize = 16;

/// How many TRBs one Transfer Descriptor may chain.
pub const MAX_TRBS_PER_TD: usize = 64;

/// How many work items — one command, or one Transfer Descriptor — a single
/// `run()` executes before leaving the rest pending.
///
/// The outer bound: a command can make an endpoint runnable and a transfer can
/// ring a doorbell, so without this the engine has no termination argument at
/// all.
pub const MAX_WORK_ITEMS: usize = 256;

/// How many packets one data TRB moves per visit.
///
/// A TRB may name 64 KiB, which is 8192 packets at an eight-byte maximum
/// packet size. Stopping short leaves the TD where it is for the next
/// microframe, which is bounded and is what a controller with a finite
/// microframe does.
pub const MAX_PACKETS: usize = 1024;

/// How many transactions the `SET_ADDRESS` an Address Device Command issues may
/// take.
///
/// A control transfer with no data stage is a `SETUP` and a status stage, so
/// three is generous; the budget exists because [`host::ControlTransfer`]
/// answers `NAK` by making no progress and a device that always `NAK`s would
/// otherwise spin.
const MAX_CONTROL_STEPS: usize = 8;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// The parts of a controller a board gets to choose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Params {
    /// How many root ports. 1 to 15 — [`MAX_PORTS`] is what the fabric routes.
    pub ports: u8,
    /// How many device slots, 1 to [`MAX_SLOTS`]. `HCSPARAMS1.MaxSlots`.
    pub slots: u8,
    /// How many clock-domain ticks one 125 µs microframe takes.
    ///
    /// A property rather than a constant because it belongs to the *board's*
    /// clock tree: at the 60 MHz a USB 2.0 PHY runs at it is exactly 7500.
    pub microframe_ticks: u64,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            ports: 1,
            slots: 8,
            microframe_ticks: 7500,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the ring positions the
/// controller keeps for itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Regs {
    /// Domain ticks simulated. The authoritative copy; an atomic mirrors it.
    ticks: u64,
    usbcmd: u32,
    usbsts: u32,
    dnctrl: u32,
    /// `CRCR`'s pointer, as software last wrote it. The flags are not stored:
    /// §5.4.5 says every one of them reads back zero except `CRR`.
    crcr_ptr: u64,
    dcbaap: u64,
    config: u32,
    portsc: [u32; MAX_PORTS],
    /// Whether each port's change bits were non-zero last time they were
    /// looked at — the `PSCEG` variable §4.19.2 defines the Port Status Change
    /// Event as the rising edge of.
    psceg: [bool; MAX_PORTS],
    mfindex: u32,

    // -- interrupter 0 (§5.5.2) --------------------------------------------
    iman: u32,
    imod: u32,
    /// The moderation down-counter, in 250 ns units.
    imodc: u32,
    erstsz: u32,
    erstba: u64,
    erdp: u64,

    // -- the command ring (§4.6.1) -----------------------------------------
    /// Where the next command TRB is fetched from.
    cmd_dequeue: u64,
    /// The Consumer Cycle State the command ring is being read with.
    cmd_ccs: bool,
    /// `CRCR.CRR`.
    crr: bool,

    // -- the event ring (§4.9.4) -------------------------------------------
    /// Whether `ERSTBA` has been written, which is what starts the ring.
    er_started: bool,
    /// Which Event Ring Segment Table entry is being filled.
    er_index: u32,
    /// That segment's base address and size, cached so the enqueue pointer can
    /// be computed without a guest read under the lock.
    er_base: u64,
    /// …its size, in TRBs.
    er_size: u32,
    /// The base of the *next* segment, for the same reason: the ring-full check
    /// needs to know where the enqueue pointer would land.
    er_next_base: u64,
    /// How far into the current segment the enqueue pointer is.
    er_offset: u32,
    /// The Producer Cycle State (§4.9.4), initialised to one.
    er_pcs: bool,
    /// Whether the ring is full and the controller has stopped consuming its
    /// command and transfer rings (§4.9.4, step 13b).
    er_full: bool,

    // -- pending work ------------------------------------------------------
    /// One bit per slot; slot *n* is bit *n*, and bit 0 is never set.
    slot_enabled: u32,
    /// Whether the command ring has been rung and not yet drained.
    cmd_pending: bool,
    /// One bit per Device Context Index, per slot: a doorbell that has not been
    /// answered yet.
    ep_pending: [u32; MAX_SLOTS + 1],
    /// The same, for endpoints that answered `NAK` and are waiting for the next
    /// microframe.
    ep_retry: [u32; MAX_SLOTS + 1],
}

impl Regs {
    fn reset(ports: u8) -> Regs {
        let mut regs = Regs {
            ticks: 0,
            usbcmd: 0,
            usbsts: STS_HCH,
            dnctrl: 0,
            crcr_ptr: 0,
            dcbaap: 0,
            config: 0,
            portsc: [0; MAX_PORTS],
            psceg: [false; MAX_PORTS],
            mfindex: 0,
            iman: 0,
            imod: IMOD_RESET,
            imodc: 0,
            erstsz: 0,
            erstba: 0,
            erdp: 0,
            cmd_dequeue: 0,
            cmd_ccs: true,
            crr: false,
            er_started: false,
            er_index: 0,
            er_base: 0,
            er_size: 0,
            er_next_base: 0,
            er_offset: 0,
            er_pcs: true,
            er_full: false,
            slot_enabled: 0,
            cmd_pending: false,
            ep_pending: [0; MAX_SLOTS + 1],
            ep_retry: [0; MAX_SLOTS + 1],
        };
        for port in regs.portsc.iter_mut().take(usize::from(ports)) {
            // `HCCPARAMS1.PPC` is zero, so every port is hard-wired to power
            // and `PP` reads one (§5.4.8). `PLS` resets to RxDetect.
            *port = PORT_PP | (PLS_RXDETECT << PORT_PLS_SHIFT);
        }
        regs
    }

    /// Whether the controller is executing (§5.4.1, §5.4.2).
    fn running(&self) -> bool {
        self.usbcmd & CMD_RS != 0 && self.usbsts & (STS_HCH | STS_HCE) == 0
    }

    /// Whether the event ring holds anything software has not read (§4.9.4).
    ///
    /// This is the *Interrupt Pending Enable* of §4.17.2's flow diagram: the
    /// enqueue pointer having moved away from the dequeue pointer.
    fn ipe(&self) -> bool {
        self.er_started && self.enqueue_addr() != self.erdp & ERDP_PTR
    }

    /// Where the next event TRB goes.
    fn enqueue_addr(&self) -> u64 {
        self.er_base
            .wrapping_add(u64::from(self.er_offset).wrapping_mul(TRB_BYTES))
    }

    /// Where the one after that would go — needed for the ring-full check
    /// (§4.9.4, steps 12 and 13).
    fn next_enqueue_addr(&self) -> u64 {
        if self.er_offset + 1 < self.er_size {
            self.er_base
                .wrapping_add(u64::from(self.er_offset + 1).wrapping_mul(TRB_BYTES))
        } else {
            self.er_next_base
        }
    }

    /// How many device slots software has enabled (§5.4.7).
    fn slots_enabled(&self) -> u32 {
        self.config & CONFIG_SLOTS
    }

    /// Re-derive the interrupt state after an event has been posted or a
    /// register written (§4.17.2, Figure 4-22).
    ///
    /// Called with the register lock held, because every input is a register.
    fn arm_interrupt(&mut self) {
        if self.iman & IMAN_IE == 0 || !self.ipe() || self.erdp & ERDP_EHB != 0 || self.imodc != 0 {
            return;
        }
        if self.iman & IMAN_IP == 0 {
            self.iman |= IMAN_IP;
            // §5.4.2 bit 3: `EINT` is the logical OR of every interrupter's
            // `IP` zero-to-one transition.
            self.usbsts |= STS_EINT;
        }
        self.erdp |= ERDP_EHB;
        self.imodc = self.imod & 0xffff;
    }
}

/// An xHCI host controller: the register file, the rings and the contexts.
///
/// The **engine**, separate from the register map for the same reason
/// [`crate::dev::usb::ehci::Hcd`] is: an SoC or a PCI function that wants this
/// controller at different offsets wraps it rather than forking it.
pub struct Xhci {
    bus: Arc<UsbBus>,
    params: Params,
    regs: Mutex<Regs>,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `Regs::ticks`.
    ticks: AtomicU64,
    /// The tick the next microframe falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The space this controller masters. `Weak`, like every bus master's
    /// handle: the machine owns the space.
    space: Mutex<Option<Weak<AddressSpace>>>,
    requester: AtomicU32,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The level the interrupt output is being held at, so a debug read is
    /// free.
    irq_level: AtomicU32,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
    /// Whether the engine is already running somewhere up the stack.
    ///
    /// **This is the whole re-entrancy answer** (module docs): a doorbell rung
    /// from inside one of this controller's own guest-memory accesses records
    /// its work and returns, and the outermost `run()` picks it up.
    busy: AtomicBool,
    /// Whether the transport permits this controller to master the bus.
    ///
    /// True out of reset, because a controller soldered to an SoC's internal
    /// bus has nothing that could say otherwise — the boards in `machines/`
    /// that map this register block directly never touch it. A **PCI**
    /// function does: *PCI Local Bus Specification* Rev 2.1 §6.2.2's Bus Master
    /// Enable is clear at reset and a function whose `COMMAND[2]` is clear may
    /// not generate a cycle, so [`pci`] drives this from the Command register
    /// and the engine fetches nothing until firmware sets it.
    ///
    /// It gates [`Xhci::space`] rather than each walk, which is the one place
    /// every DMA in this file goes through — a fetch, a context read, a
    /// writeback, an event post and the segment-table load all ask for the
    /// space first and all already do nothing when there is none.
    master: AtomicBool,
}

/// "Nothing scheduled", as [`Xhci::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

impl fmt::Debug for Xhci {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Xhci");
        s.field("ports", &self.params.ports);
        s.field("slots", &self.params.slots);
        match self.regs.try_lock() {
            Some(regs) => s.field("usbsts", &regs.usbsts).finish_non_exhaustive(),
            None => s.field("usbsts", &"<in use>").finish_non_exhaustive(),
        }
    }
}

/// What a register write asks for once the register lock is released.
///
/// The re-entrancy contract (`core::device`): decide under the lock, release,
/// *then* act outward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// Nothing but the interrupt refresh every write ends with.
    Nothing,
    /// `USBCMD.HCRST`: put everything back and start again.
    Reset,
    /// A port moved: drive a reset, or settle an enable.
    Port(u8),
    /// `ERSTBA` moved: fetch the segment table's first entries.
    EventRing,
    /// Something made work runnable: drain the rings.
    Run,
}

/// One thing the engine can do, chosen under the lock and executed without it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Job {
    /// Fetch and execute one TRB from the command ring.
    Command,
    /// Execute one Transfer Descriptor on `(slot, dci)`.
    Transfer(u8, u32),
}

impl Xhci {
    /// A controller on `bus`, configured by `params`.
    #[must_use]
    pub fn new(bus: Arc<UsbBus>, params: Params) -> Xhci {
        let params = Params {
            ports: params.ports.clamp(1, MAX_PORTS as u8),
            slots: params.slots.clamp(1, MAX_SLOTS as u8),
            microframe_ticks: params.microframe_ticks.max(1),
        };
        Xhci {
            bus,
            params,
            regs: Mutex::with_rank(HCD_RANK, Regs::reset(params.ports)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            space: Mutex::with_rank(LockRank::WIRE, None),
            requester: AtomicU32::new(RequesterId::ANONYMOUS.0),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            irq_level: AtomicU32::new(0),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
            busy: AtomicBool::new(false),
            master: AtomicBool::new(true),
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

    /// `PORTSC` for port `port` — zero-based here, one-based in the register
    /// map (§5.4.8) — or zero for a port that does not exist.
    #[must_use]
    pub fn portsc(&self, port: u8) -> u32 {
        self.regs
            .lock()
            .portsc
            .get(usize::from(port))
            .copied()
            .unwrap_or(0)
    }

    /// Whether device slot `slot` has been allocated by an Enable Slot Command.
    #[must_use]
    pub fn slot_enabled(&self, slot: u8) -> bool {
        slot_bit(slot).is_some_and(|bit| self.regs.lock().slot_enabled & bit != 0)
    }

    /// Give the controller the address space its rings live in, and the
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

    /// Tell the controller the handle that catches it up.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.lazy.lock() = Some(handle);
    }

    /// Whether the transport permits this controller to master the bus.
    ///
    /// Set by a transport that has such a bit — [`pci`]'s `COMMAND[2]` — and
    /// left alone by one that does not. Not part of the snapshot: it is the
    /// Command register said twice, and the transport re-derives it on load
    /// (`CLAUDE.md`, derived state).
    pub fn set_master(&self, allowed: bool) {
        self.master.store(allowed, Ordering::Relaxed);
    }

    /// Whether this controller may currently fetch.
    #[must_use]
    pub fn is_master(&self) -> bool {
        self.master.load(Ordering::Relaxed)
    }

    /// The space this controller masters, if it still exists and it is allowed
    /// to.
    fn space(&self) -> Option<Arc<AddressSpace>> {
        if !self.master.load(Ordering::Relaxed) {
            return None;
        }
        self.space.lock().as_ref().and_then(Weak::upgrade)
    }

    /// The attributes this controller's own accesses carry.
    fn attrs(&self) -> MemAttrs {
        MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)))
    }

    /// Bring the controller up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
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
        let _ = handle.sync(kind);
    }

    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, regs: &Regs) {
        self.ticks.store(regs.ticks, Ordering::Relaxed);
        let next = if regs.running() {
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

    /// Re-derive the interrupt output and drive it (§4.17.3).
    ///
    /// The line is the AND of `USBCMD.INTE`, `IMAN.IE` and `IMAN.IP`, and it
    /// stays asserted until software writes a one to `IP`.
    pub fn refresh_irq(&self) {
        let asserted = {
            let regs = self.regs.lock();
            regs.usbcmd & CMD_INTE != 0 && regs.iman & (IMAN_IE | IMAN_IP) == (IMAN_IE | IMAN_IP)
        };
        self.irq_level.store(u32::from(asserted), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(Level::from_bool(asserted));
        }
    }

    // -----------------------------------------------------------------
    // Guest memory
    // -----------------------------------------------------------------

    /// One dword of guest memory, or `None` for a bus fault.
    fn read32(&self, space: &AddressSpace, addr: u64) -> Option<u32> {
        space
            .read(addr, Width::U32, self.attrs())
            .ok()
            .map(|value| value as u32)
    }

    /// Store one dword of guest memory, or `None` for a bus fault.
    fn write32(&self, space: &AddressSpace, addr: u64, value: u32) -> Option<()> {
        space
            .write(addr, Width::U32, u64::from(value), self.attrs())
            .ok()
    }

    /// One 64-bit pointer, little-endian, as two dwords (§5.1).
    fn read64(&self, space: &AddressSpace, addr: u64) -> Option<u64> {
        let lo = self.read32(space, addr)?;
        let hi = self.read32(space, addr.wrapping_add(4))?;
        Some(u64::from(lo) | (u64::from(hi) << 32))
    }

    /// One TRB: four dwords (§6.4).
    fn read_trb(&self, space: &AddressSpace, addr: u64) -> Option<[u32; 4]> {
        let mut trb = [0u32; 4];
        for (i, slot) in trb.iter_mut().enumerate() {
            *slot = self.read32(space, addr.wrapping_add((i * 4) as u64))?;
        }
        Some(trb)
    }

    /// Store one TRB.
    ///
    /// The Cycle bit is written **last**, because it is the ownership flag: a
    /// consumer that saw it early would read the other three dwords before they
    /// were there (§4.9).
    fn write_trb(&self, space: &AddressSpace, addr: u64, trb: [u32; 4]) -> Option<()> {
        for (i, word) in trb.iter().enumerate().take(3) {
            self.write32(space, addr.wrapping_add((i * 4) as u64), *word)?;
        }
        self.write32(space, addr.wrapping_add(12), trb[3])
    }

    /// One context entry: eight dwords (§6.2).
    fn read_context(&self, space: &AddressSpace, addr: u64) -> Option<[u32; CONTEXT_DWORDS]> {
        let mut ctx = [0u32; CONTEXT_DWORDS];
        for (i, slot) in ctx.iter_mut().enumerate() {
            *slot = self.read32(space, addr.wrapping_add((i * 4) as u64))?;
        }
        Some(ctx)
    }

    /// Store one context entry.
    fn write_context(
        &self,
        space: &AddressSpace,
        addr: u64,
        ctx: &[u32; CONTEXT_DWORDS],
    ) -> Option<()> {
        for (i, value) in ctx.iter().enumerate() {
            self.write32(space, addr.wrapping_add((i * 4) as u64), *value)?;
        }
        Some(())
    }

    /// The Output Device Context for `slot`, out of the Device Context Base
    /// Address Array (§6.1).
    fn device_context(&self, space: &AddressSpace, slot: u8) -> Option<u64> {
        let dcbaap = self.regs.lock().dcbaap;
        if dcbaap == 0 {
            return None;
        }
        let entry = dcbaap.wrapping_add(u64::from(slot).wrapping_mul(8));
        let base = self.read64(space, entry)?;
        (base != 0).then_some(base & !0x3f)
    }

    /// A DMA access faulted (§5.4.2, `Host System Error`).
    ///
    /// The controller stops: continuing to walk rings it cannot read would
    /// invent transfers out of bus faults.
    fn host_system_error(&self) {
        let mut regs = self.regs.lock();
        regs.usbsts |= STS_HSE | STS_HCH;
        regs.usbcmd &= !CMD_RS;
        regs.crr = false;
        self.publish(&regs);
    }

    // -----------------------------------------------------------------
    // The event ring (§4.9.4)
    // -----------------------------------------------------------------

    /// Read the Event Ring Segment Table entry `index` and cache it.
    ///
    /// Called with no lock held: it reaches guest memory.
    fn load_erst(&self, space: &AddressSpace, index: u32) -> Option<(u64, u32)> {
        let (erstba, erstsz) = {
            let regs = self.regs.lock();
            (regs.erstba, regs.erstsz)
        };
        if erstsz == 0 {
            return None;
        }
        let entry = erstba.wrapping_add(u64::from(index % erstsz).wrapping_mul(16));
        let base = self.read64(space, entry)? & !0x3f;
        let size = self.read32(space, entry.wrapping_add(8))? & 0xffff;
        // §6.5: a segment holds between 16 and 4096 TRBs. A driver that says
        // otherwise gets the nearest legal size rather than an unbounded walk.
        Some((base, size.clamp(ERST_MIN_SEGMENT, ERST_MAX_SEGMENT)))
    }

    /// `ERSTBA` was written: put the Event Ring State Machine in the Start
    /// state (§5.5.2.3.2) by fetching the first two segments.
    fn start_event_ring(&self, space: &AddressSpace) {
        let erstsz = self.regs.lock().erstsz;
        if erstsz == 0 || erstsz > MAX_ERST_ENTRIES {
            // §5.5.2.3.1: zero disables the ring, and more entries than the
            // controller supports is a value it never promised to honour.
            let mut regs = self.regs.lock();
            regs.er_started = false;
            return;
        }
        let Some((base, size)) = self.load_erst(space, 0) else {
            self.host_system_error();
            return;
        };
        let next = self.load_erst(space, 1 % erstsz).map(|(b, _)| b);
        let mut regs = self.regs.lock();
        regs.er_started = true;
        regs.er_index = 0;
        regs.er_base = base;
        regs.er_size = size;
        regs.er_next_base = next.unwrap_or(base);
        regs.er_offset = 0;
        regs.er_pcs = true;
        regs.er_full = false;
    }

    /// Put one event on the ring, and arm the interrupt if that is what §4.17.2
    /// says should happen.
    ///
    /// Returns whether the event was posted. `false` means the ring was full or
    /// had never been started, and the caller stops what it is doing — which is
    /// what §4.9.4 asks of an xHC whose Primary Event Ring has filled.
    fn post_event(&self, space: &AddressSpace, trb: [u32; 4]) -> bool {
        self.post_event_maybe_blocked(space, trb, false)
    }

    /// [`post_event`](Xhci::post_event), with §4.17.5's Block Event Interrupt:
    /// the event goes on the ring and no interrupt is asserted for it, which is
    /// what a TRB with `BEI` and `IOC` both set asks for.
    fn post_event_maybe_blocked(
        &self,
        space: &AddressSpace,
        mut trb: [u32; 4],
        block: bool,
    ) -> bool {
        let plan = {
            let mut regs = self.regs.lock();
            if !regs.er_started || regs.er_full {
                None
            } else {
                let addr = regs.enqueue_addr();
                let cycle = regs.er_pcs;
                let dequeue = regs.erdp & ERDP_PTR;
                trb[3] = (trb[3] & !TRB_CYCLE) | u32::from(cycle);
                let full = regs.next_enqueue_addr() == dequeue;
                if full {
                    // §4.9.4 step 13b: the xHC writes an Event Ring Full Error
                    // Event to the EREP, advances it, and stops consuming its
                    // command and transfer rings until software moves the ERDP.
                    // The event that could not be posted is lost — this model
                    // has no internal buffer to hold it in, and says so rather
                    // than pretending otherwise (module docs).
                    trb = [
                        0,
                        0,
                        code::EVENT_RING_FULL << 24,
                        u32::from(cycle) | (trb::HOST_CONTROLLER_EVENT << TRB_TYPE_SHIFT),
                    ];
                    regs.er_full = true;
                }
                let wrapped = self.advance_enqueue(&mut regs);
                Some((addr, full, wrapped))
            }
        };

        let Some((addr, full, wrapped)) = plan else {
            return false;
        };

        if self.write_trb(space, addr, trb).is_none() {
            self.host_system_error();
            return false;
        }

        if wrapped {
            // The enqueue pointer moved into a new segment; the cached base and
            // size follow it, and so does the base of the one after.
            let (index, erstsz) = {
                let regs = self.regs.lock();
                (regs.er_index, regs.erstsz)
            };
            match (
                self.load_erst(space, index),
                self.load_erst(space, (index + 1) % erstsz.max(1)),
            ) {
                (Some((base, size)), next) => {
                    let mut regs = self.regs.lock();
                    regs.er_base = base;
                    regs.er_size = size;
                    regs.er_next_base = next.map_or(base, |(b, _)| b);
                }
                _ => {
                    self.host_system_error();
                    return false;
                }
            }
        }

        if !block {
            let mut regs = self.regs.lock();
            regs.arm_interrupt();
        }
        self.refresh_irq();
        !full
    }

    /// Move the enqueue pointer on one TRB, reporting whether it left the
    /// current segment (§4.9.4, steps 14 to 16).
    fn advance_enqueue(&self, regs: &mut Regs) -> bool {
        regs.er_offset += 1;
        if regs.er_offset < regs.er_size {
            return false;
        }
        regs.er_offset = 0;
        let erstsz = regs.erstsz.max(1);
        regs.er_index = (regs.er_index + 1) % erstsz;
        if regs.er_index == 0 {
            // Back to the first segment, so the Producer Cycle State flips.
            regs.er_pcs = !regs.er_pcs;
        }
        // The cached base and size are stale until the caller refetches them;
        // in the meantime the enqueue address is the next segment's base.
        regs.er_base = regs.er_next_base;
        true
    }

    /// A Transfer Event (§6.4.2.1).
    ///
    /// `at` is the TRB pointer and whether it is Event Data rather than an
    /// address (§6.4.2.1 bit 2); `what` is the residual and the completion
    /// code; `who` is the Slot ID and the Device Context Index. Grouped rather
    /// than eight positional arguments, because the four `u32`s were exactly
    /// the kind of list a caller transposes.
    fn transfer_event(
        &self,
        space: &AddressSpace,
        at: (u64, bool),
        what: (u32, u32),
        who: (u8, u32),
    ) -> bool {
        self.transfer_event_blocked(space, at, what, who, false)
    }

    /// [`transfer_event`](Xhci::transfer_event), with §4.17.5's Block Event
    /// Interrupt.
    fn transfer_event_blocked(
        &self,
        space: &AddressSpace,
        at: (u64, bool),
        what: (u32, u32),
        who: (u8, u32),
        block: bool,
    ) -> bool {
        let (pointer, event_data) = at;
        let (residual, completion) = what;
        let (slot, dci) = who;
        let trb = [
            pointer as u32,
            (pointer >> 32) as u32,
            (residual & 0x00ff_ffff) | (completion << 24),
            (trb::TRANSFER_EVENT << TRB_TYPE_SHIFT)
                | if event_data { 1 << 2 } else { 0 }
                | ((dci & 0x1f) << 16)
                | (u32::from(slot) << 24),
        ];
        self.post_event_maybe_blocked(space, trb, block)
    }

    /// A Command Completion Event (§6.4.2.2).
    fn command_event(&self, space: &AddressSpace, command: u64, completion: u32, slot: u8) -> bool {
        let trb = [
            command as u32,
            (command >> 32) as u32,
            completion << 24,
            (trb::COMMAND_COMPLETION_EVENT << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
        ];
        self.post_event(space, trb)
    }

    /// A Port Status Change Event (§6.4.2.3). `port` is one-based, as the
    /// register map numbers ports.
    fn port_event(&self, space: &AddressSpace, port: u8) -> bool {
        let trb = [
            u32::from(port) << 24,
            0,
            code::SUCCESS << 24,
            trb::PORT_STATUS_CHANGE_EVENT << TRB_TYPE_SHIFT,
        ];
        self.post_event(space, trb)
    }

    // -----------------------------------------------------------------
    // The register file
    // -----------------------------------------------------------------

    /// Read a capability register (§5.3), by offset from the base.
    ///
    /// Read-only and side-effect free, which is why it takes no attributes.
    #[must_use]
    pub fn read_cap(&self, offset: u64) -> u32 {
        match offset {
            // CAPLENGTH in bits 7:0, HCIVERSION in 31:16 (§5.3.1, §5.3.2).
            0x00 => u32::from(CAPLENGTH) | (u32::from(HCIVERSION) << 16),
            // HCSPARAMS1 (§5.3.3): MaxSlots, MaxIntrs = 1, MaxPorts.
            0x04 => u32::from(self.params.slots) | (1 << 8) | (u32::from(self.params.ports) << 24),
            // HCSPARAMS2 (§5.3.4): IST = 0, ERST Max, no scratchpad buffers.
            0x08 => ERST_MAX << 4,
            // HCSPARAMS3 (§5.3.5): no U1/U2 exit latencies to declare.
            0x0c => 0,
            // HCCPARAMS1 (§5.3.6). AC64 = 1: 64-bit pointers are implemented.
            // CSZ = 0: 32-byte contexts. PPC = 0: the ports are hard-wired to
            // power. The extended capability list starts at XECP_OFFSET, in
            // dwords.
            0x10 => 1 | ((XECP_OFFSET as u32 / 4) << 16),
            // DBOFF (§5.3.7) and RTSOFF (§5.3.8).
            0x14 => DB_BASE as u32,
            0x18 => RT_BASE as u32,
            // HCCPARAMS2 (§5.3.9): none of it.
            0x1c => 0,
            _ => 0,
        }
    }

    /// Read the xHCI Supported Protocol extended capability (§7, §7.2).
    ///
    /// One structure, `USB ` 2.0, covering every root port. `PSIC` is zero, so
    /// the default Speed ID mapping of Table 7-13 applies and `PORTSC.Port
    /// Speed` means Full (1), Low (2) or High (3).
    #[must_use]
    pub fn read_xecp(&self, offset: u64) -> u32 {
        match offset {
            // Capability ID 2, no next capability, revision 2.0.
            // Capability ID 2 in bits 7:0, next pointer zero in 15:8, Minor
            // Revision zero in 23:16, Major Revision 2 in 31:24.
            0x00 => 0x02 | (0x02 << 24),
            // The name string, little-endian: 'U', 'S', 'B', ' '.
            0x04 => 0x2042_5355,
            // Compatible Port Offset 1, Compatible Port Count = every port,
            // PSIC = 0.
            0x08 => 1 | (u32::from(self.params.ports) << 8),
            // Protocol Slot Type 0, which §7.2 reserves for USB.
            0x0c => 0,
            _ => 0,
        }
    }

    /// Read an operational register (§5.4), by offset from the operational
    /// base.
    #[must_use]
    pub fn read_op(&self, offset: u64) -> u32 {
        let regs = self.regs.lock();
        match offset {
            0x00 => regs.usbcmd,
            // `CNR` reads zero: this controller is ready as soon as it exists.
            0x04 => regs.usbsts & !STS_CNR,
            // PAGESIZE (§5.4.3): bit 0 set is a 4 KiB page.
            0x08 => 1,
            0x14 => regs.dnctrl,
            // §5.4.5: the pointer, `RCS`, `CS` and `CA` all read zero; only
            // `CRR` is meaningful.
            0x18 => {
                if regs.crr {
                    CRCR_CRR as u32
                } else {
                    0
                }
            }
            0x1c => 0,
            0x30 => regs.dcbaap as u32,
            0x34 => (regs.dcbaap >> 32) as u32,
            0x38 => regs.config,
            _ => {
                if let Some(port) = self.port_at(offset) {
                    return match (offset - PORT_BASE) % PORT_STRIDE {
                        0x0 => regs.portsc.get(usize::from(port)).copied().unwrap_or(0),
                        // PORTPMSC, PORTLI and PORTHLPMC: nothing this model
                        // has anything true to say about, and §5.4.9 through
                        // §5.4.11 make every field of them optional for a USB2
                        // port that implements no link power management.
                        _ => 0,
                    };
                }
                0
            }
        }
    }

    /// Which port an operational-register offset names, if it names one.
    ///
    /// Ports are one-based in the register map (§5.4.8) and zero-based here.
    #[must_use]
    fn port_at(&self, offset: u64) -> Option<u8> {
        if offset < PORT_BASE {
            return None;
        }
        let index = (offset - PORT_BASE) / PORT_STRIDE;
        (index < u64::from(self.params.ports)).then_some(index as u8)
    }

    /// Read a runtime register (§5.5), by offset from the runtime base.
    #[must_use]
    pub fn read_rt(&self, offset: u64) -> u32 {
        let regs = self.regs.lock();
        match offset {
            // MFINDEX (§5.5.1): fourteen bits.
            0x00 => regs.mfindex & 0x3fff,
            _ => {
                if !(IR0_BASE..IR0_BASE + 0x20).contains(&offset) {
                    return 0;
                }
                match offset - IR0_BASE {
                    0x00 => regs.iman,
                    0x04 => (regs.imod & 0xffff) | (regs.imodc << 16),
                    0x08 => regs.erstsz,
                    0x10 => regs.erstba as u32,
                    0x14 => (regs.erstba >> 32) as u32,
                    0x18 => regs.erdp as u32,
                    0x1c => (regs.erdp >> 32) as u32,
                    _ => 0,
                }
            }
        }
    }

    /// Write an operational register, reporting what has to happen once the
    /// lock is released.
    pub fn write_op(&self, offset: u64, value: u32) -> After {
        let mut regs = self.regs.lock();
        match offset {
            0x00 => {
                if value & CMD_HCRST != 0 {
                    // Self-clearing, and it takes everything with it (§5.4.1).
                    return After::Reset;
                }
                let was_running = regs.usbcmd & CMD_RS != 0;
                regs.usbcmd = value & CMD_MASK;
                let running = regs.usbcmd & CMD_RS != 0;
                if running != was_running {
                    if running {
                        regs.usbsts &= !STS_HCH;
                    } else {
                        // §5.4.1.1: the xHC completes what it has and halts.
                        // Everything here completes inside the write that
                        // started it, so "what it has" is nothing.
                        regs.usbsts |= STS_HCH;
                        regs.crr = false;
                    }
                }
                self.publish(&regs);
                regs.arm_interrupt();
                After::Run
            }
            0x04 => {
                // Write-1-to-clear, and only those bits: `HCH`, `CNR` and
                // `HCE` are the controller's (§5.4.2).
                regs.usbsts &= !(value & STS_W1C);
                After::Nothing
            }
            0x14 => {
                regs.dnctrl = value & 0xffff;
                After::Nothing
            }
            0x18 => {
                // §5.4.5: `CS` and `CA` are honoured only while the ring runs,
                // and the pointer and `RCS` only while it does not.
                if regs.crr {
                    if u64::from(value) & (CRCR_CS | CRCR_CA) != 0 {
                        regs.crr = false;
                        regs.cmd_pending = false;
                    }
                } else {
                    regs.crcr_ptr = (regs.crcr_ptr & !0xffff_ffff)
                        | (u64::from(value) & CRCR_PTR & 0xffff_ffff);
                    regs.cmd_dequeue = regs.crcr_ptr;
                    regs.cmd_ccs = u64::from(value) & CRCR_RCS != 0;
                }
                After::Nothing
            }
            0x1c => {
                if !regs.crr {
                    regs.crcr_ptr = (regs.crcr_ptr & 0xffff_ffff) | (u64::from(value) << 32);
                    regs.cmd_dequeue = regs.crcr_ptr;
                }
                After::Nothing
            }
            // §5.4.6: 64-byte aligned.
            0x30 => {
                regs.dcbaap = (regs.dcbaap & !0xffff_ffff) | (u64::from(value) & !0x3f);
                After::Nothing
            }
            0x34 => {
                regs.dcbaap = (regs.dcbaap & 0xffff_ffff) | (u64::from(value) << 32);
                After::Nothing
            }
            0x38 => {
                let slots = (value & CONFIG_SLOTS).min(u32::from(self.params.slots));
                regs.config = slots | (value & (CONFIG_U3E | CONFIG_CIE));
                After::Nothing
            }
            _ => {
                let Some(port) = self.port_at(offset) else {
                    return After::Nothing;
                };
                if !(offset - PORT_BASE).is_multiple_of(PORT_STRIDE) {
                    // PORTPMSC, PORTLI and PORTHLPMC are accepted and dropped.
                    return After::Nothing;
                }
                self.write_portsc(&mut regs, port, value)
            }
        }
    }

    /// `PORTSC` (§5.4.8, Table 5-27), which is four kinds of register at once.
    fn write_portsc(&self, regs: &mut Regs, port: u8, value: u32) -> After {
        let index = usize::from(port);
        let old = regs.portsc[index];

        // Write-1-to-clear the change bits first.
        let mut new = old & !(value & PORT_CHANGE);
        // `PP` is read-write-sticky; `PIC` and the wake enables are plain
        // read-write.
        new = (new & !(PORT_PP | (0x3 << PORT_PIC_SHIFT) | PORT_WAKE))
            | (value & (PORT_PP | (0x3 << PORT_PIC_SHIFT) | PORT_WAKE));
        // `PED` is write-1-to-clear: software may disable a port, never enable
        // one.
        let disabling = value & PORT_PED != 0;
        if disabling {
            new &= !PORT_PED;
        }
        // `PLS` moves only with the Link State Write Strobe.
        if value & PORT_LWS != 0 {
            let pls = (value >> PORT_PLS_SHIFT) & PORT_PLS_MASK;
            // A USB2 port answers U0 and U3; everything else is ignored.
            if pls == PLS_U0 || pls == PLS_U3 {
                new = (new & !(PORT_PLS_MASK << PORT_PLS_SHIFT)) | (pls << PORT_PLS_SHIFT);
            }
        }
        // Over-current is never asserted: a modelled bus has no current.
        new &= !PORT_OCA;
        regs.portsc[index] = new;
        // §4.19.2: `PSCEG` is the OR of the change bits, so acknowledging one
        // lowers it. Tracking that here is what makes the *next* change a
        // rising edge — a controller that only sampled `PSCEG` when it set a
        // bit would report the first port event and never another.
        regs.psceg[index] = new & PORT_CHANGE != 0;

        // `PR` is write-1-to-set, and a zero-to-one transition is what starts
        // the reset (§5.4.8, footnote 83: setting it when it is already set is
        // ignored).
        if value & PORT_PR != 0 && old & PORT_PR == 0 && new & PORT_PP != 0 {
            regs.portsc[index] |= PORT_PR;
            return After::Port(port);
        }
        if disabling {
            return After::Port(port);
        }
        After::Nothing
    }

    /// Write a runtime register (§5.5).
    pub fn write_rt(&self, offset: u64, value: u32) -> After {
        let mut regs = self.regs.lock();
        if !(IR0_BASE..IR0_BASE + 0x20).contains(&offset) {
            // MFINDEX is read-only (§5.5.1) and everything else here is RsvdZ.
            return After::Nothing;
        }
        match offset - IR0_BASE {
            0x00 => {
                // §5.5.2.1: `IP` is write-1-to-clear and `IE` is read-write.
                let clearing = value & IMAN_IP != 0 && regs.iman & IMAN_IP != 0;
                regs.iman = (regs.iman & !IMAN_IE) | (value & IMAN_IE);
                if clearing {
                    regs.iman &= !IMAN_IP;
                    // §5.5.2.2: `IMODC` is loaded with `IMODI` whenever `IP` is
                    // cleared.
                    regs.imodc = regs.imod & 0xffff;
                }
                regs.arm_interrupt();
                After::Nothing
            }
            0x04 => {
                regs.imod = value & 0xffff;
                regs.imodc = value >> 16;
                regs.arm_interrupt();
                After::Nothing
            }
            0x08 => {
                regs.erstsz = value & 0xffff;
                After::Nothing
            }
            // §5.5.2.3.2: writing this puts the Event Ring State Machine in the
            // Start state, which needs a guest read — so it happens outside the
            // lock.
            0x10 => {
                regs.erstba = (regs.erstba & !0xffff_ffff) | (u64::from(value) & !0x3f);
                After::EventRing
            }
            0x14 => {
                regs.erstba = (regs.erstba & 0xffff_ffff) | (u64::from(value) << 32);
                After::EventRing
            }
            0x18 => {
                // §5.5.2.3.3: `EHB` is write-1-to-clear, `DESI` is a hint, and
                // the rest is where software has read to.
                let clearing = u64::from(value) & ERDP_EHB != 0;
                let held = regs.erdp & ERDP_EHB;
                regs.erdp = (regs.erdp & !0xffff_ffff)
                    | (u64::from(value) & (ERDP_PTR | ERDP_DESI) & 0xffff_ffff);
                if !clearing {
                    regs.erdp |= held;
                }
                self.after_erdp(&mut regs)
            }
            0x1c => {
                regs.erdp = (regs.erdp & 0xffff_ffff) | (u64::from(value) << 32);
                self.after_erdp(&mut regs)
            }
            _ => After::Nothing,
        }
    }

    /// What a write to `ERDP` frees up.
    fn after_erdp(&self, regs: &mut Regs) -> After {
        if regs.er_full && regs.next_enqueue_addr() != regs.erdp & ERDP_PTR {
            // §4.9.4 step 17: the ring stays full until software writes the
            // ERDP, and the controller resumes when it does.
            regs.er_full = false;
        }
        regs.arm_interrupt();
        After::Run
    }

    /// A doorbell write (§5.6).
    ///
    /// Register 0 is the command ring; register *n* is device slot *n*, and its
    /// `DB Target` is the Device Context Index of the endpoint that has work.
    pub fn write_doorbell(&self, index: u64, value: u32) -> After {
        let target = value & 0xff;
        let mut regs = self.regs.lock();
        if !regs.running() {
            // §5.4.2: a halted controller generates nothing on the bus.
            return After::Nothing;
        }
        if index == 0 {
            if target != 0 {
                // §5.6: every other target on register 0 is reserved.
                return After::Nothing;
            }
            regs.crr = true;
            regs.cmd_pending = true;
            return After::Run;
        }
        let slot = index as usize;
        // §5.4.7: a disabled Device Slot does not respond to doorbell
        // references — and `index` came off the address bus, so it may name a
        // slot this controller could never have.
        let Some(bit) = u8::try_from(index).ok().and_then(slot_bit) else {
            return After::Nothing;
        };
        if slot > usize::from(self.params.slots)
            || u64::from(regs.slots_enabled()) < index
            || regs.slot_enabled & bit == 0
        {
            return After::Nothing;
        }
        if target == 0 || target > MAX_DCI {
            return After::Nothing;
        }
        regs.ep_pending[slot] |= 1 << target;
        After::Run
    }

    /// Perform whatever a register write asked for, with no lock held.
    pub fn act(&self, after: After) {
        match after {
            After::Nothing => {}
            After::Reset => self.controller_reset(),
            After::Port(port) => self.settle_port(port, true),
            After::EventRing => {
                if let Some(space) = self.space() {
                    self.start_event_ring(&space);
                }
            }
            // Every write ends with a run anyway; this arm exists so that
            // "something became runnable" is a thing a register handler can
            // say rather than a thing the caller has to infer.
            After::Run => {}
        }
        self.run();
        self.refresh_irq();
    }

    /// `USBCMD.HCRST` (§5.4.1): everything back to its reset value.
    fn controller_reset(&self) {
        {
            let mut regs = self.regs.lock();
            let ticks = regs.ticks;
            *regs = Regs {
                ticks,
                ..Regs::reset(self.params.ports)
            };
            self.publish(&regs);
        }
        for port in 0..self.params.ports {
            self.bus.set_enabled(port, false);
        }
        for port in 0..self.params.ports {
            self.settle_port(port, false);
        }
    }

    /// Bring one port's `PORTSC`, the fabric and the device behind it into
    /// agreement, and post a Port Status Change Event if that is what §4.19.2
    /// asks for.
    ///
    /// `drive_reset` is true when software has just set `PR`: unlike EHCI, the
    /// xHC times the reset itself and clears `PR` when it is done (§5.4.8), so
    /// there is no second write for software to make.
    fn settle_port(&self, port: u8, drive_reset: bool) {
        let index = usize::from(port);
        if index >= usize::from(self.params.ports) {
            return;
        }

        // What the port looks like from the fabric's side. Read outside our own
        // lock: `speed` is a call into the device.
        let connected = self.bus.connected(port);
        let plugged_changed = self.bus.take_change(port);
        let resetting = drive_reset && connected && self.portsc(port) & PORT_PR != 0;
        if resetting {
            // USB 2.0 §7.1.7.5, driven by §4.19.1.1's Reset state.
            self.bus.reset_port(port);
        }
        let speed = self.bus.speed(port);

        let (enable, changed) = {
            let mut regs = self.regs.lock();
            let mut sc = regs.portsc[index];
            let before = sc & PORT_CHANGE;

            if plugged_changed {
                sc |= PORT_CSC;
            }
            sc = (sc & !PORT_CCS) | if connected { PORT_CCS } else { 0 };

            let mut enable = false;
            if !connected {
                if sc & PORT_PED != 0 {
                    // §5.4.8: `PED` is cleared by a disconnect, and for a USB2
                    // port `PEC` is set only by a Port Error, not by this.
                    sc &= !PORT_PED;
                }
                sc &= !PORT_PR;
                sc = (sc & !(PORT_PLS_MASK << PORT_PLS_SHIFT)) | (PLS_RXDETECT << PORT_PLS_SHIFT);
            } else if resetting {
                // The reset finished. §5.4.8: `PR` clears, `PED` is set on a
                // successful reset, and `PRC` records the one-to-zero
                // transition of `PR`.
                sc &= !PORT_PR;
                sc |= PORT_PED | PORT_PRC;
                sc = (sc & !(PORT_PLS_MASK << PORT_PLS_SHIFT)) | (PLS_U0 << PORT_PLS_SHIFT);
                // §5.4.8: the Port Speed field is invalid on a USB2 port until
                // after the port is reset, so this is the moment it becomes
                // true.
                sc &= !(PORT_SPEED_MASK << PORT_SPEED_SHIFT);
                sc |= speed_id(speed) << PORT_SPEED_SHIFT;
                enable = true;
            } else if sc & PORT_PED == 0 {
                // Attached and not yet reset: §4.19.1.1 calls this Polling, and
                // it is what tells software to write `PR`.
                sc = (sc & !(PORT_PLS_MASK << PORT_PLS_SHIFT)) | (PLS_POLLING << PORT_PLS_SHIFT);
                sc &= !(PORT_SPEED_MASK << PORT_SPEED_SHIFT);
            } else {
                enable = true;
            }
            sc |= PORT_PP;
            regs.portsc[index] = sc;

            let after = sc & PORT_CHANGE;
            // §4.19.2: the event is the rising edge of `PSCEG`, which is the OR
            // of the change bits — not one event per bit.
            let was = regs.psceg[index];
            let now = after != 0;
            regs.psceg[index] = now;
            if now && !was {
                regs.usbsts |= STS_PCD;
            }
            let _ = before;
            (enable, now && !was && regs.running())
        };

        self.bus.set_enabled(port, enable);

        if changed && let Some(space) = self.space() {
            // Ports are one-based in a Port Status Change Event (§6.4.2.3).
            self.port_event(&space, port + 1);
        }
    }

    // -----------------------------------------------------------------
    // Time
    // -----------------------------------------------------------------

    /// Simulate forward to `target` domain ticks.
    ///
    /// Runs with **no lock held across an outward call**: each microframe
    /// decides what to do under the register lock, releases it, then reaches
    /// guest memory and the USB fabric.
    pub fn advance_to(&self, target: u64) {
        loop {
            {
                let mut regs = self.regs.lock();
                if !regs.running() {
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
        let wrapped = {
            let mut regs = self.regs.lock();
            // §5.5.1: fourteen bits, incremented once per microframe.
            let next = (regs.mfindex + 1) & 0x3fff;
            let wrapped = next == 0;
            regs.mfindex = next;
            // §5.5.2.2: the counter is in 250 ns units, and a microframe is
            // 125 µs — exactly 500 of them.
            regs.imodc = regs.imodc.saturating_sub(IMOD_TICKS_PER_MICROFRAME);
            // An endpoint that answered `NAK` gets another go.
            for slot in 1..=MAX_SLOTS {
                let retry = core::mem::take(&mut regs.ep_retry[slot]);
                regs.ep_pending[slot] |= retry;
            }
            regs.arm_interrupt();
            wrapped && regs.usbcmd & CMD_EWE != 0
        };

        // Outside the lock: a question for the fabric is an outward call, even
        // when the fabric answers it from an atomic (`core::device`, the
        // re-entrancy contract).
        if self.bus.any_change() {
            for port in 0..self.params.ports {
                self.settle_port(port, false);
            }
        }

        if wrapped && let Some(space) = self.space() {
            // §5.4.1 bit 10: an MFINDEX Wrap Event, when software asked for one.
            let trb = [
                0,
                0,
                code::SUCCESS << 24,
                trb::MFINDEX_WRAP_EVENT << TRB_TYPE_SHIFT,
            ];
            self.post_event(&space, trb);
        }

        self.run();
        self.refresh_irq();
    }

    // -----------------------------------------------------------------
    // The engine
    // -----------------------------------------------------------------

    /// Whether anything is waiting to be executed.
    fn has_work(&self) -> bool {
        let regs = self.regs.lock();
        if !regs.running() || regs.er_full {
            return false;
        }
        regs.cmd_pending || regs.ep_pending.iter().any(|bits| *bits != 0)
    }

    /// Take the next work item, in a fixed order: the command ring first, then
    /// slots in index order and endpoints in Device Context Index order.
    ///
    /// Fixed rather than round-robin because guest-visible order must not
    /// depend on anything but the guest (`CLAUDE.md`, determinism).
    fn next_job(&self) -> Option<Job> {
        let mut regs = self.regs.lock();
        if !regs.running() || regs.er_full {
            return None;
        }
        if regs.cmd_pending {
            regs.cmd_pending = false;
            return Some(Job::Command);
        }
        for slot in 1..=MAX_SLOTS {
            let bits = regs.ep_pending[slot];
            if bits == 0 {
                continue;
            }
            let dci = bits.trailing_zeros();
            regs.ep_pending[slot] &= !(1 << dci);
            return Some(Job::Transfer(slot as u8, dci));
        }
        None
    }

    /// Run everything the driver has made available.
    ///
    /// **Iterative, not recursive** (module docs). A doorbell rung from inside
    /// one of this controller's own guest-memory accesses records its work and
    /// returns; the loop below picks it up.
    pub fn run(&self) {
        if self.busy.swap(true, Ordering::AcqRel) {
            return;
        }
        let Some(space) = self.space() else {
            self.busy.store(false, Ordering::Release);
            return;
        };
        let mut budget = MAX_WORK_ITEMS;
        loop {
            while let Some(job) = self.next_job() {
                match job {
                    Job::Command => self.run_command(&space),
                    Job::Transfer(slot, dci) => self.run_transfer(&space, slot, dci),
                }
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

    // -----------------------------------------------------------------
    // Rings
    // -----------------------------------------------------------------

    /// Fetch the TRB at `*dequeue`, following Link TRBs (§6.4.4.1).
    ///
    /// Leaves `*dequeue` pointing *at* the returned TRB, so the caller consumes
    /// it with [`Xhci::consume`]. `None` means the ring is empty — the Cycle bit
    /// disagreed — or the walk ran out of link hops, which is the same thing
    /// from the caller's point of view: stop.
    fn fetch(
        &self,
        space: &AddressSpace,
        dequeue: &mut u64,
        ccs: &mut bool,
    ) -> Option<(u64, [u32; 4])> {
        for _ in 0..MAX_LINK_HOPS {
            let addr = *dequeue & !0xf;
            let trb = self.read_trb(space, addr)?;
            if (trb[3] & TRB_CYCLE != 0) != *ccs {
                return None;
            }
            if (trb[3] >> TRB_TYPE_SHIFT) & TRB_TYPE_MASK != trb::LINK {
                *dequeue = addr;
                return Some((addr, trb));
            }
            // §6.4.4.1: follow the segment pointer, and toggle the cycle state
            // if `TC` says to.
            *dequeue = (u64::from(trb[0]) | (u64::from(trb[1]) << 32)) & !0xf;
            if trb[3] & TRB_TC != 0 {
                *ccs = !*ccs;
            }
        }
        // A ring made of Link TRBs. Bounded, and the caller simply stops.
        None
    }

    /// Step past the TRB [`Xhci::fetch`] returned.
    fn consume(dequeue: &mut u64) {
        *dequeue = dequeue.wrapping_add(TRB_BYTES);
    }

    // -----------------------------------------------------------------
    // Commands (§4.6)
    // -----------------------------------------------------------------

    /// Fetch and execute one command TRB.
    fn run_command(&self, space: &AddressSpace) {
        let (mut dequeue, mut ccs) = {
            let regs = self.regs.lock();
            if !regs.crr {
                return;
            }
            (regs.cmd_dequeue, regs.cmd_ccs)
        };
        let Some((addr, trb)) = self.fetch(space, &mut dequeue, &mut ccs) else {
            // The ring is empty. `CRR` stays set: §5.4.5 clears it only on a
            // stop, an abort, or `R/S` going to zero.
            let mut regs = self.regs.lock();
            regs.cmd_dequeue = dequeue;
            regs.cmd_ccs = ccs;
            return;
        };
        Xhci::consume(&mut dequeue);
        {
            let mut regs = self.regs.lock();
            regs.cmd_dequeue = dequeue;
            regs.cmd_ccs = ccs;
            // Another command may be sitting behind this one; the run loop's
            // budget is what bounds the chain.
            regs.cmd_pending = true;
        }

        let kind = (trb[3] >> TRB_TYPE_SHIFT) & TRB_TYPE_MASK;
        let slot = (trb[3] >> 24) as u8;
        let input = (u64::from(trb[0]) | (u64::from(trb[1]) << 32)) & !0xf;

        let (completion, reported_slot) = match kind {
            trb::NO_OP_COMMAND => (code::SUCCESS, 0),
            trb::ENABLE_SLOT => self.cmd_enable_slot(),
            trb::DISABLE_SLOT => (self.cmd_disable_slot(space, slot), slot),
            trb::ADDRESS_DEVICE => (
                self.cmd_address_device(space, slot, input, trb[3] & TRB_BSR != 0),
                slot,
            ),
            trb::CONFIGURE_ENDPOINT => (
                self.cmd_configure_endpoint(space, slot, input, trb[3] & TRB_DC != 0),
                slot,
            ),
            trb::EVALUATE_CONTEXT => (self.cmd_evaluate_context(space, slot, input), slot),
            trb::RESET_ENDPOINT => (
                self.cmd_endpoint_state(space, slot, (trb[3] >> 16) & 0x1f, EP_STATE_STOPPED),
                slot,
            ),
            trb::STOP_ENDPOINT => (
                self.cmd_endpoint_state(space, slot, (trb[3] >> 16) & 0x1f, EP_STATE_STOPPED),
                slot,
            ),
            trb::RESET_DEVICE => (self.cmd_reset_device(space, slot), slot),
            trb::SET_TR_DEQUEUE => (
                self.cmd_set_tr_dequeue(space, slot, (trb[3] >> 16) & 0x1f, trb[0], trb[1]),
                slot,
            ),
            // §6.4.6: anything else on a command ring is a TRB Error with the
            // Slot ID cleared.
            _ => (code::TRB_ERROR, 0),
        };

        self.command_event(space, addr, completion, reported_slot);
    }

    /// Enable Slot (§4.6.3): allocate the lowest free Device Slot.
    fn cmd_enable_slot(&self) -> (u32, u8) {
        let mut regs = self.regs.lock();
        let limit = regs.slots_enabled().min(u32::from(self.params.slots));
        for slot in 1..=limit {
            let Some(bit) = u8::try_from(slot).ok().and_then(slot_bit) else {
                break;
            };
            if regs.slot_enabled & bit == 0 {
                regs.slot_enabled |= bit;
                return (code::SUCCESS, slot as u8);
            }
        }
        // §6.4.5 code 9.
        (code::NO_SLOTS_AVAILABLE, 0)
    }

    /// Disable Slot (§4.6.4).
    fn cmd_disable_slot(&self, space: &AddressSpace, slot: u8) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        if let Some(base) = self.device_context(space, slot)
            && let Some(mut ctx) = self.read_context(space, base)
        {
            // §6.2.2 Table 6-7: the Slot State goes back to Disabled/Enabled
            // and the USB Device Address with it. Ownership of the Device
            // Context returns to software (§6.2.1).
            ctx[3] = SLOT_STATE_ENABLED << SLOT_STATE_SHIFT;
            let _ = self.write_context(space, base, &ctx);
        }
        let mut regs = self.regs.lock();
        regs.slot_enabled &= !slot_bit(slot).unwrap_or(0);
        regs.ep_pending[usize::from(slot)] = 0;
        regs.ep_retry[usize::from(slot)] = 0;
        code::SUCCESS
    }

    /// Whether `slot` names an enabled Device Slot.
    ///
    /// Every command that carries a Slot ID starts here, because that field
    /// came out of a TRB in guest memory: §6.4.5 code 11 is the answer for a
    /// slot that is not enabled, and [`slot_bit`] is what makes "not a Slot ID
    /// at all" the same answer rather than a shift overflow.
    fn check_slot(&self, slot: u8) -> bool {
        slot_bit(slot).is_some_and(|bit| self.regs.lock().slot_enabled & bit != 0)
    }

    /// Address Device (§4.6.5).
    ///
    /// Enables the Default Control Endpoint, selects an address, and — unless
    /// `bsr` blocks it — issues a `SET_ADDRESS` to address zero, which is where
    /// a USB2 device answers until the status stage of that very request
    /// completes (§4.6.5, USB 2.0 §9.4.6).
    fn cmd_address_device(&self, space: &AddressSpace, slot: u8, input: u64, bsr: bool) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let Some(icc) = self.read_context(space, input) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        // §4.6.5: A0 and A1 shall both be set, and no other flag.
        if icc[1] & 0x3 != 0x3 {
            return code::PARAMETER_ERROR;
        }
        let Some(slot_ctx) = self.read_context(space, input + CONTEXT_BYTES) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        let Some(mut ep0) = self.read_context(space, input + 2 * CONTEXT_BYTES) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        if let Err(code) = check_endpoint(&ep0, EP_TYPE_CONTROL) {
            return code;
        }

        // §6.2.2 Table 6-5: ports are one-based here and zero-based on the
        // fabric.
        let port_number = (slot_ctx[1] >> SLOT_PORT_SHIFT) & 0xff;
        if port_number == 0 || port_number > u32::from(self.params.ports) {
            return code::PARAMETER_ERROR;
        }
        let port = (port_number - 1) as u8;
        if !self.bus.enabled(port) {
            // §4.6.5: a device that cannot be reached is a transaction error,
            // not a silently addressed slot.
            return code::USB_TRANSACTION_ERROR;
        }

        // The xHC selects the address (§4.6.5). The Slot ID is the obvious
        // choice and is unique by construction.
        let address = DeviceAddress(slot);
        if !bsr && !self.set_address(address) {
            return code::USB_TRANSACTION_ERROR;
        }

        // §6.2.2, §6.2.3: the Output contexts are the xHC's report of what it
        // is using, so they are the Input contexts plus the state it just
        // reached.
        let mut out_slot = slot_ctx;
        // §6.2.2: as Output, every field reflects what the xHC is *using* —
        // including the speed, which the port has just latched at reset and
        // which software may have guessed wrong.
        out_slot[0] = (out_slot[0] & !(0xf << SLOT_SPEED_SHIFT))
            | (speed_id(self.bus.speed(port)) << SLOT_SPEED_SHIFT);
        out_slot[3] = (out_slot[3] & !0xff) | u32::from(if bsr { 0 } else { address.0 });
        out_slot[3] = (out_slot[3] & !(0x1f << SLOT_STATE_SHIFT))
            | (if bsr {
                SLOT_STATE_DEFAULT
            } else {
                SLOT_STATE_ADDRESSED
            } << SLOT_STATE_SHIFT);
        // §4.5.2: only the Default Control Endpoint is enabled by this command.
        out_slot[0] = (out_slot[0] & !(0x1f << SLOT_ENTRIES_SHIFT)) | (1 << SLOT_ENTRIES_SHIFT);
        ep0[0] = (ep0[0] & !EP_STATE_MASK) | EP_STATE_RUNNING;

        if self.write_context(space, output, &out_slot).is_none()
            || self
                .write_context(space, output + CONTEXT_BYTES, &ep0)
                .is_none()
        {
            self.host_system_error();
            return code::TRB_ERROR;
        }
        code::SUCCESS
    }

    /// Drive a `SET_ADDRESS` through the fabric, a transaction at a time.
    ///
    /// [`host::ControlTransfer`] is the host-side composer [`crate::bus::usb`]
    /// already has; the budget is what stops a device that only ever `NAK`s.
    fn set_address(&self, address: DeviceAddress) -> bool {
        let mut transfer = host::ControlTransfer::host_to_device(host::set_address(address), &[]);
        for _ in 0..MAX_CONTROL_STEPS {
            match transfer.step(&self.bus, DeviceAddress::DEFAULT, 8) {
                host::Progress::Done => return true,
                host::Progress::Failed(_) => return false,
                host::Progress::Moved | host::Progress::Nak => {}
            }
        }
        false
    }

    /// Configure Endpoint (§4.6.6).
    fn cmd_configure_endpoint(
        &self,
        space: &AddressSpace,
        slot: u8,
        input: u64,
        deconfigure: bool,
    ) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let Some(mut out_slot) = self.read_context(space, output) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        if deconfigure {
            // §6.4.3.5: the Input Context Pointer is ignored, every endpoint but
            // the default control pipe is disabled, and the slot goes back to
            // Addressed.
            for dci in 2..=MAX_DCI {
                let zero = [0u32; CONTEXT_DWORDS];
                if self
                    .write_context(space, output + u64::from(dci) * CONTEXT_BYTES, &zero)
                    .is_none()
                {
                    self.host_system_error();
                    return code::TRB_ERROR;
                }
            }
            out_slot[0] = (out_slot[0] & !(0x1f << SLOT_ENTRIES_SHIFT)) | (1 << SLOT_ENTRIES_SHIFT);
            out_slot[3] = (out_slot[3] & !(0x1f << SLOT_STATE_SHIFT))
                | (SLOT_STATE_ADDRESSED << SLOT_STATE_SHIFT);
            let _ = self.write_context(space, output, &out_slot);
            return code::SUCCESS;
        }

        let Some(icc) = self.read_context(space, input) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        let drop = icc[0] & !0x3;
        let add = icc[1];
        // §4.6.6: a context may not be both added and dropped.
        if drop & add != 0 {
            return code::PARAMETER_ERROR;
        }

        for dci in 2..=MAX_DCI {
            let bit = 1u32 << dci;
            let out_addr = output + u64::from(dci) * CONTEXT_BYTES;
            if drop & bit != 0 {
                let zero = [0u32; CONTEXT_DWORDS];
                if self.write_context(space, out_addr, &zero).is_none() {
                    self.host_system_error();
                    return code::TRB_ERROR;
                }
            }
            if add & bit != 0 {
                let Some(mut ep) =
                    self.read_context(space, input + u64::from(dci + 1) * CONTEXT_BYTES)
                else {
                    self.host_system_error();
                    return code::TRB_ERROR;
                };
                // Any *valid* endpoint type is acceptable to this command, so
                // the context is checked against its own type: what the check
                // rejects here is a Not Valid type, a zero Max Packet Size and
                // a request for streams (§6.2.3).
                if let Err(code) = check_endpoint(&ep, (ep[1] >> EP_TYPE_SHIFT) & 0x7) {
                    return code;
                }
                ep[0] = (ep[0] & !EP_STATE_MASK) | EP_STATE_RUNNING;
                if self.write_context(space, out_addr, &ep).is_none() {
                    self.host_system_error();
                    return code::TRB_ERROR;
                }
            }
        }

        if add & 1 != 0 {
            let Some(in_slot) = self.read_context(space, input + CONTEXT_BYTES) else {
                self.host_system_error();
                return code::TRB_ERROR;
            };
            out_slot[0] = (out_slot[0] & !(0x1f << SLOT_ENTRIES_SHIFT))
                | (in_slot[0] & (0x1f << SLOT_ENTRIES_SHIFT));
        }
        out_slot[3] = (out_slot[3] & !(0x1f << SLOT_STATE_SHIFT))
            | (SLOT_STATE_CONFIGURED << SLOT_STATE_SHIFT);
        if self.write_context(space, output, &out_slot).is_none() {
            self.host_system_error();
            return code::TRB_ERROR;
        }
        code::SUCCESS
    }

    /// Evaluate Context (§4.6.7).
    ///
    /// §6.2.2.3 and §6.2.3.3 name exactly which fields this command looks at:
    /// Max Exit Latency and Interrupter Target in the Slot Context, and Max
    /// Packet Size in an Endpoint Context. Everything else in the Input Context
    /// is ignored, which is why this is not a wholesale copy.
    fn cmd_evaluate_context(&self, space: &AddressSpace, slot: u8, input: u64) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let Some(icc) = self.read_context(space, input) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        let add = icc[1];
        if add & 1 != 0 {
            let (Some(in_slot), Some(mut out_slot)) = (
                self.read_context(space, input + CONTEXT_BYTES),
                self.read_context(space, output),
            ) else {
                self.host_system_error();
                return code::TRB_ERROR;
            };
            // Max Exit Latency, bits 15:0 of dword 1 (§6.2.2 Table 6-5).
            out_slot[1] = (out_slot[1] & !0xffff) | (in_slot[1] & 0xffff);
            // Interrupter Target, bits 31:22 of dword 2 (Table 6-6).
            out_slot[2] = (out_slot[2] & !0xffc0_0000) | (in_slot[2] & 0xffc0_0000);
            if self.write_context(space, output, &out_slot).is_none() {
                self.host_system_error();
                return code::TRB_ERROR;
            }
        }
        for dci in 1..=MAX_DCI {
            if add & (1 << dci) == 0 {
                continue;
            }
            let out_addr = output + u64::from(dci) * CONTEXT_BYTES;
            let (Some(in_ep), Some(mut out_ep)) = (
                self.read_context(space, input + u64::from(dci + 1) * CONTEXT_BYTES),
                self.read_context(space, out_addr),
            ) else {
                self.host_system_error();
                return code::TRB_ERROR;
            };
            // Max Packet Size, bits 31:16 of dword 1 (§6.2.3 Table 6-9).
            out_ep[1] = (out_ep[1] & 0xffff) | (in_ep[1] & 0xffff_0000);
            if self.write_context(space, out_addr, &out_ep).is_none() {
                self.host_system_error();
                return code::TRB_ERROR;
            }
        }
        code::SUCCESS
    }

    /// Reset Endpoint (§4.6.8) and Stop Endpoint (§4.6.9), which differ only in
    /// which state they are legal from.
    fn cmd_endpoint_state(&self, space: &AddressSpace, slot: u8, dci: u32, state: u32) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        if dci == 0 || dci > MAX_DCI {
            return code::TRB_ERROR;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let addr = output + u64::from(dci) * CONTEXT_BYTES;
        let Some(mut ep) = self.read_context(space, addr) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        if (ep[1] >> EP_TYPE_SHIFT) & 0x7 == EP_TYPE_INVALID {
            return code::CONTEXT_STATE_ERROR;
        }
        ep[0] = (ep[0] & !EP_STATE_MASK) | state;
        if self.write_context(space, addr, &ep).is_none() {
            self.host_system_error();
            return code::TRB_ERROR;
        }
        let mut regs = self.regs.lock();
        regs.ep_pending[usize::from(slot)] &= !(1 << dci);
        regs.ep_retry[usize::from(slot)] &= !(1 << dci);
        code::SUCCESS
    }

    /// Set TR Dequeue Pointer (§4.6.10, §6.4.3.9).
    fn cmd_set_tr_dequeue(
        &self,
        space: &AddressSpace,
        slot: u8,
        dci: u32,
        lo: u32,
        hi: u32,
    ) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        if dci == 0 || dci > MAX_DCI {
            return code::TRB_ERROR;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let addr = output + u64::from(dci) * CONTEXT_BYTES;
        let Some(mut ep) = self.read_context(space, addr) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        // §6.4.3.9 note: legal only from the Stopped or Error state.
        let state = ep[0] & EP_STATE_MASK;
        if state != EP_STATE_STOPPED && state != EP_STATE_ERROR && state != EP_STATE_HALTED {
            return code::CONTEXT_STATE_ERROR;
        }
        // §6.4.3.9: bits 63:4 are the new pointer and bit 0 the Dequeue Cycle
        // State; bits 3:1 are the Stream Context Type, and this controller has
        // no streams, so they are dropped rather than stored.
        ep[2] = (lo & !0xf) | (lo & EP_DCS);
        ep[3] = hi;
        if self.write_context(space, addr, &ep).is_none() {
            self.host_system_error();
            return code::TRB_ERROR;
        }
        code::SUCCESS
    }

    /// Reset Device (§4.6.11): back to the Default state, address zero, every
    /// endpoint but the default control pipe disabled.
    fn cmd_reset_device(&self, space: &AddressSpace, slot: u8) -> u32 {
        if !self.check_slot(slot) {
            return code::SLOT_NOT_ENABLED;
        }
        let Some(output) = self.device_context(space, slot) else {
            return code::PARAMETER_ERROR;
        };
        let Some(mut out_slot) = self.read_context(space, output) else {
            self.host_system_error();
            return code::TRB_ERROR;
        };
        for dci in 2..=MAX_DCI {
            let zero = [0u32; CONTEXT_DWORDS];
            if self
                .write_context(space, output + u64::from(dci) * CONTEXT_BYTES, &zero)
                .is_none()
            {
                self.host_system_error();
                return code::TRB_ERROR;
            }
        }
        out_slot[3] = 0;
        out_slot[3] |= SLOT_STATE_DEFAULT << SLOT_STATE_SHIFT;
        out_slot[0] = (out_slot[0] & !(0x1f << SLOT_ENTRIES_SHIFT)) | (1 << SLOT_ENTRIES_SHIFT);
        if self.write_context(space, output, &out_slot).is_none() {
            self.host_system_error();
            return code::TRB_ERROR;
        }
        let mut regs = self.regs.lock();
        regs.ep_pending[usize::from(slot)] = 0;
        regs.ep_retry[usize::from(slot)] = 0;
        code::SUCCESS
    }

    // -----------------------------------------------------------------
    // Transfers (§4.11)
    // -----------------------------------------------------------------

    /// Execute one Transfer Descriptor on `(slot, dci)`.
    fn run_transfer(&self, space: &AddressSpace, slot: u8, dci: u32) {
        let Some(output) = self.device_context(space, slot) else {
            return;
        };
        let Some(slot_ctx) = self.read_context(space, output) else {
            self.host_system_error();
            return;
        };
        let ep_addr = output + u64::from(dci) * CONTEXT_BYTES;
        let Some(mut ep) = self.read_context(space, ep_addr) else {
            self.host_system_error();
            return;
        };

        let ep_type = (ep[1] >> EP_TYPE_SHIFT) & 0x7;
        if ep_type == EP_TYPE_INVALID {
            // §6.4.5 code 12: a doorbell for an endpoint that is not enabled.
            self.transfer_event(
                space,
                (0, false),
                (0, code::ENDPOINT_NOT_ENABLED),
                (slot, dci),
            );
            return;
        }
        if ep[0] & EP_STATE_MASK == EP_STATE_DISABLED {
            self.transfer_event(
                space,
                (0, false),
                (0, code::ENDPOINT_NOT_ENABLED),
                (slot, dci),
            );
            return;
        }
        if ep[0] & EP_STATE_MASK != EP_STATE_RUNNING {
            // Halted, Stopped or Error: software has to recover it first
            // (§4.8.3). The doorbell is dropped rather than obeyed.
            return;
        }

        let address = DeviceAddress((slot_ctx[3] & 0xff) as u8);
        let endpoint = if dci == 1 {
            0
        } else {
            ((dci >> 1) & 0xf) as u8
        };
        // An endpoint context that named a zero maximum packet size is a driver
        // bug; one byte is the only forward progress available and is better
        // than dividing by zero.
        let mps = ((ep[1] >> EP_MPS_SHIFT) & 0xffff).max(1);
        // §6.2.3 Table 6-9: types 1-3 are OUT and 5-7 are IN; type 4 is control
        // and takes its direction from each TRB.
        let ep_in = ep_type >= 5;

        let start_dequeue = (u64::from(ep[2]) | (u64::from(ep[3]) << 32)) & !0xf;
        let start_ccs = ep[2] & EP_DCS != 0;
        let mut dequeue = start_dequeue;
        let mut ccs = start_ccs;

        let mut halted = false;
        let mut naked = false;
        let mut more = false;

        'td: for step in 0..MAX_TRBS_PER_TD {
            let Some((addr, trb)) = self.fetch(space, &mut dequeue, &mut ccs) else {
                break;
            };
            let kind = (trb[3] >> TRB_TYPE_SHIFT) & TRB_TYPE_MASK;
            let chain = trb[3] & TRB_CH != 0;
            let ioc = trb[3] & TRB_IOC != 0;
            let bei = trb[3] & TRB_BEI != 0;
            let _ = trb[3] & TRB_ENT;

            let outcome = match kind {
                trb::SETUP_STAGE => self.do_setup(&trb, address, endpoint),
                trb::DATA_STAGE | trb::NORMAL => {
                    let dir = if kind == trb::DATA_STAGE {
                        trb[3] & TRB_DIR != 0
                    } else if ep_type == EP_TYPE_CONTROL {
                        // A Normal TRB continuing a control Data Stage TD keeps
                        // the stage's direction, which the chain it is part of
                        // established (§4.11.2.2). With no stage to inherit
                        // from, an IN is the only safe reading.
                        true
                    } else {
                        ep_in
                    };
                    self.do_data(space, &trb, address, endpoint, dir, mps)
                }
                trb::STATUS_STAGE => {
                    let dir = trb[3] & TRB_DIR != 0;
                    self.do_status(address, endpoint, dir)
                }
                trb::NO_OP => Outcome::ok(0, 0),
                trb::EVENT_DATA => {
                    // §6.4.4.2: a software-defined event carrying the TRB's
                    // parameter.
                    if ioc {
                        let param = u64::from(trb[0]) | (u64::from(trb[1]) << 32);
                        if !self.transfer_event(
                            space,
                            (param, true),
                            (0, code::SUCCESS),
                            (slot, dci),
                        ) {
                            break 'td;
                        }
                    }
                    Xhci::consume(&mut dequeue);
                    if chain {
                        continue 'td;
                    }
                    more = true;
                    break 'td;
                }
                trb::ISOCH => {
                    // Recognised and refused rather than half-implemented.
                    Outcome::failed(code::TRB_ERROR)
                }
                _ => Outcome::failed(code::TRB_ERROR),
            };

            if outcome.nak {
                naked = true;
                break 'td;
            }
            Xhci::consume(&mut dequeue);

            let last = !chain;
            if outcome.completion != code::SUCCESS {
                // §4.10.1: an error retires the TD and generates an event even
                // if `IOC` is clear.
                if outcome.completion == code::STALL_ERROR
                    || outcome.completion == code::USB_TRANSACTION_ERROR
                    || outcome.completion == code::BABBLE
                {
                    halted = true;
                }
                self.transfer_event(
                    space,
                    (addr, false),
                    (outcome.residual, outcome.completion),
                    (slot, dci),
                );
                break 'td;
            }
            if outcome.short && (trb[3] & TRB_ISP != 0 || ioc) {
                // §6.4.1.1 bit 2: a short packet retires the TRB without error
                // and the controller advances to the next TD; if `ISP` and
                // `IOC` are both set only one event is queued.
                if !self.transfer_event(
                    space,
                    (addr, false),
                    (outcome.residual, code::SHORT_PACKET),
                    (slot, dci),
                ) {
                    break 'td;
                }
                // Skip the rest of the chain: it belongs to a transfer the
                // device has already ended.
                if chain {
                    self.skip_chain(space, &mut dequeue, &mut ccs);
                }
                more = true;
                break 'td;
            }
            // §6.4.1.1 bit 9: with `BEI` the Transfer Event is still queued —
            // it is only the interrupt at the next threshold that is blocked
            // (§4.17.5).
            if ioc
                && !self.transfer_event_blocked(
                    space,
                    (addr, false),
                    (outcome.residual, code::SUCCESS),
                    (slot, dci),
                    bei,
                )
            {
                break 'td;
            }
            if last {
                more = true;
                break 'td;
            }
            let _ = step;
        }

        if naked {
            // Rewind to the start of the TD: see the module docs on why partial
            // progress inside one is discarded.
            dequeue = start_dequeue;
            ccs = start_ccs;
            let mut regs = self.regs.lock();
            regs.ep_retry[usize::from(slot)] |= 1 << dci;
        } else if more {
            // There may be another TD behind this one; the run loop's budget is
            // what bounds the chain.
            let mut regs = self.regs.lock();
            regs.ep_pending[usize::from(slot)] |= 1 << dci;
        }

        // §6.2.3 Table 6-10: the Output Endpoint Context is where the dequeue
        // pointer lives, so this controller has no hidden per-endpoint state.
        ep[2] = (dequeue as u32 & !0xf) | u32::from(ccs);
        ep[3] = (dequeue >> 32) as u32;
        if halted {
            ep[0] = (ep[0] & !EP_STATE_MASK) | EP_STATE_HALTED;
        }
        if self.write_context(space, ep_addr, &ep).is_none() {
            self.host_system_error();
        }
    }

    /// Walk to the end of a chained Transfer Descriptor without executing it.
    fn skip_chain(&self, space: &AddressSpace, dequeue: &mut u64, ccs: &mut bool) {
        for _ in 0..MAX_TRBS_PER_TD {
            let Some((_, trb)) = self.fetch(space, dequeue, ccs) else {
                return;
            };
            Xhci::consume(dequeue);
            if trb[3] & TRB_CH == 0 {
                return;
            }
        }
    }

    /// A Setup Stage TRB (§6.4.1.2.1): eight bytes of immediate data.
    fn do_setup(&self, trb: &[u32; 4], address: DeviceAddress, endpoint: u8) -> Outcome {
        if trb[3] & TRB_IDT == 0 {
            // §6.4.1.2.1 bit 6: `IDT` shall be set in a Setup Stage TRB.
            return Outcome::failed(code::TRB_ERROR);
        }
        let mut raw = [0u8; 8];
        raw[0..4].copy_from_slice(&trb[0].to_le_bytes());
        raw[4..8].copy_from_slice(&trb[1].to_le_bytes());
        let packet = SetupPacket::decode(&raw);
        // §4.6.5: the xHC shall never forward a `SET_ADDRESS` from a transfer
        // ring to a device — addressing is the Address Device Command's job.
        if packet.request_type == 0 && packet.request == crate::bus::usb::request::SET_ADDRESS {
            return Outcome::failed(code::TRB_ERROR);
        }
        match self.bus.setup(address, endpoint, packet) {
            Status::Ack => Outcome::ok(0, 0),
            Status::Nak => Outcome::nak(),
            status => Outcome::failed(status_code(status)),
        }
    }

    /// A Normal or Data Stage TRB (§6.4.1.1, §6.4.1.2.2).
    fn do_data(
        &self,
        space: &AddressSpace,
        trb: &[u32; 4],
        address: DeviceAddress,
        endpoint: u8,
        dir_in: bool,
        mps: u32,
    ) -> Outcome {
        // §6.4.1.1 Table 6-21: seventeen bits, so 64 KiB at most.
        let total = trb[2] & 0x1_ffff;
        let immediate = trb[3] & TRB_IDT != 0;
        let buffer = u64::from(trb[0]) | (u64::from(trb[1]) << 32);
        if immediate && (dir_in || total > 8) {
            // §6.4.1.1 bit 6: `IDT` shall not be set on an IN endpoint, and the
            // length shall be at most eight.
            return Outcome::failed(code::TRB_ERROR);
        }

        let mut moved = 0u32;
        let mut short = false;
        for packet in 0..MAX_PACKETS {
            let remaining = total - moved;
            // A zero-length transfer is one transaction, not none (§4.9.1).
            if remaining == 0 && packet > 0 {
                break;
            }
            let want = mps.min(remaining);
            let completion = if dir_in {
                let mut buf = alloc::vec![0u8; want as usize];
                let completion = self.bus.read(address, endpoint, &mut buf);
                if completion.status == Status::Ack {
                    let n = (completion.len as u32).min(want) as usize;
                    // §4.9.1: with a zero-length transfer the Data Buffer
                    // Pointer *is ignored*, so nothing may touch the address
                    // it names — a guest is entitled to leave it pointing
                    // nowhere.
                    if n > 0
                        && space
                            .write_bytes(
                                buffer.wrapping_add(u64::from(moved)),
                                &buf[..n],
                                self.attrs(),
                            )
                            .is_err()
                    {
                        return Outcome::failed(code::TRB_ERROR);
                    }
                    if (n as u32) < want {
                        short = true;
                    }
                    Completion::ack(n as u64)
                } else {
                    completion
                }
            } else {
                let mut buf = alloc::vec![0u8; want as usize];
                if immediate {
                    let mut raw = [0u8; 8];
                    raw[0..4].copy_from_slice(&trb[0].to_le_bytes());
                    raw[4..8].copy_from_slice(&trb[1].to_le_bytes());
                    buf.copy_from_slice(&raw[moved as usize..moved as usize + want as usize]);
                } else if want > 0
                    && space
                        .read_bytes(
                            buffer.wrapping_add(u64::from(moved)),
                            &mut buf,
                            self.attrs(),
                        )
                        .is_err()
                {
                    return Outcome::failed(code::TRB_ERROR);
                }
                self.bus.write(address, endpoint, &buf)
            };

            match completion.status {
                Status::Ack => {
                    moved += (completion.len as u32).min(want);
                    if short || moved >= total {
                        break;
                    }
                }
                Status::Nak => {
                    if moved == 0 {
                        return Outcome::nak();
                    }
                    // Part of a TD moved and the device then declined. Rewinding
                    // would resend those bytes, so the transfer is reported
                    // short instead — unreachable for every device in this tree
                    // (module docs).
                    short = true;
                    break;
                }
                status => {
                    let mut outcome = Outcome::failed(status_code(status));
                    outcome.residual = total - moved;
                    return outcome;
                }
            }
        }

        // §6.4.2.1 Table 6-38: a Transfer Event reports the *residual*, not what
        // moved.
        Outcome {
            completion: code::SUCCESS,
            residual: total - moved,
            short,
            nak: false,
        }
    }

    /// A Status Stage TRB (§6.4.1.2.3): a zero-length transaction.
    fn do_status(&self, address: DeviceAddress, endpoint: u8, dir_in: bool) -> Outcome {
        let completion = if dir_in {
            self.bus.read(address, endpoint, &mut [])
        } else {
            self.bus.write(address, endpoint, &[])
        };
        match completion.status {
            Status::Ack => Outcome::ok(0, 0),
            Status::Nak => Outcome::nak(),
            status => Outcome::failed(status_code(status)),
        }
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
                ..Regs::reset(self.params.ports)
            };
            self.publish(&regs);
        }
        for port in 0..self.params.ports {
            self.bus.set_enabled(port, false);
        }
        for port in 0..self.params.ports {
            self.settle_port(port, false);
        }
        self.refresh_irq();
    }

    /// Serialize the register file and the ring positions.
    ///
    /// # What a snapshot mid-transfer means
    ///
    /// Nothing special, and that is by construction. A transaction runs to
    /// completion inside the doorbell write or the microframe that started it,
    /// and everything that outlives one — the contexts, the transfer rings, the
    /// event ring — lives in **guest memory**, which the RAM device saves. The
    /// controller's own durable state is the registers below plus where it has
    /// read to in the command and event rings.
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let regs = *self.regs.lock();
        w.write_u64(regs.ticks)?;
        w.write_u32(regs.usbcmd)?;
        w.write_u32(regs.usbsts)?;
        w.write_u32(regs.dnctrl)?;
        w.write_u64(regs.crcr_ptr)?;
        w.write_u64(regs.dcbaap)?;
        w.write_u32(regs.config)?;
        w.write_u32(regs.mfindex)?;
        w.write_u32(regs.iman)?;
        w.write_u32(regs.imod)?;
        w.write_u32(regs.imodc)?;
        w.write_u32(regs.erstsz)?;
        w.write_u64(regs.erstba)?;
        w.write_u64(regs.erdp)?;
        w.write_u64(regs.cmd_dequeue)?;
        w.write_bool(regs.cmd_ccs)?;
        w.write_bool(regs.crr)?;
        w.write_bool(regs.er_started)?;
        w.write_u32(regs.er_index)?;
        w.write_u64(regs.er_base)?;
        w.write_u32(regs.er_size)?;
        w.write_u64(regs.er_next_base)?;
        w.write_u32(regs.er_offset)?;
        w.write_bool(regs.er_pcs)?;
        w.write_bool(regs.er_full)?;
        w.write_u32(regs.slot_enabled)?;
        w.write_bool(regs.cmd_pending)?;
        w.write_seq_len(MAX_PORTS as u64)?;
        for port in regs.portsc {
            w.write_u32(port)?;
        }
        for flag in regs.psceg {
            w.write_bool(flag)?;
        }
        w.write_seq_len(MAX_SLOTS as u64 + 1)?;
        for slot in 0..=MAX_SLOTS {
            w.write_u32(regs.ep_pending[slot])?;
            w.write_u32(regs.ep_retry[slot])?;
        }
        Ok(())
    }

    /// Restore what [`save`](Xhci::save) wrote.
    ///
    /// # Errors
    ///
    /// [`Error::State`] for a truncated or malformed chunk.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let mut regs = Regs {
            ticks: r.read_u64()?,
            usbcmd: r.read_u32()?,
            usbsts: r.read_u32()?,
            dnctrl: r.read_u32()?,
            crcr_ptr: r.read_u64()?,
            dcbaap: r.read_u64()?,
            config: r.read_u32()?,
            mfindex: r.read_u32()?,
            iman: r.read_u32()?,
            imod: r.read_u32()?,
            imodc: r.read_u32()?,
            erstsz: r.read_u32()?,
            erstba: r.read_u64()?,
            erdp: r.read_u64()?,
            cmd_dequeue: r.read_u64()?,
            cmd_ccs: r.read_bool()?,
            crr: r.read_bool()?,
            er_started: r.read_bool()?,
            er_index: r.read_u32()?,
            er_base: r.read_u64()?,
            er_size: r.read_u32()?,
            er_next_base: r.read_u64()?,
            er_offset: r.read_u32()?,
            er_pcs: r.read_bool()?,
            er_full: r.read_bool()?,
            slot_enabled: r.read_u32()?,
            cmd_pending: r.read_bool()?,
            portsc: [0; MAX_PORTS],
            psceg: [false; MAX_PORTS],
            ep_pending: [0; MAX_SLOTS + 1],
            ep_retry: [0; MAX_SLOTS + 1],
        };
        let count = r.read_seq_len(4)?;
        if count != MAX_PORTS as u64 {
            return Err(Error::State(alloc::format!(
                "usb.xhci: a snapshot with {count} ports, not {MAX_PORTS}"
            )));
        }
        for port in &mut regs.portsc {
            *port = r.read_u32()?;
        }
        for flag in &mut regs.psceg {
            *flag = r.read_bool()?;
        }
        let count = r.read_seq_len(8)?;
        if count != MAX_SLOTS as u64 + 1 {
            return Err(Error::State(alloc::format!(
                "usb.xhci: a snapshot with {count} slots, not {}",
                MAX_SLOTS + 1
            )));
        }
        for slot in 0..=MAX_SLOTS {
            regs.ep_pending[slot] = r.read_u32()?;
            regs.ep_retry[slot] = r.read_u32()?;
        }
        // A segment size out of range would let a restored ring walk further
        // than the guest allocated; clamp it exactly as a fresh fetch does.
        if regs.er_size != 0 {
            regs.er_size = regs.er_size.clamp(ERST_MIN_SEGMENT, ERST_MAX_SEGMENT);
        }
        if regs.er_offset >= regs.er_size.max(1) {
            regs.er_offset = 0;
        }
        {
            let mut slot = self.regs.lock();
            *slot = regs;
            self.publish(&slot);
        }
        // The fabric's enable bits are derived state and are never serialized
        // (`ROADMAP.md` §4.5): they come back from `PORTSC`.
        for port in 0..self.params.ports {
            let enabled = self.portsc(port) & PORT_PED != 0;
            self.bus.set_enabled(port, enabled);
        }
        self.refresh_irq();
        Ok(())
    }
}

/// What executing one transfer TRB did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Outcome {
    /// The completion code to report, [`code::SUCCESS`] if it worked.
    completion: u32,
    /// Bytes the TRB asked for and did not move (§6.4.2.1, Table 6-38).
    residual: u32,
    /// The device ended the transfer early.
    short: bool,
    /// The device has nothing right now: retry, do not report.
    nak: bool,
}

impl Outcome {
    const fn ok(residual: u32, _len: u32) -> Outcome {
        Outcome {
            completion: code::SUCCESS,
            residual,
            short: false,
            nak: false,
        }
    }

    const fn failed(completion: u32) -> Outcome {
        Outcome {
            completion,
            residual: 0,
            short: false,
            nak: false,
        }
    }

    const fn nak() -> Outcome {
        Outcome {
            completion: code::SUCCESS,
            residual: 0,
            short: false,
            nak: true,
        }
    }
}

/// The completion code a fabric handshake becomes (§6.4.5).
fn status_code(status: Status) -> u32 {
    match status {
        Status::Ack | Status::Nak => code::SUCCESS,
        Status::Stall => code::STALL_ERROR,
        Status::Babble => code::BABBLE,
        Status::NoDevice | Status::Error => code::USB_TRANSACTION_ERROR,
    }
}

/// The Protocol Speed ID a link speed reports in `PORTSC` (§7.2.2.1.1,
/// Table 7-13).
///
/// `PSIC` is zero in this controller's Supported Protocol capability, so the
/// default mapping applies: 1 is full speed, 2 low, 3 high. Zero is "undefined
/// speed", which §5.4.8 says is what a port with nothing on it reports.
fn speed_id(speed: Option<Speed>) -> u32 {
    match speed {
        Some(Speed::Full) => 1,
        Some(Speed::Low) => 2,
        Some(Speed::High) => 3,
        None => 0,
    }
}

/// Whether an Endpoint Context is one this controller can honour.
///
/// Streams are not modelled, so a non-zero `MaxPStreams` is a *Parameter Error*
/// rather than a field quietly ignored (§6.2.3, Table 6-8).
fn check_endpoint(ep: &[u32; CONTEXT_DWORDS], want_type: u32) -> core::result::Result<(), u32> {
    if (ep[0] >> EP_MAXPSTREAMS_SHIFT) & 0x1f != 0 {
        return Err(code::PARAMETER_ERROR);
    }
    let ep_type = (ep[1] >> EP_TYPE_SHIFT) & 0x7;
    if ep_type == EP_TYPE_INVALID || ep_type != want_type {
        return Err(code::PARAMETER_ERROR);
    }
    if (ep[1] >> EP_MPS_SHIFT) & 0xffff == 0 {
        return Err(code::PARAMETER_ERROR);
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An xHCI host controller as a machine object.
#[derive(Debug)]
pub struct XhciController {
    xhci: Arc<Xhci>,
    region: RegionRef,
}

impl XhciController {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this controller is the root of. Required.
    /// * `ports` — how many root ports, 1 to 15. Defaults to 1.
    /// * `slots` — how many device slots, 1 to 31. Defaults to 8.
    /// * `microframe` — how many clock-domain ticks one 125 µs microframe
    ///   takes. Defaults to 7500, which is the number at the 60 MHz a USB 2.0
    ///   PHY runs at.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property, [`Error::Config`]
    /// for a value outside its range or a bus that is already smaller than the
    /// port count asked for.
    pub fn new(props: &Props) -> Result<XhciController> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let ports = r.or_range("ports", 1u64, 1..=MAX_PORTS as u64)?;
        let slots = r.or_range("slots", 8u64, 1..=MAX_SLOTS as u64)?;
        let microframe = r.or_range("microframe", 7500u64, 1..=u64::from(u32::MAX))?;
        r.finish()?;

        let bus = buses::attach(props, &bus_name, ports as u8)?;
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
        Ok(XhciController::with_bus(
            bus,
            Params {
                ports: ports as u8,
                slots: slots as u8,
                microframe_ticks: microframe,
            },
        ))
    }

    /// A controller on a bus the caller already holds.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>, params: Params) -> XhciController {
        let xhci = Arc::new(Xhci::new(bus, params));
        let region = register_region(&xhci, "xhci", REGISTER_BYTES);
        XhciController { xhci, region }
    }

    /// The engine underneath.
    #[must_use]
    pub fn xhci(&self) -> &Arc<Xhci> {
        &self.xhci
    }
}

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output. Level-triggered, and the AND of `USBCMD.INTE`,
    /// `IMAN.IE` and `IMAN.IP` (xHCI 1.2 §4.17.3).
    pub const IRQ: &str = "irq";
}

/// The register block of `xhci`, as something an address space dispatches to.
///
/// `len` is how much address space the block claims. [`REGISTER_BYTES`] is what
/// it needs; a **base address register** takes a power of two and nothing else
/// (*PCI Local Bus Specification* Rev 2.1 §6.2.5.1), so [`pci`] asks for the
/// next one up. The tail costs nothing: §5.5 and §5.6 make every dword past the
/// last interrupter reserved, and this block already reads reserved space as
/// zero and ignores writes to it.
///
/// A `len` below [`REGISTER_BYTES`] would hide registers, so it is raised
/// rather than honoured.
#[must_use]
pub fn register_region(xhci: &Arc<Xhci>, name: &str, len: u64) -> RegionRef {
    let port = Arc::new(XhciPort {
        xhci: Arc::clone(xhci),
    });
    Arc::new(Region::io(
        name,
        len.max(REGISTER_BYTES),
        port as Arc<dyn MemOps>,
    ))
}

/// The xHCI register block, as something an address space dispatches to.
struct XhciPort {
    xhci: Arc<Xhci>,
}

impl fmt::Debug for XhciPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("XhciPort").finish_non_exhaustive()
    }
}

impl XhciPort {
    /// The dword at `offset & !3`, wherever in the block it lives.
    fn dword(&self, offset: u64) -> u32 {
        let aligned = offset & !0x3;
        if aligned < XECP_OFFSET {
            self.xhci.read_cap(aligned)
        } else if aligned < XECP_OFFSET + 0x10 {
            self.xhci.read_xecp(aligned - XECP_OFFSET)
        } else if aligned < OP_BASE {
            0
        } else if aligned < DB_BASE {
            self.xhci.read_op(aligned - OP_BASE)
        } else if aligned < RT_BASE {
            // §5.6: a doorbell reads back zero and software should treat the
            // value as undefined.
            0
        } else {
            self.xhci.read_rt(aligned - RT_BASE)
        }
    }

    /// One dword write, dispatched to whichever register file owns the offset.
    fn store(&self, offset: u64, value: u32) {
        if offset < OP_BASE {
            // §5.3 and §7: the capability and extended-capability registers are
            // read-only.
            return;
        }
        let after = if offset < DB_BASE {
            self.xhci.write_op(offset - OP_BASE, value)
        } else if offset < RT_BASE {
            let index = (offset - DB_BASE) / 4;
            self.xhci.write_doorbell(index, value)
        } else {
            self.xhci.write_rt(offset - RT_BASE, value)
        };
        self.xhci.act(after);
    }
}

impl MemOps for XhciPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.xhci.sync_for(attrs);
        match dst.len() {
            1 | 2 | 4 => {
                let bytes = self.dword(offset).to_le_bytes();
                let lane = (offset & 0x3) as usize;
                if lane + dst.len() > 4 {
                    return Err(BusError::BadAccess);
                }
                dst.copy_from_slice(&bytes[lane..lane + dst.len()]);
                Ok(())
            }
            // §5.1: with `AC64` set, software *should* read the 64-bit registers
            // with Qword accesses, so one is answered as its two dwords.
            8 => {
                if offset & 0x7 != 0 {
                    return Err(BusError::BadAccess);
                }
                dst[0..4].copy_from_slice(&self.dword(offset).to_le_bytes());
                dst[4..8].copy_from_slice(&self.dword(offset + 4).to_le_bytes());
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A doorbell has no harmless version, and `USBSTS`, `IMAN.IP`,
            // `ERDP.EHB` and every `PORTSC` change bit are write-1-to-clear:
            // a debugger writing here would consume a TRB, advance a dequeue
            // pointer, or acknowledge an interrupt the guest never saw
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.xhci.sync_for(attrs);
        match src.len() {
            4 => {
                if offset & 0x3 != 0 {
                    return Err(BusError::BadAccess);
                }
                self.store(offset, u32::from_le_bytes([src[0], src[1], src[2], src[3]]));
                Ok(())
            }
            // §5.1: low dword first, high dword second — which is what a Qword
            // write to a 64-bit register means.
            8 => {
                if offset & 0x7 != 0 {
                    return Err(BusError::BadAccess);
                }
                self.store(offset, u32::from_le_bytes([src[0], src[1], src[2], src[3]]));
                self.store(
                    offset + 4,
                    u32::from_le_bytes([src[4], src[5], src[6], src[7]]),
                );
                Ok(())
            }
            // §5.4.4 and §5.6: byte and halfword writes produce undefined
            // results, so they are refused rather than guessed at.
            _ => Err(BusError::BadAccess),
        }
    }

    fn constraints(&self) -> AccessConstraints {
        // Reads may be 8, 16, 32 or 64 bits: `CAPLENGTH` is a byte and
        // `HCIVERSION` a halfword (§5.3), and a 64-bit register wants a Qword
        // (§5.1). Writes are checked separately.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U64)
            .with_natural_alignment(true)
    }
}

impl Device for XhciController {
    fn class(&self) -> &'static DeviceClass {
        &XHCI_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and a `wire`
        // statement connects the interrupt.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        self.xhci.reset(kind);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.xhci.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.xhci.load(r)
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "an xHCI controller drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.xhci.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.xhci.refresh_irq();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. `MFINDEX` runs on its own and a guest that polls it has to see the
    /// answer at the cycle it polled.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.xhci.ticks()
    }

    fn advance_to(&self, tick: u64) {
        self.xhci.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.xhci.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.xhci.attach_lazy(handle);
    }
}

impl Instance for XhciController {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an xHCI controller masters the bus its rings and contexts live on: add \
                 `space = mem` to the object",
            ),
        })?;
        self.xhci.attach_space(space, ctx.requester());
        Ok(())
    }
}

/// The `usb.xhci` device class.
pub static XHCI_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an xHCI USB host controller: the xHCI 1.2 register file, a command ring, device \
              and endpoint contexts, transfer rings and an event ring, all DMA-read out of guest \
              RAM",
    properties: XHCI_PROPERTIES,
    construct: |props| Ok(Box::new(XhciController::new(props)?)),
};

/// The properties [`XHCI_CLASS`] accepts.
static XHCI_PROPERTIES: &[PropertySpec] = &[
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
        name: "slots",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many device slots, 1 to 31 (default 8)",
    },
    PropertySpec {
        name: "microframe",
        kind: ValueKind::Uint,
        required: false,
        summary: "clock-domain ticks in one 125 us microframe (default 7500, exact at 60 MHz)",
    },
];

/// Add [`XHCI_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&XHCI_CLASS)
}

/// Bind [`XHCI_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| {
        Ok(Arc::new(XhciController::new(props)?))
    })
}

/// What the validator should know about `usb.xhci`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, MAX_PORTS as u64))
        .prop(PropSchema::new("slots", ValueKind::Uint).range(1, MAX_SLOTS as u64))
        .prop(PropSchema::new("microframe", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
}

/// Where the operational registers, the doorbell array and the runtime
/// registers sit inside the block, for a test or a guest that has to build a
/// driver against them.
pub mod offset {
    /// The operational register base — `CAPLENGTH` (§5.3.1).
    pub const OPERATIONAL: u64 = super::OP_BASE;
    /// The first port register set, relative to [`OPERATIONAL`] (§5.4).
    pub const PORT: u64 = super::PORT_BASE;
    /// Bytes between one port register set and the next.
    pub const PORT_STRIDE: u64 = super::PORT_STRIDE;
    /// The doorbell array — `DBOFF` (§5.3.7).
    pub const DOORBELL: u64 = super::DB_BASE;
    /// The runtime registers — `RTSOFF` (§5.3.8).
    pub const RUNTIME: u64 = super::RT_BASE;
    /// Interrupter 0's register set, relative to [`RUNTIME`] (§5.5).
    pub const INTERRUPTER0: u64 = super::IR0_BASE;
    /// The xHCI Supported Protocol extended capability (§7.2).
    pub const XECP: u64 = super::XECP_OFFSET;
}

/// The size of one context entry, for a driver building an Input Context
/// (§6.2).
pub const CONTEXT_SIZE: u64 = CONTEXT_BYTES;

/// The size of one TRB (§6.4).
pub const TRB_SIZE: u64 = TRB_BYTES;
