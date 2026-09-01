//! A Synopsys DesignWare USB 2.0 OTG controller (`dwc2`), as STM32's OTG_FS
//! block instantiates it — **both roles**: a host that enumerates devices, and
//! a device that a host enumerates.
//!
//! # What this controller is, and why it is not an EHCI
//!
//! An EHCI is a **bus master**: the driver builds queue heads and transfer
//! descriptors in guest RAM, hands over two list roots, and the controller
//! walks them by itself. This core does none of that. It has
//!
//! * **host channels** — register triples the driver programs *directly* with
//!   an address, an endpoint, a direction, a packet size and a byte count, then
//!   arms with `HCCHAR.CHENA`; and
//! * **a shared FIFO** the driver pushes and pops through a set of 4 KiB
//!   windows, with received packets announced one at a time by a status word
//!   read out of `GRXSTSP`.
//!
//! There is no schedule in memory, no descriptor, and — in the full-speed
//! configuration modelled here — **no DMA at all**. The core never reads guest
//! memory, so unlike [`crate::dev::usb::ehci`] it needs no `space =` in a
//! machine file: the bytes reach it because the *CPU* wrote them into a FIFO
//! window.
//!
//! ```text
//!   guest                     this controller                 fabric
//!   ─────                     ───────────────                 ──────
//!   HCCHARn  = addr|ep|dir ─► channel n armed
//!   HCTSIZn  = size|cnt|pid                  ──► SETUP / IN / OUT ──► device
//!   DFIFO(n) = data        ─► its TX staging
//!                             the RX FIFO   ◄── data returned
//!   GRXSTSP  ◄──────────────  one status word per packet
//!   DFIFO(n) ◄──────────────  that packet's bytes
//!   HCINTn / HAINT / GINTSTS  ◄── what happened
//! ```
//!
//! That is a completely different shape of controller from an EHCI, and it is
//! the reason this file exists: it is the test of whether [`crate::bus::usb`]
//! is genuinely controller-agnostic. **It needed no change.** The fabric's seam
//! is a *transaction* — `SETUP`, `IN`, `OUT`, and a handshake back — and a host
//! channel is nothing but a register-shaped way of asking for one, exactly as a
//! queue head is a memory-shaped way of asking for one.
//!
//! # Dual role, both of them real
//!
//! The block is dual-role and so is this model. `GUSBCFG.FDMOD` selects the
//! device side, `GINTSTS.CMOD` reports which one is running, and reaching for
//! the other role's registers raises `GINTSTS.MMIS` as the silicon does. The
//! host half is this file; the device half is [`device`], and the two share
//! one register lock, one shared receive FIFO and one interrupt pin, because on
//! the die they are one block.
//!
//! Device mode is where the guest presents *itself* as a peripheral — a board
//! that is a USB serial port or a printer rather than a USB port you plug one
//! into. It is a [`UsbDevice`] implementation over `DIEPCTLn`/`DOEPCTLn`, and
//! **the fabric carried it very nearly unchanged**: see [`device`] for the
//! whole argument, and `docs/buses/usb.md` §4.1 for the three small things that
//! did have to move.
//!
//! # Speed is real here, and is not the constraint EHCI has
//!
//! STM32's OTG_FS has an on-chip **full-speed** transceiver and nothing else,
//! and `HCFG.FSLSS` is the register that says so. So this controller drives
//! full- **and low-speed** devices, which an EHCI cannot do at all, and cannot
//! drive a high-speed one, which an EHCI can. The honest consequence differs
//! too: EHCI hands an unreachable port to a companion controller (EHCI 1.0
//! §4.2.2) and rsemu's model does that. A dwc2 root port has no companion — it
//! is the only controller on the pins — so what happens instead is that the
//! reset completes, `HPRT.PENCHNG` fires, and **`HPRT.PENA` stays clear**: a
//! driver learns the port failed to enable rather than waiting forever or
//! enumerating a device the pins could not have carried. `speed = "high"`
//! configures the high-speed variant of the same core, for a board that has a
//! ULPI PHY.
//!
//! # The interrupt, which is most of how this block is used
//!
//! `HCINTn` feeds `HAINT` through `HCINTMSKn`, `HAINT` feeds `GINTSTS.HCINT`
//! through `HAINTMSK`, `GINTSTS` is gated by `GINTMSK`, and the whole tree is
//! gated again by `GAHBCFG.GINTMSK`. All of that collapses to **one level output
//! pin, `irq`**, re-derived from the register file rather than latched — a net
//! delivers changes, so a pin left at a constructor default would never be
//! corrected, and [`Device::announce`] re-derives it for that reason.
//!
//! The pin carries no number. Which interrupt an OTG_FS is on is a fact about
//! the *part* — 67 on an STM32F407 — and belongs in the machine file, which
//! wires this pin straight to the CPU's own:
//!
//! ```text
//!   object otgfs "usb.dwc2" { clock = otgclk, bus = usb0, speed = "full" }
//!   map  mem 0x50000000 size 256K = otgfs
//!   wire otgfs.irq -> cpu.irq67
//! ```
//!
//! There is nothing to put in between: a Cortex-M's NVIC is inside the core.
//!
//! # Time
//!
//! The scheduler owns it (`CLAUDE.md`), and this is a lazily advanced device
//! (`ROADMAP.md` §4.2) exactly as the EHCI is. A frame is `HFIR.FRIVL` PHY
//! clocks — the guest programs it, and until it does the reset value of
//! `0xea60` really is the interval, which is what the silicon does — times
//! [`Params::phy_ticks`] domain ticks per PHY clock. At the 48 MHz an OTG_FS
//! PHY runs at, a driver writes `48000` and a frame is exactly 48 000 ticks,
//! integer and residue-free, which is what §4.2's oscillator forest exists for.
//!
//! **Transactions are executed at frame boundaries**, up to a per-frame byte
//! budget taken from the signalling rate — 1500 bytes at full speed, 187 at low
//! speed, which is what 12 Mb/s and 1.5 Mb/s are per millisecond — with each
//! transaction charged its payload plus the 13 bytes of protocol overhead USB
//! 2.0 §5.11.3 gives a full-speed bulk or interrupt transaction. That reproduces
//! the bus's real throughput. It is coarser than silicon in *latency* — a real
//! full-speed transaction takes about 50 µs, so a channel armed just after a
//! frame boundary waits longer here than it would on hardware — and that is the
//! one timing simplification in this file, stated rather than hidden.
//!
//! # Everything a guest can drive is bounded
//!
//! The guest owns the channel registers and the FIFO windows, so:
//!
//! * a frame executes at most [`MAX_TRANSACTIONS_PER_FRAME`] transactions, and
//!   the byte budget alone would already end it;
//! * a periodic channel gets **one** transaction per frame, which is what a
//!   service interval means, so an idle interrupt endpoint cannot spend the
//!   whole frame `NAK`ing;
//! * the receive FIFO and each channel's transmit staging are capped by the
//!   programmed FIFO sizes, themselves capped by [`Params::fifo_words`] — the
//!   part's dedicated RAM — so no register write can make this device allocate;
//! * `HFIR.FRIVL` is clamped to [`MIN_FRAME_PHY_CLOCKS`], because a frame
//!   interval of one PHY clock is not a frame interval, it is a way to make the
//!   host spin.
//!
//! `fuzz/fuzz_targets/usb_dwc2.rs` drives arbitrary bytes at the register block
//! and the FIFO windows for exactly this reason.
//!
//! # `MemAttrs::debug`
//!
//! This block has the trap the EHCI's `USBSTS` has, twice over, and both are
//! tested:
//!
//! * **`GRXSTSP` pops the receive FIFO when it is read.** A debug read must pop
//!   nothing, so it answers what `GRXSTSR` would — the same word, queue
//!   untouched.
//! * **A FIFO window read consumes the packet.** A debug read returns the word
//!   without advancing the read pointer.
//! * A debug **write** is refused outright ([`BusError::BadAccess`]): `HCINTn`
//!   is write-1-to-clear, `GRSTCTL` resets the core, `HCCHAR.CHENA` starts a
//!   transaction and a FIFO write injects bytes onto the wire. None of those has
//!   a harmless version.
//! * A debug read advances no time: reads sync with
//!   [`AccessKind::Debug`], so reading `HFNUM` from a monitor does not move the
//!   frame counter.
//!
//! # What is not modelled, said plainly
//!
//! * **DMA.** `GAHBCFG.DMAEN` reads zero, which is what a core with no DMA
//!   reports and what makes a driver take the slave-mode path this file
//!   implements. The high-speed core's buffer DMA, and `HCDMAn` with it, is not
//!   here.
//! * **Split transactions.** `HCSPLTn` stores and reads back and does nothing:
//!   splits reach a low-speed device *through a hub*, and there is no hub device
//!   in this tree ([`crate::bus::usb`] says so). A low-speed device plugged
//!   straight into the root port needs no split and works.
//! * **Isochronous.** A channel programmed with `EPTYP = 01b` is serviced once
//!   per frame like an interrupt channel and its data moves; what is missing is
//!   the frame-parity scheduling `HCCHAR.ODDFRM` selects, so `ODDFRM` stores and
//!   reads back and is not acted on.
//! * **OTG session request, HNP and SRP.** `GOTGCTL`'s writable bits store and
//!   read back and the status bits follow the role — a settled A-device session
//!   in host mode, a B-device one in device mode — but nothing *negotiates*. A
//!   board that flips roles at runtime does so by writing `FDMOD`/`FHMOD`, which
//!   is what firmware does anyway; the ID-pin dance that would do it by itself
//!   is not here.
//! * The device half has its own list, in [`device`]: back-to-back setup
//!   packets, suspend and remote wakeup, and the frame-parity selectors.
//!
//! # Sources
//!
//! ST's **RM0090** (STM32F405/415/407/417/427/437/429/439 reference manual),
//! §34 *USB on-the-go full-speed (OTG_FS)* — the register map of both roles and
//! the reset values, `GRXSTSP`'s packet-status encoding, `HPRT`'s
//! write-1-to-clear bits and the channel registers; §34.15.3 for the device
//! block, which [`device`] cites in its own right — and the **USB 2.0
//! specification** (usb.org, free download) for everything above the
//! controller: §5.11.3 for the
//! per-transaction protocol overhead the frame budget is charged in, §8.4 for
//! what a transaction is, §8.6.1 for the data toggle, and §9 for the device
//! framework. No emulator source was consulted, and no driver: the Linux dwc2
//! driver is GPLv2 and ST's HAL is vendor-licensed, and neither was opened
//! (`ROADMAP.md` §1).

pub mod device;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use core::fmt;

use crate::bus::usb::{
    Completion, DeviceAddress, HCD_RANK, SetupPacket, Speed, Status, TransferType, UsbBus,
    UsbDevice, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.dwc2";

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// Version 2 appended the device-mode register file: `DCFG`, `DCTL`, the two
/// endpoint arrays and their transmit staging.
pub const STATE_VERSION: u32 = 2;

pub use device::{Dwc2Gadget, MAX_ENDPOINTS};

/// The one root port a dwc2 core has.
///
/// `HPRT` is a single register, not an array: this controller is the whole bus,
/// and reaching a second device needs a hub, which this tree does not model.
pub const ROOT_PORT: u8 = 0;

/// How many host channels the register map has room for — `HAINT` is sixteen
/// bits and `GRXSTSP.CHNUM` is four.
pub const MAX_CHANNELS: usize = 16;

/// How much address space the block takes.
///
/// The registers stop at `PCGCCTL` (`+0xe00`) and the sixteen FIFO windows end
/// at `+0x10000`; ST reserves 256 KiB for the whole peripheral, which is what a
/// board maps.
pub const REGISTER_BYTES: u64 = 0x4_0000;

/// Where the first FIFO access window starts.
///
/// Window *n* is `FIFO_BASE + n * FIFO_WINDOW`, is 4 KiB wide, and **every
/// address inside it is the same FIFO port** — the window is an aperture, not
/// memory.
pub const FIFO_BASE: u64 = 0x1000;

/// The stride between FIFO windows. See [`FIFO_BASE`].
pub const FIFO_WINDOW: u64 = 0x1000;

/// The most transactions one frame executes, whatever the registers say.
///
/// The byte budget below already ends a full-speed frame long before this; this
/// is the bound that does not depend on arithmetic a guest can influence.
pub const MAX_TRANSACTIONS_PER_FRAME: usize = 1024;

/// The smallest frame interval `HFIR.FRIVL` is allowed to mean, in PHY clocks.
///
/// A frame is a millisecond. A guest may write anything into a sixteen-bit
/// field, and a frame interval of one PHY clock would ask this device to
/// simulate 48 million frames a second — so the *effective* interval is clamped
/// here, in the same spirit as the EHCI's bounded walks. About 20 µs at 48 MHz.
pub const MIN_FRAME_PHY_CLOCKS: u64 = 1000;

/// The non-data bytes one full-speed bulk or interrupt transaction costs on the
/// wire (USB 2.0 §5.11.3): token, sync, EOP, handshake and the inter-packet
/// gaps.
///
/// Charged against the frame budget, so a stream of `NAK`s costs bus time —
/// which is what stops an endpoint that never answers from starving the frame
/// it shares.
pub const PROTOCOL_OVERHEAD_BYTES: u32 = 13;

// ---------------------------------------------------------------------------
// Register offsets (RM0090 §34.15)
// ---------------------------------------------------------------------------

/// OTG control and status.
const GOTGCTL: u64 = 0x000;
/// OTG interrupt, write-1-to-clear.
const GOTGINT: u64 = 0x004;
/// AHB configuration: the global interrupt enable and the DMA select.
const GAHBCFG: u64 = 0x008;
/// USB configuration: the PHY select and the role force bits.
const GUSBCFG: u64 = 0x00c;
/// Reset control. Every reset bit here self-clears.
const GRSTCTL: u64 = 0x010;
/// Core interrupt status.
const GINTSTS: u64 = 0x014;
/// Core interrupt mask.
const GINTMSK: u64 = 0x018;
/// Receive status debug read — the head of the queue, *not* popped.
const GRXSTSR: u64 = 0x01c;
/// Receive status read and pop. **Reading this changes the FIFO.**
const GRXSTSP: u64 = 0x020;
/// Receive FIFO size, in 32-bit words.
const GRXFSIZ: u64 = 0x024;
/// Host non-periodic transmit FIFO size.
const GNPTXFSIZ: u64 = 0x028;
/// Host non-periodic transmit FIFO and queue status.
const GNPTXSTS: u64 = 0x02c;
/// The vendor's general-configuration slot — ST calls it `GCCFG` and puts the
/// transceiver's power control in it.
const GCCFG: u64 = 0x038;
/// The vendor's user-ID slot — ST calls it `CID`.
const CID: u64 = 0x03c;
/// Host periodic transmit FIFO size.
const HPTXFSIZ: u64 = 0x100;
/// Host configuration: the PHY clock select.
const HCFG: u64 = 0x400;
/// Host frame interval.
const HFIR: u64 = 0x404;
/// Host frame number and the time remaining in it.
const HFNUM: u64 = 0x408;
/// Host periodic transmit FIFO and queue status.
const HPTXSTS: u64 = 0x410;
/// Which channels have a pending interrupt.
const HAINT: u64 = 0x414;
/// Which channels may raise one.
const HAINTMSK: u64 = 0x418;
/// Host port control and status: the root port.
const HPRT: u64 = 0x440;
/// Where the per-channel registers start.
const HCCHAR_BASE: u64 = 0x500;
/// How far apart one channel's registers are from the next.
const CHANNEL_STRIDE: u64 = 0x20;
/// Where the host register block ends and the device one begins.
const DEVICE_BASE: u64 = 0x800;
/// Power and clock gating control.
const PCGCCTL: u64 = 0xe00;
/// Where the device register block ends.
const DEVICE_END: u64 = PCGCCTL;

// -- GOTGCTL ----------------------------------------------------------------

/// Session request success. Read-only.
const OTGCTL_SRQSCS: u32 = 1 << 0;
/// Connector ID status: zero is an A-device, which is a host.
const OTGCTL_CIDSTS: u32 = 1 << 16;
/// A-session valid. Read-only.
const OTGCTL_ASVLD: u32 = 1 << 18;
/// B-session valid. Read-only.
const OTGCTL_BSVLD: u32 = 1 << 19;
/// The bits software may set: the session and HNP requests and their enables.
const OTGCTL_WRITABLE: u32 = (1 << 1) | (1 << 9) | (1 << 10) | (1 << 11);

// -- GAHBCFG ----------------------------------------------------------------

/// The global interrupt enable: with this clear the `irq` output never rises.
const AHBCFG_GINTMSK: u32 = 1 << 0;
/// What software may set. `DMAEN` is not in it — this core has no DMA.
const AHBCFG_WRITABLE: u32 = AHBCFG_GINTMSK | (0xf << 1) | (1 << 7) | (1 << 8);

// -- GUSBCFG ----------------------------------------------------------------

/// Full-speed serial transceiver select. An FS core has nothing else, so this
/// reads one.
const USBCFG_PHYSEL: u32 = 1 << 6;
/// Force host mode.
const USBCFG_FHMOD: u32 = 1 << 29;
/// Force device mode.
const USBCFG_FDMOD: u32 = 1 << 30;
/// What software may set.
const USBCFG_WRITABLE: u32 = 0x7
    | USBCFG_PHYSEL
    | (1 << 8)
    | (1 << 9)
    | (0xf << 10)
    | USBCFG_FHMOD
    | USBCFG_FDMOD
    | (1 << 31);
/// `GUSBCFG` out of reset: the turnaround-time default, with the FS PHY
/// selected.
const USBCFG_RESET_VALUE: u32 = 0x0000_0a00 | USBCFG_PHYSEL;

// -- GRSTCTL ----------------------------------------------------------------

/// Core soft reset. Self-clearing.
const RSTCTL_CSRST: u32 = 1 << 0;
/// Host frame counter reset. Self-clearing.
const RSTCTL_FCRST: u32 = 1 << 2;
/// Receive FIFO flush. Self-clearing.
const RSTCTL_RXFFLSH: u32 = 1 << 4;
/// Transmit FIFO flush. Self-clearing.
const RSTCTL_TXFFLSH: u32 = 1 << 5;
/// Which FIFO `TXFFLSH` flushes; `0x10` means all of them.
const RSTCTL_TXFNUM_SHIFT: u32 = 6;
/// The AHB master is idle — always, here, because this core has no bus master
/// to be busy. A driver spins on it after asking for a reset.
const RSTCTL_AHBIDL: u32 = 1 << 31;

// -- GINTSTS / GINTMSK ------------------------------------------------------

/// Current mode: one is host. Read-only.
pub const GINT_CMOD: u32 = 1 << 0;
/// Mode mismatch: the application touched the other role's registers.
pub const GINT_MMIS: u32 = 1 << 1;
/// An OTG event is pending in `GOTGINT`. Read-only.
const GINT_OTGINT: u32 = 1 << 2;
/// Start of frame.
pub const GINT_SOF: u32 = 1 << 3;
/// The receive FIFO has a packet waiting. Read-only.
pub const GINT_RXFLVL: u32 = 1 << 4;
/// The non-periodic transmit FIFO is empty. Read-only.
const GINT_NPTXFE: u32 = 1 << 5;
/// An incomplete periodic transfer.
const GINT_IPXFR: u32 = 1 << 21;
/// Something changed in `HPRT`. Read-only: clear the `HPRT` bit instead.
pub const GINT_HPRTINT: u32 = 1 << 24;
/// Some channel has an unmasked interrupt. Read-only: clear `HCINTn` instead.
pub const GINT_HCINT: u32 = 1 << 25;
/// The periodic transmit FIFO is empty. Read-only.
const GINT_PTXFE: u32 = 1 << 26;
/// The connector ID changed.
const GINT_CIDSCHG: u32 = 1 << 28;
/// The device disconnected.
pub const GINT_DISCINT: u32 = 1 << 29;
/// A session request.
const GINT_SRQINT: u32 = 1 << 30;
/// A resume or remote wakeup.
const GINT_WKUINT: u32 = 1 << 31;

/// The write-1-to-clear half of `GINTSTS`.
///
/// The device-mode bits are in here even though nothing sets them: a driver that
/// clears the whole register on startup must not be surprised, and their absence
/// would be a difference a guest could see.
const GINT_W1C: u32 = GINT_MMIS
    | GINT_SOF
    | (1 << 10)
    | (1 << 11)
    | (1 << 12)
    | (1 << 13)
    | (1 << 14)
    | (1 << 15)
    | (1 << 20)
    | GINT_IPXFR
    | GINT_CIDSCHG
    | GINT_DISCINT
    | GINT_SRQINT
    | GINT_WKUINT;

// -- HCFG -------------------------------------------------------------------

/// PHY clock select, bits 1:0. `01b` is the 48 MHz an FS transceiver runs at.
const HCFG_FSLSPCS_MASK: u32 = 0x3;
/// FS- and LS-only support. Read-only, and set exactly when the PHY cannot do
/// high speed — the register that answers "what can this port carry".
const HCFG_FSLSS: u32 = 1 << 2;

// -- HPRT -------------------------------------------------------------------

/// Something is plugged in. Read-only.
pub const HPRT_PCSTS: u32 = 1 << 0;
/// A connect was detected. Write-1-to-clear.
pub const HPRT_PCDET: u32 = 1 << 1;
/// The port is enabled. **Write-1-to-clear** — writing a one *disables* it,
/// which is why every driver masks this bit out of a read-modify-write.
pub const HPRT_PENA: u32 = 1 << 2;
/// The enable state changed. Write-1-to-clear.
pub const HPRT_PENCHNG: u32 = 1 << 3;
/// Overcurrent. Never asserted: a modelled bus has no current.
const HPRT_POCA: u32 = 1 << 4;
/// Overcurrent changed. Write-1-to-clear.
const HPRT_POCCHNG: u32 = 1 << 5;
/// Resume signalling.
const HPRT_PRES: u32 = 1 << 6;
/// The port is suspended.
const HPRT_PSUSP: u32 = 1 << 7;
/// Drive a bus reset. Software sets it, waits, and clears it; the port is
/// enabled when it does.
pub const HPRT_PRST: u32 = 1 << 8;
/// Line status, bits 11:10. Read-only.
const HPRT_PLSTS_SHIFT: u32 = 10;
/// Port power.
pub const HPRT_PPWR: u32 = 1 << 12;
/// Port speed, bits 18:17. Read-only.
const HPRT_PSPD_SHIFT: u32 = 17;
/// `PSPD`: high speed.
const PSPD_HIGH: u32 = 0;
/// `PSPD`: full speed.
const PSPD_FULL: u32 = 1;
/// `PSPD`: low speed.
const PSPD_LOW: u32 = 2;
/// The write-1-to-clear bits.
const HPRT_W1C: u32 = HPRT_PCDET | HPRT_PENA | HPRT_PENCHNG | HPRT_POCCHNG;
/// The bits software sets directly.
const HPRT_WRITABLE: u32 = HPRT_PRES | HPRT_PSUSP | HPRT_PRST | HPRT_PPWR | (0xf << 13);

// -- HCCHARn ----------------------------------------------------------------

/// Maximum packet size, bits 10:0.
const HCCHAR_MPSIZ_MASK: u32 = 0x7ff;
/// Endpoint number, bits 14:11.
const HCCHAR_EPNUM_SHIFT: u32 = 11;
/// Endpoint direction: set is `IN`.
const HCCHAR_EPDIR: u32 = 1 << 15;
/// Endpoint type, bits 19:18 — the same two-bit encoding an endpoint descriptor
/// carries (USB 2.0 §9.6.6), which is why
/// [`TransferType::from_attribute_bits`] decodes it.
const HCCHAR_EPTYP_SHIFT: u32 = 18;
/// Device address, bits 28:22.
const HCCHAR_DAD_SHIFT: u32 = 22;
/// Halt the channel.
const HCCHAR_CHDIS: u32 = 1 << 30;
/// Arm the channel.
const HCCHAR_CHENA: u32 = 1 << 31;

// -- HCINTn -----------------------------------------------------------------

/// The transfer finished: the packet count reached zero, or a short packet
/// ended it.
pub const HCINT_XFRC: u32 = 1 << 0;
/// The channel halted — because software asked, or because of an error.
pub const HCINT_CHH: u32 = 1 << 1;
/// The endpoint stalled.
pub const HCINT_STALL: u32 = 1 << 3;
/// The endpoint had nothing to give, or could not take it.
pub const HCINT_NAK: u32 = 1 << 4;
/// The transaction was acknowledged.
pub const HCINT_ACK: u32 = 1 << 5;
/// A transaction error: nothing answered, or the packet was corrupt.
pub const HCINT_TXERR: u32 = 1 << 7;
/// The device sent more than `MPSIZ`.
pub const HCINT_BBERR: u32 = 1 << 8;
/// Every bit `HCINTn` defines. All of them are write-1-to-clear.
const HCINT_MASK: u32 = 0x7ff;

// -- HCTSIZn ----------------------------------------------------------------

/// Transfer size in bytes, bits 18:0.
const TSIZ_XFRSIZ_MASK: u32 = 0x7_ffff;
/// Packet count, bits 28:19.
const TSIZ_PKTCNT_SHIFT: u32 = 19;
/// The packet-count field's width.
const TSIZ_PKTCNT_MASK: u32 = 0x3ff;
/// The data PID, bits 30:29. The core keeps it current as the toggle advances.
const TSIZ_DPID_SHIFT: u32 = 29;
/// `DPID`: `DATA0`.
const DPID_DATA0: u32 = 0;
/// `DPID`: `DATA1`.
const DPID_DATA1: u32 = 2;
/// `DPID`: `SETUP` on a control endpoint (`MDATA` anywhere else).
const DPID_SETUP: u32 = 3;

// -- GRXSTSR / GRXSTSP ------------------------------------------------------

/// Byte count, bits 14:4.
const RXSTS_BCNT_SHIFT: u32 = 4;
/// Data PID, bits 16:15.
const RXSTS_DPID_SHIFT: u32 = 15;
/// Packet status, bits 20:17.
const RXSTS_PKTSTS_SHIFT: u32 = 17;
/// `PKTSTS`: an `IN` data packet is in the FIFO behind this word.
const PKTSTS_IN_DATA: u32 = 0b0010;
/// `PKTSTS`: the transfer on this channel completed. No bytes follow.
const PKTSTS_IN_COMPLETE: u32 = 0b0011;
/// `PKTSTS`: the channel halted. No bytes follow.
const PKTSTS_CHANNEL_HALTED: u32 = 0b0111;

// -- FIFO sizing ------------------------------------------------------------

/// The depth field of `GRXFSIZ`, in words.
const FIFO_DEPTH_MASK: u32 = 0xffff;
/// Where the depth sits in `GNPTXFSIZ` and `HPTXFSIZ`.
const FIFO_DEPTH_SHIFT: u32 = 16;
/// How many transaction requests the core's request queue holds.
///
/// Reported as entirely free, always, because this model never queues one: a
/// transaction is executed the moment the frame reaches its channel.
const TX_QUEUE_DEPTH: u32 = 8;

/// "Nothing scheduled", as [`Dwc2::next_event_tick`] spells it.
const NO_EVENT: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// The parts of a dwc2 core an SoC gets to choose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Params {
    /// How many host channels, 1 to [`MAX_CHANNELS`]. STM32's OTG_FS has 8.
    pub channels: u8,
    /// How many bidirectional endpoints the **device** side has, 1 to
    /// [`MAX_ENDPOINTS`]. STM32's OTG_FS has 4 — endpoint zero and three more.
    ///
    /// Endpoint zero always exists, so one means "the default pipe and nothing
    /// else", which is a legal if useless gadget.
    pub endpoints: u8,
    /// How many 32-bit words of dedicated FIFO RAM the block has.
    ///
    /// The guest partitions it between the receive FIFO and the two transmit
    /// FIFOs, and this is the ceiling every partition is clamped to — which is
    /// also what keeps a register write from making this device allocate.
    /// STM32's OTG_FS has 1.25 KiB, so 320.
    pub fifo_words: u32,
    /// How many clock-domain ticks one PHY clock takes.
    ///
    /// One when the controller sits on the 48 MHz domain its transceiver runs
    /// at, which is the ordinary case and what makes `HFIR = 48000` a frame of
    /// exactly 48 000 ticks. A board that clocks the block from a faster domain
    /// says so here, rather than anyone reaching for a float (`CLAUDE.md`,
    /// determinism).
    pub phy_ticks: u64,
    /// The fastest this core's transceiver can signal.
    ///
    /// [`Speed::Full`] is an OTG_FS; [`Speed::High`] is the same core with a
    /// high-speed PHY. It decides `HCFG.FSLSS`, and whether a device attached to
    /// the root port can be reached at all.
    pub max_speed: Speed,
    /// What the vendor's user-ID register reads. ST calls it `CID`.
    pub cid: u32,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            channels: 8,
            endpoints: 4,
            fifo_words: 320,
            phy_ticks: 1,
            max_speed: Speed::Full,
            cid: 0,
        }
    }
}

impl Params {
    /// The configuration clamped into what the register fields can express.
    fn clamped(self) -> Params {
        Params {
            channels: self.channels.clamp(1, MAX_CHANNELS as u8),
            endpoints: self.endpoints.clamp(1, MAX_ENDPOINTS as u8),
            fifo_words: self.fifo_words.clamp(1, FIFO_DEPTH_MASK),
            phy_ticks: self.phy_ticks.max(1),
            ..self
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// One host channel: its four programmable registers, and the bytes staged for
/// it by writes to its FIFO window.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Channel {
    hcchar: u32,
    hcsplt: u32,
    hcint: u32,
    hcintmsk: u32,
    hctsiz: u32,
    /// Bytes written into this channel's FIFO window and not yet transmitted.
    ///
    /// A queue rather than a slice because the guest fills it a word at a time
    /// and the controller drains it a packet at a time, and the two are not
    /// synchronised — which is exactly what a transmit FIFO is.
    tx: VecDeque<u8>,
}

impl Channel {
    fn mps(&self) -> u32 {
        self.hcchar & HCCHAR_MPSIZ_MASK
    }

    fn endpoint(&self) -> u8 {
        ((self.hcchar >> HCCHAR_EPNUM_SHIFT) & 0xf) as u8
    }

    fn address(&self) -> DeviceAddress {
        DeviceAddress(((self.hcchar >> HCCHAR_DAD_SHIFT) & 0x7f) as u8)
    }

    fn dir_in(&self) -> bool {
        self.hcchar & HCCHAR_EPDIR != 0
    }

    /// What kind of endpoint this channel is pointed at.
    ///
    /// The two bits mean what they mean in an endpoint descriptor, so the
    /// fabric's own decoder reads them — a small thing, and a real one: the
    /// controller and the device agree on the encoding because neither of them
    /// owns it.
    fn transfer_type(&self) -> TransferType {
        TransferType::from_attribute_bits(((self.hcchar >> HCCHAR_EPTYP_SHIFT) & 0x3) as u8)
    }

    /// Whether this channel is on the periodic side of the frame, and so gets
    /// one transaction per frame rather than as many as the budget allows.
    fn periodic(&self) -> bool {
        matches!(
            self.transfer_type(),
            TransferType::Isochronous | TransferType::Interrupt
        )
    }

    fn armed(&self) -> bool {
        self.hcchar & HCCHAR_CHENA != 0 && self.hcchar & HCCHAR_CHDIS == 0
    }

    fn xfrsiz(&self) -> u32 {
        self.hctsiz & TSIZ_XFRSIZ_MASK
    }

    fn pktcnt(&self) -> u32 {
        (self.hctsiz >> TSIZ_PKTCNT_SHIFT) & TSIZ_PKTCNT_MASK
    }

    fn dpid(&self) -> u32 {
        (self.hctsiz >> TSIZ_DPID_SHIFT) & 0x3
    }

    fn set_xfrsiz(&mut self, value: u32) {
        self.hctsiz = (self.hctsiz & !TSIZ_XFRSIZ_MASK) | (value & TSIZ_XFRSIZ_MASK);
    }

    fn set_pktcnt(&mut self, value: u32) {
        self.hctsiz = (self.hctsiz & !(TSIZ_PKTCNT_MASK << TSIZ_PKTCNT_SHIFT))
            | ((value & TSIZ_PKTCNT_MASK) << TSIZ_PKTCNT_SHIFT);
    }

    fn set_dpid(&mut self, value: u32) {
        self.hctsiz =
            (self.hctsiz & !(0x3 << TSIZ_DPID_SHIFT)) | ((value & 0x3) << TSIZ_DPID_SHIFT);
    }

    /// Stop the channel and say why it stopped (RM0090: `CHH` is raised by a
    /// disable request *or* by an error, and never by a transfer simply
    /// finishing — that is what `XFRC` is for).
    fn halt(&mut self) {
        self.hcchar &= !HCCHAR_CHENA;
        self.hcint |= HCINT_CHH;
    }
}

/// One packet in the receive FIFO: the status word `GRXSTSP` announces it with,
/// and the bytes a FIFO-window read then hands over.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct RxPacket {
    status: u32,
    data: Vec<u8>,
}

/// The single receive FIFO every host channel shares.
///
/// Two levels, because that is how the register interface exposes it: a queue
/// of *announcements*, and the payload of the one that `GRXSTSP` has already
/// popped and that the guest is reading out word by word.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct RxFifo {
    /// Packets waiting to be announced.
    queue: VecDeque<RxPacket>,
    /// The packet `GRXSTSP` last announced.
    current: Option<RxPacket>,
    /// How much of `current` has been read out through a FIFO window.
    read: usize,
}

/// How many 32-bit words `bytes` bytes occupy in a FIFO.
fn words_of(bytes: usize) -> u32 {
    (bytes as u32).div_ceil(4)
}

impl RxFifo {
    /// How many words are occupied, counting the status word each queued packet
    /// still owes.
    fn words(&self) -> u32 {
        let queued: u32 = self
            .queue
            .iter()
            .map(|p| 1 + words_of(p.data.len()))
            .fold(0, u32::saturating_add);
        let current = self
            .current
            .as_ref()
            .map_or(0, |p| words_of(p.data.len().saturating_sub(self.read)));
        queued.saturating_add(current)
    }

    /// Whether `GINTSTS.RXFLVL` is set: there is something to read.
    fn level(&self) -> bool {
        !self.queue.is_empty()
            || self
                .current
                .as_ref()
                .is_some_and(|p| self.read < p.data.len())
    }

    /// `GRXSTSR`: the head of the queue, left where it is.
    fn peek_status(&self) -> u32 {
        self.queue.front().map_or(0, |p| p.status)
    }

    /// `GRXSTSP`: the head of the queue, taken.
    ///
    /// Anything left unread of the previous packet is dropped, which is what
    /// popping a new status word means on the real FIFO: the read pointer moves
    /// to the next packet whether or not the last one was drained.
    fn pop_status(&mut self) -> u32 {
        match self.queue.pop_front() {
            Some(packet) => {
                let status = packet.status;
                self.current = Some(packet);
                self.read = 0;
                status
            }
            None => 0,
        }
    }

    /// The next word of the announced packet, without taking it.
    fn peek_word(&self) -> u32 {
        let Some(packet) = self.current.as_ref() else {
            return 0;
        };
        let mut word = [0u8; 4];
        for (i, slot) in word.iter_mut().enumerate() {
            if let Some(byte) = packet.data.get(self.read + i) {
                *slot = *byte;
            }
        }
        u32::from_le_bytes(word)
    }

    /// The next word of the announced packet, taken.
    fn pop_word(&mut self) -> u32 {
        let word = self.peek_word();
        if let Some(packet) = self.current.as_ref() {
            self.read = (self.read + 4).min(packet.data.len());
        }
        word
    }

    fn clear(&mut self) {
        self.queue.clear();
        self.current = None;
        self.read = 0;
    }
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Domain ticks simulated. The authoritative copy; an atomic mirrors it.
    ticks: u64,
    gotgctl: u32,
    gotgint: u32,
    gahbcfg: u32,
    gusbcfg: u32,
    /// The latched write-1-to-clear half of `GINTSTS`. Everything else in that
    /// register is derived on read.
    gintsts: u32,
    gintmsk: u32,
    grxfsiz: u32,
    gnptxfsiz: u32,
    hptxfsiz: u32,
    gccfg: u32,
    cid: u32,
    hcfg: u32,
    hfir: u32,
    /// `HFNUM.FRNUM`. The remaining-time half of that register is derived from
    /// the tick and never stored.
    frnum: u32,
    hprt: u32,
    haintmsk: u32,
    pcgcctl: u32,
    /// Sixteen always, whatever [`Params::channels`] is: a snapshot's shape
    /// must not depend on a construction property, or a state saved by one
    /// board would not load into another built from the same class.
    channels: [Channel; MAX_CHANNELS],
    rx: RxFifo,
    /// The device-mode register file. Present whichever role is running, for
    /// the same reason the host registers are: the block has both, and a
    /// snapshot's shape must not depend on which one a guest happened to
    /// select.
    dev: device::DeviceState,
}

impl State {
    fn reset(params: Params) -> State {
        State {
            ticks: 0,
            gotgctl: 0,
            gotgint: 0,
            gahbcfg: 0,
            gusbcfg: USBCFG_RESET_VALUE,
            gintsts: 0,
            gintmsk: 0,
            // The three FIFO reset values RM0090 lists. Nothing leans on them:
            // the documented startup sequence programs all three before a
            // channel can move a byte, and this model clamps every depth to the
            // part's dedicated RAM anyway.
            grxfsiz: 0x0000_0200,
            gnptxfsiz: 0x0200_0200,
            hptxfsiz: 0x0200_0600,
            gccfg: 0,
            cid: params.cid,
            hcfg: 0,
            // 0xea60 is the reset value, and it is *not* one millisecond at
            // 48 MHz — the driver writes 48000. Until it does, frames really are
            // 25% long here, which is what the silicon does with the same
            // register.
            hfir: 0xea60,
            // `HFNUM` resets with the frame number all ones, so the first
            // start-of-frame is frame zero.
            frnum: 0x3fff,
            hprt: 0,
            haintmsk: 0,
            pcgcctl: 0,
            channels: core::array::from_fn(|_| Channel::default()),
            rx: RxFifo::default(),
            dev: device::DeviceState::reset(),
        }
    }
}

/// Which `HPRT.PSPD` encoding a speed is.
fn pspd_code(speed: Speed) -> u32 {
    match speed {
        Speed::High => PSPD_HIGH,
        Speed::Full => PSPD_FULL,
        Speed::Low => PSPD_LOW,
    }
}

/// The speed `HPRT.PSPD` is reporting.
fn pspd_speed(hprt: u32) -> Speed {
    match (hprt >> HPRT_PSPD_SHIFT) & 0x3 {
        PSPD_HIGH => Speed::High,
        PSPD_LOW => Speed::Low,
        _ => Speed::Full,
    }
}

/// How many payload bytes one frame of this speed can carry.
///
/// The signalling rate divided by a thousand frames a second, in bytes: 12 Mb/s
/// is 1500 bytes a frame and 1.5 Mb/s is 187. High speed counts a frame's eight
/// microframes together, since this model executes a whole frame at once.
fn bytes_per_frame(speed: Speed) -> u32 {
    match speed {
        Speed::Low => 187,
        Speed::Full => 1500,
        Speed::High => 60_000,
    }
}

/// The receive-FIFO entry that announces a channel halting.
///
/// The core reports a halt through the same queue the data comes out of, so a
/// driver reading `GRXSTSP` sees it there as well as in `HCINTn`.
fn halted_packet(channel: usize) -> RxPacket {
    RxPacket {
        status: (channel as u32) | (PKTSTS_CHANNEL_HALTED << RXSTS_PKTSTS_SHIFT),
        data: Vec::new(),
    }
}

/// The data PID that follows `dpid` (USB 2.0 §8.6.1).
///
/// A `SETUP` is always followed by `DATA1`, which is the rule a control
/// transfer's data stage depends on.
fn next_dpid(dpid: u32) -> u32 {
    match dpid {
        DPID_DATA0 | DPID_SETUP => DPID_DATA1,
        _ => DPID_DATA0,
    }
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// What a register write asks for once the register lock is released.
///
/// The re-entrancy contract (`core::device`): decide under the lock, release,
/// *then* act outward. Every one of these reaches the fabric.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// Nothing but the interrupt refresh every write ends with.
    Nothing,
    /// `GRSTCTL.CSRST`: put everything back.
    CoreReset,
    /// Bring the root port, the register and the fabric back into agreement.
    Port,
    /// `HPRT.PRST` was released: drive the reset and decide who keeps the port.
    FinishReset,
    /// `DCTL.SDIS` moved: put the gadget on the bus, or take it off.
    Gadget,
}

/// A transaction the frame decided to issue.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Job {
    device: DeviceAddress,
    endpoint: u8,
    kind: JobKind,
    /// Whether the channel is on the periodic side, so the frame knows it has
    /// had its turn.
    periodic: bool,
    /// The packet this transaction reserves the bus for. An `IN` is charged its
    /// endpoint's maximum packet size whatever comes back, because that is what
    /// the host had to reserve (USB 2.0 §5.11.3).
    want: u32,
}

/// The three things a transaction can be, which is also the whole of
/// [`crate::bus::usb::UsbDevice`].
#[derive(Debug, Clone, PartialEq, Eq)]
enum JobKind {
    Setup(SetupPacket),
    In,
    Out(Vec<u8>),
}

impl Job {
    /// What this transaction costs the frame: its packet plus the protocol
    /// overhead of USB 2.0 §5.11.3.
    fn cost(&self) -> u32 {
        self.want.saturating_add(PROTOCOL_OVERHEAD_BYTES)
    }
}

/// A DesignWare USB 2.0 OTG core in host mode: the register file, the host
/// channels and the FIFO.
///
/// The **engine**, kept separate from the machine object for the reason
/// [`crate::dev::usb::ehci::Hcd`] is: a variant with a different register
/// placement, or an embedder driving one directly, wants this and not a
/// `Device`.
pub struct Dwc2 {
    bus: Arc<UsbBus>,
    params: Params,
    state: Mutex<State>,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick the next frame falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The level that output is being held at, so a debug read is free.
    irq_level: AtomicU32,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
    /// This core seen from the *other* side of the connector: what a host
    /// enumerates when `GUSBCFG.FDMOD` is selected and `DCTL.SDIS` is clear.
    gadget: Arc<Dwc2Gadget>,
    /// Which port of the bus the gadget plugs into.
    gadget_port: u8,
    /// Whether it is currently plugged in. Not serialized — it is derived from
    /// `GUSBCFG` and `DCTL` and is restored from them.
    attached: AtomicU32,
}

impl fmt::Debug for Dwc2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Dwc2");
        s.field("channels", &self.params.channels);
        s.field("max_speed", &self.params.max_speed);
        match self.state.try_lock() {
            Some(state) => s.field("hprt", &state.hprt).finish_non_exhaustive(),
            None => s.field("state", &"<in use>").finish_non_exhaustive(),
        }
    }
}

impl Dwc2 {
    /// A controller on `bus`, configured by `params`, plugging its device half
    /// into port `gadget_port` of that bus when its firmware selects device
    /// mode.
    ///
    /// Shared from the start, because the gadget half holds a weak reference back
    /// here and a bus that has the gadget plugged into it therefore reaches the
    /// core — without a cycle, which is the point of building it this way
    /// rather than handing out an `Arc` afterwards.
    #[must_use]
    pub fn new(bus: Arc<UsbBus>, params: Params, gadget_port: u8) -> Arc<Dwc2> {
        let params = params.clamped();
        Arc::new_cyclic(|me| Dwc2 {
            bus,
            params,
            state: Mutex::with_rank(HCD_RANK, State::reset(params)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            irq_level: AtomicU32::new(0),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
            gadget: Arc::new(Dwc2Gadget::new(me.clone())),
            gadget_port,
            attached: AtomicU32::new(0),
        })
    }

    /// The device half, for an embedder that wants to plug it in by hand.
    #[must_use]
    pub fn gadget(&self) -> &Arc<Dwc2Gadget> {
        &self.gadget
    }

    /// Which port of the bus the gadget plugs into.
    #[must_use]
    pub fn gadget_port(&self) -> u8 {
        self.gadget_port
    }

    /// Whether the device half is currently on the bus.
    #[must_use]
    pub fn is_attached(&self) -> bool {
        self.attached.load(Ordering::Relaxed) != 0
    }

    /// Put the gadget on the bus, or take it off, to match `GUSBCFG.FDMOD` and
    /// `DCTL.SDIS`.
    ///
    /// **No lock of ours is held**: this reaches the fabric, which then reaches
    /// whatever host is on the far side.
    pub fn settle_gadget(&self) {
        let want = {
            let state = self.state.lock();
            device::soft_connected(&state)
        };
        if want {
            if self
                .attached
                .compare_exchange(0, 1, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                let device = Arc::clone(&self.gadget) as Arc<dyn UsbDevice>;
                if self.bus.attach(self.gadget_port, device).is_err() {
                    // Something else is already in that port. A machine
                    // description bug, and the honest outcome is a device that
                    // is simply not on the bus.
                    self.attached.store(0, Ordering::Relaxed);
                }
            }
        } else if self
            .attached
            .compare_exchange(1, 0, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            self.bus.detach(self.gadget_port);
        }
    }

    /// How this core was configured.
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

    /// `GINTSTS` as the guest would read it.
    #[must_use]
    pub fn interrupt_status(&self) -> u32 {
        let state = self.state.lock();
        self.gintsts(&state)
    }

    /// `HPRT`, for a test that wants to see what the guest would.
    #[must_use]
    pub fn port_status(&self) -> u32 {
        self.state.lock().hprt
    }

    /// `HCINTn` for channel `channel`, or zero for one that does not exist.
    #[must_use]
    pub fn channel_status(&self, channel: u8) -> u32 {
        self.state
            .lock()
            .channels
            .get(usize::from(channel))
            .map_or(0, |c| c.hcint)
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

    // -----------------------------------------------------------------
    // Time
    // -----------------------------------------------------------------

    /// Whether the core is in host mode.
    ///
    /// A dwc2 with a soldered A-plug is a host without being told; `FDMOD` is
    /// the only thing that makes it something else, and this model stops rather
    /// than pretending to be a device.
    fn host_mode(state: &State) -> bool {
        state.gusbcfg & USBCFG_FDMOD == 0
    }

    /// Whether frames are happening at all.
    fn running(state: &State) -> bool {
        Dwc2::host_mode(state) && state.pcgcctl & 1 == 0
    }

    /// How many domain ticks one frame takes.
    fn frame_ticks(&self, state: &State) -> u64 {
        u64::from(state.hfir & 0xffff)
            .max(MIN_FRAME_PHY_CLOCKS)
            .saturating_mul(self.params.phy_ticks)
            .max(1)
    }

    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        let next = if Dwc2::running(state) {
            let ft = self.frame_ticks(state);
            (state.ticks / ft + 1).saturating_mul(ft)
        } else {
            NO_EVENT
        };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// The tick the next frame falls on, if frames are happening.
    #[must_use]
    pub fn next_event_tick(&self) -> Option<u64> {
        match self.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
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
        // A refusal means catch-up is already running further up the stack. The
        // access still has to be answered.
        let _ = handle.sync(kind);
    }

    /// Simulate forward to `target` domain ticks.
    ///
    /// No lock is held across an outward call: each frame decides what to do
    /// under the register lock, releases it, then reaches the fabric.
    pub fn advance_to(&self, target: u64) {
        loop {
            {
                let mut state = self.state.lock();
                if !Dwc2::running(&state) {
                    state.ticks = state.ticks.max(target);
                    self.publish(&state);
                    return;
                }
                let ft = self.frame_ticks(&state);
                let next = (state.ticks / ft + 1).saturating_mul(ft);
                if next > target {
                    state.ticks = state.ticks.max(target);
                    self.publish(&state);
                    return;
                }
                state.ticks = next;
                self.publish(&state);
            }
            self.frame();
        }
    }

    /// One frame: a start-of-frame, then as much of the bus as fits in it.
    fn frame(&self) {
        // The port first: something plugged in between frames has to reach
        // `HPRT` before anything looks at the bus.
        if self.bus.any_change() {
            self.settle_port();
        }
        let frame = {
            let mut state = self.state.lock();
            state.frnum = (state.frnum + 1) & 0x3fff;
            state.gintsts |= GINT_SOF;
            state.frnum
        };
        // The `SOF` token goes out before any other token in the frame does,
        // and it goes to everything on the wire rather than to an address — so
        // it is the fabric's broadcast rather than a transaction. Outside our
        // own lock, like every outward call here.
        self.bus.start_of_frame(frame as u16);
        self.service();
        self.refresh_irq();
    }

    // -----------------------------------------------------------------
    // The root port
    // -----------------------------------------------------------------

    /// Bring `HPRT`, the fabric and whatever is plugged in back into agreement.
    fn settle_port(&self) {
        // Read outside our own lock: `speed` is a call into the device.
        let connected = self.bus.connected(ROOT_PORT);
        let speed = self.bus.speed(ROOT_PORT);
        let plugged_changed = self.bus.take_change(ROOT_PORT);

        let disable = {
            let mut state = self.state.lock();
            let host = Dwc2::host_mode(&state);
            if plugged_changed {
                state.hprt |= HPRT_PCDET;
                if !connected {
                    state.gintsts |= GINT_DISCINT;
                }
            }
            let mut port = state.hprt;
            port = (port & !HPRT_PCSTS) | if connected { HPRT_PCSTS } else { 0 };

            // The line state before any reset is how a host tells a low-speed
            // device from a full-speed one: a low-speed device pulls D- up,
            // which is the K state, `01b` here.
            // A modelled bus has no current, so overcurrent never asserts.
            port &= !(HPRT_POCA | (0x3 << HPRT_PLSTS_SHIFT));
            if connected && speed == Some(Speed::Low) {
                port |= 0x1 << HPRT_PLSTS_SHIFT;
            }

            let mut disable = false;
            if !connected || !host {
                if port & HPRT_PENA != 0 {
                    port |= HPRT_PENCHNG;
                }
                port &= !(HPRT_PENA | HPRT_PRST | HPRT_PSUSP);
                disable = true;
            }
            state.hprt = port;
            disable
        };

        if disable {
            self.bus.set_enabled(ROOT_PORT, false);
        }
    }

    /// Software released `HPRT.PRST`: drive the reset and decide whether this
    /// port can talk to what is on it.
    ///
    /// The speed decision is the honest one and it is not EHCI's. There is no
    /// companion controller to hand an unreachable device to — this core is the
    /// only thing on the pins — and there is no register encoding for "that
    /// device signals faster than this transceiver". So the port simply does
    /// **not** enable and `PENCHNG` does not fire, which is a driver's reset
    /// timing out: the same thing that happens on a bench when the pins cannot
    /// carry what is plugged into them.
    ///
    /// (On real hardware a *high-speed-capable* device would fall back to full
    /// speed, because it starts at full speed and only chirps its way up during
    /// the reset. A device model in this tree declares one fixed speed
    /// ([`crate::bus::usb::UsbDevice::speed`]), so a `Speed::High` model here is
    /// a device that signals high speed and nothing else.)
    pub fn finish_reset(&self) {
        self.bus.reset_port(ROOT_PORT);
        let speed = self.bus.speed(ROOT_PORT);
        let keep = {
            let mut state = self.state.lock();
            let mut port = state.hprt & !HPRT_PRST;
            port &= !(0x3 << HPRT_PSPD_SHIFT);
            let keep = match speed {
                Some(speed) if speed <= self.params.max_speed => {
                    port |= pspd_code(speed) << HPRT_PSPD_SHIFT;
                    if port & HPRT_PENA == 0 {
                        port |= HPRT_PENCHNG;
                    }
                    port |= HPRT_PENA;
                    true
                }
                _ => {
                    if port & HPRT_PENA != 0 {
                        port |= HPRT_PENCHNG;
                    }
                    port &= !HPRT_PENA;
                    false
                }
            };
            state.hprt = port;
            keep
        };
        self.bus.set_enabled(ROOT_PORT, keep);
    }

    // -----------------------------------------------------------------
    // The frame's work
    // -----------------------------------------------------------------

    /// Execute as many transactions as this frame has room for.
    fn service(&self) {
        let (enabled, speed) = {
            let state = self.state.lock();
            (
                state.hprt & HPRT_PENA != 0 && Dwc2::host_mode(&state),
                pspd_speed(state.hprt),
            )
        };
        if !enabled {
            return;
        }

        let mut budget = bytes_per_frame(speed);
        let mut served = [false; MAX_CHANNELS];
        let mut executed = 0usize;

        'frame: loop {
            let mut moved = false;
            for (channel, served) in served
                .iter_mut()
                .enumerate()
                .take(usize::from(self.params.channels))
            {
                let Some(job) = self.prepare(channel, *served) else {
                    continue;
                };
                let cost = job.cost();
                if cost > budget {
                    // The frame is full. Whatever is left is next frame's, which
                    // is what a bus running out of time in a millisecond does.
                    break 'frame;
                }
                budget -= cost;
                if job.periodic {
                    *served = true;
                }
                let (completion, data) = self.execute(&job);
                self.retire(channel, &job, completion, &data);
                moved = true;
                executed += 1;
                if executed >= MAX_TRANSACTIONS_PER_FRAME {
                    break 'frame;
                }
            }
            if !moved {
                break;
            }
        }
    }

    /// Decide what channel `channel` wants to do, under the lock.
    ///
    /// `None` means "not this time": not armed, out of packets, already served
    /// this frame, or waiting for the guest to push more bytes into its FIFO.
    fn prepare(&self, channel: usize, served: bool) -> Option<Job> {
        let mut state = self.state.lock();
        let rx_words = state.rx.words();
        let rx_depth = self.rx_depth(&state);
        let channels = &mut state.channels;
        let c = channels.get_mut(channel)?;
        if !c.armed() || c.pktcnt() == 0 {
            return None;
        }
        let periodic = c.periodic();
        if periodic && served {
            return None;
        }

        let mps = c.mps();
        let device = c.address();
        let endpoint = c.endpoint();

        if c.dir_in() {
            // The core will not fetch a packet it has nowhere to put: one status
            // word plus the packet itself has to fit in the receive FIFO.
            let need = 1 + words_of(mps as usize);
            if rx_words.saturating_add(need) > rx_depth {
                return None;
            }
            return Some(Job {
                device,
                endpoint,
                kind: JobKind::In,
                periodic,
                want: mps,
            });
        }

        let setup = c.dpid() == DPID_SETUP && matches!(c.transfer_type(), TransferType::Control);
        if setup {
            // Eight bytes, always (USB 2.0 §9.3). A channel programmed for fewer
            // is malformed, and inventing the missing bytes would hand the device
            // a request nobody wrote.
            if c.xfrsiz() < SetupPacket::SIZE as u32 {
                c.hcint |= HCINT_TXERR;
                c.halt();
                return None;
            }
            if c.tx.len() < SetupPacket::SIZE as usize {
                return None;
            }
            let mut raw = [0u8; 8];
            for (i, slot) in raw.iter_mut().enumerate() {
                *slot = c.tx[i];
            }
            return Some(Job {
                device,
                endpoint,
                kind: JobKind::Setup(SetupPacket::decode(&raw)),
                periodic,
                want: SetupPacket::SIZE as u32,
            });
        }

        let want = mps.min(c.xfrsiz());
        if (c.tx.len() as u32) < want {
            // The guest has not finished pushing this packet yet.
            return None;
        }
        let data: Vec<u8> = c.tx.iter().copied().take(want as usize).collect();
        Some(Job {
            device,
            endpoint,
            kind: JobKind::Out(data),
            periodic,
            want,
        })
    }

    /// Put one transaction on the bus. **No lock of ours is held here.**
    fn execute(&self, job: &Job) -> (Completion, Vec<u8>) {
        match &job.kind {
            JobKind::Setup(packet) => {
                let status = self.bus.setup(job.device, job.endpoint, *packet);
                (
                    Completion {
                        status,
                        len: SetupPacket::SIZE,
                    },
                    Vec::new(),
                )
            }
            JobKind::In => {
                let mut buf = alloc::vec![0u8; job.want as usize];
                let completion = self.bus.read(job.device, job.endpoint, &mut buf);
                let n = (completion.len as usize).min(buf.len());
                buf.truncate(n);
                (completion, buf)
            }
            JobKind::Out(data) => (self.bus.write(job.device, job.endpoint, data), Vec::new()),
        }
    }

    /// Fold one transaction's outcome back into the channel and the FIFO.
    ///
    /// Every count here moves by saturating arithmetic. The register lock was
    /// released across the transaction, so another CPU's `HCTSIZn` write may
    /// have landed in between, and "the guest reprogrammed the channel
    /// mid-transaction" must be a strange outcome rather than a panic.
    fn retire(&self, channel: usize, job: &Job, completion: Completion, data: &[u8]) {
        let mut state = self.state.lock();
        let depth = self.rx_depth(&state);
        // Filled under the channel borrow, pushed once it is released.
        let mut announce: Vec<RxPacket> = Vec::new();

        {
            let Some(c) = state.channels.get_mut(channel) else {
                return;
            };
            match completion.status {
                Status::Ack => {
                    c.hcint |= HCINT_ACK;
                    match &job.kind {
                        JobKind::Setup(_) => {
                            for _ in 0..SetupPacket::SIZE {
                                c.tx.pop_front();
                            }
                            let moved = (SetupPacket::SIZE as u32).min(c.xfrsiz());
                            c.set_xfrsiz(c.xfrsiz().saturating_sub(moved));
                            c.set_pktcnt(c.pktcnt().saturating_sub(1));
                            // §8.6.1: whatever follows a `SETUP` is `DATA1`.
                            c.set_dpid(DPID_DATA1);
                            if c.pktcnt() == 0 {
                                c.hcint |= HCINT_XFRC;
                            }
                        }
                        JobKind::Out(sent) => {
                            for _ in 0..sent.len() {
                                c.tx.pop_front();
                            }
                            let moved = (sent.len() as u32).min(c.xfrsiz());
                            c.set_xfrsiz(c.xfrsiz().saturating_sub(moved));
                            c.set_pktcnt(c.pktcnt().saturating_sub(1));
                            c.set_dpid(next_dpid(c.dpid()));
                            if c.pktcnt() == 0 {
                                c.hcint |= HCINT_XFRC;
                            }
                        }
                        JobKind::In => {
                            let n = data.len() as u32;
                            if n > c.xfrsiz() {
                                // More than the driver said it would take
                                // (USB 2.0 §8.7.4). Nothing is folded in.
                                c.hcint |= HCINT_BBERR;
                                c.halt();
                                announce.push(halted_packet(channel));
                            } else {
                                let dpid = c.dpid();
                                let short = n < c.mps();
                                c.set_xfrsiz(c.xfrsiz().saturating_sub(n));
                                c.set_pktcnt(c.pktcnt().saturating_sub(1));
                                c.set_dpid(next_dpid(dpid));
                                let complete = short || c.pktcnt() == 0;
                                if complete {
                                    c.hcint |= HCINT_XFRC;
                                }
                                announce.push(RxPacket {
                                    status: (channel as u32)
                                        | (n << RXSTS_BCNT_SHIFT)
                                        | (dpid << RXSTS_DPID_SHIFT)
                                        | (PKTSTS_IN_DATA << RXSTS_PKTSTS_SHIFT),
                                    data: data.to_vec(),
                                });
                                if complete {
                                    announce.push(RxPacket {
                                        status: (channel as u32)
                                            | (dpid << RXSTS_DPID_SHIFT)
                                            | (PKTSTS_IN_COMPLETE << RXSTS_PKTSTS_SHIFT),
                                        data: Vec::new(),
                                    });
                                }
                            }
                        }
                    }
                }
                // Not an error: the endpoint had nothing, and the same packet
                // goes out again — which is why nothing was drained from the
                // transmit staging above (USB 2.0 §8.4.5).
                Status::Nak => c.hcint |= HCINT_NAK,
                Status::Stall => {
                    c.hcint |= HCINT_STALL;
                    c.halt();
                    announce.push(halted_packet(channel));
                }
                Status::Babble => {
                    c.hcint |= HCINT_BBERR;
                    c.halt();
                    announce.push(halted_packet(channel));
                }
                Status::NoDevice | Status::Error => {
                    c.hcint |= HCINT_TXERR;
                    c.halt();
                    announce.push(halted_packet(channel));
                }
            }
        }

        // A FIFO that is full drops what will not fit. It is the only thing a
        // fixed amount of RAM can do, and it is what bounds this queue against a
        // guest that arms a channel over and over and never reads `GRXSTSP`.
        for packet in announce {
            if state.rx.words() >= depth {
                break;
            }
            state.rx.queue.push_back(packet);
        }
    }

    // -----------------------------------------------------------------
    // FIFO sizing
    // -----------------------------------------------------------------

    /// The receive FIFO's depth in words, clamped to the RAM the part has.
    fn rx_depth(&self, state: &State) -> u32 {
        (state.grxfsiz & FIFO_DEPTH_MASK).min(self.params.fifo_words)
    }

    /// A transmit FIFO's depth in words, clamped to the RAM the part has.
    fn tx_depth(&self, state: &State, periodic: bool) -> u32 {
        let raw = if periodic {
            state.hptxfsiz >> FIFO_DEPTH_SHIFT
        } else {
            state.gnptxfsiz >> FIFO_DEPTH_SHIFT
        };
        (raw & FIFO_DEPTH_MASK).min(self.params.fifo_words)
    }

    /// How many words are staged across every channel on one side of the frame.
    fn tx_used(state: &State, periodic: bool, channels: u8) -> u32 {
        state
            .channels
            .iter()
            .take(usize::from(channels))
            .filter(|c| c.periodic() == periodic)
            .map(|c| words_of(c.tx.len()))
            .fold(0, u32::saturating_add)
    }

    // -----------------------------------------------------------------
    // Interrupts
    // -----------------------------------------------------------------

    /// `HAINT`: which channels have an unmasked interrupt pending.
    fn haint(&self, state: &State) -> u32 {
        let mut bits = 0;
        for (index, c) in state
            .channels
            .iter()
            .take(usize::from(self.params.channels))
            .enumerate()
        {
            if c.hcint & c.hcintmsk != 0 {
                bits |= 1u32 << index;
            }
        }
        bits
    }

    /// `GINTSTS` as the guest reads it: the latched half, plus everything this
    /// controller derives rather than storing.
    fn gintsts(&self, state: &State) -> u32 {
        let mut value = state.gintsts;
        if Dwc2::host_mode(state) {
            value |= GINT_CMOD;
        }
        if state.gotgint != 0 {
            value |= GINT_OTGINT;
        }
        if state.rx.level() {
            value |= GINT_RXFLVL;
        }
        if !Dwc2::host_mode(state) {
            // The other role's derived bits. `IEPINT`/`OEPINT` are what a
            // gadget driver's handler is written around, and they come out of
            // `DAINT` exactly as `HCINT` comes out of `HAINT`.
            return value | device::gintsts(self, state);
        }
        if Dwc2::tx_used(state, false, self.params.channels) == 0 {
            value |= GINT_NPTXFE;
        }
        if Dwc2::tx_used(state, true, self.params.channels) == 0 {
            value |= GINT_PTXFE;
        }
        if state.hprt & (HPRT_PCDET | HPRT_PENCHNG | HPRT_POCCHNG) != 0 {
            value |= GINT_HPRTINT;
        }
        if self.haint(state) & state.haintmsk != 0 {
            value |= GINT_HCINT;
        }
        value
    }

    /// Re-derive the interrupt output and drive it.
    ///
    /// Called with no lock of ours held. `GAHBCFG.GINTMSK` is the master
    /// enable: with it clear the pin never rises however loud `GINTSTS` is.
    pub fn refresh_irq(&self) {
        let asserted = {
            let state = self.state.lock();
            state.gahbcfg & AHBCFG_GINTMSK != 0 && self.gintsts(&state) & state.gintmsk != 0
        };
        self.irq_level.store(u32::from(asserted), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(Level::from_bool(asserted));
        }
    }
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

impl Dwc2 {
    /// Whether `offset` belongs to the role the core is *not* currently in.
    ///
    /// RM0090 raises `GINTSTS.MMIS` when the application reaches for a host
    /// register in device mode or the other way round, and it is genuinely
    /// useful: it is how a driver finds out that its `FDMOD`/`FHMOD` write has
    /// not taken effect yet. The FIFO windows are not in either block — they
    /// belong to whichever role is running.
    fn wrong_role(state: &State, offset: u64) -> bool {
        if Dwc2::host_mode(state) {
            (DEVICE_BASE..DEVICE_END).contains(&offset)
        } else {
            (HCFG..DEVICE_BASE).contains(&offset)
        }
    }

    /// Read one 32-bit register, or one word out of a FIFO window.
    ///
    /// `debug` is [`MemAttrs::debug`]: it makes `GRXSTSP` peek instead of pop
    /// and a FIFO read leave the read pointer alone, which is the whole content
    /// of "a debugger must not consume the guest's data".
    #[must_use]
    pub fn read(&self, offset: u64, debug: bool) -> u32 {
        let offset = offset & !0x3;
        let mut state = self.state.lock();

        if offset >= FIFO_BASE {
            let window = (offset - FIFO_BASE) / FIFO_WINDOW;
            if window >= MAX_CHANNELS as u64 {
                return 0;
            }
            // One receive FIFO is shared by every channel, so which window the
            // read came through does not change the answer — the window number
            // matters on the *write* side, which is per channel.
            return if debug {
                state.rx.peek_word()
            } else {
                state.rx.pop_word()
            };
        }

        if !debug && Dwc2::wrong_role(&state, offset) {
            state.gintsts |= GINT_MMIS;
        }

        if (DEVICE_BASE..DEVICE_END).contains(&offset) {
            return device::read(self, &state, offset);
        }

        if let Some(endpoint) = device::tx_fifo_register(self, offset) {
            return device::read_tx_fifo(&state, endpoint);
        }

        if (HCCHAR_BASE..DEVICE_BASE).contains(&offset) {
            let index = ((offset - HCCHAR_BASE) / CHANNEL_STRIDE) as usize;
            let register = (offset - HCCHAR_BASE) % CHANNEL_STRIDE;
            if index >= usize::from(self.params.channels) {
                return 0;
            }
            let channel = &state.channels[index];
            return match register {
                0x00 => channel.hcchar,
                0x04 => channel.hcsplt,
                0x08 => channel.hcint,
                0x0c => channel.hcintmsk,
                0x10 => channel.hctsiz,
                // `HCDMAn`: this core has no DMA, and the register does not
                // exist on the full-speed part.
                _ => 0,
            };
        }

        match offset {
            GOTGCTL => {
                let mut value = state.gotgctl & OTGCTL_WRITABLE;
                if Dwc2::host_mode(&state) {
                    value |= OTGCTL_SRQSCS | OTGCTL_ASVLD;
                } else {
                    value |= OTGCTL_CIDSTS | OTGCTL_BSVLD;
                }
                value
            }
            GOTGINT => state.gotgint,
            GAHBCFG => state.gahbcfg & AHBCFG_WRITABLE,
            GUSBCFG => {
                let mut value = state.gusbcfg & USBCFG_WRITABLE;
                if self.params.max_speed != Speed::High {
                    // There is no other transceiver to select.
                    value |= USBCFG_PHYSEL;
                }
                value
            }
            // Every reset bit here is self-clearing and this model completes
            // each one inside the write, so what is left to read is the
            // AHB-idle flag a driver spins on.
            GRSTCTL => RSTCTL_AHBIDL,
            GINTSTS => self.gintsts(&state),
            GINTMSK => state.gintmsk,
            GRXSTSR => state.rx.peek_status(),
            GRXSTSP => {
                if debug {
                    state.rx.peek_status()
                } else {
                    state.rx.pop_status()
                }
            }
            GRXFSIZ => state.grxfsiz,
            GNPTXFSIZ => state.gnptxfsiz,
            GNPTXSTS => {
                // In device mode this register describes endpoint zero's
                // transmit FIFO, which is the one `GNPTXFSIZ` sizes — the same
                // register, read from whichever side is transmitting.
                let free = if Dwc2::host_mode(&state) {
                    self.tx_depth(&state, false).saturating_sub(Dwc2::tx_used(
                        &state,
                        false,
                        self.params.channels,
                    ))
                } else {
                    device::tx_depth(self, &state, 0)
                        .saturating_sub(words_of(state.dev.din[0].tx.len()))
                };
                free | (TX_QUEUE_DEPTH << 16)
            }
            GCCFG => state.gccfg,
            CID => state.cid,
            HPTXFSIZ => state.hptxfsiz,
            HCFG => {
                let mut value = state.hcfg & HCFG_FSLSPCS_MASK;
                if self.params.max_speed != Speed::High {
                    value |= HCFG_FSLSS;
                }
                value
            }
            HFIR => state.hfir & 0xffff,
            HFNUM => {
                let ft = self.frame_ticks(&state);
                let remaining = ft - (state.ticks % ft);
                state.frnum | ((remaining.min(0xffff) as u32) << 16)
            }
            HPTXSTS => {
                let free = self.tx_depth(&state, true).saturating_sub(Dwc2::tx_used(
                    &state,
                    true,
                    self.params.channels,
                ));
                free | (TX_QUEUE_DEPTH << 16)
            }
            HAINT => self.haint(&state),
            HAINTMSK => state.haintmsk,
            HPRT => state.hprt,
            PCGCCTL => state.pcgcctl,
            _ => 0,
        }
    }

    /// Write one 32-bit register, or push one word into a FIFO window.
    ///
    /// Returns what has to happen once the register lock is released; hand it
    /// to [`Dwc2::act`].
    pub fn write(&self, offset: u64, value: u32) -> After {
        let offset = offset & !0x3;
        let mut state = self.state.lock();

        if offset >= FIFO_BASE {
            let window = ((offset - FIFO_BASE) / FIFO_WINDOW) as usize;
            if !Dwc2::host_mode(&state) {
                // In device mode window *n* is `IN` endpoint *n*'s transmit
                // FIFO, not channel *n*'s: the windows belong to whichever role
                // is running, which is why they raise no mode mismatch.
                device::push_word(self, &mut state, window, value);
                return After::Nothing;
            }
            if window >= usize::from(self.params.channels) {
                return After::Nothing;
            }
            let periodic = state.channels[window].periodic();
            let depth = self.tx_depth(&state, periodic);
            let used = Dwc2::tx_used(&state, periodic, self.params.channels);
            // A FIFO that is full drops what does not fit, which is the only
            // thing a fixed amount of RAM can do — and is what bounds this
            // device's memory against a guest that writes for ever.
            if used < depth {
                state.channels[window].tx.extend(value.to_le_bytes());
            }
            return After::Nothing;
        }

        if Dwc2::wrong_role(&state, offset) {
            state.gintsts |= GINT_MMIS;
        }

        let mut after = After::Nothing;

        if (DEVICE_BASE..DEVICE_END).contains(&offset) {
            let connect = device::write(self, &mut state, offset, value);
            self.publish(&state);
            return if connect {
                After::Gadget
            } else {
                After::Nothing
            };
        }

        if let Some(endpoint) = device::tx_fifo_register(self, offset) {
            device::write_tx_fifo(&mut state, endpoint, value);
            self.publish(&state);
            return After::Nothing;
        }

        if (HCCHAR_BASE..DEVICE_BASE).contains(&offset) {
            let index = ((offset - HCCHAR_BASE) / CHANNEL_STRIDE) as usize;
            let register = (offset - HCCHAR_BASE) % CHANNEL_STRIDE;
            if index < usize::from(self.params.channels) {
                let channel = &mut state.channels[index];
                match register {
                    0x00 => {
                        channel.hcchar = value;
                        if value & HCCHAR_CHDIS != 0 {
                            // The disable request is the *only* way a channel
                            // halts on purpose, and it is answered immediately:
                            // there is no transaction in flight to wait for,
                            // because a transaction here is executed to
                            // completion inside a frame.
                            channel.halt();
                        }
                    }
                    0x04 => channel.hcsplt = value,
                    0x08 => channel.hcint &= !(value & HCINT_MASK),
                    0x0c => channel.hcintmsk = value & HCINT_MASK,
                    0x10 => channel.hctsiz = value,
                    _ => {}
                }
            }
            self.publish(&state);
            return after;
        }

        match offset {
            GOTGCTL => state.gotgctl = value & OTGCTL_WRITABLE,
            GOTGINT => state.gotgint &= !value,
            GAHBCFG => state.gahbcfg = value & AHBCFG_WRITABLE,
            GUSBCFG => {
                let was_host = Dwc2::host_mode(&state);
                state.gusbcfg = value & USBCFG_WRITABLE;
                if Dwc2::host_mode(&state) != was_host {
                    state.gintsts |= GINT_CIDSCHG;
                    after = After::Port;
                }
            }
            GRSTCTL => {
                if value & RSTCTL_CSRST != 0 {
                    after = After::CoreReset;
                } else {
                    if value & RSTCTL_RXFFLSH != 0 {
                        state.rx.clear();
                    }
                    if value & RSTCTL_TXFFLSH != 0 {
                        // In host mode `TXFNUM` names a FIFO, not a channel:
                        // zero is the non-periodic one, one is the periodic one,
                        // and 0x10 is all of them.
                        let which = (value >> RSTCTL_TXFNUM_SHIFT) & 0x1f;
                        for channel in state.channels.iter_mut() {
                            let mine = match which {
                                0 => !channel.periodic(),
                                1 => channel.periodic(),
                                0x10 => true,
                                _ => false,
                            };
                            if mine {
                                channel.tx.clear();
                            }
                        }
                    }
                    if value & RSTCTL_FCRST != 0 {
                        state.frnum = 0x3fff;
                    }
                    // `HSRST` (bit 1) resets the AHB clock domain's state
                    // machines. There is no register here that a bus-clock reset
                    // would change and a core reset would not, so it is accepted
                    // and self-clears like the rest, and nothing is invented to
                    // give it something to do.
                }
            }
            GINTSTS => state.gintsts &= !(value & GINT_W1C),
            GINTMSK => state.gintmsk = value,
            GRXFSIZ => state.grxfsiz = value & FIFO_DEPTH_MASK,
            GNPTXFSIZ => state.gnptxfsiz = value,
            GCCFG => state.gccfg = value,
            CID => state.cid = value,
            HPTXFSIZ => state.hptxfsiz = value,
            HCFG => state.hcfg = value & HCFG_FSLSPCS_MASK,
            HFIR => state.hfir = value & 0xffff,
            HAINTMSK => state.haintmsk = value & 0xffff,
            HPRT => {
                let old = state.hprt;
                let mut port = old;
                // Write-1-to-clear, and `PENA` is in that set: writing a one
                // there disables the port rather than enabling it.
                port &= !(value & HPRT_W1C);
                port = (port & !HPRT_WRITABLE) | (value & HPRT_WRITABLE);
                if port & HPRT_PRST != 0 {
                    // The port is disabled for as long as reset is asserted.
                    port &= !HPRT_PENA;
                }
                state.hprt = port;
                after = if old & HPRT_PRST != 0 && port & HPRT_PRST == 0 {
                    After::FinishReset
                } else if (old ^ port) & (HPRT_PENA | HPRT_PRST | HPRT_PPWR) != 0 {
                    After::Port
                } else {
                    After::Nothing
                };
            }
            PCGCCTL => state.pcgcctl = value,
            // Read-only, or reserved, or the device block this model does not
            // have: accepted and dropped, which is what silicon does with a
            // write to a read-only register.
            _ => {}
        }

        self.publish(&state);
        after
    }

    /// Do what a register write asked for, with no register lock held.
    pub fn act(&self, after: After) {
        match after {
            After::Nothing => {}
            After::CoreReset => self.core_reset(),
            After::Port => {
                let enabled = {
                    let state = self.state.lock();
                    state.hprt & HPRT_PENA != 0 && Dwc2::host_mode(&state)
                };
                self.bus.set_enabled(ROOT_PORT, enabled);
                self.settle_port();
                // A role change is also a connect or a disconnect on the far
                // side of the same connector.
                self.settle_gadget();
            }
            After::FinishReset => self.finish_reset(),
            After::Gadget => self.settle_gadget(),
        }
    }

    /// `GRSTCTL.CSRST`: everything back to its reset value, the tick apart.
    ///
    /// The role select goes back with it. `GUSBCFG` is a core register and a
    /// core reset resets the core; the same reading settled the equivalent
    /// question for [`crate::dev::usb::chipidea`], and here it costs nothing
    /// because a dwc2 with no `FDMOD` is a host anyway.
    fn core_reset(&self) {
        {
            let mut state = self.state.lock();
            let ticks = state.ticks;
            *state = State {
                ticks,
                ..State::reset(self.params)
            };
            self.publish(&state);
        }
        self.bus.set_enabled(ROOT_PORT, false);
        self.settle_port();
        // `DCTL.SDIS` is set again, so the gadget comes off the bus: a core
        // reset really does unplug the device, which is what a host sees.
        self.settle_gadget();
        self.refresh_irq();
    }

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    /// Return to the documented reset state.
    ///
    /// The tick is **not** rewound: `Machine::reset` does not rewind clock
    /// domains, and a lazily advanced device that zeroed its own tick would
    /// claim to be in the past for ever after.
    pub fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.state.lock();
            let ticks = state.ticks;
            *state = State {
                ticks,
                ..State::reset(self.params)
            };
            self.publish(&state);
        }
        self.bus.set_enabled(ROOT_PORT, false);
        self.settle_port();
        self.settle_gadget();
        self.refresh_irq();
    }

    /// Serialize the register file, the channels and the FIFO.
    ///
    /// # What a snapshot mid-transfer means
    ///
    /// Rather more than it does for an EHCI, and that is why this is longer
    /// than a list of registers. An EHCI's durable state is entirely in guest
    /// memory — the queue head, the overlay, the descriptor — so the RAM device
    /// saves it. This core's is not: the bytes of a half-pushed `OUT` packet
    /// live in a **transmit FIFO inside the controller**, and a received packet
    /// the guest has not finished reading out of `GRXSTSP` lives in the receive
    /// FIFO. Both are saved here, down to how many bytes of the announced packet
    /// the guest has already taken, because a snapshot that dropped them would
    /// restore a machine whose next FIFO read returns something else.
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let state = self.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u32(state.gotgctl)?;
        w.write_u32(state.gotgint)?;
        w.write_u32(state.gahbcfg)?;
        w.write_u32(state.gusbcfg)?;
        w.write_u32(state.gintsts)?;
        w.write_u32(state.gintmsk)?;
        w.write_u32(state.grxfsiz)?;
        w.write_u32(state.gnptxfsiz)?;
        w.write_u32(state.hptxfsiz)?;
        w.write_u32(state.gccfg)?;
        w.write_u32(state.cid)?;
        w.write_u32(state.hcfg)?;
        w.write_u32(state.hfir)?;
        w.write_u32(state.frnum)?;
        w.write_u32(state.hprt)?;
        w.write_u32(state.haintmsk)?;
        w.write_u32(state.pcgcctl)?;

        w.write_seq_len(MAX_CHANNELS as u64)?;
        for channel in &state.channels {
            w.write_u32(channel.hcchar)?;
            w.write_u32(channel.hcsplt)?;
            w.write_u32(channel.hcint)?;
            w.write_u32(channel.hcintmsk)?;
            w.write_u32(channel.hctsiz)?;
            let staged: Vec<u8> = channel.tx.iter().copied().collect();
            w.write_bytes(&staged)?;
        }

        w.write_seq_len(state.rx.queue.len() as u64)?;
        for packet in &state.rx.queue {
            w.write_u32(packet.status)?;
            w.write_bytes(&packet.data)?;
        }
        match &state.rx.current {
            Some(packet) => {
                w.write_bool(true)?;
                w.write_u32(packet.status)?;
                w.write_bytes(&packet.data)?;
                w.write_u64(state.rx.read as u64)?;
            }
            None => w.write_bool(false)?,
        }

        // The device half, always — a snapshot's shape does not depend on which
        // role the guest happened to have selected when it was taken.
        device::save(&state.dev, w)
    }

    /// Restore what [`save`](Dwc2::save) wrote.
    ///
    /// # Errors
    ///
    /// [`Error::State`] for a truncated or malformed chunk.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let mut restored = State {
            ticks: r.read_u64()?,
            gotgctl: r.read_u32()?,
            gotgint: r.read_u32()?,
            gahbcfg: r.read_u32()?,
            gusbcfg: r.read_u32()?,
            gintsts: r.read_u32()?,
            gintmsk: r.read_u32()?,
            grxfsiz: r.read_u32()?,
            gnptxfsiz: r.read_u32()?,
            hptxfsiz: r.read_u32()?,
            gccfg: r.read_u32()?,
            cid: r.read_u32()?,
            hcfg: r.read_u32()?,
            hfir: r.read_u32()?,
            frnum: r.read_u32()?,
            hprt: r.read_u32()?,
            haintmsk: r.read_u32()?,
            pcgcctl: r.read_u32()?,
            channels: core::array::from_fn(|_| Channel::default()),
            rx: RxFifo::default(),
            dev: device::DeviceState::reset(),
        };

        let count = r.read_seq_len(24)?;
        if count != MAX_CHANNELS as u64 {
            return Err(Error::State(alloc::format!(
                "usb.dwc2: a snapshot with {count} channels, not {MAX_CHANNELS}"
            )));
        }
        for channel in &mut restored.channels {
            channel.hcchar = r.read_u32()?;
            channel.hcsplt = r.read_u32()?;
            channel.hcint = r.read_u32()?;
            channel.hcintmsk = r.read_u32()?;
            channel.hctsiz = r.read_u32()?;
            channel.tx = r.read_bytes()?.iter().copied().collect();
        }

        let packets = r.read_seq_len(12)?;
        for _ in 0..packets {
            let status = r.read_u32()?;
            let data = r.read_bytes()?.to_vec();
            restored.rx.queue.push_back(RxPacket { status, data });
        }
        if r.read_bool()? {
            let status = r.read_u32()?;
            let data = r.read_bytes()?.to_vec();
            let read = r.read_u64()? as usize;
            restored.rx.read = read.min(data.len());
            restored.rx.current = Some(RxPacket { status, data });
        }
        restored.dev = device::load(r)?;

        {
            let mut state = self.state.lock();
            *state = restored;
            self.publish(&state);
        }
        // The fabric's enable bit is derived state and is never serialized
        // (`ROADMAP.md` §4.5): it comes back from `HPRT`.
        let enabled = {
            let state = self.state.lock();
            state.hprt & HPRT_PENA != 0 && Dwc2::host_mode(&state)
        };
        self.bus.set_enabled(ROOT_PORT, enabled);
        // So is whether the gadget is plugged in: it comes back from
        // `GUSBCFG.FDMOD` and `DCTL.SDIS`, never from the chunk.
        self.settle_gadget();
        self.refresh_irq();
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// A dwc2 host controller as a machine object.
#[derive(Debug)]
pub struct Dwc2Controller {
    core: Arc<Dwc2>,
    region: RegionRef,
}

impl Dwc2Controller {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this controller is the root of. Required.
    /// * `channels` — how many host channels, 1 to 16. Defaults to 8, which is
    ///   what STM32's OTG_FS has.
    /// * `endpoints` — how many device-mode endpoints, 1 to 16. Defaults to 4,
    ///   which is what STM32's OTG_FS has.
    /// * `port` — which port of `bus` the device half plugs into when firmware
    ///   selects device mode. Defaults to 0, the port a host controller on the
    ///   same bus roots.
    /// * `fifo` — 32-bit words of dedicated FIFO RAM. Defaults to 320, which is
    ///   the 1.25 KiB of an OTG_FS.
    /// * `phyclock` — clock-domain ticks in one PHY clock. Defaults to 1, the
    ///   controller sitting on its own 48 MHz domain.
    /// * `speed` — `"full"` (default) for an OTG_FS, `"high"` for the same core
    ///   behind a high-speed PHY.
    /// * `cid` — what the vendor's user-ID register reads. Defaults to zero,
    ///   because the value belongs to the part rather than to the core.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property, [`Error::Config`]
    /// for a value outside its range or a bus that is already smaller than this
    /// controller needs.
    pub fn new(props: &Props) -> Result<Dwc2Controller> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let channels = r.or_range("channels", 8u64, 1..=MAX_CHANNELS as u64)?;
        let endpoints = r.or_range("endpoints", 4u64, 1..=MAX_ENDPOINTS as u64)?;
        let port = r.or_range("port", u64::from(ROOT_PORT), 0..=u64::from(u8::MAX))?;
        let fifo = r.or_range("fifo", 320u64, 1..=u64::from(FIFO_DEPTH_MASK))?;
        let phyclock = r.or_range("phyclock", 1u64, 1..=u64::from(u32::MAX))?;
        let speed = r.or_str("speed", "full")?.to_string();
        let cid = r.or_range("cid", 0u64, 0..=u64::from(u32::MAX))?;
        r.finish()?;

        let max_speed = match speed.as_str() {
            "full" => Speed::Full,
            "high" => Speed::High,
            other => {
                return Err(Error::Config {
                    at: String::from(CLASS_NAME),
                    message: alloc::format!(
                        "`speed` is the fastest this core's transceiver can signal, so it is \
                         `full` or `high`, not `{other}`"
                    ),
                });
            }
        };

        // One root port: `HPRT` is a single register, and a second device needs
        // a hub this tree does not model. A board that wires this core as a
        // *device* asks for a port past that, and the bus grows to hold it.
        let bus = buses::attach(props, &bus_name, (port as u8).saturating_add(1))?;
        Ok(Dwc2Controller::with_bus_at(
            bus,
            Params {
                channels: channels as u8,
                endpoints: endpoints as u8,
                fifo_words: fifo as u32,
                phy_ticks: phyclock,
                max_speed,
                cid: cid as u32,
            },
            port as u8,
        ))
    }

    /// A controller on a bus the caller already holds, its device half on port
    /// zero of it.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>, params: Params) -> Dwc2Controller {
        Dwc2Controller::with_bus_at(bus, params, ROOT_PORT)
    }

    /// The same, with the device half on a named port.
    #[must_use]
    pub fn with_bus_at(bus: Arc<UsbBus>, params: Params, gadget_port: u8) -> Dwc2Controller {
        let core = Dwc2::new(bus, params, gadget_port);
        let port = Arc::new(Dwc2Port {
            core: Arc::clone(&core),
        });
        let region = Arc::new(Region::io("dwc2", REGISTER_BYTES, port as Arc<dyn MemOps>));
        Dwc2Controller { core, region }
    }

    /// The engine underneath.
    #[must_use]
    pub fn core(&self) -> &Arc<Dwc2> {
        &self.core
    }
}

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output. Level-triggered, and the AND of `GINTSTS` with
    /// `GINTMSK`, gated by `GAHBCFG.GINTMSK`.
    pub const IRQ: &str = "irq";
}

/// The register block and the FIFO windows, as something an address space
/// dispatches to.
struct Dwc2Port {
    core: Arc<Dwc2>,
}

impl fmt::Debug for Dwc2Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Dwc2Port").finish_non_exhaustive()
    }
}

impl MemOps for Dwc2Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.core.sync_for(attrs);
        let value = self.core.read(offset, attrs.debug);
        if !attrs.debug {
            // A read can change the interrupt: popping `GRXSTSP` clears
            // `RXFLVL`, and touching the other role's registers sets `MMIS`.
            self.core.refresh_irq();
        }
        narrow_read(offset, value, dst)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // `HCINTn` is write-1-to-clear, `GRSTCTL` resets the core,
            // `HCCHAR.CHENA` starts a transaction and a FIFO write puts bytes
            // on the wire. None of that has a harmless version
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let Some(value) = word_write(src) else {
            return Err(BusError::BadAccess);
        };
        self.core.sync_for(attrs);
        let after = self.core.write(offset, value);
        self.core.act(after);
        self.core.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Reads may be narrow — a driver reads `CID` a byte at a time as
        // readily as a word — but every register in this block is 32 bits and a
        // FIFO port is a word, so writes are checked separately.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

/// Answer a 1-, 2- or 4-byte read out of the dword at `offset & !3`.
fn narrow_read(offset: u64, value: u32, dst: &mut [u8]) -> MemResult {
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

/// The dword a register write carries, or `None` for a width this block does
/// not accept.
fn word_write(src: &[u8]) -> Option<u32> {
    (src.len() == 4).then(|| u32::from_le_bytes([src[0], src[1], src[2], src[3]]))
}

impl Device for Dwc2Controller {
    fn class(&self) -> &'static DeviceClass {
        &DWC2_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A `map` statement places the region and a `wire` statement connects
        // the interrupt, so neither of those is here. What *is* an outward
        // action is the device half plugging into the bus — and out of reset
        // `DCTL.SDIS` is set, so realize plugs nothing in. It is here because
        // this is where an outward action belongs, and a `load` before the
        // first register write must find the fabric already agreeing with the
        // registers.
        self.core.settle_gadget();
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        self.core.reset(kind);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.core.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.core.load(r)
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!("a dwc2 controller drives `{}` and nothing else", pin::IRQ),
            });
        }
        self.core.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.core.refresh_irq();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. Frames happen on their own, and a guest that polls `GINTSTS` has to
    /// see the answer at the cycle it polled.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.core.ticks()
    }

    fn advance_to(&self, tick: u64) {
        self.core.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.core.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.core.attach_lazy(handle);
    }
}

/// Nothing to bind: this core is not a bus master.
///
/// That is worth a sentence rather than an empty impl, because it is the
/// clearest difference between this controller and the EHCI beside it. An EHCI
/// *must* be given `space =` — it reads its own work out of guest memory. A
/// dwc2 in slave mode never issues a memory access at all: every byte it moves
/// was put there by the CPU, one word at a time, through a FIFO window.
impl Instance for Dwc2Controller {}

/// The `usb.dwc2` device class.
pub static DWC2_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Synopsys DesignWare USB 2.0 OTG host controller (dwc2), as STM32's OTG_FS \
              instantiates it: host channels and a shared FIFO, with no DMA and no schedule \
              in guest memory",
    properties: DWC2_PROPERTIES,
    construct: |props| Ok(Box::new(Dwc2Controller::new(props)?)),
};

/// The properties [`DWC2_CLASS`] accepts.
static DWC2_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "bus",
        kind: ValueKind::Str,
        required: true,
        summary: "the named USB bus this controller is the root of",
    },
    PropertySpec {
        name: "channels",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many host channels, 1 to 16 (default 8, an STM32 OTG_FS)",
    },
    PropertySpec {
        name: "endpoints",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many device-mode endpoints, 1 to 16 (default 4, an STM32 OTG_FS)",
    },
    PropertySpec {
        name: "port",
        kind: ValueKind::Uint,
        required: false,
        summary: "which port of the bus the device half plugs into (default 0)",
    },
    PropertySpec {
        name: "fifo",
        kind: ValueKind::Uint,
        required: false,
        summary: "32-bit words of dedicated FIFO RAM (default 320, the 1.25 KiB of an OTG_FS)",
    },
    PropertySpec {
        name: "phyclock",
        kind: ValueKind::Uint,
        required: false,
        summary: "clock-domain ticks in one PHY clock (default 1, the block on its 48 MHz domain)",
    },
    PropertySpec {
        name: "speed",
        kind: ValueKind::Str,
        required: false,
        summary: "the fastest the transceiver can signal: `full` (default) or `high`",
    },
    PropertySpec {
        name: "cid",
        kind: ValueKind::Uint,
        required: false,
        summary: "what the vendor's user-ID register reads (default 0)",
    },
];

/// Add [`DWC2_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&DWC2_CLASS)
}

/// Bind [`DWC2_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| {
        Ok(Arc::new(Dwc2Controller::new(props)?))
    })
}

/// What the validator should know about `usb.dwc2`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("channels", ValueKind::Uint).range(1, MAX_CHANNELS as u64))
        .prop(PropSchema::new("endpoints", ValueKind::Uint).range(1, MAX_ENDPOINTS as u64))
        .prop(PropSchema::new("port", ValueKind::Uint).range(0, u64::from(u8::MAX)))
        .prop(PropSchema::new("fifo", ValueKind::Uint).range(1, u64::from(FIFO_DEPTH_MASK)))
        .prop(PropSchema::new("phyclock", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("speed", ValueKind::Str).values(&["full", "high"]))
        .prop(PropSchema::new("cid", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
}
