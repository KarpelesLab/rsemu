//! The other half of the core: **device mode**, where the guest is the
//! peripheral and somebody else is the host.
//!
//! # What changes, and what does not
//!
//! Nothing about the block's shape. The same `GRSTCTL`, the same shared receive
//! FIFO, the same `GRXSTSP` announcing packets one at a time, the same
//! `GINTSTS` collapsing to one pin. What changes is the *direction of the
//! arrows*: host channels become **endpoints**, and instead of the core putting
//! a token on the wire because a channel was armed, the core answers a token
//! somebody else put there.
//!
//! ```text
//!   host (elsewhere)          this core, in device mode           guest
//!   ────────────────          ─────────────────────────           ─────
//!   SETUP  ─────────────────► RX FIFO: SETUP data + complete ──► GRXSTSP, DFIFO
//!                             DOEPINT0.STUP ──► DAINT ──► GINTSTS.OEPINT ──► irq
//!   IN     ─────────────────► DIEPCTLn.EPENA?  the staged packet ◄── DFIFO(n)
//!          ◄──── the reply    DIEPINTn.XFRC ──► DAINT ──► GINTSTS.IEPINT
//!   OUT    ─────────────────► DOEPCTLn.EPENA?  RX FIFO ──────────► GRXSTSP, DFIFO
//! ```
//!
//! # It is a [`UsbDevice`], not a [`Peripheral`](crate::bus::usb::Peripheral)
//!
//! That is the whole seam question, and it is worth stating plainly because the
//! tempting answer is wrong. [`crate::bus::usb::Peripheral`] is
//! [`crate::bus::usb::Endpoint0`] wrapped around a
//! [`crate::bus::usb::Function`]: it answers the eleven standard requests of
//! USB 2.0 §9.4 **in the emulator**, out of a descriptor table the emulator
//! holds. That is exactly what must *not* happen here. In device mode the
//! guest's firmware owns `GET_DESCRIPTOR`, owns `SET_ADDRESS`, owns the
//! descriptors, and gets them wrong in its own way if it is buggy — which is
//! the entire point of emulating it.
//!
//! So [`Dwc2Gadget`] implements [`UsbDevice`] directly. The trait's own
//! documentation anticipated this: *"implemented by `Peripheral` for anything
//! built the ordinary way, and directly by anything that genuinely is not,
//! which so far is nothing."* This is the something.
//!
//! # NAK is the whole synchronisation story
//!
//! A transaction arrives synchronously — a host controller, or a host-side
//! driver, calls straight into [`UsbDevice::transfer_in`] — and the guest is
//! not running at that instant. It cannot be: it is the same machine, or
//! another one, and either way it makes progress between transactions and not
//! during them.
//!
//! Nothing needs to be invented for that, because USB already has the answer. A
//! device that has not queued its reply yet answers `NAK`, and the host comes
//! back next frame. So the rule here is that **an endpoint that is not armed
//! NAKs**, and a firmware that has armed `DIEPCTLn.EPENA` and pushed its bytes
//! into the FIFO is one whose next `IN` gets them. No queue, no callback, no
//! deferred completion.
//!
//! # Soft connect is what puts the device on the bus
//!
//! `DCTL.SDIS` resets **set** — the core comes up soft-disconnected, and RM0090
//! gives `DCTL` the reset value `0x0000_0002` for that reason. So a board that
//! instantiates this class does not have a device on the bus until its firmware
//! clears that bit, which is exactly what a real board does with the pull-up on
//! D+. Clearing it is what calls
//! [`UsbBus::attach`](crate::bus::usb::UsbBus::attach); setting it again, or
//! selecting host mode, is what calls `detach`.
//!
//! # What is deliberately not here
//!
//! * **Dedicated `SETUP` back-to-back handling.** `DOEPTSIZ0.STUPCNT` stores,
//!   reads back and is decremented, and `DOEPINT.B2BSTUP` is defined and never
//!   raised: this model delivers one setup packet per transaction and there is
//!   nothing that can arrive behind it.
//! * **Global `IN`/`OUT` NAK.** `DCTL.SGINAK`/`CGINAK`/`SGONAK`/`CGONAK` set and
//!   clear `DCTL.GINSTS`/`GONSTS` and `GINTSTS.GINAKEFF`/`GONAKEFF`, and the
//!   global NAK state **is** honoured: with it set, every endpoint NAKs. What
//!   is not modelled is the `PKTSTS = 0001b` "global OUT NAK" entry the core
//!   pushes into the receive FIFO, because nothing here can be mid-packet when
//!   the bit is set.
//! * **Remote wakeup and suspend.** `DCTL.RWUSIG` stores and reads back;
//!   `DSTS.SUSPSTS` reads zero. There is no idle bus in this fabric to be
//!   suspended from — a modelled host that stops issuing transactions is
//!   indistinguishable from one that is busy — so inventing a suspend timer
//!   would be inventing an event.
//! * **Isochronous frame parity and the data toggle.** `DIEPCTLn.SD0PID` and
//!   `SODDFRM` are accepted and dropped: they are self-clearing selectors, and
//!   this fabric carries no PID on an endpoint transaction for them to select.
//!   Same reason `HCCHAR.ODDFRM` is inert on the host side.
//!
//! # Sources
//!
//! ST's **RM0090** §34.15.3 (the device-mode register map: `DCFG`, `DCTL`,
//! `DSTS`, `DIEPMSK`/`DOEPMSK`, `DAINT`, `DIEPCTLn`/`DOEPCTLn`,
//! `DIEPINTn`/`DOEPINTn`, `DIEPTSIZn`/`DOEPTSIZn`, `DTXFSTSn`, `DIEPTXFn`) and
//! §34.15.2 for `GRXSTSP`'s device-mode `PKTSTS` encoding, together with the
//! **USB 2.0 specification** §8.4 for what a transaction is, §8.5.3 for the
//! three stages of a control transfer and §9.2.7 for the rule that a `SETUP` is
//! never `NAK`ed or `STALL`ed. **No driver and no emulator was consulted**: the
//! Linux dwc2 gadget driver is GPLv2 and ST's device library is
//! vendor-licensed, and neither was opened (`ROADMAP.md` §1).

#[cfg(test)]
mod tests;

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{Completion, DeviceAddress, SetupPacket, Speed, Status, UsbDevice};
use crate::core::error::{Error, Result};
use crate::core::state::{Sink, Source};
use alloc::sync::Weak;

use super::{
    DPID_DATA0, Dwc2, PKTSTS_IN_COMPLETE, PKTSTS_IN_DATA, RXSTS_BCNT_SHIFT, RXSTS_DPID_SHIFT,
    RXSTS_PKTSTS_SHIFT, RxPacket, State, words_of,
};

// ---------------------------------------------------------------------------
// Register offsets (RM0090 §34.15.3)
// ---------------------------------------------------------------------------

/// Device configuration.
pub(super) const DCFG: u64 = 0x800;
/// Device control.
pub(super) const DCTL: u64 = 0x804;
/// Device status. Read-only.
pub(super) const DSTS: u64 = 0x808;
/// `IN` endpoint common interrupt mask.
pub(super) const DIEPMSK: u64 = 0x810;
/// `OUT` endpoint common interrupt mask.
pub(super) const DOEPMSK: u64 = 0x814;
/// All-endpoints interrupt. Read-only.
pub(super) const DAINT: u64 = 0x818;
/// All-endpoints interrupt mask.
pub(super) const DAINTMSK: u64 = 0x81c;
/// V\_BUS discharge time.
pub(super) const DVBUSDIS: u64 = 0x828;
/// V\_BUS pulsing time.
pub(super) const DVBUSPULSE: u64 = 0x82c;
/// Which endpoints' `TXFE` reaches `DAINT`.
pub(super) const DIEPEMPMSK: u64 = 0x834;

/// Where the `IN` endpoint register triples start.
pub(super) const DIEP_BASE: u64 = 0x900;
/// Where the `OUT` endpoint register triples start.
pub(super) const DOEP_BASE: u64 = 0xb00;
/// The stride between one endpoint's registers and the next.
pub(super) const EP_STRIDE: u64 = 0x20;

/// `DIEPTXF1`. Endpoint *n*'s transmit FIFO sizing is at `DIEPTXF_BASE +
/// (n - 1) * 4`; endpoint zero's is `GNPTXFSIZ`, which is why this starts at
/// one rather than zero.
pub(super) const DIEPTXF_BASE: u64 = 0x104;

// -- DCFG -------------------------------------------------------------------

/// Device speed, bits 1:0.
const DCFG_DSPD_MASK: u32 = 0x3;
/// `DSPD`: high speed.
const DSPD_HIGH: u32 = 0;
/// `DSPD`: full speed on a high-speed (ULPI) transceiver.
const DSPD_FULL_HS_PHY: u32 = 1;
/// `DSPD`: low speed.
const DSPD_LOW: u32 = 2;
/// `DSPD`: full speed on the internal full-speed transceiver — what an OTG_FS
/// is, and what its firmware writes.
const DSPD_FULL_FS_PHY: u32 = 3;
/// The device address `SET_ADDRESS` gave us, bits 10:4.
const DCFG_DAD_SHIFT: u32 = 4;
/// The width of that field.
const DCFG_DAD_MASK: u32 = 0x7f;
/// What software may set: the speed, the non-zero-length-status handshake bit,
/// the address and the periodic frame interval.
const DCFG_WRITABLE: u32 =
    DCFG_DSPD_MASK | (1 << 2) | (DCFG_DAD_MASK << DCFG_DAD_SHIFT) | (0x3 << 11);

// -- DCTL -------------------------------------------------------------------

/// Remote wakeup signalling.
const DCTL_RWUSIG: u32 = 1 << 0;
/// **Soft disconnect.** Set out of reset: the core comes up off the bus.
pub(super) const DCTL_SDIS: u32 = 1 << 1;
/// Global `IN` NAK is in effect. Read-only; `SGINAK`/`CGINAK` drive it.
const DCTL_GINSTS: u32 = 1 << 2;
/// Global `OUT` NAK is in effect. Read-only; `SGONAK`/`CGONAK` drive it.
const DCTL_GONSTS: u32 = 1 << 3;
/// Set global `IN` NAK. Self-clearing.
const DCTL_SGINAK: u32 = 1 << 7;
/// Clear global `IN` NAK. Self-clearing.
const DCTL_CGINAK: u32 = 1 << 8;
/// Set global `OUT` NAK. Self-clearing.
const DCTL_SGONAK: u32 = 1 << 9;
/// Clear global `OUT` NAK. Self-clearing.
const DCTL_CGONAK: u32 = 1 << 10;
/// The bits software may set directly: wakeup signalling, soft disconnect, the
/// test control field and "power-on programming done".
const DCTL_WRITABLE: u32 = DCTL_RWUSIG | DCTL_SDIS | (0x7 << 4) | (1 << 11);
/// `DCTL` out of reset (RM0090): soft-disconnected.
const DCTL_RESET_VALUE: u32 = DCTL_SDIS;

// -- DSTS -------------------------------------------------------------------

/// Suspend status. Always clear here — see the module docs.
const DSTS_SUSPSTS: u32 = 1 << 0;
/// Enumerated speed, bits 2:1.
const DSTS_ENUMSPD_SHIFT: u32 = 1;
/// The frame number of the last `SOF`, bits 21:8.
const DSTS_FNSOF_SHIFT: u32 = 8;

// -- DIEPCTLn / DOEPCTLn ----------------------------------------------------

/// Maximum packet size, bits 10:0 — but only two bits on endpoint zero, where
/// it is an *encoding* rather than a number.
const EPCTL_MPSIZ_MASK: u32 = 0x7ff;
/// The endpoint is active in the current configuration.
const EPCTL_USBAEP: u32 = 1 << 15;
/// The endpoint is NAKing. Read-only; `SNAK`/`CNAK` drive it.
const EPCTL_NAKSTS: u32 = 1 << 17;
/// Endpoint type, bits 19:18 — the same encoding an endpoint descriptor carries
/// (USB 2.0 §9.6.6).
const EPCTL_EPTYP_SHIFT: u32 = 18;
/// Snoop mode, `OUT` endpoints only.
const EPCTL_SNPM: u32 = 1 << 20;
/// Stall this endpoint.
const EPCTL_STALL: u32 = 1 << 21;
/// Which transmit FIFO an `IN` endpoint uses, bits 25:22.
const EPCTL_TXFNUM_SHIFT: u32 = 22;
/// Clear the NAK. Self-clearing.
const EPCTL_CNAK: u32 = 1 << 26;
/// Set the NAK. Self-clearing.
const EPCTL_SNAK: u32 = 1 << 27;
/// Set `DATA0`, or select an even frame. Self-clearing.
const EPCTL_SD0PID: u32 = 1 << 28;
/// Select an odd frame. Self-clearing.
const EPCTL_SODDFRM: u32 = 1 << 29;

const _: () = {
    // A compile-time note that the two frame/toggle selectors really are
    // accepted and dropped rather than stored: they are not in the writable
    // set, and a later edit that put them there would be claiming this model
    // carries a data toggle on an endpoint transaction, which it does not.
    assert!(EPCTL_WRITABLE & (EPCTL_SD0PID | EPCTL_SODDFRM) == 0);
};
/// Disable the endpoint.
const EPCTL_EPDIS: u32 = 1 << 30;
/// Arm the endpoint: it will answer the next transaction.
const EPCTL_EPENA: u32 = 1 << 31;
/// The bits that are software's to hold. The self-clearing ones
/// (`CNAK`/`SNAK`/`SD0PID`/`SODDFRM`) are acted on and not stored, and
/// `NAKSTS`, `EPENA` and `EPDIS` are handled by hand.
const EPCTL_WRITABLE: u32 = EPCTL_MPSIZ_MASK
    | EPCTL_USBAEP
    | (0x3 << EPCTL_EPTYP_SHIFT)
    | EPCTL_SNPM
    | EPCTL_STALL
    | (0xf << EPCTL_TXFNUM_SHIFT);

// -- DIEPINTn ---------------------------------------------------------------

/// The transfer finished.
pub(super) const DIEPINT_XFRC: u32 = 1 << 0;
/// The endpoint was disabled by software.
const DIEPINT_EPDISD: u32 = 1 << 1;
/// A timeout on a control `IN`. Never raised: this fabric has no lost packets.
const DIEPINT_TOC: u32 = 1 << 3;
/// An `IN` token arrived when the transmit FIFO was empty.
pub(super) const DIEPINT_ITTXFE: u32 = 1 << 4;
/// The `IN` endpoint NAK became effective.
const DIEPINT_INEPNE: u32 = 1 << 6;
/// The transmit FIFO is empty. **Derived, not latched** — it is a level, and it
/// reaches `DAINT` through `DIEPEMPMSK` rather than `DIEPMSK`.
const DIEPINT_TXFE: u32 = 1 << 7;
/// Every bit `DIEPINTn` defines.
const DIEPINT_MASK: u32 =
    DIEPINT_XFRC | DIEPINT_EPDISD | DIEPINT_TOC | DIEPINT_ITTXFE | DIEPINT_INEPNE | DIEPINT_TXFE;
/// The subset software clears by writing one. `TXFE` is not in it: it is
/// derived from whether the FIFO is empty, and a write cannot make it not be.
const DIEPINT_W1C: u32 = DIEPINT_MASK & !DIEPINT_TXFE;

// -- DOEPINTn ---------------------------------------------------------------

/// The transfer finished.
pub(super) const DOEPINT_XFRC: u32 = 1 << 0;
/// The endpoint was disabled by software.
const DOEPINT_EPDISD: u32 = 1 << 1;
/// A `SETUP` transaction completed. **The interrupt enumeration turns on.**
pub(super) const DOEPINT_STUP: u32 = 1 << 3;
/// An `OUT` token arrived while the endpoint was disabled.
const DOEPINT_OTEPDIS: u32 = 1 << 4;
/// Two `SETUP` packets back to back. Defined and never raised — see the module
/// docs.
const DOEPINT_B2BSTUP: u32 = 1 << 6;
/// Every bit `DOEPINTn` defines, all of them write-1-to-clear.
const DOEPINT_MASK: u32 =
    DOEPINT_XFRC | DOEPINT_EPDISD | DOEPINT_STUP | DOEPINT_OTEPDIS | DOEPINT_B2BSTUP;

// -- DIEPTSIZn / DOEPTSIZn --------------------------------------------------

/// Transfer size, bits 18:0 — bits 6:0 on endpoint zero.
const DTSIZ_XFRSIZ_MASK: u32 = 0x7_ffff;
/// Endpoint zero's narrower transfer size.
const DTSIZ0_XFRSIZ_MASK: u32 = 0x7f;
/// Packet count, bits 28:19 — one bit on endpoint zero.
const DTSIZ_PKTCNT_SHIFT: u32 = 19;
/// The packet-count field's width.
const DTSIZ_PKTCNT_MASK: u32 = 0x3ff;
/// Endpoint zero's one-bit packet count.
const DTSIZ0_PKTCNT_MASK: u32 = 0x1;
/// `DOEPTSIZ0.STUPCNT`, bits 30:29: how many back-to-back setup packets the
/// endpoint will take.
const DTSIZ0_STUPCNT_SHIFT: u32 = 29;

// -- GINTSTS, the device-mode half ------------------------------------------

/// The global `IN` NAK became effective. Read-only.
const GINT_GINAKEFF: u32 = 1 << 6;
/// The global `OUT` NAK became effective. Read-only.
const GINT_GONAKEFF: u32 = 1 << 7;
/// A bus reset was detected.
pub(super) const GINT_USBRST: u32 = 1 << 12;
/// Speed enumeration finished, so `DSTS.ENUMSPD` is valid.
pub(super) const GINT_ENUMDNE: u32 = 1 << 13;
/// Some `IN` endpoint has an unmasked interrupt. Read-only: clear `DIEPINTn`.
pub(super) const GINT_IEPINT: u32 = 1 << 18;
/// Some `OUT` endpoint has an unmasked interrupt. Read-only: clear `DOEPINTn`.
pub(super) const GINT_OEPINT: u32 = 1 << 19;
/// The non-periodic transmit FIFO is empty. Read-only.
const GINT_NPTXFE: u32 = 1 << 5;

// -- GRXSTSP, the device-mode `PKTSTS` encoding (RM0090 §34.15.2) -----------

/// An `OUT` data packet is in the FIFO behind this word. Numerically the same
/// code an `IN` data packet has in host mode, which is not a coincidence: it is
/// one FIFO and one encoding, read from whichever side is receiving.
const PKTSTS_OUT_DATA: u32 = PKTSTS_IN_DATA;
/// The `OUT` transfer on this endpoint completed. No bytes follow.
const PKTSTS_OUT_COMPLETE: u32 = PKTSTS_IN_COMPLETE;
/// A `SETUP` transaction completed. No bytes follow.
const PKTSTS_SETUP_COMPLETE: u32 = 0b0100;
/// The eight bytes of a `SETUP` packet are in the FIFO behind this word.
const PKTSTS_SETUP_DATA: u32 = 0b0110;
/// The frame number this packet arrived in, bits 24:21.
const RXSTS_FRMNUM_SHIFT: u32 = 21;

/// How many endpoints the register map has room for: `DAINT` is sixteen bits
/// each way.
pub const MAX_ENDPOINTS: usize = 16;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// One `IN` endpoint: three registers, and the bytes the guest has staged for
/// it by writing its FIFO window.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct InEndpoint {
    ctl: u32,
    int: u32,
    tsiz: u32,
    /// Staged, not yet transmitted. A queue for the same reason a host
    /// channel's is: the guest fills it a word at a time and the bus drains it
    /// a packet at a time.
    pub(super) tx: VecDeque<u8>,
}

/// One `OUT` endpoint. No staging of its own: everything received goes into the
/// single shared receive FIFO, which is what makes `GRXSTSP` the only way to
/// find out which endpoint a packet was for.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct OutEndpoint {
    ctl: u32,
    int: u32,
    tsiz: u32,
}

/// Everything the device half of the core holds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct DeviceState {
    dcfg: u32,
    dctl: u32,
    diepmsk: u32,
    doepmsk: u32,
    daintmsk: u32,
    dvbusdis: u32,
    dvbuspulse: u32,
    diepempmsk: u32,
    /// `DIEPTXFn` for *n* ≥ 1. Index zero is unused and always reads zero;
    /// endpoint zero's transmit FIFO is sized by `GNPTXFSIZ`.
    dieptxf: [u32; MAX_ENDPOINTS],
    /// `DSTS.FNSOF`: the frame number of the last start-of-frame seen.
    fnsof: u32,
    /// Sixteen always, whatever the configured endpoint count is — the same
    /// argument the host channels make: a snapshot's shape must not depend on a
    /// construction property.
    pub(super) din: [InEndpoint; MAX_ENDPOINTS],
    dout: [OutEndpoint; MAX_ENDPOINTS],
}

impl DeviceState {
    /// The documented reset state.
    pub(super) fn reset() -> DeviceState {
        DeviceState {
            dcfg: 0,
            dctl: DCTL_RESET_VALUE,
            diepmsk: 0,
            doepmsk: 0,
            daintmsk: 0,
            dvbusdis: 0x0000_17d7,
            dvbuspulse: 0x0000_05b8,
            diepempmsk: 0,
            dieptxf: [0x0200_0400; MAX_ENDPOINTS],
            fnsof: 0,
            din: core::array::from_fn(|_| InEndpoint::default()),
            dout: core::array::from_fn(|_| OutEndpoint::default()),
        }
    }

    /// What a **bus** reset does, as opposed to a core reset (USB 2.0 §9.1.1.3).
    ///
    /// The address goes, every endpoint is disarmed and unstalled, and the
    /// interrupt latches are cleared — but `DCTL` does not, because soft
    /// connect is the *application's* decision and a host resetting the port is
    /// not a reason to fall off the bus.
    fn bus_reset(&mut self) {
        self.dcfg &= !(DCFG_DAD_MASK << DCFG_DAD_SHIFT);
        self.dctl &= !(DCTL_GINSTS | DCTL_GONSTS);
        for ep in &mut self.din {
            ep.ctl &= !(EPCTL_EPENA | EPCTL_STALL | EPCTL_NAKSTS);
            ep.int = 0;
            ep.tsiz = 0;
            ep.tx.clear();
        }
        for ep in &mut self.dout {
            ep.ctl &= !(EPCTL_EPENA | EPCTL_STALL | EPCTL_NAKSTS);
            ep.int = 0;
            ep.tsiz = 0;
        }
        self.fnsof = 0;
    }

    /// The address the guest's firmware has told the core to answer to.
    fn address(&self) -> DeviceAddress {
        DeviceAddress(((self.dcfg >> DCFG_DAD_SHIFT) & DCFG_DAD_MASK) as u8)
    }
}

/// `DIEPCTL0.MPSIZ` is an *encoding*, not a number (RM0090): `00b` is 64 bytes,
/// and each step down halves it.
fn ep0_packet_size(ctl: u32) -> u32 {
    match ctl & 0x3 {
        0 => 64,
        1 => 32,
        2 => 16,
        _ => 8,
    }
}

/// What one endpoint's `wMaxPacketSize` is, in bytes.
fn packet_size(endpoint: usize, ctl: u32) -> u32 {
    if endpoint == 0 {
        ep0_packet_size(ctl)
    } else {
        (ctl & EPCTL_MPSIZ_MASK).max(1)
    }
}

/// `XFRSIZ`, whose field is narrower on endpoint zero.
fn xfrsiz(endpoint: usize, tsiz: u32) -> u32 {
    if endpoint == 0 {
        tsiz & DTSIZ0_XFRSIZ_MASK
    } else {
        tsiz & DTSIZ_XFRSIZ_MASK
    }
}

/// `PKTCNT`, whose field is one bit on endpoint zero.
fn pktcnt(endpoint: usize, tsiz: u32) -> u32 {
    let mask = if endpoint == 0 {
        DTSIZ0_PKTCNT_MASK
    } else {
        DTSIZ_PKTCNT_MASK
    };
    (tsiz >> DTSIZ_PKTCNT_SHIFT) & mask
}

/// Rewrite `XFRSIZ` and `PKTCNT` in one go, honouring the narrower fields of
/// endpoint zero.
fn set_counts(endpoint: usize, tsiz: u32, bytes: u32, packets: u32) -> u32 {
    let (xmask, pmask) = if endpoint == 0 {
        (DTSIZ0_XFRSIZ_MASK, DTSIZ0_PKTCNT_MASK)
    } else {
        (DTSIZ_XFRSIZ_MASK, DTSIZ_PKTCNT_MASK)
    };
    let cleared = tsiz & !(xmask | (pmask << DTSIZ_PKTCNT_SHIFT));
    cleared | (bytes & xmask) | ((packets & pmask) << DTSIZ_PKTCNT_SHIFT)
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// Read one device-mode register.
pub(super) fn read(core: &Dwc2, state: &State, offset: u64) -> u32 {
    let endpoints = usize::from(core.params().endpoints);
    let dev = &state.dev;

    if (DIEP_BASE..DOEP_BASE).contains(&offset) {
        let index = ((offset - DIEP_BASE) / EP_STRIDE) as usize;
        if index >= endpoints {
            return 0;
        }
        let ep = &dev.din[index];
        return match (offset - DIEP_BASE) % EP_STRIDE {
            0x00 => in_ctl(index, ep.ctl),
            0x08 => in_int(ep),
            0x10 => ep.tsiz,
            // `DTXFSTSn`: how many words of this endpoint's transmit FIFO are
            // free. The guest polls it before pushing a packet.
            0x18 => tx_free(core, state, index),
            _ => 0,
        };
    }

    if (DOEP_BASE..super::DEVICE_END).contains(&offset) {
        let index = ((offset - DOEP_BASE) / EP_STRIDE) as usize;
        if index >= endpoints {
            return 0;
        }
        let ep = &dev.dout[index];
        return match (offset - DOEP_BASE) % EP_STRIDE {
            0x00 => out_ctl(index, ep.ctl, dev.din[0].ctl),
            0x08 => ep.int & DOEPINT_MASK,
            0x10 => ep.tsiz,
            _ => 0,
        };
    }

    match offset {
        DCFG => dev.dcfg & DCFG_WRITABLE,
        DCTL => dev.dctl & (DCTL_WRITABLE | DCTL_GINSTS | DCTL_GONSTS),
        DSTS => {
            let mut value = enumerated_speed(core, dev) << DSTS_ENUMSPD_SHIFT;
            value |= (dev.fnsof & 0x3fff) << DSTS_FNSOF_SHIFT;
            // `SUSPSTS` is never set: see the module docs.
            value & !DSTS_SUSPSTS
        }
        DIEPMSK => dev.diepmsk & DIEPINT_MASK,
        DOEPMSK => dev.doepmsk & DOEPINT_MASK,
        DAINT => daint(core, state),
        DAINTMSK => dev.daintmsk,
        DVBUSDIS => dev.dvbusdis & 0xffff,
        DVBUSPULSE => dev.dvbuspulse & 0xfff,
        DIEPEMPMSK => dev.diepempmsk & 0xffff,
        _ => 0,
    }
}

/// Write one device-mode register.
///
/// Returns whether the write changed something the fabric has to be told about
/// — which is only ever soft connect.
pub(super) fn write(core: &Dwc2, state: &mut State, offset: u64, value: u32) -> bool {
    let endpoints = usize::from(core.params().endpoints);

    if (DIEP_BASE..DOEP_BASE).contains(&offset) {
        let index = ((offset - DIEP_BASE) / EP_STRIDE) as usize;
        if index < endpoints {
            let register = (offset - DIEP_BASE) % EP_STRIDE;
            write_in_endpoint(state, index, register, value);
        }
        return false;
    }

    if (DOEP_BASE..super::DEVICE_END).contains(&offset) {
        let index = ((offset - DOEP_BASE) / EP_STRIDE) as usize;
        if index < endpoints {
            let register = (offset - DOEP_BASE) % EP_STRIDE;
            write_out_endpoint(state, index, register, value);
        }
        return false;
    }

    let dev = &mut state.dev;
    match offset {
        DCFG => dev.dcfg = value & DCFG_WRITABLE,
        DCTL => {
            let was = dev.dctl;
            dev.dctl = (dev.dctl & (DCTL_GINSTS | DCTL_GONSTS)) | (value & DCTL_WRITABLE);
            // The four self-clearing NAK controls, acted on and not stored.
            if value & DCTL_SGINAK != 0 {
                dev.dctl |= DCTL_GINSTS;
            }
            if value & DCTL_CGINAK != 0 {
                dev.dctl &= !DCTL_GINSTS;
            }
            if value & DCTL_SGONAK != 0 {
                dev.dctl |= DCTL_GONSTS;
            }
            if value & DCTL_CGONAK != 0 {
                dev.dctl &= !DCTL_GONSTS;
            }
            return (was ^ dev.dctl) & DCTL_SDIS != 0;
        }
        DIEPMSK => dev.diepmsk = value & DIEPINT_MASK,
        DOEPMSK => dev.doepmsk = value & DOEPINT_MASK,
        DAINTMSK => dev.daintmsk = value,
        DVBUSDIS => dev.dvbusdis = value & 0xffff,
        DVBUSPULSE => dev.dvbuspulse = value & 0xfff,
        DIEPEMPMSK => dev.diepempmsk = value & 0xffff,
        // `DAINT` and `DSTS` are read-only, and so is anything reserved.
        _ => {}
    }
    false
}

/// `DIEPTXFn`, which lives in the *global* register block rather than either
/// role's — RM0090 puts it at `+0x104`, immediately after the host's
/// `HPTXFSIZ`. Returns `None` for an offset that is not one of them.
pub(super) fn tx_fifo_register(core: &Dwc2, offset: u64) -> Option<usize> {
    let endpoints = u64::from(core.params().endpoints);
    if endpoints < 2 || !(DIEPTXF_BASE..DIEPTXF_BASE + (endpoints - 1) * 4).contains(&offset) {
        return None;
    }
    Some(((offset - DIEPTXF_BASE) / 4 + 1) as usize)
}

/// Read `DIEPTXFn`.
pub(super) fn read_tx_fifo(state: &State, endpoint: usize) -> u32 {
    state.dev.dieptxf.get(endpoint).copied().unwrap_or(0)
}

/// Write `DIEPTXFn`.
pub(super) fn write_tx_fifo(state: &mut State, endpoint: usize, value: u32) {
    if let Some(slot) = state.dev.dieptxf.get_mut(endpoint) {
        *slot = value;
    }
}

/// `DIEPCTLn` as the guest reads it. `USBAEP` is read-only and set on endpoint
/// zero, which is always active; `EPTYP` is read-only there too, and is
/// `00b` — control, the only thing endpoint zero may be.
fn in_ctl(endpoint: usize, ctl: u32) -> u32 {
    let mut value = ctl;
    if endpoint == 0 {
        value |= EPCTL_USBAEP;
        value &= !(0x3 << EPCTL_EPTYP_SHIFT);
        value = (value & !EPCTL_MPSIZ_MASK) | (ctl & 0x3);
    }
    value
}

/// `DOEPCTLn` as the guest reads it. On endpoint zero `MPSIZ` is read-only and
/// *mirrors `DIEPCTL0`'s* — one control endpoint, one packet size.
fn out_ctl(endpoint: usize, ctl: u32, diepctl0: u32) -> u32 {
    let mut value = ctl;
    if endpoint == 0 {
        value |= EPCTL_USBAEP;
        value &= !(0x3 << EPCTL_EPTYP_SHIFT);
        value = (value & !EPCTL_MPSIZ_MASK) | (diepctl0 & 0x3);
        // `EPDIS` is read-only on `DOEPCTL0`: the default pipe cannot be
        // disabled, only stalled.
        value &= !EPCTL_EPDIS;
    }
    value
}

/// `DIEPINTn` as the guest reads it: the latched bits, plus the derived
/// `TXFE`.
fn in_int(ep: &InEndpoint) -> u32 {
    let mut value = ep.int & DIEPINT_W1C;
    if ep.tx.is_empty() {
        value |= DIEPINT_TXFE;
    }
    value
}

fn write_in_endpoint(state: &mut State, index: usize, register: u64, value: u32) {
    let ep = &mut state.dev.din[index];
    match register {
        0x00 => {
            let stored = ep.ctl & (EPCTL_EPENA | EPCTL_NAKSTS);
            ep.ctl = stored | (value & EPCTL_WRITABLE);
            if value & EPCTL_CNAK != 0 {
                ep.ctl &= !EPCTL_NAKSTS;
            }
            if value & EPCTL_SNAK != 0 {
                ep.ctl |= EPCTL_NAKSTS;
            }
            // `SD0PID` and `SODDFRM` are accepted and not stored: they select
            // a data toggle and a frame parity, and this fabric carries neither
            // on an endpoint transaction.
            if value & EPCTL_EPENA != 0 {
                ep.ctl |= EPCTL_EPENA;
            }
            if value & EPCTL_EPDIS != 0 {
                // Answered immediately: there is no transaction in flight to
                // wait for, because a transaction here is executed to
                // completion inside the call that delivers it.
                ep.ctl &= !EPCTL_EPENA;
                ep.int |= DIEPINT_EPDISD;
            }
        }
        0x08 => ep.int &= !(value & DIEPINT_W1C),
        0x10 => ep.tsiz = value,
        _ => {}
    }
}

fn write_out_endpoint(state: &mut State, index: usize, register: u64, value: u32) {
    let ep = &mut state.dev.dout[index];
    match register {
        0x00 => {
            let stored = ep.ctl & (EPCTL_EPENA | EPCTL_NAKSTS);
            ep.ctl = stored | (value & EPCTL_WRITABLE);
            if value & EPCTL_CNAK != 0 {
                ep.ctl &= !EPCTL_NAKSTS;
            }
            if value & EPCTL_SNAK != 0 {
                ep.ctl |= EPCTL_NAKSTS;
            }
            if value & EPCTL_EPENA != 0 {
                ep.ctl |= EPCTL_EPENA;
            }
            if value & EPCTL_EPDIS != 0 && index != 0 {
                ep.ctl &= !EPCTL_EPENA;
                ep.int |= DOEPINT_EPDISD;
            }
        }
        0x08 => ep.int &= !(value & DOEPINT_MASK),
        0x10 => ep.tsiz = value,
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// FIFO sizing
// ---------------------------------------------------------------------------

/// How deep endpoint `endpoint`'s transmit FIFO is, in words, clamped to the
/// dedicated RAM the part actually has.
pub(super) fn tx_depth(core: &Dwc2, state: &State, endpoint: usize) -> u32 {
    let raw = if endpoint == 0 {
        state.gnptxfsiz >> super::FIFO_DEPTH_SHIFT
    } else {
        state.dev.dieptxf.get(endpoint).copied().unwrap_or(0) >> super::FIFO_DEPTH_SHIFT
    };
    (raw & super::FIFO_DEPTH_MASK).min(core.params().fifo_words)
}

/// How many words of that FIFO are free — which is `DTXFSTSn`.
fn tx_free(core: &Dwc2, state: &State, endpoint: usize) -> u32 {
    let used = words_of(state.dev.din[endpoint].tx.len());
    tx_depth(core, state, endpoint).saturating_sub(used)
}

/// Push one word into endpoint `endpoint`'s transmit FIFO. A FIFO that is full
/// drops what does not fit — the only thing a fixed amount of RAM can do, and
/// what bounds this against a guest that writes for ever.
pub(super) fn push_word(core: &Dwc2, state: &mut State, endpoint: usize, value: u32) {
    if endpoint >= usize::from(core.params().endpoints) {
        return;
    }
    if tx_free(core, state, endpoint) == 0 {
        return;
    }
    state.dev.din[endpoint].tx.extend(value.to_le_bytes());
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

/// One `IN` endpoint's interrupt bits, after its masks.
///
/// `TXFE` is masked by `DIEPEMPMSK` rather than by `DIEPMSK`, which is the one
/// thing about this tree that is not obvious from the register names.
fn in_pending(state: &State, index: usize) -> u32 {
    let dev = &state.dev;
    let mut mask = dev.diepmsk & !DIEPINT_TXFE;
    if dev.diepempmsk & (1u32 << index) != 0 {
        mask |= DIEPINT_TXFE;
    }
    in_int(&dev.din[index]) & mask
}

/// One `OUT` endpoint's interrupt bits, after `DOEPMSK`.
fn out_pending(state: &State, index: usize) -> u32 {
    state.dev.dout[index].int & state.dev.doepmsk & DOEPINT_MASK
}

/// `DAINT`: which endpoints have an unmasked interrupt. `IN` in the low half,
/// `OUT` in the high one.
pub(super) fn daint(core: &Dwc2, state: &State) -> u32 {
    let mut bits = 0;
    for index in 0..usize::from(core.params().endpoints) {
        if in_pending(state, index) != 0 {
            bits |= 1u32 << index;
        }
        if out_pending(state, index) != 0 {
            bits |= 1u32 << (index + 16);
        }
    }
    bits
}

/// The bits device mode contributes to `GINTSTS`.
pub(super) fn gintsts(core: &Dwc2, state: &State) -> u32 {
    let mut value = 0;
    let pending = daint(core, state) & state.dev.daintmsk;
    if pending & 0x0000_ffff != 0 {
        value |= GINT_IEPINT;
    }
    if pending & 0xffff_0000 != 0 {
        value |= GINT_OEPINT;
    }
    if state.dev.dctl & DCTL_GINSTS != 0 {
        value |= GINT_GINAKEFF;
    }
    if state.dev.dctl & DCTL_GONSTS != 0 {
        value |= GINT_GONAKEFF;
    }
    if state.dev.din[0].tx.is_empty() {
        value |= GINT_NPTXFE;
    }
    value
}

/// `DSTS.ENUMSPD` — what the core says it enumerated at.
fn enumerated_speed(core: &Dwc2, dev: &DeviceState) -> u32 {
    match speed_of(core, dev) {
        Speed::High => DSPD_HIGH,
        Speed::Low => DSPD_LOW,
        Speed::Full if core.params().max_speed == Speed::High => DSPD_FULL_HS_PHY,
        Speed::Full => DSPD_FULL_FS_PHY,
    }
}

/// How fast this core signals: what its firmware asked for in `DCFG.DSPD`, but
/// never faster than the transceiver the board gave it.
fn speed_of(core: &Dwc2, dev: &DeviceState) -> Speed {
    let asked = match dev.dcfg & DCFG_DSPD_MASK {
        DSPD_HIGH => Speed::High,
        DSPD_LOW => Speed::Low,
        DSPD_FULL_HS_PHY | DSPD_FULL_FS_PHY => Speed::Full,
        _ => Speed::Full,
    };
    asked.min(core.params().max_speed)
}

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

/// Serialize the device half.
///
/// # Errors
///
/// Whatever the sink refuses.
pub(super) fn save<S: Sink + ?Sized>(dev: &DeviceState, w: &mut S) -> Result<()> {
    w.write_u32(dev.dcfg)?;
    w.write_u32(dev.dctl)?;
    w.write_u32(dev.diepmsk)?;
    w.write_u32(dev.doepmsk)?;
    w.write_u32(dev.daintmsk)?;
    w.write_u32(dev.dvbusdis)?;
    w.write_u32(dev.dvbuspulse)?;
    w.write_u32(dev.diepempmsk)?;
    w.write_u32(dev.fnsof)?;
    w.write_seq_len(MAX_ENDPOINTS as u64)?;
    for value in &dev.dieptxf {
        w.write_u32(*value)?;
    }
    w.write_seq_len(MAX_ENDPOINTS as u64)?;
    for ep in &dev.din {
        w.write_u32(ep.ctl)?;
        w.write_u32(ep.int)?;
        w.write_u32(ep.tsiz)?;
        let staged: Vec<u8> = ep.tx.iter().copied().collect();
        w.write_bytes(&staged)?;
    }
    w.write_seq_len(MAX_ENDPOINTS as u64)?;
    for ep in &dev.dout {
        w.write_u32(ep.ctl)?;
        w.write_u32(ep.int)?;
        w.write_u32(ep.tsiz)?;
    }
    Ok(())
}

/// Restore what [`save`] wrote.
///
/// # Errors
///
/// [`Error::State`] for a truncated or malformed chunk.
pub(super) fn load<'a, S: Source<'a> + ?Sized>(r: &mut S) -> Result<DeviceState> {
    let mut dev = DeviceState {
        dcfg: r.read_u32()?,
        dctl: r.read_u32()?,
        diepmsk: r.read_u32()?,
        doepmsk: r.read_u32()?,
        daintmsk: r.read_u32()?,
        dvbusdis: r.read_u32()?,
        dvbuspulse: r.read_u32()?,
        diepempmsk: r.read_u32()?,
        fnsof: r.read_u32()?,
        dieptxf: [0; MAX_ENDPOINTS],
        din: core::array::from_fn(|_| InEndpoint::default()),
        dout: core::array::from_fn(|_| OutEndpoint::default()),
    };

    let count = r.read_seq_len(4)?;
    if count != MAX_ENDPOINTS as u64 {
        return Err(Error::State(alloc::format!(
            "usb.dwc2: a snapshot with {count} transmit FIFOs, not {MAX_ENDPOINTS}"
        )));
    }
    for slot in &mut dev.dieptxf {
        *slot = r.read_u32()?;
    }

    let count = r.read_seq_len(16)?;
    if count != MAX_ENDPOINTS as u64 {
        return Err(Error::State(alloc::format!(
            "usb.dwc2: a snapshot with {count} IN endpoints, not {MAX_ENDPOINTS}"
        )));
    }
    for ep in &mut dev.din {
        ep.ctl = r.read_u32()?;
        ep.int = r.read_u32()?;
        ep.tsiz = r.read_u32()?;
        ep.tx = r.read_bytes()?.iter().copied().collect();
    }

    let count = r.read_seq_len(12)?;
    if count != MAX_ENDPOINTS as u64 {
        return Err(Error::State(alloc::format!(
            "usb.dwc2: a snapshot with {count} OUT endpoints, not {MAX_ENDPOINTS}"
        )));
    }
    for ep in &mut dev.dout {
        ep.ctl = r.read_u32()?;
        ep.int = r.read_u32()?;
        ep.tsiz = r.read_u32()?;
    }
    Ok(dev)
}

// ---------------------------------------------------------------------------
// The gadget: this core, as something on somebody else's bus
// ---------------------------------------------------------------------------

/// The core in device mode, seen from the bus.
///
/// A separate object rather than an `impl` on [`Dwc2`] itself so that plugging
/// it into a [`UsbBus`](crate::bus::usb::UsbBus) does not make a reference
/// cycle: the bus holds this, this holds a [`Weak`] back to the core, and the
/// core holds the bus. Nothing keeps anything else alive after the machine is
/// dropped.
pub struct Dwc2Gadget {
    core: Weak<Dwc2>,
}

impl fmt::Debug for Dwc2Gadget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Dwc2Gadget")
            .field("live", &(self.core.strong_count() > 0))
            .finish()
    }
}

impl Dwc2Gadget {
    /// The gadget half of `core`.
    #[must_use]
    pub(super) fn new(core: Weak<Dwc2>) -> Dwc2Gadget {
        Dwc2Gadget { core }
    }
}

impl UsbDevice for Dwc2Gadget {
    fn speed(&self) -> Speed {
        match self.core.upgrade() {
            Some(core) => {
                let state = core.state.lock();
                speed_of(&core, &state.dev)
            }
            None => Speed::Full,
        }
    }

    fn address(&self) -> DeviceAddress {
        match self.core.upgrade() {
            Some(core) => {
                let state = core.state.lock();
                state.dev.address()
            }
            None => DeviceAddress::DEFAULT,
        }
    }

    /// The host reset the port. `GINTSTS.USBRST` and then `ENUMDNE`, which is
    /// the pair a gadget driver's interrupt handler is written around.
    ///
    /// Both at once, because this model has no reset *duration*: on silicon
    /// they are microseconds apart, with the speed negotiation in between, and
    /// there is nothing in this fabric for that interval to contain. A driver
    /// that handles them in one pass of its handler and a driver that takes two
    /// interrupts both work.
    fn bus_reset(&self) {
        let Some(core) = self.core.upgrade() else {
            return;
        };
        {
            let mut state = core.state.lock();
            state.dev.bus_reset();
            state.rx.clear();
            state.gintsts |= GINT_USBRST | GINT_ENUMDNE;
        }
        core.refresh_irq();
    }

    fn start_of_frame(&self, frame: u16) {
        let Some(core) = self.core.upgrade() else {
            return;
        };
        {
            let mut state = core.state.lock();
            state.dev.fnsof = u32::from(frame) & 0x3fff;
            state.gintsts |= super::GINT_SOF;
        }
        core.refresh_irq();
    }

    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status {
        let Some(core) = self.core.upgrade() else {
            return Status::NoDevice;
        };
        let status = {
            let mut state = core.state.lock();
            accept_setup(&core, &mut state, endpoint, packet)
        };
        core.refresh_irq();
        status
    }

    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let completion = {
            let mut state = core.state.lock();
            answer_in(&core, &mut state, endpoint, dst, true)
        };
        core.refresh_irq();
        completion
    }

    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let completion = {
            let mut state = core.state.lock();
            accept_out(&core, &mut state, endpoint, src)
        };
        core.refresh_irq();
        completion
    }

    /// The debug path: what the next `IN` would produce, with nothing drained
    /// and no interrupt raised.
    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let mut state = core.state.lock();
        answer_in(&core, &mut state, endpoint, dst, false)
    }
}

/// Whether the device half is on the bus at all: device mode selected, and soft
/// connect asserted.
pub(super) fn soft_connected(state: &State) -> bool {
    !Dwc2::host_mode(state) && state.dev.dctl & DCTL_SDIS == 0
}

/// A `SETUP` transaction arrived.
///
/// §9.2.7: a setup packet is never `NAK`ed and never `STALL`ed, so the only
/// refusal here is a receive FIFO with no room for it — which on silicon is a
/// packet the core simply cannot take, and is reported the only way this fabric
/// can report it.
fn accept_setup(core: &Dwc2, state: &mut State, endpoint: u8, packet: SetupPacket) -> Status {
    if !soft_connected(state) {
        return Status::NoDevice;
    }
    let index = usize::from(endpoint & 0x0f);
    if index >= usize::from(core.params().endpoints) {
        return Status::Stall;
    }

    let depth = core.rx_depth(state);
    // The eight bytes plus their status word, and then the completion word.
    let need = 1 + words_of(SetupPacket::SIZE as usize) + 1;
    if state.rx.words().saturating_add(need) > depth {
        return Status::Nak;
    }

    let frame = state.dev.fnsof & 0xf;
    let bytes = packet.encode();
    state.rx.queue.push_back(RxPacket {
        status: (index as u32)
            | ((SetupPacket::SIZE as u32) << RXSTS_BCNT_SHIFT)
            | (DPID_DATA0 << RXSTS_DPID_SHIFT)
            | (PKTSTS_SETUP_DATA << RXSTS_PKTSTS_SHIFT)
            | (frame << RXSTS_FRMNUM_SHIFT),
        data: bytes.to_vec(),
    });
    state.rx.queue.push_back(RxPacket {
        status: (index as u32)
            | (DPID_DATA0 << RXSTS_DPID_SHIFT)
            | (PKTSTS_SETUP_COMPLETE << RXSTS_PKTSTS_SHIFT)
            | (frame << RXSTS_FRMNUM_SHIFT),
        data: Vec::new(),
    });

    let ep = &mut state.dev.dout[index];
    ep.int |= DOEPINT_STUP;
    // `STUPCNT` counts down as setup packets land, and stops at zero.
    let stupcnt = (ep.tsiz >> DTSIZ0_STUPCNT_SHIFT) & 0x3;
    ep.tsiz = (ep.tsiz & !(0x3 << DTSIZ0_STUPCNT_SHIFT))
        | (stupcnt.saturating_sub(1) << DTSIZ0_STUPCNT_SHIFT);
    // A setup packet ends whatever this control pipe was doing: a stall armed
    // for the previous request does not survive into the next one (§9.2.7 — the
    // condition on a control endpoint is cleared by the next `SETUP`, which is
    // why a host does not have to send `CLEAR_FEATURE` after a request the
    // device refused).
    state.dev.din[index].ctl &= !EPCTL_STALL;
    state.dev.dout[index].ctl &= !EPCTL_STALL;
    Status::Ack
}

/// An `IN` transaction arrived: the host wants bytes.
///
/// `commit` is false for the debug path, which must answer identically and
/// change nothing.
fn answer_in(
    core: &Dwc2,
    state: &mut State,
    endpoint: u8,
    dst: &mut [u8],
    commit: bool,
) -> Completion {
    if !soft_connected(state) {
        return Completion::absent();
    }
    let index = usize::from(endpoint & 0x0f);
    if index >= usize::from(core.params().endpoints) {
        return Completion::stall();
    }
    let global_nak = state.dev.dctl & DCTL_GINSTS != 0;
    let (ctl, tsiz, staged) = {
        let ep = &state.dev.din[index];
        (ep.ctl, ep.tsiz, ep.tx.len())
    };
    if ctl & EPCTL_STALL != 0 {
        return Completion::stall();
    }
    if global_nak || ctl & EPCTL_NAKSTS != 0 || ctl & EPCTL_EPENA == 0 {
        return Completion::nak();
    }
    let packets = pktcnt(index, tsiz);
    if packets == 0 {
        return Completion::nak();
    }

    let mps = packet_size(index, ctl);
    let remaining = xfrsiz(index, tsiz);
    let want = mps.min(remaining) as usize;
    if staged < want {
        // The guest has not finished pushing this packet. RM0090 has a name for
        // a token that arrives in that state, and this is it.
        if commit {
            state.dev.din[index].int |= DIEPINT_ITTXFE;
        }
        return Completion::nak();
    }
    if want > dst.len() {
        // The host reserved less than this endpoint wants to send. On the wire
        // that is a device sending more than the host will take (USB 2.0
        // §8.7.4), and the fabric has the handshake for it.
        return Completion {
            status: Status::Babble,
            len: 0,
        };
    }

    let data: Vec<u8> = state.dev.din[index].tx.iter().copied().take(want).collect();
    dst[..want].copy_from_slice(&data);
    if !commit {
        return Completion::ack(want as u64);
    }

    let ep = &mut state.dev.din[index];
    for _ in 0..want {
        ep.tx.pop_front();
    }
    let left = remaining.saturating_sub(want as u32);
    let packets = packets.saturating_sub(1);
    ep.tsiz = set_counts(index, ep.tsiz, left, packets);
    if packets == 0 {
        ep.ctl &= !EPCTL_EPENA;
        ep.int |= DIEPINT_XFRC;
    }
    Completion::ack(want as u64)
}

/// An `OUT` transaction arrived: the host is handing over bytes.
fn accept_out(core: &Dwc2, state: &mut State, endpoint: u8, src: &[u8]) -> Completion {
    if !soft_connected(state) {
        return Completion::absent();
    }
    let index = usize::from(endpoint & 0x0f);
    if index >= usize::from(core.params().endpoints) {
        return Completion::stall();
    }
    let global_nak = state.dev.dctl & DCTL_GONSTS != 0;
    let (ctl, tsiz) = {
        let ep = &state.dev.dout[index];
        (ep.ctl, ep.tsiz)
    };
    if ctl & EPCTL_STALL != 0 {
        return Completion::stall();
    }
    if ctl & EPCTL_EPENA == 0 {
        if !src.is_empty() {
            state.dev.dout[index].int |= DOEPINT_OTEPDIS;
        }
        return Completion::nak();
    }
    if global_nak || ctl & EPCTL_NAKSTS != 0 {
        return Completion::nak();
    }
    let packets = pktcnt(index, tsiz);
    if packets == 0 {
        return Completion::nak();
    }

    let depth = core.rx_depth(state);
    let need = 1 + words_of(src.len()) + 1;
    if state.rx.words().saturating_add(need) > depth {
        // No room. A device whose receive FIFO is full NAKs, which is the whole
        // reason `NAK` exists (USB 2.0 §8.4.5).
        return Completion::nak();
    }

    let mps = packet_size(index, ctl);
    let remaining = xfrsiz(index, tsiz);
    let frame = state.dev.fnsof & 0xf;
    let taken = src.len().min(remaining as usize);
    if taken > 0 {
        state.rx.queue.push_back(RxPacket {
            status: (index as u32)
                | ((taken as u32) << RXSTS_BCNT_SHIFT)
                | (DPID_DATA0 << RXSTS_DPID_SHIFT)
                | (PKTSTS_OUT_DATA << RXSTS_PKTSTS_SHIFT)
                | (frame << RXSTS_FRMNUM_SHIFT),
            data: src[..taken].to_vec(),
        });
    }

    let short = (taken as u32) < mps;
    let ep = &mut state.dev.dout[index];
    let left = remaining.saturating_sub(taken as u32);
    let packets = packets.saturating_sub(1);
    ep.tsiz = set_counts(index, ep.tsiz, left, packets);
    // §5.8.3: a short packet ends the transfer whether or not the count ran
    // out, which is how a host tells a device it has finished.
    if packets == 0 || short {
        ep.ctl &= !EPCTL_EPENA;
        ep.int |= DOEPINT_XFRC;
        state.rx.queue.push_back(RxPacket {
            status: (index as u32)
                | (DPID_DATA0 << RXSTS_DPID_SHIFT)
                | (PKTSTS_OUT_COMPLETE << RXSTS_PKTSTS_SHIFT)
                | (frame << RXSTS_FRMNUM_SHIFT),
            data: Vec::new(),
        });
    }
    Completion::ack(taken as u64)
}
