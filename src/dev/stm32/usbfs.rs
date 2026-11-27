//! The STM32 **USB full-speed device** peripheral — the `USB` block of the
//! STM32F0x2 and the STM32WB, not the dwc2 OTG core.
//!
//! One class, `st.usbfs`. This is ST's device-only controller: eight endpoint
//! registers, a 1 KiB packet-buffer memory the endpoints draw their buffers
//! from, and a buffer-descriptor table in that memory that says where each
//! buffer is. It is a **peripheral, never a host** — there is no `HPRT`, no
//! channel list, nothing that drives a bus — so it plugs into [`UsbBus`] as a
//! [`UsbDevice`] and a modelled host enumerates it, exactly the way the
//! [`dwc2`](super::super::usb::dwc2) gadget does. That is the whole reason it
//! belongs here rather than being faked: the firmware under test is a USB
//! device stack, and the thing it talks to is the register file below.
//!
//! # One class, two families
//!
//! RM0091 §30 (STM32F04x/F072/F078) and RM0434 §40 (STM32WB) describe the same
//! block in the same words — the register descriptions and the register map
//! are identical, the packet memory is 1 KiB "structured as 512 half-words of
//! 16 bits" with the linear (one byte per byte address) layout in both. Two
//! things differ, and neither is a register:
//!
//! * The F0 places the block on "APB", the WB on "APB1". Nothing here sees the
//!   difference.
//! * The F0x2's packet memory is shared with its CAN: "When the CAN peripheral
//!   clock is enabled in the RCC_APB1ENR register, only the first 768 Bytes are
//!   available to USB while the last 256 Bytes are used by CAN" (RM0091 Table
//!   121, note 2). That is the `can-enable` input; the WB has no CAN and leaves
//!   it unwired, which is the whole 1 KiB.
//!
//! # The register file
//!
//! Register map (RM0434 §40.6.3 Table 263, RM0091 §30.6.3), offsets from the
//! block base. Each register is sixteen bits in the low half of a thirty-two-
//! bit word, and "can be accessed by half-words (16-bit) or words (32-bit)";
//! the high half and the gap at `0x20`…`0x3c` are reserved. The block's
//! boundary-address slot is 1 KiB; the window this model decodes ends after
//! `BCDR`.
//!
//! | Offset | Register | Reset | Implemented bits |
//! | --- | --- | --- | --- |
//! | `0x00`…`0x1c` | `EP0R`…`EP7R` | `0x0000` | all sixteen (rules below) |
//! | `0x40` | `CNTR` | `0x0003` | the eight interrupt masks `CTRM`…`L1REQM` (15:7), `L1RESUME`, `RESUME`, `FSUSP`, `LP_MODE`, `PDWN`, `FRES`; bit 6 reserved |
//! | `0x44` | `ISTR` | `0x0000` | `CTR` (r), the `rc_w0` flags `PMAOVR`…`L1REQ` (14:7), `DIR` and `EP_ID[3:0]` (r); 6:5 reserved |
//! | `0x48` | `FNR` | `0x0XXX` | `RXDP`, `RXDM`, `LCK`, `LSOF[1:0]`, `FN[10:0]`, all read-only |
//! | `0x4c` | `DADDR` | `0x0000` | `EF` (7), `ADD[6:0]`; 15:8 reserved |
//! | `0x50` | `BTABLE` | `0x0000` | `BTABLE[15:3]`; 2:0 "forced by hardware to 0" |
//! | `0x54` | `LPMCSR` | `0x0000` | `BESL[3:0]`, `REMWAKE` (r), `LPMACK`, `LPMEN` (rw); bit 2 and 15:8 reserved |
//! | `0x58` | `BCDR` | `0x0000` | `DPPU` (15), `SDEN`/`PDEN`/`DCDEN`/`BCDEN` (3:0) rw; `PS2DET`/`SDET`/`PDET`/`DCDET` (7:4) r; 14:8 reserved |
//!
//! A write never sets a reserved or read-only bit; each register's mask is
//! named beside its offset below. A word write is the register in its low half
//! and a dropped high half. An offset outside the table is a bus fault here;
//! the manual says only "Reserved".
//!
//! The endpoint register is the one with the access rules a driver is written
//! around, and getting them wrong is a stack that hangs rather than a value
//! that looks off:
//!
//! * `CTR_RX` (15) and `CTR_TX` (7) are **`rc_w0`**: hardware sets them when a
//!   transaction completes, and software clears a bit by writing **zero** to
//!   it. Writing one leaves it as it was — which is what lets a driver clear
//!   one direction's flag without touching the other's in a single store.
//! * `DTOG_RX` (14), `STAT_RX` (13:12), `DTOG_TX` (6) and `STAT_TX` (5:4) are
//!   **toggle** (`t`): a written one flips the bit, a written zero leaves it. A
//!   driver that wants `STAT_RX = VALID` writes the exclusive-or of where it is
//!   and where it wants to be, and the hardware does the rest.
//! * `SETUP` (11) is **read-only** — hardware sets it on a `SETUP` packet.
//! * `EP_TYPE` (10:9), `EP_KIND` (8) and `EA` (3:0) are plain read/write.
//!
//! `ISTR`'s flag bits are `rc_w0` as a group (a write ANDs), while `CTR`, `DIR`
//! and `EP_ID` are read-only and **computed** from the endpoint registers'
//! pending `CTR_RX`/`CTR_TX`. That is how a driver's interrupt handler finds
//! the endpoint to service without scanning all eight.
//!
//! What the controller does on the wire — the transaction path, data toggles,
//! double-buffered and isochronous endpoints, and the bus events (`RESET`,
//! `SOF`/`ESOF`, suspend and resume, LPM) — is the code below, and its comments
//! cite the manual section by section.
//!
//! # Battery charging detection (`BCDR`)
//!
//! The detector's results are a property of what the cable is plugged into,
//! not of the chip, so they come from the `port-type` property: `sdp` (the
//! default — a standard host port, which is what every board written before
//! the property assumed), `cdp`, `dcp`, `ps2` or `none`. RM0434 §40.6.1 gives
//! the bits and no sequence beyond them; [`bcd_status`] has the table and the
//! line conditions from USB Battery Charging 1.2 that decide each result.
//! While `BCDEN` is set the block is off the bus: "the USB PHY is fully
//! controlled by BCD and cannot be used for normal communication".
//!
//! # The packet memory
//!
//! A separate region — the block's SRAM — mapped at its own base: a flat run of
//! 1024 bytes, addressed the way any other memory is. "The packet memory
//! should be accessed only by byte (8-bit) or half-word (16-bit) accesses.
//! Word (32-bit) accesses are not allowed" (RM0434 §40.6.2), so the region's
//! constraints refuse a word. The buffer-descriptor table lives at `BTABLE`
//! and holds, per endpoint register, four sixteen-bit words — `ADDR_TX`,
//! `COUNT_TX`, `ADDR_RX`, `COUNT_RX` — and the data buffers sit at those
//! `ADDR_*` byte offsets. `COUNT_RX`'s high bits are the allocated block size
//! and are left alone; hardware writes only the low ten bits, the received
//! byte count.
//!
//! With `can-enable` high (F0x2 only) the top 256 bytes are CAN's. RM0091 does
//! not say what a USB or CPU access there does; this model reads zero and
//! drops the write, for the controller and the CPU alike, and leaves the bytes
//! USB had there as they were.
//!
//! # On the bus
//!
//! A [`UsbDevice`]. The host's `SETUP`, `IN` and `OUT` transactions are
//! matched to an endpoint register by its `EA` field — not by the register's
//! position — exactly as the token's address field is matched on silicon. The
//! device signals **full speed** and answers at `DADDR`'s address. It is on the
//! bus while it is clocked, out of `PDWN` and `FRES`, its `DPPU` pull-up is
//! set and `BCDEN` is clear.
//!
//! # The clock, the interrupt, the inputs
//!
//! `enable` and `reset` arrive from the reset and clock controller, as
//! [`gate`] describes; an unwired `enable` leaves the block clocked, so a board
//! written before the gate existed does not change. `can-enable` is the
//! F0x2's `RCC_APB1ENR.CANEN`; like the gate's levels it is wiring, not
//! snapshot state — the RCC republishes it on load. The `irq` output carries
//! the low-priority USB interrupt — level, the OR of the enabled `ISTR` flags
//! under `CNTR`'s mask — and the board wires it to its core's USB vector;
//! `irq_hp` carries the high-priority one (see [`pin::IRQ_HP`] for what feeds
//! it and how each family wires it).
//!
//! # Sources
//!
//! ST **RM0091** rev 10 §30 "Universal serial bus full-speed device interface
//! (USB)" (STM32F0x2), and ST **RM0434** rev 16 §40 of the same title
//! (STM32WB). USB Battery Charging Specification revision 1.2 for what each
//! kind of port does to the data lines during detection. No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{
    Completion, DataPid, DeviceAddress, SetupPacket, Speed, Status, UsbBus, UsbDevice, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::AtomicBool;
use crate::core::sync::{AtomicU32, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.usbfs";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 2;

/// How many endpoint registers the block has. Fixed on this peripheral.
const ENDPOINTS: usize = 8;

/// Register-window size, through `BCDR`.
pub const REGISTER_BYTES: u64 = 0x60;

/// Packet-buffer memory size: 1 KiB on both families — "The size of the Packet
/// Memory is 1024 bytes, structured as 512 half-words of 16 bits" (RM0434
/// §40.4.1; RM0091 §30.4.1 says "up to").
pub const PMA_BYTES: u64 = 0x400;

/// How much of the packet memory is USB's while the F0x2's CAN clock runs:
/// "When the CAN peripheral clock is enabled in the RCC_APB1ENR register, only
/// the first 768 Bytes are available to USB while the last 256 Bytes are used
/// by CAN" (RM0091 Table 121, note 2).
pub const PMA_BYTES_WITH_CAN: u64 = 0x300;

// Register word offsets (each register is the low half of its 32-bit word).
const OFF_CNTR: u64 = 0x40;
const OFF_ISTR: u64 = 0x44;
const OFF_FNR: u64 = 0x48;
const OFF_DADDR: u64 = 0x4c;
const OFF_BTABLE: u64 = 0x50;
const OFF_LPMCSR: u64 = 0x54;
const OFF_BCDR: u64 = 0x58;

// --- EPnR bit groups ---
const EP_CTR_RX: u16 = 1 << 15;
const EP_DTOG_RX: u16 = 1 << 14;
const EP_STAT_RX: u16 = 0b11 << 12;
const EP_SETUP: u16 = 1 << 11;
const EP_CTR_TX: u16 = 1 << 7;
const EP_DTOG_TX: u16 = 1 << 6;
const EP_STAT_TX: u16 = 0b11 << 4;
const EP_EA: u16 = 0x000f;
/// `EP_TYPE` (10:9): Table 259, `00` bulk, `01` control, `10` iso, `11` interrupt.
const EP_TYPE: u16 = 0b11 << 9;
/// `EP_KIND` (8): `DBL_BUF` on a bulk endpoint, `STATUS_OUT` on a control one,
/// unused on the other two (Table 260).
const EP_KIND: u16 = 1 << 8;
const TYPE_BULK: u16 = 0b00;
const TYPE_CONTROL: u16 = 0b01;
const TYPE_ISO: u16 = 0b10;
/// The plain read/write bits: `EP_TYPE` (10:9), `EP_KIND` (8) and `EA` (3:0).
const EP_RW: u16 = 0b0000_0111_0000_1111;

/// `STAT_RX`/`STAT_TX` field values (RM0091 §30.6.2, Table).
const STAT_DISABLED: u16 = 0b00;
/// `0b01` is STALL; this model reaches it through [`State::stat_rx`] /
/// [`State::stat_tx`] comparisons rather than naming it.
const STAT_NAK: u16 = 0b10;
const STAT_VALID: u16 = 0b11;

// --- CNTR bits ---
/// The interrupt-enable bits that line up with the `ISTR` flags, `CTRM` (15)
/// down through `L1REQM` (7) — each gates the interrupt its `ISTR` flag raises.
const CNTR_IRQ_MASK: u16 = 0xff80;
/// `CNTR`'s implemented bits: all but bit 6, "Reserved, must be kept at reset
/// value" (RM0434 §40.6.1).
const CNTR_WRITABLE: u16 = 0xffbf;
const CNTR_PDWN: u16 = 1 << 1;
const CNTR_FRES: u16 = 1 << 0;
/// `CNTR`'s reset value: powered down and held in reset (RM0091 §30.6.1).
const CNTR_RESET: u16 = CNTR_PDWN | CNTR_FRES;

// --- ISTR bits ---
const ISTR_CTR: u16 = 1 << 15;
const ISTR_RESET: u16 = 1 << 10;
const ISTR_DIR: u16 = 1 << 4;
/// The latched, `rc_w0`-writable flags: `PMAOVR` (14) down through `L1REQ` (7).
/// `CTR`, `DIR` and `EP_ID` are read-only and computed.
const ISTR_FLAG_MASK: u16 = 0x7f80;

// --- DADDR / BTABLE / LPMCSR / BCDR ---
const DADDR_ADD: u16 = 0x007f;
/// `EF` (7) and `ADD[6:0]`; bits 15:8 are "Reserved".
const DADDR_WRITABLE: u16 = 0x00ff;
/// `BTABLE[15:3]`; bits 2:0 are "Reserved, forced by hardware to 0".
const BTABLE_WRITABLE: u16 = 0xfff8;
/// `LPMACK` (1) and `LPMEN` (0) are the only `rw` bits; `BESL[3:0]` (7:4) and
/// `REMWAKE` (3) are `r`, written by the hardware from the last ACKed LPM
/// token, and bit 2 is reserved.
const LPMCSR_WRITABLE: u16 = 0x0003;
/// `DADDR.EF` (7): "If this bit is at '0 no transactions are handled,
/// irrespective of the settings of USB_EPnR registers." (RM0434 §40.6.1)
const DADDR_EF: u16 = 1 << 7;
const BCDR_DPPU: u16 = 1 << 15;
const BCDR_PS2DET: u16 = 1 << 7;
const BCDR_SDET: u16 = 1 << 6;
const BCDR_PDET: u16 = 1 << 5;
const BCDR_DCDET: u16 = 1 << 4;
const BCDR_SDEN: u16 = 1 << 3;
const BCDR_PDEN: u16 = 1 << 2;
const BCDR_DCDEN: u16 = 1 << 1;
const BCDR_BCDEN: u16 = 1 << 0;
/// The `rw` bits of `BCDR`: `DPPU` and the four enables. The detection
/// results (7:4) are `r` and computed ([`bcd_status`]); 14:8 are reserved.
const BCDR_WRITABLE: u16 = BCDR_DPPU | BCDR_SDEN | BCDR_PDEN | BCDR_DCDEN | BCDR_BCDEN;

/// Low ten bits of a `COUNT_*` word: the byte count. The high bits are the RX
/// buffer's allocated block size and are the application's, not ours.
const COUNT_MASK: u16 = 0x03ff;
/// `COUNTn_RX.BL_SIZE` (15): blocks of two bytes when clear, of 32 when set.
const COUNT_BL_SIZE: u16 = 1 << 15;

/// How a register's endpoint uses its buffer-descriptor entry.
///
/// The two special kinds are the ones RM0434 §40.5.3/§40.5.4 give "special
/// handling of packet buffers": both descriptor slots serve the one enabled
/// direction, and the direction's `DTOG` bit picks which.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// Control, interrupt, or bulk without `DBL_BUF`: slot 0 is the TX
    /// buffer, slot 1 the RX buffer.
    Single,
    /// Bulk with `EP_KIND = DBL_BUF`: "Writing EP_TYPE bit field at '00 [...]
    /// and Setting EP_KIND bit at '1 (DBL_BUF)" (§40.5.3).
    DoubleBulk,
    /// `EP_TYPE = 10`: "The Isochronous behavior for an endpoint is selected by
    /// setting the EP_TYPE bits at '10" (§40.5.4).
    Iso,
}

impl Kind {
    fn of(reg: u16) -> Kind {
        match (reg & EP_TYPE) >> 9 {
            TYPE_ISO => Kind::Iso,
            TYPE_BULK if reg & EP_KIND != 0 => Kind::DoubleBulk,
            _ => Kind::Single,
        }
    }

    /// Whether this endpoint is in the set the `ISTR.EP_ID` priority and the
    /// high-priority line single out: "Isochronous and double-buffered bulk
    /// endpoints are considered first" (§40.6.1, `EP_ID`).
    fn is_priority(self) -> bool {
        self != Kind::Single
    }
}

fn is_control(reg: u16) -> bool {
    (reg & EP_TYPE) >> 9 == TYPE_CONTROL
}

/// The allocated size of a reception buffer, from its `COUNTn_RX` word
/// (RM0434 §40.6.2, `BL_SIZE`/`NUM_BLOCK` and Table 262).
///
/// "If BL_SIZE=0, the memory block is 2-byte large [...] If BL_SIZE=1, the
/// memory block is 32-byte large". Table 262 counts `NUM_BLOCK` blocks when
/// `BL_SIZE = 0` (1 → 2 bytes, 31 → 62) and `NUM_BLOCK + 1` when it is set
/// (0 → 32 bytes, 14 → 480). `NUM_BLOCK = 0` with `BL_SIZE = 0` is "Not
/// allowed"; it is taken as no room at all, so nothing is written. The table
/// stops filling the `BL_SIZE = 1` column after 14 and marks 31 "N/A"; the text
/// says the size "theoretically ranges from 32 to 1024 bytes", so the same
/// `+ 1` rule is carried on — the packet memory's own end bounds it anyway.
fn rx_capacity(count_rx: u16) -> usize {
    let blocks = usize::from((count_rx >> 10) & 0x1f);
    if count_rx & COUNT_BL_SIZE != 0 {
        (blocks + 1) * 32
    } else {
        blocks * 2
    }
}

/// The CRC16 a data packet carries (USB 2.0 §8.3.5.2: generator
/// `x^16 + x^15 + x^2 + 1`, register seeded with ones, the remainder sent
/// inverted, least-significant bit first).
///
/// The block writes it into the reception buffer behind the payload: buffer
/// locations are written "for a number of bytes corresponding to the received
/// data packet length, CRC included (i.e. data payload length + 2), or up to
/// the last allocated memory location [...] whichever comes first" (RM0434
/// §40.5.2). The two bytes land in wire order, which is the CRC's low byte
/// first.
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xa001
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest and the bus can see or change.
#[derive(Debug, Clone)]
struct State {
    /// The eight endpoint registers.
    ep: [u16; ENDPOINTS],
    cntr: u16,
    /// `ISTR` flag bits only; `CTR`/`DIR`/`EP_ID` are computed on read.
    istr: u16,
    daddr: u16,
    btable: u16,
    lpmcsr: u16,
    bcdr: u16,
    /// The packet-buffer memory.
    pma: Vec<u8>,
    /// How many bytes of [`pma`](State::pma), from the bottom, are USB's:
    /// all of it, or [`PMA_BYTES_WITH_CAN`] while the F0x2's CAN clock runs.
    ///
    /// Not guest state: it follows the `can-enable` wire, and is re-derived
    /// from it whenever the state is rebuilt ([`Usbfs::set_state`]).
    usb_pma_len: usize,
    /// Per endpoint register (bit `n`): a double-buffered bulk endpoint has
    /// completed its first transaction since `DBL_BUF` was set, so its special
    /// flow control is in force. "The end of the first transaction occurring
    /// after having set DBL_BUF, triggers the special flow control of
    /// double-buffered bulk endpoints, which is used for all other
    /// transactions addressed to this endpoint until DBL_BUF remain set"
    /// (RM0434 §40.5.3). Hidden hardware state, so it is in the snapshot.
    dbl_live: u8,
}

impl State {
    fn reset() -> State {
        State {
            ep: [0; ENDPOINTS],
            cntr: CNTR_RESET,
            istr: 0,
            daddr: 0,
            btable: 0,
            lpmcsr: 0,
            bcdr: 0,
            pma: vec![0; PMA_BYTES as usize],
            usb_pma_len: PMA_BYTES as usize,
            dbl_live: 0,
        }
    }

    fn stat_rx(&self, n: usize) -> u16 {
        (self.ep[n] & EP_STAT_RX) >> 12
    }
    fn stat_tx(&self, n: usize) -> u16 {
        (self.ep[n] & EP_STAT_TX) >> 4
    }
    /// Force `STAT_RX` of endpoint `n` to `v` (a hardware action, a direct set
    /// rather than the guest's toggle).
    fn set_stat_rx(&mut self, n: usize, v: u16) {
        self.ep[n] = (self.ep[n] & !EP_STAT_RX) | ((v & 0b11) << 12);
    }
    fn set_stat_tx(&mut self, n: usize, v: u16) {
        self.ep[n] = (self.ep[n] & !EP_STAT_TX) | ((v & 0b11) << 4);
    }

    /// The packet-memory byte at `off` as the controller sees it — zero past
    /// the end, and zero in the part CAN holds (see [`pma_slot`](State::pma_slot)).
    ///
    /// Every controller-side access to the packet memory goes through this or
    /// [`pma_slot`](State::pma_slot), so the CAN share is honoured in one place.
    fn pma_byte(&self, off: usize) -> u8 {
        if off < self.usb_pma_len {
            self.pma.get(off).copied().unwrap_or(0)
        } else {
            0
        }
    }
    /// The packet-memory byte at `off` for writing, or `None` past the end or
    /// in the part CAN holds: RM0091 says only that those 256 bytes "are used
    /// by CAN", not what a USB access there does, so the conservative reading
    /// is that it reaches nothing — a read gets zero, a write is lost, and the
    /// bytes USB had there are left as they were.
    fn pma_slot(&mut self, off: usize) -> Option<&mut u8> {
        if off < self.usb_pma_len {
            self.pma.get_mut(off)
        } else {
            None
        }
    }
    fn pma_u16(&self, off: u16) -> u16 {
        let o = off as usize;
        u16::from_le_bytes([self.pma_byte(o), self.pma_byte(o + 1)])
    }
    fn set_pma_u16(&mut self, off: u16, v: u16) {
        self.pma_write(off, &v.to_le_bytes());
    }
    fn pma_write(&mut self, addr: u16, data: &[u8]) {
        for (i, b) in data.iter().enumerate() {
            if let Some(slot) = self.pma_slot(addr as usize + i) {
                *slot = *b;
            }
        }
    }

    /// The buffer-descriptor-table byte offset of endpoint register `n`.
    fn btable_entry(&self, n: usize) -> u16 {
        self.btable.wrapping_add((n as u16) * 8)
    }
    /// `(ADDR, &COUNT)` of descriptor slot `slot` (0 or 1) of register `n`.
    ///
    /// Slot 0 is the entry's first pair (`ADDRn_TX`/`COUNTn_TX`, or the
    /// `_TX_0`/`_RX_0` buffer of a double-buffered or isochronous endpoint),
    /// slot 1 its second (`ADDRn_RX`/`COUNTn_RX`, or `_TX_1`/`_RX_1`) —
    /// §40.6.2's notes on each location.
    fn slot(&self, n: usize, slot: usize) -> (u16, u16) {
        let e = self.btable_entry(n).wrapping_add((slot as u16) * 4);
        (self.pma_u16(e), e.wrapping_add(2))
    }

    /// Copy a received data packet into descriptor slot `slot` of register `n`
    /// and say whether it fitted.
    ///
    /// "The USB peripheral will never change the contents of memory locations
    /// adjacent to the allocated memory buffers; if a packet bigger than the
    /// allocated buffer length is received (buffer overrun condition) the data
    /// are copied to the memory only up to the last available location"
    /// (RM0434 §40.5.2). What is written is the payload and its CRC, cut at
    /// the allocated size; the overrun is "the length of the data packet
    /// payload [...] greater than the allocated buffer". `COUNTn_RX` is not
    /// touched here — on an overrun it must not be.
    fn receive(&mut self, n: usize, slot: usize, data: &[u8]) -> bool {
        let (addr, count) = self.slot(n, slot);
        let capacity = rx_capacity(self.pma_u16(count));
        let crc = crc16(data).to_le_bytes();
        for (i, b) in data.iter().chain(crc.iter()).take(capacity).enumerate() {
            if let Some(cell) = self.pma_slot(usize::from(addr) + i) {
                *cell = *b;
            }
        }
        data.len() <= capacity
    }

    /// Write the received byte count into slot `slot`'s `COUNT` word, keeping
    /// `BL_SIZE`/`NUM_BLOCK`: "the internal COUNT register is copied back in
    /// the COUNTn_RX location [...] leaving unaffected BL_SIZE and NUM_BLOCK
    /// fields" (§40.5.2).
    fn set_rx_count(&mut self, n: usize, slot: usize, len: usize) {
        let (_, count) = self.slot(n, slot);
        let old = self.pma_u16(count);
        let len = len.min(usize::from(COUNT_MASK)) as u16;
        self.set_pma_u16(count, (old & !COUNT_MASK) | len);
    }

    /// The `STAT` side of a completed double-buffered bulk transaction on
    /// register `n` (`rx`: the reception direction).
    ///
    /// The first transaction after `DBL_BUF` is set ends the ordinary way —
    /// "Unlike common transactions, and the first one after DBL_BUF setting,
    /// STAT bit pair is not affected by the transaction termination and its
    /// value remains '11 (Valid)" (RM0434 §40.5.3) — so it is forced to `NAK`,
    /// and the special flow control takes over from the next one.
    fn finish_double(&mut self, n: usize, rx: bool) {
        if self.dbl_live & (1 << n) == 0 {
            if rx {
                self.set_stat_rx(n, STAT_NAK);
            } else {
                self.set_stat_tx(n, STAT_NAK);
            }
            self.dbl_live |= 1 << n;
        }
    }

    /// Set `CTR_RX` on register `n` with the `SETUP` bit's rule: it "changes
    /// its value only for control endpoints" and "is kept frozen while CTR_RX
    /// bit is at 1" (§40.6.1, `SETUP`).
    fn complete_rx(&mut self, n: usize, setup: bool) {
        let reg = self.ep[n];
        let mut new = reg | EP_CTR_RX;
        if is_control(reg) && reg & EP_CTR_RX == 0 {
            new = if setup {
                new | EP_SETUP
            } else {
                new & !EP_SETUP
            };
        }
        self.ep[n] = new;
    }

    /// The endpoint register whose `EA` matches `ea`, lowest index first.
    ///
    /// This is the address match the token's `ADDR` field gets on silicon, and
    /// why `EA` rather than the register's position decides which buffer a
    /// transaction lands in.
    fn match_ea(&self, ea: u16) -> Option<usize> {
        (0..ENDPOINTS).find(|&n| self.ep[n] & EP_EA == ea & EP_EA)
    }

    /// `ISTR` as a read answers it: the latched flags, plus the computed
    /// `CTR`/`DIR`/`EP_ID` naming the highest-priority pending transfer.
    ///
    /// The priority is RM0434 §40.6.1's (`EP_ID`): "Two endpoint sets are
    /// defined, in order of priority: Isochronous and double-buffered bulk
    /// endpoints are considered first and then the other endpoints are
    /// examined. If more than one endpoint from the same set is requesting an
    /// interrupt, the EP_ID bits [...] are assigned according to the lowest
    /// requesting endpoint register". `DIR` is 1 when "CTR_RX bit or both
    /// CTR_TX/CTR_RX are set" on that register.
    fn istr_value(&self) -> u16 {
        for priority in [true, false] {
            for n in 0..ENDPOINTS as u16 {
                let ep = self.ep[n as usize];
                if Kind::of(ep).is_priority() != priority {
                    continue;
                }
                if ep & EP_CTR_RX != 0 {
                    return self.istr | ISTR_CTR | ISTR_DIR | n;
                }
                if ep & EP_CTR_TX != 0 {
                    return self.istr | ISTR_CTR | n;
                }
            }
        }
        self.istr & !ISTR_CTR
    }

    /// Whether the high-priority line stands: a pending `CTR_RX`/`CTR_TX` on
    /// an isochronous or double-buffered bulk endpoint.
    ///
    /// The manual's grounds, quoted: "Endpoint transaction completion can be
    /// handled in a different way to reduce interrupt response latency. [...]
    /// An endpoint dedicated interrupt condition is activated independently
    /// from the CTRM bit in the USB_CNTR register. Both interrupt conditions
    /// remain active until software clears the pending bit in the
    /// corresponding USB_EPnR register" (RM0434 §40.6.1, `USB_ISTR`), and the
    /// vector table's "USB_HP — USB high priority interrupt" beside "USB_LP —
    /// USB low priority interrupt (including USB wake-up)" (RM0434 Table 72,
    /// the CPU1 vector table). So the line ignores `CTRM` and falls only when
    /// `CTR_x` is cleared. **Which endpoints feed it is not stated in RM0434
    /// or RM0091** — this is an assumption, not a quote: the set taken is the
    /// one the same register description already singles out for latency,
    /// "Isochronous and double-buffered bulk endpoints are considered first".
    /// Evidence from silicon that says otherwise should change this function
    /// and nothing else.
    fn irq_hp_asserted(&self) -> bool {
        self.ep
            .iter()
            .any(|&ep| Kind::of(ep).is_priority() && ep & (EP_CTR_RX | EP_CTR_TX) != 0)
    }

    /// Whether the interrupt line stands: an enabled `ISTR` flag under `CNTR`'s
    /// mask. `CTR` rides in through [`istr_value`](State::istr_value).
    fn irq_asserted(&self) -> bool {
        self.istr_value() & self.cntr & CNTR_IRQ_MASK != 0
    }
}

// ---------------------------------------------------------------------------
// Battery charging detection
// ---------------------------------------------------------------------------

/// What the cable is plugged into, as the USB Battery Charging specification
/// 1.2 classifies the far end — the one thing `BCDR`'s detection results
/// depend on, and a fact about the attachment rather than the chip.
///
/// A closed set on purpose: [`bcd_status`] matches it exhaustively, and a new
/// kind of port is a new row of that table, not a value a board can invent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortType {
    /// A Standard Downstream Port: an ordinary host or hub port. The default,
    /// and what every board written before this property existed plugs into.
    #[default]
    Sdp,
    /// A Charging Downstream Port: a host port that also supplies charging
    /// current, and answers primary detection.
    Cdp,
    /// A Dedicated Charging Port: a wall charger, `D+` shorted to `D-`, no host.
    Dcp,
    /// A port that pulls **both** data lines up — a PS/2 port, or a proprietary
    /// charger that does not follow BC 1.2 (RM0434 §40.6.1, `PS2DET`).
    Ps2,
    /// Nothing: the data lines float.
    Unplugged,
}

impl PortType {
    /// The values the `port-type` property takes, in [`PortType`] order.
    pub const NAMES: [&'static str; 5] = ["sdp", "cdp", "dcp", "ps2", "none"];

    /// Parse a `port-type` property value.
    #[must_use]
    pub fn from_name(name: &str) -> Option<PortType> {
        Some(match name {
            "sdp" => PortType::Sdp,
            "cdp" => PortType::Cdp,
            "dcp" => PortType::Dcp,
            "ps2" => PortType::Ps2,
            "none" => PortType::Unplugged,
            _ => return None,
        })
    }

    /// Data contact detection: with the device sourcing a small current into
    /// `D+` (BC 1.2 `IDP_SRC`) and pulling `D-` down (`RDM_DWN`), does `D+`
    /// read low? A host port's own `D+` pull-down does that for an SDP or CDP;
    /// a DCP's `D+`–`D-` short hands `D+` to the device's own `D-` pull-down. A
    /// port that pulls `D+` up, or no port at all, leaves `D+` high.
    fn data_contact(self) -> bool {
        matches!(self, PortType::Sdp | PortType::Cdp | PortType::Dcp)
    }

    /// Primary detection: with the device driving `VDP_SRC` onto `D+`, is `D-`
    /// above `VDAT_REF`? A CDP drives `D-` back, a DCP's short carries `D+`
    /// across, and a pull-up holds it high; an SDP's `D-` pull-down keeps it
    /// low, and so does the device's own sink current when nothing is there.
    fn primary(self) -> bool {
        matches!(self, PortType::Cdp | PortType::Dcp | PortType::Ps2)
    }

    /// Secondary detection, the mirror of primary: `VDM_SRC` onto `D-`, is `D+`
    /// above `VDAT_REF`? Only a port that joins the lines (DCP) or pulls `D+`
    /// up says yes; a CDP does not drive `D+` back, which is the distinction.
    fn secondary(self) -> bool {
        matches!(self, PortType::Dcp | PortType::Ps2)
    }

    /// The `PS2DET` comparator during primary detection: `D-` above `VLGC`,
    /// i.e. pulled up from outside rather than merely carrying `VDP_SRC`.
    fn dm_pulled_up(self) -> bool {
        matches!(self, PortType::Ps2)
    }
}

/// `BCDR`'s read-only result bits for a register holding `bcdr`, on a cable
/// into `port`.
///
/// RM0434 §40.6.1 (RM0091 §30.6.1 is word-for-word the same) gives the bits and
/// no sequence beyond them, so the table is the bit descriptions:
///
/// | Bit | Mode | 0 | 1 |
/// | --- | --- | --- | --- |
/// | `DCDET` (4) | `DCDEN` | "data lines contact not detected" | "data lines contact detected" |
/// | `PDET` (5) | `PDEN` | "no BCD support detected (connected to SDP or proprietary device)" | "BCD support detected (connected to ACA, CDP or DCP)" |
/// | `PS2DET` (7) | `PDEN` | "Normal port detected (connected to SDP, ACA, CDP or DCP)" | "PS2 port or proprietary charger detected" |
/// | `SDET` (6) | `SDEN` | "CDP detected" | "DCP detected" |
///
/// What decides each result is the line condition BC 1.2's detection steps
/// (data contact detect, primary detection, secondary detection) sense; the
/// [`PortType`] methods say which port meets which. Three rules are this
/// model's reading where the manual is silent:
///
/// * Nothing reads back unless `BCDEN` is set — the detector is off — and a
///   result shows only while its own mode is selected (the manual says this
///   outright for `PS2DET`: "active only during PD").
/// * "Only one detection mode (DCD, PD, SD or OFF) should be selected to work
///   correctly." With two or more selected every result reads zero, rather
///   than inventing which mode wins.
/// * The comparators are modelled as settled: the result is there on the
///   first read after the mode is selected. Firmware that waits out BC 1.2's
///   timings first sees the same answer it would on silicon.
fn bcd_status(bcdr: u16, port: PortType) -> u16 {
    if bcdr & BCDR_BCDEN == 0 {
        return 0;
    }
    let bit = |cond: bool, b: u16| if cond { b } else { 0 };
    match bcdr & (BCDR_DCDEN | BCDR_PDEN | BCDR_SDEN) {
        BCDR_DCDEN => bit(port.data_contact(), BCDR_DCDET),
        BCDR_PDEN => bit(port.primary(), BCDR_PDET) | bit(port.dm_pulled_up(), BCDR_PS2DET),
        BCDR_SDEN => bit(port.secondary(), BCDR_SDET),
        _ => 0,
    }
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// The USB device controller: the register file, the packet memory and the
/// gadget that is this block seen from the bus.
pub struct Usbfs {
    state: Mutex<State>,
    gate: ClockGate,
    /// The `irq` output (`USB_LP`), connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The `irq_hp` output (`USB_HP`), connected at realize time.
    irq_hp: Mutex<Option<WireSource>>,
    /// The bus this device plugs into when its pull-up is attached.
    bus: Arc<UsbBus>,
    /// Which port of [`bus`](Usbfs::bus) it plugs into.
    port: u8,
    /// `0` while off the bus, `1` while attached. A latch, not guest state.
    attached: AtomicU32,
    /// This block seen from the bus. Holds a [`Weak`] back here.
    gadget: Mutex<Option<Arc<Gadget>>>,
    /// What the cable is plugged into — configuration, fixed at construction.
    port_type: PortType,
    /// The F0x2's `RCC_APB1ENR.CANEN`, as the `can-enable` wire last
    /// delivered it. Low (and so all 1 KiB USB's) when nothing drives it.
    can: AtomicBool,
    /// The `can-enable` pins built for this block. A net holds its sinks
    /// weakly, so the strong reference lives here.
    can_pins: Mutex<Vec<Arc<CanPin>>>,
}

impl fmt::Debug for Usbfs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Usbfs")
            .field("port", &self.port)
            .field("attached", &(self.attached.load(Ordering::Relaxed) != 0))
            .field("port_type", &self.port_type)
            .field("can", &self.can.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Usbfs {
    /// Build the engine on `bus`, its device half plugging into `port`, on a
    /// cable into a standard host port.
    #[must_use]
    pub fn new(bus: Arc<UsbBus>, port: u8) -> Arc<Usbfs> {
        Usbfs::with_port_type(bus, port, PortType::Sdp)
    }

    /// The same, with the cable plugged into `port_type` — what `BCDR`'s
    /// battery-charging detection finds.
    #[must_use]
    pub fn with_port_type(bus: Arc<UsbBus>, port: u8, port_type: PortType) -> Arc<Usbfs> {
        let me = Arc::new(Usbfs {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            gate: ClockGate::new(),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            irq_hp: Mutex::with_rank(LockRank::WIRE, None),
            bus,
            port,
            attached: AtomicU32::new(0),
            gadget: Mutex::with_rank(LockRank::WIRE, None),
            port_type,
            can: AtomicBool::new(false),
            can_pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        });
        *me.gadget.lock() = Some(Arc::new(Gadget {
            core: Arc::downgrade(&me),
        }));
        me
    }

    /// The bus it is on.
    #[must_use]
    pub fn bus(&self) -> &Arc<UsbBus> {
        &self.bus
    }

    /// Whether the device half is currently on the bus.
    #[must_use]
    pub fn is_attached(&self) -> bool {
        self.attached.load(Ordering::Relaxed) != 0
    }

    /// The level the interrupt output is being driven to.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.state.lock().irq_asserted())
    }

    /// The level the high-priority interrupt output is being driven to.
    #[must_use]
    pub fn irq_hp_level(&self) -> Level {
        Level::from_bool(self.state.lock().irq_hp_asserted())
    }

    /// `ISTR` as the guest would read it.
    #[must_use]
    pub fn interrupt_status(&self) -> u16 {
        self.state.lock().istr_value()
    }

    /// What the cable is plugged into.
    #[must_use]
    pub fn port_type(&self) -> PortType {
        self.port_type
    }

    /// Whether the F0x2's CAN currently holds the top 256 bytes of packet
    /// memory.
    #[must_use]
    pub fn can_holds_pma(&self) -> bool {
        self.can.load(Ordering::Relaxed)
    }

    /// Drive the `can-enable` input directly — the route a test takes.
    pub fn set_can_enable(&self, high: bool) {
        self.can.store(high, Ordering::Relaxed);
        self.state.lock().usb_pma_len = Usbfs::usb_pma_len(high);
    }

    fn usb_pma_len(can: bool) -> usize {
        if can {
            PMA_BYTES_WITH_CAN as usize
        } else {
            PMA_BYTES as usize
        }
    }

    /// Replace the state wholesale (reset, snapshot load), keeping the one
    /// field that follows a wire rather than the guest.
    fn set_state(&self, mut state: State) {
        state.usb_pma_len = Usbfs::usb_pma_len(self.can.load(Ordering::Relaxed));
        *self.state.lock() = state;
    }

    /// Whether the device is electrically on the bus: clocked, powered, out of
    /// reset, its `DPPU` pull-up attached, and the PHY not lent to the charger
    /// detector — "When enabled, the USB PHY is fully controlled by BCD and
    /// cannot be used for normal communication" (`BCDR.BCDEN`, RM0434 §40.6.1).
    fn connected(state: &State, gate: &ClockGate) -> bool {
        gate.live()
            && state.bcdr & BCDR_DPPU != 0
            && state.bcdr & BCDR_BCDEN == 0
            && state.cntr & CNTR_PDWN == 0
            && state.cntr & CNTR_FRES == 0
    }

    /// Put the gadget on the bus, or take it off, to match the pull-up and
    /// power bits. **No lock of ours is held** — this reaches the fabric.
    pub fn settle(&self) {
        let want = {
            let state = self.state.lock();
            Usbfs::connected(&state, &self.gate)
        };
        if want {
            if self
                .attached
                .compare_exchange(0, 1, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                let gadget = self
                    .gadget
                    .lock()
                    .clone()
                    .expect("the gadget is built in `new`");
                if self
                    .bus
                    .attach(self.port, gadget as Arc<dyn UsbDevice>)
                    .is_err()
                {
                    // Something else is already in that port — a machine
                    // description bug; the honest outcome is a device not on
                    // the bus.
                    self.attached.store(0, Ordering::Relaxed);
                }
            }
        } else if self
            .attached
            .compare_exchange(1, 0, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            self.bus.detach(self.port);
        }
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.irq.lock() = Some(source);
        self.refresh_irq();
    }

    /// Connect the high-priority interrupt output.
    pub fn connect_irq_hp(&self, source: WireSource) {
        *self.irq_hp.lock() = Some(source);
        self.refresh_irq();
    }

    /// Republish both interrupt lines. Called with **no lock held**.
    pub fn refresh_irq(&self) {
        let (lp, hp) = {
            let state = self.state.lock();
            (state.irq_asserted(), state.irq_hp_asserted())
        };
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(lp));
        }
        let source = self.irq_hp.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(hp));
        }
    }

    fn do_reset(&self) {
        self.set_state(State::reset());
    }

    // -- register file --------------------------------------------------

    /// Read the register at word offset `word`, or `None` for one this block
    /// does not decode.
    fn read_reg(&self, word: u64) -> Option<u16> {
        let state = self.state.lock();
        Some(match word {
            o if o < 0x20 => state.ep[(o / 4) as usize],
            OFF_CNTR => state.cntr,
            OFF_ISTR => state.istr_value(),
            OFF_FNR => 0,
            OFF_DADDR => state.daddr,
            OFF_BTABLE => state.btable,
            OFF_LPMCSR => state.lpmcsr,
            OFF_BCDR => state.bcdr | bcd_status(state.bcdr, self.port_type),
            _ => return None,
        })
    }

    /// Write `value` to the register at word offset `word`. Returns whether the
    /// decode succeeded and whether the bus attachment may have changed.
    fn write_reg(&self, word: u64, value: u16) -> (bool, bool) {
        let mut state = self.state.lock();
        match word {
            o if o < 0x20 => {
                let n = (o / 4) as usize;
                write_epnr(&mut state.ep[n], value);
                if Kind::of(state.ep[n]) != Kind::DoubleBulk {
                    // `DBL_BUF` (or bulk) is gone: the next time it is set,
                    // its first transaction is an ordinary one again.
                    state.dbl_live &= !(1 << n);
                }
            }
            OFF_CNTR => state.cntr = value & CNTR_WRITABLE,
            OFF_ISTR => state.istr &= value & ISTR_FLAG_MASK,
            // Every bit is `r`; the write decodes and changes nothing.
            OFF_FNR => {}
            OFF_DADDR => state.daddr = value & DADDR_WRITABLE,
            OFF_BTABLE => state.btable = value & BTABLE_WRITABLE,
            OFF_LPMCSR => {
                state.lpmcsr = (state.lpmcsr & !LPMCSR_WRITABLE) | (value & LPMCSR_WRITABLE);
            }
            // The result bits are computed on read and never stored.
            OFF_BCDR => state.bcdr = value & BCDR_WRITABLE,
            _ => return (false, false),
        }
        // `CNTR.PDWN`/`FRES` and `BCDR.DPPU`/`BCDEN` decide whether the device
        // is on the bus; a write to either may have flipped that.
        let settle = matches!(word, OFF_CNTR | OFF_BCDR);
        (true, settle)
    }

    // -- the transactions the gadget runs -------------------------------

    /// A `SETUP` token and its eight-byte `DATA0` packet.
    ///
    /// RM0434 §40.5.2, "Control transfers", is the rule set:
    ///
    /// * "SETUP transactions are handled by control endpoints only" and "are
    ///   ignored by endpoints of other kinds" (§40.6.1, `EP_TYPE`) — no
    ///   handshake, which on this bus is [`Status::NoDevice`].
    /// * "When the STAT_RX bits are set to '01 (STALL) or '10 (NAK) and a
    ///   SETUP token is received, the USB accepts the data, performing the
    ///   required data transfers and sends back an ACK handshake." The
    ///   `EP_TYPE` description says the opposite for NAK ("the USB peripheral
    ///   will not answer, simulating a receive error"); the control-transfer
    ///   section is the detailed one, and USB 2.0 §8.5.3 requires a device to
    ///   accept a `SETUP`, so this follows it.
    /// * "If that endpoint has a previously issued CTR_RX request not yet
    ///   acknowledged by the application [...] the USB discards the SETUP
    ///   transaction and does not answer with any handshake packet regardless
    ///   of its state, simulating a reception error" — [`Status::Error`], the
    ///   host's cue to retry.
    /// * `STAT_RX = DISABLED`: "all reception requests addressed to this
    ///   endpoint are ignored" (Table 258).
    /// * On success "the values of DTOG_TX and DTOG_RX bits of the addressed
    ///   endpoint registers are set to 1 and 0 respectively [...] and both
    ///   STAT_TX and STAT_RX are set to '10 (NAK)". `DTOG_RX`'s own
    ///   description places the clear "at the reception of a SETUP PID", and
    ///   the same register's rule — it toggles "when the ACK handshake is sent
    ///   [...] following a data packet reception having a matching data PID
    ///   value" — then applies to the `SETUP`'s `DATA0`, so it reads 1
    ///   afterwards: the `DATA1` the first data stage carries. (Were it left
    ///   at 0, an `OUT` data stage would be a toggle mismatch.)
    fn do_setup(&self, endpoint: u8, packet: &SetupPacket) -> Status {
        let mut state = self.state.lock();
        if state.daddr & DADDR_EF == 0 {
            return Status::NoDevice;
        }
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return Status::NoDevice;
        };
        let reg = state.ep[n];
        if !is_control(reg) || state.stat_rx(n) == STAT_DISABLED {
            return Status::NoDevice;
        }
        if reg & EP_CTR_RX != 0 {
            return Status::Error;
        }
        // A control endpoint is single-buffered ("Endpoint 0 is always used
        // for control transfer in single-buffer mode", §40.4.1): slot 1 is RX.
        let bytes = packet.encode();
        if !state.receive(n, 1, &bytes) {
            // The overrun rule is stated for "OUT/SETUP" alike (§40.5.2).
            return Status::Stall;
        }
        state.set_rx_count(n, 1, bytes.len());
        state.complete_rx(n, true);
        state.ep[n] |= EP_DTOG_RX | EP_DTOG_TX;
        state.set_stat_rx(n, STAT_NAK);
        state.set_stat_tx(n, STAT_NAK);
        Status::Ack
    }

    /// An `OUT` token and its data packet. `pid` is the packet's data PID, or
    /// `None` when the host did not say — taken as the one expected.
    ///
    /// The single-buffered path is RM0434 §40.5.2, "OUT and SETUP packets":
    /// the addressed register's `STAT_RX` answers a non-`VALID` endpoint ("a
    /// NAK or STALL handshake packet is sent instead of the ACK [...] and no
    /// data is written in the reception memory buffers"), and on success
    /// "DTOG_RX bit is toggled, the endpoint is made invalid by setting
    /// STAT_RX = '10 (NAK) and bit CTR_RX is set". On an overrun "a STALL
    /// handshake is sent instead of the usual ACK [...] no interrupt is
    /// generated and the transaction is considered failed", and "none of the
    /// previously listed actions take place".
    fn do_out(&self, endpoint: u8, src: &[u8], pid: Option<DataPid>) -> Completion {
        let mut state = self.state.lock();
        if state.daddr & DADDR_EF == 0 {
            return Completion::absent();
        }
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return Completion::absent();
        };
        let reg = state.ep[n];
        let kind = Kind::of(reg);
        let dtog = reg & EP_DTOG_RX != 0;
        let live = state.dbl_live & (1 << n) != 0;
        match state.stat_rx(n) {
            STAT_VALID => {}
            STAT_NAK => return Completion::nak(),
            STAT_DISABLED => return Completion::absent(), // Table 258: ignored
            _ => return Completion::stall(),
        }
        if kind == Kind::DoubleBulk && live && dtog == (reg & EP_DTOG_TX != 0) {
            // "the actual endpoint status is masked as '10 (NAK) when a buffer
            // conflict between the USB peripheral and the application software
            // is detected (this condition is identified by DTOG and SW_BUF
            // having the same value" (§40.5.3). SW_BUF of a reception
            // endpoint is bit 6 (Table 254).
            return Completion::nak();
        }
        if kind != Kind::Iso && pid.is_some_and(|pid| pid.toggle() != dtog) {
            // A retransmission of a packet already taken. DTOG_RX toggles only
            // "following a data packet reception having a matching data PID
            // value", and CTR_RX is not set "as in the case of protocol errors
            // or data toggle mismatches" (§40.6.1). The manual does not name
            // the handshake; USB 2.0 §8.6.4 has the receiver ACK the duplicate
            // and discard it, which is what the host needs to move on. Nothing
            // is written: "no data is actually transferred".
            return Completion::ack(src.len() as u64);
        }
        if kind == Kind::Single && is_control(reg) && reg & EP_KIND != 0 && !src.is_empty() {
            // STATUS_OUT: "all OUT transactions containing more than zero data
            // bytes are answered 'STALL' instead of 'ACK'" (§40.6.1,
            // `EP_KIND`). The manual does not say whether the bytes reach the
            // buffer; a refused transaction writes none here.
            return Completion::stall();
        }
        let slot = match kind {
            Kind::Single => 1,
            // Tables 255/256: DTOG_RX = 0 → ADDRn_RX_0 (slot 0), 1 → RX_1.
            Kind::DoubleBulk | Kind::Iso => usize::from(dtog),
        };
        let fitted = state.receive(n, slot, src);
        if !fitted && kind != Kind::Iso {
            return Completion::stall();
        }
        // Isochronous: "CRC errors or buffer-overrun conditions occurring
        // during Isochronous OUT transfers are anyway considered as correct
        // transactions and they always trigger an CTR_RX event" (§40.5.4).
        // What COUNT then holds is not stated; the bytes that reached the
        // buffer is what it is given.
        let (_, count) = state.slot(n, slot);
        let len = src.len().min(rx_capacity(state.pma_u16(count)));
        state.set_rx_count(n, slot, len);
        state.complete_rx(n, false);
        state.ep[n] ^= EP_DTOG_RX;
        match kind {
            Kind::Single => state.set_stat_rx(n, STAT_NAK),
            // "STAT bit pair is not affected by transaction completion; [...]
            // the endpoint remains always '11 (Valid)" (§40.5.4).
            Kind::Iso => {}
            Kind::DoubleBulk => state.finish_double(n, true),
        }
        Completion::ack(src.len() as u64)
    }

    /// An `IN` token: the data packet the endpoint has staged, and the PID it
    /// went out with. `commit` is false for the debug peek, which changes
    /// nothing.
    ///
    /// RM0434 §40.5.2, "IN packets": the packet goes out "according to
    /// USB_EPnR bit DTOG_TX"; "If the addressed endpoint is not valid, a NAK or
    /// STALL handshake packet is sent instead of the data packet"; and on the
    /// host's ACK "DTOG_TX bit is toggled, the endpoint is made invalid by
    /// setting STAT_TX=10 (NAK) and bit CTR_TX is set". The double-buffered
    /// and isochronous paths pick the buffer by `DTOG_TX` (Tables 255/256).
    fn do_in(&self, endpoint: u8, dst: &mut [u8], commit: bool) -> (Completion, Option<DataPid>) {
        let mut state = self.state.lock();
        if state.daddr & DADDR_EF == 0 {
            return (Completion::absent(), None);
        }
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return (Completion::absent(), None);
        };
        let reg = state.ep[n];
        let kind = Kind::of(reg);
        let dtog = reg & EP_DTOG_TX != 0;
        let live = state.dbl_live & (1 << n) != 0;
        match state.stat_tx(n) {
            STAT_VALID => {}
            STAT_NAK => return (Completion::nak(), None),
            STAT_DISABLED => return (Completion::absent(), None), // Table 261: ignored
            _ => return (Completion::stall(), None),
        }
        if kind == Kind::DoubleBulk && live && dtog == (reg & EP_DTOG_RX != 0) {
            // The buffer conflict of §40.5.3; SW_BUF of a transmission endpoint
            // is bit 14 (Table 254).
            return (Completion::nak(), None);
        }
        let slot = match kind {
            Kind::Single => 0,
            Kind::DoubleBulk | Kind::Iso => usize::from(dtog),
        };
        let (addr, count) = state.slot(n, slot);
        let len = (state.pma_u16(count) & COUNT_MASK) as usize;
        if len > dst.len() {
            // The device has more staged than the host will take in one packet:
            // a device sending past `wMaxPacketSize` (USB 2.0 §8.7.4).
            return (
                Completion {
                    status: Status::Babble,
                    len: 0,
                },
                None,
            );
        }
        let base = addr as usize;
        for (i, cell) in dst[..len].iter_mut().enumerate() {
            *cell = state.pma_byte(base + i);
        }
        // "Isochronous transfers do not support data toggle sequencing and
        // always use DATA0 PID to start any data packet" (§40.5.4).
        let pid = DataPid::from_toggle(kind != Kind::Iso && dtog);
        if commit {
            state.ep[n] |= EP_CTR_TX;
            state.ep[n] ^= EP_DTOG_TX;
            match kind {
                Kind::Single => state.set_stat_tx(n, STAT_NAK),
                Kind::Iso => {}
                Kind::DoubleBulk => state.finish_double(n, false),
            }
        }
        (Completion::ack(len as u64), Some(pid))
    }

    fn do_bus_reset(&self) {
        let mut state = self.state.lock();
        // Back to the Default state: the function answers address zero again.
        // Hardware raises the reset interrupt; firmware's handler re-arms EP0.
        state.daddr &= !DADDR_ADD;
        state.istr |= ISTR_RESET;
    }
}

impl Gated for Usbfs {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }
    fn gate_reset(&self) {
        self.do_reset();
    }
}

/// The `can-enable` input, as something a wire can drive.
///
/// Holds its device **weakly** (the device owns the pin), exactly as
/// [`gate::GatePin`] does and for the same reason.
#[derive(Debug)]
struct CanPin {
    owner: Weak<Usbfs>,
    inputs: FanIn,
}

impl WireSink for CanPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        if let Some(owner) = self.owner.upgrade() {
            owner.set_can_enable(high);
        }
    }
}

/// Apply the `EPnR` write rules to `reg` for a guest write of `value`.
fn write_epnr(reg: &mut u16, value: u16) {
    let old = *reg;
    let mut new = value & EP_RW; // plain read/write bits
    new |= old & EP_SETUP; // read-only
    // `rc_w0`: a written zero clears, a written one keeps.
    for bit in [EP_CTR_RX, EP_CTR_TX] {
        if value & bit != 0 {
            new |= old & bit;
        }
    }
    // toggle: a written one flips, a written zero keeps.
    for bit in [EP_DTOG_RX, EP_DTOG_TX] {
        let cur = old & bit;
        new |= if value & bit != 0 { cur ^ bit } else { cur };
    }
    for mask in [EP_STAT_RX, EP_STAT_TX] {
        new |= (old & mask) ^ (value & mask); // each written one toggles its bit
    }
    *reg = new;
}

// ---------------------------------------------------------------------------
// The gadget
// ---------------------------------------------------------------------------

/// This block seen from the bus. A separate object holding a [`Weak`] back to
/// the engine, so that plugging it into a [`UsbBus`] is not a reference cycle.
struct Gadget {
    core: Weak<Usbfs>,
}

impl fmt::Debug for Gadget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Gadget")
            .field("live", &(self.core.strong_count() > 0))
            .finish()
    }
}

impl Gadget {
    fn out(&self, endpoint: u8, src: &[u8], pid: Option<DataPid>) -> Completion {
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let c = core.do_out(endpoint, src, pid);
        core.refresh_irq();
        c
    }
}

impl UsbDevice for Gadget {
    fn speed(&self) -> Speed {
        Speed::Full
    }

    fn address(&self) -> DeviceAddress {
        match self.core.upgrade() {
            Some(core) => DeviceAddress((core.state.lock().daddr & DADDR_ADD) as u8),
            None => DeviceAddress::DEFAULT,
        }
    }

    fn bus_reset(&self) {
        let Some(core) = self.core.upgrade() else {
            return;
        };
        core.do_bus_reset();
        core.refresh_irq();
    }

    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status {
        let Some(core) = self.core.upgrade() else {
            return Status::NoDevice;
        };
        let status = core.do_setup(endpoint, &packet);
        core.refresh_irq();
        status
    }

    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        self.transfer_in_pid(endpoint, dst).0
    }

    fn transfer_in_pid(&self, endpoint: u8, dst: &mut [u8]) -> (Completion, Option<DataPid>) {
        let Some(core) = self.core.upgrade() else {
            return (Completion::absent(), None);
        };
        let c = core.do_in(endpoint, dst, true);
        core.refresh_irq();
        c
    }

    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        self.out(endpoint, src, None)
    }

    fn transfer_out_pid(&self, endpoint: u8, src: &[u8], pid: DataPid) -> Completion {
        self.out(endpoint, src, Some(pid))
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        match self.core.upgrade() {
            Some(core) => core.do_in(endpoint, dst, false).0,
            None => Completion::absent(),
        }
    }
}

// ---------------------------------------------------------------------------
// The register and packet-memory windows
// ---------------------------------------------------------------------------

/// The register file, as something an address space dispatches to.
struct Regs {
    core: Arc<Usbfs>,
}

impl fmt::Debug for Regs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Regs").finish_non_exhaustive()
    }
}

impl MemOps for Regs {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        let lane = (offset & 0x3) as usize;
        if lane + dst.len() > 4 {
            return Err(BusError::BadAccess);
        }
        // Gated off: the whole block reads as zero ([`gate`]).
        let value = if self.core.gate.live() {
            self.core
                .read_reg(offset & !0x3)
                .ok_or(BusError::BadAccess)?
        } else {
            0
        };
        // The register is the low half of its word; the high half reads zero.
        let bytes = u32::from(value).to_le_bytes();
        dst.copy_from_slice(&bytes[lane..lane + dst.len()]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Clearing `CTR`, toggling `STAT` and arming the pull-up all have
            // guest-visible effects with no harmless form (`ROADMAP.md` §15).
            return Err(BusError::BadAccess);
        }
        let lane = (offset & 0x3) as usize;
        if lane >= 2 {
            // The reserved high half of the word: a write is dropped.
            return Ok(());
        }
        // A word write is the register in its low half and the reserved
        // half above it, which drops — `constraints` admits words, and a
        // stack that stores the whole `uint32_t` is not writing past the
        // register.
        let src = if lane == 0 && src.len() == 4 {
            &src[..2]
        } else {
            src
        };
        if lane + src.len() > 2 {
            return Err(BusError::BadAccess);
        }
        if !self.core.gate.live() {
            return Ok(());
        }
        let mut raw = [0u8; 2];
        raw[lane..lane + src.len()].copy_from_slice(src);
        let value = u16::from_le_bytes(raw);
        let (ok, settle) = self.core.write_reg(offset & !0x3, value);
        if !ok {
            return Err(BusError::BadAccess);
        }
        if settle {
            self.core.settle();
        }
        self.core.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Half-word and word, naturally aligned: these registers are sixteen
        // bits and the manual's drivers reach them as `uint16_t` or `uint32_t`.
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U16, Width::U32)
    }
}

/// The packet-buffer memory window.
struct Pma {
    core: Arc<Usbfs>,
}

impl fmt::Debug for Pma {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Pma").finish_non_exhaustive()
    }
}

impl MemOps for Pma {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let state = self.core.state.lock();
        if !self.core.gate.live() {
            dst.fill(0);
            return Ok(());
        }
        // The CPU sees the share CAN holds the way the controller does: RM0091
        // is silent on it, and reading zero is the answer that cannot leak a
        // stale USB buffer as if it were live.
        for (i, b) in dst.iter_mut().enumerate() {
            *b = state.pma_byte(offset as usize + i);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A debug write to a buffer the controller is about to transmit
            // would put bytes on the wire the guest never wrote.
            return Err(BusError::BadAccess);
        }
        let mut state = self.core.state.lock();
        if !self.core.gate.live() {
            return Ok(());
        }
        for (i, b) in src.iter().enumerate() {
            if let Some(slot) = state.pma_slot(offset as usize + i) {
                *slot = *b;
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The packet memory should be accessed only by byte (8-bit) or
        // half-word (16-bit) accesses. Word (32-bit) accesses are not allowed"
        // (RM0434 §40.6.2, RM0091 §30.6.2). What a word access does on silicon
        // is not stated; refusing it is the answer that tells the firmware.
        AccessConstraints::word(Width::U16, Endian::Little).with_widths(Width::U8, Width::U16)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 USB full-speed device peripheral as a machine object.
#[derive(Debug)]
pub struct UsbfsPeripheral {
    core: Arc<Usbfs>,
    regs: RegionRef,
    pma: RegionRef,
}

/// The pin names a machine description wires.
pub mod pin {
    /// The low-priority USB interrupt (`USB_LP`). Level, the OR of the
    /// enabled `ISTR` flags under `CNTR`'s mask.
    pub const IRQ: &str = "irq";
    /// Input: the F0x2's `RCC_APB1ENR.CANEN`. High hands the top 256 bytes of
    /// packet memory to CAN (RM0091 Table 121, note 2). Unwired is low — the
    /// STM32WB, which has no CAN, and every board written before the pin.
    pub const CAN_ENABLE: &str = "can-enable";
    /// The high-priority USB interrupt (`USB_HP`). Level, a pending
    /// `CTR_RX`/`CTR_TX` on an isochronous or double-buffered bulk endpoint,
    /// whatever `CNTR.CTRM` says. The STM32WB wires it to NVIC 19 (`USB_HP`)
    /// and `irq` to 20 (`USB_LP`), RM0434 Table 72. The F0x2 has one vector
    /// for the block — position 31, "USB global interrupt (combined with EXTI
    /// line 18)", RM0091 Table 36 — so an F0x2 board wires `irq` to 31 and
    /// leaves this pin unwired; RM0091 does not say whether the
    /// endpoint-dedicated condition reaches that vector when `CTRM` is clear,
    /// and this leaves it not reaching it.
    pub const IRQ_HP: &str = "irq_hp";
}

impl UsbfsPeripheral {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this device plugs into. Required.
    /// * `port` — which port of `bus` it occupies. Defaults to 0.
    /// * `port-type` — what the cable is plugged into, for `BCDR`'s battery
    ///   charging detection: `sdp` (a standard host port, the default), `cdp`,
    ///   `dcp`, `ps2` or `none`. See [`PortType`].
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property, and
    /// [`Error::Config`] for a `port-type` that is not one of those.
    pub fn new(props: &Props) -> Result<UsbfsPeripheral> {
        let mut r = props.reader();
        let bus_name = String::from(r.require_str("bus")?);
        let port = r.or_range("port", 0u64, 0..=u64::from(u8::MAX))?;
        let port_type_name = r.or_str("port-type", "sdp")?;
        r.finish()?;
        let Some(port_type) = PortType::from_name(port_type_name) else {
            return Err(Error::Config {
                at: String::from("port-type"),
                message: format!(
                    "`port-type` is {port_type_name:?}; the BC 1.2 port kinds this model \
                     detects are {:?}",
                    PortType::NAMES
                ),
            });
        };
        let bus = buses::attach(props, &bus_name, (port as u8).saturating_add(1))?;
        Ok(UsbfsPeripheral::with_bus_port_type(
            bus, port as u8, port_type,
        ))
    }

    /// A peripheral on a bus the caller already holds, on port zero of it.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>) -> UsbfsPeripheral {
        UsbfsPeripheral::with_bus_at(bus, 0)
    }

    /// The same, on a named port.
    #[must_use]
    pub fn with_bus_at(bus: Arc<UsbBus>, port: u8) -> UsbfsPeripheral {
        UsbfsPeripheral::with_bus_port_type(bus, port, PortType::Sdp)
    }

    /// The same, with the cable plugged into `port_type`.
    #[must_use]
    pub fn with_bus_port_type(bus: Arc<UsbBus>, port: u8, port_type: PortType) -> UsbfsPeripheral {
        let core = Usbfs::with_port_type(bus, port, port_type);
        let regs = Arc::new(Region::io(
            "usbfs",
            REGISTER_BYTES,
            Arc::new(Regs {
                core: Arc::clone(&core),
            }) as Arc<dyn MemOps>,
        ));
        let pma = Arc::new(Region::io(
            "usbfs-pma",
            PMA_BYTES,
            Arc::new(Pma {
                core: Arc::clone(&core),
            }) as Arc<dyn MemOps>,
        ));
        UsbfsPeripheral { core, regs, pma }
    }

    /// The engine underneath.
    #[must_use]
    pub fn core(&self) -> &Arc<Usbfs> {
        &self.core
    }
}

impl Device for UsbfsPeripheral {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Match the fabric to the registers now, so a device that comes up with
        // its pull-up already attached (or a snapshot restored to that) is on
        // the bus before the first transaction.
        self.core.settle();
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        self.core.do_reset();
        self.core.settle();
        self.core.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.core.state.lock();
        for ep in state.ep {
            w.write_u16(ep)?;
        }
        w.write_u16(state.cntr)?;
        w.write_u16(state.istr)?;
        w.write_u16(state.daddr)?;
        w.write_u16(state.btable)?;
        w.write_u16(state.lpmcsr)?;
        w.write_u16(state.bcdr)?;
        w.write_bytes(&state.pma)?;
        w.write_u8(state.dbl_live)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset();
        for ep in &mut state.ep {
            *ep = r.read_u16()?;
        }
        state.cntr = r.read_u16()?;
        state.istr = r.read_u16()?;
        state.daddr = r.read_u16()?;
        state.btable = r.read_u16()?;
        state.lpmcsr = r.read_u16()?;
        state.bcdr = r.read_u16()?;
        let pma = r.read_bytes()?;
        if pma.len() != PMA_BYTES as usize {
            return Err(Error::State(format!(
                "snapshot has a {}-byte USB packet memory, this block has {}",
                pma.len(),
                PMA_BYTES
            )));
        }
        state.pma.copy_from_slice(pma);
        state.dbl_live = r.read_u8()?;
        self.core.set_state(state);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "pma" | "sram" => Some(Arc::clone(&self.pma)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The clock gate's `enable` and `reset`, and the F0x2's CAN share.
        if let Some(pin) = gate::sink(&self.core, port, sources) {
            return Some(pin);
        }
        if port != pin::CAN_ENABLE {
            return None;
        }
        let pin = Arc::new(CanPin {
            owner: Arc::downgrade(&self.core),
            inputs: FanIn::new(sources),
        });
        self.core.can_pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            pin::IRQ => self.core.connect_irq(source),
            pin::IRQ_HP => self.core.connect_irq_hp(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: format!(
                        "a USB device peripheral drives `{}` and `{}` and nothing else",
                        pin::IRQ,
                        pin::IRQ_HP
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.core.refresh_irq();
    }
}

impl Instance for UsbfsPeripheral {}

/// The `st.usbfs` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 USB full-speed device peripheral (the F0x2 and STM32WB USB block)",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the USB bus this device plugs into",
        },
        PropertySpec {
            name: "port",
            kind: ValueKind::Uint,
            required: false,
            summary: "which port of the bus it occupies (default 0)",
        },
        PropertySpec {
            name: "port-type",
            kind: ValueKind::Str,
            required: false,
            summary: "what the cable is plugged into, for battery charging detection: \
                      sdp (default), cdp, dcp, ps2 or none",
        },
    ],
    construct: |props| Ok(Box::new(UsbfsPeripheral::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| {
        Ok(Arc::new(UsbfsPeripheral::new(props)?))
    })
}

/// What the validator should know about `st.usbfs`.
#[must_use]
pub fn schema() -> ClassSchema {
    let schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("port", ValueKind::Uint).range(0, u64::from(u8::MAX)))
        .prop(PropSchema::new("port-type", ValueKind::Str).values(&PortType::NAMES))
        .port(pin::IRQ, PortDir::Out)
        .port(pin::CAN_ENABLE, PortDir::In)
        .port(pin::IRQ_HP, PortDir::Out)
        .region("")
        .region("regs")
        .region("pma")
        .region("sram");
    // `RCC`'s `enable`/`reset`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests;
