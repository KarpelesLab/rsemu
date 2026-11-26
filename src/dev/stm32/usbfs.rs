//! The STM32 **USB full-speed device** peripheral — the `USB_FS` block of the
//! F0x2/F1/F3/L0 families, not the dwc2 OTG core.
//!
//! One class, `st.usbfs`. This is the device-only controller ST has carried
//! since the F1: eight endpoint registers, a packet-buffer memory the endpoints
//! draw their buffers from, and a buffer-descriptor table that says where each
//! buffer is. It is a **peripheral, never a host** — there is no `HPRT`, no
//! channel list, nothing that drives a bus — so it plugs into
//! [`UsbBus`] as a [`UsbDevice`]
//! and a modelled host enumerates it, exactly the way the [`dwc2`](super::super::usb::dwc2)
//! gadget does. That is the whole reason it belongs here rather than being
//! faked: the firmware under test is a USB device stack, and the thing it talks
//! to is the register file below.
//!
//! # The register file
//!
//! Register window (RM0091 §30.6, offsets from the block base), each a sixteen-
//! bit register in the low half of a thirty-two-bit word:
//!
//! | Offset | Register | |
//! | --- | --- | --- |
//! | `0x00`…`0x1c` | `EP0R`…`EP7R` | the eight endpoint registers |
//! | `0x40` | `CNTR` | control: the interrupt-enable mask, and `PDWN`/`FRES` power |
//! | `0x44` | `ISTR` | interrupt status |
//! | `0x48` | `FNR` | frame number (read-only; this model answers zero) |
//! | `0x4c` | `DADDR` | the device address and the function-enable bit |
//! | `0x50` | `BTABLE` | where the buffer-descriptor table sits in packet memory |
//! | `0x54` | `LPMCSR` | the LPM control/status (storage) |
//! | `0x58` | `BCDR` | battery-charging detection; `DPPU` is the pull-up |
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
//!   **toggle**: a written one flips the bit, a written zero leaves it. A
//!   driver that wants `STAT_RX = VALID` writes the exclusive-or of where it is
//!   and where it wants to be, and the hardware does the rest.
//! * `SETUP` (11) is **read-only** — hardware sets it on a `SETUP` packet.
//! * `EP_TYPE` (10:9), `EP_KIND` (8) and `EA` (3:0) are plain read/write.
//!
//! `ISTR`'s flag bits are `rc_w0` as a group (a write ANDs), while `CTR`, `DIR`
//! and `EP_ID` are read-only and **computed**: `CTR` is the OR of every
//! endpoint's pending `CTR_RX`/`CTR_TX`, and `DIR`/`EP_ID` name the endpoint the
//! highest-priority one of those is on. That is how a driver's interrupt
//! handler finds the endpoint to service without scanning all eight.
//!
//! # The packet memory
//!
//! A separate region — the block's SRAM — mapped at its own base. On this
//! family it is **linear**: a flat run of bytes (1 KiB here), addressed the way
//! any other memory is, which is the "1 × 16 bit / word" organisation of the
//! F0x2 rather than the F1's doubled, gap-ridden one. The buffer-descriptor
//! table lives at `BTABLE` and holds, per endpoint register, four sixteen-bit
//! words — `ADDR_TX`, `COUNT_TX`, `ADDR_RX`, `COUNT_RX` — and the data buffers
//! sit at those `ADDR_*` byte offsets. `COUNT_RX`'s high bits are the allocated
//! block size and are left alone; hardware writes only the low ten bits, the
//! received byte count.
//!
//! # On the bus
//!
//! A [`UsbDevice`]. The host's `SETUP`, `IN` and
//! `OUT` transactions are matched to an endpoint register by its `EA` field —
//! not by the register's position — exactly as the token's address field is
//! matched on silicon, and the matched register's `STAT` decides the handshake:
//! `VALID` proceeds, `NAK` asks the host to retry, `STALL` refuses, `DISABLED`
//! is no endpoint at all. A completed transaction sets the direction's `CTR`,
//! forces its `STAT` back to `NAK` (so firmware must re-arm — the hardware's
//! anti-overrun), and raises the interrupt. The device signals **full speed**
//! and answers at `DADDR`'s address.
//!
//! # The clock, the interrupt
//!
//! `enable` and `reset` arrive from the reset and clock controller, as
//! [`gate`] describes; an unwired `enable` leaves the block
//! clocked, so a board written before the gate existed does not change. One
//! `irq` output carries the low-priority USB interrupt — level, the OR of the
//! enabled `ISTR` flags under `CNTR`'s mask — and the board wires it to its
//! core's USB vector.
//!
//! # Sources
//!
//! ST **RM0091** rev 9 §30 "Universal serial bus full-speed device interface
//! (USB)", for the STM32F0x2 line. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{
    Completion, DeviceAddress, SetupPacket, Speed, Status, UsbBus, UsbDevice, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.usbfs";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many endpoint registers the block has. Fixed on this peripheral.
const ENDPOINTS: usize = 8;

/// Register-window size, through `BCDR`.
pub const REGISTER_BYTES: u64 = 0x60;

/// Packet-buffer memory size: 1 KiB on the F0x2.
pub const PMA_BYTES: u64 = 0x400;

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

// --- DADDR / BCDR ---
const DADDR_ADD: u16 = 0x007f;
const BCDR_DPPU: u16 = 1 << 15;

/// Low ten bits of a `COUNT_*` word: the byte count. The high bits are the RX
/// buffer's allocated block size and are the application's, not ours.
const COUNT_MASK: u16 = 0x03ff;

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

    fn pma_u16(&self, off: u16) -> u16 {
        let o = off as usize;
        self.pma
            .get(o..o + 2)
            .map_or(0, |s| u16::from_le_bytes([s[0], s[1]]))
    }
    fn set_pma_u16(&mut self, off: u16, v: u16) {
        let o = off as usize;
        if let Some(s) = self.pma.get_mut(o..o + 2) {
            s.copy_from_slice(&v.to_le_bytes());
        }
    }
    fn pma_write(&mut self, addr: u16, data: &[u8]) {
        for (i, b) in data.iter().enumerate() {
            if let Some(slot) = self.pma.get_mut(addr as usize + i) {
                *slot = *b;
            }
        }
    }

    /// The buffer-descriptor-table byte offset of endpoint register `n`.
    fn btable_entry(&self, n: usize) -> u16 {
        self.btable.wrapping_add((n as u16) * 8)
    }
    /// `(ADDR_TX, &COUNT_TX, ADDR_RX, &COUNT_RX)` for endpoint register `n`.
    fn descr(&self, n: usize) -> (u16, u16, u16, u16) {
        let e = self.btable_entry(n);
        (
            self.pma_u16(e),
            e.wrapping_add(2),
            self.pma_u16(e.wrapping_add(4)),
            e.wrapping_add(6),
        )
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
    fn istr_value(&self) -> u16 {
        for n in 0..ENDPOINTS as u16 {
            let ep = self.ep[n as usize];
            if ep & EP_CTR_RX != 0 {
                return self.istr | ISTR_CTR | ISTR_DIR | n;
            }
            if ep & EP_CTR_TX != 0 {
                return self.istr | ISTR_CTR | n;
            }
        }
        self.istr & !ISTR_CTR
    }

    /// Whether the interrupt line stands: an enabled `ISTR` flag under `CNTR`'s
    /// mask. `CTR` rides in through [`istr_value`](State::istr_value).
    fn irq_asserted(&self) -> bool {
        self.istr_value() & self.cntr & CNTR_IRQ_MASK != 0
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
    /// The `irq` output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The bus this device plugs into when its pull-up is attached.
    bus: Arc<UsbBus>,
    /// Which port of [`bus`](Usbfs::bus) it plugs into.
    port: u8,
    /// `0` while off the bus, `1` while attached. A latch, not guest state.
    attached: AtomicU32,
    /// This block seen from the bus. Holds a [`Weak`] back here.
    gadget: Mutex<Option<Arc<Gadget>>>,
}

impl fmt::Debug for Usbfs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Usbfs")
            .field("port", &self.port)
            .field("attached", &(self.attached.load(Ordering::Relaxed) != 0))
            .finish_non_exhaustive()
    }
}

impl Usbfs {
    /// Build the engine on `bus`, its device half plugging into `port`.
    #[must_use]
    pub fn new(bus: Arc<UsbBus>, port: u8) -> Arc<Usbfs> {
        let me = Arc::new(Usbfs {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            gate: ClockGate::new(),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            bus,
            port,
            attached: AtomicU32::new(0),
            gadget: Mutex::with_rank(LockRank::WIRE, None),
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

    /// `ISTR` as the guest would read it.
    #[must_use]
    pub fn interrupt_status(&self) -> u16 {
        self.state.lock().istr_value()
    }

    /// Whether the device is electrically on the bus: clocked, powered, out of
    /// reset, and its `DPPU` pull-up attached.
    fn connected(state: &State, gate: &ClockGate) -> bool {
        gate.live()
            && state.bcdr & BCDR_DPPU != 0
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

    /// Republish the interrupt line. Called with **no lock held**.
    pub fn refresh_irq(&self) {
        let asserted = self.state.lock().irq_asserted();
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(asserted));
        }
    }

    fn do_reset(&self) {
        *self.state.lock() = State::reset();
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
            OFF_BCDR => state.bcdr,
            _ => return None,
        })
    }

    /// Write `value` to the register at word offset `word`. Returns whether the
    /// decode succeeded and whether the bus attachment may have changed.
    fn write_reg(&self, word: u64, value: u16) -> (bool, bool) {
        let mut state = self.state.lock();
        match word {
            o if o < 0x20 => write_epnr(&mut state.ep[(o / 4) as usize], value),
            OFF_CNTR => state.cntr = value,
            OFF_ISTR => state.istr &= value & ISTR_FLAG_MASK,
            OFF_DADDR => state.daddr = value,
            OFF_BTABLE => state.btable = value,
            OFF_LPMCSR => state.lpmcsr = value,
            OFF_BCDR => state.bcdr = value,
            _ => return (false, false),
        }
        // `CNTR.PDWN`/`FRES` and `BCDR.DPPU` decide whether the device is on the
        // bus; a write to either may have flipped that.
        let settle = matches!(word, OFF_CNTR | OFF_BCDR);
        (true, settle)
    }

    // -- the transactions the gadget runs -------------------------------

    fn do_setup(&self, endpoint: u8, packet: &SetupPacket) -> Status {
        let mut state = self.state.lock();
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return Status::Stall;
        };
        if state.stat_rx(n) == STAT_DISABLED {
            // No reception armed on a control endpoint: nothing can take it.
            return Status::Stall;
        }
        let (_, _, addr_rx, count_rx) = state.descr(n);
        let bytes = packet.encode();
        state.pma_write(addr_rx, &bytes);
        let cr = state.pma_u16(count_rx);
        state.set_pma_u16(count_rx, (cr & !COUNT_MASK) | SetupPacket::SIZE as u16);
        state.ep[n] |= EP_CTR_RX | EP_SETUP;
        // §30.5.3: a `SETUP` forces both directions to `NAK`, so a stale `VALID`
        // from the previous transfer does not answer the new one's data stage
        // before firmware has prepared it.
        state.set_stat_rx(n, STAT_NAK);
        state.set_stat_tx(n, STAT_NAK);
        Status::Ack
    }

    fn do_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        let mut state = self.state.lock();
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return Completion::stall();
        };
        match state.stat_rx(n) {
            STAT_VALID => {}
            STAT_NAK => return Completion::nak(),
            _ => return Completion::stall(), // DISABLED or STALL
        }
        let (_, _, addr_rx, count_rx) = state.descr(n);
        state.pma_write(addr_rx, src);
        let cr = state.pma_u16(count_rx);
        state.set_pma_u16(
            count_rx,
            (cr & !COUNT_MASK) | (src.len() as u16 & COUNT_MASK),
        );
        state.ep[n] = (state.ep[n] | EP_CTR_RX) & !EP_SETUP;
        state.set_stat_rx(n, STAT_NAK);
        Completion::ack(src.len() as u64)
    }

    fn do_in(&self, endpoint: u8, dst: &mut [u8], commit: bool) -> Completion {
        let mut state = self.state.lock();
        let Some(n) = state.match_ea(u16::from(endpoint)) else {
            return Completion::stall();
        };
        match state.stat_tx(n) {
            STAT_VALID => {}
            STAT_NAK => return Completion::nak(),
            _ => return Completion::stall(), // DISABLED or STALL
        }
        let (addr_tx, count_tx, _, _) = state.descr(n);
        let len = (state.pma_u16(count_tx) & COUNT_MASK) as usize;
        if len > dst.len() {
            // The device has more staged than the host will take in one packet:
            // a device sending past `wMaxPacketSize` (USB 2.0 §8.7.4).
            return Completion {
                status: Status::Babble,
                len: 0,
            };
        }
        let base = addr_tx as usize;
        for (i, slot) in dst[..len].iter_mut().enumerate() {
            *slot = state.pma.get(base + i).copied().unwrap_or(0);
        }
        if commit {
            state.ep[n] |= EP_CTR_TX;
            state.set_stat_tx(n, STAT_NAK);
        }
        Completion::ack(len as u64)
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
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let c = core.do_in(endpoint, dst, true);
        core.refresh_irq();
        c
    }

    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        let Some(core) = self.core.upgrade() else {
            return Completion::absent();
        };
        let c = core.do_out(endpoint, src);
        core.refresh_irq();
        c
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        match self.core.upgrade() {
            Some(core) => core.do_in(endpoint, dst, false),
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
        for (i, b) in dst.iter_mut().enumerate() {
            *b = state.pma.get(offset as usize + i).copied().unwrap_or(0);
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
            if let Some(slot) = state.pma.get_mut(offset as usize + i) {
                *slot = *b;
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U8, Width::U32)
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
    /// The low-priority USB interrupt. Level, the OR of the enabled `ISTR`
    /// flags under `CNTR`'s mask.
    pub const IRQ: &str = "irq";
}

impl UsbfsPeripheral {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this device plugs into. Required.
    /// * `port` — which port of `bus` it occupies. Defaults to 0.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property.
    pub fn new(props: &Props) -> Result<UsbfsPeripheral> {
        let mut r = props.reader();
        let bus_name = String::from(r.require_str("bus")?);
        let port = r.or_range("port", 0u64, 0..=u64::from(u8::MAX))?;
        r.finish()?;
        let bus = buses::attach(props, &bus_name, (port as u8).saturating_add(1))?;
        Ok(UsbfsPeripheral::with_bus_at(bus, port as u8))
    }

    /// A peripheral on a bus the caller already holds, on port zero of it.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>) -> UsbfsPeripheral {
        UsbfsPeripheral::with_bus_at(bus, 0)
    }

    /// The same, on a named port.
    #[must_use]
    pub fn with_bus_at(bus: Arc<UsbBus>, port: u8) -> UsbfsPeripheral {
        let core = Usbfs::new(bus, port);
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
        w.write_bytes(&state.pma)
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
        *self.core.state.lock() = state;
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
        // The clock gate's `enable` and `reset`, and nothing else inward.
        gate::sink(&self.core, port, sources)
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: format!(
                    "a USB device peripheral drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.core.connect_irq(source);
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
    summary: "STM32 USB full-speed device peripheral (the F0x2/F1/F3/L0 USB_FS block)",
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
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
        .region("pma")
        .region("sram");
    // `RCC`'s `enable`/`reset`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests;
