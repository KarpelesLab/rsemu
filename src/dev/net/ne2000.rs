//! An NE2000 Ethernet card: a National Semiconductor DP8390 NIC plus the
//! Novell card's data and reset ports.
//!
//! # Sources
//!
//! Every register, bit and procedure below comes from one of these. No emulator
//! source was opened (`ROADMAP.md` §1) — in particular not QEMU's, Bochs's or
//! VirtualBox's `ne2000`, all of which are GPL.
//!
//! * **DP8390D/NS32490D *NIC Network Interface Controller*** data sheet
//!   (National Semiconductor). Cited below as *DP8390D*, by the section names
//!   the data sheet uses: "Register Descriptions" for the bit fields,
//!   "Packet Reception" for the buffer ring and its four-byte header,
//!   "Packet Transmission" for TPSR/TBCR/TXP, "Remote DMA" for RSAR/RBCR and
//!   the Send Packet command, "Multicast Address Filtering" for the hash
//!   table, and "Buffer Ring Overflow" for the recovery procedure a driver has
//!   to run after `ISR.OVW`.
//! * **IEEE 802.3** §4.2.3.3 and §4.2.7.1 for the minimum and maximum frame,
//!   and §3.2.9 for the CRC the multicast hash is computed with.
//! * The **Novell NE2000** card's own two additions to the DP8390 — the remote
//!   DMA data window at `base+0x10` and the reset strap at `base+0x18` — plus
//!   the 32-byte address PROM with each byte doubled and `0x57 0x57` ("WW") at
//!   PROM offsets 14 and 15, which is how a driver identifies the card. These
//!   are card, not chip, and are documented in the card's own programming
//!   notes rather than in the data sheet.
//!
//! # Why an NE2000 and not an e1000
//!
//! It needs no bus master. A DP8390's packet buffer is 16 KiB of RAM *on the
//! card*, reached through a data port with the chip's own "remote DMA" engine —
//! so the whole device is 32 I/O ports and one interrupt pin, and it can be
//! driven end to end by a Z80 with `IN` and `OUT`. An e1000 is descriptor rings
//! in guest memory behind PCI, which would make this model depend on another
//! agent's in-flight `bus::pci` work and would make the smallest board that can
//! test it a PC. Every OS from DOS onward has an NE2000 driver, and the part is
//! fully described by a public data sheet.
//!
//! # The register file
//!
//! ```text
//!   base+0x00 .. 0x0f   the DP8390's sixteen registers, in four banked pages
//!   base+0x10 .. 0x17   the card's remote DMA data window (all eight mirror)
//!   base+0x18 .. 0x1f   the card's reset strap
//! ```
//!
//! The page is chosen by `CR` bits 6-7, and the same offset means different
//! things on read and on write — offset 7 is `ISR` both ways but offset 4 is
//! `TSR` read and `TPSR` written (*DP8390D*, "Register Descriptions").
//!
//! # Time, and why this device is lazy
//!
//! A frame arrives when the outside says, not when the guest's clock says, so
//! delivery has to be pinned to a **virtual** tick or the machine stops being
//! reproducible ([`link`](super::link) argues the case at length). This device
//! is therefore [lazy](crate::core::device::Device::is_lazy): it holds its own
//! tick in its own clock domain, publishes the tick of the next queued arrival
//! through [`Device::next_event_tick`], and the scheduler stops the world at
//! exactly that tick and calls [`Device::advance_to`]. A guest register access
//! catches the device up first, so the ring pointer a driver reads is the one
//! that belongs to the cycle it read it on.
//!
//! Nothing here reads a wall clock, sleeps, or spawns anything.
//!
//! # Deliberate simplifications, all guest-visible
//!
//! * **No FCS.** No backend on this seam produces a frame check sequence, so
//!   none is stored, none is counted, and `RSR.CRC` is never set. A driver that
//!   subtracts the four header bytes from the header's count gets exactly the
//!   frame that was on the wire.
//! * **The transmitter is instantaneous.** `CR.TXP` is clear and `ISR.PTX` is
//!   set by the time the write that started the transmission returns. Nothing
//!   models the 51.2 µs slot time, so `NCR`, `TSR.COL` and `TSR.OWC` are always
//!   zero: there are no collisions on a wire with one station on it.
//! * **`FIFO` (page 0, offset 6) reads as zero**, and the tally counters
//!   `CNTR0`-`CNTR2` count only what this model can see — frames dropped for
//!   want of ring space are not, because an overflow stops the receiver.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

use super::link::{MAX_FRAME_LEN, MIN_FRAME_LEN, MacAddr, NetLink, ports};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "net.ne2000";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much I/O space the card answers: sixteen chip registers, the data
/// window, and the reset strap.
pub const REGISTER_WINDOW_LEN: u64 = 0x20;

/// The network port a machine file gets if it names none.
const DEFAULT_LINK: &str = "net0";

/// The address the card's 16 KiB of buffer RAM starts at in the DP8390's own
/// 16-bit address space. Below it the remote DMA engine sees the address PROM.
const MEM_START: u16 = 0x4000;

/// One past the last byte of buffer RAM.
const MEM_END: u16 = 0x8000;

/// How much buffer RAM an NE2000 carries.
pub const MEM_LEN: usize = (MEM_END - MEM_START) as usize;

/// The first ring page a driver may use: `MEM_START >> 8`.
const PAGE_FIRST: u8 = (MEM_START >> 8) as u8;

/// One past the last ring page: `MEM_END >> 8`, which is `0x00` in eight bits,
/// so the stop page is held as the `u16` it is.
const PAGE_LAST: u16 = MEM_END >> 8;

/// How many bytes of address PROM the card presents. Sixteen values, each
/// doubled, so a driver reading 32 bytes in byte mode takes every other one.
const PROM_LEN: usize = 32;

/// The card signature at PROM values 14 and 15 — `'W' 'W'`, which is what a
/// driver looks for to tell an NE2000 from an NE1000.
const PROM_SIGNATURE: u8 = 0x57;

// -- CR, offset 0, every page (DP8390D, "Register Descriptions") -------------

/// Stop: abort everything and idle the chip.
const CR_STP: u8 = 0x01;
/// Start: leave the reset state.
const CR_STA: u8 = 0x02;
/// Transmit packet: send `TBCR` bytes from `TPSR`.
const CR_TXP: u8 = 0x04;
/// The remote DMA command, bits 3-5.
const CR_RD_MASK: u8 = 0x38;
/// The register page, bits 6-7.
const CR_PS_MASK: u8 = 0xc0;

/// `RD` = 001: remote read — the guest reads card memory through the data port.
const RD_READ: u8 = 1;
/// `RD` = 010: remote write — the guest writes card memory through it.
const RD_WRITE: u8 = 2;
/// `RD` = 011: send packet — a remote read of the packet at `BNRY`, with the
/// byte count taken from that packet's own header.
const RD_SEND: u8 = 3;

// -- ISR / IMR, offset 7 and 15 of page 0 ------------------------------------

/// A packet was received without error.
const ISR_PRX: u8 = 0x01;
/// A packet was transmitted without error.
const ISR_PTX: u8 = 0x02;
/// A packet was received with an error.
const ISR_RXE: u8 = 0x04;
/// A transmission failed.
const ISR_TXE: u8 = 0x08;
/// The receive ring overflowed.
const ISR_OVW: u8 = 0x10;
/// A tally counter has reached 0x80.
const ISR_CNT: u8 = 0x20;
/// Remote DMA complete: `RBCR` reached zero.
const ISR_RDC: u8 = 0x40;
/// Reset status: set while the chip is stopped, cleared when it is started.
/// Not maskable, which is why `IMR` is only seven bits.
const ISR_RST: u8 = 0x80;
/// The seven bits `IMR` can enable.
const IMR_MASK: u8 = 0x7f;

// -- RCR, page 0 offset 12 ---------------------------------------------------
//
// `RCR.SEP` (bit 0, "save errored packets") is deliberately absent: this model
// produces no errored packets to save, because the seam carries no FCS.

/// Accept runt packets — anything shorter than the 802.3 minimum.
const RCR_AR: u8 = 0x02;
/// Accept broadcast.
const RCR_AB: u8 = 0x04;
/// Accept multicast that passes the hash table.
const RCR_AM: u8 = 0x08;
/// Promiscuous physical: accept every unicast address.
const RCR_PRO: u8 = 0x10;
/// Monitor mode: count packets, buffer none.
const RCR_MON: u8 = 0x20;

// -- TCR, page 0 offset 13 ---------------------------------------------------

/// The loopback mode field, bits 1-2. Non-zero means the transmitter is wired
/// back to the receiver and nothing reaches the wire.
const TCR_LB_MASK: u8 = 0x06;

// -- DCR, page 0 offset 14 ---------------------------------------------------

/// Word transfer select: the data port moves sixteen bits at a time.
const DCR_WTS: u8 = 0x01;
/// Byte order select: the high byte comes first when `WTS` is set.
const DCR_BOS: u8 = 0x02;

// -- TSR, page 0 offset 4 (read) ---------------------------------------------

/// Packet transmitted.
const TSR_PTX: u8 = 0x01;
/// Transmit aborted.
const TSR_ABT: u8 = 0x08;
/// Carrier sense lost.
const TSR_CRS: u8 = 0x10;

// -- RSR, page 0 offset 12 (read) --------------------------------------------

/// Packet received intact.
const RSR_PRX: u8 = 0x01;
/// The packet was accepted because of its group address, not its physical one.
const RSR_PHY: u8 = 0x20;

/// Everything the guest can see or change.
#[derive(Debug)]
struct State {
    /// The device's position in its own clock domain.
    tick: u64,

    cr: u8,
    isr: u8,
    imr: u8,
    dcr: u8,
    rcr: u8,
    tcr: u8,
    tsr: u8,
    rsr: u8,

    /// The receive ring, in 256-byte pages.
    pstart: u8,
    pstop: u8,
    bnry: u8,
    curr: u8,

    /// The transmit buffer's page, and how many bytes to send from it.
    tpsr: u8,
    tbcr: u16,

    /// Remote DMA: where the guest asked to start, and how much is left.
    rsar: u16,
    rbcr: u16,
    /// The remote DMA address as it advances. Guest-visible as `CRDA0`/`CRDA1`.
    crda: u16,
    /// What is left of `rbcr` for the transfer in progress.
    remaining: u16,
    /// The local DMA address, guest-visible as `CLDA0`/`CLDA1`. This model
    /// moves a whole frame at once, so it always points at the last one.
    clda: u16,

    /// The station address the driver programmed, page 1 offsets 1-6.
    par: [u8; 6],
    /// The multicast hash table, page 1 offsets 8-15.
    mar: [u8; 8],

    /// The three tally counters, page 0 offsets 13-15.
    cntr: [u8; 3],

    /// Set when a frame arrived with no room in the ring. The receiver stays
    /// off until the driver runs the recovery of *DP8390D*, "Buffer Ring
    /// Overflow" — which begins with `STP`.
    overflow: bool,

    /// The card's 16 KiB of buffer RAM.
    mem: Vec<u8>,
}

impl State {
    /// The state the chip powers up in and returns to on a reset.
    ///
    /// *DP8390D*, "Register Descriptions": reset leaves `CR` at `0x21` — stopped,
    /// page 0, remote DMA aborted — and sets `ISR.RST`. Everything else is
    /// zero, which is why a driver has to program `DCR`, `RCR`, `TCR`, the ring
    /// and the station address before it starts the chip.
    fn new() -> State {
        State {
            tick: 0,
            cr: CR_STP | (4 << 3),
            isr: ISR_RST,
            imr: 0,
            dcr: 0,
            rcr: 0,
            tcr: 0,
            tsr: 0,
            rsr: 0,
            pstart: 0,
            pstop: 0,
            bnry: 0,
            curr: 0,
            tpsr: 0,
            tbcr: 0,
            rsar: 0,
            rbcr: 0,
            crda: 0,
            remaining: 0,
            clda: 0,
            par: [0; 6],
            mar: [0; 8],
            cntr: [0; 3],
            overflow: false,
            mem: vec![0; MEM_LEN],
        }
    }

    /// Which register page `CR` selects.
    fn page(&self) -> u8 {
        (self.cr & CR_PS_MASK) >> 6
    }

    /// The remote DMA command `CR` selects. 4-7 all mean abort/complete.
    fn dma_command(&self) -> u8 {
        (self.cr & CR_RD_MASK) >> 3
    }

    /// Whether the interrupt pin should be asserted.
    ///
    /// `ISR.RST` is not maskable and does not drive the pin: `IMR` is seven
    /// bits wide (*DP8390D*, "Register Descriptions").
    fn irq(&self) -> bool {
        self.isr & self.imr & IMR_MASK != 0
    }

    /// Whether the ring pointers name a usable ring.
    ///
    /// A driver that has not programmed them yet — or has programmed nonsense —
    /// gets a receiver that drops rather than one that writes outside its own
    /// memory.
    fn ring_ok(&self) -> bool {
        let start = u16::from(self.pstart);
        let stop = u16::from(self.pstop);
        let curr = u16::from(self.curr);
        let bnry = u16::from(self.bnry);
        start >= u16::from(PAGE_FIRST)
            && stop <= PAGE_LAST
            && start + 1 < stop
            && (start..stop).contains(&curr)
            && (start..stop).contains(&bnry)
    }

    /// Whether a frame arriving now would be taken in.
    fn receiver_on(&self) -> bool {
        self.cr & CR_STA != 0
            && self.cr & CR_STP == 0
            && self.tcr & TCR_LB_MASK == 0
            && !self.overflow
            && self.ring_ok()
    }

    /// How many pages the NIC may still fill before it would run into the
    /// boundary pointer.
    ///
    /// One page is always left unused so that `CURR == BNRY` keeps meaning
    /// "empty" — the only way the two pointers can be told apart.
    fn free_pages(&self) -> u16 {
        let start = u16::from(self.pstart);
        let stop = u16::from(self.pstop);
        let ring = stop - start;
        let curr = u16::from(self.curr);
        let bnry = u16::from(self.bnry);
        let used = (curr + ring - bnry) % ring;
        ring - used - 1
    }

    /// One byte of the DP8390's address space, as remote DMA sees it.
    fn mem_read(&self, prom: &[u8; PROM_LEN], addr: u16) -> u8 {
        if (addr as usize) < PROM_LEN {
            prom[addr as usize]
        } else if (MEM_START..MEM_END).contains(&addr) {
            self.mem[(addr - MEM_START) as usize]
        } else {
            // Nothing drives the bus there. Ones, as an unterminated bus reads.
            0xff
        }
    }

    /// One byte into buffer RAM. Anything outside it — the PROM included — is
    /// read-only and the write is swallowed.
    fn mem_write(&mut self, addr: u16, value: u8) {
        if (MEM_START..MEM_END).contains(&addr) {
            self.mem[(addr - MEM_START) as usize] = value;
        }
    }

    /// Step the remote DMA address one byte on.
    ///
    /// *DP8390D*, "Remote DMA": the address wraps from `PSTOP` back to `PSTART`,
    /// which is what lets a driver read a packet that straddles the end of the
    /// ring in one transfer.
    fn dma_step(&mut self) {
        self.crda = self.crda.wrapping_add(1);
        if self.ring_ok() && self.crda == self.pstop_addr() {
            self.crda = self.pstart_addr();
        }
    }

    /// The first byte of the ring.
    fn pstart_addr(&self) -> u16 {
        u16::from(self.pstart) << 8
    }

    /// One past the last byte of the ring.
    fn pstop_addr(&self) -> u16 {
        u16::from(self.pstop) << 8
    }

    /// Bump a tally counter, saturating at 0xff and raising `ISR.CNT` at 0x80
    /// as the data sheet's counters do.
    fn tally(&mut self, index: usize) {
        let c = &mut self.cntr[index];
        // Saturating rather than wrapping, and said so deliberately
        // (`CLAUDE.md`, arithmetic): the data sheet's counters stop at 0xff and
        // wait to be read.
        *c = c.saturating_add(1);
        if *c >= 0x80 {
            self.isr |= ISR_CNT;
        }
    }
}

/// Why the address filter turned a frame away.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Reject {
    /// Not for us, and `RCR` did not say to take it anyway.
    Filtered,
    /// Shorter than 802.3 allows and `RCR.AR` is clear.
    Runt,
    /// Monitor mode: counted, never buffered.
    Monitored,
}

/// The card, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The interrupt output, at [`LockRank::LEAF`] so the pin can be driven
    /// with nothing else held.
    out: Mutex<Option<WireSource>>,
    /// The catch-up handle the register paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// Published so [`Device::current_tick`] can answer without a lock — the
    /// scheduler asks it with its own slot lock held at [`LockRank::LEAF`].
    tick: AtomicU64,
    /// Whether the receiver would take a frame offered right now.
    ///
    /// Published beside the tick for the same reason: [`Device::next_event_tick`]
    /// is asked with the scheduler's slot lock held and may not take one. The
    /// *arrival* half of the answer is asked of the link, which publishes it
    /// lock-free too — a value cached here would be stale exactly when the
    /// outside queued something.
    rx_ready: AtomicBool,
    /// The far side of the wire.
    link: Arc<dyn NetLink>,
    /// The name the link was opened under, for `Debug` and diagnostics.
    link_name: String,
    /// The address PROM: sixteen values, each stored twice.
    prom: [u8; PROM_LEN],
    /// The address the PROM carries, which is the card's own.
    mac: MacAddr,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("link", &self.link_name);
        s.field("mac", &self.mac);
        s.field("tick", &self.tick.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s
                .field("cr", &state.cr)
                .field("isr", &state.isr)
                .field("curr", &state.curr)
                .field("bnry", &state.bnry)
                .finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Republish the two lock-free values. Called with the state lock held.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        // Only promise an event the device could act on: a stopped or overflowed
        // receiver would otherwise be woken every tick for a frame it is never
        // going to take, and the scheduler would grind. Whatever turns the
        // receiver back on republishes.
        self.rx_ready.store(state.receiver_on(), Ordering::Relaxed);
    }

    /// The tick the scheduler should next stop the world at, without taking a
    /// lock.
    fn next_event(&self) -> Option<u64> {
        if !self.rx_ready.load(Ordering::Relaxed) {
            return None;
        }
        // `Device::next_event_tick` must name a tick in the future; a frame that
        // is already due wants the very next one.
        let now = self.tick.load(Ordering::Relaxed);
        self.link
            .next_arrival()
            .map(|at| at.max(now.saturating_add(1)))
    }

    /// Drive the interrupt pin. Never called with the state lock held.
    fn drive(&self, asserted: bool) {
        let out = self.out.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// Recompute and drive the interrupt line from the current state.
    fn refresh(&self) {
        let asserted = {
            let state = self.state.lock();
            self.republish(&state);
            state.irq()
        };
        self.drive(asserted);
    }

    /// Catch up before an access, exactly as the RTC and the 8254 do (§4.2).
    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        // A refusal means catch-up is already running further up the stack; the
        // access still has to be answered from where the device stands.
        let _ = handle.sync(AccessKind::Guest);
    }

    /// Would this frame be taken in, and if not, why not?
    ///
    /// *DP8390D*, "Register Descriptions" (RCR) and "Multicast Address
    /// Filtering". Order matters: monitor mode counts everything the filter
    /// would have accepted, so the filter runs first.
    fn accepts(state: &State, par: MacAddr, frame: &[u8]) -> core::result::Result<bool, Reject> {
        if (frame.len() as u64) < MIN_FRAME_LEN && state.rcr & RCR_AR == 0 {
            return Err(Reject::Runt);
        }
        let mut dst = [0u8; 6];
        dst.copy_from_slice(&frame[..6]);
        let dst = MacAddr(dst);

        // Returns whether the frame was accepted for its *group* address, which
        // is what RSR.PHY reports.
        let group = if dst.is_broadcast() {
            if state.rcr & RCR_AB == 0 {
                return Err(Reject::Filtered);
            }
            true
        } else if dst.is_multicast() {
            let bucket = dst.multicast_hash();
            let passes = state.mar[(bucket >> 3) as usize] & (1 << (bucket & 7)) != 0;
            if state.rcr & RCR_AM == 0 || !passes {
                return Err(Reject::Filtered);
            }
            true
        } else if dst == par || state.rcr & RCR_PRO != 0 {
            false
        } else {
            return Err(Reject::Filtered);
        };

        if state.rcr & RCR_MON != 0 {
            return Err(Reject::Monitored);
        }
        Ok(group)
    }

    /// Put one frame in the ring. Called with the state lock held.
    ///
    /// *DP8390D*, "Packet Reception": each packet is preceded by a four-byte
    /// header — receive status, the page the *next* packet starts on, and a
    /// 16-bit byte count that **includes the header itself** — and the whole
    /// thing wraps from `PSTOP` back to `PSTART`.
    /// Returns false when there was no room, which is a ring overflow.
    fn store(state: &mut State, frame: &[u8], group: bool) -> bool {
        let total = frame.len() + 4;
        // Round up to whole pages: the NIC always leaves the next packet page
        // aligned, which is what makes the header's next-page pointer enough.
        let pages = total.div_ceil(256) as u16;
        if pages > state.free_pages() {
            return false;
        }

        let start = u16::from(state.curr);
        let stop = u16::from(state.pstop);
        let ring_start = u16::from(state.pstart);
        let mut next = start + pages;
        if next >= stop {
            next -= stop - ring_start;
        }

        let rsr = RSR_PRX | if group { RSR_PHY } else { 0 };
        let header = [rsr, next as u8, total as u8, (total >> 8) as u8];

        let mut addr = start << 8;
        for byte in header.iter().copied().chain(frame.iter().copied()) {
            state.mem_write(addr, byte);
            addr = addr.wrapping_add(1);
            if addr == state.pstop_addr() {
                addr = state.pstart_addr();
            }
        }

        state.clda = addr;
        state.curr = next as u8;
        state.rsr = rsr;
        state.isr |= ISR_PRX;
        true
    }

    /// Take in everything the link has for us up to `target`, then stand there.
    ///
    /// This is the whole of the receive path's relationship with time.
    fn advance_to(&self, target: u64) {
        let asserted = {
            let mut state = self.state.lock();
            if target <= state.tick {
                // Running backwards is a no-op, not an error.
                return;
            }
            let par = MacAddr(state.par);
            while state.receiver_on() {
                // Taking the link's own leaf lock under the device lock is the
                // ranked order (`core::sync`), and is what the 16550 does with
                // its character port.
                let Some(frame) = self.link.receive(target) else {
                    break;
                };
                if frame.len() < 6 || frame.len() as u64 > MAX_FRAME_LEN {
                    // Not a frame. Nothing on this seam should produce one.
                    state.tally(2);
                    continue;
                }
                match Self::accepts(&state, par, &frame) {
                    Ok(group) => {
                        if !Self::store(&mut state, &frame, group) {
                            // *DP8390D*, "Buffer Ring Overflow": the bit is set
                            // and the receiver stops until the driver's
                            // recovery, which starts with STP.
                            state.isr |= ISR_OVW;
                            state.overflow = true;
                        }
                    }
                    Err(Reject::Runt) => {
                        state.isr |= ISR_RXE;
                        state.tally(1);
                    }
                    Err(Reject::Monitored) => state.tally(0),
                    Err(Reject::Filtered) => {}
                }
            }
            state.tick = target;
            self.republish(&state);
            state.irq()
        };
        self.drive(asserted);
    }

    /// Read one of the sixteen chip registers. `debug` suppresses every side
    /// effect.
    fn read_chip(&self, index: u8, debug: bool) -> u8 {
        let mut state = self.state.lock();
        let page = state.page();
        match (page, index) {
            (_, 0) => state.cr,
            (0, 1) => state.clda as u8,
            (0, 2) => (state.clda >> 8) as u8,
            (0, 3) => state.bnry,
            (0, 4) => state.tsr,
            // No collisions on a wire with one station on it.
            (0, 5) => 0,
            // The FIFO is not modelled; the data sheet calls it diagnostic.
            (0, 6) => 0,
            (0, 7) => state.isr,
            (0, 8) => state.crda as u8,
            (0, 9) => (state.crda >> 8) as u8,
            // Reserved on a DP8390. An RTL8019AS puts its ID here; this is not
            // one, and saying so is what lets a driver tell them apart.
            (0, 10 | 11) => 0,
            (0, 12) => state.rsr,
            (0, 13..=15) => {
                let which = (index - 13) as usize;
                let value = state.cntr[which];
                if !debug {
                    // The tally counters are cleared by the read. This is
                    // exactly the side effect `MemAttrs::debug` exists for.
                    state.cntr[which] = 0;
                }
                value
            }
            (1, 1..=6) => state.par[(index - 1) as usize],
            (1, 7) => state.curr,
            (1, 8..=15) => state.mar[(index - 8) as usize],
            // Page 2 is read-back for diagnostics (*DP8390D*, "Register
            // Descriptions"): the write-only page 0 registers, readable.
            (2, 1) => state.pstart,
            (2, 2) => state.pstop,
            (2, 3) => state.bnry,
            (2, 4) => state.tpsr,
            (2, 8) => state.rcr,
            (2, 9) => state.tcr,
            (2, 10) => state.dcr,
            (2, 11) => state.imr,
            // Page 3 does not exist on a DP8390, and the rest of page 2 is
            // reserved.
            _ => 0,
        }
    }

    /// Write one of the sixteen chip registers.
    ///
    /// Returns a frame to put on the wire, if this write started a
    /// transmission. Putting it there is an outward call, and the caller makes
    /// it with the lock released.
    fn write_chip(&self, index: u8, value: u8) -> Option<Vec<u8>> {
        let mut state = self.state.lock();
        let page = state.page();
        let mut frame = None;
        let mut mac_changed = false;
        match (page, index) {
            (_, 0) => {
                let was_started = state.cr & CR_STA != 0 && state.cr & CR_STP == 0;
                state.cr = value;
                if value & CR_STP != 0 {
                    // Stop: the reset bit comes back, the DMA is abandoned, and
                    // an overflow is forgiven — which is what makes the data
                    // sheet's recovery procedure work.
                    state.isr |= ISR_RST;
                    state.remaining = 0;
                    state.overflow = false;
                } else if value & CR_STA != 0 {
                    state.isr &= !ISR_RST;
                }
                match state.dma_command() {
                    RD_READ | RD_WRITE => {
                        state.crda = state.rsar;
                        state.remaining = state.rbcr;
                        if state.remaining == 0 {
                            state.isr |= ISR_RDC;
                        }
                    }
                    RD_SEND => {
                        // *DP8390D*, "Remote DMA": the transfer starts at the
                        // boundary pointer and its length is the byte count in
                        // that packet's own header.
                        let base = u16::from(state.bnry) << 8;
                        let lo = state.mem_read(&self.prom, base.wrapping_add(2));
                        let hi = state.mem_read(&self.prom, base.wrapping_add(3));
                        state.crda = base;
                        state.remaining = u16::from(lo) | (u16::from(hi) << 8);
                        state.rbcr = state.remaining;
                        if state.remaining == 0 {
                            state.isr |= ISR_RDC;
                        }
                    }
                    // 0 is "not allowed" and 4-7 abort. Both stop the engine.
                    _ => state.remaining = 0,
                }
                if value & CR_TXP != 0 && value & CR_STP == 0 {
                    // The carrier is the link's business, and asking is a leaf
                    // lock under this one — the ranked order allows it, and it
                    // has to be settled before the status registers are.
                    let carrier = self.link.link_up();
                    frame = Self::start_transmit(&mut state, was_started, carrier);
                }
            }
            (0, 1) => state.pstart = value,
            (0, 2) => state.pstop = value,
            (0, 3) => {
                state.bnry = value;
                // Freeing a page may make the ring usable again, which changes
                // when the next arrival can land.
                state.overflow = state.overflow && !state.ring_ok();
            }
            (0, 4) => state.tpsr = value,
            (0, 5) => state.tbcr = (state.tbcr & 0xff00) | u16::from(value),
            (0, 6) => state.tbcr = (state.tbcr & 0x00ff) | (u16::from(value) << 8),
            // ISR is write-one-to-clear, and RST is not writable at all.
            (0, 7) => state.isr &= !(value & IMR_MASK),
            (0, 8) => state.rsar = (state.rsar & 0xff00) | u16::from(value),
            (0, 9) => state.rsar = (state.rsar & 0x00ff) | (u16::from(value) << 8),
            (0, 10) => state.rbcr = (state.rbcr & 0xff00) | u16::from(value),
            (0, 11) => state.rbcr = (state.rbcr & 0x00ff) | (u16::from(value) << 8),
            (0, 12) => state.rcr = value,
            (0, 13) => state.tcr = value,
            (0, 14) => state.dcr = value,
            (0, 15) => state.imr = value & IMR_MASK,
            (1, 1..=6) => {
                state.par[(index - 1) as usize] = value;
                mac_changed = true;
            }
            (1, 7) => state.curr = value,
            (1, 8..=15) => state.mar[(index - 8) as usize] = value,
            // Pages 2 and 3 are not written. The data sheet is explicit that
            // page 2 is for diagnostics and that writing it is not supported.
            _ => {}
        }
        let par = MacAddr(state.par);
        self.republish(&state);
        drop(state);
        if mac_changed {
            // An outward call, with the lock released (`CLAUDE.md`).
            self.link.set_mac(par);
        }
        frame
    }

    /// Pull `TBCR` bytes out of the transmit page and mark the transmission
    /// finished.
    ///
    /// *DP8390D*, "Packet Transmission". Called with the state lock held; the
    /// frame it returns goes on the wire once the caller has let go. The status
    /// registers are settled here, before the lock is released, so a guest can
    /// never see a half-finished transmission.
    fn start_transmit(state: &mut State, started: bool, carrier: bool) -> Option<Vec<u8>> {
        // TXP clears whatever happens: this model's transmitter is
        // instantaneous, and the bit is how the driver knows it is done.
        state.cr &= !CR_TXP;
        let len = state.tbcr as usize;
        if !started || len == 0 {
            // A transmission with the chip stopped, or of nothing at all, is
            // not a transmission at all.
            return None;
        }
        if !carrier {
            // *DP8390D*: with no carrier the transmission aborts and both bits
            // are reported, which is what a driver's link check is looking at.
            state.tsr = TSR_CRS | TSR_ABT;
            state.isr |= ISR_TXE;
            return None;
        }
        let mut frame = Vec::with_capacity(len.min(MAX_FRAME_LEN as usize));
        let mut addr = u16::from(state.tpsr) << 8;
        for _ in 0..len.min(MAX_FRAME_LEN as usize) {
            // The PROM is never a transmit source: the transmit page always
            // points into buffer RAM.
            frame.push(if (MEM_START..MEM_END).contains(&addr) {
                state.mem[(addr - MEM_START) as usize]
            } else {
                0xff
            });
            addr = addr.wrapping_add(1);
        }
        state.clda = addr;
        state.tsr = TSR_PTX;
        state.isr |= ISR_PTX;
        Some(frame)
    }

    /// The byte `ahead` positions along the transfer in progress, without
    /// touching it: the debugger's view of the data window.
    ///
    /// A debug read must not advance the remote DMA (`ROADMAP.md` §15,
    /// invariant 5) — this window is the NIC's version of a FIFO, and reading
    /// it is exactly the pop the rule exists to forbid. `ahead` is 1 for the
    /// high half of a 16-bit access, which is why this is not simply "the byte
    /// at `CRDA`".
    fn peek_data(&self, ahead: u16) -> u8 {
        let state = self.state.lock();
        let command = state.dma_command();
        if state.remaining <= ahead || (command != RD_READ && command != RD_SEND) {
            return 0xff;
        }
        let mut addr = state.crda;
        for _ in 0..ahead {
            addr = addr.wrapping_add(1);
            if state.ring_ok() && addr == state.pstop_addr() {
                addr = state.pstart_addr();
            }
        }
        state.mem_read(&self.prom, addr)
    }

    /// Read one byte through the card's remote DMA data window, advancing it.
    fn read_data(&self) -> u8 {
        let mut state = self.state.lock();
        let command = state.dma_command();
        if state.remaining == 0 || (command != RD_READ && command != RD_SEND) {
            // Nothing is being transferred; the window is not driven.
            return 0xff;
        }
        let byte = state.mem_read(&self.prom, state.crda);
        state.dma_step();
        state.remaining -= 1;
        if state.remaining == 0 {
            state.isr |= ISR_RDC;
            if state.dma_command() == RD_SEND {
                // Send Packet leaves the boundary pointer on the packet after
                // the one just read, which is the whole convenience of it.
                let base = u16::from(state.bnry) << 8;
                state.bnry = state.mem_read(&self.prom, base.wrapping_add(1));
                state.overflow = state.overflow && !state.ring_ok();
            }
        }
        byte
    }

    /// Write one byte through the card's remote DMA data window.
    fn write_data(&self, value: u8) {
        let mut state = self.state.lock();
        if state.remaining == 0 || state.dma_command() != RD_WRITE {
            return;
        }
        let addr = state.crda;
        state.mem_write(addr, value);
        state.dma_step();
        state.remaining -= 1;
        if state.remaining == 0 {
            state.isr |= ISR_RDC;
        }
    }

    /// Whether the data window is moving sixteen bits at a time.
    fn word_mode(&self) -> bool {
        self.state.lock().dcr & DCR_WTS != 0
    }

    /// Whether a word transfer puts the high byte first (`DCR.BOS`).
    fn big_endian_data(&self) -> bool {
        self.state.lock().dcr & DCR_BOS != 0
    }

    /// The card's reset strap, at `base+0x18`.
    ///
    /// Not a DP8390 register: it is a latch on the NE2000 board that pulses the
    /// chip's reset pin. A read and a write both do it, because which of the
    /// two a given card decodes is a card-level detail and drivers do both.
    fn card_reset(&self) {
        {
            let mut state = self.state.lock();
            let tick = state.tick;
            let mem = core::mem::take(&mut state.mem);
            *state = State::new();
            // Reset is a pin, not a power cycle: the buffer RAM keeps whatever
            // was in it, and the device keeps its place in virtual time.
            state.mem = mem;
            state.tick = tick;
            self.republish(&state);
        }
        self.drive(false);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !attrs.debug {
            // A debug read must not move the device's clock any more than it
            // may advance a ring pointer.
            self.sync();
        }
        let offset = offset & 0x1f;
        match (offset, dst.len()) {
            (0x00..=0x0f, 1) => dst[0] = self.read_chip(offset as u8, attrs.debug),
            (0x10..=0x17, 1) => {
                dst[0] = if attrs.debug {
                    self.peek_data(0)
                } else {
                    self.read_data()
                };
            }
            (0x10..=0x17, 2) if self.word_mode() => {
                let (first, second) = if attrs.debug {
                    (self.peek_data(0), self.peek_data(1))
                } else {
                    (self.read_data(), self.read_data())
                };
                let (lo, hi) = if self.big_endian_data() {
                    (second, first)
                } else {
                    (first, second)
                };
                dst[0] = lo;
                dst[1] = hi;
            }
            (0x18..=0x1f, 1) => {
                if !attrs.debug {
                    self.card_reset();
                }
                dst[0] = 0xff;
            }
            _ => return Err(BusError::BadAccess),
        }
        if !attrs.debug {
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Every register here changes something: a write to CR transmits, a
            // write to the data window moves the DMA on, and a write to the
            // strap resets the card. None can be made harmless.
            return Err(BusError::BadAccess);
        }
        self.sync();
        let offset = offset & 0x1f;
        let mut outgoing = None;
        match (offset, src.len()) {
            (0x00..=0x0f, 1) => outgoing = self.write_chip(offset as u8, src[0]),
            (0x10..=0x17, 1) => self.write_data(src[0]),
            (0x10..=0x17, 2) if self.word_mode() => {
                let (first, second) = if self.big_endian_data() {
                    (src[1], src[0])
                } else {
                    (src[0], src[1])
                };
                self.write_data(first);
                self.write_data(second);
            }
            (0x18..=0x1f, 1) => self.card_reset(),
            _ => return Err(BusError::BadAccess),
        }
        if let Some(frame) = outgoing {
            // The outward call, with the state lock released (`CLAUDE.md`,
            // re-entrancy): a loopback link queues this straight back as an
            // arrival, and a hub calls into a peer device from inside it.
            let now = self.state.lock().tick;
            self.link.transmit(now, &frame);
        }
        self.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Eight-bit registers on an eight-bit port bus, except the data window,
        // which a 16-bit host reads sixteen bits at a time when `DCR.WTS` says
        // so. Alignment is not required: the window mirrors across eight ports
        // and a card in an 8-bit slot only ever does byte cycles.
        AccessConstraints {
            min: Width::U8,
            max: Width::U16,
            natural_alignment: false,
            endian: Endian::Little,
            allow_bulk: false,
            ..AccessConstraints::ANY
        }
    }
}

/// An NE2000 Ethernet card.
#[derive(Debug)]
pub struct Ne2000 {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Ne2000 {
    /// Validate `props` and build the card.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown, and
    /// [`Error::Config`] if `mac` is not six hexadecimal octets.
    pub fn new(props: &Props) -> Result<Ne2000> {
        let mut r = props.reader();
        let link_name = r.or("link", String::from(DEFAULT_LINK))?;
        let mac = match r.optional_str("mac")? {
            Some(text) => MacAddr::parse(text)?,
            // A locally administered unicast address, so a default board on a
            // hub does not collide with real hardware. 52:54:00 is the
            // conventional prefix for an emulated NIC.
            None => MacAddr::new([0x52, 0x54, 0x00, 0x12, 0x34, 0x56]),
        };
        r.finish()?;
        let link = ports::attach(props, &link_name)?;
        Ok(Ne2000::with_link(link, link_name, mac))
    }

    /// Build one against a link the caller already has.
    #[must_use]
    pub fn with_link(link: Arc<dyn NetLink>, link_name: String, mac: MacAddr) -> Ne2000 {
        // The address PROM: sixteen values, each stored twice, so a driver
        // reading 32 bytes in byte mode takes every other one. Values 0-5 are
        // the station address and 14-15 are the card signature.
        let mut prom = [0u8; PROM_LEN];
        for (i, byte) in mac.octets().iter().enumerate() {
            prom[i * 2] = *byte;
            prom[i * 2 + 1] = *byte;
        }
        prom[28] = PROM_SIGNATURE;
        prom[29] = PROM_SIGNATURE;

        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            out: Mutex::with_rank(LockRank::LEAF, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            rx_ready: AtomicBool::new(false),
            link,
            link_name,
            prom,
            mac,
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Ne2000 { regs, region }
    }

    /// The name of the network port this card is attached to.
    #[must_use]
    pub fn link_name(&self) -> &str {
        &self.regs.link_name
    }

    /// The address in the card's PROM.
    #[must_use]
    pub fn mac(&self) -> MacAddr {
        self.regs.mac
    }

    /// Advance to `tick` of the card's clock domain, taking in whatever the
    /// link has for it.
    ///
    /// This is what the scheduler calls; a test that is not running one calls it
    /// directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// Whether the interrupt output is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq()
    }
}

/// The `net.ne2000` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "NE2000 Ethernet card (DP8390 NIC with 16 KiB of buffer RAM)",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: false,
            summary: "the network port to attach to, by name (default \"net0\")",
        },
        PropertySpec {
            name: "mac",
            kind: ValueKind::Str,
            required: false,
            summary: "the station address in the card's PROM, aa:bb:cc:dd:ee:ff",
        },
    ],
    construct: |props| Ok(Box::new(Ne2000::new(props)?)),
};

impl Device for Ne2000 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: tell the far side which address is on this
        // card, before any driver has programmed PAR0-5.
        self.regs.link.set_mac(self.regs.mac);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            *state = State::new();
            // The scheduler owns the domain; a device reset does not rewind it.
            state.tick = tick;
            self.regs.republish(&state);
        }
        self.regs.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an NE2000 drives one pin, `irq`"),
            });
        }
        *self.regs.out.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == "irq" {
            self.regs.refresh();
        }
    }

    fn is_lazy(&self) -> bool {
        // Not because it computes anything per tick, but because *when* a frame
        // becomes visible is guest-visible and has to be a virtual time rather
        // than a host one. See the module documentation.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.regs.next_event()
    }

    fn attach_lazy(&self, handle: crate::core::sched::LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        for byte in [
            state.cr,
            state.isr,
            state.imr,
            state.dcr,
            state.rcr,
            state.tcr,
            state.tsr,
            state.rsr,
            state.pstart,
            state.pstop,
            state.bnry,
            state.curr,
            state.tpsr,
        ] {
            w.write_u8(byte)?;
        }
        for half in [
            state.tbcr,
            state.rsar,
            state.rbcr,
            state.crda,
            state.remaining,
            state.clda,
        ] {
            w.write_u16(half)?;
        }
        w.write_bytes(&state.par)?;
        w.write_bytes(&state.mar)?;
        w.write_bytes(&state.cntr)?;
        w.write_bool(state.overflow)?;
        w.write_bytes(&state.mem)?;
        // The device's own position in its domain. The scheduler restores the
        // domain; without this the two would disagree and every arrival would
        // look overdue.
        w.write_u64(state.tick)
        // The link's queues are the outside's state, not the machine's, and are
        // deliberately absent (`ROADMAP.md` §4.5). So is the PROM, which is a
        // construction property.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        state.cr = r.read_u8()?;
        state.isr = r.read_u8()?;
        state.imr = r.read_u8()?;
        state.dcr = r.read_u8()?;
        state.rcr = r.read_u8()?;
        state.tcr = r.read_u8()?;
        state.tsr = r.read_u8()?;
        state.rsr = r.read_u8()?;
        state.pstart = r.read_u8()?;
        state.pstop = r.read_u8()?;
        state.bnry = r.read_u8()?;
        state.curr = r.read_u8()?;
        state.tpsr = r.read_u8()?;
        state.tbcr = r.read_u16()?;
        state.rsar = r.read_u16()?;
        state.rbcr = r.read_u16()?;
        state.crda = r.read_u16()?;
        state.remaining = r.read_u16()?;
        state.clda = r.read_u16()?;
        let par = r.read_bytes()?;
        let mar = r.read_bytes()?;
        let cntr = r.read_bytes()?;
        if par.len() != 6 || mar.len() != 8 || cntr.len() != 3 {
            return Err(Error::State(alloc::format!(
                "snapshot has a {}-byte station address, a {}-byte hash table and {} tally counters",
                par.len(),
                mar.len(),
                cntr.len()
            )));
        }
        state.par.copy_from_slice(par);
        state.mar.copy_from_slice(mar);
        state.cntr.copy_from_slice(cntr);
        state.overflow = r.read_bool()?;
        let mem = r.read_bytes()?;
        if mem.len() != MEM_LEN {
            return Err(Error::State(alloc::format!(
                "snapshot has {} bytes of buffer RAM in a {MEM_LEN}-byte card",
                mem.len()
            )));
        }
        state.mem = mem.to_vec();
        state.tick = r.read_u64()?;
        {
            let mut live = self.regs.state.lock();
            *live = state;
            self.regs.republish(&live);
        }
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Ne2000 {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ne2000::new(props)?)))
}

/// What the validator should know about `net.ne2000`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("link", ValueKind::Str))
        .prop(PropSchema::new("mac", ValueKind::Str))
        .region("")
        .region("regs")
        .port("irq", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU64;
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use crate::dev::net::link::NetPort;

    /// A wire sink that remembers the last level it was driven to.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU64,
        edges: AtomicU64,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            let now = u64::from(level.is_high());
            if self.level.swap(now, Ordering::Relaxed) != now {
                self.edges.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// A card, its wire, and the far side of it.
    struct Board {
        card: Ne2000,
        port: Arc<NetPort>,
        irq: Arc<Probe>,
    }

    /// The station address every test programs into `PAR0`-`PAR5`.
    const STATION: [u8; 6] = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];

    /// The ring, as a driver lays it out: the transmit buffer occupies pages
    /// 0x40-0x45 (six pages, enough for one maximum frame) and the receive ring
    /// is everything above it.
    const TPSR: u8 = 0x40;
    const PSTART: u8 = 0x46;
    const PSTOP: u8 = 0x80;

    fn board() -> Board {
        board_on(Arc::new(NetPort::new()))
    }

    fn board_on(port: Arc<NetPort>) -> Board {
        let card = Ne2000::with_link(
            Arc::clone(&port) as Arc<dyn NetLink>,
            "test".to_string(),
            MacAddr::new(STATION),
        );
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let irq = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&irq) as Arc<dyn WireSink>, 0)
            .build_shared();
        card.connect("irq", WireSource::new(wire, id))
            .expect("an NE2000 drives irq");
        Board { card, port, irq }
    }

    impl Board {
        fn inb(&self, offset: u64) -> u8 {
            let mut byte = [0u8; 1];
            self.card
                .regs
                .read(offset, &mut byte, MemAttrs::DEFAULT)
                .expect("a byte read is legal");
            byte[0]
        }

        fn peek(&self, offset: u64) -> u8 {
            let mut byte = [0u8; 1];
            self.card
                .regs
                .read(offset, &mut byte, MemAttrs::DEBUG)
                .expect("a debug byte read is legal");
            byte[0]
        }

        fn outb(&self, offset: u64, value: u8) {
            self.card
                .regs
                .write(offset, &[value], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        fn irq_high(&self) -> bool {
            self.irq.level.load(Ordering::Relaxed) == 1
        }

        // -- what a real NE2000 driver does, in the data sheet's order ------

        /// Read the 32-byte address PROM, which is how a driver learns the
        /// card's address and identifies it as an NE2000.
        fn read_prom(&self) -> [u8; PROM_LEN] {
            self.outb(0x00, 0x21); // stop, page 0, abort DMA
            self.outb(0x0e, 0x48); // DCR: byte transfers, normal operation
            self.outb(0x0a, PROM_LEN as u8); // RBCR0
            self.outb(0x0b, 0x00); // RBCR1
            self.outb(0x08, 0x00); // RSAR0
            self.outb(0x09, 0x00); // RSAR1
            self.outb(0x00, 0x0a); // remote read, start
            let mut prom = [0u8; PROM_LEN];
            for byte in &mut prom {
                *byte = self.inb(0x10);
            }
            prom
        }

        /// *DP8390D*, "Initialization Procedures", in order.
        fn init(&self, imr: u8, rcr: u8) {
            self.outb(0x00, 0x21); // 1. STP, page 0, abort remote DMA
            self.outb(0x0e, 0x48); // 2. DCR: byte-wide, normal (LS=1)
            self.outb(0x0a, 0x00); // 3. clear the remote byte count
            self.outb(0x0b, 0x00);
            self.outb(0x0c, 0x20); // 4. RCR: monitor mode while we set up
            self.outb(0x0d, 0x02); // 5. TCR: internal loopback while we set up
            self.outb(0x04, TPSR); // 6. the transmit page
            self.outb(0x01, PSTART); // 7. the ring
            self.outb(0x02, PSTOP);
            self.outb(0x03, PSTART); // BNRY: the ring is empty
            self.outb(0x07, 0xff); // 8. clear every interrupt
            self.outb(0x0f, imr); // 9. and enable the ones we want
            self.outb(0x00, 0x61); // 10. page 1, still stopped
            for (i, byte) in STATION.iter().enumerate() {
                self.outb(1 + i as u64, *byte);
            }
            for i in 0..8 {
                self.outb(8 + i, 0x00); // no multicast
            }
            self.outb(0x07, PSTART); // CURR: an empty ring is CURR == BNRY
            self.outb(0x00, 0x22); // 11. page 0, START
            self.outb(0x0d, 0x00); // 12. out of loopback
            self.outb(0x0c, rcr); // 13. and take packets
        }

        /// Move `bytes` into card memory at `addr` with a remote DMA write, as
        /// a driver does before every transmission.
        fn dma_write(&self, addr: u16, bytes: &[u8]) {
            self.outb(0x0a, bytes.len() as u8);
            self.outb(0x0b, (bytes.len() >> 8) as u8);
            self.outb(0x08, addr as u8);
            self.outb(0x09, (addr >> 8) as u8);
            self.outb(0x00, 0x12); // remote write, start
            for byte in bytes {
                self.outb(0x10, *byte);
            }
            assert_eq!(
                self.inb(0x07) & ISR_RDC,
                ISR_RDC,
                "the DMA said it had finished"
            );
            self.outb(0x07, ISR_RDC);
        }

        /// Pull `len` bytes out of card memory at `addr` with a remote DMA
        /// read, as a driver does for every received packet.
        fn dma_read(&self, addr: u16, len: u16) -> Vec<u8> {
            self.outb(0x0a, len as u8);
            self.outb(0x0b, (len >> 8) as u8);
            self.outb(0x08, addr as u8);
            self.outb(0x09, (addr >> 8) as u8);
            self.outb(0x00, 0x0a); // remote read, start
            let out = (0..len).map(|_| self.inb(0x10)).collect::<Vec<_>>();
            self.outb(0x07, ISR_RDC);
            out
        }

        /// Transmit `frame`, exactly as a driver does: DMA it into the transmit
        /// page, set the byte count, and pull `TXP`.
        fn transmit(&self, frame: &[u8]) {
            self.dma_write(u16::from(TPSR) << 8, frame);
            self.outb(0x04, TPSR);
            self.outb(0x05, frame.len() as u8);
            self.outb(0x06, (frame.len() >> 8) as u8);
            self.outb(0x00, 0x26); // TXP, START, abort remote DMA
        }

        /// The boundary pointer, and the page the NIC is writing to.
        fn ring(&self) -> (u8, u8) {
            let bnry = self.inb(0x03);
            self.outb(0x00, 0x62); // page 1, started
            let curr = self.inb(0x07);
            self.outb(0x00, 0x22); // back to page 0
            (bnry, curr)
        }

        /// Take one packet out of the ring the way a driver's interrupt handler
        /// does, and move `BNRY` on to the next one.
        ///
        /// `BNRY` here names the first page the *host* has not read, so an
        /// empty ring is `CURR == BNRY` — the data sheet's own statement of it,
        /// and what makes the Send Packet command's "start at the boundary
        /// pointer" mean what it says.
        fn receive(&self) -> Option<Vec<u8>> {
            let (bnry, curr) = self.ring();
            if bnry == curr {
                return None;
            }
            let base = u16::from(bnry) << 8;
            let header = self.dma_read(base, 4);
            let next = header[1];
            let count = u16::from(header[2]) | (u16::from(header[3]) << 8);
            assert_eq!(header[0] & RSR_PRX, RSR_PRX, "the packet is marked intact");
            let frame = self.dma_read(base.wrapping_add(4), count - 4);
            self.outb(0x03, next);
            Some(frame)
        }
    }

    /// A frame of `len` bytes from `src` to `dst`, filled with a recognisable
    /// pattern. 60 is the shortest 802.3 allows without an FCS.
    fn frame(dst: [u8; 6], src: [u8; 6], len: usize) -> Vec<u8> {
        let mut f = Vec::with_capacity(len);
        f.extend_from_slice(&dst);
        f.extend_from_slice(&src);
        f.extend_from_slice(&[0x08, 0x00]);
        for i in f.len()..len {
            f.push((i * 7 + 1) as u8);
        }
        f
    }

    #[test]
    fn the_prom_carries_the_station_address_doubled_and_the_card_signature() {
        let b = board();
        let prom = b.read_prom();
        for (i, byte) in STATION.iter().enumerate() {
            assert_eq!(prom[i * 2], *byte);
            assert_eq!(prom[i * 2 + 1], *byte, "each value appears twice");
        }
        assert_eq!(
            (prom[28], prom[29]),
            (PROM_SIGNATURE, PROM_SIGNATURE),
            "an NE2000 says `WW`, which is how a driver tells it from an NE1000"
        );
    }

    #[test]
    fn a_frame_the_driver_transmits_reaches_the_link_byte_for_byte() {
        // Half of the end-to-end proof: the guest builds a frame in card
        // memory through the remote DMA port, pulls TXP, and the exact bytes
        // come out of the backend.
        let b = board();
        b.init(ISR_PTX | ISR_PRX, RCR_AB);
        let sent = frame([0x02, 0, 0, 0, 0, 0x99], STATION, 60);
        b.transmit(&sent);

        assert_eq!(b.port.take().as_deref(), Some(&sent[..]), "byte for byte");
        assert_eq!(b.inb(0x04) & TSR_PTX, TSR_PTX, "TSR says it went");
        assert_eq!(b.inb(0x07) & ISR_PTX, ISR_PTX, "and ISR raised PTX");
        assert_eq!(b.inb(0x00) & CR_TXP, 0, "TXP cleared when it finished");
        assert!(b.irq_high(), "the interrupt is asserted through the wire");
        b.outb(0x07, ISR_PTX);
        assert!(!b.irq_high(), "and acknowledging it lets the pin go");
    }

    #[test]
    fn a_frame_the_link_delivers_is_read_out_through_the_ring_with_an_interrupt() {
        // The other half: a frame injected at the backend crosses the ring and
        // comes back out of the guest's receive path, with the pin asserted.
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        let arriving = frame(STATION, [0x02, 0, 0, 0, 0, 0x01], 74);
        assert!(b.port.deliver_at(1_000, &arriving));

        assert!(!b.irq_high(), "nothing has arrived yet");
        b.card.advance_to(1_000);
        assert!(b.irq_high(), "the arrival raised the pin");
        assert_eq!(b.inb(0x07) & ISR_PRX, ISR_PRX);

        assert_eq!(b.receive().as_deref(), Some(&arriving[..]), "byte for byte");
        b.outb(0x07, ISR_PRX);
        assert!(!b.irq_high(), "acknowledged");
        assert_eq!(b.receive(), None, "and the ring is empty again");
    }

    #[test]
    fn a_frame_becomes_visible_at_its_tick_and_not_one_tick_earlier() {
        // The determinism claim, stated as a test: *when* the guest sees a
        // frame is a virtual time, and it is the one the input named.
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        b.port
            .deliver_at(5_000, &frame(STATION, [2, 0, 0, 0, 0, 1], 60));
        assert_eq!(
            b.card.next_event_tick(),
            Some(5_000),
            "and the scheduler is told to stop there"
        );

        b.card.advance_to(4_999);
        assert!(!b.irq_high(), "not yet");
        assert_eq!(b.ring().0, b.ring().1, "the ring is still empty");
        b.card.advance_to(5_000);
        assert!(b.irq_high(), "now");
        assert!(b.receive().is_some());
        assert_eq!(b.card.next_event_tick(), None, "and nothing else is queued");
    }

    #[test]
    fn a_loopback_link_carries_the_drivers_own_frame_back_to_it() {
        // The deterministic backend that needs no second machine: what the
        // driver transmits arrives at its own receiver, one wire delay later.
        let b = board_on(Arc::new(NetPort::loopback(400)));
        b.init(ISR_PRX | ISR_PTX, RCR_AB);
        let sent = frame(STATION, STATION, 64);
        b.transmit(&sent);
        assert_eq!(b.card.next_event_tick(), Some(400));
        b.card.advance_to(399);
        assert_eq!(b.receive(), None);
        b.card.advance_to(400);
        assert_eq!(b.receive().as_deref(), Some(&sent[..]));
    }

    #[test]
    fn the_address_filter_is_the_data_sheets() {
        let b = board();
        // Unicast for us, unicast for somebody else, and broadcast.
        b.init(ISR_PRX, RCR_AB);
        let mine = frame(STATION, [2, 0, 0, 0, 0, 1], 60);
        let theirs = frame([0x02, 0, 0, 0, 0, 0x77], [2, 0, 0, 0, 0, 1], 60);
        let bcast = frame([0xff; 6], [2, 0, 0, 0, 0, 1], 60);
        for f in [&mine, &theirs, &bcast] {
            b.port.deliver(f);
        }
        b.card.advance_to(10);
        assert_eq!(b.receive().as_deref(), Some(&mine[..]));
        assert_eq!(
            b.receive().as_deref(),
            Some(&bcast[..]),
            "the one for somebody else was dropped, the broadcast was not"
        );
        assert_eq!(b.receive(), None);

        // With AB clear the broadcast goes too; with PRO set everything stays.
        let b = board();
        b.init(ISR_PRX, 0);
        b.port.deliver(&bcast);
        b.card.advance_to(10);
        assert_eq!(b.receive(), None, "RCR.AB is clear");

        let b = board();
        b.init(ISR_PRX, RCR_PRO);
        b.port.deliver(&theirs);
        b.card.advance_to(10);
        assert_eq!(
            b.receive().as_deref(),
            Some(&theirs[..]),
            "promiscuous takes a frame addressed to anyone"
        );
    }

    #[test]
    fn a_multicast_frame_needs_its_bit_in_the_hash_table() {
        let b = board();
        b.init(ISR_PRX, RCR_AM);
        let group = MacAddr::parse("01:00:5e:00:00:01").unwrap();
        let f = frame(group.octets(), [2, 0, 0, 0, 0, 1], 60);
        b.port.deliver(&f);
        b.card.advance_to(10);
        assert_eq!(b.receive(), None, "the table is empty, so nothing passes");

        // Set exactly the one bit this address hashes to.
        let bucket = group.multicast_hash();
        b.outb(0x00, 0x62); // page 1
        b.outb(8 + u64::from(bucket >> 3), 1 << (bucket & 7));
        b.outb(0x00, 0x22); // page 0
        b.port.deliver(&f);
        b.card.advance_to(20);
        assert_eq!(b.receive().as_deref(), Some(&f[..]));
    }

    #[test]
    fn a_runt_is_refused_unless_the_driver_asked_for_runts() {
        let b = board();
        b.init(ISR_PRX | ISR_RXE, RCR_AB);
        let runt = frame(STATION, [2, 0, 0, 0, 0, 1], 20);
        b.port.deliver(&runt);
        b.card.advance_to(10);
        assert_eq!(b.receive(), None);
        assert_eq!(b.inb(0x07) & ISR_RXE, ISR_RXE, "and it is reported");

        let b = board();
        b.init(ISR_PRX, RCR_AB | RCR_AR);
        b.port.deliver(&runt);
        b.card.advance_to(10);
        assert_eq!(b.receive().as_deref(), Some(&runt[..]));
    }

    #[test]
    fn a_packet_that_straddles_the_end_of_the_ring_comes_out_whole() {
        // The wrap is the one thing about a DP8390's ring that a driver cannot
        // paper over, and a remote DMA read has to wrap with it.
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        // Park CURR two pages below PSTOP so the next packet runs off the end.
        b.outb(0x00, 0x62);
        b.outb(0x07, PSTOP - 2);
        b.outb(0x00, 0x22);
        b.outb(0x03, PSTOP - 2); // BNRY with it: the ring is empty

        let big = frame(STATION, [2, 0, 0, 0, 0, 1], 700);
        b.port.deliver(&big);
        b.card.advance_to(10);
        let (_, curr) = b.ring();
        assert!(
            curr < PSTOP - 2,
            "the packet wrapped: CURR came back round to {curr:#x}"
        );
        assert_eq!(b.receive().as_deref(), Some(&big[..]));
    }

    #[test]
    fn a_ring_with_no_room_overflows_and_stops_the_receiver() {
        let b = board();
        b.init(ISR_PRX | ISR_OVW, RCR_AB);
        // The ring is 58 pages; a 1024-byte packet takes five including its
        // header, so a dozen of them cannot fit.
        let big = frame(STATION, [2, 0, 0, 0, 0, 1], 1024);
        for i in 0..20 {
            b.port.deliver_at(i, &big);
        }
        b.card.advance_to(100);
        assert_eq!(b.inb(0x07) & ISR_OVW, ISR_OVW, "the overflow is reported");
        assert!(b.irq_high());
        assert_eq!(
            b.card.next_event_tick(),
            None,
            "and the receiver stops asking to be woken for frames it cannot take"
        );

        // *DP8390D*, "Buffer Ring Overflow": the recovery starts with STP, and
        // it is what puts the receiver back.
        b.outb(0x00, 0x21);
        b.outb(0x0a, 0x00);
        b.outb(0x0b, 0x00);
        b.outb(0x0d, 0x02); // loopback while we drain
        b.outb(0x00, 0x22);
        while b.receive().is_some() {}
        b.outb(0x07, ISR_OVW);
        b.outb(0x0d, 0x00); // out of loopback
        assert!(
            b.card.next_event_tick().is_some(),
            "and the receiver is asking again"
        );
    }

    #[test]
    fn transmitting_with_no_carrier_reports_a_lost_carrier() {
        let b = board();
        b.init(ISR_PTX | ISR_TXE, RCR_AB);
        b.port.set_link(false);
        let f = frame([0xff; 6], STATION, 60);
        b.transmit(&f);
        assert_eq!(b.port.pending_output(), 0, "nothing went out");
        assert_eq!(b.inb(0x04) & (TSR_CRS | TSR_ABT), TSR_CRS | TSR_ABT);
        assert_eq!(b.inb(0x07) & ISR_TXE, ISR_TXE);
    }

    #[test]
    fn the_send_packet_command_reads_a_packet_and_moves_the_boundary() {
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        let f = frame(STATION, [2, 0, 0, 0, 0, 1], 60);
        b.port.deliver(&f);
        b.card.advance_to(10);

        // *DP8390D*, "Remote DMA": Send Packet starts at BNRY and takes its
        // length from the packet's own header, so the driver programs neither.
        let bnry = b.inb(0x03);
        b.outb(0x00, 0x1a); // RD = 011, start
        let header_and_frame = (0..f.len() + 4).map(|_| b.inb(0x10)).collect::<Vec<_>>();
        assert_eq!(&header_and_frame[4..], &f[..]);
        assert_eq!(b.inb(0x07) & ISR_RDC, ISR_RDC);
        assert_ne!(b.inb(0x03), bnry, "and the boundary moved on by itself");
    }

    #[test]
    fn a_debug_access_pops_nothing_and_moves_nothing() {
        // The rule this test exists for: a debugger read must not advance a
        // ring pointer, drain a DMA, clear a tally counter or reset the card.
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        let f = frame(STATION, [2, 0, 0, 0, 0, 1], 60);
        b.port.deliver(&f);
        b.card.advance_to(10);

        // Arm a remote DMA read of the packet header.
        b.outb(0x0a, 4);
        b.outb(0x0b, 0);
        let base = u16::from(b.inb(0x03)) << 8;
        b.outb(0x08, base as u8);
        b.outb(0x09, (base >> 8) as u8);
        b.outb(0x00, 0x0a);

        let crda = (b.peek(0x08), b.peek(0x09));
        let first = b.peek(0x10);
        assert_eq!(b.peek(0x10), first, "a debug read of the data port repeats");
        assert_eq!((b.peek(0x08), b.peek(0x09)), crda, "CRDA did not move");
        assert_eq!(b.inb(0x10), first, "and a real read still gets that byte");

        // The tally counters are cleared by a *guest* read, not a debug one.
        let b2 = board();
        b2.init(ISR_PRX, RCR_AB | RCR_MON);
        b2.port.deliver(&f);
        b2.card.advance_to(10);
        assert_eq!(b2.peek(0x0d), 1, "monitor mode counted it");
        assert_eq!(b2.peek(0x0d), 1, "and a debug read left it there");
        assert_eq!(b2.inb(0x0d), 1);
        assert_eq!(b2.inb(0x0d), 0, "a guest read cleared it");

        // The reset strap is a side effect too.
        let b3 = board();
        b3.init(ISR_PRX, RCR_AB);
        assert_eq!(b3.peek(0x1f), 0xff);
        assert_eq!(
            b3.inb(0x00) & CR_STP,
            0,
            "a debug read of the strap left the card running"
        );
        b3.outb(0x00, 0x62);
        assert_eq!(b3.inb(0x01), STATION[0], "the address is still programmed");
    }

    #[test]
    fn a_debug_write_is_refused_outright() {
        let b = board();
        for offset in [0x00, 0x07, 0x10, 0x1f] {
            assert!(
                b.card.regs.write(offset, &[0x00], MemAttrs::DEBUG).is_err(),
                "a debug write to {offset:#x} must not be accepted"
            );
        }
    }

    #[test]
    fn the_reset_strap_puts_the_chip_back_where_it_started() {
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        assert_eq!(b.inb(0x00) & CR_STP, 0, "started");
        b.outb(0x1f, 0x00);
        assert_eq!(b.inb(0x00), CR_STP | (4 << 3), "CR is 0x21 again");
        assert_eq!(b.inb(0x07) & ISR_RST, ISR_RST, "and ISR.RST is set");
        assert!(!b.irq_high());
    }

    #[test]
    fn word_transfers_move_two_bytes_when_dcr_says_so() {
        let b = board();
        b.init(ISR_PRX, RCR_AB);
        b.outb(0x0e, 0x49); // DCR: WTS
        // A 16-bit write through the data window, then a 16-bit read back.
        b.outb(0x0a, 4);
        b.outb(0x0b, 0);
        b.outb(0x08, 0x00);
        b.outb(0x09, 0x40);
        b.outb(0x00, 0x12);
        b.card
            .regs
            .write(0x10, &[0x34, 0x12], MemAttrs::DEFAULT)
            .unwrap();
        b.card
            .regs
            .write(0x10, &[0x78, 0x56], MemAttrs::DEFAULT)
            .unwrap();

        b.outb(0x0a, 4);
        b.outb(0x0b, 0);
        b.outb(0x08, 0x00);
        b.outb(0x09, 0x40);
        b.outb(0x00, 0x0a);

        // A debug word read shows the *next two* bytes and moves nothing, so
        // asking twice answers twice the same.
        let mut peeked = [0u8; 2];
        b.card
            .regs
            .read(0x10, &mut peeked, MemAttrs::DEBUG)
            .unwrap();
        assert_eq!(peeked, [0x34, 0x12]);
        b.card
            .regs
            .read(0x10, &mut peeked, MemAttrs::DEBUG)
            .unwrap();
        assert_eq!(peeked, [0x34, 0x12], "a debug word read advanced the DMA");

        let mut out = [0u8; 2];
        b.card.regs.read(0x10, &mut out, MemAttrs::DEFAULT).unwrap();
        assert_eq!(out, [0x34, 0x12]);
        b.card.regs.read(0x10, &mut out, MemAttrs::DEFAULT).unwrap();
        assert_eq!(out, [0x78, 0x56]);
    }

    #[test]
    fn a_word_access_to_a_chip_register_is_refused() {
        let b = board();
        assert!(
            b.card
                .regs
                .read(0x00, &mut [0u8; 2], MemAttrs::DEFAULT)
                .is_err(),
            "the sixteen chip registers are eight bits wide"
        );
        assert!(
            b.card
                .regs
                .read(0x10, &mut [0u8; 2], MemAttrs::DEFAULT)
                .is_err(),
            "and the data window is too until DCR.WTS says otherwise"
        );
    }

    #[test]
    fn a_snapshot_round_trips_to_an_identical_state_hash() {
        let saved = board();
        saved.init(ISR_PRX | ISR_PTX, RCR_AB);
        let f = frame(STATION, [2, 0, 0, 0, 0, 1], 100);
        saved.port.deliver_at(700, &f);
        saved.card.advance_to(700);
        saved.transmit(&frame([0xff; 6], STATION, 60));

        let mut shape = MachineShape::new();
        shape.add_device("nic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("nic", CLASS.name, CLASS.version).unwrap();
            saved.card.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = board();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("nic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.card.load(&mut chunk.reader()).unwrap();

        // The hash: save the restored device and compare the bytes. Anything
        // the encoding forgot shows up here rather than three months later.
        let mut shape = MachineShape::new();
        shape.add_device("nic", CLASS.name).unwrap();
        let mut w2 = StateWriter::new(shape);
        {
            let mut chunk = w2.chunk("nic", CLASS.name, CLASS.version).unwrap();
            restored.card.save(&mut chunk).unwrap();
        }
        assert_eq!(w2.to_vec().unwrap(), bytes, "identical state");

        // And the guest sees the same machine: the packet is still in the ring.
        assert_eq!(restored.card.current_tick(), 700);
        assert!(restored.card.irq_asserted());
        assert_eq!(restored.receive().as_deref(), Some(&f[..]));
    }

    #[test]
    fn the_link_is_told_the_address_the_driver_programs() {
        let b = board();
        // Realize alone publishes the PROM's address.
        let hosts = crate::core::hosts::HostObjects::new();
        let mut deferred = crate::core::device::Deferred::new();
        b.card
            .realize(&mut RealizeCtx::new(
                "nic",
                crate::core::space::RequesterId::ANONYMOUS,
                &mut deferred,
                &hosts,
            ))
            .unwrap();
        assert_eq!(b.port.mac(), MacAddr::new(STATION));

        let other = MacAddr::parse("02:11:22:33:44:55").unwrap();
        b.outb(0x00, 0x61);
        for (i, byte) in other.octets().iter().enumerate() {
            b.outb(1 + i as u64, *byte);
        }
        assert_eq!(b.port.mac(), other, "and PAR0-5 moves it");
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let card =
            Ne2000::new(&Props::new().with("mac", "aa:bb:cc:dd:ee:ff")).expect("a MAC is legal");
        assert_eq!(card.mac().to_string(), "aa:bb:cc:dd:ee:ff");
        assert_eq!(card.link_name(), DEFAULT_LINK);
        assert!(Ne2000::new(&Props::new().with("mac", "nonsense")).is_err());
        assert!(Ne2000::new(&Props::new().with("lnik", "net0")).is_err());
    }

    #[test]
    fn the_card_drives_one_pin_and_says_so_about_the_rest() {
        let b = board();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(b.card.connect("tx", WireSource::new(wire, id)).is_err());
    }
}
