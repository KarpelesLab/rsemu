//! An Intel 8237A DMA controller, plus the PC's page-register latch file.
//!
//! # Sources
//!
//! * *Intel 8237A/8237A-4/8237A-5 High Performance Programmable DMA
//!   Controller* data sheet. The register map, the byte pointer flip-flop, the
//!   command/mode/request/mask register bit layouts, the four transfer types,
//!   the four service modes, autoinitialise, fixed versus rotating priority and
//!   the terminal-count rule all come from it.
//! * *IBM Personal Computer AT Technical Reference* (1984). The board: two
//!   controllers, the second cascaded through the first on channel 4, the
//!   16-bit channels counting in words, and — the part that is in no Intel data
//!   sheet — which page latch belongs to which channel.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # Two regions, because the board decodes them 0x80 apart
//!
//! [`Device::region`] publishes `"regs"` (also `""`) and `"pages"`, and they
//! are deliberately not one aperture:
//!
//! * `regs` is the sixteen-byte control block of the 8237A itself — the chip
//!   Intel sells, at port `0x00` on an AT.
//! * `pages` is **not part of the 8237 at all**. The chip drives sixteen
//!   address lines and an AT has twenty-four, so the board puts a 74LS612-style
//!   latch file beside it to supply the high bits, and decodes it at port
//!   `0x80` — 0x80 away from the controller, with the interrupt controller and
//!   the timer in between. Two regions is what that wiring is.
//!
//! One consequence worth stating: each instance of this class carries its own
//! latch file, while a real AT has one shared by both controllers. A board that
//! maps only the first controller's `pages` therefore leaves the second one's
//! page bits at zero. Sharing the latches is a board-level change — one more
//! object, or both controllers naming one region — not something either 8237
//! can do for itself.
//!
//! # The control block
//!
//! ```text
//!   0x0  channel 0 address    0x8  W: command      R: status
//!   0x1  channel 0 count      0x9  W: request register
//!   0x2  channel 1 address    0xa  W: single mask bit
//!   0x3  channel 1 count      0xb  W: mode register
//!   0x4  channel 2 address    0xc  W: clear byte pointer flip-flop
//!   0x5  channel 2 count      0xd  W: master clear  R: temporary register
//!   0x6  channel 3 address    0xe  W: clear mask register
//!   0x7  channel 3 count      0xf  W: write all mask bits
//! ```
//!
//! The address and count registers are sixteen bits behind an eight-bit port,
//! so a **byte pointer flip-flop** decides which half the next access sees. One
//! flip-flop is shared by all of them, which is why software clears it (`0xc`)
//! before programming a channel and why a debugger must not disturb it.
//!
//! Reading gives the *current* address or count; writing sets the current
//! register **and** the base register, which is what autoinitialise reloads
//! from.
//!
//! # The word controller
//!
//! An AT has two of these. `mode = "word"`, `base = 4` gives the second one:
//! every register at twice the spacing (a 32-byte block, only even offsets
//! decoded), addresses and counts measured in *words*, and two bytes moved per
//! request. Its address register supplies A1-A16 and the page latch the rest.
//! Channel 4 is the cascade input the first controller hangs off, and it never
//! transfers anything.
//!
//! # What is modelled and what is not
//!
//! Command bits 2 (controller disable), 4 (rotating priority), 6 (DREQ sense)
//! and 7 (DACK sense) do what they say. Bits 0 and 1 (memory-to-memory, and the
//! channel-0 address hold that only applies to it), 3 (compressed timing) and 5
//! (extended write) are stored and round-trip through a snapshot but have no
//! effect: two are bus-timing knobs with no meaning above the pin level, and
//! memory-to-memory is not modelled, because nothing on a PC uses it.
//!
//! A software request (`0x9`) is serviced here in any non-cascade mode. The
//! data sheet recognises software requests in block mode only; the difference
//! is visible to a guest and it is deliberate, because a request that silently
//! did nothing in the three other modes is a far worse failure to debug than
//! one that works.
//!
//! There is no arbitration timing. A request is serviced the moment it is seen
//! rather than on the next bus cycle, because this controller is driven by wire
//! events: it registers no scheduler event and reads no clock (`CLAUDE.md`,
//! determinism).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{DmaPeripheral, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.dma";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Channels one 8237A has.
pub const CHANNELS: usize = 4;

/// How much address space a byte controller's register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 16;

/// How much a word controller's answers: the same sixteen registers, each at an
/// even offset, because the second controller sits on the high half of a 16-bit
/// bus and only `A1` upwards reaches it.
pub const WORD_REGISTER_WINDOW_LEN: u64 = 32;

/// How much the page-register latch file answers.
pub const PAGE_WINDOW_LEN: u64 = 16;

/// The most units one request may move before the burst is cut short.
///
/// Demand and block mode transfer for as long as the peripheral wants service,
/// and a peripheral that never drops `DREQ` — a buggy guest driver, a fuzz
/// case, an autoinitialising channel wired to a device that is always ready —
/// would otherwise spin forever inside one wire event and hang the machine.
///
/// 65536 is chosen so the bound can never truncate a *legal* transfer: the
/// count register is sixteen bits and holds one less than the number of units,
/// so the longest transfer the chip can express is exactly this many. What the
/// bound stops is the case that has no terminal count at all.
pub const MAX_BURST_UNITS: u32 = 65_536;

// -- command register (write 0x8) -------------------------------------------

/// Controller disable: no channel is serviced while it is set.
const CMD_DISABLE: u8 = 0x04;
/// Rotating priority: the channel just serviced becomes the lowest.
const CMD_ROTATING: u8 = 0x10;
/// `DREQ` is asserted low rather than high.
const CMD_DREQ_ACTIVE_LOW: u8 = 0x40;
/// `DACK` is asserted high rather than low.
const CMD_DACK_ACTIVE_HIGH: u8 = 0x80;

// -- mode register (write 0xb) ----------------------------------------------

/// Bits 0-1: which channel the rest of the byte programs.
const MODE_CHANNEL: u8 = 0x03;
/// Bits 2-3: the transfer type.
const MODE_TRANSFER: u8 = 0x0c;
/// Bit 4: reload address and count from the base registers at terminal count.
const MODE_AUTOINIT: u8 = 0x10;
/// Bit 5: count the address down rather than up.
const MODE_DECREMENT: u8 = 0x20;
/// Bits 6-7: which service mode.
const MODE_SELECT: u8 = 0xc0;

/// Verify: the chip runs the cycle but moves nothing.
const XFER_VERIFY: u8 = 0;
/// Write transfer: the peripheral supplies a byte, memory receives it.
const XFER_WRITE: u8 = 1;
/// Read transfer: memory supplies a byte, the peripheral receives it.
const XFER_READ: u8 = 2;
/// The fourth encoding is illegal and selects no transfer at all.
const XFER_ILLEGAL: u8 = 3;

/// Demand mode: transfer while the peripheral keeps asking.
const SELECT_DEMAND: u8 = 0;
/// Single mode: one unit per request.
const SELECT_SINGLE: u8 = 1;
/// Block mode: transfer until terminal count.
const SELECT_BLOCK: u8 = 2;
/// Cascade mode: the channel is another controller's hold request rather than a
/// transfer of its own.
const SELECT_CASCADE: u8 = 3;

/// Which latch in the page file supplies each channel's high address bits.
///
/// **This is the AT's wiring, and it is not in numeric order.** The latch file
/// is one chip with sixteen addresses and the board picked whichever outputs
/// were convenient to route, so channel 0's page is at offset 7, channel 2's at
/// offset 1, and so on. It looks like a transcription mistake and is not; the
/// numbers are the *IBM PC/AT Technical Reference* I/O map's (ports 0x87, 0x83,
/// 0x81, 0x82, 0x8f, 0x8b, 0x89, 0x8a).
///
/// Index 4 is the refresh page latch rather than a DMA channel: channel 4 is
/// the cascade and never transfers, so nothing ever reads that entry.
const PAGE_OFFSET: [u8; 8] = [0x7, 0x3, 0x1, 0x2, 0xf, 0xb, 0x9, 0xa];

/// Which controller this is: how wide a unit is, and where its channels start.
#[derive(Debug, Clone, Copy)]
struct Config {
    /// True for the 16-bit controller: registers at twice the spacing,
    /// addresses and counts in words, two bytes per request.
    word: bool,
    /// The number of this controller's first channel — 0 or 4.
    base: u8,
}

impl Config {
    /// The absolute channel number of local channel `index`.
    fn channel(&self, index: usize) -> u8 {
        self.base.wrapping_add(index as u8)
    }

    /// How many bytes one request moves.
    fn unit(&self) -> u64 {
        if self.word { 2 } else { 1 }
    }

    /// How much address space the control block answers.
    fn window(&self) -> u64 {
        if self.word {
            WORD_REGISTER_WINDOW_LEN
        } else {
            REGISTER_WINDOW_LEN
        }
    }
}

/// One channel's programmable state.
#[derive(Debug, Clone, Copy, Default)]
struct Channel {
    /// The current address register, counted in units.
    addr: u16,
    /// What autoinitialise reloads [`Channel::addr`] from.
    base_addr: u16,
    /// The current word count, holding one less than the units remaining.
    count: u16,
    /// What autoinitialise reloads [`Channel::count`] from.
    base_count: u16,
    /// The mode byte as written, channel-select bits included.
    mode: u8,
    /// Whether the channel's request is masked off.
    masked: bool,
}

/// Everything the guest can see or change.
#[derive(Debug, Clone)]
struct State {
    ch: [Channel; CHANNELS],
    /// The command register. Write-only on the chip; kept for save/load.
    command: u8,
    /// Terminal-count flags, bits 0-3 — the low half of the status register.
    ///
    /// The high half (which channels are requesting) is not stored: it is a
    /// live function of the `DREQ` pins and the request register, and the
    /// snapshot of a net's level belongs to the net (`ROADMAP.md` §4.3).
    tc: u8,
    /// Software requests, bits 0-3.
    request: u8,
    /// Which `DREQ` pins are currently asserted, bits 0-3. Not saved, for the
    /// reason above.
    drq: u8,
    /// The byte pointer: false selects the low half of a 16-bit register.
    flipflop: bool,
    /// Under rotating priority, the channel that is currently highest.
    rotate: u8,
    /// The temporary register. Only a memory-to-memory transfer writes it, and
    /// this model performs none, so it reads back whatever a load put there.
    temp: u8,
    /// The board's page latches, by offset within the latch file.
    pages: [u8; 16],
    /// The space transfers traverse.
    ///
    /// `Weak`, like every bus master's handle: the machine owns the space, and
    /// a device that kept its own space alive would close a cycle nothing could
    /// drop (see `dev/nes/dma.rs`).
    bus: Option<Weak<AddressSpace>>,
    /// The identity accesses from this controller carry.
    requester: RequesterId,
}

impl Default for State {
    fn default() -> State {
        State {
            // Reset sets the mask register, so every channel comes up masked.
            ch: [Channel {
                masked: true,
                ..Channel::default()
            }; CHANNELS],
            command: 0,
            tc: 0,
            request: 0,
            drq: 0,
            flipflop: false,
            rotate: 0,
            temp: 0,
            pages: [0; 16],
            bus: None,
            requester: RequesterId::ANONYMOUS,
        }
    }
}

impl State {
    /// The master clear command, which the data sheet defines as having the
    /// same effect as a hardware reset.
    ///
    /// It clears the command, status, request and temporary registers and the
    /// byte pointer, and *sets* the mask register. It does **not** clear the
    /// address and count registers, which the data sheet leaves undefined until
    /// software programs them — and it does not touch the `DREQ` pins or the
    /// page latches, neither of which is inside the chip.
    fn master_clear(&mut self) {
        self.command = 0;
        self.tc = 0;
        self.request = 0;
        self.flipflop = false;
        self.rotate = 0;
        self.temp = 0;
        for ch in &mut self.ch {
            ch.masked = true;
        }
    }
}

/// What one unit of a transfer will do: computed under the state lock, carried
/// out with it released.
#[derive(Debug, Clone, Copy)]
struct Plan {
    /// The address in the traversed space, page bits included.
    phys: u64,
    /// One of [`XFER_VERIFY`], [`XFER_WRITE`], [`XFER_READ`].
    xfer: u8,
    /// Whether the count expires on this unit — the `TC`/`EOP` pulse.
    terminal: bool,
    /// Whether the mode stops the burst after this unit.
    single: bool,
}

/// The state both regions and every pin share.
struct Shared {
    cfg: Config,
    /// Everything mutable, at `DEVICE` rank. **Never held across a call into a
    /// peripheral or into the address space** (`CLAUDE.md`, re-entrancy): the
    /// bus handle and the peripheral are cloned out and the lock released
    /// before the first outward call.
    state: Mutex<State>,
    /// The peripheral driving each channel's `DREQ`, at `LEAF` so it can be
    /// taken with nothing else held.
    peers: [Mutex<Option<Weak<dyn DmaPeripheral>>>; CHANNELS],
    /// Each channel's `DACK` output.
    dack: [Mutex<Option<WireSource>>; CHANNELS],
    /// The shared `/EOP` output.
    eop: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("cfg", &self.cfg);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish_non_exhaustive(),
            None => s.field("state", &"<in use>").finish_non_exhaustive(),
        }
    }
}

impl Shared {
    /// Which register `offset` selects, or `None` if nothing is decoded there.
    ///
    /// The word controller's registers sit at even offsets only; an access to
    /// an odd one reaches no chip select and is open bus.
    fn decode(&self, offset: u64) -> Option<u8> {
        if self.cfg.word {
            if offset & 1 != 0 {
                return None;
            }
            Some(((offset >> 1) & 0xf) as u8)
        } else {
            Some((offset & 0xf) as u8)
        }
    }

    /// Read one control-block register. `debug` suppresses every side effect.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        let mut state = self.state.lock();
        match index {
            0x0..=0x7 => {
                let ch = state.ch[usize::from(index >> 1)];
                let value = if index & 1 == 0 { ch.addr } else { ch.count };
                let half = if state.flipflop {
                    (value >> 8) as u8
                } else {
                    value as u8
                };
                if !debug {
                    state.flipflop = !state.flipflop;
                }
                half
            }
            0x8 => {
                let requesting = (state.drq | state.request) & 0x0f;
                let status = (state.tc & 0x0f) | (requesting << 4);
                if !debug {
                    // The terminal-count bits are cleared by the read, which is
                    // exactly the side effect a debugger must not cause.
                    state.tc = 0;
                }
                status
            }
            0xd => state.temp,
            // Everything else is write-only. Nothing drives the bus, so the
            // access reads as an unpopulated port would.
            _ => 0xff,
        }
    }

    /// Write one control-block register, then service whatever that unblocked.
    fn write_register(&self, index: u8, value: u8) {
        {
            let mut state = self.state.lock();
            let channel = usize::from(value & MODE_CHANNEL);
            let bit = 1u8 << channel;
            match index {
                0x0..=0x7 => {
                    let ff = state.flipflop;
                    let is_count = index & 1 == 1;
                    let ch = &mut state.ch[usize::from(index >> 1)];
                    let reg = if is_count {
                        &mut ch.base_count
                    } else {
                        &mut ch.base_addr
                    };
                    *reg = if ff {
                        (*reg & 0x00ff) | (u16::from(value) << 8)
                    } else {
                        (*reg & 0xff00) | u16::from(value)
                    };
                    // A write loads the base and the current register together.
                    if is_count {
                        ch.count = ch.base_count;
                    } else {
                        ch.addr = ch.base_addr;
                    }
                    state.flipflop = !ff;
                }
                0x8 => state.command = value,
                // Bits 0-1 the channel, bit 2 the request bit itself.
                0x9 if value & 0x04 != 0 => state.request |= bit,
                0x9 => state.request &= !bit,
                0xa => state.ch[channel].masked = value & 0x04 != 0,
                0xb => state.ch[channel].mode = value,
                0xc => state.flipflop = false,
                0xd => state.master_clear(),
                0xe => {
                    for ch in &mut state.ch {
                        ch.masked = false;
                    }
                }
                _ => {
                    for (i, ch) in state.ch.iter_mut().enumerate() {
                        ch.masked = value & (1u8 << i) != 0;
                    }
                }
            }
        }
        // Unmasking a channel whose peripheral is already asking, or setting a
        // software request, starts a transfer — with the lock released, which is
        // the whole point of doing it here rather than above.
        self.service_pending();
    }

    /// Note a change of `DREQ` on local channel `index`, and service it.
    ///
    /// `level` is the level of the net; whether that means *asserted* is the
    /// command register's business (bit 6).
    fn set_request(&self, index: usize, level: Level) {
        let asserted = {
            let mut state = self.state.lock();
            let asserted = if state.command & CMD_DREQ_ACTIVE_LOW != 0 {
                level.is_low()
            } else {
                level.is_high()
            };
            Shared::record_request(&mut state, index, asserted);
            asserted
        };
        if asserted {
            self.service(index);
        }
    }

    /// Record the `DREQ` level of local channel `index`.
    fn record_request(state: &mut State, index: usize, asserted: bool) {
        let bit = 1u8 << index;
        if asserted {
            state.drq |= bit;
        } else {
            state.drq &= !bit;
        }
    }

    /// The channels in priority order, highest first.
    ///
    /// Fixed priority is channel 0 highest. Rotating priority (command bit 4)
    /// starts wherever the last service left the rotation.
    fn priority_order(state: &State) -> [usize; CHANNELS] {
        let start = if state.command & CMD_ROTATING != 0 {
            usize::from(state.rotate)
        } else {
            0
        };
        let mut order = [0usize; CHANNELS];
        for (i, slot) in order.iter_mut().enumerate() {
            *slot = (start + i) % CHANNELS;
        }
        order
    }

    /// Service every channel that is asking, in priority order.
    fn service_pending(&self) {
        let (order, pending) = {
            let state = self.state.lock();
            (
                Shared::priority_order(&state),
                (state.drq | state.request) & 0x0f,
            )
        };
        for index in order {
            if pending & (1u8 << index) != 0 {
                self.service(index);
            }
        }
    }

    /// Decide what the next unit on local channel `index` does, or `None` if
    /// the channel is not in a state to transfer.
    fn plan(&self, index: usize) -> Option<Plan> {
        let state = self.state.lock();
        if state.command & CMD_DISABLE != 0 {
            return None;
        }
        // Channel 4 is the cascade the byte controller's hold request arrives
        // on. It carries no data, whatever its mode register says.
        if self.cfg.channel(index) == 4 {
            return None;
        }
        let ch = state.ch[index];
        if ch.masked {
            return None;
        }
        let xfer = (ch.mode & MODE_TRANSFER) >> 2;
        if xfer == XFER_ILLEGAL {
            return None;
        }
        let select = (ch.mode & MODE_SELECT) >> 6;
        let single = match select {
            SELECT_DEMAND | SELECT_BLOCK => false,
            SELECT_SINGLE => true,
            _ => {
                // Cascade, and nothing else: only two bits reach here. The
                // channel is another controller's HRQ/HLDA handshake and moves
                // nothing of its own.
                debug_assert_eq!(select, SELECT_CASCADE);
                return None;
            }
        };
        let latch = usize::from(PAGE_OFFSET[usize::from(self.cfg.channel(index))]);
        let page = u64::from(state.pages[latch]);
        let addr = u64::from(ch.addr);
        // The page latch supplies A16 upwards on a byte controller. On the word
        // controller the address register counts words, so it supplies A1-A16
        // and the latch the rest — which is why a 16-bit channel cannot cross a
        // 128 KiB boundary while a byte one cannot cross 64 KiB. (Bit 0 of a
        // word channel's latch is not connected on real hardware; every AT BIOS
        // writes it zero, so ORing it in agrees with the board.)
        let phys = if self.cfg.word {
            (page << 16) | (addr << 1)
        } else {
            (page << 16) | addr
        };
        Some(Plan {
            phys,
            xfer,
            // The count register holds one less than the number of units, so
            // the transfer ends when a decrement takes it from 0 to 0xffff.
            // `wrapping_sub` in `complete` is therefore the intended
            // arithmetic and not an accident: 0xffff means 65536 units, and the
            // address counter wraps inside its own 64 KiB page for the same
            // reason — the chip has no carry out of `A15`.
            terminal: ch.count == 0,
            single,
        })
    }

    /// Advance the address and count after a unit, and handle terminal count.
    fn complete(&self, index: usize, terminal: bool) {
        let mut state = self.state.lock();
        let bit = 1u8 << index;
        let ch = &mut state.ch[index];
        // Wrapping on purpose: the counter is sixteen bits wide and a byte
        // channel that runs off the end of its page comes back at the bottom of
        // the same page rather than carrying into the latch.
        let step = if ch.mode & MODE_DECREMENT != 0 {
            u16::MAX // a wrapping add of 0xffff is a decrement
        } else {
            1
        };
        ch.addr = ch.addr.wrapping_add(step);
        ch.count = ch.count.wrapping_sub(1);
        if terminal {
            if ch.mode & MODE_AUTOINIT != 0 {
                ch.addr = ch.base_addr;
                ch.count = ch.base_count;
            } else {
                ch.masked = true;
            }
            state.tc |= bit;
            // A software request is satisfied by the transfer it asked for.
            state.request &= !bit;
        }
    }

    /// Drive a channel's `DACK`, honouring the sense bit in the command
    /// register. Never called with the state lock held.
    fn drive_dack(&self, index: usize, asserted: bool) {
        let Some(out) = self.dack[index].lock().clone() else {
            return;
        };
        let active = if self.state.lock().command & CMD_DACK_ACTIVE_HIGH != 0 {
            Level::High
        } else {
            Level::Low
        };
        out.set(if asserted { active } else { active.inverted() });
    }

    /// Pulse `/EOP`, which is asserted low.
    ///
    /// Nothing in the data path depends on it — the `terminal` flag travels
    /// with the byte through [`DmaPeripheral`] — so this is for a board that
    /// wires `/EOP` somewhere, not for the transfer itself.
    fn pulse_eop(&self) {
        let Some(out) = self.eop.lock().clone() else {
            return;
        };
        out.set(Level::Low);
        out.set(Level::High);
    }

    /// Move bytes for local channel `index` until the mode, the peripheral or
    /// the burst bound says to stop.
    ///
    /// The state lock is taken only inside [`Shared::plan`] and
    /// [`Shared::complete`]; every peripheral call and every bus access below
    /// happens with nothing of ours held.
    fn service(&self, index: usize) {
        let peer = self.peers[index]
            .lock()
            .clone()
            .as_ref()
            .and_then(Weak::upgrade);
        let (bus, attrs) = {
            let state = self.state.lock();
            (
                state.bus.as_ref().and_then(Weak::upgrade),
                MemAttrs::DEFAULT.with_requester(state.requester),
            )
        };
        // `Instance::bind` refuses a machine that reaches here without a space,
        // so this is a hand-wired caller that skipped `attach_bus`.
        let Some(bus) = bus else {
            return;
        };
        let unit = self.cfg.unit();

        let mut acknowledged = false;
        let mut moved = 0u32;
        while moved < MAX_BURST_UNITS {
            let Some(plan) = self.plan(index) else { break };
            if plan.xfer != XFER_VERIFY && peer.is_none() {
                // Nothing drives the data pins. Counting the unit anyway would
                // fabricate a transfer that never happened.
                break;
            }
            if !acknowledged {
                self.drive_dack(index, true);
                acknowledged = true;
            }

            let mut faulted = false;
            match (plan.xfer, peer.as_ref()) {
                (XFER_WRITE, Some(peer)) => {
                    for i in 0..unit {
                        // `TC` is asserted on the last byte of the unit, not on
                        // each half of a word.
                        let byte = peer.dma_read(plan.terminal && i + 1 == unit);
                        if bus
                            .write(plan.phys.wrapping_add(i), Width::U8, u64::from(byte), attrs)
                            .is_err()
                        {
                            faulted = true;
                            break;
                        }
                    }
                }
                (XFER_READ, Some(peer)) => {
                    for i in 0..unit {
                        match bus.read(plan.phys.wrapping_add(i), Width::U8, attrs) {
                            Ok(value) => {
                                peer.dma_write(value as u8, plan.terminal && i + 1 == unit)
                            }
                            Err(_) => {
                                faulted = true;
                                break;
                            }
                        }
                    }
                }
                // A verify cycle drives the address and the acknowledge but no
                // data. It still counts, which is what makes it useful.
                _ => {}
            }
            if faulted {
                // The 8237 has no status bit for a bus fault, so there is
                // nothing to report to the guest: the burst stops and the unit
                // that faulted is not counted.
                break;
            }

            self.complete(index, plan.terminal);
            moved = moved.wrapping_add(1);
            if plan.terminal {
                self.pulse_eop();
            }
            // Single mode is *not* stopped here, and that is a decision worth
            // stating. On real silicon single transfer mode moves one unit,
            // releases `HRQ` so the CPU gets a bus cycle, and — if `DREQ` is
            // still asserted — immediately arbitrates for the bus again. Demand
            // mode simply never lets go. The two therefore differ in how long
            // the CPU is held off, and **not at all** in the sequence of bytes
            // that moves.
            //
            // rsemu models no bus arbitration cycles: a transfer happens
            // between one guest access and the next, and there is nothing to
            // interleave a released bus with. So the two modes are
            // indistinguishable here, and stopping single mode after one unit
            // would not model the difference — it would model a controller that
            // never finishes, because a peripheral holds `DREQ` at a level and
            // a level that does not change delivers no second notification.
            // That is what a floppy read looked like before this comment
            // existed: one byte in memory and a controller stuck in its
            // execution phase forever.
            //
            // The bound below is what keeps that honest: the burst still ends
            // at a terminal count, at a mask, or at `MAX_BURST_UNITS`.
            let _ = plan.single;
            match peer.as_ref() {
                Some(peer) if peer.dma_ready() => {}
                _ => break,
            }
        }

        if acknowledged {
            self.drive_dack(index, false);
            let mut state = self.state.lock();
            if state.command & CMD_ROTATING != 0 {
                // The channel just served becomes the lowest priority.
                state.rotate = ((index + 1) % CHANNELS) as u8;
            }
        }
    }
}

/// The sixteen-byte control block, as something an address space dispatches to.
#[derive(Debug)]
struct ControlBlock {
    shared: Arc<Shared>,
}

impl MemOps for ControlBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = match self.shared.decode(offset) {
            Some(index) => self.shared.read_register(index, attrs.debug),
            None => 0xff,
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Every register here does something: a mode write reprograms a
            // channel and a mask write can start a transfer. None of it can be
            // made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        if let Some(index) = self.shared.decode(offset) {
            self.shared.write_register(index, *value);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on an 8-bit port bus, both controllers alike: the word
        // controller moves 16-bit *data*, but its registers are still bytes.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// The board's page-register latch file.
#[derive(Debug)]
struct PageLatches {
    shared: Arc<Shared>,
}

impl MemOps for PageLatches {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // A latch: reading one has no side effect at all, so `debug` needs no
        // special case here.
        *byte = self.shared.state.lock().pages[(offset & 0xf) as usize];
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Changing where the next transfer lands is not a harmless act.
            return Err(BusError::BadAccess);
        }
        self.shared.state.lock().pages[(offset & 0xf) as usize] = *value;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// One channel's `DREQ` input pin.
#[derive(Debug)]
struct RequestPin {
    shared: Arc<Shared>,
    index: usize,
    /// Which sources are asserting, so a net with more than one driver is
    /// resolved rather than dropped on the first deassertion (`ROADMAP.md`
    /// §4.3).
    inputs: FanIn,
}

impl WireSink for RequestPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Resolved as the wired-OR of the net's drivers; the *sense* of the
        // resulting level — whether high or low means "requesting" — is the
        // command register's business, not the net's.
        self.shared
            .set_request(self.index, self.inputs.resolve(Resolve::Or));
    }
}

/// An Intel 8237A DMA controller.
#[derive(Debug)]
pub struct Dma8237 {
    shared: Arc<Shared>,
    regs: RegionRef,
    pages: RegionRef,
    /// The device owns its input pins; a wire holds only a `Weak` to them.
    pins: Mutex<Vec<Arc<RequestPin>>>,
}

impl Dma8237 {
    /// Validate `props` and build the device. Performs no outward action.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, is unknown, or
    /// names something other than a controller this class can be.
    pub fn new(props: &Props) -> Result<Dma8237> {
        let mut r = props.reader();
        let mode = r.or_str("mode", "byte")?;
        let base = r.or_range("base", 0u64, 0..=4)?;
        r.finish()?;
        let word = match mode {
            "byte" => false,
            "word" => true,
            other => {
                return Err(Error::Property(format!(
                    "property `mode` must be \"byte\" or \"word\", not \"{other}\""
                )));
            }
        };
        Dma8237::with_config(word, base as u8)
    }

    /// Build one directly: `word` picks the 16-bit controller, `base` the
    /// number of its first channel.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] unless `base` is 0 or 4. Any other value would put
    /// the channels where no page latch is wired.
    pub fn with_config(word: bool, base: u8) -> Result<Dma8237> {
        if base != 0 && base != 4 {
            return Err(Error::Property(format!(
                "property `base` must be 0 or 4 (a controller owns four consecutive \
                 channels), not {base}"
            )));
        }
        let cfg = Config { word, base };
        let shared = Arc::new(Shared {
            cfg,
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            peers: core::array::from_fn(|_| Mutex::with_rank(LockRank::LEAF, None)),
            dack: core::array::from_fn(|_| Mutex::with_rank(LockRank::LEAF, None)),
            eop: Mutex::with_rank(LockRank::LEAF, None),
        });
        let regs: RegionRef = Arc::new(Region::io(
            "pc.dma.regs",
            cfg.window(),
            Arc::new(ControlBlock {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let pages: RegionRef = Arc::new(Region::io(
            "pc.dma.pages",
            PAGE_WINDOW_LEN,
            Arc::new(PageLatches {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Ok(Dma8237 {
            shared,
            regs,
            pages,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    /// Connect the address space this controller's transfers traverse.
    ///
    /// The machine layer calls this from [`Instance::bind`]; a caller wiring a
    /// board by hand calls it directly.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        let mut state = self.shared.state.lock();
        state.bus = Some(Arc::downgrade(space));
        state.requester = requester;
    }

    /// Whether this is the 16-bit controller.
    #[must_use]
    pub fn is_word(&self) -> bool {
        self.shared.cfg.word
    }

    /// The number of this controller's first channel.
    #[must_use]
    pub fn channel_base(&self) -> u8 {
        self.shared.cfg.base
    }

    /// The local index of absolute channel `channel`, if it is one of ours.
    fn index_of(&self, channel: u8) -> Option<usize> {
        let index = channel.checked_sub(self.shared.cfg.base)?;
        (usize::from(index) < CHANNELS).then_some(usize::from(index))
    }

    /// The local channel a pin named `<prefix><n>` refers to.
    fn pin_index(&self, port: &str, prefix: &str) -> Option<usize> {
        let number: u8 = port.strip_prefix(prefix)?.parse().ok()?;
        self.index_of(number)
    }

    /// Assert or deassert `DREQ` on absolute channel `channel` directly.
    ///
    /// What the pin does, without a wire. `asserted` is the logical request,
    /// already resolved against the command register's `DREQ` sense bit — a
    /// caller with a wire lets [`Device::sink`] do that instead.
    pub fn request(&self, channel: u8, asserted: bool) {
        let Some(index) = self.index_of(channel) else {
            return;
        };
        {
            let mut state = self.shared.state.lock();
            Shared::record_request(&mut state, index, asserted);
        }
        if asserted {
            self.shared.service(index);
        }
    }

    /// The current address register of absolute channel `channel`.
    #[must_use]
    pub fn address(&self, channel: u8) -> Option<u16> {
        let index = self.index_of(channel)?;
        Some(self.shared.state.lock().ch[index].addr)
    }

    /// The current word count of absolute channel `channel`.
    #[must_use]
    pub fn count(&self, channel: u8) -> Option<u16> {
        let index = self.index_of(channel)?;
        Some(self.shared.state.lock().ch[index].count)
    }

    /// Whether absolute channel `channel` is masked.
    #[must_use]
    pub fn masked(&self, channel: u8) -> Option<bool> {
        let index = self.index_of(channel)?;
        Some(self.shared.state.lock().ch[index].masked)
    }
}

/// The `pc.dma` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Intel 8237A DMA controller, with the PC's page-register latches",
    properties: &[
        PropertySpec {
            name: "mode",
            kind: ValueKind::Str,
            required: false,
            summary: "\"byte\" for the 8-bit controller, \"word\" for the 16-bit one \
                      (default \"byte\")",
        },
        PropertySpec {
            name: "base",
            kind: ValueKind::Uint,
            required: false,
            summary: "the number of the first channel, 0 or 4 (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Dma8237::new(props)?)),
};

impl Device for Dma8237 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. The regions are placed by `map` statements and the
        // space arrives at bind time, which runs after every region is mapped —
        // the ordering a bus master needs.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            state.master_clear();
            if kind == ResetKind::Cold {
                // The data sheet leaves the address and count registers
                // undefined until software programs them. Zero is the
                // deterministic choice, and determinism is not optional
                // (`CLAUDE.md`). The latches are the board's, and a power-on
                // clears those too.
                for ch in &mut state.ch {
                    *ch = Channel {
                        masked: true,
                        ..Channel::default()
                    };
                }
                state.pages = [0; 16];
            }
        }
        for index in 0..CHANNELS {
            self.shared.drive_dack(index, false);
        }
        let out = self.shared.eop.lock().clone();
        if let Some(out) = out {
            out.set(Level::High);
        }
    }

    /// `regs` (and `""`) is the 8237's control block; `pages` is the board's
    /// latch file, which the AT decodes 0x80 away from it.
    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "pages" => Some(Arc::clone(&self.pages)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let index = self.pin_index(port, "dreq")?;
        let pin = Arc::new(RequestPin {
            shared: Arc::clone(&self.shared),
            index,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn attach_dma_peripheral(&self, port: &str, peer: Weak<dyn DmaPeripheral>) {
        if let Some(index) = self.pin_index(port, "dreq") {
            *self.shared.peers[index].lock() = Some(peer);
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port == "eop" {
            *self.shared.eop.lock() = Some(source);
            return Ok(());
        }
        match self.pin_index(port, "dack") {
            Some(index) => {
                *self.shared.dack[index].lock() = Some(source);
                Ok(())
            }
            None => Err(Error::Config {
                at: port.to_string(),
                message: format!(
                    "an 8237A drives `dack{}`..`dack{}` and `eop`",
                    self.shared.cfg.base,
                    usize::from(self.shared.cfg.base) + CHANNELS - 1
                ),
            }),
        }
    }

    fn announce(&self, port: &str) {
        // Both outputs idle deasserted, and `/EOP` idles *high*, so a fresh net
        // — which starts low — has to be told.
        if port == "eop" {
            let out = self.shared.eop.lock().clone();
            if let Some(out) = out {
                out.set(Level::High);
            }
        } else if let Some(index) = self.pin_index(port, "dack") {
            self.shared.drive_dack(index, false);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        for ch in &state.ch {
            w.write_u16(ch.addr)?;
            w.write_u16(ch.base_addr)?;
            w.write_u16(ch.count)?;
            w.write_u16(ch.base_count)?;
            w.write_u8(ch.mode)?;
            w.write_bool(ch.masked)?;
        }
        w.write_u8(state.command)?;
        w.write_u8(state.tc)?;
        w.write_u8(state.request)?;
        w.write_bool(state.flipflop)?;
        w.write_u8(state.rotate)?;
        w.write_u8(state.temp)?;
        w.write_all(&state.pages)
        // Neither the attached peripherals nor the address space appear: they
        // are wiring, not state. Nor does `drq` — that is a net's level, and the
        // net restores it.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut ch = [Channel::default(); CHANNELS];
        for slot in &mut ch {
            slot.addr = r.read_u16()?;
            slot.base_addr = r.read_u16()?;
            slot.count = r.read_u16()?;
            slot.base_count = r.read_u16()?;
            slot.mode = r.read_u8()?;
            slot.masked = r.read_bool()?;
        }
        let command = r.read_u8()?;
        let tc = r.read_u8()?;
        let request = r.read_u8()?;
        let flipflop = r.read_bool()?;
        let rotate = r.read_u8()?;
        if usize::from(rotate) >= CHANNELS {
            return Err(Error::State(format!(
                "rotating priority position {rotate} is not a channel of a \
                 {CHANNELS}-channel controller"
            )));
        }
        let temp = r.read_u8()?;
        let mut pages = [0u8; 16];
        for byte in &mut pages {
            *byte = r.read_u8()?;
        }
        let mut state = self.shared.state.lock();
        state.ch = ch;
        state.command = command;
        state.tc = tc;
        state.request = request;
        state.flipflop = flipflop;
        state.rotate = rotate;
        state.temp = temp;
        state.pages = pages;
        Ok(())
    }
}

/// The machine layer's half: a DMA controller is a bus master.
impl Instance for Dma8237 {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "an 8237A masters the bus it transfers across: add `space = mem` to the \
                 object that declares it",
            ),
        })?;
        self.attach_bus(space, ctx.requester());
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Dma8237::new(props)?)))
}

/// What the validator should know about `pc.dma`.
///
/// One class covers both controllers, so all eight channels' pins are declared
/// here; which four an instance answers to depends on its `base`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("mode", ValueKind::Str).values(&["byte", "word"]))
        .prop(PropSchema::new("base", ValueKind::Uint).range(0, 4))
        .region("")
        .region("regs")
        .region("pages")
        .port("eop", PortDir::Out);
    for channel in 0..8u8 {
        schema = schema
            .port(format!("dreq{channel}"), PortDir::In)
            .port(format!("dack{channel}"), PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RamStore;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::collections::VecDeque;

    /// Where the test board's RAM ends. A megabyte reaches page 7.
    const RAM_LEN: u64 = 0x10_0000;

    /// A peripheral that hands out a queue of bytes and records what it is
    /// given.
    #[derive(Debug)]
    struct Peer {
        supply: Mutex<VecDeque<u8>>,
        taken: Mutex<Vec<u8>>,
        terminals: Mutex<u32>,
        served: Mutex<u32>,
        ready: bool,
    }

    impl Peer {
        fn new(ready: bool) -> Peer {
            Peer {
                supply: Mutex::with_rank(LockRank::LEAF, VecDeque::new()),
                taken: Mutex::with_rank(LockRank::LEAF, Vec::new()),
                terminals: Mutex::with_rank(LockRank::LEAF, 0),
                served: Mutex::with_rank(LockRank::LEAF, 0),
                ready,
            }
        }

        /// A peripheral that drops its request after every unit — a floppy
        /// controller between sectors, say. It is what makes a *single* unit
        /// observable, now that a level-held request is serviced until the
        /// peripheral says stop.
        fn supplying_one_at_a_time(bytes: &[u8]) -> Peer {
            let peer = Peer::new(false);
            peer.supply.lock().extend(bytes.iter().copied());
            peer
        }

        fn supplying(bytes: &[u8]) -> Peer {
            let peer = Peer::new(true);
            peer.supply.lock().extend(bytes.iter().copied());
            peer
        }
    }

    impl DmaPeripheral for Peer {
        fn dma_read(&self, terminal: bool) -> u8 {
            *self.served.lock() += 1;
            if terminal {
                *self.terminals.lock() += 1;
            }
            self.supply.lock().pop_front().unwrap_or(0xee)
        }

        fn dma_write(&self, byte: u8, terminal: bool) {
            *self.served.lock() += 1;
            if terminal {
                *self.terminals.lock() += 1;
            }
            self.taken.lock().push(byte);
        }

        fn dma_ready(&self) -> bool {
            self.ready
        }
    }

    /// A controller with a megabyte of RAM under it and a peripheral on one
    /// channel.
    struct Rig {
        space: Arc<AddressSpace>,
        dma: Dma8237,
        regs: ControlBlock,
        pages: PageLatches,
        peer: Arc<Peer>,
        channel: u8,
    }

    impl Rig {
        fn new(word: bool, base: u8, channel: u8, peer: Peer) -> Rig {
            let space = Arc::new(AddressSpace::new("mem", 24));
            {
                let mut topo = space.topology();
                topo.map(
                    Arc::new(Region::ram("ram", Arc::new(RamStore::new(RAM_LEN)))),
                    0,
                )
                .expect("maps");
            }
            let dma = Dma8237::with_config(word, base).expect("a legal controller");
            dma.attach_bus(&space, RequesterId::ANONYMOUS);
            let peer = Arc::new(peer);
            dma.attach_dma_peripheral(
                &format!("dreq{channel}"),
                Arc::downgrade(&peer) as Weak<dyn DmaPeripheral>,
            );
            let regs = ControlBlock {
                shared: Arc::clone(&dma.shared),
            };
            let pages = PageLatches {
                shared: Arc::clone(&dma.shared),
            };
            Rig {
                space,
                dma,
                regs,
                pages,
                peer,
                channel,
            }
        }

        fn byte(channel: u8, peer: Peer) -> Rig {
            Rig::new(false, 0, channel, peer)
        }

        /// Write control-block register `index`, at whatever spacing this
        /// controller uses.
        fn poke(&self, index: u64, value: u8) {
            let offset = if self.dma.is_word() { index * 2 } else { index };
            self.regs
                .write(offset, &[value], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        fn peek(&self, index: u64) -> u8 {
            self.peek_attrs(index, MemAttrs::DEFAULT)
        }

        fn peek_attrs(&self, index: u64, attrs: MemAttrs) -> u8 {
            let offset = if self.dma.is_word() { index * 2 } else { index };
            let mut byte = [0u8; 1];
            self.regs
                .read(offset, &mut byte, attrs)
                .expect("a byte read is legal");
            byte[0]
        }

        fn set_page(&self, page: u8) {
            let offset = u64::from(PAGE_OFFSET[usize::from(self.channel)]);
            self.pages
                .write(offset, &[page], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        /// Program the channel: page, 16-bit address, 16-bit count, mode — in
        /// the order a driver programs them, and then unmask.
        ///
        /// Masking first is not decoration: a channel whose `DREQ` is still
        /// asserted would otherwise start a burst the moment the mode register
        /// is written, halfway through being reprogrammed.
        fn program(&self, page: u8, addr: u16, count: u16, mode: u8) {
            let local = u64::from(self.channel - self.dma.channel_base());
            self.poke(0xa, 0x04 | (local as u8));
            self.set_page(page);
            self.poke(0xc, 0); // clear the byte pointer first, as software does
            self.poke(local * 2, addr as u8);
            self.poke(local * 2, (addr >> 8) as u8);
            self.poke(local * 2 + 1, count as u8);
            self.poke(local * 2 + 1, (count >> 8) as u8);
            self.poke(0xb, mode | (local as u8));
            self.poke(0xa, local as u8); // unmask: bit 2 clear
        }

        fn ram(&self, addr: u64) -> u8 {
            self.space
                .read(addr, Width::U8, MemAttrs::DEFAULT)
                .expect("mapped") as u8
        }

        fn set_ram(&self, addr: u64, value: u8) {
            self.space
                .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
                .expect("mapped");
        }
    }

    /// A mode byte: transfer type, service mode, and the flags.
    fn mode(xfer: u8, select: u8, flags: u8) -> u8 {
        (xfer << 2) | (select << 6) | flags
    }

    #[test]
    fn two_writes_through_the_flip_flop_make_one_sixteen_bit_address() {
        let rig = Rig::byte(1, Peer::new(true));
        rig.poke(0xc, 0);
        rig.poke(2, 0x34);
        rig.poke(2, 0x12);
        assert_eq!(rig.dma.address(1), Some(0x1234));
        // And it reads back in the same order.
        rig.poke(0xc, 0);
        assert_eq!(rig.peek(2), 0x34);
        assert_eq!(rig.peek(2), 0x12);

        // Clearing the flip-flop mid-register puts the next access back on the
        // low half, which is the whole reason software writes 0xc first.
        rig.poke(2, 0xff);
        rig.poke(0xc, 0);
        rig.poke(2, 0x78);
        rig.poke(2, 0x56);
        assert_eq!(rig.dma.address(1), Some(0x5678));
    }

    #[test]
    fn a_write_to_memory_transfer_lands_at_the_page_shifted_up_by_sixteen() {
        let rig = Rig::byte(2, Peer::supplying(&[0xde, 0xad, 0xbe, 0xef]));
        rig.program(0x02, 0x1234, 3, mode(XFER_WRITE, SELECT_BLOCK, 0));
        rig.dma.request(2, true);

        for (i, expected) in [0xde, 0xad, 0xbe, 0xefu8].into_iter().enumerate() {
            assert_eq!(rig.ram(0x0002_1234 + i as u64), expected, "byte {i}");
        }
        assert_eq!(*rig.peer.terminals.lock(), 1, "TC on the last byte only");
    }

    #[test]
    fn a_read_from_memory_transfer_hands_the_bytes_to_the_peripheral() {
        let rig = Rig::byte(3, Peer::new(true));
        for (i, value) in [1u8, 2, 3, 4].into_iter().enumerate() {
            rig.set_ram(0x0003_0100 + i as u64, value);
        }
        rig.program(0x03, 0x0100, 3, mode(XFER_READ, SELECT_BLOCK, 0));
        rig.dma.request(3, true);

        assert_eq!(*rig.peer.taken.lock(), alloc::vec![1, 2, 3, 4]);
        assert_eq!(*rig.peer.terminals.lock(), 1);
    }

    #[test]
    fn the_address_counts_down_when_the_mode_says_so() {
        let rig = Rig::byte(1, Peer::supplying(&[0xaa, 0xbb, 0xcc]));
        rig.program(
            0x01,
            0x0200,
            2,
            mode(XFER_WRITE, SELECT_BLOCK, MODE_DECREMENT),
        );
        rig.dma.request(1, true);

        assert_eq!(rig.ram(0x0001_0200), 0xaa);
        assert_eq!(rig.ram(0x0001_01ff), 0xbb);
        assert_eq!(rig.ram(0x0001_01fe), 0xcc);
    }

    #[test]
    fn terminal_count_sets_a_status_bit_masks_the_channel_and_the_read_clears_it() {
        let rig = Rig::byte(2, Peer::supplying(&[0x11]));
        rig.program(0x00, 0x0040, 0, mode(XFER_WRITE, SELECT_SINGLE, 0));
        rig.dma.request(2, true);

        assert_eq!(rig.ram(0x40), 0x11);
        assert_eq!(rig.dma.masked(2), Some(true), "TC masks without autoinit");

        // A debug read shows the bit and leaves it exactly where it was.
        assert_eq!(rig.peek_attrs(8, MemAttrs::DEBUG) & 0x0f, 0x04);
        assert_eq!(rig.peek_attrs(8, MemAttrs::DEBUG) & 0x0f, 0x04);
        // A real read shows it once.
        assert_eq!(rig.peek(8) & 0x0f, 0x04);
        assert_eq!(rig.peek(8) & 0x0f, 0x00, "reading the status clears it");
    }

    #[test]
    fn autoinitialise_reloads_the_base_registers_and_leaves_the_channel_open() {
        // A peripheral that drops its request between units, so the reload is
        // observable between them rather than being run straight through.
        let rig = Rig::byte(1, Peer::supplying_one_at_a_time(&[0x01, 0x02]));
        rig.program(
            0x00,
            0x0300,
            0,
            mode(XFER_WRITE, SELECT_SINGLE, MODE_AUTOINIT),
        );
        rig.dma.request(1, true);
        assert_eq!(rig.ram(0x300), 0x01);
        assert_eq!(rig.dma.masked(1), Some(false), "autoinit does not mask");
        assert_eq!(rig.dma.address(1), Some(0x0300), "address reloaded");
        assert_eq!(rig.dma.count(1), Some(0), "count reloaded");

        rig.dma.request(1, true);
        assert_eq!(rig.ram(0x300), 0x02, "and it starts over");
    }

    #[test]
    fn a_masked_channel_does_not_transfer() {
        let rig = Rig::byte(2, Peer::supplying(&[0x77]));
        rig.program(0x00, 0x0500, 0, mode(XFER_WRITE, SELECT_SINGLE, 0));
        rig.poke(0xa, 0x04 | 2); // mask channel 2 again
        rig.dma.request(2, true);
        assert_eq!(rig.ram(0x500), 0x00);
        assert_eq!(*rig.peer.served.lock(), 0);

        // Clearing the mask register while the request is still asserted is
        // what actually starts it.
        rig.poke(0xe, 0);
        assert_eq!(rig.ram(0x500), 0x77);
    }

    #[test]
    fn the_page_table_is_the_ats_wiring_and_not_a_numeric_one() {
        assert_eq!(PAGE_OFFSET[2], 0x1, "channel 2's page latch is port 0x81");
        assert_eq!(PAGE_OFFSET[0], 0x7);
        assert_eq!(PAGE_OFFSET[1], 0x3);
        assert_eq!(PAGE_OFFSET[3], 0x2);

        // And a transfer really reads that latch: write the page at offset 1
        // and channel 2's bytes appear in page 5.
        let rig = Rig::byte(2, Peer::supplying(&[0x5a]));
        rig.pages
            .write(0x1, &[0x05], MemAttrs::DEFAULT)
            .expect("a latch takes a byte");
        rig.poke(0xc, 0);
        rig.poke(4, 0x00);
        rig.poke(4, 0x00);
        rig.poke(5, 0x00);
        rig.poke(5, 0x00);
        rig.poke(0xb, mode(XFER_WRITE, SELECT_SINGLE, 0) | 2);
        rig.poke(0xa, 2);
        rig.dma.request(2, true);
        assert_eq!(rig.ram(0x0005_0000), 0x5a);

        let mut byte = [0u8; 1];
        rig.pages
            .read(0x1, &mut byte, MemAttrs::DEFAULT)
            .expect("a latch reads back");
        assert_eq!(byte[0], 0x05);
    }

    #[test]
    fn the_word_controller_moves_two_bytes_and_shifts_its_address_left() {
        let rig = Rig::new(true, 4, 6, Peer::supplying(&[0x21, 0x43, 0x65, 0x87]));
        // Address 0x1000 in *words* is 0x2000 in bytes, under page 2.
        rig.program(0x02, 0x1000, 1, mode(XFER_WRITE, SELECT_BLOCK, 0));
        rig.dma.request(6, true);

        assert_eq!(rig.ram(0x0002_2000), 0x21);
        assert_eq!(rig.ram(0x0002_2001), 0x43);
        assert_eq!(rig.ram(0x0002_2002), 0x65);
        assert_eq!(rig.ram(0x0002_2003), 0x87);
        assert_eq!(*rig.peer.served.lock(), 4, "four bytes, two requests");
        assert_eq!(*rig.peer.terminals.lock(), 1, "TC on the last byte only");
        // The address register counts words, so two units advanced it by two.
        assert_eq!(rig.dma.address(6), Some(0x1002));
    }

    #[test]
    fn a_word_controllers_registers_answer_only_at_even_offsets() {
        let rig = Rig::new(true, 4, 5, Peer::new(true));
        let mut byte = [0u8; 1];
        rig.regs
            .read(0x1, &mut byte, MemAttrs::DEFAULT)
            .expect("the access completes");
        assert_eq!(byte[0], 0xff, "nothing is decoded at an odd offset");
        assert_eq!(rig.dma.regs.len(), WORD_REGISTER_WINDOW_LEN);
    }

    #[test]
    fn channel_four_is_the_cascade_and_never_transfers() {
        let rig = Rig::new(true, 4, 4, Peer::supplying(&[0x99]));
        rig.program(0x00, 0x0000, 0, mode(XFER_WRITE, SELECT_BLOCK, 0));
        rig.dma.request(4, true);
        assert_eq!(*rig.peer.served.lock(), 0);
    }

    #[test]
    fn a_burst_against_a_peripheral_that_never_stops_asking_hits_the_bound() {
        // Autoinitialise means no terminal count ever masks the channel, and
        // the peripheral is always ready: on real hardware this runs until the
        // guest intervenes. Here it stops at MAX_BURST_UNITS rather than
        // hanging the machine inside one wire event.
        let rig = Rig::byte(0, Peer::new(true));
        rig.program(
            0x00,
            0x0000,
            0x00ff,
            mode(XFER_WRITE, SELECT_BLOCK, MODE_AUTOINIT),
        );
        rig.dma.request(0, true);
        assert_eq!(*rig.peer.served.lock(), MAX_BURST_UNITS);
    }

    #[test]
    fn a_burst_lasts_as_long_as_the_peripheral_keeps_asking() {
        // A peripheral that drops `DREQ` after each unit gets one unit per
        // request, which is what "single transfer mode" is usually taken to
        // mean...
        let rig = Rig::byte(1, Peer::supplying_one_at_a_time(&[0x01, 0x02, 0x03]));
        rig.program(0x00, 0x0700, 0x00ff, mode(XFER_WRITE, SELECT_SINGLE, 0));
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 1);
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 2);
        assert_eq!(rig.ram(0x700), 0x01);
        assert_eq!(rig.ram(0x701), 0x02);

        // ...and one that holds the line, like a floppy controller with a
        // sector to deliver, is served until its count runs out. On silicon
        // that is many arbitrations rather than one; there are no bus cycles
        // here to tell them apart, and a controller that stopped after one unit
        // would simply never finish, because a held level delivers no second
        // notification. See `service`.
        let held = Rig::byte(1, Peer::supplying(&[0xaa, 0xbb, 0xcc, 0xdd]));
        held.program(0x00, 0x0700, 3, mode(XFER_WRITE, SELECT_SINGLE, 0));
        held.dma.request(1, true);
        assert_eq!(*held.peer.served.lock(), 4, "the whole programmed count");
        assert_eq!(held.ram(0x703), 0xdd);
        assert_eq!(held.dma.masked(1), Some(true), "and it reached TC");
    }

    #[test]
    fn a_verify_transfer_counts_without_moving_anything() {
        let rig = Rig::byte(2, Peer::supplying(&[0x42, 0x43]));
        rig.program(0x00, 0x0800, 1, mode(XFER_VERIFY, SELECT_BLOCK, 0));
        rig.dma.request(2, true);
        assert_eq!(rig.ram(0x800), 0x00, "verify moves no data");
        assert_eq!(*rig.peer.served.lock(), 0, "and touches no data pin");
        assert_eq!(rig.dma.masked(2), Some(true), "but it still reached TC");
    }

    #[test]
    fn an_illegal_transfer_type_and_a_disabled_controller_both_refuse() {
        let rig = Rig::byte(1, Peer::supplying(&[0x01]));
        rig.program(0x00, 0x0900, 0, mode(XFER_ILLEGAL, SELECT_SINGLE, 0));
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 0, "the fourth encoding is illegal");
        rig.dma.request(1, false);

        // A legal mode this time, but the controller is switched off.
        rig.poke(8, CMD_DISABLE);
        rig.program(0x00, 0x0900, 0, mode(XFER_WRITE, SELECT_SINGLE, 0));
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 0);
        rig.dma.request(1, false);

        rig.poke(8, 0);
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 1);
        assert_eq!(rig.ram(0x900), 0x01);
    }

    #[test]
    fn a_cascade_channel_moves_nothing_of_its_own() {
        let rig = Rig::byte(3, Peer::supplying(&[0x01]));
        rig.program(0x00, 0x0a00, 0, mode(XFER_WRITE, SELECT_CASCADE, 0));
        rig.dma.request(3, true);
        assert_eq!(*rig.peer.served.lock(), 0);
    }

    #[test]
    fn a_debug_write_is_refused_rather_than_made_harmless() {
        let rig = Rig::byte(1, Peer::new(true));
        assert!(rig.regs.write(0xb, &[0x45], MemAttrs::DEBUG).is_err());
        assert!(rig.pages.write(0x3, &[0x01], MemAttrs::DEBUG).is_err());
        // And a debug read of an address register leaves the flip-flop alone.
        rig.poke(0xc, 0);
        rig.poke(2, 0xcd);
        rig.poke(2, 0xab);
        rig.poke(0xc, 0);
        assert_eq!(rig.peek_attrs(2, MemAttrs::DEBUG), 0xcd);
        assert_eq!(
            rig.peek_attrs(2, MemAttrs::DEBUG),
            0xcd,
            "still the low half"
        );
        assert_eq!(rig.peek(2), 0xcd);
        assert_eq!(rig.peek(2), 0xab, "a real read did advance it");
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let rig = Rig::byte(0, Peer::new(true));
        assert!(rig.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(rig.regs.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
        assert!(rig.pages.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_master_clear_masks_every_channel_and_clears_the_byte_pointer() {
        let rig = Rig::byte(1, Peer::new(true));
        rig.program(0x00, 0x0a00, 5, mode(XFER_WRITE, SELECT_BLOCK, 0));
        rig.poke(2, 0x11); // leave the flip-flop on the high half
        rig.poke(0xd, 0); // master clear
        assert_eq!(rig.dma.masked(1), Some(true));
        // The address and count survive, as the data sheet says they do.
        assert_eq!(rig.dma.count(1), Some(5));
        rig.poke(2, 0x22);
        assert_eq!(
            rig.dma.address(1).map(|a| a & 0xff),
            Some(0x22),
            "the byte pointer came back to the low half"
        );
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let word = Dma8237::new(&Props::new().with("mode", "word").with("base", 4u64))
            .expect("the AT's second controller");
        assert!(word.is_word());
        assert_eq!(word.channel_base(), 4);
        assert!(Dma8237::new(&Props::new().with("mode", "nibble")).is_err());
        assert!(Dma8237::new(&Props::new().with("base", 2u64)).is_err());
        assert!(Dma8237::new(&Props::new().with("bass", 4u64)).is_err());
        let default = Dma8237::new(&Props::new()).expect("no properties is legal");
        assert!(!default.is_word());
        assert_eq!(default.channel_base(), 0);
    }

    #[test]
    fn the_pins_are_the_ones_this_controller_owns() {
        let second = Dma8237::with_config(true, 4).expect("legal");
        assert!(second.sink("dreq5", &[]).is_some());
        assert!(second.sink("dreq1", &[]).is_none(), "not its channel");
        assert!(second.sink("dack5", &[]).is_none(), "an output, not a sink");
        assert!(second.region("pages").is_some());
        assert!(second.region("nothing").is_none());
    }

    #[test]
    fn state_round_trips_byte_for_byte() {
        fn image(dev: &Dma8237) -> Vec<u8> {
            let mut shape = MachineShape::new();
            shape.add_device("dma", CLASS_NAME).expect("unique path");
            let mut writer = StateWriter::new(shape);
            {
                let mut chunk = writer
                    .chunk("dma", CLASS_NAME, STATE_VERSION)
                    .expect("one chunk");
                dev.save(&mut chunk).expect("saves");
            }
            writer.to_vec().expect("encodes")
        }

        let rig = Rig::byte(2, Peer::supplying(&[0x10, 0x20]));
        rig.program(
            0x02,
            0x4321,
            1,
            mode(XFER_READ, SELECT_SINGLE, MODE_AUTOINIT),
        );
        rig.dma.request(2, true);
        rig.dma.request(2, false);
        // Leave a few odd corners set: a command byte, a software request, and
        // a half-written address register.
        rig.poke(8, CMD_ROTATING | CMD_DREQ_ACTIVE_LOW);
        rig.poke(9, 0x04 | 1);
        rig.poke(0xc, 0);
        rig.poke(6, 0x99);

        let saved = image(&rig.dma);

        let restored = Dma8237::with_config(false, 0).expect("legal");
        let reader = StateReader::new(&saved).expect("decodes");
        let chunk = reader
            .load("dma", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("finds the chunk");
        restored.load(&mut chunk.reader()).expect("loads");

        assert_eq!(image(&restored), saved, "the two images must be identical");
        assert_eq!(restored.address(2), rig.dma.address(2));
        assert_eq!(restored.count(2), rig.dma.count(2));
        assert_eq!(restored.masked(2), rig.dma.masked(2));
    }

    #[test]
    fn a_cold_reset_returns_every_register_to_a_documented_value() {
        let rig = Rig::byte(1, Peer::new(true));
        rig.program(0x07, 0x1111, 4, mode(XFER_WRITE, SELECT_BLOCK, 0));
        rig.dma.reset(ResetKind::Cold);
        assert_eq!(rig.dma.address(1), Some(0));
        assert_eq!(rig.dma.count(1), Some(0));
        assert_eq!(rig.dma.masked(1), Some(true));
        let mut byte = [0u8; 1];
        rig.pages
            .read(u64::from(PAGE_OFFSET[1]), &mut byte, MemAttrs::DEFAULT)
            .expect("reads");
        assert_eq!(byte[0], 0, "the board's latches go too");
        // The bus handle is wiring, not state, and survives.
        rig.program(0x00, 0x0b00, 0, mode(XFER_WRITE, SELECT_SINGLE, 0));
        rig.dma.request(1, true);
        assert_eq!(*rig.peer.served.lock(), 1);
    }
}
