//! The STM32 DMA controllers: one transfer engine behind two register faces.
//!
//! # Sources
//!
//! * ST **RM0090** rev 21 (STM32F405/415, F407/417, F427/437, F429/439), §10
//!   — the **stream-based** controller of the F2/F4/F7: eight streams,
//!   `LISR`/`HISR`/`LIFCR`/`HIFCR`, `SxCR`/`SxNDTR`/`SxPAR`/`SxM0AR`/`SxM1AR`
//!   /`SxFCR`, the software/hardware priority pair of §10.3.2, the
//!   double-buffer mode of §10.3.10 and the memory-to-memory mode of §10.3.11.
//! * ST **RM0351** rev 9 (STM32L4x5/L4x6), §11 — the **channel-based**
//!   controller of the F0/F1/F3/L0/L1/L4/G0/G4/WB: seven channels,
//!   `ISR`/`IFCR`, `CCRx`/`CNDTRx`/`CPARx`/`CMARx`, `CSELR`, and Table 41's
//!   programmable-data-width behaviour.
//!
//! **No emulator source of any licence was consulted** (`CLAUDE.md`,
//! provenance). Every bit position, every reset value and every rule below is
//! from one of those two manuals.
//!
//! # One engine, two faces
//!
//! A stream and a channel are the same machine with different paperwork. Both
//! hold a peripheral address, a memory address and a count; both step the two
//! addresses by their own programmed width, convert between the widths, raise a
//! half-transfer flag at the midpoint and a transfer-complete flag at zero, and
//! reload in circular mode. What differs is where the bits sit, how many units
//! there are, how the flags are packed into the status word, and which extras
//! exist — `MEM2MEM` and `CSELR` on the channel controller, a FIFO, bursts,
//! `PFCTRL` and double buffering on the stream one.
//!
//! So [`Variant`] selects the register face and the unit count, and everything
//! below the decode is shared. A model that forked the engine as well would
//! have two places for every transfer bug to hide.
//!
//! ```text
//!   variant = "stream"    8 streams   window 0xd0   pins irq0..irq7, req0..req7
//!   variant = "channel"   7 channels  window 0xac   pins irq1..irq7, req1..req7
//! ```
//!
//! Each `req` pin also has a selector-qualified form, `req{n}c{sel}`, which is
//! the other half of RM0090 Table 43 — see "`CHSEL`" below.
//!
//! The pin numbers follow the **manual's** numbering, which is why the channel
//! face starts at one: RM0351 calls them `DMA1_Channel1`..`DMA1_Channel7` and a
//! board file should be able to say what the reference manual says.
//!
//! # A transfer takes time
//!
//! A real controller moves one beat per bus cycle and arbitrates between units;
//! it does not empty a buffer inside the store that set `EN`. So this device is
//! **runnable**, and one tick of its own clock domain is one beat — the same
//! idiom [`crate::dev::stm32::usart`] uses for a character time. A board picks
//! the rate by picking the domain:
//!
//! ```text
//!   object dma2 "st.dma" { clock = hse * 21 / 4, space = mem, variant = "stream" }
//! ```
//!
//! is a beat every four AHB cycles. The alternative — transferring inside the
//! MMIO write that enables the stream — is not a simplification but a wrong
//! answer: firmware that starts a DMA and then polls a peripheral flag, or that
//! counts on `NDTR` decreasing while it does something else, sees a machine no
//! silicon behaves like. Advancing in the scheduler's quantum also keeps the
//! ordering against the CPU deterministic: a beat never lands mid-instruction.
//!
//! Within one call the units are served in **RM0090 §10.3.2 order** — software
//! priority `PL` first, then the lower unit number — one beat at a time, so two
//! streams at the same priority interleave rather than one starving the other.
//!
//! # What a requesting peripheral does
//!
//! On the silicon the peripheral has no data path to the controller at all. It
//! raises a request line; the controller then performs an **ordinary bus
//! access at `CPAR`**, which is the peripheral's own data register, and it is
//! that access which clears `TXE`/`RXNE` and makes the peripheral drop the
//! line. So the seam here is the line and nothing else, and a peripheral needs
//! no knowledge of DMA beyond driving it:
//!
//! * The controller exposes an **input pin per unit and per selector**. A
//!   board wires the peripheral's request output to the cell its part's
//!   request matrix puts it on — RM0090 Table 43, RM0351 Table 40 — exactly as
//!   it wires an interrupt to the core's numbered pin. The matrix is a fact
//!   about the *part*, so it lives in the board file and not in this device
//!   (see `machines/stm32f407.machine` on why `wire usart2.irq -> cpu.irq38`
//!   is written there).
//! * **A level of [`Level::High`] means "I want service".** Hold it while you
//!   have data (or room) and drop it when the controller's access to your data
//!   register has satisfied you: that is a FIFO-style peripheral and it gets
//!   continuous service. Pulse it high and low once per item and each pulse is
//!   latched and buys exactly one beat: that is a single-item peripheral. Both
//!   work, and nothing else needs deciding.
//! * A peripheral that would rather answer than drive may publish a
//!   [`DmaPeripheral`] on the same pin, whose [`dma_ready`](DmaPeripheral::dma_ready)
//!   is polled as the level. Its `dma_read`/`dma_write` are **not** used: the
//!   data goes over the bus at `CPAR`, as on the part.
//! * Nothing outward happens inside `set_level`. The request is recorded in an
//!   atomic and the beat happens later, in [`Device::run`] — so a peripheral
//!   may raise its line from inside its own register handler, with its own lock
//!   held, and cannot be re-entered by the controller.
//!
//! # `CHSEL`, and why a request pin carries a channel number
//!
//! RM0090 Table 43 is a **matrix**, not a list. Every request reaches a
//! particular stream *on a particular channel*, several requests share each
//! stream, and `SxCR.CHSEL` is how firmware says which of them this stream is
//! listening to. `TIM2_UP` is on DMA1 stream 1 channel 3 **and** on stream 7
//! channel 3; a guest driving a display out of stream 7 with `CHSEL = 5` must
//! not have TIM2's update event stealing its beats.
//!
//! So a request pin carries the channel:
//!
//! ```text
//!   req3         stream 3, any channel      — served whatever CHSEL reads
//!   req3c4       stream 3, channel 4        — served only while CHSEL == 4
//! ```
//!
//! and the same on the channel face, where the selector is this unit's nibble
//! of `CSELR` (RM0351 §11.6.7) and reaches sixteen values rather than eight.
//! The unnumbered pin is not a legacy form to be migrated away: it is what a
//! board writes when one peripheral is wired to one unit and the selector is
//! not part of what is being modelled, and a `mem2mem` unit needs no pin at
//! all.
//!
//! A pin may have **several drivers**, because a cell of the table routinely
//! holds several requests: `TIM2_CH2` and `TIM2_CH4` are both stream 6
//! channel 3, OR'd onto one line on the die. The pin resolves its drivers as a
//! wired-OR, so two `wire` statements into `req6c3` behave the way the silicon
//! does instead of the second one cancelling the first.
//!
//! # Who ends the transfer: `PFCTRL`
//!
//! Normally the *stream* is the flow controller — it counts `NDTR` down and
//! stops at zero. RM0090 §10.3.2 lets the **peripheral** be the flow
//! controller instead (`SxCR.PFCTRL`), which is how ST's own F4 SD driver arms
//! the SDIO streams: the card decides how much data there is, `NDTR` is only a
//! maximum, and the peripheral signals its last item.
//!
//! That signal is [`DmaPeripheral::dma_last`], asked of the unit's data-side
//! peer before each beat. It is **not** the `terminal` flag on
//! [`DmaPeripheral::dma_read`] — that one runs the other way, telling a
//! peripheral that the *controller's* count has expired, which is precisely
//! the case `PFCTRL` turns off. When it answers true the beat still happens
//! and then `TCIF` goes up and `EN` comes down, whatever `NDTR` is left.
//! `CIRC` and `DBM` get no say, both being combinations the manual forbids
//! alongside `PFCTRL`; a `NDTR` that reaches zero first also stops the stream,
//! per RM0090 §10.5.6, and that is firmware having under-programmed the
//! maximum. A peripheral that publishes no peer, or one whose `dma_last` is
//! the default `false`, never ends a flow-controlled transfer — which is what
//! arming `PFCTRL` against a peripheral that cannot flow-control does on the
//! part.
//!
//! # A bus master, and the re-entrancy contract
//!
//! The controller masters the space its object declares (`space = mem`), with
//! its own [`RequesterId`], and that is how a board expresses what RM0090
//! §2.3's bus matrix does: an F4's CCM is reachable by the core and **not** by
//! DMA, so a board that models the distinction gives this object a space in
//! which CCM is not mapped.
//!
//! Mastering the bus means calling into other devices, which is what
//! `CLAUDE.md`'s re-entrancy contract is about. The rule here is mechanical:
//!
//! * The register state is one [`Mutex`] at [`LockRank::DEVICE`]. It is taken
//!   to *plan* a beat and released; taken again to *commit* one and released.
//!   **It is never held across a bus access, a wire change, or a call into a
//!   peripheral.**
//! * The request latch is not a lock at all — one atomic per unit — so the
//!   inbound wire path takes nothing and can never deadlock against the
//!   outbound one.
//! * The interrupt outputs are at [`LockRank::WIRE`], which is above `DEVICE`;
//!   the handle is cloned out and the lock dropped before the level is driven.
//!
//! # What is modelled and what is not
//!
//! Modelled: both register faces; `EN` arming and the write protection that
//! comes with it; `PSIZE`/`MSIZE` with RM0351 Table 41's truncate-on-narrowing
//! and zero-extend-on-widening; `PINC`/`MINC` and the stream face's `PINCOS`;
//! `CIRC` reload; the stream face's `DBM`/`CT` double buffer; `MEM2MEM` and
//! stream `DIR=10`; **`CHSEL`/`CSELR` gating the request**; **`PFCTRL`, with
//! the peripheral ending the transfer**; `HTIF` at the midpoint, `TCIF` at
//! zero, `TEIF` on a bus fault and on an illegal configuration; per-unit
//! interrupt outputs; priority arbitration; the whole of the snapshot.
//!
//! **DMAMUX** (RM0432 §14), which the L4+/G4/H7/WB route requests through, is
//! a separate class, `st.dmamux`; `mux = true` on *this* object is still
//! refused, because a DMA controller multiplexing its own requests is not what
//! those parts do.
//!
//! Not modelled, and stored-and-inert rather than silently absent:
//!
//! * **The stream FIFO.** `SxFCR` round-trips, `FS` reads as empty and `FEIF`
//!   is never raised. `MBURST`/`PBURST` are stored. Direct mode and FIFO mode
//!   move the same bytes here; what a FIFO buys on the part is bus efficiency
//!   and a burst's timing, and neither is visible to firmware that is not
//!   measuring the bus.
//! * **`DMEIF`, the direct-mode error.** Nothing raises it.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{DmaPeripheral, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "st.dma";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
///
/// 2: the request latch became one bit per *selector slot* rather than one
/// boolean per unit, when `CHSEL` started gating (RM0090 Table 43).
const STATE_VERSION: u32 = 2;

/// The most units either face has — eight, the stream controller's.
pub const MAX_UNITS: usize = 8;

/// The most request selectors either face has.
///
/// Sixteen, from the channel face: `CSELR` gives each channel a **four-bit**
/// `CxS` field (RM0351 §11.6.7). The stream face's `CHSEL` is three bits, so
/// eight of these are never selected there. Bounds that come from a field
/// width rather than from a part's request table are the ones that stay true
/// across the family.
pub const MAX_SELECTORS: usize = 16;

/// Latch slots per unit: one per selector, plus slot zero.
///
/// **Slot zero is the unnumbered `req{n}` pin** — a request that arrives with
/// no channel attached and is therefore served whatever `CHSEL`/`CSELR` says.
/// It is what a board writes when it is wiring one peripheral to one unit and
/// does not care to model the selector, and what every test that predates the
/// gating uses.
const SLOTS: usize = MAX_SELECTORS + 1;

/// Beats one [`Device::run`] call will move however long the budget is.
///
/// A quantum with no event in it can be arbitrarily long, and a circular
/// memory-to-memory transfer never ends. The cap turns "this call runs
/// forever" into "this call consumes fewer ticks than it was offered", which
/// the scheduler already handles: the remaining ticks come back on the next
/// call. It never truncates a *finite* transfer, only defers it.
const MAX_BEATS_PER_RUN: u64 = 4096;

// ---------------------------------------------------------------------------
// which controller
// ---------------------------------------------------------------------------

/// Which register face an instance wears.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090 §10: eight streams, `LISR`/`HISR`, `SxCR`, a FIFO.
    Stream,
    /// RM0351 §11: seven channels, `ISR`, `CCRx`, `CSELR`.
    Channel,
}

impl Variant {
    /// How many transfer units this face has.
    #[must_use]
    pub const fn units(self) -> usize {
        match self {
            Variant::Stream => 8,
            Variant::Channel => 7,
        }
    }

    /// What a unit is called in its manual, and hence in a pin name: streams
    /// count from zero, channels from one.
    #[must_use]
    pub const fn first_pin(self) -> usize {
        match self {
            Variant::Stream => 0,
            Variant::Channel => 1,
        }
    }

    /// How many bytes of address space the register block decodes.
    #[must_use]
    pub const fn window(self) -> u64 {
        match self {
            // 0x10 for the four status registers, then 0x18 per stream.
            Variant::Stream => 0x10 + 0x18 * 8,
            // 0x08 for ISR/IFCR, 0x14 per channel, then CSELR at 0xa8.
            Variant::Channel => 0xac,
        }
    }
}

// ---------------------------------------------------------------------------
// register bits
// ---------------------------------------------------------------------------

/// `EN`, bit 0 of `SxCR` and of `CCRx` alike.
const CR_EN: u32 = 1 << 0;

// Stream `SxCR` (RM0090 §10.5.5).
const S_CR_DMEIE: u32 = 1 << 1;
const S_CR_TEIE: u32 = 1 << 2;
const S_CR_HTIE: u32 = 1 << 3;
const S_CR_TCIE: u32 = 1 << 4;
/// `PFCTRL`, bit 5: the **peripheral** is the flow controller, not the stream.
const S_CR_PFCTRL: u32 = 1 << 5;
const S_CR_CIRC: u32 = 1 << 8;
const S_CR_PINC: u32 = 1 << 9;
const S_CR_MINC: u32 = 1 << 10;
const S_CR_PINCOS: u32 = 1 << 15;
const S_CR_DBM: u32 = 1 << 18;
const S_CR_CT: u32 = 1 << 19;
/// The bits a write may change while the stream is enabled: `EN` itself and
/// the four interrupt enables (RM0090 §10.5.5 — the rest are write-protected).
const S_CR_LIVE: u32 = CR_EN | S_CR_DMEIE | S_CR_TEIE | S_CR_HTIE | S_CR_TCIE;
/// Bits 31..28 are reserved and read as zero.
const S_CR_MASK: u32 = 0x0fff_ffff;

// Channel `CCRx` (RM0351 §11.6.3).
const C_CR_TCIE: u32 = 1 << 1;
const C_CR_HTIE: u32 = 1 << 2;
const C_CR_TEIE: u32 = 1 << 3;
const C_CR_DIR: u32 = 1 << 4;
const C_CR_CIRC: u32 = 1 << 5;
const C_CR_PINC: u32 = 1 << 6;
const C_CR_MINC: u32 = 1 << 7;
const C_CR_MEM2MEM: u32 = 1 << 14;
const C_CR_LIVE: u32 = CR_EN | C_CR_TCIE | C_CR_HTIE | C_CR_TEIE;
/// Bits 31..15 are reserved and read as zero.
const C_CR_MASK: u32 = 0x0000_7fff;

/// `SxFCR`'s reset value: `FTH = 01`, `FS = 100` (empty) — RM0090 §10.5.10.
const S_FCR_RESET: u32 = 0x0000_0021;
/// The writable part of `SxFCR`: `FTH`, `DMDIS`, `FEIE`. `FS` is read-only.
const S_FCR_WRITABLE: u32 = 0b1000_0111;

// Normalised event flags, one byte per unit. The two faces pack these
// differently into their status words; the engine only ever sees these.
const F_TC: u8 = 1 << 0;
const F_HT: u8 = 1 << 1;
const F_TE: u8 = 1 << 2;
const F_DME: u8 = 1 << 3;
const F_FE: u8 = 1 << 4;
const F_ALL: u8 = F_TC | F_HT | F_TE | F_DME | F_FE;

/// Where stream `s`'s six flag bits start inside `LISR`/`HISR`
/// (RM0090 §10.5.1): streams 0 and 4 at bit 0, 1 and 5 at 6, 2 and 6 at 16,
/// 3 and 7 at 22.
const STREAM_FLAG_SHIFT: [u32; 4] = [0, 6, 16, 22];

// Bit offsets within one stream's group.
const S_FEIF: u32 = 0;
const S_DMEIF: u32 = 2;
const S_TEIF: u32 = 3;
const S_HTIF: u32 = 4;
const S_TCIF: u32 = 5;

// Bit offsets within one channel's nibble (RM0351 §11.6.1).
const C_GIF: u32 = 0;
const C_TCIF: u32 = 1;
const C_HTIF: u32 = 2;
const C_TEIF: u32 = 3;

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Which way a beat goes, once the face's encoding is decoded away.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    /// Read the peripheral port, write the memory port.
    PeriphToMem,
    /// Read the memory port, write the peripheral port.
    MemToPeriph,
}

/// One unit's programming, plus the pointers a running transfer walks.
#[derive(Debug, Clone, Copy)]
struct Unit {
    /// `SxCR` or `CCRx`, as written.
    cr: u32,
    /// `SxFCR`. Zero on the channel face, which has no FIFO.
    fcr: u32,
    /// `SxNDTR`/`CNDTRx` as programmed — the value `CIRC` reloads from.
    ndtr: u32,
    /// `SxPAR`/`CPARx`.
    par: u32,
    /// `SxM0AR`/`CMARx`.
    m0ar: u32,
    /// `SxM1AR`. Unused on the channel face.
    m1ar: u32,
    /// What `NDTR` reads back: the items still to move.
    cur_ndtr: u32,
    /// The peripheral-port pointer, stepped by `PINC`.
    cur_par: u32,
    /// The memory-port pointer, stepped by `MINC`.
    cur_mar: u32,
    /// `EN` was seen going 0→1 and the pointers are loaded.
    running: bool,
    /// `HTIF` has already been raised for this pass through the buffer.
    half_done: bool,
    /// The normalised event flags: `F_TC` and friends.
    flags: u8,
}

impl Unit {
    const fn reset(variant: Variant) -> Unit {
        Unit {
            cr: 0,
            fcr: match variant {
                Variant::Stream => S_FCR_RESET,
                Variant::Channel => 0,
            },
            ndtr: 0,
            par: 0,
            m0ar: 0,
            m1ar: 0,
            cur_ndtr: 0,
            cur_par: 0,
            cur_mar: 0,
            running: false,
            half_done: false,
            flags: 0,
        }
    }
}

/// Everything the register block owns.
#[derive(Debug, Clone)]
struct State {
    unit: [Unit; MAX_UNITS],
    /// `CSELR`, the channel face's request select. Stored, not acted on — see
    /// the module documentation.
    cselr: u32,
}

impl State {
    fn reset(variant: Variant) -> State {
        State {
            unit: [Unit::reset(variant); MAX_UNITS],
            cselr: 0,
        }
    }
}

/// A unit's configuration, decoded out of whichever face wrote it.
#[derive(Debug, Clone, Copy)]
struct Config {
    dir: Dir,
    /// No request line is needed: memory-to-memory.
    mem2mem: bool,
    circ: bool,
    dbm: bool,
    /// `CT`: which of `M0AR`/`M1AR` the memory port is currently on.
    ct: bool,
    pinc: bool,
    minc: bool,
    /// The width the peripheral port moves per beat.
    pw: Width,
    /// The width the memory port moves per beat.
    mw: Width,
    /// What `PINC` steps the peripheral pointer by — `PINCOS` forces four.
    pstep: u32,
    /// `PL`, the software priority. Higher wins.
    prio: u8,
    /// Which interrupt sources are enabled, as `F_*` bits.
    ie: u8,
    /// `CHSEL` (stream) or this unit's `CSELR` nibble (channel): which of the
    /// part's request lines this unit is listening to.
    sel: u8,
    /// `PFCTRL`: the peripheral ends the transfer, not the count.
    pfctrl: bool,
}

/// A unit that could move a beat this round, and what it needs to.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    unit: usize,
    /// False only for memory-to-memory, the one mode with no peripheral.
    needs_request: bool,
    /// `CHSEL`/`CSELR` as the unit currently reads it — which request lines
    /// wired to this unit it is listening to.
    sel: u8,
}

impl Candidate {
    const NONE: Candidate = Candidate {
        unit: 0,
        needs_request: false,
        sel: 0,
    };
}

/// A single beat, planned with the lock held and executed without it.
#[derive(Debug, Clone, Copy)]
struct Beat {
    src: u64,
    dst: u64,
    src_w: Width,
    dst_w: Width,
}

/// The width `PSIZE`/`MSIZE` code `code` selects, or `None` for the reserved
/// encoding `11`.
const fn size_width(code: u32) -> Option<Width> {
    match code {
        0 => Some(Width::U8),
        1 => Some(Width::U16),
        2 => Some(Width::U32),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// the shared object
// ---------------------------------------------------------------------------

/// The controller's guts, shared between the device and its MMIO region.
struct Shared {
    variant: Variant,
    units: usize,
    /// Every register, at `DEVICE` rank. **Never held across a bus access, a
    /// wire change or a call into a peripheral** (`CLAUDE.md`, re-entrancy).
    state: Mutex<State>,
    /// The level each request line is currently driving, per unit and per
    /// **selector slot**. A peripheral that holds its line high is asking for
    /// continuous service; whether the unit hears it depends on `CHSEL`.
    held: [[AtomicBool; SLOTS]; MAX_UNITS],
    /// A rising edge that has not yet bought its beat. Consumed by the beat,
    /// so a peripheral that pulses once per item gets one beat per pulse.
    ///
    /// An atomic and not a lock, deliberately: the inbound wire path must be
    /// able to run inside a peripheral's own critical section.
    pending: [[AtomicBool; SLOTS]; MAX_UNITS],
    /// The optional data-side handle a requester may publish on its pin. Only
    /// [`DmaPeripheral::dma_ready`] is used — see the module documentation.
    peer: [Mutex<Option<Weak<dyn DmaPeripheral>>>; MAX_UNITS],
    /// Each unit's interrupt output, at `WIRE` rank so it nests under `DEVICE`.
    irq: [Mutex<Option<WireSource>>; MAX_UNITS],
    /// The space this controller masters, weakly: the machine owns the space.
    bus: Mutex<Option<(Weak<AddressSpace>, RequesterId)>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("variant", &self.variant);
        s.field("units", &self.units);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish_non_exhaustive(),
            None => s.field("state", &"<in use>").finish_non_exhaustive(),
        }
    }
}

impl Shared {
    fn new(variant: Variant) -> Shared {
        Shared {
            variant,
            units: variant.units(),
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(variant)),
            held: core::array::from_fn(|_| core::array::from_fn(|_| AtomicBool::new(false))),
            pending: core::array::from_fn(|_| core::array::from_fn(|_| AtomicBool::new(false))),
            peer: core::array::from_fn(|_| Mutex::with_rank(LockRank::LEAF, None)),
            irq: core::array::from_fn(|_| Mutex::with_rank(LockRank::WIRE, None)),
            bus: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    // -- configuration decode ------------------------------------------------

    /// Decode `unit`'s control register, or `None` if it is programmed with a
    /// combination the part calls reserved.
    ///
    /// It takes the whole `state` and not one [`Unit`] because the channel
    /// face's request selector lives in a *shared* register: `CSELR` holds all
    /// seven nibbles (RM0351 §11.6.7), where the stream face keeps `CHSEL`
    /// inside each `SxCR`.
    fn config(&self, state: &State, unit: usize) -> Option<Config> {
        let u = &state.unit[unit];
        match self.variant {
            Variant::Stream => {
                let dir = match (u.cr >> 6) & 3 {
                    0 => Dir::PeriphToMem,
                    1 => Dir::MemToPeriph,
                    // Memory-to-memory: RM0090 §10.3.11 makes the peripheral
                    // port the source and the memory port the destination.
                    2 => Dir::PeriphToMem,
                    _ => return None,
                };
                let pw = size_width((u.cr >> 11) & 3)?;
                let mw = size_width((u.cr >> 13) & 3)?;
                let mut ie = 0u8;
                if u.cr & S_CR_TCIE != 0 {
                    ie |= F_TC;
                }
                if u.cr & S_CR_HTIE != 0 {
                    ie |= F_HT;
                }
                if u.cr & S_CR_TEIE != 0 {
                    ie |= F_TE;
                }
                if u.cr & S_CR_DMEIE != 0 {
                    ie |= F_DME;
                }
                if u.fcr & 0x80 != 0 {
                    ie |= F_FE;
                }
                Some(Config {
                    dir,
                    mem2mem: (u.cr >> 6) & 3 == 2,
                    circ: u.cr & S_CR_CIRC != 0,
                    dbm: u.cr & S_CR_DBM != 0,
                    ct: u.cr & S_CR_CT != 0,
                    pinc: u.cr & S_CR_PINC != 0,
                    minc: u.cr & S_CR_MINC != 0,
                    pw,
                    mw,
                    // `PINCOS` forces the peripheral increment to four
                    // regardless of `PSIZE` — RM0090 §10.5.5.
                    pstep: if u.cr & S_CR_PINCOS != 0 {
                        4
                    } else {
                        pw.bytes() as u32
                    },
                    prio: ((u.cr >> 16) & 3) as u8,
                    ie,
                    sel: ((u.cr >> 25) & 7) as u8,
                    // RM0090 §10.5.5: "when the memory-to-memory mode is
                    // selected, `PFCTRL` is forced to 0 by hardware" — there is
                    // no peripheral on either port to be the flow controller.
                    pfctrl: u.cr & S_CR_PFCTRL != 0 && (u.cr >> 6) & 3 != 2,
                })
            }
            Variant::Channel => {
                let pw = size_width((u.cr >> 8) & 3)?;
                let mw = size_width((u.cr >> 10) & 3)?;
                let mut ie = 0u8;
                if u.cr & C_CR_TCIE != 0 {
                    ie |= F_TC;
                }
                if u.cr & C_CR_HTIE != 0 {
                    ie |= F_HT;
                }
                if u.cr & C_CR_TEIE != 0 {
                    ie |= F_TE;
                }
                Some(Config {
                    // `DIR` says which port is the source whether or not
                    // `MEM2MEM` is set; `MEM2MEM` only removes the need for a
                    // request (RM0351 §11.4.7).
                    dir: if u.cr & C_CR_DIR != 0 {
                        Dir::MemToPeriph
                    } else {
                        Dir::PeriphToMem
                    },
                    mem2mem: u.cr & C_CR_MEM2MEM != 0,
                    circ: u.cr & C_CR_CIRC != 0,
                    dbm: false,
                    ct: false,
                    pinc: u.cr & C_CR_PINC != 0,
                    minc: u.cr & C_CR_MINC != 0,
                    pw,
                    mw,
                    pstep: pw.bytes() as u32,
                    prio: ((u.cr >> 12) & 3) as u8,
                    ie,
                    // RM0351 §11.6.7: `C1S` at bits 3:0, `C2S` at 7:4, and so
                    // on, and this face's unit 0 is the manual's channel 1.
                    sel: ((state.cselr >> (4 * unit)) & 0xf) as u8,
                    // There is no `PFCTRL` on the channel face at all: the
                    // channel controller is always the flow controller.
                    pfctrl: false,
                })
            }
        }
    }

    /// The interrupt-enable bits a write may change while the unit is running.
    const fn live_mask(&self) -> u32 {
        match self.variant {
            Variant::Stream => S_CR_LIVE,
            Variant::Channel => C_CR_LIVE,
        }
    }

    const fn cr_mask(&self) -> u32 {
        match self.variant {
            Variant::Stream => S_CR_MASK,
            Variant::Channel => C_CR_MASK,
        }
    }

    // -- the request latch ---------------------------------------------------

    /// Record a request line's new level. Takes no lock and calls nothing.
    ///
    /// `slot` is [`SLOTS`]'s numbering: zero for the unnumbered pin, `sel + 1`
    /// for a line the board attached to selector `sel`.
    fn set_request(&self, unit: usize, slot: usize, level: Level) {
        if unit >= self.units || slot >= SLOTS {
            return;
        }
        let high = level == Level::High;
        self.held[unit][slot].store(high, Ordering::SeqCst);
        if high {
            self.pending[unit][slot].store(true, Ordering::SeqCst);
        }
    }

    /// The two latch slots `unit` listens to while its selector reads `sel`.
    ///
    /// RM0090 Table 43 is a matrix, not a list: a request reaches a stream only
    /// on the *channel* the table puts it on, so a stream with `CHSEL = 4`
    /// hears the line wired to `req{n}c4` and is deaf to every other line on
    /// the same stream. Slot zero is always heard, because a board that wired
    /// the plain `req{n}` pin said it was not modelling the selector.
    const fn slots_for(sel: u8) -> [usize; 2] {
        [0, sel as usize + 1]
    }

    /// Whether `unit`'s peripheral is asking for service, with `unit`'s
    /// selector currently reading `sel`.
    ///
    /// Called with nothing of ours held, because it may call into the
    /// peripheral.
    fn requesting(&self, unit: usize, sel: u8) -> bool {
        for slot in Shared::slots_for(sel) {
            if self.held[unit][slot].load(Ordering::SeqCst)
                || self.pending[unit][slot].load(Ordering::SeqCst)
            {
                return true;
            }
        }
        match self.peer(unit) {
            Some(peer) => peer.dma_ready(),
            None => false,
        }
    }

    /// `unit`'s data-side peer, if it published one and is still alive.
    fn peer(&self, unit: usize) -> Option<Arc<dyn DmaPeripheral>> {
        self.peer[unit]
            .lock()
            .clone()
            .as_ref()
            .and_then(Weak::upgrade)
    }

    /// One unit's row of a latch array, packed one bit per slot.
    fn latch_bits(&self, latch: &[[AtomicBool; SLOTS]; MAX_UNITS], unit: usize) -> u32 {
        let mut bits = 0u32;
        for (slot, cell) in latch[unit].iter().enumerate() {
            if cell.load(Ordering::SeqCst) {
                bits |= 1 << slot;
            }
        }
        bits
    }

    /// Put a packed row back.
    fn set_latch_bits(&self, latch: &[[AtomicBool; SLOTS]; MAX_UNITS], unit: usize, bits: u32) {
        for (slot, cell) in latch[unit].iter().enumerate() {
            cell.store(bits & (1 << slot) != 0, Ordering::SeqCst);
        }
    }

    /// Spend the edge `unit` was just served on.
    ///
    /// Both slots, because either could have been the one asking and a beat
    /// answers the unit rather than the pin.
    fn spend_request(&self, unit: usize, sel: u8) {
        for slot in Shared::slots_for(sel) {
            self.pending[unit][slot].store(false, Ordering::SeqCst);
        }
    }

    // -- flag packing --------------------------------------------------------

    /// The value `LISR`/`HISR` (`high`) or `ISR` reads.
    fn status(&self, state: &State, high: bool) -> u32 {
        let mut out = 0u32;
        match self.variant {
            Variant::Stream => {
                let base = usize::from(high) * 4;
                for (i, &shift) in STREAM_FLAG_SHIFT.iter().enumerate() {
                    let flags = state.unit[base + i].flags;
                    for (bit, pos) in [
                        (F_FE, S_FEIF),
                        (F_DME, S_DMEIF),
                        (F_TE, S_TEIF),
                        (F_HT, S_HTIF),
                        (F_TC, S_TCIF),
                    ] {
                        if flags & bit != 0 {
                            out |= 1 << (shift + pos);
                        }
                    }
                }
            }
            Variant::Channel => {
                for i in 0..self.units {
                    let flags = state.unit[i].flags;
                    let shift = 4 * i as u32;
                    if flags & (F_TC | F_HT | F_TE) != 0 {
                        out |= 1 << (shift + C_GIF);
                    }
                    for (bit, pos) in [(F_TC, C_TCIF), (F_HT, C_HTIF), (F_TE, C_TEIF)] {
                        if flags & bit != 0 {
                            out |= 1 << (shift + pos);
                        }
                    }
                }
            }
        }
        out
    }

    /// Apply a write-1-to-clear to `LIFCR`/`HIFCR` (`high`) or `IFCR`.
    fn clear_flags(&self, state: &mut State, value: u32, high: bool) {
        match self.variant {
            Variant::Stream => {
                let base = usize::from(high) * 4;
                for (i, &shift) in STREAM_FLAG_SHIFT.iter().enumerate() {
                    let mut clear = 0u8;
                    for (bit, pos) in [
                        (F_FE, S_FEIF),
                        (F_DME, S_DMEIF),
                        (F_TE, S_TEIF),
                        (F_HT, S_HTIF),
                        (F_TC, S_TCIF),
                    ] {
                        if value & (1 << (shift + pos)) != 0 {
                            clear |= bit;
                        }
                    }
                    state.unit[base + i].flags &= !clear;
                }
            }
            Variant::Channel => {
                for i in 0..self.units {
                    let shift = 4 * i as u32;
                    // `CGIFx` clears the whole nibble — RM0351 §11.6.2.
                    let mut clear = if value & (1 << (shift + C_GIF)) != 0 {
                        F_ALL
                    } else {
                        0
                    };
                    for (bit, pos) in [(F_TC, C_TCIF), (F_HT, C_HTIF), (F_TE, C_TEIF)] {
                        if value & (1 << (shift + pos)) != 0 {
                            clear |= bit;
                        }
                    }
                    state.unit[i].flags &= !clear;
                }
            }
        }
    }

    // -- register decode -----------------------------------------------------

    /// Which unit and which of its registers `offset` names, if any.
    fn unit_reg(&self, offset: u64) -> Option<(usize, u64)> {
        match self.variant {
            Variant::Stream => {
                if offset < 0x10 {
                    return None;
                }
                let rel = offset - 0x10;
                let unit = (rel / 0x18) as usize;
                (unit < self.units).then_some((unit, rel % 0x18))
            }
            Variant::Channel => {
                if !(0x08..0x08 + 0x14 * self.units as u64).contains(&offset) {
                    return None;
                }
                let rel = offset - 0x08;
                Some(((rel / 0x14) as usize, rel % 0x14))
            }
        }
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match (self.variant, offset) {
            (Variant::Stream, 0x00) => return self.status(&state, false),
            (Variant::Stream, 0x04) => return self.status(&state, true),
            // The clear registers are write-only and read as zero.
            (Variant::Stream, 0x08 | 0x0c) => return 0,
            (Variant::Channel, 0x00) => return self.status(&state, false),
            (Variant::Channel, 0x04) => return 0,
            (Variant::Channel, 0xa8) => return state.cselr,
            _ => {}
        }
        let Some((unit, reg)) = self.unit_reg(offset) else {
            return 0;
        };
        let u = &state.unit[unit];
        match (self.variant, reg) {
            (_, 0x00) => u.cr & self.cr_mask(),
            (_, 0x04) => u.cur_ndtr & 0xffff,
            (_, 0x08) => u.par,
            (_, 0x0c) => u.m0ar,
            (Variant::Stream, 0x10) => u.m1ar,
            // `FS` reads 100b, "FIFO empty", because there is no FIFO here.
            (Variant::Stream, 0x14) => (u.fcr & S_FCR_WRITABLE) | 0x20,
            _ => 0,
        }
    }

    /// Apply a register write. Returns the units whose interrupt level may
    /// have moved, so the caller can drive their wires with nothing held.
    fn write_register(&self, offset: u64, value: u32) -> u8 {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (Variant::Stream, 0x00 | 0x04) | (Variant::Channel, 0x00) => return 0,
            (Variant::Stream, 0x08) => {
                self.clear_flags(&mut state, value, false);
                return 0xff;
            }
            (Variant::Stream, 0x0c) => {
                self.clear_flags(&mut state, value, true);
                return 0xff;
            }
            (Variant::Channel, 0x04) => {
                self.clear_flags(&mut state, value, false);
                return 0xff;
            }
            (Variant::Channel, 0xa8) => {
                state.cselr = value;
                return 0;
            }
            _ => {}
        }
        let Some((unit, reg)) = self.unit_reg(offset) else {
            return 0;
        };
        let mask = self.cr_mask();
        if reg == 0x00 {
            let live = self.live_mask();
            let u = &mut state.unit[unit];
            let was_enabled = u.cr & CR_EN != 0;
            // While a unit runs, only `EN` and the interrupt enables take
            // effect; the rest of the control register is write-protected.
            u.cr = if u.running {
                (u.cr & !live) | (value & live & mask)
            } else {
                value & mask
            };
            let now_enabled = u.cr & CR_EN != 0;
            if !now_enabled {
                u.running = false;
            }
            if !was_enabled && now_enabled {
                self.arm(unit, &mut state);
            }
            return 1 << unit;
        }
        let u = &mut state.unit[unit];
        match (self.variant, reg) {
            // `NDTR`, `PAR` and `M0AR` are write-protected while the unit runs
            // (RM0090 §10.5.6, RM0351 §11.6.4). `M1AR` is not: swapping the
            // idle half is the point of double buffering.
            (_, 0x04) if !u.running => {
                u.ndtr = value & 0xffff;
                u.cur_ndtr = u.ndtr;
            }
            (_, 0x08) if !u.running => u.par = value,
            (_, 0x0c) if !u.running => u.m0ar = value,
            (Variant::Stream, 0x10) => u.m1ar = value,
            (Variant::Stream, 0x14) => u.fcr = value & S_FCR_WRITABLE,
            _ => {}
        }
        0
    }

    /// `EN` has gone 0→1: latch the pointers, or refuse the configuration.
    fn arm(&self, unit: usize, state: &mut State) {
        let cfg = self.config(state, unit);
        let u = &mut state.unit[unit];
        let Some(cfg) = cfg else {
            // A reserved `DIR` or a reserved `PSIZE`/`MSIZE`. RM0090 §10.3.13
            // makes a configuration error a transfer error: the flag goes up
            // and the stream comes straight back down.
            u.cr &= !CR_EN;
            u.running = false;
            u.flags |= F_TE;
            return;
        };
        u.cur_ndtr = u.ndtr;
        u.cur_par = u.par;
        u.cur_mar = if cfg.dbm && cfg.ct { u.m1ar } else { u.m0ar };
        u.half_done = false;
        u.running = u.cur_ndtr != 0;
    }

    // -- the engine ----------------------------------------------------------

    /// The units that could move a beat, highest priority first.
    ///
    /// RM0090 §10.3.2: the software priority `PL` decides, and units of equal
    /// priority go in ascending unit number.
    fn candidates(&self, state: &State) -> ([Candidate; MAX_UNITS], usize) {
        let mut out = [Candidate::NONE; MAX_UNITS];
        let mut len = 0;
        for prio in (0..4u8).rev() {
            for unit in 0..self.units {
                let u = &state.unit[unit];
                if !u.running || u.cur_ndtr == 0 {
                    continue;
                }
                let Some(cfg) = self.config(state, unit) else {
                    continue;
                };
                if cfg.prio != prio {
                    continue;
                }
                out[len] = Candidate {
                    unit,
                    needs_request: !cfg.mem2mem,
                    sel: cfg.sel,
                };
                len += 1;
            }
        }
        (out, len)
    }

    /// Plan `unit`'s next beat, with the lock held and nothing called outward.
    ///
    /// The `bool` alongside is `PFCTRL`, carried out of the lock so the caller
    /// knows whether to ask the peripheral about the end of the transfer.
    fn plan(&self, state: &State, unit: usize) -> Option<(Beat, bool)> {
        let u = &state.unit[unit];
        if !u.running || u.cur_ndtr == 0 {
            return None;
        }
        let cfg = self.config(state, unit)?;
        let u = &state.unit[unit];
        let (pw, mw) = (cfg.pw, cfg.mw);
        let (par, mar) = (u64::from(u.cur_par), u64::from(u.cur_mar));
        let beat = match cfg.dir {
            Dir::PeriphToMem => Beat {
                src: par,
                dst: mar,
                src_w: pw,
                dst_w: mw,
            },
            Dir::MemToPeriph => Beat {
                src: mar,
                dst: par,
                src_w: mw,
                dst_w: pw,
            },
        };
        Some((beat, cfg.pfctrl))
    }

    /// Account for a beat that has already happened. Returns the unit's new
    /// interrupt level, to be driven once the lock is gone.
    ///
    /// `last` is the peripheral's answer to "was that your final item?",
    /// sampled before the beat and meaningful only under `PFCTRL`.
    fn commit(&self, unit: usize, faulted: bool, last: bool) -> Level {
        let mut state = self.state.lock();
        let Some(cfg) = self.config(&state, unit) else {
            return Level::Low;
        };
        let u = &mut state.unit[unit];
        if faulted {
            // RM0090 §10.3.13: a bus error on either port sets `TEIF` and
            // disables the stream. The item that faulted is not counted.
            u.flags |= F_TE;
            u.cr &= !CR_EN;
            u.running = false;
        } else {
            if cfg.pinc {
                u.cur_par = u.cur_par.wrapping_add(cfg.pstep);
            }
            if cfg.minc {
                u.cur_mar = u.cur_mar.wrapping_add(cfg.mw.bytes() as u32);
            }
            u.cur_ndtr = u.cur_ndtr.wrapping_sub(1);
            if !u.half_done && u.ndtr >= 2 && u.cur_ndtr <= u.ndtr / 2 {
                u.half_done = true;
                u.flags |= F_HT;
            }
            if cfg.pfctrl && last {
                // RM0090 §10.3.2, peripheral flow control: the peripheral has
                // just passed its last item, so *it* ends the transfer —
                // `TCIF` goes up and the stream disables itself with whatever
                // `NDTR` happens to be left. Neither `CIRC` nor `DBM` gets a
                // say: the manual forbids both alongside `PFCTRL`, and
                // reloading here would be a transfer the peripheral has said
                // is over.
                u.flags |= F_TC;
                u.cr &= !CR_EN;
                u.running = false;
            } else if u.cur_ndtr == 0 {
                u.flags |= F_TC;
                if cfg.pfctrl {
                    // The count was a *maximum* (RM0090 §10.3.2) and it ran
                    // out before the peripheral was done. RM0090 §10.5.6: a
                    // stream whose `NDTR` is zero can serve no transaction, so
                    // it stops here — and, unlike the flow-controlled end
                    // above, this is the firmware having under-programmed the
                    // maximum rather than a transfer that finished.
                    u.cr &= !CR_EN;
                    u.running = false;
                } else if cfg.dbm {
                    // Double buffer: swap `CT`, reload, and point the memory
                    // port at the other buffer (RM0090 §10.3.10).
                    u.cr ^= S_CR_CT;
                    u.cur_ndtr = u.ndtr;
                    u.cur_par = u.par;
                    u.cur_mar = if u.cr & S_CR_CT != 0 { u.m1ar } else { u.m0ar };
                    u.half_done = false;
                } else if cfg.circ {
                    u.cur_ndtr = u.ndtr;
                    u.cur_par = u.par;
                    u.cur_mar = u.m0ar;
                    u.half_done = false;
                } else {
                    u.cr &= !CR_EN;
                    u.running = false;
                }
            }
        }
        Level::from_bool(u.flags & cfg.ie != 0)
    }

    /// Move one beat, or report that nothing wanted to.
    ///
    /// The shape is the re-entrancy contract written out: plan under the lock,
    /// release, touch the bus, retake the lock to commit, release, and only
    /// then drive a wire.
    fn step(&self, bus: &AddressSpace, attrs: MemAttrs) -> bool {
        let (cands, len) = {
            let state = self.state.lock();
            self.candidates(&state)
        };
        let mut chosen = None;
        for cand in &cands[..len] {
            if !cand.needs_request || self.requesting(cand.unit, cand.sel) {
                chosen = Some(*cand);
                break;
            }
        }
        let Some(cand) = chosen else { return false };
        let unit = cand.unit;

        let Some((beat, pfctrl)) = ({
            let state = self.state.lock();
            self.plan(&state, unit)
        }) else {
            return false;
        };

        // Peripheral flow control (RM0090 §10.3.2): ask, *before* moving the
        // item, whether it is the peripheral's last. Before, because on the
        // part the signal arrives with the data — and because a peripheral
        // whose last word has just been read out has nothing left to answer
        // with. Nothing of ours is held, as for every other outward call here.
        let last = pfctrl && self.peer(unit).is_some_and(|peer| peer.dma_last());

        // Nothing of ours is held for either access.
        let moved = bus.read(beat.src, beat.src_w, attrs).and_then(|value| {
            // RM0351 Table 41: a narrowing conversion keeps the least
            // significant bytes, a widening one zero-extends.
            let truncated = match beat.dst_w.bytes() {
                n if n >= 8 => value,
                n => value & ((1u64 << (n * 8)) - 1),
            };
            bus.write(beat.dst, beat.dst_w, truncated, attrs)
        });

        let level = self.commit(unit, moved.is_err(), last);
        // The edge is spent whether or not the beat faulted: the peripheral
        // was served, and a fault is not a reason to serve it twice.
        self.spend_request(unit, cand.sel);
        self.drive_irq(unit, level);
        true
    }

    /// Recompute and drive one unit's interrupt output.
    fn refresh_irq(&self, unit: usize) {
        let level = {
            let state = self.state.lock();
            match self.config(&state, unit) {
                Some(cfg) => Level::from_bool(state.unit[unit].flags & cfg.ie != 0),
                None => Level::Low,
            }
        };
        self.drive_irq(unit, level);
    }

    /// Drive one unit's interrupt output with nothing of ours held.
    fn drive_irq(&self, unit: usize, level: Level) {
        let source = self.irq[unit].lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Drive every unit whose bit is set in `mask`.
    fn refresh_mask(&self, mask: u8) {
        for unit in 0..self.units {
            if mask & (1 << unit) != 0 {
                self.refresh_irq(unit);
            }
        }
    }

    /// The space this controller masters, and the id its accesses carry.
    fn bus(&self) -> Option<(Arc<AddressSpace>, RequesterId)> {
        let handle = self.bus.lock().clone()?;
        Some((handle.0.upgrade()?, handle.1))
    }
}

/// One request *pin*, with the wired-OR of everything driving it.
///
/// A cell of RM0090 Table 43 routinely holds more than one request —
/// `TIM2_CH2` and `TIM2_CH4` are both DMA1 stream 6 channel 3 — and on the
/// part those are OR'd onto one line into the stream. A board therefore writes
/// two `wire` statements into the same pin, and without this the second
/// driver's low would cancel the first's high. [`FanIn`] is exactly that
/// bookkeeping, so the pin owns one and resolves it as a wired-OR.
///
/// It is *derived* state and deliberately not snapshotted: a `FanIn` is a
/// cache of what the drivers last said, the resolved level is what the latch
/// in [`Shared`] holds, and a load is followed by the machine re-announcing
/// every net.
#[derive(Debug)]
struct RequestPin {
    shared: Arc<Shared>,
    unit: usize,
    slot: usize,
    drivers: FanIn,
}

impl WireSink for RequestPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        // No lock, no outward call: the beat this asks for happens in `run`.
        // That is what lets a peripheral raise its request from inside its own
        // register handler.
        self.drivers.set(src, level);
        self.shared
            .set_request(self.unit, self.slot, self.drivers.resolve(Resolve::Or));
    }
}

// ---------------------------------------------------------------------------
// the MMIO face
// ---------------------------------------------------------------------------

/// The register block, as something an address space dispatches to.
#[derive(Debug)]
struct Registers {
    shared: Arc<Shared>,
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing here has a read side effect — the status registers are
        // cleared through `IFCR` and `NDTR` only counts down as beats move —
        // so a debug read is the same read (`ROADMAP.md` §15, invariant 5).
        let bytes = self.shared.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `SxCR` would start a transfer and one to `IFCR`
            // would drop an interrupt the guest has not seen. Neither can be
            // made harmless, so it is refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        let touched = self
            .shared
            .write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        self.shared.refresh_mask(touched);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers have to be accessed by words (32 bits)" —
        // RM0090 §10.5, RM0351 §11.6.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An STM32 DMA controller.
#[derive(Debug)]
pub struct Dma {
    shared: Arc<Shared>,
    region: RegionRef,
    /// The device owns its input pins; a wire holds only a `Weak` to them.
    pins: Mutex<Vec<Arc<RequestPin>>>,
}

impl Dma {
    /// Validate `props` and build the controller.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given; [`Error::Config`] if `mux` asks
    /// for the DMAMUX, which is not written.
    pub fn new(props: &Props) -> Result<Dma> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "stream", &["stream", "channel"])? {
            "channel" => Variant::Channel,
            _ => Variant::Stream,
        };
        let mux = r.or("mux", false)?;
        r.finish()?;
        if mux {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "DMAMUX request routing (RM0432 §14) is a separate peripheral on the die and \
                     a separate class here, `st.dmamux`: declare one and wire it between the \
                     peripherals and this controller's `req` pins, rather than asking a DMA \
                     controller to be its own multiplexer",
                ),
            });
        }
        Ok(Dma::with_variant(variant))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_variant(variant: Variant) -> Dma {
        let shared = Arc::new(Shared::new(variant));
        let region = Arc::new(Region::io(
            "dma",
            variant.window(),
            Arc::new(Registers {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Dma {
            shared,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Which register face this instance wears.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.variant
    }

    /// Point the controller at the space it masters.
    ///
    /// Normally done by [`Instance::bind`] from the object's `space =`
    /// property; a test that builds its own space calls this.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        *self.shared.bus.lock() = Some((Arc::downgrade(space), requester));
    }

    /// Connect unit `unit`'s interrupt output.
    pub fn connect_irq(&self, unit: usize, source: WireSource) {
        if unit < self.shared.units {
            *self.shared.irq[unit].lock() = Some(source);
            self.shared.refresh_irq(unit);
        }
    }

    /// Raise or drop unit `unit`'s **unselected** request line directly, as a
    /// peripheral wired to the plain `req{n}` pin would.
    ///
    /// Served whatever `CHSEL`/`CSELR` reads — see [`Dma::set_selected_request`]
    /// for the gated form.
    pub fn set_request(&self, unit: usize, level: Level) {
        self.shared.set_request(unit, 0, level);
    }

    /// Raise or drop the request line the board attached to unit `unit`'s
    /// selector `sel`, as a peripheral wired to `req{unit}c{sel}` would.
    ///
    /// It buys a beat only while the unit's `CHSEL` (stream face) or `CSELR`
    /// nibble (channel face) reads `sel`, which is the half of RM0090
    /// Table 43 a `req{n}`-only wiring throws away.
    pub fn set_selected_request(&self, unit: usize, sel: u8, level: Level) {
        self.shared.set_request(unit, sel as usize + 1, level);
    }

    /// What `NDTR` currently reads for `unit`.
    #[must_use]
    pub fn remaining(&self, unit: usize) -> u32 {
        self.shared.state.lock().unit[unit].cur_ndtr
    }

    /// Whether `unit` is armed and counting.
    #[must_use]
    pub fn is_running(&self, unit: usize) -> bool {
        self.shared.state.lock().unit[unit].running
    }

    /// Move up to `beats` beats, in priority order. Returns how many moved.
    ///
    /// This is what [`Device::run`] does per quantum, exposed so a test can
    /// pace a transfer without a scheduler.
    pub fn pump(&self, beats: u64) -> u64 {
        let Some((bus, requester)) = self.shared.bus() else {
            return 0;
        };
        let attrs = MemAttrs::DEFAULT.with_requester(requester);
        let mut moved = 0;
        while moved < beats && self.shared.step(&bus, attrs) {
            moved += 1;
        }
        moved
    }

    /// The pin index a `req`/`irq` name selects, in this face's numbering.
    fn pin_index(&self, port: &str, prefix: &str) -> Option<usize> {
        let n: usize = port.strip_prefix(prefix)?.parse().ok()?;
        let first = self.shared.variant.first_pin();
        let unit = n.checked_sub(first)?;
        (unit < self.shared.units).then_some(unit)
    }

    /// The unit and latch slot a `req` pin name selects.
    ///
    /// `req3` is the unnumbered line and lands in slot zero. `req3c4` is
    /// "stream 3, channel 4" — one cell of RM0090 Table 43 — and lands in
    /// slot 5, where only a stream reading `CHSEL = 4` can hear it.
    fn req_pin(&self, port: &str) -> Option<(usize, usize)> {
        let rest = port.strip_prefix("req")?;
        let (number, sel) = match rest.split_once('c') {
            Some((number, sel)) => {
                let sel: usize = sel.parse().ok()?;
                (number, sel.checked_add(1)?)
            }
            None => (rest, 0),
        };
        if sel > MAX_SELECTORS {
            return None;
        }
        let n: usize = number.parse().ok()?;
        let unit = n.checked_sub(self.shared.variant.first_pin())?;
        (unit < self.shared.units).then_some((unit, sel))
    }
}

impl Device for Dma {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `bind` takes the space and a `map` statement places
        // the register block.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::reset(self.shared.variant);
        for unit in 0..self.shared.units {
            for slot in 0..SLOTS {
                self.shared.held[unit][slot].store(false, Ordering::SeqCst);
                self.shared.pending[unit][slot].store(false, Ordering::SeqCst);
            }
        }
        for unit in 0..self.shared.units {
            self.shared.refresh_irq(unit);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock().clone();
        w.write_u8(self.shared.units as u8)?;
        for unit in 0..self.shared.units {
            let u = &state.unit[unit];
            for value in [
                u.cr, u.fcr, u.ndtr, u.par, u.m0ar, u.m1ar, u.cur_ndtr, u.cur_par, u.cur_mar,
            ] {
                w.write_u32(value)?;
            }
            w.write_bool(u.running)?;
            w.write_bool(u.half_done)?;
            w.write_u8(u.flags)?;
            // The request latch is guest-visible through its effect: a
            // transfer caught mid-flight resumes only if the peripheral is
            // still asking, so the levels travel with the state. One bit per
            // selector slot, packed, because `SLOTS` of them per unit as
            // separate booleans is the same information four times the size.
            w.write_u32(self.shared.latch_bits(&self.shared.held, unit))?;
            w.write_u32(self.shared.latch_bits(&self.shared.pending, unit))?;
        }
        w.write_u32(state.cselr)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let units = usize::from(r.read_u8()?);
        if units != self.shared.units {
            return Err(Error::State(format!(
                "snapshot has {units} units, this controller has {}",
                self.shared.units
            )));
        }
        let mut state = State::reset(self.shared.variant);
        let mut latch = [(0u32, 0u32); MAX_UNITS];
        for (unit, slot) in latch.iter_mut().enumerate().take(units) {
            let u = &mut state.unit[unit];
            u.cr = r.read_u32()?;
            u.fcr = r.read_u32()?;
            u.ndtr = r.read_u32()?;
            u.par = r.read_u32()?;
            u.m0ar = r.read_u32()?;
            u.m1ar = r.read_u32()?;
            u.cur_ndtr = r.read_u32()?;
            u.cur_par = r.read_u32()?;
            u.cur_mar = r.read_u32()?;
            u.running = r.read_bool()?;
            u.half_done = r.read_bool()?;
            u.flags = r.read_u8()?;
            *slot = (r.read_u32()?, r.read_u32()?);
        }
        state.cselr = r.read_u32()?;
        *self.shared.state.lock() = state;
        for (unit, &(held, pending)) in latch.iter().enumerate().take(units) {
            self.shared.set_latch_bits(&self.shared.held, unit, held);
            self.shared
                .set_latch_bits(&self.shared.pending, unit, pending);
        }
        for unit in 0..units {
            self.shared.refresh_irq(unit);
        }
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match self.pin_index(port, "irq") {
            Some(unit) => {
                self.connect_irq(unit, source);
                Ok(())
            }
            None => Err(Error::Config {
                at: port.to_string(),
                message: format!(
                    "a `{CLASS_NAME}` drives one interrupt per unit: `irq{}`..`irq{}`",
                    self.shared.variant.first_pin(),
                    self.shared.variant.first_pin() + self.shared.units - 1
                ),
            }),
        }
    }

    fn announce(&self, port: &str) {
        if let Some(unit) = self.pin_index(port, "irq") {
            self.shared.refresh_irq(unit);
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let (unit, slot) = self.req_pin(port)?;
        let pin = Arc::new(RequestPin {
            shared: Arc::clone(&self.shared),
            unit,
            slot,
            drivers: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn attach_dma_peripheral(&self, port: &str, peer: Weak<dyn DmaPeripheral>) {
        // The data-side handle is per *unit*: a stream has one peripheral
        // feeding it at a time, whichever of its eight channels is selected.
        if let Some((unit, _)) = self.req_pin(port) {
            *self.shared.peer[unit].lock() = Some(peer);
        }
    }

    fn is_runnable(&self) -> bool {
        // A beat costs bus time and the scheduler owns time (`CLAUDE.md`).
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        let cap = budget.ticks.min(MAX_BEATS_PER_RUN);
        let moved = self.pump(cap);
        // Stopped early because nothing wanted to move: there is nothing more
        // this quantum could have done, so consume it. Stopped at the cap:
        // consume only what was done and take the rest next call.
        Consumed::new(if moved < cap { budget.ticks } else { cap })
    }
}

/// The machine layer's half: a DMA controller is a bus master.
impl Instance for Dma {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a DMA controller masters the bus it transfers across: add `space = mem` to the \
                 object that declares it — and give it the space DMA can actually reach, which on \
                 an F4 excludes CCM (RM0090 §2.3)",
            ),
        })?;
        self.attach_bus(space, ctx.requester());
        Ok(())
    }
}

/// The `st.dma` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 DMA controller: eight streams (RM0090 §10) or seven channels (RM0351 §11)",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register face: \"stream\" (F2/F4/F7) or \"channel\" (F0/F1/F3/L0/L4/G0)",
        },
        PropertySpec {
            name: "mux",
            kind: ValueKind::Bool,
            required: false,
            summary: "route requests through a DMAMUX — refused: `st.dmamux` is its own class",
        },
    ],
    construct: |props| Ok(Box::new(Dma::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Dma::new(props)?)))
}

/// What the validator should know about `st.dma`.
///
/// Both faces' pins are declared, because one class covers both and which
/// subset an instance answers to depends on its `variant`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["stream", "channel"]))
        .prop(PropSchema::new("mux", ValueKind::Bool))
        .region("")
        .region("regs");
    for unit in 0..MAX_UNITS {
        schema = schema
            .port(format!("irq{unit}"), PortDir::Out)
            .port(format!("req{unit}"), PortDir::In);
        // And the selector-qualified form, one pin per cell of the part's
        // request matrix, declared as a bank so an error message prints
        // `req0c0`..`req0c15` rather than sixteen lines. Both faces get all
        // sixteen: `CHSEL` only reaches eight, and a stream wired to `req0c9`
        // would never fire, but the schema is per class and an instance's
        // `variant` is a property.
        schema = schema.port_bank(format!("req{unit}c"), PortDir::In, MAX_SELECTORS as u32);
    }
    schema
}

#[cfg(test)]
mod tests;
