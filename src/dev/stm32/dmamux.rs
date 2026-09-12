//! The STM32 **DMAMUX**, the request multiplexer that sits between the
//! peripherals and a channel-based DMA controller.
//!
//! # Sources
//!
//! * ST **RM0432** rev 8 (STM32L4+: L4P5/L4Q5, L4R5/L4S5, L4R7/L4S7, L4R9/L4S9),
//!   **§14 "DMA request multiplexer (DMAMUX)"** — the register map of §14.5,
//!   the channel multiplexer of §14.3.2, the synchronization mode and its
//!   `NBREQ`/`SPOL`/`SYNC_ID` of §14.3.3, the event generation of §14.3.4 and
//!   the request generator of §14.3.5.
//! * ST **RM0440** (STM32G4) §13 and ST **RM0434** (STM32WB) §15 describe the
//!   same block. Where the three differ it is only in *how many* channels and
//!   request lines the part bonds and in which source sits on which line —
//!   both facts about the part, and both therefore in the board file rather
//!   than here.
//!
//! **RM0351 is not one of the sources, and cannot be.** The STM32L4x5/L4x6 of
//! RM0351 — the part [`crate::dev::stm32::dma`]'s channel face is written from
//! — has **no DMAMUX at all**: it routes requests with `DMA_CSELR`, a four-bit
//! selector per channel inside the DMA controller itself. The DMAMUX arrives
//! with the L4**+** (RM0432) and is standard from the G0/G4/WB/H7/L5/U5 on. A
//! model that cited RM0351 here would be citing a manual that does not contain
//! the peripheral.
//!
//! **No emulator source of any licence was consulted** (`CLAUDE.md`,
//! provenance). Every bit position and every rule below is from one of the
//! manuals named above.
//!
//! # What it is for
//!
//! On an F4 the request matrix is fixed in silicon: RM0090 Table 43 says SDIO
//! is on DMA2 stream 3 channel 4 and nothing can move it, which is why
//! [`crate::dev::stm32::dma`]'s stream face gates a request on `CHSEL` and why
//! a board file spells the matrix out in `wire` statements. The DMAMUX replaces
//! that table with a register: **any** of the part's request lines can be
//! steered to **any** DMA channel by writing its number into `DMAMUX_CxCR`.
//!
//! So this device is, at heart, a wire router. Request line *n* comes in on
//! pin `req{n}`; channel *x*'s output goes out on pin `ch{x}` to one of an
//! `st.dma`'s `req` inputs; and `DMAREQ_ID` in `DMAMUX_CxCR` decides which
//! input reaches which output. Everything else in this file is the two
//! elaborations RM0432 adds on top: **synchronization**, which holds requests
//! back until an external event lets a burst of them through, and the
//! **request generator**, which manufactures requests from a trigger with no
//! peripheral involved at all.
//!
//! # Wiring it
//!
//! The DMAMUX belongs in front of a **channel-face** controller, and the
//! channel face numbers its pins from one because RM0351 calls them
//! `DMA1_Channel1`..`DMA1_Channel7`. So a board writes:
//!
//! ```text
//!   object dmamux "st.dmamux" { channels = 7 }
//!   object dma1   "st.dma"    { space = mem, variant = "channel" }
//!
//!   wire dmamux.ch0 -> dma1.req1     # DMAMUX channel 0 drives DMA1 channel 1
//!   wire dmamux.ch1 -> dma1.req2
//!   wire spi1.dma-tx -> dmamux.req17 # whatever the part's table calls it
//! ```
//!
//! The off-by-one between `ch0` and `req1` is not a mistake to iron out: each
//! side is numbered the way *its own* manual numbers it, and a board file that
//! reads like the reference manual is the point.
//!
//! **There is no L4+/G4/WB board in this tree yet**, so the wiring above is
//! what one would look like rather than a description of a machine file that
//! exists. The device is tested against its own registers and pins.
//!
//! # The request lines, and the four that are not pins
//!
//! `DMAREQ_ID` is seven bits, so there are 128 request line numbers. Three
//! ranges, and a board only ever wires the third:
//!
//! * **0 — no request.** The reset value of `DMAMUX_CxCR`, and the way a guest
//!   parks a channel. The output stays low.
//! * **1 to 4 — the four request generators.** These lines have no pin because
//!   nothing outside the block drives them: generator *g* drives line *g + 1*
//!   from inside this device, which is what makes `DMAREQ_ID = 1` mean "fed by
//!   generator 0". The private `Shared::emit` is the whole of it.
//! * **5 to 127 — the peripherals**, on pins `req5`..`req127`. Which peripheral
//!   is on which number is a table in the part's manual, so it is a `wire`
//!   statement and not a constant in this file.
//!
//! A caveat worth stating rather than burying: the *numbering* of that third
//! range, and whether the generators really sit at 1–4 or at 0–3, is
//! part-specific — RM0440's G4 table starts its peripheral assignments at a
//! different offset from RM0432's L4+ one. This model follows RM0432, where
//! line 0 is the idle encoding. A part that disagrees needs its own board
//! wiring, not a change here.
//!
//! # Levels, edges, and what one request is
//!
//! [`crate::dev::stm32::dma`] accepts a request either way round: a level held
//! high means "serve me continuously", and a pulse means "serve me once". The
//! DMAMUX preserves whichever its input used, because it is a router:
//!
//! * With `SE = 0` a channel's output **is** its input's level, re-driven
//!   whenever that input moves. A FIFO-style peripheral holding its line high
//!   gets exactly the continuous service it would get wired straight to the
//!   controller.
//! * With `SE = 1` the output is a latch: it goes high on a rising input edge
//!   *if* the burst counter has credit, drops on the falling edge, and is
//!   forced low once the credit is spent.
//! * A request generator has no input level to pass on, so it **pulses**
//!   `GNBREQ + 1` times.
//!
//! That last case is the one seam that loses information, and it loses it
//! downstream rather than here: `st.dma` latches a rising edge into one atomic
//! per unit, so several pulses delivered before the controller next runs
//! collapse into a single beat. A generator programmed with `GNBREQ = 3` moves
//! one item, not four, unless the scheduler interleaves. Nothing in this file
//! can fix that — it is the shape of the request latch on the other side — and
//! it is written down here so that the next person to look at a short
//! generator-driven transfer starts in the right place.
//!
//! # Synchronization (`SE`, `SPOL`, `SYNC_ID`, `NBREQ`)
//!
//! RM0432 §14.3.3. A synchronized channel does not forward anything until an
//! edge arrives on one of the eight `sync` inputs — `SYNC_ID` picks it,
//! `SPOL` picks the edge (`00` none, `01` rising, `10` falling, `11` both). On
//! that edge the channel is given credit for `NBREQ + 1` requests, forwards
//! exactly that many, and then goes quiet again until the next sync event.
//!
//! With `EGE = 1` the channel also **pulses its `evt{x}` output** as the last
//! of those requests goes out (§14.3.4). On the part that pulse is routed back
//! into another channel's `sync` input, which is how a driver chains two
//! transfers; here it is an ordinary output pin and a board may do the same.
//!
//! # Overruns (`SOF`, `OF`)
//!
//! Both counters raise a flag when they are asked to start over before they
//! have finished:
//!
//! * **`SOF{x}` in `DMAMUX_CSR`** — a sync event reached channel *x* while it
//!   still had credit left from the previous one (§14.3.3). The credit is
//!   reloaded regardless; the flag records that requests were lost.
//! * **`OF{x}` in `DMAMUX_RGSR`** — a trigger reached generator *x* while it
//!   still had requests it had not managed to emit (§14.3.5). That happens
//!   here when **no channel selects the generator's line**, which is the
//!   configuration error the flag exists to catch: a generator wired to
//!   nothing, triggered twice.
//!
//! Each is cleared by writing a one to the matching bit of `DMAMUX_CFR` /
//! `DMAMUX_RGCFR`, and each raises the single [`pin::IRQ`] output when its
//! enable — `SOIE` in the channel's `CxCR`, `OIE` in the generator's `RGxCR` —
//! is set. One pin rather than one per channel, because the part has one
//! vector: the G4's Table 100 calls it `DMAMUX_OVR`.
//!
//! # The re-entrancy contract
//!
//! The same shape as [`crate::dev::stm32::dma`], for the same reason: driving
//! a request output reaches into a DMA controller, which may reach back.
//!
//! * All the registers are one [`Mutex`] at [`LockRank::DEVICE`]. It is taken
//!   to decide what changed and **released before any wire moves**.
//! * The outputs are at [`LockRank::WIRE`], above `DEVICE`; each handle is
//!   cloned out and the lock dropped before the level is driven.
//! * An inbound `set_level` therefore takes the register lock, computes, drops
//!   it, and only then re-drives — so a peripheral may raise its request from
//!   inside its own register handler, with its own lock held.
//!
//! # What is not modelled
//!
//! * **Nothing is timed.** A request that can be forwarded is forwarded inside
//!   the wire event that brought it. The DMAMUX on the part is combinational
//!   for an unsynchronized channel and adds one clock of latency otherwise,
//!   which no firmware can observe without a scope.
//! * **`DMAMUX2`**, the H7's second instance in front of the BDMA, is this
//!   same class with a different `channels` and a different board table.
//!
//! `no_std + alloc`, no `unsafe`, no dependencies.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "st.dmamux";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The pin names this device answers to.
pub mod pin {
    /// The single overrun interrupt — the G4 vector table's `DMAMUX_OVR`.
    pub const IRQ: &str = "irq";
    /// A channel's request output, `ch0`…, wired to an `st.dma` `req` input.
    pub const CHANNEL: &str = "ch";
    /// A channel's event output, `evt0`…, pulsed when `EGE` is set.
    pub const EVENT: &str = "evt";
    /// A peripheral request input, `req5`…`req127`.
    pub const REQUEST: &str = "req";
    /// A synchronization input, `sync0`…`sync7`.
    pub const SYNC: &str = "sync";
    /// A request-generator trigger input, `trg0`…`trg31`.
    pub const TRIGGER: &str = "trg";
}

/// The most channels a DMAMUX serves — sixteen, on the largest parts.
///
/// `channels` narrows what an instance answers to; this is what
/// [`schema`] declares, because the validator cannot see a property's value.
pub const MAX_CHANNELS: usize = 16;

/// How many request generators the block has. Four in every manual that
/// describes it (RM0432 §14.5.5: `DMAMUX_RG0CR`…`DMAMUX_RG3CR`).
pub const GENERATORS: usize = 4;

/// How many request lines `DMAREQ_ID` can name: seven bits, so 128.
const REQUEST_LINES: usize = 1 << 7;

/// How many synchronization inputs `SYNC_ID` can name: three bits.
const SYNC_LINES: usize = 1 << 3;

/// How many trigger inputs `SIG_ID` can name: five bits.
const TRIGGER_LINES: usize = 1 << 5;

/// The request line generator `g` drives: line 1 for generator 0, and so on.
///
/// Lines 1 to 4 have no `req` pin for exactly this reason — see the module
/// documentation on the three ranges of `DMAREQ_ID`.
const fn generator_line(g: usize) -> usize {
    g + 1
}

/// The lowest request line a board may wire a pin to: 0 is the idle encoding
/// and 1 to 4 belong to the generators.
const FIRST_EXTERNAL_LINE: u32 = generator_line(GENERATORS - 1) as u32 + 1;

// ---------------------------------------------------------------------------
// register bits
// ---------------------------------------------------------------------------

// `DMAMUX_CxCR` (RM0432 §14.5.2).
const CCR_DMAREQ_ID: u32 = 0x7f;
const CCR_SOIE: u32 = 1 << 8;
const CCR_EGE: u32 = 1 << 9;
const CCR_SE: u32 = 1 << 16;
const CCR_SPOL_SHIFT: u32 = 17;
const CCR_NBREQ_SHIFT: u32 = 19;
const CCR_NBREQ_MASK: u32 = 0x1f;
const CCR_SYNC_ID_SHIFT: u32 = 24;
const CCR_SYNC_ID_MASK: u32 = 0x7;
/// Bits 7, 10–15, 27–31 are reserved and read as zero.
const CCR_MASK: u32 = 0x07ff_037f;

// `DMAMUX_RGxCR` (RM0432 §14.5.5).
const RGCR_SIG_ID: u32 = 0x1f;
const RGCR_OIE: u32 = 1 << 8;
const RGCR_GE: u32 = 1 << 16;
const RGCR_GPOL_SHIFT: u32 = 17;
const RGCR_GNBREQ_SHIFT: u32 = 19;
const RGCR_GNBREQ_MASK: u32 = 0x3;
/// Bits 5–7, 9–15, 21–31 are reserved and read as zero.
const RGCR_MASK: u32 = 0x001f_011f;

// Register offsets.
const OFF_CSR: u64 = 0x80;
const OFF_CFR: u64 = 0x84;
const OFF_RGCR: u64 = 0x100;
const OFF_RGSR: u64 = 0x140;
const OFF_RGCFR: u64 = 0x144;
/// How many bytes the register block decodes: through `DMAMUX_RGCFR`.
const WINDOW: u64 = OFF_RGCFR + 4;

/// Whether polarity code `pol` selects the edge `rising`.
///
/// RM0432 §14.5.2 and §14.5.5 give both fields the same encoding: `00` no
/// event — which disables the channel's synchronization or the generator
/// outright — `01` rising, `10` falling, `11` both.
const fn edge_selected(pol: u32, rising: bool) -> bool {
    match pol & 0x3 {
        0b01 => rising,
        0b10 => !rising,
        0b11 => true,
        // `00`: no event. Nothing triggers.
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the register block owns, plus the latches a running burst walks.
#[derive(Debug, Clone)]
struct State {
    /// `DMAMUX_CxCR`, as written.
    ccr: [u32; MAX_CHANNELS],
    /// `DMAMUX_RGxCR`, as written.
    rgcr: [u32; GENERATORS],
    /// `SOF`, one bit per channel, read through `DMAMUX_CSR`.
    sof: u32,
    /// `OF`, one bit per generator, read through `DMAMUX_RGSR`.
    of: u32,
    /// How many more requests a synchronized channel may still forward.
    /// Meaningless while `SE = 0`.
    credit: [u32; MAX_CHANNELS],
    /// How many requests a triggered generator still owes its line.
    owed: [u32; GENERATORS],
    /// Whether each channel is currently forwarding — the level `ch{x}` drives.
    forwarding: [bool; MAX_CHANNELS],
    /// The level each request line is at. Index is the `DMAREQ_ID` value, so
    /// entry 0 is the idle encoding and is never high; 1 to 4 are driven by
    /// [`Shared::emit`] and the rest by the `req` pins.
    request: [bool; REQUEST_LINES],
    /// The level each `sync` input is at, for edge detection.
    sync: [bool; SYNC_LINES],
    /// The level each `trg` input is at, for edge detection.
    trigger: [bool; TRIGGER_LINES],
}

impl State {
    const fn reset() -> State {
        State {
            ccr: [0; MAX_CHANNELS],
            rgcr: [0; GENERATORS],
            sof: 0,
            of: 0,
            credit: [0; MAX_CHANNELS],
            owed: [0; GENERATORS],
            forwarding: [false; MAX_CHANNELS],
            request: [false; REQUEST_LINES],
            sync: [false; SYNC_LINES],
            trigger: [false; TRIGGER_LINES],
        }
    }

    /// The request line channel `c` is programmed to listen to.
    fn source_line(&self, c: usize) -> usize {
        (self.ccr[c] & CCR_DMAREQ_ID) as usize
    }

    /// Whether channel `c` may forward a request right now.
    ///
    /// An unsynchronized channel always may. A synchronized one may only while
    /// the credit its last sync event granted has not run out — RM0432
    /// §14.3.3.
    fn may_forward(&self, c: usize) -> bool {
        self.ccr[c] & CCR_SE == 0 || self.credit[c] > 0
    }
}

// ---------------------------------------------------------------------------
// what an event changed
// ---------------------------------------------------------------------------

/// The outward work an update owes, collected under the lock and paid without
/// it.
///
/// Nothing here is optional bookkeeping: this struct **is** the re-entrancy
/// contract. A handler mutates [`State`], fills one of these in, drops the
/// lock, and hands it to [`Shared::apply`].
#[derive(Debug, Default, Clone, Copy)]
struct Update {
    /// Re-drive every channel's request level from `forwarding`.
    levels: bool,
    /// How many extra request pulses each channel owes — a generator's doing.
    pulses: [u32; MAX_CHANNELS],
    /// Channels whose `evt` output should pulse, one bit each.
    events: u32,
}

impl Update {
    /// An update that only re-drives the levels.
    const fn levels() -> Update {
        Update {
            levels: true,
            pulses: [0; MAX_CHANNELS],
            events: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// the shared object
// ---------------------------------------------------------------------------

/// The multiplexer's guts, shared between the device and its MMIO region.
struct Shared {
    /// How many channels this instance serves.
    channels: usize,
    /// Every register, at `DEVICE` rank. **Never held across a wire change**
    /// (`CLAUDE.md`, re-entrancy).
    state: Mutex<State>,
    /// Each channel's request output, at `WIRE` rank so it nests under
    /// `DEVICE`.
    ch: [Mutex<Option<WireSource>>; MAX_CHANNELS],
    /// Each channel's event output.
    evt: [Mutex<Option<WireSource>>; MAX_CHANNELS],
    /// The single overrun interrupt.
    irq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("channels", &self.channels);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish_non_exhaustive(),
            None => s.field("state", &"<in use>").finish_non_exhaustive(),
        }
    }
}

impl Shared {
    fn new(channels: usize) -> Shared {
        Shared {
            channels,
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            ch: core::array::from_fn(|_| Mutex::with_rank(LockRank::WIRE, None)),
            evt: core::array::from_fn(|_| Mutex::with_rank(LockRank::WIRE, None)),
            irq: Mutex::with_rank(LockRank::WIRE, None),
        }
    }

    // -- the multiplexer ----------------------------------------------------

    /// A request line moved. Recompute every channel that listens to it.
    ///
    /// This is the whole of the routing, and it is deliberately the shortest
    /// function in the file: for each channel, if `DMAREQ_ID` names this line,
    /// the channel's output becomes the line's level — gated, when `SE` is set,
    /// by the credit the last sync event granted.
    fn line_moved(&self, state: &mut State, line: usize) -> Update {
        let mut update = Update::levels();
        for c in 0..self.channels {
            if state.source_line(c) != line {
                continue;
            }
            if !state.request[line] {
                // The peripheral dropped its line: stop forwarding, whatever
                // mode the channel is in.
                state.forwarding[c] = false;
                continue;
            }
            if state.ccr[c] & CCR_SE == 0 {
                state.forwarding[c] = true;
                continue;
            }
            // Synchronized: a rising edge spends one unit of credit, and the
            // request is forwarded only if there was credit to spend.
            if state.credit[c] > 0 {
                state.forwarding[c] = true;
                self.spend(state, c, &mut update);
            }
        }
        update
    }

    /// Charge channel `c` one request against its sync credit, raising the
    /// event output if that was the last of the burst.
    fn spend(&self, state: &mut State, c: usize, update: &mut Update) {
        state.credit[c] -= 1;
        if state.credit[c] == 0 && state.ccr[c] & CCR_EGE != 0 {
            // RM0432 §14.3.4: the event goes out as the last of the `NBREQ + 1`
            // requests does.
            update.events |= 1 << c;
        }
    }

    /// A `sync` input moved: hand out credit to the channels watching it.
    fn sync_moved(&self, state: &mut State, line: usize, rising: bool) -> Update {
        let mut update = Update::levels();
        for c in 0..self.channels {
            let ccr = state.ccr[c];
            if ccr & CCR_SE == 0 {
                continue;
            }
            if ((ccr >> CCR_SYNC_ID_SHIFT) & CCR_SYNC_ID_MASK) as usize != line {
                continue;
            }
            if !edge_selected(ccr >> CCR_SPOL_SHIFT, rising) {
                continue;
            }
            if state.credit[c] > 0 {
                // RM0432 §14.3.3: a sync event arriving before the previous
                // burst was spent is a synchronization overrun. The new event
                // is taken anyway; the flag records what was lost.
                state.sof |= 1 << c;
            }
            state.credit[c] = ((ccr >> CCR_NBREQ_SHIFT) & CCR_NBREQ_MASK) + 1;
            // A peripheral that has been holding its line high all along is
            // asking right now, so the first of the burst goes out on the sync
            // edge itself rather than waiting for an edge that will never come.
            if state.request[state.source_line(c)] {
                state.forwarding[c] = true;
                self.spend(state, c, &mut update);
            }
        }
        update
    }

    /// A `trg` input moved: arm the generators watching it, then let them emit.
    fn trigger_moved(&self, state: &mut State, line: usize, rising: bool) -> Update {
        let mut update = Update::levels();
        for g in 0..GENERATORS {
            let rgcr = state.rgcr[g];
            if rgcr & RGCR_GE == 0 {
                continue;
            }
            if (rgcr & RGCR_SIG_ID) as usize != line {
                continue;
            }
            if !edge_selected(rgcr >> RGCR_GPOL_SHIFT, rising) {
                continue;
            }
            if state.owed[g] > 0 {
                // RM0432 §14.3.5: a trigger arriving while the generator still
                // owes requests is a trigger overrun. In this model that means
                // nothing is listening to the generator's line — see the module
                // documentation.
                state.of |= 1 << g;
            }
            state.owed[g] = ((rgcr >> RGCR_GNBREQ_SHIFT) & RGCR_GNBREQ_MASK) + 1;
            self.emit(state, g, &mut update);
        }
        update
    }

    /// Pay out generator `g`'s owed requests, as pulses on its request line.
    ///
    /// A generator has no input level to pass on, so each request is a pulse
    /// rather than a level: the line goes high and straight back down, once per
    /// request. If no channel is in a position to take one, the debt stays on
    /// the books and the next trigger is an overrun.
    fn emit(&self, state: &mut State, g: usize, update: &mut Update) {
        let line = generator_line(g);
        while state.owed[g] > 0 {
            let mut taken = false;
            for c in 0..self.channels {
                if state.source_line(c) != line || !state.may_forward(c) {
                    continue;
                }
                update.pulses[c] += 1;
                if state.ccr[c] & CCR_SE != 0 {
                    self.spend(state, c, update);
                }
                taken = true;
            }
            if !taken {
                // Nobody selects this generator. Leave the rest owed.
                return;
            }
            state.owed[g] -= 1;
        }
    }

    /// Recompute every channel's `forwarding` latch from the register image.
    ///
    /// Called after a control-register write and after a snapshot load, where
    /// there is no edge to react to but the routing may have changed entirely.
    /// A synchronized channel starts from no credit: RM0432 §14.3.3 makes the
    /// sync event, not the write, what opens the gate.
    fn resettle(&self, state: &mut State) {
        for c in 0..self.channels {
            state.forwarding[c] = state.request[state.source_line(c)] && state.may_forward(c);
        }
    }

    // -- driving the outputs ------------------------------------------------

    /// The level each channel's request output should be driving.
    fn levels(&self, state: &State) -> [Level; MAX_CHANNELS] {
        core::array::from_fn(|c| Level::from_bool(c < self.channels && state.forwarding[c]))
    }

    /// Whether the overrun interrupt should be asserted.
    ///
    /// One pin for both kinds of overrun, because the part has one vector.
    fn irq_level(&self, state: &State) -> Level {
        for c in 0..self.channels {
            if state.sof & (1 << c) != 0 && state.ccr[c] & CCR_SOIE != 0 {
                return Level::High;
            }
        }
        for g in 0..GENERATORS {
            if state.of & (1 << g) != 0 && state.rgcr[g] & RGCR_OIE != 0 {
                return Level::High;
            }
        }
        Level::Low
    }

    /// Pay out an [`Update`] with **nothing of ours held**.
    ///
    /// The order matters: levels before pulses, so a channel that both settles
    /// to a new level and owes generator pulses does not deliver the pulses
    /// into a stale level and lose them.
    fn apply(&self, update: Update) {
        if update.levels {
            let levels = {
                let state = self.state.lock();
                self.levels(&state)
            };
            for (c, &level) in levels.iter().enumerate().take(self.channels) {
                self.drive(&self.ch[c], level);
            }
        }
        for c in 0..self.channels {
            if update.pulses[c] > 0 {
                // Cloned out and the guard dropped *before* the pulse: an
                // `if let` on the guard itself would hold a `WIRE` lock across
                // the delivery, and a sink that takes a `DEVICE` lock of its
                // own — every device does — would invert the ranking.
                let source = self.ch[c].lock().clone();
                if let Some(source) = source {
                    for _ in 0..update.pulses[c] {
                        source.pulse(Level::High);
                    }
                }
            }
            if update.events & (1 << c) != 0 {
                let source = self.evt[c].lock().clone();
                if let Some(source) = source {
                    source.pulse(Level::High);
                }
            }
        }
        self.refresh_irq();
    }

    /// Drive one output with nothing of ours held.
    fn drive(&self, slot: &Mutex<Option<WireSource>>, level: Level) {
        let source = slot.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Recompute and drive the overrun interrupt.
    fn refresh_irq(&self) {
        let level = {
            let state = self.state.lock();
            self.irq_level(&state)
        };
        self.drive(&self.irq, level);
    }

    /// Re-drive every output from the state as it stands.
    fn refresh(&self) {
        self.apply(Update::levels());
    }

    // -- register decode ----------------------------------------------------

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            OFF_CSR => return state.sof,
            // The clear registers are write-only and read as zero.
            OFF_CFR | OFF_RGCFR => return 0,
            OFF_RGSR => return state.of,
            _ => {}
        }
        if offset < OFF_CSR {
            let c = (offset / 4) as usize;
            return if c < self.channels {
                state.ccr[c] & CCR_MASK
            } else {
                0
            };
        }
        if (OFF_RGCR..OFF_RGCR + 4 * GENERATORS as u64).contains(&offset) {
            let g = ((offset - OFF_RGCR) / 4) as usize;
            return state.rgcr[g] & RGCR_MASK;
        }
        0
    }

    /// Apply a register write, returning the outward work it owes.
    fn write_register(&self, offset: u64, value: u32) -> Update {
        let mut state = self.state.lock();
        match offset {
            // `CSR` and `RGSR` are read-only status.
            OFF_CSR | OFF_RGSR => return Update::default(),
            OFF_CFR => {
                state.sof &= !value;
                return Update::default();
            }
            OFF_RGCFR => {
                state.of &= !value;
                return Update::default();
            }
            _ => {}
        }
        if offset < OFF_CSR {
            let c = (offset / 4) as usize;
            if c < self.channels {
                state.ccr[c] = value & CCR_MASK;
                // Re-pointing a channel at another line, or turning
                // synchronization on, changes what it forwards this instant.
                state.credit[c] = 0;
                self.resettle(&mut state);
            }
            return Update::levels();
        }
        if (OFF_RGCR..OFF_RGCR + 4 * GENERATORS as u64).contains(&offset) {
            let g = ((offset - OFF_RGCR) / 4) as usize;
            state.rgcr[g] = value & RGCR_MASK;
            if state.rgcr[g] & RGCR_GE == 0 {
                // Disabling a generator abandons whatever it still owed, which
                // is also what stops a stale debt from reporting an overrun the
                // guest could not have caused.
                state.owed[g] = 0;
            }
            return Update::levels();
        }
        Update::default()
    }

    // -- the inbound wires --------------------------------------------------

    /// Record a request line's new level and route it.
    fn set_request(&self, line: usize, level: Level) {
        let update = {
            let mut state = self.state.lock();
            let high = level == Level::High;
            if line == 0 || line >= REQUEST_LINES || state.request[line] == high {
                return;
            }
            state.request[line] = high;
            self.line_moved(&mut state, line)
        };
        self.apply(update);
    }

    /// Record a synchronization input's new level and hand out credit.
    fn set_sync(&self, line: usize, level: Level) {
        let update = {
            let mut state = self.state.lock();
            let high = level == Level::High;
            if line >= SYNC_LINES || state.sync[line] == high {
                return;
            }
            state.sync[line] = high;
            self.sync_moved(&mut state, line, high)
        };
        self.apply(update);
    }

    /// Record a trigger input's new level and run the generators.
    fn set_trigger(&self, line: usize, level: Level) {
        let update = {
            let mut state = self.state.lock();
            let high = level == Level::High;
            if line >= TRIGGER_LINES || state.trigger[line] == high {
                return;
            }
            state.trigger[line] = high;
            self.trigger_moved(&mut state, line, high)
        };
        self.apply(update);
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
        // Nothing here has a read side effect — the overrun flags are cleared
        // through `CFR`/`RGCFR` — so a debug read is the same read
        // (`ROADMAP.md` §15, invariant 5).
        let bytes = self.shared.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CFR` would drop an overrun the guest has not
            // seen, and one to `CxCR` would re-route a live request. Neither
            // can be made harmless, so it is refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        let update = self
            .shared
            .write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        self.shared.apply(update);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The DMAMUX registers have to be accessed by words (32 bits)" —
        // RM0432 §14.5.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// input pins
// ---------------------------------------------------------------------------

/// Which bank an input pin belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bank {
    /// A peripheral request line, `req{n}`.
    Request,
    /// A synchronization input, `sync{n}`.
    Sync,
    /// A request-generator trigger, `trg{n}`.
    Trigger,
}

/// One input pin, as something a wire can drive.
///
/// Keeps a [`FanIn`] because a wire hands each sink the level of the *driver
/// that changed* rather than the resolved level of the net, and two
/// peripherals sharing one request line is a thing a part does.
#[derive(Debug)]
struct InputPin {
    shared: Arc<Shared>,
    bank: Bank,
    line: u32,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let level = self.inputs.resolve(Resolve::Or);
        let line = self.line as usize;
        match self.bank {
            Bank::Request => self.shared.set_request(line, level),
            Bank::Sync => self.shared.set_sync(line, level),
            Bank::Trigger => self.shared.set_trigger(line, level),
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An STM32 DMA request multiplexer.
#[derive(Debug)]
pub struct Dmamux {
    shared: Arc<Shared>,
    region: RegionRef,
    /// The input pins the machine layer has taken. The device keeps the strong
    /// reference: a net holds its sinks weakly.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Dmamux {
    /// Validate `props` and build the multiplexer.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Dmamux> {
        let mut r = props.reader();
        let channels = r.or_range("channels", 7u64, 1..=MAX_CHANNELS as u64)? as usize;
        r.finish()?;
        Ok(Dmamux::with_channels(channels))
    }

    /// Build one with `channels` channels — the route a test takes.
    ///
    /// `channels` is clamped to `1..=`[`MAX_CHANNELS`].
    #[must_use]
    pub fn with_channels(channels: usize) -> Dmamux {
        let channels = channels.clamp(1, MAX_CHANNELS);
        let shared = Arc::new(Shared::new(channels));
        let region = Arc::new(Region::io(
            "dmamux",
            WINDOW,
            Arc::new(Registers {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Dmamux {
            shared,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// How many channels this instance serves.
    #[must_use]
    pub fn channels(&self) -> usize {
        self.shared.channels
    }

    /// Connect channel `c`'s request output.
    pub fn connect_channel(&self, c: usize, source: WireSource) {
        if c < self.shared.channels {
            *self.shared.ch[c].lock() = Some(source);
            self.shared.refresh();
        }
    }

    /// Connect channel `c`'s event output.
    pub fn connect_event(&self, c: usize, source: WireSource) {
        if c < self.shared.channels {
            *self.shared.evt[c].lock() = Some(source);
        }
    }

    /// Connect the overrun interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
        self.shared.refresh_irq();
    }

    /// Drive request line `line` directly, as a wired peripheral would.
    ///
    /// Lines 0 to [`GENERATORS`] are not a peripheral's to drive — 0 is the
    /// idle encoding and the rest belong to the request generators — and a
    /// call naming one does nothing.
    pub fn set_request(&self, line: usize, level: Level) {
        if line >= FIRST_EXTERNAL_LINE as usize {
            self.shared.set_request(line, level);
        }
    }

    /// Drive synchronization input `line` directly.
    pub fn set_sync(&self, line: usize, level: Level) {
        self.shared.set_sync(line, level);
    }

    /// Drive trigger input `line` directly.
    pub fn set_trigger(&self, line: usize, level: Level) {
        self.shared.set_trigger(line, level);
    }

    /// The level channel `c`'s request output is currently at.
    #[must_use]
    pub fn forwarding(&self, c: usize) -> bool {
        c < self.shared.channels && self.shared.state.lock().forwarding[c]
    }

    /// The index an input pin name selects, and which bank it is in.
    fn input_index(&self, port: &str) -> Option<(Bank, u32)> {
        if let Some(n) = port_index(port, pin::REQUEST, REQUEST_LINES as u32) {
            // Lines 0 to 4 have no pin: the module documentation says why.
            return (n >= FIRST_EXTERNAL_LINE).then_some((Bank::Request, n));
        }
        if let Some(n) = port_index(port, pin::SYNC, SYNC_LINES as u32) {
            return Some((Bank::Sync, n));
        }
        port_index(port, pin::TRIGGER, TRIGGER_LINES as u32).map(|n| (Bank::Trigger, n))
    }
}

impl Device for Dmamux {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the register block and
        // `wire` statements place the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::reset();
        // Every output idles low, and `refresh` is what says so out loud —
        // a channel that was forwarding when the reset arrived has a wire
        // still holding high otherwise.
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock().clone();
        w.write_u8(self.shared.channels as u8)?;
        for c in 0..self.shared.channels {
            w.write_u32(state.ccr[c])?;
            w.write_u32(state.credit[c])?;
            w.write_bool(state.forwarding[c])?;
        }
        for g in 0..GENERATORS {
            w.write_u32(state.rgcr[g])?;
            w.write_u32(state.owed[g])?;
        }
        w.write_u32(state.sof)?;
        w.write_u32(state.of)?;
        // The input levels are guest-visible through their effect: a channel
        // resumes forwarding only if its peripheral is still asking, and an
        // edge detector that forgot where its input was would invent an edge
        // on the next change. So the levels travel with the state.
        for word in 0..REQUEST_LINES / 32 {
            let mut bits = 0u32;
            for bit in 0..32 {
                if state.request[word * 32 + bit] {
                    bits |= 1 << bit;
                }
            }
            w.write_u32(bits)?;
        }
        let mut sync = 0u8;
        for (i, &level) in state.sync.iter().enumerate() {
            if level {
                sync |= 1 << i;
            }
        }
        w.write_u8(sync)?;
        let mut trigger = 0u32;
        for (i, &level) in state.trigger.iter().enumerate() {
            if level {
                trigger |= 1 << i;
            }
        }
        w.write_u32(trigger)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let channels = usize::from(r.read_u8()?);
        if channels != self.shared.channels {
            return Err(Error::State(format!(
                "snapshot has {channels} channels, this multiplexer has {}",
                self.shared.channels
            )));
        }
        let mut state = State::reset();
        for c in 0..channels {
            state.ccr[c] = r.read_u32()?;
            state.credit[c] = r.read_u32()?;
            state.forwarding[c] = r.read_bool()?;
        }
        for g in 0..GENERATORS {
            state.rgcr[g] = r.read_u32()?;
            state.owed[g] = r.read_u32()?;
        }
        state.sof = r.read_u32()?;
        state.of = r.read_u32()?;
        for word in 0..REQUEST_LINES / 32 {
            let bits = r.read_u32()?;
            for bit in 0..32 {
                state.request[word * 32 + bit] = bits & (1 << bit) != 0;
            }
        }
        // Line 0 is the idle encoding and is never high, whatever a snapshot
        // claims: a channel parked on `DMAREQ_ID = 0` must stay parked.
        state.request[0] = false;
        let sync = r.read_u8()?;
        for (i, level) in state.sync.iter_mut().enumerate() {
            *level = sync & (1 << i) != 0;
        }
        let trigger = r.read_u32()?;
        for (i, level) in state.trigger.iter_mut().enumerate() {
            *level = trigger & (1 << i) != 0;
        }
        *self.shared.state.lock() = state;
        self.shared.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port == pin::IRQ {
            self.connect_irq(source);
            return Ok(());
        }
        if let Some(c) = port_index(port, pin::CHANNEL, self.shared.channels as u32) {
            self.connect_channel(c as usize, source);
            return Ok(());
        }
        if let Some(c) = port_index(port, pin::EVENT, self.shared.channels as u32) {
            self.connect_event(c as usize, source);
            return Ok(());
        }
        Err(Error::Config {
            at: port.to_string(),
            message: format!(
                "a `{CLASS_NAME}` drives `{}0`…`{}{last}`, `{}0`…`{}{last}` and `{}`",
                pin::CHANNEL,
                pin::CHANNEL,
                pin::EVENT,
                pin::EVENT,
                pin::IRQ,
                last = self.shared.channels - 1
            ),
        })
    }

    fn announce(&self, port: &str) {
        // A machine that wires an output after a snapshot load has to be told
        // about a level that survived it.
        if port == pin::IRQ {
            self.shared.refresh_irq();
        } else if port_index(port, pin::CHANNEL, self.shared.channels as u32).is_some() {
            self.shared.refresh();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let (bank, line) = self.input_index(port)?;
        let pin = Arc::new(InputPin {
            shared: Arc::clone(&self.shared),
            bank,
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }
}

impl Instance for Dmamux {}

/// The `st.dmamux` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 DMA request multiplexer (RM0432 §14): CxCR routing, synchronization and \
              the four request generators",
    properties: &[PropertySpec {
        name: "channels",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many DMA channels this multiplexer feeds (7 by default, up to 16)",
    }],
    construct: |props| Ok(Box::new(Dmamux::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Dmamux::new(props)?)))
}

/// What the validator should know about `st.dmamux`.
///
/// The banks are declared at the widest part's size, because `channels`
/// narrows what the *device* accepts and the validator cannot see a property's
/// value — the same argument [`crate::dev::stm32::exti`] makes about its lines.
/// The `req` bank likewise declares all 128 spellings and the device refuses
/// the first five, which is where the knowledge that they belong to the
/// generators lives.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("channels", ValueKind::Uint).range(1, MAX_CHANNELS as u64))
        .region("")
        .region("regs")
        .port_bank(pin::REQUEST, PortDir::In, REQUEST_LINES as u32)
        .port_bank(pin::SYNC, PortDir::In, SYNC_LINES as u32)
        .port_bank(pin::TRIGGER, PortDir::In, TRIGGER_LINES as u32)
        .port_bank(pin::CHANNEL, PortDir::Out, MAX_CHANNELS as u32)
        .port_bank(pin::EVENT, PortDir::Out, MAX_CHANNELS as u32)
        .port(pin::IRQ, PortDir::Out)
}

#[cfg(test)]
mod tests;
