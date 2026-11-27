//! The S-CPU's on-chip peripherals: the 5A22's registers, DMA and HDMA, the
//! timers, the multiplier and divider, the joypad ports, and the bus speed.
//!
//! The Ricoh 5A22 is a 65C816 core ([`cpu::w65c816`](crate::cpu::w65c816))
//! with a set of peripherals on the same die. The core is its own device; this
//! is everything else, as one device because it is one block of logic that
//! shares one register decoder and one view of the beam:
//!
//! | Window | Registers |
//! | --- | --- |
//! | `joy` (`$4016`–`$4017`) | `JOYSER0`/`JOYSER1`: the manual pad latch and the two serial data lines |
//! | `regs` (`$4200`–`$421F`) | `NMITIMEN`, `WRIO`, the multiplier and divider, `HTIME`/`VTIME`, `MDMAEN`, `HDMAEN`, `MEMSEL`; `RDNMI`, `TIMEUP`, `HVBJOY`, `RDIO`, the results, `JOY1`–`JOY4` |
//! | `dma` (`$4300`–`$437F`) | eight channels of `DMAPx` … `NLTRx` |
//!
//! The board maps the three windows into every system bank (`$00–$3F`,
//! `$80–$BF`). It also names this device as the core's `rdy` gate and its
//! `timing` provider, and that is how the rest of the chip reaches the core.
//!
//! # Bus speed: `timing`
//!
//! The 5A22 runs each bus cycle at the speed of the address it drives
//! (fullsnes, "SNES Timings"; SNESdev wiki, "Timing"). This device publishes
//! [`ExportId::ACCESS_TIMING`] and answers, in master clocks:
//!
//! | Address | Clocks |
//! | --- | --- |
//! | banks `$00–$3F`/`$80–$BF`: `$0000–$1FFF`, `$6000–$7FFF` | 8 |
//! | the same banks: `$2000–$3FFF`, `$4200–$5FFF` | 6 |
//! | the same banks: `$4000–$41FF` (the old joypad ports) | 12 |
//! | `$8000–$FFFF` of `$00–$3F`, and all of `$40–$7F` | 8 |
//! | `$8000–$FFFF` of `$80–$BF`, and all of `$C0–$FF` | 6 with `MEMSEL` bit 0 (FastROM), else 8 |
//! | an internal operation | 6 |
//!
//! # Stalls: `rdy`
//!
//! DMA, HDMA and DRAM refresh all stop the core. They share one counter of
//! **owed master clocks**: each adds what it costs, and the core pays it off
//! on its next held cycle. This device is the core's
//! [`CycleGate`], which returns [`Arbitration::Steal`] while anything is owed
//! or a general-purpose DMA is moving, and [`AccessTiming`] prices a held
//! cycle at whatever is owed (6 if nothing is, which is a `WAI` cycle).
//!
//! * **General-purpose DMA** starts on the cycle after the `MDMAEN` write
//!   and moves one byte per held cycle, each costing 8 clocks, with 8 for the
//!   start and 8 per channel (fullsnes, "SNES DMA and HDMA Timings"). Doing
//!   it a byte per cycle keeps every write to the PPU at its own time.
//! * **HDMA** is beam-driven, so it runs from this device's own scheduled
//!   events rather than from the core: the table initialisation at line 0,
//!   dot 6, and one transfer per active channel at dot 278 of each line
//!   before vertical blank. A core sitting in `WAI` still gets its HDMA.
//!   Costs: 18 clocks of overhead plus 8 per channel (and 16 for an
//!   indirect address) at initialisation; 18, plus 8 per active channel,
//!   8 per byte and 16 per indirect address reload, on each line.
//! * **DRAM refresh** takes 40 clocks of every line, at dot 134 (master clock
//!   538).
//!
//! The transfers themselves are ordinary accesses to the A-bus space with
//! this device's requester id; the B-bus is the A-bus window `$002100+bb`,
//! which is where the board maps the PPU, the APU ports and WRAM's port. DMA
//! cannot reach the S-CPU's own registers or the B-bus through the A-bus side
//! (fullsnes: `$2100–$21FF`, `$4000–$43FF` of the system banks): a read there
//! returns the DMA's open bus and a write goes nowhere.
//!
//! # NMI, IRQ and the joypads
//!
//! * The PPU's `vblank` level comes in on the `vblank` pin. Its rising edge
//!   sets `RDNMI` bit 7 (cleared by reading `$4210`, and at the end of
//!   vertical blank); the `nmi` output is `NMITIMEN` bit 7 AND that flag, so
//!   enabling NMI while the flag stands raises one, as on the part.
//! * `NMITIMEN` bits 5–4 select the timer: H (every line at `HTIME`), V (line
//!   `VTIME`, dot 0) or both. When it fires `TIMEUP` bit 7 sets and the `irq`
//!   output follows it until `$4211` is read or the timer is disabled.
//! * With `NMITIMEN` bit 0, the pads are read automatically at the start of
//!   vertical blank into `JOY1`/`JOY2`; `HVBJOY` bit 0 reads busy for 4224
//!   clocks afterwards. `JOY3`/`JOY4` read zero (no multitap).
//! * `$4016` bit 0 is the latch: while it is high the shift registers reload
//!   continuously, and each read of `$4016`/`$4017` shifts out one bit, B
//!   first, with ones after the sixteenth. The pads come through
//!   [`input::pads`](super::input::pads), named by the `pads` property.
//!
//! # One clock for the beam events
//!
//! HDMA, DRAM refresh and the H/V timer are beam positions, and the beam is
//! the PPU's. But this device and the PPU are two lazily-advanced devices,
//! each caught up when something needs it, and they are rarely at the same
//! tick: the core runs a whole bus cycle (or a whole DMA stall) past one of
//! this device's events before the scheduler delivers it, and anything that
//! touches the PPU meanwhile has moved the PPU on.
//!
//! So an event is never "the PPU's distance to that dot, counted from here".
//! Both chips hang off the same crystal with the same divider (the board
//! gives them `clock = master`), which makes their ticks the same numbers,
//! and the PPU publishes the [`Frame`] its beam is in: where line 0 began, as
//! a master tick, and what decides the length of each line. From that, "dot
//! `h` of line `v`" is an absolute tick by integer arithmetic, and it is the
//! same tick wherever either chip has been caught up to. This device keeps
//! the frame its own tick is in (`State::frame`), so that it still has it
//! when the PPU has gone on into the next one.
//!
//! # Known deviations
//!
//! * The multiplier and divider answer **at once**; the hardware takes 8 and
//!   16 CPU cycles, and a read in between sees a partial result.
//! * The H/V IRQ fires at the start of dot `HTIME`, not the few dots later
//!   fullsnes measures, and the `NMITIMEN`/`HTIME` write races near the
//!   trigger point are not modelled.
//! * DMA's alignment to the CPU clock at start and end is folded into the
//!   fixed 8-clock overhead.
//! * A GP DMA and an HDMA on the same channel do not interfere; an `HDMAEN`
//!   write mid-frame takes effect at the next frame's initialisation.
//!
//! Sources: nocash **fullsnes** ("SNES I/O Map", "SNES DMA and HDMA", "SNES
//! Timings", "SNES Controllers", "SNES Maths Multiply/Divide"); the **SNESdev
//! wiki** ("MMIO registers", "DMA registers", "Timing"). No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{
    Arbitration, CycleGate, Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx,
    ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
    RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::cpu::w65c816::{AccessTiming, BusCycle, Pins, TimingPort};
use crate::dev::snes::input::{Pad, pads};
use crate::dev::snes::ppu::timing::{VBLANK_LINE, VBLANK_LINE_OVERSCAN};
use crate::dev::snes::ppu::{Frame, SnesPpuBeam};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "snes.scpu";

/// `$4200`–`$421F`.
pub const REGS_REGION: &str = "regs";
/// `$4300`–`$437F`.
pub const DMA_REGION: &str = "dma";
/// `$4016`–`$4017`.
pub const JOY_REGION: &str = "joy";

/// The NMI output, to the core's `nmi`.
pub const NMI_PIN: &str = "nmi";
/// The IRQ output, to the core's `irq`.
pub const IRQ_PIN: &str = "irq";
/// The PPU's vertical-blank level, in.
pub const VBLANK_PIN: &str = "vblank";

// 2: the chunk gained the frame this device's own tick is in; see
// [`migrations`].
const STATE_VERSION: u32 = 2;
const DEFAULT_PADS: &str = "snes-pads";

/// The CPU version `RDNMI` reports in bits 3–0 (fullsnes: "5A22 version
/// number (version 2 exists)").
const CPU_VERSION: u8 = 2;

/// What a held cycle costs when nothing is owed: a `WAI` cycle.
const IDLE_CLOCKS: u64 = 6;

const DMA_BYTE: u64 = 8;
const DMA_START: u64 = 8;
const DMA_CHANNEL: u64 = 8;
const HDMA_OVERHEAD: u64 = 18;
const HDMA_CHANNEL: u64 = 8;
const HDMA_INDIRECT: u64 = 16;
const REFRESH_CLOCKS: u64 = 40;
const AUTO_JOY_CLOCKS: u64 = 4224;

/// How many frames a search for a beam position looks through: the one the
/// tick is in and the two after it, which is both interlace fields.
const FRAMES_AHEAD: usize = 3;
/// How many frames apart this device and the PPU may be before the frames in
/// between stop being reconstructed one by one. No scheduler lets them get
/// that far; a unit test that advances only one of them does.
const FRAMES_APART: usize = 4;

/// The beam events, in the order simultaneous ones fire.
const EV_INIT: u8 = 0;
const EV_LINE: u8 = 1;
const EV_REFRESH: u8 = 2;
const EV_IRQ: u8 = 3;

const HDMA_INIT_DOT: u16 = 6;
const HDMA_DOT: u16 = 278;
const REFRESH_DOT: u16 = 134;

/// The B-bus offsets a DMA unit visits, by `DMAPx` bits 2–0 (fullsnes, "DMA
/// transfer unit"). Modes 6 and 7 repeat 2 and 3.
const PATTERNS: [&[u8]; 8] = [
    &[0],
    &[0, 1],
    &[0, 0],
    &[0, 0, 1, 1],
    &[0, 1, 2, 3],
    &[0, 1, 0, 1],
    &[0, 0],
    &[0, 0, 1, 1],
];

/// One DMA channel's registers and its HDMA progress.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Channel {
    dmap: u8,
    bbad: u8,
    a1t: u16,
    a1b: u8,
    das: u16,
    dasb: u8,
    a2a: u16,
    nltr: u8,
    unused: u8,
    /// HDMA: transfer on the coming line.
    do_transfer: bool,
    /// HDMA: still running this frame.
    hdma_live: bool,
}

impl Channel {
    const fn pattern(&self) -> &'static [u8] {
        PATTERNS[(self.dmap & 7) as usize]
    }
    const fn b_to_a(&self) -> bool {
        self.dmap & 0x80 != 0
    }
    const fn indirect(&self) -> bool {
        self.dmap & 0x40 != 0
    }
}

/// The register file and everything that is not hot-path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    nmitimen: u8,
    wrio: u8,
    wrmpya: u8,
    wrdiv: u16,
    htime: u16,
    vtime: u16,
    mdmaen: u8,
    hdmaen: u8,
    rdnmi: bool,
    timeup: bool,
    rddiv: u16,
    rdmpy: u16,
    joy: [u16; 2],
    joy_busy_until: u64,
    strobe: bool,
    shift: [u16; 2],
    /// How many bits each shift register has given since the latch.
    shifted: [u8; 2],
    ch: [Channel; 8],
    /// GP DMA: the channel in progress and how far into its unit.
    dma_channel_started: bool,
    dma_byte: u32,
    /// The DMA's own data-bus latch.
    mdr: u8,
    /// The frame this device's own tick is in, less the region (which is the
    /// PPU's to say): where it started, its field, whether it is interlaced
    /// and how many lines it has. `None` until the first time the beam is
    /// asked, and after a reset.
    frame: Option<(u64, bool, bool, u16)>,
    /// The next tick each beam event falls on.
    next_init: u64,
    next_line: u64,
    next_refresh: u64,
    next_irq: Option<u64>,
    /// The level the nmi and irq outputs are at.
    nmi_level: bool,
    irq_level: bool,
    vblank: bool,
}

impl State {
    fn reset() -> State {
        State {
            nmitimen: 0,
            wrio: 0xFF,
            wrmpya: 0xFF,
            wrdiv: 0xFFFF,
            htime: 0x1FF,
            vtime: 0x1FF,
            mdmaen: 0,
            hdmaen: 0,
            rdnmi: false,
            timeup: false,
            rddiv: 0,
            rdmpy: 0,
            joy: [0; 2],
            joy_busy_until: 0,
            strobe: false,
            shift: [0; 2],
            shifted: [16; 2],
            ch: [Channel {
                dmap: 0xFF,
                bbad: 0xFF,
                a1t: 0xFFFF,
                a1b: 0xFF,
                das: 0xFFFF,
                dasb: 0xFF,
                a2a: 0xFFFF,
                nltr: 0xFF,
                unused: 0xFF,
                do_transfer: false,
                hdma_live: false,
            }; 8],
            dma_channel_started: false,
            dma_byte: 0,
            mdr: 0,
            frame: None,
            next_init: 0,
            next_line: 0,
            next_refresh: 0,
            next_irq: None,
            nmi_level: false,
            irq_level: false,
            vblank: false,
        }
    }

    fn want_nmi(&self) -> bool {
        self.nmitimen & 0x80 != 0 && self.rdnmi
    }
}

struct Shared {
    state: Mutex<State>,
    pad: Arc<Pad>,
    /// FastROM: `MEMSEL` bit 0, read on every bus cycle.
    memsel: AtomicBool,
    /// Master clocks the core owes.
    owed: AtomicU64,
    /// A GP DMA is pending or moving: the gate's fast check.
    dma_active: AtomicBool,
    tick: AtomicU64,
    next_event: AtomicU64,
    requester: AtomicU32,
    space: Mutex<Option<Weak<AddressSpace>>>,
    beam: Mutex<Option<Arc<SnesPpuBeam>>>,
    nmi: Mutex<Option<WireSource>>,
    irq: Mutex<Option<WireSource>>,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("tick", &self.tick.load(Ordering::Relaxed))
            .field("owed", &self.owed.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

/// How a DMA moves one byte.
#[derive(Debug, Clone, Copy)]
struct Move {
    a: u32,
    b: u8,
    b_to_a: bool,
}

const fn a_bus_reachable(addr: u32) -> bool {
    let bank = (addr >> 16) as u8;
    let lo = addr & 0xFFFF;
    !(bank & 0x40 == 0 && ((lo >= 0x2100 && lo <= 0x21FF) || (lo >= 0x4000 && lo <= 0x43FF)))
}

impl Shared {
    fn attrs(&self, mdr: u8) -> MemAttrs {
        MemAttrs::DEFAULT
            .with_requester(RequesterId(self.requester.load(Ordering::Relaxed)))
            .with_bus(mdr)
    }

    fn space(&self) -> Option<Arc<AddressSpace>> {
        self.space.lock().as_ref().and_then(Weak::upgrade)
    }

    fn beam(&self) -> Option<Arc<SnesPpuBeam>> {
        self.beam.lock().clone()
    }

    /// Perform one DMA byte: no lock held. Returns the byte that crossed.
    fn transfer(&self, space: &AddressSpace, m: Move, mdr: u8) -> u8 {
        let b_addr = 0x2100 | u64::from(m.b);
        let attrs = self.attrs(mdr);
        if m.b_to_a {
            let byte = space
                .read(b_addr, Width::U8, attrs)
                .map_or(mdr, |v| v as u8);
            if a_bus_reachable(m.a) {
                let _ = space.write(u64::from(m.a), Width::U8, u64::from(byte), attrs);
            }
            byte
        } else {
            let byte = if a_bus_reachable(m.a) {
                space
                    .read(u64::from(m.a), Width::U8, attrs)
                    .map_or(mdr, |v| v as u8)
            } else {
                mdr
            };
            let _ = space.write(b_addr, Width::U8, u64::from(byte), attrs);
            byte
        }
    }

    fn read_table(&self, space: &AddressSpace, addr: u32, mdr: u8) -> u8 {
        if !a_bus_reachable(addr) {
            return mdr;
        }
        space
            .read(u64::from(addr), Width::U8, self.attrs(mdr))
            .map_or(mdr, |v| v as u8)
    }

    /// Drive `nmi` and `irq` to what the state says. No lock held.
    fn drive_lines(&self) {
        let (nmi, irq, changed_nmi, changed_irq) = {
            let mut s = self.state.lock();
            let nmi = s.want_nmi();
            let irq = s.timeup;
            let c = (nmi != s.nmi_level, irq != s.irq_level);
            s.nmi_level = nmi;
            s.irq_level = irq;
            (nmi, irq, c.0, c.1)
        };
        if changed_nmi && let Some(w) = self.nmi.lock().clone() {
            w.set(Level::from_bool(nmi));
        }
        if changed_irq && let Some(w) = self.irq.lock().clone() {
            w.set(Level::from_bool(irq));
        }
    }

    fn publish_next(&self, s: &State) {
        let mut next = s.next_init.min(s.next_line).min(s.next_refresh);
        if let Some(irq) = s.next_irq {
            next = next.min(irq);
        }
        let tick = self.tick.load(Ordering::Relaxed);
        self.next_event
            .store(next.max(tick.saturating_add(1)), Ordering::Relaxed);
    }

    /// The frame master tick `t` of this device's own clock falls in, and the
    /// interlace setting the frames after it are projected with.
    ///
    /// The PPU is the authority: whenever the frame it is in holds `t`, that
    /// is the answer. When it has gone on into a later frame, the answer is
    /// the frame this device was last in (or the ones that follow from it);
    /// when it is behind, the ones that follow from the PPU's.
    fn frame_at(&self, beam: &SnesPpuBeam, t: u64) -> (Frame, bool) {
        let (ppu, interlace_next) = beam.beam_frame();
        let stored = self
            .state
            .lock()
            .frame
            .map(|(start, field, interlace, lines)| Frame {
                region: ppu.region,
                start,
                field,
                interlace,
                lines,
            });
        let walk = |mut f: Frame, step: &dyn Fn(Frame) -> Option<Frame>| -> Option<Frame> {
            for _ in 0..FRAMES_APART {
                if f.contains(t) {
                    return Some(f);
                }
                f = step(f)?;
            }
            None
        };
        let frame = if ppu.contains(t) {
            Some(ppu)
        } else if t >= ppu.end() {
            walk(ppu, &|f| Some(f.next(interlace_next)))
        } else {
            // The PPU is in a later frame than `t`. Forwards from the frame
            // this device knows, stepping onto the PPU's own where they meet;
            // failing that, backwards from the PPU's. A frame in between is
            // taken to have the interlace setting of the PPU's.
            stored
                .filter(|f| f.start <= t)
                .and_then(|f| {
                    walk(f, &|f| {
                        Some(if f.end() == ppu.start {
                            ppu
                        } else {
                            f.next(ppu.interlace)
                        })
                    })
                })
                .or_else(|| walk(ppu, &|f| f.prev(ppu.interlace)))
        };
        // Further apart than any scheduler lets them get. There is no honest
        // answer; a frame that starts here is at least a deterministic one.
        let frame = frame.unwrap_or(Frame { start: t, ..ppu });
        self.state.lock().frame = Some((frame.start, frame.field, frame.interlace, frame.lines));
        (frame, interlace_next)
    }

    /// Recompute every beam event, as when a register that places one is
    /// written: each is the first occurrence strictly after this device's
    /// tick.
    fn reschedule_all(&self) {
        self.reschedule(None);
    }

    /// Recompute the beam events after `fired` has run at this device's tick.
    ///
    /// Each is the first occurrence strictly after the tick — except one that
    /// is due *at* it and has not fired yet, which keeps its place and fires
    /// next. (Strictly after, for the one that just fired, is what makes it
    /// fire once.)
    fn reschedule(&self, fired: Option<u8>) {
        let Some(beam) = self.beam() else {
            return;
        };
        let now = self.tick.load(Ordering::Relaxed);
        let (frame, interlace_next) = self.frame_at(&beam, now);
        // The first time the beam is at dot `h` of line `v` after `now`.
        let at = |v: u16, h: u16| -> Option<u64> {
            let mut f = frame;
            for _ in 0..FRAMES_AHEAD {
                if let Some(tick) = f.tick_of(v, h).filter(|&tick| tick > now) {
                    return Some(tick);
                }
                f = f.next(interlace_next);
            }
            None
        };
        // The first time it is at dot `h` of any line: this line's, or the
        // next's.
        let on_a_line = |h: u16| -> Option<u64> {
            let v = frame.line_at(now)?;
            frame
                .tick_of(v, h)
                .filter(|&tick| tick > now)
                .or_else(|| frame.tick_of(v + 1, h))
                .or_else(|| frame.next(interlace_next).tick_of(0, h))
        };
        let mut s = self.state.lock();
        // A target the beam never reaches never fires, and the registers hold
        // one at power-on ($1FF in each). "Setting a value larger than the
        // maximum H counter value of 339 will prevent the timer's H condition
        // from being met", and likewise for V, whose maximum "depends on the
        // region (261 for NTSC, 311 for PAL) and interlacing" (SNESdev wiki,
        // *MMIO registers*, HTIME and VTIME). A `Frame` already answers `None`
        // for a line or a dot it does not have, so the targets go to it as
        // written rather than folded into range first.
        let irq = match (s.nmitimen >> 4) & 3 {
            0 => None,
            1 if s.htime > 339 => None,
            1 => on_a_line(s.htime),
            2 => at(s.vtime, 0),
            _ => at(s.vtime, s.htime),
        };
        let pending = |kind: u8, next: u64| fired.is_some_and(|f| f != kind) && next == now;
        if !pending(EV_INIT, s.next_init) {
            s.next_init = at(0, HDMA_INIT_DOT).unwrap_or(u64::MAX);
        }
        if !pending(EV_LINE, s.next_line) {
            s.next_line = on_a_line(HDMA_DOT).unwrap_or(u64::MAX);
        }
        if !pending(EV_REFRESH, s.next_refresh) {
            s.next_refresh = on_a_line(REFRESH_DOT).unwrap_or(u64::MAX);
        }
        if !pending(EV_IRQ, s.next_irq.unwrap_or(u64::MAX)) {
            s.next_irq = irq;
        }
        self.publish_next(&s);
    }

    /// Run every event up to `target`.
    fn advance_to(&self, target: u64) {
        loop {
            let (due, kind) = {
                let s = self.state.lock();
                let mut best = (s.next_init, EV_INIT);
                for (t, k) in [(s.next_line, EV_LINE), (s.next_refresh, EV_REFRESH)] {
                    if t < best.0 {
                        best = (t, k);
                    }
                }
                if let Some(t) = s.next_irq
                    && t < best.0
                {
                    best = (t, EV_IRQ);
                }
                best
            };
            if due > target {
                break;
            }
            self.tick.store(due, Ordering::Relaxed);
            match kind {
                EV_INIT => self.hdma_init(),
                EV_LINE => self.hdma_line(),
                EV_REFRESH => {
                    self.owed.fetch_add(REFRESH_CLOCKS, Ordering::Relaxed);
                }
                _ => {
                    self.state.lock().timeup = true;
                    self.drive_lines();
                }
            }
            // Every event is a beam position; recomputing them all from the
            // frame is cheap, cannot drift, and picks up a `SETINI` change.
            self.reschedule(Some(kind));
        }
        self.tick.store(target, Ordering::Relaxed);
        let s = self.state.lock();
        self.publish_next(&s);
    }

    fn hdma_init(&self) {
        let Some(space) = self.space() else {
            return;
        };
        let (enabled, mut chans, mut mdr) = {
            let s = self.state.lock();
            (s.hdmaen, s.ch, s.mdr)
        };
        let mut cost = 0;
        if enabled != 0 {
            cost += HDMA_OVERHEAD;
        }
        for (i, c) in chans.iter_mut().enumerate() {
            c.hdma_live = false;
            if enabled & (1 << i) == 0 {
                continue;
            }
            cost += HDMA_CHANNEL;
            c.a2a = c.a1t;
            let bank = u32::from(c.a1b) << 16;
            c.nltr = self.read_table(&space, bank | u32::from(c.a2a), mdr);
            mdr = c.nltr;
            c.a2a = c.a2a.wrapping_add(1);
            if c.indirect() {
                cost += HDMA_INDIRECT;
                let lo = self.read_table(&space, bank | u32::from(c.a2a), mdr);
                let hi = self.read_table(&space, bank | u32::from(c.a2a.wrapping_add(1)), lo);
                mdr = hi;
                c.das = u16::from_le_bytes([lo, hi]);
                c.a2a = c.a2a.wrapping_add(2);
            }
            c.hdma_live = c.nltr != 0;
            c.do_transfer = true;
        }
        {
            let mut s = self.state.lock();
            s.ch = chans;
            s.mdr = mdr;
        }
        self.owed.fetch_add(cost, Ordering::Relaxed);
    }

    fn hdma_line(&self) {
        let Some(beam) = self.beam() else {
            return;
        };
        // The line this event is on — this device's tick, not wherever the
        // PPU has got to, which may be the line after.
        let now = self.tick.load(Ordering::Relaxed);
        let Some(v) = self.frame_at(&beam, now).0.line_at(now) else {
            return;
        };
        // Lines 225 to 239 are picture or blank by `SETINI`'s overscan bit,
        // which the PPU samples at line 225 and has therefore sampled by now.
        let blank = v >= VBLANK_LINE_OVERSCAN || (v >= VBLANK_LINE && v >= beam.vblank_line());
        if blank {
            return;
        }
        let Some(space) = self.space() else {
            return;
        };
        let (enabled, mut chans, mut mdr) = {
            let s = self.state.lock();
            (s.hdmaen, s.ch, s.mdr)
        };
        let live = |c: &Channel, i: usize| enabled & (1 << i) != 0 && c.hdma_live;
        if !chans.iter().enumerate().any(|(i, c)| live(c, i)) {
            return;
        }
        let mut cost = HDMA_OVERHEAD;
        for (i, c) in chans.iter_mut().enumerate() {
            if !live(c, i) {
                continue;
            }
            cost += HDMA_CHANNEL;
            if c.do_transfer {
                for &off in c.pattern() {
                    let a = if c.indirect() {
                        let a = (u32::from(c.dasb) << 16) | u32::from(c.das);
                        c.das = c.das.wrapping_add(1);
                        a
                    } else {
                        let a = (u32::from(c.a1b) << 16) | u32::from(c.a2a);
                        c.a2a = c.a2a.wrapping_add(1);
                        a
                    };
                    let m = Move {
                        a,
                        b: c.bbad.wrapping_add(off),
                        b_to_a: c.b_to_a(),
                    };
                    mdr = self.transfer(&space, m, mdr);
                    cost += DMA_BYTE;
                }
            }
            c.nltr = c.nltr.wrapping_sub(1);
            c.do_transfer = c.nltr & 0x80 != 0;
            if c.nltr & 0x7F == 0 {
                let bank = u32::from(c.a1b) << 16;
                c.nltr = self.read_table(&space, bank | u32::from(c.a2a), mdr);
                mdr = c.nltr;
                c.a2a = c.a2a.wrapping_add(1);
                if c.indirect() {
                    cost += HDMA_INDIRECT;
                    let lo = self.read_table(&space, bank | u32::from(c.a2a), mdr);
                    let hi = self.read_table(&space, bank | u32::from(c.a2a.wrapping_add(1)), lo);
                    mdr = hi;
                    c.das = u16::from_le_bytes([lo, hi]);
                    c.a2a = c.a2a.wrapping_add(2);
                }
                c.hdma_live = c.nltr != 0;
                c.do_transfer = true;
            }
        }
        {
            let mut s = self.state.lock();
            s.ch = chans;
            s.mdr = mdr;
        }
        self.owed.fetch_add(cost, Ordering::Relaxed);
    }

    /// The next GP DMA byte, or `None` when every channel is done. Updates
    /// the channel's counters as it goes, and the owed clocks.
    fn next_dma_move(&self) -> Option<(Move, u8)> {
        let mut s = self.state.lock();
        if s.mdmaen == 0 {
            self.dma_active.store(false, Ordering::Relaxed);
            return None;
        }
        let i = s.mdmaen.trailing_zeros() as usize;
        if !s.dma_channel_started {
            s.dma_channel_started = true;
            s.dma_byte = 0;
            self.owed.fetch_add(DMA_CHANNEL, Ordering::Relaxed);
        }
        let c = s.ch[i];
        let pattern = c.pattern();
        let m = Move {
            a: (u32::from(c.a1b) << 16) | u32::from(c.a1t),
            b: c.bbad
                .wrapping_add(pattern[s.dma_byte as usize % pattern.len()]),
            b_to_a: c.b_to_a(),
        };
        // DMAPx bits 4–3: 00 increment, 10 decrement, x1 fixed.
        let step: u16 = match (c.dmap >> 3) & 3 {
            0 => 1,
            2 => 0xFFFF,
            _ => 0,
        };
        let ch = &mut s.ch[i];
        ch.a1t = ch.a1t.wrapping_add(step);
        // DAS counts down to zero; a count of zero is 65536 bytes.
        ch.das = ch.das.wrapping_sub(1);
        let done = ch.das == 0;
        s.dma_byte += 1;
        if done {
            s.mdmaen &= !(1 << i);
            s.dma_channel_started = false;
        }
        self.owed.fetch_add(DMA_BYTE, Ordering::Relaxed);
        Some((m, s.mdr))
    }
}

/// The gate and the timing provider: the same object, since they meet in
/// `owed`.
#[derive(Debug)]
struct Port {
    shared: Arc<Shared>,
}

impl CycleGate for Port {
    fn arbitrate(&self, _cycle: u64, _held: u64, bus: u8, _write: bool) -> Arbitration {
        let s = &self.shared;
        if s.owed.load(Ordering::Relaxed) != 0 {
            return Arbitration::Steal(bus);
        }
        if !s.dma_active.load(Ordering::Relaxed) {
            return Arbitration::Release;
        }
        let Some((m, mdr)) = s.next_dma_move() else {
            return Arbitration::Release;
        };
        let Some(space) = s.space() else {
            return Arbitration::Steal(bus);
        };
        let byte = s.transfer(&space, m, mdr);
        s.state.lock().mdr = byte;
        Arbitration::Steal(byte)
    }
}

impl AccessTiming for Port {
    #[inline]
    fn clocks(&self, cycle: BusCycle) -> u64 {
        if cycle.pins.has(Pins::HELD) {
            let owed = self.shared.owed.swap(0, Ordering::Relaxed);
            return if owed == 0 { IDLE_CLOCKS } else { owed };
        }
        if cycle.pins.is_internal() {
            return 6;
        }
        speed(cycle.addr, self.shared.memsel.load(Ordering::Relaxed))
    }
}

/// The master clocks an access to `addr` takes (see the module table).
#[must_use]
pub const fn speed(addr: u32, fast_rom: bool) -> u64 {
    let bank = (addr >> 16) as u8;
    let lo = addr & 0xFFFF;
    if bank & 0x40 == 0 {
        if lo < 0x2000 {
            8
        } else if lo < 0x4000 {
            6
        } else if lo < 0x4200 {
            12
        } else if lo < 0x6000 {
            6
        } else if lo < 0x8000 || bank & 0x80 == 0 || !fast_rom {
            8
        } else {
            6
        }
    } else if bank >= 0xC0 && fast_rom {
        6
    } else {
        8
    }
}

// ---------------------------------------------------------------------------
// The register windows
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Window {
    Regs,
    Dma,
    Joy,
}

#[derive(Debug)]
struct Regs {
    shared: Arc<Shared>,
    window: Window,
}

impl Regs {
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.shared.lazy.lock().clone();
        if let Some(h) = handle {
            let kind = if attrs.debug {
                AccessKind::Debug
            } else {
                AccessKind::Guest
            };
            let _ = h.sync(kind);
        }
    }

    fn read_regs(&self, offset: u8, bus: u8, debug: bool) -> u8 {
        let beam = self.shared.beam();
        let (vblank, hblank, master) = match &beam {
            Some(b) => (b.vblank(), b.hblank(), b.peek_master()),
            None => (false, false, 0),
        };
        let mut s = self.shared.state.lock();
        match offset {
            0x10 => {
                let v = (u8::from(s.rdnmi) << 7) | (bus & 0x70) | CPU_VERSION;
                if !debug {
                    s.rdnmi = false;
                }
                v
            }
            0x11 => {
                let v = (u8::from(s.timeup) << 7) | (bus & 0x7F);
                if !debug {
                    s.timeup = false;
                }
                v
            }
            0x12 => {
                (u8::from(vblank) << 7)
                    | (u8::from(hblank) << 6)
                    | (bus & 0x3E)
                    | u8::from(master < s.joy_busy_until)
            }
            0x13 => s.wrio,
            0x14 => s.rddiv as u8,
            0x15 => (s.rddiv >> 8) as u8,
            0x16 => s.rdmpy as u8,
            0x17 => (s.rdmpy >> 8) as u8,
            0x18 => s.joy[0] as u8,
            0x19 => (s.joy[0] >> 8) as u8,
            0x1A => s.joy[1] as u8,
            0x1B => (s.joy[1] >> 8) as u8,
            0x1C..=0x1F => 0,
            _ => bus,
        }
    }

    fn write_regs(&self, offset: u8, v: u8) {
        let mut extlatch = None;
        let mut reschedule = false;
        {
            let mut s = self.shared.state.lock();
            match offset {
                0x00 => {
                    s.nmitimen = v;
                    if v & 0x30 == 0 {
                        s.timeup = false;
                    }
                    reschedule = true;
                }
                0x01 => {
                    extlatch = Some(v & 0x80 != 0);
                    s.wrio = v;
                }
                0x02 => s.wrmpya = v,
                0x03 => {
                    s.rdmpy = u16::from(s.wrmpya) * u16::from(v);
                    // "For some reason, the hardware does additionally set
                    // RDDIVL=WRMPYB, and RDDIVH=00h" (fullsnes, "SNES Maths
                    // Multiply/Divide").
                    s.rddiv = u16::from(v);
                }
                0x04 => s.wrdiv = (s.wrdiv & 0xFF00) | u16::from(v),
                0x05 => s.wrdiv = (s.wrdiv & 0x00FF) | (u16::from(v) << 8),
                0x06 => {
                    if v == 0 {
                        s.rddiv = 0xFFFF;
                        s.rdmpy = s.wrdiv;
                    } else {
                        s.rddiv = s.wrdiv / u16::from(v);
                        s.rdmpy = s.wrdiv % u16::from(v);
                    }
                }
                0x07 => {
                    s.htime = (s.htime & 0x100) | u16::from(v);
                    reschedule = true;
                }
                0x08 => {
                    s.htime = (s.htime & 0xFF) | (u16::from(v & 1) << 8);
                    reschedule = true;
                }
                0x09 => {
                    s.vtime = (s.vtime & 0x100) | u16::from(v);
                    reschedule = true;
                }
                0x0A => {
                    s.vtime = (s.vtime & 0xFF) | (u16::from(v & 1) << 8);
                    reschedule = true;
                }
                0x0B => {
                    if v != 0 {
                        s.mdmaen = v;
                        s.dma_channel_started = false;
                        self.shared.owed.fetch_add(DMA_START, Ordering::Relaxed);
                        self.shared.dma_active.store(true, Ordering::Relaxed);
                    }
                }
                0x0C => s.hdmaen = v,
                0x0D => self.shared.memsel.store(v & 1 != 0, Ordering::Relaxed),
                _ => {}
            }
        }
        if let (Some(level), Some(beam)) = (extlatch, self.shared.beam()) {
            beam.set_extlatch(level);
        }
        if reschedule {
            self.shared.reschedule_all();
        }
        self.shared.drive_lines();
    }

    fn read_dma(&self, offset: u8, bus: u8) -> u8 {
        let s = self.shared.state.lock();
        let c = &s.ch[usize::from(offset >> 4)];
        match offset & 0xF {
            0x0 => c.dmap,
            0x1 => c.bbad,
            0x2 => c.a1t as u8,
            0x3 => (c.a1t >> 8) as u8,
            0x4 => c.a1b,
            0x5 => c.das as u8,
            0x6 => (c.das >> 8) as u8,
            0x7 => c.dasb,
            0x8 => c.a2a as u8,
            0x9 => (c.a2a >> 8) as u8,
            0xA => c.nltr,
            0xB | 0xF => c.unused,
            _ => bus,
        }
    }

    fn write_dma(&self, offset: u8, v: u8) {
        let mut s = self.shared.state.lock();
        let c = &mut s.ch[usize::from(offset >> 4)];
        match offset & 0xF {
            0x0 => c.dmap = v,
            0x1 => c.bbad = v,
            0x2 => c.a1t = (c.a1t & 0xFF00) | u16::from(v),
            0x3 => c.a1t = (c.a1t & 0x00FF) | (u16::from(v) << 8),
            0x4 => c.a1b = v,
            0x5 => c.das = (c.das & 0xFF00) | u16::from(v),
            0x6 => c.das = (c.das & 0x00FF) | (u16::from(v) << 8),
            0x7 => c.dasb = v,
            0x8 => c.a2a = (c.a2a & 0xFF00) | u16::from(v),
            0x9 => c.a2a = (c.a2a & 0x00FF) | (u16::from(v) << 8),
            0xA => c.nltr = v,
            0xB | 0xF => c.unused = v,
            _ => {}
        }
    }

    fn latch_pads(&self, s: &mut State) {
        s.shift = [self.shared.pad.get(0), self.shared.pad.get(1)];
        s.shifted = [0; 2];
    }

    fn read_joy(&self, offset: u8, bus: u8, debug: bool) -> u8 {
        let mut s = self.shared.state.lock();
        if s.strobe {
            self.latch_pads(&mut s);
        }
        let port = usize::from(offset & 1);
        let bit = if s.shifted[port] >= 16 {
            1
        } else {
            u8::from(s.shift[port] & 0x8000 != 0)
        };
        if !debug && !s.strobe && s.shifted[port] < 16 {
            s.shift[port] <<= 1;
            s.shifted[port] += 1;
        }
        if port == 0 {
            (bus & 0xFC) | bit
        } else {
            (bus & 0xE0) | 0x1C | bit
        }
    }

    fn write_joy(&self, offset: u8, v: u8) {
        if offset != 0 {
            return;
        }
        let mut s = self.shared.state.lock();
        let strobe = v & 1 != 0;
        if s.strobe && !strobe {
            self.latch_pads(&mut s);
        }
        s.strobe = strobe;
        if strobe {
            self.latch_pads(&mut s);
        }
    }
}

impl MemOps for Regs {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.sync(attrs);
        let o = offset as u8;
        *byte = match self.window {
            Window::Regs => self.read_regs(o, attrs.bus, attrs.debug),
            Window::Dma => self.read_dma(o, attrs.bus),
            Window::Joy => self.read_joy(o, attrs.bus, attrs.debug),
        };
        if !attrs.debug && self.window == Window::Regs {
            self.shared.drive_lines();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [v] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let o = offset as u8;
        match self.window {
            Window::Regs => self.write_regs(o, *v),
            Window::Dma => self.write_dma(o, *v),
            Window::Joy => self.write_joy(o, *v),
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `vblank` input.
#[derive(Debug)]
struct VblankPin {
    shared: Weak<Shared>,
    inputs: FanIn,
}

impl WireSink for VblankPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        let Some(shared) = self.shared.upgrade() else {
            return;
        };
        let master = shared.beam().map_or(0, |b| b.peek_master());
        {
            let mut s = shared.state.lock();
            if high && !s.vblank {
                s.rdnmi = true;
                if s.nmitimen & 1 != 0 {
                    s.joy = [shared.pad.get(0), shared.pad.get(1)];
                    s.shifted = [16; 2];
                    s.joy_busy_until = master + AUTO_JOY_CLOCKS;
                }
            } else if !high && s.vblank {
                s.rdnmi = false;
            }
            s.vblank = high;
        }
        shared.drive_lines();
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The `snes.scpu` device.
pub struct SCpu {
    shared: Arc<Shared>,
    port: Arc<Port>,
    /// The `ppu` link, resolved to the beam handle at bind time.
    ppu_link: Option<String>,
    /// The input pins built for wires. A net holds its sinks weakly, so the
    /// strong reference that keeps a pin alive is this one.
    pins: Mutex<alloc::vec::Vec<Arc<VblankPin>>>,
    regs: RegionRef,
    dma: RegionRef,
    joy: RegionRef,
}

impl fmt::Debug for SCpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SCpu")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl SCpu {
    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property nothing here accepts is given, or the pad port cannot
    /// be opened.
    pub fn new(props: &Props) -> Result<SCpu> {
        let mut r = props.reader();
        let pads_name = String::from(r.or_str("pads", DEFAULT_PADS)?);
        let ppu_link = r.optional_link("ppu")?.map(|l| String::from(l.as_str()));
        r.finish()?;
        let pad = pads::attach(props, &pads_name)?;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            pad,
            memsel: AtomicBool::new(false),
            owed: AtomicU64::new(0),
            dma_active: AtomicBool::new(false),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            requester: AtomicU32::new(0),
            space: Mutex::new(None),
            beam: Mutex::new(None),
            nmi: Mutex::with_rank(LockRank::WIRE, None),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::new(None),
        });
        let window = |name: &'static str, len: u64, w: Window| -> RegionRef {
            Arc::new(MmioRegion::io(
                name,
                len,
                Arc::new(Regs {
                    shared: Arc::clone(&shared),
                    window: w,
                }) as Arc<dyn MemOps>,
            ))
        };
        let regs = window("snes.scpu.regs", 0x20, Window::Regs);
        let dma = window("snes.scpu.dma", 0x80, Window::Dma);
        let joy = window("snes.scpu.joy", 2, Window::Joy);
        let port = Arc::new(Port {
            shared: Arc::clone(&shared),
        });
        Ok(SCpu {
            shared,
            port,
            ppu_link,
            pins: Mutex::new(alloc::vec::Vec::new()),
            regs,
            dma,
            joy,
        })
    }

    /// The pad port this device reads.
    #[must_use]
    pub fn pad(&self) -> Arc<Pad> {
        Arc::clone(&self.shared.pad)
    }

    /// Master clocks the core still owes to DMA, HDMA and refresh.
    #[must_use]
    pub fn owed(&self) -> u64 {
        self.shared.owed.load(Ordering::Relaxed)
    }
}

/// The `snes.scpu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "SNES 5A22 peripherals: NMI/IRQ timers, DMA/HDMA, multiply/divide, joypads, bus speed",
    properties: &[
        PropertySpec {
            name: "pads",
            kind: ValueKind::Str,
            required: false,
            summary: "the pad port to read the two controllers from (default `snes-pads`)",
        },
        PropertySpec {
            name: "ppu",
            kind: ValueKind::Link,
            required: true,
            summary: "the S-PPU whose beam this device times its events by",
        },
    ],
    construct: |props| Ok(Box::new(SCpu::new(props)?)),
};

/// Carry a v1 `snes.scpu` chunk forward.
///
/// v2 appends the frame the device's own tick is in. A v1 device kept none —
/// it measured every event from the PPU's position — so the step writes "not
/// known", and the first event after the load takes the PPU's, which is what
/// v1 did every time.
///
/// # Errors
///
/// If the step is already registered.
pub fn migrations(migrations: &mut crate::core::state::Migrations) -> Result<()> {
    migrations.register(CLASS_NAME, 1, |r, out| {
        let body = r.take(r.remaining())?;
        out.extend_from_slice(body);
        out.write_bool(false)?;
        out.write_u64(0)?;
        out.write_bool(false)?;
        out.write_bool(false)?;
        out.write_u16(0)
    })
}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(SCpu::new(props)?)))
}

/// What the validator should know about `snes.scpu`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("pads", ValueKind::Str))
        .prop(PropSchema::new("ppu", ValueKind::Link).required())
        .region(REGS_REGION)
        .region(DMA_REGION)
        .region(JOY_REGION)
        .port(NMI_PIN, PortDir::Out)
        .port(IRQ_PIN, PortDir::Out)
        .port(VBLANK_PIN, PortDir::In)
}

impl Device for SCpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            REGS_REGION => Some(Arc::clone(&self.regs)),
            DMA_REGION => Some(Arc::clone(&self.dma)),
            JOY_REGION => Some(Arc::clone(&self.joy)),
            _ => None,
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        if which == ExportId::CYCLE_GATE {
            return Some(Export::Gate(Arc::clone(&self.port) as Arc<dyn CycleGate>));
        }
        if which == ExportId::ACCESS_TIMING {
            let timing = Arc::clone(&self.port) as Arc<dyn AccessTiming>;
            return Some(Export::Opaque(Arc::new(TimingPort(timing))));
        }
        None
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let slot = match port {
            NMI_PIN => &self.shared.nmi,
            IRQ_PIN => &self.shared.irq,
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!("the S-CPU drives `{NMI_PIN}` and `{IRQ_PIN}`"),
                });
            }
        };
        *slot.lock() = Some(source);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != VBLANK_PIN {
            return None;
        }
        let pin = Arc::new(VblankPin {
            shared: Arc::downgrade(&self.shared),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut s = self.shared.state.lock();
            let (vblank, nmi_level, irq_level) = (s.vblank, s.nmi_level, s.irq_level);
            *s = State::reset();
            s.vblank = vblank;
            // What the two output wires were last driven to is not register
            // state, and a reset does not make it false: `drive_lines` below
            // compares against it, and has to see that a line which was high
            // must now be driven low. NMITIMEN is $00 after a reset, so
            // neither can stay asserted.
            s.nmi_level = nmi_level;
            s.irq_level = irq_level;
        }
        self.shared.memsel.store(false, Ordering::Relaxed);
        self.shared.owed.store(0, Ordering::Relaxed);
        self.shared.dma_active.store(false, Ordering::Relaxed);
        self.shared.reschedule_all();
        self.shared.drive_lines();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.shared.state.lock();
        for b in [s.nmitimen, s.wrio, s.wrmpya, s.mdmaen, s.hdmaen, s.mdr] {
            w.write_u8(b)?;
        }
        for v in [
            s.wrdiv, s.htime, s.vtime, s.rddiv, s.rdmpy, s.joy[0], s.joy[1], s.shift[0], s.shift[1],
        ] {
            w.write_u16(v)?;
        }
        for b in [s.rdnmi, s.timeup, s.strobe, s.dma_channel_started, s.vblank] {
            w.write_bool(b)?;
        }
        w.write_u8(s.shifted[0])?;
        w.write_u8(s.shifted[1])?;
        w.write_u32(s.dma_byte)?;
        w.write_u64(s.joy_busy_until)?;
        for c in &s.ch {
            for b in [c.dmap, c.bbad, c.a1b, c.dasb, c.nltr, c.unused] {
                w.write_u8(b)?;
            }
            for v in [c.a1t, c.das, c.a2a] {
                w.write_u16(v)?;
            }
            w.write_bool(c.do_transfer)?;
            w.write_bool(c.hdma_live)?;
        }
        w.write_bool(self.shared.memsel.load(Ordering::Relaxed))?;
        w.write_u64(self.shared.owed.load(Ordering::Relaxed))?;
        w.write_bool(self.shared.dma_active.load(Ordering::Relaxed))?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        // The scheduled beam events. Not derivable at load: the PPU they
        // are computed from may not have been restored yet, and a resumed
        // machine must fire them on exactly the ticks the original would.
        for t in [s.next_init, s.next_line, s.next_refresh] {
            w.write_u64(t)?;
        }
        w.write_u64(s.next_irq.unwrap_or(u64::MAX))?;
        w.write_bool(s.nmi_level)?;
        w.write_bool(s.irq_level)?;
        // The frame this device's tick is in. Not derivable at load either:
        // the PPU may be in the next one.
        let (start, field, interlace, lines) = s.frame.unwrap_or((0, false, false, 0));
        w.write_bool(s.frame.is_some())?;
        w.write_u64(start)?;
        w.write_bool(field)?;
        w.write_bool(interlace)?;
        w.write_u16(lines)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::reset();
        s.nmitimen = r.read_u8()?;
        s.wrio = r.read_u8()?;
        s.wrmpya = r.read_u8()?;
        s.mdmaen = r.read_u8()?;
        s.hdmaen = r.read_u8()?;
        s.mdr = r.read_u8()?;
        s.wrdiv = r.read_u16()?;
        s.htime = r.read_u16()? & 0x1FF;
        s.vtime = r.read_u16()? & 0x1FF;
        s.rddiv = r.read_u16()?;
        s.rdmpy = r.read_u16()?;
        s.joy = [r.read_u16()?, r.read_u16()?];
        s.shift = [r.read_u16()?, r.read_u16()?];
        s.rdnmi = r.read_bool()?;
        s.timeup = r.read_bool()?;
        s.strobe = r.read_bool()?;
        s.dma_channel_started = r.read_bool()?;
        s.vblank = r.read_bool()?;
        s.shifted = [r.read_u8()?.min(16), r.read_u8()?.min(16)];
        s.dma_byte = r.read_u32()?;
        s.joy_busy_until = r.read_u64()?;
        for c in &mut s.ch {
            c.dmap = r.read_u8()?;
            c.bbad = r.read_u8()?;
            c.a1b = r.read_u8()?;
            c.dasb = r.read_u8()?;
            c.nltr = r.read_u8()?;
            c.unused = r.read_u8()?;
            c.a1t = r.read_u16()?;
            c.das = r.read_u16()?;
            c.a2a = r.read_u16()?;
            c.do_transfer = r.read_bool()?;
            c.hdma_live = r.read_bool()?;
        }
        let memsel = r.read_bool()?;
        let owed = r.read_u64()?;
        let dma_active = r.read_bool()?;
        let tick = r.read_u64()?;
        s.next_init = r.read_u64()?;
        s.next_line = r.read_u64()?;
        s.next_refresh = r.read_u64()?;
        s.next_irq = Some(r.read_u64()?).filter(|&t| t != u64::MAX);
        s.nmi_level = r.read_bool()?;
        s.irq_level = r.read_bool()?;
        let known = r.read_bool()?;
        let frame = (r.read_u64()?, r.read_bool()?, r.read_bool()?, r.read_u16()?);
        // A frame with no lines has no tick in it, and the arithmetic that
        // finds a line assumes there is one.
        s.frame = (known && frame.3 > 0 && frame.3 <= 313).then_some(frame);
        self.shared.memsel.store(memsel, Ordering::Relaxed);
        self.shared.owed.store(owed, Ordering::Relaxed);
        self.shared.dma_active.store(dma_active, Ordering::Relaxed);
        self.shared.tick.store(tick, Ordering::Relaxed);
        self.shared.publish_next(&s);
        *self.shared.state.lock() = s;
        Ok(())
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let next = self.shared.next_event.load(Ordering::Relaxed);
        (next != u64::MAX).then_some(next)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for SCpu {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from("the S-CPU's DMA needs the A-bus space (`space = abus`)"),
        })?;
        *self.shared.space.lock() = Some(Arc::downgrade(space));
        self.shared
            .requester
            .store(ctx.requester().0, Ordering::Relaxed);
        if let Some(name) = &self.ppu_link {
            let beam = ctx.export_as::<SnesPpuBeam>(name, ExportId::SNES_PPU_BEAM)?;
            *self.shared.beam.lock() = Some(beam);
            self.shared.reschedule_all();
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "scpu_tests.rs"]
mod tests;
