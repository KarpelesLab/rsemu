//! `mac.mouse`: the Macintosh's one-button mouse, as two quadrature pulse
//! trains and a switch to ground.
//!
//! ```text
//!   osc    mouseclk = 1M Hz
//!   object mouse "mac.mouse" { clock = mouseclk }
//!
//!   wire mouse.x1     -> scc.dcda { pull = "down" }  # channel A: X1
//!   wire mouse.y1     -> scc.dcdb { pull = "down" }  # channel B: Y1
//!   wire mouse.x2     -> via.pb4  { pull = "down" }  # X2
//!   wire mouse.y2     -> via.pb5  { pull = "down" }  # Y2
//!   wire mouse.button -> via.pb3  { pull = "up" }
//! ```
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition — the VIA
//! port-assignment tables: "PB3 mouse switch (0 = button down)", "PB4 mouse X2"
//! and "PB5 mouse Y2"; and the SCC's two carrier detects carry X1 and Y1,
//! which is what the *Guide* leaves them for and what this tree's `mac.scc`
//! already names them. **Which axis is on which channel is a measurement**,
//! not a reading: a carrier-detect transition on channel A moves the low word
//! of `MTemp` at `$828` and one on channel B moves the high word, and a
//! QuickDraw `Point` is `{vertical, horizontal}` — so channel A is the
//! *horizontal* axis and channel B the vertical. `tests/mac_plus.rs` asserts
//! it against a real ROM.
//!
//! No emulator source was consulted and the ROM was not disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).
//!
//! # Two wires an axis, and only one of them is counted
//!
//! Each wheel produces two pulse trains ninety degrees apart. The Macintosh
//! takes an interrupt on **every transition of X1** — it is a carrier detect,
//! and the SCC's external/status condition is a change either way — and reads
//! X2 on the VIA to decide which way the wheel turned. So a full quadrature
//! cycle, four states, carries **two** counts rather than four, and
//! [`STEPS_PER_COUNT`] is 2: one mouse count is two phase transitions, of which
//! the second moves X1.
//!
//! That is why this device works in phase steps internally and in *counts* at
//! its edges. A count is what the ROM adds to `MTemp`, and therefore what a
//! host has to reason about when it decides how far a pointer should go.
//!
//! # Transitions, not a position
//!
//! A host reports motion as a delta and this device turns it into transitions
//! on four wires, one at a time, exactly as `amiga.mouse` does and for the
//! first of the three reasons that file gives: the guest counts edges, so a
//! delta of three hundred delivered at once is three hundred edges the guest
//! has to be given time to take an interrupt for. Spread at a rate a hand can
//! reach, the same motion arrives as a run of small, correct counts. Motion a
//! host delivers faster than [`DEFAULT_STEP_TICKS`] allows is not lost: it
//! waits, and arrives late.
//!
//! The default rate is set by what the *ROM* does with the counts rather than
//! by what a hand can do: Apple's mouse scaling doubles a delta of six or more
//! counts in one 60.15 Hz tick, so [`DEFAULT_STEP_TICKS`] keeps it under that
//! and one count is then one pixel. It is the *distance* that is capped rather
//! than each axis's own rate, because that is what the ROM measures — see
//! `State::interval`. Both constants carry their measurements.
//!
//! # The button waits its turn
//!
//! A button change is applied **after the motion posted before it** has been
//! clocked out, so a click lands where the pointer was sent rather than part
//! way there. `amiga.mouse` argues the same.
//!
//! # Determinism
//!
//! Through the record/replay seam: the mouse is a named host object,
//! [`Mouse`], opened by the device in `new(props)`; [`mice::channel`] and
//! [`mice::sink`] are its channel and sink. A payload is [`mice::RECORD_BYTES`]
//! bytes per report — signed 16-bit X and Y deltas, little-endian, and a button
//! byte with bit 0 the one button, the RFB order the rest of `host::input`
//! speaks.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "mac.mouse";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The host mouse port a mouse opens when its `mouse` property is not given.
pub const DEFAULT_MOUSE_PORT: &str = "mouse";

/// Ticks between quadrature transitions by default: 2 400, which at a 1 MHz
/// clock is one every 2.4 ms and so one **count** every 4.8 ms — 208 counts a
/// second on one axis, and half that on each of two ([`State::interval`]).
///
/// The number is set by what Apple's ROM does with the counts rather than by
/// what a hand can do, because the ROM applies **its own mouse scaling** and
/// a guest that accelerates cannot be pointed at anything by a host whose
/// cursor is absolute. Measured, by posting a burst and reading `MTemp`:
///
/// ```text
///   counts in one 60.15 Hz tick    1   3   5    6    7    8   16   32
///   MTemp moves by                 1   3   5   12   14   16   32   64
/// ```
///
/// — six or more in one tick is doubled and five or fewer is passed through.
/// Sweeping the rate instead of the burst size gives the same boundary from
/// the other side: 100 counts at 294 a second (`step-ticks` 1 700, 4.9 counts
/// a tick) come through as 118, and at 208 a second they come through as 100.
///
/// So 2 400 is the fastest round number with margin, and it buys an exact
/// [`PIXELS_PER_COUNT`](crate::host::input::mac::PIXELS_PER_COUNT) of one at
/// the cost of a pointer that crosses the 512-pixel screen in about two and a
/// half seconds. `step-ticks` turns it down for somebody who would rather have
/// the speed and live with the acceleration, which is after all what a real
/// Macintosh does to a real mouse.
///
/// **It is not exact to the count.** 400 counts on one axis move `Mouse` by
/// 398, at every rate from 2 400 to 6 000 ticks a step and not at all at
/// 12 000: the guest counts *interrupts*, and an edge that arrives while the
/// processor is inside the level-2 handler with the VIA also waiting is an
/// edge nothing counts. One in two hundred, it does not cancel, and a sweep
/// into a screen edge is what puts the two ends back together — which is what
/// a person does without thinking about it and what `tests/mac_plus.rs` does
/// deliberately.
pub const DEFAULT_STEP_TICKS: u64 = 2_400;

/// Quadrature transitions in one mouse count. See *Two wires an axis*.
pub const STEPS_PER_COUNT: i32 = 2;

/// How many reports wait before the newest are merged.
pub const MAX_QUEUED: usize = 64;

/// The largest backlog of phase steps an axis keeps, either way. Past it a host
/// is throwing the pointer about faster than the guest could follow in any
/// case.
pub const MAX_BACKLOG: i32 = 32_767;

/// The output pins, in the order the device holds their wires.
pub const OUTPUT_PINS: [&str; 5] = ["x1", "x2", "y1", "y2", "button"];

/// The one button, as a bit of a report: RFB's left.
pub const BUTTON: u8 = 0x01;

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

/// A quadrature phase as the pair's levels, `(pin1, pin2)`, both active high.
///
/// The Gray code of the phase, so exactly one of the two changes per step and
/// counting up walks `00 → 01 → 11 → 10`. Which direction of rotation the ROM
/// reads as "right" and "down" is settled in [`Mouse::report`]'s sign, against
/// a real ROM (`tests/mac_plus.rs`); the encoder itself has no preference.
#[inline]
#[must_use]
pub const fn phase_pins(phase: u8) -> (bool, bool) {
    let gray = phase ^ (phase >> 1);
    (gray & 0b01 != 0, gray & 0b10 != 0)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Report {
    /// Phase steps, not counts: converted on the way in.
    dx: i32,
    dy: i32,
    buttons: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    ticks: u64,
    next: u64,
    step_ticks: u64,
    /// Phase steps still to clock out on each axis for the report at the head.
    dx: i32,
    dy: i32,
    /// Quadrature phase of each wheel, 0-3.
    phase_x: u8,
    phase_y: u8,
    /// Buttons down, as the pins show them.
    buttons: u8,
    /// Reports not yet started. The head's motion is in `dx`/`dy` once taken.
    queue: VecDeque<Report>,
    /// The buttons the report whose motion is in progress will apply.
    pending_buttons: Option<u8>,
}

impl State {
    fn new(step_ticks: u64) -> State {
        State {
            ticks: 0,
            next: NO_EVENT,
            step_ticks,
            dx: 0,
            dy: 0,
            phase_x: 0,
            phase_y: 0,
            buttons: 0,
            queue: VecDeque::new(),
            pending_buttons: None,
        }
    }

    /// What each output stage drives, in [`OUTPUT_PINS`] order. The four
    /// quadrature pins are levels; `button` is `true` when the switch is
    /// closed, which pulls its line low.
    fn pins(&self) -> [bool; 5] {
        let (x1, x2) = phase_pins(self.phase_x);
        let (y1, y2) = phase_pins(self.phase_y);
        [x1, x2, y1, y2, self.buttons & BUTTON != 0]
    }

    fn post(&mut self, report: Report) {
        if self.queue.len() >= MAX_QUEUED
            && let Some(last) = self.queue.back_mut()
        {
            last.dx = (last.dx + report.dx).clamp(-MAX_BACKLOG, MAX_BACKLOG);
            last.dy = (last.dy + report.dy).clamp(-MAX_BACKLOG, MAX_BACKLOG);
            last.buttons = report.buttons;
        } else {
            self.queue.push_back(Report {
                dx: report.dx.clamp(-MAX_BACKLOG, MAX_BACKLOG),
                dy: report.dy.clamp(-MAX_BACKLOG, MAX_BACKLOG),
                buttons: report.buttons,
            });
        }
        if self.next == NO_EVENT {
            self.take();
        }
    }

    /// Settle whatever needs no time: finish the report in progress if its
    /// motion is done, and start the next. Leaves `next` on the tick of the
    /// next transition, or [`NO_EVENT`].
    fn take(&mut self) {
        loop {
            if self.dx != 0 || self.dy != 0 {
                if self.next == NO_EVENT {
                    self.next = self.ticks + self.interval();
                }
                return;
            }
            if let Some(b) = self.pending_buttons.take() {
                self.buttons = b;
            }
            let Some(report) = self.queue.pop_front() else {
                self.next = NO_EVENT;
                return;
            };
            self.dx = report.dx;
            self.dy = report.dy;
            self.pending_buttons = Some(report.buttons);
        }
    }

    /// Ticks to the next transition: [`step_ticks`](Self::step_ticks) for each
    /// axis still moving.
    ///
    /// **Doubling the interval when both axes move is what caps the *distance*
    /// rate rather than each axis's own**, and that is what the ROM measures.
    /// Its scaling threshold is on the two axes' movement *together*: 100
    /// counts on one axis at 166 a second come through exactly, and 100 on each
    /// at the same rate come through as 180 (measured). Stepping both axes at
    /// half rate keeps the sum where a single axis put it, so a diagonal sweep
    /// is exact for the same reason a straight one is — and is the honest shape
    /// besides, since a hand moving a mouse diagonally at some speed is not
    /// moving either wheel as fast as it would on its own.
    fn interval(&self) -> u64 {
        let axes = u64::from(self.dx != 0) + u64::from(self.dy != 0);
        self.step_ticks * axes.max(1)
    }

    /// One transition on every axis with steps left.
    fn step(&mut self) {
        if self.dx != 0 {
            let up = self.dx > 0;
            self.phase_x = if up {
                self.phase_x + 1
            } else {
                self.phase_x + 3
            } & 3;
            self.dx -= if up { 1 } else { -1 };
        }
        if self.dy != 0 {
            let up = self.dy > 0;
            self.phase_y = if up {
                self.phase_y + 1
            } else {
                self.phase_y + 3
            } & 3;
            self.dy -= if up { 1 } else { -1 };
        }
        self.next = NO_EVENT;
        if self.dx != 0 || self.dy != 0 {
            self.next = self.ticks + self.interval();
        } else {
            self.take();
        }
    }
}

#[derive(Debug, Default, Clone)]
struct Outputs {
    pins: [Option<WireSource>; 5],
}

/// A Macintosh mouse: its two wheels, its one button, and the door motion
/// comes in through.
pub struct Mouse {
    state: Mutex<State>,
    ticks: AtomicU64,
    next_event: AtomicU64,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Mouse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Mouse");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Default for Mouse {
    fn default() -> Mouse {
        Mouse::new()
    }
}

impl Mouse {
    /// A mouse lying still with its button up.
    #[must_use]
    pub fn new() -> Mouse {
        Mouse {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(DEFAULT_STEP_TICKS)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// Move by `(dx, dy)` **counts** — right and down positive, as the ROM
    /// accumulates them into `MTemp` — and then hold `held` ([`BUTTON`]).
    ///
    /// The device end of the record/replay channel.
    ///
    /// **The two axes turn opposite ways**, and that is a measurement rather
    /// than a choice: with the phase walking `00 → 01 → 11 → 10`, the ROM
    /// counts `MTemp`'s horizontal coordinate *down* and its vertical
    /// coordinate *up*. So X is negated on the way into the phase generator
    /// and Y is not. Nothing in any document gives the sense of either pair —
    /// it depends on which way round a wheel's encoder is mounted — and the
    /// guest is the only thing that can settle it. `tests/mac_plus.rs` puts
    /// the pointer somewhere and finds it there in the picture, which is the
    /// assertion that both signs are right.
    pub fn report(&self, dx: i32, dy: i32, held: u8) {
        self.sync();
        let steps = |counts: i32, sense: i32| -> i32 {
            counts
                .saturating_mul(sense * STEPS_PER_COUNT)
                .clamp(-MAX_BACKLOG, MAX_BACKLOG)
        };
        self.update(|st| {
            st.post(Report {
                dx: steps(dx, -1),
                dy: steps(dy, 1),
                buttons: held & BUTTON,
            });
        });
    }

    /// The buttons the pins show now.
    #[must_use]
    pub fn buttons(&self) -> u8 {
        self.state.lock().buttons
    }

    /// Counts not yet clocked out, `(x, y)`, the queue included.
    ///
    /// In counts, as [`report`](Self::report) takes them, and with the same
    /// sense — so a report of `(5, 0)` that has not started shows as `(5, 0)`.
    #[must_use]
    pub fn backlog(&self) -> (i64, i64) {
        let st = self.state.lock();
        let (mut x, mut y) = (i64::from(st.dx), i64::from(st.dy));
        for r in &st.queue {
            x += i64::from(r.dx);
            y += i64::from(r.dy);
        }
        let per = i64::from(STEPS_PER_COUNT);
        (-x / per, y / per)
    }

    /// Ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// Run the mouse until `target` ticks have passed in total, one transition
    /// at a time with the pins driven between them.
    pub fn advance_to(&self, target: u64) {
        loop {
            let moved = {
                let mut st = self.state.lock();
                if st.next != NO_EVENT && st.next <= target {
                    st.ticks = st.ticks.max(st.next);
                    let before = st.pins();
                    st.step();
                    self.publish(&st);
                    Some(before != st.pins())
                } else {
                    if target > st.ticks {
                        st.ticks = target;
                    }
                    self.publish(&st);
                    None
                }
            };
            match moved {
                Some(true) => self.refresh(),
                Some(false) => {}
                None => break,
            }
        }
    }

    fn publish(&self, st: &State) {
        self.ticks.store(st.ticks, Ordering::Relaxed);
        self.next_event.store(st.next, Ordering::Relaxed);
    }

    fn update(&self, f: impl FnOnce(&mut State)) {
        let moved = {
            let mut st = self.state.lock();
            let before = st.pins();
            f(&mut st);
            self.publish(&st);
            before != st.pins()
        };
        if moved {
            self.refresh();
        }
    }

    /// Drive every output stage, holding no lock.
    ///
    /// The four quadrature lines are **push-pull**: they come out of the
    /// mouse's own logic and one of them reaches a Z8530 input directly, so
    /// `drive` with a strong level, for the reason `mac.video` gives about its
    /// blanking pins — a fresh source already sitting at `Level::Low` makes
    /// `set(Low)` a no-op that never reaches the far end, and the realize
    /// sweep then announces nothing. Their nets need a `pull` for the same
    /// reason: the sweep refreshes a *resolved* net, and without one the SCC
    /// and the VIA came up holding their own pins' pull-ups — which a snapshot
    /// round trip caught, because a built board and a restored one then
    /// disagreed about `dcd_latched`.
    ///
    /// The button is the other kind: a switch to ground, so it pulls low or
    /// lets the port's pull-up have the line.
    fn refresh(&self) {
        let pins = self.state.lock().pins();
        let out = self.out.lock().clone();
        for (i, (src, high)) in out.pins.iter().zip(pins).enumerate() {
            let Some(src) = src else { continue };
            if i == OUTPUT_PINS.len() - 1 {
                src.drive(if high { Drive::Low } else { Drive::HiZ });
            } else {
                src.drive(Drive::strong(Level::from(high)));
            }
        }
    }

    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
    }
}

/// The build's named Macintosh mice.
pub mod mice {
    use super::{BUTTON, Mouse};
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a mouse is filed under in a build's host objects.
    pub const KIND: HostKind = HostKind::door("mac-mouse", make_sink);

    /// The mouse `name` refers to in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of object holds that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Mouse>> {
        hosts.open(KIND, name, Mouse::new)
    }

    /// The device's side of [`open`].
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Mouse>> {
        props.host(KIND, name, Mouse::new)
    }

    /// The mouse called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Mouse>>> {
        hosts.get(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// Bytes per recorded report: X delta and Y delta as little-endian `i16`,
    /// then the button byte.
    pub const RECORD_BYTES: usize = 5;

    /// One report as a payload.
    #[must_use]
    pub const fn encode(dx: i16, dy: i16, buttons: u8) -> [u8; RECORD_BYTES] {
        let x = dx.to_le_bytes();
        let y = dy.to_le_bytes();
        [x[0], x[1], y[0], y[1], buttons]
    }

    /// The channel the mouse called `name` moves on: `mac-mouse:mouse`.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The sink that applies recorded reports to `mouse`, in order. A trailing
    /// partial report is discarded.
    ///
    /// No rewind hook: the queue is part of the snapshot.
    #[must_use]
    pub fn sink(mouse: &Arc<Mouse>) -> Arc<dyn InputSink> {
        let mouse = Arc::clone(mouse);
        Arc::new(FnSink::new("mac-mouse", move |payload: &[u8]| {
            let (reports, _partial) = payload.as_chunks::<RECORD_BYTES>();
            for [x0, x1, y0, y1, b] in reports {
                let dx = i16::from_le_bytes([*x0, *x1]);
                let dy = i16::from_le_bytes([*y0, *y1]);
                mouse.report(i32::from(dx), i32::from(dy), *b & BUTTON);
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Mouse>().ok()?))
    }
}

/// The `mac.mouse` device.
#[derive(Debug)]
pub struct MacMouse {
    mouse: Arc<Mouse>,
}

impl MacMouse {
    /// Validate `props` and open the mouse they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a bad or unknown property, [`Error::Config`] if
    /// the name is held by something that is not a mouse.
    pub fn new(props: &Props) -> Result<MacMouse> {
        let mut r = props.reader();
        let port = r.or_str("mouse", DEFAULT_MOUSE_PORT)?.to_string();
        let step = r.or_range::<u64>("step-ticks", DEFAULT_STEP_TICKS, 1..=1_000_000_000)?;
        r.finish()?;
        let mouse = mice::attach(props, &port)?;
        mouse.state.lock().step_ticks = step;
        Ok(MacMouse { mouse })
    }

    /// A device around a mouse the caller already holds.
    #[must_use]
    pub fn with(mouse: Arc<Mouse>) -> MacMouse {
        MacMouse { mouse }
    }

    /// The mouse.
    #[must_use]
    pub fn mouse(&self) -> &Arc<Mouse> {
        &self.mouse
    }
}

impl Device for MacMouse {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A mouse has no reset line. Its wheels stay where they are.
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.mouse.state.lock().clone();
        w.write_u64(st.ticks)?;
        w.write_u64(st.next)?;
        w.write_u32(st.dx as u32)?;
        w.write_u32(st.dy as u32)?;
        w.write_u8(st.phase_x)?;
        w.write_u8(st.phase_y)?;
        w.write_u8(st.buttons)?;
        w.write_u16(st.pending_buttons.map_or(0, |b| 0x100 | u16::from(b)))?;
        w.write_seq_len(st.queue.len() as u64)?;
        for r in &st.queue {
            w.write_u32(r.dx as u32)?;
            w.write_u32(r.dy as u32)?;
            w.write_u8(r.buttons)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let step = self.mouse.state.lock().step_ticks;
        let mut st = State::new(step);
        st.ticks = r.read_u64()?;
        st.next = r.read_u64()?;
        st.dx = (r.read_u32()? as i32).clamp(-MAX_BACKLOG, MAX_BACKLOG);
        st.dy = (r.read_u32()? as i32).clamp(-MAX_BACKLOG, MAX_BACKLOG);
        st.phase_x = r.read_u8()? & 3;
        st.phase_y = r.read_u8()? & 3;
        st.buttons = r.read_u8()? & BUTTON;
        let pending = r.read_u16()?;
        st.pending_buttons = (pending & 0x100 != 0).then_some(pending as u8 & BUTTON);
        let count = r.read_seq_len(9)?;
        if count > MAX_QUEUED as u64 {
            return Err(Error::State(format!(
                "{CLASS_NAME}: {count} queued reports, more than the {MAX_QUEUED} a mouse holds"
            )));
        }
        for _ in 0..count {
            st.queue.push_back(Report {
                dx: (r.read_u32()? as i32).clamp(-MAX_BACKLOG, MAX_BACKLOG),
                dy: (r.read_u32()? as i32).clamp(-MAX_BACKLOG, MAX_BACKLOG),
                buttons: r.read_u8()? & BUTTON,
            });
        }
        if st.next != NO_EVENT && st.next <= st.ticks {
            st.next = st.ticks + 1;
        }
        self.mouse.update(|now| *now = st);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(i) = OUTPUT_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "a Macintosh mouse drives `x1`, `x2`, `y1`, `y2` and `button`",
                ),
            });
        };
        self.mouse.out.lock().pins[i] = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.mouse.refresh();
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.mouse.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.mouse.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.mouse.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.mouse.lazy.lock() = Some(handle);
    }
}

impl Instance for MacMouse {}

/// The `mac.mouse` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Macintosh mouse: host motion as quadrature transitions for the SCC's carrier \
              detects and the VIA's port B, and one button on a VIA pin",
    properties: &[
        PropertySpec {
            name: "mouse",
            kind: ValueKind::Str,
            required: false,
            summary: "the host mouse port it is moved through (default `mouse`)",
        },
        PropertySpec {
            name: "step-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks of its clock between quadrature transitions (default 2 400)",
        },
    ],
    construct: |props| Ok(Box::new(MacMouse::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(MacMouse::new(props)?)))
}

/// What the validator should know about `mac.mouse`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("mouse", ValueKind::Str))
        .prop(PropSchema::new("step-ticks", ValueKind::Uint));
    for pin in OUTPUT_PINS {
        schema = schema.port(pin, PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests;
