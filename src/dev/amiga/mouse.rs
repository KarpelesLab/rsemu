//! `amiga.mouse`: the two-button mouse in game port 0, as quadrature pulse
//! trains and switches to ground.
//!
//! ```text
//!   osc periph = 1000000 Hz
//!   object mouse "amiga.mouse" { clock = periph }
//!
//!   wire mouse.y      -> denise.m0v  { pull = "up" }   # pin 1, FORW*
//!   wire mouse.yq     -> denise.m0vq { pull = "up" }   # pin 3, LEFT*
//!   wire mouse.x      -> denise.m0h  { pull = "up" }   # pin 2, BACK*
//!   wire mouse.xq     -> denise.m0hq { pull = "up" }   # pin 4, RIGH*
//!   wire mouse.left   -> cia_a.pa6   { pull = "up" }   # pin 6, fire
//!   wire mouse.right  -> paula.potly { pull = "up" }   # pin 9
//!   wire mouse.middle -> paula.potlx { pull = "up" }   # pin 5
//! ```
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//!
//! * **Chapter 8, "Reading Mouse/Trackball Controllers"** (pp. 229-232): "for
//!   each direction, a mechanical wheel inside the mouse will produce two pulse
//!   trains, one 90 degrees out of phase with the other"; "the counters
//!   increment when the mouse is moved to the right or down (toward you)";
//!   "about 200 count pulses per inch"; and "Mouse Buttons": the left button on
//!   `CIAAPRA` bit 6 for the first port, "button 2 (right button on Amiga
//!   mouse) is connected to pin 9 … button 3, when used, is connected to pin
//!   5".
//! * **Appendix A, `JOY0DAT`** (p. 281): the connector pin table — pin 1
//!   `FORW*`/`Y`, pin 3 `LEFT*`/`YQ`, pin 2 `BACK*`/`X`, pin 4 `RIGH*`/`XQ` —
//!   and "the joystick functions are all active low at the connector pins".
//! * **Appendix E, CIA port assignments**: "PA6..game port 0, pin 6 (fire
//!   button\*)".
//! * **Table 8-4** for pins 5 and 9 on `POTGO`'s `DATLX` and `DATLY`.
//!
//! No emulator source was consulted (`ROADMAP.md` §1).
//!
//! # Transitions, not counter writes
//!
//! A host reports motion as a delta. This device turns it into **quadrature
//! transitions on four wires**, one count at a time, and Denise counts them;
//! nothing writes `JOY0DAT`. Three reasons, each of them the manual's:
//!
//! 1. **The counters wrap, and software is told to rely on it.** "These counters
//!    will wrap around … you must read the counters at least once each vertical
//!    blanking period … the new value of a counter minus the previous value will
//!    represent the number of mouse counts since the last check." A host that
//!    reports 300 pixels in one event and a model that adds 300 to an 8-bit
//!    counter hands the guest a counter that moved 44 — *the other way* for a
//!    signed reading. Spread over time at a rate a real mouse can reach, the
//!    same motion reads as a run of small, correct deltas.
//! 2. **The low two bits are the pins.** "Bits 1 and 0 of each counter may be
//!    read to determine the state of these two clock pins", and `JOYTEST` writes
//!    "xx" there. A counter written directly has nowhere to keep that fact; a
//!    counter clocked by the pins has it for free.
//! 3. **A joystick is the same four pins.** Table 8-3 reads `JOY0DAT` bits
//!    1 xor 0 as "back". Denise decoding pin levels means a joystick model later
//!    is four switches and no change to Denise.
//!
//! The rate is [`DEFAULT_STEP_TICKS`] per count: 5 000 counts a second at the
//! board's 1 MHz clock, 25 inches a second at the manual's 200 counts an inch,
//! which is 100 counts in a PAL field and 83 in an NTSC one — under the 127 a
//! once-a-field reader can tell from a wrap. Both axes step together. Motion a
//! host delivers faster than that is not lost: it waits, and arrives late.
//!
//! # Buttons wait their turn
//!
//! A button change is applied **after the motion posted before it** has been
//! clocked out, so a click lands where the pointer was sent rather than part
//! way there. Reports queue in order ([`MAX_QUEUED`] of them; beyond that the
//! newest two are merged, keeping the later buttons and the sum of the motion).
//!
//! # Determinism
//!
//! Through the record/replay seam: the mouse is a named host object,
//! [`Mouse`], opened by the device in `new(props)`; [`mice::channel`] and
//! [`mice::sink`] are its channel and sink. A payload is [`mice::RECORD_BYTES`]
//! bytes per report — signed 16-bit X and Y deltas, little-endian, and a button
//! byte with bit 0 left, bit 1 middle, bit 2 right, the RFB order the rest of
//! `host::input` speaks.

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
use crate::core::wire::{Drive, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.mouse";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The host mouse port a mouse opens when its `mouse` property is not given.
pub const DEFAULT_MOUSE_PORT: &str = "mouse";

/// Ticks between quadrature transitions by default: 200, which at a 1 MHz
/// clock is 5 000 counts a second. See the module docs.
pub const DEFAULT_STEP_TICKS: u64 = 200;

/// How many reports wait before the newest are merged.
pub const MAX_QUEUED: usize = 64;

/// The largest backlog of counts an axis keeps, either way. Past it a host is
/// throwing the pointer about faster than the guest could follow in any case.
pub const MAX_BACKLOG: i32 = 32_767;

/// The output pins, in the order the device holds their wires: the connector's
/// `X`, `XQ`, `Y`, `YQ`, then pins 6, 9 and 5.
pub const OUTPUT_PINS: [&str; 7] = ["x", "xq", "y", "yq", "left", "right", "middle"];

/// Button bits in a report: RFB's order.
pub mod buttons {
    /// The left button, on CIA-A `PA6`.
    pub const LEFT: u8 = 0x01;
    /// The middle button, on pin 5.
    pub const MIDDLE: u8 = 0x02;
    /// The right button, on pin 9.
    pub const RIGHT: u8 = 0x04;
}

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

/// A quadrature phase as the pins' levels: `(pin, pin-Q)` pulled low.
///
/// Denise reads bit 1 of a counter as `!Q` and bit 0 as `pin xor Q` (Appendix
/// A: "LEFT and RIGHT … directly available on the Y1 and X1 bits",
/// "FORWARD … Y1 xor Y0"). So a counter's two low bits are the phase in binary
/// exactly when the active-high pair `(!Q, !pin)` is that binary number's Gray
/// code, and counting up — right, or toward you — walks
/// `00 → 01 → 11 → 10`.
#[inline]
#[must_use]
pub const fn phase_pins(phase: u8) -> (bool, bool) {
    // Gray code of the phase: bit 1 = !Q active, bit 0 = !pin active.
    let gray = phase ^ (phase >> 1);
    let q_low = gray & 0b10 != 0;
    let pin_low = gray & 0b01 != 0;
    (pin_low, q_low)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Report {
    dx: i32,
    dy: i32,
    buttons: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    ticks: u64,
    next: u64,
    step_ticks: u64,
    /// Counts still to clock out on each axis for the report at the head.
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

    /// What each output stage does, in [`OUTPUT_PINS`] order: `true` pulls low.
    fn pins(&self) -> [bool; 7] {
        let (x, xq) = phase_pins(self.phase_x);
        let (y, yq) = phase_pins(self.phase_y);
        [
            x,
            xq,
            y,
            yq,
            self.buttons & buttons::LEFT != 0,
            self.buttons & buttons::RIGHT != 0,
            self.buttons & buttons::MIDDLE != 0,
        ]
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
                    self.next = self.ticks + self.step_ticks;
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

    /// One transition on every axis with counts left.
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
            self.next = self.ticks + self.step_ticks;
        } else {
            self.take();
        }
    }
}

#[derive(Debug, Default, Clone)]
struct Outputs {
    pins: [Option<WireSource>; 7],
}

/// An Amiga mouse: its wheels, its buttons, and the door motion comes in
/// through.
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
    /// A mouse lying still with no button down.
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

    /// Move by `(dx, dy)` counts — right and down positive, as the counters
    /// count — and then hold `buttons` ([`buttons`] bits).
    ///
    /// The device end of the record/replay channel.
    pub fn report(&self, dx: i32, dy: i32, held: u8) {
        self.sync();
        self.update(|st| {
            st.post(Report {
                dx,
                dy,
                buttons: held & (buttons::LEFT | buttons::MIDDLE | buttons::RIGHT),
            });
        });
    }

    /// The buttons the pins show now.
    #[must_use]
    pub fn buttons(&self) -> u8 {
        self.state.lock().buttons
    }

    /// Counts not yet clocked out, `(x, y)`, the queue included.
    #[must_use]
    pub fn backlog(&self) -> (i64, i64) {
        let st = self.state.lock();
        let (mut x, mut y) = (i64::from(st.dx), i64::from(st.dy));
        for r in &st.queue {
            x += i64::from(r.dx);
            y += i64::from(r.dy);
        }
        (x, y)
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

    /// Drive every output stage, holding no lock. Open-collector: a closed
    /// switch or a lit phase pulls low, and otherwise the port's pull-up has
    /// the line.
    fn refresh(&self) {
        let pins = self.state.lock().pins();
        let out = self.out.lock().clone();
        for (src, low) in out.pins.iter().zip(pins) {
            if let Some(src) = src {
                src.drive(if low { Drive::Low } else { Drive::HiZ });
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

/// The build's named Amiga mice.
pub mod mice {
    use super::Mouse;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a mouse is filed under in a build's host objects.
    pub const KIND: HostKind = HostKind::door("amiga-mouse", make_sink);

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

    /// The channel the mouse called `name` moves on: `amiga-mouse:mouse`.
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
        Arc::new(FnSink::new("amiga-mouse", move |payload: &[u8]| {
            let (reports, _partial) = payload.as_chunks::<RECORD_BYTES>();
            for [x0, x1, y0, y1, b] in reports {
                let dx = i16::from_le_bytes([*x0, *x1]);
                let dy = i16::from_le_bytes([*y0, *y1]);
                mouse.report(i32::from(dx), i32::from(dy), *b);
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Mouse>().ok()?))
    }
}

/// The `amiga.mouse` device.
#[derive(Debug)]
pub struct AmigaMouse {
    mouse: Arc<Mouse>,
}

impl AmigaMouse {
    /// Validate `props` and open the mouse they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a bad or unknown property, [`Error::Config`] if
    /// the name is held by something that is not a mouse.
    pub fn new(props: &Props) -> Result<AmigaMouse> {
        let mut r = props.reader();
        let port = r.or_str("mouse", DEFAULT_MOUSE_PORT)?.to_string();
        let step = r.or_range::<u64>("step-ticks", DEFAULT_STEP_TICKS, 1..=1_000_000_000)?;
        r.finish()?;
        let mouse = mice::attach(props, &port)?;
        mouse.state.lock().step_ticks = step;
        Ok(AmigaMouse { mouse })
    }

    /// A device around a mouse the caller already holds.
    #[must_use]
    pub fn with(mouse: Arc<Mouse>) -> AmigaMouse {
        AmigaMouse { mouse }
    }

    /// The mouse.
    #[must_use]
    pub fn mouse(&self) -> &Arc<Mouse> {
        &self.mouse
    }
}

impl Device for AmigaMouse {
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
        st.buttons = r.read_u8()? & 7;
        let pending = r.read_u16()?;
        st.pending_buttons = (pending & 0x100 != 0).then_some(pending as u8 & 7);
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
                buttons: r.read_u8()? & 7,
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
                    "an Amiga mouse drives `x`, `xq`, `y`, `yq`, `left`, `right` and `middle`",
                ),
            });
        };
        self.mouse.out.lock().pins[i] = Some(source);
        self.mouse.refresh();
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

impl Instance for AmigaMouse {}

/// The `amiga.mouse` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Amiga mouse: host motion as quadrature transitions for Denise's counters, \
              the left button on a CIA pin and the others on the pot pins",
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
            summary: "ticks of its clock between quadrature transitions (default 200)",
        },
    ],
    construct: |props| Ok(Box::new(AmigaMouse::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(AmigaMouse::new(props)?)))
}

/// What the validator should know about `amiga.mouse`.
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
