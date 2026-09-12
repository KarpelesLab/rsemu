//! `keypad.matrix`: R row lines and C column lines with a switch at each cross.
//!
//! The part every embedded board has and no register block describes, because
//! there is no register block: a keypad matrix is copper and switches, and the
//! firmware does the scanning. R row conductors, C column conductors, and at
//! each intersection a momentary contact that — **while it is held** — shorts
//! that row conductor to that column conductor. Nothing else. Every behaviour
//! below, ghosting included, is that one sentence plus the net's resistors.
//!
//! ```text
//!            col0     col1     col2        ← pulled up, read by the firmware
//!              │        │        │
//!   row0 ──────o────────o────────o──
//!              │        │        │
//!   row1 ──────o────────o────────X──        X = held: row1 shorted to col2
//!              │        │        │
//!   row2 ──────o────────o────────o──
//!              ↑
//!        driven low, one row at a time
//! ```
//!
//! # Why this needs [`Drive`] and not [`Level`]
//!
//! A switch has no output stage. It does not drive high, it does not drive low:
//! it *connects two conductors*, and what the net then sits at is decided by
//! whatever else is on it — the MCU's push-pull row driver, the board's
//! pull-up, the `PUPDR` bits inside the pad. So this device only ever presents
//! [`Drive::Low`] or [`Drive::HiZ`], never [`Drive::High`], and it expects its
//! nets to carry a [`Pull`](crate::core::wire::Pull): write
//! `wire keypad.col0 -> gpioc.in0 { pull = "up" }` and the whole net is
//! resolved rather than per-sink. A keypad on a per-sink net still runs, but
//! "nobody is driving" is not a state such a net can represent, so an idle
//! column reads low and every scan finds every key held.
//!
//! # Which way it conducts
//!
//! **Both.** Rows and columns are the same kind of conductor and the switch
//! between them has no direction, so the model is symmetric: a firmware that
//! drives one row low and reads the columns and a firmware that drives one
//! column low and reads the rows both work, from the same device and the same
//! wiring. The exception is `diodes = true`, where each key has a series diode
//! in it and current flows row → column only; then the columns are driven and
//! the rows never are, which is exactly what the diode is for.
//!
//! Every `row{n}` and `col{n}` pin is therefore [`PortDir::InOut`]. A machine
//! file that only ever scans one way may wire each pin one way and nothing
//! complains — a row that is only ever written is sensed and never driven, a
//! column that is only ever read is driven and never sensed. A conductor no
//! `wire` statement names at all is a track that goes nowhere: it pulls nothing
//! down and nothing pulls it down, which is why a matrix declared wider than
//! the board wires up does not fabricate shorts along its unused edge.
//!
//! [`PortDir::InOut`]: crate::machine::validate::PortDir::InOut
//!
//! # Ghosting, and the diode that cures it
//!
//! With plain switches the closed contacts form an **undirected graph** joining
//! row nodes to column nodes, and a conductor is pulled low when anything in
//! its connected component is pulled low. Hold (0,0), (0,1) and (1,0); scan
//! row 1. Row 1 is low, so col 0 is low through (1,0); col 0 reaches row 0
//! through (0,0); row 0 reaches col 1 through (0,1) — and the firmware reads a
//! key at (1,1) that nobody is touching. That is the classic *ghost*, it is the
//! reason a keyboard has a diode per key, and it falls out of the component
//! computation rather than being coded as a special case.
//!
//! `diodes = true` says each key is a switch **in series with a diode**, so a
//! column can never pull a row low and the path above is broken at its second
//! hop. A column is then low exactly when some key in it has its own row low —
//! no components, no ghosts, and full N-key rollover.
//!
//! # Contact bounce
//!
//! `bounce = n` makes each make and each break chatter `n` extra times,
//! `bounce-time` apart, before settling on the state the host asked for. It is
//! a real defect of a real switch and firmware is supposed to debounce it, so
//! it is worth being able to reproduce.
//!
//! **The scheduler owns that time** (`CLAUDE.md`). A bouncing keypad is a
//! *lazily advanced* device (`ROADMAP.md` §4.2): it holds its own tick,
//! publishes the tick its next transition falls on through
//! [`Device::next_event_tick`], and is walked to it. It never sleeps, never
//! reads a clock and never spawns anything. A keypad with `bounce = 0` — the
//! default — is not lazy at all and needs no clock domain; a keypad with
//! `bounce` set *does*, and the realizer refuses a board that asks for one
//! without a `clock`, naming the instance. `bounce-time` is counted in **ticks
//! of that clock domain**, not in nanoseconds, for the reason `atmel.at24c`
//! counts `write-ticks` that way: the time path has no floats and a device does
//! not own a frequency.
//!
//! # Determinism
//!
//! Which keys a person is holding is a non-deterministic input crossing into
//! the machine, and `ROADMAP.md` §0 says every one of those goes through the
//! record/replay seam or it is a determinism bug. It does: the matrix lives in
//! a **named host object**, [`Keys`], which this device opens by name from
//! `new(props)` exactly as the Game Boy's joypad opens its pad. [`keys::channel`]
//! is the channel a recorder registers, [`keys::sink`] is what a recorded
//! payload does, and a board whose keypad the recorder does not know about is
//! refused by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) at
//! build time.
//!
//! # Sources
//!
//! No datasheet describes a keypad matrix because a keypad matrix is not a
//! part: it is a wiring pattern, and everything here is derived from what a
//! switch between two conductors does. The electrical model it is expressed in
//! — strong drivers beating weak ones, open-drain stages, a net's own resistor
//! — is [`core::wire`](crate::core::wire)'s, which cites ST RM0090 §8.3.10 for
//! the pad side of it. No emulator source was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU32, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, Level, WireId, WireSink, WireSource};
use crate::machine::validate::port_index;

/// The class name a machine description writes.
const CLASS_NAME: &str = "keypad.matrix";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The most rows a matrix may have.
///
/// Sixteen is one GPIO port, which is what a matrix this size is wired to. The
/// limit exists so that a key index fits in a byte — see
/// [`keys::RECORD_BYTES`] — and so a badly written machine file cannot ask for
/// a four-billion-node union-find.
pub const MAX_ROWS: u64 = 16;

/// The most columns a matrix may have. As [`MAX_ROWS`].
pub const MAX_COLS: u64 = 16;

/// The most keys a matrix may have: [`MAX_ROWS`] × [`MAX_COLS`].
pub const MAX_KEYS: usize = (MAX_ROWS * MAX_COLS) as usize;

/// The prefix of the row pins: `row0` … `row{rows-1}`.
pub const ROW_PORT: &str = "row";

/// The prefix of the column pins: `col0` … `col{cols-1}`.
pub const COL_PORT: &str = "col";

/// The host keypad port a machine gets when its description names none.
pub const DEFAULT_KEYPAD_PORT: &str = "keypad";

/// How many recompute passes one outermost change runs before giving up.
///
/// Reached only by a board that has wired the matrix into a combinational loop,
/// which is a machine-description error; an ordinary scan settles in one pass,
/// and a pass that releases a line it was holding settles in two.
const SETTLE_LIMIT: u32 = 64;

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// One contact still chattering its way to a settled position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Bounce {
    /// Which key, as a row-major index.
    key: u32,
    /// How many transitions are still to come. The last one is not a toggle:
    /// it forces the contact to `target`, so a bounce always settles where the
    /// host asked it to whatever `bounce`'s parity is.
    remaining: u32,
    /// The tick of the device's own domain the next transition falls on.
    next: u64,
    /// Where this contact ends up.
    target: bool,
}

/// Everything one keypad knows, behind one lock.
///
/// The geometry is here too rather than in a second structure because a host
/// object is created empty — `Keys::new` takes no arguments, since a name is
/// the only thing that travels from a machine file into a constructor — and is
/// then [`configured`](Keys::configure) by the device that opened it.
struct State {
    /// How many row conductors. Zero until the device configures the port.
    rows: usize,
    /// How many column conductors.
    cols: usize,
    /// Whether each key carries a series diode.
    diodes: bool,
    /// How many extra transitions a make or a break produces.
    bounce: u32,
    /// The interval between them, in ticks of this device's clock domain.
    bounce_ticks: u64,
    /// The key names, row-major, or empty for the `"r,c"` default.
    names: Vec<String>,
    /// Which keys the host is holding down, row-major. Architectural: it is
    /// the level the record/replay seam carries.
    pressed: Vec<bool>,
    /// Which contacts are actually closed. Equal to `pressed` except while a
    /// contact is bouncing, which is exactly why the two are separate.
    contacts: Vec<bool>,
    /// The contacts still bouncing, ordered by `(next, key)` so that the order
    /// two simultaneous bounces fire in is a function of the matrix and not of
    /// the order a host happened to post them.
    bounces: Vec<Bounce>,
    /// Whether each conductor reaches anything at all: a `wire` statement
    /// named it, in either direction. A conductor nobody wired is a track that
    /// goes nowhere — nothing pulls it down and it pulls nothing down — which
    /// is not the same as one sitting at [`Level::Low`], and conflating the two
    /// would let a key on an unwired column ghost.
    wired: Vec<bool>,
    /// What the outside is holding each conductor at, indexed by
    /// [line](Keys::line_of). **Derived** (`ROADMAP.md` §4.5): it is another
    /// device's output, so it is not snapshotted — the wire sweep re-announces
    /// it after a load.
    ///
    /// [`Level::Low`] to begin with, because that is what a net with nothing
    /// driving it reads in this model (`core::wire`), and a sink whose default
    /// disagreed with the wire's would never be told: a driver announcing the
    /// level a fresh net already sits at delivers nothing.
    sensed: Vec<Level>,
    /// What this device is presenting on each conductor. Derived, as `sensed`
    /// is: it is a function of `contacts` and `sensed`.
    driving: Vec<Drive>,
    /// The output port of each conductor, once the realizer has handed it over.
    sources: Vec<Option<WireSource>>,
    /// The tick of this device's clock domain it has simulated up to. Only
    /// moves when `bounce` is non-zero, which is the only thing here that
    /// takes time.
    ticks: u64,
}

impl fmt::Debug for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("State")
            .field("rows", &self.rows)
            .field("cols", &self.cols)
            .field("diodes", &self.diodes)
            .field("pressed", &self.pressed)
            .field("contacts", &self.contacts)
            .field("bounces", &self.bounces)
            .field("wired", &self.wired)
            .field("sensed", &self.sensed)
            .field("driving", &self.driving)
            .field("ticks", &self.ticks)
            .finish_non_exhaustive()
    }
}

impl State {
    /// An unconfigured matrix: no rows, no columns, nothing held.
    fn empty() -> State {
        State {
            rows: 0,
            cols: 0,
            diodes: false,
            bounce: 0,
            bounce_ticks: 0,
            names: Vec::new(),
            pressed: Vec::new(),
            contacts: Vec::new(),
            bounces: Vec::new(),
            wired: Vec::new(),
            sensed: Vec::new(),
            driving: Vec::new(),
            sources: Vec::new(),
            ticks: 0,
        }
    }

    /// How many conductors there are: rows first, then columns.
    fn lines(&self) -> usize {
        self.rows + self.cols
    }

    /// How many keys there are.
    fn key_count(&self) -> usize {
        self.rows * self.cols
    }
}

/// The geometry a device configures its host object with.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Geometry {
    rows: usize,
    cols: usize,
    diodes: bool,
    bounce: u32,
    bounce_ticks: u64,
    names: Vec<String>,
}

// ---------------------------------------------------------------------------
// the host object
// ---------------------------------------------------------------------------

/// The matrix itself: what is held, which contacts are closed, and the pins.
///
/// This is the **host object** a build files under [`keys::KIND`] and the name
/// the machine description gave — the keypad rather than a copy of it. The
/// device holds it and a host that presses a key opens it by name. One object
/// rather than a device handle plus a mirror of its state, because two would
/// have to be kept in step and nothing would check that they were.
///
/// The wires live here too, and have to: a press is what drives them, and a
/// press comes from out here.
pub struct Keys {
    /// [`LockRank::DEVICE`]. Never held across a call onto a wire.
    state: Mutex<State>,
    /// Whether a recompute is already running, so a re-entrant one hands its
    /// work to it instead of recursing. The same trick `Wire::deliver` uses,
    /// and for the same reason.
    busy: AtomicBool,
    /// Whether something changed that the running recompute has not seen.
    dirty: AtomicBool,
    /// How many recomputes gave up at [`SETTLE_LIMIT`]. A diagnostic: non-zero
    /// means the board wired this matrix into a combinational loop.
    unsettled: AtomicU32,
}

impl fmt::Debug for Keys {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Keys")
            .field("state", &self.state)
            .field("unsettled", &self.unsettled.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Default for Keys {
    fn default() -> Keys {
        Keys::new()
    }
}

impl Keys {
    /// An unconfigured, empty keypad.
    ///
    /// What [`keys::open`] and [`keys::attach`] create on first mention; the
    /// device that opens it gives it its geometry.
    #[must_use]
    pub fn new() -> Keys {
        Keys {
            state: Mutex::with_rank(LockRank::DEVICE, State::empty()),
            busy: AtomicBool::new(false),
            dirty: AtomicBool::new(false),
            unsettled: AtomicU32::new(0),
        }
    }

    /// Give this keypad its shape, or agree that it already has it.
    ///
    /// Idempotent on purpose: two `keypad.matrix` objects naming one host port
    /// are one keypad seen twice, which is legal as long as they describe the
    /// same keypad. Two that disagree are a machine-description error, and this
    /// is where it surfaces rather than at the first press.
    fn configure(&self, geom: &Geometry) -> Result<()> {
        let mut s = self.state.lock();
        if s.rows != 0 || s.cols != 0 {
            let same = s.rows == geom.rows
                && s.cols == geom.cols
                && s.diodes == geom.diodes
                && s.bounce == geom.bounce
                && s.bounce_ticks == geom.bounce_ticks
                && s.names == geom.names;
            if same {
                return Ok(());
            }
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: format!(
                    "this host keypad port is already a {}x{} matrix; a second `{CLASS_NAME}` on \
                     the same `keys` name must describe the same keypad",
                    s.rows, s.cols
                ),
            });
        }
        s.rows = geom.rows;
        s.cols = geom.cols;
        s.diodes = geom.diodes;
        s.bounce = geom.bounce;
        s.bounce_ticks = geom.bounce_ticks;
        s.names = geom.names.clone();
        s.pressed = vec![false; geom.rows * geom.cols];
        s.contacts = vec![false; geom.rows * geom.cols];
        s.wired = vec![false; geom.rows + geom.cols];
        s.sensed = vec![Level::Low; geom.rows + geom.cols];
        s.driving = vec![Drive::HiZ; geom.rows + geom.cols];
        s.sources = vec![None; geom.rows + geom.cols];
        Ok(())
    }

    /// How many rows this matrix has. Zero until a device has configured it.
    #[must_use]
    pub fn rows(&self) -> usize {
        self.state.lock().rows
    }

    /// How many columns this matrix has.
    #[must_use]
    pub fn cols(&self) -> usize {
        self.state.lock().cols
    }

    /// How many keys this matrix has: rows × columns.
    #[must_use]
    pub fn key_count(&self) -> usize {
        self.state.lock().key_count()
    }

    /// The row-major index of the key at `(row, col)`, if there is one.
    #[must_use]
    pub fn index_at(&self, row: usize, col: usize) -> Option<usize> {
        let s = self.state.lock();
        (row < s.rows && col < s.cols).then(|| row * s.cols + col)
    }

    /// The index of the key called `name`, from the `layout` property.
    ///
    /// With no `layout` the names are `"r,c"` — `"1,2"` is the key on row 1,
    /// column 2 — so a host input map always has something to say.
    #[must_use]
    pub fn index_of(&self, name: &str) -> Option<usize> {
        let s = self.state.lock();
        if !s.names.is_empty() {
            return s.names.iter().position(|n| n == name);
        }
        let (row, col) = name.split_once(',')?;
        let row: usize = row.trim().parse().ok()?;
        let col: usize = col.trim().parse().ok()?;
        (row < s.rows && col < s.cols).then(|| row * s.cols + col)
    }

    /// What the key at `index` is called.
    #[must_use]
    pub fn name_of(&self, index: usize) -> Option<String> {
        let s = self.state.lock();
        if index >= s.key_count() {
            return None;
        }
        match s.names.get(index) {
            Some(name) => Some(name.clone()),
            None => Some(format!("{},{}", index / s.cols, index % s.cols)),
        }
    }

    /// Whether the host is holding the key at `index`.
    #[must_use]
    pub fn pressed(&self, index: usize) -> bool {
        self.state
            .lock()
            .pressed
            .get(index)
            .copied()
            .unwrap_or(false)
    }

    /// Press or release one key.
    ///
    /// The device end of the record/replay channel — see the [module
    /// docs](self). An index past the end of the matrix is ignored rather than
    /// refused: a recording made against a bigger keypad must not abort a
    /// replay half way through.
    ///
    /// The state lock is released before the conductors are driven, because
    /// driving them is an outward call (`CLAUDE.md`, re-entrancy).
    pub fn set(&self, index: usize, down: bool) {
        let moved = {
            let mut s = self.state.lock();
            if index >= s.key_count() || s.pressed[index] == down {
                false
            } else {
                s.pressed[index] = down;
                // The contact takes the new position at once and then chatters
                // around it, which is what a switch does: the first make is
                // real, and the bounces follow it.
                s.contacts[index] = down;
                if s.bounce > 0 {
                    let (n, step, now) = (s.bounce, s.bounce_ticks, s.ticks);
                    s.bounces.retain(|b| b.key as usize != index);
                    s.bounces.push(Bounce {
                        key: index as u32,
                        remaining: n,
                        next: now.saturating_add(step),
                        target: down,
                    });
                    s.bounces.sort_by_key(|b| (b.next, b.key));
                }
                true
            }
        };
        if moved {
            self.touch();
        }
    }

    /// Press or release the key called `name`, reporting whether there is one.
    pub fn press(&self, name: &str, down: bool) -> bool {
        match self.index_of(name) {
            Some(index) => {
                self.set(index, down);
                true
            }
            None => false,
        }
    }

    /// Replace the whole held state from a packed bitmap, LSB of byte 0 first.
    ///
    /// Bits past the end of the matrix are ignored, and a short mask releases
    /// the keys it does not mention — it is the whole state, not a patch.
    pub fn set_mask(&self, mask: &[u8]) {
        let count = self.state.lock().key_count();
        for index in 0..count {
            let byte = mask.get(index / 8).copied().unwrap_or(0);
            self.set(index, byte & (1 << (index % 8)) != 0);
        }
    }

    /// The whole held state as a packed bitmap, LSB of byte 0 first.
    #[must_use]
    pub fn get(&self) -> Vec<u8> {
        pack(&self.state.lock().pressed)
    }

    /// Which contacts are actually closed right now, packed as [`Keys::get`].
    ///
    /// Differs from [`Keys::get`] only while a contact is bouncing.
    #[must_use]
    pub fn contacts(&self) -> Vec<u8> {
        pack(&self.state.lock().contacts)
    }

    /// How many recomputes gave up before the matrix stopped changing: a
    /// diagnostic, like [`Wire::unsettled`](crate::core::wire::Wire::unsettled)
    /// and for the same reason. Non-zero only for a board that has wired this
    /// matrix into a combinational loop.
    #[must_use]
    pub fn unsettled(&self) -> u32 {
        self.unsettled.load(Ordering::Relaxed)
    }

    // -- the conductors ------------------------------------------------------

    /// The line number of row `n`.
    fn row_line(n: usize) -> usize {
        n
    }

    /// The line number of column `n`, given how many rows there are.
    fn col_line(rows: usize, n: usize) -> usize {
        rows + n
    }

    /// The line a port name refers to, or `None` for a name that is not a pin.
    fn line_of(&self, port: &str) -> Option<usize> {
        let s = self.state.lock();
        if let Some(i) = port_index(port, ROW_PORT, s.rows as u32) {
            return Some(Keys::row_line(i as usize));
        }
        port_index(port, COL_PORT, s.cols as u32).map(|i| Keys::col_line(s.rows, i as usize))
    }

    /// Note that a `wire` statement reached this conductor.
    fn mark_wired(&self, line: usize) {
        let mut s = self.state.lock();
        if line < s.lines() {
            s.wired[line] = true;
        }
    }

    /// Take the output port of one conductor, and sense what the net is at.
    fn attach(&self, line: usize, source: WireSource) {
        {
            let mut s = self.state.lock();
            if line >= s.lines() {
                return;
            }
            // Nothing is driven yet, so the net's level *is* what the outside
            // is holding it at.
            s.sensed[line] = source.net_level();
            s.sources[line] = Some(source);
            s.wired[line] = true;
        }
        self.touch();
    }

    /// Told what a conductor is at.
    ///
    /// **A line this device is holding low is a line it cannot see.** That is
    /// true of the hardware — a closed contact shorts the two conductors, and
    /// an ammeter is the only way to tell which end is sinking the current —
    /// and it is what stops the model feeding its own output back into its
    /// input and latching a column low for ever. The remembered level therefore
    /// stands still for as long as this device drives, and is refreshed the
    /// moment it lets go.
    fn sense(&self, line: usize, level: Level) {
        let moved = {
            let mut s = self.state.lock();
            if line >= s.lines() {
                return;
            }
            // Reading the net here is not an outward *call*: `drive_state` and
            // `net_level` are atomic loads on the wire and call nothing back,
            // so the lock is not held across anything re-entrant.
            let outside = match &s.sources[line] {
                Some(source) if !source.drive_state().is_hiz() => return,
                Some(source) => source.net_level(),
                // A pin the machine file only ever wires *into* has no output
                // port, so there is nothing of ours on the net to discount.
                None => level,
            };
            if s.sensed[line] == outside {
                false
            } else {
                s.sensed[line] = outside;
                true
            }
        };
        if moved {
            self.touch();
        }
    }

    /// Re-drive every conductor from what is held — the realize sweep, a reset
    /// and a snapshot load.
    fn touch(&self) {
        self.dirty.store(true, Ordering::SeqCst);
        self.settle();
    }

    /// Recompute until nothing more changes.
    ///
    /// Iterative rather than recursive, exactly as `Wire::deliver` is: a
    /// recompute that re-enters this — a sibling device reacting to a column
    /// this one just pulled down, and driving a row back — records its work and
    /// returns, and the outermost pass picks it up.
    fn settle(&self) {
        if self.busy.swap(true, Ordering::SeqCst) {
            return;
        }
        loop {
            let mut passes: u32 = 0;
            while self.dirty.swap(false, Ordering::SeqCst) {
                self.recompute();
                passes += 1;
                if passes >= SETTLE_LIMIT {
                    self.unsettled.fetch_add(1, Ordering::SeqCst);
                    break;
                }
            }
            self.busy.store(false, Ordering::SeqCst);
            // A change made between the last scan and the release above would
            // have seen the flag set and returned, so re-check before leaving.
            if !self.dirty.load(Ordering::SeqCst) {
                return;
            }
            if self.busy.swap(true, Ordering::SeqCst) {
                return;
            }
        }
    }

    /// One pass: work out what each conductor should be presenting, then
    /// present it — outside the lock, because driving a wire is an outward
    /// call.
    fn recompute(&self) {
        let plan = {
            let mut s = self.state.lock();
            let want = desired_drives(&s);
            let mut plan: Vec<(Drive, WireSource)> = Vec::new();
            for (line, drive) in want.iter().copied().enumerate() {
                if s.driving[line] == drive {
                    continue;
                }
                s.driving[line] = drive;
                if let Some(source) = s.sources[line].clone() {
                    plan.push((drive, source));
                }
            }
            plan
        };
        for (drive, source) in &plan {
            source.drive(*drive);
        }
        // Letting go of a line that somebody else is *also* holding low does
        // not move the net, so the wire delivers nothing and `sense` never
        // runs. Ask the net directly for every line this device is no longer
        // driving; with nothing of ours on it, its level is the outside's.
        let mut s = self.state.lock();
        for line in 0..s.lines() {
            if !s.driving[line].is_hiz() {
                continue;
            }
            let Some(source) = s.sources[line].clone() else {
                continue;
            };
            let outside = source.net_level();
            if s.sensed[line] != outside {
                s.sensed[line] = outside;
                self.dirty.store(true, Ordering::SeqCst);
            }
        }
    }

    // -- time ----------------------------------------------------------------

    /// Walk the bouncing contacts up to `tick`.
    fn advance(&self, tick: u64) {
        loop {
            let fired = {
                let mut s = self.state.lock();
                match s.bounces.first().copied() {
                    Some(b) if b.next <= tick => {
                        s.ticks = b.next;
                        let key = b.key as usize;
                        if b.remaining <= 1 {
                            s.contacts[key] = b.target;
                            s.bounces.remove(0);
                        } else {
                            let step = s.bounce_ticks;
                            s.contacts[key] = !s.contacts[key];
                            let head = &mut s.bounces[0];
                            head.remaining -= 1;
                            head.next = head.next.saturating_add(step);
                            s.bounces.sort_by_key(|e| (e.next, e.key));
                        }
                        true
                    }
                    _ => false,
                }
            };
            if !fired {
                break;
            }
            self.touch();
        }
        let mut s = self.state.lock();
        if s.ticks < tick {
            s.ticks = tick;
        }
    }
}

/// What each conductor should be presenting, given what is closed and what the
/// outside is holding.
///
/// Deterministic by construction (`CLAUDE.md`): a fixed sweep over row-major
/// indices and a union-find over a fixed node numbering, with no map anywhere.
///
/// Two rules are shared by both halves:
///
/// * The device never presents [`Drive::High`]. It is switches.
/// * The device never drives a conductor it can already see held low. The net
///   ends up at the same level either way, and not driving keeps the line
///   observable — see [`Keys::sense`].
fn desired_drives(s: &State) -> Vec<Drive> {
    let (rows, cols) = (s.rows, s.cols);
    let mut want = vec![Drive::HiZ; rows + cols];

    // A conductor is "held low" only if something is actually attached to it.
    let held_low = |line: usize| s.wired[line] && s.sensed[line].is_low();

    if s.diodes {
        // A diode in series with each contact: current flows row → column, so
        // a low row pulls its column down and a low column pulls nothing.
        for c in 0..cols {
            let line = Keys::col_line(rows, c);
            if !s.wired[line] || held_low(line) {
                continue;
            }
            if (0..rows).any(|r| s.contacts[r * cols + c] && held_low(Keys::row_line(r))) {
                want[line] = Drive::Low;
            }
        }
        return want;
    }

    // Bare contacts: the closed keys join row nodes to column nodes into an
    // undirected graph, and a component containing anything held low is low
    // throughout. That *is* the ghost.
    let n = rows + cols;
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    for r in 0..rows {
        for c in 0..cols {
            if !s.contacts[r * cols + c] {
                continue;
            }
            let a = find(&mut parent, Keys::row_line(r));
            let b = find(&mut parent, Keys::col_line(rows, c));
            if a != b {
                parent[a] = b;
            }
        }
    }
    let mut low = vec![false; n];
    for line in 0..n {
        if held_low(line) {
            let root = find(&mut parent, line);
            low[root] = true;
        }
    }
    for (line, drive) in want.iter_mut().enumerate() {
        if !s.wired[line] || held_low(line) {
            continue;
        }
        if low[find(&mut parent, line)] {
            *drive = Drive::Low;
        }
    }
    want
}

/// A bool per key as a bitmap, LSB of byte 0 first.
fn pack(bits: &[bool]) -> Vec<u8> {
    let mut out = vec![0u8; bits.len().div_ceil(8)];
    for (index, bit) in bits.iter().enumerate() {
        if *bit {
            out[index / 8] |= 1 << (index % 8);
        }
    }
    out
}

/// The inverse of [`pack`], into a buffer of the length the caller wants.
fn unpack(bytes: &[u8], out: &mut [bool]) {
    for (index, slot) in out.iter_mut().enumerate() {
        let byte = bytes.get(index / 8).copied().unwrap_or(0);
        *slot = byte & (1 << (index % 8)) != 0;
    }
}

// ---------------------------------------------------------------------------
// the record/replay seam
// ---------------------------------------------------------------------------

/// The build's named keypad ports.
///
/// The same shape as [`chardev::ports`](crate::host::chardev::ports) and the
/// Game Boy's `gb::joypad::pads`: a *name* is the only thing that can travel
/// from a machine description into a device constructor, and both ends resolve
/// it against the build's own
/// [`HostObjects`](crate::core::hosts::HostObjects).
///
/// ```text
/// machine file:  object pad "keypad.matrix" { rows = 4, cols = 3, keys = "keypad" }
/// device:        keys::attach(props, "keypad")  ──┐
/// host:          keys::open(&hosts, "keypad")   ──┴─► the same Arc<Keys>
/// ```
pub mod keys {
    use super::Keys;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a keypad port is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::door("keypad", make_sink);

    /// The keypad port `name` refers to in `hosts`, creating it on first
    /// mention.
    ///
    /// The **host** side of the rendezvous: called before anybody presses
    /// anything, or after the build to pick up what the device opened.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Keys>> {
        hosts.open(KIND, name, Keys::new)
    }

    /// The keypad port `name` refers to in the build these properties belong
    /// to.
    ///
    /// The **device** side, called from `new(props)`: acquiring a host object
    /// is allocation, not an outward action
    /// ([`core::hosts`](crate::core::hosts) argues the case). A `Props` that
    /// belongs to no build gets a private keypad, so a device a unit test built
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Keys>> {
        props.host(KIND, name, Keys::new)
    }

    /// The keypad port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Keys>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// How many bytes one recorded key movement is: the key index, then `0`
    /// for released and anything else for held.
    ///
    /// Two bytes rather than a whole bitmap. A pad has eight buttons and a
    /// console samples them all off one strobe, so `nes::input::pads` records
    /// the *level* of the lot; a matrix may have 256 keys, and recording 32
    /// bytes to say one of them moved would make a recording a hundred times
    /// the size of the thing it records. Here a key is an independent switch,
    /// so a movement is the natural unit — and it stays a level rather than an
    /// edge, because the payload says where the key *is*, not that it changed.
    ///
    /// A longer payload is several movements, applied in order. A trailing odd
    /// byte is discarded: half a movement says nothing.
    pub const RECORD_BYTES: usize = 2;

    /// The record/replay channel the keypad called `name` is pressed through.
    ///
    /// `keypad:keypad`, which is the same `(kind, name)` pair the host-object
    /// table files the keypad under — so a board whose keypad has no channel is
    /// refused by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal)
    /// naming this string.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that applies a recorded payload to `keys`.
    ///
    /// [`RECORD_BYTES`] bytes per movement, as described there.
    ///
    /// No rewind hook: a key holds a level rather than a queue, and the level
    /// is part of the machine snapshot a rewind restores.
    #[must_use]
    pub fn sink(keys: &Arc<Keys>) -> Arc<dyn InputSink> {
        let keys = Arc::clone(keys);
        Arc::new(FnSink::new("keypad", move |payload: &[u8]| {
            let (movements, _odd) = payload.as_chunks::<RECORD_BYTES>();
            for [index, down] in movements {
                keys.set(usize::from(*index), *down != 0);
            }
        }))
    }

    /// [`sink`], reached through the erased handle the host-object table holds.
    ///
    /// What [`KIND`] carries so that
    /// [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) can wire
    /// this keypad port to a recorder without the caller having to name it.
    /// `None` means something that is not a [`Keys`] is filed under `keypad` —
    /// two modules claiming one kind name, which the seal reports rather than
    /// guesses at.
    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Keys>().ok()?))
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// Every row and column pin, as one sink.
///
/// One object for the whole matrix rather than one per conductor: a
/// [`SinkPin`] carries a line number, which is what tells the shared sink which
/// conductor a level arrived on.
struct MatrixPin {
    keys: Arc<Keys>,
}

impl fmt::Debug for MatrixPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MatrixPin").finish_non_exhaustive()
    }
}

impl WireSink for MatrixPin {
    fn set_level(&self, _src: WireId, line: u32, level: Level) {
        self.keys.sense(line as usize, level);
    }
}

/// A GPIO matrix keypad.
pub struct Keypad {
    keys: Arc<Keys>,
    /// Kept, because a net refers to its sinks weakly.
    pin: Arc<MatrixPin>,
    /// Whether contacts bounce, and so whether this device takes time at all.
    bouncing: bool,
}

impl fmt::Debug for Keypad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Keypad")
            .field("keys", &self.keys)
            .field("bouncing", &self.bouncing)
            .finish_non_exhaustive()
    }
}

impl Keypad {
    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a geometry outside [`MAX_ROWS`] × [`MAX_COLS`],
    /// a `layout` whose length does not match, a duplicate or empty key name,
    /// or `bounce` without a `bounce-time`; [`Error::Config`] if the host
    /// keypad port is already a different keypad.
    pub fn new(props: &Props) -> Result<Keypad> {
        let mut r = props.reader();
        let rows = r.require_range::<u64>("rows", 1..=MAX_ROWS)? as usize;
        let cols = r.require_range::<u64>("cols", 1..=MAX_COLS)? as usize;
        let diodes = r.or("diodes", false)?;
        let layout = r.or_str("layout", "")?.to_string();
        let bounce = r.or_range::<u64>("bounce", 0, 0..=64)? as u32;
        let bounce_ticks = r.or("bounce-time", 0u64)?;
        let port = r.or_str("keys", DEFAULT_KEYPAD_PORT)?.to_string();
        r.finish()?;

        if bounce > 0 && bounce_ticks == 0 {
            return Err(Error::Property(
                "`bounce` needs a `bounce-time` to space the transitions out: every transition \
                 at the same tick is one transition"
                    .to_string(),
            ));
        }
        let names = parse_layout(&layout, rows, cols)?;

        let keys = keys::attach(props, &port)?;
        keys.configure(&Geometry {
            rows,
            cols,
            diodes,
            bounce,
            bounce_ticks,
            names,
        })?;
        Ok(Keypad {
            pin: Arc::new(MatrixPin {
                keys: Arc::clone(&keys),
            }),
            keys,
            bouncing: bounce > 0,
        })
    }

    /// The keypad this device reads: the host end of the seam.
    #[must_use]
    pub fn keys(&self) -> &Arc<Keys> {
        &self.keys
    }
}

/// Split the `layout` property into one name per key.
///
/// Empty means the names are `"r,c"`, computed on demand. Anything else must
/// name every key exactly once: a short list would silently leave half the
/// matrix unnameable, and a repeated name would make [`Keys::index_of`] pick
/// one of two keys by position in a list nobody thinks of as ordered.
fn parse_layout(layout: &str, rows: usize, cols: usize) -> Result<Vec<String>> {
    if layout.is_empty() {
        return Ok(Vec::new());
    }
    let names: Vec<String> = layout.split(',').map(|n| n.trim().to_string()).collect();
    if names.len() != rows * cols {
        return Err(Error::Property(format!(
            "`layout` names {} key(s) but this is a {rows}x{cols} matrix, which has {}; the \
             names are row-major",
            names.len(),
            rows * cols
        )));
    }
    for (index, name) in names.iter().enumerate() {
        if name.is_empty() {
            return Err(Error::Property(format!(
                "`layout` entry {index} is empty; every key needs a name"
            )));
        }
        if names[..index].contains(name) {
            return Err(Error::Property(format!(
                "`layout` uses the name `{name}` twice; a name has to pick out one key"
            )));
        }
    }
    Ok(names)
}

impl Device for Keypad {
    fn class(&self) -> &'static DeviceClass {
        &KEYPAD_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing: the conductors do not exist yet. Two-phase construction puts
        // the outward action in `announce`, which the realize sweep calls once
        // every net has been built and every source handed over.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut s = self.keys.state.lock();
            // Which keys a person is holding is not something a reset changes
            // — the same argument `gb.joypad` makes — but a contact caught
            // mid-bounce settles: a reset is not a moment in the switch's life,
            // so leaving it chattering would be inventing a transition.
            s.bounces.clear();
            let held = s.pressed.clone();
            s.contacts.copy_from_slice(&held);
            // `sensed` is not touched: it belongs to whatever drives those
            // nets, and resetting this device does not move another device's
            // pin. `ticks` is not rewound either — `Machine::reset` does not
            // rewind clock domains (`ROADMAP.md` §4.2).
        }
        self.keys.touch();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.keys.state.lock();
        w.write_bytes(&pack(&s.pressed))?;
        w.write_bytes(&pack(&s.contacts))?;
        w.write_u64(s.ticks)?;
        w.write_u64(s.bounces.len() as u64)?;
        for b in &s.bounces {
            w.write_u32(b.key)?;
            w.write_u32(b.remaining)?;
            w.write_u64(b.next)?;
            w.write_bool(b.target)?;
        }
        Ok(())
        // `sensed` and `driving` are derived and are not written: the wire
        // sweep re-announces every net after a load (`ROADMAP.md` §4.5), and
        // `load` re-drives from what it read.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let pressed = r.read_bytes()?;
        let contacts = r.read_bytes()?;
        let ticks = r.read_u64()?;
        // A bounce is 17 bytes on the wire; claiming more than the chunk holds
        // is a corrupt snapshot, not an allocation request.
        let count = r.read_seq_len(17)?;
        let mut bounces = Vec::with_capacity((count as usize).min(MAX_KEYS));
        for _ in 0..count {
            bounces.push(Bounce {
                key: r.read_u32()?,
                remaining: r.read_u32()?,
                next: r.read_u64()?,
                target: r.read_bool()?,
            });
        }
        {
            let mut s = self.keys.state.lock();
            let keys = s.key_count();
            let mut bits = vec![false; keys];
            unpack(pressed, &mut bits);
            s.pressed.copy_from_slice(&bits);
            unpack(contacts, &mut bits);
            s.contacts.copy_from_slice(&bits);
            s.ticks = ticks;
            // A snapshot is untrusted input: a bounce naming a key this matrix
            // does not have is dropped rather than indexed with.
            bounces.retain(|b| (b.key as usize) < keys);
            bounces.sort_by_key(|b| (b.next, b.key));
            s.bounces = bounces;
        }
        self.keys.touch();
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let line = self.keys.line_of(port)?;
        self.keys.mark_wired(line);
        Some(SinkPin {
            sink: Arc::clone(&self.pin) as Arc<dyn WireSink>,
            line: line as u32,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(line) = self.keys.line_of(port) else {
            let (rows, cols) = (self.keys.rows(), self.keys.cols());
            return Err(Error::Config {
                at: String::from(port),
                message: format!(
                    "a keypad matrix has `{ROW_PORT}0`..`{ROW_PORT}{}` and \
                     `{COL_PORT}0`..`{COL_PORT}{}`, and every one of them is bidirectional",
                    rows - 1,
                    cols - 1
                ),
            });
        };
        self.keys.attach(line, source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.keys.touch();
    }

    /// Yes. A closed contact forwards a level from one conductor to the other
    /// within the instant and keeps nothing — which is what makes a loop
    /// through a keypad a machine-description error rather than a handshake.
    fn combinational(&self) -> bool {
        true
    }

    // -- lazily advanced (`ROADMAP.md` §4.2), and only when bouncing ---------

    /// Only with `bounce` set. A clean contact takes no time at all, and
    /// claiming a clock domain this device does not need would make every
    /// keypad in the tree require a `clock` property.
    fn is_lazy(&self) -> bool {
        self.bouncing
    }

    fn current_tick(&self) -> u64 {
        self.keys.state.lock().ticks
    }

    fn advance_to(&self, tick: u64) {
        self.keys.advance(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.keys.state.lock().bounces.first().map(|b| b.next)
    }
}

impl crate::machine::Instance for Keypad {}

/// The properties `keypad.matrix` takes.
static KEYPAD_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "rows",
        kind: ValueKind::Uint,
        required: true,
        summary: "how many row conductors, 1 to 16",
    },
    PropertySpec {
        name: "cols",
        kind: ValueKind::Uint,
        required: true,
        summary: "how many column conductors, 1 to 16",
    },
    PropertySpec {
        name: "diodes",
        kind: ValueKind::Bool,
        required: false,
        summary: "a series diode in each key, so no ghost keys (default false)",
    },
    PropertySpec {
        name: "layout",
        kind: ValueKind::Str,
        required: false,
        summary: "the key names, comma separated, row-major; empty means \"r,c\"",
    },
    PropertySpec {
        name: "keys",
        kind: ValueKind::Str,
        required: false,
        summary: "the host keypad port presses arrive through (default \"keypad\")",
    },
    PropertySpec {
        name: "bounce",
        kind: ValueKind::Uint,
        required: false,
        summary: "extra contact transitions per make or break (default 0, a clean contact)",
    },
    PropertySpec {
        name: "bounce-time",
        kind: ValueKind::Uint,
        required: false,
        summary: "the interval between them, in ticks of this device's clock domain",
    },
];

/// The `keypad.matrix` device class.
pub static KEYPAD_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a GPIO matrix keypad: R rows by C columns of switch contacts, with ghosting \
              unless every key has a diode",
    properties: KEYPAD_PROPERTIES,
    construct: |props| Ok(Box::new(Keypad::new(props)?) as Box<dyn Device>),
};

/// Add [`KEYPAD_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&KEYPAD_CLASS)
}

/// Bind [`KEYPAD_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Keypad::new(props)?)))
}

/// What the validator should know about `keypad.matrix`.
///
/// Both banks are declared at their maximum width rather than at the width the
/// object asked for, for the reason `machine::combinator`'s schemas are: the
/// validator does not evaluate properties, so it checks the *spelling* — `row3`
/// is a pin and `rowx` is a typo — and the count is checked at realize, where
/// the object exists.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .combinational()
        .prop(PropSchema::new("rows", ValueKind::Uint).range(1, MAX_ROWS))
        .prop(PropSchema::new("cols", ValueKind::Uint).range(1, MAX_COLS))
        .prop(PropSchema::new("diodes", ValueKind::Bool))
        .prop(PropSchema::new("layout", ValueKind::Str))
        .prop(PropSchema::new("keys", ValueKind::Str))
        .prop(PropSchema::new("bounce", ValueKind::Uint).range(0, 64))
        .prop(PropSchema::new("bounce-time", ValueKind::Uint))
        // Every conductor is an input *and* an output: the device senses what
        // the scan drives on it and shorts it to whatever a held key joins it
        // to. A machine file that scans one way names each pin in one `wire`
        // statement; one that scans both names it in two, and the resolver
        // folds those into one net.
        .port_bank(ROW_PORT, PortDir::InOut, MAX_ROWS as u32)
        .port_bank(COL_PORT, PortDir::InOut, MAX_COLS as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Pull, Wire, WireIdAllocator};

    /// A keypad and the nets around it: one wire per conductor, each with a
    /// pull-up, each carrying an external driver the test can move.
    struct Board {
        keypad: Keypad,
        /// The external driver of each conductor — the MCU's pin.
        outside: Vec<WireSource>,
        /// The nets, so a test can ask what a conductor is actually at.
        nets: Vec<Arc<Wire>>,
        rows: usize,
    }

    impl Board {
        fn new(rows: usize, cols: usize, diodes: bool) -> Board {
            Board::with_props(
                Props::new()
                    .with("rows", rows as u64)
                    .with("cols", cols as u64)
                    .with("diodes", diodes),
            )
        }

        fn with_props(props: Props) -> Board {
            let keypad = Keypad::new(&props).expect("a well-formed keypad");
            let (rows, cols) = (keypad.keys.rows(), keypad.keys.cols());
            let ids = WireIdAllocator::new();
            let mut outside = Vec::new();
            let mut nets = Vec::new();
            for line in 0..rows + cols {
                let port = if line < rows {
                    format!("{ROW_PORT}{line}")
                } else {
                    format!("{COL_PORT}{}", line - rows)
                };
                // Two sources: the keypad's contact and the scan driving the
                // line, on a net with the board's pull-up. Exactly what a
                // `wire … { pull = "up" }` statement builds.
                let (mine, theirs) = (ids.alloc(), ids.alloc());
                let sink = keypad
                    .sink(&port, &[mine, theirs])
                    .expect("every conductor is an input");
                assert_eq!(sink.line, line as u32, "the pin knows which conductor");
                let wire = Wire::builder()
                    .sources(&[mine, theirs])
                    .sink_weak(Arc::downgrade(&sink.sink), sink.line)
                    .resolved(Pull::Up)
                    .build_shared();
                keypad
                    .connect(&port, WireSource::new(Arc::clone(&wire), mine))
                    .expect("every conductor is an output too");
                outside.push(WireSource::new(Arc::clone(&wire), theirs));
                nets.push(wire);
            }
            // The realize sweep.
            keypad.announce(ROW_PORT);
            Board {
                keypad,
                outside,
                nets,
                rows,
            }
        }

        /// Drive row `r`, as a scan does: low to select it, Hi-Z to let the
        /// pull-up have it.
        fn drive_row(&self, r: usize, drive: Drive) {
            self.outside[Keys::row_line(r)].drive(drive);
        }

        /// The same for a column, for the symmetric scan.
        fn drive_col(&self, c: usize, drive: Drive) {
            self.outside[Keys::col_line(self.rows, c)].drive(drive);
        }

        fn row(&self, r: usize) -> Level {
            self.nets[Keys::row_line(r)].resolve_net()
        }

        fn col(&self, c: usize) -> Level {
            self.nets[Keys::col_line(self.rows, c)].resolve_net()
        }

        fn press(&self, r: usize, c: usize, down: bool) {
            let index = self
                .keypad
                .keys
                .index_at(r, c)
                .expect("a key of this matrix");
            self.keypad.keys.set(index, down);
        }

        /// The device's own snapshot, byte for byte.
        fn image(&self) -> Vec<u8> {
            let mut shape = MachineShape::new();
            shape.add_device("pad", CLASS_NAME).expect("a fresh shape");
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w
                    .chunk("pad", CLASS_NAME, STATE_VERSION)
                    .expect("the only chunk");
                Device::save(&self.keypad, &mut chunk).expect("a keypad saves");
            }
            w.to_vec().expect("a complete image")
        }
    }

    /// FNV-1a, so "the same state hash" is a hash rather than a memcmp.
    fn hash(bytes: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x100_0000_01b3);
        }
        h
    }

    #[test]
    fn a_pressed_key_pulls_its_column_low_only_while_its_row_is_driven_low() {
        let board = Board::new(4, 3, false);
        board.press(1, 2, true);

        board.drive_row(1, Drive::Low);
        assert_eq!(board.col(2), Level::Low, "row 1 is low and (1,2) is held");
        assert_eq!(board.col(0), Level::High, "nothing joins row 1 to col 0");
        assert_eq!(board.col(1), Level::High, "nor to col 1");

        board.drive_row(1, Drive::HiZ);
        board.drive_row(0, Drive::Low);
        assert_eq!(
            board.col(2),
            Level::High,
            "the key is on row 1, and row 1 is no longer selected"
        );
        assert_eq!(board.col(0), Level::High);
        assert_eq!(board.col(1), Level::High);
    }

    #[test]
    fn nothing_pressed_reads_all_columns_high_through_the_pull_ups() {
        let board = Board::new(4, 3, false);
        for r in 0..4 {
            board.drive_row(r, Drive::Low);
            for c in 0..3 {
                assert_eq!(board.col(c), Level::High, "row {r}, col {c}");
            }
            board.drive_row(r, Drive::HiZ);
        }
    }

    #[test]
    fn two_keys_in_one_column_are_both_readable_in_their_own_row_scan() {
        let board = Board::new(4, 3, false);
        board.press(0, 1, true);
        board.press(2, 1, true);

        board.drive_row(0, Drive::Low);
        assert_eq!(board.col(1), Level::Low, "(0,1) is held");
        board.drive_row(0, Drive::HiZ);

        board.drive_row(1, Drive::Low);
        assert_eq!(board.col(1), Level::High, "nothing is held on row 1");
        board.drive_row(1, Drive::HiZ);

        board.drive_row(2, Drive::Low);
        assert_eq!(board.col(1), Level::Low, "(2,1) is held");
        board.drive_row(2, Drive::HiZ);
    }

    #[test]
    fn three_corners_of_a_rectangle_ghost_the_fourth_without_diodes_and_not_with_them() {
        for diodes in [false, true] {
            let board = Board::new(4, 3, diodes);
            board.press(0, 0, true);
            board.press(0, 1, true);
            board.press(1, 0, true);

            board.drive_row(1, Drive::Low);
            assert_eq!(board.col(0), Level::Low, "(1,0) is really held");
            assert_eq!(
                board.col(1).is_low(),
                !diodes,
                "the fourth corner is a ghost exactly when there are no diodes"
            );
        }
    }

    #[test]
    fn a_column_scan_reads_the_rows_the_same_way_round() {
        // The matrix is symmetric without diodes: a firmware may drive a column
        // and read the rows.
        let board = Board::new(4, 3, false);
        board.press(2, 1, true);

        board.drive_col(1, Drive::Low);
        assert_eq!(board.row(2), Level::Low, "(2,1) shorts col 1 to row 2");
        assert_eq!(board.row(0), Level::High);
        assert_eq!(board.row(3), Level::High);
    }

    #[test]
    fn a_diode_stops_a_column_from_pulling_a_row_down() {
        let board = Board::new(4, 3, true);
        board.press(2, 1, true);
        board.drive_col(1, Drive::Low);
        assert_eq!(
            board.row(2),
            Level::High,
            "current flows row to column only, so a column scan reads nothing"
        );
    }

    #[test]
    fn releasing_a_key_lets_the_pull_up_have_the_column_back() {
        let board = Board::new(4, 3, false);
        board.drive_row(1, Drive::Low);
        board.press(1, 2, true);
        assert_eq!(board.col(2), Level::Low);
        board.press(1, 2, false);
        assert_eq!(board.col(2), Level::High, "the contact opened");
    }

    #[test]
    fn the_device_never_drives_a_conductor_high() {
        let board = Board::new(2, 2, false);
        board.press(0, 0, true);
        board.drive_row(0, Drive::Low);
        let driving = board.keypad.keys.state.lock().driving.clone();
        for (line, drive) in driving.iter().enumerate() {
            assert!(
                matches!(drive, Drive::Low | Drive::HiZ),
                "line {line} is a switch contact, not an output stage: {drive:?}"
            );
        }
    }

    #[test]
    fn a_key_can_be_named_by_a_layout_and_pressed_by_name() {
        let board = Board::with_props(
            Props::new()
                .with("rows", 2u64)
                .with("cols", 2u64)
                .with("layout", "1,2,ok,cancel"),
        );
        assert_eq!(board.keypad.keys.index_of("ok"), Some(2));
        assert_eq!(board.keypad.keys.name_of(3).as_deref(), Some("cancel"));
        board.drive_row(1, Drive::Low);
        assert!(board.keypad.keys.press("ok", true), "`ok` is a key");
        assert_eq!(board.col(0), Level::Low, "`ok` is the key at (1,0)");
        assert!(!board.keypad.keys.press("nope", true), "and `nope` is not");
    }

    #[test]
    fn without_a_layout_a_key_is_named_by_its_coordinates() {
        let board = Board::new(4, 3, false);
        assert_eq!(board.keypad.keys.name_of(5).as_deref(), Some("1,2"));
        assert_eq!(board.keypad.keys.index_of("1,2"), Some(5));
        assert_eq!(board.keypad.keys.index_of("9,9"), None);
    }

    #[test]
    fn a_layout_of_the_wrong_length_or_with_a_repeat_is_refused() {
        let short = Props::new()
            .with("rows", 2u64)
            .with("cols", 2u64)
            .with("layout", "a,b,c");
        assert!(Keypad::new(&short).is_err(), "three names, four keys");
        let repeat = Props::new()
            .with("rows", 2u64)
            .with("cols", 2u64)
            .with("layout", "a,b,a,c");
        assert!(Keypad::new(&repeat).is_err(), "`a` picks out two keys");
    }

    #[test]
    fn the_whole_held_state_travels_as_a_bitmap() {
        let board = Board::new(4, 3, false);
        board.set_mask_for_test();
        assert!(board.keypad.keys.pressed(0));
        assert!(board.keypad.keys.pressed(11));
        assert!(!board.keypad.keys.pressed(1));
        assert_eq!(board.keypad.keys.get(), alloc::vec![0x01, 0x08]);
    }

    impl Board {
        fn set_mask_for_test(&self) {
            // Bit 0 and bit 11 of a twelve-key matrix.
            self.keypad.keys.set_mask(&[0x01, 0x08]);
        }
    }

    #[test]
    fn a_bouncing_press_toggles_the_column_before_settling() {
        let board = Board::with_props(
            Props::new()
                .with("rows", 2u64)
                .with("cols", 2u64)
                .with("bounce", 3u64)
                .with("bounce-time", 10u64),
        );
        assert!(board.keypad.is_lazy(), "a bouncing keypad takes time");
        board.drive_row(0, Drive::Low);
        board.press(0, 1, true);

        // The first make is real, and the chatter follows it.
        assert_eq!(board.col(1), Level::Low, "the contact closed");
        assert_eq!(board.keypad.next_event_tick(), Some(10));

        board.keypad.advance_to(10);
        assert_eq!(board.col(1), Level::High, "and bounced open");
        board.keypad.advance_to(20);
        assert_eq!(board.col(1), Level::Low, "and shut again");
        board.keypad.advance_to(30);
        assert_eq!(
            board.col(1),
            Level::Low,
            "and settled where it was asked to"
        );
        assert_eq!(
            board.keypad.next_event_tick(),
            None,
            "with nothing left in flight"
        );
        assert_eq!(board.keypad.current_tick(), 30);
        assert_eq!(
            board.keypad.keys.get(),
            board.keypad.keys.contacts(),
            "a settled contact is where the host put it"
        );
    }

    #[test]
    fn a_clean_keypad_takes_no_time_and_needs_no_clock() {
        let board = Board::new(2, 2, false);
        assert!(
            !board.keypad.is_lazy(),
            "with `bounce = 0` there is nothing to schedule, so the realizer asks for no clock"
        );
        assert_eq!(board.keypad.next_event_tick(), None);
    }

    #[test]
    fn bounce_without_an_interval_is_refused() {
        let props = Props::new()
            .with("rows", 2u64)
            .with("cols", 2u64)
            .with("bounce", 4u64);
        assert!(
            Keypad::new(&props).is_err(),
            "every transition at one tick is one transition"
        );
    }

    #[test]
    fn a_geometry_outside_the_limits_is_refused() {
        for props in [
            Props::new().with("rows", 0u64).with("cols", 3u64),
            Props::new().with("rows", 17u64).with("cols", 3u64),
            Props::new().with("rows", 3u64).with("cols", 17u64),
            Props::new().with("cols", 3u64),
        ] {
            assert!(Keypad::new(&props).is_err(), "{props:?}");
        }
    }

    #[test]
    fn a_snapshot_round_trip_reaches_an_identical_state_hash() {
        let saved = Board::with_props(
            Props::new()
                .with("rows", 4u64)
                .with("cols", 3u64)
                .with("bounce", 3u64)
                .with("bounce-time", 10u64),
        );
        saved.drive_row(1, Drive::Low);
        saved.press(1, 2, true);
        saved.press(0, 0, true);
        saved.keypad.advance_to(10);
        let first = saved.image();

        let restored = Board::with_props(
            Props::new()
                .with("rows", 4u64)
                .with("cols", 3u64)
                .with("bounce", 3u64)
                .with("bounce-time", 10u64),
        );
        restored.drive_row(1, Drive::Low);
        let reader = StateReader::new(&first).expect("a well-formed image");
        let chunk = reader
            .load("pad", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("the keypad's own chunk");
        Device::load(&restored.keypad, &mut chunk.reader()).expect("a keypad loads");

        assert_eq!(
            hash(&restored.image()),
            hash(&first),
            "the same keypad, bit for bit"
        );
        // And it is a live keypad, not just the same bytes: the restored
        // bounce runs out where the saved one would have.
        restored.keypad.advance_to(20);
        restored.keypad.advance_to(30);
        assert_eq!(restored.col(2), Level::Low, "(1,2) settled held");
        assert_eq!(restored.keypad.next_event_tick(), None);
    }

    #[test]
    fn a_reset_settles_a_bouncing_contact_without_lifting_a_thumb() {
        let board = Board::with_props(
            Props::new()
                .with("rows", 2u64)
                .with("cols", 2u64)
                .with("bounce", 5u64)
                .with("bounce-time", 4u64),
        );
        board.drive_row(0, Drive::Low);
        board.press(0, 1, true);
        board.keypad.advance_to(4);
        assert_eq!(board.col(1), Level::High, "mid-bounce");

        board.keypad.reset(ResetKind::Cold);
        assert!(board.keypad.keys.pressed(1), "the key is still held");
        assert_eq!(board.col(1), Level::Low, "and the contact settled closed");
        assert_eq!(board.keypad.next_event_tick(), None);
    }

    #[test]
    fn two_devices_may_share_one_host_keypad_only_if_they_agree_about_it() {
        let hosts = Arc::new(crate::core::hosts::HostObjects::new());
        let props = |cols: u64| {
            Props::new()
                .with("rows", 4u64)
                .with("cols", cols)
                .with_hosts(Arc::clone(&hosts))
        };
        let first = Keypad::new(&props(3)).expect("the first opens it");
        assert!(Keypad::new(&props(3)).is_ok(), "the second agrees");
        assert!(
            Keypad::new(&props(4)).is_err(),
            "and the third describes a different keypad"
        );
        drop(first);
    }

    #[test]
    fn a_recorded_movement_is_two_bytes_and_lands_on_the_matrix() {
        use crate::core::record::InputSink;
        let keys = Arc::new(Keys::new());
        keys.configure(&Geometry {
            rows: 4,
            cols: 3,
            diodes: false,
            bounce: 0,
            bounce_ticks: 0,
            names: Vec::new(),
        })
        .expect("a fresh keypad");
        let sink: Arc<dyn InputSink> = keys::sink(&keys);

        assert_eq!(keys::RECORD_BYTES, 2);
        sink.deliver(&[5, 1, 0, 1]);
        assert!(
            keys.pressed(5) && keys.pressed(0),
            "two movements, in order"
        );
        sink.deliver(&[5, 0]);
        assert!(!keys.pressed(5) && keys.pressed(0), "and one release");
        // Untrusted input: a recording made against a bigger keypad must not
        // abort the replay.
        sink.deliver(&[200, 1]);
        sink.deliver(&[7]);
        assert_eq!(keys.get(), alloc::vec![0x01, 0x00]);
    }
}
