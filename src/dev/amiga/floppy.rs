//! An Amiga floppy drive: the mechanism on the CIA ports, and the cells under
//! the head for Paula.
//!
//! One class, `amiga.floppy`. The disk controller is split across three chips
//! and this is the part that is none of them: the drive at the end of the
//! cable.
//!
//! ```text
//!   object df0 "amiga.floppy" { clock = clk / 8, paula = paula }
//!
//!   wire cia_b.pb7 -> df0.mtr   { pull = "up" }    # MTR*
//!   wire cia_b.pb3 -> df0.sel   { pull = "up" }    # SEL0*
//!   wire cia_b.pb2 -> df0.side  { pull = "up" }    # SIDE*
//!   wire cia_b.pb1 -> df0.dir   { pull = "up" }    # DIR
//!   wire cia_b.pb0 -> df0.step  { pull = "up" }    # STEP*
//!
//!   wire df0.rdy   -> cia_a.pa5 { pull = "up" }    # RDY*
//!   wire df0.tk0   -> cia_a.pa4 { pull = "up" }    # TK0*
//!   wire df0.wpro  -> cia_a.pa3 { pull = "up" }    # WPRO*
//!   wire df0.chng  -> cia_a.pa2 { pull = "up" }    # CHNG*
//!   wire df0.index -> cia_b.flag { pull = "up" }   # INDEX*
//! ```
//!
//! Every pin carries its **electrical level**: the CIA writes a zero to
//! `PB7` to turn a motor on, and this drive pulls `RDY*` to ground to say it is
//! ready. The outputs are open-collector — driven low or let go — so a
//! deselected drive leaves the shared lines to the CIA's pull-ups and to
//! whichever drive is selected.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//! chapter 8's Table 8-5 ("Disk Subsystem") for what each CIA bit does, and
//! Appendix E's external disk connector for what a drive does with it —
//! "Motor on data, clocked into drive's motor-on flip-flop by the active
//! transition of SELxB*", the change flop that "is reset when drive is
//! selected and the head stepped, but only if a disk is installed", "Side 1 if
//! active", "Inactive to step towards center", an index pulse "once per disk
//! revolution" from the selected drive, and the 32-bit identification stream
//! read on `RDY*` with the motor off. No emulator source was consulted.
//!
//! # What the manual does not give, and what is chosen instead
//!
//! * **The spindle speed.** A track is [`TRACK_CELLS`] cells: 100 000, the
//!   200 ms of a 300 rpm drive at the manual's two-microsecond cell. The
//!   rotation is a function of absolute time — cell `n` of every track passes
//!   the head at tick `7n` of each revolution — so a track reads back where it
//!   was written, and Paula's cell clock and the drive agree by construction.
//! * **The index pulse's width**, [`INDEX_CELLS`] cells — about two
//!   milliseconds. What the CIA sees is the falling edge.
//! * **Spin-up.** `RDY*` asserts as soon as the motor flop is set with a disk
//!   in. The manual tells software to wait "one half second (500ms), or for the
//!   DSKRDY* line to go low", so software that waits either way works.
//! * **The identification word** is `$FFFF FFFF`, "Amiga standard 3.25
//!   diskette" (sic).
//!
//! # The disk
//!
//! [`MfmDisk`] is raw MFM: 160 tracks of cells, cylinder 0 side 0 first, as a
//! controller would see them. The `image` media slot takes exactly that, back
//! to back ([`MfmDisk::from_raw`]) — a layout of this crate's own, not an Amiga
//! format. Nothing here encodes a file system; turning an ADF's sectors into
//! AmigaDOS tracks is a separate job, and the tests build their tracks by hand.
//! Writes land in the tracks and are part of the snapshot; nothing is written
//! back to the host.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

use super::paula::{DiskDrive, FAST_CELL_TICKS, PaulaPort};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.floppy";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// Cylinders on a double-density disk: 80, which with two sides and 11 sectors
/// of 512 bytes is the manual's "over 900,000 bytes per disk".
pub const CYLINDERS: u8 = 80;

/// Tracks: two per cylinder.
pub const TRACKS: usize = CYLINDERS as usize * 2;

/// Cells per revolution. See the module docs.
pub const TRACK_CELLS: u64 = 100_000;

/// Bytes of raw MFM a track holds.
pub const TRACK_BYTES: usize = (TRACK_CELLS / 8) as usize;

/// How long the index pulse holds `INDEX*` low, in cells.
pub const INDEX_CELLS: u64 = 1_000;

/// One revolution in colour clocks.
const REVOLUTION_TICKS: u64 = TRACK_CELLS * FAST_CELL_TICKS;

/// The identification word read on `RDY*` with the motor off (Appendix E,
/// "External Disk Connector Defined Identifications").
pub const DRIVE_ID: u32 = 0xffff_ffff;

/// The drive's lock rank: above [`LockRank::DEVICE`], because Paula asks for
/// cells while holding its own state lock, and below [`LockRank::WIRE`].
pub const MEDIA_RANK: LockRank = LockRank::new(0x5400);

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

// -- pins -------------------------------------------------------------------

const LINE_MTR: u32 = 0;
const LINE_SEL: u32 = 1;
const LINE_SIDE: u32 = 2;
const LINE_DIR: u32 = 3;
const LINE_STEP: u32 = 4;

/// The inputs, in line order.
pub const INPUT_PINS: [&str; 5] = ["mtr", "sel", "side", "dir", "step"];

/// The outputs, in the order the drive holds their wires.
pub const OUTPUT_PINS: [&str; 5] = ["rdy", "tk0", "wpro", "chng", "index"];

// ---------------------------------------------------------------------------
// the disk
// ---------------------------------------------------------------------------

/// A disk as the head sees it: raw MFM cells, one track per cylinder and side.
#[derive(Clone, PartialEq, Eq)]
pub struct MfmDisk {
    /// [`TRACKS`] tracks of [`TRACK_BYTES`] bytes, most significant cell first.
    tracks: Vec<Vec<u8>>,
    /// Whether the write-protect tab is open.
    pub write_protected: bool,
}

impl fmt::Debug for MfmDisk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MfmDisk")
            .field("tracks", &self.tracks.len())
            .field("write_protected", &self.write_protected)
            .finish()
    }
}

impl Default for MfmDisk {
    fn default() -> MfmDisk {
        MfmDisk::blank()
    }
}

impl MfmDisk {
    /// An unformatted disk: no flux anywhere.
    #[must_use]
    pub fn blank() -> MfmDisk {
        MfmDisk {
            tracks: vec![vec![0; TRACK_BYTES]; TRACKS],
            write_protected: false,
        }
    }

    /// A disk from a raw dump: [`TRACKS`] tracks of [`TRACK_BYTES`] bytes back
    /// to back, cylinder 0 side 0 first. `None` if the length is anything
    /// else.
    ///
    /// This layout is this crate's own — the cells exactly as the head passes
    /// them, with nothing encoded — and not an Amiga file format.
    #[must_use]
    pub fn from_raw(bytes: &[u8]) -> Option<MfmDisk> {
        if bytes.len() != TRACKS * TRACK_BYTES {
            return None;
        }
        Some(MfmDisk {
            tracks: bytes.chunks(TRACK_BYTES).map(<[u8]>::to_vec).collect(),
            write_protected: false,
        })
    }

    /// Replace one track's cells, starting at the index. Shorter data leaves
    /// the rest of the track blank; longer data is cut at the revolution.
    ///
    /// # Panics
    ///
    /// If `track` is not below [`TRACKS`].
    pub fn set_track(&mut self, track: usize, mfm: &[u8]) {
        let t = &mut self.tracks[track];
        t.fill(0);
        let n = mfm.len().min(TRACK_BYTES);
        t[..n].copy_from_slice(&mfm[..n]);
    }

    /// One track's cells.
    ///
    /// # Panics
    ///
    /// If `track` is not below [`TRACKS`].
    #[must_use]
    pub fn track(&self, track: usize) -> &[u8] {
        &self.tracks[track]
    }

    fn cell(&self, track: usize, pos: u64) -> u8 {
        let byte = self.tracks[track][(pos / 8) as usize];
        (byte >> (7 - pos % 8)) & 1
    }

    fn set_cell(&mut self, track: usize, pos: u64, cell: u8) {
        let byte = &mut self.tracks[track][(pos / 8) as usize];
        let mask = 0x80 >> (pos % 8);
        if cell & 1 != 0 {
            *byte |= mask;
        } else {
            *byte &= !mask;
        }
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Colour clocks simulated.
    ticks: u64,
    // The input levels as last seen.
    mtr: bool,
    sel: bool,
    side: bool,
    dir: bool,
    step: bool,
    /// Whether `SEL*` is active.
    selected: bool,
    /// The motor-on flop.
    motor: bool,
    cylinder: u8,
    /// The disk-change flop: `CHNG*` asserted.
    changed: bool,
    /// The next identification bit to present.
    id_next: u8,
    /// The identification bit `RDY*` shows while selected with the motor off.
    id_bit: bool,
    disk: Option<MfmDisk>,
}

impl State {
    fn power_on() -> State {
        State {
            ticks: 0,
            mtr: true,
            sel: true,
            side: true,
            dir: true,
            step: true,
            selected: false,
            motor: false,
            cylinder: 0,
            // "Drive's change flop is set at power up."
            changed: true,
            id_next: 0,
            id_bit: false,
            disk: None,
        }
    }

    fn spinning(&self) -> bool {
        self.selected && self.motor && self.disk.is_some()
    }

    fn track(&self) -> usize {
        // "Side 1 if active, side 0 if inactive": SIDE* low is side 1.
        usize::from(self.cylinder) * 2 + usize::from(!self.side)
    }

    fn index_low(&self) -> bool {
        self.spinning() && self.ticks % REVOLUTION_TICKS < INDEX_CELLS * FAST_CELL_TICKS
    }

    /// What each output stage is doing: `true` is pulling the line low.
    fn pins(&self) -> [bool; 5] {
        let sel = self.selected;
        let disk = self.disk.as_ref();
        [
            sel && if self.motor {
                disk.is_some()
            } else {
                self.id_bit
            },
            sel && self.cylinder == 0,
            sel && disk.is_some_and(|d| d.write_protected),
            sel && self.changed,
            self.index_low(),
        ]
    }

    fn next_event(&self) -> u64 {
        if !self.spinning() {
            return NO_EVENT;
        }
        let phase = self.ticks % REVOLUTION_TICKS;
        let width = INDEX_CELLS * FAST_CELL_TICKS;
        if phase < width {
            self.ticks - phase + width
        } else {
            self.ticks - phase + REVOLUTION_TICKS
        }
    }

    fn input(&mut self, line: u32, level: bool) {
        match line {
            LINE_MTR => self.mtr = level,
            LINE_SEL => {
                let falling = self.sel && !level;
                self.sel = level;
                if level {
                    self.selected = false;
                } else if falling {
                    self.select();
                }
            }
            LINE_SIDE => self.side = level,
            LINE_DIR => self.dir = level,
            LINE_STEP => {
                let falling = self.step && !level;
                self.step = level;
                if falling && self.selected {
                    self.step_head();
                }
            }
            _ => {}
        }
    }

    /// The active transition of `SEL*`: it clocks the motor flop, and with
    /// the motor off it clocks out an identification bit.
    fn select(&mut self) {
        self.selected = true;
        let was = self.motor;
        self.motor = !self.mtr;
        if was && !self.motor {
            // "The transition from motor on to motor off reinitializes the
            // serial shift register."
            self.id_next = 0;
        }
        if !self.motor {
            self.id_bit = DRIVE_ID >> (31 - self.id_next) & 1 != 0;
            self.id_next = (self.id_next + 1) % 32;
        }
    }

    fn step_head(&mut self) {
        // DIR high steps out, towards track 0; low steps in. A drive "must
        // refuse to step outward" at track 0.
        if self.dir {
            self.cylinder = self.cylinder.saturating_sub(1);
        } else if self.cylinder + 1 < CYLINDERS {
            self.cylinder += 1;
        }
        if self.disk.is_some() {
            self.changed = false;
        }
    }
}

// ---------------------------------------------------------------------------
// shared
// ---------------------------------------------------------------------------

/// The output stages.
#[derive(Debug, Clone, Default)]
struct Outputs {
    pins: [Option<WireSource>; 5],
}

struct Shared {
    state: Mutex<State>,
    ticks: AtomicU64,
    next_event: AtomicU64,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
    paula: Mutex<Option<PaulaPort>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(state.next_event(), Ordering::Relaxed);
    }

    /// Drive every output and tell Paula, with no lock held while doing
    /// either.
    fn refresh(&self) {
        let pins = self.state.lock().pins();
        let out = self.out.lock().clone();
        for (src, low) in out.pins.iter().zip(pins) {
            if let Some(src) = src {
                src.drive(if low { Drive::Low } else { Drive::HiZ });
            }
        }
        let paula = self.paula.lock().clone();
        if let Some(paula) = paula {
            paula.drive_changed();
        }
    }

    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    fn update(&self, f: impl FnOnce(&mut State)) {
        let moved = {
            let mut state = self.state.lock();
            let before = (state.pins(), state.spinning());
            f(&mut state);
            self.publish(&state);
            before != (state.pins(), state.spinning())
        };
        if moved {
            self.refresh();
        }
    }

    fn advance_to(&self, target: u64) {
        self.update(|st| {
            if target > st.ticks {
                st.ticks = target;
            }
        });
    }
}

impl DiskDrive for Shared {
    fn reading(&self) -> bool {
        self.state.lock().spinning()
    }

    fn read_cells(&self, start: u64, cell: u64, out: &mut [u8]) {
        let state = self.state.lock();
        if !state.spinning() {
            return;
        }
        let track = state.track();
        let Some(disk) = state.disk.as_ref() else {
            return;
        };
        for (i, o) in out.iter_mut().enumerate() {
            let at = start + i as u64 * cell;
            *o |= disk.cell(track, (at / FAST_CELL_TICKS) % TRACK_CELLS);
        }
    }

    fn write_cells(&self, start: u64, cell: u64, cells: &[u8]) {
        let mut state = self.state.lock();
        if !state.spinning() {
            return;
        }
        let track = state.track();
        let Some(disk) = state.disk.as_mut() else {
            return;
        };
        if disk.write_protected {
            return;
        }
        for (i, c) in cells.iter().enumerate() {
            let at = start + i as u64 * cell;
            disk.set_cell(track, (at / FAST_CELL_TICKS) % TRACK_CELLS, *c);
        }
    }
}

// ---------------------------------------------------------------------------
// the pins
// ---------------------------------------------------------------------------

/// One input.
#[derive(Debug)]
struct InputPin {
    shared: Arc<Shared>,
    line: u32,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.shared.sync();
        let line = self.line;
        self.shared.update(|st| st.input(line, high));
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// A floppy drive.
#[derive(Debug)]
pub struct Floppy {
    shared: Arc<Shared>,
    paula_path: String,
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Floppy {
    /// Validate `props` and build an empty drive.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `paula` is missing, or a property this class does
    /// not know was given.
    pub fn new(props: &Props) -> Result<Floppy> {
        let mut r = props.reader();
        let paula_path = r.require_link("paula")?.as_str().to_string();
        let image = r.optional_media("image")?;
        let protected: bool = r.or("write-protected", false)?;
        r.finish()?;
        let drive = Floppy::bare(paula_path);
        if let Some(image) = image.filter(|m| !m.is_empty()) {
            let mut disk = MfmDisk::from_raw(image.bytes()).ok_or_else(|| {
                Error::Property(format!(
                    "property `image`: `{}` is {} bytes, and a raw MFM disk is {} tracks of \
                     {TRACK_BYTES} bytes, {} in all",
                    image.name(),
                    image.len(),
                    TRACKS,
                    TRACKS * TRACK_BYTES
                ))
            })?;
            disk.write_protected = protected;
            drive.shared.state.lock().disk = Some(disk);
        }
        Ok(drive)
    }

    /// A drive whose controller is the object at `paula_path`.
    #[must_use]
    pub fn bare(paula_path: String) -> Floppy {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(MEDIA_RANK, State::power_on()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            paula: Mutex::with_rank(LockRank::LEAF, None),
        });
        Floppy {
            shared,
            paula_path,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Put a disk in. The change flop stays set until the head is stepped.
    pub fn insert(&self, disk: MfmDisk) {
        self.shared.update(|st| st.disk = Some(disk));
    }

    /// Take the disk out, which sets the change flop.
    pub fn eject(&self) -> Option<MfmDisk> {
        let mut taken = None;
        self.shared.update(|st| {
            taken = st.disk.take();
            st.changed = true;
        });
        taken
    }

    /// A copy of the disk in the drive, writes included.
    #[must_use]
    pub fn disk(&self) -> Option<MfmDisk> {
        self.shared.state.lock().disk.clone()
    }

    /// The cylinder the head is over.
    #[must_use]
    pub fn cylinder(&self) -> u8 {
        self.shared.state.lock().cylinder
    }

    /// Whether the motor flop is set.
    #[must_use]
    pub fn motor(&self) -> bool {
        self.shared.state.lock().motor
    }

    /// Whether `SEL*` is active.
    #[must_use]
    pub fn selected(&self) -> bool {
        self.shared.state.lock().selected
    }

    /// Attach to a controller by hand, as `bind` does from a machine file.
    pub fn attach(&self, paula: PaulaPort) {
        paula.attach_drive(Arc::clone(&self.shared) as Arc<dyn DiskDrive>);
        *self.shared.paula.lock() = Some(paula);
    }

    /// Run the drive until `target` colour clocks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl Device for Floppy {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // DRESB*: "Drives should reset their motor-on flip-flops." The head,
        // the disk and the change flop are mechanical and stay where they are.
        self.shared.update(|st| st.motor = false);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.shared.state.lock().clone();
        w.write_u64(st.ticks)?;
        for v in [
            st.mtr,
            st.sel,
            st.side,
            st.dir,
            st.step,
            st.selected,
            st.motor,
            st.changed,
            st.id_bit,
        ] {
            w.write_bool(v)?;
        }
        w.write_u8(st.cylinder)?;
        w.write_u8(st.id_next)?;
        w.write_bool(st.disk.is_some())?;
        if let Some(disk) = &st.disk {
            w.write_bool(disk.write_protected)?;
            for track in &disk.tracks {
                w.write_bytes(track)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut st = State::power_on();
        st.ticks = r.read_u64()?;
        st.mtr = r.read_bool()?;
        st.sel = r.read_bool()?;
        st.side = r.read_bool()?;
        st.dir = r.read_bool()?;
        st.step = r.read_bool()?;
        st.selected = r.read_bool()?;
        st.motor = r.read_bool()?;
        st.changed = r.read_bool()?;
        st.id_bit = r.read_bool()?;
        st.cylinder = r.read_u8()?.min(CYLINDERS - 1);
        st.id_next = r.read_u8()? % 32;
        if r.read_bool()? {
            let mut disk = MfmDisk::blank();
            disk.write_protected = r.read_bool()?;
            for track in &mut disk.tracks {
                let bytes = r.read_bytes()?;
                if bytes.len() != TRACK_BYTES {
                    return Err(Error::State(format!(
                        "a floppy track of {} bytes; a track is {TRACK_BYTES}",
                        bytes.len()
                    )));
                }
                track.copy_from_slice(bytes);
            }
            st.disk = Some(disk);
        }
        {
            let mut state = self.shared.state.lock();
            *state = st;
            self.shared.publish(&state);
        }
        self.shared.refresh();
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(i) = OUTPUT_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "a floppy drive drives `rdy`, `tk0`, `wpro`, `chng` and `index`",
                ),
            });
        };
        self.shared.out.lock().pins[i] = Some(source);
        self.shared.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = INPUT_PINS.iter().position(|p| *p == port)? as u32;
        let pin = Arc::new(InputPin {
            shared: Arc::clone(&self.shared),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    fn is_lazy(&self) -> bool {
        // The index pulse has to reach the CIA on its own tick.
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Floppy {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let paula = ctx
            .export_as::<PaulaPort>(&self.paula_path, ExportId::PAULA)
            .map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!("`paula` has to name an `amiga.paula`: {e}"),
            })?;
        self.attach(PaulaPort::clone(&paula));
        Ok(())
    }
}

/// The `amiga.floppy` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Amiga floppy drive: motor, select, step and side from the CIAs, RDY/TK0/WPRO/\
              CHNG and the index pulse back, and raw MFM cells for Paula",
    properties: &[
        PropertySpec {
            name: "paula",
            kind: ValueKind::Link,
            required: true,
            summary: "the `amiga.paula` whose read line the drive is on",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot a raw MFM disk is bound to; absent or empty is no disk",
        },
        PropertySpec {
            name: "write-protected",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the disk's write-protect tab is open",
        },
    ],
    construct: |props| Ok(Box::new(Floppy::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Floppy::new(props)?)))
}

/// What the validator should know about `amiga.floppy`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("paula", ValueKind::Link).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("write-protected", ValueKind::Bool));
    for pin in INPUT_PINS {
        schema = schema.port(pin, PortDir::In);
    }
    for pin in OUTPUT_PINS {
        schema = schema.port(pin, PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests;
