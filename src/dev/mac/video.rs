//! The Macintosh's video circuit: 512 × 342 one-bit pixels read straight out
//! of main memory, and the two blanking signals that come back off the beam.
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition — the "Video" chapter
//! for the raster, and chapter 3 for where the screen buffer sits relative to
//! the top of RAM. No emulator source was consulted (`ROADMAP.md` §1).
//!
//! # There is no video chip
//!
//! That is the whole design. A Macintosh Plus has no frame buffer of its own:
//! a counter walks main memory in step with the beam and shifts the bits out,
//! stealing the cycles it needs from the processor. So this device is a **bus
//! master with no registers** — nothing in the machine's address space belongs
//! to it. What it owns is a position in the raster, two output pins, and the
//! ability to read guest memory when a host asks for a picture.
//!
//! Reading at capture time rather than latching each line is the same decision
//! `dev::lcd::scanout` makes and for the same reason: software that redraws
//! without waiting for the blanking interval tears on real hardware, and a
//! model that quietly stabilised the picture would hide the guest's bug.
//!
//! # The raster
//!
//! One dot clock of 15.6672 MHz, twice the processor's 7.8336 MHz:
//!
//! ```text
//!   704 dot clocks a line   =  512 visible + 192 blanked  →  22.2545 kHz
//!   370 lines a frame       =  342 visible +  28 blanked  →  60.15 Hz
//! ```
//!
//! A one bit is **black**: the Macintosh's screen is white and the bits are
//! ink. The most significant bit of each byte is the leftmost pixel, and a row
//! is 64 bytes, so a frame is 21,888 bytes.
//!
//! # Where the buffer is
//!
//! Not at a fixed address — at a fixed distance below the *top of installed
//! memory*, which is why the ROM has to size memory before it can draw
//! anything:
//!
//! ```text
//!   main screen buffer        top - $5900
//!   alternate screen buffer   top - $D900
//! ```
//!
//! The VIA's `PAGE2` bit (`PA6`) picks between them, and it picks the
//! *alternate* buffer when it is **low**. The 896 bytes between the main
//! buffer's end and the top of memory are the sound and disk-speed buffer,
//! which is why the two are the distance apart that they are.
//!
//! # The two pins
//!
//! `vblank` is the vertical blanking signal that reaches the VIA's `CA1` and
//! raises the 60.15 Hz interrupt every Macintosh's tick chain is built on. It
//! is driven high for the 28 blanked lines and low for the 342 visible ones,
//! so whichever edge software programs `PCR` to select, it gets one a frame.
//!
//! `hblank` is `H4`, which reaches the VIA's `PB6` — software polls it rather
//! than taking an interrupt from it. It is only *scheduled* when a machine
//! file wires it: a level that changes 44,509 times a second is 44,509
//! scheduler visits a second, and a board that does not read the pin should
//! not pay for them.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AddressSpace, MemAttrs, RegionRef, RequesterId, UnassignedPolicy};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.video";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// Visible pixels across.
pub const WIDTH: u32 = 512;
/// Visible lines.
pub const HEIGHT: u32 = 342;
/// Bytes in one row of the screen buffer: `WIDTH / 8`.
pub const ROW_BYTES: u64 = (WIDTH as u64) / 8;
/// Bytes in a whole frame.
pub const FRAME_BYTES: u64 = ROW_BYTES * HEIGHT as u64;

/// Dot clocks in one scan line: 512 visible and 192 blanked.
pub const DOTS_PER_LINE: u64 = 704;
/// Scan lines in one frame: 342 visible and 28 blanked.
pub const LINES_PER_FRAME: u64 = 370;
/// Dot clocks in one frame.
pub const FRAME_TICKS: u64 = DOTS_PER_LINE * LINES_PER_FRAME;
/// The tick within a frame at which vertical blanking begins.
const VBLANK_AT: u64 = DOTS_PER_LINE * HEIGHT as u64;

/// How far below the top of memory the main screen buffer starts.
pub const MAIN_OFFSET: u64 = 0x5900;
/// How far below the top of memory the alternate screen buffer starts.
pub const ALT_OFFSET: u64 = 0xd900;

/// The output pin that carries vertical blanking.
const VBLANK_PIN: &str = "vblank";
/// The output pin that carries `H4`.
const HBLANK_PIN: &str = "hblank";
/// The input pin the VIA's `PA6` drives.
const PAGE2_PIN: &str = "page2";

/// `next_event` when there is nothing to wake up for.
const NO_EVENT: u64 = u64::MAX;

/// Where the beam is and what the pins say.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Dot clocks since reset.
    ticks: u64,
    /// The level the VIA is driving onto `PAGE2`. High selects the main
    /// buffer; low selects the alternate one.
    ///
    /// Snapshotted, for the reason `mac.via` gives about its own pins: a
    /// restore ends with the realize sweep, and a level that comes back wrong
    /// makes that sweep look like a change.
    page2: bool,
}

impl State {
    fn fresh(ticks: u64) -> State {
        State { ticks, page2: true }
    }

    /// Whether the beam is in vertical blanking at `ticks`.
    fn vblank_at(ticks: u64) -> bool {
        ticks % FRAME_TICKS >= VBLANK_AT
    }

    /// Whether the beam is in horizontal blanking at `ticks`.
    fn hblank_at(ticks: u64) -> bool {
        ticks % DOTS_PER_LINE >= WIDTH as u64
    }

    /// How many frames have begun their vertical blanking by `ticks`.
    ///
    /// The frame counter a host compares against a surface's serial. Closed
    /// form rather than a loop: a machine may run a virtual minute between two
    /// captures.
    fn frames_at(ticks: u64) -> u64 {
        if ticks < VBLANK_AT {
            0
        } else {
            (ticks - VBLANK_AT) / FRAME_TICKS + 1
        }
    }

    /// The next tick at which a pin this device drives changes level.
    ///
    /// `hblank` is only counted when something is wired to it, because a pin
    /// nobody reads is not worth 44,509 wake-ups a second.
    fn next_event(&self, hblank_wired: bool) -> u64 {
        let frame = self.ticks % FRAME_TICKS;
        let mut next = if frame < VBLANK_AT {
            self.ticks + (VBLANK_AT - frame)
        } else {
            self.ticks + (FRAME_TICKS - frame)
        };
        if hblank_wired {
            let dot = self.ticks % DOTS_PER_LINE;
            let line = if dot < WIDTH as u64 {
                self.ticks + (WIDTH as u64 - dot)
            } else {
                self.ticks + (DOTS_PER_LINE - dot)
            };
            next = next.min(line);
        }
        next
    }
}

/// Everything the device and its host adapter share.
struct Shared {
    state: Mutex<State>,
    /// Main memory, in a private space of this object's own at base zero.
    /// `None` until bind.
    bus: Mutex<Option<Arc<AddressSpace>>>,
    /// How many bytes of it there are — the top the buffer hangs below.
    ram_len: AtomicU64,
    /// The requester the reads carry.
    requester: Mutex<RequesterId>,
    /// The output pins, `None` until a `wire` statement claims one.
    out: Mutex<Outputs>,
    /// Published without a lock, for the scheduler.
    ticks: AtomicU64,
    next_event: AtomicU64,
    /// Frames completed, for a host that draws only on change.
    frames: AtomicU64,
    /// The catch-up handle a capture syncs through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
}

#[derive(Default, Clone)]
struct Outputs {
    vblank: Option<WireSource>,
    hblank: Option<WireSource>,
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Video");
        s.field("ram", &self.ram_len.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    fn publish(&self, state: &State, hblank_wired: bool) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event
            .store(state.next_event(hblank_wired), Ordering::Relaxed);
        self.frames
            .store(State::frames_at(state.ticks), Ordering::Relaxed);
    }

    fn hblank_wired(&self) -> bool {
        self.out.lock().hblank.is_some()
    }

    /// Drive both pins to whatever the beam position now says.
    ///
    /// With no lock held: a wire delivers synchronously, so driving from
    /// inside the critical section would run the VIA's sink under this
    /// device's lock. Mutate, release, *then* call outward (`CLAUDE.md`).
    fn refresh(&self) {
        let ticks = self.state.lock().ticks;
        let out = self.out.lock().clone();
        // `drive` rather than `set`, and the difference is the realize sweep.
        // A fresh source is already at `Level::Low`, so `set(Low)` is not a
        // change and never reaches the far end — which left the VIA holding
        // the pull-up its pin has when nothing is wired to it, and the first
        // blanking interval then looked like an edge that had already
        // happened. A push-pull output is what this circuit really has, and
        // announcing it as one makes the sweep do its job.
        if let Some(src) = &out.vblank {
            src.drive(Drive::strong(Level::from(State::vblank_at(ticks))));
        }
        if let Some(src) = &out.hblank {
            src.drive(Drive::strong(Level::from(State::hblank_at(ticks))));
        }
    }

    fn advance_to(&self, target: u64) {
        let wired = self.hblank_wired();
        let changed = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            let before = (State::vblank_at(state.ticks), State::hblank_at(state.ticks));
            state.ticks = target;
            self.publish(&state, wired);
            before != (State::vblank_at(target), State::hblank_at(target))
        };
        if changed {
            self.refresh();
        }
    }

    /// Bring the beam up to date before a capture.
    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    /// Where the frame being displayed starts in main memory.
    fn base(&self) -> u64 {
        let top = self.ram_len.load(Ordering::Relaxed);
        let offset = if self.state.lock().page2 {
            MAIN_OFFSET
        } else {
            ALT_OFFSET
        };
        top.saturating_sub(offset)
    }
}

/// The video circuit, as a host adapter sees it.
///
/// Held by both the device and `host::display::mac`, which is why it is a
/// separate type: `Device` has no `Any` in its supertrait chain, so a host
/// keeps its handle from construction.
#[derive(Debug)]
pub struct Screen {
    shared: Arc<Shared>,
}

impl Screen {
    /// Frames whose vertical blanking has begun since reset.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    /// The picture's shape, which on this machine never changes.
    #[must_use]
    pub fn geometry(&self) -> (u32, u32) {
        (WIDTH, HEIGHT)
    }

    /// Where the frame being displayed starts in main memory.
    #[must_use]
    pub fn base(&self) -> u64 {
        self.shared.base()
    }

    /// How long one frame lasts, in ticks of this device's clock domain.
    #[must_use]
    pub fn frame_ticks(&self) -> u64 {
        FRAME_TICKS
    }

    /// Copy the frame on screen now into `dst` as packed bits, and say how
    /// big it is and which frame it is.
    ///
    /// `dst` is 64 bytes a row, 342 rows, most significant bit leftmost, a one
    /// bit black. It is resized and completely overwritten; a read that the
    /// address space refuses leaves those bytes zero, which is a white screen
    /// rather than a panic — a base pointing into a hole is a guest bug, not
    /// ours.
    ///
    /// The reads carry `MemAttrs::debug`, because a video circuit walking
    /// memory must not pop a FIFO or advance anybody's pointer.
    pub fn copy_frame(&self, dst: &mut Vec<u8>) -> (u32, u32, u64) {
        // Catch the beam up first, so the frame counter reported here is the
        // one belonging to the pixels below rather than an older one.
        self.shared.sync();
        dst.clear();
        dst.resize(FRAME_BYTES as usize, 0);
        let serial = self.frames();
        let base = self.shared.base();
        let bus = self.shared.bus.lock().clone();
        if let Some(bus) = bus {
            let attrs = MemAttrs::DEBUG.with_requester(*self.shared.requester.lock());
            // One read for the whole frame: this is the only bulk transfer the
            // device makes, and 21,888 single-byte dispatches through the
            // address space would dominate it.
            if bus.read_bytes(base, dst, attrs).is_err() {
                dst.fill(0);
            }
        }
        (WIDTH, HEIGHT, serial)
    }
}

/// The `PAGE2` input.
#[derive(Debug)]
struct Page2Pin {
    shared: Arc<Shared>,
    inputs: FanIn,
}

impl WireSink for Page2Pin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.shared.state.lock().page2 = high;
    }
}

/// The Macintosh video circuit.
#[derive(Debug)]
pub struct Video {
    shared: Arc<Shared>,
    screen: Arc<Screen>,
    /// The object named by `ram`, resolved at bind.
    ram_path: String,
    /// The pin, kept alive here: a net holds only a `Weak` to its sinks.
    pin: Mutex<Option<Arc<Page2Pin>>>,
}

impl Video {
    /// Build the video circuit.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `ram` is missing or of the wrong kind, or if a
    /// property this class does not know was given.
    pub fn new(props: &Props) -> Result<Video> {
        let mut r = props.reader();
        let ram_path = r.require_link("ram")?.as_str().to_string();
        r.finish()?;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(0)),
            bus: Mutex::with_rank(LockRank::LEAF, None),
            ram_len: AtomicU64::new(0),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId::ANONYMOUS),
            out: Mutex::with_rank(LockRank::LEAF, Outputs::default()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            frames: AtomicU64::new(0),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        // Publish before anybody can ask: the scheduler reads `next_event`
        // out of an atomic and must not see "nothing to do" on a device whose
        // beam is already running.
        {
            let state = *shared.state.lock();
            shared.publish(&state, false);
        }
        let screen = Arc::new(Screen {
            shared: Arc::clone(&shared),
        });
        Ok(Video {
            shared,
            screen,
            ram_path,
            pin: Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// The handle a host adapter keeps.
    #[must_use]
    pub fn screen(&self) -> Arc<Screen> {
        Arc::clone(&self.screen)
    }

    /// Point the circuit at main memory, once the machine layer has resolved
    /// it.
    ///
    /// # Errors
    ///
    /// If the region cannot be mapped into the private space, or if it is too
    /// small to hold a screen buffer where a Macintosh puts one.
    pub fn attach(&self, ram: &RegionRef, bits: u32) -> Result<()> {
        if ram.len() < ALT_OFFSET {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "`ram` holds {:#x} bytes and the alternate screen buffer starts {ALT_OFFSET:#x} \
                     below the top of memory",
                    ram.len()
                ),
            });
        }
        let space = AddressSpace::new(format!("{CLASS_NAME}.ram"), bits)
            .with_unassigned(UnassignedPolicy::OPEN_BUS);
        space.topology().map(Arc::clone(ram), 0)?;
        self.shared.ram_len.store(ram.len(), Ordering::Relaxed);
        *self.shared.bus.lock() = Some(Arc::new(space));
        Ok(())
    }

    /// Run the beam until `target` dot clocks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Whether the beam is in vertical blanking.
    #[must_use]
    pub fn in_vblank(&self) -> bool {
        State::vblank_at(self.shared.ticks.load(Ordering::Relaxed))
    }

    /// Whether the beam is in horizontal blanking.
    #[must_use]
    pub fn in_hblank(&self) -> bool {
        State::hblank_at(self.shared.ticks.load(Ordering::Relaxed))
    }

    /// Set the level the VIA is driving onto `PAGE2`, for a test with no wire
    /// graph.
    pub fn set_page2(&self, high: bool) {
        self.shared.state.lock().page2 = high;
    }
}

impl Device for Video {
    fn class(&self) -> &'static DeviceClass {
        &VIDEO_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: this device has no registers, and the wire graph
        // brings its pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let page2 = state.page2;
            *state = State::fresh(state.ticks);
            // What the VIA is driving is the VIA's and survives our reset.
            state.page2 = page2;
            self.shared.publish(&state, self.shared.hblank_wired());
        }
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Where the beam is, and which buffer it is reading. The framebuffer
        // itself is deliberately absent: it is ordinary guest memory, and the
        // `ram` object saves it (§4.5).
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_bool(state.page2)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        {
            let mut state = self.shared.state.lock();
            state.ticks = r.read_u64()?;
            state.page2 = r.read_bool()?;
            self.shared.publish(&state, self.shared.hblank_wired());
        }
        self.shared.refresh();
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.shared.out.lock();
            match port {
                VBLANK_PIN => out.vblank = Some(source),
                HBLANK_PIN => out.hblank = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: String::from(port),
                        message: String::from("the video circuit drives `vblank` and `hblank`"),
                    });
                }
            }
        }
        // Wiring `hblank` changes what has to be scheduled.
        {
            let state = *self.shared.state.lock();
            self.shared.publish(&state, self.shared.hblank_wired());
        }
        // **No refresh here.** A net has no sinks yet when its source is handed
        // out, so a level driven now goes nowhere — and having driven it, the
        // realize sweep's `announce` would then see no change and deliver
        // nothing either. Driving only from `announce`, which runs once every
        // sink is attached, is what makes the pin arrive at all (§4.3).
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != PAGE2_PIN {
            return None;
        }
        let pin = Arc::new(Page2Pin {
            shared: Arc::clone(&self.shared),
            inputs: FanIn::new(sources),
        });
        *self.pin.lock() = Some(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. The blanking interrupt has to reach the processor on the line it
    /// happens, and a capture has to see the beam where it really is.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Video::advance_to(self, tick);
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

impl Instance for Video {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bits = ctx.space().map_or(32, |s| s.bits());
        let ram = ctx.region(&self.ram_path, "").map_err(|e| Error::Config {
            at: ctx.path().to_string(),
            message: format!("`ram` has to name the memory the beam reads: {e}"),
        })?;
        *self.shared.requester.lock() = ctx.requester();
        self.attach(&ram, bits)?;
        // The pins idle wherever the beam is, which at reset is line 0.
        let state = *self.shared.state.lock();
        self.shared.publish(&state, self.shared.hblank_wired());
        Ok(())
    }
}

/// The `mac.video` device class.
pub static VIDEO_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Macintosh video circuit: 512x342 one-bit pixels read out of main memory",
    properties: &[PropertySpec {
        name: "ram",
        kind: ValueKind::Link,
        required: true,
        summary: "main memory, whose top the screen buffer hangs below",
    }],
    construct: |props| Ok(Box::new(Video::new(props)?)),
};

/// Add [`VIDEO_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&VIDEO_CLASS)
}

/// Bind [`VIDEO_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Video::new(props)?)))
}

/// What the validator should know about `mac.video`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("ram", ValueKind::Link).required())
        .port(VBLANK_PIN, PortDir::Out)
        .port(HBLANK_PIN, PortDir::Out)
        .port(PAGE2_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
