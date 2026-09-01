//! A generic memory-mapped SPI controller.
//!
//! # This models no particular chip, and says so
//!
//! There is no standard SPI controller the way there is a 16550 or a 6522 —
//! every SoC invents its own register block — so this one is **rsemu's**, and
//! the register map below is defined here rather than transcribed from a
//! datasheet. It is deliberately the smallest thing that can exercise every
//! part of [`super`]: the four modes, word sizes from 1 to 32 bits, both bit
//! orders, per-device chip selects, full duplex, and *both* modelling styles.
//!
//! A real SoC's controller is a thin register wrapper over the same machinery:
//! implement its register block, drive [`SpiBus::transfer`] or the wires, and
//! reuse [`super::Shifter`] for the bit level.
//!
//! # Register map
//!
//! Sixteen bytes of 32-bit little-endian registers.
//!
//! | Offset | Name | Access | Meaning |
//! | --- | --- | --- | --- |
//! | `0x00` | `CTRL` | R/W | bit 0 `EN`; bit 1 `CPOL`; bit 2 `CPHA`; bit 3 `LSB`; bits 12:8 word bits minus one |
//! | `0x04` | `CLKDIV` | R/W | half of one SCK period, in clock-domain ticks, minus one |
//! | `0x08` | `CS` | R/W | one bit per chip select; the lowest set bit is asserted, `0` selects nothing |
//! | `0x0c` | `STATUS` | R | bit 0 `BUSY`; bit 1 `RXVALID` |
//! | `0x10` | `DATA` | R/W | write starts a transfer; read pops the received word |
//! | `0x14` | `LINES` | R/W | raw pin control, [`Link::Wired`] only: bit 0 SCK, bit 1 MOSI, bit 2 CS (as driven, so `1` is *deasserted*), bit 8 MISO (read-only) |
//!
//! `LINES` is what makes the bit-banging case reachable from a guest without a
//! separate GPIO controller: with `EN` clear, firmware can drive the three
//! output pins itself and read MISO back, one write per edge, and the slave on
//! the other end cannot tell the difference from a controller-driven transfer.
//! Real SoCs commonly expose exactly this, and it is the reason
//! `docs/buses/low-speed.md` asks for the wire model at all.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). The controller is a *lazily
//! advanced* device (`ROADMAP.md` §4.2): it holds its own tick, publishes the
//! tick its current transfer finishes on, and is caught up before any register
//! access. A transfer costs `bits × 2 × (CLKDIV + 1)` ticks **in both link
//! modes** — the honest duration either way, so a guest polling `BUSY` sees the
//! same timing whichever the machine file chose. What differs is only whether
//! the individual edges exist on wires.
//!
//! A transfer's effect lands when it *finishes*, not when `DATA` is written.
//! That is why the slave is called from
//! [`Device::advance_to`] rather than
//! from the write handler.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::{BitOrder, ChipSelect, Format, Link, MAX_CHIP_SELECTS, Mode, SpiBus, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "spi.controller";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
pub const REGISTER_BYTES: u64 = 0x18;

/// `CTRL` bit 0: the controller answers `DATA` writes.
const CTRL_EN: u32 = 1 << 0;
/// `CTRL` bit 1: clock polarity.
const CTRL_CPOL: u32 = 1 << 1;
/// `CTRL` bit 2: clock phase.
const CTRL_CPHA: u32 = 1 << 2;
/// `CTRL` bit 3: least significant bit first.
const CTRL_LSB: u32 = 1 << 3;
/// `CTRL` bits 12:8: one less than the word width.
const CTRL_BITS_SHIFT: u32 = 8;
/// The mask of `CTRL` bits 12:8, once shifted down.
const CTRL_BITS_MASK: u32 = 0x1f;
/// Everything `CTRL` defines. Writes to anything else read back as zero.
const CTRL_MASK: u32 =
    CTRL_EN | CTRL_CPOL | CTRL_CPHA | CTRL_LSB | (CTRL_BITS_MASK << CTRL_BITS_SHIFT);

/// `STATUS` bit 0: a transfer is in flight.
const STATUS_BUSY: u32 = 1 << 0;
/// `STATUS` bit 1: `DATA` holds a word nobody has read.
const STATUS_RXVALID: u32 = 1 << 1;

/// `LINES` bit 0: the level driven on SCK.
const LINES_SCK: u32 = 1 << 0;
/// `LINES` bit 1: the level driven on MOSI.
const LINES_MOSI: u32 = 1 << 1;
/// `LINES` bit 2: the level driven on the chip select. High is *deasserted*,
/// because CS is active low on the pin.
const LINES_CS: u32 = 1 << 2;
/// `LINES` bit 8: the level read back from MISO. Read-only.
const LINES_MISO: u32 = 1 << 8;
/// The `LINES` bits software may drive.
const LINES_WRITABLE: u32 = LINES_SCK | LINES_MOSI | LINES_CS;

/// The pin names a machine description wires.
pub mod pin {
    /// The serial clock the controller drives.
    pub const SCK: &str = "sck";
    /// Data out to the slaves.
    pub const MOSI: &str = "mosi";
    /// Data in. The controller's only input pin.
    pub const MISO: &str = "miso";
    /// The chip-select outputs are `cs0`, `cs1`, … up to the `chip-selects`
    /// property. This is their prefix.
    pub const CS_PREFIX: &str = "cs";
    /// The wire line number the MISO sink answers on.
    pub const MISO_LINE: u32 = 0;
}

/// "Nothing scheduled", as [`Shared::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// A generic memory-mapped SPI controller.
#[derive(Debug)]
pub struct SpiController {
    shared: Arc<Shared>,
    region: RegionRef,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// How words reach the slaves. Fixed at construction and written down in
    /// the machine file, which is the whole point (`docs/buses/low-speed.md`).
    link: Link,
    /// The bus this controller drives in [`Link::Transactional`] mode. `None`
    /// in a machine that only wires the pins up.
    bus: Option<Arc<SpiBus>>,
    /// How many chip-select outputs this controller has.
    chip_selects: u8,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick the next edge or completion falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The level last seen on the MISO input.
    miso: AtomicBool,
    /// The output pins, connected at realize time.
    pins: Mutex<Pins>,
    /// The MISO input pin, kept alive here because the net that receives it
    /// holds only a weak reference (`core::device`, §4.3's weak edge).
    miso_pin: Mutex<Option<Arc<MisoSink>>>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// The wire outputs, all optional until a machine description connects them.
#[derive(Debug, Default)]
struct Pins {
    sck: Option<WireSource>,
    mosi: Option<WireSource>,
    cs: [Option<WireSource>; MAX_CHIP_SELECTS],
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Domain ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    ctrl: u32,
    clkdiv: u32,
    cs: u32,
    lines: u32,
    /// The word `DATA` was written with, shifting out.
    tx: u32,
    /// The word received. Only meaningful with `rx_valid`.
    rx: u32,
    rx_valid: bool,
    /// Whether a transfer is in flight.
    busy: bool,
    /// The tick the in-flight transfer began on.
    started: u64,
    /// Edges emitted so far, in [`Link::Wired`]. Unused transactionally.
    edges: u32,
    /// The bits captured so far from MISO, in [`Link::Wired`].
    shift_in: u32,
}

impl Default for State {
    fn default() -> State {
        State {
            ticks: 0,
            // Eight-bit words, mode 0, MSB first, disabled.
            ctrl: (7 << CTRL_BITS_SHIFT),
            clkdiv: 0,
            cs: 0,
            // SCK low (mode 0's idle), MOSI low, CS deasserted.
            lines: LINES_CS,
            tx: 0,
            rx: 0,
            rx_valid: false,
            busy: false,
            started: 0,
            edges: 0,
            shift_in: 0,
        }
    }
}

impl State {
    /// The framing `CTRL` currently describes.
    fn format(&self) -> Format {
        Format::new(
            Mode::from_cpol_cpha(self.ctrl & CTRL_CPOL != 0, self.ctrl & CTRL_CPHA != 0),
            ((self.ctrl >> CTRL_BITS_SHIFT) & CTRL_BITS_MASK) as u8 + 1,
            if self.ctrl & CTRL_LSB != 0 {
                BitOrder::LsbFirst
            } else {
                BitOrder::MsbFirst
            },
        )
    }

    /// Half of one SCK period, in domain ticks. Never zero, or a transfer
    /// would complete on the tick it started and `next_event_tick` would stop
    /// being strictly greater than `current_tick`.
    fn half_period(&self) -> u64 {
        u64::from(self.clkdiv) + 1
    }

    /// How many wire edges one word takes: two per bit.
    fn total_edges(&self) -> u32 {
        u32::from(self.format().bits) * 2
    }

    /// The tick the in-flight transfer completes on.
    fn end_tick(&self) -> u64 {
        self.started
            .saturating_add(u64::from(self.total_edges()) * self.half_period())
    }

    /// The chip select `CS` asserts: the lowest set bit, or none.
    fn selected(&self) -> Option<ChipSelect> {
        if self.cs == 0 {
            return None;
        }
        Some(ChipSelect(self.cs.trailing_zeros() as u8))
    }

    /// `STATUS` as software reads it.
    fn status(&self) -> u32 {
        let mut s = 0;
        if self.busy {
            s |= STATUS_BUSY;
        }
        if self.rx_valid {
            s |= STATUS_RXVALID;
        }
        s
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("link", &self.link);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl SpiController {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `link` — `"transactional"` or `"wired"`. Required, and deliberately
    ///   so: `docs/buses/low-speed.md` asks for this choice to be made rather
    ///   than defaulted into.
    /// * `bus` — the name of the [`SpiBus`] to drive. Required for
    ///   `transactional`, ignored for `wired`.
    /// * `chip-selects` — how many `csN` outputs, 1 to 8. Defaults to 1.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property or a missing required one,
    /// [`Error::Config`] for a `link` this module does not know or a
    /// `chip-selects` outside 1..=8.
    pub fn new(props: &Props) -> Result<SpiController> {
        let mut r = props.reader();
        let link_name = r.require_str("link")?.to_string();
        let bus_name = r.optional_str("bus")?.map(ToString::to_string);
        let chip_selects: u64 = r.or("chip-selects", 1)?;
        r.finish()?;

        let link = Link::from_name(&link_name).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: alloc::format!(
                "`link` is `{link_name}`; it must be one of {:?} — see docs/buses/low-speed.md \
                 for which to pick",
                Link::NAMES
            ),
        })?;
        if !(1..=MAX_CHIP_SELECTS as u64).contains(&chip_selects) {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "`chip-selects` is {chip_selects}; an SPI bus routes 1 to {MAX_CHIP_SELECTS}"
                ),
            });
        }
        if link == Link::Transactional && bus_name.is_none() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "a `transactional` controller reaches its slaves through a named bus; \
                     give it `bus = \"spi0\"` and name the same bus on each slave",
                ),
            });
        }
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        Ok(SpiController::with_bus(link, bus, chip_selects as u8))
    }

    /// A controller on a bus the caller already holds.
    ///
    /// What [`SpiController::new`] ends up calling, and the way to build one
    /// without going through the named table — an embedder that owns its own
    /// [`SpiBus`], or a test that wants a bus nothing else can reach.
    #[must_use]
    pub fn with_bus(link: Link, bus: Option<Arc<SpiBus>>, chip_selects: u8) -> SpiController {
        let chip_selects = chip_selects.clamp(1, MAX_CHIP_SELECTS as u8);
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            link,
            bus,
            chip_selects,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            miso: AtomicBool::new(true),
            pins: Mutex::with_rank(LockRank::WIRE, Pins::default()),
            miso_pin: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        let port = Arc::new(ControllerPort {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("spi", REGISTER_BYTES, port as Arc<dyn MemOps>));
        SpiController { shared, region }
    }

    /// How this controller carries a word.
    #[must_use]
    pub fn link(&self) -> Link {
        self.shared.link
    }

    /// The bus it drives transactionally, if it has one.
    #[must_use]
    pub fn bus(&self) -> Option<&Arc<SpiBus>> {
        self.shared.bus.as_ref()
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Whether a transfer is in flight.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.shared.state.lock().busy
    }

    /// The word most recently received, and whether it has been read.
    #[must_use]
    pub fn rx(&self) -> (u32, bool) {
        let state = self.shared.state.lock();
        (state.rx, state.rx_valid)
    }

    /// The framing `CTRL` currently describes.
    #[must_use]
    pub fn format(&self) -> Format {
        self.shared.state.lock().format()
    }

    /// Run the controller until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// One wire action the engine wants performed once the state lock is released.
#[derive(Debug, Clone, Copy)]
enum Emit {
    Sck(Level),
    Mosi(Level),
    Cs(ChipSelect, Level),
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event
            .store(Shared::next_event(state), Ordering::Relaxed);
    }

    /// The tick the next thing happens on, or [`NO_EVENT`].
    ///
    /// Strictly greater than `state.ticks` whenever it is not [`NO_EVENT`],
    /// which the scheduler requires or catch-up stops making progress.
    fn next_event(state: &State) -> u64 {
        if !state.busy {
            return NO_EVENT;
        }
        let half = state.half_period();
        let next = state
            .started
            .saturating_add((u64::from(state.edges) + 1) * half);
        next.max(state.ticks.saturating_add(1))
    }

    /// Bring the controller up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up for this device is already running further
        // up the stack. The access still has to be answered, and answering it
        // from where the controller stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Drive one output pin. Called with no lock of ours held.
    fn emit(&self, action: Emit) {
        let port = {
            let pins = self.pins.lock();
            match action {
                Emit::Sck(_) => pins.sck.clone(),
                Emit::Mosi(_) => pins.mosi.clone(),
                Emit::Cs(cs, _) => pins.cs.get(usize::from(cs.0)).and_then(|p| p.clone()),
            }
        };
        let level = match action {
            Emit::Sck(l) | Emit::Mosi(l) => l,
            Emit::Cs(_, l) => l,
        };
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Re-drive every output from the state, for the realize sweep.
    fn announce_all(&self) {
        let (sck, mosi, cs, selected, count) = {
            let state = self.state.lock();
            (
                Level::from_bool(state.lines & LINES_SCK != 0),
                Level::from_bool(state.lines & LINES_MOSI != 0),
                Level::from_bool(state.lines & LINES_CS != 0),
                state.selected(),
                self.chip_selects,
            )
        };
        self.emit(Emit::Sck(sck));
        self.emit(Emit::Mosi(mosi));
        for i in 0..count {
            let this = ChipSelect(i);
            // A chip select is asserted low, so an unselected line is high.
            // `LINES` can also hold the line low with nothing in `CS`, which is
            // the bit-banging case.
            let level = if selected == Some(this) || (i == 0 && cs.is_low()) {
                Level::Low
            } else {
                Level::High
            };
            self.emit(Emit::Cs(this, level));
        }
    }

    /// Move the chip-select wires to match `CS`, in [`Link::Wired`].
    fn drive_chip_selects(&self, selected: Option<ChipSelect>) {
        for i in 0..self.chip_selects {
            let this = ChipSelect(i);
            let level = if selected == Some(this) {
                Level::Low
            } else {
                Level::High
            };
            self.emit(Emit::Cs(this, level));
        }
    }

    /// Point the transactional bus at whatever `CS` now says.
    fn select_transactional(&self, selected: Option<ChipSelect>) {
        if let Some(bus) = &self.bus {
            bus.select(selected);
        }
    }

    /// Start a transfer of `word`, if the controller is in a position to.
    ///
    /// Returns whether one started. Called with the state lock held; the
    /// outward part happens afterwards.
    fn begin(state: &mut State, word: u32) -> bool {
        if state.ctrl & CTRL_EN == 0 || state.busy {
            return false;
        }
        let format = state.format();
        state.tx = format.truncate(word);
        state.busy = true;
        state.started = state.ticks;
        state.edges = 0;
        state.shift_in = 0;
        true
    }

    /// The bit of `tx` that goes out for bit index `n`.
    fn tx_bit(state: &State, n: u32) -> Level {
        let format = state.format();
        let bit = match format.order {
            BitOrder::MsbFirst => (state.tx >> (u32::from(format.bits) - 1 - n)) & 1,
            BitOrder::LsbFirst => (state.tx >> n) & 1,
        };
        Level::from_bool(bit != 0)
    }

    /// Fold a sampled MISO bit into the received word.
    fn capture(state: &mut State, n: u32, level: Level) {
        let format = state.format();
        if !level.as_bool() {
            return;
        }
        match format.order {
            BitOrder::MsbFirst => state.shift_in |= 1 << (u32::from(format.bits) - 1 - n),
            BitOrder::LsbFirst => state.shift_in |= 1 << n,
        }
    }

    /// Simulate forward to `target` domain ticks.
    ///
    /// Runs with **no lock held across an outward call**: each step decides
    /// what to do under the state lock, releases it, then drives the wire or
    /// reaches the slave (`core::device`, the re-entrancy contract).
    fn advance_to(&self, target: u64) {
        loop {
            enum Step {
                Done,
                // A wired edge: drive these, in order, then loop.
                Edges(Vec<Emit>),
                // A transactional word: hand it to the bus and store the reply.
                Word(u32),
            }

            let step = {
                let mut state = self.state.lock();
                if target <= state.ticks && !state.busy {
                    state.ticks = state.ticks.max(target);
                    self.publish(&state);
                    Step::Done
                } else if !state.busy {
                    state.ticks = target;
                    self.publish(&state);
                    Step::Done
                } else {
                    let half = state.half_period();
                    match self.link {
                        Link::Transactional => {
                            let end = state.end_tick();
                            if end > target {
                                state.ticks = target;
                                self.publish(&state);
                                Step::Done
                            } else {
                                state.ticks = end;
                                let word = state.tx;
                                // `busy` stays set until the reply lands, so a
                                // re-entrant access sees a transfer in flight
                                // rather than a half-finished one.
                                Step::Word(word)
                            }
                        }
                        Link::Wired => {
                            let edge_at = state
                                .started
                                .saturating_add((u64::from(state.edges) + 1) * half);
                            if edge_at > target {
                                state.ticks = target;
                                self.publish(&state);
                                Step::Done
                            } else {
                                state.ticks = edge_at;
                                let k = state.edges;
                                let format = state.format();
                                let idle = format.mode.idle_level();
                                // Even edges move SCK away from idle, odd ones
                                // move it back.
                                let level = if k.is_multiple_of(2) {
                                    idle.inverted()
                                } else {
                                    idle
                                };
                                let bit = k / 2;
                                let mut out = Vec::new();
                                if format.mode.samples_on(level) {
                                    // Sample first: the level on MISO belongs
                                    // to the bit about to be clocked in, and it
                                    // was put there before this edge.
                                    let miso = Level::from_bool(self.miso.load(Ordering::Relaxed));
                                    Shared::capture(&mut state, bit, miso);
                                } else {
                                    // The changing edge. What goes out is the
                                    // bit the *next* sampling edge will clock
                                    // in, and `(k + 1) / 2` is its index in
                                    // both phases: with CPHA 0 the changing
                                    // edge is odd and follows a sample, with
                                    // CPHA 1 it is even and precedes one.
                                    // Writing it once is what keeps the two
                                    // phases from drifting apart.
                                    let next = k.div_ceil(2).min(u32::from(format.bits) - 1);
                                    out.push(Emit::Mosi(Shared::tx_bit(&state, next)));
                                }
                                out.push(Emit::Sck(level));
                                state.lines = (state.lines & !LINES_SCK)
                                    | if level.is_high() { LINES_SCK } else { 0 };
                                state.edges += 1;
                                if state.edges >= state.total_edges() {
                                    state.busy = false;
                                    state.rx = format.truncate(state.shift_in);
                                    state.rx_valid = true;
                                    // SCK returns to idle; MOSI is left where
                                    // the last bit put it, as real silicon does.
                                }
                                self.publish(&state);
                                Step::Edges(out)
                            }
                        }
                    }
                }
            };

            match step {
                Step::Done => return,
                Step::Edges(actions) => {
                    for action in actions {
                        self.emit(action);
                    }
                }
                Step::Word(word) => {
                    // Outward, with no lock of ours held.
                    let reply = self.bus.as_ref().map_or(u32::MAX, |bus| bus.transfer(word));
                    let mut state = self.state.lock();
                    let format = state.format();
                    state.rx = format.truncate(reply);
                    state.rx_valid = true;
                    state.busy = false;
                    self.publish(&state);
                }
            }
        }
    }

    /// Start the first bit going out, in [`Link::Wired`].
    ///
    /// With CPHA 0 the first bit has to be on MOSI *before* the first clock
    /// edge, because that edge is the one that samples it.
    fn present_first_bit(&self) {
        if self.link != Link::Wired {
            return;
        }
        let level = {
            let state = self.state.lock();
            if !state.busy {
                return;
            }
            Shared::tx_bit(&state, 0)
        };
        {
            let mut state = self.state.lock();
            state.lines =
                (state.lines & !LINES_MOSI) | if level.is_high() { LINES_MOSI } else { 0 };
        }
        self.emit(Emit::Mosi(level));
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The memory-mapped registers.
struct ControllerPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for ControllerPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ControllerPort").finish_non_exhaustive()
    }
}

impl ControllerPort {
    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => state.ctrl,
            0x04 => state.clkdiv,
            0x08 => state.cs,
            0x0c => state.status(),
            // `DATA` is the trap `MemAttrs::debug` exists for: a debugger that
            // read it would consume the guest's word and clear `RXVALID`, and
            // the guest would then read a stale one (`ROADMAP.md` §15,
            // invariant 5).
            0x10 => {
                let value = state.rx;
                if !debug {
                    state.rx_valid = false;
                }
                value
            }
            0x14 => {
                let mut lines = state.lines & LINES_WRITABLE;
                if self.shared.miso.load(Ordering::Relaxed) {
                    lines |= LINES_MISO;
                }
                lines
            }
            _ => 0,
        }
    }

    /// Write one register, reporting what has to happen once the lock is
    /// released.
    fn write_register(&self, offset: u64, value: u32) -> AfterWrite {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => {
                let was = state.format();
                state.ctrl = value & CTRL_MASK;
                if state.busy && state.format() != was {
                    // Rewriting the framing mid-word abandons it, which is what
                    // silicon does too — the shift register is reconfigured
                    // under the transfer.
                    state.busy = false;
                    state.edges = 0;
                }
                let idle = state.format().mode.idle_level();
                state.lines =
                    (state.lines & !LINES_SCK) | if idle.is_high() { LINES_SCK } else { 0 };
                self.shared.publish(&state);
                AfterWrite::Announce
            }
            0x04 => {
                state.clkdiv = value;
                self.shared.publish(&state);
                AfterWrite::Nothing
            }
            0x08 => {
                state.cs = value & ((1u32 << self.shared.chip_selects) - 1);
                let selected = state.selected();
                state.lines =
                    (state.lines & !LINES_CS) | if selected.is_some() { 0 } else { LINES_CS };
                self.shared.publish(&state);
                AfterWrite::Select(selected)
            }
            // `STATUS` is read-only.
            0x0c => AfterWrite::Nothing,
            0x10 => {
                let started = Shared::begin(&mut state, value);
                self.shared.publish(&state);
                if started {
                    AfterWrite::Started
                } else {
                    AfterWrite::Nothing
                }
            }
            0x14 => {
                state.lines = (state.lines & !LINES_WRITABLE) | (value & LINES_WRITABLE);
                let sck = Level::from_bool(state.lines & LINES_SCK != 0);
                let mosi = Level::from_bool(state.lines & LINES_MOSI != 0);
                let cs = Level::from_bool(state.lines & LINES_CS != 0);
                AfterWrite::Lines { sck, mosi, cs }
            }
            _ => AfterWrite::Nothing,
        }
    }
}

/// What a register write asks for once the state lock is released.
#[derive(Debug, Clone, Copy)]
enum AfterWrite {
    Nothing,
    /// Re-drive every output.
    Announce,
    /// The chip select moved.
    Select(Option<ChipSelect>),
    /// A transfer began.
    Started,
    /// Software drove the pins itself.
    Lines {
        sck: Level,
        mosi: Level,
        cs: Level,
    },
}

impl MemOps for ControllerPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let value = self.read_register(offset, attrs.debug);
        dst.copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would start a transfer or move a chip select,
            // neither of which the core can make harmless.
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        match self.write_register(offset, value) {
            AfterWrite::Nothing => {}
            AfterWrite::Announce => self.shared.announce_all(),
            AfterWrite::Select(selected) => match self.shared.link {
                Link::Transactional => self.shared.select_transactional(selected),
                Link::Wired => self.shared.drive_chip_selects(selected),
            },
            AfterWrite::Started => self.shared.present_first_bit(),
            AfterWrite::Lines { sck, mosi, cs } => {
                // Order matters: data and chip select settle before the clock
                // edge that samples them, which is what firmware bit-banging
                // one register at a time relies on.
                self.shared.emit(Emit::Mosi(mosi));
                let selected = self.shared.state.lock().selected();
                match (self.shared.link, selected) {
                    (Link::Wired, _) => {
                        // `LINES` drives chip select 0 directly; higher ones
                        // stay where `CS` left them.
                        self.shared.emit(Emit::Cs(ChipSelect(0), cs));
                    }
                    (Link::Transactional, _) => {
                        // Nothing to drive: a transactional link has no wires.
                        // The register still reads back, so firmware that pokes
                        // it sees its own writes and the machine file's choice
                        // is visible rather than silently ignored.
                    }
                }
                self.shared.emit(Emit::Sck(sck));
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The MISO input
// ---------------------------------------------------------------------------

/// The controller's MISO input pin.
struct MisoSink {
    shared: Arc<Shared>,
}

impl fmt::Debug for MisoSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MisoSink").finish_non_exhaustive()
    }
}

impl WireSink for MisoSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.shared.miso.store(level.as_bool(), Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for SpiController {
    fn class(&self) -> &'static DeviceClass {
        &SPI_CONTROLLER_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements connect the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let ticks = state.ticks;
            *state = State {
                ticks,
                ..State::default()
            };
            self.shared.publish(&state);
        }
        self.shared.miso.store(true, Ordering::Relaxed);
        self.shared.select_transactional(None);
        self.shared.announce_all();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u32(state.ctrl)?;
        w.write_u32(state.clkdiv)?;
        w.write_u32(state.cs)?;
        w.write_u32(state.lines)?;
        w.write_u32(state.tx)?;
        w.write_u32(state.rx)?;
        w.write_bool(state.rx_valid)?;
        w.write_bool(state.busy)?;
        w.write_u64(state.started)?;
        w.write_u32(state.edges)?;
        w.write_u32(state.shift_in)
        // `miso` is not saved: it is the level *another* device is driving, and
        // that device restores its own state and drives it again
        // (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            ticks: r.read_u64()?,
            ctrl: r.read_u32()?,
            clkdiv: r.read_u32()?,
            cs: r.read_u32()?,
            lines: r.read_u32()?,
            tx: r.read_u32()?,
            rx: r.read_u32()?,
            rx_valid: r.read_bool()?,
            busy: r.read_bool()?,
            started: r.read_u64()?,
            edges: r.read_u32()?,
            shift_in: r.read_u32()?,
        };
        {
            let mut slot = self.shared.state.lock();
            *slot = state;
            self.shared.publish(&slot);
        }
        self.shared.select_transactional(state.selected());
        self.shared.announce_all();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        if port != pin::MISO {
            return None;
        }
        let pin = Arc::new(MisoSink {
            shared: Arc::clone(&self.shared),
        });
        // Kept, because a net refers to its sinks weakly.
        *self.shared.miso_pin.lock() = Some(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin as Arc<dyn WireSink>,
            line: pin::MISO_LINE,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let mut pins = self.shared.pins.lock();
        match port {
            pin::SCK => pins.sck = Some(source),
            pin::MOSI => pins.mosi = Some(source),
            _ => {
                let index = port
                    .strip_prefix(pin::CS_PREFIX)
                    .and_then(|n| n.parse::<usize>().ok())
                    .filter(|n| *n < usize::from(self.shared.chip_selects));
                match index {
                    Some(index) => pins.cs[index] = Some(source),
                    None => {
                        return Err(Error::Config {
                            at: String::from(port),
                            message: alloc::format!(
                                "an SPI controller drives `{}`, `{}` and `{}0`..`{}{}`",
                                pin::SCK,
                                pin::MOSI,
                                pin::CS_PREFIX,
                                pin::CS_PREFIX,
                                self.shared.chip_selects - 1
                            ),
                        });
                    }
                }
            }
        }
        drop(pins);
        self.shared.announce_all();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.announce_all();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A transfer takes real time, a guest polls `BUSY` to find out when
    /// it is done, and the answer has to be the one at the cycle of the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        SpiController::advance_to(self, tick);
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

impl Instance for SpiController {}

/// The `spi.controller` device class.
pub static SPI_CONTROLLER_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a generic memory-mapped SPI controller: four modes, 1-32 bit words, \
              eight chip selects, transactional or wired",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "how words reach the slaves: `transactional` or `wired`",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus this controller drives, for `transactional`",
        },
        PropertySpec {
            name: "chip-selects",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many `csN` outputs, 1 to 8 (default 1)",
        },
    ],
    construct: |props| Ok(Box::new(SpiController::new(props)?)),
};

/// Add [`SPI_CONTROLLER_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SPI_CONTROLLER_CLASS)
}

/// Bind [`SPI_CONTROLLER_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(SpiController::new(props)?)))
}

/// What the validator should know about `spi.controller`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("link", ValueKind::Str)
                .required()
                .values(Link::NAMES),
        )
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("chip-selects", ValueKind::Uint).range(1, MAX_CHIP_SELECTS as u64))
        .port(pin::SCK, PortDir::Out)
        .port(pin::MOSI, PortDir::Out)
        .port(pin::MISO, PortDir::In)
        .region("")
        .region("regs");
    for i in 0..MAX_CHIP_SELECTS {
        schema = schema.port(alloc::format!("{}{i}", pin::CS_PREFIX), PortDir::Out);
    }
    schema
}
