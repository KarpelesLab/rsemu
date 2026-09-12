//! The bit level: two open-drain nets, and the engines that ride them.
//!
//! This is the half of [`super`] that makes [`Link::Wired`](super::Link::Wired)
//! real rather than notional. Nothing here schedules anything — the master's
//! engine is stepped one half period at a time by whoever owns the clock domain
//! (`CLAUDE.md`: the scheduler owns time), and the slave's engine is told about
//! edges by whoever drives them.
//!
//! # Open drain, and why the wire model already had it
//!
//! An I²C line has no driver that pulls it high: every device either pulls it
//! low or lets go, and a pull-up resistor supplies the high (UM10204 §3.1.1).
//! The net's level is therefore the **AND** of its drivers, which is exactly
//! [`Resolve::And`], and each participant
//! resolves it from a [`FanIn`] over the net's sources — the per-driver
//! bookkeeping `ROADMAP.md` §4.3 built for wired-OR interrupts, used for the
//! other polarity.
//!
//! [`OpenDrain`] is that pin. A device drives [`Level::Low`] to pull the line
//! down and [`Level::High`] to release it; what it *reads* is the resolved net,
//! which may be low because somebody else is pulling.
//!
//! Three behaviours fall straight out of modelling it this way rather than
//! being special-cased:
//!
//! * **Acknowledge** (§3.1.6) is the receiver pulling SDA low during the ninth
//!   clock while the transmitter has released it.
//! * **Clock stretching** (§3.1.9) is a slave holding SCL low; the master
//!   releases SCL, sees the net still low, and makes no progress.
//! * **Arbitration** (§3.1.8) is a master that released SDA reading it low, and
//!   concluding that another master is driving.
//!
//! # A machine file wires each line twice
//!
//! Both lines are bidirectional for every participant, and rsemu's wire graph
//! builds one net per connected component of `wire` statements with separate
//! *drives* and *receives* flags per pin (`machine::realize`). So a board says:
//!
//! ```text
//!   wire i2c.scl -> eeprom.scl
//!   wire eeprom.scl -> i2c.scl
//!   wire i2c.sda -> eeprom.sda
//!   wire eeprom.sda -> i2c.sda
//! ```
//!
//! which is two nets, each with two drivers and two receivers — one piece of
//! copper per line, exactly as on a board. The apparent redundancy is the DSL
//! having no `<->`, not a modelling choice.
//!
//! # Re-entrancy
//!
//! Driving a pin re-enters every sink on the net, including our own. The rules
//! that keep that bounded and deadlock-free are:
//!
//! * A pin is driven with **no engine lock held**.
//! * A [`MasterWires`] never drives from a wire callback. It records levels;
//!   its outputs move only when its owner ticks it. So a master's `set_level`
//!   terminates immediately.
//! * A [`SlaveWires`] changes SDA **only on an SCL falling edge**, which is
//!   what the protocol requires anyway (§3.1.3: "the data line can only change
//!   when the clock signal on the SCL line is LOW"). It therefore cannot
//!   observe its own SDA change as a START or a STOP, both of which are defined
//!   as SDA moving *while SCL is high* (§3.1.4).
//!
//! The one exception is the STOP handler, which releases SDA unconditionally to
//! get out of a transaction abandoned mid-byte. Self-observing that release as
//! a second STOP is harmless: the engine is already idle.
//!
//! # Three engines, and why the third is the realistic one
//!
//! [`MasterWires`] and [`SlaveWires`] each model one role on a pin pair of its
//! own. A **board has neither**: an I²C peripheral drives SCL and SDA when it
//! is the controller and watches the same two nets when it is not, and the
//! interesting behaviour is in the transitions. [`ControllerWires`] is that
//! part — one pin pair, both state machines, their drive requests wired-AND
//! together the way the pad does it — and it is what lets a controller be
//! addressed by another controller, acknowledge per byte, stretch the clock as
//! a level, and lose arbitration and then answer the address that beat it.
//!
//! The two halves share nothing but the pins: a controller's target face is the
//! same [`I2cSlave`] a transactional [`I2cBus`](super::I2cBus) would route to,
//! so a wired slave cannot behave differently from a transactional one. That
//! was the objection that kept this unbuilt, and sharing the face is the answer
//! to it.

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::{Ack, Address, Direction, I2cSlave, WIRES_RANK};
use crate::core::error::Result;
use crate::core::state::{Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

/// Which line a wire is connected to.
///
/// A device declares one [`WireSink`] per line and tells this module which by
/// `line`, which is what [`crate::core::device::SinkPin::line`] carries.
pub mod pin {
    /// The serial clock.
    pub const SCL: u32 = 0;
    /// The serial data line.
    pub const SDA: u32 = 1;
    /// The name a machine description writes for the clock line.
    pub const SCL_NAME: &str = "scl";
    /// The name for the data line.
    pub const SDA_NAME: &str = "sda";
}

/// One open-drain pin on a shared net.
///
/// Holds what *we* drive and what the *net* reads, which on an open-drain line
/// are different questions and the difference is the whole protocol.
pub struct OpenDrain {
    /// Per-driver levels of the net. `None` until the machine tells us who the
    /// drivers are, which it does when it takes our sink.
    ///
    /// A leaf lock: it is never held across any other acquisition, let alone
    /// across a call into a device.
    fan: Mutex<Option<FanIn>>,
    /// Our own driver, connected at realize time.
    port: Mutex<Option<WireSource>>,
    /// What we are driving. `true` is released (high).
    driving: AtomicBool,
    /// The resolved level of the net. `true` is high.
    net: AtomicBool,
}

impl fmt::Debug for OpenDrain {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OpenDrain")
            .field("driving", &self.driving())
            .field("net", &self.net())
            .finish()
    }
}

impl Default for OpenDrain {
    fn default() -> OpenDrain {
        OpenDrain::new()
    }
}

impl OpenDrain {
    /// A released pin on a net nobody has described yet.
    ///
    /// Both the driven level and the net start high, which is the idle state of
    /// a pulled-up line. The realize sweep (§4.3) then makes every driver
    /// announce, so the [`FanIn`] agrees before anything moves.
    #[must_use]
    pub fn new() -> OpenDrain {
        OpenDrain {
            fan: Mutex::with_rank(LockRank::LEAF, None),
            port: Mutex::with_rank(LockRank::WIRE, None),
            driving: AtomicBool::new(true),
            net: AtomicBool::new(true),
        }
    }

    /// Learn who else drives this net.
    ///
    /// Called from [`crate::core::device::Device::sink`], which is handed the
    /// list; a [`FanIn`] can only be built once the sources are known.
    ///
    /// **Every slot starts released.** [`FanIn::new`] starts them low, which is
    /// the neutral level for the wired-*OR* it was built for (an interrupt line
    /// nobody is asserting). On an open-drain net low is the *asserted* level,
    /// so a fresh fan-in would read the net low until every driver had spoken —
    /// and the realize sweep announces them one at a time, so the net would pass
    /// through a low no board ever holds. Anything watching for a START, or
    /// latching `BUSY`, sees that as a transaction beginning.
    pub fn learn_sources(&self, sources: &[WireId]) {
        let fan = FanIn::new(sources);
        for src in sources {
            fan.set(*src, Level::High);
        }
        *self.fan.lock() = Some(fan);
    }

    /// Attach our own driver.
    pub fn connect(&self, source: WireSource) {
        *self.port.lock() = Some(source);
    }

    /// Whether anything has connected a driver here.
    #[must_use]
    pub fn is_connected(&self) -> bool {
        self.port.lock().is_some()
    }

    /// What we are driving. [`Level::High`] means released.
    #[must_use]
    pub fn driving(&self) -> Level {
        Level::from_bool(self.driving.load(Ordering::Relaxed))
    }

    /// The resolved level of the net: the wired-AND of every driver.
    #[must_use]
    pub fn net(&self) -> Level {
        Level::from_bool(self.net.load(Ordering::Relaxed))
    }

    /// Pull the line low, or release it.
    ///
    /// **Call this with no engine lock held**: it drives the wire, which
    /// synchronously re-enters every sink on the net, our own included.
    pub fn drive(&self, level: Level) {
        if self.driving.swap(level.as_bool(), Ordering::Relaxed) == level.as_bool() {
            // Re-driving a level we already hold must not look like an edge to
            // anything watching.
            return;
        }
        self.publish(level);
    }

    /// Re-drive whatever we hold, for the realize sweep
    /// ([`crate::core::device::Device::announce`]).
    pub fn announce(&self) {
        self.publish(self.driving());
    }

    /// Push our level onto the net.
    fn publish(&self, level: Level) {
        // Cloned out and the lock released before the call: driving re-enters
        // every sink on the net (the re-entrancy contract in `core::device`).
        let port = self.port.lock().clone();
        match port {
            Some(port) => {
                port.set(level);
            }
            None => {
                // Nothing wired. There is no net to resolve against, so what we
                // drive is what we read — which is what a pin with only a
                // pull-up on it does.
                self.net.store(level.as_bool(), Ordering::Relaxed);
            }
        }
    }

    /// Record one driver's level; report the net's level if it moved.
    ///
    /// The whole of a [`WireSink`] implementation for this pin.
    pub fn observe(&self, src: WireId, level: Level) -> Option<Level> {
        let resolved = {
            let fan = self.fan.lock();
            match fan.as_ref() {
                Some(fan) => {
                    fan.set(src, level);
                    fan.resolve(Resolve::And)
                }
                // A driver we were never told about. Treat it as the only one,
                // which is right for the single-driver case a unit test builds
                // by hand and never happens in a realized machine, because the
                // machine layer always calls `learn_sources`.
                None => level,
            }
        };
        if self.net.swap(resolved.as_bool(), Ordering::Relaxed) == resolved.as_bool() {
            return None;
        }
        Some(resolved)
    }

    /// The architectural state: what we drive, and what we last saw.
    #[must_use]
    pub fn snapshot(&self) -> (bool, bool) {
        (
            self.driving.load(Ordering::Relaxed),
            self.net.load(Ordering::Relaxed),
        )
    }

    /// Restore what [`OpenDrain::snapshot`] returned.
    ///
    /// The *net* level is derived — every other driver restores its own state
    /// and announces (`ROADMAP.md` §4.5) — so it is a starting point rather
    /// than truth, and the sweep corrects it.
    pub fn restore(&self, state: (bool, bool)) {
        self.driving.store(state.0, Ordering::Relaxed);
        self.net.store(state.1, Ordering::Relaxed);
    }
}

/// What an engine wants driven once its state lock is released.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Emit {
    Sda(Level),
    Scl(Level),
}

/// Something that owns two open-drain lines and wants to be told about them.
///
/// Private: it exists only so [`SlaveWires`] and [`MasterWires`] can share one
/// [`WireSink`] implementation, and it appears in no public signature.
trait LineObserver: Send + Sync + fmt::Debug {
    /// One driver on `line` moved to `level`.
    fn observe_line(&self, line: u32, src: WireId, level: Level);
}

/// One line of an engine, as the wire graph sees it.
struct PinSink {
    owner: Arc<dyn LineObserver>,
    line: u32,
}

impl fmt::Debug for PinSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PinSink").field("line", &self.line).finish()
    }
}

impl WireSink for PinSink {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.owner.observe_line(self.line, src, level);
    }
}

/// The bit a transmitter puts out for slot position `n`, MSB first (§3.1.5).
fn tx_bit(byte: u8, n: u8) -> Level {
    Level::from_bool(byte & (0x80 >> n.min(7)) != 0)
}

// ---------------------------------------------------------------------------
// One pad pair
// ---------------------------------------------------------------------------

/// Which internal half of a participant is asking a line for a level.
///
/// A pad has **one** driver on the board; behind it a controller that also
/// answers to an address has **two** engines. So the two requests are
/// wired-AND together before the pin moves — the same rule §3.1.1 gives the
/// net, applied one level further in, which is what the pad does on silicon.
/// [`MasterWires`] and [`SlaveWires`] use only [`half::ONE`];
/// [`ControllerWires`] uses both, and that is the whole of how one pin pair
/// carries both roles.
mod half {
    /// The only half a single-role engine has, and the controller half of one
    /// that has two.
    pub(super) const ONE: usize = 0;
    /// The target half of a [`super::ControllerWires`].
    pub(super) const TARGET: usize = 1;
    /// How many there are.
    pub(super) const COUNT: usize = 2;
}

/// The two open-drain nets one participant shares, and who is pulling them.
///
/// Private, and shared by all three engines so that a line behaves the same
/// however many state machines sit behind it.
struct Lines {
    /// The clock line.
    scl: OpenDrain,
    /// The data line.
    sda: OpenDrain,
    /// Every input pin handed out by `sink`.
    ///
    /// **A net holds only a weak reference to its sinks** (`core::device`),
    /// which is what stops a wire cycle leaking — so a sink nobody else holds
    /// is dropped the instant it is handed over, and the wire silently delivers
    /// to nothing. Keeping them here is the strong half of that arrangement.
    pins: Mutex<Vec<Arc<PinSink>>>,
    /// What each internal half asks of SCL (index 0) and SDA (index 1).
    /// `true` is released.
    wants: [[AtomicBool; half::COUNT]; 2],
}

impl fmt::Debug for Lines {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lines")
            .field("scl", &self.scl)
            .field("sda", &self.sda)
            .finish_non_exhaustive()
    }
}

impl Lines {
    /// A released pair on nets nobody has described yet.
    fn new() -> Lines {
        Lines {
            scl: OpenDrain::new(),
            sda: OpenDrain::new(),
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
            wants: [
                [AtomicBool::new(true), AtomicBool::new(true)],
                [AtomicBool::new(true), AtomicBool::new(true)],
            ],
        }
    }

    /// One line's pin, or `None` for a line number nothing here owns.
    fn pin(&self, line: u32) -> Option<&OpenDrain> {
        match line {
            pin::SCL => Some(&self.scl),
            pin::SDA => Some(&self.sda),
            _ => None,
        }
    }

    /// A sink for one line, kept here as well for the reason [`Lines::pins`]
    /// records.
    fn sink(
        &self,
        owner: Arc<dyn LineObserver>,
        line: u32,
        sources: &[WireId],
    ) -> Arc<dyn WireSink> {
        if let Some(pin) = self.pin(line) {
            pin.learn_sources(sources);
        }
        let pin = Arc::new(PinSink { owner, line });
        self.pins.lock().push(Arc::clone(&pin));
        pin as Arc<dyn WireSink>
    }

    /// Attach our driver for one line.
    fn connect(&self, line: u32, source: WireSource) {
        if let Some(pin) = self.pin(line) {
            pin.connect(source);
        }
    }

    /// Re-drive both lines, for the realize sweep.
    fn announce(&self) {
        self.sda.announce();
        self.scl.announce();
    }

    /// Record one driver's level; report the net's level if it moved.
    fn observe(&self, line: u32, src: WireId, level: Level) -> Option<Level> {
        self.pin(line)?.observe(src, level)
    }

    /// One half asks a line for a level; the pin takes the AND of both halves.
    ///
    /// **Call this with no engine lock held**: it drives the wire, which
    /// synchronously re-enters every sink on the net, our own included.
    fn drive(&self, who: usize, emit: Emit) {
        let (index, level) = match emit {
            Emit::Scl(level) => (0, level),
            Emit::Sda(level) => (1, level),
        };
        self.wants[index][who].store(level.as_bool(), Ordering::Relaxed);
        let released = self.wants[index].iter().all(|w| w.load(Ordering::Relaxed));
        let pin = if index == 0 { &self.scl } else { &self.sda };
        pin.drive(Level::from_bool(released));
    }

    /// Let go of both lines from every half: power-on, and a reset.
    fn release_all(&self) {
        for line in &self.wants {
            for want in line {
                want.store(true, Ordering::Relaxed);
            }
        }
        self.sda.drive(Level::High);
        self.scl.drive(Level::High);
    }

    /// What one half is asking of `(SCL, SDA)`; `true` is released.
    fn wants(&self, who: usize) -> (bool, bool) {
        (
            self.wants[0][who].load(Ordering::Relaxed),
            self.wants[1][who].load(Ordering::Relaxed),
        )
    }

    /// Put back what [`Lines::wants`] returned, without moving the pin.
    fn set_wants(&self, who: usize, state: (bool, bool)) {
        self.wants[0][who].store(state.0, Ordering::Relaxed);
        self.wants[1][who].store(state.1, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// The slave's bit engine
// ---------------------------------------------------------------------------

/// Where a slave is in a transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// No transaction, or one that has ended.
    Idle,
    /// A START happened; the next nine bits are the first address byte.
    Addr1,
    /// A ten-bit header matched; the next nine are the second address byte
    /// (§3.1.11).
    Addr2,
    /// Addressed, and the master is writing to us.
    Rx,
    /// Addressed, and we are transmitting.
    Tx,
    /// A transaction that went to another device. Ignore until the next START
    /// or STOP.
    NotUs,
}

/// A stable code for a phase, for the snapshot.
const fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::Idle => 0,
        Phase::Addr1 => 1,
        Phase::Addr2 => 2,
        Phase::Rx => 3,
        Phase::Tx => 4,
        Phase::NotUs => 5,
    }
}

/// The inverse of [`phase_code`]. An unknown code loads as idle rather than
/// panicking: a snapshot is untrusted input (`ROADMAP.md` §4.5).
const fn phase_from_code(code: u8) -> Phase {
    match code {
        1 => Phase::Addr1,
        2 => Phase::Addr2,
        3 => Phase::Rx,
        4 => Phase::Tx,
        5 => Phase::NotUs,
        _ => Phase::Idle,
    }
}

/// The slave's bit-level state.
#[derive(Debug)]
struct SlaveBits {
    phase: Phase,
    /// What the current nine-bit slot ends in, decided when its acknowledge is.
    next: Phase,
    /// Bits shifted in, or the byte being shifted out.
    shift: u8,
    /// Rising edges seen in the current nine-bit slot, 0 to 9.
    count: u8,
    /// The top two bits of a ten-bit header that matched, awaiting its second
    /// byte.
    ten_high: u8,
    /// The direction the current address asked for.
    dir: Direction,
    /// The ten-bit address most recently matched, so a repeated START with
    /// `1111 0XX1` can be recognised (§3.1.11: "a matching target remembers
    /// that it was addressed before").
    ten_last: Option<u16>,
    /// Whether we were addressed when the current START arrived, so a repeated
    /// START that goes elsewhere ends our transaction and one that comes back
    /// to us does not.
    was_addressed: bool,
    /// Whether the master refused the byte we just transmitted (§3.1.6, reason
    /// 5: "a controller-receiver must signal the end of the transfer").
    master_nacked: bool,
    /// Last seen levels of the two nets.
    scl: Level,
    sda: Level,
}

impl Default for SlaveBits {
    fn default() -> SlaveBits {
        SlaveBits {
            phase: Phase::Idle,
            next: Phase::Idle,
            shift: 0,
            count: 0,
            ten_high: 0,
            dir: Direction::Write,
            ten_last: None,
            was_addressed: false,
            master_nacked: false,
            scl: Level::High,
            sda: Level::High,
        }
    }
}

/// Everything a snapshot of a [`SlaveWires`] needs.
///
/// A named struct rather than a tuple because a snapshot that silently swapped
/// two of its booleans would round-trip cleanly and behave wrongly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlaveWiresState {
    /// The engine phase, as a stable code.
    pub phase: u8,
    /// The phase the current slot ends in.
    pub next: u8,
    /// The shift register.
    pub shift: u8,
    /// Rising edges seen in the current slot.
    pub count: u8,
    /// The top two bits of a matched ten-bit header.
    pub ten_high: u8,
    /// Whether the current address asked for a read.
    pub read: bool,
    /// The last ten-bit address matched.
    pub ten_last: Option<u16>,
    /// Whether we were addressed when the current START arrived.
    pub was_addressed: bool,
    /// Whether the master refused the last byte we transmitted.
    pub master_nacked: bool,
    /// The last seen SCL net level.
    pub scl: bool,
    /// The last seen SDA net level.
    pub sda: bool,
    /// The SCL pin's driven and net levels.
    pub scl_out: (bool, bool),
    /// The SDA pin's driven and net levels.
    pub sda_out: (bool, bool),
}

impl SlaveWiresState {
    /// Encode into a snapshot chunk.
    ///
    /// The codec lives here rather than in each device that embeds a
    /// [`SlaveWires`], so two devices cannot disagree about the format of a
    /// state neither of them owns.
    ///
    /// # Errors
    ///
    /// Whatever the sink reports.
    pub fn write<S: Sink + ?Sized>(self, w: &mut S) -> Result<()> {
        w.write_u8(self.phase)?;
        w.write_u8(self.next)?;
        w.write_u8(self.shift)?;
        w.write_u8(self.count)?;
        w.write_u8(self.ten_high)?;
        w.write_bool(self.read)?;
        // Both halves are always written, so both are always read: a
        // conditional encoding would desynchronise the rest of the chunk.
        w.write_bool(self.ten_last.is_some())?;
        w.write_u16(self.ten_last.unwrap_or(0))?;
        w.write_bool(self.was_addressed)?;
        w.write_bool(self.master_nacked)?;
        w.write_bool(self.scl)?;
        w.write_bool(self.sda)?;
        w.write_bool(self.scl_out.0)?;
        w.write_bool(self.scl_out.1)?;
        w.write_bool(self.sda_out.0)?;
        w.write_bool(self.sda_out.1)
    }

    /// Decode what [`SlaveWiresState::write`] wrote.
    ///
    /// # Errors
    ///
    /// [`crate::Error::State`] if the chunk ends early or holds a non-canonical
    /// bool.
    pub fn read<'a, S: Source<'a> + ?Sized>(r: &mut S) -> Result<SlaveWiresState> {
        let phase = r.read_u8()?;
        let next = r.read_u8()?;
        let shift = r.read_u8()?;
        let count = r.read_u8()?;
        let ten_high = r.read_u8()?;
        let read = r.read_bool()?;
        let has_ten = r.read_bool()?;
        let ten = r.read_u16()?;
        Ok(SlaveWiresState {
            phase,
            next,
            shift,
            count,
            ten_high,
            read,
            ten_last: has_ten.then_some(ten),
            was_addressed: r.read_bool()?,
            master_nacked: r.read_bool()?,
            scl: r.read_bool()?,
            sda: r.read_bool()?,
            scl_out: (r.read_bool()?, r.read_bool()?),
            sda_out: (r.read_bool()?, r.read_bool()?),
        })
    }
}

/// A slave's wire-level pins: the bit-banging front end, ready made.
///
/// Wrap one of these around an `Arc<dyn I2cSlave>` and the device gains `scl`
/// and `sda` as open-drain nets, with no protocol code of its own. **This is
/// what a peripheral needs in order to be driven by a GPIO controller** — or by
/// an I²C controller in [`Link::Wired`](super::Link::Wired) mode, which is
/// electrically the same thing.
///
/// # Locking
///
/// The bit state takes [`WIRES_RANK`], which sits between
/// [`LockRank::BUS`](crate::core::sync::LockRank::BUS) and
/// [`LockRank::DEVICE`](crate::core::sync::LockRank::DEVICE); that constant's
/// docs give the whole ladder. The lock is held across the call into the slave
/// on purpose — reassembling a byte and handing it over is one step — so the
/// slave's own state must rank *below* it, which `DEVICE` does. It is always
/// released before a pin is driven.
///
/// # Stretching
///
/// A device whose [`I2cSlave::stretching`] can return `true` **must** call
/// [`refresh_stretch`](SlaveWires::refresh_stretch) from its `advance_to`, or
/// it will hold SCL low forever. Nothing here reads a clock.
pub struct SlaveWires {
    slave: Arc<dyn I2cSlave>,
    core: SlaveCore,
    lines: Lines,
}

impl fmt::Debug for SlaveWires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SlaveWires")
            .field("slave", &self.slave)
            .field("lines", &self.lines)
            .finish_non_exhaustive()
    }
}

impl LineObserver for SlaveWires {
    fn observe_line(&self, line: u32, src: WireId, level: Level) {
        let Some(resolved) = self.lines.observe(line, src, level) else {
            return;
        };
        // Decide under the lock, drive outside it.
        for emit in self.core.observe(&*self.slave, line, resolved) {
            self.lines.drive(half::ONE, emit);
        }
    }
}

impl SlaveWires {
    /// Pins for `slave`.
    #[must_use]
    pub fn new(slave: Arc<dyn I2cSlave>) -> SlaveWires {
        SlaveWires {
            slave,
            core: SlaveCore::new(),
            lines: Lines::new(),
        }
    }

    /// The device these pins belong to.
    #[must_use]
    pub fn slave(&self) -> &Arc<dyn I2cSlave> {
        &self.slave
    }

    /// The clock pin.
    #[must_use]
    pub fn scl(&self) -> &OpenDrain {
        &self.lines.scl
    }

    /// The data pin.
    #[must_use]
    pub fn sda(&self) -> &OpenDrain {
        &self.lines.sda
    }

    /// A sink for one line, for [`crate::core::device::Device::sink`].
    ///
    /// The returned pin is **also kept here**, because the net that receives it
    /// holds only a weak reference; a net keeps only a weak one (`core::device`),
    /// so the strong half has to live somewhere.
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32, sources: &[WireId]) -> Arc<dyn WireSink> {
        self.lines
            .sink(Arc::clone(self) as Arc<dyn LineObserver>, line, sources)
    }

    /// Attach our driver for one line.
    pub fn connect(&self, line: u32, source: WireSource) {
        self.lines.connect(line, source);
    }

    /// Re-drive both lines, for the realize sweep.
    pub fn announce(&self) {
        self.lines.announce();
    }

    /// Re-read whether the slave is still stretching, and drive SCL to match.
    ///
    /// A slave that stretches releases SCL on **its own** timeline, so its
    /// device calls this from `advance_to` once the internal work is done
    /// (§3.1.9). A part with no SCL driver never needs it.
    pub fn refresh_stretch(&self) {
        self.lines
            .drive(half::ONE, Emit::Scl(stretch_level(&*self.slave)));
    }

    /// Reset to power-on: no transaction, both lines released.
    pub fn reset(&self) {
        self.core.reset();
        self.lines.release_all();
    }

    /// The architectural state.
    #[must_use]
    pub fn snapshot(&self) -> SlaveWiresState {
        self.core
            .snapshot(self.lines.scl.snapshot(), self.lines.sda.snapshot())
    }

    /// Restore what [`SlaveWires::snapshot`] returned.
    pub fn restore(&self, state: SlaveWiresState) {
        self.core.restore(state);
        self.lines.scl.restore(state.scl_out);
        self.lines.sda.restore(state.sda_out);
        self.lines
            .set_wants(half::ONE, (state.scl_out.0, state.sda_out.0));
    }
}

/// The level a target's SCL driver should be at (§3.1.9).
fn stretch_level(slave: &dyn I2cSlave) -> Level {
    if slave.stretching() {
        Level::Low
    } else {
        Level::High
    }
}

/// The target's nine-bit slot machine, with no pins of its own.
///
/// Split out from [`SlaveWires`] so that [`ControllerWires`] can run the
/// identical machine on a pin pair it shares with a master engine. A wired
/// target and a wired controller-as-target are therefore the same code, which
/// is the property this whole module exists to keep.
#[derive(Debug)]
struct SlaveCore {
    bits: Mutex<SlaveBits>,
}

impl SlaveCore {
    /// An idle machine.
    fn new() -> SlaveCore {
        SlaveCore {
            bits: Mutex::with_rank(WIRES_RANK, SlaveBits::default()),
        }
    }

    /// Forget everything: no transaction, nothing driven.
    fn reset(&self) {
        *self.bits.lock() = SlaveBits::default();
    }

    /// A line moved to `resolved`. Reports what to drive once the lock is gone.
    fn observe(&self, slave: &dyn I2cSlave, line: u32, resolved: Level) -> Vec<Emit> {
        let mut bits = self.bits.lock();
        if line == pin::SCL {
            self.on_scl(slave, &mut bits, resolved)
        } else {
            self.on_sda(slave, &mut bits, resolved)
        }
    }

    /// SDA moved. Only interesting while SCL is high: that is a START or a STOP
    /// (§3.1.4).
    fn on_sda(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits, level: Level) -> Vec<Emit> {
        let was = bits.sda;
        bits.sda = level;
        if bits.scl.is_low() || was == level {
            return Vec::new();
        }
        if level.is_low() {
            // High to low while SCL is high: START, or repeated START.
            bits.was_addressed = matches!(bits.phase, Phase::Rx | Phase::Tx);
            bits.phase = Phase::Addr1;
            bits.next = Phase::Idle;
            bits.shift = 0;
            bits.count = 0;
            bits.master_nacked = false;
            // Nothing is driven: between bytes SDA is already released, and
            // pulling it low here would be indistinguishable from the START we
            // just saw.
            Vec::new()
        } else {
            // Low to high while SCL is high: STOP.
            let ending = matches!(bits.phase, Phase::Rx | Phase::Tx);
            *bits = SlaveBits {
                ten_last: bits.ten_last,
                ..SlaveBits::default()
            };
            bits.scl = Level::High;
            bits.sda = Level::High;
            if ending {
                slave.stop();
            }
            alloc::vec![Emit::Sda(Level::High)]
        }
    }

    /// SCL moved: the clock. Rising samples, falling changes (§3.1.3).
    fn on_scl(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits, level: Level) -> Vec<Emit> {
        let was = bits.scl;
        bits.scl = level;
        if was == level {
            return Vec::new();
        }
        if level.is_high() {
            self.on_scl_rising(slave, bits);
            Vec::new()
        } else {
            self.on_scl_falling(slave, bits)
        }
    }

    /// A bit is valid on the rising edge; capture it.
    fn on_scl_rising(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits) {
        let sda = bits.sda;
        match bits.phase {
            Phase::Idle | Phase::NotUs => return,
            Phase::Addr1 | Phase::Addr2 | Phase::Rx if bits.count < 8 => {
                bits.shift = (bits.shift << 1) | u8::from(sda.is_high());
            }
            Phase::Tx if bits.count == 8 => {
                // The ninth clock of a byte we transmitted: the master's
                // acknowledge (§3.1.6).
                let ack = Ack::from_level(sda);
                bits.master_nacked = !ack.is_ack();
                slave.read_ack(ack);
            }
            _ => {}
        }
        if bits.count < 9 {
            bits.count += 1;
        }
    }

    /// The falling edge is where the data line may change (§3.1.3).
    fn on_scl_falling(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits) -> Vec<Emit> {
        match bits.phase {
            Phase::Idle | Phase::NotUs => Vec::new(),
            _ => match bits.count {
                // Eight data bits are in and the acknowledge slot begins.
                8 => self.begin_ack(slave, bits),
                // The acknowledge slot is over.
                9 => self.end_slot(slave, bits),
                // Mid-byte. Only a transmitter has anything to say.
                n => {
                    if bits.phase == Phase::Tx && n < 8 {
                        alloc::vec![Emit::Sda(tx_bit(bits.shift, n))]
                    } else {
                        Vec::new()
                    }
                }
            },
        }
    }

    /// Decide and drive the acknowledge for the byte just received, and record
    /// what the slot ends in.
    fn begin_ack(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits) -> Vec<Emit> {
        let byte = bits.shift;
        let (ack, next) = match bits.phase {
            Phase::Addr1 if Address::is_ten_bit_header(byte) => {
                let high = (byte >> 1) & 0b11;
                bits.dir = Direction::from_bit(byte);
                match bits.dir {
                    // The header is only half an address, so its acknowledge is
                    // for the header alone (§3.1.11, A1) and several devices may
                    // give it.
                    Direction::Write => {
                        if slave.ten_bit_header(high) {
                            bits.ten_high = high;
                            (Ack::Ack, Phase::Addr2)
                        } else {
                            (Ack::Nack, Phase::NotUs)
                        }
                    }
                    // A read header after a repeated START addresses whichever
                    // device matched the preceding write header.
                    Direction::Read => match bits.ten_last {
                        Some(full) if (full >> 8) as u8 == high => {
                            bits.ten_high = high;
                            let ack = slave.address(Address::Ten(full), Direction::Read);
                            (
                                ack,
                                if ack.is_ack() {
                                    Phase::Tx
                                } else {
                                    Phase::NotUs
                                },
                            )
                        }
                        _ => (Ack::Nack, Phase::NotUs),
                    },
                }
            }
            Phase::Addr1 => {
                bits.dir = Direction::from_bit(byte);
                let ack = slave.address(Address::seven_from_byte(byte), bits.dir);
                let next = match (ack, bits.dir) {
                    (Ack::Nack, _) => Phase::NotUs,
                    (Ack::Ack, Direction::Write) => Phase::Rx,
                    (Ack::Ack, Direction::Read) => Phase::Tx,
                };
                (ack, next)
            }
            Phase::Addr2 => {
                let full = (u16::from(bits.ten_high) << 8) | u16::from(byte);
                let ack = slave.address(Address::Ten(full), bits.dir);
                if ack.is_ack() {
                    bits.ten_last = Some(full);
                }
                let next = match (ack, bits.dir) {
                    (Ack::Nack, _) => Phase::NotUs,
                    (Ack::Ack, Direction::Write) => Phase::Rx,
                    (Ack::Ack, Direction::Read) => Phase::Tx,
                };
                (ack, next)
            }
            Phase::Rx => {
                let ack = slave.write(byte);
                // We refused the byte, so the master must stop or restart
                // (§3.1.6) and we say nothing more until it does.
                (
                    ack,
                    if ack.is_ack() {
                        Phase::Rx
                    } else {
                        Phase::NotUs
                    },
                )
            }
            // We are transmitting: the acknowledge slot belongs to the master,
            // so release the line for it (§3.1.6, "the transmitter releases the
            // SDA line during the acknowledge clock pulse").
            Phase::Tx => {
                bits.next = Phase::Tx;
                return alloc::vec![Emit::Sda(Level::High)];
            }
            Phase::Idle | Phase::NotUs => return Vec::new(),
        };
        bits.next = next;
        alloc::vec![Emit::Sda(ack.level())]
    }

    /// The nine-bit slot is complete: release the line and move on.
    fn end_slot(&self, slave: &dyn I2cSlave, bits: &mut SlaveBits) -> Vec<Emit> {
        let mut out = alloc::vec![Emit::Sda(Level::High)];
        bits.count = 0;
        bits.shift = 0;
        let was_ours = matches!(bits.phase, Phase::Rx | Phase::Tx);
        bits.phase = match bits.next {
            // A master that refused our byte has ended the read (§3.1.6, reason
            // 5), so we go quiet and wait for the STOP or repeated START.
            Phase::Tx if bits.master_nacked => Phase::NotUs,
            next => next,
        };

        // A repeated START that went to somebody else ends our transaction
        // (§3.1.11).
        if (bits.was_addressed || was_ours) && bits.phase == Phase::NotUs {
            slave.stop();
            bits.was_addressed = false;
        }
        if matches!(bits.phase, Phase::Rx | Phase::Tx) {
            bits.was_addressed = true;
        }

        // A transmitter has to present the first bit on this same falling edge.
        if bits.phase == Phase::Tx {
            bits.shift = slave.read();
            out.push(Emit::Sda(tx_bit(bits.shift, 0)));
        }
        // Byte-level clock stretching: hold SCL down if the device says it
        // needs time before the next byte (§3.1.9).
        if slave.stretching() {
            out.push(Emit::Scl(Level::Low));
        }
        out
    }

    /// The architectural state, given what the pins report.
    fn snapshot(&self, scl_out: (bool, bool), sda_out: (bool, bool)) -> SlaveWiresState {
        let bits = self.bits.lock();
        SlaveWiresState {
            phase: phase_code(bits.phase),
            next: phase_code(bits.next),
            shift: bits.shift,
            count: bits.count,
            ten_high: bits.ten_high,
            read: bits.dir == Direction::Read,
            ten_last: bits.ten_last,
            was_addressed: bits.was_addressed,
            master_nacked: bits.master_nacked,
            scl: bits.scl.is_high(),
            sda: bits.sda.is_high(),
            scl_out,
            sda_out,
        }
    }

    /// Put the bit state back. The pins are the caller's business.
    fn restore(&self, state: SlaveWiresState) {
        let mut bits = self.bits.lock();
        bits.phase = phase_from_code(state.phase);
        bits.next = phase_from_code(state.next);
        bits.shift = state.shift;
        bits.count = state.count.min(9);
        bits.ten_high = state.ten_high & 0b11;
        bits.dir = if state.read {
            Direction::Read
        } else {
            Direction::Write
        };
        bits.ten_last = state.ten_last.filter(|a| *a <= 0x3ff);
        bits.was_addressed = state.was_addressed;
        bits.master_nacked = state.master_nacked;
        bits.scl = Level::from_bool(state.scl);
        bits.sda = Level::from_bool(state.sda);
    }
}

// ---------------------------------------------------------------------------
// The master's bit engine
// ---------------------------------------------------------------------------

/// One thing a master asks the wires to do.
///
/// A whole bus event, not a bit: the engine turns each of these into the
/// [`START_HALF_PERIODS`](super::START_HALF_PERIODS),
/// [`BYTE_HALF_PERIODS`](super::BYTE_HALF_PERIODS) or
/// [`STOP_HALF_PERIODS`](super::STOP_HALF_PERIODS) ticks its transactional
/// counterpart is charged, so the two links cost the same virtual time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MasterOp {
    /// A START, or a repeated START. §3.1.4 makes them the same condition; what
    /// distinguishes them is only whether the bus was already busy.
    Start,
    /// Send eight bits, then read the receiver's acknowledge.
    Write(u8),
    /// Read eight bits, then drive this acknowledge (§3.1.6).
    Read(Ack),
    /// A STOP. Both lines end released and the bus is free.
    Stop,
}

/// What one half period of the master engine produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MasterEvent {
    /// Nothing to do; no time need be charged.
    Idle,
    /// A half period passed and the operation continues.
    Working,
    /// A half period passed waiting for SCL to be released by somebody else
    /// (§3.1.9). The operation has made no progress.
    Stretched,
    /// Another master is driving the bus (§3.1.8). The operation is abandoned
    /// and both lines are released.
    ArbitrationLost,
    /// A START condition is on the bus.
    Started,
    /// A byte went out and this came back on the ninth clock.
    Wrote(Ack),
    /// A byte came in.
    Read(u8),
    /// A STOP condition is on the bus.
    Stopped,
}

/// The master's bit-level state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MasterBits {
    op: Option<MasterOp>,
    /// Which half of the current bit, or which step of a START or STOP.
    phase: u8,
    /// Bit slots completed, 0 to 9.
    count: u8,
    /// Bits shifted in on a read.
    shift: u8,
    /// The acknowledge sampled on a write.
    ack: Ack,
    /// Whether the level we put on SDA this half period was ours to defend,
    /// which is what makes an arbitration check meaningful (§3.1.8).
    arbitrating: bool,
    /// The level we put on SDA this half period.
    driven: Level,
    /// Whether a transaction is open, as ST's `BUSY` bit defines it.
    busy: bool,
    /// Whether SCL has actually been observed high since this bit's low half
    /// began.
    ///
    /// The one bit of state that tells **clock synchronisation** (§3.1.7) from
    /// **clock stretching** (§3.1.9), which are electrically the same picture —
    /// SCL low while we are releasing it. If SCL went high and came back down,
    /// another controller ended the high period and ours ends with it; if it
    /// never went high at all, somebody is holding it and we have made no
    /// progress. Without this the two are indistinguishable and one of them has
    /// to be got wrong.
    saw_high: bool,
    /// Last seen net levels, for START and STOP detection.
    scl: Level,
    sda: Level,
}

impl Default for MasterBits {
    fn default() -> MasterBits {
        MasterBits {
            op: None,
            phase: 0,
            count: 0,
            shift: 0,
            ack: Ack::Nack,
            arbitrating: false,
            driven: Level::High,
            busy: false,
            saw_high: false,
            scl: Level::High,
            sda: Level::High,
        }
    }
}

/// Everything a snapshot of a [`MasterWires`] needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MasterWiresState {
    /// The operation in flight, as a stable code: 0 none, 1 START, 2 write,
    /// 3 read, 4 STOP.
    pub op: u8,
    /// The byte being written, or the acknowledge to drive on a read.
    pub operand: u8,
    /// The step within the operation.
    pub phase: u8,
    /// Bit slots completed.
    pub count: u8,
    /// Bits shifted in.
    pub shift: u8,
    /// Whether the sampled acknowledge was an ACK.
    pub ack: bool,
    /// Whether the level driven this half period is ours to defend.
    pub arbitrating: bool,
    /// The level driven this half period.
    pub driven: bool,
    /// Whether a transaction is open.
    pub busy: bool,
    /// Whether SCL has been seen high since this bit's low half began.
    pub saw_high: bool,
    /// The last seen SCL net level.
    pub scl: bool,
    /// The last seen SDA net level.
    pub sda: bool,
    /// The SCL pin's driven and net levels.
    pub scl_out: (bool, bool),
    /// The SDA pin's driven and net levels.
    pub sda_out: (bool, bool),
}

impl MasterWiresState {
    /// Encode into a snapshot chunk.
    ///
    /// # Errors
    ///
    /// Whatever the sink reports.
    pub fn write<S: Sink + ?Sized>(self, w: &mut S) -> Result<()> {
        w.write_u8(self.op)?;
        w.write_u8(self.operand)?;
        w.write_u8(self.phase)?;
        w.write_u8(self.count)?;
        w.write_u8(self.shift)?;
        w.write_bool(self.ack)?;
        w.write_bool(self.arbitrating)?;
        w.write_bool(self.driven)?;
        w.write_bool(self.busy)?;
        w.write_bool(self.saw_high)?;
        w.write_bool(self.scl)?;
        w.write_bool(self.sda)?;
        w.write_bool(self.scl_out.0)?;
        w.write_bool(self.scl_out.1)?;
        w.write_bool(self.sda_out.0)?;
        w.write_bool(self.sda_out.1)
    }

    /// Decode what [`MasterWiresState::write`] wrote.
    ///
    /// # Errors
    ///
    /// [`crate::Error::State`] if the chunk ends early or holds a non-canonical
    /// bool.
    pub fn read<'a, S: Source<'a> + ?Sized>(r: &mut S) -> Result<MasterWiresState> {
        Ok(MasterWiresState {
            op: r.read_u8()?,
            operand: r.read_u8()?,
            phase: r.read_u8()?,
            count: r.read_u8()?,
            shift: r.read_u8()?,
            ack: r.read_bool()?,
            arbitrating: r.read_bool()?,
            driven: r.read_bool()?,
            busy: r.read_bool()?,
            saw_high: r.read_bool()?,
            scl: r.read_bool()?,
            sda: r.read_bool()?,
            scl_out: (r.read_bool()?, r.read_bool()?),
            sda_out: (r.read_bool()?, r.read_bool()?),
        })
    }
}

/// A master's wire-level pins: the bit engine a controller drives.
///
/// The mirror of [`SlaveWires`], and the reason a memory-mapped I²C controller
/// needs no bit handling of its own: it submits a [`MasterOp`], ticks this once
/// per SCL half period out of its own clock domain, and reads back a
/// [`MasterEvent`].
///
/// # It stretches too
///
/// Between operations both this engine's SCL driver stays **low**, because
/// every operation ends with SCL low and nothing releases it until the next one
/// starts. That is not an accident of the implementation, it is the behaviour
/// ST's peripheral documents: RM0090's Figure 243 note 1 — "the EV5, EV6, EV9,
/// EV8_1 and EV8_2 events stretch SCL low until the end of the corresponding
/// software sequence". A master waiting for its driver to write a register
/// really does hold the clock down, and here that is visible on the net.
pub struct MasterWires {
    core: MasterCore,
    lines: Lines,
}

impl fmt::Debug for MasterWires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MasterWires")
            .field("core", &self.core)
            .field("lines", &self.lines)
            .finish()
    }
}

impl Default for MasterWires {
    fn default() -> MasterWires {
        MasterWires::new()
    }
}

impl LineObserver for MasterWires {
    fn observe_line(&self, line: u32, src: WireId, level: Level) {
        let Some(resolved) = self.lines.observe(line, src, level) else {
            return;
        };
        self.core.observe(line, resolved);
    }
}

impl MasterWires {
    /// An idle master with both lines released.
    #[must_use]
    pub fn new() -> MasterWires {
        MasterWires {
            core: MasterCore::new(),
            lines: Lines::new(),
        }
    }

    /// The clock pin.
    #[must_use]
    pub fn scl(&self) -> &OpenDrain {
        &self.lines.scl
    }

    /// The data pin.
    #[must_use]
    pub fn sda(&self) -> &OpenDrain {
        &self.lines.sda
    }

    /// A sink for one line, for [`crate::core::device::Device::sink`].
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32, sources: &[WireId]) -> Arc<dyn WireSink> {
        self.lines
            .sink(Arc::clone(self) as Arc<dyn LineObserver>, line, sources)
    }

    /// Attach our driver for one line.
    pub fn connect(&self, line: u32, source: WireSource) {
        self.lines.connect(line, source);
    }

    /// Re-drive both lines, for the realize sweep.
    pub fn announce(&self) {
        self.lines.announce();
        self.core.settle(&self.lines);
    }

    /// Whether a transaction is open, as ST's `BUSY` bit defines it.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.core.busy()
    }

    /// Whether an operation is in flight.
    #[must_use]
    pub fn is_working(&self) -> bool {
        self.core.is_working()
    }

    /// Start `op`. Ignored, reporting `false`, if one is already running.
    pub fn submit(&self, op: MasterOp) -> bool {
        self.core.submit(op)
    }

    /// Change the acknowledge a [`MasterOp::Read`] in flight will drive.
    ///
    /// The acknowledge is the ninth clock, so it is not decided when the read
    /// starts — it is decided by whatever the controller's `ACK` bit says when
    /// the eighth data bit has gone by. §3.1.6's "a controller-receiver must
    /// signal the end of the transfer to the target transmitter" is exactly the
    /// case that needs it: a driver clears `ACK` after reading the second-last
    /// byte, while the last one is already being clocked in.
    ///
    /// Reports whether it landed: `false` once the acknowledge bit is already on
    /// the wire, or when no read is in flight.
    pub fn set_read_ack(&self, ack: Ack) -> bool {
        self.core.set_read_ack(ack)
    }

    /// Abandon whatever is in flight and release both lines.
    ///
    /// What a peripheral reset does, and what §3.1.8 says a master that has
    /// lost arbitration does with its drivers.
    pub fn abort(&self) {
        self.core.abort();
        self.lines.release_all();
    }

    /// Reset to power-on.
    pub fn reset(&self) {
        self.core.reset();
        self.lines.release_all();
    }

    /// Advance one SCL half period.
    ///
    /// Called from the owner's `advance_to`, once per half period of its clock
    /// domain. Nothing here reads a clock or schedules anything.
    pub fn tick(&self) -> MasterEvent {
        let (event, emits) = self.core.tick(&self.lines);
        for emit in emits {
            self.lines.drive(half::ONE, emit);
        }
        event
    }

    /// The architectural state.
    #[must_use]
    pub fn snapshot(&self) -> MasterWiresState {
        self.core
            .snapshot(self.lines.scl.snapshot(), self.lines.sda.snapshot())
    }

    /// Restore what [`MasterWires::snapshot`] returned.
    pub fn restore(&self, state: MasterWiresState) {
        self.core.restore(state);
        self.lines.scl.restore(state.scl_out);
        self.lines.sda.restore(state.sda_out);
        self.lines
            .set_wants(half::ONE, (state.scl_out.0, state.sda_out.0));
    }
}

/// The controller's half-period machine, with no pins of its own.
///
/// Split out of [`MasterWires`] for the reason [`SlaveCore`] is: a
/// [`ControllerWires`] runs this exact machine on a pin pair it shares with a
/// target engine, so a controller that is also a target is not a third
/// implementation of I²C.
#[derive(Debug)]
struct MasterCore {
    bits: Mutex<MasterBits>,
}

impl MasterCore {
    /// An idle controller.
    fn new() -> MasterCore {
        MasterCore {
            bits: Mutex::with_rank(WIRES_RANK, MasterBits::default()),
        }
    }

    /// A line moved. **Recording only** — a master never drives from a wire
    /// callback, which is what makes that path terminate immediately (see the
    /// module docs).
    fn observe(&self, line: u32, resolved: Level) {
        let mut bits = self.bits.lock();
        if line == pin::SCL {
            bits.scl = resolved;
            if resolved.is_high() {
                // The clock really did get up. What separates §3.1.7's
                // synchronisation from §3.1.9's stretching, and the only place
                // it can be recorded — by the time `tick` looks, the line may
                // have been pulled down again by another controller.
                bits.saw_high = true;
            }
        } else {
            let was = bits.sda;
            bits.sda = resolved;
            // `BUSY`, as RM0090 §25.6.7 defines it: "set by hardware on
            // detection of SDA or SCL low ... cleared by hardware on detection
            // of a Stop condition".
            if bits.scl.is_high() && was.is_low() && resolved.is_high() {
                bits.busy = false;
            }
        }
        if resolved.is_low() {
            bits.busy = true;
        }
    }

    /// Whether a transaction is open, as ST's `BUSY` bit defines it.
    fn busy(&self) -> bool {
        self.bits.lock().busy
    }

    /// Re-derive what the nets say, once the realize sweep has settled them.
    ///
    /// The sweep makes every driver announce **in turn**, so a net passes
    /// through levels no board ever holds: a [`FanIn`] slot reads low until its
    /// own driver has spoken, and the first announcement on a two-driver net
    /// therefore resolves low. `BUSY` latched from one of those is a
    /// transaction that never happened — and a controller that then refuses to
    /// start, because §3.1.8 only lets it start on a free bus, never sends
    /// anything at all.
    ///
    /// Skipped while an operation is in flight, which is the case a snapshot
    /// restores into: there the levels are real and `BUSY` came back with them.
    fn settle(&self, lines: &Lines) {
        let mut bits = self.bits.lock();
        if bits.op.is_some() {
            return;
        }
        bits.scl = lines.scl.net();
        bits.sda = lines.sda.net();
        bits.busy = bits.scl.is_low() || bits.sda.is_low();
    }

    /// Whether an operation is in flight.
    fn is_working(&self) -> bool {
        self.bits.lock().op.is_some()
    }

    /// Start `op`. Ignored, reporting `false`, if one is already running.
    fn submit(&self, op: MasterOp) -> bool {
        let mut bits = self.bits.lock();
        if bits.op.is_some() {
            return false;
        }
        bits.op = Some(op);
        bits.phase = 0;
        bits.count = 0;
        bits.shift = 0;
        bits.ack = Ack::Nack;
        bits.arbitrating = false;
        true
    }

    /// Change the acknowledge a read in flight will drive.
    fn set_read_ack(&self, ack: Ack) -> bool {
        let mut bits = self.bits.lock();
        if bits.count > 8 || !matches!(bits.op, Some(MasterOp::Read(_))) {
            return false;
        }
        bits.op = Some(MasterOp::Read(ack));
        true
    }

    /// Drop whatever is in flight. The caller lets the lines go.
    fn abort(&self) {
        self.bits.lock().op = None;
    }

    /// Forget everything. The caller lets the lines go.
    fn reset(&self) {
        *self.bits.lock() = MasterBits::default();
    }

    /// Advance one SCL half period, reporting what to drive afterwards.
    fn tick(&self, lines: &Lines) -> (MasterEvent, Vec<Emit>) {
        let mut bits = self.bits.lock();
        match bits.op {
            None => (MasterEvent::Idle, Vec::new()),
            Some(MasterOp::Start) => self.step_start(lines, &mut bits),
            Some(MasterOp::Write(byte)) => self.step_byte(lines, &mut bits, Some(byte), Ack::Nack),
            Some(MasterOp::Read(ack)) => self.step_byte(lines, &mut bits, None, ack),
            Some(MasterOp::Stop) => self.step_stop(lines, &mut bits),
        }
    }

    /// A START: release both lines, pull SDA low while SCL is high, pull SCL
    /// low (§3.1.4).
    fn step_start(&self, lines: &Lines, bits: &mut MasterBits) -> (MasterEvent, Vec<Emit>) {
        match bits.phase {
            0 => {
                let (scl_ours, sda_ours) = lines.wants(half::ONE);
                if bits.busy && scl_ours && sda_ours {
                    // §3.1.8: "A controller may start a transfer only if the
                    // bus is free." A transaction is open and **we** are
                    // driving neither line, so it is somebody else's, and the
                    // gap between two of their bits is not a START opportunity
                    // — pulling SDA down in one would forge a START condition
                    // in the middle of their byte. Wait, and pay the half
                    // period for waiting.
                    //
                    // A *repeated* START is the same operation and must not be
                    // caught here: we hold SCL low between our own operations,
                    // so the **controller half's own request** is what tells
                    // the two apart. It has to be the half's request rather
                    // than the pin: on a [`ControllerWires`] the target half
                    // pulls the same SDA down to acknowledge somebody else's
                    // byte, and reading the pin would call that owning the bus.
                    return (MasterEvent::Stretched, Vec::new());
                }
                bits.phase = 1;
                (MasterEvent::Working, alloc::vec![Emit::Sda(Level::High)])
            }
            1 => {
                bits.phase = 2;
                (MasterEvent::Working, alloc::vec![Emit::Scl(Level::High)])
            }
            2 => {
                if lines.scl.net().is_low() {
                    return (MasterEvent::Stretched, Vec::new());
                }
                if lines.sda.net().is_low() {
                    // Somebody else already owns the line. §3.1.8: "A
                    // controller may start a transfer only if the bus is free."
                    return (MasterEvent::ArbitrationLost, self.give_up(bits));
                }
                bits.phase = 3;
                (MasterEvent::Working, alloc::vec![Emit::Sda(Level::Low)])
            }
            _ => {
                bits.op = None;
                bits.phase = 0;
                (MasterEvent::Started, alloc::vec![Emit::Scl(Level::Low)])
            }
        }
    }

    /// Nine bit slots: eight data bits and the acknowledge (§3.1.5, §3.1.6).
    fn step_byte(
        &self,
        lines: &Lines,
        bits: &mut MasterBits,
        byte: Option<u8>,
        ack: Ack,
    ) -> (MasterEvent, Vec<Emit>) {
        if bits.phase == 0 {
            // The low half: SCL is down, so this is when SDA may move.
            let (level, defend) = match (byte, bits.count) {
                // Writing a data bit: ours to defend.
                (Some(v), n) if n < 8 => (tx_bit(v, n), true),
                // Writing: the acknowledge slot belongs to the receiver.
                (Some(_), _) => (Level::High, false),
                // Reading a data bit: the transmitter drives it.
                (None, n) if n < 8 => (Level::High, false),
                // Reading: we drive the acknowledge, and it is ours to defend.
                (None, _) => (ack.level(), true),
            };
            bits.arbitrating = defend;
            bits.driven = level;
            bits.phase = 1;
            bits.saw_high = false;
            return (
                MasterEvent::Working,
                alloc::vec![Emit::Sda(level), Emit::Scl(Level::High)],
            );
        }

        // The high half: the bit is valid, so this is when it may be read.
        if lines.scl.net().is_low() && !bits.saw_high {
            // SCL never got up at all: somebody is holding it down (§3.1.9) and
            // this half period bought no progress. Had it gone high and come
            // back, the low would instead be another controller ending the high
            // period — §3.1.7's synchronisation — and the bit would be over.
            return (MasterEvent::Stretched, Vec::new());
        }
        let sda = lines.sda.net();
        if bits.arbitrating && bits.driven.is_high() && sda.is_low() {
            // §3.1.8: "The first time a controller tries to send a HIGH, but
            // detects that the SDA level is LOW, the controller knows that it
            // has lost the arbitration and turns off its SDA output driver."
            return (MasterEvent::ArbitrationLost, self.give_up(bits));
        }
        match (byte, bits.count) {
            (Some(_), n) if n >= 8 => bits.ack = Ack::from_level(sda),
            (None, n) if n < 8 => bits.shift = (bits.shift << 1) | u8::from(sda.is_high()),
            _ => {}
        }
        bits.count += 1;
        bits.phase = 0;
        let done = bits.count > 8;
        let out = alloc::vec![Emit::Scl(Level::Low)];
        if !done {
            return (MasterEvent::Working, out);
        }
        let event = match byte {
            Some(_) => MasterEvent::Wrote(bits.ack),
            None => MasterEvent::Read(bits.shift),
        };
        bits.op = None;
        bits.count = 0;
        (event, out)
    }

    /// A STOP: SDA rises while SCL is high, and both lines stay released
    /// (§3.1.4).
    fn step_stop(&self, lines: &Lines, bits: &mut MasterBits) -> (MasterEvent, Vec<Emit>) {
        if bits.phase == 0 {
            bits.phase = 1;
            return (
                MasterEvent::Working,
                alloc::vec![Emit::Sda(Level::Low), Emit::Scl(Level::High)],
            );
        }
        if lines.scl.net().is_low() {
            return (MasterEvent::Stretched, Vec::new());
        }
        bits.op = None;
        bits.phase = 0;
        (MasterEvent::Stopped, alloc::vec![Emit::Sda(Level::High)])
    }

    /// Drop the operation and let go of both lines.
    fn give_up(&self, bits: &mut MasterBits) -> Vec<Emit> {
        bits.op = None;
        bits.phase = 0;
        bits.count = 0;
        alloc::vec![Emit::Sda(Level::High), Emit::Scl(Level::High)]
    }

    /// The architectural state, given what the pins report.
    fn snapshot(&self, scl_out: (bool, bool), sda_out: (bool, bool)) -> MasterWiresState {
        let bits = self.bits.lock();
        let (op, operand) = match bits.op {
            None => (0, 0),
            Some(MasterOp::Start) => (1, 0),
            Some(MasterOp::Write(b)) => (2, b),
            Some(MasterOp::Read(a)) => (3, u8::from(a.is_ack())),
            Some(MasterOp::Stop) => (4, 0),
        };
        MasterWiresState {
            op,
            operand,
            phase: bits.phase,
            count: bits.count,
            shift: bits.shift,
            ack: bits.ack.is_ack(),
            arbitrating: bits.arbitrating,
            driven: bits.driven.is_high(),
            busy: bits.busy,
            saw_high: bits.saw_high,
            scl: bits.scl.is_high(),
            sda: bits.sda.is_high(),
            scl_out,
            sda_out,
        }
    }

    /// Put the bit state back. The pins are the caller's business.
    fn restore(&self, state: MasterWiresState) {
        let mut bits = self.bits.lock();
        bits.op = match state.op {
            1 => Some(MasterOp::Start),
            2 => Some(MasterOp::Write(state.operand)),
            3 => Some(MasterOp::Read(if state.operand != 0 {
                Ack::Ack
            } else {
                Ack::Nack
            })),
            4 => Some(MasterOp::Stop),
            // An unknown code loads as "nothing in flight": a snapshot is
            // untrusted input (`ROADMAP.md` §4.5).
            _ => None,
        };
        bits.phase = state.phase.min(3);
        bits.count = state.count.min(9);
        bits.shift = state.shift;
        bits.ack = if state.ack { Ack::Ack } else { Ack::Nack };
        bits.arbitrating = state.arbitrating;
        bits.driven = Level::from_bool(state.driven);
        bits.busy = state.busy;
        bits.saw_high = state.saw_high;
        bits.scl = Level::from_bool(state.scl);
        bits.sda = Level::from_bool(state.sda);
    }
}

// ---------------------------------------------------------------------------
// Both roles on one pin pair
// ---------------------------------------------------------------------------

/// Everything a snapshot of a [`ControllerWires`] needs.
///
/// Both halves are written, because both are real: a snapshot can land between
/// two SCL edges with the controller half half way through a byte it is sending
/// *and* the target half half way through the byte it is watching go past.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControllerWiresState {
    /// The controller half. Its `scl_out`/`sda_out` are the pins themselves.
    pub master: MasterWiresState,
    /// The target half. Its `scl_out`/`sda_out` are the same pins again —
    /// there is only one pair — and are ignored on load.
    pub slave: SlaveWiresState,
    /// What the controller half is asking of `(SCL, SDA)`; `true` is released.
    pub master_wants: (bool, bool),
    /// What the target half is asking of `(SCL, SDA)`; `true` is released.
    ///
    /// Not derivable from the pin: a target half stretching SCL while the
    /// controller half also holds it low is indistinguishable from one that is
    /// not, and restoring it wrong releases the clock a byte early.
    pub target_wants: (bool, bool),
}

impl ControllerWiresState {
    /// Encode into a snapshot chunk.
    ///
    /// # Errors
    ///
    /// Whatever the sink reports.
    pub fn write<S: Sink + ?Sized>(self, w: &mut S) -> Result<()> {
        self.master.write(w)?;
        self.slave.write(w)?;
        w.write_bool(self.master_wants.0)?;
        w.write_bool(self.master_wants.1)?;
        w.write_bool(self.target_wants.0)?;
        w.write_bool(self.target_wants.1)
    }

    /// Decode what [`ControllerWiresState::write`] wrote.
    ///
    /// # Errors
    ///
    /// [`crate::Error::State`] if the chunk ends early or holds a non-canonical
    /// bool.
    pub fn read<'a, S: Source<'a> + ?Sized>(r: &mut S) -> Result<ControllerWiresState> {
        let master = MasterWiresState::read(r)?;
        let slave = SlaveWiresState::read(r)?;
        Ok(ControllerWiresState {
            master,
            slave,
            master_wants: (r.read_bool()?, r.read_bool()?),
            target_wants: (r.read_bool()?, r.read_bool()?),
        })
    }
}

/// One pin pair carrying **both** roles: the wire-level engine a real I²C
/// peripheral has.
///
/// [`MasterWires`] and [`SlaveWires`] each model half a controller, and a board
/// has no such thing. An STM32 I²C block — or any other — drives SCL and SDA
/// when it is the controller and *watches the same two nets* when it is not,
/// and the transitions between those two states are the interesting part:
///
/// * **It can be addressed.** Another controller sends a START and an address
///   on the nets this one would otherwise drive, and the target half decodes it
///   out of the edges, acknowledges per byte by pulling SDA low in the ninth
///   clock, and stretches by **holding SCL down as a level** anything else on
///   the bus can see — not by answering a question the way
///   [`I2cBus::stretching`](super::I2cBus::stretching) has to.
/// * **It can lose arbitration.** Two controllers starting together is the case
///   only the wired model can express (§3.1.8): both drive, the wired-AND makes
///   the lower address win, the loser reads a low where it drove a high and
///   turns its driver off — and its target half, which has been following the
///   same byte since the START, answers if the winner was addressing *it*. That
///   is the whole of §39.4.10's "the peripheral automatically switches back to
///   slave mode", and it needs no switching: both halves were always listening.
///
/// # One pad, two internal drivers
///
/// The two halves both want to drive, so their requests are wired-AND together
/// before the pin moves. That is not a tie-break: it is the same open-drain
/// rule the net itself follows (§3.1.1), applied inside the pad. A
/// controller half holding SCL low between operations and a target half
/// stretching it are both "SCL low", and either releasing alone must not raise
/// it.
///
/// # The target face is optional and settable
///
/// [`attach_slave`](ControllerWires::attach_slave) takes the
/// [`I2cSlave`] this controller presents to the bus. It is separate from
/// construction because a peripheral's target face is usually a view of the
/// same register file the engine belongs to, so the two cannot both be built
/// first. With none attached this behaves exactly as a [`MasterWires`].
///
/// # Stretching
///
/// As [`SlaveWires`]: a face whose [`I2cSlave::stretching`] can return `true`
/// **must** have its device call
/// [`refresh_stretch`](ControllerWires::refresh_stretch) from `advance_to` and
/// after any register access that might have released the stall, or it will
/// hold SCL low forever.
pub struct ControllerWires {
    master: MasterCore,
    target: SlaveCore,
    /// The face this controller shows the bus, if it has one.
    ///
    /// Cloned out before every call, never held across one: the face's own
    /// state ranks below [`WIRES_RANK`] and the target engine's lock is held
    /// across the call into it.
    face: Mutex<Option<Arc<dyn I2cSlave>>>,
    lines: Lines,
}

impl fmt::Debug for ControllerWires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ControllerWires")
            .field("master", &self.master)
            .field("target", &self.target)
            .field("lines", &self.lines)
            .finish()
    }
}

impl Default for ControllerWires {
    fn default() -> ControllerWires {
        ControllerWires::new()
    }
}

impl LineObserver for ControllerWires {
    fn observe_line(&self, line: u32, src: WireId, level: Level) {
        let Some(resolved) = self.lines.observe(line, src, level) else {
            return;
        };
        // The controller half records and never drives from here, which is what
        // bounds the re-entry (the module docs). Its lock is taken and released
        // before the target half's: the two are the same rank, so nesting them
        // would be a ladder violation as well as a deadlock.
        self.master.observe(line, resolved);
        let Some(face) = self.face() else {
            return;
        };
        for emit in self.target.observe(&*face, line, resolved) {
            self.lines.drive(half::TARGET, emit);
        }
    }
}

impl ControllerWires {
    /// An idle controller with no target face and both lines released.
    #[must_use]
    pub fn new() -> ControllerWires {
        ControllerWires {
            master: MasterCore::new(),
            target: SlaveCore::new(),
            face: Mutex::with_rank(LockRank::LEAF, None),
            lines: Lines::new(),
        }
    }

    /// Give this controller the face it answers an address with.
    ///
    /// Call it once, from the device's constructor. Replacing a face mid-run
    /// would leave the target engine part way through somebody else's byte.
    pub fn attach_slave(&self, face: Arc<dyn I2cSlave>) {
        *self.face.lock() = Some(face);
    }

    /// The face, if one was attached.
    fn face(&self) -> Option<Arc<dyn I2cSlave>> {
        self.face.lock().clone()
    }

    /// The clock pin.
    #[must_use]
    pub fn scl(&self) -> &OpenDrain {
        &self.lines.scl
    }

    /// The data pin.
    #[must_use]
    pub fn sda(&self) -> &OpenDrain {
        &self.lines.sda
    }

    /// A sink for one line, for [`crate::core::device::Device::sink`].
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32, sources: &[WireId]) -> Arc<dyn WireSink> {
        self.lines
            .sink(Arc::clone(self) as Arc<dyn LineObserver>, line, sources)
    }

    /// Attach our driver for one line.
    pub fn connect(&self, line: u32, source: WireSource) {
        self.lines.connect(line, source);
    }

    /// Re-drive both lines, for the realize sweep.
    pub fn announce(&self) {
        self.lines.announce();
        self.master.settle(&self.lines);
    }

    /// Whether a transaction is open, as ST's `BUSY` bit defines it.
    ///
    /// **Another controller's transaction counts**, which is the point: `BUSY`
    /// is a property of the nets, and a peripheral that ignored somebody else's
    /// START would try to drive over it.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.master.busy()
    }

    /// Whether an operation of ours is in flight.
    #[must_use]
    pub fn is_working(&self) -> bool {
        self.master.is_working()
    }

    /// Start `op`. Ignored, reporting `false`, if one is already running.
    pub fn submit(&self, op: MasterOp) -> bool {
        self.master.submit(op)
    }

    /// Change the acknowledge a [`MasterOp::Read`] in flight will drive.
    pub fn set_read_ack(&self, ack: Ack) -> bool {
        self.master.set_read_ack(ack)
    }

    /// Abandon the operation in flight and let the controller half's drivers
    /// go.
    ///
    /// **The target half keeps whatever it is doing.** A peripheral that has
    /// just lost arbitration is exactly the case (§3.1.8, §39.4.10): its
    /// controller half is finished and its target half may be half way through
    /// answering the winner.
    pub fn abort(&self) {
        self.master.abort();
        self.lines.drive(half::ONE, Emit::Sda(Level::High));
        self.lines.drive(half::ONE, Emit::Scl(Level::High));
    }

    /// Reset both halves to power-on: no transaction, both lines released.
    pub fn reset(&self) {
        self.master.reset();
        self.target.reset();
        self.lines.release_all();
    }

    /// Re-read whether the face is still stretching, and drive SCL to match.
    ///
    /// Does nothing when no face is attached, so a controller that never
    /// answers an address pays one atomic load for it.
    pub fn refresh_stretch(&self) {
        let Some(face) = self.face() else {
            return;
        };
        self.lines
            .drive(half::TARGET, Emit::Scl(stretch_level(&*face)));
    }

    /// Advance the controller half one SCL half period.
    ///
    /// The target half is not ticked: it is driven by edges, and the edges this
    /// produces reach it through the net like anybody else's.
    pub fn tick(&self) -> MasterEvent {
        let (event, emits) = self.master.tick(&self.lines);
        for emit in emits {
            self.lines.drive(half::ONE, emit);
        }
        event
    }

    /// The architectural state of both halves.
    #[must_use]
    pub fn snapshot(&self) -> ControllerWiresState {
        let scl_out = self.lines.scl.snapshot();
        let sda_out = self.lines.sda.snapshot();
        ControllerWiresState {
            master: self.master.snapshot(scl_out, sda_out),
            slave: self.target.snapshot(scl_out, sda_out),
            master_wants: self.lines.wants(half::ONE),
            target_wants: self.lines.wants(half::TARGET),
        }
    }

    /// Restore what [`ControllerWires::snapshot`] returned.
    pub fn restore(&self, state: ControllerWiresState) {
        self.master.restore(state.master);
        self.target.restore(state.slave);
        self.lines.scl.restore(state.master.scl_out);
        self.lines.sda.restore(state.master.sda_out);
        self.lines.set_wants(half::ONE, state.master_wants);
        self.lines.set_wants(half::TARGET, state.target_wants);
    }
}
