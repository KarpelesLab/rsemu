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
    pub fn learn_sources(&self, sources: &[WireId]) {
        *self.fan.lock() = Some(FanIn::new(sources));
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
    bits: Mutex<SlaveBits>,
    /// The clock line. Driven low only to stretch (§3.1.9).
    scl: OpenDrain,
    /// The data line.
    sda: OpenDrain,
    /// Every input pin handed out by [`SlaveWires::sink`].
    ///
    /// **A net holds only a weak reference to its sinks** (`core::device`),
    /// which is what stops a wire cycle leaking — so a sink nobody else holds
    /// is dropped the instant it is handed over, and the wire silently delivers
    /// to nothing. Keeping them here is the strong half of that arrangement.
    pins: Mutex<Vec<Arc<PinSink>>>,
}

impl fmt::Debug for SlaveWires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SlaveWires")
            .field("slave", &self.slave)
            .field("scl", &self.scl)
            .field("sda", &self.sda)
            .finish_non_exhaustive()
    }
}

impl LineObserver for SlaveWires {
    fn observe_line(&self, line: u32, src: WireId, level: Level) {
        let moved = match line {
            pin::SCL => self.scl.observe(src, level).map(|l| (pin::SCL, l)),
            pin::SDA => self.sda.observe(src, level).map(|l| (pin::SDA, l)),
            _ => None,
        };
        let Some((line, resolved)) = moved else {
            return;
        };

        // Decide under the lock, drive outside it.
        let emits = {
            let mut bits = self.bits.lock();
            if line == pin::SCL {
                self.on_scl(&mut bits, resolved)
            } else {
                self.on_sda(&mut bits, resolved)
            }
        };
        for emit in emits {
            match emit {
                Emit::Sda(level) => self.sda.drive(level),
                Emit::Scl(level) => self.scl.drive(level),
            }
        }
    }
}

impl SlaveWires {
    /// Pins for `slave`.
    #[must_use]
    pub fn new(slave: Arc<dyn I2cSlave>) -> SlaveWires {
        SlaveWires {
            slave,
            bits: Mutex::with_rank(WIRES_RANK, SlaveBits::default()),
            scl: OpenDrain::new(),
            sda: OpenDrain::new(),
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
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
        &self.scl
    }

    /// The data pin.
    #[must_use]
    pub fn sda(&self) -> &OpenDrain {
        &self.sda
    }

    /// A sink for one line, for [`crate::core::device::Device::sink`].
    ///
    /// The returned pin is **also kept here**, because the net that receives it
    /// holds only a weak reference; see the field's own note.
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32, sources: &[WireId]) -> Arc<dyn WireSink> {
        match line {
            pin::SCL => self.scl.learn_sources(sources),
            pin::SDA => self.sda.learn_sources(sources),
            _ => {}
        }
        let pin = Arc::new(PinSink {
            owner: Arc::clone(self) as Arc<dyn LineObserver>,
            line,
        });
        self.pins.lock().push(Arc::clone(&pin));
        pin as Arc<dyn WireSink>
    }

    /// Attach our driver for one line.
    pub fn connect(&self, line: u32, source: WireSource) {
        match line {
            pin::SCL => self.scl.connect(source),
            pin::SDA => self.sda.connect(source),
            _ => {}
        }
    }

    /// Re-drive both lines, for the realize sweep.
    pub fn announce(&self) {
        self.sda.announce();
        self.scl.announce();
    }

    /// Re-read whether the slave is still stretching, and drive SCL to match.
    ///
    /// A slave that stretches releases SCL on **its own** timeline, so its
    /// device calls this from `advance_to` once the internal work is done
    /// (§3.1.9). A part with no SCL driver never needs it.
    pub fn refresh_stretch(&self) {
        let want = if self.slave.stretching() {
            Level::Low
        } else {
            Level::High
        };
        self.scl.drive(want);
    }

    /// Reset to power-on: no transaction, both lines released.
    pub fn reset(&self) {
        *self.bits.lock() = SlaveBits::default();
        self.sda.drive(Level::High);
        self.scl.drive(Level::High);
    }

    /// SDA moved. Only interesting while SCL is high: that is a START or a STOP
    /// (§3.1.4).
    fn on_sda(&self, bits: &mut SlaveBits, level: Level) -> Vec<Emit> {
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
                self.slave.stop();
            }
            alloc::vec![Emit::Sda(Level::High)]
        }
    }

    /// SCL moved: the clock. Rising samples, falling changes (§3.1.3).
    fn on_scl(&self, bits: &mut SlaveBits, level: Level) -> Vec<Emit> {
        let was = bits.scl;
        bits.scl = level;
        if was == level {
            return Vec::new();
        }
        if level.is_high() {
            self.on_scl_rising(bits);
            Vec::new()
        } else {
            self.on_scl_falling(bits)
        }
    }

    /// A bit is valid on the rising edge; capture it.
    fn on_scl_rising(&self, bits: &mut SlaveBits) {
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
                self.slave.read_ack(ack);
            }
            _ => {}
        }
        if bits.count < 9 {
            bits.count += 1;
        }
    }

    /// The falling edge is where the data line may change (§3.1.3).
    fn on_scl_falling(&self, bits: &mut SlaveBits) -> Vec<Emit> {
        match bits.phase {
            Phase::Idle | Phase::NotUs => Vec::new(),
            _ => match bits.count {
                // Eight data bits are in and the acknowledge slot begins.
                8 => self.begin_ack(bits),
                // The acknowledge slot is over.
                9 => self.end_slot(bits),
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
    fn begin_ack(&self, bits: &mut SlaveBits) -> Vec<Emit> {
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
                        if self.slave.ten_bit_header(high) {
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
                            let ack = self.slave.address(Address::Ten(full), Direction::Read);
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
                let ack = self.slave.address(Address::seven_from_byte(byte), bits.dir);
                let next = match (ack, bits.dir) {
                    (Ack::Nack, _) => Phase::NotUs,
                    (Ack::Ack, Direction::Write) => Phase::Rx,
                    (Ack::Ack, Direction::Read) => Phase::Tx,
                };
                (ack, next)
            }
            Phase::Addr2 => {
                let full = (u16::from(bits.ten_high) << 8) | u16::from(byte);
                let ack = self.slave.address(Address::Ten(full), bits.dir);
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
                let ack = self.slave.write(byte);
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
    fn end_slot(&self, bits: &mut SlaveBits) -> Vec<Emit> {
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
            self.slave.stop();
            bits.was_addressed = false;
        }
        if matches!(bits.phase, Phase::Rx | Phase::Tx) {
            bits.was_addressed = true;
        }

        // A transmitter has to present the first bit on this same falling edge.
        if bits.phase == Phase::Tx {
            bits.shift = self.slave.read();
            out.push(Emit::Sda(tx_bit(bits.shift, 0)));
        }
        // Byte-level clock stretching: hold SCL down if the device says it
        // needs time before the next byte (§3.1.9).
        if self.slave.stretching() {
            out.push(Emit::Scl(Level::Low));
        }
        out
    }

    /// The architectural state.
    #[must_use]
    pub fn snapshot(&self) -> SlaveWiresState {
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
            scl_out: self.scl.snapshot(),
            sda_out: self.sda.snapshot(),
        }
    }

    /// Restore what [`SlaveWires::snapshot`] returned.
    pub fn restore(&self, state: SlaveWiresState) {
        {
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
        self.scl.restore(state.scl_out);
        self.sda.restore(state.sda_out);
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
    bits: Mutex<MasterBits>,
    scl: OpenDrain,
    sda: OpenDrain,
    /// Kept for the same reason [`SlaveWires::pins`] is.
    pins: Mutex<Vec<Arc<PinSink>>>,
}

impl fmt::Debug for MasterWires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("MasterWires");
        match self.bits.try_lock() {
            Some(bits) => s.field("bits", &*bits),
            None => s.field("bits", &"<in use>"),
        };
        s.field("scl", &self.scl).field("sda", &self.sda).finish()
    }
}

impl Default for MasterWires {
    fn default() -> MasterWires {
        MasterWires::new()
    }
}

impl LineObserver for MasterWires {
    fn observe_line(&self, line: u32, src: WireId, level: Level) {
        let moved = match line {
            pin::SCL => self.scl.observe(src, level).map(|l| (pin::SCL, l)),
            pin::SDA => self.sda.observe(src, level).map(|l| (pin::SDA, l)),
            _ => None,
        };
        let Some((line, resolved)) = moved else {
            return;
        };
        // Recording only — a master never drives from a wire callback, which is
        // what makes this path terminate immediately (see the module docs).
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
}

impl MasterWires {
    /// An idle master with both lines released.
    #[must_use]
    pub fn new() -> MasterWires {
        MasterWires {
            bits: Mutex::with_rank(WIRES_RANK, MasterBits::default()),
            scl: OpenDrain::new(),
            sda: OpenDrain::new(),
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        }
    }

    /// The clock pin.
    #[must_use]
    pub fn scl(&self) -> &OpenDrain {
        &self.scl
    }

    /// The data pin.
    #[must_use]
    pub fn sda(&self) -> &OpenDrain {
        &self.sda
    }

    /// A sink for one line, for [`crate::core::device::Device::sink`].
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32, sources: &[WireId]) -> Arc<dyn WireSink> {
        match line {
            pin::SCL => self.scl.learn_sources(sources),
            pin::SDA => self.sda.learn_sources(sources),
            _ => {}
        }
        let pin = Arc::new(PinSink {
            owner: Arc::clone(self) as Arc<dyn LineObserver>,
            line,
        });
        self.pins.lock().push(Arc::clone(&pin));
        pin as Arc<dyn WireSink>
    }

    /// Attach our driver for one line.
    pub fn connect(&self, line: u32, source: WireSource) {
        match line {
            pin::SCL => self.scl.connect(source),
            pin::SDA => self.sda.connect(source),
            _ => {}
        }
    }

    /// Re-drive both lines, for the realize sweep.
    pub fn announce(&self) {
        self.sda.announce();
        self.scl.announce();
    }

    /// Whether a transaction is open, as ST's `BUSY` bit defines it.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.bits.lock().busy
    }

    /// Whether an operation is in flight.
    #[must_use]
    pub fn is_working(&self) -> bool {
        self.bits.lock().op.is_some()
    }

    /// Start `op`. Ignored, reporting `false`, if one is already running.
    pub fn submit(&self, op: MasterOp) -> bool {
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
        let mut bits = self.bits.lock();
        if bits.count > 8 || !matches!(bits.op, Some(MasterOp::Read(_))) {
            return false;
        }
        bits.op = Some(MasterOp::Read(ack));
        true
    }

    /// Abandon whatever is in flight and release both lines.
    ///
    /// What a peripheral reset does, and what §3.1.8 says a master that has
    /// lost arbitration does with its drivers.
    pub fn abort(&self) {
        self.bits.lock().op = None;
        self.sda.drive(Level::High);
        self.scl.drive(Level::High);
    }

    /// Reset to power-on.
    pub fn reset(&self) {
        *self.bits.lock() = MasterBits::default();
        self.sda.drive(Level::High);
        self.scl.drive(Level::High);
    }

    /// Advance one SCL half period.
    ///
    /// Called from the owner's `advance_to`, once per half period of its clock
    /// domain. Nothing here reads a clock or schedules anything.
    pub fn tick(&self) -> MasterEvent {
        let (event, emits) = {
            let mut bits = self.bits.lock();
            match bits.op {
                None => (MasterEvent::Idle, Vec::new()),
                Some(MasterOp::Start) => self.step_start(&mut bits),
                Some(MasterOp::Write(byte)) => self.step_byte(&mut bits, Some(byte), Ack::Nack),
                Some(MasterOp::Read(ack)) => self.step_byte(&mut bits, None, ack),
                Some(MasterOp::Stop) => self.step_stop(&mut bits),
            }
        };
        for emit in emits {
            match emit {
                Emit::Sda(level) => self.sda.drive(level),
                Emit::Scl(level) => self.scl.drive(level),
            }
        }
        event
    }

    /// A START: release both lines, pull SDA low while SCL is high, pull SCL
    /// low (§3.1.4).
    fn step_start(&self, bits: &mut MasterBits) -> (MasterEvent, Vec<Emit>) {
        match bits.phase {
            0 => {
                bits.phase = 1;
                (MasterEvent::Working, alloc::vec![Emit::Sda(Level::High)])
            }
            1 => {
                bits.phase = 2;
                (MasterEvent::Working, alloc::vec![Emit::Scl(Level::High)])
            }
            2 => {
                if self.scl.net().is_low() {
                    return (MasterEvent::Stretched, Vec::new());
                }
                if self.sda.net().is_low() {
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
        if self.scl.net().is_low() && !bits.saw_high {
            // SCL never got up at all: somebody is holding it down (§3.1.9) and
            // this half period bought no progress. Had it gone high and come
            // back, the low would instead be another controller ending the high
            // period — §3.1.7's synchronisation — and the bit would be over.
            return (MasterEvent::Stretched, Vec::new());
        }
        let sda = self.sda.net();
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
    fn step_stop(&self, bits: &mut MasterBits) -> (MasterEvent, Vec<Emit>) {
        if bits.phase == 0 {
            bits.phase = 1;
            return (
                MasterEvent::Working,
                alloc::vec![Emit::Sda(Level::Low), Emit::Scl(Level::High)],
            );
        }
        if self.scl.net().is_low() {
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

    /// The architectural state.
    #[must_use]
    pub fn snapshot(&self) -> MasterWiresState {
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
            scl_out: self.scl.snapshot(),
            sda_out: self.sda.snapshot(),
        }
    }

    /// Restore what [`MasterWires::snapshot`] returned.
    pub fn restore(&self, state: MasterWiresState) {
        {
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
        self.scl.restore(state.scl_out);
        self.sda.restore(state.sda_out);
    }
}
