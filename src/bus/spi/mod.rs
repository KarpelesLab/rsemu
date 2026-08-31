//! SPI: the Serial Peripheral Interface, both ways round.
//!
//! # There is no SPI standard
//!
//! `docs/buses/low-speed.md` says it plainly: SPI has no formal specification —
//! Motorola's original application note plus each peripheral's datasheet is all
//! there is, and *in practice the device datasheet is the specification*. So
//! this module fixes only what every SPI link agrees on — a clock, two data
//! lines, a chip select, and the fact that both directions move together — and
//! makes everything a datasheet can disagree about ([`Mode`], word size,
//! [`BitOrder`]) a per-device declaration in [`Format`].
//!
//! # The decision this module exists to make explicit
//!
//! `docs/buses/low-speed.md` again: these are *timing protocols on wires*, so
//! they model naturally onto [`crate::core::wire`] plus a clock domain — and
//! most emulators cheat and model them transactionally instead. That is faster
//! and usually fine, but **some guest firmware bit-bangs the lines directly
//! through GPIO and will notice**. So rsemu does both, the choice is a
//! machine-description property rather than an accident, and a machine file
//! says which one it is using.
//!
//! ```text
//!                     ┌──────────── Link::Transactional ────────────┐
//!   controller ───────┤  bus.transfer(cs, word)  ── one call, one    ├──► slave
//!                     │  word, no wires toggled, no clock consumed  │
//!                     └─────────────────────────────────────────────┘
//!
//!                     ┌──────────── Link::Wired ────────────────────┐
//!   controller ───────┤  drives SCK/MOSI/CS as real wires, one edge ├──► slave
//!    (or a GPIO       │  per half bit period, on the scheduler      │    pins
//!     controller,     │  ─ the slave's own Shifter reassembles the  │
//!     or the guest    │  word and calls exactly the same method     │
//!     toggling pins)  └─────────────────────────────────────────────┘
//! ```
//!
//! **The peripheral is written once.** A device model implements [`SpiSlave`],
//! which is word-level, and gets the bit-level front end for free: [`Shifter`]
//! turns SCK edges into [`SpiSlave::transfer`] calls, and [`SlavePins`] wraps a
//! `Shifter` in [`crate::core::wire::WireSink`]s so the peripheral's `sck`,
//! `mosi` and `cs` pins can be driven by *anything* — an SPI controller in
//! [`Link::Wired`] mode, a GPIO controller, or a test. That is what makes the
//! bit-banging guest work without a second device model to keep in step with
//! the first.
//!
//! # Full duplex is not optional
//!
//! MOSI and MISO shift simultaneously: the byte a controller receives during a
//! transfer is the one the slave had loaded *before* that transfer started, not
//! a reply to it. Transactional models routinely get this wrong by making the
//! call look like a request/response. [`SpiSlave::transfer`] takes the outgoing
//! word and returns the incoming one in the same call precisely so that a
//! device model cannot express the wrong thing: whatever it returns is what was
//! already in its shift register.
//!
//! # Finding each other
//!
//! A controller and its slaves are separate objects in a machine description and
//! there is no `core::bus` yet, so they meet through [`buses`], a named
//! rendezvous table exactly like [`crate::host::chardev::ports`]. Both ends name
//! the same bus (`bus = "spi0"`), the slave also names its chip select
//! (`cs = 0`), and whichever is constructed first creates the [`SpiBus`].
//!
//! # Sources
//!
//! No emulator was consulted (`ROADMAP.md` §1). The mode numbering, the
//! CPOL/CPHA table and the meaning of "leading"/"trailing" edge are the
//! universal conventions restated in every SPI peripheral datasheet; the
//! concrete timing this module was checked against is the Sitronix **ST7272A**
//! datasheet v0.5, §7.1 ("3-wire Serial Interface") and §9.3.3 ("System Bus
//! Timing for 3-Wire SPI Interface"), which is the first device
//! ([`crate::dev::sitronix`]) hung off it.

pub mod controller;

#[cfg(test)]
mod tests;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::sync::{AtomicBool, AtomicU32, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, WireId, WireSink, WireSource};

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

/// Which chip select a slave answers to.
///
/// A newtype rather than a bare integer because a controller with eight chip
/// selects and a slave that thinks it is on `cs 0` is the classic SPI wiring
/// bug, and it reads identically to a correct one when both are `u8`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[repr(transparent)]
pub struct ChipSelect(pub u8);

impl fmt::Display for ChipSelect {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "cs{}", self.0)
    }
}

/// The four SPI modes, as every datasheet numbers them.
///
/// | Mode | CPOL | CPHA | Data is sampled on | SCK idles |
/// | --- | --- | --- | --- | --- |
/// | 0 | 0 | 0 | the rising (leading) edge | low |
/// | 1 | 0 | 1 | the falling (trailing) edge | low |
/// | 2 | 1 | 0 | the falling (leading) edge | high |
/// | 3 | 1 | 1 | the rising (trailing) edge | high |
///
/// A real `enum` rather than the extensible-newtype pattern (`CLAUDE.md`):
/// there are exactly four, there will never be a fifth, and exhaustiveness is
/// what makes the shifter's edge logic checkable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Mode {
    /// CPOL 0, CPHA 0. The most common, and what the ST7272A wants.
    #[default]
    Mode0,
    /// CPOL 0, CPHA 1.
    Mode1,
    /// CPOL 1, CPHA 0.
    Mode2,
    /// CPOL 1, CPHA 1.
    Mode3,
}

impl Mode {
    /// The mode numbered `n`, or `None` above 3.
    #[must_use]
    pub const fn from_number(n: u8) -> Option<Mode> {
        match n {
            0 => Some(Mode::Mode0),
            1 => Some(Mode::Mode1),
            2 => Some(Mode::Mode2),
            3 => Some(Mode::Mode3),
            _ => None,
        }
    }

    /// The mode's number, 0 to 3.
    #[must_use]
    pub const fn number(self) -> u8 {
        match self {
            Mode::Mode0 => 0,
            Mode::Mode1 => 1,
            Mode::Mode2 => 2,
            Mode::Mode3 => 3,
        }
    }

    /// The mode with this clock polarity and phase.
    #[must_use]
    pub const fn from_cpol_cpha(cpol: bool, cpha: bool) -> Mode {
        match (cpol, cpha) {
            (false, false) => Mode::Mode0,
            (false, true) => Mode::Mode1,
            (true, false) => Mode::Mode2,
            (true, true) => Mode::Mode3,
        }
    }

    /// Clock polarity: the level SCK idles at between words.
    #[must_use]
    pub const fn cpol(self) -> bool {
        matches!(self, Mode::Mode2 | Mode::Mode3)
    }

    /// Clock phase: `false` samples on the leading edge of each bit, `true` on
    /// the trailing one.
    #[must_use]
    pub const fn cpha(self) -> bool {
        matches!(self, Mode::Mode1 | Mode::Mode3)
    }

    /// The level SCK sits at while no word is being clocked.
    #[must_use]
    pub const fn idle_level(self) -> Level {
        if self.cpol() { Level::High } else { Level::Low }
    }

    /// Whether a transition to `to` is the edge this mode samples on.
    ///
    /// With CPHA 0 the sampling edge is the one that moves SCK *away* from
    /// idle; with CPHA 1 it is the one that moves it back. Written this way
    /// rather than as four cases so that the two polarities cannot drift apart.
    #[must_use]
    pub const fn samples_on(self, to: Level) -> bool {
        let away_from_idle = to.is_high() != self.cpol();
        away_from_idle != self.cpha()
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "mode{}", self.number())
    }
}

/// Which end of the word goes down the wire first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum BitOrder {
    /// Most significant bit first. What almost every peripheral wants, and what
    /// the ST7272A's `R/W, A6..A0, D7..D0` framing is.
    #[default]
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
}

impl fmt::Display for BitOrder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            BitOrder::MsbFirst => "msb-first",
            BitOrder::LsbFirst => "lsb-first",
        })
    }
}

/// The narrowest word an SPI link can carry.
pub const MIN_WORD_BITS: u8 = 1;

/// The widest. A `u32` carries the word, and 32 bits covers every framing this
/// module has met — the ST7272A's is 16 (datasheet §7.1: "Each serial command
/// consists of 16 bits of data").
pub const MAX_WORD_BITS: u8 = 32;

/// Everything a datasheet gets to decide about how bits are framed.
///
/// Carried by value: it is four bytes of configuration and copying it is
/// cheaper than reaching through an `Arc` for it on every word.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Format {
    /// Clock polarity and phase.
    pub mode: Mode,
    /// How many bits are in one word, 1 to 32.
    pub bits: u8,
    /// Which bit leads.
    pub order: BitOrder,
}

impl Format {
    /// Mode 0, eight bits, MSB first — the default nearly every part uses.
    pub const DEFAULT: Format = Format {
        mode: Mode::Mode0,
        bits: 8,
        order: BitOrder::MsbFirst,
    };

    /// A format, with `bits` clamped into [`MIN_WORD_BITS`]..=[`MAX_WORD_BITS`].
    ///
    /// Clamped rather than refused because this is also how a guest's register
    /// write reaches the model, and a nonsense field in a control register is
    /// the guest's bug to see in its own timing, not a reason to fault the
    /// access.
    #[must_use]
    pub const fn new(mode: Mode, bits: u8, order: BitOrder) -> Format {
        let bits = if bits < MIN_WORD_BITS {
            MIN_WORD_BITS
        } else if bits > MAX_WORD_BITS {
            MAX_WORD_BITS
        } else {
            bits
        };
        Format { mode, bits, order }
    }

    /// The mask covering the significant bits of a word in this format.
    #[must_use]
    pub const fn mask(self) -> u32 {
        if self.bits >= 32 {
            u32::MAX
        } else {
            (1u32 << self.bits) - 1
        }
    }

    /// Drop everything above the word width.
    #[must_use]
    pub const fn truncate(self, word: u32) -> u32 {
        word & self.mask()
    }
}

impl Default for Format {
    fn default() -> Format {
        Format::DEFAULT
    }
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}-bit {}", self.mode, self.bits, self.order)
    }
}

// ---------------------------------------------------------------------------
// The device seam
// ---------------------------------------------------------------------------

/// A peripheral on an SPI bus, as the bus sees it.
///
/// Word-level on purpose: a datasheet describes a command as a *word* — the
/// ST7272A's is `R/W A6..A0 D7..D0` — and a device model that had to reassemble
/// it from bits would be writing the shifter again, differently, once per
/// device. [`Shifter`] does that once and calls in here.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0).
///
/// # The contract
///
/// * [`select`](SpiSlave::select) is called on every change of the chip select,
///   with `true` meaning *asserted* — the logical sense, not the electrical
///   one, since CS is active low on the wire.
/// * [`transfer`](SpiSlave::transfer) is called once per complete word, and
///   only while selected. It is **full duplex**: the returned word is what was
///   already in the slave's shift register when the transfer began.
/// * [`format`](SpiSlave::format) says how the device frames a word. A
///   controller may ignore it and use its own; a mismatch is a machine
///   description bug, and [`SpiBus::check_format`] is how a machine finds out.
pub trait SpiSlave: Send + Sync + fmt::Debug {
    /// How this device frames a word.
    fn format(&self) -> Format;

    /// The chip select changed. `true` is asserted.
    ///
    /// Deassertion is what most parts use to commit a command, so a model that
    /// ignores this is usually wrong. The ST7272A's is explicit — datasheet
    /// §7.1(b): "Command loading operation starts from the falling edge of CS
    /// and is completed at the next rising edge of CS."
    fn select(&self, selected: bool);

    /// Exchange one word. `mosi` goes out, the return value comes in.
    ///
    /// Only called while selected.
    fn transfer(&self, mosi: u32) -> u32;

    /// What this slave would put on MISO if a word started now, without
    /// starting one.
    ///
    /// For a debugger and for [`SpiBus::peek`]. Defaults to all-ones, which is
    /// what an undriven pulled-up MISO reads as. **Must have no side effects**
    /// — this is the [`crate::core::space::MemAttrs::debug`] rule applied to a
    /// bus rather than to a register block.
    fn peek(&self) -> u32 {
        u32::MAX
    }

    /// After how many bits this part turns the data line around, if it does.
    ///
    /// Plenty of parts put a command in the first half of a word and answer in
    /// the second half of *the same* word rather than the next one — the
    /// ST7272A's read frame is `R A6..A0 D7..D0` with the master driving the
    /// first eight bits and the panel the last eight (datasheet §7.1, "Read
    /// Mode"). A word-level seam that could not express that would force every
    /// such device to invent its own bit handling, which is exactly what
    /// [`Shifter`] exists to prevent.
    ///
    /// `None`, the default, is an ordinary full-duplex part whose outgoing word
    /// is fixed before the transfer begins.
    ///
    /// **MSB-first only.** A turnaround part numbers its frame from the most
    /// significant bit by construction; [`partial`](SpiSlave::partial) is not
    /// called for an LSB-first format.
    fn turnaround(&self) -> Option<u8> {
        None
    }

    /// The first `bits` of a word have arrived; load the rest of the outgoing
    /// one if this part answers mid-word.
    ///
    /// `received` holds those bits right-aligned. Return the word whose low
    /// `format().bits - bits` bits should be driven from here on, or `None` to
    /// leave the outgoing word alone. Called on every sampling edge, so an
    /// implementation looks at `bits` first and usually says nothing.
    ///
    /// **Must have no side effect a repeat would change**: the transactional
    /// and wired links call it at different moments, and the two must agree.
    fn partial(&self, bits: u8, received: u32) -> Option<u32> {
        let _ = (bits, received);
        None
    }
}

/// One word through `slave`, honouring a mid-word turnaround.
///
/// The single place [`SpiSlave::turnaround`] is interpreted for a link that
/// does not clock individual bits, so that [`SpiBus::transfer`] and
/// [`SlavePins`] cannot drift apart about what a read frame returns.
pub fn exchange(slave: &dyn SpiSlave, mosi: u32) -> u32 {
    let format = slave.format();
    let turn = match (slave.turnaround(), format.order) {
        (Some(n), BitOrder::MsbFirst) if n > 0 && n < format.bits => Some(n),
        _ => None,
    };
    // Before the transfer, because the answer is decoded from the bits that
    // arrived first — which on the wire is exactly when it happens.
    let spliced = turn.and_then(|n| {
        let remaining = format.bits - n;
        slave
            .partial(n, mosi >> remaining)
            .map(|word| (remaining, word))
    });
    let presented = format.truncate(slave.transfer(mosi));
    match spliced {
        Some((remaining, word)) => {
            let mask = if remaining >= 32 {
                u32::MAX
            } else {
                (1u32 << remaining) - 1
            };
            (presented & !mask) | (word & mask)
        }
        None => presented,
    }
}

// ---------------------------------------------------------------------------
// Modelling mode
// ---------------------------------------------------------------------------

/// How a controller carries a word to its slaves.
///
/// **The point of this type is that it is written down.** A machine file that
/// says `link = "transactional"` has made a choice; one that never mentions it
/// has inherited a default, which is exactly the accident
/// `docs/buses/low-speed.md` asks us to avoid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Link {
    /// One call per word through [`SpiBus`]. No wires move.
    ///
    /// The fast, ordinary model, and the right one when nothing outside the
    /// controller can see the lines. **The transfer still costs its real time**
    /// — the controller charges the scheduler `bits × ticks-per-bit` either way
    /// — so a guest that polls a busy flag sees the same timing under both.
    /// What differs is only whether the individual edges exist.
    #[default]
    Transactional,

    /// SCK, MOSI and CS are driven as real wires, one edge per half bit period,
    /// paced by the scheduler.
    ///
    /// Slower, and necessary when anything else is watching the lines: a logic
    /// analyser model, a second slave sharing MOSI, or — the case this exists
    /// for — guest firmware that also bit-bangs the same pins through GPIO and
    /// would notice a controller that teleported a byte.
    Wired,
}

impl Link {
    /// Parse the spelling a machine description uses.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Link> {
        match name {
            "transactional" => Some(Link::Transactional),
            "wired" => Some(Link::Wired),
            _ => None,
        }
    }

    /// The spelling a machine description uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Link::Transactional => "transactional",
            Link::Wired => "wired",
        }
    }

    /// Every spelling, for a validator's enumeration.
    pub const NAMES: &'static [&'static str] = &["transactional", "wired"];
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// The fabric
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Where this module sits in the lock ladder
// ---------------------------------------------------------------------------

/// The rank an [`SpiBus`]'s own routing state takes.
///
/// **Not [`LockRank::BUS`]**, and the reason is worth writing down because the
/// name says otherwise. A CPU core holds its execution state across a guest
/// access — the RISC-V hart's session mutex is itself `LockRank::BUS` — so by
/// the time an MMIO write reaches a device, `BUS` is *already held*. A fabric
/// that also took `BUS` would be a lock-order violation on the first register
/// write, which is exactly what the ladder is for.
///
/// So the SPI fabric sits in the band between [`LockRank::BUS`] and
/// [`LockRank::DEVICE`], which is what [`LockRank::new`] exists for. The order
/// a transfer actually travels is:
///
/// ```text
///   CPU session (BUS 0x4000)
///     → SpiBus routing      (0x4400, here)
///       → SlavePins shifter (0x4800, SHIFTER_RANK)
///         → the slave's own state (DEVICE 0x5000)
///           → its output wires     (WIRE 0x6000)
/// ```
pub const FABRIC_RANK: LockRank = LockRank::new(0x4400);

/// The rank a [`SlavePins`] shift register takes.
///
/// Below the device's own state, because the shifter lock is deliberately held
/// across the call into the slave: reassembling a word and handing it over is
/// one step, and dropping the lock in the middle would let a second edge
/// interleave. See [`FABRIC_RANK`] for the whole ladder.
pub const SHIFTER_RANK: LockRank = LockRank::new(0x4800);

/// How many chip selects one bus routes.
///
/// Eight, because a controller's chip-select register is conventionally a byte
/// and nothing in this tree wants a ninth. A slave that asks for more is a
/// configuration error at construction, not a silent wrap.
pub const MAX_CHIP_SELECTS: usize = 8;

/// One SPI bus: a set of chip selects, each with at most one slave.
///
/// The fabric proper. It holds no timing and no clock — a bus is wires, and
/// *time belongs to the controller*, which is the only thing on the link with a
/// clock domain (`CLAUDE.md`: the scheduler owns time).
pub struct SpiBus {
    slaves: Mutex<[Option<Arc<dyn SpiSlave>>; MAX_CHIP_SELECTS]>,
    /// Which chip select is currently asserted, or [`MAX_CHIP_SELECTS`] for
    /// none. Published lock-free so a debug read never has to take the lock.
    active: AtomicU32,
}

impl fmt::Debug for SpiBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("SpiBus");
        s.field("active", &self.active.load(Ordering::Relaxed));
        match self.slaves.try_lock() {
            Some(slaves) => s.field("attached", &slaves.iter().filter(|s| s.is_some()).count()),
            None => s.field("attached", &"<in use>"),
        };
        s.finish()
    }
}

/// The `active` sentinel for "no chip select asserted".
const NO_SELECTION: u32 = MAX_CHIP_SELECTS as u32;

impl SpiBus {
    /// An empty bus.
    #[must_use]
    pub fn new() -> SpiBus {
        SpiBus {
            // `[None; N]` needs Copy, which `Option<Arc<_>>` is not.
            slaves: Mutex::with_rank(FABRIC_RANK, Default::default()),
            active: AtomicU32::new(NO_SELECTION),
        }
    }

    /// Put `slave` on chip select `cs`.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if `cs` is out of range or already taken. Two
    /// slaves on one chip select is not a wiring style, it is a short.
    pub fn attach(&self, cs: ChipSelect, slave: Arc<dyn SpiSlave>) -> crate::Result<()> {
        let index = usize::from(cs.0);
        if index >= MAX_CHIP_SELECTS {
            return Err(crate::Error::Config {
                at: alloc::format!("{cs}"),
                message: alloc::format!("an SPI bus routes {MAX_CHIP_SELECTS} chip selects"),
            });
        }
        let mut slaves = self.slaves.lock();
        if slaves[index].is_some() {
            return Err(crate::Error::Config {
                at: alloc::format!("{cs}"),
                message: alloc::string::String::from(
                    "two devices on one SPI chip select; give one of them another `cs`",
                ),
            });
        }
        slaves[index] = Some(slave);
        Ok(())
    }

    /// Remove whatever is on `cs`, reporting whether there was anything.
    pub fn detach(&self, cs: ChipSelect) -> bool {
        let index = usize::from(cs.0);
        if index >= MAX_CHIP_SELECTS {
            return false;
        }
        self.slaves.lock()[index].take().is_some()
    }

    /// The slave on `cs`, if any.
    #[must_use]
    pub fn slave(&self, cs: ChipSelect) -> Option<Arc<dyn SpiSlave>> {
        let index = usize::from(cs.0);
        if index >= MAX_CHIP_SELECTS {
            return None;
        }
        self.slaves.lock()[index].clone()
    }

    /// Every occupied chip select, in order.
    #[must_use]
    pub fn attached(&self) -> Vec<ChipSelect> {
        let slaves = self.slaves.lock();
        (0..MAX_CHIP_SELECTS)
            .filter(|i| slaves[*i].is_some())
            .map(|i| ChipSelect(i as u8))
            .collect()
    }

    /// Which chip select is asserted, if any.
    #[must_use]
    pub fn selected(&self) -> Option<ChipSelect> {
        match self.active.load(Ordering::Relaxed) {
            NO_SELECTION => None,
            n => Some(ChipSelect(n as u8)),
        }
    }

    /// Assert `cs`, deasserting whatever was asserted before.
    ///
    /// A bus has one chip select active at a time by construction, which is
    /// what makes "the controller forgot to deassert" a modelled situation
    /// rather than an undetectable one.
    pub fn select(&self, cs: Option<ChipSelect>) {
        let want = cs.map_or(NO_SELECTION, |c| u32::from(c.0));
        let had = self.active.swap(want, Ordering::Relaxed);
        if had == want {
            return;
        }
        // Deassert first, then assert: a slave's `select(false)` is where it
        // commits a command, and it must run before the next one starts.
        //
        // The `Arc`s are cloned out and the lock is released before either
        // call — a slave may reach back into the machine from `select`, which
        // is the re-entrancy contract in `core::device`.
        let (old, new) = {
            let slaves = self.slaves.lock();
            let old = (had != NO_SELECTION)
                .then(|| slaves[had as usize].clone())
                .flatten();
            let new = (want != NO_SELECTION)
                .then(|| slaves[want as usize].clone())
                .flatten();
            (old, new)
        };
        if let Some(slave) = old {
            slave.select(false);
        }
        if let Some(slave) = new {
            slave.select(true);
        }
    }

    /// Exchange one word with whichever slave is selected.
    ///
    /// Returns what came back on MISO. With nothing selected — or a chip select
    /// nothing answers — the result is all-ones, which is what a pulled-up,
    /// undriven MISO reads as. That is deliberately *not* an error: a
    /// controller clocking a bus with no slave on it is a perfectly ordinary
    /// thing for firmware to do while probing.
    pub fn transfer(&self, word: u32) -> u32 {
        let Some(cs) = self.selected() else {
            return u32::MAX;
        };
        let Some(slave) = self.slave(cs) else {
            return u32::MAX;
        };
        // Outside the lock: the slave may remap, drive a wire or reach a
        // sibling from inside `transfer`.
        exchange(&*slave, word)
    }

    /// What the selected slave would return, without transferring anything.
    ///
    /// The debug-access path. Nothing observable changes.
    #[must_use]
    pub fn peek(&self) -> u32 {
        self.selected()
            .and_then(|cs| self.slave(cs))
            .map_or(u32::MAX, |s| s.peek())
    }

    /// Whether every attached slave agrees with `format`.
    ///
    /// Returns the first chip select that does not, so a caller can name it.
    /// A bus does not enforce this — a controller is entitled to clock a slave
    /// in the wrong mode, and the result should be garbage rather than a panic
    /// — but a machine that wants the check can make it at realize time.
    #[must_use]
    pub fn check_format(&self, format: Format) -> Option<ChipSelect> {
        let slaves = self.slaves.lock();
        (0..MAX_CHIP_SELECTS).find_map(|i| {
            let slave = slaves[i].as_ref()?;
            (slave.format() != format).then_some(ChipSelect(i as u8))
        })
    }
}

impl Default for SpiBus {
    fn default() -> SpiBus {
        SpiBus::new()
    }
}

/// The named rendezvous: how a controller and its slaves find each other.
///
/// Modelled on [`crate::host::chardev::ports`], and a seam for the same reason
/// — a machine description can hand two independently constructed devices only
/// a *name*, and `core::bus` (`ROADMAP.md` §4) does not exist yet. When it
/// does, this becomes its registry and nothing else here changes.
///
/// ```
/// # #[cfg(feature = "bus-spi")] {
/// use rsemu::bus::spi::buses;
///
/// use std::sync::Arc;
///
/// let a = buses::open("doctest-spi");
/// let b = buses::open("doctest-spi");
/// assert!(Arc::ptr_eq(&a, &b), "the same name is the same bus");
/// buses::close("doctest-spi");
/// # }
/// ```
pub mod buses {
    use super::SpiBus;
    use alloc::collections::BTreeMap;
    use alloc::string::{String, ToString};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::sync::{LockRank, Mutex};

    /// Name to bus. `BTreeMap`, so listing is in name order rather than hash
    /// order (`CLAUDE.md`, determinism).
    static TABLE: Mutex<BTreeMap<String, Arc<SpiBus>>> =
        Mutex::with_rank(LockRank::LEAF, BTreeMap::new());

    /// The bus called `name`, creating it if this is the first mention.
    ///
    /// Both ends call this, and whichever is constructed first makes the bus.
    #[must_use]
    pub fn open(name: &str) -> Arc<SpiBus> {
        let mut table = TABLE.lock();
        if let Some(bus) = table.get(name) {
            return Arc::clone(bus);
        }
        let bus = Arc::new(SpiBus::new());
        table.insert(name.to_string(), Arc::clone(&bus));
        bus
    }

    /// The bus called `name`, if it has been opened.
    #[must_use]
    pub fn get(name: &str) -> Option<Arc<SpiBus>> {
        TABLE.lock().get(name).map(Arc::clone)
    }

    /// Forget `name`, reporting whether there was one.
    ///
    /// Anything still holding the `Arc` keeps working; a later [`open`] of the
    /// same name is a fresh bus. For tests that want the name back.
    pub fn close(name: &str) -> bool {
        TABLE.lock().remove(name).is_some()
    }

    /// Every open bus's name, in order.
    #[must_use]
    pub fn names() -> Vec<String> {
        TABLE.lock().keys().cloned().collect()
    }
}

// ---------------------------------------------------------------------------
// The bit-level front end
// ---------------------------------------------------------------------------

/// What a [`Shifter`] did with an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shifted {
    /// A bit was captured and the word is still assembling.
    ///
    /// Carried rather than bare so a caller can offer them to
    /// [`SpiSlave::partial`], which is how a part that turns the data line
    /// around mid-word gets to answer in the frame that asked.
    Partial {
        /// How many bits of the word have arrived.
        bits: u8,
        /// Those bits, right-aligned.
        received: u32,
    },
    /// The clock moved, but on the edge this mode changes data rather than
    /// samples it. Nothing was captured.
    Edge,
    /// A whole word came in, and this is what went out in exchange.
    Word {
        /// The word received on MOSI.
        mosi: u32,
        /// The word the slave put on MISO during the same transfer.
        miso: u32,
    },
    /// The edge was ignored — not selected, or the clock did not move.
    Idle,
}

/// A serial-to-parallel shift register: SCK edges in, whole words out.
///
/// This is the piece that makes one device model serve both link styles. It
/// holds no clock and schedules nothing — it is told about edges by whoever is
/// driving them, and *that* is who owns the time.
///
/// # Which edge does what
///
/// With CPHA 0 the bit is *sampled* on the leading edge and *changed* on the
/// trailing one; with CPHA 1 it is the other way round. Only the sampling edge
/// advances the word, so a `Shifter` sees exactly `bits` sampling edges per
/// word regardless of mode.
///
/// # MISO
///
/// Full duplex means the outgoing word must be known *before* the incoming one
/// is complete, so the shifter is **preloaded**: [`Shifter::preload`] is called
/// when the chip select is asserted, and each completed word reloads it from
/// the slave's answer to the word just received. That is what real hardware
/// does — the shift register holds its outgoing word and shifts both ways at
/// once — and it is why [`SpiSlave::transfer`]'s return value is what was
/// *already* there rather than a reply.
///
/// [`Shifter::miso`] is the bit currently presented.
#[derive(Debug)]
pub struct Shifter {
    format: Format,
    /// Bits received so far, left-aligned into the word for MSB-first.
    rx: u32,
    /// Bits still to send, consumed from whichever end `order` says.
    tx: u32,
    /// How many sampling edges have landed in the current word.
    count: u8,
    /// Whether CS is asserted.
    selected: bool,
    /// The last SCK level seen, for edge detection.
    sck: Level,
    /// The level MOSI is being held at between edges.
    mosi: Level,
    /// Whether `tx` holds a real outgoing word. False before the first
    /// [`Shifter::preload`], and MISO reads as the pull-up until it is true.
    loaded: bool,
}

impl Shifter {
    /// A shifter framing words as `format` says, with SCK at its idle level and
    /// nothing selected.
    #[must_use]
    pub fn new(format: Format) -> Shifter {
        Shifter {
            format,
            rx: 0,
            tx: 0,
            count: 0,
            selected: false,
            sck: format.mode.idle_level(),
            mosi: Level::Low,
            loaded: false,
        }
    }

    /// The framing in use.
    #[must_use]
    pub const fn format(&self) -> Format {
        self.format
    }

    /// Change the framing, abandoning any word in progress.
    ///
    /// A controller whose guest rewrites its mode register mid-word does
    /// exactly this, and the half-assembled word is lost on real silicon too.
    pub fn set_format(&mut self, format: Format) {
        self.format = format;
        self.sck = format.mode.idle_level();
        self.abandon();
    }

    /// Whether a word is part-way through.
    #[must_use]
    pub const fn in_word(&self) -> bool {
        self.count > 0
    }

    /// How many bits of the current word have arrived.
    #[must_use]
    pub const fn bit_count(&self) -> u8 {
        self.count
    }

    /// Whether the chip select is asserted.
    #[must_use]
    pub const fn selected(&self) -> bool {
        self.selected
    }

    /// The level the shifter is presenting on MISO.
    ///
    /// All-ones is the idle state, so an unselected shifter presents `High` —
    /// the pull-up. Real parts tri-state instead; a wire with one driver
    /// cannot, and reading the pull-up is what the controller would see.
    #[must_use]
    pub fn miso(&self) -> Level {
        if !self.selected || !self.loaded {
            return Level::High;
        }
        let bit = match self.format.order {
            BitOrder::MsbFirst => {
                let shift = self.format.bits - 1 - self.count.min(self.format.bits - 1);
                (self.tx >> shift) & 1
            }
            BitOrder::LsbFirst => (self.tx >> self.count.min(self.format.bits - 1)) & 1,
        };
        Level::from_bool(bit != 0)
    }

    /// Hold MOSI at `level` until the next sampling edge.
    pub fn set_mosi(&mut self, level: Level) {
        self.mosi = level;
    }

    /// Load the word to be shifted out.
    ///
    /// Called when the chip select is asserted, and again — internally — from
    /// the slave's answer each time a word completes.
    pub fn preload(&mut self, word: u32) {
        self.tx = self.format.truncate(word);
        self.loaded = true;
    }

    /// The chip select changed.
    ///
    /// Returns the word that was in flight if deassertion abandoned one, so a
    /// device can implement the datasheet's own rule about short frames — the
    /// ST7272A's is §7.1(d): "If less than 16 bits of SCL are input while CS is
    /// low, the transferred data is ignored."
    pub fn set_select(&mut self, selected: bool) -> Option<u32> {
        if selected == self.selected {
            return None;
        }
        self.selected = selected;
        let partial = self.in_word().then_some(self.rx);
        self.abandon();
        partial
    }

    /// SCK moved to `level`.
    ///
    /// `exchange` is called when a word completes: it is handed the word just
    /// received and returns the one to shift out next. A closure rather than an
    /// `&dyn SpiSlave` because the shifter is also used *by* a controller,
    /// which has no slave to hand it.
    pub fn set_sck(&mut self, level: Level, mut exchange: impl FnMut(u32) -> u32) -> Shifted {
        if level == self.sck {
            return Shifted::Idle;
        }
        self.sck = level;
        if !self.selected {
            return Shifted::Idle;
        }
        if !self.format.mode.samples_on(level) {
            // The changing edge. Nothing is captured; MISO has already been
            // presented for the bit about to be sampled.
            return Shifted::Edge;
        }
        // Capture MOSI.
        match self.format.order {
            BitOrder::MsbFirst => {
                self.rx = (self.rx << 1) | u32::from(self.mosi.as_bool());
            }
            BitOrder::LsbFirst => {
                self.rx |= u32::from(self.mosi.as_bool()) << self.count;
            }
        }
        self.count += 1;
        if self.count < self.format.bits {
            return Shifted::Partial {
                bits: self.count,
                received: self.rx,
            };
        }
        let mosi = self.format.truncate(self.rx);
        let miso = self.tx;
        self.rx = 0;
        self.count = 0;
        // The next word's outgoing bits come from the slave's response to this
        // one, which is what a preloaded shift register does.
        self.preload(exchange(mosi));
        Shifted::Word { mosi, miso }
    }

    /// Throw away any word in progress and forget the preload.
    pub fn abandon(&mut self) {
        self.rx = 0;
        self.tx = 0;
        self.count = 0;
        self.loaded = false;
    }

    /// Everything a snapshot needs. Paired with [`Shifter::restore`].
    #[must_use]
    pub const fn snapshot(&self) -> (u32, u32, u8, bool, bool, bool, bool) {
        (
            self.rx,
            self.tx,
            self.count,
            self.selected,
            self.sck.is_high(),
            self.mosi.is_high(),
            self.loaded,
        )
    }

    /// Restore what [`Shifter::snapshot`] returned.
    pub fn restore(&mut self, state: (u32, u32, u8, bool, bool, bool, bool)) {
        let (rx, tx, count, selected, sck, mosi, loaded) = state;
        self.rx = rx;
        self.tx = tx;
        self.count = count.min(self.format.bits);
        self.selected = selected;
        self.sck = Level::from_bool(sck);
        self.mosi = Level::from_bool(mosi);
        self.loaded = loaded;
    }
}

// ---------------------------------------------------------------------------
// A slave's pins
// ---------------------------------------------------------------------------

/// Which of a slave's input pins a wire is connected to.
///
/// A device declares one [`crate::core::wire::WireSink`] per pin and tells this
/// module which by `line`, which is what
/// [`crate::core::device::SinkPin::line`] carries.
pub mod pin {
    /// The serial clock, `SCL` on the ST7272A.
    pub const SCK: u32 = 0;
    /// Data from the controller, `SDA` on a 3-wire part.
    pub const MOSI: u32 = 1;
    /// Chip select. **Active low on the wire**, as the pin on the part is.
    pub const CS: u32 = 2;
    /// The name a machine description writes for the clock pin.
    pub const SCK_NAME: &str = "sck";
    /// The name for the data-in pin.
    pub const MOSI_NAME: &str = "mosi";
    /// The name for the chip select.
    pub const CS_NAME: &str = "cs";
    /// The name for the data-out pin the slave drives.
    pub const MISO_NAME: &str = "miso";
}

/// A slave's wire-level pins: the bit-banging front end, ready made.
///
/// Wrap one of these around an `Arc<dyn SpiSlave>` and the device gains `sck`,
/// `mosi` and `cs` inputs and a `miso` output, with no protocol code of its
/// own. **This is what a peripheral needs in order to be driven by a GPIO
/// controller** — or by an SPI controller in [`Link::Wired`] mode, which is
/// electrically the same thing.
///
/// # Locking
///
/// The shifter takes [`SHIFTER_RANK`], which sits between [`LockRank::BUS`] and
/// [`LockRank::DEVICE`]; that constant's docs give the whole ladder and why
/// neither of the two named ranks would do. The lock is held across the call
/// into the slave on purpose — reassembling a word and handing it over is one
/// step — so the slave's own state must rank *below* it, which `DEVICE` does.
///
/// The chip-select path releases the shifter before calling the slave, because
/// `select` is where a part commits a command and may reach further
/// ([`crate::core::device`], the re-entrancy contract).
pub struct SlavePins {
    slave: Arc<dyn SpiSlave>,
    shifter: Mutex<Shifter>,
    /// The MISO output, connected at realize time.
    miso: Mutex<Option<WireSource>>,
    /// The level MISO is being held at. An atomic so a debug read is free.
    miso_level: AtomicBool,
    /// Every input pin handed out by [`SlavePins::sink`].
    ///
    /// **A net holds only a weak reference to its sinks** (`core::device`),
    /// which is what stops an IRQ/ack loop leaking — so a sink nobody else
    /// holds is dropped the instant it is handed over, and the wire silently
    /// delivers to nothing. Keeping them here is the strong half of that
    /// arrangement, and the device owning its own pins is exactly what §4.3
    /// intends.
    pins: Mutex<Vec<Arc<PinSink>>>,
}

impl fmt::Debug for SlavePins {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SlavePins")
            .field("slave", &self.slave)
            .field(
                "miso",
                &Level::from_bool(self.miso_level.load(Ordering::Relaxed)),
            )
            .finish_non_exhaustive()
    }
}

impl SlavePins {
    /// Pins for `slave`, framing words the way it asks.
    #[must_use]
    pub fn new(slave: Arc<dyn SpiSlave>) -> SlavePins {
        let format = slave.format();
        SlavePins {
            slave,
            shifter: Mutex::with_rank(SHIFTER_RANK, Shifter::new(format)),
            miso: Mutex::with_rank(LockRank::WIRE, None),
            miso_level: AtomicBool::new(true),
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        }
    }

    /// The device these pins belong to.
    #[must_use]
    pub fn slave(&self) -> &Arc<dyn SpiSlave> {
        &self.slave
    }

    /// Connect the MISO output.
    pub fn connect_miso(&self, source: WireSource) {
        *self.miso.lock() = Some(source);
        self.publish_miso();
    }

    /// The level MISO is being driven to.
    #[must_use]
    pub fn miso_level(&self) -> Level {
        Level::from_bool(self.miso_level.load(Ordering::Relaxed))
    }

    /// Re-drive MISO from whatever the shifter is presenting.
    ///
    /// Also the realize-sweep answer for the `miso` pin
    /// ([`crate::core::device::Device::announce`]).
    pub fn publish_miso(&self) {
        let level = self.shifter.lock().miso();
        self.miso_level.store(level.is_high(), Ordering::Relaxed);
        let port = self.miso.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Drive one of the input pins.
    ///
    /// `line` is one of [`pin::SCK`], [`pin::MOSI`], [`pin::CS`]. An unknown
    /// line is ignored rather than panicking: a machine description that wires
    /// something odd is a config error the validator reports, not a crash.
    pub fn drive(&self, line: u32, level: Level) {
        match line {
            pin::MOSI => {
                self.shifter.lock().set_mosi(level);
                // MOSI does not move the word on, so MISO cannot change.
                return;
            }
            pin::CS => {
                // CS is active low on the pin: the ST7272A's frame runs from
                // the falling edge to the next rising one (datasheet §7.1(b)).
                let selected = level.is_low();
                let moved = {
                    let mut shifter = self.shifter.lock();
                    let was = shifter.selected();
                    shifter.set_select(selected);
                    (was != shifter.selected()).then(|| shifter.selected())
                };
                let Some(moved) = moved else {
                    // The line did not move. Re-driving a level a wire already
                    // holds must not look like a fresh frame to the part.
                    return;
                };
                // Outside the lock: `select` is where a part commits a command
                // and it may reach back into the machine.
                self.slave.select(moved);
                if moved {
                    // Preload the outgoing word before the first clock edge.
                    // With CPHA 0 the controller samples MISO on the *leading*
                    // edge of bit 0, so a shift register that only loaded on
                    // the first edge would present the pull-up for that bit.
                    let word = self.slave.peek();
                    self.shifter.lock().preload(word);
                }
                self.publish_miso();
                return;
            }
            pin::SCK => {}
            _ => return,
        }

        // SCK. The closure reaches the slave, and the shifter lock is held
        // while it runs — which is why it is the *only* outward call this path
        // makes, and why `SpiSlave` implementors mutate their own state and
        // defer anything else (`core::device`, the re-entrancy contract).
        let shifted = {
            let mut shifter = self.shifter.lock();
            let slave = &self.slave;
            // `transfer` returns the word that just went *out*, which the
            // shifter has already sent; what it needs next is whatever the
            // slave has loaded now. Asking for both is what keeps this path
            // and `SpiBus::transfer` telling the same story.
            shifter.set_sck(level, |word| {
                slave.transfer(word);
                slave.peek()
            })
        };
        if let Shifted::Partial { bits, received } = shifted {
            // Each shifter lock is its own statement, deliberately. A guard
            // taken in an `if` condition lives to the end of the whole `if`, so
            // folding these into one `&&` chain re-enters a `LockRank::BUS`
            // mutex while still holding it — which the ladder catches, and
            // which would be a deadlock on a threaded backend.
            let msb_first = self.shifter.lock().format().order == BitOrder::MsbFirst;
            if msb_first && self.slave.turnaround() == Some(bits) {
                // Outside the shifter lock: a part answering mid-word may reach
                // its own registers, and this is the moment it learns the
                // address.
                if let Some(word) = self.slave.partial(bits, received) {
                    self.shifter.lock().preload(word);
                }
            }
        }
        self.publish_miso();
    }

    /// A sink for one input pin, for [`crate::core::device::Device::sink`].
    ///
    /// The returned pin is **also kept here**, because the net that receives it
    /// holds only a weak reference; see the field's own note.
    #[must_use]
    pub fn sink(self: &Arc<Self>, line: u32) -> Arc<dyn WireSink> {
        let pin = Arc::new(PinSink {
            pins: Arc::clone(self),
            line,
        });
        self.pins.lock().push(Arc::clone(&pin));
        pin as Arc<dyn WireSink>
    }

    /// Reset to power-on: nothing selected, SCK idle, no word in flight.
    pub fn reset(&self) {
        let format = self.slave.format();
        {
            let mut shifter = self.shifter.lock();
            shifter.set_format(format);
            shifter.set_select(false);
        }
        self.publish_miso();
    }

    /// Everything a snapshot needs.
    #[must_use]
    pub fn snapshot(&self) -> (u32, u32, u8, bool, bool, bool, bool) {
        self.shifter.lock().snapshot()
    }

    /// Restore what [`SlavePins::snapshot`] returned.
    pub fn restore(&self, state: (u32, u32, u8, bool, bool, bool, bool)) {
        self.shifter.lock().restore(state);
        self.publish_miso();
    }
}

/// One pin of a [`SlavePins`], as the wire graph sees it.
struct PinSink {
    pins: Arc<SlavePins>,
    line: u32,
}

impl fmt::Debug for PinSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PinSink").field("line", &self.line).finish()
    }
}

impl WireSink for PinSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.pins.drive(self.line, level);
    }
}
