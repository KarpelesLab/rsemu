//! I²C: the two-wire bus, both ways round.
//!
//! # The specification
//!
//! Unlike SPI, I²C **has** one: NXP's **UM10204**, *I²C-bus specification and
//! user manual*, and it is free. Everything this module fixes comes from it,
//! cited by section. The revision read while writing this was **Rev. 7.0, 1
//! October 2021**, which renamed *master*/*slave* to *controller*/*target*.
//! This tree keeps the older words, for two reasons: every device datasheet and
//! every SoC reference manual it has to agree with still uses them
//! (ST's RM0090 calls the register bit `MSL`, "Master/slave"), and
//! [`crate::bus::spi`] already spells its device seam `SpiSlave`. The
//! specification's own terms appear in the docs where a section is quoted.
//!
//! # The shape of the bus, and why it is not SPI's
//!
//! SPI is point-to-point plus a chip select: the controller picks a wire and
//! shifts a word down it. I²C has neither. It is **two open-drain lines with a
//! pull-up** (§3.1.1), shared by every device, and the selection is *in band* —
//! a START condition, then an address byte every device compares against its
//! own (§3.1.10). Three consequences run through this whole module:
//!
//! * **There is no [`ChipSelect`](crate::bus::spi::ChipSelect) equivalent.**
//!   [`I2cBus`] holds a list of slaves and offers each address to all of them,
//!   which is also what makes the general call (§3.1.13) expressible at all.
//! * **Every byte is answered.** A ninth clock carries an [`Ack`] from the
//!   receiver (§3.1.6), in both directions, so the unit of transfer is a byte
//!   *and* an acknowledgement — never a bare `u8`.
//! * **The lines are wired-AND.** Any device may pull one low; nobody drives
//!   one high. That single fact is what gives clock stretching (§3.1.9) and
//!   multi-master arbitration (§3.1.8), and it is why the wired model here uses
//!   [`Resolve::And`](crate::core::wire::Resolve::And) rather than the
//!   point-to-point wires SPI drives.
//!
//! # The decision this module exists to make explicit
//!
//! `docs/buses/low-speed.md`, as for SPI: these are timing protocols on wires,
//! most emulators model them transactionally, that is much faster and usually
//! fine — and **some guest firmware bit-bangs the lines through GPIO and will
//! notice**. I²C is bit-banged far more often than SPI is. So rsemu does both,
//! the choice is a machine-description property rather than an accident, and a
//! machine file says which it uses.
//!
//! ```text
//!                  ┌──────────── Link::Transactional ─────────────┐
//!   controller ────┤  bus.start(addr, dir) / write / read / stop  ├──► slave
//!                  │  one call per bus event, no wires toggled     │
//!                  └──────────────────────────────────────────────┘
//!
//!                  ┌──────────── Link::Wired ─────────────────────┐
//!   controller ────┤  SCL and SDA are two open-drain nets. Every  ├──► slave
//!    (or a GPIO    │  participant pulls low or releases; the net   │    pins
//!     controller,  │  is the AND of its drivers. The slave's own   │
//!     or a guest   │  bit engine reassembles the byte and calls    │
//!     toggling     │  exactly the same method.                     │
//!     pins)        └──────────────────────────────────────────────┘
//! ```
//!
//! **The peripheral is written once.** A device model implements [`I2cSlave`],
//! which is byte-level, and gets the bit-level front end for free from
//! [`wires::SlaveWires`]. `both_link_models_produce_identical_traffic` asserts
//! that the two paths deliver the identical sequence of `I2cSlave` calls.
//!
//! # Clock stretching is real in both, and differently
//!
//! This is the part a transactional model usually cannot express, so it is
//! worth being precise about what each one does.
//!
//! * [`Link::Wired`]: a slave that needs time **pulls SCL low** (§3.1.9). The
//!   master releases SCL and then looks at the net; while the net reads low it
//!   makes no progress and burns half periods. That is the physical mechanism,
//!   modelled as the physical mechanism.
//! * [`Link::Transactional`]: there is no SCL, so the master asks
//!   [`I2cBus::stretching`] instead and stalls on the answer, burning the same
//!   half periods.
//!
//! The two are honestly different — one is a level on a net that anything else
//! can also watch, the other is a question — and they cost the same virtual
//! time, so firmware that polls a busy flag sees one timeline.
//!
//! # Timing, and why the two links cost the same
//!
//! A bus event's price is fixed here, in half periods of the master's SCL, so
//! that a transactional controller and a wired one charge the scheduler the
//! same amount: [`START_HALF_PERIODS`], [`BYTE_HALF_PERIODS`],
//! [`STOP_HALF_PERIODS`]. A byte is nine bit slots (eight data plus the
//! acknowledge, §3.1.5) at two half periods each.
//!
//! # Finding each other
//!
//! As in [`crate::bus::spi`]: a controller and its slaves are separate objects
//! in a machine description and there is no `core::bus` yet, so they meet
//! through [`buses`], a named rendezvous table. Both ends name the same bus
//! (`bus = "i2c1"`).
//!
//! # Sources
//!
//! No emulator was consulted (`ROADMAP.md` §1). Every protocol statement here
//! comes from NXP **UM10204** Rev. 7.0, cited by section. The Linux `i2c`
//! subsystem is GPLv2 and was not opened.

pub mod wires;

#[cfg(test)]
mod tests;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::sync::{LockRank, Mutex};

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

/// Which way the bytes after an address byte travel.
///
/// The eighth bit of the first byte after a START (§3.1.10): "a 'zero'
/// indicates a transmission (WRITE), a 'one' indicates a request for data
/// (READ)". Named from the **master's** point of view, as the specification
/// names it — [`Direction::Read`] means the slave transmits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Direction {
    /// The master transmits. `R/W̅` is 0.
    #[default]
    Write,
    /// The master receives; the addressed slave transmits. `R/W̅` is 1.
    Read,
}

impl Direction {
    /// The `R/W̅` bit as it appears in the address byte.
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Direction::Write => 0,
            Direction::Read => 1,
        }
    }

    /// The direction an address byte's low bit encodes.
    #[must_use]
    pub const fn from_bit(bit: u8) -> Direction {
        if bit & 1 == 0 {
            Direction::Write
        } else {
            Direction::Read
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Direction::Write => "write",
            Direction::Read => "read",
        })
    }
}

/// The ninth bit of every byte (§3.1.6).
///
/// A real `enum` rather than a `bool`, and this is exactly the case
/// [`Level`](crate::core::wire::Level) is an enum for: the acknowledge is
/// **active low** on the wire, so `true` at a call site means whichever of the
/// two a reader guesses. It also never gains a third value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Ack {
    /// The receiver pulled SDA low during the ninth clock: byte taken.
    #[default]
    Ack,
    /// SDA stayed high: not acknowledged. §3.1.6 lists the five reasons.
    Nack,
}

impl Ack {
    /// The acknowledge a receiver holding SDA at this level is giving.
    ///
    /// Low is [`Ack::Ack`] — the acknowledge is active low.
    #[must_use]
    pub const fn from_level(level: crate::core::wire::Level) -> Ack {
        if level.is_low() { Ack::Ack } else { Ack::Nack }
    }

    /// The level a receiver drives to give this acknowledge.
    #[must_use]
    pub const fn level(self) -> crate::core::wire::Level {
        match self {
            Ack::Ack => crate::core::wire::Level::Low,
            Ack::Nack => crate::core::wire::Level::High,
        }
    }

    /// Whether this is an acknowledgement.
    #[must_use]
    pub const fn is_ack(self) -> bool {
        matches!(self, Ack::Ack)
    }

    /// The wired-AND of two acknowledges.
    ///
    /// One low driver is enough to pull SDA down, so an ACK from *any* receiver
    /// is the ACK the master sees. §3.1.13 relies on this for the general call:
    /// "if one or more targets acknowledge, the not-acknowledge will not be
    /// seen by the controller."
    #[must_use]
    pub const fn merge(self, other: Ack) -> Ack {
        match (self, other) {
            (Ack::Nack, Ack::Nack) => Ack::Nack,
            _ => Ack::Ack,
        }
    }
}

impl fmt::Display for Ack {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Ack::Ack => "ack",
            Ack::Nack => "nack",
        })
    }
}

/// A slave address: seven bits, or ten.
///
/// A real `enum` because the two are not the same number in a wider field —
/// they are *different framings on the wire*, one byte against two (§3.1.11) —
/// and a `u16` that sometimes means one and sometimes the other is the bug this
/// type exists to make unrepresentable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Address {
    /// The ordinary case (§3.1.10). Always in `0..=0x7f`.
    Seven(u8),
    /// Ten-bit addressing (§3.1.11). Always in `0..=0x3ff`.
    Ten(u16),
}

/// The general call address, `0000 000` with `R/W̅` = 0 (§3.1.13).
///
/// "for addressing every device connected to the I²C-bus at the same time".
/// Only meaningful with [`Direction::Write`]; the same seven bits with `R/W̅`
/// set are the START byte instead (Table 4).
pub const GENERAL_CALL: Address = Address::Seven(0x00);

/// The first byte of a 10-bit address, before the two address bits and `R/W̅`
/// are folded in: `1111 0XX` (§3.1.11).
const TEN_BIT_HEADER: u8 = 0b1111_0000;

/// The mask that isolates a first byte's 10-bit header.
const TEN_BIT_HEADER_MASK: u8 = 0b1111_1000;

impl Address {
    /// A seven-bit address, or `None` above `0x7f`.
    #[must_use]
    pub const fn seven(address: u8) -> Option<Address> {
        if address > 0x7f {
            None
        } else {
            Some(Address::Seven(address))
        }
    }

    /// A ten-bit address, or `None` above `0x3ff`.
    #[must_use]
    pub const fn ten(address: u16) -> Option<Address> {
        if address > 0x3ff {
            None
        } else {
            Some(Address::Ten(address))
        }
    }

    /// The address as a number, whichever width it is.
    #[must_use]
    pub const fn bits(self) -> u16 {
        match self {
            Address::Seven(a) => a as u16,
            Address::Ten(a) => a,
        }
    }

    /// Whether this is a ten-bit address.
    #[must_use]
    pub const fn is_ten_bit(self) -> bool {
        matches!(self, Address::Ten(_))
    }

    /// The first byte on the wire, `R/W̅` included.
    ///
    /// For seven bits that is `address << 1 | R/W̅` (§3.1.10); for ten it is the
    /// header `1111 0XX` carrying the top two address bits (§3.1.11).
    #[must_use]
    pub const fn first_byte(self, dir: Direction) -> u8 {
        match self {
            Address::Seven(a) => (a << 1) | dir.bit(),
            Address::Ten(a) => TEN_BIT_HEADER | (((a >> 8) as u8 & 0b11) << 1) | dir.bit(),
        }
    }

    /// The second address byte, for ten-bit addressing only.
    ///
    /// "the eight bits of the second byte of the target address" (§3.1.11).
    #[must_use]
    pub const fn second_byte(self) -> Option<u8> {
        match self {
            Address::Seven(_) => None,
            Address::Ten(a) => Some(a as u8),
        }
    }

    /// The top two bits of a ten-bit address, as they ride in the header.
    #[must_use]
    pub const fn ten_bit_high(self) -> Option<u8> {
        match self {
            Address::Seven(_) => None,
            Address::Ten(a) => Some((a >> 8) as u8 & 0b11),
        }
    }

    /// Whether a first byte is a ten-bit header rather than a seven-bit
    /// address: `1111 0XX`.
    #[must_use]
    pub const fn is_ten_bit_header(byte: u8) -> bool {
        byte & TEN_BIT_HEADER_MASK == TEN_BIT_HEADER
    }

    /// The seven-bit address a first byte carries, ignoring `R/W̅`.
    #[must_use]
    pub const fn seven_from_byte(byte: u8) -> Address {
        Address::Seven(byte >> 1)
    }

    /// Whether these seven bits are reserved (§3.1.12, Table 4).
    ///
    /// Two groups of eight: `0000 XXX` and `1111 XXX`. A device model is free
    /// to answer one anyway — several standard functions live there — but a
    /// machine that puts an ordinary EEPROM on `0x00` has a bug, and this is
    /// what lets a validator say so.
    #[must_use]
    pub const fn is_reserved(self) -> bool {
        match self {
            Address::Seven(a) => a & 0b111_1000 == 0 || a & 0b111_1000 == 0b111_1000,
            // A ten-bit address is *reached* through a reserved header; the
            // address itself is not one of the reserved seven-bit slots.
            Address::Ten(_) => false,
        }
    }
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Address::Seven(a) => write!(f, "{a:#04x}"),
            Address::Ten(a) => write!(f, "{a:#05x}/10"),
        }
    }
}

// ---------------------------------------------------------------------------
// Cost
// ---------------------------------------------------------------------------

/// How many SCL half periods a START or repeated START costs.
///
/// Four, and the same four either way round: release SDA, release SCL, pull SDA
/// low while SCL is high (which *is* the START, §3.1.4), pull SCL low. From an
/// idle bus the first two are already true and cost nothing but time, which is
/// the bus-free time a real master leaves anyway.
pub const START_HALF_PERIODS: u32 = 4;

/// How many SCL half periods one byte costs.
///
/// Eighteen: nine bit slots — eight data bits plus the acknowledge (§3.1.5,
/// §3.1.6) — at two half periods each.
pub const BYTE_HALF_PERIODS: u32 = 18;

/// How many SCL half periods a STOP costs.
///
/// Two: release SCL, then release SDA while SCL is high (§3.1.4).
pub const STOP_HALF_PERIODS: u32 = 2;

// ---------------------------------------------------------------------------
// The device seam
// ---------------------------------------------------------------------------

/// A peripheral on an I²C bus, as the bus sees it.
///
/// Byte-level on purpose, for the same reason [`SpiSlave`](crate::bus::spi::SpiSlave)
/// is word-level: a datasheet describes a transaction in bytes, and a device
/// model that had to reassemble them from bit transitions would be writing
/// [`wires::SlaveWires`] again, differently, once per device.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0).
///
/// # The contract
///
/// * [`address`](I2cSlave::address) is called on every START and repeated
///   START, on *every* attached device, with the address the master sent.
///   Returning [`Ack::Ack`] means "that is me, and I am taking this transfer";
///   [`Ack::Nack`] means it is not. More than one device may accept the
///   [`GENERAL_CALL`]; for any other address, two acceptors is a machine
///   description bug ([`I2cBus::conflicts`] counts them).
/// * [`write`](I2cSlave::write) and [`read`](I2cSlave::read) are called only
///   while addressed, and only in the direction the address byte asked for.
/// * [`read_ack`](I2cSlave::read_ack) reports what the master did with the byte
///   [`read`](I2cSlave::read) just handed over. **That is where an address
///   counter advances**, because on the wire it is the acknowledge that says
///   the byte arrived (§3.1.6).
/// * [`stop`](I2cSlave::stop) is called on a STOP, and on a repeated START that
///   addressed somebody else — both end this device's transaction (§3.1.11:
///   "remains addressed by the controller until it receives a STOP condition
///   (P) or a repeated START condition (Sr) followed by a different target
///   address").
pub trait I2cSlave: Send + Sync + fmt::Debug {
    /// The bus addressed somebody. Answer [`Ack::Ack`] to take the transfer.
    ///
    /// Called on every attached device, addressed or not, so a device that
    /// wants the general call sees it.
    fn address(&self, address: Address, dir: Direction) -> Ack;

    /// Whether this device holds any ten-bit address whose top two bits are
    /// `high`, for the first-byte acknowledge of a ten-bit write (§3.1.11: "It
    /// is possible that more than one device finds a match and generate an
    /// acknowledge (A1)").
    ///
    /// Defaults to `false`, which is every seven-bit-only part.
    fn ten_bit_header(&self, high: u8) -> bool {
        let _ = high;
        false
    }

    /// A byte from the master. Answer whether it was taken.
    fn write(&self, byte: u8) -> Ack;

    /// The byte to put on the bus for a master read.
    ///
    /// Does **not** advance anything: the byte is not delivered until the
    /// master acknowledges it, which arrives at [`read_ack`](I2cSlave::read_ack).
    fn read(&self) -> u8;

    /// What the master said about the byte [`read`](I2cSlave::read) handed over.
    ///
    /// [`Ack::Nack`] is the master saying "that was the last one" (§3.1.6,
    /// reason 5), after which the slave releases SDA and expects a STOP or a
    /// repeated START.
    fn read_ack(&self, ack: Ack) {
        let _ = ack;
    }

    /// A STOP, or a repeated START that went to another device.
    fn stop(&self);

    /// Whether this device is holding SCL low (§3.1.9).
    ///
    /// Most parts cannot — "most target devices do not include an SCL driver so
    /// they are unable to stretch the clock" — so the default is `false`, and a
    /// model that returns `true` must also have an `scl` output pin for the
    /// wired link to be able to show it.
    fn stretching(&self) -> bool {
        false
    }

    /// What [`read`](I2cSlave::read) would return, without any side effect.
    ///
    /// The [`MemAttrs::debug`](crate::core::space::MemAttrs::debug) rule applied
    /// to a bus rather than to a register block. Defaults to all-ones, which is
    /// what an undriven, pulled-up SDA reads as.
    fn peek(&self) -> u8 {
        0xff
    }
}

// ---------------------------------------------------------------------------
// Modelling mode
// ---------------------------------------------------------------------------

/// How a controller carries a byte to its slaves.
///
/// **The point of this type is that it is written down.** A machine file that
/// says `link = "transactional"` has made a choice; one that never mentions it
/// has inherited a default, which is exactly the accident
/// `docs/buses/low-speed.md` asks us to avoid. Controllers therefore make the
/// property *required*, as [`crate::bus::spi`] does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Link {
    /// One call per bus event through [`I2cBus`]. No wires move.
    ///
    /// The fast, ordinary model, and the right one when nothing outside the
    /// controller can see the lines. **A transfer still costs its real time**,
    /// so a guest polling a status flag sees the same timing either way; what
    /// differs is only whether the individual edges exist.
    #[default]
    Transactional,

    /// SCL and SDA are two open-drain nets, driven one edge per half bit
    /// period, paced by the scheduler.
    ///
    /// Slower, and necessary when anything else is watching the lines: a second
    /// master arbitrating for the bus, a logic analyser model, or — the case
    /// this exists for — guest firmware that also bit-bangs the same pins
    /// through GPIO and would notice a controller that teleported a byte.
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
// Where this module sits in the lock ladder
// ---------------------------------------------------------------------------

/// The rank an [`I2cBus`]'s own routing state takes.
///
/// **Not [`LockRank::BUS`]**, for the reason `docs/buses/low-speed.md` records
/// and [`crate::bus::spi::FABRIC_RANK`] found the hard way: a CPU core holds its
/// execution state across a guest access — the RISC-V hart's session mutex *is*
/// `LockRank::BUS` — so by the time an MMIO write reaches a device, `BUS` is
/// already held. A fabric that also took `BUS` would be a lock-order violation
/// on the first register write.
///
/// This fabric sits in the same band SPI's does, one step further along so the
/// two can appear in one machine without their ladders colliding:
///
/// ```text
///   CPU session (BUS 0x4000)
///     → I2cBus routing        (0x4500, here)
///       → the wire bit engine (0x4900, WIRES_RANK)
///         → the slave's own state (DEVICE 0x5000)
///           → its output wires     (WIRE 0x6000)
///             → a fan-in table     (LEAF)
/// ```
pub const FABRIC_RANK: LockRank = LockRank::new(0x4500);

/// The rank a [`wires::SlaveWires`] or [`wires::MasterWires`] bit engine takes.
///
/// Below the device's own state, because the engine's lock is deliberately held
/// across the call into the slave: reassembling a byte and handing it over is
/// one step, and dropping the lock in the middle would let a second edge
/// interleave. See [`FABRIC_RANK`] for the whole ladder.
pub const WIRES_RANK: LockRank = LockRank::new(0x4900);

/// How many devices one bus routes.
///
/// UM10204 sets no count limit — the real one is bus capacitance (§7.2) — so
/// this is a sanity bound rather than a physical one: 128 is the whole seven-bit
/// address space, and a machine description that names a hundred and twenty-nine
/// devices has a generated-file bug rather than a board.
pub const MAX_SLAVES: usize = 128;

// ---------------------------------------------------------------------------
// The fabric
// ---------------------------------------------------------------------------

/// What the bus is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusState {
    /// No transaction. Both lines are released.
    ///
    /// "The bus is considered to be free again a certain time after the STOP
    /// condition" (§3.1.4).
    Free,
    /// A START has happened and no device answered the address.
    ///
    /// Still busy — the bus is busy from the START (§3.1.4) — but nothing is
    /// listening, so bytes go nowhere and read back as the pull-up.
    Unaddressed,
    /// A device took the transfer and bytes are moving this way.
    Addressed {
        /// Which way.
        dir: Direction,
        /// How many devices answered. More than one is only legal for the
        /// general call (§3.1.13).
        responders: usize,
    },
}

impl BusState {
    /// Whether a transaction is open: after a START, before the STOP.
    #[must_use]
    pub const fn is_busy(self) -> bool {
        !matches!(self, BusState::Free)
    }
}

/// One I²C bus: a set of devices sharing two lines.
///
/// The fabric proper. It holds no timing and no clock — a bus is wires, and
/// *time belongs to the master*, which is the only thing on the link with a
/// clock domain (`CLAUDE.md`: the scheduler owns time).
pub struct I2cBus {
    inner: Mutex<Inner>,
}

/// Everything the fabric tracks, under one lock.
#[derive(Debug, Default)]
struct Inner {
    slaves: Vec<Arc<dyn I2cSlave>>,
    /// Indices into `slaves` of the devices that took the current transfer.
    addressed: Vec<usize>,
    /// Whether a transaction is open.
    started: bool,
    /// Which way the current transfer runs.
    dir: Direction,
    /// How many times two devices answered one non-general-call address.
    conflicts: u32,
}

impl fmt::Debug for I2cBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("I2cBus");
        match self.inner.try_lock() {
            Some(inner) => s
                .field("attached", &inner.slaves.len())
                .field("started", &inner.started)
                .field("addressed", &inner.addressed.len()),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

impl I2cBus {
    /// An empty bus.
    #[must_use]
    pub fn new() -> I2cBus {
        I2cBus {
            inner: Mutex::with_rank(FABRIC_RANK, Inner::default()),
        }
    }

    /// Put `slave` on the bus.
    ///
    /// There is no chip select to collide over — a device's address is its own
    /// business and only it knows what it answers — so this cannot detect two
    /// parts on one address. [`conflicts`](I2cBus::conflicts) reports it at run
    /// time instead, the first time two devices acknowledge the same address.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if the bus already holds [`MAX_SLAVES`].
    pub fn attach(&self, slave: Arc<dyn I2cSlave>) -> crate::Result<()> {
        let mut inner = self.inner.lock();
        if inner.slaves.len() >= MAX_SLAVES {
            return Err(crate::Error::Config {
                at: alloc::string::String::from("i2c bus"),
                message: alloc::format!("an I2C bus in rsemu routes at most {MAX_SLAVES} devices"),
            });
        }
        inner.slaves.push(slave);
        Ok(())
    }

    /// How many devices are attached.
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.lock().slaves.len()
    }

    /// Whether nothing is attached.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// How many times two devices have acknowledged one non-general-call
    /// address.
    ///
    /// A machine-description bug rather than a bus event: on real copper both
    /// would drive SDA and the master would see an ACK, which is precisely why
    /// it is invisible unless something counts it.
    #[must_use]
    pub fn conflicts(&self) -> u32 {
        self.inner.lock().conflicts
    }

    /// What the bus is doing.
    #[must_use]
    pub fn state(&self) -> BusState {
        let inner = self.inner.lock();
        if !inner.started {
            return BusState::Free;
        }
        if inner.addressed.is_empty() {
            return BusState::Unaddressed;
        }
        BusState::Addressed {
            dir: inner.dir,
            responders: inner.addressed.len(),
        }
    }

    /// A START or repeated START, followed by an address.
    ///
    /// Every attached device is offered the address in attachment order — which
    /// is machine-file order, so it is deterministic (`CLAUDE.md`) — and the
    /// ones that acknowledge take the transfer. The returned [`Ack`] is the
    /// wired-AND of their answers: one acknowledgement is enough, which is what
    /// makes the general call work (§3.1.13).
    ///
    /// Devices addressed by a *previous* transfer that are not addressed by this
    /// one are told [`I2cSlave::stop`] first — §3.1.11's "until it receives a
    /// STOP condition (P) or a repeated START condition (Sr) followed by a
    /// different target address".
    pub fn start(&self, address: Address, dir: Direction) -> Ack {
        // Everything outward happens with the lock released; the lock is only
        // ever held to read or write our own bookkeeping (the re-entrancy
        // contract in `core::device`).
        let (previous, candidates) = {
            let inner = self.inner.lock();
            let previous: Vec<Arc<dyn I2cSlave>> = inner
                .addressed
                .iter()
                .filter_map(|i| inner.slaves.get(*i).cloned())
                .collect();
            (previous, inner.slaves.clone())
        };

        // Offer the address. For a ten-bit write the first byte is only a
        // header and its acknowledge is a separate question (§3.1.11), which
        // `ten_bit_header` answers; a device that says no there never sees the
        // full address.
        let mut taken: Vec<usize> = Vec::new();
        let mut answer = Ack::Nack;
        for (i, slave) in candidates.iter().enumerate() {
            if let (Address::Ten(_), Some(high)) = (address, address.ten_bit_high())
                && !slave.ten_bit_header(high)
            {
                continue;
            }
            if slave.address(address, dir).is_ack() {
                taken.push(i);
                answer = Ack::Ack;
            }
        }

        // A device that was addressed and is not any more ends its transaction.
        for (i, slave) in candidates.iter().enumerate() {
            if !taken.contains(&i) && previous.iter().any(|p| Arc::ptr_eq(p, slave)) {
                slave.stop();
            }
        }

        let mut inner = self.inner.lock();
        if taken.len() > 1 && address != GENERAL_CALL {
            inner.conflicts = inner.conflicts.saturating_add(1);
        }
        inner.addressed = taken;
        inner.started = true;
        inner.dir = dir;
        answer
    }

    /// The first byte of a ten-bit address, on its own (§3.1.11).
    ///
    /// A ten-bit address is two bytes and the *first* one is acknowledged
    /// separately, by every device whose top two address bits match — "It is
    /// possible that more than one device finds a match and generate an
    /// acknowledge (A1)". So a transactional master, which sends the header and
    /// the second byte as two [`BYTE_HALF_PERIODS`] events, asks this for the
    /// first and [`start`](I2cBus::start) for the second, and the slaves see
    /// exactly the calls a wired master's bit engine would make.
    ///
    /// Purely a query: nothing is addressed and no state changes, here or in
    /// any device.
    #[must_use]
    pub fn ten_bit_header(&self, high: u8) -> Ack {
        let slaves = self.inner.lock().slaves.clone();
        if slaves.iter().any(|s| s.ten_bit_header(high & 0b11)) {
            Ack::Ack
        } else {
            Ack::Nack
        }
    }

    /// One byte from the master to whoever is addressed.
    ///
    /// With nothing addressed the byte goes nowhere and the answer is
    /// [`Ack::Nack`] — an undriven, pulled-up SDA during the ninth clock, which
    /// is exactly what a master clocking a bus with no device on it sees
    /// (§3.1.6, reason 1). That is deliberately not an error: probing is an
    /// ordinary thing for firmware to do.
    pub fn write(&self, byte: u8) -> Ack {
        let addressed = self.addressed();
        let mut answer = Ack::Nack;
        for slave in addressed {
            answer = answer.merge(slave.write(byte));
        }
        answer
    }

    /// One byte from the addressed slave to the master.
    ///
    /// `ack` is what the master does with it, applied after the byte is taken,
    /// because on the wire the acknowledge is the ninth clock and comes after
    /// the eight data bits (§3.1.5).
    ///
    /// With nothing addressed the answer is `0xff`: the pull-up.
    pub fn read(&self, ack: Ack) -> u8 {
        let addressed = self.addressed();
        // Two transmitters is a short, not a wiring style; the general call is
        // write-only, so this can only be a machine bug and `conflicts` has
        // already counted it. The first device wins, deterministically.
        let Some(slave) = addressed.first() else {
            return 0xff;
        };
        let byte = slave.read();
        slave.read_ack(ack);
        byte
    }

    /// What a read would return, without transferring anything.
    ///
    /// The debug-access path. Nothing observable changes.
    #[must_use]
    pub fn peek(&self) -> u8 {
        self.addressed().first().map_or(0xff, |s| s.peek())
    }

    /// A STOP: the transaction ends and the bus goes free (§3.1.4).
    pub fn stop(&self) {
        let addressed = self.addressed();
        {
            let mut inner = self.inner.lock();
            inner.addressed.clear();
            inner.started = false;
        }
        for slave in addressed {
            slave.stop();
        }
    }

    /// Whether any attached device is holding SCL low (§3.1.9).
    ///
    /// The transactional link's stand-in for looking at the wire. Every
    /// attached device is asked, not just the addressed one, because a part
    /// that is still finishing the *previous* transaction stretches too.
    #[must_use]
    pub fn stretching(&self) -> bool {
        let slaves = self.inner.lock().slaves.clone();
        slaves.iter().any(|s| s.stretching())
    }

    /// The devices currently taking the transfer.
    fn addressed(&self) -> Vec<Arc<dyn I2cSlave>> {
        let inner = self.inner.lock();
        inner
            .addressed
            .iter()
            .filter_map(|i| inner.slaves.get(*i).cloned())
            .collect()
    }
}

impl Default for I2cBus {
    fn default() -> I2cBus {
        I2cBus::new()
    }
}

/// The named rendezvous: how a controller and its slaves find each other.
///
/// Modelled on [`crate::bus::spi::buses`] and, under it,
/// [`crate::host::chardev::ports`], and a seam for the same reason — a machine
/// description can hand two independently constructed devices only a *name*,
/// and `core::bus` (`ROADMAP.md` §4) does not exist yet. When it does, this
/// becomes its registry and nothing else here changes.
///
/// ```
/// # #[cfg(feature = "bus-i2c")] {
/// use rsemu::bus::i2c::buses;
/// use rsemu::core::HostObjects;
///
/// use std::sync::Arc;
///
/// let hosts = HostObjects::new();
/// let a = buses::open(&hosts, "i2c1").unwrap();
/// let b = buses::open(&hosts, "i2c1").unwrap();
/// assert!(Arc::ptr_eq(&a, &b), "the same name is the same bus");
///
/// // And a second build's `i2c1` is a second bus, not this one.
/// let elsewhere = HostObjects::new();
/// let c = buses::open(&elsewhere, "i2c1").unwrap();
/// assert!(!Arc::ptr_eq(&a, &c));
/// # }
/// ```
pub mod buses {
    use super::I2cBus;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;

    /// The kind an I²C bus is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::rendezvous("i2c-bus");

    /// The I²C bus `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<I2cBus>> {
        hosts.open(KIND, name, I2cBus::new)
    }

    /// The I²C bus `name` refers to in the build these properties are being
    /// read for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)` — acquiring a host object
    /// is allocation, and [`core::hosts`](crate::core::hosts) argues why. A
    /// `Props` that belongs to no build gets a private one, so a device a unit
    /// test constructed directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<I2cBus>> {
        props.host(KIND, name, I2cBus::new)
    }

    /// The I²C bus called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<I2cBus>>> {
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
}
