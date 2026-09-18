//! SWI: Microchip CryptoAuthentication's **single-wire interface**, one
//! self-clocked line with a UART at the host end.
//!
//! # The specification
//!
//! Two openly published Microchip data sheets, cited by section throughout:
//! **ATECC608B-TFLXTLS** (DS40002249B) chapter 8, *Single-Wire Interface*, and
//! **ATSHA204A** (DS40002025A) chapter 5, which documents the identical wire
//! and is the one that writes the token byte values and the bit order down.
//! Neither carries a confidentiality marking. **CryptoAuthLib was not opened**
//! — it is not permissively licensed, whatever `ROADMAP.md`'s issue text once
//! said — and no emulator was consulted (`ROADMAP.md` §1).
//!
//! # Why this is not [`crate::bus::i2c`] and not a GPIO
//!
//! SWI is one wire, `SDA`, and there is no clock on it at all: it is
//! *asynchronously timed*, and the receiver recovers the bit from pulse widths
//! (DS40002249B §8). There is no address, no acknowledge and no STOP. What
//! there is, is a UART:
//!
//! > The bit timings are designed to permit a standard UART running at
//! > 230.4 kBaud to transmit and receive the tokens efficiently. Each byte
//! > transmitted or received by the UART corresponds to a single bit received
//! > or transmitted by the device. (DS40002249B §8.1)
//!
//! > The UART must be set to seven data bits, no parity and one Stop bit.
//! > (DS40002249B §9.3.2, note 1)
//!
//! So **the unit of this fabric is one UART frame**, and one frame carries one
//! logical *bit*. That is why [`Token`] is a byte and [`Flag`] is not: a flag
//! is eight tokens, which is to say eight frames.
//!
//! ```text
//!   host UART TX ─┐                       ┌─ ATECC SDA
//!                 ├──────[ 1 kΩ ]─────────┤
//!   host UART RX ─┘                       └─ (SCL is a GPIO, §8.4)
//!
//!   0x7f ─► one       0x7d ─► zero       0x00 slowly ─► wake
//!   eight tokens, LSb first, make a flag or a byte of a group
//! ```
//!
//! # The hierarchy (DS40002249B §8)
//!
//! * **Tokens** — one data bit, or the wake event. [`Token`].
//! * **Flags** — eight tokens saying what comes next. [`Flag`].
//! * **Groups** — `count | packet… | CRC-16[2]`, following a command or
//!   transmit flag. Byte for byte *the same group the I²C face carries*
//!   (DS40002249B §4.1), which is the whole point: a device model implements
//!   the packet layer once.
//! * **Packets** — the command's own parameters, which belong to the device.
//!
//! # The wake is arithmetic, not a special call
//!
//! A wake is a low pulse of at least tWLO on the same wire, and a host makes
//! one by sending `0x00` *slowly*:
//!
//! > The Wake condition requires that either the system processor manually
//! > drive the SDA pin low for tWLO, or a data byte of 0x00 be transmitted at
//! > a clock rate sufficiently slow so that SDA is low for a minimum period of
//! > tWLO. (DS40002249B §7.1.1)
//!
//! At 7N1 a `0x00` frame holds the line low for the start bit plus seven data
//! bits — eight bit times. At the 230.4 kBaud the tokens use that is 34.7 µs,
//! which is **less** than the 60 µs tWLO the part wants, so a driver drops the
//! baud rate to wake (69.4 µs at 115.2 kBaud) and puts it back. This link
//! therefore carries the frame's **longest low time in nanoseconds** alongside
//! every token ([`SwiLink::low_ns`]) and lets the device decide what is a wake,
//! which is what the silicon does. Nothing here hard-codes tWLO; that is a
//! parameter of the part, not of the wire.
//!
//! # What this fabric does not model, deliberately
//!
//! * **The echo.** A real host ties TX and RX together through a resistor, so
//!   its UART receives everything it sends (§8.5). That is a property of the
//!   *wiring*, and every driver discards it. [`SwiLink::recv`] answers what the
//!   **device** drove, so a test reads what the part said rather than what it
//!   just asked.
//! * **Edges.** There is no wired, bit-banged front end here the way
//!   [`crate::bus::i2c::wires`] is one, because SWI's own data sheet defines
//!   the timings as UART frames. A guest that bit-bangs SWI through GPIO would
//!   need one; nothing in the tree does yet, and inventing the pulse-width
//!   decoder before something needs it would be guessing.
//! * **Time.** Like [`crate::bus::i2c::I2cBus`], this fabric holds no clock:
//!   *time belongs to the host*, which is the only end with a baud generator.
//!   A UART model that drives this link charges its own scheduler a frame time
//!   per [`SwiLink::send`].
//!
//! # Finding each other
//!
//! As in [`crate::bus::i2c::buses`]: a named rendezvous table, [`links`], since
//! a machine description can only hand two independently constructed objects a
//! *name*. There is no in-tree UART that speaks this yet — `host::chardev` is a
//! byte stream, not a frame seam (`src/dev/stm32/usart.rs` says why its TX is
//! not a wire) — so today the host end is a test or a downstream driver. The
//! device end is complete.

#[cfg(test)]
mod tests;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::sync::{LockRank, Mutex};

// ---------------------------------------------------------------------------
// Where this module sits in the lock ladder
// ---------------------------------------------------------------------------

/// The rank a [`SwiLink`]'s own state takes.
///
/// The same band [`crate::bus::i2c::FABRIC_RANK`] and
/// [`crate::bus::spi::FABRIC_RANK`] sit in, one step further along so all three
/// can appear in one machine without their ladders colliding, and — for the
/// reason those two record — **not** [`LockRank::BUS`], which a CPU core is
/// already holding by the time an MMIO write reaches a device.
///
/// ```text
///   CPU session (BUS 0x4000)
///     → SwiLink (0x4600, here)
///       → the device's own state (DEVICE 0x5000)
/// ```
pub const FABRIC_RANK: LockRank = LockRank::new(0x4600);

// ---------------------------------------------------------------------------
// The wire's symbols
// ---------------------------------------------------------------------------

/// The token rate the part's timings are designed for (DS40002249B §8.1).
pub const BAUD: u64 = 230_400;

/// Bit times in one frame: one start, seven data, one stop (DS40002249B §9.3.2
/// note 1 — "seven data bits, no parity and one Stop bit").
pub const FRAME_BITS: u64 = 9;

/// How many tokens carry one byte, flag or group byte alike (DS40002025A §5,
/// "8 I/O tokens would be needed to create a single byte of data").
pub const TOKEN_BITS: u32 = 8;

/// One UART frame on the wire: a single logical bit, or the wake event.
///
/// The values are DS40002025A Table 5-1's, which is the data sheet that writes
/// the UART side down: `0x7F` a one, `0x7D` a zero, `0x00` slowly a wake. Only
/// the low **seven** bits are transmitted at 7N1, so bit 7 is not on the wire
/// and is ignored here.
///
/// An extensible enumeration rather than a Rust `enum` (`CLAUDE.md`, "Type
/// conventions"): the data sheet reserves every other value, and a model that
/// could not *hold* a reserved one could not be handed a malformed stream.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Token(pub u8);

impl Token {
    /// A logic zero (DS40002025A Table 5-1): start low, one high bit, then the
    /// second low pulse the receiver measures.
    pub const ZERO: Token = Token(0x7d);

    /// A logic one (DS40002025A Table 5-1): the start pulse and nothing else.
    pub const ONE: Token = Token(0x7f);

    /// The wake token (DS40002025A Table 5-1): every bit of the frame low but
    /// the stop bit, which is a wake **only if the baud rate makes it long
    /// enough** — see the module docs.
    pub const WAKE: Token = Token(0x00);

    /// The token carrying `bit`.
    #[must_use]
    #[inline]
    pub const fn of(bit: bool) -> Token {
        if bit { Token::ONE } else { Token::ZERO }
    }

    /// The bit this token carries, or `None` for anything the data sheet does
    /// not define — which is not an error here, only something the *device*
    /// has an answer to (DS40002249B §8.3.1).
    #[must_use]
    #[inline]
    pub const fn bit(self) -> Option<bool> {
        match self.0 & 0x7f {
            0x7f => Some(true),
            0x7d => Some(false),
            _ => None,
        }
    }

    /// The longest run of bit times this frame holds the line low, at 7N1.
    ///
    /// Start bit, then the seven data bits least-significant first, then the
    /// stop bit which is always high. This is the only thing that distinguishes
    /// a wake from a data token, and it is arithmetic on the frame rather than
    /// a table: `0x00` is eight, every legal data token is one.
    #[must_use]
    pub const fn low_bits(self) -> u64 {
        // The start bit is low, and is where the run begins.
        let mut run: u64 = 1;
        let mut longest: u64 = 1;
        let mut i = 0;
        while i < 7 {
            if (self.0 >> i) & 1 == 0 {
                run += 1;
                if run > longest {
                    longest = run;
                }
            } else {
                run = 0;
            }
            i += 1;
        }
        longest
    }

    /// How long this frame holds the line low, in nanoseconds, at `baud`.
    ///
    /// Integer nanoseconds, never seconds in a float (`CLAUDE.md`,
    /// "Determinism"). A `baud` of zero answers zero rather than dividing by
    /// it; [`SwiLink::set_baud`] refuses one in the first place.
    #[must_use]
    pub const fn low_ns(self, baud: u64) -> u64 {
        if baud == 0 {
            return 0;
        }
        self.low_bits().saturating_mul(1_000_000_000) / baud
    }
}

impl fmt::Debug for Token {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.bit() {
            Some(true) => f.write_str("Token::ONE"),
            Some(false) => f.write_str("Token::ZERO"),
            None if self.0 == 0 => f.write_str("Token::WAKE"),
            None => write!(f, "Token({:#04x})", self.0),
        }
    }
}

/// The eight-token byte that opens every transaction (DS40002249B Table 8-1).
///
/// > The system is always host, so before any I/O transaction, the system must
/// > send an eight bit flag to the device to indicate the I/O operation that
/// > will be subsequently performed.
///
/// An extensible enumeration for the same reason [`Token`] is one: "All other
/// values are reserved and must not be used", and a model is handed them.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Flag(pub u8);

impl Flag {
    /// `0x77` — a command group follows (DS40002249B Table 8-1).
    pub const COMMAND: Flag = Flag(0x77);

    /// `0x88` — turn the bus around; the device transmits its response group
    /// after tTURNAROUND (DS40002249B Table 8-1, §8.2 "Transmit Flag").
    pub const TRANSMIT: Flag = Flag(0x88);

    /// `0xBB` — go to idle: the I/O buffer is flushed, `TempKey` survives
    /// (DS40002249B Table 8-1, §8.2 "Idle Flag").
    pub const IDLE: Flag = Flag(0xbb);

    /// `0xCC` — go to sleep: a complete reset of the volatile state
    /// (DS40002249B Table 8-1, §8.2 "Sleep Flag").
    pub const SLEEP: Flag = Flag(0xcc);

    /// Every flag the data sheet defines, in value order.
    pub const ALL: [Flag; 4] = [Flag::COMMAND, Flag::TRANSMIT, Flag::IDLE, Flag::SLEEP];

    /// What this flag is called, or `"reserved"`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 {
            0x77 => "command",
            0x88 => "transmit",
            0xbb => "idle",
            0xcc => "sleep",
            _ => "reserved",
        }
    }
}

impl fmt::Debug for Flag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Flag::{}({:#04x})", self.name(), self.0)
    }
}

/// The tokens that carry `byte`, least-significant bit first.
///
/// > Flags are always transmitted LSb first. (DS40002025A §5)
///
/// The group's bytes ride the same eight-token mechanism, so they go the same
/// way round — which is also the order this family's CRC-16 feeds its input
/// (DS40002249B §4.1), as a wire that sends LSb first would make natural.
#[must_use]
pub fn tokens_of(byte: u8) -> [Token; TOKEN_BITS as usize] {
    let mut out = [Token::ZERO; TOKEN_BITS as usize];
    let mut i = 0;
    while i < TOKEN_BITS as usize {
        out[i] = Token::of((byte >> i) & 1 != 0);
        i += 1;
    }
    out
}

// ---------------------------------------------------------------------------
// The device seam
// ---------------------------------------------------------------------------

/// A part listening on a single wire.
///
/// One call per UART frame in each direction, because on this bus a frame *is*
/// a bit. A device implements this beside its [`crate::bus::i2c::I2cSlave`]
/// face and shares one packet layer between them, which is what the ATECC
/// model does.
pub trait SwiSlave: Send + Sync + fmt::Debug {
    /// A frame arrived from the host.
    ///
    /// `low_ns` is how long that frame held the line low, which is the only
    /// thing distinguishing a wake from a data token (DS40002249B §8.1). The
    /// device compares it against its own tWLO; the wire does not know one.
    fn token(&self, token: Token, low_ns: u64);

    /// The next frame the device drives back, if it is transmitting.
    ///
    /// `None` means the line is idle — which, to a real host, is a UART that
    /// receives nothing.
    fn next_token(&self) -> Option<Token>;

    /// The same, without consuming it.
    ///
    /// The `MemAttrs::debug` rule as it applies to a bus (`CLAUDE.md`,
    /// "Devices"): a monitor's look must not advance the response.
    fn peek_token(&self) -> Option<Token> {
        None
    }
}

// ---------------------------------------------------------------------------
// The fabric
// ---------------------------------------------------------------------------

/// One single-wire link: a host UART and the one part on the other end.
///
/// **Point to point.** There is no address on this wire and no acknowledge, so
/// two parts on one line could not be told apart; [`attach`](SwiLink::attach)
/// refuses a second.
pub struct SwiLink {
    inner: Mutex<Inner>,
}

/// Everything the link tracks, under one lock.
struct Inner {
    slave: Option<Arc<dyn SwiSlave>>,
    /// The host UART's baud rate, which decides how long a frame holds the
    /// line low and therefore what counts as a wake.
    baud: u64,
    /// Frames the host has sent, for a machine that wants to see the traffic.
    sent: u64,
}

impl fmt::Debug for SwiLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("SwiLink");
        match self.inner.try_lock() {
            Some(inner) => s
                .field("attached", &inner.slave.is_some())
                .field("baud", &inner.baud)
                .field("sent", &inner.sent),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

impl Default for SwiLink {
    fn default() -> SwiLink {
        SwiLink::new()
    }
}

impl SwiLink {
    /// An empty link at the data sheet's [`BAUD`].
    #[must_use]
    pub fn new() -> SwiLink {
        SwiLink {
            inner: Mutex::with_rank(
                FABRIC_RANK,
                Inner {
                    slave: None,
                    baud: BAUD,
                    sent: 0,
                },
            ),
        }
    }

    /// Put the one part on the wire.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if something is already on it.
    pub fn attach(&self, slave: Arc<dyn SwiSlave>) -> Result<()> {
        let mut inner = self.inner.lock();
        if inner.slave.is_some() {
            return Err(Error::Config {
                at: alloc::string::String::from("swi link"),
                message: alloc::string::String::from(
                    "a single-wire link carries one device: there is no address on this wire, so \
                     a second part could never be told from the first",
                ),
            });
        }
        inner.slave = Some(slave);
        Ok(())
    }

    /// Whether a part is on the wire.
    #[must_use]
    pub fn is_attached(&self) -> bool {
        self.inner.lock().slave.is_some()
    }

    /// The host UART's baud rate.
    #[must_use]
    pub fn baud(&self) -> u64 {
        self.inner.lock().baud
    }

    /// Set the host UART's baud rate.
    ///
    /// This is a real knob, not a formality: a host wakes the part by dropping
    /// to a rate at which a `0x00` frame is low for tWLO and putting it back
    /// afterwards (module docs, DS40002249B §7.1.1).
    ///
    /// # Errors
    ///
    /// [`Error::Config`] for a rate of zero.
    pub fn set_baud(&self, baud: u64) -> Result<()> {
        if baud == 0 {
            return Err(Error::Config {
                at: alloc::string::String::from("swi link"),
                message: alloc::string::String::from(
                    "`baud` is 0; a self-clocked wire is timed by the host's UART and a rate of \
                     zero has no frame",
                ),
            });
        }
        self.inner.lock().baud = baud;
        Ok(())
    }

    /// How long `token` holds the line low at this link's current baud rate.
    #[must_use]
    pub fn low_ns(&self, token: Token) -> u64 {
        token.low_ns(self.inner.lock().baud)
    }

    /// How many frames the host has transmitted.
    #[must_use]
    pub fn sent(&self) -> u64 {
        self.inner.lock().sent
    }

    /// The host UART transmits one frame.
    ///
    /// The lock is released before the device is called (`CLAUDE.md`, the
    /// re-entrancy contract): the part reaches its own clock domain from
    /// inside, and a link that still held its lock would be in the way.
    pub fn send(&self, token: Token) {
        let (slave, baud) = {
            let mut inner = self.inner.lock();
            inner.sent = inner.sent.saturating_add(1);
            (inner.slave.clone(), inner.baud)
        };
        if let Some(slave) = slave {
            slave.token(token, token.low_ns(baud));
        }
    }

    /// The host UART receives one frame, if the part is driving one.
    #[must_use]
    pub fn recv(&self) -> Option<Token> {
        let slave = self.inner.lock().slave.clone();
        slave.and_then(|slave| slave.next_token())
    }

    /// Whether the part has a frame waiting, without taking it.
    #[must_use]
    pub fn pending(&self) -> bool {
        let slave = self.inner.lock().slave.clone();
        slave.and_then(|slave| slave.peek_token()).is_some()
    }

    /// Send a flag: eight tokens, least-significant bit first (DS40002025A §5).
    pub fn flag(&self, flag: Flag) {
        for token in tokens_of(flag.0) {
            self.send(token);
        }
    }

    /// Send the bytes of a group, eight tokens each.
    ///
    /// The caller sends [`Flag::COMMAND`] first; this is the group itself,
    /// which is byte for byte the I²C face's packet (DS40002249B §4.1).
    pub fn write_group(&self, bytes: &[u8]) {
        for byte in bytes {
            for token in tokens_of(*byte) {
                self.send(token);
            }
        }
    }

    /// Read a whole group back: the count byte says how long it is.
    ///
    /// `None` if the part stops driving part-way through, which is what a real
    /// host sees as its UART receiving nothing more — the device is busy, out
    /// of synchronisation, or asleep (DS40002249B §8.3.2).
    #[must_use]
    pub fn read_group(&self) -> Option<Vec<u8>> {
        let count = self.read_byte()?;
        let mut group = Vec::with_capacity(usize::from(count));
        group.push(count);
        for _ in 1..count {
            group.push(self.read_byte()?);
        }
        Some(group)
    }

    /// Eight tokens back from the part, as a byte.
    ///
    /// `None` if it stops driving, or drives something that is neither a one
    /// nor a zero — which it never does, but the type says so.
    #[must_use]
    pub fn read_byte(&self) -> Option<u8> {
        let mut byte = 0u8;
        for i in 0..TOKEN_BITS {
            let bit = self.recv()?.bit()?;
            byte |= u8::from(bit) << i;
        }
        Some(byte)
    }
}

// ---------------------------------------------------------------------------
// The rendezvous
// ---------------------------------------------------------------------------

/// The named table a host and a part meet through.
///
/// Exactly [`crate::bus::i2c::buses`]'s shape, for exactly its reason: a
/// machine description can hand two independently constructed objects a *name*
/// and nothing else. Both ends write `link = "swi0"`.
pub mod links {
    use super::SwiLink;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;

    /// The kind an SWI link is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::rendezvous("swi-link");

    /// The link `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<SwiLink>> {
        hosts.open(KIND, name, SwiLink::new)
    }

    /// The link `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs to
    /// no build gets a private one, so a device a unit test constructed
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<SwiLink>> {
        props.host(KIND, name, SwiLink::new)
    }

    /// The link called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<SwiLink>>> {
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
