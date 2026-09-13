//! The Microchip ATECC508A / ATECC608A / ATECC608B CryptoAuthentication
//! secure element, over I²C.
//!
//! The part behind the "secure element" footprint on a very large number of
//! boards — Arduino, Adafruit, ESP32, STM32, LoRaWAN modules, TPM-lite roles —
//! and the first device in this tree whose entire purpose is cryptography.
//! Firmware that talks to one cannot get past its provisioning check without a
//! model, which is what this is.
//!
//! # Source
//!
//! The Microchip **ATECC508A** (DS20005927), **ATECC608A** (DS40001977) and
//! **ATECC608B** (DS40002239) datasheets, cited by section throughout, plus
//! FIPS 180-4 (SHA-256), RFC 2104 (HMAC) and FIPS 186-4 (ECDSA over P-256) for
//! the primitives. **No emulator and no vendor library was consulted**
//! (`ROADMAP.md` §1): CryptoAuthLib is permissively licensed and would have
//! been readable, but the datasheet is the better source and is the only one
//! this file is written from.
//!
//! # What is modelled
//!
//! * **The I²C transport** (§7.1): the seven-bit address the `I2C_Address`
//!   config byte selects, and the *word address* byte that opens every write —
//!   `0x00` Reset, `0x01` Sleep, `0x02` Idle, `0x03` Command.
//! * **The power state machine** (§6): sleep, idle and awake, the wake token
//!   `04 11 33 43` a freshly woken part answers with, and the **watchdog**
//!   (§6.3) that puts the part back to sleep by itself after tWATCHDOG whether
//!   or not anybody is talking to it. A model without that watchdog desyncs
//!   long-running firmware, which is why it is here rather than "later".
//! * **The packet layer** (§9.1): `count | opcode | param1 | param2[2] | data…
//!   | CRC-16[2]` in, `count | data… | CRC-16[2]` out, with the datasheet's own
//!   CRC-16 — polynomial `0x8005`, each byte fed least-significant bit first.
//!   The wake token is the datasheet's own worked example of it:
//!   `CRC(04 11) = 33 43`, which [`crc16`]'s test asserts.
//! * **The zones** (§2): a 128-byte configuration zone, a 64-byte OTP zone and
//!   a data zone of sixteen slots (36 bytes each for 0–7, 416 for slot 8, 72
//!   for 9–15), the `SlotConfig`/`KeyConfig` words that govern every one of
//!   them, and the two-stage lock — configuration first, then data and OTP —
//!   that decides which commands are legal at all.
//! * **The commands** (§9.2): `Read`, `Write` (clear and encrypted), `Lock`,
//!   `UpdateExtra`, `Info`, `Counter`, `Random`, `Nonce`, `GenDig`, `MAC`,
//!   `CheckMac`, `HMAC`, `SHA`, `GenKey`, `PrivWrite`, `Sign`, `Verify`,
//!   `ECDH`, `SelfTest`, `Pause`, and the 608's `AES`.
//!
//! # What is not
//!
//! * **The SWI single-wire transport** (§7.2). It is not a mode of this model
//!   and pretending otherwise would be worse than the gap: SWI carries each
//!   *logical bit* as one 230.4 kbaud UART token (`0x7F` = 1, `0x7D` = 0) with
//!   flag bytes `0x88` command, `0x77` transmit, `0xCC` idle, `0xBB` sleep, and
//!   a wake is a ≥ 60 µs low pulse on the same wire. What it needs is a
//!   *transport seam that does not exist yet*: a one-wire, self-clocked link
//!   with a UART on the other end, which is neither [`crate::bus::i2c`] nor a
//!   GPIO. Everything above the transport in this file — the packet layer, the
//!   state machine and every command — is already independent of it
//!   (the command engine takes a packet and returns a packet), so SWI is a new
//!   front end plus a link type, not a second device.
//! * `DeriveKey`, `KDF`, `SecureBoot` and the 608's `KDF`-adjacent modes. They
//!   answer a parse error, and the module says so rather than inventing a
//!   message layout.
//! * The `VolatileKey`, `SecureBootPersistent` and `IO protection key` (608)
//!   features, and the encrypted-read path of `Read` — an `IsSecret` slot is
//!   refused rather than returned under a session key.
//! * **The monotonic counters' representation.** They are values here, and on
//!   the die they are configuration bytes 52–67 in a packed encoding designed
//!   so that a count can only go up as bits are burned. A host reads them with
//!   `Counter`, which is exact; a host that reads *configuration bytes 52–67*
//!   sees zeros rather than that encoding.
//! * **`ChipMode`'s watchdog selection.** tWATCHDOG is the `watchdog-ticks`
//!   property rather than a field decoded out of the configuration zone,
//!   because the period is in *this board's* clock domain and a configuration
//!   byte cannot know what that domain runs at. The default is the datasheet's
//!   1.3 s for a 1 MHz domain.
//!
//! # Keys, randomness and determinism
//!
//! **Every byte this part calls random comes from the board's seed**
//! (`CLAUDE.md`, "Determinism"). A `seed` property is mixed with the device's
//! instance path at realize through
//! [`crate::core::rand::derive_seed`], and the resulting
//! [`Stream`] is what `Random`, `Nonce`, `GenKey` and `PrivWrite`'s key
//! material draw from; the stream's position is saved in the snapshot. Two runs
//! of one machine file therefore generate the *same* key pair and hash to the
//! same state — which is the point, and is also exactly why a guest that wants
//! unpredictable bytes must not ask an emulator for them.
//!
//! ECDSA signing is deterministic for a second, independent reason: it is
//! RFC 6979, which `purecrypto` implements, so a signature is a pure function
//! of the key and the digest with no nonce to record.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). Like [`super::at24c`] this is a
//! *lazily advanced* device (`ROADMAP.md` §4.2): it holds its own tick,
//! publishes the tick its current command finishes on, and is caught up before
//! anything touches it. A command takes its datasheet execution time, during
//! which the part **NACKs its own address** — which is what makes a driver's
//! poll loop behave the way the datasheet's flow chart says.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use purecrypto::cipher::{Aes128, BlockCipher};
use purecrypto::ec::ecdh::EcdhPrivateKey;
use purecrypto::ec::ecdsa::{EcdsaPrivateKey, EcdsaPublicKey, Signature};
use purecrypto::hash::{Digest, HmacSha256, Sha256};

use crate::bus::i2c::wires::{SlaveWires, SlaveWiresState, pin as line};
use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::rand::{Stream, derive_seed};
use crate::core::sched::LazyHandle;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::WireSource;
use crate::machine::realize::Instance;

#[cfg(test)]
mod tests;

/// The class name a machine description writes.
const CLASS_NAME: &str = "atmel.atecc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// The wire-level constants (§7.1, §9.1)
// ---------------------------------------------------------------------------

/// The seven-bit address a part with the default `I2C_Address` of `0xC0`
/// answers (§2.2.4: the byte holds the address in bits 7:1).
pub const DEFAULT_ADDRESS: u8 = 0x60;

/// The address a wake pulse looks like on this link.
///
/// On real hardware a wake is **not** an address at all: it is SDA held low for
/// at least tWLO (§6.1). Firmware produces that by addressing `0x00` at a low
/// bit rate, which is precisely what this model watches for — an address phase
/// to `0x00` is the wake token, and the part answers it with a NACK, as the
/// datasheet says it does.
pub const WAKE_ADDRESS: u8 = 0x00;

/// The word address that resets the I/O buffer's read pointer (§7.1.1).
pub const WORD_RESET: u8 = 0x00;
/// The word address that puts the part to sleep (§6.2).
pub const WORD_SLEEP: u8 = 0x01;
/// The word address that puts the part in idle mode (§6.2).
pub const WORD_IDLE: u8 = 0x02;
/// The word address that opens a command packet (§9.1).
pub const WORD_COMMAND: u8 = 0x03;

/// The four bytes a freshly woken part answers a read with (§6.1).
pub const WAKE_TOKEN: [u8; 4] = [0x04, 0x11, 0x33, 0x43];

// Status bytes (§9.3, Table "Status/Error Codes").

/// The command completed.
pub const STATUS_OK: u8 = 0x00;
/// `CheckMac` or `Verify` failed to match.
pub const STATUS_MISCOMPARE: u8 = 0x01;
/// The packet was legal but its contents were not.
pub const STATUS_PARSE: u8 = 0x03;
/// An ECC computation could not be completed.
pub const STATUS_ECC_FAULT: u8 = 0x05;
/// A self test failed (608).
pub const STATUS_SELF_TEST: u8 = 0x07;
/// The command could not be executed in the current state.
pub const STATUS_EXEC: u8 = 0x0f;
/// The part is awake and has done nothing yet.
pub const STATUS_AFTER_WAKE: u8 = 0x11;
/// The watchdog would expire before the command could finish (§6.3).
pub const STATUS_WATCHDOG: u8 = 0xee;
/// The packet's CRC was wrong, or it was malformed (§9.1.3).
pub const STATUS_CRC: u8 = 0xff;

// Opcodes (§9.2).

/// `Pause` (508A).
pub const OP_PAUSE: u8 = 0x01;
/// `Read`.
pub const OP_READ: u8 = 0x02;
/// `MAC`.
pub const OP_MAC: u8 = 0x08;
/// `HMAC` (508A).
pub const OP_HMAC: u8 = 0x11;
/// `Write`.
pub const OP_WRITE: u8 = 0x12;
/// `GenDig`.
pub const OP_GENDIG: u8 = 0x15;
/// `Nonce`.
pub const OP_NONCE: u8 = 0x16;
/// `Lock`.
pub const OP_LOCK: u8 = 0x17;
/// `Random`.
pub const OP_RANDOM: u8 = 0x1b;
/// `DeriveKey` — not modelled.
pub const OP_DERIVEKEY: u8 = 0x1c;
/// `UpdateExtra`.
pub const OP_UPDATEEXTRA: u8 = 0x20;
/// `Counter`.
pub const OP_COUNTER: u8 = 0x24;
/// `CheckMac`.
pub const OP_CHECKMAC: u8 = 0x28;
/// `Info`.
pub const OP_INFO: u8 = 0x30;
/// `GenKey`.
pub const OP_GENKEY: u8 = 0x40;
/// `Sign`.
pub const OP_SIGN: u8 = 0x41;
/// `ECDH`.
pub const OP_ECDH: u8 = 0x43;
/// `Verify`.
pub const OP_VERIFY: u8 = 0x45;
/// `PrivWrite`.
pub const OP_PRIVWRITE: u8 = 0x46;
/// `SHA`.
pub const OP_SHA: u8 = 0x47;
/// `AES` (608).
pub const OP_AES: u8 = 0x51;
/// `KDF` (608) — not modelled.
pub const OP_KDF: u8 = 0x56;
/// `SelfTest` (608).
pub const OP_SELFTEST: u8 = 0x77;
/// `SecureBoot` (608) — not modelled.
pub const OP_SECUREBOOT: u8 = 0x80;

// ---------------------------------------------------------------------------
// Geometry (§2)
// ---------------------------------------------------------------------------

/// The configuration zone, in bytes (§2.2).
pub const CONFIG_BYTES: usize = 128;
/// The OTP zone, in bytes (§2.3).
pub const OTP_BYTES: usize = 64;
/// How many slots the data zone holds (§2.1).
pub const SLOTS: usize = 16;

/// Each slot's size in bytes (§2.1, Table 2-3).
pub const SLOT_SIZES: [u64; SLOTS] = [
    36, 36, 36, 36, 36, 36, 36, 36, 416, 72, 72, 72, 72, 72, 72, 72,
];

/// The data zone, in bytes: the sum of [`SLOT_SIZES`].
pub const DATA_BYTES: usize = 1208;

/// The byte offset of `SlotConfig[0]` in the configuration zone (§2.2).
pub const SLOTCONFIG_OFFSET: usize = 20;
/// The byte offset of `KeyConfig[0]` in the configuration zone (§2.2).
pub const KEYCONFIG_OFFSET: usize = 96;
/// `UserExtra` (§2.2).
pub const USEREXTRA_OFFSET: usize = 84;
/// `Selector` (§2.2).
pub const SELECTOR_OFFSET: usize = 85;
/// `LockValue`: the data and OTP zones' lock byte (§2.2).
pub const LOCKVALUE_OFFSET: usize = 86;
/// `LockConfig`: the configuration zone's lock byte (§2.2).
pub const LOCKCONFIG_OFFSET: usize = 87;
/// `SlotLocked`, a bit per slot, `1` meaning unlocked (§2.2).
pub const SLOTLOCKED_OFFSET: usize = 88;

/// What a lock byte reads as while the zone is unlocked (§2.2).
pub const UNLOCKED: u8 = 0x55;
/// What it reads as once the zone is locked.
pub const LOCKED: u8 = 0x00;

/// The largest value a monotonic counter reaches: 2²¹ − 1 (§9.2, `Counter`).
pub const COUNTER_MAX: u32 = 2_097_151;

/// The number of bytes the first fifteen configuration bytes occupy — the
/// serial number, revision and interface bytes no `Write` may ever touch
/// (§2.2: "bytes 0 through 15 … are not writable").
const CONFIG_FIXED: usize = 16;

// ---------------------------------------------------------------------------
// Timing (§9.4)
// ---------------------------------------------------------------------------

/// tWATCHDOG in ticks of this device's clock domain (§6.3).
///
/// 1.3 s, which is the datasheet's figure for a `ChipMode.WatchdogTimeout` of
/// zero, expressed for a 1 MHz domain — the same convention
/// [`super::at24c`]'s `write-ticks` uses, and for the same reason: a device
/// does not own a frequency, so the number is in *its board's* ticks.
pub const DEFAULT_WATCHDOG_TICKS: u64 = 1_300_000;

/// How long each command takes, in microseconds.
///
/// These are the datasheet's per-command **maximum** execution times (§9.4),
/// rounded to the millisecond, and the maximum is the right figure for a model
/// because it is the only one a driver is entitled to assume: firmware either
/// waits that long or polls, and both work against it. The table is indexed by
/// opcode through [`exec_us`].
const EXEC_US: &[(u8, u64)] = &[
    (OP_PAUSE, 3_000),
    (OP_READ, 1_000),
    (OP_MAC, 14_000),
    (OP_HMAC, 23_000),
    (OP_WRITE, 26_000),
    (OP_GENDIG, 11_000),
    (OP_NONCE, 29_000),
    (OP_LOCK, 32_000),
    (OP_RANDOM, 23_000),
    (OP_UPDATEEXTRA, 10_000),
    (OP_COUNTER, 20_000),
    (OP_CHECKMAC, 13_000),
    (OP_INFO, 1_000),
    (OP_GENKEY, 115_000),
    (OP_SIGN, 60_000),
    (OP_ECDH, 58_000),
    (OP_VERIFY, 72_000),
    (OP_PRIVWRITE, 48_000),
    (OP_SHA, 9_000),
    (OP_AES, 27_000),
    (OP_SELFTEST, 625_000),
];

/// The execution time of `op`, in microseconds; the `Read` figure for anything
/// the table does not name, which is the smallest the part ever takes.
fn exec_us(op: u8) -> u64 {
    EXEC_US
        .iter()
        .find(|(opcode, _)| *opcode == op)
        .map_or(1_000, |(_, us)| *us)
}

// ---------------------------------------------------------------------------
// The part
// ---------------------------------------------------------------------------

/// Which member of the family this object is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Part {
    /// The ATECC508A (DS20005927).
    Ecc508a,
    /// The ATECC608A (DS40001977).
    Ecc608a,
    /// The ATECC608B (DS40002239).
    Ecc608b,
}

impl Part {
    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Part::Ecc508a => "atecc508a",
            Part::Ecc608a => "atecc608a",
            Part::Ecc608b => "atecc608b",
        }
    }

    /// Parse that spelling.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Part> {
        match name {
            "atecc508a" => Some(Part::Ecc508a),
            "atecc608a" => Some(Part::Ecc608a),
            "atecc608b" => Some(Part::Ecc608b),
            _ => None,
        }
    }

    /// `RevNum`, configuration bytes 4–7 (§2.2.2), and what `Info(Revision)`
    /// answers.
    #[must_use]
    pub const fn revnum(self) -> [u8; 4] {
        match self {
            Part::Ecc508a => [0x00, 0x00, 0x50, 0x00],
            Part::Ecc608a => [0x00, 0x00, 0x60, 0x02],
            Part::Ecc608b => [0x00, 0x00, 0x60, 0x03],
        }
    }

    /// Whether this part has the 608's AES engine (§9.2, `AES`).
    #[must_use]
    pub const fn has_aes(self) -> bool {
        matches!(self, Part::Ecc608a | Part::Ecc608b)
    }

    /// Whether this part has the 508A's `HMAC` command, which the 608 dropped.
    #[must_use]
    pub const fn has_hmac(self) -> bool {
        matches!(self, Part::Ecc508a)
    }
}

/// What the part is doing between wakes (§6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Power {
    /// Asleep: answers nothing, and every volatile register is gone (§6.2).
    #[default]
    Sleep,
    /// Idle: answers nothing until woken, but `TempKey` and the RNG seed
    /// survive (§6.2).
    Idle,
    /// Awake, with the watchdog running.
    Awake,
}

impl Power {
    const fn code(self) -> u8 {
        match self {
            Power::Sleep => 0,
            Power::Idle => 1,
            Power::Awake => 2,
        }
    }

    const fn from_code(code: u8) -> Power {
        match code {
            1 => Power::Idle,
            2 => Power::Awake,
            _ => Power::Sleep,
        }
    }
}

/// Where the current I²C transaction is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// Not addressed.
    #[default]
    Idle,
    /// Addressed for a write; the next byte is the word address (§7.1).
    WantWord,
    /// A word address other than `Command` arrived; nothing more is expected.
    AfterWord,
    /// Collecting a command packet (§9.1).
    Command,
    /// Addressed for a read; handing out the response buffer.
    Reading,
}

impl Phase {
    const fn code(self) -> u8 {
        match self {
            Phase::Idle => 0,
            Phase::WantWord => 1,
            Phase::AfterWord => 2,
            Phase::Command => 3,
            Phase::Reading => 4,
        }
    }

    const fn from_code(code: u8) -> Phase {
        match code {
            1 => Phase::WantWord,
            2 => Phase::AfterWord,
            3 => Phase::Command,
            4 => Phase::Reading,
            _ => Phase::Idle,
        }
    }
}

/// Which zone a command names (§2, and `param1` bits 1:0 of `Read`/`Write`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Zone {
    Config,
    Otp,
    Data,
}

impl Zone {
    const fn from_bits(bits: u8) -> Option<Zone> {
        match bits & 0x03 {
            0 => Some(Zone::Config),
            1 => Some(Zone::Otp),
            2 => Some(Zone::Data),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// CRC-16 (§9.1.3)
// ---------------------------------------------------------------------------

/// The datasheet's CRC-16, over `data`, little-endian as it goes on the wire.
///
/// Polynomial `0x8005`, register initialised to zero, each byte fed **least
/// significant bit first** (§9.1.3's own pseudo-code). The datasheet supplies
/// its own test vector without saying so: the wake token is `04 11` followed by
/// its CRC, `33 43`.
#[must_use]
pub fn crc16(data: &[u8]) -> [u8; 2] {
    let mut reg: u16 = 0;
    for byte in data {
        for shift in 0..8 {
            let data_bit = (*byte >> shift) & 1;
            let crc_bit = (reg >> 15) as u8 & 1;
            reg <<= 1;
            if data_bit != crc_bit {
                reg ^= 0x8005;
            }
        }
    }
    [(reg & 0xff) as u8, (reg >> 8) as u8]
}

/// Build a command packet: `count | opcode | param1 | param2[2] | data | CRC`.
///
/// The framing of §9.1.1, in one place, so a driver, a test and this model
/// cannot disagree about it. `param2` goes out little-endian.
#[must_use]
pub fn command(opcode: u8, param1: u8, param2: u16, data: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(data.len() + 7);
    packet.push((data.len() + 7) as u8);
    packet.push(opcode);
    packet.push(param1);
    packet.push((param2 & 0xff) as u8);
    packet.push((param2 >> 8) as u8);
    packet.extend_from_slice(data);
    let crc = crc16(&packet);
    packet.extend_from_slice(&crc);
    packet
}

/// Wrap `body` in a response packet: `count | body | CRC` (§9.1.2).
fn response(body: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(body.len() + 3);
    packet.push((body.len() + 3) as u8);
    packet.extend_from_slice(body);
    let crc = crc16(&packet);
    packet.extend_from_slice(&crc);
    packet
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An ATECC508A/608A/608B on an I²C bus.
#[derive(Debug)]
pub struct Atecc {
    shared: Arc<Shared>,
    wires: Arc<SlaveWires>,
    /// The bus to hook onto at realize time, if the machine named one.
    bus: Option<Arc<I2cBus>>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    part: Part,
    /// The seven-bit address, from `I2C_Address` (§2.2.4).
    address: u8,
    /// A multiplier on [`EXEC_US`], for a board whose domain is not 1 MHz.
    exec_scale: u64,
    /// tWATCHDOG, in ticks of this device's clock domain (§6.3).
    watchdog_ticks: u64,
    /// The board's seed, before this instance's path is mixed into it.
    board_seed: u64,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick the current command finishes on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The catch-up handle, once the machine has given us one.
    lazy: Mutex<Option<LazyHandle>>,
}

/// "Nothing scheduled".
const NO_EVENT: u64 = u64::MAX;

/// Everything a snapshot has to carry.
#[derive(Debug, Clone)]
struct State {
    /// Domain ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    /// The configuration zone (§2.2).
    config: Vec<u8>,
    /// The OTP zone (§2.3).
    otp: Vec<u8>,
    /// The data zone, sixteen slots laid end to end (§2.1).
    data: Vec<u8>,
    /// The nine-byte serial number, which also lives in the config zone.
    serial: [u8; 9],
    power: Power,
    /// The tick the watchdog puts the part to sleep on (§6.3).
    awake_until: u64,
    /// Whether a command is executing.
    busy: bool,
    /// The tick it finishes on.
    busy_until: u64,
    phase: Phase,
    /// The command packet being collected.
    rx: Vec<u8>,
    /// The response waiting to be read out.
    tx: Vec<u8>,
    /// How much of it the master has taken.
    tx_pos: usize,
    /// `TempKey` (§9.2, and the tables every command's message is built from).
    temp_key: [u8; 32],
    /// Whether `TempKey` holds anything.
    temp_key_valid: bool,
    /// `TempKey.SourceFlag`: `false` when the value came from an internally
    /// generated random number, `true` when the host supplied it.
    temp_key_from_input: bool,
    /// Whether the last thing to write `TempKey` was `GenDig`.
    temp_key_gendig: bool,
    /// The message a `SHA` context has accumulated (§9.2, `SHA`).
    sha_msg: Vec<u8>,
    /// Whether a `SHA` context is open.
    sha_active: bool,
    /// The two monotonic counters (§2.2, `Counter[0,1]`).
    counters: [u32; 2],
    /// The deterministic byte source everything "random" comes from.
    stream: Stream,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("AteccShared");
        s.field("part", &self.part.name());
        s.field("address", &alloc::format!("{:#04x}", self.address));
        match self.state.try_lock() {
            Some(state) => s
                .field("power", &state.power)
                .field("busy", &state.busy)
                .field("phase", &state.phase),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

impl Atecc {
    /// Validate `props` and build the part.
    ///
    /// Properties:
    ///
    /// * `part` — `atecc508a`, `atecc608a` or `atecc608b`. Defaults to
    ///   `atecc608a`, and the choice decides `RevNum`, whether `AES` exists and
    ///   whether `HMAC` does.
    /// * `seed` — the board's seed, mixed with this instance's path at realize
    ///   (see the module docs). Defaults to 0.
    /// * `address` — the seven-bit I²C address, which a real part carries in
    ///   `I2C_Address`. Defaults to [`DEFAULT_ADDRESS`].
    /// * `config` — a media slot holding the initial configuration zone. Bytes
    ///   0–15 are ignored: they are the serial number, the revision and the
    ///   interface bytes, which no host can write (§2.2). So are the two lock
    ///   bytes and `SlotLocked`, which `lock` below decides — a zone is locked
    ///   by saying so, not by writing `0x00` into an image.
    /// * `otp`, `data` — media slots holding the initial OTP and data zones.
    /// * `lock` — `none`, `config` or `data`: which zones this part comes up
    ///   with already locked, for firmware that expects a provisioned device.
    ///   `data` implies `config`, as the part does (§2.2: the configuration
    ///   zone must be locked first). Defaults to `none`.
    /// * `exec-scale` — a multiplier on the datasheet execution times, for a
    ///   board whose clock domain is not the 1 MHz the defaults assume.
    ///   Defaults to 1.
    /// * `watchdog-ticks` — tWATCHDOG in ticks of this device's clock domain.
    ///   Defaults to [`DEFAULT_WATCHDOG_TICKS`]; `0` disables the watchdog,
    ///   which no real part can do and which exists for a test that wants to
    ///   hold the part awake.
    /// * `bus` — the named [`I2cBus`] to hang off.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for an
    /// unknown `part` or `lock`, an address above `0x7f`, or an image longer
    /// than its zone.
    pub fn new(props: &Props) -> Result<Atecc> {
        let mut r = props.reader();
        let part_name = r.or_str("part", "atecc608a")?;
        let seed: u64 = r.or("seed", 0u64)?;
        let address: u64 = r.or("address", u64::from(DEFAULT_ADDRESS))?;
        let config = r
            .optional_media("config")?
            .map(crate::core::props::Media::to_bytes);
        let otp = r
            .optional_media("otp")?
            .map(crate::core::props::Media::to_bytes);
        let data = r
            .optional_media("data")?
            .map(crate::core::props::Media::to_bytes);
        let lock = r.or_str("lock", "none")?;
        let exec_scale: u64 = r.or("exec-scale", 1u64)?;
        let watchdog_ticks: u64 = r.or("watchdog-ticks", DEFAULT_WATCHDOG_TICKS)?;
        let bus_name = r.optional_str("bus")?.map(String::from);
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        let part = Part::from_name(part_name).ok_or_else(|| {
            bad(alloc::format!(
                "`part` is `{part_name}`; this class models `atecc508a`, `atecc608a` and \
                 `atecc608b`"
            ))
        })?;
        if address > 0x7f {
            return Err(bad(alloc::format!(
                "`address` is {address:#x}; an ATECC answers a seven-bit address (§7.1)"
            )));
        }
        if exec_scale == 0 {
            return Err(bad(String::from(
                "`exec-scale` is 0; a command takes time (§9.4), and zero would make the \
                 acknowledge-polling a driver does meaningless",
            )));
        }
        let (lock_config, lock_data) = match lock {
            "none" => (false, false),
            "config" => (true, false),
            "data" => (true, true),
            other => {
                return Err(bad(alloc::format!(
                    "`lock` is `{other}`; it is `none`, `config` or `data` — and `data` implies \
                     `config`, because §2.2 makes the configuration zone lock first"
                )));
            }
        };

        let mut state = State {
            ticks: 0,
            config: alloc::vec![0u8; CONFIG_BYTES],
            otp: alloc::vec![0xff_u8; OTP_BYTES],
            data: alloc::vec![0u8; DATA_BYTES],
            serial: [0x01, 0x23, 0, 0, 0, 0, 0, 0, 0xee],
            power: Power::Sleep,
            awake_until: 0,
            busy: false,
            busy_until: 0,
            phase: Phase::Idle,
            rx: Vec::new(),
            tx: Vec::new(),
            tx_pos: 0,
            temp_key: [0; 32],
            temp_key_valid: false,
            temp_key_from_input: false,
            temp_key_gendig: false,
            sha_msg: Vec::new(),
            sha_active: false,
            counters: [0; 2],
            stream: Stream::new(seed),
        };
        if let Some(image) = config {
            if image.len() > CONFIG_BYTES {
                return Err(bad(alloc::format!(
                    "`config` is {} bytes and the configuration zone is {CONFIG_BYTES}",
                    image.len()
                )));
            }
            state.config[..image.len()].copy_from_slice(&image);
        }
        if let Some(image) = otp {
            if image.len() > OTP_BYTES {
                return Err(bad(alloc::format!(
                    "`otp` is {} bytes and the OTP zone is {OTP_BYTES}",
                    image.len()
                )));
            }
            state.otp[..image.len()].copy_from_slice(&image);
        }
        if let Some(image) = data {
            if image.len() > DATA_BYTES {
                return Err(bad(alloc::format!(
                    "`data` is {} bytes and the data zone is {DATA_BYTES}",
                    image.len()
                )));
            }
            state.data[..image.len()].copy_from_slice(&image);
        }
        // The fixed head of the configuration zone, which no image and no
        // `Write` may set (§2.2): the serial number, the revision, the
        // interface selection and the address.
        state.config[LOCKVALUE_OFFSET] = if lock_data { LOCKED } else { UNLOCKED };
        state.config[LOCKCONFIG_OFFSET] = if lock_config { LOCKED } else { UNLOCKED };
        state.config[SLOTLOCKED_OFFSET] = 0xff;
        state.config[SLOTLOCKED_OFFSET + 1] = 0xff;

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, state),
            part,
            address: address as u8,
            exec_scale,
            watchdog_ticks,
            board_seed: seed,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        shared.write_fixed_config();
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        let wires = Arc::new(SlaveWires::new(Arc::clone(&shared) as Arc<dyn I2cSlave>));
        Ok(Atecc { shared, wires, bus })
    }

    /// The seven-bit address this part answers (§7.1).
    #[must_use]
    pub fn address(&self) -> Address {
        Address::Seven(self.shared.address)
    }

    /// Which member of the family this is.
    #[must_use]
    pub fn part(&self) -> Part {
        self.shared.part
    }

    /// The nine-byte serial number (§2.2.1).
    #[must_use]
    pub fn serial(&self) -> [u8; 9] {
        self.shared.state.lock().serial
    }

    /// This part as a bus device, for a controller that hands it whole bytes.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.shared) as Arc<dyn I2cSlave>
    }

    /// The part's wire pins, for a controller that drives them directly.
    #[must_use]
    pub fn wires(&self) -> &Arc<SlaveWires> {
        &self.wires
    }

    /// Whether the part is awake (§6).
    #[must_use]
    pub fn awake(&self) -> bool {
        let mut state = self.shared.state.lock();
        let now = self.shared.now(&state);
        self.shared.expire_watchdog(&mut state, now);
        state.power == Power::Awake
    }

    /// Whether a command is executing (§9.4), which is when the part NACKs.
    #[must_use]
    pub fn busy(&self) -> bool {
        let state = self.shared.state.lock();
        let now = self.shared.now(&state);
        state.busy && now < state.busy_until
    }

    /// A copy of the configuration zone, without touching any protocol state.
    #[must_use]
    pub fn config(&self) -> Vec<u8> {
        self.shared.state.lock().config.clone()
    }

    /// A copy of one slot of the data zone, for a test or a monitor.
    #[must_use]
    pub fn slot(&self, slot: usize) -> Option<Vec<u8>> {
        if slot >= SLOTS {
            return None;
        }
        let state = self.shared.state.lock();
        let base = slot_offset(slot) as usize;
        let len = SLOT_SIZES[slot] as usize;
        Some(state.data[base..base + len].to_vec())
    }

    /// The seed the board handed this instance, before the path is mixed in.
    #[must_use]
    pub fn board_seed(&self) -> u64 {
        self.shared.board_seed
    }

    /// The seed this instance's stream actually runs from.
    #[must_use]
    pub fn stream_seed(&self) -> u64 {
        self.shared.state.lock().stream.seed()
    }

    /// Re-seed the stream from the board seed and this instance's path, and
    /// derive the serial number from it.
    ///
    /// What [`Device::realize`] does; a test that is not building a machine
    /// calls it directly. Two instances of this class on one board therefore
    /// hold different keys without anybody writing a second number into the
    /// machine file — and the serial number, which firmware reads and prints,
    /// differs with them.
    pub fn seed_from_path(&self, path: &str) {
        let derived = derive_seed(self.shared.board_seed, path);
        let mut state = self.shared.state.lock();
        state.stream = Stream::new(derived);
        // A second mix, so the serial number is not simply the first thing the
        // RNG would have dealt: the two are independent draws from one seed.
        let mut serial_stream = Stream::new(derive_seed(derived, "serial"));
        let mut middle = [0u8; 6];
        serial_stream.fill(&mut middle);
        state.serial = [
            0x01, 0x23, middle[0], middle[1], middle[2], middle[3], middle[4], middle[5], 0xee,
        ];
        drop(state);
        self.shared.write_fixed_config();
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// `TempKey`, if it holds anything.
    ///
    /// Test-only, and deliberately so: on real hardware `TempKey` is the one
    /// register a host can never read, and a public accessor would be a
    /// back door in a model of a part whose whole job is not having one.
    #[cfg(test)]
    fn temp_key_for_test(&self) -> Option<[u8; 32]> {
        let state = self.shared.state.lock();
        state.temp_key_valid.then_some(state.temp_key)
    }

    /// Put a counter somewhere near its limit.
    ///
    /// Test-only. Clocking two million `Counter` commands through the bus to
    /// reach the wall would take minutes and prove the same thing.
    #[cfg(test)]
    fn set_counter_for_test(&self, id: usize, value: u32) {
        self.shared.state.lock().counters[id] = value;
    }

    /// Run the part until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

/// Where a slot starts in the flattened data zone.
fn slot_offset(slot: usize) -> u64 {
    SLOT_SIZES.iter().take(slot.min(SLOTS)).sum()
}

// ---------------------------------------------------------------------------
// Slot policy (§2.2.5 SlotConfig, §2.2.6 KeyConfig)
// ---------------------------------------------------------------------------

/// What `SlotConfig.WriteConfig` permits.
///
/// The four behaviours the model enforces, which is this file's reading of
/// §2.2.5's table: `0b0000` is "always", `0b0001` is "PubInvalid", bit 1
/// selects an encrypted write, and every remaining combination forbids a write
/// outright. A machine description that configures a slot gets exactly the
/// behaviour it asks for, which is what firmware is testing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WriteConfig {
    Always,
    PubInvalid,
    Encrypt,
    Never,
}

impl WriteConfig {
    const fn decode(bits: u8) -> WriteConfig {
        match bits & 0x0f {
            0b0000 => WriteConfig::Always,
            0b0001 => WriteConfig::PubInvalid,
            w if w & 0b0010 != 0 => WriteConfig::Encrypt,
            _ => WriteConfig::Never,
        }
    }
}

impl State {
    /// `SlotConfig[slot]` (§2.2.5), little-endian in the configuration zone.
    fn slot_config(&self, slot: usize) -> u16 {
        let at = SLOTCONFIG_OFFSET + slot * 2;
        u16::from(self.config[at]) | (u16::from(self.config[at + 1]) << 8)
    }

    /// `KeyConfig[slot]` (§2.2.6).
    fn key_config(&self, slot: usize) -> u16 {
        let at = KEYCONFIG_OFFSET + slot * 2;
        u16::from(self.config[at]) | (u16::from(self.config[at + 1]) << 8)
    }

    /// `SlotConfig.IsSecret`, bit 7: the slot may never be read in the clear.
    fn is_secret(&self, slot: usize) -> bool {
        self.slot_config(slot) & 0x0080 != 0
    }

    /// `SlotConfig.EncryptRead`, bit 6.
    fn encrypt_read(&self, slot: usize) -> bool {
        self.slot_config(slot) & 0x0040 != 0
    }

    /// `SlotConfig.WriteConfig`, bits 15:12.
    fn write_config(&self, slot: usize) -> WriteConfig {
        WriteConfig::decode((self.slot_config(slot) >> 12) as u8)
    }

    /// `KeyConfig.Private`, bit 0: the slot holds a P-256 private key.
    fn is_private(&self, slot: usize) -> bool {
        self.key_config(slot) & 0x0001 != 0
    }

    /// Whether the configuration zone is locked (§2.2).
    fn config_locked(&self) -> bool {
        self.config[LOCKCONFIG_OFFSET] == LOCKED
    }

    /// Whether the data and OTP zones are locked.
    fn data_locked(&self) -> bool {
        self.config[LOCKVALUE_OFFSET] == LOCKED
    }

    /// Whether one slot has been individually locked (§2.2, `SlotLocked`).
    fn slot_locked(&self, slot: usize) -> bool {
        let word = u16::from(self.config[SLOTLOCKED_OFFSET])
            | (u16::from(self.config[SLOTLOCKED_OFFSET + 1]) << 8);
        word & (1 << slot) == 0
    }

    /// The 32 bytes a key slot holds.
    ///
    /// A private key lives in the **last 32 bytes** of a 36-byte slot (§2.1:
    /// the four leading bytes are zero), and a symmetric key occupies the first
    /// 32. This returns the right ones for the slot's `KeyConfig`.
    fn key_bytes(&self, slot: usize) -> [u8; 32] {
        let base = slot_offset(slot) as usize;
        let at = if self.is_private(slot) && SLOT_SIZES[slot] >= 36 {
            base + 4
        } else {
            base
        };
        let mut out = [0u8; 32];
        out.copy_from_slice(&self.data[at..at + 32]);
        out
    }

    /// Write those 32 bytes back.
    fn set_key_bytes(&mut self, slot: usize, key: &[u8; 32]) {
        let base = slot_offset(slot) as usize;
        let at = if self.is_private(slot) && SLOT_SIZES[slot] >= 36 {
            base + 4
        } else {
            base
        };
        self.data[at..at + 32].copy_from_slice(key);
    }
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

impl Shared {
    /// Put the bytes §2.2 makes read-only back where they belong.
    fn write_fixed_config(&self) {
        let mut state = self.state.lock();
        let serial = state.serial;
        state.config[0..4].copy_from_slice(&serial[0..4]);
        state.config[4..8].copy_from_slice(&self.part.revnum());
        state.config[8..13].copy_from_slice(&serial[4..9]);
        // Byte 13 is the 608's `AES_Enable` and reserved on a 508A; byte 14 is
        // `I2C_Enable`, whose bit 0 selects the I²C interface over SWI — which
        // is the only interface this model has (§2.2.4).
        state.config[13] = u8::from(self.part.has_aes());
        state.config[14] = 0x01;
        state.config[15] = 0x00;
        state.config[16] = self.address << 1;
        self.publish(&state);
    }

    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        let next = if state.busy {
            state.busy_until.max(state.ticks.saturating_add(1))
        } else if state.power == Power::Awake && self.watchdog_ticks != 0 {
            state.awake_until.max(state.ticks.saturating_add(1))
        } else {
            NO_EVENT
        };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Simulate forward.
    fn advance_to(&self, target: u64) {
        let mut state = self.state.lock();
        if target <= state.ticks {
            return;
        }
        state.ticks = target;
        if state.busy && target >= state.busy_until {
            state.busy = false;
        }
        self.expire_watchdog(&mut state, target);
        self.publish(&state);
    }

    /// Where this device's clock domain has got to.
    ///
    /// **Not [`LazyHandle::sync`]**, for the reason [`super::at24c`] spells
    /// out: the bus reaches this device from inside [`SlaveWires`]'s bit lock,
    /// and `sync` would re-enter it.
    fn now(&self, state: &State) -> u64 {
        let handle = self.lazy.lock().clone();
        match handle {
            Some(handle) => handle.present_tick().max(state.ticks),
            None => state.ticks,
        }
    }

    /// §6.3: after tWATCHDOG the part puts *itself* to sleep, with no help from
    /// the host and in the middle of whatever it was doing.
    fn expire_watchdog(&self, state: &mut State, now: u64) {
        if self.watchdog_ticks == 0 || state.power != Power::Awake {
            return;
        }
        if now >= state.awake_until {
            self.go_to_sleep(state);
        }
    }

    /// Whether a command is executing (§9.4).
    fn is_busy(&self, state: &State, now: u64) -> bool {
        state.busy && now < state.busy_until
    }

    /// §6.2: sleep clears every volatile register.
    fn go_to_sleep(&self, state: &mut State) {
        state.power = Power::Sleep;
        state.phase = Phase::Idle;
        state.rx.clear();
        state.tx.clear();
        state.tx_pos = 0;
        state.temp_key = [0; 32];
        state.temp_key_valid = false;
        state.temp_key_from_input = false;
        state.temp_key_gendig = false;
        state.sha_msg.clear();
        state.sha_active = false;
        state.busy = false;
    }

    /// §6.2: idle keeps `TempKey` and the RNG seed; only the I/O buffer and the
    /// watchdog go.
    fn go_idle(&self, state: &mut State) {
        state.power = Power::Idle;
        state.phase = Phase::Idle;
        state.rx.clear();
        state.tx.clear();
        state.tx_pos = 0;
    }

    /// §6.1: the wake pulse, and the `04 11 33 43` a read then answers.
    fn wake(&self, state: &mut State, now: u64) {
        if state.power == Power::Sleep {
            state.temp_key = [0; 32];
            state.temp_key_valid = false;
            state.temp_key_from_input = false;
            state.temp_key_gendig = false;
            state.sha_msg.clear();
            state.sha_active = false;
        }
        state.power = Power::Awake;
        state.awake_until = now.saturating_add(self.watchdog_ticks);
        state.phase = Phase::Idle;
        state.rx.clear();
        state.tx.clear();
        state.tx.extend_from_slice(&WAKE_TOKEN);
        state.tx_pos = 0;
        state.busy = false;
        self.publish(state);
    }

    /// Take the collected packet and start executing it (§9.1).
    fn start_command(&self, state: &mut State, now: u64) {
        let packet = core::mem::take(&mut state.rx);
        state.tx.clear();
        state.tx_pos = 0;

        // §9.1.1: the count byte covers the whole packet, CRC included, and a
        // packet whose CRC does not check out gets 0xFF rather than an attempt
        // at execution.
        let body = match validate(&packet) {
            Ok(body) => body,
            Err(status) => {
                state.tx = response(&[status]);
                state.busy = true;
                state.busy_until = now.saturating_add(self.scaled(exec_us(OP_READ)));
                self.publish(state);
                return;
            }
        };
        let (opcode, param1, param2, data) = body;
        let ticks = self.scaled(exec_us(opcode));

        // §6.3's other half: rather than being cut off mid-command by its own
        // watchdog, the part refuses a command it cannot finish in time.
        if self.watchdog_ticks != 0 && now.saturating_add(ticks) > state.awake_until {
            state.tx = response(&[STATUS_WATCHDOG]);
            state.busy = true;
            state.busy_until = now.saturating_add(self.scaled(exec_us(OP_READ)));
            self.publish(state);
            return;
        }

        let body = match self.execute(state, opcode, param1, param2, data) {
            Ok(body) => body,
            Err(status) => alloc::vec![status],
        };
        state.tx = response(&body);
        state.busy = true;
        state.busy_until = now.saturating_add(ticks);
        self.publish(state);
    }

    /// Datasheet microseconds into this board's ticks.
    fn scaled(&self, us: u64) -> u64 {
        us.saturating_mul(self.exec_scale)
    }

    /// Run one command. `Err` is the status byte the part answers with.
    ///
    /// This is the whole of the device above the transport, and it is
    /// deliberately a pure function of the state plus a packet: the SWI front
    /// end the module docs describe would call exactly this.
    fn execute(
        &self,
        state: &mut State,
        opcode: u8,
        param1: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        match opcode {
            OP_INFO => self.op_info(state, param1, param2),
            OP_READ => self.op_read(state, param1, param2),
            OP_WRITE => self.op_write(state, param1, param2, data),
            OP_LOCK => self.op_lock(state, param1, param2),
            OP_UPDATEEXTRA => self.op_update_extra(state, param1, param2),
            OP_COUNTER => self.op_counter(state, param1, param2),
            OP_RANDOM => self.op_random(state, param1),
            OP_NONCE => self.op_nonce(state, param1, param2, data),
            OP_GENDIG => self.op_gendig(state, param1, param2),
            OP_MAC => self.op_mac(state, param1, param2, data),
            OP_CHECKMAC => self.op_checkmac(state, param1, param2, data),
            OP_HMAC => self.op_hmac(state, param1, param2),
            OP_SHA => self.op_sha(state, param1, param2, data),
            OP_GENKEY => self.op_genkey(state, param1, param2),
            OP_PRIVWRITE => self.op_privwrite(state, param1, param2, data),
            OP_SIGN => self.op_sign(state, param1, param2),
            OP_VERIFY => self.op_verify(state, param1, param2, data),
            OP_ECDH => self.op_ecdh(state, param1, param2, data),
            OP_AES => self.op_aes(state, param1, param2, data),
            OP_SELFTEST => self.op_selftest(state, param1),
            OP_PAUSE => self.op_pause(state, param1),
            // A command this model does not implement. `DeriveKey`, `KDF` and
            // `SecureBoot` land here, and a parse error is the honest answer:
            // inventing their message layouts would be worse than the gap the
            // module docs admit to.
            _ => Err(STATUS_PARSE),
        }
    }

    // -- the commands -------------------------------------------------------

    /// `Info` (§9.2): four bytes about the part.
    fn op_info(&self, state: &State, mode: u8, param2: u16) -> core::result::Result<Vec<u8>, u8> {
        match mode {
            // Revision. The one mode every driver calls first, because it is
            // legal on a blank part and identifies the silicon.
            0x00 => Ok(self.part.revnum().to_vec()),
            // KeyValid: whether the public key stored in `param2`'s slot was
            // generated or validated. This model keeps no separate validity
            // flag, so the answer is "valid if the slot holds a key at all".
            0x01 => {
                let slot = usize::from(param2 & 0x0f);
                let any = state.key_bytes(slot).iter().any(|b| *b != 0);
                Ok(alloc::vec![u8::from(any), 0, 0, 0])
            }
            // State. The datasheet returns a bitfield of volatile-register
            // flags; the three this model has are spelled out here rather than
            // guessed at wholesale.
            0x02 => {
                let mut byte = 0u8;
                if state.temp_key_valid {
                    byte |= 0x01;
                }
                if state.temp_key_from_input {
                    byte |= 0x02;
                }
                if state.temp_key_gendig {
                    byte |= 0x04;
                }
                Ok(alloc::vec![byte, 0, 0, 0])
            }
            // GPIO (608). This model has no GPIO pin, and says so.
            0x03 if self.part.has_aes() => Ok(alloc::vec![0, 0, 0, 0]),
            _ => Err(STATUS_PARSE),
        }
    }

    /// `Read` (§9.2): four or thirty-two bytes out of a zone.
    fn op_read(&self, state: &State, mode: u8, param2: u16) -> core::result::Result<Vec<u8>, u8> {
        let zone = Zone::from_bits(mode).ok_or(STATUS_PARSE)?;
        let long = mode & 0x80 != 0;
        let len = if long { 32 } else { 4 };
        let (offset, slot) = decode_address(zone, param2)?;
        if long && offset % 32 != 0 {
            return Err(STATUS_PARSE);
        }

        match zone {
            Zone::Config => {
                // The configuration zone is always readable — it has to be, or
                // a host could not discover the part (§2.2).
                read_slice(&state.config, offset, len)
            }
            Zone::Otp => {
                if !state.config_locked() {
                    return Err(STATUS_EXEC);
                }
                read_slice(&state.otp, offset, len)
            }
            Zone::Data => {
                // §2.1: the data zone cannot be read until it is locked, which
                // is what stops a half-provisioned part from leaking.
                if !state.data_locked() {
                    return Err(STATUS_EXEC);
                }
                if state.is_secret(slot) || state.encrypt_read(slot) {
                    // An encrypted read would need a `GenDig` session key. The
                    // module docs list it as absent; refusing is the datasheet's
                    // own answer for a clear-text read of a secret slot.
                    return Err(STATUS_EXEC);
                }
                let base = slot_offset(slot) + offset;
                read_slice(&state.data, base, len)
            }
        }
    }

    /// `Write` (§9.2), in the clear and encrypted.
    fn op_write(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        let zone = Zone::from_bits(mode).ok_or(STATUS_PARSE)?;
        let long = mode & 0x80 != 0;
        let encrypted = mode & 0x40 != 0;
        let len = if long { 32usize } else { 4 };
        let want = if encrypted { len + 32 } else { len };
        if data.len() != want {
            return Err(STATUS_PARSE);
        }
        let (offset, slot) = decode_address(zone, param2)?;
        if long && offset % 32 != 0 {
            return Err(STATUS_PARSE);
        }
        if encrypted && !long {
            // §9.2: only a 32-byte write can be encrypted — the session key is
            // 32 bytes and there is nothing to XOR a four-byte write with.
            return Err(STATUS_PARSE);
        }

        // Decrypt first, so the permission checks below see the plaintext.
        let mut plain = data[..len].to_vec();
        if encrypted {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            let session = state.temp_key;
            for (i, byte) in plain.iter_mut().enumerate() {
                *byte ^= session[i];
            }
            // The integrity MAC of §9.2's `Write` table, which is what stops a
            // bit-flip in the ciphertext from landing silently.
            let mut msg = Vec::with_capacity(96);
            msg.extend_from_slice(&session);
            msg.push(OP_WRITE);
            msg.push(mode);
            msg.push((param2 & 0xff) as u8);
            msg.push((param2 >> 8) as u8);
            msg.push(state.serial[8]);
            msg.extend_from_slice(&state.serial[0..2]);
            msg.extend_from_slice(&[0u8; 25]);
            msg.extend_from_slice(&plain);
            let expect = Sha256::digest(&msg);
            if expect.as_ref() != &data[len..] {
                return Err(STATUS_MISCOMPARE);
            }
        }

        match zone {
            Zone::Config => {
                if state.config_locked() {
                    // §2.2: once locked, only `UpdateExtra` moves a
                    // configuration byte.
                    return Err(STATUS_EXEC);
                }
                if (offset as usize) < CONFIG_FIXED {
                    // The serial number, the revision and the interface bytes.
                    return Err(STATUS_EXEC);
                }
                // The lock bytes are set by `Lock`, never by `Write`.
                let end = offset as usize + len;
                if (offset as usize) <= LOCKCONFIG_OFFSET && end > LOCKVALUE_OFFSET {
                    return Err(STATUS_EXEC);
                }
                write_slice(&mut state.config, offset, &plain)?;
            }
            Zone::Otp => {
                // §2.3: the OTP zone is writable between the two locks and
                // never afterwards.
                if !state.config_locked() || state.data_locked() {
                    return Err(STATUS_EXEC);
                }
                write_slice(&mut state.otp, offset, &plain)?;
            }
            Zone::Data => {
                if !state.config_locked() {
                    // §2.1: nothing reaches the data zone until the
                    // configuration that governs it is fixed.
                    return Err(STATUS_EXEC);
                }
                if state.data_locked() {
                    if state.slot_locked(slot) {
                        return Err(STATUS_EXEC);
                    }
                    match state.write_config(slot) {
                        WriteConfig::Always => {}
                        WriteConfig::PubInvalid => {}
                        WriteConfig::Encrypt => {
                            if !encrypted {
                                return Err(STATUS_EXEC);
                            }
                        }
                        WriteConfig::Never => return Err(STATUS_EXEC),
                    }
                } else if !long {
                    // §2.1: before the data zone is locked a write must be a
                    // whole 32-byte block.
                    return Err(STATUS_EXEC);
                }
                let base = slot_offset(slot) + offset;
                if base + len as u64 > SLOT_SIZES[slot] + slot_offset(slot) {
                    return Err(STATUS_PARSE);
                }
                write_slice(&mut state.data, base, &plain)?;
            }
        }
        Ok(alloc::vec![STATUS_OK])
    }

    /// `Lock` (§9.2): the two-stage lock, and the summary CRC that proves the
    /// host and the part agree about what is being frozen.
    fn op_lock(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        let ignore_crc = mode & 0x80 != 0;
        match mode & 0x03 {
            // The configuration zone.
            0x00 => {
                if state.config_locked() {
                    return Err(STATUS_EXEC);
                }
                if !ignore_crc {
                    let summary = crc16(&state.config);
                    if u16::from_le_bytes(summary) != param2 {
                        return Err(STATUS_MISCOMPARE);
                    }
                }
                state.config[LOCKCONFIG_OFFSET] = LOCKED;
            }
            // The data and OTP zones together.
            0x01 => {
                if !state.config_locked() || state.data_locked() {
                    return Err(STATUS_EXEC);
                }
                if !ignore_crc {
                    let mut both = state.data.clone();
                    both.extend_from_slice(&state.otp);
                    let summary = crc16(&both);
                    if u16::from_le_bytes(summary) != param2 {
                        return Err(STATUS_MISCOMPARE);
                    }
                }
                state.config[LOCKVALUE_OFFSET] = LOCKED;
            }
            // A single slot (§2.2, `SlotLocked`).
            0x02 => {
                if !state.data_locked() {
                    return Err(STATUS_EXEC);
                }
                let slot = usize::from((mode >> 2) & 0x0f);
                let mut word = u16::from(state.config[SLOTLOCKED_OFFSET])
                    | (u16::from(state.config[SLOTLOCKED_OFFSET + 1]) << 8);
                word &= !(1u16 << slot);
                state.config[SLOTLOCKED_OFFSET] = (word & 0xff) as u8;
                state.config[SLOTLOCKED_OFFSET + 1] = (word >> 8) as u8;
            }
            _ => return Err(STATUS_PARSE),
        }
        Ok(alloc::vec![STATUS_OK])
    }

    /// `UpdateExtra` (§9.2): the two configuration bytes that stay writable
    /// after the zone is locked.
    fn op_update_extra(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        if !state.config_locked() {
            return Err(STATUS_EXEC);
        }
        let at = match mode {
            0x00 => USEREXTRA_OFFSET,
            0x01 => SELECTOR_OFFSET,
            _ => return Err(STATUS_PARSE),
        };
        // §2.2: each may be moved off zero exactly once.
        if state.config[at] != 0 {
            return Err(STATUS_EXEC);
        }
        state.config[at] = (param2 & 0xff) as u8;
        Ok(alloc::vec![STATUS_OK])
    }

    /// `Counter` (§9.2): read, or increment and read.
    fn op_counter(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        let id = usize::from(param2 & 0x0f);
        if id > 1 {
            return Err(STATUS_PARSE);
        }
        match mode {
            0x00 => {}
            0x01 => {
                if state.counters[id] >= COUNTER_MAX {
                    // The counter is monotonic and 21 bits wide; at its limit
                    // the command fails and the value stays where it is.
                    return Err(STATUS_EXEC);
                }
                state.counters[id] += 1;
            }
            _ => return Err(STATUS_PARSE),
        }
        Ok(state.counters[id].to_le_bytes().to_vec())
    }

    /// `Random` (§9.2): thirty-two bytes.
    fn op_random(&self, state: &mut State, mode: u8) -> core::result::Result<Vec<u8>, u8> {
        if mode > 1 {
            return Err(STATUS_PARSE);
        }
        Ok(self.random32(state).to_vec())
    }

    /// The thirty-two bytes `Random` and `Nonce` draw.
    ///
    /// §9.2: **before the configuration zone is locked the part does not
    /// generate random numbers at all** — it answers the fixed pattern
    /// `FFFF0000` over and over, so that a provisioning script cannot mistake a
    /// blank part for a working one. Afterwards the bytes come from the board's
    /// seed, never from the host.
    fn random32(&self, state: &mut State) -> [u8; 32] {
        let mut out = [0u8; 32];
        if !state.config_locked() {
            for (i, byte) in out.iter_mut().enumerate() {
                *byte = if i % 4 < 2 { 0xff } else { 0x00 };
            }
            return out;
        }
        state.stream.fill(&mut out);
        out
    }

    /// `Nonce` (§9.2): load `TempKey`, from a random number or from the host.
    fn op_nonce(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        match mode & 0x03 {
            // Random, with and without a seed update. This model's stream is
            // its seed, so the two differ only in the flag they set.
            0x00 | 0x01 => {
                if data.len() != 20 {
                    return Err(STATUS_PARSE);
                }
                let rand_out = self.random32(state);
                // §9.2's `Nonce` table: TempKey = SHA-256(RandOut ‖ NumIn ‖
                // Opcode ‖ Mode ‖ LSB(Param2)), 55 bytes in all.
                let mut msg = Vec::with_capacity(55);
                msg.extend_from_slice(&rand_out);
                msg.extend_from_slice(data);
                msg.push(OP_NONCE);
                msg.push(mode);
                msg.push((param2 & 0xff) as u8);
                state
                    .temp_key
                    .copy_from_slice(Sha256::digest(&msg).as_ref());
                state.temp_key_valid = true;
                state.temp_key_from_input = false;
                state.temp_key_gendig = false;
                Ok(rand_out.to_vec())
            }
            // Pass-through: the host's thirty-two bytes *are* TempKey.
            0x03 => {
                if data.len() != 32 {
                    return Err(STATUS_PARSE);
                }
                state.temp_key.copy_from_slice(data);
                state.temp_key_valid = true;
                state.temp_key_from_input = true;
                state.temp_key_gendig = false;
                Ok(alloc::vec![STATUS_OK])
            }
            _ => Err(STATUS_PARSE),
        }
    }

    /// `GenDig` (§9.2): fold a stored key into `TempKey`.
    fn op_gendig(
        &self,
        state: &mut State,
        zone: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        if !state.temp_key_valid {
            return Err(STATUS_EXEC);
        }
        let slot = usize::from(param2 & 0x0f);
        let value = match zone {
            // Config: the 32-byte block `param2` selects.
            0x00 => {
                let at = usize::from(param2 & 0x03) * 32;
                let mut v = [0u8; 32];
                v.copy_from_slice(&state.config[at..at + 32]);
                v
            }
            // OTP: likewise.
            0x01 => {
                let at = usize::from(param2 & 0x01) * 32;
                let mut v = [0u8; 32];
                v.copy_from_slice(&state.otp[at..at + 32]);
                v
            }
            // Data: the slot's key.
            0x02 => state.key_bytes(slot),
            _ => return Err(STATUS_PARSE),
        };
        // §9.2's `GenDig` table: TempKey = SHA-256(KeyValue ‖ Opcode ‖ Zone ‖
        // KeyID[2] ‖ SN[8] ‖ SN[0:1] ‖ 0x00 × 25 ‖ TempKey), 96 bytes.
        let mut msg = Vec::with_capacity(96);
        msg.extend_from_slice(&value);
        msg.push(OP_GENDIG);
        msg.push(zone);
        msg.push((param2 & 0xff) as u8);
        msg.push((param2 >> 8) as u8);
        msg.push(state.serial[8]);
        msg.extend_from_slice(&state.serial[0..2]);
        msg.extend_from_slice(&[0u8; 25]);
        msg.extend_from_slice(&state.temp_key);
        state
            .temp_key
            .copy_from_slice(Sha256::digest(&msg).as_ref());
        state.temp_key_valid = true;
        state.temp_key_gendig = true;
        Ok(alloc::vec![STATUS_OK])
    }

    /// The eighty-eight byte message `MAC`, `CheckMac` and `HMAC` are built
    /// from (§9.2, the `MAC` command's table).
    ///
    /// ```text
    ///   32  the key, or TempKey
    ///   32  the challenge, or TempKey
    ///    1  Opcode
    ///    1  Mode
    ///    2  KeyID, little-endian
    ///    8  OTP[0:7]  or zeros   (mode bit 4)
    ///    3  OTP[8:10] or zeros   (mode bit 5)
    ///    1  SN[8]
    ///    4  SN[4:7]  or zeros    (mode bit 6)
    ///    2  SN[0:1]
    ///    2  SN[2:3]  or zeros    (mode bit 6)
    /// ```
    fn mac_message(
        state: &State,
        opcode: u8,
        mode: u8,
        param2: u16,
        first: &[u8; 32],
        second: &[u8; 32],
    ) -> Vec<u8> {
        let mut msg = Vec::with_capacity(88);
        msg.extend_from_slice(first);
        msg.extend_from_slice(second);
        msg.push(opcode);
        msg.push(mode);
        msg.push((param2 & 0xff) as u8);
        msg.push((param2 >> 8) as u8);
        if mode & 0x10 != 0 {
            msg.extend_from_slice(&state.otp[0..8]);
        } else {
            msg.extend_from_slice(&[0u8; 8]);
        }
        if mode & 0x20 != 0 {
            msg.extend_from_slice(&state.otp[8..11]);
        } else {
            msg.extend_from_slice(&[0u8; 3]);
        }
        msg.push(state.serial[8]);
        if mode & 0x40 != 0 {
            msg.extend_from_slice(&state.serial[4..8]);
        } else {
            msg.extend_from_slice(&[0u8; 4]);
        }
        msg.extend_from_slice(&state.serial[0..2]);
        if mode & 0x40 != 0 {
            msg.extend_from_slice(&state.serial[2..4]);
        } else {
            msg.extend_from_slice(&[0u8; 2]);
        }
        msg
    }

    /// `MAC` (§9.2): a digest over a key and a challenge.
    fn op_mac(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if mode & 0x88 != 0 {
            // Bits 3 and 7 are reserved and must be zero.
            return Err(STATUS_PARSE);
        }
        let slot = usize::from(param2 & 0x0f);
        // Mode bit 1: the first thirty-two bytes are TempKey rather than a key.
        let first = if mode & 0x02 != 0 {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            state.temp_key
        } else {
            state.key_bytes(slot)
        };
        // Mode bit 0: the second thirty-two are TempKey rather than the
        // challenge in the packet.
        let second = if mode & 0x01 != 0 {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            state.temp_key
        } else {
            if data.len() != 32 {
                return Err(STATUS_PARSE);
            }
            let mut c = [0u8; 32];
            c.copy_from_slice(data);
            c
        };
        // Mode bit 2 must agree with TempKey.SourceFlag whenever TempKey is
        // used at all — the check that stops a host from passing its own value
        // off as an internally generated nonce.
        if mode & 0x03 != 0 {
            let claims_input = mode & 0x04 != 0;
            if claims_input != state.temp_key_from_input {
                return Err(STATUS_EXEC);
            }
        }
        let msg = Shared::mac_message(state, OP_MAC, mode, param2, &first, &second);
        Ok(Sha256::digest(&msg).as_ref().to_vec())
    }

    /// `CheckMac` (§9.2): verify a client's response.
    ///
    /// The message is the `MAC` layout with the thirteen bytes of `OtherData`
    /// standing in for the opcode, mode, key id and serial-number fields — the
    /// client sent them, so the part must use the client's copies.
    fn op_checkmac(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if data.len() != 77 {
            // 32 client challenge + 32 client response + 13 OtherData.
            return Err(STATUS_PARSE);
        }
        let slot = usize::from(param2 & 0x0f);
        let other = &data[64..77];
        // The same two mode bits `MAC` uses, and deliberately so: bit 1 puts
        // `TempKey` in the first thirty-two bytes, bit 0 in the second.
        let first = if mode & 0x02 != 0 {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            state.temp_key
        } else {
            state.key_bytes(slot)
        };
        let second = if mode & 0x01 != 0 {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            state.temp_key
        } else {
            let mut c = [0u8; 32];
            c.copy_from_slice(&data[0..32]);
            c
        };

        let mut msg = Vec::with_capacity(88);
        msg.extend_from_slice(&first);
        msg.extend_from_slice(&second);
        msg.extend_from_slice(&other[0..4]);
        if mode & 0x20 != 0 {
            msg.extend_from_slice(&state.otp[0..8]);
        } else {
            msg.extend_from_slice(&[0u8; 8]);
        }
        msg.extend_from_slice(&other[4..7]);
        msg.push(state.serial[8]);
        msg.extend_from_slice(&other[7..11]);
        msg.extend_from_slice(&state.serial[0..2]);
        msg.extend_from_slice(&other[11..13]);
        let expect = Sha256::digest(&msg);
        if expect.as_ref() == &data[32..64] {
            Ok(alloc::vec![STATUS_OK])
        } else {
            Err(STATUS_MISCOMPARE)
        }
    }

    /// `HMAC` (§9.2, 508A): HMAC-SHA-256 over the `MAC` layout, keyed by the
    /// slot, with `TempKey` as the second thirty-two bytes and zeros as the
    /// first.
    fn op_hmac(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        if !self.part.has_hmac() {
            // The 608 dropped the command; `KDF` replaced it.
            return Err(STATUS_PARSE);
        }
        if !state.temp_key_valid {
            return Err(STATUS_EXEC);
        }
        let slot = usize::from(param2 & 0x0f);
        let key = state.key_bytes(slot);
        let temp = state.temp_key;
        let msg = Shared::mac_message(state, OP_HMAC, mode, param2, &[0u8; 32], &temp);
        Ok(HmacSha256::mac(&key, &msg).as_ref().to_vec())
    }

    /// `SHA` (§9.2): a SHA-256 context the host drives a block at a time.
    ///
    /// The message is accumulated rather than the compression function's
    /// chaining value being kept, and that is deliberate: `purecrypto`'s
    /// `Digest` cannot export mid-message state, and a snapshot has to be able
    /// to carry whatever a half-finished context holds. The visible behaviour
    /// is identical — a digest is a function of the message — and what it costs
    /// is the message's own length in RAM, bounded by [`SHA_MAX`].
    fn op_sha(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        match mode & 0x07 {
            // Start.
            0x00 => {
                state.sha_msg.clear();
                state.sha_active = true;
                Ok(alloc::vec![STATUS_OK])
            }
            // Update: one 64-byte block.
            0x01 => {
                if !state.sha_active {
                    return Err(STATUS_EXEC);
                }
                if data.len() != 64 {
                    return Err(STATUS_PARSE);
                }
                if state.sha_msg.len() + data.len() > SHA_MAX {
                    return Err(STATUS_EXEC);
                }
                state.sha_msg.extend_from_slice(data);
                Ok(alloc::vec![STATUS_OK])
            }
            // End: nought to sixty-three trailing bytes, and the digest.
            0x02 => {
                if !state.sha_active {
                    return Err(STATUS_EXEC);
                }
                if data.len() != usize::from(param2) || data.len() > 63 {
                    return Err(STATUS_PARSE);
                }
                if state.sha_msg.len() + data.len() > SHA_MAX {
                    return Err(STATUS_EXEC);
                }
                state.sha_msg.extend_from_slice(data);
                let digest = Sha256::digest(&state.sha_msg);
                state.sha_msg.clear();
                state.sha_active = false;
                // §9.2: the digest also lands in TempKey, which is what lets a
                // host sign a message longer than a nonce.
                state.temp_key.copy_from_slice(digest.as_ref());
                state.temp_key_valid = true;
                state.temp_key_from_input = true;
                state.temp_key_gendig = false;
                Ok(digest.as_ref().to_vec())
            }
            _ => Err(STATUS_PARSE),
        }
    }

    /// `GenKey` (§9.2): make a P-256 key pair, or recompute a public key.
    fn op_genkey(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        if !state.config_locked() {
            // A key generated into a slot whose `KeyConfig` could still change
            // would be a key with no policy.
            return Err(STATUS_EXEC);
        }
        let slot = usize::from(param2 & 0x0f);
        if !state.is_private(slot) {
            return Err(STATUS_EXEC);
        }
        if mode & 0x04 != 0 {
            // Create: a new private key, from the board's seed.
            if state.slot_locked(slot) {
                return Err(STATUS_EXEC);
            }
            let key = generate_private_key(&mut state.stream);
            state.set_key_bytes(slot, &key);
        }
        let private = state.key_bytes(slot);
        let key = EcdsaPrivateKey::from_bytes(&private).map_err(|_| STATUS_ECC_FAULT)?;
        Ok(public_xy(&key.public_key()).to_vec())
    }

    /// `PrivWrite` (§9.2): put a private key the host chose into a slot.
    fn op_privwrite(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if mode & 0x40 != 0 {
            // The encrypted form needs a `GenDig` session key; the clear form
            // is the one this model implements, and §9.2 allows it only before
            // the data zone is locked anyway.
            return Err(STATUS_EXEC);
        }
        if data.len() != 36 {
            return Err(STATUS_PARSE);
        }
        if state.data_locked() {
            return Err(STATUS_EXEC);
        }
        let slot = usize::from(param2 & 0x0f);
        if !state.is_private(slot) {
            return Err(STATUS_EXEC);
        }
        let mut key = [0u8; 32];
        key.copy_from_slice(&data[4..36]);
        EcdsaPrivateKey::from_bytes(&key).map_err(|_| STATUS_PARSE)?;
        state.set_key_bytes(slot, &key);
        Ok(alloc::vec![STATUS_OK])
    }

    /// `Sign` (§9.2): ECDSA over the digest in `TempKey`.
    fn op_sign(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
    ) -> core::result::Result<Vec<u8>, u8> {
        // Bit 7 is the external-message mode, which is the one a host uses to
        // sign something it hashed itself. The internal mode signs a message
        // the part assembles from its own state and is not modelled.
        if mode & 0x80 == 0 {
            return Err(STATUS_PARSE);
        }
        if !state.temp_key_valid {
            return Err(STATUS_EXEC);
        }
        let slot = usize::from(param2 & 0x0f);
        if !state.is_private(slot) {
            return Err(STATUS_EXEC);
        }
        let private = state.key_bytes(slot);
        let key = EcdsaPrivateKey::from_bytes(&private).map_err(|_| STATUS_ECC_FAULT)?;
        let sig = key
            .sign_prehash::<Sha256>(&state.temp_key)
            .map_err(|_| STATUS_ECC_FAULT)?;
        Ok(sig.to_bytes().to_vec())
    }

    /// `Verify` (§9.2): check a signature over the digest in `TempKey`.
    fn op_verify(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if !state.temp_key_valid {
            return Err(STATUS_EXEC);
        }
        let (sig_bytes, public) = match mode & 0x03 {
            // Stored: the public key is in the slot `param2` names.
            0x00 => {
                if data.len() != 64 {
                    return Err(STATUS_PARSE);
                }
                let slot = usize::from(param2 & 0x0f);
                let base = slot_offset(slot) as usize;
                if SLOT_SIZES[slot] < 72 {
                    return Err(STATUS_EXEC);
                }
                // §2.1: a stored public key occupies 72 bytes, X and Y each
                // padded to 36 with four leading zeros.
                let mut xy = [0u8; 64];
                xy[0..32].copy_from_slice(&state.data[base + 4..base + 36]);
                xy[32..64].copy_from_slice(&state.data[base + 40..base + 72]);
                (&data[0..64], xy)
            }
            // External: the host supplied the key after the signature.
            0x02 => {
                if data.len() != 128 {
                    return Err(STATUS_PARSE);
                }
                let mut xy = [0u8; 64];
                xy.copy_from_slice(&data[64..128]);
                (&data[0..64], xy)
            }
            _ => return Err(STATUS_PARSE),
        };
        let mut sec1 = [0u8; 65];
        sec1[0] = 0x04;
        sec1[1..65].copy_from_slice(&public);
        let key = EcdsaPublicKey::from_sec1(&sec1).map_err(|_| STATUS_ECC_FAULT)?;
        let mut raw = [0u8; 64];
        raw.copy_from_slice(sig_bytes);
        let sig = Signature::from_bytes(&raw);
        match key.verify_prehash(&state.temp_key, &sig) {
            Ok(()) => Ok(alloc::vec![STATUS_OK]),
            Err(_) => Err(STATUS_MISCOMPARE),
        }
    }

    /// `ECDH` (§9.2): the X coordinate of the shared point.
    fn op_ecdh(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if data.len() != 64 {
            return Err(STATUS_PARSE);
        }
        let slot = usize::from(param2 & 0x0f);
        if !state.is_private(slot) {
            return Err(STATUS_EXEC);
        }
        let private = state.key_bytes(slot);
        let key = EcdhPrivateKey::from_bytes(&private).map_err(|_| STATUS_ECC_FAULT)?;
        let mut sec1 = [0u8; 65];
        sec1[0] = 0x04;
        sec1[1..65].copy_from_slice(data);
        let peer = EcdsaPublicKey::from_sec1(&sec1).map_err(|_| STATUS_ECC_FAULT)?;
        let shared = key.diffie_hellman(&peer).map_err(|_| STATUS_ECC_FAULT)?;
        if mode & 0x01 != 0 {
            // Into the slot above the key, rather than out on the wire.
            let target = slot + 1;
            if target >= SLOTS {
                return Err(STATUS_EXEC);
            }
            state.set_key_bytes(target, &shared);
            return Ok(alloc::vec![STATUS_OK]);
        }
        Ok(shared.to_vec())
    }

    /// `AES` (§9.2, 608 only): one AES-128 block.
    fn op_aes(
        &self,
        state: &mut State,
        mode: u8,
        param2: u16,
        data: &[u8],
    ) -> core::result::Result<Vec<u8>, u8> {
        if !self.part.has_aes() {
            return Err(STATUS_PARSE);
        }
        if data.len() != 16 {
            return Err(STATUS_PARSE);
        }
        // `param2` of 0xFFFF names TempKey rather than a slot, which is how a
        // host encrypts under a key it just derived.
        let key = if param2 == 0xffff {
            if !state.temp_key_valid {
                return Err(STATUS_EXEC);
            }
            let mut k = [0u8; 16];
            k.copy_from_slice(&state.temp_key[0..16]);
            k
        } else {
            let slot = usize::from(param2 & 0x0f);
            let block = usize::from((mode >> 6) & 0x03);
            let full = state.key_bytes(slot);
            let at = (block * 16) % 32;
            let mut k = [0u8; 16];
            k.copy_from_slice(&full[at..at + 16]);
            k
        };
        let cipher = Aes128::new(&key);
        let mut block = [0u8; 16];
        block.copy_from_slice(data);
        match mode & 0x07 {
            0x00 => cipher.encrypt_block(&mut block),
            0x01 => cipher.decrypt_block(&mut block),
            _ => return Err(STATUS_PARSE),
        }
        Ok(block.to_vec())
    }

    /// `SelfTest` (§9.2, 608): every test passes, because every engine here is
    /// software that either works or does not compile.
    fn op_selftest(&self, _state: &State, mode: u8) -> core::result::Result<Vec<u8>, u8> {
        if !self.part.has_aes() {
            return Err(STATUS_PARSE);
        }
        let _ = mode;
        Ok(alloc::vec![STATUS_OK])
    }

    /// `Pause` (§9.2, 508A): a part whose `Selector` does not match the
    /// argument goes idle, which is how several parts share one bus.
    fn op_pause(&self, state: &mut State, selector: u8) -> core::result::Result<Vec<u8>, u8> {
        if state.config[SELECTOR_OFFSET] != selector {
            state.power = Power::Idle;
        }
        Ok(alloc::vec![STATUS_OK])
    }
}

/// How long a message a `SHA` context will accumulate before refusing more.
///
/// The real part has no such limit — it keeps a chaining value, not a message —
/// so this is the model's own bound, and it is here so a guest cannot make the
/// emulator allocate without end. Sixty-four kilobytes is far more than the
/// certificate-sized messages the command exists for.
const SHA_MAX: usize = 64 * 1024;

/// Draw a private key the curve accepts.
///
/// FIPS 186-4 B.4.2's "testing candidates" — take 32 bytes, reject anything
/// that is not in `[1, n-1]`, try again — with the bytes coming from the
/// board's seed rather than from an entropy source, which is what makes a
/// generated key reproducible.
fn generate_private_key(stream: &mut Stream) -> [u8; 32] {
    loop {
        let mut candidate = [0u8; 32];
        stream.fill(&mut candidate);
        if EcdsaPrivateKey::from_bytes(&candidate).is_ok() {
            return candidate;
        }
    }
}

/// A public key as the part hands it over: X ‖ Y, sixty-four bytes, with no
/// SEC1 tag (§9.2, `GenKey`).
fn public_xy(key: &EcdsaPublicKey) -> [u8; 64] {
    let sec1 = key.to_sec1();
    let mut xy = [0u8; 64];
    xy.copy_from_slice(&sec1[1..65]);
    xy
}

/// Split `param2` into an offset within its zone and, for the data zone, the
/// slot it names (§9.2, "Address Encoding").
///
/// Bits 2:0 are the word offset within a 32-byte block, bits 6:3 the slot and
/// bits 11:8 the block. A word is four bytes, so the byte offset is
/// `block * 32 + word * 4`.
fn decode_address(zone: Zone, param2: u16) -> core::result::Result<(u64, usize), u8> {
    let word = u64::from(param2 & 0x07);
    let slot = usize::from((param2 >> 3) & 0x0f);
    let block = u64::from((param2 >> 8) & 0x0f);
    let offset = block * 32 + word * 4;
    match zone {
        Zone::Config => {
            if offset as usize >= CONFIG_BYTES {
                return Err(STATUS_PARSE);
            }
            Ok((offset, 0))
        }
        Zone::Otp => {
            if offset as usize >= OTP_BYTES {
                return Err(STATUS_PARSE);
            }
            Ok((offset, 0))
        }
        Zone::Data => {
            if offset >= SLOT_SIZES[slot] {
                return Err(STATUS_PARSE);
            }
            Ok((offset, slot))
        }
    }
}

/// `len` bytes of `zone` from `offset`, or a parse error if they are not there.
fn read_slice(zone: &[u8], offset: u64, len: usize) -> core::result::Result<Vec<u8>, u8> {
    let at = offset as usize;
    zone.get(at..at + len)
        .map(<[u8]>::to_vec)
        .ok_or(STATUS_PARSE)
}

/// Put `bytes` into `zone` at `offset`, or a parse error if they do not fit.
fn write_slice(zone: &mut [u8], offset: u64, bytes: &[u8]) -> core::result::Result<(), u8> {
    let at = offset as usize;
    let slice = zone.get_mut(at..at + bytes.len()).ok_or(STATUS_PARSE)?;
    slice.copy_from_slice(bytes);
    Ok(())
}

/// What a well-formed packet says: opcode, `param1`, `param2` and the data.
type Parsed<'a> = (u8, u8, u16, &'a [u8]);

/// Check a command packet's framing and CRC (§9.1.1, §9.1.3), or give back the
/// status byte the part answers a malformed one with.
fn validate(packet: &[u8]) -> core::result::Result<Parsed<'_>, u8> {
    if packet.len() < 7 {
        return Err(STATUS_CRC);
    }
    let count = usize::from(packet[0]);
    if count != packet.len() || count < 7 {
        return Err(STATUS_CRC);
    }
    let crc = crc16(&packet[..count - 2]);
    if crc != packet[count - 2..count] {
        return Err(STATUS_CRC);
    }
    let opcode = packet[1];
    let param1 = packet[2];
    let param2 = u16::from(packet[3]) | (u16::from(packet[4]) << 8);
    Ok((opcode, param1, param2, &packet[5..count - 2]))
}

// ---------------------------------------------------------------------------
// The I2C face
// ---------------------------------------------------------------------------

impl I2cSlave for Shared {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let mut state = self.state.lock();
        let now = self.now(&state);
        self.expire_watchdog(&mut state, now);
        let Address::Seven(a) = address else {
            return Ack::Nack;
        };
        if a == WAKE_ADDRESS {
            // §6.1: SDA held low is the wake, and a host produces it by
            // addressing zero. The part does not acknowledge the token.
            if dir == Direction::Write {
                self.wake(&mut state, now);
            }
            return Ack::Nack;
        }
        if a != self.address {
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        if state.power != Power::Awake {
            // Asleep or idle: the part is not on the bus at all (§6.2).
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        if self.is_busy(&state, now) {
            // §9.4: while a command runs the part answers nothing, which is
            // what makes a driver's poll loop terminate.
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        match dir {
            Direction::Write => {
                state.phase = Phase::WantWord;
                Ack::Ack
            }
            Direction::Read => {
                if state.tx_pos >= state.tx.len() {
                    // Nothing to hand over. A read here is a driver bug, and
                    // the part's answer to one is silence.
                    state.phase = Phase::Idle;
                    return Ack::Nack;
                }
                state.phase = Phase::Reading;
                Ack::Ack
            }
        }
    }

    fn write(&self, byte: u8) -> Ack {
        let mut state = self.state.lock();
        match state.phase {
            Phase::WantWord => {
                // §7.1: the first byte after the address is the word address.
                match byte {
                    WORD_RESET => {
                        state.tx_pos = 0;
                        state.phase = Phase::AfterWord;
                        Ack::Ack
                    }
                    WORD_SLEEP => {
                        self.go_to_sleep(&mut state);
                        self.publish(&state);
                        Ack::Ack
                    }
                    WORD_IDLE => {
                        self.go_idle(&mut state);
                        self.publish(&state);
                        Ack::Ack
                    }
                    WORD_COMMAND => {
                        state.rx.clear();
                        state.phase = Phase::Command;
                        Ack::Ack
                    }
                    _ => {
                        state.phase = Phase::Idle;
                        Ack::Nack
                    }
                }
            }
            Phase::Command => {
                // §9.1: the packet is collected whole and executed at the STOP.
                if state.rx.len() >= MAX_PACKET {
                    return Ack::Nack;
                }
                state.rx.push(byte);
                Ack::Ack
            }
            Phase::Idle | Phase::AfterWord | Phase::Reading => Ack::Nack,
        }
    }

    fn read(&self) -> u8 {
        let state = self.state.lock();
        if state.phase != Phase::Reading {
            return 0xff;
        }
        state.tx.get(state.tx_pos).copied().unwrap_or(0xff)
    }

    fn read_ack(&self, ack: Ack) {
        let mut state = self.state.lock();
        if state.tx_pos < state.tx.len() {
            state.tx_pos += 1;
        }
        if !ack.is_ack() {
            state.phase = Phase::Idle;
        }
    }

    fn stop(&self) {
        let mut state = self.state.lock();
        if state.phase == Phase::Command && !state.rx.is_empty() {
            let now = self.now(&state);
            self.start_command(&mut state, now);
        }
        state.phase = Phase::Idle;
    }

    fn peek(&self) -> u8 {
        // The `MemAttrs::debug` rule on a bus: a monitor's look must not move
        // the response pointer.
        let state = self.state.lock();
        state.tx.get(state.tx_pos).copied().unwrap_or(0xff)
    }
}

/// The longest packet the part will collect.
///
/// §9.1.1 makes the count byte one byte wide, so a packet cannot exceed 255
/// bytes and a host that keeps writing is one whose framing has already gone
/// wrong.
const MAX_PACKET: usize = 255;

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Atecc {
    fn class(&self) -> &'static DeviceClass {
        &ATECC_CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The path is only available here, and it is what makes two of these
        // on one board hold different keys.
        self.seed_from_path(ctx.path());
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // A reset is a power cycle: the part comes up asleep with nothing
            // volatile in it. The zones survive — they are EEPROM — and so does
            // the tick, because `Machine::reset` does not rewind clock domains
            // (`ROADMAP.md` §4.2).
            self.shared.go_to_sleep(&mut state);
            state.awake_until = 0;
            state.busy_until = 0;
            state.counters = [0; 2];
            // The stream goes back to its seed, so a reset-and-rerun deals the
            // same numbers and generates the same keys.
            state.stream.rewind();
            self.shared.publish(&state);
        }
        self.wires.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_bytes(&state.config)?;
        w.write_bytes(&state.otp)?;
        w.write_bytes(&state.data)?;
        w.write_bytes(&state.serial)?;
        w.write_u8(state.power.code())?;
        w.write_u64(state.awake_until)?;
        w.write_bool(state.busy)?;
        w.write_u64(state.busy_until)?;
        w.write_u8(state.phase.code())?;
        w.write_bytes(&state.rx)?;
        w.write_bytes(&state.tx)?;
        w.write_u64(state.tx_pos as u64)?;
        w.write_bytes(&state.temp_key)?;
        w.write_bool(state.temp_key_valid)?;
        w.write_bool(state.temp_key_from_input)?;
        w.write_bool(state.temp_key_gendig)?;
        w.write_bytes(&state.sha_msg)?;
        w.write_bool(state.sha_active)?;
        w.write_u32(state.counters[0])?;
        w.write_u32(state.counters[1])?;
        // The stream's *position*: a snapshot taken mid-stream resumes
        // mid-stream rather than replaying numbers a guest already has.
        state.stream.save(w)?;
        drop(state);
        self.wires.snapshot().write(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        let config = r.read_bytes()?.to_vec();
        let otp = r.read_bytes()?.to_vec();
        let data = r.read_bytes()?.to_vec();
        let serial = r.read_bytes()?.to_vec();
        let power = Power::from_code(r.read_u8()?);
        let awake_until = r.read_u64()?;
        let busy = r.read_bool()?;
        let busy_until = r.read_u64()?;
        let phase = Phase::from_code(r.read_u8()?);
        let rx = r.read_bytes()?.to_vec();
        let tx = r.read_bytes()?.to_vec();
        let tx_pos = r.read_u64()?;
        let temp_key = r.read_bytes()?.to_vec();
        let temp_key_valid = r.read_bool()?;
        let temp_key_from_input = r.read_bool()?;
        let temp_key_gendig = r.read_bool()?;
        let sha_msg = r.read_bytes()?.to_vec();
        let sha_active = r.read_bool()?;
        let counters = [r.read_u32()?, r.read_u32()?];

        {
            let mut state = self.shared.state.lock();
            // A snapshot is untrusted input: a zone of the wrong length is
            // dropped rather than resized, which would move every slot.
            if config.len() == state.config.len() {
                state.config = config;
            }
            if otp.len() == state.otp.len() {
                state.otp = otp;
            }
            if data.len() == state.data.len() {
                state.data = data;
            }
            if serial.len() == 9 {
                state.serial.copy_from_slice(&serial);
            }
            if temp_key.len() == 32 {
                state.temp_key.copy_from_slice(&temp_key);
            }
            state.ticks = ticks;
            state.power = power;
            state.awake_until = awake_until;
            state.busy = busy;
            state.busy_until = busy_until;
            state.phase = phase;
            state.rx = rx;
            state.tx = tx;
            state.tx_pos = (tx_pos as usize).min(state.tx.len());
            state.temp_key_valid = temp_key_valid;
            state.temp_key_from_input = temp_key_from_input;
            state.temp_key_gendig = temp_key_gendig;
            state.sha_msg = sha_msg;
            state.sha_active = sha_active;
            state.counters = counters;
            // The seed comes from the machine description and stays where it
            // is; only the *position* is in the snapshot (`core::rand`).
            state.stream.load(r)?;
            self.shared.publish(&state);
        }
        let bits = SlaveWiresState::read(r)?;
        self.wires.restore(bits);
        Ok(())
    }

    fn sink(
        &self,
        port: &str,
        sources: &[crate::core::wire::WireId],
    ) -> Option<crate::core::device::SinkPin> {
        match port {
            line::SCL_NAME => Some(crate::core::device::SinkPin {
                sink: self.wires.sink(line::SCL, sources),
                line: line::SCL,
            }),
            line::SDA_NAME => Some(crate::core::device::SinkPin {
                sink: self.wires.sink(line::SDA, sources),
                line: line::SDA,
            }),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            line::SCL_NAME => self.wires.connect(line::SCL, source),
            line::SDA_NAME => self.wires.connect(line::SDA, source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an ATECC drives only `{}` and `{}`, and only ever low: both are \
                         open-drain (§7.1)",
                        line::SCL_NAME,
                        line::SDA_NAME
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.wires.announce();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes, for two reasons: a command's execution time (§9.4), during which
    /// the part NACKs, and the watchdog (§6.3), which fires with nobody
    /// talking to the part at all.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Atecc::advance_to(self, tick);
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

impl Instance for Atecc {}

/// The `atmel.atecc` device class.
pub static ATECC_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Microchip ATECC508A/608A/608B CryptoAuthentication secure element on I2C: zones \
              and locks, the wake/sleep watchdog, CRC-16 packets, SHA/HMAC/MAC, P-256 keygen, \
              sign, verify and ECDH",
    properties: &[
        PropertySpec {
            name: "part",
            kind: ValueKind::Str,
            required: false,
            summary: "atecc508a, atecc608a or atecc608b (default atecc608a)",
        },
        PropertySpec {
            name: "seed",
            kind: ValueKind::Uint,
            required: false,
            summary: "the board's seed; mixed with this instance's path at realize",
        },
        PropertySpec {
            name: "address",
            kind: ValueKind::Uint,
            required: false,
            summary: "the seven-bit I2C address (default 0x60, from I2C_Address = 0xC0)",
        },
        PropertySpec {
            name: "config",
            kind: ValueKind::Media,
            required: false,
            summary: "the initial configuration zone; bytes 0-15 are the part's own (§2.2)",
        },
        PropertySpec {
            name: "otp",
            kind: ValueKind::Media,
            required: false,
            summary: "the initial OTP zone (64 bytes)",
        },
        PropertySpec {
            name: "data",
            kind: ValueKind::Media,
            required: false,
            summary: "the initial data zone (1208 bytes, sixteen slots end to end)",
        },
        PropertySpec {
            name: "lock",
            kind: ValueKind::Str,
            required: false,
            summary: "which zones come up locked: none, config or data (default none)",
        },
        PropertySpec {
            name: "exec-scale",
            kind: ValueKind::Uint,
            required: false,
            summary: "multiplier on the datasheet execution times, for a domain that is not 1 MHz",
        },
        PropertySpec {
            name: "watchdog-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "tWATCHDOG in ticks of this device's clock domain (§6.3; 0 disables it)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named I2C bus to hang off, for a transactional link",
        },
    ],
    construct: |props| Ok(Box::new(Atecc::new(props)?)),
};

/// Add [`ATECC_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ATECC_CLASS)
}

/// Bind [`ATECC_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Atecc::new(props)?)))
}

/// What the validator should know about `atmel.atecc`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("part", ValueKind::Str))
        .prop(PropSchema::new("seed", ValueKind::Uint))
        .prop(PropSchema::new("address", ValueKind::Uint).range(0, 0x7f))
        .prop(PropSchema::new("config", ValueKind::Media))
        .prop(PropSchema::new("otp", ValueKind::Media))
        .prop(PropSchema::new("data", ValueKind::Media))
        .prop(PropSchema::new("lock", ValueKind::Str))
        .prop(PropSchema::new("exec-scale", ValueKind::Uint))
        .prop(PropSchema::new("watchdog-ticks", ValueKind::Uint))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .port(line::SCL_NAME, PortDir::InOut)
        .port(line::SDA_NAME, PortDir::InOut)
}
