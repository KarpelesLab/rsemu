//! The Atmel AT24C01D / AT24C02D I²C serial EEPROM.
//!
//! 1 Kbit (128 × 8) or 2 Kbit (256 × 8) of non-volatile storage on two wires —
//! the part that hangs off more embedded I²C buses than anything else, and the
//! smallest honest device that proves a bus works: a guest that writes a page
//! and reads it back has exercised addressing, the direction bit, per-byte
//! acknowledges, the repeated START, and the device's own internal timing.
//!
//! # Source
//!
//! Atmel **AT24C01D and AT24C02D** datasheet,
//! `Atmel-8871F-SEEPROM-AT24C01D-02D-Datasheet_012017`, cited by section
//! throughout. No emulator was consulted (`ROADMAP.md` §1).
//!
//! # What is modelled
//!
//! * **Device addressing** (§4.1, Table 4-1): `1010 A2 A1 A0` plus `R/W̅`. The
//!   three hardware pins are the [`chip`](AT24C_CLASS) property, so the eight
//!   parts a board may cascade are eight objects on one bus.
//! * **Byte write** (§5.1) and **page write** (§5.2), including the roll-over
//!   that makes a page write past the page boundary overwrite the *start of the
//!   same page* rather than the next one — the trap the datasheet warns about
//!   and the one a driver bug looks like.
//! * The **internally self-timed write cycle** (§5.4): after the STOP that ends
//!   a write, the part answers nothing at all for tWR, so
//!   **acknowledge polling** (§5.3) works exactly as the flow chart says — the
//!   device NACKs its own address until the cycle finishes.
//! * **Current address read** (§6.1), **random read** (§6.2) and **sequential
//!   read** (§6.3), with the read counter's roll-over over the *whole array*
//!   rather than within a page.
//! * **Write protection** (§5.5): `WP` high protects the full array, and it is
//!   sampled **at the STOP condition** — so a `WP` that goes high mid-transfer
//!   still blocks the write, and the part still acknowledges every byte of it.
//!
//! # What is not
//!
//! The 4K/8K/16K members of the family, which steal `A2`/`A1`/`A0` from the
//! device address byte to extend the memory address. They are a different
//! addressing scheme rather than a bigger version of this one, and modelling
//! them as a `size` property would quietly produce a part that does not exist.
//! [`AT24C_CLASS`] therefore refuses a `size` above 256.
//!
//! Also absent: the software reset sequence of §3.5 (there is no protocol state
//! this model can get stuck in that a START does not clear) and the AC timing
//! of §8.4 (a bit-level timing violation is not something an emulated master
//! can commit — it clocks in half periods).
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). This is a *lazily advanced* device
//! (`ROADMAP.md` §4.2): it holds its own tick, publishes the tick its write
//! cycle finishes on, and is caught up before anything touches it. It never
//! sleeps and never reads a clock.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::i2c::wires::{SlaveWires, SlaveWiresState, pin as line};
use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

#[cfg(test)]
mod tests;

/// The class name a machine description writes.
const CLASS_NAME: &str = "atmel.at24c";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The four-bit device type identifier every 24Cxx answers to: `1010`
/// (§4.1, Table 4-1).
pub const DEVICE_TYPE: u8 = 0b1010;

/// The base seven-bit address, with all three hardware pins low.
pub const BASE_ADDRESS: u8 = DEVICE_TYPE << 3;

/// The AT24C02D's array: 256 words of eight bits (§4).
pub const DEFAULT_SIZE: u64 = 256;

/// One page, in bytes. Eight for both parts (§4: "16 pages of 8 bytes each …
/// 32 pages of 8 bytes each").
pub const DEFAULT_PAGE: u64 = 8;

/// The biggest array this model accepts.
///
/// One byte of word address (§4.1, Table 4-2) is the whole reason: a larger
/// 24Cxx does not widen this field, it moves the extra bits into the *device*
/// address, and that is a different part. See the module docs.
pub const MAX_SIZE: u64 = 256;

/// The default internal write cycle, in ticks of this device's clock domain.
///
/// §5.4 and the feature list give tWR as 5 ms maximum. Expressed in ticks
/// rather than in seconds because the time path has no floats (`CLAUDE.md`) and
/// a device does not own a frequency: a board that clocks this part from a
/// 1 MHz domain gets exactly 5 ms from the default.
pub const DEFAULT_WRITE_TICKS: u64 = 5_000;

/// What the device is doing between START and STOP.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// Not addressed.
    #[default]
    Idle,
    /// Addressed for a write; the next byte is the word address (§4.1: "For all
    /// operations except the Current Address Read, a Word Address byte must be
    /// transmitted to the device immediately following the Device Address").
    WantWordAddress,
    /// The word address arrived; further bytes are data for the page buffer.
    Writing,
    /// Addressed for a read.
    Reading,
}

/// The pin names a machine description wires.
pub mod pin {
    /// The write-protect input (§5.5).
    pub const WP: &str = "wp";
    /// The wire line number the `WP` sink answers on.
    ///
    /// Numbered past [`crate::bus::i2c::wires::pin::SDA`] so one device can host
    /// the two bus lines and this one without their line numbers colliding.
    pub const WP_LINE: u32 = 2;
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An AT24C01D/02D serial EEPROM.
#[derive(Debug)]
pub struct At24c {
    shared: Arc<Shared>,
    wires: Arc<SlaveWires>,
    /// The bus to hook onto at realize time, if the machine named one.
    bus: Option<Arc<I2cBus>>,
    /// The `WP` pin, kept alive here because the net that receives it holds
    /// only a weak reference (`core::device`, §4.3's weak edge).
    wp_pin: Mutex<Option<Arc<WriteProtectSink>>>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// How many bytes the array holds. A power of two, at most [`MAX_SIZE`].
    size: u64,
    /// One page, in bytes. A power of two dividing `size`.
    page: u64,
    /// The seven-bit address the three hardware pins select (§4.1).
    address: u8,
    /// tWR, in ticks of this device's clock domain (§5.4).
    write_ticks: u64,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick the write cycle ends on, or [`NO_EVENT`].
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
    /// The array.
    mem: Vec<u8>,
    /// The internal data word address counter (§6.1).
    word: u64,
    /// Where in the transaction we are.
    phase: Phase,
    /// The page a write in progress is landing in.
    page_base: u64,
    /// The bytes it has staged, one slot per byte of the page.
    page_data: Vec<u8>,
    /// Which of those slots the master has actually written.
    page_touched: Vec<bool>,
    /// The tick the internal write cycle ends on, or 0 when idle.
    busy_until: u64,
    /// Whether a write cycle is running at all.
    busy: bool,
    /// The level on `WP`, and whether anything drives it.
    ///
    /// The default is **low**, and that is the datasheet's answer rather than
    /// an invented one: Table 1-1's note 1 says the `WP` pin is internally
    /// pulled down to ground when it is not driven, and §5.5's Table 5-1 says a
    /// grounded `WP` means "None — Write Protection Not Enabled".
    wp: Level,
    /// Whether a machine description wired `WP` at all, for `Debug` and for a
    /// test that wants to tell "tied low by the board" from "driven low".
    wp_wired: bool,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("At24cShared");
        s.field("address", &alloc::format!("{:#04x}", self.address));
        s.field("size", &self.size);
        s.field("page", &self.page);
        match self.state.try_lock() {
            Some(state) => s
                .field("phase", &state.phase)
                .field("word", &state.word)
                .field("busy", &state.busy),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

impl At24c {
    /// Validate `props` and build the part.
    ///
    /// Properties:
    ///
    /// * `chip` — the three hardware address pins `A2 A1 A0` as one number, 0
    ///   to 7 (§4.1). The device answers `0x50 | chip`. Defaults to 0.
    /// * `size` — the array in bytes: 128 for an AT24C01D, 256 for an AT24C02D.
    ///   A power of two, at most [`MAX_SIZE`]. Defaults to 256.
    /// * `page` — one page in bytes, a power of two dividing `size`. Defaults
    ///   to 8, which is what both parts have (§4).
    /// * `write-ticks` — tWR in ticks of this device's clock domain (§5.4).
    ///   Defaults to [`DEFAULT_WRITE_TICKS`].
    /// * `image` — a media slot holding the initial contents. Shorter than the
    ///   array is fine; the rest stays erased. Absent means all `0xff`, which
    ///   is how the part is delivered (§7).
    /// * `bus` — the named [`I2cBus`] to hang off, for a transactional link. A
    ///   machine that only wires the pins up needs none.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for a
    /// `chip` above 7, a `size` that is not a power of two or is above
    /// [`MAX_SIZE`], a `page` that does not divide it, or an `image` longer
    /// than the array.
    pub fn new(props: &Props) -> Result<At24c> {
        let mut r = props.reader();
        let chip: u64 = r.or("chip", 0)?;
        let size: u64 = r.or_size("size", DEFAULT_SIZE)?;
        let page: u64 = r.or_size("page", DEFAULT_PAGE)?;
        let write_ticks: u64 = r.or("write-ticks", DEFAULT_WRITE_TICKS)?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        let bus_name = r.optional_str("bus")?.map(String::from);
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        if chip > 7 {
            return Err(bad(alloc::format!(
                "`chip` is {chip}; it is the three hardware pins A2 A1 A0 (datasheet §4.1), so 0 \
                 to 7"
            )));
        }
        if size == 0 || !size.is_power_of_two() || size > MAX_SIZE {
            return Err(bad(alloc::format!(
                "`size` is {size}; the word address is one byte (§4.1, Table 4-2), so this part \
                 holds a power of two up to {MAX_SIZE} — a 4K or larger 24Cxx moves the extra \
                 address bits into the device address and is a different part"
            )));
        }
        if page == 0 || !page.is_power_of_two() || page > size {
            return Err(bad(alloc::format!(
                "`page` is {page}; it must be a power of two no larger than the {size}-byte array"
            )));
        }
        let mut mem = alloc::vec![0xff_u8; size as usize];
        if let Some(image) = image {
            if image.len() as u64 > size {
                return Err(bad(alloc::format!(
                    "`image` is {} bytes and the array is {size}",
                    image.len()
                )));
            }
            mem[..image.len()].copy_from_slice(&image);
        }

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    ticks: 0,
                    mem,
                    word: 0,
                    phase: Phase::Idle,
                    page_base: 0,
                    page_data: alloc::vec![0; page as usize],
                    page_touched: alloc::vec![false; page as usize],
                    busy_until: 0,
                    busy: false,
                    wp: Level::Low,
                    wp_wired: false,
                },
            ),
            size,
            page,
            address: BASE_ADDRESS | (chip as u8),
            write_ticks,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        // Opening the bus is allocation: `buses::attach` is a get-or-create in
        // the build's own host-object table and nothing outside this machine can
        // see it (`core::hosts` argues why that belongs in `new`). *Hooking this
        // part onto it* is the outward half and happens in `realize`, which is
        // what two-phase construction asks for.
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        let wires = Arc::new(SlaveWires::new(Arc::clone(&shared) as Arc<dyn I2cSlave>));
        Ok(At24c {
            shared,
            wires,
            bus,
            wp_pin: Mutex::with_rank(LockRank::WIRE, None),
        })
    }

    /// The seven-bit address this part answers (§4.1).
    #[must_use]
    pub fn address(&self) -> Address {
        Address::Seven(self.shared.address)
    }

    /// How many bytes the array holds.
    #[must_use]
    pub fn size(&self) -> u64 {
        self.shared.size
    }

    /// One page, in bytes.
    #[must_use]
    pub fn page(&self) -> u64 {
        self.shared.page
    }

    /// This part as a bus device, for a controller that hands it whole bytes.
    ///
    /// The transactional half of the seam. A machine file gets the same thing
    /// by naming a `bus`; this is for an embedder or a test that owns its own
    /// [`I2cBus`].
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.shared) as Arc<dyn I2cSlave>
    }

    /// The part's wire pins, for a controller that drives them directly.
    #[must_use]
    pub fn wires(&self) -> &Arc<SlaveWires> {
        &self.wires
    }

    /// The internal data word address counter (§6.1).
    #[must_use]
    pub fn word_address(&self) -> u64 {
        self.shared.state.lock().word
    }

    /// Whether the internal write cycle is running (§5.4).
    ///
    /// While it is, the part answers nothing — which is what makes acknowledge
    /// polling (§5.3) work.
    #[must_use]
    pub fn busy(&self) -> bool {
        let state = self.shared.state.lock();
        self.shared.is_busy(&state)
    }

    /// One byte of the array, without touching the protocol state.
    ///
    /// The debug view: this is what a monitor or a test reads, and it moves no
    /// address counter.
    #[must_use]
    pub fn byte(&self, at: u64) -> Option<u8> {
        let state = self.shared.state.lock();
        state.mem.get(usize::try_from(at).ok()?).copied()
    }

    /// The whole array, copied.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        self.shared.state.lock().mem.clone()
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Run the part until `target` domain ticks have passed in total.
    ///
    /// Nothing here touches a wire. A part that stretched the clock would
    /// release SCL from this method — that is the contract
    /// [`SlaveWires::refresh_stretch`] documents — but this one has no SCL
    /// driver at all (datasheet Table 1-1 makes `SCL` an *Input*), so it never
    /// stretches and there is nothing to let go of.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(
            if state.busy {
                state.busy_until.max(state.ticks.saturating_add(1))
            } else {
                NO_EVENT
            },
            Ordering::Relaxed,
        );
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
        self.publish(&state);
    }

    /// Where this device's clock domain has got to.
    ///
    /// **Not [`LazyHandle::sync`]**, deliberately. The bus reaches this device
    /// from inside [`SlaveWires`]'s bit lock, and `sync` calls back into
    /// `advance_to`, which would drive a wire and re-enter that lock — a
    /// deadlock rather than a ladder violation, because the two locks are the
    /// same one. Asking where the domain *is* answers the only question this
    /// part has (has tWR elapsed?) and touches nothing: the scheduler
    /// republishes every lazy device's position after each advance of virtual
    /// time (`core::sched`, `publish_lazy_positions`).
    ///
    /// With no handle — a unit test holding the device directly — the device's
    /// own tick is the best answer there is.
    fn now(&self, state: &State) -> u64 {
        let handle = self.lazy.lock().clone();
        match handle {
            Some(handle) => handle.present_tick().max(state.ticks),
            None => state.ticks,
        }
    }

    /// Whether the internally self-timed write cycle is still running (§5.4).
    fn is_busy(&self, state: &State) -> bool {
        state.busy && self.now(state) < state.busy_until
    }

    /// Advance the word address counter after a write, which rolls over inside
    /// the page (§5.2).
    fn step_write(&self, state: &mut State) {
        let low = (state.word + 1) & (self.page - 1);
        state.word = (state.word & !(self.page - 1)) | low;
    }

    /// Advance it after a read, which rolls over across the whole array
    /// (§6.1: "from the last byte of the last page to the first byte of the
    /// first page").
    fn step_read(&self, state: &mut State) {
        state.word = (state.word + 1) & (self.size - 1);
    }

    /// Commit whatever a page write staged, and start the write cycle (§5.4).
    fn commit(&self, state: &mut State) {
        let touched = state.page_touched.iter().any(|t| *t);
        if !touched {
            // A "dummy write" — the address phase of a random read (§6.2) —
            // stages nothing, so no cycle begins. That is exactly why the
            // datasheet says its data byte and STOP must be omitted.
            return;
        }
        // §5.5: "The status of the WP pin is sampled at the Stop condition for
        // every Byte Write or Page Write command prior to the start of an
        // internally self-timed Write operation", and a protected part
        // "will acknowledge the Device Address, Word address, and Data bytes
        // but no write cycle will occur when the Stop condition is issued".
        if state.wp.is_low() {
            for i in 0..state.page_data.len() {
                if state.page_touched[i] {
                    let at = state.page_base + i as u64;
                    if let Some(slot) = state.mem.get_mut(at as usize) {
                        *slot = state.page_data[i];
                    }
                }
            }
            state.busy = true;
            state.busy_until = state.ticks.saturating_add(self.write_ticks);
        }
        state.page_touched.fill(false);
        self.publish(state);
    }
}

// ---------------------------------------------------------------------------
// The I2C face
// ---------------------------------------------------------------------------

impl I2cSlave for Shared {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let mut state = self.state.lock();
        let Address::Seven(a) = address else {
            // This part has no ten-bit address (§4.1 knows only the eight-bit
            // Device Address byte).
            return Ack::Nack;
        };
        if a != self.address {
            // §4.1: "If a valid comparison is not made, the device will NACK
            // and return to a standby state."
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        if self.is_busy(&state) {
            // §5.3: "The device will not respond with an ACK while the write
            // cycle is ongoing." This one line is acknowledge polling.
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        state.phase = match dir {
            Direction::Write => Phase::WantWordAddress,
            Direction::Read => Phase::Reading,
        };
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        let mut state = self.state.lock();
        match state.phase {
            Phase::WantWordAddress => {
                // §4.1, Table 4-2. On the AT24C01D the top bit is a don't care,
                // which masking to the array size is exactly.
                state.word = u64::from(byte) & (self.size - 1);
                state.page_base = state.word & !(self.page - 1);
                state.page_touched.fill(false);
                state.phase = Phase::Writing;
                Ack::Ack
            }
            Phase::Writing => {
                // Staged, not stored: §5.5 makes `WP` decide at the STOP, and
                // §5.2's roll-over means a long page write overwrites its own
                // earlier bytes rather than the next page's.
                let slot = (state.word - state.page_base) as usize;
                if let Some(cell) = state.page_data.get_mut(slot) {
                    *cell = byte;
                    state.page_touched[slot] = true;
                }
                self.step_write(&mut state);
                Ack::Ack
            }
            Phase::Idle | Phase::Reading => Ack::Nack,
        }
    }

    fn read(&self) -> u8 {
        let state = self.state.lock();
        if state.phase != Phase::Reading {
            return 0xff;
        }
        state.mem.get(state.word as usize).copied().unwrap_or(0xff)
    }

    fn read_ack(&self, ack: Ack) {
        let mut state = self.state.lock();
        // The counter moves on either way: §6.1 defines it as "the last address
        // accessed during the last Read or Write operation, incremented by
        // one", and the byte *was* accessed — it went out on the wire. What the
        // acknowledge decides is only whether another one follows.
        self.step_read(&mut state);
        if !ack.is_ack() {
            // §6.1: a NACK "will force the device into standby mode".
            state.phase = Phase::Idle;
        }
    }

    fn stop(&self) {
        let mut state = self.state.lock();
        if state.phase == Phase::Writing {
            self.commit(&mut state);
        }
        state.phase = Phase::Idle;
    }

    fn peek(&self) -> u8 {
        let state = self.state.lock();
        state.mem.get(state.word as usize).copied().unwrap_or(0xff)
    }
}

// ---------------------------------------------------------------------------
// The write-protect pin
// ---------------------------------------------------------------------------

/// The `WP` input (§5.5).
struct WriteProtectSink {
    shared: Arc<Shared>,
}

impl fmt::Debug for WriteProtectSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WriteProtectSink").finish_non_exhaustive()
    }
}

impl WireSink for WriteProtectSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.shared.state.lock().wp = level;
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for At24c {
    fn class(&self) -> &'static DeviceClass {
        &AT24C_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: joining the bus. Two-phase construction
        // (`CLAUDE.md`) puts it here rather than in `new`.
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // The tick is *not* zeroed: `Machine::reset` does not rewind clock
            // domains (`ROADMAP.md` §4.2), so a lazily advanced device that
            // rewound its own would then be ahead of nothing and behind
            // everything. The `WP` level is not touched either: it belongs to
            // whatever drives it, and resetting this device does not move
            // another device's pin.
            state.word = 0;
            state.phase = Phase::Idle;
            state.page_base = 0;
            state.page_touched.fill(false);
            state.busy = false;
            state.busy_until = 0;
            // The array survives: it is an EEPROM, and a power-on reset of the
            // board is not an erase.
            self.shared.publish(&state);
        }
        self.wires.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_bytes(&state.mem)?;
        w.write_u64(state.word)?;
        w.write_u8(phase_code(state.phase))?;
        w.write_u64(state.page_base)?;
        w.write_bytes(&state.page_data)?;
        w.write_u64(state.page_touched.len() as u64)?;
        for t in &state.page_touched {
            w.write_bool(*t)?;
        }
        w.write_u64(state.busy_until)?;
        w.write_bool(state.busy)?;
        drop(state);
        // The bit-level engine: a snapshot taken part-way through an address
        // phase has to resume mid-address, not restart the transfer.
        self.wires.snapshot().write(w)
        // `wp` is not saved: it is the level *another* device drives, and that
        // device restores its own state and drives it again (`ROADMAP.md`
        // §4.5). An unwired pin is low by Table 1-1's note 1, which `reset`
        // already established.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        let mem = r.read_bytes()?.to_vec();
        let word = r.read_u64()?;
        let phase = phase_from_code(r.read_u8()?);
        let page_base = r.read_u64()?;
        let page_data = r.read_bytes()?.to_vec();
        let touched_len = r.read_seq_len(1)?;
        let mut page_touched = Vec::with_capacity(touched_len.min(MAX_SIZE) as usize);
        for _ in 0..touched_len {
            page_touched.push(r.read_bool()?);
        }
        let busy_until = r.read_u64()?;
        let busy = r.read_bool()?;
        let bits = SlaveWiresState::read(r)?;

        {
            let mut state = self.shared.state.lock();
            if mem.len() == state.mem.len() {
                state.mem = mem;
            }
            state.ticks = ticks;
            state.word = word & (self.shared.size - 1);
            state.phase = phase;
            state.page_base = page_base & !(self.shared.page - 1);
            if page_data.len() == state.page_data.len() {
                state.page_data = page_data;
            }
            if page_touched.len() == state.page_touched.len() {
                state.page_touched = page_touched;
            }
            state.busy_until = busy_until;
            state.busy = busy;
            self.shared.publish(&state);
        }
        self.wires.restore(bits);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        match port {
            line::SCL_NAME => Some(SinkPin {
                sink: self.wires.sink(line::SCL, sources),
                line: line::SCL,
            }),
            line::SDA_NAME => Some(SinkPin {
                sink: self.wires.sink(line::SDA, sources),
                line: line::SDA,
            }),
            pin::WP => {
                self.shared.state.lock().wp_wired = true;
                let sink = Arc::new(WriteProtectSink {
                    shared: Arc::clone(&self.shared),
                });
                // Kept, because a net refers to its sinks weakly.
                *self.wp_pin.lock() = Some(Arc::clone(&sink));
                Some(SinkPin {
                    sink: sink as Arc<dyn WireSink>,
                    line: pin::WP_LINE,
                })
            }
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
                        "an AT24C drives only `{}` and `{}`, and only ever low: both are \
                         open-drain (datasheet Table 1-1). `{}` is an input.",
                        line::SCL_NAME,
                        line::SDA_NAME,
                        pin::WP
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

    /// Yes, and for one reason: §5.4's internally self-timed write cycle. A
    /// master polls for its end by addressing the part (§5.3), and the answer
    /// has to be the one at the cycle of the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        At24c::advance_to(self, tick);
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

impl Instance for At24c {}

/// A stable code for a phase, for the snapshot.
const fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::Idle => 0,
        Phase::WantWordAddress => 1,
        Phase::Writing => 2,
        Phase::Reading => 3,
    }
}

/// The inverse. An unknown code loads as idle: a snapshot is untrusted input.
const fn phase_from_code(code: u8) -> Phase {
    match code {
        1 => Phase::WantWordAddress,
        2 => Phase::Writing,
        3 => Phase::Reading,
        _ => Phase::Idle,
    }
}

/// The `atmel.at24c` device class.
pub static AT24C_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Atmel AT24C01D/02D I2C serial EEPROM: page write, sequential read, \
              acknowledge polling, write protect",
    properties: &[
        PropertySpec {
            name: "chip",
            kind: ValueKind::Uint,
            required: false,
            summary: "the hardware address pins A2 A1 A0 as one number, 0 to 7 (§4.1)",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Uint,
            required: false,
            summary: "the array in bytes: 128 for an AT24C01D, 256 for an AT24C02D (default 256)",
        },
        PropertySpec {
            name: "page",
            kind: ValueKind::Uint,
            required: false,
            summary: "one page in bytes (default 8, which is what both parts have)",
        },
        PropertySpec {
            name: "write-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "tWR in ticks of this device's clock domain (§5.4; default 5000)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "initial contents; absent means all 0xff, as delivered (§7)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named I2C bus to hang off, for a transactional link",
        },
    ],
    construct: |props| Ok(Box::new(At24c::new(props)?)),
};

/// Add [`AT24C_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&AT24C_CLASS)
}

/// Bind [`AT24C_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(At24c::new(props)?)))
}

/// What the validator should know about `atmel.at24c`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("chip", ValueKind::Uint).range(0, 7))
        .prop(PropSchema::new("size", ValueKind::Uint).range(1, MAX_SIZE))
        .prop(PropSchema::new("page", ValueKind::Uint).range(1, MAX_SIZE))
        .prop(PropSchema::new("write-ticks", ValueKind::Uint))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("bus", ValueKind::Str))
        // Both bus lines are open drain, so each is an input *and* an output;
        // a machine file names them in two `wire` statements and the resolver
        // folds those into one net.
        .port(line::SCL_NAME, PortDir::InOut)
        .port(line::SDA_NAME, PortDir::InOut)
        .port(pin::WP, PortDir::In)
}
