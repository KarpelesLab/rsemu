//! The STM32**F4** SPI peripheral.
//!
//! # Which family, and why it matters
//!
//! ST reuses the name `SPI` across families for blocks that share pins and
//! nothing else. This models the **F4's**, from ST's **RM0090** (rev 21, June
//! 2024, DocID018909), chapter **28 "Serial peripheral interface (SPI)"** — the
//! classic nine-register, 16-bit-wide, no-FIFO block that most SPI guest
//! software targets, and the one on the F405/415, F407/417, F427/437 and
//! F429/439.
//!
//! The **H7's SPI is a different IP, not a superset** (RM0433): its
//! configuration is split across `CFG1`/`CFG2`, `CR2` holds a `TSIZE` transfer
//! counter, `CR1` a `CSTART` bit, the data register is split into `TXDR` and
//! `RXDR` behind multi-entry FIFOs with `TXP`/`RXP`/`EOT` flags, and the frame
//! size runs from 4 to 32 bits. Nothing in the F4's `0x00`-`0x20` map survives
//! that. A board with an H7 gets a second module rather than a property on this
//! one.
//!
//! # Register map (§28.5.10, Table 130)
//!
//! | Offset | Name | Reset | Notes |
//! | --- | --- | --- | --- |
//! | `0x00` | `CR1` | `0x0000` | mode, framing, baud rate, `SPE` |
//! | `0x04` | `CR2` | `0x0000` | `SSOE`, DMA and interrupt enables |
//! | `0x08` | `SR` | `0x0002` | `TXE` is set out of reset |
//! | `0x0c` | `DR` | `0x0000` | **two buffers**: a write loads Tx, a read pops Rx |
//! | `0x10` | `CRCPR` | `0x0007` | the CRC polynomial |
//! | `0x14` | `RXCRCR` | `0x0000` | read-only |
//! | `0x18` | `TXCRCR` | `0x0000` | read-only |
//! | `0x1c` | `I2SCFGR` | `0x0000` | see below |
//! | `0x20` | `I2SPR` | `0x0002` | see below |
//!
//! Every register is sixteen bits in a thirty-two bit slot; §28.5 says
//! accesses are by half-word or word. Byte access is **not defined by the
//! manual at all** — ST's own headers do it to `DR` in 8-bit frame format, so
//! it is accepted here and reaches the low half, and the module says so rather
//! than pretending the manual answered.
//!
//! # The parts real drivers trip over, and which are modelled
//!
//! * **`SSM`/`SSI`/`SSOE`, and `MODF`.** §28.3.1: with `SSM` set "the slave
//!   select information is driven internally by the value of the `SSI` bit …
//!   the external NSS pin remains free"; with `SSM` clear and `SSOE` set, NSS
//!   "is driven low when the master starts the communication and is kept low
//!   until the SPI is disabled". A master that sees its NSS low takes a **mode
//!   fault** (§28.3.10): `MODF` sets, `SPE` clears, `MSTR` clears — the
//!   peripheral demotes itself to a slave — and *"hardware does not allow the
//!   setting of the `SPE` and `MSTR` bits while the `MODF` bit is set"*. That
//!   last clause is the one that turns a driver bug into a peripheral that
//!   will not start, and it is modelled: a write of `SPE` while `MODF` stands
//!   is dropped. Clearing takes the manual's two steps — an access to `SR`,
//!   then a write to `CR1`.
//! * **`OVR` is not cleared by clearing it.** §28.3.10: "clearing the `OVR`
//!   bit is done by a read from the `SPI_DR` register followed by a read
//!   access to the `SPI_SR` register", and until then the receive buffer is
//!   *frozen* — every further frame is dropped rather than overwriting it.
//! * **`DR` is two registers.** A write goes to the Tx buffer, a read comes
//!   from the Rx buffer, and in 8-bit format §28.5.4 says the top half of a
//!   read is forced to zero.
//! * **Receive-only masters clock themselves.** §28.3.4: with `RXONLY` set (or
//!   `BIDIMODE` set and `BIDIOE` clear) a master "communication starts
//!   immediately and stops when the `SPE` bit is cleared" — no `DR` write is
//!   needed and none is expected, which is how a driver reads a flash without
//!   writing dummy bytes.
//!
//! # Both link models, as the fabric demands
//!
//! `link` is required and has no default, exactly as
//! [`crate::bus::spi::controller`]'s is and for the reason
//! `docs/buses/low-speed.md` gives. A frame costs `bits × 2^(BR+1)` ticks of
//! this peripheral's clock domain either way — §28.5.1's baud-rate divisor is
//! `2^(BR+1)`, so one bit is one `SCK` period is `2^(BR+1)` ticks of `PCLK` —
//! so a driver polling `BSY` sees the same timing under both. What differs is
//! only whether the edges exist.
//!
//! # Slave mode
//!
//! With `MSTR` clear the peripheral generates no clock and starts nothing; it
//! *answers*. That half is reached through the fabric's own
//! [`SlavePins`] on the `sck-in`, `mosi-in`,
//! `nss-in` and `miso-out` pins, so another controller — or a guest bit-banging
//! GPIO — clocks it and `DR`, `TXE`, `RXNE`, `OVR` and `BSY` move exactly as
//! they would in master mode.
//!
//! **NSS is split into two pins**, `nss` (out) and `nss-in` (in). The real part
//! has one bidirectional pin; an rsemu wire has fixed drivers and cannot be
//! tri-stated, which is the same split
//! [`crate::dev::sitronix`] makes for the ST7272A's `SDA`.
//!
//! # What is not modelled, and says so
//!
//! `I2SCFGR` and `I2SPR` are stored and read back — a driver that probes them
//! must see its own writes — but **I²S itself is not implemented**: setting
//! `I2SMOD` does not turn this into an audio interface, and `CHSIDE` and `UDR`
//! never set. Modelling half an I²S peripheral would be worse than modelling
//! none, and a machine that wants one should say so. The `FRF` (TI frame
//! format) bit is likewise stored; `FRE` never sets, because TI framing
//! changes where NSS pulses and nothing in this tree watches for that.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::spi::{
    BitOrder, ChipSelect, Format, Link, MAX_CHIP_SELECTS, Mode, SlavePins, SpiBus, SpiSlave, buses,
    pin as slave_pin,
};
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
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "stm32.spi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the peripheral occupies.
///
/// The registers stop at `0x23`; the STM32 bus gives every peripheral a 1 KiB
/// slot (RM0090 Table 1), and reads above the last register answer zero.
pub const REGISTER_BYTES: u64 = 0x400;

/// The last defined register offset, past which reads answer zero.
const LAST_REGISTER: u64 = 0x20;

// -- CR1 (§28.5.1) ----------------------------------------------------------

/// Clock phase.
const CR1_CPHA: u16 = 1 << 0;
/// Clock polarity.
const CR1_CPOL: u16 = 1 << 1;
/// Master configuration.
const CR1_MSTR: u16 = 1 << 2;
/// Baud-rate control, bits 5:3. The divisor is `2^(BR + 1)`.
const CR1_BR_SHIFT: u32 = 3;
/// And its mask, once shifted down.
const CR1_BR_MASK: u16 = 0x7;
/// SPI enable.
const CR1_SPE: u16 = 1 << 6;
/// Least significant bit first.
const CR1_LSBFIRST: u16 = 1 << 7;
/// Internal slave select — the value `SSM` substitutes for the NSS pin.
const CR1_SSI: u16 = 1 << 8;
/// Software slave management.
const CR1_SSM: u16 = 1 << 9;
/// Receive only.
const CR1_RXONLY: u16 = 1 << 10;
/// Data frame format: set is sixteen bits, clear is eight.
const CR1_DFF: u16 = 1 << 11;
/// The next transfer carries the CRC.
const CR1_CRCNEXT: u16 = 1 << 12;
/// Hardware CRC calculation enable.
const CR1_CRCEN: u16 = 1 << 13;
/// Output enable in bidirectional mode.
const CR1_BIDIOE: u16 = 1 << 14;
/// Bidirectional data mode: one data wire instead of two.
const CR1_BIDIMODE: u16 = 1 << 15;

// -- CR2 (§28.5.2) ----------------------------------------------------------

/// Rx buffer DMA enable.
const CR2_RXDMAEN: u16 = 1 << 0;
/// Tx buffer DMA enable.
const CR2_TXDMAEN: u16 = 1 << 1;
/// SS output enable.
const CR2_SSOE: u16 = 1 << 2;
/// Frame format: set selects TI mode.
const CR2_FRF: u16 = 1 << 4;
/// Error interrupt enable.
const CR2_ERRIE: u16 = 1 << 5;
/// Rx buffer not empty interrupt enable.
const CR2_RXNEIE: u16 = 1 << 6;
/// Tx buffer empty interrupt enable.
const CR2_TXEIE: u16 = 1 << 7;
/// Everything `CR2` defines. Bit 3 is forced to zero by hardware (§28.5.2).
const CR2_MASK: u16 =
    CR2_RXDMAEN | CR2_TXDMAEN | CR2_SSOE | CR2_FRF | CR2_ERRIE | CR2_RXNEIE | CR2_TXEIE;

// -- SR (§28.5.3) -----------------------------------------------------------

/// Receive buffer not empty.
const SR_RXNE: u16 = 1 << 0;
/// Transmit buffer empty.
const SR_TXE: u16 = 1 << 1;
/// Underrun. I²S only; never set here.
const SR_UDR: u16 = 1 << 3;
/// The received CRC did not match.
const SR_CRCERR: u16 = 1 << 4;
/// Master mode fault.
const SR_MODF: u16 = 1 << 5;
/// Overrun: a frame arrived with the receive buffer still full.
const SR_OVR: u16 = 1 << 6;
/// Busy: a transfer is in flight.
const SR_BSY: u16 = 1 << 7;
/// TI-mode frame error. Never set here; see the module docs.
const SR_FRE: u16 = 1 << 8;

/// What `SR` reads as out of reset: the transmit buffer is empty.
const SR_RESET: u16 = SR_TXE;

/// The polynomial `CRCPR` powers up with (§28.5.5).
const CRCPR_RESET: u16 = 0x0007;

/// What `I2SPR` powers up with (§28.5.9).
const I2SPR_RESET: u16 = 0x0002;

/// "Nothing scheduled", as [`Shared::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

/// The pin names a machine description wires.
pub mod pin {
    /// The serial clock this peripheral drives as a master.
    pub const SCK: &str = "sck";
    /// Data out to the slaves, as a master.
    pub const MOSI: &str = "mosi";
    /// Data in from the slaves, as a master.
    pub const MISO: &str = "miso";
    /// The slave-select output, as a master with `SSOE` set. Active low.
    pub const NSS: &str = "nss";
    /// The slave-select input: what a master watches for a mode fault, and
    /// what selects this peripheral when it is the slave. Active low.
    pub const NSS_IN: &str = "nss-in";
    /// The clock input, as a slave.
    pub const SCK_IN: &str = "sck-in";
    /// The data input, as a slave.
    pub const MOSI_IN: &str = "mosi-in";
    /// The data output, as a slave.
    pub const MISO_OUT: &str = "miso-out";

    /// The wire line the MISO input answers on.
    pub const MISO_LINE: u32 = 0;
    /// The wire line the NSS input answers on.
    pub const NSS_IN_LINE: u32 = 1;
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Domain ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    cr1: u16,
    cr2: u16,
    sr: u16,
    /// The Tx buffer. A `DR` write lands here.
    tx: u16,
    /// The Rx buffer. A `DR` read comes from here.
    rx: u16,
    crcpr: u16,
    rxcrc: u16,
    txcrc: u16,
    i2scfgr: u16,
    i2spr: u16,
    /// Whether the Tx buffer holds a word nothing has shifted yet.
    tx_pending: bool,
    /// The word in the shift register, going out.
    shift: u16,
    /// Whether that shift register is running.
    busy: bool,
    /// Whether the frame in flight is the CRC rather than data.
    crc_frame: bool,
    /// The tick the in-flight frame began on.
    started: u64,
    /// Edges emitted so far, in [`Link::Wired`].
    edges: u32,
    /// Bits captured from MISO so far, in [`Link::Wired`].
    shift_in: u32,
    /// Whether the NSS output is being held low.
    nss_low: bool,
    /// The level the NSS input is at. Reset does not move it — it belongs to
    /// whatever drives it (`ROADMAP.md` §4.5).
    nss_in: Level,
    /// Whether a `DR` read has happened since `OVR` set, which is the first
    /// half of §28.3.10's clearing sequence.
    ovr_dr_read: bool,
    /// Whether `SR` has been accessed since `MODF` set, which is the first
    /// half of §28.3.10's other clearing sequence.
    modf_sr_seen: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            ticks: 0,
            cr1: 0,
            cr2: 0,
            sr: SR_RESET,
            tx: 0,
            rx: 0,
            crcpr: CRCPR_RESET,
            rxcrc: 0,
            txcrc: 0,
            i2scfgr: 0,
            i2spr: I2SPR_RESET,
            tx_pending: false,
            shift: 0,
            busy: false,
            crc_frame: false,
            started: 0,
            edges: 0,
            shift_in: 0,
            nss_low: false,
            // A pin nothing drives sits at its inactive level rather than the
            // low a fresh net idles at: a master on a board that never wires
            // NSS must not take a mode fault for the machine file's silence.
            nss_in: Level::High,
            ovr_dr_read: false,
            modf_sr_seen: false,
        }
    }
}

impl State {
    /// The framing `CR1` describes (§28.5.1).
    fn format(&self) -> Format {
        Format::new(
            Mode::from_cpol_cpha(self.cr1 & CR1_CPOL != 0, self.cr1 & CR1_CPHA != 0),
            if self.cr1 & CR1_DFF != 0 { 16 } else { 8 },
            if self.cr1 & CR1_LSBFIRST != 0 {
                BitOrder::LsbFirst
            } else {
                BitOrder::MsbFirst
            },
        )
    }

    /// Half of one `SCK` period, in domain ticks.
    ///
    /// §28.5.1's divisor is `2^(BR + 1)` of `PCLK`, so a whole bit is that
    /// many ticks and a half period is `2^BR` — never zero, which the
    /// scheduler requires or catch-up stops making progress.
    fn half_period(&self) -> u64 {
        1u64 << ((self.cr1 >> CR1_BR_SHIFT) & CR1_BR_MASK)
    }

    /// How many wire edges one frame takes: two per bit.
    fn total_edges(&self) -> u32 {
        u32::from(self.format().bits) * 2
    }

    /// The tick the in-flight frame completes on.
    fn end_tick(&self) -> u64 {
        self.started
            .saturating_add(u64::from(self.total_edges()) * self.half_period())
    }

    fn is_master(&self) -> bool {
        self.cr1 & CR1_MSTR != 0
    }

    fn is_enabled(&self) -> bool {
        self.cr1 & CR1_SPE != 0
    }

    /// Whether the peripheral drives no data line: `RXONLY`, or bidirectional
    /// with the output disabled (§28.3.4).
    fn receive_only(&self) -> bool {
        self.cr1 & CR1_RXONLY != 0 || (self.cr1 & CR1_BIDIMODE != 0 && self.cr1 & CR1_BIDIOE == 0)
    }

    /// Whether the peripheral keeps no received word: bidirectional with the
    /// output enabled (§28.3.4).
    fn transmit_only(&self) -> bool {
        self.cr1 & CR1_BIDIMODE != 0 && self.cr1 & CR1_BIDIOE != 0
    }

    /// The level the NSS pin sees, whichever way `SSM` says to look (§28.3.1).
    fn nss_level(&self) -> Level {
        if self.cr1 & CR1_SSM != 0 {
            Level::from_bool(self.cr1 & CR1_SSI != 0)
        } else {
            self.nss_in
        }
    }

    /// Whether a master should be taking a mode fault right now (§28.3.10).
    ///
    /// Only when the peripheral is not itself driving NSS: with `SSOE` set the
    /// output is its own, and a master cannot fault on the level it drives.
    fn mode_fault_due(&self) -> bool {
        self.is_master()
            && !(self.cr1 & CR1_SSM == 0 && self.cr2 & CR2_SSOE != 0)
            && self.nss_level().is_low()
    }

    /// Whether the NSS output should be low: §28.3.1's "driven low when the
    /// master starts the communication and kept low until the SPI is
    /// disabled".
    fn nss_output_low(&self) -> bool {
        self.is_master() && self.is_enabled() && self.cr1 & CR1_SSM == 0 && self.cr2 & CR2_SSOE != 0
    }
}

/// One turn of the CRC, MSB first.
///
/// §28.3.6 says only that the calculator is "CRC8 for 8-bit data" and "CRC16
/// for 16-bit data" with the `CRCPR` polynomial; it does not write the
/// recurrence down, so this is the conventional non-reflected, zero-initial
/// form, and a guest checking a CRC against a peer that uses another
/// convention will disagree with this model exactly as it would with itself.
fn crc_step(crc: u16, data: u16, bits: u8, poly: u16) -> u16 {
    let width = u32::from(bits);
    let mask: u32 = if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    };
    let top = 1u32 << (width - 1);
    let mut acc = u32::from(crc) & mask;
    for i in (0..width).rev() {
        let bit = (u32::from(data) >> i) & 1;
        let msb = acc & top != 0;
        acc = (acc << 1) & mask;
        if msb != (bit != 0) {
            acc ^= u32::from(poly) & mask;
        }
    }
    acc as u16
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An STM32F4 SPI peripheral.
#[derive(Debug)]
pub struct Stm32Spi {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
    region: RegionRef,
}

/// The wire outputs, all optional until a machine description connects them.
#[derive(Debug, Default)]
struct Pins {
    sck: Option<WireSource>,
    mosi: Option<WireSource>,
    nss: Option<WireSource>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// How words reach the slaves, written down in the machine file.
    link: Link,
    /// The bus this peripheral drives as a master, transactionally.
    bus: Option<Arc<SpiBus>>,
    /// Which chip select on that bus the NSS output corresponds to.
    cs: ChipSelect,
    /// Domain ticks simulated, for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The tick the next edge or completion falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The level last seen on the MISO input.
    miso: AtomicBool,
    /// The interrupt output level, published for a debug read.
    irq_level: AtomicBool,
    pins: Mutex<Pins>,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// Input pins handed out by [`Device::sink`], kept alive here because a net
    /// refers to its sinks weakly (`core::device`, §4.3's weak edge).
    sinks: Mutex<Vec<Arc<InputSink>>>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("link", &self.link).field("cs", &self.cs);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Stm32Spi {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `link` — `"transactional"` or `"wired"`. Required, and deliberately
    ///   so: `docs/buses/low-speed.md` asks for the choice to be made rather
    ///   than defaulted into.
    /// * `bus` — the named [`SpiBus`] this peripheral drives as a master.
    ///   Required for `transactional`.
    /// * `cs` — which chip select on that bus the NSS output corresponds to.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for a `link` this module does not know or a `cs` out
    /// of range.
    pub fn new(props: &Props) -> Result<Stm32Spi> {
        let mut r = props.reader();
        let link_name = r.require_str("link")?.to_string();
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs = r.or_range("cs", 0u64, 0..=(MAX_CHIP_SELECTS as u64 - 1))?;
        r.finish()?;

        let link = Link::from_name(&link_name).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: alloc::format!(
                "`link` is `{link_name}`; it must be one of {:?} — see docs/buses/low-speed.md \
                 for which to pick",
                Link::NAMES
            ),
        })?;
        if link == Link::Transactional && bus_name.is_none() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "a `transactional` SPI master reaches its slaves through a named bus; give \
                     it `bus = \"spi1\"` and name the same bus on each slave",
                ),
            });
        }
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        Ok(Stm32Spi::with_bus(link, bus, ChipSelect(cs as u8)))
    }

    /// A peripheral on a bus the caller already holds.
    #[must_use]
    pub fn with_bus(link: Link, bus: Option<Arc<SpiBus>>, cs: ChipSelect) -> Stm32Spi {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            link,
            bus,
            cs,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            miso: AtomicBool::new(true),
            irq_level: AtomicBool::new(false),
            pins: Mutex::with_rank(LockRank::WIRE, Pins::default()),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            sinks: Mutex::with_rank(LockRank::WIRE, Vec::new()),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        let port = Arc::new(RegisterBlock {
            shared: Arc::clone(&shared),
            pins: Arc::clone(&pins),
        });
        let region = Arc::new(Region::io(
            "stm32-spi",
            REGISTER_BYTES,
            port as Arc<dyn MemOps>,
        ));
        Stm32Spi {
            shared,
            pins,
            region,
        }
    }

    /// How this peripheral carries a word.
    #[must_use]
    pub fn link(&self) -> Link {
        self.shared.link
    }

    /// The bus it drives as a master, if it has one.
    #[must_use]
    pub fn bus(&self) -> Option<&Arc<SpiBus>> {
        self.shared.bus.as_ref()
    }

    /// Its slave-side pins, for a controller that clocks it directly.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// `SR`, as software would read it — without the side effects a read has.
    #[must_use]
    pub fn status(&self) -> u16 {
        self.shared.state.lock().sr
    }

    /// The framing `CR1` currently describes.
    #[must_use]
    pub fn format(&self) -> Format {
        self.shared.state.lock().format()
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.shared.irq_level.load(Ordering::Relaxed)
    }

    /// Run the peripheral until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// the engine
// ---------------------------------------------------------------------------

/// One wire action the engine wants performed once the state lock is released.
#[derive(Debug, Clone, Copy)]
enum Emit {
    Sck(Level),
    Mosi(Level),
    Nss(Level),
}

impl Shared {
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event
            .store(Shared::next_event(state), Ordering::Relaxed);
    }

    /// The tick the next thing happens on, or [`NO_EVENT`].
    fn next_event(state: &State) -> u64 {
        if !state.busy {
            // A receive-only master clocks itself for as long as it is
            // enabled, so it is never idle (§28.3.4).
            if state.is_master() && state.is_enabled() && state.receive_only() {
                return state.ticks.saturating_add(1);
            }
            return NO_EVENT;
        }
        let half = state.half_period();
        let next = state
            .started
            .saturating_add((u64::from(state.edges) + 1) * half);
        next.max(state.ticks.saturating_add(1))
    }

    /// Bring the peripheral up to date before an access.
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
        // A refusal means catch-up is already running further up the stack;
        // answering from where the peripheral stands is the only defined thing
        // to do.
        let _ = handle.sync(kind);
    }

    fn emit(&self, action: Emit) {
        let port = {
            let pins = self.pins.lock();
            match action {
                Emit::Sck(_) => pins.sck.clone(),
                Emit::Mosi(_) => pins.mosi.clone(),
                Emit::Nss(_) => pins.nss.clone(),
            }
        };
        let level = match action {
            Emit::Sck(l) | Emit::Mosi(l) | Emit::Nss(l) => l,
        };
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Re-drive every output from the state, for the realize sweep.
    fn announce_all(&self) {
        let (sck, nss) = {
            let state = self.state.lock();
            (
                state.format().mode.idle_level(),
                Level::from_bool(!state.nss_low),
            )
        };
        self.emit(Emit::Sck(sck));
        self.emit(Emit::Nss(nss));
        self.publish_irq();
    }

    /// Whether any enabled interrupt source is asserting (§28.3.11's table).
    fn irq_state(state: &State) -> bool {
        let sr = state.sr;
        let cr2 = state.cr2;
        (cr2 & CR2_TXEIE != 0 && sr & SR_TXE != 0)
            || (cr2 & CR2_RXNEIE != 0 && sr & SR_RXNE != 0)
            || (cr2 & CR2_ERRIE != 0 && sr & (SR_MODF | SR_OVR | SR_CRCERR | SR_FRE | SR_UDR) != 0)
    }

    fn publish_irq(&self) {
        let level = Level::from_bool(Shared::irq_state(&self.state.lock()));
        self.irq_level.store(level.is_high(), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Move the NSS output and, transactionally, the bus's chip select.
    fn drive_nss(&self, low: bool) {
        self.emit(Emit::Nss(Level::from_bool(!low)));
        if self.link == Link::Transactional
            && let Some(bus) = &self.bus
        {
            bus.select(low.then_some(self.cs));
        }
    }

    /// Start a frame, if the peripheral is in a position to.
    ///
    /// Called with the state lock held. Returns whether one started.
    fn begin(state: &mut State) -> bool {
        if state.busy || !state.is_enabled() || !state.is_master() {
            return false;
        }
        // §28.3.6: with `CRCNEXT` set the next frame carries the CRC instead
        // of the data, and the calculators are frozen while it does.
        let crc_frame = state.cr1 & CR1_CRCEN != 0 && state.cr1 & CR1_CRCNEXT != 0;
        if crc_frame {
            state.shift = state.txcrc;
            state.cr1 &= !CR1_CRCNEXT;
        } else if state.receive_only() {
            // §28.3.4: the clock free-runs and nothing is driven out.
            state.shift = 0xffff;
        } else if state.tx_pending {
            state.shift = state.tx;
            state.tx_pending = false;
        } else {
            return false;
        }
        state.crc_frame = crc_frame;
        // The Tx buffer emptied into the shift register, which is exactly what
        // §28.3.7 says sets `TXE`.
        state.sr |= SR_TXE;
        state.sr |= SR_BSY;
        state.busy = true;
        state.started = state.ticks;
        state.edges = 0;
        state.shift_in = 0;
        true
    }

    /// A frame finished with `received` on the data line.
    fn finish(state: &mut State, received: u16) {
        let format = state.format();
        let received = (received as u32 & format.mask()) as u16;
        state.busy = false;
        state.sr &= !SR_BSY;
        if state.crc_frame {
            // §28.3.6: at the end of a CRC frame the received word is compared
            // with the calculated one.
            if received != state.rxcrc {
                state.sr |= SR_CRCERR;
            }
            state.crc_frame = false;
            return;
        }
        if state.cr1 & CR1_CRCEN != 0 {
            let poly = state.crcpr;
            let bits = format.bits;
            state.txcrc = crc_step(state.txcrc, state.shift, bits, poly);
            state.rxcrc = crc_step(state.rxcrc, received, bits, poly);
        }
        if state.transmit_only() {
            // §28.3.4: nothing arrives on a wire the peripheral is driving.
            return;
        }
        if state.sr & SR_RXNE != 0 {
            // §28.3.10: "the receiver buffer contents are not updated with the
            // newly received data" — the buffer freezes and the frame is lost.
            state.sr |= SR_OVR;
            state.ovr_dr_read = false;
        } else {
            state.rx = received;
            state.sr |= SR_RXNE;
        }
    }

    /// The bit of the shift register that goes out for bit index `n`.
    fn tx_bit(state: &State, n: u32) -> Level {
        let format = state.format();
        let bit = match format.order {
            BitOrder::MsbFirst => (state.shift >> (u32::from(format.bits) - 1 - n)) & 1,
            BitOrder::LsbFirst => (state.shift >> n) & 1,
        };
        Level::from_bool(bit != 0)
    }

    /// Fold a sampled MISO bit into the received word.
    fn capture(state: &mut State, n: u32, level: Level) {
        if !level.as_bool() {
            return;
        }
        let format = state.format();
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
                /// A wired edge: drive these, in order, then loop.
                Edges(Vec<Emit>),
                /// A transactional frame: hand it to the bus, store the reply.
                Word(u16),
                /// A frame began; present its first bit and loop.
                Present,
            }

            let step = {
                let mut state = self.state.lock();
                if !state.busy {
                    // A receive-only master starts the next frame the instant
                    // the previous one ends, for as long as `SPE` stands.
                    if state.ticks < target && Shared::begin(&mut state) {
                        self.publish(&state);
                        Step::Present
                    } else {
                        state.ticks = state.ticks.max(target);
                        self.publish(&state);
                        Step::Done
                    }
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
                                Step::Word(state.shift)
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
                                let level = if k.is_multiple_of(2) {
                                    idle.inverted()
                                } else {
                                    idle
                                };
                                let bit = k / 2;
                                let mut out = Vec::new();
                                if format.mode.samples_on(level) {
                                    let miso = Level::from_bool(self.miso.load(Ordering::Relaxed));
                                    Shared::capture(&mut state, bit, miso);
                                } else {
                                    let next = k.div_ceil(2).min(u32::from(format.bits) - 1);
                                    out.push(Emit::Mosi(Shared::tx_bit(&state, next)));
                                }
                                out.push(Emit::Sck(level));
                                state.edges += 1;
                                if state.edges >= state.total_edges() {
                                    let received = state.shift_in as u16;
                                    Shared::finish(&mut state, received);
                                }
                                self.publish(&state);
                                Step::Edges(out)
                            }
                        }
                    }
                }
            };

            match step {
                Step::Done => {
                    self.publish_irq();
                    return;
                }
                Step::Edges(actions) => {
                    for action in actions {
                        self.emit(action);
                    }
                    self.publish_irq();
                }
                Step::Word(word) => {
                    // Outward, with no lock of ours held.
                    let reply = self
                        .bus
                        .as_ref()
                        .map_or(0xffff, |bus| bus.transfer(u32::from(word)) as u16);
                    {
                        let mut state = self.state.lock();
                        Shared::finish(&mut state, reply);
                        self.publish(&state);
                    }
                    self.publish_irq();
                }
                Step::Present => {
                    self.present_first_bit();
                    self.publish_irq();
                }
            }
        }
    }

    /// Put the first bit on MOSI before the first clock edge.
    ///
    /// With CPHA 0 that edge is the one that samples it, so a shift register
    /// that only loaded on the first edge would present the wrong bit.
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
        self.emit(Emit::Mosi(level));
    }

    /// Re-evaluate the mode-fault condition after something moved NSS or `CR1`.
    ///
    /// Returns whether the fault fired, so the caller can drive the outputs
    /// the demotion implies once it has let go of the lock.
    fn check_mode_fault(state: &mut State) -> bool {
        if !state.mode_fault_due() {
            return false;
        }
        // §28.3.10, and this is the whole of it: the flag sets, the peripheral
        // switches itself off, and it demotes itself to a slave.
        state.sr |= SR_MODF;
        state.cr1 &= !(CR1_SPE | CR1_MSTR);
        state.busy = false;
        state.sr &= !SR_BSY;
        state.nss_low = false;
        state.modf_sr_seen = false;
        true
    }
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

struct RegisterBlock {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
}

impl fmt::Debug for RegisterBlock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RegisterBlock").finish_non_exhaustive()
    }
}

/// What a register access asks for once the state lock is released.
#[derive(Debug, Clone, Copy, Default)]
struct After {
    /// Move the NSS output (and the transactional bus's chip select) to this.
    nss: Option<bool>,
    /// Re-read the slave-side shifter's framing from `CR1`.
    reframe: bool,
    /// Present the first bit of a frame that just began.
    started: bool,
    /// Re-drive every output.
    announce: bool,
}

impl RegisterBlock {
    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u16 {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => state.cr1,
            0x04 => state.cr2,
            0x08 => {
                let value = state.sr;
                if !debug {
                    // The trap `MemAttrs::debug` exists for, twice over. A
                    // read of `SR` is the *second half* of §28.3.10's overrun
                    // clearing sequence and the *first half* of its mode-fault
                    // one; a debugger that took either step would leave the
                    // guest's own sequence half-consumed and its next read
                    // lying.
                    if state.sr & SR_MODF != 0 {
                        state.modf_sr_seen = true;
                    }
                    if state.sr & SR_OVR != 0 && state.ovr_dr_read {
                        state.sr &= !SR_OVR;
                        state.ovr_dr_read = false;
                    }
                    // §28.5.3: `FRE` is cleared by reading `SR`.
                    state.sr &= !SR_FRE;
                }
                value
            }
            0x0c => {
                let value = state.rx;
                if !debug {
                    // And the third trap: a debugger that read `DR` would pop
                    // the guest's word and clear `RXNE`, and the guest would
                    // then read a stale one.
                    state.sr &= !SR_RXNE;
                    if state.sr & SR_OVR != 0 {
                        state.ovr_dr_read = true;
                    }
                }
                // §28.5.4: in 8-bit frame format the top half of a read is
                // forced to zero.
                if state.cr1 & CR1_DFF == 0 {
                    value & 0xff
                } else {
                    value
                }
            }
            0x10 => state.crcpr,
            0x14 => state.rxcrc,
            0x18 => state.txcrc,
            0x1c => state.i2scfgr,
            0x20 => state.i2spr,
            _ => 0,
        }
    }

    /// Write one register, reporting what has to happen once the lock is
    /// released.
    fn write_register(&self, offset: u64, value: u16) -> After {
        let mut after = After::default();
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => {
                let was_format = state.format();
                let was_nss = state.nss_output_low();
                let modf = state.sr & SR_MODF != 0;
                let mut next = value;
                if modf {
                    // §28.3.10: "hardware does not allow the setting of the
                    // SPE and MSTR bits while the MODF bit is set". This is
                    // the clause that turns a driver's missing NSS pull-up
                    // into a peripheral that silently will not start.
                    next &= !(CR1_SPE | CR1_MSTR);
                    if state.modf_sr_seen {
                        // The second half of the clearing sequence.
                        state.sr &= !SR_MODF;
                        state.modf_sr_seen = false;
                    }
                }
                let crc_rising = next & CR1_CRCEN != 0 && state.cr1 & CR1_CRCEN == 0;
                state.cr1 = next;
                if crc_rising {
                    // §28.5.5: enabling the calculator resets both registers.
                    state.rxcrc = 0;
                    state.txcrc = 0;
                }
                if state.cr1 & CR1_SPE == 0 {
                    // Disabling the peripheral stops whatever was in flight
                    // and clears `BSY` (§28.3.7).
                    state.busy = false;
                    state.sr &= !SR_BSY;
                }
                Shared::check_mode_fault(&mut state);
                if state.format() != was_format {
                    after.reframe = true;
                }
                let nss = state.nss_output_low();
                state.nss_low = nss;
                if nss != was_nss {
                    after.nss = Some(nss);
                }
                after.started = Shared::begin(&mut state);
                after.announce = true;
                self.shared.publish(&state);
            }
            0x04 => {
                let was_nss = state.nss_output_low();
                state.cr2 = value & CR2_MASK;
                Shared::check_mode_fault(&mut state);
                let nss = state.nss_output_low();
                state.nss_low = nss;
                if nss != was_nss {
                    after.nss = Some(nss);
                }
                self.shared.publish(&state);
            }
            0x08 => {
                // §28.5.3: every bit is read-only except `CRCERR`, which is
                // cleared by writing zero to it. A *write* to `SR` is also the
                // first half of the mode-fault clearing sequence.
                if state.sr & SR_MODF != 0 {
                    state.modf_sr_seen = true;
                }
                if value & SR_CRCERR == 0 {
                    state.sr &= !SR_CRCERR;
                }
            }
            0x0c => {
                let format = state.format();
                state.tx = format.truncate(u32::from(value)) as u16;
                state.tx_pending = true;
                state.sr &= !SR_TXE;
                after.started = Shared::begin(&mut state);
                self.shared.publish(&state);
            }
            0x10 => state.crcpr = value,
            // `RXCRCR` and `TXCRCR` are read-only (§28.5.6, §28.5.7).
            0x14 | 0x18 => {}
            0x1c => state.i2scfgr = value,
            0x20 => state.i2spr = value,
            _ => {}
        }
        after
    }

    /// Perform the outward half of an access.
    fn settle(&self, after: After) {
        if after.reframe {
            // The slave-side shifter caches the framing it was built with, and
            // the fabric offers no way to change it without abandoning the
            // word in flight — which is what §28.5.1 says to do anyway
            // ("`DFF` should be written only when SPI is disabled").
            self.pins.reset();
        }
        if let Some(low) = after.nss {
            self.shared.drive_nss(low);
        }
        if after.started {
            self.shared.present_first_bit();
        }
        if after.announce {
            self.shared.announce_all();
        }
        self.shared.publish_irq();
    }
}

impl MemOps for RegisterBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !matches!(dst.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + dst.len() as u64 > 4 || (dst.len() == 4 && within != 0) {
            return Err(BusError::BadAccess);
        }
        if register > LAST_REGISTER {
            dst.fill(0);
            return Ok(());
        }
        self.shared.sync(attrs);
        let value = u32::from(self.read_register(register, attrs.debug));
        let bytes = value.to_le_bytes();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = bytes[(within as usize + i).min(3)];
        }
        if !attrs.debug {
            // A read of `DR` or `SR` clears flags, and a cleared flag can drop
            // the interrupt line.
            self.shared.publish_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !matches!(src.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + src.len() as u64 > 4 || (src.len() == 4 && within != 0) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would start a frame or move a chip select, neither
            // of which the core can make harmless.
            return Err(BusError::BadAccess);
        }
        if register > LAST_REGISTER {
            return Ok(());
        }
        self.shared.sync(attrs);
        // A byte write reaches its own lane of the sixteen-bit register; the
        // rest keeps what it had, which is what a narrow store on this bus
        // does.
        let value = if src.len() == 1 && within == 1 {
            let old = self.read_register(register, true);
            (old & 0x00ff) | (u16::from(src[0]) << 8)
        } else if src.len() == 1 {
            let old = self.read_register(register, true);
            (old & 0xff00) | u16::from(src[0])
        } else {
            u16::from(src[0]) | (u16::from(src[1]) << 8)
        };
        let after = self.write_register(register, value);
        self.settle(after);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the slave face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        self.state.lock().format()
    }

    fn select(&self, selected: bool) {
        let mut state = self.state.lock();
        // NSS is what selects a slave, and it is the same pin a master watches
        // for a mode fault (§28.3.1). `SlavePins` has already turned the wire's
        // active-low level into this boolean.
        state.nss_in = Level::from_bool(!selected);
        Shared::check_mode_fault(&mut state);
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let mut state = self.state.lock();
        if state.is_master() || !state.is_enabled() {
            // Not listening. A master's own MISO input is not this path, and a
            // disabled peripheral drives nothing.
            return u32::MAX;
        }
        let out = state.shift;
        state.shift = state.tx;
        state.tx_pending = false;
        state.sr |= SR_TXE;
        Shared::finish(&mut state, mosi as u16);
        u32::from(out)
    }

    fn peek(&self) -> u32 {
        let state = self.state.lock();
        if state.is_master() || !state.is_enabled() {
            return u32::MAX;
        }
        u32::from(state.tx)
    }
}

// ---------------------------------------------------------------------------
// the input pins
// ---------------------------------------------------------------------------

/// One of the peripheral's own input pins — the ones `SlavePins` does not own.
struct InputSink {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
    line: u32,
}

impl fmt::Debug for InputSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InputSink")
            .field("line", &self.line)
            .finish()
    }
}

impl WireSink for InputSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        match self.line {
            pin::MISO_LINE => {
                self.shared.miso.store(level.as_bool(), Ordering::Relaxed);
            }
            pin::NSS_IN_LINE => {
                // The pin serves both roles: it selects this peripheral when
                // it is the slave, and it is what a master faults on.
                self.pins.drive(slave_pin::CS, level);
                let faulted = {
                    let mut state = self.shared.state.lock();
                    state.nss_in = level;
                    Shared::check_mode_fault(&mut state)
                };
                if faulted {
                    self.shared.drive_nss(false);
                    self.shared.publish_irq();
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Stm32Spi {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements connect the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // The tick is kept: `Machine::reset` does not rewind a clock
            // domain, and a lazily advanced device that zeroed its own tick
            // would then be asked to advance backwards.
            let ticks = state.ticks;
            // And the input level is kept: it belongs to whatever drives it,
            // and resetting this device does not move another device's pin.
            let nss_in = state.nss_in;
            *state = State {
                ticks,
                nss_in,
                ..State::default()
            };
            self.shared.publish(&state);
        }
        self.shared.miso.store(true, Ordering::Relaxed);
        self.pins.reset();
        self.shared.drive_nss(false);
        self.shared.announce_all();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u16(state.cr1)?;
        w.write_u16(state.cr2)?;
        w.write_u16(state.sr)?;
        w.write_u16(state.tx)?;
        w.write_u16(state.rx)?;
        w.write_u16(state.crcpr)?;
        w.write_u16(state.rxcrc)?;
        w.write_u16(state.txcrc)?;
        w.write_u16(state.i2scfgr)?;
        w.write_u16(state.i2spr)?;
        w.write_bool(state.tx_pending)?;
        w.write_u16(state.shift)?;
        w.write_bool(state.busy)?;
        w.write_bool(state.crc_frame)?;
        w.write_u64(state.started)?;
        w.write_u32(state.edges)?;
        w.write_u32(state.shift_in)?;
        w.write_bool(state.nss_low)?;
        // The two half-consumed flag-clearing sequences. A snapshot taken
        // between a driver's `DR` read and its `SR` read is a snapshot with
        // half of §28.3.10's overrun sequence done, and restoring it as
        // untouched would make the guest's next read lie.
        w.write_bool(state.ovr_dr_read)?;
        w.write_bool(state.modf_sr_seen)?;
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        w.write_bool(sck)?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)
        // `nss_in` and the MISO level are not saved: they are levels *another*
        // device is driving, and that device restores its own state and drives
        // them again (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State {
            ticks: r.read_u64()?,
            cr1: r.read_u16()?,
            cr2: r.read_u16()?,
            sr: r.read_u16()?,
            tx: r.read_u16()?,
            rx: r.read_u16()?,
            crcpr: r.read_u16()?,
            rxcrc: r.read_u16()?,
            txcrc: r.read_u16()?,
            i2scfgr: r.read_u16()?,
            i2spr: r.read_u16()?,
            tx_pending: r.read_bool()?,
            shift: r.read_u16()?,
            busy: r.read_bool()?,
            crc_frame: r.read_bool()?,
            started: r.read_u64()?,
            edges: r.read_u32()?,
            shift_in: r.read_u32()?,
            nss_low: r.read_bool()?,
            nss_in: Level::High,
            ovr_dr_read: r.read_bool()?,
            modf_sr_seen: r.read_bool()?,
        };
        let pins = (
            r.read_u32()?,
            r.read_u32()?,
            r.read_u8()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
        );
        {
            let mut slot = self.shared.state.lock();
            state.nss_in = slot.nss_in;
            *slot = state;
            self.shared.publish(&slot);
        }
        self.pins.restore(pins);
        self.shared.drive_nss(state.nss_low);
        self.shared.announce_all();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        // The slave-side data pins are the fabric's own.
        let slave_line = match port {
            pin::SCK_IN => Some(slave_pin::SCK),
            pin::MOSI_IN => Some(slave_pin::MOSI),
            _ => None,
        };
        if let Some(line) = slave_line {
            return Some(SinkPin {
                sink: self.pins.sink(line),
                line,
            });
        }
        let line = match port {
            pin::MISO => pin::MISO_LINE,
            pin::NSS_IN => pin::NSS_IN_LINE,
            _ => return None,
        };
        let sink = Arc::new(InputSink {
            shared: Arc::clone(&self.shared),
            pins: Arc::clone(&self.pins),
            line,
        });
        // Kept, because a net refers to its sinks weakly.
        self.shared.sinks.lock().push(Arc::clone(&sink));
        Some(SinkPin {
            sink: sink as Arc<dyn WireSink>,
            line,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            pin::MISO_OUT => {
                self.pins.connect_miso(source);
                return Ok(());
            }
            "irq" => {
                *self.shared.irq.lock() = Some(source);
                self.shared.publish_irq();
                return Ok(());
            }
            _ => {}
        }
        let mut pins = self.shared.pins.lock();
        match port {
            pin::SCK => pins.sck = Some(source),
            pin::MOSI => pins.mosi = Some(source),
            pin::NSS => pins.nss = Some(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an STM32 SPI drives `{}`, `{}`, `{}`, `{}` and `irq`",
                        pin::SCK,
                        pin::MOSI,
                        pin::NSS,
                        pin::MISO_OUT
                    ),
                });
            }
        }
        drop(pins);
        self.shared.announce_all();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == pin::MISO_OUT {
            self.pins.publish_miso();
        } else {
            self.shared.announce_all();
        }
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A frame takes real time, a driver polls `BSY` and `TXE` to find
    /// out when it is done, and the answer has to be the one at the cycle of
    /// the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Stm32Spi::advance_to(self, tick);
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

impl Instance for Stm32Spi {}

/// The `stm32.spi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32F4 SPI (RM0090 §28): CR1/CR2/SR/DR/CRCPR, master and slave, the four \
              CPOL/CPHA modes, 8- and 16-bit frames, SSM/SSI/SSOE and the mode fault",
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
            summary: "the named SPI bus this peripheral masters, for `transactional`",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus the NSS output stands for (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Stm32Spi::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Stm32Spi::new(props)?)))
}

/// What the validator should know about `stm32.spi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("link", ValueKind::Str)
                .required()
                .values(Link::NAMES),
        )
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .port(pin::SCK, PortDir::Out)
        .port(pin::MOSI, PortDir::Out)
        .port(pin::NSS, PortDir::Out)
        .port(pin::MISO, PortDir::In)
        .port(pin::SCK_IN, PortDir::In)
        .port(pin::MOSI_IN, PortDir::In)
        .port(pin::NSS_IN, PortDir::In)
        .port(pin::MISO_OUT, PortDir::Out)
        .port("irq", PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
