//! Game Boy cartridges: the header, and the memory bank controllers.
//!
//! A Game Boy cartridge is a ROM, optionally some RAM, optionally a battery to
//! keep the RAM alive, and — on everything but the smallest games — a **memory
//! bank controller** soldered next to them. The MBC is not a memory-mapped
//! peripheral in the usual sense: its registers are *write-only*, and they are
//! at the same addresses the ROM answers reads at. Writing `$2000` selects a
//! ROM bank; reading `$2000` reads the bank-0 byte that has always been there.
//!
//! ```text
//!   $0000-$3FFF   ROM bank 0        (MBC1 in advanced mode can bank this too)
//!   $4000-$7FFF   ROM bank 1..N     switchable
//!   $A000-$BFFF   cartridge RAM     switchable, or the MBC3 RTC registers
//! ```
//!
//! # Why this is `Io` rather than a rebasable alias
//!
//! `ROADMAP.md` §4.1 makes bank switching the motivating example of a *rebase*:
//! a fixed aperture whose contents slide, one atomic store, no flat-view
//! rebuild. That is exactly what the ROM windows are — but a region tree
//! resolves reads and writes to the *same* region, and here they have to go to
//! different places. A rebasable `Rom` alias at `$0000-$7FFF` would swallow the
//! bank-select writes (`RomWrite::Ignore`) or fault on them (`RomWrite::Fault`);
//! neither is a Game Boy. So the windows are [`Region::io`](crate::core::space::Region::io)
//! regions and the bank
//! arithmetic is one add on the read path, with the bank base kept in an atomic
//! so no lock is taken to read a byte of ROM.
//!
//! This is a real cost — no host-pointer fast path for cartridge reads, which is
//! every opcode fetch — and it is written down here rather than discovered later.
//! The generic fix is a region kind whose writes reach a handler while its reads
//! stay direct; that is a `core::space` change and phase 4 is not the place for
//! it (see the phase-4 genericity note in the machine file).
//!
//! # The RTC has its own crystal, and so does this device
//!
//! An MBC3 with a timer carries a **32.768 kHz watch crystal on the cartridge**,
//! entirely independent of the console's 4.194304 MHz one. `ROADMAP.md` §4.2 is
//! explicit that two crystals have no exact relationship and that emulating one
//! would be emulating a precision the hardware never had — so the machine file
//! declares a second oscillator and this device counts *its* ticks. Seconds are
//! [`RTC_HZ`] ticks, exactly, with no floating point anywhere.
//!
//! The device is therefore **lazily advanced** (`ROADMAP.md` §4.2): it holds its
//! own tick, and the `$A000` window catches it up before answering when an RTC
//! register is selected. A ROM read never syncs, because a ROM byte does not
//! depend on the time.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0), *The Cartridge Header*, *MBC1*,
//! *MBC3*, *MBC5*. No emulator source was consulted.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region as MmioRegion, RegionRef,
    RomStore,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;

/// The cartridge RTC's crystal, in hertz. A standard 32.768 kHz watch can.
pub const RTC_HZ: u64 = 32_768;

/// How large the `$0000-$7FFF` ROM window is.
pub const ROM_WINDOW: u64 = 0x8000;

/// How large one ROM bank is.
pub const ROM_BANK: u64 = 0x4000;

/// How large the `$A000-$BFFF` external RAM window is, and one RAM bank.
pub const RAM_BANK: u64 = 0x2000;

/// The name a `map` statement reaches the `$0000-$7FFF` window by.
pub const ROM_REGION: &str = "rom";

/// The name a `map` statement reaches the `$A000-$BFFF` window by.
pub const RAM_REGION: &str = "ram";

// ---------------------------------------------------------------------------
// The header
// ---------------------------------------------------------------------------

/// Which bank controller a cartridge carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mapper {
    /// No controller at all: 32 KiB of ROM, wired straight through.
    None,
    /// MBC1 — up to 2 MiB of ROM and 32 KiB of RAM, with a banking-mode bit
    /// that decides whether the two extra bank bits address ROM or RAM.
    Mbc1,
    /// MBC2 — 256 KiB of ROM and 512 *nibbles* of on-chip RAM.
    Mbc2,
    /// MBC3 — up to 2 MiB of ROM, 32 KiB of RAM, and optionally a real-time
    /// clock on its own crystal.
    Mbc3,
    /// MBC5 — up to 8 MiB of ROM and 128 KiB of RAM, and the first controller
    /// where bank 0 is selectable rather than special.
    Mbc5,
}

impl Mapper {
    /// The name a person would recognise.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Mapper::None => "ROM only",
            Mapper::Mbc1 => "MBC1",
            Mapper::Mbc2 => "MBC2",
            Mapper::Mbc3 => "MBC3",
            Mapper::Mbc5 => "MBC5",
        }
    }
}

impl fmt::Display for Mapper {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What the byte at `$0147` says the cartridge is.
///
/// Pan Docs, *The Cartridge Header*. The values are sparse and the unassigned
/// ones are genuinely unassigned, so an unknown byte is an error rather than a
/// guess: a cartridge whose controller we misidentify fails in ways that look
/// like a CPU bug.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CartKind {
    /// Which controller.
    pub mapper: Mapper,
    /// Whether the cartridge has RAM at `$A000`.
    pub ram: bool,
    /// Whether that RAM is battery-backed, and so belongs in a snapshot even
    /// across a power cycle.
    pub battery: bool,
    /// Whether an MBC3 carries its real-time clock.
    pub rtc: bool,
}

impl CartKind {
    const fn new(mapper: Mapper, ram: bool, battery: bool, rtc: bool) -> CartKind {
        CartKind {
            mapper,
            ram,
            battery,
            rtc,
        }
    }

    /// Decode the cartridge-type byte at `$0147`.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Option<CartKind> {
        use Mapper::{Mbc1, Mbc2, Mbc3, Mbc5, None as Plain};
        Some(match byte {
            0x00 => CartKind::new(Plain, false, false, false),
            0x01 => CartKind::new(Mbc1, false, false, false),
            0x02 => CartKind::new(Mbc1, true, false, false),
            0x03 => CartKind::new(Mbc1, true, true, false),
            0x05 => CartKind::new(Mbc2, true, false, false),
            0x06 => CartKind::new(Mbc2, true, true, false),
            0x08 => CartKind::new(Plain, true, false, false),
            0x09 => CartKind::new(Plain, true, true, false),
            0x0f => CartKind::new(Mbc3, false, true, true),
            0x10 => CartKind::new(Mbc3, true, true, true),
            0x11 => CartKind::new(Mbc3, false, false, false),
            0x12 => CartKind::new(Mbc3, true, false, false),
            0x13 => CartKind::new(Mbc3, true, true, false),
            0x19 => CartKind::new(Mbc5, false, false, false),
            0x1a => CartKind::new(Mbc5, true, false, false),
            0x1b => CartKind::new(Mbc5, true, true, false),
            // $1C-$1E are the rumble variants. The rumble motor is a wire we do
            // not drive, so they are their RAM-and-battery equivalents.
            0x1c => CartKind::new(Mbc5, false, false, false),
            0x1d => CartKind::new(Mbc5, true, false, false),
            0x1e => CartKind::new(Mbc5, true, true, false),
            _ => return None,
        })
    }
}

/// A parsed cartridge image.
#[derive(Clone)]
pub struct Cartridge {
    bytes: Arc<[u8]>,
    title: String,
    kind: CartKind,
    rom_banks: u32,
    ram_len: u64,
    header_checksum_ok: bool,
}

impl fmt::Debug for Cartridge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cartridge")
            .field("title", &self.title)
            .field("kind", &self.kind)
            .field("rom_banks", &self.rom_banks)
            .field("ram_len", &self.ram_len)
            .field("header_checksum_ok", &self.header_checksum_ok)
            .finish()
    }
}

/// One past the last header byte. The header starts at $0100.
const HEADER_END: usize = 0x0150;

impl Cartridge {
    /// Parse an image.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image is too short to hold a header, if its
    /// cartridge-type byte names a controller that does not exist, or if the
    /// declared ROM size does not match the image.
    pub fn parse(bytes: impl Into<Arc<[u8]>>) -> Result<Cartridge> {
        let bytes: Arc<[u8]> = bytes.into();
        if bytes.len() < HEADER_END {
            return Err(config(alloc::format!(
                "a cartridge image is at least {HEADER_END} bytes; this one is {}",
                bytes.len()
            )));
        }
        let kind = CartKind::from_byte(bytes[0x0147]).ok_or_else(|| {
            config(alloc::format!(
                "cartridge type ${:02x} at $0147 is not an assigned value",
                bytes[0x0147]
            ))
        })?;

        // $0148: 32 KiB shifted left by the byte. Values above 8 exist only on
        // pirate carts and are not assigned.
        let size_byte = bytes[0x0148];
        if size_byte > 8 {
            return Err(config(alloc::format!(
                "ROM size byte ${size_byte:02x} at $0148 is not an assigned value"
            )));
        }
        let rom_banks = 2u32 << size_byte;
        let declared = u64::from(rom_banks) * ROM_BANK;
        if declared != bytes.len() as u64 {
            return Err(config(alloc::format!(
                "the header declares {declared} bytes of ROM ({rom_banks} banks) but the image \
                 is {} bytes",
                bytes.len()
            )));
        }

        // $0149. The value 1 was never used on a real cartridge; Pan Docs lists
        // it as unused, and treating it as "no RAM" is what hardware does.
        let ram_len = match bytes[0x0149] {
            0x00 | 0x01 => 0,
            0x02 => RAM_BANK,
            0x03 => 4 * RAM_BANK,
            0x04 => 16 * RAM_BANK,
            0x05 => 8 * RAM_BANK,
            other => {
                return Err(config(alloc::format!(
                    "RAM size byte ${other:02x} at $0149 is not an assigned value"
                )));
            }
        };
        // MBC2's 512x4 bits live inside the controller, so its header says
        // "no RAM" and it has some anyway.
        let ram_len = if kind.mapper == Mapper::Mbc2 {
            512
        } else {
            ram_len
        };

        let title = bytes[0x0134..0x0144]
            .iter()
            .take_while(|b| **b != 0)
            .filter(|b| b.is_ascii_graphic() || **b == b' ')
            .map(|b| *b as char)
            .collect::<String>()
            .trim_end()
            .into();

        // The boot ROM refuses to start a cartridge whose header checksum is
        // wrong, and the result of the check is also what leaves H and C set in
        // the post-boot flag register. We do not refuse — a test ROM with a
        // deliberately bad header is still worth running — but we record it.
        let mut sum = 0u8;
        for byte in &bytes[0x0134..0x014d] {
            sum = sum.wrapping_sub(*byte).wrapping_sub(1);
        }
        let header_checksum_ok = sum == bytes[0x014d];

        Ok(Cartridge {
            bytes,
            title,
            kind,
            rom_banks,
            ram_len,
            header_checksum_ok,
        })
    }

    /// The game's name, as the header spells it.
    #[must_use]
    pub fn title(&self) -> &str {
        &self.title
    }

    /// What controller and features the header declares.
    #[must_use]
    pub fn kind(&self) -> CartKind {
        self.kind
    }

    /// How many 16 KiB ROM banks the image holds.
    #[must_use]
    pub fn rom_banks(&self) -> u32 {
        self.rom_banks
    }

    /// How many bytes of cartridge RAM the header declares.
    #[must_use]
    pub fn ram_len(&self) -> u64 {
        self.ram_len
    }

    /// Whether the header's own checksum agrees with the header.
    #[must_use]
    pub fn header_checksum_ok(&self) -> bool {
        self.header_checksum_ok
    }

    /// The raw image.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
}

fn config(message: impl Into<String>) -> Error {
    Error::Config {
        at: String::from("gb.cart"),
        message: message.into(),
    }
}

// ---------------------------------------------------------------------------
// The real-time clock
// ---------------------------------------------------------------------------

/// The five MBC3 clock registers, as `$08`-`$0C` select them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rtc {
    /// Seconds, 0-59.
    pub seconds: u8,
    /// Minutes, 0-59.
    pub minutes: u8,
    /// Hours, 0-23.
    pub hours: u8,
    /// The low eight bits of the day counter.
    pub day_low: u8,
    /// Bit 0 is day bit 8, bit 6 halts the clock, bit 7 is the day-overflow
    /// carry — which, once set, stays set until the program clears it.
    pub day_high: u8,
}

impl Rtc {
    /// Whether the halt bit is set, in which case the clock does not run.
    #[must_use]
    pub const fn halted(&self) -> bool {
        self.day_high & 0x40 != 0
    }

    /// Advance by one second, carrying all the way up to the day-overflow bit.
    fn tick_second(&mut self) {
        self.seconds += 1;
        if self.seconds < 60 {
            return;
        }
        self.seconds = 0;
        self.minutes += 1;
        if self.minutes < 60 {
            return;
        }
        self.minutes = 0;
        self.hours += 1;
        if self.hours < 24 {
            return;
        }
        self.hours = 0;
        let day = (u16::from(self.day_high & 1) << 8) | u16::from(self.day_low);
        let day = day.wrapping_add(1);
        self.day_low = day as u8;
        self.day_high = (self.day_high & 0xfe) | ((day >> 8) as u8 & 1);
        if day > 0x1ff {
            // Bit 7 latches the overflow and is never cleared by hardware.
            self.day_high |= 0x80;
            self.day_low = 0;
            self.day_high &= 0xfe;
        }
    }

    /// Read register `index`, which is `$08` + the register number.
    fn read(&self, index: u8) -> u8 {
        match index {
            0x08 => self.seconds,
            0x09 => self.minutes,
            0x0a => self.hours,
            0x0b => self.day_low,
            // Bits 1-5 are not implemented.
            _ => self.day_high | 0x3e,
        }
    }

    fn write(&mut self, index: u8, value: u8) {
        match index {
            // Out-of-range values are writable and count on from where they are
            // — the counters are plain and have no clamp.
            0x08 => self.seconds = value & 0x3f,
            0x09 => self.minutes = value & 0x3f,
            0x0a => self.hours = value & 0x1f,
            0x0b => self.day_low = value,
            _ => self.day_high = value & 0xc1,
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// Everything the two windows share.
struct Shared {
    cart: Cartridge,
    rom: Arc<RomStore>,
    ram: Option<Arc<RamStore>>,
    banks: Mutex<Banks>,
    /// Byte offset into the ROM that `$0000` currently reads from.
    ///
    /// Published as an atomic so that a ROM read — which is every opcode fetch —
    /// takes no lock. The mapper writes it from inside the bank lock, which is
    /// the cold path.
    rom0_base: AtomicU64,
    /// Byte offset into the ROM that `$4000` currently reads from.
    rom1_base: AtomicU64,
    /// Byte offset into the cartridge RAM that `$A000` currently reads from.
    ram_base: AtomicU64,
    /// Whether `$A000-$BFFF` answers at all.
    ram_enabled: AtomicBool,
    /// The catch-up handle, for the RTC. Its own leaf-ranked lock because it is
    /// read from inside a guest access.
    lazy: Mutex<Option<LazyHandle>>,
    /// The tick this device has simulated up to, in its own (RTC) clock domain.
    /// Republished on every advance; the scheduler reads it with its slot held
    /// at [`LockRank::LEAF`] and so it must not be behind a lock.
    tick: AtomicU64,
}

/// The mapper's registers, and the RTC.
#[derive(Debug, Clone, Copy, Default)]
struct Banks {
    /// The low bank register: five bits on MBC1, seven on MBC3, eight on MBC5.
    bank_low: u16,
    /// The high bank register: two bits on MBC1 and MBC3 (where it is the RAM
    /// bank), one on MBC5.
    bank_high: u8,
    /// MBC1's banking mode: false is simple, true is advanced.
    advanced: bool,
    /// Which RTC register `$A000` answers for, or zero for RAM.
    rtc_select: u8,
    /// The last value written to `$6000-$7FFF`, for the latch's 0-then-1 edge.
    latch_state: u8,
    /// The live clock.
    rtc: Rtc,
    /// The copy the program reads.
    rtc_latched: Rtc,
    /// Ticks of the RTC crystal not yet accounted to a whole second.
    rtc_residual: u64,
}

impl Shared {
    /// Recompute the three window bases from the bank registers.
    ///
    /// One function rather than one per mapper, because the *rule* differs per
    /// mapper but the published result does not — and a second place that
    /// computes a base is a second place to get MBC1's advanced mode wrong.
    fn republish(&self, banks: &Banks) {
        let rom_banks = u64::from(self.cart.rom_banks);
        let mask = rom_banks - 1; // always a power of two
        let (rom0, rom1) = match self.cart.kind.mapper {
            Mapper::None => (0, 1),
            Mapper::Mbc1 => {
                // Bank 0 of each 512 KiB group cannot be selected: the low five
                // bits are forced to 1 when they are zero, so $20/$40/$60 read
                // as $21/$41/$61. That is the MBC1 quirk every large game works
                // around (Pan Docs, *MBC1*).
                let low = if banks.bank_low & 0x1f == 0 {
                    1
                } else {
                    banks.bank_low & 0x1f
                };
                let high = u64::from(banks.bank_high & 3) << 5;
                let lower = if banks.advanced { high } else { 0 };
                (lower & mask, (high | u64::from(low)) & mask)
            }
            Mapper::Mbc2 => {
                let low = if banks.bank_low & 0x0f == 0 {
                    1
                } else {
                    banks.bank_low & 0x0f
                };
                (0, u64::from(low) & mask)
            }
            Mapper::Mbc3 => {
                let low = if banks.bank_low & 0x7f == 0 {
                    1
                } else {
                    banks.bank_low & 0x7f
                };
                (0, u64::from(low) & mask)
            }
            // MBC5 is the first controller where bank 0 really means bank 0.
            Mapper::Mbc5 => {
                let bank = (u64::from(banks.bank_high & 1) << 8) | u64::from(banks.bank_low & 0xff);
                (0, bank & mask)
            }
        };
        self.rom0_base.store(rom0 * ROM_BANK, Ordering::Relaxed);
        self.rom1_base.store(rom1 * ROM_BANK, Ordering::Relaxed);

        let ram_banks = match &self.ram {
            Some(ram) => (ram.len() / RAM_BANK).max(1),
            None => 1,
        };
        let ram_bank = match self.cart.kind.mapper {
            // In simple mode MBC1 cannot reach past RAM bank 0.
            Mapper::Mbc1 if !banks.advanced => 0,
            Mapper::Mbc1 | Mapper::Mbc3 => u64::from(banks.bank_high & 3),
            Mapper::Mbc5 => u64::from(banks.bank_high & 0x0f),
            _ => 0,
        };
        self.ram_base
            .store((ram_bank % ram_banks) * RAM_BANK, Ordering::Relaxed);
    }

    /// Catch the RTC up before answering an access that depends on it.
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
        // A refusal means catch-up is already running further up the stack. The
        // access still has to be answered, and answering it from where the clock
        // stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Advance the RTC to `tick` of the cartridge crystal's domain.
    fn advance_to(&self, tick: u64) {
        let now = self.tick.load(Ordering::Relaxed);
        if tick <= now {
            return;
        }
        let elapsed = tick - now;
        {
            let mut banks = self.banks.lock();
            if self.cart.kind.rtc && !banks.rtc.halted() {
                let total = banks.rtc_residual + elapsed;
                let seconds = total / RTC_HZ;
                banks.rtc_residual = total % RTC_HZ;
                // Integer division and a residual accumulator: no floating point
                // anywhere in the time path (CLAUDE.md, Determinism).
                for _ in 0..seconds.min(u64::from(u32::MAX)) {
                    banks.rtc.tick_second();
                }
            } else {
                banks.rtc_residual = 0;
            }
        }
        self.tick.store(tick, Ordering::Relaxed);
    }

    /// The tick the RTC's own next visible change falls on.
    fn next_event_tick(&self) -> Option<u64> {
        if !self.cart.kind.rtc {
            return None;
        }
        let residual = self.banks.lock().rtc_residual;
        Some(self.tick.load(Ordering::Relaxed) + (RTC_HZ - residual).max(1))
    }
}

/// A Game Boy cartridge as a device.
pub struct GbCart {
    shared: Arc<Shared>,
    rom_region: RegionRef,
    ram_region: RegionRef,
}

impl fmt::Debug for GbCart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbCart")
            .field("cart", &self.shared.cart)
            .finish_non_exhaustive()
    }
}

impl GbCart {
    /// Build a cartridge device around a parsed image.
    #[must_use]
    pub fn new(cart: Cartridge) -> GbCart {
        let rom = Arc::new(RomStore::new(cart.bytes.to_vec()));
        let ram = (cart.ram_len > 0).then(|| Arc::new(RamStore::new(cart.ram_len)));
        let shared = Arc::new(Shared {
            cart,
            rom,
            ram,
            banks: Mutex::with_rank(LockRank::DEVICE, Banks::default()),
            rom0_base: AtomicU64::new(0),
            rom1_base: AtomicU64::new(ROM_BANK),
            ram_base: AtomicU64::new(0),
            ram_enabled: AtomicBool::new(false),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
        });
        shared.republish(&Banks::default());
        let rom_region = Arc::new(MmioRegion::io(
            "gb.cart.rom",
            ROM_WINDOW,
            Arc::new(RomWindow {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let ram_region = Arc::new(MmioRegion::io(
            "gb.cart.ram",
            RAM_BANK,
            Arc::new(RamWindow {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbCart {
            shared,
            rom_region,
            ram_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If the `rom` media slot is missing or the image does not parse.
    pub fn from_props(props: &Props) -> Result<GbCart> {
        let mut r = props.reader();
        let media = r.require_media("rom")?;
        r.finish()?;
        Ok(GbCart::new(Cartridge::parse(media.to_bytes())?))
    }

    /// The parsed image.
    #[must_use]
    pub fn cartridge(&self) -> &Cartridge {
        &self.shared.cart
    }

    /// Which ROM bank `$4000` currently reads from.
    #[must_use]
    pub fn rom_bank(&self) -> u64 {
        self.shared.rom1_base.load(Ordering::Relaxed) / ROM_BANK
    }

    /// Which ROM bank `$0000` currently reads from. Always 0 except on an MBC1
    /// in advanced mode with a 1 MiB or larger image.
    #[must_use]
    pub fn rom_bank_low(&self) -> u64 {
        self.shared.rom0_base.load(Ordering::Relaxed) / ROM_BANK
    }

    /// Which RAM bank `$A000` currently reads from.
    #[must_use]
    pub fn ram_bank(&self) -> u64 {
        self.shared.ram_base.load(Ordering::Relaxed) / RAM_BANK
    }

    /// Whether cartridge RAM is currently enabled.
    #[must_use]
    pub fn ram_enabled(&self) -> bool {
        self.shared.ram_enabled.load(Ordering::Relaxed)
    }

    /// The live real-time clock, if this cartridge has one.
    #[must_use]
    pub fn rtc(&self) -> Option<Rtc> {
        self.shared
            .cart
            .kind
            .rtc
            .then(|| self.shared.banks.lock().rtc)
    }

    /// Read one byte of cartridge RAM directly — for a test or a monitor.
    ///
    /// Bypasses the enable bit and the bank register, so it is machine setup
    /// rather than a guest access.
    #[must_use]
    pub fn peek_ram(&self, offset: u64) -> Option<u8> {
        self.shared.ram.as_ref()?.read_u8(offset).ok()
    }

    /// Write one byte of cartridge RAM directly.
    pub fn poke_ram(&self, offset: u64, value: u8) {
        if let Some(ram) = &self.shared.ram {
            let _ = ram.write_u8(offset, value);
        }
    }

    /// Connect the catch-up handle the RTC syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

/// The `$0000-$7FFF` window: ROM on read, the mapper's registers on write.
struct RomWindow {
    shared: Arc<Shared>,
}

impl fmt::Debug for RomWindow {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RomWindow").finish_non_exhaustive()
    }
}

impl MemOps for RomWindow {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // No lock: the bases are atomics precisely so that an opcode fetch does
        // not take one. The two windows are indexed separately rather than by
        // subtracting `ROM_BANK` from the high base — on an MBC5 the high window
        // really can select bank 0, and that subtraction would go below zero.
        let addr = if offset < ROM_BANK {
            self.shared.rom0_base.load(Ordering::Relaxed) + offset
        } else {
            self.shared.rom1_base.load(Ordering::Relaxed) + (offset - ROM_BANK)
        };
        *byte = self
            .shared
            .rom
            .as_bytes()
            .get(addr as usize)
            .copied()
            .unwrap_or(0xff);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        let value = *value;
        let mut banks = self.shared.banks.lock();
        match self.shared.cart.kind.mapper {
            Mapper::None => {}
            Mapper::Mbc1 => match offset >> 13 {
                0 => self
                    .shared
                    .ram_enabled
                    .store(value & 0x0f == 0x0a, Ordering::Relaxed),
                1 => banks.bank_low = u16::from(value & 0x1f),
                2 => banks.bank_high = value & 0x03,
                _ => banks.advanced = value & 1 != 0,
            },
            Mapper::Mbc2 => {
                // MBC2 has no separate register window: address bit 8 chooses
                // between RAM enable and the bank number (Pan Docs, *MBC2*).
                if offset < 0x4000 {
                    if offset & 0x0100 == 0 {
                        self.shared
                            .ram_enabled
                            .store(value & 0x0f == 0x0a, Ordering::Relaxed);
                    } else {
                        banks.bank_low = u16::from(value & 0x0f);
                    }
                }
            }
            Mapper::Mbc3 => match offset >> 13 {
                0 => self
                    .shared
                    .ram_enabled
                    .store(value & 0x0f == 0x0a, Ordering::Relaxed),
                1 => banks.bank_low = u16::from(value & 0x7f),
                2 => {
                    if (0x08..=0x0c).contains(&value) {
                        banks.rtc_select = value;
                    } else {
                        banks.rtc_select = 0;
                        banks.bank_high = value & 0x03;
                    }
                }
                _ => {
                    // The latch is a 0-then-1 edge, so that a program reading
                    // five registers sees one consistent instant.
                    if banks.latch_state == 0 && value == 1 {
                        banks.rtc_latched = banks.rtc;
                    }
                    banks.latch_state = value;
                }
            },
            Mapper::Mbc5 => match offset >> 12 {
                0 | 1 => self
                    .shared
                    .ram_enabled
                    .store(value & 0x0f == 0x0a, Ordering::Relaxed),
                // MBC5 splits the bank number across two registers at $2000 and
                // $3000 rather than one, which is how it reaches 512 banks.
                2 => banks.bank_low = (banks.bank_low & 0x100) | u16::from(value),
                3 => banks.bank_low = (banks.bank_low & 0xff) | (u16::from(value & 1) << 8),
                4 | 5 => banks.bank_high = value & 0x0f,
                _ => {}
            },
        }
        self.shared.republish(&banks);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `$A000-$BFFF` window: cartridge RAM, or the MBC3 clock.
struct RamWindow {
    shared: Arc<Shared>,
}

impl fmt::Debug for RamWindow {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RamWindow").finish_non_exhaustive()
    }
}

impl MemOps for RamWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Disabled RAM reads as $FF: the window is simply not driven, and the
        // bus is pulled up.
        if !self.shared.ram_enabled.load(Ordering::Relaxed) {
            *byte = 0xff;
            return Ok(());
        }
        let select = self.shared.banks.lock().rtc_select;
        if select != 0 {
            // Only now is the time worth knowing, so only now is the clock
            // caught up. A ROM byte never depends on it.
            self.shared.sync(attrs);
            *byte = self.shared.banks.lock().rtc_latched.read(select);
            return Ok(());
        }
        let Some(ram) = &self.shared.ram else {
            *byte = 0xff;
            return Ok(());
        };
        let base = self.shared.ram_base.load(Ordering::Relaxed);
        let addr = (base + offset) % ram.len().max(1);
        *byte = ram.read_u8(addr).unwrap_or(0xff);
        // MBC2's RAM is 512 nibbles: the high half of every byte is not there.
        if self.shared.cart.kind.mapper == Mapper::Mbc2 {
            *byte |= 0xf0;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if !self.shared.ram_enabled.load(Ordering::Relaxed) {
            return Ok(());
        }
        let select = self.shared.banks.lock().rtc_select;
        if select != 0 {
            self.shared.sync(attrs);
            let mut banks = self.shared.banks.lock();
            banks.rtc.write(select, *value);
            // A write reaches the live clock *and* the latched copy: the program
            // that just set the time expects to read it back.
            banks.rtc_latched.write(select, *value);
            banks.rtc_residual = 0;
            return Ok(());
        }
        let Some(ram) = &self.shared.ram else {
            return Ok(());
        };
        let base = self.shared.ram_base.load(Ordering::Relaxed);
        let addr = (base + offset) % ram.len().max(1);
        let _ = ram.write_u8(addr, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `gb.cart` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.cart",
    version: 1,
    summary: "Game Boy cartridge: header, ROM-only, MBC1, MBC2, MBC3 (with RTC) and MBC5",
    properties: &[PropertySpec {
        name: "rom",
        kind: ValueKind::Media,
        required: true,
        summary: "the cartridge image, as the name of a media slot (`rom = \"cart\"`)",
    }],
    construct: |props| Ok(Box::new(GbCart::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbCart {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            ROM_REGION => Some(Arc::clone(&self.rom_region)),
            RAM_REGION => Some(Arc::clone(&self.ram_region)),
            _ => None,
        }
    }

    fn reset(&self, kind: ResetKind) {
        let mut banks = self.shared.banks.lock();
        let rtc = banks.rtc;
        *banks = Banks::default();
        // Battery-backed state survives a warm reset, and so does the clock —
        // the whole point of the battery is that pulling the reset line does not
        // reach it.
        if kind != ResetKind::Cold && self.shared.cart.kind.battery {
            banks.rtc = rtc;
            banks.rtc_latched = rtc;
        }
        self.shared.ram_enabled.store(false, Ordering::Relaxed);
        self.shared.republish(&banks);
        drop(banks);
        if kind == ResetKind::Cold
            && let Some(ram) = &self.shared.ram
        {
            let _ = ram.fill(0, ram.len(), 0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let banks = *self.shared.banks.lock();
        w.write_u16(banks.bank_low)?;
        w.write_u8(banks.bank_high)?;
        w.write_bool(banks.advanced)?;
        w.write_u8(banks.rtc_select)?;
        w.write_u8(banks.latch_state)?;
        w.write_bool(self.shared.ram_enabled.load(Ordering::Relaxed))?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        w.write_u64(banks.rtc_residual)?;
        for rtc in [banks.rtc, banks.rtc_latched] {
            w.write_u8(rtc.seconds)?;
            w.write_u8(rtc.minutes)?;
            w.write_u8(rtc.hours)?;
            w.write_u8(rtc.day_low)?;
            w.write_u8(rtc.day_high)?;
        }
        // Battery-backed RAM is architectural state: a snapshot that dropped it
        // would lose the player's save file (CLAUDE.md, Devices).
        match &self.shared.ram {
            Some(ram) => {
                let mut bytes = vec![0u8; ram.len() as usize];
                ram.read_at(0, &mut bytes)
                    .map_err(|e| Error::State(alloc::format!("cartridge RAM: {e}")))?;
                w.write_bytes(&bytes)?;
            }
            None => w.write_bytes(&[])?,
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut banks = Banks {
            bank_low: r.read_u16()?,
            bank_high: r.read_u8()?,
            advanced: r.read_bool()?,
            rtc_select: r.read_u8()?,
            latch_state: r.read_u8()?,
            ..Banks::default()
        };
        let ram_enabled = r.read_bool()?;
        let tick = r.read_u64()?;
        banks.rtc_residual = r.read_u64()?;
        for slot in [false, true] {
            let rtc = Rtc {
                seconds: r.read_u8()?,
                minutes: r.read_u8()?,
                hours: r.read_u8()?,
                day_low: r.read_u8()?,
                day_high: r.read_u8()?,
            };
            if slot {
                banks.rtc_latched = rtc;
            } else {
                banks.rtc = rtc;
            }
        }
        let bytes = r.read_bytes()?;
        if let Some(ram) = &self.shared.ram {
            if bytes.len() as u64 != ram.len() {
                return Err(Error::State(alloc::format!(
                    "snapshot holds {} bytes of cartridge RAM, this cartridge has {}",
                    bytes.len(),
                    ram.len()
                )));
            }
            ram.write_at(0, bytes)
                .map_err(|e| Error::State(alloc::format!("cartridge RAM: {e}")))?;
        }
        self.shared
            .ram_enabled
            .store(ram_enabled, Ordering::Relaxed);
        self.shared.tick.store(tick, Ordering::Relaxed);
        self.shared.republish(&banks);
        *self.shared.banks.lock() = banks;
        Ok(())
    }

    // -- lazily advanced, for the RTC only ----------------------------------

    fn is_lazy(&self) -> bool {
        // Even without an RTC: the machine file gives every cartridge the same
        // clock, and a cartridge that reported `false` here would make the
        // `clock =` line in the file an error for some images and required for
        // others, which is a worse machine description than one dead atomic.
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.shared.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        GbCart::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for GbCart {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(GbCart::from_props(props)?)))
}

/// What the validator should know about `gb.cart`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .region(ROM_REGION)
        .region(RAM_REGION)
}

/// Build a minimal, valid cartridge image for a test or a demonstration.
///
/// `banks` 16 KiB banks of ROM, a correct header checksum, and `program`
/// assembled at the `$0100` entry point. Public because the machine tests and
/// the PPU's tests both want one and neither should have to hand-assemble a
/// header.
///
/// # Panics
///
/// If `banks` is not a power of two between 2 and 512, or if `program` does not
/// fit between `$0100` and the header at `$0104`.
#[must_use]
pub fn synthetic_image(banks: u32, kind_byte: u8, ram_byte: u8, program: &[u8]) -> Vec<u8> {
    assert!(
        (2..=512).contains(&banks) && banks.is_power_of_two(),
        "a cartridge has a power-of-two number of banks, at least two"
    );
    let mut rom = vec![0u8; banks as usize * ROM_BANK as usize];
    // $0100-$0103 is the entry point: three bytes and a jump, conventionally
    // `NOP; JP $0150`.
    rom[0x0100] = 0x00;
    rom[0x0101] = 0xc3;
    rom[0x0102] = 0x50;
    rom[0x0103] = 0x01;
    rom[0x0150..0x0150 + program.len()].copy_from_slice(program);
    rom[0x0147] = kind_byte;
    rom[0x0148] = (banks.trailing_zeros() - 1) as u8;
    rom[0x0149] = ram_byte;
    let mut sum = 0u8;
    for byte in &rom[0x0134..0x014d] {
        sum = sum.wrapping_sub(*byte).wrapping_sub(1);
    }
    rom[0x014d] = sum;
    rom
}
