//! The STM32 reset and clock controller.
//!
//! `st.rcc` is the chip that every vendor startup sequence talks to first and
//! the one a board cannot fake with RAM. `SystemInit` sets `CR.HSEON` and spins
//! until `CR.HSERDY` comes back; it programs `PLLCFGR`, sets `CR.PLLON` and
//! spins on `CR.PLLRDY`; it writes `CFGR.SW` and spins until `CFGR.SWS` agrees.
//! A RAM cell reads back what was written, so **every one of those spins is
//! infinite** — which is why this device is the first of the STM32 peripherals
//! to be built rather than the fifth.
//!
//! # What it does
//!
//! | | |
//! | --- | --- |
//! | ready bits | each `xxxON` produces its `xxxRDY` after `ready-delay` ticks of this device's own clock domain, and drops it the moment `xxxON` clears |
//! | the switch | `CFGR.SW` is refused outright when the source it names is not ready; when it is accepted, `SWS` follows one tick later |
//! | the tree | `SYSCLK`, `HCLK`, `PCLK1`, `PCLK2`, the timer clocks and `RTCCLK` are computed as **exact rationals** from the PLL factors and the prescalers, and published through [`Clocks`] |
//! | gating | every `xxENR` and `xxRSTR` bit is an output pin, so a peripheral is told it has no clock and is told when its reset line is pulled |
//! | the backup domain | `BDCR` is write-protected until `PWR_CR.DBP` arrives on the `dbp` input, and `BDRST` clears the domain |
//! | reset causes | `CSR`'s `xxRSTF` flags are set by a pulse on the matching input pin and cleared by `RMVF` |
//!
//! # Time, and why there is no sleeping
//!
//! A crystal takes milliseconds to start and a PLL takes microseconds to lock.
//! Neither is a host-clock reading: the device is **lazily advanced**
//! ([`Device::is_lazy`]), it holds its own tick in its own clock domain, and
//! the guest access that polls `CR` is what catches it up. `ready-delay` is
//! therefore a count of *those* ticks — a board with `clock = hse` on its
//! `rcc` object is counting 125 ns ticks of an 8 MHz can — and the default is
//! short on purpose. Nothing here reads the wall clock and nothing sleeps.
//!
//! # The clock outputs
//!
//! The rates live in a [`Clocks`] handle the device publishes as
//! [`ExportId::CLOCK_TREE`], and a consumer names its RCC in the machine file
//! (`rcc = "rcc"`) and asks for it at bind time. Each output is an exact
//! [`Rational`] in hertz: `8 MHz / 8 × 336 / 2` is `168000000/1` and not a
//! rounded `f64`, because a ratio inside one oscillator's tree is exact by
//! construction (`CLAUDE.md`, *Determinism*).
//!
//! **And it re-rates the scheduler's clock domains.** A board that says
//!
//! ```text
//! osc hse = 8000000 Hz
//! object sysclk "clock" { clock = hse * 2 }      # the reset rate: HSI, 16 MHz
//! object hclk   "clock" { clock = sysclk }
//! object pclk1  "clock" { clock = hclk / 4 }
//! object pclk2  "clock" { clock = hclk / 2 }
//! object rcc "st.rcc" {
//!   clock = hse, variant = "f4", hse = 8000000,
//!   sysclk = "sysclk", hclk = "hclk", pclk1 = "pclk1", pclk2 = "pclk2"
//! }   # and `timclk1`/`timclk2` for what the timers count
//! ```
//!
//! gets a `SYSCLK` that really is 168 MHz once the guest's `SystemInit` has
//! run, and peripherals hung off `pclk1` that really do follow `PPRE1`. The
//! request goes through [`crate::core::clock::ClockControl`] and
//! is applied by the scheduler at a round boundary, never mid-round; that type
//! documents the rule and what becomes of the ticks already counted. An output
//! no board named drives nothing, which is every board that has not been
//! rewritten to the shape above — they keep the fixed ratios their machine file
//! declares, and [`Clocks`] is still the way a peripheral reads a rate.
//!
//! `drive_domains` has the two deviations this bakes in: a rating is measured
//! against the domain's parent, so HSI and the PLL are modelled as exact ratios
//! of the HSE crystal rather than as independent cans, and an output of zero
//! leaves its domain where it was rather than stopping it.
//!
//! # Sources
//!
//! * *STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
//!   Arm-based 32-bit MCUs*, ST **RM0090** rev 21, §7 "Reset and clock control
//!   (RCC)" — §7.2 for the tree, §7.3 for the register map.
//! * *STM32L4x5 and STM32L4x6 advanced Arm-based 32-bit MCUs*, ST **RM0351**
//!   rev 9, §6 "Reset and clock control (RCC)" — §6.2 for the tree, §6.4 for
//!   the register map.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # Known deviations
//!
//! * The `*LPENR` (F4) and `*SMENR` (L4) low-power gate registers are storage
//!   that reads back, and they reset to **zero** rather than to the manual's
//!   "every implemented peripheral enabled" constants. Those constants were
//!   not to hand to check, and a number transcribed from memory is worse than
//!   a stated gap. Nothing in a startup path reads them.
//! * `CIR` (F4) and `CIER`/`CIFR`/`CICR` (L4) are storage: no RCC interrupt is
//!   raised, because nothing generates a clock-security or ready event other
//!   than the ready bits themselves and no firmware in the tree asks for one.
//! * `SSCGR`, `PLLI2SCFGR`, `PLLSAICFGR`, `DCKCFGR`, `CCIPR`, `CCIPR2` and
//!   `CRRCR` read back and select nothing: their outputs have no consumer yet.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::{ClockControl, DomainId, Rational};
use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.rcc";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many 32-bit words the widest layout occupies. The L4 map ends at
/// `CCIPR2` (`+0x9c`), so forty words covers both families and the snapshot
/// encoding is one shape whatever the variant.
const WORDS: usize = 0x28;

/// How many `xxxON`/`xxxRDY` pairs the widest layout has.
const MAX_READY: usize = 8;

/// "No deadline": this ready bit is not waiting on anything.
const NO_DEADLINE: u64 = u64::MAX;

/// The default startup delay, in ticks of this device's own clock domain.
///
/// Sixteen ticks is two microseconds of an 8 MHz can — the order of a PLL lock
/// and several orders below a real crystal's millisecond startup. It is
/// deliberately short: a board that wants the crystal's real figure writes
/// `ready-delay` and gets it, and firmware that polls a ready bit cannot tell
/// the difference except in how much virtual time it spends waiting.
const DEFAULT_READY_DELAY: u64 = 16;

/// The name of the `PWR_CR.DBP` input.
pub const DBP_PIN: &str = "dbp";

/// The name of the RTC clock-enable output (`BDCR.RTCEN`).
pub const RTCEN_PIN: &str = "rtcen";

/// The name of the backup-domain reset output (`BDCR.BDRST`).
pub const BDRST_PIN: &str = "bdrst";

/// How wide a gate bank is. Every `xxENR`/`xxRSTR` is one bit per peripheral.
pub const BANK_WIDTH: u32 = 32;

// ---------------------------------------------------------------------------
// Clock outputs
// ---------------------------------------------------------------------------

/// Which output of the clock tree a consumer is asking about.
///
/// An open id space rather than an enum, the `pktkit` `EtherType` pattern
/// (`CLAUDE.md`, *Type conventions*): an H7 has clock outputs an F4 does not,
/// and adding one must not break a `match` somewhere else in the tree.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ClockOutput(pub u16);

impl ClockOutput {
    /// The system clock, after `CFGR.SW`/`SWS` has chosen its source.
    pub const SYSCLK: ClockOutput = ClockOutput(0);
    /// The AHB clock: `SYSCLK` divided by `CFGR.HPRE`.
    pub const HCLK: ClockOutput = ClockOutput(1);
    /// The APB1 (low-speed) peripheral clock: `HCLK / PPRE1`.
    pub const PCLK1: ClockOutput = ClockOutput(2);
    /// The APB2 (high-speed) peripheral clock: `HCLK / PPRE2`.
    pub const PCLK2: ClockOutput = ClockOutput(3);
    /// What an APB1 timer counts: `PCLK1`, doubled unless `PPRE1` is one.
    pub const TIMCLK1: ClockOutput = ClockOutput(4);
    /// What an APB2 timer counts: `PCLK2`, doubled unless `PPRE2` is one.
    pub const TIMCLK2: ClockOutput = ClockOutput(5);
    /// What `BDCR.RTCSEL` selects for the RTC, zero when nothing is selected.
    pub const RTCCLK: ClockOutput = ClockOutput(6);
    /// The PLL's `Q` output — 48 MHz for USB and the SDIO on a configured F4.
    pub const PLL48: ClockOutput = ClockOutput(7);

    /// How many outputs there are, which is the width of a [`Clocks`] table.
    pub const COUNT: usize = 8;

    /// The name this output is known by, for a diagnostic.
    #[must_use]
    pub fn name(self) -> Option<&'static str> {
        Some(match self {
            ClockOutput::SYSCLK => "sysclk",
            ClockOutput::HCLK => "hclk",
            ClockOutput::PCLK1 => "pclk1",
            ClockOutput::PCLK2 => "pclk2",
            ClockOutput::TIMCLK1 => "timclk1",
            ClockOutput::TIMCLK2 => "timclk2",
            ClockOutput::RTCCLK => "rtcclk",
            ClockOutput::PLL48 => "pll48",
            _ => return None,
        })
    }
}

impl fmt::Display for ClockOutput {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "clock output #{}", self.0),
        }
    }
}

/// The clock outputs a machine file may hand `st.rcc` a domain for, each with
/// the output it hangs off — RM0090 §7.2's tree, in the order it is walked.
///
/// A property of this name takes the *object* whose clock domain is that
/// output; `None` for a parent means the crystal itself, which is the `hse`
/// property. Declaration order is top-down, so a single pass over it rates
/// every output against something already settled.
pub const OUTPUT_TREE: &[(ClockOutput, &str)] = &[
    (ClockOutput::SYSCLK, "sysclk"),
    (ClockOutput::HCLK, "hclk"),
    (ClockOutput::PCLK1, "pclk1"),
    (ClockOutput::PCLK2, "pclk2"),
    (ClockOutput::TIMCLK1, "timclk1"),
    (ClockOutput::TIMCLK2, "timclk2"),
];

/// Which output `out` hangs off, or `None` when it hangs off the crystal.
fn parent_output(out: ClockOutput) -> Option<ClockOutput> {
    Some(match out {
        ClockOutput::HCLK => ClockOutput::SYSCLK,
        ClockOutput::PCLK1 | ClockOutput::PCLK2 => ClockOutput::HCLK,
        ClockOutput::TIMCLK1 => ClockOutput::PCLK1,
        ClockOutput::TIMCLK2 => ClockOutput::PCLK2,
        _ => return None,
    })
}

/// The rates an `st.rcc` is driving, as a consumer sees them.
///
/// Published as [`ExportId::CLOCK_TREE`]. A peripheral holds one of these from
/// bind time onwards — it is **wiring, not guest state**, so it is never
/// serialized and it survives reset.
///
/// Rates are exact [`Rational`] hertz. A stopped or unselected output reads as
/// zero, which is the honest answer for "the guest has not switched this on"
/// and is distinguishable from any real frequency.
///
/// [`Clocks::generation`] counts changes. A consumer that caches a derived
/// number — a baud divisor, a prescaler reload — compares against the
/// generation it cached at and recomputes when it differs; that is cheaper
/// than recomputing per access and it cannot go stale.
#[derive(Debug)]
pub struct Clocks {
    rates: Mutex<[Rational; ClockOutput::COUNT]>,
    generation: AtomicU64,
}

impl Clocks {
    /// A tree with every output stopped.
    fn new() -> Clocks {
        Clocks {
            rates: Mutex::with_rank(LockRank::LEAF, [Rational::integer(0); ClockOutput::COUNT]),
            generation: AtomicU64::new(0),
        }
    }

    /// The exact rate of `out`, in hertz. Zero when that output is stopped.
    #[must_use]
    pub fn rate(&self, out: ClockOutput) -> Rational {
        self.rates
            .lock()
            .get(usize::from(out.0))
            .copied()
            .unwrap_or(Rational::integer(0))
    }

    /// How many times any rate has changed since the machine was built.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Publish a recomputed table, bumping the generation if anything moved.
    ///
    /// Returns whether anything did, so a caller with more to do about a change
    /// — re-rating the scheduler's domains — does it only when there is one.
    fn publish(&self, next: [Rational; ClockOutput::COUNT]) -> bool {
        {
            let mut rates = self.rates.lock();
            if *rates == next {
                return false;
            }
            *rates = next;
        }
        // Release, so a consumer that reads the generation and then the rates
        // cannot see the new number with the old table.
        self.generation.fetch_add(1, Ordering::Release);
        true
    }
}

// ---------------------------------------------------------------------------
// Variants and layout
// ---------------------------------------------------------------------------

/// Which family's register map this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090's F4 map: `PLLCFGR` at `+0x04`, `BDCR` at `+0x70`.
    F4,
    /// RM0351's L4 map: `ICSCR` at `+0x04`, `PLLCFGR` at `+0x0c`, `BDCR` at
    /// `+0x90`, and an MSI the F4 does not have.
    L4,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::L4 => "l4",
        }
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // Through `DCKCFGR` at `+0x8c` (RM0090 §7.3.23).
            Variant::F4 => 0x90,
            // Through `CCIPR2` at `+0x9c` (RM0351 §6.4.31).
            Variant::L4 => 0xa0,
        }
    }

    fn layout(self) -> &'static Layout {
        match self {
            Variant::F4 => &F4,
            Variant::L4 => &L4,
        }
    }

    /// The reset-cause pins this family offers.
    fn causes(self) -> &'static [ResetCause] {
        match self {
            Variant::F4 => F4_CAUSES,
            Variant::L4 => L4_CAUSES,
        }
    }
}

/// Which oscillator or PLL a ready bit belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Osc {
    /// The internal high-speed RC — 16 MHz on both families.
    Hsi,
    /// The external high-speed crystal.
    Hse,
    /// The L4's multi-speed internal RC.
    Msi,
    /// The main PLL.
    Pll,
    /// A secondary PLL: `PLLI2S`/`PLLSAI` on an F4, `PLLSAI1`/`PLLSAI2` on an
    /// L4. Modelled for its ready bit only — nothing consumes its output yet.
    Aux,
    /// The 32.768 kHz backup-domain crystal.
    Lse,
    /// The internal low-speed RC.
    Lsi,
}

/// An `xxxON`/`xxxRDY` pair and what it switches on.
#[derive(Debug, Clone, Copy)]
struct ReadyBit {
    /// The register both bits live in, as a byte offset.
    reg: u64,
    /// The `xxxON` bit the guest writes.
    on: u32,
    /// The `xxxRDY` bit the hardware answers with.
    rdy: u32,
    /// Which source it starts.
    osc: Osc,
}

/// One bank of peripheral gate bits: a register and the pin prefix a machine
/// file wires it by.
#[derive(Debug, Clone, Copy)]
struct Bank {
    /// The pin prefix — `ahb1en0` … `ahb1en31`.
    prefix: &'static str,
    /// The register's byte offset.
    reg: u64,
}

/// A `CSR` reset flag and the input pin that latches it.
#[derive(Debug, Clone, Copy)]
struct ResetCause {
    pin: &'static str,
    bit: u32,
}

/// Everything that differs between the two families.
#[derive(Debug)]
struct Layout {
    cr: u64,
    cfgr: u64,
    pllcfgr: u64,
    bdcr: u64,
    csr: u64,
    /// `RMVF`'s bit in `CSR` — 24 on an F4, 23 on an L4.
    rmvf: u32,
    /// The clock-enable banks, in register order.
    enable: &'static [Bank],
    /// The peripheral-reset banks, in register order.
    reset: &'static [Bank],
    /// Every `xxxON`/`xxxRDY` pair.
    ready: &'static [ReadyBit],
    /// Reset values, as `(offset, value)`. Everything unnamed resets to zero.
    reset_values: &'static [(u64, u32)],
}

/// Every gate-pin prefix either layout uses, so the schema can declare the
/// union.
///
/// `apb1en` is `APB1ENR` on an F4 and `APB1ENR1` on an L4; `apb1enb` is the
/// L4's **second** APB1 word and an F4 has none. A `connect` to a bank the
/// variant does not have is refused by name.
///
/// The second word is `apb1enb` rather than `apb1en2` because a bank pin is a
/// prefix followed by decimal digits, and `apb1en21` would then be two pins
/// with one spelling — `APB1ENR1` bit 21 and `APB1ENR2` bit 1. A letter cannot
/// be read as a digit, so the ambiguity cannot arise.
const ALL_BANKS: &[&str] = &[
    "ahb1en", "ahb2en", "ahb3en", "apb1en", "apb1enb", "apb2en", "ahb1rst", "ahb2rst", "ahb3rst",
    "apb1rst", "apb1rstb", "apb2rst",
];

/// Every reset-cause pin either layout offers, for the schema's union.
const ALL_CAUSES: &[&str] = &[
    "borrst", "pinrst", "porrst", "sftrst", "iwdgrst", "wwdgrst", "lpwrrst", "fwrst", "oblrst",
];

// -- the F4 map (RM0090 §7.3) ------------------------------------------------

static F4_ENABLE: &[Bank] = &[
    Bank {
        prefix: "ahb1en",
        reg: 0x30,
    },
    Bank {
        prefix: "ahb2en",
        reg: 0x34,
    },
    Bank {
        prefix: "ahb3en",
        reg: 0x38,
    },
    Bank {
        prefix: "apb1en",
        reg: 0x40,
    },
    Bank {
        prefix: "apb2en",
        reg: 0x44,
    },
];

static F4_RESET_BANKS: &[Bank] = &[
    Bank {
        prefix: "ahb1rst",
        reg: 0x10,
    },
    Bank {
        prefix: "ahb2rst",
        reg: 0x14,
    },
    Bank {
        prefix: "ahb3rst",
        reg: 0x18,
    },
    Bank {
        prefix: "apb1rst",
        reg: 0x20,
    },
    Bank {
        prefix: "apb2rst",
        reg: 0x24,
    },
];

static F4_READY: &[ReadyBit] = &[
    ReadyBit {
        reg: 0x00,
        on: 0,
        rdy: 1,
        osc: Osc::Hsi,
    },
    ReadyBit {
        reg: 0x00,
        on: 16,
        rdy: 17,
        osc: Osc::Hse,
    },
    ReadyBit {
        reg: 0x00,
        on: 24,
        rdy: 25,
        osc: Osc::Pll,
    },
    // `PLLI2S`, then `PLLSAI` — the second exists only on an F42x/F43x and is
    // harmless on an F407, where the bit is reserved and firmware never sets
    // it.
    ReadyBit {
        reg: 0x00,
        on: 26,
        rdy: 27,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x00,
        on: 28,
        rdy: 29,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x70,
        on: 0,
        rdy: 1,
        osc: Osc::Lse,
    },
    ReadyBit {
        reg: 0x74,
        on: 0,
        rdy: 1,
        osc: Osc::Lsi,
    },
];

static F4_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: HSI on and, once it is ready, HSIRDY — the chip runs from HSI out
    // of reset. HSITRIM defaults to 16, which is the `0x80` (RM0090 §7.3.1,
    // "Reset value: 0x0000 XX83").
    (0x00, 0x0000_0083),
    // `PLLCFGR`: M = 16, N = 192, P = /2, Q = 4 (§7.3.2).
    (0x04, 0x2400_3010),
    // `AHB1ENR`: the CCM data RAM's clock is on out of reset (§7.3.10).
    (0x30, 0x0010_0000),
    // `CSR`: a power-on sets the BOR, pin and POR reset flags (§7.3.21).
    (0x74, 0x0e00_0000),
];

static F4_CAUSES: &[ResetCause] = &[
    ResetCause {
        pin: "borrst",
        bit: 25,
    },
    ResetCause {
        pin: "pinrst",
        bit: 26,
    },
    ResetCause {
        pin: "porrst",
        bit: 27,
    },
    ResetCause {
        pin: "sftrst",
        bit: 28,
    },
    ResetCause {
        pin: "iwdgrst",
        bit: 29,
    },
    ResetCause {
        pin: "wwdgrst",
        bit: 30,
    },
    ResetCause {
        pin: "lpwrrst",
        bit: 31,
    },
];

static F4: Layout = Layout {
    cr: 0x00,
    cfgr: 0x08,
    pllcfgr: 0x04,
    bdcr: 0x70,
    csr: 0x74,
    rmvf: 24,
    enable: F4_ENABLE,
    reset: F4_RESET_BANKS,
    ready: F4_READY,
    reset_values: F4_RESET_VALUES,
};

// -- the L4 map (RM0351 §6.4) ------------------------------------------------

static L4_ENABLE: &[Bank] = &[
    Bank {
        prefix: "ahb1en",
        reg: 0x48,
    },
    Bank {
        prefix: "ahb2en",
        reg: 0x4c,
    },
    Bank {
        prefix: "ahb3en",
        reg: 0x50,
    },
    Bank {
        prefix: "apb1en",
        reg: 0x58,
    },
    Bank {
        prefix: "apb1enb",
        reg: 0x5c,
    },
    Bank {
        prefix: "apb2en",
        reg: 0x60,
    },
];

static L4_RESET_BANKS: &[Bank] = &[
    Bank {
        prefix: "ahb1rst",
        reg: 0x28,
    },
    Bank {
        prefix: "ahb2rst",
        reg: 0x2c,
    },
    Bank {
        prefix: "ahb3rst",
        reg: 0x30,
    },
    Bank {
        prefix: "apb1rst",
        reg: 0x38,
    },
    Bank {
        prefix: "apb1rstb",
        reg: 0x3c,
    },
    Bank {
        prefix: "apb2rst",
        reg: 0x40,
    },
];

static L4_READY: &[ReadyBit] = &[
    ReadyBit {
        reg: 0x00,
        on: 0,
        rdy: 1,
        osc: Osc::Msi,
    },
    // The L4's `HSION` is bit 8 and its `HSIRDY` is bit **10**, not 9 — bit 9
    // is `HSIKERON` (RM0351 §6.4.1). The pair is a table for exactly this.
    ReadyBit {
        reg: 0x00,
        on: 8,
        rdy: 10,
        osc: Osc::Hsi,
    },
    ReadyBit {
        reg: 0x00,
        on: 16,
        rdy: 17,
        osc: Osc::Hse,
    },
    ReadyBit {
        reg: 0x00,
        on: 24,
        rdy: 25,
        osc: Osc::Pll,
    },
    ReadyBit {
        reg: 0x00,
        on: 26,
        rdy: 27,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x00,
        on: 28,
        rdy: 29,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x90,
        on: 0,
        rdy: 1,
        osc: Osc::Lse,
    },
    ReadyBit {
        reg: 0x94,
        on: 0,
        rdy: 1,
        osc: Osc::Lsi,
    },
];

static L4_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: MSI on at range 6, which is 4 MHz — the L4 boots on the MSI, not
    // on an HSI (RM0351 §6.4.1, "Reset value: 0x0000 0063").
    (0x00, 0x0000_0063),
    // `PLLCFGR` and the two SAI PLLs: N = 16, everything else zero (§6.4.4).
    (0x0c, 0x0000_1000),
    (0x10, 0x0000_1000),
    (0x14, 0x0000_1000),
    // `CSR`: MSISRANGE = 4 MHz, and a power-on sets the BOR and pin flags
    // (§6.4.29).
    (0x94, 0x0c00_0600),
];

static L4_CAUSES: &[ResetCause] = &[
    ResetCause {
        pin: "fwrst",
        bit: 24,
    },
    ResetCause {
        pin: "oblrst",
        bit: 25,
    },
    ResetCause {
        pin: "pinrst",
        bit: 26,
    },
    ResetCause {
        pin: "borrst",
        bit: 27,
    },
    ResetCause {
        pin: "sftrst",
        bit: 28,
    },
    ResetCause {
        pin: "iwdgrst",
        bit: 29,
    },
    ResetCause {
        pin: "wwdgrst",
        bit: 30,
    },
    ResetCause {
        pin: "lpwrrst",
        bit: 31,
    },
];

static L4: Layout = Layout {
    cr: 0x00,
    cfgr: 0x08,
    pllcfgr: 0x0c,
    bdcr: 0x90,
    csr: 0x94,
    rmvf: 23,
    enable: L4_ENABLE,
    reset: L4_RESET_BANKS,
    ready: L4_READY,
    reset_values: L4_RESET_VALUES,
};

// ---------------------------------------------------------------------------
// The declared source frequencies
// ---------------------------------------------------------------------------

/// What the board says its oscillators run at.
///
/// The crystal on `OSC_IN` is a fact about the **board**, not about the part,
/// so it arrives as a property. It has to agree with the `osc` the machine
/// file declares for the same crystal, and nothing checks that today — see the
/// module documentation on the missing clock-control seam.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frequencies {
    /// The external high-speed crystal, in hertz.
    pub hse: u64,
    /// The internal high-speed RC — 16 MHz on both families.
    pub hsi: u64,
    /// The backup-domain crystal.
    pub lse: u64,
    /// The internal low-speed RC.
    pub lsi: u64,
}

impl Default for Frequencies {
    fn default() -> Frequencies {
        Frequencies {
            // The STM32F4 Discovery board's can, and the one
            // `machines/stm32f407.machine` declares.
            hse: 8_000_000,
            hsi: 16_000_000,
            lse: 32_768,
            lsi: 32_000,
        }
    }
}

/// The L4's MSI ranges, `CR.MSIRANGE` = 0…11 (RM0351 §6.4.1).
const MSI_RANGES: [u64; 12] = [
    100_000, 200_000, 400_000, 800_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000,
    24_000_000, 32_000_000, 48_000_000,
];

/// `CFGR.HPRE` as a divisor (RM0090 §7.3.3, RM0351 §6.4.3 — the same table).
fn hpre_div(field: u32) -> u64 {
    match field {
        0..=7 => 1,
        8 => 2,
        9 => 4,
        10 => 8,
        11 => 16,
        12 => 64,
        13 => 128,
        14 => 256,
        _ => 512,
    }
}

/// `CFGR.PPREx` as a divisor. `0xx` is one; otherwise `2^(field - 3)`.
fn ppre_div(field: u32) -> u64 {
    if field < 4 { 1 } else { 1 << (field - 3) }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// When each ready bit comes true, in this device's ticks; [`NO_DEADLINE`]
    /// for one that is not waiting.
    ready_at: [u64; MAX_READY],
    /// When `SWS` catches up with `SW`, and what it will then say.
    switch_at: u64,
    switch_to: u32,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(layout: &Layout) -> State {
        let mut words = [0u32; WORDS];
        for (offset, value) in layout.reset_values {
            words[(*offset / 4) as usize] = *value;
        }
        State {
            words,
            ready_at: [NO_DEADLINE; MAX_READY],
            switch_at: NO_DEADLINE,
            switch_to: 0,
            tick: 0,
        }
    }

    #[inline]
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    #[inline]
    fn word_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The output key of the `rtcen` pin, above every bank.
const KEY_RTCEN: u32 = 0xffff_0000;
/// The output key of the `bdrst` pin.
const KEY_BDRST: u32 = 0xffff_0001;

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    layout: &'static Layout,
    freq: Frequencies,
    ready_delay: u64,
    /// The published rates. Derived state; never serialized.
    clocks: Arc<Clocks>,
    /// The connected gate outputs, keyed by `bank index * 32 + bit`, plus the
    /// two named backup-domain pins at [`KEY_RTCEN`] and [`KEY_BDRST`].
    outputs: Mutex<Vec<(u32, WireSource)>>,
    /// `PWR_CR.DBP`, as it arrives on the `dbp` input. Lock-free because the
    /// `BDCR` write path samples it while holding the state lock, and reaching
    /// into `PWR` from there would be one device-rank lock inside another.
    dbp: AtomicU32,
    /// Whether anything is driving `dbp` at all.
    dbp_wired: AtomicU32,
    /// The lock-free half of the lazy contract: `current_tick` and
    /// `next_event_tick` are called at [`LockRank::LEAF`] and must not lock.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
    /// Which object the machine file named for each driven clock output, in
    /// the order the chain is measured in. Wiring, fixed at construction.
    named: Vec<(ClockOutput, String)>,
    /// The domains those names resolved to, filled in at bind.
    domains: Mutex<Vec<(ClockOutput, DomainId)>>,
    /// The clock-control seam, or `None` for an instance nobody registered —
    /// a unit test holding the device directly.
    control: Mutex<Option<Arc<ClockControl>>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("freq", &self.freq)
            .field("ready-delay", &self.ready_delay);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    // -- the lazy seam -----------------------------------------------------

    /// Catch the device up to the present before answering an access.
    ///
    /// Called with **no lock held**: the handle re-enters this device through
    /// [`Device::advance_to`], which takes the state lock.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else { return };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }

    /// Republish what the lock-free lazy surface reads.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let mut next = state.switch_at;
        for at in state.ready_at {
            next = next.min(at);
        }
        // The contract says a reported event must be strictly in the future,
        // or catch-up makes no progress and the device stalls where it stands.
        let next = if next <= state.tick { u64::MAX } else { next };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Simulate forward to `tick`: ready bits come true, `SWS` catches up.
    fn advance_to(&self, tick: u64) {
        {
            let mut state = self.state.lock();
            if tick <= state.tick {
                return;
            }
            state.tick = tick;
            for (i, bit) in self.layout.ready.iter().enumerate() {
                if state.ready_at[i] <= tick {
                    state.ready_at[i] = NO_DEADLINE;
                    *state.word_mut(bit.reg) |= 1 << bit.rdy;
                }
            }
            if state.switch_at <= tick {
                state.switch_at = NO_DEADLINE;
                let sws = state.switch_to;
                let cfgr = state.word_mut(self.layout.cfgr);
                *cfgr = (*cfgr & !0b1100) | (sws << 2);
            }
            self.republish(&state);
        }
        // Outward, after the critical section: the rates may have moved and a
        // consumer of `Clocks` is a stranger (`CLAUDE.md`, re-entrancy).
        self.recompute();
    }

    // -- the clock tree ----------------------------------------------------

    /// Whether the source behind `osc` is ready. Anything else is a rate of
    /// zero, which is what "the guest has not switched it on" looks like.
    fn osc_ready(&self, state: &State, osc: Osc) -> bool {
        self.layout
            .ready
            .iter()
            .any(|b| b.osc == osc && state.word(b.reg) & (1 << b.rdy) != 0)
    }

    /// The MSI's programmed rate (L4 only).
    ///
    /// `CR.MSIRGSEL` chooses whether the range comes from `CR.MSIRANGE` or
    /// from `CSR.MSISRANGE`, which is the range a standby wake-up comes back
    /// at (RM0351 §6.4.1).
    fn msi_hz(&self, state: &State) -> u64 {
        let cr = state.word(self.layout.cr);
        let range = if cr & (1 << 3) != 0 {
            (cr >> 4) & 0xf
        } else {
            (state.word(self.layout.csr) >> 8) & 0xf
        };
        MSI_RANGES
            .get(range as usize)
            .copied()
            .unwrap_or(MSI_RANGES[6])
    }

    /// The rate of one source, in hertz, or zero when it is not running.
    fn source_hz(&self, state: &State, osc: Osc) -> Rational {
        if !self.osc_ready(state, osc) {
            return Rational::integer(0);
        }
        Rational::integer(match osc {
            Osc::Hsi => self.freq.hsi,
            Osc::Hse => self.freq.hse,
            Osc::Lse => self.freq.lse,
            Osc::Lsi => self.freq.lsi,
            Osc::Msi => self.msi_hz(state),
            // Neither is a source in its own right.
            Osc::Pll | Osc::Aux => 0,
        })
    }

    /// The main PLL's system output and its 48 MHz output.
    ///
    /// Exact rational arithmetic throughout: a multiplier and a divider inside
    /// one oscillator's tree are integers and the ratio between them is exact
    /// by construction (`CLAUDE.md`, *Determinism*).
    fn pll(&self, state: &State) -> (Rational, Rational) {
        let zero = Rational::integer(0);
        if !self.osc_ready(state, Osc::Pll) {
            return (zero, zero);
        }
        let cfg = state.word(self.layout.pllcfgr);
        match self.variant {
            Variant::F4 => {
                // RM0090 §7.3.2: M[5:0], N[14:6], P[17:16] (÷ 2·(P+1)),
                // SRC[22], Q[27:24].
                let m = u64::from(cfg & 0x3f);
                let n = u64::from((cfg >> 6) & 0x1ff);
                let p = 2 * (u64::from((cfg >> 16) & 0x3) + 1);
                let q = u64::from((cfg >> 24) & 0xf);
                let src = if cfg & (1 << 22) != 0 {
                    Osc::Hse
                } else {
                    Osc::Hsi
                };
                if m == 0 || n == 0 {
                    return (zero, zero);
                }
                let Some(vco) = self.source_hz(state, src).checked_scale(n, m) else {
                    return (zero, zero);
                };
                let sys = vco.checked_scale(1, p).unwrap_or(zero);
                let out48 = if q == 0 {
                    zero
                } else {
                    vco.checked_scale(1, q).unwrap_or(zero)
                };
                (sys, out48)
            }
            Variant::L4 => {
                // RM0351 §6.4.4: SRC[1:0], M[6:4] (÷ M+1), N[14:8],
                // Q[22:21] (÷ 2·(Q+1)) behind QEN[20], R[26:25] (÷ 2·(R+1))
                // behind REN[24].
                let src = match cfg & 0x3 {
                    1 => Osc::Msi,
                    2 => Osc::Hsi,
                    3 => Osc::Hse,
                    _ => return (zero, zero),
                };
                let m = u64::from((cfg >> 4) & 0x7) + 1;
                let n = u64::from((cfg >> 8) & 0x7f);
                let q = 2 * (u64::from((cfg >> 21) & 0x3) + 1);
                let r = 2 * (u64::from((cfg >> 25) & 0x3) + 1);
                if n == 0 {
                    return (zero, zero);
                }
                let Some(vco) = self.source_hz(state, src).checked_scale(n, m) else {
                    return (zero, zero);
                };
                let sys = if cfg & (1 << 24) != 0 {
                    vco.checked_scale(1, r).unwrap_or(zero)
                } else {
                    zero
                };
                let out48 = if cfg & (1 << 20) != 0 {
                    vco.checked_scale(1, q).unwrap_or(zero)
                } else {
                    zero
                };
                (sys, out48)
            }
        }
    }

    /// What `BDCR.RTCSEL` has selected, gated by `RTCEN`.
    fn rtc_hz(&self, state: &State) -> Rational {
        let zero = Rational::integer(0);
        let bdcr = state.word(self.layout.bdcr);
        if bdcr & (1 << 15) == 0 {
            return zero;
        }
        match ((bdcr >> 8) & 0x3, self.variant) {
            (1, _) => self.source_hz(state, Osc::Lse),
            (2, _) => self.source_hz(state, Osc::Lsi),
            // An F4 divides HSE by `CFGR.RTCPRE`, and a value below 2 means no
            // clock (RM0090 §7.3.3). An L4 divides it by a fixed 32
            // (RM0351 §6.4.3).
            (3, Variant::F4) => {
                let pre = u64::from((state.word(self.layout.cfgr) >> 16) & 0x1f);
                if pre < 2 {
                    zero
                } else {
                    self.source_hz(state, Osc::Hse)
                        .checked_scale(1, pre)
                        .unwrap_or(zero)
                }
            }
            (3, Variant::L4) => self
                .source_hz(state, Osc::Hse)
                .checked_scale(1, 32)
                .unwrap_or(zero),
            _ => zero,
        }
    }

    /// The whole tree, from the registers as they now stand.
    fn rates(&self, state: &State) -> [Rational; ClockOutput::COUNT] {
        let zero = Rational::integer(0);
        let cfgr = state.word(self.layout.cfgr);
        let (pll_sys, pll48) = self.pll(state);
        // `SWS`, not `SW`: what the system is *running on* is what the
        // hardware answers with, and the two differ for one tick after a
        // switch.
        let sws = (cfgr >> 2) & 0x3;
        let sysclk = match (self.variant, sws) {
            // RM0090 §7.3.3: 00 HSI, 01 HSE, 10 PLL.
            (Variant::F4, 0) => self.source_hz(state, Osc::Hsi),
            (Variant::F4, 1) => self.source_hz(state, Osc::Hse),
            (Variant::F4, 2) => pll_sys,
            // RM0351 §6.4.3: 00 MSI, 01 HSI16, 10 HSE, 11 PLL.
            (Variant::L4, 0) => self.source_hz(state, Osc::Msi),
            (Variant::L4, 1) => self.source_hz(state, Osc::Hsi),
            (Variant::L4, 2) => self.source_hz(state, Osc::Hse),
            (Variant::L4, 3) => pll_sys,
            _ => zero,
        };

        let (ppre1_shift, ppre2_shift) = match self.variant {
            // RM0090 §7.3.3: PPRE1[12:10], PPRE2[15:13].
            Variant::F4 => (10, 13),
            // RM0351 §6.4.3: PPRE1[10:8], PPRE2[13:11].
            Variant::L4 => (8, 11),
        };
        let hpre = (cfgr >> 4) & 0xf;
        let ppre1 = (cfgr >> ppre1_shift) & 0x7;
        let ppre2 = (cfgr >> ppre2_shift) & 0x7;

        let hclk = sysclk.checked_scale(1, hpre_div(hpre)).unwrap_or(zero);
        let pclk1 = hclk.checked_scale(1, ppre_div(ppre1)).unwrap_or(zero);
        let pclk2 = hclk.checked_scale(1, ppre_div(ppre2)).unwrap_or(zero);
        // "If APBx prescaler is 1, the timer clock is PCLKx, else 2 × PCLKx"
        // (RM0090 §7.2, RM0351 §6.2).
        let tim1 = if ppre_div(ppre1) == 1 {
            pclk1
        } else {
            pclk1.checked_scale(2, 1).unwrap_or(zero)
        };
        let tim2 = if ppre_div(ppre2) == 1 {
            pclk2
        } else {
            pclk2.checked_scale(2, 1).unwrap_or(zero)
        };

        let mut out = [zero; ClockOutput::COUNT];
        out[usize::from(ClockOutput::SYSCLK.0)] = sysclk;
        out[usize::from(ClockOutput::HCLK.0)] = hclk;
        out[usize::from(ClockOutput::PCLK1.0)] = pclk1;
        out[usize::from(ClockOutput::PCLK2.0)] = pclk2;
        out[usize::from(ClockOutput::TIMCLK1.0)] = tim1;
        out[usize::from(ClockOutput::TIMCLK2.0)] = tim2;
        out[usize::from(ClockOutput::RTCCLK.0)] = self.rtc_hz(state);
        out[usize::from(ClockOutput::PLL48.0)] = pll48;
        out
    }

    /// Recompute and publish the tree. Takes the state lock briefly and makes
    /// no outward call while holding it.
    fn recompute(&self) {
        let next = {
            let state = self.state.lock();
            self.rates(&state)
        };
        if self.clocks.publish(next) {
            self.drive_domains(&next);
        }
    }

    /// Ask the scheduler to re-rate whichever clock domains this controller was
    /// told are its outputs.
    ///
    /// # What a rating is measured against
    ///
    /// A [`ClockForest`](crate::core::clock::ClockForest) domain is rated
    /// against its **parent**, so a request is a ratio and never a frequency.
    /// The tree the machine file is expected to have built is the one RM0090
    /// §7.2 draws, and [`OUTPUT_TREE`] is that shape written down: `SYSCLK` off
    /// the crystal, `HCLK` off `SYSCLK`, `PCLK1` and `PCLK2` off `HCLK`, and
    /// each timer clock off its own APB clock. Every output is rated against
    /// the **nearest ancestor this instance was actually given**, falling all
    /// the way back to the `hse` reference: a board that names only `pclk1`
    /// gets `pclk1 / hse`, which is right if that is how it wired the domain
    /// and wrong if it did not.
    ///
    /// `timclk1`/`timclk2` are outputs of their own rather than a `× 2` a board
    /// could write itself, because the doubling is conditional — RM0090 §7.2:
    /// "if the APB prescaler is 1 the timer clock is PCLKx, else 2 × PCLKx" —
    /// and a machine file that froze either answer would be wrong half the
    /// time.
    ///
    /// **`hse` is the reference, and it has to be the truth.** The property
    /// already has to agree with the board's `osc hse` — `stm32f407.machine`
    /// says so at the point it writes the number twice — and this is the second
    /// thing that depends on it.
    ///
    /// # The deviation this bakes in
    ///
    /// An output expressed as a ratio of the domain's parent cannot say *which*
    /// crystal it came from. A board with one high-speed oscillator therefore
    /// models HSI and the PLL as exact ratios of HSE: the rate a guest measures
    /// is exactly right, and the physical independence of the two cans is not
    /// modelled. Saying it properly needs a reparent across trees at the moment
    /// `SWS` changes, which is a bigger seam than this one and is not here.
    ///
    /// An output of zero — its source is stopped, or `SWS` names something that
    /// is not running — leaves its domain at the rate it had rather than
    /// stopping it. Stopping a domain is
    /// [`ClockForest::set_gated`](crate::core::clock::ClockForest::set_gated)'s
    /// business and belongs with the peripheral clock-enable half of the
    /// problem, not here.
    fn drive_domains(&self, rates: &[Rational; ClockOutput::COUNT]) {
        let control = self.control.lock().clone();
        let Some(control) = control else { return };
        let outputs = self.domains.lock().clone();
        if outputs.is_empty() {
            return;
        }
        let reference = Rational::integer(self.freq.hse);
        if reference.is_zero() {
            return;
        }
        for (out, _) in OUTPUT_TREE {
            let Some(domain) = outputs.iter().find(|(o, _)| o == out).map(|(_, d)| *d) else {
                continue;
            };
            let hz = rates[usize::from(out.0)];
            if hz.is_zero() {
                continue;
            }
            // Up the tree until something this instance drives is found: that
            // domain is the parent the machine file must have given it, so that
            // is what the ratio is against.
            let mut against = reference;
            let mut cursor = *out;
            while let Some(parent) = parent_output(cursor) {
                if outputs.iter().any(|(o, _)| *o == parent) {
                    against = rates[usize::from(parent.0)];
                    break;
                }
                cursor = parent;
            }
            if against.is_zero() {
                continue;
            }
            if let Some(ratio) = hz.checked_div(against) {
                control.request_ratio(domain, ratio);
            }
        }
    }

    // -- the gate outputs --------------------------------------------------

    /// The key a bank's bit is filed under.
    fn key(bank: u32, bit: u32) -> u32 {
        bank * BANK_WIDTH + bit
    }

    /// Which register and bit a key names, or `None` for the named pins.
    fn key_source(&self, key: u32) -> Option<(u64, u32)> {
        if key >= KEY_RTCEN {
            return None;
        }
        let bank = (key / BANK_WIDTH) as usize;
        let bit = key % BANK_WIDTH;
        let banks = self.layout.enable.len();
        let reg = if bank < banks {
            self.layout.enable[bank].reg
        } else {
            self.layout.reset.get(bank - banks)?.reg
        };
        Some((reg, bit))
    }

    /// Drive every connected gate pin to whatever the registers now say.
    ///
    /// Called with **no lock held**. The state is copied out first, the wire
    /// list second, and the outward `set` calls last — a sink is free to call
    /// back into this device, and a level a net is already at is not delivered
    /// at all, so refreshing every connected pin is cheaper than diffing.
    fn refresh_outputs(&self) {
        let words = self.state.lock().words;
        let bdcr = words[(self.layout.bdcr / 4) as usize];
        let pending: Vec<(WireSource, Level)> = {
            let outputs = self.outputs.lock();
            outputs
                .iter()
                .map(|(key, source)| {
                    let high = match *key {
                        KEY_RTCEN => bdcr & (1 << 15) != 0,
                        KEY_BDRST => bdcr & (1 << 16) != 0,
                        key => match self.key_source(key) {
                            Some((reg, bit)) => words[(reg / 4) as usize] & (1 << bit) != 0,
                            None => false,
                        },
                    };
                    (source.clone(), Level::from_bool(high))
                })
                .collect()
        };
        for (source, level) in pending {
            source.set(level);
        }
    }

    // -- the register file -------------------------------------------------

    /// Which bits of `offset` the hardware owns and a guest write cannot set.
    fn read_only(&self, offset: u64) -> u32 {
        let mut mask = 0;
        for bit in self.layout.ready {
            if bit.reg == offset {
                mask |= 1 << bit.rdy;
            }
        }
        if offset == self.layout.cfgr {
            // `SWS` answers `SW`; it is never written.
            mask |= 0b1100;
        }
        if offset == self.layout.csr {
            // The reset flags, which only `RMVF` clears.
            mask |= 0xff00_0000 & !(1 << self.layout.rmvf);
        }
        mask
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        let value = state.word(offset);
        if offset == self.layout.csr {
            // `RMVF` is "cleared by writing 1" and reads back zero.
            return value & !(1 << self.layout.rmvf);
        }
        value
    }

    /// Write one register, then whatever it implies.
    ///
    /// The critical section ends before any wire moves and before the rate
    /// table is published, which is the re-entrancy contract in one function.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            let before = state.word(offset);

            if offset == self.layout.bdcr {
                self.write_bdcr(&mut state, value);
            } else {
                let keep = self.read_only(offset);
                *state.word_mut(offset) = (before & keep) | (value & !keep);
            }

            if offset == self.layout.csr && value & (1 << self.layout.rmvf) != 0 {
                // "RMVF: remove reset flag … cleared by software by writing 1."
                *state.word_mut(offset) &= !0xff00_0000;
            }

            self.arm_ready_bits(&mut state, offset, before);

            if offset == self.layout.cfgr {
                self.arm_switch(&mut state, before);
            }
            self.republish(&state);
        }
        self.refresh_outputs();
        self.recompute();
    }

    /// `BDCR`, which is in the backup domain and write-protected by
    /// `PWR_CR.DBP`.
    ///
    /// RM0090 §7.3.20: "after reset, these bits are write-protected and the
    /// DBP bit in the Power control register has to be set before these can be
    /// modified". The protected set is the whole writable register — `LSEON`,
    /// `LSEBYP`, `RTCSEL`, `RTCEN` and `BDRST` — and not a subset of it.
    fn write_bdcr(&self, state: &mut State, value: u32) {
        if self.dbp_wired.load(Ordering::Relaxed) != 0 && self.dbp.load(Ordering::Relaxed) == 0 {
            // Dropped on the floor, exactly as the hardware drops it. A board
            // with no `st.pwr` wired to `dbp` has nothing modelling the
            // protection, so it gets none rather than a backup domain that can
            // never be opened.
            return;
        }
        let keep = self.read_only(self.layout.bdcr);
        let before = state.word(self.layout.bdcr);
        let next = (before & keep) | (value & !keep);
        *state.word_mut(self.layout.bdcr) = next;
        if next & (1 << 16) != 0 {
            // "BDRST: backup domain software reset" — everything in the domain
            // goes, the LSE and its ready bit included, while the bit stands.
            *state.word_mut(self.layout.bdcr) = 1 << 16;
            for (i, bit) in self.layout.ready.iter().enumerate() {
                if bit.reg == self.layout.bdcr {
                    state.ready_at[i] = NO_DEADLINE;
                }
            }
        }
    }

    /// Start or stop whatever `offset`'s `xxxON` bits just changed.
    fn arm_ready_bits(&self, state: &mut State, offset: u64, before: u32) {
        let now = state.tick;
        let after = state.word(offset);
        for (i, bit) in self.layout.ready.iter().enumerate() {
            if bit.reg != offset {
                continue;
            }
            let was_on = before & (1 << bit.on) != 0;
            let is_on = after & (1 << bit.on) != 0;
            if is_on && !was_on {
                state.ready_at[i] = now.saturating_add(self.ready_delay);
            } else if !is_on && was_on {
                // "cleared by hardware when the oscillator is switched off."
                state.ready_at[i] = NO_DEADLINE;
                *state.word_mut(offset) &= !(1 << bit.rdy);
            }
        }
    }

    /// Accept or refuse a `CFGR.SW` change.
    ///
    /// A source that is not ready is refused outright: the `SW` field keeps
    /// the value it had and `SWS` never moves. Firmware that polls `xxxRDY`
    /// before switching — which is every vendor sequence — never notices.
    fn arm_switch(&self, state: &mut State, before: u32) {
        let cfgr = state.word(self.layout.cfgr);
        let requested = cfgr & 0x3;
        let previous = before & 0x3;
        if requested == previous {
            return;
        }
        let osc = match (self.variant, requested) {
            (Variant::F4, 0) => Osc::Hsi,
            (Variant::F4, 1) => Osc::Hse,
            (Variant::F4, 2) => Osc::Pll,
            (Variant::L4, 0) => Osc::Msi,
            (Variant::L4, 1) => Osc::Hsi,
            (Variant::L4, 2) => Osc::Hse,
            (Variant::L4, 3) => Osc::Pll,
            // An F4's `SW = 11` is reserved.
            _ => {
                *state.word_mut(self.layout.cfgr) = (cfgr & !0x3) | previous;
                return;
            }
        };
        if !self.osc_ready(state, osc) {
            *state.word_mut(self.layout.cfgr) = (cfgr & !0x3) | previous;
            return;
        }
        state.switch_at = state.tick.saturating_add(1);
        state.switch_to = requested;
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Before the lock, and harmless for a debug access: catching the
        // device up is what makes a ready bit come true under a polling loop,
        // and a debug read advances nothing (`ROADMAP.md` §15, invariant 5).
        self.sync(attrs);
        let value = self.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would start an oscillator and one to an
            // `ENR` would move a peripheral's gate wire. Neither can be made
            // harmless, so it is refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers have to be accessed by words (32 bits)."
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// What an input pin of the RCC does when it changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputKind {
    /// `PWR_CR.DBP`, which unlocks `BDCR`.
    Dbp,
    /// A reset source, which latches a `CSR` flag on its rising edge.
    Cause(u32),
}

/// One of the RCC's input pins, as something a wire can drive.
///
/// Holds the register block rather than the [`Rcc`]: the device owns the pin,
/// and a pin that owned the device back would be a cycle nothing could drop.
#[derive(Debug)]
pub struct InputPin {
    regs: Arc<Registers>,
    kind: InputKind,
    inputs: FanIn,
}

impl InputPin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        match self.kind {
            InputKind::Dbp => {
                // One atomic and nothing else: the `BDCR` write path samples
                // it and there is no outward call to make.
                self.regs.dbp.store(u32::from(high), Ordering::Relaxed);
            }
            InputKind::Cause(bit) => {
                if !high {
                    return;
                }
                // The flag is latched on the rising edge and stays until
                // `RMVF`; the pulse's width is not modelled and does not
                // matter.
                let mut state = self.regs.state.lock();
                let csr = self.regs.layout.csr;
                *state.word_mut(csr) |= 1 << bit;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 reset and clock controller.
#[derive(Debug)]
pub struct Rcc {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins this device owns. A net holds only a weak reference.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Rcc {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Rcc> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "l4"])? {
            "l4" => Variant::L4,
            _ => Variant::F4,
        };
        let defaults = Frequencies::default();
        let freq = Frequencies {
            hse: r.or("hse", defaults.hse)?,
            hsi: r.or("hsi", defaults.hsi)?,
            lse: r.or("lse", defaults.lse)?,
            lsi: r.or("lsi", defaults.lsi)?,
        };
        let ready_delay = r.or("ready-delay", DEFAULT_READY_DELAY)?;
        let mut named: Vec<(ClockOutput, String)> = Vec::new();
        for (out, prop) in OUTPUT_TREE {
            if let Some(name) = r.optional_str(prop)? {
                named.push((*out, String::from(name)));
            }
        }
        r.finish()?;
        Ok(Rcc::with_outputs(variant, freq, ready_delay, named))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_config(variant: Variant, freq: Frequencies, ready_delay: u64) -> Rcc {
        Rcc::with_outputs(variant, freq, ready_delay, Vec::new())
    }

    /// Build one that drives clock domains, naming the object behind each
    /// output.
    ///
    /// The names are resolved against the machine's objects at bind time; an
    /// output nobody named drives nothing. See `drive_domains` for what a
    /// rating is measured against.
    #[must_use]
    pub fn with_outputs(
        variant: Variant,
        freq: Frequencies,
        ready_delay: u64,
        named: Vec<(ClockOutput, String)>,
    ) -> Rcc {
        let layout = variant.layout();
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(layout)),
            variant,
            layout,
            freq,
            ready_delay,
            clocks: Arc::new(Clocks::new()),
            outputs: Mutex::with_rank(LockRank::WIRE, Vec::new()),
            dbp: AtomicU32::new(0),
            dbp_wired: AtomicU32::new(0),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            named,
            domains: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            control: Mutex::with_rank(LockRank::LEAF, None),
        });
        regs.recompute();
        let region = Arc::new(Region::io(
            "rcc",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rcc {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// The rates this controller is driving.
    ///
    /// The same handle [`ExportId::CLOCK_TREE`] publishes, for a caller that
    /// already has the device in hand.
    #[must_use]
    pub fn clocks(&self) -> Arc<Clocks> {
        Arc::clone(&self.regs.clocks)
    }

    /// Whether the gate bit `bank`'s bit `bit` is set.
    ///
    /// `bank` is the pin prefix — `"apb1en"`, `"ahb1rst"` — which is what a
    /// machine file names and what a test asks about. `None` when this variant
    /// has no such bank.
    #[must_use]
    pub fn gate(&self, bank: &str, bit: u32) -> Option<bool> {
        let key = bank_key(self.regs.layout, bank, bit)?;
        let (reg, _) = self.regs.key_source(key)?;
        Some(self.regs.state.lock().word(reg) & (1 << bit) != 0)
    }

    /// Set `PWR_CR.DBP` directly, for a test with no wire in hand.
    ///
    /// Also marks the input as driven, so the backup domain is protected from
    /// this moment on.
    pub fn set_dbp(&self, high: bool) {
        self.regs.dbp_wired.store(1, Ordering::Relaxed);
        self.regs.dbp.store(u32::from(high), Ordering::Relaxed);
    }
}

/// The output key for `bank`'s bit `bit`, or `None` if this layout has no such
/// bank.
fn bank_key(layout: &Layout, bank: &str, bit: u32) -> Option<u32> {
    if bit >= BANK_WIDTH {
        return None;
    }
    if let Some(i) = layout.enable.iter().position(|b| b.prefix == bank) {
        return Some(Registers::key(i as u32, bit));
    }
    let i = layout.reset.iter().position(|b| b.prefix == bank)?;
    Some(Registers::key((layout.enable.len() + i) as u32, bit))
}

/// Split `port` into a declared bank prefix and a bit number.
fn parse_bank_pin(layout: &Layout, port: &str) -> Option<u32> {
    // Order does not matter: every prefix is followed by digits, and no
    // prefix is another prefix followed by digits — see [`ALL_BANKS`].
    for name in ALL_BANKS {
        if let Some(bit) = port_index(port, name, BANK_WIDTH) {
            return bank_key(layout, name, bit);
        }
    }
    None
}

/// The banks this layout offers, for a diagnostic.
fn banks_of(layout: &Layout) -> String {
    let mut names: Vec<&str> = layout.enable.iter().map(|b| b.prefix).collect();
    names.extend(layout.reset.iter().map(|b| b.prefix));
    names.join(", ")
}

impl Device for Rcc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements bring the pins.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let backup = state.word(self.regs.layout.bdcr);
            let flags = state.word(self.regs.layout.csr) & 0xff00_0000;
            let tick = state.tick;
            *state = State::reset(self.regs.layout);
            state.tick = tick;
            if kind != ResetKind::Cold {
                // "Battery-backed and always-on state survives": the backup
                // domain is exactly that, and so is the record of why the part
                // reset.
                *state.word_mut(self.regs.layout.bdcr) = backup;
                let csr = state.word_mut(self.regs.layout.csr);
                *csr = (*csr & !0xff00_0000) | flags;
            }
            self.regs.republish(&state);
        }
        self.regs.refresh_outputs();
        self.regs.recompute();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        for at in state.ready_at {
            w.write_u64(at)?;
        }
        w.write_u64(state.switch_at)?;
        w.write_u32(state.switch_to)?;
        // The controller's own position in its domain: the scheduler restores
        // the domain, and without this the two would disagree and every
        // pending ready bit would come true at the wrong instant.
        w.write_u64(state.tick)
        // The wire handles, the `dbp` level and the published rate table are
        // the machine's wiring and derived state, not the chip's (invariant 3).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(self.regs.layout);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        for at in &mut state.ready_at {
            *at = r.read_u64()?;
        }
        state.switch_at = r.read_u64()?;
        state.switch_to = r.read_u32()?;
        state.tick = r.read_u64()?;
        {
            let mut held = self.regs.state.lock();
            *held = state;
            self.regs.republish(&held);
        }
        self.regs.refresh_outputs();
        self.regs.recompute();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::CLOCK_TREE)
            .then(|| Export::Opaque(Arc::clone(&self.regs.clocks) as Arc<_>))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let key = match port {
            RTCEN_PIN => KEY_RTCEN,
            BDRST_PIN => KEY_BDRST,
            other => parse_bank_pin(self.regs.layout, other).ok_or_else(|| Error::Config {
                at: port.to_string(),
                message: format!(
                    "an `{}` RCC drives `{RTCEN_PIN}`, `{BDRST_PIN}` and bits 0…{} of each of \
                     {}",
                    self.regs.variant.as_str(),
                    BANK_WIDTH - 1,
                    banks_of(self.regs.layout),
                ),
            })?,
        };
        self.regs.outputs.lock().push((key, source));
        self.regs.refresh_outputs();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh_outputs();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let kind = if port == DBP_PIN {
            self.regs.dbp_wired.store(1, Ordering::Relaxed);
            InputKind::Dbp
        } else {
            InputKind::Cause(
                self.regs
                    .variant
                    .causes()
                    .iter()
                    .find(|c| c.pin == port)?
                    .bit,
            )
        };
        let line = match kind {
            InputKind::Dbp => 0,
            InputKind::Cause(bit) => bit,
        };
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            kind,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    fn is_lazy(&self) -> bool {
        // A ready bit comes true some number of ticks after the guest asked
        // for it, and the guest's poll of `CR` is what has to see it. The
        // scheduler owns that time; this device never sleeps and never reads
        // the host clock.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }

    fn attach_clock_control(&self, control: Arc<ClockControl>) {
        *self.regs.control.lock() = Some(control);
        // The rates were computed before this arrived — at construction, and
        // again at every reset — so the first request has to be made from here
        // rather than from the next one. Without it a part that comes out of
        // reset on its internal RC would leave its domains at whatever the
        // machine file declared until the guest happened to touch `CFGR`.
        let rates = *self.regs.clocks.rates.lock();
        self.regs.drive_domains(&rates);
    }
}

impl Instance for Rcc {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let mut resolved = Vec::with_capacity(self.regs.named.len());
        for (out, name) in &self.regs.named {
            let peer = ctx.peer(name)?;
            let domain = peer.domain().ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: format!(
                    "`{out}` names `{name}`, which has no clock domain of its own; give it a \
                     rate with `clock = …` — an `object {name} \"clock\"` is the usual way to \
                     say that this is a node of the tree and nothing else"
                ),
            })?;
            resolved.push((*out, domain));
        }
        *self.regs.domains.lock() = resolved;
        Ok(())
    }
}

/// The `st.rcc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 reset and clock control: the ready bits, the PLL and prescaler tree, \
              the peripheral gates and the backup domain",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\" (RM0090) or \"l4\" (RM0351)",
        },
        PropertySpec {
            name: "hse",
            kind: ValueKind::Uint,
            required: false,
            summary: "the external high-speed crystal, in Hz (default 8000000)",
        },
        PropertySpec {
            name: "hsi",
            kind: ValueKind::Uint,
            required: false,
            summary: "the internal high-speed RC, in Hz (default 16000000)",
        },
        PropertySpec {
            name: "lse",
            kind: ValueKind::Uint,
            required: false,
            summary: "the backup-domain crystal, in Hz (default 32768)",
        },
        PropertySpec {
            name: "lsi",
            kind: ValueKind::Uint,
            required: false,
            summary: "the internal low-speed RC, in Hz (default 32000)",
        },
        PropertySpec {
            name: "ready-delay",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many ticks of this device's clock domain an oscillator takes to \
                      become ready (default 16)",
        },
        PropertySpec {
            name: "sysclk",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is SYSCLK; it must hang off the `hse` \
                      crystal, and this controller re-rates it as the registers move",
        },
        PropertySpec {
            name: "hclk",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is HCLK; it must hang off `sysclk`",
        },
        PropertySpec {
            name: "pclk1",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is PCLK1; it must hang off `hclk`",
        },
        PropertySpec {
            name: "pclk2",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is PCLK2; it must hang off `hclk`",
        },
        PropertySpec {
            name: "timclk1",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is what an APB1 timer counts; it must \
                      hang off `pclk1`, and is PCLK1 doubled unless PPRE1 is one",
        },
        PropertySpec {
            name: "timclk2",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is what an APB2 timer counts; it must \
                      hang off `pclk2`, and is PCLK2 doubled unless PPRE2 is one",
        },
    ],
    construct: |props| Ok(Box::new(Rcc::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rcc::new(props)?)))
}

/// What the validator should know about `st.rcc`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4"]))
        .prop(PropSchema::new("hse", ValueKind::Uint))
        .prop(PropSchema::new("hsi", ValueKind::Uint))
        .prop(PropSchema::new("lse", ValueKind::Uint))
        .prop(PropSchema::new("lsi", ValueKind::Uint))
        .prop(PropSchema::new("ready-delay", ValueKind::Uint))
        .region("")
        .region("regs")
        .port(RTCEN_PIN, PortDir::Out)
        .port(BDRST_PIN, PortDir::Out)
        .port(DBP_PIN, PortDir::In);
    // The driven clock outputs, from the one table that says what they are.
    for (_, prop) in OUTPUT_TREE {
        schema = schema.prop(PropSchema::new(*prop, ValueKind::Str));
    }
    // The union of both layouts' pins: a `wire` to one this variant does not
    // have is refused by the device, which is where the variant is known.
    for bank in ALL_BANKS {
        schema = schema.port_bank(*bank, PortDir::Out, BANK_WIDTH);
    }
    for cause in ALL_CAUSES {
        schema = schema.port(*cause, PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests;
