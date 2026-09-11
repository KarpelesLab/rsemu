//! An STM32 TIM: the basic, general-purpose and advanced-control timers.
//!
//! One IP block with feature subsets, which is how ST documents it and how it
//! is modelled here — a single class, `st.tim`, with `variant`, `width` and
//! `channels` as construction properties:
//!
//! | `variant` | The parts it is | What it adds |
//! | --- | --- | --- |
//! | `"basic"` | `TIM6`, `TIM7` | nothing: a counter, a prescaler, `ARR`, and one update interrupt |
//! | `"general"` | `TIM2`–`TIM5`, `TIM9`–`TIM14` | capture/compare channels, down- and center-aligned counting |
//! | `"advanced"` | `TIM1`, `TIM8` | `RCR` repetition, `BDTR.MOE`, complementary outputs, split interrupt vectors |
//!
//! `width` is 16 or 32 — `TIM2` and `TIM5` on an F4 are the 32-bit ones — and
//! `channels` is how many capture/compare channels the instance bonds, which
//! is 4 on `TIM2`–`TIM5` and `TIM1`/`TIM8`, 2 on `TIM9`/`TIM12`/`TIM15`, and 1
//! on `TIM10`, `TIM11`, `TIM13`, `TIM14`, `TIM16` and `TIM17`.
//!
//! # What is modelled, and what is not
//!
//! This is a **first** TIM, and it says what it is rather than implying it is
//! the whole peripheral. Modelled:
//!
//! * The counter, the prescaler and the auto-reload, in all three counting
//!   directions: up, down, and center-aligned with `CMS`.
//! * The **shadow registers**. `PSC` is always buffered and `ARR` is buffered
//!   when `CR1.ARPE` is set, so a write lands at the next update event and not
//!   before; `CCRx` is buffered when `CCMRx.OCxPE` is set. Clearing the preload
//!   bit pushes the written value through immediately, which is what RM0090
//!   §17.4.14's "the new value is taken into account immediately" means.
//! * The update event: overflow, underflow, or `EGR.UG`, gated by `CR1.UDIS`
//!   and `CR1.URS`, and on an advanced timer divided by `RCR`. It reloads every
//!   shadow, sets `SR.UIF`, and stops the counter when `CR1.OPM` is set.
//! * Output compare, every mode of `OCxM`: frozen, active/inactive/toggle on
//!   match, the two forced levels, and PWM modes 1 and 2. Each channel is an
//!   output **wire** a board routes wherever the pin goes.
//! * `CCER`'s enables and polarities, `BDTR.MOE`, the complementary outputs
//!   `CH1N`–`CH3N`, and the `CR2.OISx` idle levels the outputs take when `MOE`
//!   is clear.
//! * `SR`'s write-zero-to-clear flags, `DIER`'s enables, and the interrupt
//!   output — one `irq` pin for a general or basic timer, which is what the
//!   vector table gives them, plus `irq-up` and `irq-cc` on an advanced timer,
//!   which has a vector each.
//!
//! **Not** modelled, deliberately, and each of these is a register that reads
//! back what was written and does nothing else:
//!
//! * **Input capture.** `CCMRx`'s input side (`CCxS`, `ICxPSC`, `ICxF`) is
//!   storage and no channel wire is an input. A capture needs an edge on a pin
//!   the machine drives, and nothing in this tree drives one yet.
//! * **The slave-mode controller**, `SMCR`: external clock modes, the trigger
//!   and gated modes, encoder mode, and `CR2.MMS`/`TRGO`. Chaining one timer
//!   to another is a pair of features — a master output and a slave input — and
//!   half of it is worse than neither.
//! * **DMA.** `DIER`'s `UDE`/`CCxDE`/`TDE` bits and `DCR`/`DMAR` are storage;
//!   there is no DMA controller to request from.
//! * **Dead-time insertion and the break input.** `BDTR` is stored, `MOE`
//!   acts, and `DTG` does not: a complementary pair switches with zero dead
//!   time. `BKIN` is not a pin.
//! * **`CR2.CCPC`/`CCUS` preloaded channel control** and the commutation event.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) on its own clock domain, one tick of
//! which is one `CK_INT` cycle. Nothing here reads a host clock or sleeps:
//! [`Tim::next_event_tick`] reports the exact tick at which the next thing a
//! program can observe happens — the next overflow, the next underflow, the
//! next compare match — and the scheduler will not let anything run past it.
//! So an interrupt lands on the cycle it is due on rather than at the end of a
//! quantum, and the arithmetic that finds that cycle is integer throughout:
//!
//! ```text
//!   counter clocks available in n ticks = (psc_counter + n) / (PSC + 1)
//!   ticks spent producing k of them     = k * (PSC + 1) - psc_counter
//! ```
//!
//! There is no floating point and no wall clock anywhere in this file, and the
//! prescaler's phase (`psc_counter`) is part of the snapshot, so a restore
//! resumes mid-division rather than rounding to the next counter clock.
//!
//! # The clock input
//!
//! The machine file binds the domain, exactly as it does for [`super::i2c`]:
//!
//! ```text
//!   object tim2 "st.tim" { clock = hse * 21 / 4, variant = "general", width = 32 }
//! ```
//!
//! That expression is **`CK_INT` itself**, not the APB bus clock. On an F4 the
//! two differ: RM0090 §7.2 says that when an APB prescaler is other than 1 the
//! timer clocks are twice the APB clock, so a board doubles the ratio itself
//! rather than this device guessing which half of that rule applies. `CR1.CKD`
//! is stored and does not divide the counter — it is the dead-time and digital
//! filter divider, neither of which is modelled.
//!
//! Peripheral clock **gating** — `RCC_APB1ENR` — is not an input here. When a
//! clock controller lands it may either publish a gated domain, in which case
//! this device needs no change, or drive an enable, in which case the seam is
//! one new input pin. Neither is invented in advance.
//!
//! # Sources
//!
//! ST **RM0090** rev 21: §17 ("General-purpose timers TIM2 to TIM5"), §18
//! ("Advanced-control timers TIM1 and TIM8") and §19 ("Basic timers TIM6 and
//! TIM7"), with §7.2 for where `CK_INT` comes from. ST **RM0351** §31–§34 is
//! the same map on an L4. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.tim";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes the register block occupies: `CR1` through `DMAR`.
pub const REGISTER_BYTES: u64 = 0x50;

/// The most capture/compare channels any variant has.
pub const MAX_CHANNELS: usize = 4;

/// How many of those have a complementary output on an advanced timer.
///
/// `CH4` has none — RM0090 §18.3.9's Table 61 stops at `OC3N`.
pub const COMPLEMENTARY_CHANNELS: usize = 3;

/// The combined interrupt output, and the only one a basic or general-purpose
/// timer has.
pub const IRQ_PIN: &str = "irq";

/// The advanced timer's update vector — `TIM1_UP` on an F4.
pub const IRQ_UP_PIN: &str = "irq-up";

/// The advanced timer's capture/compare vector — `TIM1_CC` on an F4.
pub const IRQ_CC_PIN: &str = "irq-cc";

// -- CR1 ---------------------------------------------------------------------

/// `CR1.CEN` — counter enable.
const CR1_CEN: u32 = 1 << 0;
/// `CR1.UDIS` — update disable: no update event is generated at all.
const CR1_UDIS: u32 = 1 << 1;
/// `CR1.URS` — update request source: only an over/underflow sets `UIF`.
const CR1_URS: u32 = 1 << 2;
/// `CR1.OPM` — one-pulse mode: `CEN` clears at the next update event.
const CR1_OPM: u32 = 1 << 3;
/// `CR1.DIR` — direction, when `CMS` is edge-aligned. 1 is down.
const CR1_DIR: u32 = 1 << 4;
/// `CR1.CMS` — center-aligned mode selection, bits 6:5.
const CR1_CMS_SHIFT: u32 = 5;
/// `CR1.ARPE` — auto-reload preload enable.
const CR1_ARPE: u32 = 1 << 7;

/// What a basic timer implements of `CR1`: RM0090 §19.4.1.
const CR1_MASK_BASIC: u32 = CR1_CEN | CR1_UDIS | CR1_URS | CR1_OPM | CR1_ARPE;
/// What the others implement: everything above plus `DIR`, `CMS` and `CKD`.
const CR1_MASK_FULL: u32 = 0x03ff;

// -- CR2 ---------------------------------------------------------------------

/// `CR2.MMS` — master mode selection, bits 6:4. Stored; `TRGO` is not modelled.
const CR2_MASK_BASIC: u32 = 0x0070;
/// `CR2` on a general-purpose timer: `CCDS`, `MMS`, `TI1S`.
const CR2_MASK_GENERAL: u32 = 0x00f8;
/// `CR2` on an advanced timer: the above plus `CCPC`, `CCUS` and the `OISx`
/// idle levels at bits 8–14.
const CR2_MASK_ADVANCED: u32 = 0x7ffd;

/// Bit 8 + 2n is `OISn+1`, the idle level channel n takes when `MOE` is clear.
const CR2_OIS_SHIFT: u32 = 8;

// -- DIER --------------------------------------------------------------------

/// `DIER.UIE` — update interrupt enable.
const DIER_UIE: u32 = 1 << 0;
/// `DIER.CC1IE` — bit 1 + n is channel n's.
const DIER_CC1IE: u32 = 1 << 1;

/// A basic timer has `UIE` and `UDE` and nothing else.
const DIER_MASK_BASIC: u32 = 0x0101;
/// A general-purpose timer's: `UIE`, `CCxIE`, `TIE`, `UDE`, `CCxDE`, `TDE`.
const DIER_MASK_GENERAL: u32 = 0x5f5f;
/// An advanced timer's: the above plus `COMIE`, `BIE` and `COMDE`.
const DIER_MASK_ADVANCED: u32 = 0x7fff;

// -- SR ----------------------------------------------------------------------

/// `SR.UIF` — update interrupt flag.
const SR_UIF: u32 = 1 << 0;
/// `SR.CC1IF` — bit 1 + n is channel n's.
const SR_CC1IF: u32 = 1 << 1;

/// Every flag a basic timer has.
const SR_MASK_BASIC: u32 = SR_UIF;
/// A general-purpose timer's: `UIF`, `CCxIF`, `TIF`, `CCxOF`.
const SR_MASK_GENERAL: u32 = 0x1e5f & !0x0020;
/// An advanced timer's: the above plus `COMIF` and `BIF`.
const SR_MASK_ADVANCED: u32 = 0x1eff;

// -- EGR ---------------------------------------------------------------------

/// `EGR.UG` — re-initialize the counter and generate an update of the registers.
const EGR_UG: u32 = 1 << 0;
/// `EGR.CC1G` — bit 1 + n generates channel n's capture/compare event.
const EGR_CC1G: u32 = 1 << 1;

// -- CCMR / CCER -------------------------------------------------------------

/// `CCMRx.OCxPE` within a channel's byte.
const CCMR_OCPE: u32 = 1 << 3;
/// `CCMRx.OCxM` within a channel's byte: bits 6:4.
const CCMR_OCM_SHIFT: u32 = 4;
/// `CCMRx.CCxS` within a channel's byte: bits 1:0. Nonzero is input mode.
const CCMR_CCS_MASK: u32 = 0x3;

/// `CCER.CCxE` within a channel's nibble.
const CCER_CCE: u32 = 1 << 0;
/// `CCER.CCxP` within a channel's nibble.
const CCER_CCP: u32 = 1 << 1;
/// `CCER.CCxNE` within a channel's nibble — advanced timers only.
const CCER_CCNE: u32 = 1 << 2;
/// `CCER.CCxNP` within a channel's nibble.
const CCER_CCNP: u32 = 1 << 3;

// -- output compare modes, RM0090 §17.4.7 ------------------------------------

/// Frozen: the comparison has no effect on the output.
const OCM_FROZEN: u32 = 0b000;
/// Set the channel active on match.
const OCM_ACTIVE: u32 = 0b001;
/// Set the channel inactive on match.
const OCM_INACTIVE: u32 = 0b010;
/// Toggle on match.
const OCM_TOGGLE: u32 = 0b011;
/// Force inactive, regardless of the comparison.
const OCM_FORCE_INACTIVE: u32 = 0b100;
/// Force active, regardless of the comparison.
const OCM_FORCE_ACTIVE: u32 = 0b101;
/// PWM mode 1: active while the counter is below the compare value.
const OCM_PWM1: u32 = 0b110;
/// PWM mode 2: the inverse of mode 1.
const OCM_PWM2: u32 = 0b111;

// -- BDTR --------------------------------------------------------------------

/// `BDTR.MOE` — main output enable. Clear it and every output goes idle.
const BDTR_MOE: u32 = 1 << 15;

// -- register offsets --------------------------------------------------------

const OFF_CR1: u64 = 0x00;
const OFF_CR2: u64 = 0x04;
const OFF_SMCR: u64 = 0x08;
const OFF_DIER: u64 = 0x0c;
const OFF_SR: u64 = 0x10;
const OFF_EGR: u64 = 0x14;
const OFF_CCMR1: u64 = 0x18;
const OFF_CCMR2: u64 = 0x1c;
const OFF_CCER: u64 = 0x20;
const OFF_CNT: u64 = 0x24;
const OFF_PSC: u64 = 0x28;
const OFF_ARR: u64 = 0x2c;
const OFF_RCR: u64 = 0x30;
const OFF_CCR1: u64 = 0x34;
const OFF_BDTR: u64 = 0x44;
const OFF_DCR: u64 = 0x48;
const OFF_DMAR: u64 = 0x4c;

// ---------------------------------------------------------------------------
// Variant
// ---------------------------------------------------------------------------

/// Which subset of the block an instance is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// `TIM6`/`TIM7`: a counter and an update interrupt, nothing else.
    Basic,
    /// `TIM2`–`TIM5` and the small general-purpose timers.
    General,
    /// `TIM1`/`TIM8`: `RCR`, `BDTR` and complementary outputs.
    Advanced,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::Basic => "basic",
            Variant::General => "general",
            Variant::Advanced => "advanced",
        }
    }

    /// How many capture/compare channels this variant has by default.
    fn default_channels(self) -> u64 {
        match self {
            Variant::Basic => 0,
            Variant::General | Variant::Advanced => 4,
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// What a machine file fixed about this instance, and cannot change afterwards.
#[derive(Debug, Clone, Copy)]
struct Config {
    variant: Variant,
    /// The counter's width mask: `0xffff` or `0xffff_ffff`.
    mask: u32,
    /// How many capture/compare channels exist.
    channels: usize,
    /// What `ARR` reads out of reset.
    arr_reset: u32,
}

impl Config {
    /// Whether this instance has complementary outputs at all.
    fn advanced(&self) -> bool {
        matches!(self.variant, Variant::Advanced)
    }

    /// How many complementary outputs it bonds.
    fn complementary(&self) -> usize {
        if self.advanced() {
            self.channels.min(COMPLEMENTARY_CHANNELS)
        } else {
            0
        }
    }

    fn cr1_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => CR1_MASK_BASIC,
            _ => CR1_MASK_FULL,
        }
    }

    fn cr2_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => CR2_MASK_BASIC,
            Variant::General => CR2_MASK_GENERAL,
            Variant::Advanced => CR2_MASK_ADVANCED,
        }
    }

    fn dier_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => DIER_MASK_BASIC,
            Variant::General => DIER_MASK_GENERAL,
            Variant::Advanced => DIER_MASK_ADVANCED,
        }
    }

    fn sr_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => SR_MASK_BASIC,
            Variant::General => SR_MASK_GENERAL,
            Variant::Advanced => SR_MASK_ADVANCED,
        }
    }

    /// The `CCER` bits that exist: a nibble per channel, minus `CCxNE` where
    /// there is no complementary output.
    fn ccer_mask(&self) -> u32 {
        let mut mask = 0;
        for i in 0..self.channels {
            let nibble = CCER_CCE | CCER_CCP | CCER_CCNP;
            let extra = if i < self.complementary() {
                CCER_CCNE
            } else {
                0
            };
            mask |= (nibble | extra) << (4 * i);
        }
        mask
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the two shadow sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Regs {
    cr1: u32,
    cr2: u32,
    smcr: u32,
    dier: u32,
    sr: u32,
    ccmr: [u32; 2],
    ccer: u32,
    bdtr: u32,
    dcr: u32,
    dmar: u32,

    /// The counter itself.
    cnt: u32,
    /// How many `CK_INT` ticks have accumulated towards the next counter clock.
    psc_count: u32,
    /// `PSC` as written: the preload register a read returns.
    psc: u32,
    /// `PSC` as the prescaler is actually dividing by, minus one.
    psc_shadow: u32,
    /// `ARR` as written.
    arr: u32,
    /// `ARR` as the counter is actually comparing against.
    arr_shadow: u32,
    /// `RCR` as written.
    rcr: u32,
    /// How many more over/underflows before the next update event.
    rcr_count: u32,
    /// `CCRx` as written.
    ccr: [u32; MAX_CHANNELS],
    /// `CCRx` as the comparator is actually using.
    ccr_shadow: [u32; MAX_CHANNELS],

    /// Which way the counter is going, in center-aligned mode.
    down: bool,
    /// Each channel's `OCxREF`, before polarity and the output enables.
    ///
    /// State rather than derived: in toggle mode it is a latch, and in frozen
    /// mode it holds whatever the last match left.
    ocref: [bool; MAX_CHANNELS],
}

impl Regs {
    /// The state a peripheral reset leaves, for this configuration.
    fn reset(cfg: &Config) -> Regs {
        Regs {
            cr1: 0,
            cr2: 0,
            smcr: 0,
            dier: 0,
            sr: 0,
            ccmr: [0; 2],
            ccer: 0,
            // `MOE` is a bit only an advanced timer has; everything else
            // behaves as though its outputs were permanently enabled.
            bdtr: 0,
            dcr: 0,
            dmar: 0,
            cnt: 0,
            psc_count: 0,
            psc: 0,
            psc_shadow: 0,
            arr: cfg.arr_reset,
            arr_shadow: cfg.arr_reset,
            rcr: 0,
            rcr_count: 0,
            ccr: [0; MAX_CHANNELS],
            ccr_shadow: [0; MAX_CHANNELS],
            down: false,
            ocref: [false; MAX_CHANNELS],
        }
    }

    /// `CR1.CMS`, 0 for edge-aligned.
    fn cms(&self) -> u32 {
        (self.cr1 >> CR1_CMS_SHIFT) & 0x3
    }

    /// Whether the counter is going down right now.
    fn counting_down(&self) -> bool {
        if self.cms() != 0 {
            self.down
        } else {
            self.cr1 & CR1_DIR != 0
        }
    }

    /// Channel `i`'s byte of `CCMR1`/`CCMR2`.
    fn ccmr_byte(&self, i: usize) -> u32 {
        (self.ccmr[i / 2] >> (8 * (i % 2))) & 0xff
    }

    /// Channel `i`'s `OCxM`.
    fn ocm(&self, i: usize) -> u32 {
        (self.ccmr_byte(i) >> CCMR_OCM_SHIFT) & 0x7
    }

    /// Whether channel `i` preloads its compare register.
    fn ocpe(&self, i: usize) -> bool {
        self.ccmr_byte(i) & CCMR_OCPE != 0
    }

    /// Whether channel `i` is configured as an input, in which case this model
    /// leaves its output alone.
    fn is_input(&self, i: usize) -> bool {
        self.ccmr_byte(i) & CCMR_CCS_MASK != 0
    }

    /// Channel `i`'s nibble of `CCER`.
    fn ccer_nibble(&self, i: usize) -> u32 {
        (self.ccer >> (4 * i)) & 0xf
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// The levels every pin of this device should be driving.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Levels {
    irq: bool,
    irq_up: bool,
    irq_cc: bool,
    ch: [bool; MAX_CHANNELS],
    chn: [bool; COMPLEMENTARY_CHANNELS],
}

/// Where each output pin goes, once the machine graph has connected it.
#[derive(Debug, Default, Clone)]
struct Links {
    irq: Option<WireSource>,
    irq_up: Option<WireSource>,
    irq_cc: Option<WireSource>,
    ch: [Option<WireSource>; MAX_CHANNELS],
    chn: [Option<WireSource>; COMPLEMENTARY_CHANNELS],
}

// ---------------------------------------------------------------------------
// Shared
// ---------------------------------------------------------------------------

/// What the register block and the device share.
struct Shared {
    cfg: Config,
    regs: Mutex<Regs>,
    links: Mutex<Links>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The tick reached, republished on every advance.
    ///
    /// The scheduler asks a lazily-advanced device where it is with its slot
    /// held at [`LockRank::LEAF`], so this may not be behind a lock.
    tick: AtomicU64,
    /// The tick of the next observable change, or [`u64::MAX`] for none.
    next_event: AtomicU64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("cfg", &self.cfg)
            .field("tick", &self.tick.load(Ordering::Relaxed));
        match self.regs.try_lock() {
            Some(regs) => s.field("regs", &*regs),
            None => s.field("regs", &"<locked>"),
        };
        s.finish()
    }
}

impl Shared {
    // -- the counting model --------------------------------------------------

    /// Whether the counter is running at all.
    ///
    /// RM0090 §19.4.3 on `TIMx_ARR`: "The counter is blocked while the
    /// auto-reload value is null." That is a real behaviour and it is also what
    /// keeps a freshly reset timer from asking the scheduler for an event every
    /// single tick.
    fn counting(&self, regs: &Regs) -> bool {
        regs.cr1 & CR1_CEN != 0 && regs.arr_shadow != 0
    }

    /// How many counter clocks until the next thing a program can observe.
    ///
    /// Never zero: the caller uses it as a step, and a step of nothing would
    /// stall the device where it stands. Between two of these instants *nothing
    /// changes*, which is what makes the closed-form advance below exact rather
    /// than merely fast.
    fn clocks_to_event(&self, regs: &Regs) -> u64 {
        let mask = u64::from(self.cfg.mask);
        let arr = u64::from(regs.arr_shadow);
        let cnt = u64::from(regs.cnt);
        let down = regs.counting_down();

        let mut best = if regs.cms() != 0 {
            // Center-aligned: up to `ARR`, then down to 0, an event at each end.
            if regs.down {
                if cnt > 0 { cnt } else { 1 }
            } else if cnt < arr {
                arr - cnt
            } else {
                1
            }
        } else if down {
            // The underflow happens on the clock after the counter reads zero.
            cnt + 1
        } else {
            // The overflow happens on the clock after the counter reads `ARR`.
            // A counter left above `ARR` — the classic "I lowered ARR under a
            // running counter" case — never matches the comparator, so it runs
            // up to the width's own wrap first and no update is generated
            // there.
            let to_uev = if cnt <= arr { arr - cnt + 1 } else { u64::MAX };
            to_uev.min(mask - cnt + 1)
        };

        for i in 0..self.cfg.channels {
            let ccr = u64::from(regs.ccr_shadow[i]);
            let distance = if down {
                if ccr < cnt { cnt - ccr } else { continue }
            } else if ccr > cnt {
                ccr - cnt
            } else {
                continue;
            };
            best = best.min(distance);
        }
        best.max(1)
    }

    /// An over/underflow has happened. Decide whether it is an update event.
    fn on_wrap(&self, regs: &mut Regs) {
        // RM0090 §18.3.1: the repetition counter divides the rate at which the
        // wrap becomes an update event. A timer without `RCR` keeps it at zero,
        // so every wrap is one.
        if regs.rcr_count > 0 {
            regs.rcr_count -= 1;
            return;
        }
        regs.rcr_count = regs.rcr;

        // `CR1.OPM`: "Counter stops counting at the next update event (clearing
        // the CEN bit)". The counter stops whether or not `UDIS` suppressed the
        // register update — what stops is the counter, not the bookkeeping.
        if regs.cr1 & CR1_OPM != 0 {
            regs.cr1 &= !CR1_CEN;
        }

        // `CR1.UDIS`: "The Update event is not generated, shadow registers keep
        // their value". The counter still wrapped; nothing else happened.
        if regs.cr1 & CR1_UDIS != 0 {
            return;
        }
        self.reload_shadows(regs);
        regs.sr |= SR_UIF;
    }

    /// Load every preload register into its shadow. This is what an update
    /// event *is*.
    fn reload_shadows(&self, regs: &mut Regs) {
        regs.psc_shadow = regs.psc;
        regs.arr_shadow = regs.arr;
        for i in 0..MAX_CHANNELS {
            regs.ccr_shadow[i] = regs.ccr[i];
        }
    }

    /// Move the counter `clocks` counter-clocks forward, which the caller has
    /// already bounded by [`Shared::clocks_to_event`] so that at most one
    /// boundary is crossed.
    fn step(&self, regs: &mut Regs, clocks: u64) {
        let mask = u64::from(self.cfg.mask);
        let arr = u64::from(regs.arr_shadow);
        let cnt = u64::from(regs.cnt);

        if regs.cms() != 0 {
            if regs.down {
                let next = cnt.saturating_sub(clocks);
                regs.cnt = next as u32;
                if next == 0 {
                    regs.down = false;
                    self.on_wrap(regs);
                }
            } else {
                let next = cnt + clocks;
                if next >= arr {
                    // The peak is `ARR` itself: RM0090 §17.3.3 has the counter
                    // count "from 0 to ARR-1", generate the overflow, then come
                    // back down from `ARR`.
                    regs.cnt = arr as u32;
                    regs.down = true;
                    self.on_wrap(regs);
                } else {
                    regs.cnt = next as u32;
                }
            }
        } else if regs.cr1 & CR1_DIR != 0 {
            if clocks > cnt {
                self.on_wrap(regs);
                // The reload takes the shadow as it stands *after* the update,
                // which is the whole point of buffering `ARR`.
                regs.cnt = regs.arr_shadow;
            } else {
                regs.cnt = (cnt - clocks) as u32;
            }
        } else {
            let next = cnt + clocks;
            if cnt <= arr && next > arr {
                regs.cnt = 0;
                self.on_wrap(regs);
            } else {
                regs.cnt = (next & mask) as u32;
            }
        }
    }

    /// Set the capture/compare flags for any channel the counter has just
    /// landed on, and apply the match-driven output modes.
    fn settle_matches(&self, regs: &mut Regs) {
        // RM0090 §17.4.1 on `CR1.CMS`: in center-aligned mode 1 the flags are
        // set only while counting down, in mode 2 only while counting up, and
        // in mode 3 on both. Edge-aligned mode always sets them.
        let flags_allowed = match regs.cms() {
            0 | 3 => true,
            1 => regs.down,
            _ => !regs.down,
        };
        for i in 0..self.cfg.channels {
            if regs.cnt != regs.ccr_shadow[i] {
                continue;
            }
            if flags_allowed {
                regs.sr |= SR_CC1IF << i;
            }
            // The comparator drives the output whether or not the flag was
            // allowed: the gate above is on the interrupt flag, not on the
            // hardware.
            if regs.is_input(i) {
                continue;
            }
            match regs.ocm(i) {
                OCM_ACTIVE => regs.ocref[i] = true,
                OCM_INACTIVE => regs.ocref[i] = false,
                OCM_TOGGLE => regs.ocref[i] = !regs.ocref[i],
                _ => {}
            }
        }
    }

    /// Recompute the channels whose reference output is a function of the
    /// counter rather than a latch.
    fn settle_outputs(&self, regs: &mut Regs) {
        let down = regs.counting_down();
        for i in 0..self.cfg.channels {
            if regs.is_input(i) {
                continue;
            }
            let ccr = regs.ccr_shadow[i];
            let pwm1 = if down {
                // "In downcounting mode, OCxREF is low as long as
                // CNT > CCRx, else it becomes high" — RM0090 §17.4.7.
                regs.cnt <= ccr
            } else {
                regs.cnt < ccr
            };
            regs.ocref[i] = match regs.ocm(i) {
                OCM_FORCE_INACTIVE => false,
                OCM_FORCE_ACTIVE => true,
                OCM_PWM1 => pwm1,
                OCM_PWM2 => !pwm1,
                // Frozen and the three match-driven modes hold their latch:
                // the first by definition, the others until the next match.
                OCM_FROZEN | OCM_ACTIVE | OCM_INACTIVE | OCM_TOGGLE => regs.ocref[i],
                _ => regs.ocref[i],
            };
        }
    }

    /// Advance to `target`, in ticks of this device's own clock domain.
    ///
    /// Every step is integer arithmetic on the prescaler's phase, and each
    /// iteration either lands exactly on the next observable instant or
    /// consumes the whole remaining span without reaching one. Either way the
    /// counter ends where the hardware's would.
    fn advance_locked(&self, regs: &mut Regs, target: u64) {
        let mut now = self.tick.load(Ordering::Relaxed);
        while now < target {
            if !self.counting(regs) {
                now = target;
                break;
            }
            let per = u64::from(regs.psc_shadow) + 1;
            let remaining = target - now;
            let available = (u64::from(regs.psc_count) + remaining) / per;
            if available == 0 {
                // Not even one counter clock fits: the prescaler keeps the
                // phase, which is why it is in the snapshot.
                regs.psc_count += remaining as u32;
                now = target;
                break;
            }
            let step = available.min(self.clocks_to_event(regs));
            // The first counter clock costs what is left of the current
            // division; each one after it costs a whole division.
            let used = step * per - u64::from(regs.psc_count);
            now += used;
            regs.psc_count = 0;
            self.step(regs, step);
            self.settle_matches(regs);
            self.settle_outputs(regs);
        }
        self.tick.store(now, Ordering::Relaxed);
        self.publish(regs, now);
    }

    /// The tick of the next observable change, or [`u64::MAX`] for none.
    fn compute_next_event(&self, regs: &Regs, now: u64) -> u64 {
        if !self.counting(regs) {
            return u64::MAX;
        }
        let per = u64::from(regs.psc_shadow) + 1;
        let clocks = self.clocks_to_event(regs);
        // `clocks >= 1` and `psc_count < per`, so this is at least one tick in
        // the future — which is what the scheduler requires of it.
        now.saturating_add(clocks * per - u64::from(regs.psc_count))
    }

    fn publish(&self, regs: &Regs, now: u64) {
        self.next_event
            .store(self.compute_next_event(regs, now), Ordering::Relaxed);
    }

    // -- outputs -------------------------------------------------------------

    /// Whether the main output enable lets anything out.
    ///
    /// Only an advanced timer has `BDTR`; on the others the outputs are always
    /// enabled, so there is nothing to gate with.
    fn moe(&self, regs: &Regs) -> bool {
        !self.cfg.advanced() || regs.bdtr & BDTR_MOE != 0
    }

    /// What channel `i` idles at when `MOE` is clear: `CR2.OISx`.
    fn idle_level(&self, regs: &Regs, i: usize) -> bool {
        if !self.cfg.advanced() {
            return false;
        }
        // Bit 8 is `OIS1`, 10 `OIS2`, 12 `OIS3`, 14 `OIS4`.
        regs.cr2 & (1 << (CR2_OIS_SHIFT + 2 * i as u32)) != 0
    }

    /// What complementary output `i` idles at: `CR2.OISxN`, one bit above its
    /// channel's.
    fn idle_level_n(&self, regs: &Regs, i: usize) -> bool {
        regs.cr2 & (1 << (CR2_OIS_SHIFT + 2 * i as u32 + 1)) != 0
    }

    /// Every pin's level, as a pure function of the state.
    fn levels(&self, regs: &Regs) -> Levels {
        let mut out = Levels::default();
        let uif = regs.sr & SR_UIF != 0 && regs.dier & DIER_UIE != 0;
        let mut ccif = false;
        for i in 0..self.cfg.channels {
            if regs.sr & (SR_CC1IF << i) != 0 && regs.dier & (DIER_CC1IE << i) != 0 {
                ccif = true;
            }
        }
        out.irq_up = uif;
        out.irq_cc = ccif;
        out.irq = uif || ccif;

        let moe = self.moe(regs);
        for i in 0..self.cfg.channels {
            let nibble = regs.ccer_nibble(i);
            out.ch[i] = if !moe {
                self.idle_level(regs, i)
            } else if nibble & CCER_CCE == 0 {
                // The pin is released to whatever else drives it; a wire has no
                // way to say that, so it reads inactive.
                false
            } else {
                regs.ocref[i] != (nibble & CCER_CCP != 0)
            };
            if i < self.cfg.complementary() {
                out.chn[i] = if !moe {
                    self.idle_level_n(regs, i)
                } else if nibble & CCER_CCNE == 0 {
                    false
                } else {
                    // The complement of `OCxREF`, then its own polarity —
                    // RM0090 §18.3.9, Table 61, with a dead time of zero.
                    !regs.ocref[i] != (nibble & CCER_CCNP != 0)
                };
            }
        }
        out
    }

    /// Drive every pin, with **no lock of this device held** — the re-entrancy
    /// contract (`ROADMAP.md` §4.4): a sink may call straight back in.
    fn drive(&self) {
        let levels = {
            let regs = self.regs.lock();
            self.levels(&regs)
        };
        let links = self.links.lock().clone();
        if let Some(src) = &links.irq {
            src.set(Level::from_bool(levels.irq));
        }
        if let Some(src) = &links.irq_up {
            src.set(Level::from_bool(levels.irq_up));
        }
        if let Some(src) = &links.irq_cc {
            src.set(Level::from_bool(levels.irq_cc));
        }
        for i in 0..MAX_CHANNELS {
            if let Some(src) = &links.ch[i] {
                src.set(Level::from_bool(levels.ch[i]));
            }
        }
        for i in 0..COMPLEMENTARY_CHANNELS {
            if let Some(src) = &links.chn[i] {
                src.set(Level::from_bool(levels.chn[i]));
            }
        }
    }

    /// Catch up before answering an access.
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
        let _ = handle.sync(kind);
    }

    // -- registers -----------------------------------------------------------

    /// `EGR.UG`, or a reset arriving from the slave-mode controller if one
    /// existed.
    fn software_update(&self, regs: &mut Regs) {
        // "the shadow registers keep their value" with `UDIS` set, "however the
        // counter and the prescaler are reinitialized if the UG bit is set".
        let suppressed = regs.cr1 & CR1_UDIS != 0;
        if !suppressed {
            self.reload_shadows(regs);
            regs.rcr_count = regs.rcr;
            // `URS` selects *what may set UIF*, not what may reload: with it
            // set, only a real over/underflow raises the interrupt.
            if regs.cr1 & CR1_URS == 0 {
                regs.sr |= SR_UIF;
            }
        }
        // "The counter is cleared if the center-aligned mode is selected or if
        // DIR=0 (upcounting), else it takes the auto-reload value."
        regs.cnt = if regs.cms() == 0 && regs.cr1 & CR1_DIR != 0 {
            regs.arr_shadow
        } else {
            0
        };
        regs.psc_count = 0;
        if regs.cms() != 0 {
            regs.down = false;
        }
        // `CR1.OPM` is deliberately not honoured here. A software update is how
        // firmware *loads* a one-pulse timer before starting it, and stopping
        // the counter on that load would stop the pulse before it began.
        self.settle_outputs(regs);
    }

    /// Read one 32-bit register. No side effects at all, so a debug read needs
    /// no special case beyond not advancing the clock.
    fn read_register(&self, regs: &Regs, offset: u64) -> u32 {
        let basic = matches!(self.cfg.variant, Variant::Basic);
        match offset {
            OFF_CR1 => regs.cr1,
            OFF_CR2 => regs.cr2,
            OFF_SMCR if !basic => regs.smcr,
            OFF_DIER => regs.dier,
            OFF_SR => regs.sr,
            // "These bits are always read as 0" — `EGR` is write-only.
            OFF_EGR => 0,
            OFF_CCMR1 if self.cfg.channels > 0 => regs.ccmr[0],
            OFF_CCMR2 if self.cfg.channels > 2 => regs.ccmr[1],
            OFF_CCER if self.cfg.channels > 0 => regs.ccer,
            OFF_CNT => regs.cnt,
            OFF_PSC => regs.psc,
            OFF_ARR => regs.arr,
            OFF_RCR if self.cfg.advanced() => regs.rcr,
            OFF_BDTR if self.cfg.advanced() => regs.bdtr,
            OFF_DCR if !basic => regs.dcr,
            OFF_DMAR if !basic => regs.dmar,
            _ => {
                // The compare registers, and everything this variant does not
                // implement, which reads as zero rather than faulting: the
                // aperture is the peripheral's whole kilobyte on the real part.
                if let Some(i) = channel_of(offset)
                    && i < self.cfg.channels
                {
                    return regs.ccr[i];
                }
                0
            }
        }
    }

    /// Write one 32-bit register.
    fn write_register(&self, regs: &mut Regs, offset: u64, value: u32) {
        let cfg = self.cfg;
        let basic = matches!(cfg.variant, Variant::Basic);
        match offset {
            OFF_CR1 => {
                let was_arpe = regs.cr1 & CR1_ARPE != 0;
                regs.cr1 = value & cfg.cr1_mask();
                if was_arpe && regs.cr1 & CR1_ARPE == 0 {
                    // Preload switched off: "the new value is taken into
                    // account immediately".
                    regs.arr_shadow = regs.arr;
                }
                self.settle_outputs(regs);
            }
            OFF_CR2 => regs.cr2 = value & cfg.cr2_mask(),
            OFF_SMCR if !basic => regs.smcr = value,
            OFF_DIER => regs.dier = value & cfg.dier_mask(),
            // Every flag is `rc_w0`: a zero clears it and a one leaves it be.
            OFF_SR => regs.sr &= value | !cfg.sr_mask(),
            OFF_EGR => {
                if value & EGR_UG != 0 {
                    self.software_update(regs);
                }
                for i in 0..cfg.channels {
                    if value & (EGR_CC1G << i) != 0 {
                        regs.sr |= SR_CC1IF << i;
                    }
                }
            }
            OFF_CCMR1 | OFF_CCMR2 if cfg.channels > 0 => {
                let which = usize::from(offset == OFF_CCMR2);
                if which == 1 && cfg.channels <= 2 {
                    return;
                }
                regs.ccmr[which] = value;
                // Same rule as `ARPE`: dropping `OCxPE` pushes the written
                // compare value through at once.
                for i in 0..cfg.channels {
                    if !regs.ocpe(i) {
                        regs.ccr_shadow[i] = regs.ccr[i];
                    }
                }
                self.settle_outputs(regs);
            }
            OFF_CCER if cfg.channels > 0 => {
                regs.ccer = value & cfg.ccer_mask();
            }
            OFF_CNT => {
                regs.cnt = value & cfg.mask;
                self.settle_outputs(regs);
            }
            // `PSC` has no preload-enable bit: it is always buffered, and the
            // shadow follows at the next update event.
            OFF_PSC => regs.psc = value & 0xffff,
            OFF_ARR => {
                regs.arr = value & cfg.mask;
                if regs.cr1 & CR1_ARPE == 0 {
                    regs.arr_shadow = regs.arr;
                }
                self.settle_outputs(regs);
            }
            OFF_RCR if cfg.advanced() => regs.rcr = value & 0xff,
            OFF_BDTR if cfg.advanced() => regs.bdtr = value & 0xffff,
            OFF_DCR if !basic => regs.dcr = value & 0x1f1f,
            OFF_DMAR if !basic => regs.dmar = value,
            _ => {
                if let Some(i) = channel_of(offset)
                    && i < cfg.channels
                {
                    regs.ccr[i] = value & cfg.mask;
                    if !regs.ocpe(i) {
                        regs.ccr_shadow[i] = regs.ccr[i];
                    }
                    self.settle_outputs(regs);
                }
            }
        }
    }
}

/// Which capture/compare channel `offset` names, if any.
fn channel_of(offset: u64) -> Option<usize> {
    if (OFF_CCR1..OFF_CCR1 + 4 * MAX_CHANNELS as u64).contains(&offset) {
        Some(((offset - OFF_CCR1) / 4) as usize)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 TIM.
pub struct Tim {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl fmt::Debug for Tim {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Tim")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl Tim {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, if one
    /// this class does not know was given, or if the combination names no real
    /// part — a basic timer with capture/compare channels, for instance.
    pub fn new(props: &Props) -> Result<Tim> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "general", &["basic", "general", "advanced"])? {
            "basic" => Variant::Basic,
            "advanced" => Variant::Advanced,
            _ => Variant::General,
        };
        let width = r.or::<u64>("width", 16)?;
        let channels = r.or::<u64>("channels", variant.default_channels())?;
        let arr_reset = r.or::<u64>("arr-reset", 0)?;
        r.finish()?;

        let mask: u32 = match width {
            16 => 0xffff,
            32 => 0xffff_ffff,
            _ => {
                return Err(Error::Property(format!(
                    "`width`: a TIM counter is 16 or 32 bits wide, not {width}"
                )));
            }
        };
        if channels > MAX_CHANNELS as u64 {
            return Err(Error::Property(format!(
                "`channels`: a TIM has at most {MAX_CHANNELS} channels, not {channels}"
            )));
        }
        if matches!(variant, Variant::Basic) && channels != 0 {
            return Err(Error::Property(String::from(
                "`channels`: a basic timer (TIM6/TIM7) has no capture/compare channels; \
                 a part that has them is `variant = \"general\"` or `\"advanced\"`",
            )));
        }
        if arr_reset > u64::from(mask) {
            return Err(Error::Property(format!(
                "`arr-reset`: {arr_reset:#x} does not fit a {width}-bit auto-reload"
            )));
        }

        Ok(Tim::with_config(Config {
            variant,
            mask,
            channels: channels as usize,
            arr_reset: arr_reset as u32,
        }))
    }

    /// Build one from a configuration the caller already has — the route a test
    /// takes.
    fn with_config(cfg: Config) -> Tim {
        let regs = Regs::reset(&cfg);
        let shared = Arc::new(Shared {
            cfg,
            regs: Mutex::with_rank(LockRank::DEVICE, regs),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        {
            let regs = shared.regs.lock();
            shared.publish(&regs, 0);
        }
        let region = Arc::new(Region::io(
            "st.tim.regs",
            REGISTER_BYTES,
            Arc::clone(&shared) as Arc<dyn MemOps>,
        ));
        Tim { shared, region }
    }

    /// Which subset of the block this instance is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.cfg.variant
    }

    /// How many capture/compare channels it has.
    #[must_use]
    pub fn channels(&self) -> usize {
        self.shared.cfg.channels
    }

    /// The counter, as the guest would read `CNT`.
    #[must_use]
    pub fn counter(&self) -> u32 {
        self.shared.regs.lock().cnt
    }

    /// The tick this device's own next observable change falls on, if it has
    /// one.
    #[must_use]
    pub fn next_event(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            tick => Some(tick),
        }
    }

    /// Advance to `target` ticks of this device's clock domain.
    pub fn advance_to(&self, target: u64) {
        {
            let mut regs = self.shared.regs.lock();
            self.shared.advance_locked(&mut regs, target);
        }
        // The re-entrancy contract: the lock is released before anything
        // outward happens (`ROADMAP.md` §4.4).
        self.shared.drive();
    }

    /// Advance by `ticks` more.
    pub fn advance_by(&self, ticks: u64) {
        self.advance_to(self.shared.tick.load(Ordering::Relaxed) + ticks);
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Device for Tim {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and every
        // output idles low, which is where a fresh net already is.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut regs = self.shared.regs.lock();
            *regs = Regs::reset(&self.shared.cfg);
            // The tick is the clock domain's position, not this device's state,
            // and `Machine::reset` does not rewind domains — rewinding it here
            // would ask the next catch-up to replay every cycle since power-on.
            let now = self.shared.tick.load(Ordering::Relaxed);
            self.shared.publish(&regs, now);
        }
        self.shared.drive();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        for value in [
            regs.cr1,
            regs.cr2,
            regs.smcr,
            regs.dier,
            regs.sr,
            regs.ccmr[0],
            regs.ccmr[1],
            regs.ccer,
            regs.bdtr,
            regs.dcr,
            regs.dmar,
            regs.cnt,
            regs.psc_count,
            regs.psc,
            regs.psc_shadow,
            regs.arr,
            regs.arr_shadow,
            regs.rcr,
            regs.rcr_count,
        ] {
            w.write_u32(value)?;
        }
        for i in 0..MAX_CHANNELS {
            w.write_u32(regs.ccr[i])?;
            w.write_u32(regs.ccr_shadow[i])?;
            w.write_bool(regs.ocref[i])?;
        }
        w.write_bool(regs.down)?;
        // The domain position. The next-event tick is derived from the rest and
        // is recomputed by `load`, so it is not written (`CLAUDE.md`: derived
        // state is never serialized).
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = Regs::reset(&self.shared.cfg);
        regs.cr1 = r.read_u32()?;
        regs.cr2 = r.read_u32()?;
        regs.smcr = r.read_u32()?;
        regs.dier = r.read_u32()?;
        regs.sr = r.read_u32()?;
        regs.ccmr[0] = r.read_u32()?;
        regs.ccmr[1] = r.read_u32()?;
        regs.ccer = r.read_u32()?;
        regs.bdtr = r.read_u32()?;
        regs.dcr = r.read_u32()?;
        regs.dmar = r.read_u32()?;
        regs.cnt = r.read_u32()?;
        regs.psc_count = r.read_u32()?;
        regs.psc = r.read_u32()?;
        regs.psc_shadow = r.read_u32()?;
        regs.arr = r.read_u32()?;
        regs.arr_shadow = r.read_u32()?;
        regs.rcr = r.read_u32()?;
        regs.rcr_count = r.read_u32()?;
        for i in 0..MAX_CHANNELS {
            regs.ccr[i] = r.read_u32()?;
            regs.ccr_shadow[i] = r.read_u32()?;
            regs.ocref[i] = r.read_bool()?;
        }
        regs.down = r.read_bool()?;
        let tick = r.read_u64()?;
        {
            let mut slot = self.shared.regs.lock();
            *slot = regs;
            self.shared.tick.store(tick, Ordering::Relaxed);
            self.shared.publish(&regs, tick);
        }
        self.shared.drive();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let cfg = self.shared.cfg;
        {
            let mut links = self.shared.links.lock();
            match port {
                IRQ_PIN => links.irq = Some(source),
                IRQ_UP_PIN | IRQ_CC_PIN if !cfg.advanced() => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!(
                            "only an advanced timer splits its interrupt across vectors; \
                             this one drives `{IRQ_PIN}`"
                        ),
                    });
                }
                IRQ_UP_PIN => links.irq_up = Some(source),
                IRQ_CC_PIN => links.irq_cc = Some(source),
                _ => match channel_pin(port) {
                    Some((i, false)) if i < cfg.channels => links.ch[i] = Some(source),
                    Some((i, true)) if i < cfg.complementary() => links.chn[i] = Some(source),
                    _ => {
                        return Err(Error::Config {
                            at: port.to_string(),
                            message: format!(
                                "this timer drives `{IRQ_PIN}` and `ch1`…`ch{}`",
                                cfg.channels
                            ),
                        });
                    }
                },
            }
        }
        self.shared.drive();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Tim::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Tim::next_event(self)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Tim::attach_lazy(self, handle);
    }
}

impl MemOps for Shared {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.sync(attrs);
        let value = {
            let regs = self.regs.lock();
            self.read_register(&regs, offset & !3)
        };
        match dst.len() {
            2 => {
                let half = if offset & 2 != 0 { value >> 16 } else { value };
                dst.copy_from_slice(&(half as u16).to_le_bytes());
                Ok(())
            }
            4 => {
                dst.copy_from_slice(&value.to_le_bytes());
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let reg = offset & !3;
        {
            let mut regs = self.regs.lock();
            let value = match src.len() {
                2 => {
                    let current = self.read_register(&regs, reg);
                    let half = u32::from(u16::from_le_bytes([src[0], src[1]]));
                    if offset & 2 != 0 {
                        (current & 0x0000_ffff) | (half << 16)
                    } else {
                        (current & 0xffff_0000) | half
                    }
                }
                4 => u32::from_le_bytes([src[0], src[1], src[2], src[3]]),
                _ => return Err(BusError::BadAccess),
            };
            self.write_register(&mut regs, reg, value);
            let now = self.tick.load(Ordering::Relaxed);
            self.publish(&regs, now);
        }
        self.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U16, Width::U32)
    }
}

/// Which channel output pin `port` names: its index, and whether it is the
/// complementary one.
fn channel_pin(port: &str) -> Option<(usize, bool)> {
    let rest = port.strip_prefix("ch")?;
    let (digits, complementary) = match rest.strip_suffix('n') {
        Some(digits) => (digits, true),
        None => (rest, false),
    };
    // `ch1`…`ch4`, one-based as the reference manual numbers them, with no
    // second spelling: `ch01` is a typo, not another name for the same net.
    let index: usize = match digits {
        "1" => 0,
        "2" => 1,
        "3" => 2,
        "4" => 3,
        _ => return None,
    };
    Some((index, complementary))
}

impl Instance for Tim {}

/// The `st.tim` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 TIM: counter, prescaler, shadowed auto-reload, update event and output compare",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which subset: \"basic\", \"general\" (default) or \"advanced\"",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "the counter's width in bits: 16 (default) or 32",
        },
        PropertySpec {
            name: "channels",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many capture/compare channels, 0-4 (default 4, or 0 for \"basic\")",
        },
        PropertySpec {
            name: "arr-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "what ARR reads out of reset (default 0, which is RM0090's value)",
        },
    ],
    construct: |props| Ok(Box::new(Tim::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Tim::new(props)?)))
}

/// What the validator should know about `st.tim`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["basic", "general", "advanced"]))
        .prop(PropSchema::new("width", ValueKind::Uint).range(16, 32))
        .prop(PropSchema::new("channels", ValueKind::Uint).range(0, MAX_CHANNELS as u64))
        .prop(PropSchema::new("arr-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        // An M-profile board wires these straight to the core, with the vector
        // number coming from the part's table and living in the machine file:
        // `wire tim2.irq -> cpu.irq28`.
        .port(IRQ_PIN, PortDir::Out)
        .port(IRQ_UP_PIN, PortDir::Out)
        .port(IRQ_CC_PIN, PortDir::Out)
        .port("ch1", PortDir::Out)
        .port("ch2", PortDir::Out)
        .port("ch3", PortDir::Out)
        .port("ch4", PortDir::Out)
        .port("ch1n", PortDir::Out)
        .port("ch2n", PortDir::Out)
        .port("ch3n", PortDir::Out)
}

#[cfg(test)]
mod tests;
