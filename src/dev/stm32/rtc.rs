//! The STM32 real-time clock.
//!
//! One class, `st.rtc`. It is the second thing a vendor startup sequence
//! configures after the RCC, it is the only peripheral on the die that keeps
//! its state across a system reset, and its thirty-two backup registers are
//! where firmware parks a reset reason or a boot flag on the way through a
//! reboot. A board without one boots *almost* right, which is the expensive
//! kind of wrong.
//!
//! # The registers
//!
//! The map is RM0351 §37.6 (L4); RM0090 §26.6 gives the same block on an F4
//! with `CALIBR` at `+0x18` where a later part has nothing and `TAFCR` at
//! `+0x40` where a later part has `TAMPCR`. This model implements the union:
//! both offsets exist, and the one the part in hand does not have reads back
//! what was written to it.
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `TR` | the time, in **BCD**: `SU ST MNU MNT HU HT PM` |
//! | `0x04` | `DR` | the date, in BCD: `DU DT MU MT WDU YU YT` |
//! | `0x08` | `CR` | the whole configuration: `WUCKSEL`, `FMT`, `BYPSHAD`, the alarm/wakeup/timestamp enables and their interrupt enables |
//! | `0x0c` | `ISR` | the flags, plus `INIT`/`INITF` — the only register whose low half is write-protected and whose high half is not |
//! | `0x10` | `PRER` | `PREDIV_A[22:16]` and `PREDIV_S[14:0]`, the two halves of the divider chain |
//! | `0x14` | `WUTR` | the wakeup down-counter's reload |
//! | `0x1c` | `ALRMAR` | alarm A: the calendar fields plus `MSK1`…`MSK4` and `WDSEL` |
//! | `0x20` | `ALRMBR` | alarm B, identically |
//! | `0x24` | `WPR` | the key register: `0xCA` then `0x53` unlocks, anything else locks |
//! | `0x28` | `SSR` | the synchronous prescaler's down-counter — the sub-second |
//! | `0x2c` | `SHIFTR` | shift the clock by a fraction of a second, or by a whole one |
//! | `0x30`…`0x38` | `TSTR`/`TSDR`/`TSSSR` | what the calendar read at the timestamp edge |
//! | `0x3c` | `CALR` | smooth calibration |
//! | `0x40` | `TAMPCR` | tamper configuration |
//! | `0x44`/`0x48` | `ALRMASSR`/`ALRMBSSR` | the alarms' sub-second field and its mask |
//! | `0x4c` | `OR` | the part's option bits |
//! | `0x50`…`0xcc` | `BKP0R`…`BKP31R` | thirty-two words of battery-backed storage |
//!
//! # The divider chain, and why the calendar is exact
//!
//! RTCCLK feeds a seven-bit **asynchronous** prescaler and then a fifteen-bit
//! **synchronous** one, so one calendar second is
//! `(PREDIV_A + 1) × (PREDIV_S + 1)` RTCCLK cycles — 128 × 256 = 32768 at the
//! reset values, which is exactly one second of a 32.768 kHz LSE
//! (RM0351 §37.3.1). `SSR` is the synchronous counter itself, counting *down*
//! from `PREDIV_S`.
//!
//! One tick of this device's clock domain is one RTCCLK cycle, and a machine
//! file hands it the oscillator:
//!
//! ```text
//! osc lse = 32768 Hz
//! object rtc "st.rtc" { clock = lse, epoch = "2024-01-01T00:00:00" }
//! ```
//!
//! The device is [lazily advanced](crate::core::sched): it publishes the tick
//! of its next interesting instant and the scheduler comes back at it. **It
//! never reads the host clock and never sleeps.** That is not a stylistic
//! preference — a calendar seeded from `now()` makes every run of the machine
//! a different run, and record/replay, snapshot round-trips and the state hash
//! all stop meaning anything.
//!
//! # Where the time starts
//!
//! `epoch` is the calendar the backup domain comes up with, written
//! `"YYYY-MM-DD"` or `"YYYY-MM-DDTHH:MM:SS"` (a space in place of the `T` is
//! accepted). The year must be 2000–2099, because that is the range `YT`/`YU`
//! can hold. The day of the week is **computed from the date**, so an `epoch`
//! is a real day rather than an arbitrary `WDU`.
//!
//! With no `epoch`, the calendar comes up at the manual's own reset value:
//! `TR = 0x0000_0000` and `DR = 0x0000_2101`, which is 1 January 2000 with
//! `WDU = 1`. Note that ST's reset value calls that day a **Monday** and the
//! real 1 January 2000 was a Saturday; the reset value is what the silicon
//! does and an explicit `epoch` is what a calendar does, so the two differ by
//! design and only for that one date.
//!
//! # The write-protection dance
//!
//! Almost everything is write-protected and the protection is the part
//! firmware actually trips on, so it is modelled rather than waved through
//! (RM0351 §37.3.6). `WPR` takes `0xCA` and then `0x53`; any other value
//! re-arms the lock, and so does a second `0xCA`, because the key register is
//! a two-step state machine rather than a latch. `ISR[15:8]` — the flags —
//! `TAMPCR`, `BKPxR`, `OR` and `WPR` itself stay writable throughout; the
//! manual names `ISR[13:8]` and the two tamper flags added later sit in the
//! same window.
//!
//! Then `INIT`: setting it stops the calendar, and `INITF` follows two RTCCLK
//! cycles later. `TR`, `DR` and `PRER` are writable **only** while `INITF` is
//! set, which is why a driver that skips the wait silently keeps the old time.
//! Clearing `INIT` reloads the prescalers and restarts the counter.
//!
//! # Shadow registers
//!
//! With `BYPSHAD` clear, `TR`, `DR` and `SSR` are read out of a shadow copy
//! the hardware refreshes every two RTCCLK cycles, and `RSF` says the copy is
//! valid. Software clears `RSF` and waits for it to come back — that is
//! `HAL_RTC_WaitForSynchro` — so the flag has to actually take time, and here
//! it takes the manual's two cycles.
//!
//! Reading `TR` or `SSR` **locks the shadow until `DR` is read**
//! (RM0351 §37.3.7). Without that rule a calendar read that straddles
//! midnight returns yesterday's date with today's time, and every driver in
//! the field depends on the lock rather than on getting lucky.
//!
//! With `BYPSHAD` set the reads go straight to the live counters and `RSF`
//! stays clear, which is what the manual says happens.
//!
//! # What is wired, not written
//!
//! - `rtcen` — RCC's `BDCR.RTCEN`. Low stops the counter. With nothing driving
//!   it the counter runs, the same accommodation `st.rcc` makes for
//!   an unwired `dbp`: a board that does not model the gate is not a board
//!   whose clock is off.
//! - `bdrst` — RCC's `BDCR.BDRST`. A rising edge is a **backup-domain reset**:
//!   the calendar goes back to `epoch` and every one of the thirty-two backup
//!   registers is cleared. A *system* reset ([`ResetKind::Warm`]) changes
//!   nothing at all, which is the whole reason firmware trusts `BKPxR`.
//! - `ts` — the timestamp input. With `TSE` set, the edge `TSEDGE` selects
//!   latches the calendar into `TSTR`/`TSDR`/`TSSSR` and sets `TSF`.
//! - `alarm`, `wakeup`, `stamp` — the three interrupt outputs, each held high
//!   while its flag and matching interrupt enable are both set. They are
//!   levels rather than pulses so that an EXTI line sees one rising edge per
//!   event and sees it go away when software clears the flag. **Which** EXTI
//!   line and which NVIC position is the board's business and lives in the
//!   machine file: 18/19/20 and IRQ 41/3/2 on an L4, 17/21/22 on an F4.
//!
//! # What this model does not do
//!
//! `CALR`'s smooth calibration is stored and `RECALPF` behaves, but the
//! correction is **not applied to the rate** — a calibrated RTC here keeps the
//! same time as an uncalibrated one. `TAMPCR` is storage and no tamper
//! detection is modelled, so `TAMP1F`…`TAMP3F` are only ever what software
//! wrote. `REFCKON`'s 50/60 Hz reference-clock detection is stored and
//! ignored. `OSEL`/`POL`/`COE`/`COSEL` — the alarm and calibration waveforms a
//! part drives out of `RTC_AF1` — are stored and there is no output pin for
//! them, because the pad they would reach is a `GPIOC` alternate function and
//! nothing in this tree routes one. And `ALRAWF`/`ALRBWF`/`WUTWF` follow their
//! enable bits immediately rather than after a synchronisation delay: the poll
//! firmware writes around them exits either way, so the delay would cost
//! fidelity nothing and buy a reader a puzzle.
//!
//! Each of those changes long-term accuracy or reacts to a pin no board here
//! has; none of them is in the path of a driver bring-up.
//!
//! # Sources
//!
//! ST **RM0351** rev 9 §37 "Real-time clock (RTC)" for the register map and
//! the initialisation, shadow-register and write-protection sequences, and ST
//! **RM0090** rev 21 §26 for the F4's version of the same block. The calendar
//! arithmetic — leap years, day-of-week — is the proleptic Gregorian calendar
//! and is computed here rather than tabulated. No emulator source of any
//! licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.rtc";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many backup registers the block has (RM0351 §37.6.21).
pub const BKP_COUNT: usize = 32;

/// How many bytes the registers occupy: `OR` ends at `+0x50`, and the backup
/// registers run from there to `+0xd0`.
pub const REGISTER_BYTES: u64 = 0x50 + (BKP_COUNT as u64) * 4;

/// The name of RCC's `BDCR.RTCEN` input.
pub const RTCEN_PIN: &str = "rtcen";
/// The name of RCC's `BDCR.BDRST` input.
pub const BDRST_PIN: &str = "bdrst";
/// The name of the timestamp input.
pub const TS_PIN: &str = "ts";
/// The name of the alarm A/B interrupt output.
pub const ALARM_PIN: &str = "alarm";
/// The name of the wakeup-timer interrupt output.
pub const WAKEUP_PIN: &str = "wakeup";
/// The name of the timestamp/tamper interrupt output.
pub const STAMP_PIN: &str = "stamp";

// --- offsets ---------------------------------------------------------------

/// Time register.
const TR: u64 = 0x00;
/// Date register.
const DR: u64 = 0x04;
/// Control register.
const CR: u64 = 0x08;
/// Initialisation and status register.
const ISR: u64 = 0x0c;
/// Prescaler register.
const PRER: u64 = 0x10;
/// Wakeup timer register.
const WUTR: u64 = 0x14;
/// The F4's coarse-calibration register; absent from later parts.
const CALIBR: u64 = 0x18;
/// Alarm A register.
const ALRMAR: u64 = 0x1c;
/// Alarm B register.
const ALRMBR: u64 = 0x20;
/// Write-protection register.
const WPR: u64 = 0x24;
/// Sub-second register.
const SSR: u64 = 0x28;
/// Shift control register.
const SHIFTR: u64 = 0x2c;
/// Timestamp time register.
const TSTR: u64 = 0x30;
/// Timestamp date register.
const TSDR: u64 = 0x34;
/// Timestamp sub-second register.
const TSSSR: u64 = 0x38;
/// Calibration register.
const CALR: u64 = 0x3c;
/// Tamper configuration register (`TAFCR` on an F4).
const TAMPCR: u64 = 0x40;
/// Alarm A sub-second register.
const ALRMASSR: u64 = 0x44;
/// Alarm B sub-second register.
const ALRMBSSR: u64 = 0x48;
/// Option register.
const OR: u64 = 0x4c;
/// The first backup register.
const BKP0R: u64 = 0x50;

// --- CR bits ---------------------------------------------------------------

/// `CR.WUCKSEL[2:0]`: what clocks the wakeup down-counter.
const CR_WUCKSEL: u32 = 0b111;
/// `CR.TSEDGE`: 0 latches on a rising edge, 1 on a falling one.
const CR_TSEDGE: u32 = 1 << 3;
/// `CR.REFCKON`: reference-clock detection. Stored and not modelled.
const CR_REFCKON: u32 = 1 << 4;
/// `CR.BYPSHAD`: read the live calendar rather than the shadow copy.
const CR_BYPSHAD: u32 = 1 << 5;
/// `CR.FMT`: 0 is 24-hour, 1 is AM/PM.
const CR_FMT: u32 = 1 << 6;
/// `CR.ALRAE`: alarm A enable.
const CR_ALRAE: u32 = 1 << 8;
/// `CR.ALRBE`: alarm B enable.
const CR_ALRBE: u32 = 1 << 9;
/// `CR.WUTE`: wakeup timer enable.
const CR_WUTE: u32 = 1 << 10;
/// `CR.TSE`: timestamp enable.
const CR_TSE: u32 = 1 << 11;
/// `CR.ALRAIE`: alarm A interrupt enable.
const CR_ALRAIE: u32 = 1 << 12;
/// `CR.ALRBIE`: alarm B interrupt enable.
const CR_ALRBIE: u32 = 1 << 13;
/// `CR.WUTIE`: wakeup timer interrupt enable.
const CR_WUTIE: u32 = 1 << 14;
/// `CR.TSIE`: timestamp interrupt enable.
const CR_TSIE: u32 = 1 << 15;
/// `CR.ADD1H`: add one hour. Write-only; never reads back.
const CR_ADD1H: u32 = 1 << 16;
/// `CR.SUB1H`: subtract one hour. Write-only; never reads back.
const CR_SUB1H: u32 = 1 << 17;
/// Everything `CR` keeps: bits 0–23, less the reserved bit 7 and less
/// `ADD1H`/`SUB1H`, which are actions rather than storage and never read back.
const CR_MASK: u32 = 0x00ff_ffff & !(1 << 7) & !(CR_ADD1H | CR_SUB1H);

// --- ISR bits --------------------------------------------------------------

/// `ISR.ALRAWF`: alarm A's registers may be written.
const ISR_ALRAWF: u32 = 1 << 0;
/// `ISR.ALRBWF`: alarm B's registers may be written.
const ISR_ALRBWF: u32 = 1 << 1;
/// `ISR.WUTWF`: the wakeup timer's registers may be written.
const ISR_WUTWF: u32 = 1 << 2;
/// `ISR.SHPF`: a shift operation is still pending.
const ISR_SHPF: u32 = 1 << 3;
/// `ISR.INITS`: the calendar year field has been written since a backup reset.
const ISR_INITS: u32 = 1 << 4;
/// `ISR.RSF`: the shadow registers hold a valid copy.
const ISR_RSF: u32 = 1 << 5;
/// `ISR.INITF`: initialisation mode has taken effect.
const ISR_INITF: u32 = 1 << 6;
/// `ISR.INIT`: ask for initialisation mode.
const ISR_INIT: u32 = 1 << 7;
/// `ISR.ALRAF`: alarm A matched.
const ISR_ALRAF: u32 = 1 << 8;
/// `ISR.ALRBF`: alarm B matched.
const ISR_ALRBF: u32 = 1 << 9;
/// `ISR.WUTF`: the wakeup down-counter reached zero.
const ISR_WUTF: u32 = 1 << 10;
/// `ISR.TSF`: a timestamp was captured.
const ISR_TSF: u32 = 1 << 11;
/// `ISR.TSOVF`: a timestamp arrived on top of one software had not read.
const ISR_TSOVF: u32 = 1 << 12;
/// `ISR.TAMP1F`: tamper 1. Storage here; no tamper detection is modelled.
const ISR_TAMP1F: u32 = 1 << 13;
/// `ISR.RECALPF`: a `CALR` write has not taken effect yet.
const ISR_RECALPF: u32 = 1 << 16;

/// The flags software clears by writing zero, and the only part of `ISR` the
/// key register does not protect (RM0351 §37.3.6 names `ISR[13:8]`; `TAMP2F`
/// and `TAMP3F` were added at 14 and 15 in the same window).
const ISR_RC_W0: u32 = 0xff00;

/// `ISR` out of reset: the three "write flag" bits, because nothing is enabled
/// yet. RM0351 §37.6.4 gives the reset value as `0x0000 0007`.
const ISR_RESET: u32 = ISR_ALRAWF | ISR_ALRBWF | ISR_WUTWF;

// --- other reset values ----------------------------------------------------

/// `PRER` out of reset: `PREDIV_A = 0x7f`, `PREDIV_S = 0xff`.
const PRER_RESET: u32 = 0x007f_00ff;
/// `WUTR` out of reset.
const WUTR_RESET: u32 = 0x0000_ffff;

/// How many RTCCLK cycles `INITF` takes to follow `INIT`.
///
/// "it takes around 2 RTCCLK clock cycles (due to clock synchronization)"
/// (RM0351 §37.3.5). Two is what a `while (!(RTC->ISR & RTC_ISR_INITF))` loop
/// is written against, and a zero-cycle model turns that loop into a no-op
/// that hides a driver bug rather than reproducing it.
const SYNC_CYCLES: u64 = 2;

// --- WPR keys --------------------------------------------------------------

/// The first byte of the unlock sequence.
const WPR_KEY1: u32 = 0xca;
/// The second byte of the unlock sequence.
const WPR_KEY2: u32 = 0x53;

// ---------------------------------------------------------------------------
// The calendar
// ---------------------------------------------------------------------------

/// A date and a time, held in binary and converted to BCD at the register.
///
/// The registers are BCD and the counter is not: an increment that carried in
/// BCD would have to be written twice — once for decode and once for the
/// arithmetic — and the second copy is where the leap-year bug lives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calendar {
    /// Seconds, 0–59.
    pub second: u8,
    /// Minutes, 0–59.
    pub minute: u8,
    /// Hours, 0–23. Always twenty-four-hour here; `CR.FMT` is a register
    /// format, not a different counter.
    pub hour: u8,
    /// Day of the month, 1–31.
    pub day: u8,
    /// Month, 1–12.
    pub month: u8,
    /// Year within the century, 0–99, meaning 2000–2099.
    pub year: u8,
    /// Day of the week, `WDU`: 1 is Monday and 7 is Sunday.
    pub weekday: u8,
}

impl Default for Calendar {
    fn default() -> Calendar {
        // The manual's own reset value, `DR = 0x0000 2101`: 1 January 2000
        // with `WDU = 1`. See the module documentation on why that Monday is
        // not the real one.
        Calendar {
            second: 0,
            minute: 0,
            hour: 0,
            day: 1,
            month: 1,
            year: 0,
            weekday: 1,
        }
    }
}

/// Whether `year` (a full year, not the `YU`/`YT` pair) is a leap year.
///
/// The Gregorian rule in full rather than `year % 4 == 0`. Within 2000–2099
/// the two agree, because 2000 is divisible by 400 — which is exactly the case
/// the short rule is famous for getting wrong, so the full rule is written out
/// and the reader does not have to take the coincidence on trust.
#[must_use]
pub const fn is_leap_year(year: u32) -> bool {
    year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400))
}

/// How many days `month` (1–12) has in `year`.
#[must_use]
pub const fn days_in_month(year: u32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        // Not reachable from a well-formed calendar; 30 keeps the counter
        // moving rather than wedging it if a snapshot ever carried one.
        _ => 30,
    }
}

/// Days from 1 January 2000 to `year`-`month`-`day`, proleptic Gregorian.
fn days_from_2000(year: u32, month: u8, day: u8) -> u32 {
    let mut days = 0u32;
    let mut y = 2000;
    while y < year {
        days += if is_leap_year(y) { 366 } else { 365 };
        y += 1;
    }
    let mut m = 1u8;
    while m < month {
        days += u32::from(days_in_month(year, m));
        m += 1;
    }
    days + u32::from(day) - 1
}

/// Encode `value` (0–99) as two BCD digits.
const fn to_bcd(value: u8) -> u32 {
    ((value as u32 / 10) << 4) | (value as u32 % 10)
}

/// Decode two BCD digits. A nibble above nine is not defined by the manual;
/// it is decoded at face value here and the caller clamps, so a garbage write
/// leaves a well-formed counter rather than one that never carries.
const fn from_bcd(value: u32) -> u8 {
    (((value >> 4) & 0xf) * 10 + (value & 0xf)) as u8
}

impl Calendar {
    /// The weekday `WDU` of a date: 1 is Monday, 7 is Sunday.
    ///
    /// 1 January 2000 was a Saturday, which is `WDU = 6`, and every other day
    /// is counted from it.
    #[must_use]
    pub fn weekday_of(year: u32, month: u8, day: u8) -> u8 {
        ((days_from_2000(year, month, day) + 5) % 7) as u8 + 1
    }

    /// The full year, 2000–2099.
    #[must_use]
    pub const fn full_year(&self) -> u32 {
        2000 + self.year as u32
    }

    /// Advance by one second, carrying all the way through the century.
    fn tick_second(&mut self) {
        self.second += 1;
        if self.second < 60 {
            return;
        }
        self.second = 0;
        self.minute += 1;
        if self.minute < 60 {
            return;
        }
        self.minute = 0;
        self.hour += 1;
        if self.hour < 24 {
            return;
        }
        self.hour = 0;
        self.tick_day();
    }

    /// Advance by one day, carrying the month, the year and the weekday.
    fn tick_day(&mut self) {
        // `WDU` is a counter of its own that the date drives: the hardware has
        // no way to recompute it, so firmware that wrote a wrong one keeps a
        // wrong one, consistently.
        self.weekday = self.weekday % 7 + 1;
        self.day += 1;
        if self.day <= days_in_month(self.full_year(), self.month) {
            return;
        }
        self.day = 1;
        self.month += 1;
        if self.month <= 12 {
            return;
        }
        self.month = 1;
        // `YU`/`YT` is two BCD digits, so 99 wraps to 00 and the century is
        // not modelled — neither is it on the die.
        self.year = (self.year + 1) % 100;
    }

    /// Add `hours` hours, carrying days. Used by `CR.ADD1H`/`SUB1H`.
    fn add_hours(&mut self, hours: i32) {
        let mut h = i32::from(self.hour) + hours;
        while h < 0 {
            h += 24;
            self.sub_day();
        }
        while h >= 24 {
            h -= 24;
            self.tick_day();
        }
        // `h` is in 0..24 by construction.
        self.hour = h as u8;
    }

    /// Step back one day, the inverse of [`Calendar::tick_day`].
    fn sub_day(&mut self) {
        self.weekday = (self.weekday + 5) % 7 + 1;
        if self.day > 1 {
            self.day -= 1;
            return;
        }
        if self.month > 1 {
            self.month -= 1;
        } else {
            self.month = 12;
            self.year = (self.year + 99) % 100;
        }
        self.day = days_in_month(self.full_year(), self.month);
    }

    /// The `TR` encoding, in the format `ampm` selects.
    #[must_use]
    pub fn to_tr(&self, ampm: bool) -> u32 {
        let (hour, pm) = if ampm {
            // Midnight and noon are the two the naive conversion gets wrong:
            // 00:00 is 12 AM and 12:00 is 12 PM.
            match self.hour {
                0 => (12, false),
                1..=11 => (self.hour, false),
                12 => (12, true),
                _ => (self.hour - 12, true),
            }
        } else {
            (self.hour, false)
        };
        to_bcd(self.second)
            | (to_bcd(self.minute) << 8)
            | (to_bcd(hour) << 16)
            | (u32::from(pm) << 22)
    }

    /// The `DR` encoding.
    #[must_use]
    pub fn to_dr(&self) -> u32 {
        to_bcd(self.day)
            | (to_bcd(self.month) << 8)
            | (u32::from(self.weekday) << 13)
            | (to_bcd(self.year) << 16)
    }

    /// Take a `TR` write, in the format `ampm` selects.
    ///
    /// Fields are clamped to their ranges: the manual does not define what an
    /// out-of-range BCD field counts like, and a clamped counter still carries
    /// correctly where a `0x9A` seconds field would never reach sixty.
    fn set_tr(&mut self, value: u32, ampm: bool) {
        self.second = from_bcd(value & 0x7f).min(59);
        self.minute = from_bcd((value >> 8) & 0x7f).min(59);
        let hour = from_bcd((value >> 16) & 0x3f);
        let pm = value & (1 << 22) != 0;
        self.hour = if ampm {
            let h12 = hour.clamp(1, 12);
            match (pm, h12) {
                (false, 12) => 0,
                (false, h) => h,
                (true, 12) => 12,
                (true, h) => h + 12,
            }
        } else {
            hour.min(23)
        };
    }

    /// Take a `DR` write.
    fn set_dr(&mut self, value: u32) {
        self.year = from_bcd((value >> 16) & 0xff).min(99);
        self.month = from_bcd((value >> 8) & 0x1f).clamp(1, 12);
        self.weekday = (((value >> 13) & 0x7) as u8).clamp(1, 7);
        let last = days_in_month(self.full_year(), self.month);
        self.day = from_bcd(value & 0x3f).clamp(1, last);
    }
}

/// Parse an `epoch` property.
///
/// `YYYY-MM-DD`, optionally followed by `T` or a space and `HH:MM:SS`. The
/// weekday is computed, not given: it is a fact about the date.
///
/// # Errors
///
/// [`Error::Property`] if the text is not that shape, or if the date it names
/// does not exist.
pub fn parse_epoch(text: &str) -> Result<Calendar> {
    let bad = |why: &str| {
        Error::Property(format!(
            "`epoch` is `{text}`: {why} (expected `YYYY-MM-DD` or `YYYY-MM-DDTHH:MM:SS`)"
        ))
    };
    let (date, time) = match text.find(['T', 't', ' ']) {
        Some(at) => (&text[..at], &text[at + 1..]),
        None => (text, "00:00:00"),
    };

    // One decimal field, bounded. `None` is "the text ran out", which the two
    // callers below treat differently: a missing date field is an error and a
    // missing time field is a zero.
    let number = |field: Option<&str>, max: u32| -> Result<u32> {
        let field = field.ok_or_else(|| bad("too few fields"))?;
        if field.is_empty() || !field.bytes().all(|b| b.is_ascii_digit()) {
            return Err(bad("a field is not a decimal number"));
        }
        let value: u32 = field.parse().map_err(|_| bad("a field does not fit"))?;
        if value > max {
            return Err(bad("a field is out of range"));
        }
        Ok(value)
    };

    let mut parts = date.split('-');
    let year = number(parts.next(), 9999)?;
    let month = number(parts.next(), 12)?;
    let day = number(parts.next(), 31)?;
    if parts.next().is_some() {
        return Err(bad("too many `-`-separated fields"));
    }
    // `YT`/`YU` is two BCD digits on top of a fixed 2000, so that is the whole
    // range a calendar can hold and a year outside it would be silently
    // truncated rather than refused.
    if !(2000..=2099).contains(&year) {
        return Err(bad(
            "the year must be 2000-2099, which is all `YT`/`YU` can hold",
        ));
    }
    // Not `days_in_month`'s fallback: a month of zero has to be refused here
    // rather than given thirty days.
    if month == 0 || day == 0 || day > u32::from(days_in_month(year, month as u8)) {
        return Err(bad("that date does not exist"));
    }

    let mut fields = time.split(':');
    let hour = number(fields.next().or(Some("0")), 23)?;
    let minute = number(fields.next().or(Some("0")), 59)?;
    // A leap second is not representable and the hardware has no way to count
    // one, so 60 is refused rather than folded into the next minute.
    let second = number(fields.next().or(Some("0")), 59)?;
    if fields.next().is_some() {
        return Err(bad("too many `:`-separated fields"));
    }

    Ok(Calendar {
        second: second as u8,
        minute: minute as u8,
        hour: hour as u8,
        day: day as u8,
        month: month as u8,
        year: (year - 2000) as u8,
        weekday: Calendar::weekday_of(year, month as u8, day as u8),
    })
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the divider chain's position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The counter itself.
    live: Calendar,
    /// The copy `TR`/`DR` read when `BYPSHAD` is clear.
    shadow: Calendar,
    /// The synchronous prescaler's down-counter — what `SSR` reads.
    ss: u32,
    /// The shadow copy of it.
    shadow_ss: u32,
    /// How far into the asynchronous prescaler's period the chain is.
    apre: u32,
    /// `CR`.
    cr: u32,
    /// `ISR`, minus the bits computed on the way out.
    isr: u32,
    /// `PRER`.
    prer: u32,
    /// `WUTR`.
    wutr: u32,
    /// `CALIBR`, which only an F4 has.
    calibr: u32,
    /// `ALRMAR`.
    alrmar: u32,
    /// `ALRMBR`.
    alrmbr: u32,
    /// `ALRMASSR`.
    alrmassr: u32,
    /// `ALRMBSSR`.
    alrmbssr: u32,
    /// `TSTR`, latched at the timestamp edge.
    tstr: u32,
    /// `TSDR`.
    tsdr: u32,
    /// `TSSSR`.
    tsssr: u32,
    /// `CALR`.
    calr: u32,
    /// `TAMPCR`.
    tampcr: u32,
    /// `OR`.
    or: u32,
    /// The thirty-two backup registers.
    bkp: [u32; BKP_COUNT],
    /// How much of the `WPR` unlock sequence has arrived: 1 after `0xCA`.
    wpr_stage: u8,
    /// Whether the protected registers are closed.
    locked: bool,
    /// Whether a read of `TR`/`SSR` is holding the shadow until `DR` is read.
    readout_lock: bool,
    /// Whether alarm A's comparison matched last time it was evaluated, so
    /// that `ALRAF` is set on the edge rather than on every tick of a match.
    alra_matched: bool,
    /// The same for alarm B.
    alrb_matched: bool,
    /// The wakeup down-counter.
    wut: u32,
    /// How far into the wakeup divider's period the counter is.
    wut_prescale: u32,
    /// The tick `INITF` follows `INIT` at.
    initf_due: Option<u64>,
    /// The tick the next shadow copy lands at, when one is owed.
    shadow_due: Option<u64>,
    /// The tick a pending `SHIFTR` completes at.
    shift_due: Option<u64>,
    /// The tick `RECALPF` drops at.
    recalp_due: Option<u64>,
    /// The level last seen on the timestamp input, for edge detection.
    ts_level: bool,
    /// Where the device has advanced to, in its own clock domain.
    tick: u64,
}

/// What one step of the counter produced that the outside world can see.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Events {
    /// Whether the calendar crossed at least one second boundary.
    second: bool,
}

/// What the three interrupt outputs should read.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Outputs {
    /// The alarm A/B line.
    alarm: bool,
    /// The wakeup-timer line.
    wakeup: bool,
    /// The timestamp/tamper line.
    stamp: bool,
}

impl State {
    /// A fresh backup domain, with `epoch` in the calendar.
    fn new(epoch: Calendar) -> State {
        State {
            live: epoch,
            shadow: epoch,
            ss: PRER_RESET & 0x7fff,
            shadow_ss: PRER_RESET & 0x7fff,
            apre: 0,
            cr: 0,
            isr: ISR_RESET | ISR_RSF,
            prer: PRER_RESET,
            wutr: WUTR_RESET,
            calibr: 0,
            alrmar: 0,
            alrmbr: 0,
            alrmassr: 0,
            alrmbssr: 0,
            tstr: 0,
            tsdr: 0,
            tsssr: 0,
            calr: 0,
            tampcr: 0,
            or: 0,
            bkp: [0; BKP_COUNT],
            wpr_stage: 0,
            locked: true,
            readout_lock: false,
            alra_matched: false,
            alrb_matched: false,
            wut: WUTR_RESET,
            wut_prescale: 0,
            initf_due: None,
            shadow_due: None,
            shift_due: None,
            recalp_due: None,
            ts_level: false,
            tick: 0,
        }
    }

    /// `PREDIV_A`, the asynchronous prescaler's reload.
    fn prediv_a(&self) -> u32 {
        (self.prer >> 16) & 0x7f
    }

    /// `PREDIV_S`, the synchronous prescaler's reload.
    fn prediv_s(&self) -> u32 {
        self.prer & 0x7fff
    }

    /// How many RTCCLK cycles one step of the synchronous prescaler costs.
    fn apre_period(&self) -> u64 {
        u64::from(self.prediv_a()) + 1
    }

    /// Whether initialisation mode has taken effect, which is what stops the
    /// counter and what opens `TR`, `DR` and `PRER`.
    fn initf(&self) -> bool {
        self.isr & ISR_INITF != 0
    }

    /// Whether the calendar is counting: `INIT` stops it and so does RCC
    /// taking `BDCR.RTCEN` away.
    fn counting(&self, enabled: bool) -> bool {
        enabled && self.isr & (ISR_INIT | ISR_INITF) == 0
    }

    /// Whether reads bypass the shadow copy.
    fn bypshad(&self) -> bool {
        self.cr & CR_BYPSHAD != 0
    }

    /// Whether the shadow may be refreshed right now.
    ///
    /// Not while a `TR`/`SSR` read is holding it (RM0351 §37.3.7), not in
    /// initialisation mode, and not in bypass mode where there is no copy.
    fn may_refresh(&self) -> bool {
        !self.readout_lock && !self.bypshad() && self.isr & (ISR_INIT | ISR_INITF) == 0
    }

    /// Copy the live calendar into the shadow and set `RSF`.
    fn refresh_shadow(&mut self) {
        self.shadow = self.live;
        self.shadow_ss = self.ss;
        self.isr |= ISR_RSF;
        self.shadow_due = None;
    }

    /// Copy the calendar to the shadow during initialisation.
    ///
    /// `RSF` stays clear in initialisation mode, so this is only so that the
    /// shadow a `BYPSHAD = 0` read sees on the way out of init is the value
    /// that was just written rather than the one from before it.
    fn refresh_shadow_in_init(&mut self) {
        self.shadow = self.live;
        self.shadow_ss = self.ss;
    }

    /// Arm the next shadow copy, two RTCCLK cycles out.
    fn arm_shadow(&mut self) {
        self.shadow_due = Some(self.tick + SYNC_CYCLES);
    }

    /// How many RTCCLK cycles one step of the wakeup down-counter costs, or
    /// `None` when `WUCKSEL` has it on `ck_spre` instead (RM0351 §37.6.3).
    fn wut_period(&self) -> Option<u64> {
        match self.cr & CR_WUCKSEL {
            0 => Some(16),
            1 => Some(8),
            2 => Some(4),
            3 => Some(2),
            _ => None,
        }
    }

    /// What the wakeup counter reloads with. `WUCKSEL = 11x` adds 2^16, which
    /// is how the block reaches a thirty-six-hour timeout on `ck_spre`.
    fn wut_reload(&self) -> u32 {
        let base = self.wutr & 0xffff;
        if self.cr & CR_WUCKSEL >= 0b110 {
            base + 0x1_0000
        } else {
            base
        }
    }

    /// Advance the divider chain by `n` RTCCLK cycles, returning how many
    /// one-second edges it produced.
    ///
    /// Arithmetic rather than a loop: the chain is two modular counters, and
    /// counting to 32768 one cycle at a time to find that out would be a busy
    /// loop on every catch-up.
    fn advance_chain(&mut self, n: u64) -> u64 {
        let apre_period = self.apre_period();
        let total = u64::from(self.apre) + n;
        let edges = total / apre_period;
        // Below `apre_period`, itself at most 128.
        self.apre = (total % apre_period) as u32;
        if edges == 0 {
            return 0;
        }
        let s_period = u64::from(self.prediv_s()) + 1;
        // The counter runs *down* from `PREDIV_S`, so its position in the
        // period is the distance it has already fallen.
        let position = u64::from(self.prediv_s().saturating_sub(self.ss)) + edges;
        let seconds = position / s_period;
        // Below `s_period`, which is at most 0x8000.
        self.ss = self.prediv_s() - (position % s_period) as u32;
        seconds
    }

    /// Take `edges` steps of the wakeup down-counter, returning whether it
    /// passed zero at least once.
    fn step_wakeup(&mut self, edges: u64) -> bool {
        if edges == 0 {
            return false;
        }
        let reload = self.wut_reload();
        let period = u64::from(reload) + 1;
        let position = u64::from(reload.saturating_sub(self.wut)) + edges;
        let fires = position / period;
        // Below `period`, which is at most 0x2_0000.
        self.wut = reload - (position % period) as u32;
        fires > 0
    }

    /// Advance the wakeup down-counter by `n` RTCCLK cycles on one of the
    /// `RTCCLK/2`…`RTCCLK/16` selections.
    fn advance_wakeup(&mut self, n: u64) -> bool {
        let Some(period) = self.wut_period() else {
            return false;
        };
        if self.cr & CR_WUTE == 0 {
            return false;
        }
        let total = u64::from(self.wut_prescale) + n;
        let edges = total / period;
        // Below `period`, at most 16.
        self.wut_prescale = (total % period) as u32;
        self.step_wakeup(edges)
    }

    /// Advance `n` RTCCLK cycles, with `enabled` saying whether `BDCR.RTCEN`
    /// is letting the counter run.
    fn step(&mut self, n: u64, enabled: bool) -> Events {
        let end = self.tick + n;
        let mut events = Events::default();
        let before = (self.live, self.ss);
        if self.counting(enabled) {
            let seconds = self.advance_chain(n);
            for _ in 0..seconds {
                self.live.tick_second();
                events.second = true;
            }
            if seconds > 0 && self.wut_period().is_none() && self.cr & CR_WUTE != 0 {
                // `WUCKSEL = 10x`/`11x` clocks the wakeup counter from
                // `ck_spre`, so its steps are the calendar's seconds.
                if self.step_wakeup(seconds) {
                    self.isr |= ISR_WUTF;
                }
            }
            if self.advance_wakeup(n) {
                self.isr |= ISR_WUTF;
            }
        }
        self.tick = end;
        if self.initf_due.is_some_and(|at| at <= end) {
            self.initf_due = None;
            // Only if `INIT` is still asked for: a driver that set and cleared
            // it inside the synchronisation window never enters the mode.
            if self.isr & ISR_INIT != 0 {
                self.isr |= ISR_INITF;
                // "RSF is cleared by hardware in initialization mode."
                self.isr &= !ISR_RSF;
                self.shadow_due = None;
            }
        }
        if self.shift_due.is_some_and(|at| at <= end) {
            self.shift_due = None;
        }
        if self.recalp_due.is_some_and(|at| at <= end) {
            self.recalp_due = None;
        }
        // The hardware copies every two RTCCLK cycles. At this granularity the
        // copy that follows a change is indistinguishable from an immediate
        // one, so the only copy that has to be *scheduled* is the one software
        // asks for by clearing `RSF`. The comparison is against `SSR` as well
        // as the calendar: the sub-second moves at the asynchronous
        // prescaler's rate, 256 times faster than the seconds field, and a
        // shadow refreshed only on the second would read stale between them.
        let owed = self.shadow_due.is_some_and(|at| at <= end);
        if (self.live, self.ss) != before || owed {
            if self.may_refresh() {
                self.refresh_shadow();
            } else if owed {
                // The copy cannot land while a `TR` read is holding the shadow,
                // and leaving the deadline in the past would ask the scheduler
                // for an event every tick until `DR` is read. The read that
                // releases the lock arms the next one.
                self.shadow_due = None;
            }
        }
        events
    }

    /// RTCCLK cycles until the next thing that changes what the guest reads.
    fn next_event(&self, enabled: bool) -> Option<u64> {
        let mut soonest: Option<u64> = None;
        let mut at = |delta: u64| {
            let delta = delta.max(1);
            soonest = Some(match soonest {
                Some(current) => current.min(delta),
                None => delta,
            });
        };
        if self.counting(enabled) {
            // The next one-second edge: what is left of this step of the
            // asynchronous prescaler, plus the steps still owed to the
            // synchronous one.
            let apre_left = self.apre_period() - u64::from(self.apre);
            at(apre_left + u64::from(self.ss) * self.apre_period());
            // A sub-second alarm compares against `SSR`, which moves at the
            // asynchronous prescaler's rate rather than the calendar's. Only
            // an armed one is worth waking for, or every board with an RTC
            // would take an event 256 times a second forever.
            if self.subsecond_armed() {
                at(apre_left);
            }
            if let Some(period) = self.wut_period()
                && self.cr & CR_WUTE != 0
            {
                let left = period - u64::from(self.wut_prescale);
                at(left + u64::from(self.wut) * period);
            }
        }
        for deadline in [
            self.initf_due,
            self.shadow_due,
            self.shift_due,
            self.recalp_due,
        ]
        .into_iter()
        .flatten()
        {
            at(deadline.saturating_sub(self.tick));
        }
        soonest
    }

    /// Whether either alarm is enabled with a sub-second comparison.
    fn subsecond_armed(&self) -> bool {
        (self.cr & CR_ALRAE != 0 && (self.alrmassr >> 24) & 0xf != 0)
            || (self.cr & CR_ALRBE != 0 && (self.alrmbssr >> 24) & 0xf != 0)
    }

    /// Whether an alarm register matches the live calendar.
    ///
    /// `MSK1`…`MSK4` each excuse one field from the comparison, and `WDSEL`
    /// swaps the date field for the day of the week (RM0351 §37.6.8).
    fn alarm_matches(&self, alrm: u32, alrmss: u32) -> bool {
        let now = self.live.to_tr(self.cr & CR_FMT != 0);
        // Seconds, minutes and hours, each against `MSK1`, `MSK2` and `MSK3`.
        for (mask, shift, width) in [(7u32, 0u32, 0x7fu32), (15, 8, 0x7f), (23, 16, 0x7f)] {
            if alrm & (1 << mask) == 0 && (alrm >> shift) & width != (now >> shift) & width {
                return false;
            }
        }
        if alrm & (1 << 31) == 0 {
            let field = (alrm >> 24) & 0x3f;
            let wanted = if alrm & (1 << 30) != 0 {
                // `WDSEL = 1`: the field is `WDU`, one binary digit.
                u32::from(self.live.weekday)
            } else {
                to_bcd(self.live.day)
            };
            if field != wanted {
                return false;
            }
        }
        let mask = (alrmss >> 24) & 0xf;
        if mask != 0 {
            // "MASKSS: only SS[MASKSS-1:0] are compared."
            let keep = (1u32 << mask.min(15)) - 1;
            if (alrmss & 0x7fff) & keep != self.ss & keep {
                return false;
            }
        }
        true
    }

    /// Re-evaluate both alarms, setting `ALRAF`/`ALRBF` on a rising match.
    ///
    /// An edge rather than a level: with `MSK1`…`MSK4` all set the comparison
    /// is true for a whole second, and a flag re-set on every evaluation would
    /// be one software could never clear.
    fn check_alarms(&mut self) {
        let a = self.cr & CR_ALRAE != 0 && self.alarm_matches(self.alrmar, self.alrmassr);
        if a && !self.alra_matched {
            self.isr |= ISR_ALRAF;
        }
        self.alra_matched = a;

        let b = self.cr & CR_ALRBE != 0 && self.alarm_matches(self.alrmbr, self.alrmbssr);
        if b && !self.alrb_matched {
            self.isr |= ISR_ALRBF;
        }
        self.alrb_matched = b;
    }

    /// `ISR` as the guest reads it: the stored bits plus the ones computed
    /// from elsewhere.
    fn isr_read(&self) -> u32 {
        let mut isr =
            self.isr & !(ISR_ALRAWF | ISR_ALRBWF | ISR_WUTWF | ISR_INITS | ISR_SHPF | ISR_RECALPF);
        // "ALRAWF: this bit is set by hardware when alarm A values can be
        // changed, after the ALRAE bit has been set to 0."
        if self.cr & CR_ALRAE == 0 {
            isr |= ISR_ALRAWF;
        }
        if self.cr & CR_ALRBE == 0 {
            isr |= ISR_ALRBWF;
        }
        if self.cr & CR_WUTE == 0 {
            isr |= ISR_WUTWF;
        }
        // "INITS: this bit is set by hardware when the calendar year field is
        // different from 0 (backup domain reset state)."
        if self.live.year != 0 {
            isr |= ISR_INITS;
        }
        if self.shift_due.is_some() {
            isr |= ISR_SHPF;
        }
        if self.recalp_due.is_some() {
            isr |= ISR_RECALPF;
        }
        isr
    }

    /// The levels the three interrupt outputs should be at.
    fn outputs(&self) -> Outputs {
        Outputs {
            alarm: (self.isr & ISR_ALRAF != 0 && self.cr & CR_ALRAIE != 0)
                || (self.isr & ISR_ALRBF != 0 && self.cr & CR_ALRBIE != 0),
            wakeup: self.isr & ISR_WUTF != 0 && self.cr & CR_WUTIE != 0,
            // The tamper flags are storage here, but a board that sets one
            // through a snapshot should still see the line move.
            stamp: (self.isr & ISR_TSF != 0 && self.cr & CR_TSIE != 0)
                || (self.isr & ISR_TAMP1F != 0 && self.tampcr & (1 << 2) != 0),
        }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// What a backup-domain reset puts back in the calendar.
    epoch: Calendar,
    /// `BDCR.RTCEN`, as it arrives on the `rtcen` input.
    rtcen: AtomicU32,
    /// Whether anything drives `rtcen` at all. A board that does not model the
    /// gate gets a running clock rather than a stopped one — the same
    /// accommodation `st.rcc` makes for an unwired `dbp`.
    rtcen_wired: AtomicU32,
    /// The alarm A/B interrupt output.
    alarm_out: Mutex<Option<WireSource>>,
    /// The wakeup-timer interrupt output.
    wakeup_out: Mutex<Option<WireSource>>,
    /// The timestamp/tamper interrupt output.
    stamp_out: Mutex<Option<WireSource>>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// [`State::tick`], republished for the lock-free scheduler surface: the
    /// scheduler asks [`Device::current_tick`] with its slot held at
    /// [`LockRank::LEAF`], so that call may not take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next event, or [`u64::MAX`] for none. Same
    /// no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("epoch", &self.epoch)
            .field("rtcen", &self.enabled());
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Whether RCC is letting the counter run.
    fn enabled(&self) -> bool {
        self.rtcen_wired.load(Ordering::Relaxed) == 0 || self.rtcen.load(Ordering::Relaxed) != 0
    }

    /// Republish what the lock-free lazy surface reads.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event(self.enabled()) {
            Some(delta) => state.tick.saturating_add(delta),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Drive the three interrupt lines.
    ///
    /// **Never called with the state lock held**: an interrupt reaches the
    /// EXTI, the NVIC and the core, and any of those may call back
    /// (`CLAUDE.md`, *Concurrency*).
    fn drive(&self, out: Outputs) {
        for (cell, level) in [
            (&self.alarm_out, out.alarm),
            (&self.wakeup_out, out.wakeup),
            (&self.stamp_out, out.stamp),
        ] {
            let source = cell.lock().clone();
            if let Some(source) = source {
                source.set(if level { Level::High } else { Level::Low });
            }
        }
    }

    /// Advance to `target` of the RTC's own clock domain.
    ///
    /// One iteration per internal event rather than one jump, so an alarm
    /// lands on the tick it matched on. The scheduler bounds a quantum by
    /// [`Device::next_event_tick`], so in a running machine the loop turns
    /// once.
    fn advance_to(&self, target: u64) {
        loop {
            let (reached, out) = {
                let mut state = self.state.lock();
                if target <= state.tick {
                    return;
                }
                let span = target - state.tick;
                // At least one tick, so catch-up always makes progress.
                let step = state
                    .next_event(self.enabled())
                    .unwrap_or(span)
                    .clamp(1, span);
                let events = state.step(step, self.enabled());
                if events.second || state.subsecond_armed() {
                    state.check_alarms();
                }
                self.publish(&state);
                (state.tick >= target, state.outputs())
            };
            self.drive(out);
            if reached {
                return;
            }
        }
    }

    /// Catch the RTC up before an access is dispatched to it (§4.2).
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, attrs: MemAttrs) {
        if attrs.debug {
            return;
        }
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        // A refusal means catch-up for this device is already running further
        // up the stack; the access still has to be answered from where the
        // calendar stands.
        let _ = handle.sync(AccessKind::Guest);
    }

    /// RCC moved `BDCR.RTCEN`.
    fn set_rtcen(&self, high: bool) {
        // Catch up on the old setting first, or the cycles either side of the
        // change are counted under the wrong one.
        self.sync(MemAttrs::DEFAULT);
        self.rtcen_wired.store(1, Ordering::Relaxed);
        self.rtcen.store(u32::from(high), Ordering::Relaxed);
        let state = self.state.lock();
        self.publish(&state);
    }

    /// RCC asserted `BDCR.BDRST`: the whole backup domain goes.
    fn backup_reset(&self) {
        let out = {
            let mut state = self.state.lock();
            let tick = state.tick;
            *state = State::new(self.epoch);
            // The cursor in the clock domain survives, for the reason
            // `st.iwdg`'s reset gives: it is not architectural state but this
            // device's position in a domain that did not rewind.
            state.tick = tick;
            self.publish(&state);
            state.outputs()
        };
        self.drive(out);
    }

    /// The timestamp input moved.
    fn set_ts(&self, high: bool) {
        self.sync(MemAttrs::DEFAULT);
        let out = {
            let mut state = self.state.lock();
            let edge = if state.cr & CR_TSEDGE != 0 {
                state.ts_level && !high
            } else {
                !state.ts_level && high
            };
            state.ts_level = high;
            if edge && state.cr & CR_TSE != 0 {
                if state.isr & ISR_TSF != 0 {
                    // "TSOVF is set when a time-stamp event occurs while TSF
                    // is already set": the *new* capture is the one dropped,
                    // not the one software has not read (RM0351 §37.6.4).
                    state.isr |= ISR_TSOVF;
                } else {
                    state.tstr = state.live.to_tr(state.cr & CR_FMT != 0);
                    state.tsdr = state.live.to_dr();
                    state.tsssr = state.ss & 0xffff;
                    state.isr |= ISR_TSF;
                }
            }
            state.outputs()
        };
        self.drive(out);
    }

    /// Read one register.
    ///
    /// `debug` suppresses the coherency lock in both directions: a debugger
    /// that read `TR` would otherwise freeze the guest's next calendar read,
    /// and one that read `DR` would release a lock the guest is relying on.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let mut state = self.state.lock();
        let ampm = state.cr & CR_FMT != 0;
        let shadowed = !state.bypshad();
        match offset {
            TR => {
                if shadowed && !debug {
                    state.readout_lock = true;
                }
                (if shadowed { state.shadow } else { state.live }).to_tr(ampm)
            }
            DR => {
                let value = (if shadowed { state.shadow } else { state.live }).to_dr();
                if shadowed && !debug && state.readout_lock {
                    // "the values are unlocked when RTC_DR is read", and the
                    // copy lands two cycles later.
                    state.readout_lock = false;
                    state.arm_shadow();
                }
                value
            }
            SSR => {
                if shadowed && !debug {
                    state.readout_lock = true;
                }
                (if shadowed { state.shadow_ss } else { state.ss }) & 0xffff
            }
            CR => state.cr,
            ISR => state.isr_read(),
            PRER => state.prer,
            WUTR => state.wutr,
            CALIBR => state.calibr,
            ALRMAR => state.alrmar,
            ALRMBR => state.alrmbr,
            // "WPR ... write only": a read returns zero, so firmware cannot
            // ask whether the protection is off, and the unlock has to be
            // latched rather than inferred.
            WPR | SHIFTR => 0,
            TSTR => state.tstr,
            TSDR => state.tsdr,
            TSSSR => state.tsssr,
            CALR => state.calr,
            TAMPCR => state.tampcr,
            ALRMASSR => state.alrmassr,
            ALRMBSSR => state.alrmbssr,
            OR => state.or,
            _ => match backup_index(offset) {
                Some(n) => state.bkp[n],
                None => 0,
            },
        }
    }

    /// Write one register, returning the levels the outputs should now be at.
    #[allow(clippy::too_many_lines)]
    fn write_register(&self, offset: u64, value: u32) -> Outputs {
        let mut state = self.state.lock();

        // The unprotected registers first: those are reachable whatever the
        // key register says (RM0351 §37.3.6).
        match offset {
            WPR => {
                let key = value & 0xff;
                if key == WPR_KEY2 && state.wpr_stage == 1 {
                    state.locked = false;
                } else {
                    // A fresh `0xCA` restarts the sequence, and a half-entered
                    // sequence is not the unlocked state: the key register is
                    // a two-step machine rather than a latch.
                    state.locked = true;
                }
                state.wpr_stage = u8::from(key == WPR_KEY1);
                return state.outputs();
            }
            TAMPCR => {
                state.tampcr = value & 0x07ff_ffff;
                return state.outputs();
            }
            OR => {
                state.or = value & 0xf;
                return state.outputs();
            }
            ISR => {
                // The flag half is unprotected; `INIT` and `RSF` are not.
                let clear = ISR_RC_W0 & !value;
                state.isr &= !clear;
                if clear & ISR_TSF != 0 {
                    // Clearing `TSF` clears `TSOVF` with it: an overflow only
                    // means something relative to a capture nobody read.
                    state.isr &= !ISR_TSOVF;
                }
                if !state.locked {
                    if value & ISR_RSF == 0 && state.isr & ISR_RSF != 0 {
                        state.isr &= !ISR_RSF;
                        state.arm_shadow();
                    }
                    let want = value & ISR_INIT != 0;
                    let had = state.isr & ISR_INIT != 0;
                    if want && !had {
                        state.isr |= ISR_INIT;
                        state.isr &= !ISR_RSF;
                        state.shadow_due = None;
                        state.initf_due = Some(state.tick + SYNC_CYCLES);
                    } else if !want && had {
                        state.isr &= !(ISR_INIT | ISR_INITF);
                        state.initf_due = None;
                        // "the counter restarts counting" — from the top of
                        // the divider chain, which is what makes a calendar
                        // set to 23:59:59 roll over exactly 32768 cycles later
                        // on a reset-value prescaler.
                        state.apre = 0;
                        state.ss = state.prediv_s();
                        // An alarm can legitimately match the very second the
                        // calendar was just set to, so forget the comparison
                        // made against the calendar it replaced.
                        state.alra_matched = false;
                        state.alrb_matched = false;
                        state.arm_shadow();
                    }
                }
                let out = state.outputs();
                self.publish(&state);
                return out;
            }
            _ => {}
        }
        if let Some(n) = backup_index(offset) {
            state.bkp[n] = value;
            return state.outputs();
        }

        if state.locked {
            // Everything below is write-protected and the key sequence has not
            // arrived. The write is dropped rather than faulted: the bus sees
            // a perfectly ordinary store.
            return state.outputs();
        }

        match offset {
            TR if state.initf() => {
                let ampm = state.cr & CR_FMT != 0;
                state.live.set_tr(value, ampm);
                state.refresh_shadow_in_init();
            }
            DR if state.initf() => {
                state.live.set_dr(value);
                state.refresh_shadow_in_init();
            }
            CR => {
                let previous = state.cr;
                let mut next = value & CR_MASK;
                // "WUCKSEL can be changed only when RTC_CR WUTE bit = 0"
                // (RM0351 §37.6.3).
                if previous & CR_WUTE != 0 {
                    next = (next & !CR_WUCKSEL) | (previous & CR_WUCKSEL);
                }
                if previous & CR_ALRAE != 0 && next & CR_ALRAE == 0 {
                    state.alra_matched = false;
                }
                if previous & CR_ALRBE != 0 && next & CR_ALRBE == 0 {
                    state.alrb_matched = false;
                }
                state.cr = next;
                if next & CR_WUTE != 0 && previous & CR_WUTE == 0 {
                    state.wut = state.wut_reload();
                    state.wut_prescale = 0;
                }
                if next & CR_BYPSHAD != 0 {
                    // "RSF is cleared by hardware ... when in bypass shadow
                    // register mode."
                    state.isr &= !ISR_RSF;
                    state.shadow_due = None;
                } else if previous & CR_BYPSHAD != 0 {
                    state.arm_shadow();
                }
                // `ADD1H`/`SUB1H` are actions and never read back. Both at
                // once cancel, which is what two opposite corrections do.
                let add = value & CR_ADD1H != 0;
                let sub = value & CR_SUB1H != 0;
                if add != sub {
                    state.live.add_hours(if add { 1 } else { -1 });
                    if state.may_refresh() {
                        state.refresh_shadow();
                    }
                }
            }
            PRER if state.initf() => {
                state.prer = value & 0x007f_7fff;
                state.ss = state.prediv_s();
                state.apre = 0;
            }
            // "WUTR ... can be written only when WUTWF is set", which is to
            // say while the wakeup timer is disabled.
            WUTR if state.cr & CR_WUTE == 0 => {
                state.wutr = value & 0xffff;
                state.wut = state.wut_reload();
                state.wut_prescale = 0;
            }
            CALIBR => state.calibr = value & 0x0000_01ff,
            ALRMAR if state.cr & CR_ALRAE == 0 => {
                state.alrmar = value;
                state.alra_matched = false;
            }
            ALRMBR if state.cr & CR_ALRBE == 0 => {
                state.alrmbr = value;
                state.alrb_matched = false;
            }
            ALRMASSR if state.cr & CR_ALRAE == 0 => state.alrmassr = value & 0x0f00_7fff,
            ALRMBSSR if state.cr & CR_ALRBE == 0 => state.alrmbssr = value & 0x0f00_7fff,
            SHIFTR => {
                // "This register can be written only when SHPF is 0, REFCKON
                // is 0 and the RTC is not in initialization mode."
                if state.shift_due.is_none() && state.cr & CR_REFCKON == 0 && !state.initf() {
                    // `SUBFS` *delays* the clock by that many fractions of a
                    // second, and the counter runs down, so a delay is an
                    // addition.
                    let period = state.prediv_s() + 1;
                    state.ss = (state.ss + (value & 0x7fff) % period) % period;
                    if value & (1 << 31) != 0 {
                        // `ADD1S`.
                        state.live.tick_second();
                    }
                    state.shift_due = Some(state.tick + SYNC_CYCLES);
                    if state.may_refresh() {
                        state.refresh_shadow();
                    }
                }
            }
            CALR => {
                state.calr = value & 0x0000_8dff;
                // `RECALPF` stands until the calibration cycle in flight ends.
                // The real cycle is up to thirty-two seconds; what firmware
                // needs is that the flag take *some* time, so this model uses
                // the synchronisation delay and leaves the rate uncorrected.
                // See the module documentation.
                state.recalp_due = Some(state.tick + SYNC_CYCLES);
            }
            _ => {}
        }
        state.check_alarms();
        self.publish(&state);
        state.outputs()
    }
}

/// Which backup register `offset` names, if it names one.
fn backup_index(offset: u64) -> Option<usize> {
    if !(BKP0R..REGISTER_BYTES).contains(&offset) {
        return None;
    }
    // Below `BKP_COUNT` by the range check above.
    Some(((offset - BKP0R) / 4) as usize)
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        self.sync(attrs);
        let bytes = self.read_register(offset & !3, attrs.debug).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `WPR` would open the protection the guest is
            // relying on and one to `ISR` would drop a flag it has not seen.
            // Neither has a harmless version (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        let out = self.write_register(offset & !3, value);
        // Outside the critical section, as `drive` requires.
        self.drive(out);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers can be accessed by words (32-bit) only."
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 real-time clock.
#[derive(Debug)]
pub struct Rtc {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins the machine layer has taken; a net holds its sinks
    /// weakly, so the device keeps the strong reference.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Rtc {
    /// Validate `props` and build the clock.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Rtc> {
        let mut r = props.reader();
        let epoch = match r.optional_str("epoch")? {
            Some(text) => parse_epoch(text)?,
            None => Calendar::default(),
        };
        r.touch("clock");
        r.finish()?;
        Ok(Rtc::with_epoch(epoch))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_epoch(epoch: Calendar) -> Rtc {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(epoch)),
            epoch,
            rtcen: AtomicU32::new(0),
            rtcen_wired: AtomicU32::new(0),
            alarm_out: Mutex::with_rank(LockRank::WIRE, None),
            wakeup_out: Mutex::with_rank(LockRank::WIRE, None),
            stamp_out: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region = Arc::new(Region::io(
            "rtc",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rtc {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// What a backup-domain reset puts back in the calendar.
    #[must_use]
    pub fn epoch(&self) -> Calendar {
        self.regs.epoch
    }

    /// The live calendar, whatever the shadow registers say.
    #[must_use]
    pub fn calendar(&self) -> Calendar {
        self.regs.state.lock().live
    }

    /// One backup register.
    ///
    /// # Panics
    ///
    /// If `n` is not below [`BKP_COUNT`].
    #[must_use]
    pub fn backup(&self, n: usize) -> u32 {
        self.regs.state.lock().bkp[n]
    }

    /// The tick the clock has been advanced to, in its own domain.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Advance to `tick` of the RTC's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// Drive `BDCR.RTCEN`, as RCC's `rtcen` output does.
    pub fn set_rtcen(&self, high: bool) {
        self.regs.set_rtcen(high);
    }

    /// Reset the backup domain, as an assertion of `bdrst` does.
    pub fn backup_reset(&self) {
        self.regs.backup_reset();
    }
}

impl Device for Rtc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings RCC's two outputs and the three interrupt lines.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        match kind {
            // Power-on: there is no battery and no charge left, so the backup
            // domain comes up at `epoch` like everything else.
            ResetKind::Cold => self.regs.backup_reset(),
            // **A system reset changes nothing.** The RTC and its backup
            // registers are in the backup domain (RM0351 §37.3.2), which is
            // exactly why firmware parks a reset reason in `BKP0R` and expects
            // to find it on the way back up. Only `BDCR.BDRST` clears it, and
            // that arrives on the `bdrst` pin.
            ResetKind::Warm | ResetKind::Bus => {}
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for cal in [state.live, state.shadow] {
            for field in [
                cal.second,
                cal.minute,
                cal.hour,
                cal.day,
                cal.month,
                cal.year,
                cal.weekday,
            ] {
                w.write_u8(field)?;
            }
        }
        for word in [
            state.ss,
            state.shadow_ss,
            state.apre,
            state.cr,
            state.isr,
            state.prer,
            state.wutr,
            state.calibr,
            state.alrmar,
            state.alrmbr,
            state.alrmassr,
            state.alrmbssr,
            state.tstr,
            state.tsdr,
            state.tsssr,
            state.calr,
            state.tampcr,
            state.or,
        ] {
            w.write_u32(word)?;
        }
        // The backup registers are the whole reason this device survives a
        // reset; a snapshot that lost them would lose the boot flag with them.
        for word in state.bkp {
            w.write_u32(word)?;
        }
        w.write_u8(state.wpr_stage)?;
        w.write_bool(state.locked)?;
        w.write_bool(state.readout_lock)?;
        w.write_bool(state.alra_matched)?;
        w.write_bool(state.alrb_matched)?;
        w.write_u32(state.wut)?;
        w.write_u32(state.wut_prescale)?;
        w.write_bool(state.ts_level)?;
        // The cursor in the clock domain: the scheduler restores the domain,
        // and without this the two would disagree and the next catch-up would
        // advance the calendar by however long the machine had been running.
        w.write_u64(state.tick)?;
        for deadline in [
            state.initf_due,
            state.shadow_due,
            state.shift_due,
            state.recalp_due,
        ] {
            match deadline {
                None => w.write_bool(false)?,
                Some(at) => {
                    w.write_bool(true)?;
                    w.write_u64(at)?;
                }
            }
        }
        Ok(())
        // `rtcen` is absent on purpose: it is RCC's `BDCR.RTCEN`, RCC's
        // snapshot carries it and the wire re-announces it — the same rule
        // `st.rcc` applies to the `dbp` level it is given.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.regs.epoch);
        for cal in [&mut state.live, &mut state.shadow] {
            *cal = Calendar {
                second: r.read_u8()?,
                minute: r.read_u8()?,
                hour: r.read_u8()?,
                day: r.read_u8()?,
                month: r.read_u8()?,
                year: r.read_u8()?,
                weekday: r.read_u8()?,
            };
        }
        for word in [
            &mut state.ss,
            &mut state.shadow_ss,
            &mut state.apre,
            &mut state.cr,
            &mut state.isr,
            &mut state.prer,
            &mut state.wutr,
            &mut state.calibr,
            &mut state.alrmar,
            &mut state.alrmbr,
            &mut state.alrmassr,
            &mut state.alrmbssr,
            &mut state.tstr,
            &mut state.tsdr,
            &mut state.tsssr,
            &mut state.calr,
            &mut state.tampcr,
            &mut state.or,
        ] {
            *word = r.read_u32()?;
        }
        for word in &mut state.bkp {
            *word = r.read_u32()?;
        }
        state.wpr_stage = r.read_u8()?;
        state.locked = r.read_bool()?;
        state.readout_lock = r.read_bool()?;
        state.alra_matched = r.read_bool()?;
        state.alrb_matched = r.read_bool()?;
        state.wut = r.read_u32()?;
        state.wut_prescale = r.read_u32()?;
        state.ts_level = r.read_bool()?;
        state.tick = r.read_u64()?;
        for deadline in [
            &mut state.initf_due,
            &mut state.shadow_due,
            &mut state.shift_due,
            &mut state.recalp_due,
        ] {
            *deadline = r.read_bool()?.then(|| r.read_u64()).transpose()?;
        }

        // A calendar out of range would never carry, and a prescaler position
        // past its own reload would count the first second short. Neither is a
        // state this device can reach, so a snapshot carrying one is corrupt
        // rather than merely surprising.
        for cal in [state.live, state.shadow] {
            if cal.second > 59
                || cal.minute > 59
                || cal.hour > 23
                || cal.month == 0
                || cal.month > 12
                || cal.day == 0
                || cal.day > days_in_month(cal.full_year(), cal.month)
                || cal.year > 99
                || cal.weekday == 0
                || cal.weekday > 7
            {
                return Err(Error::State(format!(
                    "snapshot has an RTC calendar of {cal:?}, which is not a date"
                )));
            }
        }
        if state.ss > state.prediv_s()
            || state.shadow_ss > state.prediv_s()
            || u64::from(state.apre) >= state.apre_period()
        {
            return Err(Error::State(format!(
                "snapshot has an RTC divider chain at apre={}, ss={} against PRER={:#010x}",
                state.apre, state.ss, state.prer
            )));
        }
        if state.wpr_stage > 1 {
            return Err(Error::State(format!(
                "snapshot has an RTC {} keys into a two-key sequence",
                state.wpr_stage
            )));
        }

        let out = {
            let mut live = self.regs.state.lock();
            *live = state;
            self.regs.publish(&live);
            live.outputs()
        };
        self.regs.drive(out);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let cell = match port {
            ALARM_PIN => &self.regs.alarm_out,
            WAKEUP_PIN => &self.regs.wakeup_out,
            STAMP_PIN => &self.regs.stamp_out,
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: format!(
                        "an RTC drives `{ALARM_PIN}`, `{WAKEUP_PIN}` and `{STAMP_PIN}`"
                    ),
                });
            }
        };
        *cell.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        let out = self.regs.state.lock().outputs();
        self.regs.drive(out);
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let kind = match port {
            RTCEN_PIN => {
                self.regs.rtcen_wired.store(1, Ordering::Relaxed);
                InputKind::Rtcen
            }
            BDRST_PIN => InputKind::Bdrst,
            TS_PIN => InputKind::Timestamp,
            _ => return None,
        };
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            kind,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn is_lazy(&self) -> bool {
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
}

impl Instance for Rtc {}

/// The `st.rtc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 real-time clock: the BCD calendar, both alarms, the wakeup timer and \
              thirty-two backup registers",
    properties: &[PropertySpec {
        name: "epoch",
        kind: ValueKind::Str,
        required: false,
        summary: "the calendar a backup-domain reset installs, `YYYY-MM-DDTHH:MM:SS`, 2000-2099",
    }],
    construct: |props| Ok(Box::new(Rtc::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rtc::new(props)?)))
}

/// What the validator should know about `st.rtc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("epoch", ValueKind::Str))
        .region("")
        .region("regs")
        .port(ALARM_PIN, PortDir::Out)
        .port(WAKEUP_PIN, PortDir::Out)
        .port(STAMP_PIN, PortDir::Out)
        .port(RTCEN_PIN, PortDir::In)
        .port(BDRST_PIN, PortDir::In)
        .port(TS_PIN, PortDir::In)
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// Which of the three inputs a pin is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputKind {
    /// RCC's `BDCR.RTCEN`.
    Rtcen,
    /// RCC's `BDCR.BDRST`.
    Bdrst,
    /// The timestamp input.
    Timestamp,
}

/// One of the RTC's inputs, as something a wire can drive.
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
            InputKind::Rtcen => self.regs.set_rtcen(high),
            // `BDRST` holds the domain in reset while it is asserted, so the
            // assertion is what does the work and RCC may either pulse it or
            // set and clear it.
            InputKind::Bdrst => {
                if high {
                    self.regs.backup_reset();
                }
            }
            InputKind::Timestamp => self.regs.set_ts(high),
        }
    }
}

#[cfg(test)]
mod tests;
