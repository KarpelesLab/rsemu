//! A Motorola MC146818 real-time clock and its CMOS RAM.
//!
//! # Sources
//!
//! * *MC146818 Real-Time Clock Plus RAM (RTC)* data sheet (Motorola). The
//!   register file, the four status registers, the divider and rate-select
//!   encodings, the 244 µs update-in-progress window, the alarm's "don't care"
//!   encoding, and the reset behaviour of the enable bits all come from it.
//! * *IBM Personal Computer AT Technical Reference* (1984) for the board: the
//!   chip answers ports `0x70`/`0x71`, its interrupt is IRQ8, and bit 7 of the
//!   index port is the **NMI mask** — a latch the board bolted onto the same
//!   write, not part of the chip.
//! * Ralf Brown's Interrupt List, ports section, for the CMOS layout the BIOS
//!   imposes on the RAM half: the equipment byte, the memory-size words and the
//!   checksum over `0x10`-`0x2d`. None of that is in the data sheet, because
//!   none of it is in the chip; it is convention, and it is the convention
//!   firmware validates on the first POST.
//!
//! **No emulator source was consulted for any of it** (`CLAUDE.md`, provenance).
//!
//! # The register block
//!
//! Two bytes, byte accesses only:
//!
//! ```text
//!   0  write  bits 0-6 select the CMOS register offset 1 reaches
//!             bit 7    NMI mask: set disables NMI  (board, not chip)
//!   0  read   0xff — the latch is write-only on a real PC
//!   1  read/write  the selected CMOS register
//! ```
//!
//! Indices `0x00`-`0x0d` are the clock and its four status registers;
//! `0x0e`-`0x7f` are ordinary battery-backed RAM.
//!
//! # How much memory the board has, in two pieces
//!
//! The memory-size bytes are the part of the RAM half that is easiest to get
//! wrong, because there are four pairs and they are not four numbers:
//!
//! ```text
//!   0x15/0x16  base memory, in KiB          — the 640 KiB below the video hole
//!   0x17/0x18  extended memory, in KiB      — the AT's own pair
//!   0x30/0x31  extended memory, in KiB      — the copy POST compares against it
//!   0x34/0x35  memory above 16 MiB, in 64 KiB units
//! ```
//!
//! `0x17`/`0x18` and `0x30`/`0x31` hold the **same** number, and that number
//! covers the 1-16 MiB region **only** — at most 0x3c00 KiB. Everything above
//! 16 MiB is counted in `0x34`/`0x35` instead, and firmware adds the two. That
//! is the same partition `INT 15h AX=E801h` reports (AX/CX the kilobytes to
//! 16 MiB, BX/DX the 64 KiB blocks above it), which is not a coincidence: the
//! CMOS pairs are where an `E801` answer is kept between boots, and
//! [`crate::fw::pcbios`] reads them exactly that way for both `E801` and its
//! `E820` map. So the [`extmem`](CLASS) property is *all* the memory above
//! 1 MiB and this module does the split; two properties would be two numbers
//! that could disagree.
//!
//! # A real-time clock that does not track real time
//!
//! **This clock does not read the host's.** `CLAUDE.md` forbids a device
//! reading the wall clock, and rsemu has no record/replay seam for one yet, so
//! a machine that drew its date from the host would produce a different trace
//! on every run and could not be replayed at all. The starting date and time
//! therefore come from the `time` property — an ISO-ish
//! `"YYYY-MM-DDTHH:MM:SS"` string, defaulting to
//! [`DEFAULT_TIME`] — and from there the calendar advances **only** from this
//! device's own clock domain, which the machine file rates at
//! [`TICKS_PER_SECOND`] Hz because that is the crystal soldered next to the
//! chip.
//!
//! That is a deliberate trade with a named reason: a guest that boots twice
//! from the same snapshot sees the same date twice, which is what makes a
//! regression hash meaningful. A machine that genuinely wants the host's date
//! passes it in as a property at launch, where it is recorded like any other
//! input.
//!
//! # Advancing
//!
//! The chip is a *sampled* device in §4.2's sense: a guest polls the seconds
//! register at an arbitrary instruction and must see the value at that instant,
//! not the one at the last quantum boundary. So it is
//! [lazy](Device::is_lazy) — it holds its own tick, is caught up before an
//! access is dispatched to it, and reports its next internal event (the next
//! periodic tap, or the next one-second update, whichever is sooner) so the run
//! loop stops there rather than thousands of cycles past it.
//!
//! # Two output pins
//!
//! `irq` is the chip's `IRQ` pin, asserted while any of PF/AF/UF is set with
//! its matching enable in status B, and dropped when status C is read. On a PC
//! it lands on IRQ8.
//!
//! `nmi_mask` is **not** the chip's. It is the board's NMI gate, which happens
//! to live in bit 7 of the index latch because that bit was spare; the board
//! ANDs it against its NMI sources. It is modelled here because firmware writes
//! it on nearly every access to this port pair, and a machine that ignored it
//! would take NMIs the BIOS had explicitly masked.
//!
//! # The one side effect that matters
//!
//! Reading status C clears PF, AF, UF and IRQF and drops the interrupt line.
//! It is the acknowledgement, and there is no other. A debugger read that did
//! it would silently eat the guest's interrupt, so
//! [`MemAttrs::debug`] suppresses it — as it suppresses catch-up, and as it
//! refuses a write outright, since a write to the index latch changes what a
//! later *real* read of offset 1 returns.

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
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.rtc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers: the index latch and the
/// data port. On a PC they are `0x70` and `0x71`.
pub const REGISTER_WINDOW_LEN: u64 = 2;

/// How many bytes of CMOS RAM the chip has, clock registers included.
pub const CMOS_BYTES: usize = 128;

/// The rate of this device's clock domain, in hertz.
///
/// The watch crystal beside the chip. Every internal period — the one-second
/// update, every periodic tap, the update-in-progress window — is an exact
/// integer number of these ticks, which is why nothing here needs a float.
pub const TICKS_PER_SECOND: u64 = 32_768;

/// The date and time a machine file gets if it names none.
///
/// Arbitrary, and deliberately so: it is a *constant*, which is the whole
/// point. See the module docs.
pub const DEFAULT_TIME: &str = "2026-01-01T00:00:00";

/// How long UIP is asserted before an update begins, in domain ticks.
///
/// The data sheet's figure is 244 µs, which at 32.768 kHz is exactly 8 cycles
/// (8/32768 s = 244.140625 µs). Software is told to treat UIP as "the time
/// registers are about to change, come back later", and the classic read loop
/// spins on it before every read.
const UIP_TICKS: u64 = 8;

// -- the register file ------------------------------------------------------

/// Seconds, 0-59.
const REG_SECONDS: u8 = 0x00;
/// Seconds alarm.
const REG_SECONDS_ALARM: u8 = 0x01;
/// Minutes, 0-59.
const REG_MINUTES: u8 = 0x02;
/// Minutes alarm.
const REG_MINUTES_ALARM: u8 = 0x03;
/// Hours, 0-23 or 1-12 with a PM bit.
const REG_HOURS: u8 = 0x04;
/// Hours alarm.
const REG_HOURS_ALARM: u8 = 0x05;
/// Day of week, 1 = Sunday.
const REG_WEEKDAY: u8 = 0x06;
/// Day of month, 1-31.
const REG_DAY: u8 = 0x07;
/// Month, 1-12.
const REG_MONTH: u8 = 0x08;
/// Year, two digits.
const REG_YEAR: u8 = 0x09;
/// Status A: UIP, the divider control, the periodic rate select.
const REG_STATUS_A: u8 = 0x0a;
/// Status B: SET, the three interrupt enables, SQWE, and the format bits.
const REG_STATUS_B: u8 = 0x0b;
/// Status C: the interrupt flags. Read-clear.
const REG_STATUS_C: u8 = 0x0c;
/// Status D: VRT, valid RAM and time.
const REG_STATUS_D: u8 = 0x0d;

/// The floppy drive types byte, as the AT BIOS lays it out.
const REG_FLOPPY: usize = 0x10;
/// The equipment byte.
const REG_EQUIPMENT: usize = 0x14;
/// Base memory, in kilobytes, low byte.
const REG_BASE_MEM: usize = 0x15;
/// Extended memory above 1 MiB, in kilobytes, low byte.
const REG_EXT_MEM: usize = 0x17;
/// The first byte the AT's CMOS checksum covers.
const CHECKSUM_FIRST: usize = 0x10;
/// The last byte the AT's CMOS checksum covers.
const CHECKSUM_LAST: usize = 0x2d;
/// Where the checksum's high byte goes.
const REG_CHECKSUM: usize = 0x2e;
/// Extended memory again, the copy the POST compares against `0x17`/`0x18`.
const REG_EXT_MEM_MIRROR: usize = 0x30;
/// The century, in BCD.
const REG_CENTURY: u8 = 0x32;
/// Memory above 16 MiB, in 64 KiB units, low byte.
const REG_HIGH_MEM: usize = 0x34;

// -- status A ---------------------------------------------------------------

/// Update in progress. Read-only: a write to status A cannot set it.
const A_UIP: u8 = 0x80;
/// The divider control field.
const A_DIVIDER: u8 = 0x70;
/// The one divider setting this model implements: a 32.768 kHz crystal.
/// Anything else selects a different crystal or holds the chain in reset, and
/// with a 32.768 kHz domain the honest answer is that the clock stops.
const A_DIVIDER_32KHZ: u8 = 0x20;
/// The periodic rate select field.
const A_RATE: u8 = 0x0f;

// -- status B ---------------------------------------------------------------

/// Freeze the time registers so software can write a consistent date.
const B_SET: u8 = 0x80;
/// Periodic interrupt enable.
const B_PIE: u8 = 0x40;
/// Alarm interrupt enable.
const B_AIE: u8 = 0x20;
/// Update-ended interrupt enable.
const B_UIE: u8 = 0x10;
/// Square wave enable. Accepted and stored; no pin is modelled, because no PC
/// board connects one.
const B_SQWE: u8 = 0x08;
/// Data mode: set for binary, clear for BCD.
const B_DM: u8 = 0x04;
/// Hour format: set for 24-hour, clear for 12-hour with a PM bit.
const B_24H: u8 = 0x02;
/// Daylight saving enable. Stored; the two hard-coded US transition dates the
/// data sheet describes are not modelled, and no PC BIOS sets this bit.
const B_DSE: u8 = 0x01;

/// The status B bits a RESET clears: the three interrupt enables and SQWE.
const B_RESET_CLEARS: u8 = B_PIE | B_AIE | B_UIE | B_SQWE;
/// The status B bits a RESET leaves alone: SET, DM, 24/12 and DSE.
const B_RESET_KEEPS: u8 = B_SET | B_DM | B_24H | B_DSE;
/// The data sheet's reset table accounts for every bit of status B, and so does
/// the pair above. Spelled as two halves because the table is, and asserted
/// here so a future edit cannot quietly leave a bit unaccounted for.
const _: () = assert!(B_RESET_CLEARS | B_RESET_KEEPS == 0xff);

// -- status C ---------------------------------------------------------------

/// Interrupt request: the OR of the three flags with their enables.
const C_IRQF: u8 = 0x80;
/// Periodic interrupt flag.
const C_PF: u8 = 0x40;
/// Alarm interrupt flag.
const C_AF: u8 = 0x20;
/// Update-ended interrupt flag.
const C_UF: u8 = 0x10;

// -- status D ---------------------------------------------------------------

/// Valid RAM and time. Set means the battery has held; we always have one.
const D_VRT: u8 = 0x80;

/// The index latch bit that disables NMI. Board logic, not chip logic.
const NMI_DISABLE: u8 = 0x80;

/// The index latch bits that actually select a register.
const INDEX_MASK: u8 = 0x7f;

/// What a read of the index port returns. The latch is write-only, and an
/// undriven PC bus floats high.
const INDEX_READS_AS: u8 = 0xff;

/// An alarm byte with both top bits set matches any value ("don't care").
const ALARM_DONT_CARE: u8 = 0xc0;

/// Status A as this model powers up: divider running, rate 6 (1024 Hz).
///
/// The value every AT BIOS writes, so a guest that never touches status A finds
/// what it expects.
const DEFAULT_STATUS_A: u8 = A_DIVIDER_32KHZ | 0x06;

/// Status B as this model powers up: 24-hour, BCD, no interrupts enabled.
const DEFAULT_STATUS_B: u8 = B_24H;

/// Base memory a machine file gets if it names none: the PC's 640 KiB.
const DEFAULT_BASE_MEM: u64 = 640 * 1024;

/// The equipment byte's default: floppy present, 80x25 colour, coprocessor.
const DEFAULT_EQUIPMENT: u8 = 0x2d;

/// The floppy byte's default: one 1.44 MB drive as A, nothing as B.
const DEFAULT_FLOPPY: u8 = 0x40;

/// How much extended memory the kilobyte pairs describe: the 15 MiB between
/// 1 MiB and 16 MiB, and not a byte more.
///
/// 0x3c00 KiB, and it is a *window* rather than a saturating cap. Above 16 MiB
/// the count continues in [`REG_HIGH_MEM`] in 64 KiB units, and firmware adds
/// the two — which is the same split `INT 15h AX=E801h` hands a caller (AX/CX
/// the 1-16 MiB region in KiB, capped at 3C00h; BX/DX everything above 16 MiB
/// in 64 KiB blocks), because the CMOS pairs are where an `E801` answer is
/// kept. Letting this pair saturate at 0xff00 instead would double-count the
/// 15 MiB the two pairs both claim, or — with `0x34`/`0x35` left at zero —
/// simply lose every megabyte above 64 MiB.
const EXT_MEM_WINDOW_KIB: u64 = 15 * 1024;

/// The unit [`REG_HIGH_MEM`] counts in: 64 KiB blocks above 16 MiB.
const HIGH_MEM_UNIT: u64 = 64 * 1024;

// ---------------------------------------------------------------------------
// the calendar
// ---------------------------------------------------------------------------

/// The date and time, in ordinary binary.
///
/// Kept binary internally and converted on every read and write according to
/// status B's DM and 24/12 bits, so the arithmetic below never has to think
/// about BCD or about a PM flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Calendar {
    second: u8,
    minute: u8,
    /// Always 0-23 in here, whatever status B says the guest sees.
    hour: u8,
    /// 1 = Sunday, as the chip numbers it.
    weekday: u8,
    day: u8,
    month: u8,
    /// The **full** year. The chip's year register holds two digits and cannot
    /// express the century, which is exactly why the AT BIOS keeps one in CMOS
    /// at `0x32` — and why the leap-year rule below needs it: 1900 is not a
    /// leap year and 2000 is, and `00` alone cannot tell them apart.
    year: u16,
}

impl Calendar {
    /// Whether `year` is a leap year in the proleptic Gregorian calendar:
    /// divisible by 4, except centuries not divisible by 400.
    fn is_leap(year: u16) -> bool {
        year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400))
    }

    /// How many days `month` has in `year`.
    fn days_in_month(year: u16, month: u8) -> u8 {
        match month {
            2 => {
                if Calendar::is_leap(year) {
                    29
                } else {
                    28
                }
            }
            4 | 6 | 9 | 11 => 30,
            // Including a nonsense month, which a guest can write and which
            // must not make this function's caller index past a table.
            _ => 31,
        }
    }

    /// The day of the week, 1 = Sunday, by Zeller's congruence.
    ///
    /// Zeller's own `h` is 0 = Saturday through 6 = Friday; the chip numbers
    /// Sunday as 1, so Saturday becomes 7 and every other value passes through.
    fn weekday_of(year: u16, month: u8, day: u8) -> u8 {
        let (m, y) = if month <= 2 {
            (i32::from(month) + 12, i32::from(year) - 1)
        } else {
            (i32::from(month), i32::from(year))
        };
        let k = y % 100;
        let j = y / 100;
        let h = (i32::from(day) + (13 * (m + 1)) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;
        if h == 0 { 7 } else { h as u8 }
    }

    /// One second of the update cycle.
    ///
    /// Written as a cascade of early returns rather than as arithmetic on a
    /// seconds count, because that is what the chip's counter chain does and
    /// because it keeps a guest-written nonsense field (`day = 31` in
    /// February) from becoming an out-of-range index.
    fn advance_second(&mut self) {
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
        // The chip counts the weekday in its own ring; it is not derived from
        // the date, which is why software may write it independently.
        self.weekday = self.weekday % 7 + 1;
        self.day += 1;
        if self.day <= Calendar::days_in_month(self.year, self.month) {
            return;
        }
        self.day = 1;
        self.month += 1;
        if self.month <= 12 {
            return;
        }
        self.month = 1;
        // Wrapping, because there is nowhere for a carry out of year 65535 to
        // go. A guest that gets there has other problems.
        self.year = self.year.wrapping_add(1);
    }
}

/// Parse `"YYYY-MM-DDTHH:MM:SS"`.
///
/// A space in place of the `T` is accepted, because people type it.
fn parse_time(text: &str) -> Result<Calendar> {
    let bad = || {
        Error::Property(format!(
            "property `time`: expected a date and time like \"{DEFAULT_TIME}\", found \"{text}\""
        ))
    };
    let bytes = text.as_bytes();
    if bytes.len() != 19 {
        return Err(bad());
    }
    let sep = |at: usize, want: u8| bytes[at] == want;
    if !sep(4, b'-') || !sep(7, b'-') || !(sep(10, b'T') || sep(10, b' ')) {
        return Err(bad());
    }
    if !sep(13, b':') || !sep(16, b':') {
        return Err(bad());
    }
    let field = |from: usize, to: usize| -> Option<u32> {
        let mut value = 0u32;
        for byte in &bytes[from..to] {
            if !byte.is_ascii_digit() {
                return None;
            }
            value = value * 10 + u32::from(byte - b'0');
        }
        Some(value)
    };
    let (Some(year), Some(month), Some(day), Some(hour), Some(minute), Some(second)) = (
        field(0, 4),
        field(5, 7),
        field(8, 10),
        field(11, 13),
        field(14, 16),
        field(17, 19),
    ) else {
        return Err(bad());
    };
    // Year 0 would make Zeller's congruence run off the bottom for January and
    // February, and a machine that wants one is not asking a real question.
    if !(1..=9999).contains(&year) || !(1..=12).contains(&month) {
        return Err(bad());
    }
    let year = year as u16;
    let month = month as u8;
    if day < 1 || day > u32::from(Calendar::days_in_month(year, month)) {
        return Err(Error::Property(format!(
            "property `time`: \"{text}\" names day {day} of a month with {} of them",
            Calendar::days_in_month(year, month)
        )));
    }
    if hour > 23 || minute > 59 || second > 59 {
        return Err(bad());
    }
    let day = day as u8;
    Ok(Calendar {
        second: second as u8,
        minute: minute as u8,
        hour: hour as u8,
        weekday: Calendar::weekday_of(year, month, day),
        day,
        month,
        year,
    })
}

/// Encode a two-digit value as BCD.
///
/// The chip has two digits and no third, so a value that could not fit is
/// reduced rather than allowed to overflow the nibble.
fn to_bcd(value: u8) -> u8 {
    let value = value % 100;
    ((value / 10) << 4) | (value % 10)
}

/// Decode a BCD byte. A non-decimal nibble decodes the way the adder would.
fn from_bcd(value: u8) -> u8 {
    (value >> 4) * 10 + (value & 0x0f)
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// The 128 bytes of CMOS, which is where status A, status B and the three
    /// alarm registers actually live. The calendar does not: it is in `now`,
    /// binary, and indices `0x00`, `0x02` and `0x04`-`0x09` of this array are
    /// unused.
    cmos: [u8; CMOS_BYTES],
    /// The index latch, **including** bit 7, the board's NMI mask.
    index: u8,
    now: Calendar,
    /// PF, AF and UF. IRQF is derived, never stored.
    flags: u8,
    /// Where the device stands in its own clock domain.
    tick: u64,
}

impl State {
    fn status_a(&self) -> u8 {
        self.cmos[REG_STATUS_A as usize]
    }

    fn status_b(&self) -> u8 {
        self.cmos[REG_STATUS_B as usize]
    }

    /// Whether the divider chain is configured for the crystal we have.
    fn running(&self) -> bool {
        self.status_a() & A_DIVIDER == A_DIVIDER_32KHZ
    }

    /// Whether the update cycle is inhibited by SET.
    fn frozen(&self) -> bool {
        self.status_b() & B_SET != 0
    }

    /// The periodic tap's period in domain ticks, or `None` when rate 0 turns
    /// it off.
    ///
    /// Rates 3-15 are `2^(rate-1)` cycles of 32.768 kHz — rate 3 is 4 cycles,
    /// 122.070 µs; rate 15 is 16384, half a second. Rates 1 and 2 are the two
    /// exceptions the data sheet spells out: 1/256 s and 1/128 s, which are 128
    /// and 256 cycles and would otherwise be 1 and 2.
    fn periodic_period(&self) -> Option<u64> {
        match self.status_a() & A_RATE {
            0 => None,
            1 => Some(TICKS_PER_SECOND / 256),
            2 => Some(TICKS_PER_SECOND / 128),
            rate => Some(1u64 << (rate - 1)),
        }
    }

    fn binary(&self) -> bool {
        self.status_b() & B_DM != 0
    }

    fn hour24(&self) -> bool {
        self.status_b() & B_24H != 0
    }

    /// A calendar field as the guest reads it.
    fn encode(&self, value: u8) -> u8 {
        if self.binary() { value } else { to_bcd(value) }
    }

    /// A calendar field as the guest wrote it.
    fn decode(&self, value: u8) -> u8 {
        if self.binary() {
            value
        } else {
            from_bcd(value)
        }
    }

    /// The hour as the guest reads it: 0-23, or 1-12 with bit 7 for PM.
    fn encode_hour(&self, hour: u8) -> u8 {
        if self.hour24() {
            return self.encode(hour);
        }
        let pm = hour >= 12;
        let twelve = match hour % 12 {
            0 => 12,
            h => h,
        };
        self.encode(twelve) | if pm { 0x80 } else { 0 }
    }

    /// The hour as the guest wrote it, back to 0-23.
    ///
    /// Note the two corners: 12 AM is hour 0, and 12 PM is hour 12, so the
    /// twelve is folded to zero *before* the twelve hours of the afternoon are
    /// added.
    fn decode_hour(&self, value: u8) -> u8 {
        if self.hour24() {
            return self.decode(value).min(23);
        }
        let pm = value & 0x80 != 0;
        let twelve = self.decode(value & 0x7f) % 12;
        if pm { twelve + 12 } else { twelve }
    }

    /// Whether one alarm byte matches, honouring the "don't care" encoding:
    /// both top bits set (`0xc0` and above) matches any value, which is how
    /// software asks for "every minute" or "every hour".
    fn alarm_field_matches(&self, raw: u8, actual: u8) -> bool {
        raw >= ALARM_DONT_CARE || self.decode(raw) == actual
    }

    /// Whether all three alarm registers match the current time.
    fn alarm_matches(&self) -> bool {
        let hours = self.cmos[REG_HOURS_ALARM as usize];
        self.alarm_field_matches(self.cmos[REG_SECONDS_ALARM as usize], self.now.second)
            && self.alarm_field_matches(self.cmos[REG_MINUTES_ALARM as usize], self.now.minute)
            && (hours >= ALARM_DONT_CARE || self.decode_hour(hours) == self.now.hour)
    }

    /// Whether the interrupt pin is asserted.
    fn irq(&self) -> bool {
        let b = self.status_b();
        (self.flags & C_PF != 0 && b & B_PIE != 0)
            || (self.flags & C_AF != 0 && b & B_AIE != 0)
            || (self.flags & C_UF != 0 && b & B_UIE != 0)
    }

    /// Status C as a read would produce it.
    fn status_c(&self) -> u8 {
        self.flags | if self.irq() { C_IRQF } else { 0 }
    }

    /// Whether UIP would read set right now.
    ///
    /// Not while the clock is stopped and not while SET freezes it: in both
    /// cases no update is coming, and software spinning on UIP would hang.
    fn uip(&self) -> bool {
        self.running()
            && !self.frozen()
            && self.tick % TICKS_PER_SECOND >= TICKS_PER_SECOND - UIP_TICKS
    }

    /// The next tick at which this device's own behaviour changes.
    ///
    /// Strictly greater than `tick`, as [`Device::next_event_tick`] requires,
    /// because both candidates are the *next* boundary rather than the current
    /// one.
    fn next_event(&self) -> Option<u64> {
        if !self.running() {
            return None;
        }
        let update = (self.tick / TICKS_PER_SECOND + 1) * TICKS_PER_SECOND;
        match self.periodic_period() {
            Some(period) => Some(update.min((self.tick / period + 1) * period)),
            None => Some(update),
        }
    }
}

/// The CMOS bytes firmware reads before it can do anything else.
#[derive(Debug, Clone, Copy)]
struct Seed {
    /// Base memory in kilobytes, for `0x15`/`0x16`.
    base_kib: u16,
    /// Extended memory in kilobytes, for `0x17`/`0x18` and `0x30`/`0x31`.
    ext_kib: u16,
    /// Memory above 16 MiB in 64 KiB units, for `0x34`/`0x35`.
    high_units: u16,
    equipment: u8,
    floppy: u8,
}

impl Default for Seed {
    fn default() -> Seed {
        Seed {
            base_kib: (DEFAULT_BASE_MEM / 1024) as u16,
            ext_kib: 0,
            high_units: 0,
            equipment: DEFAULT_EQUIPMENT,
            floppy: DEFAULT_FLOPPY,
        }
    }
}

/// Split extended memory across the two register pairs that report it.
///
/// Returns the kilobytes for `0x17`/`0x18` and `0x30`/`0x31`, and the 64 KiB
/// units above 16 MiB for `0x34`/`0x35`.
///
/// # Why a split and not one number
///
/// Neither pair is in the data sheet — `0x0e`-`0x3f` is 50 bytes of
/// general-purpose RAM as far as the MC146818 is concerned (data sheet, address
/// map), so all of this is BIOS convention. The convention is the one
/// `INT 15h AX=E801h` returns, and it is a *partition*, not a value and a
/// bigger value: the kilobyte pair describes the 1-16 MiB region only, capped
/// at 3C00h, and everything above 16 MiB is counted separately in 64 KiB
/// blocks. Firmware adds them. A model that saturated the kilobyte pair at
/// 0xff00 and left the block count at zero — which is what this did — reports
/// 63.75 MiB for any board with more than 64 MiB, and reports the 1-16 MiB
/// region twice for any board that also filled `0x34`/`0x35`.
///
/// Anything below a whole unit of either register is rounded **down**: a guest
/// told about memory that is not there is a fault, and one told about slightly
/// less than is there is not.
///
/// # Errors
///
/// [`Error::Property`] if `bytes` is more than the two pairs together can say.
/// Clamping would be a silent loss of gigabytes; the next rung of the same
/// convention is `0x5b`-`0x5d`, which nothing on these boards reads.
fn split_extended(bytes: u64) -> Result<(u16, u16)> {
    let window = bytes.min(EXT_MEM_WINDOW_KIB * 1024);
    let above = (bytes - window) / HIGH_MEM_UNIT;
    if above > u64::from(u16::MAX) {
        let max = EXT_MEM_WINDOW_KIB * 1024 + u64::from(u16::MAX) * HIGH_MEM_UNIT;
        return Err(Error::Property(format!(
            "property `extmem`: {bytes} bytes above 1 MiB does not fit the CMOS bytes that report \
             it; 0x17/0x18 and 0x30/0x31 hold the {EXT_MEM_WINDOW_KIB} KiB up to 16 MiB and \
             0x34/0x35 holds {} 64 KiB blocks above it, so at most {max} bytes",
            u16::MAX
        )));
    }
    Ok(((window / 1024) as u16, above as u16))
}

/// Write a little-endian 16-bit value into two CMOS bytes.
fn put16(cmos: &mut [u8; CMOS_BYTES], at: usize, value: u16) {
    cmos[at] = value as u8;
    cmos[at + 1] = (value >> 8) as u8;
}

/// Lay out the bytes the BIOS looks at, and check-sum them.
fn seed_cmos(seed: &Seed) -> [u8; CMOS_BYTES] {
    let mut cmos = [0u8; CMOS_BYTES];
    cmos[REG_STATUS_A as usize] = DEFAULT_STATUS_A;
    cmos[REG_STATUS_B as usize] = DEFAULT_STATUS_B;
    cmos[REG_STATUS_D as usize] = D_VRT;
    cmos[REG_FLOPPY] = seed.floppy;
    cmos[REG_EQUIPMENT] = seed.equipment;
    put16(&mut cmos, REG_BASE_MEM, seed.base_kib);
    put16(&mut cmos, REG_EXT_MEM, seed.ext_kib);
    // `0x30`/`0x31` are *extended* memory, not a second copy of base memory —
    // a confusion worth naming, because both pairs are two bytes of kilobytes
    // and the POST compares them against `0x17`/`0x18` to detect a size change.
    put16(&mut cmos, REG_EXT_MEM_MIRROR, seed.ext_kib);
    put16(&mut cmos, REG_HIGH_MEM, seed.high_units);
    // The AT's checksum: a plain 16-bit sum of `0x10` through `0x2d`, stored
    // high byte first at `0x2e`. The range is the AT's, and it is a BIOS
    // convention rather than anything the chip knows — which is why a machine
    // with a different firmware may checksum a different range and find this
    // one wrong. Nothing here recomputes it after a guest write: neither does
    // hardware, and firmware that changes a covered byte is expected to fix it.
    let sum =
        (CHECKSUM_FIRST..=CHECKSUM_LAST).fold(0u16, |acc, i| acc.wrapping_add(cmos[i].into()));
    cmos[REG_CHECKSUM] = (sum >> 8) as u8;
    cmos[REG_CHECKSUM + 1] = sum as u8;
    cmos
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

/// The pins this chip drives, once the machine has built them.
#[derive(Debug, Default)]
struct Outputs {
    /// The chip's own interrupt pin. IRQ8 on a PC.
    irq: Option<WireSource>,
    /// The board's NMI gate, high while NMI is disabled.
    nmi_mask: Option<WireSource>,
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The output pins, at [`LockRank::LEAF`] so a line can be driven with
    /// nothing else held.
    outs: Mutex<Outputs>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// Published so [`Device::current_tick`] can answer without a lock — the
    /// scheduler asks it with its own slot lock held at [`LockRank::LEAF`].
    tick: AtomicU64,
    /// The next internal event, or [`u64::MAX`] for none. Same no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("tick", &self.tick.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Republish the two lock-free numbers. Called with the state lock held.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        self.next_event
            .store(state.next_event().unwrap_or(u64::MAX), Ordering::Relaxed);
    }

    /// Drive the interrupt pin. Never called with the state lock held.
    fn drive_irq(&self, asserted: bool) {
        let out = self.outs.lock().irq.clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// Drive the NMI gate. Never called with the state lock held.
    fn drive_nmi_mask(&self, disabled: bool) {
        let out = self.outs.lock().nmi_mask.clone();
        if let Some(out) = out {
            out.set(Level::from_bool(disabled));
        }
    }

    /// Recompute and drive the interrupt line from the current state.
    fn refresh_irq(&self) {
        let asserted = self.state.lock().irq();
        self.drive_irq(asserted);
    }

    /// Catch up before an access, exactly as the CLINT does (§4.2).
    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        // A refusal means catch-up is already running further up the stack; the
        // access still has to be answered, and answering it from where the
        // device stands is the only defined thing to do.
        let _ = handle.sync(AccessKind::Guest);
    }

    /// Simulate forward to `target`.
    fn advance_to(&self, target: u64) {
        let asserted = {
            let mut state = self.state.lock();
            if target <= state.tick {
                // Running backwards is a no-op, not an error.
                return;
            }
            if state.running() {
                // The periodic flag is sticky, so however many taps fall inside
                // this step they collapse into one bit — which is why this is
                // a division rather than a loop, and why a guest that ignores
                // the interrupt for a second does not cost a second of work.
                if let Some(period) = state.periodic_period()
                    && target / period > state.tick / period
                {
                    state.flags |= C_PF;
                }
                // Updates cannot collapse: each one moves the calendar and each
                // one is a chance for the alarm to match. SET inhibits the
                // update cycle entirely — time stands still and no UF is
                // raised, which is the point of the bit.
                if !state.frozen() {
                    let mut boundary = state.tick / TICKS_PER_SECOND;
                    let last = target / TICKS_PER_SECOND;
                    while boundary < last {
                        boundary += 1;
                        state.now.advance_second();
                        state.flags |= C_UF;
                        if state.alarm_matches() {
                            state.flags |= C_AF;
                        }
                    }
                }
            }
            state.tick = target;
            self.republish(&state);
            state.irq()
        };
        self.drive_irq(asserted);
    }

    /// Read the selected CMOS register. `debug` suppresses every side effect.
    fn read_register(&self, debug: bool) -> u8 {
        let mut state = self.state.lock();
        let index = state.index & INDEX_MASK;
        match index {
            REG_SECONDS => state.encode(state.now.second),
            REG_MINUTES => state.encode(state.now.minute),
            REG_HOURS => state.encode_hour(state.now.hour),
            REG_WEEKDAY => state.encode(state.now.weekday),
            REG_DAY => state.encode(state.now.day),
            REG_MONTH => state.encode(state.now.month),
            REG_YEAR => state.encode((state.now.year % 100) as u8),
            REG_STATUS_A => {
                let uip = if state.uip() { A_UIP } else { 0 };
                (state.status_a() & !A_UIP) | uip
            }
            REG_STATUS_C => {
                let value = state.status_c();
                if !debug {
                    // The single most important side effect in the chip, and
                    // the reason a debugger must not take this path: it is the
                    // only acknowledgement there is.
                    state.flags = 0;
                }
                value
            }
            // VRT is a battery test, not a stored bit. We always have a battery.
            REG_STATUS_D => D_VRT,
            // Derived from the clock's own year rather than stored, so that the
            // leap-year rule has a full year to work with. See `Calendar::year`.
            REG_CENTURY => to_bcd((state.now.year / 100) as u8),
            // Status B, the three alarm registers, and 114 bytes of RAM.
            _ => state.cmos[index as usize],
        }
    }

    /// Write the selected CMOS register.
    ///
    /// Calendar fields are clamped to the range the counter chain can hold. The
    /// chip itself stores whatever it is given and behaves undefinedly
    /// afterwards; clamping keeps a guest-written `month = 0x99` from reaching
    /// the arithmetic above, and is the only place this model departs from
    /// "store it and see".
    fn write_register(&self, value: u8) {
        let asserted = {
            let mut state = self.state.lock();
            let index = state.index & INDEX_MASK;
            match index {
                REG_SECONDS => state.now.second = state.decode(value).min(59),
                REG_MINUTES => state.now.minute = state.decode(value).min(59),
                REG_HOURS => state.now.hour = state.decode_hour(value),
                REG_WEEKDAY => state.now.weekday = state.decode(value).clamp(1, 7),
                REG_DAY => state.now.day = state.decode(value).clamp(1, 31),
                REG_MONTH => state.now.month = state.decode(value).clamp(1, 12),
                REG_YEAR => {
                    let century = state.now.year / 100;
                    state.now.year = century * 100 + u16::from(state.decode(value) % 100);
                }
                // UIP is read-only; the rest of status A is the guest's.
                REG_STATUS_A => state.cmos[index as usize] = value & !A_UIP,
                REG_STATUS_B => state.cmos[index as usize] = value,
                // Both status registers are read-only. A write is swallowed
                // rather than faulted: firmware writes 0 to status C on the way
                // past more often than not.
                REG_STATUS_C | REG_STATUS_D => {}
                REG_CENTURY => {
                    // BCD whatever status B's DM bit says, because the byte is
                    // the BIOS's convention and the BIOS writes it in BCD. A
                    // value that is not two decimal digits is ignored, so a
                    // read still answers with the century the clock has.
                    let century = from_bcd(value);
                    if century <= 99 {
                        state.now.year = u16::from(century) * 100 + state.now.year % 100;
                    }
                }
                _ => state.cmos[index as usize] = value,
            }
            self.republish(&state);
            state.irq()
        };
        self.drive_irq(asserted);
    }

    /// Write the index latch, which is also the board's NMI gate.
    fn write_index(&self, value: u8) {
        {
            self.state.lock().index = value;
        }
        self.drive_nmi_mask(value & NMI_DISABLE != 0);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        if !attrs.debug {
            // A debug read must not move the clock any more than it may clear
            // status C (`ROADMAP.md` §15, invariant 5).
            self.sync();
        }
        *byte = if offset & 1 == 0 {
            INDEX_READS_AS
        } else {
            self.read_register(attrs.debug)
        };
        if !attrs.debug {
            self.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Even a write to the index latch is a side effect: it changes
            // which register a later guest read of offset 1 returns, and there
            // is no way to put it back.
            return Err(BusError::BadAccess);
        }
        self.sync();
        if offset & 1 == 0 {
            self.write_index(*value);
        } else {
            self.write_register(*value);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on two 8-bit ports. A 16-bit access would span the
        // index latch and the data port, which is not a thing the chip can
        // answer.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// A Motorola MC146818 real-time clock and its CMOS RAM.
#[derive(Debug)]
pub struct Rtc146818 {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Rtc146818 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `time` is not a date this calendar has, if a
    /// memory size will not fit the CMOS bytes that report it, or if a property
    /// this class does not know was given.
    pub fn new(props: &Props) -> Result<Rtc146818> {
        let mut r = props.reader();
        let time = r.or_str("time", DEFAULT_TIME)?;
        let base_mem = r.or_size("basemem", DEFAULT_BASE_MEM)?;
        let ext_mem = r.or_size("extmem", 0)?;
        let equipment = r.or_range("equipment", u64::from(DEFAULT_EQUIPMENT), 0..=255)?;
        let floppy = r.or_range("floppy", u64::from(DEFAULT_FLOPPY), 0..=255)?;
        let century: Option<u64> = r.optional("century")?;
        r.finish()?;

        let base_kib = base_mem / 1024;
        if base_kib > u64::from(u16::MAX) {
            return Err(Error::Property(format!(
                "property `basemem`: {base_kib} KiB does not fit the two CMOS bytes that report \
                 it; base memory is at most {} KiB",
                u16::MAX
            )));
        }
        // One size in, two register pairs out. `extmem` is *all* the memory
        // above 1 MiB, and the split between the pairs is arithmetic rather
        // than a second property, so the two cannot be made to disagree.
        let (ext_kib, high_units) = split_extended(ext_mem)?;

        let mut now = parse_time(time)?;
        if let Some(century) = century {
            if century > 99 {
                return Err(Error::Property(format!(
                    "property `century`: the CMOS century byte holds two digits, not {century}"
                )));
            }
            // The century byte *is* the top two digits of the clock's year, so
            // naming it explicitly moves the clock rather than seeding a byte
            // that could then disagree with the date.
            now.year = (century as u16) * 100 + now.year % 100;
            now.weekday = Calendar::weekday_of(now.year, now.month, now.day);
        }

        Ok(Rtc146818::build(
            now,
            &Seed {
                base_kib: base_kib as u16,
                ext_kib,
                high_units,
                equipment: equipment as u8,
                floppy: floppy as u8,
            },
        ))
    }

    /// One with the default CMOS seed and the default date.
    #[must_use]
    pub fn default_device() -> Rtc146818 {
        Rtc146818::at(DEFAULT_TIME).expect("the default time is a date this calendar has")
    }

    /// One with the default CMOS seed, starting at `time`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `time` is not a `"YYYY-MM-DDTHH:MM:SS"` date.
    pub fn at(time: &str) -> Result<Rtc146818> {
        Ok(Rtc146818::build(parse_time(time)?, &Seed::default()))
    }

    fn build(now: Calendar, seed: &Seed) -> Rtc146818 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    cmos: seed_cmos(seed),
                    index: 0,
                    now,
                    flags: 0,
                    tick: 0,
                },
            ),
            outs: Mutex::with_rank(LockRank::LEAF, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        {
            let state = regs.state.lock();
            regs.republish(&state);
        }
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rtc146818 { regs, region }
    }

    /// One byte of CMOS as the guest would read it, without side effects.
    ///
    /// Out-of-range indices read as zero. Index `0x0c` does **not** clear the
    /// flags here — this is the debug path.
    #[must_use]
    pub fn cmos(&self, index: u8) -> u8 {
        let saved = self.regs.state.lock().index;
        self.regs.state.lock().index = index & INDEX_MASK;
        let value = self.regs.read_register(true);
        self.regs.state.lock().index = saved;
        value
    }

    /// Whether the interrupt output is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq()
    }

    /// Whether the index latch's bit 7 currently masks NMI.
    #[must_use]
    pub fn nmi_disabled(&self) -> bool {
        self.regs.state.lock().index & NMI_DISABLE != 0
    }

    /// Advance to `tick` of the chip's own clock domain, updating the calendar
    /// and raising whatever flags that makes pending.
    ///
    /// This is what the scheduler calls; a test that is not running one calls
    /// it directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }
}

/// The `pc.rtc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "MC146818 real-time clock and CMOS RAM",
    properties: &[
        PropertySpec {
            name: "time",
            kind: ValueKind::Str,
            required: false,
            summary: "the date the clock starts at, \"YYYY-MM-DDTHH:MM:SS\" (never the host's)",
        },
        PropertySpec {
            name: "basemem",
            kind: ValueKind::Size,
            required: false,
            summary: "memory below 640K, reported in kilobytes at CMOS 0x15/0x16 (default 640K)",
        },
        PropertySpec {
            name: "extmem",
            kind: ValueKind::Size,
            required: false,
            summary: "all memory above 1M: kilobytes up to 16M at CMOS 0x17/0x18 and 0x30/0x31, \
                      the rest in 64K units at 0x34/0x35",
        },
        PropertySpec {
            name: "equipment",
            kind: ValueKind::Uint,
            required: false,
            summary: "the equipment byte at CMOS 0x14 (default 0x2d)",
        },
        PropertySpec {
            name: "floppy",
            kind: ValueKind::Uint,
            required: false,
            summary: "the floppy drive types byte at CMOS 0x10 (default 0x40, one 1.44M as A)",
        },
        PropertySpec {
            name: "century",
            kind: ValueKind::Uint,
            required: false,
            summary: "the century, read back from CMOS 0x32 as BCD (default: the one in `time`)",
        },
    ],
    construct: |props| Ok(Box::new(Rtc146818::new(props)?)),
};

impl Device for Rtc146818 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and the machine
        // file gives the device its 32.768 kHz clock domain.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The data sheet's reset table, and the reason this is not
        // `State::default()`: RESET clears PIE, AIE, UIE, SQWE and the status C
        // flags, and touches nothing else. SET, DM, 24/12 and DSE are
        // unaffected, status A is unaffected, and **the time and the RAM
        // survive** — there is a battery behind them, which is the entire point
        // of the part.
        {
            let mut state = self.regs.state.lock();
            state.cmos[REG_STATUS_B as usize] &= !B_RESET_CLEARS;
            state.flags = 0;
            // The index latch is ordinary board logic with no battery, so it
            // comes up clear — which also means NMI comes up enabled, and the
            // firmware masks it again on its own.
            state.index = 0;
            self.regs.republish(&state);
        }
        self.regs.drive_irq(false);
        self.regs.drive_nmi_mask(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let mut outs = self.regs.outs.lock();
        match port {
            "irq" => outs.irq = Some(source),
            "nmi_mask" => outs.nmi_mask = Some(source),
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from(
                        "an MC146818 drives `irq` and the board's `nmi_mask`; nothing else",
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        match port {
            "irq" => self.regs.refresh_irq(),
            "nmi_mask" => {
                let disabled = self.nmi_disabled();
                self.regs.drive_nmi_mask(disabled);
            }
            _ => {}
        }
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

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_bytes(&state.cmos)?;
        // The whole latch, NMI mask included: it is guest-visible through the
        // board's NMI gate, and a snapshot that dropped it would resume with
        // NMIs enabled that the guest had masked.
        w.write_u8(state.index)?;
        w.write_u8(state.now.second)?;
        w.write_u8(state.now.minute)?;
        w.write_u8(state.now.hour)?;
        w.write_u8(state.now.weekday)?;
        w.write_u8(state.now.day)?;
        w.write_u8(state.now.month)?;
        w.write_u16(state.now.year)?;
        w.write_u8(state.flags)?;
        // The device's own position in its domain. The scheduler restores the
        // domain; without this the two would disagree and the clock would stand
        // still until the domain caught up with it.
        w.write_u64(state.tick)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cmos = r.read_bytes()?;
        let cmos: [u8; CMOS_BYTES] = cmos.try_into().map_err(|_| {
            Error::State(format!(
                "snapshot has {} byte(s) of CMOS, this chip has {CMOS_BYTES}",
                r.remaining()
            ))
        })?;
        let index = r.read_u8()?;
        let now = Calendar {
            second: r.read_u8()?,
            minute: r.read_u8()?,
            hour: r.read_u8()?,
            weekday: r.read_u8()?,
            day: r.read_u8()?,
            month: r.read_u8()?,
            year: r.read_u16()?,
        };
        let flags = r.read_u8()?;
        let tick = r.read_u64()?;
        // Validated rather than trusted: the calendar arithmetic assumes these
        // ranges, and a corrupt snapshot must be an error rather than a march
        // through month 200.
        if now.second > 59
            || now.minute > 59
            || now.hour > 23
            || !(1..=7).contains(&now.weekday)
            || !(1..=31).contains(&now.day)
            || !(1..=12).contains(&now.month)
        {
            return Err(Error::State(format!(
                "snapshot holds an impossible date: {:04}-{:02}-{:02} {:02}:{:02}:{:02}, weekday {}",
                now.year, now.month, now.day, now.hour, now.minute, now.second, now.weekday
            )));
        }
        let (asserted, nmi) = {
            let mut state = self.regs.state.lock();
            *state = State {
                cmos,
                index,
                now,
                flags,
                tick,
            };
            self.regs.republish(&state);
            (state.irq(), state.index & NMI_DISABLE != 0)
        };
        self.regs.drive_irq(asserted);
        self.regs.drive_nmi_mask(nmi);
        Ok(())
    }
}

impl Instance for Rtc146818 {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rtc146818::new(props)?)))
}

/// What the validator should know about `pc.rtc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("time", ValueKind::Str))
        .prop(PropSchema::new("basemem", ValueKind::Size))
        .prop(PropSchema::new("extmem", ValueKind::Size))
        .prop(PropSchema::new("equipment", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("floppy", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("century", ValueKind::Uint).range(0, 99))
        .region("")
        .region("regs")
        .port("irq", PortDir::Out)
        .port("nmi_mask", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU32;
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    fn rtc() -> Rtc146818 {
        Rtc146818::default_device()
    }

    fn peek(d: &Rtc146818, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        d.regs
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(d: &Rtc146818, offset: u64, value: u8) {
        d.regs
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Read CMOS register `index` the way a guest does: index latch, then data.
    fn get(d: &Rtc146818, index: u8) -> u8 {
        poke(d, 0, index);
        peek(d, 1)
    }

    /// Write CMOS register `index` the way a guest does.
    fn set(d: &Rtc146818, index: u8, value: u8) {
        poke(d, 0, index);
        poke(d, 1, value);
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    /// A device with both output pins wired to probes.
    fn wired_at(time: &str) -> (Rtc146818, Arc<Probe>, Arc<Probe>) {
        let d = Rtc146818::at(time).expect("a date this calendar has");
        let ids = WireIdAllocator::new();
        let attach = |port: &str| {
            let id = ids.alloc();
            let probe = Arc::new(Probe::default());
            let wire = Wire::builder()
                .source(id)
                .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            d.connect(port, WireSource::new(wire, id))
                .expect("the chip drives this pin");
            probe
        };
        let irq = attach("irq");
        let nmi = attach("nmi_mask");
        (d, irq, nmi)
    }

    fn wired() -> (Rtc146818, Arc<Probe>, Arc<Probe>) {
        wired_at(DEFAULT_TIME)
    }

    /// The date as six BCD register reads, for a compact assertion.
    fn date(d: &Rtc146818) -> [u8; 7] {
        [
            get(d, REG_YEAR),
            get(d, REG_MONTH),
            get(d, REG_DAY),
            get(d, REG_HOURS),
            get(d, REG_MINUTES),
            get(d, REG_SECONDS),
            get(d, REG_WEEKDAY),
        ]
    }

    fn image(d: &Rtc146818) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("rtc", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rtc", CLASS.name, CLASS.version).unwrap();
            d.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn one_second_is_thirty_two_thousand_seven_hundred_and_sixty_eight_ticks() {
        let d = rtc();
        assert_eq!(get(&d, REG_SECONDS), 0x00);
        d.advance_to(TICKS_PER_SECOND - 1);
        assert_eq!(get(&d, REG_SECONDS), 0x00, "not yet");
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_SECONDS), 0x01);
        d.advance_to(TICKS_PER_SECOND * 59);
        assert_eq!(get(&d, REG_SECONDS), 0x59, "BCD, so 59 reads as 0x59");
        d.advance_to(TICKS_PER_SECOND * 60);
        assert_eq!(get(&d, REG_SECONDS), 0x00);
        assert_eq!(get(&d, REG_MINUTES), 0x01);
        // And running backwards is a no-op, not a rewind.
        d.advance_to(0);
        assert_eq!(get(&d, REG_MINUTES), 0x01);
    }

    #[test]
    fn the_calendar_carries_through_minutes_hours_days_months_and_years() {
        let d = Rtc146818::at("2026-12-31T23:59:59").unwrap();
        // 2026-12-31 was a Thursday: weekday 5.
        assert_eq!(date(&d), [0x26, 0x12, 0x31, 0x23, 0x59, 0x59, 5]);
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(
            date(&d),
            [0x27, 0x01, 0x01, 0x00, 0x00, 0x00, 6],
            "every field carried at once, and the weekday advanced with them"
        );
    }

    #[test]
    fn february_has_a_twenty_ninth_only_in_a_leap_year() {
        // Divisible by four: a leap year.
        let d = Rtc146818::at("2024-02-28T23:59:59").unwrap();
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_MONTH), 0x02);
        assert_eq!(get(&d, REG_DAY), 0x29);

        // A century not divisible by 400: not a leap year, and the two-digit
        // year register alone could not have told us — the century byte did.
        let d = Rtc146818::at("1900-02-28T23:59:59").unwrap();
        assert_eq!(get(&d, REG_CENTURY), 0x19);
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_MONTH), 0x03);
        assert_eq!(get(&d, REG_DAY), 0x01);

        // A century divisible by 400: a leap year after all, with the same
        // `00` in the year register.
        let d = Rtc146818::at("2000-02-28T23:59:59").unwrap();
        assert_eq!(get(&d, REG_YEAR), 0x00);
        assert_eq!(get(&d, REG_CENTURY), 0x20);
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_DAY), 0x29);

        // And the century byte follows the clock over the boundary.
        let d = Rtc146818::at("1999-12-31T23:59:59").unwrap();
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_YEAR), 0x00);
        assert_eq!(get(&d, REG_CENTURY), 0x20);
    }

    #[test]
    fn bcd_and_binary_read_back_what_was_written() {
        let d = rtc();
        // Default: BCD, 24-hour.
        set(&d, REG_HOURS, 0x17);
        set(&d, REG_MINUTES, 0x45);
        assert_eq!(get(&d, REG_HOURS), 0x17);
        assert_eq!(get(&d, REG_MINUTES), 0x45);

        // Binary, 24-hour. The stored time does not move; only the encoding of
        // it changes, which is what a driver switching modes relies on.
        set(&d, REG_STATUS_B, B_24H | B_DM);
        assert_eq!(get(&d, REG_HOURS), 17);
        assert_eq!(get(&d, REG_MINUTES), 45);
        set(&d, REG_HOURS, 23);
        assert_eq!(get(&d, REG_HOURS), 23);
    }

    #[test]
    fn twelve_hour_mode_carries_the_afternoon_in_bit_seven() {
        let d = rtc();
        set(&d, REG_STATUS_B, 0); // BCD, 12-hour.

        // 0x12 with bit 7 set is noon, not midnight.
        set(&d, REG_HOURS, 0x80 | 0x12);
        assert_eq!(get(&d, REG_HOURS), 0x92, "still 12 PM");
        set(&d, REG_STATUS_B, B_24H);
        assert_eq!(get(&d, REG_HOURS), 0x12, "which is hour 12");

        // And 0x12 without it is midnight.
        set(&d, REG_STATUS_B, 0);
        set(&d, REG_HOURS, 0x12);
        assert_eq!(get(&d, REG_HOURS), 0x12);
        set(&d, REG_STATUS_B, B_24H);
        assert_eq!(get(&d, REG_HOURS), 0x00, "which is hour 0");

        // One in the afternoon, in both notations.
        set(&d, REG_HOURS, 0x13);
        set(&d, REG_STATUS_B, 0);
        assert_eq!(get(&d, REG_HOURS), 0x81);

        // Binary 12-hour is a legal if unloved combination.
        set(&d, REG_STATUS_B, B_DM);
        assert_eq!(get(&d, REG_HOURS), 0x80 | 1);
    }

    #[test]
    fn the_periodic_flag_follows_the_rate_and_the_interrupt_follows_pie() {
        let (d, irq, _nmi) = wired();
        // Rate 15: 2^14 = 16384 ticks, half a second.
        set(&d, REG_STATUS_A, A_DIVIDER_32KHZ | 15);
        d.advance_to(16_383);
        assert_eq!(get(&d, REG_STATUS_C) & C_PF, 0, "not yet");
        d.advance_to(16_384);
        assert_eq!(get(&d, REG_STATUS_C) & C_PF, C_PF);
        assert!(!irq.high(), "the flag sets, but PIE is clear");

        // Rate 6: 32 ticks, 1024 Hz — the rate every BIOS programs.
        set(&d, REG_STATUS_A, A_DIVIDER_32KHZ | 6);
        set(&d, REG_STATUS_B, B_24H | B_PIE);
        assert_eq!(Device::next_event_tick(&d), Some(16_384 + 32));
        d.advance_to(16_384 + 31);
        assert!(!irq.high());
        d.advance_to(16_384 + 32);
        assert!(irq.high());
        assert_eq!(get(&d, REG_STATUS_C) & (C_IRQF | C_PF), C_IRQF | C_PF);
        assert!(!irq.high(), "and reading status C is the acknowledgement");

        // Rate 0 turns the tap off entirely.
        set(&d, REG_STATUS_A, A_DIVIDER_32KHZ);
        d.advance_to(16_384 + 32 + 4096);
        assert_eq!(get(&d, REG_STATUS_C) & C_PF, 0);
    }

    #[test]
    fn a_stopped_divider_stops_the_clock_and_the_taps() {
        let d = rtc();
        set(&d, REG_STATUS_A, 0x76); // divider held in reset, rate 6.
        assert_eq!(Device::next_event_tick(&d), None);
        d.advance_to(TICKS_PER_SECOND * 10);
        assert_eq!(get(&d, REG_SECONDS), 0x00, "the counter chain is stopped");
        assert_eq!(get(&d, REG_STATUS_C) & C_PF, 0);
        assert_eq!(get(&d, REG_STATUS_A) & A_UIP, 0, "and no update is coming");
    }

    #[test]
    fn the_set_bit_freezes_the_time_registers() {
        let d = rtc();
        set(&d, REG_STATUS_B, B_24H | B_SET);
        d.advance_to(TICKS_PER_SECOND * 5);
        assert_eq!(get(&d, REG_SECONDS), 0x00);
        assert_eq!(get(&d, REG_STATUS_C) & C_UF, 0, "no update ended");
        assert_eq!(get(&d, REG_STATUS_A) & A_UIP, 0, "and UIP stays clear");
        set(&d, REG_STATUS_B, B_24H);
        d.advance_to(TICKS_PER_SECOND * 6);
        assert_eq!(get(&d, REG_SECONDS), 0x01, "and it resumes where it stood");
    }

    #[test]
    fn uip_rises_two_hundred_and_forty_four_microseconds_before_an_update() {
        let d = rtc();
        d.advance_to(TICKS_PER_SECOND - UIP_TICKS - 1);
        assert_eq!(get(&d, REG_STATUS_A) & A_UIP, 0);
        d.advance_to(TICKS_PER_SECOND - UIP_TICKS);
        assert_eq!(get(&d, REG_STATUS_A) & A_UIP, A_UIP);
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_STATUS_A) & A_UIP, 0, "the update is done");
    }

    #[test]
    fn the_alarm_fires_on_a_match_and_dont_care_matches_anything() {
        let (d, irq, _nmi) = wired();
        set(&d, REG_SECONDS_ALARM, 0x05);
        set(&d, REG_MINUTES_ALARM, 0x00);
        set(&d, REG_HOURS_ALARM, 0x00);
        set(&d, REG_STATUS_B, B_24H | B_AIE);
        d.advance_to(TICKS_PER_SECOND * 4);
        assert!(!irq.high());
        d.advance_to(TICKS_PER_SECOND * 5);
        assert!(irq.high(), "00:00:05");
        assert_eq!(get(&d, REG_STATUS_C) & C_AF, C_AF);
        assert!(!irq.high());

        // 0xc0 and above in every field: every second matches.
        set(&d, REG_SECONDS_ALARM, 0xff);
        set(&d, REG_MINUTES_ALARM, 0xc0);
        set(&d, REG_HOURS_ALARM, 0xc0);
        d.advance_to(TICKS_PER_SECOND * 6);
        assert!(irq.high());
        assert_eq!(get(&d, REG_STATUS_C) & C_AF, C_AF);

        // Don't care in the seconds alone is the "once a minute" alarm.
        set(&d, REG_SECONDS_ALARM, 0xc0);
        set(&d, REG_MINUTES_ALARM, 0x01);
        set(&d, REG_HOURS_ALARM, 0x00);
        d.advance_to(TICKS_PER_SECOND * 59);
        assert_eq!(get(&d, REG_STATUS_C) & C_AF, 0, "still minute 0");
        d.advance_to(TICKS_PER_SECOND * 60);
        assert_eq!(get(&d, REG_STATUS_C) & C_AF, C_AF, "minute 1");
    }

    #[test]
    fn the_update_ended_flag_needs_uie_to_reach_the_pin() {
        let (d, irq, _nmi) = wired();
        d.advance_to(TICKS_PER_SECOND);
        assert!(!irq.high());
        assert_eq!(get(&d, REG_STATUS_C) & C_UF, C_UF);
        set(&d, REG_STATUS_B, B_24H | B_UIE);
        d.advance_to(TICKS_PER_SECOND * 2);
        assert!(irq.high());
        assert_eq!(get(&d, REG_STATUS_C) & (C_IRQF | C_UF), C_IRQF | C_UF);
        assert!(!irq.high());
    }

    #[test]
    fn a_debug_read_of_status_c_eats_nothing() {
        let (d, irq, _nmi) = wired();
        set(&d, REG_STATUS_B, B_24H | B_UIE);
        d.advance_to(TICKS_PER_SECOND);
        assert!(irq.high());

        // Select status C with an ordinary write, then look at it the way a
        // debugger would.
        poke(&d, 0, REG_STATUS_C);
        let mut byte = [0u8; 1];
        d.regs
            .read(1, &mut byte, MemAttrs::DEBUG)
            .expect("a debugger may look");
        assert_eq!(byte[0] & (C_IRQF | C_UF), C_IRQF | C_UF);
        assert!(irq.high(), "the guest's interrupt is still there");
        assert_eq!(
            get(&d, REG_STATUS_C) & C_UF,
            C_UF,
            "and the flag was not eaten"
        );
        assert!(!irq.high(), "only the real read acknowledges");
    }

    #[test]
    fn the_index_latch_carries_the_nmi_mask() {
        let (d, _irq, nmi) = wired();
        poke(&d, 0, REG_STATUS_D);
        assert!(!nmi.high(), "NMI enabled");
        assert!(!d.nmi_disabled());
        assert_eq!(peek(&d, 1), D_VRT, "and the low bits still select");

        poke(&d, 0, NMI_DISABLE | REG_STATUS_D);
        assert!(nmi.high(), "NMI masked");
        assert!(d.nmi_disabled());
        assert_eq!(peek(&d, 1), D_VRT, "the same register, still");

        poke(&d, 0, REG_STATUS_D);
        assert!(!nmi.high());
    }

    #[test]
    fn the_index_port_reads_as_ones() {
        let d = rtc();
        poke(&d, 0, 0x0a);
        assert_eq!(peek(&d, 0), INDEX_READS_AS, "the latch is write-only");
    }

    #[test]
    fn the_seeded_checksum_is_the_sum_of_the_at_range() {
        let d = rtc();
        let sum = (CHECKSUM_FIRST..=CHECKSUM_LAST)
            .fold(0u16, |acc, i| acc.wrapping_add(u16::from(d.cmos(i as u8))));
        let stored = u16::from(d.cmos(REG_CHECKSUM as u8)) << 8 | u16::from(d.cmos(0x2f));
        assert_eq!(sum, stored);
        // And the seed is actually in there, or the checksum would be over
        // 30 zero bytes and prove nothing.
        assert_eq!(d.cmos(REG_FLOPPY as u8), DEFAULT_FLOPPY);
        assert_eq!(d.cmos(REG_EQUIPMENT as u8), DEFAULT_EQUIPMENT);
        assert_eq!(d.cmos(REG_BASE_MEM as u8), 640u16 as u8);
        assert_eq!(d.cmos(REG_BASE_MEM as u8 + 1), (640u16 >> 8) as u8);
        assert_ne!(sum, 0);
    }

    /// What a firmware adds up when it wants the size of extended memory:
    /// the kilobytes at `0x30`/`0x31` plus the 64 KiB blocks at `0x34`/`0x35`.
    ///
    /// This is `src/fw/pcbios`'s own arithmetic — `post.rs` builds the E820
    /// entry for memory above 1 MiB exactly this way, and `system.rs` answers
    /// `INT 15h AX=E801h` out of the same two pairs. It is not this model's
    /// convention to choose, which is why the sum is written out here rather
    /// than being asserted against a number the model computed.
    fn extended_as_firmware_sees_it(d: &Rtc146818) -> u64 {
        let kib = u64::from(d.cmos(REG_EXT_MEM_MIRROR as u8))
            | u64::from(d.cmos(REG_EXT_MEM_MIRROR as u8 + 1)) << 8;
        let blocks =
            u64::from(d.cmos(REG_HIGH_MEM as u8)) | u64::from(d.cmos(REG_HIGH_MEM as u8 + 1)) << 8;
        kib * 1024 + blocks * HIGH_MEM_UNIT
    }

    #[test]
    fn the_memory_properties_land_where_the_bios_looks_for_them() {
        let d = Rtc146818::new(
            &Props::new()
                .with("extmem", Value::Size(64 * 1024 * 1024))
                .with("time", "1999-12-31T23:59:59"),
        )
        .expect("a legal configuration");
        // The kilobyte pair describes the 1-16 MiB region and stops there.
        let ext = u16::from(d.cmos(REG_EXT_MEM as u8)) | u16::from(d.cmos(0x18)) << 8;
        assert_eq!(u64::from(ext), EXT_MEM_WINDOW_KIB);
        let mirror = u16::from(d.cmos(REG_EXT_MEM_MIRROR as u8)) | u16::from(d.cmos(0x31)) << 8;
        assert_eq!(ext, mirror, "0x30/0x31 is extended memory, not base memory");
        // And the 49 MiB above 16 MiB is in the block pair, so the sum comes
        // back out whole.
        let high = u16::from(d.cmos(REG_HIGH_MEM as u8)) | u16::from(d.cmos(0x35)) << 8;
        assert_eq!(u64::from(high) * HIGH_MEM_UNIT, 49 * 1024 * 1024);
        assert_eq!(extended_as_firmware_sees_it(&d), 64 * 1024 * 1024);
        assert_eq!(d.cmos(REG_CENTURY), 0x19);

        // The set `machines/pc-at.machine` actually passes, so that a change
        // here is caught beside the code rather than in board validation.
        let board = Rtc146818::new(
            &Props::new()
                .with("time", DEFAULT_TIME)
                .with("basemem", Value::Size(640 * 1024))
                .with("extmem", Value::Size(15 * 1024 * 1024)),
        )
        .expect("the board's configuration");
        let ext = u16::from(board.cmos(REG_EXT_MEM as u8)) | u16::from(board.cmos(0x18)) << 8;
        assert_eq!(ext, 15 * 1024);
        assert_eq!(board.cmos(REG_HIGH_MEM as u8), 0, "nothing is above 16 MiB");
        assert_eq!(board.cmos(REG_BASE_MEM as u8), 640u16 as u8);
    }

    /// The q35 board's 128 MiB, which is where this was found: firmware read
    /// the CMOS and reported `RamSize: 0x040c0000` — 64 MiB + 768 KiB — because
    /// the kilobyte pair had saturated at 0xff00 and `0x34`/`0x35` was zero.
    #[test]
    fn a_board_with_more_than_64_mib_reports_all_of_it() {
        for extmem in [
            15 * 1024 * 1024,       // exactly the window, nothing above
            16 * 1024 * 1024,       // one megabyte over the 16 MiB line
            64 * 1024 * 1024,       // where the old saturating cap bit
            128 * 1024 * 1024,      // `machines/q35.machine`
            2 * 1024 * 1024 * 1024, // and a board a kilobyte pair cannot say
        ] {
            let d = Rtc146818::new(
                &Props::new()
                    .with("time", DEFAULT_TIME)
                    .with("extmem", Value::Size(extmem)),
            )
            .expect("a legal configuration");
            assert_eq!(
                extended_as_firmware_sees_it(&d),
                extmem,
                "a board with {extmem} bytes above 1 MiB must not lose any of it in the CMOS"
            );
        }
    }

    #[test]
    fn extended_memory_the_two_pairs_cannot_say_is_refused_rather_than_clamped() {
        let max = EXT_MEM_WINDOW_KIB * 1024 + u64::from(u16::MAX) * HIGH_MEM_UNIT;
        assert!(split_extended(max).is_ok());
        assert!(
            Rtc146818::new(&Props::new().with("extmem", Value::Size(max + HIGH_MEM_UNIT))).is_err(),
            "silently dropping 4 GiB of a guest's memory is worse than refusing to boot"
        );
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        assert!(Rtc146818::new(&Props::new().with("time", "2026-02-30T00:00:00")).is_err());
        assert!(Rtc146818::new(&Props::new().with("time", "yesterday")).is_err());
        assert!(Rtc146818::new(&Props::new().with("time", "2026-01-01T24:00:00")).is_err());
        assert!(Rtc146818::new(&Props::new().with("century", 100u64)).is_err());
        assert!(
            Rtc146818::new(&Props::new().with("basemme", Value::Size(1024))).is_err(),
            "a typo is not silently ignored"
        );
        let d = Rtc146818::new(&Props::new().with("time", "2026-01-01 12:34:56"))
            .expect("a space for the T");
        assert_eq!(get(&d, REG_HOURS), 0x12);
    }

    #[test]
    fn an_access_the_chip_cannot_answer_is_refused() {
        let d = rtc();
        assert!(d.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(d.regs.write(1, &[0u8; 2], MemAttrs::DEFAULT).is_err());
        // A debug write cannot be made harmless: even the index latch changes
        // what a later real read returns.
        assert!(d.regs.write(0, &[0x0a], MemAttrs::DEBUG).is_err());
        assert!(d.regs.write(1, &[0x00], MemAttrs::DEBUG).is_err());
    }

    #[test]
    fn a_reset_clears_the_enables_and_keeps_the_battery_backed_state() {
        let (d, irq, nmi) = wired();
        set(&d, 0x40, 0xa5); // some RAM the BIOS owns
        set(&d, REG_STATUS_B, B_24H | B_PIE | B_UIE);
        poke(&d, 0, NMI_DISABLE | 0x0a);
        d.advance_to(TICKS_PER_SECOND);
        assert!(irq.high());
        assert!(nmi.high());

        d.reset(ResetKind::Cold);
        assert!(!irq.high());
        assert!(!nmi.high(), "the latch has no battery");
        assert_eq!(get(&d, REG_STATUS_B), B_24H, "the enables are cleared");
        assert_eq!(get(&d, REG_SECONDS), 0x01, "but the time survives");
        assert_eq!(get(&d, 0x40), 0xa5, "and so does the RAM");
    }

    #[test]
    fn a_snapshot_round_trips_byte_for_byte() {
        let saved = rtc();
        saved.advance_to(TICKS_PER_SECOND * 3661 + 777);
        set(&saved, REG_STATUS_B, B_24H | B_PIE | B_AIE);
        set(&saved, REG_STATUS_A, A_DIVIDER_32KHZ | 4);
        set(&saved, REG_SECONDS_ALARM, 0xc0);
        set(&saved, 0x37, 0x5a);
        poke(&saved, 0, NMI_DISABLE | REG_STATUS_B);
        // Leave a flag pending, so the round trip has to carry one.
        saved.advance_to(TICKS_PER_SECOND * 3662);
        assert!(saved.irq_asserted());

        let first = image(&saved);

        let restored = rtc();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("rtc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(image(&restored), first, "the two images are identical");
        // Before anything that moves the index latch, which every guest-style
        // register read below does.
        assert!(restored.nmi_disabled(), "the mask bit came back too");
        assert!(restored.irq_asserted(), "and so did the pending interrupt");
        assert_eq!(date(&restored), date(&saved));
        assert_eq!(
            Device::current_tick(&restored),
            Device::current_tick(&saved)
        );
        assert_eq!(restored.cmos(0x37), 0x5a);
    }

    #[test]
    fn a_snapshot_holding_an_impossible_date_is_refused() {
        // Hand-written rather than corrupted, so the test says exactly which
        // field is wrong: the calendar arithmetic assumes these ranges and a
        // bad snapshot must be an error rather than a march through month 200.
        let mut shape = MachineShape::new();
        shape.add_device("rtc", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rtc", CLASS.name, CLASS.version).unwrap();
            chunk.write_bytes(&[0u8; CMOS_BYTES]).unwrap();
            for byte in [0u8, 0, 0, 0, 1, 1, 200] {
                chunk.write_u8(byte).unwrap();
            }
            chunk.write_u16(2026).unwrap();
            chunk.write_u8(0).unwrap();
            chunk.write_u64(0).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = rtc();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rtc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        let e = restored
            .load(&mut chunk.reader())
            .expect_err("month 200 is not a month")
            .to_string();
        assert!(e.contains("impossible date"), "{e}");
    }

    #[test]
    fn the_next_event_is_the_sooner_of_the_tap_and_the_update() {
        let d = rtc();
        // Rate 15 is 16384 ticks, which is sooner than the update at 32768.
        set(&d, REG_STATUS_A, A_DIVIDER_32KHZ | 15);
        assert_eq!(Device::next_event_tick(&d), Some(16_384));
        d.advance_to(16_384);
        assert_eq!(
            Device::next_event_tick(&d),
            Some(TICKS_PER_SECOND),
            "the update is next, and the one that fired is not reported again"
        );
        // With no tap at all, the update is the only event there is.
        set(&d, REG_STATUS_A, A_DIVIDER_32KHZ);
        assert_eq!(Device::next_event_tick(&d), Some(TICKS_PER_SECOND));
        assert!(
            Device::next_event_tick(&d).unwrap() > Device::current_tick(&d),
            "and it is always in the future, or catch-up would stall"
        );
    }

    #[test]
    fn daylight_saving_is_stored_and_deliberately_does_nothing() {
        // The data sheet's DSE moves the clock forward an hour on the last
        // Sunday in April and back on the last Sunday in October — two dates
        // hard-wired into 1970s American silicon and wrong everywhere since.
        // No PC BIOS sets the bit, so the bit is stored and the transitions are
        // not modelled; a guest that sets it gets it back.
        let d = rtc();
        set(&d, REG_STATUS_B, B_24H | B_DSE);
        assert_eq!(get(&d, REG_STATUS_B), B_24H | B_DSE);
        d.advance_to(TICKS_PER_SECOND);
        assert_eq!(get(&d, REG_SECONDS), 0x01, "and the clock is unaffected");
    }

    #[test]
    fn the_cmos_ram_is_ordinary_memory() {
        let d = rtc();
        for index in [0x0eu8, 0x33, 0x7f] {
            set(&d, index, 0x5a);
            assert_eq!(get(&d, index), 0x5a);
        }
        // And the index latch wraps at 7 bits, so 0x8e is 0x0e.
        set(&d, 0x0e, 0x11);
        poke(&d, 0, NMI_DISABLE | 0x0e);
        assert_eq!(peek(&d, 1), 0x11);
    }
}
