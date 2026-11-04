//! An Epson RX-8581 real-time clock on I²C: a BCD calendar that counts on its
//! own 32.768 kHz crystal.
//!
//! The navi's RTC (the board calls it an RA-8581SA). Nothing in its kernel
//! drives it; the GPS library reads it for the date, so the calendar is what
//! matters. The alarm, the countdown timer and the `/INT` output are kept as
//! storage and not run.
//!
//! # Registers
//!
//! Seven-bit address `0x51`. A write's first byte sets the register pointer;
//! every byte after it, read or written, moves the pointer on, and `0x0F`
//! wraps to `0x00`.
//!
//! | Reg | Here |
//! | --- | --- |
//! | `00`–`02` | seconds, minutes, hours (24-hour), BCD |
//! | `03` | weekday, one bit per day: bit 0 is Sunday |
//! | `04`–`06` | day, month, year (`00`–`99`, the 2000s), BCD |
//! | `07` | a free RAM byte |
//! | `08`–`0C` | alarm and timer: storage |
//! | `0D` | extension register: storage |
//! | `0E` | flags: `UF` 5, `TF` 4, `AF` 3, `VLF` 1. A flag is cleared by writing 0; writing 1 leaves it |
//! | `0F` | control: `UIE` 5, `TIE` 4, `AIE` 3, `STOP` 1, `RESET` 0. `STOP` holds the count |
//!
//! Writing the seconds also clears the part below a second, so a clock set
//! on the second ticks a whole second later.
//!
//! A read takes its time registers from one instant: the chip holds its
//! counter while it is being read, so a ten-byte read across a minute
//! boundary cannot mix two minutes.
//!
//! # The time it starts at
//!
//! `time` gives the date the crystal starts counting from, as
//! `"YYYY-MM-DDTHH:MM:SS"`, and only the device's clock moves it on, so a
//! run is reproducible. Without it the clock powers up as from zero volts:
//! `2000-01-01 00:00:00` with `VLF` set, which software reads as "the time is
//! not valid".
//!
//! # Sources
//!
//! Epson's RX-8581 SA/JE/NB application manual: the register table, the
//! flag and control bits, the address auto-increment and wrap, and the hold
//! during access.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "epson.rx8581";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The crystal: device-clock ticks in one second.
pub const TICKS_PER_SECOND: u64 = 32_768;

const REG_SEC: u8 = 0x00;
const REG_MIN: u8 = 0x01;
const REG_HOUR: u8 = 0x02;
const REG_WEEK: u8 = 0x03;
const REG_DAY: u8 = 0x04;
const REG_MONTH: u8 = 0x05;
const REG_YEAR: u8 = 0x06;
const REG_FLAG: u8 = 0x0e;
const REG_CONTROL: u8 = 0x0f;

/// `VLF`: the supply fell low enough that the time may be wrong.
const FLAG_VLF: u8 = 1 << 1;
/// The flags software can see.
const FLAG_MASK: u8 = 0b0011_1011;
/// `STOP`: the count is held.
const CONTROL_STOP: u8 = 1 << 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Calendar {
    second: u8,
    minute: u8,
    hour: u8,
    /// 0 = Sunday … 6 = Saturday.
    weekday: u8,
    day: u8,
    month: u8,
    /// Two digits: the 2000s.
    year: u8,
}

impl Calendar {
    const POWER_ON: Calendar = Calendar {
        second: 0,
        minute: 0,
        hour: 0,
        // 2000-01-01 was a Saturday.
        weekday: 6,
        day: 1,
        month: 1,
        year: 0,
    };

    fn days_in_month(&self) -> u8 {
        match self.month {
            // Every year the chip can count is in 2000–2099, where the
            // fourth ones are all leap years.
            2 if self.year.is_multiple_of(4) => 29,
            2 => 28,
            4 | 6 | 9 | 11 => 30,
            _ => 31,
        }
    }

    /// One second, as the counter chain carries it.
    fn tick(&mut self) {
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
        self.weekday = (self.weekday + 1) % 7;
        self.day += 1;
        if self.day <= self.days_in_month() {
            return;
        }
        self.day = 1;
        self.month += 1;
        if self.month <= 12 {
            return;
        }
        self.month = 1;
        self.year = (self.year + 1) % 100;
    }

    /// The day of the week of a date in the 2000s, 0 = Sunday.
    fn weekday_of(year: u16, month: u8, day: u8) -> u8 {
        // Sakamoto's method.
        const T: [u16; 12] = [0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4];
        let y = if month < 3 { year - 1 } else { year };
        ((y + y / 4 - y / 100 + y / 400 + T[usize::from(month - 1)] + u16::from(day)) % 7) as u8
    }

    fn register(&self, reg: u8) -> u8 {
        match reg {
            REG_SEC => bcd(self.second),
            REG_MIN => bcd(self.minute),
            REG_HOUR => bcd(self.hour),
            REG_WEEK => 1 << self.weekday,
            REG_DAY => bcd(self.day),
            REG_MONTH => bcd(self.month),
            REG_YEAR => bcd(self.year),
            _ => 0,
        }
    }

    /// A time register written. Out-of-range values are stored as they come,
    /// as the counters would hold them, and roll over at the next carry.
    fn set(&mut self, reg: u8, v: u8) {
        match reg {
            REG_SEC => self.second = binary(v & 0x7f),
            REG_MIN => self.minute = binary(v & 0x7f),
            REG_HOUR => self.hour = binary(v & 0x3f),
            REG_WEEK => {
                self.weekday = (v & 0x7f).trailing_zeros().min(6) as u8;
            }
            REG_DAY => self.day = binary(v & 0x3f),
            REG_MONTH => self.month = binary(v & 0x1f),
            REG_YEAR => self.year = binary(v),
            _ => {}
        }
    }
}

fn bcd(v: u8) -> u8 {
    (v / 10) << 4 | (v % 10)
}

fn binary(v: u8) -> u8 {
    (v >> 4) * 10 + (v & 0xf)
}

/// Parse `"YYYY-MM-DDTHH:MM:SS"` (a space for the `T` is fine), a year in
/// 2000–2099.
fn parse_time(text: &str) -> Result<Calendar> {
    let bad = || {
        Error::Property(format!(
            "property `time`: expected a date like \"2026-01-01T00:00:00\" in 2000-2099, found \"{text}\""
        ))
    };
    let b = text.as_bytes();
    if b.len() != 19
        || b[4] != b'-'
        || b[7] != b'-'
        || !(b[10] == b'T' || b[10] == b' ')
        || b[13] != b':'
        || b[16] != b':'
    {
        return Err(bad());
    }
    let field = |from: usize, to: usize| -> Option<u16> {
        b[from..to].iter().try_fold(0u16, |acc, c| {
            c.is_ascii_digit().then(|| acc * 10 + u16::from(c - b'0'))
        })
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
    if !(2000..=2099).contains(&year)
        || !(1..=12).contains(&month)
        || hour > 23
        || minute > 59
        || second > 59
    {
        return Err(bad());
    }
    let mut c = Calendar {
        second: second as u8,
        minute: minute as u8,
        hour: hour as u8,
        weekday: 0,
        day: day as u8,
        month: month as u8,
        year: (year - 2000) as u8,
    };
    if day == 0 || c.day > c.days_in_month() {
        return Err(bad());
    }
    c.weekday = Calendar::weekday_of(year, c.month, c.day);
    Ok(c)
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    now: Calendar,
    /// Ticks into the current second.
    fraction: u64,
    /// The non-time registers, indexed by register number.
    regs: [u8; 16],
    ptr: u8,
    /// The next written byte selects the register.
    select: bool,
    /// The calendar a read in progress sees.
    held: Option<Calendar>,
}

impl State {
    fn new(start: Option<Calendar>) -> State {
        let mut regs = [0u8; 16];
        if start.is_none() {
            regs[usize::from(REG_FLAG)] = FLAG_VLF;
        }
        State {
            now: start.unwrap_or(Calendar::POWER_ON),
            fraction: 0,
            regs,
            ptr: 0,
            select: false,
            held: None,
        }
    }

    fn read(&self) -> u8 {
        let reg = self.ptr;
        match reg {
            REG_SEC..=REG_YEAR => self.held.unwrap_or(self.now).register(reg),
            _ => self.regs[usize::from(reg)],
        }
    }

    fn write(&mut self, v: u8) {
        let reg = self.ptr;
        match reg {
            REG_SEC..=REG_YEAR => {
                self.now.set(reg, v);
                if reg == REG_SEC {
                    self.fraction = 0;
                }
            }
            REG_FLAG => {
                // Write 0 to clear; a 1 leaves the flag as it was.
                self.regs[usize::from(reg)] &= v | !FLAG_MASK;
            }
            _ => self.regs[usize::from(reg)] = v,
        }
        self.step();
    }

    fn step(&mut self) {
        self.ptr = (self.ptr + 1) & 0x0f;
    }

    fn advance(&mut self, ticks: u64) {
        if self.regs[usize::from(REG_CONTROL)] & CONTROL_STOP != 0 {
            return;
        }
        self.fraction += ticks;
        while self.fraction >= TICKS_PER_SECOND {
            self.fraction -= TICKS_PER_SECOND;
            self.now.tick();
        }
    }
}

struct Target {
    address: u8,
    state: Mutex<State>,
}

impl fmt::Debug for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "epson.rx8581@{:#x}", self.address)
    }
}

impl I2cSlave for Target {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        if address != Address::Seven(self.address) {
            return Ack::Nack;
        }
        let mut s = self.state.lock();
        s.select = dir == Direction::Write;
        // The counters hold for the length of the access.
        s.held = Some(s.now);
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        let mut s = self.state.lock();
        if s.select {
            s.select = false;
            s.ptr = byte & 0x0f;
        } else {
            s.write(byte);
        }
        Ack::Ack
    }

    fn read(&self) -> u8 {
        self.state.lock().read()
    }

    fn read_ack(&self, _ack: Ack) {
        self.state.lock().step();
    }

    fn stop(&self) {
        let mut s = self.state.lock();
        s.select = false;
        s.held = None;
    }
}

/// The real-time clock.
#[derive(Debug)]
pub struct Rx8581 {
    target: Arc<Target>,
    bus: Option<Arc<I2cBus>>,
    start: Option<Calendar>,
}

impl Rx8581 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property, or a
    /// `time` this chip cannot hold.
    pub fn new(props: &Props) -> Result<Rx8581> {
        let mut r = props.reader();
        let bus = r.require_str("bus")?;
        let address = r.or_range("address", 0x51u64, 0..=0x7f)? as u8;
        let start = match r.optional_str("time")? {
            Some(t) if !t.is_empty() => Some(parse_time(t)?),
            _ => None,
        };
        r.finish()?;
        let bus = buses::attach(props, bus)?;
        Ok(Rx8581::with_bus(Some(bus), address, start))
    }

    fn with_bus(bus: Option<Arc<I2cBus>>, address: u8, start: Option<Calendar>) -> Rx8581 {
        Rx8581 {
            target: Arc::new(Target {
                address,
                state: Mutex::with_rank(LockRank::DEVICE, State::new(start)),
            }),
            bus,
            start,
        }
    }

    /// The bus-facing side.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.target) as Arc<dyn I2cSlave>
    }
}

/// The device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Epson RX-8581 I2C real-time clock: the BCD calendar on its own crystal",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the I2C bus it sits on, by name",
        },
        PropertySpec {
            name: "address",
            kind: ValueKind::Uint,
            required: false,
            summary: "its seven-bit address (default 0x51)",
        },
        PropertySpec {
            name: "time",
            kind: ValueKind::Str,
            required: false,
            summary: "the date the crystal starts from, YYYY-MM-DDTHH:MM:SS; empty powers up invalid",
        },
    ],
    construct: |props| Ok(Box::new(Rx8581::new(props)?)),
};

impl Device for Rx8581 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The chip has its own backup supply: only a cold start (the first
        // power-up) puts the clock back.
        if kind == ResetKind::Cold {
            *self.target.state.lock() = State::new(self.start);
        } else {
            let mut s = self.target.state.lock();
            s.select = false;
            s.held = None;
        }
    }

    fn is_runnable(&self) -> bool {
        // To count: the crystal is its clock domain.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.target.state.lock().advance(budget.ticks);
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.target.state.lock().clone();
        write_calendar(w, s.now)?;
        w.write_u64(s.fraction)?;
        w.write_bytes(&s.regs)?;
        w.write_u8(s.ptr)?;
        w.write_bool(s.select)?;
        w.write_bool(s.held.is_some())?;
        write_calendar(w, s.held.unwrap_or(s.now))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let now = read_calendar(r)?;
        let fraction = r.read_u64()?;
        let bytes = r.read_bytes()?;
        if bytes.len() != 16 {
            return Err(Error::State(String::from(
                "an RX-8581 snapshot with a different register file",
            )));
        }
        let mut regs = [0u8; 16];
        regs.copy_from_slice(bytes);
        let ptr = r.read_u8()? & 0x0f;
        let select = r.read_bool()?;
        let holding = r.read_bool()?;
        let held = read_calendar(r)?;
        *self.target.state.lock() = State {
            now,
            fraction: fraction % TICKS_PER_SECOND,
            regs,
            ptr,
            select,
            held: holding.then_some(held),
        };
        Ok(())
    }
}

fn write_calendar(w: &mut ChunkWriter<'_>, c: Calendar) -> Result<()> {
    w.write_bytes(&[
        c.second, c.minute, c.hour, c.weekday, c.day, c.month, c.year,
    ])
}

fn read_calendar(r: &mut ChunkReader<'_>) -> Result<Calendar> {
    let b = r.read_bytes()?;
    if b.len() != 7 {
        return Err(Error::State(String::from(
            "an RX-8581 snapshot's calendar is not 7 bytes",
        )));
    }
    Ok(Calendar {
        second: b[0],
        minute: b[1],
        hour: b[2],
        weekday: b[3] % 7,
        day: b[4],
        month: b[5],
        year: b[6],
    })
}

impl Instance for Rx8581 {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rx8581::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("address", ValueKind::Uint).range(0, 0x7f))
        .prop(PropSchema::new("time", ValueKind::Str))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn rtc(time: &str) -> Rx8581 {
        Rx8581::with_bus(
            None,
            0x51,
            (!time.is_empty()).then(|| parse_time(time).unwrap()),
        )
    }

    fn write(d: &Rx8581, bytes: &[u8]) {
        let t = &d.target;
        t.address(Address::Seven(0x51), Direction::Write);
        for b in bytes {
            t.write(*b);
        }
        t.stop();
    }

    fn read(d: &Rx8581, reg: u8, n: usize) -> alloc::vec::Vec<u8> {
        let t = &d.target;
        t.address(Address::Seven(0x51), Direction::Write);
        t.write(reg);
        t.address(Address::Seven(0x51), Direction::Read);
        let out = (0..n)
            .map(|_| {
                let b = t.read();
                t.read_ack(Ack::Ack);
                b
            })
            .collect();
        t.stop();
        out
    }

    #[test]
    fn it_reads_the_date_it_was_given_in_bcd() {
        let d = rtc("2026-11-04T23:59:58");
        assert_eq!(
            read(&d, 0, 7),
            [0x58, 0x59, 0x23, 1 << 3, 0x04, 0x11, 0x26],
            "a Wednesday"
        );
        assert_eq!(read(&d, REG_FLAG, 1)[0] & FLAG_VLF, 0, "the time is valid");
    }

    #[test]
    fn the_gps_librarys_read_wraps_from_the_control_register() {
        let d = rtc("2026-01-01T12:00:00");
        let r = read(&d, REG_FLAG, 10);
        assert_eq!(
            &r[2..9],
            [0x00, 0x00, 0x12, 1 << 4, 0x01, 0x01, 0x26],
            "a Thursday"
        );
    }

    #[test]
    fn the_crystal_carries_into_a_leap_day_and_a_new_year() {
        let d = rtc("2028-02-28T23:59:59");
        d.target.state.lock().advance(TICKS_PER_SECOND);
        assert_eq!(read(&d, REG_DAY, 3), [0x29, 0x02, 0x28]);
        let d = rtc("2026-12-31T23:59:59");
        d.target.state.lock().advance(TICKS_PER_SECOND - 1);
        assert_eq!(read(&d, REG_SEC, 1), [0x59], "not yet");
        d.target.state.lock().advance(1);
        assert_eq!(read(&d, REG_DAY, 3), [0x01, 0x01, 0x27]);
        assert_eq!(read(&d, REG_WEEK, 1), [1 << 5], "2027 starts on a Friday");
    }

    #[test]
    fn without_a_time_it_powers_up_invalid_and_software_sets_it() {
        let d = rtc("");
        assert_ne!(read(&d, REG_FLAG, 1)[0] & FLAG_VLF, 0);
        write(&d, &[REG_SEC, 0x30, 0x15, 0x09, 1 << 2, 0x10, 0x06, 0x26]);
        write(&d, &[REG_FLAG, 0x00]);
        assert_eq!(read(&d, 0, 7), [0x30, 0x15, 0x09, 1 << 2, 0x10, 0x06, 0x26]);
        assert_eq!(
            read(&d, REG_FLAG, 1)[0] & FLAG_VLF,
            0,
            "cleared by writing 0"
        );
        write(&d, &[0x07, 0xa5]);
        assert_eq!(read(&d, 0x07, 1), [0xa5], "the RAM byte");
    }

    #[test]
    fn stop_holds_the_count_and_a_read_sees_one_instant() {
        let d = rtc("2026-01-01T00:00:59");
        write(&d, &[REG_CONTROL, CONTROL_STOP]);
        d.target.state.lock().advance(10 * TICKS_PER_SECOND);
        assert_eq!(read(&d, REG_SEC, 1), [0x59]);
        write(&d, &[REG_CONTROL, 0]);
        let t = &d.target;
        t.address(Address::Seven(0x51), Direction::Write);
        t.write(REG_SEC);
        t.address(Address::Seven(0x51), Direction::Read);
        let sec = t.read();
        t.read_ack(Ack::Ack);
        t.state.lock().advance(TICKS_PER_SECOND);
        let min = t.read();
        t.stop();
        assert_eq!((sec, min), (0x59, 0x00), "the minute did not move mid-read");
        assert_eq!(read(&d, REG_MIN, 1), [0x01], "but it did move");
    }

    #[test]
    fn bad_times_are_refused() {
        for t in [
            "1999-12-31T00:00:00",
            "2026-02-29T00:00:00",
            "2026-01-01T24:00:00",
            "now",
        ] {
            assert!(parse_time(t).is_err(), "{t}");
        }
    }

    #[test]
    fn a_snapshot_round_trips() {
        let d = rtc("2026-05-05T05:05:05");
        d.target.state.lock().advance(12_345);
        write(&d, &[0x07, 0x42]);
        let save = |d: &Rx8581| {
            let mut shape = MachineShape::new();
            shape.add_device("rtc", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("rtc", CLASS.name, CLASS.version).unwrap();
                d.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&d);
        let back = rtc("");
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rtc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let restored = back.target.state.lock().clone();
        assert_eq!(restored, *d.target.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
