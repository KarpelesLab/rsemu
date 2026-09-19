//! The battery-backed clock of an A500+: an Oki MSM6242B, as the board decodes
//! it at `$DC_0000`.
//!
//! One class, `amiga.rtc`.
//!
//! ```text
//!   osc rtcclk = 32768 Hz
//!   object rtc "amiga.rtc" { clock = rtcclk, time = "2026-01-01T00:00:00" }
//!   map mem 0xDC0000 size 0x10000 = mirror(rtc)
//! ```
//!
//! # Sources
//!
//! * The *MSM6242B Direct Bus Connected CMOS Real Time Clock/Calendar* data
//!   sheet (Oki Semiconductor): Figure 1's register table, the *Functional
//!   Description of Registers* for `S1`…`W`, `CD`, `CE` and `CF`, Table 1 (the
//!   week register) and Table 2 (the interrupt periods). Every behaviour below
//!   cites one of them.
//! * The *Amiga Hardware Reference Manual*, 3rd edition, Appendix D, which lists
//!   `$DC_0000`–`$DC_FFFF` as the clock's; and the board: the chip's
//!   four address pins `A0`–`A3` on the processor's `A2`–`A5` and its four data
//!   pins on `D0`–`D3`, so register *n* answers every four bytes from
//!   `$DC_0000 + 4n` on the low byte lane and the block repeats every 64 bytes
//!   through the window. The processor side of that decode — which lane, which
//!   stride — was confirmed black-box: booting Workbench 2.04 on the A500+,
//!   Kickstart 2.04 reads `CF` at `$DC_003F`, writes `HOLD` into `CD` at
//!   `$DC_0037`, reads `S1`…`W` one byte each at `$DC_0003 + 4n`, and writes
//!   `HOLD` clear again — the data sheet's Figure 10 protocol, on the odd
//!   (low) byte lane.
//!
//! **No emulator source of any licence was consulted** (`ROADMAP.md` §1).
//!
//! # A real-time clock that does not read real time
//!
//! As `pc.rtc` argues at length: a device may never read the host's clock
//! (`CLAUDE.md`, *Determinism*), so the date the chip starts at is the `time`
//! property, and from there it counts only its own 32 768 Hz domain — the
//! watch crystal beside the chip. Two runs of one machine see the same date.
//!
//! # What is modelled
//!
//! | register | behaviour |
//! | --- | --- |
//! | `S1`…`W` | BCD digits, counted with the carries, month lengths and "Auto leap year" (every fourth year of the two-digit year) of the data sheet; bits the register table marks `*` are dropped on a write and read as 0 (Note 1) |
//! | `H10` | `PM/AM` in 12-hour mode, read as 0 in 24-hour mode ("it is continuously read out as 0") |
//! | `CD` | `HOLD` holds the seconds, and a carry that fell inside it lands when it is released ("the S1 counter will be incremented by 1 second after HOLD = 0"); `BUSY`; `IRQ FLAG`, cleared only by writing 0 (Note 3); `±30 ADJ` rounds to the nearest minute |
//! | `CE` | `MASK`, `ITRPT/STND` and the period `t1 t0` (Table 2) drive `IRQ FLAG` |
//! | `CF` | `REST` clears the sub-second divider and holds it; `STOP` stops the count; `24/12` is taken only with `REST` set ("REST bit must = 1 to write to the 24/12 hour bit"); `TEST` is held |
//!
//! # What the data sheet leaves open, and what was chosen
//!
//! * **How long `BUSY` reads 1.** The data sheet gives the protocol — set
//!   `HOLD`, read `BUSY`, retry if it is set — not a window. Here it reads 1
//!   for the first four ticks (122 µs) of every second, unless `HOLD` is
//!   holding back a carry.
//! * **`±30 ADJ`** "will automatically return to a '0'" 125 µs after it is
//!   written; here the adjustment is made at once and the bit reads 0.
//! * **`TEST`**: the seconds counting "at 5.4163KHz" is not modelled.
//! * **`STD.P`**, the interrupt pin: not wired on this board, so there is no
//!   pin. `IRQ FLAG` still reports it (Figure 12's relationship: the flag is
//!   set whenever `STD.P` would be low).
//! * **The data lines the chip does not drive** — `D4`–`D15` — read as 0. The
//!   board's pull-ups are on a schematic this was not written from, and every
//!   use of the clock masks the nibble.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
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
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.rtc";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The watch crystal: one tick of the chip's domain.
pub const TICKS_PER_SECOND: u64 = 32_768;

/// The date a machine that names none starts at.
pub const DEFAULT_TIME: &str = "2026-01-01T00:00:00";

/// Sixteen registers four bytes apart: `A0`–`A3` on the processor's `A2`–`A5`.
pub const WINDOW: u64 = 64;

/// How long `BUSY` reads 1 after a carry, in ticks: 122 µs. A choice; see the
/// module documentation.
const BUSY_TICKS: u64 = 4;

/// The fixed-cycle waveform's low time, "7.8125ms", in ticks.
const PULSE_TICKS: u64 = 256;

// Figure 1's registers.
const S1: usize = 0;
const S10: usize = 1;
const MI1: usize = 2;
const MI10: usize = 3;
const H1: usize = 4;
const H10: usize = 5;
const D1: usize = 6;
const D10: usize = 7;
const MO1: usize = 8;
const MO10: usize = 9;
const Y1: usize = 10;
const Y10: usize = 11;
const W: usize = 12;
const CD: usize = 13;
const CE: usize = 14;
const CF: usize = 15;

/// The bits each of `S1`…`W` has (Figure 1; the `*` bits are not there).
const DIGIT_BITS: [u8; 13] = [
    0xf, 0x7, 0xf, 0x7, 0xf, 0x7, 0xf, 0x3, 0xf, 0x1, 0xf, 0xf, 0x7,
];

// Control register D.
const HOLD: u8 = 1 << 0;
const BUSY: u8 = 1 << 1;
const IRQ_FLAG: u8 = 1 << 2;
const ADJ_30: u8 = 1 << 3;
// Control register E.
const MASK: u8 = 1 << 0;
const ITRPT: u8 = 1 << 1;
// Control register F.
const REST: u8 = 1 << 0;
const STOP: u8 = 1 << 1;
const H24: u8 = 1 << 2;
/// `H10`'s `PM/AM` bit.
const PM: u8 = 1 << 2;

/// What the chip holds.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// `S1`…`W`, a BCD digit each.
    digits: [u8; 13],
    /// `CD`'s `HOLD`.
    hold: bool,
    /// `IRQ FLAG`, as last set: when, in ticks.
    irq_at: Option<u64>,
    ce: u8,
    cf: u8,
    /// Ticks into the current second: the divider below the 1 Hz stage.
    phase: u64,
    /// A carry fell while `HOLD` was set.
    held_carry: bool,
    /// Ticks of the chip's domain simulated.
    tick: u64,
}

impl State {
    fn digit(&self, r: usize) -> u8 {
        self.digits[r]
    }

    fn pair(&self, tens: usize, ones: usize, tens_bits: u8) -> u8 {
        (self.digits[tens] & tens_bits) * 10 + self.digits[ones]
    }

    fn set_pair(&mut self, tens: usize, ones: usize, value: u8, keep: u8) {
        self.digits[tens] = (self.digits[tens] & keep) | (value / 10);
        self.digits[ones] = value % 10;
    }

    fn running(&self) -> bool {
        self.cf & (STOP | REST) == 0
    }

    /// The period `t1 t0` selects (Table 2), for the carries that are one.
    fn period(&self) -> u8 {
        (self.ce >> 2) & 3
    }

    /// A carry of period `p` (0 = 1/64 s, 1 = second, 2 = minute, 3 = hour)
    /// happened at `at`: raise `IRQ FLAG` if that is the period selected and
    /// `MASK` lets it through. "When IRQ = 1 and timing for a new interrupt
    /// occurs, the new interrupt is ignored" in interrupt mode.
    fn carry(&mut self, p: u8, at: u64) {
        if self.ce & MASK != 0 || self.period() != p {
            return;
        }
        if self.ce & ITRPT != 0 && self.irq_at.is_some() {
            return;
        }
        self.irq_at = Some(at);
    }

    /// `IRQ FLAG` now: held until written 0 in interrupt mode, and for 7.8125
    /// ms in the fixed-cycle mode.
    fn irq(&self) -> bool {
        match self.irq_at {
            None => false,
            Some(at) => self.ce & ITRPT != 0 || self.tick < at + PULSE_TICKS,
        }
    }

    fn days_in_month(&self) -> u8 {
        let month = self.pair(MO10, MO1, 1);
        let year = self.pair(Y10, Y1, 0xf);
        match month {
            2 if year.is_multiple_of(4) => 29,
            2 => 28,
            4 | 6 | 9 | 11 => 30,
            _ => 31,
        }
    }

    /// One second, with every carry it makes.
    fn second(&mut self, at: u64) {
        self.carry(1, at);
        let s = self.pair(S10, S1, 7) + 1;
        if s < 60 {
            self.set_pair(S10, S1, s, 0);
            return;
        }
        self.set_pair(S10, S1, 0, 0);
        self.minute(at);
    }

    fn minute(&mut self, at: u64) {
        self.carry(2, at);
        let m = self.pair(MI10, MI1, 7) + 1;
        if m < 60 {
            self.set_pair(MI10, MI1, m, 0);
            return;
        }
        self.set_pair(MI10, MI1, 0, 0);
        self.carry(3, at);
        let h = self.pair(H10, H1, 3);
        let pm = self.digits[H10] & PM;
        if self.cf & H24 != 0 {
            if h < 23 {
                self.set_pair(H10, H1, h + 1, pm);
                return;
            }
            self.set_pair(H10, H1, 0, pm);
        } else {
            // Twelve-hour: 11 AM -> 12 PM, 12 -> 1, 11 PM -> 12 AM and a new
            // day.
            match h {
                11 => {
                    self.set_pair(H10, H1, 12, 0);
                    self.digits[H10] |= pm ^ PM;
                    if pm == 0 {
                        return;
                    }
                }
                12 => {
                    self.set_pair(H10, H1, 1, pm);
                    return;
                }
                _ => {
                    self.set_pair(H10, H1, h + 1, pm);
                    return;
                }
            }
        }
        self.day();
    }

    fn day(&mut self) {
        self.digits[W] = (self.digits[W] + 1) % 7;
        // "If the date February 29 or November 31, 1985, was written, it would
        // be changed automatically to March 1, or December 1, 1985 at the exact
        // time at which a carry pulse occurs for the day's digit."
        let d = self.pair(D10, D1, 3);
        if d < self.days_in_month() {
            self.set_pair(D10, D1, d + 1, 0);
            return;
        }
        self.set_pair(D10, D1, 1, 0);
        let mo = self.pair(MO10, MO1, 1);
        if mo < 12 {
            self.set_pair(MO10, MO1, mo + 1, 0);
            return;
        }
        self.set_pair(MO10, MO1, 1, 0);
        let y = self.pair(Y10, Y1, 0xf);
        self.set_pair(Y10, Y1, (y + 1) % 100, 0);
    }

    /// Count forward to `target`.
    fn advance_to(&mut self, target: u64) {
        while self.tick < target {
            if !self.running() {
                // REST clears the divider and holds it; STOP holds it.
                if self.cf & REST != 0 {
                    self.phase = 0;
                }
                self.tick = target;
                return;
            }
            let to_second = TICKS_PER_SECOND - self.phase;
            let step = to_second.min(target - self.tick);
            // The 1/64 s period's carries inside this step: the latest is the
            // only one the flag can show.
            if self.period() == 0 {
                let from = self.phase;
                let to = self.phase + step;
                let last = to / 512 * 512;
                if last > from {
                    let at = self.tick + (last - from);
                    self.carry(0, at);
                }
            }
            self.tick += step;
            self.phase += step;
            if self.phase == TICKS_PER_SECOND {
                self.phase = 0;
                if self.hold {
                    self.held_carry = true;
                } else {
                    self.second(self.tick);
                }
            }
        }
    }

    fn read(&self, r: usize) -> u8 {
        match r {
            H10 if self.cf & H24 != 0 => self.digits[H10] & !PM,
            S1..=W => self.digit(r),
            CD => {
                let busy = self.running() && self.phase < BUSY_TICKS && !self.held_carry;
                u8::from(self.hold)
                    | if busy { BUSY } else { 0 }
                    | if self.irq() { IRQ_FLAG } else { 0 }
            }
            CE => self.ce,
            CF => self.cf,
            _ => 0,
        }
    }

    fn write(&mut self, r: usize, value: u8) {
        let value = value & 0xf;
        match r {
            S1..=W => self.digits[r] = value & DIGIT_BITS[r],
            CD => {
                // Note 3: the flag can only be set to 0 by a write.
                if value & IRQ_FLAG == 0 {
                    self.irq_at = None;
                }
                let hold = value & HOLD != 0;
                if self.hold && !hold && self.held_carry {
                    self.held_carry = false;
                    self.second(self.tick);
                }
                self.hold = hold;
                if value & ADJ_30 != 0 {
                    // "30-second adjustment": to the nearest minute.
                    if self.pair(S10, S1, 7) >= 30 {
                        self.set_pair(S10, S1, 0, 0);
                        self.minute(self.tick);
                    } else {
                        self.set_pair(S10, S1, 0, 0);
                    }
                }
            }
            CE => self.ce = value,
            CF => {
                let h24 = if (self.cf | value) & REST != 0 {
                    value & H24
                } else {
                    self.cf & H24
                };
                self.cf = (value & !H24) | h24;
                if self.cf & REST != 0 {
                    self.phase = 0;
                }
            }
            // Four address pins: there is no seventeenth register.
            _ => {}
        }
    }
}

/// Parse `"YYYY-MM-DDTHH:MM:SS"` into the chip's digits, 24-hour mode, and
/// the day of the week (Table 1: Sunday is 0) from the date.
fn parse_time(text: &str) -> Result<State> {
    let bad = || {
        Error::Property(format!(
            "{CLASS_NAME}: `time` is a date like \"{DEFAULT_TIME}\", not \"{text}\""
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
    let field = |from: usize, to: usize| -> Result<u32> {
        b[from..to].iter().try_fold(0u32, |acc, &c| {
            c.is_ascii_digit()
                .then(|| acc * 10 + u32::from(c - b'0'))
                .ok_or_else(bad)
        })
    };
    let (year, month, day) = (field(0, 4)?, field(5, 7)?, field(8, 10)?);
    let (hour, minute, second) = (field(11, 13)?, field(14, 16)?, field(17, 19)?);
    let leap = year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400));
    let days = match month {
        2 if leap => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    };
    if !(1..=12).contains(&month)
        || !(1..=days).contains(&day)
        || hour > 23
        || minute > 59
        || second > 59
        || year == 0
    {
        return Err(bad());
    }
    // Zeller's congruence for the Gregorian calendar, shifted so Sunday is 0.
    let (m, y) = if month < 3 {
        (month + 12, year - 1)
    } else {
        (month, year)
    };
    let h = (day + 13 * (m + 1) / 5 + y + y / 4 - y / 100 + y / 400) % 7;
    let weekday = ((h + 6) % 7) as u8;
    let mut st = State {
        digits: [0; 13],
        hold: false,
        irq_at: None,
        ce: 0,
        cf: H24,
        phase: 0,
        held_carry: false,
        tick: 0,
    };
    st.set_pair(S10, S1, second as u8, 0);
    st.set_pair(MI10, MI1, minute as u8, 0);
    st.set_pair(H10, H1, hour as u8, 0);
    st.set_pair(D10, D1, day as u8, 0);
    st.set_pair(MO10, MO1, month as u8, 0);
    st.set_pair(Y10, Y1, (year % 100) as u8, 0);
    st.digits[W] = weekday;
    Ok(st)
}

/// The register window, as something an address space dispatches to.
struct Registers {
    state: Mutex<State>,
    /// Published for the scheduler's lock-free question.
    tick: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("tick", &self.tick.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Registers {
    fn advance_to(&self, target: u64) {
        let mut st = self.state.lock();
        st.advance_to(target);
        self.tick.store(st.tick, Ordering::Relaxed);
    }

    /// Catch up before a guest access (`ROADMAP.md` §4.2).
    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // Refused only if catch-up is already further up the stack; the
            // access is then answered from where the chip stands.
            let _ = handle.sync(AccessKind::Guest);
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !attrs.debug {
            self.sync();
        }
        let nibble = self.state.lock().read(((offset >> 2) & 0xf) as usize);
        // D0-D3: the low byte of a word, an odd address as a byte.
        match dst.len() {
            2 if offset & 1 == 0 => dst.copy_from_slice(&[0, nibble]),
            1 => dst[0] = if offset & 1 == 1 { nibble } else { 0 },
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Every register write moves the clock or its control; a debugger
            // may not.
            return Err(BusError::BadAccess);
        }
        let value = match src.len() {
            2 if offset & 1 == 0 => src[1],
            // MC68000UM Table 3-1: a byte write drives the byte on both halves
            // of the bus, so the low lane sees it at either address.
            1 => src[0],
            _ => return Err(BusError::BadAccess),
        };
        self.sync();
        self.state
            .lock()
            .write(((offset >> 2) & 0xf) as usize, value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U16, Endian::Big)
        }
    }
}

/// The `amiga.rtc` device.
#[derive(Debug)]
pub struct Rtc {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Rtc {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `time` is not a date, or a property nothing here
    /// accepts was given.
    pub fn new(props: &Props) -> Result<Rtc> {
        let mut r = props.reader();
        let time = r.or_str("time", DEFAULT_TIME)?;
        r.finish()?;
        Ok(Rtc::at(parse_time(time)?))
    }

    fn at(start: State) -> Rtc {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, start),
            tick: AtomicU64::new(0),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            WINDOW,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rtc { regs, region }
    }

    /// Register `r` as the processor would read it, without catching up.
    #[must_use]
    pub fn peek(&self, r: usize) -> u8 {
        self.regs.state.lock().read(r & 0xf)
    }

    /// Count forward to `tick` of the chip's domain.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }
}

impl Device for Rtc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A battery-backed part: the machine's reset line is not one of its
        // pins, and the date survives it. That is the point of the part.
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
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
        // No pin: nothing outside the chip changes until it is read.
        None
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.regs.state.lock();
        for d in st.digits {
            w.write_u8(d)?;
        }
        w.write_bool(st.hold)?;
        w.write_bool(st.irq_at.is_some())?;
        w.write_u64(st.irq_at.unwrap_or(0))?;
        w.write_u8(st.ce)?;
        w.write_u8(st.cf)?;
        w.write_u64(st.phase)?;
        w.write_bool(st.held_carry)?;
        w.write_u64(st.tick)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut digits = [0u8; 13];
        for (d, bits) in digits.iter_mut().zip(DIGIT_BITS) {
            *d = r.read_u8()? & bits;
        }
        let hold = r.read_bool()?;
        let irq = r.read_bool()?;
        let irq_at = r.read_u64()?;
        let ce = r.read_u8()? & 0xf;
        let cf = r.read_u8()? & 0xf;
        let phase = r.read_u64()?;
        if phase >= TICKS_PER_SECOND {
            return Err(Error::State(String::from(
                "amiga.rtc: a divider past its second",
            )));
        }
        let held_carry = r.read_bool()?;
        let tick = r.read_u64()?;
        let mut st = self.regs.state.lock();
        *st = State {
            digits,
            hold,
            irq_at: irq.then_some(irq_at),
            ce,
            cf,
            phase,
            held_carry,
            tick,
        };
        self.regs.tick.store(tick, Ordering::Relaxed);
        Ok(())
    }
}

impl Instance for Rtc {}

/// The `amiga.rtc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an A500+'s battery-backed clock: an Oki MSM6242B at $DC0000, a register every \
              four bytes",
    properties: &[PropertySpec {
        name: "time",
        kind: ValueKind::Str,
        required: false,
        summary: "the date the clock starts at, \"YYYY-MM-DDTHH:MM:SS\" (never the host's)",
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

/// What the validator should know about `amiga.rtc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("time", ValueKind::Str))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec::Vec;

    fn rtc(time: &str) -> Rtc {
        Rtc::new(&Props::new().with("time", crate::core::props::Value::from(time))).unwrap()
    }

    /// Register `r` through the window, as the processor reads it: a byte at
    /// `$DC0003 + 4r`.
    fn read(c: &Rtc, r: u64) -> u8 {
        let mut b = [0u8];
        MemOps::read(&*c.regs, 4 * r + 3, &mut b, MemAttrs::DEFAULT).unwrap();
        b[0]
    }

    fn write(c: &Rtc, r: u64, v: u8) {
        MemOps::write(&*c.regs, 4 * r + 3, &[v], MemAttrs::DEFAULT).unwrap();
    }

    /// The time as `(hh, mm, ss)` out of the registers.
    fn hms(c: &Rtc) -> (u8, u8, u8) {
        let two = |t, o| read(c, t) * 10 + read(c, o);
        (
            (read(c, H10 as u64) & 3) * 10 + read(c, H1 as u64),
            two(MI10 as u64, MI1 as u64),
            two(S10 as u64, S1 as u64),
        )
    }

    fn date(c: &Rtc) -> (u8, u8, u8, u8) {
        let two = |t: usize, o: usize| read(c, t as u64) * 10 + read(c, o as u64);
        (
            two(Y10, Y1),
            two(MO10, MO1),
            two(D10, D1),
            read(c, W as u64),
        )
    }

    #[test]
    fn the_date_given_is_the_date_read_in_bcd_digits_and_the_weekday_follows() {
        // 2026-09-19 is a Saturday: Table 1's 6.
        let c = rtc("2026-09-19T12:34:56");
        assert_eq!(hms(&c), (12, 34, 56));
        assert_eq!(date(&c), (26, 9, 19, 6));
        assert_eq!(read(&c, CF as u64) & H24, H24, "24-hour mode");
        assert!(
            Rtc::new(&Props::new().with(
                "time",
                crate::core::props::Value::from("2026-02-30T00:00:00")
            ))
            .is_err()
        );
    }

    #[test]
    fn the_register_block_repeats_every_64_bytes_on_the_low_lane() {
        let c = rtc("2026-01-01T00:00:07");
        let mut word = [0u8; 2];
        MemOps::read(&*c.regs, 0, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(word, [0, 7], "S1 in the low byte");
        let mut byte = [0u8];
        MemOps::read(&*c.regs, 2, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0], "D8-D15 are not driven");
        MemOps::read(&*c.regs, 1, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [7], "A1 is not decoded");
    }

    #[test]
    fn it_counts_its_own_crystal_through_every_carry() {
        // The last second of a leap year's February 28th's day... and of the
        // century's year 99.
        let c = rtc("2099-12-31T23:59:59");
        c.advance_to(TICKS_PER_SECOND - 1);
        assert_eq!(hms(&c), (23, 59, 59));
        c.advance_to(TICKS_PER_SECOND);
        assert_eq!(hms(&c), (0, 0, 0));
        assert_eq!(date(&c), (0, 1, 1, 5), "Thursday to Friday: 5");

        let c = rtc("2028-02-28T23:59:59");
        c.advance_to(TICKS_PER_SECOND);
        assert_eq!(
            date(&c).2,
            29,
            "2028 is a leap year: its year digits divide by four"
        );
        let c = rtc("2026-02-28T23:59:59");
        c.advance_to(TICKS_PER_SECOND);
        assert_eq!((date(&c).1, date(&c).2), (3, 1));
    }

    #[test]
    fn hold_keeps_the_digits_still_and_the_carry_lands_on_release() {
        let c = rtc("2026-01-01T00:00:10");
        c.advance_to(TICKS_PER_SECOND / 2);
        write(&c, CD as u64, HOLD | IRQ_FLAG);
        assert_eq!(read(&c, CD as u64) & BUSY, 0, "not mid-update");
        c.advance_to(TICKS_PER_SECOND + 100);
        assert_eq!(hms(&c).2, 10, "held");
        write(&c, CD as u64, IRQ_FLAG);
        assert_eq!(hms(&c).2, 11, "incremented by 1 second after HOLD = 0");
    }

    #[test]
    fn busy_reads_one_just_after_a_carry() {
        let c = rtc("2026-01-01T00:00:00");
        c.advance_to(TICKS_PER_SECOND + 1);
        assert_eq!(read(&c, CD as u64) & BUSY, BUSY);
        c.advance_to(TICKS_PER_SECOND + BUSY_TICKS);
        assert_eq!(read(&c, CD as u64) & BUSY, 0);
    }

    #[test]
    fn rest_and_stop_hold_the_count_and_24_12_needs_rest() {
        let c = rtc("2026-01-01T13:00:00");
        write(&c, CF as u64, 0);
        assert_eq!(read(&c, CF as u64) & H24, H24, "not written without REST");
        write(&c, CF as u64, REST | H24);
        write(&c, CF as u64, REST);
        assert_eq!(read(&c, CF as u64) & H24, 0, "12-hour mode");
        write(&c, CF as u64, STOP);
        c.advance_to(10 * TICKS_PER_SECOND);
        assert_eq!(hms(&c).2, 0, "stopped");
        write(&c, CF as u64, 0);
        c.advance_to(11 * TICKS_PER_SECOND);
        assert_eq!(hms(&c).2, 1, "running from a cleared divider");
    }

    #[test]
    fn twelve_hour_mode_turns_eleven_pm_into_twelve_am_and_a_new_day() {
        let c = rtc("2026-01-01T00:00:00");
        write(&c, CF as u64, REST);
        write(&c, CF as u64, 0);
        // 11:59:59 PM.
        write(&c, H10 as u64, PM | 1);
        write(&c, H1 as u64, 1);
        for (r, v) in [(MI10, 5), (MI1, 9), (S10, 5), (S1, 9)] {
            write(&c, r as u64, v);
        }
        c.advance_to(TICKS_PER_SECOND);
        assert_eq!((read(&c, H10 as u64), read(&c, H1 as u64)), (1, 2), "12 AM");
        assert_eq!(date(&c).2, 2);
    }

    #[test]
    fn the_irq_flag_follows_the_period_and_is_cleared_by_writing_zero() {
        let c = rtc("2026-01-01T00:00:00");
        write(&c, CE as u64, ITRPT | (1 << 2)); // interrupt mode, one second
        c.advance_to(TICKS_PER_SECOND);
        assert_eq!(read(&c, CD as u64) & IRQ_FLAG, IRQ_FLAG);
        c.advance_to(3 * TICKS_PER_SECOND);
        assert_eq!(read(&c, CD as u64) & IRQ_FLAG, IRQ_FLAG, "held");
        write(&c, CD as u64, 0);
        assert_eq!(read(&c, CD as u64) & IRQ_FLAG, 0);
        // The fixed-cycle waveform: low for 7.8125 ms.
        write(&c, CE as u64, 1 << 2);
        c.advance_to(4 * TICKS_PER_SECOND + 10);
        assert_eq!(read(&c, CD as u64) & IRQ_FLAG, IRQ_FLAG);
        c.advance_to(4 * TICKS_PER_SECOND + PULSE_TICKS);
        assert_eq!(read(&c, CD as u64) & IRQ_FLAG, 0);
    }

    #[test]
    fn thirty_second_adjust_rounds_to_the_nearest_minute() {
        let c = rtc("2026-01-01T00:10:31");
        write(&c, CD as u64, ADJ_30 | IRQ_FLAG);
        assert_eq!(hms(&c), (0, 11, 0));
        let c = rtc("2026-01-01T00:10:29");
        write(&c, CD as u64, ADJ_30 | IRQ_FLAG);
        assert_eq!(hms(&c), (0, 10, 0));
        assert_eq!(read(&c, CD as u64) & ADJ_30, 0);
    }

    #[test]
    fn a_debugger_reads_without_moving_and_may_not_write() {
        let c = rtc("2026-01-01T00:00:00");
        let mut b = [0u8];
        MemOps::read(&*c.regs, 3, &mut b, MemAttrs::DEBUG).unwrap();
        assert!(MemOps::write(&*c.regs, 3, &[5], MemAttrs::DEBUG).is_err());
        assert_eq!(read(&c, S1 as u64), 0);
    }

    fn snapshot(c: &Rtc) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("rtc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rtc", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(c, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_and_resumes_identically() {
        let saved = rtc("2026-03-04T05:06:07");
        write(&saved, CE as u64, ITRPT);
        write(&saved, CD as u64, HOLD | IRQ_FLAG);
        saved.advance_to(TICKS_PER_SECOND + 77);
        let bytes = snapshot(&saved);
        let restored = rtc(DEFAULT_TIME);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rtc", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(snapshot(&restored), bytes);
        for c in [&saved, &restored] {
            write(c, CD as u64, IRQ_FLAG);
            c.advance_to(5 * TICKS_PER_SECOND);
        }
        assert_eq!(snapshot(&restored), snapshot(&saved));
        assert_eq!(hms(&restored), (5, 6, 12));
    }
}
