//! `mac.rtc`: the Macintosh's clock chip — a four-byte second counter, twenty
//! bytes of parameter RAM, and the one-second interrupt — on three of the
//! VIA's port B pins.
//!
//! ```text
//!   osc    rtcxtal = 32768 Hz
//!   object rtc "mac.rtc" { clock = rtcxtal, time = "2026-01-01T00:00:00" }
//!
//!   wire via.pb2  -> rtc.enb  { pull = "up" }   # /RTCENB
//!   wire via.pb1  -> rtc.clk  { pull = "up" }   # RTCCLK
//!   wire via.pb0  -> rtc.data { pull = "up" }   # RTCDATA, both ways
//!   wire rtc.data -> via.pb0
//!   wire rtc.irq  -> via.ca2  { pull = "up" }   # the one-second interrupt
//! ```
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition, **chapter 3**,
//! "Real-time clock (RTC)", pp. 144-145:
//!
//! * What the chip is: "The RTC contains a 4-byte counter incremented once each
//!   second. Each time the counter is incremented, the RTC sends an interrupt
//!   request signal to the VIA and (if this interrupt is enabled), the VIA sends
//!   an interrupt to the main processor. The RTC also contains 256 bytes of RAM
//!   (**20 bytes in the Macintosh 512K and Macintosh 128K**) … called parameter
//!   RAM, that is powered by a battery when the Macintosh is turned off."
//! * The wires, **Table 3-11**: bit 2 output `rtcEnb`, "0 = RTC is enabled";
//!   bit 1 output `rtcClk`, "RTC's data-clock line"; bit 0 in or out
//!   `rtcData`, "RTC's serial data line".
//! * What they do: "These 3 bits constitute a simple serial interface. The
//!   rtcData bit is used as a bidirectional serial data line to send command and
//!   data bytes back and forth. The rtcClk bit is a data-clock line, **driven by
//!   the processor**, that regulates the transmission of the data and command
//!   bits … The rtcEnb bit is the serial enable line, which signals the RTC that
//!   the processor is about to send it serial commands and data."
//! * The interrupt: "The RTC generates a VIA interrupt once each second (if this
//!   interrupt is enabled). This interrupt can be enabled or disabled by writing
//!   to bit 0 of the VIA's Interrupt Enable register."
//!
//! The Guide stops there and sends the reader to *Inside Macintosh*'s Operating
//! System Utilities chapter for the command encoding, which is not a hardware
//! document and is not here. **So the encoding below was recovered by tracing a
//! real ROM** — which addresses it clocks out, in what order, and which of them
//! it then writes to — exactly as `CLAUDE.md` prescribes when the documents run
//! out. No emulator source was consulted and the ROM was not disassembled
//! (`ROADMAP.md` §1).
//!
//! # The encoding, and how it was read off the wire
//!
//! The first thing a ROM does, 131 φ2 ticks into the run, is clock out `$C1` and
//! then let go of the data line for eight more clocks. Every frame it sends has
//! that shape — one command byte, then one data byte in one direction or the
//! other, then `/RTCENB` back up — and the command bytes it sends fall into an
//! obvious pattern:
//!
//! ```text
//!   $C1 $C5 $C9 $CD … $FD    sixteen reads, four apart
//!   $A1 $A5 $A9 $AD          four more
//!   $9D $99 $95 … $81        eight reads, descending
//!   $35+$55, $31+$00, $35+$D5 three writes that bracket the rest
//!   $41+data … $7D+data      sixteen writes, four apart
//!   $21+data … $2D+data      four more
//! ```
//!
//! Four apart is bits 1-0 held at `01` and an address in **bits 6-2**; the high
//! bit is the direction, because the sixteen `$C1`-`$FD` frames listen and the
//! sixteen `$41`-`$7D` frames speak to the same addresses. So:
//!
//! ```text
//!   bit 7     1 = read, 0 = write
//!   bits 6-2  the register address
//!   bits 1-0  always 01
//! ```
//!
//! and the addresses line up with what the Guide says is in the chip:
//!
//! | address | register |
//! | --- | --- |
//! | `$00`-`$03` | the four bytes of the second counter, least significant first |
//! | `$04`-`$07` | the same four again — only two address bits are decoded here |
//! | `$08`-`$0B` | parameter RAM, the first four bytes |
//! | `$0C` | the test register, write only |
//! | `$0D` | the write-protect register, write only |
//! | `$10`-`$1F` | parameter RAM, the other sixteen |
//!
//! Sixteen plus four is **twenty bytes of parameter RAM**, which is the number
//! the Guide gives for this chip, and it is the strongest confirmation the
//! address field is read correctly: nothing else about the trace would have
//! produced exactly twenty.
//!
//! `$0D` is the write-protect register because of what brackets the writes.
//! Before the ROM writes anything it sends `$35 $55` and afterwards `$35 $D5` —
//! the same register, with bit 7 clear and then set — and in between it is
//! allowed to write. `$31 $00` between them is the test register, which the
//! chip has and which this model stores and otherwise ignores.
//!
//! The byte order of the counter (least significant at `$00`) is the one thing
//! here that a trace of the *commands* cannot settle, because the ROM only ever
//! reads it. It is settled from the other end instead: `tests/mac_plus.rs` sets
//! a date through the `time` property and reads the `Time` low-memory global
//! back out of the guest, which is a structure the ROM built rather than code it
//! ran.
//!
//! # A real-time clock that does not read real time
//!
//! As `amiga.rtc` and `pc.rtc` argue: a device may never read the host's clock
//! (`CLAUDE.md`, *Determinism*), so the date the chip starts at is the `time`
//! property and from there it counts only its own 32 768 Hz domain — the watch
//! crystal beside the chip. Two runs of one machine see the same date.
//!
//! # What is chosen where nothing states it
//!
//! * **Which clock edge carries which direction.** The processor drives the
//!   clock, so the chip must present a bit before the processor looks and sample
//!   one when the processor has settled it. This chip takes the data line on the
//!   **rising** edge and changes its own output on the **falling** one, which is
//!   the arrangement that leaves each bit stable across the whole of the high
//!   half whichever instant in it the processor reads.
//! * **`/RTCENB` going high** abandons whatever frame was in progress and
//!   releases the data line, which is what makes a frame a frame: the trace has
//!   the ROM raise it between every command and the next.
//! * **The one-second output is a square wave**, half a second low and half a
//!   second high, because that is what the last stage of a divider chain
//!   produces. The VIA takes one edge of it, so it is one interrupt a second
//!   whichever edge PCR selects — which is the behaviour the Guide describes.
//! * **What is in parameter RAM at power-on.** A machine with a flat battery,
//!   so every byte reads `$FF`; a ROM that finds that invalid writes its own
//!   defaults, which is exactly what the trace shows this one doing. `pram` sets
//!   it otherwise.
//!
//! # Not modelled
//!
//! * **The battery.** Parameter RAM is part of the snapshot and is lost with the
//!   machine; there is no separate persistence.
//! * **The 256-byte chip** of the Macintosh SE and later, and its two-byte
//!   extended command form. A Plus has the twenty-byte part.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "mac.rtc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// The serial data line, the VIA's `PB0`. Driven from both ends.
pub const DATA_PIN: &str = "data";
/// The data-clock line, the VIA's `PB1`. "Driven by the processor."
pub const CLK_PIN: &str = "clk";
/// The serial enable, the VIA's `PB2`. Active low.
pub const ENB_PIN: &str = "enb";
/// The one-second interrupt, which a Macintosh wires to the VIA's `CA2`.
pub const IRQ_PIN: &str = "irq";

/// How many bytes of parameter RAM this chip has. "20 bytes in the Macintosh
/// 512K and Macintosh 128K", and in the Plus.
pub const PRAM_BYTES: usize = 20;

/// Ticks of the watch crystal in one second. A board gives it 32 768 Hz.
pub const TICKS_PER_SECOND: u64 = 32_768;

/// The date the chip starts at when a machine file does not say.
pub const DEFAULT_TIME: &str = "2026-01-01T00:00:00";

/// Seconds from the Macintosh epoch — midnight, 1 January 1904 — to the Unix
/// one, which is what the date arithmetic below is easiest to do in.
///
/// 1904 to 1970 is 66 years holding 17 leap days (1904, 1908 … 1968; 1900 is
/// outside the range and 2000 is irrelevant), so 66 × 365 + 17 days.
const MAC_EPOCH_TO_UNIX: u32 = (66 * 365 + 17) * 86_400;

/// The address of the test register, which the chip has and nothing reads.
const ADDR_TEST: u8 = 0x0c;
/// The address of the write-protect register.
const ADDR_WRITE_PROTECT: u8 = 0x0d;
/// Bit 7 of the write-protect register: set means the counter and parameter RAM
/// refuse writes.
const WRITE_PROTECTED: u8 = 0x80;

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// the serial interface
// ---------------------------------------------------------------------------

/// Where in a frame the chip is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Serial {
    /// `/RTCENB` is high: nothing is happening and the data line is let go.
    Idle,
    /// Taking the command byte in, most significant bit first.
    Command { bits: u8, got: u8 },
    /// Taking the data byte of a write.
    Write { addr: u8, bits: u8, got: u8 },
    /// Putting the data byte of a read out, most significant bit first.
    ///
    /// `sent` is how many bits are already on the line, which is what stops the
    /// command byte's own trailing edge from eating bit 7: the chip enters this
    /// phase on the *rising* edge of the command's last bit, and the falling
    /// edge that follows is the first one that belongs to the data byte.
    Read { bits: u8, sent: u8 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Ticks of the watch crystal simulated.
    ticks: u64,
    /// The tick the one-second output next changes on.
    next: u64,
    /// The four-byte counter, incremented once a second.
    seconds: u32,
    /// Twenty bytes of battery-backed RAM.
    pram: [u8; PRAM_BYTES],
    /// The write-protect register.
    write_protect: u8,
    /// The test register: stored, and nothing more.
    test: u8,

    serial: Serial,
    /// Whether `/RTCENB` is asserted — which is the line being *low*.
    enabled: bool,
    /// The last level seen on the clock line.
    clk: bool,
    /// The last level seen on the data line.
    data_high: bool,
    /// Whether the chip is pulling the data line down.
    data_low: bool,
    /// Whether it is pulling the one-second line down.
    irq_low: bool,
}

impl State {
    /// Power-on: the given second, a flat battery, and the one-second output
    /// about to fall.
    fn power_on(seconds: u32, pram: [u8; PRAM_BYTES]) -> State {
        State {
            ticks: 0,
            next: TICKS_PER_SECOND / 2,
            seconds,
            pram,
            write_protect: WRITE_PROTECTED,
            test: 0,
            serial: Serial::Idle,
            enabled: false,
            clk: false,
            data_high: true,
            data_low: false,
            irq_low: false,
        }
    }

    /// What the two output stages are pulling, as a pair.
    fn pins(&self) -> (bool, bool) {
        (self.data_low, self.irq_low)
    }

    /// Whether the counter and parameter RAM are refusing writes.
    fn protected(&self) -> bool {
        self.write_protect & WRITE_PROTECTED != 0
    }

    /// What a read of `addr` answers.
    fn read(&self, addr: u8) -> u8 {
        match addr {
            // Only two address bits are decoded for the counter, so it answers
            // twice over; the ROM reads both halves and compares them.
            0x00..=0x07 => (self.seconds >> (8 * u32::from(addr & 3))) as u8,
            0x08..=0x0b => self.pram[usize::from(addr - 0x08)],
            0x10..=0x1f => self.pram[usize::from(addr - 0x10) + 4],
            // The test and write-protect registers are write-only, and nothing
            // else is decoded. A chip that is not driving leaves the pull-up.
            _ => 0xff,
        }
    }

    /// Apply a write of `value` to `addr`.
    fn write(&mut self, addr: u8, value: u8) {
        match addr {
            ADDR_TEST => self.test = value,
            ADDR_WRITE_PROTECT => self.write_protect = value,
            _ if self.protected() => {}
            0x00..=0x07 => {
                let shift = 8 * u32::from(addr & 3);
                self.seconds =
                    (self.seconds & !(0xffu32 << shift)) | (u32::from(value) << shift);
            }
            0x08..=0x0b => self.pram[usize::from(addr - 0x08)] = value,
            0x10..=0x1f => self.pram[usize::from(addr - 0x10) + 4] = value,
            _ => {}
        }
    }

    /// `/RTCENB` moved. Low asserts it; high abandons the frame.
    fn enable(&mut self, low: bool) {
        if self.enabled == low {
            return;
        }
        self.enabled = low;
        self.serial = if low {
            Serial::Command { bits: 0, got: 0 }
        } else {
            Serial::Idle
        };
        self.data_low = false;
    }

    /// The clock line moved.
    fn clock(&mut self, high: bool) {
        if self.clk == high {
            return;
        }
        self.clk = high;
        if !self.enabled {
            return;
        }
        if high {
            self.rising();
        } else {
            self.falling();
        }
    }

    /// The processor has settled a bit on the data line: take it.
    fn rising(&mut self) {
        let bit = u8::from(self.data_high);
        match self.serial {
            Serial::Command { bits, got } => {
                let bits = (bits << 1) | bit;
                let got = got + 1;
                if got < 8 {
                    self.serial = Serial::Command { bits, got };
                    return;
                }
                // Bit 7 is the direction and bits 6-2 are the address; bits 1-0
                // are always `01` and carry nothing.
                let addr = (bits >> 2) & 0x1f;
                if bits & 0x80 != 0 {
                    self.serial = Serial::Read {
                        bits: self.read(addr),
                        sent: 0,
                    };
                } else {
                    self.serial = Serial::Write {
                        addr,
                        bits: 0,
                        got: 0,
                    };
                }
            }
            Serial::Write { addr, bits, got } => {
                let bits = (bits << 1) | bit;
                let got = got + 1;
                if got < 8 {
                    self.serial = Serial::Write { addr, bits, got };
                } else {
                    self.write(addr, bits);
                    // The frame is over; the processor raises `/RTCENB` next.
                    self.serial = Serial::Command { bits: 0, got: 0 };
                }
            }
            Serial::Idle | Serial::Read { .. } => {}
        }
    }

    /// The processor has taken the bit that was out: put the next one up.
    fn falling(&mut self) {
        if let Serial::Read { bits, sent } = self.serial {
            if sent >= 8 {
                return;
            }
            // The line is pulled down for a zero and let go for a one.
            self.data_low = bits & 0x80 == 0;
            self.serial = Serial::Read {
                bits: bits << 1,
                sent: sent + 1,
            };
        }
    }

    /// The one-second output's next half-cycle.
    ///
    /// A square wave: the counter takes its second on the falling edge, which
    /// is the one a Macintosh's PCR selects on CA2.
    fn tick_second(&mut self) {
        self.irq_low = !self.irq_low;
        if self.irq_low {
            self.seconds = self.seconds.wrapping_add(1);
        }
        self.next = self.next + TICKS_PER_SECOND / 2;
    }

    fn next_event(&self) -> u64 {
        self.next
    }
}

// ---------------------------------------------------------------------------
// the date
// ---------------------------------------------------------------------------

/// Parse `"YYYY-MM-DDTHH:MM:SS"` into seconds since the Macintosh epoch.
///
/// # Errors
///
/// [`Error::Property`] if it is not a date in that shape, or is before 1904 —
/// the chip's counter has no way to say so.
pub fn parse_time(text: &str) -> Result<u32> {
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
    let leap = |y: u32| y.is_multiple_of(4) && (!y.is_multiple_of(100) || y.is_multiple_of(400));
    let month_days = |y: u32, m: u32| match m {
        2 if leap(y) => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    };
    if !(1904..=9999).contains(&year)
        || !(1..=12).contains(&month)
        || day < 1
        || day > month_days(year, month)
        || hour > 23
        || minute > 59
        || second > 59
    {
        return Err(bad());
    }
    let mut days = 0u32;
    for y in 1904..year {
        days += if leap(y) { 366 } else { 365 };
    }
    for m in 1..month {
        days += month_days(year, m);
    }
    days += day - 1;
    Ok(days * 86_400 + hour * 3_600 + minute * 60 + second)
}

/// The same date as seconds since the Unix epoch, for a caller that thinks in
/// those — the two differ by a constant.
#[must_use]
pub fn unix_of(mac_seconds: u32) -> i64 {
    i64::from(mac_seconds) - i64::from(MAC_EPOCH_TO_UNIX)
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
struct Outputs {
    data: Option<WireSource>,
    irq: Option<WireSource>,
}

/// The chip, and the pins that reach it.
struct Shared {
    state: Mutex<State>,
    out: Mutex<Outputs>,
    ticks: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Rtc");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    fn publish(&self, st: &State) {
        self.ticks.store(st.ticks, Ordering::Relaxed);
        self.next_event.store(st.next_event(), Ordering::Relaxed);
    }

    /// Apply `f`, then drive whatever moved — outside the lock, because a wire
    /// delivers synchronously into the VIA and the VIA has a lock of its own.
    fn update(&self, f: impl FnOnce(&mut State)) {
        let moved = {
            let mut st = self.state.lock();
            let before = st.pins();
            f(&mut st);
            self.publish(&st);
            before != st.pins()
        };
        if moved {
            self.refresh();
        }
    }

    fn refresh(&self) {
        let (data, irq) = self.state.lock().pins();
        let out = self.out.lock().clone();
        if let Some(src) = &out.data {
            src.drive(if data { Drive::Low } else { Drive::HiZ });
        }
        if let Some(src) = &out.irq {
            src.drive(if irq { Drive::Low } else { Drive::HiZ });
        }
    }

    fn advance_to(&self, target: u64) {
        loop {
            let moved = {
                let mut st = self.state.lock();
                if st.next <= target {
                    let before = st.pins();
                    st.ticks = st.ticks.max(st.next);
                    st.tick_second();
                    self.publish(&st);
                    Some(before != st.pins())
                } else {
                    if target > st.ticks {
                        st.ticks = target;
                    }
                    self.publish(&st);
                    None
                }
            };
            let Some(moved) = moved else { break };
            if moved {
                self.refresh();
            }
        }
    }
}

/// Which of the three input pins a [`Pin`] is.
const PIN_DATA: u32 = 0;
const PIN_CLK: u32 = 1;
const PIN_ENB: u32 = 2;

/// One input pin, resolving its net wired-AND with a pull-up — every driver on
/// these lines is open-collector.
#[derive(Debug)]
struct Pin {
    shared: Arc<Shared>,
    which: u32,
    inputs: FanIn,
}

impl WireSink for Pin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        let which = self.which;
        self.shared.update(|st| match which {
            PIN_DATA => st.data_high = high,
            PIN_CLK => st.clock(high),
            _ => st.enable(!high),
        });
    }
}

/// The Macintosh clock chip.
#[derive(Debug)]
pub struct Rtc {
    shared: Arc<Shared>,
    /// What the chip comes back to on a cold reset.
    initial: (u32, [u8; PRAM_BYTES]),
    /// The pins, kept alive here: a net holds only a `Weak` to its sinks.
    pins: Mutex<Vec<Arc<Pin>>>,
}

impl Rtc {
    /// Build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `time` is not a date, `pram` is not twenty bytes,
    /// or a property this class does not know was given.
    pub fn new(props: &Props) -> Result<Rtc> {
        let mut r = props.reader();
        let time = r.or_str("time", DEFAULT_TIME)?.to_string();
        let pram = r.or_str("pram", "")?.to_string();
        r.finish()?;
        let seconds = parse_time(&time)?;
        let mut bytes = [0xffu8; PRAM_BYTES];
        if !pram.is_empty() {
            let hex = pram.as_bytes();
            if hex.len() != PRAM_BYTES * 2 {
                return Err(Error::Property(format!(
                    "{CLASS_NAME}: `pram` is {PRAM_BYTES} bytes as {} hexadecimal digits, not {}",
                    PRAM_BYTES * 2,
                    hex.len()
                )));
            }
            for (byte, pair) in bytes.iter_mut().zip(hex.chunks_exact(2)) {
                let digit = |c: u8| -> Result<u8> {
                    (c as char).to_digit(16).map(|d| d as u8).ok_or_else(|| {
                        Error::Property(format!(
                            "{CLASS_NAME}: `pram` is hexadecimal digits, and `{}` is not one",
                            c as char
                        ))
                    })
                };
                *byte = (digit(pair[0])? << 4) | digit(pair[1])?;
            }
        }
        Ok(Rtc::at(seconds, bytes))
    }

    /// The same, from a date already in the chip's own units.
    #[must_use]
    pub fn at(seconds: u32, pram: [u8; PRAM_BYTES]) -> Rtc {
        let state = State::power_on(seconds, pram);
        let shared = Arc::new(Shared {
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(state.next_event()),
            state: Mutex::with_rank(LockRank::DEVICE, state),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        Rtc {
            shared,
            initial: (seconds, pram),
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// A chip with a flat battery, for a test.
    #[must_use]
    pub fn bare() -> Rtc {
        Rtc::at(0, [0xff; PRAM_BYTES])
    }

    /// The second counter now.
    #[must_use]
    pub fn seconds(&self) -> u32 {
        self.shared.state.lock().seconds
    }

    /// Parameter RAM now.
    #[must_use]
    pub fn pram(&self) -> [u8; PRAM_BYTES] {
        self.shared.state.lock().pram
    }

    /// Whether the chip is refusing writes to the counter and parameter RAM.
    #[must_use]
    pub fn write_protected(&self) -> bool {
        self.shared.state.lock().protected()
    }

    /// Ticks of the watch crystal simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Run the chip to `target` ticks.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Move one of the three input lines, for a test with no wire graph.
    pub fn set_enb(&self, low: bool) {
        self.shared.update(|st| st.enable(low));
    }

    /// The same for the clock line.
    pub fn set_clk(&self, high: bool) {
        self.shared.update(|st| st.clock(high));
    }

    /// The same for the data line.
    pub fn set_data(&self, high: bool) {
        self.shared.update(|st| st.data_high = high);
    }

    /// Whether the chip is pulling the data line low.
    #[must_use]
    pub fn data_low(&self) -> bool {
        self.shared.state.lock().data_low
    }

    /// Whether it is pulling the one-second line low.
    #[must_use]
    pub fn irq_low(&self) -> bool {
        self.shared.state.lock().irq_low
    }
}

impl Device for Rtc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The chip runs off a battery and a reset of the computer does not
        // reach it: only a power cycle puts the date and the battery back.
        if kind != ResetKind::Cold {
            return;
        }
        let (seconds, pram) = self.initial;
        self.shared.update(|st| *st = State::power_on(seconds, pram));
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.shared.state.lock().clone();
        w.write_u64(st.ticks)?;
        w.write_u64(st.next)?;
        w.write_u32(st.seconds)?;
        w.write_bytes(&st.pram)?;
        w.write_u8(st.write_protect)?;
        w.write_u8(st.test)?;
        match st.serial {
            Serial::Idle => w.write_u8(0)?,
            Serial::Command { bits, got } => {
                w.write_u8(1)?;
                w.write_u8(bits)?;
                w.write_u8(got)?;
            }
            Serial::Write { addr, bits, got } => {
                w.write_u8(2)?;
                w.write_u8(addr)?;
                w.write_u8(bits)?;
                w.write_u8(got)?;
            }
            Serial::Read { bits, sent } => {
                w.write_u8(3)?;
                w.write_u8(bits)?;
                w.write_u8(sent)?;
            }
        }
        // And the peer-driven levels, for the reason `mac.via` gives about its
        // own pins: a level that came back wrong turns the realize sweep at the
        // end of a restore into an edge the guest never had.
        for v in [st.enabled, st.clk, st.data_high, st.data_low, st.irq_low] {
            w.write_bool(v)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(format!("{CLASS_NAME}: {what}"));
        let mut st = State::power_on(0, [0xff; PRAM_BYTES]);
        st.ticks = r.read_u64()?;
        st.next = r.read_u64()?;
        st.seconds = r.read_u32()?;
        let pram = r.read_bytes()?;
        if pram.len() != PRAM_BYTES {
            return Err(bad("parameter RAM of the wrong size"));
        }
        st.pram.copy_from_slice(pram);
        st.write_protect = r.read_u8()?;
        st.test = r.read_u8()?;
        st.serial = match r.read_u8()? {
            0 => Serial::Idle,
            1 => {
                let (bits, got) = (r.read_u8()?, r.read_u8()?);
                if got > 7 {
                    return Err(bad("a command byte past its eighth bit"));
                }
                Serial::Command { bits, got }
            }
            2 => {
                let (addr, bits, got) = (r.read_u8()?, r.read_u8()?, r.read_u8()?);
                if got > 7 {
                    return Err(bad("a data byte past its eighth bit"));
                }
                Serial::Write { addr, bits, got }
            }
            3 => {
                let (bits, sent) = (r.read_u8()?, r.read_u8()?);
                if sent > 8 {
                    return Err(bad("a data byte past its eighth bit"));
                }
                Serial::Read { bits, sent }
            }
            _ => return Err(bad("an unknown serial phase")),
        };
        st.enabled = r.read_bool()?;
        st.clk = r.read_bool()?;
        st.data_high = r.read_bool()?;
        st.data_low = r.read_bool()?;
        st.irq_low = r.read_bool()?;
        if st.next <= st.ticks {
            // An event that is not in the future would stall catch-up.
            st.next = st.ticks + 1;
        }
        self.shared.update(|now| *now = st);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.shared.out.lock();
            match port {
                DATA_PIN => out.data = Some(source),
                IRQ_PIN => out.irq = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!("a Macintosh clock chip drives `{DATA_PIN}` and `{IRQ_PIN}`"),
                    });
                }
            }
        }
        self.shared.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let which = match port {
            DATA_PIN => PIN_DATA,
            CLK_PIN => PIN_CLK,
            ENB_PIN => PIN_ENB,
            _ => return None,
        };
        let pin = Arc::new(Pin {
            shared: Arc::clone(&self.shared),
            which,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: which,
        })
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Rtc::advance_to(self, tick);
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

impl Instance for Rtc {}

/// The `mac.rtc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Macintosh clock chip: a four-byte second counter, twenty bytes of parameter \
              RAM and the one-second interrupt, on three of the VIA's port B pins",
    properties: &[
        PropertySpec {
            name: "time",
            kind: ValueKind::Str,
            required: false,
            summary: "the date it starts at, `YYYY-MM-DDTHH:MM:SS` (default 2026-01-01)",
        },
        PropertySpec {
            name: "pram",
            kind: ValueKind::Str,
            required: false,
            summary: "parameter RAM as 40 hexadecimal digits (default a flat battery: all $FF)",
        },
    ],
    construct: |props| Ok(Box::new(Rtc::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if something already claimed
/// the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rtc::new(props)?)))
}

/// What the validator should know about `mac.rtc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("time", ValueKind::Str))
        .prop(PropSchema::new("pram", ValueKind::Str))
        .port(DATA_PIN, PortDir::InOut)
        .port(CLK_PIN, PortDir::In)
        .port(ENB_PIN, PortDir::In)
        .port(IRQ_PIN, PortDir::Out)
}

/// A short name for a command byte, for a monitor or a failing test.
#[must_use]
pub fn describe(command: u8) -> String {
    let addr = (command >> 2) & 0x1f;
    let dir = if command & 0x80 != 0 { "read" } else { "write" };
    match addr {
        0x00..=0x07 => format!("{dir} second counter byte {}", addr & 3),
        0x08..=0x0b => format!("{dir} parameter RAM {}", addr - 0x08),
        ADDR_TEST => format!("{dir} test register"),
        ADDR_WRITE_PROTECT => format!("{dir} write-protect register"),
        0x10..=0x1f => format!("{dir} parameter RAM {}", addr - 0x10 + 4),
        _ => format!("{dir} ${addr:02x}, which is not decoded"),
    }
}

#[cfg(test)]
mod tests;
