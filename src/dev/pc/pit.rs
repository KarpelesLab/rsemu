//! An Intel 8254 programmable interval timer.
//!
//! # Sources
//!
//! * *Intel 8254 Programmable Interval Timer* data sheet (order 231164). The
//!   register block, the control word, the six mode definitions with their
//!   waveform diagrams, the gate table, the counter-latch command and the
//!   read-back command all come from it. Every non-obvious rule below cites the
//!   part of it that says so.
//! * *IBM Personal Computer AT Technical Reference* (1984) for the wiring: the
//!   chip answers ports `0x40`-`0x43`, its input clock is the system
//!   `OSC/12` = 1.193181… MHz, counter 0 drives IRQ0, counter 1 drove the DRAM
//!   refresh request, and counter 2 goes to the speaker with its gate on port
//!   `0x61` bit 0 and its output readable in bit 5 of the same port.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # The register block
//!
//! ```text
//!   0  counter 0   read the count, write the reload value
//!   1  counter 1   ditto
//!   2  counter 2   ditto
//!   3  control     write only; a read is not driven by the chip
//! ```
//!
//! Everything goes through the same three data ports, so *which* of a counter's
//! bytes a read or a write means is a property of the counter, not of the
//! address: the access mode in the control word says one byte, the other byte,
//! or low-then-high, and in the last case each counter keeps its own read and
//! write toggle. That is the single most common source of confusion in PIT
//! code, and it is why those toggles are architectural state that snapshots.
//!
//! # Time
//!
//! The chip is a **lazily advanced** device (`ROADMAP.md` §4.2). It counts in
//! its own clock domain — the PIT input clock, which a PC machine file declares
//! as the exact rational 105000000/88 Hz — and nothing here ever converts a
//! tick to a second, or reads a host clock, or touches a float.
//!
//! Two halves, both needed:
//!
//! * **Sampled**: a guest latches and reads a counter at an arbitrary instant
//!   and must see the value at that instant. So [`MemOps::read`] and
//!   [`MemOps::write`] catch the chip up through the [`LazyHandle`] first.
//! * **Scheduled**: [`Device::next_event_tick`] reports the tick an output pin
//!   next changes, so IRQ0 is raised on the tick counter 0 actually expires on
//!   rather than at the end of whatever quantum contained it.
//!
//! [`Device::current_tick`] and [`Device::next_event_tick`] are asked with the
//! scheduler's slot held at [`LockRank::LEAF`], the rank nothing nests under,
//! so **neither may take a lock**: both are published into atomics by every
//! critical section that can move the chip.
//!
//! Catch-up walks from output edge to output edge rather than jumping to the
//! target, and drives the wires between the steps. A single `advance_to`
//! spanning several counter-0 periods therefore raises every one of them; the
//! alternative — settling on the final level — would silently swallow timer
//! interrupts whenever a quantum happened to be long.
//!
//! # Pins
//!
//! Three outputs, `out0`, `out1` and `out2`, and one input, `gate2`. Counters 0
//! and 1 have their gates tied high on an AT, so they have no input pin; a
//! counter whose gate is not driven idles high here, which is what an
//! unconnected, pulled-up input does. Counter 2's output is also readable by
//! the board's system-control port, through [`Pit8254::out`].

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.pit";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
///
/// Two address lines: three counters and the control word.
pub const REGISTER_WINDOW_LEN: u64 = 4;

/// How many counters an 8254 has.
pub const COUNTERS: usize = 3;

/// The value a read of the write-only control port returns.
///
/// The chip does not drive the bus for it, so what the CPU latches is whatever
/// the bus floats to — ones, on a PC.
const OPEN_BUS: u8 = 0xff;

// ---------------------------------------------------------------------------
// One counter
// ---------------------------------------------------------------------------

/// One of the three counters, and the whole of the chip's behaviour.
///
/// The counting element is held as a plain integer in `1..=modulus` rather than
/// as the 16 bits the guest sees, so that a reload value of zero is the honest
/// 65536 (10000 in BCD) instead of a special case in every expression. The two
/// agree bit for bit on read-back, because 65536 truncated to 16 bits is the
/// zero real hardware shows in the same instant.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Counter {
    /// The count register: what the guest last wrote, in its own
    /// representation (packed BCD when the counter is in BCD mode).
    reload: u16,
    /// The counting element.
    count: u32,
    /// The mode, 0-5. Control-word modes 6 and 7 alias 2 and 3 and are stored
    /// as the mode they alias.
    mode: u8,
    /// The access mode: 1 = low byte only, 2 = high byte only, 3 = low then
    /// high. Zero is the counter-latch command and is never stored.
    access: u8,
    /// Whether the counter counts in BCD.
    bcd: bool,
    /// The OUT pin's level.
    output: bool,
    /// The GATE pin's level.
    gate: bool,
    /// Whether the counting element holds a count and follows the clock.
    ///
    /// Distinct from [`Counter::armed`]: a mode-0 counter that has reached its
    /// terminal count keeps counting — and keeps showing the guest a wrapping
    /// value — long after its output has finished moving.
    loaded: bool,
    /// The count register has been written but not yet transferred into the
    /// counting element. This is the status byte's null-count bit.
    null_count: bool,
    /// The next clock pulse loads the counting element.
    pending_load: bool,
    /// An output transition is still to come.
    armed: bool,
    /// A count frozen by the counter-latch or read-back command.
    latched_count: Option<u16>,
    /// A status byte frozen by the read-back command.
    latched_status: Option<u8>,
    /// Access mode 3: the next byte read is the high one.
    read_high: bool,
    /// Access mode 3: the next byte written is the high one.
    write_high: bool,
    /// The low byte of a two-byte write, held until the high byte arrives.
    write_low: u8,
}

impl Default for Counter {
    fn default() -> Counter {
        Counter {
            reload: 0,
            count: 0,
            mode: 0,
            // The data sheet leaves the power-up state undefined — the counters
            // are "in an undefined state" until programmed — but an emulator
            // has to pick one and pick it deterministically. This is the state
            // a `mode 0, low-then-high, binary` control word leaves behind,
            // which is the least surprising thing for a guest that reads before
            // it writes, and it idles every output low so that a fresh net and
            // a fresh chip already agree.
            access: 3,
            bcd: false,
            output: false,
            // Counters 0 and 1 have their gates tied high on an AT, and an
            // undriven input here is pulled up rather than floating: a PIT with
            // nothing wired to it counts.
            gate: true,
            loaded: false,
            null_count: true,
            pending_load: false,
            armed: false,
            latched_count: None,
            latched_status: None,
            read_high: false,
            write_high: false,
            write_low: 0,
        }
    }
}

impl Counter {
    /// The modulus of the counting element: 65536 binary, 10000 in BCD.
    fn modulus(&self) -> u32 {
        if self.bcd { 10_000 } else { 65_536 }
    }

    /// The value a load transfers into the counting element.
    ///
    /// Zero means the modulus — 65536, or 10000 in BCD — because the counting
    /// element wraps once on the way down and so takes a whole modulus of
    /// clocks to reach zero. That is the reason `outb(0, 0x40)` gives the PC
    /// its 18.2 Hz tick rather than no tick at all.
    fn initial(&self) -> u32 {
        let m = self.modulus();
        // A BCD count with a nibble above 9 is not legal input and the data
        // sheet does not define it; folding it into the modulus keeps every
        // expression below in range at no cost.
        let raw = if self.bcd {
            bcd_to_bin(self.reload)
        } else {
            u32::from(self.reload)
        };
        match raw % m {
            0 => m,
            n => n,
        }
    }

    /// The counting element as the guest reads it back.
    fn element(&self) -> u16 {
        let v = self.count % self.modulus();
        if self.bcd { bin_to_bcd(v) } else { v as u16 }
    }

    /// The read-back status byte.
    fn status(&self) -> u8 {
        (u8::from(self.output) << 7)
            | (u8::from(self.null_count) << 6)
            | (self.access << 4)
            | (self.mode << 1)
            | u8::from(self.bcd)
    }

    /// Whether the gate currently lets this counter count.
    ///
    /// In modes 1 and 5 the gate is a trigger and does not inhibit counting at
    /// all (data sheet, the GATE table); everywhere else it is a level enable.
    fn counting(&self) -> bool {
        match self.mode {
            1 | 5 => true,
            _ => self.gate,
        }
    }

    /// Clocks until the counting element reaches zero.
    fn ticks_to_zero(&self) -> u64 {
        match self.count {
            0 => u64::from(self.modulus()),
            c => u64::from(c),
        }
    }

    /// Clocks until a mode-3 counter's output toggles.
    ///
    /// The square wave decrements by two per clock, so half the count is spent
    /// in each phase. An odd count cannot split evenly, and the data sheet
    /// resolves it by decrementing by one on the first clock of the high half
    /// and by three on the first clock of the low half: the extra clock lands
    /// in the high half, giving `(N+1)/2` high and `(N-1)/2` low.
    fn ticks_to_toggle(&self) -> u64 {
        let c = self.count;
        if c.is_multiple_of(2) {
            return u64::from(c / 2).max(1);
        }
        let first = if self.output { 1 } else { 3 };
        if c <= first {
            // A count of one is illegal in mode 3. Rather than divide by zero,
            // give it a one-clock phase, which is what the shortest phase this
            // model can express looks like.
            return 1;
        }
        u64::from(1 + (c - first) / 2)
    }

    /// Clocks until this counter's next internal event: the pulse that loads
    /// the counting element, an output edge, or a reload.
    ///
    /// `None` when nothing more will happen until the guest or the gate
    /// intervenes. Never `Some(0)` — catch-up that cannot move is a stall.
    fn next_event(&self) -> Option<u64> {
        if self.pending_load {
            return Some(1);
        }
        if !self.loaded || !self.counting() {
            return None;
        }
        match self.mode {
            // Interrupt on terminal count, and the one-shot: one edge, at zero,
            // and then nothing until the guest writes another count.
            0 | 1 => self.armed.then(|| self.ticks_to_zero()),
            // Rate generator: OUT falls when the count reaches one and rises
            // one clock later, as the counting element reloads.
            2 => Some(if self.output {
                u64::from(self.count).saturating_sub(1).max(1)
            } else {
                1
            }),
            // Square wave: always periodic while it is counting.
            3 => Some(self.ticks_to_toggle()),
            // The two strobes: one clock low at terminal count, then done.
            4 | 5 => {
                if !self.output {
                    Some(1)
                } else if self.armed {
                    Some(self.ticks_to_zero())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Transfer the count register into the counting element.
    ///
    /// This is the clock pulse the data sheet puts between a write and the
    /// start of counting, and it is what the status byte's null-count bit
    /// reports the absence of.
    fn load(&mut self) {
        self.pending_load = false;
        self.count = self.initial();
        self.null_count = false;
        self.loaded = true;
        self.armed = true;
        match self.mode {
            // The one-shot's output falls on the clock that loads the counter,
            // not on the gate edge that triggered it.
            1 => self.output = false,
            // A rate-generator or square-wave period always begins high.
            2 | 3 => self.output = true,
            _ => {}
        }
    }

    /// Decrement the counting element by `ticks` clocks.
    ///
    /// Never called with more clocks than [`Counter::next_event`] allows, so a
    /// counter that is going to do something does not run past it. A counter
    /// with no event left free-runs, which is a plain modular decrement — the
    /// wrapping is the guest-visible behaviour of a mode-0 counter read after
    /// its terminal count.
    fn run(&mut self, ticks: u64) {
        if ticks == 0 {
            return;
        }
        if self.mode == 3 {
            let mut n = ticks;
            if !self.count.is_multiple_of(2) {
                let first = if self.output { 1 } else { 3 };
                self.count = self.count.saturating_sub(first);
                n -= 1;
            }
            self.count = self.count.saturating_sub((2 * n) as u32);
            return;
        }
        if self.mode == 2 && !self.output {
            // The clock that reloads a rate generator does not also decrement
            // it; the reload is the whole of that clock's business.
            return;
        }
        let m = u64::from(self.modulus());
        let c = u64::from(self.count);
        let d = ticks % m;
        self.count = if c >= d {
            (c - d) as u32
        } else {
            (c + m - d) as u32
        };
    }

    /// Apply the event [`Counter::next_event`] counted down to.
    fn fire(&mut self) {
        match self.mode {
            // Terminal count: OUT goes high and stays high. The counter keeps
            // counting, but its output is finished.
            0 | 1 => {
                self.output = true;
                self.armed = false;
            }
            2 => {
                if self.output {
                    self.output = false;
                } else {
                    self.count = self.initial();
                    self.output = true;
                    // The reload is a load, so it clears the null-count bit and
                    // is where a count written mid-period takes effect.
                    self.null_count = false;
                }
            }
            3 => {
                self.output = !self.output;
                self.count = self.initial();
                self.null_count = false;
            }
            4 | 5 => {
                if self.output {
                    self.output = false;
                } else {
                    self.output = true;
                    self.armed = false;
                }
            }
            _ => {}
        }
    }

    /// Advance this counter by `ticks` clocks.
    ///
    /// The caller never passes more than [`Counter::next_event`] reported, so
    /// at most one event lands, and it lands on the last clock.
    fn advance(&mut self, ticks: u64) {
        if ticks == 0 {
            return;
        }
        if self.pending_load {
            // A load takes the whole clock pulse. The caller gave us exactly
            // the one tick `next_event` asked for.
            self.load();
            return;
        }
        if !self.loaded || !self.counting() {
            return;
        }
        let fires = self.next_event() == Some(ticks);
        self.run(ticks);
        if fires {
            self.fire();
        }
    }

    /// Drive the gate pin.
    fn set_gate(&mut self, level: bool) {
        if level == self.gate {
            return;
        }
        self.gate = level;
        match self.mode {
            // Hardware retriggerable: a rising edge starts, or restarts, the
            // count. A low gate does not stop it.
            1 | 5 => {
                if level {
                    self.pending_load = true;
                }
            }
            2 | 3 => {
                if level {
                    // A rising edge restarts the period, so the first pulse
                    // after the speaker is enabled is a whole period long.
                    if self.loaded {
                        self.pending_load = true;
                    }
                } else {
                    // A low gate stops the count *and* forces OUT high, which
                    // is what silences the speaker when port 0x61 bit 0 is
                    // cleared rather than leaving the cone pushed out.
                    self.output = true;
                }
            }
            // Modes 0 and 4: a level enable and nothing more.
            _ => {}
        }
    }

    /// Apply a control word that programs this counter.
    fn program(&mut self, word: u8) {
        self.access = (word >> 4) & 3;
        // Modes 6 and 7 are not separate modes: the mode field's top bit is a
        // don't-care, so 110 is mode 2 and 111 is mode 3.
        self.mode = match (word >> 1) & 7 {
            6 => 2,
            7 => 3,
            m => m,
        };
        self.bcd = word & 1 != 0;
        self.loaded = false;
        self.armed = false;
        self.pending_load = false;
        // The data sheet: writing a control word sets the null-count bit, and
        // only the clock pulse that loads the counting element clears it.
        self.null_count = true;
        self.read_high = false;
        self.write_high = false;
        // A latched value belongs to the programming that latched it, and the
        // read sequence starts over with the new control word.
        self.latched_count = None;
        self.latched_status = None;
        // OUT is low in mode 0 and high in every other mode from the moment the
        // control word is written — no clock pulse required.
        self.output = self.mode != 0;
    }

    /// Freeze the counting element for a later read.
    ///
    /// A second latch command before the first latched value has been read is
    /// ignored, so a guest gets the value it asked for rather than a fresher
    /// one (data sheet, counter-latch command).
    fn latch_count(&mut self) {
        if self.latched_count.is_none() {
            self.latched_count = Some(self.element());
        }
    }

    /// Freeze the status byte for a later read, on the same terms.
    fn latch_status(&mut self) {
        if self.latched_status.is_none() {
            self.latched_status = Some(self.status());
        }
    }

    /// Read one byte through this counter's port.
    ///
    /// `debug` suppresses every side effect: a monitor must not consume the
    /// latch or step the byte toggle out from under the guest.
    fn read(&mut self, debug: bool) -> u8 {
        if let Some(status) = self.latched_status {
            // A latched status is returned before any latched count, so a
            // read-back that asked for both is unpacked in that order.
            if !debug {
                self.latched_status = None;
            }
            return status;
        }
        let value = self.latched_count.unwrap_or_else(|| self.element());
        match self.access {
            1 => {
                if !debug {
                    self.latched_count = None;
                }
                value as u8
            }
            2 => {
                if !debug {
                    self.latched_count = None;
                }
                (value >> 8) as u8
            }
            _ => {
                if self.read_high {
                    if !debug {
                        self.read_high = false;
                        self.latched_count = None;
                    }
                    (value >> 8) as u8
                } else {
                    if !debug {
                        self.read_high = true;
                    }
                    value as u8
                }
            }
        }
    }

    /// Write one byte through this counter's port.
    fn write(&mut self, value: u8) {
        let complete = match self.access {
            1 => Some(u16::from(value)),
            2 => Some(u16::from(value) << 8),
            _ => {
                if self.write_high {
                    self.write_high = false;
                    Some(u16::from_le_bytes([self.write_low, value]))
                } else {
                    self.write_low = value;
                    self.write_high = true;
                    if self.mode == 0 {
                        // Mode 0 only: writing the first byte disables counting
                        // and takes OUT low immediately, with no clock pulse.
                        self.output = false;
                        self.loaded = false;
                        self.armed = false;
                        self.pending_load = false;
                    }
                    None
                }
            }
        };
        let Some(reload) = complete else {
            return;
        };
        self.reload = reload;
        self.null_count = true;
        match self.mode {
            // Software triggered: the write itself starts the count, on the
            // next clock pulse. In mode 0 that also takes OUT low again, which
            // is how a one-shot is re-armed without rewriting the control word.
            0 => {
                self.output = false;
                self.pending_load = true;
            }
            4 => self.pending_load = true,
            // The periodic modes, first count after a control word: it starts
            // the counter on the next clock pulse, like the software-triggered
            // modes above.
            2 | 3 if !self.loaded => self.pending_load = true,
            // Everything else waits. A count written to a mode-2 or mode-3
            // counter that is already running does not disturb the current
            // period — the reload at the end of it picks the new value up,
            // which is why reprogramming the PC's timer produces no short tick.
            // Modes 1 and 5 wait for a gate edge instead.
            _ => {}
        }
    }
}

/// Decode a packed-BCD count.
///
/// Nibbles above nine are not legal input and the data sheet does not define
/// them; each is taken at face value, which is cheap and stays in range once
/// the caller folds the result into the modulus.
fn bcd_to_bin(value: u16) -> u32 {
    let mut out = 0;
    let mut place = 1;
    for shift in [0, 4, 8, 12] {
        out += u32::from((value >> shift) & 0xf) * place;
        place *= 10;
    }
    out
}

/// Encode a count of `0..10000` as packed BCD.
fn bin_to_bcd(value: u32) -> u16 {
    let mut out = 0u16;
    let mut v = value % 10_000;
    for shift in [0, 4, 8, 12] {
        out |= ((v % 10) as u16) << shift;
        v /= 10;
    }
    out
}

// ---------------------------------------------------------------------------
// The chip
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct State {
    counters: [Counter; COUNTERS],
    /// The tick, in the chip's own clock domain, that it has been advanced to.
    tick: u64,
}

impl State {
    /// Clocks until the soonest event on any counter.
    fn next_event(&self) -> Option<u64> {
        self.counters.iter().filter_map(Counter::next_event).min()
    }

    /// Advance every counter by `ticks`, which must not pass an event.
    fn step(&mut self, ticks: u64) {
        for counter in &mut self.counters {
            counter.advance(ticks);
        }
        self.tick += ticks;
    }

    /// The three output levels, for driving once the lock is released.
    fn levels(&self) -> [bool; COUNTERS] {
        [
            self.counters[0].output,
            self.counters[1].output,
            self.counters[2].output,
        ]
    }

    /// Apply a write to the control port.
    fn control(&mut self, word: u8) {
        let select = (word >> 6) as usize;
        if select == COUNTERS {
            self.read_back(word);
            return;
        }
        let counter = &mut self.counters[select];
        if (word >> 4) & 3 == 0 {
            counter.latch_count();
        } else {
            counter.program(word);
        }
    }

    /// Apply the 8254's read-back command.
    ///
    /// Bits 1-3 select counters 0-2. Bit 5 clear latches their counts and bit 4
    /// clear latches their status bytes — both are active low, which catches
    /// every reader out once.
    fn read_back(&mut self, word: u8) {
        let want_count = word & 0x20 == 0;
        let want_status = word & 0x10 == 0;
        for (i, counter) in self.counters.iter_mut().enumerate() {
            if word & (2 << i) == 0 {
                continue;
            }
            if want_status {
                counter.latch_status();
            }
            if want_count {
                counter.latch_count();
            }
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The three output pins, at [`LockRank::LEAF`] so a line can be driven
    /// with nothing else held.
    outs: Mutex<[Option<WireSource>; COUNTERS]>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// [`State::tick`], published on every change: the scheduler asks
    /// [`Device::current_tick`] with its slot held at [`LockRank::LEAF`], so
    /// that call may not take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next event, or [`u64::MAX`] for none. Same
    /// no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("tick", &self.tick.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("counters", &state.counters).finish(),
            None => s.field("counters", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Republish what the lock-free lazy surface reads.
    ///
    /// Called from inside every critical section that can move the chip or
    /// change what its next event is.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event() {
            Some(d) => state.tick.saturating_add(d),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Drive the output pins. Never called with the state lock held.
    fn drive(&self, levels: [bool; COUNTERS]) {
        let sources = self.outs.lock().clone();
        for (source, level) in sources.iter().zip(levels) {
            if let Some(source) = source {
                source.set(Level::from_bool(level));
            }
        }
    }

    /// Advance to `target` of the chip's own clock domain, delivering every
    /// output edge on the way.
    ///
    /// One iteration per edge rather than one jump to the target: a catch-up
    /// spanning several counter-0 periods has to raise IRQ0 once per period,
    /// and settling on the final level would swallow all but the last. The
    /// scheduler bounds a quantum by [`Device::next_event_tick`], so in a
    /// running machine the loop turns once.
    fn advance_to(&self, target: u64) {
        loop {
            let (reached, levels) = {
                let mut state = self.state.lock();
                if target <= state.tick {
                    return;
                }
                let span = target - state.tick;
                // At least one tick, so catch-up always makes progress: a step
                // of zero would spin here forever.
                let step = state.next_event().unwrap_or(span).clamp(1, span);
                state.step(step);
                self.publish(&state);
                (state.tick >= target, state.levels())
            };
            self.drive(levels);
            if reached {
                return;
            }
        }
    }

    /// Catch the chip up before an access is dispatched to it (§4.2).
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
        // A refusal means catch-up for this chip is already running further up
        // the stack. The access still has to be answered, and answering it from
        // where the chip stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Drive one counter's gate, catching the chip up first so the edge lands
    /// on the tick it happened on.
    fn set_gate(&self, counter: usize, level: bool) {
        self.sync(MemAttrs::DEFAULT);
        let levels = {
            let mut state = self.state.lock();
            state.counters[counter].set_gate(level);
            self.publish(&state);
            state.levels()
        };
        self.drive(levels);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        if !attrs.debug {
            self.sync(attrs);
        }
        let index = (offset & 3) as usize;
        if index == COUNTERS {
            *byte = OPEN_BUS;
            return Ok(());
        }
        // A read latches at most; no output can move, so nothing is driven.
        let mut state = self.state.lock();
        *byte = state.counters[index].read(attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to the control port would take a counter's output
            // low and change when the guest is next interrupted. There is no
            // harmless version of it (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let index = (offset & 3) as usize;
        let levels = {
            let mut state = self.state.lock();
            if index == COUNTERS {
                state.control(*value);
            } else {
                state.counters[index].write(*value);
            }
            self.publish(&state);
            state.levels()
        };
        self.drive(levels);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on an 8-bit bus. A word access to the register file is
        // not a thing that happens, and accepting one would invent an order for
        // two writes whose order is the entire protocol.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// Counter 2's gate input, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the driver that changed rather than the resolved level of the
/// net (§4.3).
#[derive(Debug)]
pub struct GatePin {
    regs: Arc<Registers>,
    counter: usize,
    inputs: FanIn,
}

impl WireSink for GatePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.regs
            .set_gate(self.counter, self.inputs.resolve(Resolve::Or).is_high());
    }
}

/// An Intel 8254 programmable interval timer.
#[derive(Debug)]
pub struct Pit8254 {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The sinks handed out by [`Device::sink`], kept alive here — a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<GatePin>>>,
}

impl Pit8254 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Pit8254> {
        props.reader().finish()?;
        Ok(Pit8254::default_device())
    }

    /// One with no properties set.
    #[must_use]
    pub fn default_device() -> Pit8254 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            outs: Mutex::with_rank(LockRank::LEAF, [None, None, None]),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Pit8254 {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The level counter `counter`'s OUT pin is driving.
    ///
    /// The board's system-control port reads counter 2's output in bit 5 of
    /// port `0x61`, which is how the BIOS times a delay loop without an
    /// interrupt and how a program watches the speaker's own waveform. The chip
    /// is caught up first, so the answer is the level at the instant of the
    /// port read rather than at the last quantum boundary.
    ///
    /// Out-of-range counter numbers read low.
    #[must_use]
    pub fn out(&self, counter: usize) -> bool {
        if counter >= COUNTERS {
            return false;
        }
        self.regs.sync(MemAttrs::DEFAULT);
        self.regs.state.lock().counters[counter].output
    }

    /// Advance to `tick` of the chip's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a scheduler
    /// calls it directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// The tick the chip has been advanced to.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }
}

/// The `pc.pit` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Intel 8254 programmable interval timer",
    properties: &[],
    construct: |props| Ok(Box::new(Pit8254::new(props)?)),
};

impl Device for Pit8254 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. There is no battery behind a PIT, and a counter that
        // survived a reset would keep interrupting a kernel that had not
        // programmed it.
        let levels = {
            let mut state = self.regs.state.lock();
            *state = State::default();
            self.regs.publish(&state);
            state.levels()
        };
        self.regs.drive(levels);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let index = out_pin(port).ok_or_else(|| unknown_pin(port))?;
        self.regs.outs.lock()[index] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some(index) = out_pin(port) else {
            return;
        };
        // Mode 0 idles OUT low and every other mode idles it high, so this is
        // not a pin that can be left to a fresh net's default.
        let level = self.regs.state.lock().counters[index].output;
        let source = self.regs.outs.lock()[index].clone();
        if let Some(source) = source {
            source.set(Level::from_bool(level));
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // Counters 0 and 1 have their gates tied high on an AT, so only
        // counter 2's is a pin. Adding the other two is a line each the day a
        // board needs them.
        if port != "gate2" {
            return None;
        }
        let pin = Arc::new(GatePin {
            regs: Arc::clone(&self.regs),
            counter: 2,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 2 })
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
        w.write_seq_len(COUNTERS as u64)?;
        for c in &state.counters {
            w.write_u16(c.reload)?;
            w.write_u32(c.count)?;
            w.write_u8(c.mode)?;
            w.write_u8(c.access)?;
            for flag in [
                c.bcd,
                c.output,
                c.gate,
                c.loaded,
                c.null_count,
                c.pending_load,
                c.armed,
            ] {
                w.write_bool(flag)?;
            }
            match c.latched_count {
                None => w.write_bool(false)?,
                Some(v) => {
                    w.write_bool(true)?;
                    w.write_u16(v)?;
                }
            }
            match c.latched_status {
                None => w.write_bool(false)?,
                Some(v) => {
                    w.write_bool(true)?;
                    w.write_u8(v)?;
                }
            }
            // The byte toggles are as architectural as the counts: a snapshot
            // taken between the two halves of a low-then-high write has to come
            // back expecting the same half.
            w.write_bool(c.read_high)?;
            w.write_bool(c.write_high)?;
            w.write_u8(c.write_low)?;
        }
        // The chip's own position in its domain. The scheduler restores the
        // domain; without this the two would disagree and the chip would stand
        // still until the domain caught up with it.
        w.write_u64(state.tick)
        // The wire handles are the machine's wiring, not the chip's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let count = r.read_seq_len(12)? as usize;
        if count != COUNTERS {
            return Err(Error::State(format!(
                "snapshot has {count} counter(s) of 8254 state, this chip has {COUNTERS}"
            )));
        }
        let mut state = State::default();
        for c in &mut state.counters {
            c.reload = r.read_u16()?;
            c.count = r.read_u32()?;
            c.mode = r.read_u8()?;
            c.access = r.read_u8()?;
            c.bcd = r.read_bool()?;
            c.output = r.read_bool()?;
            c.gate = r.read_bool()?;
            c.loaded = r.read_bool()?;
            c.null_count = r.read_bool()?;
            c.pending_load = r.read_bool()?;
            c.armed = r.read_bool()?;
            c.latched_count = if r.read_bool()? {
                Some(r.read_u16()?)
            } else {
                None
            };
            c.latched_status = if r.read_bool()? {
                Some(r.read_u8()?)
            } else {
                None
            };
            c.read_high = r.read_bool()?;
            c.write_high = r.read_bool()?;
            c.write_low = r.read_u8()?;
            if c.mode > 5 || c.access == 0 || c.access > 3 {
                return Err(Error::State(format!(
                    "snapshot has an 8254 counter in mode {} with access mode {}",
                    c.mode, c.access
                )));
            }
            if c.count > c.modulus() {
                return Err(Error::State(format!(
                    "snapshot has an 8254 counting element of {} past its modulus",
                    c.count
                )));
            }
        }
        state.tick = r.read_u64()?;
        let levels = {
            let mut live = self.regs.state.lock();
            *live = state;
            self.regs.publish(&live);
            live.levels()
        };
        self.regs.drive(levels);
        Ok(())
    }
}

impl Instance for Pit8254 {}

/// Which counter's output pin `port` names.
fn out_pin(port: &str) -> Option<usize> {
    match port {
        "out0" => Some(0),
        "out1" => Some(1),
        "out2" => Some(2),
        _ => None,
    }
}

/// The error for a pin this chip does not drive.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: format!("an 8254 drives `out0`, `out1` and `out2`; `{port}` is none of them"),
    }
}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pit8254::new(props)?)))
}

/// What the validator should know about `pc.pit`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::PortDir;
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
        .port("out0", PortDir::Out)
        .port("out1", PortDir::Out)
        .port("out2", PortDir::Out)
        .port("gate2", PortDir::In)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// The counter-latch command for `counter`.
    const fn latch(counter: u8) -> u8 {
        counter << 6
    }

    /// A control word: counter, access mode, timer mode, BCD.
    const fn control(counter: u8, access: u8, mode: u8, bcd: bool) -> u8 {
        (counter << 6) | (access << 4) | (mode << 1) | (bcd as u8)
    }

    fn peek(pit: &Pit8254, port: u64) -> u8 {
        let mut byte = [0u8; 1];
        pit.regs
            .read(port, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peek_debug(pit: &Pit8254, port: u64) -> u8 {
        let mut byte = [0u8; 1];
        pit.regs
            .read(port, &mut byte, MemAttrs::DEBUG)
            .expect("a debugger may look");
        byte[0]
    }

    fn poke(pit: &Pit8254, port: u64, value: u8) {
        pit.regs
            .write(port, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Program `counter` and give it a 16-bit reload through the low/high
    /// access mode, returning the tick the counting element is loaded on.
    fn program(pit: &Pit8254, counter: u8, mode: u8, reload: u16) -> u64 {
        poke(pit, 3, control(counter, 3, mode, false));
        poke(pit, u64::from(counter), reload as u8);
        poke(pit, u64::from(counter), (reload >> 8) as u8);
        // The count reaches the counting element on the next clock pulse.
        let at = pit.tick() + 1;
        pit.advance_to(at);
        at
    }

    /// The level of `counter`'s output at each of `n` consecutive ticks,
    /// starting from wherever the chip already stands.
    fn samples(pit: &Pit8254, counter: usize, n: u64) -> alloc::vec::Vec<bool> {
        let start = pit.tick();
        let mut out = alloc::vec::Vec::new();
        for i in 0..n {
            pit.advance_to(start + i);
            out.push(pit.out(counter));
        }
        out
    }

    /// Wire `gate2` up so a test can drive it as the board would.
    fn gate2(pit: &Pit8254) -> WireSource {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let pin = pit.sink("gate2", &[id]).expect("counter 2 has a gate pin");
        let wire = Wire::builder()
            .source(id)
            .sink(pin.sink, pin.line)
            .build_shared();
        let source = WireSource::new(wire, id);
        source.raise();
        source
    }

    #[test]
    fn mode_3_is_a_square_wave_and_an_odd_count_spends_the_extra_tick_high() {
        // The BIOS's own control word for counter 0: low-then-high, mode 3.
        let pit = Pit8254::default_device();
        assert_eq!(program(&pit, 0, 3, 6), 1);
        assert_eq!(
            samples(&pit, 0, 12),
            [
                true, true, true, false, false, false, true, true, true, false, false, false
            ],
            "an even count splits evenly, over several periods"
        );

        let pit = Pit8254::default_device();
        program(&pit, 0, 3, 5);
        assert_eq!(
            samples(&pit, 0, 10),
            [
                true, true, true, false, false, true, true, true, false, false
            ],
            "five is three high and two low, and the period is still five"
        );
    }

    #[test]
    fn mode_2_pulses_low_for_one_tick_of_each_period() {
        let pit = Pit8254::default_device();
        program(&pit, 0, 2, 4);
        assert_eq!(
            samples(&pit, 0, 9),
            [true, true, true, false, true, true, true, false, true],
            "one low tick every four, which is what makes IRQ0 a rate"
        );
    }

    #[test]
    fn a_mode_2_reload_of_zero_means_65536() {
        // How the PC gets its 18.2 Hz tick: the BIOS writes zero, not 65536,
        // because 65536 does not fit in the sixteen bits it has.
        let pit = Pit8254::default_device();
        let loaded = program(&pit, 0, 2, 0);
        pit.advance_to(loaded + 65_534);
        assert!(pit.out(0), "still counting");
        pit.advance_to(loaded + 65_535);
        assert!(!pit.out(0), "the count reached one");
        pit.advance_to(loaded + 65_536);
        assert!(pit.out(0), "and the period is 65536 ticks, not zero");
    }

    #[test]
    fn mode_0_goes_high_at_terminal_count_and_stays_there() {
        let pit = Pit8254::default_device();
        let loaded = program(&pit, 0, 0, 3);
        assert!(!pit.out(0), "mode 0 takes OUT low on the control word");
        pit.advance_to(loaded + 2);
        assert!(!pit.out(0));
        pit.advance_to(loaded + 3);
        assert!(pit.out(0), "terminal count");
        pit.advance_to(loaded + 3 + 70_000);
        assert!(pit.out(0), "and it stays high while the counter wraps");
        assert_eq!(
            Device::next_event_tick(&pit),
            None,
            "a spent mode-0 counter has nothing left to do"
        );
    }

    #[test]
    fn the_low_then_high_access_mode_takes_two_writes_and_two_reads() {
        let pit = Pit8254::default_device();
        poke(&pit, 3, control(0, 3, 2, false));
        poke(&pit, 0, 0x34);
        assert_eq!(
            Device::next_event_tick(&pit),
            None,
            "half a count is not a count: nothing is loaded yet"
        );
        pit.advance_to(50);
        poke(&pit, 0, 0x12);
        assert_eq!(
            Device::next_event_tick(&pit),
            Some(51),
            "and the second byte arms it for the next clock"
        );
        pit.advance_to(51);

        // Reading it back takes two reads too, low byte first.
        poke(&pit, 3, latch(0));
        assert_eq!(peek(&pit, 0), 0x34);
        assert_eq!(peek(&pit, 0), 0x12);
    }

    #[test]
    fn the_latch_command_freezes_what_a_later_read_returns() {
        let pit = Pit8254::default_device();
        let loaded = program(&pit, 0, 2, 1_000);
        pit.advance_to(loaded + 100);
        poke(&pit, 3, latch(0));
        // The counter keeps counting while the latch holds.
        pit.advance_to(loaded + 400);
        let low = peek(&pit, 0);
        let high = peek(&pit, 0);
        assert_eq!(u16::from_le_bytes([low, high]), 900, "the latched value");

        // With the latch consumed, a read sees the counting element itself.
        poke(&pit, 3, latch(0));
        let live = u16::from_le_bytes([peek(&pit, 0), peek(&pit, 0)]);
        assert_eq!(live, 600, "and the counter never stopped");
    }

    #[test]
    fn the_read_back_command_reports_the_output_and_the_null_count() {
        let pit = Pit8254::default_device();
        poke(&pit, 3, control(0, 3, 2, false));
        poke(&pit, 0, 4);
        poke(&pit, 0, 0);

        // Latch status only, for counter 0: bit 4 clear, bit 5 set.
        poke(&pit, 3, 0xc0 | 0x20 | 0x02);
        let status = peek(&pit, 0);
        assert_eq!(status & 0x40, 0x40, "written but not yet loaded");
        assert_eq!(status & 0x80, 0x80, "mode 2 idles OUT high");
        assert_eq!(status & 0x0f, control(0, 0, 2, false) & 0x0f);
        assert_eq!((status >> 4) & 3, 3, "the access mode, read back");

        // One clock loads the counting element and clears the null count.
        pit.advance_to(pit.tick() + 1);
        poke(&pit, 3, 0xc0 | 0x20 | 0x02);
        assert_eq!(peek(&pit, 0) & 0x40, 0, "loaded now");

        // Both halves at once: the status byte comes back before the count.
        poke(&pit, 3, 0xc0 | 0x02);
        assert_eq!(peek(&pit, 0) & 0x80, 0x80, "the status byte first");
        assert_eq!(peek(&pit, 0), 4, "then the latched count, low byte first");
    }

    #[test]
    fn a_low_gate_stops_counter_2_in_the_periodic_modes() {
        for mode in [2u8, 3] {
            let pit = Pit8254::default_device();
            let gate = gate2(&pit);
            let loaded = program(&pit, 2, mode, 10);
            pit.advance_to(loaded + 4);
            poke(&pit, 3, latch(2));
            let running = u16::from_le_bytes([peek(&pit, 2), peek(&pit, 2)]);

            gate.lower();
            pit.advance_to(loaded + 400);
            assert!(pit.out(2), "a low gate forces OUT high in modes 2 and 3");
            poke(&pit, 3, latch(2));
            let stopped = u16::from_le_bytes([peek(&pit, 2), peek(&pit, 2)]);
            assert_eq!(stopped, running, "and the counting element is frozen");

            // Raising it again restarts the period on the next clock.
            gate.raise();
            pit.advance_to(pit.tick() + 1);
            poke(&pit, 3, latch(2));
            let restarted = u16::from_le_bytes([peek(&pit, 2), peek(&pit, 2)]);
            assert_eq!(restarted, 10, "a rising gate reloads");
            pit.advance_to(pit.tick() + 3);
            poke(&pit, 3, latch(2));
            let moved = u16::from_le_bytes([peek(&pit, 2), peek(&pit, 2)]);
            assert!(moved < 10, "and the counter is running again: {moved}");
        }
    }

    #[test]
    fn a_debug_read_consumes_neither_the_byte_toggle_nor_the_latch() {
        let pit = Pit8254::default_device();
        let loaded = program(&pit, 0, 2, 0x1234);
        pit.advance_to(loaded + 0x34);
        poke(&pit, 3, latch(0));

        // Three debug reads, all of them the low byte of the latched value.
        assert_eq!(peek_debug(&pit, 0), 0x00);
        assert_eq!(peek_debug(&pit, 0), 0x00);
        assert_eq!(peek_debug(&pit, 0), 0x00);
        // And the guest's own sequence is exactly where it left it.
        assert_eq!(peek(&pit, 0), 0x00);
        assert_eq!(peek(&pit, 0), 0x12);

        // A debug read of the control port is the open bus, and a debug write
        // is refused outright.
        assert_eq!(peek_debug(&pit, 3), OPEN_BUS);
        assert!(pit.regs.write(3, &[0], MemAttrs::DEBUG).is_err());
        assert!(pit.regs.write(0, &[0], MemAttrs::DEBUG).is_err());
    }

    #[test]
    fn a_debug_read_advances_nothing() {
        let pit = Pit8254::default_device();
        program(&pit, 0, 2, 100);
        let before = pit.tick();
        let _ = peek_debug(&pit, 0);
        assert_eq!(pit.tick(), before);
    }

    #[test]
    fn the_next_event_is_always_ahead_of_the_current_tick() {
        let pit = Pit8254::default_device();
        assert_eq!(
            Device::next_event_tick(&pit),
            None,
            "an unprogrammed chip has nothing to do"
        );
        for mode in [0u8, 1, 2, 3, 4, 5] {
            let pit = Pit8254::default_device();
            let gate = gate2(&pit);
            program(&pit, 2, mode, 7);
            // Modes 1 and 5 wait for a gate edge, so give them one.
            gate.lower();
            gate.raise();
            for _ in 0..40 {
                let now = Device::current_tick(&pit);
                let Some(at) = Device::next_event_tick(&pit) else {
                    break;
                };
                assert!(at > now, "mode {mode}: {at} must be past {now}");
                pit.advance_to(at);
            }
        }
    }

    #[test]
    fn the_control_port_is_write_only_and_only_bytes_are_taken() {
        let pit = Pit8254::default_device();
        assert_eq!(peek(&pit, 3), OPEN_BUS);
        assert!(pit.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(pit.regs.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
        assert!(pit.region("").is_some());
        assert!(pit.region("regs").is_some());
        assert!(pit.region("nope").is_none());
    }

    #[test]
    fn a_snapshot_round_trips_every_counter() {
        let saved = Pit8254::default_device();
        program(&saved, 0, 2, 0);
        program(&saved, 1, 3, 18);
        poke(&saved, 3, control(2, 1, 0, true));
        poke(&saved, 2, 0x25);
        saved.advance_to(saved.tick() + 5_000);
        // Leave a half-read latch and a half-written count behind, because
        // those toggles are exactly what a naive snapshot loses.
        poke(&saved, 3, latch(0));
        let _ = peek(&saved, 0);
        poke(&saved, 3, control(1, 3, 2, false));
        poke(&saved, 1, 0x99);

        let bytes = save_bytes(&saved);
        let restored = Pit8254::default_device();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("pit", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(
            save_bytes(&restored),
            bytes,
            "the two images are byte-identical"
        );
        assert_eq!(
            Device::current_tick(&restored),
            Device::current_tick(&saved)
        );
        // And the restored chip carries on where the saved one stopped.
        saved.advance_to(saved.tick() + 1_000);
        restored.advance_to(restored.tick() + 1_000);
        assert_eq!(save_bytes(&restored), save_bytes(&saved));
    }

    fn save_bytes(pit: &Pit8254) -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("pit", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("pit", CLASS.name, CLASS.version).unwrap();
            pit.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_reset_stops_every_counter() {
        let pit = Pit8254::default_device();
        program(&pit, 0, 2, 12);
        pit.advance_to(100);
        pit.reset(ResetKind::Cold);
        assert_eq!(Device::next_event_tick(&pit), None);
        assert!(!pit.out(0), "and every output idles low again");
        assert_eq!(Device::current_tick(&pit), 0);
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        assert!(Pit8254::new(&Props::new()).is_ok());
        assert!(Pit8254::new(&Props::new().with("frequency", 1u64)).is_err());
    }

    #[test]
    fn bcd_counts_in_decimal() {
        let pit = Pit8254::default_device();
        poke(&pit, 3, control(0, 3, 0, true));
        poke(&pit, 0, 0x00);
        poke(&pit, 0, 0x01);
        let loaded = pit.tick() + 1;
        pit.advance_to(loaded);
        poke(&pit, 3, latch(0));
        assert_eq!(
            u16::from_le_bytes([peek(&pit, 0), peek(&pit, 0)]),
            0x0100,
            "a hundred, in packed BCD"
        );
        pit.advance_to(loaded + 1);
        poke(&pit, 3, latch(0));
        assert_eq!(
            u16::from_le_bytes([peek(&pit, 0), peek(&pit, 0)]),
            0x0099,
            "and it borrows in decimal, not in binary"
        );
        pit.advance_to(loaded + 100);
        assert!(pit.out(0), "terminal count after a hundred clocks");
    }
}
