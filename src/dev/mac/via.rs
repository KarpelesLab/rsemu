//! The Macintosh's 6522 VIA: the overlay bit, two timers, the clock chip's
//! three wires, the keyboard's shift register and the mouse's two phases.
//!
//! # Sources
//!
//! * *Synertek SY6522 Versatile Interface Adapter* data sheet, and Rockwell's
//!   R6522 of the same part, for the register model: sixteen registers, two
//!   ports with a data-direction register each, two counter/timers, a shift
//!   register, and the interrupt flag/enable pair whose rules are the fiddly
//!   part of the chip.
//! * *Guide to the Macintosh Family Hardware*, 2nd edition, chapter 3 ("Memory
//!   and Addressing") for where the chip is decoded, and the VIA chapter's port
//!   assignment tables for what each of the sixteen pins does on a Macintosh.
//!
//! No emulator source was consulted (`ROADMAP.md` §1).
//!
//! # The decode is the board's, and it is A9-A12
//!
//! The Macintosh puts the VIA's four register-select pins on **A9-A12**, so its
//! sixteen registers are 512 bytes apart and the block repeats every 8 KiB
//! through the window the chip select decodes (`$E8_0000`-`$EF_FFFF` on a
//! Plus). That is why this class publishes a region of `0x2000` bytes rather
//! than sixteen, and why `machines/mac-plus.machine` maps it as a mirror
//! through the whole half-megabyte: the repetition is what the address decoder
//! does, not a convenience.
//!
//! The published low-memory equates line up with that and are the check:
//! `VIA` is `$EFE1FE`, `vBufB` is offset `$0000` and `vBufA` offset `$1E00`, so
//! port B is register 0 and port A is register **15** — the no-handshake
//! address — which is exactly what `(offset >> 9) & 15` gives.
//!
//! **`A0` is not decoded here.** The chip sits on one byte lane of the 68000's
//! word bus, so in hardware the other lane floats; modelling that would turn a
//! read of the wrong half into a bus error the guest never asks for, and
//! nothing can distinguish the two on a machine that only ever issues byte
//! accesses to this address. Byte accesses are all this region accepts, which
//! is the part that *is* observable.
//!
//! # What a Macintosh hangs off the pins
//!
//! Port A, from the Guide's port-assignment table:
//!
//! ```text
//!   PA0-PA2  sound volume, three bits
//!   PA3      SNDPG2    which sound buffer the sound circuit reads
//!   PA4      ROMOVERLAY 1 = the ROM answers at zero; cleared once
//!   PA5      SEL       the disk drive's head/register select line
//!   PA6      PAGE2     which video buffer the video circuit reads
//!   PA7      SCCWREQ   the SCC's wait/request output, an input here
//! ```
//!
//! Port B:
//!
//! ```text
//!   PB0      RTCDATA   the clock chip's serial data, both ways
//!   PB1      RTCCLK    its serial clock
//!   PB2      /RTCENB   its enable, active low
//!   PB3      SW        the mouse button, low while pressed
//!   PB4      X2        the mouse's X quadrature
//!   PB5      Y2        its Y quadrature
//!   PB6      H4        horizontal blanking, an input
//!   PB7      /SNDENB   sound enable, active low
//! ```
//!
//! and the interrupt sources, in IFR bit order:
//!
//! ```text
//!   0  CA2   the one-second interrupt from the clock chip
//!   1  CA1   vertical blanking, 60.15 Hz
//!   2  SR    the keyboard's shift register filled or emptied
//!   3  CB2   keyboard data
//!   4  CB1   keyboard clock
//!   5  T2
//!   6  T1
//!   7  the wired OR of the six above, masked by IER
//! ```
//!
//! This class knows none of that. It is a 6522, the pins are pins, and the
//! machine file is where `pa4` reaches the overlay decoder and `ca1` reaches
//! the video circuit. The one concession is [`Via::overlay_bit`], which exists
//! so a test can read the pin without building a wire graph.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.via";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies: sixteen
/// registers on A9-A12, so `16 * 512`.
pub const REGISTER_SPAN: u64 = 0x2000;

/// How far apart two registers are, in bytes — the board's A9-A12 wiring.
const REGISTER_STRIDE: u64 = 0x200;

/// `next_event` when the chip has nothing scheduled.
const NO_EVENT: u64 = u64::MAX;

// -- register numbers --------------------------------------------------------

const R_ORB: u8 = 0;
const R_ORA: u8 = 1;
const R_DDRB: u8 = 2;
const R_DDRA: u8 = 3;
const R_T1CL: u8 = 4;
const R_T1CH: u8 = 5;
const R_T1LL: u8 = 6;
const R_T1LH: u8 = 7;
const R_T2CL: u8 = 8;
const R_T2CH: u8 = 9;
const R_SR: u8 = 10;
const R_ACR: u8 = 11;
const R_PCR: u8 = 12;
const R_IFR: u8 = 13;
const R_IER: u8 = 14;
const R_ORA_NH: u8 = 15;

// -- interrupt flag bits -----------------------------------------------------

/// CA2. The Macintosh's one-second interrupt.
pub const IRQ_CA2: u8 = 1 << 0;
/// CA1. The Macintosh's vertical blanking interrupt.
pub const IRQ_CA1: u8 = 1 << 1;
/// The shift register filled or emptied.
pub const IRQ_SR: u8 = 1 << 2;
/// CB2.
pub const IRQ_CB2: u8 = 1 << 3;
/// CB1.
pub const IRQ_CB1: u8 = 1 << 4;
/// Timer 2 timed out.
pub const IRQ_T2: u8 = 1 << 5;
/// Timer 1 timed out.
pub const IRQ_T1: u8 = 1 << 6;
/// Bit 7 of IFR and IER: "any" on read, "set rather than clear" on a write to
/// IER.
const IRQ_ANY: u8 = 1 << 7;
/// The six real sources; bit 7 is derived and never stored.
const IRQ_SOURCES: u8 = 0x7f;

// -- pin lines ---------------------------------------------------------------

const LINE_PA: u32 = 0;
const LINE_PB: u32 = 8;
const LINE_CA1: u32 = 16;
const LINE_CA2: u32 = 17;
const LINE_CB1: u32 = 18;
const LINE_CB2: u32 = 19;

/// How many pin lines there are, for a fixed-size array of input levels.
const LINES: usize = 20;

/// The output pin that carries `/IRQ`, high here when the chip is asking.
const IRQ_PIN: &str = "irq";

/// Everything the guest can see or change, plus the pin levels others drive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// φ2 ticks since reset. The timers are counted against this.
    ticks: u64,

    ora: u8,
    orb: u8,
    ddra: u8,
    ddrb: u8,

    /// Timer 1's counter, and the moment it will next underflow.
    t1_latch: u16,
    t1_counter: u16,
    /// The tick at which T1 next reaches `$FFFF`; `NO_EVENT` when it is idle.
    t1_due: u64,
    /// Whether T1 has already flagged since its last reload, which is what
    /// stops a one-shot from flagging twice.
    t1_armed: bool,

    t2_latch_low: u8,
    t2_counter: u16,
    t2_due: u64,
    t2_armed: bool,

    sr: u8,
    acr: u8,
    pcr: u8,
    ifr: u8,
    ier: u8,

    /// PB7 as the timer drives it, when ACR bit 7 says it does.
    pb7_timer: bool,

    /// What other devices are driving onto each pin.
    ///
    /// **Snapshotted**, which is not what §4.5 says about another device's
    /// state, and the round-trip test is why. A restore ends with the realize
    /// sweep, in which every driver re-announces the level it is holding; if
    /// this chip came back with a *different* level on a handshake pin, that
    /// re-announcement arrives as an **edge** and latches an interrupt the
    /// guest never had. Keeping the levels makes the sweep a no-op for pins
    /// nothing has moved, which is the only way the two machines stay
    /// identical. A peer that really did change its level still delivers the
    /// edge, because the sweep still runs.
    inputs: [bool; LINES],
}

impl Default for State {
    fn default() -> Self {
        State::fresh(0)
    }
}

impl State {
    /// The documented reset state: every register zero, both counters free.
    ///
    /// The data sheet's RES description clears ORA/ORB, DDRA/DDRB, ACR, PCR,
    /// IFR and IER and leaves the counters and latches undefined; zero is the
    /// value this model picks, and a guest that reads one before writing it
    /// gets a defined answer rather than a different one per run.
    fn fresh(ticks: u64) -> State {
        State {
            ticks,
            ora: 0,
            orb: 0,
            ddra: 0,
            ddrb: 0,
            t1_latch: 0,
            t1_counter: 0,
            t1_due: NO_EVENT,
            t1_armed: false,
            t2_latch_low: 0,
            t2_counter: 0,
            t2_due: NO_EVENT,
            t2_armed: false,
            sr: 0,
            acr: 0,
            pcr: 0,
            ifr: 0,
            ier: 0,
            pb7_timer: false,
            // Every pin pulled high, which is what an input with nothing
            // driving it reads on a board full of pull-ups. A wire that has a
            // driver overwrites this during the announce sweep (§4.3).
            inputs: [true; LINES],
        }
    }

    /// Whether the chip is asserting `/IRQ`.
    fn asserting(&self) -> bool {
        self.ifr & self.ier & IRQ_SOURCES != 0
    }

    /// IFR as the guest reads it: the six sources plus the derived bit 7.
    fn ifr_read(&self) -> u8 {
        let base = self.ifr & IRQ_SOURCES;
        if self.asserting() {
            base | IRQ_ANY
        } else {
            base
        }
    }

    /// Port A as the guest reads it: output bits from ORA, input bits from the
    /// pins.
    fn read_pa(&self) -> u8 {
        let mut value = 0u8;
        for bit in 0..8u8 {
            let mask = 1u8 << bit;
            let high = if self.ddra & mask != 0 {
                self.ora & mask != 0
            } else {
                self.inputs[(LINE_PA + u32::from(bit)) as usize]
            };
            if high {
                value |= mask;
            }
        }
        value
    }

    /// Port B as the guest reads it.
    ///
    /// PB7 is the one that is not simply "ORB or the pin": with ACR bit 7 set
    /// the timer owns it, and the data sheet has the timer's level read back
    /// whatever DDRB says.
    fn read_pb(&self) -> u8 {
        let mut value = 0u8;
        for bit in 0..8u8 {
            let mask = 1u8 << bit;
            let high = if bit == 7 && self.acr & 0x80 != 0 {
                self.pb7_timer
            } else if self.ddrb & mask != 0 {
                self.orb & mask != 0
            } else {
                self.inputs[(LINE_PB + u32::from(bit)) as usize]
            };
            if high {
                value |= mask;
            }
        }
        value
    }

    /// What each of port A's eight pins is driving onto its net.
    fn drive_pa(&self) -> [Drive; 8] {
        let mut out = [Drive::HiZ; 8];
        for (bit, slot) in out.iter_mut().enumerate() {
            let mask = 1u8 << bit;
            if self.ddra & mask != 0 {
                *slot = Level::from(self.ora & mask != 0).into();
            }
        }
        out
    }

    /// The same for port B, with the timer's claim on PB7.
    fn drive_pb(&self) -> [Drive; 8] {
        let mut out = [Drive::HiZ; 8];
        for (bit, slot) in out.iter_mut().enumerate() {
            let mask = 1u8 << bit;
            if bit == 7 && self.acr & 0x80 != 0 {
                *slot = Level::from(self.pb7_timer).into();
            } else if self.ddrb & mask != 0 {
                *slot = Level::from(self.orb & mask != 0).into();
            }
        }
        out
    }

    /// The tick at which something will next happen, or `NO_EVENT`.
    fn next_event(&self) -> u64 {
        self.t1_due.min(self.t2_due)
    }

    /// Where T1 stands at `ticks`, given when it was last reloaded.
    ///
    /// The counter is 16 bits and decrements every φ2; `t1_due` is the tick of
    /// the *next* underflow, so the count now is the distance to it less one,
    /// exactly as the data sheet's "N+1.5 cycles" timing works out for a
    /// counter read after the load.
    fn t1_count_at(&self, ticks: u64) -> u16 {
        if self.t1_due == NO_EVENT {
            return self.t1_counter;
        }
        let remaining = self.t1_due.saturating_sub(ticks);
        (remaining & 0xffff) as u16
    }

    fn t2_count_at(&self, ticks: u64) -> u16 {
        if self.t2_due == NO_EVENT {
            return self.t2_counter;
        }
        let remaining = self.t2_due.saturating_sub(ticks);
        (remaining & 0xffff) as u16
    }

    /// Advance both timers to `target`, setting flags for every underflow that
    /// falls in the interval.
    ///
    /// Free-running T1 can underflow many times in one step, and only the
    /// *flag* survives — the data sheet has one flag, not a count — so the
    /// reload arithmetic jumps straight to the next underflow after `target`
    /// rather than looping. A machine that spends a second between scheduler
    /// visits must not spend a second in this function.
    fn advance_to(&mut self, target: u64) {
        if target <= self.ticks {
            return;
        }
        if self.t1_due <= target {
            self.ifr |= IRQ_T1;
            if self.acr & 0x40 != 0 {
                // Free-running: reload from the latch and keep going. One
                // period is `latch + 2` φ2 cycles (data sheet, "T1 in free-run
                // mode": N+2).
                let period = u64::from(self.t1_latch) + 2;
                let elapsed = target - self.t1_due;
                self.t1_due = target + period - (elapsed % period);
                // ACR bit 7 has the timer square-wave PB7, inverting on each
                // underflow. Over `k` underflows the level flips `k` times.
                let flips = elapsed / period + 1;
                if flips % 2 == 1 {
                    self.pb7_timer = !self.pb7_timer;
                }
            } else {
                // One-shot: the counter keeps counting down through zero, but
                // the flag is set once and PB7 goes high and stays.
                self.t1_counter = ((self.t1_due.wrapping_sub(target)) & 0xffff) as u16;
                self.t1_due = NO_EVENT;
                self.t1_armed = false;
                self.pb7_timer = true;
            }
        }
        if self.t2_due <= target {
            if self.t2_armed {
                self.ifr |= IRQ_T2;
                self.t2_armed = false;
            }
            self.t2_counter = ((self.t2_due.wrapping_sub(target)) & 0xffff) as u16;
            self.t2_due = NO_EVENT;
        }
        self.ticks = target;
    }
}

/// The sixteen registers, as something an address space can dispatch to, plus
/// everything the pins need.
struct Shared {
    state: Mutex<State>,
    /// The output pins, `None` until a `wire` statement claims one.
    out: Mutex<Outputs>,
    /// Published without a lock for the scheduler: see [`Shared::publish`].
    ticks: AtomicU64,
    next_event: AtomicU64,
    /// PA4 as the overlay decoder sees it, for a test with no wire graph.
    pa4: AtomicBool,
    /// The catch-up handle the register block syncs through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
}

/// Every output this chip drives.
#[derive(Default, Clone)]
struct Outputs {
    irq: Option<WireSource>,
    pa: [Option<WireSource>; 8],
    pb: [Option<WireSource>; 8],
    ca2: Option<WireSource>,
    cb2: Option<WireSource>,
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Via");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(state.next_event(), Ordering::Relaxed);
        self.pa4
            .store(state.read_pa() & 0x10 != 0, Ordering::Relaxed);
    }

    /// Bring the chip up to date before a register access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, debug: bool) {
        if debug {
            return;
        }
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // A refusal means catch-up for this chip is already running further
            // up the stack. The access still has to be answered, and answering
            // it from where the timers stand is the only defined thing to do.
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    /// Run the chip forward to `target` φ2 ticks. Backwards is a no-op.
    fn advance_to(&self, target: u64) {
        let changed = {
            let mut state = self.state.lock();
            let before = state.asserting();
            let pb7 = state.pb7_timer;
            state.advance_to(target);
            self.publish(&state);
            before != state.asserting() || pb7 != state.pb7_timer
        };
        if changed {
            self.refresh();
        }
    }

    /// Drive every output pin to whatever the state now says.
    ///
    /// Called with no lock held: a wire delivers synchronously, so driving from
    /// inside the critical section would run the far end's sink under this
    /// chip's lock. Mutate, release, *then* call outward (`CLAUDE.md`).
    fn refresh(&self) {
        let (irq, pa, pb) = {
            let state = self.state.lock();
            (state.asserting(), state.drive_pa(), state.drive_pb())
        };
        let out = self.out.lock().clone();
        if let Some(src) = &out.irq {
            src.set(Level::from(irq));
        }
        for (src, drive) in out.pa.iter().zip(pa) {
            if let Some(src) = src {
                src.drive(drive);
            }
        }
        for (src, drive) in out.pb.iter().zip(pb) {
            if let Some(src) = src {
                src.drive(drive);
            }
        }
    }

    /// A level arrived on one of the input pins.
    fn edge(&self, line: u32, level: bool) {
        let changed = {
            let mut state = self.state.lock();
            let was = state.inputs[line as usize];
            state.inputs[line as usize] = level;
            if was == level {
                false
            } else {
                let before = state.asserting();
                // The four handshake pins latch an interrupt on the edge PCR
                // selects. PCR bit 0 picks CA1's edge, bit 4 picks CB1's; for
                // CA2 and CB2 the input modes are bits 1-2 and 5-6, whose low
                // bit is likewise "positive edge".
                match line {
                    LINE_CA1 => {
                        if level == (state.pcr & 0x01 != 0) {
                            state.ifr |= IRQ_CA1;
                        }
                    }
                    LINE_CA2 => {
                        if state.pcr & 0x08 == 0 && level == (state.pcr & 0x04 != 0) {
                            state.ifr |= IRQ_CA2;
                        }
                    }
                    LINE_CB1 => {
                        if level == (state.pcr & 0x10 != 0) {
                            state.ifr |= IRQ_CB1;
                        }
                    }
                    LINE_CB2 if state.pcr & 0x80 == 0 && level == (state.pcr & 0x40 != 0) => {
                        state.ifr |= IRQ_CB2;
                    }
                    _ => {}
                }
                self.publish(&state);
                before != state.asserting()
            }
        };
        if changed {
            self.refresh();
        }
    }

    /// Read one register, with every side effect a real read has.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        self.sync(debug);
        let (value, changed) = {
            let mut state = self.state.lock();
            let before = state.asserting();
            let ticks = state.ticks;
            let value = match index {
                R_ORB => {
                    if !debug {
                        // Reading port B clears the CB1 flag, and CB2's unless
                        // PCR has it in an independent mode.
                        state.ifr &= !IRQ_CB1;
                        if state.pcr & 0x20 == 0 {
                            state.ifr &= !IRQ_CB2;
                        }
                    }
                    state.read_pb()
                }
                R_ORA => {
                    if !debug {
                        state.ifr &= !IRQ_CA1;
                        if state.pcr & 0x02 == 0 {
                            state.ifr &= !IRQ_CA2;
                        }
                    }
                    state.read_pa()
                }
                // The no-handshake address, which is the one a Macintosh uses:
                // same data, no flags cleared, no CA2 pulse.
                R_ORA_NH => state.read_pa(),
                R_DDRB => state.ddrb,
                R_DDRA => state.ddra,
                R_T1CL => {
                    if !debug {
                        state.ifr &= !IRQ_T1;
                    }
                    state.t1_count_at(ticks) as u8
                }
                R_T1CH => (state.t1_count_at(ticks) >> 8) as u8,
                R_T1LL => state.t1_latch as u8,
                R_T1LH => (state.t1_latch >> 8) as u8,
                R_T2CL => {
                    if !debug {
                        state.ifr &= !IRQ_T2;
                    }
                    state.t2_count_at(ticks) as u8
                }
                R_T2CH => (state.t2_count_at(ticks) >> 8) as u8,
                R_SR => {
                    if !debug {
                        state.ifr &= !IRQ_SR;
                    }
                    state.sr
                }
                R_ACR => state.acr,
                R_PCR => state.pcr,
                R_IFR => state.ifr_read(),
                R_IER => state.ier | IRQ_ANY,
                _ => unreachable!("index is masked to 0..16"),
            };
            if !debug {
                self.publish(&state);
            }
            (value, before != state.asserting())
        };
        if changed {
            self.refresh();
        }
        value
    }

    /// Write one register.
    fn write_register(&self, index: u8, value: u8) {
        self.sync(false);
        {
            let mut state = self.state.lock();
            let ticks = state.ticks;
            match index {
                R_ORB => {
                    state.orb = value;
                    state.ifr &= !IRQ_CB1;
                    if state.pcr & 0x20 == 0 {
                        state.ifr &= !IRQ_CB2;
                    }
                }
                R_ORA | R_ORA_NH => {
                    state.ora = value;
                    if index == R_ORA {
                        state.ifr &= !IRQ_CA1;
                        if state.pcr & 0x02 == 0 {
                            state.ifr &= !IRQ_CA2;
                        }
                    }
                }
                R_DDRB => state.ddrb = value,
                R_DDRA => state.ddra = value,
                R_T1CL | R_T1LL => state.t1_latch = (state.t1_latch & 0xff00) | u16::from(value),
                R_T1CH => {
                    state.t1_latch = (state.t1_latch & 0x00ff) | (u16::from(value) << 8);
                    // Writing the high counter byte transfers the latch into
                    // the counter, clears the flag and starts the timer.
                    state.t1_counter = state.t1_latch;
                    state.t1_due = ticks + u64::from(state.t1_latch) + 2;
                    state.t1_armed = true;
                    state.ifr &= !IRQ_T1;
                    if state.acr & 0x80 != 0 {
                        state.pb7_timer = false;
                    }
                }
                R_T1LH => {
                    state.t1_latch = (state.t1_latch & 0x00ff) | (u16::from(value) << 8);
                    state.ifr &= !IRQ_T1;
                }
                R_T2CL => state.t2_latch_low = value,
                R_T2CH => {
                    state.t2_counter = (u16::from(value) << 8) | u16::from(state.t2_latch_low);
                    state.t2_due = ticks + u64::from(state.t2_counter) + 2;
                    state.t2_armed = true;
                    state.ifr &= !IRQ_T2;
                }
                R_SR => {
                    state.sr = value;
                    state.ifr &= !IRQ_SR;
                }
                R_ACR => state.acr = value,
                R_PCR => state.pcr = value,
                // A one clears a flag; bit 7 is derived and is ignored.
                R_IFR => state.ifr &= !(value & IRQ_SOURCES),
                R_IER => {
                    if value & IRQ_ANY != 0 {
                        state.ier |= value & IRQ_SOURCES;
                    } else {
                        state.ier &= !(value & IRQ_SOURCES);
                    }
                }
                _ => unreachable!("index is masked to 0..16"),
            }
            self.publish(&state);
        }
        // Unconditionally, not only when the interrupt changed: a write to
        // ORB, DDRA or ACR moves pins other devices are watching.
        self.refresh();
    }
}

impl MemOps for Shared {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.read_register(register_of(offset), attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to IFR would clear a flag the guest has not seen
            // and to T1C-H would start a timer. Neither can be made harmless,
            // so it is refused rather than guessed at (invariant 5).
            return Err(BusError::BadAccess);
        }
        self.write_register(register_of(offset), *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A 6522 is an eight-bit part on one lane of the 68000's word bus: a
        // word access would read the chip and the floating other half, which
        // is not something to invent a value for.
        AccessConstraints::word(Width::U8, Endian::Big)
    }
}

/// Which register an offset into the block selects: A9-A12 and nothing else.
#[inline]
fn register_of(offset: u64) -> u8 {
    ((offset / REGISTER_STRIDE) & 0xf) as u8
}

// ---------------------------------------------------------------------------
// the pins
// ---------------------------------------------------------------------------

/// One input pin, holding the drivers on its net.
#[derive(Debug)]
struct ViaPin {
    shared: Arc<Shared>,
    line: u32,
    inputs: FanIn,
}

impl WireSink for ViaPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Wired-and with a pull-up is what a 6522's port pin sees on a board:
        // every driver is open-collector and the pin is high only while none
        // of them is pulling. `Resolve::And` says exactly that, and it is what
        // makes the Macintosh's shared RTC data line work in both directions.
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.shared.edge(self.line, high);
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// A 6522 Versatile Interface Adapter as a Macintosh decodes it.
#[derive(Debug)]
pub struct Via {
    shared: Arc<Shared>,
    region: RegionRef,
    /// The pins, kept alive here: a net holds only a `Weak` to its sinks
    /// (`ROADMAP.md` §4.3).
    pins: Mutex<Vec<Arc<ViaPin>>>,
}

impl Via {
    /// Build the chip. It takes no properties: a 6522 is a 6522.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Via> {
        props.reader().finish()?;
        Ok(Via::build())
    }

    /// The same, for a test that has no `Props` to hand.
    #[must_use]
    pub fn build() -> Via {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            out: Mutex::with_rank(LockRank::LEAF, Outputs::default()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            // Port A is all inputs out of reset and the line is pulled up, so
            // the overlay is asserted before any code has run. That is what
            // lets the processor find a reset vector in a machine whose RAM
            // holds nothing.
            pa4: AtomicBool::new(true),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_SPAN,
            Arc::clone(&shared) as Arc<dyn MemOps>,
        ));
        Via {
            shared,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Whether the chip is asserting its interrupt output.
    #[must_use]
    pub fn irq(&self) -> bool {
        self.shared.state.lock().asserting()
    }

    /// PA4 as the overlay decoder sees it: `true` while the ROM answers at
    /// zero.
    #[must_use]
    pub fn overlay_bit(&self) -> bool {
        self.shared.pa4.load(Ordering::Relaxed)
    }

    /// φ2 ticks since reset.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Read a register the way the address space would, for a test.
    #[must_use]
    pub fn peek(&self, index: u8) -> u8 {
        self.shared.read_register(index & 0xf, false)
    }

    /// Write a register the way the address space would, for a test.
    pub fn poke(&self, index: u8, value: u8) {
        self.shared.write_register(index & 0xf, value);
    }

    /// Deliver an edge on CA1 — the Macintosh's vertical blanking input.
    pub fn set_ca1(&self, level: bool) {
        self.shared.edge(LINE_CA1, level);
    }

    /// Deliver an edge on CA2 — the Macintosh's one-second interrupt.
    pub fn set_ca2(&self, level: bool) {
        self.shared.edge(LINE_CA2, level);
    }

    /// Set the level another device is driving onto a port B pin.
    pub fn set_pb(&self, bit: u32, level: bool) {
        self.shared.edge(LINE_PB + (bit & 7), level);
    }

    /// Set the level another device is driving onto a port A pin.
    pub fn set_pa(&self, bit: u32, level: bool) {
        self.shared.edge(LINE_PA + (bit & 7), level);
    }

    /// Run the chip until `target` φ2 ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Connect the catch-up handle the register block syncs through (§4.2).
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Connect one output pin by name.
    ///
    /// # Errors
    ///
    /// [`Error::Config`](crate::core::Error::Config) if the chip drives no such
    /// pin.
    pub fn connect_pin(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.shared.out.lock();
            match port {
                IRQ_PIN => out.irq = Some(source),
                "ca2" => out.ca2 = Some(source),
                "cb2" => out.cb2 = Some(source),
                _ => match port_index(port) {
                    Some((LINE_PA, n)) => out.pa[n as usize] = Some(source),
                    Some((LINE_PB, n)) => out.pb[n as usize] = Some(source),
                    _ => {
                        return Err(crate::core::error::Error::Config {
                            at: String::from(port),
                            message: String::from(
                                "a 6522 drives `irq`, `ca2`, `cb2` and `pa0`…`pb7`",
                            ),
                        });
                    }
                },
            }
        }
        self.shared.refresh();
        Ok(())
    }
}

/// `pa3` → `(LINE_PA, 3)`, `pb7` → `(LINE_PB, 7)`, anything else `None`.
fn port_index(port: &str) -> Option<(u32, u32)> {
    let base = match port.as_bytes() {
        [b'p', b'a', _] => LINE_PA,
        [b'p', b'b', _] => LINE_PB,
        _ => return None,
    };
    let digit = port.as_bytes()[2];
    (b'0'..=b'7')
        .contains(&digit)
        .then(|| (base, u32::from(digit - b'0')))
}

impl Device for Via {
    fn class(&self) -> &'static DeviceClass {
        &VIA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let inputs = state.inputs;
            *state = State::fresh(state.ticks);
            // What other devices are driving is theirs and survives our reset.
            state.inputs = inputs;
            self.shared.publish(&state);
        }
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u8(state.ora)?;
        w.write_u8(state.orb)?;
        w.write_u8(state.ddra)?;
        w.write_u8(state.ddrb)?;
        w.write_u16(state.t1_latch)?;
        w.write_u16(state.t1_counter)?;
        w.write_u64(state.t1_due)?;
        w.write_bool(state.t1_armed)?;
        w.write_u8(state.t2_latch_low)?;
        w.write_u16(state.t2_counter)?;
        w.write_u64(state.t2_due)?;
        w.write_bool(state.t2_armed)?;
        w.write_u8(state.sr)?;
        w.write_u8(state.acr)?;
        w.write_u8(state.pcr)?;
        w.write_u8(state.ifr)?;
        w.write_u8(state.ier)?;
        w.write_bool(state.pb7_timer)?;
        // And the pin levels — see the field's own comment for why.
        let mut pins = 0u32;
        for (bit, high) in state.inputs.iter().enumerate() {
            if *high {
                pins |= 1 << bit;
            }
        }
        w.write_u32(pins)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        {
            let mut state = self.shared.state.lock();
            let inputs = state.inputs;
            let mut next = State::fresh(r.read_u64()?);
            next.ora = r.read_u8()?;
            next.orb = r.read_u8()?;
            next.ddra = r.read_u8()?;
            next.ddrb = r.read_u8()?;
            next.t1_latch = r.read_u16()?;
            next.t1_counter = r.read_u16()?;
            next.t1_due = r.read_u64()?;
            next.t1_armed = r.read_bool()?;
            next.t2_latch_low = r.read_u8()?;
            next.t2_counter = r.read_u16()?;
            next.t2_due = r.read_u64()?;
            next.t2_armed = r.read_bool()?;
            next.sr = r.read_u8()?;
            next.acr = r.read_u8()?;
            next.pcr = r.read_u8()?;
            next.ifr = r.read_u8()? & IRQ_SOURCES;
            next.ier = r.read_u8()? & IRQ_SOURCES;
            next.pb7_timer = r.read_bool()?;
            let pins = r.read_u32()?;
            for (bit, high) in next.inputs.iter_mut().enumerate() {
                *high = pins & (1 << bit) != 0;
            }
            let _ = inputs;
            *state = next;
            self.shared.publish(&state);
        }
        self.shared.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        self.connect_pin(port, source)
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            "ca1" => LINE_CA1,
            "ca2" => LINE_CA2,
            "cb1" => LINE_CB1,
            "cb2" => LINE_CB2,
            _ => {
                let (base, n) = port_index(port)?;
                base + n
            }
        };
        let pin = Arc::new(ViaPin {
            shared: Arc::clone(&self.shared),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A counter read has to report the count at the cycle of the read,
    /// and an underflow has to reach the processor on the cycle it happens.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Via::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Via::attach_lazy(self, handle);
    }
}

impl Instance for Via {}

/// The `mac.via` device class.
pub static VIA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a 6522 VIA on a Macintosh's A9-A12 decode: two ports, two timers, a shift register",
    properties: &[],
    construct: |props| Ok(Box::new(Via::new(props)?)),
};

/// Add [`VIA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if something already claimed
/// the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&VIA_CLASS)
}

/// Bind [`VIA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Via::new(props)?)))
}

/// What the validator should know about `mac.via`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out);
    for pin in ["ca1", "cb1"] {
        schema = schema.port(pin, PortDir::In);
    }
    for pin in ["ca2", "cb2"] {
        schema = schema.port(pin, PortDir::InOut);
    }
    for pin in [
        "pa0", "pa1", "pa2", "pa3", "pa4", "pa5", "pa6", "pa7", "pb0", "pb1", "pb2", "pb3", "pb4",
        "pb5", "pb6", "pb7",
    ] {
        schema = schema.port(pin, PortDir::InOut);
    }
    schema
}

#[cfg(test)]
mod tests;
