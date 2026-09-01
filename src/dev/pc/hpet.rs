//! An IA-PC high precision event timer.
//!
//! # Sources
//!
//! * Intel *IA-PC HPET (High Precision Event Timers) Specification*, revision
//!   1.0a. The register block (§2.3), the general capabilities and
//!   identification register (§2.3.4), the general configuration register
//!   (§2.3.5), the general interrupt status register (§2.3.6), the main counter
//!   (§2.3.7), the per-timer configuration and capability register (§2.3.8) and
//!   the comparator with its periodic accumulator (§2.3.9) all come from it.
//!   Each is cited on the item it justifies.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # Time, and the femtosecond that is not a float
//!
//! The main counter counts one thing: ticks of the crystal driving it. So this
//! is a **lazily advanced** device (`ROADMAP.md` §4.2) in a clock domain of its
//! own, one domain tick to one counter increment, and [`Device::advance_to`]
//! is the only thing that moves it. Nothing here reads a host clock or sleeps.
//!
//! `COUNTER_CLK_PERIOD` — the field a driver divides by to turn counter units
//! into nanoseconds — is a **declared integer property in femtoseconds**, not a
//! number derived from the domain's rate. That is deliberate on two counts.
//! Deriving it would need `10^15 / rate`, which is exactly the division
//! `CLAUDE.md` forbids in the time path; and a device cannot see its domain's
//! rate anyway (`BindCtx` carries a clock domain, not the forest's picture of
//! what it is rated at). So the machine file writes the number twice — once as
//! the oscillator's frequency and once here — the same seam `pc.video`'s
//! `dot-clock` sits in, and the same note applies: a gap to close in the
//! framework, not a design.
//!
//! The consequence worth stating: this device never converts a tick to a
//! second. The period is a constant it reports and a driver's arithmetic; the
//! counter is an integer; a comparator match is an integer comparison.
//!
//! # What is not here
//!
//! * **FSB interrupt delivery** (`Tn_FSB_INT_DEL_CAP`) is not advertised, so
//!   the FSB route register is reserved and reads as zero. A timer here reaches
//!   a processor the way every other device on the board does: a wire to an
//!   interrupt controller.
//! * **`Tn_INT_ROUTE_CNF` is advisory.** In real hardware it picks which I/O
//!   APIC input the timer drives; here the wire is the machine file's, and
//!   `Tn_INT_ROUTE_CAP` is set from the `routeN` property so that it advertises
//!   exactly the input the board actually wired. Writing another value is
//!   recorded and changes nothing, which is the honest answer for a part whose
//!   output pin is soldered.
//! * **Legacy replacement route.** `LEG_RT_CNF` is implemented as far as this
//!   part reaches: it is settable and reported, and timers 0 and 1 ignore their
//!   route field while it is set — which they already do here. What it *also*
//!   does on a real board is disconnect the 8254 from IRQ0 and the RTC from
//!   IRQ8, and that is a gate on the board between three chips, not a register
//!   in any of them. rsemu has no wire combinator a machine file can
//!   instantiate yet (`machine::validate::WireCombinators` is empty), so a
//!   board that turns legacy replacement on will see both timers on the line.
//!   Said out loud rather than left to be discovered.

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
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.hpet";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
///
/// One kilobyte, which is what the ACPI description of an HPET reserves and
/// what leaves room for the 32 timers the specification allows (§2.3).
pub const REGISTER_WINDOW_LEN: u64 = 0x400;

/// The page an HPET conventionally sits at on a PC.
pub const DEFAULT_BASE: u64 = 0xfed0_0000;

/// How many comparators this part has.
///
/// Three is the specification's own minimum (§2.2: "a minimum of three
/// timers"), and the number every operating system's driver is written
/// against.
pub const TIMERS: usize = 3;

/// The revision this part reports (§2.3.4's `REV_ID`).
const REV_ID: u64 = 0x01;

/// The largest counter period the specification permits: 100 ns, in
/// femtoseconds (§2.3.4, "must be less than or equal to 05F5E100h").
pub const MAX_PERIOD_FS: u64 = 0x05F5_E100;

/// The counter period a machine file gets if it does not say: 100 ns, a 10 MHz
/// counter.
pub const DEFAULT_PERIOD_FS: u64 = 100_000_000;

// -- the register map (§2.3) ------------------------------------------------

/// General capabilities and identification, read-only.
const REG_CAP: u64 = 0x000;
/// General configuration.
const REG_CONF: u64 = 0x010;
/// General interrupt status: write one to clear.
const REG_STATUS: u64 = 0x020;
/// The main counter.
const REG_COUNTER: u64 = 0x0f0;
/// The first timer's configuration register; timers are 0x20 bytes apart.
const REG_TIMER_BASE: u64 = 0x100;
/// How far apart two timers' register blocks are.
const REG_TIMER_STRIDE: u64 = 0x20;

/// `ENABLE_CNF`: the main counter runs and interrupts are permitted (§2.3.5).
const CONF_ENABLE: u64 = 1 << 0;
/// `LEG_RT_CNF`: the legacy replacement route (§2.3.5).
const CONF_LEGACY: u64 = 1 << 1;

/// `Tn_INT_TYPE_CNF` (bit 1): set is level-triggered.
const TIMER_LEVEL: u64 = 1 << 1;
/// `Tn_INT_ENB_CNF` (bit 2).
const TIMER_ENABLE: u64 = 1 << 2;
/// `Tn_TYPE_CNF` (bit 3): set is periodic.
const TIMER_PERIODIC: u64 = 1 << 3;
/// `Tn_PER_INT_CAP` (bit 4), read-only: this timer can do periodic.
const TIMER_PERIODIC_CAP: u64 = 1 << 4;
/// `Tn_SIZE_CAP` (bit 5), read-only: this timer is 64 bits wide.
const TIMER_SIZE_CAP: u64 = 1 << 5;
/// `Tn_VAL_SET_CNF` (bit 6): the next comparator write sets the accumulator.
const TIMER_VAL_SET: u64 = 1 << 6;
/// `Tn_32MODE_CNF` (bit 8): compare only the low 32 bits.
const TIMER_32BIT: u64 = 1 << 8;

/// Which bits of a timer's configuration register software may change.
///
/// Route is bits 9-13; FSB enable (bit 14) is not offered because the
/// capability is not advertised, and every capability bit is the part's.
const TIMER_WRITABLE: u64 =
    TIMER_LEVEL | TIMER_ENABLE | TIMER_PERIODIC | TIMER_VAL_SET | TIMER_32BIT | (0x1f << 9);

/// One comparator and everything it remembers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Timer {
    /// The configuration register's writable half.
    conf: u64,
    /// The comparator, which is the accumulator in periodic mode.
    comp: u64,
    /// "The last value written to" the comparator (§2.3.9), which is what
    /// hardware adds on each expiry in periodic mode.
    period: u64,
    /// Which I/O APIC input this timer's output pin is wired to. A board fact,
    /// reported in `Tn_INT_ROUTE_CAP`.
    route: u32,
    /// The output level, for a level-triggered timer. An edge-triggered one
    /// pulses instead and leaves this low.
    output: bool,
}

impl Timer {
    /// Whether the comparison is done in 32 bits.
    fn narrow(&self) -> bool {
        self.conf & TIMER_32BIT != 0
    }

    /// The counter value this timer compares against, in its own width.
    fn now(&self, counter: u64) -> u64 {
        if self.narrow() {
            counter & 0xffff_ffff
        } else {
            counter
        }
    }

    /// Ticks until the counter reaches the comparator.
    ///
    /// Zero would mean "already equal", which cannot happen on an increment —
    /// the comparison is evaluated on each increment of the counter, so an
    /// equality that already holds is one the previous increment reported. It
    /// therefore counts as a full wrap of the timer's width, which is what the
    /// counter would really have to do to make them equal again.
    fn until(&self, counter: u64) -> u64 {
        let now = self.now(counter);
        let comp = self.now(self.comp);
        if self.narrow() {
            let delta = (comp as u32).wrapping_sub(now as u32);
            if delta == 0 {
                1 << 32
            } else {
                u64::from(delta)
            }
        } else {
            let delta = comp.wrapping_sub(now);
            // A full 64-bit wrap does not fit in a `u64`; `MAX` is one tick
            // short of it and is as far ahead as anything can be scheduled.
            if delta == 0 { u64::MAX } else { delta }
        }
    }

    /// Move the comparator past `counter` by whole periods.
    fn reload(&mut self, counter: u64) {
        if self.period == 0 {
            // A period of zero would be an infinite loop of expiries. Nothing
            // in the specification defines it; leaving the comparator alone
            // makes the timer behave as a one-shot that has already fired,
            // which is the least surprising of the available answers.
            return;
        }
        let now = self.now(counter);
        let comp = self.now(self.comp);
        let period = self.now(self.period).max(1);
        // How many whole periods it takes to get strictly ahead of the counter,
        // computed rather than looped: a guest that leaves a one-tick periodic
        // timer running for a second must not cost a second of iterations.
        let behind = if self.narrow() {
            u64::from((now as u32).wrapping_sub(comp as u32))
        } else {
            now.wrapping_sub(comp)
        };
        let steps = behind / period + 1;
        let advance = steps.wrapping_mul(period);
        self.comp = if self.narrow() {
            (self.comp & !0xffff_ffff) | (comp.wrapping_add(advance) & 0xffff_ffff)
        } else {
            self.comp.wrapping_add(advance)
        };
    }
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    /// The general configuration register.
    conf: u64,
    /// The general interrupt status register: one bit per timer.
    status: u64,
    /// The main counter.
    counter: u64,
    /// The comparators.
    timers: [Timer; TIMERS],
    /// The tick, in this part's own clock domain, it has been advanced to.
    tick: u64,
}

impl State {
    /// Whether the main counter is running (§2.3.5: `ENABLE_CNF` "overall
    /// enable ... 0 = halt main count and disable all timer interrupts").
    fn running(&self) -> bool {
        self.conf & CONF_ENABLE != 0
    }

    /// Ticks until the soonest comparator match, if the counter is running.
    fn next_event(&self) -> Option<u64> {
        if !self.running() {
            return None;
        }
        self.timers
            .iter()
            .filter(|t| t.conf & TIMER_ENABLE != 0)
            .map(|t| t.until(self.counter))
            .min()
    }

    /// Advance the counter by `span`, reporting which timers expired.
    ///
    /// Several periods inside one span collapse into one interrupt, which is
    /// what the hardware does too: a level-triggered timer's status bit is
    /// already set and an edge-triggered one's pulse has already been sent.
    fn step(&mut self, span: u64) -> [bool; TIMERS] {
        let mut fired = [false; TIMERS];
        if !self.running() || span == 0 {
            return fired;
        }
        let counter = self.counter;
        let after = counter.wrapping_add(span);
        for (index, timer) in self.timers.iter_mut().enumerate() {
            if timer.conf & TIMER_ENABLE == 0 {
                continue;
            }
            if timer.until(counter) > span {
                continue;
            }
            fired[index] = true;
            if timer.conf & TIMER_PERIODIC != 0 {
                timer.reload(after);
            }
        }
        self.counter = after;
        fired
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// One output pin per timer, at [`LockRank::LEAF`] so a line can be driven
    /// with nothing else held.
    outs: Mutex<[Option<WireSource>; TIMERS]>,
    /// The catch-up handle the register paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// The counter period this part reports, in femtoseconds.
    period_fs: u64,
    /// The vendor identification it reports.
    vendor: u16,
    /// [`State::tick`], published so [`Device::current_tick`] can answer with
    /// no lock, which the scheduler requires of it.
    tick: AtomicU64,
    /// The absolute tick of the next comparator match, or [`u64::MAX`] for
    /// none. Same no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("period_fs", &self.period_fs);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Republish the two lock-free numbers. Called with the state lock held.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event() {
            Some(d) => state.tick.saturating_add(d),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Drive the output pins. Never called with the state lock held.
    ///
    /// `pulse` is for an edge-triggered timer, whose interrupt is an event
    /// rather than a condition: the line goes up and straight back down, so a
    /// controller sees an edge whatever it was holding before.
    fn drive(&self, levels: [bool; TIMERS], pulse: [bool; TIMERS]) {
        let sources = self.outs.lock().clone();
        for ((source, level), pulse) in sources.iter().zip(levels).zip(pulse) {
            let Some(source) = source else { continue };
            if pulse {
                source.set(Level::High);
                source.set(Level::Low);
            } else {
                source.set(Level::from_bool(level));
            }
        }
    }

    /// The general capabilities and identification register (§2.3.4).
    fn capabilities(&self) -> u64 {
        REV_ID
            | ((TIMERS as u64 - 1) << 8)
            // 64-bit main counter, and the legacy replacement route is offered.
            | (1 << 13)
            | (1 << 15)
            | (u64::from(self.vendor) << 16)
            | (self.period_fs << 32)
    }

    /// One timer's configuration register as a read reports it.
    fn timer_conf(&self, timer: &Timer) -> u64 {
        timer.conf
            | TIMER_PERIODIC_CAP
            | TIMER_SIZE_CAP
            // `Tn_INT_ROUTE_CAP`: exactly the one input the board wired, which
            // is the truthful answer for a soldered output.
            | (u64::from(1u32 << (timer.route & 31)) << 32)
    }

    /// Read one 64-bit register.
    fn read_register(&self, state: &State, offset: u64) -> u64 {
        match offset {
            REG_CAP => self.capabilities(),
            REG_CONF => state.conf,
            REG_STATUS => state.status,
            REG_COUNTER => state.counter,
            _ if offset >= REG_TIMER_BASE => {
                let index = ((offset - REG_TIMER_BASE) / REG_TIMER_STRIDE) as usize;
                let within = (offset - REG_TIMER_BASE) % REG_TIMER_STRIDE;
                let Some(timer) = state.timers.get(index) else {
                    return 0;
                };
                match within {
                    0x00 => self.timer_conf(timer),
                    0x08 => timer.comp,
                    // The FSB interrupt route register, reserved here because
                    // the capability is not advertised.
                    _ => 0,
                }
            }
            // Reserved. "Reads return zero" is the specification's own answer
            // for the reserved space around the register file (§2.3).
            _ => 0,
        }
    }

    /// Write one 64-bit register.
    fn write_register(&self, state: &mut State, offset: u64, value: u64) {
        match offset {
            REG_CONF => state.conf = value & (CONF_ENABLE | CONF_LEGACY),
            REG_STATUS => {
                // Write one to clear, per bit (§2.3.6). A level-triggered
                // timer's output follows the bit, which is what makes the
                // register the acknowledge.
                state.status &= !value;
                for (index, timer) in state.timers.iter_mut().enumerate() {
                    if value & (1 << index) != 0 {
                        timer.output = false;
                    }
                }
            }
            // "Writes to this register should only be done when the counter is
            // halted" (§2.3.7). Accepted whenever it comes, because refusing
            // would be inventing a fault the part does not raise.
            REG_COUNTER => state.counter = value,
            _ if offset >= REG_TIMER_BASE => {
                let index = ((offset - REG_TIMER_BASE) / REG_TIMER_STRIDE) as usize;
                let within = (offset - REG_TIMER_BASE) % REG_TIMER_STRIDE;
                let counter = state.counter;
                let Some(timer) = state.timers.get_mut(index) else {
                    return;
                };
                match within {
                    0x00 => timer.conf = value & TIMER_WRITABLE,
                    0x08 => {
                        if timer.conf & TIMER_PERIODIC == 0 {
                            timer.comp = value;
                            timer.period = value;
                        } else {
                            // Periodic. The value written is always "the last
                            // value written", which is what an expiry adds
                            // (§2.3.9); it reaches the accumulator as well only
                            // while `Tn_VAL_SET_CNF` is set, and that bit is
                            // cleared by the write.
                            timer.period = value;
                            if timer.conf & TIMER_VAL_SET != 0 {
                                timer.comp = value;
                                timer.conf &= !TIMER_VAL_SET;
                            }
                        }
                        let _ = counter;
                    }
                    // The FSB route register: reserved, and a write to reserved
                    // space is dropped.
                    _ => {}
                }
            }
            // The capabilities register is read-only, and so is everything
            // reserved.
            _ => {}
        }
    }

    /// Catch the part up before an access is dispatched to it (§4.2).
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
        // A refusal means catch-up for this device is already running further
        // up the stack; the access still has to be answered from where the
        // device stands.
        let _ = handle.sync(kind);
    }

    /// Advance to `target` of this part's own clock domain.
    fn advance_to(&self, target: u64) {
        let (levels, pulse) = {
            let mut state = self.state.lock();
            if target <= state.tick {
                // Running backwards is a no-op, not an error.
                return;
            }
            let span = target - state.tick;
            state.tick = target;
            let fired = state.step(span);
            let mut pulse = [false; TIMERS];
            for (index, fired) in fired.into_iter().enumerate() {
                if !fired {
                    continue;
                }
                let level = state.timers[index].conf & TIMER_LEVEL != 0;
                if level {
                    // "This bit is set by hardware if the timer is set to
                    // level-triggered mode" (§2.3.6), and software clears it.
                    state.status |= 1 << index;
                    state.timers[index].output = true;
                } else {
                    pulse[index] = true;
                }
            }
            self.publish(&state);
            (state.timers.map(|t| t.output), pulse)
        };
        self.drive(levels, pulse);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !attrs.debug {
            self.sync(attrs);
        }
        let state = self.state.lock();
        // A 32-bit access reaches one half of a 64-bit register, which is how
        // every 32-bit driver reads the main counter (§2.4.7 recommends exactly
        // that sequence).
        let aligned = offset & !7;
        let value = self.read_register(&state, aligned);
        match dst.len() {
            4 => {
                let half = if offset & 4 == 0 {
                    value as u32
                } else {
                    (value >> 32) as u32
                };
                dst.copy_from_slice(&half.to_le_bytes());
                Ok(())
            }
            8 => {
                dst.copy_from_slice(&value.to_le_bytes());
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // There is no harmless write: the enable bit starts the counter,
            // the status register acknowledges an interrupt, and a comparator
            // write changes when the guest is next interrupted (`ROADMAP.md`
            // §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let aligned = offset & !7;
        let (levels, drive) = {
            let mut state = self.state.lock();
            let old = self.read_register(&state, aligned);
            let value = match src.len() {
                4 => {
                    let half = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
                    if offset & 4 == 0 {
                        (old & 0xffff_ffff_0000_0000) | u64::from(half)
                    } else {
                        (old & 0xffff_ffff) | (u64::from(half) << 32)
                    }
                }
                8 => u64::from_le_bytes([
                    src[0], src[1], src[2], src[3], src[4], src[5], src[6], src[7],
                ]),
                _ => return Err(BusError::BadAccess),
            };
            self.write_register(&mut state, aligned, value);
            self.publish(&state);
            // Only the status register can lower an output, and only from
            // inside this critical section, so the pins are re-driven for it
            // and left alone otherwise.
            (state.timers.map(|t| t.output), aligned == REG_STATUS)
        };
        if drive {
            self.drive(levels, [false; TIMERS]);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // 64-bit registers a 32-bit driver reaches one half at a time (§2.4.7),
        // so both widths, naturally aligned.
        AccessConstraints::word(Width::U64, Endian::Little).with_widths(Width::U32, Width::U64)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An IA-PC high precision event timer.
#[derive(Debug)]
pub struct Hpet {
    regs: Arc<Registers>,
    region: RegionRef,
    /// Which I/O APIC input each timer's pin is wired to, for
    /// `Tn_INT_ROUTE_CAP`. Kept so a reset can restore them.
    routes: [u32; TIMERS],
}

impl Hpet {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `period` is zero or
    /// longer than the 100 ns the specification permits, if a route names an
    /// input above 31, or if a property this class does not know was given.
    pub fn new(props: &Props) -> Result<Hpet> {
        let mut r = props.reader();
        let period_fs = r.or_range::<u64>("period", DEFAULT_PERIOD_FS, 1..=MAX_PERIOD_FS)?;
        let vendor = u16::try_from(r.or_range::<u64>("vendor", 0x8086, 0..=0xffff)?).unwrap_or(0);
        let mut routes = [0u32; TIMERS];
        for (index, route) in routes.iter_mut().enumerate() {
            // The defaults are the PC's legacy replacement assignments where
            // they exist — timer 0 on IRQ0's input and timer 1 on IRQ8's — so a
            // board that says nothing advertises the routing it is most likely
            // to have been wired for.
            let default = match index {
                0 => 2,
                1 => 8,
                _ => 0,
            };
            *route = r.or_range::<u64>(&format!("route{index}"), default, 0..=31)? as u32;
        }
        r.finish()?;
        Ok(Hpet::with_config(period_fs, vendor, routes))
    }

    /// One in the default configuration: a 10 MHz counter and three timers.
    #[must_use]
    pub fn default_device() -> Hpet {
        Hpet::with_config(DEFAULT_PERIOD_FS, 0x8086, [2, 8, 0])
    }

    /// One with the period, vendor and routing given.
    #[must_use]
    pub fn with_config(period_fs: u64, vendor: u16, routes: [u32; TIMERS]) -> Hpet {
        let mut state = State::default();
        for (timer, route) in state.timers.iter_mut().zip(routes) {
            timer.route = route;
        }
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, state),
            outs: Mutex::with_rank(LockRank::LEAF, [const { None }; TIMERS]),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            period_fs,
            vendor,
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Hpet {
            regs,
            region,
            routes,
        }
    }

    /// The counter period this part reports, in femtoseconds.
    #[must_use]
    pub fn period_fs(&self) -> u64 {
        self.regs.period_fs
    }

    /// The main counter, as a guest read would see it.
    #[must_use]
    pub fn counter(&self) -> u64 {
        self.regs.state.lock().counter
    }

    /// Advance to `tick` of this part's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a scheduler
    /// calls this.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// The tick this part has been advanced to.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Which timer's output pin `port` names, if it names one.
    fn out_pin(port: &str) -> Option<usize> {
        let index: usize = port.strip_prefix('t')?.parse().ok()?;
        (index < TIMERS).then_some(index)
    }
}

/// The `pc.hpet` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "IA-PC high precision event timer",
    properties: &[
        PropertySpec {
            name: "period",
            kind: ValueKind::Uint,
            required: false,
            summary: "the main counter's period in femtoseconds, at most 10^8 (default 10^8)",
        },
        PropertySpec {
            name: "vendor",
            kind: ValueKind::Uint,
            required: false,
            summary: "the vendor identification the capability register reports (default 0x8086)",
        },
        PropertySpec {
            name: "route0",
            kind: ValueKind::Uint,
            required: false,
            summary: "which interrupt input timer 0's pin is wired to, 0-31 (default 2)",
        },
        PropertySpec {
            name: "route1",
            kind: ValueKind::Uint,
            required: false,
            summary: "which interrupt input timer 1's pin is wired to, 0-31 (default 8)",
        },
        PropertySpec {
            name: "route2",
            kind: ValueKind::Uint,
            required: false,
            summary: "which interrupt input timer 2's pin is wired to, 0-31 (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Hpet::new(props)?)),
};

impl Device for Hpet {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. The counter comes back at zero and disabled, which is the
        // state a driver expects to find (§2.3.5: `ENABLE_CNF` "is set to 0 on
        // reset").
        let levels = {
            let mut state = self.regs.state.lock();
            *state = State::default();
            for (timer, route) in state.timers.iter_mut().zip(self.routes) {
                timer.route = route;
            }
            self.regs.publish(&state);
            [false; TIMERS]
        };
        self.regs.drive(levels, [false; TIMERS]);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let index = Hpet::out_pin(port).ok_or_else(|| Error::Config {
            at: port.to_string(),
            message: String::from("an HPET drives one pin per timer, `t0` to `t2`"),
        })?;
        self.regs.outs.lock()[index] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some(index) = Hpet::out_pin(port) else {
            return;
        };
        // Every output idles low, but a restored machine may have a
        // level-triggered timer still asserting, so it is announced from state.
        let level = self.regs.state.lock().timers[index].output;
        let source = self.regs.outs.lock()[index].clone();
        if let Some(source) = source {
            source.set(Level::from_bool(level));
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
        w.write_u64(state.conf)?;
        w.write_u64(state.status)?;
        w.write_u64(state.counter)?;
        w.write_seq_len(TIMERS as u64)?;
        for timer in &state.timers {
            w.write_u64(timer.conf)?;
            w.write_u64(timer.comp)?;
            w.write_u64(timer.period)?;
            w.write_bool(timer.output)?;
        }
        // The part's own position in its domain, for the reason the 8254's
        // `save` gives: the scheduler restores the domain, and without this the
        // two would disagree.
        w.write_u64(state.tick)
        // The routing is the board's wiring, not this part's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State {
            conf: r.read_u64()?,
            status: r.read_u64()?,
            counter: r.read_u64()?,
            ..State::default()
        };
        let count = r.read_seq_len(25)? as usize;
        if count != TIMERS {
            return Err(Error::State(format!(
                "snapshot has {count} HPET comparators, this part has {TIMERS}"
            )));
        }
        for (timer, route) in state.timers.iter_mut().zip(self.routes) {
            timer.conf = r.read_u64()?;
            timer.comp = r.read_u64()?;
            timer.period = r.read_u64()?;
            timer.output = r.read_bool()?;
            timer.route = route;
        }
        state.tick = r.read_u64()?;
        let levels = {
            let mut current = self.regs.state.lock();
            *current = state;
            self.regs.publish(&current);
            current.timers.map(|t| t.output)
        };
        self.regs.drive(levels, [false; TIMERS]);
        Ok(())
    }
}

impl Instance for Hpet {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Hpet::new(props)?)))
}

/// What the validator should know about `pc.hpet`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("period", ValueKind::Uint).range(1, MAX_PERIOD_FS))
        .prop(PropSchema::new("vendor", ValueKind::Uint).range(0, 0xffff))
        .region("")
        .region("regs");
    for index in 0..TIMERS {
        schema = schema
            .prop(PropSchema::new(format!("route{index}"), ValueKind::Uint).range(0, 31))
            .port(format!("t{index}"), PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering as AtomicOrdering};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

    /// One HPET with all three outputs wired to probes.
    struct Bench {
        hpet: Hpet,
        probes: [Arc<Probe>; TIMERS],
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), AtomicOrdering::Relaxed);
            if level.is_high() {
                self.edges.fetch_add(1, AtomicOrdering::Relaxed);
            }
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(AtomicOrdering::Relaxed) != 0
        }

        fn edges(&self) -> u32 {
            self.edges.load(AtomicOrdering::Relaxed)
        }
    }

    fn bench() -> Bench {
        let hpet = Hpet::default_device();
        let ids = WireIdAllocator::new();
        let probes: [Arc<Probe>; TIMERS] = core::array::from_fn(|_| Arc::new(Probe::default()));
        for (index, probe) in probes.iter().enumerate() {
            let src = ids.alloc();
            let wire = Wire::builder()
                .source(src)
                .sink(Arc::clone(probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            hpet.connect(&format!("t{index}"), WireSource::new(wire, src))
                .expect("every timer has an output pin");
        }
        Bench { hpet, probes }
    }

    impl Bench {
        fn poke(&self, offset: u64, value: u64) {
            self.hpet
                .regs
                .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .expect("an aligned 64-bit write is legal");
        }

        fn peek(&self, offset: u64) -> u64 {
            self.peek_with(offset, MemAttrs::DEFAULT)
        }

        fn peek_with(&self, offset: u64, attrs: MemAttrs) -> u64 {
            let mut bytes = [0u8; 8];
            self.hpet
                .regs
                .read(offset, &mut bytes, attrs)
                .expect("an aligned 64-bit read is legal");
            u64::from_le_bytes(bytes)
        }

        fn timer(&self, index: u64) -> u64 {
            REG_TIMER_BASE + index * REG_TIMER_STRIDE
        }

        fn enable(&self) {
            self.poke(REG_CONF, CONF_ENABLE);
        }
    }

    #[test]
    fn the_capability_register_describes_the_part() {
        let b = bench();
        let cap = b.peek(REG_CAP);
        assert_eq!(cap & 0xff, REV_ID);
        assert_eq!((cap >> 8) & 0x1f, TIMERS as u64 - 1, "three timers");
        assert_ne!(cap & (1 << 13), 0, "a 64-bit main counter");
        assert_ne!(cap & (1 << 15), 0, "and the legacy replacement route");
        assert_eq!((cap >> 16) & 0xffff, 0x8086);
        assert_eq!(
            cap >> 32,
            DEFAULT_PERIOD_FS,
            "100 ns in femtoseconds, exactly as declared and never derived"
        );
        const { assert!(DEFAULT_PERIOD_FS <= MAX_PERIOD_FS) };
    }

    #[test]
    fn the_counter_stands_still_until_it_is_enabled() {
        let b = bench();
        b.hpet.advance_to(1_000);
        assert_eq!(b.peek(REG_COUNTER), 0, "halted means halted");
        assert_eq!(Device::next_event_tick(&b.hpet), None);
        b.enable();
        b.hpet.advance_to(1_500);
        assert_eq!(
            b.peek(REG_COUNTER),
            500,
            "and it counts from where the enable happened, not from the epoch"
        );
    }

    #[test]
    fn a_one_shot_comparator_fires_on_the_tick_the_scheduler_was_told_about() {
        let b = bench();
        b.enable();
        b.poke(b.timer(0), TIMER_ENABLE | TIMER_LEVEL);
        b.poke(b.timer(0) + 8, 100);

        // The tick the interrupt lands on is arithmetic on the counter and the
        // comparator — two integers — published for the scheduler to stop at.
        // No clock is read to compute it, and this is the assertion that would
        // fail if one were.
        assert_eq!(Device::next_event_tick(&b.hpet), Some(100));

        b.hpet.advance_to(99);
        assert!(!b.probes[0].high());
        assert_eq!(b.peek(REG_STATUS), 0);

        b.hpet.advance_to(100);
        assert!(b.probes[0].high(), "the line went up on the match");
        assert_eq!(b.peek(REG_STATUS) & 1, 1, "and the status bit with it");

        // Write one to clear, which is the acknowledge (spec 2.3.6).
        b.poke(REG_STATUS, 1);
        assert!(!b.probes[0].high());
        assert_eq!(b.peek(REG_STATUS), 0);
    }

    #[test]
    fn the_counters_position_is_a_function_of_its_tick_and_nothing_else() {
        // The falsifiable form of "a device never reads the wall clock": a
        // thousand reads take real time and move the counter by nothing.
        let b = bench();
        b.enable();
        b.hpet.advance_to(42);
        for _ in 0..1_000 {
            assert_eq!(b.peek(REG_COUNTER), 42);
        }
    }

    #[test]
    fn a_periodic_timer_takes_its_accumulator_then_its_period() {
        let b = bench();
        b.enable();
        // The sequence every driver uses (spec 2.3.9.2.2): set the periodic and
        // value-set bits, write the first expiry, then write the period.
        b.poke(
            b.timer(1),
            TIMER_ENABLE | TIMER_LEVEL | TIMER_PERIODIC | TIMER_VAL_SET,
        );
        b.poke(b.timer(1) + 8, 100);
        assert_eq!(
            b.peek(b.timer(1)) & TIMER_VAL_SET,
            0,
            "the write clears the value-set bit"
        );
        b.poke(b.timer(1) + 8, 50);
        assert_eq!(
            b.peek(b.timer(1) + 8),
            100,
            "the comparator stands where it was"
        );

        assert_eq!(Device::next_event_tick(&b.hpet), Some(100));
        b.hpet.advance_to(100);
        assert!(b.probes[1].high());
        assert_eq!(
            b.peek(b.timer(1) + 8),
            150,
            "and hardware added the last value written"
        );
        assert_eq!(Device::next_event_tick(&b.hpet), Some(150));

        b.poke(REG_STATUS, 1 << 1);
        // A long step covering seven periods collapses into one interrupt and
        // leaves the phase exactly where the arithmetic says.
        b.hpet.advance_to(487);
        assert_eq!(b.peek(b.timer(1) + 8), 500);
    }

    #[test]
    fn an_edge_triggered_timer_pulses_and_sets_no_status_bit() {
        let b = bench();
        b.enable();
        b.poke(b.timer(2), TIMER_ENABLE);
        b.poke(b.timer(2) + 8, 10);
        b.hpet.advance_to(10);
        assert_eq!(b.probes[2].edges(), 1, "one edge");
        assert!(!b.probes[2].high(), "and the line came back down");
        assert_eq!(
            b.peek(REG_STATUS),
            0,
            "the status bit is a level-triggered timer's, not an edge one's"
        );
    }

    #[test]
    fn a_thirty_two_bit_timer_compares_only_the_low_half() {
        let b = bench();
        b.enable();
        b.poke(b.timer(0), TIMER_ENABLE | TIMER_LEVEL | TIMER_32BIT);
        // A comparator whose top half is nonsense: in 32-bit mode it is not
        // looked at, so the match is at 0x20 of the low half.
        b.poke(b.timer(0) + 8, 0xdead_beef_0000_0020);
        assert_eq!(Device::next_event_tick(&b.hpet), Some(0x20));
        b.hpet.advance_to(0x20);
        assert!(b.probes[0].high());
    }

    #[test]
    fn a_thirty_two_bit_access_reaches_one_half_of_a_sixty_four_bit_register() {
        let b = bench();
        b.enable();
        b.hpet.advance_to(0x1_0000_0005);
        let mut low = [0u8; 4];
        let mut high = [0u8; 4];
        b.hpet
            .regs
            .read(REG_COUNTER, &mut low, MemAttrs::DEFAULT)
            .unwrap();
        b.hpet
            .regs
            .read(REG_COUNTER + 4, &mut high, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(u32::from_le_bytes(low), 5);
        assert_eq!(u32::from_le_bytes(high), 1);

        // And a 32-bit write leaves the other half alone, which is what makes
        // a two-step comparator write work.
        b.poke(b.timer(0) + 8, 0);
        b.hpet
            .regs
            .write(b.timer(0) + 12, &7u32.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(b.peek(b.timer(0) + 8), 7 << 32);
    }

    #[test]
    fn a_debug_read_moves_nothing_and_a_debug_write_is_refused() {
        let b = bench();
        b.enable();
        b.poke(b.timer(0), TIMER_ENABLE | TIMER_LEVEL);
        b.poke(b.timer(0) + 8, 10);
        // A debugger read must not advance the counter, which is the one thing
        // a lazily advanced device does on a guest access.
        assert_eq!(b.peek_with(REG_COUNTER, MemAttrs::DEBUG), 0);
        assert_eq!(b.hpet.tick(), 0);
        assert!(
            b.hpet
                .regs
                .write(REG_STATUS, &1u64.to_le_bytes(), MemAttrs::DEBUG)
                .is_err(),
            "and there is no harmless write on this part"
        );
    }

    #[test]
    fn a_snapshot_round_trips_the_whole_part() {
        let saved = bench();
        saved.enable();
        saved.poke(
            saved.timer(0),
            TIMER_ENABLE | TIMER_LEVEL | TIMER_PERIODIC | TIMER_VAL_SET,
        );
        saved.poke(saved.timer(0) + 8, 64);
        saved.poke(saved.timer(0) + 8, 64);
        saved.poke(saved.timer(2), TIMER_ENABLE);
        saved.poke(saved.timer(2) + 8, 1_000);
        saved.hpet.advance_to(70);

        let mut shape = MachineShape::new();
        shape.add_device("hpet", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("hpet", CLASS.name, CLASS.version).unwrap();
            saved.hpet.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = bench();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("hpet", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.hpet.load(&mut chunk.reader()).unwrap();

        // Copied out one at a time: two parts' state locks are both at
        // `LockRank::DEVICE`.
        let after = restored.hpet.regs.state.lock().clone();
        let before = saved.hpet.regs.state.lock().clone();
        assert_eq!(after, before, "every field came back");
        assert_eq!(restored.hpet.tick(), 70, "the position in its domain too");
        assert_eq!(
            Device::next_event_tick(&restored.hpet),
            Device::next_event_tick(&saved.hpet)
        );
        assert!(
            restored.probes[0].high(),
            "and a level-triggered timer that was asserting still is"
        );

        let mut shape = MachineShape::new();
        shape.add_device("hpet", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("hpet", CLASS.name, CLASS.version).unwrap();
            restored.hpet.save(&mut chunk).unwrap();
        }
        assert_eq!(w.to_vec().unwrap(), bytes);
    }

    #[test]
    fn a_reset_halts_the_counter_and_drops_every_line() {
        let b = bench();
        b.enable();
        b.poke(b.timer(0), TIMER_ENABLE | TIMER_LEVEL);
        b.poke(b.timer(0) + 8, 5);
        b.hpet.advance_to(5);
        assert!(b.probes[0].high());
        b.hpet.reset(crate::core::device::ResetKind::Cold);
        assert!(!b.probes[0].high());
        assert_eq!(b.peek(REG_COUNTER), 0);
        assert_eq!(Device::next_event_tick(&b.hpet), None);
    }

    #[test]
    fn a_period_longer_than_a_hundred_nanoseconds_is_refused() {
        // The specification's own cap (2.3.4). A machine file that declared a
        // slower counter would be describing a part no driver will accept.
        let props = Props::new().with("period", MAX_PERIOD_FS + 1);
        assert!(Hpet::new(&props).is_err());
        let props = Props::new().with("period", MAX_PERIOD_FS);
        assert!(Hpet::new(&props).is_ok());
    }
}
