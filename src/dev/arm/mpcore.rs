//! The Cortex-A9 MPCore private memory region: the Snoop Control Unit, the
//! global timer, and each processor's private timer and watchdog.
//!
//! # Sources
//!
//! *ARM Cortex-A9 MPCore Technical Reference Manual* (DDI 0407): §1.5 for the
//! private memory region's map, chapter 2 for the Snoop Control Unit's
//! registers (§2.2.1 Control, §2.2.2 Configuration, §2.2.3 CPU Power Status,
//! §2.2.4 Invalidate All, §2.2.5-§2.2.6 Filtering Start and End, §2.2.7-§2.2.8
//! the access control registers), and chapter 4 for the timers: §4.1 the
//! private timer and watchdog blocks with their register maps (§4.2), the
//! interval formula `((PRESCALER + 1) × (Load + 1)) / PERIPHCLK`, the watchdog
//! mode and the `0x12345678`, `0x87654321` disable sequence; §4.3-§4.4 the
//! global timer, its banked comparator, control and auto-increment registers.
//! Interrupt IDs are DDI 0407 §3.2's: 27 global timer, 29 private timer, 30
//! watchdog, each a private peripheral interrupt of its own processor.
//!
//! No emulator source and no driver source of any licence was consulted.
//!
//! # The region, and which part of it is here
//!
//! ```text
//!   +0x0000  Snoop Control Unit                  region `scu`     (this class)
//!   +0x0100  interrupt controller CPU interface  `arm.gic` (version = 1) `cpu`
//!   +0x0200  global timer                        region `global`  (this class)
//!   +0x0600  private timers and watchdogs        region `private` (this class)
//!   +0x1000  interrupt distributor               `arm.gic` (version = 1) `dist`
//! ```
//!
//! The interrupt controller is *not* here: it is [`arm.gic`](super::gic) with
//! `version = 1`, whose programmers' model is the one a GICv1 driver expects at
//! these offsets. Two classes rather than one because they are two register
//! files that share nothing — the timers reach the controller through the same
//! kind of wire any other device does — and because duplicating two thousand
//! lines of interrupt controller to save a machine file three `map` statements
//! is the wrong trade.
//!
//! # Which processor is asking
//!
//! The timer blocks answer *per processor* at one address, exactly as the
//! GIC's banked registers do, so this class resolves requesters to processor
//! numbers the same way — a `processors = [cpu0, cpu1, …]` list resolved at
//! bind time through [`BindCtx::peer`](crate::machine::BindCtx::peer), and the
//! same fallback: an access from something that is not one of those
//! processors (a debugger, a DMA engine, a test) sees processor 0's bank.
//! A machine file writes the same list for both objects.
//!
//! # Time
//!
//! Every counter here is clocked by `PERIPHCLK` divided by its prescaler, so
//! this is a **lazily advanced** device (`ROADMAP.md` §4.2) whose clock domain
//! *is* `PERIPHCLK`: one domain tick is one `PERIPHCLK` cycle, and a machine
//! file chooses the rate — `clock = cpuclk / 2` is the usual A9 integration.
//! Nothing counts per tick. Each counter is kept as a value at an *anchor* tick
//! plus the prescaler's phase at that tick, so its value at any later tick is
//! one division and its next expiry is one multiplication; the scheduler is
//! told only the soonest expiry, and a timer left running for a minute costs
//! the same as one left running for a microsecond. All of it is integer
//! arithmetic (`CLAUDE.md`, determinism).
//!
//! Counter *reads* are answered where the reading processor stands in the
//! domain (`LazyHandle::reader_tick`), and *writes* that start or reload a
//! counter anchor it where the writing processor stands
//! (`LazyHandle::writer_tick`) — the same two rules `pc.hpet` follows, for the
//! same reason: on a crystal the processors share, catch-up stops where the
//! round began, and a timer armed there would fire up to a round early.
//!
//! # What the outputs are
//!
//! Per processor `n`: `gt<n>` (ID 27), `twd<n>` (ID 29) and `wdt<n>` (ID 30),
//! each a **level** — the block's event flag and its interrupt enable — to be
//! wired to that processor's private interrupt input on the GIC
//! (`gic.cpu<n>ppi11`, `gic.cpu<n>ppi13`, `gic.cpu<n>ppi14`); and
//! `wdreset<n>`, `WDRESETREQ`, a **pulse** on the watchdog expiring in
//! watchdog mode, for whatever the board's reset controller is.
//!
//! # What is not modelled
//!
//! * The SCU keeps nothing coherent, because host memory already is. Its
//!   registers are stored and read back; *Invalidate All* completes the moment
//!   it is written.
//! * The SCU Configuration register's SMP bits (7:4) mirror each processor's
//!   `ACTLR.SMP` on silicon. This block cannot see a processor's `ACTLR`, so it
//!   reports every present processor as taking part in coherency — what the
//!   register says on a running SMP system, which is the only time software
//!   reads it.
//! * The global timer's comparator is modelled on the r2p0-and-later rule
//!   that the event fires when the counter is *greater than or equal to* the
//!   comparator (DDI 0407 §4.4.1): a comparator written behind a running
//!   counter fires immediately. This model fires on the condition *becoming*
//!   true — at a write that arms it, or at the tick the counter crosses it —
//!   and does not re-raise the flag while it merely stays true.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.a9mpcore";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The most processors a Cortex-A9 MPCore has (the Configuration register's
/// CPU number field is two bits, DDI 0407 §2.2.2).
pub const MAX_CPUS: u64 = 4;

/// How much address space the SCU answers, at private-region offset `0x0000`.
pub const SCU_WINDOW_LEN: u64 = 0x100;

/// How much address space the global timer answers, at offset `0x0200`.
pub const GLOBAL_WINDOW_LEN: u64 = 0x100;

/// How much address space the private timer and watchdog answer, at offset
/// `0x0600`.
pub const PRIVATE_WINDOW_LEN: u64 = 0x100;

/// The private-region offsets of the three windows, for a board that wants to
/// compute its `map` statements rather than copy them.
pub const SCU_OFFSET: u64 = 0x0000;
/// See [`SCU_OFFSET`].
pub const GLOBAL_OFFSET: u64 = 0x0200;
/// See [`SCU_OFFSET`].
pub const PRIVATE_OFFSET: u64 = 0x0600;

/// The first word of the watchdog disable sequence (DDI 0407 §4.2.1).
const WD_DISABLE_1: u32 = 0x1234_5678;
/// The second word.
const WD_DISABLE_2: u32 = 0x8765_4321;

/// Control register bits shared by the private timer and the watchdog.
const CTRL_ENABLE: u32 = 1 << 0;
const CTRL_AUTO_RELOAD: u32 = 1 << 1;
const CTRL_IRQ_ENABLE: u32 = 1 << 2;
/// Watchdog control bit 3: watchdog mode rather than timer mode.
const CTRL_WD_MODE: u32 = 1 << 3;
/// The prescaler field, 15:8, in every control register here.
const CTRL_PRESCALER: u32 = 0xff00;

/// Global timer control bits (DDI 0407 §4.4.3). Timer enable and the
/// prescaler are common to all processors; the other three are banked.
const GT_ENABLE: u32 = 1 << 0;
const GT_COMP_ENABLE: u32 = 1 << 1;
const GT_IRQ_ENABLE: u32 = 1 << 2;
const GT_AUTO_INC: u32 = 1 << 3;
/// The banked half of the global timer's control register.
const GT_BANKED: u32 = GT_COMP_ENABLE | GT_IRQ_ENABLE | GT_AUTO_INC;

/// Which bits of SCU Control software may change (DDI 0407 §2.2.1): enable,
/// address filtering, parity, speculative linefills, force-to-port-0, SCU
/// standby and IC standby.
const SCU_CTRL_MASK: u32 = 0x7f;

/// The output pins each processor has, in the order they are stored.
const PIN_NAMES: [&str; 4] = ["gt", "twd", "wdt", "wdreset"];
const PIN_GT: usize = 0;
const PIN_TWD: usize = 1;
const PIN_WDT: usize = 2;
const PIN_WDRESET: usize = 3;

// ---------------------------------------------------------------------------
// the arithmetic
// ---------------------------------------------------------------------------

/// Where a down-counter stands `n` decrements after holding `count`, and
/// whether it reached zero on the way.
///
/// "Reached zero" means a decrement *arrived* at zero: a counter already at
/// zero at the start has fired for that already. With auto-reload the
/// decrement after zero loads `load` (DDI 0407 §4.2.2), so zero recurs every
/// `load + 1` decrements — the TRM's `(Load + 1)` in the interval formula.
/// Without it the counter stays at zero.
fn count_down(count: u32, n: u64, load: u32, reload: bool) -> (u32, bool) {
    let count = u64::from(count);
    if n == 0 {
        return (count as u32, false);
    }
    if count > 0 && n < count {
        return ((count - n) as u32, false);
    }
    let (past, fired) = if count > 0 {
        (n - count, true)
    } else {
        (n, false)
    };
    if !reload {
        return (0, fired);
    }
    let period = u64::from(load) + 1;
    let fired = fired || past >= period;
    let into = past % period;
    let value = if into == 0 { 0 } else { period - into };
    (value as u32, fired)
}

/// A private timer or a watchdog: one 32-bit down-counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Down {
    /// The Load register.
    load: u32,
    /// The counter's value at [`Down::at`].
    count: u32,
    /// The Control register.
    ctrl: u32,
    /// The Interrupt Status register's event flag.
    flag: bool,
    /// The anchor: the domain tick [`Down::count`] is the value at. Never
    /// behind the device's own tick while the counter runs.
    at: u64,
    /// How many `PERIPHCLK` ticks toward the next decrement had already
    /// elapsed at [`Down::at`] — always less than the prescaler's divisor.
    phase: u64,
}

impl Down {
    fn running(&self) -> bool {
        self.ctrl & CTRL_ENABLE != 0
    }

    /// `PRESCALER + 1`: how many `PERIPHCLK` ticks one decrement takes.
    fn divisor(&self) -> u64 {
        u64::from((self.ctrl & CTRL_PRESCALER) >> 8) + 1
    }

    /// Whether reaching zero reloads the counter. Never in watchdog mode,
    /// where reaching zero is a reset request rather than a period.
    fn reloads(&self) -> bool {
        self.ctrl & CTRL_AUTO_RELOAD != 0 && self.ctrl & CTRL_WD_MODE == 0
    }

    /// Decrements between the anchor and `t`.
    fn decrements(&self, t: u64) -> u64 {
        if !self.running() || t <= self.at {
            return 0;
        }
        (t - self.at + self.phase) / self.divisor()
    }

    /// The counter's value at `t`, moving nothing.
    fn value_at(&self, t: u64) -> u32 {
        count_down(self.count, self.decrements(t), self.load, self.reloads()).0
    }

    /// The tick the counter next arrives at zero, if it will.
    fn next_zero(&self) -> Option<u64> {
        if !self.running() {
            return None;
        }
        let k = if self.count > 0 {
            u64::from(self.count)
        } else if self.reloads() {
            u64::from(self.load) + 1
        } else {
            return None;
        };
        // `phase < divisor <= k * divisor`, so this cannot go below `at`.
        Some(
            self.at
                .saturating_add(k.saturating_mul(self.divisor()))
                .saturating_sub(self.phase),
        )
    }

    /// Carry the counter to `t`, re-anchoring there, and report whether it
    /// arrived at zero on the way. A `t` at or behind the anchor moves
    /// nothing: a writer ahead of the device anchored it there.
    fn advance(&mut self, t: u64) -> bool {
        if t <= self.at {
            return false;
        }
        if !self.running() {
            self.at = t;
            return false;
        }
        let elapsed = t - self.at + self.phase;
        let divisor = self.divisor();
        let (count, fired) = count_down(self.count, elapsed / divisor, self.load, self.reloads());
        self.count = count;
        self.phase = elapsed % divisor;
        self.at = t;
        fired
    }

    /// Put a new value in the counter at `t`, restarting the prescaler.
    fn set_count(&mut self, count: u32, t: u64) {
        self.count = count;
        self.at = self.at.max(t);
        self.phase = 0;
    }
}

/// The global timer's shared half: the 64-bit counter, its enable and its
/// prescaler.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Global {
    /// The counter's value at [`Global::at`].
    counter: u64,
    /// The common bits of the Control register: enable and prescaler.
    ctrl: u32,
    /// The anchor tick, as for [`Down::at`].
    at: u64,
    /// The prescaler's phase at the anchor, as for [`Down::phase`].
    phase: u64,
}

impl Global {
    fn running(&self) -> bool {
        self.ctrl & GT_ENABLE != 0
    }

    fn divisor(&self) -> u64 {
        u64::from((self.ctrl & CTRL_PRESCALER) >> 8) + 1
    }

    /// The counter at `t`, moving nothing. It only ever counts up.
    fn counter_at(&self, t: u64) -> u64 {
        if !self.running() || t <= self.at {
            return self.counter;
        }
        self.counter
            .wrapping_add((t - self.at + self.phase) / self.divisor())
    }

    /// The tick the counter first holds `value`, if that is still ahead.
    fn tick_of(&self, value: u64) -> Option<u64> {
        if !self.running() || value <= self.counter {
            return None;
        }
        let k = value - self.counter;
        Some(
            self.at
                .saturating_add(k.saturating_mul(self.divisor()))
                .saturating_sub(self.phase),
        )
    }

    /// Re-anchor at `t`.
    fn rebase(&mut self, t: u64) {
        if t <= self.at {
            return;
        }
        if self.running() {
            let elapsed = t - self.at + self.phase;
            let divisor = self.divisor();
            self.counter = self.counter.wrapping_add(elapsed / divisor);
            self.phase = elapsed % divisor;
        }
        self.at = t;
    }
}

/// One processor's bank of the global timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct GlobalBank {
    /// The banked control bits: comparator enable, IRQ enable, auto-increment.
    ctrl: u32,
    /// The Interrupt Status register's event flag.
    flag: bool,
    comparator: u64,
    auto_increment: u32,
}

impl GlobalBank {
    fn armed(&self) -> bool {
        self.ctrl & GT_COMP_ENABLE != 0
    }

    /// The comparator matched with the counter at `now`: raise the flag, and
    /// with auto-increment step the comparator strictly past `now` — once per
    /// event on silicon, collapsed here into however many whole increments
    /// that takes, as the counter is not stopped to deliver each one.
    fn fire(&mut self, now: u64) {
        self.flag = true;
        if self.ctrl & GT_AUTO_INC != 0 && self.auto_increment != 0 {
            let step = u64::from(self.auto_increment);
            let behind = now.wrapping_sub(self.comparator);
            let steps = behind / step + 1;
            self.comparator = self.comparator.wrapping_add(steps.wrapping_mul(step));
        }
    }
}

/// Everything one processor owns in the region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct PerCpu {
    global: GlobalBank,
    timer: Down,
    watchdog: Down,
    /// The Watchdog Reset Status register: the watchdog expired in watchdog
    /// mode. Survives a warm reset, which is the point of it — software reads
    /// it after the reset to learn why there was one.
    wd_reset: bool,
    /// The first word of the disable sequence has been written, and the next
    /// write to the Watchdog Disable register completes it or breaks it.
    wd_disarming: bool,
}

impl PerCpu {
    /// The four output levels, in [`PIN_NAMES`] order. The reset request is a
    /// pulse and is driven separately, so it idles low here.
    fn levels(&self) -> [bool; 4] {
        [
            self.global.flag && self.global.ctrl & GT_IRQ_ENABLE != 0,
            self.timer.flag && self.timer.ctrl & CTRL_IRQ_ENABLE != 0,
            self.watchdog.flag && self.watchdog.ctrl & CTRL_IRQ_ENABLE != 0,
            false,
        ]
    }
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    scu_ctrl: u32,
    scu_power: u32,
    filter_start: u32,
    filter_end: u32,
    sac: u32,
    snsac: u32,
    global: Global,
    cpus: Vec<PerCpu>,
    /// The tick, in `PERIPHCLK`, this block has been advanced to.
    tick: u64,
}

impl State {
    fn new(cpus: usize, config: &Config) -> State {
        State {
            scu_ctrl: 0,
            scu_power: 0,
            filter_start: config.filter_start,
            filter_end: config.filter_end,
            // Every processor may reach the SCU's registers out of reset
            // (DDI 0407 §2.2.7); none may from the non-secure side (§2.2.8).
            sac: (1 << cpus) - 1,
            snsac: 0,
            global: Global::default(),
            cpus: alloc::vec![PerCpu::default(); cpus],
            tick: 0,
        }
    }

    /// The soonest thing any counter does, as an absolute tick.
    fn next_event(&self) -> Option<u64> {
        let mut best: Option<u64> = None;
        let mut take = |at: Option<u64>| {
            if let Some(at) = at {
                best = Some(best.map_or(at, |b| b.min(at)));
            }
        };
        for cpu in &self.cpus {
            take(cpu.timer.next_zero());
            take(cpu.watchdog.next_zero());
            if cpu.global.armed() {
                take(self.global.tick_of(cpu.global.comparator));
            }
        }
        best
    }

    /// Carry everything to `t`; report which processors' watchdogs asked for
    /// a reset.
    fn advance(&mut self, t: u64) -> Vec<bool> {
        let mut resets = alloc::vec![false; self.cpus.len()];
        let before = self.global.counter_at(self.tick);
        let after = self.global.counter_at(t);
        if self.global.running() {
            for cpu in &mut self.cpus {
                let g = &mut cpu.global;
                if g.armed() && before < g.comparator && g.comparator <= after {
                    g.fire(after);
                }
            }
        }
        self.global.rebase(t);
        for (index, cpu) in self.cpus.iter_mut().enumerate() {
            if cpu.timer.advance(t) {
                cpu.timer.flag = true;
            }
            let watchdog_mode = cpu.watchdog.ctrl & CTRL_WD_MODE != 0;
            if cpu.watchdog.advance(t) {
                if watchdog_mode {
                    cpu.wd_reset = true;
                    resets[index] = true;
                } else {
                    cpu.watchdog.flag = true;
                }
            }
        }
        self.tick = self.tick.max(t);
        resets
    }
}

/// What the machine file configured, kept so a reset can put it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Config {
    /// The L1 data cache size, for the Configuration register's tag RAM field.
    dcache: u64,
    filter_start: u32,
    filter_end: u32,
}

/// The register blocks, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// Per processor, the four output pins in [`PIN_NAMES`] order, at
    /// [`LockRank::LEAF`].
    outs: Mutex<Vec<[Option<WireSource>; 4]>>,
    /// The catch-up handle (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// Which requester each processor number answers for; see the module docs.
    owners: Vec<AtomicU32>,
    /// [`State::tick`], published for [`Device::current_tick`], which may not
    /// take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next expiry, or [`u64::MAX`] for none.
    next_event: AtomicU64,
    cpus: usize,
    config: Config,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("cpus", &self.cpus);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Which processor an access came from — the GIC's rule, for the GIC's
    /// reason (`arm.gic`, `Registers::cpu_of`).
    fn cpu_of(&self, attrs: MemAttrs) -> usize {
        let id = attrs.requester.0;
        if id == RequesterId::ANONYMOUS.0 {
            return 0;
        }
        self.owners
            .iter()
            .position(|owner| owner.load(Ordering::Relaxed) == id)
            .unwrap_or(0)
    }

    /// Republish the lock-free numbers. Called with the state lock held.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        self.next_event
            .store(state.next_event().unwrap_or(u64::MAX), Ordering::Relaxed);
    }

    /// Drive every level output, and pulse the reset requests that fired.
    /// Never called with the state lock held.
    fn drive(&self, levels: &[[bool; 4]], resets: &[bool]) {
        let outs = self.outs.lock().clone();
        for ((pins, levels), reset) in outs.iter().zip(levels).zip(resets) {
            for pin in [PIN_GT, PIN_TWD, PIN_WDT] {
                if let Some(out) = &pins[pin] {
                    out.set(Level::from_bool(levels[pin]));
                }
            }
            if *reset && let Some(out) = &pins[PIN_WDRESET] {
                out.set(Level::High);
                out.set(Level::Low);
            }
        }
    }

    fn levels(state: &State) -> Vec<[bool; 4]> {
        state.cpus.iter().map(PerCpu::levels).collect()
    }

    fn handle(&self) -> Option<LazyHandle> {
        self.lazy.lock().clone()
    }

    /// Catch up before an access (§4.2).
    fn sync(&self, attrs: MemAttrs) {
        let Some(handle) = self.handle() else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up is already running further up the stack;
        // the access is answered from where the block stands.
        let _ = handle.sync(kind);
    }

    /// Where a reading processor stands in `PERIPHCLK`, or zero where nothing
    /// can say — see `pc.hpet`'s `reader_tick` for the whole argument.
    fn reader_tick(&self, attrs: MemAttrs) -> u64 {
        if attrs.debug {
            return 0;
        }
        self.handle()
            .map_or(0, |h| h.reader_tick(attrs.requester.0))
    }

    /// Where a writing processor stands, when catch-up could not put the block
    /// there.
    fn writer_tick(&self, attrs: MemAttrs) -> Option<u64> {
        self.handle()?.writer_tick(attrs.requester.0)
    }

    fn advance_to(&self, target: u64) {
        let (levels, resets) = {
            let mut state = self.state.lock();
            if target <= state.tick {
                return;
            }
            let resets = state.advance(target);
            self.publish(&state);
            (Self::levels(&state), resets)
        };
        self.drive(&levels, &resets);
    }

    // -- the SCU ------------------------------------------------------------

    fn scu_read(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            0x00 => state.scu_ctrl,
            0x04 => {
                // DDI 0407 §2.2.2: CPU number in 1:0 as (processors - 1), the
                // SMP bits in 7:4, and two bits of tag RAM size per processor
                // in 15:8 — 0b00 for a 16 KiB data cache, 0b01 for 32 KiB,
                // 0b10 for 64 KiB.
                let tag = match self.config.dcache {
                    0x4000 => 0b00,
                    0x8000 => 0b01,
                    _ => 0b10,
                };
                let mut value = (self.cpus as u32 - 1) & 3;
                for cpu in 0..self.cpus as u32 {
                    value |= 1 << (4 + cpu);
                    value |= tag << (8 + 2 * cpu);
                }
                value
            }
            0x08 => state.scu_power,
            0x40 => state.filter_start,
            0x44 => state.filter_end,
            0x50 => state.sac,
            0x54 => state.snsac,
            // Invalidate All is write-only; everything else is reserved.
            _ => 0,
        }
    }

    fn scu_write(&self, offset: u64, value: u32) {
        let mut state = self.state.lock();
        let present = (1u32 << self.cpus) - 1;
        match offset {
            0x00 => state.scu_ctrl = value & SCU_CTRL_MASK,
            0x08 => {
                // Two bits of power mode per processor, one byte each.
                let mut mask = 0u32;
                for cpu in 0..self.cpus {
                    mask |= 0b11 << (8 * cpu);
                }
                state.scu_power = value & mask;
            }
            // Invalidate All: there are no duplicate tags to invalidate, so it
            // is complete the moment it is written.
            0x0c => {}
            // Bits 31:20 are the 1 MiB-aligned address; the rest reads zero.
            0x40 => state.filter_start = value & 0xfff0_0000,
            0x44 => state.filter_end = value & 0xfff0_0000,
            0x50 => state.sac = value & present,
            // Per processor: component access (bits 3:0), private timer
            // (7:4) and global timer (11:8) from the non-secure side.
            0x54 => {
                let mut mask = 0u32;
                for group in 0..3 {
                    mask |= present << (4 * group);
                }
                state.snsac = value & mask;
            }
            _ => {}
        }
    }

    // -- the global timer ---------------------------------------------------

    fn global_read(&self, offset: u64, cpu: usize, reader: u64) -> u32 {
        let state = self.state.lock();
        let bank = &state.cpus[cpu].global;
        match offset {
            0x00 => state.global.counter_at(state.tick.max(reader)) as u32,
            0x04 => (state.global.counter_at(state.tick.max(reader)) >> 32) as u32,
            0x08 => (state.global.ctrl & (GT_ENABLE | CTRL_PRESCALER)) | bank.ctrl,
            0x0c => u32::from(bank.flag),
            0x10 => bank.comparator as u32,
            0x14 => (bank.comparator >> 32) as u32,
            0x18 => bank.auto_increment,
            _ => 0,
        }
    }

    fn global_write(&self, offset: u64, cpu: usize, value: u32, now: u64) {
        let mut state = self.state.lock();
        let counter = state.global.counter_at(now);
        // Whether this write can make "counter >= comparator" newly true for
        // this processor, which is when the >= rule fires at once.
        let mut arm = false;
        match offset {
            // The counter may only be written while the timer is stopped
            // (DDI 0407 §4.4.1); a write to a running one is ignored.
            0x00 | 0x04 => {
                if !state.global.running() {
                    state.global.rebase(now);
                    let old = state.global.counter;
                    state.global.counter = if offset == 0 {
                        (old & !0xffff_ffff) | u64::from(value)
                    } else {
                        (old & 0xffff_ffff) | (u64::from(value) << 32)
                    };
                }
            }
            0x08 => {
                let was_running = state.global.running();
                let common = value & (GT_ENABLE | CTRL_PRESCALER);
                if common != state.global.ctrl {
                    // Stop, start or re-divide: re-anchor where the writer
                    // stands, and a start restarts the prescaler.
                    state.global.rebase(now);
                    state.global.ctrl = common;
                    if !was_running {
                        state.global.phase = 0;
                    }
                }
                let bank = &mut state.cpus[cpu].global;
                let was_armed = bank.armed();
                bank.ctrl = value & GT_BANKED;
                arm = (bank.armed() && !was_armed) || (state.global.running() && !was_running);
            }
            0x0c => {
                if value & 1 != 0 {
                    state.cpus[cpu].global.flag = false;
                }
            }
            0x10 | 0x14 => {
                let bank = &mut state.cpus[cpu].global;
                bank.comparator = if offset == 0x10 {
                    (bank.comparator & !0xffff_ffff) | u64::from(value)
                } else {
                    (bank.comparator & 0xffff_ffff) | (u64::from(value) << 32)
                };
                arm = true;
            }
            0x18 => state.cpus[cpu].global.auto_increment = value,
            _ => {}
        }
        if arm && state.global.running() {
            // A start arms every processor's comparator at once; the other
            // writes only this processor's.
            let targets: Vec<usize> = if offset == 0x08 {
                (0..state.cpus.len()).collect()
            } else {
                alloc::vec![cpu]
            };
            for index in targets {
                let bank = &mut state.cpus[index].global;
                if bank.armed() && counter >= bank.comparator {
                    bank.fire(counter);
                }
            }
        }
    }

    // -- the private timer and watchdog ------------------------------------

    fn private_read(&self, offset: u64, cpu: usize, reader: u64) -> u32 {
        let state = self.state.lock();
        let at = state.tick.max(reader);
        let bank = &state.cpus[cpu];
        match offset {
            0x00 => bank.timer.load,
            0x04 => bank.timer.value_at(at),
            0x08 => bank.timer.ctrl,
            0x0c => u32::from(bank.timer.flag),
            0x20 => bank.watchdog.load,
            0x24 => bank.watchdog.value_at(at),
            0x28 => bank.watchdog.ctrl,
            0x2c => u32::from(bank.watchdog.flag),
            0x30 => u32::from(bank.wd_reset),
            // The disable register is write-only.
            _ => 0,
        }
    }

    /// Returns whether the watchdog asked for a reset on the way to `now`.
    fn private_write(&self, offset: u64, cpu: usize, value: u32, now: u64) -> bool {
        let mut state = self.state.lock();
        let bank = &mut state.cpus[cpu];
        // Bring the addressed counter to the writer's instant first, so the
        // write lands on the value it would have read there. An expiry on the
        // way is delivered, not lost.
        let watchdog = (0x20..0x38).contains(&offset);
        let mut reset = false;
        if watchdog {
            let mode = bank.watchdog.ctrl & CTRL_WD_MODE != 0;
            if bank.watchdog.advance(now) {
                if mode {
                    bank.wd_reset = true;
                    reset = true;
                } else {
                    bank.watchdog.flag = true;
                }
            }
        } else if bank.timer.advance(now) {
            bank.timer.flag = true;
        }
        // Any register write other than the second word breaks the disable
        // sequence (DDI 0407 §4.2.1: the two words must be written
        // consecutively).
        let disarming = core::mem::replace(&mut bank.wd_disarming, false);
        match offset {
            // "Writing to the Load Register also sets the Counter Register"
            // — for both blocks, and in watchdog mode that is the kick.
            0x00 => {
                bank.timer.load = value;
                bank.timer.set_count(value, now);
            }
            0x04 => bank.timer.set_count(value, now),
            0x08 => {
                let starting = !bank.timer.running() && value & CTRL_ENABLE != 0;
                let represcale = (bank.timer.ctrl ^ value) & CTRL_PRESCALER != 0;
                bank.timer.ctrl = value & (CTRL_PRESCALER | 0b111);
                if starting || represcale {
                    let count = bank.timer.count;
                    bank.timer.set_count(count, now);
                }
            }
            0x0c => {
                if value & 1 != 0 {
                    bank.timer.flag = false;
                }
            }
            0x20 => {
                bank.watchdog.load = value;
                bank.watchdog.set_count(value, now);
            }
            0x24 => bank.watchdog.set_count(value, now),
            0x28 => {
                let starting = !bank.watchdog.running() && value & CTRL_ENABLE != 0;
                let represcale = (bank.watchdog.ctrl ^ value) & CTRL_PRESCALER != 0;
                // Watchdog mode is set by writing one and cleared only by the
                // disable sequence or a reset (DDI 0407 §4.2.1).
                let mode = (bank.watchdog.ctrl | value) & CTRL_WD_MODE;
                bank.watchdog.ctrl = (value & (CTRL_PRESCALER | 0b111)) | mode;
                if starting || represcale {
                    let count = bank.watchdog.count;
                    bank.watchdog.set_count(count, now);
                }
            }
            0x2c => {
                if value & 1 != 0 {
                    bank.watchdog.flag = false;
                }
            }
            0x30 => {
                if value & 1 != 0 {
                    bank.wd_reset = false;
                }
            }
            0x34 => {
                if value == WD_DISABLE_1 {
                    bank.wd_disarming = true;
                } else if value == WD_DISABLE_2 && disarming {
                    bank.watchdog.ctrl &= !CTRL_WD_MODE;
                }
            }
            _ => {}
        }
        reset
    }
}

/// Which window an access is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Window {
    Scu,
    Global,
    Private,
}

/// One of the three windows: a [`MemOps`] over the shared state.
#[derive(Debug)]
struct Aperture {
    regs: Arc<Registers>,
    window: Window,
}

impl MemOps for Aperture {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let len = dst.len() as u64;
        let narrow_ok = self.window == Window::Scu;
        if !(len == 4 || (narrow_ok && matches!(len, 1 | 2))) || !offset.is_multiple_of(len) {
            return Err(BusError::BadAccess);
        }
        // No read here has a side effect — every status bit is write-one-to-
        // clear — so `debug` changes only whether time is caught up.
        if !attrs.debug {
            self.regs.sync(attrs);
        }
        let word = offset & !3;
        let cpu = self.regs.cpu_of(attrs);
        let value = match self.window {
            Window::Scu => self.regs.scu_read(word),
            Window::Global => {
                let reader = if word <= 0x04 {
                    self.regs.reader_tick(attrs)
                } else {
                    0
                };
                self.regs.global_read(word, cpu, reader)
            }
            Window::Private => {
                let reader = if word == 0x04 || word == 0x24 {
                    self.regs.reader_tick(attrs)
                } else {
                    0
                };
                self.regs.private_read(word, cpu, reader)
            }
        };
        let bytes = value.to_le_bytes();
        let at = (offset & 3) as usize;
        dst.copy_from_slice(&bytes[at..at + dst.len()]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Every write here acknowledges an interrupt, starts a counter, or
            // moves an expiry (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let len = src.len() as u64;
        // The CPU Power Status register is byte-accessible, so each processor
        // can write its own power mode without a read-modify-write of the
        // others' (DDI 0407 §2.2.3). Everything else is a word.
        let power = self.window == Window::Scu && (0x08..0x0c).contains(&offset);
        if !(len == 4 || (power && matches!(len, 1 | 2))) || !offset.is_multiple_of(len) {
            return Err(BusError::BadAccess);
        }
        self.regs.sync(attrs);
        let cpu = self.regs.cpu_of(attrs);
        let word = offset & !3;
        let value = if len == 4 {
            u32::from_le_bytes([src[0], src[1], src[2], src[3]])
        } else {
            // A narrow write to the power register: merge it into the word,
            // which has no side effect to read.
            let mut bytes = self.regs.scu_read(word).to_le_bytes();
            let at = (offset & 3) as usize;
            bytes[at..at + src.len()].copy_from_slice(src);
            u32::from_le_bytes(bytes)
        };
        if self.window == Window::Scu {
            self.regs.scu_write(word, value);
            return Ok(());
        }
        // Taken before the state lock: the handle takes leaf locks of its own.
        let writer = self.regs.writer_tick(attrs);
        let now = {
            let state = self.regs.state.lock();
            writer.map_or(state.tick, |w| w.max(state.tick))
        };
        let fired = match self.window {
            Window::Global => {
                self.regs.global_write(word, cpu, value, now);
                false
            }
            _ => self.regs.private_write(word, cpu, value, now),
        };
        let levels = {
            let state = self.regs.state.lock();
            self.regs.publish(&state);
            Registers::levels(&state)
        };
        let mut resets = alloc::vec![false; self.regs.cpus];
        resets[cpu] = fired;
        // Outside every lock: these reach the interrupt controller.
        self.regs.drive(&levels, &resets);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        match self.window {
            Window::Scu => AccessConstraints::IO
                .with_widths(Width::U8, Width::U32)
                .with_natural_alignment(true)
                .with_endian(Endian::Little),
            _ => AccessConstraints::word(Width::U32, Endian::Little),
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The Cortex-A9 MPCore's SCU, global timer, and private timers and
/// watchdogs.
#[derive(Debug)]
pub struct A9MpCore {
    regs: Arc<Registers>,
    scu: RegionRef,
    global: RegionRef,
    private: RegionRef,
    /// The machine-file paths of the processors, in processor-number order.
    processors: Vec<String>,
}

impl A9MpCore {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a processor count the SCU cannot report, a
    /// data cache size it has no encoding for, a `processors` list that does
    /// not match `cpus`, or a property this class does not know.
    pub fn new(props: &Props) -> Result<A9MpCore> {
        let mut r = props.reader();
        let cpus = r.or_range("cpus", 1u64, 1..=MAX_CPUS)?;
        let dcache = r.or_size("dcache", 32 * 1024)?;
        let filter_start = r.or_range("filter-start", 0u64, 0..=0xffff_ffff)?;
        let filter_end = r.or_range("filter-end", 0u64, 0..=0xffff_ffff)?;
        let processors = match r.optional_list("processors")? {
            Some(items) => items
                .iter()
                .map(|v| Ok(v.to_link("processors")?.as_str().to_string()))
                .collect::<Result<Vec<String>>>()?,
            None => Vec::new(),
        };
        r.finish()?;
        if !matches!(dcache, 0x4000 | 0x8000 | 0x10000) {
            return Err(Error::Property(format!(
                "`dcache` is the L1 data cache size the SCU's tag RAM field reports, and a \
                 Cortex-A9 has 16K, 32K or 64K; {dcache} bytes is none of them"
            )));
        }
        // The GIC's rule and the GIC's reason: without a map every processor
        // reads processor 0's timers.
        if cpus > 1 && processors.is_empty() {
            return Err(Error::Property(format!(
                "`cpus` is {cpus}, so the private timers answer differently per processor — \
                 add `processors = [<core>, …]` naming the {cpus} processors in order"
            )));
        }
        if !processors.is_empty() && processors.len() as u64 != cpus {
            return Err(Error::Property(format!(
                "`processors` names {} processor(s) and `cpus` says there are {cpus}",
                processors.len()
            )));
        }
        let mut dev = A9MpCore::build(
            cpus as usize,
            dcache,
            (filter_start as u32) & 0xfff0_0000,
            (filter_end as u32) & 0xfff0_0000,
        );
        dev.processors = processors;
        Ok(dev)
    }

    /// Build one directly, for a test or a hand-wired machine.
    #[must_use]
    pub fn build(cpus: usize, dcache: u64, filter_start: u32, filter_end: u32) -> A9MpCore {
        let config = Config {
            dcache,
            filter_start,
            filter_end,
        };
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(cpus, &config)),
            outs: Mutex::with_rank(LockRank::LEAF, alloc::vec![[const { None }; 4]; cpus]),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            owners: (0..cpus)
                .map(|_| AtomicU32::new(RequesterId::ANONYMOUS.0))
                .collect(),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            cpus,
            config,
        });
        let window = |name: &str, len: u64, window: Window| -> RegionRef {
            Arc::new(Region::io(
                name,
                len,
                Arc::new(Aperture {
                    regs: Arc::clone(&regs),
                    window,
                }) as Arc<dyn MemOps>,
            ))
        };
        A9MpCore {
            scu: window("arm.a9mpcore.scu", SCU_WINDOW_LEN, Window::Scu),
            global: window("arm.a9mpcore.global", GLOBAL_WINDOW_LEN, Window::Global),
            private: window("arm.a9mpcore.private", PRIVATE_WINDOW_LEN, Window::Private),
            regs,
            processors: Vec::new(),
        }
    }

    /// Say that `requester`'s accesses are processor `cpu`'s. Returns `false`
    /// for a processor this block does not have, or a requester another
    /// processor number already answers for.
    pub fn attach_processor(&self, cpu: usize, requester: RequesterId) -> bool {
        if cpu >= self.regs.cpus || requester == RequesterId::ANONYMOUS {
            return false;
        }
        if self
            .regs
            .owners
            .iter()
            .enumerate()
            .any(|(i, o)| i != cpu && o.load(Ordering::Relaxed) == requester.0)
        {
            return false;
        }
        self.regs.owners[cpu].store(requester.0, Ordering::Relaxed);
        true
    }

    /// How many processors it serves.
    #[must_use]
    pub fn cpus(&self) -> usize {
        self.regs.cpus
    }

    /// Advance to `tick` of `PERIPHCLK`. What [`Device::advance_to`] does; a
    /// test with no scheduler calls this.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// The tick it has been advanced to.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Which processor and pin a port name refers to: `gt<n>`, `twd<n>`,
    /// `wdt<n>` or `wdreset<n>`.
    fn pin(&self, port: &str) -> Option<(usize, usize)> {
        for (index, name) in PIN_NAMES.iter().enumerate() {
            if let Some(rest) = port.strip_prefix(name)
                && let Ok(cpu) = rest.parse::<usize>()
                && cpu < self.regs.cpus
            {
                return Some((cpu, index));
            }
        }
        None
    }
}

/// The `arm.a9mpcore` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Cortex-A9 MPCore private region: SCU, global timer, private timers and watchdogs",
    properties: &[
        PropertySpec {
            name: "cpus",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many Cortex-A9 processors, 1-4 (default 1)",
        },
        PropertySpec {
            name: "processors",
            kind: ValueKind::List,
            required: false,
            summary: "the processors in processor-number order, as `arm.gic` takes them; \
                      required once `cpus` is more than one",
        },
        PropertySpec {
            name: "dcache",
            kind: ValueKind::Size,
            required: false,
            summary: "the L1 data cache size the SCU reports per processor: 16K, 32K or 64K \
                      (default 32K)",
        },
        PropertySpec {
            name: "filter-start",
            kind: ValueKind::Uint,
            required: false,
            summary: "the SCU Filtering Start register's reset value (bits 31:20; default 0)",
        },
        PropertySpec {
            name: "filter-end",
            kind: ValueKind::Uint,
            required: false,
            summary: "the SCU Filtering End register's reset value (bits 31:20; default 0)",
        },
    ],
    construct: |props| Ok(Box::new(A9MpCore::new(props)?)),
};

impl Device for A9MpCore {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` statements place the windows and `wire`
        // statements bring the pins.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let levels = {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            // The Watchdog Reset Status register survives any reset but a
            // power-on one: it exists to be read *after* the reset the
            // watchdog caused (DDI 0407 §4.2.1).
            let kept: Vec<bool> = state.cpus.iter().map(|c| c.wd_reset).collect();
            *state = State::new(self.regs.cpus, &self.regs.config);
            // The tick is this block's cursor in its domain, not a register;
            // `pc.hpet`'s reset says why it must survive.
            state.tick = tick;
            state.global.at = tick;
            for cpu in &mut state.cpus {
                cpu.timer.at = tick;
                cpu.watchdog.at = tick;
            }
            if kind != ResetKind::Cold {
                for (cpu, kept) in state.cpus.iter_mut().zip(kept) {
                    cpu.wd_reset = kept;
                }
            }
            self.regs.publish(&state);
            Registers::levels(&state)
        };
        self.regs
            .drive(&levels, &alloc::vec![false; self.regs.cpus]);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "scu" => Some(Arc::clone(&self.scu)),
            "global" => Some(Arc::clone(&self.global)),
            "private" => Some(Arc::clone(&self.private)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let (cpu, pin) = self.pin(port).ok_or_else(|| Error::Config {
            at: port.to_string(),
            message: format!(
                "an A9 MPCore drives `gt<n>`, `twd<n>`, `wdt<n>` and `wdreset<n>` for each of \
                 its {} processor(s); `{port}` is not one of them",
                self.regs.cpus
            ),
        })?;
        self.regs.outs.lock()[cpu][pin] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some((cpu, pin)) = self.pin(port) else {
            return;
        };
        let level = self.regs.state.lock().cpus[cpu].levels()[pin];
        let out = self.regs.outs.lock()[cpu][pin].clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
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

    /// Also asks for `PERIPHCLK` in every processor's read view: the counters
    /// are pure functions of time.
    fn attach_lazy(&self, handle: LazyHandle) {
        handle.read_at_readers();
        *self.regs.lazy.lock() = Some(handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        for word in [
            state.scu_ctrl,
            state.scu_power,
            state.filter_start,
            state.filter_end,
            state.sac,
            state.snsac,
        ] {
            w.write_u32(word)?;
        }
        w.write_u64(state.global.counter)?;
        w.write_u32(state.global.ctrl)?;
        w.write_u64(state.global.at)?;
        w.write_u64(state.global.phase)?;
        w.write_seq_len(state.cpus.len() as u64)?;
        for cpu in &state.cpus {
            w.write_u32(cpu.global.ctrl)?;
            w.write_bool(cpu.global.flag)?;
            w.write_u64(cpu.global.comparator)?;
            w.write_u32(cpu.global.auto_increment)?;
            for down in [&cpu.timer, &cpu.watchdog] {
                w.write_u32(down.load)?;
                w.write_u32(down.count)?;
                w.write_u32(down.ctrl)?;
                w.write_bool(down.flag)?;
                w.write_u64(down.at)?;
                w.write_u64(down.phase)?;
            }
            w.write_bool(cpu.wd_reset)?;
            w.write_bool(cpu.wd_disarming)?;
        }
        w.write_u64(state.tick)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.regs.cpus, &self.regs.config);
        state.scu_ctrl = r.read_u32()?;
        state.scu_power = r.read_u32()?;
        state.filter_start = r.read_u32()?;
        state.filter_end = r.read_u32()?;
        state.sac = r.read_u32()?;
        state.snsac = r.read_u32()?;
        state.global.counter = r.read_u64()?;
        state.global.ctrl = r.read_u32()?;
        state.global.at = r.read_u64()?;
        state.global.phase = r.read_u64()?;
        let cpus = r.read_seq_len(1)? as usize;
        if cpus != self.regs.cpus {
            return Err(Error::State(format!(
                "snapshot has {cpus} A9 processor(s), this block has {}",
                self.regs.cpus
            )));
        }
        for cpu in &mut state.cpus {
            cpu.global.ctrl = r.read_u32()? & GT_BANKED;
            cpu.global.flag = r.read_bool()?;
            cpu.global.comparator = r.read_u64()?;
            cpu.global.auto_increment = r.read_u32()?;
            for down in [&mut cpu.timer, &mut cpu.watchdog] {
                down.load = r.read_u32()?;
                down.count = r.read_u32()?;
                down.ctrl = r.read_u32()?;
                down.flag = r.read_bool()?;
                down.at = r.read_u64()?;
                down.phase = r.read_u64()?;
                if down.phase >= down.divisor() {
                    return Err(Error::State(format!(
                        "a prescaler phase of {} with a divisor of {}",
                        down.phase,
                        down.divisor()
                    )));
                }
            }
            cpu.wd_reset = r.read_bool()?;
            cpu.wd_disarming = r.read_bool()?;
        }
        state.tick = r.read_u64()?;
        let levels = {
            let mut live = self.regs.state.lock();
            *live = state;
            self.regs.publish(&live);
            Registers::levels(&live)
        };
        self.regs
            .drive(&levels, &alloc::vec![false; self.regs.cpus]);
        Ok(())
    }
}

impl Instance for A9MpCore {
    /// Resolve `processors` to requester ids, exactly as `arm.gic` does.
    ///
    /// # Errors
    ///
    /// If `processors` names something this machine does not have, or one
    /// processor twice.
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        for (cpu, path) in self.processors.iter().enumerate() {
            let peer = ctx.peer(path)?;
            if !self.attach_processor(cpu, peer.requester()) {
                return Err(Error::Config {
                    at: ctx.path().to_string(),
                    message: format!(
                        "`processors` names `{path}` for processor {cpu}, but it already answers \
                         for another; a core has one bank of private timers"
                    ),
                });
            }
        }
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(A9MpCore::new(props)?)))
}

/// What the validator should know about `arm.a9mpcore`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, MAX_CPUS))
        .prop(PropSchema::new("processors", ValueKind::List))
        .prop(PropSchema::new("dcache", ValueKind::Size))
        .prop(PropSchema::new("filter-start", ValueKind::Uint).range(0, 0xffff_ffff))
        .prop(PropSchema::new("filter-end", ValueKind::Uint).range(0, 0xffff_ffff))
        .region("scu")
        .region("global")
        .region("private");
    for cpu in 0..MAX_CPUS {
        for name in PIN_NAMES {
            s = s.port(format!("{name}{cpu}"), PortDir::Out);
        }
    }
    s
}

#[cfg(test)]
mod tests;
