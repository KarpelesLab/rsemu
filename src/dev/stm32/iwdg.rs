//! The STM32 independent watchdog.
//!
//! One class, `st.iwdg`. It is a twelve-bit down-counter on the **LSI**, the
//! low-speed internal RC oscillator, and what makes it *independent* is that
//! it is the only timer on the die that keeps counting when everything the
//! firmware controls has stopped: it is not on the APB, reprogramming the PLL
//! does not move it, and once started nothing short of a reset stops it. That
//! is the whole point of the part. A bootloader arms it before jumping to an
//! application precisely so that an application which hangs is recovered by
//! hardware rather than by somebody with a programmer.
//!
//! # The registers
//!
//! Five of them, at base `0x4000_3000` (RM0090 §21.4, RM0351 §36.4):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `KR` | write-only: `0xCCCC` starts it, `0xAAAA` reloads it, `0x5555` unlocks the three below |
//! | `0x04` | `PR` | prescaler: LSI/4 through LSI/256 |
//! | `0x08` | `RLR` | the twelve-bit value a reload loads |
//! | `0x0c` | `SR` | `PVU`, `RVU`, `WVU`: a write to the matching register is still crossing into the LSI domain |
//! | `0x10` | `WINR` | the window, on the parts that have one |
//!
//! `KR` is the whole protocol and it is a *write-only* register: a read of it
//! returns zero, so firmware cannot ask whether the watchdog is running and
//! the unlock has to be latched rather than inferred. `0x5555`
//! opens `PR`, `RLR` and `WINR` for writing; **any other key closes them
//! again**, including the `0xAAAA` that kicks the dog, so a driver that
//! reloads between two configuration writes loses the second one. That is a
//! real trap and this model reproduces it.
//!
//! `WINR` is absent on an F4 and present from the F0/F3/F7/L0/L4 generation
//! onwards; `window` says which part this is. Where it exists, writing it
//! *also reloads the counter*, and a reload issued while the counter is still
//! above the window is a refresh that came too early — which resets the
//! machine exactly as a refresh that came too late does. A window watchdog
//! catches a program that has started running its main loop twice as fast as
//! it should, which a plain timeout cannot.
//!
//! # Time
//!
//! The device is [lazily advanced](crate::core::sched) on its own clock
//! domain, and a machine file gives it the LSI:
//!
//! ```text
//! osc lsi = 32000 Hz
//! object iwdg "st.iwdg" { clock = lsi }
//! ```
//!
//! One tick of that domain is one LSI cycle, so `PR` and `RLR` mean what the
//! manual's timeout table says they mean: `PR = 0` (divide by four) and
//! `RLR = 0xFFF` is 4 × 4096 = 16384 LSI cycles, which at a nominal 32 kHz is
//! 512 ms. **There is no loop and no host clock here**: the counter's next
//! interesting instant is published as
//! [`next_event_tick`](crate::core::Device::next_event_tick) and the scheduler
//! comes back at it.
//!
//! # Freezing
//!
//! `DBGMCU_APB1FZR1.DBG_IWDG_STOP` stops the counter while a debugger has the
//! core halted, which is what stops a breakpoint from resetting the board
//! under you. The bit belongs to the debug unit rather than to this
//! peripheral, so what this class offers is the `freeze` input pin the debug
//! unit drives; with nothing wired to it the watchdog counts through a halt,
//! which is what the hardware does with the bit clear.
//!
//! # The reset
//!
//! Expiry drives a **pulse on the `reset` pin**, and a board wires that to the
//! core's reset input and to whatever latches `RCC_CSR.IWDGRSTF` — the flag is
//! RCC's state, not this block's, and a watchdog that owned it would be
//! modelling the wrong chip.
//!
//! The pulse is driven **after the state lock is released**, which is the
//! re-entrancy contract and not a detail: everything downstream of a reset
//! line calls straight back into every device on the machine, this one
//! included, and a watchdog that reset the world from inside its own critical
//! section would deadlock against its own [`Device::reset`].
//!
//! Having fired, the counter stops rather than firing again every timeout. On
//! the die that question does not arise — the part is in reset — and a model
//! that kept pulsing a reset line nobody had wired would be a busy loop.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §21 "Independent watchdog (IWDG)" and ST **RM0351**
//! rev 9 §36 for the window register and the freeze bit. No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).

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
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.iwdg";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes the registers occupy. `WINR` is inside it on every part;
/// on one without the window it reads as zero, which is what an unimplemented
/// register in an allotted kilobyte does.
pub const REGISTER_BYTES: u64 = 0x14;

/// `KR`: start counting.
const KEY_START: u32 = 0xcccc;
/// `KR`: reload the counter from `RLR`.
const KEY_RELOAD: u32 = 0xaaaa;
/// `KR`: allow `PR`, `RLR` and `WINR` to be written.
const KEY_UNLOCK: u32 = 0x5555;

/// The widest the down-counter and the reload get: twelve bits.
const COUNTER_MAX: u32 = 0xfff;

/// `SR.PVU`: a `PR` write has not crossed into the LSI domain yet.
const SR_PVU: u32 = 1 << 0;
/// `SR.RVU`: the same for `RLR`.
const SR_RVU: u32 = 1 << 1;
/// `SR.WVU`: the same for `WINR`.
const SR_WVU: u32 = 1 << 2;

/// How many LSI cycles a register update takes to become visible.
///
/// "it takes up to five RC 40 kHz cycles" (RM0090 §21.4.4). Five is the
/// number the manual gives and the one firmware's polling loop is written
/// against; anything shorter makes a `while (IWDG->SR)` loop that never spins,
/// which hides a driver bug rather than reproducing it.
const UPDATE_CYCLES: u64 = 5;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the counter's position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Whether `0xCCCC` has been written. One way: nothing clears it.
    started: bool,
    /// Whether `0x5555` was the last key written.
    unlocked: bool,
    /// `PR[2:0]`.
    pr: u32,
    /// `RLR[11:0]`.
    rlr: u32,
    /// `WINR[11:0]`, on a part that has one.
    winr: u32,
    /// The down-counter.
    counter: u32,
    /// LSI cycles counted towards the next decrement.
    prescale: u32,
    /// Where the device has advanced to, in its own clock domain.
    tick: u64,
    /// The tick each of `PVU`, `RVU` and `WVU` stops reading as one at.
    update: [Option<u64>; 3],
}

impl Default for State {
    fn default() -> State {
        State {
            started: false,
            unlocked: false,
            pr: 0,
            // "Reset value: 0x0000 0FFF" for both (RM0090 §21.4.3).
            rlr: COUNTER_MAX,
            winr: COUNTER_MAX,
            counter: COUNTER_MAX,
            prescale: 0,
            tick: 0,
            update: [None; 3],
        }
    }
}

impl State {
    /// How many LSI cycles one decrement costs.
    ///
    /// "000: divider /4 … 110: divider /256". Bit pattern 111 is reserved and
    /// the table stops at /256, so an out-of-range value saturates there
    /// rather than inventing a divider of 512.
    fn divider(&self) -> u64 {
        4u64 << self.pr.min(6)
    }

    /// LSI cycles from here until the reset, if one is coming.
    fn until_reset(&self, frozen: bool) -> Option<u64> {
        if !self.started || frozen {
            return None;
        }
        let divider = self.divider();
        Some((divider - u64::from(self.prescale)) + u64::from(self.counter) * divider)
    }

    /// LSI cycles until the next thing that changes what the guest can read.
    fn next_event(&self, frozen: bool) -> Option<u64> {
        let mut soonest = self.until_reset(frozen);
        for deadline in self.update.iter().flatten() {
            let delta = deadline.saturating_sub(self.tick);
            soonest = Some(match soonest {
                Some(current) => current.min(delta),
                None => delta,
            });
        }
        soonest
    }

    /// Load the counter from `RLR`. Returns whether the reload came too early
    /// for the window, which is itself a reset (RM0351 §36.3.4).
    fn reload(&mut self) -> bool {
        let early = self.winr < COUNTER_MAX && self.counter > self.winr;
        self.counter = self.rlr;
        self.prescale = 0;
        early
    }

    /// Advance `n` LSI cycles. Returns whether the watchdog expired.
    fn step(&mut self, n: u64, frozen: bool) -> bool {
        let end = self.tick + n;
        let mut fired = false;
        if self.started && !frozen {
            let divider = self.divider();
            let mut remaining = n;
            while remaining > 0 {
                let need = divider - u64::from(self.prescale);
                if remaining < need {
                    // Cannot overflow: `need` is at most `divider`, which is
                    // at most 256, and `remaining` is smaller still.
                    self.prescale += remaining as u32;
                    break;
                }
                remaining -= need;
                self.prescale = 0;
                if self.counter == 0 {
                    // "When the counter reaches 0x000 a reset signal is
                    // generated." The decrement that would take it below zero
                    // is the one that fires.
                    fired = true;
                    self.started = false;
                    break;
                }
                self.counter -= 1;
            }
        }
        self.tick = end;
        for deadline in &mut self.update {
            if deadline.is_some_and(|at| at <= end) {
                *deadline = None;
            }
        }
        fired
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// Whether this part has `WINR`.
    window: bool,
    /// Whether the debug unit is holding the counter still.
    ///
    /// An atomic rather than a field of [`State`] for two reasons: it is read
    /// on every step, and it is **not** the machine's state — it is the debug
    /// unit's, so it stays out of the snapshot the way `st.gpio` keeps pad
    /// levels out of its own.
    frozen: AtomicBool,
    /// The reset output, pulsed on expiry.
    reset_out: Mutex<Option<WireSource>>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// [`State::tick`], republished on every change: the scheduler asks
    /// [`Device::current_tick`] with its slot held at
    /// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF), so that call may
    /// not take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next event, or [`u64::MAX`] for none. Same
    /// no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("window", &self.window)
            .field("frozen", &self.frozen.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Whether the debug unit is holding the counter.
    fn frozen(&self) -> bool {
        self.frozen.load(Ordering::Acquire)
    }

    /// Republish what the lock-free lazy surface reads.
    ///
    /// Called from inside every critical section that can move the counter or
    /// change when it next does something.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event(self.frozen()) {
            Some(delta) => state.tick.saturating_add(delta.max(1)),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Pulse the reset line.
    ///
    /// **Never called with the state lock held.** A reset reaches every device
    /// on the machine, this one included, so the outward call happens after
    /// the critical section (`CLAUDE.md`, "Concurrency").
    fn pulse_reset(&self) {
        let source = self.reset_out.lock().clone();
        if let Some(source) = source {
            source.pulse(Level::High);
        }
    }

    /// Advance to `target` of the watchdog's own clock domain.
    ///
    /// One iteration per internal event rather than one jump: the reset has to
    /// land on the tick it happened on, not at the end of the catch-up. The
    /// scheduler bounds a quantum by [`Device::next_event_tick`], so in a
    /// running machine the loop turns once.
    fn advance_to(&self, target: u64) {
        loop {
            let (reached, fired) = {
                let mut state = self.state.lock();
                if target <= state.tick {
                    return;
                }
                let span = target - state.tick;
                // At least one tick, so catch-up always makes progress.
                let step = state
                    .next_event(self.frozen())
                    .unwrap_or(span)
                    .clamp(1, span);
                let fired = state.step(step, self.frozen());
                self.publish(&state);
                (state.tick >= target, fired)
            };
            if fired {
                self.pulse_reset();
            }
            if reached {
                return;
            }
        }
    }

    /// Catch the watchdog up before an access is dispatched to it (§4.2).
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
        // counter stands.
        let _ = handle.sync(AccessKind::Guest);
    }

    /// The debug unit changed its mind about freezing.
    fn set_frozen(&self, frozen: bool) {
        // Catch up on the old setting first, or the ticks either side of the
        // change are counted under the wrong one.
        self.sync(MemAttrs::DEFAULT);
        self.frozen.store(frozen, Ordering::Release);
        let state = self.state.lock();
        self.publish(&state);
    }

    /// Read one register.
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            // "KR … these bits are write only. Read returns 0x0000."
            0x00 => 0,
            0x04 => state.pr,
            0x08 => state.rlr,
            0x0c => {
                let mut sr = 0;
                for (bit, deadline) in [SR_PVU, SR_RVU, SR_WVU].iter().zip(state.update) {
                    if deadline.is_some() {
                        sr |= bit;
                    }
                }
                sr
            }
            0x10 if self.window => state.winr,
            _ => 0,
        }
    }

    /// Write one register. Returns whether the machine is being reset.
    fn write_register(&self, offset: u64, value: u32) -> bool {
        let mut state = self.state.lock();
        let mut fired = false;
        match offset {
            0x00 => {
                match value & 0xffff {
                    KEY_START => {
                        if !state.started {
                            state.started = true;
                            state.counter = state.rlr;
                            state.prescale = 0;
                        }
                    }
                    KEY_RELOAD => fired = state.reload() && state.started,
                    KEY_UNLOCK => {}
                    _ => {}
                }
                // Only `0x5555` leaves the three configuration registers open,
                // so a kick between two of them closes the window a driver
                // thought it had.
                state.unlocked = value & 0xffff == KEY_UNLOCK;
            }
            0x04 => {
                if state.unlocked {
                    state.pr = value & 0x7;
                    let at = state.tick + UPDATE_CYCLES;
                    state.update[0] = Some(at);
                }
            }
            0x08 => {
                if state.unlocked {
                    state.rlr = value & COUNTER_MAX;
                    let at = state.tick + UPDATE_CYCLES;
                    state.update[1] = Some(at);
                }
            }
            // `SR` is read-only.
            0x0c => {}
            0x10 if self.window && state.unlocked => {
                state.winr = value & COUNTER_MAX;
                let at = state.tick + UPDATE_CYCLES;
                state.update[2] = Some(at);
                // "Writing to the window register also reloads the counter"
                // (RM0351 §36.4.5), and that reload is judged against the
                // window the write just installed rather than the one it
                // replaced — which is what makes the usual initialisation
                // sequence (set WINR, then start) legal.
                fired = state.started && state.counter > state.winr;
                state.counter = state.rlr;
                state.prescale = 0;
            }
            _ => {}
        }
        if fired {
            state.started = false;
        }
        self.publish(&state);
        fired
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        self.sync(attrs);
        // Nothing here clears or advances on a read — `SR`'s flags are cleared
        // by time, not by looking at them — so a debug read is the same read.
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `KR` would start a watchdog nobody armed, or
            // kick one the guest was about to be reset by. There is no
            // harmless version (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        if self.write_register(offset & !3, value) {
            // Outside the critical section, as `pulse_reset` requires.
            self.pulse_reset();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 independent watchdog.
#[derive(Debug)]
pub struct Iwdg {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins the machine layer has taken; a net holds its sinks
    /// weakly, so the device keeps the strong reference.
    pins: Mutex<Vec<Arc<FreezePin>>>,
}

impl Iwdg {
    /// Validate `props` and build the watchdog.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Iwdg> {
        let mut r = props.reader();
        let window = r.or("window", false)?;
        r.touch("clock");
        r.finish()?;
        Ok(Iwdg::with_window(window))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_window(window: bool) -> Iwdg {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            window,
            frozen: AtomicBool::new(false),
            reset_out: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region = Arc::new(Region::io(
            "iwdg",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Iwdg {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Whether this part has a window register.
    #[must_use]
    pub fn has_window(&self) -> bool {
        self.regs.window
    }

    /// Whether `0xCCCC` has started the counter.
    #[must_use]
    pub fn started(&self) -> bool {
        self.regs.state.lock().started
    }

    /// What the down-counter is at.
    #[must_use]
    pub fn counter(&self) -> u32 {
        self.regs.state.lock().counter
    }

    /// The tick the watchdog has been advanced to, in its own domain.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Advance to `tick` of the watchdog's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// Hold or release the counter, as the debug unit's freeze bit does.
    pub fn set_frozen(&self, frozen: bool) {
        self.regs.set_frozen(frozen);
    }
}

impl Device for Iwdg {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the reset line.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: the IWDG is reset by a system reset, which is also what
        // its own expiry causes. `FLASH_OPTR.IWDG_SW` can make the hardware
        // start it out of reset instead; this model has no option bytes, so it
        // always comes up stopped and firmware starts it.
        //
        // The tick survives, for the reason `pc.pit`'s reset gives: it is not
        // architectural state but this device's cursor in its own clock
        // domain, and the domain does not rewind because a chip on it was
        // reset.
        let mut state = self.regs.state.lock();
        let tick = state.tick;
        *state = State::default();
        state.tick = tick;
        self.regs.publish(&state);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_bool(state.started)?;
        w.write_bool(state.unlocked)?;
        w.write_u32(state.pr)?;
        w.write_u32(state.rlr)?;
        w.write_u32(state.winr)?;
        w.write_u32(state.counter)?;
        w.write_u32(state.prescale)?;
        // The cursor in the clock domain: the scheduler restores the domain,
        // and without this the two would disagree and the next catch-up would
        // advance the counter by however long the machine had been running.
        w.write_u64(state.tick)?;
        for deadline in state.update {
            match deadline {
                None => w.write_bool(false)?,
                Some(at) => {
                    w.write_bool(true)?;
                    w.write_u64(at)?;
                }
            }
        }
        Ok(())
        // `frozen` is absent on purpose: it is the debug unit's state, not the
        // machine's, and the pin is re-driven by whatever is debugging.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State {
            started: r.read_bool()?,
            unlocked: r.read_bool()?,
            pr: r.read_u32()?,
            rlr: r.read_u32()?,
            winr: r.read_u32()?,
            counter: r.read_u32()?,
            prescale: r.read_u32()?,
            tick: r.read_u64()?,
            update: [None; 3],
        };
        for deadline in &mut state.update {
            *deadline = r.read_bool()?.then(|| r.read_u64()).transpose()?;
        }
        if state.pr > 7 || state.rlr > COUNTER_MAX || state.counter > COUNTER_MAX {
            return Err(Error::State(format!(
                "snapshot has an IWDG with PR={}, RLR={}, counter={}",
                state.pr, state.rlr, state.counter
            )));
        }
        if u64::from(state.prescale) >= state.divider() {
            return Err(Error::State(format!(
                "snapshot has an IWDG {} LSI cycle(s) into a divide-by-{}",
                state.prescale,
                state.divider()
            )));
        }
        let mut live = self.regs.state.lock();
        *live = state;
        self.regs.publish(&live);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "reset" {
            return Err(Error::Config {
                at: String::from(port),
                message: String::from("an IWDG drives one pin, `reset`"),
            });
        }
        *self.regs.reset_out.lock() = Some(source);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != "freeze" {
            return None;
        }
        let pin = Arc::new(FreezePin {
            regs: Arc::clone(&self.regs),
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

impl Instance for Iwdg {}

/// The `st.iwdg` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 independent watchdog: the LSI down-counter, its key register and its window",
    properties: &[PropertySpec {
        name: "window",
        kind: ValueKind::Bool,
        required: false,
        summary: "whether the part has WINR (an F4 has not; an F0/F3/F7/L0/L4 has)",
    }],
    construct: |props| Ok(Box::new(Iwdg::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Iwdg::new(props)?)))
}

/// What the validator should know about `st.iwdg`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("window", ValueKind::Bool))
        .region("")
        .region("regs")
        .port("reset", PortDir::Out)
        .port("freeze", PortDir::In)
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// The debug unit's freeze input, as something a wire can drive.
#[derive(Debug)]
pub struct FreezePin {
    regs: Arc<Registers>,
    inputs: FanIn,
}

impl FreezePin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for FreezePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.regs
            .set_frozen(self.inputs.resolve(Resolve::Or).is_high());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU32;
    use crate::core::wire::{Wire, WireIdAllocator};

    /// An F4's: no window register.
    fn iwdg() -> Iwdg {
        Iwdg::with_window(false)
    }

    fn peek(d: &Iwdg, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        d.regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(d: &Iwdg, offset: u64, value: u32) {
        d.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Counts the rising edges of the reset line.
    #[derive(Debug, Default)]
    struct Probe {
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            if level.is_high() {
                self.edges.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn watch(d: &Iwdg) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(d, "reset", WireSource::new(wire, id)).expect("the reset pin");
        probe
    }

    fn resets(p: &Probe) -> u32 {
        p.edges.load(Ordering::Relaxed)
    }

    /// Unlock, set the prescaler and the reload, then start.
    fn arm(d: &Iwdg, pr: u32, rlr: u32) {
        poke(d, 0x00, KEY_UNLOCK);
        poke(d, 0x04, pr);
        poke(d, 0x08, rlr);
        poke(d, 0x00, KEY_START);
    }

    #[test]
    fn an_unkicked_watchdog_resets_after_prescaler_times_reload_lsi_ticks() {
        // PR = 0 is a divider of four and RLR = 0xFFF is 4096 decrements, so
        // the timeout is 4 * 4096 = 16384 LSI cycles — the 512 ms at 32 kHz
        // RM0090 Table 96 gives for this pair.
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 0, 0xfff);
        d.advance_to(16_383);
        assert_eq!(resets(&reset), 0, "not yet");
        d.advance_to(16_384);
        assert_eq!(resets(&reset), 1);
        // And it is a one-shot: nothing wired the pulse to a reset, so the
        // counter stops rather than pulsing forever.
        d.advance_to(100_000);
        assert_eq!(resets(&reset), 1);
        assert!(!d.started());
    }

    #[test]
    fn the_prescaler_scales_the_timeout() {
        // /256 with a reload of 1 is 2 * 256 = 512 LSI cycles.
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 6, 1);
        d.advance_to(511);
        assert_eq!(resets(&reset), 0);
        d.advance_to(512);
        assert_eq!(resets(&reset), 1);

        // "111: divider /256" is reserved and the table stops there, so a
        // seven behaves as a six rather than as a divider of 512.
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 7, 1);
        d.advance_to(512);
        assert_eq!(resets(&reset), 1);
    }

    #[test]
    fn reloading_with_aaaa_restarts_the_count() {
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 0, 0xff); // 4 * 256 = 1024 cycles
        d.advance_to(1000);
        assert_eq!(resets(&reset), 0);
        poke(&d, 0x00, KEY_RELOAD);
        assert_eq!(d.counter(), 0xff);
        d.advance_to(2000);
        assert_eq!(resets(&reset), 0, "the kick landed");
        d.advance_to(1000 + 1024);
        assert_eq!(resets(&reset), 1);
    }

    #[test]
    fn pr_and_rlr_are_writable_only_after_5555() {
        let d = iwdg();
        // The reset values, and a write that lands nowhere.
        assert_eq!(peek(&d, 0x04), 0);
        assert_eq!(peek(&d, 0x08), 0xfff);
        poke(&d, 0x04, 5);
        poke(&d, 0x08, 0x123);
        assert_eq!(peek(&d, 0x04), 0, "still locked");
        assert_eq!(peek(&d, 0x08), 0xfff);

        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x04, 5);
        poke(&d, 0x08, 0x123);
        assert_eq!(peek(&d, 0x04), 5);
        assert_eq!(peek(&d, 0x08), 0x123);

        // Any other key closes it again — including the kick, which is the
        // trap: a driver that reloads between two configuration writes loses
        // the second.
        poke(&d, 0x00, KEY_RELOAD);
        poke(&d, 0x08, 0x456);
        assert_eq!(peek(&d, 0x08), 0x123, "the kick re-locked it");
    }

    #[test]
    fn kr_is_write_only() {
        let d = iwdg();
        poke(&d, 0x00, KEY_START);
        assert_eq!(peek(&d, 0x00), 0, "a read of KR returns zero");
        assert!(d.started(), "even though the write took effect");
    }

    #[test]
    fn a_started_watchdog_cannot_be_stopped_by_any_key() {
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 0, 0x0f); // 4 * 16 = 64 cycles
        for key in [0u32, 0x5555, 0x1234, 0xffff, 0xcccc] {
            poke(&d, 0x00, key);
        }
        assert!(d.started());
        // Starting an already-started watchdog is not a reload either, so the
        // original deadline stands.
        d.advance_to(64);
        assert_eq!(resets(&reset), 1);
    }

    #[test]
    fn the_update_flags_stay_set_for_five_lsi_cycles() {
        let d = iwdg();
        poke(&d, 0x00, KEY_UNLOCK);
        assert_eq!(peek(&d, 0x0c), 0, "SR is clear to begin with");
        poke(&d, 0x04, 3);
        assert_eq!(peek(&d, 0x0c), SR_PVU);
        poke(&d, 0x08, 0x20);
        assert_eq!(peek(&d, 0x0c), SR_PVU | SR_RVU);
        d.advance_to(UPDATE_CYCLES - 1);
        assert_eq!(peek(&d, 0x0c), SR_PVU | SR_RVU, "still crossing");
        d.advance_to(UPDATE_CYCLES);
        assert_eq!(peek(&d, 0x0c), 0);
        // SR is read-only.
        poke(&d, 0x0c, 0xffff_ffff);
        assert_eq!(peek(&d, 0x0c), 0);
    }

    #[test]
    fn the_window_rejects_an_early_reload_with_a_reset() {
        let d = Iwdg::with_window(true);
        let reset = watch(&d);
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x04, 0); // /4
        poke(&d, 0x08, 0xff); // reload 255
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x10, 0x40); // window 64 — and this reloads too
        assert_eq!(peek(&d, 0x10), 0x40);
        poke(&d, 0x00, KEY_START);
        assert_eq!(resets(&reset), 0);

        // A kick while the counter is still above the window is too early.
        poke(&d, 0x00, KEY_RELOAD);
        assert_eq!(resets(&reset), 1, "255 is above the window of 64");
        assert!(!d.started());
    }

    #[test]
    fn a_reload_inside_the_window_is_accepted() {
        let d = Iwdg::with_window(true);
        let reset = watch(&d);
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x08, 0xff);
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x10, 0x40);
        poke(&d, 0x00, KEY_START);
        // 4 cycles per decrement; 200 decrements takes it to 55, under 64.
        d.advance_to(4 * 200);
        assert_eq!(d.counter(), 0xff - 200);
        poke(&d, 0x00, KEY_RELOAD);
        assert_eq!(resets(&reset), 0);
        assert_eq!(d.counter(), 0xff);
    }

    #[test]
    fn a_part_without_a_window_has_no_winr() {
        let d = iwdg();
        assert!(!d.has_window());
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x10, 0x40);
        assert_eq!(peek(&d, 0x10), 0, "an F4 has no window register");
        // …and no reload came with the write, so a started watchdog's deadline
        // is untouched by it.
        let reset = watch(&d);
        arm(&d, 0, 0x0f);
        d.advance_to(32);
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x10, 0x40);
        d.advance_to(64);
        assert_eq!(resets(&reset), 1);
    }

    #[test]
    fn a_frozen_counter_does_not_advance() {
        let d = iwdg();
        let reset = watch(&d);
        arm(&d, 0, 0x0f); // 64 cycles
        d.advance_to(32);
        assert_eq!(d.counter(), 7);
        d.set_frozen(true);
        d.advance_to(10_000);
        assert_eq!(resets(&reset), 0, "the debugger has it held");
        assert_eq!(d.counter(), 7);
        assert_eq!(
            Device::next_event_tick(&d),
            None,
            "nothing is coming while it is held"
        );
        d.set_frozen(false);
        d.advance_to(10_032);
        assert_eq!(resets(&reset), 1, "and it picks up where it stopped");
    }

    #[test]
    fn the_freeze_pin_is_what_the_debug_unit_drives() {
        let d = iwdg();
        let src = WireId::new(1);
        let pin = Device::sink(&d, "freeze", &[src]).expect("freeze");
        // A net holds its sinks weakly, so the device has to own this one.
        let weak = Arc::downgrade(&pin.sink);
        drop(pin);
        let alive = weak.upgrade().expect("the watchdog still owns it");
        arm(&d, 0, 0x0f);
        alive.set_level(src, 0, Level::High);
        d.advance_to(10_000);
        assert!(d.started(), "held");
        alive.set_level(src, 0, Level::Low);
        d.advance_to(10_064);
        assert!(!d.started());
        assert!(Device::sink(&d, "halt", &[src]).is_none());
    }

    #[test]
    fn the_next_event_is_the_reset_or_the_soonest_flag() {
        let d = iwdg();
        assert_eq!(Device::next_event_tick(&d), None, "stopped, nothing to do");
        poke(&d, 0x00, KEY_UNLOCK);
        poke(&d, 0x04, 0);
        assert_eq!(
            Device::next_event_tick(&d),
            Some(UPDATE_CYCLES),
            "PVU has to clear"
        );
        d.advance_to(UPDATE_CYCLES);
        poke(&d, 0x08, 0x0f);
        d.advance_to(2 * UPDATE_CYCLES);
        poke(&d, 0x00, KEY_START);
        assert_eq!(Device::next_event_tick(&d), Some(2 * UPDATE_CYCLES + 64));
        assert!(Device::is_lazy(&d));
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_advances_nothing() {
        let d = iwdg();
        assert_eq!(
            d.regs
                .write(0x00, &KEY_START.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(!d.started(), "a peek did not arm it");
        arm(&d, 0, 0x0f);
        let mut word = [0u8; 4];
        d.regs
            .read(0x08, &mut word, MemAttrs::DEBUG)
            .expect("reading RLR is free");
        assert_eq!(u32::from_le_bytes(word), 0x0f);
        assert_eq!(d.tick(), 0, "and it did not advance time to answer");
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let d = iwdg();
        let mut byte = [0u8; 1];
        assert_eq!(
            d.regs.read(0x08, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            d.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn a_reset_stops_it_and_restores_the_reset_values_but_not_the_tick() {
        let d = iwdg();
        arm(&d, 5, 0x123);
        d.advance_to(500);
        Device::reset(&d, ResetKind::Cold);
        assert!(!d.started());
        assert_eq!(peek(&d, 0x04), 0);
        assert_eq!(peek(&d, 0x08), 0xfff);
        assert_eq!(d.tick(), 500, "the domain did not rewind");
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state_and_keeps_the_time_left() {
        let saved = Iwdg::with_window(true);
        poke(&saved, 0x00, KEY_UNLOCK);
        poke(&saved, 0x04, 2); // /16
        poke(&saved, 0x08, 0x0ff);
        poke(&saved, 0x00, KEY_UNLOCK);
        poke(&saved, 0x10, 0x010);
        poke(&saved, 0x00, KEY_START);
        saved.advance_to(16 * 200 + 7);
        let left = saved.counter();
        assert_eq!(left, 0xff - 200);

        let mut shape = MachineShape::new();
        shape.add_device("iwdg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("iwdg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = Iwdg::with_window(true);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("iwdg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let offsets = [0x00, 0x04, 0x08, 0x0c, 0x10];
        let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
        let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
        assert_eq!(before, after);
        assert_eq!(restored.counter(), left);
        assert_eq!(restored.tick(), saved.tick());
        assert_eq!(
            Device::next_event_tick(&restored),
            Device::next_event_tick(&saved),
            "a snapshot with 3 ms left has 3 ms left"
        );

        // And it expires at the same instant the original would have.
        let reset_saved = watch(&saved);
        let reset_restored = watch(&restored);
        let deadline = saved.tick() + 16 * (u64::from(left) + 1) - 7;
        saved.advance_to(deadline);
        restored.advance_to(deadline);
        assert_eq!(resets(&reset_saved), 1);
        assert_eq!(resets(&reset_restored), 1);
    }

    #[test]
    fn a_corrupt_snapshot_is_refused() {
        let saved = iwdg();
        {
            let mut state = saved.regs.state.lock();
            state.counter = 0x1234; // past twelve bits
        }
        let mut shape = MachineShape::new();
        shape.add_device("iwdg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("iwdg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("iwdg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&iwdg(), &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("window", Value::from(true));
        assert!(Iwdg::new(&props).unwrap().has_window());
        assert!(!Iwdg::new(&Props::new()).unwrap().has_window());
        assert!(Iwdg::new(&Props::new().with("windowed", Value::from(true))).is_err());
    }

    #[test]
    fn the_class_is_registrable_and_constructs_through_the_registry() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);
    }

    #[test]
    fn the_schema_and_the_device_agree_about_pins_and_regions() {
        let d = iwdg();
        let schema = schema();
        assert!(schema.port_named("reset").is_some());
        assert!(schema.port_named("freeze").is_some());
        assert!(schema.port_named("irq").is_none());
        assert!(Device::sink(&d, "freeze", &[WireId::new(1)]).is_some());
        assert!(Device::connect(&d, "irq", dummy_source()).is_err());
        assert!(Device::region(&d, "").is_some());
        assert!(Device::region(&d, "regs").is_some());
        assert!(Device::region(&d, "counter").is_none());
    }

    /// A wire with one source, so a pin has something to drive.
    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }
}
