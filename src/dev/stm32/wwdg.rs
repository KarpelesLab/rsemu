//! The STM32 window watchdog.
//!
//! One class, `st.wwdg`. Where [`st.iwdg`](super::iwdg) asks only *has the
//! program stopped*, this one asks *is the program still keeping time*: a kick
//! that arrives **too early** resets the machine just as surely as one that
//! never arrives. That catches a failure a plain timeout cannot — a loop that
//! has started spinning twice as fast as it should because it is no longer
//! waiting on the thing it is supposed to wait on.
//!
//! It is also not independent: it runs on PCLK1, so stopping the APB clock
//! stops it, and a firmware that reconfigures the bus prescaler moves its
//! timeout underneath itself.
//!
//! # The registers
//!
//! Three of them, at base `0x4000_2C00` (RM0090 §20.4, RM0351 §35.4):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `CR` | `T[6:0]`, the down-counter, and `WDGA` at 7: **set once, cleared only by reset** |
//! | `0x04` | `CFR` | `W[6:0]`, the window; `WDGTB`, the prescaler; `EWI` at 9 |
//! | `0x08` | `SR` | `EWIF` at 0: the early-wakeup flag, **cleared by writing a zero** |
//!
//! `WDGTB` moved between families — bits `[8:7]` on an F4, `[12:11]` on an L4
//! — so `wdgtb-shift` says where it is on this part. Everything else about the
//! block is the same, which is why it is one class and a property rather than
//! two classes.
//!
//! # The three ways it fires
//!
//! * **Too late.** The counter reaches `0x40` and the next decrement clears
//!   `T6`, so `T` goes `0x40` → `0x3F` and the machine resets.
//! * **Too early.** `CR` is written while the counter is still above `W`. `W`
//!   resets to `0x7F`, which no counter value can exceed, so the window is
//!   disabled until firmware narrows it.
//! * **Armed below the window.** Activating with `T6` already clear resets at
//!   once, because a counter below `0x40` is a counter that has already run
//!   out.
//!
//! One decrement before the reset, at `T == 0x40`, `EWI` raises the early
//! wakeup interrupt on the `ewi` pin — the last chance a firmware has to save
//! anything before the board goes round again. It is a level, held until
//! software clears `EWIF`, which is what an NVIC wants.
//!
//! # Time
//!
//! Lazily advanced on its own clock domain, and the machine file gives it the
//! APB1 clock:
//!
//! ```text
//! object wwdg "st.wwdg" { clock = hse * 21 / 4 }   # PCLK1 = SYSCLK/4
//! ```
//!
//! One tick of that domain is one PCLK1 cycle and one decrement costs
//! `4096 << WDGTB` of them, which is the manual's timing formula written out.
//! The counter's next interesting instant is published as
//! [`next_event_tick`](crate::core::Device::next_event_tick); there is no loop
//! and no host clock anywhere in this file.
//!
//! # The reset
//!
//! Expiry drives a **pulse on the `reset` pin**, from outside the state lock —
//! see [`st.iwdg`](super::iwdg)'s module documentation for why that is the
//! re-entrancy contract rather than a detail. `RCC_CSR.WWDGRSTF` is RCC's to
//! latch off that same pin.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §20 "Window watchdog (WWDG)" and ST **RM0351** rev 9
//! §35 for the moved `WDGTB` field. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

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
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.wwdg";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes the three registers occupy.
pub const REGISTER_BYTES: u64 = 0x0c;

/// `CR.WDGA`: the activation bit.
const CR_WDGA: u32 = 1 << 7;

/// The down-counter's field, `T[6:0]`.
const T_MASK: u32 = 0x7f;

/// `T6`, the bit whose fall is the reset.
const T6: u32 = 1 << 6;

/// `CFR.EWI`: enable the early wakeup interrupt.
const CFR_EWI: u32 = 1 << 9;

/// `SR.EWIF`: the early wakeup flag.
const SR_EWIF: u32 = 1 << 0;

/// How many PCLK1 cycles one decrement costs at `WDGTB = 0`.
///
/// "the timer clock is PCLK1 divided by 4096" (RM0090 §20.3), and `WDGTB`
/// divides it again by one, two, four or eight.
const BASE_DIVIDER: u64 = 4096;

/// Where `WDGTB` sits on an F4: bits `[8:7]`.
pub const F4_WDGTB_SHIFT: u32 = 7;

/// Where it sits on an L4: bits `[12:11]`.
pub const L4_WDGTB_SHIFT: u32 = 11;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the counter's position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// `CR.WDGA`. One way: only a reset clears it.
    active: bool,
    /// `CR.T[6:0]`.
    t: u32,
    /// `CFR.W[6:0]`.
    w: u32,
    /// `CFR.WDGTB`, two bits wherever the part puts them.
    wdgtb: u32,
    /// `CFR.EWI`. Set-only, like `WDGA` (RM0090 §20.4.2).
    ewi: bool,
    /// `SR.EWIF`.
    ewif: bool,
    /// PCLK1 cycles counted towards the next decrement.
    prescale: u64,
    /// Where the device has advanced to, in its own clock domain.
    tick: u64,
}

impl Default for State {
    fn default() -> State {
        State {
            active: false,
            // "Reset value: 0x0000 007F" for both `CR` and `CFR`.
            t: T_MASK,
            w: T_MASK,
            wdgtb: 0,
            ewi: false,
            ewif: false,
            prescale: 0,
            tick: 0,
        }
    }
}

/// What a step or a register write asks the caller to do outside the lock.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Outcome {
    /// The machine is being reset.
    reset: bool,
    /// `EWIF` moved, so the interrupt line needs republishing.
    irq: bool,
}

impl State {
    /// How many PCLK1 cycles one decrement costs.
    fn divider(&self) -> u64 {
        BASE_DIVIDER << self.wdgtb
    }

    /// How many decrements until the next thing the guest can observe.
    ///
    /// The early-wakeup interrupt if one is armed and still ahead, otherwise
    /// the reset. `None` when the watchdog is not running.
    fn decrements_to_event(&self) -> Option<u64> {
        if !self.active {
            return None;
        }
        let to_reset = u64::from(self.t.saturating_sub(T6 - 1));
        if self.ewi && !self.ewif && self.t > T6 {
            return Some(u64::from(self.t - T6));
        }
        Some(to_reset)
    }

    /// PCLK1 cycles until that event.
    fn next_event(&self) -> Option<u64> {
        let decrements = self.decrements_to_event()?;
        let divider = self.divider();
        Some((divider - self.prescale) + decrements.saturating_sub(1) * divider)
    }

    /// Advance `n` PCLK1 cycles.
    fn step(&mut self, n: u64) -> Outcome {
        let end = self.tick + n;
        let mut out = Outcome::default();
        if self.active {
            let divider = self.divider();
            let mut remaining = n;
            while remaining > 0 {
                let need = divider - self.prescale;
                if remaining < need {
                    self.prescale += remaining;
                    break;
                }
                remaining -= need;
                self.prescale = 0;
                let before = self.t;
                self.t = (self.t.wrapping_sub(1)) & T_MASK;
                // "A reset is generated when the 7-bit downcounter rolls over
                // from 0x40 to 0x3F" — the fall of T6 and nothing else.
                if before & T6 != 0 && self.t & T6 == 0 {
                    out.reset = true;
                    self.active = false;
                    break;
                }
                // "EWI: an interrupt occurs whenever the counter reaches the
                // value 0x40", one decrement before the reset.
                if self.ewi && self.t == T6 && !self.ewif {
                    self.ewif = true;
                    out.irq = true;
                }
            }
        }
        self.tick = end;
        out
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// Which bit `WDGTB` starts at on this part.
    wdgtb_shift: u32,
    /// The reset output, pulsed on expiry.
    reset_out: Mutex<Option<WireSource>>,
    /// The early-wakeup interrupt output. A level, held until `EWIF` clears.
    irq_out: Mutex<Option<WireSource>>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// [`State::tick`], republished on every change: the scheduler asks
    /// [`Device::current_tick`] with its slot held at
    /// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF), so that call may
    /// not take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next event, or [`u64::MAX`] for none.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("wdgtb_shift", &self.wdgtb_shift);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Republish what the lock-free lazy surface reads.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event() {
            Some(delta) => state.tick.saturating_add(delta.max(1)),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Pulse the reset line and republish the interrupt, as `outcome` asks.
    ///
    /// **Never called with the state lock held.** A reset reaches every device
    /// on the machine, this one included, and the interrupt reaches a core
    /// that may read straight back into these registers; both happen after the
    /// critical section (`CLAUDE.md`, "Concurrency").
    fn deliver(&self, outcome: Outcome) {
        if outcome.irq {
            self.refresh_irq();
        }
        if outcome.reset {
            let source = self.reset_out.lock().clone();
            if let Some(source) = source {
                source.pulse(Level::High);
            }
        }
    }

    /// Drive the interrupt line from whatever `EWIF` now says.
    fn refresh_irq(&self) {
        let level = Level::from_bool(self.state.lock().ewif);
        let source = self.irq_out.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Advance to `target` of the watchdog's own clock domain.
    ///
    /// One iteration per internal event, so the interrupt lands on the
    /// decrement that raised it rather than at the end of the catch-up.
    fn advance_to(&self, target: u64) {
        loop {
            let (reached, outcome) = {
                let mut state = self.state.lock();
                if target <= state.tick {
                    return;
                }
                let span = target - state.tick;
                let step = state.next_event().unwrap_or(span).clamp(1, span);
                let outcome = state.step(step);
                self.publish(&state);
                (state.tick >= target, outcome)
            };
            self.deliver(outcome);
            if reached {
                return;
            }
        }
    }

    /// Catch the watchdog up before an access is dispatched to it (§4.2).
    fn sync(&self, attrs: MemAttrs) {
        if attrs.debug {
            return;
        }
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let _ = handle.sync(AccessKind::Guest);
    }

    /// `CFR`'s `WDGTB` field, as a mask.
    fn wdgtb_mask(&self) -> u32 {
        0x3 << self.wdgtb_shift
    }

    /// Read one register.
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            0x00 => state.t | if state.active { CR_WDGA } else { 0 },
            0x04 => {
                state.w | (state.wdgtb << self.wdgtb_shift) | if state.ewi { CFR_EWI } else { 0 }
            }
            0x08 => u32::from(state.ewif),
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) -> Outcome {
        let mut state = self.state.lock();
        let mut out = Outcome::default();
        match offset {
            0x00 => {
                let was_active = state.active;
                let old_t = state.t;
                if value & CR_WDGA != 0 && !state.active {
                    // "WDGA: set by software and only cleared by hardware
                    // after a reset." The prescaler starts here, so the first
                    // decrement is a full period after activation.
                    state.active = true;
                    state.prescale = 0;
                }
                state.t = value & T_MASK;
                // "A reset is generated if the downcounter is refreshed before
                // it has reached the window value."
                if was_active && old_t > state.w {
                    out.reset = true;
                }
                // Arming — or refreshing — with T6 already clear is a counter
                // that has run out before it started.
                if state.active && state.t & T6 == 0 {
                    out.reset = true;
                }
                if out.reset {
                    state.active = false;
                }
            }
            0x04 => {
                state.w = value & T_MASK;
                state.wdgtb = (value & self.wdgtb_mask()) >> self.wdgtb_shift;
                // `EWI` is set-only: "this bit is set by software and cleared
                // by hardware after a reset."
                state.ewi |= value & CFR_EWI != 0;
            }
            // "EWIF … must be cleared by software by writing 0."
            0x08 if value & SR_EWIF == 0 && state.ewif => {
                state.ewif = false;
                out.irq = true;
            }
            _ => {}
        }
        self.publish(&state);
        out
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        self.sync(attrs);
        // No register here clears on a read, so a debug read is the same read.
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would refresh a counter the guest was
            // about to be reset by, or reset the machine outright. There is no
            // harmless version (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        let outcome = self.write_register(offset & !3, value);
        self.deliver(outcome);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 window watchdog.
#[derive(Debug)]
pub struct Wwdg {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Wwdg {
    /// Validate `props` and build the watchdog.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Wwdg> {
        let mut r = props.reader();
        let shift = r.or_range(
            "wdgtb-shift",
            u64::from(F4_WDGTB_SHIFT),
            u64::from(F4_WDGTB_SHIFT)..=u64::from(L4_WDGTB_SHIFT),
        )? as u32;
        r.touch("clock");
        r.finish()?;
        Ok(Wwdg::with_shift(shift))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_shift(wdgtb_shift: u32) -> Wwdg {
        let wdgtb_shift = wdgtb_shift.clamp(F4_WDGTB_SHIFT, L4_WDGTB_SHIFT);
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            wdgtb_shift,
            reset_out: Mutex::with_rank(LockRank::WIRE, None),
            irq_out: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region = Arc::new(Region::io(
            "wwdg",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Wwdg { regs, region }
    }

    /// Which bit `WDGTB` starts at on this part.
    #[must_use]
    pub fn wdgtb_shift(&self) -> u32 {
        self.regs.wdgtb_shift
    }

    /// Whether `WDGA` has been set.
    #[must_use]
    pub fn active(&self) -> bool {
        self.regs.state.lock().active
    }

    /// What the down-counter is at.
    #[must_use]
    pub fn counter(&self) -> u32 {
        self.regs.state.lock().t
    }

    /// Whether the early-wakeup flag is set.
    #[must_use]
    pub fn early_wakeup(&self) -> bool {
        self.regs.state.lock().ewif
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
}

impl Device for Wwdg {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the reset and interrupt lines.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds, and this is the only thing that clears `WDGA`. The tick
        // survives for the reason `pc.pit`'s reset gives: it is this device's
        // cursor in its clock domain, not architectural state, and the domain
        // does not rewind because a chip on it was reset.
        {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            *state = State::default();
            state.tick = tick;
            self.regs.publish(&state);
        }
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_bool(state.active)?;
        w.write_u32(state.t)?;
        w.write_u32(state.w)?;
        w.write_u32(state.wdgtb)?;
        w.write_bool(state.ewi)?;
        w.write_bool(state.ewif)?;
        w.write_u64(state.prescale)?;
        // The cursor in the clock domain; without it the scheduler's domain
        // and this counter would disagree by however long the machine had run.
        w.write_u64(state.tick)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            active: r.read_bool()?,
            t: r.read_u32()?,
            w: r.read_u32()?,
            wdgtb: r.read_u32()?,
            ewi: r.read_bool()?,
            ewif: r.read_bool()?,
            prescale: r.read_u64()?,
            tick: r.read_u64()?,
        };
        if state.t > T_MASK || state.w > T_MASK || state.wdgtb > 3 {
            return Err(Error::State(format!(
                "snapshot has a WWDG with T={}, W={}, WDGTB={}",
                state.t, state.w, state.wdgtb
            )));
        }
        if state.prescale >= state.divider() {
            return Err(Error::State(format!(
                "snapshot has a WWDG {} cycle(s) into a divide-by-{}",
                state.prescale,
                state.divider()
            )));
        }
        {
            let mut live = self.regs.state.lock();
            *live = state;
            self.regs.publish(&live);
        }
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            "reset" => *self.regs.reset_out.lock() = Some(source),
            "ewi" => *self.regs.irq_out.lock() = Some(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: String::from("a WWDG drives `reset` and `ewi`"),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        // `ewi` idles low out of reset, which a fresh net already is, but a
        // machine wired after a snapshot load has to be told about a flag that
        // survived it.
        if port == "ewi" {
            self.regs.refresh_irq();
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
}

impl Instance for Wwdg {}

/// The `st.wwdg` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 window watchdog: the PCLK1 down-counter, its window and its early wakeup",
    properties: &[PropertySpec {
        name: "wdgtb-shift",
        kind: ValueKind::Uint,
        required: false,
        summary: "which bit CFR.WDGTB starts at: 7 on an F4 (the default), 11 on an L4",
    }],
    construct: |props| Ok(Box::new(Wwdg::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Wwdg::new(props)?)))
}

/// What the validator should know about `st.wwdg`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("wdgtb-shift", ValueKind::Uint)
                .range(u64::from(F4_WDGTB_SHIFT), u64::from(L4_WDGTB_SHIFT)),
        )
        .region("")
        .region("regs")
        .port("reset", PortDir::Out)
        .port("ewi", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU32;
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    /// An F4's: `WDGTB` at bits [8:7].
    fn wwdg() -> Wwdg {
        Wwdg::with_shift(F4_WDGTB_SHIFT)
    }

    fn peek(d: &Wwdg, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        d.regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(d: &Wwdg, offset: u64, value: u32) {
        d.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
            if level.is_high() {
                self.edges.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn watch(d: &Wwdg, port: &str) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(d, port, WireSource::new(wire, id)).expect("a pin this device drives");
        probe
    }

    fn edges(p: &Probe) -> u32 {
        p.edges.load(Ordering::Relaxed)
    }

    fn high(p: &Probe) -> bool {
        p.level.load(Ordering::Relaxed) == 1
    }

    #[test]
    fn the_reset_values_are_the_manual_s() {
        let d = wwdg();
        assert_eq!(peek(&d, 0x00), 0x7f, "CR");
        assert_eq!(peek(&d, 0x04), 0x7f, "CFR");
        assert_eq!(peek(&d, 0x08), 0, "SR");
        assert!(!d.active());
    }

    #[test]
    fn wwdg_resets_when_t_passes_0x3f_and_ewi_fires_one_tick_before() {
        let d = wwdg();
        let reset = watch(&d, "reset");
        let irq = watch(&d, "ewi");
        poke(&d, 0x04, CFR_EWI | 0x7f); // EWI on, window wide open
        poke(&d, 0x00, CR_WDGA | 0x43); // arm with T = 0x43
        // Three decrements take it to 0x40; the interrupt lands there.
        d.advance_to(3 * BASE_DIVIDER);
        assert_eq!(d.counter(), 0x40);
        assert!(d.early_wakeup());
        assert!(high(&irq), "EWIF is a level, not a pulse");
        assert_eq!(edges(&reset), 0, "one decrement still to go");

        d.advance_to(4 * BASE_DIVIDER);
        assert_eq!(edges(&reset), 1);
        assert_eq!(d.counter(), 0x3f);
        assert!(!d.active(), "a fired watchdog is a stopped watchdog");
        // Clearing EWIF is a write of *zero*; a write of one leaves it.
        poke(&d, 0x08, 1);
        assert!(d.early_wakeup());
        poke(&d, 0x08, 0);
        assert!(!d.early_wakeup());
        assert!(!high(&irq));
    }

    #[test]
    fn without_ewi_nothing_interrupts_and_the_reset_still_lands() {
        let d = wwdg();
        let reset = watch(&d, "reset");
        let irq = watch(&d, "ewi");
        poke(&d, 0x00, CR_WDGA | 0x41);
        d.advance_to(2 * BASE_DIVIDER);
        assert_eq!(edges(&reset), 1);
        assert_eq!(edges(&irq), 0);
        assert!(!d.early_wakeup());
    }

    #[test]
    fn wdgtb_scales_the_period() {
        // WDGTB = 2 is a further divide by four, so a decrement costs 16384
        // PCLK1 cycles.
        let d = wwdg();
        let reset = watch(&d, "reset");
        poke(&d, 0x04, (2 << F4_WDGTB_SHIFT) | 0x7f);
        assert_eq!(peek(&d, 0x04) >> F4_WDGTB_SHIFT & 3, 2);
        poke(&d, 0x00, CR_WDGA | 0x41);
        d.advance_to(2 * 4 * BASE_DIVIDER - 1);
        assert_eq!(edges(&reset), 0);
        d.advance_to(2 * 4 * BASE_DIVIDER);
        assert_eq!(edges(&reset), 1);
    }

    #[test]
    fn an_l4_puts_wdgtb_somewhere_else() {
        let d = Wwdg::with_shift(L4_WDGTB_SHIFT);
        assert_eq!(d.wdgtb_shift(), 11);
        poke(&d, 0x04, (3 << L4_WDGTB_SHIFT) | 0x7f);
        assert_eq!(peek(&d, 0x04), (3 << L4_WDGTB_SHIFT) | 0x7f);
        // And the F4's position is not a field here, so it reads back as zero.
        let d = Wwdg::with_shift(L4_WDGTB_SHIFT);
        poke(&d, 0x04, (3 << F4_WDGTB_SHIFT) | 0x7f);
        assert_eq!(peek(&d, 0x04), 0x7f);
    }

    #[test]
    fn refreshing_wwdg_while_t_is_above_w_resets_immediately() {
        let d = wwdg();
        let reset = watch(&d, "reset");
        poke(&d, 0x04, 0x50); // window at 0x50
        poke(&d, 0x00, CR_WDGA | 0x7f);
        assert_eq!(edges(&reset), 0, "arming is not a refresh");
        // 0x7f is above the window, so this kick is too early.
        poke(&d, 0x00, CR_WDGA | 0x7f);
        assert_eq!(edges(&reset), 1);
        assert!(!d.active());
    }

    #[test]
    fn a_refresh_inside_the_window_is_accepted() {
        let d = wwdg();
        let reset = watch(&d, "reset");
        poke(&d, 0x04, 0x50);
        poke(&d, 0x00, CR_WDGA | 0x7f);
        // 0x7f down to 0x4f is 48 decrements, which is inside the window.
        d.advance_to(48 * BASE_DIVIDER);
        assert_eq!(d.counter(), 0x4f);
        poke(&d, 0x00, CR_WDGA | 0x7f);
        assert_eq!(edges(&reset), 0);
        assert_eq!(d.counter(), 0x7f);
        assert!(d.active());
    }

    #[test]
    fn the_window_is_wide_open_until_firmware_narrows_it() {
        // W resets to 0x7f, which no counter value exceeds, so every refresh
        // is in time however soon it comes.
        let d = wwdg();
        let reset = watch(&d, "reset");
        poke(&d, 0x00, CR_WDGA | 0x7f);
        for _ in 0..10 {
            poke(&d, 0x00, CR_WDGA | 0x7f);
        }
        assert_eq!(edges(&reset), 0);
    }

    #[test]
    fn arming_with_t6_clear_resets_at_once() {
        // A counter below 0x40 has already run out, so activating there is a
        // reset rather than a very short timeout.
        let d = wwdg();
        let reset = watch(&d, "reset");
        poke(&d, 0x00, CR_WDGA | 0x3f);
        assert_eq!(edges(&reset), 1);
        assert!(!d.active());
    }

    #[test]
    fn wdga_and_ewi_are_set_only() {
        let d = wwdg();
        poke(&d, 0x00, CR_WDGA | 0x7f);
        assert!(d.active());
        poke(&d, 0x00, 0x7f); // no WDGA in the value
        assert!(d.active(), "only a reset clears WDGA");
        assert_eq!(peek(&d, 0x00) & CR_WDGA, CR_WDGA);

        poke(&d, 0x04, CFR_EWI | 0x7f);
        poke(&d, 0x04, 0x7f);
        assert_eq!(peek(&d, 0x04) & CFR_EWI, CFR_EWI, "only a reset clears EWI");

        Device::reset(&d, ResetKind::Cold);
        assert!(!d.active());
        assert_eq!(peek(&d, 0x04) & CFR_EWI, 0);
    }

    #[test]
    fn a_stopped_watchdog_has_no_next_event() {
        let d = wwdg();
        assert_eq!(Device::next_event_tick(&d), None);
        poke(&d, 0x00, CR_WDGA | 0x42);
        // Three decrements to the reset, and no EWI armed.
        assert_eq!(Device::next_event_tick(&d), Some(3 * BASE_DIVIDER));
        assert!(Device::is_lazy(&d));
        // With EWI armed the interrupt is the nearer event.
        let d = wwdg();
        poke(&d, 0x04, CFR_EWI | 0x7f);
        poke(&d, 0x00, CR_WDGA | 0x42);
        assert_eq!(Device::next_event_tick(&d), Some(2 * BASE_DIVIDER));
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_advances_nothing() {
        let d = wwdg();
        assert_eq!(
            d.regs
                .write(0x00, &(CR_WDGA | 0x7f).to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(!d.active(), "a peek did not arm it");
        poke(&d, 0x00, CR_WDGA | 0x7f);
        let mut word = [0u8; 4];
        d.regs
            .read(0x00, &mut word, MemAttrs::DEBUG)
            .expect("reading CR is free");
        assert_eq!(u32::from_le_bytes(word), CR_WDGA | 0x7f);
        assert_eq!(d.tick(), 0, "and it did not advance time to answer");
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let d = wwdg();
        let mut byte = [0u8; 1];
        assert_eq!(
            d.regs.read(0x00, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            d.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state_and_keeps_the_time_left() {
        let saved = wwdg();
        poke(&saved, 0x04, CFR_EWI | (1 << F4_WDGTB_SHIFT) | 0x60);
        poke(&saved, 0x00, CR_WDGA | 0x7f);
        // Twenty decrements of 8192 cycles each, plus a bit.
        saved.advance_to(20 * 2 * BASE_DIVIDER + 11);
        assert_eq!(saved.counter(), 0x7f - 20);

        let mut shape = MachineShape::new();
        shape.add_device("wwdg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("wwdg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = wwdg();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("wwdg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let offsets = [0x00, 0x04, 0x08];
        let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
        let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
        assert_eq!(before, after);
        assert_eq!(restored.tick(), saved.tick());
        assert_eq!(
            Device::next_event_tick(&restored),
            Device::next_event_tick(&saved),
            "a snapshot with 3 ms left has 3 ms left"
        );

        // And both fire at the same instant.
        let reset_saved = watch(&saved, "reset");
        let reset_restored = watch(&restored, "reset");
        let deadline = Device::next_event_tick(&saved).unwrap() + 64 * 2 * BASE_DIVIDER;
        saved.advance_to(deadline);
        restored.advance_to(deadline);
        assert_eq!(edges(&reset_saved), 1);
        assert_eq!(edges(&reset_restored), 1);
    }

    #[test]
    fn a_corrupt_snapshot_is_refused() {
        let saved = wwdg();
        {
            let mut state = saved.regs.state.lock();
            state.t = 0xff; // past seven bits
        }
        let mut shape = MachineShape::new();
        shape.add_device("wwdg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("wwdg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("wwdg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&wwdg(), &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("wdgtb-shift", Value::from(11u64));
        assert_eq!(Wwdg::new(&props).unwrap().wdgtb_shift(), 11);
        assert_eq!(Wwdg::new(&Props::new()).unwrap().wdgtb_shift(), 7);
        assert!(Wwdg::new(&Props::new().with("wdgtb", Value::from(11u64))).is_err());
        assert!(Wwdg::new(&Props::new().with("wdgtb-shift", Value::from(3u64))).is_err());
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
        let d = wwdg();
        let schema = schema();
        assert!(schema.port_named("reset").is_some());
        assert!(schema.port_named("ewi").is_some());
        assert!(schema.port_named("freeze").is_none());
        assert!(Device::connect(&d, "freeze", dummy_source()).is_err());
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
