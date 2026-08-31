//! The W65C22 VIA: two eight-bit ports and two sixteen-bit timers.
//!
//! # Sources
//!
//! Everything below is from the **W65C22 Versatile Interface Adapter (VIA) data
//! sheet, Western Design Center** (`www.WDC65xx.com`), by table number:
//!
//! * Table 2-1, "Register Selection" — the sixteen addresses and what read and
//!   write mean at each.
//! * Tables 2-6 and 2-7 — what a read or a write of each T1 address does to the
//!   counter, the latches and `IFR6`.
//! * Table 2-8, "Auxiliary Control Register" — the T1 and T2 mode bits.
//! * Table 2-9 — the T2 addresses, and `IFR5`.
//! * Tables 2-11 and 2-12 — the interrupt flag and enable registers, including
//!   the set/clear rule that makes `IER` bit 7 a direction rather than a flag.
//! * §3.9, "Reset (RESB)" — "Reset clears all internal registers (except T1 and
//!   T2 counters and latches, and the SR)".
//!
//! No emulator was consulted (`ROADMAP.md` §1).
//!
//! # The register map
//!
//! ```text
//!   $0 ORB/IRB    $4 T1C-L    $8 T2C-L    $C PCR
//!   $1 ORA/IRA    $5 T1C-H    $9 T2C-H    $D IFR
//!   $2 DDRB       $6 T1L-L    $A SR       $E IER
//!   $3 DDRA       $7 T1L-H    $B ACR      $F ORA/IRA, no handshake
//! ```
//!
//! # What is modelled
//!
//! * **Both ports**, with their data-direction registers. A pin configured as
//!   an output reads back what the output register holds; a pin configured as
//!   an input reads the level something outside is driving, which is
//!   [`Via::set_port_a`] / [`Via::set_port_b`] and is `0` until somebody sets
//!   it. Port B reads the *register* for output pins and port A reads the
//!   *pins* for all of them, which is the one place the two halves genuinely
//!   differ.
//! * **Both timers.** T1 in one-shot and free-run mode and T2 in its timed
//!   mode count φ2 down to zero, set `IFR6` / `IFR5`, and are cleared by the
//!   register accesses the data sheet names — reading `T1C-L` or writing
//!   `T1L-H` for the first, reading `T2C-L` or writing `T2C-H` for the second.
//! * **`IFR` and `IER`**, with bit 7 of each doing its own thing: on the flag
//!   register it is the wired-OR of every enabled flag and cannot be written,
//!   and on the enable register a write of it is the difference between
//!   setting and clearing the bits below.
//! * **The `IRQB` output**, driven from exactly the expression the data sheet
//!   gives for `IFR7`.
//!
//! The timers are **lazily advanced** (`ROADMAP.md` §4.2): they hold their own
//! φ2 tick and whoever reads a counter catches them up first, so `LDA T1C-L`
//! sees the count at the cycle it happened on rather than at the end of the
//! scheduler's quantum. [`Device::next_event_tick`] names the tick a timeout
//! falls on, so an enabled interrupt reaches the CPU on that cycle.
//!
//! # What is absent, and why
//!
//! Named rather than half-done, so that a program that needs one of these fails
//! visibly instead of subtly:
//!
//! * **`CA1`, `CA2`, `CB1` and `CB2` as pins** — the handshake modes, the input
//!   latching `ACR` bits 0 and 1 select, and `IFR` bits 0, 1, 3 and 4. Nothing
//!   on a bare board drives them, and the `PCR` is stored and read back so the
//!   software that configures them still works.
//! * **The shift register.** `SR` is stored and read back; no shifting happens
//!   and `IFR2` is never set. `ACR` bits 4-2 are stored.
//! * **`PB7` as a timer output** (`ACR` bit 7) and **`PB6` as T2's pulse input**
//!   (`ACR` bit 5). Both are pin behaviour, and both wait on the pins above. In
//!   T2's pulse-counting mode the counter therefore does not run at all, which
//!   is not a simplification: nothing is driving `PB6`, so nothing would count.
//!
//! # On this board
//!
//! Ben Eater's build brings both ports out to headers and drives an HD44780
//! character LCD from them. Nothing here models the LCD; the ports are visible
//! and settable so that when one is written it has something to attach to.

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "wdc.w65c22";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the sixteen registers occupy.
///
/// The chip decodes RS0-RS3 only, so a board that gives it an eight-kilobyte
/// window sees these sixteen repeated all through it — which is what
/// `machines/beneater-6502.machine` writes as `mirror(via)`.
pub const REGISTER_COUNT: u64 = 16;

/// The name of the interrupt output pin.
pub const IRQ_PIN: &str = "irq";

/// `IFR` bit 6: timer 1 timed out.
const IFR_T1: u8 = 0x40;
/// `IFR` bit 5: timer 2 timed out.
const IFR_T2: u8 = 0x20;
/// `IFR` bit 7 and `IER` bit 7, which are not flags.
const IFR_ANY: u8 = 0x80;

/// `ACR` bit 6: timer 1 runs continuously instead of once.
const ACR_T1_FREE_RUN: u8 = 0x40;
/// `ACR` bit 5: timer 2 counts pulses on PB6 instead of φ2.
const ACR_T2_PULSE: u8 = 0x20;

/// "Nothing scheduled", as [`Shared::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

/// The W65C22 as a device.
///
/// Two-phase like every device (`ROADMAP.md` §4.4): [`Via::new`] validates
/// properties and builds the register block; [`Device::realize`] does nothing,
/// because a `map` statement places the region.
#[derive(Debug)]
pub struct Via {
    shared: Arc<Shared>,
    region: RegionRef,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// φ2 ticks simulated, published for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The tick the next timeout falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// Everything the guest can see or change.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    /// φ2 ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    ora: u8,
    orb: u8,
    ddra: u8,
    ddrb: u8,
    /// What something outside is driving onto port A's pins.
    pa_in: u8,
    /// And port B's.
    pb_in: u8,
    /// Timer 1's counter and its two latches, as one word each.
    t1_counter: u16,
    t1_latch: u16,
    /// Whether T1 has already set its flag since the last load. Only meaningful
    /// in one-shot mode, where a second timeout raises nothing.
    t1_fired: bool,
    t2_counter: u16,
    /// T2 has a low-order latch only; the high byte goes straight to the
    /// counter (data sheet, Table 2-9).
    t2_latch_low: u8,
    t2_fired: bool,
    sr: u8,
    acr: u8,
    pcr: u8,
    ifr: u8,
    ier: u8,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Via {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`crate::core::Error::Property`] if a property this class does not know
    /// was given. It takes none.
    pub fn new(props: &Props) -> Result<Via> {
        props.reader().finish()?;
        Ok(Via::bare())
    }

    /// One with no properties to read.
    #[must_use]
    pub fn bare() -> Via {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        shared.publish(&shared.state.lock());
        let port = Arc::new(ViaPort {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("via", REGISTER_COUNT, port as Arc<dyn MemOps>));
        Via { shared, region }
    }

    /// Drive port A's pins from outside.
    ///
    /// Pins the guest has made outputs ignore this; the rest read it back at
    /// `$1` and `$F`.
    pub fn set_port_a(&self, level: u8) {
        self.shared.state.lock().pa_in = level;
    }

    /// Drive port B's pins from outside.
    pub fn set_port_b(&self, level: u8) {
        self.shared.state.lock().pb_in = level;
    }

    /// What port A's pins are at: the output register where the direction
    /// register says output, and the driven level everywhere else.
    #[must_use]
    pub fn port_a(&self) -> u8 {
        let state = self.shared.state.lock();
        (state.ora & state.ddra) | (state.pa_in & !state.ddra)
    }

    /// The same for port B.
    #[must_use]
    pub fn port_b(&self) -> u8 {
        let state = self.shared.state.lock();
        (state.orb & state.ddrb) | (state.pb_in & !state.ddrb)
    }

    /// Timer 1's counter, without disturbing its flag.
    #[must_use]
    pub fn timer1(&self) -> u16 {
        self.shared.state.lock().t1_counter
    }

    /// Timer 2's counter, without disturbing its flag.
    #[must_use]
    pub fn timer2(&self) -> u16 {
        self.shared.state.lock().t2_counter
    }

    /// The interrupt flag register as software would read it.
    #[must_use]
    pub fn ifr(&self) -> u8 {
        Shared::visible_ifr(&self.shared.state.lock())
    }

    /// φ2 ticks the timers have counted.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
        self.shared.refresh_irq();
    }

    /// Connect the catch-up handle the register block syncs through (§4.2).
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// The level the interrupt output is driving; high is "requesting".
    ///
    /// The pin on the chip is `IRQB` and is active low; inverting it is a
    /// `wire.not` device's job when one exists (`ROADMAP.md` §4.3).
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Shared::level(&self.shared.state.lock())
    }

    /// Run the timers until `target` φ2 ticks have passed in total.
    ///
    /// The catch-up entry point. Running backwards is a no-op, not an error.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event
            .store(State::next_event(state), Ordering::Relaxed);
    }

    /// The IRQ output's level, from the data sheet's own expression for `IFR7`.
    fn level(state: &State) -> Level {
        if state.ifr & state.ier & !IFR_ANY != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// `IFR` as it reads: the flags, plus bit 7 if any enabled one is set.
    fn visible_ifr(state: &State) -> u8 {
        let mut value = state.ifr;
        if Shared::level(state) == Level::High {
            value |= IFR_ANY;
        }
        value
    }

    /// Drive the interrupt pin to whatever the flags now say.
    ///
    /// Called with no lock held: the re-entrancy contract in `core::device` is
    /// that outward calls happen after the critical section, never inside it.
    fn refresh_irq(&self) {
        let level = Shared::level(&self.state.lock());
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Bring the timers up to date before an access.
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
        // the stack, which would take the VIA reading its own registers through
        // its own bus. The access still has to be answered, and answering it
        // from where the timers stand is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    fn advance_to(&self, target: u64) {
        let moved = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            let elapsed = target - state.ticks;
            let before = Shared::level(&state);
            state.run(elapsed);
            state.ticks = target;
            self.publish(&state);
            before != Shared::level(&state)
        };
        if moved {
            self.refresh_irq();
        }
    }
}

impl State {
    /// Count both timers down by `elapsed` φ2 ticks, setting flags as they
    /// pass zero.
    fn run(&mut self, elapsed: u64) {
        // -- timer 1 --------------------------------------------------------
        if self.acr & ACR_T1_FREE_RUN != 0 {
            // Free-run: the flag is set and the counter reloaded from the
            // latches at every timeout (data sheet §2.7), so a long budget can
            // contain many. Only whether *any* happened matters to the flag.
            let period = u64::from(self.t1_latch) + 1;
            let first = u64::from(self.t1_counter) + 1;
            if elapsed < first {
                self.t1_counter -= elapsed as u16;
            } else {
                let after = elapsed - first;
                self.t1_counter = self.t1_latch - (after % period) as u16;
                self.ifr |= IFR_T1;
                self.t1_fired = true;
            }
        } else {
            // One-shot: "Once set, IFR6 the T1 Interrupt Flag is reset only by
            // writing T1C-H or reading T1C-L" (§2.6). The counter keeps running
            // and rolling over; no further timeout raises anything.
            let timed_out = elapsed > u64::from(self.t1_counter);
            self.t1_counter = self.t1_counter.wrapping_sub((elapsed % 0x1_0000) as u16);
            if timed_out && !self.t1_fired {
                self.ifr |= IFR_T1;
                self.t1_fired = true;
            }
        }

        // -- timer 2 --------------------------------------------------------
        //
        // In pulse-counting mode nothing drives PB6, so nothing counts. That
        // is the hardware's answer as well as ours.
        if self.acr & ACR_T2_PULSE == 0 {
            let timed_out = elapsed > u64::from(self.t2_counter);
            self.t2_counter = self.t2_counter.wrapping_sub((elapsed % 0x1_0000) as u16);
            if timed_out && !self.t2_fired {
                self.ifr |= IFR_T2;
                self.t2_fired = true;
            }
        }
    }

    /// The absolute tick the next timeout falls on, or [`NO_EVENT`].
    ///
    /// Never equal to the current tick: a counter at zero times out on the
    /// next one, so every answer here is at least `ticks + 1`.
    fn next_event(&self) -> u64 {
        let mut next = NO_EVENT;
        if self.acr & ACR_T1_FREE_RUN != 0 || !self.t1_fired {
            next = next.min(self.ticks + u64::from(self.t1_counter) + 1);
        }
        if self.acr & ACR_T2_PULSE == 0 && !self.t2_fired {
            next = next.min(self.ticks + u64::from(self.t2_counter) + 1);
        }
        next
    }
}

/// The memory-mapped register block.
struct ViaPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for ViaPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ViaPort").finish_non_exhaustive()
    }
}

impl ViaPort {
    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        let mut state = self.shared.state.lock();
        match index {
            // $0 IRB: an output pin reads the *register*, not the pin. That is
            // the documented difference between the two ports.
            0x0 => (state.orb & state.ddrb) | (state.pb_in & !state.ddrb),
            // $1 and $F IRA: every pin reads the pin.
            0x1 | 0xf => (state.ora & state.ddra) | (state.pa_in & !state.ddra),
            0x2 => state.ddrb,
            0x3 => state.ddra,
            // $4 T1C-L: "T1 interrupt flag IFR6 is reset" (Table 2-6).
            0x4 => {
                if !debug {
                    state.ifr &= !IFR_T1;
                }
                state.t1_counter as u8
            }
            0x5 => (state.t1_counter >> 8) as u8,
            // $6 and $7 are the latches, and reading them clears nothing
            // (Table 2-7).
            0x6 => state.t1_latch as u8,
            0x7 => (state.t1_latch >> 8) as u8,
            // $8 T2C-L: "IFR5 is reset" (Table 2-9).
            0x8 => {
                if !debug {
                    state.ifr &= !IFR_T2;
                }
                state.t2_counter as u8
            }
            0x9 => (state.t2_counter >> 8) as u8,
            0xa => state.sr,
            0xb => state.acr,
            0xc => state.pcr,
            0xd => Shared::visible_ifr(&state),
            // $E: "If a read of this register is done, bit 7 will be Logic 1"
            // (Table 2-12, note 3).
            _ => state.ier | IFR_ANY,
        }
    }

    /// Write one register.
    fn write_register(&self, index: u8, value: u8) {
        let mut state = self.shared.state.lock();
        match index {
            0x0 => state.orb = value,
            0x1 | 0xf => state.ora = value,
            0x2 => state.ddrb = value,
            0x3 => state.ddra = value,
            // $4 and $6 both load the low-order latch and nothing else.
            0x4 | 0x6 => state.t1_latch = (state.t1_latch & 0xff00) | u16::from(value),
            // $5 T1C-H: "both high and low order latches are transferred into
            // T1 counter and this initiates countdown. T1 interrupt flag IFR6
            // is reset" (Table 2-6).
            0x5 => {
                state.t1_latch = (state.t1_latch & 0x00ff) | (u16::from(value) << 8);
                state.t1_counter = state.t1_latch;
                state.t1_fired = false;
                state.ifr &= !IFR_T1;
            }
            // $7 T1L-H: the latch, and the flag, but no transfer (Table 2-7).
            0x7 => {
                state.t1_latch = (state.t1_latch & 0x00ff) | (u16::from(value) << 8);
                state.ifr &= !IFR_T1;
            }
            0x8 => state.t2_latch_low = value,
            // $9 T2C-H: the high byte goes to the counter, the low byte comes
            // from the latch, and IFR5 is reset (Table 2-9).
            0x9 => {
                state.t2_counter = (u16::from(value) << 8) | u16::from(state.t2_latch_low);
                state.t2_fired = false;
                state.ifr &= !IFR_T2;
            }
            0xa => state.sr = value,
            0xb => state.acr = value,
            0xc => state.pcr = value,
            // $D: "individual flag bits may be cleared by writing a Logic 1
            // into the appropriate bit". Bit 7 is not a flag and is ignored.
            0xd => state.ifr &= !(value & !IFR_ANY),
            // $E: bit 7 says whether the ones below set or clear.
            _ => {
                if value & IFR_ANY != 0 {
                    state.ier |= value & !IFR_ANY;
                } else {
                    state.ier &= !(value & !IFR_ANY);
                }
            }
        }
        self.shared.publish(&state);
    }
}

impl MemOps for ViaPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // First, and outside every lock this device owns: a counter read has
        // to see the count at the cycle it happened on.
        self.shared.sync(attrs);
        *byte = self.read_register((offset & 0xf) as u8, attrs.debug);
        if !attrs.debug {
            self.shared.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would start a timer or clear a flag, neither of
            // which the core can make harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        // A write is as time-sensitive as a read: loading T1C-H starts a
        // countdown, and the cycle it starts on is the cycle it expires from.
        self.shared.sync(attrs);
        self.write_register((offset & 0xf) as u8, *value);
        self.shared.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A 6522 is on an 8-bit bus. A 16-bit read of T1C-L would look like a
        // sensible way to get the counter and is not a thing that can happen.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Device for Via {
    fn class(&self) -> &'static DeviceClass {
        &VIA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // "Reset clears all internal registers (except T1 and T2 counters and
        // latches, and the SR)" — §3.9. What the counters hold after power-on
        // is undefined on the chip and zero here, because a machine that
        // starts from an undefined number is not deterministic
        // (`ROADMAP.md` §0) and no correct program depends on it.
        let mut state = self.shared.state.lock();
        let keep = *state;
        *state = State {
            ticks: keep.ticks,
            t1_counter: keep.t1_counter,
            t1_latch: keep.t1_latch,
            t1_fired: keep.t1_fired,
            t2_counter: keep.t2_counter,
            t2_latch_low: keep.t2_latch_low,
            t2_fired: keep.t2_fired,
            sr: keep.sr,
            ..State::default()
        };
        self.shared.publish(&state);
        drop(state);
        self.shared.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u8(state.ora)?;
        w.write_u8(state.orb)?;
        w.write_u8(state.ddra)?;
        w.write_u8(state.ddrb)?;
        w.write_u16(state.t1_counter)?;
        w.write_u16(state.t1_latch)?;
        w.write_bool(state.t1_fired)?;
        w.write_u16(state.t2_counter)?;
        w.write_u8(state.t2_latch_low)?;
        w.write_bool(state.t2_fired)?;
        w.write_u8(state.sr)?;
        w.write_u8(state.acr)?;
        w.write_u8(state.pcr)?;
        w.write_u8(state.ifr)?;
        w.write_u8(state.ier)
        // `pa_in` and `pb_in` are deliberately absent: what is driving a pin
        // from outside is the *other* device's state, and it will restore its
        // own and drive them again (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        *state = State {
            ticks: r.read_u64()?,
            ora: r.read_u8()?,
            orb: r.read_u8()?,
            ddra: r.read_u8()?,
            ddrb: r.read_u8()?,
            pa_in: 0,
            pb_in: 0,
            t1_counter: r.read_u16()?,
            t1_latch: r.read_u16()?,
            t1_fired: r.read_bool()?,
            t2_counter: r.read_u16()?,
            t2_latch_low: r.read_u8()?,
            t2_fired: r.read_bool()?,
            sr: r.read_u8()?,
            acr: r.read_u8()?,
            pcr: r.read_u8()?,
            ifr: r.read_u8()?,
            ier: r.read_u8()?,
        };
        self.shared.publish(&state);
        drop(state);
        self.shared.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: alloc::string::String::from(port),
                message: alloc::format!("the VIA drives only `{IRQ_PIN}`"),
            });
        }
        self.connect_irq(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            self.shared.refresh_irq();
        }
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A timer counter read has to report the count at the cycle of the
    /// read, and a timeout has to reach the CPU on the cycle it happens.
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

/// The `wdc.w65c22` device class.
pub static VIA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "WDC W65C22 VIA: two 8-bit ports with data direction, and both timers",
    properties: &[],
    construct: |props| Ok(Box::new(Via::new(props)?)),
};

/// Add [`VIA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&VIA_CLASS)
}

/// Bind [`VIA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Via::new(props)?)))
}

/// What the validator should know about `wdc.w65c22`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS_NAME)
        .port(IRQ_PIN, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireId};
    use alloc::string::ToString;
    use alloc::vec::Vec;

    fn port(via: &Via) -> Arc<ViaPort> {
        Arc::new(ViaPort {
            shared: Arc::clone(&via.shared),
        })
    }

    fn peek(via: &Via, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        port(via)
            .read(index, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peek_debug(via: &Via, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        port(via)
            .read(index, &mut byte, MemAttrs::DEBUG)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(via: &Via, index: u64, value: u8) {
        port(via)
            .write(index, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// A wire with one source, so the IRQ pin has something to drive.
    fn dummy_source() -> WireSource {
        let src = WireId::new(1);
        WireSource::new(Wire::builder().source(src).build_shared(), src)
    }

    #[test]
    fn the_two_ports_read_back_differently_and_that_is_the_point() {
        let via = Via::bare();
        // Port B: outputs read the register, inputs read the pin.
        poke(&via, 0x2, 0x0f); // DDRB: low nibble out
        poke(&via, 0x0, 0xff); // ORB
        via.set_port_b(0xa0);
        assert_eq!(peek(&via, 0x0), 0xaf, "0xA from the pins, 0xF from ORB");
        assert_eq!(via.port_b(), 0xaf);

        // Port A: every pin reads the pin, and nothing but the output register
        // is driving the ones that are outputs here.
        poke(&via, 0x3, 0xf0); // DDRA: high nibble out
        poke(&via, 0x1, 0x55); // ORA
        via.set_port_a(0x0f);
        assert_eq!(peek(&via, 0x1), 0x5f);
        assert_eq!(peek(&via, 0xf), 0x5f, "$F is $1 without the handshake");
        assert_eq!(peek(&via, 0x2), 0x0f);
        assert_eq!(peek(&via, 0x3), 0xf0);
    }

    #[test]
    fn timer_one_counts_phi2_down_and_sets_its_flag_once() {
        let via = Via::bare();
        // Both timers come up armed at zero, so both time out on tick 1 and
        // are then done. Get that out of the way, and with it the check that
        // it happens at all: an unarmed timer would report no event.
        assert_eq!(via.shared.next_event.load(Ordering::Relaxed), 1);
        via.advance_to(1);
        assert_eq!(via.ifr() & (IFR_T1 | IFR_T2), IFR_T1 | IFR_T2);
        poke(&via, 0xd, IFR_T1 | IFR_T2);
        assert_eq!(via.shared.next_event.load(Ordering::Relaxed), NO_EVENT);

        // A count of 9 in one-shot mode: loading T1C-H starts it.
        poke(&via, 0x4, 0x09); // T1L-L
        poke(&via, 0x5, 0x00); // T1C-H: transfer and go
        assert_eq!(via.timer1(), 9);
        assert_eq!(via.ifr() & IFR_T1, 0);
        assert_eq!(
            via.shared.next_event.load(Ordering::Relaxed),
            11,
            "a counter of 9 loaded on tick 1 times out on tick 11"
        );

        via.advance_to(10);
        assert_eq!(via.timer1(), 0);
        assert_eq!(via.ifr() & IFR_T1, 0, "not yet");

        via.advance_to(11);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1, "IFR6 on the count to zero");
        assert_eq!(via.timer1(), 0xffff, "and it rolls over");

        // One-shot: a second pass through zero raises nothing, and there is no
        // further event to schedule.
        poke(&via, 0xd, IFR_T1); // clear it by writing a 1
        assert_eq!(via.ifr() & IFR_T1, 0);
        via.advance_to(11 + 0x1_0000);
        assert_eq!(via.ifr() & IFR_T1, 0, "one shot means one");
        assert_eq!(via.shared.next_event.load(Ordering::Relaxed), NO_EVENT);
    }

    #[test]
    fn reading_the_low_counter_clears_the_flag_and_reading_the_latch_does_not() {
        // The distinction Table 2-7 exists to draw.
        let via = Via::bare();
        poke(&via, 0x4, 0x02);
        poke(&via, 0x5, 0x00);
        via.advance_to(3);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1);

        assert_eq!(peek(&via, 0x6), 0x02, "T1L-L");
        assert_eq!(peek(&via, 0x7), 0x00, "T1L-H");
        assert_eq!(via.ifr() & IFR_T1, IFR_T1, "a latch read clears nothing");

        let _ = peek(&via, 0x4);
        assert_eq!(via.ifr() & IFR_T1, 0, "a counter read does");
    }

    #[test]
    fn free_run_reloads_from_the_latches_and_keeps_going() {
        let via = Via::bare();
        poke(&via, 0xb, ACR_T1_FREE_RUN);
        poke(&via, 0x4, 0x03);
        poke(&via, 0x5, 0x00); // period 4
        via.advance_to(4);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1);
        assert_eq!(via.timer1(), 3, "reloaded from the latches");

        poke(&via, 0xd, IFR_T1);
        via.advance_to(8);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1, "and again");
        assert_eq!(via.timer1(), 3);

        // Many periods in one budget is still one flag and the right phase.
        poke(&via, 0xd, IFR_T1);
        via.advance_to(8 + 4 * 1000 + 2);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1);
        assert_eq!(via.timer1(), 1);
    }

    #[test]
    fn timer_two_counts_phi2_unless_it_is_told_to_count_pb6() {
        let via = Via::bare();
        poke(&via, 0x8, 0x05); // T2 low latch
        poke(&via, 0x9, 0x00); // T2C-H: transfer and go
        assert_eq!(via.timer2(), 5);
        via.advance_to(6);
        assert_eq!(via.ifr() & IFR_T2, IFR_T2);
        // Reading T2C-L clears it (Table 2-9).
        let _ = peek(&via, 0x8);
        assert_eq!(via.ifr() & IFR_T2, 0);

        // Pulse-counting mode: nothing drives PB6, so nothing counts.
        let via = Via::bare();
        poke(&via, 0xb, ACR_T2_PULSE);
        poke(&via, 0x8, 0x05);
        poke(&via, 0x9, 0x00);
        via.advance_to(1000);
        assert_eq!(via.timer2(), 5, "no pulses, no counting");
        assert_eq!(via.ifr() & IFR_T2, 0);
    }

    #[test]
    fn the_enable_register_sets_and_clears_by_its_top_bit() {
        let via = Via::bare();
        assert_eq!(peek(&via, 0xe), 0x80, "read back with bit 7 set");

        poke(&via, 0xe, 0x80 | IFR_T1 | IFR_T2); // set both timer enables
        assert_eq!(peek(&via, 0xe), 0x80 | IFR_T1 | IFR_T2);
        poke(&via, 0xe, IFR_T2); // bit 7 clear: clear timer 2's
        assert_eq!(peek(&via, 0xe), 0x80 | IFR_T1);
        poke(&via, 0xe, 0x00);
        assert_eq!(peek(&via, 0xe), 0x80 | IFR_T1, "a zero changes nothing");
    }

    #[test]
    fn the_interrupt_output_follows_the_flags_and_their_enables() {
        let via = Via::bare();
        via.connect_irq(dummy_source());
        assert_eq!(via.irq_level(), Level::Low);

        // A flag with no enable does not interrupt, and bit 7 stays clear.
        poke(&via, 0x4, 0x01);
        poke(&via, 0x5, 0x00);
        via.advance_to(2);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1);
        assert_eq!(via.ifr() & IFR_ANY, 0, "IFR7 is the *enabled* wired-OR");
        assert_eq!(via.irq_level(), Level::Low);

        // Enabling it asserts immediately — the expression is combinational.
        poke(&via, 0xe, 0x80 | IFR_T1);
        assert_eq!(via.irq_level(), Level::High);
        assert_eq!(via.ifr() & IFR_ANY, IFR_ANY);

        // And clearing the flag deasserts it.
        poke(&via, 0xd, IFR_T1);
        assert_eq!(via.irq_level(), Level::Low);
    }

    #[test]
    fn a_debug_access_advances_nothing_and_clears_nothing() {
        let via = Via::bare();
        poke(&via, 0x4, 0x01);
        poke(&via, 0x5, 0x00);
        via.advance_to(2);
        assert_eq!(via.ifr() & IFR_T1, IFR_T1);
        assert_eq!(peek_debug(&via, 0x4), 0xff, "the counter rolled over");
        assert_eq!(via.ifr() & IFR_T1, IFR_T1, "and the flag is still there");

        assert_eq!(
            port(&via).write(0x5, &[0x00], MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
    }

    #[test]
    fn only_byte_accesses_are_accepted() {
        let via = Via::bare();
        let p = port(&via);
        assert_eq!(
            p.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            p.write(0, &[0, 0], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(p.constraints().min, Width::U8);
    }

    #[test]
    fn a_reset_clears_the_control_registers_and_leaves_the_timers() {
        let via = Via::bare();
        poke(&via, 0x2, 0xff);
        poke(&via, 0x0, 0xa5);
        poke(&via, 0x4, 0x34);
        poke(&via, 0x5, 0x12);
        poke(&via, 0xe, 0x80 | IFR_T1);
        via.reset(ResetKind::Cold);
        assert_eq!(peek(&via, 0x2), 0, "DDRB");
        assert_eq!(peek(&via, 0xe), 0x80, "IER");
        assert_eq!(via.timer1(), 0x1234, "§3.9: the counters survive it");
        assert_eq!(peek(&via, 0x6), 0x34, "and so do the latches");
    }

    #[test]
    fn the_whole_register_block_is_the_region() {
        let via = Via::bare();
        assert_eq!(via.region("").expect("mapped").len(), REGISTER_COUNT);
        assert!(via.region("regs").is_some());
        assert!(via.region("porta").is_none());
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = Via::bare();
        poke(&saved, 0x3, 0xff);
        poke(&saved, 0x1, 0x5a);
        poke(&saved, 0xb, ACR_T1_FREE_RUN);
        poke(&saved, 0x4, 0xff);
        poke(&saved, 0x5, 0x01);
        poke(&saved, 0xe, 0x80 | IFR_T1);
        saved.advance_to(200);

        let mut shape = MachineShape::new();
        shape.add_device("via", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("via", CLASS_NAME, STATE_VERSION).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = Via::bare();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("via", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        let before: Vec<u8> = (0..16).map(|i| peek_debug(&saved, i)).collect();
        let after: Vec<u8> = (0..16).map(|i| peek_debug(&restored, i)).collect();
        assert_eq!(before, after);
        assert_eq!(restored.ticks(), 200, "and it resumes from the same tick");

        // Both keep running identically from there.
        saved.advance_to(1000);
        restored.advance_to(1000);
        assert_eq!(saved.timer1(), restored.timer1());
        assert_eq!(saved.ifr(), restored.ifr());
    }

    #[test]
    fn the_class_is_registrable_and_takes_no_properties() {
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("a fresh registry");
        let class = registry.get(CLASS_NAME).expect("registered");
        assert_eq!(class.version, STATE_VERSION);
        assert!(class.properties.is_empty());
        let device = (class.construct)(&Props::new()).expect("nothing to give it");
        assert_eq!(device.class().name, CLASS_NAME);
        assert!(device.is_lazy(), "the timers are sampled");
        assert!(device.connect("ca1", dummy_source()).is_err());

        let e = Via::new(&Props::new().with("port", "console"))
            .expect_err("a property it does not have")
            .to_string();
        assert!(e.contains("port"), "{e}");
    }
}
