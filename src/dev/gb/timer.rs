//! The Game Boy's divider and timer — `$FF04`-`$FF07`.
//!
//! Four registers, and almost every one of them is a window onto the same piece
//! of hardware: **one 16-bit counter, clocked at the crystal rate**, which
//! nothing can stop and only a write can reset.
//!
//! ```text
//!   $FF04  DIV   the counter's high byte. Writing *any* value zeroes all 16 bits
//!   $FF05  TIMA  a second counter, clocked from a falling edge inside the first
//!   $FF06  TMA   what TIMA reloads to when it overflows
//!   $FF07  TAC   bit 2 enables TIMA; bits 0-1 pick which counter bit clocks it
//! ```
//!
//! # The falling-edge detector is the whole design
//!
//! `TIMA` is not "incremented every N cycles". It is incremented by the **falling
//! edge** of `counter_bit AND tac_enable`, and modelling it that way is what
//! produces every one of the divider's famous side effects for free rather than
//! as special cases (Pan Docs, *Timer and Divider Registers*, and Gekkio's
//! *Complete Technical Reference*):
//!
//! * Writing `DIV` while the selected bit is high drops that bit, which is a
//!   falling edge, which **increments `TIMA`**. A program that resets the
//!   divider in a loop can clock the timer far faster than `TAC` says.
//! * Changing `TAC` from a slow rate to a fast one — or disabling the timer
//!   while the selected bit is high — can produce the same spurious increment.
//! * The rates themselves are not four constants but four bit positions, and
//!   the 16 384 Hz `DIV` register is simply bits 8-15 of the same counter.
//!
//! # `DIV` is audible
//!
//! The APU's 512 Hz frame sequencer — which drives every channel's length
//! counter, envelope and sweep — is clocked from **bit 12 of this same
//! counter**, not from anything of its own. So a write to `$FF04` shifts the
//! phase of every envelope in the machine, and a program that writes it in a
//! loop can stop envelopes advancing altogether. That is not a curiosity: it is
//! a genuine cross-device relationship, and this device publishes it as an
//! ordinary output pin ([`DIV_APU_PIN`]) that the machine file wires to the APU.
//! Nothing in `core::` had to learn what a Game Boy is for that to work.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2), on the crystal's own domain — one
//! tick is one of the 4 194 304 clocks a second, four to a CPU machine cycle.
//! [`GbTimer::next_event_tick`] reports the next tick at which anything a
//! program can *see* changes: the next `DIV` increment, the next `TIMA`
//! increment, the end of the overflow reload delay. Bounding the scheduler's
//! quantum by that is what makes a mid-quantum `TIMA` read correct — not because
//! the device is caught up to the exact cycle, but because between two of its
//! own events there is nothing to catch up *to*.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0), *Timer and Divider Registers*
//! and *Audio Details*, plus Gekkio's *Game Boy: Complete Technical Reference*
//! for the overflow delay's four cycles. No emulator source was consulted.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

/// Where the register block sits in the CPU's address space.
pub const REGISTER_BASE: u64 = 0xff04;

/// How many bytes it covers: `DIV`, `TIMA`, `TMA`, `TAC`.
pub const REGISTER_LEN: u64 = 4;

/// The name a `map` statement reaches the register block by.
pub const REGISTER_REGION: &str = "regs";

/// The timer-interrupt output pin.
pub const IRQ_PIN: &str = "irq";

/// The output that clocks the APU's frame sequencer: the falling edge of bit 12
/// of the internal divider, i.e. 512 Hz.
pub const DIV_APU_PIN: &str = "div-apu";

/// Which bit of the internal counter feeds the APU's frame sequencer.
///
/// 4 194 304 / 2^13 = 512 Hz, the rate every Game Boy sound document quotes.
const DIV_APU_BIT: u16 = 1 << 12;

/// How many crystal clocks `TIMA` reads zero for after it overflows, before the
/// reload from `TMA` happens and the interrupt is requested.
///
/// Gekkio measures this as one machine cycle. It is observable: a write to
/// `TIMA` inside the window cancels the reload, and a write to `TMA` inside it
/// is the value that gets loaded.
const RELOAD_DELAY: u8 = 4;

/// What the internal counter holds when a DMG boot ROM hands control to the
/// cartridge at `$0100`.
///
/// Pan Docs' *Hardware Registers* table gives `DIV` as `$AB` at that moment on a
/// DMG, and this counter is free-running from power-on — nothing can stop it —
/// so a machine that skips the boot ROM has to start it here or every game that
/// seeds itself from `DIV` gets the same "random" number on every run.
///
/// The **low** byte is not documented anywhere, so it is zero rather than
/// guessed at: only the visible byte is a fact (`ROADMAP.md` §1 on facts versus
/// somebody's implementation of them).
pub const POST_BOOT_COUNTER: u16 = 0xab00;

/// The bit of the internal counter that `TAC`'s low two bits select.
///
/// The order is not monotonic and that is not a typo: `00` is the *slowest*
/// (bit 9, 4096 Hz) and `01` the fastest (bit 3, 262 144 Hz).
const fn selected_bit(tac: u8) -> u16 {
    match tac & 3 {
        0 => 1 << 9,
        1 => 1 << 3,
        2 => 1 << 5,
        _ => 1 << 7,
    }
}

/// Everything under the lock.
#[derive(Debug, Clone, Copy, Default)]
struct Regs {
    /// The 16-bit counter. `DIV` is its high byte.
    counter: u16,
    tima: u8,
    tma: u8,
    tac: u8,
    /// The falling-edge detector's previous input: `counter_bit AND enable`.
    edge: bool,
    /// The previous state of the APU clock bit, for its own edge detector.
    apu_edge: bool,
    /// Clocks left in the overflow reload window, or zero when none is pending.
    reload: u8,
}

impl Regs {
    /// The input to the `TIMA` edge detector as it stands.
    fn edge_input(&self) -> bool {
        self.tac & 0x04 != 0 && self.counter & selected_bit(self.tac) != 0
    }

    /// Re-evaluate the edge detector after something changed the counter or
    /// `TAC`, incrementing `TIMA` on a falling edge.
    ///
    /// Returns whether `TIMA` overflowed. This is the *only* path that
    /// increments `TIMA`, which is what makes the `DIV`-write and `TAC`-change
    /// side effects fall out rather than needing to be written down.
    fn settle_edge(&mut self) -> bool {
        let now = self.edge_input();
        let falling = self.edge && !now;
        self.edge = now;
        if !falling {
            return false;
        }
        self.tima = self.tima.wrapping_add(1);
        if self.tima == 0 {
            self.reload = RELOAD_DELAY;
            return true;
        }
        false
    }
}

/// What the register block and the device share.
struct Shared {
    regs: Mutex<Regs>,
    links: Mutex<Links>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The tick reached, republished on every advance.
    ///
    /// The scheduler asks a lazily-advanced device where it is with its slot
    /// held at [`LockRank::LEAF`], so this may not be behind a lock.
    tick: AtomicU64,
    /// [`Shared::compute_next_event`], republished alongside [`Shared::tick`].
    next_event: AtomicU64,
}

#[derive(Debug, Default)]
struct Links {
    irq: Option<WireSource>,
    div_apu: Option<WireSource>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("regs", &self.regs)
            .field("tick", &self.tick.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Shared {
    /// The next tick at which something a program can observe changes.
    ///
    /// Three candidates, and the earliest wins: the next `DIV` increment, the
    /// next `TIMA` increment, and the end of an overflow's reload window. What
    /// this buys is not precision for its own sake — it is that between two
    /// consecutive events *nothing changes*, so a read that lands in the gap is
    /// correct even though the device has not been advanced to the exact tick
    /// (`ROADMAP.md` §4.2's known intra-quantum staleness).
    fn compute_next_event(&self, regs: &Regs, now: u64) -> u64 {
        // `DIV` steps every 256 clocks.
        let to_div = 256 - u64::from(regs.counter & 0xff);
        let mut delta = to_div;
        if regs.reload > 0 {
            delta = delta.min(u64::from(regs.reload));
        }
        if regs.tac & 0x04 != 0 {
            let period = u64::from(selected_bit(regs.tac)) * 2;
            let phase = u64::from(regs.counter) % period;
            delta = delta.min(period - phase);
        }
        // The APU's 512 Hz clock is a visible effect too — it moves every
        // envelope in the machine.
        let apu_period = u64::from(DIV_APU_BIT) * 2;
        let apu_phase = u64::from(regs.counter) % apu_period;
        delta = delta.min(apu_period - apu_phase);
        now + delta.max(1)
    }

    fn publish(&self, regs: &Regs, now: u64) {
        self.tick.store(now, Ordering::Relaxed);
        self.next_event
            .store(self.compute_next_event(regs, now), Ordering::Relaxed);
    }

    /// Advance to `target`, returning whether the timer interrupt fired and
    /// how many APU frame-sequencer edges happened.
    ///
    /// The loop steps one clock at a time. It is bounded by
    /// [`compute_next_event`](Shared::compute_next_event) — catch-up never
    /// crosses the device's own next event — so a span here is at most 256
    /// clocks and usually far fewer.
    fn advance_locked(&self, regs: &mut Regs, target: u64) -> (bool, u32) {
        let mut now = self.tick.load(Ordering::Relaxed);
        let mut irq = false;
        let mut apu_edges = 0u32;
        while now < target {
            now += 1;
            // The reload window is counted down *before* the counter moves, so
            // that a write landing inside it (which cancels or redirects the
            // reload) has a whole clock to happen in.
            if regs.reload > 0 {
                regs.reload -= 1;
                if regs.reload == 0 {
                    regs.tima = regs.tma;
                    irq = true;
                }
            }
            regs.counter = regs.counter.wrapping_add(1);
            regs.settle_edge();
            let apu_now = regs.counter & DIV_APU_BIT != 0;
            if regs.apu_edge && !apu_now {
                apu_edges += 1;
            }
            regs.apu_edge = apu_now;
        }
        self.tick.store(now, Ordering::Relaxed);
        (irq, apu_edges)
    }

    /// Drive the two output pins, with no lock of this device held.
    fn drive(&self, irq: bool, apu_edges: u32) {
        let (irq_src, apu_src) = {
            let links = self.links.lock();
            (links.irq.clone(), links.div_apu.clone())
        };
        if irq && let Some(src) = irq_src {
            // A pulse, not a level: the request is an event, and the CPU's pin
            // latches the rising edge into `IF`.
            src.set(Level::High);
            src.set(Level::Low);
        }
        if let Some(src) = apu_src {
            for _ in 0..apu_edges {
                src.set(Level::High);
                src.set(Level::Low);
            }
        }
    }

    /// Catch up before answering an access.
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
        let _ = handle.sync(kind);
    }
}

/// The divider and timer as a device.
pub struct GbTimer {
    shared: Arc<Shared>,
    regs_region: RegionRef,
}

impl fmt::Debug for GbTimer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbTimer")
            .field("regs", &self.shared.regs)
            .finish_non_exhaustive()
    }
}

impl Default for GbTimer {
    fn default() -> Self {
        GbTimer::new()
    }
}

impl GbTimer {
    /// A timer in its power-on state.
    #[must_use]
    pub fn new() -> GbTimer {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::default()),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(256),
        });
        let regs_region = Arc::new(MmioRegion::io(
            "gb.timer.regs",
            REGISTER_LEN,
            Arc::new(TimerPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbTimer {
            shared,
            regs_region,
        }
    }

    /// Build one from machine-description properties. It takes none.
    ///
    /// # Errors
    ///
    /// If any property was given at all — a typo'd property that is silently
    /// ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<GbTimer> {
        props.reader().finish()?;
        Ok(GbTimer::new())
    }

    /// `DIV` as the guest reads it: the counter's high byte.
    #[must_use]
    pub fn div(&self) -> u8 {
        (self.shared.regs.lock().counter >> 8) as u8
    }

    /// The whole 16-bit internal counter — not visible to a guest, but visible
    /// to a test, and the thing every documented side effect is about.
    #[must_use]
    pub fn counter(&self) -> u16 {
        self.shared.regs.lock().counter
    }

    /// `TIMA`.
    #[must_use]
    pub fn tima(&self) -> u8 {
        self.shared.regs.lock().tima
    }

    /// `TMA`.
    #[must_use]
    pub fn tma(&self) -> u8 {
        self.shared.regs.lock().tma
    }

    /// `TAC`.
    #[must_use]
    pub fn tac(&self) -> u8 {
        self.shared.regs.lock().tac
    }

    /// Connect the timer-interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        self.shared.links.lock().irq = Some(source);
    }

    /// Connect the 512 Hz output that clocks the APU's frame sequencer.
    pub fn attach_div_apu(&self, source: WireSource) {
        self.shared.links.lock().div_apu = Some(source);
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Advance to `target` clocks since reset, driving whatever fires.
    pub fn advance_to(&self, target: u64) {
        let (irq, apu_edges) = {
            let mut regs = self.shared.regs.lock();
            let out = self.shared.advance_locked(&mut regs, target);
            self.shared
                .publish(&regs, self.shared.tick.load(Ordering::Relaxed));
            out
        };
        // Outward actions only after the critical section is released — the
        // re-entrancy contract (`ROADMAP.md` §4.4).
        self.shared.drive(irq, apu_edges);
    }

    /// Advance by `clocks` more.
    pub fn advance_by(&self, clocks: u64) {
        self.advance_to(self.shared.tick.load(Ordering::Relaxed) + clocks);
    }

    /// The tick this device's own next visible change falls on.
    #[must_use]
    pub fn next_event(&self) -> u64 {
        self.shared.next_event.load(Ordering::Relaxed)
    }

    /// Read one register by index, 0-3, without catching up — for a test.
    #[must_use]
    pub fn read_register(&self, index: u8) -> u8 {
        let regs = self.shared.regs.lock();
        read_reg(&regs, index)
    }

    /// Write one register by index, 0-3, without catching up — for a test.
    pub fn write_register(&self, index: u8, value: u8) {
        let irq = {
            let mut regs = self.shared.regs.lock();
            let out = write_reg(&mut regs, index, value);
            self.shared
                .publish(&regs, self.shared.tick.load(Ordering::Relaxed));
            out
        };
        self.shared.drive(irq, 0);
    }
}

/// `DIV`, `TIMA`, `TMA`, `TAC` by index.
fn read_reg(regs: &Regs, index: u8) -> u8 {
    match index & 3 {
        0 => (regs.counter >> 8) as u8,
        // Inside the reload window `TIMA` genuinely reads zero: the counter has
        // wrapped and the reload has not happened yet.
        1 => regs.tima,
        2 => regs.tma,
        // Only three bits exist; the rest read as ones.
        _ => regs.tac | 0xf8,
    }
}

/// Returns whether the write itself made `TIMA` overflow — which a `DIV` write
/// or a `TAC` change genuinely can, through the falling-edge detector.
fn write_reg(regs: &mut Regs, index: u8, value: u8) -> bool {
    match index & 3 {
        0 => {
            // Any value resets all sixteen bits, not just the visible byte.
            regs.counter = 0;
            regs.settle_edge()
        }
        1 => {
            // A write inside the reload window cancels the reload, and `TIMA`
            // keeps the written value instead of `TMA`.
            regs.tima = value;
            regs.reload = 0;
            false
        }
        2 => {
            regs.tma = value;
            // A write during the reload *cycle* is what gets loaded, because
            // the reload reads `TMA` when it happens rather than when the
            // overflow did.
            false
        }
        _ => {
            regs.tac = value & 0x07;
            regs.settle_edge()
        }
    }
}

/// The `$FF04`-`$FF07` register block.
struct TimerPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for TimerPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TimerPort").finish_non_exhaustive()
    }
}

impl MemOps for TimerPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // First, and outside every lock this device owns.
        self.shared.sync(attrs);
        *byte = read_reg(&self.shared.regs.lock(), offset as u8);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debugger write to `DIV` would reset the divider and could clock
            // `TIMA`; that is not something the core can make safe, so it is
            // refused rather than guessed at (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let irq = {
            let mut regs = self.shared.regs.lock();
            let out = write_reg(&mut regs, offset as u8, *value);
            self.shared
                .publish(&regs, self.shared.tick.load(Ordering::Relaxed));
            out
        };
        // The re-entrancy contract: act outward only once the lock is released.
        // Note this cannot fire the interrupt itself — the overflow starts the
        // reload delay, and the request comes four clocks later, from
        // `advance_locked`.
        let _ = irq;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `gb.timer` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.timer",
    version: 1,
    summary: "Game Boy divider and timer ($FF04-$FF07), including the 512 Hz APU clock",
    properties: &[],
    construct: |props| Ok(Box::new(GbTimer::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbTimer {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Both outputs idle low, and a fresh net is already low, so the realize
        // sweep has nothing to correct here (`ROADMAP.md` §4.3).
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs_region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            IRQ_PIN => self.attach_irq(source),
            DIV_APU_PIN => self.attach_div_apu(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "the timer drives `{IRQ_PIN}` and `{DIV_APU_PIN}`, nothing else"
                    ),
                });
            }
        }
        Ok(())
    }

    /// Back to the state a boot ROM would have left.
    ///
    /// [`GbTimer::new`] is the honest power-on state — a counter of zero — and
    /// this is what a *machine* performs, which on a console means "after the
    /// boot ROM". The split is the same one `cpu.sm83` makes with
    /// `Regs::post_boot_dmg`, and for the same reason: rsemu ships no boot ROM,
    /// so the state one would have produced has to come from somewhere.
    fn reset(&self, _kind: ResetKind) {
        let mut regs = self.shared.regs.lock();
        *regs = Regs::default();
        regs.counter = POST_BOOT_COUNTER;
        // The edge detectors start from the counter they are now looking at,
        // rather than from zero, or the first tick after a reset looks like an
        // edge that never happened.
        regs.edge = regs.edge_input();
        regs.apu_edge = regs.counter & DIV_APU_BIT != 0;
        self.shared.tick.store(0, Ordering::Relaxed);
        self.shared.publish(&regs, 0);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        w.write_u16(regs.counter)?;
        w.write_u8(regs.tima)?;
        w.write_u8(regs.tma)?;
        w.write_u8(regs.tac)?;
        w.write_bool(regs.edge)?;
        w.write_bool(regs.apu_edge)?;
        w.write_u8(regs.reload)?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let regs = Regs {
            counter: r.read_u16()?,
            tima: r.read_u8()?,
            tma: r.read_u8()?,
            tac: r.read_u8()?,
            edge: r.read_bool()?,
            apu_edge: r.read_bool()?,
            reload: r.read_u8()?,
        };
        let tick = r.read_u64()?;
        let mut slot = self.shared.regs.lock();
        *slot = regs;
        self.shared.tick.store(tick, Ordering::Relaxed);
        // The next-event tick is derived state and has to follow the load.
        self.shared.publish(&regs, tick);
        Ok(())
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        GbTimer::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        GbTimer::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for GbTimer {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(GbTimer::from_props(props)?))
    })
}

/// What the validator should know about `gb.timer`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS.name)
        .port(IRQ_PIN, PortDir::Out)
        .port(DIV_APU_PIN, PortDir::Out)
        .region(REGISTER_REGION)
}
