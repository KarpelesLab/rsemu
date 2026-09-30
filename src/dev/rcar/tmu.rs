//! A Renesas TMU — the three-channel 32-bit down-counting timer unit that
//! Renesas has carried from the SH-3 through the SH-4A into R-Car.
//!
//! # Sources
//!
//! Renesas *SH7780 Hardware Manual*, section 22 ("Timer Unit (TMU)"): §22.3
//! for the registers — `TOCR`, `TSTR`, and per channel `TCOR`, `TCNT` and
//! `TCR` with its `UNF`, `UNIE`, `CKEG` and `TPSC` fields and their reset
//! values — and §22.4 for the counting operation: `TCNT` counts down on the
//! prescaled peripheral clock while its `STR` bit is set, and on underflow is
//! reloaded from `TCOR` and sets `UNF`, which requests `TUNIn` while `UNIE`
//! is set. The SH7785 and SH7786 manuals and the R-Car TMU chapters describe
//! the same unit at the same offsets. No emulator, kernel or boot loader
//! source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # The register file
//!
//! ```text
//!   0x00  TOCR   8  timer output control: TCOE (stored; there is no pin)
//!   0x04  TSTR   8  timer start: STR0, STR1, STR2
//!   0x08  TCOR0 32  constant: what TCNT0 reloads from on underflow
//!   0x0c  TCNT0 32  the counter
//!   0x10  TCR0  16  control: UNF(8), UNIE(5), CKEG(4:3), TPSC(2:0)
//!   0x14  TCOR1 / 0x18 TCNT1 / 0x1c TCR1
//!   0x20  TCOR2 / 0x24 TCNT2 / 0x28 TCR2  (+ ICPF(9), ICPE(7:6))
//!   0x2c  TCPR2 32  input capture (reads 0: capture is not modelled)
//! ```
//!
//! Byte, halfword and word accesses are all accepted, naturally aligned, and
//! each reaches only the lanes it covers — so a byte write to `TCR + 1` is how
//! `UNF` is written without touching `UNIE`.
//!
//! `TPSC` selects `Pφ/4`, `/16`, `/64`, `/256` or `/1024` (codes 0–4). Codes
//! 5–7 select the on-chip RTC clock or the external `TCLK` pin, neither of
//! which exists here: a channel so configured **does not count**, and says so
//! by standing still rather than by inventing a rate. `CKEG` (which edge of
//! the external clock counts) is stored and has nothing to act on. Input
//! capture on channel 2 (`ICPE`, `ICPF`, `TCPR2`) is not modelled; `ICPE`
//! reads back what was written and `ICPF` never sets.
//!
//! # Counting, and the one choice the manual leaves open
//!
//! The period is `TCOR + 1` count clocks: `TCNT` runs `TCOR, TCOR − 1, …, 0`,
//! and the count clock after the one that found it at zero is the underflow,
//! which reloads `TCOR` in the same step.
//!
//! On the part the prescaler is a free-running divider on `Pφ`, so the first
//! count clock after `STR` is set arrives somewhere within one prescaler
//! period. This model restarts the channel's prescaler phase when the channel
//! is started and when its `TPSC` changes, so the first count clock is exactly
//! one prescaler period after the start. That is deterministic, and it is off
//! from any particular piece of silicon by less than one count clock at the
//! start of a run, never cumulatively. A write to `TCNT` or `TCOR` while the
//! channel runs keeps the phase.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) on this device's own clock domain,
//! one tick of which is **one `Pφ` cycle** — the machine file binds that
//! domain with `clock = …`, exactly as `st.tim` is bound to `CK_INT`. Nothing
//! is stepped per tick and nothing reads a host clock. Each channel keeps an
//! *anchor*: the tick of its last count clock (or of its start) and the
//! `TCNT` it held there. Everything else is integer arithmetic from that:
//!
//! ```text
//!   clocks(t)  = (t − anchor) / div
//!   TCNT(t)    = count − clocks                          while clocks ≤ count
//!              = TCOR − ((clocks − count − 1) mod (TCOR + 1))    after that
//!   underflows = 0,  or 1 + (clocks − count − 1) / (TCOR + 1)
//! ```
//!
//! A read of `TCNT` is caught up to the access's tick and answered from that
//! formula. The only instant the scheduler is told about
//! ([`Device::next_event_tick`]) is the next underflow of a channel whose
//! interrupt would *change* there — started, `UNIE` set, `UNF` clear — so a
//! free-running timer nobody listens to costs nothing, and an interrupt lands
//! on the tick it is due on rather than at the end of a quantum.
//!
//! # The interrupts
//!
//! One level output per channel, `irq0`–`irq2`, high while `UNF && UNIE`: the
//! `TUNI0`–`TUNI2` requests, which an R-Car routes to three GIC SPIs.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.tmu";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers: `TOCR` through `TCPR2`.
pub const REGISTER_WINDOW_LEN: u64 = 0x30;

/// How many channels one unit has.
pub const CHANNELS: usize = 3;

/// The interrupt outputs, one per channel.
pub const IRQ_PINS: [&str; CHANNELS] = ["irq0", "irq1", "irq2"];

const TOCR: u64 = 0x00;
const TSTR: u64 = 0x04;
const TCPR2: u64 = 0x2c;

/// `TCR.UNF`: underflow flag. Cleared by writing 0; writing 1 is ignored.
const TCR_UNF: u16 = 1 << 8;
/// `TCR.UNIE`: underflow interrupt enable.
const TCR_UNIE: u16 = 1 << 5;
/// `TCR2.ICPF`: input capture flag. Never set here.
const TCR_ICPF: u16 = 1 << 9;
/// The plain storage bits of `TCR0`/`TCR1`: `UNIE`, `CKEG`, `TPSC`.
const TCR_MASK: u16 = 0x003f;
/// The plain storage bits of `TCR2`, which adds `ICPE[1:0]`.
const TCR2_MASK: u16 = 0x00ff;

/// `TSTR`'s bits: `STR0`–`STR2`.
const TSTR_MASK: u8 = 0x07;
/// `TOCR`'s bit: `TCOE`.
const TOCR_MASK: u8 = 0x01;

/// Tick value meaning "no event".
const NEVER: u64 = u64::MAX;

/// The `Pφ` divider `TPSC` selects, or `None` for a clock that is not here.
fn divider(tcr: u16) -> Option<u64> {
    match tcr & 7 {
        0 => Some(4),
        1 => Some(16),
        2 => Some(64),
        3 => Some(256),
        4 => Some(1024),
        // The RTC clock and TCLK: see the module docs.
        _ => None,
    }
}

/// One channel's registers and its counting anchor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Channel {
    tcor: u32,
    /// `TCNT` as of `anchor`.
    count: u32,
    /// The tick of the channel's last count clock, or of its start. Only
    /// meaningful while the channel runs.
    anchor: u64,
    tcr: u16,
}

impl Channel {
    /// Out of reset: §22.3's table, `TCOR` and `TCNT` all ones.
    const RESET: Channel = Channel {
        tcor: u32::MAX,
        count: u32::MAX,
        anchor: 0,
        tcr: 0,
    };

    /// `TCNT` and the number of underflows after `clocks` count clocks from
    /// the anchor.
    fn after(&self, clocks: u64) -> (u32, u64) {
        let count = u64::from(self.count);
        if clocks <= count {
            return ((count - clocks) as u32, 0);
        }
        let past = clocks - count - 1;
        let period = u64::from(self.tcor) + 1;
        (
            (u64::from(self.tcor) - past % period) as u32,
            1 + past / period,
        )
    }
}

/// Everything under the lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Regs {
    tocr: u8,
    tstr: u8,
    ch: [Channel; CHANNELS],
}

impl Regs {
    const RESET: Regs = Regs {
        tocr: 0,
        tstr: 0,
        ch: [Channel::RESET; CHANNELS],
    };

    /// The divider channel `n` is counting at, or `None` if it is not.
    fn running(&self, n: usize) -> Option<u64> {
        if self.tstr & (1 << n) == 0 {
            return None;
        }
        divider(self.ch[n].tcr)
    }

    /// Count clocks channel `n` has seen from its anchor to `now`.
    fn clocks(&self, n: usize, div: u64, now: u64) -> u64 {
        now.saturating_sub(self.ch[n].anchor) / div
    }

    /// `TCNT` of channel `n` at `now`.
    fn tcnt(&self, n: usize, now: u64) -> u32 {
        match self.running(n) {
            Some(div) => self.ch[n].after(self.clocks(n, div, now)).0,
            None => self.ch[n].count,
        }
    }

    /// Move channel `n`'s anchor to its last count clock at or before `now`,
    /// keeping the prescaler's phase. A stopped channel needs no anchor.
    fn rebase(&mut self, n: usize, now: u64) {
        if let Some(div) = self.running(n) {
            let clocks = self.clocks(n, div, now);
            let (tcnt, _) = self.ch[n].after(clocks);
            self.ch[n].count = tcnt;
            self.ch[n].anchor = self.ch[n].anchor.saturating_add(clocks * div);
        }
    }

    /// Every channel's interrupt level.
    fn levels(&self) -> [bool; CHANNELS] {
        core::array::from_fn(|n| {
            let tcr = self.ch[n].tcr;
            tcr & TCR_UNF != 0 && tcr & TCR_UNIE != 0
        })
    }

    /// The tick of the next underflow that would raise an interrupt.
    fn next_event(&self, now: u64) -> u64 {
        let mut next = NEVER;
        for n in 0..CHANNELS {
            let Some(div) = self.running(n) else {
                continue;
            };
            let ch = &self.ch[n];
            if ch.tcr & TCR_UNIE == 0 || ch.tcr & TCR_UNF != 0 {
                continue;
            }
            let (_, done) = ch.after(self.clocks(n, div, now));
            // The (done + 1)-th underflow is count clock number
            // count + 1 + done × (TCOR + 1) from the anchor.
            let period = u64::from(ch.tcor) + 1;
            let clock = (u64::from(ch.count) + 1).saturating_add(done.saturating_mul(period));
            let tick = ch.anchor.saturating_add(clock.saturating_mul(div));
            next = next.min(tick);
        }
        next
    }
}

/// What the register block and the device share.
struct Shared {
    regs: Mutex<Regs>,
    irqs: Mutex<[Option<WireSource>; CHANNELS]>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The tick reached. The scheduler reads it with its slot lock held, so it
    /// may not be behind a lock.
    tick: AtomicU64,
    /// [`Regs::next_event`], republished with every change; [`NEVER`] for none.
    next_event: AtomicU64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        match self.regs.try_lock() {
            Some(regs) => s.field("regs", &*regs),
            None => s.field("regs", &"<in use>"),
        };
        s.field("tick", &self.tick.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Shared {
    fn publish(&self, regs: &Regs) {
        let now = self.tick.load(Ordering::Relaxed);
        self.next_event
            .store(regs.next_event(now), Ordering::Relaxed);
    }

    /// Drive the three outputs. Never called with the register lock held.
    fn drive(&self, levels: [bool; CHANNELS]) {
        let outs = self.irqs.lock().clone();
        for (out, level) in outs.iter().zip(levels) {
            if let Some(out) = out {
                out.set(Level::from_bool(level));
            }
        }
    }

    /// Advance to `target`, setting `UNF` on every channel that underflowed.
    fn advance_to(&self, target: u64) {
        let levels = {
            let mut regs = self.regs.lock();
            let now = self.tick.load(Ordering::Relaxed);
            if target > now {
                for n in 0..CHANNELS {
                    let Some(div) = regs.running(n) else {
                        continue;
                    };
                    let (_, before) = regs.ch[n].after(regs.clocks(n, div, now));
                    let (_, after) = regs.ch[n].after(regs.clocks(n, div, target));
                    if after > before {
                        regs.ch[n].tcr |= TCR_UNF;
                    }
                }
                self.tick.store(target, Ordering::Relaxed);
            }
            self.publish(&regs);
            regs.levels()
        };
        // Outward only once the lock is released (`CLAUDE.md`, the
        // re-entrancy contract).
        self.drive(levels);
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

    /// Read the 32-bit slot at `slot`. Reads have no side effects.
    fn read_register(&self, regs: &Regs, slot: u64, now: u64) -> u32 {
        match slot {
            TOCR => u32::from(regs.tocr),
            TSTR => u32::from(regs.tstr),
            0x08..=0x2b => {
                let n = ((slot - 0x08) / 0x0c) as usize;
                match (slot - 0x08) % 0x0c {
                    0 => regs.ch[n].tcor,
                    4 => regs.tcnt(n, now),
                    _ => u32::from(regs.ch[n].tcr),
                }
            }
            // TCPR2: input capture is not modelled.
            TCPR2 => 0,
            _ => 0,
        }
    }

    /// Write the bits `mask` of the 32-bit slot at `slot`, at tick `now`.
    fn write_register(&self, regs: &mut Regs, slot: u64, value: u32, mask: u32, now: u64) {
        let merge32 = |old: u32| (old & !mask) | (value & mask);
        match slot {
            TOCR => {
                let m = mask as u8 & TOCR_MASK;
                regs.tocr = (regs.tocr & !m) | (value as u8 & m);
            }
            TSTR => {
                let m = mask as u8 & TSTR_MASK;
                let new = (regs.tstr & !m) | (value as u8 & m);
                for n in 0..CHANNELS {
                    let bit = 1u8 << n;
                    if regs.tstr & bit != 0 && new & bit == 0 {
                        // Stopping: freeze the count where it stands.
                        regs.rebase(n, now);
                    }
                    if regs.tstr & bit == 0 && new & bit != 0 {
                        // Starting: the prescaler phase restarts here.
                        regs.ch[n].anchor = now;
                    }
                }
                regs.tstr = new;
            }
            0x08..=0x2b => {
                let n = ((slot - 0x08) / 0x0c) as usize;
                regs.rebase(n, now);
                match (slot - 0x08) % 0x0c {
                    0 => regs.ch[n].tcor = merge32(regs.ch[n].tcor),
                    4 => regs.ch[n].count = merge32(regs.ch[n].count),
                    _ => {
                        let v = value as u16;
                        let m = mask as u16;
                        let storage = if n == 2 { TCR2_MASK } else { TCR_MASK };
                        let old = regs.ch[n].tcr;
                        let mut tcr = (old & !(m & storage)) | (v & m & storage);
                        // UNF (and ICPF) clear on a written 0; a 1 is ignored.
                        let flags = if n == 2 { TCR_UNF | TCR_ICPF } else { TCR_UNF };
                        let clear = !v & m & flags;
                        tcr = (tcr & !flags) | (old & flags & !clear);
                        regs.ch[n].tcr = tcr;
                        if (old ^ tcr) & 7 != 0 {
                            // A new prescaler: its phase starts now.
                            regs.ch[n].anchor = now;
                        }
                    }
                }
            }
            // TCPR2 is read-only; the rest of the window is reserved.
            _ => {}
        }
    }
}

/// The byte-lane mask of an access of `len` bytes at `offset`.
fn lane_mask(offset: u64, len: usize) -> u32 {
    let bytes = if len >= 4 {
        0xffff_ffff
    } else {
        (1u32 << (len * 8)) - 1
    };
    bytes << ((offset & 3) * 8)
}

/// The register block.
struct Port {
    shared: Arc<Shared>,
}

impl fmt::Debug for Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Port").finish_non_exhaustive()
    }
}

impl MemOps for Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.is_empty() || dst.len() > 4 {
            return Err(BusError::BadAccess);
        }
        // First, and outside every lock this device owns.
        self.shared.sync(attrs);
        let now = self.shared.tick.load(Ordering::Relaxed);
        let value = self
            .shared
            .read_register(&self.shared.regs.lock(), offset & !3, now);
        let bytes = (value >> ((offset & 3) * 8)).to_le_bytes();
        dst.copy_from_slice(&bytes[..dst.len()]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.is_empty() || src.len() > 4 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debugger write to TSTR or TCNT would move the guest's time;
            // it cannot be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let mut value = 0u32;
        for (i, byte) in src.iter().enumerate() {
            value |= u32::from(*byte) << (i * 8);
        }
        let shift = (offset & 3) * 8;
        let now = self.shared.tick.load(Ordering::Relaxed);
        let levels = {
            let mut regs = self.shared.regs.lock();
            self.shared.write_register(
                &mut regs,
                offset & !3,
                value << shift,
                lane_mask(offset, src.len()),
                now,
            );
            self.shared.publish(&regs);
            regs.levels()
        };
        self.shared.drive(levels);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

/// A TMU: three channels.
pub struct Tmu {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl fmt::Debug for Tmu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Tmu")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl Default for Tmu {
    fn default() -> Self {
        Tmu::with_defaults()
    }
}

impl Tmu {
    /// A unit in its reset state.
    #[must_use]
    pub fn with_defaults() -> Tmu {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::RESET),
            irqs: Mutex::with_rank(LockRank::WIRE, [None, None, None]),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(NEVER),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.tmu",
            REGISTER_WINDOW_LEN,
            Arc::new(Port {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Tmu { shared, region }
    }

    /// Build one from machine-description properties. It takes none: the
    /// input clock is the device's clock domain, which the machine file binds
    /// with `clock = …` (one tick per `Pφ` cycle).
    ///
    /// # Errors
    ///
    /// If any property was given at all.
    pub fn new(props: &Props) -> Result<Tmu> {
        props.reader().finish()?;
        Ok(Tmu::with_defaults())
    }

    /// `TCNT` of channel `n` as of the tick reached, without catching up —
    /// for a test or a monitor.
    #[must_use]
    pub fn tcnt(&self, n: usize) -> u32 {
        let now = self.shared.tick.load(Ordering::Relaxed);
        self.shared.regs.lock().tcnt(n, now)
    }

    /// Whether channel `n`'s interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self, n: usize) -> bool {
        self.shared.regs.lock().levels()[n]
    }
}

/// The `rcar.tmu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Renesas TMU: three 32-bit down-counters on the prescaled peripheral clock",
    properties: &[],
    construct: |props| Ok(Box::new(Tmu::new(props)?)),
};

impl Device for Tmu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // All three outputs idle low, and a fresh net is already low.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The tick is the clock domain's position, not this device's state,
        // and is not rewound.
        {
            let mut regs = self.shared.regs.lock();
            *regs = Regs::RESET;
            self.shared.publish(&regs);
        }
        self.shared.drive([false; CHANNELS]);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(n) = IRQ_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: String::from(port),
                message: String::from("a TMU drives `irq0`, `irq1` and `irq2`, one per channel"),
            });
        };
        self.shared.irqs.lock()[n] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if IRQ_PINS.contains(&port) {
            let levels = self.shared.regs.lock().levels();
            self.shared.drive(levels);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        w.write_u8(regs.tocr)?;
        w.write_u8(regs.tstr)?;
        for ch in &regs.ch {
            w.write_u32(ch.tcor)?;
            w.write_u32(ch.count)?;
            w.write_u64(ch.anchor)?;
            w.write_u16(ch.tcr)?;
        }
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = Regs::RESET;
        regs.tocr = r.read_u8()? & TOCR_MASK;
        regs.tstr = r.read_u8()? & TSTR_MASK;
        for ch in &mut regs.ch {
            ch.tcor = r.read_u32()?;
            ch.count = r.read_u32()?;
            ch.anchor = r.read_u64()?;
            ch.tcr = r.read_u16()?;
        }
        let tick = r.read_u64()?;
        let levels = {
            let mut slot = self.shared.regs.lock();
            *slot = regs;
            self.shared.tick.store(tick, Ordering::Relaxed);
            // The next-event tick is derived state and follows the load.
            self.shared.publish(&slot);
            slot.levels()
        };
        self.shared.drive(levels);
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
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let next = self.shared.next_event.load(Ordering::Relaxed);
        (next != NEVER).then_some(next)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Tmu {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Tmu::new(props)?)))
}

/// What the validator should know about `rcar.tmu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    let mut schema = ClassSchema::new(CLASS_NAME).region("").region("regs");
    for pin in IRQ_PINS {
        schema = schema.port(pin, PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU32;
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    fn tcor(n: u64) -> u64 {
        0x08 + 0x0c * n
    }
    fn tcnt_off(n: u64) -> u64 {
        0x0c + 0x0c * n
    }
    fn tcr(n: u64) -> u64 {
        0x10 + 0x0c * n
    }

    fn port(t: &Tmu) -> Port {
        Port {
            shared: Arc::clone(&t.shared),
        }
    }

    fn read(t: &Tmu, offset: u64, len: usize) -> u32 {
        let mut buf = [0u8; 4];
        port(t)
            .read(offset, &mut buf[..len], MemAttrs::DEFAULT)
            .expect("a register read is legal");
        u32::from_le_bytes(buf)
    }

    fn write(t: &Tmu, offset: u64, len: usize, value: u32) {
        port(t)
            .write(offset, &value.to_le_bytes()[..len], MemAttrs::DEFAULT)
            .expect("a register write is legal");
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    fn with_irqs() -> (Tmu, Vec<Arc<Probe>>) {
        let tmu = Tmu::with_defaults();
        let ids = WireIdAllocator::new();
        let mut probes = Vec::new();
        for pin in IRQ_PINS {
            let id = ids.alloc();
            let probe = Arc::new(Probe::default());
            let wire = Wire::builder()
                .source(id)
                .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            tmu.connect(pin, WireSource::new(wire, id)).unwrap();
            probes.push(probe);
        }
        (tmu, probes)
    }

    fn level(p: &Probe) -> u32 {
        p.level.load(Ordering::Relaxed)
    }

    #[test]
    fn registers_come_out_of_reset_with_the_manual_values() {
        let t = Tmu::with_defaults();
        assert_eq!(read(&t, TOCR, 1), 0);
        assert_eq!(read(&t, TSTR, 1), 0);
        for n in 0..3 {
            assert_eq!(read(&t, tcor(n), 4), u32::MAX);
            assert_eq!(read(&t, tcnt_off(n), 4), u32::MAX);
            assert_eq!(read(&t, tcr(n), 2), 0);
        }
        assert_eq!(read(&t, TCPR2, 4), 0);
        assert_eq!(t.next_event_tick(), None);
    }

    #[test]
    fn the_underflow_period_is_tcor_plus_one_count_clocks_for_every_prescaler() {
        for (tpsc, div) in [(0u32, 4u64), (1, 16), (2, 64), (3, 256), (4, 1024)] {
            let (t, probes) = with_irqs();
            write(&t, tcor(0), 4, 9);
            write(&t, tcnt_off(0), 4, 9);
            write(&t, tcr(0), 2, u32::from(TCR_UNIE) | tpsc);
            write(&t, TSTR, 1, 1);
            assert_eq!(t.next_event_tick(), Some(10 * div), "TPSC {tpsc}");
            t.advance_to(10 * div - 1);
            assert_eq!(read(&t, tcr(0), 2) & u32::from(TCR_UNF), 0);
            assert_eq!(t.tcnt(0), 0, "at zero, one clock short");
            assert_eq!(level(&probes[0]), 0);
            t.advance_to(10 * div);
            assert_ne!(read(&t, tcr(0), 2) & u32::from(TCR_UNF), 0);
            assert_eq!(t.tcnt(0), 9, "reloaded from TCOR");
            assert_eq!(level(&probes[0]), 1);
        }
    }

    #[test]
    fn tcnt_reads_mid_period_come_from_the_arithmetic() {
        let t = Tmu::with_defaults();
        write(&t, tcor(1), 4, 1000);
        write(&t, tcnt_off(1), 4, 1000);
        write(&t, tcr(1), 2, 1); // Pφ/16
        write(&t, TSTR, 1, 2);
        t.advance_to(16 * 3 + 15);
        assert_eq!(read(&t, tcnt_off(1), 4), 997);
        t.advance_to(16 * 4);
        assert_eq!(read(&t, tcnt_off(1), 4), 996);
        // Halfword reads of the counter give its halves.
        assert_eq!(read(&t, tcnt_off(1), 2), 996);
        assert_eq!(read(&t, tcnt_off(1) + 2, 2), 0);
        assert_eq!(t.next_event_tick(), None, "UNIE is clear");
    }

    #[test]
    fn reload_repeats_and_many_underflows_in_one_span_set_unf_once() {
        let t = Tmu::with_defaults();
        write(&t, tcor(2), 4, 4);
        write(&t, tcnt_off(2), 4, 4);
        write(&t, TSTR, 1, 4); // Pφ/4, period 5 × 4 = 20 ticks
        t.advance_to(20);
        assert_eq!(t.tcnt(2), 4);
        t.advance_to(24);
        assert_eq!(t.tcnt(2), 3);
        t.advance_to(20 * 7 + 8);
        assert_eq!(t.tcnt(2), 2);
        assert_ne!(read(&t, tcr(2), 2) & u32::from(TCR_UNF), 0);
    }

    #[test]
    fn unf_clears_on_a_written_zero_and_ignores_a_one() {
        let (t, probes) = with_irqs();
        write(&t, tcor(0), 4, 0);
        write(&t, tcnt_off(0), 4, 0);
        write(&t, tcr(0), 2, u32::from(TCR_UNIE));
        write(&t, TSTR, 1, 1);
        t.advance_to(4);
        assert_eq!(level(&probes[0]), 1);
        assert_eq!(t.next_event_tick(), None, "the line is already up");
        // Writing 1 to UNF (with UNIE) leaves it.
        write(&t, tcr(0), 2, u32::from(TCR_UNF | TCR_UNIE));
        assert_eq!(level(&probes[0]), 1);
        // A byte write of 0 to the high byte clears UNF alone.
        write(&t, tcr(0) + 1, 1, 0);
        assert_eq!(read(&t, tcr(0), 2), u32::from(TCR_UNIE));
        assert_eq!(level(&probes[0]), 0);
        assert_eq!(
            t.next_event_tick(),
            Some(8),
            "and the next underflow is scheduled"
        );
        assert_eq!(level(&probes[1]), 0);
        assert_eq!(level(&probes[2]), 0);
    }

    #[test]
    fn tstr_stops_and_starts_a_channel() {
        let t = Tmu::with_defaults();
        write(&t, tcnt_off(0), 4, 100);
        write(&t, TSTR, 1, 1);
        t.advance_to(4 * 10 + 2);
        write(&t, TSTR, 1, 0);
        assert_eq!(read(&t, tcnt_off(0), 4), 90);
        t.advance_to(10_000);
        assert_eq!(read(&t, tcnt_off(0), 4), 90, "stopped");
        write(&t, TSTR, 1, 1);
        t.advance_to(10_000 + 3);
        assert_eq!(read(&t, tcnt_off(0), 4), 90, "the prescaler restarted");
        t.advance_to(10_000 + 4);
        assert_eq!(read(&t, tcnt_off(0), 4), 89);
    }

    #[test]
    fn a_tcnt_write_while_running_keeps_the_prescaler_phase() {
        let t = Tmu::with_defaults();
        write(&t, tcr(0), 2, 1); // /16
        write(&t, TSTR, 1, 1);
        t.advance_to(16 * 5 + 10);
        write(&t, tcnt_off(0), 4, 50);
        t.advance_to(16 * 6 - 1);
        assert_eq!(t.tcnt(0), 50);
        t.advance_to(16 * 6);
        assert_eq!(t.tcnt(0), 49, "the next clock came on the old phase");
    }

    #[test]
    fn an_external_clock_selection_does_not_count() {
        let t = Tmu::with_defaults();
        write(&t, tcr(0), 2, 7);
        write(&t, TSTR, 1, 1);
        t.advance_to(1 << 20);
        assert_eq!(t.tcnt(0), u32::MAX);
    }

    #[test]
    fn a_debug_access_reads_without_effect_and_cannot_write() {
        let t = Tmu::with_defaults();
        write(&t, TSTR, 1, 1);
        t.advance_to(8);
        let mut b = [0u8; 4];
        port(&t).read(tcnt_off(0), &mut b, MemAttrs::DEBUG).unwrap();
        assert_eq!(u32::from_le_bytes(b), u32::MAX - 2);
        assert!(port(&t).write(TSTR, &[0], MemAttrs::DEBUG).is_err());
        assert_eq!(read(&t, TSTR, 1), 1);
    }

    fn snapshot(t: &Tmu) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("tmu", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("tmu", CLASS.name, CLASS.version).unwrap();
            t.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = Tmu::with_defaults();
        write(&saved, tcor(1), 4, 77);
        write(&saved, tcr(1), 2, u32::from(TCR_UNIE) | 2);
        write(&saved, TOCR, 1, 1);
        write(&saved, TSTR, 1, 2);
        saved.advance_to(64 * 3 + 5);
        let bytes = snapshot(&saved);

        let restored = Tmu::with_defaults();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("tmu", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();
        let want = *saved.shared.regs.lock();
        assert_eq!(*restored.shared.regs.lock(), want);
        assert_eq!(snapshot(&restored), bytes, "identical state bytes");
        assert_eq!(restored.next_event_tick(), saved.next_event_tick());
        // And both carry on identically.
        saved.advance_to(100_000);
        restored.advance_to(100_000);
        assert_eq!(restored.tcnt(1), saved.tcnt(1));
        assert_eq!(restored.irq_asserted(1), saved.irq_asserted(1));
    }

    #[test]
    fn a_board_binds_the_clock_and_the_scheduler_raises_the_underflow() {
        use crate::core::clock::GlobalTime;
        use crate::core::value::Width;

        let mut options = crate::machine::BuildOptions::new();
        options.classes.insert(schema());
        for s in crate::machine::builtin::schemas() {
            options.classes.insert(s);
        }
        bind(&mut options.bindings).expect("nothing else claims rcar.tmu");
        crate::machine::builtin::bind(&mut options.bindings).expect("ram");
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("nothing else claims rcar.tmu");
        crate::machine::builtin::register(&mut registry).expect("ram");

        // `clock` is Pφ itself: 64 MHz here, so Pφ/4 with TCOR = 15999 is a
        // millisecond.
        let text = concat!(
            "machine \"m\" {\n",
            "  osc p = 64000000 Hz\n",
            "  space mem { width = 32 }\n",
            "  object sram \"ram\" { size = 4K }\n",
            "  object tmu0 \"rcar.tmu\" { clock = p }\n",
            "  map mem 0x20000000 size 4K = sram\n",
            "  map mem 0xffd80000 size 0x30 = tmu0\n",
            "}\n"
        );
        let mut machine = crate::machine::build("t.machine", text, &registry, &options)
            .expect("the board builds");
        let space = Arc::clone(machine.space("mem").expect("mem"));
        let base = 0xffd8_0000u64;
        let poke = |offset: u64, width: Width, value: u32| {
            space
                .write(base + offset, width, u64::from(value), MemAttrs::DEFAULT)
                .expect("a write reaches the TMU");
        };
        let peek = |offset: u64, width: Width| -> u32 {
            space
                .read(base + offset, width, MemAttrs::DEFAULT)
                .expect("a read reaches the TMU") as u32
        };
        poke(tcor(0), Width::U32, 15_999);
        poke(tcnt_off(0), Width::U32, 15_999);
        poke(tcr(0), Width::U16, u32::from(TCR_UNIE));
        poke(TSTR, Width::U8, 1);

        machine
            .run_for(GlobalTime::from_nanos(500_000))
            .expect("the machine runs");
        assert_eq!(peek(tcr(0), Width::U16) & u32::from(TCR_UNF), 0);
        let cnt = peek(tcnt_off(0), Width::U32);
        assert!(
            (7_990..=8_010).contains(&cnt),
            "half a millisecond in: {cnt}"
        );

        machine
            .run_for(GlobalTime::from_nanos(500_100))
            .expect("the machine runs");
        assert_ne!(
            peek(tcr(0), Width::U16) & u32::from(TCR_UNF),
            0,
            "the underflow landed"
        );
    }

    #[test]
    fn properties_are_refused() {
        assert!(Tmu::new(&Props::new()).is_ok());
        assert!(Tmu::new(&Props::new().with("clock-hz", 1u64)).is_err());
    }
}
