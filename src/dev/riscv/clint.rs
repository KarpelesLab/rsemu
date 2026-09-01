//! The CLINT: `mtime`, one `mtimecmp` per hart, and software interrupts.
//!
//! # Sources
//!
//! * *The RISC-V Instruction Set Manual, Volume II: Privileged Architecture*
//!   (CC-BY-4.0) for what the registers *mean*: `mtime` is a real-time counter
//!   incrementing at a constant rate, and a machine timer interrupt is pending
//!   whenever `mtime >= mtimecmp`. Writing `mtimecmp` is the only way to clear
//!   it — there is no acknowledge bit, which is why a timer handler that
//!   forgets to reprogram the comparator livelocks.
//! * *RISC-V Advanced Core Local Interruptor Specification* (the ACLINT
//!   document) for the register layout, which is the layout the older SiFive
//!   CLINT already had and which every RISC-V board reuses: `MSIP` at offset 0,
//!   `MTIMECMP` at `0x4000`, `MTIME` at `0xBFF8`.
//!
//! # Time comes from a clock domain, never from the host
//!
//! `ROADMAP.md` §0 forbids a device reading a wall clock, and this device is
//! the one most tempted to. `mtime` is therefore
//! `domain_ticks + offset`: the machine file rates a clock domain at the
//! board's RTC frequency, the scheduler advances it, and a guest write to
//! `mtime` moves `offset` rather than the counter. On real hardware that
//! domain is a separate crystal from the core clock, which is why
//! `machines/riscv-virt.machine` declares a second oscillator for it rather
//! than dividing the core's.
//!
//! # Why it is a lazily advanced device
//!
//! A timer is the archetype of §4.2's *sampled* behaviour: a guest reads
//! `mtime` at an arbitrary instant and must see the value at that instant, not
//! the one at the last quantum boundary. So the CLINT holds its own tick and is
//! caught up before any access is dispatched to it
//! ([`Device::is_lazy`]), and it reports the tick
//! its next comparator fires on so the run loop stops the harts there rather
//! than thousands of cycles past it.
//!
//! # The hart's `time` CSR reads this counter
//!
//! `time` (`0xc01`, what `rdtime` returns) is architecturally a read-only view
//! of the memory-mapped `mtime` that lives here, not a counter the hart owns.
//! So this block publishes `mtime` as
//! [`ExportId::TIMEBASE`](crate::core::device::ExportId::TIMEBASE) and a hart
//! that names it — `timer = clint` in the machine file — holds the same cell.
//!
//! That wiring was missing until the [`Device::export`] seam existed, and the
//! symptom was not subtle: an operating system taking its clocksource from
//! `rdtime` computes every deadline as `0 + delta`, which `mtime` is already
//! past, so it takes a timer interrupt immediately, reprograms the comparator,
//! and never advances. A live-lock with a complete console log up to the point
//! the clocksource is first used. `src/dev/riscv/tests.rs` boots a kernel far
//! enough to have shown it.
//!
//! [`Clint::mtime_cell`] is still the direct route, for a hand-wired machine
//! and for tests. [`Hart::set_time`](crate::cpu::riscv::Hart::set_time) remains
//! for a board with no CLINT at all; a hart with a timer attached overwrites it
//! on the next step, which is the right precedence.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

use super::dt::{DtSource, NodeKind, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "riscv.clint";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block occupies.
///
/// 64 KiB, because `MTIME` sits at `0xBFF8` and the block is conventionally
/// rounded up from there.
pub const REGISTER_WINDOW_LEN: u64 = 0x1_0000;

/// `MSIP` base: one 32-bit register per hart, only bit 0 implemented.
const MSIP_BASE: u64 = 0x0000;

/// `MTIMECMP` base: one 64-bit comparator per hart.
const MTIMECMP_BASE: u64 = 0x4000;

/// `MTIME`: the counter itself, one 64-bit register for the whole block.
const MTIME_OFFSET: u64 = 0xbff8;

/// The largest hart count a `machines/*.machine` file may ask for.
///
/// Not an architectural limit — it is the point past which the register block
/// would run into `MTIME`, since the comparators start at `0x4000` and take
/// eight bytes each.
pub const MAX_HARTS: u64 = (MTIME_OFFSET - MTIMECMP_BASE) / 8;

/// The rate `mtime` counts at when a machine file does not say.
///
/// 10 MHz is the conventional RISC-V board timebase; the machine file declares
/// the same number as an oscillator, and they must agree — see
/// [`Clint::new`].
pub const DEFAULT_TIMEBASE_HZ: u32 = 10_000_000;

/// Everything the guest can change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// One comparator per hart. `u64::MAX` is the reset value, which is what
    /// stops a machine coming up with its timer already firing.
    mtimecmp: Vec<u64>,
    /// One software-interrupt bit per hart.
    msip: Vec<bool>,
    /// `mtime` is `tick + offset`, so a guest write moves the origin rather
    /// than the clock domain — which nothing below `host/` may do.
    offset: u64,
    /// The domain tick the block has been advanced to. Mirrored into
    /// [`Registers::tick`], which is what the lock-free accessors read.
    tick: u64,
}

impl State {
    fn new(harts: usize) -> State {
        State {
            mtimecmp: alloc::vec![u64::MAX; harts],
            msip: alloc::vec![false; harts],
            offset: 0,
            tick: 0,
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The output pins, at [`LockRank::LEAF`] so it may be taken while nothing
    /// else is held. Two per hart: the timer line and the software line.
    outs: Mutex<Outputs>,
    /// The catch-up handle the read and write paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// Published so [`Device::current_tick`] can answer without a lock — the
    /// scheduler asks it with its own slot lock held at
    /// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF).
    tick: AtomicU64,
    /// The next tick some comparator fires on, or [`u64::MAX`] for none. Same
    /// no-lock rule.
    next_event: AtomicU64,
    /// `mtime` as anything outside this device would read it, published on
    /// every change. See the module docs: this is what a hart's `time` CSR
    /// should be reading, and the half of that wiring this module can supply.
    mtime_cell: Arc<AtomicU64>,
    harts: usize,
    timebase_hz: u32,
}

/// The wires this block drives, once the machine has built them.
#[derive(Debug, Default)]
struct Outputs {
    /// `mtip0`, `mtip1`, … — the machine timer line of each hart.
    mtip: Vec<Option<WireSource>>,
    /// `msip0`, `msip1`, … — the machine software line of each hart.
    msip: Vec<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("harts", &self.harts)
            .field("timebase_hz", &self.timebase_hz)
            .field("tick", &self.tick.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// The core-local interruptor.
#[derive(Debug)]
pub struct Clint {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Clint {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a hart count the register block cannot hold, or
    /// for a property this class does not know.
    pub fn new(props: &Props) -> Result<Clint> {
        let mut r = props.reader();
        let harts = r.or_range("harts", 1u64, 1..=MAX_HARTS)?;
        let timebase = r.or_range(
            "timebase",
            u64::from(DEFAULT_TIMEBASE_HZ),
            1..=u64::from(u32::MAX),
        )?;
        r.finish()?;
        Ok(Clint::with_harts(harts as usize, timebase as u32))
    }

    /// Build one directly, for a test or a hand-wired machine.
    #[must_use]
    pub fn with_harts(harts: usize, timebase_hz: u32) -> Clint {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(harts)),
            outs: Mutex::with_rank(
                LockRank::LEAF,
                Outputs {
                    mtip: alloc::vec![None; harts],
                    msip: alloc::vec![None; harts],
                },
            ),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            mtime_cell: Arc::new(AtomicU64::new(0)),
            harts,
            timebase_hz,
        });
        let region: RegionRef = Arc::new(Region::io(
            "riscv.clint",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Clint { regs, region }
    }

    /// How many harts this block serves.
    #[must_use]
    pub fn harts(&self) -> usize {
        self.regs.harts
    }

    /// The rate the machine file says `mtime` counts at.
    #[must_use]
    pub fn timebase_hz(&self) -> u32 {
        self.regs.timebase_hz
    }

    /// The cell holding `mtime`, updated on every advance and every write.
    ///
    /// The seam described in the module docs. A hart handed this would read the
    /// real platform timer from `rdtime` instead of a field nothing advances;
    /// nothing does yet, and this returns the same number
    /// [`mtime`](Clint::mtime) does, without taking the device's lock.
    #[must_use]
    pub fn mtime_cell(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.regs.mtime_cell)
    }

    /// `mtime` as the guest would read it.
    #[must_use]
    pub fn mtime(&self) -> u64 {
        let state = self.regs.state.lock();
        state.tick.wrapping_add(state.offset)
    }

    /// One hart's comparator.
    #[must_use]
    pub fn mtimecmp(&self, hart: usize) -> u64 {
        self.regs
            .state
            .lock()
            .mtimecmp
            .get(hart)
            .copied()
            .unwrap_or(u64::MAX)
    }

    /// Whether a hart's software interrupt bit is set.
    #[must_use]
    pub fn msip(&self, hart: usize) -> bool {
        self.regs
            .state
            .lock()
            .msip
            .get(hart)
            .copied()
            .unwrap_or(false)
    }

    /// Advance to `tick` of the CLINT's own clock domain, driving whatever
    /// timer lines that makes pending.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }
}

impl Registers {
    /// `mtime` for a state already locked.
    fn now(state: &State) -> u64 {
        state.tick.wrapping_add(state.offset)
    }

    /// Recompute the published tick and next-event values, and report which
    /// timer lines should now be asserted.
    ///
    /// Called with the state lock held; the wires are driven by the caller
    /// after it releases, which is §4.7's re-entrancy contract.
    fn republish(&self, state: &State) -> Vec<bool> {
        self.tick.store(state.tick, Ordering::Relaxed);
        let now = Self::now(state);
        self.mtime_cell.store(now, Ordering::Relaxed);
        let mut pending = Vec::with_capacity(self.harts);
        let mut soonest = u64::MAX;
        for cmp in &state.mtimecmp {
            pending.push(now >= *cmp);
            if now < *cmp {
                // The tick at which `tick + offset` reaches `cmp`. Strictly in
                // the future by construction, which is what
                // `Device::next_event_tick` requires.
                let at = cmp.wrapping_sub(state.offset);
                if at < soonest {
                    soonest = at;
                }
            }
        }
        self.next_event.store(soonest, Ordering::Relaxed);
        pending
    }

    /// Drive the timer lines. Never called with the state lock held.
    fn drive_mtip(&self, pending: &[bool]) {
        let sources: Vec<Option<WireSource>> = self.outs.lock().mtip.clone();
        for (source, on) in sources.iter().zip(pending) {
            if let Some(source) = source {
                source.set(Level::from_bool(*on));
            }
        }
    }

    /// Drive one software line. Never called with the state lock held.
    fn drive_msip(&self, hart: usize, on: bool) {
        let source = self.outs.lock().msip.get(hart).and_then(Clone::clone);
        if let Some(source) = source {
            source.set(Level::from_bool(on));
        }
    }

    fn advance_to(&self, tick: u64) {
        let pending = {
            let mut state = self.state.lock();
            if tick <= state.tick {
                return;
            }
            state.tick = tick;
            self.republish(&state)
        };
        self.drive_mtip(&pending);
    }

    /// Catch up before an access, exactly as the PPU does (§4.2).
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
        // A refusal means catch-up is already running further up the stack.
        // The access still has to be answered, and answering it from where the
        // block stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Read one 64-bit register value for `offset`, in register coordinates.
    fn read_reg(&self, offset: u64) -> Option<u64> {
        let state = self.state.lock();
        if offset == MTIME_OFFSET {
            return Some(Self::now(&state));
        }
        if offset >= MTIMECMP_BASE {
            let index = (offset - MTIMECMP_BASE) / 8;
            return state.mtimecmp.get(index as usize).copied();
        }
        let index = (offset - MSIP_BASE) / 4;
        state
            .msip
            .get(index as usize)
            .map(|on| u64::from(u32::from(*on)))
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !attrs.debug {
            self.sync(attrs);
        }
        let (width, aligned) = decode(offset, dst.len())?;
        let Some(value) = self.read_reg(aligned) else {
            // Inside the window but past the last hart: reads as zero, which
            // is what an unimplemented register in a decoded block does.
            dst.fill(0);
            return Ok(());
        };
        // A 32-bit access to a 64-bit register reads the half it addressed;
        // RV32 software does exactly this to `mtime`.
        let shift = if width == Width::U32 && offset % 8 == 4 {
            32
        } else {
            0
        };
        let value = value >> shift;
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = (value >> (8 * i)) as u8;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A debug write to `mtimecmp` would change when the guest's next
            // timer interrupt lands, which is not something the core can make
            // harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let (width, aligned) = decode(offset, src.len())?;
        let mut incoming = 0u64;
        for (i, byte) in src.iter().enumerate() {
            incoming |= u64::from(*byte) << (8 * i);
        }
        let half = width == Width::U32 && offset % 8 == 4;

        // Software interrupts are a separate line per hart, so they are driven
        // one at a time rather than through the timer sweep.
        if aligned < MTIMECMP_BASE {
            let index = ((aligned - MSIP_BASE) / 4) as usize;
            let on = incoming & 1 != 0;
            let changed = {
                let mut state = self.state.lock();
                match state.msip.get_mut(index) {
                    Some(slot) if *slot != on => {
                        *slot = on;
                        true
                    }
                    _ => false,
                }
            };
            if changed {
                self.drive_msip(index, on);
            }
            return Ok(());
        }

        let pending = {
            let mut state = self.state.lock();
            if aligned == MTIME_OFFSET {
                // Writing `mtime` moves the origin. The clock domain keeps
                // counting at its own rate, which is the only rate there is.
                let now = Registers::now(&state);
                let wanted = merge(now, incoming, width, half);
                state.offset = wanted.wrapping_sub(state.tick);
            } else {
                let index = ((aligned - MTIMECMP_BASE) / 8) as usize;
                let Some(slot) = state.mtimecmp.get(index).copied() else {
                    return Ok(());
                };
                state.mtimecmp[index] = merge(slot, incoming, width, half);
            }
            self.republish(&state)
        };
        self.drive_mtip(&pending);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U32,
            max: Width::U64,
            natural_alignment: true,
            endian: Endian::Little,
            allow_bulk: false,
            secure_only: false,
            privileged_only: false,
            drives_data_bus: true,
        }
    }
}

/// Fold a 32- or 64-bit write into an existing 64-bit register value.
fn merge(old: u64, incoming: u64, width: Width, high_half: bool) -> u64 {
    match (width, high_half) {
        (Width::U64, _) => incoming,
        (_, false) => (old & 0xffff_ffff_0000_0000) | (incoming & 0xffff_ffff),
        (_, true) => (old & 0xffff_ffff) | (incoming << 32),
    }
}

/// Check an access and reduce it to a width and a register-aligned offset.
fn decode(offset: u64, len: usize) -> MemResult<(Width, u64)> {
    let width = Width::from_bytes(len as u64).ok_or(BusError::BadAccess)?;
    match width {
        Width::U32 => {
            if !offset.is_multiple_of(4) {
                return Err(BusError::BadAccess);
            }
            // A 32-bit access to the high half of a 64-bit register addresses
            // `base + 4`; the register itself is at `base`.
            let aligned = if offset >= MTIMECMP_BASE {
                offset & !7
            } else {
                offset
            };
            Ok((width, aligned))
        }
        Width::U64 => {
            if !offset.is_multiple_of(8) || offset < MTIMECMP_BASE {
                // The `MSIP` registers are 32 bits wide; a 64-bit access there
                // would span two harts.
                return Err(BusError::BadAccess);
            }
            Ok((width, offset))
        }
        _ => Err(BusError::BadAccess),
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        NodeSpec {
            kind: NodeKind::Clint {
                timebase_hz: self.timebase_hz,
            },
            name: "clint",
            // Both spellings: the older one every RISC-V kernel still matches
            // on, and the generic one.
            compatible: &["sifive,clint0", "riscv,clint0"],
            cells: Vec::new(),
            strings: Vec::new(),
            irq_wire: None,
        }
    }
}

/// The `riscv.clint` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "RISC-V core-local interruptor: mtime, per-hart mtimecmp and software interrupts",
    properties: &[
        PropertySpec {
            name: "harts",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many harts it serves, one comparator each (default 1)",
        },
        PropertySpec {
            name: "timebase",
            kind: ValueKind::Uint,
            required: false,
            summary: "the rate mtime counts at, in Hz, as the device tree reports it",
        },
    ],
    construct: |props| Ok(Box::new(Clint::new(props)?)),
};

impl Device for Clint {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A `map` statement places the region; this says what the region *is*,
        // for the board's device-tree generator (`super::dt`). Announcing
        // yourself into a table a sibling reads is a realize-time action.
        super::dt::publish(
            ctx.hosts(),
            &self.region,
            Arc::downgrade(&self.regs) as Weak<dyn DtSource>,
        )
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: there is no battery behind `mtime` on this board, and a
        // comparator that survived a reset would fire into a kernel that had
        // not programmed it.
        let pending = {
            let mut state = self.regs.state.lock();
            *state = State::new(self.regs.harts);
            self.regs.republish(&state)
        };
        self.regs.drive_mtip(&pending);
        for hart in 0..self.regs.harts {
            self.regs.drive_msip(hart, false);
        }
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    /// Publish `mtime` as the platform timebase.
    ///
    /// The cell is allocated in [`Clint::with_harts`] and never replaced, so
    /// this is answerable from construction and a hart holding it keeps
    /// holding it across a reset — the handle is wiring, not guest state.
    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::TIMEBASE).then(|| Export::Cell(self.mtime_cell()))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let (kind, hart) = split_pin(port).ok_or_else(|| unknown_pin(port))?;
        let mut outs = self.regs.outs.lock();
        let slot = match kind {
            "mtip" => outs.mtip.get_mut(hart),
            "msip" => outs.msip.get_mut(hart),
            _ => None,
        }
        .ok_or_else(|| unknown_pin(port))?;
        *slot = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some((kind, hart)) = split_pin(port) else {
            return;
        };
        let state = self.regs.state.lock();
        let level = match kind {
            "mtip" => {
                Registers::now(&state) >= state.mtimecmp.get(hart).copied().unwrap_or(u64::MAX)
            }
            "msip" => state.msip.get(hart).copied().unwrap_or(false),
            _ => return,
        };
        drop(state);
        match kind {
            "mtip" => {
                let source = self.regs.outs.lock().mtip.get(hart).and_then(Clone::clone);
                if let Some(source) = source {
                    source.set(Level::from_bool(level));
                }
            }
            _ => self.regs.drive_msip(hart, level),
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
        w.write_seq_len(state.mtimecmp.len() as u64)?;
        for cmp in &state.mtimecmp {
            w.write_u64(*cmp)?;
        }
        for on in &state.msip {
            w.write_bool(*on)?;
        }
        w.write_u64(state.offset)?;
        // The tick is this device's own position in its domain. The scheduler
        // restores the domain; without this the two would disagree and the
        // block would stand still until the domain caught up with it.
        w.write_u64(state.tick)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let count = r.read_seq_len(9)? as usize;
        if count != self.regs.harts {
            return Err(Error::State(format!(
                "snapshot has {count} hart(s) of CLINT state, this block serves {}",
                self.regs.harts
            )));
        }
        let mut state = State::new(self.regs.harts);
        for slot in &mut state.mtimecmp {
            *slot = r.read_u64()?;
        }
        for slot in &mut state.msip {
            *slot = r.read_bool()?;
        }
        state.offset = r.read_u64()?;
        state.tick = r.read_u64()?;
        let (pending, msip) = {
            let mut live = self.regs.state.lock();
            *live = state;
            (self.regs.republish(&live), live.msip.clone())
        };
        self.regs.drive_mtip(&pending);
        for (hart, on) in msip.iter().enumerate() {
            self.regs.drive_msip(hart, *on);
        }
        Ok(())
    }
}

impl Instance for Clint {}

/// Split `mtip3` into `("mtip", 3)`.
fn split_pin(port: &str) -> Option<(&str, usize)> {
    for prefix in ["mtip", "msip"] {
        if let Some(rest) = port.strip_prefix(prefix) {
            return rest.parse::<usize>().ok().map(|n| (prefix, n));
        }
    }
    None
}

/// The error for a pin this block does not drive.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: format!(
            "the CLINT drives `mtip<hart>` and `msip<hart>`; `{port}` is neither, or names a \
             hart this block does not serve"
        ),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Clint::new(props)?)))
}

/// What the validator should know about `riscv.clint`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("harts", ValueKind::Uint).range(1, MAX_HARTS))
        .prop(PropSchema::new("timebase", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .region("")
        .region("regs");
    // One pair of pins per hart the block could serve. Declared up to the
    // block's own limit rather than to the instance's hart count: a schema is
    // per class, and the class does not know how a given file configured it.
    for hart in 0..MAX_HARTS.min(8) {
        s = s
            .port(format!("mtip{hart}"), PortDir::Out)
            .port(format!("msip{hart}"), PortDir::Out);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

    fn clint() -> Clint {
        Clint::with_harts(2, DEFAULT_TIMEBASE_HZ)
    }

    fn read64(c: &Clint, offset: u64) -> u64 {
        let mut bytes = [0u8; 8];
        c.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a 64-bit read is legal");
        u64::from_le_bytes(bytes)
    }

    fn write64(c: &Clint, offset: u64, value: u64) {
        c.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a 64-bit write is legal");
    }

    fn write32(c: &Clint, offset: u64, value: u32) {
        c.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a 32-bit write is legal");
    }

    /// A sink that just records the last level it was told.
    #[derive(Debug, Default)]
    struct Probe {
        level: crate::core::sync::AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    /// Wire `mtip0` to a probe.
    fn wired() -> (Clint, Arc<Probe>) {
        let clint = clint();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        clint
            .connect("mtip0", WireSource::new(wire, id))
            .expect("the block drives mtip0");
        (clint, probe)
    }

    #[test]
    fn the_published_cell_tracks_mtime() {
        // Half of the fix for the gap in the module docs. The other half is a
        // hart that reads this instead of its own untouched CSR field; until
        // that exists, at least the number is correct and reachable.
        let c = clint();
        let cell = c.mtime_cell();
        assert_eq!(cell.load(Ordering::Relaxed), 0);
        c.advance_to(77);
        assert_eq!(cell.load(Ordering::Relaxed), 77);
        write64(&c, MTIME_OFFSET, 9_000);
        assert_eq!(cell.load(Ordering::Relaxed), 9_000);
        assert_eq!(cell.load(Ordering::Relaxed), c.mtime());
    }

    #[test]
    fn mtime_counts_in_the_clock_domain_and_nothing_else() {
        let c = clint();
        assert_eq!(read64(&c, MTIME_OFFSET), 0);
        c.advance_to(1234);
        assert_eq!(read64(&c, MTIME_OFFSET), 1234);
        // Running backwards is a no-op, not an error: a stale catch-up target
        // must not rewind a counter the guest has already seen.
        c.advance_to(1000);
        assert_eq!(read64(&c, MTIME_OFFSET), 1234);
    }

    #[test]
    fn writing_mtime_moves_the_origin_rather_than_the_clock() {
        let c = clint();
        c.advance_to(100);
        write64(&c, MTIME_OFFSET, 5_000);
        assert_eq!(c.mtime(), 5_000);
        c.advance_to(150);
        assert_eq!(c.mtime(), 5_050, "and it keeps counting from there");
    }

    #[test]
    fn the_timer_line_follows_the_comparator_in_both_directions() {
        let (c, probe) = wired();
        assert!(!probe.high(), "a fresh comparator is u64::MAX");

        write64(&c, MTIMECMP_BASE, 50);
        assert!(!probe.high(), "not yet");
        c.advance_to(49);
        assert!(!probe.high());
        c.advance_to(50);
        assert!(probe.high(), "mtime >= mtimecmp");

        // The only way to clear it is to reprogram the comparator — there is
        // no acknowledge bit, and a handler that forgets this livelocks.
        write64(&c, MTIMECMP_BASE, 1_000);
        assert!(!probe.high());
    }

    #[test]
    fn the_next_event_is_the_soonest_comparator_and_never_the_past() {
        let c = clint();
        assert_eq!(Device::next_event_tick(&c), None, "nothing programmed");
        write64(&c, MTIMECMP_BASE, 900);
        write64(&c, MTIMECMP_BASE + 8, 400);
        assert_eq!(Device::next_event_tick(&c), Some(400));
        c.advance_to(400);
        assert_eq!(
            Device::next_event_tick(&c),
            Some(900),
            "the one that fired is not reported again"
        );
        c.advance_to(1_000);
        assert_eq!(Device::next_event_tick(&c), None);
    }

    #[test]
    fn a_32_bit_guest_reaches_both_halves_of_a_64_bit_register() {
        let c = clint();
        write32(&c, MTIMECMP_BASE, 0xdead_beef);
        write32(&c, MTIMECMP_BASE + 4, 0x0000_00ff);
        assert_eq!(c.mtimecmp(0), 0x0000_00ff_dead_beef);

        let mut half = [0u8; 4];
        c.regs
            .read(MTIMECMP_BASE + 4, &mut half, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(u32::from_le_bytes(half), 0x0000_00ff);
    }

    #[test]
    fn the_software_interrupt_is_one_bit_per_hart() {
        let c = clint();
        write32(&c, MSIP_BASE, 1);
        assert!(c.msip(0));
        assert!(!c.msip(1), "and it is not shared");
        // Only bit 0 is implemented.
        write32(&c, MSIP_BASE + 4, 0xffff_fffe);
        assert!(!c.msip(1));
        write32(&c, MSIP_BASE + 4, 0xffff_ffff);
        assert!(c.msip(1));
    }

    #[test]
    fn an_access_the_block_does_not_take_is_refused_rather_than_guessed_at() {
        let c = clint();
        // A byte write to a 32-bit register file.
        assert!(c.regs.write(0, &[1], MemAttrs::DEFAULT).is_err());
        // A 64-bit access spanning two harts' MSIP registers.
        assert!(c.regs.write(0, &[0u8; 8], MemAttrs::DEFAULT).is_err());
        // A misaligned comparator access.
        assert!(
            c.regs
                .write(MTIMECMP_BASE + 2, &[0u8; 4], MemAttrs::DEFAULT)
                .is_err()
        );
        // And a debug write, which cannot be made harmless.
        assert!(
            c.regs
                .write(MTIMECMP_BASE, &[0u8; 8], MemAttrs::DEBUG)
                .is_err()
        );
    }

    #[test]
    fn a_debug_read_advances_nothing_and_sees_the_present() {
        let c = clint();
        c.advance_to(7);
        let mut bytes = [0u8; 8];
        c.regs
            .read(MTIME_OFFSET, &mut bytes, MemAttrs::DEBUG)
            .expect("a debugger may look");
        assert_eq!(u64::from_le_bytes(bytes), 7);
    }

    #[test]
    fn a_snapshot_round_trips_every_register() {
        let saved = clint();
        saved.advance_to(4_242);
        write64(&saved, MTIMECMP_BASE, 9_000);
        write64(&saved, MTIMECMP_BASE + 8, 12);
        write32(&saved, MSIP_BASE + 4, 1);
        write64(&saved, MTIME_OFFSET, 1_000_000);

        let mut shape = MachineShape::new();
        shape.add_device("clint", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("clint", CLASS.name, CLASS.version).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = clint();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("clint", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(restored.mtime(), saved.mtime());
        assert_eq!(restored.mtimecmp(0), 9_000);
        assert_eq!(restored.mtimecmp(1), 12);
        assert!(restored.msip(1));
        assert_eq!(
            Device::current_tick(&restored),
            Device::current_tick(&saved),
            "the domain position comes back too"
        );
    }

    #[test]
    fn a_snapshot_from_a_differently_sized_block_is_refused() {
        let saved = Clint::with_harts(2, DEFAULT_TIMEBASE_HZ);
        let mut shape = MachineShape::new();
        shape.add_device("clint", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("clint", CLASS.name, CLASS.version).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let small = Clint::with_harts(1, DEFAULT_TIMEBASE_HZ);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("clint", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        let e = small.load(&mut chunk.reader()).unwrap_err().to_string();
        assert!(e.contains("2") && e.contains("1"), "{e}");
    }

    #[test]
    fn properties_are_checked_rather_than_clamped() {
        let ok = Clint::new(&Props::new().with("harts", 4u64)).expect("four harts");
        assert_eq!(ok.harts(), 4);
        assert!(Clint::new(&Props::new().with("harts", 0u64)).is_err());
        assert!(Clint::new(&Props::new().with("harts", MAX_HARTS + 1)).is_err());
        assert!(
            Clint::new(&Props::new().with("hartz", Value::Uint(1))).is_err(),
            "a typo is not silently ignored"
        );
    }

    #[test]
    fn a_reset_disarms_every_comparator() {
        let (c, probe) = wired();
        write64(&c, MTIMECMP_BASE, 0);
        c.advance_to(1);
        assert!(probe.high());
        c.reset(ResetKind::Cold);
        assert!(!probe.high());
        assert_eq!(c.mtime(), 0);
        assert_eq!(c.mtimecmp(0), u64::MAX);
    }
}
