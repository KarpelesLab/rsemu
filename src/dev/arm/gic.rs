//! The Generic Interrupt Controller, version 2: the distributor and the CPU
//! interface.
//!
//! # Source
//!
//! *ARM Generic Interrupt Controller Architecture Specification, version 2.0*
//! (ARM IHI 0048) — the programmers' model of chapter 4 (`GICD_*`), chapter 4
//! again for the CPU interface (`GICC_*`), and the interrupt handling and
//! prioritization of chapter 3. Nothing else was consulted and no driver of
//! any licence was read.
//!
//! # Why version 2 and not version 3
//!
//! `docs/platforms/arm64-virt.md` argues this at length; the short form is
//! that GICv2 is **entirely memory-mapped**, and GICv3 is not.
//!
//! * A GICv2 CPU interface is a register block at an address. A GICv3 one is a
//!   *system register* file — `ICC_IAR1_EL1`, `ICC_EOIR1_EL1`, `ICC_PMR_EL1`,
//!   `ICC_SRE_EL1` and a dozen more — which would put an interrupt controller
//!   inside `cpu.arm.a64`'s `MRS`/`MSR` path, where the board cannot reach it
//!   and where a board *without* a GIC would still carry it.
//! * GICv3 replaces the distributor's per-CPU half with a **redistributor per
//!   core**, at its own stride, holding the banked SGI and PPI state and the
//!   LPI configuration and pending tables. That is a second register file and
//!   a per-core address calculation before a single interrupt is delivered.
//! * GICv3 adds LPIs and, in practice, an ITS to route them — which is a
//!   command queue the controller DMA-walks. Message-signalled interrupts on
//!   this board have nothing to signal them.
//! * A `virt` board may legitimately present either. GICv2 supports up to
//!   eight CPUs and 1020 interrupt ids, which is more than this board has any
//!   use for, and a kernel that finds `arm,cortex-a15-gic` in its tree binds
//!   the driver it has had since 2012.
//!
//! GICv3 is the right thing to build the day this board wants more than eight
//! cores or wants MSIs. Until then it is a redistributor-per-core design
//! bought with nothing.
//!
//! # The three kinds of interrupt id
//!
//! ```text
//!   0  - 15   SGI   software generated, written to GICD_SGIR — banked per CPU
//!   16 - 31   PPI   private peripheral, one instance per CPU — banked per CPU
//!   32 - …    SPI   shared peripheral, one instance, routed by GICD_ITARGETSR
//! ```
//!
//! *Banked* is the word that carries the work: interrupt 27 is a different
//! interrupt on each core, with its own enable, its own pending bit and its own
//! priority, and a distributor that stored one copy of it would have two cores
//! sharing a timer. So the state below is `[[…; 32]; cpus]` for the low ids and
//! a flat array above them, which is the register map's own shape.
//!
//! # Why the generic timer arrives here at all
//!
//! `machines/a64-mini.machine` says the generic timer is *inside* the core and
//! goes to `IRQ` without crossing the board — which is true of that board,
//! because it has no interrupt controller for the timer to arrive at. On this
//! board it does: the timer is a private peripheral interrupt, the core drives
//! `cntv`/`cntp` out to `gic.ppi11`/`gic.ppi14`, and the GIC drives `nIRQ`
//! back. A kernel that has enabled PPI 27 in the distributor and then takes an
//! interrupt the distributor never saw reads `GICC_IAR`, is told 1023
//! (spurious), returns, and takes it again forever — which is the failure this
//! wiring exists to avoid, and it is a live-lock rather than a crash.
//!
//! # Locking
//!
//! One [`LockRank::DEVICE`] lock over the whole state, and the per-CPU output
//! lines at [`LockRank::LEAF`]. A core reaches `GICC_IAR` while holding its own
//! `BUS`-ranked execution lock, so everything here ranks below `BUS`
//! (`CLAUDE.md`, the ranked lock order); the outward call that follows —
//! driving `nIRQ` — is made with the state lock **released**, which is the
//! re-entrancy contract and is what stops a claim from re-entering the
//! distributor through the core it just interrupted.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

use super::dt::{DtSource, IntSpec, NodeKind, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.gic";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the distributor answers (IHI 0048 §4.1.2).
pub const DIST_WINDOW_LEN: u64 = 0x1000;

/// How much address space the CPU interface answers.
///
/// 8 KiB rather than 4: `GICC_DIR` lives at `0x1000`, past the first page, and
/// a kernel that finds a 4 KiB CPU interface in its device tree says so.
pub const CPU_WINDOW_LEN: u64 = 0x2000;

/// The first shared peripheral interrupt id.
pub const SPI_BASE: u32 = 32;

/// The first private peripheral interrupt id.
pub const PPI_BASE: u32 = 16;

/// How many interrupt ids are banked per CPU: the sixteen SGIs and the
/// sixteen PPIs.
const BANKED: usize = 32;

/// The id `GICC_IAR` returns when there is nothing to claim (IHI 0048 §3.2.4).
pub const SPURIOUS: u32 = 1023;

/// The largest interrupt id GICv2 has room for.
pub const MAX_INTID: u32 = 1020;

/// The most CPU interfaces a GICv2 has (`GICD_TYPER.CPUNumber` is three bits).
pub const MAX_CPUS: u64 = 8;

/// How many priority bits are implemented.
///
/// Five, so the low three bits of every priority read back as zero. A driver
/// discovers this by writing `0xff` to a priority register and reading it back,
/// and it matters: the number of implemented bits is how a kernel decides which
/// priority values it can use to preempt with.
const PRIORITY_MASK: u8 = 0xf8;

/// The priority a CPU interface reports when nothing is active — lower
/// priority than any real interrupt, because on a GIC bigger is weaker.
const IDLE_PRIORITY: u8 = 0xff;

/// `GICD_IIDR`: implementer Arm (JEP106 `0x43b`), revision 2.
const GICD_IIDR: u32 = 0x0200_043b;

/// `GICC_IIDR`: implementer Arm, architecture version 2 in bits 19:16.
const GICC_IIDR: u32 = 0x0002_043b;

/// What one interrupt's configuration says: level-sensitive or edge-triggered.
///
/// Two bits per interrupt in `GICD_ICFGR`, of which only the top one means
/// anything: `0b00` is level, `0b10` is edge (IHI 0048 §4.3.13).
const CFG_EDGE: u8 = 0b10;

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// `GICD_CTLR` bit 0: the distributor forwards nothing while it is clear.
    dist_enabled: bool,
    /// Per-CPU `GICC_CTLR` bit 0.
    cpu_enabled: Vec<bool>,
    /// Per-CPU `GICC_PMR`: an interrupt is only signalled if its priority is
    /// numerically *lower* than this.
    pmr: Vec<u8>,
    /// Per-CPU `GICC_BPR`, stored and reported. This model does not implement
    /// preemption groups, so nothing divides by it.
    bpr: Vec<u8>,
    /// Per-CPU stack of active priorities, innermost last. Empty means idle.
    running: Vec<Vec<u8>>,
    /// Per-CPU stack of active interrupt ids, parallel to `running`, so an
    /// `EOIR` for the wrong id can be refused.
    active_stack: Vec<Vec<u32>>,

    /// Banked enable bits: `banked_enabled[cpu][intid]` for `intid < 32`.
    banked_enabled: Vec<[bool; BANKED]>,
    /// Banked pending bits — the latch, not the input line.
    banked_pending: Vec<[bool; BANKED]>,
    /// Banked active bits.
    banked_active: Vec<[bool; BANKED]>,
    /// Banked priorities.
    banked_priority: Vec<[u8; BANKED]>,
    /// Banked configuration, two bits each, packed one interrupt per byte.
    banked_config: Vec<[u8; BANKED]>,
    /// What the PPI input lines are doing, per CPU. SGIs have no line.
    banked_line: Vec<[bool; BANKED]>,

    /// Shared enable bits, indexed by `intid - 32`.
    enabled: Vec<bool>,
    /// Shared pending latches.
    pending: Vec<bool>,
    /// Shared active bits.
    active: Vec<bool>,
    /// Shared priorities.
    priority: Vec<u8>,
    /// Shared target CPU masks, one bit per CPU interface.
    targets: Vec<u8>,
    /// Shared configuration, two bits each.
    config: Vec<u8>,
    /// What the SPI input lines are doing.
    line: Vec<bool>,
}

impl State {
    fn new(cpus: usize, spis: usize) -> State {
        State {
            dist_enabled: false,
            cpu_enabled: alloc::vec![false; cpus],
            pmr: alloc::vec![0; cpus],
            bpr: alloc::vec![2; cpus],
            running: alloc::vec![Vec::new(); cpus],
            active_stack: alloc::vec![Vec::new(); cpus],
            banked_enabled: alloc::vec![[false; BANKED]; cpus],
            banked_pending: alloc::vec![[false; BANKED]; cpus],
            banked_active: alloc::vec![[false; BANKED]; cpus],
            banked_priority: alloc::vec![[0u8; BANKED]; cpus],
            // PPIs come out of reset level-sensitive and SGIs edge-triggered,
            // and both fields are read-only on real hardware. This model keeps
            // them writable and simply starts them right.
            banked_config: alloc::vec![{
                let mut cfg = [0u8; BANKED];
                for slot in cfg.iter_mut().take(PPI_BASE as usize) {
                    *slot = CFG_EDGE;
                }
                cfg
            }; cpus],
            banked_line: alloc::vec![[false; BANKED]; cpus],
            enabled: alloc::vec![false; spis],
            pending: alloc::vec![false; spis],
            active: alloc::vec![false; spis],
            priority: alloc::vec![0; spis],
            targets: alloc::vec![0; spis],
            config: alloc::vec![0; spis],
            line: alloc::vec![false; spis],
        }
    }

    /// Whether `intid` is pending for `cpu`, counting the input line for a
    /// level-sensitive interrupt.
    ///
    /// A level-sensitive interrupt has no latch of its own: it is pending for
    /// exactly as long as its line is asserted, which is why a device that
    /// forgets to clear its own status register re-enters its handler.
    fn is_pending(&self, cpu: usize, intid: u32) -> bool {
        if intid < BANKED as u32 {
            let i = intid as usize;
            self.banked_pending[cpu][i]
                || (self.banked_config[cpu][i] & CFG_EDGE == 0 && self.banked_line[cpu][i])
        } else {
            let Some(i) = self.spi_index(intid) else {
                return false;
            };
            self.pending[i] || (self.config[i] & CFG_EDGE == 0 && self.line[i])
        }
    }

    fn is_enabled(&self, cpu: usize, intid: u32) -> bool {
        if intid < BANKED as u32 {
            self.banked_enabled[cpu][intid as usize]
        } else {
            self.spi_index(intid)
                .is_some_and(|i| self.enabled[i] && self.targets[i] & (1 << cpu) != 0)
        }
    }

    fn is_active(&self, cpu: usize, intid: u32) -> bool {
        if intid < BANKED as u32 {
            self.banked_active[cpu][intid as usize]
        } else {
            self.spi_index(intid).is_some_and(|i| self.active[i])
        }
    }

    fn priority_of(&self, cpu: usize, intid: u32) -> u8 {
        if intid < BANKED as u32 {
            self.banked_priority[cpu][intid as usize]
        } else {
            self.spi_index(intid)
                .map_or(IDLE_PRIORITY, |i| self.priority[i])
        }
    }

    /// The index into the shared arrays, or `None` for an id this block does
    /// not implement.
    fn spi_index(&self, intid: u32) -> Option<usize> {
        let i = intid.checked_sub(SPI_BASE)? as usize;
        (i < self.enabled.len()).then_some(i)
    }

    /// The running priority of `cpu`: the innermost active interrupt's, or
    /// idle.
    fn running_priority(&self, cpu: usize) -> u8 {
        self.running[cpu].last().copied().unwrap_or(IDLE_PRIORITY)
    }

    /// How many interrupt ids the distributor implements, including the
    /// banked ones.
    fn intids(&self) -> u32 {
        SPI_BASE + self.enabled.len() as u32
    }
}

/// The register blocks, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// Each CPU interface's `nIRQ` output, at [`LockRank::LEAF`].
    outs: Mutex<Vec<Option<WireSource>>>,
    /// Which interrupt id each driving net lands on, for the device tree.
    wires: Mutex<BTreeMap<WireId, u32>>,
    cpus: usize,
    spis: usize,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("cpus", &self.cpus).field("spis", &self.spis);
        match self.state.try_lock() {
            Some(state) => s.field("enabled", &state.dist_enabled).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// The GICv2 distributor and CPU interface.
#[derive(Debug)]
pub struct Gic {
    regs: Arc<Registers>,
    dist: RegionRef,
    cpuif: RegionRef,
    /// The sinks handed out by [`Device::sink`], kept alive here — a net holds
    /// only a weak reference to a sink.
    pins: Mutex<Vec<Arc<SourcePin>>>,
}

impl Gic {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a CPU or interrupt count the register map
    /// cannot hold, or for a property this class does not know.
    pub fn new(props: &Props) -> Result<Gic> {
        let mut r = props.reader();
        let cpus = r.or_range("cpus", 1u64, 1..=MAX_CPUS)?;
        let spis = r.or_range("spis", 96u64, 32..=u64::from(MAX_INTID - SPI_BASE))?;
        r.finish()?;
        if !spis.is_multiple_of(32) {
            return Err(Error::Property(format!(
                "`spis` is a count of shared interrupts and the distributor's registers hold \
                 thirty-two of them per word, so it must be a multiple of 32; {spis} is not"
            )));
        }
        Ok(Gic::build(cpus as usize, spis as usize))
    }

    /// Build one directly, for a test or a hand-wired machine.
    #[must_use]
    pub fn build(cpus: usize, spis: usize) -> Gic {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(cpus, spis)),
            outs: Mutex::with_rank(LockRank::LEAF, alloc::vec![None; cpus]),
            wires: Mutex::with_rank(LockRank::LEAF, BTreeMap::new()),
            cpus,
            spis,
        });
        let dist: RegionRef = Arc::new(Region::io(
            "arm.gic.dist",
            DIST_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        let cpuif: RegionRef = Arc::new(Region::io(
            "arm.gic.cpu",
            CPU_WINDOW_LEN,
            Arc::new(CpuIface {
                regs: Arc::clone(&regs),
            }) as Arc<dyn MemOps>,
        ));
        Gic {
            regs,
            dist,
            cpuif,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// How many CPU interfaces it has.
    #[must_use]
    pub fn cpus(&self) -> usize {
        self.regs.cpus
    }

    /// How many shared peripheral interrupts it implements.
    #[must_use]
    pub fn spis(&self) -> usize {
        self.regs.spis
    }

    /// Drive an interrupt input directly, as a wire would.
    ///
    /// `intid` is the architectural id: 16-31 for a private peripheral
    /// interrupt on CPU 0, 32 and up for a shared one.
    pub fn set_source(&self, intid: u32, level: bool) {
        self.regs.set_level(0, intid, level);
    }

    /// Whether `intid` is pending for CPU 0.
    #[must_use]
    pub fn is_pending(&self, intid: u32) -> bool {
        self.regs.state.lock().is_pending(0, intid)
    }

    /// Claim the highest-priority interrupt for `cpu`, as a `GICC_IAR` read
    /// would.
    pub fn claim(&self, cpu: usize) -> u32 {
        self.regs.claim(cpu)
    }

    /// End the interrupt `intid` on `cpu`, as a `GICC_EOIR` write would.
    pub fn complete(&self, cpu: usize, intid: u32) {
        self.regs.complete(cpu, intid);
    }
}

impl Registers {
    /// The highest-priority interrupt `cpu` could be signalled now, with its
    /// priority — or [`SPURIOUS`] if there is none.
    ///
    /// Ties break toward the lowest id, which the specification leaves to the
    /// implementation and which is the only stable answer.
    fn best(state: &State, cpu: usize) -> (u32, u8) {
        if !state.dist_enabled {
            return (SPURIOUS, IDLE_PRIORITY);
        }
        let mut best = SPURIOUS;
        let mut best_priority = IDLE_PRIORITY;
        for intid in 0..state.intids() {
            if !state.is_enabled(cpu, intid)
                || !state.is_pending(cpu, intid)
                || state.is_active(cpu, intid)
            {
                continue;
            }
            let priority = state.priority_of(cpu, intid);
            if priority < best_priority {
                best_priority = priority;
                best = intid;
            }
        }
        (best, best_priority)
    }

    /// Whether `cpu`'s `nIRQ` should be asserted.
    ///
    /// Three gates, and all three are the specification's: the CPU interface
    /// is enabled, the interrupt's priority is above the mask, and it is above
    /// the running priority — which is what makes a higher-priority interrupt
    /// preempt a handler and a same-priority one wait (IHI 0048 §3.2.1).
    fn signalled(state: &State, cpu: usize) -> bool {
        if !state.cpu_enabled[cpu] {
            return false;
        }
        let (intid, priority) = Self::best(state, cpu);
        intid != SPURIOUS && priority < state.pmr[cpu] && priority < state.running_priority(cpu)
    }

    /// Which CPU interfaces should now have `nIRQ` asserted.
    fn evaluate(state: &State) -> Vec<bool> {
        (0..state.cpu_enabled.len())
            .map(|cpu| Self::signalled(state, cpu))
            .collect()
    }

    /// Drive the outputs. Never called with the state lock held.
    fn drive(&self, levels: &[bool]) {
        let outs: Vec<Option<WireSource>> = self.outs.lock().clone();
        for (out, on) in outs.iter().zip(levels) {
            if let Some(out) = out {
                out.set(Level::from_bool(*on));
            }
        }
    }

    /// An input line moved.
    fn set_level(&self, cpu: usize, intid: u32, level: bool) {
        let levels = {
            let mut state = self.state.lock();
            if intid < BANKED as u32 {
                let i = intid as usize;
                if state.banked_line[cpu][i] == level {
                    return;
                }
                state.banked_line[cpu][i] = level;
                // An edge-triggered interrupt latches on the rising edge and
                // then stops caring about the line; a level-sensitive one has
                // no latch at all.
                if level && state.banked_config[cpu][i] & CFG_EDGE != 0 {
                    state.banked_pending[cpu][i] = true;
                }
            } else {
                let Some(i) = state.spi_index(intid) else {
                    return;
                };
                if state.line[i] == level {
                    return;
                }
                state.line[i] = level;
                if level && state.config[i] & CFG_EDGE != 0 {
                    state.pending[i] = true;
                }
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }

    /// `GICC_IAR`: acknowledge the highest-priority interrupt for `cpu`.
    fn claim(&self, cpu: usize) -> u32 {
        let (intid, levels) = {
            let mut state = self.state.lock();
            if !state.cpu_enabled[cpu] {
                return SPURIOUS;
            }
            let (intid, priority) = Self::best(&state, cpu);
            if intid == SPURIOUS
                || priority >= state.pmr[cpu]
                || priority >= state.running_priority(cpu)
            {
                return SPURIOUS;
            }
            // Pending to active. The latch is cleared; a level-sensitive
            // interrupt whose line is still asserted becomes active *and*
            // pending again by [`State::is_pending`], and is not offered again
            // because it is active.
            if intid < BANKED as u32 {
                let i = intid as usize;
                state.banked_pending[cpu][i] = false;
                state.banked_active[cpu][i] = true;
            } else if let Some(i) = state.spi_index(intid) {
                state.pending[i] = false;
                state.active[i] = true;
            }
            state.running[cpu].push(priority);
            state.active_stack[cpu].push(intid);
            let levels = Self::evaluate(&state);
            (intid, levels)
        };
        self.drive(&levels);
        intid
    }

    /// `GICC_EOIR`: drop the priority and deactivate.
    ///
    /// This model implements `GICC_CTLR.EOImode == 0`, where a write to
    /// `GICC_EOIR` does both. Splitting them is what `GICC_DIR` is for, and a
    /// kernel that has not asked for the split never uses it.
    fn complete(&self, cpu: usize, intid: u32) {
        let levels = {
            let mut state = self.state.lock();
            // An `EOIR` naming something other than the innermost active
            // interrupt is UNPREDICTABLE; ignoring it is the answer that
            // cannot corrupt another handler's priority.
            if state.active_stack[cpu].last() != Some(&intid) {
                return;
            }
            state.active_stack[cpu].pop();
            state.running[cpu].pop();
            if intid < BANKED as u32 {
                state.banked_active[cpu][intid as usize] = false;
            } else if let Some(i) = state.spi_index(intid) {
                state.active[i] = false;
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }

    /// `GICD_SGIR`: make a software generated interrupt pending on the target
    /// list (IHI 0048 §4.3.15).
    fn send_sgi(&self, from: usize, value: u32) {
        let levels = {
            let mut state = self.state.lock();
            let intid = (value & 0xf) as usize;
            let list = ((value >> 16) & 0xff) as usize;
            let targets: Vec<usize> = match (value >> 24) & 3 {
                // The target list field names the CPUs.
                0 => (0..state.cpu_enabled.len())
                    .filter(|cpu| list & (1 << cpu) != 0)
                    .collect(),
                // Everyone but me.
                1 => (0..state.cpu_enabled.len())
                    .filter(|cpu| *cpu != from)
                    .collect(),
                // Only me.
                _ => alloc::vec![from],
            };
            for cpu in targets {
                state.banked_pending[cpu][intid] = true;
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }

    // -- the distributor's register file ------------------------------------

    /// Pack one 32-bit word of a per-interrupt bit array.
    ///
    /// A free function rather than a method: the four bit arrays differ only
    /// in which predicate they ask of each interrupt, and this way the read
    /// side of `GICD_ISENABLER`, `GICD_ISPENDR` and `GICD_ISACTIVER` is one
    /// line each.
    fn bits_word(
        state: &State,
        cpu: usize,
        word: usize,
        pick: fn(&State, usize, u32) -> bool,
    ) -> u32 {
        let mut out = 0u32;
        for bit in 0..32 {
            let intid = (word * 32 + bit) as u32;
            if intid < state.intids() && pick(state, cpu, intid) {
                out |= 1 << bit;
            }
        }
        out
    }

    /// Read one distributor register.
    fn dist_read(&self, offset: u64, cpu: usize) -> u32 {
        let state = self.state.lock();
        match offset {
            0x000 => u32::from(state.dist_enabled),
            // `GICD_TYPER`: ITLinesNumber in 4:0 as (ids/32 - 1), CPUNumber in
            // 7:5 as (cpus - 1), no security extensions.
            0x004 => {
                let lines = state.intids() / 32 - 1;
                lines | ((self.cpus as u32 - 1) << 5)
            }
            0x008 => GICD_IIDR,
            0x080..0x100 => {
                // `GICD_IGROUPR`: every interrupt is group 0 on a GIC without
                // security extensions, and this model has none.
                0
            }
            0x100..0x180 => {
                Self::bits_word(&state, cpu, ((offset - 0x100) / 4) as usize, |s, c, i| {
                    s.is_enabled(c, i)
                })
            }
            0x180..0x200 => {
                Self::bits_word(&state, cpu, ((offset - 0x180) / 4) as usize, |s, c, i| {
                    s.is_enabled(c, i)
                })
            }
            0x200..0x280 => {
                Self::bits_word(&state, cpu, ((offset - 0x200) / 4) as usize, |s, c, i| {
                    s.is_pending(c, i)
                })
            }
            0x280..0x300 => {
                Self::bits_word(&state, cpu, ((offset - 0x280) / 4) as usize, |s, c, i| {
                    s.is_pending(c, i)
                })
            }
            0x300..0x380 => {
                Self::bits_word(&state, cpu, ((offset - 0x300) / 4) as usize, |s, c, i| {
                    s.is_active(c, i)
                })
            }
            0x380..0x400 => {
                Self::bits_word(&state, cpu, ((offset - 0x380) / 4) as usize, |s, c, i| {
                    s.is_active(c, i)
                })
            }
            0x400..0x800 => {
                let base = (offset - 0x400) as u32;
                let mut out = 0u32;
                for byte in 0..4 {
                    let intid = base + byte;
                    if intid < state.intids() {
                        out |= u32::from(state.priority_of(cpu, intid)) << (byte * 8);
                    }
                }
                out
            }
            0x800..0xc00 => {
                let base = (offset - 0x800) as u32;
                let mut out = 0u32;
                for byte in 0..4 {
                    let intid = base + byte;
                    if intid >= state.intids() {
                        continue;
                    }
                    // A banked interrupt targets the CPU reading the register
                    // and nothing else, and the field is read-only.
                    let mask = if intid < SPI_BASE {
                        1u8 << cpu
                    } else {
                        state.spi_index(intid).map_or(0, |i| state.targets[i])
                    };
                    out |= u32::from(mask) << (byte * 8);
                }
                out
            }
            0xc00..0xd00 => {
                let base = ((offset - 0xc00) * 4) as u32;
                let mut out = 0u32;
                for slot in 0..16 {
                    let intid = base + slot;
                    if intid >= state.intids() {
                        continue;
                    }
                    let cfg = if intid < BANKED as u32 {
                        state.banked_config[cpu][intid as usize]
                    } else {
                        state.spi_index(intid).map_or(0, |i| state.config[i])
                    };
                    out |= u32::from(cfg) << (slot * 2);
                }
                out
            }
            // The identification registers, which say GICv2 to anything that
            // reads them the way it reads a PrimeCell part.
            0xfe8 => 0x0000_0002,
            _ => 0,
        }
    }

    /// Write one distributor register.
    fn dist_write(&self, offset: u64, cpu: usize, value: u32) {
        if offset == 0xf00 {
            self.send_sgi(cpu, value);
            return;
        }
        let levels = {
            let mut state = self.state.lock();
            let intids = state.intids();
            match offset {
                0x000 => state.dist_enabled = value & 1 != 0,
                0x100..0x200 => {
                    // One arm for the set half and the clear half, because
                    // they differ only in which they do: `GICD_ISENABLER` at
                    // 0x100 and `GICD_ICENABLER` at 0x180 are the same bit
                    // array written from two windows.
                    let set = offset < 0x180;
                    let word = ((offset - if set { 0x100 } else { 0x180 }) / 4) as usize;
                    for bit in 0..32 {
                        if value & (1 << bit) == 0 {
                            continue;
                        }
                        let intid = (word * 32 + bit) as u32;
                        if intid >= intids {
                            continue;
                        }
                        if intid < BANKED as u32 {
                            state.banked_enabled[cpu][intid as usize] = set;
                        } else if let Some(i) = state.spi_index(intid) {
                            state.enabled[i] = set;
                        }
                    }
                }
                0x200..0x300 => {
                    // One arm for the set half and the clear half, because
                    // they differ only in which they do: `GICD_ISENABLER` at
                    // 0x200 and `GICD_ICENABLER` at 0x280 are the same bit
                    // array written from two windows.
                    let set = offset < 0x280;
                    let word = ((offset - if set { 0x200 } else { 0x280 }) / 4) as usize;
                    for bit in 0..32 {
                        if value & (1 << bit) == 0 {
                            continue;
                        }
                        let intid = (word * 32 + bit) as u32;
                        if intid >= intids {
                            continue;
                        }
                        if intid < BANKED as u32 {
                            state.banked_pending[cpu][intid as usize] = set;
                        } else if let Some(i) = state.spi_index(intid) {
                            state.pending[i] = set;
                        }
                    }
                }
                0x300..0x400 => {
                    // One arm for the set half and the clear half, because
                    // they differ only in which they do: `GICD_ISENABLER` at
                    // 0x300 and `GICD_ICENABLER` at 0x380 are the same bit
                    // array written from two windows.
                    let set = offset < 0x380;
                    let word = ((offset - if set { 0x300 } else { 0x380 }) / 4) as usize;
                    for bit in 0..32 {
                        if value & (1 << bit) == 0 {
                            continue;
                        }
                        let intid = (word * 32 + bit) as u32;
                        if intid >= intids {
                            continue;
                        }
                        if intid < BANKED as u32 {
                            state.banked_active[cpu][intid as usize] = set;
                        } else if let Some(i) = state.spi_index(intid) {
                            state.active[i] = set;
                        }
                    }
                }
                0x400..0x800 => {
                    let base = (offset - 0x400) as u32;
                    for byte in 0..4 {
                        let intid = base + byte;
                        if intid >= intids {
                            continue;
                        }
                        let priority = ((value >> (byte * 8)) as u8) & PRIORITY_MASK;
                        if intid < BANKED as u32 {
                            state.banked_priority[cpu][intid as usize] = priority;
                        } else if let Some(i) = state.spi_index(intid) {
                            state.priority[i] = priority;
                        }
                    }
                }
                0x800..0xc00 => {
                    let base = (offset - 0x800) as u32;
                    for byte in 0..4 {
                        let intid = base + byte;
                        // Banked ids target their own CPU and the field is
                        // read-only for them (IHI 0048 §4.3.12).
                        if intid < SPI_BASE || intid >= intids {
                            continue;
                        }
                        let mask = (value >> (byte * 8)) as u8;
                        if let Some(i) = state.spi_index(intid) {
                            state.targets[i] = mask;
                        }
                    }
                }
                0xc00..0xd00 => {
                    let base = ((offset - 0xc00) * 4) as u32;
                    for slot in 0..16 {
                        let intid = base + slot;
                        if intid >= intids {
                            continue;
                        }
                        let cfg = ((value >> (slot * 2)) as u8) & 0b11;
                        if intid < BANKED as u32 {
                            // SGIs are always edge-triggered and the field is
                            // read-only; PPIs are writable on some parts and
                            // fixed on others, and this one accepts the write.
                            if intid >= PPI_BASE {
                                state.banked_config[cpu][intid as usize] = cfg;
                            }
                        } else if let Some(i) = state.spi_index(intid) {
                            state.config[i] = cfg;
                        }
                    }
                }
                _ => {}
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }
}

/// The register range whose entries are **one byte each**, and which the
/// architecture therefore makes byte-accessible: `GICD_IPRIORITYR` at 0x400
/// and `GICD_ITARGETSR` at 0x800 (IHI 0048 §4.3.11 and §4.3.12).
///
/// This is not a nicety. A driver setting one interrupt's affinity writes
/// *one byte* of `GICD_ITARGETSR`, because a read-modify-write of the word
/// would race with the three interrupts either side of it — and a controller
/// that refused the byte gives a kernel an external abort in
/// `gic_set_affinity` the first time it opens its own console. That is where
/// this range came from.
const BYTE_ACCESSIBLE: core::ops::Range<u64> = 0x400..0xc00;

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let len = dst.len() as u64;
        if !matches!(len, 1 | 2 | 4) || !offset.is_multiple_of(len) {
            return Err(BusError::BadAccess);
        }
        // Every distributor register is idempotent to read — the side effects
        // are all on the CPU interface — so `debug` needs no special case
        // here, which is worth stating because it is the exception.
        let value = self.dist_read(offset & !3, self.cpu_of(attrs));
        let bytes = value.to_le_bytes();
        let at = (offset & 3) as usize;
        dst.copy_from_slice(&bytes[at..at + dst.len()]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let len = src.len() as u64;
        if !matches!(len, 1 | 2 | 4) || !offset.is_multiple_of(len) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Every writable register here changes which interrupt a core
            // takes next (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let cpu = self.cpu_of(attrs);
        if len == 4 {
            let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
            self.dist_write(offset, cpu, value);
            return Ok(());
        }
        // A narrower write, which only the byte-per-interrupt arrays accept.
        // Everywhere else the registers are bit arrays with write-one-to-set
        // semantics, and a read-modify-write of one would set bits the guest
        // did not ask for — so a narrow write there is refused rather than
        // approximated.
        if !BYTE_ACCESSIBLE.contains(&offset) {
            return Err(BusError::BadAccess);
        }
        let word = offset & !3;
        let mut bytes = self.dist_read(word, cpu).to_le_bytes();
        let at = (offset & 3) as usize;
        bytes[at..at + src.len()].copy_from_slice(src);
        self.dist_write(word, cpu, u32::from_le_bytes(bytes));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, halfword and word, naturally aligned: the priority and target
        // arrays are byte-per-interrupt and a driver writes them one byte at a
        // time. The narrow *writes* that are actually legal are checked above,
        // where the offset is known; this is only the fast reject.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
            .with_endian(Endian::Little)
    }
}

impl Registers {
    /// Which CPU interface an access came from.
    ///
    /// A GIC's banked registers answer differently depending on *who is
    /// asking*, which no other device in this tree needs and which the bus
    /// only half carries: [`MemAttrs::requester`] identifies the master, and
    /// the mapping from a requester id to a CPU interface number is the
    /// board's. With one CPU the answer is always zero, and that is the
    /// configuration this board ships; a second core needs the requester table
    /// this seam does not have yet, and `docs/platforms/arm64-virt.md` records
    /// it as the first thing SMP will need.
    fn cpu_of(&self, _attrs: MemAttrs) -> usize {
        0
    }
}

/// The CPU interface's register block: a second [`MemOps`] over the same
/// state, because it is a second aperture rather than a second device.
#[derive(Debug)]
struct CpuIface {
    regs: Arc<Registers>,
}

impl MemOps for CpuIface {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        let cpu = self.regs.cpu_of(attrs);
        let value = match offset {
            0x00 => u32::from(self.regs.state.lock().cpu_enabled[cpu]),
            0x04 => u32::from(self.regs.state.lock().pmr[cpu]),
            0x08 => u32::from(self.regs.state.lock().bpr[cpu]),
            0x0c => {
                if attrs.debug {
                    // Reading `GICC_IAR` *is* the acknowledgement: a debugger
                    // that peeked here would steal the guest's interrupt
                    // (`ROADMAP.md` §15, invariant 5).
                    return Err(BusError::BadAccess);
                }
                self.regs.claim(cpu)
            }
            0x14 => u32::from(self.regs.state.lock().running_priority(cpu)),
            0x18 => {
                // `GICC_HPPIR` is the read that tells you what `GICC_IAR`
                // would return without taking it — the one non-destructive
                // window onto the same answer.
                Registers::best(&self.regs.state.lock(), cpu).0
            }
            0x1c => u32::from(self.regs.state.lock().bpr[cpu]),
            0xfc => GICC_IIDR,
            _ => 0,
        };
        dst.copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let cpu = self.regs.cpu_of(attrs);
        match offset {
            0x00 | 0x04 | 0x08 | 0x1c => {
                let levels = {
                    let mut state = self.regs.state.lock();
                    match offset {
                        0x00 => state.cpu_enabled[cpu] = value & 1 != 0,
                        0x04 => state.pmr[cpu] = (value as u8) & PRIORITY_MASK,
                        _ => state.bpr[cpu] = (value as u8) & 7,
                    }
                    Registers::evaluate(&state)
                };
                self.regs.drive(&levels);
            }
            // `GICC_EOIR`, and `GICC_DIR` at the top of the second page. This
            // model does not split priority drop from deactivation, so both
            // do the whole thing.
            0x10 | 0x1000 => self.regs.complete(cpu, value & 0x3ff),
            _ => {}
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        NodeSpec {
            kind: NodeKind::Gic,
            name: "intc",
            // `arm,cortex-a15-gic` is the compatible string every GICv2 driver
            // has matched since GICv2 shipped; `arm,gic-400` is the part name
            // for the same programmers' model.
            compatible: &["arm,cortex-a15-gic", "arm,gic-400"],
            cells: Vec::new(),
            strings: Vec::new(),
            irq_wire: None,
        }
    }

    fn dt_interrupt(&self, wire: WireId) -> Option<IntSpec> {
        let intid = *self.wires.lock().get(&wire)?;
        if intid < SPI_BASE {
            // A private peripheral interrupt: the tree numbers it from 16, and
            // the flags cell carries the CPU mask in bits 15:8 for GICv2.
            Some(IntSpec::ppi(intid - PPI_BASE))
        } else {
            Some(IntSpec::spi(intid - SPI_BASE))
        }
    }
}

/// One of the GIC's interrupt inputs, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the driver that changed rather than the resolved level of the
/// net — and a shared peripheral interrupt is exactly the case that makes the
/// difference.
#[derive(Debug)]
pub struct SourcePin {
    regs: Arc<Registers>,
    /// Which CPU interface a banked input belongs to. Zero for an SPI, which
    /// is not banked at all.
    cpu: usize,
    intid: u32,
    inputs: FanIn,
}

impl SourcePin {
    fn new(regs: Arc<Registers>, cpu: usize, intid: u32, sources: &[WireId]) -> SourcePin {
        SourcePin {
            regs,
            cpu,
            intid,
            inputs: FanIn::new(sources),
        }
    }

    /// Which interrupt id this pin drives.
    #[must_use]
    pub fn intid(&self) -> u32 {
        self.intid
    }
}

impl WireSink for SourcePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.regs.set_level(
            self.cpu,
            self.intid,
            self.inputs.resolve(Resolve::Or).is_high(),
        );
    }
}

/// The `arm.gic` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "ARM Generic Interrupt Controller v2: distributor and CPU interface",
    properties: &[
        PropertySpec {
            name: "cpus",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many CPU interfaces it has (default 1, at most 8)",
        },
        PropertySpec {
            name: "spis",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many shared peripheral interrupts, a multiple of 32 (default 96)",
        },
    ],
    construct: |props| Ok(Box::new(Gic::new(props)?)),
};

impl Device for Gic {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Both apertures, so the generated tree's `reg` carries both — one
        // node with two entries, which is what the binding wants and what
        // stops either address being written down twice.
        let source = Arc::downgrade(&self.regs) as Weak<dyn DtSource>;
        super::dt::publish(ctx.hosts(), &self.dist, Weak::clone(&source))?;
        super::dt::publish(ctx.hosts(), &self.cpuif, source)
    }

    fn reset(&self, _kind: ResetKind) {
        let levels = {
            let mut state = self.regs.state.lock();
            *state = State::new(self.regs.cpus, self.regs.spis);
            Registers::evaluate(&state)
        };
        self.regs.drive(&levels);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "dist" => Some(Arc::clone(&self.dist)),
            "cpu" => Some(Arc::clone(&self.cpuif)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let (cpu, intid) = parse_input(port, self.regs.cpus, self.regs.spis)?;
        {
            // The one place both an interrupt id and the nets that drive it
            // are known, which is exactly what the device tree needs.
            let mut wires = self.regs.wires.lock();
            for id in sources {
                wires.insert(*id, intid);
            }
        }
        let pin = Arc::new(SourcePin::new(Arc::clone(&self.regs), cpu, intid, sources));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: intid,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let cpu = parse_output(port, self.regs.cpus).ok_or_else(|| unknown_pin(port))?;
        self.regs.outs.lock()[cpu] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some(cpu) = parse_output(port, self.regs.cpus) else {
            return;
        };
        let level = Registers::signalled(&self.regs.state.lock(), cpu);
        let out = self.regs.outs.lock()[cpu].clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_bool(state.dist_enabled)?;
        w.write_seq_len(self.regs.cpus as u64)?;
        for cpu in 0..self.regs.cpus {
            w.write_bool(state.cpu_enabled[cpu])?;
            w.write_u8(state.pmr[cpu])?;
            w.write_u8(state.bpr[cpu])?;
            w.write_seq_len(state.running[cpu].len() as u64)?;
            for (priority, intid) in state.running[cpu].iter().zip(&state.active_stack[cpu]) {
                w.write_u8(*priority)?;
                w.write_u32(*intid)?;
            }
            for i in 0..BANKED {
                w.write_bool(state.banked_enabled[cpu][i])?;
                w.write_bool(state.banked_pending[cpu][i])?;
                w.write_bool(state.banked_active[cpu][i])?;
                w.write_bool(state.banked_line[cpu][i])?;
                w.write_u8(state.banked_priority[cpu][i])?;
                w.write_u8(state.banked_config[cpu][i])?;
            }
        }
        w.write_seq_len(self.regs.spis as u64)?;
        for i in 0..self.regs.spis {
            w.write_bool(state.enabled[i])?;
            w.write_bool(state.pending[i])?;
            w.write_bool(state.active[i])?;
            w.write_bool(state.line[i])?;
            w.write_u8(state.priority[i])?;
            w.write_u8(state.targets[i])?;
            w.write_u8(state.config[i])?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.regs.cpus, self.regs.spis);
        state.dist_enabled = r.read_bool()?;
        let cpus = r.read_seq_len(1)? as usize;
        if cpus != self.regs.cpus {
            return Err(Error::State(format!(
                "snapshot has {cpus} GIC CPU interface(s), this block has {}",
                self.regs.cpus
            )));
        }
        for cpu in 0..cpus {
            state.cpu_enabled[cpu] = r.read_bool()?;
            state.pmr[cpu] = r.read_u8()?;
            state.bpr[cpu] = r.read_u8()?;
            let depth = r.read_seq_len(5)? as usize;
            for _ in 0..depth {
                state.running[cpu].push(r.read_u8()?);
                state.active_stack[cpu].push(r.read_u32()?);
            }
            for i in 0..BANKED {
                state.banked_enabled[cpu][i] = r.read_bool()?;
                state.banked_pending[cpu][i] = r.read_bool()?;
                state.banked_active[cpu][i] = r.read_bool()?;
                state.banked_line[cpu][i] = r.read_bool()?;
                state.banked_priority[cpu][i] = r.read_u8()?;
                state.banked_config[cpu][i] = r.read_u8()?;
            }
        }
        let spis = r.read_seq_len(1)? as usize;
        if spis != self.regs.spis {
            return Err(Error::State(format!(
                "snapshot has {spis} shared interrupt(s), this block implements {}",
                self.regs.spis
            )));
        }
        for i in 0..spis {
            state.enabled[i] = r.read_bool()?;
            state.pending[i] = r.read_bool()?;
            state.active[i] = r.read_bool()?;
            state.line[i] = r.read_bool()?;
            state.priority[i] = r.read_u8()?;
            state.targets[i] = r.read_u8()?;
            state.config[i] = r.read_u8()?;
        }
        let levels = {
            let mut live = self.regs.state.lock();
            *live = state;
            Registers::evaluate(&live)
        };
        self.regs.drive(&levels);
        Ok(())
    }
}

impl Instance for Gic {}

/// Which CPU interface and interrupt id an input pin name refers to.
///
/// `spi<N>` is shared interrupt `N`, which is architectural id `N + 32`.
/// `ppi<N>` is private interrupt `N` on CPU 0, architectural id `N + 16`; a
/// second core's private interrupts are `cpu<C>ppi<N>`, so a single-core board
/// never writes the prefix.
fn parse_input(port: &str, cpus: usize, spis: usize) -> Option<(usize, u32)> {
    if let Some(rest) = port.strip_prefix("spi") {
        let n = rest.parse::<u32>().ok()?;
        return ((n as usize) < spis).then_some((0, n + SPI_BASE));
    }
    let (cpu, rest) = match port.strip_prefix("cpu") {
        Some(rest) => {
            let (digits, rest) = rest.split_at(rest.find(|c: char| !c.is_ascii_digit())?);
            (digits.parse::<usize>().ok()?, rest)
        }
        None => (0, port),
    };
    let n = rest.strip_prefix("ppi")?.parse::<u32>().ok()?;
    (cpu < cpus && n < PPI_BASE).then_some((cpu, n + PPI_BASE))
}

/// Which CPU interface an output pin name refers to: `irq<C>`.
fn parse_output(port: &str, cpus: usize) -> Option<usize> {
    let cpu = port.strip_prefix("irq")?.parse::<usize>().ok()?;
    (cpu < cpus).then_some(cpu)
}

/// The error for a pin this block does not drive.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: format!(
            "a GIC drives `irq<cpu>`, one per CPU interface; `{port}` is not one of them"
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Gic::new(props)?)))
}

/// How many shared interrupt pins the validator knows about.
///
/// A schema is per class and cannot know how a given file configured the
/// instance, so the pins are declared up to a number comfortably past what a
/// board of this shape uses. A `wire … -> gic.spi999` on a 96-source block is
/// still refused — by [`Device::sink`], at realize time, with the count in
/// hand.
pub const SCHEMA_SPIS: u32 = 64;

/// What the validator should know about `arm.gic`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, MAX_CPUS))
        .prop(PropSchema::new("spis", ValueKind::Uint).range(32, u64::from(MAX_INTID - SPI_BASE)))
        .region("")
        .region("dist")
        .region("cpu");
    for spi in 0..SCHEMA_SPIS {
        s = s.port(format!("spi{spi}"), PortDir::In);
    }
    for cpu in 0..MAX_CPUS as u32 {
        for ppi in 0..PPI_BASE {
            s = s.port(format!("cpu{cpu}ppi{ppi}"), PortDir::In);
        }
        s = s.port(format!("irq{cpu}"), PortDir::Out);
    }
    for ppi in 0..PPI_BASE {
        s = s.port(format!("ppi{ppi}"), PortDir::In);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// A sink that records the last level it was told.
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

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    fn dist_read(g: &Gic, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        g.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(bytes)
    }

    fn dist_write(g: &Gic, offset: u64, value: u32) {
        g.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    fn cpu_read(g: &Gic, offset: u64) -> u32 {
        let iface = CpuIface {
            regs: Arc::clone(&g.regs),
        };
        let mut bytes = [0u8; 4];
        iface
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(bytes)
    }

    fn cpu_write(g: &Gic, offset: u64, value: u32) {
        let iface = CpuIface {
            regs: Arc::clone(&g.regs),
        };
        iface
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// A GIC with `irq0` on a probe and the distributor and interface enabled,
    /// which is the state a kernel leaves them in.
    fn armed() -> (Gic, Arc<Probe>) {
        let gic = Gic::build(1, 96);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        gic.connect("irq0", WireSource::new(wire, id))
            .expect("a GIC drives irq0");
        dist_write(&gic, 0x000, 1);
        cpu_write(&gic, 0x000, 1);
        // Mask everything weaker than 0xf0, which is what a kernel does.
        cpu_write(&gic, 0x004, 0xf0);
        (gic, probe)
    }

    /// Enable shared interrupt `n` at priority `priority`, targeted at CPU 0.
    ///
    /// Read-modify-write, because one word of `GICD_IPRIORITYR` holds four
    /// interrupts' priorities and one word of `GICD_ITARGETSR` holds four
    /// interrupts' target masks — a helper that wrote the word whole would
    /// disable the three interrupts either side of the one it was enabling,
    /// which is exactly the mistake a driver makes once.
    fn enable_spi(g: &Gic, n: u32, priority: u8) {
        let intid = n + SPI_BASE;
        let shift = (intid & 3) * 8;
        let word = u64::from(intid & !3);
        let merge = |g: &Gic, at: u64, byte: u32| {
            let was = dist_read(g, at) & !(0xff << shift);
            dist_write(g, at, was | (byte << shift));
        };
        merge(g, 0x400 + word, u32::from(priority));
        merge(g, 0x800 + word, 1);
        dist_write(g, 0x100 + u64::from(intid / 32) * 4, 1 << (intid % 32));
    }

    #[test]
    fn typer_reports_the_shape_the_block_was_built_with() {
        let gic = Gic::build(1, 96);
        let typer = dist_read(&gic, 0x004);
        // 128 interrupt ids: (128 / 32) - 1 = 3.
        assert_eq!(typer & 0x1f, 3, "ITLinesNumber");
        assert_eq!((typer >> 5) & 7, 0, "one CPU interface");
        assert_eq!(dist_read(&gic, 0x008), GICD_IIDR);
    }

    #[test]
    fn a_level_source_is_signalled_claimed_and_completed() {
        let (gic, probe) = armed();
        enable_spi(&gic, 1, 0xa0);
        assert!(!probe.high(), "nothing is asserting yet");

        gic.set_source(SPI_BASE + 1, true);
        assert!(probe.high(), "an enabled pending interrupt must signal");
        assert_eq!(cpu_read(&gic, 0x018), SPI_BASE + 1, "GICC_HPPIR");

        let claimed = cpu_read(&gic, 0x00c);
        assert_eq!(claimed, SPI_BASE + 1, "GICC_IAR");
        assert!(
            !probe.high(),
            "the line drops while the handler holds the priority"
        );
        assert_eq!(cpu_read(&gic, 0x014), 0xa0, "GICC_RPR is the handler's");

        // The device clears its own status, then the handler ends the
        // interrupt: the line must stay low afterwards.
        gic.set_source(SPI_BASE + 1, false);
        cpu_write(&gic, 0x010, claimed);
        assert_eq!(cpu_read(&gic, 0x014), u32::from(IDLE_PRIORITY));
        assert!(!probe.high());
    }

    #[test]
    fn a_level_source_that_is_still_asserted_at_eoi_comes_straight_back() {
        // The behaviour a device with a sticky status register depends on, and
        // the one a model that clears pending on claim gets wrong.
        let (gic, probe) = armed();
        enable_spi(&gic, 2, 0xa0);
        gic.set_source(SPI_BASE + 2, true);
        let claimed = cpu_read(&gic, 0x00c);
        assert_eq!(claimed, SPI_BASE + 2);
        cpu_write(&gic, 0x010, claimed);
        assert!(probe.high(), "the line is still asserted, so it re-pends");
    }

    #[test]
    fn an_edge_source_latches_and_a_claim_is_what_clears_it() {
        let (gic, probe) = armed();
        enable_spi(&gic, 3, 0xa0);
        let intid = SPI_BASE + 3;
        // Configure it edge-triggered: two bits per interrupt, the top one set.
        let word = 0xc00 + u64::from(intid / 16) * 4;
        let shift = (intid % 16) * 2;
        dist_write(&gic, word, u32::from(CFG_EDGE) << shift);

        gic.set_source(intid, true);
        gic.set_source(intid, false);
        assert!(probe.high(), "the pulse latched");
        assert_eq!(cpu_read(&gic, 0x00c), intid);
        cpu_write(&gic, 0x010, intid);
        assert!(!probe.high(), "and the latch is gone");
    }

    #[test]
    fn the_priority_mask_and_the_running_priority_both_gate_delivery() {
        let (gic, probe) = armed();
        enable_spi(&gic, 4, 0xf0);
        gic.set_source(SPI_BASE + 4, true);
        // Priority 0xf0 is not *above* a mask of 0xf0: the comparison is
        // strictly less-than, which is the whole difference between a masked
        // and an unmasked interrupt.
        assert!(!probe.high());
        cpu_write(&gic, 0x004, 0xf8);
        assert!(probe.high());

        // A second, weaker interrupt must not preempt the first.
        enable_spi(&gic, 5, 0xf0);
        let first = cpu_read(&gic, 0x00c);
        assert_eq!(first, SPI_BASE + 4);
        gic.set_source(SPI_BASE + 5, true);
        assert!(!probe.high(), "same priority does not preempt");
        cpu_write(&gic, 0x010, first);
        assert!(probe.high(), "and it is delivered once the first ends");
    }

    #[test]
    fn a_stronger_interrupt_preempts_a_running_handler() {
        let (gic, probe) = armed();
        enable_spi(&gic, 6, 0xa0);
        enable_spi(&gic, 7, 0x80);
        gic.set_source(SPI_BASE + 6, true);
        assert_eq!(cpu_read(&gic, 0x00c), SPI_BASE + 6);
        assert!(!probe.high());
        gic.set_source(SPI_BASE + 7, true);
        assert!(probe.high(), "0x80 is stronger than 0xa0");
        assert_eq!(cpu_read(&gic, 0x00c), SPI_BASE + 7);
        assert_eq!(cpu_read(&gic, 0x014), 0x80, "the inner handler's priority");
        cpu_write(&gic, 0x010, SPI_BASE + 7);
        assert_eq!(cpu_read(&gic, 0x014), 0xa0, "back to the outer one");
    }

    #[test]
    fn a_private_interrupt_is_banked_and_a_shared_one_is_not() {
        let gic = Gic::build(2, 96);
        dist_write(&gic, 0x000, 1);
        // Enabling PPI 27 through CPU 0's view must not enable it for CPU 1.
        let intid = 27u32;
        dist_write(&gic, 0x100, 1 << intid);
        let state = gic.regs.state.lock();
        assert!(state.banked_enabled[0][intid as usize]);
        assert!(!state.banked_enabled[1][intid as usize]);
    }

    #[test]
    fn one_interrupts_affinity_is_a_byte_write_and_leaves_its_neighbours_alone() {
        // What `gic_set_affinity` does, and what a word-only model refuses.
        let (gic, _) = armed();
        for n in 0..4 {
            enable_spi(&gic, n, 0xa0);
        }
        let intid = SPI_BASE + 2;
        let at = 0x800 + u64::from(intid);
        gic.regs
            .write(at, &[0x01], MemAttrs::DEFAULT)
            .expect("a byte write to GICD_ITARGETSR is legal");
        let mut byte = [0u8; 1];
        gic.regs
            .read(at, &mut byte, MemAttrs::DEFAULT)
            .expect("and so is a byte read");
        assert_eq!(byte[0], 1);
        // The three interrupts sharing that word still target CPU 0.
        assert_eq!(dist_read(&gic, 0x800 + u64::from(intid & !3)), 0x0101_0101);

        // The same for a priority, which is the other byte-per-interrupt
        // array — and the low three bits still read back as zero.
        gic.regs
            .write(0x400 + u64::from(intid), &[0xff], MemAttrs::DEFAULT)
            .expect("a byte write to GICD_IPRIORITYR is legal");
        assert_eq!(dist_read(&gic, 0x400 + u64::from(intid & !3)), 0xa0f8_a0a0);
    }

    #[test]
    fn a_narrow_write_to_a_bit_array_is_refused_rather_than_approximated() {
        // `GICD_ISENABLER` is write-one-to-set: a read-modify-write of the
        // containing word would enable interrupts the guest never named.
        let (gic, _) = armed();
        assert!(gic.regs.write(0x100, &[1], MemAttrs::DEFAULT).is_err());
        assert!(gic.regs.write(0x000, &[1], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn only_five_priority_bits_are_implemented() {
        // A driver discovers the number of implemented bits exactly this way,
        // and uses the answer to decide which priorities can preempt.
        let (gic, _) = armed();
        dist_write(&gic, 0x400 + 4 * 8, 0xffff_ffff);
        assert_eq!(dist_read(&gic, 0x400 + 4 * 8), 0xf8f8_f8f8);
    }

    #[test]
    fn a_debug_read_of_the_acknowledge_register_is_refused() {
        // Reading `GICC_IAR` is the acknowledgement, so a debugger must not.
        let (gic, _) = armed();
        enable_spi(&gic, 8, 0xa0);
        gic.set_source(SPI_BASE + 8, true);
        let iface = CpuIface {
            regs: Arc::clone(&gic.regs),
        };
        let mut bytes = [0u8; 4];
        assert!(iface.read(0x00c, &mut bytes, MemAttrs::DEBUG).is_err());
        assert_eq!(
            cpu_read(&gic, 0x00c),
            SPI_BASE + 8,
            "still there for the guest"
        );
    }

    #[test]
    fn a_disabled_distributor_forwards_nothing() {
        let (gic, probe) = armed();
        enable_spi(&gic, 9, 0xa0);
        gic.set_source(SPI_BASE + 9, true);
        assert!(probe.high());
        dist_write(&gic, 0x000, 0);
        assert!(!probe.high(), "GICD_CTLR.Enable gates everything");
    }

    #[test]
    fn a_software_generated_interrupt_reaches_the_named_cpu() {
        let (gic, probe) = armed();
        // SGI 3 at the default priority of zero, which beats everything.
        dist_write(&gic, 0x100, 1 << 3);
        // Target list filter 0b00: the list names the CPUs, and this
        // one names CPU 0.
        dist_write(&gic, 0xf00, (1 << 16) | 3);
        assert!(probe.high());
        assert_eq!(cpu_read(&gic, 0x00c) & 0x3ff, 3);
    }

    /// Everything `save` writes, as bytes — the state hash, in the only form
    /// this crate has one.
    fn snapshot(g: &Gic) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("gic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("gic", CLASS.name, CLASS.version).unwrap();
            g.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn state_round_trips_to_an_identical_hash() {
        let (gic, _) = armed();
        enable_spi(&gic, 10, 0xa0);
        gic.set_source(SPI_BASE + 10, true);
        assert_eq!(cpu_read(&gic, 0x00c), SPI_BASE + 10, "one handler running");
        let bytes = snapshot(&gic);

        let restored = Gic::build(1, 96);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("gic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(snapshot(&gic), snapshot(&restored), "the state hash");
        // And the restored controller knows a handler is still running, which
        // is the part a snapshot that forgot the active stack would lose.
        assert_eq!(cpu_read(&restored, 0x014), 0xa0);
    }

    #[test]
    fn a_pin_name_says_which_interrupt_it_is() {
        assert_eq!(parse_input("spi0", 1, 96), Some((0, 32)));
        assert_eq!(parse_input("spi95", 1, 96), Some((0, 127)));
        assert_eq!(parse_input("spi96", 1, 96), None, "past the block");
        assert_eq!(parse_input("ppi11", 1, 96), Some((0, 27)));
        assert_eq!(parse_input("cpu1ppi14", 2, 96), Some((1, 30)));
        assert_eq!(parse_input("cpu1ppi14", 1, 96), None, "one CPU only");
        assert_eq!(parse_input("ppi16", 1, 96), None, "there are sixteen");
        assert_eq!(parse_output("irq0", 1), Some(0));
        assert_eq!(parse_output("irq1", 1), None);
    }
}
