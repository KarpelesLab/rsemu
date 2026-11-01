//! Renesas SH-4A — a 32-bit SuperH interpreter.
//!
//! The SH-4A integer core with both register banks, the full exception and
//! interrupt model, the FPU in software IEEE-754, the MMU (64-entry UTLB,
//! 4-entry ITLB, the PMB of 32-bit address extended mode), the store queues,
//! and a disassembler generated from the same instruction description the
//! interpreter decodes with. It is the interpreter half of the core, which
//! means it is the **oracle** anything later is measured against (CLAUDE.md,
//! "CPU cores").
//!
//! # What is here
//!
//! | Piece | Module |
//! | --- | --- |
//! | the one declarative instruction table and decode | [`isa`] |
//! | the disassembler generated from it | [`disasm`] |
//! | the interpreter and the exception model | `exec` |
//! | the FPU | [`fpu`] |
//! | the UTLB, ITLB, PMB and the core's P4 registers | [`mmu`] |
//!
//! # How a machine binds one
//!
//! * **Address space.** One 32-bit physical bus (`space = ...`). In 29-bit
//!   mode — the reset state — P0/P1/P2/P3 reach physical `0x0000_0000 ..
//!   0x1FFF_FFFF`. The P4 area's on-chip peripheral registers
//!   (`0xFC00_0000..`) and on-chip memory (`0xE500_0000..`) go to the bus **at
//!   their P4 address**, so a peripheral is mapped where its hardware manual
//!   puts it. The core's own registers in P4 (`PTEH`, `MMUCR`, `EXPEVT`,
//!   `INTEVT`, `CCR`, `PASCR`, `CPUOPM`, ...), the memory-mapped TLB/PMB and
//!   cache arrays, and the store queues are answered inside the core and never
//!   reach the bus. See [`mmu`] for the full map.
//! * **Interrupts.** The SH-4A's interrupt controller is not part of the CPU
//!   (§5.6.3 defers to "the Interrupt Controller section of the hardware
//!   manual of the target product"), so the boundary is the one the CPU
//!   manual draws: a *priority level* and the *code* that lands in `INTEVT`.
//!   The core publishes a cell as [`ExportId::INTERRUPT_REQUEST`]; an INTC
//!   model stores [`pack_request`]`(level, intevt)` into it whenever its
//!   highest pending request changes, and zero when nothing is pending. The
//!   core accepts the request at an instruction boundary when the level
//!   exceeds `SR.IMASK` and `SR.BL` is clear — or at once from `SLEEP` — and
//!   the handler clears the source in the controller, as on hardware. Code
//!   without a machine graph calls [`Cpu::set_interrupt`].
//! * **Wires.** `nmi` (edge: a rising level latches one NMI), `reset` (a
//!   rising level latches a power-on reset), `hold` (held in reset while high;
//!   the release is a reset, from the address in the core's
//!   [`ExportId::RESET_ADDRESS`] cell if a board wrote one). An SoC reset
//!   controller that releases this core drives `hold` and writes the cell —
//!   the arrangement `rcar.rst` already uses for the Cortex-A9s.
//!
//! # Properties
//!
//! `endian` (`little`/`big` — the MD pin; a machine maps its regions in the
//! same order), `reset` (the reset vector, default `0xA000_0000`), `pvr` and
//! `prr` (what the version registers read), `held` (start held in reset).
//!
//! # What is deliberately not here
//!
//! * **Caches.** Every access goes to the bus. The cache arrays read as
//!   invalid lines and accept writes; `OCBI`/`OCBP`/`OCBWB`/`PREF`/`ICBI`
//!   check their address and do nothing else.
//! * **The user break controller** and `DBR`'s use: `DBR` is a register and
//!   nothing more.
//! * **L memory transfers.** The `LSA`/`LDA` registers hold values; there is
//!   no transfer engine.
//! * **The extended-functions variant's MMU** (8 KB/256 KB/4 MB/64 MB pages,
//!   the second data arrays): this core is the `PVR.VER = H'20` SH-4A of the
//!   *SH-4A Software Manual*.
//! * **Pipeline timing.** See *Timing*.
//!
//! # Timing
//!
//! A cycle is charged because a bus access happened — an instruction fetch, a
//! load, a store — and never from a table of instruction latencies. A `SLEEP`
//! with nothing to wake it spends the rest of its budget at once.
//!
//! # Sources
//!
//! *SH-4A Software Manual*, Renesas REJ09B0003-0150, Rev. 1.50 — the
//! programming model (section 2), the instruction set (3, 10), exceptions
//! (5), the FPU (6), the MMU (7), the store queues (8.7) and the appendices
//! for `CPUOPM`, `PVR` and `PRR`. The *SH-4A Extended Functions Software
//! Manual* (R01US0060EJ0200) was read only to confirm the instruction set is
//! the same. No emulator, assembler, disassembler or operating-system source
//! of any licence was consulted for any part of this core.

pub mod disasm;
mod exec;
pub mod fpu;
pub mod isa;
pub mod mmu;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::exec::{Exit, ExitMask, ExitingCore, Run};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, AtomicU64, LockRank, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};
pub use exec::{RESET_VECTOR, exc, fpscr, sr, vector};
pub use mmu::{Ctl, ItlbEntry, Mmu, PmbEntry, UtlbEntry};

/// What `PVR` reads by default: `CHIP = H'10` (every SH-4A) and `VER = H'20`
/// (the version the *SH-4A Software Manual* documents); `CUT` is a product's
/// own and is zero here (appendix D).
pub const DEFAULT_PVR: u32 = 0x1020_0000;

/// [`ExportId::RESET_ADDRESS`]'s "no address": reset takes the configured
/// reset vector.
pub const NO_RESET_ADDRESS: u64 = u64::MAX;

/// How a processor is configured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Byte order, fixed by a pin at power-on (§2.5).
    pub endian: Endian,
    /// Where the PC starts after a reset: `H'A000_0000` on every SH-4A
    /// (§5.3.2); a property so a test or a board can start elsewhere.
    pub reset_vector: u32,
    /// What `PVR` reads.
    pub pvr: u32,
    /// What `PRR` reads.
    pub prr: u32,
    /// This processor's identity in `MemAttrs::requester`.
    pub requester: RequesterId,
    /// Whether the core comes out of power-on held in reset.
    pub held: bool,
}

impl Config {
    /// A little-endian SH-4A starting at the architectural reset vector.
    #[must_use]
    pub const fn new() -> Config {
        Config {
            endian: Endian::Little,
            reset_vector: RESET_VECTOR,
            pvr: DEFAULT_PVR,
            prr: 0,
            requester: RequesterId::ANONYMOUS,
            held: false,
        }
    }

    /// The same configuration with a different byte order.
    #[must_use]
    pub const fn with_endian(mut self, endian: Endian) -> Self {
        self.endian = endian;
        self
    }

    /// The same configuration with a different reset vector.
    #[must_use]
    pub const fn with_reset_vector(mut self, pc: u32) -> Self {
        self.reset_vector = pc;
        self
    }

    /// The same configuration with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::new()
    }
}

/// Pack an interrupt request for the [`ExportId::INTERRUPT_REQUEST`] cell:
/// the priority level (1–15; 0 means nothing pending) in bits 3–0 and the
/// `INTEVT` code in bits 29–16.
#[must_use]
pub const fn pack_request(level: u32, intevt: u32) -> u64 {
    ((level & 0xf) as u64) | (((intevt & 0x3fff) as u64) << 16)
}

/// The processor's inputs, as atomics outside its execution lock — so an
/// interrupt controller raising a request from inside a store the processor
/// itself issued cannot re-enter the processor's critical section.
#[derive(Debug)]
pub struct Lines {
    /// The packed `(level, INTEVT)` request; published as
    /// [`ExportId::INTERRUPT_REQUEST`].
    request: Arc<AtomicU64>,
    /// A latched NMI edge.
    nmi: AtomicBool,
    /// A latched reset request.
    reset: AtomicBool,
    /// Held in reset.
    hold: AtomicBool,
    /// Where a reset starts, when a board says; published as
    /// [`ExportId::RESET_ADDRESS`].
    reset_address: Arc<AtomicU64>,
}

impl Default for Lines {
    fn default() -> Self {
        Lines {
            request: Arc::new(AtomicU64::new(0)),
            nmi: AtomicBool::new(false),
            reset: AtomicBool::new(false),
            hold: AtomicBool::new(false),
            reset_address: Arc::new(AtomicU64::new(NO_RESET_ADDRESS)),
        }
    }
}

impl Lines {
    /// The current request: `(level, INTEVT code)`.
    #[must_use]
    pub fn request(&self) -> (u32, u32) {
        // Acquire: a controller that stores a request after updating its own
        // registers wants a handler that reads them to see the update.
        let v = self.request.load(Ordering::Acquire);
        ((v & 0xf) as u32, ((v >> 16) & 0x3fff) as u32)
    }

    /// Replace the request.
    pub fn set_request(&self, level: u32, intevt: u32) {
        self.request
            .store(pack_request(level, intevt), Ordering::Release);
    }

    /// Whether an NMI edge is waiting.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.nmi.load(Ordering::Acquire)
    }

    /// Consume the NMI edge.
    pub fn take_nmi(&self) {
        self.nmi.store(false, Ordering::Release);
    }

    /// Latch an NMI edge.
    pub fn raise_nmi(&self) {
        self.nmi.store(true, Ordering::Release);
    }

    /// Latch a reset request; it takes effect at the next step.
    pub fn request_reset(&self) {
        self.reset.store(true, Ordering::Release);
    }

    /// Take a latched reset request.
    fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    /// Hold or release the core; a release is a reset.
    pub fn set_hold(&self, held: bool) {
        let was = self.hold.swap(held, Ordering::AcqRel);
        if was && !held {
            self.request_reset();
        }
    }

    /// Whether the core is held in reset.
    #[must_use]
    pub fn held(&self) -> bool {
        self.hold.load(Ordering::Acquire)
    }
}

/// Everything the interpreter mutates, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// The sinks published by [`Device::sink`], kept alive here — a net holds only
/// a weak reference to a sink.
#[derive(Debug, Default)]
struct Pins {
    nmi: Option<Arc<NmiPin>>,
    reset: Option<Arc<ResetPin>>,
    hold: Option<Arc<HoldPin>>,
}

/// One SH-4A processor.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]: a CPU
/// is a bus master and holds this lock while calling into device models, which
/// take their own `DEVICE`-ranked locks. The inputs are atomics in [`Lines`],
/// so a device driving one from inside an access this processor issued does
/// not re-enter the lock.
#[derive(Debug)]
pub struct Cpu {
    cfg: Config,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// Which architectural traps leave the core instead of vectoring.
    exits: AtomicU32,
    /// This processor's identity in `MemAttrs::requester`, assigned at bind.
    requester: AtomicU32,
    pins: sync::Mutex<Pins>,
}

impl Cpu {
    /// A processor in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Cpu::attach_space) and [`Device::realize`].
    #[must_use]
    pub fn new(cfg: Config) -> Cpu {
        let lines = Arc::new(Lines::default());
        lines.hold.store(cfg.held, Ordering::Release);
        Cpu {
            lines,
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    space: None,
                },
            ),
            exits: AtomicU32::new(ExitMask::NONE.bits()),
            requester: AtomicU32::new(cfg.requester.0),
            pins: sync::Mutex::new(Pins::default()),
            cfg,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or an out-of-range value, or a
    /// property nothing here accepts was given.
    pub fn from_props(props: &Props) -> Result<Cpu> {
        let mut r = props.reader();
        let endian = match r.or_enum("endian", "little", &["little", "big"])? {
            "big" => Endian::Big,
            _ => Endian::Little,
        };
        let reset = r.or_range("reset", u64::from(RESET_VECTOR), 0..=0xffff_ffff)?;
        let pvr = r.or_range("pvr", u64::from(DEFAULT_PVR), 0..=0xffff_ffff)?;
        let prr = r.or_range("prr", 0u64, 0..=0xffff_ffff)?;
        let held = r.or("held", false)?;
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;
        if reset & 1 != 0 {
            return Err(Error::Property(alloc::format!(
                "`reset` is 0x{reset:x}; SH-4A instructions are halfword-aligned"
            )));
        }
        Ok(Cpu::new(Config {
            endian,
            reset_vector: reset as u32,
            pvr: pvr as u32,
            prr: prr as u32,
            requester: RequesterId::ANONYMOUS,
            held,
        }))
    }

    /// This processor's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the processor the address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this processor executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Set the id accesses this processor initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// The configuration with the bind-time requester folded in.
    fn effective_config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// The processor's input latches, for a test or a hand-wired board.
    #[must_use]
    pub fn lines(&self) -> &Arc<Lines> {
        &self.lines
    }

    /// Read general register `Rn` as currently visible.
    #[must_use]
    pub fn reg(&self, n: usize) -> u32 {
        self.session.lock().state.r[n & 15]
    }

    /// Write general register `Rn` as currently visible.
    pub fn set_reg(&self, n: usize, value: u32) {
        self.session.lock().state.r[n & 15] = value;
    }

    /// `R0_BANK0`–`R7_BANK0` and `R0_BANK1`–`R7_BANK1`, wherever `SR` puts
    /// them.
    #[must_use]
    pub fn banks(&self) -> ([u32; 8], [u32; 8]) {
        let s = self.session.lock();
        (s.state.bank0(), s.state.bank1())
    }

    /// The program counter: the next instruction to execute.
    #[must_use]
    pub fn pc(&self) -> u32 {
        self.session.lock().state.pc
    }

    /// Jump somewhere, abandoning any delayed branch in flight.
    pub fn set_pc(&self, pc: u32) {
        let mut s = self.session.lock();
        s.state.pc = pc;
        s.state.delay = None;
    }

    /// The branch target if the instruction at [`Cpu::pc`] is a delay slot.
    #[must_use]
    pub fn delay_target(&self) -> Option<u32> {
        self.session.lock().state.delay
    }

    /// `SR`.
    #[must_use]
    pub fn sr(&self) -> u32 {
        self.session.lock().state.sr
    }

    /// Write `SR` the way `LDC` does, re-banking `R0`–`R7` if `MD`/`RB` move.
    pub fn set_sr(&self, value: u32) {
        self.session.lock().state.set_sr(value);
    }

    /// The control and system registers: `(GBR, VBR, SSR, SPC, SGR, DBR)`.
    #[must_use]
    pub fn control(&self) -> [u32; 6] {
        let s = &self.session.lock().state;
        [s.gbr, s.vbr, s.ssr, s.spc, s.sgr, s.dbr]
    }

    /// Set `VBR`.
    pub fn set_vbr(&self, value: u32) {
        self.session.lock().state.vbr = value;
    }

    /// Set `GBR`.
    pub fn set_gbr(&self, value: u32) {
        self.session.lock().state.gbr = value;
    }

    /// `(MACH, MACL, PR)`.
    #[must_use]
    pub fn system(&self) -> (u32, u32, u32) {
        let s = &self.session.lock().state;
        (s.mach, s.macl, s.pr)
    }

    /// Set `MACH` and `MACL`.
    pub fn set_mac(&self, mach: u32, macl: u32) {
        let mut s = self.session.lock();
        s.state.mach = mach;
        s.state.macl = macl;
    }

    /// The raw floating-point register banks, `FPRn_BANK0` and `FPRn_BANK1`.
    #[must_use]
    pub fn fpr(&self) -> [[u32; 16]; 2] {
        self.session.lock().state.fpr
    }

    /// Set one raw floating-point register, `bank` 0 or 1.
    pub fn set_fpr(&self, bank: usize, n: usize, value: u32) {
        self.session.lock().state.fpr[bank & 1][n & 15] = value;
    }

    /// `FPSCR`.
    #[must_use]
    pub fn fpscr(&self) -> u32 {
        self.session.lock().state.fpscr
    }

    /// Set `FPSCR`, masked as `LDS` masks it.
    pub fn set_fpscr(&self, value: u32) {
        self.session.lock().state.fpscr = value & fpscr::WRITABLE;
    }

    /// `FPUL`.
    #[must_use]
    pub fn fpul(&self) -> u32 {
        self.session.lock().state.fpul
    }

    /// A copy of the P4-resident control registers (`PTEH`, `MMUCR`,
    /// `EXPEVT`, `INTEVT`, ...).
    #[must_use]
    pub fn ctl(&self) -> Ctl {
        self.session.lock().state.ctl.clone()
    }

    /// A copy of the TLBs and the PMB.
    #[must_use]
    pub fn mmu(&self) -> Mmu {
        self.session.lock().state.mmu.clone()
    }

    /// Whether a `SLEEP` has the processor waiting for an interrupt.
    #[must_use]
    pub fn is_sleeping(&self) -> bool {
        self.session.lock().state.sleeping
    }

    /// Bus accesses charged since reset.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// How many accesses the address space refused. The SH-4A has no
    /// bus-error exception, so a refused read returns zero and a refused write
    /// is dropped; this is the evidence.
    #[must_use]
    pub fn bus_faults(&self) -> u64 {
        self.session.lock().state.faults
    }

    /// Present an interrupt request: priority `level` (1–15, 0 for none) with
    /// `intevt` as its `INTEVT` code. What an INTC model does through the
    /// [`ExportId::INTERRUPT_REQUEST`] cell.
    pub fn set_interrupt(&self, level: u32, intevt: u32) {
        self.lines.set_request(level, intevt);
    }

    /// Latch an NMI edge.
    pub fn raise_nmi(&self) {
        self.lines.raise_nmi();
    }

    /// Request a power-on reset; it happens at the next step.
    pub fn request_reset(&self) {
        self.lines.request_reset();
    }

    /// Execute one instruction (or take one exception or interrupt).
    ///
    /// Returns the bus accesses charged: at least one, or zero when there is
    /// no address space — which a caller must treat as "stop".
    pub fn step(&self) -> u64 {
        self.step_to_exit().0
    }

    /// Execute one instruction, reporting an [`Exit`] if it produced one.
    pub fn step_to_exit(&self) -> (u64, Option<Exit>) {
        let (used, exit) = self.advance(1, true);
        (used, exit)
    }

    /// Execute until at least `budget` accesses have been charged.
    pub fn run(&self, budget: u64) -> u64 {
        self.advance(budget, false).0
    }

    /// The run loop: one lock for the whole stretch, which is most of what an
    /// interpreter's throughput is made of.
    ///
    /// Runs until `allowance` accesses have been charged, or — with `single` —
    /// for exactly one step; stops early at an exit.
    fn advance(&self, allowance: u64, single: bool) -> (u64, Option<Exit>) {
        let cfg = self.effective_config();
        let exits = self.exit_mask();
        let mut session = self.session.lock();
        if self.lines.held() {
            // Held in reset: nothing executes, time still passes.
            let n = if single { 1 } else { allowance.max(1) };
            session.state.cycles = session.state.cycles.wrapping_add(n);
            return (n, None);
        }
        if self.lines.take_reset_request() {
            let (cycles, debt) = (session.state.cycles, session.state.debt);
            session.state = State::new(&cfg);
            session.state.cycles = cycles;
            session.state.debt = debt;
            let at = self.lines.reset_address.load(Ordering::Acquire);
            if at != NO_RESET_ADDRESS {
                session.state.pc = at as u32;
            }
        }
        let Session { state, space } = &mut *session;
        let Some(space) = space.clone() else {
            return (0, None);
        };
        let mut exec = Exec::new(state, &space, &cfg, &self.lines, exits);
        let mut used = 0u64;
        loop {
            if !single && exec.idle() {
                // Asleep with nothing pending: spend the rest of the budget
                // at once rather than a step at a time.
                let rest = allowance.saturating_sub(used).max(1);
                exec.st.cycles = exec.st.cycles.wrapping_add(rest);
                used += rest;
                break;
            }
            used += exec.step();
            if let Some(exit) = exec.take_exit() {
                return (used, Some(exit));
            }
            if single || used >= allowance {
                break;
            }
        }
        (used, None)
    }

    /// Run a scheduler budget, reporting exactly what was consumed — never
    /// more. The overshoot of the last instruction is carried as debt into
    /// the next budget, as every core in this tree does.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let (used, _) = self.advance(allowance, false);
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            ticks
        } else {
            self.session.lock().state.debt = 0;
            owed + used
        }
    }

    /// Accesses owed to the next budget.
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Run until the budget is exhausted or an armed trap leaves the core.
    pub fn run_to_exit_ticks(&self, ticks: u64) -> Run {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return Run::completed(Consumed::new(ticks));
        }
        let allowance = ticks - owed;
        let (used, exit) = self.advance(allowance, false);
        let total = owed + used;
        self.session.lock().state.debt = total.saturating_sub(ticks);
        match exit {
            Some(exit) => Run::exited(Consumed::new(total.min(ticks)), exit),
            None => Run::completed(Consumed::new(total.min(ticks))),
        }
    }

    /// Disassemble `count` instructions from `pc`, reading guest memory with
    /// debug attributes through the fixed P1/P2 mapping (never the TLB — which
    /// address space a debugger meant is not knowable).
    ///
    /// The `FPSCR.PR`/`SZ` in force now decide how FPU encodings print.
    #[must_use]
    pub fn disassemble(&self, pc: u32, count: usize) -> Vec<disasm::Disassembled> {
        let (space, fpscr) = {
            let s = self.session.lock();
            (s.space.clone(), s.state.fpscr)
        };
        let Some(space) = space else {
            return Vec::new();
        };
        let mode = disasm::Mode::from_fpscr(fpscr);
        disasm::disassemble_run(pc, count, mode, |addr| {
            let phys = if addr >> 29 == 7 {
                u64::from(addr)
            } else {
                u64::from(addr & 0x1fff_ffff)
            };
            space
                .read(phys, Width::U16, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u16)
        })
    }
}

/// The `cpu.sh4a` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.sh4a",
    version: 1,
    summary: "Renesas SH-4A interpreter: banked registers, exceptions, FPU, UTLB/ITLB/PMB MMU",
    properties: &[
        PropertySpec {
            name: "endian",
            kind: ValueKind::Str,
            required: false,
            summary: "the byte-order pin: `little` (default) or `big`",
        },
        PropertySpec {
            name: "reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the program counter starts (default 0xa0000000)",
        },
        PropertySpec {
            name: "pvr",
            kind: ValueKind::Uint,
            required: false,
            summary: "what the processor version register reads (default 0x10200000)",
        },
        PropertySpec {
            name: "prr",
            kind: ValueKind::Uint,
            required: false,
            summary: "what the product register reads (default 0)",
        },
        PropertySpec {
            name: "held",
            kind: ValueKind::Bool,
            required: false,
            summary: "start held in reset until the `hold` input falls (default false)",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists",
        },
    ],
    construct: |props| Ok(Box::new(Cpu::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Cpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let mut pins = self.pins.lock();
        match port {
            "nmi" => {
                let pin = Arc::new(NmiPin::new(Arc::clone(&self.lines), sources));
                pins.nmi = Some(Arc::clone(&pin));
                Some(SinkPin { sink: pin, line: 0 })
            }
            "reset" => {
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                Some(SinkPin { sink: pin, line: 0 })
            }
            "hold" => {
                let pin = Arc::new(HoldPin {
                    lines: Arc::clone(&self.lines),
                    inputs: FanIn::new(sources),
                });
                pins.hold = Some(Arc::clone(&pin));
                Some(SinkPin { sink: pin, line: 0 })
            }
            _ => None,
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        match which {
            ExportId::INTERRUPT_REQUEST => Some(Export::Cell(Arc::clone(&self.lines.request))),
            ExportId::RESET_ADDRESS => Some(Export::Cell(Arc::clone(&self.lines.reset_address))),
            _ => None,
        }
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let cfg = self.effective_config();
        let mut session = self.session.lock();
        session.state = State::new(&cfg);
        let at = self.lines.reset_address.load(Ordering::Acquire);
        if at != NO_RESET_ADDRESS {
            session.state.pc = at as u32;
        }
        drop(session);
        self.lines.take_reset_request();
        self.lines.take_nmi();
        if kind == ResetKind::Cold {
            // A cold start has nothing driving the request yet.
            self.lines.set_request(0, 0);
        }
        if self.cfg.held {
            self.lines.hold.store(true, Ordering::Release);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let session = self.session.lock();
        let s = &session.state;
        // The banks in canonical order, so the chunk does not depend on which
        // one happens to be visible.
        for v in s.bank0().iter().chain(s.bank1().iter()).chain(&s.r[8..]) {
            w.write_u32(*v)?;
        }
        for v in [
            s.sr, s.gbr, s.ssr, s.spc, s.sgr, s.dbr, s.vbr, s.mach, s.macl, s.pr, s.pc,
        ] {
            w.write_u32(v)?;
        }
        // Fixed-size even when no branch is pending, so every field after it
        // sits at a fixed offset — what a debugger's register map reads.
        w.write_bool(s.delay.is_some())?;
        w.write_u32(s.delay.unwrap_or(0))?;
        for bank in &s.fpr {
            for v in bank {
                w.write_u32(*v)?;
            }
        }
        w.write_u32(s.fpscr)?;
        w.write_u32(s.fpul)?;
        w.write_bool(s.ldst)?;
        w.write_bool(s.sleeping)?;
        let c = &s.ctl;
        for v in [
            c.pteh, c.ptel, c.ttb, c.tea, c.mmucr, c.ccr, c.tra, c.expevt, c.intevt, c.qacr0,
            c.qacr1, c.lmem[0], c.lmem[1], c.lmem[2], c.lmem[3], c.pascr, c.ramcr, c.irmcr,
            c.cpuopm,
        ] {
            w.write_u32(v)?;
        }
        for e in &s.mmu.utlb {
            w.write_u32(e.vpn)?;
            w.write_u32(e.ppn)?;
            w.write_u8(e.asid)?;
            w.write_u8(e.sz)?;
            w.write_u8(e.pr)?;
            w.write_u8(pack_flags(&[e.v, e.sh, e.c, e.d, e.wt, e.ub]))?;
        }
        for e in &s.mmu.itlb {
            w.write_u32(e.vpn)?;
            w.write_u32(e.ppn)?;
            w.write_u8(e.asid)?;
            w.write_u8(e.sz)?;
            w.write_u8(pack_flags(&[e.v, e.sh, e.c, e.pr]))?;
        }
        for e in &s.mmu.pmb {
            w.write_u32(e.vpn)?;
            w.write_u32(e.ppn)?;
            w.write_u8(e.sz)?;
            w.write_u8(pack_flags(&[e.v, e.c, e.ub, e.wt]))?;
        }
        for q in &s.sq {
            for v in q {
                w.write_u32(*v)?;
            }
        }
        w.write_u64(s.cycles)?;
        w.write_u64(s.debt)?;
        w.write_u64(s.faults)?;
        // The inputs are architectural: a restored machine whose controller
        // was already asserting a request must still see it.
        w.write_u64(self.lines.request.load(Ordering::Acquire))?;
        w.write_bool(self.lines.nmi.load(Ordering::Acquire))?;
        w.write_bool(self.lines.held())?;
        w.write_u64(self.lines.reset_address.load(Ordering::Acquire))?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cfg = self.effective_config();
        let mut s = State::new(&cfg);
        let mut b0 = [0u32; 8];
        let mut b1 = [0u32; 8];
        for v in b0.iter_mut().chain(b1.iter_mut()) {
            *v = r.read_u32()?;
        }
        for v in &mut s.r[8..] {
            *v = r.read_u32()?;
        }
        // `SR` first, raw, so the banks land where it says they are visible.
        s.sr = r.read_u32()? & sr::WRITABLE;
        s.set_banks(b0, b1);
        for slot in [
            &mut s.gbr,
            &mut s.ssr,
            &mut s.spc,
            &mut s.sgr,
            &mut s.dbr,
            &mut s.vbr,
            &mut s.mach,
            &mut s.macl,
            &mut s.pr,
            &mut s.pc,
        ] {
            *slot = r.read_u32()?;
        }
        let pending = r.read_bool()?;
        let target = r.read_u32()?;
        s.delay = pending.then_some(target);
        for bank in &mut s.fpr {
            for v in bank.iter_mut() {
                *v = r.read_u32()?;
            }
        }
        s.fpscr = r.read_u32()? & fpscr::WRITABLE;
        s.fpul = r.read_u32()?;
        s.ldst = r.read_bool()?;
        s.sleeping = r.read_bool()?;
        let c = &mut s.ctl;
        let [lm0, lm1, lm2, lm3] = &mut c.lmem;
        for slot in [
            &mut c.pteh,
            &mut c.ptel,
            &mut c.ttb,
            &mut c.tea,
            &mut c.mmucr,
            &mut c.ccr,
            &mut c.tra,
            &mut c.expevt,
            &mut c.intevt,
            &mut c.qacr0,
            &mut c.qacr1,
            lm0,
            lm1,
            lm2,
            lm3,
            &mut c.pascr,
            &mut c.ramcr,
            &mut c.irmcr,
            &mut c.cpuopm,
        ] {
            *slot = r.read_u32()?;
        }
        for e in &mut s.mmu.utlb {
            e.vpn = r.read_u32()?;
            e.ppn = r.read_u32()?;
            e.asid = r.read_u8()?;
            e.sz = r.read_u8()? & 3;
            e.pr = r.read_u8()? & 3;
            let f = r.read_u8()?;
            [e.v, e.sh, e.c, e.d, e.wt, e.ub] = unpack_flags(f);
        }
        for e in &mut s.mmu.itlb {
            e.vpn = r.read_u32()?;
            e.ppn = r.read_u32()?;
            e.asid = r.read_u8()?;
            e.sz = r.read_u8()? & 3;
            let f = r.read_u8()?;
            [e.v, e.sh, e.c, e.pr] = unpack_flags(f);
        }
        for e in &mut s.mmu.pmb {
            e.vpn = r.read_u32()?;
            e.ppn = r.read_u32()?;
            e.sz = r.read_u8()? & 3;
            let f = r.read_u8()?;
            [e.v, e.c, e.ub, e.wt] = unpack_flags(f);
        }
        for q in &mut s.sq {
            for v in q.iter_mut() {
                *v = r.read_u32()?;
            }
        }
        s.cycles = r.read_u64()?;
        s.debt = r.read_u64()?;
        s.faults = r.read_u64()?;
        let request = r.read_u64()?;
        let nmi = r.read_bool()?;
        let held = r.read_bool()?;
        let reset_address = r.read_u64()?;
        self.session.lock().state = s;
        self.lines.request.store(request, Ordering::Release);
        self.lines.nmi.store(nmi, Ordering::Release);
        self.lines.hold.store(held, Ordering::Release);
        self.lines
            .reset_address
            .store(reset_address, Ordering::Release);
        Ok(())
    }
}

/// Pack up to eight flags into a byte, first flag in bit 0.
fn pack_flags(flags: &[bool]) -> u8 {
    flags
        .iter()
        .enumerate()
        .fold(0, |acc, (i, f)| acc | (u8::from(*f) << i))
}

/// The inverse of [`pack_flags`].
fn unpack_flags<const N: usize>(byte: u8) -> [bool; N] {
    core::array::from_fn(|i| byte >> i & 1 != 0)
}

/// The level-3 seam (`ROADMAP.md` §2.1): `TRAPA` can leave the core as a
/// [`SYSCALL`](crate::core::exec::ExitReason::SYSCALL) exit, and the faults as
/// [`FAULT`](crate::core::exec::ExitReason::FAULT) exits, when armed.
impl ExitingCore for Cpu {
    fn exit_mask(&self) -> ExitMask {
        ExitMask::from_bits(self.exits.load(Ordering::Relaxed))
    }

    fn set_exit_mask(&self, mask: ExitMask) {
        self.exits.store(mask.bits(), Ordering::Relaxed);
    }

    fn run_to_exit(&self, budget: Budget) -> Run {
        self.run_to_exit_ticks(budget.ticks)
    }

    fn pc(&self) -> u64 {
        u64::from(Cpu::pc(self))
    }

    fn set_pc(&self, pc: u64) {
        Cpu::set_pc(self, pc as u32);
    }

    fn sp(&self) -> u64 {
        u64::from(self.reg(15))
    }

    fn set_sp(&self, sp: u64) {
        self.set_reg(15, sp as u32);
    }
}

impl Initiator for Cpu {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a processor needs an address space.
impl crate::machine::Instance for Cpu {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: "an SH-4A core needs an address space to fetch from (`space = mem`)"
                .to_string(),
        })?;
        self.attach_space(Arc::clone(space));
        self.set_requester(ctx.requester());
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Cpu::from_props(props)?)))
}

/// What the validator should know about `cpu.sh4a`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("endian", ValueKind::Str).values(&["little", "big"]))
        .prop(PropSchema::new("reset", ValueKind::Uint))
        .prop(PropSchema::new("pvr", ValueKind::Uint))
        .prop(PropSchema::new("prr", ValueKind::Uint))
        .prop(PropSchema::new("held", ValueKind::Bool))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .port("nmi", PortDir::In)
        .port("reset", PortDir::In)
        .port("hold", PortDir::In)
}

/// The NMI input: a rising level latches one NMI (§5.6.3, "NMI pin edge
/// detection").
#[derive(Debug)]
pub struct NmiPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    was_high: AtomicBool,
}

impl NmiPin {
    /// Connect the NMI input to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> NmiPin {
        NmiPin {
            lines,
            inputs: FanIn::new(sources),
            was_high: AtomicBool::new(false),
        }
    }
}

impl WireSink for NmiPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        if high && !self.was_high.swap(true, Ordering::AcqRel) {
            self.lines.raise_nmi();
        } else if !high {
            self.was_high.store(false, Ordering::Release);
        }
    }
}

/// The reset input: asserting it latches a power-on reset, taken at the next
/// step.
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
}

impl ResetPin {
    /// Connect the reset input to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
        ResetPin {
            lines,
            inputs: FanIn::new(sources),
        }
    }
}

impl WireSink for ResetPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        if self.inputs.resolve(Resolve::Or).is_high() {
            self.lines.request_reset();
        }
    }
}

/// The `hold` input: held in reset while high; the release is a reset from
/// the [`ExportId::RESET_ADDRESS`] cell's address, or the reset vector.
#[derive(Debug)]
pub struct HoldPin {
    lines: Arc<Lines>,
    inputs: FanIn,
}

impl WireSink for HoldPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.lines
            .set_hold(self.inputs.resolve(Resolve::Or).is_high());
    }
}

/// A description of this core for `rsemu describe cpu.sh4a`, built from
/// [`isa::TABLE`] so it cannot drift from what the interpreter implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for insn in isa::TABLE {
        let _ = writeln!(
            out,
            "{:04x}/{:04x} {:<8} {}",
            insn.bits,
            insn.mask,
            insn.op.mnemonic(),
            insn.op.summary()
        );
    }
    out
}
