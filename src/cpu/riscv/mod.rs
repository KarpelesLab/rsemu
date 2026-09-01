//! RISC-V — an RV64GC / RV32 interpreter with the privileged architecture.
//!
//! `ROADMAP.md` §6 picks RISC-V as the first architecture to boot a real
//! operating system, because it is the smallest credible target that boots
//! upstream Linux and because its entire specification set is freely and
//! legitimately available. This core is the interpreter half of that: the
//! oracle the IR frontend will be differentially tested against forever.
//!
//! # What is here
//!
//! | Piece | Module |
//! | --- | --- |
//! | the one declarative instruction table, decode, and `C` expansion | [`isa`] |
//! | the disassembler generated from that table | [`disasm`] |
//! | the IR frontend, behind `cpu-riscv-lift` | `lift` |
//! | the interpreter | `exec` |
//! | CSRs, privilege modes, trap causes, interrupt lines | [`csr`] |
//! | Sv39/Sv32 walk, PMP, the software TLB | [`mmu`] |
//! | software IEEE-754 binary32 and binary64 | [`crate::float`] |
//! | the `riscv-tests` runner and its ELF loader | `conformance`, [`elf`] |
//!
//! **RV64I and RV32I are the same core**, selected by [`Config::xlen`] — a
//! construction property, never a `#[cfg]`, so one build of rsemu runs an RV64
//! Linux machine and an RV32 microcontroller (`ROADMAP.md` §6). `M`, `A`, `F`,
//! `D` and `C` are individually selectable for the same reason.
//!
//! # Floating point is software, and that is the point
//!
//! `ROADMAP.md` §9.1 names a software IEEE-754 implementation as a deliverable
//! rather than an assumption, because guest floating point executed on host
//! floating point cannot be bit-identical across hosts. [`crate::float`] is
//! that implementation — it is shared with every other guest rather than owned
//! by this core, and RISC-V's rules reach it as
//! [`Env::RISCV`](crate::float::Env::RISCV) — and there is **no host-float
//! path in this core at all** —
//! not even behind a flag — so `F` and `D` results, including NaN payloads,
//! subnormals and the sticky `fcsr` flags, are reproducible on x86, AArch64
//! and wasm alike.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::riscv::{Config, Hart};
//!
//! // 64 KiB of RAM at 0, holding `addi a0, x0, 42`.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! for (i, b) in 0x02a0_0513u32.to_le_bytes().iter().enumerate() {
//!     ram.write_u8(i as u64, *b).unwrap();
//! }
//!
//! let space = AddressSpace::new("mem", 64);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let hart = Hart::new(Config::rv64gc().with_reset_vector(0));
//! hart.attach_space(Arc::new(space));
//! hart.step();
//! assert_eq!(hart.x(10), 42);
//! ```
//!
//! # Timing
//!
//! RISC-V does not architecturally define instruction timing, so there is no
//! cycle table anywhere in this core. A cycle is charged *because a bus access
//! happened* — an instruction fetch, a page-table read during a walk, a load,
//! a store — which is the accounting `ROADMAP.md` §6 asks for and the only
//! kind that is a fact rather than an invention.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual*, Volume I (Unprivileged ISA) and Volume
//! II (Privileged Architecture), both **CC-BY-4.0** and therefore quotable
//! with attribution — see `docs/cpu/riscv.md`. Nothing else was consulted; in
//! particular no emulator source of any licence was opened for any part of
//! this core.

pub mod csr;
pub mod disasm;
pub mod elf;
mod exec;
pub mod isa;

// The IR frontend needs both this core and `src/ir`, so it has its own feature
// rather than riding on either (ROADMAP.md §9).
#[cfg(feature = "cpu-riscv-lift")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-riscv-lift")))]
pub mod differential;
#[cfg(feature = "cpu-riscv-lift")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-riscv-lift")))]
pub mod lift;

pub mod mmu;

#[cfg(test)]
mod tests;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{
    DebugTranslation, Device, DeviceClass, ExportId, Initiator, PropertySpec, RealizeCtx,
    ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::exec::{Exit, ExitMask, ExitingCore, Run};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed, ExitFlag, TickCursor};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicU32, AtomicU64, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use csr::{Csrs, Extensions, Lines, Priv, irq};
use exec::{Exec, State};
use isa::Xlen;
use mmu::Tlb;

/// A mask of the bits below a page boundary.
pub(crate) const PAGE_MASK: u64 = mmu::PAGE_SIZE - 1;

/// The ABI names of the integer registers, in numeric order.
///
/// These are what a disassembler, gdb and the monitor print. `x8` has two
/// names — `s0` and `fp` — and the specification's ABI table lists `s0` first,
/// so that is the one used.
pub const X_NAMES: [&str; 32] = [
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4",
    "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
    "t5", "t6",
];

/// The ABI names of the floating-point registers, in numeric order.
pub const F_NAMES: [&str; 32] = [
    "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "fs0", "fs1", "fa0", "fa1", "fa2",
    "fa3", "fa4", "fa5", "fa6", "fa7", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9",
    "fs10", "fs11", "ft8", "ft9", "ft10", "ft11",
];

/// Look an integer register up by ABI or `xN` name.
#[must_use]
pub fn x_by_name(name: &str) -> Option<u32> {
    if let Some(rest) = name.strip_prefix('x')
        && let Ok(n) = rest.parse::<u32>()
        && n < 32
    {
        return Some(n);
    }
    if name == "fp" {
        return Some(8);
    }
    X_NAMES.iter().position(|n| *n == name).map(|i| i as u32)
}

/// How this particular hart is configured.
///
/// Construction properties, never `#[cfg]`: the difference between an RV64GC
/// application processor and an RV32IMAC microcontroller is these fields, and
/// one build of rsemu has to be able to run both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// The register and address width.
    pub xlen: Xlen,
    /// Which extensions and privilege modes are implemented.
    pub ext: Extensions,
    /// The value `mhartid` reports.
    pub hartid: u64,
    /// Where the program counter starts after a reset.
    pub reset_vector: u64,
    /// How many PMP entries are implemented.
    ///
    /// Zero means PMP is absent, which the specification defines as every
    /// access passing — the right choice for a hart with no firmware to
    /// program it. Any other count means an S-mode or U-mode access matching
    /// no entry is refused, which is what real hardware does and what OpenSBI
    /// expects.
    pub pmp_count: usize,
    /// Whether misaligned loads and stores are performed rather than trapped.
    ///
    /// Volume I leaves this to the implementation. Performing them is what
    /// most hardware does and what lets code written for such a hart run;
    /// turning it off is how you find out whether your guest depends on it.
    pub misaligned: bool,
    /// This hart's identity in `MemAttrs::requester`.
    pub requester: RequesterId,
}

impl Config {
    /// RV64GC with machine, supervisor and user mode and 16 PMP entries: the
    /// configuration a `virt` board's hart needs to boot Linux through
    /// OpenSBI.
    #[must_use]
    pub const fn rv64gc() -> Config {
        Config {
            xlen: Xlen::Rv64,
            ext: Extensions::GC,
            hartid: 0,
            reset_vector: 0x8000_0000,
            pmp_count: csr::PMP_ENTRIES,
            misaligned: true,
            requester: RequesterId::ANONYMOUS,
        }
    }

    /// RV32GC, otherwise identical to [`Config::rv64gc`].
    #[must_use]
    pub const fn rv32gc() -> Config {
        Config {
            xlen: Xlen::Rv32,
            ..Config::rv64gc()
        }
    }

    /// A bare machine-mode integer core with no extensions and no PMP: the
    /// smallest thing that runs, and what the unit tests use.
    #[must_use]
    pub const fn rv64i() -> Config {
        Config {
            xlen: Xlen::Rv64,
            ext: Extensions::I,
            pmp_count: 0,
            ..Config::rv64gc()
        }
    }

    /// The same configuration with a different reset vector.
    #[must_use]
    pub const fn with_reset_vector(mut self, pc: u64) -> Self {
        self.reset_vector = pc;
        self
    }

    /// The same configuration with a different hart id.
    #[must_use]
    pub const fn with_hartid(mut self, id: u64) -> Self {
        self.hartid = id;
        self
    }

    /// The same configuration with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// The same configuration with a different extension set.
    #[must_use]
    pub const fn with_ext(mut self, ext: Extensions) -> Self {
        self.ext = ext;
        self
    }

    /// The ISA string this configuration describes, as `misa` spells it.
    #[must_use]
    pub fn isa_string(&self) -> String {
        let mut s = String::from(self.xlen.name());
        s.push('i');
        for (present, letter) in [
            (self.ext.m, 'm'),
            (self.ext.a, 'a'),
            (self.ext.f, 'f'),
            (self.ext.d, 'd'),
            (self.ext.c, 'c'),
        ] {
            if present {
                s.push(letter);
            }
        }
        s
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::rv64gc()
    }
}

/// Everything the interpreter mutates, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    /// Derived state: never serialized, invalidated by generation
    /// (`ROADMAP.md` §4.5).
    tlb: Tlb,
    space: Option<Arc<AddressSpace>>,
    /// The platform timer, when a machine has one. Wiring rather than guest
    /// state, so it lives here beside `space` instead of in `state`: `reset`
    /// replaces `state`, and a reset must not unplug the clock.
    time_src: Option<Arc<AtomicU64>>,
}

/// One RISC-V hart.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`], for
/// the same reason the 6502's does: a CPU is a bus master and holds this lock
/// while calling into device models, which take their own `DEVICE`-ranked
/// locks. The interrupt lines are *not* under it — they are atomics in
/// [`Lines`] — so a PLIC raising an external interrupt from inside a write the
/// hart itself issued cannot re-enter the hart's own critical section.
#[derive(Debug)]
pub struct Hart {
    cfg: Config,
    /// The instance named by `timer = …`, if a machine file named one. Held as
    /// a path rather than a handle because two-phase construction forbids
    /// reaching a sibling from `new`; it is resolved at bind, through
    /// [`BindCtx::export_cell`](crate::machine::BindCtx::export_cell).
    timer: Option<String>,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// Which architectural traps leave the hart instead of vectoring into the
    /// guest ([`ExitMask`]).
    ///
    /// An atomic rather than a field of [`Config`] because a consumer changes
    /// it *while the hart runs* — a level-3 run loop arms it once, a debugger
    /// adds `BREAKPOINT` and takes it away again — and because it must survive
    /// a reset, which replaces the whole execution state. It is deliberately
    /// not in the snapshot: see [`ExitingCore::set_exit_mask`].
    exits: AtomicU32,
    /// This hart's identity in `MemAttrs::requester`, assigned at bind time.
    requester: AtomicU32,
    /// The wire sinks handed out by [`Device::sink`], kept alive here — a net
    /// holds only a weak reference to a sink, so the device owns the strong
    /// one.
    pins: sync::Mutex<Pins>,
    /// *Unwind at your next instruction boundary* (`ROADMAP.md` §4.7), handed
    /// over by the machine layer with the runnable's cursor.
    ///
    /// A leaf-ranked lock read once per budget rather than per instruction, and
    /// wiring rather than state, so it is not snapshotted.
    exit: sync::Mutex<Option<ExitFlag>>,
}

/// The sinks this hart has published, one per input pin.
#[derive(Debug, Default)]
struct Pins {
    interrupts: Vec<(u64, Arc<InterruptPin>)>,
    reset: Option<Arc<ResetPin>>,
}

impl Hart {
    /// A hart in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Hart::attach_space) and [`Device::realize`].
    #[must_use]
    pub fn new(cfg: Config) -> Hart {
        let mut cfg = cfg;
        // D without F is not a configuration the specification allows, and
        // silently honouring it would give a core whose FLW is illegal but
        // whose FLD is not.
        if cfg.ext.d {
            cfg.ext.f = true;
        }
        Hart {
            timer: None,
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    tlb: Tlb::new(),
                    space: None,
                    time_src: None,
                },
            ),
            exits: AtomicU32::new(ExitMask::NONE.bits()),
            requester: AtomicU32::new(cfg.requester.0),
            pins: sync::Mutex::new(Pins::default()),
            exit: sync::Mutex::new(None),
            cfg,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or an out-of-range value, or a
    /// property nothing here accepts was given — a typo'd property that was
    /// silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<Hart> {
        let mut r = props.reader();
        let xlen = match r.or_enum("xlen", "rv64", &["rv32", "rv64"])? {
            "rv32" => Xlen::Rv32,
            _ => Xlen::Rv64,
        };
        let isa = r.or("isa", String::from("imafdc"))?;
        let hartid = r.or("hartid", 0u64)?;
        let reset_vector = r.or("reset", 0x8000_0000u64)?;
        let pmp_count = r.or_range("pmp", csr::PMP_ENTRIES as u64, 0..=csr::PMP_ENTRIES as u64)?;
        let misaligned = r.or("misaligned", true)?;
        let supervisor = r.or("supervisor", true)?;
        let user = r.or("user", true)?;
        // Accepted, and for now only one value is: `ROADMAP.md` §5's example
        // writes `engine = "interp"`, and the IR frontend is phase 5.
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        // Validated here, resolved at bind: `new` allocates and checks, and
        // performs no outward action at all (`ROADMAP.md` §4.4).
        let timer = r.optional_link("timer")?.map(|l| String::from(l.as_str()));
        r.finish()?;

        let mut ext = Extensions {
            m: false,
            a: false,
            f: false,
            d: false,
            c: false,
            s: supervisor,
            u: user,
        };
        for letter in isa.chars() {
            match letter {
                'i' => {}
                'm' => ext.m = true,
                'a' => ext.a = true,
                'f' => ext.f = true,
                'd' => ext.d = true,
                'c' => ext.c = true,
                'g' => {
                    ext.m = true;
                    ext.a = true;
                    ext.f = true;
                    ext.d = true;
                }
                other => {
                    return Err(Error::Property(alloc::format!(
                        "`isa` names extension `{other}`, which this core does not \
                         implement; it understands i, m, a, f, d, c and g"
                    )));
                }
            }
        }
        let mut hart = Hart::new(Config {
            xlen,
            ext,
            hartid,
            reset_vector,
            pmp_count: pmp_count as usize,
            misaligned,
            requester: RequesterId::ANONYMOUS,
        });
        hart.timer = timer;
        Ok(hart)
    }

    /// This hart's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the hart the address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this hart executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Set the id accesses this hart initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// The configuration as it stands, with the bind-time requester folded in.
    fn effective_config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Read an integer register. `x0` reads as zero.
    #[must_use]
    pub fn x(&self, index: u32) -> u64 {
        self.session.lock().state.x[(index & 31) as usize]
    }

    /// Write an integer register. A write to `x0` is discarded.
    pub fn set_x(&self, index: u32, value: u64) {
        if index & 31 != 0 {
            let value = self.cfg.xlen.sext(value);
            self.session.lock().state.x[(index & 31) as usize] = value;
        }
    }

    /// Read a floating-point register, as raw bits.
    #[must_use]
    pub fn f(&self, index: u32) -> u64 {
        self.session.lock().state.f[(index & 31) as usize]
    }

    /// Write a floating-point register, as raw bits.
    pub fn set_f(&self, index: u32, value: u64) {
        self.session.lock().state.f[(index & 31) as usize] = value;
    }

    /// The program counter.
    #[must_use]
    pub fn pc(&self) -> u64 {
        self.session.lock().state.pc
    }

    /// Set the program counter.
    ///
    /// Truncated to the configured width rather than sign-extended: an RV32
    /// hart's address bus is 32 bits wide, so `0x8000_0000` is an address and
    /// not a negative number.
    pub fn set_pc(&self, pc: u64) {
        let pc = self.cfg.xlen.trunc(pc);
        self.session.lock().state.pc = pc;
    }

    /// The current privilege mode.
    #[must_use]
    pub fn priv_mode(&self) -> Priv {
        self.session.lock().state.csrs.priv_mode
    }

    /// A copy of the CSR file, for a debugger or a test.
    #[must_use]
    pub fn csrs(&self) -> Csrs {
        self.session.lock().state.csrs.clone()
    }

    /// Overwrite the CSR file.
    ///
    /// The TLB is dropped as well, because the new `satp` and `mstatus` are
    /// almost certainly not the old ones.
    pub fn set_csrs(&self, csrs: Csrs) {
        let mut session = self.session.lock();
        session.state.csrs = csrs;
        session.tlb.flush();
    }

    /// Bus accesses charged since reset.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Instructions retired since reset.
    #[must_use]
    pub fn instret(&self) -> u64 {
        self.session.lock().state.csrs.minstret
    }

    /// Whether a `WFI` is currently stalling the hart.
    #[must_use]
    pub fn is_waiting(&self) -> bool {
        self.session.lock().state.wfi
    }

    /// How many accesses the address space refused.
    ///
    /// Unlike a 6502, a RISC-V hart *can* report a bus fault to the guest — it
    /// becomes an access-fault exception — so this counter is a diagnostic
    /// rather than the only evidence. A machine whose memory map has a hole
    /// will show it climbing.
    #[must_use]
    pub fn bus_faults(&self) -> u64 {
        self.session.lock().state.faults
    }

    /// How many TLB lookups hit and how many missed.
    #[must_use]
    pub fn tlb_stats(&self) -> (u64, u64) {
        self.session.lock().tlb.stats()
    }

    /// Drive one of the interrupt-pending bits directly.
    ///
    /// `mask` is one of the [`irq`] constants. This is the method a test or a
    /// hand-wired machine uses; a realized machine drives the same bits
    /// through [`InterruptPin`].
    pub fn set_interrupt(&self, mask: u64, asserted: bool) {
        self.lines.set_pending(mask, asserted);
    }

    /// The current interrupt-pending register.
    #[must_use]
    pub fn interrupts(&self) -> u64 {
        self.lines.pending()
    }

    /// Request a reset. It happens on the next [`step`](Hart::step), because
    /// a reset is a signal rather than a method call.
    pub fn request_reset(&self) {
        self.lines.request_reset();
    }

    /// Set the value the `time` CSR reports.
    ///
    /// The platform timer belongs to a CLINT, not to the hart, so a machine
    /// without one supplies the value this way. Nothing in the core reads a
    /// host clock (`ROADMAP.md` §0).
    ///
    /// A hart with a timer attached through [`attach_time`](Hart::attach_time)
    /// overwrites this on its next step, which is the intended precedence:
    /// the CLINT owns `mtime` whenever there is a CLINT.
    pub fn set_time(&self, now: u64) {
        self.session.lock().state.csrs.mtime = now;
    }

    /// Attach the platform timer the `time` CSR reads.
    ///
    /// `time` is architecturally a *view of the platform timer*, not a counter
    /// the hart owns: Volume II defines it as a read-only shadow of the memory
    /// mapped `mtime`, which belongs to the CLINT. Handing the hart the same
    /// cell the CLINT publishes is what makes the two agree.
    ///
    /// Without this, `time` reads whatever [`set_time`](Hart::set_time) last
    /// left — zero, in a machine that never calls it. That is not a small
    /// inaccuracy: an operating system computes its next deadline as
    /// `time + delta`, the CLINT's `mtime` is already past it, the timer
    /// interrupt fires immediately, and the guest live-locks reprogramming a
    /// timer that is always expired.
    ///
    /// The value is sampled once per [`step`](Hart::step), which is as often
    /// as a guest can observe it: reading `time` takes an instruction.
    pub fn attach_time(&self, timer: Arc<AtomicU64>) {
        self.session.lock().time_src = Some(timer);
    }

    /// Execute one instruction, one trap entry, or one stalled `WFI` cycle.
    ///
    /// Returns the bus accesses charged, which is at least one — a caller can
    /// always make progress, and a stalled hart is visible through
    /// [`is_waiting`](Hart::is_waiting) rather than through a zero return.
    pub fn step(&self) -> u64 {
        self.step_to_exit().0
    }

    /// Execute one instruction, reporting an [`Exit`] if it produced one.
    ///
    /// The single-step half of [`ExitingCore::run_to_exit`]; `step` is this
    /// with the exit discarded, which is exactly right for a level-1 machine
    /// where the mask is empty and there is never one.
    pub fn step_to_exit(&self) -> (u64, Option<Exit>) {
        let cfg = self.effective_config();
        let exits = self.exit_mask();
        let mut session = self.session.lock();
        if self.lines.take_reset_request() {
            session.state = State::new(&cfg);
            session.tlb.flush();
        }
        let Session {
            state,
            tlb,
            space,
            time_src,
        } = &mut *session;
        if let Some(timer) = time_src {
            // Relaxed: `mtime` is a standalone counter, and nothing else the
            // CLINT publishes is ordered against it.
            state.csrs.mtime = timer.load(Ordering::Relaxed);
        }
        let Some(space) = space.clone() else {
            return (0, None);
        };
        let mut exec = Exec::new(state, tlb, &space, &cfg, &self.lines, exits);
        let used = exec.step();
        (used, exec.take_exit())
    }

    /// Execute until at least `budget` accesses have been charged.
    pub fn run(&self, budget: u64) -> u64 {
        let mut used = 0;
        while used < budget {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
        }
        used
    }

    /// Run a scheduler budget, reporting exactly what was consumed — never
    /// more.
    ///
    /// An instruction cannot be stopped half way, so the last one of a budget
    /// usually runs past its end. The scheduler treats an overrun as fatal,
    /// and rightly: the overrun has already executed past an event that should
    /// have stopped it. So the overshoot is *carried* — deducted from the next
    /// budget — which keeps the hart's access count and the domain's tick
    /// count in step over any number of quanta while never letting a single
    /// one overrun.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let exit = self.exit.lock().clone();
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
            // `ROADMAP.md` §4.7's block-boundary check. Nothing raises the flag
            // during an ordinary run, so this is bit-identical to the loop
            // without it until somebody stops the world, and returning early is
            // already a legal report — the `n == 0` arm above takes the same
            // exit and the debt bookkeeping below already handles it.
            if exit.as_ref().is_some_and(ExitFlag::raised) {
                break;
            }
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            ticks
        } else {
            self.session.lock().state.debt = 0;
            owed + used
        }
    }

    /// Take the safe point's exit flag out of the cursor the machine layer
    /// hands every runnable device.
    ///
    /// This hart does not publish its own position — nothing on a RISC-V board
    /// here is sampled inside an instruction the way a PPU is — so the cursor's
    /// *other* half is dropped and only the flag is kept.
    pub fn attach_cursor(&self, cursor: &TickCursor) {
        *self.exit.lock() = Some(cursor.exit_flag());
    }

    /// Accesses owed to the next budget — see [`run_budget`](Hart::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Run until the budget is exhausted or an armed trap leaves the hart.
    ///
    /// The level-3 run loop (`ROADMAP.md` §2.1). Scheduler debt is paid down
    /// exactly as [`run_budget`](Hart::run_budget) pays it, so a hart can be
    /// driven this way and by the machine scheduler without two tick
    /// accountings — and an exit reports only what it really consumed, because
    /// the caller resumes it rather than the scheduler.
    pub fn run_to_exit_ticks(&self, ticks: u64) -> Run {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return Run::completed(Consumed::new(ticks));
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let (n, exit) = self.step_to_exit();
            if n == 0 {
                break;
            }
            used += n;
            if let Some(exit) = exit {
                // An exit ends the quantum early. The instruction that exited
                // may still have overrun the budget, so the overshoot is
                // carried exactly as `run_budget` carries it: over any number
                // of quanta the reported ticks and the executed accesses stay
                // in step, and no single quantum ever overruns.
                let total = owed + used;
                self.session.lock().state.debt = total.saturating_sub(ticks);
                return Run::exited(Consumed::new(total.min(ticks)), exit);
            }
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            Run::completed(Consumed::new(ticks))
        } else {
            self.session.lock().state.debt = 0;
            Run::completed(Consumed::new(owed + used))
        }
    }

    /// Where a virtual address is mapped, as a debugger asks it.
    ///
    /// `None` is "the tables map nothing there". With `satp` in bare mode — and
    /// always in machine mode, which RISC-V never translates — this is the
    /// identity and cannot fail.
    ///
    /// Side-effect free by construction rather than by care: the walk is handed
    /// a [`mmu::ReadPte`] and so **cannot** set an accessed or dirty bit, it
    /// does not consult or fill the hart's TLB, it charges no cycles, and its
    /// descriptor reads carry [`MemAttrs::DEBUG`]. It is also permission-free —
    /// it answers where the page is, not whether an access would be allowed —
    /// so a supervisor with `SUM` clear can still be asked to show a user page.
    /// See [`Device::debug_translate`].
    #[must_use]
    pub fn translate_debug(&self, va: u64) -> Option<u64> {
        let cfg = self.effective_config();
        let session = self.session.lock();
        let space = session.space.as_ref()?;
        exec::debug_translate(&session.state, space, &cfg, va)
    }

    /// Disassemble `count` instructions starting at the **virtual** address
    /// `pc`, reading guest memory with debug attributes.
    ///
    /// This is the one to hand [`pc`](Hart::pc). Every halfword is translated
    /// through [`translate_debug`](Hart::translate_debug), so a listing that
    /// runs off the end of a mapped page carries on with a hole naming
    /// [`Missing::Untranslated`](disasm::Missing::Untranslated) rather than
    /// stopping — the count is always what was asked for.
    ///
    /// Debug attributes are the point: a monitor listing the code around the
    /// program counter must not pop a FIFO or clear a status bit on the way,
    /// and neither must the page-table walk that finds it.
    #[must_use]
    pub fn disassemble_virtual(&self, pc: u64, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let xlen = self.cfg.xlen;
        let attrs = MemAttrs::DEBUG.with_requester(self.effective_config().requester);
        disasm::disassemble_run(pc, count, xlen, |addr| {
            let phys = self
                .translate_debug(addr)
                .ok_or(disasm::Missing::Untranslated)?;
            space
                .read(phys, Width::U16, attrs)
                .map(|v| v as u16)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }

    /// Disassemble `count` instructions starting at the **physical** address
    /// `addr`.
    ///
    /// The untranslated form, and it is not a legacy shim: a monitor inspecting
    /// a firmware image, a board bring-up test and every conformance harness in
    /// the tree genuinely want a bus address. The caller says which it means —
    /// that is why neither of these is called `disassemble`.
    #[must_use]
    pub fn disassemble_physical(&self, addr: u64, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let xlen = self.cfg.xlen;
        let attrs = MemAttrs::DEBUG.with_requester(self.effective_config().requester);
        disasm::disassemble_run(addr, count, xlen, |at| {
            space
                .read(at, Width::U16, attrs)
                .map(|v| v as u16)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }
}

/// The `cpu.riscv` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.riscv",
    version: 1,
    summary: "RISC-V RV64GC / RV32 hart with M/S/U modes, Sv39 paging and software IEEE-754",
    properties: &[
        PropertySpec {
            name: "xlen",
            kind: ValueKind::Str,
            required: false,
            summary: "register width: `rv32` or `rv64` (default rv64)",
        },
        PropertySpec {
            name: "isa",
            kind: ValueKind::Str,
            required: false,
            summary: "extension letters beyond I: any of `mafdc`, or `g` for `imafd`",
        },
        PropertySpec {
            name: "hartid",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value `mhartid` reports",
        },
        PropertySpec {
            name: "reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "the address the program counter starts at (default 0x80000000)",
        },
        PropertySpec {
            name: "pmp",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many PMP entries are implemented; 0 means PMP is absent",
        },
        PropertySpec {
            name: "misaligned",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether misaligned loads and stores are performed rather than trapped",
        },
        PropertySpec {
            name: "supervisor",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether supervisor mode is implemented",
        },
        PropertySpec {
            name: "user",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether user mode is implemented",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
        PropertySpec {
            name: "timer",
            kind: ValueKind::Link,
            required: false,
            summary: "the object whose platform timer the `time` CSR reads (`timer = clint`)",
        },
    ],
    construct: |props| Ok(Box::new(Hart::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-riscv")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Which interrupt-pending bit a named input pin drives.
///
/// The names are the specification's, minus the trailing `P`: `meip` is the
/// machine external interrupt pin a PLIC drives, `mtip` the timer a CLINT
/// drives, `msip` the software interrupt, and the two supervisor pins the same
/// for a delegated platform.
fn pin_mask(port: &str) -> Option<u64> {
    match port {
        "meip" => Some(irq::MEI),
        "mtip" => Some(irq::MTI),
        "msip" => Some(irq::MSI),
        "seip" => Some(irq::SEI),
        "stip" => Some(irq::STI),
        _ => None,
    }
}

impl Device for Hart {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    /// The debug surface's route to the MMU: this is how a gdb `m` packet
    /// naming a virtual address reaches the right physical one.
    fn debug_translate(&self, va: u64) -> DebugTranslation {
        match self.translate_debug(va) {
            Some(pa) => DebugTranslation::Mapped(pa),
            None => DebugTranslation::Unmapped,
        }
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A hart with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this hart was made.
        let mut pins = self.pins.lock();
        if port == "reset" {
            let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
            pins.reset = Some(Arc::clone(&pin));
            return Some(SinkPin { sink: pin, line: 0 });
        }
        let mask = pin_mask(port)?;
        let pin = Arc::new(InterruptPin::new(Arc::clone(&self.lines), mask, sources));
        pins.interrupts.push((mask, Arc::clone(&pin)));
        Some(SinkPin {
            sink: pin,
            line: mask.trailing_zeros(),
        })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn attach_cursor(&self, cursor: TickCursor) {
        Hart::attach_cursor(self, &cursor);
    }

    fn reset(&self, kind: ResetKind) {
        let cfg = self.effective_config();
        let mut session = self.session.lock();
        session.state = State::new(&cfg);
        session.tlb.flush();
        drop(session);
        if kind == ResetKind::Cold {
            // A cold start has nothing driving the interrupt pins yet. A warm
            // one does, and clearing them would make the reset lie about the
            // machine.
            self.lines.set_all_pending(0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let session = self.session.lock();
        let s = &session.state;
        for r in s.x {
            w.write_u64(r)?;
        }
        for r in s.f {
            w.write_u64(r)?;
        }
        w.write_u64(s.pc)?;
        w.write_u64(s.cycles)?;
        w.write_u64(s.debt)?;
        w.write_u64(s.faults)?;
        w.write_bool(s.wfi)?;
        match s.reservation {
            None => w.write_bool(false)?,
            Some(addr) => {
                w.write_bool(true)?;
                w.write_u64(addr)?;
            }
        }
        let c = &s.csrs;
        w.write_u8(c.priv_mode.bits() as u8)?;
        for v in [
            c.mstatus,
            c.medeleg,
            c.mideleg,
            c.mie,
            c.mtvec,
            c.mcounteren,
            c.mcountinhibit,
            c.mscratch,
            c.mepc,
            c.mcause,
            c.mtval,
            c.menvcfg,
            c.stvec,
            c.scounteren,
            c.sscratch,
            c.sepc,
            c.scause,
            c.stval,
            c.satp,
            c.senvcfg,
            c.fcsr,
            c.minstret,
            c.mcycle,
            c.mtime,
        ] {
            w.write_u64(v)?;
        }
        w.write_u64(c.pmp_count as u64)?;
        for byte in c.pmpcfg {
            w.write_u8(byte)?;
        }
        for addr in c.pmpaddr {
            w.write_u64(addr)?;
        }
        // The interrupt lines are architectural: a restored machine whose
        // timer was already firing must still see it.
        w.write_u64(self.lines.pending())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cfg = self.effective_config();
        let mut s = State::new(&cfg);
        for slot in &mut s.x {
            *slot = r.read_u64()?;
        }
        // x0 is architecturally zero whatever a snapshot claims.
        s.x[0] = 0;
        for slot in &mut s.f {
            *slot = r.read_u64()?;
        }
        s.pc = r.read_u64()?;
        s.cycles = r.read_u64()?;
        s.debt = r.read_u64()?;
        s.faults = r.read_u64()?;
        s.wfi = r.read_bool()?;
        s.reservation = if r.read_bool()? {
            Some(r.read_u64()?)
        } else {
            None
        };
        let mode = r.read_u8()?;
        s.csrs.priv_mode = Priv::from_bits(u64::from(mode))
            .ok_or_else(|| Error::State(alloc::format!("unknown privilege mode {mode}")))?;
        let c = &mut s.csrs;
        for slot in [
            &mut c.mstatus,
            &mut c.medeleg,
            &mut c.mideleg,
            &mut c.mie,
            &mut c.mtvec,
            &mut c.mcounteren,
            &mut c.mcountinhibit,
            &mut c.mscratch,
            &mut c.mepc,
            &mut c.mcause,
            &mut c.mtval,
            &mut c.menvcfg,
            &mut c.stvec,
            &mut c.scounteren,
            &mut c.sscratch,
            &mut c.sepc,
            &mut c.scause,
            &mut c.stval,
            &mut c.satp,
            &mut c.senvcfg,
            &mut c.fcsr,
            &mut c.minstret,
            &mut c.mcycle,
            &mut c.mtime,
        ] {
            *slot = r.read_u64()?;
        }
        c.pmp_count = (r.read_u64()? as usize).min(csr::PMP_ENTRIES);
        for slot in &mut c.pmpcfg {
            *slot = r.read_u8()?;
        }
        for slot in &mut c.pmpaddr {
            *slot = r.read_u64()?;
        }
        let pending = r.read_u64()?;
        let mut session = self.session.lock();
        session.state = s;
        // The TLB is derived state and is never restored: it comes back empty,
        // which is always correct (`ROADMAP.md` §4.5).
        session.tlb.flush();
        drop(session);
        self.lines.set_all_pending(pending);
        Ok(())
    }
}

/// The level-3 seam (`ROADMAP.md` §2.1): a hart that can stop *at* an `ecall`
/// and hand control out rather than vectoring to a guest handler.
///
/// Arming [`ExitMask::USER`] is what turns this core from a machine's CPU into
/// a `qemu-user`-shaped one. Nothing else about the hart changes: the same
/// interpreter, the same address space, the same snapshot.
impl ExitingCore for Hart {
    fn exit_mask(&self) -> ExitMask {
        // Relaxed: the mask is configuration, and a change to it is ordered
        // against nothing but the next instruction fetch.
        ExitMask::from_bits(self.exits.load(Ordering::Relaxed))
    }

    fn set_exit_mask(&self, mask: ExitMask) {
        self.exits.store(mask.bits(), Ordering::Relaxed);
    }

    fn run_to_exit(&self, budget: Budget) -> Run {
        self.run_to_exit_ticks(budget.ticks)
    }

    fn pc(&self) -> u64 {
        Hart::pc(self)
    }

    fn set_pc(&self, pc: u64) {
        Hart::set_pc(self, pc);
    }

    fn sp(&self) -> u64 {
        self.x(2)
    }

    fn set_sp(&self, sp: u64) {
        self.set_x(2, sp);
    }
}

impl Initiator for Hart {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a hart needs an address space, and this is where
/// the machine gives it one.
impl crate::machine::Instance for Hart {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: "a RISC-V hart needs an address space to fetch from (`space = mem`)"
                .to_string(),
        })?;
        self.attach_space(Arc::clone(space));
        self.set_requester(ctx.requester());
        // `time` is a view of the platform timer, and the platform timer is
        // somebody else's device. A board that names one gets a `time` CSR that
        // agrees with the CLINT's `mtime`; a board that names none keeps
        // whatever `set_time` last left, which is what a machine with no timer
        // at all wants.
        if let Some(path) = &self.timer {
            self.attach_time(ctx.export_cell(path, ExportId::TIMEBASE)?);
        }
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Hart::from_props(props)?)))
}

/// What the validator should know about `cpu.riscv`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("xlen", ValueKind::Str).values(&["rv32", "rv64"]))
        .prop(PropSchema::new("isa", ValueKind::Str))
        .prop(PropSchema::new("hartid", ValueKind::Uint))
        .prop(PropSchema::new("reset", ValueKind::Uint))
        .prop(PropSchema::new("pmp", ValueKind::Uint).range(0, csr::PMP_ENTRIES as u64))
        .prop(PropSchema::new("misaligned", ValueKind::Bool))
        .prop(PropSchema::new("supervisor", ValueKind::Bool))
        .prop(PropSchema::new("user", ValueKind::Bool))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .prop(PropSchema::new("timer", ValueKind::Link))
        // Inputs only: a hart drives no line this core models.
        .port("meip", PortDir::In)
        .port("mtip", PortDir::In)
        .port("msip", PortDir::In)
        .port("seip", PortDir::In)
        .port("stip", PortDir::In)
        .port("reset", PortDir::In)
}

/// One of the hart's interrupt inputs, as something a wire can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, so this keeps a [`FanIn`] and wire-ORs the
/// sources — which is what a shared interrupt line does in hardware.
///
/// The pin keeps a handle on the hart's *input latches*, not on the hart: the
/// hart owns the pin, and a pin that owned the hart back would be a cycle the
/// machine could never drop.
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    mask: u64,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect the pin selected by `mask` — one of the [`irq`] constants — to
    /// a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, mask: u64, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            mask,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> Self {
        self.resolve = resolve;
        self
    }

    /// Which interrupt-pending bit this pin drives.
    #[must_use]
    pub fn mask(&self) -> u64 {
        self.mask
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InterruptPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.resolve(self.resolve).is_high();
        self.lines.set_pending(self.mask, asserted);
    }
}

/// The hart's reset input, as something a wire can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no `mip` bit, no mask and no handler. Asserting the line latches a request;
/// the reset itself happens on the next [`Hart::step`].
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect the reset pin to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
        ResetPin {
            lines,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for ResetPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Latch on assertion rather than on release: a machine whose reset
        // button is still held should still come up, instead of waiting for a
        // release nobody modelled.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core for `rsemu describe cpu.riscv`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for insn in isa::TABLE {
        let _ = writeln!(
            out,
            "{:08x}/{:08x} {:<10} {:<8} {}",
            insn.bits,
            insn.mask,
            insn.op.mnemonic(),
            insn.ext.name(),
            insn.op.summary()
        );
    }
    for insn in isa::CTABLE {
        let _ = writeln!(
            out,
            "    {:04x}/{:04x} {:<10} {:<8} {}",
            insn.bits,
            insn.mask,
            insn.op.mnemonic(),
            "c",
            insn.op.summary()
        );
    }
    out
}
