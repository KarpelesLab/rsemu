//! AArch64 — an A64 integer interpreter with EL0/EL1 and the VMSAv8-64 MMU.
//!
//! # Why this is a third module and not a variant of `aprofile`
//!
//! `ROADMAP.md` §6.1.1 draws the boundary between cores at the place an
//! extension flag cannot reach, and answers its own question directly: A/R and
//! M "is one boundary, and one is the number we should be prepared to defend;
//! a third appears only if AArch64 lands, which shares even less." It shares
//! less, and here is the count. `aprofile` holds ten things AArch64 does not
//! have at all:
//!
//! | | `aprofile` (A32/T32) | `a64` |
//! | --- | --- | --- |
//! | registers | 16, PC among them | 31, plus `SP` and `XZR` as *encodings* |
//! | conditional execution | a field in every A32 instruction | four instructions |
//! | modes | seven, with banked registers and `SPSR` per mode | exception levels, one `SPSR_EL1` |
//! | status | `CPSR` — a register | `PSTATE` — fields with no register |
//! | system registers | CP15, a coprocessor with `MCR`/`MRC` | a flat `op0:op1:CRn:CRm:op2` space |
//! | exception entry | a mode switch and a banked `LR` | `ELR_EL1`/`SPSR_EL1`/`ESR_EL1` |
//! | vectors | eight words at a fixed base | sixteen slots at `VBAR_EL1`, 128 bytes apart |
//! | MMU | VMSAv5: one base, domains, sections | two bases, four levels, an access flag, no domains |
//! | the PC | a general-purpose register, readable and writable | not a register at all |
//! | instruction length | 4 or 2, with interworking | 4, always |
//!
//! Nothing in that table is a construction property. A `Variant` spanning it
//! would be `#[cfg]` wearing a different hat, and an ARM926EJ-S build would
//! link a four-level 64-bit page-table walker it can never run — exactly the
//! crate-shape violation §6.1.1 forbids. So: a third core, sharing the family
//! module and, for now, nothing else. There is genuinely nothing to share:
//! the barrel shifter §6.1.1 names as common has different semantics here (no
//! carry output, no register-controlled form), and the DSP and Thumb-1
//! subsets do not exist in A64 at all.
//!
//! # The lattice, within this core
//!
//! [`isa::Features`] is the same shape as the x86 core's `Features` and the
//! RISC-V core's `Extensions`: independently selectable flags, deliberately
//! **not** `Ord`, because `features >= X` is the bug the lattice exists to
//! prevent. Every row of the instruction table names the feature it needs, and
//! [`isa::decode`] filters against the instance's set — so `CAS` on a part
//! without `FEAT_LSE` raises `UNDEFINED`, which is how a guest probes for it.
//! Named parts are the public surface: `Config::CORTEX_A53`, not a
//! hand-assembled flag set.
//!
//! # What is here
//!
//! | Piece | Module |
//! | --- | --- |
//! | the one declarative instruction table and decode | [`isa`] |
//! | the disassembler generated from that table | [`disasm`] |
//! | the interpreter | `exec` |
//! | `PSTATE`, exception levels, the system-register table | [`sysreg`] |
//! | the stage-1 translation walk and the software TLB | [`mmu`] |
//! | the SIMD&FP register file, `FPCR`/`FPSR`, and Arm's IEEE rules | [`fp`] |
//!
//! # Floating point is software, and that is the point
//!
//! `ROADMAP.md` §9.1: guest floating point executed on *host* floating point
//! cannot give bit-identical results across hosts, because no two
//! architectures agree on NaN payloads or on flush-to-zero and wasm
//! canonicalises NaNs. So every `F*` instruction here goes through
//! [`crate::float`], the shared software IEEE-754 subsystem, with `FPCR`
//! mapped onto its `Env`. There is no host-float fast path, not even behind a
//! flag.
//!
//! `CPACR_EL1.FPEN` resets to zero, so a guest takes an exception on its first
//! floating-point instruction unless it enabled access first. That is the
//! architecture rather than an inconvenience: it is how a kernel discovers
//! that a process has started using the FPU.
//!
//! # What is deliberately absent
//!
//! **Advanced SIMD** — the *vector* instructions — is not implemented, and
//! `ID_AA64PFR0_EL1.AdvSIMD` says so while `.FP` says floating point is
//! present. DDI 0487 requires those two fields to agree, so that is a part
//! nobody makes; [`Config::id_aa64pfr0`] explains why reporting an impossible
//! part beats claiming a capability that would `UNDEF` on first use.
//!
//! Also absent: `FEAT_FP16` arithmetic (half precision exists here only as a
//! conversion format, which is Armv8.0-A), EL2 and EL3 (so `HVC` and `SMC` are
//! `UNDEFINED`), AArch32 at any level, the generic timer, `LDXP`/`STXP`, the
//! unprivileged `LDTR`/`STTR` family, pointer authentication, MTE, SVE,
//! big-endian data, and the `DC ZVA` block operation — `DCZID_EL0.DZP` says
//! so.

//!
//! # Timing
//!
//! Arm does not architecturally define instruction timing, so there is no
//! cycle table in this core. A cycle is charged *because a bus access
//! happened* — the fetch, each translation-table descriptor read, each load
//! and store.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487),
//! which Arm publishes openly: chapter C for the A64 instruction set,
//! chapter D for the system registers, the exception model and the AArch64
//! virtual memory system architecture. Nothing else was consulted; in
//! particular no emulator source of any licence was opened for any part of
//! this core (`ROADMAP.md` §1).

pub mod disasm;
mod exec;
pub mod fp;
pub mod isa;
pub mod mmu;
pub mod sysreg;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{
    DebugTranslation, Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::exec::{Exit, ExitMask, ExitingCore, Run};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed, ExitFlag, TickCursor};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, AtomicU64, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};
use isa::Features;
use mmu::Tlb;
use sysreg::{El, SysRegs};

/// The names a disassembler, gdb and the monitor print for the 31 general
/// registers.
///
/// `X30` is the link register and `X29` the frame pointer *by software
/// convention*, not architecturally — the AAPCS64 says so and the hardware
/// does not — so they are printed as `x29` and `x30` here and named in the
/// documentation instead. `X31` is absent on purpose: it is not a register.
pub const X_NAMES: [&str; 31] = [
    "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14",
    "x15", "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27",
    "x28", "x29", "x30",
];

/// Look a general register up by name: `x0`-`x30`, `w0`-`w30`, `lr` or `fp`.
///
/// `sp` and `xzr` are deliberately not here: neither is a general register,
/// and returning 31 for either would hand a caller a number that means
/// different things in different encodings.
#[must_use]
pub fn x_by_name(name: &str) -> Option<u32> {
    match name {
        "lr" => return Some(30),
        "fp" => return Some(29),
        _ => {}
    }
    let rest = name.strip_prefix('x').or_else(|| name.strip_prefix('w'))?;
    let n = rest.parse::<u32>().ok()?;
    (n < 31).then_some(n)
}

/// A named part: what a `.machine` file writes, and the configuration it
/// stands for.
///
/// A pair rather than a `Config` with a name field, because the name belongs
/// to the *catalogue* of parts and not to an instance — two boards may name
/// the same part, and a `Config` a caller built by hand has no name at all.
pub type Part = (&'static str, fn() -> Config);

/// How this core is configured.
///
/// Construction properties, never `#[cfg]`: `ROADMAP.md` §2 promises machines
/// with two different CPUs in one binary, and the difference between an
/// Armv8.0 part and an Armv8.2 one is these fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Which optional instruction-set features this part has.
    pub features: Features,
    /// Where the program counter starts after a reset.
    ///
    /// `RVBAR_EL1` on real silicon, and implementation defined — which is why
    /// it is a property of the board rather than a constant here.
    pub reset_vector: u64,
    /// What `MIDR_EL1` reports.
    pub midr: u64,
    /// What `MPIDR_EL1` reports.
    pub mpidr: u64,
    /// This core's identity in `MemAttrs::requester`.
    pub requester: RequesterId,
}

impl Config {
    /// `CTR_EL0`: 64-byte cache lines, PIPT instruction cache, and the RES1
    /// bit 31. There is no cache in this core; software still reads this to
    /// decide how much to flush, and a zero would tell it to flush in
    /// zero-byte steps.
    pub const CTR: u64 = 0x8444_c004;

    /// `ID_AA64MMFR0_EL1`: a 48-bit physical address range, 16-bit ASIDs, the
    /// 4 KiB granule supported and the 16 KiB and 64 KiB granules not.
    ///
    /// `TGran4 == 0b0000` means *supported* while `TGran16 == 0b0000` means
    /// *not supported*; the two fields use opposite conventions, which is a
    /// genuine trap in the architecture and the reason this constant is
    /// written out with its fields named rather than as a bare number.
    pub const ID_AA64MMFR0: u64 = 0x0000_0000_1000_0025;

    /// A bare Armv8.0-A part with no optional feature at all — **including no
    /// floating point**.
    ///
    /// Useful as a lower bound and as what a test builds: a guest that runs
    /// here runs everywhere. `FEAT_FP` really is optional in Armv8.0-A even
    /// though every part anybody ships has it, so this is also the
    /// configuration that proves the `ID_AA64PFR0_EL1.FP` gate and the
    /// `UNDEFINED` an absent feature must raise.
    #[must_use]
    pub const fn armv8_0() -> Config {
        Config {
            features: Features::NONE,
            reset_vector: 0,
            // Implementer 0x00 rather than Arm's 0x41: this is not a part
            // anybody makes, and claiming a real `MIDR` would be a lie a guest
            // can read.
            midr: 0x0000_0f00,
            mpidr: 0x8000_0000,
            requester: RequesterId::ANONYMOUS,
        }
    }

    /// Cortex-A53: Armv8.0-A with `FEAT_CRC32` and floating point, no
    /// `FEAT_LSE`.
    ///
    /// `MIDR_EL1` 0x410FD034 — implementer `0x41` (Arm), architecture `0xF`,
    /// part `0xD03`, revision r0p4.
    #[must_use]
    pub const fn cortex_a53() -> Config {
        Config {
            features: Features {
                lse: false,
                crc32: true,
                fp: true,
            },
            midr: 0x410f_d034,
            ..Config::armv8_0()
        }
    }

    /// Cortex-A72: Armv8.0-A with `FEAT_CRC32`, part `0xD08`.
    #[must_use]
    pub const fn cortex_a72() -> Config {
        Config {
            midr: 0x410f_d080,
            ..Config::cortex_a53()
        }
    }

    /// Neoverse N1: Armv8.2-A, so `FEAT_LSE` is mandatory rather than
    /// optional. Part `0xD0C`.
    ///
    /// The pair with [`Config::cortex_a53`] is the lattice in one line: two
    /// parts of the same profile where one decodes `CAS` and the other must
    /// not.
    #[must_use]
    pub const fn neoverse_n1() -> Config {
        Config {
            features: Features::ALL,
            midr: 0x410f_d0c0,
            ..Config::armv8_0()
        }
    }

    /// The named parts a `.machine` file may ask for.
    pub const PARTS: &'static [Part] = &[
        ("armv8.0", Config::armv8_0),
        ("cortex-a53", Config::cortex_a53),
        ("cortex-a72", Config::cortex_a72),
        ("neoverse-n1", Config::neoverse_n1),
    ];

    /// Look a named part up.
    #[must_use]
    pub fn by_name(name: &str) -> Option<Config> {
        Config::PARTS
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, build)| build())
    }

    /// The same configuration with a different reset vector.
    #[must_use]
    pub const fn with_reset_vector(mut self, pc: u64) -> Self {
        self.reset_vector = pc;
        self
    }

    /// The same configuration with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// `ID_AA64ISAR0_EL1`, built from [`Config::features`].
    ///
    /// This is the register a guest reads to decide whether to use `CAS` and
    /// `CRC32`, so it must agree with what [`isa::decode`] will accept. Both
    /// come from the same `Features`, which is the only way to keep them from
    /// disagreeing.
    #[must_use]
    pub const fn id_aa64isar0(&self) -> u64 {
        let mut value = 0u64;
        if self.features.crc32 {
            // CRC32 field, bits 19:16.
            value |= 1 << 16;
        }
        if self.features.lse {
            // Atomic field, bits 23:20: 0b0010 is FEAT_LSE.
            value |= 2 << 20;
        }
        value
    }

    /// `ID_AA64PFR0_EL1`: EL0 and EL1 implemented in AArch64 only, no EL2, no
    /// EL3, floating point as [`Features::fp`] says, and **Advanced SIMD
    /// absent**.
    ///
    /// # A combination no silicon has, reported deliberately
    ///
    /// DDI 0487 says the `FP` and `AdvSIMD` fields must hold the same value:
    /// a part has both or neither. This core has scalar floating point and no
    /// vector instructions, so it reports `FP == 0b0000` and
    /// `AdvSIMD == 0b1111` — a part that does not exist.
    ///
    /// The alternative was to claim Advanced SIMD and then raise `UNDEFINED`
    /// on the first `ADD V0.4S, …`, and between describing a part nobody makes
    /// and lying about a capability, the first is the one a guest can act on:
    /// software that checks `AdvSIMD` before using a vector `memcpy` gets the
    /// right answer, and software that assumes the fields agree finds out here
    /// rather than in a fault it cannot explain. When the vector instructions
    /// land the two fields agree again and this note goes away.
    #[must_use]
    pub const fn id_aa64pfr0(&self) -> u64 {
        // EL0 = 0b0001, EL1 = 0b0001 (AArch64 only); AdvSIMD = 0b1111.
        let mut value = 0x0000_0000_00f0_0011;
        if !self.features.fp {
            // 0b1111: not implemented.
            value |= 0xf << 16;
        }
        value
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::cortex_a53()
    }
}

/// The interrupt inputs and the reset request, outside the execution lock.
///
/// Atomics rather than fields of the session, for the same reason the RISC-V
/// core keeps its lines outside: a device raising `IRQ` from inside a write
/// the core itself issued must not re-enter the core's own critical section.
#[derive(Debug, Default)]
pub struct Lines {
    pending: AtomicU64,
    reset: AtomicBool,
}

impl Lines {
    /// The `IRQ` input.
    pub const IRQ: u64 = 1 << 0;
    /// The `FIQ` input.
    pub const FIQ: u64 = 1 << 1;

    /// Drive one input.
    pub fn set(&self, mask: u64, asserted: bool) {
        if asserted {
            self.pending.fetch_or(mask, Ordering::Relaxed);
        } else {
            self.pending.fetch_and(!mask, Ordering::Relaxed);
        }
    }

    /// Overwrite every input at once, which is what a snapshot restore does.
    pub fn set_all(&self, value: u64) {
        self.pending.store(value, Ordering::Relaxed);
    }

    /// Which inputs are asserted.
    #[must_use]
    pub fn pending(&self) -> u64 {
        self.pending.load(Ordering::Relaxed)
    }

    /// Latch a reset request.
    pub fn request_reset(&self) {
        self.reset.store(true, Ordering::Relaxed);
    }

    /// Take a latched reset request, clearing it.
    pub fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::Relaxed)
    }
}

/// Everything the interpreter mutates, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    /// Derived state: never serialized, invalidated by the translation
    /// generation counter (`ROADMAP.md` §4.5).
    tlb: Tlb,
    space: Option<Arc<AddressSpace>>,
}

/// One AArch64 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]: a CPU
/// is a bus master and holds this lock while calling into device models, which
/// take their own `DEVICE`-ranked locks. The interrupt lines are *not* under
/// it — they are atomics in [`Lines`].
#[derive(Debug)]
pub struct Cpu {
    cfg: Config,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// Which architectural traps leave the core instead of vectoring into the
    /// guest ([`ExitMask`]).
    ///
    /// An atomic rather than a field of [`Config`] because a consumer changes
    /// it while the core runs, and because it must survive a reset. It is
    /// deliberately not in the snapshot.
    exits: AtomicU32,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    requester: AtomicU32,
    /// The wire sinks handed out by [`Device::sink`], kept alive here — a net
    /// holds only a weak reference to a sink.
    pins: sync::Mutex<Pins>,
    /// *Unwind at your next instruction boundary* (`ROADMAP.md` §4.7).
    exit: sync::Mutex<Option<ExitFlag>>,
}

/// The sinks this core has published, one per input pin.
#[derive(Debug, Default)]
struct Pins {
    interrupts: Vec<(u64, Arc<InterruptPin>)>,
    reset: Option<Arc<ResetPin>>,
}

impl Cpu {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Cpu::attach_space) and [`Device::realize`].
    #[must_use]
    pub fn new(cfg: Config) -> Cpu {
        Cpu {
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    tlb: Tlb::new(),
                    space: None,
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
    /// If a property has the wrong type or an out-of-range value, if `cpu`
    /// names a part this core does not implement, or if a property nothing
    /// here accepts was given.
    pub fn from_props(props: &Props) -> Result<Cpu> {
        let mut r = props.reader();
        let names: Vec<&str> = Config::PARTS.iter().map(|(n, _)| *n).collect();
        let part = r.or_enum("cpu", "cortex-a53", &names)?;
        let reset_vector = r.or("reset", 0u64)?;
        let mpidr = r.or("mpidr", 0x8000_0000u64)?;
        // Accepted, and for now only one value is: `ROADMAP.md` §5's example
        // writes `engine = "interp"`, and there is no A64 IR frontend yet.
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;

        let cfg = Config::by_name(part).ok_or_else(|| {
            Error::Property(alloc::format!("`cpu` names an unknown part `{part}`"))
        })?;
        Ok(Cpu::new(Config {
            reset_vector,
            mpidr,
            ..cfg
        }))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the core the address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Set the id accesses this core initiates carry.
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

    /// Read one of `X0`-`X30`. Any other index reads zero, because there is no
    /// `X31`.
    #[must_use]
    pub fn x(&self, index: u32) -> u64 {
        if index >= 31 {
            return 0;
        }
        self.session.lock().state.x[index as usize]
    }

    /// Write one of `X0`-`X30`. Any other index is discarded.
    pub fn set_x(&self, index: u32, value: u64) {
        if index < 31 {
            self.session.lock().state.x[index as usize] = value;
        }
    }

    /// The program counter.
    #[must_use]
    pub fn pc(&self) -> u64 {
        self.session.lock().state.pc
    }

    /// Set the program counter.
    pub fn set_pc(&self, pc: u64) {
        self.session.lock().state.pc = pc;
    }

    /// The stack pointer the current `PSTATE` selects.
    #[must_use]
    pub fn sp(&self) -> u64 {
        self.session.lock().state.sys.sp()
    }

    /// Write the stack pointer the current `PSTATE` selects.
    pub fn set_sp(&self, value: u64) {
        self.session.lock().state.sys.set_sp(value);
    }

    /// Read a SIMD&FP register whole.
    ///
    /// The 128-bit value, because that is what the register is: a caller
    /// wanting the `S` or `D` view narrows it, and handing out a `u64` here
    /// would quietly discard the top half of a `Q` register a debugger asked
    /// for.
    #[must_use]
    pub fn v(&self, index: u32) -> u128 {
        if index >= fp::V_COUNT as u32 {
            return 0;
        }
        self.session.lock().state.v.q(index)
    }

    /// Write a SIMD&FP register whole. An index past the file is discarded.
    pub fn set_v(&self, index: u32, value: u128) {
        if index < fp::V_COUNT as u32 {
            self.session.lock().state.v.set_q(index, value);
        }
    }

    /// The current exception level.
    #[must_use]
    pub fn el(&self) -> El {
        self.session.lock().state.sys.el
    }

    /// A copy of `PSTATE` and the system registers, for a debugger or a test.
    #[must_use]
    pub fn sysregs(&self) -> SysRegs {
        self.session.lock().state.sys.clone()
    }

    /// Overwrite `PSTATE` and the system registers.
    ///
    /// The TLB is dropped as well, because the new `TTBR0_EL1` and `TCR_EL1`
    /// are almost certainly not the old ones.
    pub fn set_sysregs(&self, regs: SysRegs) {
        let mut session = self.session.lock();
        session.state.sys = regs;
        session.tlb.flush();
    }

    /// Bus accesses charged since reset.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a `WFI` is currently stalling the core.
    #[must_use]
    pub fn is_waiting(&self) -> bool {
        self.session.lock().state.wfi
    }

    /// How many accesses the address space refused.
    ///
    /// A diagnostic rather than the only evidence: an AArch64 core *can*
    /// report a bus fault to the guest, as an external-abort data abort.
    #[must_use]
    pub fn bus_faults(&self) -> u64 {
        self.session.lock().state.faults
    }

    /// How many TLB lookups hit and how many missed.
    #[must_use]
    pub fn tlb_stats(&self) -> (u64, u64) {
        self.session.lock().tlb.stats()
    }

    /// Drive one of the interrupt inputs directly.
    ///
    /// `mask` is [`Lines::IRQ`] or [`Lines::FIQ`]. This is the method a test
    /// or a hand-wired machine uses; a realized machine drives the same bits
    /// through [`InterruptPin`].
    pub fn set_interrupt(&self, mask: u64, asserted: bool) {
        self.lines.set(mask, asserted);
    }

    /// Which interrupt inputs are asserted.
    #[must_use]
    pub fn interrupts(&self) -> u64 {
        self.lines.pending()
    }

    /// Request a reset. It happens on the next [`step`](Cpu::step), because a
    /// reset is a signal rather than a method call.
    pub fn request_reset(&self) {
        self.lines.request_reset();
    }

    /// Execute one instruction, one exception entry, or one stalled `WFI`.
    ///
    /// Returns the bus accesses charged, which is at least one.
    pub fn step(&self) -> u64 {
        self.step_to_exit().0
    }

    /// Execute one instruction, reporting an [`Exit`] if it produced one.
    pub fn step_to_exit(&self) -> (u64, Option<Exit>) {
        let cfg = self.effective_config();
        let exits = self.exit_mask();
        let mut session = self.session.lock();
        if self.lines.take_reset_request() {
            session.state = State::new(&cfg);
            session.tlb.flush();
        }
        let Session { state, tlb, space } = &mut *session;
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
    /// usually runs past its end. The overshoot is *carried* — deducted from
    /// the next budget — which keeps the core's access count and the domain's
    /// tick count in step over any number of quanta while never letting a
    /// single one overrun.
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
            // `ROADMAP.md` §4.7's block-boundary check.
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
    pub fn attach_cursor(&self, cursor: &TickCursor) {
        *self.exit.lock() = Some(cursor.exit_flag());
    }

    /// Accesses owed to the next budget — see [`run_budget`](Cpu::run_budget).
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
        let mut used = 0u64;
        while used < allowance {
            let (n, exit) = self.step_to_exit();
            if n == 0 {
                break;
            }
            used += n;
            if let Some(exit) = exit {
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
    /// `None` is "the tables map nothing there". With `SCTLR_EL1.M` clear this
    /// is the identity and cannot fail.
    ///
    /// Side-effect free by construction rather than by care: the walk is handed
    /// an [`mmu::ReadDescriptor`], which has no write half at all, it does not
    /// consult or fill the core's TLB, it charges no cycles, and its descriptor
    /// reads carry [`MemAttrs::DEBUG`]. It is also permission-free — it answers
    /// where the page is, not whether an access would be allowed — so a page
    /// with its access flag clear still resolves.
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
    /// This is the one to hand [`pc`](Cpu::pc). Debug attributes are the
    /// point: a monitor listing the code around the program counter must not
    /// pop a FIFO or clear a status bit on the way, and neither must the walk
    /// that finds it.
    #[must_use]
    pub fn disassemble_virtual(&self, pc: u64, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let cfg = self.effective_config();
        let attrs = MemAttrs::DEBUG.with_requester(cfg.requester);
        disasm::disassemble_run(pc, count, cfg.features, |addr| {
            let phys = self
                .translate_debug(addr)
                .ok_or(disasm::Missing::Untranslated)?;
            space
                .read(phys, Width::U32, attrs)
                .map(|v| v as u32)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }

    /// Disassemble `count` instructions starting at the **physical** address
    /// `addr`.
    ///
    /// The untranslated form, and not a legacy shim: a monitor inspecting a
    /// firmware image and every board bring-up test genuinely want a bus
    /// address. The caller says which it means — that is why neither of these
    /// is called `disassemble`.
    #[must_use]
    pub fn disassemble_physical(&self, addr: u64, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let cfg = self.effective_config();
        let attrs = MemAttrs::DEBUG.with_requester(cfg.requester);
        disasm::disassemble_run(addr, count, cfg.features, |at| {
            space
                .read(at, Width::U32, attrs)
                .map(|v| v as u32)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }
}

/// The `cpu.arm.a64` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.arm.a64",
    version: 1,
    summary: "AArch64 A64 integer core with EL0/EL1, the VMSAv8-64 MMU and a disassembler",
    properties: &[
        PropertySpec {
            name: "cpu",
            kind: ValueKind::Str,
            required: false,
            summary: "which part: `armv8.0`, `cortex-a53`, `cortex-a72` or `neoverse-n1`",
        },
        PropertySpec {
            name: "reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "the address the program counter starts at (RVBAR_EL1; default 0)",
        },
        PropertySpec {
            name: "mpidr",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value MPIDR_EL1 reports, which is how an SMP guest tells cores apart",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists for A64",
        },
    ],
    construct: |props| Ok(Box::new(Cpu::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4).
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Which interrupt input a named pin drives.
fn pin_mask(port: &str) -> Option<u64> {
    match port {
        "irq" => Some(Lines::IRQ),
        "fiq" => Some(Lines::FIQ),
        _ => None,
    }
}

impl Device for Cpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    /// The debug surface's route to the MMU: this is how a gdb `m` packet
    /// naming a virtual address reaches the right physical one.
    fn debug_translate(&self, va: u64) -> DebugTranslation {
        let session = self.session.lock();
        if !session.state.sys.mmu_enabled() {
            return DebugTranslation::Identity;
        }
        drop(session);
        match self.translate_debug(va) {
            Some(pa) => DebugTranslation::Mapped(pa),
            None => DebugTranslation::Unmapped,
        }
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A core with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
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
        Cpu::attach_cursor(self, &cursor);
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
            self.lines.set_all(0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let session = self.session.lock();
        let s = &session.state;
        for r in s.x {
            w.write_u64(r)?;
        }
        // The SIMD&FP file, low half then high, one register at a time. Two
        // words rather than a 128-bit primitive because the chunk format has
        // no wider one, and the order is fixed here so `load` reads it back
        // the same way.
        for index in 0..fp::V_COUNT as u32 {
            let value = s.v.q(index);
            w.write_u64(value as u64)?;
            w.write_u64((value >> 64) as u64)?;
        }
        w.write_u64(s.pc)?;
        w.write_u64(s.cycles)?;
        w.write_u64(s.debt)?;
        w.write_u64(s.faults)?;
        w.write_bool(s.wfi)?;
        match s.exclusive {
            None => w.write_bool(false)?,
            Some(addr) => {
                w.write_bool(true)?;
                w.write_u64(addr)?;
            }
        }
        let sys = &s.sys;
        w.write_u32(sys.nzcv.0)?;
        w.write_u64(sys.daif)?;
        w.write_u8(u8::try_from(sys.el.bits()).unwrap_or(0))?;
        w.write_bool(sys.spsel)?;
        for value in Cpu::sysreg_words(sys) {
            w.write_u64(value)?;
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
        for index in 0..fp::V_COUNT as u32 {
            let lo = r.read_u64()?;
            let hi = r.read_u64()?;
            s.v.set_q(index, u128::from(lo) | (u128::from(hi) << 64));
        }
        s.pc = r.read_u64()?;
        s.cycles = r.read_u64()?;
        s.debt = r.read_u64()?;
        s.faults = r.read_u64()?;
        s.wfi = r.read_bool()?;
        s.exclusive = if r.read_bool()? {
            Some(r.read_u64()?)
        } else {
            None
        };
        s.sys.nzcv = isa::Nzcv(r.read_u32()?);
        s.sys.daif = r.read_u64()?;
        let el = r.read_u8()?;
        s.sys.el = match el {
            0 => El::El0,
            1 => El::El1,
            other => {
                return Err(Error::State(alloc::format!(
                    "snapshot names exception level {other}, which this core does not have"
                )));
            }
        };
        s.sys.spsel = r.read_bool()?;
        let mut words = [0u64; Cpu::SYSREG_WORDS];
        for slot in &mut words {
            *slot = r.read_u64()?;
        }
        Cpu::restore_sysreg_words(&mut s.sys, &words);
        let pending = r.read_u64()?;
        let mut session = self.session.lock();
        session.state = s;
        // The TLB is derived state and is never restored: it comes back empty,
        // which is always correct (`ROADMAP.md` §4.5).
        session.tlb.flush();
        drop(session);
        self.lines.set_all(pending);
        Ok(())
    }
}

impl Cpu {
    /// How many 64-bit system-register words a snapshot carries.
    const SYSREG_WORDS: usize = 24;

    /// The system registers a snapshot carries, in a fixed order.
    ///
    /// Written out as an array rather than field by field so that
    /// [`Cpu::restore_sysreg_words`] cannot drift from it: the two are one
    /// list read in opposite directions, and a register added to one without
    /// the other is a compile error rather than a corrupted snapshot.
    ///
    /// The identification registers are absent on purpose: they are derived
    /// from the configuration, and a snapshot that carried `MIDR_EL1` could
    /// restore a core claiming to be a part it is not.
    fn sysreg_words(s: &SysRegs) -> [u64; Cpu::SYSREG_WORDS] {
        [
            s.sp_el0,
            s.sp_el1,
            s.sctlr,
            s.actlr,
            s.cpacr,
            s.ttbr0,
            s.ttbr1,
            s.tcr,
            s.mair,
            s.amair,
            s.contextidr,
            s.spsr_el1,
            s.elr_el1,
            s.esr_el1,
            s.far_el1,
            s.vbar_el1,
            s.afsr0,
            s.afsr1,
            s.tpidr_el1,
            s.tpidr_el0,
            s.tpidrro_el0,
            s.mdscr,
            s.fpcr,
            s.fpsr,
        ]
    }

    /// The inverse of [`Cpu::sysreg_words`].
    fn restore_sysreg_words(s: &mut SysRegs, w: &[u64; Cpu::SYSREG_WORDS]) {
        let fields: [&mut u64; Cpu::SYSREG_WORDS] = [
            &mut s.sp_el0,
            &mut s.sp_el1,
            &mut s.sctlr,
            &mut s.actlr,
            &mut s.cpacr,
            &mut s.ttbr0,
            &mut s.ttbr1,
            &mut s.tcr,
            &mut s.mair,
            &mut s.amair,
            &mut s.contextidr,
            &mut s.spsr_el1,
            &mut s.elr_el1,
            &mut s.esr_el1,
            &mut s.far_el1,
            &mut s.vbar_el1,
            &mut s.afsr0,
            &mut s.afsr1,
            &mut s.tpidr_el1,
            &mut s.tpidr_el0,
            &mut s.tpidrro_el0,
            &mut s.mdscr,
            &mut s.fpcr,
            &mut s.fpsr,
        ];
        for (slot, value) in fields.into_iter().zip(w) {
            *slot = *value;
        }
    }
}

/// The level-3 seam (`ROADMAP.md` §2.1): a core that can stop *at* an `SVC`
/// and hand control out rather than vectoring to a guest handler.
///
/// Arming [`ExitMask::USER`] is what turns this from a machine's CPU into a
/// user-mode one. Nothing else about the core changes: the same interpreter,
/// the same address space, the same snapshot.
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
        Cpu::pc(self)
    }

    fn set_pc(&self, pc: u64) {
        Cpu::set_pc(self, pc);
    }

    fn sp(&self) -> u64 {
        Cpu::sp(self)
    }

    fn set_sp(&self, sp: u64) {
        Cpu::set_sp(self, sp);
    }
}

impl Initiator for Cpu {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a core needs an address space, and this is where
/// the machine gives it one.
impl crate::machine::Instance for Cpu {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: "an AArch64 core needs an address space to fetch from (`space = mem`)"
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

/// What the validator should know about `cpu.arm.a64`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let names: Vec<&'static str> = Config::PARTS.iter().map(|(n, _)| *n).collect();
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("cpu", ValueKind::Str).values(names.leak()))
        .prop(PropSchema::new("reset", ValueKind::Uint))
        .prop(PropSchema::new("mpidr", ValueKind::Uint))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        // Inputs only: this core drives no line.
        .port("irq", PortDir::In)
        .port("fiq", PortDir::In)
        .port("reset", PortDir::In)
}

/// One of the core's interrupt inputs, as something a wire can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, so this keeps a [`FanIn`] and wire-ORs the
/// sources — which is what a shared interrupt line does in hardware.
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    mask: u64,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect the pin selected by `mask` to a net driven by `sources`.
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

    /// Which input this pin drives.
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
        self.lines.set(self.mask, asserted);
    }
}

/// The core's reset input, as something a wire can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no mask and no handler. Asserting the line latches a request; the reset
/// itself happens on the next [`Cpu::step`].
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
        // button is still held should still come up.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core for `rsemu describe cpu.arm.a64`.
///
/// Built from [`isa::TABLE`] and [`sysreg::SYSREGS`], so it cannot drift from
/// what the interpreter implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for insn in isa::TABLE {
        let _ = writeln!(
            out,
            "{:08x}/{:08x} {:<9} {:<6} {}",
            insn.bits,
            insn.mask,
            insn.op.mnemonic(),
            insn.feat.name(),
            insn.op.summary()
        );
    }
    for spec in sysreg::SYSREGS {
        let _ = writeln!(
            out,
            "    sysreg {:04x} {:<18} {}",
            spec.enc,
            spec.reg.name(),
            spec.reg.summary()
        );
    }
    out
}
