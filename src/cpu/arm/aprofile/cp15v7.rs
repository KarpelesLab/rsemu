//! CP15 for an ARMv7-A part: the Cortex-A9's system control coprocessor and
//! the VMSAv7 short-descriptor table walk.
//!
//! The ARMv7 sibling of [`cp15`](super::cp15), and a sibling rather than an
//! extension of it. VMSAv7 changed the descriptor format (execute-never,
//! supersections, `APX`, the access flag, `nG`), split the table base in two
//! (`TTBR0`/`TTBR1` under `TTBCR.N`), widened the fault status to five bits,
//! gave instruction aborts their own address register, and made nearly every
//! CP15 register PL1-only. A walk that handled both formats would be a walk
//! with a mode bit in every branch, and `docs/cpu/arm.md` already argued that
//! the second walk goes *beside* the first.
//!
//! # How a machine file asks for one
//!
//! `cpu = "cortex-a9"` on the `cpu.arm` object. That selects the ARMv7-A
//! architecture *and*, unless the object also says `cp15 = ...`, this
//! coprocessor — an A9 without its CP15 is not a part anyone built. A machine
//! that genuinely wants the bare core writes `cp15 = "none"`. The
//! multiprocessor identity comes from three more properties: `cpu-id` and
//! `cluster-id` become `MPIDR`, and `periphbase` becomes `CBAR`, the register
//! Linux and U-Boot read to find the SCU, the GIC and the private timers
//! (Cortex-A9 MPCore TRM, DDI 0407, 1.5 and 4.3.x "Configuration Base Address
//! Register").
//!
//! # What is modelled and what is not
//!
//! | Register | State |
//! | --- | --- |
//! | c0 identification | `MIDR`, `CTR`, `TCMTR`, `TLBTR`, `MPIDR`, `REVIDR`, the `ID_PFR`/`ID_DFR`/`ID_AFR`/`ID_MMFR`/`ID_ISAR` block, `CCSIDR`/`CLIDR`/`CSSELR`, `AIDR` |
//! | c1 | `SCTLR`: `M`, `A`, `V`, `TE`, `EE`, `AFE` live; `C`, `I`, `Z`, `SW`, `RR`, `TRE` stored. `ACTLR`, `CPACR` (reported to the core), `SCR`/`SDER`/`NSACR` stored |
//! | c2, c3 | `TTBR0`, `TTBR1`, `TTBCR` and `DACR`: live |
//! | c5, c6 | `DFSR`/`IFSR`/`DFAR`/`IFAR` latched on every abort; `ADFSR`/`AIFSR` stored |
//! | c7 | `PAR` and the `ATS1C**` VA-to-PA operations: live. Cache, branch-predictor and barrier operations: accepted, no-ops |
//! | c8 | every TLB operation invalidates the core's whole TLB; the inner-shareable forms invalidate the peers' too |
//! | c9 | the performance monitor: registers stored, **nothing counts** |
//! | c10 | `PRRR`, `NMRR`, TLB lockdown: stored |
//! | c12 | `VBAR` (live — the core reads it), `MVBAR` stored, `ISR` reads the interrupt inputs |
//! | c13 | `FCSEIDR` RAZ/WI (no FCSE on v7), `CONTEXTIDR`, the three thread-ID registers |
//! | c15 | power control stored, `CBAR` from `periphbase`, the A9's TLB-lockdown and cache-debug windows RAZ/WI |
//!
//! **No caches, no memory attributes.** TEX, C, B, S and the `PRRR`/`NMRR`
//! remap are decoded out of every descriptor and then ignored: they select a
//! cache policy and a memory type, and this machine has neither a cache nor a
//! reordering memory system for them to change. `CCSIDR` still describes the
//! A9's real 32 KiB caches, for the reason `cp15.rs` gives about the ARM926's
//! cache type register: software computes the stride of a set/way loop from
//! it, and a wrong stride is a wrong loop even when every operation in it is a
//! no-op.
//!
//! **Only the Secure state.** The Security Extensions' registers are present
//! and stored, but there is one bank of everything and the core never leaves
//! Secure state, which is where a Cortex-A9 comes out of reset and where a
//! bootloader that never executes `SMC` stays.
//!
//! # Privilege
//!
//! On ARMv7 an `MCR`/`MRC` to CP15 from User mode is Undefined, with a short
//! list of exceptions (DDI 0406C B3.17 and the per-register access tables in
//! B4.1): `TPIDRURW` read-write, `TPIDRURO` read-only, the three CP15
//! barriers (`c7, c5, 4`, `c7, c10, 4`, `c7, c10, 5`), and the performance
//! monitor when `PMUSERENR.EN` says so. [`CpOp::privileged`] is how this
//! coprocessor hears which mode asked.
//!
//! An `MCR` to a read-only register, and an access to an encoding the part
//! does not implement, is Undefined as well — that is ARMv7's rule, and
//! software probes for features by catching it. The exceptions are the
//! encodings v7 reserves as RAZ (the unallocated `c0` identification slots)
//! and the Cortex-A9's implementation-defined `c15` windows, which read as zero
//! and ignore writes because firmware written for the part touches them.
//!
//! # Several cores
//!
//! Each core of an MPCore cluster has its own CP15 and its own software TLB.
//! The inner-shareable TLB operations (`c8, c3, *`) are architecturally
//! broadcast to every core in the shareability domain, and `ID_MMFR3` says so
//! — which is what tells Linux it need not send an IPI for a TLB flush. So a
//! machine with more than one core **must** join their CP15s with
//! [`Cp15v7::join`] (or [`Arm::join_cluster`](super::Arm::join_cluster));
//! otherwise an inner-shareable flush on one core leaves the others' TLBs
//! stale.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition* (ARM DDI
//! 0406C): B3.5 for the short-descriptor formats and the walk, B3.6 and B3.7
//! for access permissions, the access flag, domains and execute-never, B3.9
//! for `TTBCR`, B3.12 for the order the checks happen in, B3.13 for the
//! fault-status encodings, B3.17/B3.18 for the CP15 register map, and B4.1 for
//! the individual registers. The identification values, `SCTLR`'s reset value,
//! the cache geometry, `ACTLR`, `CBAR` and the power control register are from
//! the *Cortex-A9 Technical Reference Manual* (ARM DDI 0388), chapter 4, and
//! the *Cortex-A9 MPCore TRM* (ARM DDI 0407). No emulator source of any licence
//! was consulted (`ROADMAP.md` §1).

use alloc::fmt;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicU32, LockRank, Ordering};
use crate::core::value::Endian;

use super::cp::{
    AccessKind, Coprocessor, CpEffect, CpFault, CpOp, CpResult, Fault, Mmu, Pa, PhysMem, Regime, Va,
};
use super::{Config, Lines};

/// `SCTLR`, the system control register (DDI 0406C B4.1.130).
pub mod sctlr {
    /// `M`, bit 0: the MMU is enabled.
    pub const M: u32 = 1 << 0;
    /// `A`, bit 1: alignment checking.
    pub const A: u32 = 1 << 1;
    /// `C`, bit 2: data and unified caches. Stored; no cache is modelled.
    pub const C: u32 = 1 << 2;
    /// `SW`, bit 10: `SWP`/`SWPB` enable. Stored.
    pub const SW: u32 = 1 << 10;
    /// `Z`, bit 11: branch prediction. Stored.
    pub const Z: u32 = 1 << 11;
    /// `I`, bit 12: instruction cache. Stored.
    pub const I: u32 = 1 << 12;
    /// `V`, bit 13: exception vectors at `0xffff0000` rather than `VBAR`.
    pub const V: u32 = 1 << 13;
    /// `RR`, bit 14: round-robin cache replacement. Stored.
    pub const RR: u32 = 1 << 14;
    /// `EE`, bit 25: exceptions are taken big-endian, and the table walk is
    /// big-endian. Only the first half is live; see [`super::Cp15v7`]'s walk.
    pub const EE: u32 = 1 << 25;
    /// `NMFI`, bit 27: FIQs are non-maskable. Read-only, from the `CFGNMFI`
    /// input, which this model ties low.
    pub const NMFI: u32 = 1 << 27;
    /// `TRE`, bit 28: TEX remap. Stored: it changes memory attributes only.
    pub const TRE: u32 = 1 << 28;
    /// `AFE`, bit 29: `AP[0]` is an access flag, and the simplified
    /// permission model applies.
    pub const AFE: u32 = 1 << 29;
    /// `TE`, bit 30: exceptions are taken in Thumb state.
    pub const TE: u32 = 1 << 30;

    /// The bits an ARMv7 `SCTLR` reads as one whatever is written: 3..6,
    /// 16, 18, 22 (`U` — unaligned support is always on in ARMv7) and 23
    /// (`XP` — the ARMv6 page-table format is the only one). Together they
    /// make the Cortex-A9's reset value `0x00c50078` (DDI 0388 4.3.9).
    pub const READ_AS_ONE: u32 = 0x00c5_0078;

    /// The bits a write can change on a Cortex-A9.
    pub const WRITABLE: u32 = M | A | C | SW | Z | I | V | RR | EE | TRE | AFE | TE;
}

/// Indices into the register file. Private: the public surface is the named
/// accessors, and the order here is the snapshot's byte order, so it only
/// ever grows at the end.
mod reg {
    pub(super) const SCTLR: usize = 0;
    pub(super) const ACTLR: usize = 1;
    pub(super) const CPACR: usize = 2;
    pub(super) const SCR: usize = 3;
    pub(super) const SDER: usize = 4;
    pub(super) const NSACR: usize = 5;
    pub(super) const TTBR0: usize = 6;
    pub(super) const TTBR1: usize = 7;
    pub(super) const TTBCR: usize = 8;
    pub(super) const DACR: usize = 9;
    pub(super) const DFSR: usize = 10;
    pub(super) const IFSR: usize = 11;
    pub(super) const ADFSR: usize = 12;
    pub(super) const AIFSR: usize = 13;
    pub(super) const DFAR: usize = 14;
    pub(super) const IFAR: usize = 15;
    pub(super) const PAR: usize = 16;
    pub(super) const CSSELR: usize = 17;
    pub(super) const PMCR: usize = 18;
    pub(super) const PMCNTEN: usize = 19;
    pub(super) const PMOVSR: usize = 20;
    pub(super) const PMSELR: usize = 21;
    pub(super) const PMCCNTR: usize = 22;
    /// Six event type registers, then six event counters.
    pub(super) const PMXEVTYPER: usize = 23;
    pub(super) const PMXEVCNTR: usize = 29;
    pub(super) const PMUSERENR: usize = 35;
    pub(super) const PMINTEN: usize = 36;
    pub(super) const PRRR: usize = 37;
    pub(super) const NMRR: usize = 38;
    pub(super) const TLB_LOCKDOWN: usize = 39;
    pub(super) const VBAR: usize = 40;
    pub(super) const MVBAR: usize = 41;
    pub(super) const CONTEXTIDR: usize = 42;
    pub(super) const TPIDRURW: usize = 43;
    pub(super) const TPIDRURO: usize = 44;
    pub(super) const TPIDRPRW: usize = 45;
    pub(super) const POWER: usize = 46;
    pub(super) const COUNT: usize = 47;
}

/// How many event counters the A9's performance monitor has (`PMCR.N`).
const PMU_COUNTERS: u32 = 6;

/// The counter-enable, overflow and interrupt-enable bits that exist: the
/// cycle counter in bit 31 and one bit per event counter.
const PMU_MASK: u32 = 0x8000_0000 | ((1 << PMU_COUNTERS) - 1);

/// The Cortex-A9's identification values (DDI 0388 4.3, r3p0).
pub mod id {
    /// `MIDR`: implementer ARM, variant 3, architecture "see the ID
    /// registers" (`0xf`), part `0xc09`, revision 0.
    pub const MIDR: u32 = 0x413f_c090;
    /// `CTR`: the ARMv7 format; 32-byte minimum lines in both caches, a
    /// 32-byte exclusives reservation granule and writeback granule, and a
    /// VIPT instruction cache.
    pub const CTR: u32 = 0x8333_8003;
    /// `ID_PFR0`: ARM and Thumb-2 state, trivial Jazelle (`BXJ` behaves as
    /// `BX`). ThumbEE is reported **absent** although the A9 has it: this core
    /// does not implement it, and an OS that saw it advertised would save and
    /// restore its CP14 registers on every context switch — and take an
    /// Undefined Instruction exception on the first one.
    pub const ID_PFR0: u32 = 0x0000_0231;
    /// `ID_PFR1`: the Security Extensions, and the standard programmers'
    /// model.
    pub const ID_PFR1: u32 = 0x0000_0011;
    /// `ID_DFR0`: **zero**, where the A9 reports `0x00010444`. The debug
    /// architecture is reached through CP14, which this core does not have;
    /// advertising v7 debug would send a kernel's hardware-breakpoint probe
    /// into an Undefined Instruction exception.
    pub const ID_DFR0: u32 = 0;
    /// `ID_AFR0`: nothing auxiliary.
    pub const ID_AFR0: u32 = 0;
    /// `ID_MMFR0`: VMSAv7, outer and inner shareability, the auxiliary
    /// control register.
    pub const ID_MMFR0: u32 = 0x0010_0103;
    /// `ID_MMFR1`: branch predictor maintenance required.
    pub const ID_MMFR1: u32 = 0x2000_0000;
    /// `ID_MMFR2`: `WFI` stalling, the CP15 barriers, the unified-TLB
    /// maintenance operations.
    pub const ID_MMFR2: u32 = 0x0123_0000;
    /// `ID_MMFR3`: set/way and by-MVA maintenance, and — bits 15..12 = 2 —
    /// hardware broadcast of the inner-shareable maintenance operations.
    pub const ID_MMFR3: u32 = 0x0010_2111;
    /// `ID_ISAR0`: bits 27..24 zero — **no hardware divide** in either
    /// instruction set.
    pub const ID_ISAR0: u32 = 0x0010_1111;
    /// `ID_ISAR1`.
    pub const ID_ISAR1: u32 = 0x1311_2111;
    /// `ID_ISAR2`.
    pub const ID_ISAR2: u32 = 0x2123_2041;
    /// `ID_ISAR3`.
    pub const ID_ISAR3: u32 = 0x1111_2131;
    /// `ID_ISAR4`.
    pub const ID_ISAR4: u32 = 0x0011_1142;
    /// `ID_ISAR5`.
    pub const ID_ISAR5: u32 = 0;
    /// `CLIDR`: one level of separate instruction and data caches, with the
    /// level of coherency, of unification and of inner-shareable unification
    /// all at 1 — so a set/way "flush everything" loop visits level 0 and
    /// stops.
    pub const CLIDR: u32 = 0x0920_0003;
    /// `CCSIDR` for the 32 KiB, four-way, 32-byte-line data cache: 256 sets,
    /// write-back, read- and write-allocate.
    pub const CCSIDR_DATA: u32 = 0x701f_e019;
    /// `CCSIDR` for the 32 KiB, four-way, 32-byte-line instruction cache.
    pub const CCSIDR_INSN: u32 = 0x201f_e019;
    /// `PRRR` out of reset.
    pub const PRRR: u32 = 0x0009_8aa4;
    /// `NMRR` out of reset.
    pub const NMRR: u32 = 0x44e0_48e0;
    /// `PMCR`'s read-only half: implementer ARM, ID code `0x09`, six
    /// counters.
    pub const PMCR: u32 = 0x4109_0000 | (super::PMU_COUNTERS << 11);
}

/// One of the three access-permission outcomes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Allow {
    None,
    Read,
    Write,
}

impl Allow {
    /// Whether this permits `kind`. A fetch needs read permission; `XN` is
    /// checked separately.
    const fn permits(self, kind: AccessKind) -> bool {
        match kind {
            AccessKind::Write => matches!(self, Allow::Write),
            _ => matches!(self, Allow::Read | Allow::Write),
        }
    }
}

/// Whether a walk enforces the checks or only resolves the mapping. See
/// `cp15.rs`'s type of the same name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Checks {
    Guest(AccessKind, bool),
    None,
}

/// Which level produced the leaf, so a fault can name itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    Section,
    Page,
}

impl Level {
    const fn access_flag(self) -> Fault {
        match self {
            Level::Section => Fault::ACCESS_FLAG_SECTION,
            Level::Page => Fault::ACCESS_FLAG_PAGE,
        }
    }

    const fn domain(self) -> Fault {
        match self {
            Level::Section => Fault::DOMAIN_SECTION,
            Level::Page => Fault::DOMAIN_PAGE,
        }
    }

    const fn permission(self) -> Fault {
        match self {
            Level::Section => Fault::PERMISSION_SECTION,
            Level::Page => Fault::PERMISSION_PAGE,
        }
    }
}

/// What a successful walk found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Leaf {
    pa: Pa,
    /// A 16 MiB supersection, which `PAR.SS` reports.
    supersection: bool,
    /// The descriptor's `NS` bit.
    non_secure: bool,
    /// The descriptor's `S` bit.
    shareable: bool,
}

/// Physical memory with nothing in it, for an `MCR` issued without a view of
/// memory (see [`Cp15v7::mcr`]).
struct NoMem;

impl PhysMem for NoMem {
    fn read_u32(&self, _at: Pa) -> Option<u32> {
        None
    }
}

/// The Cortex-A9's system control coprocessor.
///
/// Every register is an atomic, for the reason [`Cp15`](super::cp15::Cp15)
/// gives: the core samples [`Mmu::regime`] once per instruction and the walk
/// reads three or four of these per miss, and there is no invariant between
/// two registers that a lock would be protecting.
pub struct Cp15v7 {
    regs: [AtomicU32; reg::COUNT],
    /// What [`reset`](Cp15v7::reset) puts back.
    reset: [u32; reg::COUNT],
    /// `MPIDR`, built from the `cpu-id` and `cluster-id` properties.
    mpidr: u32,
    /// `CBAR`, from the `periphbase` property.
    periphbase: u32,
    /// Which `CPACR` bits a write can change, and which read as one, given
    /// which floating-point options the part has.
    cpacr_writable: u32,
    cpacr_one: u32,
    /// The core's interrupt inputs, for `ISR`. `None` for a coprocessor built
    /// outside a core, whose `ISR` then reads zero.
    lines: Option<Arc<Lines>>,
    /// Bumped by anything that could invalidate a cached translation.
    ///
    /// Shared (an `Arc`) so that a peer's inner-shareable TLB operation can
    /// bump it without reaching into this core's lock.
    generation: Arc<AtomicU32>,
    /// The other cores' generations, for the inner-shareable operations.
    /// Touched only by a `c8, c3` operation and by [`join`](Cp15v7::join),
    /// so a lock is fine here.
    peers: sync::Mutex<Vec<Arc<AtomicU32>>>,
}

impl fmt::Debug for Cp15v7 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cp15v7")
            .field("sctlr", &self.sctlr())
            .field("ttbr0", &self.ttbr0())
            .field("ttbr1", &self.ttbr1())
            .field("ttbcr", &self.ttbcr())
            .field("mpidr", &self.mpidr)
            .field("periphbase", &self.periphbase)
            .finish_non_exhaustive()
    }
}

impl Cp15v7 {
    /// A Cortex-A9 CP15 with `cfg`'s straps as its reset state.
    ///
    /// `high-vectors` is `VINITHI`, `big-endian` is `CFGEND0` (which sets
    /// `SCTLR.EE`), and `alignment-faults` sets `SCTLR.A` — each the *reset
    /// value* of a bit software owns afterwards. `MPIDR` and `CBAR` come from
    /// [`Config::cpu_id`], [`Config::cluster_id`] and [`Config::periphbase`].
    #[must_use]
    pub fn cortex_a9(cfg: &Config) -> Cp15v7 {
        let mut sctlr = sctlr::READ_AS_ONE;
        if cfg.high_vectors {
            sctlr |= sctlr::V;
        }
        if cfg.alignment_faults {
            sctlr |= sctlr::A;
        }
        if cfg.endian == Endian::Big {
            sctlr |= sctlr::EE;
        }
        let mut reset = [0u32; reg::COUNT];
        reset[reg::SCTLR] = sctlr;
        reset[reg::PRRR] = id::PRRR;
        reset[reg::NMRR] = id::NMRR;

        // CPACR (DDI 0406C B4.1.40): only cp10 and cp11 exist on an A9, plus
        // the two Advanced SIMD / D32 disables. A disable for an option the
        // part lacks reads as one and ignores writes — "that thing is
        // disabled" is true of something that is not there.
        let (mut writable, mut one) = (0u32, 0u32);
        if let Some(vfp) = cfg.arch.ext.vfp {
            writable |= 0x00f0_0000; // cp10 and cp11
            if vfp.d32 {
                writable |= 1 << 30;
            } else {
                one |= 1 << 30;
            }
            if cfg.arch.ext.neon {
                writable |= 1 << 31;
            } else {
                one |= 1 << 31;
            }
        }
        reset[reg::CPACR] = one;

        Cp15v7 {
            regs: core::array::from_fn(|i| AtomicU32::new(reset[i])),
            reset,
            // Bit 31 set: the multiprocessor format. `U` (bit 30) clear: part
            // of a cluster, even a one-core one (DDI 0406C B4.1.106).
            mpidr: 0x8000_0000 | (u32::from(cfg.cluster_id & 0xf) << 8) | u32::from(cfg.cpu_id & 3),
            periphbase: cfg.periphbase & 0xffff_e000,
            cpacr_writable: writable,
            cpacr_one: one,
            lines: None,
            generation: Arc::new(AtomicU32::new(0)),
            peers: sync::Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The same coprocessor, reading `ISR` from the core's interrupt inputs.
    pub(crate) fn with_lines(mut self, lines: Arc<Lines>) -> Cp15v7 {
        self.lines = Some(lines);
        self
    }

    /// Put `self` and `other` in one inner-shareable domain, so each one's
    /// broadcast TLB operations invalidate the other's TLB.
    ///
    /// Joining is pairwise; a four-core cluster joins every pair. Joining a
    /// coprocessor to itself, or the same pair twice, is harmless.
    pub fn join(&self, other: &Cp15v7) {
        if Arc::ptr_eq(&self.generation, &other.generation) {
            return;
        }
        for (me, them) in [(self, other), (other, self)] {
            let mut peers = me.peers.lock();
            if !peers.iter().any(|p| Arc::ptr_eq(p, &them.generation)) {
                peers.push(Arc::clone(&them.generation));
            }
        }
    }

    /// Return every register to its reset value.
    pub fn reset(&self) {
        for (reg, value) in self.regs.iter().zip(self.reset) {
            reg.store(value, Ordering::Release);
        }
        self.invalidate();
    }

    #[inline]
    fn get(&self, r: usize) -> u32 {
        self.regs[r].load(Ordering::Acquire)
    }

    fn set(&self, r: usize, value: u32) {
        self.regs[r].store(value, Ordering::Release);
    }

    /// `SCTLR`.
    #[must_use]
    pub fn sctlr(&self) -> u32 {
        self.get(reg::SCTLR)
    }

    /// `TTBR0`.
    #[must_use]
    pub fn ttbr0(&self) -> u32 {
        self.get(reg::TTBR0)
    }

    /// `TTBR1`.
    #[must_use]
    pub fn ttbr1(&self) -> u32 {
        self.get(reg::TTBR1)
    }

    /// `TTBCR`.
    #[must_use]
    pub fn ttbcr(&self) -> u32 {
        self.get(reg::TTBCR)
    }

    /// `DACR`.
    #[must_use]
    pub fn dacr(&self) -> u32 {
        self.get(reg::DACR)
    }

    /// `DFSR` and `DFAR`.
    #[must_use]
    pub fn data_fault(&self) -> (u32, u32) {
        (self.get(reg::DFSR), self.get(reg::DFAR))
    }

    /// `IFSR` and `IFAR`.
    #[must_use]
    pub fn instruction_fault(&self) -> (u32, u32) {
        (self.get(reg::IFSR), self.get(reg::IFAR))
    }

    /// `PAR`, the result of the last VA-to-PA operation.
    #[must_use]
    pub fn par(&self) -> u32 {
        self.get(reg::PAR)
    }

    /// `VBAR`.
    #[must_use]
    pub fn vbar(&self) -> u32 {
        self.get(reg::VBAR)
    }

    /// `CPACR`.
    #[must_use]
    pub fn cpacr(&self) -> u32 {
        self.get(reg::CPACR)
    }

    /// `CONTEXTIDR`.
    #[must_use]
    pub fn contextidr(&self) -> u32 {
        self.get(reg::CONTEXTIDR)
    }

    /// `MPIDR`.
    #[must_use]
    pub fn mpidr(&self) -> u32 {
        self.mpidr
    }

    /// `CBAR`: where the SCU, the GIC and the private timers are.
    #[must_use]
    pub fn periphbase(&self) -> u32 {
        self.periphbase
    }

    /// Whether the MMU is enabled — `SCTLR.M`.
    #[must_use]
    pub fn mmu_enabled(&self) -> bool {
        self.sctlr() & sctlr::M != 0
    }

    /// Tell the core's TLB that everything it cached may be wrong.
    fn invalidate(&self) {
        self.generation.fetch_add(1, Ordering::AcqRel);
    }

    /// The same, for this core and every core it was [`join`](Cp15v7::join)ed
    /// to: an inner-shareable TLB operation.
    fn invalidate_shareable(&self) {
        self.invalidate();
        for peer in self.peers.lock().iter() {
            peer.fetch_add(1, Ordering::AcqRel);
        }
    }

    /// `ISR`: the pending `I` and `F` inputs, bits 7 and 6 (DDI 0406C
    /// B4.1.90). Asynchronous aborts (`A`, bit 8) are never pending, because
    /// the core never raises one.
    fn isr(&self) -> u32 {
        let Some(lines) = &self.lines else {
            return 0;
        };
        let (irq, fiq) = lines.snapshot();
        (u32::from(irq) << 7) | (u32::from(fiq) << 6)
    }

    /// The `CCSIDR` `CSSELR` currently selects.
    fn ccsidr(&self) -> u32 {
        match self.get(reg::CSSELR) & 0xf {
            0 => id::CCSIDR_DATA,
            1 => id::CCSIDR_INSN,
            // No level-2 cache inside the core; the architecture calls the
            // value UNKNOWN, and zero is the deterministic one.
            _ => 0,
        }
    }

    /// Whether User mode may use the performance monitor right now.
    fn pmu_user(&self) -> bool {
        self.get(reg::PMUSERENR) & 1 != 0
    }
}

/// Where an access to one CP15 encoding lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Target {
    /// A plain stored register, with the bits a write may change.
    Stored(usize, u32),
    /// A read-only constant.
    Const(u32),
    /// Reads as zero, ignores writes.
    Raz,
    /// Something computed or with a side effect; handled by name.
    Special(Special),
}

/// The encodings that are more than a stored value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Special {
    Midr,
    Mpidr,
    Ccsidr,
    Isr,
    Cbar,
    Sctlr,
    Cpacr,
    /// A write that invalidates the TLB, into register `.0` masked by `.1`.
    TableBase(usize, u32),
    /// A c7 operation with no architectural effect here: cache, branch
    /// predictor, barrier. Write-only.
    Maintenance,
    /// One of the `ATS1C**` VA-to-PA operations. Write-only.
    Translate {
        write: bool,
        user: bool,
    },
    /// A c8 TLB operation. Write-only.
    Tlb {
        shareable: bool,
    },
    Pmcr,
    PmSet(usize),
    PmClear(usize),
    PmSwinc,
    PmEventType,
    PmEventCount,
}

impl Cp15v7 {
    /// Decode an encoding, and apply the User-mode rule.
    ///
    /// `None` is Undefined: an encoding the part does not have, or one this
    /// mode may not use in this direction (`write` is `MCR`).
    fn target(&self, op: CpOp, write: bool) -> Option<Target> {
        use Special as S;
        use Target::{Const, Raz, Stored};

        let t = match (op.opc1, op.crn, op.crm, op.opc2) {
            // ---- c0: identification (DDI 0406C B4.1.105, B3.18.1) ----
            (0, 0, 0, 0 | 4 | 7) => Target::Special(S::Midr),
            (0, 0, 0, 1) => Const(id::CTR),
            // No TCMs, one unified TLB with no lockable entries.
            (0, 0, 0, 2 | 3) => Const(0),
            (0, 0, 0, 5) => Target::Special(S::Mpidr),
            // REVIDR: no ECO fixes to report.
            (0, 0, 0, 6) => Const(0),
            (0, 0, 1, 0) => Const(id::ID_PFR0),
            (0, 0, 1, 1) => Const(id::ID_PFR1),
            (0, 0, 1, 2) => Const(id::ID_DFR0),
            (0, 0, 1, 3) => Const(id::ID_AFR0),
            (0, 0, 1, 4) => Const(id::ID_MMFR0),
            (0, 0, 1, 5) => Const(id::ID_MMFR1),
            (0, 0, 1, 6) => Const(id::ID_MMFR2),
            (0, 0, 1, 7) => Const(id::ID_MMFR3),
            (0, 0, 2, 0) => Const(id::ID_ISAR0),
            (0, 0, 2, 1) => Const(id::ID_ISAR1),
            (0, 0, 2, 2) => Const(id::ID_ISAR2),
            (0, 0, 2, 3) => Const(id::ID_ISAR3),
            (0, 0, 2, 4) => Const(id::ID_ISAR4),
            (0, 0, 2, 5) => Const(id::ID_ISAR5),
            // The rest of c0 c1..c7 is reserved for future feature ID
            // registers and is architecturally RAZ, so software probing a
            // newer field reads "not implemented" rather than trapping.
            (0, 0, 2, 6 | 7) | (0, 0, 3..=7, _) => Const(0),
            (1, 0, 0, 0) => Target::Special(S::Ccsidr),
            (1, 0, 0, 1) => Const(id::CLIDR),
            (1, 0, 0, 7) => Const(0), // AIDR
            (2, 0, 0, 0) => Stored(reg::CSSELR, 0xf),

            // ---- c1: system control ----
            (0, 1, 0, 0) => Target::Special(S::Sctlr),
            // ACTLR: FW, the two prefetch hints, write-full-line-of-zeros,
            // SMP, exclusive caching, alloc-in-one-way, parity
            // (DDI 0388 4.3.10). Stored only.
            (0, 1, 0, 1) => Stored(reg::ACTLR, 0x3cf),
            (0, 1, 0, 2) => Target::Special(S::Cpacr),
            (0, 1, 1, 0) => Stored(reg::SCR, 0x7f),
            (0, 1, 1, 1) => Stored(reg::SDER, 0x3),
            (0, 1, 1, 2) => Stored(reg::NSACR, 0x0007_3fff),
            // The A9's virtualization control register: nothing to control.
            (0, 1, 1, 3) => Raz,

            // ---- c2, c3: translation table control ----
            (0, 2, 0, 0) => Target::Special(S::TableBase(reg::TTBR0, 0xffff_ff7f)),
            (0, 2, 0, 1) => Target::Special(S::TableBase(reg::TTBR1, 0xffff_c07f)),
            // N, PD0, PD1. `EAE` (bit 31) is RAZ: no LPAE on an A9.
            (0, 2, 0, 2) => Target::Special(S::TableBase(reg::TTBCR, 0x37)),
            (0, 3, 0, 0) => Target::Special(S::TableBase(reg::DACR, u32::MAX)),

            // ---- c5, c6: faults ----
            (0, 5, 0, 0) => Stored(reg::DFSR, 0x1cff),
            (0, 5, 0, 1) => Stored(reg::IFSR, 0x140f),
            (0, 5, 1, 0) => Stored(reg::ADFSR, u32::MAX),
            (0, 5, 1, 1) => Stored(reg::AIFSR, u32::MAX),
            (0, 6, 0, 0) => Stored(reg::DFAR, u32::MAX),
            (0, 6, 0, 2) => Stored(reg::IFAR, u32::MAX),

            // ---- c7: cache maintenance, address translation, barriers ----
            (0, 7, 4, 0) => Stored(reg::PAR, u32::MAX),
            (0, 7, 8, opc2) => Target::Special(S::Translate {
                write: opc2 & 1 != 0,
                user: opc2 & 2 != 0,
            }),
            // `c7, c0, 4` was the ARMv6 CP15 wait-for-interrupt. ARMv7 moved
            // WFI into the instruction set and leaves this encoding a no-op
            // (DDI 0406C B4.2.3's list of retired c7 operations); software
            // for v7 executes `WFI`, which the core implements. Treating it as
            // a no-op costs a spinning idle loop at worst, where a halt the
            // part does not perform could stop a core that meant to continue.
            (0, 7, 0, 4)
            // ICIALLUIS, BPIALLIS
            | (0, 7, 1, 0 | 6)
            // ICIALLU, ICIMVAU, CP15ISB, BPIALL, BPIMVA
            | (0, 7, 5, 0 | 1 | 4 | 6 | 7)
            // DCIMVAC, DCISW
            | (0, 7, 6, 1 | 2)
            // DCCMVAC, DCCSW, CP15DSB, CP15DMB
            | (0, 7, 10, 1 | 2 | 4 | 5)
            // DCCMVAU
            | (0, 7, 11, 1)
            // DCCIMVAC, DCCISW
            | (0, 7, 14, 1 | 2) => Target::Special(S::Maintenance),

            // ---- c8: TLB maintenance ----
            (0, 8, 3, 0..=3) => Target::Special(S::Tlb { shareable: true }),
            (0, 8, 5..=7, 0..=3) => Target::Special(S::Tlb { shareable: false }),

            // ---- c9: performance monitor (DDI 0406C C12, DDI 0388 11) ----
            (0, 9, 12, 0) => Target::Special(S::Pmcr),
            (0, 9, 12, 1) => Target::Special(S::PmSet(reg::PMCNTEN)),
            (0, 9, 12, 2) => Target::Special(S::PmClear(reg::PMCNTEN)),
            (0, 9, 12, 3) => Target::Special(S::PmClear(reg::PMOVSR)),
            (0, 9, 12, 4) => Target::Special(S::PmSwinc),
            (0, 9, 12, 5) => Stored(reg::PMSELR, 0x1f),
            (0, 9, 13, 0) => Stored(reg::PMCCNTR, u32::MAX),
            (0, 9, 13, 1) => Target::Special(S::PmEventType),
            (0, 9, 13, 2) => Target::Special(S::PmEventCount),
            (0, 9, 14, 0) => Stored(reg::PMUSERENR, 1),
            (0, 9, 14, 1) => Target::Special(S::PmSet(reg::PMINTEN)),
            (0, 9, 14, 2) => Target::Special(S::PmClear(reg::PMINTEN)),

            // ---- c10: memory attributes, TLB lockdown ----
            (0, 10, 0, 0) => Stored(reg::TLB_LOCKDOWN, u32::MAX),
            (0, 10, 2, 0) => Stored(reg::PRRR, u32::MAX),
            (0, 10, 2, 1) => Stored(reg::NMRR, u32::MAX),

            // ---- c12: vectors, interrupt status ----
            (0, 12, 0, 0) => Stored(reg::VBAR, 0xffff_ffe0),
            (0, 12, 0, 1) => Stored(reg::MVBAR, 0xffff_ffe0),
            (0, 12, 1, 0) => Target::Special(S::Isr),
            // The A9's virtualization interrupt register.
            (0, 12, 1, 1) => Raz,

            // ---- c13: process and thread IDs ----
            // No FCSE on this part (ID_MMFR0 says so), which makes FCSEIDR
            // RAZ/WI rather than absent (DDI 0406C B4.1.78).
            (0, 13, 0, 0) => Raz,
            (0, 13, 0, 1) => Target::Special(S::TableBase(reg::CONTEXTIDR, u32::MAX)),
            (0, 13, 0, 2) => Stored(reg::TPIDRURW, u32::MAX),
            (0, 13, 0, 3) => Stored(reg::TPIDRURO, u32::MAX),
            (0, 13, 0, 4) => Stored(reg::TPIDRPRW, u32::MAX),

            // ---- c15: the A9's own (DDI 0388 4.3.x) ----
            (0, 15, 0, 0) => Stored(reg::POWER, 0x0000_ff01),
            // NEON busy: never, with no NEON.
            (0, 15, 1, 0) => Const(0),
            (4, 15, 0, 0) => Target::Special(S::Cbar),
            // Cache debug (opc1 3) and TLB lockdown access (opc1 5): windows
            // into structures this model does not have. RAZ/WI, because a
            // part-specific bring-up sequence may touch them and trapping it
            // would be refusing the part's own firmware.
            (3 | 5, 15, _, _) => Raz,

            _ => return None,
        };

        if op.privileged {
            return Some(t);
        }
        // User mode (DDI 0406C B3.17): the barriers, `TPIDRURW`, `TPIDRURO`
        // for reading, and the performance monitor when `PMUSERENR.EN` allows
        // (and `PMUSERENR` itself, for reading, always).
        let user = match (op.opc1, op.crn, op.crm, op.opc2) {
            (0, 7, 5, 4) | (0, 7, 10, 4 | 5) => true,
            (0, 13, 0, 2) => true,
            (0, 13, 0, 3) | (0, 9, 14, 0) => !write,
            (0, 9, 12..=13, _) => self.pmu_user(),
            _ => false,
        };
        user.then_some(t)
    }

    /// The `ATS1C**` operations: walk as the named privilege would, and
    /// leave the answer in `PAR` rather than taking an abort (DDI 0406C
    /// B4.2.6 and B4.1.112).
    fn address_translate(&self, mem: &dyn PhysMem, va: u32, write: bool, user: bool) {
        let kind = if write {
            AccessKind::Write
        } else {
            AccessKind::Read
        };
        let result = if self.mmu_enabled() {
            self.walk(mem, Va(va), Checks::Guest(kind, !user))
        } else {
            Ok(Leaf {
                pa: Pa(va),
                supersection: false,
                non_secure: false,
                shareable: false,
            })
        };
        let par = match result {
            // PA[31:12], then the attributes this model knows: `NS` (bit 9),
            // `SH` (bit 7) and `SS` (bit 1). The inner and outer cacheability
            // fields are left zero — there is no cache for them to describe.
            Ok(leaf) => {
                (leaf.pa.0 & 0xffff_f000)
                    | (u32::from(leaf.non_secure) << 9)
                    | (u32::from(leaf.shareable) << 7)
                    | (u32::from(leaf.supersection) << 1)
            }
            // `F` set, and `FS` in bits 6..1 as the DFSR's bits 12, 10 and
            // 3..0 would have them.
            Err(fault) => {
                let fsr = fault.to_fsr_v7();
                1 | ((fsr & 0xf) << 1) | (((fsr >> 10) & 1) << 5) | (((fsr >> 12) & 1) << 6)
            }
        };
        self.set(reg::PAR, par);
    }
}

impl Coprocessor for Cp15v7 {
    fn mrc(&self, op: CpOp) -> CpResult<u32> {
        if op.cp != 15 {
            return Err(CpFault::Undefined);
        }
        let target = self.target(op, false).ok_or(CpFault::Undefined)?;
        Ok(match target {
            Target::Stored(r, _) => self.get(r),
            Target::Const(value) => value,
            Target::Raz => 0,
            Target::Special(special) => match special {
                Special::Midr => id::MIDR,
                Special::Mpidr => self.mpidr,
                Special::Ccsidr => self.ccsidr(),
                Special::Isr => self.isr(),
                Special::Cbar => self.periphbase,
                Special::Sctlr => self.sctlr(),
                Special::Cpacr => self.cpacr(),
                Special::TableBase(r, _) => self.get(r),
                Special::Pmcr => id::PMCR | (self.get(reg::PMCR) & 0x39),
                Special::PmSet(r) | Special::PmClear(r) => self.get(r),
                Special::PmEventType | Special::PmEventCount => {
                    let base = if special == Special::PmEventType {
                        reg::PMXEVTYPER
                    } else {
                        reg::PMXEVCNTR
                    };
                    let sel = self.get(reg::PMSELR);
                    if sel < PMU_COUNTERS {
                        self.get(base + sel as usize)
                    } else {
                        0
                    }
                }
                // Write-only operations: reading one is Undefined.
                Special::Maintenance
                | Special::Translate { .. }
                | Special::Tlb { .. }
                | Special::PmSwinc => return Err(CpFault::Undefined),
            },
        })
    }

    /// `MCR` without a view of memory.
    ///
    /// The core never calls this — it calls [`mcr_with`](Coprocessor::mcr_with)
    /// — but a test or a SoC driving the coprocessor directly may. An address
    /// translation operation issued this way with the MMU on cannot read a
    /// table, and reports the external abort on the first-level walk that it
    /// would see on a bus with nothing behind it.
    fn mcr(&self, op: CpOp, value: u32) -> CpResult<CpEffect> {
        self.mcr_with(op, value, &NoMem)
    }

    fn mcr_with(&self, op: CpOp, value: u32, mem: &dyn PhysMem) -> CpResult<CpEffect> {
        if op.cp != 15 {
            return Err(CpFault::Undefined);
        }
        let target = self.target(op, true).ok_or(CpFault::Undefined)?;
        match target {
            Target::Stored(r, mask) => self.set(r, value & mask),
            // A write to a read-only register is Undefined on ARMv7.
            Target::Const(_) => return Err(CpFault::Undefined),
            Target::Raz => {}
            Target::Special(special) => match special {
                Special::Midr | Special::Mpidr | Special::Ccsidr | Special::Isr | Special::Cbar => {
                    return Err(CpFault::Undefined);
                }
                Special::Sctlr => {
                    let old = self.sctlr();
                    let kept = old & sctlr::NMFI;
                    self.set(
                        reg::SCTLR,
                        (value & sctlr::WRITABLE) | sctlr::READ_AS_ONE | kept,
                    );
                    // `M`, `A`, `AFE` and `V` all change what the core would
                    // decide; one bump covers every one of them.
                    self.invalidate();
                }
                Special::Cpacr => {
                    let cpacr = (value & self.cpacr_writable) | self.cpacr_one;
                    self.set(reg::CPACR, cpacr);
                }
                Special::TableBase(r, mask) => {
                    // TTBR0/1, TTBCR, DACR and CONTEXTIDR: every one changes
                    // what a cached translation would decide — the ASID in
                    // CONTEXTIDR included, because this model keeps no ASID
                    // tags and treats every `nG` mapping as belonging to
                    // whichever process is current. Flushing on the switch is
                    // what makes that correct.
                    self.set(r, value & mask);
                    self.invalidate();
                }
                Special::Maintenance => {}
                Special::Translate { write, user } => {
                    self.address_translate(mem, value, write, user);
                }
                // By ASID, by MVA, all: every form empties the whole TLB. A
                // TLB may lose an entry at any time, so over-invalidating is
                // architecturally free where under-invalidating is a stale
                // mapping.
                Special::Tlb { shareable } => {
                    if shareable {
                        self.invalidate_shareable();
                    } else {
                        self.invalidate();
                    }
                }
                Special::Pmcr => {
                    // E, D, X, DP are stored. P and C are "reset the
                    // counters" strobes and read as zero.
                    self.set(reg::PMCR, value & 0x39);
                    if value & (1 << 1) != 0 {
                        for i in 0..PMU_COUNTERS as usize {
                            self.set(reg::PMXEVCNTR + i, 0);
                        }
                    }
                    if value & (1 << 2) != 0 {
                        self.set(reg::PMCCNTR, 0);
                    }
                }
                Special::PmSet(r) => {
                    self.regs[r].fetch_or(value & PMU_MASK, Ordering::AcqRel);
                }
                Special::PmClear(r) => {
                    self.regs[r].fetch_and(!(value & PMU_MASK), Ordering::AcqRel);
                }
                // Nothing counts, so there is nothing to increment.
                Special::PmSwinc => {}
                Special::PmEventType | Special::PmEventCount => {
                    let (base, mask) = if special == Special::PmEventType {
                        (reg::PMXEVTYPER, 0xff)
                    } else {
                        (reg::PMXEVCNTR, u32::MAX)
                    };
                    let sel = self.get(reg::PMSELR);
                    if sel < PMU_COUNTERS {
                        self.set(base + sel as usize, value & mask);
                    }
                }
            },
        }
        Ok(CpEffect::NONE)
    }
}

impl Mmu for Cp15v7 {
    fn regime(&self) -> Regime {
        let sctlr = self.sctlr();
        Regime {
            generation: self.generation.load(Ordering::Acquire),
            translating: sctlr & sctlr::M != 0,
            high_vectors: sctlr & sctlr::V != 0,
            alignment_faults: sctlr & sctlr::A != 0,
            // ARMv7 always performs unaligned accesses (`SCTLR.U` is RAO).
            unaligned: true,
            vector_base: self.vbar(),
            thumb_exceptions: sctlr & sctlr::TE != 0,
            big_endian_exceptions: sctlr & sctlr::EE != 0,
            cp_access: self.cpacr(),
        }
    }

    fn translate(
        &self,
        mem: &dyn PhysMem,
        va: Va,
        kind: AccessKind,
        privileged: bool,
    ) -> core::result::Result<Pa, Fault> {
        self.walk(mem, va, Checks::Guest(kind, privileged))
            .map(|leaf| leaf.pa)
    }

    /// The same tables, with no access-flag, domain, permission or
    /// execute-never check. The Cortex-A9 has no hardware access-flag
    /// management, so no walk — debug or otherwise — ever writes a descriptor;
    /// and the fault registers are latched only by
    /// [`report_abort`](Mmu::report_abort), which the debug path never
    /// reaches.
    fn translate_debug(&self, mem: &dyn PhysMem, va: Va) -> core::result::Result<Pa, Fault> {
        self.walk(mem, va, Checks::None).map(|leaf| leaf.pa)
    }

    fn report_abort(&self, va: Va, fault: Fault, kind: AccessKind) {
        let fsr = fault.to_fsr_v7();
        if kind.is_fetch() {
            // ARMv7 has an instruction fault address register, and the IFSR
            // has no domain field (DDI 0406C B4.1.96, B4.1.97).
            self.set(reg::IFSR, fsr & !0xf0);
            self.set(reg::IFAR, va.0);
        } else {
            let wnr = u32::from(kind == AccessKind::Write) << 11;
            self.set(reg::DFSR, fsr | wnr);
            self.set(reg::DFAR, va.0);
        }
    }
}

impl Cp15v7 {
    /// The VMSAv7 short-descriptor walk (DDI 0406C B3.5), with the checks in
    /// the order B3.12.3 gives them: translation, access flag, domain, then
    /// permission and execute-never.
    ///
    /// The walk is read little-endian whatever `SCTLR.EE` says: the core's
    /// byte order is a construction property (`cp15.rs` makes the same call
    /// about the ARMv5 `B` bit), and a big-endian A9 guest is a board
    /// nobody has asked for.
    fn walk(&self, mem: &dyn PhysMem, va: Va, checks: Checks) -> core::result::Result<Leaf, Fault> {
        let va = va.0;
        let ttbcr = self.ttbcr();
        let n = ttbcr & 7;

        // TTBCR.N splits the address space (B3.5.4): with N > 0, an address
        // whose top N bits are all zero uses TTBR0, whose table then covers
        // only 2^(32-N) bytes and shrinks to 16 KiB >> N; everything else
        // uses TTBR1, whose table is always indexed by the full VA[31:20].
        let use_ttbr1 = n != 0 && (va >> (32 - n)) != 0;
        let first = if use_ttbr1 {
            if ttbcr & (1 << 5) != 0 {
                // PD1: a walk through TTBR1 is disabled, and a miss is a
                // translation fault.
                return Err(Fault::TRANSLATION_SECTION);
            }
            (self.ttbr1() & 0xffff_c000) | ((va >> 20) << 2)
        } else {
            if ttbcr & (1 << 4) != 0 {
                return Err(Fault::TRANSLATION_SECTION);
            }
            let base = self.ttbr0() & !((1u32 << (14 - n)) - 1);
            // VA[31-N:20].
            base | (((va << n) >> (n + 20)) << 2)
        };
        let descriptor = mem.read_u32(Pa(first)).ok_or(Fault::EXTERNAL_L1)?;

        match descriptor & 0b11 {
            // Nothing mapped. `0b11` is reserved on a part without LPAE's PXN
            // and faults the same way (B3.5.1).
            0b00 | 0b11 => Err(Fault::TRANSLATION_SECTION),
            // A page table: 256 entries, 1 KiB aligned, indexed by VA[19:12].
            0b01 => {
                let domain = ((descriptor >> 5) & 0xf) as u8;
                let non_secure = descriptor & (1 << 3) != 0;
                let second = (descriptor & 0xffff_fc00) | (((va >> 12) & 0xff) << 2);
                let entry = mem
                    .read_u32(Pa(second))
                    .ok_or_else(|| Fault::EXTERNAL_L2.in_domain(domain))?;
                // AP[2] is bit 9 and AP[1:0] bits 5..4 in both page formats.
                let ap = (((entry >> 9) & 1) << 2) | ((entry >> 4) & 0b11);
                let shareable = entry & (1 << 10) != 0;
                let (pa, xn) = match entry & 0b11 {
                    0b00 => return Err(Fault::TRANSLATION_PAGE.in_domain(domain)),
                    // Large page: 64 KiB, XN in bit 15.
                    0b01 => (
                        (entry & 0xffff_0000) | (va & 0x0000_ffff),
                        entry & (1 << 15) != 0,
                    ),
                    // Small page: 4 KiB, XN in bit 0.
                    _ => ((entry & 0xffff_f000) | (va & 0x0000_0fff), entry & 1 != 0),
                };
                self.check(checks, domain, ap, xn, Level::Page)?;
                Ok(Leaf {
                    pa: Pa(pa),
                    supersection: false,
                    non_secure,
                    shareable,
                })
            }
            // A section, or with bit 18 set a supersection.
            _ => {
                let supersection = descriptor & (1 << 18) != 0;
                let (pa, domain) = if supersection {
                    // 16 MiB, and always in domain 0: the supersection
                    // format spends bits 8..5 on extended address bits,
                    // which a 32-bit physical address space ignores.
                    ((descriptor & 0xff00_0000) | (va & 0x00ff_ffff), 0)
                } else {
                    (
                        (descriptor & 0xfff0_0000) | (va & 0x000f_ffff),
                        ((descriptor >> 5) & 0xf) as u8,
                    )
                };
                let ap = (((descriptor >> 15) & 1) << 2) | ((descriptor >> 10) & 0b11);
                let xn = descriptor & (1 << 4) != 0;
                self.check(checks, domain, ap, xn, Level::Section)?;
                Ok(Leaf {
                    pa: Pa(pa),
                    supersection,
                    non_secure: descriptor & (1 << 19) != 0,
                    shareable: descriptor & (1 << 16) != 0,
                })
            }
        }
    }

    /// The access flag, the domain, the permissions and execute-never, for a
    /// leaf the walk found.
    fn check(
        &self,
        checks: Checks,
        domain: u8,
        ap: u32,
        xn: bool,
        level: Level,
    ) -> core::result::Result<(), Fault> {
        let Checks::Guest(kind, privileged) = checks else {
            // A debugger asked where the page is, not whether it may touch
            // it.
            return Ok(());
        };
        let afe = self.sctlr() & sctlr::AFE != 0;
        // With AFE set, AP[0] is the access flag, and a clear one faults on
        // any access — whatever the domain says (B3.7.3).
        if afe && ap & 1 == 0 {
            return Err(level.access_flag().in_domain(domain));
        }
        match (self.dacr() >> (2 * u32::from(domain))) & 0b11 {
            // Manager: no permission check, and no execute-never either —
            // XN is one of the permission attributes a manager domain skips
            // (B3.7.2, B3.7.4).
            0b11 => Ok(()),
            0b01 => {
                let allowed = permits(ap, afe, kind, privileged);
                if allowed && !(xn && kind.is_fetch()) {
                    Ok(())
                } else {
                    Err(level.permission().in_domain(domain))
                }
            }
            // No access, and the reserved `0b10`, which behaves as no access
            // (B4.1.43 calls it UNPREDICTABLE; refusing is the reading that
            // cannot let a guest through a check it meant to fail).
            _ => Err(level.domain().in_domain(domain)),
        }
    }
}

/// The `AP[2:0]` table (DDI 0406C B3.7.1, tables B3-8 and B3-9).
fn permits(ap: u32, afe: bool, kind: AccessKind, privileged: bool) -> bool {
    use Allow::{None, Read, Write};
    let (pl1, pl0) = if afe {
        // The simplified model: AP[0] was the access flag, AP[2:1] decide.
        match (ap >> 1) & 0b11 {
            0b00 => (Write, None),
            0b01 => (Write, Write),
            0b10 => (Read, None),
            _ => (Read, Read),
        }
    } else {
        match ap & 0b111 {
            0b001 => (Write, None),
            0b010 => (Write, Read),
            0b011 => (Write, Write),
            0b101 => (Read, None),
            // 0b110 is the deprecated encoding of 0b111.
            0b110 | 0b111 => (Read, Read),
            // 0b000 is no access; 0b100 is reserved and treated the same.
            _ => (None, None),
        }
    };
    (if privileged { pl1 } else { pl0 }).permits(kind)
}

impl Cp15v7 {
    /// Write the architectural registers into a snapshot.
    ///
    /// The generation counter is not written, for the reason
    /// [`Cp15::save`](super::cp15::Cp15::save) gives; nor are `MPIDR` and
    /// `CBAR`, which are construction properties and come back with the
    /// machine file.
    ///
    /// # Errors
    ///
    /// If the sink refuses a write.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        for reg in &self.regs {
            w.write_u32(reg.load(Ordering::Acquire))?;
        }
        Ok(())
    }

    /// Restore what [`save`](Cp15v7::save) wrote.
    ///
    /// # Errors
    ///
    /// If the chunk is short.
    pub fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        for reg in &self.regs {
            reg.store(r.read_u32()?, Ordering::Release);
        }
        self.invalidate();
        Ok(())
    }
}

impl fmt::Display for Cp15v7 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (dfsr, dfar) = self.data_fault();
        let (ifsr, ifar) = self.instruction_fault();
        write!(
            f,
            "cp15 sctlr={:#010x} ttbr0={:#010x} ttbr1={:#010x} ttbcr={:#x} dacr={:#010x} \
             dfsr={dfsr:#x} dfar={dfar:#010x} ifsr={ifsr:#x} ifar={ifar:#010x} \
             contextidr={:#x}",
            self.sctlr(),
            self.ttbr0(),
            self.ttbr1(),
            self.ttbcr(),
            self.dacr(),
            self.contextidr(),
        )
    }
}

#[cfg(test)]
mod tests;
