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
//! | Advanced SIMD: arrangements, lanes, and the lanewise rules | [`simd`] |
//!
//! # The generic timer counts this core's own ticks
//!
//! `CNTPCT_EL0` is the core's domain tick counter — one tick per bus access —
//! divided by [`Config::cntdiv`], an integer the board supplies. That is
//! `ROADMAP.md` §4.2's exact intra-tree ratio: no residual, no absolute time,
//! and no host clock anywhere near it.
//!
//! It also **registers no scheduler event**, which is worth stating because
//! §4.2's rule ("a device never sleeps, never reads the wall clock, and never
//! spawns a thread to tick itself — it registers an event") reads like it
//! should. The rule exists so a device that must *act* at a future instant
//! gets dispatched then, and so nothing samples the host clock. The generic
//! timer is not a device beside the core; it is inside it, its only output is
//! a line only this core samples, and this core already looks at it once per
//! instruction and once per stalled `WFI`. An event would be a message the
//! core posted to itself and then read back one instruction late.
//!
//! The counter being the core's own tick count rather than a scheduler-published
//! one is also what keeps it deterministic: `CNTPCT_EL0` is a pure function of
//! instructions executed, so two runs of one machine with different quantum
//! sizes read the same counter and hash the same. A tick published by the
//! scheduler would not have that property.
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
//! # Advanced SIMD is the same register file
//!
//! The vector instructions live in [`simd`] and share everything with the
//! scalar ones: the register file, `FPCR`/`FPSR`, the `CPACR_EL1` trap, and
//! the arithmetic itself — a lanewise `FADD` is [`fp::add`] in a loop. What
//! `simd` adds is the *arrangement*, which is a vector operand's shape and the
//! source of most of the ways this family goes wrong.
//!
//! The slice implemented is the one that makes compiled floating-point code
//! runnable rather than a shallow pass over the whole encoding space, and
//! `simd`'s own documentation lists what is absent. The headline is that
//! `MOVI Dd, #0` — how LLVM materialises a floating-point zero — is an
//! Advanced SIMD encoding, so *scalar* floating-point code was not fully
//! runnable without it.
//!
//! # `FPSR.QC` means something
//!
//! The saturating and rounding Advanced SIMD arithmetic is here — `SQADD` and
//! its family, the halving and rounding-halving adds, the saturating shifts by
//! a register and by an immediate, the narrowing shifts, the extract-narrows,
//! the doubling multiplies, both vector and scalar — and it landed as one
//! piece because of the flag rather than in spite of it. `FPSR.QC` was
//! writable, readable and set by nothing at all, which is the same kind of
//! untruth as an `ID_AA64PFR0_EL1` reporting floating point without SIMD; it
//! is now set by every clamp and by nothing else, and the halving adds
//! deliberately leave it alone.
//!
//! # What is deliberately absent
//!
//! The reciprocal-estimate family, polynomial multiply, the pairwise long
//! adds, the halving-narrow three-different group (`ADDHN` and relatives), and
//! the by-element multiplies other than the four already here — including the
//! saturating by-element forms. `FEAT_FP16` arithmetic (half precision exists
//! here only as a conversion format, which is Armv8.0-A), EL2 and EL3 (so
//! `HVC` and `SMC` are `UNDEFINED`, and so `CNTVOFF_EL2` does not exist and
//! the virtual count equals the physical one), AArch32 at any level, the
//! pointer authentication, MTE, SVE,
//! big-endian data, and the `DC ZVA` block operation — `DCZID_EL0.DZP` says
//! so. Of the generic timer, the event stream (`CNTKCTL_EL1.EVNT*`) is storage
//! and `WFE` does not stall, so nothing drives it.
//!
//! # Accuracy
//!
//! `conformance` runs a suite this repository **builds** rather than fetches,
//! because no usable AArch64 corpus exists; its module documentation says what
//! that does and does not prove. The instruction table has also been diffed
//! against `llvm-mc` over a sample of the encoding space, which is what found
//! the missing `LDNP`/`STNP` rows, and — over the Advanced SIMD space — a
//! `FMUL` by element that decoded a bit it should have fixed, an `INS` that
//! decoded with `Q` clear, and an `LD1R` that decoded with `S` set. The
//! load/store-exclusive group and the whole `MRS`/`MSR` encoding space have
//! since been swept the same way — 396 608 words, nothing this core accepts
//! that `llvm-mc` rejects, and identical text on every exclusive pair and every
//! named system register. That sweep is what caught `CTR_EL0` and `DCZID_EL0`
//! becoming writable when `Access::El0Ro` was corrected to let EL1 write
//! `TPIDRRO_EL0`: `llvm-mc` has a writable name for one and not for the others.
//!
//! The **whole table** has since been swept in one piece, at the layer that
//! decides UNDEFINED rather than at the layer that matches fixed bits: every
//! row's fixed encoding, every one- and two-bit flip of the bits each row
//! leaves free, and every value of those bits where there are twelve or fewer
//! of them — **369 600 words**, each executed on a core and judged by whether
//! it raised `ESR_EL1.EC = 0`. Nothing this core accepts is rejected by
//! `llvm-mc`, and of the 276 698 words both accept, every mnemonic agrees
//! modulo aliasing (`llvm-mc` prints `mov` for `ORR Xd, XZR, Xm`, `cmp` for
//! `SUBS XZR`, `b.eq` where the suffix is this core's, and the `2` suffix on
//! an upper-half SIMD operation where this core puts the half in the operands).
//!
//! The 4 978 words `llvm-mc` decodes and this core does not are four groups,
//! all deliberate: `FCMP`/`FCMPE` against zero with a **non-zero `Rm`**, which
//! `exec` refuses as unallocated and `llvm-mc` ignores; IMPLEMENTATION DEFINED
//! system registers and `SYS`/`SYSL` operations this core does not implement;
//! `HVC`, because EL2 is not implemented; and `HLT`, because halting debug is
//! not.
//!
//! Two things about *how* that sweep is run are worth writing down, because
//! both produced large phantom results before they were found — 82 004
//! over-acceptances that were all the harness's. A core that has executed one
//! `MSR SCTLR_EL1` has its MMU on with no tables behind it, so every word
//! after it takes an instruction abort — `EC = 0x21`, not zero — and reads as
//! *accepted*; and a core that has executed one `WFI` retires nothing
//! afterwards and leaves `ESR_EL1` alone, which reads as accepted too. A
//! sweep of this kind wants a **fresh core per word**, and the answer it gets
//! should be spot-checked by re-asking about the words it flagged, one at a
//! time.
//!
//! The saturating group was swept the same way and **enumerated** rather than
//! sampled: every `Q`, `U`, `size`, `opcode` and `immh`:`immb` of the
//! three-same, two-misc, three-different and shift-by-immediate encodings,
//! vector and scalar, over four register triples — 141 312 words. Nothing this
//! core accepts is rejected, and every word both accept disassembles to
//! `llvm-mc`'s own text, with no exception outside two conventions that
//! predate the group (this core prints a modified immediate in hex, and spells
//! `MVN` as the `NOT` the encoding names). The sweep is also what named the
//! four scalar shifts `SSHL`/`USHL`/`SRSHL`/`URSHL`, which were missing.
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
pub mod psci;
pub mod simd;
pub mod sysreg;

// The IR frontend needs both this core and `src/ir`, so it has its own feature
// rather than riding on either (`ROADMAP.md` §9).
#[cfg(feature = "cpu-arm-a64-lift")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
pub mod differential;
#[cfg(feature = "cpu-arm-a64-lift")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
pub mod lift;

// The translated execution engine needs the frontend *and* the translation
// runtime, so it is gated on both rather than on either.
#[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
mod engine;

#[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
pub use engine::Stats as JitStats;

#[cfg(test)]
mod tests;

// The conformance corpus is built by a script into a directory named by an
// environment variable, and the runner reads it off the filesystem — so it is
// a `std` test even though the core it exercises is not.
#[cfg(all(test, feature = "std"))]
mod conformance;

#[cfg(all(test, feature = "std"))]
mod elf;

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
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};

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
    /// What `CNTFRQ_EL0` reports out of reset, in Hz.
    ///
    /// Architecturally UNKNOWN at reset and programmed by firmware, so this is
    /// the board declaring what its firmware would have written. Zero is the
    /// honest default for a core nobody told: a guest reading zero knows it
    /// was not told, which is better than a plausible number it would divide
    /// by.
    pub cntfrq: u64,
    /// How many of this core's ticks make one system-counter tick.
    ///
    /// # Why a divisor and not a frequency
    ///
    /// The counter has to advance in *virtual* time, and the only virtual
    /// time this core owns is its own domain's tick counter — one tick per
    /// bus access (`ROADMAP.md` §4.2: per-domain tick counters are the
    /// authoritative time state). Deriving the count from that is an exact
    /// integer division inside one oscillator tree, with no residual and no
    /// absolute time anywhere: it is the same relationship the NES PPU has to
    /// its CPU, and it is exact for the same reason.
    ///
    /// So the board owes two consistent numbers — `cntfrq × cntdiv` is the
    /// frequency of the domain the core is on — and `machines/a64-mini.machine`
    /// derives the second from the first rather than writing both, which is
    /// the only way to keep them from drifting apart.
    ///
    /// Never zero; [`Cpu::from_props`] refuses it and
    /// [`Config::with_counter`] saturates it.
    pub cntdiv: u64,
    /// This core's identity in `MemAttrs::requester`.
    pub requester: RequesterId,
    /// Which instruction, if either, this board answers PSCI calls on.
    ///
    /// A property of the *board* rather than of the part: `SMC` is
    /// architecturally UNDEFINED with no EL3 and `HVC` with no EL2, and this
    /// core implements neither level. Saying `smc` here is the board asserting
    /// that it has firmware behind that instruction; [`psci`] argues the case
    /// and says what the honest alternative would have cost.
    pub psci: psci::Conduit,
    /// How many processors the machine has, for `CPU_ON` and `AFFINITY_INFO`.
    ///
    /// A core cannot see its siblings, so this is the board telling it how
    /// many there are — which is the same fact `arm.boot` puts in the device
    /// tree, and the only thing that makes `CPU_ON` for processor 1 an honest
    /// answer rather than a guess.
    pub cpus: u64,
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
    /// ```text
    ///   [31:28] TGran4  = 0b0000  4 KiB supported, without FEAT_LPA2
    ///   [27:24] TGran64 = 0b1111  64 KiB not supported
    ///   [23:20] TGran16 = 0b0000  16 KiB not supported
    ///   [ 7: 4] ASIDBits = 0b0010 16-bit ASIDs
    ///   [ 3: 0] PARange  = 0b0101 48-bit physical addresses
    /// ```
    ///
    /// **The three granule fields use three different conventions**, which is
    /// a genuine trap in the architecture and the reason this constant is
    /// written out field by field rather than as a bare number: `TGran4 ==
    /// 0b0000` means *supported* and `0b1111` not, `TGran16 == 0b0000` means
    /// *not supported* and `0b0001` supported, and `TGran64` is like `TGran4`.
    /// So "4 KiB only" is `0`, `0b1111`, `0` — and the value that looks
    /// symmetrical is wrong in two fields at once.
    ///
    /// It was wrong here, in both of them, and the way it showed up is worth
    /// recording: `TGran4 == 0b0001` is not "4 KiB supported", it is *4 KiB
    /// supported **with FEAT_LPA2*** — 52-bit addressing and a different
    /// descriptor format that this core does not implement — and `TGran64 ==
    /// 0b0000` claimed a 64 KiB granule that [`mmu`] faults on.
    /// A guest reading the old value was told it could use two things that do
    /// not work.
    pub const ID_AA64MMFR0: u64 = 0x0000_0000_0f00_0025;

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
            cntfrq: 0,
            cntdiv: 1,
            requester: RequesterId::ANONYMOUS,
            psci: psci::Conduit::None,
            cpus: 1,
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
                advsimd: true,
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

    /// The same configuration with a generic-timer rate.
    ///
    /// `div` is clamped to at least one: a counter that advanced zero core
    /// ticks per count is a division by zero on the first `MRS`, and there is
    /// no value of it a caller could have meant.
    #[must_use]
    pub const fn with_counter(mut self, hz: u64, div: u64) -> Self {
        self.cntfrq = hz;
        self.cntdiv = if div == 0 { 1 } else { div };
        self
    }

    /// The system count at `cycles` core ticks.
    ///
    /// Floor division, so the count never runs ahead of the core and never
    /// goes backwards. Both matter: a guest that reads the counter twice and
    /// subtracts must not get a negative interval.
    #[inline]
    #[must_use]
    pub const fn counter_at(&self, cycles: u64) -> u64 {
        cycles / self.cntdiv
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
    /// EL3, and floating point and Advanced SIMD as [`Features`] says.
    ///
    /// # This register used to describe a part nobody makes
    ///
    /// DDI 0487 requires the `FP` and `AdvSIMD` fields to hold the same
    /// value: a part has both or neither. Until the vector instructions
    /// landed this core had scalar floating point without them, and it
    /// reported `FP == 0b0000` with `AdvSIMD == 0b1111` — an impossible
    /// combination, chosen deliberately over claiming a capability that would
    /// `UNDEF` on first use, because software checking `AdvSIMD` before a
    /// vector `memcpy` got the right answer that way.
    ///
    /// It no longer has to. The two fields are read from two flags that the
    /// named parts always set together, and
    /// `every_part_agrees_about_fp_and_advsimd` is what keeps them together —
    /// so a guest that assumes the architecture's own rule is now right.
    #[must_use]
    pub const fn id_aa64pfr0(&self) -> u64 {
        // EL0 = 0b0001, EL1 = 0b0001, both AArch64 only.
        let mut value = 0x0000_0000_0000_0011;
        // 0b1111 is "not implemented" in both fields, and 0b0000 is the
        // Armv8.0 baseline — no `FEAT_FP16`, which this core does not have.
        if !self.features.fp {
            value |= 0xf << 16;
        }
        if !self.features.advsimd {
            value |= 0xf << 20;
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
    /// A `PSCI_SYSTEM_OFF` or `SYSTEM_RESET` the guest asked for and the board
    /// has not been told about yet. See [`PowerRequest`].
    power: AtomicU32,
    /// Which of the generic timer's two outputs the board has taken *out* of
    /// the core, as [`Lines::TIMER_PHYS`] and [`Lines::TIMER_VIRT`].
    ///
    /// A board with no interrupt controller leaves both clear and the timer is
    /// wire-ORed onto the core's own `IRQ`, which is what
    /// `machines/a64-mini.machine` relies on. A board with a GIC wires the two
    /// out as private peripheral interrupts, and then the *only* route back in
    /// is through the GIC — a timer that also raised `IRQ` internally would
    /// give a kernel an interrupt its controller never saw, which it answers
    /// by reading `GICC_IAR`, being told 1023, and taking it again forever.
    timer_routed: AtomicU64,
    /// The level each of those two outputs is currently at, sampled at the end
    /// of every step so the wire can be driven with no lock held.
    timer_level: AtomicU64,
}

impl Lines {
    /// The `IRQ` input.
    pub const IRQ: u64 = 1 << 0;
    /// The `FIQ` input.
    pub const FIQ: u64 = 1 << 1;

    /// The EL1 physical timer's output, as
    /// [`route_timer`](Lines::route_timer) and
    /// [`timer_level`](Lines::timer_level) name it.
    pub const TIMER_PHYS: u64 = 1 << 0;
    /// The EL1 virtual timer's output.
    pub const TIMER_VIRT: u64 = 1 << 1;

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

    /// Record what a PSCI call asked the board to do.
    ///
    /// The **first** request wins, exactly as it does at the other end of the
    /// wire: a shutdown path that asks to power off and then resets must not
    /// come back up.
    pub fn request_power(&self, what: PowerRequest) {
        let _ = self
            .power
            .compare_exchange(0, what as u32, Ordering::Relaxed, Ordering::Relaxed);
    }

    /// Take a pending power request, clearing it.
    pub fn take_power_request(&self) -> Option<PowerRequest> {
        match self.power.swap(0, Ordering::Relaxed) {
            1 => Some(PowerRequest::Poweroff),
            2 => Some(PowerRequest::Reboot),
            _ => None,
        }
    }

    /// Take one of the generic timer's outputs out of the core and onto a
    /// wire. Called from `Device::connect`, once, before the core runs.
    pub fn route_timer(&self, which: u64) {
        self.timer_routed.fetch_or(which, Ordering::Relaxed);
    }

    /// Which timer outputs the board took out of the core.
    #[must_use]
    pub fn routed_timers(&self) -> u64 {
        self.timer_routed.load(Ordering::Relaxed)
    }

    /// Record what the timer outputs are doing now.
    pub fn set_timer_level(&self, levels: u64) {
        self.timer_level.store(levels, Ordering::Relaxed);
    }

    /// What the timer outputs were doing at the end of the last step.
    #[must_use]
    pub fn timer_level(&self) -> u64 {
        self.timer_level.load(Ordering::Relaxed)
    }
}

/// What a PSCI call asked the board to do, as it crosses the execution lock.
///
/// A plain `u32` in the atomic, because `core::sync` has no atomic enum and a
/// power request is exactly two values plus "nothing".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PowerRequest {
    /// `PSCI_SYSTEM_OFF`.
    Poweroff = 1,
    /// `PSCI_SYSTEM_RESET`.
    Reboot = 2,
}

/// The values a machine file's `engine` property takes.
///
/// Named in every build, whatever the features, so that a description
/// validates identically everywhere and a build that cannot run one refuses it
/// by name rather than by "expected one of `interp`".
const ENGINES: &[&str] = &["interp", "jit", "jit-host"];

/// Which execution engine a core runs on.
///
/// Not in [`Config`] and not in the snapshot: a snapshot taken under one
/// engine must restore under the other, and there is nothing engine-specific
/// in one to interchange (`ROADMAP.md` §4.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum Engine {
    /// The interpreter: one instruction at a time, and the oracle everything
    /// else is measured against (CLAUDE.md, "CPU cores").
    #[default]
    Interp,
    /// The translation runtime, executing lifted blocks on the portable IR
    /// interpreter: `ROADMAP.md` §9's software TLB, block cache, block
    /// chaining and self-modifying-code detection, with no host code
    /// generation.
    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
    Jit,
    /// The same, with the host code generator attached where the build and the
    /// host have one. A block the generator refuses runs on the portable
    /// backend, so this is a speed knob and never a semantic one.
    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
    JitHost,
}

#[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
impl Engine {
    /// Whether this engine runs blocks through the translation runtime.
    const fn translates(self) -> bool {
        matches!(self, Engine::Jit | Engine::JitHost)
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
    /// The translation state, built on first use because §4.4 says `new`
    /// performs no outward action — and a 256 MiB code buffer is an outward
    /// action.
    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
    jit: Option<Box<engine::Jit>>,
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
    /// Which execution engine this core runs on. Construction property, not
    /// architectural state.
    engine: Engine,
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

/// The sinks this core has published, one per input pin, and the nets its
/// four output pins drive.
///
/// A core with *outputs* is new here and is worth a sentence. Two of them are
/// the generic timer's, which on a board with an interrupt controller is a
/// private peripheral interrupt rather than something internal (see
/// [`Lines::route_timer`]); the other two carry a PSCI request out to whatever
/// the board does about it. All four are driven from
/// [`Cpu::step_to_exit`] **after** the execution lock is released, which is
/// the re-entrancy contract and is what stops a `SYSTEM_OFF` re-entering the
/// core through the device it just poked.
#[derive(Debug, Default)]
struct Pins {
    interrupts: Vec<(u64, Arc<InterruptPin>)>,
    reset: Option<Arc<ResetPin>>,
    /// The EL1 physical timer's interrupt output, `cntp`.
    cntp: Option<WireSource>,
    /// The EL1 virtual timer's interrupt output, `cntv`.
    cntv: Option<WireSource>,
    /// Pulsed when a guest calls `PSCI_SYSTEM_OFF`.
    poweroff: Option<WireSource>,
    /// Pulsed when a guest calls `PSCI_SYSTEM_RESET`.
    reboot: Option<WireSource>,
}

impl Cpu {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Cpu::attach_space) and [`Device::realize`].
    #[must_use]
    pub fn new(cfg: Config) -> Cpu {
        Cpu {
            engine: Engine::Interp,
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    tlb: Tlb::new(),
                    space: None,
                    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
                    jit: None,
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
        let cntfrq = r.or("cntfrq", 0u64)?;
        let cntdiv = r.or("cntdiv", 1u64)?;
        // Which instruction, if either, this board answers PSCI calls on. The
        // default is `none`, which is the architectural answer for a core with
        // neither EL2 nor EL3: a board that wants a firmware interface asks
        // for it, and `a64-mini` never has.
        let conduit = r.or_enum("psci", "none", psci::Conduit::NAMES)?;
        let cpus = r.or_range("cpus", 1u64, 1..=256)?;
        // Every value is named in every build, so a machine file validates the
        // same everywhere and a build that cannot run one says *why* rather
        // than "expected one of `interp`".
        let want = r.or_enum("engine", "interp", ENGINES)?;
        let want_jit = want != "interp";
        r.finish()?;

        #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
        let engine = match want {
            "jit-host" => Engine::JitHost,
            "jit" => Engine::Jit,
            _ => Engine::Interp,
        };
        #[cfg(not(all(feature = "cpu-arm-a64-lift", feature = "jit")))]
        let engine = Engine::Interp;
        #[cfg(not(all(feature = "cpu-arm-a64-lift", feature = "jit")))]
        if want_jit {
            return Err(Error::Property(alloc::string::String::from(
                "`engine = \"jit\"` needs a build with the `cpu-arm-a64-lift` and `jit` \
                 features; this one has only the interpreter. Refused rather than \
                 interpreted silently, because an engine that is not the one you asked \
                 for is a measurement that quietly means nothing",
            )));
        }
        let _ = want_jit;

        // `CNTFRQ_EL0` is a 32-bit field, so a board naming a wider frequency
        // is naming one the guest could never write back — and it would read
        // one value and write another, which is the sort of asymmetry a driver
        // uses to decide the register is broken.
        if cntfrq > u64::from(u32::MAX) {
            return Err(Error::Property(alloc::format!(
                "`cntfrq` is CNTFRQ_EL0, a 32-bit field, and {cntfrq} does not fit in one"
            )));
        }
        // Refused rather than clamped: a board that wrote `cntdiv = 0` meant
        // something, and silently reading it as 1 would give the guest a
        // counter running a hundred times too fast with nothing to say so.
        if cntdiv == 0 {
            return Err(Error::Property(alloc::string::String::from(
                "`cntdiv` is how many core ticks make one system-counter tick                  and cannot be zero",
            )));
        }
        let cfg = Config::by_name(part).ok_or_else(|| {
            Error::Property(alloc::format!("`cpu` names an unknown part `{part}`"))
        })?;
        let psci = psci::Conduit::by_name(conduit)
            .ok_or_else(|| Error::Property(alloc::format!("`psci` names `{conduit}`")))?;
        Ok(Cpu::new(Config {
            reset_vector,
            mpidr,
            cntfrq,
            cntdiv,
            psci,
            cpus,
            ..cfg
        })
        .with_engine(engine))
    }

    /// The same core, running on `engine`.
    ///
    /// A consuming builder because an engine is chosen when a core is built,
    /// like every other construction property; a machine file says
    /// `engine = "jit"` and reaches the same place through
    /// [`from_props`](Cpu::from_props).
    #[must_use]
    pub fn with_engine(mut self, engine: Engine) -> Cpu {
        self.engine = engine;
        self
    }

    /// Which execution engine this core runs on.
    #[must_use]
    pub fn engine(&self) -> Engine {
        self.engine
    }

    /// What this core's translated engine has done, or `None` on a core
    /// running the interpreter.
    ///
    /// A statistic and never a behaviour — the engines are indistinguishable
    /// to the guest — but a backend whose coverage is unmeasured is a backend
    /// whose coverage rots, which is why it is counted rather than assumed.
    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-a64-lift")))]
    #[must_use]
    pub fn jit_stats(&self) -> Option<JitStats> {
        self.session.lock().jit.as_ref().map(|jit| jit.stats())
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

    /// The generic timer's system count now — what `CNTPCT_EL0` would read.
    #[must_use]
    pub fn counter(&self) -> u64 {
        let cfg = self.effective_config();
        cfg.counter_at(self.session.lock().state.cycles)
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
        let (used, exit) = {
            let Session {
                state,
                tlb,
                space,
                #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
                    jit: _jit,
            } = &mut *session;
            let Some(space) = space.clone() else {
                return (0, None);
            };
            let mut exec = Exec::new(state, tlb, &space, &cfg, &self.lines, exits);
            let used = exec.step();
            // What one interpreted instruction wrote, handed to the block
            // cache. A core whose blocks were invalidated only by `advance`
            // would serve a stale one to anything that mixes this entry point
            // with the run loop — a monitor stepping, a test driving the core
            // by hand.
            #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
            if let Some(jit) = _jit.as_mut() {
                jit.note_writes(&mut exec);
            }
            (used, exec.take_exit())
        };
        // Everything outward happens **here**, with the execution lock
        // released. A wire callback reaches another device, that device may
        // reach back through the bus, and a core that drove a line while
        // holding its own `BUS`-ranked lock would be the deadlock the ranked
        // order exists to prevent (`CLAUDE.md`, the re-entrancy contract).
        drop(session);
        self.drive_outputs();
        (used, exit)
    }

    /// Drive the four output pins from what the last step left behind.
    ///
    /// Never called with the session lock held. The timer levels were sampled
    /// inside the step and stashed in [`Lines`]; the power request was put
    /// there by a PSCI call.
    fn drive_outputs(&self) {
        let routed = self.lines.routed_timers();
        let power = self.lines.take_power_request();
        if routed == 0 && power.is_none() {
            // The common case, and the whole of what an `a64-mini`-shaped
            // board with no interrupt controller ever does here.
            return;
        }
        let levels = self.lines.timer_level();
        let pins = self.pins.lock();
        let (cntp, cntv, poweroff, reboot) = (
            pins.cntp.clone(),
            pins.cntv.clone(),
            pins.poweroff.clone(),
            pins.reboot.clone(),
        );
        drop(pins);
        if let Some(out) = cntp.filter(|_| routed & Lines::TIMER_PHYS != 0) {
            out.set(Level::from_bool(levels & Lines::TIMER_PHYS != 0));
        }
        if let Some(out) = cntv.filter(|_| routed & Lines::TIMER_VIRT != 0) {
            out.set(Level::from_bool(levels & Lines::TIMER_VIRT != 0));
        }
        // A pulse rather than a level: a request is an event, and the board's
        // end latches it.
        let pulse = match power {
            Some(PowerRequest::Poweroff) => poweroff,
            Some(PowerRequest::Reboot) => reboot,
            None => None,
        };
        if let Some(out) = pulse {
            out.set(Level::High);
            out.set(Level::Low);
        }
    }

    /// One step of the run loop, on whichever engine this core runs.
    ///
    /// `remaining` is what is left of the caller's budget, and it is binding
    /// rather than advisory: a block whose worst case does not fit is not run
    /// and the instruction is interpreted instead, so that the core stops on
    /// the same instruction whichever engine drives it and carries the same
    /// `State::debt` into the next quantum. Both numbers are in the snapshot a
    /// machine's state hash is taken over.
    #[allow(unused_variables)]
    fn advance(&self, remaining: u64) -> (u64, Option<Exit>) {
        #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
        if self.engine.translates() {
            let cfg = self.effective_config();
            let exits = self.exit_mask();
            let mut session = self.session.lock();
            if self.lines.take_reset_request() {
                session.state = State::new(&cfg);
                session.tlb.flush();
                if let Some(jit) = session.jit.as_mut() {
                    jit.flush();
                }
            }
            if session.jit.is_none() {
                session.jit = Some(Box::new(engine::Jit::new(self.engine == Engine::JitHost)));
            }
            // The compiled fast path's shadow, attached to this core's *own*
            // TLB so the two evict together (`mmu::Tlb::attach_shadow`). It is
            // asked for here rather than at construction because it needs the
            // address space, which arrives with `attach_space`, and only the
            // engine that reads it asks for it at all.
            if !session.tlb.has_shadow()
                && session.jit.as_ref().is_some_and(|jit| jit.wants_shadow())
                && let Some(space) = session.space.clone()
            {
                session.tlb.attach_shadow(space);
            }
            let Session {
                state,
                tlb,
                space,
                jit,
            } = &mut *session;
            let Some(space) = space.clone() else {
                return (0, None);
            };
            let jit = jit.as_mut().expect("just installed");
            let out = engine::advance(jit, state, tlb, &space, &cfg, &self.lines, exits, remaining);
            // Everything outward happens **here**, with the execution lock
            // released — the same rule `step_to_exit` follows and for the same
            // reason (CLAUDE.md, the re-entrancy contract).
            drop(session);
            self.drive_outputs();
            return out;
        }
        self.step_to_exit()
    }

    /// Execute until at least `budget` accesses have been charged.
    pub fn run(&self, budget: u64) -> u64 {
        let mut used = 0;
        while used < budget {
            let n = self.advance(budget - used).0;
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
            let n = self.advance(allowance - used).0;
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
            let (n, exit) = self.advance(allowance - used);
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
///
/// Version 2: the snapshot chunk grew by the six generic-timer registers.
/// Bumped rather than migrated, because a version-1 snapshot has no timer
/// state to migrate *from* and restoring one into a core whose comparators
/// then read as zero would be a machine that fires an interrupt it never armed.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.arm.a64",
    version: 2,
    summary: "AArch64 A64 integer core with EL0/EL1, the VMSAv8-64 MMU, \
              the generic timer and a disassembler",
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
            name: "cntfrq",
            kind: ValueKind::Uint,
            required: false,
            summary: "what CNTFRQ_EL0 reports out of reset, in Hz (default 0: nobody said)",
        },
        PropertySpec {
            name: "cntdiv",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many core ticks make one system-counter tick; \
                      `cntfrq * cntdiv` is the core's clock rate (default 1)",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine: `interp`, `jit` (the translation runtime), \
                      or `jit-host` (the same, with the host code generator). Both JIT \
                      values need `cpu-arm-a64-lift` and `jit`",
        },
        PropertySpec {
            name: "psci",
            kind: ValueKind::Str,
            required: false,
            summary: "which instruction the board answers PSCI calls on: \
                      `smc`, `hvc`, or `none` (default none)",
        },
        PropertySpec {
            name: "cpus",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many processors the machine has, for PSCI CPU_ON (default 1)",
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

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let mut pins = self.pins.lock();
        match port {
            // Taking a timer's output onto a wire also takes it *out* of the
            // core: the board has an interrupt controller now, and the only
            // route back in is through it.
            "cntp" => {
                pins.cntp = Some(source);
                self.lines.route_timer(Lines::TIMER_PHYS);
            }
            "cntv" => {
                pins.cntv = Some(source);
                self.lines.route_timer(Lines::TIMER_VIRT);
            }
            "poweroff" => pins.poweroff = Some(source),
            "reboot" => pins.reboot = Some(source),
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: alloc::format!(
                        "an AArch64 core drives `cntp` and `cntv` (the generic timer's two \
                         private peripheral interrupts) and `poweroff` and `reboot` (what a \
                         PSCI call asks the board for); `{port}` is none of them"
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        // A freshly connected timer line has to start at the level the timer
        // is actually at, or a comparator that was already expired would never
        // be noticed.
        if matches!(port, "cntp" | "cntv") {
            let cfg = self.effective_config();
            let levels = {
                let session = self.session.lock();
                let count = cfg.counter_at(session.state.cycles);
                session.state.sys.timer_levels(count)
            };
            self.lines.set_timer_level(levels);
            self.drive_outputs();
        }
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
        // Both caches, and for the same reason: a reset can be accompanied by
        // a reload of the memory a translation was lifted from, and a block
        // cache is derived state in exactly `ROADMAP.md` §4.5's sense.
        #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
        if let Some(jit) = session.jit.as_mut() {
            jit.flush();
        }
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
        // **And so is the block cache**, which is the harder half to remember
        // because a snapshot's own chunk says nothing about it. A restore
        // replaces the RAM a block was lifted from, so every translation in
        // the cache describes bytes that are no longer there — and nothing
        // else would notice, because `jit::dispatch`'s invalidation watches
        // *guest stores* and a restore is not one. Measured: a debugger that
        // patched one instruction into a hot loop, then round-tripped the
        // machine through `save`/`load` to publish it, still ran the stale
        // block 107 times out of 66 667 iterations. `cpu::riscv` and
        // `cpu::x86` have always flushed here; this core did not.
        #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
        if let Some(jit) = session.jit.as_mut() {
            jit.flush();
        }
        drop(session);
        self.lines.set_all(pending);
        Ok(())
    }
}

impl Cpu {
    /// How many 64-bit system-register words a snapshot carries.
    const SYSREG_WORDS: usize = 30;

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
            // The generic timer. `CNTFRQ_EL0` is here rather than derived from
            // the configuration because a guest may have written it, and a
            // restore that quietly put the board's value back would undo a
            // write the guest can read.
            s.cntfrq,
            s.cntkctl,
            s.cntp_ctl,
            s.cntp_cval,
            s.cntv_ctl,
            s.cntv_cval,
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
            &mut s.cntfrq,
            &mut s.cntkctl,
            &mut s.cntp_ctl,
            &mut s.cntp_cval,
            &mut s.cntv_ctl,
            &mut s.cntv_cval,
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
        .prop(PropSchema::new("cntfrq", ValueKind::Uint))
        .prop(PropSchema::new("cntdiv", ValueKind::Uint))
        .prop(PropSchema::new("engine", ValueKind::Str).values(ENGINES))
        .prop(PropSchema::new("psci", ValueKind::Str).values(psci::Conduit::NAMES))
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, 256))
        .port("irq", PortDir::In)
        .port("fiq", PortDir::In)
        .port("reset", PortDir::In)
        // The generic timer's two private peripheral interrupts, for a board
        // with something to route them to, and what a PSCI call asks the board
        // for. See `Pins`.
        .port("cntp", PortDir::Out)
        .port("cntv", PortDir::Out)
        .port("poweroff", PortDir::Out)
        .port("reboot", PortDir::Out)
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
