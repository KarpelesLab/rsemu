//! The ARMv7E-M core — a Cortex-M4/M7-class interpreter: Thumb-2, the DSP
//! extensions, and the M-profile exception model with its NVIC.
//!
//! There is no ARM state in this architecture at all, so there is no A32
//! decoder here and a Cortex-M build links none. What there *is*: the whole
//! T32 instruction set in both encoding widths, `IT` blocks, the DSP (E)
//! extension including the SIMD add/sub family and the dual multiplies,
//! Handler and Thread modes with `MSP` and `PSP`, automatic register stacking
//! with `EXC_RETURN`, tail-chaining, the fault taxonomy with its status
//! registers, and the memory-mapped NVIC, SysTick, SCB and MPU at
//! `0xE000E000`.
//!
//! The **FPv4-SP / FPv5-SP floating-point unit** is here too, behind the
//! `cpu-arm-v7m-fp` feature and selected per instance by
//! [`Config::CORTEX_M4F`] or [`Config::CORTEX_M7F`]: `S0`–`S31`, `FPSCR`, the
//! single-precision instruction set, `CPACR` gating, `CONTROL.FPCA`, the
//! 26-word extended exception frame and **lazy state preservation**. A part
//! configured without it — every `Config` whose `ext.fp` is [`FpUnit::None`],
//! which includes [`Config::CORTEX_M4`] — reads `CPACR` as zero and raises a
//! `UFSR.NOCP` UsageFault on a `VMOV`, exactly as a Cortex-M4 without the
//! option does. Double precision is not implemented at either revision; see
//! "Unimplemented" below and [`fp`]'s own documentation.
//!
//! **Armv8-M Mainline** — the Cortex-M33 — is here as well, behind the
//! `cpu-arm-v8m` feature and selected by [`Config::CORTEX_M33`] /
//! `CORTEX_M33F` (`part = "cortex-m33"` / `"cortex-m33f"`): the
//! `MSPLIM`/`PSPLIM` stack limits with the `STKOF` UsageFault, the v8-M
//! `EXC_RETURN` format, `LDA`/`STL`/`LDAEX`/`STLEX`, `TT`, the PMSAv8 MPU and
//! the v8-M `CCR`/`FPCCR` bits — modelled **without the Security Extension**,
//! which is what an STM32U5 with `TZEN = 0` presents. `v8m`'s own
//! documentation says exactly what that includes and leaves out. Everything
//! v8-M changes keys off [`Config::arch`]; a v7-M part behaves identically
//! whether or not the feature is compiled in.
//!
//! **ARMv6-M** — the Cortex-M0, Cortex-M0+ and SecurCore SC000 — is the same
//! core cut down, behind the `cpu-arm-v6m` feature and selected by
//! `Config::CORTEX_M0`, `CORTEX_M0PLUS` or `SC000` (`part = "cortex-m0"`,
//! `"cortex-m0plus"` / `"cortex-m0+"`, `"sc000"`). Decode is gated per
//! encoding to the v6-M set — the sixteen-bit instructions bar `CBZ`/`CBNZ`
//! and `IT`, and of the thirty-two-bit ones only `BL`, `MSR`, `MRS`, `DMB`,
//! `DSB` and `ISB` — so everything else is UNDEFINED, and the exception model
//! is v6-M's: HardFault for every fault, no fault status registers, two
//! priority bits, `PRIMASK` the only mask, a read-only `CCR` that makes every
//! unaligned access fault, 32 interrupts, an optional `VTOR`, PMSAv6 where
//! the part has an MPU, and no bit-band. `v6m`'s documentation has the whole
//! list. It too keys off [`Config::arch`], and a v7-M part is unchanged by it.
//!
//! The **SecurCore SC300** is a Cortex-M3 by another name as far as this
//! core can tell: [`Config::SC300`] (`part = "sc300"`) is the Cortex-M3's
//! options with its own `CPUID` and CMSIS `core_sc300.h`'s `VTOR` width, and
//! needs no feature beyond the core's own.
//!
//! The reset sequence reads `SP` and `PC` from [`Config::reset_vtor`] — the
//! `vtor` property — which every reset restores. Zero, as DDI 0403 has it,
//! unless the SoC boots elsewhere; an STM32U5 boots from `0x08000000` with
//! nothing mapped at zero.
//!
//! # Using it from another crate
//!
//! Built to be consumed directly, without a `.machine` file — a downstream SoC
//! crate is the likely consumer.
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::arm::v7m::{ArmV7m, Config};
//!
//! // 64 KiB of RAM: a vector table, then `MOVS r0, #0x42` at the entry.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! ram.write_at(0x0, &0x0000_1000u32.to_le_bytes()).unwrap();  // initial SP
//! ram.write_at(0x4, &0x0000_0101u32.to_le_bytes()).unwrap();  // reset vector
//! ram.write_at(0x100, &0x2042u16.to_le_bytes()).unwrap();     // MOVS r0, #0x42
//!
//! let space = AddressSpace::new("cpu", 32);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = ArmV7m::new(Config::CORTEX_M4);
//! cpu.attach_space(Arc::new(space));
//! cpu.step();                       // the reset sequence
//! assert_eq!(cpu.pc(), 0x100);
//! cpu.step();                       // MOVS r0, #0x42
//! assert_eq!(cpu.reg(0), 0x42);
//! ```
//!
//! The rest of that surface: [`ArmV7m::run`] for a cycle budget,
//! [`ArmV7m::regs`] / [`ArmV7m::set_regs`] for the whole file,
//! [`ArmV7m::set_irq`] and [`ArmV7m::pend_irq`] for the interrupt inputs,
//! [`ArmV7m::with_sys`] to reach the NVIC and SCB from a debugger, and
//! [`ArmV7m::disassemble`] for a listing.
//!
//! # The device path
//!
//! [`ArmV7m`] is also a full [`Device`]: it has a [`CLASS`], it builds from
//! [`Props`], it takes scheduler budgets through [`Device::run`], and it
//! round-trips through [`Device::save`] and [`Device::load`].
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the T32 decoder — both widths — producing one value the interpreter and the disassembler share |
//! | [`sys`] | exception numbers, priorities, and the register map at `0xE000E000` |
//! | `fp` | the floating-point register file, `FPSCR`, and the Arm rules over [`crate::float`] |
//! | `fpisa` | the floating-point encodings, and their disassembly |
//! | `v8m` | Armv8-M Mainline: PMSAv8, the v8-M register map, `EXC_RETURN`, `TT` |
//! | `v6m` | ARMv6-M: the decode gate, the v6-M register map, PMSAv6's differences |
//! | `dsp` (private) | the SIMD and extending-move semantics |
//! | `exec` (private) | the interpreter and its timing model |
//!
//! # Unimplemented, stated plainly
//!
//! - **Double-precision floating point.** The two units modelled are
//!   FPv4-SP-D16 and FPv5-SP-D16, which are real part configurations;
//!   `MVFR0.D_Precision` reads zero and every coprocessor 11 *arithmetic*
//!   encoding is UNDEFINED. The doubleword data movement — `VLDR`/`VSTR.64`,
//!   D-register `VLDM`/`VSTM`/`VPUSH`/`VPOP`, `VMOV Dm, Rt, Rt2` — is present,
//!   as it is on those parts. An FPv5-D16 Cortex-M7 with double precision is
//!   not modelled.
//!   `FPSCR.AHP` — the alternative half-precision format — is RES0 for the
//!   same reason: it reads back as zero so a guest can tell.
//!   `tests/conformance/ledgers/cpu-arm-v7m-fp.txt` is the full list.
//! - **The debug architecture**, bar the one piece of it firmware runs on its
//!   own. No halting debug, no ITM, no FPB, no TPIU, and no DWT comparators,
//!   watchpoints or profile counters; their registers read as zero and
//!   `DWT_CTRL.NOPRFCNT` says so. What *is* there is `DEMCR.TRCENA` and
//!   `DWT_CYCCNT`, because a vendor HAL's microsecond delay spins on the
//!   cycle counter and a counter pinned at zero is a silent hang rather than
//!   a missing feature. `BKPT` is a HardFault with `HFSR.DEBUGEVT`, which is
//!   what a part with no debugger attached does.
//! - **Imprecise bus faults.** There is no write buffer, so every data abort
//!   is precise and `BFSR.IMPRECISERR` is never set.
//! - **Cache and TCM behaviour** on the M7. The core is not the place for it;
//!   a SoC's memory system is.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403; the *Armv8-M
//! Architecture Reference Manual*, Arm DDI 0553, for the Cortex-M33; the
//! *ARMv6-M Architecture Reference Manual*, ARM DDI 0419E, for the Cortex-M0
//! family; the *Cortex-M0*, *Cortex-M0+*, *Cortex-M4*, *Cortex-M7* and
//! *Cortex-M33 Technical Reference Manuals* for the implementation-defined
//! values (`CPUID`, the number of priority bits, MPU region count) and the
//! instruction timings. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

pub mod isa;
pub mod sys;

#[cfg(feature = "cpu-arm-v7m-fp")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
pub mod fp;

#[cfg(feature = "cpu-arm-v7m-fp")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
pub mod fpisa;

mod dsp;
mod exec;

#[cfg(feature = "cpu-arm-v8m")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
pub mod v8m;

#[cfg(feature = "cpu-arm-v6m")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
pub mod v6m;

#[cfg(all(test, feature = "cpu-arm-v7m-fp"))]
mod fptests;

#[cfg(test)]
mod tests;

#[cfg(all(test, feature = "cpu-arm-v8m"))]
mod v8mtests;

#[cfg(all(test, feature = "cpu-arm-v6m"))]
mod v6mtests;

// The differential harness runs this core and the A-profile one over the same
// instruction stream, so it only exists where both are compiled.
#[cfg(all(test, feature = "cpu-arm-aprofile"))]
mod differential;

// The built-ELF conformance runner shells out to `clang` and reads the
// filesystem, so it exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;
#[cfg(all(test, feature = "std"))]
mod corpus;
#[cfg(all(test, feature = "std"))]
mod elf;

use alloc::boxed::Box;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed, TickCursor};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::spin;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::validate::port_index;

use exec::{Exec, State};
use sys::{Exception, Sys};

pub use sys::{CPUID_CORTEX_M4, CPUID_CORTEX_M7, CPUID_SC300, SYST_CALIB_UNKNOWN};

#[cfg(feature = "cpu-arm-v8m")]
pub use v8m::CPUID_CORTEX_M33;

#[cfg(feature = "cpu-arm-v6m")]
pub use v6m::{CPUID_CORTEX_M0, CPUID_CORTEX_M0PLUS, CPUID_SC000};

/// `xPSR` bit positions (DDI 0403 B1.4.2).
///
/// One word holds three logical registers — `APSR`, `IPSR` and `EPSR` — and
/// the interpreter reads and writes it as one, so the masks live together.
pub mod xpsr {
    /// Negative — bit 31.
    pub const N: u32 = 1 << 31;
    /// Zero — bit 30.
    pub const Z: u32 = 1 << 30;
    /// Carry, and "not borrow" on a subtract — bit 29.
    pub const C: u32 = 1 << 29;
    /// Signed overflow — bit 28.
    pub const V: u32 = 1 << 28;
    /// Sticky saturation, set by the DSP extension and cleared only by an
    /// explicit `MSR` — bit 27.
    pub const Q: u32 = 1 << 27;
    /// Thumb state — bit 24. Clear means the core was asked for ARM state,
    /// which this architecture does not have.
    pub const T: u32 = 1 << 24;
    /// The four `GE` bits the SIMD instructions set and `SEL` reads.
    pub const GE: u32 = 0xf << 16;
    /// Where `ITSTATE` lives: `IT[7:2]` in bits 15–10 and `IT[1:0]` in bits
    /// 26–25. Split, because bits 15–10 used to be something else.
    pub const IT_MASK: u32 = (0x3f << 10) | (3 << 25);
    /// The exception number, bits 8–0. Zero means Thread mode.
    pub const EXCEPTION: u32 = 0x1ff;
    /// The five `APSR` condition and saturation flags.
    pub const FLAGS: u32 = N | Z | C | V | Q;
    /// Everything an exception return may restore, exception number aside.
    pub const WRITABLE: u32 = FLAGS | GE | T | IT_MASK;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// The optional parts of the architecture this instance has.
///
/// Independently selectable rather than implied by a version, because ARM's
/// versions are a lattice and not a chain (`ROADMAP.md` §6.1.1): a Cortex-M3
/// is ARMv7-M with no DSP, a Cortex-M4 is ARMv7E-M with it, and both exist.
/// An instruction the configured part does not have must trap as UNDEFINED —
/// that is how guests probe for a feature, so "we decoded it anyway" is a
/// conformance failure.
///
/// `#[non_exhaustive]`, like [`Config`]: copy a named set
/// ([`Extensions::CORTEX_M4`]) and change a field, so a new optional
/// extension is not a breaking change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct Extensions {
    /// The DSP (E) extension: the SIMD add/sub family, saturating
    /// arithmetic, the half-word and dual multiplies, `SEL`, `USAD8`, `PKH`
    /// and the packing extends. This is what the E in ARMv7E-M means.
    pub dsp: bool,
    /// Which floating-point unit, if any.
    ///
    /// Requires the `cpu-arm-v7m-fp` feature to be anything but
    /// [`FpUnit::None`]: [`ArmV7m::new`] forces it back rather than letting
    /// [`ArmV7m::config`] claim a unit that was not compiled in, so a build
    /// without the feature gets a part with no FPU and a `CPACR` that says so.
    pub fp: FpUnit,
    /// A memory protection unit with eight regions: PMSAv7, or PMSAv8 on an
    /// Armv8-M part, or PMSAv6 on an ARMv6-M one.
    pub mpu: bool,
    /// Unprivileged Thread mode: whether `CONTROL.nPRIV` exists.
    ///
    /// Always there on ARMv7-M and Armv8-M. On ARMv6-M it is the optional
    /// *Unprivileged/Privileged Extension* (DDI 0419E B1.3.1): a Cortex-M0
    /// has no unprivileged execution at all, so `nPRIV` is RAZ/WI and every
    /// instruction runs privileged, and a Cortex-M0+ may be built with or
    /// without it. PMSAv6 requires it (DDI 0419E B3.5), because an MPU with
    /// nothing unprivileged to restrict would be a decoration.
    pub unpriv: bool,
}

impl Extensions {
    /// A Cortex-M3: ARMv7-M, no DSP, an MPU if the SoC bought one.
    pub const CORTEX_M3: Extensions = Extensions {
        dsp: false,
        fp: FpUnit::None,
        mpu: true,
        unpriv: true,
    };
    /// An SC300: the Cortex-M3's options — ARMv7-M, no DSP, no FPU — with
    /// its optional PMSAv7 MPU modelled present, as the Cortex-M3's is.
    /// The same options as [`Extensions::CORTEX_M3`] today, kept apart so
    /// that a change to one part cannot quietly move the other.
    pub const SC300: Extensions = Extensions {
        dsp: false,
        fp: FpUnit::None,
        mpu: true,
        unpriv: true,
    };
    /// A Cortex-M4 or M7 without the floating-point option.
    pub const CORTEX_M4: Extensions = Extensions {
        dsp: true,
        fp: FpUnit::None,
        mpu: true,
        unpriv: true,
    };
    /// A Cortex-M4**F**: the same part with FPv4-SP-D16.
    pub const CORTEX_M4F: Extensions = Extensions {
        fp: FpUnit::V4Sp,
        ..Extensions::CORTEX_M4
    };
    /// A Cortex-M7 with the single-precision FPU: FPv5-SP-D16.
    pub const CORTEX_M7F: Extensions = Extensions {
        fp: FpUnit::V5Sp,
        ..Extensions::CORTEX_M4
    };
    /// A Cortex-M33 with the DSP extension and an MPU, without the FPU.
    /// The MPU here is PMSAv8 rather than PMSAv7; [`Config::arch`] is what
    /// decides which.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    pub const CORTEX_M33: Extensions = Extensions::CORTEX_M4;
    /// A Cortex-M33 with FPv5-SP-D16 — the configuration an STM32U5 ships.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    pub const CORTEX_M33F: Extensions = Extensions::CORTEX_M7F;
    /// A Cortex-M0: no DSP, no FPU, no MPU, and no unprivileged execution —
    /// "software execution is always privileged" (Cortex-M0 TRM, 3.2).
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const CORTEX_M0: Extensions = Extensions {
        dsp: false,
        fp: FpUnit::None,
        mpu: false,
        unpriv: false,
    };
    /// A Cortex-M0+ with both of its protection options built in: the
    /// eight-region PMSAv6 MPU and the Unprivileged/Privileged Extension
    /// that PMSAv6 requires (Cortex-M0+ TRM, Table 1-1).
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const CORTEX_M0PLUS: Extensions = Extensions {
        mpu: true,
        unpriv: true,
        ..Extensions::CORTEX_M0
    };
    /// An SC000: a PMSAv6 core, so the MPU and with it the
    /// Unprivileged/Privileged Extension PMSAv6 requires (DDI 0419E B3.5).
    /// The same options as [`Extensions::CORTEX_M0PLUS`] today, kept apart
    /// so that a change to one part cannot quietly move the other.
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const SC000: Extensions = Extensions {
        mpu: true,
        unpriv: true,
        ..Extensions::CORTEX_M0
    };
}

/// Which M-profile architecture the part implements.
///
/// One flag rather than a scatter of part checks: everything Armv8-M
/// Mainline changes — the stack limits, the `EXC_RETURN` format, PMSAv8, the
/// new instructions — keys off this, and so does everything ARMv6-M takes
/// away, so a guest's view and the code that builds it cannot disagree about
/// which part this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum Arch {
    /// ARMv7-M, or ARMv7E-M with [`Extensions::dsp`] (DDI 0403).
    #[default]
    V7m,
    /// Armv8-M Mainline (DDI 0553), **without the Security Extension** —
    /// see [`v8m`] for what that does and does not include.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    V8mMainline,
    /// ARMv6-M (DDI 0419): the Cortex-M0 family. Not "v7-M with some
    /// extensions off" — the instruction set is a subset no v7-M extension
    /// flag describes, and the exception model has one fault — so it is an
    /// architecture of its own here; see [`v6m`] for what changes.
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    V6m,
}

/// Which revision of the floating-point extension a part has.
///
/// Not a boolean, because the difference is guest-visible twice over:
/// `MVFR2.FPMisc` advertises the FPv5 instruction group, and an FPv5 encoding
/// executed on an FPv4 part must be UNDEFINED rather than quietly working.
/// Firmware probes by executing, so "we decoded it anyway" is a conformance
/// failure (`ROADMAP.md` §6.1.1).
///
/// **Both variants are single-precision.** FPv4-SP-D16 and FPv5-SP-D16 are
/// real part configurations; an FPv5-D16 Cortex-M7 with double precision is
/// not modelled, so `MVFR0.D_Precision` reads zero and a `VADD.F64` traps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FpUnit {
    /// No floating-point unit. `CPACR.CP10`/`CP11` are RAZ/WI and every
    /// coprocessor 10/11 encoding raises `UFSR.NOCP`.
    #[default]
    None,
    /// FPv4-SP-D16, the Cortex-M4F unit: single precision, sixteen doubles'
    /// worth of registers, no `VSEL`/`VMAXNM`/`VRINT`/`VCVTA`.
    V4Sp,
    /// FPv5-SP-D16, the single-precision Cortex-M7 unit: FPv4-SP plus the
    /// Armv8-style `VSEL`, `VMAXNM`/`VMINNM`, `VCVTA/N/P/M` and `VRINT*`.
    V5Sp,
}

impl FpUnit {
    /// Whether any floating-point unit is present.
    #[must_use]
    #[inline]
    pub const fn present(self) -> bool {
        !matches!(self, FpUnit::None)
    }

    /// Whether the FPv5 instruction group is present.
    #[must_use]
    #[inline]
    pub const fn has_v5(self) -> bool {
        matches!(self, FpUnit::V5Sp)
    }

    /// `MVFR0`, `MVFR1` and `MVFR2`, from the Cortex-M4 and Cortex-M7
    /// Technical Reference Manuals.
    ///
    /// `MVFR0 = 0x1011_0021`: all rounding modes, no short vectors, square
    /// root and divide present, no trapped exceptions, **no double
    /// precision**, single precision present, sixteen 64-bit registers.
    /// `MVFR1` advertises fused multiply-accumulate, half-precision
    /// conversion, default-NaN mode and flush-to-zero mode. `MVFR2.FPMisc`
    /// is four on FPv5 — "`VSEL`, `VMAXNM`/`VMINNM`, `VRINT*` and
    /// `VCVTA/N/P/M` are present" — and zero on FPv4.
    #[must_use]
    pub const fn mvfr(self) -> [u32; 3] {
        match self {
            FpUnit::None => [0; 3],
            FpUnit::V4Sp => [0x1011_0021, 0x1100_0011, 0],
            FpUnit::V5Sp => [0x1011_0021, 0x1200_0011, 0x0000_0040],
        }
    }
}

/// How this particular part differs from the generic ARMv7E-M.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run a Cortex-M3 and a Cortex-M7 in the same process. The public surface
/// is a *named part* — [`Config::CORTEX_M4`] — rather than a hand-assembled
/// set of flags, which is the rule §6.1.1 sets for the whole ARM family.
///
/// `#[non_exhaustive]` for the same reason: start from a named part and
/// adjust it — the `with_*` builders, or a field assignment — so a field added
/// for the next architecture revision (as `arch` and `reset_vtor` were for
/// Armv8-M) is not a breaking change to every caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Config {
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
    /// Byte order for data accesses. Instructions are always little-endian.
    ///
    /// [`Endian::Big`] is BE-8, the byte-invariant big-endian ARMv7-M
    /// defines: each byte keeps its address and a word load returns them
    /// reversed (DDI 0403 A3.3). It is fixed at reset on real parts, which is
    /// why it is here and not in a writable register.
    pub endian: Endian,
    /// Which optional parts of the architecture are present.
    pub ext: Extensions,
    /// The value `CPUID` reads.
    pub cpuid: u32,
    /// How many priority bits the NVIC implements, counted from the top.
    ///
    /// Three on most Cortex-M4 parts, four on many M7s, and eight is the
    /// architectural maximum. CMSIS discovers this by writing `0xFF` to a
    /// priority register and reading it back, so it has to be modelled.
    /// ARMv6-M fixes it at two (DDI 0419E B1.5.4), and [`ArmV7m::new`]
    /// holds an ARMv6-M configuration to that.
    pub priority_bits: u8,
    /// Whether this part has the bit-band alias windows.
    ///
    /// Cortex-M3 and Cortex-M4 do; Cortex-M7, Cortex-M33 and the ARMv6-M
    /// parts do not — ARMv6-M has no bit-band in its address map at all
    /// (DDI 0419E B3.1). It is a property of
    /// the *core's* bus matrix rather than of a board, which is why it is
    /// here and not a region a `.machine` file has to remember to map — see
    /// [`Config::with_bit_band`], which says why in full.
    pub bit_band: bool,
    /// Which architecture: ARMv7-M, Armv8-M Mainline or ARMv6-M.
    pub arch: Arch,
    /// `VTOR`'s reset value, and so where the reset sequence reads the
    /// initial `SP` and `PC`.
    ///
    /// DDI 0403 resets `VTOR` to zero on an ARMv7-M part, and a Cortex-M4
    /// boots from address zero; Armv8-M makes the reset value the SoC's
    /// (`INITNSVTOR`), and an STM32U5 ties it to its boot option bytes —
    /// `0x08000000` with nothing at all mapped at zero. Either way it is a
    /// property of the silicon around the core, not something software
    /// sets, which is why it is configuration and why every reset, warm or
    /// cold, puts it back. Bits `[6:0]` are zero, as `VTOR`'s are.
    pub reset_vtor: u32,
    /// Which bits of `VTOR` are implemented, or zero for a part with no
    /// `VTOR` at all.
    ///
    /// `0xFFFFFF80` on every ARMv7-M and Armv8-M part here. ARMv6-M makes
    /// the register optional and its width IMPLEMENTATION DEFINED
    /// (DDI 0419E B3.2.5): a Cortex-M0 has none, so `VTOR` is RAZ/WI and the
    /// vector table is at zero for good; a Cortex-M0+ built with it
    /// implements bits `[31:8]` (Cortex-M0+ TRM, Table 4-1), so a table must
    /// be 256-byte aligned. Software discovers the width by writing ones and
    /// reading back, which is why it is modelled rather than assumed.
    pub vtor_mask: u32,
    /// The value `SYST_CALIB` (`0xE000E01C`) reads.
    ///
    /// The SoC's, not the core's: it comes from the `STCALIB` integration
    /// input, so two boards on the same part can differ — SC000 silicon the
    /// project owner read gives `0x80000000` (`NOREF`, no `SKEW`, `TENMS`
    /// zero). Every preset uses [`SYST_CALIB_UNKNOWN`]; a board sets its
    /// own with the `syst-calib` property.
    pub syst_calib: u32,
}

impl Config {
    /// A Cortex-M4: ARMv7E-M with DSP and an MPU, no FPU, three priority
    /// bits.
    pub const CORTEX_M4: Config = Config {
        requester: RequesterId::ANONYMOUS,
        endian: Endian::Little,
        ext: Extensions::CORTEX_M4,
        cpuid: CPUID_CORTEX_M4,
        priority_bits: 3,
        bit_band: true,
        arch: Arch::V7m,
        reset_vtor: 0,
        vtor_mask: 0xffff_ff80,
        syst_calib: SYST_CALIB_UNKNOWN,
    };

    /// A Cortex-M7: the same architecture, a different `CPUID`, and four
    /// priority bits. The caches and TCMs are the SoC's, not the core's.
    pub const CORTEX_M7: Config = Config {
        cpuid: CPUID_CORTEX_M7,
        priority_bits: 4,
        // No bit-banding. The M7's bus interfaces are AXI and AHB with
        // caches and TCMs in front of them; ARM dropped the alias rather
        // than make a read-modify-write in the bus matrix coherent with all
        // of that. Firmware that bit-bands on an M7 faults, and so does
        // firmware that bit-bands here.
        bit_band: false,
        ..Config::CORTEX_M4
    };

    /// A Cortex-M4F: [`Config::CORTEX_M4`] with FPv4-SP-D16.
    ///
    /// `CPUID` is unchanged — the FPU is not part of the part number, and
    /// firmware discovers it from `MVFR0` and by writing `CPACR` — which is
    /// why this differs from `CORTEX_M4` in exactly one field.
    pub const CORTEX_M4F: Config = Config {
        ext: Extensions::CORTEX_M4F,
        ..Config::CORTEX_M4
    };

    /// A Cortex-M7 with the single-precision FPU: FPv5-SP-D16.
    pub const CORTEX_M7F: Config = Config {
        ext: Extensions::CORTEX_M7F,
        ..Config::CORTEX_M7
    };

    /// A Cortex-M33: Armv8-M Mainline with the DSP extension and an
    /// eight-region PMSAv8 MPU, no FPU, three priority bits, and **no
    /// Security Extension** — the view an STM32U5 with `TZEN = 0` gives its
    /// firmware.
    ///
    /// No bit-band alias: Armv8-M dropped it, so a Cortex-M33's
    /// `0x22000000` is whatever the SoC maps there.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    pub const CORTEX_M33: Config = Config {
        ext: Extensions::CORTEX_M33,
        cpuid: CPUID_CORTEX_M33,
        bit_band: false,
        arch: Arch::V8mMainline,
        ..Config::CORTEX_M4
    };

    /// A Cortex-M33 with FPv5-SP-D16, built `-mfpu=fpv5-sp-d16`.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    pub const CORTEX_M33F: Config = Config {
        ext: Extensions::CORTEX_M33F,
        ..Config::CORTEX_M33
    };

    /// A Cortex-M0 r0p0: ARMv6-M, two priority bits, SysTick, no `VTOR`, no
    /// MPU, no unprivileged execution and no bit-band.
    ///
    /// `CPUID` is `0x410CC200` (Cortex-M0 TRM, Table 4-1); everything else
    /// is DDI 0419E's fixed answer for the architecture.
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const CORTEX_M0: Config = Config {
        requester: RequesterId::ANONYMOUS,
        endian: Endian::Little,
        ext: Extensions::CORTEX_M0,
        cpuid: CPUID_CORTEX_M0,
        priority_bits: 2,
        bit_band: false,
        arch: Arch::V6m,
        reset_vtor: 0,
        vtor_mask: 0,
        syst_calib: SYST_CALIB_UNKNOWN,
    };

    /// A Cortex-M0+ r0p1 with every option a part usually ships: `VTOR`
    /// (bits `[31:8]`), the eight-region PMSAv6 MPU and unprivileged Thread
    /// mode. `CPUID` is `0x410CC601` (Cortex-M0+ TRM, Table 4-2).
    ///
    /// The options are the implementer's (Cortex-M0+ TRM, Table 1-1); a board
    /// whose silicon left the MPU out says `mpu = false`.
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const CORTEX_M0PLUS: Config = Config {
        ext: Extensions::CORTEX_M0PLUS,
        cpuid: CPUID_CORTEX_M0PLUS,
        vtor_mask: 0xffff_ff00,
        ..Config::CORTEX_M0
    };

    /// An Arm SecurCore SC000: an ARMv6-M **PMSAv6** core — the eight-region
    /// PMSAv6 MPU, so unprivileged Thread mode too (DDI 0419E B3.5 makes the
    /// one require the other), and `VTOR` at `0xE000ED08` holding bits
    /// `[31:7]`. No MMU and no address remapping: the v6-M default memory
    /// map, with the MPU the only thing that can refuse an access.
    ///
    /// The SC000 Technical Reference Manual (ARM DDI 0456) is not publicly
    /// available. The PMSAv6 MPU and `VTOR` are the project owner's
    /// statement of the part; the `VTOR` width is CMSIS `core_sc000.h`'s
    /// (Apache-2.0). A board whose silicon left the MPU out says
    /// `mpu = false`. [`CPUID_SC000`] (r0p0, part number `0xC30`),
    /// `MPU_TYPE` (eight regions) and `CCR` (`0x208`) are as the project
    /// owner read them from SC000 silicon; another revision sets
    /// [`Config::cpuid`] (or the `cpuid` property).
    #[cfg(feature = "cpu-arm-v6m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v6m")))]
    pub const SC000: Config = Config {
        ext: Extensions::SC000,
        cpuid: CPUID_SC000,
        vtor_mask: 0xffff_ff80,
        ..Config::CORTEX_M0
    };

    /// Whether this is an ARMv6-M part.
    ///
    /// Always `false` in a build without `cpu-arm-v6m`, which is what lets
    /// the optimiser delete every v6-M branch from a v7-M-only core.
    #[must_use]
    #[inline]
    pub const fn is_v6m(&self) -> bool {
        #[cfg(feature = "cpu-arm-v6m")]
        {
            matches!(self.arch, Arch::V6m)
        }
        #[cfg(not(feature = "cpu-arm-v6m"))]
        {
            false
        }
    }

    /// Whether this is an Armv8-M Mainline part.
    ///
    /// Always `false` in a build without `cpu-arm-v8m`, which is what lets
    /// the optimiser delete every v8-M branch from a v7-M-only core.
    #[must_use]
    #[inline]
    pub const fn is_v8m(&self) -> bool {
        #[cfg(feature = "cpu-arm-v8m")]
        {
            matches!(self.arch, Arch::V8mMainline)
        }
        #[cfg(not(feature = "cpu-arm-v8m"))]
        {
            false
        }
    }

    /// Same configuration, booting from a different vector table — see
    /// [`Config::reset_vtor`]. The bits `VTOR` does not implement are
    /// dropped, as `VTOR` drops them — all of them on a part with none.
    #[must_use]
    pub const fn with_reset_vtor(mut self, vtor: u32) -> Config {
        self.reset_vtor = vtor & self.vtor_mask;
        self
    }

    /// A Cortex-M3: ARMv7-M without the DSP extension, so every `SADD8`,
    /// `QADD`, `SMLAD` and `PKHBT` traps as UNDEFINED.
    pub const CORTEX_M3: Config = Config {
        ext: Extensions::CORTEX_M3,
        cpuid: 0x412f_c231,
        ..Config::CORTEX_M4
    };

    /// An Arm SecurCore SC300: the SecurCore counterpart of the Cortex-M3 —
    /// ARMv7-M without the DSP extension, no FPU, the optional eight-region
    /// PMSAv7 MPU modelled present, three priority bits and the bit-band
    /// windows, all as [`Config::CORTEX_M3`].
    ///
    /// The SC300 Technical Reference Manual is not publicly available, so
    /// this preset rests on what is. CMSIS `core_sc300.h` (V5.1.0,
    /// Apache-2.0) gives the SCB and SCnSCB the Cortex-M3's layout — `ACTLR`
    /// with `DISMCYCINT`/`DISDEFWBUF`/`DISFOLD` only, `CPACR` for a part
    /// with no FPU, the MPU behind `__MPU_PRESENT` — plus an
    /// IMPLEMENTATION DEFINED `SFCR` at `0xE000EF90`, which reads as zero
    /// here. It also gives `VTOR` a `TBLBASE` bit at 29 and a 22-bit
    /// `TBLOFF` at `[28:7]` — the layout `core_cm3.h` gives a Cortex-M3
    /// before r2p1, rather than DDI 0403E's `[31:7]` — so `VTOR` implements
    /// bits `[29:7]` and no table above `0x3FFFFF80` can be selected (the
    /// Cortex-M3 preset here is r2p1 and keeps `[31:7]`). A board whose
    /// silicon left the MPU out says `mpu = false`. The `CPUID` part number
    /// is not published; [`CPUID_SC300`] carries only the two fields
    /// DDI 0403E fixes, and a board that knows its silicon's value sets
    /// [`Config::cpuid`] (or the `cpuid` property).
    pub const SC300: Config = Config {
        ext: Extensions::SC300,
        cpuid: CPUID_SC300,
        vtor_mask: 0x3fff_ff80,
        ..Config::CORTEX_M3
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Config {
        self.requester = id;
        self
    }

    /// Same configuration, in the given byte order.
    #[must_use]
    pub const fn with_endian(mut self, endian: Endian) -> Config {
        self.endian = endian;
        self
    }

    /// Same configuration, with or without the bit-band alias windows.
    ///
    /// # Bit-banding belongs to the core
    ///
    /// The alias is decoded by the processor's own bus matrix, so it is the
    /// core's view of its address space and not a device, not a region and
    /// not a line in a `.machine` file. Three things follow, and each of
    /// them is wrong if the alias is modelled anywhere else:
    ///
    /// * it reaches **whatever** is mapped at the target, MMIO with read
    ///   side effects included, because the remap happens upstream of the
    ///   decode rather than inside some peripheral;
    /// * it is there even on a board that maps its SRAM somewhere else
    ///   entirely, because `0x22000000` aliases `0x20000000` whether or not
    ///   anything answers at `0x20000000`;
    /// * it disappears with the part, not with the board — which is exactly
    ///   what this flag is.
    ///
    /// The MPU sits *between* the core and that bus matrix, so an MPU region
    /// covering the bit-band region does not cover its alias. That is the
    /// hardware's behaviour and the reason CMSIS MPU setups have to describe
    /// both windows.
    #[must_use]
    pub const fn with_bit_band(mut self, bit_band: bool) -> Config {
        self.bit_band = bit_band;
        self
    }

    /// Same configuration, with a different number of implemented priority
    /// bits. Clamped to `1..=8`.
    #[must_use]
    pub const fn with_priority_bits(mut self, bits: u8) -> Config {
        self.priority_bits = if bits == 0 {
            1
        } else if bits > 8 {
            8
        } else {
            bits
        };
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config::CORTEX_M4
    }
}

/// The part names a `.machine` file may give as `part`.
///
/// The `f` spellings are only offered where the floating-point unit is
/// compiled in, so a board that asks for a Cortex-M4F in a build without the
/// `cpu-arm-v7m-fp` feature is rejected by the validator with the list of what
/// it could have said — rather than quietly getting a part with no FPU and
/// faulting on its first `VMOV`. The same goes for each architecture: a
/// `cortex-m33` needs `cpu-arm-v8m` and a `cortex-m0` needs `cpu-arm-v6m`,
/// and `cortex-m33f` needs both `cpu-arm-v8m` and the FPU.
const PARTS: &[&str] = &[
    #[cfg(feature = "cpu-arm-v6m")]
    "cortex-m0",
    #[cfg(feature = "cpu-arm-v6m")]
    "cortex-m0plus",
    // The spelling Arm's own documents use; `+` is an ordinary character in
    // a property string.
    #[cfg(feature = "cpu-arm-v6m")]
    "cortex-m0+",
    #[cfg(feature = "cpu-arm-v6m")]
    "sc000",
    "cortex-m3",
    "sc300",
    "cortex-m4",
    #[cfg(feature = "cpu-arm-v7m-fp")]
    "cortex-m4f",
    "cortex-m7",
    #[cfg(feature = "cpu-arm-v7m-fp")]
    "cortex-m7f",
    #[cfg(feature = "cpu-arm-v8m")]
    "cortex-m33",
    #[cfg(all(feature = "cpu-arm-v8m", feature = "cpu-arm-v7m-fp"))]
    "cortex-m33f",
];

// ---------------------------------------------------------------------------
// The visible register file
// ---------------------------------------------------------------------------

/// The architectural register file, as a debugger, a test or a snapshot wants
/// it.
///
/// `MSP` and `PSP` are both here, whichever one `R13` currently is: a
/// debugger showing only the selected stack pointer is missing half of what
/// went wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Regs {
    /// `R0`–`R15`. `r[13]` is the selected stack pointer and `r[15]` the PC.
    pub r: [u32; 16],
    /// The main stack pointer.
    pub msp: u32,
    /// The process stack pointer.
    pub psp: u32,
    /// `xPSR`.
    pub xpsr: u32,
    /// `PRIMASK.PM`.
    pub primask: bool,
    /// `FAULTMASK.FM`.
    pub faultmask: bool,
    /// `BASEPRI`.
    pub basepri: u8,
    /// `CONTROL`.
    pub control: u32,
}

impl Regs {
    /// The state a power-on leaves behind, before the reset sequence runs.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            r: [0; 16],
            msp: 0,
            psp: 0,
            xpsr: xpsr::T,
            primask: false,
            faultmask: false,
            basepri: 0,
            control: 0,
        }
    }

    /// The exception being handled, or [`Exception::THREAD`].
    #[must_use]
    pub const fn exception(&self) -> Exception {
        Exception((self.xpsr & xpsr::EXCEPTION) as u16)
    }

    /// Whether the core is in Handler mode.
    #[must_use]
    pub const fn in_handler(&self) -> bool {
        self.xpsr & xpsr::EXCEPTION != 0
    }
}

impl Default for Regs {
    fn default() -> Regs {
        Regs::new()
    }
}

impl fmt::Display for Regs {
    /// The one-line form a trace log wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, value) in self.r.iter().enumerate() {
            write!(f, "r{i}:{value:08x} ")?;
        }
        write!(
            f,
            "xpsr:{:08x} [{}{}{}{}{}] {}",
            self.xpsr,
            if self.xpsr & xpsr::N != 0 { 'N' } else { 'n' },
            if self.xpsr & xpsr::Z != 0 { 'Z' } else { 'z' },
            if self.xpsr & xpsr::C != 0 { 'C' } else { 'c' },
            if self.xpsr & xpsr::V != 0 { 'V' } else { 'v' },
            if self.xpsr & xpsr::Q != 0 { 'Q' } else { 'q' },
            self.exception()
        )
    }
}

// ---------------------------------------------------------------------------
// Interrupt inputs
// ---------------------------------------------------------------------------

/// How many `u32` words the external-interrupt level bitmap needs.
const IRQ_WORDS: usize = Exception::COUNT / 32;

/// The external interrupt inputs, kept outside the execution lock.
///
/// Atomics rather than fields under the mutex: a device asserting an IRQ from
/// inside a write the CPU itself issued would otherwise re-enter the CPU's own
/// critical section, which is a deadlock under `native-std` and a panic under
/// `single` (`ROADMAP.md` §4.7).
///
/// These are *levels*. A level-sensitive peripheral re-pends for as long as it
/// is asserted, which is what a device whose status register has not been
/// cleared does; an edge-triggered source writes `NVIC_ISPR` or `STIR`
/// instead and does not appear here at all.
#[derive(Debug)]
pub struct Lines {
    level: [AtomicU32; IRQ_WORDS],
    nmi: AtomicBool,
    reset: AtomicBool,
}

impl Default for Lines {
    fn default() -> Lines {
        Lines {
            level: [const { AtomicU32::new(0) }; IRQ_WORDS],
            nmi: AtomicBool::new(false),
            reset: AtomicBool::new(false),
        }
    }
}

impl Lines {
    /// Drive the NMI input. Level-sensitive, like the rest of them: NMI is
    /// non-maskable, not edge-triggered (DDI 0403 B1.5.14).
    fn set_nmi(&self, asserted: bool) {
        self.nmi.store(asserted, Ordering::Release);
    }

    fn nmi(&self) -> bool {
        self.nmi.load(Ordering::Acquire)
    }

    /// Drive the reset input. A high level latches a request; the sequence
    /// itself runs on the next step, because a reset is a signal and not a
    /// method call.
    fn set_reset(&self, asserted: bool) {
        if asserted {
            self.reset.store(true, Ordering::Release);
        }
    }

    /// Take the latched reset request, if there is one.
    fn take_reset(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    fn set(&self, irq: u16, asserted: bool) {
        let n = usize::from(irq);
        if n >= Exception::COUNT - 16 {
            return;
        }
        let bit = 1u32 << (n % 32);
        if asserted {
            self.level[n / 32].fetch_or(bit, Ordering::Release);
        } else {
            self.level[n / 32].fetch_and(!bit, Ordering::Release);
        }
    }

    fn get(&self, irq: u16) -> bool {
        let n = usize::from(irq);
        n < Exception::COUNT - 16
            && self.level[n / 32].load(Ordering::Acquire) & (1 << (n % 32)) != 0
    }

    fn snapshot(&self) -> [u32; IRQ_WORDS] {
        let mut out = [0u32; IRQ_WORDS];
        for (slot, atomic) in out.iter_mut().zip(self.level.iter()) {
            *slot = atomic.load(Ordering::Acquire);
        }
        out
    }

    fn restore(&self, values: &[u32; IRQ_WORDS]) {
        for (atomic, value) in self.level.iter().zip(values.iter()) {
            atomic.store(*value, Ordering::Release);
        }
    }
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// How many external interrupts a `wire` statement may name.
///
/// Sixteen of the 256 exception numbers are the system exceptions, so 240 is
/// what is left — the most a Cortex-M4 or M7 implements. A real part wires far
/// fewer; naming one this core has no vector for is a machine-file error, not
/// a silent no-op.
pub const IRQ_LINES: u32 = (Exception::COUNT - 16) as u32;

/// The pins the machine layer has taken, keeping the strong reference.
///
/// A net holds its sinks *weakly* (§4.3), so a pin nothing else keeps alive
/// dies the moment `sink` returns and the wire silently delivers to nothing.
/// The device owns them; the wire refers to them.
#[derive(Debug, Default)]
struct Pins {
    interrupts: Vec<Arc<InterruptPin>>,
    nmi: Option<Arc<NmiPin>>,
    reset: Option<Arc<ResetPin>>,
}

// ---------------------------------------------------------------------------
// The core
// ---------------------------------------------------------------------------

/// Everything the interpreter mutates, behind one lock.
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
    /// The spin detector's per-core state (`core::spin`).
    ///
    /// Beside the space rather than inside [`State`], and for the same reason
    /// the space is: neither is architectural. [`Device::load`] replaces the
    /// whole `State`, and a restore that also unplugged the detector the
    /// machine attached would be a diagnostic that quietly stops working after
    /// the first snapshot.
    spin: spin::Watch,
    /// Where every reset reads the vector table from: [`Config::reset_vtor`]
    /// until [`ArmV7m::set_vtor`] says otherwise. Configuration rather than
    /// architectural state, so beside [`State`] and not in the snapshot.
    reset_vtor: u32,
    /// The cursor the machine layer hands every runnable device, to publish
    /// this core's position into as it runs (`core::sched::TickCursor`).
    /// Wiring, not architectural state, so it survives a snapshot restore.
    cursor: Option<TickCursor>,
    /// For the length of one [`ArmV7m::run_budget`] call, what to add to
    /// [`State::cycles`] to get this core's position in its own domain:
    /// the forest's anchor plus the debt carried in, minus the cycle count at
    /// entry. `None` outside a scheduled run, which publishes nothing.
    ///
    /// Re-anchored every call rather than publishing `cycles` raw: a reset
    /// or a snapshot load replaces `State` and with it the count, and the
    /// scheduler's anchor is the one number nothing in the guest can move.
    live_offset: Option<u64>,
}

impl fmt::Debug for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Session")
            .field("state", &self.state)
            .field("space", &self.space.as_ref().map(|s| s.name()))
            .finish()
    }
}

/// An ARMv7E-M core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own `DEVICE`-ranked
/// locks, which drive `WIRE`-ranked lines. The ladder runs in the direction
/// calls travel.
///
/// The interrupt inputs are *not* under that lock — they are atomics, so a
/// device asserting an IRQ from inside a write the CPU itself issued cannot
/// re-enter the CPU's own critical section.
#[derive(Debug)]
pub struct ArmV7m {
    cfg: Config,
    lines: Arc<Lines>,
    pins: sync::Mutex<Pins>,
    session: sync::Mutex<Session>,
}

impl ArmV7m {
    /// A core in its power-on state, with no address space.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](ArmV7m::attach_space) and [`Device::realize`].
    /// The first [`step`](ArmV7m::step) runs the reset sequence, which is
    /// what reads `SP` and `PC` out of the vector table.
    ///
    /// Without the `cpu-arm-v7m-fp` feature, `cfg.ext.fp` is forced back to
    /// [`FpUnit::None`]: the unit is not compiled in, and a configuration that
    /// claimed one would make [`ArmV7m::config`] lie.
    ///
    /// An ARMv6-M configuration is held to what the architecture allows
    /// whatever fields it was built with: two priority bits, no DSP, no FPU,
    /// no bit-band, and no MPU without the unprivileged extension PMSAv6
    /// depends on (DDI 0419E B1.5.4, B3.1, B3.5).
    #[must_use]
    pub fn new(cfg: Config) -> ArmV7m {
        #[cfg(not(feature = "cpu-arm-v7m-fp"))]
        let cfg = Config {
            ext: Extensions {
                fp: FpUnit::None,
                ..cfg.ext
            },
            ..cfg
        };
        let cfg = if cfg.is_v6m() {
            Config {
                ext: Extensions {
                    dsp: false,
                    fp: FpUnit::None,
                    mpu: cfg.ext.mpu && cfg.ext.unpriv,
                    ..cfg.ext
                },
                priority_bits: 2,
                bit_band: false,
                reset_vtor: cfg.reset_vtor & cfg.vtor_mask,
                ..cfg
            }
        } else {
            cfg
        };
        ArmV7m {
            cfg,
            lines: Arc::new(Lines::default()),
            pins: sync::Mutex::with_rank(LockRank::DEVICE, Pins::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    space: None,
                    spin: spin::Watch::new(cfg.requester),
                    reset_vtor: cfg.reset_vtor,
                    cursor: None,
                    live_offset: None,
                },
            ),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or value, or a property nothing here
    /// accepts was given — a typo'd property that was silently ignored is an
    /// afternoon lost.
    pub fn from_props(props: &Props) -> Result<ArmV7m> {
        let mut r = props.reader();
        let part = if props.contains("part") {
            r.require_enum("part", PARTS)?
        } else {
            "cortex-m4"
        };
        let big_endian = r.or("big-endian", false)?;
        let priority_bits = r.or_range("priority-bits", 0u64, 0..=8)?;
        let dsp_override = r.or("dsp", true)?;
        let mpu_override = r.or("mpu", true)?;
        let fp_override = r.or("fp", true)?;
        let vtor = r.or_range("vtor", 0u64, 0..=0xffff_ff80)?;
        let cpuid = if props.contains("cpuid") {
            Some(r.or_range("cpuid", 0u64, 0..=0xffff_ffff)? as u32)
        } else {
            None
        };
        let syst_calib =
            r.or_range("syst-calib", u64::from(SYST_CALIB_UNKNOWN), 0..=0xffff_ffff)? as u32;
        r.finish()?;
        let base = match part {
            #[cfg(feature = "cpu-arm-v6m")]
            "cortex-m0" => Config::CORTEX_M0,
            #[cfg(feature = "cpu-arm-v6m")]
            "cortex-m0plus" | "cortex-m0+" => Config::CORTEX_M0PLUS,
            #[cfg(feature = "cpu-arm-v6m")]
            "sc000" => Config::SC000,
            "cortex-m3" => Config::CORTEX_M3,
            "sc300" => Config::SC300,
            "cortex-m4" => Config::CORTEX_M4,
            "cortex-m4f" => Config::CORTEX_M4F,
            "cortex-m7" => Config::CORTEX_M7,
            "cortex-m7f" => Config::CORTEX_M7F,
            #[cfg(feature = "cpu-arm-v8m")]
            "cortex-m33" => Config::CORTEX_M33,
            #[cfg(all(feature = "cpu-arm-v8m", feature = "cpu-arm-v7m-fp"))]
            "cortex-m33f" => Config::CORTEX_M33F,
            // `require_enum` has already rejected anything else.
            _ => Config::CORTEX_M4,
        };
        if u64::from(base.vtor_mask) & vtor != vtor {
            // Not silently rounded: a vector table the core would then read
            // from somewhere other than where the board file said is a
            // reset into the weeds with nothing pointing at the cause. A
            // Cortex-M0 has no VTOR at all, so any non-zero value is wrong.
            return Err(Error::Property(if base.vtor_mask == 0 {
                alloc::format!("property `vtor`: {part} has no VTOR, so it always boots from 0")
            } else {
                alloc::format!(
                    "property `vtor`: {vtor:#x} is not a value {part}'s VTOR can hold \
                     (implemented bits {:#010x})",
                    base.vtor_mask
                )
            }));
        }
        if base.is_v6m() && priority_bits != 0 && priority_bits != 2 {
            // ARMv6-M's priority fields are two bits wide by definition
            // (DDI 0419E B1.5.4); there is no part to model with any other.
            return Err(Error::Property(alloc::format!(
                "property `priority-bits`: ARMv6-M ({part}) implements exactly 2, not {priority_bits}"
            )));
        }
        let mut cfg = Config {
            endian: if big_endian {
                Endian::Big
            } else {
                Endian::Little
            },
            ext: Extensions {
                dsp: base.ext.dsp && dsp_override,
                // `fp = false` on an `f` part is how a board models the same
                // silicon with the unit left out; there is no way to add one
                // to a part that does not have it, which is why this only
                // ever subtracts.
                fp: if fp_override {
                    base.ext.fp
                } else {
                    FpUnit::None
                },
                mpu: base.ext.mpu && mpu_override,
                unpriv: base.ext.unpriv,
            },
            reset_vtor: vtor as u32,
            cpuid: cpuid.unwrap_or(base.cpuid),
            syst_calib,
            ..base
        };
        if priority_bits != 0 {
            cfg = cfg.with_priority_bits(priority_bits as u8);
        }
        Ok(ArmV7m::new(cfg))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Read `S0`–`S31`.
    ///
    /// Separate from [`ArmV7m::regs`] rather than a field of [`Regs`]: a
    /// Cortex-M3 has no such registers, and a debugger that wants them can
    /// ask. Reading one here does **not** resolve a pending lazy push, which
    /// is deliberate — these are the architectural registers, and where the
    /// interrupted context's copy currently lives is `FPCAR`'s business.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
    #[must_use]
    pub fn s(&self, index: u8) -> u32 {
        self.session.lock().state.fp.s(index)
    }

    /// Write `Sn`.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
    pub fn set_s(&self, index: u8, value: u32) {
        self.session.lock().state.fp.set_s(index, value);
    }

    /// Read `FPSCR`.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
    #[must_use]
    pub fn fpscr(&self) -> u32 {
        self.session.lock().state.fp.fpscr
    }

    /// Write `FPSCR`, masked to the bits this core implements.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m-fp")))]
    pub fn set_fpscr(&self, value: u32) {
        self.session.lock().state.fp.fpscr = value & fp::fpscr::WRITABLE;
    }

    /// Read `MSPLIM` and `PSPLIM`, in that order (Armv8-M; both zero on an
    /// ARMv7-M part, which has neither).
    ///
    /// Separate from [`Regs`] for the reason [`ArmV7m::s`] is: a register
    /// most parts do not have is asked for, not carried.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    #[must_use]
    pub fn stack_limits(&self) -> (u32, u32) {
        let s = &self.session.lock().state;
        (s.msplim, s.psplim)
    }

    /// Write `MSPLIM` and `PSPLIM`. Bits `[2:0]` are RES0 and dropped;
    /// ignored on an ARMv7-M part.
    #[cfg(feature = "cpu-arm-v8m")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v8m")))]
    pub fn set_stack_limits(&self, msplim: u32, psplim: u32) {
        if self.cfg.is_v8m() {
            let s = &mut self.session.lock().state;
            s.msplim = msplim & !7;
            s.psplim = psplim & !7;
        }
    }

    /// Keep the cursor the machine layer hands every runnable device, and
    /// publish this core's position into it on every data access while a
    /// scheduled budget runs.
    ///
    /// What makes a lazily advanced peripheral answer a poll at the cycle the
    /// load happens on: without it, a quantum's every access sees the device
    /// as it was when the quantum began.
    pub fn attach_cursor(&self, cursor: &TickCursor) {
        self.session.lock().cursor = Some(cursor.clone());
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

    /// The whole register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        let s = &self.session.lock().state;
        Regs {
            r: s.r,
            msp: s.msp(),
            psp: s.psp(),
            xpsr: s.xpsr,
            primask: s.primask,
            faultmask: s.faultmask,
            basepri: s.basepri,
            control: s.control,
        }
    }

    /// Overwrite the whole register file — a debugger, a test vector, a
    /// snapshot.
    ///
    /// The stack-pointer banking is re-derived from `CONTROL` and the mode
    /// afterwards, so `r[13]` and the `MSP`/`PSP` pair cannot be left
    /// disagreeing.
    pub fn set_regs(&self, regs: Regs) {
        let s = &mut self.session.lock().state;
        s.r = regs.r;
        s.xpsr = regs.xpsr;
        s.primask = regs.primask;
        s.faultmask = regs.faultmask;
        s.basepri = regs.basepri;
        s.control = regs.control;
        // Put both banks in place, then let `sync_stack` pick which is live.
        s.sp_is_psp = false;
        s.r[13] = regs.msp;
        s.sp_other = regs.psp;
        s.sync_stack();
        s.r[13] = if s.sp_is_psp { regs.psp } else { regs.msp };
        s.sp_other = if s.sp_is_psp { regs.msp } else { regs.psp };
    }

    /// Read one of the sixteen currently visible registers.
    #[must_use]
    pub fn reg(&self, index: u8) -> u32 {
        self.session.lock().state.r[(index & 0xf) as usize]
    }

    /// Write one of the sixteen currently visible registers.
    ///
    /// Writing `R15` sets the PC directly and does not interwork.
    pub fn set_reg(&self, index: u8, value: u32) {
        self.session.lock().state.r[(index & 0xf) as usize] = value;
    }

    /// The program counter.
    #[must_use]
    pub fn pc(&self) -> u32 {
        self.session.lock().state.r[15]
    }

    /// Set the program counter.
    pub fn set_pc(&self, value: u32) {
        self.session.lock().state.r[15] = value;
    }

    /// `xPSR`.
    #[must_use]
    pub fn xpsr(&self) -> u32 {
        self.session.lock().state.xpsr
    }

    /// Write `xPSR`.
    pub fn set_xpsr(&self, value: u32) {
        let s = &mut self.session.lock().state;
        s.xpsr = value;
        s.sync_stack();
    }

    /// The main stack pointer.
    #[must_use]
    pub fn msp(&self) -> u32 {
        self.session.lock().state.msp()
    }

    /// The process stack pointer.
    #[must_use]
    pub fn psp(&self) -> u32 {
        self.session.lock().state.psp()
    }

    /// The exception currently being handled, or [`Exception::THREAD`].
    #[must_use]
    pub fn current_exception(&self) -> Exception {
        self.session.lock().state.current_exception()
    }

    /// The current execution priority (DDI 0403 B1.5.4).
    #[must_use]
    pub fn execution_priority(&self) -> i32 {
        self.session.lock().state.execution_priority()
    }

    /// Cycles executed since power-on. See `exec`'s timing model.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether the core is asleep in `WFI` or `WFE`.
    #[must_use]
    pub fn is_asleep(&self) -> bool {
        self.session.lock().state.asleep
    }

    /// Whether the core has locked up: a fault at a priority no handler can
    /// preempt. Nothing but a reset gets out (DDI 0403 B1.5.15).
    #[must_use]
    pub fn is_locked_up(&self) -> bool {
        self.session.lock().state.locked_up
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// Whether the guest asked for a system reset through `AIRCR.SYSRESETREQ`.
    ///
    /// The core cannot reset the machine — what a system reset does is the
    /// machine's business — so this is a flag the machine polls and clears.
    #[must_use]
    pub fn reset_requested(&self) -> bool {
        self.session.lock().state.sys.reset_requested
    }

    /// Clear the `SYSRESETREQ` flag, having acted on it.
    pub fn clear_reset_request(&self) {
        self.session.lock().state.sys.reset_requested = false;
    }

    /// How many accesses the address space refused, and where the last one
    /// was.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = &self.session.lock().state;
        (s.faults, s.last_fault)
    }

    /// The comment field of the most recent `SVC`.
    ///
    /// The architecture does not give hardware this value — a handler reads
    /// the instruction back out of memory — but a host implementing
    /// semihosting wants it without doing that.
    #[must_use]
    pub fn last_svc(&self) -> u8 {
        self.session.lock().state.last_svc
    }

    /// The comment field of the most recent `BKPT`.
    #[must_use]
    pub fn last_bkpt(&self) -> u8 {
        self.session.lock().state.last_bkpt
    }

    /// Do something with the NVIC, SCB, SysTick and MPU state.
    ///
    /// The whole block is reachable through guest memory at `0xE000E000`;
    /// this is the same state for a debugger, a machine that wants to seed
    /// `VTOR`, or a test.
    pub fn with_sys<T>(&self, f: impl FnOnce(&mut Sys) -> T) -> T {
        f(&mut self.session.lock().state.sys)
    }

    /// Where the vector table is.
    #[must_use]
    pub fn vtor(&self) -> u32 {
        self.session.lock().state.sys.vtor
    }

    /// Move the vector table, **and** make that where every later reset
    /// boots from.
    ///
    /// A machine whose boot ROM is not at zero calls this before the first
    /// step — the same thing [`Config::reset_vtor`] says at construction.
    /// A guest's own write to `VTOR` moves only the live value, which the
    /// next reset puts back.
    pub fn set_vtor(&self, value: u32) {
        let mut session = self.session.lock();
        // Only the bits this part's `VTOR` implements — none on a
        // Cortex-M0, whose vector table cannot move.
        session.state.sys.vtor = value & self.cfg.vtor_mask;
        session.reset_vtor = value & self.cfg.vtor_mask;
    }

    /// Drive external interrupt `irq`'s input. Level-sensitive: it re-pends
    /// for as long as it is asserted.
    ///
    /// `asserted` is the logical level, not the pin's: inverting an
    /// active-low signal belongs to whatever models the wire.
    pub fn set_irq(&self, irq: u16, asserted: bool) {
        self.lines.set(irq, asserted);
    }

    /// Whether external interrupt `irq`'s input is asserted.
    #[must_use]
    pub fn irq_asserted(&self, irq: u16) -> bool {
        self.lines.get(irq)
    }

    /// Drive the non-maskable interrupt input. Level-sensitive, like the
    /// external ones.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.set_nmi(asserted);
    }

    /// Whether the NMI input is asserted.
    #[must_use]
    pub fn nmi_asserted(&self) -> bool {
        self.lines.nmi()
    }

    /// A handle on the core's interrupt input latches.
    ///
    /// What [`InterruptPin`], [`NmiPin`] and [`ResetPin`] are built on, and
    /// deliberately *not* a handle on the core: a pin that held the core alive
    /// would close a reference cycle §4.3 forbids. A machine file never needs
    /// this — [`Device::sink`] builds the pins — but a downstream SoC crate
    /// wiring the core up by hand does.
    #[must_use]
    pub fn lines(&self) -> Arc<Lines> {
        Arc::clone(&self.lines)
    }

    /// Cycles owed to the next budget — see [`run_budget`](ArmV7m::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Make external interrupt `irq` pending once, the way a write to
    /// `NVIC_ISPR` or `STIR` would.
    ///
    /// This is the edge-triggered path: the bit stays pending until the
    /// handler is entered or software clears it, and nothing re-asserts it.
    pub fn pend_irq(&self, irq: u16) {
        if usize::from(irq) + 16 < Exception::COUNT {
            self.session
                .lock()
                .state
                .sys
                .set_pending(Exception(irq + 16), true);
        }
    }

    /// Request a reset sequence without changing any register.
    ///
    /// It runs on the next [`step`](ArmV7m::step), because a reset is a
    /// signal rather than a method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, one exception entry, or one instruction.
    ///
    /// Returns the cycles charged: zero if there is no address space, which
    /// the caller must treat as "stop", not "retry". A sleeping core returns
    /// one cycle per call and keeps sleeping.
    pub fn step(&self) -> u64 {
        let external = self.lines.snapshot();
        let nmi = self.lines.nmi();
        let reset = self.lines.take_reset();
        let mut session = self.session.lock();
        let Session {
            state,
            space,
            spin,
            reset_vtor,
            cursor,
            live_offset,
        } = &mut *session;
        if reset {
            state.reset_pending = true;
        }
        if state.reset_pending {
            // Every reset, warm or cold, boots from the SoC's vector table
            // and not from wherever software last pointed `VTOR`
            // (DDI 0403 B3.2.5; Armv8-M takes the value from the SoC).
            state.sys.vtor = *reset_vtor;
        }
        let Some(space) = space.clone() else {
            return 0;
        };
        let live = cursor.as_ref().zip(*live_offset);
        Exec::new(state, &space, &self.cfg, spin)
            .with_live(live)
            .step(&external, nmi)
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — a core cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible
    /// state.
    pub fn run(&self, budget: u64) -> u64 {
        self.session.lock().spin.refresh();
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
    /// usually runs past its end, and [`run`](ArmV7m::run) reports the
    /// overshoot honestly. The scheduler treats an overrun as fatal — the
    /// overrun has already executed past an event that should have stopped
    /// it — so what the *device* path does instead is **carry** the
    /// overshoot: it is deducted from the next budget, which keeps the core's
    /// cycle count and its domain's tick count in step over any number of
    /// quanta while never letting a single one overrun.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = {
            let mut session = self.session.lock();
            // Once per scheduler round, which is the granularity a change of
            // arming takes effect at — the per-load path holds a copy and
            // touches no atomic (`core::spin`).
            session.spin.refresh();
            let owed = session.state.debt;
            // Where this core stands in its own domain is where the forest
            // says the call began, plus the cycles it has already run that
            // this budget still owes.
            session.live_offset = session.cursor.as_ref().map(|c| {
                c.anchor()
                    .wrapping_add(owed)
                    .wrapping_sub(session.state.cycles)
            });
            owed
        };
        if owed >= ticks {
            let mut session = self.session.lock();
            session.state.debt = owed - ticks;
            session.live_offset = None;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
        }
        let mut session = self.session.lock();
        session.live_offset = None;
        if used >= allowance {
            session.state.debt = used - allowance;
            ticks
        } else {
            // The core stopped early — no address space, nothing more to run.
            // Nothing is owed, and the scheduler is told the truth.
            session.state.debt = 0;
            owed + used
        }
    }

    /// Disassemble `count` instructions starting at `addr`, reading guest
    /// memory with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around the
    /// PC must not pop a FIFO or clear a status bit on the way
    /// (`ROADMAP.md` §15, invariant 5).
    ///
    /// # There is only one kind of address here
    ///
    /// The A profile has to say whether it means a virtual or a physical
    /// address ([`Arm::disassemble_virtual`](super::aprofile::Arm::disassemble_virtual)
    /// and its physical twin). An ARMv7-M does not, and this is not an omission:
    /// the M profile is **PMSA**. Its MPU *checks* an access against a region's
    /// permissions and never remaps one, so a virtual address and a physical
    /// address are the same number by construction (ARMv7-M ARM, B3.5 "Protected
    /// Memory System Architecture"). Passing [`pc`](ArmV7m::pc) here is
    /// correct, and there is no translation for a debugger to skip.
    #[must_use]
    pub fn disassemble(&self, addr: u32, count: usize) -> Vec<Listed> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let read = |a: u32| {
            space
                .read(u64::from(a), Width::U16, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u16)
        };
        let mut out = Vec::with_capacity(count);
        let mut pc = addr;
        for _ in 0..count {
            let Some(first) = read(pc) else { break };
            let wide = isa::is_32bit(first);
            let second = if wide {
                read(pc.wrapping_add(2))
            } else {
                Some(0)
            };
            let Some(second) = second else { break };
            out.push(Listed {
                addr: pc,
                raw: (u32::from(first) << 16) | u32::from(second),
                width: if wide { 4 } else { 2 },
                // The configured architecture's decoder, so a listing for a
                // Cortex-M0 shows `CBZ` as the UNDEFINED it is there.
                insn: isa::decode_for(self.cfg.arch, first, second),
            });
            pc = pc.wrapping_add(if wide { 4 } else { 2 });
        }
        out
    }
}

/// One disassembled instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Listed {
    /// Where it is.
    pub addr: u32,
    /// The encoding, first halfword in the top sixteen bits. For a sixteen-bit
    /// instruction the low half is zero.
    pub raw: u32,
    /// Two or four bytes.
    pub width: u32,
    /// What it decoded to.
    pub insn: isa::Insn,
}

impl fmt::Display for Listed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.width == 2 {
            write!(
                f,
                "{:08x}: {:04x}      {}",
                self.addr,
                self.raw >> 16,
                self.insn
            )
        } else {
            write!(
                f,
                "{:08x}: {:04x} {:04x} {}",
                self.addr,
                self.raw >> 16,
                self.raw & 0xffff,
                self.insn
            )
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The `cpu.arm.v7m` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.arm.v7m",
    // 2: the snapshot gained `DEMCR` and the DWT's two live registers. 0.0.x,
    // so the old layout is simply gone rather than read behind a shim.
    version: 2,
    summary: "ARMv6-M / ARMv7E-M / Armv8-M Mainline (Cortex-M0/M4/M7/M33 class) CPU core with Thumb-2, DSP, NVIC and MPU",
    properties: &[
        PropertySpec {
            name: "part",
            kind: ValueKind::Str,
            required: false,
            summary: "which part to model: cortex-m0, cortex-m0plus (cortex-m0+), sc000, cortex-m3, sc300, cortex-m4[f], cortex-m7[f] or cortex-m33[f]",
        },
        PropertySpec {
            name: "cpuid",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value CPUID reads, for silicon whose revision differs from the part's default",
        },
        PropertySpec {
            name: "syst-calib",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value SYST_CALIB reads: the SoC's STCALIB input (default 0xC0000000, NOREF and SKEW, TENMS unknown)",
        },
        PropertySpec {
            name: "vtor",
            kind: ValueKind::Uint,
            required: false,
            summary: "VTOR's reset value: where every reset reads SP and PC (default 0; only bits the part's VTOR implements, none on a Cortex-M0)",
        },
        PropertySpec {
            name: "fp",
            kind: ValueKind::Bool,
            required: false,
            summary: "leave the floating-point unit out of an `f` part (default true)",
        },
        PropertySpec {
            name: "big-endian",
            kind: ValueKind::Bool,
            required: false,
            summary: "use BE-8 byte order for data accesses",
        },
        PropertySpec {
            name: "priority-bits",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many NVIC priority bits are implemented (1-8; 0 keeps the part default)",
        },
        PropertySpec {
            name: "dsp",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the DSP (E) extension is present",
        },
        PropertySpec {
            name: "mpu",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the memory protection unit (PMSAv7, or PMSAv8 on an Armv8-M part) is present",
        },
    ],
    construct: |props| Ok(Box::new(ArmV7m::from_props(props)?)),
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

impl Device for ArmV7m {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn set_spin_detector(&self, cpu: u32, detector: Option<Arc<spin::Detector>>) -> bool {
        self.session.lock().spin.attach(cpu, detector);
        true
    }

    fn attach_cursor(&self, cursor: TickCursor) {
        ArmV7m::attach_cursor(self, &cursor);
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A core with no address space cannot fetch, but
        // realize runs *before* the machine binds one, so checking here would
        // refuse every machine — that check is `Instance::bind`'s, which is
        // where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made. The
        // strong reference stays here, because a net holds its sinks weakly.
        let mut pins = self.pins.lock();
        match port {
            "nmi" => {
                let pin = Arc::new(NmiPin::new(Arc::clone(&self.lines), sources));
                pins.nmi = Some(Arc::clone(&pin));
                Some(SinkPin {
                    sink: pin,
                    line: u32::from(Exception::NMI.0),
                })
            }
            "reset" => {
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                Some(SinkPin {
                    sink: pin,
                    line: u32::from(Exception::RESET.0),
                })
            }
            _ => {
                let irq = port_index(port, "irq", IRQ_LINES)?;
                let pin = Arc::new(InterruptPin::new(
                    Arc::clone(&self.lines),
                    irq as u16,
                    sources,
                ));
                pins.interrupts.push(Arc::clone(&pin));
                Some(SinkPin {
                    sink: pin,
                    line: irq,
                })
            }
        }
    }

    fn reset(&self, kind: ResetKind) {
        {
            let mut session = self.session.lock();
            if kind == ResetKind::Cold {
                session.state = State::new(&self.cfg);
            } else {
                // A warm reset is a pulse on the reset input: the sequence
                // runs and nothing else is forced.
                session.state.reset_pending = true;
                session.state.asleep = false;
                session.state.locked_up = false;
            }
        }
        if kind == ResetKind::Cold {
            // A cold start has nothing driving the inputs yet, so zeroing them
            // is the truth. A *warm* reset does have drivers, and clearing
            // what they assert would make the reset lie about the machine —
            // so the levels are left exactly as the wires left them.
            self.lines.restore(&[0; IRQ_WORDS]);
            self.lines.set_nmi(false);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state.clone();
        for value in state.r {
            w.write_u32(value)?;
        }
        w.write_u32(state.sp_other)?;
        w.write_bool(state.sp_is_psp)?;
        w.write_u32(state.xpsr)?;
        w.write_bool(state.primask)?;
        w.write_bool(state.faultmask)?;
        w.write_u8(state.basepri)?;
        w.write_u32(state.control)?;
        w.write_u64(state.cycles)?;
        w.write_u64(state.debt)?;
        w.write_bool(state.asleep)?;
        w.write_bool(state.event)?;
        w.write_bool(state.reset_pending)?;
        w.write_bool(state.locked_up)?;
        w.write_bool(state.exclusive.is_some())?;
        w.write_u32(state.exclusive.unwrap_or(0))?;
        w.write_u64(state.faults)?;
        w.write_u32(state.last_fault)?;
        w.write_u8(state.last_svc)?;
        w.write_u8(state.last_bkpt)?;
        save_sys(&state.sys, w)?;
        // The stack limits, only on an Armv8-M part — which keeps every
        // ARMv7-M snapshot exactly the shape it was.
        #[cfg(feature = "cpu-arm-v8m")]
        if self.cfg.is_v8m() {
            w.write_u32(state.msplim)?;
            w.write_u32(state.psplim)?;
        }
        // `S0`-`S31` and `FPSCR`, only on a part that has them.
        #[cfg(feature = "cpu-arm-v7m-fp")]
        if self.cfg.ext.fp.present() {
            for value in state.fp.s {
                w.write_u32(value)?;
            }
            w.write_u32(state.fp.fpscr)?;
        }
        for word in self.lines.snapshot() {
            w.write_u32(word)?;
        }
        // The interrupt inputs are architectural: a restored machine whose
        // peripheral was already asserting must still see it.
        w.write_bool(self.lines.nmi())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(&self.cfg);
        for value in &mut state.r {
            *value = r.read_u32()?;
        }
        state.sp_other = r.read_u32()?;
        state.sp_is_psp = r.read_bool()?;
        state.xpsr = r.read_u32()?;
        state.primask = r.read_bool()?;
        state.faultmask = r.read_bool()?;
        state.basepri = r.read_u8()?;
        state.control = r.read_u32()?;
        state.cycles = r.read_u64()?;
        state.debt = r.read_u64()?;
        state.asleep = r.read_bool()?;
        state.event = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.locked_up = r.read_bool()?;
        let has_exclusive = r.read_bool()?;
        let exclusive = r.read_u32()?;
        state.exclusive = has_exclusive.then_some(exclusive);
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u32()?;
        state.last_svc = r.read_u8()?;
        state.last_bkpt = r.read_u8()?;
        load_sys(&mut state.sys, r)?;
        #[cfg(feature = "cpu-arm-v8m")]
        if self.cfg.is_v8m() {
            state.msplim = r.read_u32()? & !7;
            state.psplim = r.read_u32()? & !7;
        }
        #[cfg(feature = "cpu-arm-v7m-fp")]
        if self.cfg.ext.fp.present() {
            for value in &mut state.fp.s {
                *value = r.read_u32()?;
            }
            state.fp.fpscr = r.read_u32()?;
        }
        let mut lines = [0u32; IRQ_WORDS];
        for word in &mut lines {
            *word = r.read_u32()?;
        }
        let nmi = r.read_bool()?;
        self.session.lock().state = state;
        self.lines.restore(&lines);
        self.lines.set_nmi(nmi);
        Ok(())
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }
}

/// Write the system block into a snapshot.
///
/// Derived state — the flattened priority view, anything a cache would
/// hold — is not written; there is none here, and the bitmaps and register
/// values are the whole of it (`ROADMAP.md` §15, invariant 3).
fn save_sys(sys: &Sys, w: &mut ChunkWriter<'_>) -> Result<()> {
    for word in sys.enable {
        w.write_u32(word)?;
    }
    for word in sys.pending {
        w.write_u32(word)?;
    }
    for word in sys.active {
        w.write_u32(word)?;
    }
    for value in sys.priority {
        w.write_u8(value)?;
    }
    w.write_u8(sys.priority_bits)?;
    w.write_u32(sys.vtor)?;
    w.write_u8(sys.prigroup)?;
    w.write_u32(sys.scr)?;
    w.write_u32(sys.ccr)?;
    w.write_u32(sys.shcsr)?;
    w.write_u32(sys.cfsr)?;
    w.write_u32(sys.hfsr)?;
    w.write_u32(sys.mmfar)?;
    w.write_u32(sys.bfar)?;
    w.write_u32(sys.afsr)?;
    w.write_u32(sys.cpacr)?;
    // The floating-point block, only on a part that has one — which keeps a
    // Cortex-M3's or an FPU-less M4's chunk the same shape whichever part it
    // is. `MVFR0`-`MVFR2` are configuration rather than state and are never
    // written; `fp_present` is derived from them and so answers the same
    // question on both sides of a round trip.
    if sys.fp_present() {
        w.write_u32(sys.fpccr)?;
        w.write_u32(sys.fpcar)?;
        w.write_u32(sys.fpdscr)?;
    }
    w.write_u32(sys.cpuid)?;
    w.write_bool(sys.reset_requested)?;
    w.write_u32(sys.syst_csr)?;
    w.write_u32(sys.syst_rvr)?;
    w.write_u32(sys.syst_cvr)?;
    w.write_u32(sys.syst_calib)?;
    // The debug and trace block. `DWT_CYCCNT` is guest-visible state a busy
    // wait is sitting on, so a snapshot that dropped it would resume into a
    // loop whose start point no longer means anything.
    w.write_u32(sys.demcr)?;
    w.write_u32(sys.dwt_ctrl)?;
    w.write_u32(sys.dwt_cyccnt)?;
    w.write_u32(sys.mpu_ctrl)?;
    w.write_u32(sys.mpu_rnr)?;
    w.write_u8(sys.mpu_regions)?;
    for value in sys.mpu_rbar {
        w.write_u32(value)?;
    }
    for value in sys.mpu_rasr {
        w.write_u32(value)?;
    }
    // PMSAv8's attribute indirection, only on a part that has it.
    #[cfg(feature = "cpu-arm-v8m")]
    if sys.v8m {
        w.write_u32(sys.mpu_mair[0])?;
        w.write_u32(sys.mpu_mair[1])?;
    }
    Ok(())
}

/// Read the system block back out of a snapshot.
fn load_sys(sys: &mut Sys, r: &mut ChunkReader<'_>) -> Result<()> {
    for word in &mut sys.enable {
        *word = r.read_u32()?;
    }
    for word in &mut sys.pending {
        *word = r.read_u32()?;
    }
    for word in &mut sys.active {
        *word = r.read_u32()?;
    }
    for value in &mut sys.priority {
        *value = r.read_u8()?;
    }
    sys.priority_bits = r.read_u8()?;
    sys.vtor = r.read_u32()?;
    sys.prigroup = r.read_u8()?;
    sys.scr = r.read_u32()?;
    sys.ccr = r.read_u32()?;
    sys.shcsr = r.read_u32()?;
    sys.cfsr = r.read_u32()?;
    sys.hfsr = r.read_u32()?;
    sys.mmfar = r.read_u32()?;
    sys.bfar = r.read_u32()?;
    sys.afsr = r.read_u32()?;
    sys.cpacr = r.read_u32()?;
    if sys.fp_present() {
        sys.fpccr = r.read_u32()?;
        sys.fpcar = r.read_u32()?;
        sys.fpdscr = r.read_u32()?;
    }
    sys.cpuid = r.read_u32()?;
    sys.reset_requested = r.read_bool()?;
    sys.syst_csr = r.read_u32()?;
    sys.syst_rvr = r.read_u32()?;
    sys.syst_cvr = r.read_u32()?;
    sys.syst_calib = r.read_u32()?;
    sys.demcr = r.read_u32()?;
    sys.dwt_ctrl = r.read_u32()?;
    sys.dwt_cyccnt = r.read_u32()?;
    sys.mpu_ctrl = r.read_u32()?;
    sys.mpu_rnr = r.read_u32()?;
    sys.mpu_regions = r.read_u8()?;
    for value in &mut sys.mpu_rbar {
        *value = r.read_u32()?;
    }
    for value in &mut sys.mpu_rasr {
        *value = r.read_u32()?;
    }
    #[cfg(feature = "cpu-arm-v8m")]
    if sys.v8m {
        sys.mpu_mair[0] = r.read_u32()?;
        sys.mpu_mair[1] = r.read_u32()?;
    }
    Ok(())
}

impl Initiator for ArmV7m {
    fn requester(&self) -> RequesterId {
        self.cfg.requester
    }
}

/// The machine layer's half: a core needs an address space, and this is where
/// the machine gives it one.
impl crate::machine::Instance for ArmV7m {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: "an ARMv7-M core needs an address space to fetch from (`space = mem`)"
                .to_string(),
        })?;
        self.attach_space(Arc::clone(space));
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(ArmV7m::from_props(props)?)))
}

/// What the validator should know about `cpu.arm.v7m`.
///
/// # Naming an interrupt
///
/// The NVIC is *inside* this core — its registers are part of the system block
/// at `0xE000E000` and its pending bits are core state — so there is no
/// separate controller object for a machine file to wire through, the way a
/// RISC-V board wires through a PLIC. A peripheral raises interrupt *n* by
/// driving this core's pin `irq{n}` directly:
///
/// ```text
/// wire usart2.irq -> cpu.irq38
/// ```
///
/// Which number a peripheral gets is a **part** fact, not an architecture one
/// and not a device one: it is a row of the vendor's vector table (for an
/// STM32F407, RM0090 Table 62). So it belongs in the `.machine` file, where
/// the part is chosen, and a device model must never hard-code one.
///
/// `nmi` and `reset` are the two non-numbered inputs. There is no `irq` pin
/// without a number, and no output pin at all: an M-profile core drives
/// nothing this model carries.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("part", ValueKind::Str).values(PARTS))
        .prop(PropSchema::new("big-endian", ValueKind::Bool))
        .prop(PropSchema::new("priority-bits", ValueKind::Uint).range(0, 8))
        .prop(PropSchema::new("dsp", ValueKind::Bool))
        .prop(PropSchema::new("mpu", ValueKind::Bool))
        .prop(PropSchema::new("fp", ValueKind::Bool))
        .prop(PropSchema::new("vtor", ValueKind::Uint).range(0, 0xffff_ff80))
        .prop(PropSchema::new("cpuid", ValueKind::Uint).range(0, 0xffff_ffff))
        .prop(PropSchema::new("syst-calib", ValueKind::Uint).range(0, 0xffff_ffff))
        // Inputs only. `irq0`…`irq239` are the NVIC's external lines.
        .port_bank("irq", PortDir::In, IRQ_LINES)
        .port("nmi", PortDir::In)
        .port("reset", PortDir::In)
}

/// One external interrupt input, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, so this keeps a [`FanIn`] and wire-ORs the
/// sources — which is what a shared interrupt line does in hardware.
///
/// The pin keeps a handle on the core's *input latches*, not on the core: the
/// core owns the pin, and a pin that owned the core back would be a reference
/// cycle §4.3 forbids and that the machine could never drop. It also could not
/// be built at all from [`Device::sink`], which has only `&self`.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    irq: u16,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect external interrupt `irq` to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, irq: u16, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            irq,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> InterruptPin {
        self.resolve = resolve;
        self
    }

    /// Which external interrupt this is.
    #[must_use]
    pub fn irq(&self) -> u16 {
        self.irq
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
        self.lines.set(self.irq, asserted);
    }
}

/// The non-maskable interrupt input.
///
/// Separate from [`InterruptPin`] because NMI is not one of the NVIC's
/// external lines: it has no enable bit, no priority register, and no number
/// a `NVIC_ISER` write can reach.
#[derive(Debug)]
pub struct NmiPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl NmiPin {
    /// Connect the NMI input to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> NmiPin {
        NmiPin {
            lines,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> NmiPin {
        self.resolve = resolve;
        self
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for NmiPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.resolve(self.resolve).is_high();
        self.lines.set_nmi(asserted);
    }
}

/// The core's reset input.
///
/// Not an interrupt: no vector, no priority, no pending bit. Asserting the
/// line latches a request; the reset sequence itself runs on the next
/// [`ArmV7m::step`], because a reset is a signal rather than a method call.
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect the reset input to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
        ResetPin {
            lines,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> ResetPin {
        self.resolve = resolve;
        self
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
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.set_reset(true);
        }
    }
}
