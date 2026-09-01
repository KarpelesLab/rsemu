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
//! What there is **not**: the FPv4-SP / FPv5 floating-point unit. `CPACR`
//! exists and reads zero from reset, so a `VMOV` raises a UsageFault with
//! `UFSR.NOCP` exactly as it would on a Cortex-M4 without the option — which
//! is honest, and is what lets firmware detect the absence. Lazy FP stacking,
//! `FPCCR` and the extended exception frame are absent with it. See
//! "Unimplemented" below.
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
//! | `dsp` (private) | the SIMD and extending-move semantics |
//! | `exec` (private) | the interpreter and its timing model |
//!
//! # Unimplemented, stated plainly
//!
//! - **FPv4-SP / FPv5.** No `S0`–`S31`, no `FPSCR`, no lazy stacking. A
//!   coprocessor 10/11 access is a `NOCP` UsageFault, which is the correct
//!   behaviour for a part without the option but is *not* an implementation
//!   of the option.
//! - **The debug architecture.** No DWT, ITM, FPB, TPIU or halting debug.
//!   Their registers read as zero. `BKPT` is a HardFault with
//!   `HFSR.DEBUGEVT`, which is what a part with no debugger attached does.
//! - **Imprecise bus faults.** There is no write buffer, so every data abort
//!   is precise and `BFSR.IMPRECISERR` is never set.
//! - **Cache and TCM behaviour** on the M7. The core is not the place for it;
//!   a SoC's memory system is.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403; the *Cortex-M4* and
//! *Cortex-M7 Technical Reference Manuals* for the implementation-defined
//! values (`CPUID`, the number of priority bits, MPU region count) and the
//! instruction timings. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

pub mod isa;
pub mod sys;

mod dsp;
mod exec;

#[cfg(test)]
mod tests;

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
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::validate::port_index;

use exec::{Exec, State};
use sys::{Exception, Sys};

pub use sys::{CPUID_CORTEX_M4, CPUID_CORTEX_M7};

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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Extensions {
    /// The DSP (E) extension: the SIMD add/sub family, saturating
    /// arithmetic, the half-word and dual multiplies, `SEL`, `USAD8`, `PKH`
    /// and the packing extends. This is what the E in ARMv7E-M means.
    pub dsp: bool,
    /// The FPv4-SP / FPv5 floating-point unit.
    ///
    /// **Not implemented.** [`ArmV7m::new`] forces this to false rather than
    /// letting [`ArmV7m::config`] claim a unit that is not there, so a
    /// caller that sets it gets a part without an FPU and a `CPACR` that says
    /// so. The field exists because the extension is real and will have
    /// somewhere to go when it lands.
    pub fp: bool,
    /// A PMSAv7 memory protection unit with eight regions.
    pub mpu: bool,
}

impl Extensions {
    /// A Cortex-M3: ARMv7-M, no DSP, an MPU if the SoC bought one.
    pub const CORTEX_M3: Extensions = Extensions {
        dsp: false,
        fp: false,
        mpu: true,
    };
    /// A Cortex-M4 or M7: ARMv7E-M, DSP and MPU present, FPU absent.
    pub const CORTEX_M4: Extensions = Extensions {
        dsp: true,
        fp: false,
        mpu: true,
    };
}

/// How this particular part differs from the generic ARMv7E-M.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run a Cortex-M3 and a Cortex-M7 in the same process. The public surface
/// is a *named part* — [`Config::CORTEX_M4`] — rather than a hand-assembled
/// set of flags, which is the rule §6.1.1 sets for the whole ARM family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    pub priority_bits: u8,
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
    };

    /// A Cortex-M7: the same architecture, a different `CPUID`, and four
    /// priority bits. The caches and TCMs are the SoC's, not the core's.
    pub const CORTEX_M7: Config = Config {
        cpuid: CPUID_CORTEX_M7,
        priority_bits: 4,
        ..Config::CORTEX_M4
    };

    /// A Cortex-M3: ARMv7-M without the DSP extension, so every `SADD8`,
    /// `QADD`, `SMLAD` and `PKHBT` traps as UNDEFINED.
    pub const CORTEX_M3: Config = Config {
        ext: Extensions::CORTEX_M3,
        cpuid: 0x412f_c231,
        ..Config::CORTEX_M4
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
    /// `cfg.ext.fp` is forced false: there is no floating-point unit, and a
    /// configuration that claimed one would make [`ArmV7m::config`] lie.
    #[must_use]
    pub fn new(cfg: Config) -> ArmV7m {
        let cfg = Config {
            ext: Extensions {
                fp: false,
                ..cfg.ext
            },
            ..cfg
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
            r.require_enum("part", &["cortex-m3", "cortex-m4", "cortex-m7"])?
        } else {
            "cortex-m4"
        };
        let big_endian = r.or("big-endian", false)?;
        let priority_bits = r.or_range("priority-bits", 0u64, 0..=8)?;
        let dsp_override = r.or("dsp", true)?;
        let mpu_override = r.or("mpu", true)?;
        r.finish()?;
        let base = match part {
            "cortex-m3" => Config::CORTEX_M3,
            "cortex-m4" => Config::CORTEX_M4,
            "cortex-m7" => Config::CORTEX_M7,
            // `require_enum` has already rejected anything else.
            _ => Config::CORTEX_M4,
        };
        let mut cfg = Config {
            endian: if big_endian {
                Endian::Big
            } else {
                Endian::Little
            },
            ext: Extensions {
                dsp: base.ext.dsp && dsp_override,
                fp: false,
                mpu: base.ext.mpu && mpu_override,
            },
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

    /// Move the vector table. A machine whose boot ROM is not at zero sets
    /// this before the first step.
    pub fn set_vtor(&self, value: u32) {
        self.session.lock().state.sys.vtor = value & 0xffff_ff80;
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
        let Session { state, space } = &mut *session;
        if reset {
            state.reset_pending = true;
        }
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &self.cfg).step(&external, nmi)
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — a core cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible
    /// state.
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
    /// usually runs past its end, and [`run`](ArmV7m::run) reports the
    /// overshoot honestly. The scheduler treats an overrun as fatal — the
    /// overrun has already executed past an event that should have stopped
    /// it — so what the *device* path does instead is **carry** the
    /// overshoot: it is deducted from the next budget, which keeps the core's
    /// cycle count and its domain's tick count in step over any number of
    /// quanta while never letting a single one overrun.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
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
                insn: isa::decode(first, second),
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
    version: 1,
    summary: "ARMv7E-M (Cortex-M4/M7 class) CPU core with Thumb-2, DSP, NVIC and MPU",
    properties: &[
        PropertySpec {
            name: "part",
            kind: ValueKind::Str,
            required: false,
            summary: "which part to model: cortex-m3, cortex-m4 or cortex-m7",
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
            summary: "whether a PMSAv7 memory protection unit is present",
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
    w.write_u32(sys.cpuid)?;
    w.write_bool(sys.reset_requested)?;
    w.write_u32(sys.syst_csr)?;
    w.write_u32(sys.syst_rvr)?;
    w.write_u32(sys.syst_cvr)?;
    w.write_u32(sys.syst_calib)?;
    w.write_u32(sys.mpu_ctrl)?;
    w.write_u32(sys.mpu_rnr)?;
    w.write_u8(sys.mpu_regions)?;
    for value in sys.mpu_rbar {
        w.write_u32(value)?;
    }
    for value in sys.mpu_rasr {
        w.write_u32(value)?;
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
    sys.cpuid = r.read_u32()?;
    sys.reset_requested = r.read_bool()?;
    sys.syst_csr = r.read_u32()?;
    sys.syst_rvr = r.read_u32()?;
    sys.syst_cvr = r.read_u32()?;
    sys.syst_calib = r.read_u32()?;
    sys.mpu_ctrl = r.read_u32()?;
    sys.mpu_rnr = r.read_u32()?;
    sys.mpu_regions = r.read_u8()?;
    for value in &mut sys.mpu_rbar {
        *value = r.read_u32()?;
    }
    for value in &mut sys.mpu_rasr {
        *value = r.read_u32()?;
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
        .prop(PropSchema::new("part", ValueKind::Str).values(&[
            "cortex-m3",
            "cortex-m4",
            "cortex-m7",
        ]))
        .prop(PropSchema::new("big-endian", ValueKind::Bool))
        .prop(PropSchema::new("priority-bits", ValueKind::Uint).range(0, 8))
        .prop(PropSchema::new("dsp", ValueKind::Bool))
        .prop(PropSchema::new("mpu", ValueKind::Bool))
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
