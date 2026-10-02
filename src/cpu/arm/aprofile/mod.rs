//! The A-profile 32-bit core — an ARM926EJ-S-class ARMv5TE interpreter that
//! also executes the ARMv7-A **A32** and **T32** (Thumb-2) instruction sets
//! when its part says so.
//!
//! As an ARMv5TE part it covers what an ARM9 SoC needs and nothing it does
//! not: the full 32-bit ARM instruction set including `CLZ`, both forms of
//! `BLX`, `BKPT` and the E extensions (`QADD`, `SMLA<x><y>`, `LDRD`/`STRD`,
//! `PLD`); the full 16-bit Thumb set with interworking; all seven processor
//! modes with their banked registers; the complete exception model; and, when
//! a machine asks for one, a real [`cp15::Cp15`] with the VMSAv5 MMU behind
//! it — or, on a Cortex-A9, a [`cp15v7::Cp15v7`] with the VMSAv7
//! short-descriptor MMU and, with `cpu-arm-aprofile-vfp`, VFPv3-D32. The caches and the TCMs are **not** here — those are the SoC's, and
//! anything else it wants to add attaches through [`cp::Coprocessor`] and
//! [`cp::Mmu`].
//!
//! Configured as a later part ([`Config::arch`], `cpu = "cortex-a9"`), the
//! ARM-state instruction set grows to ARMv7-A's: the ARMv6 media instructions
//! (`REV`, the extends, `SEL`, the parallel add/subtract family with `GE`,
//! `SSAT`/`USAT`, `PKH`, the dual and most-significant-word multiplies,
//! `USAD8`, `UMAAL`), `CPS`, `SRS`/`RFE`, `SETEND` with big-endian data
//! accesses, the load/store exclusives with a local monitor ([`monitor`]),
//! the ARMv6K hints (`WFI`, `WFE`/`SEV` with an event register, `YIELD`), the
//! v6T2 additions (`MOVW`/`MOVT`, the bitfield instructions, `RBIT`, `MLS`,
//! `LDRHT` and friends), and ARMv7's barriers, `PLI`, `PLDW`, `DBG`,
//! interworking data-processing writes to the PC and true unaligned access.
//! The PSR gains `GE`, `E`, `A`, `J` and the IT state, with ARMv6's `MSR`
//! write rules and exception entry.
//!
//! Thumb state grows with it: the ARMv6 16-bit additions (`REV`, the
//! extends, `CPS`, `SETEND`), and on a part with Thumb-2 the whole 32-bit
//! T32 encoding space, `IT` blocks, `CBZ`/`CBNZ` and the 16-bit hints
//! ([`thumb2`]). A 32-bit Thumb instruction decodes to the same
//! [`isa::Insn`] an A32 word does and runs through the same executor — one
//! `ADD`, one `LDREX`, one `SMLAD` — with the Thumb-specific rules (the
//! `IT` state, flag-setting inside a block, `Align(PC, 4)`, the link values)
//! kept where the architecture puts them. Decode is gated on the part, so an
//! ARM926EJ-S is bit-for-bit what it was and a feature probe on it still
//! traps. `tests/conformance/ledgers/cpu-arm-aprofile-a32.txt` lists what is
//! deliberately absent (`SMC` executes as Undefined; Monitor mode, a global
//! exclusive monitor, ThumbEE and Advanced SIMD are elsewhere or later).
//!
//! # Using it from another crate
//!
//! This core is built to be consumed directly, without a `.machine` file.
//! There are two entry paths and they are equally supported.
//!
//! ## The direct path
//!
//! Construct, hand it an address space, and drive it:
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::arm::aprofile::{Arm, Config};
//!
//! // 64 KiB of RAM with `MOV r0, #0x42` at the reset vector.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! for (i, byte) in 0xe3a0_0042u32.to_le_bytes().iter().enumerate() {
//!     ram.write_u8(i as u64, *byte).unwrap();
//! }
//!
//! let space = AddressSpace::new("cpu", 32);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = Arm::new(Config::ARM926EJS);
//! cpu.attach_space(Arc::new(space));
//! cpu.step();                       // the reset sequence
//! cpu.step();                       // MOV r0, #0x42
//! assert_eq!(cpu.reg(0), 0x42);
//! ```
//!
//! The rest of that surface: [`Arm::run`] for a cycle budget, [`Arm::regs`]
//! and [`Arm::set_regs`] for the whole file, [`Arm::reg`]/[`Arm::set_reg`] and
//! [`Arm::cpsr`]/[`Arm::set_cpsr`] for one register, [`Arm::set_irq`] and
//! [`Arm::set_fiq`] to drive the interrupt inputs, [`Arm::attach_mmu`] and
//! [`Arm::attach_coprocessor`] for the system seam, and
//! [`Arm::disassemble_virtual`] — or [`Arm::disassemble_physical`], because the
//! caller says which kind of address it means — for a listing.
//!
//! ## The device path
//!
//! [`Arm`] is also a full [`Device`]: it has a [`CLASS`], it can be built from
//! [`Props`] by [`Arm::from_props`] or through the [`Registry`] once
//! [`register`] has run, it reports [`Device::is_runnable`], it takes
//! scheduler budgets through [`Device::run`], and it round-trips its state
//! through [`Device::save`] and [`Device::load`]. A machine that describes its
//! CPU in a `.machine` file gets the same core.
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the ARM decoder, producing one semantic value that both the interpreter and the disassembler read |
//! | [`isa_v6`] | the same decoder's ARMv6-and-later half, gated on the part's [`Extensions`] |
//! | [`media`] | the ARMv6 media arithmetic as pure functions |
//! | [`monitor`] | the local exclusive monitor, and the seam for a global one |
//! | [`thumb`] | the same for 16-bit Thumb, gated on the part as [`isa`] is |
//! | [`thumb2`] | the 32-bit T32 decoder, producing [`isa::Insn`] values, and the UAL printer for either Thumb width |
//! | [`disasm`] | the disassembler built on those two |
//! | [`cp`] | the coprocessor and MMU traits, the software TLB, `FlatMmu`, and a CP15 stub |
//! | [`cp15`] | the ARMv5 system control coprocessor and the VMSAv5 table walk |
//! | `exec` (private) | the interpreter, and the timing model it implements; `exec_v6.rs` is its ARMv6-and-later half |
//! | [`cp15v7`] | the Cortex-A9 system control coprocessor and the VMSAv7 short-descriptor walk |
//! | `vfp` (feature `cpu-arm-aprofile-vfp`) | the VFP register file, `FPSCR`/`FPEXC`/`FPSID`/`MVFR`, and the ARMv7 rules around [`crate::float`] |
//! | `vfpisa` (same feature) | the VFP A32 decoder and its disassembly |
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual*, ARM DDI 0100, ARMv5 revisions —
//! chapters A2 (programmer's model), A3 (ARM encodings), A4 (ARM
//! instructions), A5 (addressing modes), A6/A7 (Thumb), A10 (the DSP
//! extensions), B2 (the system control coprocessor) and B4 (fault status);
//! and ARM DDI 0406C (ARMv7-A and ARMv7-R) — A2–A5 and A8 for the A32
//! additions, A6 and A8 for T32, A2.5.2 for `ITSTATE`, B1 for the PSRs, the
//! exception model and the event register.
//! Cycle counts from ARM's own instruction-cycle timing summaries. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

pub mod arch;
pub mod cp;
pub mod cp15;
pub mod cp15v7;
pub mod disasm;
mod exec;
pub mod isa;
pub mod isa_v6;
pub mod media;
pub mod monitor;
pub mod thumb;
pub mod thumb2;
#[cfg(feature = "cpu-arm-aprofile-vfp")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-aprofile-vfp")))]
pub mod vfp;
#[cfg(feature = "cpu-arm-aprofile-vfp")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-aprofile-vfp")))]
pub mod vfpisa;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_t32;
#[cfg(test)]
mod tests_v7;
#[cfg(all(test, feature = "cpu-arm-aprofile-vfp"))]
mod vfptests;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    DebugTranslation, Device, DeviceClass, Export, ExportId, Initiator, PropertySpec, RealizeCtx,
    ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, AtomicU64, LockRank, Ordering};
use crate::core::value::Endian;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

pub use arch::{Arch, Extensions, Version, Vfp};
use cp::{Coprocessor, FlatMmu, Mmu, Tlb};
use cp15::Cp15;
use cp15v7::Cp15v7;
use exec::{Exec, State};
use monitor::{GlobalMonitor, LocalMonitor};

pub use exec::Exception;

/// The current program status register's bits (ARM ARM A2.5).
pub mod psr {
    /// Negative — bit 31.
    pub const N: u32 = 1 << 31;
    /// Zero — bit 30.
    pub const Z: u32 = 1 << 30;
    /// Carry, and "not borrow" on a subtract — bit 29.
    pub const C: u32 = 1 << 29;
    /// Signed overflow — bit 28.
    pub const V: u32 = 1 << 28;
    /// Sticky saturation, set by the DSP extensions and cleared only by an
    /// explicit `MSR` — bit 27.
    pub const Q: u32 = 1 << 27;
    /// Jazelle state — bit 24 (ARMv5TEJ and later; always clear here, since
    /// Jazelle is trivial).
    pub const J: u32 = 1 << 24;
    /// The four greater-than-or-equal flags the ARMv6 parallel add/subtract
    /// instructions set and `SEL` reads — bits 19..16.
    pub const GE: u32 = 0xf << 16;
    /// Big-endian data accesses (ARMv6, `SETEND`) — bit 9.
    pub const E: u32 = 1 << 9;
    /// Asynchronous (imprecise) abort mask (ARMv6) — bit 8.
    pub const A: u32 = 1 << 8;
    /// The Thumb-2 `IT` block state, split across bits 26..25 (`IT[1:0]`) and
    /// 15..10 (`IT[7:2]`) (DDI 0406C A2.5.2).
    pub const IT: u32 = 0x0600_fc00;
    /// IRQ disable — bit 7.
    pub const I: u32 = 1 << 7;
    /// FIQ disable — bit 6.
    pub const F: u32 = 1 << 6;
    /// Thumb state — bit 5.
    pub const T: u32 = 1 << 5;
    /// The five-bit mode field.
    pub const MODE: u32 = 0x1f;
}

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

/// One of the processor's seven modes (ARM ARM A2.2).
///
/// A `#[repr(transparent)]` newtype rather than an enum, because the field is
/// five bits and guest code can put any of the thirty-two values in it; an
/// enum would have to have a `Reserved(u8)` arm anyway and would lose the free
/// round trip through `CPSR` (CLAUDE.md, "Type conventions").
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mode(pub u8);

impl Mode {
    /// Unprivileged. The only mode with no `SPSR`, and the only one that
    /// cannot change mode.
    pub const USER: Mode = Mode(0b1_0000);
    /// Fast interrupt. Banks `R8`–`R14` rather than just `R13`–`R14`, which is
    /// the whole reason FIQ is fast.
    pub const FIQ: Mode = Mode(0b1_0001);
    /// Interrupt.
    pub const IRQ: Mode = Mode(0b1_0010);
    /// Supervisor: entered by reset and by `SWI`.
    pub const SUPERVISOR: Mode = Mode(0b1_0011);
    /// Abort: entered by a prefetch or data abort.
    pub const ABORT: Mode = Mode(0b1_0111);
    /// Undefined: entered by an undefined instruction.
    pub const UNDEFINED: Mode = Mode(0b1_1011);
    /// System: privileged, but shares the User register bank and has no
    /// `SPSR`.
    pub const SYSTEM: Mode = Mode(0b1_1111);

    /// Every mode, in the order a debugger should list them.
    pub const ALL: &'static [Mode] = &[
        Mode::USER,
        Mode::FIQ,
        Mode::IRQ,
        Mode::SUPERVISOR,
        Mode::ABORT,
        Mode::UNDEFINED,
        Mode::SYSTEM,
    ];

    /// Which `R13`/`R14` bank this mode uses.
    ///
    /// User and System share bank 0 — that is what System mode is *for*. A
    /// mode value the architecture does not define is UNPREDICTABLE; mapping
    /// it to the User bank keeps the core deterministic instead of panicking
    /// on guest data.
    #[must_use]
    pub const fn bank(self) -> usize {
        match self.0 & 0x1f {
            0b1_0001 => 1,
            0b1_0010 => 2,
            0b1_0011 => 3,
            0b1_0111 => 4,
            0b1_1011 => 5,
            _ => 0,
        }
    }

    /// Which `SPSR` this mode has, if any.
    ///
    /// `None` for User and System, which is why an exception return from
    /// either is UNPREDICTABLE.
    #[must_use]
    pub const fn spsr_index(self) -> Option<usize> {
        match self.bank() {
            0 => None,
            n => Some(n - 1),
        }
    }

    /// Whether the mode is privileged. Everything except User.
    #[must_use]
    pub const fn is_privileged(self) -> bool {
        self.0 & 0x1f != Mode::USER.0
    }

    /// Whether this is one of the seven modes the architecture defines.
    #[must_use]
    pub const fn is_defined(self) -> bool {
        matches!(
            self.0 & 0x1f,
            0b1_0000 | 0b1_0001 | 0b1_0010 | 0b1_0011 | 0b1_0111 | 0b1_1011 | 0b1_1111
        )
    }

    /// The short name a debugger prints.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 & 0x1f {
            0b1_0000 => "usr",
            0b1_0001 => "fiq",
            0b1_0010 => "irq",
            0b1_0011 => "svc",
            0b1_0111 => "abt",
            0b1_1011 => "und",
            0b1_1111 => "sys",
            _ => "???",
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// The architectural register file, banked registers and `SPSR`s included.
///
/// Public and `Copy` because a debugger, a tracer, a test and a snapshot all
/// want to read it out and put it back. The sixteen visible registers live in
/// [`Regs::r`]; the shadow banks hold whatever the *current* mode is not
/// using, and [`Regs::set_mode`] is the only thing that moves values between
/// them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Regs {
    /// The sixteen currently visible registers. `r[15]` is the PC.
    pub r: [u32; 16],
    /// The current program status register.
    pub cpsr: u32,
    /// `R13` and `R14` for the bank *not* currently loaded, indexed by
    /// [`Mode::bank`]. The entry for the current mode is stale by
    /// construction.
    pub banked_sp_lr: [[u32; 2]; 6],
    /// `R8`–`R12`, index 0 for every non-FIQ mode and index 1 for FIQ.
    pub banked_r8_r12: [[u32; 5]; 2],
    /// The five `SPSR`s, indexed by [`Mode::spsr_index`].
    pub spsr: [u32; 5],
}

impl Regs {
    /// The state a reset leaves behind: Supervisor mode, both interrupts
    /// masked, ARM state (ARM ARM A2.6.2).
    ///
    /// Every general register is zero. Real hardware leaves them undefined;
    /// zero is the reproducible choice, and determinism is a first-class mode
    /// (`ROADMAP.md` §0).
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            r: [0; 16],
            cpsr: Mode::SUPERVISOR.0 as u32 | psr::I | psr::F,
            banked_sp_lr: [[0; 2]; 6],
            banked_r8_r12: [[0; 5]; 2],
            spsr: [0; 5],
        }
    }

    /// The current mode.
    #[must_use]
    pub const fn mode(&self) -> Mode {
        Mode((self.cpsr & psr::MODE) as u8)
    }

    /// Whether the core is in Thumb state.
    #[must_use]
    pub const fn is_thumb(&self) -> bool {
        self.cpsr & psr::T != 0
    }

    /// The program counter.
    #[must_use]
    pub const fn pc(&self) -> u32 {
        self.r[15]
    }

    /// The `SPSR` of the current mode, or `None` in User and System.
    #[must_use]
    pub const fn spsr(&self) -> Option<u32> {
        match self.mode().spsr_index() {
            Some(i) => Some(self.spsr[i]),
            None => None,
        }
    }

    /// Write the current mode's `SPSR`. A no-op in User and System.
    pub const fn set_spsr(&mut self, value: u32) {
        if let Some(i) = self.mode().spsr_index() {
            self.spsr[i] = value;
        }
    }

    /// Change mode, moving the banked registers with it.
    ///
    /// This is the only place register banking happens, which is what makes it
    /// possible to reason about at all: everything else — exception entry,
    /// `MSR`, an exception return — funnels through here.
    pub const fn set_mode(&mut self, to: Mode) {
        let from = self.mode();
        if from.0 & 0x1f == to.0 & 0x1f {
            return;
        }
        let (old_bank, new_bank) = (from.bank(), to.bank());
        if old_bank != new_bank {
            self.banked_sp_lr[old_bank][0] = self.r[13];
            self.banked_sp_lr[old_bank][1] = self.r[14];
            self.r[13] = self.banked_sp_lr[new_bank][0];
            self.r[14] = self.banked_sp_lr[new_bank][1];
        }
        // FIQ banks five more registers than anyone else, so the swap only
        // happens when FIQ is on exactly one side of the transition.
        let old_fiq = old_bank == 1;
        let new_fiq = new_bank == 1;
        if old_fiq != new_fiq {
            let (out, into) = if old_fiq { (1, 0) } else { (0, 1) };
            let mut i = 0;
            while i < 5 {
                self.banked_r8_r12[out][i] = self.r[8 + i];
                self.r[8 + i] = self.banked_r8_r12[into][i];
                i += 1;
            }
        }
        self.cpsr = (self.cpsr & !psr::MODE) | ((to.0 as u32) & psr::MODE);
    }

    /// Write the whole `CPSR`, banking registers if the mode changed.
    ///
    /// This is what an exception return does, and what `MSR CPSR_c` does. A
    /// bare assignment to [`Regs::cpsr`] would change the mode field without
    /// moving the registers, which is the classic way to corrupt a stack
    /// pointer.
    ///
    /// `M[4]` — bit 4 of the mode field — is forced set. It is the bit that
    /// distinguishes the 26-bit modes from the 32-bit ones, and no ARMv5 part
    /// implements the 26-bit modes, so on real hardware it reads as one and
    /// cannot be cleared. An `SPSR` has no such constraint, which is why this
    /// is here and not in [`Regs::set_spsr`].
    pub const fn write_cpsr(&mut self, value: u32) {
        let value = value | 0x10;
        self.set_mode(Mode((value & psr::MODE) as u8));
        self.cpsr = value;
    }

    /// Read register `index` as some *other* mode would see it.
    ///
    /// What `LDM`/`STM` with the `S` bit needs, and what a debugger showing
    /// every bank needs.
    #[must_use]
    pub const fn reg_in_mode(&self, mode: Mode, index: u8) -> u32 {
        let index = (index & 0xf) as usize;
        let current = self.mode();
        if mode.0 & 0x1f == current.0 & 0x1f {
            return self.r[index];
        }
        match index {
            8..=12 => {
                let want_fiq = mode.bank() == 1;
                if want_fiq == (current.bank() == 1) {
                    self.r[index]
                } else {
                    self.banked_r8_r12[if want_fiq { 1 } else { 0 }][index - 8]
                }
            }
            13 | 14 => {
                if mode.bank() == current.bank() {
                    self.r[index]
                } else {
                    self.banked_sp_lr[mode.bank()][index - 13]
                }
            }
            _ => self.r[index],
        }
    }

    /// Write register `index` as some *other* mode would see it.
    pub const fn set_reg_in_mode(&mut self, mode: Mode, index: u8, value: u32) {
        let index = (index & 0xf) as usize;
        let current = self.mode();
        if mode.0 & 0x1f == current.0 & 0x1f {
            self.r[index] = value;
            return;
        }
        match index {
            8..=12 => {
                let want_fiq = mode.bank() == 1;
                if want_fiq == (current.bank() == 1) {
                    self.r[index] = value;
                } else {
                    self.banked_r8_r12[if want_fiq { 1 } else { 0 }][index - 8] = value;
                }
            }
            13 | 14 => {
                if mode.bank() == current.bank() {
                    self.r[index] = value;
                } else {
                    self.banked_sp_lr[mode.bank()][index - 13] = value;
                }
            }
            _ => self.r[index] = value,
        }
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
            "cpsr:{:08x} [{}{}{}{}{}{}{}{} {}]",
            self.cpsr,
            if self.cpsr & psr::N != 0 { 'N' } else { 'n' },
            if self.cpsr & psr::Z != 0 { 'Z' } else { 'z' },
            if self.cpsr & psr::C != 0 { 'C' } else { 'c' },
            if self.cpsr & psr::V != 0 { 'V' } else { 'v' },
            if self.cpsr & psr::Q != 0 { 'Q' } else { 'q' },
            if self.cpsr & psr::I != 0 { 'I' } else { 'i' },
            if self.cpsr & psr::F != 0 { 'F' } else { 'f' },
            if self.cpsr & psr::T != 0 { 'T' } else { 't' },
            self.mode()
        )
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// How this particular part differs from the generic ARMv5TE.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run two ARM machines with different vector placement and different
/// endianness at the same time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
    /// Byte order for data accesses.
    ///
    /// ARMv5 supports a big-endian data path. The core presents a
    /// byte-addressed memory and assembles multi-byte values in this order,
    /// swapping only where the region it is talking to declares a different
    /// one (`ROADMAP.md` §4.1's per-region endianness). For word-aligned
    /// accesses that is exactly ARMv5's BE-32; for sub-word accesses it is the
    /// byte-invariant reading, because the byte-lane muxing BE-32 describes is
    /// a property of the memory system rather than of the core.
    pub endian: Endian,
    /// Put the exception vectors at `0xffff0000` from reset.
    ///
    /// This is the `VINITHI` input. It sets the *reset value* of CP15's `V`
    /// bit when the core has a CP15, and is the whole answer when it does not;
    /// either way guest code that clears `V` moves the vectors back down, which
    /// is what hardware does.
    pub high_vectors: bool,
    /// Take a Data Abort on an unaligned access rather than rotating.
    ///
    /// CP15's `A` bit, and the strap that sets its reset value. Off by default,
    /// because that is ARMv5's reset state: with it clear an unaligned `LDR`
    /// rotates the loaded word (ARM ARM A2.8.2).
    pub alignment_faults: bool,
    /// What a store of `R15` writes: the instruction's address plus this.
    ///
    /// The architecture permits eight or twelve and leaves the choice to the
    /// implementation (ARM ARM A4.1.99). ARM926EJ-S stores plus eight;
    /// ARM7TDMI stores plus twelve, which is what the public conformance
    /// corpus was generated against.
    pub store_pc_offset: u8,
    /// Which system control coprocessor to build the core with.
    ///
    /// [`System::None`] is the default and is what every existing board asks
    /// for; [`System::Arm926EjS`] adds CP15 and the VMSAv5 MMU. See [`System`]
    /// for why an MMU is a construction property and not a connection.
    pub system: System,
    /// Which architecture version and extensions the part implements.
    ///
    /// Decode consults this: an instruction the configured part does not have
    /// takes an Undefined Instruction exception rather than executing, because
    /// that is how guests probe for features (`ROADMAP.md` §6.1.1).
    pub arch: Arch,
    /// This core's number within its cluster: `MPIDR.Aff0`, `0..=3` on a
    /// Cortex-A9 MPCore. Read only by an ARMv7 CP15.
    pub cpu_id: u8,
    /// The cluster's number: `MPIDR.Aff1`, the `CLUSTERID` input.
    pub cluster_id: u8,
    /// The MPCore private peripheral base — the `PERIPHBASE` input, which
    /// `CBAR` (`MRC p15, 4, Rd, c15, c0, 0`) reports so software can find the
    /// SCU, the GIC and the private timers (DDI 0407 1.5). Bits 12..0 are not
    /// part of the value.
    pub periphbase: u32,
}

impl Config {
    /// An ARM926EJ-S **macrocell**: little-endian, low vectors, no alignment
    /// faults, `STR pc` storing the instruction's address plus eight, and no
    /// system coprocessor.
    ///
    /// The part with its CP15 is [`ARM926EJS_MMU`](Config::ARM926EJS_MMU), and
    /// the split is deliberate rather than a naming accident. A real
    /// ARM926EJ-S has CP15, so this constant is the smaller claim — but it is
    /// the one two consumers need: the ARMv4T conformance corpus and the
    /// ARMv7E-M differential tester both use this core as an *oracle*, and an
    /// oracle that answers `MCR p15` instead of taking an Undefined Instruction
    /// exception is answering a different question. Anything modelling a board
    /// should say `ARM926EJS_MMU` or `cp15 = "arm926ejs"`.
    pub const ARM926EJS: Config = Config {
        requester: RequesterId::ANONYMOUS,
        endian: Endian::Little,
        high_vectors: false,
        alignment_faults: false,
        store_pc_offset: 8,
        // The macrocell without its CP15, which is what this core was before
        // one existed and what every board that does not ask still gets.
        system: System::None,
        arch: Arch::V5TE,
        cpu_id: 0,
        cluster_id: 0,
        periphbase: 0,
    };

    /// A whole ARM926EJ-S: the same core with its system control coprocessor,
    /// the VMSAv5 MMU and the part's identification registers.
    ///
    /// The MMU is still *off* — c1's `M` bit is clear out of reset — so a core
    /// built this way executes identically to [`ARM926EJS`](Config::ARM926EJS)
    /// until guest code turns it on.
    pub const ARM926EJS_MMU: Config = Config {
        system: System::Arm926EjS,
        ..Config::ARM926EJS
    };

    /// A Cortex-A9 MPCore core: ARMv7-A with its CP15 and the VMSAv7 MMU,
    /// CPU 0 of cluster 0, private peripherals at zero until the board says
    /// otherwise with [`periphbase`](Config::periphbase).
    ///
    /// The MMU is off out of reset, and so are the caches this model does not
    /// have.
    pub const CORTEX_A9: Config = Config {
        system: System::CortexA9,
        arch: Arch::CORTEX_A9,
        ..Config::ARM926EJS
    };

    /// An ARM7TDMI-shaped configuration: the same core, but storing `R15` as
    /// the instruction plus twelve.
    ///
    /// The instruction set is still ARMv5TE — this only changes the one
    /// implementation-defined value that the ARMv4T conformance corpus
    /// observes.
    pub const ARM7TDMI: Config = Config {
        store_pc_offset: 12,
        ..Config::ARM926EJS
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

    /// Same configuration, with the vectors at `0xffff0000`.
    #[must_use]
    pub const fn with_high_vectors(mut self, high: bool) -> Config {
        self.high_vectors = high;
        self
    }

    /// Same configuration, with alignment checking on or off.
    #[must_use]
    pub const fn with_alignment_faults(mut self, on: bool) -> Config {
        self.alignment_faults = on;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config::ARM926EJS
    }
}

// ---------------------------------------------------------------------------
// Interrupt inputs
// ---------------------------------------------------------------------------

/// The two interrupt inputs, kept outside the execution lock.
///
/// Atomics rather than fields under the mutex: a device asserting IRQ from
/// inside a write the CPU itself issued would otherwise re-enter the CPU's own
/// critical section, which is a deadlock under `native-std` and a panic under
/// `single`. Both ARM interrupt inputs are level-sensitive, so there is no
/// edge latch to keep either (`ROADMAP.md` §4.7).
#[derive(Debug, Default)]
pub(crate) struct Lines {
    irq: AtomicBool,
    fiq: AtomicBool,
    /// A reset asked for by the `reset` pin, latched until the next step folds
    /// it into the execution state.
    ///
    /// A latch rather than a direct write to `State::reset_pending`, because a
    /// wire is driven from inside whatever device changed it — often from
    /// inside an access this very core issued — and reaching for the session
    /// lock there would re-enter the core's own critical section
    /// (`ROADMAP.md` §4.7).
    reset: AtomicBool,
    /// An event another core's `SEV` sent, latched the same way and for the
    /// same reason as `reset`: whoever sends it must not need this core's
    /// execution lock.
    event: AtomicBool,
    /// Held in reset by the `hold` input: the core executes nothing until it
    /// is released, and the release is a reset.
    hold: AtomicBool,
}

impl Lines {
    fn snapshot(&self) -> (bool, bool) {
        (
            self.irq.load(Ordering::Acquire),
            self.fiq.load(Ordering::Acquire),
        )
    }

    fn restore(&self, (irq, fiq): (bool, bool)) {
        self.irq.store(irq, Ordering::Release);
        self.fiq.store(fiq, Ordering::Release);
    }

    /// Latch a reset request. Cleared by whoever folds it into the state.
    fn request_reset(&self) {
        self.reset.store(true, Ordering::Release);
    }

    /// Consume the latch, reporting whether one was owed.
    fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    /// Consume a pending event, reporting whether one was sent.
    fn take_event(&self) -> bool {
        self.event.swap(false, Ordering::AcqRel)
    }

    /// Hold or release the core. A release latches a reset, so the core
    /// starts from its reset address the way a part leaving reset does.
    fn set_hold(&self, held: bool) {
        let was = self.hold.swap(held, Ordering::AcqRel);
        if was && !held {
            self.request_reset();
        }
    }
}

/// [`ExportId::RESET_ADDRESS`]'s "no address": reset takes the architectural
/// vector.
pub const NO_RESET_ADDRESS: u64 = u64::MAX;

/// Which system control coprocessor the core is built with.
///
/// **This is how a `.machine` file asks for an MMU**, and it is deliberately a
/// construction property rather than a new connection mechanism. `CLAUDE.md`
/// and `ROADMAP.md` §4.4 both push back hard on inventing a fourth way for two
/// things to find each other — `Device::export` absorbed three of them — and
/// none of the existing three fits: CP15 is not a region, not a wire, and not a
/// handle one *device* publishes for another. It is part of the CPU. The ARM
/// ARM says so by specifying it in the architecture manual rather than leaving
/// it to a SoC's, and the RISC-V core here already agrees, carrying its own
/// Sv32/Sv39 MMU inside `cpu::riscv`.
///
/// So a machine file writes `cp15 = "arm926ejs"` on its `cpu.arm` object and
/// gets one, exactly as a 6502 writes `variant = "rp2a03"`. What is left behind
/// [`cp::Coprocessor`] and [`cp::Mmu`] is what those traits were always for: a
/// coprocessor the *SoC* adds, and an MMU that is not this architecture's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum System {
    /// None. Addresses are physical, the vectors are where the straps put them,
    /// and a coprocessor instruction is Undefined — an ARM926EJ-S macrocell
    /// with its CP15 left out, which is what this core was until now.
    None,
    /// An ARM926EJ-S CP15: the VMSAv5 MMU, the domain model, the fault
    /// registers, and the part's identification values.
    Arm926EjS,
    /// A Cortex-A9 CP15: the VMSAv7 short-descriptor MMU with `TTBR0`/`TTBR1`,
    /// execute-never and the access flag, the ARMv7 fault registers, `VBAR`,
    /// `CPACR`, the thread-ID registers, and the part's identification and
    /// MPCore registers. What `cpu = "cortex-a9"` gets unless the machine
    /// says otherwise. See [`cp15v7`].
    CortexA9,
}

impl System {
    /// The name a `.machine` file writes.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            System::None => "none",
            System::Arm926EjS => "arm926ejs",
            System::CortexA9 => "cortex-a9",
        }
    }

    /// Every name the `cp15` property accepts.
    pub const NAMES: &'static [&'static str] = &["none", "arm926ejs", "cortex-a9"];

    /// Parse one of [`NAMES`](System::NAMES).
    ///
    /// Not `FromStr`: this is infallible-with-`None` rather than an error
    /// type, because the caller that has one — `or_enum` — has already
    /// produced the good error message and only needs the value.
    #[must_use]
    pub fn parse(name: &str) -> Option<System> {
        match name {
            "none" => Some(System::None),
            "arm926ejs" => Some(System::Arm926EjS),
            "cortex-a9" => Some(System::CortexA9),
            _ => None,
        }
    }
}

/// Which interrupt input a pin drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// The maskable input, masked by `CPSR.I`.
    Irq,
    /// The fast input, masked by `CPSR.F`, which banks five extra registers.
    Fiq,
}

// ---------------------------------------------------------------------------
// The core
// ---------------------------------------------------------------------------

/// Everything the interpreter mutates, behind one lock.
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
    mmu: Arc<dyn Mmu>,
    coprocessors: [Option<Arc<dyn Coprocessor>>; 16],
    /// The software TLB. Derived state: never serialized, emptied by reset, by
    /// a snapshot restore, and by either generation counter moving.
    tlb: Tlb,
    /// The global exclusive monitor, when the machine has several cores.
    global_monitor: Option<Arc<dyn GlobalMonitor>>,
}

impl fmt::Debug for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Session")
            .field("state", &self.state)
            .field("space", &self.space.as_ref().map(|s| s.name()))
            .field("mmu", &self.mmu)
            .field(
                "coprocessors",
                &self.coprocessors.iter().filter(|c| c.is_some()).count(),
            )
            .field("tlb", &self.tlb.stats())
            .field("global_monitor", &self.global_monitor.is_some())
            .finish()
    }
}

/// An A-profile ARM core: ARMv5TE, or the A32 half of ARMv7-A, as its
/// [`Config::arch`] says.
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
/// device asserting IRQ from inside a write the CPU itself issued cannot
/// re-enter the CPU's own critical section.
#[derive(Debug)]
pub struct Arm {
    cfg: Config,
    /// The system control coprocessor, when [`Config::system`] asked for one.
    ///
    /// Held here as well as inside the session because it is *wiring*, not
    /// guest state: it must survive a reset, it is answerable before the core
    /// has ever run, and a monitor or a test wants the concrete type rather
    /// than a `dyn Mmu`.
    cp15: Option<SystemCp>,
    lines: Arc<Lines>,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    ///
    /// Separate from [`Config::requester`] because a machine file names no
    /// requester: the machine layer allocates one per initiator and hands it
    /// over in [`Instance::bind`](crate::machine::Instance::bind), which is
    /// after `new` (`ROADMAP.md` §4.4).
    requester: AtomicU32,
    session: sync::Mutex<Session>,
    /// Where a reset starts the core, when the board says ([`ExportId::RESET_ADDRESS`]):
    /// a SoC whose secondary cores leave reset at an address software wrote
    /// into a boot-address register rather than at the vector. Published as a
    /// cell, so the board's register can write it without reaching this
    /// core's lock; [`NO_RESET_ADDRESS`] means the architectural vector.
    reset_address: Arc<AtomicU64>,
    /// Whether the core starts held (`held = true`), and goes back to held on
    /// a warm reset of the machine.
    held_at_reset: bool,
    /// Set by [`step`](Arm::step) when the core did nothing and will go on
    /// doing nothing until an input changes, so [`run_budget`](Arm::run_budget)
    /// can hand the rest of its budget back at once instead of a cycle at a
    /// time.
    idle: AtomicBool,
    /// The strong end of every pin this core has handed to a wire.
    ///
    /// A net holds its sinks weakly — the machine owns devices and a wire
    /// merely refers to them (§4.3) — so a pin nothing else kept alive would
    /// die on the way out of [`Device::sink`] and the wire would silently
    /// deliver to nothing.
    pins: sync::Mutex<Pins>,
}

/// The system control coprocessor a core was built with, by architecture.
///
/// An enum rather than a trait object because the two are genuinely different
/// register files with different public surfaces — a monitor or a test wants
/// `ttbr1()` on one and `fcse_pid()` on the other — and because the snapshot
/// code must know which one it is writing.
#[derive(Debug, Clone)]
enum SystemCp {
    /// ARMv5: [`Cp15`].
    V5(Arc<Cp15>),
    /// ARMv7: [`Cp15v7`].
    V7(Arc<Cp15v7>),
}

/// The pins [`Device::sink`] has built, kept alive by the core that owns them.
#[derive(Debug, Default)]
struct Pins {
    irq: Option<Arc<InterruptPin>>,
    fiq: Option<Arc<InterruptPin>>,
    reset: Option<Arc<ResetPin>>,
    hold: Option<Arc<HoldPin>>,
}

impl Arm {
    /// A core in its power-on state, with no address space and no
    /// coprocessors.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Arm::attach_space) and [`Device::realize`]. The
    /// first [`step`](Arm::step) runs the reset sequence, which is what puts
    /// the PC on the reset vector.
    ///
    /// Infallible, so it cannot refuse a part this build cannot model: a
    /// [`Config`] whose `arch.ext.vfp` is set, in a build without the
    /// `cpu-arm-aprofile-vfp` feature, gets a core with **no** VFP — every
    /// coprocessor 10/11 instruction UNDEFINED, and [`config`](Arm::config)
    /// saying so rather than claiming a unit that is not there.
    /// [`try_new`](Arm::try_new) is the constructor that refuses instead, and
    /// is what a machine file reaches (`ROADMAP.md` §6.1.1).
    #[must_use]
    pub fn new(cfg: Config) -> Arm {
        #[cfg(not(feature = "cpu-arm-aprofile-vfp"))]
        let cfg = {
            let mut cfg = cfg;
            cfg.arch.ext.vfp = None;
            cfg
        };
        let lines = Arc::new(Lines::default());
        let cp15 = match cfg.system {
            System::None => None,
            System::Arm926EjS => Some(SystemCp::V5(Arc::new(Cp15::arm926ejs(&cfg)))),
            System::CortexA9 => Some(SystemCp::V7(Arc::new(
                Cp15v7::cortex_a9(&cfg).with_lines(Arc::clone(&lines)),
            ))),
        };
        // `Option<Arc<_>>` is not `Copy`, so the array cannot be written
        // `[None; 16]`.
        let mut coprocessors: [Option<Arc<dyn Coprocessor>>; 16] = [const { None }; 16];
        let mmu: Arc<dyn Mmu> = match &cp15 {
            Some(SystemCp::V5(cp)) => {
                coprocessors[15] = Some(Arc::clone(cp) as Arc<dyn Coprocessor>);
                Arc::clone(cp) as Arc<dyn Mmu>
            }
            Some(SystemCp::V7(cp)) => {
                coprocessors[15] = Some(Arc::clone(cp) as Arc<dyn Coprocessor>);
                Arc::clone(cp) as Arc<dyn Mmu>
            }
            // With no CP15 the flat map carries the board's straps, because the
            // installed MMU is the one authority on them (see `cp::FlatMmu`).
            None => Arc::new(FlatMmu {
                high_vectors: cfg.high_vectors,
                alignment_faults: cfg.alignment_faults,
            }),
        };
        Arm {
            cfg,
            cp15,
            lines,
            requester: AtomicU32::new(cfg.requester.0),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                    mmu,
                    coprocessors,
                    tlb: Tlb::new(),
                    global_monitor: None,
                },
            ),
            reset_address: Arc::new(AtomicU64::new(NO_RESET_ADDRESS)),
            held_at_reset: false,
            idle: AtomicBool::new(false),
            pins: sync::Mutex::new(Pins::default()),
        }
    }

    /// Start the core held in reset, as a secondary core of a SoC whose boot
    /// ROM never runs it: it executes nothing until its `hold` input falls.
    #[must_use]
    pub fn held(mut self) -> Arm {
        self.held_at_reset = true;
        self.lines.hold.store(true, Ordering::Release);
        self
    }

    /// Whether the core is held in reset.
    #[must_use]
    pub fn is_held(&self) -> bool {
        self.lines.hold.load(Ordering::Acquire)
    }

    /// The cell a reset reads its start address from.
    #[must_use]
    pub fn reset_address(&self) -> &Arc<AtomicU64> {
        &self.reset_address
    }

    /// A core in its power-on state, refusing a part whose extensions this
    /// build did not compile in.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] naming the missing Cargo feature when the part has
    /// VFP and `cpu-arm-aprofile-vfp` is off, and when it claims Advanced
    /// SIMD, which no build of this core implements. A preset that silently
    /// lost its FPU would boot a hard-float guest into an Undefined
    /// Instruction exception on its first `VMSR` (`ROADMAP.md` §6.1.1).
    pub fn try_new(cfg: Config) -> Result<Arm> {
        if cfg.arch.ext.vfp.is_some() && !cfg!(feature = "cpu-arm-aprofile-vfp") {
            return Err(Error::Property(
                "this ARM part has a VFP floating-point unit, which this build does not \
                 include: enable the `cpu-arm-aprofile-vfp` Cargo feature"
                    .into(),
            ));
        }
        if cfg.arch.ext.neon {
            return Err(Error::Property(
                "this ARM part claims Advanced SIMD (NEON), which the A-profile core \
                 does not implement"
                    .into(),
            ));
        }
        Ok(Arm::new(cfg))
    }

    /// The VFP register file and control registers, or `None` when the part
    /// has no VFP.
    #[cfg(feature = "cpu-arm-aprofile-vfp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-aprofile-vfp")))]
    #[must_use]
    pub fn vfp(&self) -> Option<vfp::VfpRegs> {
        self.cfg.arch.ext.vfp?;
        Some(self.session.lock().state.vfp)
    }

    /// Replace the VFP register file and control registers. Ignored on a
    /// part with no VFP, which has nowhere to put them.
    #[cfg(feature = "cpu-arm-aprofile-vfp")]
    #[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-aprofile-vfp")))]
    pub fn set_vfp(&self, regs: vfp::VfpRegs) {
        if self.cfg.arch.ext.vfp.is_some() {
            self.session.lock().state.vfp = regs;
        }
    }

    /// This core's system control coprocessor, if it was built with one.
    ///
    /// The concrete type rather than a trait object: a monitor listing CP15,
    /// a test asserting a fault status, and a SoC that wants to seed the
    /// translation table base all want to read named registers.
    #[must_use]
    ///
    /// `None` on a core built with an ARMv7 CP15, which answers
    /// [`cp15v7`](Arm::cp15v7) instead.
    pub fn cp15(&self) -> Option<&Arc<Cp15>> {
        match &self.cp15 {
            Some(SystemCp::V5(cp)) => Some(cp),
            _ => None,
        }
    }

    /// This core's ARMv7 system control coprocessor, if it was built with
    /// one ([`System::CortexA9`]).
    #[must_use]
    pub fn cp15v7(&self) -> Option<&Arc<Cp15v7>> {
        match &self.cp15 {
            Some(SystemCp::V7(cp)) => Some(cp),
            _ => None,
        }
    }

    /// Put this core and `other` in one inner-shareable domain, so a
    /// broadcast TLB operation on either invalidates both TLBs.
    ///
    /// An MPCore machine calls this for every pair of its cores. Without it an
    /// inner-shareable `TLBIALLIS` reaches only the core that executed it —
    /// and a Cortex-A9's `ID_MMFR3` tells the OS that it reaches all of them,
    /// so the OS will not send the IPI that would have covered the gap. A no-op
    /// unless both cores have an ARMv7 CP15. See [`cp15v7`] "Several cores".
    pub fn join_cluster(&self, other: &Arm) {
        if let (Some(mine), Some(theirs)) = (self.cp15v7(), other.cp15v7()) {
            mine.join(theirs);
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or value, or a property nothing here
    /// accepts was given — a typo'd property that was silently ignored is an
    /// afternoon lost.
    pub fn from_props(props: &Props) -> Result<Arm> {
        let mut r = props.reader();
        let big_endian = r.or("big-endian", false)?;
        let high_vectors = r.or("high-vectors", false)?;
        let alignment_faults = r.or("alignment-faults", false)?;
        let store_pc_offset = r.or_range("store-pc-offset", 8u64, 8..=12)?;
        let part = r.or_enum("cpu", "arm926ejs", PARTS)?;
        // A Cortex-A9 without its CP15 is not a part anyone built, so naming
        // the part brings the coprocessor; `cp15 = "none"` still removes it.
        let default_system = if part == "cortex-a9" {
            "cortex-a9"
        } else {
            "none"
        };
        let system = r.or_enum("cp15", default_system, System::NAMES)?;
        let cpu_id = r.or_range("cpu-id", 0u64, 0..=3)?;
        let cluster_id = r.or_range("cluster-id", 0u64, 0..=15)?;
        let periphbase = r.or_addr("periphbase", 0)?;
        let held = r.or("held", false)?;
        // Accepted and ignored: there is one engine until phase 5, and a
        // machine file that names it should not have to be edited when the
        // second one lands.
        let _engine = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;
        if store_pc_offset != 8 && store_pc_offset != 12 {
            return Err(Error::Property(
                "store-pc-offset must be 8 (ARM926EJ-S) or 12 (ARM7TDMI)".into(),
            ));
        }
        let periphbase = u32::try_from(periphbase)
            .map_err(|_| Error::Property("periphbase must fit in 32 bits".into()))?;
        // `or_enum` already rejected anything not in `NAMES`.
        let system = System::parse(system).unwrap_or(System::None);
        // Each CP15 describes one architecture's MMU and one part's identity;
        // an ARMv5 core reporting a Cortex-A9's ID registers, or the reverse,
        // is a machine file mistake and not a configuration.
        match (system, part) {
            (System::CortexA9, "arm926ejs") => {
                return Err(Error::Property(
                    "cp15 = \"cortex-a9\" needs cpu = \"cortex-a9\"".into(),
                ));
            }
            (System::Arm926EjS, "cortex-a9") => {
                return Err(Error::Property(
                    "cp15 = \"arm926ejs\" needs cpu = \"arm926ejs\"".into(),
                ));
            }
            _ => {}
        }
        let core = Arm::try_new(Config {
            requester: RequesterId::ANONYMOUS,
            endian: if big_endian {
                Endian::Big
            } else {
                Endian::Little
            },
            high_vectors,
            alignment_faults,
            store_pc_offset: store_pc_offset as u8,
            system,
            arch: part_arch(part),
            cpu_id: cpu_id as u8,
            cluster_id: cluster_id as u8,
            periphbase,
        })?;
        Ok(if held { core.held() } else { core })
    }

    /// This core's configuration, with the bind-time requester folded in.
    #[must_use]
    pub fn config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Give the core the identity its accesses travel under.
    ///
    /// The machine layer calls this from `bind`; a crate driving the core
    /// directly usually sets [`Config::requester`] at construction instead.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Give the core the address space it executes from.
    ///
    /// Separate from construction because the space is built by the machine
    /// assembly layer; a crate driving the core directly calls this itself.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Install the object that translates addresses and owns the control bits
    /// the core reads.
    ///
    /// Rarely needed now: a core built with [`System::Arm926EjS`] already has
    /// a [`Cp15`] installed here and at coprocessor 15. This replaces it, for a
    /// SoC whose memory management is genuinely not the architecture's — and
    /// such a SoC usually passes the same object to
    /// [`attach_coprocessor`](Arm::attach_coprocessor) as well, because one
    /// type implementing both traits is what a real system coprocessor is.
    ///
    /// The MMU installed here becomes the **only** authority on the vector base
    /// and the alignment check, so an implementation that means to honour the
    /// board's `VINITHI` strap has to be told about it; the default
    /// [`FlatMmu`] is constructed from [`Config`] for exactly that reason.
    pub fn attach_mmu(&self, mmu: Arc<dyn Mmu>) {
        let mut session = self.session.lock();
        session.mmu = mmu;
        // Whatever the old one had decided is not this one's answer.
        session.tlb.flush();
    }

    /// How many TLB lookups hit and how many missed since the last flush.
    ///
    /// Derived state and therefore not in a snapshot; this is for `rsemu`'s
    /// statistics and for a benchmark that wants to prove the TLB is working.
    #[must_use]
    pub fn tlb_stats(&self) -> (u64, u64) {
        self.session.lock().tlb.stats()
    }

    /// Install a coprocessor at number `cp` (`0..=15`).
    ///
    /// Numbers above fifteen are impossible in the encoding, so this takes the
    /// low four bits and asks no questions.
    pub fn attach_coprocessor(&self, cp: u8, coprocessor: Arc<dyn Coprocessor>) {
        self.session.lock().coprocessors[(cp & 0xf) as usize] = Some(coprocessor);
    }

    /// Share a global exclusive monitor with the other cores of a machine.
    ///
    /// Only meaningful for a part with `LDREX`/`STREX` and a machine with
    /// more than one core; see [`monitor`] for what it is consulted about.
    /// Without one, a store-exclusive answers to this core's local monitor
    /// alone, which is exactly right for a single core.
    pub fn attach_global_monitor(&self, monitor: Arc<dyn GlobalMonitor>) {
        self.session.lock().global_monitor = Some(monitor);
    }

    /// Deliver an event, as another core's `SEV` does (ARMv6K).
    ///
    /// Sets the event register at the next step, which wakes a `WFE` in
    /// progress or lets the next one fall straight through. Lock-free, so a
    /// core executing `SEV` can call it on its siblings from inside its own
    /// step.
    pub fn send_event(&self) {
        self.lines.event.store(true, Ordering::Release);
    }

    /// Whether the event register is set.
    #[must_use]
    pub fn event_pending(&self) -> bool {
        self.lines.event.load(Ordering::Acquire) || self.session.lock().state.event
    }

    /// The granule the local exclusive monitor has marked, if any.
    #[must_use]
    pub fn exclusive_tag(&self) -> Option<u32> {
        self.session.lock().state.monitor.tag()
    }

    /// Remove the coprocessor at number `cp`, so its instructions become
    /// Undefined again.
    pub fn detach_coprocessor(&self, cp: u8) {
        self.session.lock().coprocessors[(cp & 0xf) as usize] = None;
    }

    /// The whole register file, banked registers included.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().state.regs
    }

    /// Overwrite the whole register file — a debugger, a test vector, a
    /// snapshot.
    pub fn set_regs(&self, regs: Regs) {
        self.session.lock().state.regs = regs;
    }

    /// Read one of the sixteen currently visible registers.
    #[must_use]
    pub fn reg(&self, index: u8) -> u32 {
        self.session.lock().state.regs.r[(index & 0xf) as usize]
    }

    /// Write one of the sixteen currently visible registers.
    ///
    /// Writing `R15` sets the PC directly and does not interwork; use
    /// [`set_cpsr`](Arm::set_cpsr) to change instruction set.
    pub fn set_reg(&self, index: u8, value: u32) {
        self.session.lock().state.regs.r[(index & 0xf) as usize] = value;
    }

    /// The program counter.
    #[must_use]
    pub fn pc(&self) -> u32 {
        self.session.lock().state.regs.r[15]
    }

    /// Set the program counter.
    pub fn set_pc(&self, value: u32) {
        self.session.lock().state.regs.r[15] = value;
    }

    /// The current program status register.
    #[must_use]
    pub fn cpsr(&self) -> u32 {
        self.session.lock().state.regs.cpsr
    }

    /// Write the whole `CPSR`, banking registers if the mode changes.
    pub fn set_cpsr(&self, value: u32) {
        self.session.lock().state.regs.write_cpsr(value);
    }

    /// The current mode.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.session.lock().state.regs.mode()
    }

    /// Whether the core is in Thumb state.
    #[must_use]
    pub fn is_thumb(&self) -> bool {
        self.session.lock().state.regs.is_thumb()
    }

    /// Bus cycles executed since power-on. See `exec`'s timing model.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether the core is waiting for an interrupt.
    ///
    /// Set by a coprocessor returning [`cp::CpEffect::HALT`], which is how
    /// CP15's "wait for interrupt" register is implemented, and on ARMv6K
    /// and later by `WFI` and by a `WFE` with no event pending (which an
    /// event, from [`Arm::send_event`] or `SEV`, also ends). A halted core
    /// still consumes budget — it is idling, not stopped — and wakes on either
    /// interrupt input whether or not that interrupt is masked.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// How many accesses the address space refused, and where the last one
    /// was.
    ///
    /// A refused access is an external abort and *does* raise an exception on
    /// ARM, unlike the 6502's open bus — but a machine whose memory map has a
    /// hole will show this climbing long before it works out why its guest is
    /// in the abort handler.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// The comment field of the most recent `SWI`.
    ///
    /// The architecture does not give hardware this value — a handler reads
    /// the instruction back out of memory — but a host that implements
    /// semihosting wants it without doing that.
    #[must_use]
    pub fn last_swi(&self) -> u32 {
        self.session.lock().state.last_swi
    }

    /// The comment field of the most recent `BKPT`.
    ///
    /// With no debug hardware attached, `BKPT` takes a Prefetch Abort
    /// (ARM ARM A4.1.10); this is how a host debugger sees which breakpoint it
    /// was.
    #[must_use]
    pub fn last_bkpt(&self) -> u16 {
        self.session.lock().state.last_bkpt
    }

    /// Drive the IRQ input. Level-sensitive: taken while asserted and `I` is
    /// clear.
    ///
    /// `asserted` is the logical level, not the pin's: a real `nIRQ` is
    /// active-low, and inverting it belongs to whatever models the wire.
    pub fn set_irq(&self, asserted: bool) {
        self.lines.irq.store(asserted, Ordering::Release);
    }

    /// Whether IRQ is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.lines.irq.load(Ordering::Acquire)
    }

    /// Drive the FIQ input. Level-sensitive, like IRQ.
    pub fn set_fiq(&self, asserted: bool) {
        self.lines.fiq.store(asserted, Ordering::Release);
    }

    /// Whether FIQ is currently asserted.
    #[must_use]
    pub fn fiq_asserted(&self) -> bool {
        self.lines.fiq.load(Ordering::Acquire)
    }

    /// Request a reset sequence without changing any register.
    ///
    /// It runs on the next [`step`](Arm::step), because a reset is a signal
    /// rather than a method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, one exception entry, or one instruction.
    ///
    /// Returns the cycles charged: zero if there is no address space, which
    /// the caller must treat as "stop", not "retry". A core waiting for an
    /// interrupt returns one cycle per call and keeps waiting.
    pub fn step(&self) -> u64 {
        if self.lines.hold.load(Ordering::Acquire) {
            // Held in reset: no fetch, no state change, and the latch a release
            // will set is still to come. A cycle passes all the same, so a
            // caller stepping by hand still sees time move.
            self.idle.store(true, Ordering::Relaxed);
            self.session.lock().state.cycles += 1;
            return 1;
        }
        let (irq, fiq) = self.lines.snapshot();
        let reset = self.lines.take_reset_request();
        let event = self.lines.take_event();
        if reset {
            // The reset input resets the whole processor, and CP15 is part
            // of it (DDI 0406C B1.9.10, "Reset"): the MMU comes back off, the
            // vector base back to its strap. Without this a guest that pulls
            // its own reset with the MMU on -- Linux's restart path does --
            // fetches the reset vector through its own page tables.
            match &self.cp15 {
                Some(SystemCp::V5(cp15)) => cp15.reset(),
                Some(SystemCp::V7(cp15)) => cp15.reset(),
                None => {}
            }
        }
        let cfg = self.config();
        let mut session = self.session.lock();
        let Session {
            state,
            space,
            mmu,
            coprocessors,
            tlb,
            global_monitor,
        } = &mut *session;
        // The `reset` pin latches outside the lock; this is where the latch
        // becomes execution state, and it must happen before the step so an
        // assertion is honoured by the very next instruction boundary. An
        // event from another core's `SEV` is folded in the same way.
        state.reset_pending |= reset;
        if reset {
            tlb.flush();
        }
        state.event |= event;
        let Some(space) = space.clone() else {
            return 0;
        };
        let mmu = Arc::clone(mmu);
        let global = global_monitor.clone();
        let resetting = state.reset_pending;
        let used = Exec::new(
            state,
            &space,
            mmu.as_ref(),
            tlb,
            coprocessors,
            &cfg,
            global.as_deref(),
        )
        .step(irq, fiq);
        if resetting {
            // The reset sequence put the PC on the vector; a board that says
            // otherwise moves it, before the first fetch.
            let at = self.reset_address.load(Ordering::Acquire);
            if at != NO_RESET_ADDRESS {
                state.regs.r[15] = at as u32;
            }
        }
        // Still halted after a step means nothing woke it: no interrupt line,
        // no event. Nothing will until an input changes.
        self.idle.store(state.halted, Ordering::Relaxed);
        used
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — an ARM cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    ///
    /// [`run_budget`](Arm::run_budget) is the same loop with the overshoot
    /// carried forward instead, which is what the scheduler needs.
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

    /// Execute for at most `ticks`, carrying any overshoot into the next call.
    ///
    /// The scheduler hands out a budget and refuses a report larger than it, so
    /// the instruction that ran past the end is paid for by the *following*
    /// budget through `State::debt` — which keeps the core's cycle count exact
    /// while never letting its clock domain run ahead of the timeline.
    ///
    /// A halted core, or one with no address space, consumes only the debt it
    /// owed plus whatever it managed.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            // The last instruction was longer than this whole budget: charge
            // the budget against the debt and execute nothing.
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                // No address space. Stop — retrying would spin.
                break;
            }
            used += n;
            if self.idle.load(Ordering::Relaxed) && used < allowance {
                // Halted (WFI, WFE, or held in reset) with nothing to wake it.
                // Stepping on would charge one cycle per call until the budget
                // ran out and change nothing else: an input can only change
                // when the scheduler runs whatever drives it, which is after
                // this budget. So the rest passes at once, and the cycle count
                // still says it passed.
                let rest = allowance - used;
                self.session.lock().state.cycles += rest;
                used = allowance;
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

    /// Cycles owed to the next budget — see [`run_budget`](Arm::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Where a virtual address is mapped, as a debugger asks it.
    ///
    /// `None` is "the tables map nothing there". With the MMU off — and on a
    /// core built with `cp15 = "none"`, where it always is — this is the
    /// identity and cannot fail.
    ///
    /// Side-effect free by construction, which is the whole point of it being a
    /// separate call rather than a flag on the execution path: it does not
    /// consult or fill the core's TLB, it does not charge a cycle, it does not
    /// latch `FSR` or `FAR`, and the descriptor reads it makes carry
    /// [`MemAttrs::DEBUG`] so a page table living under an MMIO region is not
    /// disturbed by being looked at. VMSAv5 has no accessed or dirty bit to set
    /// — and could not set one anyway, because
    /// [`PhysMem`](cp::PhysMem) is read-only.
    ///
    /// It also asks a *permission-free* question: which physical address this
    /// virtual one names, not whether some access to it would be allowed. See
    /// [`Device::debug_translate`].
    #[must_use]
    pub fn translate_debug(&self, va: u32) -> Option<u32> {
        let cfg = self.config();
        let session = self.session.lock();
        let space = session.space.as_ref()?;
        exec::debug_translate(space, session.mmu.as_ref(), &cfg, va)
    }

    /// Disassemble `count` instructions starting at the **virtual** address
    /// `addr`, reading guest memory with debug attributes.
    ///
    /// This is the one to hand [`pc`](Arm::pc), and the reason the two forms
    /// have different names: an ARM program counter is a virtual address, and
    /// with the MMU on a listing that skipped translation would decode whatever
    /// happens to sit at the same number on the bus. Every byte is translated
    /// through [`translate_debug`](Arm::translate_debug), so a listing that runs
    /// off the end of a mapped page carries on into
    /// [`Listed::Unreadable`](disasm::Listed::Unreadable) with
    /// [`Missing::Untranslated`](disasm::Missing::Untranslated) rather than
    /// stopping or inventing bytes — the count is always what was asked for.
    ///
    /// `thumb` picks the instruction set; pass [`Arm::is_thumb`] to follow the
    /// core.
    #[must_use]
    pub fn disassemble_virtual(&self, addr: u32, count: usize, thumb: bool) -> Vec<disasm::Listed> {
        let cfg = self.config();
        let session = self.session.lock();
        let Some(space) = session.space.clone() else {
            return Vec::new();
        };
        let mmu = Arc::clone(&session.mmu);
        drop(session);
        let attrs = MemAttrs::DEBUG.with_requester(cfg.requester);
        disasm::disassemble_run_for(&cfg.arch, addr, count, thumb, |a| {
            let pa = exec::debug_translate(&space, mmu.as_ref(), &cfg, a)
                .ok_or(disasm::Missing::Untranslated)?;
            space
                .read(u64::from(pa), crate::core::value::Width::U8, attrs)
                .map(|v| v as u8)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }

    /// Disassemble `count` instructions starting at the **physical** address
    /// `addr`.
    ///
    /// The untranslated form, and it is not a legacy shim: a monitor inspecting
    /// a ROM image, a board bring-up test and every conformance harness in the
    /// tree genuinely want a bus address, and forcing them through a page table
    /// the guest has not built yet would be the wrong answer, not a safer one.
    /// The caller says which it means — that is why neither of these is called
    /// `disassemble`.
    ///
    /// Debug attributes are the point either way: a monitor listing the code
    /// around the PC must not pop a FIFO or clear a status bit on the way
    /// (`ROADMAP.md` §15, invariant 5).
    #[must_use]
    pub fn disassemble_physical(
        &self,
        addr: u32,
        count: usize,
        thumb: bool,
    ) -> Vec<disasm::Listed> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let attrs = MemAttrs::DEBUG.with_requester(self.config().requester);
        disasm::disassemble_run_for(&self.cfg.arch, addr, count, thumb, |a| {
            space
                .read(u64::from(a), crate::core::value::Width::U8, attrs)
                .map(|v| v as u8)
                .map_err(|_| disasm::Missing::Unmapped)
        })
    }
}

/// Every name the `cpu` property accepts: the part whose architecture the
/// core implements.
pub const PARTS: &[&str] = &["arm926ejs", "cortex-a9"];

/// The architecture a part name selects. `or_enum` has already rejected
/// anything not in [`PARTS`].
fn part_arch(part: &str) -> Arch {
    match part {
        "cortex-a9" => Arch::CORTEX_A9,
        _ => Arch::V5TE,
    }
}

/// The `cpu.arm` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.arm",
    // 2: the chunk gained the scheduler debt, without which a restored core
    //    runs one instruction free.
    // 3: a core built with `cp15 = "arm926ejs"` appends its CP15 registers. A
    //    core without one writes the same bytes it wrote at v2, but the chunk
    //    is no longer the same shape for every instance of the class, so the
    //    version moves for all of them rather than silently for some.
    // 4: a core built with `cp15 = "cortex-a9"` appends its ARMv7 CP15
    //    registers instead. Every configuration that existed at v3 writes the
    //    same bytes it did, which is why the v3 step in `migrations` is the
    //    identity.
    // 5: a core whose part has VFP appends the register file, FPSCR and
    //    FPEXC after CP15. Same reasoning as 3: a core without writes the v4
    //    bytes, but the class's chunk is no longer one shape.
    // 6: a core whose part has ARMv6 appends the local exclusive monitor and
    //    the event register after everything else. An ARMv5 core's bytes are
    //    unchanged, and a v5 chunk without the trailer means the reset values:
    //    see `migrations`.
    // 7: every core appends its `hold` latch and its reset address, nine
    //    bytes, last; the v6 step in `migrations` appends what a v6 core
    //    implied (not held, the architectural vector).
    version: 7,
    summary: "A-profile 32-bit ARM CPU core: ARMv5TE (ARM926EJ-S) with Thumb, or ARMv7-A (Cortex-A9) with A32 and Thumb-2",
    properties: &[
        PropertySpec {
            name: "big-endian",
            kind: ValueKind::Bool,
            required: false,
            summary: "use big-endian byte order for data accesses",
        },
        PropertySpec {
            name: "high-vectors",
            kind: ValueKind::Bool,
            required: false,
            summary: "put the exception vectors at 0xffff0000 from reset (VINITHI)",
        },
        PropertySpec {
            name: "alignment-faults",
            kind: ValueKind::Bool,
            required: false,
            summary: "take a data abort on an unaligned access instead of rotating",
        },
        PropertySpec {
            name: "cp15",
            kind: ValueKind::Str,
            required: false,
            summary: "the system control coprocessor: `none`, `arm926ejs` (VMSAv5) or `cortex-a9` (VMSAv7); defaults to the part's own",
        },
        PropertySpec {
            name: "cpu-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "this core's number in its cluster, MPIDR.Aff0 (0-3; ARMv7 CP15 only)",
        },
        PropertySpec {
            name: "cluster-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the cluster's number, MPIDR.Aff1 (0-15; ARMv7 CP15 only)",
        },
        PropertySpec {
            name: "periphbase",
            kind: ValueKind::Addr,
            required: false,
            summary: "the MPCore private peripheral base CBAR reports (ARMv7 CP15 only)",
        },
        PropertySpec {
            name: "cpu",
            kind: ValueKind::Str,
            required: false,
            summary: "which part's architecture: `arm926ejs` (ARMv5TE) or `cortex-a9` (ARMv7-A)",
        },
        PropertySpec {
            name: "store-pc-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "what a store of R15 writes: the instruction plus 8 or plus 12",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
        PropertySpec {
            name: "held",
            kind: ValueKind::Bool,
            required: false,
            summary: "start held in reset until the `hold` input falls (default false)",
        },
    ],
    construct: |props| Ok(Box::new(Arm::from_props(props)?)),
};

/// This class's snapshot upgrade steps.
///
/// v3 to v4 is the identity: v4 added a byte layout only for a CP15 that did
/// not exist at v3, so every v3 chunk already is a valid v4 chunk. Registered
/// in the commit that bumped the version, because that is `machine::migrate`'s
/// rule — and because without it every v3 snapshot of an ARM926 board would be
/// refused for a change that did not touch it.
///
/// # Errors
///
/// v5 to v6 is the identity too: v6 added a trailer that only an
/// ARMv6-or-later core writes, and `load` reads it only if it is there.
///
/// # Errors
///
/// If a step is already registered for this class.
pub fn migrations(migrations: &mut crate::core::state::Migrations) -> Result<()> {
    migrations.register(CLASS.name, 3, |r, out| {
        let body = r.take(r.remaining())?;
        out.extend_from_slice(body);
        Ok(())
    })?;
    // v4 -> v5 is the identity for the same reason: only a VFP part appends
    // anything, and no configuration that could be saved at v4 had VFP state.
    migrations.register(CLASS.name, 4, |r, out| {
        let body = r.take(r.remaining())?;
        out.extend_from_slice(body);
        Ok(())
    })?;
    migrations.register(CLASS.name, 5, |r, out| {
        let body = r.take(r.remaining())?;
        out.extend_from_slice(body);
        Ok(())
    })?;
    // v6 -> v7 appends the trailer a v6 core implied: not held, the
    // architectural vector.
    migrations.register(CLASS.name, 6, |r, out| {
        let body = r.take(r.remaining())?;
        out.extend_from_slice(body);
        out.push(0);
        out.extend_from_slice(&NO_RESET_ADDRESS.to_le_bytes());
        Ok(())
    })
}

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-arm-aprofile")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Arm {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    /// The debug surface's route to the MMU: this is how a gdb `m` packet
    /// naming a virtual address reaches the right physical one.
    ///
    /// A 32-bit core, so the address is truncated on the way in and widened on
    /// the way out. An address above 4 GiB cannot be mapped by anything an
    /// ARMv5 has, so it is refused rather than silently wrapped into the low
    /// four gigabytes and answered as if it were a different address.
    fn debug_translate(&self, va: u64) -> DebugTranslation {
        let Ok(va) = u32::try_from(va) else {
            return DebugTranslation::Unmapped;
        };
        match self.translate_debug(va) {
            Some(pa) => DebugTranslation::Mapped(u64::from(pa)),
            None => DebugTranslation::Unmapped,
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::RESET_ADDRESS).then(|| Export::Cell(Arc::clone(&self.reset_address)))
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // CP15 is reset by the same signal the core is, warm or cold, so a
        // reset puts its registers back too — including the MMU enable, which
        // is what makes a rebooted machine fetch its reset vector physically
        // (DDI 0406C B1.9.10). The reset pin path does the same in `step`.
        match &self.cp15 {
            Some(SystemCp::V5(cp15)) => cp15.reset(),
            Some(SystemCp::V7(cp15)) => cp15.reset(),
            None => {}
        }
        {
            let mut session = self.session.lock();
            // Derived state, and the cheapest correct thing to do with it.
            session.tlb.flush();
            if kind == ResetKind::Cold {
                session.state = State::new();
            } else {
                // A warm reset is a pulse on the reset input: the reset
                // sequence runs, and nothing else is forced.
                session.state.reset_pending = true;
                session.state.halted = false;
                session.state.waiting_for_event = false;
            }
        }
        if kind == ResetKind::Cold {
            // The input levels belong to whatever drives them; only a cold
            // start may assume they are idle.
            self.lines.restore((false, false));
            // An event sent to the machine that was is not owed to this one.
            self.lines.take_event();
        }
        // The latch is internal bookkeeping either way: the sequence the
        // machine just asked for is the one it owed.
        self.lines.take_reset_request();
        // A core the board holds in reset goes back to being held when the
        // board resets; whatever releases it will again.
        if self.held_at_reset {
            self.lines.hold.store(true, Ordering::Release);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Fold the pin's latch in first. It is not a separate field in the
        // chunk: `reset_pending` is where it was always going, and a snapshot
        // taken between an assertion and the next step would otherwise lose
        // the reset entirely.
        let reset = self.lines.take_reset_request();
        let state = {
            let mut session = self.session.lock();
            session.state.reset_pending |= reset;
            session.state
        };
        for value in state.regs.r {
            w.write_u32(value)?;
        }
        w.write_u32(state.regs.cpsr)?;
        for bank in state.regs.banked_sp_lr {
            w.write_u32(bank[0])?;
            w.write_u32(bank[1])?;
        }
        for bank in state.regs.banked_r8_r12 {
            for value in bank {
                w.write_u32(value)?;
            }
        }
        for value in state.regs.spsr {
            w.write_u32(value)?;
        }
        w.write_u64(state.cycles)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.reset_pending)?;
        w.write_u64(state.faults)?;
        w.write_u32(state.last_fault)?;
        w.write_u32(state.last_swi)?;
        w.write_u16(state.last_bkpt)?;
        w.write_u64(state.debt)?;
        let (irq, fiq) = self.lines.snapshot();
        w.write_bool(irq)?;
        w.write_bool(fiq)?;
        // CP15 last, so the bytes a core without one writes are exactly the
        // bytes it always wrote. The TLB is not here and never will be: it is
        // derived state, and a snapshot that carried it would be asserting
        // something about the future rather than about the machine.
        match &self.cp15 {
            Some(SystemCp::V5(cp15)) => cp15.save(w)?,
            Some(SystemCp::V7(cp15)) => cp15.save(w)?,
            None => {}
        }
        // VFP after CP15, for the same reason CP15 is last: a part without
        // one writes exactly what it always did. `new` has already cleared
        // `vfp` in a build without the feature, so this is the one test.
        #[cfg(feature = "cpu-arm-aprofile-vfp")]
        if self.cfg.arch.ext.vfp.is_some() {
            state.vfp.save(w)?;
        }
        // v4: the ARMv6 execution state, only for a part that has it — so an
        // ARMv5 core's chunk is byte-for-byte what it was.
        if self.cfg.arch.ext.v6 {
            let event = state.event || self.lines.event.load(Ordering::Acquire);
            match state.monitor.tag() {
                Some(tag) => {
                    w.write_bool(true)?;
                    w.write_u32(tag)?;
                }
                None => {
                    w.write_bool(false)?;
                    w.write_u32(0)?;
                }
            }
            w.write_bool(event)?;
            w.write_bool(state.waiting_for_event)?;
        }
        // v7, last for every part.
        w.write_bool(self.lines.hold.load(Ordering::Acquire))?;
        w.write_u64(self.reset_address.load(Ordering::Acquire))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        for value in &mut state.regs.r {
            *value = r.read_u32()?;
        }
        state.regs.cpsr = r.read_u32()?;
        for bank in &mut state.regs.banked_sp_lr {
            bank[0] = r.read_u32()?;
            bank[1] = r.read_u32()?;
        }
        for bank in &mut state.regs.banked_r8_r12 {
            for value in bank {
                *value = r.read_u32()?;
            }
        }
        for value in &mut state.regs.spsr {
            *value = r.read_u32()?;
        }
        state.cycles = r.read_u64()?;
        state.halted = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u32()?;
        state.last_swi = r.read_u32()?;
        state.last_bkpt = r.read_u16()?;
        state.debt = r.read_u64()?;
        let irq = r.read_bool()?;
        let fiq = r.read_bool()?;
        match &self.cp15 {
            Some(SystemCp::V5(cp15)) => cp15.load(r)?,
            Some(SystemCp::V7(cp15)) => cp15.load(r)?,
            None => {}
        }
        #[cfg(feature = "cpu-arm-aprofile-vfp")]
        if self.cfg.arch.ext.vfp.is_some() {
            state.vfp = vfp::VfpRegs::load(r)?;
        }
        // The v6 trailer, before the v7 one's nine bytes. A v3 chunk (carried forward unchanged by
        // `migrations`) has none, and a v3 build never executed an `LDREX` or
        // a `WFE` — it decoded both as Undefined — so the state it implies is
        // exactly the default: monitor open, no event, not waiting.
        if self.cfg.arch.ext.v6 && r.remaining() > 9 {
            let armed = r.read_bool()?;
            let tag = r.read_u32()?;
            state.monitor = LocalMonitor::from_tag(armed.then_some(tag));
            state.event = r.read_bool()?;
            state.waiting_for_event = r.read_bool()?;
        }
        let held = r.read_bool()?;
        let reset_address = r.read_u64()?;
        // Stored, not set: a restore is not a release.
        self.lines.hold.store(held, Ordering::Release);
        self.reset_address.store(reset_address, Ordering::Release);
        self.lines.take_event();
        {
            let mut session = self.session.lock();
            session.state = state;
            // Whatever the TLB held describes the machine we are replacing.
            session.tlb.flush();
        }
        self.lines.restore((irq, fiq));
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made.
        //
        // Every pin is named the way the package names it, minus the bar:
        // `nIRQ` and `nFIQ` are asserted low on real silicon, and inverting a
        // level belongs to whatever models the wire, not to the core.
        let mut pins = self.pins.lock();
        let sink: Arc<dyn WireSink> = match port {
            "irq" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Irq,
                    sources,
                ));
                pins.irq = Some(Arc::clone(&pin));
                pin
            }
            "fiq" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Fiq,
                    sources,
                ));
                pins.fiq = Some(Arc::clone(&pin));
                pin
            }
            "reset" => {
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                pin
            }
            "hold" => {
                let pin = Arc::new(HoldPin {
                    lines: Arc::clone(&self.lines),
                    inputs: FanIn::new(sources),
                });
                pins.hold = Some(Arc::clone(&pin));
                pin
            }
            _ => return None,
        };
        Some(SinkPin { sink, line: 0 })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }
}

impl Initiator for Arm {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a core needs an address space, and this is where
/// the machine gives it one.
///
/// **CP15 does not arrive here either**, and that is the point: it arrived at
/// construction. `cp15 = "arm926ejs"` on the object is read by
/// [`Arm::from_props`], so by the time the machine layer is binding an address
/// space the core already has its MMU (see [`System`]). Binding stayed a
/// two-line function and `Device::export` did not have to grow a shape for a
/// `dyn Coprocessor`.
///
/// What a downstream SoC still does through [`Arm::attach_mmu`] and
/// [`Arm::attach_coprocessor`] is add what is genuinely its own: the caches,
/// the TCMs, a coprocessor 14.
impl crate::machine::Instance for Arm {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        // A CPU with no address space cannot fetch, and a machine that runs
        // zero instructions and says nothing is the worst of both worlds.
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "an ARM core needs an address space to fetch from (`space = mem`)",
            ),
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
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Arm::from_props(props)?)))
}

/// What the validator should know about `cpu.arm`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("big-endian", ValueKind::Bool))
        .prop(PropSchema::new("high-vectors", ValueKind::Bool))
        .prop(PropSchema::new("alignment-faults", ValueKind::Bool))
        .prop(PropSchema::new("store-pc-offset", ValueKind::Uint).range(8, 12))
        .prop(PropSchema::new("cp15", ValueKind::Str).values(System::NAMES))
        .prop(PropSchema::new("cpu", ValueKind::Str).values(PARTS))
        .prop(PropSchema::new("cpu-id", ValueKind::Uint).range(0, 3))
        .prop(PropSchema::new("cluster-id", ValueKind::Uint).range(0, 15))
        .prop(PropSchema::new("periphbase", ValueKind::Addr))
        .prop(PropSchema::new("held", ValueKind::Bool))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        // Inputs only: an ARM926EJ-S drives nothing this core models. The
        // bus-facing outputs a real part has -- `nMREQ`, `nRW`, `nWAIT` -- are
        // the address space's business, not a wire's.
        .port("irq", PortDir::In)
        .port("fiq", PortDir::In)
        .port("reset", PortDir::In)
        .port("hold", PortDir::In)
}

/// The `hold` input: the core is held in reset while it is high, and leaves
/// reset — from its reset address — when it falls.
///
/// Separate from `reset`, which is a pulse the core answers at once even if
/// the line stays asserted: a board that keeps a secondary core off until
/// software releases it needs the level.
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

/// One of the core's two interrupt inputs, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, because a net with several drivers is resolved
/// by whoever cares. An ARM interrupt line typically has one driver — an
/// interrupt controller — but wire-OR is the right default for the open-drain
/// case, and it is what an SoC without a controller does.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    which: Interrupt,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect `which` input of `cpu` to a net driven by `sources`.
    ///
    /// The pin keeps a handle on the core's *input latches*, not on the core:
    /// the core owns the pin — something must, since a net holds only a weak
    /// reference to its sinks — and a pin that owned the core back would be a
    /// cycle the machine could never drop.
    #[must_use]
    pub fn new(cpu: Arc<Arm>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin::from_lines(Arc::clone(&cpu.lines), which, sources)
    }

    /// The same, given the latches directly.
    fn from_lines(lines: Arc<Lines>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            which,
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

    /// Which input this is.
    #[must_use]
    pub fn which(&self) -> Interrupt {
        self.which
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
        match self.which {
            Interrupt::Irq => self.lines.irq.store(asserted, Ordering::Release),
            Interrupt::Fiq => self.lines.fiq.store(asserted, Ordering::Release),
        }
    }
}

/// The core's reset input, as something a [`Wire`] can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no mask, no banked link register and no vector of its own beyond address
/// zero. Asserting the line latches a request; the sequence itself runs on the
/// next [`Arm::step`], which is when the core can fetch from the vector.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect `cpu`'s reset pin to a net driven by `sources`.
    #[must_use]
    pub fn new_for(cpu: Arc<Arm>, sources: &[WireId]) -> ResetPin {
        ResetPin::new(Arc::clone(&cpu.lines), sources)
    }

    /// The same, given the latches directly.
    fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
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
