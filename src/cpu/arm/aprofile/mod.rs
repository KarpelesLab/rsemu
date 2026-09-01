//! The ARMv5TE core — an ARM926EJ-S-class interpreter with Thumb, the DSP
//! extensions, and a coprocessor seam where CP15 goes.
//!
//! Covers what an ARM9 SoC needs and nothing it does not: the full 32-bit ARM
//! instruction set including `CLZ`, both forms of `BLX`, `BKPT` and the E
//! extensions (`QADD`, `SMLA<x><y>`, `LDRD`/`STRD`, `PLD`); the full 16-bit
//! Thumb set with interworking; all seven processor modes with their banked
//! registers; the complete exception model; and, when a machine asks for one,
//! a real [`cp15::Cp15`] with the VMSAv5 MMU behind it. The caches and the
//! TCMs are **not** here — those are the SoC's, and anything else it wants to
//! add attaches through [`cp::Coprocessor`] and [`cp::Mmu`].
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
//! [`Arm::attach_coprocessor`] for the system seam, and [`Arm::disassemble`]
//! for a listing.
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
//! | [`thumb`] | the same for Thumb |
//! | [`disasm`] | the disassembler built on those two |
//! | [`cp`] | the coprocessor and MMU traits, the software TLB, `FlatMmu`, and a CP15 stub |
//! | [`cp15`] | the ARMv5 system control coprocessor and the VMSAv5 table walk |
//! | `exec` (private) | the interpreter, and the timing model it implements |
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual*, ARM DDI 0100, ARMv5 revisions —
//! chapters A2 (programmer's model), A3 (ARM encodings), A4 (ARM
//! instructions), A5 (addressing modes), A6/A7 (Thumb), A10 (the DSP
//! extensions), B2 (the system control coprocessor) and B4 (fault status).
//! Cycle counts from ARM's own instruction-cycle timing summaries. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

pub mod cp;
pub mod cp15;
pub mod disasm;
mod exec;
pub mod isa;
pub mod thumb;

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
use crate::core::value::Endian;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use cp::{Coprocessor, FlatMmu, Mmu, Tlb};
use cp15::Cp15;
use exec::{Exec, State};

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
}

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
}

impl System {
    /// The name a `.machine` file writes.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            System::None => "none",
            System::Arm926EjS => "arm926ejs",
        }
    }

    /// Every name the `cp15` property accepts.
    pub const NAMES: &'static [&'static str] = &["none", "arm926ejs"];

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
            .finish()
    }
}

/// An ARMv5TE core.
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
    cp15: Option<Arc<Cp15>>,
    lines: Arc<Lines>,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    ///
    /// Separate from [`Config::requester`] because a machine file names no
    /// requester: the machine layer allocates one per initiator and hands it
    /// over in [`Instance::bind`](crate::machine::Instance::bind), which is
    /// after `new` (`ROADMAP.md` §4.4).
    requester: AtomicU32,
    session: sync::Mutex<Session>,
    /// The strong end of every pin this core has handed to a wire.
    ///
    /// A net holds its sinks weakly — the machine owns devices and a wire
    /// merely refers to them (§4.3) — so a pin nothing else kept alive would
    /// die on the way out of [`Device::sink`] and the wire would silently
    /// deliver to nothing.
    pins: sync::Mutex<Pins>,
}

/// The pins [`Device::sink`] has built, kept alive by the core that owns them.
#[derive(Debug, Default)]
struct Pins {
    irq: Option<Arc<InterruptPin>>,
    fiq: Option<Arc<InterruptPin>>,
    reset: Option<Arc<ResetPin>>,
}

impl Arm {
    /// A core in its power-on state, with no address space and no
    /// coprocessors.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Arm::attach_space) and [`Device::realize`]. The
    /// first [`step`](Arm::step) runs the reset sequence, which is what puts
    /// the PC on the reset vector.
    #[must_use]
    pub fn new(cfg: Config) -> Arm {
        let cp15 = match cfg.system {
            System::None => None,
            System::Arm926EjS => Some(Arc::new(Cp15::arm926ejs(&cfg))),
        };
        // `Option<Arc<_>>` is not `Copy`, so the array cannot be written
        // `[None; 16]`.
        let mut coprocessors: [Option<Arc<dyn Coprocessor>>; 16] = [const { None }; 16];
        let mmu: Arc<dyn Mmu> = match &cp15 {
            Some(cp) => {
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
            lines: Arc::new(Lines::default()),
            requester: AtomicU32::new(cfg.requester.0),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                    mmu,
                    coprocessors,
                    tlb: Tlb::new(),
                },
            ),
            pins: sync::Mutex::new(Pins::default()),
        }
    }

    /// This core's system control coprocessor, if it was built with one.
    ///
    /// The concrete type rather than a trait object: a monitor listing CP15,
    /// a test asserting a fault status, and a SoC that wants to seed the
    /// translation table base all want to read named registers.
    #[must_use]
    pub fn cp15(&self) -> Option<&Arc<Cp15>> {
        self.cp15.as_ref()
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
        let system = r.or_enum("cp15", "none", System::NAMES)?;
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
        Ok(Arm::new(Config {
            requester: RequesterId::ANONYMOUS,
            endian: if big_endian {
                Endian::Big
            } else {
                Endian::Little
            },
            high_vectors,
            alignment_faults,
            store_pc_offset: store_pc_offset as u8,
            // `or_enum` already rejected anything not in `NAMES`.
            system: System::parse(system).unwrap_or(System::None),
        }))
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
    /// CP15's "wait for interrupt" register is implemented. A halted core
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
        let (irq, fiq) = self.lines.snapshot();
        let reset = self.lines.take_reset_request();
        let cfg = self.config();
        let mut session = self.session.lock();
        let Session {
            state,
            space,
            mmu,
            coprocessors,
            tlb,
        } = &mut *session;
        // The `reset` pin latches outside the lock; this is where the latch
        // becomes execution state, and it must happen before the step so an
        // assertion is honoured by the very next instruction boundary.
        state.reset_pending |= reset;
        let Some(space) = space.clone() else {
            return 0;
        };
        let mmu = Arc::clone(mmu);
        Exec::new(state, &space, mmu.as_ref(), tlb, coprocessors, &cfg).step(irq, fiq)
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

    /// Disassemble `count` instructions starting at `addr`, reading guest
    /// memory with debug attributes.
    ///
    /// `thumb` picks the instruction set; pass [`Arm::is_thumb`] to follow the
    /// core. Debug attributes are the point: a monitor listing the code around
    /// the PC must not pop a FIFO or clear a status bit on the way
    /// (`ROADMAP.md` §15, invariant 5).
    ///
    /// **`addr` is physical.** With the MMU on, a caller that passes
    /// [`pc`](Arm::pc) — a virtual address — gets whatever is at that physical
    /// address instead, which is usually nothing. Translating here needs the
    /// walk to run under [`MemAttrs::DEBUG`] and needs an answer for a listing
    /// that runs off the end of a mapped page, and neither is decided; until it
    /// is, a debugger that wants a translated listing translates first.
    #[must_use]
    pub fn disassemble(&self, addr: u32, count: usize, thumb: bool) -> Vec<disasm::Listed> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(addr, count, thumb, |a| {
            space
                .read(u64::from(a), crate::core::value::Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
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
    version: 3,
    summary: "ARMv5TE (ARM926EJ-S class) 32-bit CPU core with Thumb and the DSP extensions",
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
            summary: "the system control coprocessor: `none`, or `arm926ejs` for CP15 and the MMU",
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
    ],
    construct: |props| Ok(Box::new(Arm::from_props(props)?)),
};

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

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // CP15 is reset by the same signal the core is, so a cold start puts
        // its registers back too — including the MMU enable, which is what
        // makes a rebooted machine fetch its reset vector physically.
        if kind == ResetKind::Cold
            && let Some(cp15) = &self.cp15
        {
            cp15.reset();
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
            }
        }
        if kind == ResetKind::Cold {
            // The input levels belong to whatever drives them; only a cold
            // start may assume they are idle.
            self.lines.restore((false, false));
        }
        // The latch is internal bookkeeping either way: the sequence the
        // machine just asked for is the one it owed.
        self.lines.take_reset_request();
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
        if let Some(cp15) = &self.cp15 {
            cp15.save(w)?;
        }
        Ok(())
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
        if let Some(cp15) = &self.cp15 {
            cp15.load(r)?;
        }
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
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        // Inputs only: an ARM926EJ-S drives nothing this core models. The
        // bus-facing outputs a real part has -- `nMREQ`, `nRW`, `nWAIT` -- are
        // the address space's business, not a wire's.
        .port("irq", PortDir::In)
        .port("fiq", PortDir::In)
        .port("reset", PortDir::In)
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
