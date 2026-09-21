//! The Motorola 680x0 — the 68000 as a bus-accurate interpreter with a
//! modelled prefetch queue, and the 68010, 68020 and 68EC020 as models of it.
//!
//! The plain 68000, as fitted to the Amiga, the Atari ST, the Mega Drive and
//! the first Macintoshes: 32-bit registers, a 16-bit data bus, 24 address
//! pins, two stack pointers and a supervisor/user split. And, chosen by the
//! `model` property ([`Model`]), the 68010 some of those machines were
//! upgraded with and the 68020 the later Amigas were built around — see
//! *The 68010* and *The 68020* below. A 68000 is exactly what it was before
//! the other models existed: the same bus cycles, the same times, the same
//! snapshot bytes.
//!
//! # The 68010
//!
//! The same core with the 68010's differences (MC68000UM §1.3, §6, §9;
//! M68000PRM Appendix A):
//!
//! - a vector base register, and `MOVEC` to reach it and the `SFC`/`DFC`
//!   function-code registers; `MOVES` to use them; `RTD`; `MOVE from CCR`;
//!   `BKPT`, which with no breakpoint hardware on the bus is an illegal
//!   instruction;
//! - `MOVE from SR` privileged;
//! - a format word in every exception frame: format `$0` for four words, and
//!   format `$8` — twenty-nine words — for a bus or address error, which `RTE`
//!   can return from. This core restarts the faulted instruction rather than
//!   continuing it from microcode state it does not have; `exec.rs`'s `fault`
//!   says how that is made to come out the same, including a cycle the
//!   handler completed in software;
//! - `CLR`, `Scc` and `MOVE from SR` no longer read their destination first.
//!
//! Its timing is the 68000's, per access, with those accesses removed and the
//! format word's write added, which makes every exception time in MC68000UM
//! Table 9-19 that the core can take come out as published. The 68010's own
//! faster microcode for `MULU`, `DIVU` and some branches (Tables 9-6 and 9-15)
//! is not modelled, and neither is **loop mode** — a one-word instruction
//! repeated by the `DBcc` after it, run from the prefetch queue with only its
//! operand cycles on the bus (MC68000UM Appendix A), a timing effect with no
//! architectural one.
//!
//! # The 68020
//!
//! The 68010 plus everything MC68020UM and M68000PRM give the 68020, with no
//! coprocessor:
//!
//! - 32 address pins on the 68020, 24 on the 68EC020 — the only difference
//!   between the two (MC68020UM §1) — and operands at any alignment; only an
//!   instruction fetch from an odd address is an address error;
//! - the scaled brief extension word and the full format: base and outer
//!   displacements, suppressed base and index, memory indirection pre- and
//!   post-indexed (M68000PRM §2.2);
//! - `MULS.L`/`MULU.L`, `DIVS.L`/`DIVU.L`/`DIVSL.L`/`DIVUL.L`, the eight bit-field
//!   instructions, `CAS`, `CAS2`, `CHK2`, `CMP2`, `CHK.L`, `PACK`, `UNPK`,
//!   `EXTB.L`, `LINK.L`, `TRAPcc`, `Bcc.L`, `CALLM` and `RTM` for type 0
//!   descriptors, and `TST`/`CMPI` on the extra modes;
//! - the master and interrupt stack pointers and the **M** bit, trace on
//!   change of flow (**T0**), and the stack frames of MC68020UM Table 6-5:
//!   formats `$0`, `$1` (the throwaway frame an interrupt leaves on the
//!   interrupt stack when taken in master state), `$2`, `$A` and `$B`. Format
//!   `$9` belongs to a coprocessor and is never built; `RTE` treats it as a
//!   format error;
//! - `CACR` and `CAAR` through `MOVEC`. **The instruction cache is state
//!   only**: its enable and freeze bits are kept, clear and clear-entry act on
//!   contents that are not modelled, and it has no effect on timing or on
//!   what reaches the bus — every fetch goes to memory, as with the cache off;
//! - with no coprocessor, every F-line word is the line-F exception, and
//!   `cpSAVE`/`cpRESTORE` are privileged first (MC68020UM §7.5.2);
//! - a prefetch the 68020 cannot complete is a bus error only when the word is
//!   used (§6.1.2), so running up to the last word of mapped memory is not
//!   one.
//!
//! Its **timing is the cache-case column** of MC68020UM §8.2, charged per
//! instruction: the one place this core uses a table, because the 68020 has
//! no published per-access timing to count. `timing.rs` says why that column
//! and where the tables are silent. Its bus cycles are the 68000's sequence,
//! driven sixteen bits at a time as a 68020 does on a 16-bit port — which is
//! what every region this framework maps accepts — and are not what its time
//! is built from.
//!
//! # The 68030
//!
//! The 68020 with the differences MC68030UM records, in both packages:
//!
//! - **`CALLM` and `RTM` are gone.** A 68030 takes an unimplemented
//!   instruction exception on either (MC68030UM §12.1.3), which for their
//!   line-0 encodings is vector 4. Nothing else was removed, and nothing
//!   outside the F line was added.
//! - **32 address pins on both parts.** The MC68EC030 drops the MMU, not the
//!   address bus (MC68EC030UM §1), so unlike the 68EC020 it is *not* a 24-bit
//!   machine.
//! - **`CACR` grew a data cache.** **EI**, **FI**, **IBE**, **ED**, **FD**,
//!   **DBE** and **WA** have storage; the four clear bits — **CEI**, **CI**,
//!   **CED** and **CD** — act on contents and read back as zero (MC68030UM
//!   §6.3.1, Figure 6-14). As on the 68020 the caches themselves are state
//!   only: every access goes to memory and nothing here is a cache hit.
//! - **The stack frames and the special status word are the 68020's**, format
//!   for format and bit for bit (MC68030UM §8.2.1 and Table 8-6), so bus and
//!   address faults behave exactly as they do on a 68020 — including this
//!   core's restart-instead-of-continue reading of `RTE`.
//! - **The paged memory management unit**, on the full 68030 only —
//!   `PMOVE`, `PTEST`, `PLOAD` and `PFLUSH`, the translation tree, the
//!   twenty-two-entry address translation cache and the two transparent
//!   translation registers. `mmu.rs` is the whole of it and says where each
//!   piece comes from. An MC68EC030 keeps `TT0`, `TT1` and `MMUSR` under
//!   their other names — `AC0`, `AC1` and `ACUSR` — and takes the line-F
//!   exception on everything else in that class (MC68EC030UM §9.4).
//! - Its time is the 68020's table, which is stated as an approximation
//!   rather than measured — `timing.rs` says exactly how far that goes.
//!
//! What a 68030 does **not** have here, beyond the caches above: `CIOUT` and
//! `MMUDIS` have no pins to drive or be driven, and the burst fills `CACR`'s
//! **IBE** and **DBE** enable have nothing to fill.
//!
//! # What "bus-accurate" means here
//!
//! A 68000 bus cycle is four clocks, and every published instruction time is a
//! sum of bus cycles and microcode idle cycles (MC68000UM §8). This
//! interpreter has no per-instruction cycle table for it: each access it makes
//! charges four, and the idle time is charged where the manual says it is
//! spent. A device watching the bus sees the same reads and writes real
//! hardware would, in the same order — including the extra word `MOVEM` reads
//! past the end of its register list, and the destination read `CLR` performs
//! before writing zero.
//!
//! # The prefetch queue
//!
//! The 68000 holds two instruction words and refills them a word at a time,
//! and that is *observable*: it decides the program counter an address-error
//! frame pushes and the order a `MOVE` to an absolute long address puts its
//! write in. So it is modelled, not approximated. The invariant is that
//! [`Regs::prefetch`]`[0]` is the word at [`Regs::pc`] and `prefetch[1]` is
//! the word at `pc + 2`; executing one instruction slides the queue once per
//! instruction word. The module documentation on `exec.rs` has the long form.
//! The later models keep the same queue.
//!
//! # Big-endian, and only 24 address pins
//!
//! The 68000 is big-endian, so **every region this core reaches must declare
//! big-endian byte order** — `Region::ram(..).with_endian(Endian::Big)`, and
//! `AddressSpace::with_endian(Endian::Big)` for the unmapped fallback. Byte
//! order is a property of the region rather than of the master
//! (`ROADMAP.md` §4.1), which is what lets a little-endian device sit on the
//! same bus; the core does not byte-swap behind the framework's back.
//!
//! Addresses reach the bus modulo 16 MiB, because A24–A31 are not brought out
//! of the package. `(xxx).L` with a high byte set therefore aliases into the
//! low 16 MiB, which is how the Amiga's mirrors and the Mac's 24-bit mode
//! work, and the core masks every access accordingly. So do the 68010 and the
//! 68EC020; the 68020 drives all 32 lines, and needs a 32-bit space.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::core::value::Endian;
//! use rsemu::cpu::m68k::{Config, M68k};
//!
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! // Reset vector: SSP = $2000, PC = $400.
//! for (offset, byte) in [(3, 0x20u8), (6, 0x04), (7, 0x00)] {
//!     ram.write_u8(offset, byte).unwrap();
//! }
//! // MOVEQ #$42,D0 at $400.
//! ram.write_u8(0x400, 0x70).unwrap();
//! ram.write_u8(0x401, 0x42).unwrap();
//!
//! let space = AddressSpace::new("cpu", 24).with_endian(Endian::Big);
//! let region = Region::ram("ram", ram).with_endian(Endian::Big);
//! space.topology().map(region, 0).unwrap();
//!
//! let cpu = M68k::new(Config::default());
//! cpu.attach_space(Arc::new(space));
//! cpu.step();                       // the reset sequence
//! assert_eq!(cpu.regs().pc, 0x400);
//! cpu.step();                       // MOVEQ
//! assert_eq!(cpu.regs().d[0], 0x42);
//! ```
//!
//! # The floating-point coprocessor
//!
//! The `fpu` property attaches an **MC68881** or **MC68882** to any part
//! with the F-line coprocessor interface — a 68020 or later — and `none`,
//! the default, leaves every existing board exactly what it was. With one
//! attached, coprocessor id 1's encodings become instructions; without one
//! they are the line-F exception, which is what a main processor takes when
//! nothing answers.
//!
//! Implemented (M68881UM; `fpu.rs` and `transcend.rs`):
//!
//! - the eight 80-bit data registers, `FPCR`, `FPSR` and `FPIAR`, every
//!   field and every rule the manual gives them — the condition codes by
//!   result *data type* (Table 2-1), the eight exception bits, the five
//!   accrued equations (§2.3.4), and `FPIAR` loaded before an instruction
//!   that can trap;
//! - six of the seven external formats: byte, word and long integers,
//!   single, double and extended. **Packed decimal is not implemented** and
//!   takes the line-F exception, which is what a 68040 does for that format;
//! - `FMOVE` in both directions, `FMOVEM` in both list orders and both
//!   dynamic forms, `FMOVECR`'s whole constant ROM, and `FMOVE(M)` for the
//!   three control registers;
//! - `FBcc`, `FScc`, `FDBcc`, `FTRAPcc` and `FNOP`, all thirty-two
//!   predicates, with `BSUN` raised by the sixteen that signal and taken as
//!   a **pre-instruction** exception so an `RTE` that changes nothing runs
//!   into it again;
//! - the arithmetic — `FADD`, `FSUB`, `FMUL`, `FDIV`, `FSGLMUL`, `FSGLDIV`,
//!   `FABS`, `FNEG`, `FSQRT`, `FINT`, `FINTRZ`, `FGETEXP`, `FGETMAN`,
//!   `FSCALE`, `FMOD`, `FREM`, `FCMP`, `FTST` — and all eighteen
//!   transcendentals;
//! - `FSAVE` and `FRESTORE`, with the null frame for an untouched unit and
//!   an idle frame of the coprocessor's own length.
//!
//! **Every value goes through `src/float`**, which is integer arithmetic
//! rounded exactly once, so a guest's floating point is identical on every
//! host and in a browser (`ROADMAP.md` §9.1). The 68881's extended format is
//! x87's value encoding with four differences — a ninety-six-bit memory
//! layout, unnormalized numbers as values, pseudo-infinities and pseudo-NaNs
//! as ordinary ones, and its own created NaN — and `fpu.rs` says what each
//! one costs.
//!
//! The transcendentals are computed at **twice** the destination's precision
//! and rounded once, and their argument reduction is exact for every
//! representable argument. That is *more* accurate than the part, whose own
//! manual allows it 4096 units in the last place (§4.3.2) and which "loses
//! all accuracy" for trigonometric arguments above about 10^20; a hundred
//! and thirty-three values checked against GNU `bc` at ninety digits come
//! back correctly rounded. `docs/cpu/m68k.md` lists that and the rest of
//! what differs from the part.
//!
//! # How accurate, measured
//!
//! `ROADMAP.md` §0: accuracy is measured, never asserted. Against
//! `SingleStepTests/680x0`'s 68000 corpus — 124 instruction files, 1 000 058
//! vectors — this core reproduces **every** vector's final registers, both
//! stack pointers, prefetch queue and memory, **and** every vector's cycle
//! count, **and** every vector's complete bus trace, access for access in
//! order. Two vectors are skipped as corpus errors and are named and argued
//! for in the runner, and the known-failures ledger carries nothing the corpus
//! covers.
//!
//! The corpus has no licence file, so it is fetched and run, never vendored.
//! `src/cpu/m68k/conformance.rs` has the command.
//!
//! There is no such corpus for the 68010 or 68020. The same runner pushes every
//! 68000 vector through each of them as well and fails on any difference from
//! the 68000's result that is not one the manuals document; it prints each
//! documented difference with the number of vectors that show it. The new
//! instructions, addressing modes and frames are covered by hand-written tests
//! whose expected values come from the manuals.
//!
//! What the corpus does *not* reach, because every vector runs in supervisor
//! state with the interrupt mask at seven and tracing off: reset, interrupts,
//! `STOP`, tracing, user mode and the privilege violation. Those are covered
//! by the hand-written tests beside it.
//!
//! # What is not modelled
//!
//! Stated here rather than discovered later:
//!
//! - **CPU space.** `MemAttrs` carries no function code, so nothing on the bus
//!   can answer a cycle in it. The interrupt-acknowledge cycle is charged but
//!   not driven: a vectoring controller arms its vector through
//!   [`M68k::set_interrupt_vector`] or answers through `core::wire`'s
//!   `IntAck`, and there is no spurious-interrupt path. `BKPT`'s acknowledge
//!   goes unanswered and ends in an illegal instruction; `CALLM` with a type 1
//!   descriptor, which needs access-control hardware there, is a format error;
//!   `MOVES` to a function code keeps only its supervisor bit.
//! - **`STOP`'s bus behaviour.** It settles the prefetch queue before
//!   stopping, which costs two bus cycles hardware makes on the way out
//!   instead. The state is identical; the trace and the four-cycle published
//!   time are not.
//! - **Continuing a faulted instruction.** `RTE` from a 68010 or 68020 long
//!   bus-fault frame restarts the instruction instead; see `exec.rs`.
//! - **A 68020 on a 32-bit port.** The bus is driven sixteen bits at a time
//!   whatever the region; the 68020's dynamic bus sizing to a wider port is
//!   not modelled.
//! - **The coprocessor interface itself.** A 68881 is a device in CPU space
//!   and the main processor talks to it through coprocessor interface
//!   registers (M68881UM §7); with no function code on the bus nothing can
//!   answer one, so the instructions are executed directly. The bus trace of
//!   a floating-point instruction is therefore not the hardware's, and
//!   neither is its time. Nor is the *concurrency*: a real coprocessor runs
//!   beside the main processor, so an arithmetic trap arrives
//!   pre-instruction on the *next* floating-point instruction, where this
//!   core reports it post-instruction on the one that caused it. `FPIAR`,
//!   `FPSR` and the vector are the same either way.
//! - **Packed decimal.** The `011` and `111` source and destination formats
//!   take the line-F exception.
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction description; decode and disassembly both read it |
//! | [`disasm`] | the disassembler generated from that description |
//! | `exec` (private) | the interpreter, the prefetch queue and exception processing |
//! | `timing` (private) | the 68020's cache-case instruction times |
//! | `mmu` (private) | the 68030's translation tree, cache and registers |
//! | `fpu` (private) | the coprocessor's registers, formats and arithmetic |
//! | `transcend` (private) | its transcendentals, at 128-bit precision |
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): the *M68000 Family
//! Programmer's Reference Manual* (Motorola M68000PM/AD) for the instruction
//! set, encodings and condition codes; the *M68000 8-/16-/32-Bit
//! Microprocessors User's Manual* (MC68000UM) for the 68000's and the 68010's
//! exception processing, stack frames, signals and timing tables; and the
//! *MC68020 User's Manual* (MC68020UM) for the 68020's; and the *MC68030
//! User's Manual* (MC68030UM) and the *MC68EC030 User's Manual* for the two
//! 68030 packages, Section 9 of each being the memory management unit and the
//! access control unit respectively; and the *MC68881/MC68882 Floating-Point
//! Coprocessor User's Manual* (M68881UM) for the coprocessor. All are listed
//! in `docs/cpu/m68k.md`. No copyleft emulator was consulted, and no emulator
//! source of any licence was used for the instruction semantics or for the
//! transcendental algorithms.

pub mod disasm;
mod exec;
mod fpu;
pub mod isa;
mod mmu;
mod mmu040;
mod timing;
mod transcend;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_68010;
#[cfg(test)]
mod tests_68020;
#[cfg(test)]
mod tests_68030;
#[cfg(test)]
mod tests_68040;
#[cfg(test)]
mod tests_fpu;
#[cfg(test)]
mod tests_transcend;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt::{self, Write as _};

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU8, AtomicU16, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{
    FanIn, IntAck, IntAckCycle, IntAckHandlers, IntAckResponse, Level, Resolve, WireId, WireSink,
};
use crate::float::x87::F80;

use exec::{Bank, Exec, State};

pub use fpu::Coprocessor;
pub use isa::Model;

/// The values the `fpu` property accepts, in [`Coprocessor::ALL`] order.
const FPU_NAMES: [&str; 4] = [
    Coprocessor::None.name(),
    Coprocessor::M68881.name(),
    Coprocessor::M68882.name(),
    Coprocessor::M68040.name(),
];

/// The values the `model` property accepts, in [`Model::ALL`] order.
const MODEL_NAMES: [&str; 9] = [
    Model::M68000.name(),
    Model::M68010.name(),
    Model::M68020.name(),
    Model::M68EC020.name(),
    Model::M68030.name(),
    Model::M68EC030.name(),
    Model::M68040.name(),
    Model::M68LC040.name(),
    Model::M68EC040.name(),
];

/// The 24 address pins.
///
/// The 68000 has 32-bit registers and 24 address lines: A0 is not brought out
/// (the two byte-select strobes replace it) and A24–A31 do not exist. Every
/// address therefore reaches the bus modulo 16 MiB, which is why an Amiga sees
/// its chip RAM mirrored and why `(xxx).L` with a high byte set still lands in
/// the low 16 MiB (MC68000UM §3, *Signal Description*).
pub const ADDRESS_MASK: u32 = 0x00ff_ffff;

/// The status register's bits.
///
/// The low byte is the condition code register, which user code may write; the
/// high byte is the *system byte* — trace, supervisor state and the interrupt
/// mask — and writing it requires supervisor state (M68000PRM §1.3).
pub mod flags {
    /// Carry.
    pub const C: u16 = 0x0001;
    /// Overflow.
    pub const V: u16 = 0x0002;
    /// Zero.
    pub const Z: u16 = 0x0004;
    /// Negative.
    pub const N: u16 = 0x0008;
    /// Extend — the carry a multi-precision operation propagates.
    ///
    /// Separate from **C** on purpose: `CMP` sets carry without disturbing the
    /// extend of an `ADDX` chain in progress.
    pub const X: u16 = 0x0010;
    /// Every condition code bit.
    pub const CCR: u16 = 0x001f;
    /// The interrupt priority mask, bits 10–8.
    pub const IPL: u16 = 0x0700;
    /// Supervisor state.
    pub const S: u16 = 0x2000;
    /// Trace: take a trace exception after each instruction.
    pub const T: u16 = 0x8000;
    /// Every bit the 68000 implements.
    ///
    /// Bits 11, 12 and 14 have no storage and read as zero; the 68020's **M**
    /// bit is one of them.
    pub const IMPLEMENTED: u16 = T | S | IPL | CCR;
    /// The 68020's master/interrupt state: with **S** set, whether `A7` is the
    /// master stack pointer (MC68020UM §1.3.2).
    pub const M: u16 = 0x1000;
    /// The 68020's second trace bit, **T0**: trace on a change of flow. **T**
    /// is the 68020's **T1** (MC68020UM §6.1.7).
    pub const T0: u16 = 0x4000;

    /// Every bit a given model implements: the 68010 has the 68000's, the
    /// 68020 adds **M** and **T0**.
    #[must_use]
    pub const fn implemented(model: super::Model) -> u16 {
        if model.has_020() {
            IMPLEMENTED | M | T0
        } else {
            IMPLEMENTED
        }
    }
}

/// The exception vector numbers the family defines.
///
/// A vector's address is four times its number, and the table starts at zero —
/// which on a 68000 cannot be moved, because there is no vector base register
/// (MC68000UM §6.1). From the 68010 on it starts at `VBR`.
pub mod vector {
    /// Vector 0: the initial supervisor stack pointer.
    pub const RESET_SSP: u8 = 0;
    /// Vector 1: the initial program counter.
    pub const RESET_PC: u8 = 1;
    /// Vector 2: bus error — an access the hardware refused.
    pub const BUS_ERROR: u8 = 2;
    /// Vector 3: address error — a word or long access to an odd address.
    pub const ADDRESS_ERROR: u8 = 3;
    /// Vector 4: illegal instruction.
    pub const ILLEGAL: u8 = 4;
    /// Vector 5: divide by zero.
    pub const DIVIDE_BY_ZERO: u8 = 5;
    /// Vector 6: `CHK` found the register outside its bounds.
    pub const CHK: u8 = 6;
    /// Vector 7: `TRAPV` with **V** set.
    pub const TRAPV: u8 = 7;
    /// Vector 8: privilege violation.
    pub const PRIVILEGE: u8 = 8;
    /// Vector 9: trace.
    pub const TRACE: u8 = 9;
    /// Vector 10: an unimplemented `$Axxx` instruction.
    pub const LINE_A: u8 = 10;
    /// Vector 11: an unimplemented `$Fxxx` instruction.
    pub const LINE_F: u8 = 11;
    /// Vector 14: format error — an `RTE` found a frame format it cannot use
    /// (68010 on), or `CALLM`/`RTM` a descriptor it does not recognise.
    pub const FORMAT_ERROR: u8 = 14;
    /// Vector 15: uninitialized interrupt vector.
    pub const UNINITIALIZED: u8 = 15;
    /// Vector 24: spurious interrupt — no device answered the acknowledge.
    pub const SPURIOUS: u8 = 24;
    /// Vectors 25–31: the autovectors, one per interrupt level.
    ///
    /// The level is added: level 1 uses vector 25. Vector 24 is the spurious
    /// slot immediately below.
    pub const AUTOVECTOR_BASE: u8 = 24;
    /// Vectors 32–47: the `TRAP #0`–`TRAP #15` family.
    pub const TRAP_BASE: u8 = 32;
    /// Vector 48: the coprocessor branched or set on an unordered condition
    /// (MC68030UM Table 8-1).
    pub const FP_BSUN: u8 = 48;
    /// Vector 49: an inexact floating-point result.
    pub const FP_INEXACT: u8 = 49;
    /// Vector 50: a floating-point divide by zero.
    pub const FP_DIVIDE_BY_ZERO: u8 = 50;
    /// Vector 51: floating-point underflow.
    pub const FP_UNDERFLOW: u8 = 51;
    /// Vector 52: a floating-point operand error.
    pub const FP_OPERAND_ERROR: u8 = 52;
    /// Vector 53: floating-point overflow.
    pub const FP_OVERFLOW: u8 = 53;
    /// Vector 54: a signalling not-a-number was an operand.
    pub const FP_SIGNALING_NAN: u8 = 54;
    /// Vector 55: an operand whose data format the 68040 leaves to software
    /// — a denormalized or unnormalized number, or packed decimal
    /// (M68040UM Table 9-9, §9.6.2).
    pub const FP_UNSUPPORTED_TYPE: u8 = 55;
    /// Vector 56: an MMU configuration error — a `PMOVE` that loaded `TC`,
    /// `CRP` or `SRP` with a value the unit cannot use (MC68030UM §9.7.5.3).
    pub const MMU_CONFIG: u8 = 56;
}

/// The architectural register file, as a debugger or a test vector sees it.
///
/// `a[7]` is whichever stack pointer the **S** bit currently selects, and is
/// always equal to [`Regs::ssp`] in supervisor state or [`Regs::usp`] in user
/// state — the 68000 has two physical `A7`s and one name for them. A 68020 has
/// three: with **S** and **M** both set, `a[7]` is [`Regs::msp`].
///
/// The control registers a 68000 does not have read as zero on one, and
/// writing them there changes nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// The eight data registers.
    pub d: [u32; 8],
    /// The eight address registers; `a[7]` is the active stack pointer.
    pub a: [u32; 8],
    /// The user stack pointer.
    pub usp: u32,
    /// The supervisor stack pointer — the *interrupt* stack pointer, `ISP`,
    /// on a 68020, which is the one reset loads (MC68020UM §6.1.1).
    pub ssp: u32,
    /// The program counter: the address of the word in `prefetch[0]`.
    pub pc: u32,
    /// The status register. See [`flags`].
    pub sr: u16,
    /// The two-word instruction prefetch queue.
    pub prefetch: [u16; 2],
    /// The 68020's master stack pointer.
    pub msp: u32,
    /// The vector base register (68010 on).
    pub vbr: u32,
    /// The source function code, three bits (68010 on).
    pub sfc: u8,
    /// The destination function code, three bits (68010 on).
    pub dfc: u8,
    /// The 68020's cache control register; only **E** and **F** are kept.
    pub cacr: u32,
    /// The 68020's cache address register.
    pub caar: u32,
    /// The translation control register: thirty-two bits on a 68030
    /// (MC68030UM Figure 9-36), sixteen on a 68040, where only **E** and
    /// **P** exist and the rest read as zero (M68040UM Figure 3-4).
    pub tc: u32,
    /// The 68030's CPU root pointer, all sixty-four bits. The 68040's
    /// user-mode root pointer is [`Regs::urp`], which is a plain address.
    pub crp: u64,
    /// The supervisor root pointer: a 64-bit descriptor on a 68030, and on a
    /// 68040 a 32-bit address in the low half (M68040UM Figure 3-3).
    pub srp: u64,
    /// The 68040's user root pointer, the translation table root for user
    /// accesses (M68040UM §3.1.1). Bits 8–0 must be zero.
    pub urp: u32,
    /// The 68030's transparent translation registers, `TT0` then `TT1` —
    /// `AC0` and `AC1` on an MC68EC030.
    pub tt: [u32; 2],
    /// The 68040's *instruction* transparent translation registers, `ITT0`
    /// then `ITT1` — `IACR0`/`IACR1` on an MC68EC040. A different layout from
    /// the 68030's (M68040UM Figure 3-5).
    pub itt: [u32; 2],
    /// The 68040's *data* transparent translation registers, `DTT0` then
    /// `DTT1` — `DACR0`/`DACR1` on an MC68EC040.
    pub dtt: [u32; 2],
    /// The MMU status register — `ACUSR` on an MC68EC030.
    ///
    /// Sixteen bits on a 68030, where the top half is always zero, and
    /// thirty-two on a 68040, where the top twenty carry a physical address
    /// (M68040UM Figure 3-6).
    pub mmusr: u32,
    /// The coprocessor's eight floating-point data registers, always in the
    /// extended format.
    ///
    /// Eighty bits each, so they are here rather than in [`Reg`], which
    /// hands out at most thirty-two.
    pub fp: [F80; 8],
    /// The floating-point control register: an enable byte and a mode byte.
    pub fpcr: u32,
    /// The floating-point status register: condition codes, a quotient byte,
    /// an exception byte and an accrued byte.
    pub fpsr: u32,
    /// The address of the floating-point instruction a trap handler should
    /// look at.
    pub fpiar: u32,
}

impl Regs {
    /// Whether the core is in supervisor state.
    #[must_use]
    pub const fn supervisor(&self) -> bool {
        self.sr & flags::S != 0
    }

    /// Whether the **M** bit selects the master stack pointer — meaningful
    /// only in supervisor state on a 68020.
    #[must_use]
    pub const fn master(&self) -> bool {
        self.sr & (flags::S | flags::M) == flags::S | flags::M
    }

    /// Whether a status flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u16) -> bool {
        self.sr & mask != 0
    }

    /// The condition code register: the low byte of `SR`.
    #[must_use]
    pub const fn ccr(&self) -> u8 {
        (self.sr & flags::CCR) as u8
    }

    /// The interrupt priority mask, 0–7.
    #[must_use]
    pub const fn ipl_mask(&self) -> u8 {
        ((self.sr & flags::IPL) >> 8) as u8
    }
}

impl fmt::Display for Regs {
    /// The shape a trace log wants: the two register files, then `SR` decoded.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, value) in self.d.iter().enumerate() {
            write!(f, "D{i}:{value:08x} ")?;
        }
        for (i, value) in self.a.iter().enumerate() {
            write!(f, "A{i}:{value:08x} ")?;
        }
        write!(f, "PC:{:08x} SR:{:04x} [", self.pc, self.sr)?;
        for (mask, name) in [
            (flags::T, 'T'),
            (flags::S, 'S'),
            (flags::X, 'X'),
            (flags::N, 'N'),
            (flags::Z, 'Z'),
            (flags::V, 'V'),
            (flags::C, 'C'),
        ] {
            f.write_char(if self.flag(mask) { name } else { '-' })?;
        }
        write!(f, "] I{}", self.ipl_mask())
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// A data register, `D0`–`D7`.
    D(u8),
    /// An address register, `A0`–`A7`.
    A(u8),
    /// The user stack pointer.
    Usp,
    /// The supervisor stack pointer; the interrupt stack pointer on a 68020.
    Ssp,
    /// The program counter.
    Pc,
    /// The status register.
    Sr,
    /// The 68020's master stack pointer.
    Msp,
    /// The vector base register.
    Vbr,
    /// The source function code register.
    Sfc,
    /// The destination function code register.
    Dfc,
    /// The 68020's cache control register.
    Cacr,
    /// The 68020's cache address register.
    Caar,
    /// The 68030's translation control register.
    Tc,
    /// Transparent translation register 0 — `AC0` on an MC68EC030.
    Tt0,
    /// Transparent translation register 1 — `AC1`.
    Tt1,
    /// The 68040's instruction transparent translation register 0 — `IACR0`
    /// on an MC68EC040.
    Itt0,
    /// The 68040's instruction transparent translation register 1 — `IACR1`.
    Itt1,
    /// The 68040's data transparent translation register 0 — `DACR0` on an
    /// MC68EC040.
    Dtt0,
    /// The 68040's data transparent translation register 1 — `DACR1`.
    Dtt1,
    /// The 68040's user root pointer.
    Urp,
    /// The 68040's supervisor root pointer, which unlike the 68030's is a
    /// plain 32-bit address.
    Srp,
    /// The MMU status register — `ACUSR` on an MC68EC030.
    Mmusr,
    /// The high half of the CPU root pointer: **L/U**, **LIMIT** and **DT**.
    ///
    /// Two names for one register because a root pointer is sixty-four bits
    /// wide and [`Reg`] hands out thirty-two; [`Regs::crp`] is the whole
    /// thing, and this is what a debugger that works in machine words sees.
    CrpHi,
    /// The low half of the CPU root pointer: the table address.
    CrpLo,
    /// The high half of the supervisor root pointer.
    SrpHi,
    /// The low half of the supervisor root pointer.
    SrpLo,
    /// The floating-point control register.
    Fpcr,
    /// The floating-point status register.
    Fpsr,
    /// The floating-point instruction address register.
    Fpiar,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[
        Reg::D(0),
        Reg::D(1),
        Reg::D(2),
        Reg::D(3),
        Reg::D(4),
        Reg::D(5),
        Reg::D(6),
        Reg::D(7),
        Reg::A(0),
        Reg::A(1),
        Reg::A(2),
        Reg::A(3),
        Reg::A(4),
        Reg::A(5),
        Reg::A(6),
        Reg::A(7),
        Reg::Usp,
        Reg::Ssp,
        Reg::Pc,
        Reg::Sr,
    ];

    /// The 68010's additions to [`Reg::ALL`].
    pub const M68010: &'static [Reg] = &[Reg::Vbr, Reg::Sfc, Reg::Dfc];

    /// The 68020's additions to the 68010's.
    pub const M68020: &'static [Reg] = &[Reg::Msp, Reg::Cacr, Reg::Caar];

    /// The MC68EC030's additions to the 68020's: the access control unit,
    /// which is the 68030's transparent translation registers under another
    /// name (MC68EC030UM §9.3).
    pub const M68EC030: &'static [Reg] = &[Reg::Tt0, Reg::Tt1, Reg::Mmusr];

    /// A floating-point coprocessor's three control registers. The eight
    /// data registers are eighty bits wide and live in [`Regs::fp`].
    pub const FPU: &'static [Reg] = &[Reg::Fpcr, Reg::Fpsr, Reg::Fpiar];

    /// The full MC68030's additions to those: the paged unit's own
    /// registers.
    pub const M68030: &'static [Reg] = &[Reg::Tc, Reg::CrpHi, Reg::CrpLo, Reg::SrpHi, Reg::SrpLo];

    /// Every 68040 package's additions to the 68020's: the four transparent
    /// translation registers, which on an MC68EC040 are the access control
    /// registers and are all it has (M68040UM Appendix B).
    pub const M68EC040: &'static [Reg] = &[Reg::Itt0, Reg::Itt1, Reg::Dtt0, Reg::Dtt1];

    /// What the parts with the 68040's paged unit add to those.
    pub const M68040: &'static [Reg] = &[Reg::Tc, Reg::Urp, Reg::Srp, Reg::Mmusr];

    /// Every register `model` has, in the order a debugger should list them.
    #[must_use]
    pub fn all_for(model: Model) -> Vec<Reg> {
        let mut out = Reg::ALL.to_vec();
        if model.has_010() {
            out.extend_from_slice(Reg::M68010);
        }
        if model.has_020() {
            out.extend_from_slice(Reg::M68020);
            if model.has_040() {
                // The 68040 has no cache address register (M68000PRM §6,
                // *MOVEC*: `$802` is "for the MC68020 and MC68030 only").
                out.retain(|reg| *reg != Reg::Caar);
            }
        }
        if model.has_030() {
            out.extend_from_slice(Reg::M68EC030);
        }
        if model.has_mmu() {
            out.extend_from_slice(Reg::M68030);
        }
        if model.has_040() {
            out.extend_from_slice(Reg::M68EC040);
        }
        if model.has_mmu_040() {
            out.extend_from_slice(Reg::M68040);
        }
        out
    }

    /// Every register a core with this configuration has.
    ///
    /// The three floating-point control registers are only there when a
    /// coprocessor is; the eight data registers are eighty bits wide and are
    /// in [`Regs::fp`] rather than here.
    #[must_use]
    pub fn all_for_config(cfg: Config) -> Vec<Reg> {
        let mut out = Reg::all_for(cfg.model);
        if cfg.fpu.present() {
            out.extend_from_slice(Reg::FPU);
        }
        out
    }

    /// How wide the register is.
    #[must_use]
    /// The 68030's `MMUSR` is sixteen bits and the 68040's is thirty-two
    /// (MC68030UM Figure 9-38; M68040UM Figure 3-6). One name, so one width:
    /// the wider, whose top half is always zero on the narrower part.
    pub const fn width(self) -> Width {
        match self {
            Reg::Sr => Width::U16,
            Reg::Sfc | Reg::Dfc => Width::U8,
            _ => Width::U32,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u32 {
        match self {
            Reg::D(n) => regs.d[(n & 7) as usize],
            Reg::A(n) => regs.a[(n & 7) as usize],
            Reg::Usp => regs.usp,
            Reg::Ssp => regs.ssp,
            Reg::Pc => regs.pc,
            Reg::Sr => regs.sr as u32,
            Reg::Msp => regs.msp,
            Reg::Vbr => regs.vbr,
            Reg::Sfc => regs.sfc as u32,
            Reg::Dfc => regs.dfc as u32,
            Reg::Cacr => regs.cacr,
            Reg::Caar => regs.caar,
            Reg::Tc => regs.tc,
            Reg::Tt0 => regs.tt[0],
            Reg::Tt1 => regs.tt[1],
            Reg::Itt0 => regs.itt[0],
            Reg::Itt1 => regs.itt[1],
            Reg::Dtt0 => regs.dtt[0],
            Reg::Dtt1 => regs.dtt[1],
            Reg::Urp => regs.urp,
            Reg::Srp => regs.srp as u32,
            Reg::Mmusr => regs.mmusr,
            Reg::CrpHi => (regs.crp >> 32) as u32,
            Reg::CrpLo => regs.crp as u32,
            Reg::SrpHi => (regs.srp >> 32) as u32,
            Reg::SrpLo => regs.srp as u32,
            Reg::Fpcr => regs.fpcr,
            Reg::Fpsr => regs.fpsr,
            Reg::Fpiar => regs.fpiar,
        }
    }

    /// Write this register into a register file, truncating to its width.
    ///
    /// Writing `A7` writes whichever bank is active, and writing `USP`, `SSP`
    /// or `MSP` writes that bank whether or not it is active — which is what a
    /// debugger showing all of them needs.
    pub const fn set(self, regs: &mut Regs, value: u32) {
        match self {
            Reg::D(n) => regs.d[(n & 7) as usize] = value,
            Reg::A(n) => {
                let n = (n & 7) as usize;
                regs.a[n] = value;
                if n == 7 {
                    if regs.master() {
                        regs.msp = value;
                    } else if regs.supervisor() {
                        regs.ssp = value;
                    } else {
                        regs.usp = value;
                    }
                }
            }
            Reg::Usp => {
                regs.usp = value;
                if !regs.supervisor() {
                    regs.a[7] = value;
                }
            }
            Reg::Ssp => {
                regs.ssp = value;
                if regs.supervisor() && !regs.master() {
                    regs.a[7] = value;
                }
            }
            Reg::Msp => {
                regs.msp = value;
                if regs.master() {
                    regs.a[7] = value;
                }
            }
            Reg::Pc => regs.pc = value,
            Reg::Sr => regs.sr = value as u16,
            Reg::Vbr => regs.vbr = value,
            Reg::Sfc => regs.sfc = (value & 7) as u8,
            Reg::Dfc => regs.dfc = (value & 7) as u8,
            Reg::Cacr => regs.cacr = value,
            Reg::Caar => regs.caar = value,
            Reg::Tc => regs.tc = value,
            Reg::Tt0 => regs.tt[0] = value,
            Reg::Tt1 => regs.tt[1] = value,
            Reg::Itt0 => regs.itt[0] = value,
            Reg::Itt1 => regs.itt[1] = value,
            Reg::Dtt0 => regs.dtt[0] = value,
            Reg::Dtt1 => regs.dtt[1] = value,
            Reg::Urp => regs.urp = value,
            Reg::Srp => regs.srp = (regs.srp & 0xffff_ffff_0000_0000) | value as u64,
            Reg::Mmusr => regs.mmusr = value,
            Reg::CrpHi => regs.crp = (regs.crp & 0xffff_ffff) | ((value as u64) << 32),
            Reg::CrpLo => regs.crp = (regs.crp & 0xffff_ffff_0000_0000) | value as u64,
            Reg::SrpHi => regs.srp = (regs.srp & 0xffff_ffff) | ((value as u64) << 32),
            Reg::SrpLo => regs.srp = (regs.srp & 0xffff_ffff_0000_0000) | value as u64,
            Reg::Fpcr => regs.fpcr = value,
            Reg::Fpsr => regs.fpsr = value,
            Reg::Fpiar => regs.fpiar = value,
        }
    }

    /// Look a register up by name, as gdb and the monitor spell it.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        let bytes = name.as_bytes();
        match (bytes.first(), bytes.len()) {
            (Some(b'd' | b'D'), 2) if bytes[1].is_ascii_digit() && bytes[1] <= b'7' => {
                Some(Reg::D(bytes[1] - b'0'))
            }
            (Some(b'a' | b'A'), 2) if bytes[1].is_ascii_digit() && bytes[1] <= b'7' => {
                Some(Reg::A(bytes[1] - b'0'))
            }
            _ => match name {
                "usp" => Some(Reg::Usp),
                "ssp" | "sp" | "isp" => Some(Reg::Ssp),
                "pc" => Some(Reg::Pc),
                "sr" => Some(Reg::Sr),
                "msp" => Some(Reg::Msp),
                "vbr" => Some(Reg::Vbr),
                "sfc" => Some(Reg::Sfc),
                "dfc" => Some(Reg::Dfc),
                "cacr" => Some(Reg::Cacr),
                "caar" => Some(Reg::Caar),
                "tc" => Some(Reg::Tc),
                "tt0" | "ac0" => Some(Reg::Tt0),
                "tt1" | "ac1" => Some(Reg::Tt1),
                "itt0" | "iacr0" => Some(Reg::Itt0),
                "itt1" | "iacr1" => Some(Reg::Itt1),
                "dtt0" | "dacr0" => Some(Reg::Dtt0),
                "dtt1" | "dacr1" => Some(Reg::Dtt1),
                "urp" => Some(Reg::Urp),
                "srp" => Some(Reg::Srp),
                "mmusr" | "acusr" => Some(Reg::Mmusr),
                "crph" => Some(Reg::CrpHi),
                "crpl" => Some(Reg::CrpLo),
                "srph" => Some(Reg::SrpHi),
                "srpl" => Some(Reg::SrpLo),
                "fpcr" => Some(Reg::Fpcr),
                "fpsr" => Some(Reg::Fpsr),
                "fpiar" => Some(Reg::Fpiar),
                _ => None,
            },
        }
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reg::D(n) => write!(f, "d{n}"),
            Reg::A(n) => write!(f, "a{n}"),
            Reg::Usp => f.write_str("usp"),
            Reg::Ssp => f.write_str("ssp"),
            Reg::Pc => f.write_str("pc"),
            Reg::Sr => f.write_str("sr"),
            Reg::Msp => f.write_str("msp"),
            Reg::Vbr => f.write_str("vbr"),
            Reg::Sfc => f.write_str("sfc"),
            Reg::Dfc => f.write_str("dfc"),
            Reg::Cacr => f.write_str("cacr"),
            Reg::Caar => f.write_str("caar"),
            Reg::Tc => f.write_str("tc"),
            Reg::Tt0 => f.write_str("tt0"),
            Reg::Tt1 => f.write_str("tt1"),
            Reg::Itt0 => f.write_str("itt0"),
            Reg::Itt1 => f.write_str("itt1"),
            Reg::Dtt0 => f.write_str("dtt0"),
            Reg::Dtt1 => f.write_str("dtt1"),
            Reg::Urp => f.write_str("urp"),
            Reg::Srp => f.write_str("srp"),
            Reg::Mmusr => f.write_str("mmusr"),
            Reg::CrpHi => f.write_str("crph"),
            Reg::CrpLo => f.write_str("crpl"),
            Reg::SrpHi => f.write_str("srph"),
            Reg::SrpLo => f.write_str("srpl"),
            Reg::Fpcr => f.write_str("fpcr"),
            Reg::Fpsr => f.write_str("fpsr"),
            Reg::Fpiar => f.write_str("fpiar"),
        }
    }
}

/// How this particular part differs from the generic 68000.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run an Amiga and a Mega Drive at the same time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
    /// Which member of the family this is.
    pub model: Model,
    /// Which floating-point coprocessor is attached, if any.
    ///
    /// A property of the *board* rather than of the part: the same 68020 is
    /// a 68020 with a 68881 and a 68020 without one, and the difference is
    /// visible in the opcode map. Only a processor with the F-line
    /// coprocessor interface can have one, which begins at the 68020
    /// (MC68020UM §7).
    pub fpu: Coprocessor,
}

impl Config {
    /// A plain MC68000.
    pub const MC68000: Config = Config {
        requester: RequesterId::ANONYMOUS,
        model: Model::M68000,
        fpu: Coprocessor::None,
    };

    /// An MC68010.
    pub const MC68010: Config = Config::MC68000.with_model(Model::M68010);

    /// An MC68020.
    pub const MC68020: Config = Config::MC68000.with_model(Model::M68020);

    /// An MC68EC020: a 68020 with 24 address pins.
    pub const MC68EC020: Config = Config::MC68000.with_model(Model::M68EC020);

    /// An MC68030.
    pub const MC68030: Config = Config::MC68000.with_model(Model::M68030);

    /// An MC68EC030: a 68030 with no paged memory management unit.
    pub const MC68EC030: Config = Config::MC68000.with_model(Model::M68EC030);

    /// An MC68040, whose floating-point unit is on the chip.
    pub const MC68040: Config = Config::MC68000
        .with_model(Model::M68040)
        .with_fpu(Coprocessor::M68040);

    /// An MC68LC040: a 68040 with no floating-point unit.
    pub const MC68LC040: Config = Config::MC68000.with_model(Model::M68LC040);

    /// An MC68EC040: a 68040 with neither a floating-point unit nor a paged
    /// memory management unit.
    pub const MC68EC040: Config = Config::MC68000.with_model(Model::M68EC040);

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Same configuration, as a different model.
    #[must_use]
    pub const fn with_model(mut self, model: Model) -> Self {
        self.model = model;
        self
    }

    /// Same configuration, with a floating-point coprocessor attached.
    #[must_use]
    pub const fn with_fpu(mut self, fpu: Coprocessor) -> Self {
        self.fpu = fpu;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::MC68000
    }
}

/// No interrupt vector has been supplied by an acknowledging device.
const NO_VECTOR: u16 = 0x100;

/// The interrupt and reset pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device raising
/// an interrupt from inside a write the CPU itself issued would otherwise
/// re-enter the CPU's own critical section, which is a deadlock under
/// `native-std` and a panic under `single`. A pin that is one atomic store
/// needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug)]
pub(crate) struct Lines {
    /// The encoded level on IPL0–IPL2, 0 (none) to 7 (non-maskable).
    ipl: AtomicU8,
    /// A vector supplied by an interrupt controller, or [`NO_VECTOR`] for the
    /// autovector the 68000 uses when `VPA` is asserted.
    vector: AtomicU16,
    /// A transition to level seven, latched until it is serviced.
    ///
    /// Level seven is edge-triggered, so the level alone is not enough to know
    /// whether to take it.
    level_seven: AtomicBool,
    /// How many times the `RESET` instruction has pulsed the reset line.
    ///
    /// A counter rather than a wire because `RESET` resets *peripherals*, not
    /// the processor, and what is on the other end is the machine's business.
    resets: AtomicU32,
    /// A reset asked for by the `reset` pin, latched until the next step folds
    /// it into the execution state.
    ///
    /// A latch rather than a write into `State::reset_pending`, because a wire
    /// is driven from inside whatever device changed it — often from inside an
    /// access this very core issued — and reaching for the session lock there
    /// would re-enter the core's own critical section (`ROADMAP.md` §4.7).
    reset: AtomicBool,
    /// What answers the interrupt-acknowledge cycle, if any controller does.
    ///
    /// A **list**, not a slot, and not one slot per `IPL` pin either: a 68000
    /// broadcasts the level it is acknowledging on A3-A1 and every controller
    /// in CPU space decides for itself whether it is the one being asked. So
    /// the cycle goes to each attached controller in turn, carrying the level
    /// ([`IntAckCycle::at_level`]), until one stops declining — which is how a
    /// board with two vectoring controllers on different `IPL` pins works, and
    /// what could not be said before the cycle carried an argument.
    ///
    /// Weak references, behind a leaf lock released before each outward call:
    /// the machine owns the devices, a wire merely refers to them (§4.3), and
    /// a controller answering is free to take its own locks.
    acks: IntAckHandlers,
}

impl Default for Lines {
    fn default() -> Lines {
        Lines {
            ipl: AtomicU8::new(0),
            // Autovectoring, not vector 0 — which is the reset stack pointer,
            // and would send the first interrupt somewhere very strange.
            vector: AtomicU16::new(NO_VECTOR),
            level_seven: AtomicBool::new(false),
            resets: AtomicU32::new(0),
            reset: AtomicBool::new(false),
            acks: IntAckHandlers::new(),
        }
    }
}

impl Lines {
    fn set_ipl(&self, level: u8) {
        let level = level.min(7);
        let previous = self.ipl.swap(level, Ordering::AcqRel);
        if level == 7 && previous != 7 {
            self.level_seven.store(true, Ordering::Release);
        }
    }

    /// Consume a latched transition to level seven, reporting whether there
    /// was one.
    pub(crate) fn take_level_seven(&self) -> bool {
        self.level_seven.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn ipl(&self) -> u8 {
        self.ipl.load(Ordering::Acquire)
    }

    fn set_vector(&self, vector: Option<u8>) {
        self.vector
            .store(vector.map_or(NO_VECTOR, u16::from), Ordering::Release);
    }

    pub(crate) fn take_vector(&self) -> Option<u8> {
        match self.vector.swap(NO_VECTOR, Ordering::AcqRel) {
            NO_VECTOR => None,
            other => Some(other as u8),
        }
    }

    /// Latch a reset request from the `reset` pin.
    fn request_reset_pin(&self) {
        self.reset.store(true, Ordering::Release);
    }

    /// Consume that latch, reporting whether a reset was owed.
    fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    /// Add a controller to those that answer the interrupt-acknowledge cycle.
    fn attach_ack(&self, ack: Weak<dyn IntAck>) {
        self.acks.attach(ack);
    }

    /// Run the acknowledge cycle for `level`: the vector a controller supplies,
    /// or `None` for the autovector.
    ///
    /// The three things a 68000 acknowledge cycle can end in, and what each
    /// means here:
    ///
    /// - a controller drives a vector number and `DTACK`
    ///   ([`IntAckResponse::Vector`]) — that vector;
    /// - a controller recognises the level and asserts `VPA`
    ///   ([`IntAckResponse::Autovector`]) — `None`, and the controllers behind
    ///   it are never asked, because the cycle is over;
    /// - nobody claims the level ([`IntAckResponse::Declined`]) — also `None`.
    ///   Hardware would leave the cycle to be terminated by the board, whose
    ///   address decode asserts `VPA` on most 68000 machines and `BERR` (hence
    ///   [`vector::SPURIOUS`]) on the rest. The board's decode is not modelled,
    ///   and autovectoring is what the common one does.
    ///
    /// An armed [`set_interrupt_vector`](M68k::set_interrupt_vector) is checked
    /// first, so a test or a host driving the core by hand still works.
    ///
    /// No lock is held across the outward call: the re-entrancy contract
    /// forbids holding one across a call into another device (§4.7).
    pub(crate) fn acknowledge(&self, level: u8) -> Option<u8> {
        if let Some(armed) = self.take_vector() {
            return Some(armed);
        }
        match self.acks.run(IntAckCycle::at_level(level)) {
            IntAckResponse::Vector(vector) => Some(vector as u8),
            IntAckResponse::Autovector | IntAckResponse::Declined => None,
        }
    }

    pub(crate) fn pulse_reset(&self) {
        self.resets.fetch_add(1, Ordering::AcqRel);
    }

    fn resets(&self) -> u32 {
        self.resets.load(Ordering::Acquire)
    }

    fn snapshot(&self) -> (u8, u16, bool, u32) {
        (
            self.ipl(),
            self.vector.load(Ordering::Acquire),
            self.level_seven.load(Ordering::Acquire),
            self.resets(),
        )
    }

    fn restore(&self, (ipl, vector, level_seven, resets): (u8, u16, bool, u32)) {
        self.ipl.store(ipl, Ordering::Release);
        self.vector.store(vector, Ordering::Release);
        self.level_seven.store(level_seven, Ordering::Release);
        self.resets.store(resets, Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// A 680x0 core: a 68000, or the 68010 or 68020 its [`Model`] names.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank, rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own `DEVICE`-ranked
/// locks, which drive `WIRE`-ranked lines. The ladder runs in the direction
/// calls travel.
///
/// The interrupt pins are *not* under that lock: they are atomics, so a device
/// raising an interrupt from inside a write the CPU itself issued cannot
/// re-enter the CPU's own critical section.
#[derive(Debug)]
pub struct M68k {
    lines: Arc<Lines>,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    ///
    /// The `requester` property sets it at construction; the machine layer
    /// overrides it in [`Instance::bind`](crate::machine::Instance::bind),
    /// because a machine allocates one per initiator (`ROADMAP.md` §4.4).
    requester: AtomicU32,
    /// Which processor this is. Fixed at construction.
    model: Model,
    /// Which coprocessor answers the F line. Fixed at construction.
    fpu: Coprocessor,
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
///
/// One [`InterruptPins`] for all three `IPL` lines, not one per line: they
/// carry an encoded *level* rather than three independent requests, so the pin
/// object has to see all three to know what the level is.
#[derive(Debug, Default)]
struct Pins {
    ipl: Option<Arc<InterruptPins>>,
    reset: Option<Arc<ResetPin>>,
}

impl M68k {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](M68k::attach_space) and [`Device::realize`]. The
    /// first [`step`](M68k::step) runs the reset sequence, which is where
    /// vectors 0 and 1 are read.
    #[must_use]
    pub fn new(cfg: Config) -> M68k {
        M68k {
            lines: Arc::new(Lines::default()),
            requester: AtomicU32::new(cfg.requester.0),
            model: cfg.model,
            fpu: cfg.fpu,
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(cfg.model),
                    space: None,
                },
            ),
            pins: sync::Mutex::new(Pins::default()),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property nothing here accepts was given — a typo'd property that
    /// was silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<M68k> {
        let mut r = props.reader();
        let requester = r.or_range("requester", 0u64, 0..=u64::from(u32::MAX))?;
        // Accepted and ignored: there is one engine until phase 5, and a
        // machine file that names it should not need editing when the second
        // one lands.
        let _engine = r.or_enum("engine", "interp", &["interp"])?;
        let model = r.or_enum("model", Model::M68000.name(), &MODEL_NAMES)?;
        // A 68040's floating-point unit is part of the *part*, not of the
        // board: an MC68040 has one and an MC68LC040 does not, and those are
        // different order codes. So the property's default follows the model
        // rather than being `none` everywhere.
        let default_fpu = if Model::from_name(model).is_some_and(Model::has_onchip_fpu) {
            Coprocessor::M68040.name()
        } else {
            Coprocessor::None.name()
        };
        let fpu = r.or_enum("fpu", default_fpu, &FPU_NAMES)?;
        r.finish()?;
        let model = Model::from_name(model).unwrap_or_default();
        let fpu = Coprocessor::from_name(fpu).unwrap_or_default();
        if fpu.is_onchip_040() && !model.has_onchip_fpu() {
            return Err(Error::Config {
                at: String::from("cpu.m68k"),
                message: alloc::format!(
                    "the `68040` floating-point unit is on the MC68040's own chip; \
                     a {model} does not have one"
                ),
            });
        }
        if model.has_onchip_fpu() && !fpu.present() {
            return Err(Error::Config {
                at: String::from("cpu.m68k"),
                message: String::from(
                    "an MC68040 has a floating-point unit on the chip and no way to \
                     switch it off; the part without one is the `68lc040`",
                ),
            });
        }
        // A coprocessor answers the F line, and the F-line coprocessor
        // interface arrived with the 68020 (MC68020UM §7). A 68881 can be
        // wired to a 68000 as an ordinary peripheral, but then it is not a
        // coprocessor and its instructions do not exist.
        if fpu.present() && !fpu.is_onchip_040() && !model.has_coprocessor_interface() {
            let why = if model.has_040() {
                "the 68040 dropped it and answers the F line itself"
            } else {
                "the F-line interface starts at the 68020"
            };
            return Err(Error::Config {
                at: String::from("cpu.m68k"),
                message: alloc::format!(
                    "a {fpu} is a coprocessor and a {model} has no coprocessor interface; {why}"
                ),
            });
        }
        Ok(M68k::new(
            Config::default()
                .with_requester(RequesterId(requester as u32))
                .with_model(model)
                .with_fpu(fpu),
        ))
    }

    /// This core's configuration.
    ///
    /// The requester id lives in an atomic because the machine layer assigns
    /// it at bind time — so this is built rather than stored.
    #[must_use]
    pub fn config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            model: self.model,
            fpu: self.fpu,
        }
    }

    /// Which processor this core is.
    #[must_use]
    pub fn model(&self) -> Model {
        self.model
    }

    /// Give the core the identity its accesses travel under.
    ///
    /// The machine layer calls this from `bind`; a crate driving the core
    /// directly usually sets the `requester` property at construction instead.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Give the core the address space it executes from.
    ///
    /// The space must be big-endian, or every word the core reads is
    /// byte-swapped — see the module documentation.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        let state = self.session.lock().state;
        Regs {
            d: state.d,
            a: state.a,
            usp: state.usp(),
            ssp: state.ssp(),
            pc: state.pc,
            sr: state.sr,
            prefetch: state.prefetch,
            msp: state.sp(Bank::Master),
            vbr: state.vbr,
            sfc: state.sfc,
            dfc: state.dfc,
            cacr: state.cacr,
            caar: state.caar,
            // Three names are shared between the two memory management
            // units because the registers are: `TC`, `SRP` and `MMUSR` exist
            // on both parts, with different widths and completely different
            // layouts. A core is one model, so only one of the two can ever
            // have written them.
            tc: if state.model.has_040() {
                u32::from(state.mmu040.tcr)
            } else {
                state.mmu.tc
            },
            crp: state.mmu.crp,
            srp: if state.model.has_040() {
                u64::from(state.mmu040.srp)
            } else {
                state.mmu.srp
            },
            urp: state.mmu040.urp,
            tt: state.mmu.tt,
            itt: state.mmu040.itt,
            dtt: state.mmu040.dtt,
            mmusr: if state.model.has_040() {
                state.mmu040.mmusr
            } else {
                u32::from(state.mmu.mmusr)
            },
            fp: state.fpu.fp,
            fpcr: state.fpu.fpcr,
            fpsr: state.fpu.fpsr,
            fpiar: state.fpu.fpiar,
        }
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    ///
    /// [`Regs::usp`], [`Regs::ssp`] and [`Regs::msp`] are authoritative:
    /// `a[7]` is set from whichever `regs.sr` selects, so a caller cannot
    /// leave the banks disagreeing. Bits and registers the model does not
    /// have are dropped.
    pub fn set_regs(&self, regs: Regs) {
        let mut session = self.session.lock();
        let state = &mut session.state;
        state.d = regs.d;
        state.a = regs.a;
        state.sr = regs.sr & state.sr_mask();
        state.banks = [regs.usp, regs.ssp, regs.msp];
        state.a[7] = state.banks[state.bank_of(state.sr) as usize];
        state.pc = regs.pc;
        state.prefetch = regs.prefetch;
        // A register file placed from outside is whole: nothing is owed to a
        // fetch that failed before it arrived.
        state.poison = [None, None];
        if self.model.has_010() {
            state.vbr = regs.vbr;
            state.sfc = regs.sfc & 7;
            state.dfc = regs.dfc & 7;
        }
        if self.model.has_020() {
            state.cacr = regs.cacr & state.cacr_mask();
            state.caar = regs.caar;
        } else {
            state.banks[Bank::Master as usize] = 0;
        }
        if self.model.has_030() {
            // The address translation cache is derived state, so it is never
            // placed from outside — but it *describes* the registers below,
            // and an entry made under one mapping would answer wrongly under
            // another. So a register that actually changes empties it, and a
            // register file put back unchanged leaves it alone.
            let changed = state.mmu.tt != regs.tt
                || (self.model.has_mmu()
                    && (state.mmu.tc != regs.tc
                        || state.mmu.crp != regs.crp
                        || state.mmu.srp != regs.srp));
            if changed {
                state.mmu.flush_all();
            }
            state.mmu.tt = regs.tt;
            state.mmu.mmusr = regs.mmusr as u16;
            if self.model.has_mmu() {
                state.mmu.tc = regs.tc;
                state.mmu.crp = regs.crp;
                state.mmu.srp = regs.srp;
            }
        }
        if self.model.has_040() {
            // As on the 68030: the address translation cache is derived
            // state and is never placed from outside, but it *describes*
            // these registers, so a register that actually changes empties
            // it and a register file put back unchanged leaves it alone.
            let changed = state.mmu040.itt != regs.itt
                || state.mmu040.dtt != regs.dtt
                || (self.model.has_mmu_040()
                    && (state.mmu040.tcr != regs.tc as u16
                        || state.mmu040.urp != regs.urp
                        || state.mmu040.srp != regs.srp as u32));
            if changed {
                state.mmu040.flush_all();
            }
            state.mmu040.itt = regs.itt;
            state.mmu040.dtt = regs.dtt;
            if self.model.has_mmu_040() {
                state.mmu040.tcr = regs.tc as u16;
                state.mmu040.urp = regs.urp;
                state.mmu040.srp = regs.srp as u32;
                state.mmu040.mmusr = regs.mmusr;
            }
        }
        if self.fpu.present() {
            state.fpu.fp = regs.fp;
            state.fpu.fpcr = regs.fpcr & fpu::bits::FPCR_IMPLEMENTED;
            state.fpu.fpsr = regs.fpsr & fpu::bits::FPSR_IMPLEMENTED;
            state.fpu.fpiar = regs.fpiar;
            // `FSAVE` reports the null state for a unit "not modified since
            // the last hardware reset" (M68881UM §4), so a register file
            // placed from outside leaves it null exactly when what was placed
            // *is* the reset state.
            let reset = fpu::Fpu::RESET;
            state.fpu.null = state.fpu.fp == reset.fp
                && state.fpu.fpcr == reset.fpcr
                && state.fpu.fpsr == reset.fpsr
                && state.fpu.fpiar == reset.fpiar;
        }
    }

    /// Read one register by name.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u32 {
        reg.get(&self.regs())
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u32) {
        let mut regs = self.regs();
        reg.set(&mut regs, value);
        self.set_regs(regs);
    }

    /// Cycles executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a double bus fault has halted the core.
    ///
    /// A 68000 that faults while taking an exception asserts `HALT` and stops
    /// until a reset. [`step`](M68k::step) returns zero cycles once this is
    /// true, so a scheduler must notice it rather than spin.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// The vector of the exception the last [`step`](M68k::step) took, if it
    /// took one — for tests that need to know how a step ended.
    #[cfg(test)]
    pub(crate) fn last_exception(&self) -> Option<u8> {
        self.session.lock().state.last_vector
    }

    /// Whether `STOP` has suspended the core until an interrupt.
    #[must_use]
    pub fn is_stopped(&self) -> bool {
        self.session.lock().state.stopped
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// How many accesses the address space refused, and where the last one
    /// was.
    ///
    /// A refused access becomes a bus-error exception, so unlike the 6502 this
    /// is not the whole story — but a machine whose memory map has a hole will
    /// still show it climbing.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// How many times the `RESET` instruction has pulsed the reset line.
    ///
    /// `RESET` resets peripherals, not the processor; what hangs off the pin
    /// is the machine's business, so the core only counts.
    #[must_use]
    pub fn reset_pulses(&self) -> u32 {
        self.lines.resets()
    }

    /// Drive IPL0–IPL2 with an encoded interrupt level, 0 (none) to 7.
    ///
    /// Levels 1–6 are level-sensitive: they are taken, and re-taken, while
    /// they exceed the mask in `SR`. **Level 7 is edge-triggered** — the
    /// transition to it is what the processor recognises — so holding the pins
    /// at 7 raises exactly one non-maskable interrupt, and raising another
    /// means dropping the level and driving 7 again.
    pub fn set_ipl(&self, level: u8) {
        self.lines.set_ipl(level);
    }

    /// The level currently encoded on the interrupt pins.
    #[must_use]
    pub fn ipl(&self) -> u8 {
        self.lines.ipl()
    }

    /// Supply the vector number the *next* interrupt acknowledge will fetch.
    ///
    /// **Consumed by that acknowledge**, exactly as a device answering the
    /// cycle would be: a controller arms a vector per interrupt, and anything
    /// that does not arm one autovectors, which is what asserting `VPA` means
    /// and what most 68000 machines do. `None` disarms it again.
    ///
    /// The acknowledge cycle itself does not reach the bus — it is CPU space,
    /// and `MemAttrs` carries no function code — so this is how a vectoring
    /// controller talks to the core. See `exec.rs`'s `take_interrupt`.
    pub fn set_interrupt_vector(&self, vector: Option<u8>) {
        self.lines.set_vector(vector);
    }

    /// The vector armed for the next acknowledge, if any.
    #[must_use]
    pub fn interrupt_vector(&self) -> Option<u8> {
        match self.lines.vector.load(Ordering::Acquire) {
            NO_VECTOR => None,
            other => Some(other as u8),
        }
    }

    /// Say whether the core still owes a reset sequence.
    ///
    /// A fresh core owes one, so a register file written before the first
    /// [`step`](M68k::step) would be thrown away by it. Anything that places a
    /// core mid-program — a debugger, a test vector, a machine resuming a
    /// loaded image, the differential tester the IR frontend will need — turns
    /// it off first. [`request_reset`](M68k::request_reset) is the same switch
    /// the other way round, named for the common case.
    pub fn set_reset_pending(&self, pending: bool) {
        self.session.lock().state.reset_pending = pending;
    }

    /// Bring a halted or stopped core back to life without resetting it.
    ///
    /// A double bus fault halts the processor and only a reset restarts it on
    /// real hardware; this is the debugger's override, and the way a test
    /// places a core that a previous vector left halted.
    pub fn resume(&self) {
        let mut session = self.session.lock();
        session.state.halted = false;
        session.state.stopped = false;
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](M68k::step), because that is
    /// when the CPU can read vectors 0 and 1 — a reset is a signal, not a
    /// method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, exception sequence, or instruction.
    ///
    /// Returns the cycles charged: zero if the core is halted or has no
    /// address space, which the caller must treat as "stop", not "retry".
    pub fn step(&self) -> u64 {
        let reset = self.lines.take_reset_request();
        let cfg = self.config();
        let mut session = self.session.lock();
        let Session { state, space } = &mut *session;
        // The `reset` pin latches outside the lock; this is where the latch
        // becomes execution state, before the step, so a pulse is honoured at
        // the very next instruction boundary.
        state.reset_pending |= reset;
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — a 68000 cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    /// Stops early if the core halts.
    ///
    /// [`run_budget`](M68k::run_budget) is the same loop with the overshoot
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
    /// A halted core — one a double bus fault stopped — still consumes its
    /// budget: the clock keeps running, and a domain that freezes there falls
    /// behind the reset that would restart it.
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
                // Halted, stopped, or no address space. Retrying would spin.
                self.session.lock().state.debt = 0;
                return ticks;
            }
            used += n;
        }
        self.session.lock().state.debt = used - allowance;
        ticks
    }

    /// Cycles owed to the next budget — see [`run_budget`](M68k::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around PC
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    #[must_use]
    pub fn disassemble(&self, pc: u32, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let mask = self.model.address_mask();
        let copro = if self.fpu.present() {
            isa::Copro::FPU
        } else {
            isa::Copro::NONE
        };
        disasm::disassemble_run_with(self.model, copro, pc, count, |addr| {
            space
                .read(u64::from(addr & mask), Width::U16, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u16)
        })
    }
}

/// The `cpu.m68k` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.m68k",
    // 2: the chunk gained the scheduler debt, without which a restored core
    //    runs one instruction free.
    // 3: and the 68030's memory management registers. The address
    //    translation cache is *not* in it — it is derived state, and a
    //    restored core rebuilds it with a table search (CLAUDE.md,
    //    *Devices*).
    // 4: and the 68040's memory management registers, which are different
    //    registers rather than wider ones and so take their own chunk, plus
    //    what its floating-point unit still owes an `FSAVE`.
    version: 4,
    summary: "Motorola MC68000/68010/68020/68030/68040 32-bit CPU core, interpreter",
    properties: &[
        PropertySpec {
            name: "requester",
            kind: ValueKind::Uint,
            required: false,
            summary: "this core's requester id in MemAttrs, for an IOMMU or a per-master filter",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "which processor: `68000` (the default), `68010`, `68020`, \
`68ec020`, `68030`, `68ec030`, `68040`, `68lc040` or `68ec040`",
        },
        PropertySpec {
            name: "fpu",
            kind: ValueKind::Str,
            required: false,
            summary: "which floating-point unit: `none`, `68881` or `68882` on a 68020 \
or 68030, and `68040` — the on-chip one, and the default — on a 68040",
        },
    ],
    construct: |props| Ok(Box::new(M68k::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-m68k")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for M68k {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    /// The three `IPL` pins and `RESET`.
    ///
    /// **`ipl0`, `ipl1` and `ipl2` are three ports onto one sink**, and that is
    /// the shape a 68000 forces. The pins carry an encoded *priority level*,
    /// not three independent requests, so nothing can decide what the level is
    /// without seeing all three at once — which means one [`InterruptPins`]
    /// object with a [`FanIn`] per line, handed out three times with different
    /// [`SinkPin::line`] numbers. A machine with a single source drives one
    /// line and gets level 1, 2 or 4; a machine with a priority encoder drives
    /// all three.
    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            "ipl0" => 0u32,
            "ipl1" => 1,
            "ipl2" => 2,
            "reset" => {
                let mut pins = self.pins.lock();
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                return Some(SinkPin { sink: pin, line: 0 });
            }
            _ => return None,
        };
        // One object, created on the first `IPL` port asked for and *kept*.
        // Rebuilding it per port would hand each net a different sink, and a
        // net holds its sink weakly — the earlier ones would die on the spot.
        // So the fan-in for this line is installed into the object that
        // already exists; a line nothing asked for has no sources and rests
        // low, which is a zero in that bit of the level.
        let object = {
            let mut pins = self.pins.lock();
            Arc::clone(pins.ipl.get_or_insert_with(|| {
                Arc::new(InterruptPins::from_lines(Arc::clone(&self.lines)))
            }))
        };
        // Outside the critical section: `install` takes the pins' own
        // `WIRE`-ranked lock, and this one is a leaf (`ROADMAP.md` §4.7).
        object.install(line as usize, sources);
        Some(SinkPin { sink: object, line })
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        // Any `IPL` pin, and every controller offered on one is kept: a 68000's
        // acknowledge cycle puts the *level* on A3-A1 and each device in CPU
        // space decides whether it is the one being asked, so a handler belongs
        // to the processor rather than to one line. A controller encoding level
        // 5 drives `ipl0` and `ipl2` and is offered twice; `IntAckHandlers`
        // keeps it once. See [`Lines::acknowledge`].
        if matches!(port, "ipl0" | "ipl1" | "ipl2") {
            self.lines.attach_ack(ack);
        }
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            // A cold start has no defined register contents on real hardware;
            // zeroing them is the reproducible choice, and determinism is a
            // first-class mode (`ROADMAP.md` §0).
            session.state = State::new(self.model);
        } else {
            // A warm reset is a pulse on the RESET pin: the register file
            // keeps its values and only the sequence's own effects apply.
            session.state.reset_pending = true;
            session.state.halted = false;
            session.state.stopped = false;
        }
        drop(session);
        // The sequence the machine just asked for is the one the pin owed.
        self.lines.take_reset_request();
        if kind == ResetKind::Cold {
            self.lines.restore((0, NO_VECTOR, false, 0));
        }
    }

    /// The chunk, version 2.
    ///
    /// A 68000's is exactly what it was before this core knew about any other
    /// processor, byte for byte. A later model appends a tail — its model, all
    /// three stack pointers, the control registers and the fault bookkeeping —
    /// which is a shape no older reader was ever asked for: a snapshot names
    /// its device's properties, and an older build had no `model` to name. A
    /// 68010 snapshot offered to a 68000 core fails on the trailing bytes
    /// rather than loading half of itself.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Fold the `RESET` pin's latch in first. It is not a field of its own
        // in the chunk: `reset_pending` is where it was always going, and a
        // snapshot taken between an assertion and the next step would otherwise
        // lose the reset entirely.
        let reset = self.lines.take_reset_request();
        let state = {
            let mut session = self.session.lock();
            session.state.reset_pending |= reset;
            session.state
        };
        for value in state.d {
            w.write_u32(value)?;
        }
        for value in state.a {
            w.write_u32(value)?;
        }
        // The stack pointer `a[7]` is not: the 68000's two-bank layout.
        let other_sp = if state.supervisor() {
            state.usp()
        } else {
            state.ssp()
        };
        w.write_u32(other_sp)?;
        w.write_u32(state.pc)?;
        w.write_u16(state.sr)?;
        w.write_u16(state.prefetch[0])?;
        w.write_u16(state.prefetch[1])?;
        w.write_u64(state.cycles)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.stopped)?;
        w.write_bool(state.reset_pending)?;
        w.write_u64(state.faults)?;
        w.write_u32(state.last_fault)?;
        w.write_u64(state.debt)?;
        let (ipl, vector, level_seven, resets) = self.lines.snapshot();
        w.write_u8(ipl)?;
        w.write_u16(vector)?;
        w.write_bool(level_seven)?;
        w.write_u32(resets)?;
        if self.model == Model::M68000 {
            return Ok(());
        }
        w.write_u8(self.model as u8)?;
        w.write_u32(state.sp(Bank::User))?;
        w.write_u32(state.sp(Bank::Interrupt))?;
        w.write_u32(state.sp(Bank::Master))?;
        w.write_u32(state.vbr)?;
        w.write_u8(state.sfc)?;
        w.write_u8(state.dfc)?;
        w.write_u32(state.cacr)?;
        w.write_u32(state.caar)?;
        w.write_bool(state.replay.is_some())?;
        let replay = state.replay.unwrap_or(exec::Replay {
            addr: 0,
            read: false,
            width: 0,
            data: 0,
        });
        w.write_u32(replay.addr)?;
        w.write_bool(replay.read)?;
        w.write_u8(replay.width)?;
        w.write_u32(replay.data)?;
        for slot in state.poison {
            w.write_bool(slot.is_some())?;
            w.write_u32(slot.unwrap_or(0))?;
        }
        // Every 68040 package has the four transparent translation
        // registers; only the parts with the paged unit have the rest. The
        // 68030 branch below is skipped on a 68040 — `has_030` is false,
        // because a 68040 is not a 68030 with extras.
        if self.model.has_040() {
            for value in state.mmu040.itt {
                w.write_u32(value)?;
            }
            for value in state.mmu040.dtt {
                w.write_u32(value)?;
            }
            if self.model.has_mmu_040() {
                w.write_u16(state.mmu040.tcr)?;
                w.write_u32(state.mmu040.urp)?;
                w.write_u32(state.mmu040.srp)?;
                w.write_u32(state.mmu040.mmusr)?;
            }
        }
        // Both 68030 packages have the transparent translation registers and
        // the status register; only the full part has the paged unit's.
        if self.model.has_030() {
            w.write_u32(state.mmu.tt[0])?;
            w.write_u32(state.mmu.tt[1])?;
            w.write_u16(state.mmu.mmusr)?;
        }
        if self.model.has_mmu() {
            w.write_u32(state.mmu.tc)?;
            w.write_u64(state.mmu.crp)?;
            w.write_u64(state.mmu.srp)?;
        }
        if !self.fpu.present() {
            return Ok(());
        }
        for value in state.fpu.fp {
            w.write_u16(value.sign_exp)?;
            w.write_u64(value.sig)?;
        }
        w.write_u32(state.fpu.fpcr)?;
        w.write_u32(state.fpu.fpsr)?;
        w.write_u32(state.fpu.fpiar)?;
        w.write_bool(state.fpu.null)?;
        if !self.fpu.is_onchip_040() {
            return Ok(());
        }
        // What a 68040's `FSAVE` still owes an emulation handler. It is
        // architectural state — the handler will read it — so it travels,
        // unlike the address translation cache beside it, which is derived.
        let pending = state.fpu.pending_040;
        w.write_bool(pending.is_some())?;
        let pending = pending.unwrap_or(fpu::State040 {
            command: 0,
            etemp: F80::ZERO,
            stag: 0,
            fptemp: F80::ZERO,
            dtag: 0,
            post_instruction: false,
        });
        w.write_u16(pending.command)?;
        for value in [pending.etemp, pending.fptemp] {
            w.write_u16(value.sign_exp)?;
            w.write_u64(value.sig)?;
        }
        w.write_u8(pending.stag)?;
        w.write_u8(pending.dtag)?;
        w.write_bool(pending.post_instruction)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.model);
        for slot in &mut state.d {
            *slot = r.read_u32()?;
        }
        for slot in &mut state.a {
            *slot = r.read_u32()?;
        }
        let other_sp = r.read_u32()?;
        state.pc = r.read_u32()?;
        let sr = r.read_u16()?;
        if sr & !state.sr_mask() != 0 {
            return Err(Error::State(alloc::format!(
                "status register 0x{sr:04x} sets bits a {} does not implement",
                self.model
            )));
        }
        state.sr = sr;
        // The two-bank reading, which is the whole story on a 68000 and is
        // overwritten from the tail on anything later.
        if state.supervisor() {
            state.banks[Bank::User as usize] = other_sp;
        } else {
            state.banks[Bank::Interrupt as usize] = other_sp;
        }
        state.prefetch[0] = r.read_u16()?;
        state.prefetch[1] = r.read_u16()?;
        state.cycles = r.read_u64()?;
        state.halted = r.read_bool()?;
        state.stopped = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u32()?;
        state.debt = r.read_u64()?;
        let ipl = r.read_u8()?;
        if ipl > 7 {
            return Err(Error::State(alloc::format!(
                "interrupt level {ipl} does not fit on three pins"
            )));
        }
        let vector = r.read_u16()?;
        if vector != NO_VECTOR && vector > 0xff {
            return Err(Error::State(alloc::format!(
                "interrupt vector 0x{vector:04x} is not a vector number"
            )));
        }
        let level_seven = r.read_bool()?;
        let resets = r.read_u32()?;
        if self.model != Model::M68000 {
            let model = r.read_u8()?;
            if model != self.model as u8 {
                return Err(Error::State(alloc::format!(
                    "a snapshot of model {model} cannot be loaded into a {}",
                    self.model
                )));
            }
            let usp = r.read_u32()?;
            let isp = r.read_u32()?;
            let msp = r.read_u32()?;
            state.banks = [usp, isp, msp];
            state.a[7] = state.banks[state.bank_of(state.sr) as usize];
            state.vbr = r.read_u32()?;
            state.sfc = r.read_u8()? & 7;
            state.dfc = r.read_u8()? & 7;
            state.cacr = r.read_u32()? & state.cacr_mask();
            state.caar = r.read_u32()?;
            let has_replay = r.read_bool()?;
            let replay = exec::Replay {
                addr: r.read_u32()?,
                read: r.read_bool()?,
                width: r.read_u8()?,
                data: r.read_u32()?,
            };
            state.replay = has_replay.then_some(replay);
            for slot in &mut state.poison {
                let poisoned = r.read_bool()?;
                let addr = r.read_u32()?;
                *slot = poisoned.then_some(addr);
            }
        }
        if self.model.has_040() {
            for slot in &mut state.mmu040.itt {
                *slot = r.read_u32()? & mmu040::ttr::IMPLEMENTED;
            }
            for slot in &mut state.mmu040.dtt {
                *slot = r.read_u32()? & mmu040::ttr::IMPLEMENTED;
            }
            if self.model.has_mmu_040() {
                state.mmu040.tcr = r.read_u16()? & mmu040::tcr::IMPLEMENTED;
                state.mmu040.urp = r.read_u32()? & !0x1ff;
                state.mmu040.srp = r.read_u32()? & !0x1ff;
                state.mmu040.mmusr = r.read_u32()? & mmu040::mmusr::IMPLEMENTED;
            }
        }
        if self.model.has_030() {
            state.mmu.tt[0] = r.read_u32()?;
            state.mmu.tt[1] = r.read_u32()?;
            state.mmu.mmusr = r.read_u16()?;
            if self.model.has_mmu() {
                state.mmu.tc = r.read_u32()?;
                state.mmu.crp = r.read_u64()?;
                state.mmu.srp = r.read_u64()?;
            }
        }
        if self.fpu.present() {
            for slot in &mut state.fpu.fp {
                let sign_exp = r.read_u16()?;
                let sig = r.read_u64()?;
                *slot = F80::new(sign_exp, sig);
            }
            state.fpu.fpcr = r.read_u32()? & fpu::bits::FPCR_IMPLEMENTED;
            state.fpu.fpsr = r.read_u32()? & fpu::bits::FPSR_IMPLEMENTED;
            state.fpu.fpiar = r.read_u32()?;
            state.fpu.null = r.read_bool()?;
            if self.fpu.is_onchip_040() {
                let present = r.read_bool()?;
                let command = r.read_u16()?;
                let mut operand = [F80::ZERO; 2];
                for slot in &mut operand {
                    let sign_exp = r.read_u16()?;
                    let sig = r.read_u64()?;
                    *slot = F80::new(sign_exp, sig);
                }
                let state040 = fpu::State040 {
                    command,
                    etemp: operand[0],
                    fptemp: operand[1],
                    stag: r.read_u8()? & 7,
                    dtag: r.read_u8()? & 7,
                    post_instruction: r.read_bool()?,
                };
                state.fpu.pending_040 = present.then_some(state040);
            }
        }
        self.session.lock().state = state;
        self.lines.restore((ipl, vector, level_seven, resets));
        Ok(())
    }
}
impl Initiator for M68k {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a core needs an address space, and this is where
/// the machine gives it one.
///
/// **The space must be big-endian.** A 68000 reads a word as high byte first,
/// and `core::space` carries endianness per region rather than per initiator,
/// so a machine file that maps little-endian RAM under a 68000 gets every word
/// byte-swapped. That is not something `bind` can check — the map is the
/// board's, and a big-endian core sharing a region with a little-endian one is
/// a legitimate configuration (`ROADMAP.md` §5's motivating case).
impl crate::machine::Instance for M68k {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("a 68000 needs an address space to fetch from (`space = mem`)"),
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
    bindings.bind(CLASS.name, |props| Ok(Arc::new(M68k::from_props(props)?)))
}

/// What the validator should know about `cpu.m68k`.
///
/// # Three interrupt pins, not one
///
/// `ipl0`, `ipl1` and `ipl2` carry an encoded **priority level**, 0 to 7, not
/// three independent requests — which is genuinely different from an IRQ line
/// and is why this core has three ports where the others have one. A board with
/// a single source wires it to one pin and gets level 1, 2 or 4; a board with a
/// priority encoder wires all three.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("requester", ValueKind::Uint))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .prop(PropSchema::new("model", ValueKind::Str).values(&MODEL_NAMES))
        .prop(PropSchema::new("fpu", ValueKind::Str).values(&FPU_NAMES))
        // Inputs only. `BERR`, `HALT`, `BR`/`BG` and `VPA` are real pins with
        // no model behind them: a bus error is reported through the address
        // space's result rather than a wire, and `VPA` is what *not* answering
        // the acknowledge cycle means.
        .port("ipl0", PortDir::In)
        .port("ipl1", PortDir::In)
        .port("ipl2", PortDir::In)
        .port("reset", PortDir::In)
}

/// The three interrupt priority inputs, as something a [`Wire`] can drive.
///
/// IPL0–IPL2 carry an encoded *level*, not three independent requests, so a
/// net per line would be the wrong model: this sink keeps a [`FanIn`] per line
/// and recomputes the level whenever any of them changes. A machine with a
/// single interrupt source can drive one line and get level 1, 2 or 4; a
/// machine with a priority encoder drives all three.
///
/// The pins are active-low on real hardware; inverting them belongs to
/// whatever models the wire, so a high level here means "asserted".
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPins {
    lines: Arc<Lines>,
    /// One fan-in per line, behind a lock because the machine layer installs
    /// them **one port at a time**.
    ///
    /// `Device::sink` is asked for `ipl0`, `ipl1` and `ipl2` separately and is
    /// told each net's drivers only when that net is built — but all three have
    /// to live in one object, because they encode a single level and no one of
    /// them can be resolved alone. Rebuilding the object per port would hand
    /// each net a different sink, and a net holds its sink weakly, so the
    /// earlier ones would die on the spot. Hence interior mutability rather
    /// than a `[FanIn; 3]` fixed at construction.
    inputs: sync::Mutex<[FanIn; 3]>,
    resolve: Resolve,
}

impl InterruptPins {
    /// Connect `cpu`'s three interrupt inputs to nets driven by `sources`.
    ///
    /// `sources[i]` is every id that drives IPL`i`. Wire-OR by default, which
    /// is how an open-collector interrupt line behaves.
    ///
    /// The object keeps a handle on the core's *input latches*, not on the
    /// core: the core owns the pins — something must, since a net holds only a
    /// weak reference to its sinks — and pins that owned the core back would be
    /// a cycle the machine could never drop.
    #[must_use]
    pub fn new(cpu: Arc<M68k>, sources: [&[WireId]; 3]) -> InterruptPins {
        let pins = InterruptPins::from_lines(Arc::clone(&cpu.lines));
        for (line, srcs) in sources.iter().enumerate() {
            pins.install(line, srcs);
        }
        pins
    }

    /// The same, given the latches directly and no sources yet.
    fn from_lines(lines: Arc<Lines>) -> InterruptPins {
        InterruptPins {
            lines,
            inputs: sync::Mutex::with_rank(
                LockRank::WIRE,
                [FanIn::new(&[]), FanIn::new(&[]), FanIn::new(&[])],
            ),
            resolve: Resolve::Or,
        }
    }

    /// Tell line `line` which ids drive it.
    fn install(&self, line: usize, sources: &[WireId]) {
        self.inputs.lock()[line.min(2)] = FanIn::new(sources);
    }

    /// The same pins with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> Self {
        self.resolve = resolve;
        self
    }

    /// The level currently resolved on one line.
    #[must_use]
    pub fn level(&self, line: usize) -> Level {
        self.inputs.lock()[line.min(2)].resolve(self.resolve)
    }

    /// The three lines as the priority level they encode, 0 to 7.
    #[must_use]
    pub fn encoded(&self) -> u8 {
        let inputs = self.inputs.lock();
        let mut encoded = 0u8;
        for (bit, input) in inputs.iter().enumerate() {
            if input.resolve(self.resolve).is_high() {
                encoded |= 1 << bit;
            }
        }
        encoded
    }
}

impl WireSink for InterruptPins {
    fn set_level(&self, src: WireId, line: u32, level: Level) {
        let encoded = {
            let inputs = self.inputs.lock();
            inputs[(line as usize).min(2)].set(src, level);
            let mut encoded = 0u8;
            for (bit, input) in inputs.iter().enumerate() {
                if input.resolve(self.resolve).is_high() {
                    encoded |= 1 << bit;
                }
            }
            encoded
        };
        // Outside the critical section, per the re-entrancy contract — even
        // though what follows is one atomic store (`ROADMAP.md` §4.7).
        self.lines.set_ipl(encoded);
    }
}

/// The core's `RESET` input, as something a [`Wire`] can drive.
///
/// Not one of the `IPL` pins, and not an interrupt: `RESET` is a level the
/// board holds, and on a 68000 it is bidirectional — the `RESET` *instruction*
/// drives it outward to reset peripherals without resetting the processor,
/// which is what [`M68k::reset_pulses`] counts. This is the inward half.
///
/// Asserting the line latches a request; the sequence, which reads vectors 0
/// and 1, runs on the next [`M68k::step`].
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect `cpu`'s `RESET` pin to a net driven by `sources`.
    #[must_use]
    pub fn new_for(cpu: Arc<M68k>, sources: &[WireId]) -> ResetPin {
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
        // Latch on assertion rather than on release: a machine holding its
        // reset button down should still come up, instead of waiting for a
        // release nobody modelled.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset_pin();
        }
    }
}

/// A description of this core's instruction set for `rsemu describe cpu.m68k`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements. Each row says which processors have it, because half the
/// question about a 680x0 encoding is which part it is legal on.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for pattern in isa::TABLE {
        let insn = pattern.insn;
        let mark = if insn.privileged { '!' } else { ' ' };
        let models = match insn.models {
            m if m == isa::Models::ALL => "all   ",
            m if m == isa::Models::FROM_010 => "010+  ",
            m if m == isa::Models::FROM_020 => "020+  ",
            m if m == isa::Models::FROM_030 => "030+  ",
            m if m == isa::Models::M68020 => "020   ",
            m if m == isa::Models::M68030 => "030   ",
            m if m == isa::Models::UNTIL_010 => "000010",
            _ => "000   ",
        };
        let _ = writeln!(
            out,
            "{:04x}/{:04x} {models} {mark}{:<8} {}",
            pattern.mask,
            pattern.value,
            insn.op.mnemonic(),
            insn.op.summary()
        );
    }
    out
}
