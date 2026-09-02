//! Intel x86: the 8086 and 8088 in real mode, the 80386 and 80486 with
//! protection, paging and 32-bit operands, and x86-64 with long mode.
//!
//! `ROADMAP.md` §6 calls x86 "the hard one". One interpreter covers every
//! part, selected by [`Variant`] and [`Features`] rather than by a second
//! module, because the generations really are close to a superset chain — and
//! where they are not, the difference is named and modelled rather than
//! flattened (the table in the private `exec` module lists all ten, and
//! [`isa::L64`] lists the ones long mode added).
//!
//! # The lattice, not the ladder
//!
//! [`Variant`] names a *part* and [`Features`] says what it has, and the two
//! are separate on purpose (`ROADMAP.md` §6.1.1). `PAE` arrived on a Pentium
//! Pro with no long mode; `SYSCALL` on an AMD K6 with no 64-bit anything;
//! `NX` on parts that shipped both with and without it inside one model
//! number. So a decode or execute site asks whether the *feature* is present,
//! never whether the variant is at least some other variant — and [`Variant`]
//! is deliberately not `Ord`, so it cannot grow the comparison that would make
//! that possible.
//!
//! # What is modelled
//!
//! **Real mode, on every part.** `segment:offset` through a 20-bit adder on an
//! 8086, with the address wrapping at 1 MiB — `0xffff:0x0010` is physical
//! `0x00000`, which is the behaviour the A20 gate was later invented to
//! suppress. On a 386 the same instructions go through a cached descriptor
//! instead, so the address is 32 bits and the segment's limit is checked.
//!
//! **Every 8086 instruction**, the undocumented encodings included, because
//! the 8086 has no invalid-opcode exception and the hardware corpus tests all
//! of them: `SALC`, `SETMO`, `POP CS`, the `60`-`6F` jump aliases, the second
//! `RET` encodings, and the group extensions Intel left blank. The undefined
//! *flag* results with them: each was matched against silicon rather than
//! guessed, and `exec` documents them one by one.
//!
//! **The 80186 and 80386 additions**: `PUSHA`/`POPA`, `ENTER`/`LEAVE`,
//! `BOUND`, `INS`/`OUTS`, the immediate shifts, the three-operand `IMUL`, and
//! the whole two-byte `0F` page — `MOVZX`/`MOVSX`, `BSF`/`BSR`, the bit tests,
//! `SHLD`/`SHRD`, `SETcc`, the near conditional jumps, `LSS`/`LFS`/`LGS`, and
//! the control-, debug- and test-register moves. On an 80486, `CPUID`,
//! `BSWAP`, `XADD`, `CMPXCHG`, `INVLPG` and the cache instructions.
//!
//! **Protected mode** ([`prot`]): the global, local and interrupt descriptor
//! tables; segment registers as a selector plus a **cached descriptor**, which
//! is what makes a live descriptor edit invisible until a reload and what
//! makes unreal mode work; privilege levels with the `CPL`/`RPL`/`DPL` checks;
//! call, interrupt, trap and task gates; the task state segment, its
//! privilege-0 stack and its I/O permission bitmap; and task switching.
//!
//! **Paging** ([`paging`]): one walk at four depths — off, the two-level
//! directory and table, PAE's three levels of 64-bit entries, and IA-32e's
//! four — with 4 MiB, 2 MiB and 1 GiB pages, `CR2` and `CR3`, the accessed and
//! dirty bits written by the walk itself, a translation-lookaside buffer so
//! they are written once rather than on every access, `INVLPG`, and page
//! faults with their error code. The debug walk shares the same function
//! rather than duplicating it.
//!
//! **Long mode** ([`prot`], [`isa`]): `EFER.LME` and the `LMA` bit the
//! processor sets for itself when paging comes on; `CR4` and the
//! model-specific registers; the `REX` prefix, sixteen 64-bit registers, and
//! `RIP`-relative addressing; the changed default operand sizes and the
//! twenty-odd encodings long mode reclaimed; 64-bit and compatibility submodes
//! with the descriptor `L` bit that selects them; sixteen-byte system
//! descriptors and interrupt gates with their interrupt-stack table; the
//! canonical-address rule; and `SYSCALL`, `SYSRET` and `SWAPGS`.
//!
//! **The x87 unit** ([`fpu`], and [`crate::float`] for the arithmetic): the
//! eight-register rotating stack with its tag word, the control word's
//! precision and rounding control, the status word's condition codes, the six
//! exception masks and the deferred `#MF` they produce, `FXCH` and the stack
//! overflow and underflow rules, the environment and whole-unit save formats,
//! and the 80-bit double extended format itself. **In software throughout** —
//! there is no host `f32` or `f64` anywhere on the path from an escape to a
//! result, which is what makes two hosts agree bit for bit (`ROADMAP.md`
//! §9.1).
//!
//! **SSE and SSE2** ([`fpu`] again): sixteen `XMM` registers, `MXCSR` with its
//! rounding, flush-to-zero and denormals-are-zero bits mapped onto
//! [`crate::float::Env`], the scalar and packed single- and double-precision
//! arithmetic, the compares that write `EFLAGS` and the ones that write a
//! lane mask, the conversions, the shuffles, and `FXSAVE`/`FXRSTOR`.
//! `CR4.OSFXSR` and `CR4.OSXMMEXCPT` now decide something: without the first
//! an SSE instruction is `#UD`, and without the second an unmasked SIMD
//! exception is `#UD` rather than `#XM`.
//!
//! **The model-specific registers** ([`prot::msr`]): `RDMSR` and `WRMSR`
//! reaching `IA32_EFER`, the four `SYSCALL` registers, the three segment-base
//! registers, the time-stamp counter that `RDTSC` also reads, and
//! `IA32_APIC_BASE` — which is the one whose *state* is not in the processor at
//! all, and reaches this core's own interrupt controller through
//! [`crate::core::wire::LocalController`]. An address this
//! core does not implement raises `#GP(0)`; a guest that read a plausible zero
//! instead would conclude a feature was present and disabled, and misbehave a
//! long way from the cause.
//!
//! **Starting a second processor**: the INIT and Start-Up pair, which is what
//! makes an application processor possible. `INIT` — a pin, or a message from
//! this core's own local interrupt controller — is a *lesser* restart than
//! `RESET`: it leaves the processor in the **wait-for-SIPI** state rather than
//! fetching from the reset vector, and nothing but a Start-Up moves it from
//! there, not an `INTR` and not an `NMI`. A Start-Up names a page, and the
//! processor begins executing at `CS:IP = page << 8 : 0`. See
//! [`X86::request_init`] and [`X86::start_up`], the *MultiProcessor
//! Specification* v1.4 §B.4 for the sequence a guest writes, and
//! `tests/pc_apic_smp.rs` for a guest writing it.
//!
//! **The exception model**: faults, traps and aborts, with the vectors and
//! error codes the manual gives them; faults restart the instruction they came
//! from; and the double-fault table decides whether a second exception is
//! taken on its own or escalates, with a third shutting the processor down.
//!
//! A separate I/O address space for `IN`/`OUT`, supplied by the machine as a
//! second [`AddressSpace`] — not a corner of memory, which is what the
//! architecture says and what a PC's chipset relies on.
//!
//! # What is not
//!
//! - **No MMX.** Its eight registers alias the x87 stack, which is a
//!   correctness trap rather than a feature, and `CPUID`'s `MMX` bit is clear.
//! - **No x87 transcendentals.** `F2XM1`, `FYL2X`, `FYL2XP1`, `FPTAN`,
//!   `FPATAN`, `FSIN`, `FCOS` and `FSINCOS` are unassigned in [`isa`] and
//!   raise `#UD`. Computing them to the last bit of a 64-bit significand
//!   without a host `f64` is a subproject of its own, and an approximation
//!   would be a silently wrong answer where a missing instruction is a loud
//!   one. `FBLD`/`FBSTP` (packed decimal) and `FISTTP` (SSE3) are absent for
//!   the same reason of scope.
//! - **No `FERR#` pin.** An unmasked x87 exception is delivered as `#MF` only
//!   with `CR0.NE` set; with it clear a real PC routes the exception through
//!   the chipset to IRQ 13, and no wire models that, so the exception stays
//!   pending instead of being delivered down a path that does not exist.
//! - **No SSE3 or later**, no AVX, and no `XSAVE`.
//! - **No hardware task switching in long mode**, which is the architecture:
//!   a far transfer to a task gate or a task state segment is `#GP` there, and
//!   the 64-bit task state segment is read only for its stack pointers.
//! - **No virtual-address width above 48 bits.** Five-level paging (`LA57`)
//!   is not implemented, and `CPUID` leaf `8000_0008` reports 48.
//! - **No virtual-8086 mode.** `EFLAGS.VM` has storage and nothing sets it.
//! - **No debug breakpoints.** `DR0`-`DR7` round-trip; arming one fires
//!   nothing. `TR6`/`TR7` likewise store and do nothing.
//! - **No alignment check.** `CR0.AM` and `EFLAGS.AC` have storage; no `#AC`
//!   is ever raised.
//! - **`LOCK` is decoded and ignored.** One core, one bus, and nothing to
//!   contend with; the invalid-opcode exception a `LOCK` on a non-lockable
//!   instruction should raise is not enforced.
//! - **The accessed bit** is set when a selector is loaded by `MOV Sreg` or by
//!   a far transfer to a code segment, but not by the segment loads a gate,
//!   an `IRET` or a task switch performs. Hardware sets it in all of them.
//! - **No 286-format task state segment.** Switching to one raises `#TS`
//!   rather than silently truncating the state it cannot hold.
//! - **No A20 gate.** Masking address line 20 happens between the processor
//!   and memory on a PC, so it belongs to the chipset, not here.
//! - Cycle *counts* are documented timing, not measured timing: the bus
//!   interface unit's prefetching is not overlapped with execution, so a count
//!   taken over a long run is an upper bound.
//!
//! # Accuracy is measured
//!
//! `SingleStepTests/8088` is a hardware-generated corpus: ten thousand vectors
//! per opcode, captured from a real AMD D8088 with an Arduino interposer,
//! complete with the bus trace. The private `conformance` module runs it
//! **twice** — once as an 8088, which is what it was captured from, and once
//! as an 80386, where every disagreement is traced to a documented difference
//! between the parts. `ROADMAP.md` §0 asks for accuracy to be measured rather
//! than asserted, and on this architecture there is no other honest way to
//! claim anything.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::x86::{Config, X86};
//!
//! // A megabyte of RAM, and `mov ax, 0x1234` at the reset vector.
//! let ram = Arc::new(RamStore::new(0x10_0000));
//! for (i, b) in [0xb8u8, 0x34, 0x12].into_iter().enumerate() {
//!     ram.write_u8(0xffff0 + i as u64, b).unwrap();
//! }
//!
//! let mem = AddressSpace::new("mem", 20);
//! mem.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = X86::new(Config::default());   // an 8088
//! cpu.attach_space(Arc::new(mem));
//! cpu.step();                       // the reset sequence
//! assert_eq!(cpu.regs().cs, 0xffff);
//! cpu.step();                       // mov ax, 0x1234
//! assert_eq!(cpu.regs().rax & 0xffff, 0x1234);
//! ```
//!
//! A 386 or a 486 resets differently, and the difference is the one that
//! decides whether firmware runs: the `CS` *selector* is `f000` but its cached
//! *base* is `ffff0000`, so the first instruction is fetched from physical
//! `fffffff0`, sixteen bytes below the top of the address space. See
//! [`prot::Sys::reset`].
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction table — three opcode maps and the group tables — and the stream decoder both the interpreter and the disassembler use |
//! | [`disasm`] | the disassembler generated from those tables |
//! | [`prot`] | descriptors, selectors, the system register file, and the protected-mode control transfers |
//! | [`paging`] | the page-table walk and the translation-lookaside buffer |
//! | `exec` (private) | the interpreter, its flag rules, and the undefined-flag notes |
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): Intel's *iAPX 86/88, 186/188
//! User's Manual* and *8086 Family User's Manual* for the 16-bit parts (
//! bitsavers has the originals), the **80386 Programmer's Reference Manual**
//! for protection, paging, the exception model and the 32-bit instruction
//! forms, the *Intel SDM* volume 3 for the same material restated, and
//! sandpile.org's encoding tables for what the manuals leave out. Undefined
//! behaviour was measured against `SingleStepTests/8088` (MIT), which is
//! hardware output rather than anyone's emulator, and the 32-bit encodings
//! were cross-checked against GNU `as` and `objdump`.
//!
//! For **long mode**: the *Intel SDM* volume 2 for `REX`, the changed operand
//! sizes and the reclaimed encodings, and volume 3 chapters 4 (paging), 6
//! (the 64-bit interrupt descriptor table) and 9.8.5 (the activation
//! sequence); and the *AMD64 Architecture Programmer's Manual* volumes 2 and
//! 3, which are clearer on the parts AMD designed — the submodes, `SYSCALL`,
//! and which descriptor fields stop being read. Every non-obvious behaviour
//! carries its volume and section where it is implemented.
//!
//! **No copyleft emulator was consulted** — `docs/cpu/x86.md` names the three
//! that people reach for when x86 gets hard and records that all three are
//! forbidden.

pub mod disasm;
mod exec;
mod fpexec;
pub mod fpu;
pub mod isa;
pub mod paging;
pub mod prot;

#[cfg(test)]
mod tests;

// A real firmware image, read from wherever an environment variable points and
// never vendored — the same rule the conformance corpus follows
// (`ROADMAP.md` §1, §12).
#[cfg(all(test, feature = "std"))]
mod firmware;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    DebugTranslation, Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{
    FanIn, IntAck, IntAckCycle, IntAckHandlers, IntAckResponse, Level, LocalController, Resolve,
    WireId, WireSink,
};

use exec::{Exec, State};

/// The flags register.
///
/// Bits 1, 12, 13, 14 and 15 have no storage on an 8086: they read as one.
/// Bits 3 and 5 read as zero. That is not a convention this core invented —
/// it is what the corpus's captured flag words show on every vector, and
/// [`Regs::normalise_flags`] is where it is enforced.
pub mod flags {
    /// Carry.
    pub const CF: u32 = 0x0001;
    /// Parity of the low eight bits of the result.
    pub const PF: u32 = 0x0004;
    /// Auxiliary (BCD) carry out of bit 3.
    pub const AF: u32 = 0x0010;
    /// Zero.
    pub const ZF: u32 = 0x0040;
    /// Sign — a copy of the result's most significant bit.
    pub const SF: u32 = 0x0080;
    /// Trap: take a type-1 interrupt after every instruction.
    pub const TF: u32 = 0x0100;
    /// Interrupt enable, for the maskable `INTR` input only.
    pub const IF: u32 = 0x0200;
    /// Direction: string operations count down when set.
    pub const DF: u32 = 0x0400;
    /// Signed overflow.
    pub const OF: u32 = 0x0800;
    /// I/O privilege level, bits 12-13 (80286 and later).
    ///
    /// The highest privilege level at which `IN`, `OUT`, `CLI` and `STI` are
    /// allowed without consulting the task's I/O permission bitmap.
    pub const IOPL: u32 = 0x3000;
    /// How far to shift [`IOPL`] down to get the level itself.
    pub const IOPL_SHIFT: u32 = 12;
    /// Nested task, bit 14: `IRET` switches tasks back rather than returning.
    pub const NT: u32 = 0x4000;
    /// Resume, bit 16 (80386): suppress debug faults for one instruction.
    pub const RF: u32 = 0x0001_0000;
    /// Virtual 8086 mode, bit 17 (80386).
    pub const VM: u32 = 0x0002_0000;
    /// Alignment check, bit 18 (80486).
    pub const AC: u32 = 0x0004_0000;

    /// Every bit that has storage on an 8086.
    pub const DEFINED: u32 = CF | PF | AF | ZF | SF | TF | IF | DF | OF;

    /// The bits that always read as one on an 8086.
    ///
    /// Bits 12-15 gained meanings on the 80286 and 80386, which is why this
    /// is the *8086's* constant and not the architecture's.
    pub const RESERVED_SET: u32 = 0xf002;

    /// Every bit that has storage on an 80286 or 80386.
    pub const DEFINED_386: u32 = DEFINED | IOPL | NT | RF | VM;

    /// Every bit that has storage on an 80486.
    pub const DEFINED_486: u32 = DEFINED_386 | AC;

    /// Bit 1 reads as one on every part in the family.
    pub const ALWAYS_SET: u32 = 0x0002;

    /// The bits `POPF` and `IRET` may never write.
    ///
    /// `VM` and `RF` are settable only by a task switch or by `IRET` from a
    /// stack frame that has them; letting `POPF` set `VM` would be a
    /// privilege escalation, which is exactly why the 386 forbids it.
    pub const POPF_FORBIDDEN: u32 = VM | RF;

    /// The status bits `SAHF` and `LAHF` move between `AH` and the flags.
    pub const LOW_BYTE: u32 = CF | PF | AF | ZF | SF;
}

/// Which part this is.
///
/// A construction property rather than a second module, because the x86
/// generations are close to a superset chain: one interpreter covers all four,
/// and the differences that are *not* supersets — sixteen reclaimed opcodes,
/// the flags register growing bits, `PUSH SP` changing what it pushes, the
/// shift count gaining a mask — are selected here (`ROADMAP.md` §6.1.1, and
/// the same argument the 6502 core makes for its three parts).
///
/// Deliberately **not** `Ord`: `ROADMAP.md` §6.1.1's whole point is that
/// `if variant >= X` is the bug, and a type that cannot be compared cannot
/// grow one. Ask [`Features`] instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Variant {
    /// 16-bit external bus, six-byte prefetch queue, real mode only.
    I8086,
    /// 8-bit external bus, four-byte prefetch queue. The IBM PC's processor,
    /// and the one `SingleStepTests/8088` was captured from.
    I8088,
    /// The 80386: 32-bit registers and addressing, protected mode with
    /// descriptors and privilege levels, two-level paging, and the two-byte
    /// opcode map.
    I80386,
    /// The 80486: everything the 386 has, plus `CPUID`, `BSWAP`, `XADD`,
    /// `CMPXCHG`, `INVLPG`, the cache-control instructions, and `CR0.WP`.
    ///
    /// This is the variant firmware wants: SeaBIOS executes `CPUID`, which a
    /// 386 answers with an invalid-opcode exception.
    I80486,
    /// A generic x86-64 part: everything the 486 has, plus `CR4`, the
    /// model-specific registers, physical address extension, four-level
    /// paging, and long mode with its `REX` prefix and sixteen 64-bit
    /// registers.
    ///
    /// Deliberately not named after a chip. What distinguishes one x86-64 from
    /// another is a set of *independently selectable* extensions — [`Features`]
    /// — and a name here would imply a fixed bundle of them. This is the
    /// baseline: the architecture AMD introduced, with the pieces every part
    /// implementing it has to have.
    X86_64,
}

/// The extensions an instance has, selected independently of the part.
///
/// `ROADMAP.md` §6.1.1: a lattice, not a ladder. x86 gets this wrong more
/// often than most architectures because the marketing names *look* linear —
/// but `PAE` arrived on a Pentium Pro without long mode, `CMOV` on a Pentium
/// Pro but not on the contemporary Cyrix parts, `SYSCALL` on an AMD K6 with no
/// 64-bit anything, and `NX` on parts that shipped both with and without it
/// inside the same model number. So a decode site asks whether the *feature*
/// is present, never whether the variant is at least some other variant, and
/// there is no `PartialOrd` on [`Variant`] for it to reach for.
///
/// Total and un-`cfg`'d, as `riscv::Extensions` is: every field exists in
/// every build, so `Features` is one type with one shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Features {
    /// The 80486 additions: `CPUID`, `BSWAP`, `XADD`, `CMPXCHG`, `INVLPG`, the
    /// cache-control instructions, and `CR0.WP`.
    pub extras_486: bool,
    /// `CR4` exists. Everything below it in this struct needs a bit in it, so
    /// a part with an extension and no `CR4` is not expressible — which is
    /// correct, because no such part exists.
    pub cr4: bool,
    /// The model-specific registers, `RDMSR` and `WRMSR`.
    pub msr: bool,
    /// Physical address extension: `CR4.PAE`, 64-bit page-table entries, and
    /// the three-level walk.
    pub pae: bool,
    /// Long mode: `EFER.LME`/`LMA`, four-level paging, the `REX` prefix, the
    /// sixteen 64-bit registers, and 64-bit and compatibility submodes.
    ///
    /// Implies [`pae`](Features::pae) at *construction* — a long-mode part
    /// without physical address extension cannot exist, because the four-level
    /// walk is the PAE walk with a level added — and [`Features::validate`] says
    /// so rather than silently turning it on.
    pub long: bool,
    /// The no-execute page bit: `EFER.NXE` and bit 63 of a page-table entry.
    pub nx: bool,
    /// `SYSCALL` and `SYSRET`, with `STAR`, `LSTAR`, `CSTAR` and `SFMASK`.
    pub syscall: bool,
    /// The conditional moves, `CMOVcc` and their `FCMOV` counterparts. Only
    /// the integer half is implemented here; the floating-point half belongs
    /// to whoever lands x87.
    pub cmov: bool,
    /// The page-size extension: `CR4.PSE` and 4 MiB pages in a legacy
    /// two-level walk.
    pub pse: bool,
    /// Global pages: `CR4.PGE` and bit 8 of a page-table entry.
    pub pge: bool,
    /// An on-die x87 floating-point unit: the eight-register stack, the
    /// `D8`-`DF` escapes, and `CPUID` leaf 1's `FPU` bit.
    ///
    /// A *part* property rather than an architectural level, which is the
    /// whole reason it is here: an 80386 needed a separate 80387 and shipped
    /// far more often without one, a 486SX had the unit fused off, and a 486DX
    /// has it. All three are the same instruction set otherwise.
    pub fpu: bool,
    /// `CMPXCHG8B`, and `CPUID`'s `CX8` bit. A Pentium addition — the 486
    /// does not have it, and Linux checks for it by name.
    pub cx8: bool,
    /// `FXSAVE`/`FXRSTOR` and the `CR4.OSFXSR` bit that means something.
    ///
    /// Separable from [`sse`](Features::sse) in exactly one direction: a part
    /// can have `FXSR` without `SSE` — the Pentium II did — but not the other
    /// way, because `CR4.OSFXSR` is how an operating system says it has
    /// somewhere to save the SSE state. [`Features::validate`] enforces it.
    pub fxsr: bool,
    /// SSE: the sixteen `XMM` registers, `MXCSR`, and the single-precision
    /// scalar and packed instructions.
    pub sse: bool,
    /// SSE2: the double-precision half, the packed-integer logic, and the
    /// conversions between them. Requires [`sse`](Features::sse).
    pub sse2: bool,
}

impl Features {
    /// Nothing beyond the base architecture — an 8086, an 8088 or a 386.
    pub const NONE: Features = Features {
        extras_486: false,
        cr4: false,
        msr: false,
        pae: false,
        long: false,
        nx: false,
        syscall: false,
        cmov: false,
        pse: false,
        pge: false,
        fpu: false,
        cx8: false,
        fxsr: false,
        sse: false,
        sse2: false,
    };

    /// What an 80486DX has: the 486 additions and an on-die x87 unit.
    ///
    /// `CMPXCHG8B` is deliberately absent — it arrived with the Pentium, and a
    /// guest that probes `CPUID`'s `CX8` bit on a part claiming family 4 and
    /// finds it set has been told something no 486 was ever true of.
    pub const I80486: Features = Features {
        extras_486: true,
        fpu: true,
        ..Features::NONE
    };

    /// An 80486SX: the same part with the floating-point unit fused off.
    ///
    /// Here because it is the cleanest demonstration of what §6.1.1 is for —
    /// one bit of difference inside one model number, expressible without a
    /// second [`Variant`].
    pub const I80486SX: Features = Features {
        fpu: false,
        ..Features::I80486
    };

    /// What the baseline x86-64 part has.
    ///
    /// Every one of these is architecturally required of a processor that
    /// enters long mode (*AMD64 Architecture Programmer's Manual* volume 2
    /// §1.2, and the *Intel SDM* volume 3 §9.8.5 sequence, which cannot be
    /// executed without `CR4.PAE` and the `EFER` MSR).
    pub const X86_64: Features = Features {
        extras_486: true,
        cr4: true,
        msr: true,
        pae: true,
        long: true,
        nx: true,
        syscall: true,
        cmov: true,
        pse: true,
        pge: true,
        fpu: true,
        cx8: true,
        fxsr: true,
        sse: true,
        sse2: true,
    };

    /// Whether this combination describes a part that could exist.
    ///
    /// # Errors
    ///
    /// If an extension is selected whose prerequisite is not.
    pub fn validate(self) -> Result<()> {
        let missing = |need: &str, want: &str| {
            Err(Error::Property(alloc::format!(
                "an x86 with `{want}` must also have `{need}`"
            )))
        };
        if self.long && !self.pae {
            return missing("pae", "long");
        }
        if self.pae && !self.cr4 {
            return missing("cr4", "pae");
        }
        if (self.pse || self.pge) && !self.cr4 {
            return missing("cr4", "pse or pge");
        }
        if self.nx && !self.long {
            // `EFER.NXE` lives in the same MSR long mode is armed from, and no
            // part shipped one without the other.
            return missing("long", "nx");
        }
        if (self.msr || self.cr4) && !self.extras_486 {
            return missing("extras_486", "cr4 or msr");
        }
        if self.sse2 && !self.sse {
            return missing("sse", "sse2");
        }
        if self.sse && !self.fxsr {
            // `CR4.OSFXSR` is how an operating system says it has somewhere to
            // save the `XMM` registers; an `SSE` with no `FXSR` would be a
            // processor whose state no scheduler could preserve.
            return missing("fxsr", "sse");
        }
        if self.sse && !self.fpu {
            // The two share `CR0.EM` and `CR0.TS`, and `FXSAVE`'s image has
            // the x87 registers in it whether or not anything uses them.
            return missing("fpu", "sse");
        }
        if self.fxsr && !self.cr4 {
            return missing("cr4", "fxsr");
        }
        if self.long && !self.sse2 {
            // Not an implementation limit: every x86-64 part has SSE2, the
            // 64-bit ABI passes floating-point arguments in `XMM` registers,
            // and Linux refuses to boot without it. A long-mode part with it
            // switched off is not a processor anyone shipped.
            return missing("sse2", "long");
        }
        Ok(())
    }
}

impl Variant {
    /// How many bytes the prefetch queue holds.
    ///
    /// The 386 and 486 have deeper pipelines than a queue depth describes;
    /// sixteen bytes is the 386's prefetch queue and the number that bounds a
    /// legal instruction, which is what this is used for.
    #[must_use]
    pub const fn queue_bytes(self) -> u8 {
        match self {
            Variant::I8086 => 6,
            Variant::I8088 => 4,
            Variant::I80386 | Variant::I80486 | Variant::X86_64 => 16,
        }
    }

    /// How many bytes one bus cycle can transfer.
    #[must_use]
    pub const fn bus_bytes(self) -> u8 {
        match self {
            Variant::I8086 => 2,
            Variant::I8088 => 1,
            Variant::I80386 | Variant::I80486 => 4,
            // A 64-bit part moves eight bytes a cycle.
            Variant::X86_64 => 8,
        }
    }

    /// How many clocks one bus cycle costs with no wait states.
    ///
    /// Four T-states on an 8086, two on a 386 or 486.
    #[must_use]
    pub const fn bus_clocks(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => 4,
            Variant::I80386 | Variant::I80486 | Variant::X86_64 => 2,
        }
    }

    /// Which opcode map this part decodes with.
    #[must_use]
    pub const fn map(self) -> isa::Gen {
        match self {
            Variant::I8086 | Variant::I8088 => isa::Gen::I8086,
            Variant::I80386 | Variant::I80486 | Variant::X86_64 => isa::Gen::I386,
        }
    }

    /// Whether this part has 32-bit registers, protected mode and paging.
    #[must_use]
    pub const fn is_32bit(self) -> bool {
        matches!(self, Variant::I80386 | Variant::I80486 | Variant::X86_64)
    }

    /// The extensions this part has when nothing overrides them.
    ///
    /// A *preset*, in `ROADMAP.md` §6.1.1's sense: the public surface is a
    /// name and the name expands to a point in the lattice. Nothing downstream
    /// branches on the variant to decide whether an instruction exists — it
    /// asks the [`Features`] the instance was built with, which a machine
    /// description may narrow.
    #[must_use]
    pub const fn features(self) -> Features {
        match self {
            Variant::I8086 | Variant::I8088 | Variant::I80386 => Features::NONE,
            Variant::I80486 => Features::I80486,
            Variant::X86_64 => Features::X86_64,
        }
    }

    /// The bits the flags register has storage for.
    #[must_use]
    pub const fn flag_mask(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => flags::DEFINED,
            Variant::I80386 => flags::DEFINED_386,
            Variant::I80486 | Variant::X86_64 => flags::DEFINED_486,
        }
    }

    /// The bits the flags register always reads as one.
    #[must_use]
    pub const fn flag_fixed(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => flags::RESERVED_SET,
            Variant::I80386 | Variant::I80486 | Variant::X86_64 => flags::ALWAYS_SET,
        }
    }

    /// The value `EDX` holds after a reset: the processor's signature.
    ///
    /// The 386 and 486 both leave the family, model and stepping there, which
    /// is how software identified the part before `CPUID` existed. The 8086
    /// specifies nothing.
    #[must_use]
    pub const fn reset_signature(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => 0,
            // Family 3, model 0, stepping 8 — a late D-step 386DX.
            Variant::I80386 => 0x0000_0308,
            // Family 4, model 8 — a 486DX with CPUID.
            Variant::I80486 => 0x0000_0480,
            // Family 6, model 15, stepping 1 — what software that predates
            // `CPUID` reads out of `EDX`, and what leaf 1 reports below.
            Variant::X86_64 => 0x0000_06f1,
        }
    }

    /// The part's name, as a machine description writes it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::I8086 => "8086",
            Variant::I8088 => "8088",
            Variant::I80386 => "80386",
            Variant::I80486 => "80486",
            Variant::X86_64 => "x86-64",
        }
    }

    /// Every name a machine description may write.
    pub const NAMES: &'static [&'static str] = &["8086", "8088", "80386", "80486", "x86-64"];

    /// Look one up by the name a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        match name {
            "8086" => Some(Variant::I8086),
            "8088" => Some(Variant::I8088),
            "80386" | "386" | "i386" => Some(Variant::I80386),
            "80486" | "486" | "i486" => Some(Variant::I80486),
            "x86-64" | "x86_64" | "amd64" | "x64" => Some(Variant::X86_64),
            _ => None,
        }
    }
}

impl fmt::Display for Variant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How this particular part is configured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Which member of the family this is.
    pub variant: Variant,
    /// Which extensions this instance has.
    ///
    /// Defaults to [`Variant::features`] and may be narrowed from there: a
    /// part that has `CPUID` but no `CMOV` is a real configuration, and it is
    /// the one a guest probes for.
    pub features: Features,
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// An 8088: 8-bit bus, four-byte queue.
    pub const I8088: Config = Config {
        variant: Variant::I8088,
        features: Features::NONE,
        requester: RequesterId::ANONYMOUS,
    };

    /// An 8086: 16-bit bus, six-byte queue.
    pub const I8086: Config = Config {
        variant: Variant::I8086,
        ..Config::I8088
    };

    /// An 80386.
    pub const I80386: Config = Config {
        variant: Variant::I80386,
        ..Config::I8088
    };

    /// An 80486 — the variant a PC firmware image expects.
    pub const I80486: Config = Config {
        variant: Variant::I80486,
        features: Features::I80486,
        ..Config::I8088
    };

    /// A baseline x86-64 part, with long mode available but not entered:
    /// a processor still resets into real mode however wide it is.
    pub const X86_64: Config = Config {
        variant: Variant::X86_64,
        features: Features::X86_64,
        ..Config::I8088
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Same configuration, as a different part **with that part's features**.
    ///
    /// The features come along because a variant name is a preset, not a bus
    /// width: asking for an x86-64 and getting one that cannot enter long mode
    /// would be the silent downgrade §6.1.1 forbids. Narrow them afterwards
    /// with [`with_features`](Config::with_features) if that is what is meant.
    #[must_use]
    pub const fn with_variant(mut self, variant: Variant) -> Self {
        self.variant = variant;
        self.features = variant.features();
        self
    }

    /// Same configuration, with the extension set replaced.
    #[must_use]
    pub const fn with_features(mut self, features: Features) -> Self {
        self.features = features;
        self
    }
}

impl Default for Config {
    /// An 8088, because that is the part the conformance corpus was captured
    /// from and therefore the one whose behaviour here is checked against
    /// silicon. A machine that wants protected mode asks for it by name.
    fn default() -> Self {
        Config::I8088
    }
}

/// The physical address a `segment:offset` pair names.
///
/// The 8086 has no address translation: the segment is shifted left four bits
/// and added to the offset in a 20-bit adder. The mask is the whole point —
/// `0xffff:0x0010` is `0x100000`, which wraps to `0x00000`, and DOS software
/// used that wrap deliberately. The IBM AT's A20 gate exists because the
/// 80286 stopped wrapping and broke those programs.
#[inline]
#[must_use]
pub const fn linear(segment: u16, offset: u16) -> u64 {
    // Computed in the guest's own twenty-bit width and *then* widened: the
    // wrap at 1 MiB is the whole point of the function, and summing in
    // sixty-four bits and masking afterwards would agree only by accident.
    ((((segment as u32) << 4).wrapping_add(offset as u32)) & 0xf_ffff) as u64
}

/// The architectural register file.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read
/// it out and put it back — this is the surface a future gdbstub serialises
/// (`ROADMAP.md` §9's debug story), and [`Reg`] enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Regs {
    /// Accumulator. `EAX` is its low half, `AX` the low quarter, and `AL`/`AH`
    /// its low two bytes.
    pub rax: u64,
    /// Count — the implicit operand of `LOOP`, the string repeats, and the
    /// variable shifts.
    pub rcx: u64,
    /// Data — the implicit high half of a multiply or divide, and the I/O port
    /// register.
    pub rdx: u64,
    /// Base.
    pub rbx: u64,
    /// Stack pointer, in `SS`.
    pub rsp: u64,
    /// Base pointer; addressing modes that use it default to `SS`.
    pub rbp: u64,
    /// Source index, for the string instructions.
    pub rsi: u64,
    /// Destination index, for the string instructions.
    pub rdi: u64,
    /// The eight registers long mode added, `R8` through `R15`.
    ///
    /// An array rather than eight named fields because nothing names one of
    /// them implicitly: they exist only as ModRM numbers 8-15, reachable only
    /// behind a `REX` prefix, so indexing is the only access there is.
    pub r: [u64; 8],
    /// Instruction pointer, an offset within `CS`.
    pub rip: u64,
    /// The flags register. See [`flags`].
    ///
    /// Thirty-two bits even on a 64-bit part: `RFLAGS` bits 63-32 are reserved
    /// and read as zero (*Intel SDM* volume 1 §3.4.3), so a `u32` is the whole
    /// register rather than a truncation of one. `PUSHFQ` zero-extends.
    pub eflags: u32,
    /// Extra segment — always the destination of a string instruction.
    pub es: u16,
    /// Code segment.
    pub cs: u16,
    /// Stack segment.
    pub ss: u16,
    /// Data segment.
    pub ds: u16,
    /// The 80386's first extra segment. No instruction uses it implicitly.
    pub fs: u16,
    /// The 80386's second extra segment.
    pub gs: u16,
}

/// The 16-bit register order used by the ModRM `reg` and `rm` fields.
const WORD_ORDER: [Reg; 8] = [
    Reg::Ax,
    Reg::Cx,
    Reg::Dx,
    Reg::Bx,
    Reg::Sp,
    Reg::Bp,
    Reg::Si,
    Reg::Di,
];

/// The 32-bit register order used by the same fields.
const DWORD_ORDER: [Reg; 8] = [
    Reg::Eax,
    Reg::Ecx,
    Reg::Edx,
    Reg::Ebx,
    Reg::Esp,
    Reg::Ebp,
    Reg::Esi,
    Reg::Edi,
];

impl Regs {
    /// The state a power-on reset leaves an 8086 in.
    ///
    /// `CS:IP` is `ffff:0000`, sixteen bytes below the top of memory, which is
    /// why every PC has a far jump there. Every other segment register is
    /// zero, and the flags hold only their hard-wired bits.
    ///
    /// A 386 resets differently — `f000:fff0` with a `CS` *base* of
    /// `ffff0000`, which is not expressible in this struct alone — so
    /// [`Variant`]-aware reset lives in the interpreter, where the hidden
    /// descriptor caches it has to set are also in scope.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            rax: 0,
            rcx: 0,
            rdx: 0,
            rbx: 0,
            rsp: 0,
            rbp: 0,
            rsi: 0,
            rdi: 0,
            r: [0; 8],
            rip: 0,
            eflags: flags::RESERVED_SET,
            es: 0,
            cs: 0xffff,
            ss: 0,
            ds: 0,
            fs: 0,
            gs: 0,
        }
    }

    /// Force the flags word's hard-wired bits into shape for a given part.
    ///
    /// Applied on every write to the flags register, not only on
    /// `POPF`/`IRET`: the bits have no storage, so a value that has been
    /// through this is the only value the register can ever hold. The part
    /// matters because bits 12-15 have storage on a 386 and none on an 8086.
    #[inline]
    #[must_use]
    pub const fn normalise_flags(variant: Variant, value: u32) -> u32 {
        (value & variant.flag_mask()) | variant.flag_fixed()
    }

    /// Whether a flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u32) -> bool {
        self.eflags & mask != 0
    }

    /// The current I/O privilege level, 0 to 3.
    #[inline]
    #[must_use]
    pub const fn iopl(&self) -> u8 {
        ((self.eflags & flags::IOPL) >> flags::IOPL_SHIFT) as u8
    }

    /// Read one of the sixteen 64-bit registers by ModRM number.
    ///
    /// Numbers 8-15 are `R8`-`R15`, which only a `REX` prefix can name; a
    /// decoder that never sets one never produces them.
    #[inline]
    #[must_use]
    pub const fn qword(&self, index: u8) -> u64 {
        match index & 15 {
            0 => self.rax,
            1 => self.rcx,
            2 => self.rdx,
            3 => self.rbx,
            4 => self.rsp,
            5 => self.rbp,
            6 => self.rsi,
            7 => self.rdi,
            n => self.r[(n - 8) as usize],
        }
    }

    /// Write one of the sixteen 64-bit registers by ModRM number.
    #[inline]
    pub const fn set_qword(&mut self, index: u8, value: u64) {
        match index & 15 {
            0 => self.rax = value,
            1 => self.rcx = value,
            2 => self.rdx = value,
            3 => self.rbx = value,
            4 => self.rsp = value,
            5 => self.rbp = value,
            6 => self.rsi = value,
            7 => self.rdi = value,
            n => self.r[(n - 8) as usize] = value,
        }
    }

    /// Read one of the sixteen 32-bit registers by ModRM number.
    #[inline]
    #[must_use]
    pub const fn dword(&self, index: u8) -> u32 {
        self.qword(index) as u32
    }

    /// Write one of the sixteen 32-bit registers by ModRM number.
    ///
    /// The upper half is **zeroed**, not preserved: a 32-bit result in 64-bit
    /// mode is zero-extended into the whole register (*Intel SDM* volume 1
    /// §3.4.1.1), which is the rule that makes `mov eax, eax` a truncation. On
    /// a part with no upper half there is nothing to zero, so this is one rule
    /// rather than two.
    #[inline]
    pub const fn set_dword(&mut self, index: u8, value: u32) {
        self.set_qword(index, value as u64);
    }

    /// Read one of the sixteen 16-bit registers by ModRM number.
    #[inline]
    #[must_use]
    pub const fn word(&self, index: u8) -> u16 {
        self.qword(index) as u16
    }

    /// Write one of the sixteen 16-bit registers by ModRM number.
    ///
    /// The high bits are **preserved**, which is the 386's rule and not an
    /// implementation convenience: `mov ax, 0` leaves the top of `EAX` alone,
    /// and code that switches between operand sizes depends on it. Note the
    /// asymmetry with [`set_dword`](Regs::set_dword) — it is the architecture's,
    /// not ours.
    #[inline]
    pub const fn set_word(&mut self, index: u8, value: u16) {
        let merged = (self.qword(index) & !0xffff) | value as u64;
        self.set_qword(index, merged);
    }

    /// Read one of the eight legacy 8-bit registers by ModRM number.
    ///
    /// Numbers 0-3 are the low halves of `AX`-`BX` and 4-7 the high halves, in
    /// the same register order — which is why `AH` is 4 and not 1. This is the
    /// encoding with **no** `REX` prefix; see [`byte_rex`](Regs::byte_rex) for
    /// the other one.
    #[inline]
    #[must_use]
    pub const fn byte(&self, index: u8) -> u8 {
        let word = self.word(index & 3);
        if index & 4 == 0 {
            word as u8
        } else {
            (word >> 8) as u8
        }
    }

    /// Write one of the eight legacy 8-bit registers by ModRM number.
    #[inline]
    pub const fn set_byte(&mut self, index: u8, value: u8) {
        let word = self.word(index & 3);
        let merged = if index & 4 == 0 {
            (word & 0xff00) | value as u16
        } else {
            (word & 0x00ff) | ((value as u16) << 8)
        };
        self.set_word(index & 3, merged);
    }

    /// Read the low byte of one of the sixteen registers.
    ///
    /// The presence of a `REX` prefix — *any* `REX` prefix, including `40`,
    /// which sets no bit at all — replaces `AH`, `CH`, `DH` and `BH` with
    /// `SPL`, `BPL`, `SIL` and `DIL` (*Intel SDM* volume 2 §2.2.1.2). That is
    /// why this is a second accessor rather than a wider index into the first:
    /// register number 4 means two different things depending on a prefix that
    /// carries no operand of its own.
    #[inline]
    #[must_use]
    pub const fn byte_rex(&self, index: u8) -> u8 {
        self.qword(index) as u8
    }

    /// Write the low byte of one of the sixteen registers.
    #[inline]
    pub const fn set_byte_rex(&mut self, index: u8, value: u8) {
        let merged = (self.qword(index) & !0xff) | value as u64;
        self.set_qword(index, merged);
    }

    /// Read a general register at a width of 1, 2, 4 or 8 bytes.
    ///
    /// `rex` says whether the instruction carried a `REX` prefix, which only
    /// changes what a byte-sized register number 4-7 names.
    #[inline]
    #[must_use]
    pub const fn read(&self, index: u8, size: u8, rex: bool) -> u64 {
        match size {
            1 if rex => self.byte_rex(index) as u64,
            1 => self.byte(index) as u64,
            2 => self.word(index) as u64,
            4 => self.dword(index) as u64,
            _ => self.qword(index),
        }
    }

    /// Write a general register at a width of 1, 2, 4 or 8 bytes.
    #[inline]
    pub const fn write(&mut self, index: u8, size: u8, rex: bool, value: u64) {
        match size {
            1 if rex => self.set_byte_rex(index, value as u8),
            1 => self.set_byte(index, value as u8),
            2 => self.set_word(index, value as u16),
            4 => self.set_dword(index, value as u32),
            _ => self.set_qword(index, value),
        }
    }

    /// Read a segment register by number: `ES`, `CS`, `SS`, `DS`, `FS`, `GS`.
    ///
    /// Numbers 6 and 7 have no register; they read as zero rather than
    /// aliasing, because a 386 rejects them and an 8086 never produces them
    /// (its decoder masks the field to two bits before it gets here).
    #[inline]
    #[must_use]
    pub const fn segment(&self, index: u8) -> u16 {
        match index {
            0 => self.es,
            1 => self.cs,
            2 => self.ss,
            3 => self.ds,
            4 => self.fs,
            5 => self.gs,
            _ => 0,
        }
    }

    /// Write a segment register by number.
    #[inline]
    pub const fn set_segment(&mut self, index: u8, value: u16) {
        match index {
            0 => self.es = value,
            1 => self.cs = value,
            2 => self.ss = value,
            3 => self.ds = value,
            4 => self.fs = value,
            5 => self.gs = value,
            _ => {}
        }
    }
}

impl Default for Regs {
    fn default() -> Self {
        Regs::new()
    }
}

impl fmt::Display for Regs {
    /// The two-line form a trace log wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "RAX:{:016x} RBX:{:016x} RCX:{:016x} RDX:{:016x} RSP:{:016x} RBP:{:016x} \
             RSI:{:016x} RDI:{:016x} ES:{:04x} CS:{:04x} SS:{:04x} DS:{:04x} FS:{:04x} \
             GS:{:04x} RIP:{:016x} F:{:08x}",
            self.rax,
            self.rbx,
            self.rcx,
            self.rdx,
            self.rsp,
            self.rbp,
            self.rsi,
            self.rdi,
            self.es,
            self.cs,
            self.ss,
            self.ds,
            self.fs,
            self.gs,
            self.rip,
            self.eflags
        )
    }
}

/// One named register, for a debugger that works by name or index.
///
/// Both widths are here on purpose. The 32-bit names are the architectural
/// registers and are what [`Reg::ALL`] — and therefore the snapshot and the
/// gdb register map — walks; the 16-bit names are *views* into their low
/// halves, so a monitor, a test vector or a corpus that speaks 8086 can name
/// `ax` and get the sixteen bits it means without knowing about the other
/// sixteen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// Accumulator, 32 bits.
    Eax,
    /// Count, 32 bits.
    Ecx,
    /// Data, 32 bits.
    Edx,
    /// Base, 32 bits.
    Ebx,
    /// Stack pointer, 32 bits.
    Esp,
    /// Base pointer, 32 bits.
    Ebp,
    /// Source index, 32 bits.
    Esi,
    /// Destination index, 32 bits.
    Edi,
    /// Instruction pointer, 32 bits.
    Eip,
    /// Flags, 32 bits.
    Eflags,
    /// Extra segment.
    Es,
    /// Code segment.
    Cs,
    /// Stack segment.
    Ss,
    /// Data segment.
    Ds,
    /// First 386 extra segment.
    Fs,
    /// Second 386 extra segment.
    Gs,
    /// The low half of the accumulator.
    Ax,
    /// The low half of the count register.
    Cx,
    /// The low half of the data register.
    Dx,
    /// The low half of the base register.
    Bx,
    /// The low half of the stack pointer.
    Sp,
    /// The low half of the base pointer.
    Bp,
    /// The low half of the source index.
    Si,
    /// The low half of the destination index.
    Di,
    /// The low half of the instruction pointer.
    Ip,
    /// The low half of the flags register.
    Flags,
    /// Accumulator, 64 bits.
    Rax,
    /// Count, 64 bits.
    Rcx,
    /// Data, 64 bits.
    Rdx,
    /// Base, 64 bits.
    Rbx,
    /// Stack pointer, 64 bits.
    Rsp,
    /// Base pointer, 64 bits.
    Rbp,
    /// Source index, 64 bits.
    Rsi,
    /// Destination index, 64 bits.
    Rdi,
    /// The first of the eight registers long mode added.
    R8,
    /// `R9`.
    R9,
    /// `R10`.
    R10,
    /// `R11`.
    R11,
    /// `R12`.
    R12,
    /// `R13`.
    R13,
    /// `R14`.
    R14,
    /// `R15`.
    R15,
    /// Instruction pointer, 64 bits.
    Rip,
}

impl Reg {
    /// The architectural registers, in the order a debugger should list them
    /// and the order the snapshot writes them.
    ///
    /// This is gdb's i386 core ordering — the eight general registers, `EIP`,
    /// `EFLAGS`, then the six selectors — which is why the first sixty-four
    /// bytes of a saved core are directly usable as a `g` packet.
    pub const ALL: &'static [Reg] = &[
        Reg::Eax,
        Reg::Ecx,
        Reg::Edx,
        Reg::Ebx,
        Reg::Esp,
        Reg::Ebp,
        Reg::Esi,
        Reg::Edi,
        Reg::Eip,
        Reg::Eflags,
        Reg::Cs,
        Reg::Ss,
        Reg::Ds,
        Reg::Es,
        Reg::Fs,
        Reg::Gs,
    ];

    /// The 16-bit views, which [`Reg::ALL`] deliberately omits: they alias
    /// registers already in it, and a snapshot that wrote both would have two
    /// copies of the same state.
    pub const NARROW: &'static [Reg] = &[
        Reg::Ax,
        Reg::Cx,
        Reg::Dx,
        Reg::Bx,
        Reg::Sp,
        Reg::Bp,
        Reg::Si,
        Reg::Di,
        Reg::Ip,
        Reg::Flags,
    ];

    /// The sixteen 64-bit general registers and `RIP`, in ModRM number order.
    ///
    /// Deliberately *not* in [`Reg::ALL`]: the first eight alias registers
    /// already there, and the snapshot writes this block separately so that
    /// [`Reg::ALL`]'s prefix stays the gdb i386 core block it has always been.
    pub const WIDE: &'static [Reg] = &[
        Reg::Rax,
        Reg::Rcx,
        Reg::Rdx,
        Reg::Rbx,
        Reg::Rsp,
        Reg::Rbp,
        Reg::Rsi,
        Reg::Rdi,
        Reg::R8,
        Reg::R9,
        Reg::R10,
        Reg::R11,
        Reg::R12,
        Reg::R13,
        Reg::R14,
        Reg::R15,
        Reg::Rip,
    ];

    /// The register's name, lower case, as gdb and the monitor spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::Eax => "eax",
            Reg::Ecx => "ecx",
            Reg::Edx => "edx",
            Reg::Ebx => "ebx",
            Reg::Esp => "esp",
            Reg::Ebp => "ebp",
            Reg::Esi => "esi",
            Reg::Edi => "edi",
            Reg::Eip => "eip",
            Reg::Eflags => "eflags",
            Reg::Es => "es",
            Reg::Cs => "cs",
            Reg::Ss => "ss",
            Reg::Ds => "ds",
            Reg::Fs => "fs",
            Reg::Gs => "gs",
            Reg::Ax => "ax",
            Reg::Cx => "cx",
            Reg::Dx => "dx",
            Reg::Bx => "bx",
            Reg::Sp => "sp",
            Reg::Bp => "bp",
            Reg::Si => "si",
            Reg::Di => "di",
            Reg::Ip => "ip",
            Reg::Flags => "flags",
            Reg::Rax => "rax",
            Reg::Rcx => "rcx",
            Reg::Rdx => "rdx",
            Reg::Rbx => "rbx",
            Reg::Rsp => "rsp",
            Reg::Rbp => "rbp",
            Reg::Rsi => "rsi",
            Reg::Rdi => "rdi",
            Reg::R8 => "r8",
            Reg::R9 => "r9",
            Reg::R10 => "r10",
            Reg::R11 => "r11",
            Reg::R12 => "r12",
            Reg::R13 => "r13",
            Reg::R14 => "r14",
            Reg::R15 => "r15",
            Reg::Rip => "rip",
        }
    }

    /// How wide the register is.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::Eax
            | Reg::Ecx
            | Reg::Edx
            | Reg::Ebx
            | Reg::Esp
            | Reg::Ebp
            | Reg::Esi
            | Reg::Edi
            | Reg::Eip
            | Reg::Eflags => Width::U32,
            Reg::Rax
            | Reg::Rcx
            | Reg::Rdx
            | Reg::Rbx
            | Reg::Rsp
            | Reg::Rbp
            | Reg::Rsi
            | Reg::Rdi
            | Reg::R8
            | Reg::R9
            | Reg::R10
            | Reg::R11
            | Reg::R12
            | Reg::R13
            | Reg::R14
            | Reg::R15
            | Reg::Rip => Width::U64,
            _ => Width::U16,
        }
    }

    /// Read this register out of a register file.
    ///
    /// A 16-bit view returns its low half zero-extended, so a caller that
    /// asked for `ax` never sees the other sixteen bits by accident.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u64 {
        match self {
            Reg::Eax => regs.rax as u32 as u64,
            Reg::Ecx => regs.rcx as u32 as u64,
            Reg::Edx => regs.rdx as u32 as u64,
            Reg::Ebx => regs.rbx as u32 as u64,
            Reg::Esp => regs.rsp as u32 as u64,
            Reg::Ebp => regs.rbp as u32 as u64,
            Reg::Esi => regs.rsi as u32 as u64,
            Reg::Edi => regs.rdi as u32 as u64,
            Reg::Eip => regs.rip as u32 as u64,
            Reg::Eflags => regs.eflags as u64,
            Reg::Es => regs.es as u64,
            Reg::Cs => regs.cs as u64,
            Reg::Ss => regs.ss as u64,
            Reg::Ds => regs.ds as u64,
            Reg::Fs => regs.fs as u64,
            Reg::Gs => regs.gs as u64,
            Reg::Ax => regs.rax & 0xffff,
            Reg::Cx => regs.rcx & 0xffff,
            Reg::Dx => regs.rdx & 0xffff,
            Reg::Bx => regs.rbx & 0xffff,
            Reg::Sp => regs.rsp & 0xffff,
            Reg::Bp => regs.rbp & 0xffff,
            Reg::Si => regs.rsi & 0xffff,
            Reg::Di => regs.rdi & 0xffff,
            Reg::Ip => regs.rip & 0xffff,
            Reg::Flags => (regs.eflags & 0xffff) as u64,
            Reg::Rax => regs.rax,
            Reg::Rcx => regs.rcx,
            Reg::Rdx => regs.rdx,
            Reg::Rbx => regs.rbx,
            Reg::Rsp => regs.rsp,
            Reg::Rbp => regs.rbp,
            Reg::Rsi => regs.rsi,
            Reg::Rdi => regs.rdi,
            Reg::R8 => regs.r[0],
            Reg::R9 => regs.r[1],
            Reg::R10 => regs.r[2],
            Reg::R11 => regs.r[3],
            Reg::R12 => regs.r[4],
            Reg::R13 => regs.r[5],
            Reg::R14 => regs.r[6],
            Reg::R15 => regs.r[7],
            Reg::Rip => regs.rip,
        }
    }

    /// Write this register into a register file.
    ///
    /// A 16-bit view leaves the high half alone, exactly as a 16-bit
    /// instruction does. Nothing here normalises the flags: the hard-wired
    /// bits depend on the [`Variant`], which a bare register file does not
    /// know, so [`X86::set_reg`] does it where the part is in scope.
    pub const fn set(self, regs: &mut Regs, value: u64) {
        match self {
            // The 32-bit names write a 32-bit register, and a 32-bit write
            // zero-extends — the same rule [`Regs::set_dword`] states, applied
            // here so a debugger's `set eax` behaves as an instruction's would.
            Reg::Eax => regs.rax = value as u32 as u64,
            Reg::Ecx => regs.rcx = value as u32 as u64,
            Reg::Edx => regs.rdx = value as u32 as u64,
            Reg::Ebx => regs.rbx = value as u32 as u64,
            Reg::Esp => regs.rsp = value as u32 as u64,
            Reg::Ebp => regs.rbp = value as u32 as u64,
            Reg::Esi => regs.rsi = value as u32 as u64,
            Reg::Edi => regs.rdi = value as u32 as u64,
            Reg::Eip => regs.rip = value as u32 as u64,
            Reg::Eflags => regs.eflags = value as u32,
            Reg::Es => regs.es = value as u16,
            Reg::Cs => regs.cs = value as u16,
            Reg::Ss => regs.ss = value as u16,
            Reg::Ds => regs.ds = value as u16,
            Reg::Fs => regs.fs = value as u16,
            Reg::Gs => regs.gs = value as u16,
            Reg::Ax => regs.rax = (regs.rax & !0xffff) | (value & 0xffff),
            Reg::Cx => regs.rcx = (regs.rcx & !0xffff) | (value & 0xffff),
            Reg::Dx => regs.rdx = (regs.rdx & !0xffff) | (value & 0xffff),
            Reg::Bx => regs.rbx = (regs.rbx & !0xffff) | (value & 0xffff),
            Reg::Sp => regs.rsp = (regs.rsp & !0xffff) | (value & 0xffff),
            Reg::Bp => regs.rbp = (regs.rbp & !0xffff) | (value & 0xffff),
            Reg::Si => regs.rsi = (regs.rsi & !0xffff) | (value & 0xffff),
            Reg::Di => regs.rdi = (regs.rdi & !0xffff) | (value & 0xffff),
            Reg::Ip => regs.rip = (regs.rip & !0xffff) | (value & 0xffff),
            Reg::Flags => regs.eflags = (regs.eflags & 0xffff_0000) | (value as u32 & 0xffff),
            Reg::Rax => regs.rax = value,
            Reg::Rcx => regs.rcx = value,
            Reg::Rdx => regs.rdx = value,
            Reg::Rbx => regs.rbx = value,
            Reg::Rsp => regs.rsp = value,
            Reg::Rbp => regs.rbp = value,
            Reg::Rsi => regs.rsi = value,
            Reg::Rdi => regs.rdi = value,
            Reg::R8 => regs.r[0] = value,
            Reg::R9 => regs.r[1] = value,
            Reg::R10 => regs.r[2] = value,
            Reg::R11 => regs.r[3] = value,
            Reg::R12 => regs.r[4] = value,
            Reg::R13 => regs.r[5] = value,
            Reg::R14 => regs.r[6] = value,
            Reg::R15 => regs.r[7] = value,
            Reg::Rip => regs.rip = value,
        }
    }

    /// Look a register up by name, 16-bit views included.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL
            .iter()
            .chain(Reg::NARROW.iter())
            .chain(Reg::WIDE.iter())
            .copied()
            .find(|r| r.name() == name)
    }

    /// The register the ModRM `reg`/`rm` field selects for a word operand.
    #[must_use]
    pub const fn from_word_index(index: u8) -> Reg {
        WORD_ORDER[(index & 7) as usize]
    }

    /// The register the ModRM `reg`/`rm` field selects for a dword operand.
    #[must_use]
    pub const fn from_dword_index(index: u8) -> Reg {
        DWORD_ORDER[(index & 7) as usize]
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which interrupt a poll latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// Maskable, level-sensitive, and vectored by whatever answers the
    /// acknowledge cycle. Taken only while `IF` is set.
    Intr,
    /// Non-maskable, edge-sensitive, vectored through entry 2. `IF` does not
    /// gate it.
    Nmi,
}

/// The interrupt input pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device asserting
/// `INTR` from inside a write the CPU itself issued would otherwise re-enter
/// the CPU's own critical section, which is a deadlock under `native-std` and
/// a panic under `single`. The re-entrancy contract says mutate your own state
/// in a short critical section and act outward afterwards; a pin that is one
/// atomic store needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug)]
pub(crate) struct Lines {
    /// `INTR` is level-sensitive: it is taken whenever it is asserted and `IF`
    /// is set.
    intr: AtomicBool,
    /// The vector the acknowledge cycle would return.
    ///
    /// On a real PC the 8259A drives it onto the data bus during the second
    /// `INTA` cycle. Modelling it as a latched byte rather than a bus
    /// transaction keeps the interrupt controller out of the CPU's type
    /// signature (`ROADMAP.md` §15, invariant 1); a controller that wants the
    /// real handshake sets this from its own acknowledge path.
    intr_vector: AtomicU32,
    /// The last level seen on NMI, for edge detection.
    nmi_level: AtomicBool,
    /// NMI is edge-sensitive: a rising edge sets this latch, which stays set
    /// until the interrupt is serviced, however long that takes.
    nmi_latch: AtomicBool,
    /// A reset asked for by the `reset` pin, latched until the next step folds
    /// it into the execution state.
    ///
    /// A latch rather than a write into `State::reset_pending`, because a wire
    /// is driven from inside whatever device changed it — a `RESET` written to
    /// port `0x92` arrives from inside an `OUT` this very core issued — and
    /// reaching for the session lock there would re-enter the core's own
    /// critical section (`ROADMAP.md` §4.7).
    reset: AtomicBool,
    /// The mask a physical address is `AND`ed with on its way to the bus:
    /// either all ones, or all ones with bit 20 clear.
    ///
    /// The A20 gate is not a processor feature and this file says so in its
    /// module documentation: on a real PC the gate sits in the chipset, between
    /// the CPU and memory. rsemu has no device between an initiator and its
    /// address space to put it in, and the gate is *exactly* a suppression of
    /// the address wrap this core does for itself, so it is modelled here as an
    /// input pin (`a20`) and one mask.
    ///
    /// All ones until something drives that pin, so a machine that wires no
    /// gate behaves as it always did rather than losing its odd megabytes to a
    /// chip it does not have.
    a20_mask: AtomicU32,
    /// Whether an `a20` pin exists at all.
    ///
    /// The distinction the mask alone cannot make: a board with no gate has
    /// bit 20 permanently connected, while a board that wires one takes the
    /// level off its net. Between `connect` and the realize sweep there is no
    /// level to take, so the mask sits shut — a fresh net is low — and the
    /// sweep replaces it with what the chipset actually drives before anything
    /// executes. After that the pin is never invented again, reset included.
    a20_wired: AtomicBool,
    /// What answers the `INTR` acknowledge cycle, if a controller drives it.
    ///
    /// An AT has exactly one controller here — the slave 8259A hangs off the
    /// master's `IR2`, not off `INTR`, and the master delegates to it inside
    /// its own acknowledge — but the seam is a list all the same, so a board
    /// with two controllers wired to one `INTR` net is expressible rather than
    /// silently mis-served.
    ///
    /// Weak references, behind a leaf lock released before each outward call:
    /// the machine owns both devices, a CPU that kept its PIC alive would close
    /// a cycle nothing could drop (`ROADMAP.md` §4.3), and the controller is
    /// free to take its own locks.
    acks: IntAckHandlers,
    /// The `INIT` pin's level, as whatever drives that net says.
    ///
    /// Separate from [`init_peer`](Lines::init_peer) because they are two
    /// different drivers of one architectural condition and a single flag could
    /// not tell which of them dropped it — the wired-OR bug `ROADMAP.md` §4.3
    /// names, in miniature.
    init_pin: AtomicBool,
    /// The same condition as this core's own local controller reports it.
    init_peer: AtomicBool,
    /// A rising edge on either, latched until a step folds it into the
    /// execution state — exactly as [`reset`](Lines::reset) is, and for the
    /// same re-entrancy reason.
    init_latch: AtomicBool,
    /// The page a Start-Up named, or [`NO_STARTUP`] for none.
    ///
    /// A `u32` rather than an `Option<u8>` because it is written from inside a
    /// write this core issued — the bootstrap processor's store to its own
    /// interrupt command register — and reaching for a lock there would
    /// re-enter this core's critical section (§4.7).
    startup: AtomicU32,
    /// This core's own interrupt controller, if a machine wired one.
    ///
    /// Weak and behind a leaf lock, for the reason [`acks`](Lines::acks) is.
    /// Asked once per instruction boundary — see [`has_intc`](Lines::has_intc)
    /// for what keeps that free on a machine that has no such controller.
    intc: sync::Mutex<Option<Weak<dyn LocalController>>>,
    /// Whether [`intc`](Lines::intc) holds anything.
    ///
    /// The hot path's gate: one relaxed load per instruction on a machine with
    /// no local controller, rather than a lock acquisition per instruction.
    has_intc: AtomicBool,
}

/// [`Lines::startup`] when no Start-Up is waiting. Not a page: a Start-Up's
/// vector is eight bits, so no real value can collide with it.
const NO_STARTUP: u32 = u32::MAX;

impl Default for Lines {
    fn default() -> Lines {
        Lines {
            intr: AtomicBool::new(false),
            intr_vector: AtomicU32::new(0),
            nmi_level: AtomicBool::new(false),
            nmi_latch: AtomicBool::new(false),
            reset: AtomicBool::new(false),
            a20_mask: AtomicU32::new(u32::MAX),
            a20_wired: AtomicBool::new(false),
            acks: IntAckHandlers::new(),
            init_pin: AtomicBool::new(false),
            init_peer: AtomicBool::new(false),
            init_latch: AtomicBool::new(false),
            startup: AtomicU32::new(NO_STARTUP),
            intc: sync::Mutex::new(None),
            has_intc: AtomicBool::new(false),
        }
    }
}

impl Lines {
    fn set_intr(&self, asserted: bool) {
        self.intr.store(asserted, Ordering::Release);
    }

    fn intr_asserted(&self) -> bool {
        self.intr.load(Ordering::Acquire)
    }

    fn set_intr_vector(&self, vector: u8) {
        self.intr_vector.store(u32::from(vector), Ordering::Release);
    }

    pub(crate) fn intr_vector(&self) -> u8 {
        self.intr_vector.load(Ordering::Acquire) as u8
    }

    /// Drive the NMI pin, latching a rising edge.
    fn set_nmi(&self, asserted: bool) {
        let previous = self.nmi_level.swap(asserted, Ordering::AcqRel);
        if asserted && !previous {
            self.nmi_latch.store(true, Ordering::Release);
        }
    }

    pub(crate) fn nmi_pending(&self) -> bool {
        self.nmi_latch.load(Ordering::Acquire)
    }

    /// Consume the NMI latch, reporting whether it was set.
    pub(crate) fn take_nmi_pending(&self) -> bool {
        self.nmi_latch.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn intr_pending(&self) -> bool {
        self.intr_asserted()
    }

    fn clear_nmi_latch(&self) {
        self.nmi_latch.store(false, Ordering::Release);
    }

    /// Latch a reset request. Cleared by whoever folds it into the state.
    fn request_reset(&self) {
        self.reset.store(true, Ordering::Release);
    }

    /// Consume the latch, reporting whether a reset was owed.
    fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    /// Open or close the A20 gate. `open` is the logical level of the pin.
    fn set_a20(&self, open: bool) {
        let mask = if open { u32::MAX } else { !(1u32 << 20) };
        self.a20_mask.store(mask, Ordering::Release);
    }

    /// The mask a physical address is `AND`ed with on its way to the bus.
    pub(crate) fn a20_mask(&self) -> u32 {
        self.a20_mask.load(Ordering::Relaxed)
    }

    /// Note that a gate has been wired, and shut it: a fresh net sits low.
    fn wire_a20(&self) {
        self.a20_wired.store(true, Ordering::Release);
        self.set_a20(false);
    }

    /// Add a controller to those that answer the acknowledge cycle on `INTR`.
    fn attach_ack(&self, ack: Weak<dyn IntAck>) {
        self.acks.attach(ack);
    }

    /// Drive the `INIT` pin, latching a rising edge.
    fn set_init_pin(&self, asserted: bool) {
        let previous = self.init_pin.swap(asserted, Ordering::AcqRel);
        if asserted && !previous {
            self.init_latch.store(true, Ordering::Release);
        }
    }

    /// The same, for the level this core's own controller reports.
    fn set_init_peer(&self, asserted: bool) {
        self.init_peer.store(asserted, Ordering::Release);
    }

    /// Latch an INIT the core has not run the sequence for yet.
    fn request_init(&self) {
        self.init_latch.store(true, Ordering::Release);
    }

    /// Consume the latch, reporting whether an INIT sequence is owed.
    ///
    /// Loaded before it is swapped, unlike the `NMI` latch beside it. Both are
    /// read once per instruction, but this one is read by *every* processor on
    /// a multiprocessor board and a read-modify-write takes the cache line
    /// exclusively every time — which is a line two cores would then trade back
    /// and forth for the whole of a run in which nothing happens.
    pub(crate) fn take_init_request(&self) -> bool {
        if !self.init_latch.load(Ordering::Relaxed) {
            return false;
        }
        self.init_latch.swap(false, Ordering::AcqRel)
    }

    /// Whether `INIT` is asserted from either driver, which holds the
    /// processor in reset rather than merely restarting it.
    pub(crate) fn init_held(&self) -> bool {
        self.init_pin.load(Ordering::Acquire) || self.init_peer.load(Ordering::Acquire)
    }

    /// Latch a Start-Up naming `page`.
    ///
    /// Last one wins, which is what the hardware does with a message it has
    /// already accepted and not yet acted on: there is one latch.
    fn request_startup(&self, page: u8) {
        self.startup.store(u32::from(page), Ordering::Release);
    }

    /// Consume the Start-Up latch, reporting the page it named.
    ///
    /// Loaded before it is swapped, for the reason
    /// [`take_init_request`](Lines::take_init_request) gives.
    pub(crate) fn take_startup(&self) -> Option<u8> {
        if self.startup.load(Ordering::Relaxed) == NO_STARTUP {
            return None;
        }
        match self.startup.swap(NO_STARTUP, Ordering::AcqRel) {
            NO_STARTUP => None,
            page => Some(page as u8),
        }
    }

    /// Whether a Start-Up is latched, without consuming it.
    fn startup_pending(&self) -> Option<u8> {
        match self.startup.load(Ordering::Acquire) {
            NO_STARTUP => None,
            page => Some(page as u8),
        }
    }

    /// Whether an INIT sequence is owed, without consuming the latch.
    fn init_latched(&self) -> bool {
        self.init_latch.load(Ordering::Acquire)
    }

    /// The `INIT` pin's level and the level this core's controller reports, as
    /// two separate facts — which is what a snapshot has to write, since
    /// restoring their disjunction could not tell them apart afterwards.
    fn init_levels(&self) -> (bool, bool) {
        (
            self.init_pin.load(Ordering::Acquire),
            self.init_peer.load(Ordering::Acquire),
        )
    }

    /// Put the INIT and Start-Up state back as a snapshot recorded it.
    fn restore_startup(&self, pin: bool, peer: bool, latch: bool, page: Option<u8>) {
        self.init_pin.store(pin, Ordering::Release);
        self.init_peer.store(peer, Ordering::Release);
        self.init_latch.store(latch, Ordering::Release);
        self.startup
            .store(page.map_or(NO_STARTUP, u32::from), Ordering::Release);
    }

    /// Whether a machine wired a local interrupt controller to this core.
    pub(crate) fn has_local_controller(&self) -> bool {
        self.has_intc.load(Ordering::Relaxed)
    }

    /// Adopt this core's own interrupt controller.
    fn attach_intc(&self, peer: Weak<dyn LocalController>) {
        *self.intc.lock() = Some(peer);
        self.has_intc.store(true, Ordering::Release);
    }

    /// This core's own interrupt controller, if a machine wired one.
    ///
    /// The lock is released before the caller uses what it holds, because
    /// everything the controller is asked runs its own critical section.
    fn intc(&self) -> Option<Arc<dyn LocalController>> {
        if !self.has_intc.load(Ordering::Relaxed) {
            return None;
        }
        let peer = self.intc.lock().clone();
        peer.and_then(|weak| weak.upgrade())
    }

    /// Ask this core's controller what it has, and fold it into the latches.
    ///
    /// Called once per instruction boundary with **no execution lock held**:
    /// the controller takes its own, and it is free to drive this core's `INTR`
    /// pin back while it does (§4.7's re-entrancy contract).
    fn poll_intc(&self) {
        let Some(intc) = self.intc() else { return };
        let signal = intc.take_startup();
        if signal.init {
            self.request_init();
        }
        self.set_init_peer(signal.held);
        if let Some(page) = signal.page {
            self.request_startup(page);
        }
    }

    /// `IA32_APIC_BASE`, as this core's own controller reports it.
    pub(crate) fn base_register(&self) -> Option<u64> {
        self.intc().map(|intc| intc.base_register())
    }

    /// Write `IA32_APIC_BASE` through to the controller that owns it.
    ///
    /// Reports whether there was a controller to take it, so a `WRMSR` to a
    /// register no part of this machine implements raises `#GP` rather than
    /// being swallowed.
    pub(crate) fn set_base_register(&self, value: u64) -> bool {
        match self.intc() {
            Some(intc) => {
                intc.set_base_register(value);
                true
            }
            None => false,
        }
    }

    /// Run the acknowledge cycle and report the vector.
    ///
    /// A controller on the net answers it — and moves the request from pending
    /// to in service while it is there, which is the half of the handshake a
    /// latched vector byte cannot do. With nothing attached the latched byte is
    /// the answer, which is what a test driving the pin by hand sets.
    ///
    /// The cycle presents nothing — an 8086 puts no level on the bus, it just
    /// strobes `INTA` twice — so it is an [`IntAckCycle::vector_only`], and a
    /// controller that answers at all answers with a vector.
    ///
    /// No lock is held across the outward call: the re-entrancy contract
    /// forbids holding one across a call into another device (§4.7), and a PIC
    /// answering an acknowledge takes its own.
    pub(crate) fn acknowledge(&self) -> u8 {
        match self.acks.run(IntAckCycle::vector_only()) {
            IntAckResponse::Vector(vector) => vector as u8,
            // No autovector on an x86: a cycle nobody terminates reads the
            // floating bus, which is what the latched byte models.
            IntAckResponse::Autovector | IntAckResponse::Declined => self.intr_vector(),
        }
    }

    fn snapshot(&self) -> (bool, bool, bool, u8) {
        (
            self.intr_asserted(),
            self.nmi_level.load(Ordering::Acquire),
            self.nmi_pending(),
            self.intr_vector(),
        )
    }

    fn restore(&self, (intr, level, latch, vector): (bool, bool, bool, u8)) {
        self.intr.store(intr, Ordering::Release);
        self.nmi_level.store(level, Ordering::Release);
        self.nmi_latch.store(latch, Ordering::Release);
        self.intr_vector.store(u32::from(vector), Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    memory: Option<Arc<AddressSpace>>,
    io: Option<Arc<AddressSpace>>,
}

/// An Intel 8086 or 8088 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank, rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own
/// `DEVICE`-ranked locks, which drive `WIRE`-ranked lines. The ladder runs in
/// the direction calls travel, so the debug order check passes for the real
/// call graph and fires on an inverted one.
///
/// The interrupt pins are *not* under that lock: they are atomics, so a device
/// asserting `INTR` from inside a write the CPU itself issued cannot re-enter
/// the CPU's own critical section.
#[derive(Debug)]
pub struct X86 {
    cfg: Config,
    /// Which of the two class names built this instance.
    ///
    /// The same core answers to `cpu.x86` and `cpu.i8086`, and a snapshot chunk
    /// is keyed by path but *carries* the class name (`ROADMAP.md` §4.5), so
    /// reporting the one the machine file actually named is the difference
    /// between a snapshot that describes the machine and one that describes a
    /// machine it could have been.
    class: &'static DeviceClass,
    lines: Arc<Lines>,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    ///
    /// Separate from [`Config::requester`] because a machine file names no
    /// requester: the machine layer allocates one per initiator and hands it
    /// over in [`Instance::bind`](crate::machine::Instance::bind), which is
    /// after `new` (§4.4).
    requester: AtomicU32,
    /// The name of the address space `IN` and `OUT` reach, from the `iospace`
    /// property. Resolved in `bind`, because spaces do not exist before then.
    iospace: String,
    session: sync::Mutex<Session>,
    /// The strong end of every pin this core has handed to a wire.
    ///
    /// A net holds its sinks weakly — the machine owns devices and a wire
    /// merely refers to them (§4.3) — so a pin nothing else kept alive would
    /// die on the way out of [`Device::sink`] and the wire would silently
    /// deliver to nothing.
    pins: sync::Mutex<Vec<(String, Arc<InputPin>)>>,
}

impl X86 {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](X86::attach_space) and [`Device::realize`].
    /// The first [`step`](X86::step) runs the reset sequence.
    #[must_use]
    pub fn new(cfg: Config) -> X86 {
        X86 {
            cfg,
            class: &CLASS,
            lines: Arc::new(Lines::default()),
            requester: AtomicU32::new(cfg.requester.0),
            iospace: String::new(),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(cfg.variant),
                    memory: None,
                    io: None,
                },
            ),
            pins: sync::Mutex::new(Vec::new()),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type, the variant is not one of the four
    /// this core knows, or a property nothing here accepts was given — a
    /// typo'd property that was silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<X86> {
        X86::from_props_defaulting(props, Variant::I8088)
    }

    /// The same, with a different default part.
    ///
    /// Two device classes share this constructor and disagree only about what
    /// "unspecified" means: `cpu.i8086` is an 8088 and `cpu.x86` is an 80486.
    ///
    /// # Errors
    ///
    /// As [`X86::from_props`].
    pub fn from_props_defaulting(props: &Props, default: Variant) -> Result<X86> {
        let mut r = props.reader();
        // `model` is accepted as a synonym for `variant` because the class was
        // called `cpu.i8086` and used that name before there was anything to
        // choose but the bus width.
        let named = r.optional_str("model")?;
        let variant = r.or_enum("variant", default.name(), Variant::NAMES)?;
        let variant = match named {
            Some(name) => Variant::from_name(name)
                .ok_or_else(|| Error::Property(alloc::format!("unknown x86 model `{name}`")))?,
            None => Variant::from_name(variant).expect("the enum listed above"),
        };
        // Accepted and ignored: there is one engine until phase 5, and a
        // machine file that names it should not need editing when the second
        // one lands.
        let _engine = r.or_enum("engine", "interp", &["interp"])?;
        // `space =` is structural and there is exactly one of it, so the
        // *second* address space is named by an ordinary string property and
        // looked up with `BindCtx::space_named`.
        let iospace = r.optional_str("iospace")?.unwrap_or("").to_string();
        // The lattice's per-instance half (`ROADMAP.md` §6.1.1): the variant
        // name expands to a preset, and a machine description may switch an
        // extension off to model a part that lacked it. Switching one *on*
        // that the preset does not have is refused by `validate` where it
        // would make an impossible part.
        let mut features = variant.features();
        for name in [
            "cpuid", "cr4", "msr", "pae", "long", "nx", "syscall", "cmov", "pse", "pge", "fpu",
            "cx8", "fxsr", "sse", "sse2",
        ] {
            let Some(on) = r.optional::<bool>(name)? else {
                continue;
            };
            match name {
                "cpuid" => features.extras_486 = on,
                "cr4" => features.cr4 = on,
                "msr" => features.msr = on,
                "pae" => features.pae = on,
                "long" => features.long = on,
                "nx" => features.nx = on,
                "syscall" => features.syscall = on,
                "cmov" => features.cmov = on,
                "pse" => features.pse = on,
                "pge" => features.pge = on,
                "fpu" => features.fpu = on,
                "cx8" => features.cx8 = on,
                "fxsr" => features.fxsr = on,
                "sse" => features.sse = on,
                _ => features.sse2 = on,
            }
        }
        features.validate()?;
        r.finish()?;
        let mut cpu = X86::new(Config {
            variant,
            features,
            requester: RequesterId::ANONYMOUS,
        });
        cpu.iospace = iospace;
        Ok(cpu)
    }

    /// The same core under the other class name.
    ///
    /// [`I8086_CLASS`]'s constructor calls this, so an instance a machine file
    /// declared as `cpu.i8086` reports that class rather than `cpu.x86` — which
    /// the realizer checks, and which a snapshot chunk carries. Public because
    /// anything assembling its own [`Bindings`](crate::machine::Bindings) has
    /// to be able to build the same thing the catalog does.
    #[must_use]
    pub fn as_i8086(mut self) -> X86 {
        self.class = &I8086_CLASS;
        self
    }

    /// Which address space `IN` and `OUT` reach, by name, or `""` for none.
    #[must_use]
    pub fn io_space_name(&self) -> &str {
        &self.iospace
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

    /// Whether the A20 gate is open — that is, whether address bit 20 reaches
    /// memory.
    ///
    /// Open unless something drives the `a20` pin low. A board with no gate
    /// wired has bit 20 permanently connected; a board that wires one takes
    /// the level from its net, which the realize sweep announces before
    /// anything executes. On a PC/AT that level is **high** — the 8042's
    /// output port comes up all-ones, and a gate shut at power-on would mask
    /// bit 20 out of the reset vector itself.
    ///
    /// The gate is not a processor feature on real silicon — it sits in the
    /// chipset, between the CPU and the bus — but rsemu has no device between
    /// an initiator and its address space, and the gate is exactly a
    /// suppression of the address wrap this core does for itself.
    #[must_use]
    pub fn a20_open(&self) -> bool {
        self.lines.a20_mask() == u32::MAX
    }

    /// Drive the A20 gate directly, for a caller with no wire.
    pub fn set_a20(&self, open: bool) {
        self.lines.set_a20(open);
    }

    /// Give the core the memory space it executes from.
    ///
    /// Separate from construction because the space is built by the machine
    /// assembly layer; when `RealizeCtx` grows space accessors this moves into
    /// [`Device::realize`] and the method stays as the way a test wires one up.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().memory = Some(space);
    }

    /// Give the core the **separate** I/O address space `IN` and `OUT` reach.
    ///
    /// Not a window into memory: the 8086 drives a distinct status line for an
    /// I/O cycle, and a PC's chipset decodes it differently. A core with no
    /// I/O space reads ones and discards writes, which is what an unpopulated
    /// bus does.
    pub fn attach_io_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().io = Some(space);
    }

    /// The memory space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().memory.clone()
    }

    /// The I/O space, if one is attached.
    #[must_use]
    pub fn io_space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().io.clone()
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().state.regs
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    ///
    /// The prefetch queue is flushed, because the bytes in it belonged to the
    /// old `CS:EIP`. In real mode the segment registers' cached bases are
    /// recomputed to match the selectors written, so a caller that sets `CS`
    /// gets a core that fetches from where it asked; in protected mode they
    /// are **not**, because a selector alone does not say what descriptor was
    /// cached — use [`X86::set_sys`] for that.
    pub fn set_regs(&self, regs: Regs) {
        let mut session = self.session.lock();
        session.state.regs = regs;
        session.state.regs.eflags = Regs::normalise_flags(self.cfg.variant, regs.eflags);
        if !session.state.sys.protected() {
            for index in 0..isa::seg::COUNT as u8 {
                let selector = session.state.regs.segment(index);
                let entry = session.state.sys.seg_mut(index);
                entry.selector = selector;
                entry.base = u64::from(selector) << 4;
            }
        }
        session.state.queue.flush();
    }

    /// The system registers: the segment descriptor caches, the descriptor
    /// table registers, and `CR0`-`CR3`.
    #[must_use]
    pub fn sys(&self) -> prot::Sys {
        self.session.lock().state.sys
    }

    /// Overwrite the system registers.
    ///
    /// The translation-lookaside buffer is flushed with them: it is derived
    /// from `CR3` and the page tables, and keeping stale entries across a
    /// wholesale replacement of the system state is exactly the bug the buffer
    /// exists to hide.
    pub fn set_sys(&self, sys: prot::Sys) {
        let mut session = self.session.lock();
        session.state.sys = sys;
        session.state.tlb.flush();
        session.state.queue.flush();
    }

    /// The x87 unit: the register stack, the three words, and the pointers.
    #[must_use]
    pub fn x87(&self) -> fpu::X87 {
        self.session.lock().state.x87
    }

    /// Overwrite the x87 unit.
    ///
    /// The tag word comes across as given rather than being recomputed from
    /// the registers: `FRSTOR` can legitimately leave the two disagreeing, and
    /// a setter that quietly agreed with itself would be unable to reproduce
    /// that.
    pub fn set_x87(&self, x87: fpu::X87) {
        self.session.lock().state.x87 = x87;
    }

    /// The SSE registers and `MXCSR`.
    #[must_use]
    pub fn sse(&self) -> fpu::Sse {
        self.session.lock().state.sse
    }

    /// Overwrite the SSE registers and `MXCSR`.
    pub fn set_sse(&self, sse: fpu::Sse) {
        self.session.lock().state.sse = sse;
    }

    /// Read one register by name. A 16-bit view returns its low half.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u64 {
        reg.get(&self.session.lock().state.regs)
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u64) {
        let mut session = self.session.lock();
        reg.set(&mut session.state.regs, value);
        if matches!(reg, Reg::Eflags | Reg::Flags) {
            let value = session.state.regs.eflags;
            session.state.regs.eflags = Regs::normalise_flags(self.cfg.variant, value);
        }
        if matches!(reg, Reg::Cs | Reg::Eip | Reg::Ip) {
            if reg == Reg::Cs && !session.state.sys.protected() {
                let selector = session.state.regs.cs;
                let entry = session.state.sys.seg_mut(isa::seg::CS);
                entry.selector = selector;
                entry.base = u64::from(selector) << 4;
            }
            session.state.queue.flush();
        }
    }

    /// Clock cycles executed since power-on.
    ///
    /// Four per bus cycle plus the manual's internal execution figures. This
    /// is documented timing rather than measured timing: the bus interface
    /// unit's prefetching is not overlapped with execution here, so a count
    /// taken over a long run is an upper bound rather than the number a
    /// logic analyser would show. The module documentation says why that
    /// trade was made.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a `HLT` has stopped the core.
    ///
    /// A halted 8086 restarts on any interrupt, so this is not the terminal
    /// state a `JAM` is on a 6502 — but a scheduler still has to notice it
    /// rather than spin, because [`step`](X86::step) charges nothing while
    /// it holds.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// The prefetch queue's current contents, oldest byte first.
    ///
    /// Exposed because it is genuinely observable on an 8088 — the queue
    /// status lines are pins — and because the conformance corpus specifies
    /// it as part of a vector's initial state.
    #[must_use]
    pub fn prefetch_queue(&self) -> Vec<u8> {
        self.session.lock().state.queue.contents()
    }

    /// Install a prefetch queue, as if the bus interface unit had already
    /// fetched these bytes from `CS:IP` onwards.
    ///
    /// # Errors
    ///
    /// If more bytes are supplied than the part's queue holds.
    pub fn set_prefetch_queue(&self, bytes: &[u8]) -> Result<()> {
        let mut session = self.session.lock();
        session.state.queue.install(bytes).map_err(|()| {
            Error::Property(alloc::format!(
                "the {} prefetch queue holds {} bytes, not {}",
                self.cfg.variant,
                self.cfg.variant.queue_bytes(),
                bytes.len()
            ))
        })
    }

    /// How many accesses an address space refused, and where the last one was.
    ///
    /// The 8086 has no bus-error input, so a refused access cannot raise an
    /// exception: a read returns whatever was last on the data bus, which is
    /// what an unterminated bus does. This counter is how that becomes visible
    /// instead of silent — a machine whose memory map has a hole will show it
    /// climbing.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u64) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// Drive the `INTR` pin. Level-sensitive: it is taken while asserted and
    /// `IF` is set.
    ///
    /// `asserted` is the logical level, not the pin's polarity; inverting a
    /// real signal belongs to whatever models the wire.
    pub fn set_intr(&self, asserted: bool) {
        self.lines.set_intr(asserted);
    }

    /// Whether `INTR` is currently asserted.
    #[must_use]
    pub fn intr_asserted(&self) -> bool {
        self.lines.intr_asserted()
    }

    /// Set the vector the next `INTR` acknowledge will read.
    ///
    /// A PC's 8259A drives this onto the data bus during the second `INTA`
    /// cycle. Setting it before asserting `INTR` is the whole handshake as far
    /// as the CPU is concerned.
    pub fn set_intr_vector(&self, vector: u8) {
        self.lines.set_intr_vector(vector);
    }

    /// The vector the next acknowledge would read.
    #[must_use]
    pub fn intr_vector(&self) -> u8 {
        self.lines.intr_vector()
    }

    /// Drive the NMI pin. Edge-sensitive: a rising edge latches, and the latch
    /// survives until the interrupt is taken.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.set_nmi(asserted);
    }

    /// A complete NMI pulse, for a caller that does not model the pin's level.
    pub fn pulse_nmi(&self) {
        self.lines.set_nmi(true);
        self.lines.set_nmi(false);
    }

    /// Whether an NMI edge is latched and not yet serviced.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.lines.nmi_pending()
    }

    /// Whether the next instruction runs with interrupts inhibited.
    ///
    /// Set for exactly one instruction after `MOV SS,x` and `POP SS`, so that
    /// the `SS:SP` pair can be reloaded without an interrupt landing on the
    /// half-changed stack. On an 8086 the shadow covers NMI as well as
    /// `INTR`, which later parts changed.
    #[must_use]
    pub fn interrupt_shadow(&self) -> bool {
        self.session.lock().state.int_shadow
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](X86::step): a reset is a
    /// signal, not a method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Whether the `reset` pin has latched a request the core has not run yet.
    ///
    /// Distinct from [`reset_pending`](X86::reset_pending), which is the
    /// *execution state*: the pin's latch lives outside the execution lock and
    /// is folded into that state by the next [`step`](X86::step). A board test
    /// watching a `RESET` pulse arrive is watching this one.
    #[must_use]
    pub fn reset_requested(&self) -> bool {
        self.lines.reset.load(Ordering::Acquire)
    }

    /// Request an INIT, without changing any register.
    ///
    /// The lesser of the two restarts (SDM Vol 3A Table 9-1): the sequence runs
    /// on the next [`step`](X86::step), and what it leaves behind is a
    /// processor in the **wait-for-SIPI** state rather than one fetching from
    /// the reset vector. Nothing but a Start-Up leaves that state — see
    /// [`start_up`](X86::start_up) — which is exactly the difference between an
    /// INIT and a `RESET` and the reason a second processor needs both.
    pub fn request_init(&self) {
        self.lines.request_init();
    }

    /// Deliver a Start-Up naming `page`.
    ///
    /// The processor leaves wait-for-SIPI and begins executing at
    /// `CS:IP = page << 8 : 0` — a real-mode segment whose base is
    /// `page << 12`, so physical `000PP000H` (*MultiProcessor Specification*
    /// v1.4 §B.4, Intel SDM Vol 3A §8.4.3).
    ///
    /// Latched rather than applied: a Start-Up is a message that arrives from
    /// inside whatever device sent it — on a real board, from inside a store
    /// the *other* processor issued — so it lands in an atomic outside the
    /// execution lock and the next step folds it in (§4.7).
    ///
    /// A Start-Up to a processor that is not waiting for one is ignored, which
    /// is why the specification's algorithm sends two and does not care that
    /// the second is redundant.
    pub fn start_up(&self, page: u8) {
        self.lines.request_startup(page);
    }

    /// Whether the core is halted awaiting a Start-Up.
    ///
    /// Distinct from [`is_halted`](X86::is_halted): a `HLT` is left by any
    /// interrupt, and this is left by nothing but a Start-Up. An `INTR` or an
    /// `NMI` arriving here stays pending rather than being taken.
    #[must_use]
    pub fn is_waiting_for_startup(&self) -> bool {
        self.session.lock().state.wait_for_sipi
    }

    /// Whether an INIT sequence has been latched and not yet run.
    #[must_use]
    pub fn init_requested(&self) -> bool {
        self.lines.init_latched()
    }

    /// Whether the `INIT` line is asserted, from a pin or from this core's own
    /// interrupt controller.
    ///
    /// While it is, the processor is held in reset and charges no cycles.
    #[must_use]
    pub fn init_held(&self) -> bool {
        self.lines.init_held()
    }

    /// Run the `INTR` acknowledge cycle now and report the vector.
    ///
    /// This is exactly what the core does when it takes a maskable interrupt,
    /// exposed because it is the half of the handshake nothing else can
    /// observe: whatever drives `INTR` answers with a vector *and* moves the
    /// request from pending to in service. It has side effects on that
    /// controller, so it is for a monitor or a test standing in for the
    /// processor, not for polling.
    pub fn acknowledge(&self) -> u8 {
        self.lines.acknowledge()
    }

    /// Execute one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the clock cycles charged: zero if the core is halted with no
    /// interrupt pending, or has no address space, which the caller must treat
    /// as "stop", not "retry".
    pub fn step(&self) -> u64 {
        // Ask this core's own interrupt controller what it has, **before** the
        // execution lock is taken: it takes its own, and it may drive this
        // core's `INTR` pin back while it is answering (§4.7). One relaxed load
        // when no such controller is wired, which is every machine in the tree
        // but a multiprocessor one.
        self.lines.poll_intc();
        let reset = self.lines.take_reset_request();
        let cfg = self.config();
        let mut session = self.session.lock();
        let Session { state, memory, io } = &mut *session;
        // The `reset` pin latches outside the lock; this is where the latch
        // becomes execution state, and it happens before the step so a pulse is
        // honoured at the very next instruction boundary.
        state.reset_pending |= reset;
        let Some(memory) = memory.clone() else {
            return 0;
        };
        let io = io.clone();
        Exec::new(state, &memory, io.as_deref(), &cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — an 8086 cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    /// Stops early if the core halts.
    ///
    /// [`run_budget`](X86::run_budget) is the same loop with the overshoot
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
    /// A halted core, one that has shut down on a triple fault, or one with no
    /// address space consumes only the debt it owed plus whatever it managed.
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
                // Halted with nothing pending, shut down, or no address space.
                // Either way retrying would spin — but the budget is still
                // consumed, or the scheduler never advances past a `HLT`
                // waiting for the timer that would wake it.
                self.session.lock().state.debt = 0;
                return ticks;
            }
            used += n;
        }
        self.session.lock().state.debt = used - allowance;
        ticks
    }

    /// Clocks owed to the next budget — see [`run_budget`](X86::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Where a **linear** address lives, as a debugger asks it.
    ///
    /// Linear, not virtual-with-a-segment: on this family the debugger's
    /// address has already had segmentation applied — gdb's i386 target works
    /// in the flat space its descriptors describe — and paging is the only
    /// translation left to do. A caller holding a `CS:EIP` pair adds the
    /// segment's cached base first, which is what
    /// [`disassemble`](X86::disassemble) does.
    ///
    /// Side-effect free by construction, not by care: the walk sets no
    /// accessed or dirty bit, does not consult or fill the TLB, does not latch
    /// `CR2`, charges no cycles, and reads both descriptors with
    /// [`MemAttrs::DEBUG`]. It is permission-free as well — it answers where
    /// the page is, not whether an access would be allowed — so a debugger can
    /// be shown a user page while the core is in ring 0.
    ///
    /// [`DebugTranslation::Identity`] when paging is off, which is a different
    /// fact from [`DebugTranslation::Unmapped`]: the first is a processor with
    /// nothing to translate, the second is a listing that has run off the end
    /// of a mapped page.
    #[must_use]
    pub fn translate_debug(&self, linear: u64) -> DebugTranslation {
        let Some(space) = self.space() else {
            return DebugTranslation::Identity;
        };
        let sys = self.session.lock().state.sys;
        paging::debug_translate(&sys, self.cfg.features, &space, linear)
    }

    /// Disassemble `count` instructions starting at the current `CS:EIP`,
    /// reading guest memory with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around `EIP`
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    ///
    /// Reads go through the code segment's **cached base**, so a listing in
    /// protected mode shows the instructions the processor would fetch rather
    /// than the ones at `selector << 4`, and then through
    /// [`translate_debug`](X86::translate_debug), so a listing of a *paged*
    /// guest shows its code rather than whatever physical memory happens to
    /// sit at the same number. A byte the tables do not map reads as `None`
    /// and the decoder reports the instruction truncated, which is the honest
    /// answer for a listing that has run off the end of a page.
    #[must_use]
    pub fn disassemble(&self, cs: u16, eip: u64, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let (base, bits, legacy, sys) = {
            let session = self.session.lock();
            let seg = session.state.sys.seg(isa::seg::CS);
            let legacy = !self.cfg.variant.is_32bit();
            let base = if seg.selector == cs {
                seg.base
            } else {
                u64::from(cs) << 4
            };
            // The listing is decoded at the width the processor would
            // fetch at, which in long mode is the code segment's `L` bit
            // rather than its `D` bit.
            let bits = if legacy {
                isa::Bits::B16
            } else if self.cfg.features.long && session.state.sys.long_mode() && seg.long() {
                isa::Bits::B64
            } else if seg.big() {
                isa::Bits::B32
            } else {
                isa::Bits::B16
            };
            (base, bits, legacy, session.state.sys)
        };
        let map = self.cfg.variant.map();
        let features = self.cfg.features;
        disasm::disassemble_run_as(map, bits, cs, eip, count, |addr| {
            // Segmentation first, then paging: that is the order the address
            // unit works in, and doing only the first is what used to make a
            // listing of a paged guest show whatever physical memory sat at
            // the same number.
            let linear = if legacy {
                base.wrapping_add(addr) & 0xf_ffff
            } else {
                base.wrapping_add(addr)
            };
            let addr = paging::debug_translate(&sys, features, &space, linear).phys(linear)?;
            space
                .read(addr, Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
    }
}

/// The `cpu.x86` device class.
///
/// Defaults to an 80486 because that is the part a firmware image expects: a
/// machine that asks for "an x86" and gets an 8088 fails on the first `CPUID`.
/// The 8086 and 8088 are reachable by name, and [`I8086_CLASS`] keeps the
/// older class name working with its own default.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.x86",
    // 4: the register file widened to sixty-four bits and the chunk gained the
    //    long-mode block — `CR4`, `EFER`, the segment-base MSRs and `R8`-`R15`.
    // 5: the floating-point block — the eight x87 registers with the control,
    //    status and tag words and the environment pointers, then the sixteen
    //    `XMM` registers and `MXCSR`.
    // 6: the multiprocessor block — the wait-for-SIPI state, the two `INIT`
    //    levels, the INIT latch and the Start-Up page.
    version: 6,
    summary: "Intel x86 CPU core: 8086/8088 real mode, 80386/80486 protected mode, or x86-64",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "\"8086\", \"8088\", \"80386\", \"80486\" (the default) or \"x86-64\"",
        },
        PropertySpec {
            name: "cpuid",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: CPUID, BSWAP, XADD, CMPXCHG, INVLPG and CR0.WP",
        },
        PropertySpec {
            name: "cr4",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: CR4 exists, which every extension below needs a bit in",
        },
        PropertySpec {
            name: "msr",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: RDMSR, WRMSR, RDTSC and the model-specific registers",
        },
        PropertySpec {
            name: "pae",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: physical address extension and the three-level walk",
        },
        PropertySpec {
            name: "long",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: long mode, REX, and four-level paging",
        },
        PropertySpec {
            name: "nx",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: EFER.NXE and the no-execute page bit",
        },
        PropertySpec {
            name: "syscall",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: SYSCALL and SYSRET",
        },
        PropertySpec {
            name: "cmov",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: the integer conditional moves",
        },
        PropertySpec {
            name: "pse",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: CR4.PSE and 4 MiB pages in a two-level walk",
        },
        PropertySpec {
            name: "pge",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: CR4.PGE and the global-page bit",
        },
        PropertySpec {
            name: "fpu",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: the on-die x87 unit (a 486SX is a 486 with this off)",
        },
        PropertySpec {
            name: "cx8",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: CMPXCHG8B, and CMPXCHG16B in long mode",
        },
        PropertySpec {
            name: "fxsr",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: FXSAVE/FXRSTOR and CR4.OSFXSR",
        },
        PropertySpec {
            name: "sse",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: the XMM registers, MXCSR and the single-precision SIMD",
        },
        PropertySpec {
            name: "sse2",
            kind: ValueKind::Bool,
            required: false,
            summary: "override the preset: the double-precision and packed-integer SIMD",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "accepted as a synonym for \"variant\", which this class used to be called",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
        PropertySpec {
            name: "iospace",
            kind: ValueKind::Str,
            required: false,
            summary: "the name of the separate address space IN and OUT reach",
        },
    ],
    construct: |props| {
        Ok(Box::new(X86::from_props_defaulting(
            props,
            Variant::I80486,
        )?))
    },
};

/// The `cpu.i8086` device class: the same core, defaulting to an 8088.
///
/// Kept as its own name so a machine description written against the 16-bit
/// core keeps meaning what it meant. A build links one core either way.
pub static I8086_CLASS: DeviceClass = DeviceClass {
    name: "cpu.i8086",
    version: 6,
    summary: "Intel 8086 / 8088 16-bit CPU core, real mode, hardware-checked interpreter",
    properties: CLASS.properties,
    construct: |props| {
        Ok(Box::new(
            X86::from_props_defaulting(props, Variant::I8088)?.as_i8086(),
        ))
    },
};

/// Add this core's classes to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-x86")]` arm.
///
/// # Errors
///
/// If something already claimed either name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)?;
    reg.add(&I8086_CLASS)
}

impl Device for X86 {
    fn class(&self) -> &'static DeviceClass {
        self.class
    }

    /// The debug surface's route to the page unit: how a gdb `m` packet
    /// naming an address in a paged guest reaches the right physical one.
    ///
    /// The address is taken as **linear** — segmentation has already been
    /// applied by whoever asked — so this is exactly
    /// [`translate_debug`](X86::translate_debug). It is no longer narrowed to
    /// 32 bits: a long-mode guest's linear address really is 48 significant
    /// bits sign-extended to 64, and truncating one would report `Unmapped`
    /// for every kernel address, which are all above the boundary.
    fn debug_translate(&self, va: u64) -> DebugTranslation {
        self.translate_debug(va)
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made. It is
        // not decoration on this board — `A20` and `RESET` each have two
        // drivers on an AT, the keyboard controller and the chipset's fast
        // port, and a pin that only remembered "somebody said high" would drop
        // the line when either of them said low.
        let which = match port {
            "intr" => Input::Intr,
            "nmi" => Input::Nmi,
            "reset" => Input::Reset,
            // No `INIT` pin on a 16-bit part: it arrived with the parts that
            // could be a second processor. A machine file naming one on an 8088
            // is a wiring error and is told so, rather than being given a pin
            // that does nothing.
            "init" if self.cfg.variant.is_32bit() => Input::Init,
            "a20" => Input::A20,
            _ => return None,
        };
        if which == Input::A20 {
            self.lines.wire_a20();
        }
        // No `wire_init` beside that: `INIT` needs no such note. A fresh net
        // sits low and `INIT` is asserted high, so a pin nobody has driven yet
        // reads as de-asserted — which is what a processor nobody is holding in
        // reset wants, and it is the *net's* answer rather than this core's
        // guess. The A20 gate needed the opposite treatment for the opposite
        // reason, and `Device::reset` says why.
        let pin = Arc::new(InputPin::new(Arc::clone(&self.lines), which, sources));
        self.pins.lock().push((port.to_string(), Arc::clone(&pin)));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        // Only `INTR` has an acknowledge cycle: `NMI` is vectored through entry
        // 2 by the architecture and nothing drives a vector for it.
        if port == "intr" {
            self.lines.attach_ack(ack);
        }
    }

    /// This core's *own* interrupt controller — a local APIC — offered on the
    /// pin it drives.
    ///
    /// `intr` and `nmi` both, because a local APIC drives both of them and the
    /// machine offers the peer on whichever net it was wired along; a board
    /// that wires only `nmi` should still be able to start this processor.
    /// Attaching the same controller twice is harmless: there is one slot.
    fn attach_local_controller(&self, port: &str, peer: Weak<dyn LocalController>) {
        if matches!(port, "intr" | "nmi") {
            self.lines.attach_intc(peer);
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
            session.state = State::new(self.cfg.variant);
        } else {
            // A warm reset is a pulse on the RESET pin, and on an 8086 that is
            // not subtle: the sequence loads CS:IP itself, so what survives is
            // only the general registers.
            session.state.reset_pending = true;
            session.state.halted = false;
            session.state.int_shadow = false;
            session.state.queue.flush();
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((false, false, false, 0));
            // **The A20 mask is deliberately not touched here.** The gate is
            // not in the processor — it is a chipset AND gate on the address
            // bus, modelled here only because rsemu has nothing between an
            // initiator and its space — so a `RESET` on the processor does not
            // move it. Driving it shut from here was inventing a level for an
            // input pin, and the machine could not correct it: a driver
            // re-announcing the level it was already at is not a change, and
            // `Wire::set` delivers changes. The result was an AT whose 8042
            // held the gate open while the core masked bit 20 anyway, so the
            // reset vector at `0xfffffff0` was fetched from `0xffeffff0`,
            // which decodes to nothing. Whatever the net says, the pin says.
            //
            // A board with no gate wired keeps `u32::MAX` from construction,
            // which is bit 20 permanently connected — the right answer for a
            // machine that has no such chip.
        } else {
            // The input *levels* belong to whatever drives them, not to the
            // CPU — clearing them here would make a reset lie about the
            // machine. The edge latch is internal, so it goes.
            self.lines.clear_nmi_latch();
        }
        // The sequence the machine just asked for is the one the pin owed.
        self.lines.take_reset_request();
        // And it outranks an INIT, which is a *lesser* restart: a processor
        // that has just been reset is not owed the sequence that would have put
        // it back where reset already put it. The two latches are internal, so
        // they go; the `INIT` **level** does not, for exactly the reason the
        // A20 comment above gives — whatever drives that net still drives it,
        // and a core that cleared it here would start executing while the board
        // was still holding it in reset.
        self.lines.take_init_request();
        self.lines.take_startup();
    }

    /// The snapshot layout.
    ///
    /// The **first sixty-four bytes are gdb's i386 core register block**, in
    /// its order and at its widths: eight general registers, `EIP`, `EFLAGS`,
    /// then `CS`, `SS`, `DS`, `ES`, `FS`, `GS` as doublewords. That is not a
    /// coincidence — `host::gdb::arch` indexes straight into this prefix, and
    /// making the layouts agree removes a translation step that could drift.
    ///
    /// After the prefix come the hidden descriptor caches, which are
    /// architectural state and not derivable from the selectors
    /// ([`prot`]'s module documentation says why), the descriptor table
    /// registers, the control and debug registers, and the interrupt pins.
    /// The translation-lookaside buffer is **not** written: it is a cache of
    /// the page tables and is rebuilt on demand.
    ///
    /// The **long-mode block** comes last, and it comes last on purpose: the
    /// sixteen 64-bit general registers, `RIP`, `CR4`, `EFER` and the four
    /// segment-base and `SYSCALL` model-specific registers. Appending rather
    /// than widening the gdb prefix in place keeps `host::gdb::arch`'s
    /// indexing into the first sixty-four bytes valid, and the whole of the
    /// register file is still written exactly once — the prefix holds each
    /// register's low half and this block replaces it with the full width.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Fold the `reset` pin's latch in first. It is not a field of its own
        // in the chunk: `reset_pending` is where it was always going, and a
        // snapshot taken between an assertion and the next step would otherwise
        // lose the reset entirely.
        let reset = self.lines.take_reset_request();
        let state = {
            let mut session = self.session.lock();
            session.state.reset_pending |= reset;
            session.state
        };
        for reg in Reg::ALL {
            w.write_u32(reg.get(&state.regs) as u32)?;
        }
        w.write_u64(state.cycles)?;
        for index in 0..isa::seg::COUNT as u8 {
            let s = state.sys.seg(index);
            w.write_u16(s.selector)?;
            w.write_u64(s.base)?;
            w.write_u32(s.limit)?;
            w.write_u32(s.ar)?;
        }
        for s in [state.sys.ldtr, state.sys.task] {
            w.write_u16(s.selector)?;
            w.write_u64(s.base)?;
            w.write_u32(s.limit)?;
            w.write_u32(s.ar)?;
        }
        for t in [state.sys.gdtr, state.sys.idtr] {
            w.write_u64(t.base)?;
            w.write_u32(t.limit)?;
        }
        w.write_u32(state.sys.cr0)?;
        w.write_u64(state.sys.cr2)?;
        w.write_u64(state.sys.cr3)?;
        for value in state.sys.dr {
            w.write_u64(value)?;
        }
        for value in state.sys.test {
            w.write_u32(value)?;
        }
        w.write_bool(state.halted)?;
        w.write_bool(state.shutdown)?;
        w.write_bool(state.reset_pending)?;
        w.write_bool(state.int_shadow)?;
        w.write_u8(state.open_bus)?;
        w.write_u64(state.faults)?;
        w.write_u64(state.last_fault)?;
        let queue = state.queue.contents();
        w.write_u8(queue.len() as u8)?;
        for byte in queue {
            w.write_u8(byte)?;
        }
        w.write_u64(state.debt)?;
        let (intr, nmi_level, nmi_latch, vector) = self.lines.snapshot();
        w.write_bool(intr)?;
        w.write_bool(nmi_level)?;
        w.write_bool(nmi_latch)?;
        w.write_u8(vector)?;
        // The A20 gate is an input level, and a restored machine whose gate was
        // closed must still have it closed — the chipset that drives it is not
        // going to announce again until the next realize sweep.
        w.write_bool(self.a20_open())?;
        // The long-mode block. Written unconditionally rather than behind a
        // feature check: a chunk whose shape depended on the instance's
        // extensions could not be loaded into an instance configured
        // differently, and diagnosing *that* is worse than sixteen wasted
        // doublewords on an 8088.
        for reg in Reg::WIDE {
            w.write_u64(reg.get(&state.regs))?;
        }
        w.write_u64(state.sys.cr4)?;
        w.write_u64(state.sys.efer)?;
        w.write_u64(state.sys.fs_base)?;
        w.write_u64(state.sys.gs_base)?;
        w.write_u64(state.sys.kernel_gs_base)?;
        w.write_u64(state.sys.star)?;
        w.write_u64(state.sys.lstar)?;
        w.write_u64(state.sys.cstar)?;
        w.write_u64(state.sys.sfmask)?;
        // The floating-point block, appended for the same reason the
        // long-mode block was: everything before it keeps the offset it had,
        // so `host::gdb::arch`'s indexing into the first sixty-four bytes and
        // every chunk a previous version wrote stay where they were.
        //
        // Written unconditionally, on an 8088 as much as on an x86-64. A chunk
        // whose *shape* depended on the instance's features could not be
        // loaded into an instance configured differently, and a snapshot that
        // silently refuses to load is worse than a few hundred wasted bytes.
        for reg in state.x87.regs {
            w.write_u64(reg.sig)?;
            w.write_u16(reg.sign_exp)?;
        }
        w.write_u16(state.x87.control)?;
        w.write_u16(state.x87.status)?;
        w.write_u16(state.x87.tag)?;
        w.write_u16(state.x87.last_op)?;
        w.write_u64(state.x87.last_ip)?;
        w.write_u64(state.x87.last_dp)?;
        w.write_u16(state.x87.last_cs)?;
        w.write_u16(state.x87.last_ds)?;
        for reg in state.sse.xmm {
            w.write_u64(reg[0])?;
            w.write_u64(reg[1])?;
        }
        w.write_u32(state.sse.mxcsr)?;
        // The multiprocessor block, appended for the reason the two before it
        // were: everything ahead of it keeps the offset it had.
        //
        // The two `INIT` **levels** go in for the same reason the A20 gate's
        // does — they belong to whatever drives them, and nothing is going to
        // announce again until the next realize sweep, so a restored processor
        // that was being held in reset must still be held. The latch and the
        // Start-Up page are internal, and a snapshot taken between a Start-Up
        // arriving and the step that acts on it would otherwise lose the
        // processor entirely.
        w.write_bool(state.wait_for_sipi)?;
        let (init_pin, init_peer) = self.lines.init_levels();
        w.write_bool(init_pin)?;
        w.write_bool(init_peer)?;
        w.write_bool(self.lines.init_latched())?;
        let page = self.lines.startup_pending();
        w.write_bool(page.is_some())?;
        w.write_u8(page.unwrap_or(0))?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.cfg.variant);
        for reg in Reg::ALL {
            let value = r.read_u32()?;
            reg.set(&mut state.regs, u64::from(value));
        }
        state.cycles = r.read_u64()?;
        for index in 0..isa::seg::COUNT as u8 {
            let s = state.sys.seg_mut(index);
            s.selector = r.read_u16()?;
            s.base = r.read_u64()?;
            s.limit = r.read_u32()?;
            s.ar = r.read_u32()?;
        }
        for slot in [0usize, 1] {
            let s = prot::SegReg {
                selector: r.read_u16()?,
                base: r.read_u64()?,
                limit: r.read_u32()?,
                ar: r.read_u32()?,
            };
            if slot == 0 {
                state.sys.ldtr = s;
            } else {
                state.sys.task = s;
            }
        }
        for slot in [0usize, 1] {
            let t = prot::TableReg {
                base: r.read_u64()?,
                limit: r.read_u32()?,
            };
            if slot == 0 {
                state.sys.gdtr = t;
            } else {
                state.sys.idtr = t;
            }
        }
        state.sys.cr0 = r.read_u32()?;
        state.sys.cr2 = r.read_u64()?;
        state.sys.cr3 = r.read_u64()?;
        for i in 0..8 {
            state.sys.dr[i] = r.read_u64()?;
        }
        for i in 0..8 {
            state.sys.test[i] = r.read_u32()?;
        }
        state.halted = r.read_bool()?;
        state.shutdown = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.int_shadow = r.read_bool()?;
        state.open_bus = r.read_u8()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u64()?;
        let len = r.read_u8()?;
        let mut queue = Vec::with_capacity(usize::from(len));
        for _ in 0..len {
            queue.push(r.read_u8()?);
        }
        state.queue.install(&queue).map_err(|()| {
            Error::State(alloc::format!(
                "snapshot has a {len}-byte prefetch queue; an {} holds {}",
                self.cfg.variant,
                self.cfg.variant.queue_bytes()
            ))
        })?;
        state.debt = r.read_u64()?;
        let intr = r.read_bool()?;
        let nmi_level = r.read_bool()?;
        let nmi_latch = r.read_bool()?;
        let vector = r.read_u8()?;
        let a20 = r.read_bool()?;
        // The long-mode block, in the order `save` wrote it. Every register
        // here was already written once, as a `u32`, in the prefix gdb's i386
        // layout reads — so this block contributes the **upper** half only and
        // takes its low half from what the prefix restored.
        //
        // Letting the full 64-bit value win instead would be correct for a
        // plain save/load round trip, where the two copies agree by
        // construction, and silently wrong for any editor of the prefix: a
        // debugger writing `ebx` through a `P` packet edits the `u32` copy,
        // and a wide block that overwrote it would discard the write and read
        // back the old value. That is exactly what
        // `a_real_gdb_debugs_a_guest_end_to_end` caught.
        for reg in Reg::WIDE {
            let wide = r.read_u64()?;
            let value = match reg {
                // `r8`-`r15` are long-mode-only and have no `u32` counterpart
                // in the prefix, so there is nothing there to preserve and
                // reading one would merge in whatever the prefix loop happened
                // to leave behind. The wide value is the whole story.
                Reg::R8
                | Reg::R9
                | Reg::R10
                | Reg::R11
                | Reg::R12
                | Reg::R13
                | Reg::R14
                | Reg::R15 => wide,
                _ => (wide & !0xffff_ffff) | (reg.get(&state.regs) & 0xffff_ffff),
            };
            reg.set(&mut state.regs, value);
        }
        state.sys.cr4 = r.read_u64()?;
        state.sys.efer = r.read_u64()?;
        state.sys.fs_base = r.read_u64()?;
        state.sys.gs_base = r.read_u64()?;
        state.sys.kernel_gs_base = r.read_u64()?;
        state.sys.star = r.read_u64()?;
        state.sys.lstar = r.read_u64()?;
        state.sys.cstar = r.read_u64()?;
        state.sys.sfmask = r.read_u64()?;
        // The floating-point block, in the order `save` wrote it. Nothing here
        // was written twice, so — unlike the wide register block above — the
        // values simply win.
        for i in 0..8 {
            let sig = r.read_u64()?;
            let sign_exp = r.read_u16()?;
            state.x87.regs[i] = crate::float::x87::F80::new(sign_exp, sig);
        }
        state.x87.control = r.read_u16()?;
        state.x87.status = r.read_u16()?;
        // The tag word is architectural state and is restored as it was
        // written, **not** recomputed from the register contents: `FRSTOR` can
        // legitimately leave the two disagreeing, and a load that "fixed" it
        // would silently change what the guest sees.
        state.x87.tag = r.read_u16()?;
        state.x87.last_op = r.read_u16()?;
        state.x87.last_ip = r.read_u64()?;
        state.x87.last_dp = r.read_u64()?;
        state.x87.last_cs = r.read_u16()?;
        state.x87.last_ds = r.read_u16()?;
        for i in 0..16 {
            let lo = r.read_u64()?;
            let hi = r.read_u64()?;
            state.sse.xmm[i] = [lo, hi];
        }
        state.sse.mxcsr = r.read_u32()?;
        // The multiprocessor block, in the order `save` wrote it.
        state.wait_for_sipi = r.read_bool()?;
        let init_pin = r.read_bool()?;
        let init_peer = r.read_bool()?;
        let init_latch = r.read_bool()?;
        let has_startup = r.read_bool()?;
        let startup_page = r.read_u8()?;
        let startup = has_startup.then_some(startup_page);
        // The translation-lookaside buffer is derived, so it is not in the
        // snapshot and starts empty — which is correct rather than merely
        // convenient, because the page tables it would cache have just been
        // restored underneath it.
        state.tlb.flush();
        self.session.lock().state = state;
        self.lines.restore((intr, nmi_level, nmi_latch, vector));
        self.lines.set_a20(a20);
        self.lines
            .restore_startup(init_pin, init_peer, init_latch, startup);
        Ok(())
    }
}

impl Initiator for X86 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a core needs its two address spaces, and this is
/// where the machine gives it them.
///
/// `space =` is structural and there is exactly one of it, so the **I/O** space
/// is named by the `iospace` string property and looked up by name. That is not
/// a workaround: a machine with two x86 cores sharing one port space and
/// separate memory is a real configuration, and naming the second space keeps
/// it expressible.
impl crate::machine::Instance for X86 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let memory = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("an x86 needs an address space to fetch from (`space = mem`)"),
        })?;
        self.attach_space(Arc::clone(memory));
        if !self.iospace.is_empty() {
            let io = ctx
                .space_named(&self.iospace)
                .ok_or_else(|| Error::Config {
                    at: ctx.path().to_string(),
                    message: alloc::format!(
                        "`iospace = \"{}\"` names no address space in this machine",
                        self.iospace
                    ),
                })?;
            self.attach_io_space(Arc::clone(io));
        }
        self.set_requester(ctx.requester());
        Ok(())
    }
}

/// Bind [`CLASS`] and [`I8086_CLASS`] into the machine graph.
///
/// # Errors
///
/// If either class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(X86::from_props_defaulting(
            props,
            Variant::I80486,
        )?))
    })?;
    bindings.bind(I8086_CLASS.name, |props| {
        Ok(Arc::new(
            X86::from_props_defaulting(props, Variant::I8088)?.as_i8086(),
        ))
    })
}

/// What the validator should know about both class names.
#[must_use]
pub fn schemas() -> Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![schema_for(CLASS.name), schema_for(I8086_CLASS.name)]
}

/// One class's schema. The two differ only in the name and in what
/// "unspecified" means for `variant`, which the validator does not police.
fn schema_for(name: &'static str) -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(name)
        .prop(PropSchema::new("variant", ValueKind::Str).values(Variant::NAMES))
        .prop(PropSchema::new("model", ValueKind::Str).values(Variant::NAMES))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .prop(PropSchema::new("iospace", ValueKind::Str))
        // Inputs only: the outputs a real part has — `M/IO`, `LOCK`, the
        // `INTA` strobes — are the address space's business or the acknowledge
        // cycle's, not a wire's.
        .port("intr", PortDir::In)
        .port("nmi", PortDir::In)
        .port("reset", PortDir::In)
        // The lesser restart. Offered on every class here even though a 16-bit
        // part has no such pin — the schema describes the class, and the class
        // covers five parts; `Device::sink` is where the part decides, and it
        // refuses this one on an 8086.
        .port("init", PortDir::In)
        // Not a processor pin on real silicon: the gate is in the chipset,
        // between the CPU and the bus. It is an input here because this core
        // does its own address wrapping and the gate is exactly a suppression
        // of that wrap — see [`Lines::a20_mask`].
        .port("a20", PortDir::In)
}

/// Which input a [`InputPin`] drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Input {
    /// `INTR`, level-sensitive and gated by `IF`.
    Intr,
    /// `NMI`, edge-sensitive.
    Nmi,
    /// `RESET`, latched on assertion.
    Reset,
    /// `INIT`, level-sensitive: the rising edge runs the INIT sequence and the
    /// level holds the processor there until it drops.
    Init,
    /// The A20 gate: high opens it, low masks address bit 20.
    A20,
}

/// One of the core's input pins, as something a [`Wire`] can drive.
///
/// The pin keeps a handle on the core's *input latches*, not on the core: the
/// core owns the pin — something must, since a net holds only a weak reference
/// to its sinks — and a pin that owned the core back would be a cycle the
/// machine could never drop.
///
/// Every pin wire-ORs its sources. On an AT that is load-bearing rather than
/// defensive: `A20` and `RESET` each have two drivers, the keyboard controller
/// and the chipset's fast port, and either releasing must not drop a line the
/// other is holding.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InputPin {
    lines: Arc<Lines>,
    which: Input,
    inputs: FanIn,
    resolve: Resolve,
}

impl InputPin {
    fn new(lines: Arc<Lines>, which: Input, sources: &[WireId]) -> InputPin {
        InputPin {
            lines,
            which,
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

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.resolve(self.resolve).is_high();
        match self.which {
            Input::Intr => self.lines.set_intr(asserted),
            Input::Nmi => self.lines.set_nmi(asserted),
            // Latch on assertion rather than on release: a machine holding its
            // reset button down should still come up, instead of waiting for a
            // release nobody modelled.
            Input::Reset => {
                if asserted {
                    self.lines.request_reset();
                }
            }
            // Unlike `RESET`, the level is kept as well as the edge: `INIT` is
            // one half of a level-triggered pair (SDM Vol 3A §10.6.1), and a
            // processor whose `INIT` is still asserted is held in reset rather
            // than restarted.
            Input::Init => self.lines.set_init_pin(asserted),
            Input::A20 => self.lines.set_a20(asserted),
        }
    }
}

/// One of the CPU's two interrupt inputs, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, because a net with several drivers is resolved
/// by whoever cares. A PC's `INTR` line comes from one 8259A, but its NMI is
/// wire-ORed from the parity checker and the coprocessor, so this keeps a
/// [`FanIn`].
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
    /// Connect `which` pin of `cpu` to a net driven by `sources`.
    ///
    /// Wire-OR by default: any source asserting asserts the pin, which is how
    /// an open-collector interrupt line behaves.
    ///
    /// The pin holds the core's input latches rather than the core, for the
    /// reason [`InputPin`] gives: the core owns the pin, and owning it back
    /// would close a cycle the machine could never drop.
    #[must_use]
    pub fn new(cpu: Arc<X86>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines: Arc::clone(&cpu.lines),
            which,
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

    /// Which pin this is.
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
            Interrupt::Intr => self.lines.set_intr(asserted),
            Interrupt::Nmi => self.lines.set_nmi(asserted),
        }
    }
}

/// A description of the 8086's opcode map, for `rsemu describe cpu.i8086`.
#[must_use]
pub fn describe_isa() -> String {
    describe_isa_for(Variant::I8088)
}

/// A description of one part's opcode map.
///
/// Built from [`isa::TABLE`] and its 386 delta, so it cannot drift from what
/// the interpreter implements. Group opcodes are expanded one extension per
/// line, because a map that hides `F7 /6` hides the divide, and the two-byte
/// page is listed after the primary one on the parts that have it.
#[must_use]
pub fn describe_isa_for(variant: Variant) -> String {
    use core::fmt::Write as _;
    let map = variant.map();
    let mut out = String::new();
    let mark = |class: isa::Class| match class {
        isa::Class::Documented => ' ',
        isa::Class::Alias => '=',
        isa::Class::Undocumented => '*',
        isa::Class::Undefined => '?',
        isa::Class::Prefix => ':',
        isa::Class::Escape => '~',
    };
    let row = |out: &mut String, prefix: &str, opcode: u8, insn: isa::Insn| {
        if insn.group == isa::Grp::None {
            let _ = writeln!(
                out,
                "{prefix}{opcode:02x}    {}{:<7} {}",
                mark(insn.class),
                insn.op.mnemonic(),
                insn.op.summary()
            );
        } else {
            for reg in 0..8u8 {
                let sub = isa::resolve_as(map, insn, reg);
                let _ = writeln!(
                    out,
                    "{prefix}{opcode:02x}/{reg} {}{:<7} {}",
                    mark(sub.class),
                    sub.op.mnemonic(),
                    sub.op.summary()
                );
            }
        }
    };
    for opcode in 0..=255u8 {
        row(&mut out, "", opcode, isa::decode_as(map, opcode));
    }
    if matches!(map, isa::Gen::I386) {
        for opcode in 0..=255u8 {
            if !isa::LISTED_0F[opcode as usize] {
                continue;
            }
            row(&mut out, "0f ", opcode, isa::decode_0f(opcode));
        }
    }
    // The 64-bit column, listed separately rather than folded in, because a
    // part that has long mode still decodes the legacy map above whenever it
    // is not in it. Generated from the same rows, so an encoding cannot be
    // reclaimed in the decoder and not here.
    if variant.features().long {
        let _ = writeln!(out, "\n-- 64-bit mode differs --");
        let long_row = |out: &mut String, prefix: &str, opcode: u8, insn: isa::Insn| {
            if matches!(insn.long, isa::L64::Same) {
                return;
            }
            let now = insn.in_long();
            let _ = writeln!(
                out,
                "{prefix}{opcode:02x}    {:<7} -> {}{:<7} {}",
                insn.op.mnemonic(),
                mark(now.class),
                now.op.mnemonic(),
                now.op.summary()
            );
        };
        for opcode in 0..=255u8 {
            long_row(&mut out, "", opcode, isa::decode_as(map, opcode));
        }
        for opcode in 0..=255u8 {
            long_row(&mut out, "0f ", opcode, isa::decode_0f(opcode));
        }
    }
    out
}
