//! Intel x86: the 8086 and 8088 in real mode, and the 80386 and 80486 with
//! protection, paging and 32-bit operands.
//!
//! `ROADMAP.md` §6 calls x86 "the hard one". One interpreter covers all four
//! parts, selected by [`Variant`] rather than by a second module, because the
//! generations really are close to a superset chain — and where they are not,
//! the difference is named and modelled rather than flattened (the table in
//! the private `exec` module lists all ten).
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
//! **Paging** ([`paging`]): the two-level directory and table walk, `CR2` and
//! `CR3`, the accessed and dirty bits written by the walk itself, a
//! translation-lookaside buffer so they are written once rather than on every
//! access, `INVLPG`, and page faults with the three-bit error code.
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
//! - **No floating-point unit.** There is no 387 and no `CPUID` bit claiming
//!   one; with `CR0.EM` or `CR0.TS` set an escape raises `#NM` so software can
//!   emulate, which is what an operating system that wants to do so asks for.
//! - **No virtual-8086 mode.** `EFLAGS.VM` has storage and nothing sets it.
//! - **No debug breakpoints.** `DR0`-`DR7` round-trip; arming one fires
//!   nothing.
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
//! assert_eq!(cpu.regs().eax & 0xffff, 0x1234);
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
//! were cross-checked against GNU `as` and `objdump`. **No copyleft emulator
//! was consulted** — `docs/cpu/x86.md` names the three that people reach for
//! when x86 gets hard and records that all three are forbidden.

pub mod disasm;
mod exec;
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
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
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
            Variant::I80386 | Variant::I80486 => 16,
        }
    }

    /// How many bytes one bus cycle can transfer.
    #[must_use]
    pub const fn bus_bytes(self) -> u8 {
        match self {
            Variant::I8086 => 2,
            Variant::I8088 => 1,
            Variant::I80386 | Variant::I80486 => 4,
        }
    }

    /// How many clocks one bus cycle costs with no wait states.
    ///
    /// Four T-states on an 8086, two on a 386 or 486.
    #[must_use]
    pub const fn bus_clocks(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => 4,
            Variant::I80386 | Variant::I80486 => 2,
        }
    }

    /// Which opcode map this part decodes with.
    #[must_use]
    pub const fn map(self) -> isa::Gen {
        match self {
            Variant::I8086 | Variant::I8088 => isa::Gen::I8086,
            Variant::I80386 | Variant::I80486 => isa::Gen::I386,
        }
    }

    /// Whether this part has 32-bit registers, protected mode and paging.
    #[must_use]
    pub const fn is_32bit(self) -> bool {
        matches!(self, Variant::I80386 | Variant::I80486)
    }

    /// Whether this part implements `CPUID` and the other 80486 additions.
    #[must_use]
    pub const fn has_486_extras(self) -> bool {
        matches!(self, Variant::I80486)
    }

    /// The bits the flags register has storage for.
    #[must_use]
    pub const fn flag_mask(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => flags::DEFINED,
            Variant::I80386 => flags::DEFINED_386,
            Variant::I80486 => flags::DEFINED_486,
        }
    }

    /// The bits the flags register always reads as one.
    #[must_use]
    pub const fn flag_fixed(self) -> u32 {
        match self {
            Variant::I8086 | Variant::I8088 => flags::RESERVED_SET,
            Variant::I80386 | Variant::I80486 => flags::ALWAYS_SET,
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
        }
    }

    /// Every name a machine description may write.
    pub const NAMES: &'static [&'static str] = &["8086", "8088", "80386", "80486"];

    /// Look one up by the name a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        match name {
            "8086" => Some(Variant::I8086),
            "8088" => Some(Variant::I8088),
            "80386" | "386" | "i386" => Some(Variant::I80386),
            "80486" | "486" | "i486" => Some(Variant::I80486),
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
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// An 8088: 8-bit bus, four-byte queue.
    pub const I8088: Config = Config {
        variant: Variant::I8088,
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
        ..Config::I8088
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Same configuration, as a different part.
    #[must_use]
    pub const fn with_variant(mut self, variant: Variant) -> Self {
        self.variant = variant;
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
pub const fn linear(segment: u16, offset: u16) -> u32 {
    (((segment as u32) << 4).wrapping_add(offset as u32)) & 0xf_ffff
}

/// The architectural register file.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read
/// it out and put it back — this is the surface a future gdbstub serialises
/// (`ROADMAP.md` §9's debug story), and [`Reg`] enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Regs {
    /// Accumulator. `AX` is its low half and `AL`/`AH` its low two bytes.
    pub eax: u32,
    /// Count — the implicit operand of `LOOP`, the string repeats, and the
    /// variable shifts.
    pub ecx: u32,
    /// Data — the implicit high half of a multiply or divide, and the I/O port
    /// register.
    pub edx: u32,
    /// Base.
    pub ebx: u32,
    /// Stack pointer, in `SS`.
    pub esp: u32,
    /// Base pointer; addressing modes that use it default to `SS`.
    pub ebp: u32,
    /// Source index, for the string instructions.
    pub esi: u32,
    /// Destination index, for the string instructions.
    pub edi: u32,
    /// Instruction pointer, an offset within `CS`.
    pub eip: u32,
    /// The flags register. See [`flags`].
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
            eax: 0,
            ecx: 0,
            edx: 0,
            ebx: 0,
            esp: 0,
            ebp: 0,
            esi: 0,
            edi: 0,
            eip: 0,
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

    /// Read one of the eight 32-bit registers by ModRM number.
    #[inline]
    #[must_use]
    pub const fn dword(&self, index: u8) -> u32 {
        match index & 7 {
            0 => self.eax,
            1 => self.ecx,
            2 => self.edx,
            3 => self.ebx,
            4 => self.esp,
            5 => self.ebp,
            6 => self.esi,
            _ => self.edi,
        }
    }

    /// Write one of the eight 32-bit registers by ModRM number.
    #[inline]
    pub const fn set_dword(&mut self, index: u8, value: u32) {
        match index & 7 {
            0 => self.eax = value,
            1 => self.ecx = value,
            2 => self.edx = value,
            3 => self.ebx = value,
            4 => self.esp = value,
            5 => self.ebp = value,
            6 => self.esi = value,
            _ => self.edi = value,
        }
    }

    /// Read one of the eight 16-bit registers by ModRM number.
    #[inline]
    #[must_use]
    pub const fn word(&self, index: u8) -> u16 {
        self.dword(index) as u16
    }

    /// Write one of the eight 16-bit registers by ModRM number.
    ///
    /// The high half is **preserved**, which is the 386's rule and not an
    /// implementation convenience: `mov ax, 0` leaves the top of `EAX` alone,
    /// and code that switches between operand sizes depends on it.
    #[inline]
    pub const fn set_word(&mut self, index: u8, value: u16) {
        let merged = (self.dword(index) & 0xffff_0000) | value as u32;
        self.set_dword(index, merged);
    }

    /// Read one of the eight 8-bit registers by ModRM number.
    ///
    /// Numbers 0-3 are the low halves of `AX`-`BX` and 4-7 the high halves, in
    /// the same register order — which is why `AH` is 4 and not 1.
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

    /// Write one of the eight 8-bit registers by ModRM number.
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

    /// Read a general register at a width of 1, 2 or 4 bytes.
    #[inline]
    #[must_use]
    pub const fn read(&self, index: u8, size: u8) -> u32 {
        match size {
            1 => self.byte(index) as u32,
            2 => self.word(index) as u32,
            _ => self.dword(index),
        }
    }

    /// Write a general register at a width of 1, 2 or 4 bytes.
    #[inline]
    pub const fn write(&mut self, index: u8, size: u8, value: u32) {
        match size {
            1 => self.set_byte(index, value as u8),
            2 => self.set_word(index, value as u16),
            _ => self.set_dword(index, value),
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
            "EAX:{:08x} EBX:{:08x} ECX:{:08x} EDX:{:08x} ESP:{:08x} EBP:{:08x} ESI:{:08x} \
             EDI:{:08x} ES:{:04x} CS:{:04x} SS:{:04x} DS:{:04x} FS:{:04x} GS:{:04x} \
             EIP:{:08x} F:{:08x}",
            self.eax,
            self.ebx,
            self.ecx,
            self.edx,
            self.esp,
            self.ebp,
            self.esi,
            self.edi,
            self.es,
            self.cs,
            self.ss,
            self.ds,
            self.fs,
            self.gs,
            self.eip,
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
            _ => Width::U16,
        }
    }

    /// Read this register out of a register file.
    ///
    /// A 16-bit view returns its low half zero-extended, so a caller that
    /// asked for `ax` never sees the other sixteen bits by accident.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u32 {
        match self {
            Reg::Eax => regs.eax,
            Reg::Ecx => regs.ecx,
            Reg::Edx => regs.edx,
            Reg::Ebx => regs.ebx,
            Reg::Esp => regs.esp,
            Reg::Ebp => regs.ebp,
            Reg::Esi => regs.esi,
            Reg::Edi => regs.edi,
            Reg::Eip => regs.eip,
            Reg::Eflags => regs.eflags,
            Reg::Es => regs.es as u32,
            Reg::Cs => regs.cs as u32,
            Reg::Ss => regs.ss as u32,
            Reg::Ds => regs.ds as u32,
            Reg::Fs => regs.fs as u32,
            Reg::Gs => regs.gs as u32,
            Reg::Ax => regs.eax & 0xffff,
            Reg::Cx => regs.ecx & 0xffff,
            Reg::Dx => regs.edx & 0xffff,
            Reg::Bx => regs.ebx & 0xffff,
            Reg::Sp => regs.esp & 0xffff,
            Reg::Bp => regs.ebp & 0xffff,
            Reg::Si => regs.esi & 0xffff,
            Reg::Di => regs.edi & 0xffff,
            Reg::Ip => regs.eip & 0xffff,
            Reg::Flags => regs.eflags & 0xffff,
        }
    }

    /// Write this register into a register file.
    ///
    /// A 16-bit view leaves the high half alone, exactly as a 16-bit
    /// instruction does. Nothing here normalises the flags: the hard-wired
    /// bits depend on the [`Variant`], which a bare register file does not
    /// know, so [`X86::set_reg`] does it where the part is in scope.
    pub const fn set(self, regs: &mut Regs, value: u32) {
        match self {
            Reg::Eax => regs.eax = value,
            Reg::Ecx => regs.ecx = value,
            Reg::Edx => regs.edx = value,
            Reg::Ebx => regs.ebx = value,
            Reg::Esp => regs.esp = value,
            Reg::Ebp => regs.ebp = value,
            Reg::Esi => regs.esi = value,
            Reg::Edi => regs.edi = value,
            Reg::Eip => regs.eip = value,
            Reg::Eflags => regs.eflags = value,
            Reg::Es => regs.es = value as u16,
            Reg::Cs => regs.cs = value as u16,
            Reg::Ss => regs.ss = value as u16,
            Reg::Ds => regs.ds = value as u16,
            Reg::Fs => regs.fs = value as u16,
            Reg::Gs => regs.gs = value as u16,
            Reg::Ax => regs.eax = (regs.eax & 0xffff_0000) | (value & 0xffff),
            Reg::Cx => regs.ecx = (regs.ecx & 0xffff_0000) | (value & 0xffff),
            Reg::Dx => regs.edx = (regs.edx & 0xffff_0000) | (value & 0xffff),
            Reg::Bx => regs.ebx = (regs.ebx & 0xffff_0000) | (value & 0xffff),
            Reg::Sp => regs.esp = (regs.esp & 0xffff_0000) | (value & 0xffff),
            Reg::Bp => regs.ebp = (regs.ebp & 0xffff_0000) | (value & 0xffff),
            Reg::Si => regs.esi = (regs.esi & 0xffff_0000) | (value & 0xffff),
            Reg::Di => regs.edi = (regs.edi & 0xffff_0000) | (value & 0xffff),
            Reg::Ip => regs.eip = (regs.eip & 0xffff_0000) | (value & 0xffff),
            Reg::Flags => regs.eflags = (regs.eflags & 0xffff_0000) | (value & 0xffff),
        }
    }

    /// Look a register up by name, 16-bit views included.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL
            .iter()
            .chain(Reg::NARROW.iter())
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
#[derive(Debug, Default)]
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
    lines: Lines,
    session: sync::Mutex<Session>,
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
            lines: Lines::default(),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(cfg.variant),
                    memory: None,
                    io: None,
                },
            ),
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
        r.finish()?;
        Ok(X86::new(Config {
            variant,
            requester: RequesterId::ANONYMOUS,
        }))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
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
                entry.base = u32::from(selector) << 4;
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

    /// Read one register by name. A 16-bit view returns its low half.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u32 {
        reg.get(&self.session.lock().state.regs)
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u32) {
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
                entry.base = u32::from(selector) << 4;
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
    pub fn bus_faults(&self) -> (u64, u32) {
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

    /// Execute one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the clock cycles charged: zero if the core is halted with no
    /// interrupt pending, or has no address space, which the caller must treat
    /// as "stop", not "retry".
    pub fn step(&self) -> u64 {
        let mut session = self.session.lock();
        let Session { state, memory, io } = &mut *session;
        let Some(memory) = memory.clone() else {
            return 0;
        };
        let io = io.clone();
        Exec::new(state, &memory, io.as_deref(), &self.cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — an 8086 cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    /// Stops early if the core halts.
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

    /// Disassemble `count` instructions starting at the current `CS:EIP`,
    /// reading guest memory with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around `EIP`
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    ///
    /// Reads go through the code segment's **cached base**, so a listing in
    /// protected mode shows the instructions the processor would fetch rather
    /// than the ones at `selector << 4`. Paging is not walked: a debugger that
    /// wants a paged listing has to translate for itself, because walking the
    /// tables here would set accessed bits and a debug read must not.
    #[must_use]
    pub fn disassemble(&self, cs: u16, eip: u32, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        let (base, bits32, legacy) = {
            let session = self.session.lock();
            let seg = session.state.sys.seg(isa::seg::CS);
            let legacy = !self.cfg.variant.is_32bit();
            let base = if seg.selector == cs {
                seg.base
            } else {
                u32::from(cs) << 4
            };
            (base, !legacy && seg.big(), legacy)
        };
        let map = self.cfg.variant.map();
        disasm::disassemble_run_as(map, bits32, cs, eip, count, |addr| {
            let addr = if legacy {
                u64::from(base.wrapping_add(addr) & 0xf_ffff)
            } else {
                u64::from(base.wrapping_add(addr))
            };
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
    version: 2,
    summary: "Intel x86 CPU core: 8086/8088 real mode, or 80386/80486 with protection and paging",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "\"8086\", \"8088\", \"80386\" or \"80486\" (the default)",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "accepted as a synonym for \"variant\", which this class used to be called",
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
    version: 2,
    summary: "Intel 8086 / 8088 16-bit CPU core, real mode, hardware-checked interpreter",
    properties: CLASS.properties,
    construct: |props| Ok(Box::new(X86::from_props_defaulting(props, Variant::I8088)?)),
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
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A CPU with no address space cannot fetch, and failing here is the
        // difference between a config error and a machine that runs zero
        // instructions and says nothing.
        if self.session.lock().memory.is_none() {
            return Err(ctx.error("no memory address space attached to this core"));
        }
        Ok(())
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
        } else {
            // The input *levels* belong to whatever drives them, not to the
            // CPU — clearing them here would make a reset lie about the
            // machine. The edge latch is internal, so it goes.
            self.lines.clear_nmi_latch();
        }
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
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state;
        for reg in Reg::ALL {
            w.write_u32(reg.get(&state.regs))?;
        }
        w.write_u64(state.cycles)?;
        for index in 0..isa::seg::COUNT as u8 {
            let s = state.sys.seg(index);
            w.write_u16(s.selector)?;
            w.write_u32(s.base)?;
            w.write_u32(s.limit)?;
            w.write_u32(s.ar)?;
        }
        for s in [state.sys.ldtr, state.sys.task] {
            w.write_u16(s.selector)?;
            w.write_u32(s.base)?;
            w.write_u32(s.limit)?;
            w.write_u32(s.ar)?;
        }
        for t in [state.sys.gdtr, state.sys.idtr] {
            w.write_u32(t.base)?;
            w.write_u32(t.limit)?;
        }
        w.write_u32(state.sys.cr0)?;
        w.write_u32(state.sys.cr2)?;
        w.write_u32(state.sys.cr3)?;
        for value in state.sys.dr {
            w.write_u32(value)?;
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
        w.write_u32(state.last_fault)?;
        let queue = state.queue.contents();
        w.write_u8(queue.len() as u8)?;
        for byte in queue {
            w.write_u8(byte)?;
        }
        let (intr, nmi_level, nmi_latch, vector) = self.lines.snapshot();
        w.write_bool(intr)?;
        w.write_bool(nmi_level)?;
        w.write_bool(nmi_latch)?;
        w.write_u8(vector)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.cfg.variant);
        for reg in Reg::ALL {
            let value = r.read_u32()?;
            reg.set(&mut state.regs, value);
        }
        state.cycles = r.read_u64()?;
        for index in 0..isa::seg::COUNT as u8 {
            let s = state.sys.seg_mut(index);
            s.selector = r.read_u16()?;
            s.base = r.read_u32()?;
            s.limit = r.read_u32()?;
            s.ar = r.read_u32()?;
        }
        for slot in [0usize, 1] {
            let s = prot::SegReg {
                selector: r.read_u16()?,
                base: r.read_u32()?,
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
                base: r.read_u32()?,
                limit: r.read_u32()?,
            };
            if slot == 0 {
                state.sys.gdtr = t;
            } else {
                state.sys.idtr = t;
            }
        }
        state.sys.cr0 = r.read_u32()?;
        state.sys.cr2 = r.read_u32()?;
        state.sys.cr3 = r.read_u32()?;
        for i in 0..8 {
            state.sys.dr[i] = r.read_u32()?;
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
        state.last_fault = r.read_u32()?;
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
        let intr = r.read_bool()?;
        let nmi_level = r.read_bool()?;
        let nmi_latch = r.read_bool()?;
        let vector = r.read_u8()?;
        // The translation-lookaside buffer is derived, so it is not in the
        // snapshot and starts empty — which is correct rather than merely
        // convenient, because the page tables it would cache have just been
        // restored underneath it.
        state.tlb.flush();
        self.session.lock().state = state;
        self.lines.restore((intr, nmi_level, nmi_latch, vector));
        Ok(())
    }
}

impl Initiator for X86 {
    fn requester(&self) -> RequesterId {
        self.cfg.requester
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
    cpu: Arc<X86>,
    which: Interrupt,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect `which` pin of `cpu` to a net driven by `sources`.
    ///
    /// Wire-OR by default: any source asserting asserts the pin, which is how
    /// an open-collector interrupt line behaves.
    #[must_use]
    pub fn new(cpu: Arc<X86>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            cpu,
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
            Interrupt::Intr => self.cpu.set_intr(asserted),
            Interrupt::Nmi => self.cpu.set_nmi(asserted),
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
    out
}
