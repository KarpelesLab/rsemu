//! Protected mode: descriptors, selectors, privilege, gates and tasks.
//!
//! This file holds the *data* half of protection — the shapes the 80386 puts
//! in memory and in its hidden registers, and the pure predicates over them.
//! The half that touches the bus (walking a table, loading a segment, taking a
//! gate, switching a task) belongs to the interpreter, because every one of
//! those operations can fault and faulting is the interpreter's business.
//!
//! # The hidden descriptor cache is architectural state
//!
//! A segment register on a 386 is not a number. It is a *selector* plus a
//! cached copy of the descriptor the selector named **at the moment it was
//! loaded** — base, limit and access rights — and the processor uses the cache
//! on every access without ever looking at the table again. Two consequences,
//! and both of them are load-bearing:
//!
//! - Rewriting a descriptor in the GDT changes nothing until something
//!   reloads the segment register. Emulators that translate `segment:offset`
//!   by reading the table each time appear to work and then fail on the first
//!   guest that edits a live descriptor.
//! - The cache survives a return to real mode. Load a 4 GiB limit in
//!   protected mode, clear `CR0.PE`, and `DS` still reaches 4 GiB with a
//!   16-bit offset — "unreal mode", which real 386 and 486 silicon does and
//!   which BIOSes use to copy above 1 MiB without leaving real mode.
//!
//! So [`SegReg`] is **saved and restored**, unlike the translation-lookaside
//! buffer beside it in [`paging`](super::paging). CLAUDE.md's rule that derived state
//! is never serialized is about state that can be re-derived; this cannot be,
//! and a snapshot that dropped it would silently break unreal mode across a
//! save/load. The TLB, which *is* re-derivable from the page tables, is not
//! serialized.
//!
//! # Sources
//!
//! Intel's *80386 Programmer's Reference Manual* — chapter 5 (memory
//! management), chapter 6 (protection), chapter 7 (multitasking) and chapter 9
//! (exceptions and interrupts) — and the *Intel 64 and IA-32 Architectures
//! Software Developer's Manual*, volume 3, for the same material restated.
//! Hardware documentation only: no copyleft emulator was consulted, and
//! `docs/cpu/x86.md` names the ones that are forbidden.

use super::isa::seg;
use super::paging;

// ---------------------------------------------------------------------------
// Selectors
// ---------------------------------------------------------------------------

/// A segment selector's parts.
///
/// Thirteen bits of index, one bit choosing the global or the local table, and
/// two bits of requested privilege level. The RPL is the reason `ARPL` exists:
/// a selector handed inward by a less privileged caller must not be trusted to
/// have a privilege level lower than the caller's own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Selector(pub u16);

impl Selector {
    /// The null selector. Loading it into `DS`-`GS` is legal and makes the
    /// segment unusable; loading it into `SS` or `CS` is a fault.
    pub const NULL: Selector = Selector(0);

    /// The descriptor's index within its table.
    #[inline]
    #[must_use]
    pub const fn index(self) -> u32 {
        (self.0 >> 3) as u32
    }

    /// The byte offset of the descriptor within its table.
    #[inline]
    #[must_use]
    pub const fn offset(self) -> u32 {
        (self.0 & 0xfff8) as u32
    }

    /// Whether the selector names the local descriptor table.
    #[inline]
    #[must_use]
    pub const fn is_ldt(self) -> bool {
        self.0 & 4 != 0
    }

    /// The requested privilege level, 0 to 3.
    #[inline]
    #[must_use]
    pub const fn rpl(self) -> u8 {
        (self.0 & 3) as u8
    }

    /// Whether this is the null selector — index 0 of the global table, which
    /// the architecture reserves as "no segment".
    #[inline]
    #[must_use]
    pub const fn is_null(self) -> bool {
        self.0 & 0xfffc == 0
    }

    /// The same selector with a different requested privilege level.
    #[inline]
    #[must_use]
    pub const fn with_rpl(self, rpl: u8) -> Selector {
        Selector((self.0 & 0xfffc) | (rpl as u16 & 3))
    }
}

// ---------------------------------------------------------------------------
// Descriptor types
// ---------------------------------------------------------------------------

/// The four-bit `TYPE` field of a **system** descriptor.
///
/// Application descriptors — code and data — use the same four bits for their
/// own flags, which is what the `S` bit distinguishes.
pub mod sys_type {
    /// An available 80286 task state segment.
    pub const TSS16_AVAIL: u8 = 1;
    /// A local descriptor table.
    pub const LDT: u8 = 2;
    /// A busy 80286 task state segment.
    pub const TSS16_BUSY: u8 = 3;
    /// An 80286 call gate.
    pub const CALL_GATE16: u8 = 4;
    /// A task gate. The same in both sizes: it names a TSS and nothing else.
    pub const TASK_GATE: u8 = 5;
    /// An 80286 interrupt gate.
    pub const INT_GATE16: u8 = 6;
    /// An 80286 trap gate.
    pub const TRAP_GATE16: u8 = 7;
    /// An available 80386 task state segment.
    pub const TSS32_AVAIL: u8 = 9;
    /// A busy 80386 task state segment.
    pub const TSS32_BUSY: u8 = 11;
    /// An 80386 call gate.
    pub const CALL_GATE32: u8 = 12;
    /// An 80386 interrupt gate.
    pub const INT_GATE32: u8 = 14;
    /// An 80386 trap gate.
    pub const TRAP_GATE32: u8 = 15;
}

/// The bits of the access-rights word this core keeps, at their hardware
/// positions within a descriptor's high doubleword.
///
/// Keeping the hardware layout rather than unpacking into fields is what makes
/// `LAR` a mask and `SGDT`-style round-trips exact.
pub mod ar {
    /// Everything a descriptor's high doubleword holds that is not base or
    /// limit — which is exactly what `LAR` returns.
    pub const MASK: u32 = 0x00f0_ff00;
    /// Accessed: set by hardware the first time a selector is loaded.
    pub const ACCESSED: u32 = 0x0000_0100;
    /// For a data segment, writable; for a code segment, readable.
    pub const RW: u32 = 0x0000_0200;
    /// For a data segment, expand-down; for a code segment, conforming.
    pub const DC: u32 = 0x0000_0400;
    /// Executable: this is a code segment.
    pub const CODE: u32 = 0x0000_0800;
    /// Not a system descriptor.
    pub const S: u32 = 0x0000_1000;
    /// Descriptor privilege level, bits 13-14.
    pub const DPL: u32 = 0x0000_6000;
    /// How far to shift [`DPL`] down.
    pub const DPL_SHIFT: u32 = 13;
    /// Present.
    pub const PRESENT: u32 = 0x0000_8000;
    /// Available to software; the architecture never looks at it.
    pub const AVL: u32 = 0x0010_0000;
    /// Long: a 64-bit code segment. Bit 53 of the descriptor, which is bit 21
    /// of the high doubleword — the bit beside `AVL` that a 386 left as zero.
    pub const L: u32 = 0x0020_0000;
    /// Default operand and address size, or "big" for a stack segment.
    pub const DB: u32 = 0x0040_0000;
    /// Granularity: the limit counts 4 KiB pages rather than bytes.
    pub const GRANULAR: u32 = 0x0080_0000;

    /// The access rights a real-mode data segment has.
    ///
    /// Present, an application descriptor, a writable accessed data segment at
    /// privilege 0 — which is what real mode is: privilege 0 with no checks.
    pub const REAL_DATA: u32 = PRESENT | S | RW | ACCESSED;
    /// The access rights a real-mode code segment has: readable and
    /// executable.
    pub const REAL_CODE: u32 = PRESENT | S | CODE | RW | ACCESSED;
}

/// One segment register: the selector, and the descriptor cached when it was
/// loaded.
///
/// See the module documentation for why the cache is architectural state
/// rather than something to be re-derived.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SegReg {
    /// The selector last written.
    pub selector: u16,
    /// The cached base address.
    ///
    /// Sixty-four bits, because `FS` and `GS` keep a full-width base in
    /// long mode — written through `IA32_FS_BASE`/`IA32_GS_BASE` rather
    /// than through a descriptor, which is why the register is wider than
    /// anything that could be loaded into it from a table.
    pub base: u64,
    /// The cached limit, already expanded by the granularity bit and
    /// **inclusive**: the last byte offset the segment covers.
    pub limit: u32,
    /// The cached access rights, at their hardware bit positions. See [`ar`].
    pub ar: u32,
}

impl SegReg {
    /// A real-mode data segment at a given selector.
    #[must_use]
    pub const fn real_data(selector: u16) -> SegReg {
        SegReg {
            selector,
            base: (selector as u64) << 4,
            limit: 0xffff,
            ar: ar::REAL_DATA,
        }
    }

    /// A real-mode code segment at a given selector.
    #[must_use]
    pub const fn real_code(selector: u16) -> SegReg {
        SegReg {
            selector,
            base: (selector as u64) << 4,
            limit: 0xffff,
            ar: ar::REAL_CODE,
        }
    }

    /// The unusable state a null selector leaves behind.
    #[must_use]
    pub const fn null() -> SegReg {
        SegReg {
            selector: 0,
            base: 0,
            limit: 0,
            ar: 0,
        }
    }

    /// The descriptor privilege level.
    #[inline]
    #[must_use]
    pub const fn dpl(self) -> u8 {
        ((self.ar & ar::DPL) >> ar::DPL_SHIFT) as u8
    }

    /// Whether the present bit is set.
    #[inline]
    #[must_use]
    pub const fn present(self) -> bool {
        self.ar & ar::PRESENT != 0
    }

    /// Whether this is a code or data descriptor rather than a system one.
    #[inline]
    #[must_use]
    pub const fn is_app(self) -> bool {
        self.ar & ar::S != 0
    }

    /// Whether this is an executable segment.
    #[inline]
    #[must_use]
    pub const fn is_code(self) -> bool {
        self.is_app() && self.ar & ar::CODE != 0
    }

    /// Whether this is a data segment.
    #[inline]
    #[must_use]
    pub const fn is_data(self) -> bool {
        self.is_app() && self.ar & ar::CODE == 0
    }

    /// Whether a data segment may be written, or a code segment read.
    #[inline]
    #[must_use]
    pub const fn rw(self) -> bool {
        self.ar & ar::RW != 0
    }

    /// Whether a code segment is conforming — reachable from a *less*
    /// privileged level without changing privilege.
    #[inline]
    #[must_use]
    pub const fn conforming(self) -> bool {
        self.is_code() && self.ar & ar::DC != 0
    }

    /// Whether a data segment expands downward, as a stack does.
    #[inline]
    #[must_use]
    pub const fn expand_down(self) -> bool {
        self.is_data() && self.ar & ar::DC != 0
    }

    /// The `D`/`B` bit: a code segment's default operand size, or a stack
    /// segment's pointer width.
    #[inline]
    #[must_use]
    pub const fn big(self) -> bool {
        self.ar & ar::DB != 0
    }

    /// The `L` bit: this code segment runs in 64-bit mode rather than
    /// compatibility mode.
    ///
    /// `L` and `D` together are how long mode's two submodes are selected, and
    /// `L = 1` with `D = 1` is architecturally invalid — there is no
    /// "64-bit mode with 32-bit defaults", because the defaults *are* the
    /// difference. Loading such a descriptor into `CS` is `#GP`.
    #[inline]
    #[must_use]
    pub const fn long(self) -> bool {
        self.ar & ar::L != 0
    }

    /// The system-descriptor type, meaningful only when [`SegReg::is_app`] is
    /// false.
    #[inline]
    #[must_use]
    pub const fn sys_type(self) -> u8 {
        ((self.ar >> 8) & 0xf) as u8
    }

    /// Whether a segment may be read at all.
    ///
    /// Every data segment may; a code segment only if its readable bit is set.
    #[inline]
    #[must_use]
    pub const fn readable(self) -> bool {
        if self.is_code() {
            self.rw()
        } else {
            self.is_data()
        }
    }

    /// Whether a segment may be written.
    ///
    /// Never through a code segment, whatever its readable bit says — which is
    /// the whole point of having one.
    #[inline]
    #[must_use]
    pub const fn writable(self) -> bool {
        self.is_data() && self.rw()
    }

    /// Whether an offset of `size` bytes starting at `offset` lies inside the
    /// segment.
    ///
    /// Three cases, and the third is the one people get wrong. An ordinary
    /// segment covers `0..=limit`. An expand-down segment covers
    /// `limit+1..=0xffff` (or `..=0xffffffff` when `B` is set), because a
    /// stack grows toward zero and its limit says how far down it may go.
    /// And a zero-size access — which nothing generates here, but a
    /// `size == 0` caller would — must not wrap the addition into success.
    #[must_use]
    pub const fn in_bounds(self, offset: u64, size: u64) -> bool {
        if size == 0 {
            return true;
        }
        let last = match offset.checked_add(size - 1) {
            Some(last) => last,
            None => return false,
        };
        // A limit is thirty-two bits wide however wide the offset is, so
        // an offset above 4 GiB is outside every segment. In 64-bit mode
        // limits are not checked at all and the caller does not get here.
        if last > 0xffff_ffff {
            return false;
        }
        let limit = self.limit as u64;
        if self.expand_down() {
            let top: u64 = if self.big() { 0xffff_ffff } else { 0xffff };
            offset > limit && last <= top
        } else {
            last <= limit
        }
    }
}

/// A descriptor table register: `GDTR` and `IDTR`, which have a base and a
/// limit and no selector at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TableReg {
    /// The linear address of the table's first byte.
    pub base: u64,
    /// The table's limit, inclusive: one less than its size in bytes.
    pub limit: u32,
}

/// A raw descriptor, as eight bytes in a table.
///
/// Kept as the two doublewords rather than unpacked, because half the
/// instructions that touch one (`LAR`, `LSL`, `SGDT`) want the raw bits back
/// and the packing is the architecture's, not ours.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RawDesc {
    /// Bytes 0-3: the low half of the limit and the low half of the base.
    pub low: u32,
    /// Bytes 4-7: the rest of the base, the access rights, and the top nibble
    /// of the limit.
    pub high: u32,
    /// Bytes 8-11 of a **sixteen-byte** system descriptor: the top half of a
    /// base or of a gate's offset. Zero for the eight-byte form.
    ///
    /// Long mode did not widen code and data descriptors at all — they have no
    /// base and no limit there — but it doubled every *system* descriptor, so
    /// an `LDT`, a task state segment and every interrupt gate are sixteen
    /// bytes with the extra address in this doubleword. A table therefore
    /// holds two sizes of entry at once, which is why the reader has to know
    /// which kind it is looking for before it reads.
    pub upper: u32,
}

impl RawDesc {
    /// The base address, reassembled from its three pieces.
    ///
    /// Three pieces because the 80286 had a 24-bit base in one contiguous
    /// field and the 386 had to find sixteen more bits somewhere that a 286
    /// descriptor left zero.
    #[must_use]
    pub const fn base(self) -> u32 {
        (self.low >> 16) | ((self.high & 0xff) << 16) | (self.high & 0xff00_0000)
    }

    /// The limit, expanded by the granularity bit and inclusive.
    ///
    /// When `G` is set the limit counts 4 KiB pages, and the low twelve bits
    /// of the resulting byte limit are **ones**, not zeroes: a granular limit
    /// of 0 covers the first 4096 bytes, not one byte. That is why `G` with a
    /// limit of `0xfffff` reaches exactly 4 GiB.
    #[must_use]
    pub const fn limit(self) -> u32 {
        let raw = (self.low & 0xffff) | (self.high & 0x000f_0000);
        if self.high & ar::GRANULAR != 0 {
            (raw << 12) | 0xfff
        } else {
            raw
        }
    }

    /// The access-rights bits this core keeps.
    #[must_use]
    pub const fn ar(self) -> u32 {
        self.high & ar::MASK
    }

    /// The descriptor privilege level.
    #[must_use]
    pub const fn dpl(self) -> u8 {
        ((self.high & ar::DPL) >> ar::DPL_SHIFT) as u8
    }

    /// Whether the present bit is set.
    #[must_use]
    pub const fn present(self) -> bool {
        self.high & ar::PRESENT != 0
    }

    /// Whether this is a code or data descriptor.
    #[must_use]
    pub const fn is_app(self) -> bool {
        self.high & ar::S != 0
    }

    /// The four-bit type field.
    #[must_use]
    pub const fn kind(self) -> u8 {
        ((self.high >> 8) & 0xf) as u8
    }

    /// The base address including a sixteen-byte descriptor's upper half.
    #[must_use]
    pub const fn base64(self) -> u64 {
        (self.base() as u64) | ((self.upper as u64) << 32)
    }

    /// This descriptor as a loaded segment register.
    #[must_use]
    pub const fn to_seg(self, selector: u16) -> SegReg {
        SegReg {
            selector,
            base: self.base64(),
            limit: self.limit(),
            ar: self.ar(),
        }
    }

    // -- Gates --------------------------------------------------------------
    //
    // A gate reuses the same eight bytes for something else entirely: a
    // selector and a 32-bit offset, split around the access-rights byte.

    /// A gate's target selector.
    #[must_use]
    pub const fn gate_selector(self) -> u16 {
        (self.low >> 16) as u16
    }

    /// A gate's target offset, both halves.
    #[must_use]
    pub const fn gate_offset(self) -> u32 {
        (self.low & 0xffff) | (self.high & 0xffff_0000)
    }

    /// A gate's target offset including a sixteen-byte gate's upper half.
    #[must_use]
    pub const fn gate_offset64(self) -> u64 {
        (self.gate_offset() as u64) | ((self.upper as u64) << 32)
    }

    /// How many doublewords a call gate copies from the old stack to the new.
    ///
    /// Zero in long mode: a 64-bit call gate has no parameter count, because
    /// the field it lived in became the interrupt-stack-table index and
    /// arguments go in registers anyway.
    #[must_use]
    pub const fn gate_argc(self) -> u8 {
        (self.high & 0x1f) as u8
    }

    /// A 64-bit interrupt gate's interrupt-stack-table index, 0 to 7.
    ///
    /// Zero means "use the stack the privilege change selects", which is the
    /// 32-bit behaviour; one to seven name an unconditional stack in the task
    /// state segment, which is what makes a double-fault handler able to run
    /// after the kernel stack itself has gone bad (*Intel SDM* volume 3
    /// §6.14.5).
    #[must_use]
    pub const fn gate_ist(self) -> u8 {
        (self.high & 0x7) as u8
    }
}

// ---------------------------------------------------------------------------
// CR0
// ---------------------------------------------------------------------------

/// The bits of `CR0`.
pub mod cr0 {
    /// Protection enable. Setting it enters protected mode.
    pub const PE: u32 = 1 << 0;
    /// Math present: a coprocessor is installed.
    pub const MP: u32 = 1 << 1;
    /// Emulation: coprocessor instructions raise #NM instead of executing.
    pub const EM: u32 = 1 << 2;
    /// Task switched: the next coprocessor instruction raises #NM so the
    /// operating system can save the floating-point state lazily.
    pub const TS: u32 = 1 << 3;
    /// Extension type: on a 386, whether the coprocessor is a 387 or a 287.
    /// Hard-wired to one on a 486.
    pub const ET: u32 = 1 << 4;
    /// Numeric error (80486): report coprocessor errors as #MF rather than
    /// through the `FERR`/`IGNNE` pins.
    pub const NE: u32 = 1 << 5;
    /// Write protect (80486): supervisor writes obey the page tables'
    /// read-only bit, which is what copy-on-write in kernel space needs.
    pub const WP: u32 = 1 << 16;
    /// Alignment mask (80486).
    pub const AM: u32 = 1 << 18;
    /// Not write-through (80486 cache control).
    pub const NW: u32 = 1 << 29;
    /// Cache disable (80486).
    pub const CD: u32 = 1 << 30;
    /// Paging enable.
    pub const PG: u32 = 1 << 31;

    /// The bits `LMSW` may write: it reaches only the low four, and it can
    /// set `PE` but famously cannot clear it.
    pub const MSW: u32 = PE | MP | EM | TS;

    /// The bits an 80386 has storage for.
    pub const VALID_386: u32 = PE | MP | EM | TS | ET | PG;

    /// The bits an 80486 has storage for.
    pub const VALID_486: u32 = VALID_386 | NE | WP | AM | NW | CD;
}

/// The bits of `CR4` this core models.
///
/// `CR4` is where the post-486 extensions are switched on one at a time, which
/// makes it the register a lattice is actually visible in: a guest sets `PAE`
/// and the paging mode changes underneath it without anything else moving.
///
/// *Intel SDM* volume 3 §2.5.
pub mod cr4 {
    /// Virtual-8086 mode extensions. Storage only — no virtual-8086 mode is
    /// modelled.
    pub const VME: u64 = 1 << 0;
    /// Protected-mode virtual interrupts. Storage only.
    pub const PVI: u64 = 1 << 1;
    /// Time-stamp disable: `RDTSC` becomes privileged. Storage only.
    pub const TSD: u64 = 1 << 2;
    /// Debugging extensions. Storage only.
    pub const DE: u64 = 1 << 3;
    /// Page size extension: 4 MiB pages in a two-level walk.
    pub const PSE: u64 = 1 << 4;
    /// Physical address extension: 64-bit entries and a third level.
    pub const PAE: u64 = 1 << 5;
    /// Machine check enable. Storage only.
    pub const MCE: u64 = 1 << 6;
    /// Page global enable: the global bit in a page-table entry.
    pub const PGE: u64 = 1 << 7;
    /// Performance-counter enable. Storage only.
    pub const PCE: u64 = 1 << 8;
    /// `FXSAVE`/`FXRSTOR` and the SSE state.
    ///
    /// **Storage only, deliberately.** The bit exists because an operating
    /// system writes it before it will use SSE and reads it back to check;
    /// modelling the bit and not the arithmetic is the honest half, and the
    /// `CPUID` feature bits that would invite a guest to use it are clear.
    pub const OSFXSR: u64 = 1 << 9;
    /// Unmasked SIMD floating-point exceptions. Storage only, same reason.
    pub const OSXMMEXCPT: u64 = 1 << 10;
}

/// The bits of `IA32_EFER`, the register long mode is armed from.
///
/// *AMD64 Architecture Programmer's Manual* volume 2 §3.1.7, and *Intel SDM*
/// volume 3 §2.2.1.
pub mod efer {
    /// `SYSCALL`/`SYSRET` enable.
    pub const SCE: u64 = 1 << 0;
    /// Long mode **enable**: software sets this to arm long mode.
    pub const LME: u64 = 1 << 8;
    /// Long mode **active**: the *processor* sets this when paging is enabled
    /// with `LME` on, and clears it when paging goes away.
    ///
    /// The two-bit dance is the whole of the mode transition: software cannot
    /// write `LMA`, and reading it back is how a guest knows it is in long
    /// mode rather than merely having asked to be.
    pub const LMA: u64 = 1 << 10;
    /// No-execute enable: bit 63 of a page-table entry stops being reserved.
    pub const NXE: u64 = 1 << 11;

    /// The bits software may write. `LMA` is conspicuously not among them.
    pub const WRITABLE: u64 = SCE | LME | NXE;
}

/// The addresses of the model-specific registers this core implements.
///
/// A short list on purpose: an unimplemented `RDMSR` raises `#GP`, which is
/// what hardware does for an address it does not have and what lets a guest
/// probe. Inventing a zero for every address is how an emulator convinces a
/// kernel that a feature exists.
pub mod msr {
    /// `IA32_EFER`.
    pub const EFER: u32 = 0xc000_0080;
    /// `IA32_STAR`.
    pub const STAR: u32 = 0xc000_0081;
    /// `IA32_LSTAR`.
    pub const LSTAR: u32 = 0xc000_0082;
    /// `IA32_CSTAR`.
    pub const CSTAR: u32 = 0xc000_0083;
    /// `IA32_FMASK`.
    pub const SFMASK: u32 = 0xc000_0084;
    /// `IA32_FS_BASE`.
    pub const FS_BASE: u32 = 0xc000_0100;
    /// `IA32_GS_BASE`.
    pub const GS_BASE: u32 = 0xc000_0101;
    /// `IA32_KERNEL_GS_BASE`.
    pub const KERNEL_GS_BASE: u32 = 0xc000_0102;
}

/// Whether an address is canonical: bits 63-48 must all equal bit 47.
///
/// The rule that keeps a 64-bit address space from quietly becoming a 64-bit
/// *pointer* space — software cannot store a tag in the top bits and expect
/// the processor to ignore it, which is exactly what it was invented to
/// prevent. A non-canonical address raises `#GP(0)`, or `#SS(0)` through the
/// stack segment, *before* any translation is attempted.
///
/// *Intel SDM* volume 1 §3.3.7.1.
#[inline]
#[must_use]
pub const fn canonical(addr: u64) -> bool {
    // Sign-extend bit 47 and compare. Written as a shift pair rather than a
    // mask so that widening the implemented linear address to 57 bits later is
    // one constant.
    ((addr << 16) as i64 >> 16) as u64 == addr
}

/// The system-register file: everything that is architectural state but is not
/// a general register.
///
/// One struct rather than fields scattered through the execution state,
/// because a task switch replaces almost all of it at once and a snapshot has
/// to write all of it in a fixed order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sys {
    /// The six segment registers with their cached descriptors, indexed by
    /// [`seg::ES`] through [`seg::GS`].
    pub segs: [SegReg; seg::COUNT],
    /// The global descriptor table register.
    pub gdtr: TableReg,
    /// The interrupt descriptor table register. In real mode this is the
    /// vector table's base and limit — which is why `LIDT` can move the
    /// vectors of a real-mode 386 and cannot move an 8086's.
    pub idtr: TableReg,
    /// The local descriptor table register: a selector plus its descriptor.
    pub ldtr: SegReg,
    /// The task register: the current task state segment.
    pub task: SegReg,
    /// Control register 0.
    pub cr0: u32,
    /// Control register 2: the linear address of the last page fault.
    pub cr2: u64,
    /// Control register 3: the top page table's physical base, plus its
    /// low-bit flags on later parts.
    pub cr3: u64,
    /// Control register 4: the extension enables, `PAE` among them.
    ///
    /// Present in the struct on every part and writable only where
    /// [`Features::cr4`](super::Features::cr4) says so — the register file
    /// has one shape, and which parts of it exist is a property of the
    /// instance rather than of the type (`ROADMAP.md` §6.1.1).
    pub cr4: u64,
    /// The extended feature enable register, `IA32_EFER`.
    ///
    /// A model-specific register rather than a control register, which is
    /// how long mode came to be armed by a `WRMSR` and entered by a write
    /// to `CR0`.
    pub efer: u64,
    /// `IA32_FS_BASE`: the base `FS` uses in 64-bit mode.
    pub fs_base: u64,
    /// `IA32_GS_BASE`: the base `GS` uses in 64-bit mode.
    pub gs_base: u64,
    /// `IA32_KERNEL_GS_BASE`: the base `SWAPGS` exchanges `GS` with.
    pub kernel_gs_base: u64,
    /// `IA32_STAR`: the selectors `SYSCALL` and `SYSRET` load.
    pub star: u64,
    /// `IA32_LSTAR`: the 64-bit entry point `SYSCALL` jumps to.
    pub lstar: u64,
    /// `IA32_CSTAR`: the entry point a `SYSCALL` from compatibility mode
    /// jumps to.
    pub cstar: u64,
    /// `IA32_FMASK`: the flags `SYSCALL` clears.
    pub sfmask: u64,
    /// The eight debug registers. `DR0`-`DR3` are addresses, `DR6` is the
    /// status and `DR7` the control word.
    pub dr: [u64; 8],
    /// The test registers. On a 386 only `TR6` and `TR7` exist, and they poke
    /// at the translation-lookaside buffer directly.
    pub test: [u32; 8],
}

impl Sys {
    /// The state a 386 or 486 reset leaves behind.
    ///
    /// `CS` is the interesting one: the *selector* is `f000` but the cached
    /// *base* is `ffff0000`, so `CS:EIP` of `f000:fff0` addresses physical
    /// `fffffff0` — sixteen bytes below the top of the 4 GiB space, where the
    /// ROM is. The first far jump reloads `CS` in real mode, which recomputes
    /// the base as `selector << 4` and drops the processor into the first
    /// megabyte for good. Every PC firmware image begins with that jump, and
    /// an emulator that resets to a base of `000f0000` runs the ROM's *second*
    /// copy or nothing at all.
    #[must_use]
    pub fn reset() -> Sys {
        let mut segs = [SegReg::real_data(0); seg::COUNT];
        segs[seg::CS as usize] = SegReg {
            selector: 0xf000,
            base: 0xffff_0000,
            limit: 0xffff,
            ar: ar::REAL_CODE,
        };
        Sys {
            segs,
            gdtr: TableReg { base: 0, limit: 0 },
            // The real-mode interrupt vector table: 256 entries of four bytes.
            idtr: TableReg {
                base: 0,
                limit: 0x3ff,
            },
            ldtr: SegReg::null(),
            task: SegReg::null(),
            cr0: 0,
            cr2: 0,
            cr3: 0,
            cr4: 0,
            efer: 0,
            fs_base: 0,
            gs_base: 0,
            kernel_gs_base: 0,
            star: 0,
            lstar: 0,
            cstar: 0,
            sfmask: 0,
            dr: [0, 0, 0, 0, 0, 0, 0xffff_0ff0, 0x0000_0400],
            test: [0; 8],
        }
    }

    /// The state an 8086 has: no protection at all, and segment bases that are
    /// always the selector shifted by four.
    #[must_use]
    pub fn reset_8086() -> Sys {
        let mut sys = Sys::reset();
        sys.segs = [SegReg::real_data(0); seg::COUNT];
        sys.segs[seg::CS as usize] = SegReg::real_code(0xffff);
        sys.dr = [0; 8];
        sys
    }

    /// Whether protected mode is on.
    #[inline]
    #[must_use]
    pub const fn protected(&self) -> bool {
        self.cr0 & cr0::PE != 0
    }

    /// Whether paging is on. Paging without protection is impossible: the
    /// processor refuses to set `PG` while `PE` is clear.
    #[inline]
    #[must_use]
    pub const fn paging(&self) -> bool {
        self.cr0 & cr0::PG != 0
    }

    /// Whether long mode is **active** — `EFER.LMA`, which the processor sets
    /// when paging is enabled with `EFER.LME`.
    ///
    /// True in *both* of long mode's submodes: this says the four-level walk
    /// is in force and the 64-bit interrupt and task structures are, not that
    /// the current code segment is 64-bit. [`Sys::sixty_four`] says that.
    #[inline]
    #[must_use]
    pub const fn long_mode(&self) -> bool {
        self.efer & efer::LMA != 0
    }

    /// Whether the processor is executing in **64-bit mode** rather than
    /// compatibility mode.
    ///
    /// Long mode active *and* the current code segment's `L` bit set. The two
    /// are genuinely different processors from software's point of view — one
    /// has sixteen 64-bit registers and no segmentation, the other runs a
    /// 32-bit binary unchanged — and both are reached through the same page
    /// tables, which is the point of the design.
    ///
    /// *AMD64 Architecture Programmer's Manual* volume 2 §1.3.
    #[inline]
    #[must_use]
    pub const fn sixty_four(&self) -> bool {
        self.long_mode() && self.seg(seg::CS).long()
    }

    /// Which paging scheme is in force.
    ///
    /// The three inputs do not compose the way their names suggest: `PG`
    /// alone is the two-level walk, `PG` with `PAE` is the three-level one,
    /// and `PG` with `PAE` **and** `LMA` is the four-level one — while `PAE`
    /// without `PG` is no paging at all, however set the bit is.
    #[inline]
    #[must_use]
    pub const fn paging_mode(&self, features: super::Features) -> paging::Mode {
        if self.cr0 & cr0::PG == 0 {
            return paging::Mode::Off;
        }
        if !features.pae || self.cr4 & cr4::PAE == 0 {
            return paging::Mode::Legacy;
        }
        if self.long_mode() {
            paging::Mode::Ia32e
        } else {
            paging::Mode::Pae
        }
    }

    /// Everything a page-table walk needs, gathered in one value.
    #[must_use]
    pub fn tables(&self, features: super::Features) -> paging::Tables {
        paging::Tables {
            mode: self.paging_mode(features),
            cr3: self.cr3,
            pse: features.pse && self.cr4 & cr4::PSE != 0,
            nxe: features.nx && self.efer & efer::NXE != 0,
            wp: features.extras_486 && self.cr0 & cr0::WP != 0,
        }
    }

    /// A segment register.
    #[inline]
    #[must_use]
    pub const fn seg(&self, index: u8) -> SegReg {
        self.segs[(index as usize) % seg::COUNT]
    }

    /// A mutable segment register.
    #[inline]
    pub const fn seg_mut(&mut self, index: u8) -> &mut SegReg {
        &mut self.segs[(index as usize) % seg::COUNT]
    }
}

impl Default for Sys {
    fn default() -> Self {
        Sys::reset()
    }
}

// ---------------------------------------------------------------------------
// The task state segment
// ---------------------------------------------------------------------------

/// Byte offsets within a 32-bit task state segment.
///
/// Only the fields the architecture itself reads or writes are named; the rest
/// of the 104-byte minimum is the general register image, which is walked by
/// offset.
pub mod tss32 {
    /// The selector of the task that called this one.
    pub const BACK_LINK: u64 = 0x00;
    /// The privilege-0 stack pointer.
    pub const ESP0: u64 = 0x04;
    /// The privilege-0 stack selector.
    pub const SS0: u64 = 0x08;
    /// The page directory base for this task.
    pub const CR3: u64 = 0x1c;
    /// The saved instruction pointer.
    pub const EIP: u64 = 0x20;
    /// The saved flags.
    pub const EFLAGS: u64 = 0x24;
    /// The first of the eight saved general registers, in ModRM order.
    pub const EAX: u64 = 0x28;
    /// The first of the six saved selectors: `ES`, `CS`, `SS`, `DS`, `FS`,
    /// `GS` — which is *not* the ModRM segment order, and getting it wrong
    /// swaps the code and stack segments of every switched-to task.
    pub const ES: u64 = 0x48;
    /// The task's local descriptor table selector.
    pub const LDT: u64 = 0x60;
    /// The debug trap flag, bit 0 of the word at 0x64.
    pub const TRAP: u64 = 0x64;
    /// The offset of the I/O permission bitmap, in the halfword at 0x66.
    pub const IOMAP_BASE: u64 = 0x66;
    /// The smallest legal limit for a 32-bit TSS: the fixed area is 104 bytes
    /// and the limit is inclusive.
    pub const MIN_LIMIT: u64 = 0x67;
}

/// Where the `n`th privilege level's stack pointer and selector live in a
/// 32-bit task state segment.
///
/// `ESP0` at 4 and `SS0` at 8, then eight bytes per level.
#[must_use]
pub const fn tss32_stack(level: u8) -> (u64, u64) {
    let base = 4 + (level as u64 & 3) * 8;
    (base, base + 4)
}

/// The order the six selectors appear in a task state segment, as segment
/// register numbers.
///
/// `ES CS SS DS FS GS` — the numbering the TSS uses, which differs from the
/// ModRM numbering only in that `CS` and `ES` are not swapped. Written out so
/// the difference is visible rather than assumed.
pub const TSS_SEG_ORDER: [u8; 6] = [seg::ES, seg::CS, seg::SS, seg::DS, seg::FS, seg::GS];

// ---------------------------------------------------------------------------
// The interpreter's half: everything that touches the bus
// ---------------------------------------------------------------------------

use super::exec::{Ex, Exec, Fault, VEC_GP, VEC_NP, VEC_SS, VEC_TS, VEC_UD};
use super::flags;
use super::isa::{Fields, Op};

/// Which way a task switch was entered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SwitchKind {
    /// An `IRET` following the back link out of a nested task.
    Return,
}

impl Exec<'_> {
    /// Refuse an instruction that only exists in protected mode.
    ///
    /// `SLDT`, `STR`, `LLDT`, `LTR`, `LAR`, `LSL`, `VERR`, `VERW` and `ARPL`
    /// are all `#UD` in real mode — not no-ops, and not silently useful.
    pub(super) fn require_protected(&self) -> Ex<()> {
        if self.protected() {
            Ok(())
        } else {
            Err(Fault::bare(VEC_UD))
        }
    }

    /// Refuse an instruction a program at this privilege level may not run.
    pub(super) fn require_ring0(&self) -> Ex<()> {
        if self.protected() && self.cpl() != 0 {
            Err(Fault::gp(0))
        } else {
            Ok(())
        }
    }

    /// Which vector a fault against this segment register raises.
    ///
    /// Anything to do with the stack segment is `#SS`, everything else is
    /// `#GP`. The distinction is not cosmetic: an operating system's `#SS`
    /// handler runs on a known-good stack precisely because the fault it
    /// handles says the current one is unusable.
    const fn seg_fault_vector(sr: u8) -> u8 {
        if sr == seg::SS { VEC_SS } else { VEC_GP }
    }

    /// Turn a `segment:offset` pair into a linear address, checking the
    /// segment's limit and its access rights on the way.
    pub(super) fn seg_linear(&mut self, sr: u8, offset: u64, size: u64, write: bool) -> Ex<u64> {
        let s = self.state.sys.seg(sr);
        let vector = Self::seg_fault_vector(sr);
        if self.sixty_four() {
            // 64-bit mode has no segmentation worth the name: `CS`, `DS`,
            // `ES` and `SS` are treated as having a base of zero and no limit,
            // and their present and type bits are not consulted either. `FS`
            // and `GS` survive, with a base that comes from an MSR rather than
            // from a descriptor — which is why they are the two every 64-bit
            // ABI puts thread-local storage behind.
            //
            // What replaces the limit check is the canonical check: the
            // address must be a sign-extension of bit 47, and it is `#GP(0)`
            // — or `#SS(0)` through the stack — if it is not. *Intel SDM*
            // volume 3 §3.4.4 and volume 1 §3.3.7.1.
            let base = match sr {
                seg::FS => self.state.sys.fs_base,
                seg::GS => self.state.sys.gs_base,
                _ => 0,
            };
            let lin = base.wrapping_add(offset);
            if !canonical(lin) {
                return Err(Fault::coded(vector, 0));
            }
            return Ok(lin);
        }
        if self.protected() {
            // A null selector leaves the register unusable rather than
            // pointing at address zero, and using it is a fault — which is
            // what makes clearing `DS` on the way to user mode a safety
            // measure rather than a formality.
            if !s.present() {
                return Err(Fault::coded(vector, 0));
            }
            if write && !s.writable() {
                return Err(Fault::coded(vector, 0));
            }
            if !write && !s.readable() {
                return Err(Fault::coded(vector, 0));
            }
        }
        if !s.in_bounds(offset, size) {
            return Err(Fault::coded(vector, 0));
        }
        Ok(s.base.wrapping_add(offset))
    }

    // -- Descriptor tables ---------------------------------------------

    /// Read the eight bytes a selector names.
    ///
    /// The error code of every fault this raises is the selector with its
    /// bottom two bits cleared, which is what a handler needs to identify the
    /// descriptor without the caller's requested privilege level in the way.
    pub(super) fn descriptor(&mut self, selector: u16, vector: u8) -> Ex<RawDesc> {
        let sel = Selector(selector);
        let table = if sel.is_ldt() {
            let ldtr = self.state.sys.ldtr;
            if !ldtr.present() {
                return Err(Fault::coded(vector, u32::from(selector & 0xfffc)));
            }
            TableReg {
                base: ldtr.base,
                limit: ldtr.limit,
            }
        } else {
            self.state.sys.gdtr
        };
        if sel.offset() + 7 > table.limit {
            return Err(Fault::coded(vector, u32::from(selector & 0xfffc)));
        }
        let addr = table.base.wrapping_add(u64::from(sel.offset()));
        let low = self.sys_read32(addr)?;
        let high = self.sys_read32(addr.wrapping_add(4))?;
        // In long mode a **system** descriptor is sixteen bytes: the extra
        // doubleword carries the top half of the base. A code or data
        // descriptor stays eight, because it has no base to widen. Which kind
        // this is is the `S` bit, so the second half can only be read once the
        // first has been.
        let system = high & ar::S == 0;
        let upper = if self.state.sys.long_mode() && system {
            if sel.offset() + 15 > table.limit {
                return Err(Fault::coded(vector, u32::from(selector & 0xfffc)));
            }
            self.sys_read32(addr.wrapping_add(8))?
        } else {
            0
        };
        Ok(RawDesc { low, high, upper })
    }

    /// Set a descriptor's accessed bit, as loading a selector does.
    ///
    /// Software can see this: a kernel that watches the bit knows which
    /// segments a task has touched. Skipping the write when the bit is already
    /// set is what hardware does and keeps a read-only descriptor table from
    /// being written on every load.
    fn mark_accessed(&mut self, selector: u16, desc: RawDesc) -> Ex<()> {
        if desc.high & ar::ACCESSED != 0 {
            return Ok(());
        }
        let sel = Selector(selector);
        let table = if sel.is_ldt() {
            let ldtr = self.state.sys.ldtr;
            TableReg {
                base: ldtr.base,
                limit: ldtr.limit,
            }
        } else {
            self.state.sys.gdtr
        };
        let addr = table
            .base
            .wrapping_add(u64::from(sel.offset()))
            .wrapping_add(4);
        self.sys_write32(addr, desc.high | ar::ACCESSED)
    }

    /// Rewrite a system descriptor's type field, as marking a task state
    /// segment busy or available does.
    fn set_sys_type(&mut self, selector: u16, kind: u8) -> Ex<()> {
        let sel = Selector(selector);
        let table = self.state.sys.gdtr;
        let addr = table
            .base
            .wrapping_add(u64::from(sel.offset()))
            .wrapping_add(4);
        let high = self.sys_read32(addr)?;
        let updated = (high & !0x0000_0f00) | (u32::from(kind) << 8);
        self.sys_write32(addr, updated)
    }

    // -- Loading a segment register ------------------------------------

    /// Load one of `DS`, `ES`, `FS`, `GS` or `SS`.
    ///
    /// `CS` never comes through here: the only way to change the code segment
    /// is a control transfer, which has its own checks and its own faults.
    pub(super) fn load_segment(&mut self, index: u8, selector: u16) -> Ex<()> {
        if index >= seg::COUNT as u8 {
            return Err(Fault::bare(VEC_UD));
        }
        if !self.protected() {
            // Real mode recomputes the base and **leaves the limit and the
            // access rights alone**. That is not a shortcut: it is what makes
            // "unreal mode" work on real silicon, and firmware relies on it to
            // reach memory above 1 MiB without leaving real mode.
            let cached = self.state.sys.seg(index);
            let entry = self.state.sys.seg_mut(index);
            entry.selector = selector;
            entry.base = u64::from(selector) << 4;
            if !cached.present() {
                // Nothing valid was ever loaded here; give it the real-mode
                // defaults rather than a zero limit that would fault at once.
                entry.limit = 0xffff;
                entry.ar = ar::REAL_DATA;
            }
            self.state.regs.set_segment(index, selector);
            if index == seg::SS {
                self.state.int_shadow = true;
            }
            return Ok(());
        }

        let sel = Selector(selector);
        let cpl = self.cpl();
        if index == seg::SS {
            // The stack is the one segment that may not be null, must be
            // writable, and must be at exactly the current privilege level.
            if sel.is_null() {
                return Err(Fault::gp(0));
            }
            let desc = self.descriptor(selector, VEC_GP)?;
            if !desc.is_app()
                || desc.high & ar::CODE != 0
                || desc.high & ar::RW == 0
                || sel.rpl() != cpl
                || desc.dpl() != cpl
            {
                return Err(Fault::gp(u32::from(selector & 0xfffc)));
            }
            if !desc.present() {
                return Err(Fault::coded(VEC_SS, u32::from(selector & 0xfffc)));
            }
            self.mark_accessed(selector, desc)?;
            *self.state.sys.seg_mut(index) = desc.to_seg(selector);
            self.state.regs.ss = selector;
            // Every write to `SS` inhibits interrupts for one instruction,
            // whichever encoding did it, so that `mov ss,ax` / `mov esp,ebx`
            // cannot be split by an interrupt landing on a half-changed stack.
            self.state.int_shadow = true;
            return Ok(());
        }

        if sel.is_null() {
            // Legal, and the register becomes unusable rather than pointing
            // anywhere. Using it afterwards is `#GP(0)`.
            *self.state.sys.seg_mut(index) = SegReg::null();
            self.state.regs.set_segment(index, selector);
            return Ok(());
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        let readable_code = desc.is_app() && desc.high & ar::CODE != 0 && desc.high & ar::RW != 0;
        let data = desc.is_app() && desc.high & ar::CODE == 0;
        if !(readable_code || data) {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        // A conforming code segment is reachable from anywhere; everything
        // else has to be at least as privileged as both the caller and the
        // selector's own request.
        let conforming = desc.high & (ar::CODE | ar::DC) == (ar::CODE | ar::DC);
        if !conforming {
            let effective = if cpl > sel.rpl() { cpl } else { sel.rpl() };
            if effective > desc.dpl() {
                return Err(Fault::gp(u32::from(selector & 0xfffc)));
            }
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }
        self.mark_accessed(selector, desc)?;
        *self.state.sys.seg_mut(index) = desc.to_seg(selector);
        self.state.regs.set_segment(index, selector);
        Ok(())
    }

    /// Load `CS` from an already-validated descriptor, at a given privilege.
    fn commit_cs(&mut self, selector: u16, desc: RawDesc, cpl: u8) {
        let selector = Selector(selector).with_rpl(cpl).0;
        *self.state.sys.seg_mut(seg::CS) = desc.to_seg(selector);
        self.state.regs.cs = selector;
        self.state.queue.flush();
    }

    // -- Far control transfer ------------------------------------------

    /// A far jump or call: real mode, a code segment, a call gate, a task
    /// gate, or a task state segment.
    pub(super) fn far_transfer(
        &mut self,
        selector: u16,
        offset: u64,
        is_call: bool,
        opsize: u8,
    ) -> Ex<()> {
        if !self.protected() {
            if is_call {
                let cs = self.state.regs.cs;
                let ip = self.state.regs.rip;
                self.push(u64::from(cs), opsize)?;
                self.push(ip, opsize)?;
            }
            let entry = self.state.sys.seg_mut(seg::CS);
            entry.selector = selector;
            entry.base = u64::from(selector) << 4;
            if !entry.present() {
                entry.limit = 0xffff;
                entry.ar = ar::REAL_CODE;
            }
            self.state.regs.cs = selector;
            self.state.regs.rip = if opsize == 2 { offset & 0xffff } else { offset };
            self.state.queue.flush();
            return Ok(());
        }

        let sel = Selector(selector);
        if sel.is_null() {
            return Err(Fault::gp(0));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        let cpl = self.cpl();

        if desc.is_app() {
            if desc.high & ar::CODE == 0 {
                return Err(Fault::gp(u32::from(selector & 0xfffc)));
            }
            let conforming = desc.high & ar::DC != 0;
            let ok = if conforming {
                desc.dpl() <= cpl
            } else {
                desc.dpl() == cpl && sel.rpl() <= cpl
            };
            if !ok {
                return Err(Fault::gp(u32::from(selector & 0xfffc)));
            }
            if !desc.present() {
                return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
            }
            let target = desc.to_seg(selector);
            // `L = 1` with `D = 1` is an invalid combination rather than a
            // reserved one: there is no 64-bit code segment with 32-bit
            // defaults, and hardware faults rather than choosing (*AMD64
            // Architecture Programmer's Manual* volume 2 §4.8.1).
            if self.state.sys.long_mode() && target.long() && target.big() {
                return Err(Fault::gp(u32::from(selector & 0xfffc)));
            }
            // A 64-bit target has no limit to check, and the offset must be
            // canonical instead.
            if target.long() && self.state.sys.long_mode() {
                if !canonical(offset) {
                    return Err(Fault::gp(0));
                }
            } else if !target.in_bounds(offset, 1) {
                return Err(Fault::gp(0));
            }
            if is_call {
                let cs = self.state.regs.cs;
                let ip = self.state.regs.rip;
                self.push(u64::from(cs), opsize)?;
                self.push(ip, opsize)?;
            }
            self.mark_accessed(selector, desc)?;
            self.commit_cs(selector, desc, cpl);
            self.state.regs.rip = offset;
            return Ok(());
        }

        match desc.kind() {
            sys_type::CALL_GATE16 | sys_type::CALL_GATE32 => {
                self.call_gate(selector, desc, is_call)
            }
            sys_type::TASK_GATE => {
                if desc.dpl() < cpl || desc.dpl() < sel.rpl() {
                    return Err(Fault::gp(u32::from(selector & 0xfffc)));
                }
                if !desc.present() {
                    return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
                }
                self.switch_task(desc.gate_selector(), is_call, None)
            }
            sys_type::TSS32_AVAIL | sys_type::TSS16_AVAIL => {
                if desc.dpl() < cpl || desc.dpl() < sel.rpl() {
                    return Err(Fault::gp(u32::from(selector & 0xfffc)));
                }
                self.switch_task(selector, is_call, None)
            }
            _ => Err(Fault::gp(u32::from(selector & 0xfffc))),
        }
    }

    /// A call or jump through a call gate, which is how a program at one
    /// privilege level enters code at a higher one.
    fn call_gate(&mut self, gate_sel: u16, gate: RawDesc, is_call: bool) -> Ex<()> {
        let cpl = self.cpl();
        let sel = Selector(gate_sel);
        if gate.dpl() < cpl || gate.dpl() < sel.rpl() {
            return Err(Fault::gp(u32::from(gate_sel & 0xfffc)));
        }
        if !gate.present() {
            return Err(Fault::coded(VEC_NP, u32::from(gate_sel & 0xfffc)));
        }
        let gate32 = gate.kind() == sys_type::CALL_GATE32;
        let size = if gate32 { 4u8 } else { 2u8 };
        let target_sel = gate.gate_selector();
        if Selector(target_sel).is_null() {
            return Err(Fault::gp(0));
        }
        let target = self.descriptor(target_sel, VEC_GP)?;
        if !target.is_app() || target.high & ar::CODE == 0 || target.dpl() > cpl {
            return Err(Fault::gp(u32::from(target_sel & 0xfffc)));
        }
        if !target.present() {
            return Err(Fault::coded(VEC_NP, u32::from(target_sel & 0xfffc)));
        }
        let conforming = target.high & ar::DC != 0;
        // A long-mode call gate is sixteen bytes with a 64-bit offset, and it
        // reuses the 32-bit gate's type — there is no separate encoding,
        // because there is no 32-bit call gate in long mode to collide with.
        let long = self.state.sys.long_mode();
        let offset = if long {
            gate.gate_offset64()
        } else if gate32 {
            u64::from(gate.gate_offset())
        } else {
            u64::from(gate.gate_offset() & 0xffff)
        };
        let size = if long { 8u8 } else { size };

        if !conforming && target.dpl() < cpl && is_call {
            // The privilege really changes, so the stack does too: the new one
            // comes out of the task state segment, and the caller's is saved
            // on it along with the parameters the gate says to copy.
            let new_cpl = target.dpl();
            let (ss_sel, new_sp) = self.tss_stack(new_cpl)?;
            let ss_desc = self.validate_stack(ss_sel, new_cpl)?;
            let argc = gate.gate_argc();
            let mut args = [0u64; 31];
            for (i, slot) in args.iter_mut().enumerate().take(argc as usize) {
                let off = self.sp().wrapping_add(u64::from(size) * (i as u64));
                *slot = self.read_mem(seg::SS, off, size)?;
            }
            let old_ss = self.state.regs.ss;
            let old_sp = self.sp();
            let old_cs = self.state.regs.cs;
            let old_ip = self.state.regs.rip;

            self.commit_cs(target_sel, target, new_cpl);
            *self.state.sys.seg_mut(seg::SS) = ss_desc.to_seg(ss_sel);
            self.state.regs.ss = ss_sel;
            self.set_sp(new_sp);

            self.push(u64::from(old_ss), size)?;
            self.push(old_sp, size)?;
            for i in (0..argc as usize).rev() {
                self.push(args[i], size)?;
            }
            self.push(u64::from(old_cs), size)?;
            self.push(old_ip, size)?;
            self.state.regs.rip = offset;
            return Ok(());
        }

        if is_call {
            let cs = self.state.regs.cs;
            let ip = self.state.regs.rip;
            self.push(u64::from(cs), size)?;
            self.push(ip, size)?;
        }
        let new_cpl = if conforming {
            cpl
        } else if target.dpl() > cpl {
            target.dpl()
        } else {
            cpl
        };
        self.commit_cs(target_sel, target, new_cpl);
        self.state.regs.rip = offset;
        Ok(())
    }

    /// The stack pointer and selector a given privilege level uses, out of the
    /// current task state segment.
    fn tss_stack(&mut self, level: u8) -> Ex<(u16, u64)> {
        let tss = self.state.sys.task;
        if !tss.present() {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        let wide =
            tss.sys_type() == sys_type::TSS32_AVAIL || tss.sys_type() == sys_type::TSS32_BUSY;
        let (esp_off, ss_off) = if wide {
            tss32_stack(level)
        } else {
            // A 286 task state segment packs the same six values into
            // halfwords starting at offset 2.
            let base = 2 + (u64::from(level) & 3) * 4;
            (base, base + 2)
        };
        if ss_off + 3 > u64::from(tss.limit) {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        let sp = if wide {
            u64::from(self.sys_read32(tss.base.wrapping_add(u64::from(esp_off)))?)
        } else {
            u64::from(self.sys_read16(tss.base.wrapping_add(u64::from(esp_off)))?)
        };
        let ss = self.sys_read16(tss.base.wrapping_add(u64::from(ss_off)))? as u16;
        Ok((ss, sp))
    }

    /// `RSP0`, `RSP1` or `RSP2` out of the 64-bit task state segment.
    ///
    /// The 64-bit task state segment is not a task's saved state at all — long
    /// mode dropped hardware task switching — it is a table of seven stack
    /// pointers and three privilege-level stacks, and nothing else. `RSP0` is
    /// at offset 4, and each is eight bytes (*Intel SDM* volume 3 §7.7).
    fn tss_rsp(&mut self, level: u8) -> Ex<u64> {
        let tss = self.state.sys.task;
        if !tss.present() {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        let offset = 4 + u64::from(level & 3) * 8;
        if offset + 7 > u64::from(tss.limit) {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        self.sys_read(tss.base.wrapping_add(offset), 8)
    }

    /// One of the seven interrupt-stack-table pointers.
    ///
    /// `IST1` is at offset 36, and each is eight bytes. The mechanism exists
    /// so that a handler for a fault that may have destroyed the kernel stack
    /// — `#DF`, `#NMI`, `#MC` — lands on a stack chosen by the *gate* rather
    /// than by the privilege change, which in those cases has not happened.
    fn tss_ist(&mut self, index: u8) -> Ex<u64> {
        let tss = self.state.sys.task;
        if !tss.present() || index == 0 || index > 7 {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        let offset = 36 - 8 + u64::from(index) * 8;
        if offset + 7 > u64::from(tss.limit) {
            return Err(Fault::coded(VEC_TS, u32::from(tss.selector & 0xfffc)));
        }
        self.sys_read(tss.base.wrapping_add(offset), 8)
    }

    /// Check that a selector names a stack usable at a given privilege level.
    fn validate_stack(&mut self, selector: u16, level: u8) -> Ex<RawDesc> {
        let sel = Selector(selector);
        if sel.is_null() || sel.rpl() != level {
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_TS)?;
        if !desc.is_app()
            || desc.high & ar::CODE != 0
            || desc.high & ar::RW == 0
            || desc.dpl() != level
        {
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_SS, u32::from(selector & 0xfffc)));
        }
        Ok(desc)
    }

    /// A far return.
    pub(super) fn return_far(&mut self, opsize: u8, extra: u64) -> Ex<()> {
        let ip = self.pop(opsize)?;
        let selector = self.pop(opsize)? as u16;
        if !self.protected() {
            let entry = self.state.sys.seg_mut(seg::CS);
            entry.selector = selector;
            entry.base = u64::from(selector) << 4;
            self.state.regs.cs = selector;
            self.state.regs.rip = if opsize == 2 { ip & 0xffff } else { ip };
            let sp = self.sp().wrapping_add(extra);
            self.set_sp(sp);
            self.state.queue.flush();
            return Ok(());
        }
        let sp = self.sp().wrapping_add(extra);
        self.set_sp(sp);

        let sel = Selector(selector);
        if sel.is_null() {
            return Err(Fault::gp(0));
        }
        let cpl = self.cpl();
        if sel.rpl() < cpl {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if !desc.is_app() || desc.high & ar::CODE == 0 {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let conforming = desc.high & ar::DC != 0;
        let ok = if conforming {
            desc.dpl() <= sel.rpl()
        } else {
            desc.dpl() == sel.rpl()
        };
        if !ok {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }

        if sel.rpl() > cpl {
            // Returning outward: the caller's stack comes back off this one.
            let new_sp = self.pop(opsize)?;
            let new_ss = self.pop(opsize)? as u16;
            let ss_desc = self.validate_return_stack(new_ss, sel.rpl())?;
            self.commit_cs(selector, desc, sel.rpl());
            *self.state.sys.seg_mut(seg::SS) = ss_desc.to_seg(new_ss);
            self.state.regs.ss = new_ss;
            self.set_sp(new_sp.wrapping_add(extra));
            self.state.regs.rip = if opsize == 2 { ip & 0xffff } else { ip };
            self.drop_privileged_segments(sel.rpl());
            return Ok(());
        }
        self.commit_cs(selector, desc, cpl);
        self.state.regs.rip = if opsize == 2 { ip & 0xffff } else { ip };
        Ok(())
    }

    /// The stack check a far return makes, which faults as `#GP`/`#SS` rather
    /// than `#TS` because no task state segment was involved.
    fn validate_return_stack(&mut self, selector: u16, level: u8) -> Ex<RawDesc> {
        let sel = Selector(selector);
        if sel.is_null() || sel.rpl() != level {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if !desc.is_app()
            || desc.high & ar::CODE != 0
            || desc.high & ar::RW == 0
            || desc.dpl() != level
        {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_SS, u32::from(selector & 0xfffc)));
        }
        Ok(desc)
    }

    /// After returning to a less privileged level, discard any data segment
    /// the new level may not use.
    ///
    /// Without this a program could keep a privileged selector across a return
    /// and read kernel memory with it — the "stale selector" hole the
    /// architecture closes here rather than at the point of use.
    fn drop_privileged_segments(&mut self, cpl: u8) {
        for index in [seg::ES, seg::DS, seg::FS, seg::GS] {
            let s = self.state.sys.seg(index);
            if !s.present() {
                continue;
            }
            let conforming = s.is_code() && s.ar & ar::DC != 0;
            if !conforming && s.dpl() < cpl {
                *self.state.sys.seg_mut(index) = SegReg::null();
                self.state.regs.set_segment(index, 0);
            }
        }
    }

    /// `IRET`, which is a far return that also restores the flags — or, with
    /// `NT` set, a task switch back to the caller.
    pub(super) fn iret(&mut self, opsize: u8) -> Ex<()> {
        if !self.protected() {
            let ip = self.pop(opsize)?;
            let cs = self.pop(opsize)? as u16;
            let fl = self.pop(opsize)?;
            let entry = self.state.sys.seg_mut(seg::CS);
            entry.selector = cs;
            entry.base = u64::from(cs) << 4;
            self.state.regs.cs = cs;
            self.state.regs.rip = if opsize == 2 { ip & 0xffff } else { ip };
            let kept: u32 = if opsize == 2 { 0xffff_0000 } else { 0 };
            let old = self.state.regs.eflags;
            self.set_flags((fl as u32 & !kept) | (old & kept));
            self.state.queue.flush();
            return Ok(());
        }
        if self.flag(flags::NT) {
            // A nested task returns to whoever called it, named by the back
            // link at offset zero of the current task state segment.
            let tss = self.state.sys.task;
            let back = self.sys_read16(tss.base)? as u16;
            return self.switch_task(back, false, Some(SwitchKind::Return));
        }

        let ip = self.pop(opsize)?;
        let selector = self.pop(opsize)? as u16;
        let fl = self.pop(opsize)?;
        let cpl = self.cpl();
        let sel = Selector(selector);
        if sel.is_null() {
            return Err(Fault::gp(0));
        }
        if sel.rpl() < cpl {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if !desc.is_app() || desc.high & ar::CODE == 0 {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let conforming = desc.high & ar::DC != 0;
        let ok = if conforming {
            desc.dpl() <= sel.rpl()
        } else {
            desc.dpl() == sel.rpl()
        };
        if !ok {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }

        // In long mode `IRET` **always** pops `SS:RSP`, because the interrupt
        // always pushed it. That is the difference that breaks a 32-bit `IRET`
        // reused unchanged: the frame is five words, not three, and popping
        // three leaves the stack two deep in the wrong place.
        let long = self.state.sys.long_mode();
        let outward = sel.rpl() > cpl;
        if long || outward {
            let new_sp = self.pop(opsize)?;
            let new_ss = self.pop(opsize)? as u16;
            let new_cpl = sel.rpl();
            self.commit_cs(selector, desc, new_cpl);
            if long && Selector(new_ss).is_null() {
                // Returning to 64-bit code with a null stack selector is
                // legal: `SS` has no base or limit there, and the selector is
                // kept only for its privilege level.
                *self.state.sys.seg_mut(seg::SS) = SegReg {
                    selector: new_ss,
                    base: 0,
                    limit: 0,
                    ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED,
                };
            } else {
                let ss_desc = self.validate_return_stack(new_ss, new_cpl)?;
                *self.state.sys.seg_mut(seg::SS) = ss_desc.to_seg(new_ss);
            }
            self.state.regs.ss = new_ss;
            self.set_sp(new_sp);
        } else {
            self.commit_cs(selector, desc, cpl);
        }
        self.state.regs.rip = match opsize {
            2 => ip & 0xffff,
            4 => ip & 0xffff_ffff,
            _ => ip,
        };
        self.restore_flags(fl, cpl, opsize);
        if outward {
            self.drop_privileged_segments(sel.rpl());
        }
        Ok(())
    }

    /// Put the flags back after an `IRET`, keeping the fields this privilege
    /// level may not change.
    fn restore_flags(&mut self, value: u64, cpl: u8, opsize: u8) {
        let old = self.state.regs.eflags;
        let value = value as u32;
        let mut keep = flags::VM;
        if cpl > self.state.regs.iopl() {
            keep |= flags::IF;
        }
        if cpl > 0 {
            keep |= flags::IOPL;
        }
        if opsize == 2 {
            keep |= 0xffff_0000;
        }
        // `RF` is cleared by `IRET` rather than restored, which is what stops
        // a debug fault from repeating forever.
        self.set_flags(((value & !keep) | (old & keep)) & !flags::RF);
    }

    // -- Interrupts and gates ------------------------------------------

    /// The privilege check `INT n` makes and an exception does not.
    ///
    /// A user program may only invoke a gate whose descriptor privilege level
    /// is at least its own — which is how an operating system exposes `INT 80`
    /// to user space while keeping the rest of the table to itself.
    pub(super) fn check_software_gate(&mut self, vector: u8) -> Ex<()> {
        let stride = if self.state.sys.long_mode() { 16 } else { 8 };
        let offset = u32::from(vector) * stride;
        let idtr = self.state.sys.idtr;
        if offset + stride - 1 > idtr.limit {
            return Err(Fault::gp(u32::from(vector) * 8 + 2));
        }
        let high = self.sys_read32(idtr.base.wrapping_add(u64::from(offset)).wrapping_add(4))?;
        let dpl = ((high & ar::DPL) >> ar::DPL_SHIFT) as u8;
        if dpl < self.cpl() {
            return Err(Fault::gp(u32::from(vector) * 8 + 2));
        }
        Ok(())
    }

    /// Take an interrupt or exception.
    ///
    /// Three different sequences hide behind one name, and which one runs is
    /// the difference between a working machine and a mystery: the 8086's, a
    /// 386's in real mode, and a 386's in protected mode.
    pub(super) fn take_interrupt(&mut self, vector: u8, error: Option<u32>) -> Ex<()> {
        self.state.halted = false;
        if self.legacy() {
            return self.legacy_interrupt(vector);
        }
        if self.protected() {
            return self.protected_interrupt(vector, error);
        }
        self.real_interrupt(vector)
    }

    /// The 8086's interrupt sequence.
    ///
    /// The order is the one the hardware traces show, and it is not the order
    /// the manual's prose suggests: **the vector is read first**, before
    /// anything is pushed. Then flags, then `CS`, then the return `IP` — and
    /// `IF` and `TF` are cleared between the flags push and the `CS` push, so
    /// the saved flags still have them.
    fn legacy_interrupt(&mut self, vector: u8) -> Ex<()> {
        let base = u32::from(vector) << 2;
        let target_ip = self.read_ivt(base);
        let target_cs = self.read_ivt(base.wrapping_add(2));

        let saved = u64::from(self.state.regs.eflags);
        self.push(saved, 2)?;
        self.set_flag(flags::IF | flags::TF, false);
        let cs = self.state.regs.cs;
        let ip = self.state.regs.rip & 0xffff;
        self.push(u64::from(cs), 2)?;
        self.push(ip, 2)?;

        self.state.regs.cs = target_cs as u16;
        self.state.regs.rip = target_ip;
        let entry = self.state.sys.seg_mut(seg::CS);
        entry.selector = target_cs as u16;
        entry.base = u64::from(target_cs & 0xffff) << 4;
        self.state.queue.flush();
        Ok(())
    }

    /// One halfword of the 8086's vector table.
    ///
    /// Read through segment zero with the 8086's own adder, so the 1 MiB wrap
    /// applies to it exactly as it does to every other access.
    fn read_ivt(&mut self, offset: u32) -> u64 {
        let low = super::linear(0, offset as u16);
        let high = super::linear(0, (offset as u16).wrapping_add(1));
        let lo = self.phys_read(u64::from(low), 1);
        let hi = self.phys_read(u64::from(high), 1);
        lo | (hi << 8)
    }

    /// A 386's real-mode interrupt sequence.
    ///
    /// Almost the 8086's, with one difference that matters: the vector table's
    /// position comes from `IDTR`, which `LIDT` can move. A real-mode 386 with
    /// a relocated `IDTR` is how some protected-mode monitors keep their own
    /// vectors while running real-mode code.
    fn real_interrupt(&mut self, vector: u8) -> Ex<()> {
        let offset = u32::from(vector) * 4;
        let idtr = self.state.sys.idtr;
        if offset + 3 > idtr.limit {
            return Err(Fault::gp(u32::from(vector) * 8 + 2));
        }
        let entry_addr = idtr.base.wrapping_add(u64::from(offset));
        let target_ip = u64::from(self.sys_read16(entry_addr)?);
        let target_cs = self.sys_read16(entry_addr.wrapping_add(2))? as u16;

        let saved = u64::from(self.state.regs.eflags);
        let cs = self.state.regs.cs;
        let ip = self.state.regs.rip & 0xffff;
        self.push(saved, 2)?;
        self.push(u64::from(cs), 2)?;
        self.push(ip, 2)?;
        self.set_flag(flags::IF | flags::TF | flags::AC | flags::RF, false);

        self.state.regs.cs = target_cs;
        self.state.regs.rip = target_ip;
        let entry = self.state.sys.seg_mut(seg::CS);
        entry.selector = target_cs;
        entry.base = u64::from(target_cs) << 4;
        self.state.queue.flush();
        Ok(())
    }

    /// The protected-mode interrupt sequence: a gate out of the interrupt
    /// descriptor table, and possibly a stack switch to go with it.
    #[allow(clippy::too_many_lines)]
    fn protected_interrupt(&mut self, vector: u8, error: Option<u32>) -> Ex<()> {
        // Long mode doubled the interrupt descriptor table's entries, so the
        // *stride* changes with the mode. An emulator that keeps eight here
        // reads the second half of the previous gate and lands somewhere
        // plausible-looking, which is the worst way for this to fail.
        let long = self.state.sys.long_mode();
        let stride: u32 = if long { 16 } else { 8 };
        let index = u32::from(vector) * stride;
        let idtr = self.state.sys.idtr;
        // The error code of a fault *about the table itself* names the entry,
        // with bit 1 set to say the interrupt descriptor table rather than the
        // global one.
        let table_error = u32::from(vector) * 8 + 2;
        if index + stride - 1 > idtr.limit {
            return Err(Fault::gp(table_error));
        }
        let addr = idtr.base.wrapping_add(u64::from(index));
        let gate = RawDesc {
            low: self.sys_read32(addr)?,
            high: self.sys_read32(addr.wrapping_add(4))?,
            upper: if long {
                self.sys_read32(addr.wrapping_add(8))?
            } else {
                0
            },
        };
        if gate.is_app() {
            return Err(Fault::gp(table_error));
        }
        let kind = gate.kind();
        if kind == sys_type::TASK_GATE && !long {
            if !gate.present() {
                return Err(Fault::coded(VEC_NP, table_error));
            }
            self.switch_task(gate.gate_selector(), true, None)?;
            if let Some(code) = error {
                self.push(u64::from(code), 4)?;
            }
            return Ok(());
        }
        let gate32 = matches!(kind, sys_type::INT_GATE32 | sys_type::TRAP_GATE32);
        let gate16 = matches!(kind, sys_type::INT_GATE16 | sys_type::TRAP_GATE16);
        // In long mode the 32-bit gate types are reused for the 64-bit gates —
        // there is no separate encoding — and the 16-bit ones are invalid.
        if !gate32 && (!gate16 || long) {
            return Err(Fault::gp(table_error));
        }
        if !gate.present() {
            return Err(Fault::coded(VEC_NP, table_error));
        }
        let interrupt_gate = matches!(kind, sys_type::INT_GATE16 | sys_type::INT_GATE32);
        let size = if long {
            8u8
        } else if gate32 {
            4u8
        } else {
            2u8
        };

        let target_sel = gate.gate_selector();
        if Selector(target_sel).is_null() {
            return Err(Fault::gp(0));
        }
        let target = self.descriptor(target_sel, VEC_GP)?;
        let cpl = self.cpl();
        if !target.is_app() || target.high & ar::CODE == 0 || target.dpl() > cpl {
            return Err(Fault::gp(u32::from(target_sel & 0xfffc)));
        }
        if !target.present() {
            return Err(Fault::coded(VEC_NP, u32::from(target_sel & 0xfffc)));
        }
        let conforming = target.high & ar::DC != 0;
        let offset = if long {
            gate.gate_offset64()
        } else if gate32 {
            u64::from(gate.gate_offset())
        } else {
            u64::from(gate.gate_offset() & 0xffff)
        };

        let old_flags = u64::from(self.state.regs.eflags);
        let old_cs = self.state.regs.cs;
        let old_ip = self.state.regs.rip;

        // Long mode pushes `SS:RSP` **always**, even without a privilege
        // change, and it switches stacks whenever the gate names an interrupt
        // stack table entry. Both exist so that a handler never has to trust
        // the stack it was interrupted on — the 32-bit design, which only
        // switched on a ring change, could not protect a ring-0 fault.
        let ist = if long { gate.gate_ist() } else { 0 };
        let switching = (!conforming && target.dpl() < cpl) || ist != 0;
        if switching {
            let new_cpl = if conforming { cpl } else { target.dpl() };
            let (ss_sel, new_sp) = if ist != 0 {
                (0u16, self.tss_ist(ist)?)
            } else if long {
                // A 64-bit stack switch loads a *null* `SS` with the new
                // privilege level in its selector: there is no stack
                // descriptor to validate, only `RSP0` out of the 64-bit task
                // state segment.
                (u16::from(new_cpl), self.tss_rsp(new_cpl)?)
            } else {
                self.tss_stack(new_cpl)?
            };
            let old_ss = self.state.regs.ss;
            let old_sp = self.sp();

            // `CS` is committed before the pushes so that the writes are made
            // at the *new* privilege level: a page marked supervisor-only is
            // exactly where a ring-0 stack lives.
            self.commit_cs(target_sel, target, new_cpl);
            if long {
                *self.state.sys.seg_mut(seg::SS) = SegReg {
                    selector: ss_sel,
                    base: 0,
                    limit: 0,
                    ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED,
                };
            } else {
                let ss_desc = self.validate_stack(ss_sel, new_cpl)?;
                *self.state.sys.seg_mut(seg::SS) = ss_desc.to_seg(ss_sel);
            }
            self.state.regs.ss = ss_sel;
            self.set_sp(new_sp);

            self.push(u64::from(old_ss), size)?;
            self.push(old_sp, size)?;
            self.push(old_flags, size)?;
            self.push(u64::from(old_cs), size)?;
            self.push(old_ip, size)?;
        } else if long {
            let new_cpl = if conforming {
                cpl
            } else {
                target.dpl().max(cpl)
            };
            let old_ss = self.state.regs.ss;
            let old_sp = self.sp();
            self.commit_cs(target_sel, target, new_cpl);
            self.push(u64::from(old_ss), size)?;
            self.push(old_sp, size)?;
            self.push(old_flags, size)?;
            self.push(u64::from(old_cs), size)?;
            self.push(old_ip, size)?;
        } else {
            let new_cpl = if conforming {
                cpl
            } else if target.dpl() > cpl {
                target.dpl()
            } else {
                cpl
            };
            self.commit_cs(target_sel, target, new_cpl);
            self.push(old_flags, size)?;
            self.push(u64::from(old_cs), size)?;
            self.push(old_ip, size)?;
        }
        if let Some(code) = error {
            self.push(u64::from(code), size)?;
        }
        self.state.regs.rip = offset;
        // A trap gate leaves interrupts enabled — which is the whole
        // difference between the two, and why a system call goes through a
        // trap gate and a device interrupt through an interrupt gate.
        let mut clear = flags::TF | flags::NT | flags::VM | flags::RF;
        if interrupt_gate {
            clear |= flags::IF;
        }
        self.state.regs.eflags &= !clear;
        Ok(())
    }

    // -- Task switching -------------------------------------------------

    /// A task switch to the task state segment `selector` names.
    ///
    /// The three ways in differ only in what happens to the busy bit and the
    /// back link: a jump leaves no link, a call or an interrupt sets one and
    /// sets `NT`, and an `IRET` with `NT` set follows one back.
    #[allow(clippy::too_many_lines)]
    pub(super) fn switch_task(
        &mut self,
        selector: u16,
        nested: bool,
        kind: Option<SwitchKind>,
    ) -> Ex<()> {
        let returning = matches!(kind, Some(SwitchKind::Return));
        let sel = Selector(selector);
        if sel.is_null() || sel.is_ldt() {
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if desc.is_app() {
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }
        let wide = match desc.kind() {
            sys_type::TSS32_AVAIL => true,
            sys_type::TSS32_BUSY if returning => true,
            sys_type::TSS16_AVAIL => false,
            sys_type::TSS16_BUSY if returning => false,
            _ => return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc))),
        };
        if !wide {
            // A 286 task state segment holds no `EIP`, no 32-bit registers and
            // no `CR3`, so switching to one would silently truncate the task's
            // state. Refusing is the honest answer until it is implemented.
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }
        let new_tss = desc.to_seg(selector);
        if u64::from(new_tss.limit) < tss32::MIN_LIMIT {
            return Err(Fault::coded(VEC_TS, u32::from(selector & 0xfffc)));
        }

        // 1. Save the outgoing task, if there is one.
        let outgoing = self.state.sys.task;
        if outgoing.present() {
            self.save_task_state(outgoing)?;
            if returning {
                self.set_sys_type(outgoing.selector, sys_type::TSS32_AVAIL)?;
            }
        }

        // 2. Read the incoming task's saved state.
        let base = new_tss.base;
        let cr3 = self.sys_read32(base.wrapping_add(tss32::CR3))?;
        let eip = self.sys_read32(base.wrapping_add(tss32::EIP))?;
        let eflags = self.sys_read32(base.wrapping_add(tss32::EFLAGS))?;
        let mut gpr = [0u32; 8];
        for (i, slot) in gpr.iter_mut().enumerate() {
            *slot = self.sys_read32(base.wrapping_add(tss32::EAX + 4 * i as u64))?;
        }
        let mut selectors = [0u16; 6];
        for (i, slot) in selectors.iter_mut().enumerate() {
            *slot = self.sys_read16(base.wrapping_add(tss32::ES + 4 * i as u64))? as u16;
        }
        let ldt = self.sys_read16(base.wrapping_add(tss32::LDT))? as u16;

        if !returning && nested {
            // The incoming task points back at the one it interrupted.
            let previous = u32::from(self.state.sys.task.selector);
            self.sys_write32(base, previous)?;
        }
        if !returning {
            self.set_sys_type(selector, sys_type::TSS32_BUSY)?;
        }

        // 3. Commit.
        self.state.sys.task = SegReg {
            ar: (new_tss.ar & !0x0000_0f00) | (u32::from(sys_type::TSS32_BUSY) << 8),
            ..new_tss
        };
        // A task switch always sets `TS`, so the first coprocessor instruction
        // in the new task traps and the operating system can swap the
        // floating-point state lazily.
        self.state.sys.cr0 |= cr0::TS;
        if self.state.sys.paging() {
            self.state.sys.cr3 = u64::from(cr3);
            self.state.tlb.flush();
        }

        for (i, value) in gpr.iter().enumerate() {
            self.state.regs.set_dword(i as u8, *value);
        }
        self.state.regs.rip = u64::from(eip);
        let mut flags_value = eflags;
        if !returning && nested {
            flags_value |= flags::NT;
        } else if returning {
            flags_value &= !flags::NT;
        }
        self.set_flags(flags_value);

        // 4. Reload the descriptor caches, `LDTR` first: the segment selectors
        //    may name entries in the new task's local table, which does not
        //    exist until this line runs.
        self.load_ldtr(ldt)?;
        // `CS` is loaded without the usual privilege arithmetic, because the
        // new task's privilege level is whatever its own `CS` says it is.
        let cs_sel = selectors[1];
        let cs_desc = self.descriptor(cs_sel, VEC_TS)?;
        if !cs_desc.is_app() || cs_desc.high & ar::CODE == 0 {
            return Err(Fault::coded(VEC_TS, u32::from(cs_sel & 0xfffc)));
        }
        *self.state.sys.seg_mut(seg::CS) = cs_desc.to_seg(cs_sel);
        self.state.regs.cs = cs_sel;
        self.state.queue.flush();

        for (i, index) in TSS_SEG_ORDER.iter().enumerate() {
            if *index == seg::CS {
                continue;
            }
            let value = selectors[i];
            if *index == seg::SS {
                let stack = self.validate_stack(value, (cs_sel & 3) as u8)?;
                *self.state.sys.seg_mut(seg::SS) = stack.to_seg(value);
                self.state.regs.ss = value;
            } else {
                self.load_segment(*index, value)?;
            }
        }
        Ok(())
    }

    /// Write the outgoing task's state into its own task state segment.
    fn save_task_state(&mut self, tss: SegReg) -> Ex<()> {
        let base = tss.base;
        let eip = self.state.regs.rip;
        self.sys_write32(base.wrapping_add(tss32::EIP), eip as u32)?;
        let eflags = self.state.regs.eflags;
        self.sys_write32(base.wrapping_add(tss32::EFLAGS), eflags)?;
        for i in 0..8u8 {
            let value = self.state.regs.dword(i);
            self.sys_write32(base.wrapping_add(tss32::EAX + 4 * u64::from(i)), value)?;
        }
        for (i, index) in TSS_SEG_ORDER.iter().enumerate() {
            let value = u32::from(self.state.regs.segment(*index));
            self.sys_write32(base.wrapping_add(tss32::ES + 4 * i as u64), value)?;
        }
        let ldt = u32::from(self.state.sys.ldtr.selector);
        self.sys_write32(base.wrapping_add(tss32::LDT), ldt)
    }

    /// Load the local descriptor table register.
    fn load_ldtr(&mut self, selector: u16) -> Ex<()> {
        let sel = Selector(selector);
        if sel.is_null() {
            self.state.sys.ldtr = SegReg::null();
            return Ok(());
        }
        if sel.is_ldt() {
            // A local descriptor table cannot live in a local descriptor
            // table; the selector has to name the global one.
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if desc.is_app() || desc.kind() != sys_type::LDT {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }
        self.state.sys.ldtr = desc.to_seg(selector);
        Ok(())
    }

    // -- The system instructions ---------------------------------------

    /// `LGDT` and `LIDT`: six bytes of limit and base out of memory.
    pub(super) fn load_table_register(&mut self, f: &Fields) -> Ex<()> {
        self.require_ring0()?;
        if f.rm_is_register() {
            return Err(Fault::bare(VEC_UD));
        }
        let (sr, off) = self.ea();
        let limit = self.read_mem(sr, off, 2)? as u32;
        // In 64-bit mode the pseudo-descriptor is ten bytes, not six: the
        // base is a full linear address and there is no 16-bit form. That
        // is why the width comes from the mode rather than from `opsize`,
        // which a `66` prefix could otherwise narrow.
        let width = if self.sixty_four() { 8u8 } else { 4 };
        let base = self.read_mem(sr, off.wrapping_add(2), width)?;
        // With a 16-bit operand size only twenty-four bits of base are loaded,
        // which is the 286 compatibility this instruction carries. The top
        // byte is not read from somewhere else — it is discarded.
        let base = if f.opsize == 2 && !self.sixty_four() {
            base & 0x00ff_ffff
        } else {
            base
        };
        let reg = TableReg { base, limit };
        if f.insn.op == Op::LGDT {
            self.state.sys.gdtr = reg;
        } else {
            self.state.sys.idtr = reg;
        }
        Ok(())
    }

    /// `SGDT` and `SIDT`.
    pub(super) fn store_table_register(&mut self, f: &Fields) -> Ex<()> {
        if f.rm_is_register() {
            return Err(Fault::bare(VEC_UD));
        }
        let reg = if f.insn.op == Op::SGDT {
            self.state.sys.gdtr
        } else {
            self.state.sys.idtr
        };
        let (sr, off) = self.ea();
        self.write_mem(sr, off, 2, u64::from(reg.limit & 0xffff))?;
        let width = if self.sixty_four() { 8u8 } else { 4 };
        let base = if f.opsize == 2 && !self.sixty_four() {
            reg.base & 0x00ff_ffff
        } else {
            reg.base
        };
        self.write_mem(sr, off.wrapping_add(2), width, base)
    }

    /// `LLDT` and `LTR`.
    pub(super) fn load_system_selector(&mut self, f: &Fields) -> Ex<()> {
        self.require_protected()?;
        self.require_ring0()?;
        let selector = self.read_arg(f, f.insn.dst, 2)? as u16;
        if f.insn.op == Op::LLDT {
            return self.load_ldtr(selector);
        }
        let sel = Selector(selector);
        if sel.is_null() || sel.is_ldt() {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        let desc = self.descriptor(selector, VEC_GP)?;
        if desc.is_app() || !matches!(desc.kind(), sys_type::TSS16_AVAIL | sys_type::TSS32_AVAIL) {
            return Err(Fault::gp(u32::from(selector & 0xfffc)));
        }
        if !desc.present() {
            return Err(Fault::coded(VEC_NP, u32::from(selector & 0xfffc)));
        }
        // Loading the task register marks the segment busy, which is what
        // stops a second task from being switched into the same state.
        let busy = if desc.kind() == sys_type::TSS32_AVAIL {
            sys_type::TSS32_BUSY
        } else {
            sys_type::TSS16_BUSY
        };
        self.set_sys_type(selector, busy)?;
        self.state.sys.task = SegReg {
            ar: (desc.ar() & !0x0000_0f00) | (u32::from(busy) << 8),
            ..desc.to_seg(selector)
        };
        Ok(())
    }

    /// `LMSW`, the 286's way into protected mode.
    ///
    /// It reaches only the low four bits of `CR0`, and it famously **cannot
    /// clear `PE`**: the 286 had no way back out of protected mode at all, and
    /// the 386 kept the asymmetry so that 286 software behaves the same way.
    /// Leaving protected mode on a 386 needs `MOV CR0`.
    pub(super) fn lmsw(&mut self, f: &Fields) -> Ex<()> {
        self.require_ring0()?;
        let value = self.read_arg(f, f.insn.dst, 2)?;
        let old = self.state.sys.cr0;
        self.state.sys.cr0 = (old & !cr0::MSW) | (value as u32 & cr0::MSW) | (old & cr0::PE);
        Ok(())
    }

    /// Read a descriptor without loading it, for `LAR`, `LSL`, `VERR` and
    /// `VERW`.
    ///
    /// A selector past the end of its table is a *miss*, not a fault: that is
    /// the difference between asking and loading, and it is why these four
    /// instructions exist.
    fn probe(&mut self, selector: u16) -> Ex<Option<RawDesc>> {
        let sel = Selector(selector);
        if sel.is_null() {
            return Ok(None);
        }
        let table = if sel.is_ldt() {
            let ldtr = self.state.sys.ldtr;
            if !ldtr.present() {
                return Ok(None);
            }
            TableReg {
                base: ldtr.base,
                limit: ldtr.limit,
            }
        } else {
            self.state.sys.gdtr
        };
        if sel.offset() + 7 > table.limit {
            return Ok(None);
        }
        let addr = table.base.wrapping_add(u64::from(sel.offset()));
        let low = self.sys_read32(addr)?;
        let high = self.sys_read32(addr.wrapping_add(4))?;
        Ok(Some(RawDesc {
            low,
            high,
            upper: 0,
        }))
    }

    /// Whether a descriptor is visible from the current privilege level.
    fn descriptor_visible(&self, desc: RawDesc, sel: Selector) -> bool {
        let conforming = desc.is_app() && desc.high & (ar::CODE | ar::DC) == (ar::CODE | ar::DC);
        let effective = if self.cpl() > sel.rpl() {
            self.cpl()
        } else {
            sel.rpl()
        };
        conforming || desc.dpl() >= effective
    }

    /// `LAR` and `LSL`: ask about a descriptor without loading it.
    ///
    /// Both set `ZF` to say whether the selector is visible at all, which is
    /// the only way a program can probe a descriptor table without risking a
    /// fault.
    pub(super) fn lar_lsl(&mut self, f: &Fields) -> Ex<()> {
        self.require_protected()?;
        let selector = self.read_arg(f, f.insn.src, 2)? as u16;
        let Some(desc) = self.probe(selector)? else {
            self.set_flag(flags::ZF, false);
            return Ok(());
        };
        let visible = self.descriptor_visible(desc, Selector(selector));
        let usable = if f.insn.op == Op::LAR {
            // `LAR` refuses the descriptor types that have no meaningful
            // access-rights byte to report.
            desc.is_app()
                || matches!(
                    desc.kind(),
                    sys_type::TSS16_AVAIL
                        | sys_type::LDT
                        | sys_type::TSS16_BUSY
                        | sys_type::CALL_GATE16
                        | sys_type::TASK_GATE
                        | sys_type::TSS32_AVAIL
                        | sys_type::TSS32_BUSY
                        | sys_type::CALL_GATE32
                )
        } else {
            // `LSL` works only on segments and task state segments — a gate
            // has no limit.
            desc.is_app()
                || matches!(
                    desc.kind(),
                    sys_type::TSS16_AVAIL
                        | sys_type::LDT
                        | sys_type::TSS16_BUSY
                        | sys_type::TSS32_AVAIL
                        | sys_type::TSS32_BUSY
                )
        };
        if !visible || !usable {
            self.set_flag(flags::ZF, false);
            return Ok(());
        }
        self.set_flag(flags::ZF, true);
        let value = u64::from(if f.insn.op == Op::LAR {
            desc.ar()
        } else {
            desc.limit()
        });
        let opsize = f.opsize;
        self.write_arg(f, f.insn.dst, opsize, value)
    }

    /// `VERR` and `VERW`: can this privilege level read, or write, that
    /// segment?
    pub(super) fn verify(&mut self, f: &Fields) -> Ex<()> {
        self.require_protected()?;
        let selector = self.read_arg(f, f.insn.dst, 2)? as u16;
        let Some(desc) = self.probe(selector)? else {
            self.set_flag(flags::ZF, false);
            return Ok(());
        };
        if !desc.is_app() || !desc.present() {
            self.set_flag(flags::ZF, false);
            return Ok(());
        }
        let visible = self.descriptor_visible(desc, Selector(selector));
        let code = desc.high & ar::CODE != 0;
        let ok = if f.insn.op == Op::VERR {
            visible && (!code || desc.high & ar::RW != 0)
        } else {
            visible && !code && desc.high & ar::RW != 0
        };
        self.set_flag(flags::ZF, ok);
        Ok(())
    }

    /// `ARPL`: raise a selector's requested privilege level to the caller's.
    ///
    /// The instruction an operating system runs on every selector a user
    /// program hands it, so that a ring-3 caller cannot pass a selector with
    /// `RPL` 0 and have the kernel dereference it on its behalf.
    pub(super) fn arpl(&mut self, f: &Fields) -> Ex<()> {
        self.require_protected()?;
        let dst = self.read_arg(f, f.insn.dst, 2)? as u16;
        let src = self.read_arg(f, f.insn.src, 2)? as u16;
        if dst & 3 < src & 3 {
            self.set_flag(flags::ZF, true);
            let raised = (dst & 0xfffc) | (src & 3);
            self.write_arg(f, f.insn.dst, 2, u64::from(raised))?;
        } else {
            self.set_flag(flags::ZF, false);
        }
        Ok(())
    }

    // -- Control, debug and test registers -----------------------------

    /// `MOV r32, CRn`.
    pub(super) fn read_control(&mut self, index: u8) -> Ex<u64> {
        self.require_ring0()?;
        match index {
            0 => Ok(u64::from(self.state.sys.cr0)),
            2 => Ok(self.state.sys.cr2),
            3 => Ok(self.state.sys.cr3),
            4 if self.cfg.features.cr4 => Ok(self.state.sys.cr4),
            // `CR1` is reserved and `CR4` arrived with the Pentium; naming one
            // on a part that has none is an invalid opcode, not a read of
            // zero — a guest probes with exactly this.
            _ => Err(Fault::bare(VEC_UD)),
        }
    }

    /// `MOV CRn, r`.
    ///
    /// The three writes that change the processor's *mode* all land here, and
    /// the ordering rules between them are the whole of the long-mode
    /// transition (*Intel SDM* volume 3 §9.8.5):
    ///
    /// 1. `CR0.PG` is cleared, so nothing is translating.
    /// 2. `CR4.PAE` is set, because the four-level walk is the PAE walk.
    /// 3. `CR3` is loaded with the `PML4`'s address.
    /// 4. `EFER.LME` is set — through a `WRMSR`, not through here.
    /// 5. `CR0.PG` is set again, and *the processor* sets `EFER.LMA`.
    ///
    /// Only step 5 makes the transition; the rest are prerequisites, and each
    /// is enforced below rather than assumed.
    pub(super) fn write_control(&mut self, index: u8, value: u64) -> Ex<()> {
        self.require_ring0()?;
        match index {
            0 => {
                let valid = if self.cfg.features.extras_486 {
                    cr0::VALID_486
                } else {
                    cr0::VALID_386
                };
                let old = self.state.sys.cr0;
                let new = (value as u32) & valid;
                // Paging without protection is impossible: the page tables are
                // walked with linear addresses, and without segmentation there
                // is nothing for them to protect.
                if new & cr0::PG != 0 && new & cr0::PE == 0 {
                    return Err(Fault::gp(0));
                }
                let lme = self.state.sys.efer & efer::LME != 0;
                if new & cr0::PG != 0 && old & cr0::PG == 0 && lme {
                    // Entering long mode. `CR4.PAE` must already be set: the
                    // four-level walk *is* the PAE walk with a level added, so
                    // there is no long mode without it.
                    if self.state.sys.cr4 & cr4::PAE == 0 {
                        return Err(Fault::gp(0));
                    }
                    self.state.sys.efer |= efer::LMA;
                } else if new & cr0::PG == 0 && old & cr0::PG != 0 {
                    // Leaving paging leaves long mode with it — `LMA` is a
                    // status bit, not a latch.
                    self.state.sys.efer &= !efer::LMA;
                }
                self.state.sys.cr0 = new;
                if (old ^ new) & (cr0::PG | cr0::WP | cr0::PE) != 0 {
                    self.state.tlb.flush();
                }
                self.state.queue.flush();
                Ok(())
            }
            2 => {
                self.state.sys.cr2 = value;
                Ok(())
            }
            3 => {
                self.state.sys.cr3 = value;
                // Every write to `CR3` flushes the whole buffer, which is how
                // an operating system switches address spaces.
                self.state.tlb.flush();
                Ok(())
            }
            4 if self.cfg.features.cr4 => {
                let old = self.state.sys.cr4;
                // Only the bits this core models have storage. A guest that
                // sets one we do not have reads back a zero and knows.
                let valid = cr4::VME
                    | cr4::PVI
                    | cr4::TSD
                    | cr4::DE
                    | cr4::PSE
                    | cr4::PAE
                    | cr4::MCE
                    | cr4::PGE
                    | cr4::PCE
                    | cr4::OSFXSR
                    | cr4::OSXMMEXCPT;
                let new = value & valid;
                // `CR4.PAE` cannot be cleared while long mode is active: the
                // page tables under the processor's feet would change shape.
                if self.state.sys.long_mode() && new & cr4::PAE == 0 {
                    return Err(Fault::gp(0));
                }
                self.state.sys.cr4 = new;
                if (old ^ new) & (cr4::PAE | cr4::PSE | cr4::PGE) != 0 {
                    self.state.tlb.flush();
                }
                Ok(())
            }
            _ => Err(Fault::bare(VEC_UD)),
        }
    }

    /// `RDMSR`: read the model-specific register `ECX` names into `EDX:EAX`.
    ///
    /// # Errors
    ///
    /// `#GP(0)` outside ring 0 or for an address this core does not implement,
    /// and `#UD` on a part with no model-specific registers at all.
    pub(super) fn rdmsr(&mut self) -> Ex<()> {
        if !self.cfg.features.msr {
            return Err(Fault::bare(VEC_UD));
        }
        self.require_ring0()?;
        let index = self.state.regs.rcx as u32;
        let value = self.msr_read(index)?;
        // `EDX:EAX`, and each half **zero-extends** into its 64-bit register
        // as any other 32-bit write does.
        self.state.regs.set_dword(0, value as u32);
        self.state.regs.set_dword(2, (value >> 32) as u32);
        Ok(())
    }

    /// `WRMSR`: write `EDX:EAX` to the model-specific register `ECX` names.
    ///
    /// # Errors
    ///
    /// As [`Exec::rdmsr`], plus `#GP(0)` for a value the register refuses.
    pub(super) fn wrmsr(&mut self) -> Ex<()> {
        if !self.cfg.features.msr {
            return Err(Fault::bare(VEC_UD));
        }
        self.require_ring0()?;
        let index = self.state.regs.rcx as u32;
        let value =
            u64::from(self.state.regs.rax as u32) | (u64::from(self.state.regs.rdx as u32) << 32);
        self.msr_write(index, value)
    }

    fn msr_read(&mut self, index: u32) -> Ex<u64> {
        let sys = &self.state.sys;
        let value = match index {
            msr::EFER if self.cfg.features.long => sys.efer,
            msr::STAR if self.cfg.features.syscall => sys.star,
            msr::LSTAR if self.cfg.features.syscall => sys.lstar,
            msr::CSTAR if self.cfg.features.syscall => sys.cstar,
            msr::SFMASK if self.cfg.features.syscall => sys.sfmask,
            msr::FS_BASE if self.cfg.features.long => sys.fs_base,
            msr::GS_BASE if self.cfg.features.long => sys.gs_base,
            msr::KERNEL_GS_BASE if self.cfg.features.long => sys.kernel_gs_base,
            _ => return Err(Fault::gp(0)),
        };
        Ok(value)
    }

    fn msr_write(&mut self, index: u32, value: u64) -> Ex<()> {
        match index {
            msr::EFER if self.cfg.features.long => {
                // `LME` may not be changed while paging is on: the transition
                // is defined only across a `CR0.PG` edge, and allowing it
                // would leave `LMA` describing a walk that never happened.
                let old = self.state.sys.efer;
                let mut new = value & efer::WRITABLE;
                if !self.cfg.features.nx {
                    new &= !efer::NXE;
                }
                if !self.cfg.features.syscall {
                    new &= !efer::SCE;
                }
                if (old ^ new) & efer::LME != 0 && self.state.sys.cr0 & cr0::PG != 0 {
                    return Err(Fault::gp(0));
                }
                // `LMA` is read-only: whatever software wrote there is
                // discarded and the processor's own bit is kept.
                self.state.sys.efer = new | (old & efer::LMA);
                if (old ^ new) & efer::NXE != 0 {
                    self.state.tlb.flush();
                }
                Ok(())
            }
            msr::STAR if self.cfg.features.syscall => {
                self.state.sys.star = value;
                Ok(())
            }
            msr::LSTAR if self.cfg.features.syscall => {
                if !canonical(value) {
                    return Err(Fault::gp(0));
                }
                self.state.sys.lstar = value;
                Ok(())
            }
            msr::CSTAR if self.cfg.features.syscall => {
                if !canonical(value) {
                    return Err(Fault::gp(0));
                }
                self.state.sys.cstar = value;
                Ok(())
            }
            msr::SFMASK if self.cfg.features.syscall => {
                self.state.sys.sfmask = value;
                Ok(())
            }
            // Writing a segment base MSR writes the *hidden* base of the
            // register too, which is the only way to give `FS` or `GS` a base
            // in 64-bit mode — there is no descriptor involved.
            msr::FS_BASE if self.cfg.features.long => {
                if !canonical(value) {
                    return Err(Fault::gp(0));
                }
                self.state.sys.fs_base = value;
                self.state.sys.seg_mut(seg::FS).base = value;
                Ok(())
            }
            msr::GS_BASE if self.cfg.features.long => {
                if !canonical(value) {
                    return Err(Fault::gp(0));
                }
                self.state.sys.gs_base = value;
                self.state.sys.seg_mut(seg::GS).base = value;
                Ok(())
            }
            msr::KERNEL_GS_BASE if self.cfg.features.long => {
                if !canonical(value) {
                    return Err(Fault::gp(0));
                }
                self.state.sys.kernel_gs_base = value;
                Ok(())
            }
            _ => Err(Fault::gp(0)),
        }
    }

    /// A code or data segment as `SYSCALL` and `SYSRET` synthesise one.
    ///
    /// Neither instruction reads the descriptor table at all: the selectors
    /// come from `IA32_STAR` and the *descriptors* are fixed by the
    /// architecture. That is the whole speed-up — a system call that walks the
    /// GDT is a system call that touches memory — and it is also why an
    /// operating system has to lay its GDT out in the order `STAR` assumes
    /// rather than the other way round.
    fn syscall_seg(selector: u16, code: bool, long: bool) -> SegReg {
        let mut ar = ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::GRANULAR;
        if code {
            ar |= ar::CODE;
            if long {
                ar |= ar::L;
            } else {
                ar |= ar::DB;
            }
        } else {
            ar |= ar::DB;
        }
        SegReg {
            selector,
            base: 0,
            limit: 0xffff_ffff,
            ar,
        }
    }

    /// `SYSCALL`: the fast path into the kernel.
    ///
    /// `RCX` takes the return address and `R11` the flags — *not* the stack,
    /// which is left exactly as user mode had it. A kernel's first job after
    /// `SYSCALL` is therefore to find a stack of its own, which is what
    /// `SWAPGS` and a per-CPU structure behind `GS` are for.
    ///
    /// *AMD64 Architecture Programmer's Manual* volume 3, `SYSCALL`.
    ///
    /// # Errors
    ///
    /// `#UD` where the feature is absent, `EFER.SCE` is clear, or the
    /// processor is not in 64-bit mode.
    pub(super) fn syscall(&mut self) -> Ex<()> {
        if !self.cfg.features.syscall || self.state.sys.efer & efer::SCE == 0 || !self.sixty_four()
        {
            return Err(Fault::bare(VEC_UD));
        }
        let sys = self.state.sys;
        // `STAR[47:32]` is the kernel's `CS`; `SS` is that plus eight, which
        // is why the GDT has to hold them adjacently.
        let cs_sel = ((sys.star >> 32) as u16) & 0xfffc;
        self.state.regs.rcx = self.state.regs.rip;
        self.state.regs.r[3] = u64::from(self.state.regs.eflags);
        self.state.regs.rip = sys.lstar;
        *self.state.sys.seg_mut(seg::CS) = Self::syscall_seg(cs_sel, true, true);
        *self.state.sys.seg_mut(seg::SS) = Self::syscall_seg(cs_sel + 8, false, true);
        self.state.regs.cs = cs_sel;
        self.state.regs.ss = cs_sel + 8;
        let masked = self.state.regs.eflags & !(sys.sfmask as u32) & !flags::RF;
        self.set_flags(masked);
        self.state.queue.flush();
        Ok(())
    }

    /// `SYSRET`: back out again.
    ///
    /// `REX.W` selects the 64-bit form, which returns to 64-bit mode; without
    /// it the return is to compatibility mode, and the selectors differ by
    /// sixteen rather than by nothing. Returning always lands at privilege 3 —
    /// there is no other level `SYSRET` can reach, which is why a kernel
    /// cannot use it to return to itself.
    ///
    /// # Errors
    ///
    /// `#UD` outside 64-bit mode, `#GP(0)` outside ring 0.
    pub(super) fn sysret(&mut self, opsize: u8) -> Ex<()> {
        if !self.cfg.features.syscall || self.state.sys.efer & efer::SCE == 0 || !self.sixty_four()
        {
            return Err(Fault::bare(VEC_UD));
        }
        self.require_ring0()?;
        let sys = self.state.sys;
        let base = (sys.star >> 48) as u16;
        let wide = opsize == 8;
        // `STAR[63:48]` names the compatibility-mode `CS`; the 64-bit one is
        // sixteen further on, and both come back at privilege 3.
        let cs_sel = (base + if wide { 16 } else { 0 }) | 3;
        let ss_sel = (base + 8) | 3;
        let target = self.state.regs.rcx;
        if wide && !canonical(target) {
            return Err(Fault::gp(0));
        }
        *self.state.sys.seg_mut(seg::CS) = Self::syscall_seg(cs_sel, true, wide);
        *self.state.sys.seg_mut(seg::SS) = Self::syscall_seg(ss_sel, false, false);
        self.state.regs.cs = cs_sel;
        self.state.regs.ss = ss_sel;
        self.state.regs.rip = if wide { target } else { target & 0xffff_ffff };
        // `R11` carries the flags back, and `RF` and `VM` are cleared rather
        // than restored — the same rule `IRET` follows.
        let restored = (self.state.regs.r[3] as u32) & !(flags::RF | flags::VM);
        self.set_flags(restored);
        self.state.queue.flush();
        Ok(())
    }

    /// `SWAPGS`: exchange the `GS` base with `IA32_KERNEL_GS_BASE`.
    ///
    /// One instruction because it has to be atomic with respect to an
    /// interrupt: a kernel entered from user mode has no register it can
    /// safely clobber to find its own state, so the exchange is the first
    /// thing it does and nothing may run between the two halves.
    ///
    /// # Errors
    ///
    /// `#UD` outside 64-bit mode, `#GP(0)` outside ring 0.
    pub(super) fn swapgs(&mut self) -> Ex<()> {
        if !self.sixty_four() {
            return Err(Fault::bare(VEC_UD));
        }
        self.require_ring0()?;
        let sys = &mut self.state.sys;
        core::mem::swap(&mut sys.gs_base, &mut sys.kernel_gs_base);
        sys.seg_mut(seg::GS).base = sys.gs_base;
        Ok(())
    }

    /// `MOV r32, DRn`.
    ///
    /// The registers exist and round-trip; the breakpoints they describe are
    /// **not** implemented, so `DR7` can be armed and nothing will fire. That
    /// is a known gap rather than a silent one: software that sets a
    /// breakpoint gets no trap rather than a wrong one.
    pub(super) fn read_debug(&mut self, index: u8) -> Ex<u64> {
        self.require_ring0()?;
        Ok(self.state.sys.dr[usize::from(Self::debug_index(index))])
    }

    /// `MOV DRn, r32`.
    pub(super) fn write_debug(&mut self, index: u8, value: u64) -> Ex<()> {
        self.require_ring0()?;
        self.state.sys.dr[usize::from(Self::debug_index(index))] = value;
        Ok(())
    }

    /// `DR4` and `DR5` alias `DR6` and `DR7` on a 386 and a 486.
    const fn debug_index(index: u8) -> u8 {
        match index & 7 {
            4 => 6,
            5 => 7,
            other => other,
        }
    }

    /// `MOV r32, TRn`: the 386's translation-lookaside-buffer test registers.
    pub(super) fn read_test(&mut self, index: u8) -> Ex<u64> {
        self.require_ring0()?;
        if index < 6 {
            return Err(Fault::bare(VEC_UD));
        }
        Ok(u64::from(self.state.sys.test[(index & 7) as usize]))
    }

    /// `MOV TRn, r32`.
    ///
    /// Stored and otherwise ignored. Writing `TR7` on real silicon injects an
    /// entry into the translation buffer; here the buffer is a cache of the
    /// page tables and nothing else, so an injected entry would have nothing
    /// to be consistent with.
    pub(super) fn write_test(&mut self, index: u8, value: u64) -> Ex<()> {
        self.require_ring0()?;
        if index < 6 {
            return Err(Fault::bare(VEC_UD));
        }
        self.state.sys.test[(index & 7) as usize] = value as u32;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_selector_splits_into_index_table_and_privilege() {
        let s = Selector(0x0023);
        assert_eq!(s.index(), 4);
        assert_eq!(s.offset(), 0x20);
        assert!(!s.is_ldt());
        assert_eq!(s.rpl(), 3);
        assert!(!s.is_null());

        let local = Selector(0x000f);
        assert!(local.is_ldt());
        assert_eq!(local.index(), 1);

        // Every selector whose top thirteen bits are zero is null, whatever
        // its requested privilege level says.
        assert!(Selector(0).is_null());
        assert!(Selector(3).is_null());
        assert!(!Selector(8).is_null());
    }

    #[test]
    fn a_descriptor_reassembles_its_base_from_three_fields() {
        // Base 0x12345678, limit 0xfffff, granular, 32-bit code segment.
        let d = RawDesc {
            low: 0x5678_ffff,
            high: 0x00cf_9a12 | 0x3400_0000,
            upper: 0,
        };
        assert_eq!(d.base(), 0x3412_5678);
        assert_eq!(d.limit(), 0xffff_ffff);
        assert!(d.present());
        assert!(d.is_app());
        assert_eq!(d.dpl(), 0);
    }

    #[test]
    fn granularity_fills_the_low_twelve_bits_with_ones() {
        // A granular limit of zero covers a whole page, not one byte: that is
        // what makes `G` with 0xfffff reach exactly 4 GiB.
        let d = RawDesc {
            low: 0x0000_0000,
            high: 0x0080_9200,
            upper: 0,
        };
        assert_eq!(d.limit(), 0xfff);
    }

    #[test]
    fn an_expand_down_segment_covers_the_top_of_its_range() {
        // A stack segment with limit 0x0fff and B clear: legal offsets are
        // 0x1000 through 0xffff.
        let s = SegReg {
            selector: 0x10,
            base: 0,
            limit: 0x0fff,
            ar: ar::PRESENT | ar::S | ar::RW | ar::DC,
        };
        assert!(s.expand_down());
        assert!(!s.in_bounds(0x0fff, 1));
        assert!(s.in_bounds(0x1000, 1));
        assert!(s.in_bounds(0xfffe, 2));
        assert!(!s.in_bounds(0xffff, 2));
    }

    #[test]
    fn an_ordinary_segment_rejects_an_access_that_straddles_its_limit() {
        let s = SegReg::real_data(0x1000);
        assert_eq!(s.base, 0x1_0000);
        assert!(s.in_bounds(0xfffe, 2));
        assert!(!s.in_bounds(0xffff, 2));
        assert!(s.in_bounds(0xffff, 1));
        // An access that would wrap the 32-bit space is outside every segment.
        assert!(!s.in_bounds(0xffff_ffff, 4));
    }

    #[test]
    fn a_code_segment_is_never_writable_however_its_flags_read() {
        let code = SegReg {
            selector: 8,
            base: 0,
            limit: 0xffff,
            ar: ar::PRESENT | ar::S | ar::CODE | ar::RW,
        };
        assert!(code.readable());
        assert!(!code.writable());
        let data = SegReg::real_data(0);
        assert!(data.readable());
        assert!(data.writable());
    }

    #[test]
    fn the_reset_code_segment_addresses_the_top_of_the_space() {
        // The detail every 386 emulator has to get right for firmware to run:
        // the selector says f000 but the cached base says ffff0000.
        let sys = Sys::reset();
        let cs = sys.seg(seg::CS);
        assert_eq!(cs.selector, 0xf000);
        assert_eq!(cs.base, 0xffff_0000);
        assert_eq!(cs.base.wrapping_add(0xfff0), 0xffff_fff0);
        assert!(!sys.protected());
        assert!(!sys.paging());
    }

    #[test]
    fn a_gate_splits_its_offset_around_the_access_byte() {
        let g = RawDesc {
            low: 0x0008_1234,
            high: 0xabcd_8e00,
            upper: 0,
        };
        assert_eq!(g.gate_selector(), 8);
        assert_eq!(g.gate_offset(), 0xabcd_1234);
        assert_eq!(g.kind(), sys_type::INT_GATE32);
        assert!(g.present());
        assert!(!g.is_app());
    }
}
