//! Coprocessor 0: the R3000 system control processor.
//!
//! # This is the R3000 CP0, not the R4000 CP0
//!
//! The two are routinely conflated and they are different chips with different
//! manuals. The differences that bite, all of them load-bearing here:
//!
//! | | R3000 (this file) | R4000 |
//! | --- | --- | --- |
//! | mode and interrupt enable | a **three-deep stack** of `KU`/`IE` pairs in `Status[5:0]`, pushed on an exception and popped by `RFE` | `Status.EXL` / `Status.ERL`, and `ERET` |
//! | nesting protection | none — a second exception simply pushes again and the third level falls off the end | `EXL` suppresses re-entry |
//! | TLB entries | one `EntryLo` per entry, mapping one page | `EntryLo0`/`EntryLo1`, mapping a page *pair* |
//! | `Wired` | **does not exist**; the boundary `TLBWR` will not touch is hard-wired at 8 | a writable register |
//! | `Context` | 11-bit `PTEBase`, 19-bit `BadVPN` | `BadVPN2`, and a different split |
//! | vectors | `0x8000_0000`/`0x8000_0080`, or `0xBFC0_0100`/`0xBFC0_0180` with `BEV` | `0x8000_0000`/`…0180`/`…0200` |
//!
//! An implementation written from a MIPS32 manual gets every row of that table
//! wrong, which is why this file cites an R3000-era source for each of them.
//!
//! # Sources
//!
//! *IDT R3051/R3052/R3081 Family Hardware User's Manual*, the "CPU Core"
//! chapters covering the system control coprocessor, the memory management
//! unit and the exception model — the clearest description of the R3000A CP0
//! there is. Kane & Heinrich, *MIPS RISC Architecture*, chapters 4 and 5, for
//! the register formats and the exception codes. The LSI Logic LR33300/LR33310
//! datasheet for the part that has no TLB. Where the two disagree, the
//! disagreement is noted at the point it matters.

use crate::core::sync::{AtomicBool, AtomicU32, Ordering};

use super::isa::Endian;

/// Bit positions and masks in the `Status` register (CP0 register 12).
///
/// The bottom six bits are the interrupt-enable and mode **stack**, three
/// levels deep: *current*, *previous* and *old*. An exception shifts it left
/// two places and clears the current pair; `RFE` shifts it right two places,
/// leaving the old pair duplicated. There is no `EXL` here and no `ERL`.
pub mod status {
    /// Current interrupt enable. Clear means no interrupt is taken at all.
    pub const IEC: u32 = 1 << 0;
    /// Current mode: set means **user**, clear means kernel.
    pub const KUC: u32 = 1 << 1;
    /// Previous interrupt enable.
    pub const IEP: u32 = 1 << 2;
    /// Previous mode.
    pub const KUP: u32 = 1 << 3;
    /// Old interrupt enable.
    pub const IEO: u32 = 1 << 4;
    /// Old mode.
    pub const KUO: u32 = 1 << 5;
    /// The whole three-deep stack, which is all `RFE` and exception entry
    /// touch.
    pub const STACK: u32 = 0x3f;
    /// The interrupt mask, bits 15..8, one bit per `Cause.IP` bit.
    pub const IM: u32 = 0xff << 8;
    /// How far the interrupt mask is shifted.
    pub const IM_SHIFT: u32 = 8;
    /// Isolate the data cache: stores go to the cache and never to memory.
    pub const ISC: u32 = 1 << 16;
    /// Swap the instruction and data caches, so the D-cache side of an
    /// isolated access reaches the I-cache instead.
    pub const SWC: u32 = 1 << 17;
    /// Parity zero.
    pub const PZ: u32 = 1 << 18;
    /// Cache miss, set by hardware on an isolated-cache miss.
    pub const CM: u32 = 1 << 19;
    /// Parity error.
    pub const PE: u32 = 1 << 20;
    /// TLB shutdown: set when two TLB entries matched one address, and never
    /// cleared except by a reset.
    pub const TS: u32 = 1 << 21;
    /// Bootstrap exception vectors: send exceptions to the uncached
    /// `0xBFC0_01xx` pair rather than the cached `0x8000_00xx` pair.
    pub const BEV: u32 = 1 << 22;
    /// Reverse endianness in user mode.
    pub const RE: u32 = 1 << 25;
    /// Coprocessor 0 usable, and the base of the four `CU` bits.
    pub const CU0: u32 = 1 << 28;
    /// How far the `CU` bits are shifted.
    pub const CU_SHIFT: u32 = 28;

    /// The bits software may write.
    ///
    /// `CM` and `TS` are set by hardware and are read-only; every reserved bit
    /// reads zero. A machine's `CU` bits are narrowed further at write time by
    /// which coprocessors the configured part actually has.
    pub const WRITABLE: u32 = 0xf257_ff3f;
}

/// Bit positions and masks in the `Cause` register (CP0 register 13).
pub mod cause_bits {
    /// The exception code, bits 6..2.
    pub const EXC_CODE: u32 = 0x1f << 2;
    /// How far the exception code is shifted.
    pub const EXC_SHIFT: u32 = 2;
    /// The pending-interrupt bits, 15..8.
    pub const IP: u32 = 0xff << 8;
    /// How far the pending-interrupt bits are shifted.
    pub const IP_SHIFT: u32 = 8;
    /// The two software interrupt bits, 9..8 — the only bits of `Cause`
    /// software may write.
    pub const SW: u32 = 0x3 << 8;
    /// The six hardware interrupt bits, 15..10, driven by the pins.
    pub const HW: u32 = 0x3f << 10;
    /// How far the hardware interrupt bits are shifted.
    pub const HW_SHIFT: u32 = 10;
    /// The coprocessor number a coprocessor-unusable exception names, bits
    /// 29..28.
    pub const CE: u32 = 0x3 << 28;
    /// How far the coprocessor number is shifted.
    pub const CE_SHIFT: u32 = 28;
    /// Set when the exception was taken on an instruction in a **branch delay
    /// slot**, in which case `EPC` points at the branch rather than at the
    /// instruction that faulted.
    pub const BD: u32 = 1 << 31;

    /// The bits software may write: the two software interrupt requests, and
    /// nothing else.
    pub const WRITABLE: u32 = SW;
}

/// The five-bit exception codes `Cause.ExcCode` carries.
///
/// R3000 defines codes 0 to 12 and reserves the rest; there is no trap
/// instruction and no floating-point exception code, because there is no
/// architectural trap and the FPU is an optional external part.
pub mod exc {
    /// An interrupt.
    pub const INT: u32 = 0;
    /// A store to a page whose dirty bit is clear.
    pub const MOD: u32 = 1;
    /// A TLB miss or an invalid entry, on a load or an instruction fetch.
    pub const TLBL: u32 = 2;
    /// A TLB miss or an invalid entry, on a store.
    pub const TLBS: u32 = 3;
    /// An address error on a load or an instruction fetch: misaligned, or a
    /// kernel address referenced from user mode.
    pub const ADEL: u32 = 4;
    /// An address error on a store.
    pub const ADES: u32 = 5;
    /// A bus error on an instruction fetch.
    pub const IBE: u32 = 6;
    /// A bus error on a data access.
    pub const DBE: u32 = 7;
    /// A `SYSCALL` instruction.
    pub const SYS: u32 = 8;
    /// A `BREAK` instruction.
    pub const BP: u32 = 9;
    /// An encoding this part does not implement.
    pub const RI: u32 = 10;
    /// A coprocessor instruction for a coprocessor that is absent or not
    /// enabled; `Cause.CE` names which.
    pub const CPU: u32 = 11;
    /// Signed overflow in `ADD`, `ADDI` or `SUB`.
    pub const OV: u32 = 12;

    /// The name the monitor and an error message print.
    #[must_use]
    pub const fn name(code: u32) -> &'static str {
        match code {
            INT => "Int",
            MOD => "Mod",
            TLBL => "TLBL",
            TLBS => "TLBS",
            ADEL => "AdEL",
            ADES => "AdES",
            IBE => "IBE",
            DBE => "DBE",
            SYS => "Sys",
            BP => "Bp",
            RI => "RI",
            CPU => "CpU",
            OV => "Ov",
            _ => "reserved",
        }
    }
}

/// The CP0 register numbers, as `MFC0` and `MTC0` name them.
pub mod reg {
    /// Which TLB entry `TLBR` and `TLBWI` act on, and where `TLBP` reports its
    /// answer.
    pub const INDEX: u32 = 0;
    /// A free-running counter `TLBWR` uses to pick a victim entry.
    pub const RANDOM: u32 = 1;
    /// The physical half of a TLB entry.
    pub const ENTRY_LO: u32 = 2;
    /// The R3000A breakpoint program counter.
    pub const BPC: u32 = 3;
    /// The page-table pointer a refill handler reads.
    pub const CONTEXT: u32 = 4;
    /// The R3000A breakpoint data address.
    pub const BDA: u32 = 5;
    /// The R3000A jump-destination register.
    pub const JUMP_DEST: u32 = 6;
    /// The R3000A debug and cache-invalidate control register.
    pub const DCIC: u32 = 7;
    /// The address that caused the most recent address or TLB exception.
    pub const BAD_VADDR: u32 = 8;
    /// The R3000A breakpoint data-address mask.
    pub const BDAM: u32 = 9;
    /// The virtual half of a TLB entry.
    pub const ENTRY_HI: u32 = 10;
    /// The R3000A breakpoint program-counter mask.
    pub const BPCM: u32 = 11;
    /// The status register.
    pub const STATUS: u32 = 12;
    /// Why the most recent exception happened.
    pub const CAUSE: u32 = 13;
    /// Where to resume after an exception.
    pub const EPC: u32 = 14;
    /// The processor revision identifier.
    pub const PRID: u32 = 15;

    /// The name the disassembler and the monitor print.
    ///
    /// Registers 16 to 31 do not exist on an R3000 and read as zero, so they
    /// are printed by number rather than given invented names.
    #[must_use]
    pub const fn name(n: u32) -> Option<&'static str> {
        Some(match n {
            INDEX => "index",
            RANDOM => "random",
            ENTRY_LO => "entrylo",
            BPC => "bpc",
            CONTEXT => "context",
            BDA => "bda",
            JUMP_DEST => "jumpdest",
            DCIC => "dcic",
            BAD_VADDR => "badvaddr",
            BDAM => "bdam",
            ENTRY_HI => "entryhi",
            BPCM => "bpcm",
            STATUS => "sr",
            CAUSE => "cause",
            EPC => "epc",
            PRID => "prid",
            _ => return None,
        })
    }
}

/// Where the processor starts after a reset.
pub const RESET_VECTOR: u32 = 0xbfc0_0000;
/// The TLB-refill vector with `Status.BEV` clear.
pub const REFILL_VECTOR: u32 = 0x8000_0000;
/// The general exception vector with `Status.BEV` clear.
pub const GENERAL_VECTOR: u32 = 0x8000_0080;
/// The TLB-refill vector with `Status.BEV` set — uncached, in the boot ROM.
pub const REFILL_VECTOR_BEV: u32 = 0xbfc0_0100;
/// The general exception vector with `Status.BEV` set.
pub const GENERAL_VECTOR_BEV: u32 = 0xbfc0_0180;

/// How many entries an R3000 TLB has.
pub const TLB_ENTRIES: usize = 64;

/// The lowest entry `TLBWR` will pick.
///
/// The R3000 has **no `Wired` register** — that is an R4000 addition. The
/// boundary below which `Random` never falls is hard-wired at eight, which is
/// why an R3000 kernel keeps its permanently-mapped entries in slots 0 to 7 and
/// why it does not have to program anything to make that safe.
pub const TLB_WIRED: u32 = 8;

/// Which of the four fixed address regions a virtual address falls in.
///
/// The R3000 memory map is a property of the top bits of the address and of
/// nothing else — there is no register that moves a boundary. `kuseg` is the
/// low two gigabytes and is the only region user code may touch; `kseg0` and
/// `kseg1` are two views of the same low 512 MB of physical memory, cached and
/// uncached; `kseg2` is mapped like `kuseg` but is kernel-only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Segment {
    /// `0x0000_0000`–`0x7FFF_FFFF`: TLB-mapped, cached, reachable from user
    /// mode.
    Kuseg,
    /// `0x8000_0000`–`0x9FFF_FFFF`: physical address minus the base, cached,
    /// kernel only.
    Kseg0,
    /// `0xA000_0000`–`0xBFFF_FFFF`: the same physical addresses as `kseg0`,
    /// uncached, kernel only. Where reset code and memory-mapped registers
    /// live.
    Kseg1,
    /// `0xC000_0000`–`0xFFFF_FFFF`: TLB-mapped, cached, kernel only.
    Kseg2,
}

impl Segment {
    /// Which segment an address is in.
    #[inline]
    #[must_use]
    pub const fn of(vaddr: u32) -> Segment {
        match vaddr >> 29 {
            0..=3 => Segment::Kuseg,
            4 => Segment::Kseg0,
            5 => Segment::Kseg1,
            _ => Segment::Kseg2,
        }
    }

    /// Whether user mode may reference this segment.
    #[inline]
    #[must_use]
    pub const fn user_accessible(self) -> bool {
        matches!(self, Segment::Kuseg)
    }

    /// Whether addresses here go through the TLB on a part that has one.
    #[inline]
    #[must_use]
    pub const fn mapped(self) -> bool {
        matches!(self, Segment::Kuseg | Segment::Kseg2)
    }

    /// Whether a miss here takes the dedicated refill vector.
    ///
    /// Only `kuseg` does. A TLB miss in `kseg2` raises the same `TLBL`/`TLBS`
    /// code but vectors to the *general* handler, because the fast refill path
    /// exists for the user page table and a kernel mapping is not on it.
    #[inline]
    #[must_use]
    pub const fn uses_refill_vector(self) -> bool {
        matches!(self, Segment::Kuseg)
    }

    /// Where a direct-mapped segment lands in physical memory.
    ///
    /// `kseg0` and `kseg1` both strip the top three bits, which is what makes
    /// `0x8000_0000` and `0xA000_0000` two views of one megabyte of RAM.
    #[inline]
    #[must_use]
    pub const fn unmapped_phys(vaddr: u32) -> u32 {
        vaddr & 0x1fff_ffff
    }
}

/// One TLB entry: the virtual half and the physical half, exactly as `EntryHi`
/// and `EntryLo` hold them.
///
/// Stored as the raw register values rather than as decoded fields, because
/// `TLBR` has to hand software back precisely what `TLBWI` was given —
/// including the bits neither the hardware nor this model interprets.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TlbEntry {
    /// `EntryHi`: the virtual page number in bits 31..12 and the address-space
    /// identifier in bits 11..6.
    pub hi: u32,
    /// `EntryLo`: the physical frame number in bits 31..12, then `N`
    /// (noncacheable), `D` (dirty, meaning writable), `V` (valid) and `G`
    /// (global) in bits 11 to 8.
    pub lo: u32,
}

impl TlbEntry {
    /// The virtual page number, as the top twenty bits of an address.
    #[inline]
    #[must_use]
    pub const fn vpn(self) -> u32 {
        self.hi & 0xffff_f000
    }

    /// The address-space identifier this entry belongs to.
    #[inline]
    #[must_use]
    pub const fn asid(self) -> u32 {
        (self.hi >> 6) & 0x3f
    }

    /// The physical frame, as the top twenty bits of an address.
    #[inline]
    #[must_use]
    pub const fn pfn(self) -> u32 {
        self.lo & 0xffff_f000
    }

    /// Whether the entry matches every address space (`G`).
    #[inline]
    #[must_use]
    pub const fn global(self) -> bool {
        self.lo & (1 << 8) != 0
    }

    /// Whether the entry is valid (`V`). An invalid entry that *matches* is
    /// not a refill: it raises `TLBL`/`TLBS` through the general vector.
    #[inline]
    #[must_use]
    pub const fn valid(self) -> bool {
        self.lo & (1 << 9) != 0
    }

    /// Whether the entry may be written (`D`).
    ///
    /// The bit is called "dirty" and behaves as a **write-enable**: the R3000
    /// has no hardware dirty-bit update, so a kernel clears `D` to catch the
    /// first write and sets it in the `Mod` handler.
    #[inline]
    #[must_use]
    pub const fn writable(self) -> bool {
        self.lo & (1 << 10) != 0
    }

    /// Whether the mapping is marked noncacheable (`N`).
    #[inline]
    #[must_use]
    pub const fn noncacheable(self) -> bool {
        self.lo & (1 << 11) != 0
    }
}

/// The 64-entry, fully associative TLB.
///
/// **Architectural state, not a cache.** Unlike the software TLB in
/// `cpu::riscv::mmu`, which is a derived accelerator a snapshot may drop, this
/// one is guest-visible through `TLBR` and `TLBP`: an operating system writes
/// entries and reads them back, so it is saved and restored like any other
/// register file (`ROADMAP.md` §4.5 distinguishes exactly these two cases).
#[derive(Debug, Clone)]
pub struct Tlb {
    entries: [TlbEntry; TLB_ENTRIES],
}

impl Default for Tlb {
    fn default() -> Self {
        Tlb::new()
    }
}

/// What looking an address up in the TLB produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lookup {
    /// A matching, valid entry. Carries the physical frame and whether the
    /// entry permits writes.
    Hit {
        /// The physical frame the page maps to.
        pfn: u32,
        /// Whether the entry's `D` bit permits a store.
        writable: bool,
        /// Whether the entry is marked noncacheable.
        noncacheable: bool,
    },
    /// Nothing matched: a refill.
    Miss,
    /// An entry matched but its `V` bit was clear.
    Invalid,
    /// More than one entry matched. Real hardware may physically damage itself
    /// here and latches `Status.TS`; the caller sets that bit and treats it as
    /// a miss.
    Conflict,
}

impl Tlb {
    /// An empty TLB.
    ///
    /// Every entry is zero, which is a *valid* mapping of virtual page zero to
    /// physical frame zero with `V` clear — so nothing is reachable through it
    /// until software writes an entry, which is what a real one comes up like.
    #[must_use]
    pub const fn new() -> Tlb {
        Tlb {
            entries: [TlbEntry { hi: 0, lo: 0 }; TLB_ENTRIES],
        }
    }

    /// One entry, by index. Indices wrap into range rather than panicking:
    /// `Index` is six bits wide and every value of it names an entry.
    #[inline]
    #[must_use]
    pub fn entry(&self, index: u32) -> TlbEntry {
        self.entries[(index as usize) % TLB_ENTRIES]
    }

    /// Overwrite one entry.
    pub fn set_entry(&mut self, index: u32, entry: TlbEntry) {
        self.entries[(index as usize) % TLB_ENTRIES] = entry;
    }

    /// Every entry, in index order — for the monitor and for a snapshot.
    #[must_use]
    pub fn entries(&self) -> &[TlbEntry; TLB_ENTRIES] {
        &self.entries
    }

    /// Look one virtual address up.
    ///
    /// An entry matches when its virtual page number equals the address's and
    /// either its `G` bit is set or its `ASID` equals the current one. Two
    /// matches is a hardware fault rather than a preference for the lower
    /// index, and is reported as such.
    #[must_use]
    pub fn lookup(&self, vaddr: u32, asid: u32) -> Lookup {
        let vpn = vaddr & 0xffff_f000;
        let mut found: Option<TlbEntry> = None;
        for entry in &self.entries {
            if entry.vpn() != vpn {
                continue;
            }
            if !entry.global() && entry.asid() != asid {
                continue;
            }
            if found.is_some() {
                return Lookup::Conflict;
            }
            found = Some(*entry);
        }
        match found {
            None => Lookup::Miss,
            Some(e) if !e.valid() => Lookup::Invalid,
            Some(e) => Lookup::Hit {
                pfn: e.pfn(),
                writable: e.writable(),
                noncacheable: e.noncacheable(),
            },
        }
    }

    /// The index of the entry matching `hi`, for `TLBP`.
    #[must_use]
    pub fn probe(&self, hi: u32) -> Option<u32> {
        let vpn = hi & 0xffff_f000;
        let asid = (hi >> 6) & 0x3f;
        self.entries
            .iter()
            .position(|e| e.vpn() == vpn && (e.global() || e.asid() == asid))
            .map(|i| i as u32)
    }
}

/// The interrupt inputs and the reset request, outside the execution lock.
///
/// Same shape and same reason as the RISC-V core's: a device raising an
/// interrupt from inside a write the CPU itself issued must not have to take
/// the CPU's own lock, or a `sw` to an interrupt controller would deadlock
/// against the instruction that made it.
#[derive(Debug, Default)]
pub struct Lines {
    /// The six hardware interrupt pins, in `Cause.IP[7:2]` order, as a
    /// six-bit level.
    hw: AtomicU32,
    /// Whether a reset has been requested since the last step.
    reset: AtomicBool,
}

impl Lines {
    /// Drive one hardware interrupt pin. `pin` is 0 to 5, matching
    /// `Cause.IP[2]` to `Cause.IP[7]`.
    pub fn set_hw(&self, pin: u32, asserted: bool) {
        if pin >= 6 {
            // There is no seventh pin. Ignoring it beats aliasing it onto one
            // that exists, which would make a wiring mistake in a machine file
            // look like a working interrupt.
            return;
        }
        let bit = 1u32 << pin;
        // Relaxed: the interrupt level is a standalone fact about a wire, and
        // nothing else is ordered against it. The CPU samples it once per
        // instruction, which is as often as a guest can observe it.
        let mut cur = self.hw.load(Ordering::Relaxed);
        loop {
            let next = if asserted { cur | bit } else { cur & !bit };
            match self
                .hw
                .compare_exchange_weak(cur, next, Ordering::Relaxed, Ordering::Relaxed)
            {
                Ok(_) => return,
                Err(seen) => cur = seen,
            }
        }
    }

    /// The six hardware pins as a level.
    #[must_use]
    pub fn hw(&self) -> u32 {
        self.hw.load(Ordering::Relaxed) & 0x3f
    }

    /// Replace every pin at once — how a snapshot restores them.
    pub fn set_all_hw(&self, level: u32) {
        self.hw.store(level & 0x3f, Ordering::Relaxed);
    }

    /// Latch a reset request. It takes effect at the next step, because a
    /// reset is a signal rather than a method call.
    pub fn request_reset(&self) {
        self.reset.store(true, Ordering::Relaxed);
    }

    /// Take a latched reset request, if there is one.
    pub fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::Relaxed)
    }
}

/// The coprocessor-0 register file.
///
/// Plain fields rather than an array, because only sixteen of the thirty-two
/// numbers exist on an R3000 and half of those have a shape worth naming. The
/// six R3000A debug registers (`BPC`, `BDA`, `JumpDest`, `DCIC`, `BDAM`,
/// `BPCM`) are **stored but not acted on**: a guest that programs a hardware
/// breakpoint reads back what it wrote and the breakpoint never fires. That is
/// a documented gap rather than a silent one — see the module docs.
#[derive(Debug, Clone)]
pub struct Cp0 {
    /// Which TLB entry `TLBR`/`TLBWI` use, and where `TLBP` reports a hit.
    pub index: u32,
    /// The victim counter `TLBWR` uses, counting down from 63 to
    /// [`TLB_WIRED`].
    pub random: u32,
    /// The physical half of the entry being written or read.
    pub entry_lo: u32,
    /// The page-table base a refill handler indexes with the faulting page.
    pub context: u32,
    /// The address of the most recent address-error or TLB exception.
    pub bad_vaddr: u32,
    /// The virtual half of the entry being written or read, and the current
    /// address-space identifier.
    pub entry_hi: u32,
    /// The status register: the mode stack, the interrupt mask, the cache
    /// controls and the coprocessor-usable bits.
    pub status: u32,
    /// Why the most recent exception happened.
    ///
    /// Only the two software-interrupt bits of this are software-writable, and
    /// the six hardware-interrupt bits are **not stored here at all** — they
    /// are read live off the pins, because `Cause.IP` on an R3000 reports the
    /// state of the wires rather than a latch.
    pub cause: u32,
    /// Where to resume after an exception, which is the *branch* rather than
    /// the delay slot when `Cause.BD` is set.
    pub epc: u32,
    /// The read-only processor revision identifier.
    pub prid: u32,
    /// The R3000A debug registers, in the order
    /// `BPC`, `BDA`, `JumpDest`, `DCIC`, `BDAM`, `BPCM`.
    pub debug: [u32; 6],
}

impl Cp0 {
    /// The register file as it is after a reset.
    ///
    /// `BEV` is set and `TS` is clear, which is what a reset leaves; the mode
    /// stack comes up kernel with interrupts disabled, which is the only state
    /// from which the reset vector's code can run. `Random` starts at its
    /// maximum.
    #[must_use]
    pub fn new(prid: u32) -> Cp0 {
        Cp0 {
            index: 0,
            random: (TLB_ENTRIES - 1) as u32,
            entry_lo: 0,
            context: 0,
            bad_vaddr: 0,
            entry_hi: 0,
            status: status::BEV,
            cause: 0,
            epc: 0,
            prid,
            debug: [0; 6],
        }
    }

    /// Whether the processor is in kernel mode.
    #[inline]
    #[must_use]
    pub const fn kernel_mode(&self) -> bool {
        self.status & status::KUC == 0
    }

    /// Whether interrupts are enabled at all.
    #[inline]
    #[must_use]
    pub const fn interrupts_enabled(&self) -> bool {
        self.status & status::IEC != 0
    }

    /// The current address-space identifier.
    #[inline]
    #[must_use]
    pub const fn asid(&self) -> u32 {
        (self.entry_hi >> 6) & 0x3f
    }

    /// Whether coprocessor `n` may be used right now.
    ///
    /// Coprocessor 0 is usable from kernel mode whatever `CU0` says, which is
    /// what lets a reset handler run before it has written `Status` at all.
    #[inline]
    #[must_use]
    pub const fn coprocessor_usable(&self, n: u32) -> bool {
        if n == 0 && self.kernel_mode() {
            return true;
        }
        self.status & (1 << (status::CU_SHIFT + n)) != 0
    }

    /// The byte order data accesses use.
    ///
    /// `Status.RE` reverses endianness **in user mode only**, which is how a
    /// big-endian kernel runs a little-endian user program. Kernel accesses
    /// always use the pin's order.
    #[inline]
    #[must_use]
    pub fn data_endian(&self, pin: Endian) -> Endian {
        if !self.kernel_mode() && self.status & status::RE != 0 {
            match pin {
                Endian::Big => Endian::Little,
                Endian::Little => Endian::Big,
            }
        } else {
            pin
        }
    }

    /// `Cause` as software reads it, with the live pin levels merged in.
    #[must_use]
    pub fn cause_with(&self, hw: u32) -> u32 {
        (self.cause & !cause_bits::HW) | ((hw & 0x3f) << cause_bits::HW_SHIFT)
    }

    /// Which interrupt requests are both pending and unmasked.
    #[must_use]
    pub fn ready_interrupts(&self, hw: u32) -> u32 {
        let pending = (self.cause_with(hw) & cause_bits::IP) >> cause_bits::IP_SHIFT;
        let mask = (self.status & status::IM) >> status::IM_SHIFT;
        pending & mask
    }

    /// Push the mode stack, as taking an exception does.
    ///
    /// Shift `Status[5:0]` left two places and clear the current pair, which
    /// leaves the processor in kernel mode with interrupts disabled and the
    /// two older levels remembered. The third level falls off the end: an
    /// R3000 has no `EXL` and no protection against a third nested exception
    /// losing the outermost return state.
    pub fn push_mode(&mut self) {
        let stack = self.status & status::STACK;
        self.status = (self.status & !status::STACK) | ((stack << 2) & status::STACK);
    }

    /// Pop the mode stack, as `RFE` does.
    ///
    /// Shift `Status[5:0]` right two places. The *old* pair stays where it is
    /// as well as being copied down, which is exactly what the hardware does
    /// and why a handler that returns twice sees the same outer state twice.
    /// `RFE` does not change the program counter — it is executed in the delay
    /// slot of the `JR` that does.
    pub fn pop_mode(&mut self) {
        let stack = self.status & status::STACK;
        let popped = (stack >> 2) | (stack & (status::KUO | status::IEO));
        self.status = (self.status & !status::STACK) | popped;
    }

    /// Set `Context.BadVPN` from a faulting address.
    ///
    /// The R3000 `Context` is `PTEBase` in bits 31..21 and `BadVPN` in bits
    /// 20..2 — nineteen bits, which is `VA[30:12]`, because `kuseg` is two
    /// gigabytes and `VA[31]` is therefore always zero for the addresses a
    /// refill handler sees. This is **not** the R4000's `BadVPN2`.
    pub fn set_context_vpn(&mut self, vaddr: u32) {
        self.context = (self.context & 0xffe0_0000) | ((vaddr & 0x7fff_f000) >> 10);
    }

    /// Advance `Random`, which happens once per instruction.
    ///
    /// It counts **down** and wraps from [`TLB_WIRED`] back to the top entry,
    /// so `TLBWR` never evicts one of the eight entries a kernel keeps for
    /// itself. There is no `Wired` register to program: the boundary is fixed.
    pub fn tick_random(&mut self) {
        self.random = if self.random <= TLB_WIRED {
            (TLB_ENTRIES - 1) as u32
        } else {
            self.random - 1
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mode_stack_is_three_deep_and_rfe_pops_it() {
        let mut cp0 = Cp0::new(0);
        // User mode, interrupts on: KUc = IEc = 1, the rest clear.
        cp0.status = status::KUC | status::IEC;
        cp0.push_mode();
        assert_eq!(
            cp0.status & status::STACK,
            status::KUP | status::IEP,
            "the current pair moved to previous and the new current is kernel \
             mode with interrupts off"
        );
        assert!(cp0.kernel_mode());
        assert!(!cp0.interrupts_enabled());

        cp0.pop_mode();
        assert_eq!(cp0.status & status::STACK, status::KUC | status::IEC);
        assert!(!cp0.kernel_mode());
        assert!(cp0.interrupts_enabled());
    }

    #[test]
    fn a_third_nested_exception_loses_the_outermost_level() {
        // The R3000 has no EXL: the stack is three deep and the third push
        // shifts the oldest pair off the end. A model that quietly kept it
        // would let a guest return from a nesting depth real hardware cannot.
        let mut cp0 = Cp0::new(0);
        cp0.status = status::KUC | status::IEC;
        cp0.push_mode();
        cp0.push_mode();
        assert_eq!(cp0.status & status::STACK, status::KUO | status::IEO);
        cp0.push_mode();
        assert_eq!(cp0.status & status::STACK, 0, "the outermost pair is gone");
    }

    #[test]
    fn rfe_leaves_the_old_pair_in_place() {
        // The shift duplicates the old pair rather than clearing it, so two
        // RFEs in a row give the same answer twice. That is the hardware's
        // behaviour and software depends on it not being a rotate.
        let mut cp0 = Cp0::new(0);
        cp0.status = status::KUO | status::IEO;
        cp0.pop_mode();
        assert_eq!(
            cp0.status & status::STACK,
            status::KUO | status::IEO | status::KUP | status::IEP
        );
        cp0.pop_mode();
        assert_eq!(
            cp0.status & status::STACK,
            status::KUO | status::IEO | status::KUP | status::IEP | status::KUC | status::IEC
        );
    }

    #[test]
    fn the_segments_are_where_the_manual_puts_them() {
        assert_eq!(Segment::of(0x0000_0000), Segment::Kuseg);
        assert_eq!(Segment::of(0x7fff_ffff), Segment::Kuseg);
        assert_eq!(Segment::of(0x8000_0000), Segment::Kseg0);
        assert_eq!(Segment::of(0x9fff_ffff), Segment::Kseg0);
        assert_eq!(Segment::of(0xa000_0000), Segment::Kseg1);
        assert_eq!(Segment::of(0xbfff_ffff), Segment::Kseg1);
        assert_eq!(Segment::of(0xc000_0000), Segment::Kseg2);
        assert_eq!(Segment::of(0xffff_ffff), Segment::Kseg2);

        // The two direct-mapped segments are the same physical memory.
        assert_eq!(Segment::unmapped_phys(0x8000_1234), 0x0000_1234);
        assert_eq!(Segment::unmapped_phys(0xa000_1234), 0x0000_1234);
        assert_eq!(Segment::unmapped_phys(0xbfc0_0000), 0x1fc0_0000);
    }

    #[test]
    fn only_kuseg_takes_the_refill_vector() {
        assert!(Segment::Kuseg.uses_refill_vector());
        assert!(!Segment::Kseg2.uses_refill_vector());
        assert!(Segment::Kseg2.mapped());
        assert!(!Segment::Kseg0.mapped());
    }

    #[test]
    fn context_holds_the_faulting_page_where_the_r3000_puts_it() {
        let mut cp0 = Cp0::new(0);
        cp0.context = 0x1234_5678;
        cp0.set_context_vpn(0x0abc_d123);
        // PTEBase, bits 31..21, is untouched.
        assert_eq!(cp0.context & 0xffe0_0000, 0x1234_5678 & 0xffe0_0000);
        // BadVPN, bits 20..2, is VA[30:12].
        assert_eq!((cp0.context >> 2) & 0x7ffff, 0x0abcd);
        assert_eq!(cp0.context & 3, 0);
    }

    #[test]
    fn random_counts_down_and_never_reaches_the_wired_entries() {
        let mut cp0 = Cp0::new(0);
        assert_eq!(cp0.random, 63);
        let mut seen_low = u32::MAX;
        for _ in 0..200 {
            cp0.tick_random();
            seen_low = seen_low.min(cp0.random);
        }
        assert_eq!(seen_low, TLB_WIRED, "TLBWR must never pick entries 0..7");
        assert!(cp0.random <= 63);
    }

    #[test]
    fn a_tlb_entry_matches_on_asid_unless_it_is_global() {
        let mut tlb = Tlb::new();
        // Virtual page 0x1000_0 for ASID 5, valid and writable.
        tlb.set_entry(
            0,
            TlbEntry {
                hi: 0x1000_0000 | (5 << 6),
                lo: 0x0020_0000 | (1 << 9) | (1 << 10),
            },
        );
        assert!(matches!(tlb.lookup(0x1000_0abc, 5), Lookup::Hit { .. }));
        assert_eq!(tlb.lookup(0x1000_0abc, 6), Lookup::Miss);

        // The same entry marked global matches every address space.
        tlb.set_entry(
            0,
            TlbEntry {
                hi: 0x1000_0000 | (5 << 6),
                lo: 0x0020_0000 | (1 << 8) | (1 << 9) | (1 << 10),
            },
        );
        assert!(matches!(tlb.lookup(0x1000_0abc, 6), Lookup::Hit { .. }));
    }

    #[test]
    fn an_entry_with_v_clear_is_invalid_rather_than_a_miss() {
        // The distinction decides which vector the exception takes, so it is
        // not a nicety: a refill goes to 0x8000_0000 and an invalid entry to
        // 0x8000_0080.
        let mut tlb = Tlb::new();
        tlb.set_entry(
            3,
            TlbEntry {
                hi: 0x2000_0000,
                lo: 0x0030_0000 | (1 << 8),
            },
        );
        assert_eq!(tlb.lookup(0x2000_0000, 0), Lookup::Invalid);
    }

    #[test]
    fn two_matching_entries_are_a_conflict() {
        let mut tlb = Tlb::new();
        let e = TlbEntry {
            hi: 0x3000_0000,
            lo: 0x0040_0000 | (1 << 8) | (1 << 9),
        };
        tlb.set_entry(1, e);
        tlb.set_entry(2, e);
        assert_eq!(tlb.lookup(0x3000_0000, 0), Lookup::Conflict);
    }

    #[test]
    fn probe_finds_the_index_tlbp_should_report() {
        let mut tlb = Tlb::new();
        tlb.set_entry(
            17,
            TlbEntry {
                hi: 0x4000_0000 | (9 << 6),
                lo: (1 << 9),
            },
        );
        assert_eq!(tlb.probe(0x4000_0000 | (9 << 6)), Some(17));
        assert_eq!(tlb.probe(0x4000_0000 | (8 << 6)), None);
    }

    #[test]
    fn cop0_is_usable_from_kernel_mode_without_cu0() {
        let mut cp0 = Cp0::new(0);
        cp0.status = 0; // kernel mode, no CU bits at all
        assert!(cp0.coprocessor_usable(0));
        assert!(!cp0.coprocessor_usable(2));
        // In user mode it needs CU0 like any other coprocessor.
        cp0.status = status::KUC;
        assert!(!cp0.coprocessor_usable(0));
        cp0.status = status::KUC | status::CU0;
        assert!(cp0.coprocessor_usable(0));
    }

    #[test]
    fn the_hardware_interrupt_bits_come_from_the_pins() {
        let mut cp0 = Cp0::new(0);
        // Software interrupt 0 requested, hardware pin 2 (Cause.IP4) high.
        cp0.cause = 1 << 8;
        let merged = cp0.cause_with(0b000_100);
        assert_eq!(merged & cause_bits::IP, (1 << 8) | (1 << 12));
        // Nothing is ready until the mask lets it through.
        assert_eq!(cp0.ready_interrupts(0b000_100), 0);
        cp0.status = status::IM;
        assert_eq!(cp0.ready_interrupts(0b000_100), 0b0001_0001);
    }

    #[test]
    fn reverse_endianness_applies_only_to_user_mode() {
        let mut cp0 = Cp0::new(0);
        cp0.status = status::RE; // set, but kernel mode
        assert_eq!(cp0.data_endian(Endian::Big), Endian::Big);
        cp0.status = status::RE | status::KUC;
        assert_eq!(cp0.data_endian(Endian::Big), Endian::Little);
        cp0.status = status::KUC;
        assert_eq!(cp0.data_endian(Endian::Big), Endian::Big);
    }

    #[test]
    fn the_lines_hold_six_pins_and_a_reset_request() {
        let lines = Lines::default();
        assert_eq!(lines.hw(), 0);
        lines.set_hw(0, true);
        lines.set_hw(5, true);
        assert_eq!(lines.hw(), 0b10_0001);
        lines.set_hw(0, false);
        assert_eq!(lines.hw(), 0b10_0000);
        // A seventh pin does not exist and is ignored rather than aliasing.
        lines.set_hw(6, true);
        assert_eq!(lines.hw(), 0b10_0000);

        assert!(!lines.take_reset_request());
        lines.request_reset();
        assert!(lines.take_reset_request());
        assert!(!lines.take_reset_request());
    }
}
