//! Address translation: the Sv39 and Sv32 page-table walk, PMP, and the
//! software TLB that sits in front of both.
//!
//! *The RISC-V Instruction Set Manual, Volume II: Privileged Architecture*
//! (CC-BY-4.0) — the "Supervisor Address Translation and Protection" chapter
//! for `satp` and the Sv32/Sv39 walk algorithm, and the "Physical Memory
//! Protection" chapter for the `pmpcfg`/`pmpaddr` matching rules.
//!
//! # Why the TLB is unconditional
//!
//! `ROADMAP.md` §4.1 makes the software TLB part of every CPU, not an
//! MMU-only feature, and this is the core it was designed for. It is
//! **derived state**: never serialized, and invalidated wholesale by a
//! generation counter that `SFENCE.VMA`, a `satp` write and any `mstatus`
//! change that alters translation all bump. A snapshot restores the
//! generation and the TLB comes back empty, which is always correct and never
//! stale.
//!
//! # Structure
//!
//! [`Tlb`] is direct-mapped and **split by access type**, which is what makes
//! caching safe: an entry only exists because a walk for *that* access type
//! succeeded, so a cached store translation has already had its dirty bit set
//! and a cached fetch translation has already been checked for execute
//! permission. One shared array would need the permission bits re-checked on
//! every hit, which is most of the walk's cost back again.

use super::csr::{Csrs, Priv, status};
use super::isa::Xlen;

/// What an access is for.
///
/// The three cases differ in which PTE permission bit they need, which fault
/// they raise, and which half of the TLB they use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// An instruction fetch.
    Fetch,
    /// A load, or the read half of an atomic.
    Load,
    /// A store, or the write half of an atomic.
    Store,
}

impl Access {
    /// The index of this access type's half of the TLB.
    #[inline]
    const fn slot(self) -> usize {
        match self {
            Access::Fetch => 0,
            Access::Load => 1,
            Access::Store => 2,
        }
    }
}

/// Why a translation failed.
///
/// The distinction matters to the guest: a page fault means "the tables say
/// no" and a well-written kernel will handle it, while an access fault means
/// "the physical memory protection says no" and normally will not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// The page tables refused, or a PTE is malformed.
    Page,
    /// PMP refused, or a page-table read itself faulted.
    Access,
}

/// PTE bit positions, shared by Sv32 and Sv39.
pub mod pte {
    /// Valid.
    pub const V: u64 = 1 << 0;
    /// Readable.
    pub const R: u64 = 1 << 1;
    /// Writable.
    pub const W: u64 = 1 << 2;
    /// Executable.
    pub const X: u64 = 1 << 3;
    /// Accessible from user mode.
    pub const U: u64 = 1 << 4;
    /// Global: valid in every address space.
    pub const G: u64 = 1 << 5;
    /// Accessed.
    pub const A: u64 = 1 << 6;
    /// Dirty.
    pub const D: u64 = 1 << 7;
}

/// The size of a page, in bytes and in bits.
pub const PAGE_BITS: u32 = 12;
/// The number of bytes in a page.
pub const PAGE_SIZE: u64 = 1 << PAGE_BITS;

/// Physical memory as the page-table walk *reads* it.
///
/// Split out of [`PhysMem`] so that the debug walk can demand this and only
/// this. Setting the accessed and dirty bits is the one side effect a RISC-V
/// translation has, and [`translate_debug`] must not have it — so rather than
/// asking the walk to remember a flag, the debug entry point takes a memory
/// handle that **cannot write**. A caller that gets it wrong does not compile.
pub trait ReadPte {
    /// Read a `bytes`-wide page-table entry from a physical address.
    fn read_pte(&mut self, addr: u64, bytes: u32) -> Option<u64>;
}

/// Physical memory as a *guest* page-table walk needs to see it.
///
/// A trait rather than two closures because a walk both reads a PTE and may
/// write it back to set the accessed and dirty bits, and threading two
/// `FnMut`s through a loop is worse than one object.
pub trait PhysMem: ReadPte {
    /// Write a `bytes`-wide page-table entry back to a physical address.
    fn write_pte(&mut self, addr: u64, bytes: u32, value: u64) -> Option<()>;
}

/// The parameters of one translation scheme.
struct Scheme {
    /// How many levels the walk has.
    levels: u32,
    /// How many bits of virtual page number each level consumes.
    vpn_bits: u32,
    /// How wide a page-table entry is, in bytes.
    pte_bytes: u32,
    /// How many bits of the virtual address are significant.
    va_bits: u32,
    /// How wide a physical page number is, in both `satp` and a PTE.
    ///
    /// This is a *field width*, and reading it as anything wider silently
    /// folds the neighbouring field into the address: in `satp` the neighbour
    /// is `ASID`, and software probes `ASID`'s width by writing all ones to
    /// it (Volume II, "Supervisor Address Translation and Protection").
    ppn_bits: u32,
}

/// Sv32: two levels of 10-bit indices over a 32-bit address space.
const SV32: Scheme = Scheme {
    levels: 2,
    vpn_bits: 10,
    pte_bytes: 4,
    va_bits: 32,
    ppn_bits: 22,
};

/// Sv39: three levels of 9-bit indices over a 39-bit sign-extended address
/// space.
const SV39: Scheme = Scheme {
    levels: 3,
    vpn_bits: 9,
    pte_bytes: 8,
    va_bits: 39,
    ppn_bits: 44,
};

/// The translation scheme this `XLEN` walks.
const fn scheme(xlen: Xlen) -> &'static Scheme {
    match xlen {
        Xlen::Rv32 => &SV32,
        Xlen::Rv64 => &SV39,
    }
}

/// A mask of the low `bits` bits.
const fn mask(bits: u32) -> u64 {
    (1u64 << bits) - 1
}

/// Whether address translation is switched on for `mode`.
///
/// Machine mode is never translated — Volume II is explicit that `satp` has no
/// effect on M-mode accesses — which is why the effective privilege that
/// `MPRV` produces is what this takes, not the current mode.
#[must_use]
pub fn translation_active(csrs: &Csrs, mode: Priv) -> bool {
    if mode == Priv::Machine {
        return false;
    }
    match csrs.xlen {
        Xlen::Rv32 => csrs.satp >> 31 != 0,
        Xlen::Rv64 => csrs.satp >> 60 == 8,
    }
}

/// The address-space identifier currently installed, for tagging TLB entries.
#[must_use]
pub fn asid(csrs: &Csrs) -> u64 {
    match csrs.xlen {
        Xlen::Rv32 => (csrs.satp >> 22) & 0x1ff,
        Xlen::Rv64 => (csrs.satp >> 44) & 0xffff,
    }
}

/// The root page table's physical address.
///
/// `satp.PPN` is 22 bits under Sv32 and **44** under Sv39; `ASID` sits
/// directly above it. Masking wider than the field moves the root page table
/// the moment a guest writes a non-zero `ASID` — which every Linux kernel
/// does at boot, writing all ones to discover how many `ASID` bits the
/// hardware implements.
fn root(csrs: &Csrs) -> u64 {
    (csrs.satp & mask(scheme(csrs.xlen).ppn_bits)) << PAGE_BITS
}

/// What a walk found, and what the architecture would write on the way.
///
/// The walk itself is pure: it reads descriptors and reports the accessed and
/// dirty update its leaf *would* need, and the caller decides whether to make
/// it. That is what lets one description of Sv32/Sv39 serve both a guest access
/// — which sets those bits — and a debugger's, which must not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Walked {
    /// The physical address the virtual one names.
    phys: u64,
    /// `(address, width, new value)` for the accessed/dirty write-back, when
    /// the leaf needs one and the walk was a guest's.
    update: Option<(u64, u32, u64)>,
}

/// Translate a virtual address, walking the page tables.
///
/// Returns the physical address, or the fault to raise. The caller is
/// responsible for the PMP check on the result — [`pmp_allows`] — because the
/// walk itself must also PMP-check every table read, and doing both here would
/// hide which one refused.
///
/// # Panics
///
/// Never: every array index is derived from a masked field.
pub fn translate<M: PhysMem>(
    csrs: &Csrs,
    mem: &mut M,
    addr: u64,
    kind: Access,
    mode: Priv,
) -> Result<u64, Fault> {
    let walked = walk(csrs, mem, addr, Some(kind), mode)?;
    // The specification allows either raising a page fault when A or D is
    // clear, or setting them in hardware. Setting them is what real
    // implementations do and what lets an operating system leave them out of
    // its fault handler entirely.
    if let Some((at, bytes, value)) = walked.update {
        if !pmp_allows(csrs, at, u64::from(bytes), Access::Store, mode) {
            return Err(Fault::Access);
        }
        mem.write_pte(at, bytes, value).ok_or(Fault::Access)?;
    }
    Ok(walked.phys)
}

/// Resolve a virtual address for a debugger: the same tables, no side effects,
/// and no permission check.
///
/// Three differences from [`translate`], and each of them is the point:
///
/// * **`mem` is [`ReadPte`], not [`PhysMem`]**, so the accessed and dirty bits
///   cannot be set — not "are not", *cannot be*. A debugger read that marked a
///   page accessed would be changing the thing it came to look at, and on a
///   guest that reclaims pages by their accessed bit it would change what the
///   guest then does.
/// * **No [`Access`] kind and no permission check.** The question is where the
///   page is, not whether some access to it would be allowed; a debugger has no
///   privilege level of its own. Without this, a supervisor with `SUM` clear
///   could not be asked to show a user page, which is most of what a kernel
///   debugger is for.
/// * **No PMP check on the descriptor reads.** PMP is a protection, and this
///   call is not an access.
///
/// `mode` is still needed, because whether translation happens at all is a
/// property of the privilege the hart is running at: machine mode is never
/// translated.
///
/// # Errors
///
/// A [`Fault`] meaning "nothing is mapped there", which a debugger reports
/// rather than raises.
pub fn translate_debug<M: ReadPte>(
    csrs: &Csrs,
    mem: &mut M,
    addr: u64,
    mode: Priv,
) -> Result<u64, Fault> {
    Ok(walk(csrs, mem, addr, None, mode)?.phys)
}

/// The Sv32/Sv39 walk itself, shared by both entry points.
///
/// `kind` is `None` for a debugger's walk: no permission check, no PMP on the
/// descriptor reads, and no accessed/dirty update reported. It is private
/// precisely so that "which kind of walk is this" is never a parameter anyone
/// outside this module has to remember to pass — the two public functions above
/// are the whole surface.
fn walk<M: ReadPte>(
    csrs: &Csrs,
    mem: &mut M,
    addr: u64,
    kind: Option<Access>,
    mode: Priv,
) -> Result<Walked, Fault> {
    if !translation_active(csrs, mode) {
        return Ok(Walked {
            phys: addr,
            update: None,
        });
    }
    let s = scheme(csrs.xlen);
    // Sv39 requires bits 63:39 of the virtual address to be a sign extension
    // of bit 38. An address that is not is not merely unmapped, it is
    // malformed, and faults without a walk.
    if csrs.xlen == Xlen::Rv64 {
        let shift = 64 - s.va_bits;
        if ((addr as i64) << shift) >> shift != addr as i64 {
            return Err(Fault::Page);
        }
    }

    let mut table = root(csrs);
    let mut level = s.levels;
    loop {
        level -= 1;
        let shift = PAGE_BITS + s.vpn_bits * level;
        let index = (addr >> shift) & ((1 << s.vpn_bits) - 1);
        let entry_addr = table + index * u64::from(s.pte_bytes);
        // Every page-table read is itself a physical access and is subject to
        // PMP; a walk that reads outside the permitted region is an access
        // fault, not a page fault. A debugger's walk is not an access, so PMP —
        // a protection — does not apply to it.
        if kind.is_some()
            && !pmp_allows(csrs, entry_addr, u64::from(s.pte_bytes), Access::Load, mode)
        {
            return Err(Fault::Access);
        }
        let pte = mem.read_pte(entry_addr, s.pte_bytes).ok_or(Fault::Access)?;

        if pte & pte::V == 0 || (pte & pte::R == 0 && pte & pte::W != 0) {
            // Invalid, or the reserved write-without-read encoding.
            return Err(Fault::Page);
        }
        // A PTE's `PPN` is the same width as `satp`'s, and everything above it
        // is reserved: bits 63:54 of an Sv39 PTE belong to `N` (Svnapot) and
        // `PBMT` (Svpbmt), neither of which this core implements. Volume II
        // says a guest must leave a reserved bit zero and that setting one
        // raises a page fault, so this refuses rather than translating with a
        // physical address that has a reserved bit folded into it.
        if csrs.xlen == Xlen::Rv64 && pte >> (10 + s.ppn_bits) != 0 {
            return Err(Fault::Page);
        }
        let ppn = (pte >> 10) & mask(s.ppn_bits);
        if pte & (pte::R | pte::X) == 0 {
            // A pointer to the next level down.
            if level == 0 {
                return Err(Fault::Page);
            }
            table = ppn << PAGE_BITS;
            continue;
        }

        // A leaf. Check the permissions before anything else, so a
        // permission failure never sets the accessed bit. A debugger's walk
        // skips this entirely: it asked where the page is.
        if let Some(kind) = kind
            && !permitted(csrs, pte, kind, mode)
        {
            return Err(Fault::Page);
        }
        // A superpage whose low physical page-number bits are not zero is
        // misaligned and faults.
        if level > 0 && ppn & ((1 << (s.vpn_bits * level)) - 1) != 0 {
            return Err(Fault::Page);
        }

        // Which accessed/dirty bits the leaf is missing, reported rather than
        // written: the walk stays pure and the caller decides. A debugger's
        // walk reports none, and could not perform one anyway — `mem` is
        // [`ReadPte`].
        let update = kind.and_then(|kind| {
            let need = pte::A | if kind == Access::Store { pte::D } else { 0 };
            if pte & need == need {
                None
            } else {
                Some((entry_addr, s.pte_bytes, pte | need))
            }
        });

        // Assemble the physical address: the untranslated low bits of the
        // virtual address for the levels the superpage spans, then the PTE's
        // page number above.
        let low_bits = PAGE_BITS + s.vpn_bits * level;
        let phys = (ppn << PAGE_BITS) & !((1u64 << low_bits) - 1);
        return Ok(Walked {
            phys: phys | (addr & ((1u64 << low_bits) - 1)),
            update,
        });
    }
}

/// Whether a leaf PTE permits this access from this privilege.
fn permitted(csrs: &Csrs, pte: u64, kind: Access, mode: Priv) -> bool {
    let user_page = pte & pte::U != 0;
    match mode {
        Priv::User => {
            if !user_page {
                return false;
            }
        }
        Priv::Supervisor => {
            if user_page {
                // A supervisor may never *execute* from a user page, and may
                // only read or write one when SUM permits it.
                if kind == Access::Fetch || csrs.mstatus & status::SUM == 0 {
                    return false;
                }
            }
        }
        // Machine mode does not translate, so it never reaches this function.
        Priv::Machine => {}
    }
    match kind {
        Access::Fetch => pte & pte::X != 0,
        // MXR makes an execute-only page readable, which is how a kernel
        // inspects code it has mapped without execute-and-read permission.
        Access::Load => pte & pte::R != 0 || (csrs.mstatus & status::MXR != 0 && pte & pte::X != 0),
        Access::Store => pte & pte::W != 0,
    }
}

/// Whether physical memory protection permits an access.
///
/// Volume II, "Physical Memory Protection": entries are matched in order and
/// the **first** match decides, whether it grants or refuses. An M-mode access
/// that matches an unlocked entry is permitted regardless of the entry's
/// permission bits; a locked entry constrains M-mode too, which is what makes
/// the lock bit useful. An S-mode or U-mode access that matches nothing is
/// refused, because at least one entry is implemented.
#[must_use]
pub fn pmp_allows(csrs: &Csrs, addr: u64, len: u64, kind: Access, mode: Priv) -> bool {
    let last = addr.wrapping_add(len.saturating_sub(1));
    let mut matched = None;
    for i in 0..csrs.pmp_count {
        let cfg = csrs.pmpcfg[i];
        let Some((lo, hi)) = pmp_range(csrs, i) else {
            continue;
        };
        // An access that straddles the edge of a region is refused rather than
        // split: the specification requires the whole access to match one
        // entry.
        if addr >= lo && addr < hi {
            if last >= hi {
                return false;
            }
            matched = Some(cfg);
            break;
        }
        if last >= lo && last < hi {
            return false;
        }
    }
    match matched {
        Some(cfg) => {
            let locked = cfg & 0x80 != 0;
            if mode == Priv::Machine && !locked {
                return true;
            }
            let bit = match kind {
                Access::Load => 0b001,
                Access::Store => 0b010,
                Access::Fetch => 0b100,
            };
            cfg & bit != 0
        }
        // No entry matched: machine mode may do anything, and so may everyone
        // else if PMP is not implemented at all.
        None => mode == Priv::Machine || csrs.pmp_count == 0,
    }
}

/// Entry `i`'s half-open physical range, or `None` when it matches nothing.
///
/// `A = OFF` matches nothing, and so does a TOR entry whose top is not above
/// its bottom. Written once because [`pmp_allows`] and [`pmp_page_uniform`]
/// must agree about every edge: the second exists to say that the first gives
/// one answer for a whole page, and two copies of the decoding are two
/// opportunities for it to be wrong about that.
fn pmp_range(csrs: &Csrs, i: usize) -> Option<(u64, u64)> {
    let cfg = csrs.pmpcfg[i];
    let (lo, hi) = match (cfg >> 3) & 3 {
        0 => return None,
        // TOR: the previous entry's address is the bottom of the range.
        1 => {
            let lo = if i == 0 { 0 } else { csrs.pmpaddr[i - 1] << 2 };
            (lo, csrs.pmpaddr[i] << 2)
        }
        2 => {
            let base = csrs.pmpaddr[i] << 2;
            (base, base + 4)
        }
        _ => napot(csrs.pmpaddr[i]),
    };
    (hi > lo).then_some((lo, hi))
}

/// Whether PMP gives the *same* answer for every access inside one page.
///
/// [`pmp_allows`] asked about a whole page is not this question, and the
/// difference is a silent wrong answer rather than a slow one. Its loop takes
/// the **first** entry that contains the access's start, and an entry that
/// lies wholly *inside* the page contains neither the page's first byte nor
/// its last — so it is skipped for the page and matched for an access in the
/// middle of it. A page-wide grant can therefore sit in front of a byte-wide
/// refusal.
///
/// So the condition here is stronger: no active entry may **partially** overlap
/// the page. With that, every access inside the page reaches the same entry the
/// page-wide question reached, and its answer is the page-wide answer.
///
/// This is what a caller needs before it may cache "this page is fast" —
/// [`Tlb`]'s shadow does, because the compiled fast path skips the PMP check
/// entirely and a page PMP is not uniform over cannot be cached at all.
#[must_use]
pub fn pmp_page_uniform(csrs: &Csrs, page: u64, kind: Access, mode: Priv) -> bool {
    let end = match page.checked_add(PAGE_SIZE) {
        Some(end) => end,
        None => return false,
    };
    for i in 0..csrs.pmp_count {
        let Some((lo, hi)) = pmp_range(csrs, i) else {
            continue;
        };
        // Disjoint is fine; containing the page is fine; anything else is an
        // edge inside the page.
        if hi <= page || lo >= end {
            continue;
        }
        if lo > page || hi < end {
            return false;
        }
    }
    pmp_allows(csrs, page, PAGE_SIZE, kind, mode)
}

/// Decode a NAPOT `pmpaddr` into a half-open physical range.
///
/// The encoding is a run of low ones marking the size: `yyyy0` is 8 bytes,
/// `yyy01` is 16, and so on, with an all-ones register covering everything.
fn napot(addr: u64) -> (u64, u64) {
    let ones = (!addr).trailing_zeros();
    if ones >= 62 {
        return (0, u64::MAX);
    }
    let size_bits = ones + 3;
    let base = (addr & !((1u64 << ones) - 1)) << 2;
    (base, base + (1u64 << size_bits))
}

/// How many entries each half of the TLB holds.
///
/// Direct-mapped and a power of two, so a lookup is a mask and a compare —
/// `ROADMAP.md` §9's fast path is "mask, compare, add" and this is the
/// interpreter's version of it.
pub const TLB_ENTRIES: usize = 256;

/// One cached translation.
#[derive(Debug, Clone, Copy, Default)]
struct Entry {
    /// The tag: virtual page number, ASID, privilege and generation, so a
    /// stale entry can never be mistaken for a hit.
    tag: u64,
    /// The physical address of the page this maps to.
    base: u64,
    /// Whether this slot holds anything.
    valid: bool,
}

/// The per-hart software TLB.
///
/// Derived state in the strict sense of `ROADMAP.md` §4.5: never serialized,
/// and safe to throw away at any moment.
#[derive(Debug)]
pub struct Tlb {
    slots: [[Entry; TLB_ENTRIES]; 3],
    hits: u64,
    misses: u64,
    /// The host half of the same answers, for a code generator that inlines a
    /// load. See [`Tlb::attach_shadow`].
    #[cfg(feature = "jit")]
    shadow: Option<alloc::boxed::Box<crate::jit::Tlb>>,
}

impl Default for Tlb {
    fn default() -> Self {
        Tlb::new()
    }
}

impl Tlb {
    /// An empty TLB.
    #[must_use]
    pub fn new() -> Tlb {
        Tlb {
            slots: [[Entry::default(); TLB_ENTRIES]; 3],
            hits: 0,
            misses: 0,
            #[cfg(feature = "jit")]
            shadow: None,
        }
    }

    /// Throw everything away.
    ///
    /// Cheaper than it looks and used rarely: the generation counter in the
    /// tag already invalidates entries logically, so this exists for reset and
    /// for a snapshot restore.
    pub fn flush(&mut self) {
        self.slots = [[Entry::default(); TLB_ENTRIES]; 3];
        // The shadow is only ever as live as this table is, so it goes with
        // it. Losing it costs a refill; keeping it after a flush would let a
        // compiled load skip a walk this table would have charged for.
        #[cfg(feature = "jit")]
        if let Some(shadow) = self.shadow.as_mut() {
            shadow.flush();
        }
    }

    /// Give this TLB a [`jit::Tlb`](crate::jit::Tlb) shadow over `space`.
    ///
    /// # What the shadow is
    ///
    /// This table answers *virtual page → physical page*, which is half of what
    /// a compiled load needs; the other half is *physical page → host address*,
    /// and that is what `jit::Tlb` caches. The shadow is that second half,
    /// indexed by the **same** virtual page, in the same slot, so that a
    /// compiled load can go from a guest address to a host one in a mask, a
    /// compare and an add — `ROADMAP.md` §9.1's first mechanism, inlined.
    ///
    /// # Why it has to be here rather than beside the engine
    ///
    /// A compiled load that hits the shadow charges **one** tick and skips the
    /// walk. That is only right if this table would have hit too, so the two
    /// have to stay in lockstep — and lockstep is a property of *every* path
    /// that can insert here, not only of the translated one. An interpreted
    /// `amoadd` inserts, a trap handler outside the lifted subset inserts, a
    /// debugger's single step inserts; each of those evicts a slot, and a
    /// shadow living next to the engine would not hear about any of them.
    /// Owning it here means `Exec::translate` maintains both at once and there
    /// is no other way in.
    ///
    /// The two are the same size for the same reason: the slot a page lands in
    /// must be the same slot in both, or an eviction here would leave a shadow
    /// entry alive that promises a hit this table no longer has.
    #[cfg(feature = "jit")]
    pub fn attach_shadow(&mut self, space: alloc::sync::Arc<crate::core::space::AddressSpace>) {
        let shadow = crate::jit::Tlb::with_entries(space, TLB_ENTRIES as u64);
        debug_assert_eq!(
            shadow.entries(),
            TLB_ENTRIES as u64,
            "the shadow must index exactly as this table does"
        );
        self.shadow = Some(alloc::boxed::Box::new(shadow));
    }

    /// The shadow, if one was attached.
    #[cfg(feature = "jit")]
    #[inline]
    pub fn shadow_mut(&mut self) -> Option<&mut crate::jit::Tlb> {
        self.shadow.as_deref_mut()
    }

    /// Whether a shadow is attached.
    #[cfg(feature = "jit")]
    #[inline]
    #[must_use]
    pub fn has_shadow(&self) -> bool {
        self.shadow.is_some()
    }

    /// How many lookups hit and how many missed, for `rsemu` statistics.
    #[must_use]
    pub fn stats(&self) -> (u64, u64) {
        (self.hits, self.misses)
    }

    /// The tag for a page.
    #[inline]
    fn tag(vpn: u64, asid: u64, mode: Priv, generation: u64) -> u64 {
        // The generation goes in the high bits so a bump invalidates every
        // entry at once without touching them.
        (generation << 40) ^ (vpn.wrapping_mul(0x9e37_79b9_7f4a_7c15)) ^ (asid << 2) ^ mode.bits()
    }

    /// Look a page up.
    #[inline]
    pub fn lookup(
        &mut self,
        kind: Access,
        vpn: u64,
        asid: u64,
        mode: Priv,
        generation: u64,
    ) -> Option<u64> {
        let tag = Self::tag(vpn, asid, mode, generation);
        let slot = &self.slots[kind.slot()][(vpn as usize) & (TLB_ENTRIES - 1)];
        if slot.valid && slot.tag == tag {
            self.hits += 1;
            Some(slot.base)
        } else {
            self.misses += 1;
            None
        }
    }

    /// Record a successful translation.
    #[inline]
    pub fn insert(
        &mut self,
        kind: Access,
        vpn: u64,
        asid: u64,
        mode: Priv,
        generation: u64,
        base: u64,
    ) {
        self.slots[kind.slot()][(vpn as usize) & (TLB_ENTRIES - 1)] = Entry {
            tag: Self::tag(vpn, asid, mode, generation),
            base,
            valid: true,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::super::csr::{Extensions, PMP_ENTRIES, num};
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    /// A flat physical memory for the walker to read tables out of.
    struct Ram(Vec<u8>);

    impl ReadPte for Ram {
        fn read_pte(&mut self, addr: u64, bytes: u32) -> Option<u64> {
            let at = addr as usize;
            let end = at + bytes as usize;
            let slice = self.0.get(at..end)?;
            let mut v = 0u64;
            for (i, b) in slice.iter().enumerate() {
                v |= u64::from(*b) << (8 * i);
            }
            Some(v)
        }
    }

    impl PhysMem for Ram {
        fn write_pte(&mut self, addr: u64, bytes: u32, value: u64) -> Option<()> {
            let at = addr as usize;
            for i in 0..bytes as usize {
                *self.0.get_mut(at + i)? = (value >> (8 * i)) as u8;
            }
            Some(())
        }
    }

    /// An Sv39 hierarchy mapping one 4 KiB page, with the root at 0x1000.
    fn sv39_machine(perms: u64) -> (Csrs, Ram) {
        let mut ram = Ram(vec![0; 0x8000]);
        // Level 2 entry at 0x1000 points at 0x2000; level 1 at 0x2000 points
        // at 0x3000; level 0 at 0x3000 is the leaf for physical 0x4000.
        ram.write_pte(0x1000, 8, ((0x2000 >> 12) << 10) | pte::V)
            .unwrap();
        ram.write_pte(0x2000, 8, ((0x3000 >> 12) << 10) | pte::V)
            .unwrap();
        ram.write_pte(0x3000, 8, ((0x4000 >> 12) << 10) | pte::V | perms)
            .unwrap();
        // PMP is left unimplemented in these fixtures so a walk exercises the
        // page tables and nothing else; the PMP tests below build their own.
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, 0);
        csrs.satp = (8 << 60) | (0x1000 >> 12);
        csrs.priv_mode = Priv::Supervisor;
        (csrs, ram)
    }

    #[test]
    fn machine_mode_is_never_translated() {
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W | pte::A | pte::D);
        assert_eq!(
            translate(&csrs, &mut ram, 0x1234, Access::Load, Priv::Machine),
            Ok(0x1234)
        );
    }

    #[test]
    fn a_three_level_walk_finds_the_leaf() {
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W | pte::A | pte::D);
        assert_eq!(
            translate(&csrs, &mut ram, 0x0123, Access::Load, Priv::Supervisor),
            Ok(0x4123)
        );
    }

    #[test]
    fn permissions_are_enforced_per_access_type() {
        let (csrs, mut ram) = sv39_machine(pte::R | pte::A);
        assert!(translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor).is_ok());
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Store, Priv::Supervisor),
            Err(Fault::Page)
        );
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Fetch, Priv::Supervisor),
            Err(Fault::Page)
        );
    }

    #[test]
    fn the_user_bit_and_sum_decide_supervisor_access() {
        let (mut csrs, mut ram) = sv39_machine(pte::R | pte::U | pte::A);
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor),
            Err(Fault::Page),
            "a supervisor needs SUM to read a user page"
        );
        csrs.mstatus |= status::SUM;
        assert!(translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor).is_ok());
        // SUM never permits execution from a user page.
        let (mut csrs, mut ram) = sv39_machine(pte::X | pte::U | pte::A);
        csrs.mstatus |= status::SUM;
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Fetch, Priv::Supervisor),
            Err(Fault::Page)
        );
        // And a user may not touch a supervisor page.
        let (csrs, mut ram) = sv39_machine(pte::R | pte::A);
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Load, Priv::User),
            Err(Fault::Page)
        );
    }

    #[test]
    fn mxr_makes_an_execute_only_page_readable() {
        let (mut csrs, mut ram) = sv39_machine(pte::X | pte::A);
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor),
            Err(Fault::Page)
        );
        csrs.mstatus |= status::MXR;
        assert!(translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor).is_ok());
    }

    #[test]
    fn the_accessed_and_dirty_bits_are_set_by_the_walk() {
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W);
        translate(&csrs, &mut ram, 0, Access::Store, Priv::Supervisor).unwrap();
        let leaf = ram.read_pte(0x3000, 8).unwrap();
        assert_ne!(leaf & pte::A, 0);
        assert_ne!(leaf & pte::D, 0);
        // A load sets A but not D.
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W);
        translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor).unwrap();
        let leaf = ram.read_pte(0x3000, 8).unwrap();
        assert_ne!(leaf & pte::A, 0);
        assert_eq!(leaf & pte::D, 0);
    }

    #[test]
    fn a_debug_walk_sets_no_accessed_or_dirty_bit() {
        // The direct statement of the no-side-effects rule: the same tables,
        // the same leaf, and afterwards the PTE is byte-for-byte what it was.
        // `translate` on the line below would set A — that is the assertion
        // above this one — so a regression here is a debugger changing the
        // guest's page-reclaim decisions by looking at memory.
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W);
        let before = ram.read_pte(0x3000, 8).unwrap();
        assert_eq!(before & (pte::A | pte::D), 0, "the fixture starts clean");
        assert_eq!(
            translate_debug(&csrs, &mut ram, 0, Priv::Supervisor),
            Ok(0x4000)
        );
        assert_eq!(
            ram.read_pte(0x3000, 8).unwrap(),
            before,
            "a debug walk wrote to the page table"
        );
    }

    #[test]
    fn a_debug_walk_resolves_a_page_the_permissions_would_hide() {
        // A user page with `SUM` clear: a supervisor load faults, and a
        // debugger still gets to see where the page is. A debugger has no
        // privilege level of its own — it asked "where", not "may I".
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W | pte::U | pte::A | pte::D);
        assert_eq!(
            translate(&csrs, &mut ram, 0x40, Access::Load, Priv::Supervisor),
            Err(Fault::Page)
        );
        assert_eq!(
            translate_debug(&csrs, &mut ram, 0x40, Priv::Supervisor),
            Ok(0x4040)
        );
    }

    #[test]
    fn a_debug_walk_of_an_unmapped_address_still_faults() {
        // Permission-free is not fault-free: nothing is mapped in the second
        // gigabyte of the fixture, and a listing that runs into it has to be
        // told so rather than handed a plausible number.
        let (csrs, mut ram) = sv39_machine(pte::R | pte::W | pte::A | pte::D);
        assert_eq!(
            translate_debug(&csrs, &mut ram, 0x4000_0000, Priv::Supervisor),
            Err(Fault::Page)
        );
    }

    #[test]
    fn a_non_canonical_sv39_address_faults_without_a_walk() {
        let (csrs, mut ram) = sv39_machine(pte::R | pte::A);
        assert_eq!(
            translate(
                &csrs,
                &mut ram,
                0x0000_8000_0000_0000,
                Access::Load,
                Priv::Supervisor
            ),
            Err(Fault::Page)
        );
        // The top of the address space is canonical and merely unmapped.
        assert_eq!(
            translate(&csrs, &mut ram, !0xfffu64, Access::Load, Priv::Supervisor),
            Err(Fault::Page)
        );
    }

    #[test]
    fn a_misaligned_superpage_faults() {
        let mut ram = Ram(vec![0; 0x8000]);
        // A level-1 leaf (2 MiB superpage) whose PPN[0] is not zero.
        ram.write_pte(0x1000, 8, ((0x2000 >> 12) << 10) | pte::V)
            .unwrap();
        ram.write_pte(0x2000, 8, ((0x4001) << 10) | pte::V | pte::R | pte::A)
            .unwrap();
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, 0);
        csrs.satp = (8 << 60) | (0x1000 >> 12);
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor),
            Err(Fault::Page)
        );
    }

    #[test]
    fn a_superpage_carries_the_low_virtual_bits_through() {
        let mut ram = Ram(vec![0; 0x8000]);
        ram.write_pte(0x1000, 8, ((0x2000 >> 12) << 10) | pte::V)
            .unwrap();
        // A 2 MiB superpage at physical 0x40_0000.
        ram.write_pte(
            0x2000,
            8,
            ((0x40_0000u64 >> 12) << 10) | pte::V | pte::R | pte::A,
        )
        .unwrap();
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, 0);
        csrs.satp = (8 << 60) | (0x1000 >> 12);
        assert_eq!(
            translate(&csrs, &mut ram, 0x1_2345, Access::Load, Priv::Supervisor),
            Ok(0x41_2345)
        );
    }

    #[test]
    fn the_reserved_write_without_read_encoding_faults() {
        let (_, mut ram) = sv39_machine(0);
        ram.write_pte(0x3000, 8, ((0x4000 >> 12) << 10) | pte::V | pte::W)
            .unwrap();
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, 0);
        csrs.satp = (8 << 60) | (0x1000 >> 12);
        assert_eq!(
            translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor),
            Err(Fault::Page)
        );
    }

    #[test]
    fn pmp_lets_machine_mode_through_when_nothing_is_configured() {
        let csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        assert!(pmp_allows(
            &csrs,
            0x8000_0000,
            4,
            Access::Load,
            Priv::Machine
        ));
        assert!(
            !pmp_allows(&csrs, 0x8000_0000, 4, Access::Load, Priv::Supervisor),
            "an unmatched supervisor access is refused"
        );
    }

    #[test]
    fn a_napot_entry_covers_its_declared_range() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        // A 16-byte NAPOT region at physical 0x1000: address = 0x1000>>2 with
        // one trailing one.
        csrs.pmpaddr[0] = (0x1000 >> 2) | 1;
        csrs.pmpcfg[0] = 0b0001_1001; // A = NAPOT, R
        assert!(pmp_allows(&csrs, 0x1000, 4, Access::Load, Priv::Supervisor));
        assert!(pmp_allows(&csrs, 0x100c, 4, Access::Load, Priv::Supervisor));
        assert!(!pmp_allows(
            &csrs,
            0x1010,
            4,
            Access::Load,
            Priv::Supervisor
        ));
        assert!(!pmp_allows(
            &csrs,
            0x1000,
            4,
            Access::Store,
            Priv::Supervisor
        ));
        // An access that straddles the top edge is refused whole.
        assert!(!pmp_allows(
            &csrs,
            0x100e,
            4,
            Access::Load,
            Priv::Supervisor
        ));
    }

    #[test]
    fn the_all_ones_napot_entry_covers_everything() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        csrs.pmpaddr[0] = u64::MAX >> 10;
        csrs.pmpcfg[0] = 0b0001_1111;
        for mode in [Priv::User, Priv::Supervisor, Priv::Machine] {
            assert!(pmp_allows(&csrs, 0x8000_0000, 8, Access::Store, mode));
        }
    }

    #[test]
    fn a_tor_entry_uses_the_previous_address_as_its_base() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        csrs.pmpaddr[0] = 0x1000 >> 2;
        csrs.pmpaddr[1] = 0x2000 >> 2;
        csrs.pmpcfg[1] = 0b0000_1001; // A = TOR, R
        assert!(!pmp_allows(&csrs, 0x0fff, 1, Access::Load, Priv::User));
        assert!(pmp_allows(&csrs, 0x1000, 1, Access::Load, Priv::User));
        assert!(!pmp_allows(&csrs, 0x2000, 1, Access::Load, Priv::User));
    }

    #[test]
    fn a_locked_entry_constrains_machine_mode_too() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        csrs.pmpaddr[0] = u64::MAX >> 10;
        csrs.pmpcfg[0] = 0x80 | 0b0001_1001; // locked, NAPOT, read-only
        assert!(pmp_allows(&csrs, 0x1000, 4, Access::Load, Priv::Machine));
        assert!(!pmp_allows(&csrs, 0x1000, 4, Access::Store, Priv::Machine));
    }

    #[test]
    fn a_page_with_an_entry_inside_it_is_not_uniform_even_when_the_page_is_allowed() {
        // The exact hazard `pmp_page_uniform` exists for, and the reason
        // asking `pmp_allows` about the whole page is not the same question.
        // An entry lying *wholly inside* the page contains neither its first
        // byte nor its last, so `pmp_allows`'s loop skips it for the page and
        // matches it for an access in the middle — a page-wide grant sitting
        // in front of a byte-wide refusal.
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        // Entry 0: sixteen bytes at 0x8000_1010 — strictly inside the page,
        // touching neither its first byte nor its last — with no permissions.
        csrs.pmpaddr[0] = (0x8000_1010u64 >> 2) | 1;
        csrs.pmpcfg[0] = 0b0001_1000; // NAPOT, no R/W/X
        // Entry 1: everything, readable.
        csrs.pmpaddr[1] = u64::MAX >> 10;
        csrs.pmpcfg[1] = 0b0001_1001; // NAPOT, R
        let page = 0x8000_1000;
        assert!(
            pmp_allows(&csrs, page, PAGE_SIZE, Access::Load, Priv::Supervisor),
            "the page-wide question falls through to the entry that grants"
        );
        assert!(
            !pmp_allows(&csrs, page + 0x10, 4, Access::Load, Priv::Supervisor),
            "but an access inside it reaches the entry that refuses"
        );
        assert!(
            !pmp_page_uniform(&csrs, page, Access::Load, Priv::Supervisor),
            "so the page must never be cached as fast"
        );
    }

    #[test]
    fn a_page_an_entry_only_half_covers_is_not_uniform_either() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        // A TOR entry ending in the middle of the page: readable below, and
        // nothing matches above.
        csrs.pmpaddr[0] = (0x8000_1800u64) >> 2;
        csrs.pmpcfg[0] = 0b0000_1001; // TOR, R
        assert!(!pmp_page_uniform(
            &csrs,
            0x8000_1000,
            Access::Load,
            Priv::Supervisor
        ));
        // The page below it is wholly inside, so that one is uniform.
        assert!(pmp_page_uniform(
            &csrs,
            0x8000_0000,
            Access::Load,
            Priv::Supervisor
        ));
    }

    #[test]
    fn a_uniform_page_answers_the_same_way_everywhere_in_it() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        csrs.pmpaddr[0] = u64::MAX >> 10;
        csrs.pmpcfg[0] = 0b0001_1001; // NAPOT over everything, read-only
        let page = 0x8000_2000;
        assert!(pmp_page_uniform(
            &csrs,
            page,
            Access::Load,
            Priv::Supervisor
        ));
        for off in [0, 8, PAGE_SIZE - 8] {
            assert!(pmp_allows(
                &csrs,
                page + off,
                8,
                Access::Load,
                Priv::Supervisor
            ));
        }
        // Uniform is not the same as allowed: a store is refused everywhere in
        // the same page, and the shadow must not cache that as fast either.
        assert!(!pmp_page_uniform(
            &csrs,
            page,
            Access::Store,
            Priv::Supervisor
        ));
    }

    #[test]
    fn an_entry_that_only_touches_a_page_does_not_make_it_non_uniform() {
        // The other edge of the same test, and the one a *conservative*
        // mistake hides in: an entry that stops exactly where the page starts,
        // or starts exactly where it ends, overlaps nothing. Refusing those
        // costs no correctness and is therefore invisible to every agreement
        // test — it just quietly turns the fast path off for the page next to
        // every PMP region a firmware configures, which on this board is the
        // page next to everything OpenSBI locks down.
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        let page = 0x8000_1000;
        // Entry 0: TOR ending exactly at the page's first byte.
        csrs.pmpaddr[0] = page >> 2;
        csrs.pmpcfg[0] = 0b0000_1001; // TOR, R
        // Entry 1: NA4 at exactly the page's end, so it starts where the page
        // stops.
        csrs.pmpaddr[1] = (page + PAGE_SIZE) >> 2;
        csrs.pmpcfg[1] = 0b0001_0001; // NA4, R
        // Entry 2: everything, so the page itself has a granting match.
        csrs.pmpaddr[2] = u64::MAX >> 10;
        csrs.pmpcfg[2] = 0b0001_1001; // NAPOT, R
        assert!(
            pmp_page_uniform(&csrs, page, Access::Load, Priv::Supervisor),
            "an adjacent entry is not an overlapping one"
        );
    }

    #[test]
    fn an_empty_pmp_range_is_no_range_at_all() {
        // A TOR entry whose top is not above its bottom matches nothing —
        // `pmp_allows` has always skipped it. `pmp_page_uniform` has to skip
        // it too, or an empty range that happens to sit inside a page turns
        // the fast path off for it forever, silently and for no reason.
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        let page = 0x8000_1000;
        // A TOR entry at the same address as the one below it: an empty range,
        // strictly inside the page.
        csrs.pmpaddr[0] = (page + 0x100) >> 2;
        csrs.pmpcfg[0] = 0; // A = OFF, so entry 1's bottom is entry 0's address
        csrs.pmpaddr[1] = (page + 0x100) >> 2;
        csrs.pmpcfg[1] = 0b0000_1001; // TOR, R -- range is [x, x), empty
        csrs.pmpaddr[2] = u64::MAX >> 10;
        csrs.pmpcfg[2] = 0b0001_1001; // NAPOT over everything, R
        assert_eq!(pmp_range(&csrs, 1), None, "an empty range matches nothing");
        assert!(
            pmp_page_uniform(&csrs, page, Access::Load, Priv::Supervisor),
            "and so cannot be the thing that makes a page non-uniform"
        );
    }

    #[test]
    fn a_hart_with_no_pmp_entries_calls_every_page_uniform() {
        let csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, 0);
        assert!(pmp_page_uniform(&csrs, 0, Access::Load, Priv::Supervisor));
        // And a hart with entries but none configured refuses S-mode
        // everywhere — uniformly, but as a refusal.
        let csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        assert!(!pmp_page_uniform(&csrs, 0, Access::Load, Priv::Supervisor));
        assert!(pmp_page_uniform(&csrs, 0, Access::Load, Priv::Machine));
    }

    #[cfg(feature = "jit")]
    #[test]
    fn the_shadow_indexes_exactly_as_this_table_does() {
        // The lockstep the compiled fast path's single tick rests on: a page
        // must land in the same slot in both, or an eviction here leaves a
        // shadow entry alive that promises a hit this table no longer has.
        use crate::core::space::AddressSpace;
        use alloc::sync::Arc;
        let space = Arc::new(AddressSpace::new("mem", 64));
        let mut tlb = Tlb::new();
        assert!(!tlb.has_shadow());
        tlb.attach_shadow(space);
        let shadow = tlb.shadow_mut().expect("just attached");
        assert_eq!(shadow.entries(), TLB_ENTRIES as u64);
        // And the index is the same function of the page in both.
        assert_eq!(crate::jit::PAGE_SIZE, PAGE_SIZE);
    }

    #[cfg(feature = "jit")]
    #[test]
    fn a_flush_of_this_table_empties_the_shadow_with_it() {
        // A flush is not a generation bump: `Tlb::flush` is what a reset, a
        // debugger's CSR write and a snapshot restore do, and none of them has
        // to move `translation_gen`. So a shadow entry that outlived one would
        // still *match* its tag while the entry it stands for is gone — a
        // compiled load charging one tick where the interpreter walks, and, if
        // the restored `satp` maps the page somewhere else, reading the wrong
        // physical page outright.
        use crate::core::space::{AddressSpace, RamStore, Region};
        use crate::ir::AccessKind;
        use alloc::sync::Arc;
        let ram = Arc::new(RamStore::new(0x4000));
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map(Region::ram("ram", ram), 0)
            .expect("nothing else is mapped");
        let mut tlb = Tlb::new();
        tlb.attach_shadow(Arc::new(space));
        let ctx = crate::jit::Context {
            level: Priv::Supervisor.bits() as u8,
            translating: true,
        };
        let shadow = tlb.shadow_mut().expect("just attached");
        shadow.fill(AccessKind::Load, 0, 0, ctx);
        assert!(
            shadow.caches(AccessKind::Load, 0, ctx),
            "the fixture never cached the page it is about to flush"
        );
        tlb.flush();
        let shadow = tlb.shadow_mut().expect("the shadow survives a flush");
        assert!(
            !shadow.caches(AccessKind::Load, 0, ctx),
            "its contents did not"
        );
    }

    #[test]
    fn the_tlb_hits_only_on_an_exact_tag() {
        let mut tlb = Tlb::new();
        tlb.insert(Access::Load, 0x1234, 7, Priv::Supervisor, 3, 0x4000);
        assert_eq!(
            tlb.lookup(Access::Load, 0x1234, 7, Priv::Supervisor, 3),
            Some(0x4000)
        );
        // A different access type, ASID, privilege or generation all miss.
        assert_eq!(
            tlb.lookup(Access::Store, 0x1234, 7, Priv::Supervisor, 3),
            None
        );
        assert_eq!(
            tlb.lookup(Access::Load, 0x1234, 8, Priv::Supervisor, 3),
            None
        );
        assert_eq!(tlb.lookup(Access::Load, 0x1234, 7, Priv::User, 3), None);
        assert_eq!(
            tlb.lookup(Access::Load, 0x1234, 7, Priv::Supervisor, 4),
            None
        );
        tlb.flush();
        assert_eq!(
            tlb.lookup(Access::Load, 0x1234, 7, Priv::Supervisor, 3),
            None
        );
    }

    #[test]
    fn a_non_zero_asid_does_not_move_the_root_page_table() {
        // How Linux discovers ASIDLEN, and the shape that found this bug:
        // write all ones to `satp.ASID`, read back which bits stuck. Volume
        // II makes `ASID` a separate field from `PPN`, so a walk under an
        // all-ones ASID must reach exactly the same leaf.
        let (mut csrs, mut ram) = sv39_machine(pte::R | pte::A);
        let bare = translate(&csrs, &mut ram, 0x0123, Access::Load, Priv::Supervisor);
        assert_eq!(bare, Ok(0x4123));
        csrs.satp |= 0xffff << 44;
        assert_eq!(root(&csrs), 0x1000, "ASID is not part of the root address");
        assert_eq!(
            translate(&csrs, &mut ram, 0x0123, Access::Load, Priv::Supervisor),
            bare,
            "an all-ones ASID must not move the page tables"
        );
    }

    #[test]
    fn every_asid_bit_sticks_and_none_of_them_reaches_the_ppn() {
        // `satp.ASID` is WARL and this core implements all of it: 16 bits
        // under Sv39, 9 under Sv32. Software probes the width by writing ones
        // and reading back, so what sticks here is what a guest believes.
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        csrs.write(num::SATP, (8 << 60) | (0xffff << 44) | 0x81fcb, 0)
            .unwrap();
        assert_eq!(asid(&csrs), 0xffff);
        assert_eq!(root(&csrs), 0x81fcb << 12);

        let mut csrs = Csrs::new(Xlen::Rv32, Extensions::GC, 0, PMP_ENTRIES);
        csrs.write(num::SATP, (1 << 31) | (0x1ff << 22) | 0x123, 0)
            .unwrap();
        assert_eq!(asid(&csrs), 0x1ff);
        assert_eq!(root(&csrs), 0x123 << 12);
    }

    #[test]
    fn a_reserved_pte_bit_faults_rather_than_moving_the_page() {
        // Bits 63:54 of an Sv39 PTE are `N` and `PBMT` and the reserved space
        // between them. This core implements neither Svnapot nor Svpbmt, and
        // Volume II says a guest that sets one of those bits takes a page
        // fault — never a translation with the bit folded into the address.
        for bit in [54, 61, 62, 63] {
            let (csrs, mut ram) = sv39_machine(pte::R | pte::A);
            let leaf = ram.read_pte(0x3000, 8).unwrap();
            ram.write_pte(0x3000, 8, leaf | (1 << bit)).unwrap();
            assert_eq!(
                translate(&csrs, &mut ram, 0, Access::Load, Priv::Supervisor),
                Err(Fault::Page),
                "bit {bit} is reserved"
            );
        }
    }

    #[test]
    fn a_satp_write_invalidates_the_whole_tlb_by_generation() {
        let mut csrs = Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES);
        let before = csrs.translation_gen;
        csrs.write(num::SATP, 8 << 60, 0).unwrap();
        assert_ne!(csrs.translation_gen, before);
    }
}
