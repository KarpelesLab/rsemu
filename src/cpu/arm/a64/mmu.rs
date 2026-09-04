//! The AArch64 stage-1 translation regime for EL1&0, and the software TLB in
//! front of it.
//!
//! This is a different machine from the VMSAv5 walker in `arm/aprofile`, not a
//! wider version of it. VMSAv5 has one table base, a two-level walk of
//! 1 MiB sections and 4 KiB pages, a *domain* per section and permission bits
//! that mean different things depending on `SCTLR.S` and `SCTLR.R`. AArch64
//! has **two** table bases splitting the address space at the top, a walk
//! whose depth is computed from `TCR_EL1.TnSZ`, 64-bit descriptors, no domains
//! at all, hierarchical permissions that accumulate down the walk, and an
//! access flag. Sharing code between the two would be sharing the word "MMU".
//!
//! # What is implemented
//!
//! The 4 KiB granule, `TTBR0_EL1`/`TTBR1_EL1`, a walk of one to four levels
//! chosen by `T0SZ`/`T1SZ`, block descriptors at levels 1 and 2, the access
//! flag, `AP[2:1]`, `UXN`/`PXN`, the hierarchical `APTable`/`UXNTable`/
//! `PXNTable` accumulation, `SCTLR_EL1.WXN`, ASIDs from whichever `TTBR`
//! `TCR_EL1.A1` names, and `EPD0`/`EPD1`.
//!
//! # What is not
//!
//! The 16 KiB and 64 KiB granules (a `TCR` selecting one faults rather than
//! silently walking as if it were 4 KiB), stage 2, `FEAT_LPA`/`FEAT_LPA2`,
//! hardware access-flag and dirty-bit update (`FEAT_HAFDBS` — software must
//! set `AF` itself, which every Armv8.0 guest already does), the contiguous
//! hint, `FEAT_PAN`/`FEAT_UAO`, `FEAT_HPDS` hierarchical-permission disable,
//! and big-endian descriptors. Each of those is a fault or a documented
//! no-op, never a guess.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487),
//! chapter D8 "The AArch64 Virtual Memory System Architecture": the
//! translation-table walk, the VMSAv8-64 descriptor formats, the access
//! permissions and the fault encodings `ESR_ELx.DFSC` uses.

use super::sysreg::{El, SysRegs, sctlr};

/// Which kind of access is being translated.
///
/// The three differ in permission checking and in which abort a failure
/// raises, which is why one enum carries both jobs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// An instruction fetch.
    Fetch,
    /// A data read.
    Load,
    /// A data write.
    Store,
}

impl Access {
    /// Which of the TLB's three sets this access uses.
    ///
    /// Separate sets rather than one set plus a permission re-check, because a
    /// hit must be usable without consulting the descriptor again.
    #[inline]
    #[must_use]
    pub const fn slot(self) -> usize {
        match self {
            Access::Fetch => 0,
            Access::Load => 1,
            Access::Store => 2,
        }
    }

    /// Whether this access writes, which `ESR_ELx.ISS.WnR` reports.
    #[inline]
    #[must_use]
    pub const fn is_write(self) -> bool {
        matches!(self, Access::Store)
    }
}

/// Why a translation failed.
///
/// The level is part of the fault because `ESR_ELx.DFSC` reports it, and a
/// guest's page-fault handler branches on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// No valid descriptor at this level.
    Translation(u32),
    /// A valid descriptor whose access flag is clear.
    AccessFlag(u32),
    /// A valid descriptor that does not permit this access.
    Permission(u32),
    /// An output address wider than the implemented physical address size, or
    /// a table base that is not aligned to its own size.
    AddressSize(u32),
    /// The access was not aligned and alignment checking is on.
    Alignment,
    /// The bus refused an access made during the walk, or the access itself.
    External,
}

impl Fault {
    /// The `ESR_ELx.DFSC`/`IFSC` encoding (DDI 0487 D17.2.37).
    #[must_use]
    pub const fn dfsc(self) -> u64 {
        match self {
            Fault::AddressSize(level) => (level & 3) as u64,
            Fault::Translation(level) => 0b000100 | (level & 3) as u64,
            Fault::AccessFlag(level) => 0b001000 | (level & 3) as u64,
            Fault::Permission(level) => 0b001100 | (level & 3) as u64,
            Fault::External => 0b010000,
            Fault::Alignment => 0b100001,
        }
    }
}

/// The base-2 logarithm of the 4 KiB page size.
pub const PAGE_BITS: u32 = 12;

/// The page size this core translates with.
pub const PAGE_SIZE: u64 = 1 << PAGE_BITS;

/// A mask of the bits below a page boundary.
pub const PAGE_MASK: u64 = PAGE_SIZE - 1;

/// How many bits of physical address this core produces.
///
/// 48, which is what `ID_AA64MMFR0_EL1.PARange = 0b0101` reports and what the
/// descriptor's output-address field holds without `FEAT_LPA`.
pub const PA_BITS: u32 = 48;

/// The descriptor fields this walker reads.
pub mod desc {
    /// Bit 0: the descriptor is valid.
    pub const VALID: u64 = 1 << 0;
    /// Bit 1: at levels 0-2 a table rather than a block; at level 3 a page.
    pub const TABLE: u64 = 1 << 1;
    /// The output address, bits 47:12.
    pub const ADDR: u64 = 0x0000_ffff_ffff_f000;
    /// `AP[2:1]`, bits 7:6: bit 6 grants EL0 access, bit 7 makes it read-only.
    pub const AP_SHIFT: u32 = 6;
    /// The access flag.
    pub const AF: u64 = 1 << 10;
    /// Not global: the entry is tagged with the current ASID.
    pub const NG: u64 = 1 << 11;
    /// Privileged execute-never.
    pub const PXN: u64 = 1 << 53;
    /// Unprivileged execute-never.
    pub const UXN: u64 = 1 << 54;
    /// `PXNTable`, a table descriptor's hierarchical privileged XN.
    pub const PXN_TABLE: u64 = 1 << 59;
    /// `UXNTable`, a table descriptor's hierarchical unprivileged XN.
    pub const UXN_TABLE: u64 = 1 << 60;
    /// `APTable`, bits 62:61.
    pub const AP_TABLE_SHIFT: u32 = 61;
}

/// The `TCR_EL1` fields this walker reads.
pub mod tcr {
    /// `T0SZ`, bits 5:0.
    #[must_use]
    pub const fn t0sz(tcr: u64) -> u32 {
        (tcr & 0x3f) as u32
    }
    /// `EPD0`, bit 7: disable a `TTBR0_EL1` walk.
    #[must_use]
    pub const fn epd0(tcr: u64) -> bool {
        tcr & (1 << 7) != 0
    }
    /// `TG0`, bits 15:14: `0b00` is the 4 KiB granule.
    #[must_use]
    pub const fn tg0(tcr: u64) -> u32 {
        ((tcr >> 14) & 3) as u32
    }
    /// `T1SZ`, bits 21:16.
    #[must_use]
    pub const fn t1sz(tcr: u64) -> u32 {
        ((tcr >> 16) & 0x3f) as u32
    }
    /// `A1`, bit 22: `TTBR1_EL1` supplies the ASID rather than `TTBR0_EL1`.
    #[must_use]
    pub const fn a1(tcr: u64) -> bool {
        tcr & (1 << 22) != 0
    }
    /// `EPD1`, bit 23: disable a `TTBR1_EL1` walk.
    #[must_use]
    pub const fn epd1(tcr: u64) -> bool {
        tcr & (1 << 23) != 0
    }
    /// `TG1`, bits 31:30: `0b10` is the 4 KiB granule. The encoding differs
    /// from `TG0`'s, which is a genuine asymmetry in the architecture and a
    /// classic source of a walker that works on one half of the address space
    /// only.
    #[must_use]
    pub const fn tg1(tcr: u64) -> u32 {
        ((tcr >> 30) & 3) as u32
    }
    /// `AS`, bit 36: the ASID is 16 bits rather than 8.
    #[must_use]
    pub const fn asid16(tcr: u64) -> bool {
        tcr & (1u64 << 36) != 0
    }
}

/// Physical memory as the walker reads it.
///
/// Read-only by construction: without `FEAT_HAFDBS` an AArch64 walk never
/// writes a descriptor, so there is no write half of this trait to leave
/// unused — and a debug walk therefore *cannot* have a side effect, rather
/// than merely not having one today.
pub trait ReadDescriptor {
    /// Read a 64-bit descriptor from physical memory, or `None` if the bus
    /// refused.
    fn read_descriptor(&mut self, addr: u64) -> Option<u64>;
}

/// Which half of the address space a virtual address falls in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Half {
    /// `TTBR0_EL1`: the low half, where the top bits are zero.
    Low,
    /// `TTBR1_EL1`: the high half, where the top bits are one.
    High,
}

/// One translation regime's parameters, worked out from `TCR_EL1`.
#[derive(Debug, Clone, Copy)]
struct Regime {
    /// Which table base to start from.
    base: u64,
    /// How many bits of virtual address the regime translates.
    input_bits: u32,
    /// The level the walk starts at, 0 to 3.
    start_level: u32,
    /// The ASID, already narrowed to 8 or 16 bits.
    asid: u64,
}

/// Which half `va` selects, or `None` if it is in neither — the hole between
/// the two halves, which faults at level 0.
fn select_half(va: u64, t0sz: u32, t1sz: u32) -> Option<Half> {
    // The low half is selected when every bit above the T0SZ boundary is
    // zero, and the high half when every one of them is one. Computed on the
    // full 64-bit address rather than on a truncated one: widening later is
    // how a walker ends up translating a hole.
    let low_bits = 64 - t0sz;
    let high_bits = 64 - t1sz;
    if low_bits < 64 && va >> low_bits == 0 {
        return Some(Half::Low);
    }
    if high_bits < 64 && va >> high_bits == (u64::MAX >> high_bits) {
        return Some(Half::High);
    }
    None
}

/// Work out the walk parameters for `va`.
///
/// `None` is a level-0 translation fault: an address in neither half, a walk
/// the guest disabled with `EPD0`/`EPD1`, a granule this core does not
/// implement, or a `TnSZ` outside the range the 4 KiB granule allows.
fn regime(regs: &SysRegs, va: u64) -> Option<Regime> {
    let t0sz = tcr::t0sz(regs.tcr);
    let t1sz = tcr::t1sz(regs.tcr);
    let half = select_half(va, t0sz, t1sz)?;
    let (base, tnsz, disabled, granule_ok) = match half {
        // TG0 spells the 4 KiB granule 0b00 and TG1 spells it 0b10. The
        // asymmetry is the architecture's.
        Half::Low => (
            regs.ttbr0,
            t0sz,
            tcr::epd0(regs.tcr),
            tcr::tg0(regs.tcr) == 0b00,
        ),
        Half::High => (
            regs.ttbr1,
            t1sz,
            tcr::epd1(regs.tcr),
            tcr::tg1(regs.tcr) == 0b10,
        ),
    };
    if disabled || !granule_ok {
        return None;
    }
    // Without FEAT_TTST or FEAT_LPA2 the 4 KiB granule allows 16..=39.
    if !(16..=39).contains(&tnsz) {
        return None;
    }
    let input_bits = 64 - tnsz;
    // Each level resolves nine bits above the twelve of page offset, so the
    // walk needs ceil((input_bits - 12) / 9) levels and starts that many from
    // the bottom.
    let levels = (input_bits - PAGE_BITS).div_ceil(9);
    let start_level = 4 - levels;
    let asid_source = if tcr::a1(regs.tcr) {
        regs.ttbr1
    } else {
        regs.ttbr0
    };
    let asid_mask = if tcr::asid16(regs.tcr) {
        0xffff
    } else {
        0x00ff
    };
    Some(Regime {
        // The table base is bits 47:1 of TTBRn, aligned to the table's size;
        // bit 0 is CnP, which this core does not model.
        base: base & desc::ADDR,
        input_bits,
        start_level,
        asid: (asid_source >> 48) & asid_mask,
    })
}

/// Whether an access is permitted by an accumulated permission set.
///
/// DDI 0487 D8.4: `AP[1]` grants EL0 access and `AP[2]` makes the region
/// read-only, at both levels. `APTable` bits accumulate down the walk by a
/// bitwise OR, so a table descriptor can only ever remove permission.
fn permitted(ap: u32, pxn: bool, uxn: bool, wxn: bool, kind: Access, el: El) -> bool {
    let el0_access = ap & 1 != 0;
    let read_only = ap & 2 != 0;
    let writable = !read_only && (el == El::El1 || el0_access);
    match kind {
        Access::Load => el == El::El1 || el0_access,
        Access::Store => writable,
        Access::Fetch => {
            // SCTLR_EL1.WXN: anything writable at this level is execute-never.
            if wxn && writable {
                return false;
            }
            match el {
                El::El1 => !pxn,
                El::El0 => el0_access && !uxn,
            }
        }
    }
}

/// The result of a successful walk: where the page is, and the ASID tagging it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Translation {
    /// The translated physical address, offset included.
    pub pa: u64,
    /// The ASID the entry is tagged with, or `None` if the descriptor was
    /// global.
    pub asid: Option<u64>,
}

/// Walk the tables for `va`.
///
/// `check` selects whether permissions and the access flag are enforced. A
/// debugger asks *where* a page is, not whether the guest could reach it, so
/// [`translate_debug`] passes `false` and gets an answer for a page the
/// current level could not touch.
fn walk<M: ReadDescriptor>(
    regs: &SysRegs,
    mem: &mut M,
    va: u64,
    kind: Access,
    el: El,
    check: bool,
) -> Result<Translation, Fault> {
    let Some(regime) = regime(regs, va) else {
        return Err(Fault::Translation(0));
    };
    let mut level = regime.start_level;
    let mut table = regime.base;
    // Hierarchical permissions, accumulated down the walk.
    let mut ap_table = 0u32;
    let mut pxn_table = false;
    let mut uxn_table = false;
    let wxn = check && regs.sctlr & sctlr::WXN != 0;

    loop {
        // The bits this level indexes with. The first level of a walk may use
        // fewer than nine, because `TnSZ` decides where the input address
        // starts rather than the level does.
        let shift = PAGE_BITS + 9 * (3 - level);
        let index_bits = if level == regime.start_level {
            regime.input_bits - shift
        } else {
            9
        };
        let index = (va >> shift) & ((1u64 << index_bits) - 1);
        // A table smaller than 4 KiB must still be aligned to its own size.
        if table & ((8u64 << index_bits) - 1) != 0 {
            return Err(Fault::AddressSize(level));
        }
        let entry_addr = table + index * 8;
        let d = mem.read_descriptor(entry_addr).ok_or(Fault::External)?;

        if d & desc::VALID == 0 {
            return Err(Fault::Translation(level));
        }
        let is_table = d & desc::TABLE != 0 && level < 3;
        // A level-3 descriptor with bit 1 clear is a reserved encoding, and a
        // level-0 block does not exist at the 4 KiB granule.
        if level == 3 && d & desc::TABLE == 0 {
            return Err(Fault::Translation(level));
        }
        if level == 0 && !is_table {
            return Err(Fault::Translation(level));
        }

        if is_table {
            ap_table |= ((d >> desc::AP_TABLE_SHIFT) & 3) as u32;
            pxn_table |= d & desc::PXN_TABLE != 0;
            uxn_table |= d & desc::UXN_TABLE != 0;
            table = d & desc::ADDR;
            level += 1;
            continue;
        }

        // A block or a page: the walk ends here.
        if check && d & desc::AF == 0 {
            return Err(Fault::AccessFlag(level));
        }
        let ap = (((d >> desc::AP_SHIFT) & 3) as u32) | ap_table;
        let pxn = (d & desc::PXN != 0) | pxn_table;
        let uxn = (d & desc::UXN != 0) | uxn_table;
        if check && !permitted(ap, pxn, uxn, wxn, kind, el) {
            return Err(Fault::Permission(level));
        }
        let block_bits = PAGE_BITS + 9 * (3 - level);
        let block_mask = (1u64 << block_bits) - 1;
        let output = (d & desc::ADDR & !block_mask) | (va & block_mask);
        if output >= 1u64 << PA_BITS {
            return Err(Fault::AddressSize(level));
        }
        return Ok(Translation {
            pa: output,
            asid: if d & desc::NG != 0 {
                Some(regime.asid)
            } else {
                None
            },
        });
    }
}

/// Translate one virtual address, enforcing permissions and the access flag.
///
/// # Errors
///
/// The [`Fault`] the guest's abort handler will see, level included.
pub fn translate<M: ReadDescriptor>(
    regs: &SysRegs,
    mem: &mut M,
    va: u64,
    kind: Access,
    el: El,
) -> Result<Translation, Fault> {
    walk(regs, mem, va, kind, el, true)
}

/// Resolve an address the way a debugger asks it.
///
/// Permission-free and access-flag-free — it answers *where the page is*, not
/// whether the current level could touch it — and side-effect free by
/// construction rather than by care: [`ReadDescriptor`] has no write half at
/// all, the caller hands it descriptor reads carrying `MemAttrs::DEBUG`, and
/// an AArch64 walk without `FEAT_HAFDBS` never updates a descriptor anyway.
///
/// `None` means the tables map nothing there.
#[must_use]
pub fn translate_debug<M: ReadDescriptor>(regs: &SysRegs, mem: &mut M, va: u64) -> Option<u64> {
    if !regs.mmu_enabled() {
        return Some(va);
    }
    walk(regs, mem, va, Access::Load, El::El1, false)
        .ok()
        .map(|t| t.pa)
}

// ---------------------------------------------------------------------------
// The TLB
// ---------------------------------------------------------------------------

/// How many entries each of the TLB's three sets holds.
///
/// Direct-mapped and a power of two, so a lookup is a mask and a compare.
pub const TLB_ENTRIES: usize = 256;

/// One cached translation.
#[derive(Debug, Clone, Copy, Default)]
struct Entry {
    /// Virtual page number, ASID, exception level and generation, hashed so a
    /// stale entry can never be mistaken for a hit.
    tag: u64,
    /// The physical address of the page's base.
    base: u64,
    /// Whether this slot holds anything.
    valid: bool,
}

/// The per-core software TLB.
///
/// Derived state in the strict sense of `ROADMAP.md` §4.5: never serialized,
/// and safe to throw away at any moment. The generation counter in
/// [`SysRegs::translation_gen`] is what a `TLBI` bumps, so an invalidation
/// costs nothing until the next lookup.
#[derive(Debug)]
pub struct Tlb {
    slots: [[Entry; TLB_ENTRIES]; 3],
    hits: u64,
    misses: u64,
    /// A [`jit::Tlb`](crate::jit::Tlb) indexed exactly as this table is, so a
    /// compiled access can resolve a guest address to a host one without
    /// calling back. See [`Tlb::attach_shadow`].
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
    pub fn flush(&mut self) {
        self.slots = [[Entry::default(); TLB_ENTRIES]; 3];
        // The shadow is only ever as live as this table is, so it goes with
        // it: an entry that promised a hit here must not outlive the entry
        // that made the promise.
        #[cfg(feature = "jit")]
        if let Some(shadow) = self.shadow.as_mut() {
            shadow.flush();
        }
    }

    /// Give this TLB a [`jit::Tlb`](crate::jit::Tlb) shadow over `space`.
    ///
    /// # What the shadow is
    ///
    /// This table answers *virtual page to physical page*, which is half of
    /// what a compiled load needs; the other half is *physical page to host
    /// address*, and that is what `jit::Tlb` caches. The shadow is that second
    /// half, indexed by the **same** virtual page in the **same** slot, so a
    /// compiled access goes from a guest address to a host one in a mask, a
    /// compare and an add — `ROADMAP.md` §9.1's first mechanism, inlined.
    ///
    /// # Why it lives here rather than beside the engine
    ///
    /// A compiled load that hits the shadow charges **one** tick and skips the
    /// walk. That is only right if this table would have hit too, so the two
    /// have to stay in lockstep — and lockstep is a property of *every* path
    /// that can insert here, not only of the translated one. An interpreted
    /// exclusive inserts, a trap handler outside the lifted subset inserts, a
    /// debugger's single step inserts; each of those evicts a slot, and a
    /// shadow living next to the engine would not hear about any of them.
    /// Owning it here means `Exec::translate` maintains both at once and there
    /// is no other way in.
    ///
    /// The two are the same size for the same reason: the slot a page lands in
    /// must be the same slot in both, or an eviction here would leave a shadow
    /// entry alive that promises a hit this table no longer has.
    ///
    /// # What AArch64 contributes that RISC-V does not
    ///
    /// The **ASID**. An entry here is tagged with it; an entry in the shadow is
    /// not, and does not need to be, because an AArch64 ASID lives in
    /// `TTBR0_EL1[63:48]` — so changing it is a `TTBR0_EL1` write, and a
    /// `TTBR0_EL1` write bumps [`SysRegs::translation_gen`], which is the
    /// counter both this table's tag and the shadow's stamp carry. The two go
    /// stale together, which is what `jit::fast` asks for.
    ///
    /// Nothing else about this architecture needs handling: address tagging is
    /// not implemented (an address carrying one falls in neither half and
    /// faults), which `TTBR` a walk starts from is a pure function of the
    /// address, and the 4 KiB granule is the only one this core accepts — so a
    /// virtual page number here is a virtual page number there.
    #[cfg(feature = "jit")]
    #[cfg_attr(docsrs, doc(cfg(feature = "jit")))]
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
    #[cfg_attr(docsrs, doc(cfg(feature = "jit")))]
    #[inline]
    pub fn shadow_mut(&mut self) -> Option<&mut crate::jit::Tlb> {
        self.shadow.as_deref_mut()
    }

    /// Whether a shadow is attached.
    #[cfg(feature = "jit")]
    #[cfg_attr(docsrs, doc(cfg(feature = "jit")))]
    #[inline]
    #[must_use]
    pub fn has_shadow(&self) -> bool {
        self.shadow.is_some()
    }

    /// How many lookups hit and how many missed.
    #[must_use]
    pub fn stats(&self) -> (u64, u64) {
        (self.hits, self.misses)
    }

    /// The tag for a page.
    #[inline]
    fn tag(vpn: u64, asid: u64, el: El, generation: u64) -> u64 {
        // The generation goes in the high bits so a bump invalidates every
        // entry at once without touching them.
        (generation << 40) ^ vpn.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ (asid << 2) ^ el.bits()
    }

    /// Look a page up.
    #[inline]
    pub fn lookup(
        &mut self,
        kind: Access,
        vpn: u64,
        asid: u64,
        el: El,
        generation: u64,
    ) -> Option<u64> {
        let tag = Self::tag(vpn, asid, el, generation);
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
        el: El,
        generation: u64,
        base: u64,
    ) {
        self.slots[kind.slot()][(vpn as usize) & (TLB_ENTRIES - 1)] = Entry {
            tag: Self::tag(vpn, asid, el, generation),
            base,
            valid: true,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    /// A flat physical memory for the walker to read tables out of.
    struct Ram(Vec<u8>);

    impl Ram {
        fn put(&mut self, addr: u64, value: u64) {
            let at = addr as usize;
            self.0[at..at + 8].copy_from_slice(&value.to_le_bytes());
        }
    }

    impl ReadDescriptor for Ram {
        fn read_descriptor(&mut self, addr: u64) -> Option<u64> {
            let at = addr as usize;
            let slice = self.0.get(at..at + 8)?;
            let mut bytes = [0u8; 8];
            bytes.copy_from_slice(slice);
            Some(u64::from_le_bytes(bytes))
        }
    }

    /// A 39-bit-VA regime — the one Linux uses on a 4 KiB-granule kernel —
    /// with a three-level walk mapping virtual 0 to physical 0x4000.
    ///
    /// `T0SZ = 25` gives 39 bits of input, `ceil((39 - 12) / 9) == 3` levels,
    /// so the walk starts at level 1.
    fn three_level(leaf_attrs: u64) -> (SysRegs, Ram) {
        let mut ram = Ram(vec![0; 0x8000]);
        // Level 1 at 0x1000 -> level 2 at 0x2000 -> level 3 at 0x3000.
        ram.put(0x1000, 0x2000 | desc::VALID | desc::TABLE);
        ram.put(0x2000, 0x3000 | desc::VALID | desc::TABLE);
        ram.put(0x3000, 0x4000 | desc::VALID | desc::TABLE | leaf_attrs);
        let mut regs = SysRegs::new();
        regs.sctlr |= sctlr::M;
        regs.ttbr0 = 0x1000;
        regs.tcr = 25 | (25 << 16) | (0b10u64 << 30);
        (regs, ram)
    }

    #[test]
    fn a_three_level_walk_reaches_the_page() {
        let (regs, mut ram) = three_level(desc::AF);
        let t = translate(&regs, &mut ram, 0x123, Access::Load, El::El1).unwrap();
        assert_eq!(t.pa, 0x4123);
        // Global by default: no nG bit was set.
        assert_eq!(t.asid, None);
    }

    #[test]
    fn a_clear_access_flag_faults_at_the_leaf_level() {
        let (regs, mut ram) = three_level(0);
        let err = translate(&regs, &mut ram, 0, Access::Load, El::El1).unwrap_err();
        assert_eq!(err, Fault::AccessFlag(3));
        // 0b001000 | 3
        assert_eq!(err.dfsc(), 0b001011);
    }

    #[test]
    fn ap_bits_are_enforced_at_both_levels() {
        // AP = 0b00: EL1 read/write, EL0 nothing.
        let (regs, mut ram) = three_level(desc::AF);
        assert!(translate(&regs, &mut ram, 0, Access::Store, El::El1).is_ok());
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Load, El::El0).unwrap_err(),
            Fault::Permission(3)
        );
        // AP = 0b10: read-only at EL1.
        let (regs, mut ram) = three_level(desc::AF | (2 << desc::AP_SHIFT));
        assert!(translate(&regs, &mut ram, 0, Access::Load, El::El1).is_ok());
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Store, El::El1).unwrap_err(),
            Fault::Permission(3)
        );
        // AP = 0b01: read/write at both.
        let (regs, mut ram) = three_level(desc::AF | (1 << desc::AP_SHIFT));
        assert!(translate(&regs, &mut ram, 0, Access::Store, El::El0).is_ok());
    }

    #[test]
    fn a_table_descriptor_can_only_remove_permission() {
        let (regs, mut ram) = three_level(desc::AF | (1 << desc::AP_SHIFT));
        // The leaf grants EL0 read/write; APTable = 0b10 on the level-2
        // descriptor makes the whole subtree read-only.
        ram.put(
            0x2000,
            0x3000 | desc::VALID | desc::TABLE | (2u64 << desc::AP_TABLE_SHIFT),
        );
        assert!(translate(&regs, &mut ram, 0, Access::Load, El::El0).is_ok());
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Store, El::El0).unwrap_err(),
            Fault::Permission(3)
        );
        // And UXNTable removes execution from it.
        ram.put(0x2000, 0x3000 | desc::VALID | desc::TABLE | desc::UXN_TABLE);
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Fetch, El::El0).unwrap_err(),
            Fault::Permission(3)
        );
    }

    #[test]
    fn wxn_makes_a_writable_page_execute_never() {
        let (mut regs, mut ram) = three_level(desc::AF);
        assert!(translate(&regs, &mut ram, 0, Access::Fetch, El::El1).is_ok());
        regs.sctlr |= sctlr::WXN;
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Fetch, El::El1).unwrap_err(),
            Fault::Permission(3)
        );
    }

    #[test]
    fn a_block_descriptor_ends_the_walk_early() {
        let mut ram = Ram(vec![0; 0x8000]);
        // Level 1 -> level 2; the level-2 entry is a 2 MiB block.
        ram.put(0x1000, 0x2000 | desc::VALID | desc::TABLE);
        ram.put(0x2000, 0x40_0000 | desc::VALID | desc::AF);
        let mut regs = SysRegs::new();
        regs.sctlr |= sctlr::M;
        regs.ttbr0 = 0x1000;
        regs.tcr = 25 | (25 << 16) | (0b10u64 << 30);
        let t = translate(&regs, &mut ram, 0x1_2345, Access::Load, El::El1).unwrap();
        assert_eq!(t.pa, 0x40_0000 + 0x1_2345);
    }

    #[test]
    fn the_high_half_uses_ttbr1() {
        let (mut regs, mut ram) = three_level(desc::AF);
        regs.ttbr1 = 0x1000;
        regs.ttbr0 = 0;
        // 39-bit T1SZ: the high half starts at 0xffff_ff80_0000_0000.
        let va = 0xffff_ff80_0000_0000;
        assert_eq!(
            translate(&regs, &mut ram, va, Access::Load, El::El1)
                .unwrap()
                .pa,
            0x4000
        );
        // The hole between the halves translates nothing.
        assert_eq!(
            translate(
                &regs,
                &mut ram,
                0x0000_8000_0000_0000,
                Access::Load,
                El::El1
            )
            .unwrap_err(),
            Fault::Translation(0)
        );
    }

    #[test]
    fn epd_disables_a_half() {
        let (mut regs, mut ram) = three_level(desc::AF);
        regs.tcr |= 1 << 7; // EPD0
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Load, El::El1).unwrap_err(),
            Fault::Translation(0)
        );
    }

    #[test]
    fn an_unimplemented_granule_faults_rather_than_guessing() {
        let (mut regs, mut ram) = three_level(desc::AF);
        // TG0 = 0b01 is the 64 KiB granule, which this core does not have.
        regs.tcr |= 0b01 << 14;
        assert_eq!(
            translate(&regs, &mut ram, 0, Access::Load, El::El1).unwrap_err(),
            Fault::Translation(0)
        );
    }

    #[test]
    fn a_four_level_walk_starts_at_level_zero() {
        let mut ram = Ram(vec![0; 0x8000]);
        ram.put(0x1000, 0x2000 | desc::VALID | desc::TABLE);
        ram.put(0x2000, 0x3000 | desc::VALID | desc::TABLE);
        ram.put(0x3000, 0x4000 | desc::VALID | desc::TABLE);
        ram.put(0x4000, 0x5000 | desc::VALID | desc::TABLE | desc::AF);
        let mut regs = SysRegs::new();
        regs.sctlr |= sctlr::M;
        regs.ttbr0 = 0x1000;
        // T0SZ = 16: 48 bits of input, four levels.
        regs.tcr = 16 | (16 << 16) | (0b10u64 << 30);
        assert_eq!(
            translate(&regs, &mut ram, 0xabc, Access::Load, El::El1)
                .unwrap()
                .pa,
            0x5abc
        );
    }

    #[test]
    fn the_debug_walk_ignores_permissions_and_the_access_flag() {
        // No AF, and no EL0 access: an ordinary translate refuses both.
        let (regs, mut ram) = three_level(0);
        assert!(translate(&regs, &mut ram, 0x10, Access::Load, El::El0).is_err());
        assert_eq!(translate_debug(&regs, &mut ram, 0x10), Some(0x4010));
    }

    #[test]
    fn the_debug_walk_is_the_identity_with_the_mmu_off() {
        let mut regs = SysRegs::new();
        regs.sctlr = 0;
        let mut ram = Ram(vec![0; 0x100]);
        assert_eq!(
            translate_debug(&regs, &mut ram, 0xdead_beef),
            Some(0xdead_beef)
        );
    }

    #[test]
    fn the_generation_invalidates_every_entry_at_once() {
        let mut tlb = Tlb::new();
        tlb.insert(Access::Load, 1, 0, El::El1, 0, 0x4000);
        assert_eq!(tlb.lookup(Access::Load, 1, 0, El::El1, 0), Some(0x4000));
        assert_eq!(tlb.lookup(Access::Load, 1, 0, El::El1, 1), None);
        // And a different exception level is a different entry.
        assert_eq!(tlb.lookup(Access::Load, 1, 0, El::El0, 0), None);
    }
}
