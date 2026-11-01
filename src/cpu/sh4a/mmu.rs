//! The memory management unit: the 64-entry UTLB, the 4-entry ITLB, the
//! 16-entry PMB of 32-bit address extended mode, and the processor's own
//! control registers in the P4 area.
//!
//! # The address map this core presents to the bus
//!
//! The SH-4A has a 32-bit virtual space and — in the 29-bit mode it resets
//! into — a 29-bit physical one (manual §7.1.1). The core hands the machine
//! **one 32-bit physical bus** and puts everything on it at a fixed place:
//!
//! * **P0/U0/P3 with `MMUCR.AT = 0`, and P1/P2**: the virtual address with
//!   its top three bits cleared — physical `0x0000_0000..0x1FFF_FFFF`.
//! * **With `PASCR.SE = 1`** (32-bit address extended mode, §7.7): P0/U0/P3
//!   untranslated become the full 32-bit virtual address, and P1/P2 go through
//!   the PMB to a 32-bit physical address.
//! * **The P4 area** (`0xE000_0000..`): the store queues, the memory-mapped
//!   cache and TLB arrays and this core's own registers are answered *inside*
//!   the core. Everything else in P4 — the on-chip peripheral control
//!   registers at `0xFC00_0000..0xFFFF_FFFF`, the L memory at `0xE500_0000` —
//!   goes to the bus **at its P4 address**, so a machine maps an SH-4A
//!   peripheral where the part's hardware manual says it is.
//! * **Area 7 through the TLB** (`0x1C00_0000..0x1FFF_FFFF` in 29-bit mode,
//!   `0xFC00_0000..` in 32-bit mode) is the control-register area again
//!   (§7.1.1, "Physical Address Space"), so it is routed exactly as the P4
//!   address it aliases.
//!
//! # What is a cache and what is architecture
//!
//! The UTLB, ITLB and PMB are **architectural**: software reads them back
//! through the memory-mapped arrays (§7.6, §7.7.5), so they are saved. The
//! operand and instruction caches are not modelled — every access goes to the
//! bus — and their memory-mapped arrays read as invalid lines and accept
//! writes, which is what a guest's "invalidate every line" loop needs.
//!
//! # Sources
//!
//! *SH-4A Software Manual*, REJ09B0003-0150: §7.1–7.6 (UTLB/ITLB, the
//! translation flowcharts 7.9 and 7.10, `MMUCR`, the memory-mapped arrays),
//! §7.7 (PMB and `PASCR.SE`), §8.2 and §8.7 (`CCR`, `QACR0/1`, the store
//! queues), §5.2 (`TRA`, `EXPEVT`, `INTEVT`), appendix A (`CPUOPM`) and
//! appendix D (`PVR`, `PRR`).

/// One UTLB entry (manual figure 7.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UtlbEntry {
    /// The virtual page number, as the address bits it covers (bits 31–10).
    pub vpn: u32,
    /// The address space identifier.
    pub asid: u8,
    /// The physical page number, as address bits (bits 31–10; 28–10 in
    /// 29-bit mode).
    pub ppn: u32,
    /// Page size: 0 = 1 KB, 1 = 4 KB, 2 = 64 KB, 3 = 1 MB.
    pub sz: u8,
    /// Protection key: `PR[1:0]`.
    pub pr: u8,
    /// Valid.
    pub v: bool,
    /// Shared: matches every ASID.
    pub sh: bool,
    /// Cacheable.
    pub c: bool,
    /// Dirty: a store to a page with `D` clear raises an initial page write
    /// exception.
    pub d: bool,
    /// Write-through.
    pub wt: bool,
    /// Unbuffered write (32-bit address extended mode only).
    pub ub: bool,
}

/// One ITLB entry (manual figure 7.8): no `D`, no `WT`, one protection bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ItlbEntry {
    /// The virtual page number, as address bits.
    pub vpn: u32,
    /// The address space identifier.
    pub asid: u8,
    /// The physical page number, as address bits.
    pub ppn: u32,
    /// Page size, as in the UTLB.
    pub sz: u8,
    /// The upper bit of the UTLB's `PR`: user-mode access permitted.
    pub pr: bool,
    /// Valid.
    pub v: bool,
    /// Shared.
    pub sh: bool,
    /// Cacheable.
    pub c: bool,
}

/// One PMB entry (manual figure 7.17).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PmbEntry {
    /// Virtual page number: address bits 31–24.
    pub vpn: u32,
    /// Physical page number: address bits 31–24.
    pub ppn: u32,
    /// Page size: 0 = 16 MB, 1 = 64 MB, 2 = 128 MB, 3 = 512 MB.
    pub sz: u8,
    /// Valid.
    pub v: bool,
    /// Cacheable.
    pub c: bool,
    /// Unbuffered write.
    pub ub: bool,
    /// Write-through.
    pub wt: bool,
}

/// How many UTLB entries there are.
pub const UTLB_ENTRIES: usize = 64;
/// How many ITLB entries there are.
pub const ITLB_ENTRIES: usize = 4;
/// How many PMB entries there are.
pub const PMB_ENTRIES: usize = 16;

/// The address bits a UTLB/ITLB page of size code `sz` translates.
#[inline]
#[must_use]
pub const fn page_mask(sz: u8) -> u32 {
    match sz & 3 {
        0 => 0xffff_fc00,
        1 => 0xffff_f000,
        2 => 0xffff_0000,
        _ => 0xfff0_0000,
    }
}

/// The address bits a PMB page of size code `sz` translates.
#[inline]
#[must_use]
pub const fn pmb_mask(sz: u8) -> u32 {
    match sz & 3 {
        0 => 0xff00_0000,
        1 => 0xfc00_0000,
        2 => 0xf800_0000,
        _ => 0xe000_0000,
    }
}

/// The TLBs and the PMB.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mmu {
    /// The unified TLB.
    pub utlb: [UtlbEntry; UTLB_ENTRIES],
    /// The instruction TLB.
    pub itlb: [ItlbEntry; ITLB_ENTRIES],
    /// The privileged space mapping buffer.
    pub pmb: [PmbEntry; PMB_ENTRIES],
}

impl Default for Mmu {
    fn default() -> Self {
        Mmu {
            utlb: [UtlbEntry::default(); UTLB_ENTRIES],
            itlb: [ItlbEntry::default(); ITLB_ENTRIES],
            pmb: [PmbEntry::default(); PMB_ENTRIES],
        }
    }
}

impl Mmu {
    /// Clear every `V` bit — `MMUCR.TI`, and a power-on reset (§7.3.1: "V ...
    /// Cleared to 0 by a power-on reset. Not affected by a manual reset.").
    pub fn invalidate_tlbs(&mut self) {
        for e in &mut self.utlb {
            e.v = false;
        }
        for e in &mut self.itlb {
            e.v = false;
        }
    }

    /// The PMB's `V` bits too, for a power-on reset (§7.7.3).
    pub fn invalidate_all(&mut self) {
        self.invalidate_tlbs();
        for e in &mut self.pmb {
            e.v = false;
        }
    }
}

/// The result of an associative search.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hit {
    /// Nothing matched.
    Miss,
    /// Exactly one entry matched.
    One(usize),
    /// More than one entry matched — a multiple-hit exception, which resets
    /// the processor (§7.5.1, §7.5.4).
    Multiple,
}

/// Whether a TLB entry's ASID must match, per flowcharts 7.9 and 7.10: the
/// ASID is compared when `SH = 0` and either `MMUCR.SV = 0` or the access is
/// from user mode.
#[inline]
const fn asid_compared(sh: bool, sv: bool, privileged: bool) -> bool {
    !sh && (!sv || !privileged)
}

impl Mmu {
    /// Search the UTLB for `va`.
    #[must_use]
    pub fn search_utlb(&self, va: u32, asid: u8, sv: bool, privileged: bool) -> Hit {
        let mut found = Hit::Miss;
        for (i, e) in self.utlb.iter().enumerate() {
            if !e.v {
                continue;
            }
            let mask = page_mask(e.sz);
            if (va ^ e.vpn) & mask != 0 {
                continue;
            }
            if asid_compared(e.sh, sv, privileged) && e.asid != asid {
                continue;
            }
            found = match found {
                Hit::Miss => Hit::One(i),
                _ => return Hit::Multiple,
            };
        }
        found
    }

    /// Search the ITLB for `va`.
    #[must_use]
    pub fn search_itlb(&self, va: u32, asid: u8, sv: bool, privileged: bool) -> Hit {
        let mut found = Hit::Miss;
        for (i, e) in self.itlb.iter().enumerate() {
            if !e.v {
                continue;
            }
            let mask = page_mask(e.sz);
            if (va ^ e.vpn) & mask != 0 {
                continue;
            }
            if asid_compared(e.sh, sv, privileged) && e.asid != asid {
                continue;
            }
            found = match found {
                Hit::Miss => Hit::One(i),
                _ => return Hit::Multiple,
            };
        }
        found
    }

    /// Search the PMB for a P1/P2 address.
    #[must_use]
    pub fn search_pmb(&self, va: u32) -> Hit {
        let mut found = Hit::Miss;
        for (i, e) in self.pmb.iter().enumerate() {
            if e.v && (va ^ e.vpn) & pmb_mask(e.sz) == 0 {
                found = match found {
                    Hit::Miss => Hit::One(i),
                    _ => return Hit::Multiple,
                };
            }
        }
        found
    }
}

/// Which ITLB entry `MMUCR.LRUI` says to replace (manual §7.2.5): `111xxx` →
/// 0, `0xx11x` → 1, `x0x0x1` → 2, `xx0x00` → 3. The other patterns are
/// "setting prohibited"; this falls back to entry 0 for them rather than
/// refusing to refill, because a refill that never happens hangs the guest.
#[must_use]
pub const fn lrui_victim(lrui: u32) -> usize {
    // Bit patterns as (mask, value) over LRUI[5:0], in the manual's order.
    if lrui & 0b111_000 == 0b111_000 {
        0
    } else if lrui & 0b100_110 == 0b000_110 {
        1
    } else if lrui & 0b010_101 == 0b000_001 {
        2
    } else if lrui & 0b001_011 == 0b000_000 {
        3
    } else {
        0
    }
}

/// `MMUCR.LRUI` after ITLB entry `used` was used (manual §7.2.5): entry 0
/// clears bits 5–3; entry 1 sets 5 and clears 2–1; entry 2 sets 4 and 2 and
/// clears 0; entry 3 sets 3, 1 and 0.
#[must_use]
pub const fn lrui_touch(lrui: u32, used: usize) -> u32 {
    match used {
        0 => lrui & !0b111_000,
        1 => (lrui | 0b100_000) & !0b000_110,
        2 => (lrui | 0b010_100) & !0b000_001,
        _ => lrui | 0b001_011,
    }
}

// ---------------------------------------------------------------------------
// The control registers this core answers in P4
// ---------------------------------------------------------------------------

/// P4 addresses of the registers the core itself owns (manual §11.1 and the
/// appendices). Anything else in P4 goes to the bus.
pub mod reg {
    /// Page table entry high.
    pub const PTEH: u32 = 0xff00_0000;
    /// Page table entry low.
    pub const PTEL: u32 = 0xff00_0004;
    /// Translation table base.
    pub const TTB: u32 = 0xff00_0008;
    /// TLB exception address.
    pub const TEA: u32 = 0xff00_000c;
    /// MMU control.
    pub const MMUCR: u32 = 0xff00_0010;
    /// Cache control.
    pub const CCR: u32 = 0xff00_001c;
    /// TRAPA exception register.
    pub const TRA: u32 = 0xff00_0020;
    /// Exception event register.
    pub const EXPEVT: u32 = 0xff00_0024;
    /// Interrupt event register.
    pub const INTEVT: u32 = 0xff00_0028;
    /// Processor version register.
    pub const PVR: u32 = 0xff00_0030;
    /// Queue address control 0.
    pub const QACR0: u32 = 0xff00_0038;
    /// Queue address control 1.
    pub const QACR1: u32 = 0xff00_003c;
    /// Product register.
    pub const PRR: u32 = 0xff00_0044;
    /// L memory transfer source address 0.
    pub const LSA0: u32 = 0xff00_0050;
    /// L memory transfer source address 1.
    pub const LSA1: u32 = 0xff00_0054;
    /// L memory transfer destination address 0.
    pub const LDA0: u32 = 0xff00_0058;
    /// L memory transfer destination address 1.
    pub const LDA1: u32 = 0xff00_005c;
    /// Physical address space control.
    pub const PASCR: u32 = 0xff00_0070;
    /// On-chip memory control.
    pub const RAMCR: u32 = 0xff00_0074;
    /// Instruction re-fetch inhibit control.
    pub const IRMCR: u32 = 0xff00_0078;
    /// CPU operation mode.
    pub const CPUOPM: u32 = 0xff2f_0000;
    /// The first store-queue read address (§8.7.5).
    pub const SQ_READ: u32 = 0xff00_1000;
}

/// `MMUCR` bits.
pub mod mmucr {
    /// Address translation enable.
    pub const AT: u32 = 1 << 0;
    /// TLB invalidate (write-only, reads as zero).
    pub const TI: u32 = 1 << 2;
    /// Single virtual memory mode.
    pub const SV: u32 = 1 << 8;
    /// Store queues privileged-only.
    pub const SQMD: u32 = 1 << 9;
    /// The UTLB replace counter's position.
    pub const URC_SHIFT: u32 = 10;
    /// The UTLB replace boundary's position.
    pub const URB_SHIFT: u32 = 18;
    /// The ITLB LRU field's position.
    pub const LRUI_SHIFT: u32 = 26;
    /// The bits software can write (§7.2.5's R/W row): LRUI, URB, URC, SQMD,
    /// SV, TI and AT.
    pub const WRITABLE: u32 = 0xfcfc_ff05;
}

/// `PASCR.SE`: 32-bit address extended mode (§7.7.6).
pub const PASCR_SE: u32 = 1 << 31;

/// `CPUOPM.INTMU`: an accepted interrupt raises `SR.IMASK` to its level
/// (appendix A).
pub const CPUOPM_INTMU: u32 = 1 << 3;

/// `CPUOPM`'s reset value: reserved bits 9–6 and `RABD` read as one
/// (appendix A).
pub const CPUOPM_RESET: u32 = 0x0000_03e0;

/// The processor's own registers that live in P4 rather than in the
/// register file.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Ctl {
    /// `PTEH`.
    pub pteh: u32,
    /// `PTEL`.
    pub ptel: u32,
    /// `TTB`.
    pub ttb: u32,
    /// `TEA`.
    pub tea: u32,
    /// `MMUCR`.
    pub mmucr: u32,
    /// `CCR`.
    pub ccr: u32,
    /// `TRA`.
    pub tra: u32,
    /// `EXPEVT`.
    pub expevt: u32,
    /// `INTEVT`.
    pub intevt: u32,
    /// `QACR0`.
    pub qacr0: u32,
    /// `QACR1`.
    pub qacr1: u32,
    /// `LSA0`, `LSA1`, `LDA0`, `LDA1` — held, with no L-memory transfer
    /// engine behind them.
    pub lmem: [u32; 4],
    /// `PASCR`.
    pub pascr: u32,
    /// `RAMCR`.
    pub ramcr: u32,
    /// `IRMCR`.
    pub irmcr: u32,
    /// `CPUOPM`.
    pub cpuopm: u32,
}

impl Ctl {
    /// `MMUCR.AT`.
    #[inline]
    #[must_use]
    pub const fn at(&self) -> bool {
        self.mmucr & mmucr::AT != 0
    }

    /// `MMUCR.SV`.
    #[inline]
    #[must_use]
    pub const fn sv(&self) -> bool {
        self.mmucr & mmucr::SV != 0
    }

    /// `PASCR.SE`.
    #[inline]
    #[must_use]
    pub const fn se(&self) -> bool {
        self.pascr & PASCR_SE != 0
    }

    /// The current ASID, `PTEH[7:0]`.
    #[inline]
    #[must_use]
    pub const fn asid(&self) -> u8 {
        self.pteh as u8
    }

    /// `MMUCR.URC`.
    #[inline]
    #[must_use]
    pub const fn urc(&self) -> usize {
        ((self.mmucr >> mmucr::URC_SHIFT) & 0x3f) as usize
    }

    /// Advance `MMUCR.URC` once, as every UTLB access does (§7.2.5): it wraps
    /// to zero at `URB` when `URB` is non-zero, and at 64 otherwise.
    pub fn tick_urc(&mut self) {
        let urb = (self.mmucr >> mmucr::URB_SHIFT) & 0x3f;
        let mut urc = (self.urc() as u32 + 1) & 0x3f;
        if urb != 0 && urc == urb {
            urc = 0;
        }
        self.mmucr = (self.mmucr & !(0x3f << mmucr::URC_SHIFT)) | (urc << mmucr::URC_SHIFT);
    }

    /// `MMUCR.LRUI`.
    #[inline]
    #[must_use]
    pub const fn lrui(&self) -> u32 {
        self.mmucr >> mmucr::LRUI_SHIFT
    }

    /// Replace `MMUCR.LRUI`.
    pub fn set_lrui(&mut self, lrui: u32) {
        self.mmucr =
            (self.mmucr & !(0x3f << mmucr::LRUI_SHIFT)) | ((lrui & 0x3f) << mmucr::LRUI_SHIFT);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_lru_victim_follows_the_touches() {
        // From reset (LRUI = 0) the manual's table names entry 3; touch it and
        // the victim moves on, until every entry has been used once and the
        // first is the least recent again.
        let mut lrui = 0;
        assert_eq!(lrui_victim(lrui), 3);
        lrui = lrui_touch(lrui, 3);
        assert_eq!(lrui_victim(lrui), 2);
        lrui = lrui_touch(lrui, 2);
        assert_eq!(lrui_victim(lrui), 1);
        lrui = lrui_touch(lrui, 1);
        assert_eq!(lrui_victim(lrui), 0);
        lrui = lrui_touch(lrui, 0);
        assert_eq!(lrui_victim(lrui), 3);
    }

    #[test]
    fn urc_wraps_at_the_boundary() {
        let mut c = Ctl {
            mmucr: (4 << mmucr::URB_SHIFT) | (3 << mmucr::URC_SHIFT),
            ..Ctl::default()
        };
        c.tick_urc();
        assert_eq!(c.urc(), 0);
        c.mmucr = 63 << mmucr::URC_SHIFT;
        c.tick_urc();
        assert_eq!(c.urc(), 0);
    }

    #[test]
    fn a_shared_page_matches_any_asid_and_a_private_one_does_not() {
        let mut mmu = Mmu::default();
        mmu.utlb[5] = UtlbEntry {
            vpn: 0x0040_0000,
            asid: 7,
            ppn: 0x0c00_0000,
            sz: 1,
            v: true,
            ..UtlbEntry::default()
        };
        assert_eq!(mmu.search_utlb(0x0040_0123, 7, false, true), Hit::One(5));
        assert_eq!(mmu.search_utlb(0x0040_0123, 8, false, true), Hit::Miss);
        // Single virtual memory mode in privileged mode ignores the ASID.
        assert_eq!(mmu.search_utlb(0x0040_0123, 8, true, true), Hit::One(5));
        mmu.utlb[5].sh = true;
        assert_eq!(mmu.search_utlb(0x0040_0123, 8, false, false), Hit::One(5));
        mmu.utlb[9] = mmu.utlb[5];
        assert_eq!(mmu.search_utlb(0x0040_0123, 7, false, true), Hit::Multiple);
    }
}
