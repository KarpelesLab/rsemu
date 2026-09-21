//! The MC68040's memory management unit.
//!
//! Everything here is M68040UM Section 3, and none of it is the 68030's.
//! The two units share a name, a purpose and almost nothing else — which is
//! why this is a module beside [`mmu`](super::mmu) rather than a set of
//! branches inside it:
//!
//! | | 68030 (MC68030UM §9) | 68040 (M68040UM §3) |
//! | --- | --- | --- |
//! | page size | eight sizes, 256 B to 32 KiB, from `TC`'s **PS** | two, 4 KiB or 8 KiB, from `TC`'s **P** |
//! | table shape | one to five levels, widths from `TC`'s `TIx` | exactly three, 7/7/6 or 7/7/5 bits |
//! | descriptors | six formats, short and long, with limits | three: table, page and indirect, all one long word |
//! | root pointers | 64-bit descriptors, `CRP` and `SRP` | 32-bit addresses, `URP` and `SRP` |
//! | reached by | `PMOVE` over the coprocessor interface | `MOVEC` |
//! | `TTx` | two, function-code base and mask | four, split instruction and data, an **S** field |
//! | ATC | 22 entries, fully associative | 64 entries, four-way set associative |
//! | fault report | the 68020's format `$A`/`$B` frames | format `$7`, thirty words |
//!
//! # What is here and what is in `exec.rs`
//!
//! The same division the 68030's module makes: everything that does not touch
//! the bus. [`search`] is the table walk, and it takes a callback that reads
//! or writes one long word at a *physical* address, so the descriptor fetches
//! and the history-bit write-backs are real bus cycles, charged and logged
//! like any other.
//!
//! # What is not modelled
//!
//! - **Two address translation caches.** The instruction and data memory
//!   units have one each (§3.3). They hold the same entries: the tables are
//!   shared ("No distinction is made in the translation of instruction
//!   accesses versus data accesses", §3.2.1), every `PFLUSH` variant selects
//!   both (M68000PRM §6, *PFLUSH* (MC68040): "invalidates address translation
//!   cache entries in **both** the instruction and data address translation
//!   caches"), and nothing reads an ATC's contents but `PTEST`, which
//!   rewrites the entry it reports on. So one cache is kept. The split that
//!   *is* observable — the instruction and data transparent translation
//!   registers, which "provide an exception to the merged instruction and
//!   data address space" (§3.4) — is modelled in full.
//! - **The replacement algorithm.** §3.3 calls it "pseudo-random" and says
//!   only that "a 2-bit counter, which is incremented for each ATC access,
//!   points to the entry to replace". This module fills an invalid way when
//!   the set has one and otherwise cycles a two-bit counter *per install*
//!   rather than per access, which is *a* replacement policy and not *that*
//!   one. It is observable only through `PTEST` after more than four pages
//!   have been touched in one set.
//! - **`MDIS`.** There is no such input here; `TC`'s **E** bit is the switch
//!   (§3.6.2).
//! - **Burst descriptor fetches**, and the `LOCK` the read-modify-write
//!   history updates of Table 3-1 assert. Every descriptor is read and
//!   written as an ordinary long word, and the core holds its own bus lock
//!   across the whole instruction, which is stronger.
//! - **`UPA1`/`UPA0` and the cache modes as signals.** `U1`, `U0` and `CM`
//!   are read out of the descriptors, accumulated into the ATC entry and
//!   reported by `PTEST`, but there is no pin to drive and no cache to
//!   switch between write-through and copyback.

/// Translation control register bits (M68040UM Figure 3-4).
///
/// Sixteen bits, of which two exist: "Bits 13–0 are undefined (reserved)".
pub(super) mod tcr {
    /// Bit 15: translation enable.
    pub(in super::super) const E: u16 = 0x8000;
    /// Bit 14: page size — clear for 4 KiB, set for 8 KiB.
    pub(in super::super) const P: u16 = 0x4000;
    /// Every bit the register implements.
    pub(in super::super) const IMPLEMENTED: u16 = E | P;
}

/// Transparent translation register bits (M68040UM Figure 3-5).
///
/// "Bits 12–10, 7, 4, 3, 1, and 0 always read as zero", which leaves the
/// eight-bit base, the eight-bit mask, **E**, the two-bit **S** field, `U1`,
/// `U0`, the two-bit **CM** and **W**.
pub(super) mod ttr {
    /// Bit 15: enable.
    pub(in super::super) const E: u32 = 0x0000_8000;
    /// Bits 14–13: which privilege modes match.
    pub(super) const S_SHIFT: u32 = 13;
    /// Bit 9, bit 8: the user page attributes, echoed to `UPA1`/`UPA0`.
    pub(super) const U1: u32 = 0x0000_0200;
    /// See [`U1`].
    pub(super) const U0: u32 = 0x0000_0100;
    /// Bits 6–5: the cache mode.
    pub(super) const CM_SHIFT: u32 = 5;
    /// Bit 2: write protect.
    pub(in super::super) const W: u32 = 0x0000_0004;
    /// Every bit the register implements.
    pub(in super::super) const IMPLEMENTED: u32 =
        0xffff_0000 | E | (3 << S_SHIFT) | U1 | U0 | (3 << CM_SHIFT) | W;
}

/// MMU status register bits (M68040UM Figure 3-6).
pub(in super::super) mod mmusr {
    /// Bit 11: a transfer error happened during the table search. "If the
    /// B-bit is set, all other bits are zero."
    pub(in super::super) const B: u32 = 0x0000_0800;
    /// Bit 10: the page descriptor's **G** bit.
    pub(in super::super) const G: u32 = 0x0000_0400;
    /// Bit 9, bit 8: the page descriptor's user attributes.
    pub(in super::super) const U1: u32 = 0x0000_0200;
    /// See [`U1`].
    pub(in super::super) const U0: u32 = 0x0000_0100;
    /// Bit 7: supervisor protected.
    pub(in super::super) const S: u32 = 0x0000_0080;
    /// Bits 6–5: the cache mode, copied from the page descriptor.
    pub(in super::super) const CM_SHIFT: u32 = 5;
    /// Bit 4: modified.
    pub(in super::super) const M: u32 = 0x0000_0010;
    /// Bit 2: write protected — set if **W** was set in *any* descriptor the
    /// search read, which "does not indicate that a violation has occurred".
    pub(in super::super) const W: u32 = 0x0000_0004;
    /// Bit 1: a transparent translation register answered.
    pub(in super::super) const T: u32 = 0x0000_0002;
    /// Bit 0: resident.
    pub(in super::super) const R: u32 = 0x0000_0001;
    /// Every bit the register implements. Bit 3 is drawn as `O` in Figure
    /// 3-6 and written as zero (M68000PRM §6, *PTEST* (MC68040)).
    pub(in super::super) const IMPLEMENTED: u32 =
        0xffff_f000 | B | G | U1 | U0 | S | (3 << CM_SHIFT) | M | W | T | R;
}

/// Descriptor bits (M68040UM Figures 3-11 and 3-12).
///
/// One long word in every format, and the low bits line up: **W** at 2 and
/// **U** at 3 in a table descriptor as well as a page one, so the search
/// reads them the same way at every level.
pub(super) mod desc {
    /// Bits 1–0 of a table descriptor — **UDT**. `00` and `01` are invalid,
    /// `10` and `11` resident.
    pub(super) const UDT: u32 = 3;
    /// Bits 1–0 of a page descriptor — **PDT**. `00` invalid, `01` and `11`
    /// resident, `10` indirect.
    pub(super) const PDT: u32 = 3;
    /// `PDT = 10`: bits 31–2 hold the address of the real page descriptor.
    pub(super) const PDT_INDIRECT: u32 = 2;
    /// Bit 2: write protected, in every format.
    pub(super) const W: u32 = 1 << 2;
    /// Bit 3: used, in every format.
    pub(super) const U: u32 = 1 << 3;
    /// Bit 4: modified — page descriptors only.
    pub(super) const M: u32 = 1 << 4;
    /// Bit 7: supervisor protected — page descriptors only.
    pub(super) const S: u32 = 1 << 7;
    /// Bits 10–5: the attributes a page descriptor hands to the entry —
    /// **G**, `U1`, `U0`, **S** and **CM**. **M** and **W** are accumulated
    /// separately because a table descriptor can set **W** too.
    pub(super) const ATTRIBUTES: u32 = 0x0000_07e0;
}

/// One address translation cache entry (M68040UM Figure 3-21).
///
/// The `data` half is deliberately laid out as an `MMUSR` value: `PTEST`'s
/// whole job is to report what a search put in an entry, and Figure 3-6 and
/// Figure 3-21 name the same fields. Reporting is then a copy rather than a
/// re-encoding, and the two cannot drift apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Entry040 {
    /// Bits 31–12 (or 31–13) the logical address, bit 1 valid, bit 0 `FC2`.
    pub tag: u32,
    /// The physical address and the page attributes, in `MMUSR` positions.
    pub data: u32,
}

impl Entry040 {
    /// The tag's valid bit.
    pub(super) const VALID: u32 = 1 << 1;
    /// The tag's `FC2`: set for a supervisor access.
    pub(super) const FC2: u32 = 1;

    const fn valid(self) -> bool {
        self.tag & Entry040::VALID != 0
    }

    /// Whether this entry answers for `la` in this privilege mode.
    ///
    /// "All 16 bits of this field are used in the comparison of this entry to
    /// an incoming logical address when the page size is 4 Kbytes. For
    /// 8-Kbytes pages, the least significant bit of this field is ignored"
    /// (§3.3) — which is what masking off the page offset does.
    const fn matches(self, la: u32, supervisor: bool, page_mask: u32) -> bool {
        self.valid()
            && (self.tag & Entry040::FC2 != 0) == supervisor
            && (self.tag ^ la) & !page_mask == 0
    }

    /// Whether the entry is global, and so survives `PFLUSHN`/`PFLUSHAN`.
    const fn global(self) -> bool {
        self.data & mmusr::G != 0
    }
}

/// How many sets the cache has, and how many ways each holds: "four-way
/// set-associative caches that each store 64 logical-to-physical address
/// translations" (§3.3).
pub(super) const ATC_SETS: usize = 16;
/// See [`ATC_SETS`].
pub(super) const ATC_WAYS: usize = 4;

/// The 68040's memory management registers.
///
/// `Copy` for the same reason the 68030's are: [`State`](super::exec::State)
/// is, and the register file travels by value through `regs()` and the
/// snapshot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Regs040 {
    /// Translation control (Figure 3-4).
    pub tcr: u16,
    /// User root pointer (Figure 3-3). Bits 8–0 must be zero.
    pub urp: u32,
    /// Supervisor root pointer.
    pub srp: u32,
    /// `ITT0` and `ITT1` — `IACR0`/`IACR1` on an MC68EC040.
    pub itt: [u32; 2],
    /// `DTT0` and `DTT1` — `DACR0`/`DACR1`.
    pub dtt: [u32; 2],
    /// What `PTEST` last reported (Figure 3-6).
    pub mmusr: u32,
    /// The address translation cache, sixteen sets of four ways.
    ///
    /// **Derived state**: never serialized, and dropping it costs nothing but
    /// a table search (CLAUDE.md, *Devices*) — the history bits it would have
    /// written are written again by the search that replaces it.
    pub atc: [[Entry040; ATC_WAYS]; ATC_SETS],
    /// The two-bit counter §3.3 replaces with.
    pub next: u8,
}

impl Regs040 {
    /// A cold start: everything zero.
    pub(super) const RESET: Regs040 = Regs040 {
        tcr: 0,
        urp: 0,
        srp: 0,
        itt: [0; 2],
        dtt: [0; 2],
        mmusr: 0,
        atc: [[Entry040 { tag: 0, data: 0 }; ATC_WAYS]; ATC_SETS],
        next: 0,
    };

    /// What the `RSTI` signal does: "the E-bits of the TCR and TTRs are
    /// cleared, disabling address translation ... A reset of the processor
    /// does not invalidate any entries in the ATCs or alter the page size"
    /// (§3.6.1). **P** surviving reset is the part worth spelling out: the
    /// manual says so twice, in §3.1.2 as well.
    pub(super) const fn reset_pin(&mut self) {
        self.tcr &= !tcr::E;
        self.itt[0] &= !ttr::E;
        self.itt[1] &= !ttr::E;
        self.dtt[0] &= !ttr::E;
        self.dtt[1] &= !ttr::E;
    }

    /// Whether paged translation is switched on.
    #[inline]
    pub(super) const fn enabled(&self) -> bool {
        self.tcr & tcr::E != 0
    }

    /// Whether the unit has anything to say about an access at all.
    ///
    /// Not the same question as [`Regs040::enabled`]: "the TTRs operate
    /// independently of the E-bit in the TCR" (§3.1.3), so a transparently
    /// translated block is still write protected with paged translation
    /// switched off — and an MC68EC040, which has no **E** bit to set, has
    /// nothing *but* transparent translation. With all five enables clear
    /// every address is its own and the fast path costs one branch.
    #[inline]
    pub(super) const fn active(&self) -> bool {
        self.tcr & tcr::E != 0
            || (self.itt[0] | self.itt[1] | self.dtt[0] | self.dtt[1]) & ttr::E != 0
    }

    /// The page size's `log2`: 12 for 4 KiB, 13 for 8 KiB (§3.1.2, **P**).
    #[inline]
    pub(super) const fn page_bits(&self) -> u32 {
        if self.tcr & tcr::P != 0 { 13 } else { 12 }
    }

    /// The mask of the bits a page offset occupies.
    #[inline]
    pub(super) const fn page_mask(&self) -> u32 {
        (1u32 << self.page_bits()) - 1
    }

    /// Which root pointer an access uses: the supervisor one for a
    /// supervisor access, the user one otherwise (§3.1.1).
    pub(super) const fn root(&self, supervisor: bool) -> u32 {
        if supervisor { self.srp } else { self.urp }
    }

    /// Which pair of transparent translation registers an access consults:
    /// the instruction pair for a program-space fetch, the data pair
    /// otherwise (§3.4).
    ///
    /// Everything but an instruction prefetch goes through the data memory
    /// unit — "the instruction memory unit is only used for instruction
    /// prefetches, [so] different instruction and data TTRs can cause PC
    /// relative operand fetches to be translated differently from
    /// instruction prefetches".
    pub(super) const fn ttr_pair(&self, program: bool) -> &[u32; 2] {
        if program { &self.itt } else { &self.dtt }
    }

    /// Which set of the cache a logical address indexes: "the four bits of
    /// the logical address located just above the page offset" (§3.3).
    #[inline]
    const fn set_of(&self, la: u32) -> usize {
        ((la >> self.page_bits()) & (ATC_SETS as u32 - 1)) as usize
    }

    /// Find the entry that answers for `la`, if there is one.
    pub(super) fn lookup(&self, la: u32, supervisor: bool) -> Option<Entry040> {
        let mask = self.page_mask();
        self.atc[self.set_of(la)]
            .iter()
            .copied()
            .find(|e| e.matches(la, supervisor, mask))
    }

    /// Put `entry` in the cache, replacing whatever already answers for the
    /// same logical address and privilege mode.
    pub(super) fn install(&mut self, entry: Entry040) {
        let mask = self.page_mask();
        let la = entry.tag & !mask;
        let supervisor = entry.tag & Entry040::FC2 != 0;
        let set = self.set_of(la);
        let ways = &mut self.atc[set];
        let slot = match ways.iter().position(|e| e.matches(la, supervisor, mask)) {
            Some(existing) => existing,
            None => match ways.iter().position(|e| !e.valid()) {
                // "The MMU replaces an invalid entry when the ATC stores a
                // new address translation" (§3.3).
                Some(free) => free,
                None => {
                    let at = self.next as usize % ATC_WAYS;
                    self.next = self.next.wrapping_add(1);
                    at
                }
            },
        };
        ways[slot] = entry;
    }

    /// Invalidate everything: `PFLUSHA` (M68000PRM §6, *PFLUSH* (MC68040)).
    pub(super) const fn flush_all(&mut self) {
        let mut set = 0;
        while set < ATC_SETS {
            let mut way = 0;
            while way < ATC_WAYS {
                self.atc[set][way].tag &= !Entry040::VALID;
                way += 1;
            }
            set += 1;
        }
    }

    /// `PFLUSHAN`: everything that is not global.
    pub(super) fn flush_non_global(&mut self) {
        for set in &mut self.atc {
            for entry in set {
                if !entry.global() {
                    entry.tag &= !Entry040::VALID;
                }
            }
        }
    }

    /// `PFLUSH (An)` and `PFLUSHN (An)`: the entry for one page in one
    /// privilege mode, optionally sparing global entries.
    pub(super) fn flush_page(&mut self, la: u32, supervisor: bool, global_too: bool) {
        let mask = self.page_mask();
        for entry in &mut self.atc[self.set_of(la)] {
            if entry.matches(la, supervisor, mask) && (global_too || !entry.global()) {
                entry.tag &= !Entry040::VALID;
            }
        }
    }

    /// The physical address an entry gives for a logical one.
    #[inline]
    pub(super) const fn physical(&self, entry: Entry040, la: u32) -> u32 {
        let mask = self.page_mask();
        (entry.data & !mask) | (la & mask)
    }
}

/// One long-word access to physical memory, supplied by the caller.
///
/// `None` for the value means a read, and the result is the long word, or
/// `None` if the bus refused. A write returns `Some(0)` when it lands.
pub(super) type BusLong<'a> = dyn FnMut(u32, Option<u32>) -> Option<u32> + 'a;

/// What a table search found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Found040 {
    /// The entry the search produced, ready for the cache. Its **R** bit is
    /// clear when the search did not reach a resident page descriptor, and a
    /// later access through it takes an access error (§3.5).
    pub entry: Entry040,
    /// The bus refused a descriptor fetch or a history write-back. "The
    /// B-bit is set if a transfer error is encountered during the table
    /// search ... If the B-bit is set, all other bits are zero" (§3.1.4).
    pub bus_error: bool,
}

impl Found040 {
    /// The `MMUSR` value a `PTEST` reports (§3.1.4; M68000PRM §6, *PTEST*
    /// (MC68040)).
    pub(super) const fn mmusr(&self) -> u32 {
        if self.bus_error {
            return mmusr::B;
        }
        self.entry.data & mmusr::IMPLEMENTED
    }
}

/// Run one table search (M68040UM §3.2).
///
/// Three levels, always: the root index is logical address bits 31-25, the
/// pointer index bits 24-18, and the page index bits 17-12 for 4 KiB pages
/// or 17-13 for 8 KiB ones, each scaled by four into its table. Figure
/// 3-13's worked example is the test beside this.
///
/// `write` asks for the page's **M** bit, and `supervisor` decides both which
/// root pointer is used and whether a page descriptor's **S** bit is a
/// violation.
pub(super) fn search(
    regs: &Regs040,
    la: u32,
    supervisor: bool,
    write: bool,
    bus: &mut BusLong<'_>,
) -> Found040 {
    let page_bits = regs.page_bits();
    let mut out = Found040 {
        entry: Entry040 {
            // Valid, because an entry is created even for a page that is not
            // resident: "when an invalid descriptor is encountered, an ATC
            // entry is created for the logical address with the resident bit
            // in the MMUSR clear" (§3.2.2.3). **R** is what tells them apart.
            tag: (la & !regs.page_mask()) | Entry040::VALID | u32::from(supervisor),
            data: 0,
        },
        bus_error: false,
    };
    // **W** accumulates down the tree: it "is set when a W-bit is set in any
    // of the descriptors encountered during the table search" (§3.3).
    let mut write_protected = false;
    let mut at = regs.root(supervisor);
    // The two table levels. Their descriptors have the same format; only how
    // far the next table's address reaches differs, and that follows from how
    // big the next table is (Figure 3-11).
    for level in 0..2u32 {
        let index = if level == 0 {
            la >> 25
        } else {
            (la >> 18) & 0x7f
        };
        // Wrapping: a root pointer near the top of the address space wraps
        // into it, as every other physical address here does.
        let address = at.wrapping_add(index * 4);
        let Some(descriptor) = bus(address, None) else {
            out.bus_error = true;
            return out;
        };
        write_protected |= descriptor & desc::W != 0;
        if descriptor & desc::UDT < 2 {
            // "00 or 01 = Invalid ... All other bits in the descriptor are
            // ignored" (§3.2.2.3, **UDT**).
            return out;
        }
        // "The processor automatically sets this bit when a descriptor is
        // accessed in which the U-bit is clear ... The processor never clears
        // this bit" (§3.2.2.3, **U**). Table 3-1's read-modify-write and the
        // `LOCK` it asserts are not modelled; the write itself is.
        if descriptor & desc::U == 0 && bus(address, Some(descriptor | desc::U)).is_none() {
            out.bus_error = true;
            return out;
        }
        // The next table's base. A root table and a pointer table are both
        // 128 entries of four bytes, so a pointer table is 512-byte aligned;
        // a page table is 64 or 32 entries, so 256- or 128-byte aligned.
        let align = if level == 0 {
            0x1ff
        } else if page_bits == 12 {
            0xff
        } else {
            0x7f
        };
        at = descriptor & !align;
    }
    // The page level.
    let index = (la >> page_bits) & if page_bits == 12 { 0x3f } else { 0x1f };
    let mut address = at.wrapping_add(index * 4);
    let Some(mut descriptor) = bus(address, None) else {
        out.bus_error = true;
        return out;
    };
    if descriptor & desc::PDT == desc::PDT_INDIRECT {
        // "Bits 31-2 contain the physical address of the page descriptor"
        // (§3.2.2.3, **PDT**). The indirect descriptor carries nothing else:
        // no W, no U, no M, which is the point — "the modified indication is
        // maintained only in the single descriptor" (§3.2.4.1).
        address = descriptor & !3;
        let Some(real) = bus(address, None) else {
            out.bus_error = true;
            return out;
        };
        // "This encoding is invalid for a page descriptor pointed to by an
        // indirect descriptor", and invalid is what it is treated as.
        if real & desc::PDT == desc::PDT_INDIRECT {
            return out;
        }
        descriptor = real;
    }
    write_protected |= descriptor & desc::W != 0;
    if descriptor & desc::PDT == 0 {
        return out;
    }
    let supervisor_violation = descriptor & desc::S != 0 && !supervisor;
    // Table 3-1: **U** is always set if clear, and **M** is set for a write
    // only when the accumulated write protection and the supervisor check
    // both allow it — "the processor sets the bit if the table search does
    // not encounter a set W-bit or a supervisor violation" (§3.2.5). "The
    // M68040 never clears this bit."
    let mut update = descriptor | desc::U;
    if write && !write_protected && !supervisor_violation {
        update |= desc::M;
    }
    if update != descriptor && bus(address, Some(update)).is_none() {
        out.bus_error = true;
        return out;
    }
    descriptor = update;
    // The entry's attributes, in `MMUSR` positions — which is where the page
    // descriptor already keeps **G**, `U1`, `U0`, **S** and **CM**, and one
    // bit away from where it keeps **M** and **W**.
    out.entry.data = (descriptor & !regs.page_mask())
        | (descriptor & desc::ATTRIBUTES)
        | (if descriptor & desc::M != 0 {
            mmusr::M
        } else {
            0
        })
        | (if write_protected { mmusr::W } else { 0 })
        | mmusr::R;
    out
}

/// What a transparent translation register match gives back.
///
/// The `CM`, `U1`/`U0` and `W` bits, so a caller does not have to know which
/// of the pair answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Transparent {
    /// The cache mode, 0-3 (§3.1.3, **CM**).
    pub cm: u8,
    /// The two user page attributes, `U1` in bit 1 and `U0` in bit 0.
    pub upa: u8,
    /// Whether the block is write protected.
    pub write_protected: bool,
}

/// Whether one transparent translation register answers for an access
/// (§3.1.3, §3.4).
///
/// The **S** field decides which privilege modes match: `00` user only, `01`
/// supervisor only, `1x` both.
pub(super) fn transparent_match(reg: u32, la: u32, supervisor: bool) -> bool {
    if reg & ttr::E == 0 {
        return false;
    }
    let privilege_ok = match (reg >> ttr::S_SHIFT) & 3 {
        0 => !supervisor,
        1 => supervisor,
        _ => true,
    };
    if !privilege_ok {
        return false;
    }
    let base = (reg >> 24) as u8;
    let mask = (reg >> 16) as u8;
    (base ^ (la >> 24) as u8) & !mask == 0
}

/// What a register says about a block it matched.
pub(super) const fn transparent_of(reg: u32) -> Transparent {
    Transparent {
        cm: ((reg >> ttr::CM_SHIFT) & 3) as u8,
        upa: (((reg & ttr::U1) >> 8) | ((reg & ttr::U0) >> 8)) as u8,
        write_protected: reg & ttr::W != 0,
    }
}

/// Which of a pair of transparent translation registers answers, if either.
///
/// "If both registers match, the TT0 status bits are used for the access"
/// (§3.4), which is why this is a search over the pair rather than an OR of
/// their attributes the way the 68030's is.
pub(super) fn transparent(pair: &[u32; 2], la: u32, supervisor: bool) -> Option<Transparent> {
    if transparent_match(pair[0], la, supervisor) {
        return Some(transparent_of(pair[0]));
    }
    if transparent_match(pair[1], la, supervisor) {
        return Some(transparent_of(pair[1]));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_clears_the_enables_and_leaves_the_page_size() {
        let mut regs = Regs040 {
            tcr: tcr::E | tcr::P,
            itt: [ttr::E; 2],
            dtt: [ttr::E; 2],
            ..Regs040::RESET
        };
        regs.reset_pin();
        assert!(!regs.enabled());
        // §3.6.1: "RSTI does not affect the P-bit of the TCR."
        assert_eq!(regs.tcr, tcr::P);
        assert_eq!(regs.itt, [0; 2]);
        assert_eq!(regs.dtt, [0; 2]);
    }
}
