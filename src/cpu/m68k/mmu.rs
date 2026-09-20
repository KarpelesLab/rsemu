//! The MC68030's paged memory management unit.
//!
//! Everything here is MC68030UM Section 9, and the parts that matter are
//! specified precisely enough to be got precisely right — which is why this
//! is a module of its own rather than a hundred lines inside `exec.rs`:
//!
//! - the register layouts, Figures 9-35 to 9-38;
//! - the six descriptor formats, Figures 9-9 to 9-18;
//! - the table search, Figure 9-25, with its three subroutines: the search
//!   initialisation (9-26), the ATC entry creation (9-27), the limit check
//!   (9-28) and the descriptor fetch with its history-bit updates (9-29);
//! - the address translation cache, §9.4;
//! - transparent translation, §9.3.
//!
//! # What is here and what is in `exec.rs`
//!
//! Everything that does not touch the bus. [`search`] is the table walk and
//! it *does* touch the bus, so it takes a callback: the caller hands it a
//! function that reads or writes one long word at a **physical** address, and
//! `exec.rs` supplies one that drives the real bus, charges its cycles and
//! logs it. A table search is visible on the bus on hardware, and it is
//! visible here.
//!
//! # The two places the manual does not decide for us
//!
//! Both are recorded rather than quietly resolved:
//!
//! 1. **The limit check at the root.** §9.7.1 says a root pointer whose
//!    descriptor type is `$1` — an early-termination page descriptor — has
//!    its limit checked "regardless of the state of the FCL bit", while
//!    Figure 9-28 returns without checking when `y = 'RP'` and `FCL = 1`, and
//!    §9.7.2's description of **FCL** agrees with the flowchart ("the limit
//!    field of CRP or SRP is ignored"). **This module follows Figure 9-28**,
//!    because the flowcharts are the detailed specification and two of the
//!    three statements agree with them.
//! 2. **The replacement policy.** §9.4 calls it "a pseudo least recently used
//!    algorithm" and does not publish it. This module fills an invalid entry
//!    when there is one and otherwise cycles round-robin, which is *a*
//!    replacement policy and not *that* one. It is observable only through
//!    `PTEST` level 0 after more than twenty-two pages have been touched.
//!
//! # What is not modelled
//!
//! - **`CIOUT`.** A descriptor's cache-inhibit bit is read, accumulated and
//!   kept in the ATC entry, and `PTEST` can see it, but there is no cache to
//!   inhibit and no pin to drive.
//! - **`MMUDIS`.** There is no such input here; `TC`'s **E** bit is the only
//!   switch.
//! - **`RMC` across a table search.** A search is indivisible on hardware
//!   (§9.5.2); here the core holds its own bus lock across the whole
//!   instruction, which is stronger, and nothing drives `RMC`.
//! - **Burst descriptor fetches.** Every descriptor is read as long words on
//!   the same 16-bit-at-a-time bus every other access uses.

/// Translation control register bits (MC68030UM Figure 9-36).
pub(super) mod tc {
    /// Bit 31: translation enable.
    pub(in super::super) const E: u32 = 0x8000_0000;
    /// Bit 25: supervisor root pointer enable.
    pub(super) const SRE: u32 = 0x0200_0000;
    /// Bit 24: function code lookup.
    pub(super) const FCL: u32 = 0x0100_0000;
    /// Every bit the register implements; "all unimplemented fields of this
    /// register are read as zeros and must always be written as zeros".
    pub(in super::super) const IMPLEMENTED: u32 = E | SRE | FCL | 0x00ff_ffff;
}

/// Transparent translation register bits (MC68030UM Figure 9-37).
pub(super) mod tt {
    /// Bit 15: enable.
    pub(super) const E: u32 = 0x0000_8000;
    /// Bit 10: cache inhibit.
    pub(super) const CI: u32 = 0x0000_0400;
    /// Bit 9: read/write — set makes *reads* transparent.
    pub(super) const RW: u32 = 0x0000_0200;
    /// Bit 8: read/write mask — set ignores [`RW`].
    pub(super) const RWM: u32 = 0x0000_0100;
    /// Every bit the register implements: the address base and mask, those
    /// four, and the two three-bit function-code fields.
    pub(in super::super) const IMPLEMENTED: u32 = 0xffff_0000 | E | CI | RW | RWM | 0x0077;
}

/// MMU status register bits (MC68030UM Figure 9-38).
pub(in super::super) mod mmusr {
    /// Bit 15: bus error.
    pub(in super::super) const B: u16 = 0x8000;
    /// Bit 14: limit violation.
    pub(in super::super) const L: u16 = 0x4000;
    /// Bit 13: supervisor-only violation.
    pub(in super::super) const S: u16 = 0x2000;
    /// Bit 11: write protected.
    pub(in super::super) const W: u16 = 0x0800;
    /// Bit 10: invalid translation.
    pub(in super::super) const I: u16 = 0x0400;
    /// Bit 9: modified.
    pub(in super::super) const M: u16 = 0x0200;
    /// Bit 6: a transparent translation register matched. On an MC68EC030
    /// this is the only bit, and it is called **AC** (MC68EC030UM Figure
    /// 9-4).
    pub(in super::super) const T: u16 = 0x0040;
    /// Bits 2–0: how many tables the search read.
    pub(in super::super) const N: u16 = 0x0007;
    /// Every bit the register implements.
    pub(in super::super) const IMPLEMENTED: u16 = B | L | S | W | I | M | T | N;
}

/// A descriptor's type field: the two low bits of every format
/// (MC68030UM §9.5.1.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dt {
    /// `$0` — invalid.
    Invalid,
    /// `$1` — a page descriptor, wherever it is found.
    Page,
    /// `$2` — the next table holds four-byte descriptors.
    Short,
    /// `$3` — the next table holds eight-byte descriptors.
    Long,
}

impl Dt {
    const fn of(word: u32) -> Dt {
        match word & 3 {
            0 => Dt::Invalid,
            1 => Dt::Page,
            2 => Dt::Short,
            _ => Dt::Long,
        }
    }

    /// The number of bytes a descriptor in the table this one names occupies,
    /// which is also the scale factor for that table's index.
    const fn table_width(self) -> u32 {
        match self {
            Dt::Long => 8,
            _ => 4,
        }
    }
}

/// Descriptor status bits, at the positions the formats share.
///
/// **U**, **WP**, **M** and **CI** sit in the same places in the short
/// formats' single long word and in the long formats' upper one (Figures 9-10
/// to 9-14), which is what lets one decoder read both. **S** is the only bit
/// a short descriptor does not have.
mod desc {
    /// Bit 2: write protect.
    pub(super) const WP: u32 = 1 << 2;
    /// Bit 3: used.
    pub(super) const U: u32 = 1 << 3;
    /// Bit 4: modified — page descriptors only.
    pub(super) const M: u32 = 1 << 4;
    /// Bit 6: cache inhibit — page descriptors only.
    pub(super) const CI: u32 = 1 << 6;
    /// Bit 8: supervisor only — long formats only.
    pub(super) const S: u32 = 1 << 8;
}

/// How many entries the address translation cache holds (MC68030UM §9.4: "a
/// 22-entry fully associative ... cache").
pub(super) const ATC_ENTRIES: usize = 22;

/// One address translation cache entry, packed the way the hardware packs it.
///
/// The logical half is a validity bit, three function-code bits and the top
/// twenty-four logical address bits; the physical half is **B**, **CI**,
/// **WP**, **M** and the top twenty-four physical address bits (§9.4). Both
/// halves fit in a `u32` because a page is at least 256 bytes, so the low
/// eight bits of each address are not part of the entry and are free for the
/// flags — which is the shape the manual draws.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Entry {
    /// Valid (bit 3), function code (bits 2–0), logical address (31–8).
    pub tag: u32,
    /// Bus error (3), cache inhibit (2), write protect (1), modified (0),
    /// physical address (31–8).
    pub data: u32,
}

impl Entry {
    /// The tag's valid bit.
    pub(super) const VALID: u32 = 1 << 3;
    /// The data half's bus-error bit.
    pub(super) const BERR: u32 = 1 << 3;
    /// Cache inhibit.
    pub(super) const CI: u32 = 1 << 2;
    /// Write protect.
    pub(super) const WP: u32 = 1 << 1;
    /// Modified.
    pub(super) const M: u32 = 1;

    const fn valid(self) -> bool {
        self.tag & Entry::VALID != 0
    }

    /// Whether this entry answers for `la` with function code `fc`.
    ///
    /// "All 24 bits of this field are used in the comparison ... when the
    /// page size is 256 bytes. For larger page sizes, the appropriate number
    /// of least significant bits of this field are ignored" (§9.4).
    const fn matches(self, la: u32, fc: u8, page_mask: u32) -> bool {
        self.valid()
            && self.tag & 7 == (fc & 7) as u32
            && (self.tag ^ la) & !page_mask & 0xffff_ff00 == 0
    }
}

/// The MMU's programmer-visible registers and its cache.
///
/// `Copy` because [`State`](super::exec::State) is: the register file travels
/// by value through `regs()` and the snapshot path, and a cache of
/// twenty-two eight-byte entries does not change that.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Mmu {
    /// Translation control (Figure 9-36).
    pub tc: u32,
    /// CPU root pointer, the whole 64-bit descriptor (Figure 9-35).
    pub crp: u64,
    /// Supervisor root pointer.
    pub srp: u64,
    /// The two transparent translation registers, `TT0` then `TT1` —
    /// `AC0` and `AC1` on an MC68EC030.
    pub tt: [u32; 2],
    /// The status register `PTEST` writes.
    pub mmusr: u16,
    /// The address translation cache. **Derived state**: never serialized,
    /// and dropping it costs nothing but a table search (CLAUDE.md,
    /// *Devices*) — the history bits it would have written are written again
    /// by the search that replaces it.
    pub atc: [Entry; ATC_ENTRIES],
    /// Where the replacement algorithm looks next.
    pub next: u8,
}

impl Default for Mmu {
    fn default() -> Mmu {
        Mmu::RESET
    }
}

impl Mmu {
    /// A cold start: everything zero.
    pub(super) const RESET: Mmu = Mmu {
        tc: 0,
        crp: 0,
        srp: 0,
        tt: [0; 2],
        mmusr: 0,
        atc: [Entry { tag: 0, data: 0 }; ATC_ENTRIES],
        next: 0,
    };

    /// What the `RESET` signal does: "the E bits of the TC and TTx registers
    /// are cleared, disabling address translation ... A reset of the
    /// processor does not invalidate any entries in the ATC" (§9.2.2).
    pub(super) const fn reset_pin(&mut self) {
        self.tc &= !tc::E;
        self.tt[0] &= !tt::E;
        self.tt[1] &= !tt::E;
    }

    /// Whether translation is switched on.
    #[inline]
    pub(super) const fn enabled(&self) -> bool {
        self.tc & tc::E != 0
    }

    /// The page size's `log2`, from `TC`'s **PS** field.
    ///
    /// Only 8–15 are legal and `PMOVE` refuses anything else with a
    /// configuration exception (§9.7.2), so this never sees one — but a
    /// register that was never written reads zero, and a zero here would make
    /// the masks below nonsense, so it is clamped.
    #[inline]
    pub(super) const fn page_bits(&self) -> u32 {
        let ps = (self.tc >> 20) & 0xf;
        if ps < 8 { 8 } else { ps }
    }

    /// The mask of the bits a page offset occupies.
    #[inline]
    pub(super) const fn page_mask(&self) -> u32 {
        (1u32 << self.page_bits()) - 1
    }

    /// The initial shift: how many high logical address bits the search
    /// ignores.
    const fn initial_shift(&self) -> u32 {
        (self.tc >> 16) & 0xf
    }

    /// The four table index widths, `TIA` to `TID`.
    const fn index_widths(&self) -> [u32; 4] {
        [
            (self.tc >> 12) & 0xf,
            (self.tc >> 8) & 0xf,
            (self.tc >> 4) & 0xf,
            self.tc & 0xf,
        ]
    }

    /// Whether the first table is indexed by the function code.
    const fn fcl(&self) -> bool {
        self.tc & tc::FCL != 0
    }

    /// Whether a value would be a legal `TC` — checked only when **E** is
    /// being set, because that is when the manual checks it (§9.7.2).
    ///
    /// "The TIx fields are added together until a zero field is reached, and
    /// this sum is added to PS and IS. The total must be 32", and `PS` must
    /// be one of the eight defined sizes.
    pub(super) fn tc_is_consistent(value: u32) -> bool {
        let ps = (value >> 20) & 0xf;
        if !(8..=15).contains(&ps) {
            return false;
        }
        let mut total = ps + ((value >> 16) & 0xf);
        for shift in [12, 8, 4, 0] {
            let width = (value >> shift) & 0xf;
            if width == 0 {
                break;
            }
            total += width;
        }
        total == 32
    }

    /// Which root pointer an access uses (Table 9-2: the supervisor one only
    /// when **SRE** and **FC2** are both set).
    const fn root(&self, fc: u8) -> u64 {
        if self.tc & tc::SRE != 0 && fc & 4 != 0 {
            self.srp
        } else {
            self.crp
        }
    }

    /// Whether either transparent translation register answers for this
    /// access, and whether it inhibits caching (§9.3).
    ///
    /// The two are checked independently and their **CI** bits are ORed when
    /// both match.
    pub(super) fn transparent(&self, la: u32, fc: u8, write: bool) -> Option<bool> {
        let mut matched = false;
        let mut ci = false;
        for reg in self.tt {
            if transparent_match(reg, la, fc, write) {
                matched = true;
                ci |= reg & tt::CI != 0;
            }
        }
        matched.then_some(ci)
    }

    /// Find the cache entry that answers for `la`, if there is one.
    pub(super) fn lookup(&self, la: u32, fc: u8) -> Option<Entry> {
        let mask = self.page_mask();
        self.atc
            .iter()
            .copied()
            .find(|entry| entry.matches(la, fc, mask))
    }

    /// Invalidate everything: `PFLUSHA`, and the flush a `PMOVE` with **FD**
    /// clear performs (§9.7.5.1).
    pub(super) const fn flush_all(&mut self) {
        let mut i = 0;
        while i < ATC_ENTRIES {
            self.atc[i].tag &= !Entry::VALID;
            i += 1;
        }
    }

    /// Invalidate every entry whose function code matches `fc` under `mask`.
    ///
    /// A **set** mask bit means the corresponding function-code bit *applies*
    /// and a clear one means it is ignored: "Ones in the mask correspond to
    /// applicable bits; zeros are bits to be ignored" (M68000PRM §6,
    /// *PFLUSH*'s **Mask** field). The worked example in the same page is the
    /// test beside this.
    pub(super) fn flush_fc(&mut self, fc: u8, mask: u8) {
        for entry in &mut self.atc {
            if (entry.tag as u8 ^ fc) & mask & 7 == 0 {
                entry.tag &= !Entry::VALID;
            }
        }
    }

    /// The same, restricted to the page one logical address is in.
    pub(super) fn flush_fc_address(&mut self, fc: u8, mask: u8, la: u32) {
        let page = !self.page_mask() & 0xffff_ff00;
        for entry in &mut self.atc {
            let fc_ok = (entry.tag as u8 ^ fc) & mask & 7 == 0;
            if fc_ok && (entry.tag ^ la) & page == 0 {
                entry.tag &= !Entry::VALID;
            }
        }
    }

    /// Put `entry` in the cache, replacing whatever already answers for the
    /// same logical address and function code.
    pub(super) fn install(&mut self, entry: Entry) {
        let mask = self.page_mask();
        let la = entry.tag & 0xffff_ff00;
        let fc = (entry.tag & 7) as u8;
        let slot = match self.atc.iter().position(|e| e.matches(la, fc, mask)) {
            Some(existing) => existing,
            None => match self.atc.iter().position(|e| !e.valid()) {
                Some(free) => free,
                None => {
                    let at = self.next as usize % ATC_ENTRIES;
                    self.next = ((at + 1) % ATC_ENTRIES) as u8;
                    at
                }
            },
        };
        self.atc[slot] = entry;
    }

    /// The physical address an entry gives for a logical one.
    #[inline]
    pub(super) const fn physical(&self, entry: Entry, la: u32) -> u32 {
        let mask = self.page_mask();
        (entry.data & !mask & 0xffff_ff00) | (la & mask)
    }
}

/// Whether one transparent translation register answers for an access.
fn transparent_match(reg: u32, la: u32, fc: u8, write: bool) -> bool {
    if reg & tt::E == 0 {
        return false;
    }
    let base = (reg >> 24) as u8;
    let mask = (reg >> 16) as u8;
    if (base ^ (la >> 24) as u8) & !mask != 0 {
        return false;
    }
    let fc_base = ((reg >> 4) & 7) as u8;
    let fc_mask = (reg & 7) as u8;
    if (fc_base ^ (fc & 7)) & !fc_mask != 0 {
        return false;
    }
    // With RWM clear the register answers for one direction only, and "for
    // transparent translation of read-modify-write cycles ... RWM must be set
    // to one. If the RWM bit equals zero, neither the read nor the write of
    // any read-modify-write cycle is transparently translated" (§9.7.3) —
    // which this core cannot distinguish, because its read-modify-write
    // instructions issue ordinary reads and writes.
    if reg & tt::RWM != 0 {
        return true;
    }
    (reg & tt::RW != 0) != write
}

/// What a table search found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Found {
    /// The entry the search produced, ready for the cache. An entry with
    /// [`Entry::BERR`] set is the manual's "invalid ATC entry (B bit set)":
    /// a later access to it takes a bus error.
    pub entry: Entry,
    /// How many descriptors the search read — `MMUSR`'s **N** field.
    pub levels: u8,
    /// A limit field rejected an index.
    pub limit: bool,
    /// The bus refused a descriptor fetch or a history-bit write-back.
    pub bus_error: bool,
    /// A long-format descriptor's **S** bit forbade this access.
    pub supervisor: bool,
    /// A descriptor's type field was `$0`.
    pub invalid: bool,
    /// The physical address of the last descriptor the search read, which
    /// `PTEST` can be asked to hand back in an address register.
    pub last_descriptor: u32,
    /// The search stopped because it had read as many tables as it was
    /// allowed, not because it reached a page descriptor. Only a `PTEST`
    /// whose level field is shallower than the tree can see this, and it
    /// means there is nothing to put in the cache.
    pub capped: bool,
}

impl Found {
    /// The `MMUSR` value a `PTEST` of level 1–7 reports (Table 9-3).
    pub(super) const fn mmusr(&self) -> u16 {
        let mut out = (self.levels as u16) & mmusr::N;
        if self.bus_error {
            out |= mmusr::B;
        }
        if self.limit {
            out |= mmusr::L;
        }
        if self.supervisor {
            out |= mmusr::S;
        }
        // "The W bit is undefined if the I bit is set" — this core reports
        // what it accrued rather than leaving it undefined, which is a
        // superset of what the manual promises.
        if self.entry.data & Entry::WP != 0 {
            out |= mmusr::W;
        }
        if self.invalid || self.bus_error || self.limit {
            out |= mmusr::I;
        }
        if self.entry.data & Entry::M != 0 {
            out |= mmusr::M;
        }
        out
    }
}

/// One long-word access to physical memory, supplied by the caller.
///
/// `None` for the value means a read, and the result is the long word, or
/// `None` if the bus refused. A write returns `Some(0)` when it lands.
pub(super) type BusLong<'a> = dyn FnMut(u32, Option<u32>) -> Option<u32> + 'a;

/// A descriptor as the search holds it: its status long word and, for the
/// long formats, the second one.
#[derive(Clone, Copy)]
struct Desc {
    status: u32,
    low: u32,
    /// Four or eight.
    width: u32,
}

impl Desc {
    const fn dt(self) -> Dt {
        Dt::of(self.status)
    }

    /// The table address a table descriptor names: bits 31–4 of the short
    /// format's only long word, or of the long format's second one.
    const fn table_address(self) -> u32 {
        (if self.width == 8 {
            self.low
        } else {
            self.status
        }) & !0xf
    }

    /// The page address a page descriptor names: bits 31–8, with the bits
    /// below the page size dropped ("when the page size is larger than 256
    /// bytes, one or more of the least significant bits of this field are not
    /// used", §9.5.1.1).
    const fn page_address(self, page_mask: u32) -> u32 {
        (if self.width == 8 {
            self.low
        } else {
            self.status
        }) & 0xffff_ff00
            & !page_mask
    }

    /// The descriptor address an indirect descriptor names: bits 31–2 of the
    /// short format's long word, or of the long format's second one (Figures
    /// 9-17, 9-18).
    const fn descriptor_address(self) -> u32 {
        (if self.width == 8 {
            self.low
        } else {
            self.status
        }) & !3
    }

    /// Whether this is a long format, which is what carries a limit and an
    /// **S** bit.
    const fn is_long(self) -> bool {
        self.width == 8
    }
}

/// Whether an index is outside a long-format descriptor's limit
/// (Figure 9-28).
///
/// With **L/U** clear the limit is an upper one and an index above it is out
/// of bounds; with it set the limit is a lower one and an index below it is.
fn limit_violated(status: u32, index: u32) -> bool {
    let lower = status & 0x8000_0000 != 0;
    let limit = (status >> 16) & 0x7fff;
    if lower { index < limit } else { index > limit }
}

/// Walk the translation tree for one logical address (Figure 9-25).
///
/// `max_levels` caps how many tables are read, which is what `PTEST`'s level
/// field asks for; seven is the whole tree. `bus` reads and writes long words
/// at **physical** addresses — the history bits the search writes back go
/// through it too, which is why it can fail there as well.
///
/// The caller installs [`Found::entry`] unless this was a `PTEST`.
pub(super) fn search(
    mmu: &Mmu,
    la: u32,
    fc: u8,
    write: bool,
    max_levels: u8,
    bus: &mut BusLong<'_>,
) -> Found {
    let page_mask = mmu.page_mask();
    let mut out = Found {
        entry: Entry {
            tag: (la & !page_mask & 0xffff_ff00) | u32::from(fc & 7) | Entry::VALID,
            data: 0,
        },
        ..Found::default()
    };

    // Figure 9-26, "initialize accrued status": the write-protect and
    // supervisor bits start clear and are ORed along the way. The cache
    // inhibit is *assigned* from the page descriptor, because only a page
    // descriptor has one.
    let mut acc = Accrued {
        wp: false,
        ci: false,
        supervisor: false,
        user: fc & 4 == 0,
    };

    let root = mmu.root(fc);
    let root_status = (root >> 32) as u32;
    // The root pointer is always a long-format descriptor, so it always has
    // a limit and `LAST_SIZE` starts at eight.
    let mut last = Desc {
        status: root_status,
        low: root as u32,
        width: 8,
    };
    let mut table = (root as u32) & !0xf;
    let mut width = last.dt().table_width();

    let widths = mmu.index_widths();
    // The bit above the next index. Bits above this were dropped by the
    // initial shift and take no part in the search.
    let mut top = 32u32.saturating_sub(mmu.initial_shift());

    match last.dt() {
        Dt::Invalid => {
            // `PMOVE` refuses an invalid root pointer with a configuration
            // exception before it can ever be used (§9.7.5.3); reaching here
            // means the register was never written.
            out.invalid = true;
            return fail(out);
        }
        Dt::Page => {
            // Early termination at the root: "the MC68030 internally
            // calculates an ATC entry (page descriptor) ... by adding
            // (unsigned) the value in the table address field to the incoming
            // logical address" (§9.7.1).
            if !mmu.fcl() && limit_violated(root_status, index(la, top, widths[0])) {
                out.limit = true;
                return fail(out);
            }
            return complete(out, table, la, top, page_mask, &acc, write);
        }
        _ => {}
    }

    // The optional function-code level. It is never limit-checked: "when
    // function code lookup is enabled ... the limit field of CRP or SRP is
    // ignored" (§9.7.2).
    if mmu.fcl() {
        let at = table.wrapping_add(u32::from(fc & 7) * width);
        let Some(desc) = read_descriptor(bus, at, width, &mut out) else {
            return fail(out);
        };
        out.levels = out.levels.saturating_add(1);
        out.last_descriptor = at;
        match desc.dt() {
            Dt::Invalid => {
                out.invalid = true;
                return fail(out);
            }
            Dt::Page => {
                if !history(bus, at, desc, write, &mut acc, &mut out) {
                    return fail(out);
                }
                accrue(&mut acc, desc, true);
                if acc.violated() {
                    out.supervisor = true;
                    return fail(out);
                }
                return complete(
                    out,
                    desc.page_address(page_mask),
                    la,
                    top,
                    page_mask,
                    &acc,
                    write,
                );
            }
            next => {
                if !history(bus, at, desc, write, &mut acc, &mut out) {
                    return fail(out);
                }
                accrue(&mut acc, desc, false);
                if acc.violated() {
                    out.supervisor = true;
                    return fail(out);
                }
                last = desc;
                table = desc.table_address();
                width = next.table_width();
            }
        }
    }

    // The four `TIx` levels, A to D.
    let mut level = 0usize;
    loop {
        if out.levels >= max_levels {
            // A `PTEST` that asked for fewer levels than the tree has. There
            // is no translation, but nothing was *found* invalid either, so
            // the entry is marked unusable and `MMUSR`'s **I** stays clear.
            out.capped = true;
            return fail(out);
        }
        let this = index(la, top, widths[level]);
        // Figure 9-28: only a long-format descriptor carries a limit.
        if last.is_long() && limit_violated(last.status, this) {
            out.limit = true;
            return fail(out);
        }
        top = top.saturating_sub(widths[level]);

        let at = table.wrapping_add(this * width);
        let Some(desc) = read_descriptor(bus, at, width, &mut out) else {
            return fail(out);
        };
        out.levels = out.levels.saturating_add(1);
        out.last_descriptor = at;

        match desc.dt() {
            Dt::Invalid => {
                out.invalid = true;
                return fail(out);
            }
            Dt::Page => {
                // Normal termination if this was the last level, early
                // termination if there are more `TIx` fields below it — the
                // difference is only whether there are unused logical address
                // bits left to add, which `complete` handles either way.
                if !history(bus, at, desc, write, &mut acc, &mut out) {
                    return fail(out);
                }
                accrue(&mut acc, desc, true);
                if acc.violated() {
                    out.supervisor = true;
                    return fail(out);
                }
                return complete(
                    out,
                    desc.page_address(page_mask),
                    la,
                    top,
                    page_mask,
                    &acc,
                    write,
                );
            }
            next => {
                if !history(bus, at, desc, write, &mut acc, &mut out) {
                    return fail(out);
                }
                accrue(&mut acc, desc, false);
                if acc.violated() {
                    out.supervisor = true;
                    return fail(out);
                }
                let more = level < 3 && widths[level + 1] != 0;
                if !more {
                    // "No more TIx fields (must be indirect)": the descriptor
                    // at the bottom of the tree that is still a table
                    // descriptor points at a page descriptor somewhere else.
                    let at = desc.descriptor_address();
                    let Some(pointed) = read_descriptor(bus, at, next.table_width(), &mut out)
                    else {
                        return fail(out);
                    };
                    out.levels = out.levels.saturating_add(1);
                    out.last_descriptor = at;
                    if pointed.dt() != Dt::Page {
                        out.invalid = true;
                        return fail(out);
                    }
                    if !history(bus, at, pointed, write, &mut acc, &mut out) {
                        return fail(out);
                    }
                    accrue(&mut acc, pointed, true);
                    if acc.violated() {
                        out.supervisor = true;
                        return fail(out);
                    }
                    return complete(
                        out,
                        pointed.page_address(page_mask),
                        la,
                        top,
                        page_mask,
                        &acc,
                        write,
                    );
                }
                last = desc;
                table = desc.table_address();
                width = next.table_width();
                level += 1;
            }
        }
    }
}

/// The accrued status a search carries down the tree (Figure 9-26).
struct Accrued {
    wp: bool,
    ci: bool,
    supervisor: bool,
    user: bool,
}

impl Accrued {
    /// Whether a supervisor-only descriptor has been reached from user state.
    const fn violated(&self) -> bool {
        self.supervisor && self.user
    }
}

/// The index a level takes out of the logical address.
fn index(la: u32, top: u32, width: u32) -> u32 {
    if width == 0 || width > 15 {
        return 0;
    }
    (la >> top.saturating_sub(width)) & ((1u32 << width) - 1)
}

/// Fold one descriptor's protection bits into the accrued status
/// (Figure 9-29's `ACC_STATUS` assignments).
fn accrue(acc: &mut Accrued, desc: Desc, page: bool) {
    acc.wp |= desc.status & desc::WP != 0;
    if desc.is_long() {
        acc.supervisor |= desc.status & desc::S != 0;
    }
    if page {
        // Assigned rather than ORed: only a page descriptor has one.
        acc.ci = desc.status & desc::CI != 0;
    }
}

/// Read a four- or eight-byte descriptor.
fn read_descriptor(bus: &mut BusLong<'_>, at: u32, width: u32, out: &mut Found) -> Option<Desc> {
    let Some(status) = bus(at, None) else {
        out.bus_error = true;
        return None;
    };
    let low = if width == 8 {
        match bus(at.wrapping_add(4), None) {
            Some(word) => word,
            None => {
                out.bus_error = true;
                return None;
            }
        }
    } else {
        0
    };
    Some(Desc { status, low, width })
}

/// Write the **U** and **M** history bits back, as Figure 9-29 does.
///
/// Returns false if the bus refused the write-back, which aborts the search
/// exactly as a failed fetch does.
///
/// The two exceptions the flowchart does not draw are in §9.5.1.1's prose and
/// are honoured here: **U** is not set "after a supervisor violation is
/// detected", and **M** is not set "after a descriptor with the WP bit set is
/// encountered, or after a supervisor violation is encountered". The
/// write-protect test includes *this* descriptor's own bit, because a write
/// to a page this descriptor protects will fault and never modify it.
fn history(
    bus: &mut BusLong<'_>,
    at: u32,
    desc: Desc,
    write: bool,
    acc: &mut Accrued,
    out: &mut Found,
) -> bool {
    if acc.violated() {
        return true;
    }
    let page = desc.dt() == Dt::Page;
    let protected = acc.wp || desc.status & desc::WP != 0;
    let mut status = desc.status;
    if status & desc::U == 0 {
        status |= desc::U;
    }
    if page && write && !protected {
        status |= desc::M;
    }
    if status == desc.status {
        return true;
    }
    if bus(at, Some(status)).is_none() {
        out.bus_error = true;
        return false;
    }
    true
}

/// Turn a terminated search into a valid ATC entry (Figure 9-27's right-hand
/// half).
///
/// `base` is the address field of the last descriptor read and `top` is the
/// bit above the logical address bits nothing consumed. For a search that
/// reached the bottom of the tree there are none, and the addition is a
/// no-op; for an early termination they are the offset within the block the
/// descriptor maps. Figure 9-19 calls the addition signed, which on two
/// 32-bit values is `wrapping_add`.
fn complete(
    mut out: Found,
    base: u32,
    la: u32,
    top: u32,
    page_mask: u32,
    acc: &Accrued,
    write: bool,
) -> Found {
    let unused = if top >= 32 {
        la
    } else {
        la & ((1u32 << top) - 1)
    } & !page_mask;
    let frame = base.wrapping_add(unused) & !page_mask & 0xffff_ff00;
    let mut data = frame;
    if acc.wp {
        data |= Entry::WP;
    }
    if acc.ci {
        data |= Entry::CI;
    }
    // The search that set **M** in memory is the search that puts it in the
    // entry, so the write that caused it does not come straight back for
    // another one (§9.4, **M**).
    if write && !acc.wp {
        data |= Entry::M;
    }
    out.entry.data = data;
    out
}

/// Mark a search as having produced no translation: the manual's "create
/// invalid ATC entry (B bit set)".
const fn fail(mut out: Found) -> Found {
    out.entry.data |= Entry::BERR;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_translation_control_consistency_check_is_the_manuals() {
        // §9.7.2: "The values in the Tlx fields are added until the first
        // zero is encountered. The values in the PS and IS fields are added
        // to the sum of the Tlx fields. If the sum is not equal to 32, the
        // PMOVE instruction causes an MMU configuration exception."
        // IS=0, TIA=7, TIB=7, TIC=6, TID=0, PS=12 (4K): 12+0+7+7+6 = 32.
        assert!(Mmu::tc_is_consistent(
            tc::E | (12 << 20) | (7 << 12) | (7 << 8) | (6 << 4)
        ));
        // The canonical 4K, two-level tree: IS 0, TIA 10, TIB 10, PS 12.
        assert!(Mmu::tc_is_consistent(
            tc::E | (12 << 20) | (10 << 12) | (10 << 8)
        ));
        // One bit too many.
        assert!(!Mmu::tc_is_consistent(
            tc::E | (12 << 20) | (11 << 12) | (10 << 8)
        ));
        // A page size the manual reserves.
        assert!(!Mmu::tc_is_consistent(
            tc::E | (7 << 20) | (10 << 12) | (10 << 8)
        ));
        // Fields after the first zero take no part.
        assert!(Mmu::tc_is_consistent(
            tc::E | (13 << 20) | (10 << 12) | (9 << 8) | 0xf
        ));
    }

    #[test]
    fn a_transparent_register_matches_the_manuals_worked_example() {
        // §9.3: "to transparently translate supervisor data read accesses of
        // addresses $00000000-$0FFFFFFF, the LOGICAL BASE ADDRESS field is
        // set to $0X, the LOGICAL ADDRESS MASK is set to $0F, the R/W bit is
        // set to 1, the RWM bit is set to 0, the FC BASE is set to $5, and
        // the FC MASK field is set to $0."
        // Base $00 in bits 31-24, mask $0f in 23-16.
        let reg = (0x0f << 16) | tt::E | tt::RW | (5 << 4);
        assert!(transparent_match(reg, 0x0000_0000, 5, false));
        assert!(transparent_match(reg, 0x0fff_ffff, 5, false));
        assert!(!transparent_match(reg, 0x1000_0000, 5, false), "outside");
        assert!(!transparent_match(reg, 0x0000_0000, 5, true), "a write");
        assert!(!transparent_match(reg, 0x0000_0000, 1, false), "user data");
        // "to transparently translate user program space ... the RWM bit of
        // the register is set to 1, the FC BASE is set to $2, and the FC MASK
        // is set to $0." The example says nothing about the address fields,
        // so an all-ones address mask covers the whole space — which is what
        // "transparently translate user program space" must mean.
        let program = tt::E | tt::RWM | (2 << 4) | (0xff << 16);
        assert!(transparent_match(program, 0xdead_beef, 2, false));
        assert!(transparent_match(program, 0xdead_beef, 2, true));
        assert!(!transparent_match(program, 0xdead_beef, 6, false));
        // A disabled register is ignored entirely.
        assert!(!transparent_match(program & !tt::E, 0, 2, false));
    }

    #[test]
    fn a_limit_is_upper_or_lower_by_its_flag() {
        // §9.5.1.1: "When the LlU bit is set, the LIMIT field contains the
        // unsigned lower limit; the index value ... must be greater than or
        // equal to the value in the LIMIT field. When the bit is cleared, the
        // limit is an unsigned upper limit, and the index value must be less
        // than or equal to the LIMIT."
        let upper = 0x0010_0000; // L/U clear, LIMIT = $10
        assert!(!limit_violated(upper, 0x10));
        assert!(limit_violated(upper, 0x11));
        let lower = 0x8010_0000; // L/U set, LIMIT = $10
        assert!(!limit_violated(lower, 0x10));
        assert!(limit_violated(lower, 0x0f));
        // "To suppress the limit function, the LlU bit is cleared and the
        // limit field is set to ones ($7FFF ...), or the LlU bit is set and
        // the limit field is cleared ($8000 ...)."
        assert!(!limit_violated(0x7fff_0000, 0x7fff));
        assert!(!limit_violated(0x8000_0000, 0));
    }

    #[test]
    fn the_cache_matches_on_the_bits_the_page_size_leaves() {
        let mut mmu = Mmu::RESET;
        // 4K pages: the low twelve logical bits are the offset.
        mmu.tc = tc::E | (12 << 20) | (10 << 12) | (10 << 8);
        mmu.install(Entry {
            tag: 0x1234_5000 | Entry::VALID | 5,
            data: 0x00ab_c000,
        });
        assert!(mmu.lookup(0x1234_5000, 5).is_some());
        assert!(mmu.lookup(0x1234_5fff, 5).is_some(), "same page");
        assert!(mmu.lookup(0x1234_6000, 5).is_none(), "the next page");
        assert!(mmu.lookup(0x1234_5000, 1).is_none(), "another space");
        let entry = mmu.lookup(0x1234_5abc, 5).expect("a hit");
        assert_eq!(mmu.physical(entry, 0x1234_5abc), 0x00ab_cabc);
        mmu.flush_all();
        assert!(mmu.lookup(0x1234_5000, 5).is_none());
    }

    #[test]
    fn a_flush_selects_by_function_code_under_its_mask() {
        let mut mmu = Mmu::RESET;
        mmu.tc = tc::E | (12 << 20) | (10 << 12) | (10 << 8);
        for fc in 0..8u8 {
            mmu.install(Entry {
                tag: 0x1000_0000 | Entry::VALID | u32::from(fc),
                data: 0,
            });
        }
        // "a mask operand of 100 causes the instruction to consider only the
        // most significant bit of the FC operand. If the FC operand is 001,
        // function codes 000, 001, 010, and 011 are selected."
        mmu.flush_fc(0b001, 0b100);
        for fc in 0..4u8 {
            assert!(mmu.lookup(0x1000_0000, fc).is_none(), "fc {fc} flushed");
        }
        for fc in 4..8u8 {
            assert!(mmu.lookup(0x1000_0000, fc).is_some(), "fc {fc} kept");
        }
    }
}
