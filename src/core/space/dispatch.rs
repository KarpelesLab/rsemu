//! Page-granular dispatch: the first level of lookup (`ROADMAP.md` §4.1).
//!
//! Two costs are budgeted here rather than discovered later.
//!
//! **Size.** A dense table over the low 4 GiB is ~10⁶ entries. The roadmap
//! budgets 16 B each — 16 MiB per space, which on `wasm32` is half a percent
//! of the entire linear memory *per bus master*. This implementation spends
//! **4 bytes**: an entry is a tagged `u32` naming a [`FlatView`] index, and the
//! flat entry it names already holds the store handle, the offset cell and the
//! constraints. So the low 4 GiB costs 4 MiB, and the table is still
//! **opt-in per space** ([`DispatchPolicy`]) because a dozen masters at 4 MiB
//! is still 48 MiB nobody asked for.
//!
//! **Granularity.** A page cannot express every mapping: the NES puts the APU
//! at `$4000` for 32 bytes, and PC I/O ports are byte-granular. So an entry can
//! say [`DispatchEntry::SubPage`] — "several things live in this page, consult
//! the flat view" — which is also the honest reason the NES gets no benefit
//! from a dense table at all.

use super::flat::FlatView;
use alloc::vec::Vec;

/// Whether an address space gets a dense page table, and how big.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum DispatchPolicy {
    /// No table: every lookup is a binary search over the flat view. The
    /// default, and the right answer for a 64 KiB machine.
    #[default]
    Flat,
    /// A dense table of `1 << page_bits` pages covering `[0, cover)`.
    Dense {
        /// Page size, as a shift.
        page_bits: u32,
        /// Bytes of address space the table covers. Above it, lookups fall
        /// back to the flat view.
        cover: u64,
    },
    /// Build a table if the realized extent makes one cheap.
    ///
    /// The rule: 4 KiB pages covering the flat view's extent, but only if that
    /// is at most [`Dispatch::MAX_ENTRIES`] entries and the view has enough
    /// entries for a binary search to cost anything.
    Auto,
}

/// What a dispatch lookup yields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchEntry {
    /// Nothing is mapped anywhere in this page; apply the space's unassigned
    /// policy without touching the flat view.
    Unassigned,
    /// More than one thing lives in this page, or something covers only part
    /// of it: consult the flat view.
    SubPage,
    /// One flat entry covers the whole page. The index is into
    /// [`FlatView::entries`].
    Mapped(u32),
    /// As [`DispatchEntry::Mapped`], and the entry is plain RAM.
    ///
    /// This is the fast path: the caller knows without a virtual call, a
    /// binary search, or a match that the access is a store offset away. It is
    /// the safe stand-in for the roadmap's "host pointer + length" — see
    /// [`RamStore`](super::RamStore) for why there is no raw pointer here.
    Direct(u32),
}

/// A dense page-granular dispatch table.
#[derive(Debug)]
pub struct Dispatch {
    page_bits: u32,
    cover: u64,
    table: Vec<u32>,
}

const TAG_SHIFT: u32 = 30;
const TAG_MAPPED: u32 = 0;
const TAG_DIRECT: u32 = 1 << TAG_SHIFT;
const TAG_SUBPAGE: u32 = 2 << TAG_SHIFT;
const TAG_UNASSIGNED: u32 = 3 << TAG_SHIFT;
const INDEX_MASK: u32 = (1 << TAG_SHIFT) - 1;

impl Dispatch {
    /// The largest table this will build: 4 MiB of `u32`s.
    pub const MAX_ENTRIES: u64 = 1 << 20;

    /// Build a table for `view` under `policy`, or `None` if the policy asks
    /// for no table or the table would be too big.
    #[must_use]
    pub fn build(view: &FlatView, policy: DispatchPolicy) -> Option<Dispatch> {
        let (page_bits, cover) = match policy {
            DispatchPolicy::Flat => return None,
            DispatchPolicy::Dense { page_bits, cover } => (page_bits, cover),
            DispatchPolicy::Auto => {
                // Below a handful of entries the binary search is two
                // comparisons and a table is pure overhead.
                if view.len() < 8 {
                    return None;
                }
                (12, view.extent())
            }
        };
        if page_bits == 0 || page_bits >= 64 || cover == 0 {
            return None;
        }
        let pages = cover.div_ceil(1u64 << page_bits);
        if pages > Self::MAX_ENTRIES {
            return None;
        }
        let n = usize::try_from(pages).ok()?;
        let page_size = 1u64 << page_bits;
        let mut table = Vec::with_capacity(n);
        for page in 0..pages {
            let start = page << page_bits;
            let end = start.saturating_add(page_size);
            table.push(match view.find(start) {
                Some(i) => {
                    let e = view.entry(i).expect("index from find");
                    if e.end() >= end {
                        let tag = if e.is_direct_ram() {
                            TAG_DIRECT
                        } else {
                            TAG_MAPPED
                        };
                        match u32::try_from(i) {
                            Ok(i) if i <= INDEX_MASK => tag | i,
                            _ => TAG_SUBPAGE,
                        }
                    } else {
                        TAG_SUBPAGE
                    }
                }
                None => {
                    // Nothing at the page start: either the page is entirely
                    // empty, or something starts partway into it.
                    let next_start = view
                        .entries()
                        .iter()
                        .find(|e| e.start() >= start)
                        .map(super::FlatEntry::start);
                    match next_start {
                        Some(s) if s < end => TAG_SUBPAGE,
                        _ => TAG_UNASSIGNED,
                    }
                }
            });
        }
        Some(Dispatch {
            page_bits,
            cover,
            table,
        })
    }

    /// Page size in bytes.
    #[inline]
    #[must_use]
    pub fn page_size(&self) -> u64 {
        1u64 << self.page_bits
    }

    /// How much address space the table covers.
    #[inline]
    #[must_use]
    pub fn cover(&self) -> u64 {
        self.cover
    }

    /// Number of entries.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.table.len()
    }

    /// Whether the table is empty.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    /// Look `addr` up, or `None` if it is outside the covered range.
    #[inline]
    #[must_use]
    pub fn lookup(&self, addr: u64) -> Option<DispatchEntry> {
        if addr >= self.cover {
            return None;
        }
        let raw = *self
            .table
            .get(usize::try_from(addr >> self.page_bits).ok()?)?;
        Some(match raw & !INDEX_MASK {
            TAG_DIRECT => DispatchEntry::Direct(raw & INDEX_MASK),
            TAG_SUBPAGE => DispatchEntry::SubPage,
            TAG_UNASSIGNED => DispatchEntry::Unassigned,
            _ => DispatchEntry::Mapped(raw & INDEX_MASK),
        })
    }
}
