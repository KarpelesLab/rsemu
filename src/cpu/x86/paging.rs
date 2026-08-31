//! Two-level paging and the translation-lookaside buffer.
//!
//! The 80386's page unit sits *behind* segmentation: `segment:offset` becomes
//! a 32-bit linear address, and only then does paging turn that into a
//! physical one. Everything here operates on linear addresses, and the
//! physical addresses it produces are what reach [`AddressSpace`].
//!
//! The translation is a two-level walk. Bits 31-22 index a page directory
//! whose base is in `CR3`; bits 21-12 index the page table that entry names;
//! bits 11-0 are the offset within the 4 KiB page. There are no large pages —
//! that is `CR4.PSE` on a Pentium — and no physical address extension, so a
//! physical address is 32 bits and nothing wider is representable.
//!
//! # Why the TLB is modelled rather than skipped
//!
//! A walk is two memory reads, so caching them is a large speed-up; but that
//! is not the reason it is here. The reason is that the accessed and dirty
//! bits are written *by the walk*, and software can see when they are written.
//! A model with no TLB writes them on every access, which is wrong in a way
//! that a demand-paging kernel notices. A model with one writes them once,
//! which is what hardware does.
//!
//! The TLB is **derived state**: it is re-derivable from the page tables, so
//! CLAUDE.md's rule applies and it is never serialized. It is flushed on
//! every write to `CR3`, on `INVLPG`, on any change to `CR0.PG` or `CR0.WP`,
//! on a task switch that reloads `CR3`, and whenever the address space's
//! topology generation changes underneath us.
//!
//! # Sources
//!
//! Intel's *80386 Programmer's Reference Manual*, chapter 5.2 ("Page
//! Translation") and chapter 9.9 ("Page Fault"), for the entry format, the
//! walk, the accessed/dirty rules and the error code's three bits.
//!
//! [`AddressSpace`]: crate::core::space::AddressSpace

/// The bits of a page directory or page table entry.
pub mod pte {
    /// Present. When clear, every other bit is software's to use.
    pub const PRESENT: u32 = 1 << 0;
    /// Writable.
    pub const WRITABLE: u32 = 1 << 1;
    /// User: accessible at privilege level 3.
    pub const USER: u32 = 1 << 2;
    /// Write-through (80486).
    pub const PWT: u32 = 1 << 3;
    /// Cache disable (80486).
    pub const PCD: u32 = 1 << 4;
    /// Accessed: set by the processor on any use of this entry.
    pub const ACCESSED: u32 = 1 << 5;
    /// Dirty: set by the processor on a write through this page table entry.
    /// The architecture leaves it undefined in a *directory* entry.
    pub const DIRTY: u32 = 1 << 6;
    /// The frame address: the entry with its flags masked off.
    pub const FRAME: u32 = 0xffff_f000;
}

/// The three bits of a page-fault error code.
pub mod pf {
    /// Set when the fault was a protection violation rather than a
    /// not-present page. The distinction matters: a kernel's fault handler
    /// branches on it before anything else.
    pub const PROTECTION: u32 = 1 << 0;
    /// Set when the access that faulted was a write.
    pub const WRITE: u32 = 1 << 1;
    /// Set when the access that faulted was made at privilege level 3.
    pub const USER: u32 = 1 << 2;
}

/// How many entries the translation-lookaside buffer holds.
///
/// A power of two so the index is a mask. Thirty-two entries is a 386's
/// figure; the exact number is not architectural — software cannot count the
/// entries, only observe that translations are cached at all — so this is a
/// speed/footprint choice rather than a fidelity one.
pub const TLB_ENTRIES: usize = 32;

/// One cached translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TlbEntry {
    /// The linear page number, or [`TlbEntry::EMPTY`].
    pub page: u32,
    /// The physical frame address, already masked.
    pub frame: u32,
    /// The `U/S` bit, ANDed across the directory and table entries.
    pub user: bool,
    /// The `R/W` bit, ANDed across the directory and table entries.
    pub writable: bool,
    /// Whether the dirty bit is already set in the page table entry. When it
    /// is not, a write has to go the long way round to set it.
    pub dirty: bool,
}

impl TlbEntry {
    /// The tag that means "nothing cached here".
    ///
    /// A linear page number is 20 bits, so `u32::MAX` cannot collide with a
    /// real one.
    pub const EMPTY: u32 = u32::MAX;

    /// An empty entry.
    #[must_use]
    pub const fn empty() -> TlbEntry {
        TlbEntry {
            page: TlbEntry::EMPTY,
            frame: 0,
            user: false,
            writable: false,
            dirty: false,
        }
    }
}

/// A direct-mapped translation-lookaside buffer.
///
/// Direct-mapped rather than associative because the replacement policy is not
/// observable and a deterministic index keeps the whole thing reproducible:
/// the same guest run evicts the same entries on every host, which is what
/// `ROADMAP.md` §0 asks of everything in the emulation core.
#[derive(Debug, Clone, Copy)]
pub struct Tlb {
    entries: [TlbEntry; TLB_ENTRIES],
    /// The address-space topology generation these translations were taken
    /// under. A remap invalidates them all.
    generation: u64,
}

impl Tlb {
    /// An empty buffer.
    #[must_use]
    pub const fn new() -> Tlb {
        Tlb {
            entries: [TlbEntry::empty(); TLB_ENTRIES],
            generation: 0,
        }
    }

    /// Discard every cached translation.
    pub const fn flush(&mut self) {
        let mut i = 0;
        while i < TLB_ENTRIES {
            self.entries[i] = TlbEntry::empty();
            i += 1;
        }
    }

    /// Discard the translation for one linear address, as `INVLPG` does.
    pub const fn invalidate(&mut self, linear: u32) {
        let page = linear >> 12;
        let slot = (page as usize) % TLB_ENTRIES;
        if self.entries[slot].page == page {
            self.entries[slot] = TlbEntry::empty();
        }
    }

    /// Discard everything if the address space has been remapped since these
    /// translations were taken.
    ///
    /// Called before every lookup. The generation counter is read without a
    /// lock by design (`core::space`), so this is one relaxed load per access.
    pub const fn sync(&mut self, generation: u64) {
        if self.generation != generation {
            self.generation = generation;
            self.flush();
        }
    }

    /// Look one linear page up.
    #[inline]
    #[must_use]
    pub const fn get(&self, linear: u32) -> Option<TlbEntry> {
        let page = linear >> 12;
        let entry = self.entries[(page as usize) % TLB_ENTRIES];
        if entry.page == page {
            Some(entry)
        } else {
            None
        }
    }

    /// Record a translation.
    pub const fn insert(&mut self, entry: TlbEntry) {
        self.entries[(entry.page as usize) % TLB_ENTRIES] = entry;
    }

    /// How many entries hold a translation. Diagnostics only — software
    /// cannot see this, and nothing guest-visible may depend on it.
    #[must_use]
    pub fn occupancy(&self) -> usize {
        self.entries
            .iter()
            .filter(|e| e.page != TlbEntry::EMPTY)
            .count()
    }
}

impl Default for Tlb {
    fn default() -> Self {
        Tlb::new()
    }
}

/// The page-fault error code for one failed access.
///
/// `present` is whether the page *was* present — a protection violation — as
/// opposed to a missing translation.
#[must_use]
pub const fn fault_code(present: bool, write: bool, user: bool) -> u32 {
    let mut code = 0;
    if present {
        code |= pf::PROTECTION;
    }
    if write {
        code |= pf::WRITE;
    }
    if user {
        code |= pf::USER;
    }
    code
}

/// Whether a write is permitted through an entry with these effective bits.
///
/// The supervisor case is the interesting one. On a 386 the kernel may write
/// any present page whatever the read-only bit says; the 486 added `CR0.WP`
/// so that it could be made to obey, which is what copy-on-write in kernel
/// space needs. Passing `wp` false therefore models a 386 exactly.
#[must_use]
pub const fn write_allowed(writable: bool, user_access: bool, wp: bool) -> bool {
    if writable {
        return true;
    }
    !user_access && !wp
}

// ---------------------------------------------------------------------------
// The walk
// ---------------------------------------------------------------------------

use super::exec::{Ex, Exec, Fault, VEC_PF};
use super::prot::cr0;

impl Exec<'_> {
    /// Turn a linear address into a physical one.
    ///
    /// `write` and `user` describe the access being made, not the page: they
    /// decide which permission bits are consulted and what the error code
    /// says if the access is refused.
    ///
    /// The caller checks [`super::prot::Sys::paging`] first — an unpaged
    /// processor has no translation to do and no accessed bits to set, and
    /// going through here anyway would charge two phantom bus cycles per
    /// access.
    pub(super) fn translate(&mut self, linear: u32, write: bool, user: bool) -> Ex<u32> {
        let generation = self.mem.generation();
        self.state.tlb.sync(generation);
        let wp = self.variant().has_486_extras() && self.state.sys.cr0 & cr0::WP != 0;

        if let Some(entry) = self.state.tlb.get(linear) {
            let allowed =
                (!user || entry.user) && (!write || write_allowed(entry.writable, user, wp));
            if allowed && (!write || entry.dirty) {
                return Ok(entry.frame | (linear & 0xfff));
            }
            if !allowed {
                self.state.sys.cr2 = linear;
                return Err(Fault::coded(VEC_PF, fault_code(true, write, user)));
            }
            // Permitted, but the dirty bit is not set yet. Fall through to the
            // walk so that the write reaches the page table, because software
            // can see when it does.
        }

        let dir_base = self.state.sys.cr3 & pte::FRAME;
        let dir_addr = dir_base.wrapping_add((linear >> 22) * 4);
        let dir = self.phys_read(dir_addr, 4);
        if dir & pte::PRESENT == 0 {
            self.state.sys.cr2 = linear;
            return Err(Fault::coded(VEC_PF, fault_code(false, write, user)));
        }
        let table_addr = (dir & pte::FRAME).wrapping_add(((linear >> 12) & 0x3ff) * 4);
        let table = self.phys_read(table_addr, 4);
        if table & pte::PRESENT == 0 {
            self.state.sys.cr2 = linear;
            return Err(Fault::coded(VEC_PF, fault_code(false, write, user)));
        }

        // The effective permissions are the **conjunction** of the two levels:
        // a user-accessible page inside a supervisor-only directory entry is
        // not user-accessible.
        let user_ok = dir & pte::USER != 0 && table & pte::USER != 0;
        let writable = dir & pte::WRITABLE != 0 && table & pte::WRITABLE != 0;
        if (user && !user_ok) || (write && !write_allowed(writable, user, wp)) {
            self.state.sys.cr2 = linear;
            return Err(Fault::coded(VEC_PF, fault_code(true, write, user)));
        }

        // The accessed and dirty bits are written by the walk itself, which is
        // why a translation-lookaside buffer has to exist here: without one,
        // every access would rewrite them and a kernel watching for the write
        // would see the wrong thing.
        if dir & pte::ACCESSED == 0 {
            self.phys_write(dir_addr, 4, dir | pte::ACCESSED);
        }
        let mut table_bits = table;
        let mut want = pte::ACCESSED;
        if write {
            want |= pte::DIRTY;
        }
        if table_bits & want != want {
            table_bits |= want;
            self.phys_write(table_addr, 4, table_bits);
        }

        self.state.tlb.insert(TlbEntry {
            page: linear >> 12,
            frame: table_bits & pte::FRAME,
            user: user_ok,
            writable,
            dirty: table_bits & pte::DIRTY != 0,
        });
        Ok((table_bits & pte::FRAME) | (linear & 0xfff))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_error_code_names_the_three_things_a_handler_branches_on() {
        assert_eq!(fault_code(false, false, false), 0);
        assert_eq!(fault_code(true, false, false), 1);
        assert_eq!(fault_code(false, true, false), 2);
        assert_eq!(fault_code(true, true, true), 7);
    }

    #[test]
    fn the_kernel_may_write_a_read_only_page_only_without_write_protect() {
        // A 386 has no CR0.WP, so this is what a 386 does.
        assert!(write_allowed(false, false, false));
        // A 486 with WP set obeys the page tables even in ring 0.
        assert!(!write_allowed(false, false, true));
        // User space never gets the exemption.
        assert!(!write_allowed(false, true, false));
        assert!(write_allowed(true, true, true));
    }

    #[test]
    fn a_lookup_misses_after_a_flush_and_after_an_invalidate() {
        let mut tlb = Tlb::new();
        let entry = TlbEntry {
            page: 0x1_2345,
            frame: 0x9000_0000,
            user: true,
            writable: true,
            dirty: true,
        };
        tlb.insert(entry);
        assert_eq!(tlb.get(0x1234_5678).map(|e| e.frame), Some(0x9000_0000));
        // A different page in the same slot is a miss, not a wrong hit.
        assert!(
            tlb.get(0x1234_5678 + (TLB_ENTRIES as u32) * 0x1000)
                .is_none()
        );
        tlb.invalidate(0x1234_5000);
        assert!(tlb.get(0x1234_5678).is_none());

        tlb.insert(entry);
        assert_eq!(tlb.occupancy(), 1);
        tlb.flush();
        assert_eq!(tlb.occupancy(), 0);
    }

    #[test]
    fn a_topology_change_invalidates_every_translation() {
        let mut tlb = Tlb::new();
        tlb.sync(7);
        tlb.insert(TlbEntry {
            page: 4,
            frame: 0x4000,
            user: false,
            writable: true,
            dirty: false,
        });
        assert!(tlb.get(0x4000).is_some());
        tlb.sync(7);
        assert!(tlb.get(0x4000).is_some());
        tlb.sync(8);
        assert!(tlb.get(0x4000).is_none());
    }
}
