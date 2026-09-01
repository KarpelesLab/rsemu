//! Address translation: the two-, three- and four-level walks, and the
//! translation-lookaside buffer.
//!
//! The page unit sits *behind* segmentation: `segment:offset` becomes a linear
//! address, and only then does paging turn that into a physical one.
//! Everything here operates on linear addresses, and the physical addresses it
//! produces are what reach [`AddressSpace`].
//!
//! # One walk, four modes
//!
//! There are four paging modes and they are the *same walk* at different
//! depths, which is why [`walk`] is one function taking a [`Mode`] rather than
//! four functions that would drift apart:
//!
//! | Mode | Levels | Entry | Top table | Large pages |
//! | --- | --- | --- | --- | --- |
//! | [`Mode::Off`] | — | — | — | linear *is* physical |
//! | [`Mode::Legacy`] | 2 | 4 bytes | 1024-entry directory at `CR3` | 4 MiB with `CR4.PSE` |
//! | [`Mode::Pae`] | 3 | 8 bytes | **4**-entry pointer table at `CR3` | 2 MiB |
//! | [`Mode::Ia32e`] | 4 | 8 bytes | 512-entry `PML4` at `CR3` | 2 MiB and 1 GiB |
//!
//! The four-entry page-directory-pointer table is the one that catches people
//! out: in PAE it is indexed by two bits, is aligned to 32 bytes rather than a
//! page, and its entries carry **no** `R/W` or `U/S` bits — those appear only
//! when the same structure is reused as a level of an IA-32e walk. Modelling
//! the difference is what [`Mode::top_index_bits`] and [`Mode::level_perms`]
//! are for.
//!
//! # Why the debug walk shares the code
//!
//! `Device::debug_translate` needs the same translation with none of the side
//! effects: no accessed or dirty bit, no `CR2`, no TLB fill, no cycles
//! charged. Writing a second walk for it is how the two get out of step — one
//! gains 1 GiB pages and the other does not — so [`walk`] is parameterised by
//! how it *reads* an entry and returns the entries it touched, and the caller
//! decides whether to write anything back. The executing walk writes; the
//! debug path reads with `MemAttrs::DEBUG` and writes nothing.
//!
//! # Why the TLB is modelled rather than skipped
//!
//! A walk is two to four memory reads, so caching them is a large speed-up;
//! but that is not the reason it is here. The reason is that the accessed and
//! dirty bits are written *by the walk*, and software can see when they are
//! written. A model with no TLB writes them on every access, which is wrong in
//! a way that a demand-paging kernel notices. A model with one writes them
//! once, which is what hardware does.
//!
//! The TLB is **derived state**: it is re-derivable from the page tables, so
//! CLAUDE.md's rule applies and it is never serialized. It is flushed on every
//! write to `CR3`, on `INVLPG`, on any change to `CR0.PG`, `CR0.WP`, `CR4.PAE`
//! or `EFER.LME`, on a task switch that reloads `CR3`, and whenever the
//! address space's topology generation changes underneath us.
//!
//! # Sources
//!
//! Intel's *80386 Programmer's Reference Manual* §5.2 ("Page Translation") and
//! §9.9 ("Page Fault") for the two-level walk, the accessed/dirty rules and
//! the error code's three bits; the *Intel SDM* volume 3 chapter 4 ("Paging")
//! for PAE (§4.4), IA-32e (§4.5), the large-page forms and the error code's
//! later bits; and the *AMD64 Architecture Programmer's Manual* volume 2
//! chapter 5, which describes the four-level walk AMD designed and is clearer
//! about which fields are ignored at which level.
//!
//! [`AddressSpace`]: crate::core::space::AddressSpace

/// The bits of a page-table entry, at every level and in every mode.
///
/// One set of constants because the *positions* really are shared: what
/// changes between modes is the entry's width and which of these bits are
/// consulted, not where they sit.
pub mod pte {
    /// Present. When clear, every other bit is software's to use.
    pub const PRESENT: u64 = 1 << 0;
    /// Writable.
    pub const WRITABLE: u64 = 1 << 1;
    /// User: accessible at privilege level 3.
    pub const USER: u64 = 1 << 2;
    /// Write-through (80486).
    pub const PWT: u64 = 1 << 3;
    /// Cache disable (80486).
    pub const PCD: u64 = 1 << 4;
    /// Accessed: set by the processor on any use of this entry.
    pub const ACCESSED: u64 = 1 << 5;
    /// Dirty: set by the processor on a write through this entry. Defined
    /// only in an entry that maps a page.
    pub const DIRTY: u64 = 1 << 6;
    /// Page size: this entry maps a large page rather than naming the next
    /// table. Reserved in a level that has no large-page form.
    pub const PAGE_SIZE: u64 = 1 << 7;
    /// Global: the translation survives a `CR3` reload, when `CR4.PGE` is set.
    pub const GLOBAL: u64 = 1 << 8;
    /// Execute-disable, bit 63, when `EFER.NXE` is set. Reserved otherwise,
    /// which is why setting it without `NXE` is a reserved-bit page fault
    /// rather than a no-op.
    pub const NX: u64 = 1 << 63;

    /// The frame address in a 4-byte (legacy) entry.
    pub const FRAME32: u64 = 0xffff_f000;
    /// The frame address in an 8-byte entry, with a 52-bit physical address.
    pub const FRAME64: u64 = 0x000f_ffff_ffff_f000;
    /// The 4 MiB frame address in a legacy directory entry with `PS` set.
    ///
    /// Bits 21-13 of such an entry are the physical address's bits 39-32 —
    /// the "PSE-36" extension — which this core does not model: a 4 MiB page
    /// lives below 4 GiB here, and those bits are ignored rather than
    /// rejected.
    pub const FRAME_4M: u64 = 0xffc0_0000;
}

/// The three bits of a page-fault error code, and the two later parts added.
pub mod pf {
    /// Set when the fault was a protection violation rather than a
    /// not-present page. The distinction matters: a kernel's fault handler
    /// branches on it before anything else.
    pub const PROTECTION: u32 = 1 << 0;
    /// Set when the access that faulted was a write.
    pub const WRITE: u32 = 1 << 1;
    /// Set when the access that faulted was made at privilege level 3.
    pub const USER: u32 = 1 << 2;
    /// Set when a reserved bit was set in one of the entries the walk read.
    /// Only reachable where there *are* reserved bits, which is PAE and
    /// IA-32e.
    pub const RESERVED: u32 = 1 << 3;
    /// Set when the access that faulted was an instruction fetch and
    /// `EFER.NXE` is on.
    pub const FETCH: u32 = 1 << 4;
}

/// Which translation scheme is in force.
///
/// Derived from `CR0.PG`, `CR4.PAE` and `EFER.LMA` together — no one of the
/// three decides it — which is why this is computed in one place
/// ([`super::prot::Sys::paging_mode`]) rather than at every point of use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// `CR0.PG` clear: the linear address is the physical address.
    Off,
    /// Two levels of 4-byte entries. A 386, a 486, or any later part with
    /// `CR4.PAE` clear.
    Legacy,
    /// Three levels of 8-byte entries: `CR4.PAE` set and long mode off.
    Pae,
    /// Four levels of 8-byte entries: IA-32e paging, which long mode requires.
    Ia32e,
}

impl Mode {
    /// How many levels the walk descends, top table included.
    #[must_use]
    pub const fn levels(self) -> u8 {
        match self {
            Mode::Off => 0,
            Mode::Legacy => 2,
            Mode::Pae => 3,
            Mode::Ia32e => 4,
        }
    }

    /// How many bytes one entry occupies.
    #[must_use]
    pub const fn entry_bytes(self) -> u8 {
        match self {
            Mode::Legacy => 4,
            _ => 8,
        }
    }

    /// How many address bits index the **top** table.
    ///
    /// Ten for a legacy directory, **two** for a PAE pointer table, nine for a
    /// `PML4`. The PAE answer is the odd one and it is not a rounding of nine:
    /// the table really has four entries and really is only 32 bytes long.
    #[must_use]
    pub const fn top_index_bits(self) -> u32 {
        match self {
            Mode::Off => 0,
            Mode::Legacy => 10,
            Mode::Pae => 2,
            Mode::Ia32e => 9,
        }
    }

    /// How many address bits index every level below the top.
    #[must_use]
    pub const fn index_bits(self) -> u32 {
        match self {
            Mode::Legacy => 10,
            _ => 9,
        }
    }

    /// The mask `CR3` is taken through to find the top table.
    ///
    /// A PAE pointer table is 32-byte aligned, not page aligned, which is the
    /// one place `CR3` is not simply masked to a frame.
    #[must_use]
    pub const fn cr3_mask(self) -> u64 {
        match self {
            Mode::Off => 0,
            Mode::Legacy => pte::FRAME32,
            Mode::Pae => 0xffff_ffe0,
            Mode::Ia32e => pte::FRAME64,
        }
    }

    /// Whether entries at `level` (0 = top) carry `R/W` and `U/S` bits that
    /// take part in the permission conjunction.
    ///
    /// A PAE page-directory-pointer entry does not: bits 1, 2 and 63 are
    /// reserved there and the processor ignores them (*Intel SDM* volume 3
    /// §4.4.2, table 4-8). The identical structure in an IA-32e walk *does*
    /// carry them, which is exactly the sort of difference a shared walk has
    /// to be told rather than assume.
    #[must_use]
    pub const fn level_perms(self, level: u8) -> bool {
        !(matches!(self, Mode::Pae) && level == 0)
    }
}

/// How many entries the translation-lookaside buffer holds.
///
/// A power of two so the index is a mask. Thirty-two entries is a 386's
/// figure; the exact number is not architectural — software cannot count the
/// entries, only observe that translations are cached at all — so this is a
/// speed/footprint choice rather than a fidelity one.
pub const TLB_ENTRIES: usize = 32;

/// One cached translation, always of a 4 KiB region.
///
/// A large page fills several of these rather than one wide entry. That costs
/// a walk per 4 KiB the first time and buys a lookup that never has to ask how
/// big the page was — and since software cannot observe the buffer's contents,
/// only whether a walk happened, it is indistinguishable from the alternative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TlbEntry {
    /// The linear page number, or [`TlbEntry::EMPTY`].
    pub page: u64,
    /// The physical frame address, already masked.
    pub frame: u64,
    /// The `U/S` bit, ANDed across every level.
    pub user: bool,
    /// The `R/W` bit, ANDed across every level.
    pub writable: bool,
    /// The execute-disable bit, ORed across every level.
    pub no_execute: bool,
    /// Whether the dirty bit is already set in the entry that maps the page.
    /// When it is not, a write has to go the long way round to set it.
    pub dirty: bool,
}

impl TlbEntry {
    /// The tag that means "nothing cached here".
    ///
    /// A linear page number is at most 36 bits, so `u64::MAX` cannot collide
    /// with a real one.
    pub const EMPTY: u64 = u64::MAX;

    /// An empty entry.
    #[must_use]
    pub const fn empty() -> TlbEntry {
        TlbEntry {
            page: TlbEntry::EMPTY,
            frame: 0,
            user: false,
            writable: false,
            no_execute: false,
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
    pub const fn invalidate(&mut self, linear: u64) {
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
    pub const fn get(&self, linear: u64) -> Option<TlbEntry> {
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

/// What one access is asking for, as the walk needs to know it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Access {
    /// The access writes.
    pub write: bool,
    /// The access is made at privilege level 3.
    pub user: bool,
    /// The access is an instruction fetch, so execute-disable applies.
    pub fetch: bool,
}

impl Access {
    /// A supervisor read — how the processor walks its own structures.
    pub const SYSTEM: Access = Access {
        write: false,
        user: false,
        fetch: false,
    };

    /// A data read or write at a privilege level.
    #[must_use]
    pub const fn data(write: bool, user: bool) -> Access {
        Access {
            write,
            user,
            fetch: false,
        }
    }

    /// An instruction fetch at a privilege level.
    #[must_use]
    pub const fn fetch(user: bool) -> Access {
        Access {
            write: false,
            user,
            fetch: true,
        }
    }
}

/// The parts of the system register file a walk depends on.
///
/// Passed as a value rather than borrowed from `Sys` so that the debug walk,
/// which holds a snapshot, and the executing walk, which holds the live state,
/// go through exactly the same code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tables {
    /// Which scheme is in force.
    pub mode: Mode,
    /// `CR3`, in its raw form; the mode decides how much of it is the base.
    pub cr3: u64,
    /// `CR4.PSE`: 4 MiB pages in a legacy walk.
    pub pse: bool,
    /// `EFER.NXE`: bit 63 of an entry means execute-disable rather than
    /// reserved.
    pub nxe: bool,
    /// `CR0.WP`: the supervisor obeys the read-only bit.
    pub wp: bool,
}

/// One completed walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Walk {
    /// The physical address the linear one translates to.
    pub phys: u64,
    /// The effective `U/S` bit: the conjunction across every level that has
    /// one.
    pub user: bool,
    /// The effective `R/W` bit.
    pub writable: bool,
    /// The effective execute-disable bit: the *disjunction* across levels,
    /// because any level may forbid execution.
    pub no_execute: bool,
    /// Whether the entry that maps the page is global.
    pub global: bool,
    /// The entries the walk read, top-down: `(physical address, value)`.
    pub entries: [(u64, u64); 4],
    /// How many of `entries` are filled.
    pub depth: u8,
}

impl Walk {
    /// The entry that maps the page — the last one read.
    #[must_use]
    pub const fn leaf(&self) -> (u64, u64) {
        self.entries[(self.depth as usize) - 1]
    }
}

/// Why a walk did not produce a translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WalkFail {
    /// An entry had its present bit clear.
    NotPresent,
    /// An entry had a reserved bit set.
    Reserved,
}

/// Walk the page tables for one linear address, reading entries through
/// `read` and writing nothing.
///
/// `read` takes a physical address and an entry width in bytes. Making it a
/// parameter is what lets the executing walk charge bus cycles and the debug
/// walk use `MemAttrs::DEBUG` without either of them owning a second copy of
/// the walk itself.
///
/// The permission bits are returned rather than checked: whether an access is
/// *allowed* depends on `CR0.WP` and on whether it was a fetch, and deciding
/// that here would mean the debug walk — which must answer "where is this
/// page" regardless of permissions — could not share the code.
///
/// # Errors
///
/// [`WalkFail::NotPresent`] or [`WalkFail::Reserved`], which the caller turns
/// into a page fault with the right error code.
///
/// # Panics
///
/// If called with [`Mode::Off`], which has no walk to do.
pub fn walk<R>(t: Tables, linear: u64, read: &mut R) -> Result<Walk, WalkFail>
where
    R: FnMut(u64, u8) -> u64,
{
    let levels = t.mode.levels();
    assert!(levels > 0, "Mode::Off has no walk");
    let width = t.mode.entry_bytes();
    let mut table = t.cr3 & t.mode.cr3_mask();
    let mut user = true;
    let mut writable = true;
    let mut no_execute = false;
    let mut entries = [(0u64, 0u64); 4];

    // The shift of the top level's index. Each level below it moves down by
    // `index_bits`, so 10/10/12, 2/9/9/12 and 9/9/9/9/12 all come out of the
    // same two lines.
    let mut shift = 12 + t.mode.index_bits() * u32::from(levels - 1);
    for level in 0..levels {
        let bits = if level == 0 {
            t.mode.top_index_bits()
        } else {
            t.mode.index_bits()
        };
        let index = (linear >> shift) & ((1u64 << bits) - 1);
        let addr = table.wrapping_add(index * u64::from(width));
        let entry = read(addr, width);
        entries[level as usize] = (addr, entry);
        if entry & pte::PRESENT == 0 {
            return Err(WalkFail::NotPresent);
        }
        if reserved_set(t, entry, level, levels) {
            return Err(WalkFail::Reserved);
        }
        if t.mode.level_perms(level) {
            user &= entry & pte::USER != 0;
            writable &= entry & pte::WRITABLE != 0;
            if t.nxe {
                no_execute |= entry & pte::NX != 0;
            }
        }

        let last = level + 1 == levels;
        let large = !last && entry & pte::PAGE_SIZE != 0 && large_allowed(t, level);
        if last || large {
            let frame_mask = if width == 4 {
                if large { pte::FRAME_4M } else { pte::FRAME32 }
            } else {
                pte::FRAME64
            };
            // A large page's frame is masked at *its* granularity and the
            // offset is everything below it, computed from the shift this
            // level was reached at — so 4 MiB, 2 MiB and 1 GiB need no
            // special cases of their own.
            let page_shift = if large { shift } else { 12 };
            let page_mask = (1u64 << page_shift) - 1;
            return Ok(Walk {
                phys: ((entry & frame_mask) & !page_mask) | (linear & page_mask),
                user,
                writable,
                no_execute,
                global: entry & pte::GLOBAL != 0,
                entries,
                depth: level + 1,
            });
        }
        table = entry
            & if width == 4 {
                pte::FRAME32
            } else {
                pte::FRAME64
            };
        shift -= t.mode.index_bits();
    }
    unreachable!("the last level always returns")
}

/// Whether this entry sets a bit the mode reserves.
///
/// Two are checked, and they are the two a real guest trips over: `PS` in a
/// level that has no large-page form, and `NX` without `EFER.NXE`. The
/// physical-address bits above the part's width are not checked, because this
/// core's address space is narrower than 52 bits anyway and rejecting them
/// would fault on an address nothing could have reached.
const fn reserved_set(t: Tables, entry: u64, level: u8, levels: u8) -> bool {
    if !t.nxe && t.mode.entry_bytes() == 8 && entry & pte::NX != 0 {
        return true;
    }
    // `PS` in the last level is the page-attribute-table bit, not a size.
    if entry & pte::PAGE_SIZE != 0 && level + 1 < levels && !large_allowed(t, level) {
        return true;
    }
    false
}

/// Whether a `PS` bit at this level really means a large page.
///
/// A legacy directory entry needs `CR4.PSE`; a PAE or IA-32e directory entry
/// always may; a `PML4` entry never may; and an IA-32e pointer-table entry
/// maps a 1 GiB page, which this core supports because the walk falls out of
/// the same arithmetic and refusing would be a special case rather than a
/// saving.
const fn large_allowed(t: Tables, level: u8) -> bool {
    match t.mode {
        Mode::Off => false,
        Mode::Legacy => t.pse && level == 0,
        Mode::Pae => level == 1,
        Mode::Ia32e => level == 1 || level == 2,
    }
}

use super::exec::{Ex, Exec, Fault, VEC_PF};
use crate::core::device::DebugTranslation;
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

impl Exec<'_> {
    /// Turn a linear address into a physical one.
    ///
    /// `access` describes the access being made, not the page: it decides
    /// which permission bits are consulted and what the error code says if the
    /// access is refused.
    ///
    /// The caller checks [`super::prot::Sys::paging`] first — an unpaged
    /// processor has no translation to do and no accessed bits to set, and
    /// going through here anyway would charge phantom bus cycles per access.
    pub(super) fn translate_access(&mut self, linear: u64, access: Access) -> Ex<u64> {
        let generation = self.mem.generation();
        self.state.tlb.sync(generation);
        let t = self.state.sys.tables(self.cfg.features);
        let write = access.write;
        let user = access.user;

        if let Some(entry) = self.state.tlb.get(linear) {
            let allowed = (!user || entry.user)
                && (!write || write_allowed(entry.writable, user, t.wp))
                && !(access.fetch && entry.no_execute);
            if allowed && (!write || entry.dirty) {
                return Ok(entry.frame | (linear & 0xfff));
            }
            if !allowed {
                return Err(self.page_fault(linear, access, true, entry.no_execute));
            }
            // Permitted, but the dirty bit is not set yet. Fall through to the
            // walk so that the write reaches the page table, because software
            // can see when it does.
        }

        let walked = {
            let mut read = |addr: u64, width: u8| -> u64 { self.phys_read(addr, width) };
            match walk(t, linear, &mut read) {
                Ok(w) => w,
                Err(fail) => {
                    let present = matches!(fail, WalkFail::Reserved);
                    let mut fault = self.page_fault(linear, access, present, false);
                    if matches!(fail, WalkFail::Reserved) {
                        fault.error = fault.error.map(|e| e | pf::RESERVED);
                    }
                    return Err(fault);
                }
            }
        };

        if (user && !walked.user)
            || (write && !write_allowed(walked.writable, user, t.wp))
            || (access.fetch && walked.no_execute)
        {
            return Err(self.page_fault(linear, access, true, walked.no_execute));
        }

        // The accessed and dirty bits are written by the walk itself, which is
        // why a translation-lookaside buffer has to exist here: without one,
        // every access would rewrite them and a kernel watching for the write
        // would see the wrong thing.
        let width = t.mode.entry_bytes();
        let depth = usize::from(walked.depth);
        for (level, (addr, value)) in walked.entries.iter().copied().enumerate().take(depth) {
            let mut want = pte::ACCESSED;
            if level + 1 == depth && write {
                want |= pte::DIRTY;
            }
            if value & want != want {
                self.phys_write(addr, width, value | want);
            }
        }

        let (_, leaf) = walked.leaf();
        self.state.tlb.insert(TlbEntry {
            page: linear >> 12,
            frame: walked.phys & !0xfff,
            user: walked.user,
            writable: walked.writable,
            no_execute: walked.no_execute,
            dirty: leaf & pte::DIRTY != 0 || write,
        });
        Ok(walked.phys)
    }

    /// A data read or write — the common case, and the one every operand
    /// takes.
    pub(super) fn translate(&mut self, linear: u64, write: bool, user: bool) -> Ex<u64> {
        self.translate_access(linear, Access::data(write, user))
    }

    /// Latch `CR2` and build the page fault for a refused access.
    fn page_fault(&mut self, linear: u64, access: Access, present: bool, nx: bool) -> Fault {
        self.state.sys.cr2 = linear;
        let mut code = fault_code(present, access.write, access.user);
        // The instruction-fetch bit exists only where execute-disable does;
        // reporting it otherwise would tell a handler about a distinction its
        // processor cannot make.
        if access.fetch && nx {
            code |= pf::FETCH;
        }
        Fault::coded(VEC_PF, code)
    }
}

/// Where a linear address lives, as a debugger asks it — **without touching
/// anything**.
///
/// The executing walk cannot be reused here, and that is the whole reason
/// `Device::debug_translate` is a separate entry point: the real walk sets the
/// accessed and dirty bits, fills the TLB and latches `CR2`. Every one of
/// those is guest-visible, and a debugger that caused them would be changing
/// what it came to look at (`ROADMAP.md` §15, invariant 5). So this shares the
/// [`walk`] and not the side effects: it reads every entry with
/// `MemAttrs::DEBUG` — a page table can sit under an MMIO region — and
/// answers *where the page is*, not whether an access would be allowed. A
/// debugger showing a user page while the core is in ring 0 is doing its job.
pub(super) fn debug_translate(
    sys: &super::prot::Sys,
    features: super::Features,
    space: &AddressSpace,
    linear: u64,
) -> DebugTranslation {
    let t = sys.tables(features);
    if matches!(t.mode, Mode::Off) {
        // Not a failure to translate: with `CR0.PG` clear the linear address
        // *is* the physical one, and saying so is a different fact from "the
        // tables map nothing here".
        return DebugTranslation::Identity;
    }
    let mut failed = false;
    let mut read = |addr: u64, width: u8| -> u64 {
        let w = if width == 4 { Width::U32 } else { Width::U64 };
        match space.read(addr, w, MemAttrs::DEBUG) {
            Ok(value) => value,
            Err(_) => {
                failed = true;
                0
            }
        }
    };
    match walk(t, linear, &mut read) {
        Ok(w) if !failed => DebugTranslation::Mapped(w.phys),
        _ => DebugTranslation::Unmapped,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

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
            no_execute: false,
            dirty: true,
        };
        tlb.insert(entry);
        assert_eq!(tlb.get(0x1234_5678).map(|e| e.frame), Some(0x9000_0000));
        // A different page in the same slot is a miss, not a wrong hit.
        assert!(
            tlb.get(0x1234_5678 + (TLB_ENTRIES as u64) * 0x1000)
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
            no_execute: false,
            dirty: false,
        });
        assert!(tlb.get(0x4000).is_some());
        tlb.sync(7);
        assert!(tlb.get(0x4000).is_some());
        tlb.sync(8);
        assert!(tlb.get(0x4000).is_none());
    }

    /// Walk over a list of `(address, value)` pairs, so the four modes can be
    /// checked without a whole machine.
    fn walk_over(t: Tables, linear: u64, table: &[(u64, u64)]) -> Result<Walk, WalkFail> {
        let mut reads: Vec<u64> = Vec::new();
        let mut read = |addr: u64, _w: u8| -> u64 {
            reads.push(addr);
            table
                .iter()
                .find(|(a, _)| *a == addr)
                .map_or(0, |(_, v)| *v)
        };
        walk(t, linear, &mut read)
    }

    fn tables(mode: Mode, cr3: u64) -> Tables {
        Tables {
            mode,
            cr3,
            pse: true,
            nxe: true,
            wp: false,
        }
    }

    #[test]
    fn a_legacy_walk_indexes_ten_bits_then_ten_then_twelve() {
        // 80386 PRM §5.2: directory index is bits 31-22, table index 21-12.
        let t = tables(Mode::Legacy, 0x1000);
        let flags = pte::PRESENT | pte::WRITABLE | pte::USER;
        // 0x0080_4123: directory entry 2, table entry 4, offset 0x123.
        let w = walk_over(
            t,
            0x0080_4123,
            &[
                (0x1000 + 2 * 4, 0x2000 | flags),
                (0x2000 + 4 * 4, 0x9000 | flags),
            ],
        )
        .expect("both levels present");
        assert_eq!(w.phys, 0x9123);
        assert_eq!(w.depth, 2);
        assert!(w.user && w.writable);
    }

    #[test]
    fn a_legacy_directory_entry_with_ps_maps_four_megabytes() {
        let t = tables(Mode::Legacy, 0x1000);
        let w = walk_over(
            t,
            0x0080_4123,
            &[(
                0x1000 + 2 * 4,
                0x0080_0000 | pte::PRESENT | pte::WRITABLE | pte::PAGE_SIZE,
            )],
        )
        .expect("the directory entry maps the page itself");
        // The offset within a 4 MiB page is the low 22 bits.
        assert_eq!(w.phys, 0x0080_0000 | 0x4123);
        assert_eq!(w.depth, 1);
    }

    #[test]
    fn a_directory_entry_with_ps_and_no_cr4_pse_is_a_reserved_bit_fault() {
        let mut t = tables(Mode::Legacy, 0x1000);
        t.pse = false;
        let err = walk_over(
            t,
            0x0080_4123,
            &[(0x1000 + 2 * 4, 0x0080_0000 | pte::PRESENT | pte::PAGE_SIZE)],
        )
        .expect_err("PS without CR4.PSE is reserved");
        assert_eq!(err, WalkFail::Reserved);
    }

    #[test]
    fn a_pae_pointer_table_has_four_entries_and_no_permission_bits() {
        // SDM volume 3 §4.4.2: CR3 is 32-byte aligned, the index is two bits,
        // and a PDPTE carries no R/W or U/S.
        let t = tables(Mode::Pae, 0xff20);
        let flags = pte::PRESENT | pte::WRITABLE | pte::USER;
        let w = walk_over(
            t,
            0xc040_2123,
            &[
                // Bits 31-30 of 0xc040_2123 are 3.
                (0xff20 + 3 * 8, 0x2_0000 | pte::PRESENT),
                // Bits 29-21 are 2.
                (0x2_0000 + 2 * 8, 0x3_0000 | flags),
                // Bits 20-12 are 2.
                (0x3_0000 + 2 * 8, 0x4_0000 | flags),
            ],
        )
        .expect("three levels present");
        assert_eq!(w.phys, 0x4_0123);
        assert_eq!(w.depth, 3);
        // The pointer entry set neither bit, and it must not have narrowed the
        // permissions to nothing.
        assert!(w.user && w.writable);
    }

    #[test]
    fn an_ia32e_walk_descends_four_levels_of_nine_bits() {
        let t = tables(Mode::Ia32e, 0x1000);
        let linear = 0x0000_7f80_1234_5678u64;
        let i = |shift: u32| ((linear >> shift) & 0x1ff) * 8;
        let flags = pte::PRESENT | pte::WRITABLE | pte::USER;
        let w = walk_over(
            t,
            linear,
            &[
                (0x1000 + i(39), 0x2000 | flags),
                (0x2000 + i(30), 0x3000 | flags),
                (0x3000 + i(21), 0x4000 | flags),
                (0x4000 + i(12), 0x5000 | flags),
            ],
        )
        .expect("four levels present");
        assert_eq!(w.phys, 0x5678);
        assert_eq!(w.depth, 4);
    }

    #[test]
    fn an_ia32e_directory_entry_with_ps_maps_two_megabytes() {
        let t = tables(Mode::Ia32e, 0x1000);
        let linear = 0x0000_0000_0020_3456u64;
        let i = |shift: u32| ((linear >> shift) & 0x1ff) * 8;
        let flags = pte::PRESENT | pte::WRITABLE | pte::USER;
        let w = walk_over(
            t,
            linear,
            &[
                (0x1000 + i(39), 0x2000 | flags),
                (0x2000 + i(30), 0x3000 | flags),
                (0x3000 + i(21), 0x20_0000 | flags | pte::PAGE_SIZE),
            ],
        )
        .expect("the directory entry maps the page");
        assert_eq!(w.phys, 0x20_0000 | 0x3456);
        assert_eq!(w.depth, 3);
    }

    #[test]
    fn an_ia32e_pointer_entry_with_ps_maps_a_gigabyte() {
        let t = tables(Mode::Ia32e, 0x1000);
        let linear = 0x0000_0000_4020_3456u64;
        let i = |shift: u32| ((linear >> shift) & 0x1ff) * 8;
        let flags = pte::PRESENT | pte::WRITABLE | pte::USER;
        let w = walk_over(
            t,
            linear,
            &[
                (0x1000 + i(39), 0x2000 | flags),
                (0x2000 + i(30), 0x4000_0000 | flags | pte::PAGE_SIZE),
            ],
        )
        .expect("the pointer entry maps the page");
        assert_eq!(w.phys, 0x4000_0000 | 0x0020_3456);
        assert_eq!(w.depth, 2);
    }

    #[test]
    fn a_pml4_entry_may_not_set_ps() {
        let t = tables(Mode::Ia32e, 0x1000);
        let err = walk_over(t, 0, &[(0x1000, 0x2000 | pte::PRESENT | pte::PAGE_SIZE)])
            .expect_err("PS is reserved in a PML4 entry");
        assert_eq!(err, WalkFail::Reserved);
    }

    #[test]
    fn the_execute_disable_bit_is_reserved_without_efer_nxe() {
        let mut t = tables(Mode::Ia32e, 0x1000);
        t.nxe = false;
        let err = walk_over(t, 0, &[(0x1000, 0x2000 | pte::PRESENT | pte::NX)])
            .expect_err("bit 63 is reserved with NXE clear");
        assert_eq!(err, WalkFail::Reserved);
    }

    #[test]
    fn permissions_are_the_conjunction_of_every_level() {
        let t = tables(Mode::Ia32e, 0x1000);
        // A user page under a supervisor-only entry is not user-accessible,
        // and a writable page under a read-only one is not writable.
        let w = walk_over(
            t,
            0,
            &[
                (0x1000, 0x2000 | pte::PRESENT | pte::WRITABLE),
                (0x2000, 0x3000 | pte::PRESENT | pte::WRITABLE | pte::USER),
                (0x3000, 0x4000 | pte::PRESENT | pte::USER),
                (0x4000, 0x5000 | pte::PRESENT | pte::WRITABLE | pte::USER),
            ],
        )
        .expect("present at every level");
        assert!(!w.user);
        assert!(!w.writable);
    }

    #[test]
    fn a_missing_entry_reports_which_kind_of_failure_it_was() {
        let t = tables(Mode::Ia32e, 0x1000);
        assert_eq!(walk_over(t, 0, &[]), Err(WalkFail::NotPresent));
    }
}
