//! Backing stores for memory regions: guest RAM and ROM.
//!
//! # Why RAM is a `Vec<AtomicU8>` and not a `Vec<u8>`
//!
//! `RamStore` is addressed **by byte offset, never by handing out
//! `&mut [u8]`** (`ROADMAP.md` §4.7). That is not a stylistic choice: the same
//! store is reachable from several CPU threads at once, and on wasm it has to
//! be able to live in a shared linear memory. Handing out a slice would either
//! require a lock on the hottest path in the emulator or an aliasing `unsafe`.
//!
//! A `Vec<AtomicU8>` gives byte-offset addressing that is `Sync` **with no
//! `unsafe` at all**. Relaxed byte loads and stores compile to ordinary `mov`
//! on x86-64 and `ldrb`/`strb` on AArch64 — the atomicity is in the type
//! system, not in the instruction stream. What it costs is bulk copies: a page
//! copy is a byte loop rather than a `memcpy`. That is the one place where the
//! sanctioned "RAM host-pointer fast path" `unsafe` (`ROADMAP.md` §0) would
//! buy something, and it can be added later *behind this same API* without
//! touching a single caller. It is deliberately not taken now.
//!
//! Ordering is `Relaxed` throughout. Guest atomicity and guest barriers are
//! the IR lifter's job (§4.7): the store provides per-byte atomicity so that a
//! racing access is never undefined behaviour, and nothing more.
//!
//! # What per-byte atomicity is not
//!
//! "Per-byte, and nothing more" is a stronger claim than it looks, so it is
//! spelled out rather than left to be discovered. A four-byte guest store is
//! **four independent byte stores**, and a four-byte guest load that overlaps
//! one comes back holding a mixture of the old and the new word — a value that
//! was never in memory. Every architecture in the tree forbids that for a
//! naturally aligned access up to its register width: *Intel SDM* volume 3
//! §9.1.1, ARM DDI 0487 B2.2.1 ("single-copy atomicity"), RISC-V Unprivileged
//! ISA §1.4. A guest that publishes a pointer with an ordinary store and reads
//! it with an ordinary load — which is every `WRITE_ONCE`/`READ_ONCE` pair in a
//! Linux kernel — is relying on exactly that guarantee.
//!
//! It is reachable, and `tests/smp_single_copy_atomicity.rs` reaches it: sixty
//! thousand aligned four-byte loads racing sixty thousand aligned four-byte
//! stores tear **117 to 361 times** on two host threads. On **one** host thread
//! it tears zero times however finely the two programs are interleaved, because
//! nothing runs between the bytes of a store — so this is a
//! `ThreadingMode::Parallel` property, and `Parallel` is opt-in
//! (`--threading parallel`; no machine file in the tree asks for it, and
//! `usermode`'s `ThreadSet` runs every guest thread on one host thread by
//! design).
//!
//! ## Who has it and who does not, which is not what this said before
//!
//! This section used to say the gap was "engine-dependent … a store the JIT
//! *inlines* never comes through here at all: `jit::x86` emits one host store
//! of the guest's width, so on an x86-64 host an inlined aligned access **is**
//! single-copy atomic". Every clause of that is true and the conclusion a
//! reader draws from it is wrong twice.
//!
//! **First: tearing is a property of the *store*, not of the access.** A store
//! made in four pieces can be seen in halves by any observer, and no care on
//! the loading side un-tears it — an inlined four-byte host `mov` reading a
//! word another core is part-way through writing byte by byte returns exactly
//! the mixture the byte loop left there. So an inlined *store* cannot be seen
//! in halves by anybody (`jit::x86::compile`'s `store` emits one `store_trunc`
//! of the guest's width and its `probe` refuses anything not naturally aligned,
//! so the host instruction is atomic by *Intel SDM* volume 3 §9.1.1), while an
//! inlined *load* is no defence at all. The guarantee follows whoever is
//! storing.
//!
//! **Second: `jit::x86` is the host backend, not the guest.** Inlining exists
//! only where a core publishes `FastMem::load_plan` and `FastMem::store_plan`,
//! and the two that do are **AArch64 and RISC-V**. Nothing under `cpu::x86`
//! mentions `MemPlan` at all — an **x86 guest inlines no memory access
//! whatever**, so every x86 guest store goes through this byte loop in *both*
//! engines. `tests/smp_single_copy_atomicity.rs` runs two 486s, so what it
//! measures is not "the interpreter's residual"; it is that guest's residual,
//! full stop.
//!
//! So the shape is: an aligned store by an AArch64 or RISC-V core, from inside
//! a compiled block, through a filled fast-store entry, is single-copy atomic.
//! Every other store in the tree is not — including that same core's store one
//! instruction later if the block ended, the entry missed, or the instruction
//! did not lift. Two engines therefore disagree about the same word, and so do
//! two guest architectures. That is the shape a differential test between
//! engines is least likely to catch, because each engine is self-consistent.
//!
//! ## What removing it would cost, measured
//!
//! Three shapes are built and timed by `tests/memory_model_costs.rs` (release,
//! best of five, four million operations, nanoseconds per access):
//!
//! | | store 1 B | 2 B | 4 B | 8 B | load 1 B | 2 B | 4 B | 8 B |
//! | --- | --- | --- | --- | --- | --- | --- | --- | --- |
//! | `Vec<AtomicU8>` — today | 0.74 | 0.89 | 1.25 | 1.99 | 0.67 | 0.90 | 1.30 | 2.12 |
//! | `Vec<AtomicU64>`, sub-word writes by CAS | 3.86 | 4.05 | 4.24 | 1.55 | 0.65 | 0.72 | 1.03 | 1.53 |
//! | `Vec<AtomicU8>` + a wide aligned atomic through a cast pointer | 1.34 | 1.34 | 1.40 | 1.40 | 1.22 | 1.27 | 1.31 | 1.35 |
//!
//! Read the third row **per width**, not as an average. "Roughly cost-neutral"
//! was the wrong summary: it is one instruction whatever the width, so it is
//! dearer than today at one and two bytes and *cheaper* at four and eight —
//! and at eight bytes it is cheaper by a third on a store and by 40% on a load.
//! For a 64-bit guest, whose stores are mostly four and eight bytes wide, the
//! wide shape is a **speed-up** that happens to also be correct.
//!
//! ### And the denominator, which was quoted rather than measured
//!
//! "+3 ns is +12% of the hottest path in the emulator" divides by the wrong
//! thing. `SpaceView::write_span` is not a path a guest executes; it is a piece
//! of one, and the same test file now measures both:
//!
//! | | ns |
//! | --- | --- |
//! | the byte loop, four bytes | 1.3 |
//! | one whole `AddressSpace::write`, four bytes | 24.8 |
//! | one interpreted `mov [bx], eax` | 117 |
//! | one interpreted `mov [bx], al` | 101 |
//! | one interpreted `nop`, for scale | 56 |
//!
//! Against the instruction, the byte loop is ~1% and the CAS shape's +3 ns is
//! **+2 to +3%** — not +12% of anything a guest runs. Both candidate fixes are
//! affordable here, which was the thing in doubt.
//!
//! One case is *not* measured and should be before anyone spends the money: a
//! compiled block whose store takes the **slow** path. An AArch64 or RISC-V
//! block that inlines its stores never reaches this file and pays nothing; an
//! x86 block reaches it for every store, and a compiled instruction is several
//! times cheaper than an interpreted one, so the same three nanoseconds are a
//! larger fraction there. That measurement needs the JIT and a lifted block,
//! and it belongs with whoever implements a fix rather than with the file that
//! prices one.
//!
//! ## Why the safe-looking shape is not the safe one
//!
//! `Vec<AtomicU64>` with sub-word writes by compare-exchange reads as the
//! option with no `unsafe` in it, and inside this file it is. Two things are
//! wrong with that reading.
//!
//! **It charges the most to the access that needed the fix least.** A one-byte
//! store is already single-copy atomic and always was; under word cells it
//! becomes a `lock cmpxchg` costing five times today's, spent entirely on not
//! disturbing seven neighbours the byte representation kept apart for free.
//!
//! **It does not remove the mixed-size assumption, it hides it.** The JIT's
//! inlined store writes one to eight bytes straight into the allocation
//! through [`RamStore::host_ptr`]. Make the cells `AtomicU64` and that becomes
//! a *narrow* machine store landing inside a cell the interpreter is
//! read-modify-writing whole — mixed size, exactly as before, but now stated
//! nowhere and reviewed by nobody, because one of the two accesses is machine
//! code the compiler never sees. `jit::x86::rt`'s own `// SAFETY:` says "those
//! bytes are `AtomicU8` … the emitted `mov` is the instruction a relaxed
//! atomic byte store compiles to", and that sentence stops being true. Whoever
//! takes this route owes `jit/` a re-review in the same change; it is not a
//! `core/`-local edit.
//!
//! (The word CAS is at least not *lost-update* prone against that narrow
//! store: a compare-exchange compares the whole word, so a neighbouring byte
//! written between its load and its swap fails the compare and it retries.
//! That was worth checking and it holds.)
//!
//! ## The `unsafe` question, stated for the design review
//!
//! The wide shape needs one `unsafe` block in this file: take
//! `cells.as_ptr()`, offset it, cast to `AtomicU16`/`U32`/`U64`, and do one
//! relaxed access. `CLAUDE.md` says an eighth sanctioned subsystem is a design
//! review rather than a commit, so here is the case both ways.
//!
//! **For it being the first subsystem rather than an eighth.** `ROADMAP.md` §0
//! sanctions "the RAM host-pointer fast path"; [`RamStore::host_ptr`] is that
//! seam and it is *in this file*. The operation is the one the sanction names,
//! on the bytes the sanction is about, under the three obligations `host_ptr`
//! already writes down — and they are easier to discharge here than in the
//! backend, because the store owns the allocation and can see the bounds. The
//! tree already performs this exact access: `jit::x86` stores an aligned host
//! word of the guest's width to these same bytes on every inlined store.
//! Subsystems are counted by seam and not by file — `accel/sys.rs` and
//! `accel/kvm.rs` are one subsystem across two files and seventeen sites — so
//! this would be one seam across two.
//!
//! Two details make it sound and neither is obvious. **The alignment comes free
//! from the KVM slack**: a `Vec<AtomicU8>` has layout alignment 1, so a
//! naturally aligned *guest offset* would ordinarily say nothing at all about
//! the host address — it is the [`HOST_PAGE`]` - 1` bytes of padding added so a
//! store can be a memory slot that makes guest byte zero 4 KiB aligned, and
//! therefore makes an aligned offset an aligned host address. And **the pointer
//! has to come from `cells.as_ptr()`**, which carries whole-allocation
//! provenance, rather than from `&cells[i]`, which carries one byte's; reading
//! eight bytes through the latter is what would make the cast wrong rather than
//! merely unusual.
//!
//! **Against, and this is what actually decides it.** The seven sanctioned
//! sites are all "this pointer, lifetime or thread invariant is upheld by
//! construction". This one is different in kind: *the language does not define
//! it*. Eight `AtomicU8`s and one `AtomicU64` covering the same bytes are
//! different memory locations that overlap, and the Rust and C++ models state
//! the data-race rule per location; the hardware defines mixed-size access and
//! the models decline to. Today the tree gets away with it because the two
//! widths are in different translation units *by construction* — one of them is
//! machine code LLVM never sees — so no optimiser can act on the overlap.
//! Writing it in Rust puts both widths in front of the same optimiser for the
//! first time. What that optimiser does today is checkable and was checked: one
//! machine access per atomic access, no splitting, no merging, no forwarding
//! across them, and `memory_model_costs`'s mixed-size litmus finds no illegal
//! value in five million rounds. "What it does today" is the whole guarantee on
//! offer, and it is not the same thing as a guarantee.
//!
//! And the sanctioned #1 lives in `jit/`, which is `std`, x86-64-only and
//! feature-gated. This file is `core/`: never feature-gated, compiled for every
//! target in the matrix including both wasm profiles, and on the path of every
//! store of every board. That is a different blast radius for the same seam,
//! and the ceiling `CLAUDE.md` sets is a ceiling on risk rather than on grep
//! hits.
//!
//! ## What is not taken, and what would change the answer
//!
//! Neither shape is taken. `CLAUDE.md`'s test for an eighth subsystem is "what
//! exactly is lost by refusing it?", and the seventh met that test because
//! refusing it meant shipping a data-loss defect. Refusing here leaves a
//! specification violation that **no shipped configuration can observe**:
//! `Deterministic` is the default and structurally cannot tear, no machine file
//! selects `Parallel`, `usermode` runs every guest thread on one host thread,
//! and `Accel` performs the guest's accesses in silicon. Spending the project's
//! eighth exemption on that is not the same trade the seventh was.
//!
//! The trigger that changes it is a configuration that runs guest cores on host
//! threads *by default* — a board that selects `Parallel`, or `usermode`'s
//! `ThreadSet` moving off one thread. At that moment "nothing shipped can
//! observe it" stops being true, and the table above should be read again. When
//! it is, prefer the **wide** shape: it makes every store in the tree perform
//! the access an inlined AArch64 or RISC-V store already performs, which is one
//! fewer way for two engines — and two guest architectures — to disagree rather
//! than one more.
//!
//! What changed here is that the boundary has a reproducer, three re-derivable
//! rows, the denominator they should be divided by, and an argument that does
//! not depend on anyone's recollection of a number.
//!
//! # Why the allocation is host-page aligned
//!
//! A hypervisor is handed guest RAM as a *host address*, and both KVM's
//! `KVM_SET_USER_MEMORY_REGION` and Hypervisor.framework's `hv_vm_map` reject
//! one that is not page aligned. A `Vec<AtomicU8>` from the global allocator
//! has layout alignment **1**, so before this it was structurally impossible
//! for a board's declared `ram` to be a memory slot — `ROADMAP.md` phase 7's
//! gate ("the phase-6 machines boot under KVM") was blocked on an allocator
//! detail.
//!
//! The fix is deliberately the smallest one that could work: allocate
//! [`HOST_PAGE`]` - 1` extra bytes and remember the offset at which the
//! allocation first crosses a page boundary. [`RamStore::host_addr`] reports
//! that address and it is page aligned by construction.
//!
//! What this **does not** do is as important as what it does:
//!
//! * It is not an `mmap`. There is no new syscall, nothing target-specific,
//!   and no `unsafe` — `Vec::as_ptr` is safe, and the value is reported as a
//!   `u64`, never as a pointer or a slice. `core/` stays `no_std`, and a wasm
//!   build gets the identical code path (see below for what it costs there).
//! * It does not change the API by one function. Guest RAM is still addressed
//!   **by byte offset and never handed out as `&mut [u8]`** (`CLAUDE.md`,
//!   "Targets"), so the store can still live in a `SharedArrayBuffer`; that
//!   rule is about the *shape of the accessors*, and the accessors are
//!   untouched.
//! * It does not add a second RAM type or make the store a property of the
//!   machine. Two stores would mean two code paths, boards that are
//!   accelerable and boards that are not, and a `Region::ram` arm per backing
//!   — for a property every allocation can simply have.
//!
//! The costs, stated rather than discovered: **up to [`HOST_PAGE`]` - 1` wasted
//! bytes per store**, which on a wasm build (where the address is a linear
//! memory offset that no hypervisor will ever read) buys nothing at all. A
//! machine with a dozen RAM objects loses under 48 KiB. The alternative —
//! aligning only when some feature is on — would make the *offset arithmetic*
//! differ between builds, which is precisely the kind of thing that makes a
//! state hash target-dependent.

use super::attrs::MemResult;
use crate::core::error::BusError;
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};

/// Default dirty-tracking granularity: 4 KiB, the page size everything else
/// assumes.
pub const DEFAULT_PAGE_BITS: u32 = 12;

/// The alignment every backing store's allocation is given, in bytes.
///
/// 4 KiB, because that is the granularity a hypervisor's second-dimension page
/// tables work in: `KVM_SET_USER_MEMORY_REGION` requires a page-aligned
/// `userspace_addr`, and 4 KiB is the base page on every architecture this
/// crate can accelerate on. It is a constant rather than a query of the host,
/// because it is a property of the *guest* interface being satisfied — a host
/// with 16 KiB pages still accepts a 4 KiB-aligned address, it just needs the
/// value rounded further, and that is the accelerator's business, not the
/// store's.
///
/// Deliberately separate from [`DEFAULT_PAGE_BITS`], which is dirty-tracking
/// granularity and is allowed to differ.
pub const HOST_PAGE: u64 = 4096;

/// The offset, from `addr`, of the first [`HOST_PAGE`]-aligned byte at or
/// after it.
#[inline]
const fn align_gap(addr: usize) -> usize {
    // `HOST_PAGE` is a power of two, so the gap is `(-addr) mod HOST_PAGE`.
    addr.wrapping_neg() % (HOST_PAGE as usize)
}

/// Writable guest memory, addressed by byte offset and shareable across
/// threads.
///
/// Carries its own dirty-page bitmap, because the write path is the only place
/// dirty state can be recorded: host signals are forbidden (wasm has none), so
/// there is no way to trap a write after the fact (`ROADMAP.md` §4.1).
pub struct RamStore {
    /// `len` bytes of guest RAM, preceded by `base` bytes of alignment slack.
    ///
    /// Never resized after construction — a hypervisor holds
    /// [`host_addr`](RamStore::host_addr) until its memory slot is removed, so
    /// a reallocation would hand the guest a window onto whatever the
    /// allocator did next. Every method takes `&self`, which is what makes
    /// that unrepresentable rather than merely intended.
    cells: Vec<AtomicU8>,
    /// Index of guest byte zero: chosen so `cells.as_ptr() + base` is
    /// [`HOST_PAGE`] aligned.
    base: usize,
    dirty: Vec<AtomicU64>,
    page_bits: u32,
    len: u64,
}

impl fmt::Debug for RamStore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RamStore")
            .field("len", &self.len)
            .field("page_size", &self.page_size())
            .field("host_addr", &format_args!("{:#x}", self.host_addr()))
            .finish_non_exhaustive()
    }
}

impl RamStore {
    /// Allocate `len` zeroed bytes with the default dirty-page granularity.
    ///
    /// # Panics
    ///
    /// If `len` does not fit in a host `usize` — a 4 GiB guest cannot be
    /// backed by a 32-bit host allocation, and pretending otherwise only moves
    /// the failure somewhere less obvious.
    #[must_use]
    pub fn new(len: u64) -> Self {
        Self::with_page_bits(len, DEFAULT_PAGE_BITS)
    }

    /// Allocate `len` zeroed bytes, tracking dirtiness at `1 << page_bits`
    /// granularity.
    ///
    /// # Panics
    ///
    /// If `len` does not fit in a host `usize`, or `page_bits` is 0 or >= 64.
    #[must_use]
    pub fn with_page_bits(len: u64, page_bits: u32) -> Self {
        assert!(page_bits > 0 && page_bits < 64, "implausible page size");
        let n = usize::try_from(len).expect("guest RAM larger than the host address space");
        let pages = len.div_ceil(1u64 << page_bits);
        let words = usize::try_from(pages.div_ceil(64)).expect("dirty bitmap too large");
        // The slack that buys a page-aligned `host_addr`. A zero-length store
        // has no bytes to align, and allocating a page for it to say so would
        // be worse than admitting it has no address.
        let pad = if n == 0 { 0 } else { HOST_PAGE as usize - 1 };
        let mut cells = Vec::new();
        cells.resize_with(n + pad, || AtomicU8::new(0));
        let base = if n == 0 {
            0
        } else {
            align_gap(cells.as_ptr() as usize)
        };
        let mut dirty = Vec::new();
        dirty.resize_with(words, || AtomicU64::new(0));
        RamStore {
            cells,
            base,
            dirty,
            page_bits,
            len,
        }
    }

    /// The host address of guest byte zero, as an integer.
    ///
    /// [`HOST_PAGE`] aligned, and stable for the life of the store — which
    /// together are exactly the contract a hypervisor's memory-slot call makes
    /// of a `userspace_addr`. Meaningless for a zero-length store, which has
    /// no bytes to point at.
    ///
    /// **A `u64`, not a pointer and not a slice**, and that is the whole
    /// design: this hands out a *fact about the allocation*, not a way to
    /// alias it. Reading these bytes from Rust still goes through the
    /// byte-offset accessors below; the only consumer that dereferences the
    /// address is a kernel that was handed it, in one of `ROADMAP.md` §0's
    /// sanctioned subsystems. On a target with no hypervisor the value is a
    /// linear-memory offset and nothing asks for it.
    #[inline]
    #[must_use]
    pub fn host_addr(&self) -> u64 {
        (self.cells.as_ptr() as usize + self.base) as u64
    }

    /// Size in bytes.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the store is zero-sized.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Dirty-tracking granularity in bytes.
    #[inline]
    #[must_use]
    pub fn page_size(&self) -> u64 {
        1u64 << self.page_bits
    }

    /// Number of dirty-tracked pages.
    #[inline]
    #[must_use]
    pub fn page_count(&self) -> u64 {
        self.len.div_ceil(self.page_size())
    }

    #[inline]
    fn range(&self, offset: u64, len: u64) -> MemResult<usize> {
        let end = offset.checked_add(len).ok_or(BusError::BadAccess)?;
        if end > self.len {
            return Err(BusError::BadAccess);
        }
        // `self.len` fits in a usize by construction, so `offset` does too.
        let at = usize::try_from(offset).map_err(|_| BusError::BadAccess)?;
        Ok(self.base + at)
    }

    /// Copy `dst.len()` bytes from `offset` into `dst`.
    ///
    /// Never has a side effect, so [`MemAttrs::debug`](super::MemAttrs::debug)
    /// needs no special case here.
    #[inline]
    pub fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        let base = self.range(offset, dst.len() as u64)?;
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self.cells[base + i].load(Ordering::Relaxed);
        }
        Ok(())
    }

    /// Copy `src` into the store at `offset`, marking the pages it touches
    /// dirty.
    #[inline]
    pub fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        let base = self.range(offset, src.len() as u64)?;
        for (i, b) in src.iter().enumerate() {
            self.cells[base + i].store(*b, Ordering::Relaxed);
        }
        self.mark_dirty(offset, src.len() as u64);
        Ok(())
    }

    /// Read one byte.
    #[inline]
    pub fn read_u8(&self, offset: u64) -> MemResult<u8> {
        let base = self.range(offset, 1)?;
        Ok(self.cells[base].load(Ordering::Relaxed))
    }

    /// Write one byte, marking its page dirty.
    #[inline]
    pub fn write_u8(&self, offset: u64, value: u8) -> MemResult {
        let base = self.range(offset, 1)?;
        self.cells[base].store(value, Ordering::Relaxed);
        self.mark_dirty(offset, 1);
        Ok(())
    }

    /// Set `len` bytes at `offset` to `value`, marking them dirty.
    pub fn fill(&self, offset: u64, len: u64, value: u8) -> MemResult {
        let base = self.range(offset, len)?;
        let n = usize::try_from(len).map_err(|_| BusError::BadAccess)?;
        for cell in &self.cells[base..base + n] {
            cell.store(value, Ordering::Relaxed);
        }
        self.mark_dirty(offset, len);
        Ok(())
    }

    /// Mark the pages covering `[offset, offset + len)` dirty.
    ///
    /// Public because a device that writes its own backing store through some
    /// other path (a framebuffer blit, a DMA engine) still owes the dirty bit.
    pub fn mark_dirty(&self, offset: u64, len: u64) {
        if len == 0 {
            return;
        }
        let first = offset >> self.page_bits;
        let last = offset.saturating_add(len - 1) >> self.page_bits;
        for page in first..=last.min(self.page_count().saturating_sub(1)) {
            let (word, bit) = (page / 64, page % 64);
            if let Some(w) = self.dirty.get(word as usize) {
                w.fetch_or(1u64 << bit, Ordering::Relaxed);
            }
        }
    }

    /// Whether `page` has been written since the last clear.
    #[must_use]
    pub fn is_page_dirty(&self, page: u64) -> bool {
        let (word, bit) = (page / 64, page % 64);
        self.dirty
            .get(word as usize)
            .is_some_and(|w| w.load(Ordering::Relaxed) & (1u64 << bit) != 0)
    }

    /// Test and clear one page's dirty bit.
    pub fn take_page_dirty(&self, page: u64) -> bool {
        let (word, bit) = (page / 64, page % 64);
        match self.dirty.get(word as usize) {
            Some(w) => w.fetch_and(!(1u64 << bit), Ordering::Relaxed) & (1u64 << bit) != 0,
            None => false,
        }
    }

    /// Clear every dirty bit.
    pub fn clear_dirty(&self) {
        for w in &self.dirty {
            w.store(0, Ordering::Relaxed);
        }
    }

    /// Call `f` with each dirty page index, in ascending order.
    ///
    /// Ascending order is a determinism requirement, not a convenience: a
    /// framebuffer refresh or a live snapshot that visited pages in a
    /// hash-ordered sequence would produce run-dependent output.
    pub fn for_each_dirty_page(&self, mut f: impl FnMut(u64)) {
        for (i, w) in self.dirty.iter().enumerate() {
            let mut bits = w.load(Ordering::Relaxed);
            while bits != 0 {
                let bit = bits.trailing_zeros() as u64;
                bits &= bits - 1;
                let page = (i as u64) * 64 + bit;
                if page < self.page_count() {
                    f(page);
                }
            }
        }
    }

    /// The host address of the byte at `offset`, if `len` bytes follow it
    /// inside this store.
    ///
    /// **The seam `ROADMAP.md` §0's "RAM host-pointer fast path" is for**, and
    /// the module docs above predicted it: *"it can be added later behind this
    /// same API without touching a single caller"*. Nothing in this crate
    /// dereferences the result from Rust — the one consumer is the x86-64 JIT
    /// backend, which bakes the address into generated machine code so that a
    /// guest access becomes a mask, a compare, an add and a `mov`
    /// (`ROADMAP.md` §9.1's first mechanism, *"inlined into generated code"*).
    ///
    /// That is a `mov` in **both** directions now: `jit::x86::compile`'s
    /// `store` writes through the address and pays the dirty bit afterwards
    /// through `FastMem::note_fast_store`. "Read-only" below is a rule about
    /// this Rust API, not a description of what the generated code does — and
    /// the inlined store being one aligned host instruction of the guest's
    /// width is exactly why an inlined store, unlike an interpreted one, cannot
    /// be seen in halves (see the module documentation's "It is the *store*
    /// side that decides").
    ///
    /// Returning a raw pointer is itself safe; every obligation is on whoever
    /// reads through it, and there are three:
    ///
    /// * The pointer is valid only while this store is alive. The backing
    ///   allocation is made once in [`RamStore::with_page_bits`] and never
    ///   grows, so it does not move — but a caller must keep its
    ///   `Arc<RamStore>` for as long as the address is live.
    /// * Only the `len` bytes this call was asked about are in range.
    /// * The bytes are [`AtomicU8`], written by other threads with relaxed
    ///   stores. A reader must be a plain machine load of the same kind — the
    ///   instruction a relaxed atomic byte load compiles to — and must never
    ///   form a Rust reference to them, or a concurrent guest write is a data
    ///   race rather than the relaxed traffic the type promises.
    ///
    /// **Read-only on purpose.** A write through a host pointer would skip
    /// [`RamStore::mark_dirty`], and the dirty bitmap is the only record a
    /// framebuffer refresh or a live snapshot has (§4.1). A backend that wants
    /// to inline stores owes that bit first.
    #[inline]
    #[must_use]
    pub fn host_ptr(&self, offset: u64, len: u64) -> Option<*const AtomicU8> {
        let end = offset.checked_add(len)?;
        if end > self.len {
            return None;
        }
        // `self.base` is the alignment slack that precedes guest byte zero, so
        // a guest offset is *not* an index into `cells` — every other accessor
        // goes through `self.base + at` and this one must too. Getting it wrong
        // is silent: the compiled fast path reads a valid, wrong address inside
        // the same allocation, and only a differential against the interpreter
        // says so.
        let base = self.base.checked_add(usize::try_from(offset).ok()?)?;
        // Indexing, then a reference-to-pointer cast: both safe, and the index
        // is in range because `self.len` bounds it and fits a `usize` by
        // construction. A zero-length request at the very end of the store
        // would index one past, so it is refused rather than special-cased.
        let cell = self.cells.get(base)?;
        Some(core::ptr::from_ref(cell))
    }

    /// How many pages are dirty.
    #[must_use]
    pub fn dirty_page_count(&self) -> u64 {
        let mut n = 0;
        self.for_each_dirty_page(|_| n += 1);
        n
    }
}

/// Read-only backing store.
///
/// Immutable once built, so it needs no interior mutability and no dirty
/// tracking. What happens to a *write* is a property of the region, not of the
/// store — see [`RomWrite`](super::RomWrite).
pub struct RomStore {
    /// `len` bytes of ROM, preceded by `base` bytes of alignment slack.
    bytes: Vec<u8>,
    base: usize,
    len: usize,
}

impl fmt::Debug for RomStore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RomStore")
            .field("len", &self.len)
            .field("host_addr", &format_args!("{:#x}", self.host_addr()))
            .finish_non_exhaustive()
    }
}

impl RomStore {
    /// Take ownership of `bytes` as ROM contents.
    ///
    /// The image is **copied** into a host-page-aligned allocation, for the
    /// reason [`RamStore`] carries one: firmware is the one region a guest must
    /// be able to *fetch* from, and a hypervisor cannot fetch from a region
    /// that is not a memory slot — KVM's instruction emulator declines a fetch
    /// that would come back as an MMIO exit. A ROM that cannot be a slot is a
    /// board that cannot boot under acceleration, so the copy is the price.
    #[must_use]
    pub fn new(bytes: Vec<u8>) -> Self {
        Self::from_slice(&bytes)
    }

    /// The same, from a borrowed image.
    #[must_use]
    pub fn from_slice(image: &[u8]) -> Self {
        let mut store = RomStore::zeroed(image.len() as u64);
        let base = store.base;
        store.bytes[base..base + image.len()].copy_from_slice(image);
        store
    }

    /// A ROM of `len` zero bytes, for tests and for a socket with no cartridge
    /// in it.
    ///
    /// # Panics
    ///
    /// If `len` does not fit in a host `usize`.
    #[must_use]
    pub fn zeroed(len: u64) -> Self {
        let n = usize::try_from(len).expect("ROM larger than the host address space");
        let pad = if n == 0 { 0 } else { HOST_PAGE as usize - 1 };
        let bytes = alloc::vec![0u8; n + pad];
        let base = if n == 0 {
            0
        } else {
            align_gap(bytes.as_ptr() as usize)
        };
        RomStore {
            bytes,
            base,
            len: n,
        }
    }

    /// Size in bytes.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len as u64
    }

    /// Whether the ROM is zero-sized.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The host address of ROM byte zero, as an integer.
    ///
    /// [`HOST_PAGE`] aligned and stable for the life of the store — the
    /// read-only twin of [`RamStore::host_addr`], and documented there. A
    /// hypervisor installs it as a read-only slot, so a guest *write* still
    /// leaves hardware and arrives at [`RomWrite`](super::RomWrite) the way it
    /// already does under the interpreter.
    #[inline]
    #[must_use]
    pub fn host_addr(&self) -> u64 {
        (self.bytes.as_ptr() as usize + self.base) as u64
    }

    /// The contents, for hashing and snapshotting.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[self.base..self.base + self.len]
    }

    /// Copy `dst.len()` bytes from `offset` into `dst`.
    #[inline]
    pub fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        let end = offset
            .checked_add(dst.len() as u64)
            .ok_or(BusError::BadAccess)?;
        if end > self.len() {
            return Err(BusError::BadAccess);
        }
        let at = usize::try_from(offset).map_err(|_| BusError::BadAccess)? + self.base;
        dst.copy_from_slice(&self.bytes[at..at + dst.len()]);
        Ok(())
    }
}
