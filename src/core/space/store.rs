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

use super::attrs::MemResult;
use crate::core::error::BusError;
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};

/// Default dirty-tracking granularity: 4 KiB, the page size everything else
/// assumes.
pub const DEFAULT_PAGE_BITS: u32 = 12;

/// Writable guest memory, addressed by byte offset and shareable across
/// threads.
///
/// Carries its own dirty-page bitmap, because the write path is the only place
/// dirty state can be recorded: host signals are forbidden (wasm has none), so
/// there is no way to trap a write after the fact (`ROADMAP.md` §4.1).
pub struct RamStore {
    cells: Vec<AtomicU8>,
    dirty: Vec<AtomicU64>,
    page_bits: u32,
    len: u64,
}

impl fmt::Debug for RamStore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RamStore")
            .field("len", &self.len)
            .field("page_size", &self.page_size())
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
        let mut cells = Vec::new();
        cells.resize_with(n, || AtomicU8::new(0));
        let mut dirty = Vec::new();
        dirty.resize_with(words, || AtomicU64::new(0));
        RamStore {
            cells,
            dirty,
            page_bits,
            len,
        }
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
        usize::try_from(offset).map_err(|_| BusError::BadAccess)
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
    /// guest load becomes a mask, a compare, an add and a `mov`
    /// (`ROADMAP.md` §9.1's first mechanism, *"inlined into generated code"*).
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
        let base = usize::try_from(offset).ok()?;
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
    bytes: Vec<u8>,
}

impl fmt::Debug for RomStore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RomStore")
            .field("len", &self.bytes.len())
            .finish_non_exhaustive()
    }
}

impl RomStore {
    /// Take ownership of `bytes` as ROM contents.
    #[must_use]
    pub fn new(bytes: Vec<u8>) -> Self {
        RomStore { bytes }
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
        RomStore {
            bytes: alloc::vec![0u8; n],
        }
    }

    /// Size in bytes.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.bytes.len() as u64
    }

    /// Whether the ROM is zero-sized.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// The contents, for hashing and snapshotting.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
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
        let base = usize::try_from(offset).map_err(|_| BusError::BadAccess)?;
        dst.copy_from_slice(&self.bytes[base..base + dst.len()]);
        Ok(())
    }
}
