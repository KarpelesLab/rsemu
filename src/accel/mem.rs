//! Guest RAM that a hypervisor can be given a pointer to, and that the
//! interpreter still reaches by byte offset.
//!
//! # The tension, stated
//!
//! `CLAUDE.md` is explicit: *"Guest RAM is addressed by byte offset, never by
//! handing out `&mut [u8]`, so it can live in a `SharedArrayBuffer`. Do not
//! 'simplify' that API."* [`RamStore`](crate::core::space::RamStore) honours
//! that with a `Vec<AtomicU8>` and **no `unsafe` at all**.
//!
//! KVM wants the opposite thing: `KVM_SET_USER_MEMORY_REGION` takes a
//! `userspace_addr`, the kernel installs it in the guest's second-dimension
//! page tables, and hardware then reads and writes those bytes with no
//! software in the path at all. It also requires that address to be **page
//! aligned**, which a `Vec<AtomicU8>` from the global allocator is not: its
//! layout alignment is 1.
//!
//! So this type exists, and it resolves the tension rather than choosing a
//! side:
//!
//! * The backing store is a page-aligned anonymous `mmap` ([`sys::Mapping`]),
//!   so there is a host address to hand the kernel.
//! * Everything in Rust reaches it as `&[AtomicU8]` through
//!   [`sys::Mapping::cells`] — the same element type,
//!   the same relaxed per-byte atomicity and the same byte-offset API
//!   `RamStore` has. No `&mut [u8]` is ever produced, so nothing here would
//!   have to change to put the allocation somewhere else.
//! * It implements [`MemOps`], so it maps into a
//!   [`AddressSpace`](crate::core::space::AddressSpace) as an ordinary region
//!   and every device, DMA master, debugger and snapshot in the crate reaches
//!   the identical bytes the guest is executing out of.
//!
//! The `AtomicU8` element type is not a formality here. While a vCPU is inside
//! `KVM_RUN`, guest hardware is writing this memory concurrently with anything
//! the host does to it, and per-byte atomics are the strongest statement the
//! Rust abstract machine can make about that. A `Vec<u8>` would make every
//! such access a data race by definition.
//!
//! # What this is *not*
//!
//! It is not a replacement for `RamStore` and no machine file reaches it. A
//! board declares `ram` and gets a `RamStore`; running *that* under KVM needs
//! `RamStore` itself to gain a page-aligned backing and a host-pointer
//! accessor, which is the sanctioned "RAM host-pointer fast path" `unsafe`
//! site its own module documents as *"deliberately not taken now"*. Until it
//! is taken, an accelerated machine's RAM comes from here.

use crate::core::error::BusError;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use alloc::sync::Arc;
use core::fmt;
use core::sync::atomic::{AtomicU8, Ordering};

use super::sys::{self, PAGE_SIZE};

/// Page-aligned guest memory, addressed by byte offset.
///
/// Shareable: `Send + Sync` because every access goes through `AtomicU8`.
pub struct HostPages {
    map: sys::Mapping,
}

impl fmt::Debug for HostPages {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HostPages")
            .field("host_addr", &format_args!("{:#x}", self.map.addr()))
            .field("len", &self.map.len())
            .finish()
    }
}

impl HostPages {
    /// Allocate `len` zeroed bytes, page aligned.
    ///
    /// # Errors
    ///
    /// [`super::AccelError::Sys`] if `len` is zero or not a multiple of
    /// [`PAGE_SIZE`], or if the mapping cannot be made.
    pub fn new(len: u64) -> super::AccelResult<HostPages> {
        let map = sys::map_anonymous(len).map_err(|e| super::AccelError::Sys {
            what: "mmap guest RAM",
            errno: e,
        })?;
        Ok(HostPages { map })
    }

    /// Size in bytes.
    #[inline]
    #[must_use]
    pub const fn len(&self) -> u64 {
        self.map.len()
    }

    /// Whether the region is empty. Never true; here for clippy's sake.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    /// The host address the hypervisor is given.
    ///
    /// Page aligned, and stable for the life of this object — which is exactly
    /// the contract `KVM_SET_USER_MEMORY_REGION` needs, since the kernel keeps
    /// the address until the slot is deleted.
    #[inline]
    #[must_use]
    pub const fn host_addr(&self) -> u64 {
        self.map.addr()
    }

    /// The bytes, as a slice of atomics.
    ///
    /// There is **no `unsafe` in this file**: the one `from_raw_parts` the
    /// subsystem needs lives on [`sys::Mapping::cells`], which both this store
    /// and the `kvm_run` page reach it through.
    #[inline]
    fn cells(&self) -> &[AtomicU8] {
        self.map.cells()
    }

    #[inline]
    fn range(&self, offset: u64, len: u64) -> MemResult<usize> {
        let end = offset.checked_add(len).ok_or(BusError::BadAccess)?;
        if end > self.len() {
            return Err(BusError::BadAccess);
        }
        usize::try_from(offset).map_err(|_| BusError::BadAccess)
    }

    /// Copy `dst.len()` bytes from `offset` into `dst`.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] if the range leaves the region.
    #[inline]
    pub fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        let base = self.range(offset, dst.len() as u64)?;
        let cells = self.cells();
        for (i, b) in dst.iter_mut().enumerate() {
            *b = cells[base + i].load(Ordering::Relaxed);
        }
        Ok(())
    }

    /// Copy `src` to `offset`.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] if the range leaves the region.
    #[inline]
    pub fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        let base = self.range(offset, src.len() as u64)?;
        let cells = self.cells();
        for (i, b) in src.iter().enumerate() {
            cells[base + i].store(*b, Ordering::Relaxed);
        }
        Ok(())
    }

    /// Read one byte.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] if `offset` is outside the region.
    #[inline]
    pub fn read_u8(&self, offset: u64) -> MemResult<u8> {
        let at = self.range(offset, 1)?;
        Ok(self.cells()[at].load(Ordering::Relaxed))
    }

    /// Write one byte.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] if `offset` is outside the region.
    #[inline]
    pub fn write_u8(&self, offset: u64, value: u8) -> MemResult {
        let at = self.range(offset, 1)?;
        self.cells()[at].store(value, Ordering::Relaxed);
        Ok(())
    }

    /// Set `len` bytes at `offset` to `value`.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] if the range leaves the region.
    pub fn fill(&self, offset: u64, len: u64, value: u8) -> MemResult {
        let base = self.range(offset, len)?;
        let cells = self.cells();
        let count = usize::try_from(len).map_err(|_| BusError::BadAccess)?;
        for cell in &cells[base..base + count] {
            cell.store(value, Ordering::Relaxed);
        }
        Ok(())
    }

    /// Wrap this store in a region that can be mapped into an address space.
    ///
    /// The region is [`Region::io`] rather than [`Region::ram`] because the
    /// latter takes a [`RamStore`](crate::core::space::RamStore). Guest-visible
    /// behaviour is identical; what is given up is the dispatcher's host-pointer
    /// fast path, which under KVM is cold anyway — the guest's own accesses do
    /// not go through the address space at all.
    #[must_use]
    pub fn region(self: &Arc<Self>) -> RegionRef {
        let len = self.len();
        Arc::new(Region::io(
            "accel.ram",
            len,
            Arc::clone(self) as Arc<dyn MemOps>,
        ))
    }

    /// Round `len` up to a whole number of pages, which is what KVM requires
    /// of a memory region's size.
    #[must_use]
    pub const fn page_align(len: u64) -> u64 {
        len.div_ceil(PAGE_SIZE) * PAGE_SIZE
    }
}

impl MemOps for HostPages {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // RAM has no side effects, so `MemAttrs::debug` needs no special case.
        self.read_at(offset, dst)
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        self.write_at(offset, src)
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_store_is_page_aligned_and_zeroed() {
        let ram = HostPages::new(2 * PAGE_SIZE).expect("mmap");
        assert_eq!(ram.host_addr() % PAGE_SIZE, 0);
        assert_eq!(ram.len(), 2 * PAGE_SIZE);
        assert!(!ram.is_empty());
        let mut buf = [0xffu8; 16];
        ram.read_at(0, &mut buf).expect("read");
        assert_eq!(buf, [0u8; 16]);
    }

    #[test]
    fn bytes_written_come_back() {
        let ram = HostPages::new(PAGE_SIZE).expect("mmap");
        ram.write_at(0x10, b"rsemu").expect("write");
        let mut buf = [0u8; 5];
        ram.read_at(0x10, &mut buf).expect("read");
        assert_eq!(&buf, b"rsemu");
        ram.write_u8(0x20, 0x5a).expect("write");
        assert_eq!(ram.read_u8(0x20).expect("read"), 0x5a);
        ram.fill(0x30, 4, 0xcc).expect("fill");
        let mut buf = [0u8; 5];
        ram.read_at(0x2f, &mut buf).expect("read");
        assert_eq!(buf, [0, 0xcc, 0xcc, 0xcc, 0xcc]);
    }

    #[test]
    fn an_access_off_the_end_is_a_bus_fault_not_a_panic() {
        let ram = HostPages::new(PAGE_SIZE).expect("mmap");
        let mut buf = [0u8; 8];
        assert_eq!(
            ram.read_at(PAGE_SIZE - 4, &mut buf),
            Err(BusError::BadAccess)
        );
        assert_eq!(ram.write_at(PAGE_SIZE, b"x"), Err(BusError::BadAccess));
        assert_eq!(ram.read_u8(PAGE_SIZE), Err(BusError::BadAccess));
        assert_eq!(ram.fill(0, PAGE_SIZE + 1, 0), Err(BusError::BadAccess));
        // An offset that would overflow the addition must fault rather than
        // wrap into a range that looks legal.
        assert_eq!(ram.read_at(u64::MAX, &mut buf), Err(BusError::BadAccess));
    }

    #[test]
    fn the_same_bytes_are_visible_through_an_address_space() {
        use crate::core::space::AddressSpace;

        let ram = Arc::new(HostPages::new(PAGE_SIZE).expect("mmap"));
        let space = AddressSpace::new("mem", 32);
        space.topology().map(ram.region(), 0x1000).expect("map");

        ram.write_u8(0x40, 0xa5).expect("write");
        assert_eq!(
            space
                .read(0x1040, crate::core::Width::U8, MemAttrs::DEFAULT)
                .expect("read"),
            0xa5
        );

        space
            .write(0x1041, crate::core::Width::U8, 0x5a, MemAttrs::DEFAULT)
            .expect("write");
        assert_eq!(ram.read_u8(0x41).expect("read"), 0x5a);
    }

    #[test]
    fn page_align_rounds_up() {
        assert_eq!(HostPages::page_align(0), 0);
        assert_eq!(HostPages::page_align(1), PAGE_SIZE);
        assert_eq!(HostPages::page_align(PAGE_SIZE), PAGE_SIZE);
        assert_eq!(HostPages::page_align(PAGE_SIZE + 1), 2 * PAGE_SIZE);
    }
}
