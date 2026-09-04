//! Split virtqueues: descriptor tables, available rings and used rings.
//!
//! # Source
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard, §2.7 ("Split
//! Virtqueues") and its subsections. That document is free, complete and
//! normative, and it is the only thing that was read: `ROADMAP.md` §1 singles
//! out Linux's virtio *drivers* as the most common way the provenance rule gets
//! broken, so no driver source of any licence was opened.
//!
//! # The three rings
//!
//! ```text
//!   descriptor table   16 bytes each: { le64 addr, le32 len, le16 flags, le16 next }
//!   available ring     driver writes:  le16 flags, le16 idx, le16 ring[size]
//!   used ring          device writes:  le16 flags, le16 idx, { le32 id, le32 len }[size]
//! ```
//!
//! All three live in *guest* memory, which is why every access here goes
//! through an [`AddressSpace`] rather than through a slice: the device is a bus
//! master (`ROADMAP.md` §4.4) and its view is not necessarily the CPU's.
//!
//! # Nothing here trusts the guest
//!
//! A descriptor may point anywhere, claim any length, and chain to itself. So
//! every chain walk is bounded by the queue size (§2.7.7 makes a longer chain
//! illegal), every index is masked, and a descriptor that does not resolve ends
//! the chain instead of the process. A malicious or merely broken driver gets a
//! stalled queue, never a panic and never an unbounded allocation.

use alloc::vec::Vec;

use crate::core::space::{AddressSpace, MemAttrs, MemResult, RequesterId};
use crate::core::value::Width;

/// `VIRTQ_DESC_F_NEXT` — the buffer continues in `next` (§2.7.5).
pub const DESC_F_NEXT: u16 = 1;
/// `VIRTQ_DESC_F_WRITE` — the device writes this buffer, the driver reads it.
pub const DESC_F_WRITE: u16 = 2;
/// `VIRTQ_DESC_F_INDIRECT` — the buffer is itself a descriptor table (§2.7.7).
pub const DESC_F_INDIRECT: u16 = 4;

/// Bytes per descriptor (§2.7.5).
const DESC_SIZE: u64 = 16;

/// The largest queue this transport advertises.
///
/// The specification allows up to 32768; a smaller number keeps the bounded
/// walks below genuinely bounded and is plenty for a block device and an
/// entropy source.
pub const QUEUE_SIZE_MAX: u32 = 256;

/// Where a queue's three rings live and how big it is.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Layout {
    /// How many descriptors, which is also the ring length. Always a power of
    /// two (§2.7).
    pub size: u32,
    /// Guest-physical address of the descriptor table.
    pub desc: u64,
    /// Guest-physical address of the available ring (the "driver area").
    pub avail: u64,
    /// Guest-physical address of the used ring (the "device area").
    pub used: u64,
    /// Whether the driver has finished configuring it.
    pub ready: bool,
}

impl Layout {
    /// Whether the queue is usable: ready, sized, and with all three rings
    /// placed.
    #[must_use]
    pub fn is_live(&self) -> bool {
        self.ready && self.size > 0 && self.desc != 0 && self.avail != 0 && self.used != 0
    }
}

/// One descriptor, as read out of guest memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Descriptor {
    /// Guest-physical address of the buffer.
    pub addr: u64,
    /// How many bytes.
    pub len: u32,
    /// [`DESC_F_NEXT`] and friends.
    pub flags: u16,
    /// The next descriptor in the chain, when [`DESC_F_NEXT`] is set.
    pub next: u16,
}

impl Descriptor {
    /// Whether the device writes this buffer rather than reading it.
    #[must_use]
    pub fn is_write(&self) -> bool {
        self.flags & DESC_F_WRITE != 0
    }
}

/// A queue the device can walk, bound to the address space its DMA traverses.
#[derive(Debug)]
pub struct Queue<'a> {
    layout: Layout,
    space: &'a AddressSpace,
    attrs: MemAttrs,
}

impl<'a> Queue<'a> {
    /// Bind `layout` to the space the device masters.
    #[must_use]
    pub fn new(layout: Layout, space: &'a AddressSpace, requester: RequesterId) -> Queue<'a> {
        Queue {
            layout,
            space,
            attrs: MemAttrs::DEFAULT.with_requester(requester),
        }
    }

    /// The queue's layout.
    #[must_use]
    pub fn layout(&self) -> Layout {
        self.layout
    }

    /// The driver's `idx`: how many entries it has ever made available
    /// (§2.7.6). Wraps at 16 bits, so it is compared by difference, never by
    /// order.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn avail_idx(&self) -> MemResult<u16> {
        self.read16(self.layout.avail + 2)
    }

    /// The head descriptor of the `n`th available entry.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn avail_head(&self, n: u16) -> MemResult<u16> {
        let slot = u64::from(n % self.ring_len());
        self.read16(self.layout.avail + 4 + slot * 2)
    }

    /// Read one descriptor by index. Out-of-range indices return `None` rather
    /// than reading somewhere arbitrary.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn descriptor(&self, index: u16) -> MemResult<Option<Descriptor>> {
        if u32::from(index) >= self.layout.size {
            return Ok(None);
        }
        let at = self.layout.desc + u64::from(index) * DESC_SIZE;
        Ok(Some(Descriptor {
            addr: self.read64(at)?,
            len: self.read32(at + 8)?,
            flags: self.read16(at + 12)?,
            next: self.read16(at + 14)?,
        }))
    }

    /// Walk a descriptor chain from `head`, following [`DESC_F_INDIRECT`] one
    /// level deep as the specification allows (§2.7.7).
    ///
    /// The walk is bounded by the queue size in each table, so a chain that
    /// loops back on itself ends rather than spinning.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn chain(&self, head: u16) -> MemResult<Vec<Descriptor>> {
        let mut out = Vec::new();
        let mut index = head;
        for _ in 0..self.layout.size {
            let Some(desc) = self.descriptor(index)? else {
                break;
            };
            if desc.flags & DESC_F_INDIRECT != 0 {
                self.walk_indirect(&desc, &mut out)?;
            } else {
                out.push(desc);
            }
            if desc.flags & DESC_F_NEXT == 0 {
                break;
            }
            index = desc.next;
        }
        Ok(out)
    }

    /// Walk an indirect descriptor table. Nested indirection is illegal
    /// (§2.7.7), so a nested flag ends the walk rather than recursing.
    fn walk_indirect(&self, desc: &Descriptor, out: &mut Vec<Descriptor>) -> MemResult<()> {
        let count = u64::from(desc.len) / DESC_SIZE;
        let mut index = 0u64;
        for _ in 0..count {
            let at = desc.addr + index * DESC_SIZE;
            let entry = Descriptor {
                addr: self.read64(at)?,
                len: self.read32(at + 8)?,
                flags: self.read16(at + 12)?,
                next: self.read16(at + 14)?,
            };
            if entry.flags & DESC_F_INDIRECT != 0 {
                return Ok(());
            }
            out.push(entry);
            if entry.flags & DESC_F_NEXT == 0 {
                return Ok(());
            }
            index = u64::from(entry.next);
            if index >= count {
                return Ok(());
            }
        }
        Ok(())
    }

    /// Publish a completed chain: append `(head, written)` to the used ring and
    /// bump its index (§2.7.8).
    ///
    /// The index is written **after** the entry, because the driver reads the
    /// index to decide whether the entry is there. Getting that order wrong
    /// gives a driver that occasionally reads a used entry that has not been
    /// written yet, which is the sort of bug that only appears under load.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn publish(&self, used_idx: u16, head: u16, written: u32) -> MemResult<u16> {
        let slot = u64::from(used_idx % self.ring_len());
        let at = self.layout.used + 4 + slot * 8;
        self.write32(at, u32::from(head))?;
        self.write32(at + 4, written)?;
        let next = used_idx.wrapping_add(1);
        self.write16(self.layout.used + 2, next)?;
        Ok(next)
    }

    /// Read `dst.len()` bytes from a chain's readable buffers, in order.
    ///
    /// Returns how many bytes were filled, which is short when the chain holds
    /// less than was asked for.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn read_chain(&self, chain: &[Descriptor], skip: u64, dst: &mut [u8]) -> MemResult<usize> {
        let mut skip = skip;
        let mut filled = 0usize;
        for desc in chain.iter().filter(|d| !d.is_write()) {
            let len = u64::from(desc.len);
            if skip >= len {
                skip -= len;
                continue;
            }
            let take = ((len - skip) as usize).min(dst.len() - filled);
            if take == 0 {
                break;
            }
            self.space.read_bytes(
                desc.addr + skip,
                &mut dst[filled..filled + take],
                self.attrs,
            )?;
            filled += take;
            skip = 0;
            if filled == dst.len() {
                break;
            }
        }
        Ok(filled)
    }

    /// Write `src` into a chain's writable buffers, in order, starting `skip`
    /// bytes in.
    ///
    /// Returns how many bytes were placed.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses.
    pub fn write_chain(&self, chain: &[Descriptor], skip: u64, src: &[u8]) -> MemResult<usize> {
        let mut skip = skip;
        let mut done = 0usize;
        for desc in chain.iter().filter(|d| d.is_write()) {
            let len = u64::from(desc.len);
            if skip >= len {
                skip -= len;
                continue;
            }
            let take = ((len - skip) as usize).min(src.len() - done);
            if take == 0 {
                break;
            }
            self.space
                .write_bytes(desc.addr + skip, &src[done..done + take], self.attrs)?;
            done += take;
            skip = 0;
            if done == src.len() {
                break;
            }
        }
        Ok(done)
    }

    /// How many bytes of a chain the device may write.
    #[must_use]
    pub fn writable_len(chain: &[Descriptor]) -> u64 {
        chain
            .iter()
            .filter(|d| d.is_write())
            .map(|d| u64::from(d.len))
            .sum()
    }

    /// How many bytes of a chain the device may read.
    #[must_use]
    pub fn readable_len(chain: &[Descriptor]) -> u64 {
        chain
            .iter()
            .filter(|d| !d.is_write())
            .map(|d| u64::from(d.len))
            .sum()
    }

    /// The ring length, never zero so the modulo below is always defined.
    fn ring_len(&self) -> u16 {
        self.layout.size.clamp(1, u32::from(u16::MAX)) as u16
    }

    fn read16(&self, at: u64) -> MemResult<u16> {
        self.space
            .read(at, Width::U16, self.attrs)
            .map(|v| v as u16)
    }

    fn read32(&self, at: u64) -> MemResult<u32> {
        self.space
            .read(at, Width::U32, self.attrs)
            .map(|v| v as u32)
    }

    fn read64(&self, at: u64) -> MemResult<u64> {
        self.space.read(at, Width::U64, self.attrs)
    }

    fn write16(&self, at: u64, value: u16) -> MemResult {
        self.space
            .write(at, Width::U16, u64::from(value), self.attrs)
    }

    fn write32(&self, at: u64, value: u32) -> MemResult {
        self.space
            .write(at, Width::U32, u64::from(value), self.attrs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use alloc::sync::Arc;

    /// A guest with 64 KiB at zero, and a queue laid out in it.
    struct Guest {
        space: AddressSpace,
        layout: Layout,
    }

    /// Where the three rings go in the test guest.
    const DESC: u64 = 0x1000;
    const AVAIL: u64 = 0x2000;
    const USED: u64 = 0x3000;
    const SIZE: u32 = 8;

    impl Guest {
        fn new() -> Guest {
            let space = AddressSpace::new("mem", 64);
            space
                .topology()
                .map(Region::ram("ram", Arc::new(RamStore::new(0x1_0000))), 0)
                .unwrap();
            Guest {
                space,
                layout: Layout {
                    size: SIZE,
                    desc: DESC,
                    avail: AVAIL,
                    used: USED,
                    ready: true,
                },
            }
        }

        fn queue(&self) -> Queue<'_> {
            Queue::new(self.layout, &self.space, RequesterId(1))
        }

        fn poke(&self, at: u64, width: Width, value: u64) {
            self.space
                .write(at, width, value, MemAttrs::DEFAULT)
                .unwrap();
        }

        fn desc(&self, index: u64, addr: u64, len: u32, flags: u16, next: u16) {
            let at = DESC + index * DESC_SIZE;
            self.poke(at, Width::U64, addr);
            self.poke(at + 8, Width::U32, u64::from(len));
            self.poke(at + 12, Width::U16, u64::from(flags));
            self.poke(at + 14, Width::U16, u64::from(next));
        }

        fn offer(&self, idx: u16, head: u16) {
            self.poke(
                AVAIL + 4 + u64::from(idx % SIZE as u16) * 2,
                Width::U16,
                u64::from(head),
            );
            self.poke(AVAIL + 2, Width::U16, u64::from(idx + 1));
        }

        fn read(&self, at: u64, width: Width) -> u64 {
            self.space.read(at, width, MemAttrs::DEBUG).unwrap()
        }
    }

    #[test]
    fn a_chain_is_walked_in_order_and_ends_without_the_next_flag() {
        let g = Guest::new();
        g.desc(0, 0x8000, 4, DESC_F_NEXT, 1);
        g.desc(1, 0x9000, 8, DESC_F_NEXT | DESC_F_WRITE, 2);
        g.desc(2, 0xa000, 1, DESC_F_WRITE, 0);
        let q = g.queue();
        let chain = q.chain(0).unwrap();
        assert_eq!(chain.len(), 3);
        assert_eq!(chain[0].addr, 0x8000);
        assert!(!chain[0].is_write());
        assert!(chain[2].is_write());
        assert_eq!(Queue::readable_len(&chain), 4);
        assert_eq!(Queue::writable_len(&chain), 9);
    }

    #[test]
    fn a_chain_that_loops_back_on_itself_ends_rather_than_spinning() {
        // Nothing here trusts the guest. A driver that writes `next = 0` on
        // descriptor 0 must not hang the emulator.
        let g = Guest::new();
        g.desc(0, 0x8000, 4, DESC_F_NEXT, 0);
        let chain = g.queue().chain(0).unwrap();
        assert_eq!(chain.len(), SIZE as usize, "bounded by the queue size");
    }

    #[test]
    fn a_descriptor_index_past_the_table_ends_the_chain() {
        let g = Guest::new();
        g.desc(0, 0x8000, 4, DESC_F_NEXT, 99);
        let chain = g.queue().chain(0).unwrap();
        assert_eq!(chain.len(), 1);
        assert_eq!(g.queue().descriptor(99).unwrap(), None);
    }

    #[test]
    fn an_indirect_table_is_followed_one_level_and_no_further() {
        let g = Guest::new();
        // A descriptor pointing at a table of two entries at 0x4000.
        g.desc(0, 0x4000, 2 * DESC_SIZE as u32, DESC_F_INDIRECT, 0);
        for (i, (addr, flags, next)) in [(0x8000u64, DESC_F_NEXT, 1u16), (0x9000, DESC_F_WRITE, 0)]
            .into_iter()
            .enumerate()
        {
            let at = 0x4000 + i as u64 * DESC_SIZE;
            g.poke(at, Width::U64, addr);
            g.poke(at + 8, Width::U32, 16);
            g.poke(at + 12, Width::U16, u64::from(flags));
            g.poke(at + 14, Width::U16, u64::from(next));
        }
        let chain = g.queue().chain(0).unwrap();
        assert_eq!(chain.len(), 2);
        assert_eq!(chain[1].addr, 0x9000);

        // A nested indirect is illegal and ends the walk.
        g.poke(0x4000 + 12, Width::U16, u64::from(DESC_F_INDIRECT));
        assert!(g.queue().chain(0).unwrap().is_empty());
    }

    #[test]
    fn reading_and_writing_a_chain_crosses_descriptor_boundaries() {
        let g = Guest::new();
        g.desc(0, 0x8000, 2, DESC_F_NEXT, 1);
        g.desc(1, 0x8100, 2, DESC_F_NEXT, 2);
        g.desc(2, 0x9000, 4, DESC_F_WRITE, 0);
        for (i, byte) in [1u64, 2].into_iter().enumerate() {
            g.poke(0x8000 + i as u64, Width::U8, byte);
        }
        for (i, byte) in [3u64, 4].into_iter().enumerate() {
            g.poke(0x8100 + i as u64, Width::U8, byte);
        }

        let q = g.queue();
        let chain = q.chain(0).unwrap();
        let mut buf = [0u8; 4];
        assert_eq!(q.read_chain(&chain, 0, &mut buf).unwrap(), 4);
        assert_eq!(buf, [1, 2, 3, 4]);

        // And `skip` lands inside the second descriptor.
        let mut two = [0u8; 2];
        assert_eq!(q.read_chain(&chain, 1, &mut two).unwrap(), 2);
        assert_eq!(two, [2, 3]);

        assert_eq!(q.write_chain(&chain, 0, &[9, 8, 7, 6]).unwrap(), 4);
        assert_eq!(g.read(0x9000, Width::U32), 0x0607_0809);
    }

    #[test]
    fn a_short_chain_reads_and_writes_short_rather_than_running_off_the_end() {
        let g = Guest::new();
        g.desc(0, 0x8000, 2, 0, 0);
        let q = g.queue();
        let chain = q.chain(0).unwrap();
        let mut buf = [0xffu8; 8];
        assert_eq!(q.read_chain(&chain, 0, &mut buf).unwrap(), 2);
        assert_eq!(
            q.write_chain(&chain, 0, &[1, 2, 3]).unwrap(),
            0,
            "no writable buffer"
        );
    }

    #[test]
    fn the_used_index_is_written_after_the_entry() {
        let g = Guest::new();
        let q = g.queue();
        assert_eq!(q.publish(0, 3, 512).unwrap(), 1);
        assert_eq!(g.read(USED + 4, Width::U32), 3, "the head");
        assert_eq!(g.read(USED + 8, Width::U32), 512, "how much was written");
        assert_eq!(g.read(USED + 2, Width::U16), 1, "and then the index");
    }

    #[test]
    fn the_available_ring_wraps_at_the_queue_size_not_at_the_index() {
        let g = Guest::new();
        // Index 9 in a queue of 8 is slot 1.
        g.offer(9, 5);
        let q = g.queue();
        assert_eq!(q.avail_idx().unwrap(), 10);
        assert_eq!(q.avail_head(9).unwrap(), 5);
        assert_eq!(q.avail_head(1).unwrap(), 5, "the same slot");
    }

    #[test]
    fn a_queue_is_live_only_once_every_ring_is_placed() {
        let mut layout = Layout::default();
        assert!(!layout.is_live());
        layout.size = 8;
        layout.desc = 0x1000;
        layout.avail = 0x2000;
        layout.used = 0x3000;
        assert!(!layout.is_live(), "not until the driver says ready");
        layout.ready = true;
        assert!(layout.is_live());
    }
}
