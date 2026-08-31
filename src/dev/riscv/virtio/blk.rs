//! virtio-blk: a block device behind the MMIO transport.
//!
//! # Source
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard, §5.2 ("Block
//! Device"): device ID 2, the configuration layout, the request header, and
//! the one-byte status that ends every request. No driver source was read —
//! see [`queue`](super::queue).
//!
//! # A request
//!
//! One descriptor chain carries all three parts, and the driver decides how to
//! split them across descriptors; nothing here assumes one part per descriptor,
//! because real drivers do not lay them out that way.
//!
//! ```text
//!   readable   le32 type, le32 reserved, le64 sector      (16 bytes)
//!   readable   the data, for a write
//!   writable   the data, for a read
//!   writable   one status byte: 0 ok, 1 I/O error, 2 unsupported
//! ```
//!
//! # The backing store
//!
//! A byte vector, in memory, sized by the machine file or filled from a media
//! image. `ROADMAP.md` §7.1 puts real storage on `fstool::BlockDevice` — qcow2,
//! partition tables, filesystems, crash injection — and that is where this
//! goes; the trait is `std` and feature-gated, so it cannot land in a `no_std`
//! module. What is here is enough to boot a root filesystem out of a ramdisk
//! image and to prove the transport, and it is deliberately not more.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};

use super::queue::{Descriptor, Queue};
use super::{Backend, DEVICE_ID_BLOCK};

/// Bytes per sector, as virtio-blk counts them (§5.2.4). Not the block size of
/// the medium — `capacity` is always in 512-byte units.
pub const SECTOR_SIZE: u64 = 512;

/// The request header: type, a reserved word, and the starting sector.
const HEADER_LEN: u64 = 16;

/// `VIRTIO_BLK_T_IN` — read from the device.
const T_IN: u32 = 0;
/// `VIRTIO_BLK_T_OUT` — write to the device.
const T_OUT: u32 = 1;
/// `VIRTIO_BLK_T_FLUSH`.
const T_FLUSH: u32 = 4;
/// `VIRTIO_BLK_T_GET_ID` — fetch the device's serial number.
const T_GET_ID: u32 = 8;

/// `VIRTIO_BLK_S_OK`.
const S_OK: u8 = 0;
/// `VIRTIO_BLK_S_IOERR`.
const S_IOERR: u8 = 1;
/// `VIRTIO_BLK_S_UNSUPP`.
const S_UNSUPP: u8 = 2;

/// How many bytes the serial number occupies in a `GET_ID` reply (§5.2.6).
const ID_BYTES: usize = 20;

/// The disk image.
struct Disk {
    bytes: Mutex<Vec<u8>>,
    serial: String,
    read_only: bool,
}

impl fmt::Debug for Disk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Disk")
            .field("serial", &self.serial)
            .field("read_only", &self.read_only)
            .field("bytes", &self.bytes.try_lock().map(|b| b.len()))
            .finish()
    }
}

/// A virtio block device.
#[derive(Debug)]
pub struct VirtioBlk {
    disk: Arc<Disk>,
}

impl VirtioBlk {
    /// A device backed by `image`, rounded up to a whole number of sectors.
    #[must_use]
    pub fn new(image: Vec<u8>, serial: String, read_only: bool) -> VirtioBlk {
        let mut bytes = image;
        // A capacity is in whole sectors, so a short tail would be a sector the
        // guest can address and only partly read.
        let padded = bytes.len().next_multiple_of(SECTOR_SIZE as usize);
        bytes.resize(padded, 0);
        VirtioBlk {
            disk: Arc::new(Disk {
                bytes: Mutex::with_rank(LockRank::DEVICE, bytes),
                serial,
                read_only,
            }),
        }
    }

    /// How many 512-byte sectors the guest sees.
    #[must_use]
    pub fn capacity(&self) -> u64 {
        self.disk.bytes.lock().len() as u64 / SECTOR_SIZE
    }

    /// A copy of the medium, for a test that wants to check what was written.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        self.disk.bytes.lock().clone()
    }

    /// Whether writes are refused.
    #[must_use]
    pub fn is_read_only(&self) -> bool {
        self.disk.read_only
    }

    /// Serve one request chain, returning the status byte and how many payload
    /// bytes were written into the chain.
    fn serve(&self, q: &Queue<'_>, chain: &[Descriptor]) -> (u8, u64) {
        let writable = Queue::writable_len(chain);
        if writable == 0 || Queue::readable_len(chain) < HEADER_LEN {
            // Not a request at all. There is nowhere to put a status byte, so
            // the chain is simply completed with nothing written.
            return (S_OK, 0);
        }
        let mut header = [0u8; HEADER_LEN as usize];
        if q.read_chain(chain, 0, &mut header).unwrap_or(0) < header.len() {
            return (S_IOERR, 0);
        }
        let kind = u32::from_le_bytes([header[0], header[1], header[2], header[3]]);
        let sector = u64::from_le_bytes([
            header[8], header[9], header[10], header[11], header[12], header[13], header[14],
            header[15],
        ]);
        // The last writable byte is the status; everything before it is data.
        let payload = writable - 1;

        match kind {
            T_IN => {
                let mut buf = alloc::vec![0u8; payload as usize];
                {
                    let disk = self.disk.bytes.lock();
                    let Some(at) = sector.checked_mul(SECTOR_SIZE) else {
                        return (S_IOERR, 0);
                    };
                    let Ok(at) = usize::try_from(at) else {
                        return (S_IOERR, 0);
                    };
                    let Some(slice) = disk.get(at..at + buf.len()) else {
                        return (S_IOERR, 0);
                    };
                    buf.copy_from_slice(slice);
                }
                match q.write_chain(chain, 0, &buf) {
                    Ok(n) => (S_OK, n as u64),
                    Err(_) => (S_IOERR, 0),
                }
            }
            T_OUT => {
                if self.disk.read_only {
                    return (S_IOERR, 0);
                }
                let want = Queue::readable_len(chain) - HEADER_LEN;
                let mut buf = alloc::vec![0u8; want as usize];
                if q.read_chain(chain, HEADER_LEN, &mut buf).unwrap_or(0) < buf.len() {
                    return (S_IOERR, 0);
                }
                let mut disk = self.disk.bytes.lock();
                let Some(at) = sector.checked_mul(SECTOR_SIZE) else {
                    return (S_IOERR, 0);
                };
                let Ok(at) = usize::try_from(at) else {
                    return (S_IOERR, 0);
                };
                let Some(slice) = disk.get_mut(at..at + buf.len()) else {
                    return (S_IOERR, 0);
                };
                slice.copy_from_slice(&buf);
                (S_OK, 0)
            }
            // Nothing is cached, so a flush has nothing to do and succeeds.
            T_FLUSH => (S_OK, 0),
            T_GET_ID => {
                let mut id = [0u8; ID_BYTES];
                let serial = self.disk.serial.as_bytes();
                let take = serial.len().min(ID_BYTES);
                id[..take].copy_from_slice(&serial[..take]);
                let take = (payload as usize).min(ID_BYTES);
                match q.write_chain(chain, 0, &id[..take]) {
                    Ok(n) => (S_OK, n as u64),
                    Err(_) => (S_IOERR, 0),
                }
            }
            _ => (S_UNSUPP, 0),
        }
    }
}

impl Backend for VirtioBlk {
    fn device_id(&self) -> u32 {
        DEVICE_ID_BLOCK
    }

    fn queue_count(&self) -> usize {
        // One request queue. `VIRTIO_BLK_F_MQ` is not offered, so §5.2.2 says
        // there is exactly one.
        1
    }

    fn features(&self) -> u64 {
        // Nothing beyond `VIRTIO_F_VERSION_1`, which the transport adds. Every
        // §5.2.3 feature is optional, and a device that offers none is a device
        // whose driver takes the simple path — which is the path that is easy
        // to be sure about.
        0
    }

    fn config_read(&self, offset: u64, dst: &mut [u8]) {
        // §5.2.4: the whole configuration is optional except `capacity`, a
        // little-endian 64-bit sector count at offset 0. Everything past it
        // belongs to a feature this device does not offer, so it reads zero.
        let capacity = self.capacity().to_le_bytes();
        for (i, byte) in dst.iter_mut().enumerate() {
            let at = offset + i as u64;
            *byte = usize::try_from(at)
                .ok()
                .and_then(|at| capacity.get(at))
                .copied()
                .unwrap_or(0);
        }
    }

    fn handle(&self, _queue: usize, q: &Queue<'_>, chain: &[Descriptor]) -> u32 {
        let (status, written) = self.serve(q, chain);
        let writable = Queue::writable_len(chain);
        if writable == 0 {
            return 0;
        }
        // The status byte is the last writable byte of the chain, wherever the
        // driver put it.
        let _ = q.write_chain(chain, writable - 1, &[status]);
        (written + 1) as u32
    }

    fn reset(&self) {
        // The medium survives a device reset — it is the disk, not a register.
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The medium is architectural state: a guest that wrote to it and then
        // restored a snapshot must see what the snapshot saw, or its
        // filesystem is corrupt (`ROADMAP.md` §4.5, "storage is snapshotted
        // with the machine or not at all").
        w.write_bytes(&self.disk.bytes.lock())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        let mut disk = self.disk.bytes.lock();
        if bytes.len() != disk.len() {
            return Err(Error::State(alloc::format!(
                "snapshot has a {}-byte disk, this device has {}",
                bytes.len(),
                disk.len()
            )));
        }
        disk.clear();
        disk.extend_from_slice(bytes);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId};
    use crate::core::value::Width;
    use crate::dev::riscv::virtio::queue::{DESC_F_NEXT, DESC_F_WRITE, Layout};

    const DESC: u64 = 0x1000;
    const AVAIL: u64 = 0x2000;
    const USED: u64 = 0x3000;
    const HDR: u64 = 0x5000;
    const DATA: u64 = 0x6000;
    const STATUS: u64 = 0x7000;

    struct Guest {
        space: AddressSpace,
        layout: Layout,
    }

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
                    size: 8,
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

        fn peek(&self, at: u64, width: Width) -> u64 {
            self.space.read(at, width, MemAttrs::DEBUG).unwrap()
        }

        fn desc(&self, index: u64, addr: u64, len: u32, flags: u16, next: u16) {
            let at = DESC + index * 16;
            self.poke(at, Width::U64, addr);
            self.poke(at + 8, Width::U32, u64::from(len));
            self.poke(at + 12, Width::U16, u64::from(flags));
            self.poke(at + 14, Width::U16, u64::from(next));
        }

        /// A three-descriptor request: header, data, status.
        fn request(&self, kind: u32, sector: u64, data_len: u32, data_writable: bool) {
            self.poke(HDR, Width::U32, u64::from(kind));
            self.poke(HDR + 4, Width::U32, 0);
            self.poke(HDR + 8, Width::U64, sector);
            self.desc(0, HDR, HEADER_LEN as u32, DESC_F_NEXT, 1);
            let data_flags = DESC_F_NEXT | if data_writable { DESC_F_WRITE } else { 0 };
            self.desc(1, DATA, data_len, data_flags, 2);
            self.desc(2, STATUS, 1, DESC_F_WRITE, 0);
        }

        fn chain(&self) -> Vec<Descriptor> {
            self.queue().chain(0).unwrap()
        }
    }

    fn disk(sectors: usize) -> VirtioBlk {
        let mut image = alloc::vec![0u8; sectors * SECTOR_SIZE as usize];
        for (i, byte) in image.iter_mut().enumerate() {
            *byte = (i % 251) as u8;
        }
        VirtioBlk::new(image, String::from("rsemu-test"), false)
    }

    #[test]
    fn the_capacity_is_reported_in_sectors() {
        let d = disk(4);
        assert_eq!(d.capacity(), 4);
        let mut config = [0u8; 8];
        d.config_read(0, &mut config);
        assert_eq!(u64::from_le_bytes(config), 4);
        // A short image is rounded up rather than leaving a partial sector.
        assert_eq!(
            VirtioBlk::new(alloc::vec![0; 1], String::new(), false).capacity(),
            1
        );
    }

    #[test]
    fn a_read_returns_the_sector_and_a_good_status() {
        let g = Guest::new();
        let d = disk(4);
        g.request(T_IN, 1, SECTOR_SIZE as u32, true);
        let chain = g.chain();
        let written = d.handle(0, &g.queue(), &chain);
        assert_eq!(written, SECTOR_SIZE as u32 + 1, "the data plus the status");
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);
        // Sector 1 starts at byte 512 of the image, which the fixture filled
        // with `i % 251`.
        assert_eq!(g.peek(DATA, Width::U8) as u8, (512 % 251) as u8);
    }

    #[test]
    fn a_write_lands_on_the_medium() {
        let g = Guest::new();
        let d = disk(4);
        for i in 0..SECTOR_SIZE {
            g.poke(DATA + i, Width::U8, 0xa5);
        }
        g.request(T_OUT, 2, SECTOR_SIZE as u32, false);
        let chain = g.chain();
        assert_eq!(d.handle(0, &g.queue(), &chain), 1, "just the status byte");
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);
        assert_eq!(d.contents()[1024], 0xa5);
        assert_eq!(d.contents()[1024 + 511], 0xa5);
        assert_eq!(
            d.contents()[1024 + 512],
            (1536 % 251) as u8,
            "and no further"
        );
    }

    #[test]
    fn a_read_past_the_end_reports_an_io_error_rather_than_reading_something() {
        let g = Guest::new();
        let d = disk(2);
        g.request(T_IN, 99, SECTOR_SIZE as u32, true);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR);
    }

    #[test]
    fn a_write_to_a_read_only_device_is_refused() {
        let g = Guest::new();
        let d = VirtioBlk::new(alloc::vec![0u8; 1024], String::new(), true);
        g.request(T_OUT, 0, SECTOR_SIZE as u32, false);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR);
        assert!(d.is_read_only());
    }

    #[test]
    fn an_unknown_request_type_is_reported_as_unsupported() {
        let g = Guest::new();
        let d = disk(2);
        g.request(0x1234, 0, 8, true);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_UNSUPP);
    }

    #[test]
    fn get_id_returns_the_serial_padded_to_twenty_bytes() {
        let g = Guest::new();
        let d = disk(2);
        g.request(T_GET_ID, 0, ID_BYTES as u32, true);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);
        let mut id = [0u8; ID_BYTES];
        for (i, byte) in id.iter_mut().enumerate() {
            *byte = g.peek(DATA + i as u64, Width::U8) as u8;
        }
        assert_eq!(&id[..10], b"rsemu-test");
        assert_eq!(id[10], 0, "and the rest is padding");
    }

    #[test]
    fn a_flush_succeeds_because_nothing_is_cached() {
        let g = Guest::new();
        let d = disk(2);
        g.request(T_FLUSH, 0, 1, true);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);
    }

    #[test]
    fn a_chain_with_no_status_byte_is_completed_rather_than_faulting() {
        // Nothing here trusts the guest: a driver that offers a chain with no
        // writable descriptor at all must not take the emulator with it.
        let g = Guest::new();
        let d = disk(2);
        g.desc(0, HDR, HEADER_LEN as u32, 0, 0);
        let chain = g.chain();
        assert_eq!(d.handle(0, &g.queue(), &chain), 0);
    }

    #[test]
    fn a_snapshot_carries_the_medium() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

        let g = Guest::new();
        let saved = disk(2);
        for i in 0..SECTOR_SIZE {
            g.poke(DATA + i, Width::U8, 0x5a);
        }
        g.request(T_OUT, 0, SECTOR_SIZE as u32, false);
        let chain = g.chain();
        saved.handle(0, &g.queue(), &chain);

        let mut shape = MachineShape::new();
        shape.add_device("disk", "virtio.blk").unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("disk", "virtio.blk", 1).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = disk(2);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("disk", "virtio.blk", 1, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();
        assert_eq!(restored.contents()[0], 0x5a);

        // A disk of a different size is refused rather than truncated.
        let other = disk(4);
        let chunk = reader
            .load("disk", "virtio.blk", 1, &Migrations::new())
            .unwrap();
        assert!(other.load(&mut chunk.reader()).is_err());
    }
}
