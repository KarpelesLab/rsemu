//! virtio-blk: a block device behind the MMIO transport.
//!
//! # Source
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard, §5.2 ("Block
//! Device"): device ID 2 (§5.2.1), the one request virtqueue (§5.2.2), the
//! feature bits (§5.2.3), the `capacity` configuration field (§5.2.4), and the
//! request header, payload and one-byte status that make up a request
//! (§5.2.6). No driver source was read — see [`queue`](super::queue).
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
//! # The backing store is a `Medium`
//!
//! The same seam an `ata.disk`'s platter and an NVMe
//! namespace use — [`dev::medium::Medium`](crate::dev::medium::Medium) — and for the
//! same three reasons. A [`RamStore`](crate::core::space::RamStore) is what a
//! machine file's media slot gives, which is `no_std` and is what a wasm build
//! runs on; a [`dev::blk::Image`](crate::dev::blk) is a host file through
//! `fstool::BlockDevice`, so `rsemu run riscv-virt --drive disk=root.qcow2`
//! boots off a sparse image that stays on disk rather than a copy of it in
//! host memory; and the snapshot policy arrives with the medium instead of
//! being decided here (`ROADMAP.md` §7.1).
//!
//! This module names neither `std` nor `fstool`: it sees the trait, and the
//! trait is `no_std`. That is the whole reason the seam lives on the side that
//! cannot see `std`.
//!
//! # Bounds
//!
//! Every length in a request is guest-supplied, and two of them used to reach
//! an allocator directly. They do not now: a transfer is range-checked against
//! the medium in `u64` **before** anything is allocated, and then moved in
//! 64 KiB pieces, so a chain claiming a terabyte costs one 64 KiB
//! buffer and one `VIRTIO_BLK_S_IOERR`. `dev/nvme` bounds its PRP walks for
//! the same reason and argues it in the same words.
//!
//! # Time
//!
//! A medium access takes **zero guest time**, whether it is a memcpy or a
//! `pread` five levels into a qcow2's backing chain. If the host's actual
//! latency reached the guest's timeline, two runs of one machine would diverge
//! on how warm the page cache was (`docs/buses/storage.md`).

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::dev::medium::{Medium, Snapshot};

use super::queue::{Descriptor, Queue};
use super::{Backend, DEVICE_ID_BLOCK};

/// Bytes per sector, as virtio-blk counts them (§5.2.4). Not the block size of
/// the medium — `capacity` is always in 512-byte units.
pub const SECTOR_SIZE: u64 = 512;

/// The request header: type, a reserved word, and the starting sector.
const HEADER_LEN: u64 = 16;

/// How much of a transfer is staged at a time.
///
/// A request's payload length is whatever the guest's descriptors add up to,
/// which is a queue's worth of `u32` lengths. Staging the whole of it would
/// let a driver ask for an allocation of any size it can describe; a chunk
/// bounds that to one buffer whatever the request claims, and costs nothing on
/// the requests a real driver issues, which are a page or two.
const CHUNK: u64 = 64 * 1024;

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

/// `VIRTIO_BLK_F_RO` (§5.2.3, bit 5): the device is read-only.
///
/// Offered only when it is true. Telling a guest it may write and then failing
/// every write is worse than an honest read-only disk — the same argument
/// `ata.disk` makes about the `IDENTIFY` it reports.
const F_RO: u64 = 1 << 5;

/// `VIRTIO_BLK_F_FLUSH` (§5.2.3, bit 9): the device honours a cache flush.
///
/// Always offered, because it is always true and it stopped being *trivially*
/// true when the medium became a file. A write reaches the medium inside the
/// call that carried it, but "reached the medium" and "is durable" are
/// different claims once there is a host page cache in between, and
/// `VIRTIO_BLK_T_FLUSH` is how a guest filesystem asks for the second one
/// (§5.2.6).
const F_FLUSH: u64 = 1 << 9;

/// How many bytes the serial number occupies in a `GET_ID` reply (§5.2.6).
const ID_BYTES: usize = 20;

/// A virtio block device.
#[derive(Debug)]
pub struct VirtioBlk {
    media: Arc<dyn Medium>,
    serial: String,
    read_only: bool,
    /// The medium's capacity in bytes, read once: [`Medium::capacity`] is
    /// fixed for the life of the device, and a `config_read` should not take
    /// an image's lock to learn something that cannot change.
    bytes: u64,
}

impl VirtioBlk {
    /// A device over `media`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the medium is empty or
    /// does not hold a whole number of 512-byte sectors — `capacity` is
    /// counted in them (§5.2.4), so a short tail would be a sector the guest
    /// can address and only partly read.
    pub fn new(media: Arc<dyn Medium>, serial: String, read_only: bool) -> Result<VirtioBlk> {
        let bytes = media.capacity();
        if bytes == 0 || !bytes.is_multiple_of(SECTOR_SIZE) {
            return Err(Error::Config {
                at: String::from(super::BLK_CLASS_NAME),
                message: alloc::format!(
                    "a virtio disk holds a whole number of {SECTOR_SIZE}-byte sectors, and \
                     {bytes} byte(s) is not a whole number of them"
                ),
            });
        }
        Ok(VirtioBlk {
            read_only: read_only || media.is_read_only(),
            media,
            serial,
            bytes,
        })
    }

    /// How many 512-byte sectors the guest sees.
    #[must_use]
    pub fn capacity(&self) -> u64 {
        self.bytes / SECTOR_SIZE
    }

    /// The medium behind the device, for a host that wants to check what a
    /// guest wrote without going back through the virtqueue.
    #[must_use]
    pub fn medium(&self) -> &Arc<dyn Medium> {
        &self.media
    }

    /// Make every write so far durable, as a `T_FLUSH` request does.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the host refused.
    pub fn flush(&self) -> Result<()> {
        self.media
            .flush()
            .map_err(|e| Error::State(alloc::format!("virtio disk: {e}")))
    }

    /// Whether writes are refused.
    #[must_use]
    pub fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// Every byte on the medium, for a [`Snapshot::Capture`] chunk.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the medium could not be
    /// read.
    pub fn contents(&self) -> Result<Vec<u8>> {
        let mut out = alloc::vec![0u8; self.bytes as usize];
        self.media
            .read_at(0, &mut out)
            .map_err(|e| Error::State(alloc::format!("virtio disk: {e}")))?;
        Ok(out)
    }

    /// The byte offset `len` bytes at `sector` start at, or `None` if the range
    /// runs off the end of the medium.
    ///
    /// In `u64` throughout and checked before any of it becomes a length: a
    /// guest picks both numbers, and `sector * 512 + len` is exactly where a
    /// 64-bit guest on a 32-bit host bites (`docs/buses/storage.md`).
    fn range(&self, sector: u64, len: u64) -> Option<u64> {
        let at = sector.checked_mul(SECTOR_SIZE)?;
        let end = at.checked_add(len)?;
        (end <= self.bytes).then_some(at)
    }

    /// `VIRTIO_BLK_T_IN`: medium to chain.
    ///
    /// A chunk is read out of the medium and *then* placed in guest memory,
    /// never both at once: the medium's own lock is not held across a bus
    /// master's access to the space it masters (`ROADMAP.md` §4.7).
    fn read_in(&self, q: &Queue<'_>, chain: &[Descriptor], sector: u64, len: u64) -> (u8, u64) {
        let Some(at) = self.range(sector, len) else {
            return (S_IOERR, 0);
        };
        let mut buf = alloc::vec![0u8; CHUNK.min(len) as usize];
        let mut done = 0u64;
        while done < len {
            let take = CHUNK.min(len - done) as usize;
            let part = &mut buf[..take];
            if self.media.read_at(at + done, part).is_err() {
                return (S_IOERR, done);
            }
            match q.write_chain(chain, done, part) {
                Ok(n) => {
                    done += n as u64;
                    if n < take {
                        // Guest memory refused part of a buffer the chain's own
                        // descriptors claimed was there.
                        return (S_IOERR, done);
                    }
                }
                Err(_) => return (S_IOERR, done),
            }
        }
        (S_OK, done)
    }

    /// `VIRTIO_BLK_T_OUT`: chain to medium.
    fn write_out(&self, q: &Queue<'_>, chain: &[Descriptor], sector: u64, len: u64) -> (u8, u64) {
        if self.read_only {
            return (S_IOERR, 0);
        }
        let Some(at) = self.range(sector, len) else {
            return (S_IOERR, 0);
        };
        let mut buf = alloc::vec![0u8; CHUNK.min(len) as usize];
        let mut done = 0u64;
        while done < len {
            let take = CHUNK.min(len - done) as usize;
            let part = &mut buf[..take];
            if q.read_chain(chain, HEADER_LEN + done, part).unwrap_or(0) < take {
                return (S_IOERR, 0);
            }
            if self.media.write_at(at + done, part).is_err() {
                return (S_IOERR, 0);
            }
            done += take as u64;
        }
        (S_OK, 0)
    }

    /// Serve one request chain, returning the status byte and how many payload
    /// bytes were written into the chain.
    fn serve(&self, q: &Queue<'_>, chain: &[Descriptor]) -> (u8, u64) {
        let writable = Queue::writable_len(chain);
        let readable = Queue::readable_len(chain);
        if writable == 0 || readable < HEADER_LEN {
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
            T_IN => self.read_in(q, chain, sector, payload),
            T_OUT => self.write_out(q, chain, sector, readable - HEADER_LEN),
            // §5.2.6: what the guest is asking for is durability, and with a
            // file behind the medium that is a real `fsync` rather than a
            // no-op. A `RamStore` took the write in the call that carried it
            // and answers immediately.
            T_FLUSH => match self.media.flush() {
                Ok(()) => (S_OK, 0),
                Err(_) => (S_IOERR, 0),
            },
            T_GET_ID => {
                let mut id = [0u8; ID_BYTES];
                let serial = self.serial.as_bytes();
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
        // Two of §5.2.3's bits, and only the two this device can keep its word
        // about. The rest of that section — segment limits, geometry,
        // topology, discard, multiqueue — describes promises nothing here
        // makes, and offering a bit a driver then relies on is worse than
        // offering none.
        F_FLUSH | if self.read_only { F_RO } else { 0 }
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

    fn flush(&self) -> Result<()> {
        VirtioBlk::flush(self)
    }

    fn reset(&self) {
        // The medium survives a device reset — it is the disk, not a register.
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // On the terms the medium itself sets, which is the decision
        // `ata.disk` and `nvme.controller` both take and for the same reasons
        // ([`Snapshot`]). A `RamStore` captures, so the encoding of a
        // media-slot disk is byte for byte what it was before this device had
        // a seam at all, and its state hash is unchanged.
        match self.media.snapshot() {
            Snapshot::Capture => w.write_bytes(&self.contents()?),
            Snapshot::Reference => {
                // Flush *first*: the reference is only worth anything if the
                // file on disk holds what the guest had written by the moment
                // the snapshot was taken.
                self.media
                    .flush()
                    .map_err(|e| Error::State(alloc::format!("virtio disk: {e}")))?;
                w.write_bytes(self.media.describe().as_bytes())
            }
            Snapshot::Refuse => Err(Error::State(alloc::format!(
                "this virtio disk's medium ({}) refuses to be snapshotted",
                self.media.describe()
            ))),
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        match self.media.snapshot() {
            Snapshot::Capture => {
                if bytes.len() as u64 != self.bytes {
                    return Err(Error::State(alloc::format!(
                        "snapshot has a {}-byte disk, this device has {}",
                        bytes.len(),
                        self.bytes
                    )));
                }
                self.media
                    .write_at(0, bytes)
                    .map_err(|e| Error::State(alloc::format!("virtio disk: {e}")))
            }
            Snapshot::Reference => {
                // The bytes are still in the image file; what the chunk holds
                // is *which* image, and the check is that it is still that one.
                // A snapshot taken of a capturing disk lands here as a
                // mismatched identity rather than as a silent misread.
                let want = self.media.describe();
                if bytes != want.as_bytes() {
                    return Err(Error::State(alloc::format!(
                        "the snapshot references a different medium: it names `{}` and this \
                         disk holds `{want}`",
                        String::from_utf8_lossy(&bytes[..bytes.len().min(120)])
                    )));
                }
                Ok(())
            }
            Snapshot::Refuse => Err(Error::State(alloc::format!(
                "this virtio disk's medium ({}) refuses to be snapshotted",
                self.media.describe()
            ))),
        }
    }
}

#[cfg(test)]
mod tests;
