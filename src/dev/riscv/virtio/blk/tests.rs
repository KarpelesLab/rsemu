//! virtio-blk against a medium.
//!
//! Everything a guest can see is checked **against the medium** rather than
//! against a buffer the device owns — the standard `tests/pc_at_ide.rs` and
//! `tests/nvme_board.rs` hold — and every transfer asserts the neighbouring
//! block untouched, which is what catches a length computed in blocks and
//! applied in bytes.

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

/// A disk of `sectors` sectors, filled so that every byte says where it is.
fn disk(sectors: usize) -> VirtioBlk {
    let store = RamStore::new(sectors as u64 * SECTOR_SIZE);
    let pattern: Vec<u8> = (0..sectors * SECTOR_SIZE as usize)
        .map(|i| (i % 251) as u8)
        .collect();
    Medium::write_at(&store, 0, &pattern).expect("a fresh store takes its pattern");
    VirtioBlk::new(Arc::new(store), String::from("rsemu-test"), false).expect("a whole disk")
}

/// `len` bytes of the medium at `offset`, read through the seam rather than
/// through anything the device owns.
fn on_medium(d: &VirtioBlk, offset: u64, len: usize) -> Vec<u8> {
    let mut out = alloc::vec![0u8; len];
    d.medium().read_at(offset, &mut out).expect("in range");
    out
}

#[test]
fn the_capacity_is_reported_in_sectors() {
    let d = disk(4);
    assert_eq!(d.capacity(), 4);
    let mut config = [0u8; 8];
    d.config_read(0, &mut config);
    assert_eq!(u64::from_le_bytes(config), 4);
}

#[test]
fn a_medium_that_is_not_whole_sectors_is_refused_rather_than_rounded() {
    // A partial tail would be a sector the guest can address and only partly
    // read, so it is a configuration error and not a silent `resize`.
    let stub = Arc::new(RamStore::new(700)) as Arc<dyn Medium>;
    let e = VirtioBlk::new(stub, String::new(), false).expect_err("700 is not whole sectors");
    assert!(alloc::format!("{e}").contains("512-byte sectors"), "{e}");
    let empty = Arc::new(RamStore::new(0)) as Arc<dyn Medium>;
    assert!(VirtioBlk::new(empty, String::new(), false).is_err());
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
    // What the guest got is what the medium holds at sector 1, byte for byte.
    let want = on_medium(&d, SECTOR_SIZE, SECTOR_SIZE as usize);
    let got: Vec<u8> = (0..SECTOR_SIZE)
        .map(|i| g.peek(DATA + i, Width::U8) as u8)
        .collect();
    assert_eq!(got, want);
}

#[test]
fn a_write_reaches_the_medium_and_stops_at_the_sector_boundary() {
    let g = Guest::new();
    let d = disk(4);
    for i in 0..SECTOR_SIZE {
        g.poke(DATA + i, Width::U8, 0xa5);
    }
    // Sector 3 is the last one, so the block *before* it is the neighbour: a
    // length computed in sectors and applied in bytes would land there.
    let before = on_medium(&d, SECTOR_SIZE, SECTOR_SIZE as usize);
    g.request(T_OUT, 2, SECTOR_SIZE as u32, false);
    let chain = g.chain();
    assert_eq!(d.handle(0, &g.queue(), &chain), 1, "just the status byte");
    assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);

    assert_eq!(
        on_medium(&d, 2 * SECTOR_SIZE, SECTOR_SIZE as usize),
        alloc::vec![0xa5u8; SECTOR_SIZE as usize],
        "the whole target sector"
    );
    assert_eq!(
        on_medium(&d, SECTOR_SIZE, SECTOR_SIZE as usize),
        before,
        "and the block before it is untouched"
    );
    assert_eq!(
        on_medium(&d, 3 * SECTOR_SIZE, SECTOR_SIZE as usize),
        (0..SECTOR_SIZE)
            .map(|i| ((1536 + i) % 251) as u8)
            .collect::<Vec<u8>>(),
        "and so is the block after it"
    );
}

#[test]
fn a_transfer_longer_than_one_chunk_crosses_the_staging_buffer() {
    // The staging buffer is 64 KiB, so a 96 KiB request takes two turns of the
    // loop, and an off-by-one in the second one would show as a seam.
    let sectors = (3 * CHUNK / SECTOR_SIZE) as usize / 2 * 2;
    let d = disk(sectors);
    // One long writable descriptor over a big enough RAM window.
    let big = AddressSpace::new("mem", 64);
    big.topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(0x40_0000))), 0)
        .unwrap();
    let layout = Layout {
        size: 8,
        desc: DESC,
        avail: AVAIL,
        used: USED,
        ready: true,
    };
    let big = Guest { space: big, layout };
    let len = (sectors as u64 * SECTOR_SIZE) as u32;
    let data = 0x10_0000u64;
    big.poke(HDR, Width::U32, u64::from(T_IN));
    big.poke(HDR + 8, Width::U64, 0);
    big.desc(0, HDR, HEADER_LEN as u32, DESC_F_NEXT, 1);
    big.desc(1, data, len, DESC_F_NEXT | DESC_F_WRITE, 2);
    big.desc(2, STATUS, 1, DESC_F_WRITE, 0);
    let chain = big.chain();
    assert_eq!(d.handle(0, &big.queue(), &chain), len + 1);
    assert_eq!(big.peek(STATUS, Width::U8) as u8, S_OK);

    let mut got = alloc::vec![0u8; len as usize];
    big.space
        .read_bytes(data, &mut got, MemAttrs::DEBUG)
        .expect("mapped");
    assert_eq!(got, on_medium(&d, 0, len as usize));

    // And back the other way: write the whole thing shifted by one byte, so a
    // chunk that reused a stale buffer would be visible.
    let mut send: Vec<u8> = got.clone();
    send.rotate_left(1);
    big.space
        .write_bytes(data, &send, MemAttrs::DEFAULT)
        .expect("mapped");
    big.poke(HDR, Width::U32, u64::from(T_OUT));
    big.desc(1, data, len, DESC_F_NEXT, 2);
    let chain = big.chain();
    d.handle(0, &big.queue(), &chain);
    assert_eq!(big.peek(STATUS, Width::U8) as u8, S_OK);
    assert_eq!(on_medium(&d, 0, len as usize), send);
}

#[test]
fn a_read_past_the_end_reports_an_io_error_rather_than_reading_something() {
    let g = Guest::new();
    let d = disk(2);
    g.request(T_IN, 99, SECTOR_SIZE as u32, true);
    let chain = g.chain();
    d.handle(0, &g.queue(), &chain);
    assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR);
    assert_eq!(g.peek(DATA, Width::U8), 0, "and nothing was placed");
}

#[test]
fn a_sector_number_that_overflows_the_byte_offset_is_refused() {
    // `sector * 512` is computed in `u64` and checked, so a driver naming a
    // sector near the top of the address space gets an I/O error rather than
    // an arithmetic panic — which is what an unchecked `at + len` used to give
    // in a debug build.
    let g = Guest::new();
    let d = disk(2);
    for sector in [u64::MAX, u64::MAX / SECTOR_SIZE, 1 << 62] {
        g.request(T_IN, sector, SECTOR_SIZE as u32, true);
        let chain = g.chain();
        d.handle(0, &g.queue(), &chain);
        assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR, "sector {sector}");
    }
}

#[test]
fn a_chain_claiming_gigabytes_costs_one_chunk_and_an_error() {
    // Nothing here trusts the guest, and a descriptor length is guest data. A
    // chain whose writable descriptors add up to four gigabytes must not turn
    // into a four-gigabyte allocation on the way to being refused.
    let g = Guest::new();
    let d = disk(2);
    g.poke(HDR, Width::U32, u64::from(T_IN));
    g.poke(HDR + 8, Width::U64, 0);
    g.desc(0, HDR, HEADER_LEN as u32, DESC_F_NEXT, 1);
    g.desc(1, DATA, u32::MAX, DESC_F_NEXT | DESC_F_WRITE, 2);
    g.desc(2, STATUS, 1, DESC_F_WRITE, 0);
    let chain = g.chain();
    d.handle(0, &g.queue(), &chain);
    assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR);
}

#[test]
fn a_write_to_a_read_only_device_is_refused_and_the_medium_is_untouched() {
    let g = Guest::new();
    let store = Arc::new(RamStore::new(1024)) as Arc<dyn Medium>;
    let d = VirtioBlk::new(store, String::new(), true).expect("two sectors");
    for i in 0..SECTOR_SIZE {
        g.poke(DATA + i, Width::U8, 0xa5);
    }
    g.request(T_OUT, 0, SECTOR_SIZE as u32, false);
    let chain = g.chain();
    d.handle(0, &g.queue(), &chain);
    assert_eq!(g.peek(STATUS, Width::U8) as u8, S_IOERR);
    assert!(d.is_read_only());
    assert_eq!(on_medium(&d, 0, 4), alloc::vec![0u8; 4]);
    // And the guest is told before it tries: §5.2.3's read-only bit.
    assert_eq!(d.features() & F_RO, F_RO);
    assert_eq!(disk(2).features() & F_RO, 0);
}

#[test]
fn a_medium_that_refuses_writes_read_only_protects_the_device_whatever_was_asked() {
    // The medium's own answer wins over the machine file's, for the reason
    // `ata.disk` gives: telling a guest it may write and then failing every
    // write is worse than an honest read-only disk.
    #[derive(Debug)]
    struct Locked(RamStore);
    impl Medium for Locked {
        fn capacity(&self) -> u64 {
            self.0.len()
        }
        fn read_at(&self, offset: u64, dst: &mut [u8]) -> crate::core::space::MemResult {
            RamStore::read_at(&self.0, offset, dst)
        }
        fn write_at(&self, _offset: u64, _src: &[u8]) -> crate::core::space::MemResult {
            Err(crate::core::error::BusError::Protected)
        }
        fn is_read_only(&self) -> bool {
            true
        }
    }
    let d = VirtioBlk::new(
        Arc::new(Locked(RamStore::new(1024))),
        String::new(),
        // The machine file said writable.
        false,
    )
    .expect("two sectors");
    assert!(d.is_read_only());
    assert_eq!(d.features() & F_RO, F_RO);
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
fn a_flush_is_offered_and_is_passed_to_the_medium() {
    let g = Guest::new();
    let d = disk(2);
    assert_eq!(d.features() & F_FLUSH, F_FLUSH, "§5.2.3 bit 9 is offered");
    g.request(T_FLUSH, 0, 1, true);
    let chain = g.chain();
    d.handle(0, &g.queue(), &chain);
    assert_eq!(g.peek(STATUS, Width::U8) as u8, S_OK);
}

#[test]
fn a_medium_that_cannot_be_made_durable_fails_the_flush() {
    #[derive(Debug)]
    struct NoSync(RamStore);
    impl Medium for NoSync {
        fn capacity(&self) -> u64 {
            self.0.len()
        }
        fn read_at(&self, offset: u64, dst: &mut [u8]) -> crate::core::space::MemResult {
            RamStore::read_at(&self.0, offset, dst)
        }
        fn write_at(&self, offset: u64, src: &[u8]) -> crate::core::space::MemResult {
            RamStore::write_at(&self.0, offset, src)
        }
        fn flush(&self) -> crate::core::space::MemResult {
            Err(crate::core::error::BusError::Unassigned)
        }
    }
    let g = Guest::new();
    let d = VirtioBlk::new(Arc::new(NoSync(RamStore::new(1024))), String::new(), false)
        .expect("two sectors");
    g.request(T_FLUSH, 0, 1, true);
    let chain = g.chain();
    d.handle(0, &g.queue(), &chain);
    assert_eq!(
        g.peek(STATUS, Width::U8) as u8,
        S_IOERR,
        "a barrier the host refused is not a barrier that succeeded"
    );
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
fn a_snapshot_of_a_capturing_medium_carries_it() {
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    let g = Guest::new();
    let saved = disk(2);
    for i in 0..SECTOR_SIZE {
        g.poke(DATA + i, Width::U8, 0x5a);
    }
    g.request(T_OUT, 0, SECTOR_SIZE as u32, false);
    let chain = g.chain();
    saved.handle(0, &g.queue(), &chain);
    assert_eq!(saved.medium().snapshot(), Snapshot::Capture);

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
    assert_eq!(on_medium(&restored, 0, 1)[0], 0x5a);

    // A disk of a different size is refused rather than truncated.
    let other = disk(4);
    let chunk = reader
        .load("disk", "virtio.blk", 1, &Migrations::new())
        .unwrap();
    assert!(other.load(&mut chunk.reader()).is_err());
}

#[test]
fn a_snapshot_of_a_referencing_medium_records_its_identity_and_flushes_it() {
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::Mutex;

    /// A medium that references rather than captures, and counts its flushes.
    #[derive(Debug)]
    struct Referenced {
        store: RamStore,
        name: &'static str,
        flushes: Mutex<u32>,
    }

    impl Medium for Referenced {
        fn capacity(&self) -> u64 {
            self.store.len()
        }
        fn read_at(&self, offset: u64, dst: &mut [u8]) -> crate::core::space::MemResult {
            RamStore::read_at(&self.store, offset, dst)
        }
        fn write_at(&self, offset: u64, src: &[u8]) -> crate::core::space::MemResult {
            RamStore::write_at(&self.store, offset, src)
        }
        fn flush(&self) -> crate::core::space::MemResult {
            *self.flushes.lock() += 1;
            Ok(())
        }
        fn snapshot(&self) -> Snapshot {
            Snapshot::Reference
        }
        fn describe(&self) -> String {
            String::from(self.name)
        }
    }

    let image = |name: &'static str| {
        Arc::new(Referenced {
            store: RamStore::new(1024),
            name,
            flushes: Mutex::new(0),
        })
    };

    let one = image("qcow2 /images/root.qcow2 1024");
    let saved = VirtioBlk::new(Arc::clone(&one) as Arc<dyn Medium>, String::new(), false)
        .expect("two sectors");

    let mut shape = MachineShape::new();
    shape.add_device("disk", "virtio.blk").unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("disk", "virtio.blk", 1).unwrap();
        saved.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    assert_eq!(
        *one.flushes.lock(),
        1,
        "the file is made to match the moment before it is referenced"
    );
    assert!(
        bytes.len() < 512,
        "the chunk holds an identity, not a kilobyte of disk: {} bytes",
        bytes.len()
    );

    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("disk", "virtio.blk", 1, &Migrations::new())
        .unwrap();
    saved.load(&mut chunk.reader()).expect("the same image");

    // A different image under the same slot is refused rather than trusted.
    let other = VirtioBlk::new(
        image("qcow2 /images/other.qcow2 1024") as Arc<dyn Medium>,
        String::new(),
        false,
    )
    .expect("two sectors");
    let chunk = reader
        .load("disk", "virtio.blk", 1, &Migrations::new())
        .unwrap();
    let e = other
        .load(&mut chunk.reader())
        .expect_err("a different medium");
    assert!(alloc::format!("{e}").contains("different medium"), "{e}");
}

#[test]
fn a_medium_that_refuses_to_be_snapshotted_fails_the_save_loudly() {
    use crate::core::state::{MachineShape, StateWriter};

    #[derive(Debug)]
    struct Refusing(RamStore);
    impl Medium for Refusing {
        fn capacity(&self) -> u64 {
            self.0.len()
        }
        fn read_at(&self, offset: u64, dst: &mut [u8]) -> crate::core::space::MemResult {
            RamStore::read_at(&self.0, offset, dst)
        }
        fn write_at(&self, offset: u64, src: &[u8]) -> crate::core::space::MemResult {
            RamStore::write_at(&self.0, offset, src)
        }
        fn snapshot(&self) -> Snapshot {
            Snapshot::Refuse
        }
        fn describe(&self) -> String {
            String::from("/dev/sda")
        }
    }

    let d = VirtioBlk::new(Arc::new(Refusing(RamStore::new(512))), String::new(), false)
        .expect("one sector");
    let mut shape = MachineShape::new();
    shape.add_device("disk", "virtio.blk").unwrap();
    let mut w = StateWriter::new(shape);
    let mut chunk = w.chunk("disk", "virtio.blk", 1).unwrap();
    let e = d.save(&mut chunk).expect_err("refused");
    assert!(alloc::format!("{e}").contains("refuses"), "{e}");
}
