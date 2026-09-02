//! `riscv-virt`'s virtio disk, backed by a **qcow2 file** rather than by a
//! buffer in host memory.
//!
//! Everything here is driven the way a guest drives it: the virtio-MMIO
//! register block at `0x10001000` is reached through the board's own address
//! space, the descriptor table, available ring and used ring live in the
//! board's DRAM, and one write to `QueueNotify` is what makes work happen. No
//! device object is reached for; nothing calls a method on `VirtioBlk`.
//!
//! The standard `tests/pc_at_ide.rs` and `tests/nvme_board.rs` hold is that a
//! transfer is checked **against the medium** rather than against the device's
//! own buffer, with the neighbouring block asserted untouched so that a length
//! computed in blocks and applied in bytes fails. This holds it too, and holds
//! it against an `Arc<Image>` the test kept on the host side of the seam —
//! which is the same object `rsemu run riscv-virt --drive disk=root.qcow2`
//! installs.
//!
//! # Source
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard: §4.2.2 for every
//! register offset written below, §2.1 for the status handshake, §2.7 for the
//! three rings, and §5.2.6 for the request layout. **No driver source of any
//! licence was opened** — `ROADMAP.md` §1 names Linux's virtio drivers
//! specifically. No image format is implemented in rsemu: qcow2 comes from
//! `fstool` (Karpelès Lab, MIT), which `ROADMAP.md` §7.1 names as the storage
//! substrate.

#![cfg(all(feature = "machine-riscv-virt", feature = "dev-blk"))]

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::ata::{Medium, Snapshot};
use rsemu::dev::blk::{Image, ImageOptions};
use rsemu::machine::Machine;

// -- the board ---------------------------------------------------------------

/// Where `machines/riscv-virt.machine` maps the block device.
const BLK: u64 = 0x1000_1000;
/// The device configuration space (§4.2.2).
const CONFIG: u64 = 0x100;

/// The disk this test makes: 4 MiB, which is deliberately *not* the machine
/// file's `storage = 16M`. A capacity of 8192 sectors is therefore proof on its
/// own that the medium the host installed won over the machine file's size.
const DISK_BYTES: u64 = 4 * 1024 * 1024;
const SECTOR: u64 = 512;

// The three rings and the three parts of a request, in DRAM.
const DESC: u64 = 0x8010_0000;
const AVAIL: u64 = 0x8010_1000;
const USED: u64 = 0x8010_2000;
const HDR: u64 = 0x8010_3000;
const DATA: u64 = 0x8010_4000;
const STATUS: u64 = 0x8010_5000;
const QSIZE: u64 = 8;

// §2.7.5 descriptor flags.
const F_NEXT: u64 = 1;
const F_WRITE: u64 = 2;

// §5.2.6 request types and status bytes.
const T_IN: u32 = 0;
const T_OUT: u32 = 1;
const T_FLUSH: u32 = 4;
const T_GET_ID: u32 = 8;
const S_OK: u8 = 0;
const S_IOERR: u8 = 1;

/// A scratch path nobody else is using, for this process and this call.
fn scratch(name: &str) -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rsemu-riscv-virtio-{}-{}-{name}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    path
}

/// What sector `lba` is stamped with, so a transfer that lands one sector out
/// fails rather than passing by luck.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; SECTOR as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// `riscv-virt` with a freshly written qcow2 under its `disk` media slot, and
/// a second handle to that image so every assertion can check the file.
fn board(name: &str) -> (Machine, Arc<Image>, PathBuf) {
    board_sized(name, DISK_BYTES)
}

/// [`board`], with the image's capacity chosen by the caller.
fn board_sized(name: &str, bytes: u64) -> (Machine, Arc<Image>, PathBuf) {
    let path = scratch(&format!("{name}.qcow2"));
    let image = Arc::new(
        Image::open(&path, &ImageOptions::new().create(bytes)).expect("a qcow2 is created"),
    );
    // Stamp the first few sectors and the last one. A qcow2 is sparse, so the
    // rest costs nothing and reads as zero.
    for lba in [0u64, 1, 2, 3, 4, 5, 6, 7, bytes / SECTOR - 1] {
        image
            .write_at(lba * SECTOR, &stamp(lba))
            .expect("in range and writable");
    }
    image.flush().expect("the host takes it");

    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    // What `rsemu run riscv-virt --drive disk=root.qcow2` does, and the whole
    // of what it does: the machine file still names a media slot and never a
    // host path.
    rsemu::dev::blk::install(&options.realize.hosts, "disk", Arc::clone(&image))
        .expect("nothing else claimed the slot");
    // Bound to no bytes: the medium above wins, and this is only how the
    // machine file's `image = "disk"` finds a slot at all.
    options.realize.media.insert("disk", Vec::new());
    options.realize.media.insert("firmware", Vec::new());
    options.realize.media.insert("initrd", Vec::new());
    options.realize.media.insert("flash0", Vec::new());
    options.realize.media.insert("flash1", Vec::new());
    let console = format!("test.riscv.virtio.{name}.console");
    let power = format!("test.riscv.virtio.{name}.power");
    options = options
        .with_param("console", console)
        .with_param("power", power);

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let entry = rsemu::machine::catalog::machine("riscv-virt").expect("this build ships it");
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("riscv-virt does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, image, path)
}

// -- the guest's side of the bus ---------------------------------------------

fn poke(m: &Machine, at: u64, width: Width, value: u64) {
    m.space("mem")
        .expect("the board has one space")
        .write(at, width, value, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("write {at:#x}: {e}"));
}

fn peek(m: &Machine, at: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the board has one space")
        .read(at, width, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read {at:#x}: {e}"))
}

/// A read with `MemAttrs::debug`, which a debugger and a monitor use and which
/// must move nothing.
fn spy(m: &Machine, at: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the board has one space")
        .read(at, width, MemAttrs::DEBUG)
        .unwrap_or_else(|e| panic!("debug read {at:#x}: {e}"))
}

fn reg(m: &Machine, off: u64) -> u32 {
    peek(m, BLK + off, Width::U32) as u32
}

fn set(m: &Machine, off: u64, value: u32) {
    poke(m, BLK + off, Width::U32, u64::from(value));
}

/// The §2.1 handshake and one queue, exactly in the order a driver does it.
fn bring_up(m: &Machine) {
    assert_eq!(reg(m, 0x000), 0x7472_6976, "MagicValue is `virt`");
    assert_eq!(reg(m, 0x004), 2, "the modern transport, not legacy");
    assert_eq!(reg(m, 0x008), 2, "device ID 2 is a block device (§5.2.1)");
    set(m, 0x070, 1); // ACKNOWLEDGE
    set(m, 0x070, 1 | 2); // | DRIVER
    // Accept everything offered, one 32-bit word at a time (§4.2.2).
    for word in [0u32, 1] {
        set(m, 0x014, word);
        let offered = reg(m, 0x010);
        set(m, 0x024, word);
        set(m, 0x020, offered);
    }
    set(m, 0x070, 1 | 2 | 8); // | FEATURES_OK
    assert_eq!(reg(m, 0x070) & 8, 8, "the device accepted the feature set");

    set(m, 0x030, 0); // QueueSel
    set(m, 0x038, QSIZE as u32); // QueueNum
    set(m, 0x080, DESC as u32);
    set(m, 0x084, (DESC >> 32) as u32);
    set(m, 0x090, AVAIL as u32);
    set(m, 0x094, (AVAIL >> 32) as u32);
    set(m, 0x0a0, USED as u32);
    set(m, 0x0a4, (USED >> 32) as u32);
    set(m, 0x044, 1); // QueueReady
    set(m, 0x070, 1 | 2 | 8 | 4); // | DRIVER_OK
}

fn desc(m: &Machine, index: u64, addr: u64, len: u32, flags: u64, next: u64) {
    let at = DESC + index * 16;
    poke(m, at, Width::U64, addr);
    poke(m, at + 8, Width::U32, u64::from(len));
    poke(m, at + 12, Width::U16, flags);
    poke(m, at + 14, Width::U16, next);
}

/// Lay out a three-descriptor request: header, one data buffer, the status
/// byte. The driver decides the split, and this is the split a real one uses.
fn request(m: &Machine, kind: u32, sector: u64, len: u32, writable: bool) {
    poke(m, HDR, Width::U32, u64::from(kind));
    poke(m, HDR + 4, Width::U32, 0);
    poke(m, HDR + 8, Width::U64, sector);
    poke(m, STATUS, Width::U8, 0xff);
    desc(m, 0, HDR, 16, F_NEXT, 1);
    desc(
        m,
        1,
        DATA,
        len,
        F_NEXT | if writable { F_WRITE } else { 0 },
        2,
    );
    desc(m, 2, STATUS, 1, F_WRITE, 0);
}

/// Offer the chain at descriptor 0 and ring the doorbell. Returns the status
/// byte and the used ring's length for the completed entry.
fn submit(m: &Machine, avail_idx: u16) -> (u8, u32) {
    poke(
        m,
        AVAIL + 4 + u64::from(avail_idx % QSIZE as u16) * 2,
        Width::U16,
        0,
    );
    poke(
        m,
        AVAIL + 2,
        Width::U16,
        u64::from(avail_idx.wrapping_add(1)),
    );
    set(m, 0x050, 0); // QueueNotify
    let slot = u64::from(avail_idx % QSIZE as u16);
    let len = peek(m, USED + 4 + slot * 8 + 4, Width::U32) as u32;
    let status = peek(m, STATUS, Width::U8) as u8;
    // Acknowledge the interrupt so the next request starts from a clean line.
    set(m, 0x064, reg(m, 0x060));
    (status, len)
}

/// `len` bytes of the image at `offset`, read through the `Medium` seam.
fn on_image(image: &Image, offset: u64, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    image.read_at(offset, &mut out).expect("in range");
    out
}

/// What the guest can see in its data buffer.
fn in_guest(m: &Machine, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    m.space("mem")
        .expect("the board has one space")
        .read_bytes(DATA, &mut out, MemAttrs::DEBUG)
        .expect("DRAM");
    out
}

// -- the tests ---------------------------------------------------------------

#[test]
fn the_capacity_the_guest_reads_is_the_images_and_not_the_machine_files() {
    let (m, image, path) = board("capacity");
    // §5.2.4: `capacity` is a little-endian 64-bit sector count at offset 0.
    let sectors = peek(&m, BLK + CONFIG, Width::U64);
    assert_eq!(sectors, DISK_BYTES / SECTOR);
    assert_eq!(sectors * SECTOR, image.capacity());
    assert_ne!(
        sectors * SECTOR,
        16 * 1024 * 1024,
        "the machine file's `storage = 16M` did not win over `--drive`"
    );
    drop(m);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_guest_read_comes_out_of_the_image() {
    let (m, image, path) = board("read");
    bring_up(&m);
    request(&m, T_IN, 3, SECTOR as u32, true);
    let (status, len) = submit(&m, 0);
    assert_eq!(status, S_OK);
    assert_eq!(len, SECTOR as u32 + 1, "the sector plus the status byte");
    assert_eq!(
        in_guest(&m, SECTOR as usize),
        on_image(&image, 3 * SECTOR, SECTOR as usize),
        "what the guest got is what the qcow2 holds"
    );
    assert_eq!(
        in_guest(&m, 2),
        vec![3, 0],
        "and it is sector 3, not 2 or 4"
    );
    drop(m);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_guest_write_reaches_the_image_and_stops_at_the_sector_boundary() {
    let (m, image, path) = board("write");
    bring_up(&m);

    let before_4 = on_image(&image, 4 * SECTOR, SECTOR as usize);
    let before_6 = on_image(&image, 6 * SECTOR, SECTOR as usize);
    let payload: Vec<u8> = (0..SECTOR)
        .map(|i| (i as u8).wrapping_mul(7) ^ 0xc3)
        .collect();
    m.space("mem")
        .expect("one space")
        .write_bytes(DATA, &payload, MemAttrs::DEFAULT)
        .expect("DRAM");

    request(&m, T_OUT, 5, SECTOR as u32, false);
    let (status, len) = submit(&m, 0);
    assert_eq!(status, S_OK);
    assert_eq!(len, 1, "a write puts only the status byte in the chain");

    // Against the medium, not against anything the device holds.
    assert_eq!(on_image(&image, 5 * SECTOR, SECTOR as usize), payload);
    assert_eq!(
        on_image(&image, 4 * SECTOR, SECTOR as usize),
        before_4,
        "the block before it is untouched"
    );
    assert_eq!(
        on_image(&image, 6 * SECTOR, SECTOR as usize),
        before_6,
        "and so is the block after it"
    );

    // And once the guest has asked for durability, the bytes are in the file
    // as far as anything outside this process is concerned.
    request(&m, T_FLUSH, 0, 1, true);
    assert_eq!(submit(&m, 1).0, S_OK);
    drop(m);

    // The strongest form of the claim: reopen the image from scratch, with the
    // machine and the first handle both gone.
    let reopened = Image::open(&path, &ImageOptions::new()).expect("still a qcow2");
    assert_eq!(on_image(&reopened, 5 * SECTOR, SECTOR as usize), payload);
    drop(reopened);
    drop(image);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_write_into_a_sparse_hole_survives_a_flush_and_a_reopen() {
    // The strongest durability claim, and the one a qcow2 makes hardest: the
    // target sector is in a cluster the image has never allocated, so serving
    // this write means allocating one and rewriting the L2 table, the L1 table
    // and the refcount blocks. `VIRTIO_BLK_T_FLUSH` is what the guest has to
    // say to make all of that reach the file — a data cluster written without
    // its metadata is a hole again the next time the image is opened.
    let (m, image, path) = board("sparse");
    bring_up(&m);
    // 1000 * 512 is inside the eighth 64 KiB cluster, which `board` never
    // touched.
    let far = 1000u64;
    let payload: Vec<u8> = (0..SECTOR).map(|i| (i as u8) ^ 0x3c).collect();
    assert_eq!(
        on_image(&image, far * SECTOR, SECTOR as usize),
        vec![0u8; SECTOR as usize],
        "the hole is a hole to start with"
    );
    m.space("mem")
        .expect("one space")
        .write_bytes(DATA, &payload, MemAttrs::DEFAULT)
        .expect("DRAM");
    request(&m, T_OUT, far, SECTOR as u32, false);
    assert_eq!(submit(&m, 0).0, S_OK);
    request(&m, T_FLUSH, 0, 1, true);
    assert_eq!(submit(&m, 1).0, S_OK);

    drop(m);
    drop(image);
    let reopened = Image::open(&path, &ImageOptions::new()).expect("still a qcow2");
    assert_eq!(
        on_image(&reopened, far * SECTOR, SECTOR as usize),
        payload,
        "a newly allocated cluster and the metadata that finds it both reached the file"
    );
    drop(reopened);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_multi_sector_transfer_lands_where_it_says_and_no_further() {
    // Eight sectors at once, which is more than one descriptor's worth of the
    // splits a driver uses and enough that a stride bug shows up.
    let (m, image, path) = board("multi");
    bring_up(&m);
    let count = 8u64;
    let bytes = (count * SECTOR) as usize;
    let payload: Vec<u8> = (0..bytes).map(|i| (i % 253) as u8).collect();
    m.space("mem")
        .expect("one space")
        .write_bytes(DATA, &payload, MemAttrs::DEFAULT)
        .expect("DRAM");

    let start = 64u64;
    let before = on_image(&image, (start + count) * SECTOR, SECTOR as usize);
    request(&m, T_OUT, start, bytes as u32, false);
    assert_eq!(submit(&m, 0).0, S_OK);
    assert_eq!(on_image(&image, start * SECTOR, bytes), payload);
    assert_eq!(
        on_image(&image, (start + count) * SECTOR, SECTOR as usize),
        before,
        "the sector after the run is untouched"
    );

    // And back: the same bytes come out.
    m.space("mem")
        .expect("one space")
        .write_bytes(DATA, &vec![0u8; bytes], MemAttrs::DEFAULT)
        .expect("DRAM");
    request(&m, T_IN, start, bytes as u32, true);
    let (status, len) = submit(&m, 1);
    assert_eq!(status, S_OK);
    assert_eq!(len, bytes as u32 + 1);
    assert_eq!(in_guest(&m, bytes), payload);
    drop(m);
    drop(image);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_request_past_the_end_of_the_image_is_an_io_error() {
    let (m, image, path) = board("past-end");
    bring_up(&m);
    let last = DISK_BYTES / SECTOR;
    request(&m, T_IN, last, SECTOR as u32, true);
    assert_eq!(submit(&m, 0).0, S_IOERR, "one past the last sector");
    request(&m, T_IN, u64::MAX, SECTOR as u32, true);
    assert_eq!(
        submit(&m, 1).0,
        S_IOERR,
        "and a sector whose byte offset does not fit in 64 bits"
    );
    // The last sector itself is fine, which is what makes the two above a
    // bound rather than a refusal to work.
    request(&m, T_IN, last - 1, SECTOR as u32, true);
    assert_eq!(submit(&m, 2).0, S_OK);
    assert_eq!(
        in_guest(&m, SECTOR as usize),
        on_image(&image, (last - 1) * SECTOR, SECTOR as usize)
    );
    drop(m);
    drop(image);
    let _ = std::fs::remove_file(path);
}

#[test]
fn get_id_reports_the_serial_the_machine_file_gave() {
    let (m, image, path) = board("get-id");
    bring_up(&m);
    request(&m, T_GET_ID, 0, 20, true);
    assert_eq!(submit(&m, 0).0, S_OK);
    let id = in_guest(&m, 20);
    assert_eq!(&id[..12], b"rsemu-virt-0");
    assert_eq!(id[12], 0, "and the rest is padding");
    drop(m);
    drop(image);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_debug_read_moves_nothing() {
    // Invariant 5: a debugger read must not advance a virtqueue index or
    // consume a used-ring entry.
    let (m, image, path) = board("debug");
    bring_up(&m);
    request(&m, T_IN, 1, SECTOR as u32, true);
    poke(&m, AVAIL + 4, Width::U16, 0);
    poke(&m, AVAIL + 2, Width::U16, 1);

    // Every transport register, plus the configuration space, read the way a
    // monitor reads them.
    for off in [
        0x000, 0x004, 0x008, 0x00c, 0x010, 0x034, 0x044, 0x050, 0x060, 0x070, 0x0fc,
    ] {
        let _ = spy(&m, BLK + off, Width::U32);
    }
    let _ = spy(&m, BLK + CONFIG, Width::U64);

    assert_eq!(
        spy(&m, USED + 2, Width::U16),
        0,
        "the used index never moved"
    );
    assert_eq!(
        spy(&m, STATUS, Width::U8) as u8,
        0xff,
        "and nothing was served"
    );
    assert_eq!(reg(&m, 0x060), 0, "no interrupt was raised");

    // A write with the debug attribute is refused outright rather than
    // silently doing the thing: `QueueNotify` cannot be made side-effect free.
    assert!(
        m.space("mem")
            .expect("one space")
            .write(BLK + 0x050, Width::U32, 0, MemAttrs::DEBUG)
            .is_err()
    );
    assert_eq!(spy(&m, USED + 2, Width::U16), 0);

    // And the ordinary path still works, which is what makes the above a
    // property of the attribute rather than of a broken queue.
    set(&m, 0x050, 0);
    assert_eq!(peek(&m, USED + 2, Width::U16), 1);
    assert_eq!(peek(&m, STATUS, Width::U8) as u8, S_OK);
    assert_eq!(
        in_guest(&m, SECTOR as usize),
        on_image(&image, SECTOR, SECTOR as usize)
    );
    drop(m);
    drop(image);
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_snapshot_references_the_image_and_round_trips_to_the_same_hash() {
    let (mut m, image, path) = board("snapshot");
    bring_up(&m);
    let payload: Vec<u8> = (0..SECTOR).map(|i| (i as u8) ^ 0x99).collect();
    m.space("mem")
        .expect("one space")
        .write_bytes(DATA, &payload, MemAttrs::DEFAULT)
        .expect("DRAM");
    request(&m, T_OUT, 2, SECTOR as u32, false);
    assert_eq!(submit(&m, 0).0, S_OK);

    assert_eq!(
        image.snapshot(),
        Snapshot::Reference,
        "a file-backed disk references rather than copying"
    );
    let saved = m.save().expect("the machine saves");
    let before = m.state_hash().expect("the machine hashes");
    m.load(&saved).expect("its own snapshot loads");
    assert_eq!(m.state_hash().expect("hashes"), before);

    // `save` flushed the image first, so what is on disk matches the moment.
    assert_eq!(on_image(&image, 2 * SECTOR, SECTOR as usize), payload);
    drop(m);
    drop(image);
    let _ = std::fs::remove_file(path);

    // And the falsifiable form of "references rather than captures": a disk
    // sixteen times the size produces a snapshot the same size, give or take
    // the digits in the identity string. A capturing disk would be 60 MiB
    // larger. (The board's DRAM and its two NOR banks dominate the total, so
    // comparing two runs is the honest test rather than an absolute bound.)
    let (small, small_image, small_path) = board_sized("snapshot-small", DISK_BYTES);
    let small_bytes = small.save().expect("saves").len();
    drop(small);
    drop(small_image);
    let _ = std::fs::remove_file(small_path);

    let (big, big_image, big_path) = board_sized("snapshot-large", 16 * DISK_BYTES);
    let big_bytes = big.save().expect("saves").len();
    drop(big);
    drop(big_image);
    let _ = std::fs::remove_file(big_path);

    assert!(
        small_bytes.abs_diff(big_bytes) < 256,
        "a {}-byte disk and a {}-byte disk gave {small_bytes}- and {big_bytes}-byte snapshots",
        DISK_BYTES,
        16 * DISK_BYTES
    );
}

#[test]
fn the_media_slot_path_still_gives_a_disk_of_the_machine_files_size() {
    // The `no_std` contract, unchanged: bytes bound to a named media slot, no
    // host file anywhere, and the machine file's `storage` is the capacity.
    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    let front = stamp(0);
    options.realize.media.insert("disk", front.clone());
    options.realize.media.insert("firmware", Vec::new());
    options.realize.media.insert("initrd", Vec::new());
    options.realize.media.insert("flash0", Vec::new());
    options.realize.media.insert("flash1", Vec::new());
    let options = options
        .with_param("console", "test.riscv.virtio.slot.console")
        .with_param("power", "test.riscv.virtio.slot.power");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let entry = rsemu::machine::catalog::machine("riscv-virt").expect("shipped");
    let mut m = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect("riscv-virt realizes on a media slot");
    m.reset(ResetKind::Cold);
    m.sweep();

    assert_eq!(
        peek(&m, BLK + CONFIG, Width::U64),
        16 * 1024 * 1024 / SECTOR,
        "`storage = 16M`, padded out with zeroes"
    );
    bring_up(&m);
    request(&m, T_IN, 0, SECTOR as u32, true);
    assert_eq!(submit(&m, 0).0, S_OK);
    assert_eq!(in_guest(&m, SECTOR as usize), front, "the bound bytes");
}
