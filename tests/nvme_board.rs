//! An NVM Express controller, driven the way a driver drives one.
//!
//! `tests/pc_at_ide.rs` is the standard this is written to: everything here
//! goes through the machine's own address spaces — configuration cycles at
//! `0xcf8`/`0xcfc`, the register block at wherever this test's driver decided
//! to put the base address register, and the interrupt controller at `0x20` —
//! and nothing reaches into the device except to check the **medium**, which is
//! the point. A model that never touched a medium would pass a test that
//! compared a read against the device's own buffer; it cannot pass this one.
//!
//! The order is weakest claim first:
//!
//! * the bus shows a class `010802h` function whose BAR sizes as 8 KiB;
//! * the controller refuses to come ready with an admin queue it cannot use,
//!   and comes ready with one it can;
//! * `Identify` describes the namespace the board was actually given;
//! * a block **read** arrives from the medium and a block **write** reaches it,
//!   both DMA'd by the controller into memory this test never handed it
//!   directly;
//! * a transfer too big for two pages walks a **PRP list** the driver built;
//! * the completion interrupt travels a **wire** into an 8259A, is visible in
//!   its interrupt request register, and drops when the driver writes the
//!   completion queue head doorbell;
//! * a debugger may read every register and may write none;
//! * the board snapshots and restores to an identical state hash.

#![cfg(feature = "machine-nvme-mini")]

use std::sync::Arc;

use rsemu::core::device::ResetKind;
use rsemu::core::space::{MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::dev::medium::Medium;
use rsemu::machine::Machine;

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Bytes in a logical block on the test namespace.
const LBA: u64 = 512;

/// How many blocks the namespace holds. 4096 of them is 2 MiB, which is what
/// `machines/nvme-mini.machine`'s `disk` parameter defaults to.
const BLOCKS: u64 = 4096;

/// What block `lba` holds on a freshly stamped namespace.
///
/// Every block says which block it is, so a transfer that lands one block out —
/// the classic off-by-one in an LBA computation — fails rather than passing on
/// identical zeroes.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; LBA as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// The whole namespace image.
fn image() -> Vec<u8> {
    let mut out = Vec::with_capacity((BLOCKS * LBA) as usize);
    for lba in 0..BLOCKS {
        out.extend_from_slice(&stamp(lba));
    }
    out
}

/// The board, and the medium the *host* installed under its media slot.
///
/// The medium is kept on this side of the seam deliberately: `--drive
/// nvme0=disk.img` installs one exactly like this, and holding a second handle
/// to it is what lets every assertion below check the bytes that reached
/// storage rather than the bytes the controller says it moved.
fn board() -> (Machine, Arc<RamStore>) {
    let store = Arc::new(RamStore::new(BLOCKS * LBA));
    RamStore::write_at(&store, 0, &image()).expect("the image fits");

    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    // The machine file names the slot; the run says what is behind it.
    rsemu::dev::medium::install(
        &options.realize.hosts,
        "nvme0",
        Arc::clone(&store) as Arc<dyn Medium>,
    )
    .expect("nothing else claimed the name");
    // Bound to no bytes: the medium above wins, and this is only how the
    // machine file's `image = "nvme0"` finds a slot at all.
    options.realize.media.insert("nvme0", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let entry = &rsemu::machine::catalog::NVME_MINI;
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, store)
}

// ---------------------------------------------------------------------------
// the two spaces, as a driver reaches them
// ---------------------------------------------------------------------------

fn outb(m: &Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

fn inb(m: &Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

fn peek32(m: &Machine, addr: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped dword") as u32
}

fn poke32(m: &Machine, addr: u64, value: u32) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped dword");
}

fn poke_bytes(m: &Machine, addr: u64, bytes: &[u8]) {
    m.space("mem")
        .expect("the memory space")
        .write_bytes(addr, bytes, MemAttrs::DEFAULT)
        .expect("mapped memory");
}

fn peek_bytes(m: &Machine, addr: u64, len: u64) -> Vec<u8> {
    let mut out = vec![0u8; len as usize];
    m.space("mem")
        .expect("the memory space")
        .read_bytes(addr, &mut out, MemAttrs::DEFAULT)
        .expect("mapped memory");
    out
}

// ---------------------------------------------------------------------------
// configuration space, mechanism #1
// ---------------------------------------------------------------------------

/// Where the controller sits: bus 0, device 4, function 0, as the machine file
/// says.
const NVME_DEVICE: u32 = 4;

fn config_read(m: &Machine, register: u16) -> u32 {
    let addr = 0x8000_0000 | (NVME_DEVICE << 11) | u32::from(register & 0xfc);
    m.space("port")
        .expect("the I/O space")
        .write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    m.space("port")
        .expect("the I/O space")
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("CONFDATA answers") as u32
}

fn config_write(m: &Machine, register: u16, value: u32) {
    let addr = 0x8000_0000 | (NVME_DEVICE << 11) | u32::from(register & 0xfc);
    m.space("port")
        .expect("the I/O space")
        .write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    m.space("port")
        .expect("the I/O space")
        .write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("CONFDATA takes a dword");
}

/// The Type 00h header offsets this test names.
const CFG_VENDOR: u16 = 0x00;
const CFG_COMMAND: u16 = 0x04;
const CFG_CLASS: u16 = 0x08;
const CFG_BAR0: u16 = 0x10;
const CFG_BAR1: u16 = 0x14;
const CFG_INT_PIN: u16 = 0x3c;

/// `COMMAND[1]` and `COMMAND[2]`: memory space and bus master.
const COMMAND_ON: u32 = 0x0006;

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

/// Where this test's driver puts the controller's 8 KiB register window. Above
/// the board's 16 MiB of RAM, which is where firmware allocates from.
const BAR_BASE: u64 = 0xf000_0000;

const REG_CAP: u64 = 0x00;
const REG_VS: u64 = 0x08;
const REG_INTMS: u64 = 0x0c;
const REG_CC: u64 = 0x14;
const REG_CSTS: u64 = 0x1c;
const REG_AQA: u64 = 0x24;
const REG_ASQ: u64 = 0x28;
const REG_ACQ: u64 = 0x30;

/// `CSTS.RDY` and `CSTS.CFS`.
const CSTS_RDY: u32 = 1;
const CSTS_CFS: u32 = 2;

/// `CC` as a driver writes it to start a controller, **as a literal**.
///
/// *NVM Express* base specification, Figure "Controller Configuration": `EN`
/// is bit 0, `CSS` bits 06:04, `MPS` 10:07, `AMS` 13:11, `SHN` 15:14, `IOSQES`
/// 19:16 and `IOCQES` 23:20. So `0x0046_0001` is `EN` set, the NVM command set,
/// 4 KiB pages, round-robin arbitration, no shutdown notification, 64-byte
/// submission entries (`IOSQES` 6) and 16-byte completion entries (`IOCQES` 4)
/// — the two entry sizes NVMe defines, and the word a Linux 6.6 kernel was
/// measured writing to this controller.
///
/// It is spelled out rather than assembled from named shifts on purpose: a
/// test that builds `CC` from the same constants the model decodes it with
/// asserts only that the file agrees with itself, and passes just as happily
/// with every field one bit out of place. That is precisely the defect this
/// constant exists to catch.
const CC_ENABLE: u32 = 0x0046_0001;

/// The same, with `IOCQES` 5 — a 32-byte completion entry, which NVMe does not
/// define and this controller must refuse.
const CC_BAD_IOCQES: u32 = 0x0056_0001;

/// `CC.SHN` = 01b, a normal shutdown: bits 15:14, so bit 14.
const CC_SHN_NORMAL: u32 = 0x0000_4000;

/// Where the queues live in the board's RAM.
const ASQ: u64 = 0x0010_0000;
const ACQ: u64 = 0x0010_1000;
const IOSQ: u64 = 0x0010_2000;
const IOCQ: u64 = 0x0010_3000;
/// Where a data buffer goes.
const DATA: u64 = 0x0020_0000;
/// Where a PRP list goes.
const PRP_LIST: u64 = 0x0030_0000;

/// Entries in each queue. Sixteen admin, eight I/O — both zero-based in the
/// registers that carry them, which is exactly the off-by-one this test would
/// rather find here than in a driver.
const ADMIN_ENTRIES: u32 = 16;
const IO_ENTRIES: u32 = 8;

fn reg32(m: &Machine, offset: u64) -> u32 {
    peek32(m, BAR_BASE + offset)
}

fn set_reg32(m: &Machine, offset: u64, value: u32) {
    poke32(m, BAR_BASE + offset, value);
}

fn set_reg64(m: &Machine, offset: u64, value: u64) {
    // Two dwords, low half first: §3.1 permits either, and this is what a
    // 32-bit driver has to do.
    set_reg32(m, offset, value as u32);
    set_reg32(m, offset + 4, (value >> 32) as u32);
}

/// The doorbell for queue `qid`: the submission tail, or the completion head.
fn doorbell(qid: u64, completion: bool) -> u64 {
    0x1000 + (2 * qid + u64::from(completion)) * 4
}

// ---------------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------------

/// The fields of a submission queue entry this test fills in (NVMe 1.4 §4.2).
#[derive(Debug, Clone, Copy, Default)]
struct Sqe {
    opcode: u8,
    cid: u16,
    nsid: u32,
    prp1: u64,
    prp2: u64,
    cdw10: u32,
    cdw11: u32,
    cdw12: u32,
}

impl Sqe {
    fn bytes(&self) -> [u8; 64] {
        let mut out = [0u8; 64];
        let cdw0 = u32::from(self.opcode) | (u32::from(self.cid) << 16);
        out[0..4].copy_from_slice(&cdw0.to_le_bytes());
        out[4..8].copy_from_slice(&self.nsid.to_le_bytes());
        out[24..32].copy_from_slice(&self.prp1.to_le_bytes());
        out[32..40].copy_from_slice(&self.prp2.to_le_bytes());
        out[40..44].copy_from_slice(&self.cdw10.to_le_bytes());
        out[44..48].copy_from_slice(&self.cdw11.to_le_bytes());
        out[48..52].copy_from_slice(&self.cdw12.to_le_bytes());
        out
    }
}

/// One completion queue entry, as a driver reads it back (§4.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cqe {
    dw0: u32,
    sqhd: u16,
    sqid: u16,
    cid: u16,
    phase: bool,
    status: u16,
}

fn read_cqe(m: &Machine, base: u64, slot: u32) -> Cqe {
    let at = base + u64::from(slot) * 16;
    let dw0 = peek32(m, at);
    let dw2 = peek32(m, at + 8);
    let dw3 = peek32(m, at + 12);
    Cqe {
        dw0,
        sqhd: dw2 as u16,
        sqid: (dw2 >> 16) as u16,
        cid: dw3 as u16,
        phase: dw3 & (1 << 16) != 0,
        status: (dw3 >> 17) as u16,
    }
}

/// A driver's whole state: which slot each queue is up to.
#[derive(Debug, Default)]
struct Driver {
    admin_tail: u32,
    admin_head: u32,
    io_tail: u32,
    /// The slot the *next* completion will land in.
    io_next: u32,
    /// The slot the controller has been told about, which is what holds the
    /// interrupt line down while the two differ.
    io_head: u32,
    cid: u16,
}

impl Driver {
    /// Put `cmd` on the admin queue, ring the doorbell, and read the completion
    /// back. Everything in between is the controller's own doing.
    fn admin(&mut self, m: &Machine, mut cmd: Sqe) -> Cqe {
        self.cid = self.cid.wrapping_add(1);
        cmd.cid = self.cid;
        poke_bytes(m, ASQ + u64::from(self.admin_tail) * 64, &cmd.bytes());
        self.admin_tail = (self.admin_tail + 1) % ADMIN_ENTRIES;
        set_reg32(m, doorbell(0, false), self.admin_tail);
        let cqe = read_cqe(m, ACQ, self.admin_head);
        assert_eq!(cqe.cid, cmd.cid, "the completion is for another command");
        self.admin_head = (self.admin_head + 1) % ADMIN_ENTRIES;
        set_reg32(m, doorbell(0, true), self.admin_head);
        cqe
    }

    /// The same for I/O queue 1.
    fn io(&mut self, m: &Machine, mut cmd: Sqe) -> Cqe {
        self.cid = self.cid.wrapping_add(1);
        cmd.cid = self.cid;
        poke_bytes(m, IOSQ + u64::from(self.io_tail) * 64, &cmd.bytes());
        self.io_tail = (self.io_tail + 1) % IO_ENTRIES;
        set_reg32(m, doorbell(1, false), self.io_tail);
        let cqe = read_cqe(m, IOCQ, self.io_next);
        assert_eq!(cqe.cid, cmd.cid, "the completion is for another command");
        self.io_next = (self.io_next + 1) % IO_ENTRIES;
        cqe
    }

    /// Acknowledge the completion `io` just read, which is what lowers the
    /// interrupt line.
    fn ack_io(&mut self, m: &Machine) {
        self.io_head = (self.io_head + 1) % IO_ENTRIES;
        set_reg32(m, doorbell(1, true), self.io_head);
    }
}

// ---------------------------------------------------------------------------
// bringing the controller up, as a driver does
// ---------------------------------------------------------------------------

/// Enumerate, place the register window, and switch the function on.
fn place_bar(m: &Machine) {
    // §6.2.5.1's sizing protocol: all ones in, and the window's size out.
    config_write(m, CFG_BAR0, 0xffff_ffff);
    config_write(m, CFG_BAR1, 0xffff_ffff);
    config_write(m, CFG_BAR0, BAR_BASE as u32);
    config_write(m, CFG_BAR1, (BAR_BASE >> 32) as u32);
    config_write(m, CFG_COMMAND, COMMAND_ON);
}

/// The 8259A, initialised the way a driver of a level-triggered PCI interrupt
/// has to initialise one.
fn init_pic(m: &Machine) {
    outb(m, 0x20, 0x11); // ICW1: cascade, ICW4 to follow
    outb(m, 0x21, 0x08); // ICW2: vectors from 0x08
    outb(m, 0x21, 0x04); // ICW3: a slave would be on IR2
    outb(m, 0x21, 0x01); // ICW4: 8086 mode
    outb(m, 0x21, 0xdf); // OCW1: everything masked but IR5
    // The edge/level control register: IR5 is level triggered, because NVMe's
    // `INTx#` is a level (§7.5.1.1) and an edge-triggered input would latch the
    // first completion and miss every later one raised while the line was
    // already low.
    outb(m, 0x4d0, 1 << 5);
}

/// The interrupt request register, which for a level-triggered line is the line
/// itself.
fn irr(m: &Machine) -> u8 {
    outb(m, 0x20, 0x0a); // OCW3: the next read of port 0 is the IRR
    inb(m, 0x20)
}

/// Build the admin queues and set `CC.EN`, then spin on `CSTS.RDY`.
fn enable(m: &Machine) {
    // §3.1.8: both fields are zero-based.
    set_reg32(
        m,
        REG_AQA,
        (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16),
    );
    set_reg64(m, REG_ASQ, ASQ);
    set_reg64(m, REG_ACQ, ACQ);
    // §3.1.5: the NVM command set, 4 KiB pages, 64-byte submission entries and
    // 16-byte completion entries, and go.
    set_reg32(m, REG_CC, CC_ENABLE);
    for _ in 0..100 {
        if reg32(m, REG_CSTS) & CSTS_RDY != 0 {
            return;
        }
    }
    panic!(
        "the controller never came ready: CSTS = {:#x}",
        reg32(m, REG_CSTS)
    );
}

/// Create I/O completion queue 1 and I/O submission queue 1 on it.
fn create_io_queues(m: &Machine, d: &mut Driver) {
    // §5.3: QID in CDW10's low half, the zero-based size in its high half;
    // physically contiguous, interrupts enabled, vector 0.
    let cqe = d.admin(
        m,
        Sqe {
            opcode: 0x05,
            prp1: IOCQ,
            cdw10: 1 | ((IO_ENTRIES - 1) << 16),
            cdw11: 0x0003,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "Create I/O Completion Queue failed");
    // §5.4: the same shape, with the completion queue's id in CDW11's high half.
    let cqe = d.admin(
        m,
        Sqe {
            opcode: 0x01,
            prp1: IOSQ,
            cdw10: 1 | ((IO_ENTRIES - 1) << 16),
            cdw11: 0x0001 | (1 << 16),
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "Create I/O Submission Queue failed");
}

/// A board with the controller placed, enabled and carrying one I/O queue pair
/// — everything a driver does before its first read.
fn ready() -> (Machine, Arc<RamStore>, Driver) {
    let (m, store) = board();
    init_pic(&m);
    place_bar(&m);
    enable(&m);
    let mut d = Driver::default();
    create_io_queues(&m, &mut d);
    (m, store, d)
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_bus_shows_a_storage_controller_with_a_64_bit_register_window() {
    // The weakest claim, and the one everything else rests on: a driver finds
    // this device by class code and reaches it through a window it sizes.
    let (m, _store) = board();

    let ids = config_read(&m, CFG_VENDOR);
    assert_eq!(ids & 0xffff, 0x1234, "the vendor the machine file declares");
    // Class 01h mass storage, sub-class 08h non-volatile memory, programming
    // interface 02h NVM Express — `010802h`, which is what a driver enumerates
    // for (NVMe 1.4 §2.1).
    assert_eq!(
        config_read(&m, CFG_CLASS) >> 8,
        0x0001_0802,
        "the class code an NVMe driver looks for"
    );
    assert_eq!(
        config_read(&m, CFG_INT_PIN) >> 8 & 0xff,
        0x01,
        "the function drives INTA#"
    );

    // Out of reset the function decodes nothing, so the window is not there.
    assert_eq!(
        config_read(&m, CFG_COMMAND) & 0xffff,
        0,
        "every enable bit is clear until firmware sets one"
    );
    assert_eq!(
        m.space("mem")
            .expect("the memory space")
            .read(BAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .expect("read-as-ones"),
        0xffff_ffff,
        "nothing decodes at the base until the BAR is placed and enabled"
    );

    // §6.2.5.1's sizing protocol.
    config_write(&m, CFG_BAR0, 0xffff_ffff);
    config_write(&m, CFG_BAR1, 0xffff_ffff);
    // 8 KiB of memory, 64-bit (type 10b in bits 2:1), not prefetchable.
    assert_eq!(config_read(&m, CFG_BAR0), 0xffff_e004);
    assert_eq!(config_read(&m, CFG_BAR1), 0xffff_ffff);

    place_bar(&m);
    assert_eq!(config_read(&m, CFG_BAR0), BAR_BASE as u32 | 0x4);
    // And now the register block answers where the driver put it.
    assert_eq!(reg32(&m, REG_VS), 0x0001_0400, "NVMe 1.4.0");
    let cap = u64::from(reg32(&m, REG_CAP)) | (u64::from(reg32(&m, REG_CAP + 4)) << 32);
    assert_eq!(cap & 0xffff, 1023, "CAP.MQES, zero-based");
    assert_eq!(
        (cap >> 32) & 0xf,
        0,
        "CAP.DSTRD: a four-byte doorbell stride"
    );
    assert_eq!((cap >> 37) & 1, 1, "CAP.CSS: the NVM command set");
}

#[test]
fn the_controller_refuses_an_admin_queue_it_cannot_use() {
    // §3.1.5: a controller that cannot accept the configuration must not set
    // CSTS.RDY, and §3.1.6 gives it CSTS.CFS to say so. Coming ready anyway
    // would leave a driver waiting on a queue that was never built.
    let (m, _store) = board();
    place_bar(&m);

    // An admin submission queue base that is not page aligned.
    set_reg32(&m, REG_AQA, 0x000f_000f);
    set_reg64(&m, REG_ASQ, ASQ + 8);
    set_reg64(&m, REG_ACQ, ACQ);
    set_reg32(&m, REG_CC, CC_ENABLE);
    let csts = reg32(&m, REG_CSTS);
    assert_eq!(csts & CSTS_RDY, 0, "it must not come ready");
    assert_eq!(csts & CSTS_CFS, CSTS_CFS, "and it must say why");

    // A completion entry size that is not sixteen bytes is refused the same
    // way, on a controller that has been reset out of its fatal state.
    let (m, _store) = board();
    place_bar(&m);
    set_reg32(&m, REG_AQA, 0x000f_000f);
    set_reg64(&m, REG_ASQ, ASQ);
    set_reg64(&m, REG_ACQ, ACQ);
    set_reg32(&m, REG_CC, CC_BAD_IOCQES);
    assert_eq!(reg32(&m, REG_CSTS) & CSTS_RDY, 0);

    // And the configuration this test's driver actually uses works.
    let (m, _store) = board();
    place_bar(&m);
    enable(&m);
    assert_eq!(reg32(&m, REG_CSTS), CSTS_RDY);
}

/// `CC` carries its fields where the *specification* puts them, decided by
/// literal register words rather than by this file's own arithmetic.
///
/// The companion of [`CC_ENABLE`]. Each word below is chosen so that the same
/// bit pattern means something different if a field sits one bit out of place,
/// and the assertion is on what the controller *does* with it — so a model that
/// agrees with a test that agrees with the model cannot make this pass.
///
/// *NVM Express* base specification, Figure "Controller Configuration": `EN` 0,
/// `CSS` 06:04, `MPS` 10:07, `AMS` 13:11, `SHN` 15:14, `IOSQES` 19:16,
/// `IOCQES` 23:20, and 31:24 reserved.
#[test]
fn cc_carries_its_fields_where_the_specification_puts_them() {
    /// Ready the admin queue registers, which every case below needs.
    fn queues(m: &Machine) {
        set_reg32(
            m,
            REG_AQA,
            (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16),
        );
        set_reg64(m, REG_ASQ, ASQ);
        set_reg64(m, REG_ACQ, ACQ);
    }

    // The word a current Linux kernel writes. Read with the specification's
    // field positions it is IOSQES 6 / IOCQES 4; one bit higher it is 3 and 2,
    // neither of which NVMe defines, and the controller would refuse it.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, CC_ENABLE);
    let csts = reg32(&m, REG_CSTS);
    assert_eq!(
        csts & CSTS_CFS,
        0,
        "CC = {CC_ENABLE:#010x} is the configuration a Linux kernel writes and \
         the controller reported a fatal status (CSTS = {csts:#010x})"
    );
    assert_eq!(csts & CSTS_RDY, CSTS_RDY, "and it must come ready");
    // Every field in that word is implemented, so it reads back whole: a
    // reserved-bit mask that swallowed part of one would show up here.
    assert_eq!(
        reg32(&m, REG_CC),
        CC_ENABLE,
        "CC reads back what was written"
    );

    // The mirror image: the word that would carry IOSQES 6 / IOCQES 4 if the
    // fields sat one bit higher. Its real IOSQES is 12, which is not a size
    // NVMe defines, so a controller reading CC correctly refuses it.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, 1 | (6 << 17) | (4 << 21));
    let csts = reg32(&m, REG_CSTS);
    assert_eq!(
        csts & CSTS_RDY,
        0,
        "IOSQES 12 is not a submission entry size"
    );
    assert_eq!(csts & CSTS_CFS, CSTS_CFS, "and CSTS.CFS says so");

    // MPS is four bits at 10:07, and CAP.MPSMAX is 4 (64 KiB). MPS 5 asks for
    // a 128 KiB page: refused. Its bits are 10:08, so a model that dropped the
    // top of MPS as reserved would read this word as MPS 0 and come ready.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, CC_ENABLE | (5 << 7));
    let csts = reg32(&m, REG_CSTS);
    assert_eq!(csts & CSTS_RDY, 0, "MPS 5 exceeds CAP.MPSMAX");
    assert_eq!(csts & CSTS_CFS, CSTS_CFS, "and CSTS.CFS says so");

    // AMS is 13:11, and CAP.AMS advertises round robin only, so 001b — the
    // weighted round robin with urgent priority class — is refused.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, CC_ENABLE | (1 << 11));
    let csts = reg32(&m, REG_CSTS);
    assert_eq!(csts & CSTS_RDY, 0, "AMS 001b is not advertised by CAP.AMS");
    assert_eq!(csts & CSTS_CFS, CSTS_CFS, "and CSTS.CFS says so");

    // SHN is 15:14, so a normal shutdown notification is bit 14, and §3.1.6
    // answers it with SHST = 10b once the controller has processed it.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, CC_ENABLE);
    set_reg32(&m, REG_CC, CC_ENABLE | CC_SHN_NORMAL);
    assert_eq!(
        reg32(&m, REG_CSTS) >> 2 & 0x3,
        0b10,
        "CSTS.SHST: shutdown processing complete"
    );
    // Bit 15 alone is SHN = 10b, an abrupt shutdown, which is also a shutdown
    // — while bit 16, the first reserved bit above SHN, is not a shutdown at
    // all and must be dropped rather than stored.
    let (m, _store) = board();
    place_bar(&m);
    queues(&m);
    set_reg32(&m, REG_CC, CC_ENABLE);
    set_reg32(&m, REG_CC, CC_ENABLE | (1 << 16));
    assert_eq!(
        reg32(&m, REG_CSTS) >> 2 & 0x3,
        0,
        "bit 16 is reserved: no shutdown, and CSTS.SHST stays 00b"
    );
    assert_eq!(
        reg32(&m, REG_CC),
        CC_ENABLE,
        "and the reserved bit is not stored"
    );
}

#[test]
fn identify_describes_the_namespace_the_board_was_given() {
    let (m, store, mut d) = ready();

    // CNS 01h: the controller (§5.15).
    let cqe = d.admin(
        &m,
        Sqe {
            opcode: 0x06,
            prp1: DATA,
            cdw10: 0x01,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    let ctrl = peek_bytes(&m, DATA, 4096);
    let model = String::from_utf8_lossy(&ctrl[24..64])
        .trim_end()
        .to_string();
    assert_eq!(model, "RSEMU NVME CONTROLLER");
    assert_eq!(ctrl[512], 0x66, "SQES: 64-byte submission entries");
    assert_eq!(ctrl[513], 0x44, "CQES: 16-byte completion entries");
    assert_eq!(
        u32::from_le_bytes([ctrl[516], ctrl[517], ctrl[518], ctrl[519]]),
        1,
        "NN: one namespace"
    );

    // CNS 00h: the namespace itself.
    let cqe = d.admin(
        &m,
        Sqe {
            opcode: 0x06,
            nsid: 1,
            prp1: DATA + 0x1000,
            cdw10: 0x00,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    let ns = peek_bytes(&m, DATA + 0x1000, 4096);
    let nsze = u64::from_le_bytes(ns[0..8].try_into().expect("eight bytes"));
    assert_eq!(nsze, BLOCKS, "NSZE, in logical blocks");
    // LBA Format 0's LBADS is the block size as a power of two, in bits 23:16.
    let lbaf0 = u32::from_le_bytes(ns[128..132].try_into().expect("four bytes"));
    assert_eq!(1u64 << ((lbaf0 >> 16) & 0xff), LBA);
    assert_eq!(
        nsze * (1 << ((lbaf0 >> 16) & 0xff)),
        Medium::capacity(&*store),
        "Identify and the medium must not be describing two different disks"
    );

    // A namespace that is not there is an Invalid Namespace, not a zero page.
    let cqe = d.admin(
        &m,
        Sqe {
            opcode: 0x06,
            nsid: 2,
            prp1: DATA + 0x2000,
            cdw10: 0x00,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0x000b, "generic status 0Bh");
}

#[test]
fn a_block_read_comes_off_the_medium_and_a_write_reaches_it() {
    // The real claim, and the reason this file exists. Nothing here knows what
    // is inside the controller; the read is checked against the medium the host
    // installed, and so is the write — a model with a buffer and no medium
    // passes neither.
    let (m, store, mut d) = ready();

    // A read of one block, into memory the controller was never handed
    // directly: the address travelled in a PRP entry inside a command the
    // controller fetched for itself.
    const LBA_READ: u64 = 1234;
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            cdw10: LBA_READ as u32,
            cdw11: (LBA_READ >> 32) as u32,
            cdw12: 0, // NLB is zero-based: one block
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "the read failed");
    assert_eq!(cqe.sqid, 1, "the completion names its submission queue");
    assert_eq!(peek_bytes(&m, DATA, LBA), stamp(LBA_READ));
    // And that really is what is on the medium, rather than what the model
    // thought it should be.
    let mut on_medium = vec![0u8; LBA as usize];
    Medium::read_at(&*store, LBA_READ * LBA, &mut on_medium).expect("the medium reads");
    assert_eq!(peek_bytes(&m, DATA, LBA), on_medium);
    d.ack_io(&m);

    // A write of one block. The bytes are put in guest memory and the
    // controller fetches them.
    const LBA_WRITE: u64 = 9;
    let payload: Vec<u8> = (0..LBA as usize).map(|i| (i as u8) ^ 0xc3).collect();
    poke_bytes(&m, DATA + 0x1000, &payload);
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x01,
            nsid: 1,
            prp1: DATA + 0x1000,
            cdw10: LBA_WRITE as u32,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "the write failed");
    d.ack_io(&m);

    // **Against the medium**, not against the device's own buffer.
    let mut got = vec![0u8; LBA as usize];
    Medium::read_at(&*store, LBA_WRITE * LBA, &mut got).expect("the medium reads");
    assert_eq!(got, payload, "the write did not reach the medium");
    // And the block either side of it is untouched, which is what catches a
    // length computed in blocks and applied in bytes.
    let mut neighbour = vec![0u8; LBA as usize];
    Medium::read_at(&*store, (LBA_WRITE + 1) * LBA, &mut neighbour).expect("the medium reads");
    assert_eq!(neighbour, stamp(LBA_WRITE + 1));

    // Reading it back through the controller closes the loop.
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA + 0x2000,
            cdw10: LBA_WRITE as u32,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    assert_eq!(peek_bytes(&m, DATA + 0x2000, LBA), payload);
    d.ack_io(&m);

    // A block past the end of the namespace is LBA Out of Range (§4.6.1's
    // generic status 80h), not a silent short transfer.
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA + 0x3000,
            cdw10: BLOCKS as u32,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0x0080);
    d.ack_io(&m);
}

#[test]
fn a_transfer_of_many_pages_walks_a_prp_list_the_driver_built() {
    // One page is PRP1 alone, two are PRP1 and PRP2, and more than two is a
    // **list** in guest memory that the controller reads for itself (§4.3).
    // All three shapes are here, because the third is the one with a loop in
    // it.
    let (m, store, mut d) = ready();
    const PAGE: u64 = 4096;

    // Two pages: PRP2 is the second page, not a list.
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            prp2: DATA + PAGE,
            cdw10: 0,
            cdw12: (2 * PAGE / LBA - 1) as u32,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    d.ack_io(&m);
    for lba in 0..(2 * PAGE / LBA) {
        assert_eq!(
            peek_bytes(&m, DATA + lba * LBA, LBA),
            stamp(lba),
            "block {lba} of the two-page transfer"
        );
    }

    // Sixteen pages: PRP1, then a list of fifteen entries. The pages are
    // deliberately *not* in ascending order, because a controller that ignored
    // the list and just walked forward from PRP1 would pass an ordered one.
    const PAGES: u64 = 16;
    // Well clear of the PRP list itself: the pages a transfer lands in must not
    // overlap the list describing it, and a test that let them would be
    // checking the controller against a list its own read had overwritten.
    let base = 0x0040_0000;
    let page_of = |i: u64| base + (PAGES - 1 - i) * PAGE;
    for i in 1..PAGES {
        poke_bytes(&m, PRP_LIST + (i - 1) * 8, &page_of(i).to_le_bytes());
    }
    let blocks = PAGES * PAGE / LBA;
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: page_of(0),
            prp2: PRP_LIST,
            cdw10: 0,
            cdw12: (blocks - 1) as u32,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "the sixteen-page read failed");
    d.ack_io(&m);
    for i in 0..PAGES {
        let want: Vec<u8> = (0..PAGE / LBA)
            .flat_map(|b| stamp(i * (PAGE / LBA) + b))
            .collect();
        assert_eq!(
            peek_bytes(&m, page_of(i), PAGE),
            want,
            "page {i} of the list-walked transfer"
        );
    }

    // The same shape in the other direction: a scattered write, checked against
    // the medium.
    let payload: Vec<u8> = (0..(PAGES * PAGE) as usize)
        .map(|i| (i as u8).wrapping_mul(7) ^ 0x11)
        .collect();
    for i in 0..PAGES {
        poke_bytes(
            &m,
            page_of(i),
            &payload[(i * PAGE) as usize..((i + 1) * PAGE) as usize],
        );
    }
    let start = 1024u64;
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x01,
            nsid: 1,
            prp1: page_of(0),
            prp2: PRP_LIST,
            cdw10: start as u32,
            cdw12: (blocks - 1) as u32,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "the sixteen-page write failed");
    d.ack_io(&m);
    let mut got = vec![0u8; (PAGES * PAGE) as usize];
    Medium::read_at(&*store, start * LBA, &mut got).expect("the medium reads");
    assert_eq!(got, payload, "the scattered write did not reach the medium");

    // A PRP list entry that is not page aligned is PRP Offset Invalid (§4.3's
    // rule, and §4.6.1's generic status 13h), not a misaligned transfer.
    poke_bytes(&m, PRP_LIST, &(page_of(1) + 8).to_le_bytes());
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: page_of(0),
            prp2: PRP_LIST,
            cdw10: 0,
            cdw12: (blocks - 1) as u32,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0x0013);
    d.ack_io(&m);
}

#[test]
fn the_completion_interrupt_travels_the_wire_and_is_acknowledged() {
    // NVMe §7.5.1.1: the pin is a *level*, held down while a completion queue
    // holds an entry the host has not acknowledged. So this test watches the
    // 8259A's interrupt request register, which for a level-triggered input is
    // the line itself.
    let (m, _store, mut d) = ready();
    const IR5: u8 = 1 << 5;

    // Bringing the controller up ran admin commands, and this driver
    // acknowledged each one as it read it — so the line is low.
    assert_eq!(irr(&m) & IR5, 0, "nothing is outstanding yet");

    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            cdw10: 7,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    assert!(cqe.phase, "the first entry in a queue carries phase 1");
    assert_eq!(
        irr(&m) & IR5,
        IR5,
        "the completion did not reach the interrupt controller"
    );

    // A second completion while the first is unacknowledged keeps the line
    // down; it does not pulse.
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA + 0x1000,
            cdw10: 8,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    assert_eq!(irr(&m) & IR5, IR5);

    // Acknowledging one of the two leaves it down, and acknowledging both
    // releases it. That is the difference between a level and an edge, and it
    // is what an edge-triggered model would get wrong.
    d.ack_io(&m);
    assert_eq!(irr(&m) & IR5, IR5, "one completion is still outstanding");
    d.ack_io(&m);
    assert_eq!(irr(&m) & IR5, 0, "the line is released");

    // Masking the vector in `INTMS` (§3.1.3) also lowers it, and clearing the
    // mask brings it back — with the completion still sitting in the queue,
    // which is the whole point of a mask.
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            cdw10: 9,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    assert_eq!(irr(&m) & IR5, IR5);
    set_reg32(&m, REG_INTMS, 1);
    assert_eq!(irr(&m) & IR5, 0, "INTMS masks vector 0");
    set_reg32(&m, 0x10, 1); // INTMC
    assert_eq!(irr(&m) & IR5, IR5, "and INTMC brings it back");
    d.ack_io(&m);
    assert_eq!(irr(&m) & IR5, 0);

    // `COMMAND[10]`, Interrupt Disable, stops the function driving the pin at
    // all (PCI 3.0 §6.2.2) while the Status register's Interrupt Status bit
    // still reports the condition (§6.2.3).
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            cdw10: 10,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    config_write(&m, CFG_COMMAND, COMMAND_ON | 0x400);
    assert_eq!(irr(&m) & IR5, 0, "Interrupt Disable stops the pin");
    assert_eq!(
        config_read(&m, CFG_COMMAND) >> 16 & 0x08,
        0x08,
        "and the Status register still reports the condition"
    );
    config_write(&m, CFG_COMMAND, COMMAND_ON);
    assert_eq!(irr(&m) & IR5, IR5);
    d.ack_io(&m);
}

#[test]
fn a_controller_that_may_not_master_the_bus_fetches_nothing() {
    // Rev 2.1 §6.2.2: a function without Bus Master Enable generates no
    // accesses of its own, so it cannot fetch a command. A driver that forgot
    // sees nothing happen — which is what real hardware does, and is much
    // easier to debug than a controller that quietly works anyway.
    let (m, _store, mut d) = ready();

    // Memory space on, bus mastering off.
    config_write(&m, CFG_COMMAND, 0x0002);
    let before = read_cqe(&m, IOCQ, d.io_next);
    d.cid = d.cid.wrapping_add(1);
    let cmd = Sqe {
        opcode: 0x02,
        cid: d.cid,
        nsid: 1,
        prp1: DATA,
        cdw10: 3,
        cdw12: 0,
        ..Sqe::default()
    };
    poke_bytes(&m, IOSQ + u64::from(d.io_tail) * 64, &cmd.bytes());
    d.io_tail = (d.io_tail + 1) % IO_ENTRIES;
    set_reg32(&m, doorbell(1, false), d.io_tail);
    assert_eq!(
        read_cqe(&m, IOCQ, d.io_next),
        before,
        "a controller that may not master the bus posted a completion"
    );

    // Grant it, and the command that was already queued runs — the doorbell is
    // a tail pointer, not an event.
    config_write(&m, CFG_COMMAND, COMMAND_ON);
    set_reg32(&m, doorbell(1, false), d.io_tail);
    let cqe = read_cqe(&m, IOCQ, d.io_next);
    d.io_next = (d.io_next + 1) % IO_ENTRIES;
    assert_eq!(cqe.cid, cmd.cid);
    assert_eq!(cqe.status, 0);
    assert_eq!(peek_bytes(&m, DATA, LBA), stamp(3));
    d.ack_io(&m);
}

#[test]
fn a_debugger_may_look_but_may_not_touch() {
    // `CLAUDE.md`: a debugger read must not pop a FIFO, clear a status bit or
    // advance a pointer — and for a queue-based device that is the sharpest
    // case there is, because *submitting a command* is a register write.
    let (m, _store, mut d) = ready();
    let space = m.space("mem").expect("the memory space");

    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA,
            cdw10: 11,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);

    // Every register reads the same for a debugger as for the guest, twice
    // over, with a completion outstanding the whole time.
    for offset in [REG_CAP, REG_VS, REG_INTMS, REG_CC, REG_CSTS, REG_AQA] {
        let guest = space
            .read(BAR_BASE + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a register");
        let debug = space
            .read(BAR_BASE + offset, Width::U32, MemAttrs::DEBUG)
            .expect("a debug read of a register");
        let again = space
            .read(BAR_BASE + offset, Width::U32, MemAttrs::DEBUG)
            .expect("and again");
        assert_eq!(guest, debug, "a debug read of {offset:#x} disagreed");
        assert_eq!(
            debug, again,
            "a debug read of {offset:#x} had a side effect"
        );
    }

    // Every write is refused: `CC.EN` stops the controller, a submission
    // doorbell runs a command, and a completion doorbell acknowledges an
    // interrupt the guest has not seen.
    for offset in [REG_CC, REG_INTMS, doorbell(1, false), doorbell(1, true)] {
        assert!(
            space
                .write(BAR_BASE + offset, Width::U32, 1, MemAttrs::DEBUG)
                .is_err(),
            "a debug write to {offset:#x} was accepted"
        );
    }

    // And the completion is still outstanding, on a queue whose head has not
    // moved, with the interrupt still asserted.
    assert_eq!(read_cqe(&m, IOCQ, d.io_head).cid, cqe.cid);
    assert_eq!(irr(&m) & (1 << 5), 1 << 5);
    d.ack_io(&m);
}

#[test]
fn the_board_snapshots_and_restores_to_the_same_state_hash() {
    let (m, _store, mut d) = ready();
    let payload: Vec<u8> = (0..LBA as usize).map(|i| (i as u8) ^ 0x77).collect();
    poke_bytes(&m, DATA, &payload);
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x01,
            nsid: 1,
            prp1: DATA,
            cdw10: 40,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    d.ack_io(&m);

    let before = m.state_hash().expect("a hash");
    let bytes = m.save().expect("it snapshots");

    // A second board, brought to the same point the long way round, then
    // loaded: the hash is over the whole machine, so this covers the register
    // file, the queue descriptors, the namespace and the board's RAM at once.
    let (mut other, other_store) = board();
    other.load(&bytes).expect("it restores");
    assert_eq!(other.state_hash().expect("a hash"), before);

    // The restored controller is a working controller, and its namespace came
    // back with the block that was written into it.
    let mut got = vec![0u8; LBA as usize];
    Medium::read_at(&*other_store, 40 * LBA, &mut got).expect("the medium reads");
    assert_eq!(got, payload, "the snapshot did not carry the namespace");

    init_pic(&other);
    let mut d2 = Driver {
        admin_tail: d.admin_tail,
        admin_head: d.admin_head,
        io_tail: d.io_tail,
        io_next: d.io_next,
        io_head: d.io_head,
        cid: d.cid,
    };
    let cqe = d2.io(
        &other,
        Sqe {
            opcode: 0x02,
            nsid: 1,
            prp1: DATA + 0x4000,
            cdw10: 40,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0, "the restored controller still runs commands");
    assert_eq!(peek_bytes(&other, DATA + 0x4000, LBA), payload);
}

#[test]
fn a_shutdown_notification_flushes_and_says_so() {
    // §3.1.5's shutdown is a host saying "I am about to lose power". A
    // controller that acknowledged it without making its writes durable would
    // be lying about the one thing the notification is for.
    let (m, store, mut d) = ready();
    let payload = vec![0xa7u8; LBA as usize];
    poke_bytes(&m, DATA, &payload);
    let cqe = d.io(
        &m,
        Sqe {
            opcode: 0x01,
            nsid: 1,
            prp1: DATA,
            cdw10: 100,
            cdw12: 0,
            ..Sqe::default()
        },
    );
    assert_eq!(cqe.status, 0);
    d.ack_io(&m);

    // CC.SHN = 01b, a normal shutdown.
    let cc = reg32(&m, REG_CC);
    set_reg32(&m, REG_CC, cc | CC_SHN_NORMAL);
    // CSTS.SHST = 10b, shutdown processing complete (§3.1.6).
    assert_eq!(reg32(&m, REG_CSTS) >> 2 & 0x3, 0b10);

    let mut got = vec![0u8; LBA as usize];
    Medium::read_at(&*store, 100 * LBA, &mut got).expect("the medium reads");
    assert_eq!(got, payload);

    // And CC.EN 1 -> 0 is a Controller Reset: every queue goes and RDY follows.
    set_reg32(&m, REG_CC, 0);
    assert_eq!(reg32(&m, REG_CSTS) & CSTS_RDY, 0);
}
