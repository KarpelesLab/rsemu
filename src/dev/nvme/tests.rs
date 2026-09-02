//! The controller on its own, with the hazards a board-level test cannot
//! stage: a guest that points a PRP entry at the controller's own doorbells, a
//! PRP list that points at itself, and a snapshot chunk that describes a
//! controller no controller could have been.
//!
//! `tests/nvme_board.rs` is where the driver-shaped proof lives. This is where
//! the malicious-guest shaped one does.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, MemOps, RamStore, Region, RequesterId, UnassignedPolicy};
use crate::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;
use crate::core::wire::{Level, Resolve, Wire, WireIdAllocator, WireSource};
use crate::dev::medium::Medium;

/// Where guest RAM starts. Not zero, so that a null pointer in a command is a
/// bus fault the controller has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x40_0000;
/// Where the controller's own register block is mapped, so a test can aim a
/// PRP entry at it.
const REGS: u64 = 0x1000_0000;

/// Blocks on the test namespace, and their size.
const BLOCKS: u64 = 256;
const LBA: u64 = 512;

// Register offsets, as a driver knows them (NVMe 1.4 §3.1). Deliberately
// restated here rather than imported: a test that shared the module's own
// constants could not catch one of them moving.
const REG_CAP: u64 = 0x00;
const REG_CC: u64 = 0x14;
const REG_CSTS: u64 = 0x1c;
const REG_AQA: u64 = 0x24;
const REG_ASQ: u64 = 0x28;
const REG_ACQ: u64 = 0x30;

const ASQ: u64 = 0x0010_0000;
const ACQ: u64 = 0x0010_1000;
const IOSQ: u64 = 0x0010_2000;
const IOCQ: u64 = 0x0010_3000;
const DATA: u64 = 0x0020_0000;

const ADMIN_ENTRIES: u32 = 8;
const IO_ENTRIES: u32 = 8;

/// A controller with a stamped namespace, its own register block in an address
/// space it masters, and an interrupt output nothing is listening to.
struct Rig {
    ctrl: Arc<Controller>,
    space: Arc<AddressSpace>,
    wire: Arc<Wire>,
    irq: WireSource,
    store: Arc<RamStore>,
    tail: u32,
    head: u32,
    io_tail: u32,
    io_head: u32,
    cid: u16,
}

fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; LBA as usize];
    out[0] = lba as u8;
    out[1] = 0xa5;
    out
}

fn rig() -> Rig {
    let store = Arc::new(RamStore::new(BLOCKS * LBA));
    for lba in 0..BLOCKS {
        RamStore::write_at(&store, lba * LBA, &stamp(lba)).expect("the image fits");
    }
    let ns = Namespace::new(Arc::clone(&store) as Arc<dyn Medium>, 9, false).expect("512-byte");
    let ctrl = Arc::new(Controller::new(ns, Params::default()));

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
        topo.map(
            Region::io(
                "nvme.regs",
                REGISTER_LEN,
                Arc::clone(&ctrl) as Arc<dyn MemOps>,
            ),
            REGS,
        )
        .expect("the map fits");
    }
    ctrl.attach_space(&space, RequesterId(7));
    ctrl.set_master(true);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    let irq = WireSource::new(Arc::clone(&wire), id);
    ctrl.connect_irq(irq.clone());

    Rig {
        ctrl,
        space,
        wire,
        irq,
        store,
        tail: 0,
        head: 0,
        io_tail: 0,
        io_head: 0,
        cid: 0,
    }
}

impl Rig {
    fn reg(&self, offset: u64) -> u32 {
        self.space
            .read(REGS + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a register") as u32
    }

    fn set_reg(&self, offset: u64, value: u32) {
        self.space
            .write(
                REGS + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a register");
    }

    fn set_reg64(&self, offset: u64, value: u64) {
        self.set_reg(offset, value as u32);
        self.set_reg(offset + 4, (value >> 32) as u32);
    }

    fn poke(&self, addr: u64, bytes: &[u8]) {
        self.space
            .write_bytes(addr, bytes, MemAttrs::DEFAULT)
            .expect("mapped memory");
    }

    fn peek(&self, addr: u64, len: u64) -> Vec<u8> {
        let mut out = vec![0u8; len as usize];
        self.space
            .read_bytes(addr, &mut out, MemAttrs::DEFAULT)
            .expect("mapped memory");
        out
    }

    /// Bring the controller up with an admin queue pair and one I/O queue pair.
    fn enable(&mut self) {
        self.set_reg(REG_AQA, (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16));
        self.set_reg64(REG_ASQ, ASQ);
        self.set_reg64(REG_ACQ, ACQ);
        self.set_reg(REG_CC, 1 | (6 << 17) | (4 << 21));
        assert_eq!(self.reg(REG_CSTS) & 0x3, 1, "ready, and not fatal");
        let cqe = self.admin(0x05, 0, IOCQ, 0, 1 | ((IO_ENTRIES - 1) << 16), 0x0003, 0);
        assert_eq!(cqe.2, 0, "Create I/O Completion Queue");
        let cqe = self.admin(
            0x01,
            0,
            IOSQ,
            0,
            1 | ((IO_ENTRIES - 1) << 16),
            0x0001 | (1 << 16),
            0,
        );
        assert_eq!(cqe.2, 0, "Create I/O Submission Queue");
    }

    /// Submit one admin command and read its completion: `(dw0, cid, status)`.
    #[allow(clippy::too_many_arguments)]
    fn admin(
        &mut self,
        opcode: u8,
        nsid: u32,
        prp1: u64,
        prp2: u64,
        cdw10: u32,
        cdw11: u32,
        cdw12: u32,
    ) -> (u32, u16, u16) {
        self.cid = self.cid.wrapping_add(1);
        let mut sqe = [0u8; 64];
        let cdw0 = u32::from(opcode) | (u32::from(self.cid) << 16);
        sqe[0..4].copy_from_slice(&cdw0.to_le_bytes());
        sqe[4..8].copy_from_slice(&nsid.to_le_bytes());
        sqe[24..32].copy_from_slice(&prp1.to_le_bytes());
        sqe[32..40].copy_from_slice(&prp2.to_le_bytes());
        sqe[40..44].copy_from_slice(&cdw10.to_le_bytes());
        sqe[44..48].copy_from_slice(&cdw11.to_le_bytes());
        sqe[48..52].copy_from_slice(&cdw12.to_le_bytes());
        self.poke(ASQ + u64::from(self.tail) * 64, &sqe);
        self.tail = (self.tail + 1) % ADMIN_ENTRIES;
        self.set_reg(0x1000, self.tail);
        let at = ACQ + u64::from(self.head) * 16;
        let entry = self.peek(at, 16);
        self.head = (self.head + 1) % ADMIN_ENTRIES;
        self.set_reg(0x1004, self.head);
        let dw0 = u32::from_le_bytes([entry[0], entry[1], entry[2], entry[3]]);
        let dw3 = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]);
        (dw0, dw3 as u16, (dw3 >> 17) as u16)
    }

    /// Submit one I/O command on queue 1, read its completion, and acknowledge
    /// it — which is the whole cycle a driver runs.
    #[allow(clippy::too_many_arguments)]
    fn io(&mut self, opcode: u8, nsid: u32, prp1: u64, prp2: u64, cdw10: u32, cdw12: u32) -> u16 {
        self.cid = self.cid.wrapping_add(1);
        let mut sqe = [0u8; 64];
        let cdw0 = u32::from(opcode) | (u32::from(self.cid) << 16);
        sqe[0..4].copy_from_slice(&cdw0.to_le_bytes());
        sqe[4..8].copy_from_slice(&nsid.to_le_bytes());
        sqe[24..32].copy_from_slice(&prp1.to_le_bytes());
        sqe[32..40].copy_from_slice(&prp2.to_le_bytes());
        sqe[40..44].copy_from_slice(&cdw10.to_le_bytes());
        sqe[48..52].copy_from_slice(&cdw12.to_le_bytes());
        self.poke(IOSQ + u64::from(self.io_tail) * 64, &sqe);
        self.io_tail = (self.io_tail + 1) % IO_ENTRIES;
        self.set_reg(0x1008, self.io_tail);
        let entry = self.peek(IOCQ + u64::from(self.io_head) * 16, 16);
        let dw3 = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]);
        assert_eq!(
            dw3 as u16, self.cid,
            "the completion is for another command"
        );
        self.io_head = (self.io_head + 1) % IO_ENTRIES;
        self.set_reg(0x100c, self.io_head);
        (dw3 >> 17) as u16
    }
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn a_namespace_holds_a_whole_number_of_blocks() {
    let store: Arc<dyn Medium> = Arc::new(RamStore::new(1000));
    let e = Namespace::new(Arc::clone(&store), 9, false)
        .expect_err("1000 is not a multiple of 512")
        .to_string();
    assert!(e.contains("whole number"), "{e}");

    // And a block size no LBA format can express.
    let store: Arc<dyn Medium> = Arc::new(RamStore::new(8192));
    assert!(Namespace::new(Arc::clone(&store), 13, false).is_err());
    let ns = Namespace::new(store, 12, false).expect("4096-byte blocks");
    assert_eq!(ns.blocks(), 2);
    assert_eq!(ns.lba_bytes(), 4096);
}

#[test]
fn a_read_only_medium_makes_a_read_only_namespace() {
    // The medium's own answer wins over the property, exactly as it does for a
    // drive: telling a guest it may write and then failing every write is worse
    // than an honest read-only namespace.
    let ns = Namespace::new(Arc::new(RamStore::new(LBA)) as Arc<dyn Medium>, 9, true)
        .expect("one block");
    assert!(ns.is_read_only());
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

#[test]
fn the_registers_answer_only_at_the_widths_the_specification_names() {
    let r = rig();
    // §3.1: 32- or 64-bit, naturally aligned. A byte access is not a register
    // access and answering one would be inventing behaviour.
    assert!(
        r.space
            .read(REGS + REG_CAP, Width::U8, MemAttrs::DEFAULT)
            .is_err()
    );
    assert!(
        r.space
            .read(REGS + 2, Width::U32, MemAttrs::DEFAULT)
            .is_err(),
        "an unaligned dword"
    );
    // A 64-bit read of CAP is the same two dwords a 32-bit driver assembles.
    let wide = r
        .space
        .read(REGS + REG_CAP, Width::U64, MemAttrs::DEFAULT)
        .expect("a qword read of CAP");
    assert_eq!(
        wide,
        u64::from(r.reg(REG_CAP)) | (u64::from(r.reg(REG_CAP + 4)) << 32)
    );
}

#[test]
fn the_admin_queue_registers_are_ignored_while_the_controller_runs() {
    // §3.1.8: they are written while the controller is disabled. A write with
    // CC.EN set is undefined, and ignoring it is the answer that cannot corrupt
    // a controller mid-flight.
    let mut r = rig();
    r.enable();
    r.set_reg64(REG_ASQ, 0xdead_0000);
    assert_eq!(
        r.reg(REG_ASQ),
        ASQ as u32,
        "ASQ moved under a running controller"
    );
}

// ---------------------------------------------------------------------------
// the guest is not trusted
// ---------------------------------------------------------------------------

#[test]
fn a_prp_entry_aimed_at_the_doorbells_does_not_recurse() {
    // The hazard this device's shape creates: guest memory and this
    // controller's own register block are the same address space, so a driver
    // can make a data transfer land on `SQ1TDBL` and re-enter the write handler
    // from inside itself. The engine is iterative rather than recursive, so
    // this returns rather than growing the stack until it dies.
    let mut r = rig();
    r.enable();
    // A one-block read whose destination *is* the register block. The transfer
    // is 512 bytes, which is not a width the register block accepts, so it
    // faults — and the interesting part is that it faults rather than looping.
    let status = r.io(0x02, 1, REGS + 0x1000, 0, 0, 0);
    assert_eq!(status, 0x0004, "generic status 04h, Data Transfer Error");

    // And an eight-byte PRP *list* read aimed at the doorbells, which the
    // register block will happily answer — the controller must survive
    // whatever nonsense comes back.
    let status = r.io(0x02, 1, DATA, REGS + 0x1000, 0, 15);
    assert!(
        status != 0,
        "a list read out of the register block succeeded"
    );

    // The controller is still alive.
    assert_eq!(r.io(0x02, 1, DATA, 0, 5, 0), 0);
    assert_eq!(r.peek(DATA, LBA), stamp(5));
}

#[test]
fn a_prp_list_that_points_at_itself_terminates() {
    // §4.3 lets a list's last entry point at another list, so a guest can close
    // a ring. Every walk is bounded, and reaching the bound is a Data Transfer
    // Error rather than a hang.
    let mut r = rig();
    r.enable();
    let list = DATA + 0x1000;
    // Fill the list with valid page pointers, then make the last entry point
    // back at the list itself.
    for i in 0..512u64 {
        let entry = if i == 511 {
            list
        } else {
            DATA + 0x2000 + i * 4096
        };
        r.poke(list + i * 8, &entry.to_le_bytes());
    }
    // Ask for more than one list page can describe. A 4 KiB list holds 512
    // entries, whose last one is a pointer to the next list rather than a page
    // of data, so 2 MiB is where the chain starts — and this chain is a ring.
    let status = r.io(0x02, 1, DATA, list, 0, (3 * 1024 * 1024 / LBA - 1) as u32);
    assert_ne!(status, 0, "a ring of PRP lists was walked to completion");

    // Still alive.
    assert_eq!(r.io(0x02, 1, DATA, 0, 3, 0), 0);
}

#[test]
fn an_invalid_doorbell_value_is_fatal_rather_than_ignored() {
    // §4.1: a doorbell value outside the queue is a fatal condition, and
    // CSTS.CFS is how the controller says so. A model that masked it would let
    // a broken driver walk off the end of its own queue.
    let mut r = rig();
    r.enable();
    r.set_reg(0x1008, IO_ENTRIES);
    assert_eq!(r.reg(REG_CSTS) & 0x2, 0x2, "CSTS.CFS");
    // And a fatal controller runs nothing more.
    r.poke(IOSQ, &[0u8; 64]);
    r.set_reg(0x1008, 1);
    assert_eq!(
        u32::from_le_bytes([
            r.peek(IOCQ + 12, 4)[0],
            r.peek(IOCQ + 12, 4)[1],
            r.peek(IOCQ + 12, 4)[2],
            r.peek(IOCQ + 12, 4)[3],
        ]),
        0,
        "a fatal controller posted a completion"
    );
}

#[test]
fn a_command_on_a_queue_whose_completion_queue_is_full_waits() {
    // §4.1's back pressure. A completion queue with no room is not an error:
    // the command stays where it is until the host's head doorbell makes room.
    // The alternative — overwriting an entry the host has not read — loses a
    // completion silently.
    let mut r = rig();
    r.enable();
    // Seven commands fill an eight-entry completion queue: one slot is always
    // left free, which is how full is distinguished from empty (§4.1). Seven is
    // also all an eight-entry submission queue can hold at once, for the same
    // reason.
    for i in 0..(IO_ENTRIES - 1) {
        let mut sqe = [0u8; 64];
        let cdw0 = 0x02u32 | (u32::from(i as u16 + 100) << 16);
        sqe[0..4].copy_from_slice(&cdw0.to_le_bytes());
        sqe[4..8].copy_from_slice(&1u32.to_le_bytes());
        sqe[24..32].copy_from_slice(&(DATA + u64::from(i) * 512).to_le_bytes());
        sqe[40..44].copy_from_slice(&i.to_le_bytes());
        r.poke(IOSQ + u64::from(i) * 64, &sqe);
    }
    r.set_reg(0x1008, IO_ENTRIES - 1);
    // An eighth command, which cannot complete until the host reads one — the
    // submission queue has room for it now that the first seven were consumed.
    let mut sqe = [0u8; 64];
    let cdw0 = 0x02u32 | (0xbeefu32 << 16);
    sqe[0..4].copy_from_slice(&cdw0.to_le_bytes());
    sqe[4..8].copy_from_slice(&1u32.to_le_bytes());
    sqe[24..32].copy_from_slice(&DATA.to_le_bytes());
    r.poke(IOSQ + u64::from(IO_ENTRIES - 1) * 64, &sqe);
    r.set_reg(0x1008, 0);

    // Seven completions are there and the eighth is not.
    for i in 0..(IO_ENTRIES - 1) {
        let entry = r.peek(IOCQ + u64::from(i) * 16 + 12, 4);
        let dw3 = u32::from_le_bytes([entry[0], entry[1], entry[2], entry[3]]);
        assert_eq!(dw3 as u16, i as u16 + 100);
    }
    let entry = r.peek(IOCQ + u64::from(IO_ENTRIES - 1) * 16 + 12, 4);
    assert_eq!(
        u32::from_le_bytes([entry[0], entry[1], entry[2], entry[3]]),
        0,
        "the eighth completion overwrote the host's queue"
    );

    // Making room releases it, without another doorbell: the controller comes
    // back to the queue when the head moves.
    r.set_reg(0x100c, 4);
    let entry = r.peek(IOCQ + u64::from(IO_ENTRIES - 1) * 16 + 12, 4);
    let dw3 = u32::from_le_bytes([entry[0], entry[1], entry[2], entry[3]]);
    assert_eq!(dw3 as u16, 0xbeef, "the held command never ran");
}

// ---------------------------------------------------------------------------
// the command set
// ---------------------------------------------------------------------------

#[test]
fn the_queue_commands_refuse_what_the_specification_says_they_refuse() {
    let mut r = rig();
    r.set_reg(REG_AQA, (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16));
    r.set_reg64(REG_ASQ, ASQ);
    r.set_reg64(REG_ACQ, ACQ);
    r.set_reg(REG_CC, 1 | (6 << 17) | (4 << 21));

    // A submission queue naming a completion queue that is not there (§5.4).
    let (_, _, status) = r.admin(0x01, 0, IOSQ, 0, 1 | (7 << 16), 0x0001 | (1 << 16), 0);
    assert_eq!(
        status, 0x0100,
        "command specific 00h, Completion Queue Invalid"
    );
    // Queue identifier zero is the admin queue and cannot be created (§5.3).
    let (_, _, status) = r.admin(0x05, 0, IOCQ, 0, 7 << 16, 0x0003, 0);
    assert_eq!(status, 0x0101, "Invalid Queue Identifier");
    // A one-entry queue (§3.1.8's minimum is two).
    let (_, _, status) = r.admin(0x05, 0, IOCQ, 0, 1, 0x0003, 0);
    assert_eq!(status, 0x0102, "Invalid Queue Size");
    // Not physically contiguous, which CAP.CQR says is not on offer.
    let (_, _, status) = r.admin(0x05, 0, IOCQ, 0, 1 | (7 << 16), 0x0002, 0);
    assert_eq!(status, 0x0002, "Invalid Field in Command");
    // An interrupt vector past the mask register.
    let (_, _, status) = r.admin(0x05, 0, IOCQ, 0, 1 | (7 << 16), 0x0003 | (32 << 16), 0);
    assert_eq!(status, 0x0108, "Invalid Interrupt Vector");

    // Now a real pair, and a completion queue that cannot be deleted while a
    // submission queue still points at it (§5.5).
    let (_, _, status) = r.admin(0x05, 0, IOCQ, 0, 1 | (7 << 16), 0x0003, 0);
    assert_eq!(status, 0);
    let (_, _, status) = r.admin(0x01, 0, IOSQ, 0, 1 | (7 << 16), 0x0001 | (1 << 16), 0);
    assert_eq!(status, 0);
    let (_, _, status) = r.admin(0x04, 0, 0, 0, 1, 0, 0);
    assert_eq!(status, 0x010c, "Invalid Queue Deletion");
    let (_, _, status) = r.admin(0x00, 0, 0, 0, 1, 0, 0);
    assert_eq!(status, 0, "the submission queue deletes");
    let (_, _, status) = r.admin(0x04, 0, 0, 0, 1, 0, 0);
    assert_eq!(status, 0, "and then so does the completion queue");
}

#[test]
fn set_features_answers_with_the_queues_it_allocated() {
    // §5.21.1.7: the host asks and the controller answers with what it
    // allocated, which may be fewer. Both fields are zero-based, which is the
    // off-by-one every driver writer meets first.
    let mut r = rig();
    r.enable();
    let (dw0, _, status) = r.admin(0x09, 0, 0, 0, 0x07, 0xffff_ffff, 0);
    assert_eq!(status, 0);
    assert_eq!(dw0 & 0xffff, u32::from(Params::default().io_queues) - 1);
    assert_eq!(dw0 >> 16, u32::from(Params::default().io_queues) - 1);
    // And Get Features agrees with Set Features.
    let (again, _, status) = r.admin(0x0a, 0, 0, 0, 0x07, 0, 0);
    assert_eq!((status, again), (0, dw0));
    // A feature this controller does not have is an Invalid Field rather than a
    // plausible answer.
    let (_, _, status) = r.admin(0x09, 0, 0, 0, 0x1e, 0, 0);
    assert_eq!(status, 0x0002);
}

#[test]
fn an_asynchronous_event_request_is_held_rather_than_completed() {
    // §5.2: the command occupies a slot until an event happens. Nothing here
    // generates one, so nothing completes it — which is what real silicon does,
    // and is more honest than inventing an event or refusing the command.
    let mut r = rig();
    r.enable();
    let before = r.peek(ACQ + u64::from(r.head) * 16, 16);
    r.cid = r.cid.wrapping_add(1);
    let mut sqe = [0u8; 64];
    let cdw0 = 0x0cu32 | (u32::from(r.cid) << 16);
    sqe[0..4].copy_from_slice(&cdw0.to_le_bytes());
    r.poke(ASQ + u64::from(r.tail) * 64, &sqe);
    r.tail = (r.tail + 1) % ADMIN_ENTRIES;
    r.set_reg(0x1000, r.tail);
    assert_eq!(
        r.peek(ACQ + u64::from(r.head) * 16, 16),
        before,
        "an asynchronous event request completed with nothing to report"
    );

    // A second one exceeds the limit `Identify Controller`'s AERL reports, and
    // that *does* complete, with the specification's own status code.
    let (_, _, status) = r.admin(0x0c, 0, 0, 0, 0, 0, 0);
    assert_eq!(status, 0x0105, "Asynchronous Event Request Limit Exceeded");
}

#[test]
fn write_zeroes_moves_no_data_and_still_reaches_the_medium() {
    // §6.16: no data transfer at all, which is why the transfer size limit does
    // not apply to it and why it is worth having.
    let mut r = rig();
    r.enable();
    assert_eq!(r.io(0x08, 1, 0, 0, 10, 3), 0, "Write Zeroes");
    let mut got = vec![0u8; (4 * LBA) as usize];
    Medium::read_at(&*r.store, 10 * LBA, &mut got).expect("the medium reads");
    assert!(got.iter().all(|b| *b == 0), "the blocks are not zeroed");
    // And the block after them is untouched.
    let mut next = vec![0u8; LBA as usize];
    Medium::read_at(&*r.store, 14 * LBA, &mut next).expect("the medium reads");
    assert_eq!(next, stamp(14));
}

#[test]
fn a_flush_reaches_the_medium_and_an_unknown_opcode_is_refused() {
    let mut r = rig();
    r.enable();
    assert_eq!(r.io(0x00, 1, 0, 0, 0, 0), 0, "Flush");
    // §6: FFFFFFFFh means every namespace, which Flush accepts.
    assert_eq!(r.io(0x00, 0xffff_ffff, 0, 0, 0, 0), 0);
    // And a data command does not.
    assert_eq!(r.io(0x02, 0xffff_ffff, DATA, 0, 0, 0), 0x000b);
    // An opcode this command set does not have.
    assert_eq!(r.io(0x7f, 1, 0, 0, 0, 0), 0x0001, "Invalid Command Opcode");
}

#[test]
fn a_read_only_namespace_refuses_writes_and_says_so_in_identify() {
    let store = Arc::new(RamStore::new(BLOCKS * LBA));
    let ns = Namespace::new(Arc::clone(&store) as Arc<dyn Medium>, 9, true).expect("512-byte");
    let ctrl = Arc::new(Controller::new(ns, Params::default()));
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
        topo.map(
            Region::io(
                "nvme.regs",
                REGISTER_LEN,
                Arc::clone(&ctrl) as Arc<dyn MemOps>,
            ),
            REGS,
        )
        .expect("the map fits");
    }
    ctrl.attach_space(&space, RequesterId(7));
    ctrl.set_master(true);
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    let irq = WireSource::new(Arc::clone(&wire), id);
    ctrl.connect_irq(irq.clone());
    let mut r = Rig {
        ctrl,
        space,
        wire,
        irq,
        store,
        tail: 0,
        head: 0,
        io_tail: 0,
        io_head: 0,
        cid: 0,
    };
    r.enable();

    // §4.6.1's media and data integrity errors are where a write that did not
    // reach the medium belongs.
    assert_eq!(r.io(0x01, 1, DATA, 0, 0, 0), 0x0280, "Write Fault");
    assert_eq!(r.io(0x08, 1, 0, 0, 0, 0), 0x0280, "and Write Zeroes too");
    // NSATTR bit 0 says so in advance, which is what a driver reads.
    let (_, _, status) = r.admin(0x06, 1, DATA, 0, 0x00, 0, 0);
    assert_eq!(status, 0);
    assert_eq!(r.peek(DATA + 99, 1)[0] & 1, 1, "NSATTR: write protected");
}

// ---------------------------------------------------------------------------
// the interrupt
// ---------------------------------------------------------------------------

#[test]
fn the_output_is_a_level_the_host_releases() {
    let mut r = rig();
    r.enable();
    // Bringing it up acknowledged every admin completion, so the line is low.
    assert_eq!(r.wire.resolve(Resolve::Or), Level::Low);
    assert_eq!(r.irq.level(), Level::Low);

    // A completion the host has not acknowledged holds it down.
    let mut sqe = [0u8; 64];
    sqe[0..4].copy_from_slice(&(0x02u32 | (1 << 16)).to_le_bytes());
    sqe[4..8].copy_from_slice(&1u32.to_le_bytes());
    sqe[24..32].copy_from_slice(&DATA.to_le_bytes());
    r.poke(IOSQ, &sqe);
    r.set_reg(0x1008, 1);
    assert_eq!(r.wire.resolve(Resolve::Or), Level::High);
    assert!(r.ctrl.interrupt_pending());

    // Interrupt Disable stops the pin without changing the condition.
    r.ctrl.set_intx_disabled(true);
    assert_eq!(r.wire.resolve(Resolve::Or), Level::Low);
    assert!(r.ctrl.interrupt_pending(), "the condition is still there");
    r.ctrl.set_intx_disabled(false);
    assert_eq!(r.wire.resolve(Resolve::Or), Level::High);

    // The head doorbell releases it.
    r.set_reg(0x100c, 1);
    assert_eq!(r.wire.resolve(Resolve::Or), Level::Low);
    assert!(!r.ctrl.interrupt_pending());
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

fn snapshot(ctrl: &Controller) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("nvme", CLASS_NAME).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("nvme", CLASS_NAME, STATE_VERSION).expect("a chunk");
        ctrl.save(&mut chunk).expect("it saves");
    }
    w.to_vec().expect("it encodes")
}

#[test]
fn the_controller_round_trips_through_a_snapshot() {
    let mut r = rig();
    r.enable();
    assert_eq!(r.io(0x02, 1, DATA, 0, 7, 0), 0);
    let bytes = snapshot(&r.ctrl);

    let fresh = rig();
    let reader = StateReader::new(&bytes).expect("we just wrote it");
    let chunk = reader
        .load("nvme", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("it is in there");
    fresh.ctrl.load(&mut chunk.reader()).expect("it loads");
    assert_eq!(
        snapshot(&fresh.ctrl),
        bytes,
        "the controller did not round trip"
    );

    // And the restored controller is a working one: its queues came back, so a
    // command submitted to the queue the *other* controller created runs here.
    let mut restored = Rig {
        tail: r.tail,
        head: r.head,
        io_tail: r.io_tail,
        io_head: r.io_head,
        cid: r.cid,
        ..fresh
    };
    restored.ctrl.set_master(true);
    assert_eq!(restored.io(0x02, 1, DATA, 0, 9, 0), 0);
    assert_eq!(restored.peek(DATA, LBA), stamp(9));
}

#[test]
fn a_snapshot_describing_an_impossible_controller_is_refused() {
    // The loader is a parser on untrusted bytes (`CLAUDE.md`, testing). A queue
    // with zero entries would divide by zero the first time a doorbell moved
    // its head, so it is rejected rather than trusted.
    let r = rig();
    let mut chunk = Vec::new();
    for value in [1u32, 1, 0, 0x0007_0007] {
        chunk.extend_from_slice(&value.to_le_bytes());
    }
    chunk.extend_from_slice(&ASQ.to_le_bytes());
    chunk.extend_from_slice(&ACQ.to_le_bytes());
    chunk.extend_from_slice(&0u32.to_le_bytes());
    // One submission queue, with a zero size.
    chunk.push(1);
    chunk.extend_from_slice(&ASQ.to_le_bytes());
    chunk.extend_from_slice(&0u32.to_le_bytes());
    chunk.extend_from_slice(&0u32.to_le_bytes());
    chunk.extend_from_slice(&0u32.to_le_bytes());
    chunk.extend_from_slice(&0u16.to_le_bytes());
    let e = r
        .ctrl
        .load(&mut ChunkReader::new(&chunk))
        .expect_err("a zero-entry queue")
        .to_string();
    assert!(e.contains("submission queue"), "{e}");

    // And an arbitrary tail is refused rather than panicking.
    for len in 0..64 {
        let bytes: Vec<u8> = (0..len).map(|i| (i as u8).wrapping_mul(37)).collect();
        let _ = r.ctrl.load(&mut ChunkReader::new(&bytes));
    }
}

#[test]
fn arbitrary_traffic_terminates() {
    // The property `fuzz/fuzz_targets/nvme_mmio.rs` exists for, in a form that
    // runs under `cargo test` on every commit: whatever a guest writes into the
    // registers and into the memory behind them, a doorbell write **returns**.
    // A deterministic xorshift stands in for the fuzzer's mutations, so a
    // failure here is reproducible without a corpus.
    let r = rig();
    let mut seed = 0x243f_6a88_85a3_08d3u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for _ in 0..20_000 {
        let word = next();
        let value = word as u32;
        match (word >> 32) & 0x3 {
            // A register or a doorbell.
            0 | 1 => {
                let sel = (word >> 40) & 0x1f;
                let offset = if sel < 0x10 {
                    sel * 4
                } else {
                    0x1000 + (sel - 0x10) * 4
                };
                let _ = r.space.write(
                    REGS + offset,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
            }
            // A dword of guest memory, which is where the queues and the PRP
            // lists are.
            2 => {
                let addr = RAM_BASE + ((word >> 40) & 0x3f_ffff) * 4 % (RAM_LEN - 4);
                let _ = r
                    .space
                    .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT);
            }
            // And a read, which must never differ for a debugger.
            _ => {
                let offset = ((word >> 40) & 0xf) * 4;
                let live = r.space.read(REGS + offset, Width::U32, MemAttrs::DEFAULT);
                let dbg = r.space.read(REGS + offset, Width::U32, MemAttrs::DEBUG);
                assert_eq!(live, dbg, "a debug read of {offset:#x} disagreed");
            }
        }
    }
}
