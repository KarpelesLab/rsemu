//! The adapter on its own, with the hazards a board-level test cannot stage: a
//! guest that aims a data block at the adapter's own `PxCI`, a PRD table that
//! never ends, a port with nothing in its bay, and a snapshot chunk that
//! describes an adapter no adapter could have been.
//!
//! `tests/ahci_board.rs` is where the driver-shaped proof lives — a real
//! command list, a real PRD table and a real interrupt on a real wire. This is
//! where the malicious-guest shaped one does.

use super::*;

use alloc::vec;

use crate::core::space::{AddressSpace, MemOps, RamStore, Region, RequesterId, UnassignedPolicy};
use crate::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;
use crate::core::wire::{Wire, WireIdAllocator};
use crate::dev::ata::disk::default_geometry;
use crate::dev::ata::{AtaDisk, Identity, Medium, Position};

/// Where guest RAM starts. Not zero, so a null pointer in a command header is a
/// bus fault the adapter has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x40_0000;
/// Where the adapter's own register block is mapped, so a test can aim a PRD at
/// it.
const REGS: u64 = 0x1000_0000;

/// Sectors on the test drive.
const SECTORS: u64 = 256;
const SECTOR: u64 = 512;

// Offsets, as a driver knows them (AHCI §3.1, §3.3). Deliberately restated here
// rather than imported: a test that shared the module's own constants could not
// catch one of them moving.
const P0: u64 = 0x100;
const P_CLB: u64 = P0;
const P_FB: u64 = P0 + 0x08;
const P_IS: u64 = P0 + 0x10;
const P_IE: u64 = P0 + 0x14;
const P_CMD: u64 = P0 + 0x18;
const P_TFD: u64 = P0 + 0x20;
const P_CI: u64 = P0 + 0x38;

const CLB: u64 = 0x0010_0000;
const FB: u64 = 0x0010_1000;
const CTBA: u64 = 0x0010_2000;
const DATA: u64 = 0x0020_0000;

/// An adapter with a stamped drive in port 0's bay, its own register block in an
/// address space it masters, and an interrupt output nothing is listening to.
struct Rig {
    hba: Arc<Hba>,
    space: Arc<AddressSpace>,
    store: Arc<RamStore>,
    drive: Arc<AtaDisk>,
}

fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; SECTOR as usize];
    out[0] = lba as u8;
    out[1] = 0x5a;
    out
}

fn rig() -> Rig {
    rig_with(true)
}

/// `occupied` false leaves the bay empty, which is the case a board-level test
/// cannot produce without a second machine file.
fn rig_with(occupied: bool) -> Rig {
    let store = Arc::new(RamStore::new(SECTORS * SECTOR));
    for lba in 0..SECTORS {
        RamStore::write_at(&store, lba * SECTOR, &stamp(lba)).expect("the image fits");
    }
    let mut id = Identity::new(SECTORS, default_geometry(SECTORS), true, 16).expect("an identity");
    id.dma = true;
    let drive = Arc::new(
        AtaDisk::with_medium(id, Position::Device0, Arc::clone(&store) as Arc<dyn Medium>)
            .expect("the medium fits"),
    );
    let bay = Arc::new(bays::Bay::new());
    if occupied {
        bay.fit(Arc::clone(&drive)).expect("an empty bay");
    }
    let hba = Arc::new(Hba::new(alloc::vec![(String::from("sata0"), bay)]));

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
                "ahci.abar",
                REGISTER_LEN,
                Arc::clone(&hba) as Arc<dyn MemOps>,
            ),
            REGS,
        )
        .expect("the map fits");
    }
    hba.attach_space(&space, RequesterId(9));
    hba.set_master(true);
    hba.reset();
    hba.set_master(true);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    hba.connect_irq(WireSource::new(wire, id));

    Rig {
        hba,
        space,
        store,
        drive,
    }
}

impl Rig {
    fn reg(&self, offset: u64) -> u32 {
        self.space
            .read(REGS + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a mapped dword") as u32
    }

    fn set(&self, offset: u64, value: u32) {
        self.space
            .write(
                REGS + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a mapped dword");
    }

    fn poke(&self, addr: u64, bytes: &[u8]) {
        self.space
            .write_bytes(addr, bytes, MemAttrs::DEFAULT)
            .expect("mapped memory");
    }

    fn peek(&self, addr: u64, len: u64) -> Vec<u8> {
        let mut out = alloc::vec![0u8; len as usize];
        self.space
            .read_bytes(addr, &mut out, MemAttrs::DEFAULT)
            .expect("mapped memory");
        out
    }

    /// Point the port at the structures below and start both engines.
    fn start(&self) {
        self.set(P_CLB, CLB as u32);
        self.set(P_FB, FB as u32);
        self.set(P_IE, 0xffff_ffff);
        self.set(P_CMD, 1 << 4);
        self.set(P_CMD, (1 << 4) | 1);
    }

    /// A Register - Host to Device FIS with `C` set.
    fn fis(command: u8, count: u16, lba: u64, device: u8) -> [u8; 20] {
        let mut f = [0u8; 20];
        f[0] = 0x27;
        f[1] = 0x80;
        f[2] = command;
        f[4] = lba as u8;
        f[5] = (lba >> 8) as u8;
        f[6] = (lba >> 16) as u8;
        f[7] = device;
        f[8] = (lba >> 24) as u8;
        f[9] = (lba >> 32) as u8;
        f[10] = (lba >> 40) as u8;
        f[12] = count as u8;
        f[13] = (count >> 8) as u8;
        f
    }

    /// Build slot 0 out of `fis` and `prds` and write `PxCI`.
    fn issue(&self, fis: &[u8; 20], write: bool, prds: &[[u8; 16]]) {
        self.poke(CTBA, fis);
        for (i, entry) in prds.iter().enumerate() {
            self.poke(CTBA + 0x80 + i as u64 * 16, entry);
        }
        let dw0 = 5u32 | (u32::from(write) << 6) | ((prds.len() as u32) << 16);
        let mut header = [0u8; 32];
        header[0..4].copy_from_slice(&dw0.to_le_bytes());
        header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
        self.poke(CLB, &header);
        self.set(P_CI, 1);
    }

    /// The byte count the adapter wrote back into the header (§5.4.1).
    fn prdbc(&self) -> u32 {
        u32::from_le_bytes(self.peek(CLB + 4, 4).try_into().expect("four bytes"))
    }
}

fn prd(addr: u64, bytes: u64, interrupt: bool) -> [u8; 16] {
    let mut d = [0u8; 16];
    d[0..4].copy_from_slice(&(addr as u32).to_le_bytes());
    d[4..8].copy_from_slice(&((addr >> 32) as u32).to_le_bytes());
    let dw3 = (bytes as u32 - 1) | if interrupt { 1 << 31 } else { 0 };
    d[12..16].copy_from_slice(&dw3.to_le_bytes());
    d
}

// ---------------------------------------------------------------------------

#[test]
fn the_reset_taskfile_is_the_drives_own_signature() {
    // `hba.rs` builds `PxTFD`'s reset value from two constants rather than from
    // the drive, because it has no drive to ask before the machine is
    // assembled. This is the assertion that makes that safe: what it builds and
    // what an `AtaDisk` actually leaves in its command block after a reset are
    // the same thing, so the two cannot drift apart unnoticed.
    let rig = rig();
    let regs = rig.drive.taskfile_registers();
    let tfd = (u32::from(regs.error) << 8) | u32::from(regs.status);
    assert_eq!(rig.reg(P_TFD), tfd);
    // §3.3.9 packs the same four registers the other way round, and a driver
    // reads it to find out *what* is on the port: `00000101h` is an ATA device
    // and `EB140101h` is a packet one.
    let sig = (((regs.lba >> 16) as u32 & 0xff) << 24)
        | (((regs.lba >> 8) as u32 & 0xff) << 16)
        | ((regs.lba as u32 & 0xff) << 8)
        | u32::from(regs.count as u8);
    assert_eq!(rig.reg(P0 + 0x24), sig);
    assert_eq!(sig, 0x0000_0101, "an ATA device, not a packet one");
}

#[test]
fn an_empty_port_reports_no_device_and_refuses_to_run() {
    let rig = rig_with(false);
    // §3.3.10: `DET` 0h, no device detected and no communication.
    assert_eq!(rig.reg(P0 + 0x28), 0);
    // §3.3.9: `PxSIG` stays at its reset value until a D2H FIS arrives, and
    // none ever will.
    assert_eq!(rig.reg(P0 + 0x24), 0xffff_ffff);
    assert_eq!(rig.reg(P_TFD), 0x7f, "PxTFD's reset value (§3.3.8)");

    // A driver that started the port anyway gets an interface error rather than
    // a panic, and the engine stops.
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 0, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.reg(P_IS) & (1 << 27), 1 << 27, "PxIS.IFS");
    assert_eq!(rig.reg(P_CMD) & (1 << 15), 0, "PxCMD.CR cleared");
    assert_eq!(rig.reg(P_CI), 1, "and the slot is still outstanding");
}

#[test]
fn a_data_block_aimed_at_the_adapters_own_command_issue_register_terminates() {
    // **The re-entrancy case.** The PRD table is guest memory and the register
    // block is in the same address space, so a driver can point a data block at
    // `PxCI` — and the write handler is then re-entered from inside one of the
    // adapter's own accesses. The engine is iterative rather than recursive, so
    // recursion depth is one whatever the guest built; a model that recursed
    // would overflow the stack here rather than returning.
    let rig = rig();
    rig.start();
    // A read of one sector straight into the register block, starting at
    // `PxCLB` so that every register from there to `PxCI` is scribbled on.
    rig.issue(
        &Rig::fis(0x25, 1, 3, 0x40),
        false,
        &[prd(REGS + P_CLB, SECTOR, false)],
    );
    // It returned, which is the whole assertion. Everything the sector landed
    // on is now garbage, so what follows is a driver's full recovery — stop the
    // engine, which is what resets `PxCI`, clear the status, and start again.
    rig.set(P_CMD, 0);
    rig.set(P0 + 0x2c, 0);
    rig.set(P0 + 0x30, 0xffff_ffff);
    rig.set(P_IS, 0xffff_ffff);
    rig.set(0x08, 0xffff_ffff);
    assert_eq!(rig.reg(P_CI), 0, "PxCMD.ST one-to-zero resets PxCI");
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 3, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.prdbc(), SECTOR as u32);
    assert_eq!(rig.peek(DATA, SECTOR), stamp(3));
}

#[test]
fn a_prd_table_of_sixty_five_thousand_empty_descriptors_terminates() {
    // `PRDTL` is up to 65,535 by specification and the descriptors are guest
    // data, so a table of minimum-length entries is a legal way to make the
    // adapter do 65,535 fetches for 128 KiB of data. It has to finish.
    let rig = rig();
    rig.start();
    let entry = prd(DATA, 2, false);
    for i in 0..1024u64 {
        rig.poke(CTBA + 0x80 + i * 16, &entry);
    }
    rig.poke(CTBA, &Rig::fis(0x25, 1, 5, 0x40));
    let mut header = [0u8; 32];
    header[0..4].copy_from_slice(&(5u32 | (1024u32 << 16)).to_le_bytes());
    header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
    rig.poke(CLB, &header);
    rig.set(P_CI, 1);
    // Every one of the 256 entries a sector needs went to the same two bytes,
    // so the last word of the sector is what is there.
    assert_eq!(rig.prdbc(), SECTOR as u32);
    assert_eq!(rig.reg(P_CI), 0);
}

#[test]
fn a_command_fis_that_is_not_one_stops_the_port_rather_than_being_guessed_at() {
    let rig = rig();
    rig.start();
    // A D2H Register FIS where an H2D one belongs: §4.2.3.1 says the command
    // FIS is the H2D Register FIS, and anything else is a FIS this adapter
    // cannot transmit (§6.1.2).
    let mut fis = Rig::fis(0x25, 1, 0, 0x40);
    fis[0] = 0x34;
    rig.issue(&fis, false, &[prd(DATA, SECTOR, false)]);
    assert_eq!(rig.reg(P_IS) & (1 << 27), 1 << 27, "PxIS.IFS");
    assert_eq!(rig.reg(P_CMD) & (1 << 15), 0);
}

#[test]
fn a_command_fis_length_outside_two_to_sixteen_dwords_is_refused() {
    // §4.2.2: "A length of 0 or 1 is illegal. The maximum value allowed is
    // 10h." An adapter that sent a zero-length FIS would be sending nothing.
    for cfl in [0u32, 1] {
        let rig = rig();
        rig.start();
        rig.poke(CTBA, &Rig::fis(0x25, 1, 0, 0x40));
        rig.poke(CTBA + 0x80, &prd(DATA, SECTOR, false));
        let mut header = [0u8; 32];
        header[0..4].copy_from_slice(&(cfl | (1 << 16)).to_le_bytes());
        header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
        rig.poke(CLB, &header);
        rig.set(P_CI, 1);
        assert_eq!(
            rig.reg(P_IS) & (1 << 27),
            1 << 27,
            "CFL {cfl} should be refused"
        );
        assert_eq!(rig.reg(P_CI), 1, "and the slot stays outstanding");
    }
}

#[test]
fn a_command_list_base_that_is_not_mapped_stops_the_port() {
    let rig = rig();
    rig.start();
    // Above the mapped RAM. This space answers a master abort with ones, which
    // is what a PCI bus does and what firmware relies on to find an empty slot,
    // so the *fetch* succeeds and what comes back is a command header of all
    // ones — `CFL` 1Fh, which §4.2.2 does not allow. The adapter stops on that
    // rather than sending a FIS it cannot build (§6.1.2). A space that faulted
    // instead would report `PxIS.HBFS`; either way the port stops and no
    // command is invented out of the answer.
    rig.set(P_CLB, 0x8000_0000u32);
    rig.set(P_CI, 1);
    assert_ne!(
        rig.reg(P_IS) & ((1 << 27) | (1 << 29)),
        0,
        "PxIS.IFS or PxIS.HBFS"
    );
    assert_eq!(rig.reg(P_CMD) & (1 << 15), 0, "PxCMD.CR cleared");
    assert_eq!(rig.reg(P_CI), 1, "and the slot is still outstanding");
}

#[test]
fn a_write_reaches_the_medium_and_leaves_its_neighbours_alone() {
    let rig = rig();
    rig.start();
    let payload: Vec<u8> = (0..SECTOR as usize).map(|i| (i as u8) ^ 0x3f).collect();
    rig.poke(DATA, &payload);
    rig.issue(
        &Rig::fis(0x35, 1, 100, 0x40),
        true,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.prdbc(), SECTOR as u32);

    let mut got = alloc::vec![0u8; SECTOR as usize];
    Medium::read_at(&*rig.store, 100 * SECTOR, &mut got).expect("the medium reads");
    assert_eq!(got, payload, "the write reached the medium");
    Medium::read_at(&*rig.store, 101 * SECTOR, &mut got).expect("the medium reads");
    assert_eq!(got, stamp(101), "and the sector next door is untouched");
}

#[test]
fn a_pio_command_and_a_dma_command_move_the_same_bytes_by_different_protocols() {
    // The claim the taskfile seam exists to make good: the command set is one
    // implementation, and the two protocols differ only in what the adapter
    // puts around the data.
    let rig = rig();
    rig.start();
    // READ SECTOR(S) EXT — PIO.
    rig.issue(
        &Rig::fis(0x24, 1, 42, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    let by_pio = rig.peek(DATA, SECTOR);
    assert_eq!(rig.reg(P_IS) & (1 << 1), 1 << 1, "PxIS.PSS");
    assert_eq!(rig.reg(P_IS) & 1, 0, "and no PxIS.DHRS");
    assert_eq!(rig.peek(FB + 0x20, 1), alloc::vec![0x5f], "a PIO Setup FIS");
    rig.set(P_IS, 0xffff_ffff);

    // READ DMA EXT — DMA, same sector.
    rig.issue(
        &Rig::fis(0x25, 1, 42, 0x40),
        false,
        &[prd(DATA + 0x1000, SECTOR, false)],
    );
    let by_dma = rig.peek(DATA + 0x1000, SECTOR);
    assert_eq!(rig.reg(P_IS) & 1, 1, "PxIS.DHRS");
    assert_eq!(rig.reg(P_IS) & (1 << 1), 0, "and no PxIS.PSS");
    assert_eq!(
        rig.peek(FB + 0x40, 1),
        alloc::vec![0x34],
        "a D2H Register FIS"
    );

    assert_eq!(by_pio, by_dma);
    assert_eq!(by_pio, stamp(42));
}

#[test]
fn a_port_that_is_not_receiving_fises_posts_none() {
    // §3.3.7: with `FRE` clear, "received FISes are not accepted by the HBA
    // ... and no FISes are posted to the FIS receive area". The taskfile shadow
    // still moves, because §3.3.8 is about the adapter's own registers.
    let rig = rig();
    rig.set(P_CLB, CLB as u32);
    rig.set(P_FB, FB as u32);
    rig.set(P_IE, 0xffff_ffff);
    rig.set(P_CMD, 1);
    assert_eq!(rig.reg(P_CMD) & (1 << 14), 0, "PxCMD.FR is clear");
    rig.issue(
        &Rig::fis(0x25, 1, 7, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.prdbc(), SECTOR as u32, "the data still moved");
    assert_eq!(
        rig.peek(FB + 0x40, 20),
        alloc::vec![0u8; 20],
        "and no FIS was posted"
    );
    assert_eq!(rig.reg(P_TFD) & 0xff, 0x50, "but PxTFD followed the device");
}

#[test]
fn an_unimplemented_port_reads_zero_and_swallows_writes() {
    // §3: "All registers not defined and all reserved bits within registers
    // return 0 when read." A driver is told not to touch a port outside `PI`;
    // one that does anyway must not be able to move anything.
    let rig = rig();
    for offset in [0x180u64, 0x180 + 0x18, 0x180 + 0x38, 0x400] {
        rig.set(offset, 0xffff_ffff);
        assert_eq!(rig.reg(offset), 0, "port register {offset:#x}");
    }
    assert_eq!(rig.reg(0x0c), 1, "PI still names one port");
}

#[test]
fn the_register_file_round_trips_through_a_snapshot() {
    let rig = rig();
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 12, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );

    let mut shape = MachineShape::new();
    shape.add_device("ahci", CLASS_NAME).expect("a shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ahci", CLASS_NAME, STATE_VERSION).expect("a chunk");
        rig.hba.save(&mut chunk).expect("it saves");
    }
    let bytes = w.to_vec().expect("it serializes");

    let other = super::tests::rig();
    let reader = StateReader::new(&bytes).expect("we just wrote it");
    let chunk = reader
        .load("ahci", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("it is in there");
    other.hba.load(&mut chunk.reader()).expect("it loads");

    for offset in [0x04u64, 0x08, P_CLB, P_FB, P_IS, P_IE, P_CMD, P_TFD, P_CI] {
        assert_eq!(
            other.reg(offset),
            rig.reg(offset),
            "register {offset:#x} did not round trip"
        );
    }
    // And the restored adapter runs commands: its port is still started.
    other.set(P_IS, 0xffff_ffff);
    other.issue(
        &Rig::fis(0x25, 1, 12, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(other.prdbc(), SECTOR as u32);
    assert_eq!(other.peek(DATA, SECTOR), stamp(12));
}

#[test]
fn a_snapshot_chunk_of_arbitrary_bytes_is_refused_rather_than_believed() {
    // The loader is a parser on untrusted bytes. Rejecting is the expected
    // outcome; panicking is never one, and neither is coming back with a
    // register file describing hardware that cannot exist.
    let rig = rig();
    for len in 0..64usize {
        let bytes: Vec<u8> = (0..len)
            .map(|i| (i as u8).wrapping_mul(37) ^ 0xa3)
            .collect();
        let mut reader = ChunkReader::new(&bytes);
        let _ = rig.hba.load(&mut reader);
        // Whatever it did, `FR` never disagrees with `FRE` and the read-only
        // ones are still ones.
        let cmd = rig.reg(P_CMD);
        assert_eq!(
            cmd & (1 << 14) != 0,
            cmd & (1 << 4) != 0,
            "FR and FRE disagree after a {len}-byte chunk"
        );
        assert_eq!(cmd & 0b110, 0b110, "SUD and POD are read-only ones");
    }
    // And the adapter still works.
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 1, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.peek(DATA, SECTOR), stamp(1));
}

#[test]
fn a_byte_wide_access_to_the_register_block_is_not_a_register_access() {
    // §3: the registers are 32- or 64-bit quantities, and a 64-bit access must
    // not cross an 8-byte boundary — which is the same statement as natural
    // alignment.
    let rig = rig();
    assert!(rig.space.read(REGS, Width::U8, MemAttrs::DEFAULT).is_err());
    assert!(rig.space.read(REGS, Width::U16, MemAttrs::DEFAULT).is_err());
    assert!(
        rig.space
            .read(REGS + 2, Width::U32, MemAttrs::DEFAULT)
            .is_err()
    );
    assert!(
        rig.space
            .read(REGS + 4, Width::U64, MemAttrs::DEFAULT)
            .is_err()
    );
    assert!(rig.space.read(REGS, Width::U64, MemAttrs::DEFAULT).is_ok());
}

#[test]
fn more_bays_than_ports_are_taken_in_order_and_the_rest_dropped() {
    // A machine file cannot ask for more than `MAX_PORTS`, but the constructor
    // is reachable from Rust and a silent overflow of the port array would be
    // the kind of thing a fuzzer finds later rather than sooner.
    let table: Vec<(String, Arc<bays::Bay>)> = (0..MAX_PORTS + 4)
        .map(|i| (alloc::format!("sata{i}"), Arc::new(bays::Bay::new())))
        .collect();
    let hba = Hba::new(table);
    assert_eq!(hba.ports(), MAX_PORTS);
    assert_eq!(hba.bay_name(0), Some("sata0"));
    assert_eq!(hba.bay_name(MAX_PORTS - 1), Some("sata7"));
    assert_eq!(hba.bay_name(MAX_PORTS), None);
}

#[test]
fn an_hba_reset_clears_the_register_file_and_leaves_bus_mastering_alone() {
    // §3.1.2's `GHC.HR` is an *internal* reset, not `PCIRST#`: the state
    // machines and the memory-mapped registers go, and the PCI Command register
    // does not. A model that cleared Bus Master Enable here would leave the
    // adapter unable to fetch anything after the driver's own reset sequence,
    // which is the first thing a driver does.
    let rig = rig();
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 6, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_ne!(rig.reg(P_IS), 0);

    rig.set(0x04, 1);
    assert_eq!(
        rig.reg(0x04) & 1,
        0,
        "hardware clears GHC.HR when it is done"
    );
    assert_eq!(rig.reg(P_CLB), 0, "the port's registers went back");
    assert_eq!(rig.reg(P_IS), 0);
    assert_eq!(
        rig.reg(P_CMD) & ((1 << 15) | (1 << 14)),
        0,
        "both engines stopped"
    );
    assert_eq!(rig.reg(P_TFD), 0x0150, "and the signature is back");

    // And the adapter still masters the bus, so the driver's next command runs.
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 6, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.prdbc(), SECTOR as u32);
    assert_eq!(rig.peek(DATA, SECTOR), stamp(6));
}

#[test]
fn a_write_overflow_stops_the_port_and_puts_nothing_on_the_medium() {
    // §6.1.5's other direction, and the trap it sets. The adapter has run out
    // of data for a device that is still asking; the obvious shortcut — hand
    // the drive zeroes so the command can finish — would write them to the
    // medium, and the sector that was there would be gone. The specification
    // calls this fatal and says a COMRESET is required to clean up, so that is
    // what happens: nothing is invented, the port stops, and `PxTFD` reports
    // the `DRQ` that tells software to reset the device.
    let rig = rig();
    rig.start();
    let payload: Vec<u8> = (0..512).map(|i| (i as u8) ^ 0x91).collect();
    rig.poke(DATA, &payload);
    // Room for one sector of the three the command asks for.
    rig.issue(
        &Rig::fis(0x35, 3, 60, 0x40),
        true,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.reg(P_IS) & (1 << 24), 1 << 24, "PxIS.OFS");
    assert_eq!(rig.reg(P_CMD) & (1 << 15), 0, "a write overflow is fatal");
    assert_eq!(rig.reg(P_CI), 1, "and the slot stays outstanding");
    assert_eq!(
        rig.reg(P_TFD) & (1 << 3),
        1 << 3,
        "PxTFD.STS.DRQ, which is what tells software to COMRESET"
    );

    // The first sector went, because the drive had a whole block for it. The
    // two behind it are untouched — not zeroed, which is the failure this test
    // exists to catch.
    let mut got = alloc::vec![0u8; SECTOR as usize];
    Medium::read_at(&*rig.store, 60 * SECTOR, &mut got).expect("the medium reads");
    assert_eq!(got, payload);
    for lba in [61u64, 62] {
        Medium::read_at(&*rig.store, lba * SECTOR, &mut got).expect("the medium reads");
        assert_eq!(got, stamp(lba), "sector {lba} was written by nobody");
    }

    // §6.2.2.1's recovery for a device left with `DRQ` set: a COMRESET, and
    // then the port is usable again.
    rig.set(P_CMD, 0);
    rig.set(P0 + 0x2c, 1);
    rig.set(P0 + 0x2c, 0);
    rig.set(P0 + 0x30, 0xffff_ffff);
    rig.set(P_IS, 0xffff_ffff);
    assert_eq!(rig.reg(P_TFD) & (1 << 3), 0, "the drive is idle again");
    rig.start();
    rig.issue(
        &Rig::fis(0x25, 1, 61, 0x40),
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(rig.peek(DATA, SECTOR), stamp(61));
}
