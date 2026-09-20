//! A CD-ROM on a Serial ATA port, driven the way a driver drives one.
//!
//! **No firmware, no processor and no machine file.** The board is an address
//! space with RAM in it, an `ahci.hba` mastering that space, and an `ata.cdrom`
//! in its port 0 bay — assembled here out of the public API, because what is
//! under test is the *adapter carrying a packet device* and a board in the
//! middle would be a second thing that could be wrong. `tests/ahci_board.rs`
//! is the same shape with a hard disk in the bay, and
//! `tests/pc_at_cdrom.rs` is the same command set reached through eight I/O
//! ports instead.
//!
//! The disc is an ISO 9660 image this test assembles from nothing — the same
//! `tests/iso9660` builder the AT's CD-ROM tests use. Nothing is vendored and
//! nothing is fetched.
//!
//! Every offset, bit and structure below is written out of the specification
//! rather than imported from the model, so that the two have to agree:
//!
//! * **Serial ATA AHCI Specification, Revision 1.3.1** (Intel) — §3.3 the port
//!   registers, §4.2.2 the command header and its `A` bit, §4.2.3 the command
//!   table and the `ACMD` field at offset `40h`, §5.6.3 the PIO flows, §6.1.4
//!   `TFES`;
//! * **Serial ATA, Revision 1.0** §8.5.2 the Register - Host to Device FIS;
//! * **T13 ATA/ATAPI-6** §8.21 `PACKET` and its byte count limit, §8.16
//!   `IDENTIFY PACKET DEVICE`, §9.1 the reset signature;
//! * **SFF-8020i** — `INQUIRY`, `READ CD-ROM CAPACITY`, `READ(10)`,
//!   `TEST UNIT READY` and the unit-attention rule.
//!
//! The order is weakest claim first:
//!
//! * the port reports `EB140101h` — the packet-device signature — where a hard
//!   disk reports `00000101h`, and `PxCMD.ATAPI` is a bit a driver can set;
//! * `IDENTIFY PACKET DEVICE` travels the command list as an ordinary PIO
//!   data-in command and word 0 says this is a CD-ROM;
//! * `INQUIRY` travels as a **packet** command: the twelve bytes come out of
//!   the command table's `ACMD` field because the header's `A` bit says so, and
//!   no PRD is spent on them;
//! * the power-on unit attention is reported once, which stops the port, and
//!   §6.2.2's recovery restarts it — a driver's ordinary first exchange;
//! * `READ CD-ROM CAPACITY` reports the disc the test built;
//! * `READ(10)` of the primary volume descriptor lands `CD001` in guest memory
//!   the adapter was never handed directly, with the neighbouring block
//!   untouched;
//! * a multi-block `READ(10)` cut into pieces by the byte count limit
//!   reassembles into the right bytes;
//! * a command whose header has `A` clear cannot be completed and is reported
//!   as an interface error rather than hanging.

#![cfg(all(feature = "dev-ahci", feature = "dev-ata-atapi"))]

use std::sync::Arc;

use rsemu::core::space::{
    AddressSpace, MemAttrs, MemOps, RamStore, Region, RequesterId, UnassignedPolicy,
};
use rsemu::core::value::Width;
use rsemu::dev::ahci::hba::{Hba, REGISTER_LEN, port_offset, structure_sizes};
use rsemu::dev::ata::atapi::{AtapiDrive, Identity};
use rsemu::dev::ata::bays::Bay;
use rsemu::dev::ata::{AtaDevice, Position};
use rsemu::dev::medium::Medium;

mod iso9660;

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Where guest RAM starts. Not zero, so a null pointer in a command header is a
/// bus fault rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x40_0000;
/// Where this "driver" decided to put the register block.
const REGS: u64 = 0x1000_0000;

/// A CD-ROM's logical block (ECMA-119 §6.1.2).
const BLOCK: u64 = 2048;

// The structures the driver builds in its own RAM.
const CLB: u64 = 0x0010_0000;
const FB: u64 = 0x0010_1000;
const CTBA: u64 = 0x0010_2000;
const DATA: u64 = 0x0020_0000;

// Port register offsets, as a driver knows them (AHCI §3.3). Restated rather
// than imported: a test sharing the module's constants could not catch one of
// them moving.
const P_CLB: u64 = 0x00;
const P_FB: u64 = 0x08;
const P_IS: u64 = 0x10;
const P_IE: u64 = 0x14;
const P_CMD: u64 = 0x18;
const P_TFD: u64 = 0x20;
const P_SIG: u64 = 0x24;
const P_CI: u64 = 0x38;

/// `PxCMD.ST`, `PxCMD.FRE`, `PxCMD.CR` and `PxCMD.ATAPI` (§3.3.7).
const CMD_ST: u32 = 1 << 0;
const CMD_FRE: u32 = 1 << 4;
const CMD_CR: u32 = 1 << 15;
const CMD_ATAPI: u32 = 1 << 24;

/// `PxIS.DHRS`, `PxIS.PSS` and `PxIS.TFES` (§3.3.5, §6.1.4).
const IS_DHRS: u32 = 1 << 0;
const IS_PSS: u32 = 1 << 1;
const IS_TFES: u32 = 1 << 30;
const IS_IFS: u32 = 1 << 27;

/// What a packet device leaves in the four command block registers after a
/// reset, as `PxSIG` packs them (ATA/ATAPI-6 §9.1, AHCI §3.3.9).
const SIG_ATAPI: u32 = 0xeb14_0101;

/// The `A` and `C` bits of a command header's first dword (§4.2.2).
const HEADER_A: u32 = 1 << 5;
const HEADER_C: u32 = 1 << 10;

/// The ATA opcodes this test issues. Written here because a driver has them.
const CMD_PACKET: u8 = 0xa0;
const CMD_IDENTIFY_PACKET: u8 = 0xa1;

/// The packet opcodes (SFF-8020i).
const PKT_TEST_UNIT_READY: u8 = 0x00;
const PKT_INQUIRY: u8 = 0x12;
const PKT_READ_CAPACITY: u8 = 0x25;
const PKT_READ_10: u8 = 0x28;

struct Rig {
    hba: Arc<Hba>,
    space: Arc<AddressSpace>,
    /// How many logical blocks the disc the test built holds.
    blocks: u64,
    /// The disc, so an assertion can check the bytes that reached the guest
    /// against the bytes on the medium rather than against the model.
    image: Vec<u8>,
}

/// The board, with an ISO in the drive unless `disc` says otherwise.
fn rig_with(disc: bool, dma: bool) -> Rig {
    let image = iso9660::data_disc(b"AHCI CD-ROM, no firmware in sight.\n");
    let store = Arc::new(RamStore::new(image.len() as u64));
    RamStore::write_at(&store, 0, &image).expect("the image fits");

    let mut id = Identity::new();
    id.dma = dma;
    let medium: Option<Arc<dyn Medium>> = disc.then(|| Arc::clone(&store) as Arc<dyn Medium>);
    let drive = Arc::new(
        AtapiDrive::with_disc(id, Position::Device0, medium).expect("the disc is whole blocks"),
    );

    let bay = Arc::new(Bay::new());
    bay.fit_device(Arc::clone(&drive) as Arc<dyn AtaDevice>)
        .expect("an empty bay");
    let hba = Arc::new(Hba::new(vec![(String::from("sata0"), bay)]));

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
    // `reset` is `PCIRST#`, which clears Bus Master Enable; firmware sets it
    // again, and so does this.
    hba.set_master(true);

    Rig {
        hba,
        space,
        blocks: image.len() as u64 / BLOCK,
        image,
    }
}

fn rig() -> Rig {
    rig_with(true, false)
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

    fn port(&self, within: u64) -> u32 {
        self.reg(port_offset(0) + within)
    }

    fn set_port(&self, within: u64, value: u32) {
        self.set(port_offset(0) + within, value);
    }

    fn poke(&self, at: u64, bytes: &[u8]) {
        self.space
            .write_bytes(at, bytes, MemAttrs::DEFAULT)
            .expect("guest RAM");
    }

    fn peek(&self, at: u64, len: usize) -> Vec<u8> {
        let mut out = vec![0u8; len];
        self.space
            .read_bytes(at, &mut out, MemAttrs::DEFAULT)
            .expect("guest RAM");
        out
    }

    /// Clear the structures and start the port, as a driver's bring-up does.
    fn start(&self) {
        let (list, fis) = structure_sizes();
        self.poke(CLB, &vec![0u8; list as usize]);
        self.poke(FB, &vec![0u8; fis as usize]);
        self.set_port(P_CLB, CLB as u32);
        self.set_port(P_FB, FB as u32);
        self.set_port(P_IE, 0xffff_ffff);
        self.set_port(P_IS, 0xffff_ffff);
        // `FRE` before `ST`: §5.5.1's start sequence. `ATAPI` because the
        // signature said so, which is exactly what a driver does with it.
        self.set_port(P_CMD, CMD_FRE | CMD_ATAPI);
        self.set_port(P_CMD, CMD_FRE | CMD_ATAPI | CMD_ST);
    }

    /// Issue one command out of slot 0 and return the port's `PxIS`.
    ///
    /// `cfis` is the Register - Host to Device FIS, `acmd` the command packet
    /// or `None`, and `prd` the one scatter/gather entry — address and byte
    /// count — or `None` for a command that moves nothing.
    fn issue(&self, cfis: &[u8; 20], acmd: Option<&[u8]>, prd: Option<(u64, u32)>) -> u32 {
        self.poke(CTBA, &[0u8; 0x80 + 16]);
        self.poke(CTBA, cfis);
        let mut dw0 = 5u32 | HEADER_C;
        if let Some(packet) = acmd {
            // §4.2.3: the `ACMD` field is at offset 40h in the command table,
            // and §4.2.2's `A` bit is what says it holds a command.
            dw0 |= HEADER_A;
            self.poke(CTBA + 0x40, packet);
        }
        if let Some((addr, bytes)) = prd {
            dw0 |= 1 << 16; // PRDTL
            let mut prd_entry = [0u8; 16];
            prd_entry[0..4].copy_from_slice(&(addr as u32).to_le_bytes());
            prd_entry[4..8].copy_from_slice(&((addr >> 32) as u32).to_le_bytes());
            // §4.2.3.3: `DBC` is a zero-based byte count.
            prd_entry[12..16].copy_from_slice(&(bytes - 1).to_le_bytes());
            self.poke(CTBA + 0x80, &prd_entry);
        }
        let mut header = [0u8; 32];
        header[0..4].copy_from_slice(&dw0.to_le_bytes());
        header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
        self.poke(CLB, &header);

        self.set_port(P_IS, 0xffff_ffff);
        self.set_port(P_CI, 1);
        self.port(P_IS)
    }

    /// `PRDBC`, the byte count the adapter says actually transferred (§5.4.1).
    fn prdbc(&self) -> u32 {
        u32::from_le_bytes(self.peek(CLB + 4, 4).try_into().expect("four bytes"))
    }

    /// §6.2.2's recovery: stop the port and start it again.
    fn restart(&self) {
        self.set_port(P_CMD, CMD_FRE | CMD_ATAPI);
        self.set_port(P_IS, 0xffff_ffff);
        self.set_port(P_CMD, CMD_FRE | CMD_ATAPI | CMD_ST);
    }
}

/// A Register - Host to Device FIS carrying `command` (Serial ATA §8.5.2).
///
/// `lba` is the whole 24-bit field, which for a `PACKET` is the byte count
/// limit in its top sixteen bits (ATA/ATAPI-6 §8.21.4).
fn h2d(command: u8, feature: u8, lba: u32) -> [u8; 20] {
    let mut fis = [0u8; 20];
    fis[0] = 0x27; // Register - Host to Device
    fis[1] = 1 << 7; // `C`: this is a command, not a Device Control write
    fis[2] = command;
    fis[3] = feature;
    fis[4] = lba as u8;
    fis[5] = (lba >> 8) as u8;
    fis[6] = (lba >> 16) as u8;
    fis
}

/// A `PACKET` command FIS with `limit` as its byte count limit.
fn packet_fis(limit: u16) -> [u8; 20] {
    h2d(CMD_PACKET, 0, u32::from(limit) << 8)
}

/// A twelve-byte command descriptor block.
fn cdb(bytes: &[u8]) -> [u8; 12] {
    let mut out = [0u8; 12];
    out[..bytes.len()].copy_from_slice(bytes);
    out
}

fn read_10(lba: u32, blocks: u16) -> [u8; 12] {
    let mut out = [0u8; 12];
    out[0] = PKT_READ_10;
    out[2..6].copy_from_slice(&lba.to_be_bytes());
    out[7..9].copy_from_slice(&blocks.to_be_bytes());
    out
}

// ---------------------------------------------------------------------------
// the port
// ---------------------------------------------------------------------------

/// `PxSIG` is how a driver learns what is on a port, and a packet device is the
/// only reason the register is interesting: ATA/ATAPI-6 §9.1 has it leave
/// `01h/01h/14h/EBh` in the Sector Count and three LBA registers, which
/// AHCI §3.3.9 packs as `EB140101h`.
#[test]
fn a_packet_device_reports_the_atapi_signature() {
    let rig = rig();
    assert_eq!(rig.port(P_SIG), SIG_ATAPI);
    // And `PxTFD` is `00h`, not the `50h` a hard disk leaves: §7.15.6.3 makes
    // `DRDY` a bit a packet device does not have, so its Status register at
    // rest is zero. That pair is the whole of how a driver tells them apart.
    assert_eq!(rig.port(P_TFD) & 0xff, 0x00);
}

/// `PxCMD.ATAPI` is a bit a driver sets once it has read that signature
/// (§3.3.7). It has no consequence in a model with no activity LED, and a
/// driver still expects to read back what it wrote.
#[test]
fn the_port_remembers_that_a_packet_device_is_on_it() {
    let rig = rig();
    assert_eq!(rig.port(P_CMD) & CMD_ATAPI, 0);
    rig.start();
    assert_eq!(rig.port(P_CMD) & CMD_ATAPI, CMD_ATAPI);
    assert_eq!(rig.port(P_CMD) & CMD_CR, CMD_CR);
}

/// An empty tray is a drive, not an absence: the port is occupied, the
/// signature is a packet device's, and the drive answers `MEDIUM NOT PRESENT`
/// when asked for the disc it does not have.
#[test]
fn a_drive_with_no_disc_is_still_a_drive_on_the_port() {
    let rig = rig_with(false, false);
    assert_eq!(rig.port(P_SIG), SIG_ATAPI);
    rig.start();
    // The first command takes the power-on unit attention; the second gets the
    // honest answer. Both fail, which is what stops the port each time.
    for _ in 0..2 {
        let is = rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
        assert_ne!(
            is & IS_TFES,
            0,
            "a failed packet command is a taskfile error"
        );
        rig.restart();
    }
}

// ---------------------------------------------------------------------------
// an ordinary PIO command, with no packet in it
// ---------------------------------------------------------------------------

/// `IDENTIFY PACKET DEVICE` is an **ATA** command and not a packet one, so it
/// travels the command list with the header's `A` bit clear and no `ACMD` at
/// all — which is why that bit is per command rather than per port.
#[test]
fn identify_packet_device_travels_the_command_list() {
    let rig = rig();
    rig.start();
    let is = rig.issue(&h2d(CMD_IDENTIFY_PACKET, 0, 0), None, Some((DATA, 512)));
    // A PIO data-in command completes on its last PIO Setup FIS (§5.6.3.3),
    // so `PSS` and not `DHRS`.
    assert_ne!(is & IS_PSS, 0, "PxIS = {is:#x}");
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_eq!(rig.prdbc(), 512);

    let block = rig.peek(DATA, 512);
    let word0 = u16::from(block[0]) | (u16::from(block[1]) << 8);
    // §8.16 word 0: bits 15:14 = 10b, an ATAPI device; bits 12:8 = 5, the
    // CD-ROM command packet set; bit 7, removable medium.
    assert_eq!(word0 & 0xc000, 0x8000, "word 0 = {word0:#06x}");
    assert_eq!((word0 >> 8) & 0x1f, 5, "word 0 = {word0:#06x}");
    assert_ne!(word0 & 0x0080, 0, "word 0 = {word0:#06x}");
}

// ---------------------------------------------------------------------------
// packet commands
// ---------------------------------------------------------------------------

/// The exchange a driver actually performs, in order, and the thing this whole
/// commit is for: the twelve bytes of command reach the drive out of the
/// command table's `ACMD` field.
///
/// `INQUIRY` is first because SFF-8020i §9.3 exempts it from the unit-attention
/// rule, so it is the one command a freshly powered drive answers.
#[test]
fn inquiry_travels_as_a_packet_out_of_the_command_table() {
    let rig = rig();
    rig.start();
    let is = rig.issue(
        &packet_fis(2048),
        Some(&cdb(&[PKT_INQUIRY, 0, 0, 0, 36])),
        Some((DATA, 36)),
    );
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    // §9.10: a packet data-in command ends on a Register - Device to Host FIS
    // *after* its last data block — the one place the packet protocol differs
    // from an ATA PIO read, which has no such FIS. So both bits.
    assert_ne!(is & IS_PSS, 0, "PxIS = {is:#x}");
    assert_ne!(is & IS_DHRS, 0, "PxIS = {is:#x}");
    // The command packet is not part of the transfer: 36 bytes of response and
    // not 48.
    assert_eq!(rig.prdbc(), 36);

    let answer = rig.peek(DATA, 36);
    // §10.4: peripheral device type 05h is a CD-ROM, and bit 7 of byte 1 is
    // RMB, a removable medium.
    assert_eq!(answer[0] & 0x1f, 0x05, "{answer:02x?}");
    assert_eq!(answer[1] & 0x80, 0x80, "{answer:02x?}");
    assert_eq!(&answer[8..13], b"RSEMU", "{answer:02x?}");
}

/// The power-on unit attention, reported once, and the recovery a driver does
/// about it. SFF-8020i §9.3 gives it to the first command that is neither
/// `INQUIRY` nor `REQUEST SENSE`, and AHCI §6.1.4 turns the `CHK` that carries
/// it into `PxIS.TFES` — which §6.2.2 makes fatal, clearing `PxCMD.CR` and
/// leaving the slot in `PxCI` for software to find.
#[test]
fn the_power_on_unit_attention_stops_the_port_and_the_driver_restarts_it() {
    let rig = rig();
    rig.start();
    let is = rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
    assert_ne!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_eq!(rig.port(P_CMD) & CMD_CR, 0, "a fatal error clears CR");
    assert_eq!(rig.port(P_CI) & 1, 1, "the failing slot stays outstanding");

    rig.restart();
    assert_eq!(rig.port(P_CMD) & CMD_CR, CMD_CR);
    assert_eq!(rig.port(P_CI), 0, "ST one-to-zero clears PxCI");

    // And now the drive is ready, because the attention has been reported.
    let is = rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_ne!(
        is & IS_DHRS,
        0,
        "a non-data packet command ends on a D2H FIS"
    );
}

/// `READ CD-ROM CAPACITY` reports the **address of the last block**, not the
/// count, and the block length beside it.
#[test]
fn read_capacity_describes_the_disc_the_test_built() {
    let rig = rig();
    rig.start();
    // Absorb the unit attention first, the way a driver does.
    rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
    rig.restart();

    let is = rig.issue(
        &packet_fis(2048),
        Some(&cdb(&[PKT_READ_CAPACITY])),
        Some((DATA, 8)),
    );
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    let answer = rig.peek(DATA, 8);
    let last = u32::from_be_bytes(answer[0..4].try_into().expect("four bytes"));
    let length = u32::from_be_bytes(answer[4..8].try_into().expect("four bytes"));
    assert_eq!(u64::from(last), rig.blocks - 1);
    assert_eq!(u64::from(length), BLOCK);
}

/// A `READ(10)` of the primary volume descriptor, which ECMA-119 §6.2.1 puts at
/// logical block 16 and §8.4 starts with `CD001`.
///
/// The bytes are checked against the image this test wrote to the medium, not
/// against anything the adapter said, and the block after the destination is
/// asserted untouched — an adapter that moved one block too many would pass a
/// test that only looked at the first.
#[test]
fn read_10_lands_the_volume_descriptor_in_guest_memory() {
    let rig = rig();
    rig.start();
    rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
    rig.restart();

    rig.poke(DATA, &vec![0xa5u8; (BLOCK * 2) as usize]);
    let is = rig.issue(
        &packet_fis(2048),
        Some(&read_10(16, 1)),
        Some((DATA, BLOCK as u32)),
    );
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_eq!(u64::from(rig.prdbc()), BLOCK);

    let got = rig.peek(DATA, BLOCK as usize);
    assert_eq!(&got[1..6], b"CD001", "{:02x?}", &got[..8]);
    let at = 16 * BLOCK as usize;
    assert_eq!(got, rig.image[at..at + BLOCK as usize]);
    assert!(
        rig.peek(DATA + BLOCK, BLOCK as usize)
            .iter()
            .all(|b| *b == 0xa5),
        "the adapter wrote past the block it was asked for"
    );
}

/// The byte count limit is the most a packet device may move in one `DRQ`
/// block (ATA/ATAPI-6 §8.21.5), so a four-block read at a limit of one block is
/// four blocks on the link and one transfer to the driver. `CAP.PMD` says this
/// adapter does multiple-block PIO, and this is that claim being kept.
#[test]
fn a_byte_count_limit_cuts_the_transfer_and_it_reassembles() {
    let rig = rig();
    rig.start();
    rig.issue(&packet_fis(2048), Some(&cdb(&[PKT_TEST_UNIT_READY])), None);
    rig.restart();

    let blocks = 4u64;
    let want = (blocks * BLOCK) as u32;
    rig.poke(DATA, &vec![0u8; want as usize]);
    let is = rig.issue(
        &packet_fis(BLOCK as u16),
        Some(&read_10(16, blocks as u16)),
        Some((DATA, want)),
    );
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_eq!(rig.prdbc(), want);
    let at = 16 * BLOCK as usize;
    assert_eq!(
        rig.peek(DATA, want as usize),
        rig.image[at..at + want as usize]
    );
}

/// The same read with the DMA bit set in the Features register, which is what a
/// driver on a bus-mastering adapter actually issues. `PxIS.PSS` must **not**
/// appear: a DMA command has no PIO Setup FIS at all (Serial ATA 2.6 §11.9), and
/// an adapter that posted one would show a driver a PIO transfer that never
/// happened.
#[test]
fn a_dma_packet_command_ends_on_a_register_fis_and_not_a_pio_setup() {
    let rig = rig_with(true, true);
    rig.start();
    rig.issue(
        &h2d(CMD_PACKET, 1, u32::from(BLOCK as u16) << 8),
        Some(&cdb(&[PKT_TEST_UNIT_READY])),
        None,
    );
    rig.restart();

    let is = rig.issue(
        &h2d(CMD_PACKET, 1, u32::from(BLOCK as u16) << 8),
        Some(&read_10(16, 1)),
        Some((DATA, BLOCK as u32)),
    );
    assert_eq!(is & IS_TFES, 0, "PxIS = {is:#x}");
    assert_ne!(is & IS_DHRS, 0, "PxIS = {is:#x}");
    assert_eq!(is & IS_PSS, 0, "a DMA command has no PIO Setup FIS");
    let got = rig.peek(DATA, BLOCK as usize);
    assert_eq!(&got[1..6], b"CD001", "{:02x?}", &got[..8]);
}

/// A `PACKET` whose command header has the `A` bit clear. The device raises
/// `DRQ` over a block it will not proceed without and the adapter has nowhere
/// to get one from — on a real link the host would sit there until it timed the
/// command out. §6.1.2's interface error is the nearest honest report, and the
/// point is that the engine stops rather than spinning.
#[test]
fn a_packet_command_with_no_acmd_is_an_interface_error() {
    let rig = rig();
    rig.start();
    // `issue` sets `A` only when it is given a packet, so this is the case.
    let is = rig.issue(&packet_fis(2048), None, Some((DATA, 36)));
    assert_ne!(is & IS_IFS, 0, "PxIS = {is:#x}");
    assert_eq!(rig.port(P_CMD) & CMD_CR, 0, "a fatal error clears CR");
    // And the adapter is still answering, which is the part a spin would fail.
    assert_eq!(rig.port(P_SIG), SIG_ATAPI);
    assert!(rig.hba.ports() == 1);
}
