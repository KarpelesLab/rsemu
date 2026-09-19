//! Gayle as a 68000 sees it: through a big-endian space, at the addresses the
//! A600 decodes it at.
//!
//! The drive's own behaviour is `src/dev/ata/disk/tests.rs`'s. What is asserted
//! here is what happens between an address and a drive — the chip selects, the
//! byte swap, the four registers and the interrupt they make of `INTRQ` — plus
//! the identification sequence, the empty card slot, the overlay, `debug`
//! reads, and a snapshot.

use super::*;
use crate::core::space::RamStore;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::ata::disk::{self, Identity, Position, SECTOR, default_geometry};
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------

const IDE_BASE: u64 = 0xDA_0000;
const REGS_BASE: u64 = 0xDA_8000;
const ID_BASE: u64 = 0xDE_1000;
const CIA_ODD: u64 = 0xBF_E000;
const CIA_EVEN: u64 = 0xBF_D000;

/// The command block, `-IDE_CS1` at the "16 bit" timing: register *n* at
/// `$DA2000 + 4n`.
const fn cmd(n: u64) -> u64 {
    IDE_BASE + 0x2000 + 4 * n
}

/// Device Control / Alternate Status: `-IDE_CS2`, register 6.
const DEVCTL: u64 = IDE_BASE + 0x3018;

const STATUS: u64 = REGS_BASE;
const CHANGE: u64 = REGS_BASE + 0x1000;
const ENABLE: u64 = REGS_BASE + 0x2000;
const CONFIG: u64 = REGS_BASE + 0x3000;

/// What an undriven lane reads: the floating-bus byte the access carries
/// (`MemAttrs::bus`). Every read here carries this one, which no register
/// below ever holds, so a lane that floats cannot pass for one that answered.
const FLOAT: u8 = 0xA5;

/// The Device register's two always-one bits (ATA-1's `1 x 1 DRV HEAD`). The
/// drive keeps its own copy private; a driver writes them.
const DEV_OBS: u8 = 0xA0;

/// A wire sink that remembers the last level it was given.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl Probe {
    fn high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

/// A 64-sector drive whose sector 0 starts `RDSK` — an Amiga hard-disk image's
/// first four bytes — and whose every other byte says where it is.
fn hdf() -> Vec<u8> {
    let mut image = alloc::vec![0u8; (64 * SECTOR) as usize];
    for (i, byte) in image.iter_mut().enumerate() {
        *byte = (i as u8) ^ ((i >> 9) as u8).wrapping_mul(0x3b);
    }
    image[..4].copy_from_slice(b"RDSK");
    image
}

fn disk(image: &[u8], position: Position) -> Arc<AtaDisk> {
    let sectors = image.len() as u64 / SECTOR;
    let id = Identity::new(sectors, default_geometry(sectors), true, 16).expect("a valid drive");
    let drive = AtaDisk::with_identity(id, position).expect("it fits in host memory");
    drive.load_image(0, image).expect("the image fits");
    Arc::new(drive)
}

struct Rig {
    gayle: Gayle,
    space: AddressSpace,
    int2: Arc<Probe>,
    int6: Arc<Probe>,
    ovl: Arc<Probe>,
    /// The two CIA stand-ins, odd first: a page of RAM each.
    cias: [Arc<RamStore>; 2],
}

/// A Gayle with `master`/`slave` fitted, every region placed where the A600
/// places it, both CIA links pointed at stand-in RAM, and every pin on a probe.
fn rig_with(master: Option<Arc<AtaDisk>>, slave: Option<Arc<AtaDisk>>) -> Rig {
    let bays = [Arc::new(Bay::new()), Arc::new(Bay::new())];
    for (bay, drive) in bays.iter().zip([master, slave]) {
        if let Some(drive) = drive {
            bay.fit(drive).expect("an empty bay");
        }
    }
    let gayle = Gayle::with_bays(
        bays,
        [String::from("ide-master"), String::from("ide-slave")],
        [None, None],
    );
    let cias = [
        Arc::new(RamStore::new(CIA_WINDOW_LEN)),
        Arc::new(RamStore::new(CIA_WINDOW_LEN)),
    ];
    for (which, store) in CiaSelect::ALL.into_iter().zip(&cias) {
        let region: RegionRef = Arc::new(
            Region::ram(format!("cia.{}", which.name()), Arc::clone(store))
                .with_endian(Endian::Big),
        );
        gayle
            .attach_cia(which, &region, 24)
            .expect("a private space");
    }

    let space = AddressSpace::new("mem", 24)
        .with_endian(Endian::Big)
        .with_unassigned(UnassignedPolicy::OPEN_BUS);
    for (name, at) in [
        (IDE_REGION, IDE_BASE),
        (REGS_REGION, REGS_BASE),
        (ID_REGION, ID_BASE),
        ("cia-odd", CIA_ODD),
        ("cia-even", CIA_EVEN),
    ] {
        let region = Device::region(&gayle, name).expect("the region exists");
        space.topology().map(region, at).expect("it maps");
    }

    let ids = WireIdAllocator::new();
    let probe = |pin: &str| {
        let probe = Arc::new(Probe::default());
        let id = ids.alloc();
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        gayle
            .connect(pin, WireSource::new(wire, id))
            .expect("the pin exists");
        gayle.announce(pin);
        probe
    };
    let int2 = probe(INT2_PIN);
    let int6 = probe(INT6_PIN);
    let ovl = probe(OVL_PIN);
    Rig {
        gayle,
        space,
        int2,
        int6,
        ovl,
        cias,
    }
}

fn rig() -> Rig {
    rig_with(Some(disk(&hdf(), Position::Device0)), None)
}

fn bus() -> MemAttrs {
    MemAttrs::DEFAULT.with_bus(FLOAT)
}

impl Rig {
    fn rb(&self, addr: u64) -> u8 {
        self.space
            .read(addr, Width::U8, bus())
            .expect("a byte read") as u8
    }

    fn rw(&self, addr: u64) -> u16 {
        self.space
            .read(addr, Width::U16, bus())
            .expect("a word read") as u16
    }

    fn peek_b(&self, addr: u64) -> u8 {
        self.space
            .read(addr, Width::U8, MemAttrs::DEBUG.with_bus(FLOAT))
            .expect("a debug byte read") as u8
    }

    fn peek_w(&self, addr: u64) -> u16 {
        self.space
            .read(addr, Width::U16, MemAttrs::DEBUG.with_bus(FLOAT))
            .expect("a debug word read") as u16
    }

    fn wb(&self, addr: u64, value: u8) {
        self.space
            .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("a byte write");
    }

    fn ww(&self, addr: u64, value: u16) {
        self.space
            .write(addr, Width::U16, u64::from(value), MemAttrs::DEFAULT)
            .expect("a word write");
    }

    /// Read `words` words from the data register into memory order, the way a
    /// `move.w (a0),(a1)+` loop lays them down.
    fn drain(&self, words: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(words * 2);
        for _ in 0..words {
            out.extend_from_slice(&self.rw(cmd(0)).to_be_bytes());
        }
        out
    }
}

// ---------------------------------------------------------------------------
// decode
// ---------------------------------------------------------------------------

#[test]
fn the_three_drive_address_lines_are_a2_to_a4_and_the_select_is_a12() {
    assert_eq!(ide_decode(0x2000), (Select::Command, 0));
    assert_eq!(ide_decode(0x201C), (Select::Command, 7));
    // A1 is not decoded: `$DA201E` is the same register as `$DA201C`.
    assert_eq!(ide_decode(0x201E), (Select::Command, 7));
    // A13 is timing only: the "8 bit" window is the same command block.
    assert_eq!(ide_decode(0x0004), (Select::Command, 1));
    assert_eq!(ide_decode(0x3018), (Select::Control, CONTROL_REGISTER));
    assert_eq!(ide_decode(0x1018), (Select::Control, CONTROL_REGISTER));
    for (da, reg) in [
        Reg::Data,
        Reg::Feature,
        Reg::SectorCount,
        Reg::LbaLow,
        Reg::LbaMid,
        Reg::LbaHigh,
        Reg::Device,
        Reg::Command,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(command_register(da as u8), reg);
    }
    assert_eq!(register_at(0x0000), 0);
    assert_eq!(register_at(0x1FFF), 1);
    assert_eq!(register_at(0x2000), 2);
    assert_eq!(register_at(0x3000), 3);
    assert_eq!(register_at(0x7000), 3, "the 32 KiB window repeats the four");
}

#[test]
fn the_eight_bit_registers_are_on_the_even_byte_and_the_odd_one_floats() {
    let r = rig();
    // LBA mid is a plain read/write register on the drive: an echo test, which
    // is how Kickstart looks for a drive before it sends a command.
    r.wb(cmd(4), 0x12);
    assert_eq!(r.rb(cmd(4)), 0x12);
    assert_eq!(r.rb(cmd(4) + 1), FLOAT, "DD15-DD8 are not driven");
    assert_eq!(r.rw(cmd(4)), 0x1200 | u16::from(FLOAT));
    // A byte written at the odd address is on both halves of the bus, so it
    // reaches DD7-DD0 all the same.
    r.wb(cmd(4) + 1, 0x34);
    assert_eq!(r.rb(cmd(4)), 0x34);
    // A word write puts D15-D8 on DD7-DD0.
    r.ww(cmd(4), 0x56AA);
    assert_eq!(r.rb(cmd(4)), 0x56);
    // The same register through the "8 bit" window, and through A1 set.
    assert_eq!(r.rb(IDE_BASE + 4 * 4), 0x56);
    assert_eq!(r.rb(cmd(4) + 2), 0x56);
}

#[test]
fn the_control_block_is_a12_high_and_its_other_addresses_float() {
    let r = rig();
    // Alternate status: a drive at rest, never zero and never the float.
    let alt = r.rb(DEVCTL);
    assert_ne!(alt, 0);
    assert_ne!(alt, FLOAT);
    assert_eq!(r.rb(IDE_BASE + 0x1018), alt, "the \"8 bit\" alias");
    assert_eq!(r.rb(IDE_BASE + 0x3000), FLOAT, "no register at CS3FX, DA 0");
    // `$DA4000` up selects nothing (section 7.0, "None"), so the region stops
    // short of it and the board's own open bus answers there.
    assert_eq!(IDE_WINDOW_LEN, 0x4000);
    assert_eq!(r.rb(IDE_BASE + 0x4000), FLOAT);
}

#[test]
fn an_empty_cable_floats_and_an_empty_position_beside_a_drive_reads_zero() {
    let empty = rig_with(None, None);
    assert_eq!(empty.rb(cmd(7)), FLOAT);
    assert_eq!(empty.rb(DEVCTL), FLOAT);
    assert_eq!(empty.rw(cmd(0)), u16::from_be_bytes([FLOAT, FLOAT]));

    let r = rig();
    r.wb(cmd(6), DEV_OBS | disk::DEV_SELECT); // device 1: nobody
    assert_eq!(r.rb(cmd(7)), 0, "the master answers for the empty slave");
    r.wb(cmd(6), DEV_OBS);
    assert_ne!(r.rb(cmd(7)), 0);
}

// ---------------------------------------------------------------------------
// the data path and the interrupt
// ---------------------------------------------------------------------------

#[test]
fn identify_arrives_byte_swapped_and_raises_int2_through_gayle() {
    let r = rig();
    // Enable the IDE interrupt onto INT2, as Kickstart does before it sends a
    // command.
    r.wb(ENABLE, IDE);
    assert!(!r.int2.high());

    r.wb(cmd(6), DEV_OBS);
    r.wb(cmd(7), disk::cmd::IDENTIFY);

    // The drive raised INTRQ; Gayle shows the line, latched the change, and
    // pulled INT2.
    assert_eq!(r.rb(STATUS) & IDE, IDE, "the line itself");
    assert_eq!(r.rb(CHANGE) & IDE, IDE, "its change latch");
    assert!(r.int2.high(), "INT2");
    assert!(!r.int6.high(), "and not INT6");

    // Reading the drive's status acknowledges it: the line drops, which is a
    // second change, and INT2 stays up until software writes the latch clear.
    let status = r.rb(cmd(7));
    assert_ne!(status & disk::ST_DRQ, 0, "a block is waiting");
    assert_eq!(r.rb(STATUS) & IDE, 0);
    assert!(r.int2.high(), "the latch holds until a 0 is written");
    r.wb(CHANGE, 0x7C);
    assert_eq!(r.rb(CHANGE) & IDE, 0);
    assert!(!r.int2.high());

    // 256 words. In memory each arrives low byte first — the swap — so the
    // model string, whose first character ATA puts in each word's high byte,
    // reads pairwise swapped at the 68000's addresses.
    let block = r.drain(256);
    let words: Vec<u16> = block
        .chunks(2)
        .map(|b| u16::from_le_bytes([b[0], b[1]]))
        .collect();
    let mut model = alloc::string::String::new();
    for w in &words[27..47] {
        model.push((w >> 8) as u8 as char);
        model.push(*w as u8 as char);
    }
    assert!(
        model.starts_with("RSEMU HARDDISK"),
        "the drive's IDENTIFY, word for word: {model:?}"
    );
    assert_eq!(&block[54..58], b"SRME", "and what a 68000 sees at word 27");
}

#[test]
fn a_sector_arrives_in_memory_in_the_order_the_image_holds_it() {
    let image = hdf();
    let r = rig();
    // LBA 3, one sector.
    r.wb(cmd(6), DEV_OBS | disk::DEV_LBA);
    r.wb(cmd(2), 1);
    r.wb(cmd(3), 3);
    r.wb(cmd(4), 0);
    r.wb(cmd(5), 0);
    r.wb(cmd(7), disk::cmd::READ_SECTORS);
    let _ = r.rb(cmd(7));
    let at = (3 * SECTOR) as usize;
    assert_eq!(r.drain(256), &image[at..at + SECTOR as usize]);

    // And sector 0 starts `RDSK`, which is what `scsi.device` looks for: an
    // HDF needs no swapping to be a disk on this port.
    r.wb(cmd(3), 0);
    r.wb(cmd(2), 1);
    r.wb(cmd(4), 0);
    r.wb(cmd(5), 0);
    r.wb(cmd(7), disk::cmd::READ_SECTORS);
    let _ = r.rb(cmd(7));
    let first = r.drain(256);
    assert_eq!(&first[..4], b"RDSK");
    assert_eq!(first, &image[..SECTOR as usize]);
}

#[test]
fn a_sector_written_through_the_port_lands_in_the_image_in_order() {
    let r = rig();
    r.wb(cmd(6), DEV_OBS | disk::DEV_LBA);
    r.wb(cmd(2), 1);
    r.wb(cmd(3), 9);
    r.wb(cmd(4), 0);
    r.wb(cmd(5), 0);
    r.wb(cmd(7), disk::cmd::WRITE_SECTORS);
    let payload: Vec<u8> = (0..SECTOR as usize).map(|i| (i * 7) as u8).collect();
    for pair in payload.chunks(2) {
        r.ww(cmd(0), u16::from_be_bytes([pair[0], pair[1]]));
    }
    let drive = r.gayle.drive(Position::Device0).expect("a drive");
    let mut sector = alloc::vec![0u8; SECTOR as usize];
    drive.read_media(9 * SECTOR, &mut sector).expect("in range");
    assert_eq!(sector, payload);
}

#[test]
fn with_the_enable_clear_a_change_is_latched_but_int2_stays_quiet() {
    let r = rig();
    r.wb(cmd(6), DEV_OBS);
    r.wb(cmd(7), disk::cmd::IDENTIFY);
    assert_eq!(r.rb(CHANGE) & IDE, IDE);
    assert!(!r.int2.high());
    // Enabling it afterwards raises INT2 for the change already latched.
    r.wb(ENABLE, IDE);
    assert!(r.int2.high());
}

#[test]
fn nien_keeps_intrq_down_and_so_gayle_sees_nothing() {
    let r = rig();
    r.wb(ENABLE, IDE);
    r.wb(DEVCTL, disk::CTL_NIEN);
    r.wb(cmd(6), DEV_OBS);
    r.wb(cmd(7), disk::cmd::IDENTIFY);
    assert_eq!(r.rb(STATUS) & IDE, 0);
    assert_eq!(r.rb(CHANGE) & IDE, 0);
    assert!(!r.int2.high());
}

// ---------------------------------------------------------------------------
// the four registers and the empty card slot
// ---------------------------------------------------------------------------

#[test]
fn with_no_card_every_card_line_reads_negated_and_nothing_interrupts() {
    let r = rig();
    // Everything enabled, both levels.
    r.wb(ENABLE, 0xFF);
    assert_eq!(r.rb(STATUS) & (CC_DET | BVD2 | BVD1 | WR | BSY), 0);
    assert_eq!(r.rb(CHANGE), 0);
    assert!(!r.int2.high());
    assert!(!r.int6.high());
    // Registers are on the even byte; the odd one floats.
    assert_eq!(r.rb(STATUS + 1), FLOAT);
    assert_eq!(r.rw(ENABLE), 0xFF00 | u16::from(FLOAT));
}

#[test]
fn a_forced_card_detect_is_a_change_and_interrupts_on_int6() {
    let r = rig();
    r.wb(ENABLE, CC_DET);
    r.wb(STATUS, CC_DET);
    assert_eq!(
        r.rb(STATUS) & CC_DET,
        CC_DET,
        "forcing reads back as asserted"
    );
    assert_eq!(r.rb(CHANGE) & CC_DET, CC_DET);
    assert!(r.int6.high());
    assert!(!r.int2.high());
    // A 1 leaves a latch alone; a 0 clears it.
    r.wb(CHANGE, 0xFF);
    assert!(r.int6.high());
    r.wb(CHANGE, !CC_DET);
    assert!(!r.int6.high());
    // Letting go of the line is a change too.
    r.wb(STATUS, 0);
    assert_eq!(r.rb(CHANGE) & CC_DET, CC_DET);
}

#[test]
fn bsy_and_bvd_interrupt_on_the_level_their_bits_choose() {
    let r = rig();
    r.wb(ENABLE, BSY);
    r.wb(STATUS, BSY);
    assert!(r.int2.high() && !r.int6.high(), "level bit clear: int2");
    r.wb(ENABLE, BSY | BSY_LEVEL6);
    assert!(r.int6.high() && !r.int2.high(), "level bit set: int6");
    r.wb(STATUS, 0);
    r.wb(CHANGE, 0);
    r.wb(ENABLE, BVD1 | BVD_LEVEL6);
    r.wb(STATUS, BVD1);
    assert!(r.int6.high() && !r.int2.high());
}

#[test]
fn the_control_bits_are_plain_and_the_page_registers_read_zero() {
    let r = rig();
    r.wb(STATUS, CONTROL);
    assert_eq!(r.rb(STATUS), CONTROL);
    r.wb(CHANGE, CONTROL);
    assert_eq!(r.rb(CHANGE), CONTROL);
    // "You can tell they are unimplemented because they do not read back what
    // is written."
    r.wb(CONFIG, 0xFF);
    assert_eq!(r.rb(CONFIG), CONFIG_BITS);
}

// ---------------------------------------------------------------------------
// the identification register
// ---------------------------------------------------------------------------

#[test]
fn the_id_register_shifts_out_d_msb_first_after_a_write() {
    let r = rig();
    // What Kickstart does: a word, a byte of zero, then byte reads.
    r.ww(ID_BASE, 0xBFFF);
    r.wb(ID_BASE, 0);
    let bits: Vec<u8> = (0..8).map(|_| r.rb(ID_BASE) & ID_BIT).collect();
    assert_eq!(bits, [0x80, 0x80, 0, 0x80, 0, 0, 0, 0], "$D, then zeroes");
    assert_eq!(r.rb(ID_BASE), 0, "and zero after the eighth");
    // Only bit 7 carries anything.
    r.wb(ID_BASE, 0);
    assert_eq!(r.rb(ID_BASE), ID_BIT);
    // A write starts the sequence over, wherever it was.
    r.wb(ID_BASE, 0);
    let again: Vec<u8> = (0..4).map(|_| r.rb(ID_BASE)).collect();
    assert_eq!(again, [0x80, 0x80, 0, 0x80]);
    // Anywhere in the 4 KiB page: Gayle sees nothing below A12.
    r.wb(ID_BASE + 0x0FFE, 0);
    assert_eq!(r.rb(ID_BASE + 0x0F00), ID_BIT);
}

// ---------------------------------------------------------------------------
// the overlay
// ---------------------------------------------------------------------------

#[test]
fn the_overlay_is_up_out_of_reset_and_the_first_write_to_either_cia_drops_it() {
    for (base, index) in [(CIA_ODD, 0usize), (CIA_EVEN, 1)] {
        let r = rig();
        assert!(r.gayle.overlaid());
        assert!(r.ovl.high());
        // Reads pass through and leave it alone.
        let _ = r.rb(base + 0x201);
        assert!(r.ovl.high());
        r.wb(base + 0x201, 0x03);
        assert!(!r.gayle.overlaid());
        assert!(!r.ovl.high());
        let mut byte = [0u8; 1];
        r.cias[index].read_at(0x201, &mut byte).expect("in range");
        assert_eq!(byte[0], 0x03, "the write reached the CIA behind the select");
        // And a reset brings it back.
        r.gayle.reset(ResetKind::Warm);
        assert!(r.ovl.high());
    }
}

// ---------------------------------------------------------------------------
// MemAttrs::debug
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_pops_nothing() {
    let r = rig();
    r.wb(ENABLE, IDE);
    r.wb(cmd(6), DEV_OBS | disk::DEV_LBA);
    r.wb(cmd(2), 1);
    r.wb(cmd(3), 0);
    r.wb(cmd(4), 0);
    r.wb(cmd(5), 0);
    r.wb(cmd(7), disk::cmd::READ_SECTORS);

    // Status, peeked: the interrupt is still pending, the line still up.
    let status = r.peek_b(cmd(7));
    assert_ne!(status & disk::ST_DRQ, 0);
    assert_eq!(r.peek_b(STATUS) & IDE, IDE);
    assert!(r.int2.high());
    // Data, peeked twice: the same word, and the buffer has not moved.
    let first = r.peek_w(cmd(0));
    assert_eq!(r.peek_w(cmd(0)), first);
    assert_eq!(first.to_be_bytes(), *b"RD");
    let _ = r.rb(cmd(7));
    assert_eq!(
        r.rw(cmd(0)).to_be_bytes(),
        *b"RD",
        "the real read gets the first word"
    );

    // The identification sequence does not advance under a debugger.
    r.wb(ID_BASE, 0);
    assert_eq!(r.peek_b(ID_BASE), ID_BIT);
    assert_eq!(r.peek_b(ID_BASE), ID_BIT);
    assert_eq!(r.peek_b(ID_BASE), ID_BIT);
    let seq: Vec<u8> = (0..4).map(|_| r.rb(ID_BASE)).collect();
    assert_eq!(seq, [0x80, 0x80, 0, 0x80]);

    // A debug write is refused outright, everywhere a write does something.
    for addr in [cmd(7), cmd(0), DEVCTL, CHANGE, ID_BASE] {
        assert!(
            r.space.write(addr, Width::U8, 0, MemAttrs::DEBUG).is_err(),
            "a debug write at {addr:#x}"
        );
    }
    // And a debug write to a CIA through the select does not drop the
    // overlay.
    let _ = r
        .space
        .write(CIA_ODD + 0x201, Width::U8, 3, MemAttrs::DEBUG);
    assert!(r.gayle.overlaid());
}

// ---------------------------------------------------------------------------
// snapshot
// ---------------------------------------------------------------------------

fn snapshot(g: &Gayle) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("gayle", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("gayle", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(g, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = rig();
    // Something in every field: a forced line, a latched change, enables,
    // configuration, the ID sequence part-way, and the overlay down.
    saved.wb(ENABLE, IDE | WR | BSY_LEVEL6);
    saved.wb(STATUS, WR | 0x01);
    saved.wb(CHANGE, !0x02);
    saved.wb(CONFIG, 0x0A);
    saved.wb(ID_BASE, 0);
    let _ = saved.rb(ID_BASE);
    let _ = saved.rb(ID_BASE);
    saved.wb(CIA_EVEN, 0);
    let bytes = snapshot(&saved.gayle);

    let restored = rig();
    assert_ne!(snapshot(&restored.gayle), bytes);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("gayle", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.gayle, &mut chunk.reader()).unwrap();

    assert_eq!(snapshot(&restored.gayle), bytes, "identical state");
    assert!(!restored.gayle.overlaid());
    assert!(!restored.ovl.high(), "the pin follows the restored overlay");
    assert!(
        restored.int2.high(),
        "and INT2 the restored latch and enable"
    );
    assert_eq!(restored.rb(ID_BASE), 0, "the third bit of $D");
    assert_eq!(restored.rb(CONFIG), 0x0A);
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn the_properties_are_checked_and_the_schema_matches() {
    use crate::core::props::Value;
    let mut props = Props::new();
    props.insert("master", Value::Str(String::from("a")));
    props.insert("slave", Value::Str(String::from("a")));
    assert!(
        Gayle::new(&props).is_err(),
        "one bay cannot be both positions"
    );

    let mut props = Props::new();
    props.insert("bogus", Value::Bool(true));
    assert!(Gayle::new(&props).is_err());

    let gayle = Gayle::new(&Props::new()).expect("every property has a default");
    for region in [
        "",
        IDE_REGION,
        REGS_REGION,
        ID_REGION,
        "cia-odd",
        "cia-even",
    ] {
        assert!(Device::region(&gayle, region).is_some(), "{region}");
    }
    assert_eq!(
        Device::region(&gayle, REGS_REGION).map(|r| r.len()),
        Some(REGS_WINDOW_LEN)
    );
    let schema = schema();
    assert_eq!(schema.class, CLASS_NAME);
}
