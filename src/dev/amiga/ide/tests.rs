//! The A4000's IDE port as a 68040 sees it: through a big-endian 32-bit space,
//! at the addresses `machines/amiga-a4000.machine` decodes it at.
//!
//! The drive's own behaviour is `src/dev/ata/disk/tests.rs`'s. What is asserted
//! here is what happens between an address and a drive — the chip select, the
//! three drive address lines, the byte swap and the interrupt register — plus
//! `debug` reads and a snapshot round trip.

use super::*;
use crate::core::space::{AddressSpace, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::ata::disk::{self, Identity, Position, SECTOR, default_geometry};
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------

/// Where `machines/amiga-a4000.machine` maps the window: the two pages `A12`
/// picks between.
const BASE: u64 = 0x00DD_2000;

/// The command block's register *n*: the eight-bit register in the longword
/// slot at `$00DD2020 + 4n` sits on `D15`-`D8`, which is byte `+2`.
const fn cmd(n: u64) -> u64 {
    BASE + 0x022 + 4 * n
}

/// Device Control / Alternate Status: the control block, register 6.
const DEVCTL: u64 = BASE + 0x1022 + 4 * 6;

/// The board's interrupt register: the control block's register-0 slot, on
/// the board's own half of the bus. The ROM reads it as a word at
/// `$00DD3020` and looks at bit 15, which is bit 7 of this byte.
const INTREG: u64 = BASE + 0x1020;

/// What an undriven lane reads: the floating-bus byte the access carries. No
/// register below ever holds it, so a lane that floats cannot pass for one
/// that answered.
const FLOAT: u8 = 0xA5;

/// The Device register's two always-one bits (ATA-1's `1 x 1 DRV HEAD`).
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

fn drive_at(image: &[u8], position: Position) -> Arc<AtaDisk> {
    let sectors = image.len() as u64 / SECTOR;
    let id = Identity::new(sectors, default_geometry(sectors), true, 16).expect("a valid drive");
    let drive = AtaDisk::with_identity(id, position).expect("it fits in host memory");
    drive.load_image(0, image).expect("the image fits");
    Arc::new(drive)
}

struct Rig {
    ide: AmigaIde,
    space: AddressSpace,
    int2: Arc<Probe>,
}

fn rig_with(master: Option<Arc<AtaDisk>>, slave: Option<Arc<AtaDisk>>) -> Rig {
    let bays = [Arc::new(Bay::new()), Arc::new(Bay::new())];
    for (bay, drive) in bays.iter().zip([master, slave]) {
        if let Some(drive) = drive {
            bay.fit(drive).expect("an empty bay");
        }
    }
    let ide = AmigaIde::with_bays(
        bays,
        [String::from("ide-master"), String::from("ide-slave")],
    );

    let space = AddressSpace::new("mem", 32)
        .with_endian(Endian::Big)
        .with_unassigned(UnassignedPolicy::OPEN_BUS);
    let region = Device::region(&ide, IDE_REGION).expect("the region exists");
    space.topology().map(region, BASE).expect("it maps");

    let ids = WireIdAllocator::new();
    let int2 = Arc::new(Probe::default());
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&int2) as Arc<dyn WireSink>, 0)
        .build_shared();
    ide.connect(INT2_PIN, WireSource::new(wire, id))
        .expect("the pin exists");
    ide.announce(INT2_PIN);
    Rig { ide, space, int2 }
}

fn rig() -> Rig {
    rig_with(Some(drive_at(&hdf(), Position::Device0)), None)
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

    /// Select device 0 and wait for `DRDY`, the way a driver starts.
    fn select0(&self) {
        self.wb(cmd(6), DEV_OBS);
    }
}

// ---------------------------------------------------------------------------
// decode
// ---------------------------------------------------------------------------

#[test]
fn the_three_drive_address_lines_are_a2_to_a4_and_the_select_is_a12() {
    for n in 0..8u64 {
        assert_eq!(
            ide_decode(0x022 + 4 * n),
            Some((Select::Command, n as u8)),
            "command block register {n}"
        );
        assert_eq!(
            ide_decode(0x1022 + 4 * n),
            Some((Select::Control, n as u8)),
            "control block register {n}"
        );
    }
    // `A1` and `A0` are not decoded at all: the four bytes of a register's
    // longword slot are the same register, and which of them answers is the
    // byte lane rather than the decode.
    for lane in 0..4 {
        assert_eq!(ide_decode(0x020 + lane), Some((Select::Command, 0)));
    }
}

#[test]
fn nothing_below_a5_or_above_it_is_the_task_file() {
    // The board's select needs `A5` high and `A11`-`A6` low, which is what
    // puts the task file at `$…020` rather than at `$…000`.
    assert_eq!(ide_decode(0x000), None);
    assert_eq!(ide_decode(0x01f), None);
    assert_eq!(ide_decode(0x040), None);
    assert_eq!(ide_decode(0xfff), None);
    assert_eq!(ide_decode(0x1000), None);
}

#[test]
fn the_three_addresses_the_rom_probes_are_the_registers_they_have_to_be() {
    // What `amiga-os-310-a4000.rom` touches, recorded on a board that answers
    // nowhere else: Device/Head written, Cylinder Low and Status read.
    assert_eq!(BASE + 0x03a, cmd(6));
    assert_eq!(BASE + 0x032, cmd(4));
    assert_eq!(BASE + 0x03e, cmd(7));
    assert_eq!(command_register(6), Reg::Device);
    assert_eq!(command_register(4), Reg::LbaMid);
    assert_eq!(command_register(7), Reg::Command);
}

#[test]
fn an_eight_bit_register_is_on_the_even_byte_and_the_odd_one_floats() {
    let rig = rig();
    rig.select0();
    // Status is register 7. The even byte carries `DD7..DD0`; the odd byte is
    // `DD15..DD8`, which an eight-bit register does not drive.
    let status = rig.rb(cmd(7));
    assert_ne!(status, FLOAT, "the drive answered");
    assert!(status & disk::ST_DRDY != 0, "status {status:#04x}");
    assert_eq!(rig.rb(cmd(7) + 1), FLOAT, "the odd lane floats");
    // `A1` is not decoded, so the other even byte of the slot is the same
    // register and the other odd byte floats too.
    assert_eq!(rig.rb(cmd(7) - 2), status);
    assert_eq!(rig.rb(cmd(7) - 1), FLOAT);
}

#[test]
fn an_empty_cable_floats_both_lanes() {
    let rig = rig_with(None, None);
    assert_eq!(rig.rb(cmd(7)), FLOAT);
    assert_eq!(rig.rw(cmd(0)), u16::from_be_bytes([FLOAT, FLOAT]));
}

// ---------------------------------------------------------------------------
// the byte swap
// ---------------------------------------------------------------------------

#[test]
fn a_sector_read_through_the_byte_swapped_data_register_is_the_image() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    // READ SECTOR(S), one sector at LBA 0: the Rigid Disk Block's own block.
    rig.wb(cmd(6), DEV_OBS | 0x40); // LBA mode, device 0
    rig.wb(cmd(2), 1); // sector count
    rig.wb(cmd(3), 0); // LBA 7..0
    rig.wb(cmd(4), 0); // LBA 15..8
    rig.wb(cmd(5), 0); // LBA 23..16
    rig.wb(cmd(7), 0x20); // READ SECTOR(S)
    assert!(rig.rb(cmd(7)) & disk::ST_DRQ != 0, "the buffer is full");
    let got = rig.drain(SECTOR as usize / 2);
    assert_eq!(&got[..4], b"RDSK", "the RDB signature survives the swap");
    assert_eq!(got, image[..SECTOR as usize], "the whole sector");
}

#[test]
fn a_sector_written_through_the_data_register_comes_back_the_same() {
    // The other direction of the same swap: bytes written in memory order
    // reach the medium in memory order, so a sector written here and read back
    // is the bytes that went in. Without the swap each pair would be reversed
    // on the way out *and* on the way back, and this would still pass — so the
    // read test above, which compares against an image the drive was loaded
    // with, is the one that pins the direction; this one pins that a write is
    // the read's inverse.
    let rig = rig_with(Some(drive_at(&hdf(), Position::Device0)), None);
    rig.select0();
    let mut want = alloc::vec![0u8; SECTOR as usize];
    for (i, byte) in want.iter_mut().enumerate() {
        *byte = (i as u8).wrapping_mul(0x1d) ^ 0x3c;
    }
    want[..4].copy_from_slice(b"RDB!");

    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 1); // LBA 1
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x30); // WRITE SECTOR(S)
    assert!(
        rig.rb(cmd(7)) & disk::ST_DRQ != 0,
        "the buffer wants a block"
    );
    for pair in want.as_chunks::<2>().0 {
        rig.ww(cmd(0), u16::from_be_bytes(*pair));
    }

    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 1);
    rig.wb(cmd(7), 0x20); // READ SECTOR(S)
    assert!(rig.rb(cmd(7)) & disk::ST_DRQ != 0, "the buffer is full");
    assert_eq!(rig.drain(SECTOR as usize / 2), want);
}

#[test]
fn a_byte_write_drives_both_halves_of_the_bus() {
    let rig = rig();
    // Sector Count is an eight-bit register on `DD7..DD0`; a byte written to
    // the *odd* address still reaches it, because a byte write puts the byte
    // on both halves of the data bus.
    rig.wb(cmd(2) + 1, 0x5a);
    assert_eq!(rig.rb(cmd(2)), 0x5a);
}

// ---------------------------------------------------------------------------
// the interrupt register
// ---------------------------------------------------------------------------

#[test]
fn the_drives_intrq_reads_at_the_control_blocks_register_zero_and_pulls_int2() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    assert!(!rig.int2.high(), "no interrupt out of reset");
    assert_eq!(rig.rb(INTREG) & INT_PENDING, 0);

    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 0);
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x20); // READ SECTOR(S) — the drive raises `INTRQ`

    assert!(rig.int2.high(), "`INT2` is pulled");
    // The ROM reads it as a *word* and looks at bit 15, so the byte the board
    // drives is the first of the two.
    assert_eq!(rig.rw(INTREG) & 0x8000, 0x8000, "the register says so");
    assert_eq!(rig.rb(INTREG) & INT_PENDING, INT_PENDING);
    // The rest of that byte is nothing the board drives, and the drive's own
    // half of the longword is nobody's register at all.
    assert_eq!(rig.rb(INTREG) & !INT_PENDING, FLOAT & !INT_PENDING);
    assert_eq!(rig.rb(INTREG + 2), FLOAT);
    assert_eq!(rig.rb(INTREG + 3), FLOAT);

    // Reading Status releases `INTRQ` (ATA-1 §9.5), and the register is that
    // line rather than a latch in front of it: it goes down with it, and so
    // does `INT2`. That is what the ROM's poll loop needs — it acknowledges
    // by reading Status and never writes here at all.
    let _ = rig.rb(cmd(7));
    assert!(!rig.int2.high(), "reading Status let go of `INT2`");
    assert_eq!(rig.rw(INTREG) & 0x8000, 0);
}

#[test]
fn alternate_status_is_the_control_blocks_register_six_and_has_no_side_effect() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 0);
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x20);
    assert!(rig.int2.high());
    let alt = rig.rb(DEVCTL);
    assert_eq!(alt & disk::ST_DRQ, disk::ST_DRQ);
    assert!(rig.int2.high(), "Alternate Status does not acknowledge");
}

// ---------------------------------------------------------------------------
// `MemAttrs::debug`
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_neither_acknowledges_the_drive_nor_drops_the_interrupt() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 0);
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x20);
    assert!(rig.int2.high());
    assert_eq!(rig.peek_b(INTREG) & INT_PENDING, INT_PENDING);
    let status = rig.peek_b(cmd(7));
    assert_eq!(status & disk::ST_DRQ, disk::ST_DRQ);
    assert!(
        rig.int2.high(),
        "a debug read of Status did not acknowledge"
    );
    // And the sector buffer has not moved on.
    let got = rig.drain(SECTOR as usize / 2);
    assert_eq!(got, image[..SECTOR as usize]);
}

#[test]
fn a_debug_write_is_refused() {
    let rig = rig();
    for addr in [cmd(7), cmd(0), DEVCTL, INTREG] {
        assert!(
            rig.space
                .write(addr, Width::U8, 0, MemAttrs::DEBUG)
                .is_err(),
            "{addr:#010x} took a debug write"
        );
    }
}

// ---------------------------------------------------------------------------
// construction, reset and the snapshot
// ---------------------------------------------------------------------------

#[test]
fn two_positions_on_one_cable_cannot_be_one_bay() {
    let props = Props::new().with("master", "ata0").with("slave", "ata0");
    let err = AmigaIde::new(&props).expect_err("one bay for both");
    assert!(format!("{err}").contains("cannot both be"), "{err}");
}

/// A reset re-drives the pin from the cable, and that is all it can do: the
/// interrupt register is the drive's `INTRQ` rather than a latch in front of
/// it, and the drive is its own device with its own reset.
#[test]
fn a_reset_redrives_the_pin_from_the_cable() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 0);
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x20);
    assert!(rig.int2.high());
    Device::reset(&rig.ide, ResetKind::Cold);
    assert!(rig.ide.interrupt(), "the drive still has the line up");
    assert!(rig.int2.high());

    // Reading Status releases `INTRQ` (ATA-1 §9.5) and the pin goes with it.
    let _ = rig.rb(cmd(7));
    assert!(!rig.int2.high());
    Device::reset(&rig.ide, ResetKind::Cold);
    assert!(!rig.ide.interrupt());
    assert!(!rig.int2.high());
}

/// The port has no state of its own, and the snapshot says so: the chunk is
/// empty, and a restore leaves the pin following the cable rather than a
/// serialized copy of it.
#[test]
fn the_port_carries_no_state_of_its_own() {
    let image = hdf();
    let rig = rig_with(Some(drive_at(&image, Position::Device0)), None);
    rig.select0();
    rig.wb(cmd(6), DEV_OBS | 0x40);
    rig.wb(cmd(2), 1);
    rig.wb(cmd(3), 0);
    rig.wb(cmd(4), 0);
    rig.wb(cmd(5), 0);
    rig.wb(cmd(7), 0x20);
    assert!(rig.ide.interrupt());

    let mut shape = MachineShape::new();
    shape.add_device("ide", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ide", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&rig.ide, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    // A fresh port with a quiet cable: the restore leaves it quiet, because
    // there was nothing in the chunk to say otherwise.
    let fresh = rig_with(Some(drive_at(&image, Position::Device0)), None);
    assert!(!fresh.ide.interrupt());
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ide", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&fresh.ide, &mut chunk.reader()).unwrap();
    assert!(!fresh.ide.interrupt());
    assert!(!fresh.int2.high());

    // And the same chunk saved again is the same bytes: identical state.
    let mut shape = MachineShape::new();
    shape.add_device("ide", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ide", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&fresh.ide, &mut chunk).unwrap();
    }
    assert_eq!(w.to_vec().unwrap(), bytes);
}
