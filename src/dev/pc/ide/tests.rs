//! What the adapter does with the ports, and — as much as anything — what it
//! does when there is nothing on the cable.
//!
//! These drive the two register blocks the way an `IN` and an `OUT` reach them,
//! because everything this file models is what happens between a port number
//! and a drive. The drive's own behaviour is asserted next door, in
//! `src/dev/ata/disk/tests.rs`.

use super::*;
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::ata::disk::{
    self, Identity, Position, SECTOR, ST_DRDY, ST_DRQ, ST_DSC, ST_ERR, default_geometry,
};
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------

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

/// A drive of 4096 sectors whose every sector says which sector it is.
fn stamped(position: Position) -> Arc<AtaDisk> {
    let id = Identity::new(4096, default_geometry(4096), true, 16).expect("a valid drive");
    let disk = AtaDisk::with_identity(id, position).expect("it fits in host memory");
    for lba in 0..4096u64 {
        disk.write_media(lba * SECTOR, &stamp(lba))
            .expect("in range");
    }
    Arc::new(disk)
}

fn stamp(lba: u64) -> Vec<u8> {
    let mut out = alloc::vec![0u8; SECTOR as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// A channel with whichever positions the caller wants populated, its `irq`
/// pin on a probe.
struct Rig {
    ide: Ide,
    cmd: CommandBlock,
    ctl: ControlBlock,
    irq: Arc<Probe>,
}

fn rig_with(master: bool, slave: bool) -> Rig {
    let bays = [Arc::new(Bay::new()), Arc::new(Bay::new())];
    if master {
        bays[0]
            .fit(stamped(Position::Device0))
            .expect("an empty bay");
    }
    if slave {
        bays[1]
            .fit(stamped(Position::Device1))
            .expect("an empty bay");
    }
    let ide = Ide::with_bays(
        [Arc::clone(&bays[0]), Arc::clone(&bays[1])],
        [String::from("ide0-master"), String::from("ide0-slave")],
    );
    let cmd = CommandBlock(Arc::clone(&ide.channel));
    let ctl = ControlBlock(Arc::clone(&ide.channel));
    let ids = WireIdAllocator::new();
    let irq = Arc::new(Probe::default());
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&irq) as Arc<dyn WireSink>, 0)
        .build_shared();
    ide.connect("irq", WireSource::new(wire, id))
        .expect("the pin exists");
    Rig { ide, cmd, ctl, irq }
}

fn rig() -> Rig {
    rig_with(true, false)
}

impl Rig {
    fn inb(&self, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        self.cmd
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peekb(&self, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        self.cmd
            .read(offset, &mut byte, MemAttrs::DEBUG)
            .expect("a debug byte read is legal");
        byte[0]
    }

    fn outb(&self, offset: u64, value: u8) {
        self.cmd
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    fn inw(&self) -> u16 {
        let mut word = [0u8; 2];
        self.cmd
            .read(0, &mut word, MemAttrs::DEFAULT)
            .expect("a word read of the data port is legal");
        u16::from_le_bytes(word)
    }

    fn outw(&self, value: u16) {
        self.cmd
            .write(0, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write of the data port is legal");
    }

    /// The alternate status register at the control block.
    fn alt(&self) -> u8 {
        let mut byte = [0u8; 1];
        self.ctl
            .read(0, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn devctl(&self, value: u8) {
        self.ctl
            .write(0, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Read one sector at `lba` through the ports, the way a driver does.
    fn read_sector(&self, lba: u32) -> Vec<u8> {
        self.outb(6, 0xe0 | ((lba >> 24) as u8 & 0x0f)); // device: LBA, device 0
        self.outb(2, 1); // sector count
        self.outb(3, lba as u8);
        self.outb(4, (lba >> 8) as u8);
        self.outb(5, (lba >> 16) as u8);
        self.outb(7, disk::cmd::READ_SECTORS);
        assert_ne!(self.alt() & ST_DRQ, 0, "the drive never raised DRQ");
        let mut out = Vec::with_capacity(SECTOR as usize);
        for _ in 0..256 {
            out.extend_from_slice(&self.inw().to_le_bytes());
        }
        out
    }
}

// ---------------------------------------------------------------------------
// decode
// ---------------------------------------------------------------------------

#[test]
fn the_eight_offsets_name_the_eight_registers() {
    // The whole ATA content of the adapter, asserted so that a reordering is a
    // failing test rather than a driver that reads the wrong register.
    assert_eq!(register_at(0), Reg::Data);
    assert_eq!(register_at(1), Reg::Feature);
    assert_eq!(register_at(2), Reg::SectorCount);
    assert_eq!(register_at(3), Reg::LbaLow);
    assert_eq!(register_at(4), Reg::LbaMid);
    assert_eq!(register_at(5), Reg::LbaHigh);
    assert_eq!(register_at(6), Reg::Device);
    assert_eq!(register_at(7), Reg::Command);
    // An eight-port window repeats nothing; the mask is there so an aliased
    // mapping decodes the same way.
    assert_eq!(register_at(15), Reg::Command);
}

#[test]
fn the_channel_publishes_both_of_its_windows_and_nothing_else() {
    let rig = rig();
    assert_eq!(
        rig.ide.region("").expect("the default region").len(),
        COMMAND_WINDOW_LEN
    );
    assert_eq!(
        rig.ide.region("regs").expect("the command block").len(),
        COMMAND_WINDOW_LEN
    );
    assert_eq!(
        rig.ide.region("ctl").expect("the control block").len(),
        CONTROL_WINDOW_LEN
    );
    assert!(rig.ide.region("data").is_none());
}

// ---------------------------------------------------------------------------
// an empty cable
// ---------------------------------------------------------------------------

#[test]
fn an_empty_channel_reads_ones() {
    // Nothing is driving the bus, and an ISA bus with nothing driving it reads
    // as ones — the same fact `machines/pc-at.machine` sets
    // `unassigned = read-as-ones` for.
    let rig = rig_with(false, false);
    for offset in 0..8 {
        assert_eq!(rig.inb(offset), 0xff, "port offset {offset}");
    }
    assert_eq!(rig.alt(), 0xff);
    assert!(!rig.irq.high());
}

#[test]
fn a_lone_master_answers_for_the_slave_that_is_not_there() {
    // How a driver probes: select device 1, read status, and a channel with
    // only a master reads back **zero**, because the drive that is there drives
    // the bus low on behalf of the one that is not. A model that returned ones
    // here would look to a driver like a drive that is permanently busy.
    let rig = rig();
    assert_eq!(rig.inb(7), ST_DRDY | ST_DSC, "the master is there");

    rig.outb(6, 0xf0); // DEV set: device 1
    assert_eq!(rig.inb(7), 0x00, "and device 1 is not");
    assert_eq!(rig.alt(), 0x00);
    for offset in 1..7 {
        assert_eq!(rig.inb(offset), 0x00, "port offset {offset}");
    }

    // And back.
    rig.outb(6, 0xe0);
    assert_eq!(rig.inb(7), ST_DRDY | ST_DSC);
}

#[test]
fn selecting_device_1_switches_which_drive_answers() {
    let rig = rig_with(true, true);
    // Something only one of them will have.
    rig.outb(6, 0xe0);
    rig.outb(2, 0x11);
    rig.outb(6, 0xf0);
    rig.outb(2, 0x22);
    assert_eq!(rig.inb(2), 0x22, "device 1 kept its own sector count");
    rig.outb(6, 0xe0);
    assert_eq!(rig.inb(2), 0x11, "and device 0 kept its own");
}

// ---------------------------------------------------------------------------
// moving a sector
// ---------------------------------------------------------------------------

#[test]
fn a_sector_reaches_the_host_a_word_at_a_time() {
    let rig = rig();
    assert_eq!(rig.read_sector(1234), stamp(1234));
    assert_eq!(rig.alt(), ST_DRDY | ST_DSC, "and the command is over");
}

#[test]
fn a_sector_written_through_the_ports_lands_on_the_medium() {
    // The claim that proves the whole path: bytes go in through port offset 0
    // and come out of the drive's medium at the right byte offset — checked
    // against the medium rather than by reading the drive's own buffer back.
    let rig = rig();
    let payload = stamp(0x321);
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 0x21);
    rig.outb(4, 0x03);
    rig.outb(5, 0x00);
    rig.outb(7, disk::cmd::WRITE_SECTORS);
    assert_ne!(rig.alt() & ST_DRQ, 0, "the drive is asking for the block");
    for pair in payload.chunks(2) {
        rig.outw(u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    assert_eq!(rig.inb(7) & ST_ERR, 0);

    let drive = rig
        .ide
        .drive(Position::Device0)
        .expect("the master is fitted");
    let mut got = alloc::vec![0u8; SECTOR as usize];
    drive
        .read_media(0x321 * SECTOR, &mut got)
        .expect("in range");
    assert_eq!(got, payload);
    assert_eq!(rig.read_sector(0x321), payload, "and it reads back");
}

#[test]
fn a_byte_wide_read_of_the_data_port_still_shifts_a_whole_word() {
    // An 8-bit cycle on a 16-bit bus: the drive has no idea how wide the host's
    // access was and hands over a word regardless, and the adapter's buffer
    // keeps the half nobody asked for. Asserted rather than left to chance,
    // because the alternative — half-advancing the buffer — would desynchronise
    // every later word.
    let rig = rig();
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 0);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    let expected = stamp(0);
    assert_eq!(rig.inb(0), expected[0], "the low half of the first word");
    assert_eq!(rig.inb(0), expected[2], "and then the low half of the next");
}

#[test]
fn a_wide_access_to_a_task_file_register_is_refused() {
    // Only offset zero is more than eight bits wide. A word access anywhere
    // else is not something a real adapter decodes, and answering it would be
    // inventing behaviour.
    let rig = rig();
    let mut two = [0u8; 2];
    assert!(rig.cmd.read(2, &mut two, MemAttrs::DEFAULT).is_err());
    assert!(rig.cmd.write(2, &two, MemAttrs::DEFAULT).is_err());
    let mut four = [0u8; 4];
    assert!(rig.cmd.read(4, &mut four, MemAttrs::DEFAULT).is_err());
    // And a doubleword at offset zero is two data words, which a 32-bit local
    // bus adapter does do.
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 5);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    rig.cmd
        .read(0, &mut four, MemAttrs::DEFAULT)
        .expect("a doubleword at offset zero");
    assert_eq!(four, stamp(5)[..4]);
}

// ---------------------------------------------------------------------------
// interrupts and MemAttrs::debug
// ---------------------------------------------------------------------------

#[test]
fn the_status_port_acknowledges_and_the_alternate_status_port_does_not() {
    // The distinction the two register blocks exist to make, seen from the
    // ports rather than from the drive.
    let rig = rig();
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 9);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    assert!(rig.irq.high(), "INTRQ reached the pin");

    // Looking through the control block changes nothing.
    for _ in 0..4 {
        assert_ne!(rig.alt() & ST_DRQ, 0);
        assert!(
            rig.irq.high(),
            "the alternate status acknowledged something"
        );
    }
    // Looking through the command block does.
    let _ = rig.inb(7);
    assert!(!rig.irq.high(), "the status register did not acknowledge");
}

#[test]
fn a_debug_read_acknowledges_nothing_and_advances_nothing() {
    let rig = rig();
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 11);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    assert!(rig.irq.high());

    // Status, four times, under debug.
    for _ in 0..4 {
        assert_eq!(rig.peekb(7), ST_DRDY | ST_DSC | ST_DRQ);
        assert!(rig.irq.high(), "a debug read of 0x1f7 acknowledged it");
    }
    // Data, twice, under debug: the same word.
    let mut a = [0u8; 2];
    let mut b = [0u8; 2];
    rig.cmd.read(0, &mut a, MemAttrs::DEBUG).expect("legal");
    rig.cmd.read(0, &mut b, MemAttrs::DEBUG).expect("legal");
    assert_eq!(a, b, "a debug read of 0x1f0 advanced the buffer");

    // And the guest still gets the sector from its first byte.
    let mut out = Vec::new();
    for _ in 0..256 {
        out.extend_from_slice(&rig.inw().to_le_bytes());
    }
    assert_eq!(out, stamp(11));
}

#[test]
fn a_debug_write_is_refused_in_both_blocks() {
    // Neither can be made harmless: a write to offset 7 starts a command and a
    // write to the control block resets both drives.
    let rig = rig();
    for offset in 0..8 {
        assert!(
            rig.cmd.write(offset, &[0x00], MemAttrs::DEBUG).is_err(),
            "a debug write to command block offset {offset} was accepted"
        );
    }
    assert!(rig.ctl.write(0, &[0x00], MemAttrs::DEBUG).is_err());
}

#[test]
fn the_interrupt_pin_follows_nien_and_the_selected_drive() {
    let rig = rig_with(true, true);
    // The master takes a command and asks for attention.
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 3);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    assert!(rig.irq.high());

    // Selecting the other drive takes the line down, because INTRQ is driven by
    // whichever drive is selected and this one has nothing to say.
    rig.outb(6, 0xf0);
    assert!(!rig.irq.high());
    rig.outb(6, 0xe0);
    assert!(rig.irq.high(), "and coming back brings it up again");

    // nIEN in the control block gates it without losing it.
    rig.devctl(disk::CTL_NIEN);
    assert!(!rig.irq.high());
    rig.devctl(0);
    assert!(rig.irq.high());
}

#[test]
fn a_reset_through_the_control_block_reaches_both_drives() {
    let rig = rig_with(true, true);
    rig.outb(6, 0xe0);
    rig.outb(2, 0x33);
    rig.outb(6, 0xf0);
    rig.outb(2, 0x44);

    rig.devctl(disk::CTL_SRST);
    rig.devctl(0);

    // Both drives left the ATA signature, which is a sector count of one.
    assert_eq!(rig.inb(2), 0x01, "device 1 did not take the reset");
    rig.outb(6, 0xe0);
    assert_eq!(rig.inb(2), 0x01, "device 0 did not take the reset");
    assert!(!rig.irq.high(), "and a software reset raises no interrupt");
}

#[test]
fn a_channel_reset_re_drives_the_pin() {
    // The adapter has no state, so this is the whole of what its `reset` does —
    // and it matters, because a wire keeps the level it was last given.
    let rig = rig();
    rig.outb(6, 0xe0);
    rig.outb(2, 1);
    rig.outb(3, 2);
    rig.outb(4, 0);
    rig.outb(5, 0);
    rig.outb(7, disk::cmd::READ_SECTORS);
    assert!(rig.irq.high());

    rig.ide
        .drive(Position::Device0)
        .expect("a master")
        .power_on_reset();
    rig.ide.reset(ResetKind::Cold);
    assert!(!rig.irq.high(), "a reset left a stale level on the pin");
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn a_channel_will_not_take_one_bay_twice() {
    let mut props = Props::new();
    props.insert("master", "ata0");
    props.insert("slave", "ata0");
    assert!(
        Ide::new(&props).is_err(),
        "two positions on one cable cannot be the same bay"
    );
}

#[test]
fn the_default_bays_are_the_two_this_module_documents() {
    let ide = Ide::new(&Props::new()).expect("no property is required");
    assert_eq!(ide.bay_names(), [DEFAULT_MASTER_BAY, DEFAULT_SLAVE_BAY]);
    assert!(ide.drive(Position::Device0).is_none(), "and both are empty");
}
