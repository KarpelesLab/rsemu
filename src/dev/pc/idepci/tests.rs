//! The native-mode IDE controller, checked the way a driver checks it: read the
//! class code, size the four registers, place them, enable I/O decode, and
//! drive a drive at the addresses the registers name.
//!
//! Everything below goes through a real I/O address space, so every
//! configuration write is an `OUT` to `0xcfc` travelling through the very space
//! the windows live in. That is the whole point: nothing here can be placed by
//! the try-lock, and every window this file sees appear was placed by
//! [`PciBus::settle`], which stands in for the host bridge's scheduler drain.

use super::*;

use alloc::string::String;
use alloc::vec::Vec;

use crate::bus::pci::{CONFIG_PORT_WINDOW_LEN, ConfigPorts};
use crate::core::device::Deferred;
use crate::core::hosts::HostObjects;
use crate::core::space::{Region, RequesterId, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;
use crate::dev::ata::bays::Bay;
use crate::dev::ata::disk::{AtaDisk, Identity, Position, SECTOR, default_geometry};
use crate::dev::pc::ide::Ide;

/// Where the controller sits in these tests. Device 1 function 1 is where a
/// PIIX-lineage part puts its IDE function, and the number is arbitrary here.
const AT: Bdf = Bdf {
    bus: 0,
    device: 1,
    function: 1,
};

/// Command-block offsets a driver uses (ATA/ATAPI-6 §7): sector count, the
/// three LBA bytes, the device register and status.
const REG_SECTOR_COUNT: u64 = 2;
const REG_LBA_LOW: u64 = 3;
const REG_DEVICE: u64 = 6;
const REG_STATUS: u64 = 7;

/// A drive whose every sector says which sector it is.
fn stamped() -> Arc<AtaDisk> {
    let id = Identity::new(64, default_geometry(64), true, 16).expect("a valid drive");
    let disk = AtaDisk::with_identity(id, Position::Device0).expect("it fits in host memory");
    for lba in 0..64u64 {
        let mut sector = alloc::vec![0u8; SECTOR as usize];
        sector[0] = lba as u8;
        disk.write_media(lba * SECTOR, &sector).expect("in range");
    }
    Arc::new(disk)
}

/// A controller on a bus, its configuration ports mapped at `0xcf8` of an I/O
/// space, and its windows placed in **that same space** — which is the board's
/// shape in miniature and the condition the deferral exists for.
struct Rig {
    port: Arc<AddressSpace>,
    bus: Arc<PciBus>,
    card: IdePci,
    /// The channels, kept alive: a machine owns its devices and this rig is
    /// standing in for one.
    _channels: Vec<Ide>,
}

impl Rig {
    fn new() -> Rig {
        Rig::with_channels(true, false)
    }

    fn with_channels(primary: bool, secondary: bool) -> Rig {
        let bus = Arc::new(PciBus::new());
        let card = IdePci::with_bus(
            Arc::clone(&bus),
            AT,
            0x1234,
            0x1230,
            0x02,
            [primary, secondary],
        )
        .expect("a legal controller");

        let port = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
        let ports = Arc::new(ConfigPorts::new(Arc::clone(&bus)));
        port.topology()
            .map(
                Region::io(
                    "config",
                    CONFIG_PORT_WINDOW_LEN,
                    Arc::clone(&ports) as Arc<dyn crate::core::space::MemOps>,
                ),
                0xcf8,
            )
            .expect("0xcf8 is free");

        let mut channels = Vec::new();
        for (index, fitted) in [primary, secondary].into_iter().enumerate() {
            if !fitted {
                continue;
            }
            let bays = [Arc::new(Bay::new()), Arc::new(Bay::new())];
            bays[0].fit(stamped()).expect("an empty bay");
            let ide = Ide::with_bays(
                [Arc::clone(&bays[0]), Arc::clone(&bays[1])],
                [String::from("ata0"), String::from("ata1")],
            );
            card.attach_command(index, ide.region("cmd").expect("the command block"))
                .expect("the register is free");
            card.attach_control(index, ide.region("ctl").expect("the control block"))
                .expect("the register is free");
            channels.push(ide);
        }

        let mut deferred = Deferred::new();
        let hosts = HostObjects::new();
        let mut ctx = RealizeCtx::new("ide-pci", RequesterId::ANONYMOUS, &mut deferred, &hosts);
        card.realize(&mut ctx)
            .expect("it announces onto the fabric");
        deferred.drain();
        card.attach_space(&port).expect("the windows go in");

        Rig {
            port,
            bus,
            card,
            _channels: channels,
        }
    }

    /// Point `CONFADD` at one Dword of this controller's configuration space.
    fn select(&self, register: u16) {
        let addr = 0x8000_0000u64
            | (u64::from(AT.device) << 11)
            | (u64::from(AT.function) << 8)
            | u64::from(register & 0xfc);
        self.port
            .write(0xcf8, Width::U32, addr, MemAttrs::DEFAULT)
            .expect("a Dword write to CONFADD");
    }

    fn read_u32(&self, register: u16) -> u32 {
        self.select(register);
        self.port
            .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
            .expect("a Dword read of CONFDATA") as u32
    }

    fn write_u32(&self, register: u16, value: u32) {
        self.select(register);
        self.port
            .write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("a Dword write to CONFDATA");
    }

    fn read_u16(&self, register: u16) -> u16 {
        self.select(register);
        self.port
            .read(
                0xcfc + u64::from(register & 3),
                Width::U16,
                MemAttrs::DEFAULT,
            )
            .expect("a word read of CONFDATA") as u16
    }

    fn write_u16(&self, register: u16, value: u16) {
        self.select(register);
        self.port
            .write(
                0xcfc + u64::from(register & 3),
                Width::U16,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a word write to CONFDATA");
    }

    /// What the host bridge's `Device::advance_to` does once a round: place
    /// whatever a configuration write could not place from inside itself.
    fn drain(&self) {
        assert!(!self.bus.settle(), "one sweep is enough with nothing held");
    }

    fn inb(&self, port: u64) -> u8 {
        self.port
            .read(port, Width::U8, MemAttrs::DEFAULT)
            .expect("an I/O read is never a fault here") as u8
    }

    fn outb(&self, port: u64, value: u8) {
        self.port
            .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("an I/O write is never a fault here");
    }

    /// Size one base address register the way firmware does: all ones in, read
    /// the mask back, and put the original value back.
    fn size(&self, register: u16) -> u32 {
        let saved = self.read_u32(register);
        self.write_u32(register, 0xffff_ffff);
        let mask = self.read_u32(register);
        self.write_u32(register, saved);
        mask
    }

    /// Place both of the primary channel's windows and turn the decode on.
    fn place_primary(&self, command_base: u16, control_base: u16) {
        self.write_u32(config::BAR0, u32::from(command_base));
        self.write_u32(config::BAR0 + 4, u32::from(control_base));
        self.write_u16(config::COMMAND, config::COMMAND_IO);
        self.drain();
    }
}

#[test]
fn a_driver_finds_a_native_mode_ide_controller_by_its_class_code() {
    let rig = Rig::new();
    assert_eq!(rig.read_u32(config::VENDOR_ID), 0x1230_1234);
    // 010105h in the top three bytes of the revision/class Dword: base class 01
    // mass storage, sub-class 01 IDE, programming interface 05 — native on both
    // channels, switchable on neither, no bus master (Rev 2.1 Appendix D).
    assert_eq!(rig.read_u32(config::REVISION_ID), 0x0101_0502);
    // §6.2.1 header type 00h, single function; §6.2.4 no interrupt pin, because
    // there is no PIRQ router on this board for one to reach.
    assert_eq!(rig.read_u32(config::CACHE_LINE_SIZE) >> 16 & 0xff, 0x00);
    assert_eq!(rig.read_u32(config::INTERRUPT_LINE) & 0xff_00, 0x0000);
    assert_eq!(
        rig.read_u32(config::INTERRUPT_LINE) & 0xff,
        0xff,
        "255 is §6.2.4's 'unknown, or no connection'"
    );
}

#[test]
fn sizing_the_registers_reports_the_windows_the_specification_gives_them() {
    let rig = Rig::with_channels(true, true);
    // §6.2.5.1: the address bits below the window size are hardwired to zero,
    // and bit 0 marks the register as an I/O one.
    assert_eq!(rig.size(config::BAR0), 0xffff_fff9, "eight ports");
    assert_eq!(rig.size(config::BAR0 + 4), 0xffff_fffd, "four ports");
    assert_eq!(rig.size(config::BAR0 + 8), 0xffff_fff9);
    assert_eq!(rig.size(config::BAR0 + 12), 0xffff_fffd);
    // §6.2.5.1 again: an unimplemented register reads as all zeroes, which is
    // how firmware knows to stop looking. There is no bus-master window here.
    assert_eq!(rig.size(config::BAR0 + 16), 0);
    assert_eq!(rig.size(config::BAR0 + 20), 0);
}

#[test]
fn a_channel_that_is_not_fitted_has_no_registers_at_all() {
    let rig = Rig::new();
    assert_ne!(rig.size(config::BAR0), 0, "the primary is fitted");
    assert_eq!(
        rig.size(config::BAR0 + 8),
        0,
        "and the secondary is not, so its registers are not implemented"
    );
    assert_eq!(rig.size(config::BAR0 + 12), 0);
}

#[test]
fn a_controller_with_no_channel_at_all_is_a_machine_description_mistake() {
    let e = IdePci::new(&Props::new())
        .expect_err("neither channel named")
        .to_string();
    assert!(e.contains("primary"), "{e}");
}

/// **The end-to-end shape of the whole change, in one function.**
///
/// A driver sizes a register, places it, enables the decode, drives the drive
/// at the address it chose — then *moves* the window and drives it again at the
/// new address. Every configuration write here is an `OUT` into the space the
/// window lives in, so not one of them could place anything by itself.
#[test]
fn the_windows_follow_the_registers_and_the_drive_answers_at_both() {
    let rig = Rig::new();

    // Nothing decodes out of reset: §6.2.2's I/O space bit is clear and the
    // bases are zero.
    assert_eq!(rig.read_u16(config::COMMAND), 0);
    assert_eq!(rig.inb(0x1f0 + REG_STATUS), 0xff, "an empty I/O space");

    rig.place_primary(0x1f0, 0x3f4);
    // The device register with DEV clear selects device 0, which is fitted, and
    // a fitted drive is ready. ATA/ATAPI-6 §7.15: DRDY set, BSY clear.
    rig.outb(0x1f0 + REG_DEVICE, 0xa0);
    let status = rig.inb(0x1f0 + REG_STATUS);
    assert_ne!(status, 0xff, "something is decoding there now");
    assert_ne!(status, 0x00, "and it is a drive that is present");

    // A command-block register is a latch, which is the unambiguous proof that
    // a write and a read reached the same drive through the same window.
    rig.outb(0x1f0 + REG_SECTOR_COUNT, 0x2a);
    rig.outb(0x1f0 + REG_LBA_LOW, 0x17);
    assert_eq!(rig.inb(0x1f0 + REG_SECTOR_COUNT), 0x2a);
    assert_eq!(rig.inb(0x1f0 + REG_LBA_LOW), 0x17);

    // The control block's one decoded byte is at offset 2 of a four-byte
    // window, so a driver computes BAR1 + 2 — which lands on 0x3f6, the port a
    // compatibility-mode driver would have used, because that is what the
    // control base was set to minus two.
    assert_eq!(
        rig.inb(0x3f6),
        rig.inb(0x1f0 + REG_STATUS),
        "alternate status is the same eight bits with no acknowledge attached"
    );
    assert_eq!(
        rig.inb(0x3f4),
        0xff,
        "and nothing else in the window decodes"
    );
    assert_eq!(rig.inb(0x3f5), 0xff);
    assert_eq!(rig.inb(0x3f7), 0xff);

    // **The move.** Firmware relocates the controller, and the old ports go
    // dead while the new ones come alive — with the latched registers intact,
    // because the drive never noticed.
    rig.write_u32(config::BAR0, 0x0000_0170);
    rig.write_u32(config::BAR0 + 4, 0x0000_0374);
    assert_eq!(
        rig.inb(0x1f0 + REG_SECTOR_COUNT),
        0x2a,
        "until the drain runs, the old mapping is still the one in the map"
    );
    rig.drain();
    assert_eq!(rig.inb(0x1f0 + REG_SECTOR_COUNT), 0xff, "it left");
    assert_eq!(rig.inb(0x170 + REG_SECTOR_COUNT), 0x2a, "and arrived");
    assert_eq!(rig.inb(0x170 + REG_LBA_LOW), 0x17);
    assert_eq!(rig.inb(0x376), rig.inb(0x170 + REG_STATUS));

    // And §6.2.2's enable gates all of it without moving anything.
    rig.write_u16(config::COMMAND, 0);
    rig.drain();
    assert_eq!(rig.inb(0x170 + REG_SECTOR_COUNT), 0xff);
    assert_eq!(rig.inb(0x376), 0xff);
    rig.write_u16(config::COMMAND, config::COMMAND_IO);
    rig.drain();
    assert_eq!(rig.inb(0x170 + REG_SECTOR_COUNT), 0x2a);
}

#[test]
fn a_write_that_moves_nothing_asks_nothing_of_the_address_space() {
    // Firmware writes all-ones to size a register and the base straight after,
    // with COMMAND[0] still clear. Nothing decodes before or after, so nothing
    // is owed — a controller that asked the scheduler to come back once per
    // configuration write would cost a wake-up per register of every
    // enumeration.
    let rig = Rig::new();
    assert!(!rig.bus.retopology_owed());
    let _ = rig.size(config::BAR0);
    let _ = rig.size(config::BAR0 + 4);
    assert!(
        !rig.bus.retopology_owed(),
        "sizing a register with the decode off places nothing"
    );
}

#[test]
fn a_reset_takes_the_controller_back_to_deciding_nothing() {
    let rig = Rig::new();
    rig.place_primary(0x1f0, 0x3f4);
    assert_ne!(rig.inb(0x1f0 + REG_STATUS), 0xff);
    // `PCIRST#`: every enable and every base goes, which is the state firmware
    // expects to find when it starts enumerating — and the reason a warm reset
    // does not leave the ports where the previous guest put them.
    rig.card.reset(ResetKind::Warm);
    assert_eq!(
        rig.read_u32(config::BAR0),
        0x0000_0001,
        "only the marker bit"
    );
    assert_eq!(rig.read_u16(config::COMMAND), 0);
    assert_eq!(rig.inb(0x1f0 + REG_STATUS), 0xff);
}

#[test]
fn a_debug_configuration_write_cannot_move_a_window() {
    // `MemAttrs::debug` exists so a monitor can look without changing anything,
    // and a BAR write is the sharpest possible counterexample.
    let rig = Rig::new();
    rig.place_primary(0x1f0, 0x3f4);
    rig.select(config::BAR0);
    // `ConfigPorts` refuses one outright, because moving the address latch is
    // as unsafe for a debugger as moving a window.
    rig.port
        .write(0xcfc, Width::U32, 0x0000_0170, MemAttrs::DEBUG)
        .expect_err("the port pair refuses a debug write");
    // And this is the second lock on the same door, for a caller that reaches
    // the function directly.
    rig.bus.config_write(
        AT,
        config::BAR0,
        &0x0000_0170u32.to_le_bytes(),
        MemAttrs::DEBUG,
    );
    rig.drain();
    assert_eq!(
        rig.read_u32(config::BAR0) & !1,
        0x0000_01f0,
        "the latch did not move"
    );
    assert_ne!(
        rig.inb(0x1f0 + REG_STATUS),
        0xff,
        "and neither did the window"
    );
}

/// One snapshot chunk holding this controller's state.
fn snapshot(card: &IdePci) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape
        .add_device("ide-pci", CLASS_NAME)
        .expect("unique path");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("ide-pci", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        card.save(&mut chunk).expect("saves");
    }
    w.to_vec().expect("encodes")
}

#[test]
fn the_state_round_trips_byte_for_byte() {
    let a = Rig::new();
    a.place_primary(0x1f0, 0x3f4);
    a.write_u32(config::INTERRUPT_LINE & !3, 0x0000_000e);
    let saved = snapshot(&a.card);

    let b = Rig::new();
    let reader = StateReader::new(&saved).expect("it parses");
    let chunk = reader
        .load("ide-pci", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    b.card.load(&mut chunk.reader()).expect("it loads");

    assert_eq!(
        snapshot(&b.card),
        saved,
        "a reload saves byte-identically, which is what a state hash is"
    );
    // And the thing that is *not* saved and has to be rebuilt: where the
    // windows went. `load` uses the blocking guard, because a snapshot load
    // runs with no access in flight — so nothing is owed afterwards.
    assert!(!b.bus.retopology_owed());
    assert_ne!(b.inb(0x1f0 + REG_STATUS), 0xff);
    assert_eq!(b.card.command(), a.card.command());
}
