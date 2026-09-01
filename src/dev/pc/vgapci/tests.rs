//! The PCI display adapter, checked the way a firmware checks it: read the
//! class code, size the expansion ROM, place it, enable it, read the image out.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::bus::pci::{CONFIG_PORT_WINDOW_LEN, ConfigPorts};
use crate::core::device::{Deferred, ResetKind};
use crate::core::hosts::HostObjects;
use crate::core::space::{RequesterId, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;

/// Where the card sits in these tests.
const AT: Bdf = Bdf {
    bus: 0,
    device: 2,
    function: 0,
};

/// A "video BIOS" whose only property is that every byte says where it came
/// from, so a read through the window is unambiguous.
fn image() -> Vec<u8> {
    let mut bytes = vec![0u8; 0x2800];
    bytes[0] = 0x55;
    bytes[1] = 0xaa;
    bytes[2] = 0x50;
    bytes[0x2000] = 0xa5;
    bytes
}

/// A card on a bus, its configuration ports mapped at 0xcf8 of an I/O space,
/// and its windows placed in a memory space that reads as ones where nothing
/// is — the board's shape in miniature.
struct Rig {
    mem: Arc<AddressSpace>,
    port: Arc<AddressSpace>,
    card: VgaPci,
}

impl Rig {
    fn new() -> Rig {
        Rig::with_image(&image())
    }

    fn with_image(bytes: &[u8]) -> Rig {
        let bus = Arc::new(PciBus::new());
        let card = VgaPci::with_bus(Arc::clone(&bus), AT, 0x1234, 0x1111, 0x02, bytes)
            .expect("a legal card");

        let mem = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
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

        let mut deferred = Deferred::new();
        let hosts = HostObjects::new();
        let mut ctx = RealizeCtx::new("vga", RequesterId::ANONYMOUS, &mut deferred, &hosts);
        card.realize(&mut ctx)
            .expect("it announces onto the fabric");
        deferred.drain();
        card.attach_space(&mem).expect("the windows go in");
        card.reset(ResetKind::Cold);

        Rig { mem, port, card }
    }

    /// Point `CONFADD` at one Dword of this card's configuration space.
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

    fn peek(&self, addr: u64) -> u64 {
        self.mem
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .unwrap_or(0xdead)
    }
}

#[test]
fn a_firmware_finds_a_vga_by_its_class_code() {
    let rig = Rig::new();
    assert_eq!(rig.read_u32(config::VENDOR_ID), 0x1111_1234);
    // 030000h in the top three bytes of the revision/class Dword: base class
    // 03 display, sub-class 00 VGA-compatible, programming interface 00.
    assert_eq!(rig.read_u32(config::REVISION_ID), 0x0300_0002);
    assert_eq!(rig.read_u32(config::HEADER_TYPE - 2) >> 16 & 0xff, 0x00);
    assert_eq!(
        rig.read_u16(config::COMMAND),
        0x0000,
        "and it decodes nothing until told to"
    );
}

#[test]
fn sizing_the_expansion_rom_reports_the_window_and_not_the_image() {
    // The image is 0x2800 bytes, so the window is the next power of two.
    let rig = Rig::new();
    rig.write_u32(config::EXPANSION_ROM, 0xffff_ffff);
    let mask = rig.read_u32(config::EXPANSION_ROM);
    assert_eq!(mask, 0xffff_c001, "16 KiB, and the enable bit is writable");
    assert_eq!(!(mask & 0xffff_f800) + 1, 0x4000);
}

#[test]
fn the_window_appears_where_firmware_puts_it_and_only_when_both_enables_are_set() {
    let rig = Rig::new();
    assert_eq!(rig.peek(0xfebf_0000), 0xff, "nothing decodes yet");

    // The base and the ROM enable, but not COMMAND[1] yet: §6.2.5.2 says the
    // memory space bit has precedence, so this is still nothing.
    rig.write_u32(config::EXPANSION_ROM, 0xfebf_0001);
    assert_eq!(rig.peek(0xfebf_0000), 0xff);

    rig.write_u16(config::COMMAND, config::COMMAND_MEMORY);
    assert_eq!(rig.peek(0xfebf_0000), 0x55, "the option ROM signature");
    assert_eq!(rig.peek(0xfebf_0001), 0xaa);
    assert_eq!(rig.peek(0xfebf_2000), 0xa5, "and the far end of the image");
    assert_eq!(
        rig.peek(0xfebf_3fff),
        0xff,
        "with erased bytes to the end of the window"
    );

    // Moving it moves it, which is the whole point of a base address register.
    rig.write_u32(config::EXPANSION_ROM, 0xfe80_0001);
    assert_eq!(rig.peek(0xfebf_0000), 0xff, "it left");
    assert_eq!(rig.peek(0xfe80_0000), 0x55, "and arrived");

    // Clearing either enable takes it back out of the map — and out entirely,
    // not as a hole that faults: an address no card decodes reads as ones.
    rig.write_u16(config::COMMAND, 0);
    assert_eq!(rig.peek(0xfe80_0000), 0xff);
    rig.write_u16(config::COMMAND, config::COMMAND_MEMORY);
    assert_eq!(rig.peek(0xfe80_0000), 0x55);
    rig.write_u32(config::EXPANSION_ROM, 0xfe80_0000);
    assert_eq!(rig.peek(0xfe80_0000), 0xff);
}

#[test]
fn a_card_with_no_image_has_no_expansion_rom_register() {
    // An empty media slot is an empty ROM socket, and Rev 2.1 §6.2.5.1's way
    // of saying a register is not implemented is that it reads as zero.
    let rig = Rig::with_image(&[]);
    rig.write_u32(config::EXPANSION_ROM, 0xffff_ffff);
    assert_eq!(rig.read_u32(config::EXPANSION_ROM), 0);
    assert!(rig.card.rom().is_none());
}

#[test]
fn an_unimplemented_command_bit_reads_back_as_zero() {
    // Rev 2.1 §6.2.2 lets a function hardwire to zero what it does not
    // implement, and this one implements four bits.
    let rig = Rig::new();
    rig.write_u16(config::COMMAND, 0xffff);
    assert_eq!(rig.read_u16(config::COMMAND), COMMAND_IMPLEMENTED);
}

#[test]
fn a_debug_write_cannot_move_a_window() {
    // The invariant `MemAttrs::debug` exists for: a monitor poking at
    // configuration space must not remap the guest's memory under it.
    let rig = Rig::new();
    rig.write_u32(config::EXPANSION_ROM, 0xfebf_0001);
    rig.write_u16(config::COMMAND, config::COMMAND_MEMORY);
    assert_eq!(rig.peek(0xfebf_0000), 0x55);

    rig.card
        .regs
        .config_write(config::EXPANSION_ROM, &0xfe80_0001u32.to_le_bytes(), {
            let mut attrs = MemAttrs::DEFAULT;
            attrs.debug = true;
            attrs
        });
    assert_eq!(
        rig.read_u32(config::EXPANSION_ROM),
        0xfebf_0001,
        "the register did not move"
    );
    assert_eq!(rig.peek(0xfebf_0000), 0x55, "and neither did the window");

    // A debug *read* is harmless and is allowed, which is the other half of
    // the rule.
    let mut dst = [0u8; 4];
    rig.card.regs.config_read(config::VENDOR_ID, &mut dst, {
        let mut attrs = MemAttrs::DEFAULT;
        attrs.debug = true;
        attrs
    });
    assert_eq!(u32::from_le_bytes(dst), 0x1111_1234);
}

#[test]
fn a_reset_takes_the_window_out_of_the_map() {
    let rig = Rig::new();
    rig.write_u32(config::EXPANSION_ROM, 0xfebf_0001);
    rig.write_u16(config::COMMAND, config::COMMAND_MEMORY);
    assert_eq!(rig.peek(0xfebf_0000), 0x55);
    // `PCIRST#`: every enable and every base goes, which is the state firmware
    // expects to find when it starts enumerating.
    rig.card.reset(ResetKind::Warm);
    assert_eq!(rig.read_u32(config::EXPANSION_ROM), 0);
    assert_eq!(rig.read_u16(config::COMMAND), 0);
    assert_eq!(rig.peek(0xfebf_0000), 0xff);
}

/// One snapshot chunk holding this card's state.
fn snapshot(card: &VgaPci) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("vga", CLASS_NAME).expect("unique path");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("vga", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        card.save(&mut chunk).expect("saves");
    }
    w.to_vec().expect("encodes")
}

#[test]
fn the_state_round_trips_byte_for_byte() {
    let a = Rig::new();
    a.write_u32(config::EXPANSION_ROM, 0xfebf_0001);
    a.write_u16(config::COMMAND, config::COMMAND_MEMORY | config::COMMAND_IO);
    a.write_u32(config::INTERRUPT_LINE & !3, 0x0000_000b);
    let saved = snapshot(&a.card);

    let b = Rig::new();
    let reader = StateReader::new(&saved).expect("it parses");
    let chunk = reader
        .load("vga", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    b.card.load(&mut chunk.reader()).expect("it loads");

    assert_eq!(
        snapshot(&b.card),
        saved,
        "a reload saves byte-identically, which is what a state hash is"
    );
    // And the thing that is *not* saved and has to be rebuilt: where the
    // window went.
    assert_eq!(b.peek(0xfebf_0000), 0x55);
    assert_eq!(b.card.command(), a.card.command());
}

#[test]
fn the_class_constructs_from_properties() {
    let mut props = Props::new();
    props.insert("image", crate::core::props::Media::new("vgabios", image()));
    props.insert("device", crate::core::props::Value::Uint(3));
    let dev = (CLASS.construct)(&props).expect("a card");
    assert_eq!(dev.class().name, CLASS_NAME);

    // And a property this class does not know is an error naming it, rather
    // than a card that quietly ignores what the machine file asked for.
    let mut props = Props::new();
    props.insert("image", crate::core::props::Media::new("vgabios", image()));
    props.insert("nonsense", crate::core::props::Value::Uint(1));
    assert!((CLASS.construct)(&props).is_err());
}

#[test]
fn the_rom_window_is_read_only_and_swallows_a_write() {
    // A write to a ROM does nothing on real hardware, and firmware writes all
    // ones into windows while it sizes them.
    let rig = Rig::new();
    rig.write_u32(config::EXPANSION_ROM, 0xfebf_0001);
    rig.write_u16(config::COMMAND, config::COMMAND_MEMORY);
    assert_eq!(
        rig.mem.write(0xfebf_0000, Width::U8, 0, MemAttrs::DEFAULT),
        Err(crate::core::error::BusError::Protected),
        "the window is mapped read-only, so the space refuses before the ROM sees it"
    );
    assert_eq!(rig.peek(0xfebf_0000), 0x55, "and the image is unchanged");
    assert_eq!(
        rig.card
            .bars()
            .spec(Bars::ROM)
            .map(crate::bus::pci::Bar::len),
        Some(0x4000)
    );
}
