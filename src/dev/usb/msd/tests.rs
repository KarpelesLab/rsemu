//! The mass storage device, driven the way a host driver drives it.
//!
//! Every test here goes through the **fabric** — [`UsbBus::setup`],
//! [`UsbBus::read`], [`UsbBus::write`] — rather than calling a method on the
//! function, because the thing worth testing is that a sequence of transactions
//! a real driver would issue is one this device answers. The enumeration half
//! uses [`crate::bus::usb::ControlTransfer`], which is the host-side composer
//! the fabric already ships for exactly this.
//!
//! Data is asserted **against the medium** and never against the device's own
//! buffer, on the same principle `tests/nvme_board.rs` and `tests/ahci_board.rs`
//! state: a model with a buffer and no medium would pass a test that only
//! looked at what came back.

use super::*;

use crate::bus::usb::{ControlTransfer, DeviceAddress, Progress, Status, UsbBus, host};
use crate::core::space::RamStore;

/// Bytes in a logical block, for every test here.
const BLOCK: u64 = 512;
/// How many of them the test disk holds.
const BLOCKS: u64 = 64;
/// The address the host gives the device.
const ADDRESS: DeviceAddress = DeviceAddress(7);

/// A recognisable block: every byte says which block it came from and where in
/// it that byte sits, so a read of the wrong LBA and a read of the right LBA at
/// the wrong offset look different.
fn stamp(lba: u64) -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (lba as u8).wrapping_mul(17).wrapping_add(i as u8))
        .collect()
}

/// A disk whose every block is stamped, plus the medium behind it.
fn disk() -> (Arc<RamStore>, UsbStorage) {
    let store = Arc::new(RamStore::new(BLOCKS * BLOCK));
    for lba in 0..BLOCKS {
        RamStore::write_at(&store, lba * BLOCK, &stamp(lba)).expect("a RamStore takes bytes");
    }
    let disk = UsbStorage::with_medium(
        Arc::clone(&store) as Arc<dyn Medium>,
        BLOCK,
        false,
        true,
        Speed::High,
        (0x0781, 0x5567),
        ("RSEMU", "USB DISK", "1.00", "0123456789AB"),
    );
    (store, disk)
}

/// Drive one control transfer to completion, or panic saying how it failed.
fn run(bus: &UsbBus, address: DeviceAddress, mut xfer: ControlTransfer) -> Vec<u8> {
    for _ in 0..256 {
        match xfer.step(bus, address, 64) {
            Progress::Moved | Progress::Nak => {}
            Progress::Done => return xfer.take_data(),
            Progress::Failed(status) => panic!("the control transfer failed: {status}"),
        }
    }
    panic!("the control transfer never finished");
}

/// The same, but reporting the failure rather than panicking on it.
fn try_run(
    bus: &UsbBus,
    address: DeviceAddress,
    mut xfer: ControlTransfer,
) -> core::result::Result<Vec<u8>, Status> {
    for _ in 0..256 {
        match xfer.step(bus, address, 64) {
            Progress::Moved | Progress::Nak => {}
            Progress::Done => return Ok(xfer.take_data()),
            Progress::Failed(status) => return Err(status),
        }
    }
    panic!("the control transfer never finished");
}

/// A disk on a bus, enumerated: addressed, configured, ready for a CBW.
fn enumerated() -> (Arc<UsbBus>, Arc<RamStore>, UsbStorage) {
    let (store, disk) = disk();
    let bus = Arc::new(UsbBus::new(1));
    bus.attach(0, disk.device()).expect("an empty port");
    bus.set_enabled(0, true);
    run(
        &bus,
        DeviceAddress::DEFAULT,
        ControlTransfer::host_to_device(host::set_address(ADDRESS), &[]),
    );
    run(
        &bus,
        ADDRESS,
        ControlTransfer::host_to_device(host::set_configuration(1), &[]),
    );
    assert_eq!(disk.address(), ADDRESS);
    assert_eq!(disk.configuration(), 1);
    (bus, store, disk)
}

// ---------------------------------------------------------------------------
// The host side of Bulk-Only Transport
// ---------------------------------------------------------------------------

/// A CSW as the host decodes it (BOT §5.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HostCsw {
    signature: u32,
    tag: u32,
    residue: u32,
    status: u8,
}

/// The three phases, from the host's side of the wire.
struct Bot<'a> {
    bus: &'a UsbBus,
    mps: usize,
    tag: u32,
}

impl<'a> Bot<'a> {
    fn new(bus: &'a UsbBus, mps: u16) -> Bot<'a> {
        Bot {
            bus,
            mps: usize::from(mps),
            tag: 0x1000_0000,
        }
    }

    /// Build and send a CBW (§5.1). Returns the handshake the device gave.
    fn command(&mut self, data_length: u32, flags: u8, cdb: &[u8]) -> Status {
        self.tag = self.tag.wrapping_add(1);
        let mut cbw = [0u8; CBW_BYTES];
        cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        cbw[4..8].copy_from_slice(&self.tag.to_le_bytes());
        cbw[8..12].copy_from_slice(&data_length.to_le_bytes());
        cbw[12] = flags;
        cbw[13] = 0;
        cbw[14] = cdb.len() as u8;
        cbw[15..15 + cdb.len()].copy_from_slice(cdb);
        self.bus.write(ADDRESS, ENDPOINT_OUT, &cbw).status
    }

    /// Collect the data phase: packets until a short one, or until the host has
    /// what it asked for (USB 2.0 §5.8.3).
    fn data_in(&self, want: usize) -> (Vec<u8>, Status) {
        let mut out = Vec::new();
        while out.len() < want {
            let n = self.mps.min(want - out.len());
            let mut buf = alloc::vec![0u8; n];
            let done = self.bus.read(ADDRESS, ENDPOINT_IN, &mut buf);
            if done.status != Status::Ack {
                return (out, done.status);
            }
            let moved = done.len as usize;
            out.extend_from_slice(&buf[..moved]);
            if moved < n {
                break;
            }
        }
        (out, Status::Ack)
    }

    /// Send the data phase, a packet at a time. §6.7.3: the host never sends a
    /// zero-length packet, and sends a short one only at the end.
    fn data_out(&self, bytes: &[u8]) -> (usize, Status) {
        let mut sent = 0usize;
        while sent < bytes.len() {
            let n = self.mps.min(bytes.len() - sent);
            let done = self
                .bus
                .write(ADDRESS, ENDPOINT_OUT, &bytes[sent..sent + n]);
            if done.status != Status::Ack {
                return (sent, done.status);
            }
            sent += done.len as usize;
        }
        (sent, Status::Ack)
    }

    /// Read the CSW (§5.3.3).
    fn status(&self) -> (HostCsw, Status) {
        let mut buf = [0u8; CSW_BYTES];
        let done = self.bus.read(ADDRESS, ENDPOINT_IN, &mut buf);
        if done.status != Status::Ack {
            return (
                HostCsw {
                    signature: 0,
                    tag: 0,
                    residue: 0,
                    status: 0xff,
                },
                done.status,
            );
        }
        assert_eq!(done.len as usize, CSW_BYTES, "a CSW is thirteen bytes");
        (
            HostCsw {
                signature: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
                tag: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
                residue: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
                status: buf[12],
            },
            Status::Ack,
        )
    }

    /// A whole device-to-host command: CBW, data, CSW.
    fn read_command(&mut self, want: u32, cdb: &[u8]) -> (Vec<u8>, HostCsw) {
        assert_eq!(self.command(want, CBW_FLAG_IN, cdb), Status::Ack);
        let (data, status) = self.data_in(want as usize);
        assert_eq!(status, Status::Ack, "the data phase stalled");
        let (csw, status) = self.status();
        assert_eq!(status, Status::Ack, "the status phase stalled");
        assert_eq!(csw.signature, CSW_SIGNATURE);
        assert_eq!(csw.tag, self.tag, "the CSW must echo the CBW's tag (§5.2)");
        (data, csw)
    }

    /// A whole host-to-device command.
    fn write_command(&mut self, payload: &[u8], cdb: &[u8]) -> HostCsw {
        assert_eq!(self.command(payload.len() as u32, 0, cdb), Status::Ack);
        let (sent, status) = self.data_out(payload);
        assert_eq!(status, Status::Ack, "the data phase stalled");
        assert_eq!(sent, payload.len());
        let (csw, status) = self.status();
        assert_eq!(status, Status::Ack);
        assert_eq!(csw.tag, self.tag);
        csw
    }

    /// A command with no data phase at all (case 1).
    fn plain_command(&mut self, cdb: &[u8]) -> HostCsw {
        assert_eq!(self.command(0, 0, cdb), Status::Ack);
        let (csw, status) = self.status();
        assert_eq!(status, Status::Ack);
        assert_eq!(csw.tag, self.tag);
        csw
    }
}

/// A `READ (10)` command block (Seagate §3.16, table 97).
fn read10(lba: u32, blocks: u16) -> [u8; 10] {
    let lba = lba.to_be_bytes();
    let blocks = blocks.to_be_bytes();
    [
        opcode::READ_10,
        0,
        lba[0],
        lba[1],
        lba[2],
        lba[3],
        0,
        blocks[0],
        blocks[1],
        0,
    ]
}

/// A `WRITE (10)` command block (Seagate §3.60).
fn write10(lba: u32, blocks: u16) -> [u8; 10] {
    let mut cdb = read10(lba, blocks);
    cdb[0] = opcode::WRITE_10;
    cdb
}

/// What the medium itself holds at `lba` — the only assertion that proves a
/// byte moved rather than being echoed.
fn on_medium(store: &RamStore, lba: u64) -> Vec<u8> {
    let mut got = alloc::vec![0u8; BLOCK as usize];
    Medium::read_at(store, lba * BLOCK, &mut got).expect("the medium reads");
    got
}

// ---------------------------------------------------------------------------
// Descriptors and enumeration
// ---------------------------------------------------------------------------

#[test]
fn the_interface_is_the_one_bulk_only_transport_asks_for() {
    let (bus, _store, _disk) = enumerated();
    let tree = run(
        &bus,
        ADDRESS,
        ControlTransfer::device_to_host(host::get_descriptor(2, 0, 64)),
    );
    // Configuration descriptor, then the interface, then two endpoints.
    assert_eq!(tree[0], 9, "a configuration descriptor is nine bytes");
    let interface = &tree[9..18];
    assert_eq!(interface[1], 4, "descriptor type INTERFACE");
    assert_eq!(interface[4], 2, "bNumEndpoints: at least two (BOT §4.3)");
    assert_eq!(interface[5], CLASS_MASS_STORAGE, "bInterfaceClass 08h");
    assert_eq!(interface[6], SUBCLASS_SCSI, "bInterfaceSubClass 06h");
    assert_eq!(interface[7], PROTOCOL_BULK_ONLY, "bInterfaceProtocol 50h");

    let bulk_in = &tree[18..25];
    assert_eq!(bulk_in[2], ENDPOINT_IN | 0x80, "bEndpointAddress, IN");
    assert_eq!(bulk_in[3] & 0x3, 2, "bmAttributes says bulk (§4.4.1)");
    assert_eq!(u16::from_le_bytes([bulk_in[4], bulk_in[5]]), 512);
    let bulk_out = &tree[25..32];
    assert_eq!(bulk_out[2], ENDPOINT_OUT, "bEndpointAddress, OUT");
    assert_eq!(bulk_out[3] & 0x3, 2);
}

#[test]
fn the_device_descriptor_puts_the_class_on_the_interface_and_names_a_serial() {
    let (bus, _store, _disk) = enumerated();
    let device = run(
        &bus,
        ADDRESS,
        ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18)),
    );
    // BOT §4.1: the class codes live in the interface descriptor, not here.
    assert_eq!(device[4], 0, "bDeviceClass");
    assert_eq!(device[5], 0, "bDeviceSubClass");
    assert_eq!(device[6], 0, "bDeviceProtocol");
    // §4.1.1: `iSerialNumber` shall index a string descriptor.
    let index = device[16];
    assert_ne!(index, 0, "BOT §4 requires a serial number string");
    let serial = run(
        &bus,
        ADDRESS,
        ControlTransfer::device_to_host(host::get_descriptor(3, index, 64)),
    );
    let text: String = serial[2..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| char::from(c[0]))
        .collect();
    assert_eq!(text, "0123456789AB");
    assert!(text.len() >= 12, "§4.1.1 asks for at least twelve digits");
}

#[test]
fn get_max_lun_answers_zero_rather_than_stalling() {
    let (bus, _store, _disk) = enumerated();
    // BOT §3.2, table 3.2: class, interface, device to host, wLength 1.
    let setup = SetupPacket {
        request_type: 0xa1,
        request: class_request::GET_MAX_LUN,
        value: 0,
        index: INTERFACE,
        length: 1,
    };
    let data = run(&bus, ADDRESS, ControlTransfer::device_to_host(setup));
    assert_eq!(data, alloc::vec![0], "one logical unit, numbered zero");
}

#[test]
fn a_class_request_to_the_wrong_recipient_stalls() {
    let (bus, _store, _disk) = enumerated();
    // The same request addressed to the *device* rather than the interface.
    let setup = SetupPacket {
        request_type: 0xa0,
        request: class_request::GET_MAX_LUN,
        value: 0,
        index: INTERFACE,
        length: 1,
    };
    assert_eq!(
        try_run(&bus, ADDRESS, ControlTransfer::device_to_host(setup)),
        Err(Status::Stall)
    );
}

#[test]
fn a_low_speed_disk_is_refused_at_construction() {
    // USB 2.0 §5.8: there are no bulk transfers at low speed, so a low-speed
    // Bulk-Only device is not a device that enumerates badly — it is one that
    // cannot exist.
    let props = crate::core::props::Props::new()
        .with("bus", "usb-low")
        .with("size", 32768u64)
        .with("speed", "low");
    let error = UsbStorage::new(&props).expect_err("a low-speed disk is a configuration error");
    assert!(
        alloc::format!("{error}").contains("low speed"),
        "the message should say why: {error}"
    );
}

// ---------------------------------------------------------------------------
// The command set, against the medium
// ---------------------------------------------------------------------------

#[test]
fn a_block_read_comes_off_the_medium() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let (data, csw) = bot.read_command(BLOCK as u32, &read10(9, 1));
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 0, "case (6): the thin diagonal");
    // Against the medium, not against anything the device kept.
    assert_eq!(data, on_medium(&store, 9));
    // And it is not the neighbouring block, which is what catches a length
    // computed in blocks and applied in bytes.
    assert_ne!(data, on_medium(&store, 10));
    assert_eq!(on_medium(&store, 8), stamp(8), "the neighbour is untouched");
    assert_eq!(on_medium(&store, 10), stamp(10));
}

#[test]
fn a_multi_block_read_spans_packets_and_stays_in_order() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // Four blocks is four 512-byte packets at a 512-byte maximum packet size,
    // so this is the first thing that would break if the cursor were not
    // carried between transactions.
    let want = 4 * BLOCK;
    let (data, csw) = bot.read_command(want as u32, &read10(20, 4));
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 0);
    let mut expected = Vec::new();
    for lba in 20..24 {
        expected.extend_from_slice(&on_medium(&store, lba));
    }
    assert_eq!(data, expected);
}

#[test]
fn a_block_write_reaches_the_medium_and_leaves_its_neighbours_alone() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let payload: Vec<u8> = (0..BLOCK).map(|i| (i as u8) ^ 0x5a).collect();
    let csw = bot.write_command(&payload, &write10(33, 1));
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 0);

    assert_eq!(
        on_medium(&store, 33),
        payload,
        "the write missed the medium"
    );
    assert_eq!(on_medium(&store, 32), stamp(32));
    assert_eq!(on_medium(&store, 34), stamp(34));
}

#[test]
fn read_capacity_reports_the_last_block_and_not_the_count() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let (data, csw) = bot.read_command(8, &[opcode::READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(csw.status, status::PASSED);
    // Seagate §3.22.2, table 120: RETURNED LOGICAL BLOCK ADDRESS is the *last*
    // one, so a disk of 64 blocks reports 63. Off by one here and every host
    // reads one block past the end.
    assert_eq!(
        u32::from_be_bytes([data[0], data[1], data[2], data[3]]),
        (BLOCKS - 1) as u32
    );
    assert_eq!(
        u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
        BLOCK as u32
    );
}

#[test]
fn read_capacity_16_reports_the_same_disk_in_sixty_four_bits() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let mut cdb = [0u8; 16];
    cdb[0] = opcode::SERVICE_ACTION_IN_16;
    cdb[1] = SERVICE_ACTION_READ_CAPACITY_16;
    cdb[10..14].copy_from_slice(&32u32.to_be_bytes());
    let (data, csw) = bot.read_command(32, &cdb);
    assert_eq!(csw.status, status::PASSED);
    let last = u64::from_be_bytes(data[..8].try_into().expect("eight bytes"));
    assert_eq!(last, BLOCKS - 1);
    assert_eq!(
        u32::from_be_bytes(data[8..12].try_into().expect("four bytes")),
        BLOCK as u32
    );
}

#[test]
fn inquiry_is_the_thirty_six_bytes_the_table_describes() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A host asks for 36 and gets 36; a host that asks for more gets a short
    // packet and a residue, which is case (5) and is tested below.
    let (data, csw) = bot.read_command(36, &[opcode::INQUIRY, 0, 0, 0, 36, 0]);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(data.len(), 36);
    // Seagate §3.6.2, table 59.
    assert_eq!(data[0], 0x00, "direct-access block device, connected");
    assert_eq!(data[1], 0x80, "RMB: this disk says it is removable");
    assert_eq!(data[3] & 0x0f, 2, "RESPONSE DATA FORMAT is 2 or nothing");
    assert_eq!(data[4], 31, "ADDITIONAL LENGTH is n - 4");
    assert_eq!(&data[8..16], b"RSEMU   ", "space padded, not NUL padded");
    assert_eq!(&data[16..32], b"USB DISK        ");
    assert_eq!(&data[32..36], b"1.00");
}

#[test]
fn the_unit_serial_number_page_is_the_string_descriptor_serial() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let (data, csw) = bot.read_command(64, &[opcode::INQUIRY, 0x01, 0x80, 0, 64, 0]);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(data[1], 0x80, "PAGE CODE");
    let len = usize::from(data[3]);
    assert_eq!(&data[4..4 + len], b"0123456789AB");
}

#[test]
fn an_unknown_operation_code_fails_and_says_so_in_the_sense_data() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let csw = bot.plain_command(&[0x5c, 0, 0, 0, 0, 0]);
    assert_eq!(csw.status, status::FAILED, "§6.6.4: Command Failed");

    let (sense, csw) = bot.read_command(18, &[opcode::REQUEST_SENSE, 0, 0, 0, 18, 0]);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(sense.len(), SENSE_BYTES);
    assert_eq!(sense[0], 0x70, "current error, fixed format");
    assert_eq!(sense[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
    assert_eq!(sense[7], 10, "ADDITIONAL SENSE LENGTH");
    assert_eq!((sense[12], sense[13]), asc::INVALID_COMMAND);

    // Reading it cleared it (Seagate §3.37).
    let (sense, _) = bot.read_command(18, &[opcode::REQUEST_SENSE, 0, 0, 0, 18, 0]);
    assert_eq!(sense[2] & 0x0f, sense_key::NO_SENSE);
}

#[test]
fn a_read_past_the_end_is_out_of_range_and_not_a_wrap() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // The last block plus one, and then the largest LBA the field can hold —
    // the second is the one that would wrap into block zero if the bounds
    // arithmetic were done in the guest's width and then widened.
    for lba in [BLOCKS as u32, u32::MAX] {
        assert_eq!(
            bot.command(BLOCK as u32, CBW_FLAG_IN, &read10(lba, 1)),
            Status::Ack
        );
        // Case (4), Hi > Dn: the device has nothing, so it stalls the bulk-in
        // pipe (§6.7.2) and the host clears the halt before reading the CSW.
        let (_, status) = bot.data_in(BLOCK as usize);
        assert_eq!(status, Status::Stall);
        clear_halt(&bus, ENDPOINT_IN | 0x80);
        let (csw, status) = bot.status();
        assert_eq!(status, Status::Ack);
        assert_eq!(csw.status, status::FAILED);
        assert_eq!(csw.residue, BLOCK as u32, "nothing was relevant");

        let (sense, _) = bot.read_command(18, &[opcode::REQUEST_SENSE, 0, 0, 0, 18, 0]);
        assert_eq!(sense[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
        assert_eq!((sense[12], sense[13]), asc::LBA_OUT_OF_RANGE);
    }
    // And nothing on the medium moved.
    assert_eq!(on_medium(&store, 0), stamp(0));
}

#[test]
fn a_write_to_a_read_only_disk_is_refused_before_the_medium_sees_it() {
    let store = Arc::new(RamStore::new(BLOCKS * BLOCK));
    for lba in 0..BLOCKS {
        RamStore::write_at(&store, lba * BLOCK, &stamp(lba)).expect("bytes");
    }
    let disk = UsbStorage::with_medium(
        Arc::clone(&store) as Arc<dyn Medium>,
        BLOCK,
        true,
        false,
        Speed::High,
        (1, 2),
        ("RSEMU", "USB DISK", "1.00", "0123456789AB"),
    );
    let bus = Arc::new(UsbBus::new(1));
    bus.attach(0, disk.device()).expect("an empty port");
    bus.set_enabled(0, true);
    run(
        &bus,
        DeviceAddress::DEFAULT,
        ControlTransfer::host_to_device(host::set_address(ADDRESS), &[]),
    );
    run(
        &bus,
        ADDRESS,
        ControlTransfer::host_to_device(host::set_configuration(1), &[]),
    );

    let mut bot = Bot::new(&bus, disk.max_packet());
    let payload = alloc::vec![0xffu8; BLOCK as usize];
    // The device intends `Dn`, so this is case (9) Ho > Dn: the bytes are
    // accepted and dropped and the status says the command failed.
    assert_eq!(bot.command(BLOCK as u32, 0, &write10(5, 1)), Status::Ack);
    let (sent, status) = bot.data_out(&payload);
    assert_eq!((sent, status), (BLOCK as usize, Status::Ack));
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::FAILED);
    assert_eq!(csw.residue, BLOCK as u32, "no byte was processed");
    assert_eq!(on_medium(&store, 5), stamp(5), "the medium was not written");

    let (sense, _) = bot.read_command(18, &[opcode::REQUEST_SENSE, 0, 0, 0, 18, 0]);
    assert_eq!(sense[2] & 0x0f, sense_key::DATA_PROTECT);
    assert_eq!((sense[12], sense[13]), asc::WRITE_PROTECTED);

    // And MODE SENSE says so, which is where a driver looks before mounting.
    let (mode, _) = bot.read_command(4, &[opcode::MODE_SENSE_6, 0, 0x3f, 0, 4, 0]);
    assert_eq!(
        mode[2] & 0x80,
        0x80,
        "the WP bit of the device-specific byte"
    );
}

#[test]
fn test_unit_ready_and_the_other_no_data_commands_pass() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    for cdb in [
        alloc::vec![opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0],
        alloc::vec![opcode::START_STOP_UNIT, 0, 0, 0, 1, 0],
        alloc::vec![opcode::PREVENT_ALLOW_MEDIUM_REMOVAL, 0, 0, 0, 1, 0],
        alloc::vec![opcode::SYNCHRONIZE_CACHE_10, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        // VERIFY (10) with BYTCHK clear is a bounds check, and this one is in
        // bounds.
        alloc::vec![opcode::VERIFY_10, 0, 0, 0, 0, 4, 0, 0, 2, 0],
    ] {
        let csw = bot.plain_command(&cdb);
        assert_eq!(csw.status, status::PASSED, "{cdb:02x?} should pass");
        assert_eq!(csw.residue, 0, "case (1): Hn = Dn");
    }
}

// ---------------------------------------------------------------------------
// The thirteen cases (BOT §6.7)
// ---------------------------------------------------------------------------

/// The host's half of Reset Recovery: `CLEAR_FEATURE(ENDPOINT_HALT)`
/// (USB 2.0 §9.4.1).
fn clear_halt(bus: &UsbBus, endpoint: u8) {
    let setup = SetupPacket {
        request_type: 0x02,
        request: crate::bus::usb::request::CLEAR_FEATURE,
        value: crate::bus::usb::feature::ENDPOINT_HALT,
        index: u16::from(endpoint),
        length: 0,
    };
    run(bus, ADDRESS, ControlTransfer::host_to_device(setup, &[]));
}

#[test]
fn case_1_hn_equals_dn_passes_with_no_residue() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let csw = bot.plain_command(&[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]);
    assert_eq!((csw.status, csw.residue), (status::PASSED, 0));
}

#[test]
fn cases_2_and_3_hn_below_a_device_that_has_data_are_phase_errors() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // (2) Hn < Di: a READ with dCBWDataTransferLength zero.
    assert_eq!(bot.command(0, 0, &read10(1, 1)), Status::Ack);
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR, "§6.7.1 case (2)");

    // (3) Hn < Do: a WRITE with dCBWDataTransferLength zero.
    assert_eq!(bot.command(0, 0, &write10(1, 1)), Status::Ack);
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR, "§6.7.1 case (3)");
}

#[test]
fn case_4_hi_above_dn_stalls_the_bulk_in_pipe_and_reports_the_whole_residue() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // TEST UNIT READY has no data, and the host claims to want 64 bytes of it.
    assert_eq!(
        bot.command(64, CBW_FLAG_IN, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let (data, status) = bot.data_in(64);
    assert!(data.is_empty());
    assert_eq!(status, Status::Stall, "§6.7.2 case (4)");
    // §5.3.4 (b): the host clears the halt, then reads the CSW.
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    let (csw, status) = bot.status();
    assert_eq!(status, Status::Ack);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 64);
}

#[test]
fn case_5_hi_above_di_ends_in_a_short_packet_and_a_residue() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // The classic: a host asks for 64 bytes of INQUIRY and gets 36.
    let (data, csw) = bot.read_command(64, &[opcode::INQUIRY, 0, 0, 0, 64, 0]);
    assert_eq!(data.len(), 36);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 64 - 36, "§6.7.2 case (5)");
}

#[test]
fn case_7_hi_below_di_is_a_phase_error_with_the_bulk_in_pipe_stalled() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A two-block read the host left room for one block of.
    assert_eq!(
        bot.command(BLOCK as u32, CBW_FLAG_IN, &read10(2, 2)),
        Status::Ack
    );
    let (data, status) = bot.data_in(BLOCK as usize);
    assert!(data.is_empty());
    assert_eq!(status, Status::Stall, "§6.7.2 case (7)");
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR);
}

#[test]
fn case_8_hi_against_do_is_a_phase_error() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A WRITE with the direction bit set to Data-In: the host is listening and
    // the device wants to be told.
    assert_eq!(
        bot.command(BLOCK as u32, CBW_FLAG_IN, &write10(3, 1)),
        Status::Ack
    );
    let (_, status) = bot.data_in(BLOCK as usize);
    assert_eq!(status, Status::Stall);
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR, "§6.7.2 case (8)");
}

#[test]
fn case_9_ho_above_dn_accepts_the_bytes_and_drops_them() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    assert_eq!(
        bot.command(32, 0, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let (sent, status) = bot.data_out(&[0xa5u8; 32]);
    assert_eq!((sent, status), (32, Status::Ack), "§6.7.3 case (9)");
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(csw.residue, 32, "nothing was processed");
    assert_eq!(on_medium(&store, 0), stamp(0), "and nothing was written");
}

#[test]
fn case_10_ho_against_di_is_a_phase_error_after_the_bytes_are_drained() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A READ with the direction bit clear.
    assert_eq!(bot.command(BLOCK as u32, 0, &read10(4, 1)), Status::Ack);
    let (sent, status) = bot.data_out(&alloc::vec![0x11u8; BLOCK as usize]);
    assert_eq!((sent, status), (BLOCK as usize, Status::Ack));
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR, "§6.7.3 case (10)");
    assert_eq!(on_medium(&store, 4), stamp(4), "a READ wrote nothing");
}

#[test]
fn case_11_ho_above_do_writes_what_the_device_wanted_and_reports_the_rest() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // The host offers two blocks and the device wants one.
    let payload: Vec<u8> = (0..2 * BLOCK).map(|i| (i as u8) ^ 0x3c).collect();
    assert_eq!(
        bot.command(2 * BLOCK as u32, 0, &write10(40, 1)),
        Status::Ack
    );
    let (sent, status) = bot.data_out(&payload);
    assert_eq!((sent, status), (payload.len(), Status::Ack));
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PASSED, "§6.7.3 case (11)");
    assert_eq!(csw.residue, BLOCK as u32);
    assert_eq!(on_medium(&store, 40), payload[..BLOCK as usize]);
    assert_eq!(
        on_medium(&store, 41),
        stamp(41),
        "the extra block was dropped"
    );
}

#[test]
fn case_13_ho_below_do_is_a_phase_error() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A two-block write the host only offered one block for.
    assert_eq!(bot.command(BLOCK as u32, 0, &write10(44, 2)), Status::Ack);
    let (sent, status) = bot.data_out(&alloc::vec![0x77u8; BLOCK as usize]);
    assert_eq!((sent, status), (BLOCK as usize, Status::Ack));
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::PHASE_ERROR, "§6.7.3 case (13)");
    assert_eq!(on_medium(&store, 44), stamp(44), "nothing half-written");
    assert_eq!(on_medium(&store, 45), stamp(45));
}

// ---------------------------------------------------------------------------
// Error recovery (BOT §5.3.4, §6.6.1)
// ---------------------------------------------------------------------------

#[test]
fn a_cbw_that_is_not_valid_wedges_both_pipes_until_a_full_reset_recovery() {
    let (bus, _store, disk) = enumerated();

    // §6.2.1: the signature is wrong, so this is not a CBW.
    let mut bad = [0u8; CBW_BYTES];
    bad[0..4].copy_from_slice(&0xdead_beefu32.to_le_bytes());
    bad[14] = 6;
    assert_eq!(
        bus.write(ADDRESS, ENDPOINT_OUT, &bad).status,
        Status::Stall,
        "§6.6.1: the device stalls"
    );

    // Both pipes are stalled and stay stalled.
    let mut buf = [0u8; CSW_BYTES];
    assert_eq!(
        bus.read(ADDRESS, ENDPOINT_IN, &mut buf).status,
        Status::Stall
    );
    assert_eq!(
        bus.write(ADDRESS, ENDPOINT_OUT, &[0u8; 4]).status,
        Status::Stall
    );

    // Clearing the halts *alone* must not resume it — §6.6.1 says the state is
    // maintained until a Reset Recovery, and §5.3.4 makes the class reset the
    // first step of one.
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    clear_halt(&bus, ENDPOINT_OUT);
    assert_eq!(
        bus.read(ADDRESS, ENDPOINT_IN, &mut buf).status,
        Status::Stall
    );

    // The whole procedure, in §5.3.4's order.
    bulk_only_reset(&bus);
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    clear_halt(&bus, ENDPOINT_OUT);

    let mut bot = Bot::new(&bus, disk.max_packet());
    let csw = bot.plain_command(&[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]);
    assert_eq!(csw.status, status::PASSED, "the device is working again");
}

#[test]
fn a_cbw_that_is_not_thirty_one_bytes_is_not_a_cbw() {
    let (bus, _store, _disk) = enumerated();
    let mut short = [0u8; 30];
    short[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    // §5.1: "shall end as a short packet with exactly 31 (1Fh) bytes
    // transferred" — a thirty-byte packet is not a command, and treating it as
    // one is how a reassembly buffer gets invented.
    assert_eq!(
        bus.write(ADDRESS, ENDPOINT_OUT, &short).status,
        Status::Stall
    );
}

/// BOT §3.1: the class reset.
fn bulk_only_reset(bus: &UsbBus) {
    let setup = SetupPacket {
        request_type: 0x21,
        request: class_request::BULK_ONLY_RESET,
        value: 0,
        index: INTERFACE,
        length: 0,
    };
    run(bus, ADDRESS, ControlTransfer::host_to_device(setup, &[]));
}

#[test]
fn the_class_reset_readies_the_device_but_preserves_the_stalls() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // Case (4) leaves the bulk-in pipe halted with a CSW waiting behind it.
    assert_eq!(
        bot.command(16, CBW_FLAG_IN, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let (_, stalled) = bot.data_in(16);
    assert_eq!(stalled, Status::Stall);

    // §3.1: the reset readies the device for the next CBW and leaves the stall.
    bulk_only_reset(&bus);
    let mut buf = [0u8; CSW_BYTES];
    assert_eq!(
        bus.read(ADDRESS, ENDPOINT_IN, &mut buf).status,
        Status::Stall,
        "§3.1: the endpoint STALL conditions survive the class reset"
    );
    clear_halt(&bus, ENDPOINT_IN | 0x80);

    // And the reset dropped the abandoned command, so the pipe is idle rather
    // than holding a stale CSW.
    assert_eq!(bus.read(ADDRESS, ENDPOINT_IN, &mut buf).status, Status::Nak);
    let csw = bot.plain_command(&[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]);
    assert_eq!(csw.status, status::PASSED);
}

#[test]
fn a_bus_reset_clears_everything_including_the_stalls() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    assert_eq!(
        bot.command(16, CBW_FLAG_IN, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let (_, stalled) = bot.data_in(16);
    assert_eq!(stalled, Status::Stall);

    // USB 2.0 §9.1.1.3 is a bigger hammer than BOT §3.1: the Default state,
    // address zero, unconfigured, nothing halted.
    bus.reset_port(0);
    assert_eq!(disk.address(), DeviceAddress::DEFAULT);
    assert_eq!(disk.configuration(), 0);
    bus.set_enabled(0, true);
    let mut buf = [0u8; CSW_BYTES];
    assert_eq!(
        bus.read(DeviceAddress::DEFAULT, ENDPOINT_IN, &mut buf)
            .status,
        Status::Nak,
        "no stall survives a bus reset"
    );
}

#[test]
fn the_host_may_ask_for_a_csw_before_it_has_sent_the_cbw() {
    // §3.3 explicitly allows it, and the answer is `NAK` rather than a stall or
    // an invented status wrapper.
    let (bus, _store, _disk) = enumerated();
    let mut buf = [0u8; CSW_BYTES];
    assert_eq!(bus.read(ADDRESS, ENDPOINT_IN, &mut buf).status, Status::Nak);
}

// ---------------------------------------------------------------------------
// The debug path
// ---------------------------------------------------------------------------

#[test]
fn a_debug_peek_shows_the_data_phase_without_consuming_it() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    assert_eq!(
        bot.command(BLOCK as u32, CBW_FLAG_IN, &read10(12, 1)),
        Status::Ack
    );

    // Peek twice: a debugger that advanced the cursor would see the second
    // half the second time.
    let mut first = alloc::vec![0u8; BLOCK as usize];
    let mut second = alloc::vec![0u8; BLOCK as usize];
    assert_eq!(
        bus.peek(ADDRESS, ENDPOINT_IN, &mut first).len,
        BLOCK,
        "the peek shows the block"
    );
    assert_eq!(bus.peek(ADDRESS, ENDPOINT_IN, &mut second).len, BLOCK);
    assert_eq!(first, second);
    assert_eq!(first, on_medium(&store, 12));

    // And the real read still gets all of it.
    let (data, csw) = {
        let (data, status) = bot.data_in(BLOCK as usize);
        assert_eq!(status, Status::Ack);
        let (csw, status) = bot.status();
        assert_eq!(status, Status::Ack);
        (data, csw)
    };
    assert_eq!(data, on_medium(&store, 12));
    assert_eq!(csw.residue, 0);
}

#[test]
fn a_debug_peek_does_not_pop_the_status_wrapper() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    assert_eq!(
        bot.command(0, 0, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let mut peeked = [0u8; CSW_BYTES];
    assert_eq!(
        bus.peek(ADDRESS, ENDPOINT_IN, &mut peeked).len as usize,
        CSW_BYTES
    );
    assert_eq!(
        u32::from_le_bytes([peeked[0], peeked[1], peeked[2], peeked[3]]),
        CSW_SIGNATURE
    );
    // Twice, and then for real: three identical answers.
    let mut again = [0u8; CSW_BYTES];
    let _ = bus.peek(ADDRESS, ENDPOINT_IN, &mut again);
    assert_eq!(peeked, again);
    let (csw, _) = bot.status();
    assert_eq!(csw.signature, CSW_SIGNATURE);
    assert_eq!(csw.status, status::PASSED);
}

#[test]
fn a_debug_peek_does_not_clear_the_sense_data() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // Arm some sense.
    assert_eq!(
        bot.plain_command(&[0x5c, 0, 0, 0, 0, 0]).status,
        status::FAILED
    );
    // Peek at the REQUEST SENSE response as many times as we like…
    assert_eq!(
        bot.command(18, CBW_FLAG_IN, &[opcode::REQUEST_SENSE, 0, 0, 0, 18, 0]),
        Status::Ack
    );
    let mut peeked = alloc::vec![0u8; 18];
    for _ in 0..3 {
        let _ = bus.peek(ADDRESS, ENDPOINT_IN, &mut peeked);
        assert_eq!(peeked[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
    }
    // …and the real read still returns it.
    let (data, status) = bot.data_in(18);
    assert_eq!(status, Status::Ack);
    assert_eq!(data, peeked);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// A state hash, so "identical" is a claim about every byte rather than about
/// the fields somebody remembered to compare.
fn hash(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        h ^= u64::from(*byte);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

#[test]
fn the_disk_round_trips_through_a_snapshot() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let _ = bot.read_command(BLOCK as u32, &read10(6, 1));

    let mut saved = Vec::new();
    disk.save_to(&mut saved).expect("it saves");

    let (_, other) = disk_at(BLOCKS);
    other.load_from(&saved).expect("it loads");
    let mut again = Vec::new();
    other.save_to(&mut again).expect("it saves");
    assert_eq!(
        hash(&saved),
        hash(&again),
        "the state hash must be identical"
    );
}

#[test]
fn a_transfer_half_way_through_survives_a_snapshot() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // Two blocks, one packet in: the cursor is mid-transfer and the residue is
    // half-computed, which is exactly the state a naive snapshot drops.
    assert_eq!(
        bot.command(2 * BLOCK as u32, CBW_FLAG_IN, &read10(30, 2)),
        Status::Ack
    );
    let mut first = alloc::vec![0u8; BLOCK as usize];
    assert_eq!(bus.read(ADDRESS, ENDPOINT_IN, &mut first).len, BLOCK);
    assert_eq!(first, on_medium(&store, 30));

    let mut saved = Vec::new();
    disk.save_to(&mut saved).expect("it saves");

    // Restore into a fresh device on a fresh bus and finish the transfer there.
    let (other_store, other) = disk_at(BLOCKS);
    other.load_from(&saved).expect("it loads");
    let other_bus = Arc::new(UsbBus::new(1));
    other_bus.attach(0, other.device()).expect("an empty port");
    other_bus.set_enabled(0, true);

    let mut second = alloc::vec![0u8; BLOCK as usize];
    assert_eq!(other_bus.read(ADDRESS, ENDPOINT_IN, &mut second).len, BLOCK);
    assert_eq!(
        second,
        on_medium(&other_store, 31),
        "the restored cursor is at the second block, not back at the first"
    );
    let mut csw = [0u8; CSW_BYTES];
    let done = other_bus.read(ADDRESS, ENDPOINT_IN, &mut csw);
    assert_eq!(done.len as usize, CSW_BYTES);
    assert_eq!(csw[12], status::PASSED);
    assert_eq!(
        u32::from_le_bytes([csw[8], csw[9], csw[10], csw[11]]),
        0,
        "and the residue survived too"
    );
}

/// A second disk with the same geometry and the same stamped contents.
fn disk_at(blocks: u64) -> (Arc<RamStore>, UsbStorage) {
    let store = Arc::new(RamStore::new(blocks * BLOCK));
    for lba in 0..blocks {
        RamStore::write_at(&store, lba * BLOCK, &stamp(lba)).expect("bytes");
    }
    let disk = UsbStorage::with_medium(
        Arc::clone(&store) as Arc<dyn Medium>,
        BLOCK,
        false,
        true,
        Speed::High,
        (0x0781, 0x5567),
        ("RSEMU", "USB DISK", "1.00", "0123456789AB"),
    );
    (store, disk)
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

#[test]
fn a_huge_transfer_length_costs_one_packet_and_not_a_gigabyte() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // 65,535 blocks is 32 MiB the guest asked for. It is out of range on this
    // disk, so what is being asserted is that asking cost nothing — a device
    // that materialised the transfer before checking would have allocated it.
    assert_eq!(
        bot.command(u32::MAX, CBW_FLAG_IN, &read10(0, u16::MAX)),
        Status::Ack
    );
    let (_, status) = bot.data_in(BLOCK as usize);
    assert_eq!(status, Status::Stall);
    clear_halt(&bus, ENDPOINT_IN | 0x80);
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::FAILED);
}

#[test]
fn an_allocation_length_the_guest_chose_is_clamped() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // An INQUIRY asking for 65,535 bytes gets 36 and a residue, because the
    // device transfers the smaller of the allocation length and what it has.
    let (data, csw) = bot.read_command(0xffff, &[opcode::INQUIRY, 0, 0, 0xff, 0xff, 0]);
    assert_eq!(data.len(), 36);
    assert_eq!(csw.residue, 0xffff - 36);
    assert_eq!(csw.status, status::PASSED);
}

#[test]
fn an_allocation_length_of_zero_is_case_1_and_not_a_phase_error() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // A legal probe: "tell me nothing". Reading it as `Di` with zero length
    // would make it BOT case (2) and fail a driver that does this on purpose.
    let csw = bot.plain_command(&[opcode::INQUIRY, 0, 0, 0, 0, 0]);
    assert_eq!((csw.status, csw.residue), (status::PASSED, 0));
}

#[test]
fn a_command_block_length_outside_one_to_sixteen_fails_the_command() {
    let (bus, _store, _disk) = enumerated();
    // §5.1: "The only legal values are 1 through 16". A CBW carrying zero is
    // valid (§6.2.1) but not meaningful (§6.2.2), which is a failed command
    // rather than a wedged device.
    let mut cbw = [0u8; CBW_BYTES];
    cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    cbw[4..8].copy_from_slice(&0x4142_4344u32.to_le_bytes());
    cbw[14] = 0;
    assert_eq!(bus.write(ADDRESS, ENDPOINT_OUT, &cbw).status, Status::Ack);
    let mut buf = [0u8; CSW_BYTES];
    assert_eq!(
        bus.read(ADDRESS, ENDPOINT_IN, &mut buf).len as usize,
        CSW_BYTES
    );
    assert_eq!(buf[12], status::FAILED);
    assert_eq!(
        u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        0x4142_4344,
        "and the tag still came back"
    );
}

#[test]
fn a_reserved_flag_bit_fails_the_command() {
    let (bus, _store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    // §5.1: bits 6..0 of `bmCBWFlags` are reserved and the host sets them to
    // zero, so a CBW with one set is not meaningful (§6.2.2).
    assert_eq!(
        bot.command(0, 0x40, &[opcode::TEST_UNIT_READY, 0, 0, 0, 0, 0]),
        Status::Ack
    );
    let (csw, _) = bot.status();
    assert_eq!(csw.status, status::FAILED);
}

#[test]
fn a_command_for_a_lun_this_device_does_not_have_fails() {
    let (bus, _store, _disk) = enumerated();
    let mut cbw = [0u8; CBW_BYTES];
    cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    cbw[13] = 3;
    cbw[14] = 6;
    cbw[15] = opcode::TEST_UNIT_READY;
    assert_eq!(bus.write(ADDRESS, ENDPOINT_OUT, &cbw).status, Status::Ack);
    let mut buf = [0u8; CSW_BYTES];
    bus.read(ADDRESS, ENDPOINT_IN, &mut buf);
    assert_eq!(buf[12], status::FAILED);

    let mut sense_cbw = [0u8; CBW_BYTES];
    sense_cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    sense_cbw[8..12].copy_from_slice(&18u32.to_le_bytes());
    sense_cbw[12] = CBW_FLAG_IN;
    sense_cbw[14] = 6;
    sense_cbw[15] = opcode::REQUEST_SENSE;
    sense_cbw[19] = 18;
    assert_eq!(
        bus.write(ADDRESS, ENDPOINT_OUT, &sense_cbw).status,
        Status::Ack
    );
    let mut sense = [0u8; 18];
    bus.read(ADDRESS, ENDPOINT_IN, &mut sense);
    assert_eq!(sense[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
    assert_eq!((sense[12], sense[13]), asc::LUN_NOT_SUPPORTED);
}

#[test]
fn read_and_write_in_their_six_twelve_and_sixteen_byte_forms_agree() {
    let (bus, store, disk) = enumerated();
    let mut bot = Bot::new(&bus, disk.max_packet());
    let want = on_medium(&store, 17);

    // READ (6): a 21-bit address and an eight-bit block count.
    let (six, csw) = bot.read_command(BLOCK as u32, &[opcode::READ_6, 0, 0, 17, 1, 0]);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(six, want);

    // READ (12).
    let mut cdb = [0u8; 12];
    cdb[0] = opcode::READ_12;
    cdb[2..6].copy_from_slice(&17u32.to_be_bytes());
    cdb[6..10].copy_from_slice(&1u32.to_be_bytes());
    let (twelve, csw) = bot.read_command(BLOCK as u32, &cdb);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(twelve, want);

    // READ (16), whose LBA is sixty-four bits.
    let mut cdb = [0u8; 16];
    cdb[0] = opcode::READ_16;
    cdb[2..10].copy_from_slice(&17u64.to_be_bytes());
    cdb[10..14].copy_from_slice(&1u32.to_be_bytes());
    let (sixteen, csw) = bot.read_command(BLOCK as u32, &cdb);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(sixteen, want);

    // And a WRITE (16) reaches the medium at the LBA the wide field named.
    let payload = alloc::vec![0xc3u8; BLOCK as usize];
    let mut cdb = [0u8; 16];
    cdb[0] = opcode::WRITE_16;
    cdb[2..10].copy_from_slice(&50u64.to_be_bytes());
    cdb[10..14].copy_from_slice(&1u32.to_be_bytes());
    let csw = bot.write_command(&payload, &cdb);
    assert_eq!(csw.status, status::PASSED);
    assert_eq!(on_medium(&store, 50), payload);
    assert_eq!(on_medium(&store, 49), stamp(49));
    assert_eq!(on_medium(&store, 51), stamp(51));
}
