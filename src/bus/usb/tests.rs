//! The fabric on its own: routing, ports, speeds and the encodings.
//!
//! What a controller does with all this is tested where the controllers are
//! (`crate::dev::usb::ehci`), and what a real device does with it is tested
//! where the device is (`crate::dev::usb::hid`). These are the parts that
//! belong to neither.

use super::*;

use alloc::vec::Vec;

use crate::core::sync::{LockRank, Mutex};

// ---------------------------------------------------------------------------
// Encodings
// ---------------------------------------------------------------------------

#[test]
fn a_setup_packet_survives_the_wire() {
    let packet = SetupPacket {
        request_type: 0xa1,
        request: 0x01,
        value: 0x0100,
        index: 3,
        length: 8,
    };
    assert_eq!(SetupPacket::decode(&packet.encode()), packet);
    // Little-endian, per USB 2.0 §9.3.
    assert_eq!(
        packet.encode(),
        [0xa1, 0x01, 0x00, 0x01, 0x03, 0x00, 0x08, 0x00]
    );
}

#[test]
fn a_request_type_decomposes_the_way_the_table_says() {
    // §9.3.1, table 9-2.
    let host_to_device_standard_device = SetupPacket {
        request_type: 0x00,
        ..SetupPacket::default()
    };
    assert_eq!(host_to_device_standard_device.direction(), Direction::Out);
    assert_eq!(host_to_device_standard_device.kind(), RequestKind::Standard);
    assert_eq!(
        host_to_device_standard_device.recipient(),
        Recipient::Device
    );

    let device_to_host_class_interface = SetupPacket {
        request_type: 0xa1,
        ..SetupPacket::default()
    };
    assert_eq!(device_to_host_class_interface.direction(), Direction::In);
    assert_eq!(device_to_host_class_interface.kind(), RequestKind::Class);
    assert_eq!(
        device_to_host_class_interface.recipient(),
        Recipient::Interface
    );

    let vendor_endpoint = SetupPacket {
        request_type: 0x42,
        ..SetupPacket::default()
    };
    assert_eq!(vendor_endpoint.kind(), RequestKind::Vendor);
    assert_eq!(vendor_endpoint.recipient(), Recipient::Endpoint);
}

#[test]
fn a_device_descriptor_is_eighteen_bytes_in_the_documented_order() {
    let descriptor = DeviceDescriptor {
        usb: 0x0200,
        max_packet0: 64,
        vendor: 0x1d6b,
        product: 0x0002,
        configurations: 1,
        ..DeviceDescriptor::default()
    };
    let bytes = descriptor.encode();
    assert_eq!(bytes.len(), 18);
    assert_eq!(bytes[0], 18, "bLength");
    assert_eq!(bytes[1], 1, "bDescriptorType");
    assert_eq!(&bytes[2..4], &[0x00, 0x02], "bcdUSB, little-endian");
    assert_eq!(bytes[7], 64, "bMaxPacketSize0");
    assert_eq!(&bytes[8..10], &[0x6b, 0x1d], "idVendor");
    assert_eq!(bytes[17], 1, "bNumConfigurations");
}

#[test]
fn a_configuration_descriptor_counts_everything_after_it() {
    let mut descriptors = Descriptors::new();
    let interface = InterfaceDescriptor {
        endpoints: 1,
        ..InterfaceDescriptor::default()
    };
    let endpoint = EndpointDescriptor {
        address: 0x81,
        attributes: TransferType::Interrupt.attribute_bits(),
        max_packet: 8,
        interval: 4,
    };
    let mut body = Vec::new();
    body.extend_from_slice(&interface.encode());
    body.extend_from_slice(&endpoint.encode());
    descriptors.add_configuration(&ConfigurationDescriptor::default(), &body);

    let bytes = descriptors
        .get(DescriptorKind::CONFIGURATION, 0)
        .expect("there is one");
    assert_eq!(bytes.len(), 9 + 9 + 7);
    assert_eq!(
        u16::from_le_bytes([bytes[2], bytes[3]]),
        bytes.len() as u16,
        "wTotalLength covers the whole tree, which is why it is computed"
    );
    // §9.6.3: bit 7 of bmAttributes is reserved and reads one.
    assert_ne!(bytes[7] & 0x80, 0);
}

#[test]
fn strings_are_utf16_and_index_zero_lists_the_languages() {
    let mut descriptors = Descriptors::new();
    let index = descriptors.add_string("rsemu");
    assert_eq!(index, 1, "index zero is the language list");

    let languages = descriptors
        .get(DescriptorKind::STRING, 0)
        .expect("a language list");
    assert_eq!(languages, &[4, 3, 0x09, 0x04], "US English");

    let text = descriptors
        .get(DescriptorKind::STRING, 1)
        .expect("the string");
    assert_eq!(text[0] as usize, text.len());
    assert_eq!(text[1], 3);
    assert_eq!(&text[2..], &[b'r', 0, b's', 0, b'e', 0, b'm', 0, b'u', 0]);
}

#[test]
fn an_absent_descriptor_is_none_rather_than_empty() {
    let descriptors = Descriptors::new();
    assert!(descriptors.get(DescriptorKind::DEVICE, 0).is_none());
    assert!(descriptors.get(DescriptorKind::CONFIGURATION, 7).is_none());
    assert!(descriptors.get(DescriptorKind(0x22), 0).is_none());
}

#[test]
fn the_transfer_types_are_the_two_bit_encoding_of_the_endpoint_descriptor() {
    for kind in [
        TransferType::Control,
        TransferType::Isochronous,
        TransferType::Bulk,
        TransferType::Interrupt,
    ] {
        assert_eq!(
            TransferType::from_attribute_bits(kind.attribute_bits()),
            kind
        );
    }
    // §9.6.6, table 9-13.
    assert_eq!(TransferType::Bulk.attribute_bits(), 2);
    assert_eq!(TransferType::Interrupt.attribute_bits(), 3);
}

#[test]
fn a_speed_is_named_the_way_a_machine_file_spells_it() {
    for speed in [Speed::Low, Speed::Full, Speed::High] {
        assert_eq!(Speed::from_name(speed.name()), Some(speed));
        assert!(Speed::NAMES.contains(&speed.name()));
    }
    assert_eq!(Speed::from_name("super"), None);
    // §5.5.3: low speed can only manage eight-byte control packets.
    assert_eq!(Speed::Low.max_control_packet(), 8);
    assert_eq!(Speed::High.max_control_packet(), 64);
}

#[test]
fn a_nak_is_not_an_error_and_is_not_final() {
    assert!(!Status::Nak.is_error());
    assert!(!Status::Nak.is_final());
    assert!(Status::Ack.is_final());
    for status in [
        Status::Stall,
        Status::NoDevice,
        Status::Babble,
        Status::Error,
    ] {
        assert!(status.is_error(), "{status} should be an error");
        assert!(status.is_final());
    }
}

// ---------------------------------------------------------------------------
// A device to route to
// ---------------------------------------------------------------------------

/// The smallest thing that can answer a token: it remembers its address and
/// says how fast it is.
#[derive(Debug)]
struct Stub {
    speed: Speed,
    state: Mutex<StubState>,
}

#[derive(Debug, Default, Clone, Copy)]
struct StubState {
    address: u8,
    reads: u32,
    resets: u32,
}

impl Stub {
    fn new(speed: Speed, address: u8) -> Arc<Stub> {
        Arc::new(Stub {
            speed,
            state: Mutex::with_rank(
                LockRank::DEVICE,
                StubState {
                    address,
                    ..StubState::default()
                },
            ),
        })
    }
}

impl UsbDevice for Stub {
    fn speed(&self) -> Speed {
        self.speed
    }
    fn address(&self) -> DeviceAddress {
        DeviceAddress(self.state.lock().address)
    }
    fn bus_reset(&self) {
        let mut state = self.state.lock();
        state.address = 0;
        state.resets += 1;
    }
    fn setup(&self, _endpoint: u8, _packet: SetupPacket) -> Status {
        Status::Ack
    }
    fn transfer_in(&self, _endpoint: u8, dst: &mut [u8]) -> Completion {
        self.state.lock().reads += 1;
        let n = dst.len().min(1);
        if n > 0 {
            dst[0] = self.state.lock().address;
        }
        Completion::ack(n as u64)
    }
    fn transfer_out(&self, _endpoint: u8, src: &[u8]) -> Completion {
        Completion::ack(src.len() as u64)
    }
}

// ---------------------------------------------------------------------------
// Ports and routing
// ---------------------------------------------------------------------------

#[test]
fn a_port_takes_one_device_and_says_so_if_offered_two() {
    let bus = UsbBus::new(2);
    assert_eq!(bus.port_count(), 2);
    bus.attach(0, Stub::new(Speed::High, 0)).expect("empty");
    let err = bus
        .attach(0, Stub::new(Speed::High, 0))
        .expect_err("two devices on one port is a short, not a topology");
    assert!(alloc::format!("{err}").contains("port"));
    assert!(
        bus.attach(9, Stub::new(Speed::High, 0)).is_err(),
        "no port 9"
    );
}

#[test]
fn a_port_count_is_clamped_into_what_a_controller_can_report() {
    // `HCSPARAMS.N_PORTS` is four bits, and a bus with no ports is not a bus.
    assert_eq!(UsbBus::new(0).port_count(), 1);
    assert_eq!(UsbBus::new(200).port_count(), MAX_PORTS as u8);
}

#[test]
fn nothing_is_routed_to_a_port_the_controller_has_not_enabled() {
    let bus = UsbBus::new(1);
    bus.attach(0, Stub::new(Speed::High, 4)).expect("empty");
    assert_eq!(
        bus.setup(DeviceAddress(4), 0, SetupPacket::default()),
        Status::NoDevice,
        "a port comes up disabled, and enabling it is the controller's decision"
    );
    bus.set_enabled(0, true);
    assert_eq!(
        bus.setup(DeviceAddress(4), 0, SetupPacket::default()),
        Status::Ack
    );
}

/// The hazard the port model exists to make impossible.
#[test]
fn two_freshly_attached_devices_do_not_both_answer_address_zero() {
    let bus = UsbBus::new(2);
    let first = Stub::new(Speed::High, 0);
    let second = Stub::new(Speed::High, 0);
    bus.attach(0, Arc::clone(&first) as Arc<dyn UsbDevice>)
        .expect("empty");
    bus.attach(1, Arc::clone(&second) as Arc<dyn UsbDevice>)
        .expect("empty");

    // A host resets and addresses one port at a time (§9.1.2), so only one is
    // ever enabled while it is at the default address.
    bus.set_enabled(0, true);
    let mut byte = [0u8; 1];
    assert_eq!(bus.read(DeviceAddress::DEFAULT, 0, &mut byte).len, 1);
    assert_eq!(first.state.lock().reads, 1);
    assert_eq!(
        second.state.lock().reads,
        0,
        "the disabled port answered nothing"
    );
}

#[test]
fn a_reset_reaches_the_device_and_leaves_the_port_disabled() {
    let bus = UsbBus::new(1);
    let stub = Stub::new(Speed::High, 6);
    bus.attach(0, Arc::clone(&stub) as Arc<dyn UsbDevice>)
        .expect("empty");
    bus.set_enabled(0, true);
    bus.reset_port(0);
    assert_eq!(stub.state.lock().resets, 1);
    assert_eq!(stub.state.lock().address, 0, "back to the Default state");
    assert!(
        !bus.enabled(0),
        "enabling after a reset is the controller's decision, not the fabric's — \
         a high-speed controller that found a full-speed device hands the port over instead"
    );
}

#[test]
fn a_connection_change_is_reported_once() {
    let bus = UsbBus::new(2);
    assert!(!bus.any_change());
    bus.attach(1, Stub::new(Speed::Full, 0)).expect("empty");
    assert!(bus.any_change());
    assert!(!bus.take_change(0), "the other port did not move");
    assert!(bus.take_change(1));
    assert!(!bus.take_change(1), "and the flag is cleared by reading it");
    assert!(!bus.any_change());

    assert!(bus.detach(1));
    assert!(bus.take_change(1), "unplugging is a change too");
    assert!(!bus.detach(1), "and there is nothing left to unplug");
}

#[test]
fn a_ports_speed_is_the_devices() {
    let bus = UsbBus::new(2);
    bus.attach(0, Stub::new(Speed::Low, 0)).expect("empty");
    assert_eq!(bus.speed(0), Some(Speed::Low));
    assert_eq!(bus.speed(1), None, "nothing is plugged into port 1");
    assert!(bus.connected(0));
    assert!(!bus.connected(1));
}

#[test]
fn an_unanswered_address_is_no_device_rather_than_a_silent_zero() {
    let bus = UsbBus::new(1);
    bus.attach(0, Stub::new(Speed::High, 1)).expect("empty");
    bus.set_enabled(0, true);
    let mut byte = [0u8; 1];
    assert_eq!(
        bus.read(DeviceAddress(2), 0, &mut byte).status,
        Status::NoDevice
    );
    assert_eq!(
        bus.write(DeviceAddress(2), 0, &[1]).status,
        Status::NoDevice
    );
    assert_eq!(
        bus.setup(DeviceAddress(2), 0, SetupPacket::default()),
        Status::NoDevice
    );
}

#[test]
fn the_default_peek_does_not_invent_data() {
    // A device that cannot show its FIFO without consuming it says so, and the
    // fabric passes that on rather than making something up.
    let bus = UsbBus::new(1);
    bus.attach(0, Stub::new(Speed::High, 1)).expect("empty");
    bus.set_enabled(0, true);
    let mut byte = [0u8; 1];
    assert_eq!(bus.peek(DeviceAddress(1), 0, &mut byte).status, Status::Nak);
}

// ---------------------------------------------------------------------------
// The rendezvous
// ---------------------------------------------------------------------------

#[test]
fn a_name_reaches_the_same_bus_from_both_ends() {
    use crate::core::HostObjects;

    let hosts = HostObjects::new();
    assert!(buses::get(&hosts, "usb0").unwrap().is_none());
    let controller_side = buses::open(&hosts, "usb0", 4).unwrap();
    let device_side = buses::open(&hosts, "usb0", 1).unwrap();
    assert!(Arc::ptr_eq(&controller_side, &device_side));
    assert_eq!(
        controller_side.port_count(),
        4,
        "the first mention fixes the size"
    );
    assert_eq!(buses::names(&hosts), ["usb0"]);
    assert!(buses::close(&hosts, "usb0"));
    assert!(!buses::close(&hosts, "usb0"));
    // A later open is a fresh bus, which is what makes a test able to have the
    // name back.
    let again = buses::open(&hosts, "usb0", 2).unwrap();
    assert!(!Arc::ptr_eq(&controller_side, &again));

    // And a second build's `usb0` is a second bus, so two machines can both
    // have one without enumerating each other's devices.
    let elsewhere = HostObjects::new();
    let theirs = buses::open(&elsewhere, "usb0", 4).unwrap();
    assert!(!Arc::ptr_eq(&controller_side, &theirs));
}

// ---------------------------------------------------------------------------
// Start of frame: the one thing on the wire that is not a transaction
// ---------------------------------------------------------------------------

/// A device that does nothing but count frames.
#[derive(Debug, Default)]
struct FrameCounter {
    seen: Mutex<Vec<u16>>,
}

impl UsbDevice for FrameCounter {
    fn speed(&self) -> Speed {
        Speed::Full
    }
    fn address(&self) -> DeviceAddress {
        DeviceAddress::DEFAULT
    }
    fn bus_reset(&self) {}
    fn start_of_frame(&self, frame: u16) {
        self.seen.lock().push(frame);
    }
    fn setup(&self, _endpoint: u8, _packet: SetupPacket) -> Status {
        Status::Ack
    }
    fn transfer_in(&self, _endpoint: u8, _dst: &mut [u8]) -> Completion {
        Completion::nak()
    }
    fn transfer_out(&self, _endpoint: u8, _src: &[u8]) -> Completion {
        Completion::nak()
    }
}

#[test]
fn a_start_of_frame_reaches_every_connected_device_enabled_or_not() {
    let bus = UsbBus::new(2);
    let a = Arc::new(FrameCounter::default());
    let b = Arc::new(FrameCounter::default());
    bus.attach(0, Arc::clone(&a) as Arc<dyn UsbDevice>)
        .expect("an empty port");
    bus.attach(1, Arc::clone(&b) as Arc<dyn UsbDevice>)
        .expect("an empty port");
    // Only one port is enabled, and it makes no difference: a `SOF` is
    // broadcast, not addressed (USB 2.0 §8.4.3).
    bus.set_enabled(0, true);

    bus.start_of_frame(1);
    // Eleven bits, because that is the field the token carries.
    bus.start_of_frame(0x0fff);

    assert_eq!(*a.seen.lock(), alloc::vec![1, 0x7ff]);
    assert_eq!(
        *b.seen.lock(),
        alloc::vec![1, 0x7ff],
        "a device on a port the controller has not enabled still counts frames"
    );
}

#[test]
fn a_device_that_does_not_care_about_frames_says_nothing() {
    // The default is a no-op, which is why adding this to the trait changed no
    // existing device model.
    let bus = UsbBus::new(1);
    bus.attach(0, Stub::new(Speed::High, 4) as Arc<dyn UsbDevice>)
        .expect("an empty port");
    bus.start_of_frame(7);
    bus.set_enabled(0, true);
    assert_eq!(
        bus.find(DeviceAddress(4)).map(|d| d.speed()),
        Some(Speed::High)
    );
}

// ---------------------------------------------------------------------------
// The host-side transfer composer
// ---------------------------------------------------------------------------

/// A device with nothing but descriptors: enough to be enumerated, which is all
/// these tests ask of it.
#[derive(Debug)]
struct Plain {
    descriptors: Descriptors,
}

impl Plain {
    fn peripheral() -> Arc<Peripheral> {
        let device = DeviceDescriptor {
            vendor: 0x1d6b,
            product: 0x0104,
            max_packet0: 8,
            ..DeviceDescriptor::default()
        };
        let body = InterfaceDescriptor {
            endpoints: 0,
            class: 0xff,
            ..InterfaceDescriptor::default()
        }
        .encode();
        let mut descriptors = Descriptors::new().with_device(&device);
        descriptors.add_configuration(&ConfigurationDescriptor::default(), &body);
        Arc::new(Peripheral::new(Arc::new(Plain { descriptors })))
    }
}

impl Function for Plain {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }
    fn speed(&self) -> Speed {
        Speed::Full
    }
}

/// Step a transfer to completion, with a bound: nothing here NAKs, so anything
/// that needs more than a handful of transactions is a bug in the composer.
fn run(bus: &UsbBus, address: DeviceAddress, mps: u16, transfer: &mut ControlTransfer) -> Progress {
    for _ in 0..32 {
        let progress = transfer.step(bus, address, mps);
        if progress.is_finished() {
            return progress;
        }
    }
    panic!("a transfer with no NAKs in it did not finish");
}

fn enumerable() -> Arc<UsbBus> {
    let bus = Arc::new(UsbBus::new(1));
    bus.attach(0, Plain::peripheral() as Arc<dyn UsbDevice>)
        .expect("an empty port");
    bus.set_enabled(0, true);
    bus
}

#[test]
fn the_composer_collects_a_descriptor_a_packet_at_a_time() {
    let bus = enumerable();
    // Eight bytes a packet, so eighteen bytes is three transactions and the
    // last one is short — which is how the device says "that is all of it".
    let mut transfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert_eq!(
        run(&bus, DeviceAddress::DEFAULT, 8, &mut transfer),
        Progress::Done
    );
    let bytes = transfer.data();
    assert_eq!(bytes.len(), 18);
    assert_eq!(bytes[0], 18, "bLength");
    assert_eq!(bytes[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(&bytes[8..10], &[0x6b, 0x1d], "idVendor");
}

#[test]
fn a_short_descriptor_ends_the_data_stage_early() {
    let bus = enumerable();
    // The classic first request of an enumeration: eight bytes of an eighteen
    // byte descriptor, so the host learns `bMaxPacketSize0` before asking for
    // the rest.
    let mut transfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 8));
    assert_eq!(
        run(&bus, DeviceAddress::DEFAULT, 8, &mut transfer),
        Progress::Done
    );
    assert_eq!(transfer.data().len(), 8);
    assert_eq!(transfer.data()[7], 8, "bMaxPacketSize0");
}

#[test]
fn the_composer_gets_set_address_right_including_the_status_stage() {
    let bus = enumerable();
    // The whole point of §9.4.6: the status stage is still addressed to zero,
    // and the new address only takes effect once it completes. A composer that
    // switched early would hang here.
    let mut transfer = ControlTransfer::host_to_device(host::set_address(DeviceAddress(11)), &[]);
    assert_eq!(
        run(&bus, DeviceAddress::DEFAULT, 8, &mut transfer),
        Progress::Done
    );
    assert!(bus.find(DeviceAddress(11)).is_some());

    let mut configure = ControlTransfer::host_to_device(host::set_configuration(1), &[]);
    assert_eq!(
        run(&bus, DeviceAddress(11), 8, &mut configure),
        Progress::Done
    );
}

#[test]
fn a_request_the_device_refuses_ends_as_a_stall_and_not_a_hang() {
    let bus = enumerable();
    // `SET_DESCRIPTOR` is optional and nothing in this tree implements it
    // (§9.4.8), so the device stalls the stage after the setup.
    let setup = SetupPacket {
        request_type: 0,
        request: request::SET_DESCRIPTOR,
        value: 0x0100,
        index: 0,
        length: 4,
    };
    let mut transfer = ControlTransfer::host_to_device(setup, &[1, 2, 3, 4]);
    assert_eq!(
        run(&bus, DeviceAddress::DEFAULT, 8, &mut transfer),
        Progress::Failed(Status::Stall)
    );
    assert_eq!(transfer.failure(), Some(Status::Stall));
    // And stepping a finished transfer repeats the outcome rather than
    // restarting anything.
    assert_eq!(
        transfer.step(&bus, DeviceAddress::DEFAULT, 8),
        Progress::Failed(Status::Stall)
    );
}

#[test]
fn an_address_nothing_answers_is_no_device_rather_than_a_wait() {
    let bus = UsbBus::new(1);
    let mut transfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert_eq!(
        transfer.step(&bus, DeviceAddress::DEFAULT, 8),
        Progress::Failed(Status::NoDevice)
    );
    assert!(transfer.is_finished());
    assert!(transfer.data().is_empty());
}
