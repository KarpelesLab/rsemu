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
    let name = "test-usb-rendezvous";
    buses::close(name);
    assert!(buses::get(name).is_none());
    let controller_side = buses::open(name, 4);
    let device_side = buses::open(name, 1);
    assert!(Arc::ptr_eq(&controller_side, &device_side));
    assert_eq!(
        controller_side.port_count(),
        4,
        "the first mention fixes the size"
    );
    assert!(buses::names().iter().any(|n| n == name));
    assert!(buses::close(name));
    assert!(!buses::close(name));
    // A later open is a fresh bus, which is what makes a test able to have the
    // name back.
    let again = buses::open(name, 2);
    assert!(!Arc::ptr_eq(&controller_side, &again));
    buses::close(name);
}
