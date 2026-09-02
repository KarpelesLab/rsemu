//! The hub, at the level of a host issuing class requests at it.
//!
//! `tests/usb_hub.rs` is the claim that matters — a guest, through an EHCI,
//! reaching a disk behind a hub. These are the ones that name a paragraph of
//! USB 2.0 §11 each, so a regression says which sentence it broke.

use super::*;

use crate::bus::usb::host::{ControlTransfer, Progress};
use crate::bus::usb::{DeviceAddress, MAX_TIERS, Status, request};
use crate::core::state::ChunkReader;

/// The address the harness gives the hub, and the one it gives the device
/// behind it. Different on purpose: a reply that came back from the wrong tier
/// is then visible rather than plausible.
const HUB_ADDRESS: DeviceAddress = DeviceAddress(1);
const DEVICE_ADDRESS: DeviceAddress = DeviceAddress(2);

/// `bmRequestType` for the hub class requests of table 11-16.
const TO_HUB: u8 = 0x20;
const FROM_HUB: u8 = 0xa0;
const TO_PORT: u8 = 0x23;
const FROM_PORT: u8 = 0xa3;

fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> SetupPacket {
    SetupPacket {
        request_type,
        request,
        value,
        index,
        length,
    }
}

/// Run a whole control transfer against `bus`, which is the **root** bus: the
/// routing walk is what finds the device, whichever tier it is on.
fn control(bus: &UsbBus, address: DeviceAddress, mut xfer: ControlTransfer) -> Option<Vec<u8>> {
    for _ in 0..64 {
        match xfer.step(bus, address, 64) {
            Progress::Done => return Some(xfer.take_data()),
            Progress::Failed(_) => return None,
            Progress::Moved | Progress::Nak => {}
        }
    }
    panic!("a control transfer that never finished");
}

fn get(bus: &UsbBus, address: DeviceAddress, packet: SetupPacket) -> Option<Vec<u8>> {
    control(bus, address, ControlTransfer::device_to_host(packet))
}

fn put(bus: &UsbBus, address: DeviceAddress, packet: SetupPacket) -> bool {
    control(bus, address, ControlTransfer::host_to_device(packet, &[])).is_some()
}

/// `GetPortStatus` for a **one-based** port number, decoded.
fn port_status(bus: &UsbBus, address: DeviceAddress, port: u16) -> (u16, u16) {
    let bytes = get(
        bus,
        address,
        setup(FROM_PORT, request::GET_STATUS, 0, port, 4),
    )
    .expect("GetPortStatus is not an optional request");
    assert_eq!(bytes.len(), 4, "§11.24.2.7: four bytes");
    (
        u16::from_le_bytes([bytes[0], bytes[1]]),
        u16::from_le_bytes([bytes[2], bytes[3]]),
    )
}

/// A root bus with a hub on port 0 and `device` behind the hub's port 0, with
/// the hub's own root port already enabled — which is what an EHCI does after
/// resetting it, and what these tests need so that routing finds anything.
fn tree(hub_speed: Speed, device: Arc<dyn UsbDevice>) -> (Arc<UsbBus>, UsbHub) {
    let root = Arc::new(UsbBus::new(1));
    let below = Arc::new(UsbBus::new(4));
    let hub = UsbHub::with_bus(Arc::clone(&below), 4, hub_speed, 0x1d6b, 0x0002);
    root.attach(0, hub.device()).expect("an empty port");
    root.set_enabled(0, true);
    below.attach(0, device).expect("an empty port");
    (root, hub)
}

/// The smallest possible device to put behind the hub: a device descriptor and
/// nothing else.
///
/// Written here rather than borrowing [`crate::dev::usb::hid`]'s mouse, because
/// a hub's tests must not need another *device* feature to be enabled — the
/// whole claim is that the hub does not know or care what is behind it, and a
/// test that only compiles with a mouse in the build says the opposite.
#[derive(Debug)]
struct Plain {
    descriptors: Descriptors,
    speed: Speed,
}

impl Function for Plain {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        self.speed
    }
}

/// One of those, wrapped in the standard control pipe, ready to be attached.
fn plain(vendor: u16, speed: Speed) -> Arc<crate::bus::usb::Peripheral> {
    let device = DeviceDescriptor {
        vendor,
        max_packet0: speed.max_control_packet() as u8,
        ..DeviceDescriptor::default()
    };
    let mut descriptors = Descriptors::new().with_device(&device);
    descriptors.add_configuration(&ConfigurationDescriptor::default(), &[]);
    let function = Arc::new(Plain { descriptors, speed });
    Arc::new(crate::bus::usb::Peripheral::new(
        function as Arc<dyn Function>,
    ))
}

/// A high-speed device behind a high-speed hub: the configuration that needs no
/// transaction translator.
fn matched() -> (Arc<UsbBus>, UsbHub, Arc<crate::bus::usb::Peripheral>) {
    let device = plain(0xdead, Speed::High);
    let (root, hub) = tree(Speed::High, Arc::clone(&device) as Arc<dyn UsbDevice>);
    (root, hub, device)
}

/// Take the hub from "just plugged in" to "port 1 is enabled and the device
/// behind it answers address zero", doing exactly what USB 2.0 §11 says a host
/// does and nothing else.
fn enumerate_hub(bus: &UsbBus) {
    assert!(
        put(
            bus,
            DeviceAddress::DEFAULT,
            setup(0, request::SET_ADDRESS, u16::from(HUB_ADDRESS.0), 0, 0)
        ),
        "SET_ADDRESS"
    );
    assert!(
        put(
            bus,
            HUB_ADDRESS,
            setup(0, request::SET_CONFIGURATION, 1, 0, 0)
        ),
        "SET_CONFIGURATION"
    );
    assert!(
        put(
            bus,
            HUB_ADDRESS,
            setup(
                TO_PORT,
                class_request::SET_FEATURE,
                feature::PORT_POWER,
                1,
                0
            )
        ),
        "SetPortFeature(PORT_POWER)"
    );
    assert!(
        put(
            bus,
            HUB_ADDRESS,
            setup(
                TO_PORT,
                class_request::CLEAR_FEATURE,
                feature::C_PORT_CONNECTION,
                1,
                0
            )
        ),
        "ClearPortFeature(C_PORT_CONNECTION)"
    );
    assert!(
        put(
            bus,
            HUB_ADDRESS,
            setup(
                TO_PORT,
                class_request::SET_FEATURE,
                feature::PORT_RESET,
                1,
                0
            )
        ),
        "SetPortFeature(PORT_RESET)"
    );
}

// ---------------------------------------------------------------------------
// Descriptors
// ---------------------------------------------------------------------------

#[test]
fn the_hub_descriptor_is_the_bytes_of_table_11_13() {
    let (root, _hub, _mouse) = matched();
    enumerate_hub(&root);
    let bytes = get(
        &root,
        HUB_ADDRESS,
        setup(
            FROM_HUB,
            class_request::GET_DESCRIPTOR,
            u16::from(DESC_HUB) << 8,
            0,
            64,
        ),
    )
    .expect("GetHubDescriptor");

    // Four ports, so the two bitmaps are one byte each: 7 + 2.
    assert_eq!(bytes.len(), 9);
    assert_eq!(bytes[0], 9, "bDescLength counts itself");
    assert_eq!(bytes[1], DESC_HUB);
    assert_eq!(bytes[2], 4, "bNbrPorts");
    let characteristics = u16::from_le_bytes([bytes[3], bytes[4]]);
    assert_eq!(characteristics & 0b11, 0b01, "per-port power switching");
    assert_eq!(characteristics & 0b100, 0, "not a compound device");
    assert_eq!(
        (characteristics >> 3) & 0b11,
        0b10,
        "no over-current protection, because none is modelled"
    );
    assert_eq!(bytes[5], POWER_ON_TO_GOOD);
    assert_eq!(bytes[7], 0, "DeviceRemovable: every port is a socket");
    assert_eq!(bytes[8], 0xff, "PortPwrCtrlMask is all ones (§11.23.2.1)");
}

#[test]
fn a_high_speed_hub_declares_a_single_transaction_translator_and_a_full_speed_one_does_not() {
    for (speed, protocol, interval) in [
        (Speed::High, PROTOCOL_SINGLE_TT, INTERVAL_HIGH),
        (Speed::Full, 0, INTERVAL_FULL),
    ] {
        let (root, _hub) = tree(speed, plain(0, speed) as Arc<dyn UsbDevice>);
        let device = get(
            &root,
            DeviceAddress::DEFAULT,
            crate::bus::usb::host::get_descriptor(1, 0, 18),
        )
        .expect("the device descriptor");
        assert_eq!(
            device[4], CLASS_HUB,
            "§11.23.1 puts the class on the device"
        );
        assert_eq!(device[6], protocol, "bDeviceProtocol");

        // And the endpoint descriptor's polling interval follows the speed,
        // because the field means microframes at one and frames at the other.
        let config = get(
            &root,
            DeviceAddress::DEFAULT,
            crate::bus::usb::host::get_descriptor(2, 0, 64),
        )
        .expect("the configuration");
        assert_eq!(
            *config.last().expect("bInterval is the last byte"),
            interval
        );
    }
}

// ---------------------------------------------------------------------------
// The port state machine (§11.5)
// ---------------------------------------------------------------------------

#[test]
fn an_unpowered_port_reports_no_connection_at_all() {
    let (root, hub, _mouse) = matched();
    assert!(
        put(
            &root,
            DeviceAddress::DEFAULT,
            setup(0, request::SET_ADDRESS, u16::from(HUB_ADDRESS.0), 0, 0)
        ),
        "SET_ADDRESS"
    );

    // There *is* a mouse on port 1, and §11.5.1.1's *Powered-off* state says
    // the port does not know: this is the same decision the EHCI makes about
    // `CONFIGFLAG`, and it is deliberately not papered over.
    let (status, change) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(status, 0, "no power, no connection, no speed");
    assert_eq!(change, 0, "and nothing to report");
    assert!(!hub.port_powered(0));

    // Powering it is what makes the connection appear, with the change bit
    // §11.24.2.7.2.1 asks for.
    assert!(put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::SET_FEATURE,
            feature::PORT_POWER,
            1,
            0
        )
    ));
    let (status, change) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(status & status::POWER, status::POWER);
    assert_eq!(status & status::CONNECTION, status::CONNECTION);
    assert_eq!(status & status::HIGH_SPEED, status::HIGH_SPEED);
    assert_eq!(status & status::ENABLE, 0, "a connection is not an enable");
    assert_eq!(change & change::CONNECTION, change::CONNECTION);
}

#[test]
fn a_reset_is_what_enables_a_port_and_it_reports_that_it_finished() {
    let (root, hub, _mouse) = matched();
    enumerate_hub(&root);

    let (status, change) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(status & status::ENABLE, status::ENABLE);
    assert_eq!(change & change::RESET, change::RESET, "C_PORT_RESET");
    assert_eq!(
        change & change::CONNECTION,
        0,
        "the host cleared that one before resetting"
    );
    assert!(hub.port_enabled(0));

    // And the change bit is cleared by the request that clears it, not by
    // having been read.
    assert!(put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::CLEAR_FEATURE,
            feature::C_PORT_RESET,
            1,
            0
        )
    ));
    let (_, change) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(change, 0);
}

#[test]
fn a_device_behind_the_hub_answers_a_control_transfer_addressed_to_it() {
    let (root, _hub, mouse) = matched();
    enumerate_hub(&root);

    // **The claim.** The mouse is on no root port — it is on the hub's port 1 —
    // and the only reason this reaches it is that `UsbBus::find` walks tiers.
    assert!(
        put(
            &root,
            DeviceAddress::DEFAULT,
            setup(0, request::SET_ADDRESS, u16::from(DEVICE_ADDRESS.0), 0, 0)
        ),
        "the device behind the hub took an address"
    );
    assert_eq!(mouse.address(), DEVICE_ADDRESS);

    let descriptor = get(
        &root,
        DEVICE_ADDRESS,
        crate::bus::usb::host::get_descriptor(1, 0, 18),
    )
    .expect("the device descriptor came back through the hub");
    assert_eq!(descriptor.len(), 18);
    assert_eq!(
        u16::from_le_bytes([descriptor[8], descriptor[9]]),
        0xdead,
        "idVendor: this is the mouse's descriptor and not the hub's"
    );

    // And the hub is still answering on its own address at the same time.
    let hub_descriptor = get(
        &root,
        HUB_ADDRESS,
        crate::bus::usb::host::get_descriptor(1, 0, 18),
    )
    .expect("the hub");
    assert_eq!(hub_descriptor[4], CLASS_HUB);
}

#[test]
fn clearing_port_enable_makes_the_device_unreachable_again() {
    let (root, _hub, _mouse) = matched();
    enumerate_hub(&root);
    assert!(put(
        &root,
        DeviceAddress::DEFAULT,
        setup(0, request::SET_ADDRESS, u16::from(DEVICE_ADDRESS.0), 0, 0)
    ));
    assert!(root.find(DEVICE_ADDRESS).is_some());

    assert!(put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::CLEAR_FEATURE,
            feature::PORT_ENABLE,
            1,
            0
        )
    ));
    assert!(
        root.find(DEVICE_ADDRESS).is_none(),
        "a disabled port routes nothing, exactly as a disabled root port does"
    );
}

#[test]
fn removing_port_power_takes_the_port_all_the_way_back() {
    let (root, hub, _mouse) = matched();
    enumerate_hub(&root);
    assert!(hub.port_enabled(0));

    assert!(put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::CLEAR_FEATURE,
            feature::PORT_POWER,
            1,
            0
        )
    ));
    let (status, _) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(
        status, 0,
        "§11.5.1.1: Powered-off is below every other state"
    );
    assert!(!hub.port_enabled(0));
    assert!(
        root.find(HUB_ADDRESS).is_some(),
        "the hub itself is still on"
    );
}

#[test]
fn a_port_number_of_zero_or_past_the_end_is_a_request_error() {
    let (root, _hub, _mouse) = matched();
    enumerate_hub(&root);
    // §11.24.2.7: port numbers are one-based, so zero is not a port.
    assert!(
        get(
            &root,
            HUB_ADDRESS,
            setup(FROM_PORT, request::GET_STATUS, 0, 0, 4)
        )
        .is_none()
    );
    assert!(
        get(
            &root,
            HUB_ADDRESS,
            setup(FROM_PORT, request::GET_STATUS, 0, 5, 4)
        )
        .is_none(),
        "this hub has four ports"
    );
    assert!(!put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::SET_FEATURE,
            feature::PORT_POWER,
            9,
            0
        )
    ));
}

#[test]
fn set_port_feature_refuses_to_enable_a_port_because_only_a_reset_may() {
    let (root, hub, _mouse) = matched();
    assert!(put(
        &root,
        DeviceAddress::DEFAULT,
        setup(0, request::SET_ADDRESS, u16::from(HUB_ADDRESS.0), 0, 0)
    ));
    assert!(put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::SET_FEATURE,
            feature::PORT_POWER,
            1,
            0
        )
    ));
    // §11.24.2.13 lists five settable port features and `PORT_ENABLE` is not
    // among them: a port becomes enabled because a reset succeeded.
    assert!(!put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_PORT,
            class_request::SET_FEATURE,
            feature::PORT_ENABLE,
            1,
            0
        )
    ));
    assert!(!hub.port_enabled(0));
}

// ---------------------------------------------------------------------------
// The speed the hub cannot carry
// ---------------------------------------------------------------------------

#[test]
fn a_full_speed_device_behind_a_high_speed_hub_does_not_enable() {
    let (root, hub) = tree(Speed::High, plain(0, Speed::Full) as Arc<dyn UsbDevice>);
    enumerate_hub(&root);

    // The reset happened and the hub says what is down there — it is not
    // pretending the port is empty.
    let (status, change) = port_status(&root, HUB_ADDRESS, 1);
    assert_eq!(status & status::CONNECTION, status::CONNECTION);
    assert_eq!(status & status::HIGH_SPEED, 0, "it is a full-speed device");
    assert_eq!(change & change::RESET, change::RESET);
    // And the port did not enable, because the split-transaction data path a
    // full-speed device would need through a high-speed hub does not exist.
    assert_eq!(status & status::ENABLE, 0);
    assert!(!hub.port_enabled(0));
    // So the device is unreachable — the *same* outcome a full-speed device on
    // a bare EHCI root port gets, arrived at through the port rather than
    // through a companion controller.
    assert!(root.find(HUB_ADDRESS).is_some(), "the hub is still there");
    assert_eq!(
        root.setup(
            DeviceAddress::DEFAULT,
            0,
            setup(0, request::GET_STATUS, 0, 0, 2)
        ),
        Status::NoDevice,
        "and nothing behind it answers"
    );
}

#[test]
fn a_full_speed_device_behind_a_full_speed_hub_enumerates_completely() {
    let (root, hub) = tree(
        Speed::Full,
        plain(0x1234, Speed::Full) as Arc<dyn UsbDevice>,
    );
    enumerate_hub(&root);
    assert!(
        hub.port_enabled(0),
        "a full-speed hub carrying a full-speed device needs no translator"
    );

    assert!(put(
        &root,
        DeviceAddress::DEFAULT,
        setup(0, request::SET_ADDRESS, u16::from(DEVICE_ADDRESS.0), 0, 0)
    ));
    let descriptor = get(
        &root,
        DEVICE_ADDRESS,
        crate::bus::usb::host::get_descriptor(1, 0, 18),
    )
    .expect("through the full-speed hub");
    assert_eq!(u16::from_le_bytes([descriptor[8], descriptor[9]]), 0x1234);
    assert_eq!(
        descriptor[6], 0,
        "and it is not a hub descriptor that came back"
    );
}

// ---------------------------------------------------------------------------
// The status change endpoint (§11.12.4)
// ---------------------------------------------------------------------------

#[test]
fn the_status_change_endpoint_naks_until_something_changes_and_a_peek_never_consumes() {
    let (root, _hub, _mouse) = matched();
    enumerate_hub(&root);
    // Clear everything the enumeration left.
    for selector in [feature::C_PORT_CONNECTION, feature::C_PORT_RESET] {
        assert!(put(
            &root,
            HUB_ADDRESS,
            setup(TO_PORT, class_request::CLEAR_FEATURE, selector, 1, 0)
        ));
    }
    let mut packet = [0u8; 8];
    assert_eq!(
        root.read(HUB_ADDRESS, ENDPOINT, &mut packet).status,
        Status::Nak,
        "§11.12.4: no change, so the hub NAKs rather than sending zero bytes"
    );

    // Now unplug something. The change is discovered by the *next* transaction,
    // because a hub with no clock samples when it is spoken to.
    assert!(root.find(HUB_ADDRESS).is_some());
    let below = _hub.downstream();
    assert!(below.detach(0));
    let completion = root.read(HUB_ADDRESS, ENDPOINT, &mut packet);
    assert_eq!(completion.status, Status::Ack);
    assert_eq!(completion.len, 1, "five bits fit in one byte");
    assert_eq!(packet[0], 0b10, "bit 0 is the hub, bit 1 is port 1");

    // Reading it again gives the same answer: the bitmap is the change bits and
    // an `IN` does not clear them — only `ClearPortFeature(C_…)` does. So the
    // debug path is the same path, and it is checked as one.
    let mut again = [0u8; 8];
    assert_eq!(root.read(HUB_ADDRESS, ENDPOINT, &mut again).len, 1);
    assert_eq!(again[0], packet[0]);
    let mut peeked = [0u8; 8];
    assert_eq!(root.peek(HUB_ADDRESS, ENDPOINT, &mut peeked).len, 1);
    assert_eq!(peeked[0], packet[0]);
}

#[test]
fn a_debug_peek_does_not_sample_the_fabric_or_finish_a_reset() {
    let (root, hub, _mouse) = matched();
    assert!(put(
        &root,
        DeviceAddress::DEFAULT,
        setup(0, request::SET_ADDRESS, u16::from(HUB_ADDRESS.0), 0, 0)
    ));
    // Powering the port is the transaction that discovers the connection; do
    // not do it, so the hub's mirror still says "nothing there".
    let mut packet = [0u8; 8];
    assert_eq!(
        root.peek(HUB_ADDRESS, ENDPOINT, &mut packet).status,
        Status::Nak,
        "a debug read must not go and look"
    );
    assert!(!hub.port_powered(0));
    assert!(!hub.port_enabled(0));
    assert!(
        !hub.downstream().enabled(0),
        "and it must not have enabled anything in the fabric either"
    );
}

// ---------------------------------------------------------------------------
// The transaction translator's requests
// ---------------------------------------------------------------------------

#[test]
fn the_tt_requests_are_answered_by_a_high_speed_hub_and_refused_by_a_full_speed_one() {
    for (speed, accepted) in [(Speed::High, true), (Speed::Full, false)] {
        let (root, _hub) = tree(speed, plain(0, speed) as Arc<dyn UsbDevice>);
        enumerate_hub(&root);
        for request in [
            class_request::CLEAR_TT_BUFFER,
            class_request::RESET_TT,
            class_request::STOP_TT,
        ] {
            assert_eq!(
                put(&root, HUB_ADDRESS, setup(TO_PORT, request, 0, 1, 0)),
                accepted,
                "a hub has a transaction translator exactly when it is high speed"
            );
        }
        // `GetTTState`'s bytes are implementation defined (§11.24.2.4), so
        // inventing them would be inventing a TT's internals.
        assert!(
            get(
                &root,
                HUB_ADDRESS,
                setup(FROM_PORT, class_request::GET_TT_STATE, 0, 1, 4)
            )
            .is_none()
        );
    }
}

#[test]
fn set_hub_feature_is_refused_because_a_hub_has_no_settable_feature() {
    let (root, _hub, _mouse) = matched();
    enumerate_hub(&root);
    // §11.24.2.12: the two hub selectors are *change* bits, and `SetHubFeature`
    // may not set a change bit.
    assert!(!put(
        &root,
        HUB_ADDRESS,
        setup(
            TO_HUB,
            class_request::SET_FEATURE,
            feature::C_HUB_LOCAL_POWER,
            0,
            0
        )
    ));
    // Its status is still readable and still says everything is healthy.
    let bytes = get(
        &root,
        HUB_ADDRESS,
        setup(FROM_HUB, request::GET_STATUS, 0, 0, 4),
    )
    .expect("GetHubStatus");
    assert_eq!(bytes, alloc::vec![0, 0, 0, 0]);
}

// ---------------------------------------------------------------------------
// Topologies a machine description can build and hardware cannot
// ---------------------------------------------------------------------------

#[test]
fn two_hubs_naming_each_others_bus_is_a_bad_topology_and_not_a_hang() {
    // Neither hub can detect this at construction: each one's `downstream` is a
    // different name from its own `bus`. What stops it is the bound on the
    // routing walk, and this is the test that keeps that bound load-bearing.
    let a_bus = Arc::new(UsbBus::new(1));
    let b_bus = Arc::new(UsbBus::new(1));
    let a = UsbHub::with_bus(Arc::clone(&b_bus), 1, Speed::High, 0, 0);
    let b = UsbHub::with_bus(Arc::clone(&a_bus), 1, Speed::High, 0, 0);
    a_bus.attach(0, a.device()).expect("empty");
    a_bus.set_enabled(0, true);
    b_bus.attach(0, b.device()).expect("empty");
    b_bus.set_enabled(0, true);

    // Both hubs answer address zero, so the *first* tier matches and the walk
    // never has to descend. Ask for an address nothing has, which is what makes
    // the walk go all the way down a cycle.
    assert!(a_bus.find(DeviceAddress(42)).is_none());
    a_bus.start_of_frame(0);

    // And it is refused at realize, which is the phase that can see it: by then
    // both hubs exist, so walking down from one arrives back at one. Refusing
    // is worth doing even though routing survives, because the `Arc` cycle
    // means nothing in it is ever freed.
    let error = a.check_topology().expect_err("a cycle is not a topology");
    assert!(alloc::format!("{error}").contains("cycle"));
    assert!(b.check_topology().is_err(), "and from the other side too");

    // Untangled, both are ordinary hubs again.
    a_bus.detach(0);
    assert!(b.check_topology().is_ok());
}

#[test]
fn an_ordinary_tree_passes_the_topology_check() {
    let (root, hub, _mouse) = matched();
    assert!(hub.check_topology().is_ok(), "a hub with a mouse behind it");

    // Including a legal chain of hubs, which looks like a cycle to any check
    // that only counted tiers instead of recognising itself.
    let below = Arc::new(UsbBus::new(1));
    let second = UsbHub::with_bus(Arc::clone(&below), 1, Speed::High, 0, 0);
    hub.downstream()
        .attach(2, second.device())
        .expect("an empty port");
    assert!(hub.check_topology().is_ok());
    assert!(second.check_topology().is_ok());
    let _ = root;
}

#[test]
fn a_hub_may_be_nested_to_the_depth_the_specification_allows() {
    // Five hubs in series is the limit of USB 2.0 §4.1.1, and a device behind
    // the fifth is at tier 7 — reachable. This builds exactly that.
    let hubs = usize::from(MAX_TIERS) - 2;
    let root = Arc::new(UsbBus::new(1));
    let mut buses = alloc::vec![Arc::clone(&root)];
    let mut chain = Vec::new();
    for _ in 0..hubs {
        let below = Arc::new(UsbBus::new(1));
        let hub = UsbHub::with_bus(Arc::clone(&below), 1, Speed::High, 0, 0);
        let above = buses.last().expect("the chain starts at the root");
        above.attach(0, hub.device()).expect("empty");
        above.set_enabled(0, true);
        buses.push(Arc::clone(&below));
        chain.push(hub);
    }
    let mouse = plain(0xcafe, Speed::High);
    let last = buses.last().expect("the deepest bus");
    last.attach(0, Arc::clone(&mouse) as Arc<dyn UsbDevice>)
        .expect("empty");
    last.set_enabled(0, true);

    // Every hub answers address zero, so give the mouse one of its own by
    // reaching it where it is, and then talk to it from the root.
    let mut xfer = ControlTransfer::host_to_device(
        setup(0, request::SET_ADDRESS, u16::from(DEVICE_ADDRESS.0), 0, 0),
        &[],
    );
    while !xfer.is_finished() {
        xfer.step(last, DeviceAddress::DEFAULT, 64);
    }
    assert_eq!(mouse.address(), DEVICE_ADDRESS);

    let descriptor = get(
        &root,
        DEVICE_ADDRESS,
        crate::bus::usb::host::get_descriptor(1, 0, 18),
    )
    .expect("a device at the deepest tier the specification allows is reachable");
    assert_eq!(u16::from_le_bytes([descriptor[8], descriptor[9]]), 0xcafe);

    // One tier further is not, and it is a miss rather than a hang.
    let deeper = Arc::new(UsbBus::new(1));
    let extra = UsbHub::with_bus(Arc::clone(&deeper), 1, Speed::High, 0, 0);
    last.detach(0);
    last.attach(0, extra.device()).expect("empty");
    let beyond = plain(0, Speed::High);
    deeper
        .attach(0, Arc::clone(&beyond) as Arc<dyn UsbDevice>)
        .expect("empty");
    deeper.set_enabled(0, true);
    let mut xfer = ControlTransfer::host_to_device(
        setup(0, request::SET_ADDRESS, u16::from(DeviceAddress(9).0), 0, 0),
        &[],
    );
    while !xfer.is_finished() {
        xfer.step(&deeper, DeviceAddress::DEFAULT, 64);
    }
    assert_eq!(beyond.address(), DeviceAddress(9));
    assert!(root.find(DeviceAddress(9)).is_none());
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn a_hub_whose_downstream_bus_is_its_own_upstream_is_refused() {
    let props = Props::new().with("bus", "usb0").with("downstream", "usb0");
    let error = UsbHub::new(&props).expect_err("a hub plugged into itself");
    assert!(
        alloc::format!("{error}").contains("cycle"),
        "the message has to say what is wrong: {error}"
    );
}

#[test]
fn there_is_no_low_speed_hub() {
    let props = Props::new()
        .with("bus", "usb0")
        .with("downstream", "usb1")
        .with("speed", "low");
    let error = UsbHub::new(&props).expect_err("USB 2.0 §11.1 has no such thing");
    assert!(alloc::format!("{error}").contains("low-speed hub"));
}

#[test]
fn a_hub_lands_on_the_bus_the_machine_description_names() {
    // A `Props` with a build behind it, because both of this device's buses are
    // found by *name* in that build's table — which is the whole reason a
    // machine file can put a disk behind a hub without either object naming the
    // other.
    let hosts = Arc::new(crate::core::HostObjects::new());
    let props = Props::new()
        .with_hosts(Arc::clone(&hosts))
        .with("bus", "usb0")
        .with("port", 1u64)
        .with("downstream", "usb1")
        .with("ports", 2u64);
    let hub = UsbHub::new(&props).expect("it builds");
    assert_eq!(hub.ports(), 2);

    let root = buses::open(&hosts, "usb0", 2).expect("the same table");
    assert!(root.connected(1), "on the port it asked for");
    assert!(!root.connected(0));
    let below = buses::open(&hosts, "usb1", 2).expect("the same table");
    assert!(
        Arc::ptr_eq(&below, hub.downstream()),
        "`usb1` is the hub's own ports, and a device naming it plugs in behind"
    );
    assert_eq!(below.port_count(), 2);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_snapshot_taken_mid_enumeration_restores_to_the_same_bytes() {
    let (root, hub, _mouse) = matched();
    enumerate_hub(&root);
    // Leave a half-finished control transfer in the pipe: the hub answered the
    // `SETUP` of a `GetPortStatus` and the host has not collected it yet.
    assert_eq!(
        root.setup(
            HUB_ADDRESS,
            0,
            setup(FROM_PORT, request::GET_STATUS, 0, 1, 4)
        ),
        Status::Ack
    );

    let mut first = Vec::new();
    hub.save_state(&mut first).expect("it saves");

    let (other_root, other) = tree(
        Speed::High,
        plain(0xdead, Speed::High) as Arc<dyn UsbDevice>,
    );
    other
        .load_state(&mut ChunkReader::new(&first))
        .expect("it loads");
    let mut second = Vec::new();
    other.save_state(&mut second).expect("it saves again");
    assert_eq!(first, second, "the state hash must be identical");

    // And the restored hub is *usable*: the port is enabled again in the
    // fabric, which is derived state the loader re-derives rather than stores.
    assert!(other.port_enabled(0));
    assert!(other.downstream().enabled(0));
    let mut packet = [0u8; 8];
    assert_eq!(
        other_root.read(HUB_ADDRESS, 0, &mut packet).len,
        4,
        "the half-finished GetPortStatus finishes after the round trip"
    );
}

#[test]
fn a_snapshot_with_the_wrong_number_of_ports_is_refused() {
    let (root, hub, _mouse) = matched();
    enumerate_hub(&root);
    let mut bytes = Vec::new();
    hub.save_state(&mut bytes).expect("it saves");

    let below = Arc::new(UsbBus::new(2));
    let other = UsbHub::with_bus(Arc::clone(&below), 2, Speed::High, 0, 0);
    below
        .attach(0, plain(0, Speed::High) as Arc<dyn UsbDevice>)
        .expect("empty");
    let error = other
        .load_state(&mut ChunkReader::new(&bytes))
        .expect_err("four ports into a two-port hub");
    assert!(alloc::format!("{error}").contains("ports"));
}
