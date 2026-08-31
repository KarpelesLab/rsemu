//! A USB HID boot-protocol mouse: the smallest honest proof that the stack
//! works.
//!
//! # Why a mouse, and why high speed
//!
//! A device that only enumerates proves the control pipe. A device that only
//! moves bulk data proves the DMA walk. A HID mouse does both and is small: it
//! enumerates through every one of the standard requests, it answers a
//! class-specific `GET_DESCRIPTOR` for its report descriptor, and then it sits
//! on an **interrupt IN endpoint** that the host controller polls once a
//! millisecond out of the periodic schedule — which is the half of an EHCI that
//! nothing else in this tree would exercise.
//!
//! It is a **high-speed** device, and that is a deliberate decision rather than
//! a convenience. Real mice are low speed, and a low-speed device cannot be
//! reached by an EHCI controller at all: it needs a transaction translator
//! inside a hub, or a companion controller (EHCI 1.0 §4.2). rsemu has neither,
//! and its EHCI does the honest thing with one — it hands the port to a
//! companion and the device vanishes. So a low-speed mouse here would be a
//! device that never enumerates, dressed up as a feature. USB 2.0 permits a
//! high-speed HID device (§5.7 sets the interrupt endpoint's service interval
//! in microframes for exactly that case), so this one is high speed and says so
//! in its own descriptors.
//!
//! # Where the movement comes from
//!
//! [`HidMouse::motion`], and nowhere else. There is no host input seam here,
//! deliberately: a real pointer's movements are a **non-deterministic input
//! crossing into the machine**, and `CLAUDE.md` requires those to go through the
//! record/replay seam, which does not exist yet. Until it does, the only thing
//! that moves this mouse is a caller that knows what virtual time it is — a
//! test, or an embedder that has thought about it.
//!
//! # Sources
//!
//! **USB 2.0** §9 for the device framework, the **Device Class Definition for
//! Human Interface Devices (HID), version 1.11** — §6.2.1 for the HID
//! descriptor, §7.2 for the class requests, and Appendix E.10, whose report
//! descriptor for a three-button relative mouse is reproduced below — and the
//! **HID Usage Tables** for the usage pages. All three are free downloads from
//! usb.org.

use alloc::boxed::Box;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{
    Completion, ConfigurationDescriptor, Descriptors, DeviceDescriptor, Direction,
    EndpointDescriptor, Function, InterfaceDescriptor, Peripheral, SetupPacket, Speed,
    TransferType, UsbDevice, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::Result;
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.mouse";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// `bInterfaceClass` for HID (USB 2.0 §9.6.5 and the HID specification's own
/// §4.1).
const CLASS_HID: u8 = 3;
/// `bInterfaceSubClass` 1: this interface supports the boot protocol
/// (HID 1.11 §4.2).
const SUBCLASS_BOOT: u8 = 1;
/// `bInterfaceProtocol` 2: a mouse (HID 1.11 §4.3).
const PROTOCOL_MOUSE: u8 = 2;

/// The HID class descriptor type (HID 1.11 §7.1.1).
const DESC_HID: u8 = 0x21;
/// The report descriptor type.
const DESC_REPORT: u8 = 0x22;

/// The HID class requests this device answers (HID 1.11 §7.2).
///
/// Public because a host-side test or an embedder driving this device has to
/// spell the same numbers, and two copies of a constant are one copy too many.
pub mod class_request {
    /// §7.2.1. Returns the current report without touching the interrupt
    /// endpoint's queue.
    pub const GET_REPORT: u8 = 0x01;
    /// §7.2.3.
    pub const GET_IDLE: u8 = 0x02;
    /// §7.2.5.
    pub const GET_PROTOCOL: u8 = 0x03;
    /// §7.2.2. Output reports — a keyboard's LEDs. A mouse has none.
    pub const SET_REPORT: u8 = 0x09;
    /// §7.2.4.
    pub const SET_IDLE: u8 = 0x0a;
    /// §7.2.6.
    pub const SET_PROTOCOL: u8 = 0x0b;
}

/// The interrupt IN endpoint's number.
const ENDPOINT: u8 = 1;

/// A boot-protocol mouse report is three bytes: buttons, then relative X and Y
/// (HID 1.11 Appendix B.2).
pub const REPORT_BYTES: usize = 3;

/// `bInterval` for the interrupt endpoint.
///
/// At high speed the service interval is `2^(bInterval - 1)` microframes
/// (USB 2.0 §9.6.6), so `4` is eight microframes, which is exactly one
/// millisecond — the rate a boot mouse is polled at.
const INTERVAL: u8 = 4;

/// The report descriptor of HID 1.11 Appendix E.10: a three-button mouse with
/// relative X and Y.
///
/// Reproduced from the specification's own worked example, which is what every
/// boot mouse in existence carries. The bytes are a *format*, not an
/// implementation — this is the same kind of fact as a register layout.
pub const REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xa1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Pointer)
    0xa1, 0x00, //   Collection (Physical)
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (Button 1)
    0x29, 0x03, //     Usage Maximum (Button 3)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x95, 0x03, //     Report Count (3)
    0x75, 0x01, //     Report Size (1)
    0x81, 0x02, //     Input (Data, Variable, Absolute)
    0x95, 0x01, //     Report Count (1)
    0x75, 0x05, //     Report Size (5)
    0x81, 0x01, //     Input (Constant) — five bits of padding
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7f, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    0xc0, //   End Collection
    0xc0, // End Collection
];

/// Everything the mouse remembers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MouseState {
    /// A report waiting to be collected by the next `IN` on the interrupt
    /// endpoint, if there is one. `None` is what makes an idle mouse `NAK`,
    /// which is the whole shape of an interrupt endpoint.
    pending: Option<[u8; REPORT_BYTES]>,
    /// Which buttons are down, so `GET_REPORT` can answer with the current
    /// state rather than with movement that has already been delivered.
    buttons: u8,
    /// The `SET_IDLE` duration, in 4 ms units. Stored and reported; **not
    /// acted on** — a nonzero idle rate asks the device to repeat a report
    /// periodically, and repeating it would need a clock domain this device
    /// does not have and no host in this tree asks for.
    idle: u8,
    /// `0` boot protocol, `1` report protocol (HID 1.11 §7.2.6). Both produce
    /// the same three bytes here, because the report descriptor above *is* the
    /// boot layout.
    protocol: u8,
}

/// The mouse's class-specific half.
struct MouseFunction {
    descriptors: Descriptors,
    state: Mutex<MouseState>,
}

impl fmt::Debug for MouseFunction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("MouseFunction");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl MouseFunction {
    fn new(vendor: u16, product: u16) -> MouseFunction {
        let device = DeviceDescriptor {
            usb: 0x0200,
            // Zero: the class is on the interface, which is what every
            // composite-capable device does and what HID requires (§5.1).
            class: 0,
            subclass: 0,
            protocol: 0,
            // High speed requires exactly 64 (USB 2.0 §5.5.3).
            max_packet0: 64,
            vendor,
            product,
            device: 0x0100,
            manufacturer: 0,
            product_name: 0,
            serial: 0,
            configurations: 1,
        };

        let interface = InterfaceDescriptor {
            number: 0,
            alternate: 0,
            endpoints: 1,
            class: CLASS_HID,
            subclass: SUBCLASS_BOOT,
            protocol: PROTOCOL_MOUSE,
            name: 0,
        };
        let endpoint = EndpointDescriptor {
            address: ENDPOINT | Direction::BIT,
            attributes: TransferType::Interrupt.attribute_bits(),
            max_packet: REPORT_BYTES as u16,
            interval: INTERVAL,
        };

        let mut body = Vec::new();
        body.extend_from_slice(&interface.encode());
        body.extend_from_slice(&hid_descriptor());
        body.extend_from_slice(&endpoint.encode());

        let mut descriptors = Descriptors::new().with_device(&device);
        descriptors.add_configuration(
            &ConfigurationDescriptor {
                interfaces: 1,
                value: 1,
                name: 0,
                attributes: ConfigurationDescriptor::REMOTE_WAKEUP,
                // 100 mA, in the 2 mA units the field is counted in.
                max_power: 50,
            },
            &body,
        );
        // A high-speed device is required to have one (USB 2.0 §9.6.2): it is
        // what it would look like at full speed, and a host asks for it.
        descriptors.set_qualifier(&device, 0);

        MouseFunction {
            descriptors,
            state: Mutex::with_rank(LockRank::DEVICE, MouseState::default()),
        }
    }

    /// The report a `GET_REPORT` answers with: the buttons as they stand, and
    /// no movement.
    ///
    /// Movement is *relative*, so reporting it again through the control pipe
    /// would move the pointer twice.
    fn current_report(&self) -> [u8; REPORT_BYTES] {
        [self.state.lock().buttons, 0, 0]
    }
}

/// The nine bytes of the HID descriptor (HID 1.11 §6.2.1, table 6-1).
fn hid_descriptor() -> [u8; 9] {
    let len = (REPORT_DESCRIPTOR.len() as u16).to_le_bytes();
    [
        9,
        DESC_HID,
        // bcdHID 1.11.
        0x11,
        0x01,
        // bCountryCode: not localised.
        0x00,
        // bNumDescriptors: one, the report descriptor.
        0x01,
        DESC_REPORT,
        len[0],
        len[1],
    ]
}

impl Function for MouseFunction {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        Speed::High
    }

    fn reset(&self) {
        *self.state.lock() = MouseState::default();
    }

    fn control_in(&self, setup: SetupPacket) -> Option<Vec<u8>> {
        // The class-specific descriptors arrive as a standard
        // `GET_DESCRIPTOR` addressed to the interface (HID 1.11 §7.1.1), which
        // `Endpoint0` forwards here because it is not a type the device
        // framework defines.
        if setup.request == crate::bus::usb::request::GET_DESCRIPTOR {
            let (kind, index) = setup.descriptor();
            return match (kind, index) {
                (DESC_REPORT, 0) => Some(REPORT_DESCRIPTOR.to_vec()),
                (DESC_HID, 0) => Some(hid_descriptor().to_vec()),
                _ => None,
            };
        }
        match setup.request {
            class_request::GET_REPORT => Some(self.current_report().to_vec()),
            class_request::GET_IDLE => Some(alloc::vec![self.state.lock().idle]),
            class_request::GET_PROTOCOL => Some(alloc::vec![self.state.lock().protocol]),
            _ => None,
        }
    }

    fn control_out(&self, setup: SetupPacket, data: &[u8]) -> bool {
        match setup.request {
            // §7.2.4: the duration is the high byte of `wValue`.
            class_request::SET_IDLE => {
                self.state.lock().idle = (setup.value >> 8) as u8;
                true
            }
            // §7.2.6: the low byte selects boot or report protocol.
            class_request::SET_PROTOCOL => {
                self.state.lock().protocol = (setup.value & 1) as u8;
                true
            }
            // §7.2.2: a mouse has no output report, and accepting one that
            // changes nothing is what the specification asks for — refusing
            // would fail a host that sets a report on every HID device it sees.
            class_request::SET_REPORT => {
                let _ = data;
                true
            }
            _ => false,
        }
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT {
            return Completion::stall();
        }
        let mut state = self.state.lock();
        // Nothing to say. `NAK` rather than a zero-length packet: an interrupt
        // endpoint with no new data *is* a NAK (USB 2.0 §8.5.4), and a host
        // that saw a zero-length packet would retire its transfer descriptor
        // and stop polling.
        let Some(report) = state.pending.take() else {
            return Completion::nak();
        };
        let n = report.len().min(dst.len());
        dst[..n].copy_from_slice(&report[..n]);
        Completion::ack(n as u64)
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT {
            return Completion::stall();
        }
        // The debug path: show the queued report without taking it
        // (`ROADMAP.md` §15, invariant 5).
        let state = self.state.lock();
        let Some(report) = state.pending else {
            return Completion::nak();
        };
        let n = report.len().min(dst.len());
        dst[..n].copy_from_slice(&report[..n]);
        Completion::ack(n as u64)
    }
}

/// A USB HID boot-protocol mouse.
#[derive(Debug)]
pub struct HidMouse {
    peripheral: Arc<Peripheral>,
    function: Arc<MouseFunction>,
}

impl HidMouse {
    /// Validate `props` and build the mouse.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`](crate::bus::usb::UsbBus) to plug into. Required.
    /// * `port` — which port of it. Defaults to 0.
    /// * `vendor`, `product` — what the device descriptor reports. Default to
    ///   zero, which is what an unbranded device says.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::Error::Property) for an unknown or missing property, and whatever
    /// [`UsbBus::attach`](crate::bus::usb::UsbBus::attach) refuses — a port that does not exist, or one that
    /// already has something in it.
    pub fn new(props: &Props) -> Result<HidMouse> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let port = r.or_range("port", 0u64, 0..=u64::from(u8::MAX))?;
        let vendor = r.or_range("vendor", 0u64, 0..=u64::from(u16::MAX))?;
        let product = r.or_range("product", 0u64, 0..=u64::from(u16::MAX))?;
        r.finish()?;

        // Opening the table entry creates nothing anybody can see; a bus named
        // by a device before its controller comes up with one port, and the
        // controller then finds it too small, which it reports.
        let bus = buses::open(&bus_name, port as u8 + 1);
        let mouse = HidMouse::new_detached(vendor as u16, product as u16);
        bus.attach(port as u8, mouse.device())?;
        Ok(mouse)
    }

    /// A mouse plugged into nothing.
    ///
    /// For a test, or an embedder that owns its own [`UsbBus`](crate::bus::usb::UsbBus) and attaches the
    /// device itself with [`HidMouse::device`].
    #[must_use]
    pub fn new_detached(vendor: u16, product: u16) -> HidMouse {
        let function = Arc::new(MouseFunction::new(vendor, product));
        let peripheral = Arc::new(Peripheral::new(Arc::clone(&function) as Arc<dyn Function>));
        HidMouse {
            peripheral,
            function,
        }
    }

    /// The mouse as the fabric sees it, for [`UsbBus::attach`](crate::bus::usb::UsbBus::attach).
    #[must_use]
    pub fn device(&self) -> Arc<dyn UsbDevice> {
        Arc::clone(&self.peripheral) as Arc<dyn UsbDevice>
    }

    /// The address the host has given it, or zero before enumeration.
    #[must_use]
    pub fn address(&self) -> crate::bus::usb::DeviceAddress {
        self.peripheral.address()
    }

    /// The configuration the host selected, or zero.
    #[must_use]
    pub fn configuration(&self) -> u8 {
        self.peripheral.endpoint0().configuration()
    }

    /// Move the pointer by `(dx, dy)` with `buttons` held, queueing a report
    /// for the next poll.
    ///
    /// Relative, and signed, as HID 1.11 Appendix E.10's descriptor says: the
    /// range is -127 to 127.
    ///
    /// **The only way movement enters the machine.** A report that has not been
    /// collected is replaced rather than queued, which is what a device with a
    /// single report buffer does — and it means a caller that moves the mouse
    /// faster than the host polls loses the intermediate positions, exactly as
    /// real hardware would.
    pub fn motion(&self, dx: i8, dy: i8, buttons: u8) {
        let mut state = self.function.state.lock();
        state.buttons = buttons & 0x7;
        state.pending = Some([state.buttons, dx as u8, dy as u8]);
    }

    /// Whether a report is waiting to be collected.
    #[must_use]
    pub fn has_report(&self) -> bool {
        self.function.state.lock().pending.is_some()
    }

    /// The report descriptor this device hands a host.
    #[must_use]
    pub fn report_descriptor(&self) -> &'static [u8] {
        REPORT_DESCRIPTOR
    }
}

impl Device for HidMouse {
    fn class(&self) -> &'static DeviceClass {
        &MOUSE_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the mouse plugged itself into the named bus at
        // construction, which is the rendezvous table and not an observable
        // action.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        self.peripheral.bus_reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The control pipe first: a control transfer part-way through is
        // state, and a snapshot taken in the middle of enumeration has to
        // resume into the same half-read descriptor.
        self.peripheral.endpoint0().save(w)?;
        let state = *self.function.state.lock();
        w.write_bool(state.pending.is_some())?;
        let report = state.pending.unwrap_or([0; REPORT_BYTES]);
        w.write_all(&report)?;
        w.write_u8(state.buttons)?;
        w.write_u8(state.idle)?;
        w.write_u8(state.protocol)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.peripheral.endpoint0().load(r)?;
        let has_report = r.read_bool()?;
        let mut report = [0u8; REPORT_BYTES];
        report.copy_from_slice(r.take(REPORT_BYTES)?);
        let state = MouseState {
            pending: has_report.then_some(report),
            buttons: r.read_u8()?,
            idle: r.read_u8()?,
            protocol: r.read_u8()?,
        };
        *self.function.state.lock() = state;
        Ok(())
    }
}

impl Instance for HidMouse {}

/// The `usb.mouse` device class.
pub static MOUSE_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a USB HID boot-protocol mouse: high speed, three buttons, relative X and Y on an \
              interrupt endpoint polled once a millisecond",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the named USB bus to plug into",
        },
        PropertySpec {
            name: "port",
            kind: ValueKind::Uint,
            required: false,
            summary: "which port of that bus (default 0)",
        },
        PropertySpec {
            name: "vendor",
            kind: ValueKind::Uint,
            required: false,
            summary: "idVendor, as the device descriptor reports it (default 0)",
        },
        PropertySpec {
            name: "product",
            kind: ValueKind::Uint,
            required: false,
            summary: "idProduct (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(HidMouse::new(props)?)),
};

/// Add [`MOUSE_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::Error::Config) if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&MOUSE_CLASS)
}

/// Bind [`MOUSE_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::Error::Config) if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(HidMouse::new(props)?)))
}

/// What the validator should know about `usb.mouse`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("port", ValueKind::Uint).range(0, u64::from(u8::MAX)))
        .prop(PropSchema::new("vendor", ValueKind::Uint).range(0, u64::from(u16::MAX)))
        .prop(PropSchema::new("product", ValueKind::Uint).range(0, u64::from(u16::MAX)))
}

// A mouse that could not be built is a mouse nobody can debug, so the one
// construction error worth naming here is named.
const _: () = {
    assert!(REPORT_BYTES == 3);
};

#[cfg(test)]
impl HidMouse {
    /// Save into a plain byte vector, for a round-trip test.
    fn save_to(&self, out: &mut Vec<u8>) -> Result<()> {
        self.peripheral.endpoint0().save(out)?;
        let state = *self.function.state.lock();
        out.write_bool(state.pending.is_some())?;
        out.write_all(&state.pending.unwrap_or([0; REPORT_BYTES]))?;
        out.write_u8(state.buttons)?;
        out.write_u8(state.idle)?;
        out.write_u8(state.protocol)
    }

    /// Load what [`HidMouse::save_to`] wrote.
    fn load_from(&self, bytes: &[u8]) -> Result<()> {
        let mut r = ChunkReader::new(bytes);
        self.load(&mut r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::usb::{DeviceAddress, Status, UsbBus, request};

    /// A mouse on a one-port bus, with the port already enabled — which is
    /// what a controller does after a successful reset, and what these tests
    /// need so that routing by address finds anything.
    fn plugged() -> (Arc<UsbBus>, HidMouse) {
        let bus = Arc::new(UsbBus::new(1));
        let mouse = HidMouse::new_detached(0x1234, 0x5678);
        bus.attach(0, mouse.device()).expect("an empty port");
        bus.set_enabled(0, true);
        (bus, mouse)
    }

    /// Run one control transfer in the device-to-host direction, the way a
    /// host controller would: a `SETUP` transaction, then `IN` packets until
    /// the device runs out, then the zero-length `OUT` status stage.
    fn control_in(bus: &UsbBus, address: DeviceAddress, setup: SetupPacket) -> Vec<u8> {
        assert_eq!(bus.setup(address, 0, setup), Status::Ack);
        let mut out = Vec::new();
        loop {
            let mut packet = [0u8; 64];
            let completion = bus.read(address, 0, &mut packet);
            assert_eq!(completion.status, Status::Ack, "a data-stage IN");
            let n = completion.len as usize;
            out.extend_from_slice(&packet[..n]);
            if n < packet.len() {
                break;
            }
        }
        assert_eq!(
            bus.write(address, 0, &[]).status,
            Status::Ack,
            "status stage"
        );
        out
    }

    /// The same, host to device with no data stage: a `SETUP` and a
    /// zero-length `IN`.
    fn control_out(bus: &UsbBus, address: DeviceAddress, setup: SetupPacket) -> Status {
        assert_eq!(bus.setup(address, 0, setup), Status::Ack);
        bus.read(address, 0, &mut []).status
    }

    fn get_descriptor(kind: u8, index: u8, length: u16) -> SetupPacket {
        SetupPacket {
            request_type: 0x80,
            request: request::GET_DESCRIPTOR,
            value: (u16::from(kind) << 8) | u16::from(index),
            index: 0,
            length,
        }
    }

    #[test]
    fn the_device_descriptor_is_what_the_spec_says_it_is() {
        let (bus, _mouse) = plugged();
        let bytes = control_in(&bus, DeviceAddress::DEFAULT, get_descriptor(1, 0, 18));
        assert_eq!(bytes.len(), 18);
        assert_eq!(bytes[0], 18, "bLength");
        assert_eq!(bytes[1], 1, "bDescriptorType");
        assert_eq!(u16::from_le_bytes([bytes[2], bytes[3]]), 0x0200, "bcdUSB");
        assert_eq!(bytes[7], 64, "high speed requires bMaxPacketSize0 = 64");
        assert_eq!(u16::from_le_bytes([bytes[8], bytes[9]]), 0x1234);
        assert_eq!(u16::from_le_bytes([bytes[10], bytes[11]]), 0x5678);
        assert_eq!(bytes[17], 1, "bNumConfigurations");
    }

    #[test]
    fn the_configuration_arrives_as_one_tree() {
        let (bus, _mouse) = plugged();
        let bytes = control_in(&bus, DeviceAddress::DEFAULT, get_descriptor(2, 0, 255));
        // Configuration (9) + interface (9) + HID (9) + endpoint (7).
        assert_eq!(bytes.len(), 34);
        assert_eq!(u16::from_le_bytes([bytes[2], bytes[3]]), 34, "wTotalLength");
        assert_eq!(bytes[4], 1, "bNumInterfaces");
        assert_eq!(bytes[9 + 5], CLASS_HID, "bInterfaceClass");
        assert_eq!(bytes[9 + 6], SUBCLASS_BOOT, "bInterfaceSubClass");
        assert_eq!(bytes[9 + 7], PROTOCOL_MOUSE, "bInterfaceProtocol");
        assert_eq!(bytes[18 + 1], DESC_HID, "the HID descriptor follows");
        assert_eq!(bytes[27 + 2], ENDPOINT | 0x80, "bEndpointAddress");
        assert_eq!(bytes[27 + 3] & 0x3, 3, "an interrupt endpoint");
        assert_eq!(bytes[27 + 6], INTERVAL, "bInterval");
    }

    #[test]
    fn a_short_wlength_truncates_rather_than_overruns() {
        let (bus, _mouse) = plugged();
        // The first thing a host does is read eight bytes of an eighteen-byte
        // descriptor, because it needs `bMaxPacketSize0` before it can read
        // the rest (USB 2.0 §9.4.3).
        let bytes = control_in(&bus, DeviceAddress::DEFAULT, get_descriptor(1, 0, 8));
        assert_eq!(bytes.len(), 8);
        assert_eq!(bytes[7], 64);
    }

    #[test]
    fn the_report_descriptor_comes_from_the_interface() {
        let (bus, _mouse) = plugged();
        let setup = SetupPacket {
            // Device to host, standard, recipient interface.
            request_type: 0x81,
            request: request::GET_DESCRIPTOR,
            value: (u16::from(DESC_REPORT) << 8),
            index: 0,
            length: 256,
        };
        let bytes = control_in(&bus, DeviceAddress::DEFAULT, setup);
        assert_eq!(bytes, REPORT_DESCRIPTOR);
    }

    #[test]
    fn the_address_changes_only_when_the_status_stage_completes() {
        let (bus, mouse) = plugged();
        let setup = SetupPacket {
            request_type: 0x00,
            request: request::SET_ADDRESS,
            value: 7,
            index: 0,
            length: 0,
        };
        assert_eq!(bus.setup(DeviceAddress::DEFAULT, 0, setup), Status::Ack);
        assert_eq!(
            mouse.address(),
            DeviceAddress::DEFAULT,
            "USB 2.0 §9.4.6: the new address takes effect after the status stage, \
             and the status stage is addressed to the old one"
        );
        assert_eq!(
            bus.read(DeviceAddress::DEFAULT, 0, &mut []).status,
            Status::Ack
        );
        assert_eq!(mouse.address(), DeviceAddress(7));
        // And the old address answers nothing now.
        assert_eq!(
            bus.setup(DeviceAddress::DEFAULT, 0, setup),
            Status::NoDevice
        );
    }

    #[test]
    fn a_configured_mouse_delivers_reports_and_naks_when_idle() {
        let (bus, mouse) = plugged();
        let address = DeviceAddress::DEFAULT;
        let configure = SetupPacket {
            request_type: 0x00,
            request: request::SET_CONFIGURATION,
            value: 1,
            index: 0,
            length: 0,
        };
        assert_eq!(control_out(&bus, address, configure), Status::Ack);
        assert_eq!(mouse.configuration(), 1);

        // Nothing has moved: an interrupt endpoint with no data NAKs.
        let mut report = [0u8; 8];
        assert_eq!(bus.read(address, ENDPOINT, &mut report).status, Status::Nak);

        mouse.motion(5, -3, 0b001);
        let completion = bus.read(address, ENDPOINT, &mut report);
        assert_eq!(completion.status, Status::Ack);
        assert_eq!(completion.len, 3);
        assert_eq!(report[0], 0b001, "button 1");
        assert_eq!(report[1] as i8, 5);
        assert_eq!(report[2] as i8, -3);

        // And the report is gone.
        assert_eq!(bus.read(address, ENDPOINT, &mut report).status, Status::Nak);
    }

    #[test]
    fn a_debug_peek_does_not_consume_the_report() {
        let (bus, mouse) = plugged();
        mouse.motion(1, 1, 0);
        let mut first = [0u8; 8];
        assert_eq!(
            bus.peek(DeviceAddress::DEFAULT, ENDPOINT, &mut first).len,
            3
        );
        assert!(mouse.has_report(), "a debug read must not pop the endpoint");
        let mut second = [0u8; 8];
        assert_eq!(
            bus.read(DeviceAddress::DEFAULT, ENDPOINT, &mut second).len,
            3
        );
        assert_eq!(first[..3], second[..3]);
        assert!(!mouse.has_report());
    }

    #[test]
    fn the_class_requests_are_answered() {
        let (bus, mouse) = plugged();
        let address = DeviceAddress::DEFAULT;

        let set_idle = SetupPacket {
            // Host to device, class, recipient interface.
            request_type: 0x21,
            request: class_request::SET_IDLE,
            value: 0x2a00,
            index: 0,
            length: 0,
        };
        assert_eq!(control_out(&bus, address, set_idle), Status::Ack);
        let idle = control_in(
            &bus,
            address,
            SetupPacket {
                request_type: 0xa1,
                request: class_request::GET_IDLE,
                value: 0,
                index: 0,
                length: 1,
            },
        );
        assert_eq!(idle, alloc::vec![0x2a]);

        // `GET_REPORT` reports the buttons and no movement: movement is
        // relative, and reporting it twice would move the pointer twice.
        mouse.motion(9, 9, 0b010);
        let report = control_in(
            &bus,
            address,
            SetupPacket {
                request_type: 0xa1,
                request: class_request::GET_REPORT,
                value: 0x0100,
                index: 0,
                length: 3,
            },
        );
        assert_eq!(report, alloc::vec![0b010, 0, 0]);
        assert!(
            mouse.has_report(),
            "GET_REPORT is not a poll of the endpoint"
        );
    }

    #[test]
    fn an_unsupported_request_stalls_the_data_stage_and_not_the_setup() {
        let (bus, _mouse) = plugged();
        let setup = SetupPacket {
            request_type: 0x80,
            request: request::GET_DESCRIPTOR,
            // There is no string descriptor table on this device.
            value: 0x0300,
            index: 0,
            length: 8,
        };
        assert_eq!(
            bus.setup(DeviceAddress::DEFAULT, 0, setup),
            Status::Ack,
            "USB 2.0 §9.2.7: a SETUP is always acknowledged"
        );
        let mut buf = [0u8; 8];
        assert_eq!(
            bus.read(DeviceAddress::DEFAULT, 0, &mut buf).status,
            Status::Stall,
            "the request error lands on the data stage"
        );
    }

    #[test]
    fn a_bus_reset_returns_the_device_to_the_default_state() {
        let (bus, mouse) = plugged();
        let address = DeviceAddress::DEFAULT;
        assert_eq!(
            control_out(
                &bus,
                address,
                SetupPacket {
                    request_type: 0,
                    request: request::SET_ADDRESS,
                    value: 9,
                    index: 0,
                    length: 0,
                }
            ),
            Status::Ack
        );
        assert_eq!(mouse.address(), DeviceAddress(9));
        bus.reset_port(0);
        assert_eq!(mouse.address(), DeviceAddress::DEFAULT);
        assert_eq!(mouse.configuration(), 0);
    }

    #[test]
    fn the_mouse_round_trips_through_a_snapshot() {
        let (bus, mouse) = plugged();
        let address = DeviceAddress::DEFAULT;
        assert_eq!(
            control_out(
                &bus,
                address,
                SetupPacket {
                    request_type: 0,
                    request: request::SET_ADDRESS,
                    value: 3,
                    index: 0,
                    length: 0,
                }
            ),
            Status::Ack
        );
        assert_eq!(
            control_out(
                &bus,
                DeviceAddress(3),
                SetupPacket {
                    request_type: 0,
                    request: request::SET_CONFIGURATION,
                    value: 1,
                    index: 0,
                    length: 0,
                }
            ),
            Status::Ack
        );
        mouse.motion(-4, 4, 0b100);

        // A control transfer left deliberately half-finished: the SETUP has
        // been issued and one packet of the descriptor collected. That is
        // state, and it has to survive.
        assert_eq!(
            bus.setup(DeviceAddress(3), 0, get_descriptor(1, 0, 18)),
            Status::Ack
        );
        let mut first = [0u8; 8];
        assert_eq!(bus.read(DeviceAddress(3), 0, &mut first).len, 8);

        let mut saved = Vec::new();
        mouse.save_to(&mut saved).expect("it saves");

        let fresh = HidMouse::new_detached(0x1234, 0x5678);
        fresh.load_from(&saved).expect("it loads");
        let mut again = Vec::new();
        fresh.save_to(&mut again).expect("it saves");
        assert_eq!(saved, again, "the snapshot did not round trip");

        // And the restored device really is mid-transfer: the *rest* of the
        // descriptor comes out, not the beginning of it again.
        let bus2 = Arc::new(UsbBus::new(1));
        bus2.attach(0, fresh.device()).expect("an empty port");
        bus2.set_enabled(0, true);
        let mut rest = [0u8; 64];
        let completion = bus2.read(DeviceAddress(3), 0, &mut rest);
        assert_eq!(
            completion.len, 10,
            "eighteen bytes less the eight already read"
        );
        assert_eq!(rest[0], 0x34, "idVendor's low byte, at offset 8");
    }

    /// A report queued but not collected is state too.
    #[test]
    fn a_queued_report_survives_a_snapshot() {
        let (_bus, mouse) = plugged();
        mouse.motion(7, -7, 0b011);
        let mut saved = Vec::new();
        mouse.save_to(&mut saved).expect("it saves");

        let fresh = HidMouse::new_detached(0, 0);
        fresh.load_from(&saved).expect("it loads");
        assert!(fresh.has_report());

        let bus = Arc::new(UsbBus::new(1));
        bus.attach(0, fresh.device()).expect("an empty port");
        bus.set_enabled(0, true);
        let mut report = [0u8; 8];
        assert_eq!(
            bus.read(DeviceAddress::DEFAULT, ENDPOINT, &mut report).len,
            3
        );
        assert_eq!(report[1] as i8, 7);
        assert_eq!(report[2] as i8, -7);
    }
}
