//! Descriptors: what a device says about itself (USB 2.0 §9.5, §9.6).
//!
//! Enumeration is a host reading a tree of these out of a device over the
//! default control pipe, so a device model that wants to enumerate has to
//! produce the exact bytes §9.6's tables describe. Getting a length or an
//! offset wrong there produces a device that *almost* enumerates, which is a
//! miserable thing to debug — so the byte layout lives here, once, in encoders
//! whose field names are the specification's own.
//!
//! # Why builders and not a byte array per device
//!
//! A device model that wrote its own descriptor bytes would be writing a
//! length prefix and a `wTotalLength` by hand, and the second one is computed
//! from everything that follows it. [`Descriptors::configuration`] computes it,
//! which is the only reason this module is more than a set of constants.
//!
//! # Sources
//!
//! USB 2.0 §9.5 and §9.6 (tables 9-8 through 9-16), free from usb.org.

use alloc::vec::Vec;

/// The standard descriptor types (USB 2.0 §9.4, table 9-5).
pub mod kind {
    /// §9.6.1. One per device.
    pub const DEVICE: u8 = 1;
    /// §9.6.3. The whole configuration tree, returned in one go.
    pub const CONFIGURATION: u8 = 2;
    /// §9.6.7. UTF-16LE text, indexed, with index zero listing the languages.
    pub const STRING: u8 = 3;
    /// §9.6.5. Never returned on its own — it arrives inside a configuration.
    pub const INTERFACE: u8 = 4;
    /// §9.6.6. Likewise.
    pub const ENDPOINT: u8 = 5;
    /// §9.6.2. What a high-speed device would look like at full speed.
    pub const DEVICE_QUALIFIER: u8 = 6;
    /// §9.6.4.
    pub const OTHER_SPEED_CONFIGURATION: u8 = 7;
    /// §9.6. Reserved and never used.
    pub const INTERFACE_POWER: u8 = 8;
}

/// A descriptor type, as `wValue`'s high byte carries it.
///
/// The extensible-newtype pattern (`CLAUDE.md`, *Type conventions*), because
/// the space is genuinely open: every device class specification adds its own
/// — HID's report descriptor is `0x22` — and a `match` in this crate must not
/// have to grow a variant for each.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DescriptorKind(pub u8);

impl DescriptorKind {
    /// §9.6.1.
    pub const DEVICE: DescriptorKind = DescriptorKind(kind::DEVICE);
    /// §9.6.3.
    pub const CONFIGURATION: DescriptorKind = DescriptorKind(kind::CONFIGURATION);
    /// §9.6.7.
    pub const STRING: DescriptorKind = DescriptorKind(kind::STRING);
    /// §9.6.2.
    pub const DEVICE_QUALIFIER: DescriptorKind = DescriptorKind(kind::DEVICE_QUALIFIER);

    /// Whether this type is one the device framework itself defines, as
    /// opposed to a class specification's.
    ///
    /// The boundary matters at exactly one place: a `GET_DESCRIPTOR` for a
    /// class-specific type is the class's to answer, and [`super::Endpoint0`]
    /// forwards it rather than looking in its own table.
    #[must_use]
    pub const fn is_standard(self) -> bool {
        self.0 >= 1 && self.0 <= kind::INTERFACE_POWER
    }
}

/// The device descriptor (USB 2.0 §9.6.1, table 9-8).
///
/// Eighteen bytes, and the first thing a host ever reads — often only the first
/// eight of them, because `bMaxPacketSize0` is at offset 7 and the host needs
/// it before it can read the rest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DeviceDescriptor {
    /// `bcdUSB`: the specification release this device conforms to, BCD.
    /// `0x0200` for USB 2.0.
    pub usb: u16,
    /// `bDeviceClass`. Zero means "look at the interfaces instead".
    pub class: u8,
    /// `bDeviceSubClass`.
    pub subclass: u8,
    /// `bDeviceProtocol`.
    pub protocol: u8,
    /// `bMaxPacketSize0`: the control endpoint's packet size. 64 at high
    /// speed, and it is not optional there (§5.5.3).
    pub max_packet0: u8,
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// `bcdDevice`: the device's own release number, BCD.
    pub device: u16,
    /// `iManufacturer`: string index, or zero for none.
    pub manufacturer: u8,
    /// `iProduct`.
    pub product_name: u8,
    /// `iSerialNumber`.
    pub serial: u8,
    /// `bNumConfigurations`.
    pub configurations: u8,
}

impl DeviceDescriptor {
    /// `bLength`. Always eighteen.
    pub const SIZE: u8 = 18;

    /// The eighteen bytes, little-endian.
    #[must_use]
    pub fn encode(&self) -> [u8; 18] {
        let usb = self.usb.to_le_bytes();
        let vendor = self.vendor.to_le_bytes();
        let product = self.product.to_le_bytes();
        let device = self.device.to_le_bytes();
        [
            DeviceDescriptor::SIZE,
            kind::DEVICE,
            usb[0],
            usb[1],
            self.class,
            self.subclass,
            self.protocol,
            self.max_packet0,
            vendor[0],
            vendor[1],
            product[0],
            product[1],
            device[0],
            device[1],
            self.manufacturer,
            self.product_name,
            self.serial,
            self.configurations,
        ]
    }
}

impl Default for DeviceDescriptor {
    fn default() -> DeviceDescriptor {
        DeviceDescriptor {
            usb: 0x0200,
            class: 0,
            subclass: 0,
            protocol: 0,
            max_packet0: 64,
            vendor: 0,
            product: 0,
            device: 0x0100,
            manufacturer: 0,
            product_name: 0,
            serial: 0,
            configurations: 1,
        }
    }
}

/// The configuration descriptor (USB 2.0 §9.6.3, table 9-10).
///
/// Nine bytes, and `wTotalLength` covers everything that follows it in the same
/// `GET_DESCRIPTOR` response — which is why it is not a field here.
/// [`Descriptors::configuration`] fills it in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConfigurationDescriptor {
    /// `bNumInterfaces`.
    pub interfaces: u8,
    /// `bConfigurationValue`: what `SET_CONFIGURATION` is given to select it.
    /// Must not be zero — zero means "unconfigured" (§9.4.7).
    pub value: u8,
    /// `iConfiguration`: string index, or zero.
    pub name: u8,
    /// `bmAttributes`. Bit 7 is reserved and reads one; bit 6 is self-powered,
    /// bit 5 remote wakeup.
    pub attributes: u8,
    /// `bMaxPower`, in 2 mA units.
    pub max_power: u8,
}

impl ConfigurationDescriptor {
    /// `bLength`. Always nine.
    pub const SIZE: u8 = 9;

    /// Bit 7 of `bmAttributes`, reserved and set (§9.6.3).
    pub const RESERVED: u8 = 0x80;
    /// Bit 6: the device has its own power supply.
    pub const SELF_POWERED: u8 = 0x40;
    /// Bit 5: the device can signal resume.
    pub const REMOTE_WAKEUP: u8 = 0x20;

    /// The nine bytes, with `total` as `wTotalLength`.
    #[must_use]
    pub fn encode(&self, total: u16) -> [u8; 9] {
        let total = total.to_le_bytes();
        [
            ConfigurationDescriptor::SIZE,
            kind::CONFIGURATION,
            total[0],
            total[1],
            self.interfaces,
            self.value,
            self.name,
            self.attributes | ConfigurationDescriptor::RESERVED,
            self.max_power,
        ]
    }
}

impl Default for ConfigurationDescriptor {
    fn default() -> ConfigurationDescriptor {
        ConfigurationDescriptor {
            interfaces: 1,
            value: 1,
            name: 0,
            attributes: ConfigurationDescriptor::RESERVED,
            max_power: 50,
        }
    }
}

/// The interface descriptor (USB 2.0 §9.6.5, table 9-12).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct InterfaceDescriptor {
    /// `bInterfaceNumber`.
    pub number: u8,
    /// `bAlternateSetting`.
    pub alternate: u8,
    /// `bNumEndpoints`, **not** counting endpoint zero.
    pub endpoints: u8,
    /// `bInterfaceClass`. 3 is HID.
    pub class: u8,
    /// `bInterfaceSubClass`. 1 is "boot interface" for HID.
    pub subclass: u8,
    /// `bInterfaceProtocol`. For HID boot: 1 keyboard, 2 mouse.
    pub protocol: u8,
    /// `iInterface`: string index, or zero.
    pub name: u8,
}

impl InterfaceDescriptor {
    /// `bLength`. Always nine.
    pub const SIZE: u8 = 9;

    /// The nine bytes.
    #[must_use]
    pub fn encode(&self) -> [u8; 9] {
        [
            InterfaceDescriptor::SIZE,
            kind::INTERFACE,
            self.number,
            self.alternate,
            self.endpoints,
            self.class,
            self.subclass,
            self.protocol,
            self.name,
        ]
    }
}

/// The endpoint descriptor (USB 2.0 §9.6.6, table 9-13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EndpointDescriptor {
    /// `bEndpointAddress`: number in bits 3:0, direction in bit 7.
    pub address: u8,
    /// `bmAttributes`: the transfer type in bits 1:0.
    pub attributes: u8,
    /// `wMaxPacketSize`.
    pub max_packet: u16,
    /// `bInterval`, in frames at full speed and in `2^(bInterval-1)`
    /// microframes at high speed (§9.6.6) — so `4` is eight microframes, which
    /// is one millisecond.
    pub interval: u8,
}

impl EndpointDescriptor {
    /// `bLength`. Always seven for a non-audio endpoint.
    pub const SIZE: u8 = 7;

    /// The seven bytes.
    #[must_use]
    pub fn encode(&self) -> [u8; 7] {
        let max = self.max_packet.to_le_bytes();
        [
            EndpointDescriptor::SIZE,
            kind::ENDPOINT,
            self.address,
            self.attributes,
            max[0],
            max[1],
            self.interval,
        ]
    }
}

/// A string descriptor holding `text`, UTF-16LE (USB 2.0 §9.6.7, table 9-16).
///
/// Text longer than 126 characters is truncated: `bLength` is one byte, and a
/// descriptor that overflowed it would be a corrupt one rather than a long one.
#[must_use]
pub fn string_descriptor(text: &str) -> Vec<u8> {
    let units: Vec<u16> = text.encode_utf16().take(126).collect();
    let mut out = Vec::with_capacity(2 + units.len() * 2);
    out.push((2 + units.len() * 2) as u8);
    out.push(kind::STRING);
    for unit in units {
        out.extend_from_slice(&unit.to_le_bytes());
    }
    out
}

/// String descriptor zero: the languages this device's strings are in
/// (USB 2.0 §9.6.7, table 9-15).
///
/// `0x0409` is US English, which is what every device in this tree uses.
#[must_use]
pub fn language_descriptor(languages: &[u16]) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + languages.len() * 2);
    out.push((2 + languages.len() * 2) as u8);
    out.push(kind::STRING);
    for language in languages {
        out.extend_from_slice(&language.to_le_bytes());
    }
    out
}

/// Everything a device answers `GET_DESCRIPTOR` with.
///
/// Built once, in a device model's constructor, and read from there on — so a
/// [`super::Function`] hands out a `&Descriptors` and never has to encode
/// anything at request time.
#[derive(Debug, Clone, Default)]
pub struct Descriptors {
    device: Vec<u8>,
    configurations: Vec<Vec<u8>>,
    strings: Vec<Vec<u8>>,
    qualifier: Option<Vec<u8>>,
}

impl Descriptors {
    /// An empty table. Every device fills in at least
    /// [`device`](Descriptors::set_device) and one
    /// [`configuration`](Descriptors::configuration).
    #[must_use]
    pub fn new() -> Descriptors {
        Descriptors::default()
    }

    /// Set the device descriptor.
    pub fn set_device(&mut self, descriptor: &DeviceDescriptor) {
        self.device = descriptor.encode().to_vec();
    }

    /// Set the device descriptor, by value, for a builder chain.
    #[must_use]
    pub fn with_device(mut self, descriptor: &DeviceDescriptor) -> Descriptors {
        self.set_device(descriptor);
        self
    }

    /// Append a configuration: the configuration descriptor followed by the
    /// bytes of everything inside it, with `wTotalLength` computed.
    ///
    /// `body` is the interface, class and endpoint descriptors already encoded,
    /// in the order the host must see them (§9.4.3: "all related interface and
    /// endpoint descriptors" in one response).
    pub fn add_configuration(&mut self, descriptor: &ConfigurationDescriptor, body: &[u8]) {
        let total = (usize::from(ConfigurationDescriptor::SIZE) + body.len()).min(0xffff) as u16;
        let mut bytes = descriptor.encode(total).to_vec();
        bytes.extend_from_slice(body);
        self.configurations.push(bytes);
    }

    /// [`add_configuration`](Descriptors::add_configuration), for a builder
    /// chain.
    #[must_use]
    pub fn configuration(
        mut self,
        descriptor: &ConfigurationDescriptor,
        body: &[u8],
    ) -> Descriptors {
        self.add_configuration(descriptor, body);
        self
    }

    /// Append a string descriptor, returning the index a descriptor should
    /// refer to it by.
    ///
    /// Index zero is the language list and is created on the first call, so
    /// the first string a device adds is index 1 — which is what every
    /// descriptor in the wild assumes.
    pub fn add_string(&mut self, text: &str) -> u8 {
        if self.strings.is_empty() {
            self.strings.push(language_descriptor(&[0x0409]));
        }
        self.strings.push(string_descriptor(text));
        (self.strings.len() - 1).min(255) as u8
    }

    /// Set the device qualifier (§9.6.2), which a high-speed device is
    /// required to have.
    pub fn set_qualifier(&mut self, device: &DeviceDescriptor, other_configurations: u8) {
        let usb = device.usb.to_le_bytes();
        self.qualifier = Some(alloc::vec![
            10,
            kind::DEVICE_QUALIFIER,
            usb[0],
            usb[1],
            device.class,
            device.subclass,
            device.protocol,
            device.max_packet0,
            other_configurations,
            0,
        ]);
    }

    /// The device descriptor's bytes.
    #[must_use]
    pub fn device(&self) -> &[u8] {
        &self.device
    }

    /// How many configurations this device has.
    #[must_use]
    pub fn configuration_count(&self) -> usize {
        self.configurations.len()
    }

    /// The bytes a `GET_DESCRIPTOR` for `(kind, index)` should return, or
    /// `None` for one this device does not have — which the caller answers
    /// with a stall (§9.2.7).
    ///
    /// **No side effects.** Reading a descriptor changes nothing, on real
    /// hardware or here, which is what lets a debugger show a device's identity
    /// without disturbing it.
    #[must_use]
    pub fn get(&self, kind: DescriptorKind, index: u8) -> Option<&[u8]> {
        match kind {
            DescriptorKind::DEVICE if index == 0 => {
                (!self.device.is_empty()).then_some(self.device.as_slice())
            }
            DescriptorKind::CONFIGURATION => self
                .configurations
                .get(usize::from(index))
                .map(Vec::as_slice),
            DescriptorKind::STRING => self.strings.get(usize::from(index)).map(Vec::as_slice),
            DescriptorKind::DEVICE_QUALIFIER if index == 0 => self.qualifier.as_deref(),
            _ => None,
        }
    }

    /// `bConfigurationValue` of configuration `index`, for
    /// `SET_CONFIGURATION`.
    #[must_use]
    pub fn configuration_value(&self, index: usize) -> Option<u8> {
        // Offset 5 of a configuration descriptor (§9.6.3, table 9-10).
        self.configurations
            .get(index)
            .and_then(|c| c.get(5))
            .copied()
    }

    /// Whether `value` names a configuration this device has.
    #[must_use]
    pub fn has_configuration_value(&self, value: u8) -> bool {
        (0..self.configurations.len()).any(|i| self.configuration_value(i) == Some(value))
    }

    /// `bmAttributes` of configuration `value`, for `GET_STATUS` — whose
    /// self-powered bit is a restatement of the configuration's (§9.4.5).
    #[must_use]
    pub fn attributes_of(&self, value: u8) -> Option<u8> {
        (0..self.configurations.len())
            .find(|i| self.configuration_value(*i) == Some(value))
            // Offset 7 of a configuration descriptor.
            .and_then(|i| self.configurations[i].get(7))
            .copied()
    }
}
