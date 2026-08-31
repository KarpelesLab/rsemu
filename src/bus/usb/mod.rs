//! USB: the fabric, the device seam, and the pieces every device model shares.
//!
//! # Controller-agnostic on purpose
//!
//! `docs/buses/usb.md` asks for exactly one thing of this module: *"model the
//! device side generically — endpoints, transfer types, descriptors — so a
//! device model works behind any controller. The controller then becomes a
//! translator between its ring/queue format and generic transfers."* Nothing
//! here knows what a queue head is, what a TRB is, or that EHCI exists. A UHCI
//! or an xHCI added later plugs into this unchanged, because the only thing a
//! controller ever asks the fabric for is *a transaction*.
//!
//! ```text
//!   guest RAM              controller                 fabric              device
//!   ─────────              ──────────                 ──────              ──────
//!   QH / qTD    ──DMA──►   decode a token   ──►   route by address  ──►  Peripheral
//!   (EHCI)                 SETUP / IN / OUT       (UsbBus::setup,        └─ Endpoint0
//!   TRB ring    ──DMA──►   decode a TRB           read, write)              (standard
//!   (xHCI, later)                                                            requests)
//!                                                                        └─ Function
//!                                                                            (the class)
//! ```
//!
//! # The seam is a *transaction*, not a transfer
//!
//! USB 2.0 §8.4 makes a transaction the indivisible unit on the wire: a token
//! packet, an optional data packet, and a handshake. A transfer is a *sequence*
//! of those, and which sequence depends on the transfer type — which is the
//! controller's business, since it is the controller that holds the schedule.
//! So [`UsbDevice`] speaks transactions ([`UsbDevice::setup`],
//! [`UsbDevice::transfer_in`], [`UsbDevice::transfer_out`]) and returns a
//! handshake ([`Status`]), and every one of the four transfer types is
//! expressible as a sequence of them:
//!
//! | Transfer type | The sequence a controller issues |
//! | --- | --- |
//! | Control | `SETUP` (8 bytes), then `IN`/`OUT` data packets, then a zero-length handshake the other way (§8.5.3) |
//! | Bulk | `IN` or `OUT` packets until a short one or the byte count runs out (§8.5.2) |
//! | Interrupt | one `IN`/`OUT` per service interval, `NAK` when there is nothing (§8.5.4) |
//! | Isochronous | one `IN`/`OUT` per (micro)frame, no handshake — see the note below |
//!
//! **Isochronous is expressible but not exercised.** A device may return data
//! to an `IN` in an isochronous pipe and the fabric will carry it; what does
//! not exist yet is a controller that walks an isochronous schedule (rsemu's
//! EHCI skips `iTD`/`siTD` nodes, and says so). The seam does not need to
//! change for that to land — the transfer type never appears in a signature
//! here, exactly as it does not appear on the wire.
//!
//! # A device model is written once
//!
//! The same argument [`crate::bus::spi`] makes for `Shifter`: the standard
//! device requests of USB 2.0 §9.4 are identical for every device that has ever
//! existed, so writing them per device would be writing them wrong per device.
//! [`Endpoint0`] implements them from a [`Descriptors`] table, [`Peripheral`]
//! wraps that around a [`Function`] — the class-specific half — and the result
//! is a [`UsbDevice`]. [`crate::dev::usb::hid`] is a `Function` and nothing
//! more: a report descriptor, one class request, and an interrupt endpoint.
//!
//! # Speeds, and the thing that bites
//!
//! [`Speed`] is carried per device and reported per port, because it decides
//! routing on a real bus and it decides *whether the controller keeps the port
//! at all*: EHCI is high-speed only, and a full- or low-speed device reaches it
//! only through a transaction translator in a hub or a companion controller
//! (EHCI 1.0 §4.2). rsemu's EHCI models the honest half of that — it hands the
//! port to a companion by setting `PORTSC.Port Owner`, which is what the
//! silicon does — so a full-speed device attached to a bare EHCI *disappears*
//! rather than silently enumerating. See [`crate::dev::usb::ehci`].
//!
//! # What is not here
//!
//! * **Hubs.** The fabric has the port model a hub needs (connect, enable,
//!   speed, reset) and a hub device model would be an ordinary [`Function`]
//!   with downstream ports. What it would *also* need is EHCI split
//!   transactions (`SPLIT` tokens, the `µFrame C-mask` in a queue head, and the
//!   `siTD`), and none of that exists. So there is no hub, and a tiered bus is
//!   not modelled.
//! * **Host passthrough.** Sharing a real device from the host is a
//!   [`UsbDevice`] implementor living under `host/` and nothing more, which is
//!   why the seam is shaped this way. It is not started here: on Linux the
//!   sanctioned route is `usbfs` through raw syscalls (`libusb` is a C library
//!   and the dependency policy forbids it outright), and it is non-portable and
//!   non-deterministic enough to want the record/replay seam first.
//!
//! # Sources
//!
//! The **USB 2.0 specification** (usb.org, free download, no membership) —
//! §8.3 packet fields, §8.4 transaction formats, §8.5 transfer types, §9
//! device framework — and the **HID 1.11** and **HID Usage Tables**
//! specifications for the device under [`crate::dev::usb::hid`]. No emulator
//! source was consulted (`ROADMAP.md` §1); `docs/buses/usb.md` notes that USB
//! is the bus with the least excuse for doing otherwise, since the
//! specifications are genuinely free.

pub mod descriptor;
pub mod function;

#[cfg(test)]
mod tests;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::sync::{AtomicU32, LockRank, Mutex, Ordering};

pub use descriptor::{
    ConfigurationDescriptor, DescriptorKind, Descriptors, DeviceDescriptor, EndpointDescriptor,
    InterfaceDescriptor, language_descriptor, string_descriptor,
};
pub use function::{Endpoint0, Function, Peripheral};

// ---------------------------------------------------------------------------
// Where this module sits in the lock ladder
// ---------------------------------------------------------------------------

/// The rank a host controller's register file takes.
///
/// **Not [`LockRank::BUS`]**, for the reason `docs/buses/low-speed.md` records
/// and [`crate::bus::spi::FABRIC_RANK`] spells out: a CPU core holds its
/// execution state across a guest access — the RISC-V hart's session mutex
/// *is* `LockRank::BUS` — so `BUS` is already held by the time an MMIO write
/// reaches a controller. A fabric that also took `BUS` would be a lock-order
/// violation on the first register write.
///
/// The order a transaction actually travels:
///
/// ```text
///   CPU session            (BUS 0x4000)
///     → HCD registers      (0x4a00, here)
///       → USB fabric       (0x4b00, FABRIC_RANK)
///         → default pipe   (0x4c00, EP0_RANK)
///           → the device's own state (DEVICE 0x5000)
///             → its output wires     (WIRE 0x6000)
/// ```
///
/// A controller **must not hold this across the DMA walk or a call into the
/// fabric** — that is the re-entrancy contract in [`crate::core::device`], not
/// a rank question — and the ladder is what catches it if it does.
pub const HCD_RANK: LockRank = LockRank::new(0x4a00);

/// The rank a [`UsbBus`]'s port table takes. See [`HCD_RANK`] for the ladder.
pub const FABRIC_RANK: LockRank = LockRank::new(0x4b00);

/// The rank an [`Endpoint0`] control-pipe state machine takes.
///
/// Below the fabric and above a device's own state, because the pipe's lock is
/// deliberately held across the call into the [`Function`] — serving one
/// request is a single step. See [`HCD_RANK`] for the whole ladder, and
/// [`Endpoint0`] for why the lock is held rather than dropped.
pub const EP0_RANK: LockRank = LockRank::new(0x4c00);

// ---------------------------------------------------------------------------
// Addresses, speeds, endpoints
// ---------------------------------------------------------------------------

/// A device's bus address, as `SET_ADDRESS` assigns it (USB 2.0 §9.4.6).
///
/// A newtype rather than a bare `u8` because "the address in the token" and
/// "the endpoint in the token" are both small integers that a controller pulls
/// out of adjacent bit fields, and swapping them is the classic host-controller
/// bug — it reads identically to correct code.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct DeviceAddress(pub u8);

impl DeviceAddress {
    /// The address every device answers to before it has been assigned one
    /// (USB 2.0 §9.1.1.3: the *Default* state).
    pub const DEFAULT: DeviceAddress = DeviceAddress(0);

    /// The largest address `SET_ADDRESS` may assign — the field is seven bits.
    pub const MAX: u8 = 127;

    /// Whether this is an address a device may actually be given.
    #[must_use]
    pub const fn is_valid(self) -> bool {
        self.0 <= DeviceAddress::MAX
    }
}

impl fmt::Display for DeviceAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "usb{}", self.0)
    }
}

/// How fast a device signals (USB 2.0 §4.2.1, §7.1.1).
///
/// A real `enum` rather than the extensible-newtype pattern: USB 2.0 defines
/// exactly these three, and exhaustiveness is what makes a controller's
/// "can I even talk to this?" decision checkable. USB 3.x SuperSpeed is not a
/// fourth value here — it is a different bus with a different signalling
/// scheme, and an xHCI that grows it will say so then.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Speed {
    /// 1.5 Mb/s. Keyboards and mice from before 2000, and nothing else.
    Low,
    /// 12 Mb/s. The original USB rate.
    Full,
    /// 480 Mb/s, and the only rate EHCI itself can drive.
    #[default]
    High,
}

impl Speed {
    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Speed::Low => "low",
            Speed::Full => "full",
            Speed::High => "high",
        }
    }

    /// Parse the spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Speed> {
        match name {
            "low" => Some(Speed::Low),
            "full" => Some(Speed::Full),
            "high" => Some(Speed::High),
            _ => None,
        }
    }

    /// Every spelling, for a validator's enumeration.
    pub const NAMES: &'static [&'static str] = &["low", "full", "high"];

    /// The largest `wMaxPacketSize` this speed allows on a control endpoint
    /// (USB 2.0 §5.5.3).
    ///
    /// High speed permits — and *requires* — 64; full speed allows 8, 16, 32
    /// or 64; low speed allows only 8.
    #[must_use]
    pub const fn max_control_packet(self) -> u16 {
        match self {
            Speed::Low => 8,
            Speed::Full | Speed::High => 64,
        }
    }
}

impl fmt::Display for Speed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which way the bytes of a transfer move, *from the host's point of view*.
///
/// The host's point of view is the specification's (USB 2.0 §9.3.1: "data
/// transfer direction"), and it is worth stating because a device model
/// naturally thinks the other way round: an `In` endpoint is one the *device*
/// writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Direction {
    /// Host to device.
    #[default]
    Out,
    /// Device to host.
    In,
}

impl Direction {
    /// The direction bit of an endpoint address or a `bmRequestType`
    /// (USB 2.0 §9.3.1, §9.6.6): set means `In`.
    pub const BIT: u8 = 0x80;

    /// The direction that bit 7 of `value` encodes.
    #[must_use]
    pub const fn from_bit(value: u8) -> Direction {
        if value & Direction::BIT != 0 {
            Direction::In
        } else {
            Direction::Out
        }
    }

    /// The spelling a machine description or a diagnostic writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Direction::Out => "out",
            Direction::In => "in",
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The four transfer types an endpoint may be (USB 2.0 §5.4, §9.6.6).
///
/// The fabric itself never branches on this — a transaction is a transaction —
/// but a device model declares it in an endpoint descriptor and a controller
/// reads it out of one, so the constants belong here rather than in either.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum TransferType {
    /// Bidirectional, request/response, and the only type endpoint zero may
    /// be. Guaranteed bus bandwidth for enumeration (§5.5).
    #[default]
    Control,
    /// Guaranteed bandwidth, guaranteed latency, **no retry and no handshake**
    /// (§5.6). Audio and video.
    Isochronous,
    /// Whatever bandwidth is left over, with retries (§5.8). Storage.
    Bulk,
    /// A bounded service interval, retried (§5.7). Human input devices.
    Interrupt,
}

impl TransferType {
    /// The two-bit `bmAttributes` encoding of an endpoint descriptor (§9.6.6).
    #[must_use]
    pub const fn attribute_bits(self) -> u8 {
        match self {
            TransferType::Control => 0,
            TransferType::Isochronous => 1,
            TransferType::Bulk => 2,
            TransferType::Interrupt => 3,
        }
    }

    /// The type those two bits name.
    #[must_use]
    pub const fn from_attribute_bits(bits: u8) -> TransferType {
        match bits & 0x3 {
            0 => TransferType::Control,
            1 => TransferType::Isochronous,
            2 => TransferType::Bulk,
            _ => TransferType::Interrupt,
        }
    }

    /// The spelling a diagnostic writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            TransferType::Control => "control",
            TransferType::Isochronous => "isochronous",
            TransferType::Bulk => "bulk",
            TransferType::Interrupt => "interrupt",
        }
    }
}

impl fmt::Display for TransferType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The handshake a transaction ends in (USB 2.0 §8.4.5), plus the two failures
/// that are the *absence* of one.
///
/// A controller maps these onto its own completion codes: EHCI turns
/// [`Status::Stall`] into the qTD's `Halted` bit and [`Status::Nak`] into
/// "leave the transfer active and come back next microframe".
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Status {
    /// The data was accepted (or, for an `IN`, produced).
    #[default]
    Ack,
    /// The endpoint has nothing to give, or cannot accept right now. **Not an
    /// error**: the host retries, and an idle interrupt endpoint spends its
    /// life here.
    Nak,
    /// The endpoint is halted, or the request is not one this device answers
    /// (§9.2.7: "request error"). The host must clear the condition.
    Stall,
    /// Nothing at this address answered. There is no such handshake on the
    /// wire — silence is the signal — and a controller turns it into a
    /// transaction error after its retry count runs out.
    NoDevice,
    /// The device sent more than `wMaxPacketSize`. A protocol violation the
    /// host must report (§8.7.4).
    Babble,
    /// A CRC or bit-stuffing failure. Modelled for completeness — nothing in
    /// this tree corrupts a packet yet, and a fault-injection device would be
    /// where that comes from.
    Error,
}

impl Status {
    /// Whether this outcome retires a transfer rather than asking for a retry.
    #[must_use]
    pub const fn is_final(self) -> bool {
        !matches!(self, Status::Nak)
    }

    /// Whether the host should report this as an error to its driver.
    #[must_use]
    pub const fn is_error(self) -> bool {
        matches!(
            self,
            Status::Stall | Status::NoDevice | Status::Babble | Status::Error
        )
    }

    /// The spelling a diagnostic writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Status::Ack => "ack",
            Status::Nak => "nak",
            Status::Stall => "stall",
            Status::NoDevice => "no device",
            Status::Babble => "babble",
            Status::Error => "error",
        }
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What one transaction did: a handshake, and how many bytes moved with it.
///
/// The byte count matters even when it is smaller than the buffer: a *short
/// packet* is how a USB device says "that is all of it", and every transfer
/// type but isochronous depends on the host noticing (§5.8.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Completion {
    /// The handshake.
    pub status: Status,
    /// Bytes actually transferred. Zero for anything but [`Status::Ack`].
    pub len: u64,
}

impl Completion {
    /// `len` bytes moved, acknowledged.
    #[must_use]
    pub const fn ack(len: u64) -> Completion {
        Completion {
            status: Status::Ack,
            len,
        }
    }

    /// Nothing to give right now.
    #[must_use]
    pub const fn nak() -> Completion {
        Completion {
            status: Status::Nak,
            len: 0,
        }
    }

    /// The endpoint refuses.
    #[must_use]
    pub const fn stall() -> Completion {
        Completion {
            status: Status::Stall,
            len: 0,
        }
    }

    /// Nothing answered.
    #[must_use]
    pub const fn absent() -> Completion {
        Completion {
            status: Status::NoDevice,
            len: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// The setup packet
// ---------------------------------------------------------------------------

/// Who a control request is addressed to (USB 2.0 §9.3.1, bits 4:0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Recipient {
    /// The device as a whole.
    #[default]
    Device,
    /// One interface, named in `wIndex`.
    Interface,
    /// One endpoint, named in `wIndex`.
    Endpoint,
    /// Something else the specification does not define.
    Other,
}

/// Which rule book a control request comes from (USB 2.0 §9.3.1, bits 6:5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum RequestKind {
    /// USB 2.0 §9.4. Every device answers these, and [`Endpoint0`] does it for
    /// them.
    #[default]
    Standard,
    /// A device-class specification's. HID's `GET_REPORT` is one.
    Class,
    /// The vendor's own.
    Vendor,
    /// Reserved by the specification.
    Reserved,
}

/// The eight bytes of a `SETUP` packet (USB 2.0 §9.3, table 9-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SetupPacket {
    /// `bmRequestType`: direction, kind and recipient in one byte.
    pub request_type: u8,
    /// `bRequest`: which request, interpreted per [`RequestKind`].
    pub request: u8,
    /// `wValue`: request-specific.
    pub value: u16,
    /// `wIndex`: request-specific; an interface or endpoint number for a
    /// recipient that is one of those.
    pub index: u16,
    /// `wLength`: how many bytes the data stage carries, at most.
    pub length: u16,
}

impl SetupPacket {
    /// A setup packet is always exactly eight bytes.
    pub const SIZE: u64 = 8;

    /// Decode the eight bytes off the wire. Little-endian, per §9.3.
    #[must_use]
    pub const fn decode(bytes: &[u8; 8]) -> SetupPacket {
        SetupPacket {
            request_type: bytes[0],
            request: bytes[1],
            value: u16::from_le_bytes([bytes[2], bytes[3]]),
            index: u16::from_le_bytes([bytes[4], bytes[5]]),
            length: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    /// Encode back to the eight bytes on the wire.
    #[must_use]
    pub const fn encode(&self) -> [u8; 8] {
        let value = self.value.to_le_bytes();
        let index = self.index.to_le_bytes();
        let length = self.length.to_le_bytes();
        [
            self.request_type,
            self.request,
            value[0],
            value[1],
            index[0],
            index[1],
            length[0],
            length[1],
        ]
    }

    /// Which way the data stage moves.
    #[must_use]
    pub const fn direction(&self) -> Direction {
        Direction::from_bit(self.request_type)
    }

    /// Which rule book this request comes from.
    #[must_use]
    pub const fn kind(&self) -> RequestKind {
        match (self.request_type >> 5) & 0x3 {
            0 => RequestKind::Standard,
            1 => RequestKind::Class,
            2 => RequestKind::Vendor,
            _ => RequestKind::Reserved,
        }
    }

    /// Who it is addressed to.
    #[must_use]
    pub const fn recipient(&self) -> Recipient {
        match self.request_type & 0x1f {
            0 => Recipient::Device,
            1 => Recipient::Interface,
            2 => Recipient::Endpoint,
            _ => Recipient::Other,
        }
    }

    /// The descriptor type and index `wValue` carries for `GET_DESCRIPTOR`
    /// (§9.4.3): type in the high byte, index in the low one.
    #[must_use]
    pub const fn descriptor(&self) -> (u8, u8) {
        ((self.value >> 8) as u8, self.value as u8)
    }
}

/// The standard device requests of USB 2.0 §9.4, table 9-4.
pub mod request {
    /// §9.4.5.
    pub const GET_STATUS: u8 = 0;
    /// §9.4.1.
    pub const CLEAR_FEATURE: u8 = 1;
    /// §9.4.9.
    pub const SET_FEATURE: u8 = 3;
    /// §9.4.6.
    pub const SET_ADDRESS: u8 = 5;
    /// §9.4.3.
    pub const GET_DESCRIPTOR: u8 = 6;
    /// §9.4.8. Optional, and this tree's devices do not implement it.
    pub const SET_DESCRIPTOR: u8 = 7;
    /// §9.4.2.
    pub const GET_CONFIGURATION: u8 = 8;
    /// §9.4.7.
    pub const SET_CONFIGURATION: u8 = 9;
    /// §9.4.4.
    pub const GET_INTERFACE: u8 = 10;
    /// §9.4.10.
    pub const SET_INTERFACE: u8 = 11;
    /// §9.4.11. Isochronous only.
    pub const SYNCH_FRAME: u8 = 12;
}

/// The standard feature selectors of USB 2.0 §9.4, table 9-6.
pub mod feature {
    /// An endpoint's halt condition (§9.4.5). The only feature `CLEAR_FEATURE`
    /// is *required* to accept.
    pub const ENDPOINT_HALT: u16 = 0;
    /// Whether the device may signal resume.
    pub const DEVICE_REMOTE_WAKEUP: u16 = 1;
    /// Electrical test modes. Accepted and ignored — a modelled bus has no eye
    /// diagram.
    pub const TEST_MODE: u16 = 2;
}

// ---------------------------------------------------------------------------
// The device seam
// ---------------------------------------------------------------------------

/// A device on a USB bus, as a host controller sees it.
///
/// Transaction-level, for the reason the module docs give. Implemented by
/// [`Peripheral`] for anything built the ordinary way — a [`Function`] plus
/// [`Endpoint0`] — and directly by anything that genuinely is not, which so far
/// is nothing.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0).
pub trait UsbDevice: Send + Sync + fmt::Debug {
    /// How fast this device signals. Fixed for its lifetime: a device does not
    /// change speed, it is a different device.
    fn speed(&self) -> Speed;

    /// The address it currently answers to.
    ///
    /// [`DeviceAddress::DEFAULT`] until `SET_ADDRESS` has been issued *and its
    /// status stage has completed* — the ordering matters, and §9.4.6 is
    /// explicit about it, because the status stage itself is addressed to the
    /// old address.
    fn address(&self) -> DeviceAddress;

    /// The port was reset (USB 2.0 §9.1.1.3).
    ///
    /// Back to the Default state: address zero, unconfigured, endpoints
    /// unhalted, toggles cleared.
    fn bus_reset(&self);

    /// A `SETUP` transaction on a control endpoint.
    ///
    /// Returns the handshake. A device that does not understand the request
    /// answers [`Status::Ack`] here and [`Status::Stall`] to the following data
    /// or status stage, which is what §9.2.7 describes — but answering
    /// [`Status::Stall`] straight away is also legal and simpler, and this tree
    /// does the latter.
    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status;

    /// An `IN` transaction: the device fills `dst` and says how much.
    ///
    /// `dst.len()` is the host's `wMaxPacketSize` for this endpoint, so
    /// returning less is a short packet and is meaningful.
    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion;

    /// An `OUT` transaction: the device takes `src`.
    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion;

    /// What an `IN` would return, without taking it.
    ///
    /// The [`crate::core::space::MemAttrs::debug`] rule, applied to a bus: a
    /// debugger that wanted to show what is queued on an endpoint must not pop
    /// it. Defaults to [`Completion::nak`], which is the honest answer for a
    /// device that cannot show its FIFO without consuming it.
    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let _ = (endpoint, dst);
        Completion::nak()
    }
}

// ---------------------------------------------------------------------------
// The fabric
// ---------------------------------------------------------------------------

/// How many ports one [`UsbBus`] routes.
///
/// Fifteen, because that is what an EHCI controller can report: `HCSPARAMS`'s
/// `N_PORTS` field is four bits and zero means "no ports" (EHCI 1.0 §2.2.3). A
/// device asking for a sixteenth is a configuration error at construction, not
/// a silent wrap.
pub const MAX_PORTS: usize = 15;

/// One port of a [`UsbBus`].
#[derive(Debug, Clone, Default)]
struct Slot {
    device: Option<Arc<dyn UsbDevice>>,
    /// Whether the controller has enabled the port.
    ///
    /// The **controller's** decision, not the fabric's: enable is a bit in
    /// `PORTSC` and it is set only after a successful reset. It lives here
    /// because routing has to honour it — two devices freshly attached both
    /// answer to address zero, and only the enabled one may be talked to.
    enabled: bool,
    /// Whether something has plugged or unplugged since the controller last
    /// looked.
    changed: bool,
}

/// One USB bus: a set of ports, each with at most one device.
///
/// The fabric proper. It holds no timing and no clock — *time belongs to the
/// controller*, which is the only thing on the link with a clock domain
/// (`CLAUDE.md`: the scheduler owns time) — and it holds no schedule, because a
/// schedule is a controller's private encoding of what to do next.
pub struct UsbBus {
    ports: Mutex<Vec<Slot>>,
    /// One bit per port whose connection state has changed. Published
    /// lock-free so a controller can ask on every microframe without taking
    /// the fabric lock when nothing has happened.
    changes: AtomicU32,
}

impl fmt::Debug for UsbBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("UsbBus");
        s.field("changes", &self.changes.load(Ordering::Relaxed));
        match self.ports.try_lock() {
            Some(ports) => {
                s.field("ports", &ports.len());
                s.field(
                    "attached",
                    &ports.iter().filter(|p| p.device.is_some()).count(),
                )
            }
            None => s.field("ports", &"<in use>"),
        };
        s.finish()
    }
}

impl UsbBus {
    /// A bus with `ports` ports, nothing attached.
    ///
    /// `ports` is clamped into `1..=`[`MAX_PORTS`]: a bus with no ports is not
    /// a bus, and a sixteenth port cannot be reported by a controller.
    #[must_use]
    pub fn new(ports: u8) -> UsbBus {
        let count = usize::from(ports).clamp(1, MAX_PORTS);
        UsbBus {
            ports: Mutex::with_rank(FABRIC_RANK, alloc::vec![Slot::default(); count]),
            changes: AtomicU32::new(0),
        }
    }

    /// How many ports this bus routes.
    #[must_use]
    pub fn port_count(&self) -> u8 {
        self.ports.lock().len() as u8
    }

    /// Plug `device` into `port`.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if the port does not exist or already has
    /// something in it. Two devices on one port is not a topology, it is a
    /// machine description bug.
    pub fn attach(&self, port: u8, device: Arc<dyn UsbDevice>) -> crate::Result<()> {
        let index = usize::from(port);
        let mut ports = self.ports.lock();
        if index >= ports.len() {
            return Err(crate::Error::Config {
                at: alloc::format!("port {port}"),
                message: alloc::format!("this USB bus has {} ports", ports.len()),
            });
        }
        if ports[index].device.is_some() {
            return Err(crate::Error::Config {
                at: alloc::format!("port {port}"),
                message: alloc::string::String::from(
                    "two devices on one USB port; give one of them another `port`, \
                     or add a hub — which this tree does not model yet",
                ),
            });
        }
        ports[index].device = Some(device);
        ports[index].enabled = false;
        ports[index].changed = true;
        drop(ports);
        self.changes.fetch_or(1u32 << port, Ordering::Relaxed);
        Ok(())
    }

    /// Unplug whatever is in `port`, reporting whether there was anything.
    pub fn detach(&self, port: u8) -> bool {
        let index = usize::from(port);
        let mut ports = self.ports.lock();
        if index >= ports.len() {
            return false;
        }
        let had = ports[index].device.take().is_some();
        ports[index].enabled = false;
        if had {
            ports[index].changed = true;
        }
        drop(ports);
        if had {
            self.changes.fetch_or(1u32 << port, Ordering::Relaxed);
        }
        had
    }

    /// Whatever is in `port`.
    #[must_use]
    pub fn device(&self, port: u8) -> Option<Arc<dyn UsbDevice>> {
        let index = usize::from(port);
        self.ports.lock().get(index).and_then(|p| p.device.clone())
    }

    /// Whether anything is in `port`.
    #[must_use]
    pub fn connected(&self, port: u8) -> bool {
        self.device(port).is_some()
    }

    /// How fast whatever is in `port` signals.
    #[must_use]
    pub fn speed(&self, port: u8) -> Option<Speed> {
        // Cloned out first: `speed` is a call into the device, and the fabric
        // lock is not held across it (`core::device`, the re-entrancy
        // contract).
        self.device(port).map(|d| d.speed())
    }

    /// Whether the controller has enabled `port`.
    #[must_use]
    pub fn enabled(&self, port: u8) -> bool {
        let index = usize::from(port);
        self.ports.lock().get(index).is_some_and(|p| p.enabled)
    }

    /// Enable or disable `port`. The controller's call, not the fabric's.
    pub fn set_enabled(&self, port: u8, enabled: bool) {
        let index = usize::from(port);
        if let Some(slot) = self.ports.lock().get_mut(index) {
            slot.enabled = enabled;
        }
    }

    /// Whether `port` has been plugged or unplugged since this was last asked,
    /// clearing the flag.
    ///
    /// The controller's poll: EHCI turns a `true` here into `PORTSC`'s
    /// *Connect Status Change* bit and `USBSTS`'s *Port Change Detect*.
    pub fn take_change(&self, port: u8) -> bool {
        let index = usize::from(port);
        let mut ports = self.ports.lock();
        let Some(slot) = ports.get_mut(index) else {
            return false;
        };
        let changed = core::mem::replace(&mut slot.changed, false);
        drop(ports);
        if changed {
            self.changes.fetch_and(!(1u32 << port), Ordering::Relaxed);
        }
        changed
    }

    /// Whether *any* port has an unread change, without clearing anything.
    ///
    /// Lock-free, so a controller can ask on every microframe.
    #[must_use]
    pub fn any_change(&self) -> bool {
        self.changes.load(Ordering::Relaxed) != 0
    }

    /// Drive a bus reset into whatever is in `port` (USB 2.0 §7.1.7.5).
    ///
    /// The device returns to the Default state; the port is left *disabled*,
    /// because deciding to enable it is the controller's — a high-speed
    /// controller that finds a full-speed device hands the port to a companion
    /// instead.
    pub fn reset_port(&self, port: u8) {
        self.set_enabled(port, false);
        // Outside the lock: a device may reach further from its own reset.
        if let Some(device) = self.device(port) {
            device.bus_reset();
        }
    }

    /// The device answering to `address` on an enabled port, if any.
    ///
    /// Ports are searched in index order, which is deterministic and is what
    /// makes two freshly attached devices — both at
    /// [`DeviceAddress::DEFAULT`] — a *modelled* hazard rather than a
    /// hash-order coin flip. In practice a host resets one port at a time
    /// precisely so the situation does not arise (§9.1.2), and only the
    /// enabled port is searched, so it does not.
    #[must_use]
    pub fn find(&self, address: DeviceAddress) -> Option<Arc<dyn UsbDevice>> {
        let candidates: Vec<Arc<dyn UsbDevice>> = {
            let ports = self.ports.lock();
            ports
                .iter()
                .filter(|p| p.enabled)
                .filter_map(|p| p.device.clone())
                .collect()
        };
        // The address query is a call into the device, so the lock is released
        // first.
        candidates.into_iter().find(|d| d.address() == address)
    }

    /// A `SETUP` transaction to `address`, endpoint `endpoint`.
    pub fn setup(&self, address: DeviceAddress, endpoint: u8, packet: SetupPacket) -> Status {
        match self.find(address) {
            Some(device) => device.setup(endpoint, packet),
            None => Status::NoDevice,
        }
    }

    /// An `IN` transaction from `address`, endpoint `endpoint`.
    pub fn read(&self, address: DeviceAddress, endpoint: u8, dst: &mut [u8]) -> Completion {
        match self.find(address) {
            Some(device) => device.transfer_in(endpoint, dst),
            None => Completion::absent(),
        }
    }

    /// An `OUT` transaction to `address`, endpoint `endpoint`.
    pub fn write(&self, address: DeviceAddress, endpoint: u8, src: &[u8]) -> Completion {
        match self.find(address) {
            Some(device) => device.transfer_out(endpoint, src),
            None => Completion::absent(),
        }
    }

    /// What an `IN` would return, without taking it. The debug path.
    #[must_use]
    pub fn peek(&self, address: DeviceAddress, endpoint: u8, dst: &mut [u8]) -> Completion {
        match self.find(address) {
            Some(device) => device.peek_in(endpoint, dst),
            None => Completion::absent(),
        }
    }
}

/// The named rendezvous: how a controller and its devices find each other.
///
/// Modelled on [`crate::bus::spi::buses`] and, before it,
/// [`crate::host::chardev::ports`], and a seam for the same reason — a machine
/// description can hand two independently constructed devices only a *name*,
/// and `core::bus` (`ROADMAP.md` §4) does not exist yet. When it does, this
/// becomes its registry and every device-facing signature here stays as it is.
///
/// ```
/// # #[cfg(feature = "bus-usb")] {
/// use rsemu::bus::usb::buses;
///
/// use std::sync::Arc;
///
/// let a = buses::open("doctest-usb", 2);
/// let b = buses::open("doctest-usb", 8);
/// assert!(Arc::ptr_eq(&a, &b), "the same name is the same bus");
/// assert_eq!(a.port_count(), 2, "the first mention fixes the port count");
/// buses::close("doctest-usb");
/// # }
/// ```
pub mod buses {
    use super::UsbBus;
    use alloc::collections::BTreeMap;
    use alloc::string::{String, ToString};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::sync::{Global, LockRank};

    /// Name to bus. `BTreeMap`, so listing is in name order rather than hash
    /// order (`CLAUDE.md`, determinism); [`Global`] because a `static` is
    /// reachable from every thread in the process (`core::sync`).
    static TABLE: Global<BTreeMap<String, Arc<UsbBus>>> =
        Global::with_rank(LockRank::LEAF, BTreeMap::new());

    /// The bus called `name`, creating it with `ports` ports if this is the
    /// first mention.
    ///
    /// Both ends call this, and whichever is constructed first fixes the port
    /// count — which is the controller in every machine description that makes
    /// sense, since a device's `port` property has to be inside it.
    #[must_use]
    pub fn open(name: &str, ports: u8) -> Arc<UsbBus> {
        let mut table = TABLE.lock();
        if let Some(bus) = table.get(name) {
            return Arc::clone(bus);
        }
        let bus = Arc::new(UsbBus::new(ports));
        table.insert(name.to_string(), Arc::clone(&bus));
        bus
    }

    /// The bus called `name`, if it has been opened.
    #[must_use]
    pub fn get(name: &str) -> Option<Arc<UsbBus>> {
        TABLE.lock().get(name).map(Arc::clone)
    }

    /// Forget `name`, reporting whether there was one.
    ///
    /// Anything still holding the `Arc` keeps working; a later [`open`] of the
    /// same name is a fresh bus. For tests that want the name back.
    pub fn close(name: &str) -> bool {
        TABLE.lock().remove(name).is_some()
    }

    /// Every open bus's name, in order.
    #[must_use]
    pub fn names() -> Vec<String> {
        TABLE.lock().keys().cloned().collect()
    }
}
