//! The default control pipe, written once for every device that will ever
//! exist (USB 2.0 §9.4).
//!
//! # Why this is not a device model's problem
//!
//! Every USB device answers the same eleven standard requests, out of the same
//! descriptor tables, with the same three-stage control transfer underneath.
//! Writing that per device means getting it subtly wrong per device — the
//! classic being `SET_ADDRESS`, whose new address takes effect **after** the
//! status stage rather than when the request arrives (§9.4.6), because the
//! status stage is itself addressed to the old address. A device model that
//! switched immediately would enumerate on some hosts and not others.
//!
//! So [`Endpoint0`] is that state machine, a [`Function`] is the part that is
//! genuinely the device's — its descriptors, its class requests, its endpoints
//! — and [`Peripheral`] is the two of them as a [`super::UsbDevice`]. It is
//! the same division [`crate::bus::spi::Shifter`] makes for SPI and for the
//! same reason.
//!
//! # The three stages
//!
//! ```text
//!   control IN   SETUP ──► data IN … ──► status OUT (zero length)
//!   control OUT  SETUP ──► data OUT … ──► status IN  (zero length)
//!   no data      SETUP ──────────────► status IN  (zero length)
//! ```
//!
//! A host is allowed to end a data stage early by moving straight to the
//! status stage, and [`Endpoint0`] accepts that (§8.5.3.4): an `OUT` arriving
//! during a data-`IN` stage *is* the status stage.
//!
//! # Stalling
//!
//! §9.2.7 says a device acknowledges the `SETUP` packet unconditionally — a
//! `SETUP` is never `NAK`ed or `STALL`ed — and reports a request it does not
//! support by stalling the *following* data or status stage. This module does
//! exactly that rather than the tempting shortcut of refusing the setup, so a
//! host controller sees the halt land on the qTD the specification says it
//! lands on.

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::descriptor::{ConfigurationDescriptor, DescriptorKind, Descriptors};
use super::{
    Completion, DeviceAddress, Direction, EP0_RANK, Recipient, RequestKind, SetupPacket, Speed,
    Status, UsbDevice, feature, request,
};
use crate::core::error::{Error, Result};
use crate::core::state::{Sink, Source};
use crate::core::sync::{LockRank, Mutex};

/// The class-specific half of a device: what [`Endpoint0`] cannot know.
///
/// Everything is defaulted except the two things that genuinely differ per
/// device — what it says about itself, and how fast it signals — so a mouse is
/// a report descriptor, one class request and an interrupt endpoint, and
/// nothing else.
pub trait Function: Send + Sync + fmt::Debug {
    /// What this device says about itself. Built once, in the constructor.
    fn descriptors(&self) -> &Descriptors;

    /// How fast it signals.
    fn speed(&self) -> Speed;

    /// A bus reset arrived; return to the power-on state (§9.1.1.3).
    ///
    /// [`Endpoint0`] has already cleared the address, the configuration and
    /// the halt bits — this is for the class's own state.
    fn reset(&self) {}

    /// `SET_CONFIGURATION` selected `value`. Return whether it was accepted.
    ///
    /// `value` is a `bConfigurationValue` that [`Endpoint0`] has already
    /// checked exists, or zero for "unconfigured", so a device with one
    /// configuration can ignore this entirely.
    fn configure(&self, value: u8) -> bool {
        let _ = value;
        true
    }

    /// A class or vendor request in the device-to-host direction, or a
    /// `GET_DESCRIPTOR` for a type this module does not define — HID's report
    /// descriptor is one.
    ///
    /// Return the bytes, or `None` to stall.
    fn control_in(&self, setup: SetupPacket) -> Option<Vec<u8>> {
        let _ = setup;
        None
    }

    /// A class or vendor request in the host-to-device direction, with its
    /// data stage (empty when `wLength` is zero).
    ///
    /// Return whether it was accepted; `false` stalls.
    fn control_out(&self, setup: SetupPacket, data: &[u8]) -> bool {
        let _ = (setup, data);
        false
    }

    /// The host cleared the halt condition on `endpoint` — a `CLEAR_FEATURE`
    /// naming `ENDPOINT_HALT` (USB 2.0 §9.4.1).
    ///
    /// `endpoint` is a `bEndpointAddress`, direction bit and all, exactly as
    /// `wIndex` carried it. [`Endpoint0`] has already cleared its own halt bit;
    /// this is for a class that halted an endpoint *itself* and has to know
    /// when the host let it go.
    ///
    /// **The reason it exists**, because a hook with one caller deserves one:
    /// a device may stall a bulk endpoint as a protocol signal rather than
    /// because [`Endpoint0`] was told to, and the class specification may then
    /// say the stall survives a class reset. USB Mass Storage's Bulk-Only
    /// Transport is exactly that — §3.1: *"The device shall preserve the value
    /// of its bulk data toggle bits and endpoint STALL conditions despite the
    /// Bulk-Only Mass Storage Reset"* — so the only thing that may clear such a
    /// stall is the `CLEAR_FEATURE` the host sends afterwards, and without this
    /// the class would never hear about it.
    ///
    /// Additive, with a default that does nothing, so no existing device model
    /// changed when it arrived.
    fn halt_cleared(&self, endpoint: u8) {
        let _ = endpoint;
    }

    /// An `IN` transaction on endpoint `endpoint` — never zero, which
    /// [`Endpoint0`] owns.
    ///
    /// `dst` is one `wMaxPacketSize`, so returning fewer bytes is a short
    /// packet and means it.
    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let _ = (endpoint, dst);
        Completion::stall()
    }

    /// An `OUT` transaction on endpoint `endpoint`.
    fn endpoint_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        let _ = (endpoint, src);
        Completion::stall()
    }

    /// What an `IN` would return, without taking it — the debug path
    /// ([`super::UsbDevice::peek_in`]). **Must have no side effects.**
    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let _ = (endpoint, dst);
        Completion::nak()
    }
}

/// Which stage of a control transfer the default pipe is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Stage {
    /// No transfer in progress.
    #[default]
    Idle,
    /// Device to host: the host is collecting `buffer`.
    DataIn,
    /// Host to device: the host is filling `buffer`.
    DataOut,
    /// Waiting for the zero-length `IN` that ends a host-to-device transfer.
    StatusIn,
}

impl Stage {
    const fn code(self) -> u8 {
        match self {
            Stage::Idle => 0,
            Stage::DataIn => 1,
            Stage::DataOut => 2,
            Stage::StatusIn => 3,
        }
    }

    const fn from_code(code: u8) -> Stage {
        match code {
            1 => Stage::DataIn,
            2 => Stage::DataOut,
            3 => Stage::StatusIn,
            _ => Stage::Idle,
        }
    }
}

/// Everything the default control pipe remembers.
///
/// **All of it is guest-visible state and all of it is snapshotted**, including
/// a control transfer that is part-way through: a host that has issued the
/// `SETUP` and half the data stage, and a machine snapshotted between the two,
/// must resume into the same half-finished transfer or the driver sees a
/// descriptor with a hole in it.
#[derive(Debug, Clone, Default)]
struct Ep0State {
    /// The address in force now.
    address: u8,
    /// The address `SET_ADDRESS` asked for, applied when the status stage
    /// completes (§9.4.6).
    pending_address: Option<u8>,
    /// `bConfigurationValue`, or zero for unconfigured.
    configuration: u8,
    stage: Stage,
    /// The request being served.
    setup: SetupPacket,
    /// The data stage's bytes: what is being sent, or what has arrived.
    buffer: Vec<u8>,
    /// How much of `buffer` has been sent, for a device-to-host stage.
    offset: usize,
    /// Whether the rest of this transfer is to be stalled (§9.2.7).
    stalled: bool,
    /// One bit per halted endpoint: bit `n` for `OUT` `n`, bit `n + 16` for
    /// `IN` `n`.
    halted: u32,
    /// The `DEVICE_REMOTE_WAKEUP` feature.
    remote_wakeup: bool,
}

/// The default control pipe: endpoint zero, and the standard requests.
///
/// # Locking
///
/// One mutex at [`EP0_RANK`], between the fabric and the device's own state,
/// and it **is** held across calls into the [`Function`] — deliberately, as
/// [`crate::bus::spi::SlavePins`] holds its shifter across the call into the
/// slave. Serving a request is one step and letting a second transaction
/// interleave in the middle of it would be a bug, not a feature. The function's
/// own state therefore has to rank below, which [`LockRank::DEVICE`] does.
pub struct Endpoint0 {
    function: Arc<dyn Function>,
    state: Mutex<Ep0State>,
}

impl fmt::Debug for Endpoint0 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Endpoint0");
        s.field("function", &self.function);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Endpoint0 {
    /// A control pipe serving `function`.
    #[must_use]
    pub fn new(function: Arc<dyn Function>) -> Endpoint0 {
        Endpoint0 {
            function,
            state: Mutex::with_rank(EP0_RANK, Ep0State::default()),
        }
    }

    /// The device behind this pipe.
    #[must_use]
    pub fn function(&self) -> &Arc<dyn Function> {
        &self.function
    }

    /// The address the device answers to now.
    #[must_use]
    pub fn address(&self) -> DeviceAddress {
        DeviceAddress(self.state.lock().address)
    }

    /// The configuration in force, or zero for unconfigured.
    #[must_use]
    pub fn configuration(&self) -> u8 {
        self.state.lock().configuration
    }

    /// Whether endpoint `endpoint` (a `bEndpointAddress`, direction bit and
    /// all) is halted.
    #[must_use]
    pub fn is_halted(&self, endpoint: u8) -> bool {
        self.state.lock().halted & Ep0State::halt_bit(endpoint) != 0
    }

    /// Return to the Default state: address zero, unconfigured, nothing halted
    /// (§9.1.1.3).
    pub fn bus_reset(&self) {
        {
            let mut state = self.state.lock();
            *state = Ep0State::default();
        }
        self.function.reset();
    }

    /// A `SETUP` transaction.
    ///
    /// Always acknowledged (§9.2.7); a request this device does not support
    /// arms a stall that the following data or status stage delivers.
    pub fn setup(&self, packet: SetupPacket) -> Status {
        let mut state = self.state.lock();
        state.setup = packet;
        state.buffer.clear();
        state.offset = 0;
        state.stalled = false;
        state.stage = Stage::Idle;

        let handled = match packet.kind() {
            RequestKind::Standard => self.standard(&mut state, packet),
            RequestKind::Class | RequestKind::Vendor => self.class(&mut state, packet),
            RequestKind::Reserved => false,
        };
        if !handled {
            state.stalled = true;
        }
        Status::Ack
    }

    /// An `IN` transaction on endpoint zero: a data-stage packet, or the
    /// status stage of a host-to-device transfer.
    pub fn read(&self, dst: &mut [u8]) -> Completion {
        let mut state = self.state.lock();
        if state.stalled {
            return Completion::stall();
        }
        match state.stage {
            Stage::DataIn => {
                let remaining = state.buffer.len().saturating_sub(state.offset);
                let n = remaining.min(dst.len());
                dst[..n].copy_from_slice(&state.buffer[state.offset..state.offset + n]);
                state.offset += n;
                Completion::ack(n as u64)
            }
            Stage::StatusIn => {
                // The status stage completed. This is the moment `SET_ADDRESS`
                // takes effect (§9.4.6) — not when the request arrived.
                if let Some(address) = state.pending_address.take() {
                    state.address = address;
                }
                state.stage = Stage::Idle;
                Completion::ack(0)
            }
            // An `IN` with nothing outstanding, or during a host-to-device
            // data stage, is a protocol error the device answers with a stall.
            Stage::Idle | Stage::DataOut => Completion::stall(),
        }
    }

    /// An `OUT` transaction on endpoint zero: a data-stage packet, or the
    /// status stage of a device-to-host transfer.
    pub fn write(&self, src: &[u8]) -> Completion {
        let mut state = self.state.lock();
        if state.stalled {
            return Completion::stall();
        }
        match state.stage {
            // §8.5.3.4: the host may end a data-IN stage early, and the way it
            // does so is by moving to the status stage.
            Stage::DataIn => {
                state.stage = Stage::Idle;
                Completion::ack(0)
            }
            Stage::DataOut => {
                let want = usize::from(state.setup.length);
                let room = want.saturating_sub(state.buffer.len());
                let n = room.min(src.len());
                state.buffer.extend_from_slice(&src[..n]);
                if state.buffer.len() >= want || n < src.len() {
                    let setup = state.setup;
                    let data = core::mem::take(&mut state.buffer);
                    let accepted = match setup.kind() {
                        RequestKind::Standard => self.standard_out(&mut state, setup, &data),
                        RequestKind::Class | RequestKind::Vendor => {
                            self.function.control_out(setup, &data)
                        }
                        RequestKind::Reserved => false,
                    };
                    if accepted {
                        state.stage = Stage::StatusIn;
                    } else {
                        state.stalled = true;
                    }
                }
                Completion::ack(n as u64)
            }
            Stage::Idle | Stage::StatusIn => Completion::stall(),
        }
    }

    /// Everything a snapshot needs, written in the class's own chunk.
    ///
    /// # Errors
    ///
    /// Whatever the sink refuses.
    pub fn save<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        let state = self.state.lock();
        w.write_u8(state.address)?;
        w.write_u8(state.pending_address.unwrap_or(0))?;
        w.write_bool(state.pending_address.is_some())?;
        w.write_u8(state.configuration)?;
        w.write_u8(state.stage.code())?;
        w.write_all(&state.setup.encode())?;
        w.write_bytes(&state.buffer)?;
        w.write_u64(state.offset as u64)?;
        w.write_bool(state.stalled)?;
        w.write_u32(state.halted)?;
        w.write_bool(state.remote_wakeup)
    }

    /// Restore what [`save`](Endpoint0::save) wrote.
    ///
    /// # Errors
    ///
    /// [`Error::State`] for a truncated or malformed chunk.
    pub fn load<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let address = r.read_u8()?;
        let pending = r.read_u8()?;
        let has_pending = r.read_bool()?;
        let configuration = r.read_u8()?;
        let stage = Stage::from_code(r.read_u8()?);
        let mut setup = [0u8; 8];
        setup.copy_from_slice(r.take(8)?);
        let buffer = r.read_bytes()?.to_vec();
        let offset = r.read_u64()?;
        let stalled = r.read_bool()?;
        let halted = r.read_u32()?;
        let remote_wakeup = r.read_bool()?;

        let offset = usize::try_from(offset).map_err(|_| {
            Error::State(alloc::string::String::from(
                "usb: a control-stage offset larger than this host's address space",
            ))
        })?;
        if offset > buffer.len() {
            return Err(Error::State(alloc::format!(
                "usb: a control transfer {offset} bytes into a {}-byte buffer",
                buffer.len()
            )));
        }
        let mut state = self.state.lock();
        *state = Ep0State {
            address: address & DeviceAddress::MAX,
            pending_address: has_pending.then_some(pending & DeviceAddress::MAX),
            configuration,
            stage,
            setup: SetupPacket::decode(&setup),
            buffer,
            offset,
            stalled,
            halted,
            remote_wakeup,
        };
        Ok(())
    }

    // -- the standard requests (§9.4) ---------------------------------------

    /// Serve a standard request. Returns whether it was understood.
    fn standard(&self, state: &mut Ep0State, packet: SetupPacket) -> bool {
        if packet.direction() == Direction::In {
            let Some(bytes) = self.standard_in(state, packet) else {
                return false;
            };
            let want = usize::from(packet.length);
            state.buffer = bytes;
            state.buffer.truncate(want);
            state.stage = if want == 0 {
                Stage::StatusIn
            } else {
                Stage::DataIn
            };
            return true;
        }
        // Host to device. With no data stage the request is served now and the
        // status stage is all that remains.
        if packet.length == 0 {
            if !self.standard_out(state, packet, &[]) {
                return false;
            }
            state.stage = Stage::StatusIn;
            return true;
        }
        // The only standard host-to-device request with a data stage is
        // `SET_DESCRIPTOR`, which is optional and which nothing in this tree
        // implements (§9.4.8).
        false
    }

    /// The device-to-host standard requests. `None` stalls.
    fn standard_in(&self, state: &Ep0State, packet: SetupPacket) -> Option<Vec<u8>> {
        let descriptors = self.function.descriptors();
        match (packet.request, packet.recipient()) {
            (request::GET_STATUS, Recipient::Device) => {
                // §9.4.5: bit 0 self-powered, bit 1 remote wakeup. The
                // self-powered bit restates the configuration's `bmAttributes`,
                // so it comes from the descriptor rather than from a second
                // copy that could disagree with it.
                let self_powered = descriptors
                    .attributes_of(state.configuration)
                    .is_some_and(|a| a & ConfigurationDescriptor::SELF_POWERED != 0);
                let status = u8::from(self_powered) | (u8::from(state.remote_wakeup) << 1);
                Some(alloc::vec![status, 0])
            }
            (request::GET_STATUS, Recipient::Interface) => Some(alloc::vec![0, 0]),
            (request::GET_STATUS, Recipient::Endpoint) => {
                let halted = state.halted & Ep0State::halt_bit(packet.index as u8) != 0;
                Some(alloc::vec![u8::from(halted), 0])
            }
            (request::GET_DESCRIPTOR, Recipient::Device) => {
                let (kind, index) = packet.descriptor();
                let kind = DescriptorKind(kind);
                if kind.is_standard() {
                    descriptors.get(kind, index).map(<[u8]>::to_vec)
                } else {
                    // A class-specific descriptor addressed to the device.
                    // Rare, and the class's to answer.
                    self.function.control_in(packet)
                }
            }
            // HID's report descriptor arrives this way: a `GET_DESCRIPTOR`
            // whose recipient is the interface (HID 1.11 §7.1.1).
            (request::GET_DESCRIPTOR, _) => self.function.control_in(packet),
            (request::GET_CONFIGURATION, Recipient::Device) => {
                Some(alloc::vec![state.configuration])
            }
            (request::GET_INTERFACE, Recipient::Interface) => {
                // No device here has an alternate setting; §9.4.4 requires
                // the request to be answered anyway.
                Some(alloc::vec![0])
            }
            _ => None,
        }
    }

    /// The host-to-device standard requests. Returns whether accepted.
    fn standard_out(&self, state: &mut Ep0State, packet: SetupPacket, data: &[u8]) -> bool {
        let _ = data;
        match (packet.request, packet.recipient()) {
            (request::SET_ADDRESS, Recipient::Device) => {
                if packet.value > u16::from(DeviceAddress::MAX) {
                    return false;
                }
                // Deferred to the status stage on purpose (§9.4.6).
                state.pending_address = Some(packet.value as u8);
                true
            }
            (request::SET_CONFIGURATION, Recipient::Device) => {
                let value = packet.value as u8;
                if value != 0 && !self.function.descriptors().has_configuration_value(value) {
                    return false;
                }
                if !self.function.configure(value) {
                    return false;
                }
                state.configuration = value;
                // §9.4.7: configuring clears every halt and resets the toggles.
                state.halted = 0;
                true
            }
            (request::SET_INTERFACE, Recipient::Interface) => packet.value == 0,
            (request::CLEAR_FEATURE, Recipient::Endpoint)
            | (request::SET_FEATURE, Recipient::Endpoint) => {
                if packet.value != feature::ENDPOINT_HALT {
                    return false;
                }
                let bit = Ep0State::halt_bit(packet.index as u8);
                if packet.request == request::SET_FEATURE {
                    state.halted |= bit;
                } else {
                    state.halted &= !bit;
                    // A class may have halted this endpoint itself, in which
                    // case clearing our bit is only half the job — see
                    // [`Function::halt_cleared`]. Called with the pipe's lock
                    // held, which is this module's documented contract for
                    // every call into the function.
                    self.function.halt_cleared(packet.index as u8);
                }
                true
            }
            (request::CLEAR_FEATURE, Recipient::Device)
            | (request::SET_FEATURE, Recipient::Device) => match packet.value {
                feature::DEVICE_REMOTE_WAKEUP => {
                    state.remote_wakeup = packet.request == request::SET_FEATURE;
                    true
                }
                // Electrical test modes. Accepted and ignored: a modelled bus
                // has no eye diagram, and refusing would fail a compliance
                // driver for no benefit.
                feature::TEST_MODE => true,
                _ => false,
            },
            _ => false,
        }
    }

    /// A class or vendor request.
    fn class(&self, state: &mut Ep0State, packet: SetupPacket) -> bool {
        if packet.direction() == Direction::In {
            let Some(bytes) = self.function.control_in(packet) else {
                return false;
            };
            let want = usize::from(packet.length);
            state.buffer = bytes;
            state.buffer.truncate(want);
            state.stage = if want == 0 {
                Stage::StatusIn
            } else {
                Stage::DataIn
            };
            return true;
        }
        if packet.length == 0 {
            if !self.function.control_out(packet, &[]) {
                return false;
            }
            state.stage = Stage::StatusIn;
            return true;
        }
        // The data stage arrives in `write`, and the function is called once
        // all of it has.
        state.stage = Stage::DataOut;
        true
    }
}

impl Ep0State {
    /// Which bit of `halted` an endpoint address occupies.
    const fn halt_bit(address: u8) -> u32 {
        let number = (address & 0x0f) as u32;
        if address & Direction::BIT != 0 {
            1u32 << (number + 16)
        } else {
            1u32 << number
        }
    }
}

/// A [`Function`] plus its default control pipe: an ordinary USB device.
///
/// What a device model hands to [`super::UsbBus::attach`].
#[derive(Debug)]
pub struct Peripheral {
    ep0: Endpoint0,
}

impl Peripheral {
    /// Wrap `function` in a default control pipe.
    #[must_use]
    pub fn new(function: Arc<dyn Function>) -> Peripheral {
        Peripheral {
            ep0: Endpoint0::new(function),
        }
    }

    /// The control pipe, for a device model's `save`/`load`.
    #[must_use]
    pub fn endpoint0(&self) -> &Endpoint0 {
        &self.ep0
    }
}

impl UsbDevice for Peripheral {
    fn speed(&self) -> Speed {
        self.ep0.function().speed()
    }

    fn address(&self) -> DeviceAddress {
        self.ep0.address()
    }

    fn bus_reset(&self) {
        self.ep0.bus_reset();
    }

    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status {
        if endpoint != 0 {
            // A second control endpoint is legal USB and nothing models one.
            return Status::Stall;
        }
        self.ep0.setup(packet)
    }

    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint == 0 {
            return self.ep0.read(dst);
        }
        if self.ep0.is_halted(endpoint | Direction::BIT) {
            return Completion::stall();
        }
        self.ep0.function().endpoint_in(endpoint, dst)
    }

    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        if endpoint == 0 {
            return self.ep0.write(src);
        }
        if self.ep0.is_halted(endpoint) {
            return Completion::stall();
        }
        self.ep0.function().endpoint_out(endpoint, src)
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint == 0 {
            // The control pipe's outgoing bytes are known without consuming
            // anything, so a debugger may see them.
            let state = self.ep0.state.lock();
            if state.stalled || state.stage != Stage::DataIn {
                return Completion::nak();
            }
            let remaining = state.buffer.len().saturating_sub(state.offset);
            let n = remaining.min(dst.len());
            dst[..n].copy_from_slice(&state.buffer[state.offset..state.offset + n]);
            return Completion::ack(n as u64);
        }
        self.ep0.function().peek_in(endpoint, dst)
    }
}

const _: () = {
    // A compile-time reminder that `EP0_RANK` sits where the module docs say
    // it does: under a CPU's session lock, over a device's own state.
    assert!(EP0_RANK.0 > LockRank::BUS.0);
    assert!(EP0_RANK.0 < LockRank::DEVICE.0);
};
