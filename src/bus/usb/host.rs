//! The *host* side of a transfer, for anything driving the bus that is not a
//! modelled host controller.
//!
//! # Why this exists, and why it did not before
//!
//! [`UsbBus`] speaks transactions, and until now every caller of
//! [`UsbBus::setup`], [`UsbBus::read`] and [`UsbBus::write`] was a host
//! controller with a schedule of its own: an EHCI walking queue heads, a dwc2
//! draining host channels. *Which* sequence of transactions a control transfer
//! is made of lived in the controller, which is the right place for it, because
//! the controller is what holds the schedule (see the module docs).
//!
//! Then the guest became the device. Something has to *enumerate* it, and that
//! something is not always a controller:
//!
//! * a test harness, which is what proves a device-mode controller works at
//!   all;
//! * a loopback between two controllers, or between two machines;
//! * later, a bridge to a real host through `usbfs`, which issues whole
//!   transfers and never sees a token.
//!
//! None of those has a schedule, and every one of them would otherwise write
//! the same three-stage state machine again. So it is written here, once, on
//! the host side — exactly the argument [`super::Endpoint0`] makes on the
//! device side.
//!
//! # It never waits
//!
//! [`ControlTransfer::step`] issues **at most one transaction** and returns what
//! happened. A `NAK` is [`Progress::Nak`] and nothing else: the caller decides
//! how much guest time to let pass before trying again, because on this side of
//! the seam the device may be a guest that has not run yet.
//!
//! ```text
//!   while !xfer.is_finished() {
//!       machine.run_for(a_while)?;          // let the guest make progress
//!       xfer.step(&bus, address, 64);       // one transaction
//!   }
//! ```
//!
//! That is also what bounds it: a caller's loop is a caller's loop, and the
//! data stage can never exceed the `wLength` the setup packet named.
//!
//! # Sources
//!
//! USB 2.0 §8.5.3 (control transfers and their three stages), §8.5.3.4 (a host
//! ending a data stage early by moving to the status stage), §5.8.3 (a short
//! packet ends a transfer) and §9.3 for the setup packet. No emulator source
//! was consulted (`ROADMAP.md` §1).

use alloc::vec::Vec;

use super::{Completion, DeviceAddress, Direction, SetupPacket, Status, UsbBus};

/// What one call to [`ControlTransfer::step`] achieved.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Progress {
    /// A transaction completed and the transfer moved on. Call again.
    Moved,
    /// The device answered `NAK`. Nothing moved and nothing is wrong: let time
    /// pass and call again.
    Nak,
    /// The transfer is over and every stage succeeded.
    Done,
    /// The transfer failed, and this is the handshake that ended it —
    /// [`Status::Stall`] for a request the device refuses, [`Status::NoDevice`]
    /// for an address nothing answers.
    Failed(Status),
}

impl Progress {
    /// Whether this outcome ends the transfer, one way or the other.
    #[must_use]
    pub const fn is_finished(self) -> bool {
        matches!(self, Progress::Done | Progress::Failed(_))
    }
}

/// Which of the three stages a [`ControlTransfer`] is in (USB 2.0 §8.5.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// The eight bytes have not gone out yet.
    Setup,
    /// Collecting the device's answer.
    DataIn,
    /// Handing the device the bytes.
    DataOut,
    /// The zero-length `IN` that ends a host-to-device transfer.
    StatusIn,
    /// The zero-length `OUT` that ends a device-to-host transfer.
    StatusOut,
    /// Over.
    Finished,
}

/// One control transfer, driven a transaction at a time.
///
/// Holds no clock, no bus and no device: it is a state machine over the
/// [`UsbBus`] handed to each [`step`](ControlTransfer::step), so the caller
/// keeps the timing and the retries.
#[derive(Debug, Clone)]
pub struct ControlTransfer {
    setup: SetupPacket,
    endpoint: u8,
    stage: Stage,
    /// The device's answer, or the bytes going out.
    data: Vec<u8>,
    /// How much of an outgoing `data` has been accepted.
    sent: usize,
    /// The handshake that ended it, if it ended badly.
    failure: Option<Status>,
}

impl ControlTransfer {
    /// A device-to-host transfer: a `SETUP`, a data stage the device fills, and
    /// a zero-length `OUT` status stage.
    ///
    /// The direction bit of `setup.request_type` is not consulted — the
    /// constructor is the direction — so a `bmRequestType` that disagrees with
    /// it reaches the device unaltered, which is what a controller would do
    /// with one and is a case a device is entitled to stall.
    #[must_use]
    pub fn device_to_host(setup: SetupPacket) -> ControlTransfer {
        ControlTransfer {
            setup,
            endpoint: 0,
            stage: Stage::Setup,
            data: Vec::new(),
            sent: 0,
            failure: None,
        }
    }

    /// A host-to-device transfer carrying `data` — empty for a request with no
    /// data stage, which is most of them (`SET_ADDRESS`, `SET_CONFIGURATION`).
    #[must_use]
    pub fn host_to_device(setup: SetupPacket, data: &[u8]) -> ControlTransfer {
        ControlTransfer {
            setup,
            endpoint: 0,
            stage: Stage::Setup,
            data: data.to_vec(),
            sent: 0,
            failure: None,
        }
    }

    /// Run it on a control endpoint other than zero.
    ///
    /// Legal USB, and nothing in this tree has one — but a token carries an
    /// endpoint number and a transfer composer that hard-coded zero would be
    /// making a decision that is not its to make.
    #[must_use]
    pub fn on_endpoint(mut self, endpoint: u8) -> ControlTransfer {
        self.endpoint = endpoint & 0x0f;
        self
    }

    /// The request being served.
    #[must_use]
    pub fn request(&self) -> SetupPacket {
        self.setup
    }

    /// The bytes the device returned, or the bytes still going out.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Take the bytes the device returned, leaving the transfer empty.
    #[must_use]
    pub fn take_data(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.data)
    }

    /// Whether the transfer is over, successfully or not.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.stage == Stage::Finished
    }

    /// The handshake that ended it badly, or `None` while it is running or if
    /// it succeeded.
    #[must_use]
    pub fn failure(&self) -> Option<Status> {
        self.failure
    }

    /// Issue **one** transaction and report what happened.
    ///
    /// `max_packet` is `bMaxPacketSize0` for this device — the host's number,
    /// because it is the host that decides how much to ask for in one go. Zero
    /// is treated as eight, the smallest a control endpoint may be (§5.5.3).
    ///
    /// Calling this after the transfer has finished repeats the outcome rather
    /// than restarting anything.
    pub fn step(&mut self, bus: &UsbBus, address: DeviceAddress, max_packet: u16) -> Progress {
        let mps = usize::from(if max_packet == 0 { 8 } else { max_packet });
        match self.stage {
            Stage::Setup => self.do_setup(bus, address),
            Stage::DataIn => self.do_data_in(bus, address, mps),
            Stage::DataOut => self.do_data_out(bus, address, mps),
            Stage::StatusIn => {
                let completion = bus.read(address, self.endpoint, &mut []);
                self.do_status(completion)
            }
            Stage::StatusOut => {
                let completion = bus.write(address, self.endpoint, &[]);
                self.do_status(completion)
            }
            Stage::Finished => match self.failure {
                Some(status) => Progress::Failed(status),
                None => Progress::Done,
            },
        }
    }

    /// The `SETUP` stage. §9.2.7: a device acknowledges a setup packet
    /// unconditionally, so anything but an `ACK` here is the device being
    /// absent or the bus being wrong — never a refusal of the request, which
    /// arrives as a stall on the *next* stage.
    fn do_setup(&mut self, bus: &UsbBus, address: DeviceAddress) -> Progress {
        match bus.setup(address, self.endpoint, self.setup) {
            Status::Ack => {
                self.stage = if self.setup.length == 0 {
                    // No data stage, so what remains is the status stage — and
                    // with no data to go the other way from, it is an `IN`
                    // (§8.5.3).
                    Stage::StatusIn
                } else if self.setup.direction() == Direction::In {
                    Stage::DataIn
                } else {
                    Stage::DataOut
                };
                Progress::Moved
            }
            Status::Nak => Progress::Nak,
            status => self.fail(status),
        }
    }

    fn do_data_in(&mut self, bus: &UsbBus, address: DeviceAddress, mps: usize) -> Progress {
        let want = usize::from(self.setup.length).saturating_sub(self.data.len());
        if want == 0 {
            self.stage = Stage::StatusOut;
            return Progress::Moved;
        }
        let mut buf = alloc::vec![0u8; mps.min(want)];
        let completion = bus.read(address, self.endpoint, &mut buf);
        match completion.status {
            Status::Ack => {
                let n = (completion.len as usize).min(buf.len());
                self.data.extend_from_slice(&buf[..n]);
                // §5.8.3: a packet shorter than the maximum is how a device
                // says "that is all of it", and the host must notice.
                if n < buf.len() || self.data.len() >= usize::from(self.setup.length) {
                    self.stage = Stage::StatusOut;
                }
                Progress::Moved
            }
            Status::Nak => Progress::Nak,
            status => self.fail(status),
        }
    }

    fn do_data_out(&mut self, bus: &UsbBus, address: DeviceAddress, mps: usize) -> Progress {
        let remaining = self.data.len().saturating_sub(self.sent);
        if remaining == 0 {
            self.stage = Stage::StatusIn;
            return Progress::Moved;
        }
        let n = mps.min(remaining);
        let completion = bus.write(address, self.endpoint, &self.data[self.sent..self.sent + n]);
        match completion.status {
            Status::Ack => {
                self.sent = self.sent.saturating_add((completion.len as usize).min(n));
                if self.sent >= self.data.len() {
                    self.stage = Stage::StatusIn;
                }
                Progress::Moved
            }
            Status::Nak => Progress::Nak,
            status => self.fail(status),
        }
    }

    /// The status stage's outcome, which is the transfer's.
    fn do_status(&mut self, completion: Completion) -> Progress {
        match completion.status {
            Status::Ack => {
                self.stage = Stage::Finished;
                Progress::Done
            }
            Status::Nak => Progress::Nak,
            status => self.fail(status),
        }
    }

    fn fail(&mut self, status: Status) -> Progress {
        self.stage = Stage::Finished;
        self.failure = Some(status);
        Progress::Failed(status)
    }
}

/// The setup packet for `GET_DESCRIPTOR` (USB 2.0 §9.4.3).
///
/// Here rather than in a caller because every host-side caller needs it and
/// getting `wValue`'s halves the wrong way round is the classic mistake: the
/// **type** is the high byte and the **index** the low one.
#[must_use]
pub fn get_descriptor(kind: u8, index: u8, length: u16) -> SetupPacket {
    SetupPacket {
        request_type: Direction::BIT,
        request: super::request::GET_DESCRIPTOR,
        value: (u16::from(kind) << 8) | u16::from(index),
        index: 0,
        length,
    }
}

/// The setup packet for `SET_ADDRESS` (USB 2.0 §9.4.6).
#[must_use]
pub fn set_address(address: DeviceAddress) -> SetupPacket {
    SetupPacket {
        request_type: 0,
        request: super::request::SET_ADDRESS,
        value: u16::from(address.0),
        index: 0,
        length: 0,
    }
}

/// The setup packet for `SET_CONFIGURATION` (USB 2.0 §9.4.7).
#[must_use]
pub fn set_configuration(value: u8) -> SetupPacket {
    SetupPacket {
        request_type: 0,
        request: super::request::SET_CONFIGURATION,
        value: u16::from(value),
        index: 0,
        length: 0,
    }
}
