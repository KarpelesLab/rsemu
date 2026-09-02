//! A USB 2.0 hub: the device that makes the bus a *tree*.
//!
//! # What a hub actually is, and what it is not
//!
//! A hub is **not a router**. The address on the wire is seven flat bits and a
//! hub never looks at them: downstream of the host it is a repeater, and
//! upstream it forwards whatever a device answered (USB 2.0 §11.1.2.1). What
//! makes a device behind a hub reachable is not the hub deciding anything — it
//! is the **host** having reset and enabled the port the device is on, through
//! nine class requests it addresses to the hub like it would address any other
//! device.
//!
//! So this file is a device model of ordinary shape: [`Descriptors`], a
//! [`Function`] answering §11.24.2's class requests, and an interrupt IN
//! endpoint. What it *owns* that no other device model does is a second
//! [`UsbBus`] — its downstream ports — and the port state machine of §11.5 that
//! decides which of them a transaction may reach. The **routing** that finds a
//! device down there is [`crate::bus::usb::UsbBus::find`]'s, because the
//! topology belongs to the host and the host's view of it is the fabric.
//!
//! ```text
//!   controller ──► UsbBus "usb0" ──port 0──► usb.hub ──► UsbBus "usb1" ──port 0──► usb.storage
//!                    (root ports)              │            (downstream ports)
//!                                              └── nine class requests: power a port,
//!                                                  read its status, reset it, enable it
//! ```
//!
//! # Why it implements `UsbDevice` directly instead of being a `Peripheral`
//!
//! [`Peripheral`](crate::bus::usb::Peripheral) is [`Endpoint0`] wrapped around a
//! [`Function`], and [`Endpoint0`] deliberately **holds its lock across the call
//! into the function** — serving one control request is a single step. That is
//! fine for every other device here, whose class half only touches its own
//! state, and it is exactly wrong for a hub: `SetPortFeature(PORT_RESET)` has to
//! call [`UsbBus::reset_port`](crate::bus::usb::UsbBus::reset_port), which calls
//! `bus_reset` on the device behind the port, which takes *that* device's
//! `EP0_RANK` lock while this hub's is held. Same rank, so the ladder in
//! [`crate::bus::usb`] catches it — and it is right to, because it is the
//! re-entrancy contract of `CLAUDE.md` in its plainest form: *mutate your own
//! state in a short critical section, release it, then make any outward call —
//! or push the action onto a deferred queue.*
//!
//! A hub does the second. A port request records what it wants in the hub's own
//! state and returns; `HubDevice::flush` performs the outward half —
//! the port reset, the enable, the connection sample — around each transaction,
//! with no pipe lock held. That is why this type is a [`UsbDevice`] rather than
//! a `Peripheral`: the wrapper is where the flush goes. It is the same argument
//! [`crate::dev::usb::dwc2::device`] makes from the other direction, and it
//! needed no new lock rank, which is the claim worth making about it.
//!
//! # What the host sees, in the order it does it
//!
//! 1. `SET_ADDRESS`, `SET_CONFIGURATION` — this is an ordinary device first.
//! 2. `GetHubDescriptor` (§11.24.2.5) — how many ports, and what they can do.
//! 3. `SetPortFeature(PORT_POWER)` on each. **Until then a port reports no
//!    connection at all** (§11.5.1.1, the *Powered-off* state), and this model
//!    obeys that rather than defaulting the power on — the same decision the
//!    EHCI makes about `CONFIGFLAG`.
//! 4. `GetPortStatus` (§11.24.2.7) — now the connection is there, with
//!    `C_PORT_CONNECTION` set.
//! 5. `ClearPortFeature(C_PORT_CONNECTION)`, then
//!    `SetPortFeature(PORT_RESET)`.
//! 6. `GetPortStatus` again: `PORT_ENABLE` set, `C_PORT_RESET` set, and the
//!    speed bits saying what was found.
//! 7. From here the device behind the port answers
//!    [`DeviceAddress::DEFAULT`](crate::bus::usb::DeviceAddress::DEFAULT) and
//!    the host enumerates it exactly as if it were on a root port.
//!
//! # The transaction translator: its registers, not its data path
//!
//! A high-speed hub is *required* to have a transaction translator, so its
//! `bDeviceProtocol` is 1 (single TT) and it must answer `ClearTTBuffer`,
//! `ResetTT` and `StopTT` — which it does, as the no-ops they honestly are for a
//! TT that has never buffered anything.
//!
//! **What does not exist is the split-transaction data path**: `SPLIT` tokens, a
//! queue head's µFrame C-mask and the `siTD` (USB 2.0 §11.14, EHCI 1.0 §4.12).
//! So a full- or low-speed device behind a high-speed hub **does not enable**.
//! The port reset completes, `C_PORT_RESET` is set, the speed bits report what
//! is actually down there — and `PORT_ENABLE` stays clear, so the device is
//! unreachable rather than silently working over a path that does not exist.
//! That is the same refusal rsemu's EHCI already makes with `PORTSC.Port Owner`
//! for a full-speed device on a root port, and the same one a dwc2 makes by
//! leaving `HPRT.PENA` clear: *these pins cannot carry that device*, said at the
//! port instead of guessed at later.
//!
//! The mirror image holds and is why `speed` is a property: a **full-speed** hub
//! needs no TT at all, so `speed = "full"` with full- and low-speed devices
//! behind it is a completely modelled configuration — it is a high-speed hub
//! carrying a *slow* device that needs the translator. A hub may not be low
//! speed (§11.1: hubs are full- or high-speed), and this one refuses to be.
//!
//! # Time
//!
//! A hub in this tree has **no clock domain**, so a port reset completes before
//! the next transaction rather than in the 10 ms of §11.5.1.5, and
//! `bPwrOn2PwrGood` is a number in a descriptor rather than a delay. A driver
//! that waits and then polls sees exactly what it expects; a driver that polls
//! immediately sees the reset already finished, which is legal and is what
//! makes this model deterministic without owning a scheduler event. It is the
//! same simplification the dwc2's frame budget makes, in the same direction,
//! and it is written down here rather than discovered.
//!
//! # Sources
//!
//! **USB 2.0 §11** throughout, and nothing else: §11.1 (hub architecture and the
//! speeds a hub may be), §11.5 (the downstream port state machine), §11.12.4
//! (the status change endpoint's bitmap), §11.23 (the hub's descriptors),
//! §11.24.2 with tables 11-16, 11-17, 11-19 through 11-22 (the class requests,
//! the feature selectors and the status fields). Free from usb.org, which
//! `docs/buses/usb.md` notes leaves no excuse for working from anything else.
//! No emulator source was consulted (`ROADMAP.md` §1).

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::usb::{
    Completion, ConfigurationDescriptor, Descriptors, DeviceAddress, DeviceDescriptor, Direction,
    Endpoint0, EndpointDescriptor, Function, InterfaceDescriptor, MAX_PORTS, Recipient,
    RequestKind, SetupPacket, Speed, Status, TransferType, UsbBus, UsbDevice, buses, request,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.hub";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// `bDeviceClass` and `bInterfaceClass` for a hub (USB 2.0 §11.23.1).
const CLASS_HUB: u8 = 9;

/// `bDeviceProtocol` 1: a high-speed hub with a **single** transaction
/// translator (§11.23.1). A full-speed hub uses zero, having no TT to describe.
const PROTOCOL_SINGLE_TT: u8 = 1;

/// The hub class descriptor type (§11.23.2.1, table 11-13).
const DESC_HUB: u8 = 0x29;

/// The status change endpoint's number (§11.12.4). One IN endpoint, and a hub
/// has exactly one.
pub const ENDPOINT: u8 = 1;

/// `bInterval` at high speed, where the field is an exponent of microframes
/// (USB 2.0 §9.6.6). `12` is `2^11` microframes — 256 ms, which is the rate
/// §11.23.1 asks a high-speed hub to be polled at.
const INTERVAL_HIGH: u8 = 12;

/// `bInterval` at full speed, where the field counts frames. 255 ms, which is
/// the largest a frame-counted interval can be and what §11.23.1 specifies.
const INTERVAL_FULL: u8 = 0xff;

/// `bPwrOn2PwrGood`, in 2 ms units: 100 ms.
///
/// A descriptor field rather than a delay — see the module docs on time.
const POWER_ON_TO_GOOD: u8 = 50;

/// `bHubContrCurrent`, in mA: what the hub controller itself draws.
const CONTROLLER_CURRENT: u8 = 100;

/// The hub class requests of USB 2.0 §11.24.2, table 11-16.
///
/// The first four share their `bRequest` with the standard requests of §9.4 and
/// are told apart by `bmRequestType` naming the **class** — which is why
/// [`Endpoint0`] routes them here rather than answering them itself — and the
/// recipient, `Device` for the hub and `Other` for one of its ports.
pub mod class_request {
    /// §11.24.2.9, §11.24.2.13. `bRequest` 3.
    pub const SET_FEATURE: u8 = super::request::SET_FEATURE;
    /// §11.24.2.1, §11.24.2.2. `bRequest` 1.
    pub const CLEAR_FEATURE: u8 = super::request::CLEAR_FEATURE;
    /// §11.24.2.5. `bRequest` 6.
    pub const GET_DESCRIPTOR: u8 = super::request::GET_DESCRIPTOR;
    /// §11.24.2.6, §11.24.2.7. `bRequest` 0.
    pub const GET_STATUS: u8 = super::request::GET_STATUS;
    /// §11.24.2.3. Flushes one endpoint's buffer in the transaction
    /// translator.
    pub const CLEAR_TT_BUFFER: u8 = 8;
    /// §11.24.2.8.
    pub const RESET_TT: u8 = 9;
    /// §11.24.2.4. The TT's internal state, and its contents are
    /// implementation defined.
    pub const GET_TT_STATE: u8 = 10;
    /// §11.24.2.11.
    pub const STOP_TT: u8 = 11;
}

/// The hub and port feature selectors of USB 2.0 §11.24.2, table 11-17.
pub mod feature {
    /// The hub's `LOCAL_POWER` change bit.
    pub const C_HUB_LOCAL_POWER: u16 = 0;
    /// The hub's `OVER_CURRENT` change bit.
    pub const C_HUB_OVER_CURRENT: u16 = 1;

    /// Something is plugged into the port.
    pub const PORT_CONNECTION: u16 = 0;
    /// The port passes traffic. **Clearable, never settable** — a port is
    /// enabled by a reset succeeding (§11.24.2.13).
    pub const PORT_ENABLE: u16 = 1;
    /// The port is suspended.
    pub const PORT_SUSPEND: u16 = 2;
    /// The port drew too much current. Never modelled, and so never reported.
    pub const PORT_OVER_CURRENT: u16 = 3;
    /// Drive a reset down the port (§11.5.1.5).
    pub const PORT_RESET: u16 = 4;
    /// Port power. **Clear at reset**, and until it is set the port reports no
    /// connection at all (§11.5.1.1).
    pub const PORT_POWER: u16 = 8;
    /// The attached device signals at low speed.
    pub const PORT_LOW_SPEED: u16 = 9;
    /// Something was plugged in or unplugged.
    pub const C_PORT_CONNECTION: u16 = 16;
    /// The port was disabled by an error.
    pub const C_PORT_ENABLE: u16 = 17;
    /// A resume completed.
    pub const C_PORT_SUSPEND: u16 = 18;
    /// An over-current condition came or went.
    pub const C_PORT_OVER_CURRENT: u16 = 19;
    /// A reset completed.
    pub const C_PORT_RESET: u16 = 20;
    /// Electrical test modes. Accepted and inert: a modelled bus has no eye
    /// diagram, exactly as the device framework's `TEST_MODE` is.
    pub const PORT_TEST: u16 = 21;
    /// The port's LED, whose colour is in `wIndex`'s high byte.
    pub const PORT_INDICATOR: u16 = 22;
}

/// The bits of `wPortStatus` (USB 2.0 §11.24.2.7.1, table 11-21).
pub mod status {
    /// Something is attached.
    pub const CONNECTION: u16 = 1 << 0;
    /// The port passes traffic and a device on it can be addressed.
    pub const ENABLE: u16 = 1 << 1;
    /// The port is suspended.
    pub const SUSPEND: u16 = 1 << 2;
    /// An over-current condition. Never modelled, so never set.
    pub const OVER_CURRENT: u16 = 1 << 3;
    /// A reset is in progress. Never observed here, because a reset completes
    /// before the next transaction — see the module docs on time.
    pub const RESET: u16 = 1 << 4;
    /// The port is powered.
    pub const POWER: u16 = 1 << 8;
    /// The attached device signals at low speed.
    pub const LOW_SPEED: u16 = 1 << 9;
    /// The attached device signals at high speed. Neither bit set is full
    /// speed, which is why full speed has no constant here.
    pub const HIGH_SPEED: u16 = 1 << 10;
    /// The port is in a test mode.
    pub const TEST: u16 = 1 << 11;
    /// The port's indicator is under software control.
    pub const INDICATOR: u16 = 1 << 12;
}

/// The bits of `wPortChange` (USB 2.0 §11.24.2.7.2, table 11-22).
pub mod change {
    /// Something was plugged in or unplugged (§11.24.2.7.2.1).
    pub const CONNECTION: u16 = 1 << 0;
    /// The port was disabled by an error. Nothing here disables a port that
    /// way, so this is defined and never set.
    pub const ENABLE: u16 = 1 << 1;
    /// A resume completed (§11.24.2.7.2.3).
    pub const SUSPEND: u16 = 1 << 2;
    /// An over-current condition came or went. Never modelled.
    pub const OVER_CURRENT: u16 = 1 << 3;
    /// A reset completed (§11.24.2.7.2.5).
    pub const RESET: u16 = 1 << 4;
}

/// One downstream port, as §11.5's state machine leaves it.
///
/// Every field here is guest-visible through `GetPortStatus`, so every field is
/// snapshotted. `connected` and `speed` are a **mirror** of the downstream
/// [`UsbBus`] rather than a second source of truth: they are refreshed in
/// [`HubDevice::flush`], because sampling them is a call into the fabric and a
/// class request is answered with the control pipe's lock held.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Port {
    /// `PORT_POWER`. Clear at reset — §11.5.1.1's *Powered-off* state.
    powered: bool,
    /// `PORT_ENABLE`: whether a transaction may reach whatever is down there.
    /// Pushed to the downstream bus by [`HubDevice::flush`], which is what
    /// makes [`crate::bus::usb::UsbBus::find`] able to see it.
    enabled: bool,
    suspended: bool,
    test: bool,
    /// The `PORT_INDICATOR` colour selector, stored and reported.
    indicator: u8,
    /// Whether something is plugged in, as of the last flush.
    connected: bool,
    /// How fast it signals, meaningful only while `connected`.
    speed: Speed,
    /// `wPortChange`.
    change: u16,
}

/// Everything the hub remembers.
#[derive(Debug, Clone, Default)]
struct HubState {
    ports: Vec<Port>,
    /// One bit per port whose reset the host has asked for and which the next
    /// [`HubDevice::flush`] will perform. **The deferred queue** the module
    /// docs argue for: a reset is an outward call and a class request is
    /// answered under the control pipe's lock.
    pending_reset: u32,
    /// `wHubChange` (§11.24.2.6, table 11-20).
    change: u16,
}

/// The hub's class-specific half: descriptors, §11.24.2's requests, and the
/// status change endpoint.
struct HubFunction {
    descriptors: Descriptors,
    speed: Speed,
    ports: u8,
    /// How many bytes the status change endpoint's bitmap takes (§11.12.4).
    change_bytes: usize,
    state: Mutex<HubState>,
}

impl fmt::Debug for HubFunction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("HubFunction");
        s.field("speed", &self.speed);
        s.field("ports", &self.ports);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// How many bytes a bitmap covering the hub and `ports` ports takes
/// (USB 2.0 §11.12.4: one bit for the hub, then one per port).
const fn bitmap_bytes(ports: u8) -> usize {
    (ports as usize + 1).div_ceil(8)
}

impl HubFunction {
    fn new(ports: u8, speed: Speed, vendor: u16, product: u16) -> HubFunction {
        let change_bytes = bitmap_bytes(ports);
        let high = speed == Speed::High;

        let device = DeviceDescriptor {
            usb: 0x0200,
            // On the *device* for a hub, not on the interface: §11.23.1 is
            // explicit, and a host uses it to tell a hub from anything else
            // before it reads a configuration.
            class: CLASS_HUB,
            subclass: 0,
            // A high-speed hub must have a transaction translator, so it must
            // say which kind it has. A full-speed hub has none to describe.
            protocol: if high { PROTOCOL_SINGLE_TT } else { 0 },
            max_packet0: speed.max_control_packet() as u8,
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
            class: CLASS_HUB,
            subclass: 0,
            // Zero for both the full-speed hub and the single-TT high-speed
            // one; only a multi-TT hub uses the alternate settings that give
            // this field a value (§11.23.1).
            protocol: 0,
            name: 0,
        };
        let endpoint = EndpointDescriptor {
            address: ENDPOINT | Direction::BIT,
            attributes: TransferType::Interrupt.attribute_bits(),
            max_packet: change_bytes as u16,
            interval: if high { INTERVAL_HIGH } else { INTERVAL_FULL },
        };

        let mut body = Vec::new();
        body.extend_from_slice(&interface.encode());
        body.extend_from_slice(&endpoint.encode());

        let mut descriptors = Descriptors::new().with_device(&device);
        descriptors.add_configuration(
            &ConfigurationDescriptor {
                interfaces: 1,
                value: 1,
                name: 0,
                // A hub that powers its ports from the bus would have a budget
                // to account for; this one says it has its own supply, which is
                // what makes `bMaxPower` small and honest.
                attributes: ConfigurationDescriptor::SELF_POWERED,
                max_power: 0,
            },
            &body,
        );
        if high {
            descriptors.set_qualifier(&device, 0);
        }

        HubFunction {
            descriptors,
            speed,
            ports,
            change_bytes,
            state: Mutex::with_rank(
                LockRank::DEVICE,
                HubState {
                    ports: alloc::vec![Port::default(); usize::from(ports)],
                    pending_reset: 0,
                    change: 0,
                },
            ),
        }
    }

    /// The hub descriptor (USB 2.0 §11.23.2.1, table 11-13).
    ///
    /// Variable length: seven fixed bytes, then a `DeviceRemovable` bitmap and
    /// a `PortPwrCtrlMask` of [`bitmap_bytes`] each.
    fn hub_descriptor(&self) -> Vec<u8> {
        let bytes = bitmap_bytes(self.ports);
        let mut out = Vec::with_capacity(7 + bytes * 2);
        out.push((7 + bytes * 2) as u8);
        out.push(DESC_HUB);
        out.push(self.ports);
        // `wHubCharacteristics`: bits 1:0 per-port power switching (01b), bit 2
        // not a compound device, bits 4:3 **no over-current protection** (10b)
        // because none is modelled and claiming per-port protection that never
        // reports would be the lie, bits 6:5 a TT think time of 8 full-speed
        // bit times (00b), bit 7 no port indicators.
        let characteristics: u16 = 0b01 | (0b10 << 3);
        out.extend_from_slice(&characteristics.to_le_bytes());
        out.push(POWER_ON_TO_GOOD);
        out.push(CONTROLLER_CURRENT);
        // `DeviceRemovable`: bit 0 is reserved and bit n is port n. All zero —
        // every port of this hub is a socket, not a soldered-down device.
        out.extend(core::iter::repeat_n(0u8, bytes));
        // `PortPwrCtrlMask`: all ones, which §11.23.2.1 requires for
        // compatibility with USB 1.0 hub software.
        out.extend(core::iter::repeat_n(0xffu8, bytes));
        out
    }

    /// `wHubStatus` and `wHubChange` (§11.24.2.6, tables 11-19 and 11-20).
    ///
    /// Local power is always good and over-current never happens: neither is
    /// modelled, and both read as the healthy value rather than as a bit that
    /// might one day mean something.
    fn hub_status(&self) -> Vec<u8> {
        let change = self.state.lock().change;
        let mut out = Vec::with_capacity(4);
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&change.to_le_bytes());
        out
    }

    /// `wPortStatus` and `wPortChange` for a **zero-based** port index
    /// (§11.24.2.7, tables 11-21 and 11-22).
    fn port_status(&self, index: usize) -> Option<Vec<u8>> {
        let state = self.state.lock();
        let port = state.ports.get(index)?;
        let mut value = 0u16;
        if port.powered {
            value |= status::POWER;
            // §11.5.1.1: an unpowered port reports no connection, so the
            // connection bit is *inside* the power test rather than beside it.
            if port.connected {
                value |= status::CONNECTION;
                match port.speed {
                    Speed::Low => value |= status::LOW_SPEED,
                    Speed::High => value |= status::HIGH_SPEED,
                    Speed::Full => {}
                }
            }
        }
        if port.enabled {
            value |= status::ENABLE;
        }
        if port.suspended {
            value |= status::SUSPEND;
        }
        if port.test {
            value |= status::TEST;
        }
        if port.indicator != 0 {
            value |= status::INDICATOR;
        }
        let mut out = Vec::with_capacity(4);
        out.extend_from_slice(&value.to_le_bytes());
        out.extend_from_slice(&port.change.to_le_bytes());
        Some(out)
    }

    /// The status change bitmap (§11.12.4): bit 0 the hub, bit `n` port `n`.
    fn change_bitmap(&self) -> Vec<u8> {
        let state = self.state.lock();
        let mut out = alloc::vec![0u8; self.change_bytes];
        if state.change != 0 {
            out[0] |= 1;
        }
        for (index, port) in state.ports.iter().enumerate() {
            if port.change != 0 {
                let bit = index + 1;
                out[bit / 8] |= 1 << (bit % 8);
            }
        }
        out
    }

    /// `SetPortFeature` (§11.24.2.13). `index` is zero-based.
    fn set_port_feature(&self, index: usize, selector: u16, wvalue_high: u8) -> bool {
        let mut state = self.state.lock();
        if index >= state.ports.len() {
            return false;
        }
        match selector {
            feature::PORT_POWER => {
                state.ports[index].powered = true;
                true
            }
            feature::PORT_RESET => {
                // Deferred: driving the reset is a call into whatever is on the
                // port, and this runs with the control pipe's lock held. See
                // the module docs. §11.5.1.1: a port with no power does nothing
                // at all, and the request is still accepted.
                if state.ports[index].powered {
                    state.pending_reset |= 1u32 << index;
                }
                true
            }
            feature::PORT_SUSPEND => {
                if state.ports[index].enabled {
                    state.ports[index].suspended = true;
                }
                true
            }
            feature::PORT_TEST => {
                state.ports[index].test = true;
                true
            }
            feature::PORT_INDICATOR => {
                state.ports[index].indicator = wvalue_high;
                true
            }
            // §11.24.2.13 lists exactly the five above. `PORT_ENABLE` is
            // deliberately not one of them: a port is enabled by a *reset*
            // succeeding, never by being asked.
            _ => false,
        }
    }

    /// `ClearPortFeature` (§11.24.2.2). `index` is zero-based.
    fn clear_port_feature(&self, index: usize, selector: u16) -> bool {
        let mut state = self.state.lock();
        let Some(port) = state.ports.get_mut(index) else {
            return false;
        };
        match selector {
            feature::PORT_ENABLE => {
                port.enabled = false;
                true
            }
            feature::PORT_POWER => {
                // Removing power takes the port back to *Powered-off*, which is
                // below every other state (§11.5.1.1).
                *port = Port::default();
                true
            }
            feature::PORT_SUSPEND => {
                if port.suspended {
                    port.suspended = false;
                    // §11.24.2.2: the resume completing is what sets the
                    // change bit, and here it completes immediately.
                    port.change |= change::SUSPEND;
                }
                true
            }
            feature::PORT_INDICATOR => {
                port.indicator = 0;
                true
            }
            feature::C_PORT_CONNECTION => {
                port.change &= !change::CONNECTION;
                true
            }
            feature::C_PORT_ENABLE => {
                port.change &= !change::ENABLE;
                true
            }
            feature::C_PORT_SUSPEND => {
                port.change &= !change::SUSPEND;
                true
            }
            feature::C_PORT_OVER_CURRENT => {
                port.change &= !change::OVER_CURRENT;
                true
            }
            feature::C_PORT_RESET => {
                port.change &= !change::RESET;
                true
            }
            _ => false,
        }
    }
}

impl Function for HubFunction {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        self.speed
    }

    fn reset(&self) {
        let mut state = self.state.lock();
        for port in &mut state.ports {
            *port = Port::default();
        }
        state.pending_reset = 0;
        state.change = 0;
    }

    fn control_in(&self, setup: SetupPacket) -> Option<Vec<u8>> {
        if setup.kind() != RequestKind::Class {
            return None;
        }
        match setup.request {
            // §11.24.2.5. `wValue`'s high byte is the descriptor type; the only
            // one a USB 2.0 hub has is `0x29`. A SuperSpeed hub descriptor
            // (`0x2a`) is a different device and is refused rather than
            // approximated.
            class_request::GET_DESCRIPTOR => {
                let (kind, index) = setup.descriptor();
                (kind == DESC_HUB && index == 0).then(|| self.hub_descriptor())
            }
            class_request::GET_STATUS => match setup.recipient() {
                Recipient::Device => Some(self.hub_status()),
                // §11.24.2.7: the recipient is `Other` and `wIndex` is a
                // **one-based** port number. Zero is not a port.
                Recipient::Other => {
                    let port = (setup.index & 0xff) as usize;
                    port.checked_sub(1).and_then(|i| self.port_status(i))
                }
                _ => None,
            },
            // §11.24.2.4: the returned bytes are implementation defined, and
            // inventing them would be inventing a TT's internals. Stalled, which
            // is what a hub that does not implement an optional request does.
            class_request::GET_TT_STATE => None,
            _ => None,
        }
    }

    fn control_out(&self, setup: SetupPacket, data: &[u8]) -> bool {
        let _ = data;
        if setup.kind() != RequestKind::Class {
            return false;
        }
        let recipient = setup.recipient();
        match (setup.request, recipient) {
            (class_request::SET_FEATURE, Recipient::Device) => {
                // §11.24.2.12: a hub has no settable features — the two hub
                // selectors are *change* bits, which `SetHubFeature` may not
                // set.
                false
            }
            (class_request::CLEAR_FEATURE, Recipient::Device) => {
                let mut state = self.state.lock();
                match setup.value {
                    feature::C_HUB_LOCAL_POWER => {
                        state.change &= !1;
                        true
                    }
                    feature::C_HUB_OVER_CURRENT => {
                        state.change &= !2;
                        true
                    }
                    _ => false,
                }
            }
            (class_request::SET_FEATURE, Recipient::Other) => {
                let port = (setup.index & 0xff) as usize;
                let Some(index) = port.checked_sub(1) else {
                    return false;
                };
                self.set_port_feature(index, setup.value, (setup.index >> 8) as u8)
            }
            (class_request::CLEAR_FEATURE, Recipient::Other) => {
                let port = (setup.index & 0xff) as usize;
                let Some(index) = port.checked_sub(1) else {
                    return false;
                };
                self.clear_port_feature(index, setup.value)
            }
            // The transaction translator's own requests (§11.24.2.3, §11.24.2.8,
            // §11.24.2.11). A hub with a TT must answer them; a TT that has
            // never carried a full- or low-speed transaction — because this one
            // has no split-transaction data path — has an empty buffer, so the
            // correct answer to "flush it" is a successful no-op rather than a
            // stall. A full-speed hub has no TT at all and refuses all three.
            (
                class_request::CLEAR_TT_BUFFER | class_request::RESET_TT | class_request::STOP_TT,
                _,
            ) => self.speed == Speed::High,
            _ => false,
        }
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT {
            return Completion::stall();
        }
        let bitmap = self.change_bitmap();
        if bitmap.iter().all(|b| *b == 0) {
            // §11.12.4: nothing has changed, so the hub `NAK`s. It does *not*
            // send a zero-length packet, which would retire the host's transfer
            // and stop the polling.
            return Completion::nak();
        }
        let n = bitmap.len().min(dst.len());
        dst[..n].copy_from_slice(&bitmap[..n]);
        Completion::ack(n as u64)
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != ENDPOINT {
            return Completion::stall();
        }
        // Identical to `endpoint_in`, and that is not laziness: §11.12.4's
        // bitmap is *not* consumed by the `IN` that carries it. The bits are
        // the change bits, and only `ClearPortFeature(C_…)` clears them — so
        // reading the endpoint has no side effect to suppress, and the debug
        // path is the same code path.
        self.endpoint_in(endpoint, dst)
    }
}

/// The hub as the fabric sees it: an [`Endpoint0`] over a `HubFunction`, a
/// downstream [`UsbBus`], and the flush that separates the two.
pub struct HubDevice {
    ep0: Endpoint0,
    function: Arc<HubFunction>,
    /// The bus this hub's ports form. Handed to
    /// [`crate::bus::usb::UsbBus::find`] through [`UsbDevice::downstream`],
    /// which is the whole of what routing needs from a hub.
    downstream: Arc<UsbBus>,
}

impl fmt::Debug for HubDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HubDevice")
            .field("function", &self.function)
            .field("downstream", &self.downstream)
            .finish()
    }
}

impl HubDevice {
    fn new(function: Arc<HubFunction>, downstream: Arc<UsbBus>) -> HubDevice {
        HubDevice {
            ep0: Endpoint0::new(Arc::clone(&function) as Arc<dyn Function>),
            function,
            downstream,
        }
    }

    /// Do the outward half of everything the last request asked for, and
    /// resample the ports.
    ///
    /// **Called with no lock of this device's held**, from the [`UsbDevice`]
    /// wrapper around every transaction. Three steps, in this order, because
    /// each depends on the last:
    ///
    /// 1. **Deferred resets.** [`crate::bus::usb::UsbBus::reset_port`] calls
    ///    `bus_reset` on the device behind the port, which is an outward call
    ///    into another device at the same lock rank — the thing a class request
    ///    may not do. The reset completes here.
    /// 2. **Sample.** Ask the downstream fabric what is plugged in and how fast
    ///    it signals, with nothing locked.
    /// 3. **Reconcile**, under this hub's own lock, and then push the resulting
    ///    enable bits into the fabric — which is what makes a device down there
    ///    findable at all.
    ///
    /// The lock is taken and released between steps rather than held across
    /// them, so a `DEVICE`-rank lock is never held while a `FABRIC_RANK` one is
    /// acquired. That is the ladder, not a convention.
    fn flush(&self) {
        let ports = usize::from(self.function.ports);

        // 1. Deferred resets, outside every lock.
        let pending = {
            let mut state = self.function.state.lock();
            core::mem::take(&mut state.pending_reset)
        };
        for index in 0..ports {
            if pending & (1u32 << index) != 0 {
                self.downstream.reset_port(index as u8);
            }
        }

        // 2. Sample the fabric.
        let sample: Vec<(bool, Option<Speed>)> = (0..ports)
            .map(|index| {
                let port = index as u8;
                (self.downstream.connected(port), self.downstream.speed(port))
            })
            .collect();

        // 3. Reconcile, then publish.
        let enables: Vec<bool> = {
            let mut state = self.function.state.lock();
            let hub_speed = self.function.speed;
            for (index, port) in state.ports.iter_mut().enumerate() {
                let (connected, speed) = sample[index];
                // An unpowered port is in *Powered-off* and reports nothing
                // (§11.5.1.1), so what it mirrors is "no connection".
                let connected = connected && port.powered;
                if connected != port.connected {
                    port.connected = connected;
                    port.change |= change::CONNECTION;
                    if !connected {
                        // A disconnect takes the port out of *Enabled*
                        // (§11.5.1.4). The change bit that reports it is
                        // `C_PORT_CONNECTION`, not `C_PORT_ENABLE`, which is
                        // reserved for a port disabled by an error.
                        port.enabled = false;
                        port.suspended = false;
                    }
                }
                // Only while something is there: an empty port has no speed,
                // and recording one would put a value in the snapshot that
                // `GetPortStatus` can never report.
                if let Some(speed) = speed.filter(|_| connected) {
                    port.speed = speed;
                }
                if pending & (1u32 << index) != 0 {
                    // The reset finished. Whether it *enabled* the port is the
                    // one interesting decision a hub makes here: a port only
                    // enables for a device this hub can actually carry, which
                    // without a split-transaction data path means a device
                    // signalling at the hub's own speed. See the module docs.
                    port.enabled = connected && speed == Some(hub_speed);
                    port.suspended = false;
                    port.change |= change::RESET;
                }
            }
            state.ports.iter().map(|p| p.enabled).collect()
        };
        for (index, enabled) in enables.into_iter().enumerate() {
            self.downstream.set_enabled(index as u8, enabled);
        }
    }

    /// The control pipe, for `save`/`load`.
    #[must_use]
    pub fn endpoint0(&self) -> &Endpoint0 {
        &self.ep0
    }
}

impl UsbDevice for HubDevice {
    fn speed(&self) -> Speed {
        self.function.speed
    }

    fn address(&self) -> DeviceAddress {
        self.ep0.address()
    }

    fn bus_reset(&self) {
        // `Endpoint0::bus_reset` calls `Function::reset`, which takes every
        // port back to *Powered-off*; the flush is what pushes the resulting
        // "nothing is enabled" into the downstream fabric, so a device behind
        // an unpowered hub stops being reachable. Nothing recurses: removing
        // power does not reset the device below, it only unplugs it from the
        // routing walk.
        self.ep0.bus_reset();
        self.flush();
    }

    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status {
        if endpoint != 0 {
            return Status::Stall;
        }
        // Before: `GetPortStatus` is answered *inside* `Endpoint0::setup`, out
        // of a buffer built there, so the sample has to already have happened.
        self.flush();
        let status = self.ep0.setup(packet);
        // After: a `SetPortFeature` with no data stage was served inside that
        // call and has left its outward half in the deferred queue.
        self.flush();
        status
    }

    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint == 0 {
            return self.ep0.read(dst);
        }
        if self.ep0.is_halted(endpoint | Direction::BIT) {
            return Completion::stall();
        }
        // The status change endpoint reports what the last sample found, so the
        // sample belongs before it — this is the transaction a host uses to
        // *discover* that something was plugged in.
        self.flush();
        self.function.endpoint_in(endpoint, dst)
    }

    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        if endpoint == 0 {
            let completion = self.ep0.write(src);
            self.flush();
            return completion;
        }
        if self.ep0.is_halted(endpoint) {
            return Completion::stall();
        }
        self.function.endpoint_out(endpoint, src)
    }

    fn peek_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        // **No flush.** A debug read must not sample the fabric, set a change
        // bit, complete a deferred reset or enable a port
        // (`ROADMAP.md` §15). It answers out of the state that is already
        // there, which is exactly what a debugger should see: what the hub
        // would tell the host if the host asked right now.
        if endpoint == 0 {
            return self.ep0.peek(dst);
        }
        self.function.peek_in(endpoint, dst)
    }

    fn downstream(&self) -> Option<Arc<UsbBus>> {
        Some(Arc::clone(&self.downstream))
    }
}

/// A USB 2.0 hub.
#[derive(Debug)]
pub struct UsbHub {
    device: Arc<HubDevice>,
}

impl UsbHub {
    /// Validate `props` and build the hub.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this hub plugs *into*. Required.
    /// * `port` — which port of it. Defaults to 0.
    /// * `downstream` — the name of the [`UsbBus`] this hub's own ports form,
    ///   which is what a device behind the hub names as its `bus`. Required,
    ///   and must not be `bus`: a hub plugged into itself is a cycle a machine
    ///   description can express and hardware cannot.
    /// * `ports` — how many downstream ports, 1 to 15. Defaults to 4.
    /// * `speed` — `high` (the default) or `full`. **Not `low`**: USB 2.0 §11.1
    ///   has no low-speed hub. It decides which devices the hub's ports can
    ///   carry — see the module docs on the transaction translator.
    /// * `vendor`, `product` — what the device descriptor reports.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for a bad `speed`, a `downstream` equal to `bus`, or a
    /// downstream bus another object already sized too small — and whatever
    /// [`UsbBus::attach`] refuses.
    pub fn new(props: &Props) -> Result<UsbHub> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let port = r.or_range("port", 0u64, 0..=u64::from(u8::MAX))?;
        let downstream_name = r.require_str("downstream")?.to_string();
        let ports = r.or_range("ports", 4u64, 1..=MAX_PORTS as u64)?;
        let spelling = r.or_str("speed", Speed::High.name())?;
        let vendor = r.or_range("vendor", 0u64, 0..=u64::from(u16::MAX))?;
        let product = r.or_range("product", 0u64, 0..=u64::from(u16::MAX))?;
        r.finish()?;

        let speed = match Speed::from_name(spelling) {
            Some(Speed::Low) | None => {
                return Err(Error::Config {
                    at: String::from(CLASS_NAME),
                    message: alloc::format!(
                        "`speed` is `high` or `full`, not `{spelling}`: USB 2.0 §11.1 defines no \
                         low-speed hub"
                    ),
                });
            }
            Some(speed) => speed,
        };
        if downstream_name == bus_name {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "`downstream` is `{downstream_name}`, which is the bus this hub is plugged \
                     into; a hub whose ports are its own upstream bus is a cycle, not a topology"
                ),
            });
        }

        let downstream = buses::attach(props, &downstream_name, ports as u8)?;
        if downstream.port_count() < ports as u8 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "the USB bus `{downstream_name}` already has {} ports and this hub asked for \
                     {ports}; the first object to name a bus fixes its size",
                    downstream.port_count()
                ),
            });
        }

        let upstream = buses::attach(props, &bus_name, port as u8 + 1)?;
        let hub = UsbHub::with_bus(
            downstream,
            ports as u8,
            speed,
            vendor as u16,
            product as u16,
        );
        upstream.attach(port as u8, hub.device())?;
        Ok(hub)
    }

    /// A hub on a downstream bus the caller already holds, plugged into
    /// nothing.
    ///
    /// For a test, or an embedder that owns its own buses and attaches the hub
    /// itself with [`UsbHub::device`].
    #[must_use]
    pub fn with_bus(
        downstream: Arc<UsbBus>,
        ports: u8,
        speed: Speed,
        vendor: u16,
        product: u16,
    ) -> UsbHub {
        let function = Arc::new(HubFunction::new(ports, speed, vendor, product));
        UsbHub {
            device: Arc::new(HubDevice::new(function, downstream)),
        }
    }

    /// The hub as the fabric sees it, for
    /// [`UsbBus::attach`](crate::bus::usb::UsbBus::attach).
    #[must_use]
    pub fn device(&self) -> Arc<dyn UsbDevice> {
        Arc::clone(&self.device) as Arc<dyn UsbDevice>
    }

    /// The bus this hub's downstream ports form.
    #[must_use]
    pub fn downstream(&self) -> &Arc<UsbBus> {
        &self.device.downstream
    }

    /// How many downstream ports it has.
    #[must_use]
    pub fn ports(&self) -> u8 {
        self.device.function.ports
    }

    /// The address the host has given it, or zero before enumeration.
    #[must_use]
    pub fn address(&self) -> DeviceAddress {
        self.device.address()
    }

    /// Whether the host has powered port `index` (zero-based).
    #[must_use]
    pub fn port_powered(&self, index: u8) -> bool {
        let state = self.device.function.state.lock();
        state
            .ports
            .get(usize::from(index))
            .is_some_and(|p| p.powered)
    }

    /// Whether port `index` (zero-based) is enabled — which is to say, whether
    /// a transaction can reach whatever is on it.
    #[must_use]
    pub fn port_enabled(&self, index: u8) -> bool {
        let state = self.device.function.state.lock();
        state
            .ports
            .get(usize::from(index))
            .is_some_and(|p| p.enabled)
    }

    /// Refuse a topology in which walking *down* from this hub arrives back at
    /// this hub.
    ///
    /// # Why this is not in `new`
    ///
    /// `new` can only check what one object knows, and the only cycle one
    /// object can see is its own: `downstream == bus`, which it does refuse.
    /// Two hubs naming *each other's* bus passes that check twice, because
    /// neither exists when the other is built — and it is a perfectly ordinary
    /// pair of machine-file statements to write by accident when a bus is
    /// renamed.
    ///
    /// By `realize` the whole graph exists, which is exactly what two-phase
    /// construction is for. So this is where it is caught, and it is caught for
    /// two reasons rather than one:
    ///
    /// * **Routing would still work** — [`crate::bus::usb::UsbBus::find`] is
    ///   bounded — but it would silently do six tiers of pointless work on
    ///   every transaction on the bus.
    /// * **The machine would leak.** A [`UsbBus`] holds an `Arc` to each device
    ///   on it and a hub holds an `Arc` to the bus below it, so a cycle in the
    ///   topology is a cycle in the reference counts and nothing in it is ever
    ///   freed. A tree cannot do that; only this can. `fuzz/fuzz_targets/
    ///   usb_hub.rs` found it on its first run, by building the cycle on
    ///   purpose.
    ///
    /// The walk is bounded like every other walk over a structure someone else
    /// built. Running out of budget is not an error: it means the topology is
    /// larger than USB 2.0 §4.1.1 allows, which routing already handles by
    /// declining to look further.
    fn check_topology(&self) -> Result<()> {
        fn connected(bus: &UsbBus) -> Vec<Arc<dyn UsbDevice>> {
            (0..bus.port_count())
                .filter_map(|p| bus.device(p))
                .collect()
        }
        /// Two `Arc<dyn UsbDevice>` pointing at the same object.
        ///
        /// By data address rather than by `Arc::ptr_eq`, which compares the
        /// vtable pointer too and is meaningless for a trait object built
        /// twice from the same type.
        fn same(a: &Arc<dyn UsbDevice>, b: &Arc<dyn UsbDevice>) -> bool {
            core::ptr::addr_eq(Arc::as_ptr(a), Arc::as_ptr(b))
        }

        let me = Arc::clone(&self.device) as Arc<dyn UsbDevice>;
        let mut frontier = connected(&self.device.downstream);
        let mut budget = crate::bus::usb::MAX_DEVICES;
        for _ in 1..crate::bus::usb::MAX_TIERS {
            let mut next: Vec<Arc<dyn UsbDevice>> = Vec::new();
            for device in frontier {
                if budget == 0 {
                    return Ok(());
                }
                budget -= 1;
                if same(&device, &me) {
                    return Err(Error::Config {
                        at: String::from(CLASS_NAME),
                        message: String::from(
                            "this hub's downstream bus leads back to this hub: two hubs have been \
                             given each other's bus, which is a cycle rather than a topology. \
                             Nothing behind either of them can be reached, and the machine would \
                             never free them",
                        ),
                    });
                }
                if let Some(below) = device.downstream() {
                    next.extend(connected(&below));
                }
            }
            if next.is_empty() {
                return Ok(());
            }
            frontier = next;
        }
        Ok(())
    }

    fn save_state<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        self.device.ep0.save(w)?;
        let state = self.device.function.state.lock().clone();
        w.write_seq_len(state.ports.len() as u64)?;
        for port in &state.ports {
            let mut flags = 0u8;
            flags |= u8::from(port.powered);
            flags |= u8::from(port.enabled) << 1;
            flags |= u8::from(port.suspended) << 2;
            flags |= u8::from(port.test) << 3;
            flags |= u8::from(port.connected) << 4;
            w.write_u8(flags)?;
            w.write_u8(speed_code(port.speed))?;
            w.write_u8(port.indicator)?;
            w.write_u16(port.change)?;
        }
        w.write_u32(state.pending_reset)?;
        w.write_u16(state.change)
    }

    fn load_state<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        self.device.ep0.load(r)?;
        let count = r.read_seq_len(5)?;
        let expected = u64::from(self.device.function.ports);
        if count != expected {
            return Err(Error::State(alloc::format!(
                "usb.hub: a snapshot with {count} ports, not {expected}"
            )));
        }
        let mut ports = Vec::with_capacity(usize::from(self.device.function.ports));
        for _ in 0..count {
            let flags = r.read_u8()?;
            let speed = speed_from_code(r.read_u8()?);
            let indicator = r.read_u8()?;
            let change = r.read_u16()?;
            ports.push(Port {
                powered: flags & 1 != 0,
                enabled: flags & 2 != 0,
                suspended: flags & 4 != 0,
                test: flags & 8 != 0,
                indicator,
                connected: flags & 16 != 0,
                speed,
                change,
            });
        }
        let pending_reset = r.read_u32()?;
        let change = r.read_u16()?;
        let enables: Vec<bool> = {
            let mut state = self.device.function.state.lock();
            state.ports = ports;
            state.pending_reset = pending_reset;
            state.change = change;
            state.ports.iter().map(|p| p.enabled).collect()
        };
        // The fabric's enable bits are derived state and are never serialized
        // (`ROADMAP.md` §4.5) — they are re-derived from the hub's own port
        // state here, exactly as the EHCI re-derives them from `PORTSC`.
        for (index, enabled) in enables.into_iter().enumerate() {
            self.device.downstream.set_enabled(index as u8, enabled);
        }
        Ok(())
    }
}

/// How a [`Speed`] is written in a snapshot. Not the wire encoding of anything
/// — just a stable byte, so a future speed can be added without renumbering.
const fn speed_code(speed: Speed) -> u8 {
    match speed {
        Speed::Low => 0,
        Speed::Full => 1,
        Speed::High => 2,
    }
}

const fn speed_from_code(code: u8) -> Speed {
    match code {
        0 => Speed::Low,
        1 => Speed::Full,
        _ => Speed::High,
    }
}

impl Device for UsbHub {
    fn class(&self) -> &'static DeviceClass {
        &HUB_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward — the hub plugged itself into the named bus at
        // construction, which is the rendezvous table and not an observable
        // action, exactly as `usb.mouse` and `usb.storage` do — but this is
        // where the topology is *checked*, and that is two-phase construction
        // earning its keep rather than a place to hang a check.
        self.check_topology()
    }

    fn reset(&self, _kind: ResetKind) {
        self.device.bus_reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.save_state(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.load_state(r)
    }
}

impl Instance for UsbHub {}

/// The `usb.hub` device class.
pub static HUB_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a USB 2.0 hub: downstream ports with the state machine of §11.5, the nine class \
              requests of §11.24.2 and a status change endpoint, forming a second named bus that \
              devices behind it plug into",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the named USB bus this hub plugs into",
        },
        PropertySpec {
            name: "port",
            kind: ValueKind::Uint,
            required: false,
            summary: "which port of that bus (default 0)",
        },
        PropertySpec {
            name: "downstream",
            kind: ValueKind::Str,
            required: true,
            summary: "the name of the bus this hub's own ports form, which devices behind it name \
                      as their `bus`",
        },
        PropertySpec {
            name: "ports",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many downstream ports, 1 to 15 (default 4)",
        },
        PropertySpec {
            name: "speed",
            kind: ValueKind::Str,
            required: false,
            summary: "how fast it signals: `high` (default) or `full`; there is no low-speed hub",
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
    construct: |props| Ok(alloc::boxed::Box::new(UsbHub::new(props)?)),
};

/// Add [`HUB_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&HUB_CLASS)
}

/// Bind [`HUB_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(UsbHub::new(props)?)))
}

/// What the validator should know about `usb.hub`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("port", ValueKind::Uint).range(0, u64::from(u8::MAX)))
        .prop(PropSchema::new("downstream", ValueKind::Str).required())
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, MAX_PORTS as u64))
        .prop(PropSchema::new("speed", ValueKind::Str).values(&["high", "full"]))
        .prop(PropSchema::new("vendor", ValueKind::Uint).range(0, u64::from(u16::MAX)))
        .prop(PropSchema::new("product", ValueKind::Uint).range(0, u64::from(u16::MAX)))
}

// The status change endpoint's packet has to hold one bit for the hub and one
// per port, and a hub cannot have more ports than the fabric has (§11.12.4,
// and `bus::usb::MAX_PORTS`).
const _: () = {
    assert!(bitmap_bytes(MAX_PORTS as u8) == 2);
    assert!(bitmap_bytes(1) == 1);
    assert!(bitmap_bytes(7) == 1);
    assert!(bitmap_bytes(8) == 2);
};

#[cfg(test)]
mod tests;
