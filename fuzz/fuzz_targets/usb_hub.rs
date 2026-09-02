#![no_main]
//! The hub's untrusted surfaces: **a control request, and a topology**.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface. A hub has none — it
//! masters no bus, walks no guest structure and has no register block — so the
//! rule is read the way `usb_msd` reads it: *the input a guest chooses, aimed at
//! the code that parses it*. For a hub that input is the eight bytes of a class
//! request, and every one of `wValue`, `wIndex` and `wLength` is a number the
//! guest picked.
//!
//! But a hub has a **second** untrusted input that no other device model here
//! has, and it is the one this target really exists for:
//!
//! > **The topology is a walk, and a machine description can build a cycle.**
//! > Two hubs can name each other's downstream bus. Nothing detects that at
//! > *construction* — each hub's own `downstream` differs from its own `bus`, so
//! > the one check a hub can make locally passes — and every routing decision
//! > afterwards is a graph traversal over a graph with a loop in it.
//!
//! `UsbHub::realize` refuses such a topology, because by then the whole graph
//! exists. This fixture goes around that on purpose, building the cycle with
//! `UsbHub::with_bus` and no realize at all, because the point is to keep the
//! *bound* honest rather than to test the check: a bound that only holds because
//! a cycle is rejected elsewhere is a bound that one refactor removes.
//!
//! It is also the target that found the reason for that check. A `UsbBus` holds
//! an `Arc` to every device on it and a hub holds an `Arc` to the bus below it,
//! so a cycle in the topology is a cycle in the reference counts and the whole
//! loop leaks — which LeakSanitizer reported on this target's first run, on an
//! input that did nothing at all. Hence [`Fixture`]'s `Drop`.
//!
//! So the fixture is built *with* that cycle, deliberately, and the properties
//! checked after every step are:
//!
//! * **Every routing walk terminates.** `UsbBus::find` and
//!   `UsbBus::start_of_frame` are bounded by `MAX_TIERS` and `MAX_DEVICES`
//!   (USB 2.0 §4.1.1), and an unbounded one shows up here as a timeout.
//! * **A walk never returns the wrong device.** Whatever comes back from
//!   `find(a)` answers to `a` — the bound must not be implemented by giving up
//!   and returning something plausible.
//! * **A debug peek has no side effects.** Peeking the status change endpoint
//!   twice must give the same answer and must not clear a change bit, complete
//!   a deferred port reset, or enable a port (`ROADMAP.md` §15, invariant 5).
//! * **The status change bitmap is never longer than the endpoint's
//!   `wMaxPacketSize`** — one bit for the hub and one per port, rounded up
//!   (§11.12.4).
//! * **The snapshot loader is a parser on untrusted bytes**, so a tail of the
//!   input goes into `Device::load`, which must reject it or accept it and never
//!   panic — and the hub must still route afterwards.
//!
//! Nothing here can make the hub allocate: its port table is fixed at
//! construction and every buffer it produces is a descriptor of known length, so
//! the failure this target watches for is a hang rather than an
//! out-of-memory — which is exactly the opposite of `usb_msd`'s, and is why it
//! is a separate target.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 <8 bytes>    a raw SETUP to the hub, then one IN and one OUT
//!   0x01 nn           an IN of nn bytes on the status change endpoint,
//!                       peeked twice first
//!   0x02 pp ss        plug a device of speed ss into downstream port pp
//!   0x03 pp           unplug downstream port pp
//!   0x04 aa           route to address aa, from the root and from below
//!   0x05              a start-of-frame broadcast over the whole tree
//!   0x06              reset the root port, which resets the hub
//!   0x07              save, then load what was saved: a round trip
//!   0x08 ...          load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather than
//! mostly-rejected.

use libfuzzer_sys::fuzz_target;

use std::sync::Arc;

use rsemu::bus::usb::{DeviceAddress, SetupPacket, Speed, Status, UsbBus, request};
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::usb::hid::HidMouse;
use rsemu::dev::usb::hub::{ENDPOINT, UsbHub};

/// How many downstream ports each hub has. Small, so the fuzzer reaches the
/// interesting port numbers instead of spending its budget on indices.
const PORTS: u8 = 3;

/// The address the harness gives the near hub.
const HUB_ADDRESS: DeviceAddress = DeviceAddress(1);

/// The largest packet the fuzzer may hand an endpoint. Bigger than
/// `wMaxPacketSize` would be, on purpose.
const MAX_PACKET: usize = 96;

/// The hub's status change endpoint carries one bit for the hub and one per
/// port, rounded up to a byte (§11.12.4).
const CHANGE_BYTES: usize = (PORTS as usize + 1).div_ceil(8);

struct Fixture {
    root: Arc<UsbBus>,
    /// The hub on the root port: the one the fuzzer talks to.
    near: UsbHub,
    /// Its downstream bus, where devices are plugged and unplugged.
    below: Arc<UsbBus>,
    /// A second hub, behind the first, **whose downstream bus is the root**.
    /// That is the cycle, and it is the reason this target exists.
    _far: UsbHub,
}

fn build() -> Fixture {
    let root = Arc::new(UsbBus::new(1));
    let below = Arc::new(UsbBus::new(PORTS));
    let near = UsbHub::with_bus(Arc::clone(&below), PORTS, Speed::High, 0x1d6b, 0x0002);
    root.attach(0, near.device()).expect("an empty port");
    root.set_enabled(0, true);

    // Root ─► near ─► below ─► far ─► root ─► …
    //
    // Neither hub can see this: `near`'s downstream is not `near`'s upstream,
    // and neither is `far`'s. Only the bound on the walk stops it.
    let far = UsbHub::with_bus(Arc::clone(&root), PORTS, Speed::High, 0x1d6b, 0x0002);
    below
        .attach(PORTS - 1, far.device())
        .expect("an empty port");
    below.set_enabled(PORTS - 1, true);

    Fixture {
        root,
        near,
        below,
        _far: far,
    }
}

impl Drop for Fixture {
    /// Cut the cycle, or the fixture never frees.
    ///
    /// Not tidiness: the reference counts go round the loop, so without this
    /// every single run leaks the whole topology and LeakSanitizer reports the
    /// harness instead of the code under test. The cycle exists for the length
    /// of the run, which is what the run is about.
    fn drop(&mut self) {
        self.below.detach(PORTS - 1);
        self.root.detach(0);
    }
}

/// Give the near hub an address and configure it, so its class requests are
/// reachable at a known address.
///
/// Raw transactions rather than the host-side composer, because the composer is
/// not what is being fuzzed and a fixture that could fail to build would hide
/// the interesting inputs behind a `return`.
fn enumerate(f: &Fixture) {
    let setup = SetupPacket {
        request_type: 0,
        request: request::SET_ADDRESS,
        value: u16::from(HUB_ADDRESS.0),
        index: 0,
        length: 0,
    };
    f.root.setup(DeviceAddress::DEFAULT, 0, setup);
    // The status stage is what makes the address take effect (USB 2.0 §9.4.6).
    let _ = f.root.read(DeviceAddress::DEFAULT, 0, &mut []);

    let setup = SetupPacket {
        request_type: 0,
        request: request::SET_CONFIGURATION,
        value: 1,
        index: 0,
        length: 0,
    };
    f.root.setup(HUB_ADDRESS, 0, setup);
    let _ = f.root.read(HUB_ADDRESS, 0, &mut []);
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("hub", "usb.hub").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hub", "usb.hub", 1).ok()?;
        f.near.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

/// **The property this target is for.** Routing over a graph with a cycle in it
/// terminates, and never answers with a device that is not the one asked for.
fn routes_terminate(f: &Fixture, address: DeviceAddress) {
    for bus in [&f.root, &f.below] {
        if let Some(device) = bus.find(address) {
            assert_eq!(
                device.address(),
                address,
                "the routing walk returned a device that answers to something else"
            );
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let f = build();
    enumerate(&f);
    let mut plugged: Vec<Option<Arc<HidMouse>>> = (0..PORTS).map(|_| None).collect();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 8 > data.len() {
                    break;
                }
                let mut raw = [0u8; 8];
                raw.copy_from_slice(&data[at..at + 8]);
                at += 8;
                // Whatever those eight bytes say — a port number of 0xffff, a
                // feature selector of 0xffff, a `wLength` of 0xffff — this
                // returns, having allocated nothing proportional to any of them.
                f.root.setup(HUB_ADDRESS, 0, SetupPacket::decode(&raw));
                let mut buf = [0u8; 64];
                let _ = f.root.read(HUB_ADDRESS, 0, &mut buf);
                let _ = f.root.write(HUB_ADDRESS, 0, &buf[..8]);
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let want = usize::from(data[at]) % MAX_PACKET;
                at += 1;

                // The debug path first, twice. A monitor showing what the hub
                // would report must not sample the fabric, set a change bit, or
                // finish a deferred port reset.
                let powered_before: Vec<bool> = (0..PORTS).map(|p| f.near.port_powered(p)).collect();
                let enabled_before: Vec<bool> = (0..PORTS).map(|p| f.near.port_enabled(p)).collect();
                let mut first = vec![0u8; want];
                let mut second = vec![0u8; want];
                let a = f.root.peek(HUB_ADDRESS, ENDPOINT, &mut first);
                let b = f.root.peek(HUB_ADDRESS, ENDPOINT, &mut second);
                assert_eq!(a, b, "a debug peek had a side effect");
                assert_eq!(first, second, "a debug peek moved something");
                for port in 0..PORTS {
                    assert_eq!(
                        f.near.port_powered(port),
                        powered_before[usize::from(port)],
                        "a debug peek powered a port"
                    );
                    assert_eq!(
                        f.near.port_enabled(port),
                        enabled_before[usize::from(port)],
                        "a debug peek enabled a port"
                    );
                }

                let mut live = vec![0u8; want];
                let done = f.root.read(HUB_ADDRESS, ENDPOINT, &mut live);
                if done.status == Status::Ack {
                    let moved = done.len as usize;
                    assert!(moved <= want, "an IN returned more than the packet size");
                    assert!(
                        moved <= CHANGE_BYTES,
                        "the status change bitmap is one bit for the hub and one per port \
                         (§11.12.4), so it cannot be {moved} bytes"
                    );
                }
            }
            0x02 => {
                if at + 2 > data.len() {
                    break;
                }
                let port = data[at] % PORTS;
                let speed = match data[at + 1] % 3 {
                    0 => Speed::Low,
                    1 => Speed::Full,
                    _ => Speed::High,
                };
                at += 2;
                // The far hub's port is never disturbed: the cycle has to stay
                // in the topology for the whole run.
                if port + 1 == PORTS {
                    continue;
                }
                if plugged[usize::from(port)].is_none() {
                    let mouse = Arc::new(HidMouse::new_detached_at_speed(0, 0, speed));
                    if f.below.attach(port, mouse.device()).is_ok() {
                        plugged[usize::from(port)] = Some(mouse);
                    }
                }
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let port = data[at] % PORTS;
                at += 1;
                if port + 1 == PORTS {
                    continue;
                }
                f.below.detach(port);
                plugged[usize::from(port)] = None;
            }
            0x04 => {
                if at >= data.len() {
                    break;
                }
                let address = DeviceAddress(data[at] & DeviceAddress::MAX);
                at += 1;
                routes_terminate(&f, address);
            }
            // A `SOF` is broadcast to every device in the tree, so it walks the
            // same cycle the routing does and is bounded the same way.
            0x05 => f.root.start_of_frame(0),
            0x06 => {
                // A bus reset takes the hub back to *Powered-off* on every
                // port, so the harness re-enumerates: the interesting sequences
                // are the ones that continue.
                f.root.reset_port(0);
                f.root.set_enabled(0, true);
                enumerate(&f);
            }
            0x07 => {
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("hub", "usb.hub", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .near
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(snapshot(&fresh), Some(bytes), "the hub did not round trip");
                }
            }
            0x08 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.near.load(&mut r);
                at = data.len();
                // And the hub still routes afterwards — including through
                // whatever port state those bytes claimed.
                routes_terminate(&f, DeviceAddress::DEFAULT);
                routes_terminate(&f, HUB_ADDRESS);
                f.root.start_of_frame(0);
            }
            0x09 => f.near.reset(ResetKind::Cold),
            _ => {}
        }
        // After every step, whatever it was: the tree is still walkable.
        routes_terminate(&f, DeviceAddress::DEFAULT);
    }
});
