//! `pktkit` behind the [`NetLink`] seam: a hub, a TAP, slirp or a tunnel as
//! something a NIC model can be attached to.
//!
//! This is the one file under `dev/` that needs `std`, and it is the reason
//! `ROADMAP.md` §0 grants `dev/net/*` a `std` exception at all: `pktkit` is a
//! `std` crate. Everything else here — the seam, the NE2000 — is
//! `no_std + alloc`.
//!
//! # The bridge is a station, not a wrapper
//!
//! [`PktkitLink`] implements **both** traits. To `pktkit` it is an ordinary
//! `L2Device` — the emulated machine as a station on the network, which is what
//! it actually is — and to a NIC model it is a [`NetLink`]. So it plugs into an
//! `L2Hub`, a `connect_l2` cable, an `L2Adapter` in front of slirp or a
//! WireGuard tunnel, with nothing to configure:
//!
//! ```text
//!    the NIC model                PktkitLink                   pktkit
//!    ─────────────                ──────────                   ──────
//!    NetLink::transmit  ──────►  the installed L2Handler  ──►  hub / tap / slirp
//!                                                                    │
//!    NetLink::receive   ◄──────  NetPort (tick, seq queue) ◄──  L2Device::send
//!      (at a tick the                                            (a host thread)
//!       scheduler chose)
//! ```
//!
//! Wrapping somebody else's `L2Device` instead would not have worked, and the
//! reason is worth recording: `L2Hub::connect` installs a handler of its own on
//! every device it takes, so a wrapper that also called `set_handler` would
//! silently displace the hub and the station would go deaf.
//!
//! # Where push becomes pull
//!
//! `L2Device::send` is called the moment a frame exists, on whatever host
//! thread produced it, and at that instant the machine has no position in
//! virtual time. A NIC cannot accept a frame there: it would land in the
//! guest's receive ring on a different guest cycle on every run — the exact
//! non-determinism `CLAUDE.md` forbids. So the frame goes into a [`NetPort`]
//! and the NIC takes it out at a tick the scheduler chose.
//!
//! The queue publishes its earliest arrival into an atomic, which is what the
//! NIC answers `Device::next_event_tick` out of — it may not take a lock there.
//! A frame that lands between the scheduler asking that question and acting on
//! the answer therefore waits until the next time it asks, which is the next
//! quantum boundary. That is a bounded delay, not a stall, and it is far inside
//! the jitter a live host backend already has.
//!
//! # The determinism this does and does not buy
//!
//! A live backend is **not** deterministic and this bridge does not pretend
//! otherwise: which tick a frame lands on depends on when the host produced it.
//! What makes a run reproducible *afterwards* is
//! [`core::record`](crate::core::record), which this port is a channel of:
//! register it with [`ports::channel`](super::link::ports::channel) and
//! [`ports::sink`](super::link::ports::sink), and every frame is logged against
//! the scheduling-round boundary the machine delivered it on rather than
//! against a tick a host thread happened to produce. A station wired straight
//! to a hub — [`PktkitLink::inbox`] fed by `pktkit`'s own handler — is the
//! *unrecorded* form of the same path: the frames reach the guest, and nothing
//! writes down when.
//!
//! Two things in `pktkit` read the host's wall clock and are worth knowing
//! about before wiring a machine to them: `L2Hub`'s MAC-learning table ages
//! entries on `Instant::now`, and `L2Adapter`'s ARP and NDP caches expire the
//! same way. Neither is reachable from inside the scheduler and neither changes
//! what the guest sees on a two-station link, but a topology that depends on an
//! aged-out MAC entry is a topology whose *recording* is the only reproducible
//! artefact.
//!
//! # Threads
//!
//! This module spawns none and joins none. Backends that do — `pktkit`'s TAP
//! reader, its QEMU-socket server — spawn them on the host side of
//! [`L2Device::send`], which is above the `std` line and outside the scheduler
//! entirely. Nothing under `dev/` names `std::thread` (`CLAUDE.md`).

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use pktkit::{Frame, L2Device, L2Handler};

use crate::core::sync::{LockRank, Mutex};

use super::link::{MacAddr, NetLink, NetPort};

/// The emulated machine as a station on a `pktkit` network.
///
/// An `L2Device` to `pktkit` and a [`NetLink`] to a NIC model. Hold it as an
/// `Arc`: `pktkit` has a blanket `L2Device` impl for `Arc<T>`, so the same
/// handle goes to `L2Hub::connect_arc`, to `connect_l2`, and to
/// [`Ne2000::with_link`](super::ne2000::Ne2000::with_link).
pub struct PktkitLink {
    /// Where arrivals wait for the machine to come and get them.
    inbox: Arc<NetPort>,
    /// What to call to put a frame on the network. Installed by whatever this
    /// station is attached to; `None` until then, which is a station with no
    /// cable in it.
    handler: Mutex<Option<L2Handler>>,
}

impl fmt::Debug for PktkitLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PktkitLink")
            .field("mac", &self.inbox.mac())
            .field("attached", &self.handler.lock().is_some())
            .field("inbox", &self.inbox)
            .finish()
    }
}

impl PktkitLink {
    /// A station with a fresh inbox.
    #[must_use]
    pub fn new() -> Arc<PktkitLink> {
        PktkitLink::with_inbox(Arc::new(NetPort::new()))
    }

    /// A station whose inbox the caller already holds — a
    /// [`ports`](super::link::ports) entry a machine file named, or a port a
    /// test wants to record from.
    #[must_use]
    pub fn with_inbox(inbox: Arc<NetPort>) -> Arc<PktkitLink> {
        Arc::new(PktkitLink {
            inbox,
            // A leaf: the handler is cloned out from under this lock before it
            // is called, so nothing is ever held across the outward call.
            handler: Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// The inbox arrivals are queued in.
    ///
    /// The host object a recording names: hand it to
    /// [`ports::sink`](super::link::ports::sink) and the frames it hands the
    /// guest become one stream of a recording.
    #[must_use]
    pub fn inbox(&self) -> &Arc<NetPort> {
        &self.inbox
    }

    /// Whether anything is on the other end of the cable yet.
    #[must_use]
    pub fn attached(&self) -> bool {
        self.handler.lock().is_some()
    }
}

impl L2Device for PktkitLink {
    fn set_handler(&self, h: L2Handler) {
        *self.handler.lock() = Some(h);
    }

    fn send(&self, frame: &Frame) -> pktkit::Result<()> {
        // The network handing the machine a frame. This is the push, and this
        // is where it stops: the bytes are copied into the inbox against "the
        // NIC's next look", and the NIC decides when that is.
        self.inbox.deliver(frame.as_bytes());
        Ok(())
    }

    fn hw_addr(&self) -> pktkit::MacAddr {
        pktkit::MacAddr(self.inbox.mac().octets())
    }

    fn close(&self) -> pktkit::Result<()> {
        *self.handler.lock() = None;
        // A station whose cable has been pulled has no carrier, and a driver
        // that transmits into it gets the data sheet's lost-carrier abort.
        self.inbox.set_link(false);
        Ok(())
    }
}

impl NetLink for PktkitLink {
    fn transmit(&self, _now: u64, frame: &[u8]) {
        // A frame the guest built can be anything, and `Frame::from_slice` is a
        // cast rather than a parse — a short one would only be rejected further
        // along, inside somebody else's accessor. Refuse it here.
        if frame.len() < 14 {
            return;
        }
        // Cloned out from under the lock, then called with nothing held: a hub
        // calls straight into a peer device from inside this (`CLAUDE.md`,
        // re-entrancy).
        let handler = self.handler.lock().clone();
        if let Some(handler) = handler {
            // Errors are the wire's, not the guest's: a real NIC transmitting
            // into a dead cable also reports success to its driver.
            let _ = handler(Frame::from_slice(frame));
        }
    }

    fn receive(&self, now: u64) -> Option<Vec<u8>> {
        self.inbox.receive(now)
    }

    fn next_arrival(&self) -> Option<u64> {
        self.inbox.next_arrival()
    }

    fn link_up(&self) -> bool {
        self.inbox.link_up()
    }

    fn set_mac(&self, mac: MacAddr) {
        self.inbox.set_mac(mac);
    }
}

impl From<MacAddr> for pktkit::MacAddr {
    fn from(mac: MacAddr) -> pktkit::MacAddr {
        pktkit::MacAddr(mac.octets())
    }
}

/// The seam's address type from `pktkit`'s.
///
/// A function rather than the other `From` impl, because the orphan rule allows
/// exactly one of the two to be written here.
#[must_use]
pub fn from_pktkit_mac(mac: pktkit::MacAddr) -> MacAddr {
    MacAddr::new(mac.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pktkit::{L2Hub, MacAddr as PktMac};

    /// A 60-byte frame from `src` to `dst`, which is the shortest 802.3 allows
    /// once the FCS is taken off.
    fn frame(dst: [u8; 6], src: [u8; 6]) -> Vec<u8> {
        let mut f = Vec::with_capacity(60);
        f.extend_from_slice(&dst);
        f.extend_from_slice(&src);
        f.extend_from_slice(&[0x08, 0x00]);
        f.resize(60, 0xab);
        f
    }

    #[test]
    fn a_frame_the_network_pushes_waits_for_the_machine_to_come_and_get_it() {
        let link = PktkitLink::new();
        let sent = frame([0xff; 6], [0x52, 0x54, 0, 1, 2, 3]);
        // What a TAP reader thread does, from a TAP reader thread.
        link.send(Frame::from_slice(&sent)).unwrap();

        assert_eq!(link.next_arrival(), Some(0), "queued for the next look");
        assert_eq!(link.receive(1234).as_deref(), Some(&sent[..]));
        assert_eq!(link.receive(1234), None);
    }

    #[test]
    fn two_stations_on_a_hub_hear_each_other() {
        // The whole point of bridging to `pktkit` rather than writing a private
        // loopback: two emulated machines on one switch, and the switch is not
        // ours.
        let hub = Arc::new(L2Hub::new());
        let left = PktkitLink::new();
        let right = PktkitLink::new();
        left.set_mac(MacAddr::new([0x52, 0x54, 0, 0, 0, 1]));
        right.set_mac(MacAddr::new([0x52, 0x54, 0, 0, 0, 2]));
        let _a = hub.connect_arc(Arc::clone(&left) as Arc<dyn L2Device>);
        let _b = hub.connect_arc(Arc::clone(&right) as Arc<dyn L2Device>);
        assert!(left.attached(), "the hub installed its handler");

        let broadcast = frame([0xff; 6], [0x52, 0x54, 0, 0, 0, 1]);
        left.transmit(0, &broadcast);
        assert_eq!(right.receive(0).as_deref(), Some(&broadcast[..]));
        assert_eq!(left.receive(0), None, "and not back to the sender");

        // And a unicast the hub has learned the route for.
        let unicast = frame([0x52, 0x54, 0, 0, 0, 1], [0x52, 0x54, 0, 0, 0, 2]);
        right.transmit(0, &unicast);
        assert_eq!(left.receive(0).as_deref(), Some(&unicast[..]));
    }

    #[test]
    fn a_live_arrival_is_recorded_through_the_seam_rather_than_by_the_port() {
        use crate::core::clock::GlobalTime;
        use crate::core::record::Recorder;
        use crate::dev::net::link::ports;

        // A station whose inbox is a named host object, which is what a machine
        // file's `link = "net0"` produces.
        let hosts = crate::core::hosts::HostObjects::new();
        let inbox = ports::open(&hosts, "net0").unwrap();
        let link = PktkitLink::with_inbox(Arc::clone(&inbox));
        let recorder = Recorder::recording();
        recorder
            .register(ports::channel("net0"), ports::sink(&inbox))
            .unwrap();

        // `send` is the *unrecorded* path: a hub's handler putting a frame in
        // the inbox, which is exactly what a live backend does.
        let one = frame([0xff; 6], [0, 0, 0, 0, 0, 1]);
        link.send(Frame::from_slice(&one)).unwrap();
        assert!(link.receive(700).is_some());
        assert!(recorder.log().is_empty(), "the port writes nothing down");

        // Through the seam, the same frame is one logged event against the
        // round boundary the machine delivered it on.
        let two = frame([0xff; 6], [0, 0, 0, 0, 0, 2]);
        recorder.post(&ports::channel("net0"), &two).unwrap();
        recorder.deliver(GlobalTime::from_nanos(9_000)).unwrap();
        assert_eq!(link.receive(0).as_deref(), Some(&two[..]));
        let log = recorder.log();
        assert_eq!(log.len(), 1);
        assert_eq!(log.events()[0].at, GlobalTime::from_nanos(9_000));
    }

    #[test]
    fn a_station_with_no_cable_swallows_what_the_guest_transmits() {
        let link = PktkitLink::new();
        link.transmit(0, &frame([0xff; 6], [0; 6]));
        assert_eq!(link.receive(0), None);
        // And a runt never reaches `pktkit` at all: `Frame::from_slice` on four
        // bytes would produce a `Frame` whose accessors index out of it.
        link.transmit(0, &[1, 2, 3, 4]);
    }

    #[test]
    fn closing_the_station_drops_the_carrier() {
        let link = PktkitLink::new();
        let hub = Arc::new(L2Hub::new());
        let _h = hub.connect_arc(Arc::clone(&link) as Arc<dyn L2Device>);
        assert!(link.link_up());
        link.close().unwrap();
        assert!(!link.attached());
        assert!(!link.link_up(), "a NIC transmitting now loses carrier");
    }

    #[test]
    fn the_station_reports_the_address_the_guest_programmed() {
        let link = PktkitLink::new();
        let ours = MacAddr::parse("52:54:00:12:34:56").unwrap();
        link.set_mac(ours);
        assert_eq!(link.hw_addr().0, ours.octets());

        // And the two address types convert both ways.
        let theirs: PktMac = ours.into();
        assert_eq!(theirs.0, ours.octets());
        assert_eq!(from_pktkit_mac(theirs), ours);
    }
}
