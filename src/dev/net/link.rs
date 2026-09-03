//! The network seam: frames out, frames in, link state, MAC address.
//!
//! One trait, [`NetLink`], and one implementation of it, [`NetPort`], that is a
//! pair of frame queues in memory. A NIC model holds an `Arc<dyn NetLink>` and
//! never learns whether the other end is a `pktkit` hub, a host tap, a scripted
//! test, or its own transmit queue looped back.
//!
//! This is deliberately the same shape as [`chardev`](crate::host::chardev):
//! one seam trait, one in-memory reference implementation that every test uses,
//! and a name in the machine file that both ends resolve through the build's
//! [`HostObjects`](crate::core::hosts::HostObjects). What is different is the
//! *direction of control*, and that difference is the whole reason this module
//! exists rather than a `use pktkit::L2Device`.
//!
//! # Why receive is a pull and not a push
//!
//! `ROADMAP.md` §7.2 says every emulated NIC is a `pktkit::L2Device`. It cannot
//! be, and the reason is worth writing down rather than working around.
//!
//! `pktkit::L2Device` delivers a received frame by **calling a handler** the
//! moment the frame exists — on whatever host thread the tap reader, the hub or
//! the peer device happens to be on. An emulated machine has no defined
//! position in virtual time at that instant. Accepting the frame there would
//! mean a frame landing in the guest's receive ring at whichever guest cycle
//! the host scheduler happened to reach, which is precisely the
//! "non-deterministic input crossing into the machine" `CLAUDE.md` forbids:
//! run the same machine twice and the guest sees the packet at two different
//! cycles, so the state hash differs and every regression built on it is
//! worthless.
//!
//! So the seam inverts it. A frame arriving from outside is **queued against a
//! virtual tick**, and the NIC *pulls* it out at a tick the scheduler chose:
//!
//! ```text
//!   outside                    NetLink                     the NIC
//!   ───────                    ───────                     ───────
//!   deliver_at(tick, frame) ─►  (tick, seq) ordered queue
//!                                        │
//!   next_arrival() ◄────────────────────┤ ──► Device::next_event_tick
//!                                        │        (the scheduler stops there)
//!                               receive(now) ◄─── Device::advance_to(now)
//!                                                     └─► into the ring, IRQ
//! ```
//!
//! Two properties fall out, and they are the point of the design:
//!
//! * **Reproducible.** The same `(tick, frame)` sequence produces the same
//!   guest-visible bytes at the same guest cycles, on any host, at any speed,
//!   under any threading mode.
//! * **Recordable.** An arrival is `(instant, bytes)`, which is exactly what
//!   [`core::record`](crate::core::record) carries. This port used to keep its
//!   own `Vec<(tick, frame)>` log and replay it by re-queueing; that log is
//!   gone and the general seam is the one mechanism. A NIC's port is registered
//!   as a channel — [`ports::channel`] names it, [`ports::sink`] is the two
//!   closures that connect it — and the recorder stamps each frame with the
//!   scheduling-round boundary it delivered it on. What the port gives up is
//!   deciding *when*, which is the part it should give up: a device that
//!   timestamps its own input has to be trusted to pick a round boundary and
//!   nothing checked that it did.
//!
//! A `pktkit::L2Device` is then *one implementation of `NetLink`*
//! ([`super::pktkit`]), not the seam itself: its handler stamps arrivals into a
//! [`NetPort`] and the NIC pulls them out on the scheduler's terms. Nothing is
//! lost — hubs, slirp, TAP, WireGuard all still work — and the frame, MAC and
//! ether-type types stay `pktkit`'s wherever `pktkit` is in the build.
//!
//! # What this module does *not* do
//!
//! It does not parse packets. There is no ARP here, no IP, no checksum, no
//! ether-type table: that is what `pktkit` is for, and duplicating it would be
//! the parallel abstraction `CLAUDE.md` warns against. A frame is a byte
//! sequence and a MAC address is six bytes, because those two are all a NIC
//! model — which is a *link*-layer part — is entitled to know.
//!
//! [`MacAddr`] is six bytes with a parser and a `Display`, and exists here only
//! because `pktkit` is `std` and this seam is `no_std + alloc`: a Z80 board with
//! an NE2000 has no business pulling in a TCP stack. Conversion in both
//! directions lives in [`super::pktkit`].

use alloc::collections::{BTreeMap, VecDeque};
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};

/// The largest Ethernet frame this seam carries, in bytes.
///
/// 1518 = 14 octets of header + 1500 of payload + the 4-octet FCS, from
/// IEEE 802.3 §4.2.7.1. **The FCS is not modelled** — no backend on this seam
/// produces one and none checks one — so a frame here is at most 1514 bytes in
/// practice; the constant keeps the datasheet arithmetic honest for a NIC that
/// sizes a buffer against the standard's number.
pub const MAX_FRAME_LEN: u64 = 1518;

/// The shortest legal Ethernet frame, in bytes (IEEE 802.3 §4.2.3.3).
///
/// 64 with the FCS, so 60 without. A transmitter is required to pad up to it;
/// the DP8390 does not, and neither does this seam — that is the driver's job
/// on real hardware and it stays the driver's job here.
pub const MIN_FRAME_LEN: u64 = 60;

/// How many frames a [`NetPort`] holds in each direction before it drops.
///
/// A real NIC's receive FIFO overruns and a real wire drops; a queue that grew
/// without bound would tell the device model a lie about the hardware and would
/// let a guest that never drains its ring exhaust the host's heap.
pub const PORT_CAPACITY: usize = 256;

/// An IEEE 802 48-bit hardware address.
///
/// Six bytes in wire order: `octets()[0]` is the first byte on the wire, whose
/// low bit is the individual/group flag and whose next bit is the
/// universal/local flag (IEEE 802-2014 §8.2).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[repr(transparent)]
pub struct MacAddr(pub [u8; 6]);

impl MacAddr {
    /// The all-zero address, which is what an unprogrammed NIC reports.
    pub const ZERO: MacAddr = MacAddr([0; 6]);

    /// `ff:ff:ff:ff:ff:ff`.
    pub const BROADCAST: MacAddr = MacAddr([0xff; 6]);

    /// An address from its six octets.
    #[must_use]
    pub const fn new(octets: [u8; 6]) -> MacAddr {
        MacAddr(octets)
    }

    /// The six octets, in wire order.
    #[must_use]
    pub const fn octets(self) -> [u8; 6] {
        self.0
    }

    /// Whether this is the broadcast address.
    #[must_use]
    pub fn is_broadcast(self) -> bool {
        self.0 == [0xff; 6]
    }

    /// Whether the group bit is set — broadcast counts as multicast, as the
    /// standard defines it.
    #[must_use]
    pub const fn is_multicast(self) -> bool {
        self.0[0] & 0x01 != 0
    }

    /// Whether the group bit is clear.
    #[must_use]
    pub const fn is_unicast(self) -> bool {
        !self.is_multicast()
    }

    /// Which of the 64 multicast hash buckets this address falls in.
    ///
    /// The DP8390 filters multicast with a 64-bit table addressed by the **top
    /// six bits** of the CRC-32 of the destination address (DP8390D data sheet,
    /// "Multicast Address Filtering"). The polynomial is the ordinary Ethernet
    /// one, `0x04c1_1db7`, applied in its reflected form — the same CRC the FCS
    /// uses (IEEE 802.3 §3.2.9), so this is the frame check sequence's own
    /// function over six bytes rather than a second algorithm.
    #[must_use]
    pub fn multicast_hash(self) -> u8 {
        let mut crc: u32 = 0xffff_ffff;
        for byte in self.0 {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                // Reflected form: shift right, xor the reversed polynomial.
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xedb8_8320
                } else {
                    crc >> 1
                };
            }
        }
        // The data sheet indexes with the six *most significant* bits of the
        // unreflected CRC. In this reflected computation those are the six
        // least significant bits, in the opposite order.
        let low = (crc & 0x3f) as u8;
        let mut index = 0u8;
        for bit in 0..6 {
            if low & (1 << bit) != 0 {
                index |= 1 << (5 - bit);
            }
        }
        index
    }

    /// Parse `aa:bb:cc:dd:ee:ff`, or the same with `-` separators.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if it is not six hexadecimal octets.
    pub fn parse(text: &str) -> Result<MacAddr> {
        let bad = || Error::Config {
            at: String::from("mac"),
            message: alloc::format!(
                "`{text}` is not a MAC address: want six hex octets, aa:bb:cc:dd:ee:ff"
            ),
        };
        let mut octets = [0u8; 6];
        let mut parts = text.split([':', '-']);
        for slot in &mut octets {
            let part = parts.next().ok_or_else(bad)?;
            if part.len() != 2 {
                return Err(bad());
            }
            *slot = u8::from_str_radix(part, 16).map_err(|_| bad())?;
        }
        if parts.next().is_some() {
            return Err(bad());
        }
        Ok(MacAddr(octets))
    }
}

impl fmt::Display for MacAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d, e, g] = self.0;
        write!(f, "{a:02x}:{b:02x}:{c:02x}:{d:02x}:{e:02x}:{g:02x}")
    }
}

impl fmt::Debug for MacAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl From<[u8; 6]> for MacAddr {
    fn from(octets: [u8; 6]) -> MacAddr {
        MacAddr(octets)
    }
}

/// What a NIC model is attached to: the far side of its wire.
///
/// Every method takes `&self` — a device is shared, and the backend keeps
/// whatever interior mutability it needs — and `Send + Sync` from the first
/// commit like every other device-facing trait (`ROADMAP.md` §0). Nothing here
/// blocks, and nothing here reads a clock: virtual time is a parameter, because
/// the *caller* is the one that knows what time it is.
pub trait NetLink: Send + Sync + fmt::Debug {
    /// The guest has put `frame` on the wire, at tick `now` of the NIC's clock
    /// domain.
    ///
    /// Called with no device lock held (`CLAUDE.md`, re-entrancy): a backend is
    /// free to call straight into a peer, and a peer is free to answer.
    fn transmit(&self, now: u64, frame: &[u8]);

    /// Take the next frame whose arrival tick is at or before `now`.
    ///
    /// Returns `None` when nothing has arrived yet — never a blocking wait, and
    /// never a frame from the future. Frames come out in arrival order, and
    /// ties are broken by the order they were queued in, so the sequence is a
    /// function of the input alone.
    fn receive(&self, now: u64) -> Option<Vec<u8>>;

    /// The tick the next queued frame becomes visible, if one is queued.
    ///
    /// A NIC answers [`Device::next_event_tick`] out of this, so the scheduler
    /// stops the world at exactly that tick rather than at the end of a
    /// quantum.
    ///
    /// **Must not take a lock**, and must not allocate or block. The scheduler
    /// asks with its own slot lock held at [`LockRank::LEAF`], the rank nothing
    /// nests under, so an implementation publishes the number into an atomic as
    /// its queue changes rather than looking it up. It also has to be *fresh*:
    /// the outside queues a frame without telling the machine, so a value the
    /// device cached at its last register access would be stale exactly when it
    /// mattered.
    ///
    /// [`Device::next_event_tick`]: crate::core::device::Device::next_event_tick
    fn next_arrival(&self) -> Option<u64>;

    /// Whether the carrier is present.
    ///
    /// A NIC that transmits with this false reports a lost-carrier error to its
    /// driver, which is what the pin does on real silicon. It is a *query*, and
    /// a NIC may ask it with its own state lock held — so an implementation may
    /// take a leaf lock of its own and must not call outward from here.
    fn link_up(&self) -> bool {
        true
    }

    /// The guest has programmed its hardware address.
    ///
    /// An outward call, so a NIC makes it from `realize` or from a register
    /// write with its own lock released. The default ignores it, which is right
    /// for any backend that does not care who is on the other end.
    fn set_mac(&self, mac: MacAddr) {
        let _ = mac;
    }
}

/// Two frame queues with a lock around them: the reference [`NetLink`].
///
/// This is the deterministic end of the seam and the one every test uses. It is
/// also what a live backend attaches *to*: [`PktkitLink`] stamps frames into one
/// of these from whatever host thread produced them, so nothing inside the
/// scheduler ever calls out to a socket.
///
/// # Ordering
///
/// Arrivals are held in a `BTreeMap` keyed by `(tick, sequence)`, never a
/// `HashMap` (`CLAUDE.md`, determinism): two frames queued for the same tick
/// come out in the order they were queued.
///
/// [`PktkitLink`]: super::pktkit::PktkitLink
pub struct NetPort {
    state: Mutex<PortState>,
    /// The tick of the earliest queued arrival, or [`u64::MAX`] for none.
    ///
    /// Published beside the queue rather than read out of it, because
    /// [`NetLink::next_arrival`] is asked with the scheduler's slot lock held
    /// and may not take one of its own.
    earliest: AtomicU64,
}

/// Both queues plus the wire's own state. One struct so one lock covers them,
/// which keeps "read what the guest sent, then answer it" atomic for a test.
#[derive(Debug)]
struct PortState {
    /// Outside to guest, keyed by `(arrival tick, sequence)`.
    inbound: BTreeMap<(u64, u64), Vec<u8>>,
    /// The tie-break counter for `inbound`'s key.
    seq: u64,
    /// Guest to outside, in the order the guest sent them.
    outbound: VecDeque<Vec<u8>>,
    link_up: bool,
    mac: MacAddr,
    /// When set, a transmitted frame is queued back as an arrival this many
    /// ticks later.
    loopback: Option<u64>,
    /// Frames dropped because the inbound queue was at [`PORT_CAPACITY`].
    dropped_in: u64,
    /// Frames dropped because the outbound queue was at [`PORT_CAPACITY`].
    dropped_out: u64,
}

impl Default for PortState {
    fn default() -> PortState {
        PortState {
            inbound: BTreeMap::new(),
            seq: 0,
            outbound: VecDeque::new(),
            link_up: true,
            mac: MacAddr::ZERO,
            loopback: None,
            dropped_in: 0,
            dropped_out: 0,
        }
    }
}

impl fmt::Debug for NetPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.state.try_lock() {
            Some(state) => f
                .debug_struct("NetPort")
                .field("mac", &state.mac)
                .field("link_up", &state.link_up)
                .field("inbound", &state.inbound.len())
                .field("outbound", &state.outbound.len())
                .finish(),
            None => f
                .debug_struct("NetPort")
                .field("state", &"<in use>")
                .finish(),
        }
    }
}

impl Default for NetPort {
    fn default() -> NetPort {
        NetPort::new()
    }
}

impl NetPort {
    /// An empty port with the carrier up.
    #[must_use]
    pub fn new() -> NetPort {
        NetPort {
            // A leaf: nothing is ever locked while this one is held, so it
            // nests under a device's own state lock and under the bus.
            state: Mutex::with_rank(LockRank::LEAF, PortState::default()),
            earliest: AtomicU64::new(u64::MAX),
        }
    }

    /// Republish [`NetPort::earliest`]. Called with the state lock held.
    fn publish(&self, state: &PortState) {
        self.earliest.store(
            state.inbound.keys().next().map_or(u64::MAX, |k| k.0),
            Ordering::Relaxed,
        );
    }

    /// A port that hands every transmitted frame back as an arrival `latency`
    /// ticks later.
    ///
    /// The deterministic self-test backend: attach one to a NIC and its own
    /// driver's transmit path feeds its own receive path, through the real
    /// registers, with the delay a real wire would have. Nothing about it reads
    /// a clock — the latency is counted in the NIC's clock domain.
    #[must_use]
    pub fn loopback(latency: u64) -> NetPort {
        let port = NetPort::new();
        port.state.lock().loopback = Some(latency);
        port
    }

    // -- the outside's side -------------------------------------------------

    /// Queue `frame` to become visible to the guest at `tick`.
    ///
    /// Returns false if the queue was full, which is a drop and is counted. A
    /// `tick` already in the past means "at the NIC's next look", which is what
    /// a live backend with no opinion about virtual time asks for.
    pub fn deliver_at(&self, tick: u64, frame: &[u8]) -> bool {
        let mut state = self.state.lock();
        if state.inbound.len() >= PORT_CAPACITY {
            state.dropped_in += 1;
            return false;
        }
        let seq = state.seq;
        state.seq += 1;
        state.inbound.insert((tick, seq), frame.to_vec());
        self.publish(&state);
        true
    }

    /// Queue `frame` for the NIC's next look. Shorthand for `deliver_at(0, …)`.
    pub fn deliver(&self, frame: &[u8]) -> bool {
        self.deliver_at(0, frame)
    }

    /// Take the first frame the guest has transmitted, if any.
    #[must_use]
    pub fn take(&self) -> Option<Vec<u8>> {
        self.state.lock().outbound.pop_front()
    }

    /// Take everything the guest has transmitted.
    #[must_use]
    pub fn drain(&self) -> Vec<Vec<u8>> {
        self.state.lock().outbound.drain(..).collect()
    }

    /// How many frames the guest has transmitted and nobody has taken.
    #[must_use]
    pub fn pending_output(&self) -> usize {
        self.state.lock().outbound.len()
    }

    /// How many frames are queued for the guest, due or not.
    #[must_use]
    pub fn pending_input(&self) -> usize {
        self.state.lock().inbound.len()
    }

    /// Frames dropped for want of queue space: `(toward the guest, away)`.
    #[must_use]
    pub fn dropped(&self) -> (u64, u64) {
        let state = self.state.lock();
        (state.dropped_in, state.dropped_out)
    }

    /// Raise or drop the carrier.
    pub fn set_link(&self, up: bool) {
        self.state.lock().link_up = up;
    }

    /// The hardware address the guest last programmed.
    #[must_use]
    pub fn mac(&self) -> MacAddr {
        self.state.lock().mac
    }

    /// Forget every frame queued for the guest and not yet taken.
    ///
    /// The rewind hook ([`InputSink::on_rewind`](crate::core::record::InputSink::on_rewind)):
    /// the frames sitting in the inbound queue at a rewind target are
    /// re-delivered from the recording on the way forward, so a port that kept
    /// them would hand the guest each one twice. What the guest has already
    /// *transmitted* is not touched — that is output, and output has left.
    pub fn drop_queued(&self) {
        let mut state = self.state.lock();
        state.inbound.clear();
        self.publish(&state);
    }

    /// Empty both queues, leaving the carrier and the address.
    pub fn clear(&self) {
        let mut state = self.state.lock();
        state.inbound.clear();
        state.outbound.clear();
        self.publish(&state);
    }
}

impl NetLink for NetPort {
    fn transmit(&self, now: u64, frame: &[u8]) {
        let mut state = self.state.lock();
        if let Some(latency) = state.loopback {
            // A loopback wire is still a wire: the frame arrives later, and it
            // arrives through the same queue everything else arrives through.
            if state.inbound.len() < PORT_CAPACITY {
                let seq = state.seq;
                state.seq += 1;
                state
                    .inbound
                    .insert((now.saturating_add(latency), seq), frame.to_vec());
                self.publish(&state);
            } else {
                state.dropped_in += 1;
            }
        }
        if state.outbound.len() >= PORT_CAPACITY {
            state.dropped_out += 1;
            return;
        }
        state.outbound.push_back(frame.to_vec());
    }

    fn receive(&self, now: u64) -> Option<Vec<u8>> {
        let mut state = self.state.lock();
        let key = *state.inbound.keys().next()?;
        if key.0 > now {
            return None;
        }
        let frame = state.inbound.remove(&key)?;
        self.publish(&state);
        Some(frame)
    }

    fn next_arrival(&self) -> Option<u64> {
        // Lock-free, as the trait requires: the scheduler asks with its own
        // slot lock held.
        match self.earliest.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn link_up(&self) -> bool {
        self.state.lock().link_up
    }

    fn set_mac(&self, mac: MacAddr) {
        self.state.lock().mac = mac;
    }
}

/// The build's named network ports.
///
/// See [`chardev::ports`](crate::host::chardev::ports) for the argument: a
/// machine description carries data, so the only thing that can travel from a
/// machine file into a device constructor is a *name*, and both ends resolve it
/// against the build's own [`HostObjects`](crate::core::hosts::HostObjects).
///
/// ```text
/// machine file:  object nic "net.ne2000" { link = "net0" }
/// device:        ports::attach(props, "net0")  ──┐
/// host:          ports::open(&hosts, "net0")   ──┴─► the same Arc<NetPort>
/// ```
///
/// # A port is a record/replay channel
///
/// A frame arriving from outside is a non-deterministic input like a keystroke,
/// so it crosses in through [`core::record`](crate::core::record) and nowhere
/// else (`CLAUDE.md`, determinism). [`ports::channel`] is the name it goes
/// under and [`ports::sink`] is the object it goes to, so wiring a NIC into a
/// recording is:
///
/// ```no_run
/// # use std::sync::Arc;
/// # use rsemu::core::record::Recorder;
/// # use rsemu::dev::net::link::ports;
/// # fn demo(hosts: &rsemu::core::hosts::HostObjects, recorder: &Recorder) -> rsemu::Result<()> {
/// let port = ports::open(hosts, "net0")?;
/// recorder.register(ports::channel("net0"), ports::sink(&port))?;
/// # Ok(())
/// # }
/// ```
///
/// From then on the host offers frames with
/// [`Recorder::post`](crate::core::record::Recorder::post) rather than with
/// [`NetPort::deliver`], and the machine decides which round boundary each one
/// lands on. [`NetPort::deliver_at`] stays, because it is how the *deterministic
/// backend* is driven from inside the machine's own timeline — a loopback wire,
/// a scripted test naming a tick of the NIC's clock domain — which is not host
/// input at all.
pub mod ports {
    use super::NetPort;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a network port is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::door("netdev", make_sink);

    /// The network port `name` refers to in `hosts`, creating it on first
    /// mention.
    ///
    /// The **host** side of the rendezvous: called before anything starts
    /// feeding frames, or after the build to pick up what a device opened.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<NetPort>> {
        hosts.open(KIND, name, NetPort::new)
    }

    /// The network port `name` refers to in the build these properties belong
    /// to, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`: acquiring a host object is
    /// allocation, not an outward action ([`core::hosts`](crate::core::hosts)
    /// argues the case). A `Props` that belongs to no build gets a private port,
    /// so a device a unit test built directly still works and simply meets
    /// nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<NetPort>> {
        props.host(KIND, name, NetPort::new)
    }

    /// The network port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<NetPort>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// [`sink`], reached through the erased handle the host-object table holds.
    ///
    /// What [`KIND`] carries so that
    /// [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) can wire
    /// this network port to a recorder without the caller having to name it. `None`
    /// means something that is not a [`NetPort`] is filed under `netdev` — two
    /// modules claiming one kind name, which the seal reports rather than
    /// guesses at.
    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<NetPort>().ok()?))
    }

    /// The record/replay channel the port called `name` receives on.
    ///
    /// `netdev:net0`, which is the same `(kind, name)` pair the host-object
    /// table files the port under — so a board whose NIC has no channel is
    /// refused by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal)
    /// naming this string.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that puts a recorded payload into `port`.
    ///
    /// A payload is one Ethernet frame, delivered for the NIC's next look: the
    /// *instant* is the round boundary the recorder delivered on, and the tick
    /// the card sees it on follows from that rather than from anything the host
    /// chose. On a rewind the port drops what it is still holding, because
    /// those frames are re-delivered from the log on the way forward.
    #[must_use]
    pub fn sink(port: &Arc<NetPort>) -> Arc<dyn InputSink> {
        let receiving = Arc::clone(port);
        let rewinding = Arc::clone(port);
        Arc::new(
            FnSink::new("netdev", move |frame: &[u8]| {
                receiving.deliver(frame);
            })
            .on_rewind(move || rewinding.drop_queued()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::sync::Arc;
    use alloc::vec;

    #[test]
    fn a_mac_round_trips_through_its_text_form() {
        let mac = MacAddr::parse("52:54:00:12:34:56").expect("six octets");
        assert_eq!(mac.octets(), [0x52, 0x54, 0x00, 0x12, 0x34, 0x56]);
        assert_eq!(mac.to_string(), "52:54:00:12:34:56");
        assert_eq!(MacAddr::parse("52-54-00-12-34-56").unwrap(), mac);
        assert!(mac.is_unicast());
        assert!(MacAddr::BROADCAST.is_broadcast());
        assert!(MacAddr::BROADCAST.is_multicast());
    }

    #[test]
    fn a_mac_that_is_not_six_octets_is_rejected() {
        for bad in [
            "",
            "52:54:00:12:34",
            "52:54:00:12:34:56:78",
            "zz:54:00:12:34:56",
            "5:54:00:12:34:56",
        ] {
            assert!(MacAddr::parse(bad).is_err(), "`{bad}` should not parse");
        }
    }

    #[test]
    fn frames_cross_in_both_directions_without_meeting() {
        let port = NetPort::new();
        assert!(port.deliver(b"inbound"));
        port.transmit(0, b"outbound");
        assert_eq!(port.receive(0).as_deref(), Some(&b"inbound"[..]));
        assert_eq!(port.receive(0), None);
        assert_eq!(port.drain(), vec![b"outbound".to_vec()]);
    }

    #[test]
    fn a_frame_from_the_future_is_not_visible_yet() {
        let port = NetPort::new();
        port.deliver_at(100, b"later");
        assert_eq!(port.next_arrival(), Some(100));
        assert_eq!(port.receive(99), None, "99 is before 100");
        assert_eq!(port.receive(100).as_deref(), Some(&b"later"[..]));
        assert_eq!(port.next_arrival(), None);
    }

    #[test]
    fn two_frames_at_one_tick_come_out_in_the_order_they_went_in() {
        // The whole reason the key is (tick, seq) and the map is a BTreeMap.
        let port = NetPort::new();
        for i in 0..8u8 {
            port.deliver_at(7, &[i]);
        }
        for i in 0..8u8 {
            assert_eq!(port.receive(7).as_deref(), Some(&[i][..]));
        }
    }

    #[test]
    fn a_loopback_port_hands_a_transmission_back_after_its_latency() {
        let port = NetPort::loopback(10);
        port.transmit(100, b"hello");
        assert_eq!(port.next_arrival(), Some(110));
        assert_eq!(port.receive(109), None);
        assert_eq!(port.receive(110).as_deref(), Some(&b"hello"[..]));
        // And it still went out, so a test can see both halves.
        assert_eq!(port.drain(), vec![b"hello".to_vec()]);
    }

    #[test]
    fn a_frame_reaches_the_port_through_the_record_seam() {
        // The conversion, in one test: the host posts a frame and nothing
        // happens until the machine drains the recorder at a round boundary.
        use crate::core::clock::GlobalTime;
        use crate::core::record::Recorder;

        let hosts = crate::core::hosts::HostObjects::new();
        let port = ports::open(&hosts, "net0").unwrap();
        let recorder = Recorder::recording();
        recorder
            .register(ports::channel("net0"), ports::sink(&port))
            .unwrap();

        recorder.post(&ports::channel("net0"), b"frame").unwrap();
        assert_eq!(port.pending_input(), 0, "posting delivers nothing");
        recorder.deliver(GlobalTime::from_nanos(1_000)).unwrap();
        assert_eq!(port.receive(0).as_deref(), Some(&b"frame"[..]));

        // And the recording carries it against that boundary, under the same
        // name the host-object table files the port under.
        let log = recorder.log();
        assert_eq!(log.len(), 1);
        assert_eq!(log.events()[0].channel.to_string(), "netdev:net0");
        assert_eq!(log.events()[0].payload, b"frame");
    }

    #[test]
    fn a_rewind_drops_what_the_guest_has_not_taken() {
        // Without this the frames queued at the rewind target arrive twice:
        // once from the queue that survived and once from the log.
        let port = NetPort::new();
        port.deliver(b"queued");
        port.transmit(0, b"sent");
        port.drop_queued();
        assert_eq!(port.pending_input(), 0);
        assert_eq!(port.next_arrival(), None);
        assert_eq!(
            port.pending_output(),
            1,
            "what the guest already transmitted has left and is not rewound"
        );
    }

    #[test]
    fn a_full_port_drops_rather_than_growing() {
        let port = NetPort::new();
        for i in 0..PORT_CAPACITY {
            assert!(port.deliver_at(i as u64, b"x"));
        }
        assert!(!port.deliver(b"one too many"));
        assert_eq!(port.dropped().0, 1);
        for _ in 0..PORT_CAPACITY + 4 {
            port.transmit(0, b"y");
        }
        assert_eq!(port.pending_output(), PORT_CAPACITY);
        assert_eq!(port.dropped().1, 4);
    }

    #[test]
    fn the_carrier_and_the_address_are_part_of_the_seam() {
        let port = NetPort::new();
        assert!(port.link_up());
        port.set_link(false);
        assert!(!port.link_up());
        assert_eq!(port.mac(), MacAddr::ZERO);
        port.set_mac(MacAddr::parse("02:00:00:00:00:01").unwrap());
        assert_eq!(port.mac().to_string(), "02:00:00:00:00:01");
    }

    #[test]
    fn a_name_reaches_the_same_port_from_both_ends() {
        let hosts = crate::core::hosts::HostObjects::new();
        let device_end: Arc<dyn NetLink> = ports::open(&hosts, "net0").unwrap();
        let host_end = ports::open(&hosts, "net0").unwrap();
        host_end.deliver(b"frame");
        assert_eq!(device_end.receive(0).as_deref(), Some(&b"frame"[..]));
        device_end.transmit(0, b"reply");
        assert_eq!(host_end.drain(), vec![b"reply".to_vec()]);
        assert_eq!(ports::names(&hosts), ["net0"]);
        assert!(ports::close(&hosts, "net0"));
        assert!(ports::get(&hosts, "net0").unwrap().is_none());
    }

    #[test]
    fn two_builds_with_one_port_name_are_two_ports() {
        let left = crate::core::hosts::HostObjects::new();
        let right = crate::core::hosts::HostObjects::new();
        let a = ports::open(&left, "net0").unwrap();
        let b = ports::open(&right, "net0").unwrap();
        assert!(!Arc::ptr_eq(&a, &b));
        a.deliver(b"only mine");
        assert_eq!(b.pending_input(), 0);
    }

    #[test]
    fn the_multicast_hash_is_a_function_of_the_address() {
        // The data sheet publishes no worked example, so what is asserted is
        // that the function is one: in range, stable, and discriminating.
        for mac in [
            MacAddr::BROADCAST,
            MacAddr::parse("01:00:5e:00:00:01").unwrap(),
            MacAddr::parse("33:33:00:00:00:01").unwrap(),
        ] {
            assert!(mac.multicast_hash() < 64);
        }
        assert_ne!(
            MacAddr::parse("01:00:5e:00:00:01")
                .unwrap()
                .multicast_hash(),
            MacAddr::parse("01:00:5e:00:00:02")
                .unwrap()
                .multicast_hash(),
        );
    }
}
