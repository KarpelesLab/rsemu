//! A serial line between two devices in the same machine.
//!
//! A UART's `port` names a host character port: a terminal, a capture file,
//! the discard pile. That is right for a console and wrong for a board whose
//! serial line runs to another chip on the same board — a sub-processor, a
//! modem, a GPS receiver — because a host port is where *outside* input
//! crosses into the machine (and is recorded, and is drained and thrown away
//! when nobody watches it). Two chips on one board meeting over a wire are
//! neither.
//!
//! A [`UartLink`] is that wire: two ends, `a` and `b`, each a
//! [`CharDevice`], with what one end writes arriving at the other's read. A
//! machine file names the link on both devices (`link = "psc"`, `side = "a"`
//! on one and `"b"` on the other) and they meet through the build's host
//! object table as a [rendezvous](crate::core::hosts::HostKind::rendezvous) —
//! nothing non-deterministic crosses it.
//!
//! # Back pressure
//!
//! Each direction holds [`LINK_CAPACITY`] bytes and then refuses, so a
//! transmitter whose peer is not reading sees its FIFO stay full, as a UART
//! with flow control does. No framing, no baud rate: bytes.
//!
//! # Clocked-synchronous peers
//!
//! On a synchronous serial line the master's clock moves a byte each way at
//! once: every byte end `a` sends clocks one back. A peer that works that way
//! installs a [`Responder`] on end `b`, and a write on end `a` then calls it
//! byte by byte and queues each reply for end `a` to read **before the write
//! returns**. Polling from its own run slice instead would put the reply a
//! scheduler round or two behind the byte, which is later than a master that
//! times its slave's answer will wait for.
//!
//! # Snapshots
//!
//! Bytes one end has written and the other has not read are machine state —
//! they are on the board's own wire, not in a host's buffer — and the link is
//! not a device: it has no chunk, and a restored machine starts with a fresh
//! one that holds nothing. So **each end carries the bytes on their way to
//! it** in its own device's snapshot ([`LinkEnd::save_in_flight`],
//! [`LinkEnd::load_in_flight`]). Two ends, two directions, each saved exactly
//! once, and neither device has to know what the other is. It is the
//! arrangement `bus::i2c` and `bus::spi` use for an open transaction, and for
//! the same reason.

use alloc::collections::VecDeque;
use alloc::format;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::state::{Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::host::chardev::CharDevice;

/// How many bytes each direction holds before it pushes back.
pub const LINK_CAPACITY: usize = 4096;

/// Which end of a link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// End `a`.
    A,
    /// End `b`.
    B,
}

impl Side {
    /// Parse the `side` property.
    #[must_use]
    pub fn parse(s: &str) -> Option<Side> {
        match s {
            "a" => Some(Side::A),
            "b" => Some(Side::B),
            _ => None,
        }
    }
}

/// A clocked-synchronous peer on end `b`: answers each byte end `a` sends.
pub trait Responder: Send + Sync + fmt::Debug {
    /// End `a` clocked out `byte`; return the byte clocked back.
    ///
    /// Called from inside end `a`'s write, so with the transmitting device's
    /// own lock held: take only a [`LockRank::LEAF`] lock here, and make no
    /// outward call (drive a wire from the device's own run slice instead).
    fn clock(&self, byte: u8) -> u8;
}

/// Two queues: what end `a` has sent and `b` has not read, and the reverse.
pub struct UartLink {
    a_to_b: Mutex<VecDeque<u8>>,
    b_to_a: Mutex<VecDeque<u8>>,
    responder: Mutex<Option<Arc<dyn Responder>>>,
}

impl fmt::Debug for UartLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UartLink")
            .field("a_to_b", &self.a_to_b.lock().len())
            .field("b_to_a", &self.b_to_a.lock().len())
            .finish()
    }
}

impl Default for UartLink {
    fn default() -> UartLink {
        UartLink::new()
    }
}

impl UartLink {
    /// An idle link.
    #[must_use]
    pub fn new() -> UartLink {
        UartLink {
            a_to_b: Mutex::with_rank(LockRank::LEAF, VecDeque::new()),
            b_to_a: Mutex::with_rank(LockRank::LEAF, VecDeque::new()),
            responder: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// Make end `b` a clocked-synchronous peer: from now on each byte end `a`
    /// writes is handed to `responder` at once, and its answer is what end
    /// `a` reads. Nothing end `a` writes reaches end `b`'s queue any more.
    pub fn respond_on_b(&self, responder: Arc<dyn Responder>) {
        *self.responder.lock() = Some(responder);
    }

    /// One end, as a character device.
    #[must_use]
    pub fn end(self: &Arc<Self>, side: Side) -> Arc<LinkEnd> {
        Arc::new(LinkEnd {
            link: Arc::clone(self),
            side,
        })
    }

    /// The bytes on their way to `side`: written by the other end and not yet
    /// read by this one. For a snapshot; see the module documentation.
    #[must_use]
    pub fn in_flight(&self, side: Side) -> Vec<u8> {
        let (inbound, _) = self.queues(side);
        inbound.lock().iter().copied().collect()
    }

    /// Put the bytes on their way to `side` back where a snapshot found them,
    /// replacing whatever is queued. Nobody is told: the end reads them when
    /// it next reads, exactly as it would have.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if there are more than a direction holds. A link
    /// never queues more than [`LINK_CAPACITY`], so a snapshot that says it
    /// did was not written by one.
    pub fn restore_in_flight(&self, side: Side, bytes: &[u8]) -> Result<()> {
        if bytes.len() > LINK_CAPACITY {
            return Err(Error::State(format!(
                "a snapshot has {} byte(s) in flight on a serial link that holds {LINK_CAPACITY}",
                bytes.len()
            )));
        }
        let (inbound, _) = self.queues(side);
        let mut q = inbound.lock();
        q.clear();
        q.extend(bytes);
        Ok(())
    }

    fn queues(&self, side: Side) -> (&Mutex<VecDeque<u8>>, &Mutex<VecDeque<u8>>) {
        match side {
            // (inbound, outbound)
            Side::A => (&self.b_to_a, &self.a_to_b),
            Side::B => (&self.a_to_b, &self.b_to_a),
        }
    }
}

/// One end of a [`UartLink`].
pub struct LinkEnd {
    link: Arc<UartLink>,
    side: Side,
}

impl fmt::Debug for LinkEnd {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LinkEnd").field("side", &self.side).finish()
    }
}

impl LinkEnd {
    /// Write the bytes on their way to this end into its device's chunk.
    ///
    /// # Errors
    ///
    /// Whatever the sink reports.
    pub fn save_in_flight<S: Sink + ?Sized>(&self, w: &mut S) -> Result<()> {
        w.write_bytes(&self.link.in_flight(self.side))
    }

    /// Read back what [`save_in_flight`](LinkEnd::save_in_flight) wrote.
    ///
    /// # Errors
    ///
    /// As [`UartLink::restore_in_flight`], or whatever the source reports.
    pub fn load_in_flight<'a, S: Source<'a> + ?Sized>(&self, r: &mut S) -> Result<()> {
        let bytes = r.read_bytes()?;
        self.link.restore_in_flight(self.side, bytes)
    }
}

/// [`LinkEnd::save_in_flight`] for a device that may be on a host port
/// instead: no link is no bytes, and the chunk has the same shape either way.
///
/// # Errors
///
/// Whatever the sink reports.
pub fn save_in_flight<S: Sink + ?Sized>(end: Option<&LinkEnd>, w: &mut S) -> Result<()> {
    match end {
        Some(end) => end.save_in_flight(w),
        None => w.write_bytes(&[]),
    }
}

/// [`LinkEnd::load_in_flight`] for a device that may be on a host port.
///
/// # Errors
///
/// [`Error::State`] if the snapshot carries bytes in flight and this device
/// is on no link to put them on — it was taken from a board wired
/// differently.
pub fn load_in_flight<'a, S: Source<'a> + ?Sized>(end: Option<&LinkEnd>, r: &mut S) -> Result<()> {
    match end {
        Some(end) => end.load_in_flight(r),
        None => {
            let bytes = r.read_bytes()?;
            if bytes.is_empty() {
                Ok(())
            } else {
                Err(Error::State(format!(
                    "a snapshot has {} byte(s) in flight on a serial link, and this device is \
                     on a host port",
                    bytes.len()
                )))
            }
        }
    }
}

impl CharDevice for LinkEnd {
    fn read(&self, dst: &mut [u8]) -> usize {
        let (inbound, _) = self.link.queues(self.side);
        let mut q = inbound.lock();
        let n = dst.len().min(q.len());
        for b in dst.iter_mut().take(n) {
            *b = q.pop_front().unwrap_or(0);
        }
        n
    }

    fn write(&self, src: &[u8]) -> usize {
        if self.side == Side::A {
            let responder = self.link.responder.lock().clone();
            if let Some(responder) = responder {
                let mut sent = 0;
                for &byte in src {
                    if self.link.b_to_a.lock().len() >= LINK_CAPACITY {
                        break;
                    }
                    let reply = responder.clock(byte);
                    self.link.b_to_a.lock().push_back(reply);
                    sent += 1;
                }
                return sent;
            }
        }
        let (_, outbound) = self.link.queues(self.side);
        let mut q = outbound.lock();
        let n = src.len().min(LINK_CAPACITY - q.len());
        q.extend(&src[..n]);
        n
    }

    fn writable(&self) -> bool {
        let (_, outbound) = self.link.queues(self.side);
        outbound.lock().len() < LINK_CAPACITY
    }
}

/// Links opened by name in a build's host-object table.
pub mod links {
    use super::UartLink;
    use alloc::sync::Arc;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;

    /// The kind a UART link is filed under.
    pub const KIND: HostKind = HostKind::rendezvous("uart-link");

    /// The link `name` in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// If another kind of host object holds the name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<UartLink>> {
        hosts.open(KIND, name, UartLink::new)
    }

    /// The link `name` in the build these properties come from.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<UartLink>> {
        props.host(KIND, name, UartLink::new)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    struct Echo;
    impl Responder for Echo {
        fn clock(&self, byte: u8) -> u8 {
            !byte
        }
    }

    #[test]
    fn a_synchronous_peer_answers_inside_the_write() {
        let link = Arc::new(UartLink::new());
        let (a, b) = (link.end(Side::A), link.end(Side::B));
        link.respond_on_b(Arc::new(Echo));
        assert_eq!(a.write(&[0x0f, 0x55]), 2);
        let mut got = [0u8; 4];
        assert_eq!(a.read(&mut got), 2, "already there");
        assert_eq!(&got[..2], [0xf0, 0xaa]);
        assert_eq!(b.read(&mut got), 0, "nothing queued for end b");
    }

    #[test]
    fn what_one_end_writes_the_other_reads() {
        let link = Arc::new(UartLink::new());
        let (a, b) = (link.end(Side::A), link.end(Side::B));
        assert_eq!(a.write(b"ping"), 4);
        let mut buf = [0u8; 8];
        assert_eq!(b.read(&mut buf), 4);
        assert_eq!(&buf[..4], b"ping");
        assert_eq!(a.read(&mut buf), 0, "nothing came back");
        assert!(b.write_byte(0x06));
        assert_eq!(a.read_byte(), Some(0x06));
    }

    #[test]
    fn bytes_in_flight_are_each_ends_own_to_save_and_restore() {
        let link = Arc::new(UartLink::new());
        let (a, b) = (link.end(Side::A), link.end(Side::B));
        assert_eq!(a.write(b"to b"), 4);
        assert_eq!(b.write(b"to a!"), 5);
        // Each end holds what is on its way *to* it.
        assert_eq!(link.in_flight(Side::A), b"to a!");
        assert_eq!(link.in_flight(Side::B), b"to b");
        let (mut for_a, mut for_b) = (Vec::new(), Vec::new());
        a.save_in_flight(&mut for_a).unwrap();
        b.save_in_flight(&mut for_b).unwrap();

        // A restored machine has a fresh link with nothing on it.
        let fresh = Arc::new(UartLink::new());
        let (a2, b2) = (fresh.end(Side::A), fresh.end(Side::B));
        a2.load_in_flight(&mut crate::core::state::SliceSource::new(&for_a))
            .unwrap();
        b2.load_in_flight(&mut crate::core::state::SliceSource::new(&for_b))
            .unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(a2.read(&mut buf), 5);
        assert_eq!(&buf[..5], b"to a!");
        assert_eq!(b2.read(&mut buf), 4);
        assert_eq!(&buf[..4], b"to b");

        // Restoring replaces: it is a snapshot's view, not an append.
        fresh.restore_in_flight(Side::A, b"xy").unwrap();
        fresh.restore_in_flight(Side::A, b"z").unwrap();
        assert_eq!(fresh.in_flight(Side::A), b"z");
    }

    #[test]
    fn more_bytes_in_flight_than_a_direction_holds_is_refused() {
        let link = UartLink::new();
        let too_many = alloc::vec![0u8; LINK_CAPACITY + 1];
        assert!(link.restore_in_flight(Side::B, &too_many).is_err());
        assert!(link.in_flight(Side::B).is_empty(), "and nothing was queued");
        let full = alloc::vec![7u8; LINK_CAPACITY];
        link.restore_in_flight(Side::B, &full).unwrap();
        assert_eq!(link.in_flight(Side::B).len(), LINK_CAPACITY);
    }

    #[test]
    fn a_device_on_a_host_port_saves_no_bytes_and_refuses_any() {
        let mut none = Vec::new();
        save_in_flight(None, &mut none).unwrap();
        load_in_flight(None, &mut crate::core::state::SliceSource::new(&none)).unwrap();

        let link = Arc::new(UartLink::new());
        let (a, b) = (link.end(Side::A), link.end(Side::B));
        a.write(b"hi");
        let mut some = Vec::new();
        save_in_flight(Some(&b), &mut some).unwrap();
        assert!(
            load_in_flight(None, &mut crate::core::state::SliceSource::new(&some)).is_err(),
            "bytes on a link this device is not on"
        );
    }

    #[test]
    fn a_full_direction_pushes_back() {
        let link = Arc::new(UartLink::new());
        let a = link.end(Side::A);
        let big = alloc::vec![0u8; LINK_CAPACITY + 10];
        assert_eq!(a.write(&big), LINK_CAPACITY);
        assert!(!a.writable());
    }
}
