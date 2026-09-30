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

use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::fmt;

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

/// Two queues: what end `a` has sent and `b` has not read, and the reverse.
pub struct UartLink {
    a_to_b: Mutex<VecDeque<u8>>,
    b_to_a: Mutex<VecDeque<u8>>,
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
        }
    }

    /// One end, as a character device.
    #[must_use]
    pub fn end(self: &Arc<Self>, side: Side) -> Arc<LinkEnd> {
        Arc::new(LinkEnd {
            link: Arc::clone(self),
            side,
        })
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
    fn a_full_direction_pushes_back() {
        let link = Arc::new(UartLink::new());
        let a = link.end(Side::A);
        let big = alloc::vec![0u8; LINK_CAPACITY + 10];
        assert_eq!(a.write(&big), LINK_CAPACITY);
        assert!(!a.writable());
    }
}
