//! The SNES controller seam: what the host holds on the two pads.
//!
//! A standard SNES pad is two 4021-style shift registers chained to sixteen
//! bits, latched by the console's `OUT0` line and clocked out by reading the
//! port — the NES pad grown four buttons and four always-zero bits. Bit order
//! is the order the bits come out, which is the order [`buttons`] numbers
//! them: B first (bit 15), then Y, Select, Start, Up, Down, Left, Right, A, X,
//! L, R, and four zeros (fullsnes, "SNES Controllers"; SNESdev wiki,
//! "Standard controller"). After the sixteenth bit an official pad returns 1
//! for ever.
//!
//! The shift registers themselves, the manual `$4016`/`$4017` reads and the
//! automatic read into `JOY1`–`JOY4` are the S-CPU's
//! ([`scpu`](super::scpu)); this module is only the rendezvous between the
//! host and that device, the same named-port shape as `dev::nes::input::pads`
//! and for the same reason — a name is the only thing that can travel from a
//! machine description into a device constructor, and input crossing into
//! the machine through one narrow, named seam is what makes it recordable.

use crate::core::sync::{AtomicU16, Ordering};

/// The sixteen bits of a pad, in the order the console reads them.
pub mod buttons {
    /// B — the first bit out.
    pub const B: u16 = 0x8000;
    /// Y.
    pub const Y: u16 = 0x4000;
    /// Select.
    pub const SELECT: u16 = 0x2000;
    /// Start.
    pub const START: u16 = 0x1000;
    /// D-pad up.
    pub const UP: u16 = 0x0800;
    /// D-pad down.
    pub const DOWN: u16 = 0x0400;
    /// D-pad left.
    pub const LEFT: u16 = 0x0200;
    /// D-pad right.
    pub const RIGHT: u16 = 0x0100;
    /// A.
    pub const A: u16 = 0x0080;
    /// X.
    pub const X: u16 = 0x0040;
    /// The left shoulder.
    pub const L: u16 = 0x0020;
    /// The right shoulder.
    pub const R: u16 = 0x0010;
    /// Nothing held. Bits 3–0 are the pad's ID and read zero on a standard
    /// controller.
    pub const NONE: u16 = 0x0000;
}

/// What the host holds: the current state of the console's two pads.
///
/// Level, not events, exactly as `dev::nes::input::Pad`: the console samples
/// it when the guest latches, so a button is held for as long as the host
/// leaves its bit set.
#[derive(Debug, Default)]
pub struct Pad {
    /// Controller 1 and controller 2, in [`buttons`] order.
    held: [AtomicU16; 2],
}

impl Pad {
    /// A pad port with nothing held.
    #[must_use]
    pub fn new() -> Pad {
        Pad::default()
    }

    /// Set what controller `port` (0 or 1) is holding. An out-of-range port
    /// is ignored.
    pub fn set(&self, port: usize, held: u16) {
        if let Some(cell) = self.held.get(port) {
            // Bits 3–0 are the controller ID, not buttons: a host cannot hold
            // them.
            cell.store(held & 0xFFF0, Ordering::Relaxed);
        }
    }

    /// What controller `port` is holding; nothing for a port that does not
    /// exist.
    #[must_use]
    pub fn get(&self, port: usize) -> u16 {
        self.held
            .get(port)
            .map_or(buttons::NONE, |c| c.load(Ordering::Relaxed))
    }
}

/// The build's named SNES pad ports.
pub mod pads {
    use super::Pad;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a SNES pad port is filed under. Not `pad`: that kind's object
    /// is the NES's eight-bit [`Pad`](crate::dev::nes::input::Pad) wherever
    /// it is filed, and a lookup under one kind name must find one type.
    pub const KIND: HostKind = HostKind::door("snes-pad", make_sink);

    /// The pad port `name` refers to in `hosts`, creating it on first mention
    /// — the host side of the rendezvous.
    ///
    /// # Errors
    ///
    /// If another kind of host object is already open under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Pad>> {
        hosts.open(KIND, name, Pad::new)
    }

    /// The device side, called from `new(props)`.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Pad>> {
        props.host(KIND, name, Pad::new)
    }

    /// The pad port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Pad>>> {
        hosts.get(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// How many bytes one recorded state is: controller 1 then controller 2,
    /// each sixteen bits little-endian.
    pub const RECORD_BYTES: usize = 4;

    /// The record/replay channel the pads called `name` are pressed through.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that applies a recorded payload to `pad`: groups of
    /// [`RECORD_BYTES`], each the held state of both controllers. A short
    /// trailing group applies the controllers it has whole.
    #[must_use]
    pub fn sink(pad: &Arc<Pad>) -> Arc<dyn InputSink> {
        let pad = Arc::clone(pad);
        Arc::new(FnSink::new("snes-pad", move |payload: &[u8]| {
            for group in payload.chunks(RECORD_BYTES) {
                for (port, held) in group.as_chunks::<2>().0.iter().enumerate() {
                    pad.set(port, u16::from_le_bytes(*held));
                }
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Pad>().ok()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::sync::Arc;

    #[test]
    fn the_id_bits_cannot_be_held() {
        let pad = Pad::new();
        pad.set(0, 0xFFFF);
        assert_eq!(pad.get(0), 0xFFF0);
        pad.set(7, buttons::A);
        assert_eq!(pad.get(7), 0);
    }

    #[test]
    fn a_recorded_payload_sets_both_controllers() {
        let pad = Arc::new(Pad::new());
        let sink = pads::sink(&pad);
        let b = buttons::B | buttons::START;
        let mut payload = alloc::vec::Vec::new();
        payload.extend_from_slice(&b.to_le_bytes());
        payload.extend_from_slice(&buttons::R.to_le_bytes());
        sink.deliver(&payload);
        assert_eq!(pad.get(0), b);
        assert_eq!(pad.get(1), buttons::R);
    }
}
