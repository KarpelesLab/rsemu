//! The character-stream seam: bytes in, bytes out, never blocking.
//!
//! One trait, [`CharDevice`], and one implementation of it, [`CharPort`], that
//! is a pair of queues in memory. A device model holds an `Arc<dyn CharDevice>`
//! and never learns whether the other end is a terminal, a socket, a file, a
//! scripted test, or a browser tab.
//!
//! # Designed for the second user, not the first
//!
//! The Apple 1 PIA is the first thing to hold one of these; a 16550 UART on the
//! RISC-V board is the second, and that one is the reason the surface looks
//! like this rather than like a keyboard:
//!
//! * **Both directions, independently.** A UART transmits and receives at once,
//!   and its status register reports on each half separately.
//! * **Never blocks.** [`CharDevice::read`] returning `0` means "nothing right
//!   now", not "end of stream". A device model runs inside the scheduler; a
//!   blocking read there stops virtual time.
//! * **Back pressure is visible.** [`CharDevice::write`] reports how many bytes
//!   were taken, and [`CharDevice::writable`] answers the question a THRE bit
//!   or an Apple 1 `DA` bit is asking, without a write to find out.
//! * **Bytes, not characters.** No encoding, no line discipline, no echo. A
//!   guest that sends `0x8D` sends `0x8D`; translating that into something a
//!   host terminal wants is the *backend's* job (see [`terminal`]) or the
//!   *device's* (an Apple 1 keyboard is upper-case with bit 7 strapped high),
//!   and never this layer's.
//!
//! [`terminal`]: super::terminal
//!
//! # Ports are opened by name
//!
//! A machine file cannot hand a device a host object: `Props` carries data.
//! What *can* travel from a machine file into a device is a name — which is
//! exactly how a ROM image reaches a cartridge
//! ([`MediaTable`](crate::machine::MediaTable)). So a character port works the
//! same way:
//!
//! ```text
//! machine file:  object pia "apple1.pia" { port = "console" }
//! device:        ports::attach(props, "console")  ──┐
//! host:          ports::open(&hosts, "console")   ──┴─► the same Arc<CharPort>
//! ```
//!
//! [`ports`] used to be a process-wide `static`, which meant two machines built
//! in one process with the same port name shared a keyboard. It is now a view
//! onto the build's own [`HostObjects`](crate::core::hosts::HostObjects), which
//! `RealizeOptions` carries beside its `MediaTable`: the `port` property means
//! exactly what it always meant, and the "give ports distinct names per
//! machine" caveat is gone, because distinct machines have distinct tables.

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::fmt;

/// A bidirectional stream of bytes between a device model and the host.
///
/// Every method is non-blocking and takes `&self`: a device is shared, and the
/// backend keeps whatever interior mutability it needs. `Send + Sync` from the
/// first commit like every other device-facing trait (`ROADMAP.md` §0).
///
/// Directions are named from the *device's* point of view, because that is who
/// implements against this: [`read`](CharDevice::read) takes bytes the host has
/// produced, [`write`](CharDevice::write) offers bytes the guest has produced.
pub trait CharDevice: Send + Sync + fmt::Debug {
    /// Take up to `dst.len()` bytes the host has for the guest.
    ///
    /// Returns how many were taken, which may be `0`. Never blocks, and `0`
    /// never means end-of-stream — a terminal with nobody typing at it is not
    /// closed.
    fn read(&self, dst: &mut [u8]) -> usize;

    /// Offer `src` to the host, returning how many bytes were accepted.
    ///
    /// A short write is normal and the caller must cope: the byte at
    /// `src[accepted]` has *not* been sent. Never blocks.
    fn write(&self, src: &[u8]) -> usize;

    /// Whether [`write`](CharDevice::write) would accept at least one byte.
    ///
    /// This is what a transmitter-empty status bit is really asking. The
    /// default says yes, which is right for any backend that cannot refuse.
    fn writable(&self) -> bool {
        true
    }

    /// Push anything the backend is holding on toward the host.
    ///
    /// Defaults to doing nothing, which is right for a backend that buffers
    /// nothing.
    fn flush(&self) {}

    /// One byte from the host, if there is one.
    fn read_byte(&self) -> Option<u8> {
        let mut byte = [0u8; 1];
        (self.read(&mut byte) == 1).then_some(byte[0])
    }

    /// Offer one byte, reporting whether it was accepted.
    fn write_byte(&self, byte: u8) -> bool {
        self.write(&[byte]) == 1
    }
}

/// The number of bytes a [`CharPort`] will hold in each direction before it
/// starts refusing writes.
///
/// Large enough that a screenful of output never trips it, small enough that a
/// guest spinning in a print loop with nobody draining the other end cannot
/// grow the heap without bound. Back pressure is the point: a `CharDevice` that
/// silently buffers forever tells the device model a lie about the hardware.
pub const PORT_CAPACITY: usize = 64 * 1024;

/// Two byte queues with a lock around them: the reference [`CharDevice`].
///
/// This is the deterministic end of the seam and the one every test uses. It is
/// also what a real backend attaches *to*: [`Terminal::pump`] moves bytes
/// between the process's stdin/stdout and one of these, so nothing in the
/// scheduler ever performs a system call.
///
/// [`Terminal::pump`]: super::terminal::Terminal::pump
pub struct CharPort {
    state: crate::core::sync::Mutex<PortState>,
}

/// The two queues. Separate struct so one lock covers both, which keeps
/// "read what the guest wrote, then feed it a reply" atomic for a test.
#[derive(Debug, Default)]
struct PortState {
    /// Host to guest: what [`CharDevice::read`] hands out.
    to_guest: VecDeque<u8>,
    /// Guest to host: what [`CharDevice::write`] appends to.
    to_host: VecDeque<u8>,
}

impl fmt::Debug for CharPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.state.try_lock() {
            Some(state) => f
                .debug_struct("CharPort")
                .field("to_guest", &state.to_guest.len())
                .field("to_host", &state.to_host.len())
                .finish(),
            None => f
                .debug_struct("CharPort")
                .field("state", &"<in use>")
                .finish(),
        }
    }
}

impl Default for CharPort {
    fn default() -> Self {
        CharPort::new()
    }
}

impl CharPort {
    /// An empty port.
    #[must_use]
    pub fn new() -> CharPort {
        CharPort {
            // A leaf: nothing is ever locked while this one is held, so it
            // nests under a device's own state lock and under the bus.
            state: crate::core::sync::Mutex::new(PortState::default()),
        }
    }

    // -- the host's side ---------------------------------------------------

    /// Host side: give the guest bytes to read, returning how many were taken.
    ///
    /// Short when the queue is within [`PORT_CAPACITY`] of full, which for
    /// input means a user typing faster than the guest polls — dropping the
    /// overflow is what a real keyboard with one latch does anyway.
    pub fn feed(&self, bytes: &[u8]) -> usize {
        let mut state = self.state.lock();
        let room = PORT_CAPACITY.saturating_sub(state.to_guest.len());
        let take = room.min(bytes.len());
        state.to_guest.extend(&bytes[..take]);
        take
    }

    /// Host side: take everything the guest has emitted so far.
    #[must_use]
    pub fn drain(&self) -> Vec<u8> {
        let mut out = Vec::new();
        self.drain_into(&mut out);
        out
    }

    /// Host side: append everything the guest has emitted to `dst`.
    ///
    /// The allocation-free form, for a pump that runs many times a second.
    pub fn drain_into(&self, dst: &mut Vec<u8>) {
        let mut state = self.state.lock();
        dst.reserve(state.to_host.len());
        dst.extend(state.to_host.drain(..));
    }

    /// Host side: how many bytes the guest has emitted and nobody has taken.
    #[must_use]
    pub fn pending_output(&self) -> usize {
        self.state.lock().to_host.len()
    }

    /// Host side: how many bytes are waiting for the guest to read.
    #[must_use]
    pub fn pending_input(&self) -> usize {
        self.state.lock().to_guest.len()
    }

    /// Discard both queues.
    pub fn clear(&self) {
        let mut state = self.state.lock();
        state.to_guest.clear();
        state.to_host.clear();
    }
}

impl CharDevice for CharPort {
    fn read(&self, dst: &mut [u8]) -> usize {
        let mut state = self.state.lock();
        let mut taken = 0;
        while taken < dst.len() {
            match state.to_guest.pop_front() {
                Some(byte) => {
                    dst[taken] = byte;
                    taken += 1;
                }
                None => break,
            }
        }
        taken
    }

    fn write(&self, src: &[u8]) -> usize {
        let mut state = self.state.lock();
        let room = PORT_CAPACITY.saturating_sub(state.to_host.len());
        let take = room.min(src.len());
        state.to_host.extend(&src[..take]);
        take
    }

    fn writable(&self) -> bool {
        self.state.lock().to_host.len() < PORT_CAPACITY
    }
}

/// The build's named character ports.
///
/// See the module docs for why a *name* is the only thing that can travel from
/// a machine description into a device constructor, and
/// [`core::hosts`](crate::core::hosts) for the table this is a view onto.
pub mod ports {
    use super::CharPort;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use core::any::Any;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a character port is filed under in a build's
    /// [`HostObjects`].
    ///
    /// A [door](HostKind::door): what a person types at a console is
    /// non-deterministic input, and this is the object it crosses at. The
    /// factory is what lets
    /// [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) wire a
    /// console to a recorder without the caller having to know the port's name
    /// — which is exactly what `rsemu run … --record-input` cannot know, since
    /// the name comes out of the machine file.
    pub const KIND: HostKind = HostKind::door("chardev", make_sink);

    /// The a character port `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous: called before the host starts
    /// pumping bytes, or after the build to pick up what a device opened.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name, which is a collision between two host modules rather
    /// than anything a machine file can cause.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<CharPort>> {
        hosts.open(KIND, name, CharPort::new)
    }

    /// The a character port `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)` — acquiring a host object
    /// is allocation, and [`core::hosts`](crate::core::hosts) argues why. A
    /// `Props` that belongs to no build gets a private one, so a device a unit
    /// test constructed directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<CharPort>> {
        props.host(KIND, name, CharPort::new)
    }

    /// The a character port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<CharPort>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    ///
    /// Anything still holding the `Arc` keeps working; this only removes the
    /// table's own reference, so a later [`open`] of the same name is a fresh
    /// one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// The record/replay channel the port called `name` receives on.
    ///
    /// `chardev:console`, which is the same `(kind, name)` pair the host-object
    /// table files the port under — so a board whose console has no channel is
    /// named by this string when
    /// [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) refuses it.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that feeds a recorded payload to `port`.
    ///
    /// The whole adapter between [`core::record`](crate::core::record) and this
    /// module, and the ten lines whose absence meant the CLI's own console was
    /// unrecorded while a test that wrote these two closures by hand recorded
    /// one fine (`core::record`, "what a *new* device has to do").
    ///
    /// The rewind hook matters: bytes queued at the rewind target are
    /// re-delivered from the log on the way forward, so a port that kept them
    /// would hand the guest each one twice.
    #[must_use]
    pub fn sink(port: &Arc<CharPort>) -> Arc<dyn InputSink> {
        let feeding = Arc::clone(port);
        let clearing = Arc::clone(port);
        Arc::new(
            FnSink::new("chardev", move |bytes: &[u8]| {
                feeding.feed(bytes);
            })
            .on_rewind(move || clearing.clear()),
        )
    }

    /// [`sink`], reached through the erased handle the host-object table holds.
    ///
    /// What [`KIND`] carries so a sealed table can wire a console it was never
    /// told about. `None` means something that is not a [`CharPort`] is filed
    /// under `chardev`, which is two modules claiming one kind name.
    fn make_sink(object: &Arc<dyn Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<CharPort>().ok()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::sync::Arc;

    #[test]
    fn bytes_cross_in_both_directions_without_meeting() {
        let port = CharPort::new();
        assert_eq!(port.feed(b"hi"), 2);
        assert_eq!(port.write(b"there"), 5);

        // The guest reads what the host fed, not what the guest wrote.
        let mut buf = [0u8; 8];
        assert_eq!(port.read(&mut buf), 2);
        assert_eq!(&buf[..2], b"hi");
        assert_eq!(
            port.read(&mut buf),
            0,
            "nothing left, and it does not block"
        );

        // And the host reads what the guest wrote.
        assert_eq!(port.drain(), b"there".to_vec());
        assert!(port.drain().is_empty());
    }

    #[test]
    fn a_byte_at_a_time_is_the_same_stream() {
        let port = CharPort::new();
        port.feed(b"AB");
        assert_eq!(port.read_byte(), Some(b'A'));
        assert_eq!(port.read_byte(), Some(b'B'));
        assert_eq!(port.read_byte(), None);
        assert!(port.write_byte(b'C'));
        assert_eq!(port.drain(), b"C".to_vec());
    }

    #[test]
    fn a_full_port_pushes_back_rather_than_growing() {
        let port = CharPort::new();
        let flood = alloc::vec![b'x'; PORT_CAPACITY + 10];
        assert_eq!(port.write(&flood), PORT_CAPACITY, "the write is short");
        assert!(!port.writable(), "and it says so");
        assert_eq!(port.write(b"y"), 0);
        assert_eq!(port.pending_output(), PORT_CAPACITY);
        // Draining makes room again.
        assert_eq!(port.drain().len(), PORT_CAPACITY);
        assert!(port.writable());
        assert_eq!(port.feed(&flood), PORT_CAPACITY, "input pushes back too");
    }

    #[test]
    fn clearing_a_port_empties_both_queues() {
        let port = CharPort::new();
        port.feed(b"in");
        port.write(b"out");
        port.clear();
        assert_eq!(port.pending_input(), 0);
        assert_eq!(port.pending_output(), 0);
    }

    #[test]
    fn a_name_reaches_the_same_port_from_both_ends() {
        // The whole point of the table: two callers that never meet.
        let hosts = crate::core::hosts::HostObjects::new();
        let device_end: Arc<dyn CharDevice> = ports::open(&hosts, "console").unwrap();
        let host_end = ports::open(&hosts, "console").unwrap();
        host_end.feed(b"Q");
        assert_eq!(device_end.read_byte(), Some(b'Q'));
        device_end.write_byte(b'R');
        assert_eq!(host_end.drain(), b"R".to_vec());

        assert_eq!(ports::names(&hosts), ["console"]);
        assert!(ports::close(&hosts, "console"));
        assert!(ports::get(&hosts, "console").unwrap().is_none());
        // The Arc outlives the table entry, and a re-open is a fresh port.
        host_end.feed(b"S");
        assert_eq!(ports::open(&hosts, "console").unwrap().pending_input(), 0);
    }

    #[test]
    fn two_builds_with_one_port_name_are_two_ports() {
        // What the process-wide `static` this replaced could not do.
        let left = crate::core::hosts::HostObjects::new();
        let right = crate::core::hosts::HostObjects::new();
        let a = ports::open(&left, "console").unwrap();
        let b = ports::open(&right, "console").unwrap();
        assert!(!Arc::ptr_eq(&a, &b));
        a.feed(b"only mine");
        assert_eq!(b.pending_input(), 0);
    }
}
