//! A VNC server: the RFB protocol over TCP (`ROADMAP.md` §8, Phase 9).
//!
//! ```console
//! $ rsemu run pc-at --vnc :5900
//! $ vncviewer 127.0.0.1:5900
//! ```
//!
//! This is the frontend §8 calls "the highest-value" one, and the reason is the
//! dependency policy: a window needs a GUI toolkit and `CLAUDE.md` forbids
//! every one of them, whereas a VNC server needs a socket, a framebuffer and a
//! keyboard table. Everything a person needs to *watch* a machine and *type at*
//! it is already in the tree; this module is the wire between them.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`proto`] | the RFB messages, byte for byte, with their RFC sections |
//! | [`frame`] | a [`Surface`] as a FramebufferUpdate, with damage |
//! | [`session`] | a [`Machine`](crate::machine::Machine) driven behind one |
//! | this module | the listener, the connections, and [`VncServer::poll`] |
//!
//! The split is the gdbstub's, for the gdbstub's reason: [`proto`] owns no
//! socket, [`frame`] owns no connection, and only this module needs both — so
//! the protocol is tested against a `Vec<u8>` and the encoder against a
//! `Surface`, with no port bound.
//!
//! # Threads: none
//!
//! A VNC server would like a thread per connection and does not get one.
//! `CLAUDE.md` is explicit — submit jobs, never spawn threads; wasm cannot make
//! a worker synchronously — so every socket here is non-blocking and
//! [`VncServer::poll`] is one turn of a loop the *caller* owns. That caller is
//! also what advances the machine, which is what makes the next section
//! possible.
//!
//! # Determinism: input arrives at an instant the scheduler chose
//!
//! A client sends a key event whenever a human presses a key. That is wall-clock
//! time, and a guest may not observe it. So [`VncServer::poll`] does not deliver
//! anything: it *collects*, and hands the caller a batch of
//! [`InputEvent`]s. The caller delivers them at
//! the top of a slice — an instant the scheduler chose — and records
//! `(machine.now(), event)` in an [`InputLog`](crate::host::input::InputLog).
//! Replaying that log at the same instants reproduces the run exactly;
//! `tests/vnc_input.rs` asserts the state hashes match.
//!
//! That is the same shape the NE2000 uses for an arriving frame, and it is the
//! shape the general record/replay seam should absorb. **What that seam has to
//! offer for this module to drop its own log:**
//!
//! 1. **A named stream.** `machine.record_stream("vnc.input")` returning
//!    something a frontend appends opaque byte records to. The name is the
//!    source; the bytes are that source's business
//!    ([`InputEvent::encode`](crate::host::input::InputEvent::encode) is
//!    already a fixed twelve of them).
//! 2. **A virtual timestamp per record, supplied by the machine**, not by the
//!    caller — `now()` read at the moment of the append, so a frontend cannot
//!    record an instant the machine was never at.
//! 3. **Replay that hands records back at the same instant, before the slice
//!    that follows it.** The delivery point has to be a scheduling boundary the
//!    replay controls, or a record logged at *t* is applied at *t + slice* and
//!    the run diverges. A `Machine::run_until` that stops at the next pending
//!    record's instant would do it; so would a callback the scheduler invokes.
//! 4. **A stable tie-break for records at the same instant** — insertion order,
//!    like the scheduler's own sequence counter. Two keys in one poll are
//!    common.
//! 5. **The cursor in the snapshot.** Rewind restores a snapshot and replays
//!    forward, and a replay that restarts its log from the beginning replays
//!    every keystroke of the run twice.
//!
//! Points 1 to 4 are what [`input`](crate::host::input) implements privately
//! today. Point 5 is the one it cannot do alone, and it is why this log is a
//! separate file rather than a snapshot chunk: nothing here can make the
//! cursor part of machine state.
//!
//! # Security
//!
//! Security type `None` (RFC 6143 §7.2.1) and nothing else, on the loopback
//! interface unless an address says otherwise ([`listen`]).
//! VNC Authentication (§7.2.2) is a DES challenge over a password truncated to
//! eight characters; implementing it would invite someone to rely on it. A
//! session that has to cross a network belongs in an SSH tunnel, which is what
//! everyone does with VNC anyway.
//!
//! # Provenance
//!
//! RFC 6143, "The Remote Framebuffer Protocol", and nothing else. Every message
//! in [`proto`] cites the section that defines it. No VNC implementation was
//! consulted: TightVNC, TigerVNC, x11vnc and QEMU's server are GPL and off
//! limits (`ROADMAP.md` §1), and LibVNCServer's LGPL permits linking rather
//! than copying. Running a client against this server is black-box use and is
//! fine.

pub mod frame;
pub mod proto;
pub mod session;

pub use session::VncSession;

use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use crate::host::display::Surface;
use crate::host::input::{InputEvent, Keysym};
use crate::host::listen;

use frame::FrameEncoder;
use proto::{ClientMessage, Parsed, PixelFormat, Version};

/// How many clients may watch one machine at once.
///
/// More than one, unlike the gdbstub — two debuggers fighting over a `continue`
/// is a bug, two people watching the same screen is a demo. The cap exists
/// because each connection costs a copy of the framebuffer, and an unbounded
/// number of them is a way to exhaust memory from the network.
pub const MAX_CLIENTS: usize = 8;

/// How many bytes of un-sent update a connection may accumulate before the
/// server stops producing new ones for it.
///
/// A client that stops reading — a laptop that went to sleep with a viewer
/// open — must not be able to grow the emulator's heap without bound. Past this
/// point its outstanding request simply stays outstanding, and it gets a whole
/// frame when it starts reading again, which is what it wants anyway.
const MAX_PENDING: usize = 8 * 1024 * 1024;

/// How many bytes of a half-arrived client message may be held before the
/// connection is closed.
///
/// A client message has no length prefix except ClientCutText's (§7.5.6), which
/// is a `u32` — so a peer that says "here comes four gigabytes of clipboard"
/// and then stops writing would otherwise have the server hold the allocation
/// for it. Sixty-four kilobytes is far more than any real message: the largest
/// this server can be sent is a SetEncodings naming every encoding twice over.
/// Nothing here is authenticated, so the limit is not a nicety.
const MAX_INBOX: usize = 64 * 1024;

/// The desktop name a client shows in its title bar, when the caller names
/// nothing better.
const DEFAULT_NAME: &str = "rsemu";

/// Where one connection has got to in the handshake (RFC 6143 §7.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// ProtocolVersion sent; waiting for the client's (§7.1.1).
    Version,
    /// Security types sent; waiting for the client's choice (§7.1.2).
    Security,
    /// Waiting for ClientInit (§7.3.1).
    Init,
    /// Handshake done: normal messages (§7.5).
    Ready,
}

/// One accepted connection.
#[derive(Debug)]
struct Conn {
    stream: TcpStream,
    peer: Option<SocketAddr>,
    phase: Phase,
    version: Version,
    /// Bytes read from the socket and not yet consumed by a message.
    inbox: Vec<u8>,
    /// Bytes produced and not yet accepted by the socket.
    pending: Vec<u8>,
    encoder: FrameEncoder,
    /// An outstanding non-incremental FramebufferUpdateRequest.
    wants_full: bool,
    /// An outstanding incremental one.
    wants_incremental: bool,
    /// Whether the client asked for a shared session (§7.3.1). Recorded rather
    /// than acted on: this server never disconnects anybody for it, because
    /// deciding who gets thrown off a screen is not the protocol's business.
    shared: bool,
}

impl Conn {
    /// The pixel format this connection is being sent.
    fn format(&self) -> PixelFormat {
        self.encoder.format()
    }
}

/// An RFB server listening on a TCP port.
///
/// Owns no machine and no scanout: it is handed a [`Surface`] each poll and
/// gives back whatever the clients typed. [`session`] is what wires it to a
/// machine, and a test that wants neither can drive this directly.
#[derive(Debug)]
pub struct VncServer {
    listener: TcpListener,
    conns: Vec<Conn>,
    name: String,
    /// The geometry a newly accepted client is told about in ServerInit.
    geometry: (u16, u16),
}

impl VncServer {
    /// Bind a listener.
    ///
    /// `addr` may be `5900`, `:5900`, `host:5900` or `[::1]:5900`. **A bare
    /// port or a leading colon binds the loopback interface only** — see
    /// [`listen`] for why that is not negotiable.
    ///
    /// # Errors
    ///
    /// An address that does not parse or resolve, or a port that cannot be
    /// bound.
    pub fn bind(addr: &str) -> std::io::Result<VncServer> {
        Ok(VncServer {
            listener: listen::bind(addr)?,
            conns: Vec::new(),
            name: String::from(DEFAULT_NAME),
            geometry: (1, 1),
        })
    }

    /// Name the desktop, which is what a viewer puts in its title bar.
    #[must_use]
    pub fn named(mut self, name: &str) -> VncServer {
        self.name = name.to_string();
        self
    }

    /// Tell newly accepted clients the framebuffer is this size.
    ///
    /// ServerInit carries the geometry once (§7.3.2), before a client has said
    /// anything, so the server has to know it before the first frame. A client
    /// already connected is unaffected: it learns about a resize through the
    /// DesktopSize pseudo-encoding, or not at all — see [`frame`].
    pub fn set_geometry(&mut self, width: u32, height: u32) {
        self.geometry = (clamp16(width).max(1), clamp16(height).max(1));
    }

    /// The address actually bound, which is how a test finds the ephemeral port
    /// it asked for with `:0`.
    ///
    /// # Errors
    ///
    /// Whatever the operating system says about the socket.
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    /// How many clients are connected.
    #[must_use]
    pub fn clients(&self) -> usize {
        self.conns.len()
    }

    /// Whether anybody is watching.
    #[must_use]
    pub fn is_watched(&self) -> bool {
        !self.conns.is_empty()
    }

    /// The connected clients' addresses, for a status line.
    #[must_use]
    pub fn peers(&self) -> Vec<SocketAddr> {
        self.conns.iter().filter_map(|c| c.peer).collect()
    }

    /// One turn: accept, read, answer, and report what was typed.
    ///
    /// `surface` is the machine's current frame. Nothing here advances virtual
    /// time and nothing here delivers an event to a device — the returned
    /// events are the caller's to deliver, at an instant it chooses, so that
    /// the instant is part of the machine's history rather than of the network's
    /// (see the module docs).
    ///
    /// Never blocks. A connection that fails is dropped rather than propagated:
    /// one viewer hanging up must not end the run.
    ///
    /// # Errors
    ///
    /// Only a failure of the listener itself. A per-connection error closes
    /// that connection.
    pub fn poll(&mut self, surface: &Surface) -> std::io::Result<Vec<InputEvent>> {
        self.accept()?;
        let mut events = Vec::new();
        let mut i = 0;
        while i < self.conns.len() {
            if Self::service(&mut self.conns[i], surface, &self.name, &mut events) {
                i += 1;
            } else {
                self.conns.remove(i);
            }
        }
        Ok(events)
    }

    /// Ring every connected client's bell (§7.6.3).
    ///
    /// Nothing calls this yet — a PC speaker would. It is here because the
    /// message is three lines and leaving it out would mean the next person has
    /// to re-read the RFC to add it.
    pub fn bell(&mut self) {
        for conn in &mut self.conns {
            conn.pending.extend_from_slice(&proto::bell());
        }
    }

    /// Take any waiting connections.
    fn accept(&mut self) -> std::io::Result<()> {
        loop {
            match self.listener.accept() {
                Ok((stream, peer)) => {
                    if self.conns.len() >= MAX_CLIENTS {
                        // Hanging up is kinder than accepting and never
                        // answering: the viewer says "connection refused"
                        // rather than hanging on a handshake.
                        drop(stream);
                        continue;
                    }
                    stream.set_nonblocking(true)?;
                    // A framebuffer update is latency-sensitive and often
                    // small; Nagle's algorithm holds the last packet of one
                    // back waiting for a reply that is never coming.
                    let _ = stream.set_nodelay(true);
                    let (width, height) = self.geometry;
                    let mut conn = Conn {
                        stream,
                        peer: Some(peer),
                        phase: Phase::Version,
                        version: Version::V3_8,
                        inbox: Vec::new(),
                        pending: Vec::new(),
                        encoder: FrameEncoder::new(PixelFormat::DEFAULT, width, height),
                        wants_full: false,
                        wants_incremental: false,
                        shared: true,
                    };
                    // §7.1.1: the server speaks first.
                    conn.pending.extend_from_slice(proto::VERSION_3_8);
                    self.conns.push(conn);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(e) if e.kind() == ErrorKind::Interrupted => return Ok(()),
                Err(e) => return Err(e),
            }
        }
    }

    /// One connection's turn. Returns false when it should be dropped.
    fn service(
        conn: &mut Conn,
        surface: &Surface,
        name: &str,
        events: &mut Vec<InputEvent>,
    ) -> bool {
        let mut buf = [0u8; 4096];
        loop {
            match conn.stream.read(&mut buf) {
                Ok(0) => return false,
                Ok(n) => {
                    conn.inbox.extend_from_slice(&buf[..n]);
                    // A short read means the socket is drained; a full one may
                    // not, so go round again.
                    if n < buf.len() {
                        break;
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(_) => return false,
            }
        }

        if !Self::consume(conn, name, events) {
            return false;
        }
        if conn.inbox.len() > MAX_INBOX {
            // Whatever is in there did not parse as a message and is bigger
            // than any message can be, so no amount of further reading will
            // help.
            return false;
        }
        Self::produce(conn, surface);
        Self::flush(conn)
    }

    /// Parse everything whole in the inbox. Returns false on a fatal protocol
    /// error.
    fn consume(conn: &mut Conn, name: &str, events: &mut Vec<InputEvent>) -> bool {
        loop {
            match conn.phase {
                Phase::Version => {
                    if conn.inbox.len() < proto::VERSION_LEN {
                        return true;
                    }
                    let Some(version) = Version::parse(&conn.inbox[..proto::VERSION_LEN]) else {
                        return false;
                    };
                    conn.inbox.drain(..proto::VERSION_LEN);
                    conn.version = version;
                    if version >= Version::V3_7 {
                        // §7.1.2: offer the list, wait for a choice.
                        conn.pending.extend_from_slice(&proto::security_types());
                        conn.phase = Phase::Security;
                    } else {
                        // §7.1.2, RFB 3.3: the server states the type and there
                        // is no SecurityResult for `None`.
                        conn.pending.extend_from_slice(&proto::security_type_3_3());
                        conn.phase = Phase::Init;
                    }
                }
                Phase::Security => {
                    let Some(&choice) = conn.inbox.first() else {
                        return true;
                    };
                    conn.inbox.drain(..1);
                    if choice != proto::SECURITY_NONE {
                        if conn.version >= Version::V3_8 {
                            conn.pending
                                .extend_from_slice(&proto::security_result_failed(
                                    "rsemu offers the None security type only",
                                ));
                        }
                        let _ = conn.stream.write_all(&conn.pending);
                        return false;
                    }
                    // §7.1.3: 3.8 sends a SecurityResult even for `None`; 3.7
                    // does not.
                    if conn.version >= Version::V3_8 {
                        conn.pending.extend_from_slice(&proto::security_result_ok());
                    }
                    conn.phase = Phase::Init;
                }
                Phase::Init => {
                    let Some(&shared) = conn.inbox.first() else {
                        return true;
                    };
                    conn.inbox.drain(..1);
                    conn.shared = shared != 0;
                    let (width, height) = conn.encoder.announced();
                    conn.pending.extend_from_slice(&proto::server_init(
                        width,
                        height,
                        conn.format(),
                        name,
                    ));
                    conn.phase = Phase::Ready;
                }
                Phase::Ready => match proto::parse_client(&conn.inbox) {
                    Parsed::Incomplete => return true,
                    Parsed::Unknown(_) => return false,
                    Parsed::Message(message, used) => {
                        conn.inbox.drain(..used);
                        Self::apply(conn, message, events);
                    }
                },
            }
        }
    }

    /// Act on one decoded client message.
    fn apply(conn: &mut Conn, message: ClientMessage, events: &mut Vec<InputEvent>) {
        match message {
            ClientMessage::SetPixelFormat(format) => {
                // §7.5.1 lets a client ask for anything. One this server cannot
                // produce is ignored rather than obeyed badly: the client keeps
                // getting the format it was offered in ServerInit, which it
                // said it could decode by connecting.
                if format.is_supported() {
                    conn.encoder.set_format(format);
                }
            }
            ClientMessage::SetEncodings(list) => conn.encoder.set_encodings(&list),
            ClientMessage::UpdateRequest { incremental, .. } => {
                // The requested rectangle is deliberately ignored: this server
                // answers with the whole screen or with what changed, and
                // §7.5.3 permits a server to send more than was asked for. A
                // partial-rectangle request comes from a viewer that has
                // exposed part of its window, and the extra bytes cost less
                // than the bookkeeping to honour it exactly.
                if incremental {
                    conn.wants_incremental = true;
                } else {
                    conn.wants_full = true;
                }
            }
            ClientMessage::Key { key, down } => events.push(InputEvent::Key {
                keysym: Keysym(key),
                down,
            }),
            ClientMessage::Pointer { x, y, buttons } => events.push(InputEvent::Pointer {
                x: u32::from(x),
                y: u32::from(y),
                buttons,
            }),
            // §7.5.6. The guest has no clipboard to paste into — that needs a
            // guest agent, which is SPICE's territory — so the text is dropped.
            // Dropping it is not the same as not parsing it: the bytes have to
            // be consumed or the stream desynchronises.
            ClientMessage::CutText(_) => {}
        }
    }

    /// Answer an outstanding update request, if there is one and there is room.
    fn produce(conn: &mut Conn, surface: &Surface) {
        if conn.phase != Phase::Ready || conn.pending.len() > MAX_PENDING {
            return;
        }
        if conn.wants_full {
            if let Some(update) = conn.encoder.update(surface, false) {
                conn.pending.extend_from_slice(&update);
            }
            conn.wants_full = false;
            conn.wants_incremental = false;
        } else if conn.wants_incremental {
            // An incremental request with nothing to say stays outstanding:
            // §7.5.3's contract is that the server answers *when there is
            // something to send*, which is what makes the protocol a poll loop
            // rather than a busy one.
            if let Some(update) = conn.encoder.update(surface, true) {
                conn.pending.extend_from_slice(&update);
                conn.wants_incremental = false;
            }
        }
    }

    /// Push as much of the pending output as the socket will take.
    ///
    /// Returns false when the peer has gone. A short write is normal on a
    /// non-blocking socket and leaves the rest queued for the next poll.
    fn flush(conn: &mut Conn) -> bool {
        while !conn.pending.is_empty() {
            match conn.stream.write(&conn.pending) {
                Ok(0) => return false,
                Ok(n) => {
                    conn.pending.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return true,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(_) => return false,
            }
        }
        let _ = conn.stream.flush();
        true
    }
}

/// A pixel count as RFB carries it: sixteen bits, saturating.
#[inline]
const fn clamp16(value: u32) -> u16 {
    if value > u16::MAX as u32 {
        u16::MAX
    } else {
        value as u16
    }
}

#[cfg(test)]
mod tests;
