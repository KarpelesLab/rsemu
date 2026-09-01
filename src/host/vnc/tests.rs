//! The server, driven over a real loopback socket by a client of our own.
//!
//! One thread: the test writes, turns the server's crank once, and reads. That
//! is exactly how the server is used for real — [`VncServer::poll`] is one turn
//! of a loop the caller owns — so the test exercises the shape rather than a
//! convenient fiction, and it cannot flake on a thread scheduling decision.
//!
//! `tests/vnc_protocol.rs` does the same thing from outside the crate, against
//! a real machine's picture; this file is the unit-level version that needs no
//! machine at all.

use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use crate::host::display::{PixelFormat as SurfaceFormat, Surface};
use crate::host::input::{InputEvent, Keysym};

use super::proto::{self, ClientMessage, PixelFormat, Version, client_msg, encoding};
use super::{MAX_CLIENTS, VncServer};

/// How long a test will wait for bytes it expects before giving up.
///
/// Five seconds, and the wait sleeps rather than spins. Loopback usually
/// answers on the first turn, but "usually" is not "always" — a segment can
/// arrive split, and a busy-spin that gives up in a millisecond turns that into
/// a flaky test. Sleeping also keeps the whole file cheap: the tests that do
/// wait are the ones asserting the server *does not* answer, and those pay the
/// same few milliseconds either way.
const PATIENCE: Duration = Duration::from_secs(5);

/// How long one fruitless turn sleeps before the next.
const TURN: Duration = Duration::from_micros(200);

/// The far end of a connection, as a test drives it.
struct Client {
    stream: TcpStream,
    inbox: Vec<u8>,
}

impl Client {
    fn connect(server: &VncServer) -> Client {
        let addr = server.local_addr().expect("bound");
        let stream = TcpStream::connect(addr).expect("loopback connect");
        stream.set_nonblocking(true).expect("non-blocking");
        Client {
            stream,
            inbox: Vec::new(),
        }
    }

    fn send(&mut self, bytes: &[u8]) {
        self.stream.write_all(bytes).expect("write");
    }

    /// Turn the server's crank and drain whatever it sent.
    fn pump(&mut self, server: &mut VncServer, surface: &Surface) -> Vec<InputEvent> {
        let events = server.poll(surface).expect("poll");
        let mut buf = [0u8; 65536];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.inbox.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => panic!("read: {e}"),
            }
        }
        events
    }

    /// Wait for `n` bytes, turning the crank while waiting.
    fn take(&mut self, n: usize, server: &mut VncServer, surface: &Surface) -> Vec<u8> {
        let deadline = Instant::now() + PATIENCE;
        while self.inbox.len() < n && Instant::now() < deadline {
            self.pump(server, surface);
            if self.inbox.len() < n {
                std::thread::sleep(TURN);
            }
        }
        assert!(
            self.inbox.len() >= n,
            "wanted {n} bytes, the server sent {}",
            self.inbox.len()
        );
        self.inbox.drain(..n).collect()
    }

    /// Turn the crank until `n` input events have come back, or time is up.
    fn events(&mut self, n: usize, server: &mut VncServer, surface: &Surface) -> Vec<InputEvent> {
        let deadline = Instant::now() + PATIENCE;
        let mut seen = Vec::new();
        while seen.len() < n && Instant::now() < deadline {
            seen.extend(self.pump(server, surface));
            if seen.len() < n {
                std::thread::sleep(TURN);
            }
        }
        seen
    }

    /// Turn the crank until `done` is true, or time is up.
    fn until(
        &mut self,
        server: &mut VncServer,
        surface: &Surface,
        mut done: impl FnMut(&VncServer) -> bool,
    ) {
        let deadline = Instant::now() + PATIENCE;
        while !done(server) && Instant::now() < deadline {
            self.pump(server, surface);
            std::thread::sleep(TURN);
        }
    }

    /// The RFB 3.8 handshake, up to and including ServerInit (§7.1, §7.3).
    fn handshake(&mut self, server: &mut VncServer, surface: &Surface) -> (u16, u16, String) {
        let version = self.take(proto::VERSION_LEN, server, surface);
        assert_eq!(
            &version,
            proto::VERSION_3_8,
            "§7.1.1: the server speaks first"
        );
        self.send(proto::VERSION_3_8);

        let count = self.take(1, server, surface);
        assert_eq!(count, [1], "§7.1.2: one security type on offer");
        let types = self.take(1, server, surface);
        assert_eq!(types, [proto::SECURITY_NONE], "§7.2.1: None");
        self.send(&[proto::SECURITY_NONE]);

        let result = self.take(4, server, surface);
        assert_eq!(result, [0, 0, 0, 0], "§7.1.3: SecurityResult OK");

        self.send(&[1]); // ClientInit, shared (§7.3.1)
        let init = self.take(24, server, surface);
        let width = u16::from_be_bytes([init[0], init[1]]);
        let height = u16::from_be_bytes([init[2], init[3]]);
        assert_eq!(
            PixelFormat::parse(&init[4..20]),
            Some(PixelFormat::DEFAULT),
            "§7.3.2 carries §7.4's PIXEL_FORMAT"
        );
        let name_len = u32::from_be_bytes([init[20], init[21], init[22], init[23]]) as usize;
        let name = String::from_utf8(self.take(name_len, server, surface)).expect("ascii");
        (width, height, name)
    }

    /// Ask for an update (§7.5.3).
    fn request(&mut self, incremental: bool, width: u16, height: u16) {
        let mut msg = alloc::vec![
            client_msg::FRAMEBUFFER_UPDATE_REQUEST,
            u8::from(incremental),
            0,
            0,
            0,
            0,
        ];
        msg.extend_from_slice(&width.to_be_bytes());
        msg.extend_from_slice(&height.to_be_bytes());
        self.send(&msg);
    }

    /// Read one FramebufferUpdate and return its rectangles as
    /// `(x, y, w, h, encoding, data)`.
    #[allow(clippy::type_complexity)]
    fn update(
        &mut self,
        server: &mut VncServer,
        surface: &Surface,
    ) -> Vec<(u16, u16, u16, u16, i32, Vec<u8>)> {
        let header = self.take(4, server, surface);
        assert_eq!(header[0], proto::server_msg::FRAMEBUFFER_UPDATE);
        let count = u16::from_be_bytes([header[2], header[3]]);
        let mut rects = Vec::new();
        for _ in 0..count {
            let head = self.take(12, server, surface);
            let x = u16::from_be_bytes([head[0], head[1]]);
            let y = u16::from_be_bytes([head[2], head[3]]);
            let w = u16::from_be_bytes([head[4], head[5]]);
            let h = u16::from_be_bytes([head[6], head[7]]);
            let enc = i32::from_be_bytes([head[8], head[9], head[10], head[11]]);
            let bytes = if enc == encoding::RAW {
                usize::from(w) * usize::from(h) * 4
            } else {
                0
            };
            let data = self.take(bytes, server, surface);
            rects.push((x, y, w, h, enc, data));
        }
        rects
    }
}

fn a_picture() -> Surface {
    let mut surface = Surface::new(SurfaceFormat::BGRA8888, 4, 2);
    surface.fill([0x10, 0x20, 0x30]);
    surface
}

fn a_server() -> VncServer {
    let mut server = VncServer::bind(":0")
        .expect("an ephemeral loopback port")
        .named("a test");
    server.set_geometry(4, 2);
    server
}

#[test]
fn a_client_handshakes_and_gets_a_frame() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);

    let (width, height, name) = client.handshake(&mut server, &surface);
    assert_eq!((width, height), (4, 2));
    assert_eq!(name, "a test");
    assert_eq!(server.clients(), 1);
    assert!(server.is_watched());
    assert_eq!(server.peers().len(), 1);

    client.request(false, 4, 2);
    let rects = client.update(&mut server, &surface);
    assert_eq!(rects.len(), 1);
    let (x, y, w, h, enc, data) = &rects[0];
    assert_eq!((*x, *y, *w, *h, *enc), (0, 0, 4, 2, encoding::RAW));
    // BGRA8888 in, the default RFB pixel format out: the same bytes.
    assert_eq!(&data[..4], [0x30, 0x20, 0x10, 0xff]);
    assert_eq!(data.len(), 4 * 2 * 4);
}

#[test]
fn an_rfb_3_3_client_skips_the_security_negotiation() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);

    let version = client.take(proto::VERSION_LEN, &mut server, &surface);
    assert_eq!(&version, proto::VERSION_3_8);
    client.send(b"RFB 003.003\n");
    // §7.1.2: for 3.3 the server states the type as a u32 and there is no
    // SecurityResult for `None`.
    let chosen = client.take(4, &mut server, &surface);
    assert_eq!(chosen, [0, 0, 0, u32::from(proto::SECURITY_NONE) as u8]);
    client.send(&[1]);
    let init = client.take(24, &mut server, &surface);
    assert_eq!(u16::from_be_bytes([init[0], init[1]]), 4);
}

#[test]
fn a_client_that_wants_a_security_type_we_do_not_have_is_told_so() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.take(proto::VERSION_LEN, &mut server, &surface);
    client.send(proto::VERSION_3_8);
    client.take(2, &mut server, &surface);
    client.send(&[2]); // VNC Authentication (§7.2.2), which we do not offer.
    let result = client.take(4, &mut server, &surface);
    assert_eq!(result, [0, 0, 0, 1], "§7.1.3: failed");
    client.until(&mut server, &surface, |s| s.clients() == 0);
    assert_eq!(server.clients(), 0, "and the connection is closed");
}

#[test]
fn a_negotiated_pixel_format_is_honoured() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);

    // §7.5.1: ask for R and B the other way round.
    let mut wanted = PixelFormat::DEFAULT;
    wanted.red_shift = 0;
    wanted.blue_shift = 16;
    let mut msg = alloc::vec![client_msg::SET_PIXEL_FORMAT, 0, 0, 0];
    msg.extend_from_slice(&wanted.encode());
    client.send(&msg);

    client.request(false, 4, 2);
    let rects = client.update(&mut server, &surface);
    assert_eq!(&rects[0].5[..4], [0x10, 0x20, 0x30, 0x00]);
}

#[test]
fn an_incremental_request_waits_until_something_changes() {
    let mut server = a_server();
    let mut surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);

    client.request(false, 4, 2);
    client.update(&mut server, &surface);

    // Nothing changed: the request stays outstanding and no bytes arrive.
    client.request(true, 4, 2);
    let deadline = Instant::now() + Duration::from_millis(100);
    while Instant::now() < deadline {
        client.pump(&mut server, &surface);
        std::thread::sleep(TURN);
    }
    assert!(client.inbox.is_empty(), "an unchanged frame sends nothing");

    // Now change one row. The outstanding request is answered with that row.
    surface.put(2, 1, [0xff, 0x00, 0x00]);
    let rects = client.update(&mut server, &surface);
    assert_eq!(rects.len(), 1);
    assert_eq!(
        (rects[0].0, rects[0].1, rects[0].2, rects[0].3),
        (0, 1, 4, 1),
        "just the changed row"
    );
}

#[test]
fn a_resize_reaches_a_client_that_asked_for_desktop_size() {
    let mut server = a_server();
    let mut surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);

    // §7.5.2: Raw and the DesktopSize pseudo-encoding.
    let mut msg = alloc::vec![client_msg::SET_ENCODINGS, 0, 0, 2];
    msg.extend_from_slice(&encoding::RAW.to_be_bytes());
    msg.extend_from_slice(&encoding::DESKTOP_SIZE.to_be_bytes());
    client.send(&msg);

    client.request(false, 4, 2);
    client.update(&mut server, &surface);

    surface.reshape(SurfaceFormat::BGRA8888, 8, 4);
    surface.fill([1, 2, 3]);
    client.request(true, 4, 2);
    let rects = client.update(&mut server, &surface);
    assert_eq!(rects.len(), 2);
    assert_eq!(
        (rects[0].2, rects[0].3, rects[0].4),
        (8, 4, encoding::DESKTOP_SIZE),
        "§7.8.2 carries the new geometry"
    );
    assert_eq!((rects[1].2, rects[1].3), (8, 4));
}

#[test]
fn keys_and_the_pointer_come_back_as_input_events() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);

    // §7.5.4: 'a' down, then up. §7.5.5: the pointer at (3, 1), left held.
    client.send(&[client_msg::KEY_EVENT, 1, 0, 0, 0, 0, 0, 0x61]);
    client.send(&[client_msg::KEY_EVENT, 0, 0, 0, 0, 0, 0, 0x61]);
    client.send(&[client_msg::POINTER_EVENT, 1, 0, 3, 0, 1]);

    let seen = client.events(3, &mut server, &surface);
    assert_eq!(
        seen,
        [
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: true
            },
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: false
            },
            InputEvent::Pointer {
                x: 3,
                y: 1,
                buttons: 1
            },
        ]
    );
}

#[test]
fn a_cut_text_is_consumed_rather_than_desynchronising_the_stream() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);

    // §7.5.6, then a key. If the text were not consumed by length, the key
    // would be parsed out of the middle of it and come back wrong.
    let mut msg = alloc::vec![client_msg::CLIENT_CUT_TEXT, 0, 0, 0, 0, 0, 0, 3];
    msg.extend_from_slice(b"abc");
    msg.extend_from_slice(&[client_msg::KEY_EVENT, 1, 0, 0, 0, 0, 0xff, 0x0d]);
    client.send(&msg);

    let seen = client.events(1, &mut server, &surface);
    assert_eq!(
        seen,
        [InputEvent::Key {
            keysym: Keysym::RETURN,
            down: true
        }]
    );
}

#[test]
fn a_message_type_we_do_not_know_closes_the_connection() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);
    // There is no length prefix on a client message, so an unknown type cannot
    // be skipped — see `proto::Parsed::Unknown`.
    client.send(&[250, 0, 0, 0]);
    client.until(&mut server, &surface, |s| s.clients() == 0);
    assert_eq!(server.clients(), 0);
}

#[test]
fn two_clients_each_get_their_own_picture() {
    let mut server = a_server();
    let mut surface = a_picture();
    let mut first = Client::connect(&server);
    first.handshake(&mut server, &surface);
    first.request(false, 4, 2);
    first.update(&mut server, &surface);

    // The second attaches after a change the first has already seen, and needs
    // a whole frame rather than the damage.
    surface.fill([9, 9, 9]);
    first.request(true, 4, 2);
    first.update(&mut server, &surface);

    let mut second = Client::connect(&server);
    second.handshake(&mut server, &surface);
    assert_eq!(server.clients(), 2);
    second.request(true, 4, 2);
    let rects = second.update(&mut server, &surface);
    assert_eq!(rects.len(), 1);
    assert_eq!((rects[0].2, rects[0].3), (4, 2), "the whole screen");
}

#[test]
fn a_ninth_client_is_refused_rather_than_queued() {
    let mut server = a_server();
    let surface = a_picture();
    let mut clients = Vec::new();
    for _ in 0..MAX_CLIENTS {
        let mut client = Client::connect(&server);
        client.handshake(&mut server, &surface);
        clients.push(client);
    }
    assert_eq!(server.clients(), MAX_CLIENTS);
    let extra = TcpStream::connect(server.local_addr().expect("bound"));
    // The connection may be accepted by the kernel and dropped by us, so what
    // is asserted is the client count rather than the connect's own result.
    drop(extra);
    server.poll(&surface).expect("poll");
    assert_eq!(server.clients(), MAX_CLIENTS);
}

#[test]
fn a_client_that_hangs_up_is_forgotten() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);
    assert_eq!(server.clients(), 1);
    drop(client);
    let deadline = Instant::now() + PATIENCE;
    while server.clients() > 0 && Instant::now() < deadline {
        server.poll(&surface).expect("poll");
        std::thread::sleep(TURN);
    }
    assert_eq!(server.clients(), 0);
}

#[test]
fn the_bell_reaches_a_connected_client() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);
    server.bell();
    let byte = client.take(1, &mut server, &surface);
    assert_eq!(byte, [proto::server_msg::BELL], "§7.6.3");
}

#[test]
fn a_client_that_never_finishes_a_message_is_hung_up_on() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);
    // §7.5.6 with a length nobody is going to send. Holding the buffer open for
    // it is the allocation an unauthenticated peer must not be able to make.
    let mut msg = alloc::vec![client_msg::CLIENT_CUT_TEXT, 0, 0, 0];
    msg.extend_from_slice(&u32::MAX.to_be_bytes());
    msg.extend_from_slice(&[0u8; 4096]);
    // Keep writing until the server gives up, which it must do long before the
    // four gigabytes the message claimed.
    let deadline = Instant::now() + PATIENCE;
    while server.clients() > 0 && Instant::now() < deadline {
        if client.stream.write_all(&msg).is_err() {
            break;
        }
        client.pump(&mut server, &surface);
    }
    assert_eq!(server.clients(), 0, "the server stopped buffering");
}

#[test]
fn a_message_split_across_two_packets_is_reassembled() {
    let mut server = a_server();
    let surface = a_picture();
    let mut client = Client::connect(&server);
    client.handshake(&mut server, &surface);
    // Half a KeyEvent, a turn of the crank, then the other half.
    client.send(&[client_msg::KEY_EVENT, 1, 0]);
    let deadline = Instant::now() + Duration::from_millis(100);
    while Instant::now() < deadline {
        assert!(
            client.pump(&mut server, &surface).is_empty(),
            "half a message is not a message"
        );
        std::thread::sleep(TURN);
    }
    client.send(&[0, 0, 0, 0, 0x7a]);
    let seen = client.events(1, &mut server, &surface);
    assert_eq!(
        seen,
        [InputEvent::Key {
            keysym: Keysym::from_ascii(b'z'),
            down: true
        }]
    );
}

#[test]
fn a_client_message_parses_the_same_here_as_in_the_unit_tests() {
    // A guard against the two ends of the crate drifting: the enum the server
    // acts on is the enum `proto` produces.
    let bytes = [client_msg::KEY_EVENT, 1, 0, 0, 0, 0, 0, 0x61];
    assert!(matches!(
        proto::parse_client(&bytes),
        proto::Parsed::Message(
            ClientMessage::Key {
                key: 0x61,
                down: true
            },
            8
        )
    ));
    assert_eq!(Version::parse(proto::VERSION_3_8), Some(Version::V3_8));
}
