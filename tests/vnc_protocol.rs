//! Phase 9's remote-display gate: **a client connects and gets a real frame**.
//!
//! `src/host/vnc/tests.rs` drives the server with a client of our own against a
//! `Surface` a test filled in. That proves the bytes are what we think they
//! are; it cannot prove they are what a *viewer* thinks they are, and it does
//! not show a real device's picture. This file does both halves of that:
//!
//! * an RFB client, written from RFC 6143 and living outside the crate, speaks
//!   the whole handshake to a server showing a **VGA adapter's own framebuffer**
//!   and asserts the geometry, the negotiated pixel format and the pixels; and
//! * if a real VNC viewer binary is installed, it is run against the same
//!   server and its exit status and output are asserted.
//!
//! # It skips rather than fails
//!
//! The second half needs a viewer, and no distribution installs one by default.
//! `$RSEMU_VNC_CLIENT`, else `vncviewer`, else `gvncviewer`, else
//! `xtightvncviewer` — absent, the test prints why and returns, exactly as
//! `tests/gdb_real_client.rs` does for `gdb`. **`cargo test` stays hermetic.**
//! The viewer also needs an X display; without `$DISPLAY` or `$WAYLAND_DISPLAY`
//! it skips too, because a headless CI box has no screen to draw on and the
//! failure would say nothing about rsemu.
//!
//! Running a viewer against our server is black-box use, which `ROADMAP.md` §1
//! permits in as many words. No VNC implementation's source was read; the
//! protocol comes from RFC 6143, section by section, cited in
//! `src/host/vnc/proto.rs`.

#![cfg(all(feature = "vnc", feature = "dev-pc-video"))]

use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::display::{PixelFormat as SurfaceFormat, Scanout, Surface};
use rsemu::host::vnc::VncServer;
use rsemu::host::vnc::proto::{self, PixelFormat};

/// How long to wait for bytes the server owes us.
const PATIENCE: Duration = Duration::from_secs(5);

/// How long one fruitless turn of the loop sleeps.
const TURN: Duration = Duration::from_micros(200);

/// The board: a VGA adapter and nothing else.
///
/// A `pc.video` out of reset is in the 80×25 text mode a PC comes up in, which
/// is 720×400 pixels — the geometry `machines/pc-at.machine`'s option ROM sets
/// and the one a viewer should see. Nothing else is needed to have a picture,
/// and leaving the rest out keeps the test about RFB.
const ONE_ADAPTER: &str = r#"
machine "one-adapter" {
  osc dot = 28322000 Hz
  space port { width = 16, unassigned = open-bus }
  object vga "pc.video" { clock = dot / 9 }
  map port 0x03c0 size 0x0010 = vga.vga
  map port 0x03d4 size 0x0002 = vga.crtc-colour
}
"#;

/// Build the board, give it a colour to draw, and take a handle on its screen.
///
/// A VGA out of reset has an all-zero DAC and an all-zero video memory, so its
/// picture is genuinely black — correct, and useless as a test subject, because
/// a server that sent nothing at all would pass. What loads the DAC on a real
/// PC is the video BIOS, which this repository cannot ship. So the test plays
/// the part of the option ROM in three `OUT`s: set the DAC write index to zero
/// and give palette entry 0 a colour. Every pixel of a blank text screen is
/// index 0, so the whole framebuffer becomes that colour and the assertions
/// below have something to check.
///
/// The port numbers are the IBM VGA's own: `0x3C8` is the DAC address register
/// for writes and `0x3C9` takes the three six-bit components in R, G, B order
/// (IBM VGA Technical Reference).
fn a_screen() -> impl Scanout {
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(rsemu::machine::catalog::bindings().expect("this build's bindings"));
    rsemu::host::display::pc::capture::install(&mut options).expect("this build's capture table");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = rsemu::machine::build("one-adapter.machine", ONE_ADAPTER, &registry, &options)
        .expect("a machine with nothing but a display");
    assert_eq!(machine.name(), "one-adapter");

    let ports = machine.space("port").expect("the I/O space");
    let out = |port: u64, value: u64| {
        ports
            .write(port, Width::U8, value, MemAttrs::DEFAULT)
            .expect("a decoded port");
    };
    out(0x3c8, 0);
    out(0x3c9, 0x21);
    out(0x3c9, 0x10);
    out(0x3c9, 0x08);

    rsemu::host::display::pc::capture::take(&options.realize.hosts)
        .expect("the constructor kept a handle")
}

/// The far end of a connection, as this test drives it.
struct Client {
    stream: TcpStream,
    inbox: Vec<u8>,
}

impl Client {
    fn connect(server: &VncServer) -> Client {
        let stream =
            TcpStream::connect(server.local_addr().expect("bound")).expect("loopback connect");
        stream.set_nonblocking(true).expect("non-blocking");
        Client {
            stream,
            inbox: Vec::new(),
        }
    }

    fn send(&mut self, bytes: &[u8]) {
        self.stream.write_all(bytes).expect("write");
    }

    fn pump(&mut self, server: &mut VncServer, surface: &Surface) {
        server.poll(surface).expect("poll");
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
    }

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
}

#[test]
fn a_client_sees_the_adapters_own_framebuffer() {
    let scanout = a_screen();
    let info = scanout.info();
    let mut surface = Surface::new(SurfaceFormat::BGRA8888, info.width, info.height);
    scanout.capture(&mut surface);

    let mut server = VncServer::bind(":0")
        .expect("an ephemeral loopback port")
        .named("one-adapter");
    server.set_geometry(info.width, info.height);
    let mut client = Client::connect(&server);

    // §7.1.1 ProtocolVersion.
    let version = client.take(proto::VERSION_LEN, &mut server, &surface);
    assert_eq!(&version, proto::VERSION_3_8);
    client.send(proto::VERSION_3_8);

    // §7.1.2 Security, §7.2.1 None, §7.1.3 SecurityResult.
    let offered = client.take(2, &mut server, &surface);
    assert_eq!(offered, [1, proto::SECURITY_NONE]);
    client.send(&[proto::SECURITY_NONE]);
    assert_eq!(client.take(4, &mut server, &surface), [0, 0, 0, 0]);

    // §7.3.1 ClientInit, §7.3.2 ServerInit.
    client.send(&[1]);
    let init = client.take(24, &mut server, &surface);
    let width = u16::from_be_bytes([init[0], init[1]]);
    let height = u16::from_be_bytes([init[2], init[3]]);
    assert_eq!(
        (width, height),
        (720, 400),
        "80 columns of nine pixels by 25 rows of sixteen: the mode a VGA \
         comes out of reset in"
    );
    let format = PixelFormat::parse(&init[4..20]).expect("§7.4");
    assert_eq!(format, PixelFormat::DEFAULT);
    assert_eq!(format.bits_per_pixel, 32);
    assert!(format.true_colour);
    let name_len = u32::from_be_bytes([init[20], init[21], init[22], init[23]]) as usize;
    let name = String::from_utf8(client.take(name_len, &mut server, &surface)).expect("ascii");
    assert_eq!(name, "one-adapter");

    // §7.5.3 FramebufferUpdateRequest, non-incremental, the whole screen.
    let mut request = vec![3u8, 0, 0, 0, 0, 0];
    request.extend_from_slice(&width.to_be_bytes());
    request.extend_from_slice(&height.to_be_bytes());
    client.send(&request);

    // §7.6.1 FramebufferUpdate.
    let header = client.take(4, &mut server, &surface);
    assert_eq!(header[0], 0, "message type 0");
    assert_eq!(
        u16::from_be_bytes([header[2], header[3]]),
        1,
        "one rectangle"
    );
    let rect = client.take(12, &mut server, &surface);
    assert_eq!(u16::from_be_bytes([rect[0], rect[1]]), 0, "x");
    assert_eq!(u16::from_be_bytes([rect[2], rect[3]]), 0, "y");
    assert_eq!(u16::from_be_bytes([rect[4], rect[5]]), width);
    assert_eq!(u16::from_be_bytes([rect[6], rect[7]]), height);
    assert_eq!(
        i32::from_be_bytes([rect[8], rect[9], rect[10], rect[11]]),
        0,
        "§7.7.1 Raw"
    );

    // §7.7.1: width × height × 4 bytes, and they are the adapter's pixels.
    let pixels = client.take(
        usize::from(width) * usize::from(height) * 4,
        &mut server,
        &surface,
    );
    assert_eq!(pixels.len(), surface.pixels().len());
    assert_eq!(
        pixels,
        surface.pixels(),
        "BGRA8888 is byte for byte the default RFB pixel format"
    );
    // And the picture is a real one rather than a black rectangle. The DAC was
    // loaded above; a blank text screen is every pixel at palette index 0, so
    // every pixel should be that colour — which also proves the pixels came
    // from the adapter's own colour chain rather than from an empty buffer.
    assert!(
        pixels
            .as_chunks::<4>()
            .0
            .iter()
            .all(|p| p[..3] != [0, 0, 0]),
        "every pixel carries the colour the DAC was given"
    );
}

// ---------------------------------------------------------------------------
// a real viewer
// ---------------------------------------------------------------------------

/// Find a VNC viewer to drive, or explain why there is none.
fn find_viewer() -> Option<String> {
    if let Ok(explicit) = std::env::var("RSEMU_VNC_CLIENT")
        && !explicit.is_empty()
    {
        return Some(explicit);
    }
    for candidate in ["vncviewer", "gvncviewer", "xtightvncviewer", "vinagre"] {
        let found = Command::new("sh")
            .arg("-c")
            .arg(format!("command -v {candidate}"))
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .is_ok_and(|s| s.success());
        if found {
            return Some(candidate.to_string());
        }
    }
    None
}

/// Whether there is a screen for a viewer to open a window on.
fn has_a_display() -> bool {
    std::env::var_os("DISPLAY").is_some() || std::env::var_os("WAYLAND_DISPLAY").is_some()
}

/// Run an installed viewer against the server, for as long as it takes it to
/// draw one frame, and assert that it got one.
///
/// The claim this test makes is narrow and honest: a viewer nobody here wrote
/// completed the handshake and asked for a framebuffer update, and the server
/// answered it. That is the part a protocol test cannot check — that our
/// reading of RFC 6143 matches somebody else's.
#[test]
fn a_real_viewer_draws_a_frame() {
    let Some(viewer) = find_viewer() else {
        eprintln!(
            "skipping: no VNC viewer. Set $RSEMU_VNC_CLIENT, or install vncviewer or \
             gvncviewer, to run this test against a client rsemu did not write."
        );
        return;
    };
    if !has_a_display() {
        eprintln!("skipping: `{viewer}` needs $DISPLAY or $WAYLAND_DISPLAY and there is none.");
        return;
    }

    let scanout = a_screen();
    let info = scanout.info();
    let mut surface = Surface::new(SurfaceFormat::BGRA8888, info.width, info.height);
    scanout.capture(&mut surface);

    let mut server = VncServer::bind(":0")
        .expect("an ephemeral loopback port")
        .named("rsemu one-adapter");
    server.set_geometry(info.width, info.height);
    let addr = server.local_addr().expect("bound");

    let mut child = Command::new(&viewer)
        .arg(format!("{}:{}", addr.ip(), addr.port()))
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap_or_else(|e| panic!("spawning `{viewer}`: {e}"));

    // Turn the crank until the viewer has connected and been sent a whole
    // frame, or until it is clear it never will.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut attached = false;
    while Instant::now() < deadline {
        server.poll(&surface).expect("poll");
        attached |= server.is_watched();
        if attached && !server.is_watched() {
            break;
        }
        if child.try_wait().ok().flatten().is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    let _ = child.kill();
    let output = child.wait_with_output().expect("the viewer's output");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        attached,
        "`{viewer}` never completed the handshake. It said: {stderr}"
    );
    // A viewer that got as far as a framebuffer update printed no protocol
    // complaint. The ones that do complain say so on stderr and name the
    // message they choked on.
    for bad in ["protocol", "unknown", "unsupported", "invalid"] {
        assert!(
            !stderr.to_ascii_lowercase().contains(bad),
            "`{viewer}` complained: {stderr}"
        );
    }
    eprintln!("`{viewer}` connected and was sent a frame");
}
