//! A real GDB session against a real machine, over a real socket.
//!
//! Unit tests prove the packet layer frames correctly and the stub answers
//! correctly against a fake target. Neither shows that a debugger can actually
//! drive an Apple 1: that needs the listener, a client on the other end of a TCP
//! connection, and a guest that runs. This is that test.
//!
//! The session below is the one a person has: attach, negotiate, read the target
//! description, look at the registers, write a small program into RAM, single-
//! step it, set a breakpoint and hit it, set a watchpoint and hit it, read the
//! byte the guest stored, and detach. Every assertion is on the bytes exchanged.
//!
//! Run it with `--nocapture` to see the transcript.

#![cfg(all(feature = "gdb", feature = "machine-apple1"))]

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use rsemu::host::gdb::packet::{Event, Framer, frame};
use rsemu::host::gdb::{GdbServer, MachineTarget, Progress};

/// How long a client will wait for a reply before calling the server broken.
const REPLY_TIMEOUT: Duration = Duration::from_secs(20);

/// A GDB, near enough: framing, acknowledgements, and a transcript.
struct Client {
    stream: TcpStream,
    framer: Framer,
    /// Packets that arrived in the same read as an earlier one. A reply can be
    /// two packets — `qRcmd` answers with console output and then `OK` — and a
    /// client that threw away whatever followed the first would deadlock on the
    /// second.
    queued: std::collections::VecDeque<Vec<u8>>,
    transcript: Vec<String>,
}

impl Client {
    fn connect(addr: std::net::SocketAddr) -> Client {
        let stream = TcpStream::connect(addr).expect("the stub is listening");
        stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .expect("read timeout");
        stream.set_nodelay(true).expect("nodelay");
        Client {
            stream,
            framer: Framer::new(),
            queued: std::collections::VecDeque::new(),
            transcript: Vec::new(),
        }
    }

    /// Send one packet.
    fn send(&mut self, packet: &[u8]) {
        self.transcript
            .push(format!("-> {}", String::from_utf8_lossy(packet)));
        let mut wire = Vec::new();
        frame(packet, &mut wire);
        self.stream.write_all(&wire).expect("write");
        self.stream.flush().expect("flush");
    }

    /// Wait for one packet, acknowledging it the way GDB does.
    fn recv(&mut self) -> Vec<u8> {
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut buf = [0u8; 512];
        loop {
            if let Some(payload) = self.queued.pop_front() {
                self.transcript
                    .push(format!("<- {}", String::from_utf8_lossy(&payload)));
                return payload;
            }
            assert!(
                Instant::now() < deadline,
                "no reply within {REPLY_TIMEOUT:?}; transcript so far:\n{}",
                self.transcript.join("\n")
            );
            let read = match self.stream.read(&mut buf) {
                Ok(0) => panic!("the stub hung up"),
                Ok(n) => n,
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    continue;
                }
                Err(e) => panic!("read: {e}"),
            };
            for byte in &buf[..read] {
                match self.framer.push(*byte) {
                    Some(Event::Packet(payload)) => {
                        self.stream.write_all(b"+").expect("ack");
                        self.queued.push_back(payload);
                    }
                    Some(Event::Corrupt) => panic!("the stub sent a corrupt packet"),
                    _ => {}
                }
            }
        }
    }

    /// Send a packet and return its reply, as text.
    fn ask(&mut self, packet: &str) -> String {
        self.send(packet.as_bytes());
        String::from_utf8_lossy(&self.recv()).into_owned()
    }

    /// Send a packet that has no reply of its own.
    fn tell(&mut self, packet: &str) {
        self.send(packet.as_bytes());
    }
}

/// The server half: an Apple 1 running RSMON, driven by the session loop.
struct Server {
    addr: std::net::SocketAddr,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Server {
    fn start() -> Server {
        let server = GdbServer::bind(":0").expect("bind an ephemeral port");
        let addr = server.local_addr().expect("local_addr");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(String::from("gdb-session-test"))
            .spawn(move || {
                let mut server = server;
                // The smallest machine rsemu ships, with rsemu's own monitor in
                // its ROM socket, so the test needs no image of unclear
                // provenance.
                let mut machine = rsemu::machine::catalog::build_catalog(
                    "apple1",
                    &[("rom", rsemu::dev::apple1::RSMON)],
                )
                .expect("apple1 builds");
                let mut target = MachineTarget::new(&mut machine);
                while !flag.load(Ordering::Relaxed) {
                    match server.poll(&mut target) {
                        Ok(Progress::Kill) => break,
                        Ok(_) => {}
                        Err(e) => panic!("gdb server: {e}"),
                    }
                }
            })
            .expect("spawn");
        Server {
            addr,
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// A 16-bit little-endian value out of a `p`/`g` reply.
fn le16(hex: &str) -> u16 {
    let bytes = hex.as_bytes();
    assert!(bytes.len() >= 4, "not a 16-bit register value: {hex}");
    let byte = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).expect("hex");
    u16::from(byte(0)) | (u16::from(byte(2)) << 8)
}

#[test]
fn a_debugger_drives_an_apple_1_over_tcp() {
    let server = Server::start();
    let mut gdb = Client::connect(server.addr);

    // -- negotiation -------------------------------------------------------
    let supported = gdb.ask("qSupported:multiprocess+;swbreak+;xmlRegisters=i386");
    for feature in [
        "PacketSize=",
        "qXfer:features:read+",
        "swbreak+",
        "vContSupported+",
    ] {
        assert!(
            supported.contains(feature),
            "{feature} missing: {supported}"
        );
    }
    assert_eq!(gdb.ask("qAttached"), "1");
    assert_eq!(gdb.ask("?"), "T05thread:1;");
    assert_eq!(gdb.ask("vCont?"), "vCont;c;C;s;S;t");

    // -- the machine's CPUs, as threads ------------------------------------
    assert_eq!(gdb.ask("qfThreadInfo"), "m1", "one 6502, so one thread");
    assert_eq!(gdb.ask("qsThreadInfo"), "l");
    assert_eq!(gdb.ask("qC"), "QC1");
    assert_eq!(gdb.ask("Hg1"), "OK");
    assert_eq!(gdb.ask("Hc1"), "OK");
    let extra = gdb.ask("qThreadExtraInfo,1");
    let extra = String::from_utf8(
        (0..extra.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&extra[i..i + 2], 16).expect("hex"))
            .collect(),
    )
    .expect("utf-8");
    assert_eq!(extra, "cpu (cpu.mos6502)");

    // -- the target description --------------------------------------------
    // This is what lets GDB debug a 6502 without ever having been compiled to
    // know what one is.
    let mut xml = String::new();
    let mut offset = 0usize;
    loop {
        let reply = gdb.ask(&format!("qXfer:features:read:target.xml:{offset:x},100"));
        let (tag, body) = reply.split_at(1);
        xml.push_str(body);
        offset += body.len();
        assert!(tag == "m" || tag == "l", "unexpected qXfer tag {tag}");
        if tag == "l" {
            break;
        }
    }
    assert!(xml.contains("org.rsemu.mos6502"), "{xml}");
    for reg in ["a", "x", "y", "sp", "p", "pc"] {
        assert!(
            xml.contains(&format!("name=\"{reg}\"")),
            "the description is missing {reg}:\n{xml}"
        );
    }

    // -- registers ---------------------------------------------------------
    // Six registers, seven bytes, fourteen hex digits: a x y sp p pc.
    let regs = gdb.ask("g");
    assert_eq!(regs.len(), 14, "the 6502 g packet is seven bytes: {regs}");
    assert_eq!(gdb.ask("p5"), regs[10..14], "`p5` is the pc out of `g`");

    // The reset vector, read the way a debugger reads it: no side effects.
    let vector = gdb.ask("mfffc,2");
    assert_eq!(vector.len(), 4);
    let entry = le16(&vector);
    assert!(
        entry >= 0xff00,
        "RSMON's reset vector is in its ROM: {entry:#06x}"
    );

    // Four steps to get the reset sequence out of the way, so that the program
    // counter is ours to set from here on.
    for _ in 0..4 {
        assert_eq!(gdb.ask("s"), "T05thread:1;");
    }
    assert_eq!(
        le16(&gdb.ask("p5")) & 0xff00,
        0xff00,
        "after reset the 6502 is executing the monitor ROM"
    );

    // -- write a program, and run it ---------------------------------------
    //
    //   0300  a9 42     LDA #$42
    //   0302  8d 10 03  STA $0310
    //   0305  4c 00 03  JMP $0300
    //
    // Eight bytes of RAM, a known instruction count, and a loop — so a
    // breakpoint and a watchpoint both have something deterministic to catch.
    assert_eq!(gdb.ask("M300,8:a9428d10034c0003"), "OK");
    assert_eq!(gdb.ask("m300,8"), "a9428d10034c0003", "the write took");
    assert_eq!(gdb.ask("P5=0003"), "OK", "pc := $0300, little endian");
    assert_eq!(le16(&gdb.ask("p5")), 0x0300);

    // One step is one instruction: LDA #$42 leaves A holding $42 and the pc on
    // the next opcode.
    assert_eq!(gdb.ask("s"), "T05thread:1;");
    assert_eq!(le16(&gdb.ask("p5")), 0x0302);
    let regs = gdb.ask("g");
    assert_eq!(&regs[0..2], "42", "A holds the immediate: {regs}");

    // -- a breakpoint ------------------------------------------------------
    assert_eq!(gdb.ask("Z0,305,1"), "OK");
    gdb.tell("c");
    assert_eq!(
        gdb.recv(),
        b"T05thread:1;swbreak:;",
        "continue stops at the breakpoint, and says why"
    );
    assert_eq!(le16(&gdb.ask("p5")), 0x0305, "stopped on the JMP");
    assert_eq!(gdb.ask("m310,1"), "42", "the STA ran before the breakpoint");
    assert_eq!(gdb.ask("z0,305,1"), "OK");

    // The same breakpoint through vCont, to prove that path works too.
    assert_eq!(gdb.ask("Z0,302,1"), "OK");
    gdb.tell("vCont;c:1");
    assert_eq!(gdb.recv(), b"T05thread:1;swbreak:;");
    assert_eq!(le16(&gdb.ask("p5")), 0x0302);
    assert_eq!(gdb.ask("z0,302,1"), "OK");

    // -- a watchpoint ------------------------------------------------------
    // Clear the byte the loop writes, then watch it. The debugger's own write
    // must not trip the watchpoint it is about to set.
    assert_eq!(gdb.ask("M310,1:00"), "OK");
    assert_eq!(gdb.ask("Z2,310,1"), "OK");
    gdb.tell("c");
    assert_eq!(
        gdb.recv(),
        b"T05thread:1;watch:310;",
        "the guest's STA is seen, and the address is reported"
    );
    assert_eq!(gdb.ask("m310,1"), "42");
    assert_eq!(gdb.ask("z2,310,1"), "OK");

    // Read and access watchpoints are not available, and say so the way the
    // protocol says so — an empty reply, so GDB falls back rather than
    // believing a watchpoint was set.
    assert_eq!(gdb.ask("Z3,310,1"), "");
    assert_eq!(gdb.ask("Z4,310,1"), "");

    // -- interrupting a free-running guest ---------------------------------
    gdb.tell("c");
    gdb.stream.write_all(&[0x03]).expect("ctrl-c");
    gdb.stream.flush().expect("flush");
    assert_eq!(gdb.recv(), b"T02thread:1;", "Ctrl-C stops with SIGINT");

    // -- the monitor command ----------------------------------------------
    let hex: String = "devices"
        .bytes()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join("");
    let output = gdb.ask(&format!("qRcmd,{hex}"));
    assert!(output.starts_with('O'), "console output: {output}");
    let text = String::from_utf8(
        (1..output.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&output[i..i + 2], 16).expect("hex"))
            .collect(),
    )
    .expect("utf-8");
    assert!(text.contains("cpu.mos6502"), "{text}");
    // `qRcmd` answers with console output *and then* `OK`.
    assert_eq!(String::from_utf8_lossy(&gdb.recv()), "OK");

    // A packet nobody implements gets the empty reply, which is the protocol's
    // "no such packet" and is what GDB probes with.
    assert_eq!(gdb.ask("qWhatIsThis"), "");

    // -- detach ------------------------------------------------------------
    assert_eq!(gdb.ask("D"), "OK");

    println!("--- gdb session transcript ---");
    for line in &gdb.transcript {
        println!("{line}");
    }
}
