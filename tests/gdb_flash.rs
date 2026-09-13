//! `load` into flash, over the wire and through the target.
//!
//! GDB writes flash with three packets and nothing else: `vFlashErase` clears
//! whole blocks, `vFlashWrite` puts bytes in the cleared range, and
//! `vFlashDone` ends the sequence. A range it has been told is flash is a range
//! it will not touch with `M` or `X`, so the packets and the memory map are one
//! feature — a stub that declares `flash` without answering them turns a `load`
//! that fails cleanly into one that stalls, which is why the map used to claim
//! `ram` and `rom` only.
//!
//! # What makes a range flash
//!
//! Not a device's name. `core::space` refuses a write to a read-only mapping
//! *including a debug one*, deliberately and in as many words
//! (`src/core/space/flat.rs`), and nothing here changes that: a `rom` object is
//! still `rom` in the map and `load` into one still fails. What is offered is
//! the range whose write side is a device that published a
//! [`FlashLayout`](rsemu::core::space::FlashLayout) — which is that device's
//! promise that a debug write reaches its array, the "loader's door"
//! `st.flash` documents and `dfu.loader` already uses. So the two halves of the
//! test below are the two halves of the claim: the F4's sectors are offered and
//! programmed, and the ROM beside them is not offered at all.
//!
//! The session runs over a real socket rather than against the `Stub` in
//! process, because `vFlashWrite`'s payload is *binary*: a byte that would end
//! a packet is escaped on the wire, and a stub that read the payload before the
//! framer unescaped it would corrupt exactly the images that contain `}`, `#`,
//! `$` or `*` — which is most of them.

#![cfg(all(feature = "gdb", feature = "cpu-arm-v7m", feature = "dev-stm32-flash"))]

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use rsemu::host::gdb::packet::{Event, Framer, frame};
use rsemu::host::gdb::{
    DebugTarget, GdbServer, MachineTarget, MemKind, MemRegion, Progress, memory_map_xml,
};
use rsemu::machine::{Machine, catalog};

/// A Cortex-M4 with a 1 MiB F4 flash array at its usual address, a small ROM
/// and some RAM.
///
/// The ROM is what proves the refusal is still in place: it is mapped, it is
/// readable, and it is *not* offered to `vFlash*` — a `rom` object has no
/// loader's door, so nothing declares one for it.
const BOARD: &str = r#"
machine "gdb-flash" {
  osc hse = 8000000 Hz
  space mem { width = 32 }
  object cpu "cpu.arm.v7m" {
    clock = hse
    space = mem
    part  = "cortex-m4"
  }
  object eflash "st.flash" {
    clock   = hse
    variant = "f4"
    size    = 1M
  }
  object sram1 "ram" { size = 64K }
  object boot  "rom" { size = 4K }
  map mem 0x08000000 size 1M   = eflash.array
  map mem 0x1fff0000 size 4K   = boot
  map mem 0x20000000 size 64K  = sram1
  map mem 0x40023c00 size 0x1c = eflash
}
"#;

/// The same machine with the array taken out: a board whose only non-RAM is a
/// ROM, which is what a target with no flash at all looks like.
const NO_FLASH: &str = r#"
machine "gdb-no-flash" {
  osc hse = 8000000 Hz
  space mem { width = 32 }
  object cpu "cpu.arm.v7m" {
    clock = hse
    space = mem
    part  = "cortex-m4"
  }
  object sram1 "ram" { size = 64K }
  object boot  "rom" { size = 4K }
  map mem 0x00000000 size 64K = sram1
  map mem 0x08000000 size 4K  = boot
}
"#;

/// Where the F4's array is mapped, and the geometry RM0090 Table 5 gives it.
const FLASH: u64 = 0x0800_0000;
/// The first four sectors are 16 KiB.
const SECTOR0: u64 = 16 * 1024;
/// Sector 4 is 64 KiB, at offset 64 KiB.
const SECTOR4: u64 = 64 * 1024;
/// Sectors 5 and up are 128 KiB.
const SECTOR5: u64 = 128 * 1024;
/// Where the ROM is, which is the range `vFlash*` must refuse.
const ROM: u64 = 0x1fff_0000;
/// Where the RAM is.
const RAM: u64 = 0x2000_0000;

/// Build a board from `src`.
fn board(name: &str, src: &str) -> Machine {
    let options = catalog::build_options().expect("the catalog agrees with itself");
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(name, src, &registry, &options)
        .unwrap_or_else(|e| panic!("the {name} fixture does not realize: {e}"))
}

// ---------------------------------------------------------------------------
// The map
// ---------------------------------------------------------------------------

#[test]
fn the_map_declares_the_f4_sectors_as_flash_and_the_rom_as_rom() {
    let mut machine = board("gdb-flash.machine", BOARD);
    let target = MachineTarget::new(&mut machine);
    let map = target.memory_map(0).expect("a map");

    // RM0090 Table 5: four sectors of 16 KiB, one of 64 KiB, then 128 KiB to
    // the end of the megabyte. Three runs, three blocksizes — one region with
    // a single blocksize would be a lie about eleven of the twelve sectors,
    // and GDB aligns its erases to whatever it is told.
    let flash: Vec<&MemRegion> = map.iter().filter(|r| r.kind.is_flash()).collect();
    assert_eq!(
        flash.len(),
        3,
        "an F4 bank is three runs of equal sectors: {map:?}"
    );
    assert_eq!(
        (flash[0].start, flash[0].length, flash[0].kind),
        (FLASH, 4 * SECTOR0, MemKind::Flash { blocksize: SECTOR0 })
    );
    assert_eq!(
        (flash[1].start, flash[1].length, flash[1].kind),
        (
            FLASH + 4 * SECTOR0,
            SECTOR4,
            MemKind::Flash { blocksize: SECTOR4 }
        )
    );
    assert_eq!(
        (flash[2].start, flash[2].length, flash[2].kind),
        (
            FLASH + 128 * 1024,
            7 * SECTOR5,
            MemKind::Flash { blocksize: SECTOR5 }
        )
    );

    // The ROM is still `rom`, which is the half of this feature that is a
    // refusal: `core::space` will not take a write to a read-only mapping even
    // for a debug access, so a range with no device door behind it is not
    // offered to `vFlash*` however much someone would like to `load` into it.
    let rom = map
        .iter()
        .find(|r| r.start == ROM)
        .expect("the rom is in the map");
    assert_eq!(rom.kind, MemKind::Rom);
    // And the register window is ordinary I/O: a device that publishes no
    // layout is not flash because it sits next to one that does.
    let regs = map
        .iter()
        .find(|r| r.start == 0x4002_3c00)
        .expect("the flash interface registers are in the map");
    assert_eq!(regs.kind, MemKind::Ram);
}

#[test]
fn the_map_document_carries_a_blocksize_for_every_flash_region() {
    let mut machine = board("gdb-flash.machine", BOARD);
    let target = MachineTarget::new(&mut machine);
    let xml = memory_map_xml(&target.memory_map(0).expect("a map"));

    // The one property the memory-map DTD defines, and the one GDB refuses to
    // guess: a flash region without it is a region `load` cannot use.
    assert_eq!(
        xml.matches("type=\"flash\"").count(),
        xml.matches("<property name=\"blocksize\">").count(),
        "every flash region needs a blocksize:\n{xml}"
    );
    assert!(
        xml.contains(
            "<memory type=\"flash\" start=\"0x8000000\" length=\"0x10000\">\n    \
             <property name=\"blocksize\">0x4000</property>\n  </memory>"
        ),
        "the 16 KiB run is not written as the DTD has it:\n{xml}"
    );
    assert!(
        xml.contains("<memory type=\"rom\" start=\"0x1fff0000\" length=\"0x1000\"/>"),
        "a non-flash region should still be an empty element:\n{xml}"
    );
}

#[test]
fn a_board_with_no_array_has_no_flash_and_refuses_the_operations() {
    let mut machine = board("gdb-no-flash.machine", NO_FLASH);
    let mut target = MachineTarget::new(&mut machine);
    let map = target.memory_map(0).expect("a map");
    assert!(
        !map.iter().any(|r| r.kind.is_flash()),
        "nothing here has a loader's door: {map:?}"
    );
    // `Unsupported`, which the stub turns into an empty reply — "this stub
    // does not do that" — rather than into an error about a programming
    // sequence that never started.
    assert!(matches!(
        target.flash_erase(0, 0x0800_0000, 0x1000),
        Err(rsemu::host::gdb::TargetError::Unsupported)
    ));
    assert!(matches!(
        target.flash_write(0, 0x0800_0000, &[0u8; 4]),
        Err(rsemu::host::gdb::TargetError::Unsupported)
    ));
    assert!(matches!(
        target.flash_done(0),
        Err(rsemu::host::gdb::TargetError::Unsupported)
    ));

    // An ordinary debug write to the ROM does not reach it either, and that is
    // the rule this feature did not change: `core::space` enforces a
    // read-only mapping against a debug access too, so a `rom` store either
    // refuses the write or ignores it, and a loader gets in through a device's
    // door or not at all.
    let mut before = [0u8; 1];
    target
        .read_memory(0, 0x0800_0000, &mut before)
        .expect("the rom is readable");
    let _ = target.write_memory(0, 0x0800_0000, &[before[0] ^ 0xff]);
    let mut after = [0u8; 1];
    target
        .read_memory(0, 0x0800_0000, &mut after)
        .expect("the rom is readable");
    assert_eq!(after, before, "a debug write must not change a rom");
}

// ---------------------------------------------------------------------------
// The operations, through the target
// ---------------------------------------------------------------------------

#[test]
fn an_erase_clears_whole_sectors_and_leaves_them_reading_as_ones() {
    let mut machine = board("gdb-flash.machine", BOARD);
    let mut target = MachineTarget::new(&mut machine);

    // Program two sectors' worth of a recognisable pattern.
    let image: Vec<u8> = (0..(SECTOR0 as usize * 2))
        .map(|i| (i % 251) as u8)
        .collect();
    target
        .flash_write(0, FLASH, &image)
        .expect("the array takes a loader's write");
    target.flash_done(0).expect("done");

    // One byte of the first sector, erased. A part cannot clear half a sector,
    // so the whole 16 KiB goes — and the sector after it does not.
    target.flash_erase(0, FLASH + 1, 1).expect("erase");
    let mut back = vec![0u8; SECTOR0 as usize + 16];
    target.read_memory(0, FLASH, &mut back).expect("read back");
    assert!(
        back[..SECTOR0 as usize].iter().all(|b| *b == 0xff),
        "the sector the range touched is erased"
    );
    assert_eq!(
        &back[SECTOR0 as usize..],
        &image[SECTOR0 as usize..SECTOR0 as usize + 16],
        "the next sector is not"
    );
}

#[test]
fn a_flash_operation_outside_flash_is_refused_and_changes_nothing() {
    let mut machine = board("gdb-flash.machine", BOARD);
    let mut target = MachineTarget::new(&mut machine);

    target
        .write_memory(0, RAM, &[0x11, 0x22, 0x33, 0x44])
        .expect("ram takes an ordinary debug write");

    for (addr, what) in [(RAM, "ram"), (ROM, "rom")] {
        assert!(
            matches!(
                target.flash_write(0, addr, &[0xaa; 4]),
                Err(rsemu::host::gdb::TargetError::NotFlash)
            ),
            "{what} is not flash and must be refused as such"
        );
        assert!(
            matches!(
                target.flash_erase(0, addr, 0x100),
                Err(rsemu::host::gdb::TargetError::NotFlash)
            ),
            "{what} is not flash and must be refused as such"
        );
    }

    // A range that starts in flash and runs off the end of it is refused
    // *before* anything is written, not half way through.
    let last = FLASH + 1024 * 1024;
    assert!(matches!(
        target.flash_erase(0, last - SECTOR5, 2 * SECTOR5),
        Err(rsemu::host::gdb::TargetError::NotFlash)
    ));
    let mut tail = [0u8; 4];
    target
        .read_memory(0, last - 4, &mut tail)
        .expect("the last word is still readable");
    assert_eq!(tail, [0xff; 4], "an erased array, still erased");

    let mut ram = [0u8; 4];
    target.read_memory(0, RAM, &mut ram).expect("read ram");
    assert_eq!(ram, [0x11, 0x22, 0x33, 0x44], "the refusal wrote nothing");
}

// ---------------------------------------------------------------------------
// The operations, over the wire
// ---------------------------------------------------------------------------

/// How long a client waits for a reply before calling the server broken.
const REPLY_TIMEOUT: Duration = Duration::from_secs(20);

/// A GDB, near enough: framing, acknowledgements, and a transcript.
struct Client {
    stream: TcpStream,
    framer: Framer,
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

    /// Send one packet, escaping it exactly as `frame` does for GDB.
    fn send(&mut self, packet: &[u8]) {
        self.transcript
            .push(format!("-> {}", String::from_utf8_lossy(packet)));
        let mut wire = Vec::new();
        frame(packet, &mut wire);
        self.stream.write_all(&wire).expect("write");
        self.stream.flush().expect("flush");
    }

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

    /// Send a packet and return its reply as text.
    fn ask(&mut self, packet: &str) -> String {
        self.send(packet.as_bytes());
        String::from_utf8_lossy(&self.recv()).into_owned()
    }

    /// Send a packet whose payload is binary, and return its reply as text.
    fn ask_binary(&mut self, head: &str, data: &[u8]) -> String {
        let mut packet = Vec::from(head.as_bytes());
        packet.extend_from_slice(data);
        self.send(&packet);
        String::from_utf8_lossy(&self.recv()).into_owned()
    }
}

/// The gdbstub on a thread of its own, with the board behind it.
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
            .name(String::from("gdb-flash-test"))
            .spawn(move || {
                let mut server = server;
                let mut machine = board("gdb-flash.machine", BOARD);
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

/// Decode an `m` reply.
fn unhex(text: &str) -> Vec<u8> {
    assert!(
        text.len().is_multiple_of(2) && !text.starts_with('E'),
        "not a memory reply: {text}"
    );
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).expect("hex"))
        .collect()
}

#[test]
fn a_client_loads_an_image_into_flash_the_way_gdb_does() {
    let server = Server::start();
    let mut gdb = Client::connect(server.addr);

    // GDB decides to use the flash packets from the memory map alone, so the
    // map has to arrive before any of them makes sense.
    let supported = gdb.ask("qSupported:multiprocess+;swbreak+;xmlRegisters=arm");
    assert!(
        supported.contains("qXfer:memory-map:read+"),
        "the map is what turns `load` into `vFlash*`: {supported}"
    );
    let mut map = String::new();
    let mut offset = 0usize;
    loop {
        let reply = gdb.ask(&format!("qXfer:memory-map:read::{offset:x},100"));
        let (tag, body) = reply.split_at(1);
        map.push_str(body);
        offset += body.len();
        assert!(tag == "m" || tag == "l", "unexpected qXfer tag {tag}");
        if tag == "l" {
            break;
        }
    }
    assert!(map.contains("type=\"flash\""), "{map}");
    assert!(map.contains("<property name=\"blocksize\">0x4000"), "{map}");

    // The sequence itself: erase the first sector, write it in two pieces, and
    // say the sequence is over.
    assert_eq!(gdb.ask(&format!("vFlashErase:{FLASH:x},{SECTOR0:x}")), "OK");

    // A payload with every byte the framer has to escape (`}`, `#`, `$`, `*`)
    // and a run long enough for run-length encoding, because an image that
    // contained one of those and arrived wrong would be the defect this
    // session exists to catch.
    let first: Vec<u8> = vec![b'}', b'#', b'$', b'*', 0x00, 0x7d, 0xff, 0x42];
    let second: Vec<u8> = (0..64u8).map(|i| i.wrapping_mul(7)).collect();
    assert_eq!(
        gdb.ask_binary(&format!("vFlashWrite:{FLASH:x}:"), &first),
        "OK"
    );
    assert_eq!(
        gdb.ask_binary(
            &format!("vFlashWrite:{:x}:", FLASH + first.len() as u64),
            &second
        ),
        "OK"
    );
    assert_eq!(gdb.ask("vFlashDone"), "OK");

    // What the guest would fetch.
    let total = first.len() + second.len();
    let read = unhex(&gdb.ask(&format!("m{FLASH:x},{total:x}")));
    assert_eq!(
        &read[..first.len()],
        &first[..],
        "the escaped bytes survived"
    );
    assert_eq!(&read[first.len()..], &second[..]);

    // Past the image, the erased sector still reads as ones: `vFlashWrite`
    // wrote what it was given and nothing else.
    let tail = unhex(&gdb.ask(&format!("m{:x},4", FLASH + total as u64)));
    assert_eq!(tail, vec![0xff; 4]);
}

#[test]
fn the_protocol_refuses_a_flash_write_to_ordinary_memory() {
    let server = Server::start();
    let mut gdb = Client::connect(server.addr);
    let _ = gdb.ask("qSupported:multiprocess+");

    // `E.memtype` rather than an errno: it is what the protocol has for
    // exactly this, and GDB reports it as a flash operation on non-flash
    // memory instead of as a bus error the user has to go and find.
    assert_eq!(
        gdb.ask_binary(&format!("vFlashWrite:{RAM:x}:"), &[0xaa, 0xbb]),
        "E.memtype"
    );
    assert!(
        gdb.ask(&format!("vFlashErase:{ROM:x},1000"))
            .starts_with('E'),
        "the rom is not erasable"
    );
    // And RAM is untouched by the attempt.
    assert_eq!(unhex(&gdb.ask(&format!("m{RAM:x},2"))), vec![0x00, 0x00]);
}
