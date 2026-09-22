//! A real Macintosh Plus ROM on the `mac-plus` board, run in place, as far as
//! it gets.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any Apple ROM.** The test reads the user's own ROM file in
//! place from `RSEMU_MAC_ROM_DIR` and skips, saying so, when it is unset. What
//! is asserted is about *this emulator*: that the processor is running, that
//! no access faulted, that the video circuit is producing frames, that the ROM
//! sized memory correctly and put its screen where the hardware puts it, and a
//! hash of the picture it produced at a fixed virtual time — our rendering,
//! not ROM contents.
//!
//! `RSEMU_MAC_FRAME_DIR`, when set, receives a PNG of each frame a test checks
//! (in a build with `display-png`), so a person can look at it.
//! `RSEMU_MAC_TRACE=1` prints the processor's state once a virtual second,
//! which is how a person finds where a ROM stopped.
//!
//! # How far it gets, and what the picture shows
//!
//! The ROM chimes (the sound buffer is filled and `/SNDENB` is asserted for 43
//! frames), sizes memory, runs its memory test, initialises the SCC and the
//! IWM, reads the clock chip and writes its parameter RAM back, finishes the
//! keyboard's Model Number handshake and settles into asking it for a key
//! every quarter second, and draws the Macintosh's 50 % grey desktop with the
//! arrow cursor in the top left corner. **It then draws the insert-disk icon —
//! the floppy with a question mark — in the middle of the screen** and waits
//! there with the 60.15 Hz tick chain going.
//!
//! The icon reaches the screen only because main memory **repeats** through its
//! four-megabyte window. The ROM draws it through a pointer near the top of
//! that window rather than through `ScrnBase`, and the fold is what puts that
//! on the middle of a 1 MiB machine's screen buffer. `src/dev/mac/glue.rs` has
//! the argument; before the fold the icon went into the floating half of the
//! window and the screen stayed empty.
//!
//! From about thirteen seconds on the icon **blinks** and the ROM polls the
//! drive's disk-in-place line six to eight times a second. Put an 800K image
//! in the drive and it starts the motor, **reads the track**, decodes the two
//! boot blocks out of it, finds no system on them and puts the disk back out.
//! `the_rom_reads_a_track_and_decodes_a_sector` is the assertion that Apple's
//! own code agrees with this encoder about the low-level format, and
//! `docs/platforms/mac-plus.md` has the whole ledger, including what is still
//! open and how it was measured.
//!
//! # The ROM file
//!
//! A Macintosh Plus ROM is **131,072 bytes** and the socket takes exactly
//! that. Files in circulation are often longer — the one this was developed
//! against is 138,576 bytes — so the test takes the first 128 KiB and checks
//! the ROM's own checksum longword over them before using it. That check is
//! arithmetic over bytes the test never keeps: the first longword of a
//! Macintosh ROM is the sum of every 16-bit word after it, which is a fact
//! about the format rather than any of its contents.
//!
//! # If a golden moves
//!
//! It is a whole-machine golden: any change to the 68000, the VIA or the video
//! circuit that alters what the ROM does can move it. Say which frame and why,
//! and look at the new picture (`RSEMU_MAC_FRAME_DIR`) before accepting a new
//! hash.
//!
//! No Macintosh emulator source was consulted and the ROM was not
//! disassembled (`ROADMAP.md` §1, `CLAUDE.md`).

#![cfg(feature = "machine-mac-plus")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::dev::mac::iwm::Iwm;
use rsemu::dev::mac::mouse::{MacMouse, Mouse};
use rsemu::dev::mac::scc::Scc;
use rsemu::host::display::mac::{MacScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::host::input::mac::MacMouseSink;
use rsemu::host::input::{InputEvent, InputSink};
use rsemu::machine::{Machine, catalog};

/// How long a Macintosh Plus ROM is. The socket is a 128 KiB part.
const ROM_LEN: usize = 128 * 1024;

/// The picture at 12 virtual seconds on the stock 1 MiB board, with nothing
/// in the drive.
///
/// The icon **blinks** once the ROM is in its insert-disk loop, so this is a
/// golden of one *phase* of that blink at one fixed virtual instant. Virtual
/// time is exact, so it is stable — but if it moves, check whether the other
/// phase (`0xfbc9_cfa0_9b09_a5da`, the same screen with the icon's inner
/// question mark the other way) is what came out before deciding anything is
/// wrong. `the_insert_disk_icon_blinks` is the test that asserts the blink
/// itself.
const GOLDEN_1M: u64 = 0x63dd_d76c_9468_dfa7;

/// Where the insert-disk icon lands: a 32 × 32 box a little above the middle
/// of the 512 × 342 screen, found by watching which addresses the ROM draws it
/// through. Left, top, width, height.
const ICON: (u32, u32, u32, u32) = (240, 145, 32, 32);

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: MacScanout,
    iwm: Arc<Iwm>,
    scc: Arc<Scc>,
    /// The mouse itself, caught as the board built it: a test that posts an
    /// exact number of **counts** has to reach past the absolute-pointer sink,
    /// which speaks pixels and keeps its own idea of where the host's cursor
    /// is.
    mouse: Arc<Mouse>,
    /// The pointer, as a person's own frontend reaches it: an `InputSink` that
    /// takes absolute framebuffer positions, over the host object `mac.mouse`
    /// opened for itself.
    pointer: MacMouseSink,
}

/// Read the ROM out of `RSEMU_MAC_ROM_DIR`; `None` (having said why) if the
/// variable or the file is not there.
///
/// A file longer than 128 KiB is trimmed to the ROM and the trailing bytes are
/// reported, because they are not part of the ROM and a socket has nowhere to
/// put them.
fn rom_image(file: &str) -> Option<Vec<u8>> {
    let Ok(dir) = std::env::var("RSEMU_MAC_ROM_DIR") else {
        println!(
            "mac-plus: set RSEMU_MAC_ROM_DIR to a directory holding Mac-Plus.ROM to boot a \
             real Macintosh ROM."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    let Ok(bytes) = std::fs::read(&path) else {
        println!("mac-plus: {} is not there; skipped", path.display());
        return None;
    };
    if bytes.len() < ROM_LEN {
        println!(
            "mac-plus: {} is {} bytes; a Plus ROM is {ROM_LEN}. Skipped.",
            path.display(),
            bytes.len()
        );
        return None;
    }
    if bytes.len() > ROM_LEN {
        println!(
            "mac-plus: {} is {} bytes; taking the first {ROM_LEN} and leaving {} behind.",
            path.display(),
            bytes.len(),
            bytes.len() - ROM_LEN
        );
    }
    let image = bytes[..ROM_LEN].to_vec();
    // The first longword of a Macintosh ROM is the sum of every 16-bit word
    // after it. If that holds, the image really does start at offset zero and
    // really is this long, which is what makes trimming safe.
    let stored = u32::from_be_bytes([image[0], image[1], image[2], image[3]]);
    let sum = image[4..].as_chunks::<2>().0.iter().fold(0u32, |acc, w| {
        acc.wrapping_add(u32::from(u16::from_be_bytes(*w)))
    });
    if stored != sum {
        println!(
            "mac-plus: {} does not checksum as a 128 KiB Macintosh ROM ({stored:#010x} stored, \
             {sum:#010x} computed); skipped.",
            path.display()
        );
        return None;
    }
    println!("mac-plus: {} checksums as {stored:#010x}", path.display());
    Some(image)
}

/// Build the board around `image`, with `params` on top of the defaults and
/// nothing in the drive.
fn board(image: Vec<u8>, params: &[(&str, &str)]) -> Board {
    board_with_disk(image, params, Vec::new())
}

/// The same, with `disk` in the internal drive. An empty vector is an empty
/// drive, which is what `rsemu run` binds when nobody says `--floppy`.
fn board_with_disk(image: Vec<u8>, params: &[(&str, &str)], disk: Vec<u8>) -> Board {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    // The disk controller and the SCC, caught as they are built: a test that
    // asks whether the motor is running or whether the chip let go of `/INT`
    // is asking about the *device*, and there is no other handle to it.
    let iwms: Arc<Captured<Iwm>> = Arc::new(Captured::new());
    let keep_iwm = Arc::clone(&iwms);
    options.bindings.replace("mac.iwm", move |props| {
        let iwm = Arc::new(Iwm::new(props)?);
        keep_iwm.push(&iwm);
        Ok(iwm)
    });
    let sccs: Arc<Captured<Scc>> = Arc::new(Captured::new());
    let keep_scc = Arc::clone(&sccs);
    options.bindings.replace("mac.scc", move |props| {
        let scc = Arc::new(Scc::new(props)?);
        keep_scc.push(&scc);
        Ok(scc)
    });
    let mice: Arc<Captured<MacMouse>> = Arc::new(Captured::new());
    let keep_mouse = Arc::clone(&mice);
    options.bindings.replace("mac.mouse", move |props| {
        let mouse = Arc::new(MacMouse::new(props)?);
        keep_mouse.push(&mouse);
        Ok(mouse)
    });
    capture::install(&mut options).expect("a capture table");
    for &(name, value) in params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    options.realize.media.insert("macrom", image);
    // The drive's bay. `machine::realize` refuses a slot that is named and
    // unbound, so a machine with an empty drive binds zero bytes for it —
    // which is what the CLI's own empty-bay pass does.
    options.realize.media.insert("floppy", disk);
    // And the SCSI cable, the same way: no bytes at `hd0` is an address nobody
    // answers at, which is a Plus with nothing plugged into the port on the
    // back. `tests/mac_plus_scsi.rs` is the file that puts a disk there.
    options.realize.media.insert("hd0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-plus")
        .expect("this build ships mac-plus")
        .source;
    let machine = rsemu::machine::build("mac-plus", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a video circuit");
    let pointer =
        MacMouseSink::open(&options.realize.hosts).expect("the board has a mouse on a host port");
    Board {
        machine,
        cpu,
        scanout,
        iwm: iwms.last().expect("the binding captured the controller"),
        scc: sccs.last().expect("the binding captured the SCC"),
        mouse: Arc::clone(mice.last().expect("the binding captured the mouse").mouse()),
        pointer,
    }
}

/// FNV-1a over the captured pixels: a hash of our rendering, nothing else.
fn frame_hash(surface: &Surface) -> u64 {
    surface
        .pixels()
        .iter()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        })
}

/// Run the board on to virtual second `to`, a second at a time.
fn advance(b: &mut Board, label: &str, to: u64) {
    let trace = std::env::var_os("RSEMU_MAC_TRACE").is_some();
    for s in 1..=to {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} frames={}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
            );
        }
    }
}

/// Hash the picture on screen now, and write it out as a PNG named for
/// `label` and `seconds` when `RSEMU_MAC_FRAME_DIR` says where.
fn picture(b: &Board, label: &str, seconds: u64) -> u64 {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let hash = frame_hash(&surface);
    if let Ok(dir) = std::env::var("RSEMU_MAC_FRAME_DIR") {
        #[cfg(feature = "display-png")]
        {
            let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
            let name = format!("{label}-{seconds}s.png");
            std::fs::write(std::path::Path::new(&dir).join(name), png)
                .expect("the frame directory is writable");
        }
        #[cfg(not(feature = "display-png"))]
        let _ = dir;
    }
    // How much of the screen is ink, which is a cheap description of what is on
    // it: a white screen is 0, the Macintosh's grey desktop is half of it.
    let ink = surface
        .pixels()
        .iter()
        .step_by(3)
        .filter(|&&b| b == 0)
        .count();
    println!(
        "{label} at {seconds}s: frame {hash:#018x}, {ink} black pixels of {}",
        info.width * info.height
    );
    hash
}

/// How many of the pixels in [`ICON`] are white.
///
/// The desktop behind it is a one-pixel checkerboard, so an empty box is
/// exactly half white; the icon is a solid white floppy with a black outline,
/// so a drawn one is most of the way to all of it. That is a description of
/// the picture rather than a second hash, and it is what tells a person
/// *which* picture moved when the golden does.
fn icon_white(b: &Board) -> usize {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let pixels = surface.pixels();
    let (left, top, w, h) = ICON;
    let mut white = 0;
    for y in top..top + h {
        for x in left..left + w {
            let at = ((y * info.width + x) * 3) as usize;
            if pixels.get(at).is_some_and(|&p| p != 0) {
                white += 1;
            }
        }
    }
    white
}

/// One longword of guest memory, read the way a debugger reads it.
fn peek(b: &Board, addr: u64) -> u32 {
    b.machine
        .space("mem")
        .expect("the board has `mem`")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .unwrap_or(0) as u32
}

/// The ROM on the stock 1 MiB board: it runs, sizes memory, draws the
/// Macintosh's grey desktop with the arrow cursor in the top left corner, and
/// puts the insert-disk icon in the middle of it.
///
/// The picture is described in the module docs and in
/// `docs/platforms/mac-plus.md`. Twelve virtual seconds is well past the point
/// where it stops changing — the memory test finishes at about six and the
/// picture is settled by eight.
#[test]
fn the_rom_boots_to_the_insert_disk_screen() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board(image, &[]);
    advance(&mut b, "mac-plus", 12);
    let hash = picture(&b, "mac-plus", 12);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(
        b.scanout.frame_counter() >= 12 * 59,
        "the video circuit is producing frames: {}",
        b.scanout.frame_counter()
    );

    // What the ROM wrote into low memory, which is data it built rather than
    // code it ran. `MemTop` is how much memory it decided there was, and
    // `ScrnBase` is where it decided the screen goes — both are the decoder's
    // behaviour reported back by the guest, and the memory test loops for ever
    // when either is wrong.
    assert_eq!(peek(&b, 0x108), 0x0010_0000, "MemTop: a 1 MiB machine");
    assert_eq!(peek(&b, 0x824), 0x000f_a700, "ScrnBase: MemTop - $5900");
    assert_eq!(peek(&b, 0x2ae), 0x0040_0000, "ROMBase");
    // The 60.15 Hz tick chain is running: `Ticks` counts vertical blanking
    // interrupts, so a plausible number here is the VIA, the video circuit and
    // the processor's interrupt path all working together.
    let ticks = peek(&b, 0x16a);
    assert!(
        (100..=800).contains(&ticks),
        "Ticks is counting vertical blanking: {ticks}"
    );

    // The insert-disk icon really is on the screen, and not merely a hash that
    // happens to match: the box the ROM draws it in is mostly white, where the
    // grey desktop alone would be exactly half.
    let white = icon_white(&b);
    println!("mac-plus: {white} of {} icon pixels are white", 32 * 32);
    assert!(
        white > 700,
        "the insert-disk icon is not on the screen: {white} of {} pixels white, and an empty \
         desktop is {}",
        32 * 32,
        32 * 32 / 2
    );

    assert_eq!(
        hash, GOLDEN_1M,
        "the frame at 12s moved; look at it (RSEMU_MAC_FRAME_DIR) before accepting the new hash"
    );
}

/// The same ROM on a 4 MiB board: it sizes the expansion and moves its screen
/// buffer with it, which is the whole of what `-p ram=4M` has to do.
///
/// No golden: the picture is the same screen and the point of the test is the
/// two numbers the ROM worked out for itself. Thirty-five seconds, because a
/// memory test of four megabytes takes four times as long as one of one — the
/// icon is not up until about twenty-six.
#[test]
fn the_rom_sizes_a_four_megabyte_board_and_moves_its_screen() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board(image, &[("ram", "4M")]);
    advance(&mut b, "mac-plus-4m", 35);
    let _ = picture(&b, "mac-plus-4m", 35);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_eq!(peek(&b, 0x108), 0x0040_0000, "MemTop: a 4 MiB machine");
    assert_eq!(peek(&b, 0x824), 0x003f_a700, "ScrnBase: MemTop - $5900");
    // The icon lands in the same place on the screen whatever the memory size,
    // which is the whole point of the window folding: the ROM's pointer is a
    // constant near the top of the window and the top of the window is the top
    // of memory on every board.
    let white = icon_white(&b);
    assert!(
        white > 700,
        "the insert-disk icon is not on a 4 MiB board's screen either: {white} of {} white",
        32 * 32
    );
}

/// A synthetic 800K disk: every block says which block it is, so nothing of
/// anybody's is needed and nothing of anybody's is committed.
///
/// The block number goes in the first four bytes so that a block found in
/// guest memory can be *named*, and the rest is a fill keyed on it so that no
/// two blocks are alike. There is no system on it and no boot block, which is
/// why the ROM puts it back out again.
fn synthetic_800k() -> Vec<u8> {
    let mut image = vec![0u8; 819_200];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
        for (i, byte) in chunk.iter_mut().enumerate().skip(4) {
            *byte = (block as u8).wrapping_mul(7).wrapping_add(i as u8);
        }
    }
    image
}

/// Which blocks of `image` are sitting whole in the machine's memory.
///
/// Every block of [`synthetic_800k`] begins with its own number, so a
/// candidate is found by that longword and then confirmed against all 512
/// bytes — nothing is claimed on four bytes alone.
fn blocks_in_memory(b: &Board, image: &[u8]) -> Vec<u32> {
    let space = b.machine.space("mem").expect("the board has `mem`");
    let size = 0x10_0000usize;
    let mut ram = vec![0u8; size];
    for (a, byte) in ram.iter_mut().enumerate() {
        *byte = space
            .read(a as u64, Width::U8, MemAttrs::DEBUG)
            .unwrap_or(0) as u8;
    }
    let blocks = (image.len() / 512) as u32;
    let mut found = Vec::new();
    for at in 0..ram.len().saturating_sub(512) {
        let n = u32::from_be_bytes([ram[at], ram[at + 1], ram[at + 2], ram[at + 3]]);
        if n >= blocks {
            continue;
        }
        let want = &image[n as usize * 512..(n as usize + 1) * 512];
        if &ram[at..at + 512] == want && !found.contains(&n) {
            found.push(n);
        }
    }
    found.sort_unstable();
    found
}

/// **Apple's ROM reads a track this encoder wrote and decodes sectors out of
/// it**, and that is the only thing that can settle the low-level format.
///
/// `src/dev/mac/gcr.rs` and its decoder are each other's oracle: they would
/// agree with each other about which two bits of a byte go where in a 6-and-2
/// group, and about which of the three sums scrambles which byte, even if both
/// were wrong in the same way. A real ROM would not. So the disk goes in as
/// 1,600 numbered blocks, the ROM is left to read it, and the assertion is
/// that **the 512 bytes of a block of the image turn up whole in the
/// machine's memory** — which cannot happen unless Apple's own code found the
/// address field, found the data field, denibblized it, and checked the
/// patent's three-byte checksum over it.
///
/// It reads **blocks 0 and 1**: the boot blocks, which is what a Macintosh
/// reads first and all it needs to read to decide this disk has no system on
/// it.
///
/// This was ledger item 1 and it needed two things. The cylinders were 2.5 %
/// short, so the spindle turned 2.5 % fast and the ROM's own speed check threw
/// the disk out before it would read a byte (`gcr::SECTOR_CELLS`); and the
/// chip named no event, so a scheduler round ran on past a byte boundary and
/// the ROM was handed one byte in three (`Iwm::next_event_tick`).
/// `docs/platforms/mac-plus.md` has both measurements.
#[test]
fn the_rom_reads_a_track_and_decodes_a_sector() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let disk = synthetic_800k();
    let mut b = board_with_disk(image, &[], disk.clone());
    advance(&mut b, "mac-plus-read", 12);
    let found = blocks_in_memory(&b, &disk);
    println!("mac-plus: blocks of the image found whole in memory: {found:?}");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(
        found.contains(&0) && found.contains(&1),
        "the ROM did not decode the two boot blocks off the disk; it found {found:?}"
    );
}

/// **The ROM spins the drive up and reads it**, and when what is on it is not
/// a system disk it puts it back out and goes on asking.
///
/// This is what the drive-register table being right buys, and it is the
/// assertion that would fail if it went wrong again: the ROM tests the
/// drive-installed line before it will touch the mechanism at all, and while
/// that line said "no drive" it drew the insert-disk icon and never turned the
/// motor, whatever was in the slot (`docs/platforms/mac-plus.md`).
///
/// The disk here is 800K of numbered blocks — no boot blocks, no System file —
/// so being put back out is the right answer. What is asserted is the
/// *mechanism*: the motor turned, and the disk came out again.
///
/// It used to come out with the **cross** through it, the unreadable-disk
/// icon, because the ROM could not get a track off it. It now comes out with
/// the blinking question mark, which is a disk that was read and had no system
/// on it — a different picture for a different reason.
/// `the_rom_reads_a_track_and_decodes_a_sector` is the one that asserts the
/// reading.
#[test]
fn the_rom_spins_the_drive_up_and_rejects_a_disk_with_no_system_on_it() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board_with_disk(image, &[], synthetic_800k());
    assert!(b.iwm.has_disk(0), "the image went into the drive");

    // Up to the point where the ROM has finished with the drive: the motor
    // starts at about eight virtual seconds and the eject is at about
    // fourteen.
    let mut ran = false;
    for _ in 1..=13 {
        advance(&mut b, "mac-plus-disk", 1);
        ran |= b.iwm.motor(0);
    }
    assert!(ran, "the ROM never started the drive's motor");
    advance(&mut b, "mac-plus-disk", 3);
    let _ = picture(&b, "mac-plus-disk", 16);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(
        !b.iwm.has_disk(0),
        "the ROM found no system on the disk and left it in the drive"
    );
    assert!(!b.iwm.motor(0), "and stopped the motor after it");
}

/// And then it **polls the drive**, which it did not do before.
///
/// While the insert-disk icon is up the ROM asks the drive's disk-in-place
/// line six to eight times a second. That is what ledger item 1 was about: the
/// machine used to draw "insert a disk" and then never look.
///
/// Asserted through the picture rather than through a count of register
/// accesses, because a count of accesses is only a count of accesses: the icon
/// **blinks**, and an icon that blinks is a live insert-disk loop rather than a
/// processor parked in a two-byte one.
#[test]
fn the_insert_disk_icon_blinks() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board(image, &[]);
    advance(&mut b, "mac-plus-blink", 14);
    let mut seen = Vec::new();
    for s in 15..=20 {
        advance(&mut b, "mac-plus-blink", 1);
        seen.push(icon_white(&b));
        let _ = picture(&b, "mac-plus-blink", s);
    }
    println!("mac-plus: icon white counts second by second: {seen:?}");
    assert!(
        seen.iter().any(|&n| n != seen[0]),
        "the insert-disk icon does not blink: {seen:?}"
    );
    assert!(
        seen.iter().all(|&n| n > 700),
        "and it is an icon throughout, not a bare desktop: {seen:?}"
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
}

/// **A carrier-detect transition does not lock the machine up.**
///
/// A Macintosh Plus's mouse drives the SCC's two carrier detects, and the ROM
/// leaves them armed — `WR1 = $01`, `WR9 = $0A`, `WR15 = $08`. Moving one used
/// to put the processor into the level-2/3 handler and keep it there for ever:
/// `Reset Ext/Status Interrupts` cleared the chip's state but never
/// re-announced `/INT`, so the handler did everything the Z8530 manual asks of
/// it and was entered again immediately. `src/dev/mac/scc.rs` has the
/// argument.
///
/// This is the machine-level half of it: drive both carrier detects and check
/// that the ROM is back in its idle loop with the tick chain still counting.
#[test]
fn a_carrier_detect_transition_does_not_lock_the_machine_up() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board(image, &[]);
    advance(&mut b, "mac-plus-mouse", 16);
    let before = peek(&b, 0x16a);
    // `MTemp` is where the ROM accumulates the mouse's position. It moving is
    // what says the quadrature reached the handler and was decoded, rather
    // than merely survived.
    let rest = peek(&b, 0x828);

    // Both of them, one after the other — the two axes of a mouse. Each axis
    // goes down and comes back, and with the other phase of its quadrature
    // (the VIA's `PB4`/`PB5`) standing still the ROM counts a step each way,
    // so `MTemp` moves and then moves back. What is asserted is that it moved
    // at all *and* that the machine kept running through it.
    let mut moved = false;
    for channel in [0usize, 1] {
        for level in [false, true] {
            b.scc.set_dcd(channel, level);
            advance(&mut b, "mac-plus-mouse", 1);
            let mtemp = peek(&b, 0x828);
            println!("mac-plus: channel {channel} to {level}: MTemp = {mtemp:#010x}");
            moved |= mtemp != rest;
        }
    }

    let ticks = peek(&b, 0x16a);
    assert!(
        ticks > before + 200,
        "the 60 Hz tick chain stopped: {before} to {ticks}. The processor is \
         stuck in the SCC's interrupt handler."
    );
    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(moved, "MTemp never moved: the transitions reached nothing");
    let _ = picture(&b, "mac-plus-mouse", 20);
}

/// A 1.44 MB image is refused when the board is assembled, by name, with a
/// message that says why — rather than being handed to a drive that cannot read
/// it and failing somewhere further down.
///
/// The image here is 1,474,560 zero bytes built on the spot. Nobody's disk is
/// in this repository and this test needs none.
#[test]
fn a_1440k_image_is_refused_when_the_board_is_built() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    options.realize.media.insert("macrom", image);
    options.realize.media.insert("floppy", vec![0u8; 1_474_560]);
    options.realize.media.insert("hd0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-plus")
        .expect("this build ships mac-plus")
        .source;
    let Err(err) = rsemu::machine::build("mac-plus", source, &registry, &options) else {
        panic!("a Macintosh Plus cannot read a 1.44 MB disk");
    };
    let text = err.to_string();
    println!("mac-plus: {text}");
    for want in ["1.44 MB", "IWM", "SWIM", "800K"] {
        assert!(
            text.contains(want),
            "the refusal does not say `{want}`: {text}"
        );
    }
}

/// How far in from each edge [`arrow_mark`] starts looking.
///
/// The Macintosh desktop has **rounded corners** — the four corners are solid
/// ink, five or six pixels of it — and a run-of-ink detector walks straight
/// into them. Eight pixels clears all four, and the cursor's own rest position
/// is well inside.
const DESKTOP_INSET: u32 = 8;

/// The topmost, then leftmost, start of **four consecutive black pixels** in
/// the picture, ignoring [`DESKTOP_INSET`] pixels of every edge.
///
/// The desktop is a one-pixel checkerboard, so no two horizontally adjacent
/// pixels on it are both ink: a run of four is the cursor, or the insert-disk
/// icon's outline a hundred rows further down. Searching from the top finds the
/// cursor whenever it is above the icon, which is where these tests put it.
///
/// A better instrument than a whole-frame hash for this job, because it says
/// *where* the pointer is rather than that the picture changed.
fn arrow_mark(b: &Board) -> Option<(u32, u32)> {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let pixels = surface.pixels();
    let ink = |x: u32, y: u32| -> bool {
        let at = ((y * info.width + x) * 3) as usize;
        pixels.get(at).is_some_and(|&p| p == 0)
    };
    for y in DESKTOP_INSET..info.height - DESKTOP_INSET {
        for x in DESKTOP_INSET..info.width - DESKTOP_INSET - 3 {
            if (0..4).all(|d| ink(x + d, y)) {
                return Some((x, y));
            }
        }
    }
    None
}

/// Post an absolute pointer position through the host seam and let the guest
/// catch up: `ms` milliseconds of virtual time, a hundred at a time.
fn point_at(b: &mut Board, x: u32, y: u32, buttons: u8, ms: u64) {
    b.pointer.deliver(InputEvent::Pointer { x, y, buttons });
    for _ in 0..ms / 100 {
        b.machine
            .run_for(GlobalTime::from_nanos(100_000_000))
            .expect("it runs");
    }
}

/// **The pointer goes where it is put.**
///
/// The acceptance test for `mac.mouse`, and it is about the *picture*: the
/// arrow the ROM draws is found where the host sent the pointer, not merely
/// somewhere else than it was.
///
/// How the two ends are made to agree, which is the whole difficulty with a
/// relative mouse and an absolute host cursor:
///
/// 1. One event establishes where the host's cursor is and moves nothing
///    (`host::input::mac`).
/// 2. A sweep to the screen's top left corner **pins** the guest's pointer
///    there — the ROM clamps it to the screen, so however far the sweep
///    overshoots, both ends finish at `(0, 0)`. `amiga_a500_workbench.rs` uses
///    the bottom right corner for the same reason.
/// 3. From there a host position *is* the guest pointer's position, at
///    `PIXELS_PER_COUNT` of one, and a placement is checked by finding the
///    arrow in the frame.
///
/// The arrow's mark — the top-left of its first four-pixel run of ink — sits
/// at `(h, v + 3)` for a hot spot of `(h, v)`, which is a fact about the
/// cursor Apple's ROM draws and is asserted as one.
///
/// The second thing this asserts is that the machine is still *alive*
/// afterwards, with the tick chain counting: the first mouse to move on this
/// board livelocked it inside the SCC's interrupt handler, and
/// `src/dev/mac/glue.rs` has what that was.
#[test]
fn the_pointer_goes_where_it_is_put() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board(image, &[]);
    advance(&mut b, "mac-plus-pointer", 16);

    // Where the ROM leaves the cursor at boot.
    let rest = arrow_mark(&b).expect("the arrow is on the desktop");
    println!("mac-plus: the arrow rests at {rest:?}");
    assert_eq!(rest, (15, 18), "the boot cursor, hot spot (15, 15)");
    let ticks_before = peek(&b, 0x16a);

    // Every target is above the insert-disk icon's rows and inside
    // `DESKTOP_INSET`, so the mark the search finds is the cursor's and nothing
    // else's — and each one is reached from the corner rather than from the
    // last, which is the part worth explaining.
    //
    // **The ROM loses about one count in three hundred, and it is the ROM's.**
    // Every transition reaches the chip, latches, pulls `/INT` and is
    // serviced — 400 of 400 at every step rate, which
    // `every_count_the_mouse_sends_reaches_the_roms_handler` asserts with no
    // tolerance at all — but the cursor VBL task reads `MTemp`, scales it and
    // "also updates MTemp to reflect the new value" (Apple Technical Note
    // DV 520) at interrupt mask 0, so a count the handler adds inside that
    // window is overwritten. A real Macintosh loses it too. That error does
    // not cancel, so a run of placements measured from each other would drift
    // a pixel every few hundred counts and this test would be asserting the
    // drift rather than the tracking. Re-pinning on the corner is what a
    // person does without thinking about it, and it is what
    // `amiga_a500_workbench.rs` does with the *bottom* right one.
    // One event to establish where the host's cursor is, which moves nothing.
    point_at(&mut b, 400, 300, 0, 100);
    for (x, y) in [(100u32, 80u32), (37, 121), (300, 40), (470, 100)] {
        // Sweep off the top left corner: the ROM clamps the pointer to the
        // screen, so however far the sweep overshoots both ends finish at
        // (0, 0). Then place it, from a position the two agree on.
        point_at(&mut b, 0, 0, 0, 4_000);
        point_at(&mut b, x, y, 0, 3_000);
        let at = arrow_mark(&b).expect("the arrow is drawn");
        let m = peek(&b, 0x830);
        println!(
            "mac-plus: pointer sent to ({x}, {y}), the ROM has it at ({}, {}), the arrow is at {at:?}",
            m as u16,
            (m >> 16) as u16
        );
        assert!(
            at.0.abs_diff(x) <= 1 && at.1.abs_diff(y + 3) <= 1,
            "the pointer is not where it was sent: ({x}, {y}) should put the \
             arrow's mark at ({x}, {}), and it is at {at:?}",
            y + 3
        );
    }

    // The button, which the ROM records in `MBState` at $172: $00 down, $80 up.
    point_at(&mut b, 470, 100, 1, 500);

    let down = peek(&b, 0x172) >> 24;
    point_at(&mut b, 470, 100, 0, 500);
    let up = peek(&b, 0x172) >> 24;
    println!("mac-plus: MBState reads {down:#04x} with the button down, {up:#04x} with it up");
    assert_ne!(down, up, "the ROM did not see the button");
    assert_eq!(down, 0x00, "the Guide: PB3 low is the button down");
    assert_eq!(up, 0x80);

    // Still alive, which is the regression for the livelock.
    let ticks = peek(&b, 0x16a);
    assert!(
        ticks > ticks_before + 600,
        "the 60 Hz tick chain stopped: {ticks_before} to {ticks}. A moving mouse          used to park the processor in the level-3 handler for ever."
    );
    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    let _ = picture(&b, "mac-plus-pointer", 30);
}

/// **Every count the mouse sends becomes an interrupt the ROM services**, at
/// every step rate, and the only thing that ever goes missing afterwards is
/// Apple's own.
///
/// This is the test that settled a claim this board carried for a while: that
/// the SCC dropped about one carrier-detect transition in two hundred, because
/// "an edge arriving while the processor is in the level-2 handler with the VIA
/// also waiting is an edge nothing counts". It does not. Counted at four
/// places along the path — the pin, the chip's external/status latch, the
/// `/INT` wire, and the `Reset Ext/Status Interrupts` the guest's handler
/// writes — **all four numbers are the number of counts posted**, exactly, at
/// every rate. `mac.scc`'s [`Counters`](rsemu::dev::mac::scc::Counters) are
/// those four places.
///
/// What is short is `Mouse`, by one or two in four hundred, and that is the
/// ROM's own arithmetic rather than a lost interrupt. Apple documents the
/// mechanism in Technical Note DV 520,
/// *Device Management Overview Q&As*:
///
/// > "When the mouse has new information, it interrupts the Macintosh. The
/// > interrupt handler adds the horizontal and vertical counts to MTemp (a
/// > low-memory location), and sets crsrNew to tell the system that the
/// > coordinates are new."
///
/// > "Some time later (but before normal VBLs are executed) the cursor VBL task
/// > is executed, and it compares MTemp with RawMouse (which has the last
/// > value), and figures out the delta ... **It also updates MTemp** to reflect
/// > the new value. Then it draws the cursor."
///
/// Those two pieces of code share `MTemp` with no interlock, and the cursor
/// task runs at interrupt mask **0** — sampling `SR` through it says `$2004`.
/// So a count the handler adds after the task has read `MTemp` and before it
/// writes it back is overwritten, and a real Macintosh Plus loses it for the
/// same reason. Caught in the act on this board, ten microseconds a sample:
///
/// ```text
///   pc=401b28 sr=2004  MTemp=0085 RawMouse=0082   (the cursor task, mask 0)
///   pc=401ece sr=2004  MTemp=0085 RawMouse=0085   it has read MTemp
///   pc=401a88 sr=2204  MTemp=0085 RawMouse=0085   a carrier detect moved:
///   pc=401ade sr=2204  MTemp=0085 RawMouse=0085     the level-2 handler
///   pc=401bec sr=2200  MTemp=0086 RawMouse=0085     and it counted: 85 -> 86
///   pc=401f34 sr=2009  MTemp=0086 RawMouse=0085   back in the task
///   pc=401eee sr=2004  MTemp=0085 RawMouse=0085   which writes MTemp back
/// ```
///
/// The window is some forty microseconds of a 16.6 ms tick, which is the one
/// in two to four hundred that comes out of the far end. Hence the allowance
/// here — a *measured* bound on Apple's window, not a tolerance over an
/// unexplained loss — and hence the sweep into the corner that
/// `the_pointer_goes_where_it_is_put` still does before each placement.
#[test]
fn every_count_the_mouse_sends_reaches_the_roms_handler() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    // Counts on one axis per rate. 400 from the boot cursor's x = 15 stays
    // inside the 512-pixel screen, so the ROM's clamp never touches it.
    const COUNTS: i32 = 400;
    // Ticks a step for each board. Every one of them keeps the ROM under its
    // own scaling threshold — six counts in one 60.15 Hz tick is doubled —
    // which is what makes a count a pixel; 1 200 would not.
    const RATES: [u64; 6] = [2_400, 3_000, 4_000, 5_000, 6_000, 12_000];
    let mut exact = 0;
    for rate in RATES {
        let mut b = board(image.clone(), &[("mousestep", &rate.to_string())]);
        advance(&mut b, "mac-plus-counts", 16);
        let before = b.scc.counters();
        let at = peek(&b, 0x830);
        b.mouse.report(COUNTS, 0, 0);
        // The whole backlog at this rate, and two seconds for the ROM to draw
        // the last of it.
        let ms = 2 * u64::from(COUNTS.unsigned_abs()) * rate / 1_000 + 2_000;
        for _ in 0..ms / 100 {
            b.machine
                .run_for(GlobalTime::from_nanos(100_000_000))
                .expect("it runs");
        }
        assert_eq!(b.mouse.backlog(), (0, 0), "the mouse clocked it all out");
        let now = b.scc.counters();
        let then = peek(&b, 0x830);
        let moved = i64::from(then as u16) - i64::from(at as u16);
        let count = |after: u64, before: u64| i64::try_from(after - before).expect("a count");
        let edges = count(now.dcd_edges[0], before.dcd_edges[0]);
        let latches = count(now.ext_latches[0], before.ext_latches[0]);
        let ints = count(now.int_assertions, before.int_assertions);
        let serviced = count(now.ext_resets[0], before.ext_resets[0]);
        println!(
            "mac-plus: step-ticks {rate}: {COUNTS} counts -> {edges} transitions on DCDA, \
             {latches} external/status latches, {ints} /INT assertions, {serviced} serviced \
             by the ROM; Mouse moved {moved}"
        );
        assert_eq!(edges, i64::from(COUNTS), "the mouse sent every count");
        assert_eq!(latches, i64::from(COUNTS), "the chip latched every one");
        assert_eq!(ints, i64::from(COUNTS), "and asked for every one");
        assert_eq!(
            serviced,
            i64::from(COUNTS),
            "and the ROM serviced every one"
        );
        assert_eq!(
            count(now.dcd_edges[1], before.dcd_edges[1]),
            0,
            "one axis moved, so the other carrier detect stood still"
        );
        assert_eq!(
            (then >> 16) as u16,
            (at >> 16) as u16,
            "and the vertical coordinate did not move at all"
        );
        assert!(
            moved <= i64::from(COUNTS),
            "the ROM moved the pointer further than it was pushed: {moved} of {COUNTS}. \
             Six counts in one tick is doubled, so this is the scaling threshold."
        );
        // Apple's window, above: at most one count per execution of the cursor
        // VBL task. Measured on this board: one at 2 400, 3 000, 4 000, 5 000
        // and 6 000 ticks a step, none at 12 000.
        assert!(
            i64::from(COUNTS) - moved <= i64::from(COUNTS) / 100,
            "the ROM is short by {} of {COUNTS}, which is more than its own \
             MTemp window explains",
            i64::from(COUNTS) - moved
        );
        exact += i64::from(i64::from(COUNTS) == moved);
    }
    println!(
        "mac-plus: {}/{} of the rates land the pointer exactly; every rate delivers, latches, \
         raises and services {COUNTS} of {COUNTS}",
        exact,
        RATES.len()
    );
}
