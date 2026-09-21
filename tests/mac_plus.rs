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
//! What the board still does not do is notice a disk once one is there: the ROM
//! sits in a two-byte loop at `$4006E8` with its interrupt mask at zero and
//! never moves the drive's soft switches, so `--floppy` changes nothing yet.
//! `docs/platforms/mac-plus.md` has the whole ledger, including what has been
//! ruled out and how.
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
use rsemu::host::display::mac::{MacScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// How long a Macintosh Plus ROM is. The socket is a 128 KiB part.
const ROM_LEN: usize = 128 * 1024;

/// The picture at 12 virtual seconds on the stock 1 MiB board.
const GOLDEN_1M: u64 = 0xfbc9_cfa0_9b09_a5da;

/// Where the insert-disk icon lands: a 32 × 32 box a little above the middle
/// of the 512 × 342 screen, found by watching which addresses the ROM draws it
/// through. Left, top, width, height.
const ICON: (u32, u32, u32, u32) = (240, 145, 32, 32);

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: MacScanout,
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
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-plus")
        .expect("this build ships mac-plus")
        .source;
    let machine = rsemu::machine::build("mac-plus", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a video circuit");
    Board {
        machine,
        cpu,
        scanout,
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
fn synthetic_800k() -> Vec<u8> {
    let mut image = vec![0u8; 819_200];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        for (i, byte) in chunk.iter_mut().enumerate() {
            *byte = (block as u8).wrapping_mul(7).wrapping_add(i as u8);
        }
    }
    image
}

/// The board assembles with a disk in the drive and runs exactly as it does
/// without one — which is the current state of affairs and worth an assertion
/// rather than a paragraph: the ROM does not look at the drive.
///
/// It gets as far as the insert-disk icon either way, and then stays there:
/// the icon is what the ROM draws when it has *not* found a disk, so a board
/// that noticed one would leave this screen rather than keep it.
///
/// When it starts looking, this is the test that changes: the frame will move
/// and the assertion below will be what says so.
#[test]
fn a_disk_in_the_drive_changes_nothing_yet() {
    let Some(image) = rom_image("Mac-Plus.ROM") else {
        return;
    };
    let mut b = board_with_disk(image, &[], synthetic_800k());
    advance(&mut b, "mac-plus-disk", 12);
    let hash = picture(&b, "mac-plus-disk", 12);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_eq!(
        hash, GOLDEN_1M,
        "the picture moved with a disk in the drive. If the ROM has started \
         looking at it, that is the insert-disk work landing and this golden \
         should move; look at it (RSEMU_MAC_FRAME_DIR) first."
    );
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
