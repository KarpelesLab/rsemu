//! Real Kickstarts on the A500, run in place, to their insert-disk screens —
//! and one of them on to the Workbench desktop.
//!
//! # Why this file exists
//!
//! The milestone for the joined A500 chipset is a real ROM drawing the screen a
//! real A500 draws with no disk in the drive. Nothing synthetic proves the same
//! thing: that screen needs the copper, bitplanes, the blitter's line and
//! area modes and interrupts all working together on code nobody here wrote.
//! Getting there took five fixes the synthetic suites could not have found —
//! each is documented where it was made:
//!
//! | ROM | Stopped at | Fixed in |
//! | --- | --- | --- |
//! | 2.04 | a byte read of `$DFF07D` | `src/dev/amiga/custom.rs` (byte access) |
//! | 1.3 | the board would not build around a 256 KiB ROM | `src/dev/amiga/gary.rs` |
//! | 2.04, 3.1 | graphics' genlock probe waiting on a one-shot timer that never started | `src/dev/mos/cia.rs` |
//! | 3.1 | `AN_MemCorrupt`: a blitter line walked its D pointer by `BLTDMOD` | `src/dev/amiga/agnus/blitter.rs` |
//! | 1.3, AROS | a copper sent into `ExecBase` wrote `INTENA` | `src/dev/amiga/agnus/copper.rs` |
//! | 2.04 + Workbench 2.04 | `AN_MemCorrupt` at ~38 s, then (once past it) a desktop sheared a word a line | `agnus/blitter.rs` as above; `agnus/display.rs` (high-resolution fetch) |
//! | 1.3 + Workbench 1.3 | the insert-disk screen forever: the drive's head never left cylinder 0 | `src/dev/amiga/floppy.rs` (`STEP*` trailing edge) |
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM.** Every test reads the user's own ROM files in place
//! from `RSEMU_AMIGA_ROM_DIR` and skips, saying so, when it is unset. What is
//! asserted is about *this emulator*: that the processor is running, that no
//! access faulted, and a hash of the picture Denise produced at a fixed virtual
//! time — our rendering of the screen, not ROM contents. The screens are
//! described in words in `docs/platforms/amiga.md`.
//!
//! The Workbench test also needs `RSEMU_AMIGA_ADF_DIR`, Amiga Forever's
//! `Shared/adf`, and reads the disk in place too. Nothing of it is asserted
//! beyond the picture this emulator draws once it has booted.
//!
//! `RSEMU_AMIGA_FRAME_DIR`, when set, receives a PNG of each frame a test
//! checks (in a build with `display-png`), so a person can look at it.
//!
//! The board is the shipped `machines/amiga-a500.machine`, whose ROM socket is
//! a 512 KiB mirror — which is what lets a 256 KiB Kickstart 1.3 build at all —
//! with its drive empty.
//!
//! # If a golden moves
//!
//! The hashes are whole-machine goldens: any change to the 68000, a CIA or a
//! custom chip that alters what the ROM does can move them. Say which board,
//! which frame and why, and look at the new picture before accepting it. The
//! 2.04 and 3.1 screens animate — the disk slides into the drive over and
//! over — so a change in boot timing moves those two by itself.
//!
//! No Amiga emulator source and no AROS source was consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a500", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
}

/// Build the board around the user's ROM `file`, or `None` if it is not there.
///
/// The drive is empty unless `df0` brings a disk.
fn board(file: &str, df0: Vec<u8>) -> Option<Board> {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ROM_DIR") else {
        println!(
            "amiga-a500: set RSEMU_AMIGA_ROM_DIR to an Amiga Forever `Shared/rom` directory \
             to run real Kickstarts."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    if !path.exists() {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return None;
    }
    let image = rsemu::host::media::kickstart::open(&path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let size = image.bytes.len();

    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    capture::install(&mut options).expect("a capture table");
    options
        .resolve
        .params
        .push(("kickstart-size".to_string(), format!("{}K", size / 1024)));
    options.realize.media.insert("kickstart", image.bytes);
    // No bytes is an empty drive: the insert-disk screen is what a Kickstart
    // shows with nothing in DF0.
    options.realize.media.insert("df0", df0);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source;
    let machine = rsemu::machine::build("amiga-a500", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{file}: the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a Denise");
    Some(Board {
        machine,
        cpu,
        scanout,
    })
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

/// Run `file` to `seconds` of virtual time and hash the picture there.
///
/// `RSEMU_AMIGA_TRACE=1` prints the processor's state once a second on the
/// way, which is how a person finds where a ROM stopped.
fn run_to(file: &str, df0: Vec<u8>, seconds: u64) -> Option<(Board, u64)> {
    let mut b = board(file, df0)?;
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in 1..=seconds {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{file} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} fields={}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
            );
        }
    }
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let hash = frame_hash(&surface);
    if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
        #[cfg(feature = "display-png")]
        {
            let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
            let name = format!("{}-{seconds}s.png", file.trim_end_matches(".rom"));
            std::fs::write(std::path::Path::new(&dir).join(name), png)
                .expect("the frame directory is writable");
        }
        #[cfg(not(feature = "display-png"))]
        let _ = dir;
    }
    println!("{file} at {seconds}s: frame {hash:#018x}");
    Some((b, hash))
}

/// The checks every ROM that reaches its screen must pass, then the golden.
fn reaches_its_screen(file: &str, df0: Vec<u8>, seconds: u64, golden: u64) {
    let Some((b, hash)) = run_to(file, df0, seconds) else {
        return;
    };
    assert!(!b.cpu.is_halted(), "{file}: the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "{file}: an access faulted");
    assert!(
        b.scanout.frame_counter() >= seconds * 49,
        "{file}: Denise is producing fields"
    );
    assert_eq!(
        hash, golden,
        "{file}: the frame at {seconds}s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before \
         accepting the new hash"
    );
}

/// Kickstart 1.3 (34.5), 256 KiB: the hand holding a Workbench 1.3 disk.
#[test]
fn kickstart_1_3_draws_the_hand_and_disk() {
    reaches_its_screen("amiga-os-130.rom", Vec::new(), 12, GOLDEN_130);
}

/// Kickstart 2.04 (37.175): the check mark, the drive and the disk going in.
#[test]
fn kickstart_2_04_draws_the_insert_disk_animation() {
    reaches_its_screen("amiga-os-204.rom", Vec::new(), 28, GOLDEN_204);
}

/// Kickstart 3.1 (40.063, the A500/A600/A2000 part): the same picture.
#[test]
fn kickstart_3_1_draws_the_insert_disk_animation() {
    reaches_its_screen("amiga-os-310-a600.rom", Vec::new(), 12, GOLDEN_310);
}

/// Kickstart 2.04 with the Workbench 2.04 disk in DF0, both read in place: the
/// AmigaDOS shell window, `LoadWB`, and the Workbench desktop — a blue-framed
/// "Workbench" window holding the Ram Disk and Workbench2.0 icons, on a grey
/// 640-pixel high-resolution screen, with the red pointer at the top left.
///
/// Sixty-two virtual seconds, the least that proves it: the disk is read until
/// about 57 s, and the picture has settled by 60. Before the blitter's line
/// mode stepped D by `BLTCMOD` this run stopped with `AN_MemCorrupt` at about
/// 38 s; before the high-resolution fetch counted in eight-count blocks it
/// reached the desktop sheared a word a line.
#[test]
fn kickstart_2_04_boots_the_workbench_2_04_disk_to_its_desktop() {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ADF_DIR") else {
        println!(
            "amiga-a500: set RSEMU_AMIGA_ADF_DIR to an Amiga Forever `Shared/adf` directory \
             to boot a real Workbench disk."
        );
        return;
    };
    let path = std::path::Path::new(&dir).join("amiga-os-204-workbench.adf");
    let Ok(disk) = std::fs::read(&path) else {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return;
    };
    reaches_its_screen("amiga-os-204.rom", disk, 62, GOLDEN_204_WORKBENCH);
}

/// Kickstart 1.3 with the Workbench 1.3 disk in DF0, both read in place: the
/// AmigaDOS shell the startup-sequence opens, `LoadWB`, and the Workbench 1.3
/// desktop — a plain blue screen with a white title bar reading "Workbench
/// release." and the free-memory count, the RAM DISK and Workbench1.3 icons
/// down the right-hand edge, and the red pointer at the top left.
///
/// Ninety virtual seconds. 1.3 is slower to the desktop than 2.04 is: the
/// shell banner is up by 40 s, `LoadWB` has opened `[CLI 2]` by 60, the icons
/// are drawn by 90, and the picture no longer moves after that (the
/// free-memory figure is still settling at 84 s and is identical at 90 and
/// 100).
///
/// Before the drive stepped its head on the *trailing* edge of `STEP*` this
/// run never got past the insert-disk screen: 1.3 asserts `SEL0*` and `STEP*`
/// in one `PRB` write, every step was dropped, the change flop never reset and
/// `trackdisk` never started the motor. `src/dev/amiga/floppy.rs` has the long
/// version.
#[test]
fn kickstart_1_3_boots_the_workbench_1_3_disk_to_its_desktop() {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ADF_DIR") else {
        println!(
            "amiga-a500: set RSEMU_AMIGA_ADF_DIR to an Amiga Forever `Shared/adf` directory \
             to boot a real Workbench disk."
        );
        return;
    };
    let path = std::path::Path::new(&dir).join("amiga-os-134-workbench.adf");
    let Ok(disk) = std::fs::read(&path) else {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return;
    };
    reaches_its_screen("amiga-os-130.rom", disk, 90, GOLDEN_130_WORKBENCH);
}

/// AROS's 512 KiB main ROM alone. It cannot reach a screen on this board: its
/// graphics library is in the extended ROM an A500 has no socket for, and it
/// raises an alert saying so on the serial port. What is asserted is only that
/// it runs to that point without a fault.
#[test]
fn aros_runs_until_it_needs_its_extended_rom() {
    let Some((b, _)) = run_to("aros-20250422.rom", Vec::new(), 4) else {
        return;
    };
    assert!(!b.cpu.is_halted());
    assert_eq!(b.cpu.bus_faults().0, 0);
}

/// At 12 s: the hand and disk on white.
const GOLDEN_130: u64 = 0x0d15_a156_1521_12c1;
/// At 28 s: the check mark, the drive, and the disk below it mid-animation.
const GOLDEN_204: u64 = 0x9a92_f494_18cf_2811;
/// At 12 s: the same picture with the 3.1 text.
const GOLDEN_310: u64 = 0xfdec_fe27_9cd5_3349;
/// At 62 s with the Workbench 2.04 disk: the desktop.
const GOLDEN_204_WORKBENCH: u64 = 0x95a5_9a12_c942_e139;
/// At 90 s with the Workbench 1.3 disk: the desktop.
const GOLDEN_130_WORKBENCH: u64 = 0xb491_89ae_fb75_bd01;
