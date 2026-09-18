//! Real Kickstarts on the A500, run in place, to their insert-disk screens —
//! two of them on to the Workbench desktop — and AROS from its own boot disk
//! to its `Workbook` desktop.
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
//! | AROS | no screen: its graphics library is in a second ROM half the board had no place for | `src/dev/amiga/gary.rs` (the `ext` window) |
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
//! with its drive empty unless a test says otherwise, and its `ext` window
//! empty for every Kickstart. The A501 test adds `-p slow-ram=512K`; the AROS
//! test adds `-p chip-ram=1M`, the A501, and AROS's second ROM half in `ext`,
//! and types one key combination on the keyboard (its doc comment says why).
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
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::dev::amiga::keyboard::{DEFAULT_KEYBOARD_PORT, Keyboard, keys};
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
    /// The keyboard's host end, which a test types on as a person would.
    keyboard: Arc<Keyboard>,
}

/// What goes into the board: the ROM files, the disk, and any parameters.
struct Setup<'a> {
    /// The main ROM, in `RSEMU_AMIGA_ROM_DIR`.
    rom: &'a str,
    /// A second ROM for the `ext` window at `$E0_0000`, also in
    /// `RSEMU_AMIGA_ROM_DIR`. `None` is the empty window every Kickstart runs
    /// with.
    ext: Option<&'a str>,
    /// The bytes in DF0; none is an empty drive.
    df0: Vec<u8>,
    /// Board parameters beyond `kickstart-size`, which follows the ROM.
    params: &'a [(&'a str, &'a str)],
    /// What the frame's PNG is called, less the time and the extension.
    label: &'a str,
}

impl<'a> Setup<'a> {
    /// `rom` alone on the stock board, with the drive empty.
    fn rom(rom: &'a str) -> Setup<'a> {
        Setup {
            rom,
            ext: None,
            df0: Vec::new(),
            params: &[],
            label: rom.trim_end_matches(".rom"),
        }
    }
}

/// Read a ROM out of `RSEMU_AMIGA_ROM_DIR`, decoding a keyed one; `None`
/// (having said why) if the variable or the file is not there.
fn rom_image(file: &str) -> Option<Vec<u8>> {
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
    Some(image.bytes)
}

/// Read a disk out of `RSEMU_AMIGA_ADF_DIR`; `None` (having said why) if the
/// variable or the file is not there.
fn adf(file: &str) -> Option<Vec<u8>> {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ADF_DIR") else {
        println!(
            "amiga-a500: set RSEMU_AMIGA_ADF_DIR to an Amiga Forever `Shared/adf` directory \
             to boot a real disk."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    let Ok(disk) = std::fs::read(&path) else {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return None;
    };
    Some(disk)
}

/// Build the board `setup` describes, or `None` if a ROM is not there.
fn board(setup: Setup<'_>) -> Option<Board> {
    let image = rom_image(setup.rom)?;
    let ext = match setup.ext {
        Some(file) => rom_image(file)?,
        None => Vec::new(),
    };
    let size = image.len();

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
    for &(name, value) in setup.params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    options.realize.media.insert("kickstart", image);
    // No bytes is an empty drive: the insert-disk screen is what a Kickstart
    // shows with nothing in DF0. And no bytes in `ext` is the board without
    // the window, which is every real A500.
    options.realize.media.insert("df0", setup.df0);
    options.realize.media.insert("ext", ext);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source;
    let machine = rsemu::machine::build("amiga-a500", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{}: the board does not realize: {e}", setup.rom));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a Denise");
    let keyboard = keys::get(&options.realize.hosts, DEFAULT_KEYBOARD_PORT)
        .expect("a keyboard")
        .expect("the board has a keyboard");
    Some(Board {
        machine,
        cpu,
        scanout,
        keyboard,
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

/// Run the board on from virtual second `from` to `to`, a second at a time.
///
/// `RSEMU_AMIGA_TRACE=1` prints the processor's state once a second on the
/// way, which is how a person finds where a ROM stopped.
fn advance(b: &mut Board, label: &str, from: u64, to: u64) {
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in from + 1..=to {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} fields={}",
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
/// `label` and `seconds` when `RSEMU_AMIGA_FRAME_DIR` says where.
fn picture(b: &Board, label: &str, seconds: u64) -> u64 {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let hash = frame_hash(&surface);
    if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
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
    println!("{label} at {seconds}s: frame {hash:#018x}");
    hash
}

/// Run `setup` to `seconds` of virtual time and hash the picture there.
fn run_to(setup: Setup<'_>, seconds: u64) -> Option<(Board, u64)> {
    let label = setup.label.to_string();
    let mut b = board(setup)?;
    advance(&mut b, &label, 0, seconds);
    let hash = picture(&b, &label, seconds);
    Some((b, hash))
}

/// The checks every ROM that reaches its screen must pass, then the golden.
fn reaches_its_screen(setup: Setup<'_>, seconds: u64, golden: u64) {
    let label = setup.label.to_string();
    let Some((b, hash)) = run_to(setup, seconds) else {
        return;
    };
    assert!(!b.cpu.is_halted(), "{label}: the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "{label}: an access faulted");
    assert!(
        b.scanout.frame_counter() >= seconds * 49,
        "{label}: Denise is producing fields"
    );
    assert_eq!(
        hash, golden,
        "{label}: the frame at {seconds}s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before \
         accepting the new hash"
    );
}

/// Kickstart 1.3 (34.5), 256 KiB: the hand holding a Workbench 1.3 disk.
#[test]
fn kickstart_1_3_draws_the_hand_and_disk() {
    reaches_its_screen(Setup::rom("amiga-os-130.rom"), 12, GOLDEN_130);
}

/// Kickstart 2.04 (37.175): the check mark, the drive and the disk going in.
#[test]
fn kickstart_2_04_draws_the_insert_disk_animation() {
    reaches_its_screen(Setup::rom("amiga-os-204.rom"), 28, GOLDEN_204);
}

/// Kickstart 3.1 (40.063, the A500/A600/A2000 part): the same picture.
#[test]
fn kickstart_3_1_draws_the_insert_disk_animation() {
    reaches_its_screen(Setup::rom("amiga-os-310-a600.rom"), 12, GOLDEN_310);
}

/// Kickstart 2.04 with the Workbench 2.04 disk in DF0, both read in place: the
/// AmigaDOS shell window, `LoadWB`, and the Workbench desktop — a blue-framed
/// "Workbench" window holding the Ram Disk and Workbench2.0 icons, on a grey
/// 640-pixel high-resolution screen, with the red pointer at the top left.
///
/// Forty-five virtual seconds, the least that proves it with a margin: the
/// picture reaches this desktop at 42 s and does not move again. It took 62
/// until the 68000 was given its whole clock — Paula's serial poll used to be
/// a runnable on the processor's crystal and took half of every round
/// (`docs/platforms/amiga.md`), and the disk was read until about 57 s.
/// Before the blitter's line mode stepped D by `BLTCMOD` this run stopped with
/// `AN_MemCorrupt`; before the high-resolution fetch counted in eight-count
/// blocks it reached the desktop sheared a word a line.
#[test]
fn kickstart_2_04_boots_the_workbench_2_04_disk_to_its_desktop() {
    let Some(disk) = adf("amiga-os-204-workbench.adf") else {
        return;
    };
    let setup = Setup {
        df0: disk,
        ..Setup::rom("amiga-os-204.rom")
    };
    reaches_its_screen(setup, 45, GOLDEN_204_WORKBENCH);
}

/// Kickstart 1.3 with the Workbench 1.3 disk in DF0, both read in place: the
/// AmigaDOS shell the startup-sequence opens, `LoadWB`, and the Workbench 1.3
/// desktop — a plain blue screen with a white title bar reading "Workbench
/// release." and the free-memory count, the RAM DISK and Workbench1.3 icons
/// down the right-hand edge, and the red pointer at the top left.
///
/// Seventy-two virtual seconds. 1.3 is slower to the desktop than 2.04 is:
/// the picture changes for the last time at 68 s and is this desktop from then
/// on. With the 68000 at half its clock it took ninety — the shell banner was
/// up by 40 s, `[CLI 2]` by 60 and the free-memory figure still settling at
/// 84.
///
/// Before the drive stepped its head on the *trailing* edge of `STEP*` this
/// run never got past the insert-disk screen: 1.3 asserts `SEL0*` and `STEP*`
/// in one `PRB` write, every step was dropped, the change flop never reset and
/// `trackdisk` never started the motor. `src/dev/amiga/floppy.rs` has the long
/// version.
#[test]
fn kickstart_1_3_boots_the_workbench_1_3_disk_to_its_desktop() {
    let Some(disk) = adf("amiga-os-134-workbench.adf") else {
        return;
    };
    let setup = Setup {
        df0: disk,
        ..Setup::rom("amiga-os-130.rom")
    };
    reaches_its_screen(setup, 72, GOLDEN_130_WORKBENCH);
}

/// The same boot with an A501 in the trapdoor, `-p slow-ram=512K`: the same
/// blue desktop with the same two icons, and the title bar reads "Workbench
/// release." and **"889256 free memory"** where the stock board's reads
/// "365000" — 524 256 more, the card's 512 KiB less 32 bytes. So Kickstart
/// found the RAM at `$C0_0000` and put it on its free list. Before bank 6 was
/// decoded, the same probe found only a floating bus there; with no card it
/// now finds the chip registers repeating, and still concludes there is no RAM
/// (the stock desktop above did not move by a bit).
#[test]
fn kickstart_1_3_counts_the_a501_in_its_free_memory() {
    let Some(disk) = adf("amiga-os-134-workbench.adf") else {
        return;
    };
    let setup = Setup {
        df0: disk,
        params: &[("slow-ram", "512K")],
        label: "amiga-os-130-a501",
        ..Setup::rom("amiga-os-130.rom")
    };
    reaches_its_screen(setup, 72, GOLDEN_130_A501_WORKBENCH);
}

/// AROS from its own boot disk, both ROM halves in place: to the `Workbook`
/// desktop.
///
/// The board is the shipped one with `-p chip-ram=1M -p slow-ram=512K` and
/// AROS's second ROM half in the `ext` window at `$E0_0000` — which no real
/// A500 has, and which is the only way this two-part ROM runs at all.
/// `aros-20250422-boot.adf` is in DF0.
///
/// **55 s**: AROS's grey screen and its blue-framed "AROS" shell window, the
/// copyright, licence, version and build-date lines, and over them a "System
/// requester" reading `Please insert volume "AROS Live CD" in any drive` with
/// `Retry` and `Cancel`. The disk's own startup-sequence asks for that volume
/// — the CD Amiga Forever pairs this disk with — and this board has no drive
/// for it. That is the software asking, not the hardware stopping.
///
/// So the test answers the way a person at the keyboard would: Left-Amiga+B,
/// a requester's negative gadget, typed through the keyboard's own protocol.
///
/// **80 s**: the startup-sequence carried on to `LoadWB` and `EndCLI`. The
/// shell window is gone; the screen title bar reads "Workbook 1.0  Chip: 634k,
/// Fast: 0k, Any: 634k", the red pointer at its left, the depth gadget at its
/// right; and below it the "AROS Kickstart" disk icon and the "RAM Disk" icon.
/// The picture does not move from here on.
///
/// With 1 MiB in all — either `chip-ram=1M` alone, or 512 KiB of chip and an
/// A501 — the run reaches "Workbook 1.0" in the title bar and stops short of
/// the icons, the shell window still open. Every task waits on the same
/// signals as in this run; the difference is that chip RAM has 127 KiB free
/// in pieces of at most 26 KiB (read through the RKRM's `MemHeader` chain),
/// which is the desktop's allocation failing, not a chip. And with 512 KiB in
/// all AROS runs out of memory before it draws anything.
#[test]
fn aros_boots_its_own_disk_to_the_workbook_desktop() {
    let Some(disk) = adf("aros-20250422-boot.adf") else {
        return;
    };
    let setup = Setup {
        ext: Some("aros-20250422-ext.rom"),
        df0: disk,
        params: &[("chip-ram", "1M"), ("slow-ram", "512K")],
        label: "aros-boot",
        ..Setup::rom("aros-20250422.rom")
    };
    let label = setup.label;
    let Some((mut b, hash)) = run_to(setup, 55) else {
        return;
    };
    assert_eq!(
        hash, GOLDEN_AROS_REQUESTER,
        "the frame at 55s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before accepting the new hash"
    );

    // Left-Amiga ($66) and B ($35), Appendix G's matrix table, held long
    // enough for each code to cross the handshake.
    let tenth = GlobalTime::from_nanos(100_000_000);
    for (code, down) in [(0x66, true), (0x35, true), (0x35, false), (0x66, false)] {
        b.keyboard.press(code, down);
        b.machine.run_for(tenth).expect("it runs");
    }
    b.machine
        .run_for(GlobalTime::from_nanos(600_000_000))
        .expect("it runs");
    advance(&mut b, label, 56, 80);
    let hash = picture(&b, label, 80);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    // The card's RAM is in use, and for what the TRM says it is for: "when
    // ExecBase is transferred to $C00000". Location 4 is "the only absolute
    // memory location in the system" (HRM Appendix D).
    let exec = b
        .machine
        .space("mem")
        .expect("the memory space")
        .read(4, Width::U32, MemAttrs::DEBUG)
        .expect("chip RAM");
    assert!(
        (0xC0_0000..0xC8_0000).contains(&exec),
        "ExecBase at {exec:#x}, not in the A501's RAM"
    );
    assert_eq!(
        hash, GOLDEN_AROS_DESKTOP,
        "the frame at 80s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before accepting the new hash"
    );
}

/// At 12 s: the hand and disk on white.
const GOLDEN_130: u64 = 0x0d15_a156_1521_12c1;
/// At 28 s: the check mark, the drive, and the disk below it mid-animation.
const GOLDEN_204: u64 = 0xcfa0_61a4_23d3_703d;
/// At 12 s: the same picture with the 3.1 text.
const GOLDEN_310: u64 = 0x26cc_b705_6fbc_7ba9;
/// At 45 s with the Workbench 2.04 disk: the desktop.
const GOLDEN_204_WORKBENCH: u64 = 0x95a5_9a12_c942_e139;
/// At 72 s with the Workbench 1.3 disk: the desktop.
const GOLDEN_130_WORKBENCH: u64 = 0xb491_89ae_fb75_bd01;
/// At 72 s with the Workbench 1.3 disk and an A501: the desktop, with more
/// free memory in its title bar.
const GOLDEN_130_A501_WORKBENCH: u64 = 0xf4d8_f443_2659_d621;
/// At 55 s, AROS: the shell window and the "AROS Live CD" requester.
const GOLDEN_AROS_REQUESTER: u64 = 0x5b3d_e1c5_deeb_bd01;
/// At 80 s, AROS, the requester cancelled: the `Workbook` desktop.
const GOLDEN_AROS_DESKTOP: u64 = 0x8650_0fcf_a147_a5cd;
