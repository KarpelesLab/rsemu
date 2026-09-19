//! Real Kickstarts on the A600, booting Workbench from an IDE hard disk with
//! nothing in the floppy drive.
//!
//! # Why this file exists
//!
//! `tests/amiga_a600_board.rs` proves Gayle's IDE port with a program written
//! for the purpose. This file proves it with the one written for the machine:
//! Kickstart's own `scsi.device` finding Gayle by its identification register,
//! probing the drive, taking its interrupts through Gayle's latch on `INT2`,
//! reading the Rigid Disk Block and mounting the partition in it — and
//! AmigaDOS then booting Workbench off that partition. Nothing in any of that
//! was written here.
//!
//! What it took, each fixed where it was found (`docs/platforms/amiga.md`,
//! "A600", has the long version):
//!
//! | Stopped at | Why | Fixed in |
//! | --- | --- | --- |
//! | a loop in the ROM's first second, nothing on screen | the overlay was held until the first write to CIA-B, as the draft Gayle specification says; Kickstart's first CIA write is to CIA-A, and it needs chip RAM at zero well before it writes CIA-B | `src/dev/amiga/gayle.rs` (either CIA's write drops it) |
//! | the insert-disk screen, `$DA0000` never touched | the identification register has to read `$D` a bit at a time before Kickstart uses the IDE port at all | `src/dev/amiga/gayle.rs` (`ID`) |
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** Every test reads the user's own files in
//! place — the ROM from `RSEMU_AMIGA_ROM_DIR`, the hard-disk image from
//! `RSEMU_AMIGA_HDF_DIR` (Amiga Forever's `Shared/rom` and `Shared/hdf`) — and
//! skips, saying why, when either is not there. What is asserted is about this
//! emulator: that the processor is running, that no access faulted, that
//! Denise is producing fields, and a hash of the picture it produced at a fixed
//! virtual time. `RSEMU_AMIGA_FRAME_DIR`, when set, receives a PNG of each
//! checked frame (in a build with `display-png`); each golden below was looked
//! at before it was accepted, and is described beside its constant.
//!
//! `RSEMU_AMIGA_TRACE=1` prints the processor's state once a virtual second.
//!
//! The board is the shipped `machines/amiga-a600.machine`, unchanged: OCS
//! chips until the ECS ones land (at which point every golden here moves, and
//! should be looked at again), 1 MiB of chip RAM, DF0 empty. The image is
//! bound with `--media`'s semantics — copied into the drive — so nothing a
//! guest writes reaches the user's file.
//!
//! No Amiga emulator source, no AROS source and no Kickstart disassembly was
//! consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a600", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
}

/// The user's file `name` in the directory `var` names, or `None` having said
/// why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-a600: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a600: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A600 around the user's Kickstart `rom`, with `hdf` in the IDE
/// bay (`None` is an empty one) and nothing in DF0.
fn board(rom: &str, hdf: Option<&str>) -> Option<Board> {
    let rom_path = user_file("RSEMU_AMIGA_ROM_DIR", rom, "Shared/rom")?;
    let disk = match hdf {
        Some(name) => {
            let path = user_file("RSEMU_AMIGA_HDF_DIR", name, "Shared/hdf")?;
            std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        }
        None => Vec::new(),
    };
    let image = rsemu::host::media::kickstart::open(&rom_path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", rom_path.display()));

    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    capture::install(&mut options).expect("a capture table");
    options.realize.media.insert("kickstart", image.bytes);
    options.realize.media.insert("hd0", disk);
    // No floppy: the boot is the hard disk's or nothing.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a600")
        .expect("this build ships amiga-a600")
        .source;
    let machine = rsemu::machine::build("amiga-a600", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{rom}: the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a Denise");
    Some(Board {
        machine,
        cpu,
        scanout,
    })
}

/// FNV-1a over the captured pixels, as the A500 suites hash: our rendering,
/// nothing else.
fn frame_hash(surface: &Surface) -> u64 {
    surface
        .pixels()
        .iter()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        })
}

/// Hash the picture on screen now, and write it out as a PNG when
/// `RSEMU_AMIGA_FRAME_DIR` says where.
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

/// Boot `rom` with `hdf` for `seconds` of virtual time, then check it the way
/// every Amiga suite does and compare the picture with `golden`.
fn boots_to(rom: &str, hdf: Option<&str>, label: &str, seconds: u64, golden: u64) {
    let Some(mut b) = board(rom, hdf) else {
        return;
    };
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in 1..=seconds {
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
    let hash = picture(&b, label, seconds);
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

/// Kickstart 3.1 (40.063, the A500/A600/A2000 part) with Workbench 3.1's HDF
/// and DF0 empty: the Workbench desktop, off the hard disk.
///
/// The screen stays black for eleven seconds while the ROM finds Gayle, probes
/// the drive and AmigaDOS runs the startup-sequence; at 12 s the desktop is up
/// and does not move again. **15 s**: the grey 640-pixel Workbench screen, its
/// title bar reading "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights
/// Reserved.", the blue-framed "Workbench" window holding the "Ram Disk" icon
/// and the hard-disk icon labelled "Workbench3.1" — the partition's volume
/// name — and the red pointer at the top left.
#[test]
fn kickstart_3_1_boots_workbench_3_1_from_the_hard_disk() {
    boots_to(
        "amiga-os-310-a600.rom",
        Some("workbench-311.hdf"),
        "a600-310-wb311",
        15,
        GOLDEN_310_WB311,
    );
}

/// Kickstart 2.05 (37.350, the A600's own ROM) with Workbench 2.1's HDF: the
/// same desktop, 2.1's.
///
/// White while the ROM boots; at 9 s the AmigaDOS shell window, reading
/// "Amiga Release 2.1.1. Kickstart 37.350, Workbench 38.36"; at 13 s the
/// desktop. **15 s**: the grey screen with "Copyright © 1985-1992
/// Commodore-Amiga, Inc. All Rights Reserved" in its title bar, the
/// "Workbench" window with "Ram Disk" and the "Workbench2.1" hard-disk icon,
/// and the pointer.
#[test]
fn kickstart_2_05_boots_workbench_2_1_from_the_hard_disk() {
    boots_to(
        "amiga-os-205-a600.rom",
        Some("workbench-211.hdf"),
        "a600-205-wb211",
        15,
        GOLDEN_205_WB211,
    );
}

/// Kickstart 3.1 with the Workbench 1.3.5 HDF, which is 1.3's system on an
/// RDB disk: the 1.3-style desktop. **15 s**: a blue screen, the title bar
/// "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights Reserved.", and the
/// "Ram Disk" and "Workbench1.3" icons down the left, no window.
#[test]
fn kickstart_3_1_boots_the_workbench_1_3_hard_disk_too() {
    boots_to(
        "amiga-os-310-a600.rom",
        Some("workbench-135.hdf"),
        "a600-310-wb135",
        15,
        GOLDEN_310_WB135,
    );
}

/// With the bay empty, the same ROM finds Gayle, probes the port, finds no
/// drive, and asks for a disk.
///
/// It takes a while to give up: the screen is black until 19 s, while
/// `scsi.device` keeps selecting the drive and reading a status nothing
/// drives — the floating bus, on a port with no buffer between the cable and
/// the 68000 — until it times out. The insert-disk animation starts at 20 s.
/// **27 s**: the check mark, the drive and the disk, the frame bit for bit the
/// A500's at 12 s (`GOLDEN_310` in `tests/amiga_a500_kickstart.rs`), because
/// from there on the two boards run the same code.
#[test]
fn kickstart_3_1_with_an_empty_bay_asks_for_a_disk() {
    boots_to(
        "amiga-os-310-a600.rom",
        None,
        "a600-310-nodisk",
        27,
        GOLDEN_310_EMPTY,
    );
}

/// At 15 s, Kickstart 3.1 and Workbench 3.1: the desktop.
const GOLDEN_310_WB311: u64 = 0xa06f_52db_6660_b2a5;
/// At 15 s, Kickstart 2.05 and Workbench 2.1: the desktop.
const GOLDEN_205_WB211: u64 = 0x2462_09c0_df29_6a49;
/// At 15 s, Kickstart 3.1 and Workbench 1.3.5: the desktop.
const GOLDEN_310_WB135: u64 = 0x4450_4249_16a7_ca59;
/// At 27 s, Kickstart 3.1 with no disk anywhere: the insert-disk screen.
const GOLDEN_310_EMPTY: u64 = 0x26cc_b705_6fbc_7ba9;
