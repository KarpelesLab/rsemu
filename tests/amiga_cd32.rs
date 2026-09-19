//! The user's two-part CD32 Kickstart on `machines/amiga-cd32.machine`: the
//! animated boot screen a CD32 with an empty tray shows, drawn by Lisa out of
//! the lines Alice fetched.
//!
//! # Why this file exists
//!
//! `tests/amiga_a1200.rs` proves the AA chip set with Kickstart 3.1 on an
//! A1200. This file proves the *machine around it*: the two-part ROM in its
//! two sockets, Akiko answering at `$B8_0000`, its EEPROM on the two wires the
//! ROM bit-bangs, and `cd.device` finding an empty drive and settling into its
//! poll — after which the ROM runs the animation it was written to run, which
//! is the thing a CD32 does that nothing else does.
//!
//! **The evidence that the guest sees AA** is `GfxBase->ChipRevBits0` read out
//! of guest memory, exactly as `tests/amiga_a1200.rs` reads it: exec's library
//! list walked from `ExecBase` by name, and `gb_ChipRevBits0` at offset `$EC`
//! of the library base. On this board it reads **`$1F`** — `GFXF_HR_AGNUS`,
//! `GFXF_HR_DENISE`, `GFXF_AA_ALICE`, `GFXF_AA_LISA` and bit 4 — and it gets
//! there at 4 s, before the logo is drawn.
//!
//! **The evidence that this is the CD32's ROM and not another** is the ROM
//! identifying itself: both halves' header words are read out of the map at
//! `$F8_000C` and `$E0_000C`, and `exec.library`'s own version from
//! `ExecBase`.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM.** Both halves are read from the user's own
//! `RSEMU_AMIGA_ROM_DIR` (Amiga Forever's `Shared/rom`) and the tests skip,
//! saying why, when it is not set. What is asserted is about this emulator:
//! that the processor is running, that no access faulted, that Lisa is
//! producing fields, what `graphics.library` concluded, and a hash of the
//! picture at a fixed virtual time. `RSEMU_AMIGA_FRAME_DIR`, when set,
//! receives a PNG of each checked frame (in a build with `display-png`); each
//! golden below was looked at before it was accepted, and is described beside
//! its test.
//!
//! `RSEMU_AMIGA_TRACE=1` prints the processor's state and `ChipRevBits0` once
//! a virtual second, which is how the timings above were established.
//!
//! The board is the shipped `machines/amiga-cd32.machine`, unchanged.
//!
//! No Amiga emulator source, no FPGA reimplementation, no AROS source and no
//! Kickstart disassembly was consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-cd32", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// Kickstart 3.1 for the CD32, and the half beside it that holds `cd.device`,
/// `lowlevel.library` and the boot animation.
const ROM: &str = "amiga-os-310-cd32.rom";
const EXT: &str = "amiga-os-310-cd32-ext.rom";

struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
}

/// The user's file `name` in the directory `var` names, or `None` having said
/// why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-cd32: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-cd32: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped CD32 around the user's two ROM halves, with `disc` in the tray.
fn board(disc: Vec<u8>) -> Option<Board> {
    let rom_path = user_file("RSEMU_AMIGA_ROM_DIR", ROM, "Shared/rom")?;
    let ext_path = user_file("RSEMU_AMIGA_ROM_DIR", EXT, "Shared/rom")?;
    let rom = rsemu::host::media::kickstart::open(&rom_path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", rom_path.display()));
    let ext = rsemu::host::media::kickstart::open(&ext_path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", ext_path.display()));

    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    capture::install(&mut options).expect("a capture table");
    options.realize.media.insert("kickstart", rom.bytes);
    options.realize.media.insert("ext", ext.bytes);
    options.realize.media.insert("cd0", disc);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-cd32")
        .expect("this build ships the board")
        .source;
    let machine = rsemu::machine::build("amiga-cd32", source, &registry, &options)
        .unwrap_or_else(|e| panic!("amiga-cd32: the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a video chip");
    Some(Board {
        machine,
        cpu,
        scanout,
    })
}

/// FNV-1a over the captured pixels, as every Amiga suite here hashes: our
/// rendering, nothing else.
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
    println!(
        "{label} at {seconds}s: {}x{} frame {hash:#018x}",
        info.width, info.height
    );
    hash
}

fn peek(b: &Board, addr: u32, width: Width) -> u32 {
    b.machine
        .space("mem")
        .expect("the memory space")
        .read(u64::from(addr), width, MemAttrs::DEBUG)
        .expect("the map answers") as u32
}

/// `graphics.library`'s base, found the way `OpenLibrary` finds it: exec's
/// `LibList` from `ExecBase`, node by node, by `ln_Name`.
///
/// ROM Kernel Reference Manual layouts (`exec/execbase.h`, `exec/nodes.h`,
/// `exec/lists.h`): `LibList` is the `struct List` at offset `$17A` of
/// `ExecBase`; a list's `lh_Head` is its first longword; a node's `ln_Succ` is
/// its first longword, zero on the list's tail, and `ln_Name` is at offset 10.
fn graphics_base(b: &Board) -> Option<u32> {
    let exec = peek(b, 4, Width::U32);
    if exec == 0 {
        return None;
    }
    let mut node = peek(b, exec + 0x17a, Width::U32);
    for _ in 0..64 {
        let next = peek(b, node, Width::U32);
        if next == 0 {
            return None;
        }
        let name = peek(b, node + 10, Width::U32);
        let bytes: Vec<u8> = (0..17)
            .map(|i| peek(b, name + i, Width::U8) as u8)
            .collect();
        if bytes == b"graphics.library\0" {
            return Some(node);
        }
        node = next;
    }
    None
}

/// `gb_ChipRevBits0`, at offset `$EC` of `GfxBase` (`graphics/gfxbase.h`).
fn chip_rev_bits(b: &Board) -> u8 {
    let gfx = graphics_base(b).expect("graphics.library is on exec's library list");
    peek(b, gfx + 0xec, Width::U8) as u8
}

/// `graphics/gfxbase.h`'s flags for `gb_ChipRevBits0`, in bit order:
/// `GFXB_HR_AGNUS` 0, `GFXB_HR_DENISE` 1, `GFXB_AA_ALICE` 2, `GFXB_AA_LISA` 3.
const HR_AGNUS: u8 = 1 << 0;
const HR_DENISE: u8 = 1 << 1;
const AA_ALICE: u8 = 1 << 2;
const AA_LISA: u8 = 1 << 3;
/// All four: what an AA board's `graphics.library` should conclude.
const AA: u8 = HR_AGNUS | HR_DENISE | AA_ALICE | AA_LISA;

/// A ROM's version and revision, from the header words at `+$0C` and `+$0E`
/// (`src/host/media/kickstart.rs` has the layout).
fn rom_version(b: &Board, base: u32) -> (u16, u16) {
    (
        peek(b, base + 12, Width::U16) as u16,
        peek(b, base + 14, Width::U16) as u16,
    )
}

/// Run for `seconds` of virtual time, then check what every run must:
/// running, no fault, fields, and the picture against `golden`.
fn boots_to(disc: Vec<u8>, label: &str, seconds: u64, golden: u64) -> Option<Board> {
    let mut b = board(disc)?;
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in 1..=seconds {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            let bits = graphics_base(&b).map(|g| peek(&b, g + 0xec, Width::U8) as u8);
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} fields={} chiprev={:?}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
                bits,
            );
        }
    }
    let hash = picture(&b, label, seconds);
    assert!(!b.cpu.is_halted(), "{label}: the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "{label}: an access faulted");
    assert!(
        b.scanout.frame_counter() >= seconds * 49,
        "{label}: Lisa is producing fields"
    );
    assert_eq!(
        hash, golden,
        "{label}: the frame at {seconds}s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before \
         accepting the new hash"
    );
    Some(b)
}

/// The picture at 12 s with an empty tray.
const GOLDEN_BOOT_12S: u64 = 0x4e20_7e1e_4ab1_d5d9;
/// The same at 6 s.
const GOLDEN_BOOT_6S: u64 = 0x8f94_08ce_732f_5637;

/// The animated boot screen, **and this is the milestone the board exists
/// for**: the picture a real CD32 shows with nothing in the tray.
///
/// The screen is black for the first three virtual seconds while Kickstart
/// sizes memory and builds exec's lists, and the extended ROM's `cd.device`
/// finds Akiko, proves its chunky-to-planar converter, talks to the EEPROM
/// and finds the drive empty. At 4 s the animation is running.
///
/// **12 s**: a 1600 x 568 field — 320 of Lisa's low-resolution pixels across a
/// PAL overscan window, so the capture is five 35 ns columns per pixel —
/// holding a starfield on black, with a band of deep purple sky stretched
/// across the top third and "AMIGA CD" over it in dark red and silver serif
/// capitals, a rainbow highlight running through the "CD", and "32" raised to
/// its right in the same red with a small "TM" beside it. Below the middle,
/// drawn in perspective and seen almost edge-on, a compact disc: a grey
/// ellipse with a black hub, a white ring around the hub and four rainbow
/// diffraction streaks crossing it. The disc turns and the sky's colours cycle
/// from frame to frame — both are copper work — so the hash is of one exact
/// virtual instant and moves if anything about the timing does.
#[test]
fn the_boot_screen_with_an_empty_tray() {
    let Some(b) = boots_to(Vec::new(), "cd32-boot", 12, GOLDEN_BOOT_12S) else {
        return;
    };

    // The AA chip set, concluded by the guest's own `graphics.library`.
    let bits = chip_rev_bits(&b);
    println!("cd32-boot: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & AA,
        AA,
        "graphics.library did not find Alice and Lisa"
    );

    // And the ROM saying what it is, out of both sockets.
    let (kv, kr) = rom_version(&b, 0xF8_0000);
    let (ev, er) = rom_version(&b, 0xE0_0000);
    let exec = peek(&b, 4, Width::U32);
    let exec_version = peek(&b, exec + 0x14, Width::U16) as u16;
    println!("cd32-boot: kickstart {kv}.{kr}, extended {ev}.{er}, exec.library {exec_version}");
    assert_eq!(kv, 40, "the Kickstart half is a 3.1 ROM");
    assert_eq!(ev, 40, "the extended half is a 3.1 ROM");
    assert_eq!(
        exec_version, kv,
        "exec.library is the one in the socketed ROM"
    );
}

/// Six seconds in: the disc alone, before the sky and the lettering are drawn.
///
/// **6 s**: black, a field of small white stars, and the same grey compact
/// disc seen almost edge-on across the middle with its black hub, white ring
/// and four rainbow streaks. No sky band and no lettering yet — the animation
/// brings them in over the next few seconds, and this golden catches it part
/// way.
#[test]
fn the_animation_is_part_way_at_six_seconds() {
    let _ = boots_to(Vec::new(), "cd32-boot", 6, GOLDEN_BOOT_6S);
}

/// A disc in the tray realizes, is carried, and changes nothing the guest can
/// see — which is the honest state of this board.
///
/// `src/dev/amiga/akiko.rs` says why: the message format `cd.device` and the
/// controller pass commands through is undocumented and was not recoverable
/// from a boot with an empty tray, because with an empty tray the ROM never
/// sends one. So no command reaches the mechanism and no sector reaches the
/// guest. The drive itself is complete — `src/dev/amiga/cdrom.rs` and its
/// tests — and what is missing is the road between them.
///
/// This test records exactly that: the disc is in the drive, and the boot is
/// the same frame, hash for hash, as with an empty tray. When the road is
/// built this is where the ROM will be asked to walk it, and the second
/// assertion will stop being true.
#[test]
fn a_disc_in_the_tray_is_carried_but_not_yet_read() {
    // Sixteen ISO 9660 sectors of user data, which is the layout a CD32
    // master is cut from; what is in them does not matter, because nothing
    // reads them yet.
    let disc = vec![0u8; 16 * 2048];
    let _ = boots_to(disc, "cd32-disc", 6, GOLDEN_BOOT_6S);
}
