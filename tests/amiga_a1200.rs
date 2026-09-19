//! The user's Kickstart 3.1 on the A1200: Workbench 3.1 off the hard disk and
//! off a floppy, drawn by Lisa out of the lines Alice fetched.
//!
//! # Why this file exists
//!
//! `src/dev/amiga/agnus/tests.rs` and `tests/amiga_alice.rs` prove Alice with
//! programs written for the purpose. This file proves her with the one written
//! for the machine: Kickstart 3.1's own `graphics.library` identifying the AA
//! chip set, opening a Workbench screen on it, and AmigaDOS booting Workbench
//! 3.1 off Gayle's IDE port or off DF0 — none of which was written here.
//!
//! **The evidence that the guest sees AA**, and not merely that it boots, is
//! `GfxBase->ChipRevBits0` read out of guest memory after the boot, the way
//! `tests/amiga_a500plus.rs` reads it for the Enhanced Chip Set: exec's
//! library list is walked from `ExecBase` by name, and `gb_ChipRevBits0` at
//! offset `$EC` of the library base is asked what `graphics.library` decided.
//! On this board it reads **`$1F`** once the ROM has a boot device:
//! `GFXF_HR_AGNUS`, `GFXF_HR_DENISE`, `GFXF_AA_ALICE`, `GFXF_AA_LISA` and bit
//! 4 all set. On the A600, whose Gayle and CIAs and Paula are the same objects
//! and whose chips are an 8375 and an 8373, the same ROM family reads `$03` —
//! which is the control, and it is a test here rather than a claim.
//!
//! Traced second by second, the first three bits are set inside the first
//! virtual second and the AA pair at 2 s, before AmigaDOS mounts anything; a
//! board with no boot device at all stays at `$13`, which the third test
//! records.
//!
//! **Which of the two chips each bit comes from was measured**, by running
//! this board's own source with one chip swapped for its Enhanced Chip Set
//! part, and then by sweeping each chip's identification: with an 8373 in
//! Lisa's place it reads `$07` — bits 3 and 4 gone — so bit 3
//! (`GFXF_AA_LISA`) and bit 4 are `LISAID`'s. **Bit 2 (`GFXF_AA_ALICE`) is
//! `VPOSR`'s**, and it is bit 1 of the Agnus identification the ROM tests:
//! with Alice answering `$20`, `$21`, `$30` or `$31` it reads `$1B`, and with
//! `$22`, `$23`, `$32` or `$33` it reads `$1F`. The 8375 swap moved nothing
//! only because this tree's 8375 then answered `$22` as well, and changing
//! Alice's `$22` to `$23` moved nothing because both have the bit; the 8375
//! answers `$21` now (`src/dev/amiga/agnus/ecs.rs`), and the A600 booting off
//! its hard disk reads `$03` where it read `$07`.
//!
//! **What makes it an AA machine to the ROM beyond those bits is `LISAID` bits
//! 9 and 8**, which `graphics.library` reads as the board's fetch bandwidth
//! and sizes its whole display database from: `MaxDepth` 8 in every
//! resolution at four times, the Enhanced Chip Set's 5/4/2 at one. The
//! `screenmode` session below is the evidence, and `src/dev/amiga/denise.rs`'s
//! `LISA_ID` the measurement.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** Every test reads the user's own files in
//! place — the ROM from `RSEMU_AMIGA_ROM_DIR`, the hard-disk image from
//! `RSEMU_AMIGA_HDF_DIR`, the floppy from `RSEMU_AMIGA_ADF_DIR` (Amiga
//! Forever's `Shared/rom`, `Shared/hdf` and `Shared/adf`) — and skips, saying
//! why, when one is not there. What is asserted is about this emulator: that
//! the processor is running, that no access faulted, that Lisa is producing
//! fields, what `graphics.library` concluded, and a hash of the picture at a
//! fixed virtual time. `RSEMU_AMIGA_FRAME_DIR`, when set, receives a PNG of
//! each checked frame (in a build with `display-png`); each golden below was
//! looked at before it was accepted, and is described beside its test.
//!
//! `RSEMU_AMIGA_TRACE=1` prints the processor's state and `ChipRevBits0` once
//! a virtual second, which is how the timings above were established.
//!
//! The board is the shipped `machines/amiga-a1200.machine`, unchanged.
//! Images are bound with `--media`'s semantics — copied into the drive — so
//! nothing a guest writes reaches the user's file.
//!
//! No Amiga emulator source, no FPGA reimplementation, no AROS source and no
//! Kickstart disassembly was consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a1200", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// The A1200's Kickstart: 3.1, 40.68, the AA part.
const ROM: &str = "amiga-os-310-a1200.rom";

struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
}

/// What goes in a drive: nothing, a hard-disk image, or a floppy.
#[derive(Clone, Copy)]
enum Disk {
    None,
    Hard(&'static str),
    Floppy(&'static str),
}

/// The user's file `name` in the directory `var` names, or `None` having said
/// why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-a1200: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a1200: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A1200 around the user's Kickstart, with `disk` in whichever
/// drive it belongs in and nothing in the other.
fn board(disk: Disk) -> Option<Board> {
    board_named("amiga-a1200", ROM, disk)
}

/// The same for any of this tree's Gayle boards, so the A600 can be asked the
/// same question about the same ROM.
fn board_named(name: &str, rom: &str, disk: Disk) -> Option<Board> {
    let rom_path = user_file("RSEMU_AMIGA_ROM_DIR", rom, "Shared/rom")?;
    let (hd0, df0) = match disk {
        Disk::None => (Vec::new(), Vec::new()),
        Disk::Hard(name) => {
            let path = user_file("RSEMU_AMIGA_HDF_DIR", name, "Shared/hdf")?;
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            (bytes, Vec::new())
        }
        Disk::Floppy(name) => {
            let path = user_file("RSEMU_AMIGA_ADF_DIR", name, "Shared/adf")?;
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            (Vec::new(), bytes)
        }
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
    options.realize.media.insert("hd0", hd0);
    options.realize.media.insert("df0", df0);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine(name)
        .expect("this build ships the board")
        .source;
    let machine = rsemu::machine::build(name, source, &registry, &options)
        .unwrap_or_else(|e| panic!("{name}: the board does not realize: {e}"));
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
        .expect("chip RAM") as u32
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

/// Run for `seconds` of virtual time, then check what every run must: running,
/// no fault, fields, and the picture against `golden`.
fn boots_to(disk: Disk, label: &str, seconds: u64, golden: u64) -> Option<Board> {
    let mut b = board(disk)?;
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

/// Kickstart 3.1 (40.68, the A1200's own ROM) with Workbench 3.1's HDF and
/// nothing in DF0: the Workbench desktop, off the hard disk, **and this is the
/// milestone the board exists for**.
///
/// The screen is black for eleven seconds while the ROM finds Gayle, probes
/// the drive and AmigaDOS runs the startup-sequence; at 12 s the desktop is up
/// and does not move again. **15 s**: the grey 640-pixel Workbench screen —
/// 1280 of Lisa's 35 ns columns, because a high-resolution pixel is two of
/// them — with "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights
/// Reserved." in its title bar, the blue-framed "Workbench" window holding the
/// "Ram Disk" icon and the hard-disk icon labelled "Workbench3.1", the window's
/// scroll bars and gadgets down its right edge and along its bottom, and the
/// red pointer at the top left over the copyright line.
///
/// **The same hash at one and at four times the bandwidth.** This screen is
/// fetched with `FMODE $0003` since `LISAID` reports the A1200's 64-bit fetch,
/// where it was `FMODE $0000` before; the ROM moves the bitplane pointer and
/// the modulo to suit, and Lisa's delay from fetch to first pixel is what it
/// counts on (`src/dev/amiga/denise/aga.rs`, `fetch_block`). Getting that delay
/// wrong put the whole desktop sixteen high-resolution pixels to the left;
/// getting it right is this frame, pixel for pixel.
#[test]
fn kickstart_3_1_boots_workbench_3_1_from_the_hard_disk() {
    let Some(b) = boots_to(
        Disk::Hard("workbench-311.hdf"),
        "a1200-310-wb311",
        15,
        GOLDEN_WB311_HD,
    ) else {
        return;
    };
    let bits = chip_rev_bits(&b);
    println!("a1200-310-wb311: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & AA,
        AA,
        "graphics.library did not find Alice and Lisa"
    );
}

/// The same ROM booting Workbench 3.1 from a floppy in DF0, with the IDE bay
/// empty.
///
/// Much slower than the hard disk, as it is on a real machine: the screen is
/// black while `trackdisk.device` reads the disk track by track and AmigaDOS
/// loads the system from it. **75 s**: the same grey Workbench screen and
/// "Workbench" window, this time with a *floppy* icon labelled "Workbench3.1"
/// under the Ram Disk — the disk in DF0, not a partition.
#[test]
fn kickstart_3_1_boots_workbench_3_1_from_a_floppy() {
    let Some(b) = boots_to(
        Disk::Floppy("amiga-os-310-workbench.adf"),
        "a1200-310-df0",
        75,
        GOLDEN_WB311_DF0,
    ) else {
        return;
    };
    let bits = chip_rev_bits(&b);
    println!("a1200-310-df0: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & AA,
        AA,
        "graphics.library did not find Alice and Lisa"
    );
}

/// With both drives empty, the ROM finds Gayle, probes the port, finds no
/// drive, and asks for a disk — **in AA**, which is what this test is for.
///
/// It takes a while to give up: the processor is *stopped* for thirty-one
/// seconds while `scsi.device` waits on a drive that is not there, and the
/// screen is black. **45 s**: Kickstart 3.1's own insert-disk screen — the
/// deep purple background, the Amiga check mark in a blue-green-yellow-red
/// gradient, "3.1 ROM 40.068 / Copyright © 1985-1993 / Commodore-Amiga, Inc. /
/// All Rights Reserved." under it in a pale peach, and the disk drive at the
/// right with the disk part-way into its slot. It is 1600 columns wide where
/// the A600's is 800, because Lisa's are 35 ns columns and an 8373's are
/// 70 ns ones; **it is otherwise the A600's screen**, gradient and all, so the
/// picture is not by itself evidence of the 256-entry table. The ROM version
/// is: `40.068` here against the A600's `40.063`, which is the AA Kickstart.
///
/// **`ChipRevBits0` is `$13` here, not `$1F`**, and that is the observation
/// worth keeping: `GFXF_HR_AGNUS`, `GFXF_HR_DENISE` and bit 4 are set within
/// the first second on every A1200 run, and the ROM sets the two AA bits
/// **only once it has a boot device** — at 2 s with the hard disk, and never
/// on this run, which sits stopped until 32 s and then draws this screen.
/// So the pair asserted here is the ECS pair, and the AA pair is asserted by
/// the two tests above, where the ROM has got that far. Nothing on the disk
/// does it: the flip is at 2 s, ten seconds before AmigaDOS mounts anything.
#[test]
fn with_no_disk_the_rom_asks_for_one_and_draws_the_aa_screen() {
    let Some(b) = boots_to(Disk::None, "a1200-310-nodisk", 45, GOLDEN_EMPTY) else {
        return;
    };
    let bits = chip_rev_bits(&b);
    println!("a1200-310-nodisk: ChipRevBits0 = {bits:#04x}");
    assert_eq!(bits & HR_AGNUS, HR_AGNUS, "Alice is an ECS Agnus too");
    assert_eq!(bits & HR_DENISE, HR_DENISE, "and Lisa an ECS Denise");
    assert_eq!(bits & (AA_ALICE | AA_LISA), 0, "not without a boot device");
}

/// The control: the A600 — the same Gayle, the same CIAs, the same Paula, and
/// an 8375 and an 8373 where this board has Alice and Lisa — booting the
/// A600's own Kickstart 3.1 with nothing in either drive.
///
/// `graphics.library` finds the ECS pair and **neither AA bit**, which is what
/// makes `$1F` on the A1200 a statement about the chips rather than about the
/// ROM. Nothing here looks at the picture: `tests/amiga_a600_hdf.rs` owns the
/// A600's goldens, and this asks one question.
#[cfg(feature = "machine-amiga-a600")]
#[test]
fn the_ecs_board_beside_it_finds_neither_alice_nor_lisa() {
    let Some(mut b) = board_named("amiga-a600", "amiga-os-310-a600.rom", Disk::None) else {
        return;
    };
    for _ in 0..12 {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
    }
    let bits = chip_rev_bits(&b);
    println!("a600-310-nodisk: ChipRevBits0 = {bits:#04x}");
    assert_eq!(bits & HR_AGNUS, HR_AGNUS, "an 8375 is an ECS Agnus");
    assert_eq!(bits & HR_DENISE, HR_DENISE, "an 8373 is an ECS Denise");
    assert_eq!(bits & AA_ALICE, 0, "but an 8375 is not Alice");
    assert_eq!(bits & AA_LISA, 0, "and an 8373 is not Lisa");
}

/// The same control on the path that sets the AA pair on the A1200: the A600
/// booting Workbench 3.1 off the same hard disk.
///
/// With a boot device the ROM runs its later chip test too, and that one reads
/// bit 1 of `VPOSR`'s identification as Alice. While this tree's 8375 answered
/// `$22` the A600 read **`$07`** here — `GFXF_AA_ALICE` on an ECS board. The
/// 8375 answers `$21` now, the specification's own row, and it reads `$03`.
#[cfg(feature = "machine-amiga-a600")]
#[test]
fn the_ecs_board_booting_workbench_still_finds_no_alice() {
    let Some(mut b) = board_named(
        "amiga-a600",
        "amiga-os-310-a600.rom",
        Disk::Hard("workbench-311.hdf"),
    ) else {
        return;
    };
    for _ in 0..15 {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
    }
    let bits = chip_rev_bits(&b);
    println!("a600-310-wb311: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & (HR_AGNUS | HR_DENISE),
        HR_AGNUS | HR_DENISE,
        "the ECS pair"
    );
    assert_eq!(bits & AA_ALICE, 0, "an 8375 is not Alice");
    assert_eq!(bits & AA_LISA, 0, "an 8373 is not Lisa");
}

/// At 15 s, Kickstart 3.1 and Workbench 3.1 off the hard disk: the desktop.
const GOLDEN_WB311_HD: u64 = 0xb360_d7a3_0bf0_bc7d;
/// At 75 s, the same off a floppy: the same desktop with a floppy icon.
const GOLDEN_WB311_DF0: u64 = 0xf0fe_2791_b81a_72ed;
/// At 45 s, with neither drive filled: the AA insert-disk screen.
const GOLDEN_EMPTY: u64 = 0x83bd_b5da_233c_4551;

/// ScreenMode Preferences, opened by a person at the Workbench: what the
/// A1200's `graphics.library` *offers*, and the other half of the evidence
/// that the board is AA.
///
/// **"Maximum Colors: 256"** here against the A600's 16, in the same window
/// over the same mode. It was 16 on both until `LISAID` answered `$00F8`: the
/// ROM had found Lisa by the low byte and built every AA `DisplayInfo` — the
/// 24-bit palette, eight bits a gun, high-resolution HAM and EHB — and then
/// sized every `DimensionInfo` from bits 9 and 8, which read as ones said one
/// times the bandwidth, the Enhanced Chip Set's 5/4/2. `docs/platforms/amiga.md`,
/// "What `LISAID` bits 9 and 8 are", has how that was found. The mode *list*
/// is the A600's, and should be: it holds the modes Workbench may open on
/// (`DIPF_IS_WB`), and none of the thirty-six modes the A1200's database has
/// beyond the A600's — HAM and EHB in high and super-high resolution among
/// them — carries that flag. More would come from monitor drivers, and this
/// install's `Devs/Monitors` holds only `PAL` and `NTSC`; DblPAL, Multiscan
/// and the rest sit unused in `Storage/Monitors`, where Workbench 3.1 puts
/// them.
///
/// The session is `tests/amiga_a500plus.rs`'s, moved to the hard disk: boot
/// Workbench 3.1 off the HDF, open the "Workbench3.1" volume, open its Prefs
/// drawer, open ScreenMode, and look. Every input crosses the record/replay
/// seam as a pointer event, so the run stays deterministic.
///
/// The pointer is relative (`host::input::amiga::PIXELS_PER_COUNT` is one
/// count per host pixel, and Intuition moves it one *screen* pixel a count),
/// so the coordinates here are in the 640 × 256 screen's own pixels offset by
/// where the screen starts — the same numbers the A500+ session uses. Lisa's
/// picture is 35 ns columns, four to a low-resolution pixel and two to a
/// high-resolution one, so a thing at column `X` of the captured frame is at
/// `X / 2` here.
mod screenmode {
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::core::clock::GlobalTime;
    use rsemu::core::record::{Channel, Recorder};
    use rsemu::cpu::m68k::M68k;
    use rsemu::host::display::amiga::{DeniseScanout, capture};
    use rsemu::host::display::{PixelFormat, Scanout, Surface};
    use rsemu::host::input::amiga::{AmigaKeyboardSink, AmigaMouseSink};
    use rsemu::host::input::{self, Feed, InputEvent};
    use rsemu::machine::{Machine, catalog};

    struct Desk {
        machine: Machine,
        cpu: Arc<M68k>,
        scanout: DeniseScanout,
        recorder: Arc<Recorder>,
        channel: Channel,
        tag: &'static str,
    }

    fn desk(board: &str, rom: &str, tag: &'static str) -> Option<Desk> {
        let rom = super::user_file("RSEMU_AMIGA_ROM_DIR", rom, "Shared/rom")?;
        let hdf = super::user_file("RSEMU_AMIGA_HDF_DIR", "workbench-311.hdf", "Shared/hdf")?;
        let image = rsemu::host::media::kickstart::open(&rom.to_string_lossy())
            .unwrap_or_else(|e| panic!("{}: {e}", rom.display()));
        let disk = std::fs::read(&hdf).expect("the disk reads");

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
        options.realize.media.insert("df0", Vec::new());
        let recorder = Arc::new(Recorder::recording());
        options.realize.recorder = Some(Arc::clone(&recorder));
        let registry = catalog::registry().expect("a registry");
        let source = catalog::machine(board)
            .expect("this build ships the board")
            .source;
        let machine = rsemu::machine::build(board, source, &registry, &options)
            .unwrap_or_else(|e| panic!("{board}: the board does not realize: {e}"));

        let hosts = &options.realize.hosts;
        let feed = Arc::new(Feed::new());
        feed.attach(Arc::new(
            AmigaKeyboardSink::open(hosts).expect("the board has a keyboard"),
        ));
        feed.attach(Arc::new(
            AmigaMouseSink::open(hosts).expect("the board has a mouse"),
        ));
        let channel = input::channel(input::DEFAULT_STREAM);
        recorder
            .register(channel.clone(), input::sink(&feed))
            .expect("the channel list is open until the first round");
        let cpu = cores.last().expect("the binding captured the processor");
        let scanout = capture::take(hosts, &machine).expect("a video chip");
        Some(Desk {
            machine,
            cpu,
            scanout,
            recorder,
            channel,
            tag,
        })
    }

    /// Three PAL fields, as the other Workbench sessions hold a button.
    const HOLD_MS: u64 = 60;
    /// The screen's top-left pixel, in the pointer's coordinates.
    const SCREEN_LEFT: u32 = 130;
    const SCREEN_TOP: u32 = 30;

    impl Desk {
        fn run_ms(&mut self, ms: u64) {
            self.machine
                .run_for(GlobalTime::from_nanos(ms * 1_000_000))
                .expect("it runs");
        }

        fn pointer(&self, x: u32, y: u32, buttons: u8) {
            let event = InputEvent::Pointer { x, y, buttons };
            self.recorder
                .post(&self.channel, &event.encode())
                .expect("a registered channel");
        }

        /// Sweep the pointer past the bottom-right corner, where Intuition
        /// pins it, and come back to a known place: the only way to agree on
        /// absolute position with a relative mouse.
        fn home(&mut self) {
            self.pointer(799, 567, 0);
            self.run_ms(20);
            self.pointer(SCREEN_LEFT, SCREEN_TOP, 0);
            self.run_ms(300);
        }

        fn double_click(&mut self, (x, y): (u32, u32)) {
            self.pointer(x, y, 0);
            self.run_ms(300);
            for _ in 0..2 {
                self.pointer(x, y, 1);
                self.run_ms(HOLD_MS);
                self.pointer(x, y, 0);
                self.run_ms(HOLD_MS);
            }
        }

        fn look(&self, name: &str) -> u64 {
            let info = self.scanout.info();
            let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
            self.scanout.capture(&mut surface);
            let hash = super::frame_hash(&surface);
            if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
                #[cfg(feature = "display-png")]
                {
                    let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
                    std::fs::write(
                        std::path::Path::new(&dir)
                            .join(format!("{}-screenmode-{name}.png", self.tag)),
                        png,
                    )
                    .expect("the frame directory is writable");
                }
                #[cfg(not(feature = "display-png"))]
                let _ = dir;
            }
            println!("{}-screenmode {name}: frame {hash:#018x}", self.tag);
            hash
        }
    }

    /// The volume icon in the Workbench window, the Prefs drawer in the volume
    /// window, and the ScreenMode icon in the Prefs window.
    const VOLUME_ICON: (u32, u32) = (VOL_X, VOL_Y);
    const VOL_X: u32 = 188;
    const VOL_Y: u32 = 175;
    const PREFS_DRAWER: (u32, u32) = (200, 178);
    const SCREENMODE_ICON: (u32, u32) = (434, 210);

    /// Open the volume, the Prefs drawer and ScreenMode, and hold each picture
    /// to its golden.
    fn session(board: &str, rom: &str, tag: &'static str, goldens: [u64; 4]) {
        let Some(mut d) = desk(board, rom, tag) else {
            return;
        };
        d.run_ms(15_000);
        d.home();
        let desktop = d.look("0-desktop");
        d.double_click(VOLUME_ICON);
        d.run_ms(6_000);
        let volume = d.look("1-volume");
        d.double_click(PREFS_DRAWER);
        d.run_ms(8_000);
        let prefs = d.look("2-prefs");
        d.double_click(SCREENMODE_ICON);
        d.run_ms(8_000);
        let modes = d.look("3-screenmode");
        assert!(!d.cpu.is_halted(), "{tag}: the processor double-faulted");
        assert_eq!(d.cpu.bus_faults().0, 0, "{tag}: an access faulted");
        assert_eq!(
            [desktop, volume, prefs, modes],
            goldens,
            "{tag}: a picture moved; look at them (RSEMU_AMIGA_FRAME_DIR) before accepting \
             the new hashes"
        );
    }

    /// The A1200, Workbench 3.1 off the hard disk. Looked at:
    ///
    /// 1. The desktop, the same frame the hard-disk boot ends on.
    /// 2. The "Workbench3.1" window open over it, "73% full, 1,661K free,
    ///    4,450K in", holding Prefs, Utilities, System, Devs, Expansion,
    ///    Tools, WBStartup and Storage; the screen title reads "Amiga
    ///    Workbench 1,822,400 graphics mem 0 other mem" — Exec found the whole
    ///    2 MiB of chip RAM, which is what Alice reaches and an 8372A does
    ///    not.
    /// 3. The Prefs window: Font, Locale, Pointer, PrinterPS, Sound, IControl,
    ///    Overscan, Printer, ScreenMode, Time, Input, Palette, PrinterGfx,
    ///    Serial and WBPattern, under "1,799,048 graphics mem".
    /// 4. "ScreenMode Preferences": the display-mode list, "PAL:High Res"
    ///    selected, "Visible Size 640 x 256", "Minimum Size 640 x 200",
    ///    "Maximum Size 16368 x 16384", **"Maximum Colors: 256"**, "Supports
    ///    genlock / Draggable / 50Hz, 15.60kHz", and Colors at 4 on a slider
    ///    that now runs to 256.
    ///
    /// When `LISAID` moved from `$FFF8` to `$00F8` the last three moved, and a
    /// pixel diff of each against the frame before says exactly where: in 2
    /// and 3 only the digits of the title bar's free-memory figure, which was
    /// 1,822,912 — the four-times screen takes 512 bytes more of chip RAM — and
    /// in 4 only the "Maximum Colors" number and the Colors slider's knob,
    /// which is narrower because its range is now 256. The desktop in 1 did
    /// not move, though it is now fetched sixty-four bits at a time.
    #[test]
    fn screenmode_offers_256_colours() {
        session(
            "amiga-a1200",
            super::ROM,
            "a1200",
            [
                0xb360_d7a3_0bf0_bc7d,
                0x5eda_c751_851b_78f1,
                0xc665_2a73_fd36_3605,
                0x2551_7f0b_e6fa_8209,
            ],
        );
    }

    /// The same four clicks on the A600, whose chips are an 8375 and an 8373:
    /// the control for the test above.
    ///
    /// Its fourth picture is the A1200's window in every word but one — the
    /// same mode list, the same "Maximum Size 16368 x 16384" — with
    /// **"Maximum Colors: 16"**, at half the horizontal resolution, because an
    /// 8373's picture is 70 ns columns where Lisa's are 35 ns ones. An 8373
    /// drives none of `DENISEID`'s upper byte, so bits 9 and 8 read as ones,
    /// one times the bandwidth, and the ROM offers what an ECS machine has.
    #[cfg(feature = "machine-amiga-a600")]
    #[test]
    fn the_same_session_on_the_ecs_board() {
        session(
            "amiga-a600",
            "amiga-os-310-a600.rom",
            "a600",
            [
                0xa06f_52db_6660_b2a5,
                0x8215_9ca0_f784_5b81,
                0x682a_63b4_c7b5_fef9,
                0x0851_b0bf_3943_b87d,
            ],
        );
    }
}
