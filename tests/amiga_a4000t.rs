//! An Amiga 4000T booting Workbench 3.1 off its **SCSI** port, on the user's
//! own ROMs.
//!
//! # Why this file exists
//!
//! `amiga-os-310-a4000t.rom` is the one Kickstart in the Amiga Forever set that
//! no board in this tree could boot. On `machines/amiga-a4000.machine` it finds
//! the IDE drive, reads its identification — and then stops, because an A4000T
//! also has an **NCR 53C710 SCSI I/O processor** on the motherboard and that
//! board has none. `machines/amiga-a4000t.machine` is that board with the chip
//! on it, and the only witness that the chip is right is Commodore's own
//! `scsi.device` executing its own SCRIPTS program out of guest memory, the
//! chip mastering the bus for every instruction and every byte, `scsi.device`
//! reading a Rigid Disk Block through it and AmigaDOS mounting what it finds.
//! That is what this file is.
//!
//! What it found on the way — where the chip is, which way round its byte lanes
//! are, which byte of `DSP` starts the processor, and that a selection's
//! destination is a bus line rather than a number — is in
//! `docs/platforms/amiga.md`, "A4000T".
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** A Kickstart is Cloanto/Amiga-proprietary and
//! a Workbench hard-disk image is a licensed product; both are the user's, read
//! in place from the directories these name:
//!
//! * `RSEMU_AMIGA_ROM_DIR` — an Amiga Forever `Shared/rom`, for
//!   `amiga-os-310-a4000t.rom` and `amiga-os-3x0-a4000t.rom`.
//! * `RSEMU_AMIGA_HDF_DIR` — an Amiga Forever `Shared/hdf`, for
//!   `workbench-311.hdf`, a whole disk whose first blocks are a Rigid Disk
//!   Block.
//! * `RSEMU_AMIGA_FRAME_DIR` — where each test writes its frame as a PNG, so a
//!   golden that moves can be looked at rather than argued about.
//! * `RSEMU_AMIGA_TRACE=1` — a line a second: program counter, status register,
//!   bus faults, fields, and what `graphics.library` has concluded about the
//!   chips.
//!
//! With the first two unset every test prints why it is skipping and passes,
//! which is what keeps `cargo test` hermetic.
//!
//! Each test asserts the same four things the other Amiga suites do — the
//! processor did not double fault, no access faulted, Denise produced fields,
//! and the frame hashes to what it hashed before — and four more this board is
//! for, all read back out of guest RAM: `GfxBase->ChipRevBits0` says AA,
//! `ExecBase->AttnFlags` says a 68040 with its on-chip FPU, `ExecBase` itself
//! is in the motherboard fast RAM, and a task called `DH0` exists, which it
//! does only if a partition was read off a disk and mounted.
//!
//! The board is the shipped `machines/amiga-a4000t.machine`, unchanged.
//! `--media` copies an image into the drive and the guest's writes stay in
//! memory, so a test never writes to the user's files.
//!
//! No Amiga emulator source, no FPGA reimplementation, no AROS source and no
//! Kickstart disassembly was consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a4000t", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
}

/// Which port a test puts the disk on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Port {
    /// The SCSI cable, behind the 53C710 — what this board is for.
    Scsi,
    /// The A4000's own IDE port, which an A4000T also has.
    Ide,
    /// Neither: both drives out.
    None,
}

/// The user's file `name` in the directory `var` names, or `None` having said
/// why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-a4000t: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a4000t: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A4000T around the user's Kickstart `rom`, with `hdf` on `port`
/// and nothing in DF0.
fn board(rom: &str, hdf: Option<&str>, port: Port) -> Option<Board> {
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
    // Whichever port is not under test gets an empty drive, which is a machine
    // Commodore sold: an A4000T came with one disk, not two.
    let (scsi, ide) = match port {
        Port::Scsi => (disk, Vec::new()),
        Port::Ide => (Vec::new(), disk),
        Port::None => (Vec::new(), Vec::new()),
    };
    options.realize.media.insert("scsi0", scsi);
    options.realize.media.insert("hd0", ide);
    // No floppy: the boot is a hard disk's or nothing.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a4000t")
        .expect("this build ships amiga-a4000t")
        .source;
    let machine = rsemu::machine::build("amiga-a4000t", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{rom}: the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a Lisa");
    Some(Board {
        machine,
        cpu,
        scanout,
    })
}

/// FNV-1a over the captured pixels, as every Amiga suite hashes: our
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

/// One guest-memory read, with no side effect: `MemAttrs::DEBUG`, which every
/// device on this board honours.
fn peek(b: &Board, addr: u32, width: Width) -> u32 {
    b.machine
        .space("mem")
        .expect("the memory space")
        .read(u64::from(addr), width, MemAttrs::DEBUG)
        .expect("guest memory") as u32
}

/// `graphics.library`'s base, found the way `OpenLibrary` finds it: exec's
/// `LibList` from `ExecBase`, node by node, by `ln_Name`.
///
/// ROM Kernel Reference Manual layouts (`exec/execbase.h`, `exec/nodes.h`,
/// `exec/lists.h`): `LibList` is the `struct List` at offset `$17A` of
/// `ExecBase`; a list's `lh_Head` is its first longword; a node's `ln_Succ` is
/// its first longword, zero on the list's tail, and `ln_Name` is at offset 10.
fn library_base(b: &Board, want: &[u8]) -> Option<u32> {
    let exec = peek(b, 4, Width::U32);
    let mut node = peek(b, exec + 0x17a, Width::U32);
    for _ in 0..64 {
        let next = peek(b, node, Width::U32);
        if next == 0 {
            return None;
        }
        let name = peek(b, node + 10, Width::U32);
        let bytes: Vec<u8> = (0..want.len() as u32)
            .map(|i| peek(b, name + i, Width::U8) as u8)
            .collect();
        if bytes == want {
            return Some(node);
        }
        node = next;
    }
    None
}

/// `gb_ChipRevBits0`, at offset `$EC` of `GfxBase` (`graphics/gfxbase.h`).
fn chip_rev_bits(b: &Board) -> Option<u8> {
    let gfx = library_base(b, b"graphics.library\0")?;
    Some(peek(b, gfx + 0xec, Width::U8) as u8)
}

/// Whether exec has a task called `want`, looking on both of its lists.
///
/// `exec/execbase.h`: `TaskReady` is the `struct List` at offset `$196` of
/// `ExecBase` and `TaskWait` the one at `$1A4`; `exec/lists.h` puts `lh_Head`
/// at the head of a list and `exec/nodes.h` `ln_Succ` at the head of a node,
/// zero on the tail, with `ln_Name` at offset 10. The running task is not on
/// either list, so `ThisTask` (`$114`) is checked as well.
///
/// This is the witness that a *disk* booted rather than a picture: AmigaDOS
/// names a file-system handler task after the device it mounted, so a task
/// called `DH0` exists only if a driver read the Rigid Disk Block, found a
/// partition in it and `dos.library` mounted what it found. On this board, off
/// the SCSI port, the driver that read it executed its own SCRIPTS program out
/// of guest memory through the 53C710.
fn task_exists(b: &Board, want: &[u8]) -> bool {
    let exec = peek(b, 4, Width::U32);
    let named = |node: u32| -> bool {
        if node == 0 {
            return false;
        }
        let name = peek(b, node + 10, Width::U32);
        if name == 0 {
            return false;
        }
        (0..=want.len() as u32).all(|i| {
            let c = peek(b, name + i, Width::U8) as u8;
            if i as usize == want.len() {
                c == 0
            } else {
                c == want[i as usize]
            }
        })
    };
    if named(peek(b, exec + 0x114, Width::U32)) {
        return true;
    }
    for head in [0x196u32, 0x1a4] {
        let mut node = peek(b, exec + head, Width::U32);
        for _ in 0..64 {
            let next = peek(b, node, Width::U32);
            if next == 0 {
                break;
            }
            if named(node) {
                return true;
            }
            node = next;
        }
    }
    false
}

/// `GFXF_HR_AGNUS` and `GFXF_HR_DENISE` (*Amiga Hardware Reference Manual*
/// Appendix C, *Determining Chip Revisions*; `graphics/gfxbase.h`).
const HR_AGNUS: u8 = 1 << 0;
const HR_DENISE: u8 = 1 << 1;
/// `GFXB_AA_ALICE` and `GFXB_AA_LISA`: the AA parts, which this board has.
const AA_ALICE: u8 = 1 << 2;
const AA_LISA: u8 = 1 << 3;

/// `ExecBase->AttnFlags`, the word at offset `$128` (`exec/execbase.h`:
/// `IDNestCnt` and `TDNestCnt` are the two bytes before it).
fn attn_flags(b: &Board) -> u16 {
    let exec = peek(b, 4, Width::U32);
    peek(b, exec + 0x128, Width::U16) as u16
}

/// `exec/execbase.h`'s `AttnFlags` bits, in bit order: `AFB_68010` 0,
/// `AFB_68020` 1, `AFB_68030` 2, `AFB_68040` 3, `AFB_68881` 4, `AFB_68882` 5,
/// `AFB_FPU40` 6.
const AFF_68010: u16 = 1 << 0;
const AFF_68020: u16 = 1 << 1;
const AFF_68030: u16 = 1 << 2;
const AFF_68040: u16 = 1 << 3;
const AFF_FPU40: u16 = 1 << 6;

/// Where `machines/amiga-a4000t.machine` puts the motherboard's fast RAM.
const FAST_RAM: u32 = 0x0700_0000;

/// Boot `rom` with `hdf` on `port` for `seconds` of virtual time, then check it
/// the way every Amiga suite does and compare the picture with `golden`.
fn boots_to(
    rom: &str,
    hdf: Option<&str>,
    port: Port,
    label: &str,
    seconds: u64,
    golden: u64,
) -> Option<Board> {
    let mut b = board(rom, hdf, port)?;
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in 1..=seconds {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} fields={} \
                 chiprev={:?} attn={:#06x} execbase={:08x} dh0={}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
                chip_rev_bits(&b),
                attn_flags(&b),
                peek(&b, 4, Width::U32),
                task_exists(&b, b"DH0"),
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
    Some(b)
}

/// The processor and the chip set the guest concludes it is running on.
///
/// `aa` says whether the two AA bits are expected. They are not always: as
/// `tests/amiga_a1200.rs` records and `tests/amiga_a4000.rs` reproduces,
/// `ChipRevBits0` is `$13` until the ROM has a **boot device** and `$1F` after,
/// so a run that ends at the insert-disk screen shows the ECS pair and bit 4
/// alone.
fn the_guest_sees_the_board(b: &Board, label: &str, aa: bool) {
    let bits = chip_rev_bits(b).expect("graphics.library is on exec's library list");
    println!("{label}: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & (HR_AGNUS | HR_DENISE),
        HR_AGNUS | HR_DENISE,
        "{label}: graphics.library did not find the chips at all"
    );
    if aa {
        assert_eq!(
            bits & (AA_ALICE | AA_LISA),
            AA_ALICE | AA_LISA,
            "{label}: graphics.library did not find Alice and Lisa"
        );
    }

    let attn = attn_flags(b);
    println!("{label}: AttnFlags = {attn:#06x}");
    assert_ne!(attn & AFF_68040, 0, "{label}: exec did not find a 68040");
    assert_ne!(attn & AFF_68030, 0, "{label}: a 68040 is also a 68030");
    assert_ne!(attn & AFF_68020, 0, "{label}: a 68040 is also a 68020");
    assert_ne!(attn & AFF_68010, 0, "{label}: a 68040 is also a 68010");
    assert_ne!(
        attn & AFF_FPU40,
        0,
        "{label}: exec did not find the on-chip FPU"
    );

    let exec = peek(b, 4, Width::U32);
    println!("{label}: ExecBase = {exec:#010x}");
    assert!(
        exec >= FAST_RAM,
        "{label}: `ExecBase` is at {exec:#010x}, not in the motherboard fast RAM at {FAST_RAM:#010x}"
    );
}

/// And the disk: a mounted partition, which only a driver that read the Rigid
/// Disk Block could have produced.
fn the_guest_mounted_the_disk(b: &Board, label: &str) {
    assert!(
        task_exists(b, b"DH0"),
        "{label}: no `DH0` task, so no partition was mounted"
    );
}

/// The whole point of the board: Commodore's `scsi.device` driving the 53C710
/// with its own SCRIPTS program, off a disk on the SCSI cable and nothing in
/// the IDE bay.
///
/// Sixty seconds rather than the A4000's thirty because this ROM scans the
/// whole cable — seven addresses, each with an eight-`LUN` sweep — before it
/// has a boot device at all.
#[test]
fn kickstart_3_1_boots_workbench_3_1_off_the_scsi_port() {
    let Some(b) = boots_to(
        "amiga-os-310-a4000t.rom",
        Some("workbench-311.hdf"),
        Port::Scsi,
        "a4000t-310-scsi-wb311",
        60,
        GOLDEN_310_SCSI,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000t-310-scsi-wb311", true);
    the_guest_mounted_the_disk(&b, "a4000t-310-scsi-wb311");
}

/// Cloanto's 3.X build of the same ROM, on the same cable.
#[test]
fn the_3x0_rom_boots_workbench_3_1_off_the_scsi_port() {
    let Some(b) = boots_to(
        "amiga-os-3x0-a4000t.rom",
        Some("workbench-311.hdf"),
        Port::Scsi,
        "a4000t-3x0-scsi-wb311",
        60,
        GOLDEN_3X0_SCSI,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000t-3x0-scsi-wb311", true);
    the_guest_mounted_the_disk(&b, "a4000t-3x0-scsi-wb311");
}

/// A real A4000T has both ports, so this board keeps both, and the same ROM
/// boots the same disk off the IDE one — which is also what says the SCSI port
/// did not break anything the A4000 already had.
#[test]
fn the_same_rom_boots_off_the_ide_port_too() {
    let Some(b) = boots_to(
        "amiga-os-310-a4000t.rom",
        Some("workbench-311.hdf"),
        Port::Ide,
        "a4000t-310-ide-wb311",
        45,
        GOLDEN_310_IDE,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000t-310-ide-wb311", true);
    the_guest_mounted_the_disk(&b, "a4000t-310-ide-wb311");
}

/// Both drives out: the machine scans an empty cable and an empty bay and draws
/// its insert-disk screen.
#[test]
fn kickstart_3_1_asks_for_a_disk_with_both_ports_empty() {
    let Some(b) = boots_to(
        "amiga-os-310-a4000t.rom",
        None,
        Port::None,
        "a4000t-310-empty",
        60,
        GOLDEN_310_EMPTY,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000t-310-empty", false);
    assert!(
        !task_exists(&b, b"DH0"),
        "a4000t-310-empty: nothing to mount, so no handler"
    );
}

/// At 60 s, Kickstart 3.1 (40.068) for the A4000T with `workbench-311.hdf` on
/// the SCSI cable: the **Workbench 3.1 desktop**, 1600×568 in AGA. A light grey
/// backdrop; along the top, "Copyright © 1985-1993 Commodore-Amiga, Inc. All
/// Rights Reserved." with the screen's depth gadgets at the right; below it the
/// open "Workbench" window in its blue bordering, holding the "Ram Disk" icon
/// and, under it, the "Workbench3.1" hard-disk icon; scroll bars and arrows
/// down the right and along the bottom; the red arrow pointer at the top left,
/// where the mouse has not moved.
const GOLDEN_310_SCSI: u64 = 0xb360_d7a3_0bf0_bc7d;
/// At 60 s, `amiga-os-3x0-a4000t.rom` with the same disk on the same cable:
/// the same desktop, in the same place, with "Copyright © 1985-2017 Cloanto
/// Corporation and its licensors." along the title bar instead.
const GOLDEN_3X0_SCSI: u64 = 0x6795_35c6_e5cc_c015;
/// At 45 s, Kickstart 3.1 with the same disk on the IDE port instead — the
/// same desktop, pixel for pixel, which is the same number
/// `tests/amiga_a4000.rs` pins for an A4000: the picture does not know which
/// cable the blocks came down.
const GOLDEN_310_IDE: u64 = 0xb360_d7a3_0bf0_bc7d;
/// At 60 s, Kickstart 3.1 with both ports empty: the insert-disk screen on a
/// dark purple field, 1600×568. The Amiga check-mark in its blue-to-red
/// gradient, four lines of orange text below it — "3.1 ROM   40.070 /
/// Copyright © 1985-1993 / Commodore-Amiga, Inc. / All Rights Reserved." —
/// and, to the right, the drive slot with no diskette under it. `40.070` is
/// this ROM's own revision, one above the A4000's `40.068`.
const GOLDEN_310_EMPTY: u64 = 0xd800_8ccf_d186_4795;
