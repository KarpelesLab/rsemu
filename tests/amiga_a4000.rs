//! An Amiga 4000 booting Workbench 3.1 off its IDE port, on the user's own
//! ROMs.
//!
//! # Why this file exists
//!
//! `machines/amiga-a4000.machine` is the first board in this tree with a
//! **68040**, and the first with **motherboard fast RAM**. Either on its own
//! changes what `exec` does with the machine — a 68040 brings `AFF_68040` and
//! `AFF_FPU40`, and fast RAM moves `ExecBase` out of chip RAM into the
//! `$07000000` window — and the A4000's IDE port is at an address Commodore's
//! published documentation does not print, so it was found by watching what
//! the ROM touches. The only witness that the three together are right is
//! Commodore's own `scsi.device` reading a Rigid Disk Block off the port and
//! AmigaDOS mounting what it finds. That is what this file is.
//!
//! What it found on the way is in `docs/platforms/amiga.md`, "A4000".
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** A Kickstart is Cloanto/Amiga-proprietary and
//! a Workbench hard-disk image is a licensed product; both are the user's, read
//! in place from the directories these name:
//!
//! * `RSEMU_AMIGA_ROM_DIR` — an Amiga Forever `Shared/rom`, for
//!   `amiga-os-310-a4000.rom`, `amiga-os-3x0-a4000.rom` and
//!   `amiga-os-310-a4000t.rom`.
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
//! and the frame hashes to what it hashed before — and three more this board
//! is for: `GfxBase->ChipRevBits0` says AA, `ExecBase->AttnFlags` says a 68040
//! with its on-chip FPU, and `ExecBase` itself is in the motherboard fast RAM.
//!
//! The board is the shipped `machines/amiga-a4000.machine`, unchanged.
//! `--media` copies an image into the drive and the guest's writes stay in
//! memory, so a test never writes to the user's files.
//!
//! No Amiga emulator source, no AROS source and no Kickstart disassembly was
//! consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a4000", feature = "media-kickstart"))]

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

/// The user's file `name` in the directory `var` names, or `None` having said
/// why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-a4000: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a4000: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A4000 around the user's Kickstart `rom`, with `hdf` on the IDE
/// port (`None` is an empty bay) and nothing in DF0.
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
    let source = catalog::machine("amiga-a4000")
        .expect("this build ships amiga-a4000")
        .source;
    let machine = rsemu::machine::build("amiga-a4000", source, &registry, &options)
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

/// Where `machines/amiga-a4000.machine` puts the motherboard's fast RAM.
const FAST_RAM: u32 = 0x0700_0000;

/// Boot `rom` with `hdf` for `seconds` of virtual time, then check it the way
/// every Amiga suite does and compare the picture with `golden`.
fn boots_to(rom: &str, hdf: Option<&str>, label: &str, seconds: u64, golden: u64) -> Option<Board> {
    let mut b = board(rom, hdf)?;
    let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
    for s in 1..=seconds {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} fields={} \
                 chiprev={:?} attn={:#06x} execbase={:08x}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
                chip_rev_bits(&b),
                attn_flags(&b),
                peek(&b, 4, Width::U32),
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
/// `tests/amiga_a1200.rs` records and this board reproduces, `ChipRevBits0` is
/// `$13` until the ROM has a **boot device** and `$1F` after, so a run that
/// ends at the insert-disk screen shows the ECS pair and bit 4 alone.
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

    // And the motherboard's fast RAM, which `exec` says it found by *living*
    // in it: with a 32-bit memory list, `ExecBase` is relocated out of chip
    // RAM into the fastest memory there is. With the `$07000000` window empty
    // it stays at `$00001xxx`, so this is a statement about the board rather
    // than about the ROM.
    let exec = peek(b, 4, Width::U32);
    println!("{label}: ExecBase = {exec:#010x}");
    assert!(
        exec >= FAST_RAM,
        "{label}: `ExecBase` is at {exec:#010x}, not in the motherboard fast RAM at {FAST_RAM:#010x}"
    );
}

#[test]
fn kickstart_3_1_boots_workbench_3_1_off_the_ide_port() {
    let Some(b) = boots_to(
        "amiga-os-310-a4000.rom",
        Some("workbench-311.hdf"),
        "a4000-310-wb311",
        30,
        GOLDEN_310_WB311,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000-310-wb311", true);
}

/// With no drive in the bay the ROM draws its AGA insert-disk screen — and it
/// takes its time about it: the machine sits idle, in `STOP`, until about 35 s
/// of guest time and *then* draws. 45 s is that plus room, which is the same
/// shape `tests/amiga_a1200.rs` records for the A1200.
#[test]
fn kickstart_3_1_asks_for_a_disk_with_an_empty_bay() {
    let Some(b) = boots_to(
        "amiga-os-310-a4000.rom",
        None,
        "a4000-310-empty",
        45,
        GOLDEN_310_EMPTY,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000-310-empty", false);
}

#[test]
fn the_3x0_rom_boots_workbench_3_1_off_the_ide_port() {
    let Some(b) = boots_to(
        "amiga-os-3x0-a4000.rom",
        Some("workbench-311.hdf"),
        "a4000-3x0-wb311",
        30,
        GOLDEN_3X0_WB311,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a4000-3x0-wb311", true);
}

/// The A4000**T**'s ROM on an A4000: it finds the drive on the IDE port and
/// reads its identification, and then stops, because an A4000T also has an
/// **NCR 53C710** SCSI controller on the motherboard and this board has none.
///
/// Recorded with the whole of `$00D80000`–`$00DDFFFF` answered by a recorder:
/// the ROM writes and reads `$00DD0040`–`$00DD00EE` — a register file that is
/// not the IDE port's and not Ramsey's — and then polls `$00DD0062` several
/// hundred times and gives up, leaving the machine idle at a black screen with
/// `ExecBase` in fast RAM. So this is the board being honest about what it is,
/// not a defect in the port: the same disk, the same port and the same drive
/// boot under both of the other two ROMs. A machine file for an A4000T would
/// need that controller, which is a separate part and a separate job.
#[test]
fn the_a4000t_rom_finds_the_drive_and_stops_looking_for_its_scsi_controller() {
    boots_to(
        "amiga-os-310-a4000t.rom",
        Some("workbench-311.hdf"),
        "a4000t-310-wb311",
        30,
        GOLDEN_A4000T_WB311,
    );
}

/// At 30 s, Kickstart 3.1 (40.068) with `workbench-311.hdf` on the IDE port:
/// the **Workbench 3.1 desktop**, 1600×568 in AGA. A light grey backdrop; the
/// screen's title bar reads "Copyright © 1985-1993 Commodore-Amiga, Inc. All
/// Rights Reserved." with the depth gadgets at its right; below it the open
/// "Workbench" window in its blue bordering, holding the "Ram Disk" icon and,
/// under it, the "Workbench3.1" hard-disk icon; the scroll bars and their
/// arrows down the right and along the bottom; and the red arrow pointer at
/// the top left where the mouse has not moved.
const GOLDEN_310_WB311: u64 = 0xb360_d7a3_0bf0_bc7d;
/// At 45 s, Kickstart 3.1 with an empty bay: the insert-disk screen on a dark
/// purple field, 1600×568. The Amiga check-mark in its blue-to-red gradient,
/// four lines of orange text below it — "3.1 ROM   40.068 / Copyright © 1985-
/// 1993 / Commodore-Amiga, Inc. / All Rights Reserved." — and, to the right,
/// the drive slot with the diskette below it, mid-animation.
const GOLDEN_310_EMPTY: u64 = 0x14d5_771f_6d3d_28b9;
/// At 30 s, the 3.X ROM (`amiga-os-3x0-a4000.rom`) with the same disk: the
/// same desktop, in the same place, with "Copyright © 1985-2017 Cloanto
/// Corporation and its licensors." along the title bar instead.
const GOLDEN_3X0_WB311: u64 = 0x6795_35c6_e5cc_c015;
/// At 30 s, the A4000T ROM with the same disk: **black**, for the reason the
/// test above it gives. Pinned so it cannot silently change.
const GOLDEN_A4000T_WB311: u64 = 0x8f4d_780b_c4f5_8525;
