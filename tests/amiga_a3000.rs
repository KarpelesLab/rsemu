//! An Amiga 3000 booting Workbench off its SCSI disk, on the user's own ROMs.
//!
//! # Why this file exists
//!
//! `machines/amiga-a3000.machine` is the first board in this tree with a SCSI
//! port, and a SCSI port is three objects — a bus, a target and a controller —
//! plus a DMA controller that masters memory. Every one of those is a place a
//! model can be plausible and wrong, and the only witness that says otherwise
//! is Commodore's own `scsi.device` reading a Rigid Disk Block off it and
//! AmigaDOS mounting what it finds. That is what this file is.
//!
//! What it found on the way:
//!
//! | Stopped at | Why | Fixed in |
//! | --- | --- | --- |
//! | (see `docs/platforms/amiga.md`, "A3000 — defects found on the way") | | |
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** A Kickstart is Cloanto/Amiga-proprietary and
//! a Workbench hard-disk image is a licensed product; both are the user's, read
//! in place from the directories these name:
//!
//! * `RSEMU_AMIGA_ROM_DIR` — an Amiga Forever `Shared/rom`, for
//!   `amiga-os-310-a3000.rom` and `amiga-os-204-a3000.rom`.
//! * `RSEMU_AMIGA_HDF_DIR` — an Amiga Forever `Shared/hdf`, for
//!   `workbench-311.hdf` and `workbench-211.hdf`, whole disks whose first
//!   blocks are a Rigid Disk Block.
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
//! and the frame hashes to what it hashed before — and three more this board is
//! for, all read back out of guest RAM: `GfxBase->ChipRevBits0` says ECS,
//! `ExecBase->AttnFlags` says a 68030 with a 68882, and `ExecBase` itself has
//! moved into the motherboard fast RAM behind Ramsey. The two that boot off the
//! disk add a fourth: a task called `DH0`, which exists only if `scsi.device`
//! read the Rigid Disk Block and AmigaDOS mounted a partition out of it.
//!
//! The board is the shipped `machines/amiga-a3000.machine`, unchanged.
//! `--media` copies an image into the drive and the guest's writes stay in
//! memory, so a test never writes to the user's files.
//!
//! No Amiga emulator source, no AROS source and no Kickstart disassembly was
//! consulted (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-amiga-a3000", feature = "media-kickstart"))]

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
        println!("amiga-a3000: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a3000: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A3000 around the user's Kickstart `rom`, with `hdf` on the SCSI
/// bus (`None` is an empty address) and nothing in DF0.
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
    let source = catalog::machine("amiga-a3000")
        .expect("this build ships amiga-a3000")
        .source;
    let machine = rsemu::machine::build("amiga-a3000", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{rom}: the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a Denise");
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
        .expect("chip RAM") as u32
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
/// `GFXB_AA_ALICE`, which an ECS board must **not** show.
const AA_ALICE: u8 = 1 << 2;

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
const AFF_68881: u16 = 1 << 4;
const AFF_68882: u16 = 1 << 5;

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
                 chiprev={:?} attn={:#06x}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
                chip_rev_bits(&b),
                attn_flags(&b),
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
/// called `DH0` exists only if `scsi.device` read the Rigid Disk Block, found
/// a partition in it and `dos.library` mounted what it found.
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

/// The processor and the chip set the guest concludes it is running on.
fn the_guest_sees_the_board(b: &Board, label: &str) {
    let bits = chip_rev_bits(b).expect("graphics.library is on exec's library list");
    println!("{label}: ChipRevBits0 = {bits:#04x}");
    assert_eq!(
        bits & (HR_AGNUS | HR_DENISE),
        HR_AGNUS | HR_DENISE,
        "{label}: graphics.library did not find the ECS chips"
    );
    assert_eq!(
        bits & AA_ALICE,
        0,
        "{label}: an 8372B is not Alice, and this board has no AA in it"
    );

    // The motherboard fast RAM, witnessed by the guest rather than by the
    // machine file: exec relocates `ExecBase` into the fastest memory it has
    // found, so an `ExecBase` above `$07000000` is Kickstart having sized
    // Ramsey's window, believed it, and moved in. §2.2 of the A3000+ System
    // Specification puts that window "from $07FFFFFF building down", so with
    // the shipped 4 MiB it starts at `$07C00000`.
    let exec = peek(b, 4, Width::U32);
    println!("{label}: ExecBase = {exec:#010x}");
    assert!(
        (0x07C0_0000..0x0800_0000).contains(&exec),
        "{label}: exec did not move into the motherboard fast RAM ({exec:#010x})"
    );

    let attn = attn_flags(b);
    println!("{label}: AttnFlags = {attn:#06x}");
    assert_ne!(attn & AFF_68030, 0, "{label}: exec did not find a 68030");
    assert_ne!(attn & AFF_68020, 0, "{label}: a 68030 is also a 68020");
    assert_ne!(attn & AFF_68010, 0, "{label}: a 68030 is also a 68010");
    assert_eq!(attn & AFF_68040, 0, "{label}: this is not a 68040");
    assert_ne!(attn & AFF_68882, 0, "{label}: exec did not find the 68882");
    assert_ne!(
        attn & AFF_68881,
        0,
        "{label}: a 68882 sets the 68881 bit too"
    );
}

#[test]
fn kickstart_3_1_asks_for_a_disk_with_an_empty_scsi_bus() {
    let Some(b) = boots_to(
        "amiga-os-310-a3000.rom",
        None,
        "a3000-310-empty",
        25,
        GOLDEN_310_EMPTY,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a3000-310-empty");
}

#[test]
fn kickstart_2_04_asks_for_a_disk_with_an_empty_scsi_bus() {
    let Some(b) = boots_to(
        "amiga-os-204-a3000.rom",
        None,
        "a3000-204-empty",
        25,
        GOLDEN_204_EMPTY,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a3000-204-empty");
}

#[test]
fn kickstart_3_1_boots_workbench_3_1_off_the_scsi_disk() {
    let Some(b) = boots_to(
        "amiga-os-310-a3000.rom",
        Some("workbench-311.hdf"),
        "a3000-310-wb311",
        20,
        GOLDEN_310_WB311,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a3000-310-wb311");
    assert!(
        task_exists(&b, b"DH0"),
        "a3000-310-wb311: AmigaDOS mounted the RDB partition off the SCSI disk"
    );
}

#[test]
fn kickstart_2_04_boots_workbench_2_1_off_the_scsi_disk() {
    let Some(b) = boots_to(
        "amiga-os-204-a3000.rom",
        Some("workbench-211.hdf"),
        "a3000-204-wb211",
        20,
        GOLDEN_204_WB211,
    ) else {
        return;
    };
    the_guest_sees_the_board(&b, "a3000-204-wb211");
    assert!(
        task_exists(&b, b"DH0"),
        "a3000-204-wb211: AmigaDOS mounted the RDB partition off the SCSI disk"
    );
}

/// At 25 s, Kickstart 3.1 with nothing on the SCSI bus: the insert-disk
/// screen. A dark purple field; the Amiga check-mark in its blue-to-red
/// gradient above four lines of orange text — "3.1 ROM   40.068 / Copyright ©
/// 1985-1993 / Commodore-Amiga, Inc. / All Rights Reserved." — and, to the
/// right, the diskette held just below the drive slot, mid-animation.
const GOLDEN_310_EMPTY: u64 = 0xdea0_a1a3_8da5_1b19;
/// At 25 s, Kickstart 2.04 with nothing on the SCSI bus: the same screen in
/// 2.0's wording — "2.0 Roms (37.175) / Copyright © 1985-1991 / …" — with the
/// diskette at a different point of the same slide.
const GOLDEN_204_EMPTY: u64 = 0x74ea_2a62_c231_b235;
/// At 20 s, Kickstart 3.1 with `workbench-311.hdf` at SCSI address 0: **the
/// Workbench desktop**. The grey 3.1 backdrop under the title bar's "Copyright
/// © 1985-1993 Commodore-Amiga, Inc. All Rights Reserved.", the Workbench
/// window open across it with its scroll bars and sizing gadget, and two icons
/// in it — "Ram Disk" and the hard-disk icon labelled "Workbench3.1", which is
/// the volume name of the partition the Rigid Disk Block describes. The red
/// arrow pointer sits at the top left where it starts.
const GOLDEN_310_WB311: u64 = 0xa06f_52db_6660_b2a5;
/// At 20 s, Kickstart 2.04 with `workbench-211.hdf` at SCSI address 0: the same
/// desktop in 2.1's furniture — "Copyright © 1985-1991", the same Workbench
/// window, "Ram Disk" and "Workbench2.1".
const GOLDEN_204_WB211: u64 = 0xaa2b_caad_a888_cacd;
