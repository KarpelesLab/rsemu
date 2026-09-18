//! ADF disks in DF0, read and written the way an Amiga does it: a whole raw
//! track at a time, through Paula, to and from chip RAM.
//!
//! The board is the shipped `machines/amiga-a500.machine`, whose drive names
//! the `df0` media slot.
//!
//! # What is proved without anybody's files
//!
//! A synthetic ADF is built here, different in every byte of every sector. A
//! 68000 program hand-assembled from the MC68000 user's manual's instruction
//! formats does what `trackdisk.device` does on the way to a sector (Amiga
//! Hardware Reference Manual, chapter 8, "Floppy Disk Controller"): motor on
//! and DF0 selected through CIA-B, the head stepped in and a side picked,
//! `ADKCON` and `DSKSYNC` loaded, Agnus's `DSKPT` pointed at chip RAM, and a
//! transfer started with `DSKLEN` written twice.
//!
//! * **Reading**, the test decodes what the guest got in chip RAM with nothing
//!   but the format rules, and every sector of the track must be there, carry
//!   this track's number, pass both checksums and hold the ADF's bytes.
//! * **Writing**, the guest writes a re-encoded track back out and steps off
//!   it, and the file behind the drive — a medium, as `--drive df0=` installs —
//!   must hold the new sectors at that track's offsets and nothing else
//!   changed.
//!
//! # What is proved with them
//!
//! Behind `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_ADF_DIR` (Amiga Forever's
//! `Shared/rom` and `Shared/adf`), a real Kickstart boots a real Workbench disk,
//! read in place — and refuses the same disk with one cell of every sector's
//! checksum flipped. That pair is the evidence the checksum arithmetic has
//! from outside this repository: `trackdisk.device` checks both sums on every
//! sector, and it is the one that decides. Nothing of either file is copied,
//! kept, or asserted about beyond where the head went.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::{MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::dev::amiga::adf;
use rsemu::dev::amiga::floppy::Floppy;
use rsemu::dev::medium::{self, Medium};
use rsemu::machine::{Machine, catalog};

/// The board this file runs: the shipped A500.
fn board() -> &'static str {
    catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source
}

/// Where the guest has Agnus put, or find, the track.
const BUFFER: u32 = 0x1_0000;

/// Where in the ROM a write program keeps the track it copies to [`BUFFER`].
const ROM_TRACK: u32 = 0x1_0000;

/// The words a read asks for: a revolution is 6250, so this is a little more,
/// the manual's "slightly more than a full track read".
const READ_WORDS: u16 = 6400;

/// The words a write puts down: exactly one revolution.
const WRITE_WORDS: u16 = ((adf::SECTORS * adf::SECTOR_MFM_BYTES + adf::GAP_BYTES) / 2) as u16;

/// `INTREQR`, and its `DSKBLK` bit.
const INTREQR: u64 = 0xDF_F01E;
const DSKBLK: u64 = 1 << 1;

/// An ADF in which every byte of every sector depends on where it is.
fn synthetic_adf() -> Vec<u8> {
    (0..adf::ADF_BYTES)
        .map(|i| {
            let sector = i / adf::SECTOR_BYTES;
            (i as u8) ^ (sector as u8).wrapping_mul(37) ^ (sector >> 8) as u8
        })
        .collect()
}

// ---------------------------------------------------------------------------
// The guest
// ---------------------------------------------------------------------------

/// CIA-B's port B for DF0 selected with the motor on (Table 8-5: bit 7 `MTR*`,
/// 3 `SEL0*`, 2 `SIDE*`, 1 `DIR`, 0 `STEP*`), on `side`, stepping `inwards`,
/// with `STEP*` at `step`.
fn prb(side: u8, inwards: bool, step: bool) -> u16 {
    let side_bit = if side == 1 { 0 } else { 0x04 };
    let dir_bit = if inwards { 0 } else { 0x02 };
    0x70 | side_bit | dir_bit | u16::from(step)
}

/// `move.b #v,$BFD100`: CIA-B `PRB`.
fn to_prb(v: u16) -> [u16; 4] {
    [0x13fc, v, 0x00bf, 0xd100]
}

/// `move.w #v,$DFFxxx`.
fn to_custom(reg: u16, v: u16) -> [u16; 4] {
    [0x33fc, v, 0x00df, 0xf000 | reg]
}

/// Out of the overlay, DF0 selected, the head on `cylinder`, `side` picked.
#[rustfmt::skip]
fn seek(cylinder: u8, side: u8) -> Vec<u16> {
    assert!(cylinder >= 1, "the loop steps at least once");
    let mut code = vec![
        0x13fc, 0x0001, 0x00bf, 0xe201, // move.b #$01,$BFE201   CIA-A DDRA: PA0 out
        0x13fc, 0x0000, 0x00bf, 0xe001, // move.b #$00,$BFE001   OVL off
    ];
    // `PRB` before `DDRB`, which is the order every Kickstart writes them in:
    // `PRB` is zero out of reset, so making port B an output first would pull
    // `STEP*`, `SEL0*` and `MTR*` to ground, and releasing them again steps the
    // head a cylinder nobody asked for.
    code.extend(to_prb(0xff));                   // all inactive
    code.extend([
        0x13fc, 0x00ff, 0x00bf, 0xd300,          // move.b #$FF,$BFD300  DDRB: all out
    ]);
    code.extend(to_prb(0x7f));                   // MTR* low: the motor before the select
    code.extend(to_prb(0x77));                   // SEL0* low
    code.extend(to_prb(prb(0, true, true)));     // DIR low: inwards
    code.push(0x7000 | u16::from(cylinder - 1)); // moveq #cyl-1,d0
    code.extend(to_prb(prb(0, true, false)));    // STEP* low
    code.extend(to_prb(prb(0, true, true)));     // STEP* high
    code.extend([0x51c8, (-18i16) as u16]);      // dbra d0,*-16: back to STEP* low
    code.extend(to_prb(prb(side, true, true)));  // SIDE*
    code
}

/// Point `DSKPT` at [`BUFFER`], clear every request, enable disk DMA, and
/// start a transfer of `dsklen` — written twice, which is what starts one.
fn transfer(adkcon: u16, dsklen: u16) -> Vec<u16> {
    let mut code = Vec::new();
    code.extend(to_custom(0x09e, 0x7f00)); // ADKCON: clear the disk bits
    code.extend(to_custom(0x09e, adkcon));
    code.extend(to_custom(0x07e, 0x4489)); // DSKSYNC
    code.extend(to_custom(0x020, (BUFFER >> 16) as u16)); // DSKPTH
    code.extend(to_custom(0x022, BUFFER as u16)); // DSKPTL
    code.extend(to_custom(0x09c, 0x7fff)); // INTREQ: clear everything
    code.extend(to_custom(0x096, 0x8210)); // DMACON: SET|DMAEN|DSKEN
    code.extend(to_custom(0x024, dsklen)); // DSKLEN
    code.extend(to_custom(0x024, dsklen)); // ... twice
    code
}

/// A Kickstart-shaped image: the reset vectors, `code` at `$F8000C`, and
/// `data` at [`ROM_TRACK`].
fn rom(code: &[u16], data: &[u8]) -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x00F8_000Cu32.to_be_bytes());
    for (i, word) in code.iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    let at = ROM_TRACK as usize;
    image[at..at + data.len()].copy_from_slice(data);
    image
}

/// Seek, then read [`READ_WORDS`] with `WORDSYNC` on, then stop.
fn reader(cylinder: u8, side: u8) -> Vec<u8> {
    let mut code = seek(cylinder, side);
    // SET|MFMPREC|WORDSYNC|FAST, and DMAEN with the length.
    code.extend(transfer(0x9500, 0x8000 | READ_WORDS));
    code.push(0x60fe); // bra *
    rom(&code, &[])
}

/// Seek, copy `track` out of the ROM into chip RAM, write it, wait for the
/// write to finish, and step one cylinder out.
#[rustfmt::skip]
fn writer(cylinder: u8, side: u8, track: &[u8]) -> Vec<u8> {
    assert_eq!(track.len(), usize::from(WRITE_WORDS) * 2);
    let mut code = seek(cylinder, side);
    code.extend([
        0x41f9, 0x00f8 | (ROM_TRACK >> 16) as u16, ROM_TRACK as u16, // lea $F9xxxx,a0
        0x43f9, (BUFFER >> 16) as u16, BUFFER as u16,                 // lea BUFFER,a1
        0x303c, WRITE_WORDS * 2 - 1,                                  // move.w #n-1,d0
        0x12d8,                                                       // move.b (a0)+,(a1)+
        0x51c8, (-4i16) as u16,                                       // dbra d0,*-2
    ]);
    // SET|MFMPREC|FAST, no precompensation; DMAEN|WRITE with the length.
    code.extend(transfer(0x9100, 0xc000 | WRITE_WORDS));
    code.extend([
        0x3039, 0x00df, 0xf01e, // move.w $DFF01E,d0   INTREQR
        0x0800, 0x0001,         // btst   #1,d0        DSKBLK
        0x67f4,                 // beq.s  *-10
    ]);
    code.extend(to_prb(prb(side, false, true)));  // DIR high: outwards
    code.extend(to_prb(prb(side, false, false))); // STEP* low
    code.extend(to_prb(prb(side, false, true)));  // STEP* high
    code.push(0x60fe);                            // bra *
    rom(&code, track)
}

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// Build the board with `kickstart` and `df0` bound, keeping the drive. With
/// `medium`, the drive's disk is that medium instead, as `--drive df0=` has it.
fn build(
    kickstart: Vec<u8>,
    df0: Vec<u8>,
    medium: Option<Arc<dyn Medium>>,
) -> (Machine, Arc<Floppy>) {
    let drives: Arc<Captured<Floppy>> = Arc::new(Captured::new());
    let kept = Arc::clone(&drives);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("amiga.floppy", move |props| {
        let drive = Arc::new(Floppy::new(props)?);
        kept.push(&drive);
        Ok(drive)
    });
    if let Some(medium) = medium {
        assert!(medium::install(&options.realize.hosts, "df0", medium).expect("installs"));
    }
    options.realize.media.insert("kickstart", kickstart);
    options.realize.media.insert("df0", df0);
    options.realize.media.insert("ext", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build("amiga-a500", board(), &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let drive = drives.last().expect("the binding captured the drive");
    (machine, drive)
}

fn peek(m: &Machine, addr: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEFAULT)
        .expect("mapped")
}

#[test]
fn an_unbound_df0_is_an_empty_drive() {
    let (_, drive) = build(reader(1, 0), Vec::new(), None);
    assert!(drive.disk().is_none());
}

#[test]
fn a_guest_reads_an_adf_track_off_paula_and_every_sector_checks_out() {
    let image = synthetic_adf();
    // Cylinder 40, side 1: track 81, far enough in that a header carrying the
    // wrong track number, or a side read as the other one, cannot pass.
    let (cylinder, side) = (40u8, 1u8);
    let track = usize::from(cylinder) * 2 + usize::from(side);
    let (mut m, drive) = build(reader(cylinder, side), image.clone(), None);

    // A revolution is 200 ms; the read is a little over one, and the first
    // sync can be most of a sector away.
    m.run_for(GlobalTime::from_nanos(450_000_000))
        .expect("it runs");
    assert_eq!(drive.cylinder(), cylinder, "the guest stepped the head");
    assert_eq!(
        peek(&m, INTREQR, Width::U16) & DSKBLK,
        DSKBLK,
        "DSKBLK: the read finished"
    );

    let got: Vec<u8> = (0..u64::from(READ_WORDS) * 2)
        .map(|i| peek(&m, u64::from(BUFFER) + i, Width::U8) as u8)
        .collect();
    // WORDSYNC started the transfer on the first sync word, so the buffer
    // opens on the second.
    assert_eq!(&got[..2], &[0x44, 0x89], "the read began on a sync");

    let sectors = adf::decode_track(&got, track as u8);
    let want = &image[adf::track_offset(track)..adf::track_offset(track + 1)];
    for (s, sector) in sectors.iter().enumerate() {
        let sector = sector
            .as_deref()
            .unwrap_or_else(|| panic!("sector {s} of track {track} did not decode"));
        assert_eq!(
            sector,
            &want[s * adf::SECTOR_BYTES..(s + 1) * adf::SECTOR_BYTES],
            "sector {s}"
        );
    }
}

#[test]
fn a_guest_writes_a_track_and_the_file_behind_the_drive_gets_its_sectors() {
    let image = synthetic_adf();
    let store = Arc::new(RamStore::new(adf::ADF_BYTES as u64));
    Medium::write_at(&*store, 0, &image).expect("fills");

    let (cylinder, side) = (3u8, 0u8);
    let track = usize::from(cylinder) * 2 + usize::from(side);
    let new: Vec<u8> = image[adf::track_offset(track)..adf::track_offset(track + 1)]
        .iter()
        .map(|b| b ^ 0x5a)
        .collect();
    let mfm = adf::encode_track(track as u8, &new);
    let (mut m, drive) = build(
        writer(cylinder, side, &mfm),
        Vec::new(),
        Some(Arc::clone(&store) as Arc<dyn Medium>),
    );
    assert!(drive.disk().is_some(), "the medium is the disk");

    m.run_for(GlobalTime::from_nanos(350_000_000))
        .expect("it runs");
    assert_eq!(
        drive.cylinder(),
        cylinder - 1,
        "the guest saw DSKBLK and stepped off the track"
    );

    // Stepping off handed the track back: no flush was needed for this.
    let mut back = vec![0u8; adf::ADF_BYTES];
    Medium::read_at(&*store, 0, &mut back).expect("reads");
    assert_eq!(
        &back[adf::track_offset(track)..adf::track_offset(track + 1)],
        &new[..],
        "the written track's sectors are in the file"
    );
    let mut expected = image.clone();
    expected[adf::track_offset(track)..adf::track_offset(track + 1)].copy_from_slice(&new);
    assert!(back == expected, "and nothing else in the file moved");
    m.flush().expect("nothing is owed and nothing failed");
}

#[test]
fn bytes_in_a_slot_are_a_copy_that_the_guest_writes_and_nothing_else_sees() {
    let image = synthetic_adf();
    let (cylinder, side) = (3u8, 0u8);
    let track = usize::from(cylinder) * 2 + usize::from(side);
    let new: Vec<u8> = vec![0xa5; adf::TRACK_DATA];
    let mfm = adf::encode_track(track as u8, &new);
    let (mut m, drive) = build(writer(cylinder, side, &mfm), image, None);
    m.run_for(GlobalTime::from_nanos(350_000_000))
        .expect("it runs");
    let (now, missing) = drive.disk().expect("a disk").to_adf();
    assert_eq!(
        missing, 0,
        "every sector of the session's disk still decodes"
    );
    assert_eq!(
        &now[adf::track_offset(track)..adf::track_offset(track + 1)],
        &new[..],
        "the session's disk has the write"
    );
    m.flush()
        .expect("a copy owes nothing to anyone, so a flush is clean");
}

// ---------------------------------------------------------------------------
// The user's own files, in place
// ---------------------------------------------------------------------------

#[cfg(feature = "media-kickstart")]
mod real {
    use super::*;

    /// A file in the directory `var` names, or `None` after saying why not.
    fn users_file(var: &str, name: &str) -> Option<std::path::PathBuf> {
        let Ok(dir) = std::env::var(var) else {
            println!("amiga-adf: set {var} to boot a real disk against a real Kickstart; skipped");
            return None;
        };
        let path = std::path::Path::new(&dir).join(name);
        if !path.exists() {
            println!("amiga-adf: {} is not there; skipped", path.display());
            return None;
        }
        Some(path)
    }

    /// The Kickstart and the disk this run boots, read in place.
    fn users_media() -> Option<(Vec<u8>, Vec<u8>)> {
        let rom = std::env::var("RSEMU_AMIGA_BOOT_ROM").unwrap_or("amiga-os-204.rom".into());
        let disk =
            std::env::var("RSEMU_AMIGA_BOOT_ADF").unwrap_or("amiga-os-204-workbench.adf".into());
        let rom = users_file("RSEMU_AMIGA_ROM_DIR", &rom)?;
        let disk = users_file("RSEMU_AMIGA_ADF_DIR", &disk)?;
        let kickstart = rsemu::host::media::kickstart::open(&rom.to_string_lossy())
            .unwrap_or_else(|e| panic!("{}: {e}", rom.display()));
        let bytes = std::fs::read(&disk).unwrap_or_else(|e| panic!("{}: {e}", disk.display()));
        Some((kickstart.bytes, bytes))
    }

    /// Run `kickstart` with `df0` in the drive for `seconds`, and return the
    /// furthest cylinder the head reached. Kickstart's byte reads of custom
    /// registers go through `amiga.custom` itself (`src/dev/amiga/custom.rs`).
    fn furthest(kickstart: Vec<u8>, df0: Vec<u8>, seconds: u64) -> u8 {
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        let drives: Arc<Captured<Floppy>> = Arc::new(Captured::new());
        let kept = Arc::clone(&drives);
        options.bindings.replace("amiga.floppy", move |props| {
            let drive = Arc::new(Floppy::new(props)?);
            kept.push(&drive);
            Ok(drive)
        });
        options.realize.media.insert("kickstart", kickstart);
        options.realize.media.insert("df0", df0);
        options.realize.media.insert("ext", Vec::new());
        let registry = catalog::registry().expect("a registry");
        let mut m = rsemu::machine::build("amiga-a500", board(), &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
        let drive = drives.last().expect("the drive");
        let mut furthest = 0;
        for tenth in 0..seconds * 10 {
            m.run_for(GlobalTime::from_nanos(100_000_000))
                .expect("it runs");
            furthest = furthest.max(drive.cylinder());
            if tenth % 20 == 19 {
                println!(
                    "amiga-adf: {:>3}s  head on cylinder {:>2}, motor {}, furthest {furthest}",
                    (tenth + 1) / 10,
                    drive.cylinder(),
                    if drive.motor() { "on " } else { "off" },
                );
            }
        }
        furthest
    }

    /// The root block is sector 880, which is track 80: cylinder 40, side 0.
    /// A head that gets past it was sent there by AmigaDOS, which only runs
    /// once the boot block — two sectors trackdisk had to accept — has.
    const ROOT_CYLINDER: u8 = 40;

    #[test]
    fn a_real_workbench_disk_boots_against_a_real_kickstart() {
        let Some((kickstart, disk)) = users_media() else {
            return;
        };
        let furthest = furthest(kickstart, disk, 30);
        assert!(
            furthest > ROOT_CYLINDER,
            "the head only reached cylinder {furthest}: AmigaDOS never got past the root block"
        );
    }

    #[test]
    fn a_real_kickstart_refuses_the_same_disk_with_its_checksums_wrong() {
        let Some((kickstart, disk)) = users_media() else {
            return;
        };
        // One data cell flipped in every sector's data checksum, and then — a
        // second disk — in every header checksum. Everything else is the disk
        // that boots.
        for (field, at) in [("data", 59), ("header", 51)] {
            let encoded = rsemu::dev::amiga::floppy::MfmDisk::from_adf(&disk).expect("an ADF");
            let mut raw = Vec::with_capacity(160 * 12_500);
            for t in 0..160 {
                let mut track = encoded.track(t).to_vec();
                for s in 0..adf::SECTORS {
                    track[adf::GAP_BYTES + s * adf::SECTOR_MFM_BYTES + at] ^= 0x01;
                }
                raw.extend(track);
            }
            let furthest = furthest(kickstart.clone(), raw, 20);
            assert!(
                furthest < ROOT_CYLINDER,
                "with every {field} checksum wrong the head still reached cylinder {furthest}"
            );
        }
    }
}
