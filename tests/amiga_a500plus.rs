//! The Amiga 500+: the Enhanced Chip Set on `machines/amiga-a500plus.machine`.
//!
//! # What this file proves
//!
//! **ROM-free**, with 68000 programs hand-assembled here from the MC68000
//! user's manual's instruction formats:
//!
//! * the two chips identify themselves as Appendix C says an ECS pair does —
//!   `VPOSR`'s bits 14–8 and `DENISEID` — and, on the original chip set's
//!   `amiga-a500` beside it, as an 8371 and an 8362 do;
//! * a SuperHires screen on a productivity-mode beam: a program programs
//!   `BEAMCON0`, `HTOTAL`, `VTOTAL` and the blanking registers for 114-count
//!   lines and 525-line fields, opens a two-plane SuperHires display window
//!   with `DIWHIGH`, and the host frame has the shape the programmed raster
//!   gives it, the pixels the Appendix C colour encoding gives them, and a
//!   frame period of exactly 525 × 114 colour clocks;
//! * the same program on the A500 changes nothing an original chip set does
//!   not have: the field stays 313 lines of 227 counts, the picture its 800 ×
//!   568, and `SHRES` is not a SuperHires fetch.
//!
//! **With the user's ROMs**, read in place from `RSEMU_AMIGA_ROM_DIR` and
//! `RSEMU_AMIGA_ADF_DIR` and skipped with a printed reason when those are
//! unset: Kickstart 2.04 and 3.1 reach their insert-disk screens on the 500+
//! and Workbench 2.04 boots to its desktop — and in each, `graphics.library`
//! found the ECS chips. The evidence is `GfxBase->ChipRevBits0` read out of
//! guest RAM by the *ROM Kernel Reference Manual*'s structure layouts: exec's
//! library list from `ExecBase` (the longword at 4, "the only absolute memory
//! location in the system", HRM Appendix D), each node's `ln_Name`, and
//! `gb_ChipRevBits0` at offset `$EC` of the library base, whose
//! `GFXF_HR_AGNUS` (bit 0) and `GFXF_HR_DENISE` (bit 1) Appendix C, *Determining
//! Chip Revisions*, defines. And a person at the Workbench, through the same
//! input seam a VNC client drives, opens ScreenMode preferences on both
//! boards: the 500+'s lists the SuperHires modes and a maximum size of 16368 ×
//! 16384, the ECS blitter's; the A500's 1008 × 1024. `RSEMU_AMIGA_FRAME_DIR`
//! receives a PNG of every frame checked, in a build with `display-png`.
//!
//! **No byte of any ROM or disk is in this file**; what is asserted is this
//! emulator's rendering and the guest's own bookkeeping. No Amiga emulator
//! source and no AROS source was consulted (`ROADMAP.md` §1).

#![cfg(feature = "machine-amiga-a500plus")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::dev::amiga::custom::{CustomBus, Origin};
use rsemu::dev::amiga::denise::Video;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// the boards
// ---------------------------------------------------------------------------

/// Build a shipped Amiga board from the catalog around `rom`, drive empty.
fn build(name: &str, rom: Vec<u8>, params: &[(&str, &str)]) -> Machine {
    let entry = catalog::machine(name).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom);
    options.realize.media.insert("df0", Vec::new());
    options.realize.media.insert("ext", Vec::new());
    for &(param, value) in params {
        options
            .resolve
            .params
            .push((param.to_string(), value.to_string()));
    }
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not realize: {e}"))
}

fn video(m: &Machine) -> Arc<Video> {
    let export = m
        .device("denise")
        .expect("the board has a denise")
        .device()
        .export(ExportId::AMIGA_VIDEO)
        .expect("Denise publishes its video handle");
    Arc::clone(export.opaque().expect("opaque"))
        .downcast::<Video>()
        .expect("a Video")
}

fn custom_bus(m: &Machine) -> Arc<CustomBus> {
    let export = m
        .device("custom")
        .expect("the board has a custom")
        .device()
        .export(ExportId::CUSTOM_BUS)
        .expect("published");
    Arc::clone(export.opaque().expect("opaque"))
        .downcast::<CustomBus>()
        .expect("a CustomBus")
}

fn run_ms(m: &mut Machine, ms: u64) {
    m.run_for(GlobalTime::from_nanos(ms * 1_000_000))
        .expect("it runs");
}

// ---------------------------------------------------------------------------
// a hand-assembled program
// ---------------------------------------------------------------------------

/// The custom-chip registers the programs write, by Appendix B's offsets.
mod reg {
    pub(super) const VPOSR: u16 = 0x004;
    pub(super) const DENISEID: u16 = 0x07c;
    pub(super) const COP1LC: u16 = 0x080;
    pub(super) const COPJMP1: u16 = 0x088;
    pub(super) const DIWSTRT: u16 = 0x08e;
    pub(super) const DIWSTOP: u16 = 0x090;
    pub(super) const DDFSTRT: u16 = 0x092;
    pub(super) const DDFSTOP: u16 = 0x094;
    pub(super) const DMACON: u16 = 0x096;
    pub(super) const BPLCON0: u16 = 0x100;
    pub(super) const BPLCON1: u16 = 0x102;
    pub(super) const BPLCON2: u16 = 0x104;
    pub(super) const BPL1MOD: u16 = 0x108;
    pub(super) const BPL2MOD: u16 = 0x10a;
    pub(super) const COLOR00: u16 = 0x180;
    pub(super) const HTOTAL: u16 = 0x1c0;
    pub(super) const HSSTOP: u16 = 0x1c2;
    pub(super) const HBSTRT: u16 = 0x1c4;
    pub(super) const HBSTOP: u16 = 0x1c6;
    pub(super) const VTOTAL: u16 = 0x1c8;
    pub(super) const VSSTOP: u16 = 0x1ca;
    pub(super) const VBSTRT: u16 = 0x1cc;
    pub(super) const VBSTOP: u16 = 0x1ce;
    pub(super) const BEAMCON0: u16 = 0x1dc;
    pub(super) const HSSTRT: u16 = 0x1de;
    pub(super) const VSSTRT: u16 = 0x1e0;
    pub(super) const DIWHIGH: u16 = 0x1e4;
}

/// A program as 68000 words, hand-assembled from the MC68000 user's manual.
struct Program(Vec<u16>);

impl Program {
    /// Take the overlay down: CIA-A `DDRA` bit 0 out, `PRA` bit 0 low
    /// (Appendix E, "PA0..OVL").
    fn new() -> Program {
        Program(vec![
            0x13fc, 0x0001, 0x00bf, 0xe201, // move.b #$01,$BFE201
            0x13fc, 0x0000, 0x00bf, 0xe001, // move.b #$00,$BFE001
        ])
    }

    /// `move.w #value,$DFFxxx`.
    fn custom(&mut self, offset: u16, value: u16) -> &mut Program {
        self.0.extend([0x33fc, value, 0x00df, 0xf000 | offset]);
        self
    }

    /// Fill `words` words from `at` with `value`: `lea at,a0`,
    /// `move.w #words-1,d0`, then `move.w #value,(a0)+` / `dbra d0`.
    fn fill(&mut self, at: u32, words: u16, value: u16) -> &mut Program {
        self.0.extend([
            0x41f9,
            (at >> 16) as u16,
            at as u16, // lea at,a0
            0x303c,
            words - 1, // move.w #n,d0
            0x30fc,
            value, // move.w #value,(a0)+
            0x51c8,
            0xfffa, // dbra d0,*-4
        ]);
        self
    }

    /// Store `list` at `at` a word at a time.
    fn store(&mut self, at: u32, list: &[u16]) -> &mut Program {
        self.0.extend([0x41f9, (at >> 16) as u16, at as u16]);
        for &word in list {
            self.0.extend([0x30fc, word]);
        }
        self
    }

    /// Point the copper at `at` and restart it there.
    fn copper(&mut self, at: u32) -> &mut Program {
        self.0.extend([
            0x23fc,
            (at >> 16) as u16,
            at as u16,
            0x00df,
            0xf000 | reg::COP1LC, // move.l #at,$DFF080
        ]);
        self.custom(reg::COPJMP1, 0)
    }

    /// End in `bra *` and lay the program out as a 512 KiB ROM whose reset
    /// vectors are a stack at `stack` and the code at `$F8000C`.
    fn rom(&mut self, stack: u32) -> Vec<u8> {
        self.0.push(0x60fe);
        let mut image = vec![0u8; 512 * 1024];
        image[0..4].copy_from_slice(&stack.to_be_bytes());
        image[4..8].copy_from_slice(&0x00F8_000Cu32.to_be_bytes());
        for (i, word) in self.0.iter().enumerate() {
            let at = 0x0c + 2 * i;
            image[at..at + 2].copy_from_slice(&word.to_be_bytes());
        }
        image
    }
}

// ---------------------------------------------------------------------------
// the identifications
// ---------------------------------------------------------------------------

/// `VPOSR` bits 14–8 and `DENISEID`, as the processor reads them, with a
/// known word left on the chip bus first.
fn ids(m: &Machine) -> (u16, u16) {
    let bus = custom_bus(m);
    // Something on the bus for an 8362 to float: the last word written.
    bus.write(reg::COLOR00, 0x1234, Origin::cpu());
    let deniseid = bus.read(reg::DENISEID, Origin::cpu());
    let vposr = bus.read(reg::VPOSR, Origin::cpu());
    ((vposr >> 8) & 0x7f, deniseid)
}

/// Appendix C, *Determining Chip Revisions*: an ECS Agnus's identification has
/// bit 5 set ("A value of 20 or 30 indicates that the enhanced Hires Agnus is
/// present"); the 500+'s 8375 is the 2 MiB part, `$22` on PAL. "The enhanced
/// HighRes Denise (8373) will return $FC in the lower 8 bits."
#[test]
fn the_500plus_identifies_an_ecs_agnus_and_an_8373() {
    let m = build("amiga-a500plus", Program::new().rom(0x0010_0000), &[]);
    let (agnus, denise) = ids(&m);
    assert_eq!(agnus, 0x22, "an 8375 on PAL");
    assert_eq!(denise & 0xff, 0xfc, "an 8373");
    assert_eq!(denise, 0xfffc, "the reserved byte reads as ones here");
}

/// The same reads on the original chip set: "8367 (regular PAL) or 8371 (fat
/// PAL) = 00", and "The original Denise (8362) does not have this register, so
/// whatever value is left over on the bus from the last cycle will be there".
#[cfg(feature = "machine-amiga-a500")]
#[test]
fn the_500_identifies_an_8371_and_an_8362_floats_the_bus() {
    let m = build("amiga-a500", Program::new().rom(0x0008_0000), &[]);
    let (agnus, denise) = ids(&m);
    assert_eq!(agnus, 0x00, "an 8371");
    assert_eq!(denise, 0x1234, "the last word on the bus");
}

// ---------------------------------------------------------------------------
// the battery-backed clock
// ---------------------------------------------------------------------------

/// The 500+'s MSM6242B at `$DC_0000`: register `n` a byte at `$DC_0003 + 4n`,
/// the date `-p time` gives, counted by the board's own watch crystal while
/// the processor runs.
#[test]
fn the_battery_backed_clock_answers_at_dc0000_and_keeps_time() {
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;

    let mut m = build(
        "amiga-a500plus",
        Program::new().rom(0x0010_0000),
        &[("time", "2026-09-19T12:34:56")],
    );
    let digits = |m: &Machine| -> Vec<u8> {
        let space = m.space("mem").expect("the memory space");
        (0..13)
            .map(|n| {
                space
                    .read(0xDC_0003 + 4 * n, Width::U8, MemAttrs::DEFAULT)
                    .expect("the clock answers") as u8
            })
            .collect()
    };
    // S1 S10 MI1 MI10 H1 H10 D1 D10 MO1 MO10 Y1 Y10 W — Saturday is 6.
    assert_eq!(digits(&m), [6, 5, 4, 3, 2, 1, 9, 1, 9, 0, 6, 2, 6]);
    run_ms(&mut m, 4_000);
    assert_eq!(&digits(&m)[..4], [0, 0, 5, 3], "12:35:00, four seconds on");
    // The block repeats every 64 bytes through the 64 KiB window.
    let space = m.space("mem").expect("the memory space");
    let far = space
        .read(0xDC_FFC0 + 0x2b, Width::U8, MemAttrs::DEFAULT)
        .expect("the clock answers");
    assert_eq!(far, 6, "Y1 at the top of the window");
}

// ---------------------------------------------------------------------------
// productivity mode, and SuperHires on it
// ---------------------------------------------------------------------------

/// The programmed raster. Appendix C, *Multi-Sync and Bi-Sync Monitors*: "VGA
/// (525 lines, 114.0 colorclocks per scan line)" — `HTOTAL` is the highest
/// count and `VTOTAL` the highest line, so 113 and 524.
const HTOTAL: u16 = 113;
const VTOTAL: u16 = 524;
/// Horizontal blanking ends at count 20 and starts at 110: 90 counts shown.
const HBSTOP: u16 = 20;
const HBSTRT: u16 = 110;
/// Vertical blanking ends at line 30 and starts at 510: 480 lines shown.
const VBSTOP: u16 = 30;
const VBSTRT: u16 = 510;

/// `BEAMCON0`: `HARDDIS`, `VARVBEN`, `LOLDIS`, `VARVSYEN`, `VARHSYEN`,
/// `VARBEAMEN` and `PAL` (Appendix C, *New BEAMCON0 Register*).
const BEAMCON0: u16 = 0x4000 | 0x1000 | 0x0800 | 0x0200 | 0x0100 | 0x0080 | 0x0020;

/// Two planes of 640 SuperHires pixels, 80 bytes a row, 480 rows.
const PLANE1: u32 = 0x1_0000;
const PLANE2: u32 = 0x2_0000;
const ROW_WORDS: u16 = 40;
const PLANE_WORDS: u16 = ROW_WORDS * 480;
/// Plane 1 `1010…`, plane 2 `1100…`: pixel values 3, 2, 1, 0, repeating.
const PLANE1_WORD: u16 = 0xaaaa;
const PLANE2_WORD: u16 = 0xcccc;

/// The four colours, two bits a gun, as `(r, g, b)`: value 0 dark blue, 1
/// red, 2 green, 3 white.
const COLOURS: [(u16, u16, u16); 4] = [(0, 0, 2), (3, 0, 0), (0, 3, 0), (3, 3, 3)];

/// Each colour as the picture shows it: a two-bit gun repeated into four.
fn shown(value: usize) -> u16 {
    let (r, g, b) = COLOURS[value];
    let four = |v: u16| (v << 2) | v;
    (four(r) << 8) | (four(g) << 4) | four(b)
}

/// `COLORnn` as graphics.library would write it for SuperHires, from
/// Appendix C's table: the top two bits of each gun are colour `n & 3`'s, the
/// bottom two colour `n >> 2`'s (`COLOR01` is "gh ab", colour 1's red over
/// colour 0's).
fn shr_register(n: usize) -> u16 {
    let (top, bottom) = (COLOURS[n & 3], COLOURS[n >> 2]);
    ((top.0 << 2 | bottom.0) << 8) | ((top.1 << 2 | bottom.1) << 4) | (top.2 << 2 | bottom.2)
}

/// The window: the first pixel of a SuperHires word fetched at `DDFSTRT = $18`
/// is displayed at `x = 2 × $18 + 5 = $35`, and 640 SuperHires pixels are 160
/// low-resolution ones, so `$35`–`$D5`. The stop's `H8` is 0, which only
/// `DIWHIGH` can say; so is the start's, and lines 30 to 510 need the stop's
/// `V8` from it as well.
const DIWSTRT: u16 = (VBSTOP << 8) | 0x35;
const DIWSTOP: u16 = ((VBSTRT & 0xff) << 8) | 0xd5;
const DIWHIGH: u16 = ((VBSTRT >> 8) << 8) | (VBSTOP >> 8);
/// Ten fetch blocks of four words: `$18`–`$60`.
const DDFSTRT: u16 = 0x18;
const DDFSTOP: u16 = 0x60;

fn productivity_program() -> Vec<u8> {
    let mut p = Program::new();
    p.fill(PLANE1, PLANE_WORDS, PLANE1_WORD)
        .fill(PLANE2, PLANE_WORDS, PLANE2_WORD)
        // The copper reloads both pointers every field.
        .store(
            0x1000,
            &[
                0x00e0,
                (PLANE1 >> 16) as u16,
                0x00e2,
                PLANE1 as u16,
                0x00e4,
                (PLANE2 >> 16) as u16,
                0x00e6,
                PLANE2 as u16,
                0xffff,
                0xfffe,
            ],
        );
    for n in 0..16 {
        p.custom(reg::COLOR00 + 2 * n as u16, shr_register(n));
    }
    p.custom(reg::HTOTAL, HTOTAL)
        .custom(reg::VTOTAL, VTOTAL)
        .custom(reg::HBSTRT, HBSTRT)
        .custom(reg::HBSTOP, HBSTOP)
        .custom(reg::VBSTRT, VBSTRT)
        .custom(reg::VBSTOP, VBSTOP)
        .custom(reg::HSSTRT, 8)
        .custom(reg::HSSTOP, 16)
        .custom(reg::VSSTRT, 3)
        .custom(reg::VSSTOP, 5)
        .custom(reg::BEAMCON0, BEAMCON0)
        // BPLCON0: two planes, COLOR, SHRES.
        .custom(reg::BPLCON0, 0x2000 | 0x0200 | 0x0040)
        .custom(reg::BPLCON1, 0)
        .custom(reg::BPLCON2, 0)
        .custom(reg::BPL1MOD, 0)
        .custom(reg::BPL2MOD, 0)
        .custom(reg::DDFSTRT, DDFSTRT)
        .custom(reg::DDFSTOP, DDFSTOP)
        .custom(reg::DIWSTRT, DIWSTRT)
        .custom(reg::DIWSTOP, DIWSTOP)
        // Written last: "If this register is written last in a sequence of
        // setting the display window, it sets direct start and stop positions".
        .custom(reg::DIWHIGH, DIWHIGH)
        .copper(0x1000)
        // DMACON: SET, DMAEN, BPLEN, COPEN.
        .custom(reg::DMACON, 0x8380);
    p.rom(0x0010_0000)
}

/// The frame period a host is told, from Denise's 7M clock: `2 × clocks` ticks
/// of the crystal over four, exactly as `host::display::amiga` computes it.
fn period_ns(field_clocks: u64) -> u64 {
    // 7M is the crystal over four: 28 375 160 / 4 Hz, kept as a fraction.
    let ns = u128::from(field_clocks) * 2 * 1_000_000_000 * 4 / 28_375_160;
    u64::try_from(ns).unwrap()
}

#[test]
fn a_superhires_screen_on_a_productivity_beam_has_that_beams_shape() {
    use rsemu::host::display::Scanout;
    use rsemu::host::display::amiga::DeniseScanout;

    let mut m = build("amiga-a500plus", productivity_program(), &[]);
    let v = video(&m);
    // A few fields: the planes take the processor some milliseconds to fill.
    run_ms(&mut m, 300);

    let scanout = DeniseScanout::new(Arc::clone(&v), Some((28_375_160, 4)));
    let info = scanout.info();
    // 90 counts shown, eight SuperHires pixels each; 480 lines, one row each,
    // because a 114-count line is a 31 kHz one and is not line-doubled.
    assert_eq!(
        (info.width, info.height),
        (8 * u32::from(HBSTRT - HBSTOP), u32::from(VBSTRT - VBSTOP))
    );
    let field = u64::from(VTOTAL + 1) * u64::from(HTOTAL + 1);
    assert_eq!(v.field_clocks(), field, "525 lines of 114 counts");
    assert_eq!(scanout.frame_period_ns(), period_ns(field));
    assert_eq!(
        scanout.frame_period_ns(),
        16_873_913,
        "59.26 Hz on a PAL crystal"
    );

    let mut words = Vec::new();
    let (width, _, _) = v.copy_frame(&mut words);
    let at = |col: u32, row: u32| words[(row * width + col) as usize];
    // The window's first pixel is at x = $35, counted from HBSTOP's x = 40:
    // column 4 × 13 = 52. Before it, the border, COLOR00's colour 0.
    let first = 4 * (0x35 - 2 * u32::from(HBSTOP));
    for row in [0, 239, 479] {
        assert_eq!(at(first - 1, row), shown(0), "border at row {row}");
        for i in 0..8 {
            // Values 3, 2, 1, 0, repeating, each in its own colour.
            assert_eq!(
                at(first + i, row),
                shown(3 - (i as usize % 4)),
                "row {row} +{i}"
            );
        }
        // 640 pixels on, the window stops: the last pixel is value 0 again,
        // the one before it 1, and then the border.
        assert_eq!(at(first + 638, row), shown(1));
        assert_eq!(at(first + 640, row), shown(0));
    }
    // The whole picture, once, as a golden: looked at as a PNG before it was
    // accepted — 480 rows of fine white, green, red and dark-blue stripes in
    // a 640-pixel window, a dark-blue border either side.
    #[cfg(feature = "display-png")]
    write_png("productivity-shres", &scanout);
    assert_eq!(hash_words(&words), GOLDEN_PRODUCTIVITY);
}

/// The same program on an original chip set: `BEAMCON0` and the rest are
/// registers an 8371 does not have (Appendix C's table marks every one "new"),
/// and `SHRES` a bit an 8362 does not decode. So the field is still 313 lines of
/// 227 counts, the picture still 800 × 568, and the planes are fetched and
/// shown as low resolution.
#[cfg(feature = "machine-amiga-a500")]
#[test]
fn the_same_program_on_the_original_chip_set_changes_nothing_it_lacks() {
    let mut rom = productivity_program();
    // The stack the 500's 512 KiB can hold; the planes fit below it.
    rom[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    let mut m = build("amiga-a500", rom, &[]);
    let v = video(&m);
    run_ms(&mut m, 300);
    assert_eq!(v.geometry(), (800, 568));
    assert_eq!(v.field_clocks(), 313 * 227);
    // DIWHIGH is ignored, so the old scheme's stop has H8 set: x = $1D5, and
    // DIWSTOP's V8 is the complement of V7 — line $FE | $100 = $1FE, past the
    // field. The fetch is ten low-resolution words from $18, x = 2 × $18 + 17 =
    // $41 on. So line $20's row shows plane pixels in low resolution: value 3
    // (planes 1 and 2 both set) for one low-resolution pixel, two columns.
    let mut row = vec![0u16; 800];
    v.read_row(2 * (0x20 - 0x1d), &mut row);
    let col = |x: u16| usize::from(2 * (x - 64));
    assert_eq!(row[col(0x40)], shr_register(0), "border");
    assert_eq!(
        row[col(0x41)],
        shr_register(3),
        "value 3 through COLOR03 as it is"
    );
    assert_eq!(
        row[col(0x41) + 1],
        shr_register(3),
        "a low-resolution pixel"
    );
    assert_eq!(row[col(0x42)], shr_register(2));
}

/// FNV-1a over the picture's words, low byte first.
fn hash_words(words: &[u16]) -> u64 {
    words.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, w| {
        w.to_le_bytes()
            .iter()
            .fold(h, |h, &b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
    })
}

#[cfg(feature = "display-png")]
fn write_png(name: &str, scanout: &dyn rsemu::host::display::Scanout) {
    use rsemu::host::display::{PixelFormat, Surface};
    let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") else {
        return;
    };
    let info = scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    scanout.capture(&mut surface);
    let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
    std::fs::write(std::path::Path::new(&dir).join(format!("{name}.png")), png)
        .expect("the frame directory is writable");
}

/// The productivity picture: see the test.
const GOLDEN_PRODUCTIVITY: u64 = 0x407a_3296_ea64_d325;

// ---------------------------------------------------------------------------
// real Kickstarts
// ---------------------------------------------------------------------------

#[cfg(feature = "media-kickstart")]
mod kickstart {
    use super::*;

    use rsemu::core::Captured;
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::cpu::m68k::M68k;
    use rsemu::host::display::amiga::{DeniseScanout, capture};
    use rsemu::host::display::{PixelFormat, Scanout, Surface};

    struct Board {
        machine: Machine,
        cpu: Arc<M68k>,
        scanout: DeniseScanout,
    }

    fn user_file(var: &str, file: &str, what: &str) -> Option<std::path::PathBuf> {
        let Ok(dir) = std::env::var(var) else {
            println!("amiga-a500plus: set {var} to an Amiga Forever `{what}` directory; skipped");
            return None;
        };
        let path = std::path::Path::new(&dir).join(file);
        if !path.exists() {
            println!("amiga-a500plus: {} is not there; skipped", path.display());
            return None;
        }
        Some(path)
    }

    /// The 500+ with `rom` in its socket and `adf` (if any) in DF0, both read
    /// in place; `None`, having said why, if either is not there.
    fn board(name: &str, rom: &str, adf: Option<&str>) -> Option<Board> {
        let path = user_file("RSEMU_AMIGA_ROM_DIR", rom, "Shared/rom")?;
        let image = rsemu::host::media::kickstart::open(&path.to_string_lossy())
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let disk = match adf {
            Some(file) => {
                let path = user_file("RSEMU_AMIGA_ADF_DIR", file, "Shared/adf")?;
                std::fs::read(&path).expect("the disk reads")
            }
            None => Vec::new(),
        };

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
        options.realize.media.insert("df0", disk);
        options.realize.media.insert("ext", Vec::new());
        let registry = catalog::registry().expect("a registry");
        let source = catalog::machine(name)
            .expect("this build ships the board")
            .source;
        let machine = rsemu::machine::build(name, source, &registry, &options)
            .unwrap_or_else(|e| panic!("{rom}: the board does not realize: {e}"));
        let cpu = cores.last().expect("the binding captured the processor");
        let scanout = capture::take(&options.realize.hosts, &machine).expect("a Denise");
        Some(Board {
            machine,
            cpu,
            scanout,
        })
    }

    fn advance(b: &mut Board, label: &str, seconds: u64) {
        let trace = std::env::var_os("RSEMU_AMIGA_TRACE").is_some();
        for s in 1..=seconds {
            b.machine
                .run_for(GlobalTime::from_nanos(1_000_000_000))
                .expect("it runs");
            if trace {
                let r = b.cpu.regs();
                println!(
                    "{label} {s:3}s pc={:08x} sr={:04x} faults={:?} fields={}",
                    r.pc,
                    r.sr,
                    b.cpu.bus_faults(),
                    b.scanout.frame_counter(),
                );
            }
        }
    }

    /// FNV-1a over the captured pixels.
    fn picture(b: &Board, label: &str) -> u64 {
        let info = b.scanout.info();
        let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
        b.scanout.capture(&mut surface);
        let hash = surface
            .pixels()
            .iter()
            .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
                (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
            });
        if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
            #[cfg(feature = "display-png")]
            {
                let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
                std::fs::write(std::path::Path::new(&dir).join(format!("{label}.png")), png)
                    .expect("the frame directory is writable");
            }
            #[cfg(not(feature = "display-png"))]
            let _ = dir;
        }
        println!("{label}: {}x{} frame {hash:#018x}", info.width, info.height);
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
    /// `ExecBase`; a list's `lh_Head` is its first longword; a node's
    /// `ln_Succ` is its first longword, zero on the list's tail, and `ln_Name`
    /// is at offset 10.
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

    /// `GFXF_HR_AGNUS` and `GFXF_HR_DENISE` (Appendix C, *Determining Chip
    /// Revisions*; `graphics/gfxbase.h`).
    const HR_AGNUS: u8 = 1 << 0;
    const HR_DENISE: u8 = 1 << 1;

    /// Run `rom` (with `adf` in DF0) for `seconds` and check what every run
    /// must: running, no fault, fields, ECS found — and the golden.
    fn reaches(rom: &str, adf: Option<&str>, label: &str, seconds: u64, golden: u64) {
        let Some(mut b) = board("amiga-a500plus", rom, adf) else {
            return;
        };
        advance(&mut b, label, seconds);
        let hash = picture(&b, &format!("{label}-{seconds}s"));
        assert!(!b.cpu.is_halted(), "{label}: the processor double-faulted");
        assert_eq!(b.cpu.bus_faults().0, 0, "{label}: an access faulted");
        assert!(
            b.scanout.frame_counter() >= seconds * 49,
            "{label}: Denise is producing fields"
        );
        let bits = chip_rev_bits(&b);
        println!("{label}: ChipRevBits0 = {bits:#04x}");
        assert_eq!(
            bits & (HR_AGNUS | HR_DENISE),
            HR_AGNUS | HR_DENISE,
            "{label}: graphics.library did not find the ECS chips"
        );
        assert_eq!(
            hash, golden,
            "{label}: the frame at {seconds}s moved; look at it (RSEMU_AMIGA_FRAME_DIR) before \
             accepting the new hash"
        );
    }

    /// The control: the same ROM on the A500, whose 8371 is not an ECS
    /// Agnus, and graphics.library does not find one.
    ///
    /// **A finding, pinned rather than hidden.** It does set `GFXF_HR_DENISE`
    /// there. Watched black-box, Kickstart 2.04 reads `DENISEID` seventeen
    /// times and an 8362 answers with the floating chip bus, as Appendix C
    /// says — but this model's floating word is `amiga.custom`'s documented
    /// placeholder, the last word written, and it is `$8001` all seventeen
    /// times: a stable answer, which is what an 8373 gives. The real bus holds
    /// "whatever value is left over on the bus from the last cycle", which DMA
    /// keeps changing. So the A500's ScreenMode lists SuperHires where a real
    /// A500 would not (`screenmode` below). Replacing the placeholder with the
    /// last DMA cycle's word is `custom.rs`'s to do, and would have to be
    /// shown to leave the A500's goldens where they are.
    #[cfg(feature = "machine-amiga-a500")]
    #[test]
    fn on_the_a500_kickstart_2_04_finds_no_ecs_agnus() {
        let Some(mut b) = board("amiga-a500", "amiga-os-204.rom", None) else {
            return;
        };
        advance(&mut b, "a500-204", 28);
        let bits = chip_rev_bits(&b);
        println!("a500-204: ChipRevBits0 = {bits:#04x}");
        assert_eq!(bits & HR_AGNUS, 0, "an 8371 is not an ECS Agnus");
        assert_eq!(
            bits & HR_DENISE,
            HR_DENISE,
            "the floating-bus placeholder answers DENISEID stably; see above"
        );
    }

    /// Kickstart 2.04 (37.175), the 500+'s own ROM: its insert-disk screen.
    #[test]
    fn kickstart_2_04_finds_the_ecs_chips_and_draws_its_insert_disk_screen() {
        reaches("amiga-os-204.rom", None, "a500plus-204", 28, GOLDEN_204);
    }

    /// Kickstart 3.1 (40.063, the A500/A600/A2000 part) on the 500+.
    #[test]
    fn kickstart_3_1_finds_the_ecs_chips_and_draws_its_insert_disk_screen() {
        reaches(
            "amiga-os-310-a600.rom",
            None,
            "a500plus-310",
            12,
            GOLDEN_310,
        );
    }

    /// Kickstart 2.04 and the Workbench 2.04 disk: the desktop.
    #[test]
    fn workbench_2_04_boots_to_its_desktop_on_the_500plus() {
        reaches(
            "amiga-os-204.rom",
            Some("amiga-os-204-workbench.adf"),
            "a500plus-wb204",
            45,
            GOLDEN_WB204,
        );
    }

    // The three pictures are **bit for bit the A500's**, the same hashes as
    // `tests/amiga_a500_kickstart.rs`'s `GOLDEN_204`, `GOLDEN_310` and
    // `GOLDEN_204_WORKBENCH`: with the ECS chips found, graphics.library
    // writes `BEAMCON0 = $0020` (PAL, nothing variable) and loads `DIWHIGH`
    // in every field's copper list, and the explicit window it sets that way
    // is the window the original scheme gave. Each was looked at:
    //
    // * 2.04 at 28 s: the dark purple screen, the rainbow check mark, "2.0
    //   Roms (37.175) / Copyright © 1985-1991 / Commodore-Amiga, Inc. / All
    //   Rights Reserved" in salmon, the salmon drive and the blue disk below
    //   it mid-animation.
    // * 3.1 at 12 s: the same with "3.1 ROM 40.063 / Copyright © 1985-1993".
    // * 2.04 with Workbench 2.04 at 45 s: the grey 640-pixel desktop, the
    //   copyright in the screen's title bar with the red pointer over its first
    //   letters, and the blue-titled "Workbench" window holding the Ram Disk
    //   and Workbench2.0 icons.
    const GOLDEN_204: u64 = 0xcfa0_61a4_23d3_703d;
    const GOLDEN_310: u64 = 0x26cc_b705_6fbc_7ba9;
    const GOLDEN_WB204: u64 = 0x95a5_9a12_c942_e139;
}

// ---------------------------------------------------------------------------
// a person at the 500+'s Workbench, asking for its screen modes
// ---------------------------------------------------------------------------

/// Workbench 2.04's ScreenMode preferences on the 500+ and on the A500: the
/// list of modes graphics.library offers, which is the other half of the
/// evidence that it found the ECS chips.
///
/// Every input goes in where a VNC client's does, through the recorder's
/// frontend channel into the keyboard and mouse sinks, as
/// `tests/amiga_a500_workbench.rs` drives the A500; nothing writes guest
/// memory. The positions are framebuffer pixels on the 800 × 568 picture both
/// boards draw for this screen.
#[cfg(feature = "media-kickstart")]
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

    fn user_file(var: &str, name: &str) -> Option<std::path::PathBuf> {
        let Ok(dir) = std::env::var(var) else {
            println!("amiga-a500plus: set {var} to run this; skipped");
            return None;
        };
        let path = std::path::Path::new(&dir).join(name);
        if !path.exists() {
            println!("amiga-a500plus: {} is not there; skipped", path.display());
            return None;
        }
        Some(path)
    }

    fn desk(board: &str, tag: &'static str) -> Option<Desk> {
        let rom = user_file("RSEMU_AMIGA_ROM_DIR", "amiga-os-204.rom")?;
        let adf = user_file("RSEMU_AMIGA_ADF_DIR", "amiga-os-204-workbench.adf")?;
        let image = rsemu::host::media::kickstart::open(&rom.to_string_lossy())
            .unwrap_or_else(|e| panic!("{}: {e}", rom.display()));
        let disk = std::fs::read(&adf).expect("the disk reads");

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
        options.realize.media.insert("df0", disk);
        options.realize.media.insert("ext", Vec::new());
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
        let scanout = capture::take(hosts, &machine).expect("a Denise");
        Some(Desk {
            machine,
            cpu,
            scanout,
            recorder,
            channel,
            tag,
        })
    }

    /// Three PAL fields, as the A500 Workbench test holds a button.
    const HOLD_MS: u64 = 60;
    /// The screen's top-left pixel in the framebuffer.
    const SCREEN_LEFT: u32 = 130;
    const SCREEN_TOP: u32 = 30;
    /// The Workbench2.0 disk icon, the Prefs drawer in its window, and the
    /// ScreenMode icon in the Prefs window.
    const DISK_ICON: (u32, u32) = (188, 178);
    const PREFS_DRAWER: (u32, u32) = (252, 190);
    const SCREENMODE_ICON: (u32, u32) = (415, 262);

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
            let hash = surface
                .pixels()
                .iter()
                .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
                    (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
                });
            if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
                #[cfg(feature = "display-png")]
                {
                    let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
                    std::fs::write(
                        std::path::Path::new(&dir).join(format!("{}-{name}.png", self.tag)),
                        png,
                    )
                    .expect("the frame directory is writable");
                }
                #[cfg(not(feature = "display-png"))]
                let _ = dir;
            }
            println!("{} {name}: frame {hash:#018x}", self.tag);
            hash
        }
    }

    /// Open the disk, the Prefs drawer and ScreenMode, and hold each picture
    /// to its golden.
    fn session(board: &str, tag: &'static str, goldens: [u64; 3]) {
        let Some(mut d) = desk(board, tag) else {
            return;
        };
        d.run_ms(45_000);
        d.home();
        d.double_click(DISK_ICON);
        d.run_ms(5_000);
        let window = d.look("1-disk-window");
        d.double_click(PREFS_DRAWER);
        d.run_ms(15_000);
        let prefs = d.look("2-prefs");
        d.double_click(SCREENMODE_ICON);
        d.run_ms(10_000);
        let modes = d.look("3-screenmode");
        assert!(!d.cpu.is_halted(), "{tag}: the processor double-faulted");
        assert_eq!(d.cpu.bus_faults().0, 0, "{tag}: an access faulted");
        assert_eq!(
            [window, prefs, modes],
            goldens,
            "{tag}: a picture moved; look at them (RSEMU_AMIGA_FRAME_DIR) before accepting \
             the new hashes"
        );
    }

    /// The 500+. Looked at:
    ///
    /// 1. The Workbench2.0 window open over the desktop, the screen title
    ///    reading "Amiga Workbench 811288 graphics mem" — Exec found the whole
    ///    megabyte of chip RAM.
    /// 2. The Prefs drawer: Input, Palette, Font, ScreenMode, Printer, Serial,
    ///    IControl, WBPattern, Pointer, Overscan, PrinterGfx, Time and the
    ///    Presets drawer.
    /// 3. "ScreenMode Preferences": the display modes PAL:Hires,
    ///    PAL:SuperHires, PAL:Hires-Interlaced and PAL:SuperHires-Interlaced;
    ///    PAL:Hires selected, "Visible Size 640 x 256", "Min Size 640 x 200",
    ///    **"Max Size 16368 x 16384"** — the big blits: "Support for big
    ///    blits (up to 32k x 32k) is provided for all graphics functions if
    ///    the ECS Agnus is present" (Appendix C) — and "Max Colors 16".
    #[test]
    fn screenmode_on_the_500plus_offers_superhires_and_the_ecs_blitters_sizes() {
        session(
            "amiga-a500plus",
            "a500plus-screenmode",
            [
                0x46b5_110a_3fee_cf49,
                0x154e_5ec9_9621_eb55,
                0xca31_c7c2_6fe3_3f89,
            ],
        );
    }

    /// The same on the A500. Its first picture is
    /// `tests/amiga_a500_workbench.rs`'s disk window, bit for bit, with
    /// "287248 graphics mem"; its ScreenMode window differs from the 500+'s
    /// in one line, **"Max Size 1008 x 1024"**, the original blitter's
    /// limit. It lists the SuperHires modes too, which a real A500 would not:
    /// see `on_the_a500_kickstart_2_04_finds_no_ecs_agnus` for why.
    #[cfg(feature = "machine-amiga-a500")]
    #[test]
    fn screenmode_on_the_500_keeps_the_original_blitters_sizes() {
        session(
            "amiga-a500",
            "a500-screenmode",
            [
                0xb068_7a77_ca8e_2cad,
                0x5726_f7b6_aa8a_56b9,
                0x68b5_1ad7_1fc2_a745,
            ],
        );
    }
}
