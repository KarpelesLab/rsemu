//! The whole A500 chipset at once: a 68000 program sets up a copper list and a
//! bitplane in chip RAM, Agnus runs the list and fetches the plane, Denise
//! renders what Agnus hands it, Paula takes `VERTB` from Agnus's vertical
//! blank, and CIA-A clears the overlay and counts Agnus's `vsync` on its TOD
//! input.
//!
//! `tests/amiga_a500_board.rs` proves the map and each chip's wiring one at a
//! time; `tests/agnus_board.rs` and `tests/amiga_denise_board.rs` prove Agnus
//! and Denise on boards of their own. This is the one place all five run
//! together on `machines/amiga-a500.machine`, driven by guest code rather than
//! by the test reaching into the address space.
//!
//! The ROM is hand-assembled here from the MC68000 user's manual's instruction
//! formats. No Kickstart is in this repository. Every register value is from
//! the *Amiga Hardware Reference Manual*: Appendix A for bit layouts, chapter 3
//! for the nominal PAL window (`DIWSTRT $2C81`, `DIWSTOP $2CC1`, `DDFSTRT $38`,
//! `DDFSTOP $D0`), chapter 2 for the copper list's form.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::amiga::custom::CustomBus;
use rsemu::dev::amiga::denise::{self, Video};
use rsemu::machine::{Machine, catalog};

/// `INTREQR`, Paula's, and its `VERTB` bit (Appendix A).
const INTREQR: u64 = 0xDF_F01E;
const VERTB: u16 = 1 << 5;

/// Where the program leaves its bitplane and its copper list.
const PLANE: u16 = 0x2000;
const COPPER: u16 = 0x1000;

/// The copper list, as chapter 2 writes one: set up a one-plane low-resolution
/// display in the nominal PAL window with colour 1 red on a black background,
/// then wait for line `$80` and turn the background blue.
#[rustfmt::skip]
const LIST: [u16; 26] = [
    0x0100, 0x1200, // BPLCON0: one plane, COLOR
    0x00e0, 0x0000, // BPL1PTH
    0x00e2, PLANE, // BPL1PTL
    0x0108, 0x0000, // BPL1MOD
    0x008e, 0x2c81, // DIWSTRT
    0x0090, 0x2cc1, // DIWSTOP
    0x0092, 0x0038, // DDFSTRT
    0x0094, 0x00d0, // DDFSTOP
    0x0180, 0x0000, // COLOR00: black
    0x0182, 0x0f00, // COLOR01: red
    0x8001, 0xff00, // WAIT for line $80, any horizontal position
    0x0180, 0x000f, // COLOR00: blue
    0xffff, 0xfffe, // end of list
];

/// The program. The reset PC is in the ROM's own window at `$F8000C`, so the
/// code keeps running when it takes the overlay away from address zero.
/// Kept one instruction per line and out of `rustfmt`'s reach, so each line can
/// be checked against its encoding.
#[rustfmt::skip]
fn rom() -> Vec<u8> {
    let mut code: Vec<u16> = vec![
        0x13fc, 0x0001, 0x00bf, 0xe201, // move.b #$01,$BFE201   CIA-A DDRA: PA0 out
        0x13fc, 0x0000, 0x00bf, 0xe001, // move.b #$00,$BFE001   PRA: OVL low
        0x41f8, PLANE,                  // lea    $2000.w,a0
        0x303c, 20 * 256 - 1,           // move.w #5119,d0
        0x30fc, 0xff00,                 // move.w #$FF00,(a0)+
        0x51c8, 0xfffa,                 // dbra   d0,*-4
        0x41f8, COPPER,                 // lea    $1000.w,a0
    ];
    for word in LIST {
        code.extend([0x30fc, word]);    // move.w #word,(a0)+
    }
    code.extend([
        0x23fc, 0x0000, COPPER, 0x00df, 0xf080, // move.l #$1000,$DFF080   COP1LC
        0x33fc, 0x0000, 0x00df, 0xf088,         // move.w #0,$DFF088       COPJMP1
        0x33fc, 0x8380, 0x00df, 0xf096,         // move.w #$8380,$DFF096   DMACON: DMAEN, BPLEN, COPEN
        0x60fe,                                 // bra    *
    ]);

    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x00F8_000Cu32.to_be_bytes());
    for (i, word) in code.iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

fn boot() -> Machine {
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom());
    // DF0 names a media slot; an empty one is an empty drive (the PC floppy
    // precedent, `tests/pc_at_ide.rs`). The front ends bind it for a user.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build("amiga-a500", entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
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

fn row(v: &Video, y: u32) -> Vec<u16> {
    let mut row = vec![0u16; denise::WIDTH as usize];
    v.read_row(y, &mut row);
    row
}

/// FNV-1a over every picture row, big-endian.
fn frame_hash(v: &Video) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for y in 0..denise::Standard::Pal.height() {
        for pixel in row(v, y) {
            for byte in pixel.to_be_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    hash
}

/// A picture row for line `vpos`: Denise gives each line two, from line `$1D`.
fn y(vpos: u32) -> u32 {
    2 * (vpos - 0x1d)
}

/// The picture's golden hash after 200 ms: the nominal PAL window of twenty
/// red-and-black words on every line from `$2C` to `$12B`, the background black
/// above line `$80` and blue from it, everything outside the window the
/// background colour of its line.
const GOLDEN_FRAME: u64 = 0x2056_8c38_ec40_1225;

#[test]
fn the_copper_and_the_bitplane_reach_denise_and_vertical_blank_reaches_paula() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(200_000_000))
        .expect("it runs");
    let v = video(&m);

    // Ten PAL fields of 313 lines at 3 546 895 Hz fit in 200 ms.
    assert!(v.fields() >= 9, "Agnus pushed {} fields", v.fields());

    // Above line $80 the background is black; the plane's twenty $FF00 words
    // are eight red pixels and eight black, two columns each, from x = $81.
    let red = |r: &[u16], c: u16| r.iter().filter(|&&p| p == c).count();
    let above = row(&v, y(0x40));
    assert_eq!(red(&above, 0x0f00), 320, "the plane, above the split");
    assert_eq!(red(&above, 0x000f), 0, "black background above the split");
    assert!(above[130..146].iter().all(|&p| p == 0x0f00));
    assert!(above[146..162].iter().all(|&p| p == 0x0000));

    // From line $80 the copper has made COLOR00 blue: the plane is unchanged,
    // and every background pixel inside the window is blue.
    let below = row(&v, y(0x90));
    assert_eq!(red(&below, 0x0f00), 320, "the plane, below the split");
    assert!(
        below[146..162].iter().all(|&p| p == 0x000f),
        "blue background"
    );
    assert_eq!(red(&row(&v, y(0x7f)), 0x000f), 0, "line $7F is still black");

    // Nothing on the chipset fell through the register space.
    assert_eq!(custom_bus(&m).unclaimed(), 0);

    // Agnus's vertical blank reached Paula as VERTB, read by the guest's own
    // bus at $DFF01E.
    let intreqr = m
        .space("mem")
        .expect("the memory space")
        .read(INTREQR, Width::U16, MemAttrs::DEFAULT)
        .expect("a word") as u16;
    assert_eq!(intreqr & VERTB, VERTB, "INTREQR = {intreqr:#06x}");

    // And CIA-A's time-of-day counter counted Agnus's vertical sync: one
    // rising edge per field Agnus began. TOD is read MSB first so the three
    // bytes are latched together (8520 data sheet).
    let space = m.space("mem").expect("the memory space");
    let byte = |addr: u64| {
        space
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .expect("a byte") as u32
    };
    let tod = (byte(0xBF_EA01) << 16) | (byte(0xBF_E901) << 8) | byte(0xBF_E801);
    assert_eq!(u64::from(tod), v.fields(), "a TOD count per field");

    let hash = frame_hash(&v);
    assert_eq!(hash, GOLDEN_FRAME, "the golden moved: {hash:#018x}");
}

#[test]
fn two_boots_render_the_same_frame() {
    let hash = || {
        let mut m = boot();
        m.run_for(GlobalTime::from_nanos(200_000_000))
            .expect("it runs");
        frame_hash(&video(&m))
    };
    assert_eq!(hash(), hash());
}
