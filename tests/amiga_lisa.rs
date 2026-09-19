//! Lisa — `amiga.denise` with `revision = "aga"` — painting whole fields a
//! person can look at: 256 colours of 24 bits, a HAM8 gradient, and AA
//! sprites.
//!
//! `src/dev/amiga/denise/aga/tests.rs` has the pixel-level tests. These build
//! the same kind of field at full size through only the public API — register
//! writes on the custom-chip seam and lines on [`Video::line`], the way Alice
//! will drive the chip — assert the pixels, and then capture the field through
//! the real host adapter, `host::display::amiga`, the way a window would.
//!
//! `RSEMU_AMIGA_FRAME_DIR`, when set and built with `display-png`, receives a
//! PNG of each field: `lisa-256.png`, `lisa-ham8.png` and `lisa-sprites.png`.
//! Every expected value is the *Specification for the Advanced Amiga (AA) Chip
//! Set*'s (Commodore-Amiga), cited by section. Nothing from any Kickstart,
//! and no board: there is no AA board yet.

#![cfg(feature = "dev-amiga-denise")]

use std::collections::BTreeSet;
use std::sync::Arc;

use rsemu::dev::amiga::custom::{CustomChip, Origin};
use rsemu::dev::amiga::denise::{Fetch, Line, Revision, Standard, Video};
use rsemu::dev::amiga::regs;
use rsemu::host::display::amiga::DeniseScanout;
use rsemu::host::display::{PixelFormat, Scanout, Surface};

const BPLCON0: u16 = 0x100;
const BPLCON2: u16 = 0x104;
const BPLCON3: u16 = 0x106;
const BPLCON4: u16 = 0x10c;
const DIWSTRT: u16 = 0x08e;
const DIWSTOP: u16 = 0x090;
const FMODE: u16 = 0x1fc;
const COLOR00: u16 = 0x180;
const SPR0POS: u16 = 0x140;

/// `BPLCON3`'s reset `PF2OF = 011` and its `LOCT` bit (§4, `BPLCON3`).
const PF2OF: u16 = 0b011 << 10;
const LOCT: u16 = 1 << 9;

/// The standard PAL window: 320 low-resolution pixels from `$81`, 256 lines
/// from `$2C`.
const TOP: u16 = 0x2c;
const WIDTH: usize = 320;
const HEIGHT: u16 = 256;

/// The picture's column and row for low-resolution pixel `k` right of `$81`
/// on line `vpos`: Lisa's columns are 35 ns quarters from `OUTPUT_LEFT = 64`,
/// and a non-interlaced line fills two rows from `$1D`.
fn at(k: usize, vpos: u16) -> (u32, u32) {
    (((0x81 - 64) + k as u32) * 4, 2 * u32::from(vpos - 0x1d))
}

fn w(v: &Video, offset: u16, value: u16) {
    let reg = regs::lookup(offset).expect("a declared register");
    CustomChip::write(v, reg, value, Origin::cpu());
}

/// Palette entry `n` set to a 24-bit colour: its bank, the high nibbles, then
/// the low ones with `LOCT` — "when 24 bit colors are desired load LSB after
/// MSB" (§2, *Color Lookup Table*).
fn set_rgb(v: &Video, n: usize, rgb: u32) {
    let bank = (n / 32) as u16;
    let at = COLOR00 + 2 * (n % 32) as u16;
    let nibble = |shift: u32| ((rgb >> shift) & 0xf) as u16;
    w(v, BPLCON3, bank << 13 | PF2OF);
    w(v, at, nibble(20) << 8 | nibble(12) << 4 | nibble(4));
    w(v, BPLCON3, bank << 13 | PF2OF | LOCT);
    w(v, at, nibble(16) << 8 | nibble(8) << 4 | nibble(0));
    w(v, BPLCON3, PF2OF);
}

/// Eight plane streams showing `values`, one low-resolution pixel each.
fn streams(values: &[u8]) -> [Vec<u16>; 8] {
    let mut planes: [Vec<u16>; 8] = Default::default();
    for (p, plane) in planes.iter_mut().enumerate() {
        *plane = vec![0; values.len().div_ceil(16)];
        for (k, value) in values.iter().enumerate() {
            if value >> p & 1 != 0 {
                plane[k / 16] |= 0x8000 >> (k % 16);
            }
        }
    }
    planes
}

/// One whole PAL field, `values(vpos)` on every line of the window, fetched
/// at `$38` so the first pixel lands on `$81`.
fn field(v: &Video, values: impl Fn(u16) -> Vec<u8>) {
    for vpos in 0..313 {
        let s = streams(&if (TOP..TOP + HEIGHT).contains(&vpos) {
            values(vpos)
        } else {
            Vec::new()
        });
        v.line(&Line {
            vpos,
            clocks: 227,
            fetch: Fetch {
                start: 0x38,
                planes: [&s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &s[6], &s[7]],
            },
        });
    }
    v.field(true);
}

/// The field as a host sees it, through the real adapter.
fn capture(v: &Arc<Video>) -> Surface {
    let scanout = DeniseScanout::new(Arc::clone(v), None);
    let info = scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    scanout.capture(&mut surface);
    surface
}

/// A pixel of the captured surface as `0x00RR_GGBB`.
fn rgb(s: &Surface, (x, y): (u32, u32)) -> u32 {
    let [r, g, b] = s.get(x, y).expect("inside the picture");
    u32::from(r) << 16 | u32::from(g) << 8 | u32::from(b)
}

#[cfg(feature = "display-png")]
fn dump(name: &str, surface: &Surface) {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") else {
        return;
    };
    std::fs::create_dir_all(&dir).expect("the frame directory can be made");
    let png = rsemu::host::display::png::encode(surface).expect("a PNG");
    std::fs::write(std::path::Path::new(&dir).join(format!("{name}.png")), png)
        .expect("the frame directory is writable");
}

#[cfg(not(feature = "display-png"))]
fn dump(_name: &str, _surface: &Surface) {}

fn lisa() -> Arc<Video> {
    let v = Arc::new(Video::with_revision(Standard::Pal, Revision::Aga));
    w(&v, DIWSTRT, 0x2c81);
    w(&v, DIWSTOP, 0x2cc1);
    v
}

/// The 256-colour chart's entry `n`: red across, green down, blue fading
/// toward the far corner — eight bits a gun, and low nibbles that only a
/// `LOCT` write can put there.
fn chart(n: usize) -> u32 {
    let (row, col) = ((n >> 4) as u32, (n & 15) as u32);
    (col * 16 + row) << 16 | (row * 16 + col) << 8 | (255 - (col + row) * 8)
}

#[test]
fn eight_planes_show_all_256_twenty_four_bit_colours() {
    let v = lisa();
    for n in 0..256 {
        set_rgb(&v, n, chart(n));
    }
    // BPU = 8: "Bit plane use code 0000-1000", BPU3 at bit 4 (§4, BPLCON0).
    w(&v, BPLCON0, 0x0010);
    // A 16 × 16 grid over the window: twenty pixels by sixteen lines a cell.
    let cell = |k: usize, vpos: u16| (usize::from(vpos - TOP) / 16) * 16 + k / 20;
    field(&v, |vpos| (0..WIDTH).map(|k| cell(k, vpos) as u8).collect());

    let s = capture(&v);
    assert_eq!((s.info().width, s.info().height), (1600, 568));
    for vpos in (TOP..TOP + HEIGHT).step_by(16) {
        for k in (10..WIDTH).step_by(20) {
            assert_eq!(
                rgb(&s, at(k, vpos + 8)),
                chart(cell(k, vpos)),
                "{k}, {vpos:#x}"
            );
        }
    }
    // Every entry is on screen, and so are colours a 12-bit part cannot make.
    let seen: BTreeSet<u32> = (TOP..TOP + HEIGHT)
        .flat_map(|vpos| (0..WIDTH).map(move |k| (k, vpos)))
        .map(|(k, vpos)| rgb(&s, at(k, vpos)))
        .collect();
    assert_eq!(seen.len(), 256);
    assert!(seen.iter().any(|c| c >> 16 & 0xf != c >> 20 & 0xf));
    dump("lisa-256", &s);
}

/// The HAM8 field's pixel `k` on window line `y`: red, green and blue modified
/// in turn, red from how far across, green from how far down, blue from how
/// far across again, backwards.
fn ham8_value(k: usize, y: usize) -> u8 {
    // "BP2 BP1: 01 modify blue, 10 modify red, 11 modify green" (§2).
    let (data, control) = match k % 3 {
        0 => ((k * 63 / (WIDTH - 1)) as u8, 0b10),
        1 => ((y * 63 / usize::from(HEIGHT - 1)) as u8, 0b11),
        _ => (63 - (k * 63 / (WIDTH - 1)) as u8, 0b01),
    };
    data << 2 | control
}

#[test]
fn ham8_paints_a_gradient_no_palette_could_hold() {
    let v = lisa();
    // Colour 0 black, so the hold register starts every line at black and
    // the two low bits of each gun stay 00.
    set_rgb(&v, 0, 0);
    // BPU = 8 and HAM: "invoked when BPU field in BPLCON0 is set to 8, and
    // HAMEN is set" (§2).
    w(&v, BPLCON0, 0x0810);
    field(&v, |vpos| {
        let y = usize::from(vpos - TOP);
        (0..WIDTH).map(|k| ham8_value(k, y)).collect()
    });

    let s = capture(&v);
    // After a whole R, G, B triple, the pixel is exactly the three modifies:
    // "the data is placed in 6 MSB" of each gun.
    for vpos in [TOP, TOP + 100, TOP + HEIGHT - 1] {
        let y = usize::from(vpos - TOP);
        for k in [2usize, 158, 317] {
            let gun = |k: usize| u32::from(ham8_value(k, y) >> 2) << 2;
            let want = gun(k - 2) << 16 | gun(k - 1) << 8 | gun(k);
            assert_eq!(rgb(&s, at(k, vpos)), want, "pixel {k}, line {vpos:#x}");
        }
    }
    let seen: BTreeSet<u32> = (TOP..TOP + HEIGHT)
        .flat_map(|vpos| (0..WIDTH).map(move |k| (k, vpos)))
        .map(|(k, vpos)| rgb(&s, at(k, vpos)))
        .collect();
    // More than a table of 256 can hold — and more than the 4096 colours a
    // twelve-bit part can make at all.
    println!("lisa-ham8: {} distinct colours", seen.len());
    assert!(seen.len() > 4096, "{} colours on one screen", seen.len());
    dump("lisa-ham8", &s);
}

#[test]
fn aa_sprites_are_as_wide_and_as_fine_as_fmode_and_spres_say() {
    let v = lisa();
    // A dim 256-colour background, and eight bright sprite colours at the top
    // of the table, where BPLCON4 moves them.
    for n in 0..256 {
        set_rgb(&v, n, chart(n) >> 2 & 0x003f_3f3f);
    }
    for (i, c) in [0x00ff_ffff, 0x00ff_4040, 0x0040_ff40, 0x0040_40ff]
        .into_iter()
        .enumerate()
    {
        set_rgb(&v, 0xf0 + i, c);
    }
    w(&v, BPLCON0, 0x0010);
    // PF2P = 4, which in a single playfield puts every sprite group in front:
    // the 3rd-edition priority rules, unchanged by AA.
    w(&v, BPLCON2, 4 << 3);
    // "ESPRM … OSPRM … the 4 high order color table address bits" (§4,
    // BPLCON4): both at $F, so sprite colours are $F1-$FF.
    w(&v, BPLCON4, 0x00ff);
    // "Sprites are either 16, 32, or 64 bits wide" (§5): 64, by SPAGEM and
    // SPR32 (FMODE bits 3 and 2).
    w(&v, FMODE, 0x000c);
    // Three bands of the window, the same 64-pixel sprite at 140 ns, 70 ns and
    // 35 ns — SPRES 01, 10, 11 (§4, BPLCON3) — and so 256, 128 and 64 quarters
    // wide: "35 ns sprites on a lores screen".
    let sprite = [0xffff_0000_ff00_f0f0u64, 0xf0f0_ff00_0000_ffffu64];
    for vpos in 0..313u16 {
        let band = if (TOP..TOP + HEIGHT).contains(&vpos) {
            usize::from(vpos - TOP) * 3 / usize::from(HEIGHT)
        } else {
            0
        };
        w(&v, BPLCON3, PF2OF | (band as u16 + 1) << 6);
        w(&v, SPR0POS, 0x0060);
        w(&v, SPR0POS + 2, 0);
        v.sprite_dma(0, true, sprite[1]);
        v.sprite_dma(0, false, sprite[0]);
        let values: Vec<u8> = if (TOP..TOP + HEIGHT).contains(&vpos) {
            // The chart again, kept out of bank $F0-$FF, which is the
            // sprites': its last row repeats the one above.
            let y = usize::from(vpos - TOP);
            let cell = |k: usize| (y / 16) * 16 + k / 20;
            (0..WIDTH)
                .map(|k| if cell(k) >= 0xf0 { cell(k) - 16 } else { cell(k) } as u8)
                .collect()
        } else {
            Vec::new()
        };
        let s = streams(&values);
        v.line(&Line {
            vpos,
            clocks: 227,
            fetch: Fetch {
                start: 0x38,
                planes: [&s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &s[6], &s[7]],
            },
        });
    }
    v.field(true);

    let s = capture(&v);
    // Sprite pixel 0 is DATA 1, DATB 1 → value 3 → colour $F3; pixel 16 is
    // DATA 0, DATB 1 → 2 → $F2. The sprite starts at $60 << 3 = 768 quarters.
    let colour = |q: u32, vpos: u16| {
        let y = 2 * u32::from(vpos - 0x1d);
        let [r, g, b] = s.get(q - 256, y).expect("inside");
        u32::from(r) << 16 | u32::from(g) << 8 | u32::from(b)
    };
    for (band, step) in [(0u16, 4u32), (1, 2), (2, 1)] {
        let vpos = TOP + band * HEIGHT / 3 + 10;
        assert_eq!(colour(768, vpos), 0x0040_40ff, "band {band}: pixel 0");
        assert_eq!(
            colour(768 + 16 * step, vpos),
            0x0040_ff40,
            "band {band}: pixel 16"
        );
        // The last of 64 pixels is DATA 0, DATB 1: $F2, and then the sprite is
        // done.
        assert_eq!(
            colour(768 + 63 * step, vpos),
            0x0040_ff40,
            "band {band}: pixel 63"
        );
        assert_ne!(
            colour(768 + 64 * step, vpos) >> 16,
            0x40,
            "band {band}: past the end"
        );
    }
    dump("lisa-sprites", &s);
}
