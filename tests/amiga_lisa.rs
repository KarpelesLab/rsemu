//! Lisa — `amiga.denise` with `revision = "aga"` — painting a whole field a
//! person can look at: 256 colours of 24 bits.
//!
//! `src/dev/amiga/denise/aga/tests.rs` has the pixel-level tests. These build
//! the same kind of field at full size through only the public API — register
//! writes on the custom-chip seam and lines on [`Video::line`], the way Alice
//! will drive the chip — assert the pixels, and then capture the field through
//! the real host adapter, `host::display::amiga`, the way a window would.
//!
//! `RSEMU_AMIGA_FRAME_DIR`, when set and built with `display-png`, receives a
//! PNG of the field: `lisa-256.png`.
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
const BPLCON3: u16 = 0x106;
const DIWSTRT: u16 = 0x08e;
const DIWSTOP: u16 = 0x090;
const COLOR00: u16 = 0x180;

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
