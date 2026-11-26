//! The picture: one whole line at a time.
//!
//! Sources: the SNESdev wiki's *Backgrounds*, *Tiles*, *Tilemaps*, *Sprites*,
//! *Offset-per-tile*, *Mode 7*, *Windows*, *Color math* and *PPU registers*
//! pages, and nocash fullsnes's "SNES PPU" chapters for the same ground. Each
//! function below names what it relies on.
//!
//! A line is built as five layer buffers (BG1-4 and OBJ) of 512 entries —
//! the hires width, with a low-resolution layer stored doubled — then each of
//! the 256 low-resolution positions picks a main-screen and a sub-screen pixel
//! by the mode's priority table, and colour math combines them. In a hires
//! line (modes 5 and 6, or `SETINI` pseudo-hires) the even output column is
//! the sub screen and the odd column the main screen.

use super::engine::{Engine, FB_WIDTH};

/// Marks a transparent entry in a layer buffer's priority array.
const CLEAR: u8 = 0xFF;
/// Layer number of the sprites.
const OBJ: u8 = 4;
/// Layer number of the backdrop: what a screen shows where nothing is opaque.
const BACKDROP: u8 = 5;

/// Width of a layer buffer.
const LW: usize = 512;

/// One line's worth of every layer.
struct Layers {
    color: [[u16; LW]; 5],
    prio: [[u8; LW]; 5],
    /// Whether the sprite pixel came from palettes 4-7, the only ones colour
    /// math applies to.
    obj_math: [bool; LW],
}

/// Front-to-back layer order per mode: `(layer, priority)`.
///
/// The SNESdev wiki's *Backgrounds* page, "Layer priority", which agrees with
/// fullsnes's BGMODE table. `BG3h` first in mode 1 is the `BGMODE` bit-3 case.
fn priority_table(mode: u8, bg3_high: bool, extbg: bool) -> &'static [(u8, u8)] {
    const M0: &[(u8, u8)] = &[
        (4, 3),
        (0, 1),
        (1, 1),
        (4, 2),
        (0, 0),
        (1, 0),
        (4, 1),
        (2, 1),
        (3, 1),
        (4, 0),
        (2, 0),
        (3, 0),
    ];
    const M1: &[(u8, u8)] = &[
        (4, 3),
        (0, 1),
        (1, 1),
        (4, 2),
        (0, 0),
        (1, 0),
        (4, 1),
        (2, 1),
        (4, 0),
        (2, 0),
    ];
    const M1_BG3: &[(u8, u8)] = &[
        (2, 1),
        (4, 3),
        (0, 1),
        (1, 1),
        (4, 2),
        (0, 0),
        (1, 0),
        (4, 1),
        (4, 0),
        (2, 0),
    ];
    const M2_5: &[(u8, u8)] = &[
        (4, 3),
        (0, 1),
        (4, 2),
        (1, 1),
        (4, 1),
        (0, 0),
        (4, 0),
        (1, 0),
    ];
    const M6: &[(u8, u8)] = &[(4, 3), (0, 1), (4, 2), (4, 1), (0, 0), (4, 0)];
    const M7: &[(u8, u8)] = &[(4, 3), (4, 2), (4, 1), (0, 0), (4, 0)];
    const M7_EXT: &[(u8, u8)] = &[(4, 3), (4, 2), (1, 1), (4, 1), (0, 0), (4, 0), (1, 0)];
    match mode {
        0 => M0,
        1 if bg3_high => M1_BG3,
        1 => M1,
        2..=5 => M2_5,
        6 => M6,
        _ if extbg => M7_EXT,
        _ => M7,
    }
}

/// Bits per pixel of each BG in modes 0-6; 0 means the layer does not exist.
const fn bpp(mode: u8, bg: usize) -> u32 {
    let table: [[u32; 4]; 7] = [
        [2, 2, 2, 2],
        [4, 4, 2, 0],
        [4, 4, 0, 0],
        [8, 4, 0, 0],
        [8, 2, 0, 0],
        [4, 2, 0, 0],
        [4, 0, 0, 0],
    ];
    table[mode as usize][bg]
}

/// RGB555 from its three channels.
const fn rgb(r: u16, g: u16, b: u16) -> u16 {
    r | (g << 5) | (b << 10)
}

/// Direct colour: an 8-bit pixel `bbgggrrr` plus the tile's palette bits,
/// straight to RGB555 without CGRAM (fullsnes, CGWSEL bit 0; the wiki's
/// *Direct color* section).
const fn direct_color(pixel: u16, pal: u16) -> u16 {
    let r = ((pixel & 7) << 2) | ((pal & 1) << 1);
    let g = (((pixel >> 3) & 7) << 2) | (pal & 2);
    let b = (((pixel >> 6) & 3) << 3) | (pal & 4);
    rgb(r, g, b)
}

/// `INIDISP` brightness: each channel scaled by `b / 15`, so 15 is the colour
/// as stored and 0 is black.
fn brighten(c: u16, b: u16) -> u16 {
    if b >= 15 {
        return c;
    }
    let ch = |s: u16| ((c >> s) & 31) * b / 15;
    rgb(ch(0), ch(5), ch(10))
}

/// Colour math on two RGB555 values (the wiki's *Color math*).
fn blend(m: u16, s: u16, subtract: bool, half: bool) -> u16 {
    let ch = |shift: u16| {
        let a = (m >> shift) & 31;
        let b = (s >> shift) & 31;
        let v = if subtract { a.saturating_sub(b) } else { a + b };
        let v = if half { v >> 1 } else { v };
        v.min(31)
    };
    rgb(ch(0), ch(5), ch(10))
}

/// A window's selection nibble and logic, evaluated at every low-resolution
/// position (the wiki's *Windows*: inside is `left <= x <= right`, an empty
/// window when `left > right`; a disabled window does not mask; two enabled
/// windows combine by OR, AND, XOR or XNOR).
fn window_line(sel: u8, logic: u8, wh: [u8; 4]) -> [bool; 256] {
    let mut out = [false; 256];
    let w1_on = sel & 0x02 != 0;
    let w2_on = sel & 0x08 != 0;
    if !w1_on && !w2_on {
        return out;
    }
    for (x, o) in out.iter_mut().enumerate() {
        let x = x as u8;
        let w1 = (wh[0] <= x && x <= wh[1]) ^ (sel & 0x01 != 0);
        let w2 = (wh[2] <= x && x <= wh[3]) ^ (sel & 0x04 != 0);
        *o = match (w1_on, w2_on) {
            (true, false) => w1,
            (false, true) => w2,
            _ => match logic & 3 {
                0 => w1 || w2,
                1 => w1 && w2,
                2 => w1 ^ w2,
                _ => !(w1 ^ w2),
            },
        };
    }
    out
}

/// Sprite sizes per `OBSEL` bits 5-7, `(small, large)` as `(width, height)`.
/// Sizes 6 and 7 are the undocumented rectangular ones fullsnes lists.
const OBJ_SIZES: [[(u16, u16); 2]; 8] = [
    [(8, 8), (16, 16)],
    [(8, 8), (32, 32)],
    [(8, 8), (64, 64)],
    [(16, 16), (32, 32)],
    [(16, 16), (64, 64)],
    [(32, 32), (64, 64)],
    [(16, 32), (32, 64)],
    [(16, 32), (32, 32)],
];

impl Engine {
    fn vram_at(&self, word: u32) -> u16 {
        self.vram[(word & 0x7FFF) as usize]
    }

    /// One pixel of a planar tile row: `bpp` bits gathered from bitplane pairs
    /// eight words apart, bit `7 - x` of each (the wiki's *Tiles*).
    fn planar(&self, row: u32, bpp: u32, x: u32) -> u16 {
        let bit = 7 - (x & 7);
        let mut out = 0u16;
        for pair in 0..bpp / 2 {
            let w = self.vram_at(row + 8 * pair);
            out |= ((w >> bit) & 1) << (2 * pair);
            out |= ((w >> (8 + bit)) & 1) << (2 * pair + 1);
        }
        out
    }

    /// The mosaic block's first line for line `v`: blocks start at V = 1.
    fn mosaic_v(&self, v: u16) -> u16 {
        let size = u16::from(self.r.mosaic >> 4) + 1;
        v - (v - 1) % size
    }

    /// Render line `v` (1-based; the framebuffer row is `v - 1`).
    pub(super) fn render_line(&mut self, v: u16) {
        let row = usize::from(v - 1);
        if self.forced_blank() {
            self.store_line(row, &[0; FB_WIDTH]);
            return;
        }
        let mode = self.r.bgmode & 7;
        let hires_bg = mode == 5 || mode == 6;
        let hires = hires_bg || self.r.setini & 0x08 != 0;
        self.hires_seen |= hires;
        let extbg = self.r.setini & 0x40 != 0;

        let mut layers = Layers {
            color: [[0; LW]; 5],
            prio: [[CLEAR; LW]; 5],
            obj_math: [false; LW],
        };
        let shown = self.r.tm | self.r.ts;
        if mode == 7 {
            for bg in 0..2 {
                if shown & (1 << bg) != 0 && (bg == 0 || extbg) {
                    self.render_mode7(bg, v, &mut layers);
                }
            }
        } else {
            for bg in 0..4 {
                if bpp(mode, bg) != 0 && shown & (1 << bg) != 0 {
                    self.render_bg(bg, mode, v, &mut layers);
                }
            }
        }
        // Sprite evaluation runs whether or not OBJ is on a screen: the
        // overflow flags are set by evaluation, not by display.
        self.render_obj(v, &mut layers);

        let windows: [[bool; 256]; 6] = core::array::from_fn(|i| {
            let sel = (self.r.wsel[i / 2] >> (4 * (i % 2))) & 0x0F;
            let logic = if i < 4 {
                self.r.wbglog >> (2 * i)
            } else {
                self.r.wobjlog >> (2 * (i - 4))
            };
            window_line(sel, logic, self.r.wh)
        });

        let table = priority_table(mode, self.r.bgmode & 0x08 != 0, extbg);
        let backdrop = self.cgram[0] & 0x7FFF;
        let fixed = rgb(
            u16::from(self.r.fixed[0]),
            u16::from(self.r.fixed[1]),
            u16::from(self.r.fixed[2]),
        );
        let pick = |designate: u8, masked: u8, idx: usize, x: usize| -> (u16, u8, bool) {
            for &(layer, prio) in table {
                let bit = 1u8 << layer;
                if designate & bit == 0 || (masked & bit != 0 && windows[usize::from(layer)][x]) {
                    continue;
                }
                if layers.prio[usize::from(layer)][idx] == prio {
                    let math = layer != OBJ || layers.obj_math[idx];
                    return (layers.color[usize::from(layer)][idx], layer, math);
                }
            }
            (backdrop, BACKDROP, true)
        };

        let bright = u16::from(self.r.inidisp & 0x0F);
        let cgwsel = self.r.cgwsel;
        let cgadsub = self.r.cgadsub;
        let mut out = [0u16; FB_WIDTH];
        for x in 0..256 {
            let (main, main_layer, main_math) = pick(self.r.tm, self.r.tmw, 2 * x + 1, x);
            let (sub, sub_layer, _) = pick(self.r.ts, self.r.tsw, 2 * x, x);
            let sub_backdrop = sub_layer == BACKDROP;
            let in_cw = windows[5][x];
            // CGWSEL bits 6-7: where the main screen is forced black;
            // bits 4-5: where colour math is prevented.
            let clip = match cgwsel >> 6 {
                0 => false,
                1 => !in_cw,
                2 => in_cw,
                _ => true,
            };
            let allowed = match (cgwsel >> 4) & 3 {
                0 => true,
                1 => in_cw,
                2 => !in_cw,
                _ => false,
            };
            let mut color = if clip { 0 } else { main };
            if allowed && main_math && cgadsub & (1 << main_layer) != 0 {
                let use_sub = cgwsel & 0x02 != 0;
                let operand = if use_sub && !sub_backdrop { sub } else { fixed };
                // Halving is skipped when the main pixel was clipped to black
                // and when the operand is the sub screen's backdrop, which is
                // the fixed colour (the wiki's *Color math*).
                let half = cgadsub & 0x40 != 0 && !clip && !(use_sub && sub_backdrop);
                color = blend(color, operand, cgadsub & 0x80 != 0, half);
            }
            let color = brighten(color, bright);
            if hires {
                // The even half-dot shows the sub screen as it stands, with
                // the main backdrop where it has nothing.
                let even = if sub_backdrop { backdrop } else { sub };
                out[2 * x] = brighten(even, bright);
            } else {
                out[2 * x] = color;
            }
            out[2 * x + 1] = color;
        }
        self.store_line(row, &out);
    }

    /// Put a finished line in the framebuffer: on row `2y + field` of an
    /// interlaced frame, on both rows of the pair otherwise.
    fn store_line(&mut self, row: usize, line: &[u16; FB_WIDTH]) {
        if self.interlace_frame {
            let r = 2 * row + usize::from(self.field);
            self.fb[r * FB_WIDTH..(r + 1) * FB_WIDTH].copy_from_slice(line);
        } else {
            for r in [2 * row, 2 * row + 1] {
                self.fb[r * FB_WIDTH..(r + 1) * FB_WIDTH].copy_from_slice(line);
            }
        }
    }

    /// The tilemap entry for tile `(tx, ty)` of BG `bg`'s map: 32×32 screens,
    /// laid out by `BGnSC`'s size bits (the wiki's *Tilemaps*).
    fn map_entry(&self, bg: usize, tx: u32, ty: u32) -> u16 {
        let sc = u32::from(self.r.bgsc[bg]);
        let base = (sc & 0xFC) << 8;
        let wide = sc & 1 != 0;
        let tall = sc & 2 != 0;
        let tx = tx & if wide { 63 } else { 31 };
        let ty = ty & if tall { 63 } else { 31 };
        let screen = match (wide, tall) {
            (false, false) => 0,
            (true, false) => tx / 32,
            (false, true) => ty / 32,
            (true, true) => tx / 32 + 2 * (ty / 32),
        };
        self.vram_at(base + screen * 0x400 + (ty % 32) * 32 + tx % 32)
    }

    /// The offset-per-tile replacement for a BG column (modes 2, 4 and 6;
    /// the wiki's *Offset-per-tile*). `column` is the BG's on-screen tile
    /// column; the leftmost one is never affected.
    fn opt_offsets(&self, bg: usize, mode: u8, column: u32, hofs: u16, vofs: u16) -> (u16, u16) {
        if column == 0 {
            return (hofs, vofs);
        }
        let tx = (column - 1) + (u32::from(self.r.hofs[2]) >> 3);
        let ty = u32::from(self.r.vofs[2]) >> 3;
        let enable = 0x2000u16 << bg;
        let (mut h, mut v) = (hofs, vofs);
        let first = self.map_entry(2, tx, ty);
        if mode == 4 {
            if first & enable != 0 {
                if first & 0x8000 != 0 {
                    v = first & 0x3FF;
                } else {
                    h = (first & 0x3F8) | (hofs & 7);
                }
            }
        } else {
            let second = self.map_entry(2, tx, ty + 1);
            if first & enable != 0 {
                h = (first & 0x3F8) | (hofs & 7);
            }
            if second & enable != 0 {
                v = second & 0x3FF;
            }
        }
        (h, v)
    }

    /// One tiled background, modes 0-6.
    fn render_bg(&self, bg: usize, mode: u8, v: u16, layers: &mut Layers) {
        let depth = bpp(mode, bg);
        let hires_bg = mode == 5 || mode == 6;
        let big = self.r.bgmode & (0x10 << bg) != 0;
        // Modes 5 and 6 always fetch 16-pixel-wide tiles (fullsnes, BGMODE).
        let tw: u32 = if hires_bg || big { 16 } else { 8 };
        let th: u32 = if big { 16 } else { 8 };
        let nba = (self.r.bgnba[bg / 2] >> (4 * (bg % 2))) & 0x0F;
        let char_base = u32::from(nba) << 12;
        let mosaic = self.r.mosaic & (1 << bg) != 0;
        let msize = u32::from(self.r.mosaic >> 4) + 1;
        let mut line = u32::from(if mosaic { self.mosaic_v(v) } else { v });
        if hires_bg && self.interlace_frame {
            line = line * 2 + u32::from(self.field);
        }
        let opt = matches!(mode, 2 | 4 | 6) && bg < 2;
        let palette_base: u16 = if mode == 0 { 32 * bg as u16 } else { 0 };
        let direct = depth == 8 && self.r.cgwsel & 0x01 != 0;
        let count = if hires_bg { 512 } else { 256 };

        for out_x in 0..count {
            let lo_x = if hires_bg { out_x / 2 } else { out_x } as u32;
            let mx = if mosaic { lo_x - lo_x % msize } else { lo_x };
            let (mut hofs, mut vofs) = (self.r.hofs[bg], self.r.vofs[bg]);
            if opt {
                let column = (mx + u32::from(hofs & 7)) >> 3;
                (hofs, vofs) = self.opt_offsets(bg, mode, column, hofs, vofs);
            }
            let px = if hires_bg {
                let sub = if mosaic { 0 } else { out_x as u32 & 1 };
                2 * (mx + u32::from(hofs)) + sub
            } else {
                mx + u32::from(hofs)
            };
            let py = line + u32::from(vofs);
            let entry = self.map_entry(bg, px / tw, py / th);
            let mut fx = px % tw;
            let mut fy = py % th;
            if entry & 0x4000 != 0 {
                fx = tw - 1 - fx;
            }
            if entry & 0x8000 != 0 {
                fy = th - 1 - fy;
            }
            let tile = (u32::from(entry & 0x3FF) + fx / 8 + (fy / 8) * 16) & 0x3FF;
            let row = char_base + tile * depth * 4 + (fy & 7);
            let pixel = self.planar(row, depth, fx);
            if pixel == 0 {
                continue;
            }
            let pal = (entry >> 10) & 7;
            let color = if direct {
                direct_color(pixel, pal)
            } else {
                let index = palette_base + if depth == 8 { 0 } else { pal << depth } + pixel;
                self.cgram[usize::from(index & 0xFF)] & 0x7FFF
            };
            let prio = ((entry >> 13) & 1) as u8;
            let slots = if hires_bg {
                out_x..out_x + 1
            } else {
                2 * out_x..2 * out_x + 2
            };
            for i in slots {
                layers.color[bg][i] = color;
                layers.prio[bg][i] = prio;
            }
        }
    }

    /// Mode 7's BG1, or EXTBG's BG2 (the wiki's *Mode 7*: the matrix, the
    /// `& !63` truncation of the origin products, `M7SEL`'s screen-over
    /// modes, and EXTBG's bit 7 as priority).
    fn render_mode7(&self, bg: usize, v: u16, layers: &mut Layers) {
        let a = i32::from(self.r.m7[0] as i16);
        let b = i32::from(self.r.m7[1] as i16);
        let c = i32::from(self.r.m7[2] as i16);
        let d = i32::from(self.r.m7[3] as i16);
        let cx = super::regs::sext13(self.r.m7x);
        let cy = super::regs::sext13(self.r.m7y);
        let hofs = super::regs::sext13(self.r.m7hofs);
        let vofs = super::regs::sext13(self.r.m7vofs);
        // The origin is clipped to ten bits signed.
        let clip = |n: i32| {
            if n & 0x2000 != 0 {
                n | !0x3FF
            } else {
                n & 0x3FF
            }
        };
        let mosaic = self.r.mosaic & (1 << bg) != 0;
        let msize = i32::from(self.r.mosaic >> 4) + 1;
        let mut y = i32::from(if mosaic { self.mosaic_v(v) } else { v });
        if self.r.m7sel & 0x02 != 0 {
            y = 255 - y;
        }
        let ox = clip(hofs - cx);
        let oy = clip(vofs - cy);
        let vx = ((a * ox) & !63) + ((b * oy) & !63) + b * y + (cx << 8);
        let vy = ((c * ox) & !63) + ((d * oy) & !63) + d * y + (cy << 8);
        let over = self.r.m7sel >> 6;
        let direct = self.r.cgwsel & 0x01 != 0;
        for sx in 0..256i32 {
            let mut x = if mosaic { sx - sx % msize } else { sx };
            if self.r.m7sel & 0x01 != 0 {
                x = 255 - x;
            }
            let px = (vx + a * x) >> 8;
            let py = (vy + c * x) >> 8;
            let outside = (px | py) & !0x3FF != 0;
            let tile = if outside && over == 2 {
                continue;
            } else if outside && over == 3 {
                0
            } else {
                let (tx, ty) = ((px & 0x3FF) >> 3, (py & 0x3FF) >> 3);
                self.vram_at((ty * 128 + tx) as u32) & 0xFF
            };
            let word = u32::from(tile) * 64 + ((py & 7) * 8 + (px & 7)) as u32;
            let pixel = self.vram_at(word) >> 8;
            let (color, prio) = if bg == 0 {
                if pixel == 0 {
                    continue;
                }
                let color = if direct {
                    direct_color(pixel, 0)
                } else {
                    self.cgram[usize::from(pixel)] & 0x7FFF
                };
                (color, 0)
            } else {
                let index = pixel & 0x7F;
                if index == 0 {
                    continue;
                }
                (self.cgram[usize::from(index)] & 0x7FFF, (pixel >> 7) as u8)
            };
            let i = 2 * sx as usize;
            for slot in [i, i + 1] {
                layers.color[bg][slot] = color;
                layers.prio[bg][slot] = prio;
            }
        }
    }

    /// Sprites: range evaluation, tile fetch, and the per-line limits (the
    /// wiki's *Sprites*; fullsnes, "SNES PPU Sprites (OBJs)" and STAT77).
    ///
    /// * Evaluation walks all 128 entries from the first — sprite 0, or with
    ///   `OAMADDH` bit 7 the sprite `(OAMADD >> 1) & 127` — and keeps the
    ///   first 32 in range; a 33rd sets *range over*. A sprite at X = −256 is
    ///   counted in range although none of it can show.
    /// * Tiles are then fetched for the kept sprites **in reverse order**, one
    ///   per eight on-screen pixels; a 35th sets *time over* and the rest are
    ///   not fetched — so the sprites that lose tiles are the highest-priority
    ///   ones.
    /// * Of overlapping sprites, the earlier in evaluation order wins.
    fn render_obj(&mut self, v: u16, layers: &mut Layers) {
        let sizes = OBJ_SIZES[usize::from(self.r.obsel >> 5)];
        let interlace = self.r.setini & 0x02 != 0;
        // A sprite's top row shows on the line after its Y coordinate.
        let line = (v - 1) as u8;
        let first = if self.r.oam_priority {
            usize::from((self.r.oam_reload >> 1) & 0x7F)
        } else {
            0
        };
        let mut list = [0usize; 32];
        let mut count = 0;
        for i in 0..128 {
            let s = (first + i) & 127;
            let high = self.oam[0x200 + s / 4] >> (2 * (s % 4));
            let (w, h) = sizes[usize::from((high >> 1) & 1)];
            let y = self.oam[s * 4 + 1];
            let rows = if interlace { h / 2 } else { h };
            if u16::from(line.wrapping_sub(y)) >= rows {
                continue;
            }
            let x = sprite_x(self.oam[s * 4], high);
            if !(x == -256 || (x > -i32::from(w) && x < 256)) {
                continue;
            }
            if count == 32 {
                self.r.range_over = true;
                break;
            }
            list[count] = s;
            count += 1;
        }

        let base = u32::from(self.r.obsel & 7) << 13;
        let gap = (u32::from((self.r.obsel >> 3) & 3) + 1) << 12;
        let mut tiles = 0;
        'fetch: for &s in list[..count].iter().rev() {
            let high = self.oam[0x200 + s / 4] >> (2 * (s % 4));
            let (w, h) = sizes[usize::from((high >> 1) & 1)];
            let x = sprite_x(self.oam[s * 4], high);
            let y = self.oam[s * 4 + 1];
            let name = u16::from(self.oam[s * 4 + 2]);
            let attr = self.oam[s * 4 + 3];
            let name = name | (u16::from(attr & 1) << 8);
            let hflip = attr & 0x40 != 0;
            let vflip = attr & 0x80 != 0;
            let prio = (attr >> 4) & 3;
            let pal = u16::from((attr >> 1) & 7);
            let mut fy = u16::from(line.wrapping_sub(y));
            if interlace {
                fy = fy * 2 + u16::from(self.field);
            }
            if vflip {
                fy = h - 1 - fy;
            }
            let ncols = w / 8;
            for c in 0..ncols {
                let tx = x + 8 * i32::from(c);
                if tx <= -8 || tx >= 256 {
                    continue;
                }
                tiles += 1;
                if tiles > 34 {
                    self.r.time_over = true;
                    break 'fetch;
                }
                let col = if hflip { ncols - 1 - c } else { c };
                // The column wraps inside the name's low nibble and the row
                // inside its high nibble (fullsnes, "OBJ Tile Numbers").
                let tile = (name & 0x100)
                    | ((((name >> 4) + (fy >> 3)) & 0x0F) << 4)
                    | ((name + col) & 0x0F);
                let mut row = base + u32::from(tile & 0xFF) * 16 + u32::from(fy & 7);
                if tile & 0x100 != 0 {
                    row += gap;
                }
                for p in 0..8u32 {
                    let sx = tx + p as i32;
                    if !(0..256).contains(&sx) {
                        continue;
                    }
                    let bit = if hflip { 7 - p } else { p };
                    let pixel = self.planar(row, 4, bit);
                    if pixel == 0 {
                        continue;
                    }
                    let color = self.cgram[usize::from(128 + pal * 16 + pixel)] & 0x7FFF;
                    let i = 2 * sx as usize;
                    for slot in [i, i + 1] {
                        layers.color[usize::from(OBJ)][slot] = color;
                        layers.prio[usize::from(OBJ)][slot] = prio;
                        layers.obj_math[slot] = pal >= 4;
                    }
                }
            }
        }
    }
}

/// A sprite's X: nine bits, the ninth from the high table, sign-extended.
fn sprite_x(low: u8, high: u8) -> i32 {
    let x = i32::from(low) | (i32::from(high & 1) << 8);
    if x >= 256 { x - 512 } else { x }
}
