//! The VGA model's picture: the CRT controller's address sequence over the
//! planes, through the attribute controller and the DAC.
//!
//! # Sources
//!
//! * **FreeVGA**, *CRT Controller Registers*: the start address (0Ch/0Dh), the
//!   preset row scan and byte panning (08h), the maximum scan line and scan
//!   doubling (09h), the offset (13h) — "the starting scan line is increased by
//!   twice the value of this register multiplied by the current memory address
//!   size" — the double-word, word and byte address modes (14h bit 6, 17h bit
//!   6), the address wrap bit (17h bit 5), the CGA/Hercules row-scan
//!   substitutions (17h bits 0 and 1), the count-by-two and count-by-four
//!   clocks (17h bit 3, 14h bit 5), and the line compare (18h, 07h bit 4,
//!   09h bit 6), where "the current scan line address is reset to 0 and the
//!   Preset Row Scan is presumed to be 0".
//! * **FreeVGA**, *Attribute Controller Registers*: the graphics enable, the
//!   8-bit colour enable and the pixel panning mode (10h bits 0, 6 and 5), the
//!   colour plane enable (12h) and the horizontal pixel panning (13h).
//! * **FreeVGA**, *Graphics Registers*: the 256-colour shift mode and the shift
//!   register interleave (05h bits 6 and 5).
//! * **IBM *VGA Technical Reference***, mode 03h's register values, which fix
//!   the one reading FreeVGA leaves open: a pixel panning value of 8 is "no
//!   shift" on a nine-dot cell, so 0-7 shift by one to eight.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).
//!
//! # What the host gets
//!
//! The raster as the monitor would draw it: the displayed character clocks
//! times the dots in a cell, doubled where the sequencer halves the dot clock,
//! by the displayed scan lines. So mode 13h is 640 x 400 with every pixel a
//! 2 x 2 block, mode 12h is 640 x 480, a 320 x 240 unchained mode is 640 x 480,
//! and the text mode is 720 x 400 — the geometry comes from the registers, and
//! no mode number appears anywhere in this file.

use alloc::vec;
use alloc::vec::Vec;

use super::vga::PLANAR_LEN;
use super::{CURSOR_BLINK_FRAMES, FONT_CELL_HEIGHT, State, TEXT_BLINK_FRAMES, expand6, glyph};
use crate::core::space::RamStore;
use crate::host::display::Surface;

/// The most pixels a frame may have.
///
/// A sanity bound, not a hardware one: the extension registers are sixteen bits
/// wide each, so a guest that writes nonsense into them asks for a surface of
/// four thousand million pixels. A mode this model will not scan out is blank
/// rather than an allocation that takes the host down.
const MAX_PIXELS: u64 = 4096 * 4096;

/// The raster the registers describe, in host pixels.
pub(super) fn geometry(state: &State) -> (u32, u32) {
    if state.linear_enabled() {
        let l = state.linear();
        let pixels = u64::from(l.width) * u64::from(l.height);
        if l.width == 0 || l.height == 0 || pixels > MAX_PIXELS || bytes_per_pixel(l.bpp) == 0 {
            return (0, 0);
        }
        return (l.width, l.height);
    }
    let halved = if state.clock_halved() { 2 } else { 1 };
    let width = state.columns() * u64::from(state.char_width()) * halved;
    (width as u32, state.display_lines() as u32)
}

/// How many bytes one pixel of a `bpp`-bit linear mode occupies; `0` for a
/// depth this model does not scan out.
#[inline]
#[must_use]
pub(super) const fn bytes_per_pixel(bpp: u8) -> u64 {
    match bpp {
        8 => 1,
        15 | 16 => 2,
        24 => 3,
        32 => 4,
        _ => 0,
    }
}

/// The four planes, copied out once per frame. A `RamStore` is atomic per
/// byte and needs no lock; one bulk read is cheaper than a quarter of a
/// million single ones.
struct Planes(Vec<u8>);

impl Planes {
    fn read(vram: &RamStore) -> Planes {
        let mut bytes = vec![0u8; PLANAR_LEN as usize];
        let _ = vram.read_at(0, &mut bytes);
        Planes(bytes)
    }

    /// The four bytes at plane offset `addr`.
    #[inline]
    fn at(&self, addr: u64) -> [u8; 4] {
        let i = (addr & 0xffff) as usize * 4;
        [self.0[i], self.0[i + 1], self.0[i + 2], self.0[i + 3]]
    }
}

/// The CRT controller's addressing, read once per frame.
struct Addressing {
    /// 14h bit 6: double-word addresses.
    dword: bool,
    /// 17h bit 6 clear: word addresses.
    word: bool,
    /// 17h bit 5: word mode puts MA15 on MA0 rather than MA13.
    wrap15: bool,
    /// 17h bit 0 clear: row scan bit 0 replaces address bit 13.
    map13: bool,
    /// 17h bit 1 clear: row scan bit 1 replaces address bit 14.
    map14: bool,
    /// How many character clocks each count of the address counter lasts.
    divide: u64,
}

impl Addressing {
    fn of(state: &State) -> Addressing {
        let c14 = state.crtc[0x14];
        let c17 = state.crtc[0x17];
        Addressing {
            dword: c14 & 0x40 != 0,
            word: c17 & 0x40 == 0,
            wrap15: c17 & 0x20 != 0,
            map13: c17 & 0x01 == 0,
            map14: c17 & 0x02 == 0,
            divide: if c14 & 0x20 != 0 {
                4
            } else if c17 & 0x08 != 0 {
                2
            } else {
                1
            },
        }
    }

    /// The plane offset the counter value `ma` fetches on row scan `scan`.
    #[inline]
    fn address(&self, ma: u64, scan: u64) -> u64 {
        let ma = ma & 0xffff;
        let mut addr = if self.dword {
            (ma << 2) | (ma >> 14)
        } else if self.word {
            let low = if self.wrap15 { ma >> 15 } else { ma >> 13 };
            (ma << 1) | (low & 1)
        } else {
            ma
        } & 0xffff;
        if self.map13 {
            addr = (addr & !(1 << 13)) | ((scan & 1) << 13);
        }
        if self.map14 {
            addr = (addr & !(1 << 14)) | (((scan >> 1) & 1) << 14);
        }
        addr
    }
}

/// One scan line's worth of what the vertical walk decided.
#[derive(Debug, Clone, Copy)]
struct Line {
    /// The address counter at the start of the line.
    ma: u64,
    /// The row scan counter.
    scan: u64,
    /// Below a line compare: panning is off if the attribute controller's
    /// pixel panning mode says so.
    split: bool,
}

/// The vertical walk: for every displayed scan line, where the counter starts
/// and which row scan it is on.
fn lines(state: &State, height: u32) -> Vec<Line> {
    let max_scan = u64::from(state.crtc[9] & 0x1f);
    let double = state.crtc[9] & 0x80 != 0;
    let step = 2 * u64::from(state.crtc[0x13]);
    let compare = state.line_compare();
    let byte_pan = u64::from((state.crtc[8] >> 5) & 0x03);
    let mut ma = state.start_address() + byte_pan;
    let mut scan = u64::from(state.crtc[8] & 0x1f);
    let mut split = false;
    let mut second_half = false;
    let mut out = Vec::with_capacity(height as usize);
    for y in 0..u64::from(height) {
        if y == compare {
            ma = 0;
            scan = 0;
            split = true;
            second_half = false;
        }
        out.push(Line {
            ma: ma & 0xffff,
            scan,
            split,
        });
        // Scan doubling divides the row scan counter's clock by two (09h bit
        // 7), so every step below happens on every second line.
        if double {
            second_half = !second_half;
            if second_half {
                continue;
            }
        }
        if scan >= max_scan {
            scan = 0;
            ma = (ma + step) & 0xffff;
        } else {
            scan += 1;
        }
    }
    out
}

/// The attribute controller's pixel shift, in dots, for this line.
fn panning(state: &State, line: &Line, text: bool, eight_bit: bool) -> usize {
    if line.split && state.vga.attr[16] & 0x20 != 0 {
        return 0;
    }
    let value = usize::from(state.vga.attr[19] & 0x0f);
    if text && state.char_width() == 9 {
        // A nine-dot cell counts 8, 0, 1, ... 7 for no shift through eight.
        return if value >= 8 { 0 } else { value + 1 };
    }
    let value = value & 7;
    if eight_bit {
        // Two dots a pixel: the shift is in whole pixels.
        (value >> 1) * 2
    } else {
        value
    }
}

/// Draw the VGA model's current frame into `dst`, which the caller has already
/// shaped to [`geometry`].
pub(super) fn capture(state: &State, vram: &RamStore, dst: &mut Surface) {
    let (width, height) = geometry(state);
    if width == 0 || height == 0 {
        return;
    }
    if state.linear_enabled() {
        linear(state, vram, dst, width, height);
        return;
    }
    let planes = Planes::read(vram);
    let addressing = Addressing::of(state);
    let walk = lines(state, height);
    let halved = state.clock_halved();
    if state.vga.attr[16] & 0x01 == 0 {
        text(state, &planes, &addressing, &walk, dst, halved);
    } else {
        graphics(state, &planes, &addressing, &walk, dst, halved);
    }
}

/// Put a line of dots into `dst`, doubling each one if the dot clock is
/// halved and starting `pan` dots in.
fn emit(dst: &mut Surface, y: u32, dots: &[[u8; 3]], pan: usize, width: u32, halved: bool) {
    for x in 0..width {
        let dot = if halved { x as usize / 2 } else { x as usize } + pan;
        dst.put(x, y, dots.get(dot).copied().unwrap_or([0, 0, 0]));
    }
}

/// A text mode: character codes in plane 0, attributes in plane 1, glyphs from
/// the adapter's own font (see `FONT_ASCII` — plane 2 is not read, for the
/// reason the parent module gives).
fn text(
    state: &State,
    planes: &Planes,
    addressing: &Addressing,
    walk: &[Line],
    dst: &mut Surface,
    halved: bool,
) {
    let (width, _) = geometry(state);
    let columns = state.columns();
    let cw = state.char_width() as usize;
    let cell_height = state.cell_height();
    let colours: Vec<[u8; 3]> = (0..16u8).map(|c| state.rgb(c)).collect();
    let blink = state.blink_enabled();
    let text_visible = (state.frames / TEXT_BLINK_FRAMES).is_multiple_of(2);
    // 0Ah bit 5 is the cursor disable; the VGA's cursor blinks at a fixed
    // rate with no mode bits (FreeVGA, *Cursor Start Register*).
    let cursor_on =
        state.crtc[10] & 0x20 == 0 && (state.frames / CURSOR_BLINK_FRAMES).is_multiple_of(2);
    let cursor_at = state.cursor_address() + u64::from((state.crtc[11] >> 5) & 0x03);
    let cursor_first = u64::from(state.crtc[10] & 0x1f);
    let cursor_last = u64::from(state.crtc[11] & 0x1f);
    let line_graphics = state.vga.attr[16] & 0x04 != 0;
    let mut dots = vec![[0u8; 3]; (columns as usize + 1) * cw];
    for (y, line) in walk.iter().enumerate() {
        // One cell past the right edge, so a panned line has something to
        // shift in.
        for column in 0..=columns {
            let ma = (line.ma + column / addressing.divide) & 0xffff;
            let addr = addressing.address(ma, line.scan);
            let [code, attribute, _, _] = planes.at(addr);
            let mut foreground = attribute & 0x0f;
            let background = if blink {
                (attribute >> 4) & 0x07
            } else {
                (attribute >> 4) & 0x0f
            };
            if blink && attribute & 0x80 != 0 && !text_visible {
                foreground = background;
            }
            let fg = colours[usize::from(foreground)];
            let bg = colours[usize::from(background)];
            let source = (line.scan * FONT_CELL_HEIGHT as u64) / cell_height;
            let bits = glyph(code).get(source as usize).copied().unwrap_or(0);
            let on_cursor = cursor_on
                && ma == cursor_at & 0xffff
                && cursor_first <= cursor_last
                && line.scan >= cursor_first
                && line.scan <= cursor_last;
            let base = column as usize * cw;
            for dot in 0..cw {
                let lit = if on_cursor {
                    true
                } else if dot < 8 {
                    bits & (0x80 >> dot) != 0
                } else {
                    line_graphics && (0xc0..=0xdf).contains(&code) && bits & 0x01 != 0
                };
                dots[base + dot] = if lit { fg } else { bg };
            }
        }
        let pan = panning(state, line, true, false);
        emit(dst, y as u32, &dots, pan, width, halved);
    }
}

/// Five bits of a gun to eight, so that full scale stays full scale — the same
/// arithmetic [`expand6`] does one bit up.
#[inline]
const fn expand5(v: u8) -> u8 {
    (v << 3) | (v >> 2)
}

/// Six bits of a gun to eight, for the middle channel of a 5:6:5 mode.
#[inline]
const fn expand6bit(v: u8) -> u8 {
    (v << 2) | (v >> 4)
}

/// The linear mode: video memory as a packed framebuffer, laid out by the
/// extension registers (`docs/devices/pc-video.md`) rather than by the CRT
/// controller.
///
/// The pixel formats are VBE 2.0's direct colour ones (§13, *Mode Information
/// Block*): 5:5:5, 5:6:5, and eight bits a gun with blue in the lowest byte.
/// An eight-bit mode is an index into the DAC, which is what makes the VBE
/// palette functions the VGA's own DAC ports.
fn linear(state: &State, vram: &RamStore, dst: &mut Surface, width: u32, height: u32) {
    let l = state.linear();
    let bpp = bytes_per_pixel(l.bpp);
    let pitch = if l.pitch == 0 {
        u64::from(width) * bpp
    } else {
        l.pitch
    };
    let palette: Vec<[u8; 3]> = (0..=255u8)
        .map(|c| {
            let entry = state.vga.dac[usize::from(c & state.vga.dac_mask)];
            [expand6(entry[0]), expand6(entry[1]), expand6(entry[2])]
        })
        .collect();
    let mut row = vec![0u8; (u64::from(width) * bpp) as usize];
    for y in 0..height {
        let at = l.start + u64::from(y) * pitch;
        // Past the end of memory is black, not a fault: a card scanning out
        // beyond its own memory shows whatever the wrap gives, and nothing is
        // the honest answer for a mode that was set up wrong.
        row.fill(0);
        if at + row.len() as u64 <= vram.len() {
            let _ = vram.read_at(at, &mut row);
        }
        for x in 0..width {
            let i = (u64::from(x) * bpp) as usize;
            let rgb = match l.bpp {
                8 => palette[usize::from(row[i])],
                15 => {
                    let v = u16::from(row[i]) | (u16::from(row[i + 1]) << 8);
                    [
                        expand5(((v >> 10) & 0x1f) as u8),
                        expand5(((v >> 5) & 0x1f) as u8),
                        expand5((v & 0x1f) as u8),
                    ]
                }
                16 => {
                    let v = u16::from(row[i]) | (u16::from(row[i + 1]) << 8);
                    [
                        expand5(((v >> 11) & 0x1f) as u8),
                        expand6bit(((v >> 5) & 0x3f) as u8),
                        expand5((v & 0x1f) as u8),
                    ]
                }
                // Blue, green, red — the order a little-endian `0x00RRGGBB`
                // word is stored in.
                _ => [row[i + 2], row[i + 1], row[i]],
            };
            dst.put(x, y, rgb);
        }
    }
}

/// A graphics mode: four planes into pixels, by whichever of the three shift
/// arrangements the graphics mode register selects.
///
/// * **256-colour shift** (05h bit 6): each character clock fetches four bytes,
///   one a plane, and each is one pixel — so a 640-dot line of eight-bit
///   pixels is 320 of them, which is what the attribute controller's 8-bit
///   colour enable doubles back up to 640 dots.
/// * **Shift register interleave** (05h bit 5): two bits a pixel, the CGA
///   arrangement of modes 4 and 5 — planes 0 and 1 carry the even and odd
///   halves of the character clock.
/// * **Planar**, otherwise: one bit a plane, eight pixels of four bits.
fn graphics(
    state: &State,
    planes: &Planes,
    addressing: &Addressing,
    walk: &[Line],
    dst: &mut Surface,
    halved: bool,
) {
    let (width, _) = geometry(state);
    let columns = state.columns();
    let cw = state.char_width() as usize;
    let eight_bit = state.vga.attr[16] & 0x40 != 0;
    let shift256 = state.vga.gc[5] & 0x40 != 0;
    let interleave = state.vga.gc[5] & 0x20 != 0;
    let plane_enable = state.vga.attr[18] & 0x0f;
    // Two colour paths: a four-bit pixel goes through the attribute
    // controller's palette, an eight-bit one is "the digital color value to
    // the video DAC" itself (FreeVGA, *Color Select Register*, on mode 13h).
    let low: Vec<[u8; 3]> = (0..16u8).map(|c| state.rgb(c)).collect();
    let direct: Vec<[u8; 3]> = (0..=255u8)
        .map(|c| {
            let entry = state.vga.dac[usize::from(c & state.vga.dac_mask)];
            [expand6(entry[0]), expand6(entry[1]), expand6(entry[2])]
        })
        .collect();
    let mut dots = vec![[0u8; 3]; (columns as usize + 1) * cw];
    for (y, line) in walk.iter().enumerate() {
        for column in 0..=columns {
            let ma = (line.ma + column / addressing.divide) & 0xffff;
            let bytes = planes.at(addressing.address(ma, line.scan));
            let base = column as usize * cw;
            for dot in 0..cw {
                let colour = if shift256 {
                    // Four pixels a character clock, each two dots wide when
                    // the attribute controller samples eight bits.
                    let pixel = if eight_bit { dot / 2 } else { dot };
                    direct[usize::from(bytes[pixel.min(3)])]
                } else if interleave {
                    let byte = bytes[if dot < 4 { 0 } else { 1 }];
                    let high = bytes[if dot < 4 { 2 } else { 3 }];
                    let shift = 6 - 2 * (dot % 4);
                    let value = ((byte >> shift) & 0x03) | (((high >> shift) & 0x03) << 2);
                    low[usize::from(value & plane_enable)]
                } else if dot < 8 {
                    let bit = 7 - dot;
                    let mut value = 0u8;
                    for (p, byte) in bytes.iter().enumerate() {
                        value |= ((byte >> bit) & 1) << p;
                    }
                    low[usize::from(value & plane_enable)]
                } else {
                    // A nine-dot graphics cell has no ninth bit to shift out.
                    low[0]
                };
                dots[base + dot] = colour;
            }
        }
        let pan = panning(state, line, false, eight_bit && shift256);
        emit(dst, y as u32, &dots, pan, width, halved);
    }
}

#[cfg(test)]
mod tests {
    use alloc::sync::Arc;

    use super::super::vga::{
        EXT_BASE, EXT_BPP, EXT_CONTROL, EXT_CONTROL_LINEAR, EXT_HEIGHT, EXT_KEY, EXT_PITCH,
        EXT_WIDTH,
    };
    use super::*;
    use crate::core::space::{MemAttrs, MemOps};
    use crate::dev::pc::video::{Video, VramWindow, test_port};
    use crate::host::display::{PixelFormat, Scanout};

    /// A VGA with four planes and nothing more.
    fn device() -> Video {
        Video::vga(PLANAR_LEN, 0)
    }

    fn port_write(video: &Video, offset: u64, value: u8) {
        test_port(video)
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    fn seq(video: &Video, index: u8, value: u8) {
        port_write(video, 0x4, index);
        port_write(video, 0x5, value);
    }

    fn gc(video: &Video, index: u8, value: u8) {
        port_write(video, 0xe, index);
        port_write(video, 0xf, value);
    }

    fn crtc(video: &Video, index: u8, value: u8) {
        let port = crate::dev::pc::video::test_crtc_port(video);
        port.write(0, &[index], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
        port.write(1, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// One attribute controller register, through the flip-flop at 0x3c0, and
    /// the palette address source set again afterwards so the screen comes
    /// back on.
    fn attr(video: &Video, index: u8, value: u8) {
        // A read of the status register puts the flip-flop back in its index
        // state, which is how a guest resynchronises it and how this helper
        // knows the write below is an index.
        let mut byte = [0u8; 1];
        crate::dev::pc::video::test_status_port(video)
            .read(0, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        port_write(video, 0x0, index);
        port_write(video, 0x0, value);
        port_write(video, 0x0, 0x20);
    }

    fn poke(video: &Video, offset: u64, value: u8) {
        VramWindow {
            shared: Arc::clone(&video.shared),
        }
        .write(offset, &[value], MemAttrs::DEFAULT)
        .expect("a byte write is legal");
    }

    /// A read through the window, which is also how a guest loads the latches
    /// before a masked write — without it the bits outside the bit mask come
    /// from whatever the latches last held.
    fn peek(video: &Video, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        VramWindow {
            shared: Arc::clone(&video.shared),
        }
        .read(offset, &mut byte, MemAttrs::DEFAULT)
        .expect("a byte read is legal");
        byte[0]
    }

    fn captured(video: &Video) -> Surface {
        let scanout = video.scanout();
        let mut surface = Surface::for_scanout(&scanout);
        scanout.capture(&mut surface);
        surface
    }

    /// The DAC entry a colour index reaches, as the host sees it.
    fn rgb(video: &Video, index: u8) -> [u8; 3] {
        let state = video.shared.state.lock();
        let entry = state.vga.dac[usize::from(index)];
        [expand6(entry[0]), expand6(entry[1]), expand6(entry[2])]
    }

    /// A small planar mode whose numbers come from the registers and nowhere
    /// else: 80 dots by 8 lines, ten bytes to a line.
    fn tiny_planar(video: &Video) {
        crtc(video, 0x11, 0x00); // unprotect 00h-07h
        crtc(video, 0x00, 20); // horizontal total: 25 character clocks
        crtc(video, 0x01, 9); // ten displayed: 80 dots
        crtc(video, 0x06, 8); // vertical total: ten lines
        crtc(video, 0x07, 0x00);
        crtc(video, 0x09, 0x00); // one scan line a row, no doubling
        crtc(video, 0x12, 7); // eight displayed lines
        crtc(video, 0x13, 5); // ten bytes a line
        crtc(video, 0x14, 0x00); // byte mode
        crtc(video, 0x17, 0xe3);
        crtc(video, 0x18, 0xff);
        seq(video, 1, 0x01); // eight dots a character clock
        seq(video, 2, 0x0f);
        seq(video, 4, 0x06); // no odd/even, no chain 4
        gc(video, 5, 0x00); // write mode 0, planar shift
        gc(video, 6, 0x05); // graphics, the 64 KiB map at A0000
        for i in 0..16 {
            attr(video, i, i);
        }
        attr(video, 0x10, 0x01); // graphics
        attr(video, 0x12, 0x0f);
        attr(video, 0x13, 0x00);
    }

    #[test]
    fn the_vga_models_text_page_is_the_picture_the_6845_model_draws() {
        // The text console is what every existing pc-at test looks at, so the
        // VGA model has to draw it identically: same glyphs, same colours,
        // same 720x400, same cursor.
        let old = Video::default_device();
        let text = b"rsemu 0.1 -- PC video";
        for (i, byte) in text.iter().enumerate() {
            old.vram().write_u8(i as u64 * 2, *byte).unwrap();
            old.vram().write_u8(i as u64 * 2 + 1, 0x0f).unwrap();
        }
        for (i, byte) in [0xc9u8, 0xcd, 0xcd, 0xbb].iter().enumerate() {
            old.vram().write_u8((80 + i as u64) * 2, *byte).unwrap();
            old.vram().write_u8((80 + i as u64) * 2 + 1, 0x1e).unwrap();
        }

        let new = device();
        for (i, byte) in text.iter().enumerate() {
            poke(&new, 0x18000 + i as u64 * 2, *byte);
            poke(&new, 0x18001 + i as u64 * 2, 0x0f);
        }
        for (i, byte) in [0xc9u8, 0xcd, 0xcd, 0xbb].iter().enumerate() {
            poke(&new, 0x18000 + (80 + i as u64) * 2, *byte);
            poke(&new, 0x18001 + (80 + i as u64) * 2, 0x1e);
        }

        let before = captured(&old);
        let after = captured(&new);
        assert_eq!((after.width(), after.height()), (720, 400));
        assert_eq!(
            after.hash(),
            before.hash(),
            "the VGA model's text mode is not bit for bit the 6845 model's"
        );
    }

    #[test]
    fn a_planar_mode_draws_four_bit_pixels_from_four_planes() {
        let video = device();
        tiny_planar(&video);
        // Write mode 2 unpacks a colour into the planes; the bit mask picks
        // which pixel of the byte it lands in.
        gc(&video, 5, 0x02);
        gc(&video, 8, 0x80);
        let _ = peek(&video, 0);
        poke(&video, 0, 0x09); // pixel 0 of line 0: colour 9
        gc(&video, 8, 0x01);
        let _ = peek(&video, 0); // the latches hold what pixel 0 just became
        poke(&video, 0, 0x04); // pixel 7: colour 4
        gc(&video, 8, 0xff);
        poke(&video, 10, 0x0f); // the whole first byte of line 1: colour 15

        let surface = captured(&video);
        assert_eq!(
            (surface.width(), surface.height()),
            (80, 8),
            "the geometry is the registers': ten character clocks of eight dots \
             by eight scan lines"
        );
        assert_eq!(surface.get(0, 0), Some(rgb(&video, 9)));
        assert_eq!(surface.get(1, 0), Some(rgb(&video, 0)));
        assert_eq!(surface.get(7, 0), Some(rgb(&video, 4)));
        for x in 0..8 {
            assert_eq!(surface.get(x, 1), Some(rgb(&video, 15)), "line 1, dot {x}");
        }
        assert_eq!(surface.get(8, 1), Some(rgb(&video, 0)));

        // The colour plane enable masks a plane out of the pixel, which is
        // what a mode 0Dh-style two-plane mode uses it for.
        attr(&video, 0x12, 0x07);
        let masked = captured(&video);
        assert_eq!(masked.get(0, 0), Some(rgb(&video, 1)), "plane 3 is ignored");
    }

    #[test]
    fn the_offset_register_and_the_start_address_move_the_picture() {
        let video = device();
        tiny_planar(&video);
        gc(&video, 5, 0x02);
        gc(&video, 8, 0xff);
        // One byte of colour 15 at the start of each of the first three lines.
        for line in 0..3u64 {
            poke(&video, line * 10, 0x0f);
        }
        let surface = captured(&video);
        for line in 0..3 {
            assert_eq!(surface.get(0, line), Some(rgb(&video, 15)));
        }
        // Start one line in and the picture scrolls up by exactly the offset.
        crtc(&video, 0x0c, 0);
        crtc(&video, 0x0d, 10);
        let scrolled = captured(&video);
        assert_eq!(scrolled.get(0, 0), Some(rgb(&video, 15)));
        assert_eq!(scrolled.get(0, 2), Some(rgb(&video, 0)), "line 3 was blank");
    }

    #[test]
    fn the_line_compare_splits_the_screen_at_the_scan_line_it_names() {
        let video = device();
        tiny_planar(&video);
        gc(&video, 5, 0x02);
        gc(&video, 8, 0xff);
        poke(&video, 0, 0x0f); // colour 15 on the first line of memory
        poke(&video, 40, 0x09); // colour 9 on the fifth
        crtc(&video, 0x0c, 0);
        crtc(&video, 0x0d, 40); // start four lines in: the picture begins at 9
        crtc(&video, 0x18, 4); // and the split at line 4 goes back to 0
        let surface = captured(&video);
        assert_eq!(surface.get(0, 0), Some(rgb(&video, 9)));
        assert_eq!(
            surface.get(0, 4),
            Some(rgb(&video, 15)),
            "the bottom half starts at address 0 again"
        );
    }

    #[test]
    fn horizontal_panning_shifts_the_line_left_by_whole_dots() {
        let video = device();
        tiny_planar(&video);
        gc(&video, 5, 0x02);
        gc(&video, 8, 0x80);
        poke(&video, 0, 0x0f); // one lit pixel at dot 0
        assert_eq!(captured(&video).get(0, 0), Some(rgb(&video, 15)));
        attr(&video, 0x13, 3);
        let panned = captured(&video);
        assert_eq!(panned.get(0, 0), Some(rgb(&video, 0)));
        assert_eq!(
            panned.get(80 - 3, 0),
            Some(rgb(&video, 0)),
            "and the line has shifted three dots left"
        );
    }

    #[test]
    fn a_chained_256_colour_mode_is_four_pixels_a_character_clock() {
        let video = device();
        tiny_planar(&video);
        // What a 256-colour mode adds to the planar one: chain 4, double-word
        // addresses, the 256-colour shift and the attribute controller's
        // 8-bit sampling, which makes every pixel two dots wide.
        seq(&video, 4, 0x0e);
        crtc(&video, 0x14, 0x40);
        gc(&video, 5, 0x40);
        attr(&video, 0x10, 0x41);
        for (i, colour) in [0x20u8, 0x21, 0x22, 0x23, 0x24].iter().enumerate() {
            poke(&video, i as u64, *colour);
        }
        let surface = captured(&video);
        assert_eq!((surface.width(), surface.height()), (80, 8));
        for (i, colour) in [0x20u8, 0x21, 0x22, 0x23, 0x24].iter().enumerate() {
            let x = i as u32 * 2;
            assert_eq!(surface.get(x, 0), Some(rgb(&video, *colour)), "pixel {i}");
            assert_eq!(
                surface.get(x + 1, 0),
                Some(rgb(&video, *colour)),
                "pixel {i} is two dots wide"
            );
        }
    }

    #[test]
    fn an_unchained_256_colour_mode_reads_one_pixel_from_each_plane() {
        let video = device();
        tiny_planar(&video);
        // Mode X: the same shift arrangement with chain 4 off and byte
        // addresses, so one plane offset carries four pixels and the map mask
        // picks which of them a write reaches.
        gc(&video, 5, 0x40);
        attr(&video, 0x10, 0x41);
        seq(&video, 2, 0x04); // plane 2 only: pixel 2 of the group
        poke(&video, 0, 0x36);
        seq(&video, 2, 0x01);
        poke(&video, 0, 0x12);
        let surface = captured(&video);
        assert_eq!(surface.get(0, 0), Some(rgb(&video, 0x12)), "plane 0");
        assert_eq!(surface.get(2, 0), Some(rgb(&video, 0x00)), "plane 1");
        assert_eq!(surface.get(4, 0), Some(rgb(&video, 0x36)), "plane 2");
    }

    #[test]
    fn scan_doubling_and_the_maximum_scan_line_repeat_a_line() {
        let video = device();
        tiny_planar(&video);
        gc(&video, 5, 0x02);
        gc(&video, 8, 0xff);
        poke(&video, 0, 0x0f);
        poke(&video, 10, 0x09);
        // Two scan lines a row (09h bits 4-0), so each memory line is drawn
        // twice and eight displayed lines show four of them.
        crtc(&video, 0x09, 0x01);
        let surface = captured(&video);
        assert_eq!(surface.get(0, 0), Some(rgb(&video, 15)));
        assert_eq!(surface.get(0, 1), Some(rgb(&video, 15)));
        assert_eq!(surface.get(0, 2), Some(rgb(&video, 9)));
        assert_eq!(surface.get(0, 3), Some(rgb(&video, 9)));

        // The scan-doubling bit does the same thing by halving the row scan
        // counter's clock instead.
        crtc(&video, 0x09, 0x80);
        let doubled = captured(&video);
        assert_eq!(doubled.get(0, 1), Some(rgb(&video, 15)));
        assert_eq!(doubled.get(0, 2), Some(rgb(&video, 9)));
    }

    #[test]
    fn the_linear_mode_reads_video_memory_as_packed_pixels() {
        let video = Video::vga(PLANAR_LEN, 0);
        let ext = |index: u8, value: u8| seq(&video, EXT_BASE + index, value);
        seq(&video, EXT_BASE, EXT_KEY);
        ext(EXT_WIDTH as u8, 4);
        ext(EXT_WIDTH as u8 + 1, 0);
        ext(EXT_HEIGHT as u8, 2);
        ext(EXT_HEIGHT as u8 + 1, 0);
        ext(EXT_BPP as u8, 32);
        ext(EXT_PITCH as u8, 16);
        ext(EXT_PITCH as u8 + 1, 0);
        ext(EXT_CONTROL as u8, EXT_CONTROL_LINEAR);

        // Blue, green, red, pad — the order a little-endian 0x00RRGGBB word
        // sits in memory in.
        for (i, byte) in [0x10u8, 0x20, 0x30, 0x00].iter().enumerate() {
            video.vram().write_u8(i as u64, *byte).unwrap();
        }
        // The second line, through the pitch.
        for (i, byte) in [0x01u8, 0x02, 0x03, 0x00].iter().enumerate() {
            video.vram().write_u8(16 + i as u64, *byte).unwrap();
        }
        let surface = captured(&video);
        assert_eq!((surface.width(), surface.height()), (4, 2));
        assert_eq!(surface.get(0, 0), Some([0x30, 0x20, 0x10]));
        assert_eq!(surface.get(0, 1), Some([0x03, 0x02, 0x01]));
        assert_eq!(surface.get(1, 0), Some([0, 0, 0]));

        // Eight bits a pixel is an index into the DAC, which is what makes the
        // VBE palette calls the VGA's own DAC ports.
        ext(EXT_BPP as u8, 8);
        ext(EXT_PITCH as u8, 4);
        video.vram().write_u8(0, 0x09).unwrap();
        assert_eq!(captured(&video).get(0, 0), Some(rgb(&video, 0x09)));

        // 5:6:5 sits in two bytes, low byte first.
        ext(EXT_BPP as u8, 16);
        ext(EXT_PITCH as u8, 8);
        video.vram().write_u8(0, 0x00).unwrap();
        video.vram().write_u8(1, 0xf8).unwrap();
        assert_eq!(captured(&video).get(0, 0), Some([0xff, 0x00, 0x00]));
    }

    #[test]
    fn a_linear_geometry_larger_than_any_screen_is_refused_rather_than_allocated() {
        let video = Video::vga(PLANAR_LEN, 0);
        seq(&video, EXT_BASE, EXT_KEY);
        seq(&video, EXT_BASE + EXT_WIDTH as u8, 0xff);
        seq(&video, EXT_BASE + EXT_WIDTH as u8 + 1, 0xff);
        seq(&video, EXT_BASE + EXT_HEIGHT as u8, 0xff);
        seq(&video, EXT_BASE + EXT_HEIGHT as u8 + 1, 0xff);
        seq(&video, EXT_BASE + EXT_BPP as u8, 32);
        seq(&video, EXT_BASE + EXT_CONTROL as u8, EXT_CONTROL_LINEAR);
        let info = video.scanout().info();
        assert_eq!((info.width, info.height), (0, 0));
        assert_eq!(info.preferred_format, PixelFormat::RGBA8888);
    }
}
