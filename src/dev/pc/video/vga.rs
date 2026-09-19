//! The VGA's display memory as the processor sees it: four 64 KiB planes
//! behind a 128 KiB window, the graphics controller's write pipeline, its two
//! read modes and the four latches between them.
//!
//! # Sources
//!
//! * **IBM *Personal System/2 Display Adapter Technical Reference*** (the VGA
//!   technical reference), the sequencer and graphics controller register
//!   descriptions — every rule below is one of that chapter's register bits.
//! * **FreeVGA** ("Hardware Level VGA and SVGA Video Programming
//!   Information", J. D. Neal), the pages *Accessing the VGA Display Memory*,
//!   *Graphics Registers*, *Sequencer Registers* and *CRT Controller
//!   Registers*, which quote IBM's register text and add what was measured on
//!   real cards. It documents hardware; it is not an emulator.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).
//!
//! # Where the planes are
//!
//! In [`Shared::vram`](super::Shared), interleaved: byte `o` of plane `p` is at
//! `o × 4 + p`. That makes the latch load — "a read from display memory also
//! loads a 32 bit latch register, one byte from each plane" (FreeVGA, *Reading
//! from Display Memory*) — one four-byte read, and it is also how a linear
//! framebuffer sees the same memory: byte `n` of the aperture is plane `n & 3`
//! at offset `n >> 2`.
//!
//! # The three addressing modes, and the one reading that had to be chosen
//!
//! *Chain 4* (sequencer memory mode bit 3): the two low address bits select
//! the plane (FreeVGA, *Sequencer Memory Mode Register*). FreeVGA's memory page
//! says the address "is mapped to memory MOD 4 (shifted right 2 places)", and
//! its CRT controller page says a double-word display fetch *shifts the
//! counter left* by two. Both cannot be literal — a byte written at `4` would
//! land at plane offset 1 while the display fetched offset 4 — and the display
//! side is the one IBM's own register text states (CRTC register 14h bit 6,
//! quoted there). So the plane offset is the address with its two low bits
//! cleared, which is the reading under which mode 13h shows what it is sent,
//! and which is also why the well-known fact holds that mode 13h reaches only
//! a quarter of the memory.
//!
//! *Odd/even* (memory mode bit 2 clear for writes, graphics mode bit 4 set for
//! reads): "even system addresses access maps 0 and 2, while odd system
//! addresses access maps 1 and 3". With the graphics controller's *Chain O/E*
//! bit (register 6 bit 1) set, address bit 0 is "replaced by a higher-order
//! bit" — IBM does not say which. This model takes bit 16 of the window offset
//! under the 128 KiB map, which FreeVGA describes as the map meant for chained
//! odd/even use, and zero under the 32 KiB and 64 KiB ones, so the text page
//! at B8000 fills the even bytes of planes 0 and 1: exactly the addresses the
//! display fetches in word mode (CRTC register 17h bit 6 clear, bit 5 set).
//! The miscellaneous output's *odd/even page* bit is latched and not used,
//! because IBM's mode 3 sets it while its word-mode fetch reads even offsets;
//! taken literally it would hide the BIOS's own text page.
//!
//! *Normal*: the address is the plane offset and the map mask picks planes.

use super::{GC_REGISTERS, SEQ_REGISTERS, State};

/// How big one plane is.
pub(super) const PLANE_LEN: u64 = 64 * 1024;

/// How many planes a VGA has.
pub(super) const PLANES: usize = 4;

/// The four planes, as bytes of video memory: everything a legacy mode can
/// address.
pub(super) const PLANAR_LEN: u64 = PLANE_LEN * PLANES as u64;

/// How many registers the VGA's CRT controller has: 00h-18h.
pub(super) const CRTC_REGISTERS: usize = 25;

// -- the mode the adapter comes up in ----------------------------------------
//
// IBM's register values for mode 03h on a VGA — 80x25 text in a 9x16 cell,
// 720x400 at 70 Hz — which is where firmware leaves the card and therefore
// where a device with no firmware yet should be, for the reason the 6845
// model's `CRTC_DEFAULTS` gives.

/// CRT controller registers 00h-18h for mode 03h.
///
/// One deliberate difference from the table a video BIOS loads: the cursor
/// occupies scan lines 14-15 of the cell (registers 0Ah/0Bh) rather than 13-14,
/// so that the text console this adapter draws is the one the 6845 model has
/// always drawn, pixel for pixel.
pub(super) const MODE3_CRTC: [u8; CRTC_REGISTERS] = [
    0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f, // 00-07: 100 x 449 total
    0x00, 0x4f, 0x0e, 0x0f, 0x00, 0x00, 0x00, 0x00, // 08-0F: 16-line cell
    0x9c, 0x8e, 0x8f, 0x28, 0x1f, 0x96, 0xb9, 0xa3, // 10-17: 400 displayed
    0xff, //                                           18: no split
];

/// Sequencer registers 00h-04h for mode 03h: nine-dot cells, planes 0 and 1
/// writable, odd/even addressing.
pub(super) const MODE3_SEQ: [u8; SEQ_REGISTERS] = [0x03, 0x00, 0x03, 0x00, 0x02];

/// Graphics controller registers 00h-08h for mode 03h: odd/even reads, chained
/// odd/even, the B8000 map, every bit written.
pub(super) const MODE3_GC: [u8; GC_REGISTERS] =
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x00, 0xff];

/// Attribute controller registers 00h-14h for mode 03h.
///
/// The palette is the EGA-compatible one — colour 6 is `0x14` and colours 8-15
/// are `0x38`-`0x3f` — so the sixteen text colours reach the DAC entries that
/// the 64-colour EGA set puts brown and the bright colours in (see
/// [`ega_dac`]). Mode control `0x0c` is line graphics and blink; pixel panning
/// `0x08` is "no shift" on a nine-dot cell.
pub(super) const MODE3_ATTR: [u8; 21] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x0c, 0x00, 0x0f, 0x08, 0x00,
];

/// The miscellaneous output for mode 03h: colour addressing, RAM enabled, the
/// 28.322 MHz clock, the odd/even page bit, and the sync polarities that make a
/// 400-line monitor.
pub(super) const MODE3_MISC: u8 = 0x67;

/// DAC entry `i` of the EGA's 64-colour set, in six bits a gun.
///
/// The index is `rgbRGB`: bits 2, 1, 0 are the primary red, green and blue and
/// bits 5, 4, 3 the secondary ones, and each gun is `2 × primary + secondary`
/// thirds of full scale — which is what makes `0x14` brown and `0x38` dark grey
/// (IBM EGA technical reference, attribute controller palette registers; the
/// VGA keeps the encoding for compatibility).
#[must_use]
pub(super) const fn ega_dac(i: u8) -> [u8; 3] {
    const fn gun(i: u8, primary: u32, secondary: u32) -> u8 {
        ((i >> primary) & 1) * 0x2a + ((i >> secondary) & 1) * 0x15
    }
    [gun(i, 2, 5), gun(i, 1, 4), gun(i, 0, 3)]
}

// -- the extension registers --------------------------------------------------

/// The first sequencer index of rsemu's extension registers.
///
/// `docs/devices/pc-video.md` is the specification. They sit in the
/// sequencer's index space, where every SVGA vendor put its own, rather than at
/// a port of their own, so a board needs no new decode; and at `E0h`-`EFh`,
/// above every index the common chip-detection probes touch.
pub(super) const EXT_BASE: u8 = 0xe0;

/// How many there are.
pub(super) const EXT_REGISTERS: usize = 16;

/// `SR E0h` — write [`EXT_KEY`] to unlock the rest; any other value locks
/// them. Reads `1` unlocked, `0` locked.
pub(super) const EXT_LOCK: usize = 0x0;
/// `SR E1h` — read-only identification.
pub(super) const EXT_ID: usize = 0x1;
/// `SR E2h` — read-only interface revision.
pub(super) const EXT_REVISION: usize = 0x2;
/// `SR E3h` — control: bit 0 enables the linear mode.
pub(super) const EXT_CONTROL: usize = 0x3;
/// `SR E4h`/`E5h` — width in pixels, low byte first.
pub(super) const EXT_WIDTH: usize = 0x4;
/// `SR E6h`/`E7h` — height in lines.
pub(super) const EXT_HEIGHT: usize = 0x6;
/// `SR E8h` — bits per pixel: 8, 15, 16, 24 or 32.
pub(super) const EXT_BPP: usize = 0x8;
/// `SR E9h`/`EAh` — bytes from one line to the next.
pub(super) const EXT_PITCH: usize = 0x9;
/// `SR EBh`-`EDh` — the byte offset of the top left pixel, 24 bits.
pub(super) const EXT_START: usize = 0xb;
/// `SR EEh` — which 64 KiB of video memory the A0000 window shows in the linear
/// mode.
pub(super) const EXT_BANK: usize = 0xe;
/// `SR EFh` — read-only: video memory in 256 KiB units.
pub(super) const EXT_MEMORY: usize = 0xf;

/// What unlocks the extension registers: `'r'`.
pub(super) const EXT_KEY: u8 = 0x72;
/// What `SR E1h` reads: `'R'`.
pub(super) const EXT_ID_VALUE: u8 = 0x52;
/// What `SR E2h` reads.
pub(super) const EXT_REVISION_VALUE: u8 = 0x01;

/// `SR E3h` bit 0: the linear mode is on.
pub(super) const EXT_CONTROL_LINEAR: u8 = 0x01;

/// The geometry the extension registers describe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Linear {
    pub width: u32,
    pub height: u32,
    pub bpp: u8,
    pub pitch: u64,
    pub start: u64,
}

impl State {
    /// Whether the linear mode is on.
    #[inline]
    pub(super) fn linear_enabled(&self) -> bool {
        self.ext[EXT_CONTROL] & EXT_CONTROL_LINEAR != 0
    }

    /// The linear mode's geometry, from `SR E4h`-`EDh`.
    pub(super) fn linear(&self) -> Linear {
        let word = |at: usize| u16::from(self.ext[at]) | (u16::from(self.ext[at + 1]) << 8);
        Linear {
            width: u32::from(word(EXT_WIDTH)),
            height: u32::from(word(EXT_HEIGHT)),
            bpp: self.ext[EXT_BPP],
            pitch: u64::from(word(EXT_PITCH)),
            start: u64::from(self.ext[EXT_START])
                | (u64::from(self.ext[EXT_START + 1]) << 8)
                | (u64::from(self.ext[EXT_START + 2]) << 16),
        }
    }

    /// Read extension register `index` (`0`-`15`, i.e. `SR E0h` + index).
    pub(super) fn ext_read(&self, index: usize, vram_len: u64) -> u8 {
        let unlocked = self.ext[EXT_LOCK] != 0;
        match index {
            EXT_LOCK => u8::from(unlocked),
            _ if !unlocked => 0,
            EXT_ID => EXT_ID_VALUE,
            EXT_REVISION => EXT_REVISION_VALUE,
            EXT_MEMORY => (vram_len / PLANAR_LEN).min(255) as u8,
            _ => self.ext.get(index).copied().unwrap_or(0),
        }
    }

    /// Write extension register `index`.
    pub(super) fn ext_write(&mut self, index: usize, value: u8) {
        match index {
            EXT_LOCK => self.ext[EXT_LOCK] = u8::from(value == EXT_KEY),
            _ if self.ext[EXT_LOCK] == 0 => {}
            // Read-only.
            EXT_ID | EXT_REVISION | EXT_MEMORY => {}
            EXT_CONTROL => self.ext[EXT_CONTROL] = value & EXT_CONTROL_LINEAR,
            _ => {
                if let Some(slot) = self.ext.get_mut(index) {
                    *slot = value;
                }
            }
        }
    }
}

// -- the window ---------------------------------------------------------------

/// The graphics controller's memory map select (register 6 bits 3-2) as the
/// part of the 128 KiB window at A0000 it decodes: `(first, length)` in window
/// offsets. FreeVGA, *Miscellaneous Graphics Register*.
#[inline]
#[must_use]
pub(super) fn map_range(gc6: u8) -> (u64, u64) {
    match (gc6 >> 2) & 0x03 {
        0 => (0x00000, 0x20000),
        1 => (0x00000, 0x10000),
        2 => (0x10000, 0x08000),
        _ => (0x18000, 0x08000),
    }
}

/// Where one processor byte lands: which planes, and at what plane offset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Target {
    /// The planes a write may touch, before the map mask.
    planes: u8,
    /// The plane a read-mode-0 read returns.
    read_plane: usize,
    /// The offset within every plane.
    offset: u64,
}

impl State {
    /// Resolve map-relative address `a` under the current addressing mode.
    fn target(&self, a: u64, map_is_128k: bool) -> Target {
        let seq4 = self.vga.seq[4];
        let gc5 = self.vga.gc[5];
        let gc6 = self.vga.gc[6];
        if seq4 & 0x08 != 0 {
            // Chain 4: the two low bits pick the plane, for reads and writes.
            let plane = (a & 3) as usize;
            return Target {
                planes: 1 << plane,
                read_plane: plane,
                offset: (a & !3) & (PLANE_LEN - 1),
            };
        }
        let odd = a & 1 != 0;
        let offset = if gc6 & 0x02 != 0 {
            let high = if map_is_128k { (a >> 16) & 1 } else { 0 };
            (a & !1) | high
        } else {
            a
        } & (PLANE_LEN - 1);
        let write_odd_even = seq4 & 0x04 == 0;
        let read_odd_even = gc5 & 0x10 != 0;
        let select = usize::from(self.vga.gc[4] & 0x03);
        Target {
            planes: if write_odd_even {
                if odd { 0b1010 } else { 0b0101 }
            } else {
                0b1111
            },
            read_plane: if read_odd_even {
                (select & 2) | usize::from(odd)
            } else {
                select
            },
            offset,
        }
    }

    /// One processor write of `value` at map-relative address `a`, through the
    /// graphics controller's pipeline. Returns the planes to store and the
    /// bytes to store in them.
    ///
    /// The pipeline is FreeVGA's *Writing to Display Memory*, stage by stage:
    /// rotate, set/reset, logical operation, bit mask, map mask. Which stages a
    /// write mode uses is the graphics mode register's description (register 5
    /// bits 1-0), and in particular **write mode 3 has no logical operation**:
    /// the data rotate register's function field "is used in Write Mode 0 and
    /// Write Mode 2" only.
    pub(super) fn pipeline_write(
        &self,
        a: u64,
        value: u8,
        map_is_128k: bool,
    ) -> (u64, u8, [u8; 4]) {
        let t = self.target(a, map_is_128k);
        let gc = &self.vga.gc;
        let set_reset = gc[0] & 0x0f;
        let enable_sr = gc[1] & 0x0f;
        let rotate = u32::from(gc[3] & 0x07);
        let function = (gc[3] >> 3) & 0x03;
        let bit_mask = gc[8];
        let latch = self.latch;
        let expand = |bit: u8, p: usize| if bit & (1 << p) != 0 { 0xffu8 } else { 0x00 };
        let alu = |data: u8, latch: u8| match function {
            0 => data,
            1 => data & latch,
            2 => data | latch,
            _ => data ^ latch,
        };
        let mut out = [0u8; 4];
        for (p, slot) in out.iter_mut().enumerate() {
            *slot = match gc[5] & 0x03 {
                0 => {
                    let rotated = value.rotate_right(rotate);
                    let data = if enable_sr & (1 << p) != 0 {
                        expand(set_reset, p)
                    } else {
                        rotated
                    };
                    (alu(data, latch[p]) & bit_mask) | (latch[p] & !bit_mask)
                }
                1 => latch[p],
                2 => {
                    let data = expand(value, p);
                    (alu(data, latch[p]) & bit_mask) | (latch[p] & !bit_mask)
                }
                _ => {
                    let mask = value.rotate_right(rotate) & bit_mask;
                    (expand(set_reset, p) & mask) | (latch[p] & !mask)
                }
            };
        }
        // The map mask is the last stage (FreeVGA: "the fifth stage").
        let planes = t.planes & self.vga.seq[2] & 0x0f;
        (t.offset, planes, out)
    }

    /// One processor read at map-relative address `a`, given the four bytes at
    /// the resolved offset. Returns the offset (so the caller can load the
    /// latches from it) and the byte the read returns.
    ///
    /// Read mode 0 returns one plane; read mode 1 returns the colour compare —
    /// a bit is set where every plane the colour don't care register includes
    /// matches the colour compare register (graphics controller registers 2,
    /// 5 bit 3 and 7).
    pub(super) fn pipeline_read(&self, a: u64, map_is_128k: bool) -> (u64, usize) {
        let t = self.target(a, map_is_128k);
        (t.offset, t.read_plane)
    }

    /// Read mode 1's answer for the four bytes `planes`.
    #[inline]
    pub(super) fn colour_compare(&self, planes: [u8; 4]) -> u8 {
        let compare = self.vga.gc[2];
        let care = self.vga.gc[7];
        let mut result = 0xffu8;
        for (p, byte) in planes.iter().enumerate() {
            if care & (1 << p) != 0 {
                let want = if compare & (1 << p) != 0 { 0xff } else { 0x00 };
                result &= !(byte ^ want);
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use alloc::sync::Arc;

    use super::*;
    use crate::core::space::{MemAttrs, MemOps};
    use crate::dev::pc::video::{Video, VramWindow, test_port};

    /// A VGA with the smallest memory it can have: the four planes.
    fn device() -> Video {
        Video::vga(PLANAR_LEN, 0)
    }

    fn window(video: &Video) -> VramWindow {
        VramWindow {
            shared: Arc::clone(&video.shared),
        }
    }

    /// Write one byte of the register file at 0x3c0-0x3cf.
    fn port_write(video: &Video, offset: u64, value: u8) {
        test_port(video)
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Write one sequencer register, through the ports a guest uses.
    fn seq(video: &Video, index: u8, value: u8) {
        port_write(video, 0x4, index);
        port_write(video, 0x5, value);
    }

    /// Write one graphics controller register.
    fn gc(video: &Video, index: u8, value: u8) {
        port_write(video, 0xe, index);
        port_write(video, 0xf, value);
    }

    /// Write one byte through the A0000 window, as a processor does.
    fn poke(video: &Video, offset: u64, value: u8) {
        window(video)
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Read one byte through the window.
    fn peek(video: &Video, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        window(video)
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    /// Read one byte the way a debugger does.
    fn peek_debug(video: &Video, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        window(video)
            .read(offset, &mut byte, MemAttrs::DEBUG)
            .expect("a byte read is legal");
        byte[0]
    }

    /// Byte `offset` of plane `which`, straight out of memory.
    fn plane(video: &Video, which: usize, offset: u64) -> u8 {
        video
            .vram()
            .read_u8(offset * 4 + which as u64)
            .expect("inside video memory")
    }

    /// The four bytes at `offset`.
    fn planes_at(video: &Video, offset: u64) -> [u8; 4] {
        [
            plane(video, 0, offset),
            plane(video, 1, offset),
            plane(video, 2, offset),
            plane(video, 3, offset),
        ]
    }

    /// Put `bytes` into the four planes at `offset` without going through the
    /// pipeline, so a test can set up what a read or a logical operation is
    /// about to see.
    fn put_planes(video: &Video, offset: u64, bytes: [u8; 4]) {
        for (p, byte) in bytes.iter().enumerate() {
            video
                .vram()
                .write_u8(offset * 4 + p as u64, *byte)
                .expect("inside video memory");
        }
    }

    /// The addressing a 16-colour graphics mode sets up: the 64 KiB map at
    /// A0000, no odd/even, no chain 4, every plane writable.
    fn planar_mode(video: &Video) {
        seq(video, 4, 0x06);
        seq(video, 2, 0x0f);
        gc(video, 6, 0x05);
        gc(video, 5, 0x00);
    }

    #[test]
    fn the_map_mask_decides_which_planes_a_write_reaches() {
        let video = device();
        planar_mode(&video);
        seq(&video, 2, 0b0101);
        poke(&video, 0, 0xff);
        assert_eq!(
            planes_at(&video, 0),
            [0xff, 0x00, 0xff, 0x00],
            "planes 0 and 2 only"
        );
    }

    #[test]
    fn write_mode_0_rotates_the_data_and_honours_set_reset() {
        let video = device();
        planar_mode(&video);
        // Rotate right by four: 0x12 becomes 0x21.
        gc(&video, 3, 0x04);
        poke(&video, 0, 0x12);
        assert_eq!(plane(&video, 0, 0), 0x21);
        gc(&video, 3, 0x00);

        // Set/reset supplies planes 0 and 1 while the host byte supplies 2 and
        // 3: "a 1 bit in the Enable Set/Reset field will cause the
        // corresponding bit plane to be replaced by the bit value in the
        // corresponding Set/Reset field location, replicated 8 times".
        gc(&video, 0, 0b0001);
        gc(&video, 1, 0b0011);
        poke(&video, 1, 0x5a);
        assert_eq!(plane(&video, 0, 1), 0xff, "set/reset bit 0 is set");
        assert_eq!(plane(&video, 1, 1), 0x00, "set/reset bit 1 is clear");
        assert_eq!(plane(&video, 2, 1), 0x5a, "the host byte");
        assert_eq!(plane(&video, 3, 1), 0x5a);
    }

    #[test]
    fn the_logical_operation_combines_the_data_with_the_latches() {
        for (function, expect) in [(0u8, 0x0f), (1, 0x0c), (2, 0xff), (3, 0xf3)] {
            let video = device();
            planar_mode(&video);
            put_planes(&video, 0, [0xfc; 4]);
            // A read loads the latches; the write below combines with them.
            let _ = peek(&video, 0);
            gc(&video, 3, function << 3);
            poke(&video, 0, 0x0f);
            assert_eq!(
                plane(&video, 0, 0),
                expect,
                "function {function}: 0x0f against a latched 0xfc"
            );
        }
    }

    #[test]
    fn the_bit_mask_selects_between_the_result_and_the_latch() {
        let video = device();
        planar_mode(&video);
        put_planes(&video, 0, [0xaa; 4]);
        let _ = peek(&video, 0);
        gc(&video, 8, 0x0f);
        poke(&video, 0, 0xff);
        assert_eq!(
            plane(&video, 0, 0),
            0xaf,
            "the low nibble from the write, the high one from the latch"
        );
    }

    #[test]
    fn write_mode_1_copies_the_latches_and_ignores_the_host_byte() {
        let video = device();
        planar_mode(&video);
        put_planes(&video, 0, [0x11, 0x22, 0x33, 0x44]);
        let _ = peek(&video, 0);
        gc(&video, 5, 0x01);
        // A bit mask and a set/reset that write mode 1 must ignore.
        gc(&video, 8, 0x0f);
        gc(&video, 0, 0x0f);
        gc(&video, 1, 0x0f);
        poke(&video, 8, 0x00);
        assert_eq!(planes_at(&video, 8), [0x11, 0x22, 0x33, 0x44]);
    }

    #[test]
    fn write_mode_2_unpacks_a_pixel_value_across_the_planes() {
        let video = device();
        planar_mode(&video);
        gc(&video, 5, 0x02);
        gc(&video, 8, 0x80);
        // Colour 9 is planes 0 and 3; the bit mask leaves the rest of the byte
        // as the latch, which is zero here.
        poke(&video, 4, 0x09);
        assert_eq!(
            planes_at(&video, 4),
            [0x80, 0x00, 0x00, 0x80],
            "one pixel of colour 9"
        );
    }

    #[test]
    fn write_mode_3_ands_the_host_byte_into_the_bit_mask() {
        let video = device();
        planar_mode(&video);
        put_planes(&video, 0, [0xff; 4]);
        let _ = peek(&video, 0);
        gc(&video, 5, 0x03);
        gc(&video, 0, 0b0010);
        gc(&video, 8, 0xf0);
        // The rotated host byte AND the bit mask is the effective mask: 0x30.
        poke(&video, 0, 0x3c);
        assert_eq!(
            plane(&video, 0, 0),
            0xcf,
            "plane 0's set/reset bit is clear"
        );
        assert_eq!(plane(&video, 1, 0), 0xff, "and plane 1's is set");
        // Write mode 3 expands set/reset whatever the enable register says
        // (FreeVGA: "as if the Enable Set/Reset field were set to 1111b").
        assert_eq!(plane(&video, 2, 0), 0xcf);
    }

    #[test]
    fn a_read_loads_the_latches_and_a_debug_read_does_not() {
        let video = device();
        planar_mode(&video);
        put_planes(&video, 2, [0xde, 0xad, 0xbe, 0xef]);
        assert_eq!(peek_debug(&video, 2), 0xde, "read map select 0");
        assert_eq!(
            video.shared.state.lock().latch,
            [0, 0, 0, 0],
            "a debugger's read must not change what the guest's next write combines with"
        );
        assert_eq!(peek(&video, 2), 0xde);
        assert_eq!(video.shared.state.lock().latch, [0xde, 0xad, 0xbe, 0xef]);
    }

    #[test]
    fn read_map_select_picks_the_plane_a_read_mode_0_read_returns() {
        let video = device();
        planar_mode(&video);
        put_planes(&video, 3, [0x10, 0x20, 0x30, 0x40]);
        for (select, expect) in [(0u8, 0x10), (1, 0x20), (2, 0x30), (3, 0x40)] {
            gc(&video, 4, select);
            assert_eq!(peek(&video, 3), expect);
        }
    }

    #[test]
    fn read_mode_1_compares_the_planes_the_colour_dont_care_names() {
        let video = device();
        planar_mode(&video);
        // Pixels 0-3 are colour 5, pixels 4-7 are colour 1.
        put_planes(&video, 0, [0xff, 0x00, 0xf0, 0x00]);
        gc(&video, 5, 0x08);
        gc(&video, 2, 0x05);
        gc(&video, 7, 0x0f);
        assert_eq!(peek(&video, 0), 0xf0, "only the first four pixels are 5");
        gc(&video, 2, 0x01);
        assert_eq!(peek(&video, 0), 0x0f, "and the last four are 1");
        // Ignore plane 2 and both halves match the reference colour 1.
        gc(&video, 7, 0x0b);
        assert_eq!(peek(&video, 0), 0xff);
    }

    #[test]
    fn odd_even_addressing_splits_a_text_page_between_two_planes() {
        let video = device();
        // The state the adapter comes up in: the B8000 map, chained odd/even,
        // planes 0 and 1 writable.
        poke(&video, 0x18000, b'A');
        poke(&video, 0x18001, 0x1f);
        assert_eq!(plane(&video, 0, 0), b'A', "the code in plane 0");
        assert_eq!(plane(&video, 1, 0), 0x1f, "the attribute in plane 1");
        assert_eq!(plane(&video, 0, 1), 0x00, "and nothing at the odd offset");

        // And a read comes back the same way round, because the graphics
        // controller's odd/even read bit is set too.
        assert_eq!(peek(&video, 0x18000), b'A');
        assert_eq!(peek(&video, 0x18001), 0x1f);

        // The map mask cannot reach planes 1 and 3 through an even address.
        seq(&video, 2, 0x0f);
        poke(&video, 0x18002, 0x55);
        assert_eq!(plane(&video, 2, 2), 0x55, "even addresses reach 0 and 2");
        assert_eq!(plane(&video, 1, 2), 0x00);
        assert_eq!(plane(&video, 3, 2), 0x00);
    }

    #[test]
    fn chain_4_puts_four_consecutive_bytes_in_four_planes() {
        let video = device();
        // What mode 13h sets: chain 4, the 64 KiB map at A0000.
        seq(&video, 4, 0x0e);
        seq(&video, 2, 0x0f);
        gc(&video, 6, 0x05);
        gc(&video, 5, 0x40);
        for (i, byte) in [0x10u8, 0x11, 0x12, 0x13, 0x14].iter().enumerate() {
            poke(&video, i as u64, *byte);
        }
        assert_eq!(
            planes_at(&video, 0),
            [0x10, 0x11, 0x12, 0x13],
            "the first four bytes, one a plane, at plane offset 0"
        );
        assert_eq!(plane(&video, 0, 4), 0x14, "and the fifth at offset 4");
        assert_eq!(
            plane(&video, 0, 1),
            0x00,
            "offsets 1-3 of a plane are unused, which is why mode 13h reaches \
             a quarter of the memory"
        );
        // A read takes the same route.
        for (i, byte) in [0x10u8, 0x11, 0x12, 0x13, 0x14].iter().enumerate() {
            assert_eq!(peek(&video, i as u64), *byte);
        }
    }

    #[test]
    fn the_memory_map_select_decides_which_addresses_are_decoded() {
        for (select, inside, outside) in [
            (0x00u8, 0x00000u64, None),
            (0x04, 0x00000, Some(0x10000)),
            (0x08, 0x10000, Some(0x00000)),
            (0x0c, 0x18000, Some(0x10000)),
        ] {
            let video = device();
            planar_mode(&video);
            gc(&video, 6, select | 0x01);
            poke(&video, inside, 0x5a);
            assert_eq!(peek(&video, inside), 0x5a, "map select {select:#04x}");
            if let Some(outside) = outside {
                poke(&video, outside, 0xa5);
                assert_eq!(
                    peek(&video, outside),
                    0xff,
                    "map select {select:#04x} does not decode {outside:#07x}"
                );
            }
        }
    }

    #[test]
    fn a_128k_map_takes_its_odd_even_address_bit_from_bit_16() {
        let video = device();
        // Chained odd/even over the whole 128 KiB window: the second 64 KiB is
        // the other half of each plane, which is what the map is for.
        gc(&video, 6, 0x03);
        poke(&video, 0x00000, b'a');
        poke(&video, 0x10000, b'b');
        assert_eq!(plane(&video, 0, 0), b'a');
        assert_eq!(plane(&video, 0, 1), b'b');
    }

    #[test]
    fn display_memory_does_not_answer_while_ram_enable_is_clear() {
        let video = device();
        planar_mode(&video);
        poke(&video, 0, 0x5a);
        // Miscellaneous output bit 1 is the RAM enable (FreeVGA, *Miscellaneous
        // Output Register*).
        port_write(&video, 0x2, MODE3_MISC & !0x02);
        assert_eq!(peek(&video, 0), 0xff);
        poke(&video, 0, 0xa5);
        assert_eq!(plane(&video, 0, 0), 0x5a, "and the write was ignored");
    }

    #[test]
    fn the_extension_registers_are_locked_until_the_key_is_written() {
        let video = device();
        seq(&video, EXT_BASE + EXT_CONTROL as u8, 0x01);
        assert!(
            !video.shared.state.lock().linear_enabled(),
            "a locked register file ignores writes"
        );
        seq(&video, EXT_BASE, EXT_KEY);
        seq(&video, EXT_BASE + EXT_CONTROL as u8, 0x01);
        assert!(video.shared.state.lock().linear_enabled());
        seq(&video, EXT_BASE, 0x00);
        assert!(
            video.shared.state.lock().linear_enabled(),
            "locking the file does not undo what it holds"
        );
    }
}
