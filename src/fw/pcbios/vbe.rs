//! Graphics modes and **VBE 2.0**, over the display adapter's own registers.
//!
//! A separate file from [`video`](super::video), which is `INT 10h`'s text
//! services, because this is a separate job: `video` moves a cursor and writes
//! cells, and this programs the card.
//!
//! # What it does
//!
//! * `INT 10h AH=00h` for the **VGA graphics modes** — 0Dh, 0Eh, 10h, 12h and
//!   13h, plus 03h itself. Each is a register set in this ROM: the
//!   miscellaneous output, the five sequencer registers, the twenty-five CRT
//!   controller registers, the nine graphics controller registers and the
//!   twenty-one attribute controller registers, written in that order, with
//!   the DAC loaded and display memory cleared afterwards.
//! * `INT 10h AX=4F00h`-`4F03h` and `4F05h`, the VBE 2.0 core, over the
//!   extension registers `docs/devices/pc-video.md` specifies: the controller
//!   information block, the mode information block, set mode, get mode and the
//!   window position.
//!
//! # Which adapter, and what happens on another one
//!
//! Everything here is gated on [`Vbe::detect`], which writes the extension
//! key to `SR E0h` and reads the identification back from `SR E1h`. A board
//! with `pc.video`'s `model = "6845"`, or with somebody else's card, answers
//! with something else — and then `AH=00h` falls back to what this firmware
//! has always done (record the mode number and change nothing) and the VBE
//! functions answer "not supported", which is what a caller is required to
//! handle. The detection puts the sequencer's reset register back afterwards,
//! because on a register file that decodes three index bits `E0h` aliases to
//! it.
//!
//! # Sources
//!
//! * **VESA BIOS Extension (VBE) Core Functions Standard, Version 2.0** — §4.1
//!   the return codes, §4.2-§4.5 the functions, §13.1 the `VbeInfoBlock` and
//!   §13.2 the `ModeInfoBlock`, including the mode attributes and the direct
//!   colour fields. An interface standard: it says what a *caller* sees and
//!   nothing about how a card is driven.
//! * **IBM VGA Technical Reference**, the register values for modes 03h, 0Dh,
//!   0Eh, 10h, 12h and 13h. They are the timings those modes *are* — 100
//!   character clocks by 449 lines at 70 Hz, 100 by 525 at 60 Hz — and every
//!   one of them is checked against the geometry it produces by
//!   `tests/pc_video_modes.rs`, which is the honest way to hold a table of
//!   numbers: assert what it does rather than where it came from.
//! * **`docs/devices/pc-video.md`** for the extension registers, which are
//!   ours.
//!
//! No emulator's firmware was read (`ROADMAP.md` §1); the Bochs VBE "DISPI"
//! interface is deliberately *not* what this drives, because its only
//! specification is a GPL program's source.

use alloc::vec::Vec;

use super::{F_AX, F_BX, F_CX, F_DX, F_ES, Labels};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BX, CL, CS, CX, Cc, DI, DS, DX, ES, Label, Mem, SI,
};

/// The saved `DI` in the `PUSHA` frame: `PUSHA` pushes it last, so it is the
/// lowest word of the frame.
const F_DI: i32 = 0;

/// The sequencer's index port, which the extension registers share.
const SEQ_PORT: u16 = 0x3c4;
/// The CRT controller's index port on a colour adapter.
const CRTC_PORT: u16 = 0x3d4;
/// The graphics controller's index port.
const GC_PORT: u16 = 0x3ce;
/// The attribute controller's single port.
const ATTR_PORT: u16 = 0x3c0;
/// The input status register whose read resets the attribute flip-flop.
const STATUS_PORT: u16 = 0x3da;
/// The miscellaneous output's write address.
const MISC_PORT: u16 = 0x3c2;
/// The DAC's write index and data ports.
const DAC_INDEX_PORT: u16 = 0x3c8;
const DAC_DATA_PORT: u16 = 0x3c9;

/// The extension registers, from `docs/devices/pc-video.md`.
const EXT_LOCK: u8 = 0xe0;
const EXT_ID: u8 = 0xe1;
const EXT_CONTROL: u8 = 0xe3;
const EXT_WIDTH: u8 = 0xe4;
const EXT_HEIGHT: u8 = 0xe6;
const EXT_BPP: u8 = 0xe8;
const EXT_PITCH: u8 = 0xe9;
const EXT_START: u8 = 0xeb;
const EXT_BANK: u8 = 0xee;
const EXT_MEMORY: u8 = 0xef;
/// What unlocks them, and what `SR E1h` answers with once they are unlocked.
const EXT_KEY: u8 = 0x72;
const EXT_IDENT: u8 = 0x52;

/// Where this firmware puts the linear framebuffer if nothing else has.
///
/// rsemu's own BIOS does not enumerate the bus and assign resources — it finds
/// video through the PCI BIOS interface it implements itself — so on a board
/// where no firmware has placed the card's aperture, the VBE services place
/// it here before reporting it. Above every board's RAM and below the
/// chipset's own windows at `0xfec00000` upward, and clear of the q35's ECAM
/// window at `0xe0000000`.
const LFB_DEFAULT: u32 = 0xfd00_0000;

/// The class code the PCI BIOS finds the display adapter by: base class 03,
/// sub-class 00, programming interface 00.
const DISPLAY_CLASS: u32 = 0x0003_0000;

/// How many bytes of register values one mode's table holds.
const REGISTERS: usize = 1 + 5 + 25 + 9 + 21;

/// A `ModeInfoBlock` is 256 bytes (VBE 2.0 §13.2).
const MODE_INFO_LEN: usize = 256;
/// A `VbeInfoBlock` is 512 (§13.1).
const VBE_INFO_LEN: usize = 512;

// -- offsets inside a ModeInfoBlock (§13.2) ----------------------------------

const MI_BYTES_PER_LINE: i32 = 0x10;
const MI_X_RESOLUTION: i32 = 0x12;
const MI_Y_RESOLUTION: i32 = 0x14;
const MI_BITS_PER_PIXEL: i32 = 0x19;
const MI_PHYS_BASE: i32 = 0x28;

/// Where the total memory sits in a `VbeInfoBlock` (§13.1), in 64 KiB units.
const VI_TOTAL_MEMORY: i32 = 0x12;

/// The labels this module binds, which [`video`](super::video) calls into.
pub(super) struct Vbe {
    /// `INT 10h AH=4Fh`, the VBE dispatch.
    pub entry: Label,
    /// Program the card for the video mode in `AL`, with bit 7 of `AH` as the
    /// no-clear flag. Carry set if this adapter is not ours or the mode is not
    /// one of its own, in which case the caller's fallback runs.
    pub set_mode: Label,
    // -- internal
    detect: Label,
    program: Label,
    dac_load: Label,
    find_mode: Label,
    lfb_base: Label,
    ext_write: Label,
    ext_read: Label,
    clear_planes: Label,
    mode_table: Label,
    vbe_modes: Label,
    vbe_timing: Label,
    vbe_info: Label,
    dac_ega: Label,
    dac_256: Label,
}

impl Vbe {
    /// Every label, created up front because the references are forward.
    pub(super) fn new(a: &mut Asm) -> Vbe {
        Vbe {
            entry: a.label(),
            set_mode: a.label(),
            detect: a.label(),
            program: a.label(),
            dac_load: a.label(),
            find_mode: a.label(),
            lfb_base: a.label(),
            ext_write: a.label(),
            ext_read: a.label(),
            clear_planes: a.label(),
            mode_table: a.label(),
            vbe_modes: a.label(),
            vbe_timing: a.label(),
            vbe_info: a.label(),
            dac_ega: a.label(),
            dac_256: a.label(),
        }
    }
}

// ---------------------------------------------------------------------------
// the mode tables
// ---------------------------------------------------------------------------

/// One mode this firmware can set: its number, its register values, and what
/// the BIOS Data Area should say about it afterwards.
struct Mode {
    number: u8,
    misc: u8,
    seq: [u8; 5],
    crtc: [u8; 25],
    gc: [u8; 9],
    attr: [u8; 21],
    columns: u8,
    rows: u8,
    cell_height: u8,
    page_size: u16,
    /// Bit 0: a graphics mode (clear display memory rather than the text
    /// page). Bit 1: load the 256-colour palette rather than the EGA 64.
    flags: u8,
}

/// The EGA-compatible attribute palette: colour 6 is `14h` so that brown lands
/// on the DAC entry the 64-colour set puts it in, and 8-15 are `38h`-`3Fh`.
const EGA_PALETTE: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
];

/// The identity palette a graphics mode uses, where the pixel value *is* the
/// DAC index.
const IDENTITY_PALETTE: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
];

/// The attribute controller's twenty-one registers: sixteen palette entries
/// and the five that follow.
fn attr(palette: [u8; 16], mode: u8, plane_enable: u8, panning: u8) -> [u8; 21] {
    let mut out = [0u8; 21];
    out[..16].copy_from_slice(&palette);
    out[16] = mode;
    out[17] = 0x00;
    out[18] = plane_enable;
    out[19] = panning;
    out[20] = 0x00;
    out
}

/// Every mode this firmware programs.
fn modes() -> Vec<Mode> {
    alloc::vec![
        // 03h — 80x25 text, a 9x16 cell, 720x400 at 70 Hz. The cursor is on
        // scan lines 14-15 rather than 13-14, which is where `pc.video` has
        // always drawn it and what keeps the console pixel-identical.
        Mode {
            number: 0x03,
            misc: 0x67,
            seq: [0x03, 0x00, 0x03, 0x00, 0x02],
            crtc: [
                0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f, 0x00, 0x4f, 0x0e, 0x0f, 0x00, 0x00,
                0x00, 0x00, 0x9c, 0x8e, 0x8f, 0x28, 0x1f, 0x96, 0xb9, 0xa3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x00, 0xff],
            attr: attr(EGA_PALETTE, 0x0c, 0x0f, 0x08),
            columns: 80,
            rows: 24,
            cell_height: 16,
            page_size: 0x1000,
            flags: 0,
        },
        // 0Dh — 320x200 in sixteen colours: the dot clock halved, two scan
        // lines a row.
        Mode {
            number: 0x0d,
            misc: 0x63,
            seq: [0x03, 0x09, 0x0f, 0x00, 0x06],
            crtc: [
                0x2d, 0x27, 0x28, 0x90, 0x2b, 0x80, 0xbf, 0x1f, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x9c, 0x8e, 0x8f, 0x14, 0x00, 0x96, 0xb9, 0xe3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0f, 0xff],
            attr: attr(IDENTITY_PALETTE, 0x01, 0x0f, 0x00),
            columns: 40,
            rows: 24,
            cell_height: 8,
            page_size: 0x2000,
            flags: 0x01,
        },
        // 0Eh — 640x200 in sixteen colours.
        Mode {
            number: 0x0e,
            misc: 0x63,
            seq: [0x03, 0x01, 0x0f, 0x00, 0x06],
            crtc: [
                0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0xbf, 0x1f, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x9c, 0x8e, 0x8f, 0x28, 0x00, 0x96, 0xb9, 0xe3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0f, 0xff],
            attr: attr(IDENTITY_PALETTE, 0x01, 0x0f, 0x00),
            columns: 80,
            rows: 24,
            cell_height: 8,
            page_size: 0x4000,
            flags: 0x01,
        },
        // 10h — 640x350 in sixteen colours, on the 350-line timing.
        Mode {
            number: 0x10,
            misc: 0xa3,
            seq: [0x03, 0x01, 0x0f, 0x00, 0x06],
            crtc: [
                0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0xbf, 0x1f, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x83, 0x85, 0x5d, 0x28, 0x0f, 0x63, 0xba, 0xe3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0f, 0xff],
            attr: attr(EGA_PALETTE, 0x01, 0x0f, 0x00),
            columns: 80,
            rows: 24,
            cell_height: 14,
            page_size: 0x8000,
            flags: 0x01,
        },
        // 12h — 640x480 in sixteen colours, 60 Hz: what a Windows safe-mode
        // driver asks for.
        Mode {
            number: 0x12,
            misc: 0xe3,
            seq: [0x03, 0x01, 0x0f, 0x00, 0x06],
            crtc: [
                0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0x0b, 0x3e, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0xea, 0x8c, 0xdf, 0x28, 0x00, 0xe7, 0x04, 0xe3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0f, 0xff],
            attr: attr(EGA_PALETTE, 0x01, 0x0f, 0x00),
            columns: 80,
            rows: 29,
            cell_height: 16,
            page_size: 0x0000,
            flags: 0x01,
        },
        // 13h — 320x200 in 256 colours: chain 4, double-word addresses, the
        // 256-colour shift and the attribute controller sampling eight bits.
        Mode {
            number: 0x13,
            misc: 0x63,
            seq: [0x03, 0x01, 0x0f, 0x00, 0x0e],
            crtc: [
                0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0xbf, 0x1f, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x9c, 0x8e, 0x8f, 0x28, 0x40, 0x96, 0xb9, 0xa3, 0xff,
            ],
            gc: [0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x0f, 0xff],
            attr: attr(IDENTITY_PALETTE, 0x41, 0x0f, 0x00),
            columns: 40,
            rows: 24,
            cell_height: 8,
            page_size: 0x0000,
            flags: 0x03,
        },
    ]
}

/// The register values a VBE mode is set on top of.
///
/// Mode 12h's timing — 100 character clocks by 525 lines, 60 Hz — whatever the
/// resolution, because in the linear mode the picture's *shape* comes from the
/// extension registers and the CRT controller only keeps time. A guest polling
/// for a retrace therefore sees a 60 Hz one, and the host advances the machine
/// by a 60 Hz frame, which is true of every mode this firmware offers.
fn vbe_timing() -> Vec<u8> {
    let mut bytes = Vec::new();
    let m = &modes()[4];
    bytes.push(m.misc);
    // Extended memory enabled, no odd/even, no chain 4: the linear mode does
    // not go through the planar pipeline, but the sequencer should still say
    // the memory is there.
    bytes.extend_from_slice(&[0x03, 0x01, 0x0f, 0x00, 0x0e]);
    bytes.extend_from_slice(&m.crtc);
    bytes.extend_from_slice(&m.gc);
    bytes.extend_from_slice(&attr(IDENTITY_PALETTE, 0x01, 0x0f, 0x00));
    bytes
}

/// One VBE mode: the number a caller asks for and the picture it gets.
struct VbeMode {
    number: u16,
    width: u16,
    height: u16,
    bpp: u8,
}

/// The modes `AX=4F00h` lists.
///
/// The VBE-defined numbers for 8-bit and 5:6:5 modes, and the 16.8M ones at
/// 32 bits a pixel — which is what a card with a 32-bit framebuffer reports
/// for them, and what the `BitsPerPixel` field says. 1024x768 at 32 bits is
/// 3 MiB, inside the 4 MiB the boards give the adapter.
fn vbe_modes() -> Vec<VbeMode> {
    alloc::vec![
        VbeMode {
            number: 0x101,
            width: 640,
            height: 480,
            bpp: 8
        },
        VbeMode {
            number: 0x103,
            width: 800,
            height: 600,
            bpp: 8
        },
        VbeMode {
            number: 0x105,
            width: 1024,
            height: 768,
            bpp: 8
        },
        VbeMode {
            number: 0x111,
            width: 640,
            height: 480,
            bpp: 16
        },
        VbeMode {
            number: 0x114,
            width: 800,
            height: 600,
            bpp: 16
        },
        VbeMode {
            number: 0x117,
            width: 1024,
            height: 768,
            bpp: 16
        },
        VbeMode {
            number: 0x112,
            width: 640,
            height: 480,
            bpp: 32
        },
        VbeMode {
            number: 0x115,
            width: 800,
            height: 600,
            bpp: 32
        },
        VbeMode {
            number: 0x118,
            width: 1024,
            height: 768,
            bpp: 32
        },
    ]
}

/// One mode's `ModeInfoBlock`, as it sits in the ROM.
///
/// Every field but `PhysBasePtr` is known when the image is assembled, so the
/// block is assembled here and `AX=4F01h` copies it and patches that one
/// dword — which is the only thing that depends on where firmware put the
/// card's aperture.
fn mode_info(mode: &VbeMode) -> Vec<u8> {
    let mut b = alloc::vec![0u8; MODE_INFO_LEN];
    let put16 = |b: &mut Vec<u8>, at: usize, v: u16| {
        b[at] = v as u8;
        b[at + 1] = (v >> 8) as u8;
    };
    let bytes_per_pixel = u16::from(mode.bpp) / 8;
    let pitch = mode.width * bytes_per_pixel;
    // §13.2's mode attributes: supported (0), optional information available
    // (1), colour (3), graphics (4), and a linear framebuffer (7). Bit 6,
    // "no windowed mode", stays clear because `AX=4F05h` moves a window.
    put16(&mut b, 0x00, 0x009b);
    // Window A exists, is readable and is writable; 64 KiB of granularity and
    // size, at A000h.
    b[0x02] = 0x07;
    b[0x03] = 0x00;
    put16(&mut b, 0x04, 64);
    put16(&mut b, 0x06, 64);
    put16(&mut b, 0x08, 0xa000);
    put16(&mut b, 0x0a, 0x0000);
    // No far window function: a caller uses AX=4F05h, which is what §4.6 says
    // a null pointer means.
    put16(&mut b, 0x0c, 0);
    put16(&mut b, 0x0e, 0);
    put16(&mut b, 0x10, pitch);
    put16(&mut b, 0x12, mode.width);
    put16(&mut b, 0x14, mode.height);
    b[0x16] = 8;
    b[0x17] = 16;
    b[0x18] = 1; // one plane: packed pixels
    b[0x19] = mode.bpp;
    b[0x1a] = 1; // one bank
    // Memory model: 04h packed pixel, 06h direct colour (§13.2's table).
    b[0x1b] = if mode.bpp == 8 { 0x04 } else { 0x06 };
    b[0x1c] = 0; // bank size in KiB: not a banked-memory model
    // Image pages, counted from zero, out of the 4 MiB a board gives the card.
    let page = u32::from(pitch) * u32::from(mode.height);
    let pages = (4 * 1024 * 1024 / page).max(1) - 1;
    b[0x1d] = pages.min(255) as u8;
    b[0x1e] = 1; // reserved, and §13.2 says to set it to 1
    // The direct colour fields: sizes and positions of each gun.
    let (r, rp, g, gp, bl, bp, x, xp) = match mode.bpp {
        8 => (0, 0, 0, 0, 0, 0, 0, 0),
        15 => (5, 10, 5, 5, 5, 0, 1, 15),
        16 => (5, 11, 6, 5, 5, 0, 0, 0),
        _ => (8, 16, 8, 8, 8, 0, 8, 24),
    };
    b[0x1f] = r;
    b[0x20] = rp;
    b[0x21] = g;
    b[0x22] = gp;
    b[0x23] = bl;
    b[0x24] = bp;
    b[0x25] = x;
    b[0x26] = xp;
    b[0x27] = 0; // the DAC is not programmable in a direct colour mode
    // 0x28 is PhysBasePtr, patched at run time.
    // VBE 2.0's linear-mode fields: the same numbers, because there is one
    // layout whichever aperture a caller uses.
    put16(&mut b, 0x32, pitch);
    b[0x34] = b[0x1d];
    b[0x35] = b[0x1d];
    b[0x36] = r;
    b[0x37] = rp;
    b[0x38] = g;
    b[0x39] = gp;
    b[0x3a] = bl;
    b[0x3b] = bp;
    b[0x3c] = x;
    b[0x3d] = xp;
    b
}

/// The DAC's first sixty-four entries: the EGA's `rgbRGB` set, six bits a gun.
fn dac_ega() -> Vec<u8> {
    let mut out = Vec::with_capacity(64 * 3);
    for i in 0..64u8 {
        let gun = |primary: u8, secondary: u8| {
            ((i >> primary) & 1) * 0x2a + ((i >> secondary) & 1) * 0x15
        };
        out.push(gun(2, 5));
        out.push(gun(1, 4));
        out.push(gun(0, 3));
    }
    out
}

/// The 256-colour palette a mode 13h comes up in.
///
/// **Ours**, not IBM's: the sixteen text colours, then sixteen greys, then a
/// 6 x 6 x 6 colour cube, then black to the end. A program that cares about
/// the palette loads its own through the DAC ports, and one that does not gets
/// something it can draw a recognisable picture with.
fn dac_256() -> Vec<u8> {
    let mut out = Vec::with_capacity(256 * 3);
    let ega = dac_ega();
    // 0-15: the sixteen the text modes use, through the EGA palette's own
    // entries, so colour 6 is brown and 7 is light grey.
    for index in EGA_PALETTE {
        let at = usize::from(index) * 3;
        out.extend_from_slice(&ega[at..at + 3]);
    }
    // 16-31: a grey ramp.
    for i in 0..16u8 {
        let v = (u16::from(i) * 0x3f / 15) as u8;
        out.extend_from_slice(&[v, v, v]);
    }
    // 32-247: six levels a gun.
    let level = [0x00u8, 0x0c, 0x19, 0x26, 0x32, 0x3f];
    for r in level {
        for g in level {
            for b in level {
                out.extend_from_slice(&[r, g, b]);
            }
        }
    }
    out.resize(256 * 3, 0);
    out
}

// ---------------------------------------------------------------------------
// emission
// ---------------------------------------------------------------------------

/// Write one register of an index/data pair: `index` in `AL`, value in `AH`,
/// through one 16-bit `OUT` — the idiom every VGA reference gives.
fn out_pair(a: &mut Asm, port: u16, index: u8, value: u8) {
    a.movi(DX, port);
    a.movi(AX, u16::from(value) << 8 | u16::from(index));
    a.out_dx_ax();
}

/// A loop that writes `count` registers of the index/data pair at `port` from
/// the table at `CS:SI`, advancing `SI`.
fn register_loop(a: &mut Asm, port: u16, count: u8) {
    a.movi(DX, port);
    a.movi8(BL, 0);
    let top = a.here_label();
    a.mov8(AL, BL);
    a.out_dx_al();
    a.mov8(AL, Mem::si(0).seg(CS));
    a.inc(SI);
    a.inc(DX);
    a.out_dx_al();
    a.dec(DX);
    a.incm8(BL);
    a.alui8(Alu::CMP, BL, count);
    a.jcc(Cc::NE, top);
}

/// Emit everything this module owns.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels, v: &Vbe) {
    let table = modes();

    // -- detect --------------------------------------------------------------
    //
    // Unlock the extension registers and read the identification back. Carry
    // set means this is not our adapter.
    a.bind(v.detect);
    a.push(AX);
    a.push(DX);
    out_pair(a, SEQ_PORT, EXT_LOCK, EXT_KEY);
    a.movi8(AL, EXT_ID);
    a.out_dx_al();
    a.inc(DX);
    a.in_al_dx();
    a.dec(DX);
    a.alui8(Alu::CMP, AL, EXT_IDENT);
    let ours = a.label();
    a.jcc(Cc::E, ours);
    // Not ours. On a register file that decodes three index bits, E0h aliased
    // to the sequencer's reset register, so put that back before leaving.
    out_pair(a, SEQ_PORT, 0x00, 0x03);
    a.pop(DX);
    a.pop(AX);
    a.stc();
    a.ret();
    a.bind(ours);
    a.pop(DX);
    a.pop(AX);
    a.clc();
    a.ret();

    // -- ext_write, ext_read -------------------------------------------------
    //
    // One extension register: index in `AL`, value in `AH` for a write; index
    // in `AL` for a read, which comes back in `AL`.
    a.bind(v.ext_write);
    a.push(DX);
    a.movi(DX, SEQ_PORT);
    a.out_dx_ax();
    a.pop(DX);
    a.ret();

    a.bind(v.ext_read);
    a.push(DX);
    a.movi(DX, SEQ_PORT);
    a.out_dx_al();
    a.inc(DX);
    a.in_al_dx();
    a.pop(DX);
    a.ret();

    // -- program -------------------------------------------------------------
    //
    // `CS:SI` points at a register set: the miscellaneous output, five
    // sequencer registers, twenty-five CRT controller registers, nine graphics
    // controller registers and twenty-one attribute controller registers, in
    // the order a mode set writes them.
    a.bind(v.program);
    a.push(AX);
    a.push(BX);
    a.push(DX);
    a.push(SI);

    a.mov8(AL, Mem::si(0).seg(CS));
    a.inc(SI);
    a.movi(DX, MISC_PORT);
    a.out_dx_al();

    // The sequencer held in reset while the clock select changes, as its own
    // register 0 bit 1 asks for, and released once the five are written.
    out_pair(a, SEQ_PORT, 0x00, 0x01);
    register_loop(a, SEQ_PORT, 5);
    out_pair(a, SEQ_PORT, 0x00, 0x03);

    // Registers 00h-07h are write-protected by 11h bit 7 in whatever mode was
    // running, so that protection comes off first.
    out_pair(a, CRTC_PORT, 0x11, 0x00);
    register_loop(a, CRTC_PORT, 25);

    register_loop(a, GC_PORT, 9);

    // The attribute controller's flip-flop is put back in its index state by a
    // read of the status register, and the palette address source is set again
    // at the end so the screen comes back on.
    a.movi(DX, STATUS_PORT);
    a.in_al_dx();
    a.movi(DX, ATTR_PORT);
    a.movi8(BL, 0);
    let attr_top = a.here_label();
    a.mov8(AL, BL);
    a.out_dx_al();
    a.mov8(AL, Mem::si(0).seg(CS));
    a.inc(SI);
    a.out_dx_al();
    a.incm8(BL);
    a.alui8(Alu::CMP, BL, 21);
    a.jcc(Cc::NE, attr_top);
    a.movi8(AL, 0x20);
    a.out_dx_al();

    a.pop(SI);
    a.pop(DX);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // -- dac_load ------------------------------------------------------------
    //
    // `AL` zero for the EGA's sixty-four, non-zero for the 256-colour set.
    // `REP OUTSB` reads `DS:SI`, so `DS` becomes this ROM for the transfer.
    a.bind(v.dac_load);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    a.pushs(DS);
    let load_256 = a.label();
    let loaded = a.label();
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::NE, load_256);
    a.movi_label(SI, v.dac_ega);
    a.movi(CX, 64 * 3);
    a.jmp(loaded);
    a.bind(load_256);
    a.movi_label(SI, v.dac_256);
    a.movi(CX, 256 * 3);
    a.bind(loaded);
    a.movrs(AX, CS);
    a.movsr(DS, AX);
    a.movi(DX, DAC_INDEX_PORT);
    a.movi8(AL, 0);
    a.out_dx_al();
    a.movi(DX, DAC_DATA_PORT);
    a.cld();
    a.rep();
    a.outsb();
    a.pops(DS);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- clear_planes --------------------------------------------------------
    //
    // 64 KiB of zeroes through the window at A000h, with every plane enabled
    // and the graphics controller out of the way, which clears all four planes
    // of a planar mode and the whole of a chained one.
    a.bind(v.clear_planes);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    a.pushs(ES);
    out_pair(a, SEQ_PORT, 0x02, 0x0f);
    out_pair(a, GC_PORT, 0x08, 0xff);
    out_pair(a, GC_PORT, 0x01, 0x00);
    a.movi(AX, 0xa000);
    a.movsr(ES, AX);
    a.movi(DI, 0);
    a.movi(AX, 0);
    a.movi(CX, 0x8000);
    a.cld();
    a.rep();
    a.stosw();
    a.pops(ES);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- set_mode ------------------------------------------------------------
    //
    // `AL` is the mode and bit 7 of `AH` the no-clear flag. Carry set means
    // nothing was done and the caller's own fallback should run.
    a.bind(v.set_mode);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    a.push(AX);
    a.call(v.detect);
    let no_adapter = a.label();
    a.jcc(Cc::B, no_adapter);
    a.pop(AX);
    a.push(AX);

    // Find the mode's register set: a table of (number, offset) pairs
    // terminated by FFh.
    a.movi_label(SI, v.mode_table);
    let scan = a.here_label();
    a.mov8(BL, Mem::si(0).seg(CS));
    a.alui8(Alu::CMP, BL, 0xff);
    a.jcc(Cc::E, no_adapter);
    a.alu8(Alu::CMP, BL, AL);
    let found = a.label();
    a.jcc(Cc::E, found);
    a.alui(Alu::ADD, SI, 3);
    a.jmp(scan);

    a.bind(found);
    a.mov(SI, Mem::si(1).seg(CS));
    a.push(SI);
    a.call(v.program);
    a.pop(SI);
    // Past the registers are the BIOS Data Area's numbers and the flags.
    a.alui(Alu::ADD, SI, REGISTERS as u16);
    a.mov8(AL, Mem::si(5).seg(CS));
    a.push(AX);
    a.testi8(AL, 0x02);
    let ega_palette = a.label();
    let palette_done = a.label();
    a.jcc(Cc::E, ega_palette);
    a.movi8(AL, 1);
    a.call(v.dac_load);
    a.jmp(palette_done);
    a.bind(ega_palette);
    a.movi8(AL, 0);
    a.call(v.dac_load);
    a.bind(palette_done);
    a.pop(AX);

    // The BIOS Data Area, from the table: columns, rows less one, the cell
    // height and the page size.
    a.mov8(AL, Mem::si(0).seg(CS));
    a.movi8(AH, 0);
    a.movto(Mem::abs(super::BDA_COLUMNS), AX);
    a.mov8(AL, Mem::si(1).seg(CS));
    a.movto8(Mem::abs(super::BDA_ROWS), AL);
    a.mov8(AL, Mem::si(2).seg(CS));
    a.movi8(AH, 0);
    a.movto(Mem::abs(super::BDA_CHAR_HEIGHT), AX);
    a.mov(AX, Mem::si(3).seg(CS));
    a.movto(Mem::abs(super::BDA_PAGE_SIZE), AX);
    a.movmi(Mem::abs(super::BDA_PAGE_OFFSET), 0);
    a.movmi(Mem::abs(super::BDA_CURSOR), 0);
    a.movmi8(Mem::abs(super::BDA_ACTIVE_PAGE), 0);

    // Clear, unless the caller asked not to: the text page through the
    // firmware's own routine, display memory directly in a graphics mode.
    a.pop(AX);
    a.push(AX);
    a.testi8(AH, 0x80);
    let no_clear = a.label();
    a.jcc(Cc::NE, no_clear);
    a.mov8(AL, Mem::si(5).seg(CS));
    a.testi8(AL, 0x01);
    let graphics_clear = a.label();
    let cleared = a.label();
    a.jcc(Cc::NE, graphics_clear);
    a.call(l.clear_screen);
    a.jmp(cleared);
    a.bind(graphics_clear);
    a.call(v.clear_planes);
    a.bind(cleared);
    a.bind(no_clear);
    a.call(l.set_cursor_hw);
    // A VGA mode leaves no VBE mode in force.
    a.movi(AX, 0);
    a.movto(Mem::abs(super::BDA_VBE_MODE), AX);
    a.pop(AX);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.clc();
    a.ret();

    a.bind(no_adapter);
    a.pop(AX);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.stc();
    a.ret();

    // -- find_mode -----------------------------------------------------------
    //
    // `AX` is a VBE mode number; `SI` comes back pointing at its
    // `ModeInfoBlock` in this ROM, with carry set if there is no such mode.
    a.bind(v.find_mode);
    a.push(BX);
    a.alui(Alu::AND, AX, 0x01ff);
    a.movi_label(SI, v.vbe_modes);
    let vscan = a.here_label();
    a.mov(BX, Mem::si(0).seg(CS));
    a.alui(Alu::CMP, BX, 0xffff);
    let vmiss = a.label();
    a.jcc(Cc::E, vmiss);
    a.alu(Alu::CMP, BX, AX);
    let vfound = a.label();
    a.jcc(Cc::E, vfound);
    a.alui(Alu::ADD, SI, 4);
    a.jmp(vscan);
    a.bind(vfound);
    a.mov(SI, Mem::si(2).seg(CS));
    a.pop(BX);
    a.clc();
    a.ret();
    a.bind(vmiss);
    a.pop(BX);
    a.stc();
    a.ret();

    // -- lfb_base ------------------------------------------------------------
    //
    // Where the card's linear aperture is, as `EAX`, with carry set if there
    // is no PCI display function to ask. The card is found by class code
    // through this firmware's own PCI BIOS interface (`INT 1Ah AH=B1h`), and
    // its base address register 0 is read; a register no firmware has
    // programmed is given an address here, because nothing else on this board
    // assigns PCI resources. The memory-space enable goes on either way —
    // §6.2.5.1's window decodes nothing without it.
    a.bind(v.lfb_base);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    a.push(DI);
    a.movi(AX, 0xb103);
    a.movi32(CX, DISPLAY_CLASS);
    a.movi(SI, 0);
    a.int(0x1a);
    let no_card = a.label();
    a.jcc(Cc::B, no_card);
    // BX now names the bus and device; read BAR0.
    a.movi(AX, 0xb10a);
    a.movi(DI, 0x10);
    a.int(0x1a);
    a.jcc(Cc::B, no_card);
    a.alui32(Alu::AND, CX, 0xffff_fff0);
    let have_base = a.label();
    a.alui32(Alu::CMP, CX, 0);
    a.jcc(Cc::NE, have_base);
    // Unassigned: place the aperture.
    a.movi32(CX, LFB_DEFAULT);
    a.movi(AX, 0xb10d);
    a.movi(DI, 0x10);
    a.int(0x1a);
    a.jcc(Cc::B, no_card);
    a.movi32(CX, LFB_DEFAULT);
    a.bind(have_base);
    a.push(CX);
    // COMMAND bit 1, the memory space enable.
    a.movi(AX, 0xb109);
    a.movi(DI, 0x04);
    a.int(0x1a);
    a.alui(Alu::OR, CX, 0x0002);
    a.movi(AX, 0xb10c);
    a.movi(DI, 0x04);
    a.int(0x1a);
    a.pop(CX);
    a.mov32(AX, CX);
    a.pop(DI);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.clc();
    a.ret();
    a.bind(no_card);
    a.pop(DI);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.stc();
    a.ret();

    // -- the VBE dispatch ----------------------------------------------------
    //
    // §4.1: `AL=4Fh` says the function is supported and `AH` is the status,
    // zero for success. Everything below writes the caller's saved `AX`.
    a.bind(v.entry);
    let vbe_done = a.label();
    let vbe_fail = a.label();
    let f_info = a.label();
    let f_mode_info = a.label();
    let f_set_mode = a.label();
    let f_get_mode = a.label();
    let f_window = a.label();
    a.call(v.detect);
    a.jcc(Cc::B, vbe_fail);
    a.mov8(AL, Mem::bp(F_AX));
    for (function, target) in [
        (0x00u8, f_info),
        (0x01, f_mode_info),
        (0x02, f_set_mode),
        (0x03, f_get_mode),
        (0x05, f_window),
    ] {
        a.alui8(Alu::CMP, AL, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(vbe_fail);

    // AX=4F00h — the controller information block, and the total memory the
    // card reports through SR EFh, in 64 KiB units.
    a.bind(f_info);
    a.movsr(ES, Mem::bp(F_ES));
    a.mov(DI, Mem::bp(F_DI));
    a.mov(BX, DI);
    a.pushs(DS);
    a.movrs(AX, CS);
    a.movsr(DS, AX);
    a.movi_label(SI, v.vbe_info);
    a.movi(CX, (VBE_INFO_LEN / 2) as u16);
    a.cld();
    a.rep();
    a.movsw();
    a.pops(DS);
    a.movi8(AL, EXT_MEMORY);
    a.call(v.ext_read);
    a.movi8(AH, 0);
    // 256 KiB units to 64 KiB units.
    a.shift(crate::fw::asm16::Shift::SHL, AX, 2);
    a.movto(Mem::bx(VI_TOTAL_MEMORY).seg(ES), AX);
    a.jmp(vbe_done);

    // AX=4F01h — one mode's information block, with the aperture patched in.
    a.bind(f_mode_info);
    a.mov(AX, Mem::bp(F_CX));
    a.call(v.find_mode);
    a.jcc(Cc::B, vbe_fail);
    a.movsr(ES, Mem::bp(F_ES));
    a.mov(DI, Mem::bp(F_DI));
    a.mov(BX, DI);
    a.push(SI);
    a.pushs(DS);
    a.movrs(AX, CS);
    a.movsr(DS, AX);
    a.movi(CX, (MODE_INFO_LEN / 2) as u16);
    a.cld();
    a.rep();
    a.movsw();
    a.pops(DS);
    a.pop(SI);
    a.call(v.lfb_base);
    let no_aperture = a.label();
    a.jcc(Cc::B, no_aperture);
    a.movto32(Mem::bx(MI_PHYS_BASE).seg(ES), AX);
    a.bind(no_aperture);
    a.jmp(vbe_done);

    // AX=4F02h — set a mode. A number below 100h is a VGA mode and goes to
    // AH=00h; bit 15 of BX is the no-clear flag, bit 14 asks for the linear
    // aperture, and this card's linear mode is the only one it has.
    a.bind(f_set_mode);
    a.mov(AX, Mem::bp(F_BX));
    a.push(AX);
    a.alui(Alu::AND, AX, 0x01ff);
    a.alui(Alu::CMP, AX, 0x0100);
    let vga_mode = a.label();
    a.jcc(Cc::B, vga_mode);
    a.call(v.find_mode);
    let set_failed = a.label();
    a.jcc(Cc::B, set_failed);

    // The timing first, then the picture the extension registers describe.
    a.push(SI);
    a.movi_label(SI, v.vbe_timing);
    a.call(v.program);
    a.pop(SI);

    a.mov(BX, Mem::si(MI_X_RESOLUTION).seg(CS));
    a.movi8(AL, EXT_WIDTH);
    a.mov8(AH, BL);
    a.call(v.ext_write);
    a.movi8(AL, EXT_WIDTH + 1);
    a.mov8(AH, BH);
    a.call(v.ext_write);
    a.mov(BX, Mem::si(MI_Y_RESOLUTION).seg(CS));
    a.movi8(AL, EXT_HEIGHT);
    a.mov8(AH, BL);
    a.call(v.ext_write);
    a.movi8(AL, EXT_HEIGHT + 1);
    a.mov8(AH, BH);
    a.call(v.ext_write);
    a.mov(BX, Mem::si(MI_BYTES_PER_LINE).seg(CS));
    a.movi8(AL, EXT_PITCH);
    a.mov8(AH, BL);
    a.call(v.ext_write);
    a.movi8(AL, EXT_PITCH + 1);
    a.mov8(AH, BH);
    a.call(v.ext_write);
    a.mov8(BL, Mem::si(MI_BITS_PER_PIXEL).seg(CS));
    a.movi8(AL, EXT_BPP);
    a.mov8(AH, BL);
    a.call(v.ext_write);
    for offset in 0..3u8 {
        a.movi8(AL, EXT_START + offset);
        a.movi8(AH, 0);
        a.call(v.ext_write);
    }
    a.movi8(AL, EXT_BANK);
    a.movi8(AH, 0);
    a.call(v.ext_write);
    a.movi8(AL, EXT_CONTROL);
    a.movi8(AH, 0x01);
    a.call(v.ext_write);

    // Clear the picture unless bit 15 said not to: as many 64 KiB banks as
    // the mode covers, through the window the linear mode puts at A000h.
    a.pop(AX);
    a.push(AX);
    a.testi(AX, 0x8000);
    let keep_memory = a.label();
    a.jcc(Cc::NE, keep_memory);
    a.mov(AX, Mem::si(MI_BYTES_PER_LINE).seg(CS));
    a.mul(Mem::si(MI_Y_RESOLUTION).seg(CS));
    a.alui(Alu::ADD, AX, 0xffff);
    a.alui(Alu::ADC, DX, 0);
    a.mov(BX, DX);
    a.movi(CX, 0);
    let bank_loop = a.here_label();
    a.push(CX);
    a.movi8(AL, EXT_BANK);
    a.mov8(AH, CL);
    a.call(v.ext_write);
    a.movi(AX, 0xa000);
    a.movsr(ES, AX);
    a.movi(DI, 0);
    a.movi(AX, 0);
    a.movi(CX, 0x8000);
    a.cld();
    a.rep();
    a.stosw();
    a.pop(CX);
    a.inc(CX);
    a.alu(Alu::CMP, CX, BX);
    a.jcc(Cc::B, bank_loop);
    a.movi8(AL, EXT_BANK);
    a.movi8(AH, 0);
    a.call(v.ext_write);
    a.bind(keep_memory);

    // Remember it for AX=4F03h, and record a graphics mode in the BDA so that
    // AH=0Fh answers something rather than the last text mode.
    a.pop(AX);
    a.alui(Alu::AND, AX, 0x41ff);
    a.movto(Mem::abs(super::BDA_VBE_MODE), AX);
    a.jmp(vbe_done);

    a.bind(vga_mode);
    a.pop(AX);
    a.mov(BX, AX);
    a.alui(Alu::AND, AX, 0x007f);
    a.testi8(BH, 0x80);
    let clearing = a.label();
    a.jcc(Cc::E, clearing);
    // VBE's "do not clear" is bit 15 of BX; a VGA mode number carries it in
    // bit 7 of AL.
    a.alui8(Alu::OR, AL, 0x80);
    a.bind(clearing);
    a.movi8(AH, 0x00);
    a.int(0x10);
    a.jmp(vbe_done);

    a.bind(set_failed);
    a.pop(AX);
    a.jmp(vbe_fail);

    // AX=4F03h — the mode in force.
    a.bind(f_get_mode);
    a.mov(AX, Mem::abs(super::BDA_VBE_MODE));
    a.movto(Mem::bp(F_BX), AX);
    a.jmp(vbe_done);

    // AX=4F05h — the window position, in 64 KiB granules. Window A only, which
    // is the only one the mode information block declares.
    a.bind(f_window);
    a.mov(BX, Mem::bp(F_BX));
    a.alui8(Alu::CMP, BL, 0x00);
    a.jcc(Cc::NE, vbe_fail);
    a.alui8(Alu::CMP, BH, 0x01);
    let window_get = a.label();
    a.jcc(Cc::E, window_get);
    a.alui8(Alu::CMP, BH, 0x00);
    a.jcc(Cc::NE, vbe_fail);
    a.mov(AX, Mem::bp(F_DX));
    a.mov8(AH, AL);
    a.movi8(AL, EXT_BANK);
    a.call(v.ext_write);
    a.jmp(vbe_done);
    a.bind(window_get);
    a.movi8(AL, EXT_BANK);
    a.call(v.ext_read);
    a.movi8(AH, 0);
    a.movto(Mem::bp(F_DX), AX);
    a.jmp(vbe_done);

    a.bind(vbe_done);
    a.movi(AX, 0x004f);
    a.movto(Mem::bp(F_AX), AX);
    a.ret();

    a.bind(vbe_fail);
    a.movi(AX, 0x014f);
    a.movto(Mem::bp(F_AX), AX);
    a.ret();

    // -- the data ------------------------------------------------------------
    //
    // Past every `RET` above: nothing branches into it.
    let mut entries = Vec::new();
    for mode in &table {
        entries.push((mode.number, a.label()));
    }
    a.bind(v.mode_table);
    for (number, label) in &entries {
        a.db(&[*number]);
        a.dw_label(*label);
    }
    a.db(&[0xff]);

    for (mode, (_, label)) in table.iter().zip(entries.iter()) {
        a.bind(*label);
        a.db(&[mode.misc]);
        a.db(&mode.seq);
        a.db(&mode.crtc);
        a.db(&mode.gc);
        a.db(&mode.attr);
        a.db(&[mode.columns, mode.rows, mode.cell_height]);
        a.dw(mode.page_size);
        a.db(&[mode.flags]);
    }

    a.bind(v.vbe_timing);
    a.db(&vbe_timing());

    let vbe = vbe_modes();
    let mut infos = Vec::new();
    for _ in &vbe {
        infos.push(a.label());
    }
    a.bind(v.vbe_modes);
    for (mode, label) in vbe.iter().zip(infos.iter()) {
        a.dw(mode.number);
        a.dw_label(*label);
    }
    a.dw(0xffff);
    for (mode, label) in vbe.iter().zip(infos.iter()) {
        a.bind(*label);
        a.db(&mode_info(mode));
    }

    // The controller information block, with the mode list and the strings it
    // points at. VBE 2.0 §13.1.
    let oem = a.label();
    let vendor = a.label();
    let product = a.label();
    let revision = a.label();
    let mode_list = a.label();
    a.bind(v.vbe_info);
    a.db(b"VESA");
    a.dw(0x0200);
    a.dw_label(oem);
    a.dw(super::SEGMENT);
    a.db(&[0x00, 0x00, 0x00, 0x00]); // capabilities: nothing optional
    a.dw_label(mode_list);
    a.dw(super::SEGMENT);
    a.dw(0); // total memory, patched from SR EFh
    a.dw(0x0100); // OEM software revision
    a.dw_label(vendor);
    a.dw(super::SEGMENT);
    a.dw_label(product);
    a.dw(super::SEGMENT);
    a.dw_label(revision);
    a.dw(super::SEGMENT);
    let start = a.offset_of(v.vbe_info).expect("just bound");
    let written = usize::from(a.here() - start);
    a.fill(0, VBE_INFO_LEN - written);

    a.bind(mode_list);
    for mode in &vbe {
        a.dw(mode.number);
    }
    a.dw(0xffff);
    a.bind(oem);
    a.db(b"rsemu display adapter\0");
    a.bind(vendor);
    a.db(b"rsemu\0");
    a.bind(product);
    a.db(b"pc.video\0");
    a.bind(revision);
    a.db(b"1.0\0");

    a.bind(v.dac_ega);
    a.db(&dac_ega());
    a.bind(v.dac_256);
    a.db(&dac_256());
}
