//! **A guest sets a graphics mode through rsemu's own BIOS and draws, and the
//! host's frame is checked pixel by pixel.**
//!
//! Four programs, each a boot sector assembled with [`rsemu::fw::asm16`] — the
//! same assembler the firmware is written with — running on
//! `machines/pc-at.machine` as it ships:
//!
//! 1. **Mode 12h**, 640x480 in sixteen colours: sixteen horizontal bands,
//!    drawn with the graphics controller's write mode 2 so that every band is
//!    one `REP STOSB` over eighty bytes and the four planes are written at
//!    once. The picture proves the planar path end to end — the BIOS's mode
//!    table, the attribute controller's palette, the DAC, and the scanout's
//!    bit-plane assembly.
//! 2. **Mode 13h**, 320x200 in 256 colours: a ramp, one byte a pixel through
//!    the chained window at A000h. The host frame is 640x400, because that is
//!    the raster a VGA emits for it, so every pixel is a 2x2 block — and the
//!    colours are read back out of the DAC through its own ports rather than
//!    being written down here, which is what makes this a test of the chain
//!    and not of a table.
//! 3. **Mode X**, 320x240 unchained: the guest turns chain 4 off, programs the
//!    480-line timing itself and writes one plane at a time through the map
//!    mask. Four vertical stripes, one a plane, and the scanout has to take
//!    consecutive pixels from consecutive planes to show them.
//! 4. **VBE**, `AX=4F00h`/`4F01h`/`4F02h`: the controller information block,
//!    the mode information block for 640x480 at 32 bits a pixel, and the mode
//!    set with bit 14 — the linear framebuffer. The guest draws through the
//!    banked window; the *test* then writes through `PhysBasePtr`, which is
//!    the card's PCI base address register, and both show up in the same
//!    frame.
//!
//! Every frame is also written out as a PNG when `RSEMU_SCREENSHOT_DIR` names
//! a directory, which is the convention `tests/pc_at_board.rs` already uses.
//!
//! No accelerator: the guests are real-mode programs talking to a display
//! adapter, so the interpreter runs them everywhere.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::pc::video::VideoScanout;
use rsemu::fw::asm16::{
    AL, AX, Alu, Asm, BH, BL, BX, CX, Cc, DI, DS, DX, ES, Mem, SI, SP, SS, Shift,
};
use rsemu::host::display::{Scanout, Surface};
use rsemu::machine::{Machine, build};

/// Where the boot sector lands.
const BOOT: u16 = 0x7c00;
/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;
/// The guest writes this when it has finished drawing.
const OFF_DONE: u16 = SCRATCH;
/// And this when it starts, so a picture cannot be mistaken for a boot that
/// never happened.
const OFF_STARTED: u16 = SCRATCH + 2;
/// Where the VBE guest leaves what the services told it.
const OFF_VBE: u16 = SCRATCH + 4;

const STARTED: u16 = 0x1234;
const DONE: u16 = 0x600d;

/// A boot sector's prologue: a flat segment set and a stack.
fn prologue(a: &mut Asm) {
    a.seek(BOOT);
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.sti();
    a.movmi(Mem::abs(OFF_STARTED), STARTED);
}

/// Its epilogue: say so, and stop.
fn epilogue(a: &mut Asm) {
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movmi(Mem::abs(OFF_DONE), DONE);
    let spin = a.here_label();
    a.hlt();
    a.jmp(spin);
    assert!(
        a.here() <= BOOT + 510,
        "the boot sector is {} bytes and 510 is all a sector has",
        a.here() - BOOT
    );
    a.seek(BOOT + 510);
    a.db(&[0x55, 0xaa]);
}

/// One register of an index/data pair, the way a program writes it.
fn out_pair(a: &mut Asm, port: u16, index: u8, value: u8) {
    a.movi(DX, port);
    a.movi(AX, u16::from(value) << 8 | u16::from(index));
    a.out_dx_ax();
}

/// A 1.44 MB diskette carrying `sector`.
fn diskette(sector: Vec<u8>) -> Vec<u8> {
    let mut image = sector;
    assert_eq!(image.len(), 512, "a boot sector is one sector");
    image.resize(1_474_560, 0);
    image
}

/// Assemble a guest, put it on the diskette, boot `pc-at` on rsemu's own BIOS
/// and run until the guest says it has finished drawing.
fn run(name: &str, program: impl FnOnce(&mut Asm)) -> (Machine, VideoScanout) {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    prologue(&mut a);
    program(&mut a);
    epilogue(&mut a);
    let image = a.finish();
    let sector = image[usize::from(BOOT)..].to_vec();

    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", diskette(sector));
    for slot in ["hd0", "hd1", "cdrom"] {
        options.realize.media.insert(slot, Vec::new());
    }
    rsemu::host::display::pc::capture::install(&mut options).expect("one display class");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let scanout = rsemu::host::display::pc::capture::take_clocked(&options.realize.hosts, &m)
        .expect("the board has a display adapter");
    m.reset(ResetKind::Cold);
    m.sweep();

    for _ in 0..8000 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek16(&m, OFF_DONE) == DONE {
            break;
        }
    }
    assert_eq!(
        peek16(&m, OFF_STARTED),
        STARTED,
        "{name}: the boot sector never ran"
    );
    assert_eq!(
        peek16(&m, OFF_DONE),
        DONE,
        "{name}: the guest never finished"
    );
    (m, scanout)
}

fn peek(m: &Machine, at: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(at, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

fn peek16(m: &Machine, at: u16) -> u16 {
    u16::from(peek(m, u64::from(at))) | (u16::from(peek(m, u64::from(at) + 1)) << 8)
}

fn peek32(m: &Machine, at: u16) -> u32 {
    u32::from(peek16(m, at)) | (u32::from(peek16(m, at + 2)) << 16)
}

/// One byte into an I/O port, as the test's own hand rather than the guest's.
fn outb(m: &Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a byte to a port");
}

fn inb(m: &Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .unwrap_or(0xff) as u8
}

/// DAC entry `index`, read back through the card's own ports and expanded to
/// eight bits a gun the way the scanout does.
fn dac(m: &Machine, index: u8) -> [u8; 3] {
    outb(m, 0x3c7, index);
    let expand = |v: u8| (v << 2) | (v >> 4);
    [
        expand(inb(m, 0x3c9) & 0x3f),
        expand(inb(m, 0x3c9) & 0x3f),
        expand(inb(m, 0x3c9) & 0x3f),
    ]
}

/// The frame, as the host sees it.
fn frame(scanout: &VideoScanout) -> Surface {
    let mut surface = Surface::for_scanout(scanout);
    scanout.capture(&mut surface);
    surface
}

/// Write the frame out when a directory was named, so that a change to any of
/// this can be *looked* at rather than only hashed.
fn screenshot(surface: &Surface, name: &str) {
    #[cfg(feature = "display-png")]
    if let Ok(dir) = std::env::var("RSEMU_SCREENSHOT_DIR") {
        let bytes = rsemu::host::display::png::encode(surface).expect("a PNG");
        let path = std::path::Path::new(&dir).join(name);
        std::fs::write(&path, &bytes).expect("the screenshot directory exists");
        println!("wrote {} ({} bytes)", path.display(), bytes.len());
    }
    #[cfg(not(feature = "display-png"))]
    let _ = (surface, name);
}

// ---------------------------------------------------------------------------
// mode 12h
// ---------------------------------------------------------------------------

/// How many scan lines one band of the sixteen covers.
const BAND: u16 = 30;

#[test]
fn a_guest_sets_mode_12h_through_int_10h_and_its_bands_reach_the_host() {
    let (m, scanout) = run("mode 12h", |a| {
        a.movi(AX, 0x0012);
        a.int(0x10);
        // Write mode 2 with every bit of the mask: the byte written is the
        // colour, one pixel a bit, across all four planes at once.
        out_pair(a, 0x3ce, 0x05, 0x02);
        out_pair(a, 0x3ce, 0x08, 0xff);
        a.movi(AX, 0xa000);
        a.movsr(ES, AX);
        a.movi(DI, 0);
        a.movi(BX, 0);
        let row = a.here_label();
        a.mov(AX, BX);
        a.movi(DX, 0);
        a.movi(CX, BAND);
        a.div(CX);
        a.alui8(Alu::AND, AL, 0x0f);
        a.movi(CX, 80);
        a.cld();
        a.rep();
        a.stosb();
        a.inc(BX);
        a.alui(Alu::CMP, BX, 480);
        a.jcc(Cc::B, row);
    });

    let surface = frame(&scanout);
    assert_eq!(
        (surface.width(), surface.height()),
        (640, 480),
        "the mode's geometry comes from the registers the BIOS wrote"
    );
    // Sixteen bands, each the colour its row number says, through the
    // attribute controller's palette into the DAC.
    for band in 0..16u8 {
        let y = u32::from(band) * u32::from(BAND) + 5;
        let want = dac(&m, dac_index(&m, band));
        for x in [0u32, 1, 320, 639] {
            assert_eq!(surface.get(x, y), Some(want), "band {band} at {x},{y}");
        }
    }
    // The classic sixteen: black, blue, red, brown and white, as the EGA
    // palette and the 64-colour DAC put them.
    assert_eq!(surface.get(0, 5), Some([0x00, 0x00, 0x00]));
    assert_eq!(
        surface.get(0, 5 + u32::from(BAND)),
        Some([0x00, 0x00, 0xaa])
    );
    assert_eq!(
        surface.get(0, 5 + 4 * u32::from(BAND)),
        Some([0xaa, 0x00, 0x00])
    );
    assert_eq!(
        surface.get(0, 5 + 6 * u32::from(BAND)),
        Some([0xaa, 0x55, 0x00]),
        "colour 6 is brown, through palette register 14h"
    );
    assert_eq!(
        surface.get(0, 5 + 15 * u32::from(BAND)),
        Some([0xff, 0xff, 0xff])
    );
    screenshot(&surface, "pc-mode12.png");
}

/// Which DAC entry a sixteen-colour pixel value reaches: the attribute
/// controller's palette register, read back through 0x3c0/0x3c1.
fn dac_index(m: &Machine, colour: u8) -> u8 {
    // A read of the status register puts the flip-flop in its index state.
    let _ = inb(m, 0x3da);
    outb(m, 0x3c0, colour | 0x20);
    let value = inb(m, 0x3c1);
    let _ = inb(m, 0x3da);
    outb(m, 0x3c0, 0x20);
    value & 0x3f
}

// ---------------------------------------------------------------------------
// mode 13h
// ---------------------------------------------------------------------------

#[test]
fn a_guest_sets_mode_13h_and_every_byte_it_writes_is_a_pixel() {
    let (m, scanout) = run("mode 13h", |a| {
        a.movi(AX, 0x0013);
        a.int(0x10);
        a.movi(AX, 0xa000);
        a.movsr(ES, AX);
        a.movi(DI, 0);
        a.movi(BX, 0);
        // Every line is a ramp starting at the line number, so that both axes
        // matter and a transposed picture could not pass.
        let row = a.here_label();
        a.mov(AX, BX);
        a.movi(CX, 320);
        a.cld();
        let col = a.here_label();
        a.stosb();
        a.incm8(AL);
        a.loop_(col);
        a.inc(BX);
        a.alui(Alu::CMP, BX, 200);
        a.jcc(Cc::B, row);
    });

    let surface = frame(&scanout);
    assert_eq!(
        (surface.width(), surface.height()),
        (640, 400),
        "320x200 is a 640x400 raster with every pixel two dots by two lines"
    );
    for (x, y) in [(0u32, 0u32), (1, 0), (17, 3), (319, 199), (200, 100)] {
        let colour = ((y + x) & 0xff) as u8;
        let want = dac(&m, colour);
        for dx in 0..2 {
            for dy in 0..2 {
                assert_eq!(
                    surface.get(x * 2 + dx, y * 2 + dy),
                    Some(want),
                    "pixel {x},{y} is colour {colour}"
                );
            }
        }
    }
    screenshot(&surface, "pc-mode13.png");
}

// ---------------------------------------------------------------------------
// mode X
// ---------------------------------------------------------------------------

#[test]
fn a_guest_unchains_the_256_colour_mode_and_writes_one_plane_at_a_time() {
    let (m, scanout) = run("mode X", |a| {
        a.movi(AX, 0x0013);
        a.int(0x10);
        // Mode X, as every DOS game built it: chain 4 off, byte addresses
        // rather than double-word ones, and the 480-line timing so that a
        // 320x240 picture is square.
        out_pair(a, 0x3c4, 0x04, 0x06);
        out_pair(a, 0x3d4, 0x14, 0x00);
        out_pair(a, 0x3d4, 0x17, 0xe3);
        a.movi(DX, 0x3c2);
        a.movi8(AL, 0xe3);
        a.out_dx_al();
        out_pair(a, 0x3d4, 0x11, 0x00);
        for (index, value) in [
            (0x06u8, 0x0du8),
            (0x07, 0x3e),
            (0x09, 0x41),
            (0x10, 0xea),
            (0x11, 0xac),
            (0x12, 0xdf),
            (0x15, 0xe7),
            (0x16, 0x06),
        ] {
            out_pair(a, 0x3d4, index, value);
        }
        // One plane at a time: plane p carries every fourth pixel, so a plane
        // filled with one colour is every fourth pixel of the picture. BH is
        // the map mask, BL the plane number.
        a.movi(BX, 0x0100);
        let plane = a.here_label();
        a.movi(DX, 0x3c4);
        a.movi8(AL, 0x02);
        a.out_dx_al();
        a.inc(DX);
        a.mov8(AL, BH);
        a.out_dx_al();
        a.movi(AX, 0xa000);
        a.movsr(ES, AX);
        a.movi(DI, 0);
        // Four colours a long way apart in the palette the BIOS loaded: the
        // grey ramp at 10h, every fourth entry.
        a.mov8(AL, BL);
        a.shift8(Shift::SHL, AL, 2);
        a.alui8(Alu::ADD, AL, 0x10);
        a.movi(CX, 80 * 240);
        a.cld();
        a.rep();
        a.stosb();
        a.shift8(Shift::SHL, BH, 1);
        a.incm8(BL);
        a.alui8(Alu::CMP, BL, 4);
        a.jcc(Cc::B, plane);
    });

    let surface = frame(&scanout);
    assert_eq!(
        (surface.width(), surface.height()),
        (640, 480),
        "320x240 unchained, on the 480-line timing"
    );
    screenshot(&surface, "pc-modex.png");
    // Each plane carries every fourth pixel, and the guest filled plane p with
    // colour 0x20 + p.
    for p in 0..4u8 {
        let want = dac(&m, 0x10 + p * 4);
        let x = u32::from(p) * 2;
        assert_eq!(surface.get(x, 0), Some(want), "plane {p} at the top left");
        assert_eq!(surface.get(x, 478), Some(want), "and at the bottom");
    }
}

// ---------------------------------------------------------------------------
// VBE
// ---------------------------------------------------------------------------

/// The mode the VBE guest asks for: 640x480, 32 bits a pixel.
const VBE_MODE: u16 = 0x112;
/// Where it puts the controller information block.
const VBE_INFO: u16 = 0x0600;
/// And one mode's information block.
const MODE_INFO: u16 = 0x0900;
/// Where the guest keeps the colour it paints each 64 KiB bank.
const BANK_TABLE: u16 = 0x0520;
/// Those colours, as a 32-bit mode stores them: `0x00RRGGBB`.
const BANK_COLOURS: [u32; 4] = [0x0000_00ff, 0x0000_ff00, 0x00ff_0000, 0x00ff_ff00];
/// Which scan line the *test* draws on, through the card's PCI aperture.
const APERTURE_LINE: u32 = 300;

#[test]
fn a_guest_sets_a_vbe_linear_mode_and_the_framebuffer_is_where_the_bar_says() {
    let (m, scanout) = run("vbe", |a| {
        // AX=4F00h, the controller information block.
        a.movi(AX, 0);
        a.movsr(ES, AX);
        a.movi(DI, VBE_INFO);
        a.movi(AX, 0x4f00);
        a.int(0x10);
        a.movto(Mem::abs(OFF_VBE), AX);

        // AX=4F01h for the mode this test wants.
        a.movi(DI, MODE_INFO);
        a.movi(CX, VBE_MODE);
        a.movi(AX, 0x4f01);
        a.int(0x10);
        a.movto(Mem::abs(OFF_VBE + 2), AX);

        // AX=4F02h with bit 14: the linear framebuffer.
        a.movi(BX, VBE_MODE | 0x4000);
        a.movi(AX, 0x4f02);
        a.int(0x10);
        a.movto(Mem::abs(OFF_VBE + 4), AX);

        // AX=4F03h: what is in force now.
        a.movi(AX, 0x4f03);
        a.int(0x10);
        a.movto(Mem::abs(OFF_VBE + 6), BX);

        // Four colours, one a bank, in scratch memory where the fill loop can
        // index them.
        for (i, colour) in BANK_COLOURS.iter().enumerate() {
            a.movmi32(Mem::abs(BANK_TABLE + i as u16 * 4), *colour);
        }

        // Draw through the **banked** window, moving it with AX=4F05h: four
        // 64 KiB banks of solid colour, which is 65 536 pixels — the first
        // hundred lines or so of a 640-pixel-wide 32-bit picture.
        a.movi(SI, 0);
        let bank = a.here_label();
        a.mov(DX, SI);
        a.shift(Shift::SHR, DX, 2);
        a.movi(BX, 0);
        a.movi(AX, 0x4f05);
        a.int(0x10);
        a.mov32(AX, Mem::si(i32::from(BANK_TABLE)));
        a.movi(BX, 0xa000);
        a.movsr(ES, BX);
        a.movi(DI, 0);
        a.movi(CX, 0x4000);
        let pixel = a.here_label();
        a.movto32(Mem::di(0).seg(ES), AX);
        a.alui(Alu::ADD, DI, 4);
        a.loop_(pixel);
        a.alui(Alu::ADD, SI, 4);
        a.alui(Alu::CMP, SI, 4 * BANK_COLOURS.len() as u16);
        a.jcc(Cc::B, bank);
    });

    // Every service answered.
    for (at, what) in [
        (OFF_VBE, "4F00h"),
        (OFF_VBE + 2, "4F01h"),
        (OFF_VBE + 4, "4F02h"),
    ] {
        assert_eq!(peek16(&m, at), 0x004f, "{what} did not succeed");
    }
    assert_eq!(
        peek16(&m, OFF_VBE + 6) & 0x01ff,
        VBE_MODE,
        "4F03h does not report the mode 4F02h set"
    );

    // The controller information block: VBE 2.0 §13.1.
    let signature: Vec<u8> = (0..4).map(|i| peek(&m, u64::from(VBE_INFO) + i)).collect();
    assert_eq!(&signature, b"VESA", "the VbeInfoBlock signature");
    assert_eq!(peek16(&m, VBE_INFO + 4), 0x0200, "VBE version 2.0");
    let memory = peek16(&m, VBE_INFO + 0x12);
    assert_eq!(
        u32::from(memory) * 64 * 1024,
        4 * 1024 * 1024,
        "the card's memory, in 64 KiB units, is what the machine file gives it"
    );
    // The mode list, which is a far pointer into the ROM, holds the mode the
    // guest set.
    let list = peek32(&m, VBE_INFO + 0x0e);
    let list_at = ((list >> 16) << 4) + (list & 0xffff);
    let modes: Vec<u16> = (0..16)
        .map(|i| {
            u16::from(peek(&m, u64::from(list_at) + i * 2))
                | (u16::from(peek(&m, u64::from(list_at) + i * 2 + 1)) << 8)
        })
        .take_while(|m| *m != 0xffff)
        .collect();
    assert!(modes.contains(&VBE_MODE), "the mode list is {modes:04x?}");

    // The mode information block: §13.2.
    assert_eq!(peek16(&m, MODE_INFO + 0x12), 640, "XResolution");
    assert_eq!(peek16(&m, MODE_INFO + 0x14), 480, "YResolution");
    assert_eq!(peek(&m, u64::from(MODE_INFO) + 0x19), 32, "BitsPerPixel");
    assert_eq!(peek16(&m, MODE_INFO + 0x10), 640 * 4, "BytesPerScanLine");
    let attributes = peek16(&m, MODE_INFO);
    assert_eq!(
        attributes & 0x99,
        0x99,
        "supported, colour, and a linear one"
    );
    let phys = peek32(&m, MODE_INFO + 0x28);
    assert_ne!(phys, 0, "PhysBasePtr is where the card's BAR0 decodes");

    // The guest painted the first four banks through the window at A000h; the
    // test now draws a line through **PhysBasePtr**, which is the same memory
    // seen through the card's PCI aperture. Both have to be in the frame.
    let mem = m.space("mem").expect("the memory space");
    let line = u64::from(phys) + u64::from(APERTURE_LINE) * 640 * 4;
    for x in 0..640u64 {
        mem.write(line + x * 4, Width::U32, 0x00ff_ffff, MemAttrs::DEFAULT)
            .expect("the linear framebuffer answers at PhysBasePtr");
    }

    let surface = frame(&scanout);
    assert_eq!(
        (surface.width(), surface.height()),
        (640, 480),
        "the linear mode's geometry is the extension registers'"
    );
    // Each bank is 65 536 bytes: 16 384 pixels, 25.6 lines of 640.
    for (bank, colour) in BANK_COLOURS.iter().enumerate() {
        let want = [(colour >> 16) as u8, (colour >> 8) as u8, (*colour) as u8];
        let y = bank as u32 * 25 + 2;
        assert_eq!(
            surface.get(0, y),
            Some(want),
            "bank {bank}, drawn through the window at A000h"
        );
        assert_eq!(surface.get(639, y), Some(want));
    }
    for x in [0u32, 1, 320, 639] {
        assert_eq!(
            surface.get(x, APERTURE_LINE),
            Some([0xff, 0xff, 0xff]),
            "the line the test drew through the aperture"
        );
    }
    assert_eq!(
        surface.get(0, APERTURE_LINE + 1),
        Some([0, 0, 0]),
        "and the mode set cleared everything else"
    );
    screenshot(&surface, "pc-vbe.png");
}
