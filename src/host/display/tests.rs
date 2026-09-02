//! Tests for the scanout seam and, where the features allow it, for a whole
//! machine drawing a picture.

use super::*;

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------

#[test]
fn a_pixel_survives_every_format() {
    for format in [
        PixelFormat::RGBA8888,
        PixelFormat::BGRA8888,
        PixelFormat::RGB888,
    ] {
        let mut surface = Surface::new(format, 3, 2);
        surface.put(2, 1, [0x12, 0x34, 0x56]);
        assert_eq!(
            surface.get(2, 1),
            Some([0x12, 0x34, 0x56]),
            "{format} lost a pixel"
        );
        assert_eq!(surface.get(0, 0), Some([0, 0, 0]), "{format} smeared");
        assert_eq!(surface.stride(), 3 * format.bytes_per_pixel());
        assert_eq!(surface.len(), surface.stride() * 2);
    }
}

#[test]
fn the_byte_order_is_the_one_the_name_promises() {
    let mut rgba = Surface::new(PixelFormat::RGBA8888, 1, 1);
    rgba.put(0, 0, [1, 2, 3]);
    assert_eq!(rgba.pixels(), &[1, 2, 3, 0xff]);

    let mut bgra = Surface::new(PixelFormat::BGRA8888, 1, 1);
    bgra.put(0, 0, [1, 2, 3]);
    assert_eq!(bgra.pixels(), &[3, 2, 1, 0xff]);

    let mut rgb = Surface::new(PixelFormat::RGB888, 1, 1);
    rgb.put(0, 0, [1, 2, 3]);
    assert_eq!(rgb.pixels(), &[1, 2, 3]);
}

#[test]
fn a_pixel_outside_the_surface_is_ignored_rather_than_fatal() {
    let mut surface = Surface::new(PixelFormat::RGBA8888, 2, 2);
    surface.put(2, 0, [0xff, 0xff, 0xff]);
    surface.put(0, 2, [0xff, 0xff, 0xff]);
    surface.put(u32::MAX, u32::MAX, [0xff, 0xff, 0xff]);
    assert_eq!(surface.get(9, 9), None);
    assert!(surface.pixels().iter().all(|b| *b == 0));
}

#[test]
fn fill_paints_every_pixel_opaque() {
    let mut surface = Surface::new(PixelFormat::RGBA8888, 4, 4);
    surface.fill([10, 20, 30]);
    for y in 0..4 {
        for x in 0..4 {
            assert_eq!(surface.get(x, y), Some([10, 20, 30]));
        }
    }
    assert!(
        surface
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .all(|p| p[3] == 0xff)
    );
}

#[test]
fn reshaping_to_the_same_shape_leaves_the_buffer_alone() {
    let mut surface = Surface::new(PixelFormat::RGBA8888, 8, 8);
    surface.fill([1, 2, 3]);
    let before = surface.as_ptr();
    surface.reshape(PixelFormat::RGBA8888, 8, 8);
    assert_eq!(surface.as_ptr(), before, "a no-op reshape reallocated");
    assert_eq!(surface.get(7, 7), Some([1, 2, 3]));

    surface.reshape(PixelFormat::RGB888, 4, 4);
    assert_eq!(surface.len(), 4 * 4 * 3);
    assert_eq!(surface.format(), PixelFormat::RGB888);
}

#[test]
fn the_frame_hash_follows_the_pixels() {
    let mut a = Surface::new(PixelFormat::RGBA8888, 4, 4);
    let b = Surface::new(PixelFormat::RGBA8888, 4, 4);
    assert_eq!(a.hash(), b.hash());
    a.put(3, 3, [0, 0, 1]);
    assert_ne!(a.hash(), b.hash(), "one changed pixel must move the hash");
    // An empty surface still hashes to the FNV offset basis, so "no picture"
    // and "hash not computed" stay distinguishable — same argument as
    // `Machine::state_hash`.
    assert_eq!(
        Surface::new(PixelFormat::RGBA8888, 0, 0).hash(),
        0xcbf2_9ce4_8422_2325
    );
}

#[test]
fn a_row_is_stride_bytes_and_stops_at_the_bottom() {
    let surface = Surface::new(PixelFormat::RGB888, 5, 3);
    assert_eq!(surface.row(2).map(<[u8]>::len), Some(15));
    assert!(surface.row(3).is_none());
}

// ---------------------------------------------------------------------------
// The NES adapter
// ---------------------------------------------------------------------------

/// A 2C02 with nothing but a bus, advanced by hand — the smallest thing that
/// produces a frame, and enough to prove the conversion.
///
/// Rendering is off, so every dot shows the backdrop (`$3F00`), which is what
/// makes the expected picture a single known colour rather than a hash nobody
/// can check by eye.
#[cfg(feature = "dev-nes-ppu")]
#[test]
fn a_ppu_frame_becomes_host_pixels() {
    use alloc::sync::Arc;

    use crate::core::props::Props;
    use crate::core::space::{AddressSpace, RamStore, Region as MmioRegion, UnassignedPolicy};
    use crate::dev::ppu::{DOTS_PER_FRAME, NesPpu};
    use crate::host::display::nes::NesScanout;
    use crate::host::display::palette::nes_rgb;

    let vram = AddressSpace::new("ppu", 14).with_unassigned(UnassignedPolicy::ONES);
    {
        let mut topo = vram.topology();
        topo.map(
            Arc::new(MmioRegion::ram("chr", Arc::new(RamStore::new(0x2000)))),
            0x0000,
        )
        .expect("chr maps");
        topo.map(
            Arc::new(MmioRegion::ram(
                "nametables",
                Arc::new(RamStore::new(0x1000)),
            )),
            0x2000,
        )
        .expect("nametables map");
    }

    let ppu = Arc::new(NesPpu::new(&Props::new().with("region", "ntsc")).expect("a ppu"));
    ppu.attach_bus(Arc::new(vram));
    // $21 is the sky blue every NES game's background is, and it is a colour
    // nobody could mistake for "the buffer was never written".
    ppu.poke_palette(0x3f00, 0x21);

    let scanout = NesScanout::new(Arc::clone(&ppu));
    let mut surface = Surface::for_scanout(&scanout);
    assert_eq!((surface.width(), surface.height()), (256, 240));
    assert_eq!(scanout.frame_counter(), 0);

    ppu.advance_by(DOTS_PER_FRAME);
    let serial = scanout.capture(&mut surface);
    assert!(serial >= 1, "a frame's worth of dots produced no frame");
    assert_eq!(surface.serial(), serial);

    let sky = nes_rgb(0x21);
    assert_eq!(surface.get(0, 0), Some(sky));
    assert_eq!(surface.get(255, 239), Some(sky));
    assert_eq!(surface.get(128, 120), Some(sky));

    // The frame period is exact integer arithmetic from the oscillator forest:
    // 89342 dots x 4 master ticks x 11/236250000 s = 16.639 ms.
    assert_eq!(scanout.frame_period_ns(), 16_639_356);
}

/// The same conversion, in a format a host window wants rather than a canvas.
#[cfg(feature = "dev-nes-ppu")]
#[test]
fn a_host_can_ask_for_a_different_byte_order() {
    use alloc::sync::Arc;

    use crate::core::props::Props;
    use crate::dev::ppu::{DOTS_PER_FRAME, NesPpu};
    use crate::host::display::nes::NesScanout;
    use crate::host::display::palette::nes_rgb;

    let ppu = Arc::new(NesPpu::new(&Props::new()).expect("a ppu"));
    ppu.poke_palette(0x3f00, 0x16);
    ppu.advance_by(DOTS_PER_FRAME);

    let scanout = NesScanout::new(ppu);
    let mut surface = Surface::new(PixelFormat::BGRA8888, 1, 1);
    scanout.capture(&mut surface);
    assert_eq!((surface.width(), surface.height()), (256, 240));

    let red = nes_rgb(0x16);
    assert_eq!(surface.get(10, 10), Some(red));
    assert_eq!(&surface.pixels()[..4], &[red[2], red[1], red[0], 0xff]);
}

// ---------------------------------------------------------------------------
// A whole machine, headless
// ---------------------------------------------------------------------------

/// The machine-level regression `ROADMAP.md` §12 asks for: build a NES from its
/// description, run it for a fixed number of virtual frames, capture the
/// picture, hash it, and write a PNG.
///
/// The cartridge is a minimal NROM whose reset vector is `JMP $C000`, so the
/// CPU is deterministic and does nothing to the PPU; the backdrop is poked
/// directly. What is under test is the whole path — description, realize,
/// scheduler, the lazily-advanced PPU, the scanout seam, the palette and the
/// encoder — not the picture a game would draw.
#[cfg(all(
    feature = "machine-nes",
    feature = "dev-nes-ppu",
    feature = "display-png",
    feature = "std"
))]
#[test]
fn a_nes_boots_renders_and_captures_a_png() {
    use crate::core::clock::GlobalTime;

    use crate::host::display::palette::nes_rgb;
    use crate::host::display::{nes, png};
    use crate::machine::catalog;

    /// 16 KiB of PRG, 8 KiB of CHR, and `JMP $C000` at the reset vector.
    static MINIMAL_NROM: &[u8] = &{
        let mut image = [0u8; 16 + 16384 + 8192];
        image[0] = b'N';
        image[1] = b'E';
        image[2] = b'S';
        image[3] = 0x1a;
        image[4] = 1;
        image[5] = 1;
        image[16 + 0x3ffc] = 0x00;
        image[16 + 0x3ffd] = 0xc0;
        image[16] = 0x4c;
        image[17] = 0x00;
        image[18] = 0xc0;
        image
    };

    let entry = catalog::machine("nes-ntsc").expect("machine-nes is on");
    let registry = catalog::registry().expect("a registry");
    let mut options = catalog::build_options().expect("build options");
    options.realize.media.insert("cart", MINIMAL_NROM);

    // Take a handle on the PPU as it is constructed; see `nes::capture`.
    nes::capture::install(&mut options).expect("the bindings are intercepted");
    let mut machine = crate::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the nes-ntsc description builds");
    let scanout = nes::capture::take(&options.realize.hosts).expect("the machine has a ppu");

    scanout.ppu().poke_palette(0x3f00, 0x21);

    // Three frames of virtual time, exactly. Deterministic: the same span
    // always produces the same picture, on any host.
    let frames = 3;
    let period = scanout.frame_period_ns();
    assert!(period > 0);
    machine
        .run_for(GlobalTime::from_nanos(period * frames))
        .expect("the machine runs");

    let mut surface = Surface::for_scanout(&scanout);
    let serial = scanout.capture(&mut surface);
    assert!(
        serial >= frames - 1,
        "only {serial} frames after {frames} frame periods; is the ppu being advanced?"
    );

    // The picture: the backdrop, everywhere, because rendering was never
    // enabled. A flat frame is a weak assertion on its own, which is why the
    // hash below is the actual regression.
    let sky = nes_rgb(0x21);
    assert_eq!(surface.get(0, 0), Some(sky));
    assert_eq!(surface.get(200, 200), Some(sky));

    // The frame hash. It changes when the picture changes, which is the point;
    // when it does, look at the PNG this test writes before assuming the hash
    // is wrong.
    let expected: u64 = {
        let mut reference = Surface::new(surface.format(), surface.width(), surface.height());
        reference.fill(sky);
        reference.hash()
    };
    assert_eq!(
        surface.hash(),
        expected,
        "the frame is not the flat backdrop it should be"
    );

    // And it encodes. Round-tripped through the decoder rather than trusted:
    // an eight-byte magic number proves nothing about the pixels.
    let bytes = png::encode(&surface).expect("the surface encodes");
    assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
    let decoded = oxideav_png::decode_png(&bytes).expect("what we wrote decodes");
    assert_eq!((decoded.width, decoded.height), (256, 240));
    assert_eq!(&decoded.data[..3], &sky[..]);

    // Written where a human or a CI job can look at it. `RSEMU_SCREENSHOT_DIR`
    // aims it somewhere durable; the temporary directory is the default so the
    // test never needs a writable repository.
    let dir = std::env::var("RSEMU_SCREENSHOT_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir());
    let path = dir.join("rsemu-nes-ntsc.png");
    std::fs::write(&path, &bytes).expect("the screenshot is written");
    println!(
        "wrote {} ({} bytes), frame {serial}, hash {:#018x}",
        path.display(),
        bytes.len(),
        surface.hash()
    );

    // An APNG of the same run, to prove the animated path is wired up too.
    let animation = png::encode_animation(&[surface.clone(), surface], 2).expect("apng encodes");
    assert_eq!(&animation[..8], b"\x89PNG\r\n\x1a\n");
    assert!(animation.len() > bytes.len() / 2);
}

/// The same path with a real cartridge in it, which is the only way to see a
/// picture somebody drew rather than one this test poked.
///
/// Gated on `RSEMU_NES_TEST_ROM` like every other corpus (`CLAUDE.md`): point
/// it at an iNES image. It writes a PNG per captured frame and an APNG of the
/// lot, which is how `docs/` gets screenshots that are regenerated rather than
/// drawn. Without the variable it passes trivially, so `cargo test` offline
/// stays green.
#[cfg(all(
    feature = "machine-nes",
    feature = "dev-nes-ppu",
    feature = "display-png",
    feature = "std"
))]
#[test]
fn a_real_cartridge_draws_a_picture() {
    use crate::core::clock::GlobalTime;

    use crate::host::display::{nes, png};
    use crate::machine::catalog;

    let Ok(rom) = std::env::var("RSEMU_NES_TEST_ROM") else {
        println!("SKIP: set RSEMU_NES_TEST_ROM to an iNES image to run this");
        return;
    };
    let image = std::fs::read(&rom).expect("RSEMU_NES_TEST_ROM is readable");

    let entry = catalog::machine("nes-ntsc").expect("machine-nes is on");
    let registry = catalog::registry().expect("a registry");
    let mut options = catalog::build_options().expect("build options");
    options.realize.media.insert("cart", image.as_slice());
    nes::capture::install(&mut options).expect("the bindings are intercepted");
    let mut machine = crate::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{rom}: {e}"));
    let scanout = nes::capture::take(&options.realize.hosts).expect("the machine has a ppu");

    // Long enough for a title screen to have been drawn: a few seconds of
    // virtual time, one frame at a time so the captures are real frames.
    let period = scanout.frame_period_ns();
    let mut surface = Surface::for_scanout(&scanout);
    let mut frames = Vec::new();
    let dir = std::env::var("RSEMU_SCREENSHOT_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir());

    for frame in 0..180u64 {
        machine
            .run_for(GlobalTime::from_nanos(period))
            .expect("the machine runs");
        // Every thirtieth frame: half a second apart, which is enough to see a
        // title screen animate without writing 180 files.
        if frame % 30 == 29 {
            scanout.capture(&mut surface);
            let bytes = png::encode(&surface).expect("the surface encodes");
            let path = dir.join(format!("rsemu-nes-frame{frame:03}.png"));
            std::fs::write(&path, &bytes).expect("the screenshot is written");
            println!(
                "wrote {} ({} bytes), hash {:#018x}",
                path.display(),
                bytes.len(),
                surface.hash()
            );
            frames.push(surface.clone());
        }
    }

    assert!(scanout.frame_counter() >= 179, "the ppu is not advancing");
    let animation = png::encode_animation(&frames, 50).expect("apng encodes");
    let path = dir.join("rsemu-nes-frames.png");
    std::fs::write(&path, &animation).expect("the animation is written");
    println!("wrote {} ({} bytes)", path.display(), animation.len());
}

// ---------------------------------------------------------------------------
// The Game Boy adapter
// ---------------------------------------------------------------------------

/// The four levels the panel drives, evenly spaced, lightest first.
#[cfg(feature = "dev-gb")]
#[test]
fn the_four_shades_are_grey_and_evenly_spaced() {
    use crate::host::display::gb::gb_rgb;

    assert_eq!(gb_rgb(0), [0xff, 0xff, 0xff]);
    assert_eq!(gb_rgb(1), [0xaa, 0xaa, 0xaa]);
    assert_eq!(gb_rgb(2), [0x55, 0x55, 0x55]);
    assert_eq!(gb_rgb(3), [0x00, 0x00, 0x00]);
    // The framebuffer cannot hold anything else, but a host must not be able to
    // panic this adapter by handing it one.
    assert_eq!(gb_rgb(0xff), [0x00, 0x00, 0x00]);
}

/// A DMG's LCD controller with nothing but its own video RAM, advanced by hand
/// — the smallest thing that produces a frame, and enough to prove the
/// conversion.
///
/// The tile is drawn so that all four shades appear in every eight rows, which
/// is what makes the assertion "this is the picture" rather than "the buffer
/// was written at all".
#[cfg(feature = "dev-gb")]
#[test]
fn a_game_boy_frame_becomes_host_pixels() {
    use alloc::sync::Arc;

    use crate::dev::gb::ppu::{DOTS_PER_FRAME, GbPpu, lcdc};
    use crate::host::display::gb::{GbScanout, gb_rgb};

    let ppu = Arc::new(GbPpu::new());

    // Tile 0 at $8000: eight rows, each a solid run of one colour index. Low
    // plane is the first byte of a row and high plane the second (Pan Docs,
    // "VRAM Tile Data"), so `(0,0)`, `(ff,00)`, `(00,ff)`, `(ff,ff)` are colour
    // indices 0, 1, 2, 3.
    const ROWS: [(u8, u8); 4] = [(0x00, 0x00), (0xff, 0x00), (0x00, 0xff), (0xff, 0xff)];
    for row in 0..8u64 {
        let (low, high) = ROWS[(row % 4) as usize];
        ppu.poke_vram(row * 2, low);
        ppu.poke_vram(row * 2 + 1, high);
    }
    // The whole map is tile 0; $9800 is offset $1800 within video RAM.
    for cell in 0..0x400u64 {
        ppu.poke_vram(0x1800 + cell, 0);
    }
    // BGP $E4: index n becomes shade n, which is the identity every test ROM
    // uses so that a palette bug cannot hide behind it.
    ppu.write_register(7, 0xe4);
    ppu.write_register(0, lcdc::LCD_ENABLE | lcdc::BG_ENABLE | lcdc::TILE_DATA);

    let scanout = GbScanout::new(Arc::clone(&ppu));
    let mut surface = Surface::for_scanout(&scanout);
    assert_eq!((surface.width(), surface.height()), (160, 144));
    assert_eq!(scanout.frame_counter(), 0);

    // Two frames: the first is cut short by the four dots the controller skips
    // when the LCD comes on mid-line, so the second is the whole picture.
    ppu.advance_by(DOTS_PER_FRAME * 2);
    let serial = scanout.capture(&mut surface);
    assert!(serial >= 1, "two frames' worth of dots produced no frame");
    assert_eq!(surface.serial(), serial);

    for (y, shade) in [(0, 0u8), (1, 1), (2, 2), (3, 3), (4, 0)] {
        assert_eq!(
            surface.get(80, y),
            Some(gb_rgb(shade)),
            "line {y} is not shade {shade}"
        );
    }

    // 70224 dots at 4.194304 MHz, exactly.
    assert_eq!(scanout.frame_period_ns(), 16_742_706);
}

/// The same conversion in a format a host window wants rather than a canvas —
/// the property the wasm module leans on, since it pins `RGBA8888` whatever
/// `info` prefers and expects every adapter to convert on capture.
#[cfg(feature = "dev-gb")]
#[test]
fn a_game_boy_converts_to_whatever_the_host_asked_for() {
    use alloc::sync::Arc;

    use crate::dev::gb::ppu::{DOTS_PER_FRAME, GbPpu, lcdc};
    use crate::host::display::gb::GbScanout;

    let ppu = Arc::new(GbPpu::new());
    // Every pixel colour index 3, which BGP $E4 leaves as shade 3: black, and
    // the one shade a never-written buffer could not be mistaken for.
    for row in 0..8u64 {
        ppu.poke_vram(row * 2, 0xff);
        ppu.poke_vram(row * 2 + 1, 0xff);
    }
    ppu.write_register(7, 0xe4);
    ppu.write_register(0, lcdc::LCD_ENABLE | lcdc::BG_ENABLE | lcdc::TILE_DATA);
    ppu.advance_by(DOTS_PER_FRAME * 2);

    let scanout = GbScanout::new(ppu);
    // A one-pixel surface in the wrong format: `capture` reshapes it, so a host
    // never has to know the geometry in advance.
    let mut surface = Surface::new(PixelFormat::BGRA8888, 1, 1);
    scanout.capture(&mut surface);
    assert_eq!((surface.width(), surface.height()), (160, 144));
    assert_eq!(surface.get(10, 10), Some([0, 0, 0]));
    assert_eq!(&surface.pixels()[..4], &[0, 0, 0, 0xff]);

    let mut rgb = Surface::new(PixelFormat::RGB888, 1, 1);
    scanout.capture(&mut rgb);
    assert_eq!(rgb.len(), 160 * 144 * 3);
    assert_eq!(rgb.get(10, 10), Some([0, 0, 0]));
}

/// A whole Game Boy, described by `machines/gameboy.machine`, drawing a picture
/// a host can capture — the path `rsemu run gameboy --screenshot` takes and the
/// one the browser takes, end to end.
///
/// The cartridge is generated here, never vendored (`CLAUDE.md`): a two-bank
/// ROM whose program fills tile data and the tile map, turns the LCD on and
/// then scrolls, so the picture is busy and no two frames are identical.
#[cfg(all(feature = "machine-gameboy", feature = "std"))]
#[test]
fn a_game_boy_boots_from_its_description_and_draws() {
    use alloc::collections::BTreeSet;

    use crate::core::clock::GlobalTime;
    use crate::host::display::gb;
    use crate::machine::catalog;

    /// `LD HL,$8000` … fill 256 bytes of tile data, fill the map, set BGP,
    /// LCDC on, then scroll `SCY` forever. Hand-assembled from Pan Docs'
    /// register map and the SM83 opcode table.
    const PROGRAM: &[u8] = &[
        0x21, 0x00, 0x80, // LD   HL,$8000
        0x0e, 0x00, //       LD   C,$00
        0x3e, 0x00, //       LD   A,$00
        0x77, //             LD   (HL),A
        0x3c, //             INC  A
        0x23, //             INC  HL
        0x0d, //             DEC  C
        0x20, 0xfa, //       JR   NZ,-6
        0x21, 0x00, 0x98, // LD   HL,$9800
        0x0e, 0x00, //       LD   C,$00
        0x3e, 0x00, //       LD   A,$00
        0x77, //             LD   (HL),A
        0x3c, //             INC  A
        0x23, //             INC  HL
        0x0d, //             DEC  C
        0x20, 0xfa, //       JR   NZ,-6
        0x3e, 0xe4, //       LD   A,$e4
        0xe0, 0x47, //       LDH  ($47),A     BGP
        0x3e, 0x91, //       LD   A,$91
        0xe0, 0x40, //       LDH  ($40),A     LCDC
        0xf0, 0x42, //       LDH  A,($42)     SCY
        0x3c, //             INC  A
        0xe0, 0x42, //       LDH  ($42),A
        0x18, 0xf9, //       JR   -7
    ];

    let image = crate::dev::gb::cart::synthetic_image(2, 0x00, 0x00, PROGRAM);
    let entry = catalog::machine("gameboy").expect("machine-gameboy is on");
    let registry = catalog::registry().expect("a registry");
    let mut options = catalog::build_options().expect("build options");
    options.realize.media.insert("cart", image.as_slice());
    assert!(
        gb::capture::take(&options.realize.hosts).is_none(),
        "nothing has been built yet"
    );
    gb::capture::install(&mut options).expect("the bindings are intercepted");
    let mut machine = crate::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the gameboy description builds");
    let scanout = gb::capture::take(&options.realize.hosts).expect("the machine has an LCD");

    let period = scanout.frame_period_ns();
    assert_eq!(period, 16_742_706);
    // Twenty frames: past the two 256-byte fills and well into the scroll.
    machine
        .run_for(GlobalTime::from_nanos(period * 20))
        .expect("the machine runs");

    let mut surface = Surface::for_scanout(&scanout);
    let serial = scanout.capture(&mut surface);
    assert!(serial >= 19, "only {serial} frames after 20 frame periods");

    let shades: BTreeSet<[u8; 4]> = surface
        .pixels()
        .as_chunks::<4>()
        .0
        .iter()
        .copied()
        .collect();
    assert_eq!(
        shades.len(),
        4,
        "a DMG has four shades and this ROM draws all of them; got {}",
        shades.len()
    );
    assert!(
        surface
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .all(|p| p[3] == 0xff),
        "an ImageData built over this would be transparent"
    );

    // And it keeps drawing something new: the program scrolls every frame.
    let before = surface.hash();
    machine
        .run_for(GlobalTime::from_nanos(period))
        .expect("the machine runs");
    scanout.capture(&mut surface);
    assert_ne!(before, surface.hash(), "the picture stopped changing");

    // **Looking must not change what is looked at.** The same argument
    // `host::audio` makes for sound: a capture reads the device's framebuffer
    // and its frame counter and touches nothing else, so a run that was watched
    // has to land exactly where an unwatched one does. If this ever fails, the
    // adapter is advancing the controller rather than observing it.
    //
    // The unwatched run is driven with the *same two calls* rather than one of
    // 21 frames, and that is not laziness: `GlobalTime::from_nanos` rounds down
    // to 2⁻⁶⁴ seconds, so `from_nanos(20p) + from_nanos(p)` is a few units short
    // of `from_nanos(21p)` and the two runs would have different deadlines.
    // `tests/run_for_additive.rs` measures additivity and says so at length;
    // what is under test here is the capture, so everything else is held equal.
    let watched = machine.state_hash().expect("a state hash");
    let mut options = catalog::build_options().expect("build options");
    options.realize.media.insert("cart", image.as_slice());
    let mut blind = crate::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the gameboy description builds");
    blind
        .run_for(GlobalTime::from_nanos(period * 20))
        .expect("the machine runs");
    blind
        .run_for(GlobalTime::from_nanos(period))
        .expect("the machine runs");
    assert_eq!(
        watched,
        blind.state_hash().expect("a state hash"),
        "capturing the picture moved the machine"
    );
}
