//! Denise on a board: a 68000 program programs it through `$DFF000`, a test
//! standing in for Agnus pushes a field of lines, and a host captures the
//! picture.
//!
//! `src/dev/amiga/denise/tests.rs` has the pixel-level tests, and they drive the
//! chip directly. What they cannot prove is that any of it survives the machine
//! layer: that `custom = custom` subscribes Denise to the register space so a
//! big-endian `MOVE.W` from a real core reaches the colour table, that
//! `DIWSTRT` is routed to Denise at all (Appendix B says it is Agnus's; the
//! table follows Appendix C), that `ExportId::AMIGA_VIDEO` is what Agnus will
//! find, that a debugger's read of `CLXDAT` through the address space does not
//! clear it, and that `clock = clk / 4` gives the host adapter a frame period.
//!
//! `machines/tests/amiga-denise.machine` is the board. The firmware is
//! synthetic and hand-assembled here; nothing from any Kickstart is involved.

#![cfg(all(feature = "cpu-m68k", feature = "dev-amiga-denise"))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::amiga::custom::CustomBus;
use rsemu::dev::amiga::denise::{Fetch, Line, Video};
use rsemu::host::display::amiga::capture;
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// The board.
const BOARD: &str = include_str!("../machines/tests/amiga-denise.machine");

/// The custom-chip base (Appendix D).
const CUSTOM: u32 = 0x00df_f000;

/// A PAL line in colour clocks, and a field in lines.
const CLOCKS: u16 = 227;
const LINES: u16 = 312;

/// The program: a window, two colours, one plane, a sprite, collisions on.
///
/// Each store is `MOVE.W #imm, (abs).L` — `$33FC imm addr` in the MC68000
/// user's manual's encoding — and the program ends in `BRA *` (`$60FE`).
fn firmware() -> Vec<u8> {
    let stores: &[(u16, u16)] = &[
        (0x08e, 0x2c81), // DIWSTRT
        (0x090, 0x2cc1), // DIWSTOP
        (0x180, 0x0012), // COLOR00
        (0x182, 0x0fe0), // COLOR01
        (0x1a2, 0x0f00), // COLOR17
        (0x100, 0x1200), // BPLCON0: one plane
        (0x104, 0x0008), // BPLCON2: PF2P = 1, sprites 0-1 in front
        (0x098, 0x0041), // CLXCON: plane 1 enabled, match 1
        (0x140, 0x0060), // SPR0POS: HSTART bits 8-1
        (0x142, 0x0001), // SPR0CTL: HSTART bit 0, so HSTART = $C1
        (0x146, 0x0000), // SPR0DATB
        (0x144, 0x8000), // SPR0DATA: arms it
    ];
    let mut code: Vec<u16> = Vec::new();
    for &(offset, value) in stores {
        let addr = CUSTOM + u32::from(offset);
        code.extend([0x33fc, value, (addr >> 16) as u16, addr as u16]);
    }
    code.push(0x60fe);

    let mut image = vec![0u8; 0x400 + 2 * code.len()];
    image[0..4].copy_from_slice(&0x0002_0000u32.to_be_bytes()); // SSP: top of RAM
    image[4..8].copy_from_slice(&0x0000_0400u32.to_be_bytes()); // PC
    for (i, word) in code.iter().enumerate() {
        image[0x400 + 2 * i..0x402 + 2 * i].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// Build the board with the host capture installed, run the program, and hand
/// back the machine, the scanout and the register bus.
fn boot() -> (
    Machine,
    rsemu::host::display::amiga::DeniseScanout,
    Arc<CustomBus>,
) {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    capture::install(&mut options).expect("a capture table");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    let mut machine = rsemu::machine::build("amiga-denise", BOARD, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let scanout = capture::take(&options.realize.hosts, &machine).expect("the board has a Denise");
    // Twelve stores of six words each: well under a millisecond at 7 MHz.
    machine
        .run_for(GlobalTime::from_nanos(2_000_000))
        .expect("it runs");

    let custom = machine.device("custom").expect("a `custom`");
    let bus = Arc::clone(
        custom
            .device()
            .export(ExportId::CUSTOM_BUS)
            .expect("published")
            .opaque()
            .expect("opaque"),
    )
    .downcast::<CustomBus>()
    .expect("a CustomBus");
    (machine, scanout, bus)
}

/// The video handle, found the way Agnus will find it.
fn video(m: &Machine) -> Arc<Video> {
    let denise = m.device("denise").expect("a `denise`");
    Arc::clone(
        denise
            .device()
            .export(ExportId::AMIGA_VIDEO)
            .expect("`amiga.denise` publishes its line input")
            .opaque()
            .expect("an opaque handle"),
    )
    .downcast::<Video>()
    .expect("the handle is a `Video`")
}

/// A field of lines whose plane 1 is `$FF00` in every word from `DDFSTRT $38`
/// — eight pixels on, eight off, from `x = $81` — as Agnus would fetch it.
fn push_field(v: &Video) {
    let words = [0xff00u16; 20];
    for vpos in 0..LINES {
        v.line(&Line {
            vpos,
            clocks: CLOCKS,
            fetch: Fetch {
                start: 0x38,
                planes: [&words, &[], &[], &[], &[], &[], &[], &[]],
            },
        });
    }
    v.field(true);
}

fn read_word(m: &Machine, offset: u32, attrs: MemAttrs) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(CUSTOM + offset), Width::U16, attrs)
        .expect("a mapped word") as u16
}

#[test]
fn the_program_reaches_denise_and_every_store_is_claimed() {
    let (m, scanout, bus) = boot();
    assert!(
        bus.attached()
            .contains(rsemu::dev::amiga::regs::ChipId::DENISE),
        "`custom = custom` subscribed Denise"
    );
    assert_eq!(
        bus.unclaimed(),
        0,
        "all twelve stores are Denise's, DIWSTRT and DIWSTOP included"
    );
    let v = video(&m);
    assert!(
        Arc::ptr_eq(&v, scanout.video()),
        "the host and Agnus hold one chip"
    );
}

#[test]
fn a_pushed_field_is_the_picture_the_program_asked_for() {
    let (m, scanout, _bus) = boot();
    push_field(&video(&m));

    let info = scanout.info();
    assert_eq!((info.width, info.height), (800, 568));
    assert_eq!(scanout.frame_counter(), 1);

    let mut surface = Surface::new(PixelFormat::RGB888, 1, 1);
    assert_eq!(scanout.capture(&mut surface), 1);
    // Column for low-resolution x is 2 × (x − 64); row for line v is 2 × (v − $1D).
    let at = |x: u32, line: u32| surface.get(2 * (x - 64), 2 * (line - 0x1d)).unwrap();
    let border = [0x00, 0x11, 0x22];
    let yellow = [0xff, 0xee, 0x00];
    let red = [0xff, 0x00, 0x00];
    assert_eq!(at(0x80, 100), border, "left of the window");
    assert_eq!(
        at(0x81, 100),
        yellow,
        "the first fetched pixel is the window's first"
    );
    assert_eq!(at(0x89, 100), border, "eight on, eight off");
    assert_eq!(at(0xc0, 100), border, "the last pixel of word 3 is off");
    assert_eq!(
        at(0xc1, 100),
        red,
        "sprite 0, in front of the playfield at PF2P = 1"
    );
    assert_eq!(at(0x81, 0x2b), border, "above the window");
}

#[test]
fn clxdat_through_the_bus_survives_a_debugger_and_clears_for_the_processor() {
    let (m, _scanout, _bus) = boot();
    push_field(&video(&m));
    // Sprite 0 at $C1 sits on a set pixel of plane 1 (word 4 of $FF00 starts
    // there): bit 1, "odd bitplanes to sprite 0"; bit 5, because no even plane is
    // enabled to prevent it; bit 0 likewise for the two playfields.
    assert_eq!(read_word(&m, 0x00e, MemAttrs::DEBUG), 0x0023);
    assert_eq!(
        read_word(&m, 0x00e, MemAttrs::DEBUG),
        0x0023,
        "a debugger clears nothing"
    );
    assert_eq!(read_word(&m, 0x00e, MemAttrs::DEFAULT), 0x0023);
    assert_eq!(
        read_word(&m, 0x00e, MemAttrs::DEFAULT),
        0x0000,
        "read and clear"
    );
}

#[test]
fn the_frame_period_is_a_field_of_colour_clocks_at_the_7m_rate() {
    let (m, scanout, _bus) = boot();
    assert_eq!(scanout.frame_period_ns(), 0, "no field has finished yet");
    push_field(&video(&m));
    // 312 lines × 227 colour clocks × 2 ticks of 7M, at 28375160 / 4 Hz, in
    // exact integer arithmetic: 141648 × 4 × 10⁹ / 28375160.
    let expected = 141_648u64 * 4 * 1_000_000_000 / 28_375_160;
    assert_eq!(scanout.frame_period_ns(), expected);
    assert!(
        (19_900_000..20_000_000).contains(&expected),
        "a PAL field is about 20 ms"
    );
}

#[test]
fn two_boots_render_byte_identical_fields() {
    let hash = || {
        let (m, scanout, _bus) = boot();
        push_field(&video(&m));
        let mut surface = Surface::new(PixelFormat::RGB888, 1, 1);
        scanout.capture(&mut surface);
        surface
            .pixels()
            .iter()
            .fold(0xcbf2_9ce4_8422_2325u64, |h, b| {
                (h ^ u64::from(*b)).wrapping_mul(0x0000_0100_0000_01b3)
            })
    };
    assert_eq!(hash(), hash());
}
