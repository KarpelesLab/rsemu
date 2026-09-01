//! The Master System boards, end to end.
//!
//! A unit test can say "the VDP latched a register". This says something
//! stronger: a machine **described entirely by a `.machine` file** realizes,
//! resets a Z80 to `$0000`, runs a program out of a banked cartridge, reaches
//! the video chip and the sound chip through a *second address space*, takes an
//! interrupt from a wire, and takes a non-maskable one from a button.
//!
//! Four things here are worth proving at this level and nowhere else:
//!
//! * the **port space** is a real second space, and `OUT` lands in it;
//! * the incomplete decode is real — the same chip answers at 256 different
//!   16-bit port addresses, because a Z80 puts a register on the high half;
//! * a **bank switch** performed by the guest changes what the guest fetches,
//!   which is the `AddressSpace::rebase` path all the way through;
//! * `$0000`-`$03FF` does **not** move when it does.
//!
//! Everything here needs a machine, so the whole file is gated on `machine-sms`.

#![cfg(feature = "machine-sms")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::display::sms::{capture, frame_hash};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// Serialises the tests that share the process-wide capture table.
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Where the console's work RAM starts.
const RAM: u64 = 0xc000;

/// How many 16 KiB banks the test image has.
const BANKS: usize = 4;

/// A cartridge image, hand-assembled from Zilog's UM0080 opcode table.
///
/// ```text
///   0000: f3           di
///   0001: 31 f0 df     ld   sp, $dff0        ; a stack below the mapper regs
///   0004: 3e 04        ld   a, $04
///   0006: d3 bf        out  ($bf), a         ; VDP register 0 = mode 4
///   0008: 3e 80        ld   a, $80
///   000a: d3 bf        out  ($bf), a
///   000c: 3e 60        ld   a, $60
///   000e: d3 bf        out  ($bf), a         ; register 1 = display on + IRQ
///   0010: 3e 81        ld   a, $81
///   0012: d3 bf        out  ($bf), a
///   0014: 3e 02        ld   a, $02
///   0016: 32 fe ff     ld   ($fffe), a       ; bank 2 into slot 1
///   0019: 3a 00 40     ld   a, ($4000)       ; and read what appeared there
///   001c: 32 00 c0     ld   ($c000), a
///   001f: 3e 03        ld   a, $03
///   0021: 32 fd ff     ld   ($fffd), a       ; bank 3 into slot 0
///   0024: 3a 00 00     ld   a, ($0000)       ; the fixed kilobyte: still $f3
///   0027: 32 01 c0     ld   ($c001), a
///   002a: 3a 00 04     ld   a, ($0400)       ; just past it: bank 3
///   002d: 32 02 c0     ld   ($c002), a
///   0030: 3e 41        ld   a, $41
///   0032: d3 fd        out  ($fd), a         ; 'A' to the debug console
///   0034: c3 00 01     jp   $0100            ; over the NMI vector, to the drawing
///
///   0066: 3e 5a        ld   a, $5a           ; the NMI vector: Pause
///   0068: 32 03 c0     ld   ($c003), a
///   006b: ed 45        retn
/// ```
///
/// And the part that puts something on screen, entirely through the two VDP
/// ports — no host poking, so what the frame hash covers is the whole chain from
/// `OUT` to pixel:
///
/// ```text
///   0100: 3e ff        ld   a, $ff
///   0102: d3 bf        out  ($bf), a         ; register 2 = $FF: name table $3800
///   0104: 3e 82        ld   a, $82
///   0106: d3 bf        out  ($bf), a
///   0108: 3e 00        ld   a, $00           ; CRAM address 0, code 3
///   010a: d3 bf        out  ($bf), a
///   010c: 3e c0        ld   a, $c0
///   010e: d3 bf        out  ($bf), a
///   0110: af           xor  a
///   0111: d3 be        out  ($be), a         ; colour 0 = black
///   0113: 3e 3f        ld   a, $3f
///   0115: d3 be        out  ($be), a         ; colour 1 = white
///   0117: 3e 20        ld   a, $20           ; VRAM address $0020, code 1:
///   0119: d3 bf        out  ($bf), a         ; tile 1's pattern
///   011b: 3e 40        ld   a, $40
///   011d: d3 bf        out  ($bf), a
///   011f: 06 08        ld   b, 8
///   0121: 3e ff        ld   a, $ff           ; bitplane 0 solid ...
///   0123: d3 be        out  ($be), a
///   0125: af           xor  a                ; ... and the other three clear,
///   0126: d3 be        out  ($be), a         ; so every pixel is colour 1
///   0128: d3 be        out  ($be), a
///   012a: d3 be        out  ($be), a
///   012c: 10 f3        djnz $0121
///   012e: af           xor  a                ; name table address $3800, code 1
///   012f: d3 bf        out  ($bf), a
///   0131: 3e 78        ld   a, $78
///   0133: d3 bf        out  ($bf), a
///   0135: 3e 01        ld   a, $01
///   0137: d3 be        out  ($be), a         ; tile 1 at (0,0)
///   0139: af           xor  a
///   013a: d3 be        out  ($be), a
///   013c: 18 fe        jr   $                ; park
/// ```
fn image() -> Vec<u8> {
    let mut rom = vec![0xffu8; 0x4000 * BANKS];
    let program: &[u8] = &[
        0xf3, // di
        0x31, 0xf0, 0xdf, // ld sp, $dff0
        0x3e, 0x04, 0xd3, 0xbf, // out ($bf), $04
        0x3e, 0x80, 0xd3, 0xbf, // out ($bf), $80
        0x3e, 0x60, 0xd3, 0xbf, // out ($bf), $60
        0x3e, 0x81, 0xd3, 0xbf, // out ($bf), $81
        0x3e, 0x02, 0x32, 0xfe, 0xff, // ld ($fffe), $02
        0x3a, 0x00, 0x40, 0x32, 0x00, 0xc0, // ld a,($4000); ld ($c000),a
        0x3e, 0x03, 0x32, 0xfd, 0xff, // ld ($fffd), $03
        0x3a, 0x00, 0x00, 0x32, 0x01, 0xc0, // ld a,($0000); ld ($c001),a
        0x3a, 0x00, 0x04, 0x32, 0x02, 0xc0, // ld a,($0400); ld ($c002),a
        0x3e, 0x41, 0xd3, 0xfd, // out ($fd), 'A'
        0xc3, 0x00, 0x01, // jp $0100
    ];
    let drawing: &[u8] = &[
        0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf, // R2 = $FF
        0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0xc0, 0xd3, 0xbf, // CRAM address 0
        0xaf, 0xd3, 0xbe, // colour 0 = black
        0x3e, 0x3f, 0xd3, 0xbe, // colour 1 = white
        0x3e, 0x20, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf, // VRAM $0020, write
        0x06, 0x08, // ld b, 8
        0x3e, 0xff, 0xd3, 0xbe, // plane 0 solid
        0xaf, 0xd3, 0xbe, 0xd3, 0xbe, 0xd3, 0xbe, // planes 1-3 clear
        0x10, 0xf3, // djnz
        0xaf, 0xd3, 0xbf, 0x3e, 0x78, 0xd3, 0xbf, // VRAM $3800, write
        0x3e, 0x01, 0xd3, 0xbe, // tile 1
        0xaf, 0xd3, 0xbe, // and its high byte
        0x18, 0xfe, // jr $
    ];
    rom[..program.len()].copy_from_slice(program);
    rom[0x0100..0x0100 + drawing.len()].copy_from_slice(drawing);
    rom[0x0066..0x006b + 2].copy_from_slice(&[
        0x3e, 0x5a, // ld a, $5a
        0x32, 0x03, 0xc0, // ld ($c003), a
        0xed, 0x45, // retn
    ]);
    // A distinctive first byte in every bank, so a slot's contents name their
    // own bank. Bank 0's is the `di` above, which is fine: nothing reads it.
    for bank in 1..BANKS {
        rom[bank * 0x4000] = 0xb0 + bank as u8;
    }
    // And one just past the fixed kilobyte of each bank, for the slot-0 test.
    for bank in 0..BANKS {
        rom[bank * 0x4000 + 0x400] = 0xc0 + bank as u8;
    }
    rom
}

/// Build a board out of the catalog with the test image in its `cart` slot.
///
/// The bindings are intercepted first, because a machine hands back
/// `Arc<dyn Device>` and there is no route from one of those to an `Arc<SmsIo>`
/// - see `host::display::sms::capture`, which is the seam that exists for it.
fn boot(name: &str) -> Machine {
    let entry = catalog::machine(name).unwrap_or_else(|| panic!("this build ships {name}"));
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    capture::clear();
    capture::install(&mut options).expect("the bindings can be intercepted");
    options.realize.media.insert("cart", image());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("{name} does not realize: {e}"),
    }
}

/// Read one byte of a named space, without disturbing anything.
fn peek(m: &Machine, space: &str, addr: u64) -> u64 {
    m.space(space)
        .unwrap_or_else(|| panic!("the machine has no space called `{space}`"))
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped byte")
}

#[test]
fn both_regions_realize_with_the_core_bound_to_both_of_their_spaces() {
    let _guard = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    for name in ["sms-ntsc", "sms-pal"] {
        let m = boot(name);
        assert_eq!(m.name(), name);
        for path in ["cpu", "wram", "cart", "vdp", "psg", "io", "sdsc"] {
            assert!(
                m.device(path).is_some(),
                "{name} has no instance called `{path}`"
            );
        }
        assert!(m.space("mem").is_some());
        assert!(
            m.space("port").is_some(),
            "{name}: the I/O space is a space, not a window into memory"
        );
    }
}

#[test]
fn a_program_reaches_every_chip_through_two_address_spaces_and_banks_its_own_rom() {
    // One machine, several claims, sharing one build because every claim below
    // is about the same run. (Realizing this board used to be the expensive
    // part: the port map is 1280 mappings and `TopologyGuard` reflattened on
    // each one. It flattens once per guard now.)
    let _guard = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut m = boot("sms-ntsc");
    let vdp = capture::take_vdp().expect("the machine has a VDP");
    let console = capture::take_console().expect("the machine has a debug console");

    // -- the memory map, before anything runs ------------------------------
    assert_eq!(peek(&m, "mem", 0x0000), 0xf3, "the reset vector");
    assert_eq!(peek(&m, "mem", 0x4000), 0xb1, "slot 1 powers on at bank 1");
    assert_eq!(peek(&m, "mem", 0x8000), 0xb2, "and slot 2 at bank 2");

    // -- the port space is a real second space, incompletely decoded --------
    //
    // The V counter is at $7E, and a Z80 reaches it as `in a,($7e)` — which
    // puts A on A8-A15. Nothing on the board decodes those lines, so all 256
    // of those 16-bit addresses are the same register.
    let port = m.space("port").expect("the I/O space");
    let low = port
        .read(0x007e, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped port");
    for high in [0x01u64, 0x3e, 0xbe, 0xff] {
        assert_eq!(
            port.read((high << 8) | 0x7e, Width::U8, MemAttrs::DEBUG)
                .expect("a mapped port"),
            low,
            "port {:#06x} must be the V counter too",
            (high << 8) | 0x7e
        );
    }

    // Long enough for the whole program: it is under sixty instructions.
    m.run_for(GlobalTime::from_nanos(200_000)).expect("it runs");

    // -- what the run proves -----------------------------------------------
    //
    // Two VDP register writes arrived through `OUT ($BF),A`, which only works
    // if `iospace = "port"` resolved to a real second address space.
    assert_eq!(vdp.vdp().register(0), 0x04, "register 0 reached the chip");
    assert_eq!(vdp.vdp().register(1), 0x60, "and so did register 1");

    // What the guest stashed in RAM:
    //   $C000  what appeared at $4000 after `ld ($fffe),2`  -> bank 2
    //   $C001  what is at $0000 after `ld ($fffd),3`        -> still $F3
    //   $C002  what is at $0400 after the same write        -> bank 3
    assert_eq!(peek(&m, "mem", RAM), 0xb2, "slot 1 followed $FFFE");
    assert_eq!(
        peek(&m, "mem", RAM + 1),
        0xf3,
        "the first kilobyte does not move, whatever $FFFD says"
    );
    assert_eq!(
        peek(&m, "mem", RAM + 2),
        0xc3,
        "and the rest of slot 0 followed $FFFD"
    );

    // The work RAM is mirrored: a write at $C000 is visible at $E000.
    assert_eq!(peek(&m, "mem", 0xe000 + 2), 0xc3, "$E000 is $C000 again");

    // And the debug console caught `out ($fd),a`, through the write half of a
    // `split()` whose read half is still the control pads.
    assert_eq!(console.text(), "A");
}

#[test]
fn the_pause_button_is_a_non_maskable_interrupt() {
    let _guard = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut m = boot("sms-ntsc");
    let io = capture::take_io().expect("the machine has an I/O chip");
    // The program's first instruction is `di`, so an ordinary interrupt cannot
    // reach it. Run until it is parked in its loop.
    m.run_for(GlobalTime::from_nanos(200_000)).expect("it runs");
    assert_eq!(peek(&m, "mem", RAM + 3), 0x00, "nothing has taken an NMI");

    io.pulse_pause();
    m.run_for(GlobalTime::from_nanos(100_000)).expect("it runs");
    assert_eq!(
        peek(&m, "mem", RAM + 3),
        0x5a,
        "the handler at $0066 ran, with interrupts disabled"
    );
}

/// The hash of the frame the drawing routine above produces.
///
/// One 8x8 white tile in the corner of an otherwise black 256x192 screen. It is
/// recorded rather than derived: the point of a regression hash is that nothing
/// can produce it except the renderer producing the same pixels again.
const EXPECTED_FRAME_HASH: u64 = 11_997_852_215_037_811_045;

#[test]
fn the_guest_draws_a_picture_and_the_scanout_seam_hands_it_over() {
    // The last claim, and the one nothing else covers: a program running on the
    // emulated Z80 programmes the VDP entirely through `OUT ($BE)`/`OUT ($BF)`,
    // the chip renders, and the host side of the scanout seam converts what it
    // rendered into RGB at the right size. No host poking anywhere on that path.
    let _guard = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut m = boot("sms-ntsc");
    let scanout = capture::take_vdp().expect("the machine has a VDP");

    // Two frames: the first is drawn while the program is still setting the
    // chip up, the second with everything in place.
    m.run_for(GlobalTime::from_nanos(40_000_000)).expect("runs");
    assert!(
        scanout.vdp().frame() >= 2,
        "the VDP counted {} frames",
        scanout.vdp().frame()
    );

    // `info` reports the *current* mode's height rather than a constant: mode
    // 4's 192-line variant here, and the framebuffer is cropped to it.
    let info = scanout.info();
    assert_eq!((info.width, info.height), (256, 192));

    let mut surface = Surface::new(PixelFormat::RGBA8888, info.width, info.height);
    scanout.capture(&mut surface);
    // Colour 1 is $3F — every gun at its top level — so tile 1 is white, and
    // the two-bit ladder puts that at $FF rather than near it.
    assert_eq!(surface.get(0, 0), Some([0xff, 0xff, 0xff]));
    assert_eq!(surface.get(7, 7), Some([0xff, 0xff, 0xff]));
    // The next tile column is name-table entry 1, which the program never
    // wrote: tile 0, all zeroes, so colour 0.
    assert_eq!(surface.get(8, 0), Some([0x00, 0x00, 0x00]));

    // And a hash of the whole cropped picture, so a change anywhere in the
    // renderer has to be deliberate (`CLAUDE.md`, Testing: a machine-level
    // regression asserts a framebuffer hash).
    assert_eq!(
        frame_hash(scanout.vdp()),
        EXPECTED_FRAME_HASH,
        "the picture changed; if that was intended, update the constant and say why"
    );
}
