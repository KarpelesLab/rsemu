//! Alice and Lisa on the shipped A1200, driven by a 68EC020 program: eight
//! bitplanes out of the 256-entry colour table, the three `FMODE` fetch widths
//! drawing the same picture, and a 64-bit sprite.
//!
//! `src/dev/amiga/agnus/tests.rs` proves each piece against the chip; this is
//! the one place Alice, Lisa, the custom bus, chip RAM and a processor run
//! together on `machines/amiga-a1200.machine`, driven by guest code rather
//! than by the test reaching into the address space. It is
//! `tests/amiga_a500_chipset.rs` for the AA board.
//!
//! **No ROM.** The program is hand-assembled here from the MC68000 user's
//! manual's instruction formats, which the 68EC020 executes unchanged. Every
//! register value is the *Specification for the Advanced Amiga (AA) Chip Set*'s
//! (Commodore-Amiga), cited by section, except the nominal PAL window
//! (`DIWSTRT $2C81`, `DIWSTOP $2CC1`, `DDFSTRT $38`, `DDFSTOP $D0`), which is
//! the *Amiga Hardware Reference Manual*'s chapter 3 and which AA does not
//! change.
//!
//! `RSEMU_AMIGA_FRAME_DIR`, when set and built with `display-png`, receives
//! `alice-256.png` and `alice-sprite.png`.

#![cfg(feature = "machine-amiga-a1200")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::dev::amiga::custom::CustomBus;
use rsemu::dev::amiga::denise::Video;
use rsemu::machine::{Machine, catalog};

// Register addresses, as §3 orders them.
const DIWSTRT: u32 = 0xDF_F08E;
const DIWSTOP: u32 = 0xDF_F090;
const DDFSTRT: u32 = 0xDF_F092;
const DDFSTOP: u32 = 0xDF_F094;
const DMACON: u32 = 0xDF_F096;
const BPL1PT: u32 = 0xDF_F0E0;
const BPL1MOD: u32 = 0xDF_F108;
const BPL2MOD: u32 = 0xDF_F10A;
const BPLCON0: u32 = 0xDF_F100;
const BPLCON3: u32 = 0xDF_F106;
const COLOR00: u32 = 0xDF_F180;
const SPR0PT: u32 = 0xDF_F120;
const FMODE: u32 = 0xDF_F1FC;

/// `DMACON`'s `SET/CLR`, `DMAEN`, `BPLEN` and `SPREN` (§4).
const DMA_BITPLANES: u16 = 0x8000 | 0x0200 | 0x0100;
const DMA_SPRITES: u16 = 0x8000 | 0x0200 | 0x0020;
/// `COPEN`, bit 7: the copper, which reloads the display registers every
/// field.
const COPEN: u16 = 0x0080;

/// `BPLCON3`'s reset `PF2OF = 011`, kept in every write (§4).
const PF2OF: u16 = 0b011 << 10;
/// `BPLCON0`: `BPU3` at bit 4 — "0000-1000 (NONE thru 8 inclusive)" — and
/// `COLOR` at bit 9.
const BPU8: u16 = 1 << 4;
const COLOR_ON: u16 = 1 << 9;

/// `FMODE`'s bitplane and sprite widths (§4's two tables).
const BPL32: u16 = 1 << 0;
const BPAGEM: u16 = 1 << 1;
const SPR32: u16 = 1 << 2;
const SPAGEM: u16 = 1 << 3;

/// Where the program puts its eight bitplanes, 16 KiB apart, and its sprite.
/// The planes are above 1 MiB on purpose: only a twenty-bit pointer reaches
/// them (§3's preamble), so this is the board's 2 MiB as well as its picture.
const PLANE0: u32 = 0x10_0000;
const PLANE_STRIDE: u32 = 0x4000;
const SPRITE: u32 = 0x18_0000;

/// The fetch: twenty words a line for 256 lines, `DDFSTRT $38`–`DDFSTOP $D0`.
const WORDS_PER_LINE: u32 = 20;
const LINES: u32 = 256;

/// Each plane's fill word. Planes 1–4 halve their run length, so the low
/// nibble of a pixel's colour counts down across every word; planes 5, 6 and 7
/// are clear and plane 8 is set, which puts the whole picture in the second
/// half of the 256-entry table — a place only `BPU = 8` can reach.
const FILL: [u16; 8] = [
    0xaaaa, 0xcccc, 0xf0f0, 0xff00, 0x0000, 0x0000, 0x0000, 0xffff,
];

/// A tiny 68000 assembler: enough for the two programs below, one instruction
/// per method so each can be checked against its encoding in the user's
/// manual.
#[derive(Default)]
struct Asm(Vec<u16>);

impl Asm {
    /// `move.b #imm,(xxx).l`
    fn move_b(&mut self, imm: u8, at: u32) -> &mut Asm {
        self.0.extend([0x13fc, u16::from(imm)]);
        self.long(at)
    }

    /// `move.w #imm,(xxx).l`
    fn move_w(&mut self, imm: u16, at: u32) -> &mut Asm {
        self.0.extend([0x33fc, imm]);
        self.long(at)
    }

    /// `move.l #imm,(xxx).l`
    fn move_l(&mut self, imm: u32, at: u32) -> &mut Asm {
        self.0.push(0x23fc);
        self.long(imm);
        self.long(at)
    }

    /// `lea (xxx).l,a0`
    fn lea_a0(&mut self, at: u32) -> &mut Asm {
        self.0.push(0x41f9);
        self.long(at)
    }

    /// `move.w #imm,d0`
    fn move_d0(&mut self, imm: u16) -> &mut Asm {
        self.0.extend([0x303c, imm]);
        self
    }

    /// `move.w #imm,(a0)+`
    fn store_w(&mut self, imm: u16) -> &mut Asm {
        self.0.extend([0x30fc, imm]);
        self
    }

    /// `move.w #imm,(a0)+` then `dbra d0,*-4`: the two-word store repeated.
    fn fill_w(&mut self, imm: u16) -> &mut Asm {
        self.store_w(imm);
        self.0.extend([0x51c8, 0xfffa]);
        self
    }

    /// `bra *`
    fn halt(&mut self) -> &mut Asm {
        self.0.push(0x60fe);
        self
    }

    fn long(&mut self, value: u32) -> &mut Asm {
        self.0.extend([(value >> 16) as u16, value as u16]);
        self
    }

    /// A 512 KiB Kickstart socket with the reset vectors at its head and this
    /// code from `$F8000C`, where the second vector points.
    fn rom(&self) -> Vec<u8> {
        let mut image = vec![0u8; 512 * 1024];
        // The stack at the top of 2 MiB, and the entry in the ROM's own window
        // so the code keeps running when the overlay leaves address zero.
        image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes());
        image[4..8].copy_from_slice(&0x00F8_000Cu32.to_be_bytes());
        for (i, word) in self.0.iter().enumerate() {
            let at = 0x0c + 2 * i;
            image[at..at + 2].copy_from_slice(&word.to_be_bytes());
        }
        image
    }
}

/// Where the copper list goes, and the copper's own registers.
const COPPER: u32 = 0x00_1000;
const COP1LC: u32 = 0xDF_F080;
const COPJMP1: u32 = 0xDF_F088;

/// A copper list as chapter 2 writes one: `MOVE` pairs, then `WAIT` for a
/// position that never comes, which is how a list ends.
///
/// **The display registers belong here rather than in the processor's
/// instruction stream**, and not for elegance: Alice advances a bitplane
/// pointer as she fetches and never puts it back, so a pointer written once by
/// the processor is right for one field and past the end of the picture in the
/// next. The copper restarts from `COP1LC` at the top of every field
/// (chapter 2, *Starting the Copper After Reset*), which is what reloads them.
#[derive(Default)]
struct Copper(Vec<u16>);

impl Copper {
    /// `MOVE` immediate to a custom register.
    fn set(&mut self, at: u32, value: u16) -> &mut Copper {
        self.0.extend([(at & 0x1fe) as u16, value]);
        self
    }

    /// The `WAIT` that never comes true: "$FFFF, $FFFE".
    fn end(&mut self) -> &mut Copper {
        self.0.extend([0xffff, 0xfffe]);
        self
    }

    /// Everything both programs set: the window, the fetch and `FMODE`.
    fn window(&mut self, fmode: u16) -> &mut Copper {
        self.set(DIWSTRT, 0x2c81);
        self.set(DIWSTOP, 0x2cc1);
        self.set(DDFSTRT, 0x0038);
        self.set(DDFSTOP, 0x00d0);
        self.set(FMODE, fmode)
    }
}

/// The eight-bitplane copper list: the colour table's second half, the eight
/// pointers, the window, and `BPU = 8`.
fn planes_list(fmode: u16) -> Vec<u16> {
    let mut c = Copper::default();
    // COLOR00 black in bank 0, then colours 128-143 — bank 4's first sixteen
    // (§2's BANK table: "1 0 0 || COLOR80 - COLOR9F") — as sixteen reds.
    c.set(BPLCON3, PF2OF);
    c.set(COLOR00, 0x0000);
    c.set(BPLCON3, 4 << 13 | PF2OF);
    for n in 0..16u16 {
        c.set(COLOR00 + 2 * u32::from(n), n << 8);
    }
    c.set(BPLCON3, PF2OF);
    for p in 0..8u32 {
        let at = PLANE0 + PLANE_STRIDE * p;
        c.set(BPL1PT + 4 * p, (at >> 16) as u16);
        c.set(BPL1PT + 4 * p + 2, at as u16);
    }
    c.set(BPL1MOD, 0x0000);
    c.set(BPL2MOD, 0x0000);
    c.window(fmode);
    c.set(BPLCON0, BPU8 | COLOR_ON);
    c.end();
    c.0
}

/// The sprite copper list: one colour, the sprite's pointer, the window with a
/// 64-bit sprite fetch, and no bitplanes.
fn sprite_list() -> Vec<u16> {
    let mut c = Copper::default();
    c.set(BPLCON3, PF2OF);
    c.set(COLOR00, 0x0000);
    // An even sprite's colours are `ESPRM` (reset `0001`, §4's `BPLCON4`) over
    // the pair, so sprite colour 1 is entry $11.
    c.set(COLOR00 + 2 * 0x11, 0x0f00);
    c.set(SPR0PT, (SPRITE >> 16) as u16);
    c.set(SPR0PT + 2, SPRITE as u16);
    c.window(SPAGEM | SPR32);
    c.set(BPLCON0, COLOR_ON);
    c.end();
    c.0
}

/// The program: drop the overlay, fill chip RAM, write the copper list out
/// word by word, point the copper at it and turn DMA on.
fn program(list: &[u16], fills: &[(u32, u16, u32)], dma: u16) -> Vec<u8> {
    let mut a = Asm::default();
    // Any CIA write drops Gayle's overlay, and chip RAM answers at zero.
    a.move_b(0x00, 0xBF_E001);
    for (at, word, words) in fills {
        a.lea_a0(*at);
        a.move_d0((words - 1) as u16);
        a.fill_w(*word);
    }
    a.lea_a0(COPPER);
    for word in list {
        a.store_w(*word);
    }
    a.move_l(COPPER, COP1LC);
    a.move_w(0x0000, COPJMP1);
    a.move_w(dma, DMACON);
    a.halt();
    a.rom()
}

/// The eight-bitplane program.
fn planes_program(fmode: u16) -> Vec<u8> {
    let fills: Vec<(u32, u16, u32)> = FILL
        .iter()
        .enumerate()
        .map(|(p, word)| {
            (
                PLANE0 + PLANE_STRIDE * p as u32,
                *word,
                WORDS_PER_LINE * LINES,
            )
        })
        .collect();
    program(&planes_list(fmode), &fills, DMA_BITPLANES | COPEN)
}

/// The sprite program.
///
/// §4's `FMODE` table makes every sprite transfer eight bytes, control words
/// included, so the structure is padded to match: `SPRxPOS` and `SPRxCTL` are
/// the first word of a transfer each, then two lines of four words of A and
/// four of B, then the control pair that stops the channel. The whole thing is
/// written out word by word rather than filled.
fn sprite_program() -> Vec<u8> {
    let mut a = Asm::default();
    a.move_b(0x00, 0xBF_E001);
    a.lea_a0(SPRITE);
    // SPRxPOS $4040: VSTART $40, and `SH10`-`SH3` = $40. SPRxCTL $4201: VSTOP
    // $42, and `SH2` — 140 ns, §4's `SPRxCTL` page — set. So the sprite's
    // first pixel is low-resolution pixel `2 × $40 + 1 = $81`, the display
    // window's own left edge; with `SH2` clear it would be $80, one pixel
    // outside the window, and the window would eat it.
    let mut words = vec![0x4040u16, 0, 0, 0, 0x4201, 0, 0, 0];
    for _ in 0..2 {
        words.extend([0xffff; 4]); // A: every pixel set
        words.extend([0x0000; 4]); // B: so every pixel is sprite colour 1
    }
    // The pair the channel takes at VSTOP. A `SPRxPOS` of zero is a VSTART
    // this field will not come round to again, so the sprite stops here.
    words.extend([0u16; 8]);
    a.lea_a0(SPRITE);
    for word in &words {
        a.store_w(*word);
    }
    let list = sprite_list();
    a.lea_a0(COPPER);
    for word in &list {
        a.store_w(*word);
    }
    a.move_l(COPPER, COP1LC);
    a.move_w(0x0000, COPJMP1);
    a.move_w(DMA_SPRITES | COPEN, DMACON);
    a.halt();
    a.rom()
}

fn boot(rom: Vec<u8>) -> (Machine, rsemu::host::display::amiga::DeniseScanout) {
    boot_on("amiga-a1200", rom)
}

fn boot_on(name: &str, rom: Vec<u8>) -> (Machine, rsemu::host::display::amiga::DeniseScanout) {
    let entry = catalog::machine(name).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    rsemu::host::display::amiga::capture::install(&mut options).expect("a capture table");
    options.realize.media.insert("kickstart", rom);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let scanout = rsemu::host::display::amiga::capture::take(&options.realize.hosts, &machine)
        .expect("a Lisa");
    (machine, scanout)
}

fn video(m: &Machine) -> Arc<Video> {
    let export = m
        .device("denise")
        .expect("the board has a denise")
        .device()
        .export(ExportId::AMIGA_VIDEO)
        .expect("Denise publishes its video handle");
    Arc::clone(export.opaque().expect("opaque"))
        .downcast::<Video>()
        .expect("a Video")
}

fn custom_bus(m: &Machine) -> Arc<CustomBus> {
    let export = m
        .device("custom")
        .expect("the board has a custom")
        .device()
        .export(ExportId::CUSTOM_BUS)
        .expect("published");
    Arc::clone(export.opaque().expect("opaque"))
        .downcast::<CustomBus>()
        .expect("a CustomBus")
}

/// Lisa's picture is 1600 columns of 35 ns; `read_row_rgb` is the whole gun.
fn row(v: &Video, y: u32) -> Vec<u32> {
    let mut row = vec![0u32; 1600];
    v.read_row_rgb(y, &mut row);
    row
}

/// A picture row for line `vpos`: two rows a line from line `$1D`.
fn y(vpos: u32) -> u32 {
    2 * (vpos - 0x1d)
}

/// The window's first column: `$81` low-resolution pixels from
/// `denise::OUTPUT_LEFT`, four quarters each.
const LEFT: usize = (0x81 - 64) * 4;

fn frame_hash(v: &Video) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for line in 0..568u32 {
        for pixel in row(v, line) {
            for byte in pixel.to_be_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    hash
}

fn write_png(scanout: &rsemu::host::display::amiga::DeniseScanout, name: &str) {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") else {
        return;
    };
    #[cfg(feature = "display-png")]
    {
        use rsemu::host::display::{PixelFormat, Scanout, Surface};
        let info = scanout.info();
        let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
        scanout.capture(&mut surface);
        let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
        std::fs::write(std::path::Path::new(&dir).join(name), png)
            .expect("the frame directory is writable");
    }
    #[cfg(not(feature = "display-png"))]
    {
        let _ = (dir, scanout, name);
    }
}

fn run(rom: Vec<u8>) -> (Machine, rsemu::host::display::amiga::DeniseScanout) {
    let (mut m, scanout) = boot(rom);
    m.run_for(GlobalTime::from_nanos(200_000_000))
        .expect("it runs");
    (m, scanout)
}

/// Eight bitplanes fetched by Alice out of the second megabyte, shown by Lisa
/// out of the 256-entry table.
///
/// Planes 1–4 halve their run length and plane 8 is set, so a pixel's colour
/// is `128 + (15 − i)` for the `i`th pixel of every sixteen — sixteen reds
/// shading from `$FF` down to `$00` and repeating, twenty times a line. Only
/// `BPU = 8` reaches colour 128, and only Alice fetches planes 7 and 8.
#[test]
fn eight_planes_out_of_the_second_megabyte_fill_the_256_entry_table() {
    let (m, scanout) = run(planes_program(0));
    let v = video(&m);
    assert!(v.fields() >= 9, "Alice pushed {} fields", v.fields());
    assert_eq!(custom_bus(&m).unclaimed(), 0, "nothing fell through");

    let r = row(&v, y(0x2c));
    assert_eq!(r[LEFT - 1], 0x0000_0000, "the border is colour 0");
    for i in 0..16usize {
        let want = (u32::from((15 - i) as u8) * 17) << 16;
        let at = LEFT + 4 * i;
        assert_eq!(r[at], want, "pixel {i}: colour {}", 128 + 15 - i);
        assert!(
            r[at..at + 4].iter().all(|&p| p == want),
            "a low-resolution pixel is four 35 ns columns"
        );
    }
    // Twenty words of sixteen pixels, and the border after them.
    assert_eq!(r[LEFT + 320 * 4 - 1], 0x0000_0000, "colour 128, black");
    assert_eq!(r[LEFT + 320 * 4], 0x0000_0000, "and the border");
    // The last line of the window, and nothing after it.
    assert_eq!(row(&v, y(0x12b))[LEFT], 0x00ff_0000);
    assert_eq!(row(&v, y(0x12c))[LEFT], 0x0000_0000);

    write_png(&scanout, "alice-256.png");
    let hash = frame_hash(&v);
    assert_eq!(hash, GOLDEN_PLANES, "the golden moved: {hash:#018x}");
}

/// The same program with a 32- or 64-bit `FMODE` draws the same picture.
///
/// "The parallel to serial conversion is triggered whenever bit plane #1 is
/// written, indicating the completion of all bit planes for that word
/// (16/32/64 pixels). The MSB is output first" (§4, `BPLxDAT`), so a wider
/// fetch is the same stream in fewer transfers — it buys bus cycles, not
/// picture. Twenty words a line divides by one, two and four alike, so no
/// transfer is rounded up here.
#[test]
fn the_three_fmode_widths_draw_the_same_picture() {
    let narrow = {
        let (m, _) = run(planes_program(0));
        frame_hash(&video(&m))
    };
    for (fmode, what) in [
        (BPL32, "32 bits, normal CAS"),
        (BPAGEM, "32 bits, double CAS"),
        (BPAGEM | BPL32, "64 bits"),
    ] {
        let (m, _) = run(planes_program(fmode));
        assert_eq!(
            frame_hash(&video(&m)),
            narrow,
            "FMODE ${fmode:04x} ({what}) drew a different picture"
        );
    }
    assert_eq!(narrow, GOLDEN_PLANES);
}

/// One 64-bit sprite, fetched four words a transfer by Alice and placed by
/// Lisa.
///
/// `SPRxPOS $4040` with `SPRxCTL`'s `SH2` set puts its left edge at
/// `2 × $40 + 1 = $81` — the window's own left edge — and `SPRxCTL`'s VSTOP of
/// $42 stops it two lines down, so lines `$40` and `$41` carry 64 pixels of
/// sprite colour 1 and line `$42` carries none.
/// The A data is all ones and the B data all zeros, so every pixel is colour
/// 1 of the even sprites' bank: `ESPRM` resets to `0001` (§4, `BPLCON4`),
/// which makes it entry `$11`.
#[test]
fn a_sixty_four_bit_sprite_alice_fetched_is_where_lisa_puts_it() {
    let (m, scanout) = run(sprite_program());
    let v = video(&m);
    assert_eq!(custom_bus(&m).unclaimed(), 0, "nothing fell through");

    let red = 0x00ff_0000;
    for line in [0x40u32, 0x41] {
        let r = row(&v, y(line));
        assert_eq!(r[LEFT - 1], 0x0000_0000, "nothing before HSTART");
        assert!(
            r[LEFT..LEFT + 64 * 4].iter().all(|&p| p == red),
            "line {line:#x}: 64 low-resolution pixels of sprite"
        );
        assert_eq!(r[LEFT + 64 * 4], 0x0000_0000, "and nothing after them");
    }
    assert_eq!(row(&v, y(0x42))[LEFT], 0x0000_0000, "VSTOP is exclusive");
    assert_eq!(row(&v, y(0x3f))[LEFT], 0x0000_0000, "and VSTART inclusive");

    write_png(&scanout, "alice-sprite.png");
}

/// The eight-plane picture: sixteen reds counting down, twenty times a line,
/// on every line of the nominal PAL window and nothing outside it. Looked at
/// as `alice-256.png`.
const GOLDEN_PLANES: u64 = 0x77fd_b1d1_884b_8325;
