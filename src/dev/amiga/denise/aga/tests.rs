//! Lisa, driven the way Alice will drive her: register writes through
//! [`CustomChip`], lines through [`Video::line`], wide sprite data through
//! [`Video::sprite_dma`].
//!
//! Every expected pixel is worked out from the *Specification for the Advanced
//! Amiga (AA) Chip Set* — "the specification" below — or, where it says a
//! thing is "as before", from the 3rd-edition manual, and the comment beside
//! it says which part. Positions are in **quarters**, 35 ns each, four to a
//! low-resolution pixel: the unit Lisa's picture columns are in.

use super::super::*;
use crate::core::props::{Link, Value};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::amiga::regs;

/// A PAL line's length in colour clocks.
const CLOCKS: u16 = 227;

/// A line inside the standard PAL window (`$2C` to `$12C`).
const V: u16 = 100;

/// The quarter of the picture's first column: [`OUTPUT_LEFT`] low-resolution
/// pixels in.
const LEFT: i32 = OUTPUT_LEFT as i32 * 4;

/// The quarter the standard window opens at, `$81` low-resolution pixels in —
/// and where a low-resolution word fetched at `$38` is first shown.
const X81: i32 = 0x81 * 4;

fn lisa() -> Video {
    Video::with_revision(Standard::Pal, Revision::Aga)
}

fn reg(offset: u16) -> &'static Reg {
    regs::lookup(offset).expect("a register the address map declares")
}

/// A processor write.
fn w(v: &Video, offset: u16, value: u16) {
    CustomChip::write(v, reg(offset), value, Origin::cpu());
}

fn color(i: u16) -> u16 {
    COLOR00 + 2 * i
}

fn spr_pos(n: u16) -> u16 {
    SPR0POS + 8 * n
}

/// Palette entry `n` loaded with a whole 24-bit colour the way the
/// specification says to: "Loading the MSB always loads the LSB as well for
/// compatibility, so when 24 bit colors are desired load LSB after MSB" (§2,
/// *Color Lookup Table*), a `BANK` at a time. Leaves `BPLCON3` at its reset
/// value.
fn set_rgb(v: &Video, n: usize, rgb: u32) {
    let bank = (n / 32) as u16;
    let at = color((n % 32) as u16);
    let nibble = |shift: u32| ((rgb >> shift) & 0xf) as u16;
    w(v, BPLCON3, bank << 13 | PF2OF_DEFAULT);
    w(v, at, nibble(20) << 8 | nibble(12) << 4 | nibble(4));
    w(v, BPLCON3, bank << 13 | PF2OF_DEFAULT | LOCT);
    w(v, at, nibble(16) << 8 | nibble(8) << 4 | nibble(0));
    w(v, BPLCON3, PF2OF_DEFAULT);
}

/// A distinct 24-bit colour for every entry, with low nibbles no `LOCT = 0`
/// write could produce.
fn distinct(n: usize) -> u32 {
    let n = n as u32;
    (n << 16) | ((255 - n) << 8) | (n.wrapping_mul(37) & 0xff)
}

/// The standard window and all 256 entries [`distinct`].
fn setup(v: &Video, bplcon0: u16) {
    w(v, DIWSTRT, 0x2c81);
    w(v, DIWSTOP, 0x2cc1);
    for n in 0..256 {
        set_rgb(v, n, distinct(n));
    }
    w(v, BPLCON0, bplcon0);
}

/// A line whose fetch began at `start`.
fn line(v: &Video, vpos: u16, start: u16, planes: [&[u16]; 8]) {
    v.line(&Line {
        vpos,
        clocks: CLOCKS,
        fetch: Fetch { start, planes },
    });
}

/// The eight plane streams that show `values` one bitplane pixel each, from
/// the first fetched bit on.
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

/// A line showing `values`, a bitplane pixel each, fetched at `start`.
fn show(v: &Video, vpos: u16, start: u16, values: &[u8]) {
    let s = streams(values);
    line(
        v,
        vpos,
        start,
        [&s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &s[6], &s[7]],
    );
}

/// The 24-bit picture at line `vpos`, quarter `q`.
fn quarter(v: &Video, vpos: u16, q: i32) -> u32 {
    let row = 2 * u32::from(vpos - Standard::Pal.first_line());
    let (width, _) = v.geometry();
    let mut words = vec![0u32; width as usize];
    v.read_row_rgb(row, &mut words);
    words[usize::try_from(q - LEFT).expect("inside the picture")]
}

/// The low-resolution pixel `k` pixels right of `$81`, whose four quarters
/// must agree.
fn lores(v: &Video, vpos: u16, k: i32) -> u32 {
    let q = X81 + 4 * k;
    let first = quarter(v, vpos, q);
    for i in 1..4 {
        assert_eq!(
            quarter(v, vpos, q + i),
            first,
            "a low-resolution pixel is four equal quarters (pixel {k})"
        );
    }
    first
}

/// `BPU = 8`: the three low bits in 14–12 and `BPU3` in bit 4 (§4, `BPLCON0`).
const EIGHT: u16 = 0x0010;

// ---------------------------------------------------------------------------
// the colour table
// ---------------------------------------------------------------------------

#[test]
fn lisaid_is_f8_and_lisa_drives_it() {
    // "Lisa returns hex (f8)" (§4, LISAID).
    let v = lisa();
    assert!(CustomChip::drives(&v, reg(DENISEID)));
    assert_eq!(CustomChip::read(&v, reg(DENISEID), Origin::cpu()), 0x00f8);
    assert_eq!(v.revision(), Revision::Aga);
}

/// `LISAID` bits 9 and 8 are the board's fetch capability, and this board's is
/// four times: both bits low.
///
/// Not a spare-bits convention but a measurement — `docs/platforms/amiga.md`,
/// "What `LISAID` bits 9 and 8 are": Kickstart 3.1's `graphics.library` reads
/// exactly this pair and publishes `MaxDepth` 8 for low, high *and* super-high
/// resolution when it is `00`, and the Enhanced Chip Set's 5/4/2 when it is
/// `11`. The two bits are §4's `FMODE` pair `BPAGEM`/`BPL32` read active low,
/// so `00` is "double CAS, 32 bits wide" — an A1200's four 256K × 16 DRAMs —
/// which is `FMODE`'s four times the bandwidth.
///
/// The 8373 beside it drives none of the byte, answers `11`, and gets the
/// Enhanced Chip Set's depths, which is the control: the ECS constant is right
/// *because* its reserved byte reads as ones, and Lisa's was wrong for exactly
/// that reason.
#[test]
fn lisaids_upper_bits_say_the_fetch_is_four_times_and_an_8373s_say_one() {
    let lisa = lisa();
    let id = CustomChip::read(&lisa, reg(DENISEID), Origin::cpu());
    assert_eq!(id & 0x00ff, 0xf8, "the low byte is the specification's");
    assert_eq!(id >> 8 & 3, 0, "BPAGEM and BPL32 active low: four times");

    let ecs = Video::with_revision(Standard::Pal, Revision::Ecs);
    let ecs_id = CustomChip::read(&ecs, reg(DENISEID), Origin::cpu());
    assert_eq!(ecs_id & 0x00ff, 0xfc, "\"$FC in the lower 8 bits\"");
    assert_eq!(ecs_id >> 8 & 3, 3, "an 8373 drives neither: one times");
}

#[test]
fn a_loct_clear_write_extends_each_gun_and_a_loct_write_fills_the_low_half() {
    let v = lisa();
    setup(&v, 0x1000);
    // "When LOCT = 0 … the 4 bit values are automatically extended to 8 bits"
    // (§4, COLORx): $FA5 is $FF, $AA, $55.
    w(&v, color(1), 0x0fa5);
    show(&v, V, 0x38, &[1]);
    assert_eq!(lores(&v, V, 0), 0x00ff_aa55);

    // "LOCT can be set high and independant values for the 4 LSB of red, green
    // and blue can be written."
    w(&v, BPLCON3, PF2OF_DEFAULT | LOCT);
    w(&v, color(1), 0x0123);
    show(&v, V, 0x38, &[1]);
    assert_eq!(lores(&v, V, 0), 0x00f1_a253);
    // The twelve-bit view is the top nibble of each gun: the old part's pins.
    let mut row = vec![0u16; v.geometry().0 as usize];
    v.read_row(2 * u32::from(V - 0x1d), &mut row);
    assert_eq!(row[(X81 - LEFT) as usize], 0x0fa5);

    // "Loading the MSB always loads the LSB as well."
    w(&v, BPLCON3, PF2OF_DEFAULT);
    w(&v, color(1), 0x0fa5);
    show(&v, V, 0x38, &[1]);
    assert_eq!(lores(&v, V, 0), 0x00ff_aa55);
}

#[test]
fn the_t_bit_is_kept_in_the_table_and_never_reaches_the_picture() {
    let v = lisa();
    setup(&v, 0x0000);
    // "T bit of COLOR00 thru COLOR31 sets ZD_pin HI" — bit 15 with LOCT clear.
    w(&v, COLOR00, 0x8123);
    line(&v, V, 0x38, Default::default());
    assert_eq!(
        quarter(&v, V, X81 - 40),
        0x0011_2233,
        "the border, T dropped"
    );
    assert_eq!(
        v.state.lock().regs.palette[0],
        0x0111_2233,
        "T latched in bit 24"
    );
    // "The low order color registers do not contain a transparency (T) bit":
    // a LOCT write leaves it alone.
    w(&v, BPLCON3, PF2OF_DEFAULT | LOCT);
    w(&v, COLOR00, 0x0fff);
    assert_eq!(v.state.lock().regs.palette[0], 0x011f_2f3f);
}

#[test]
fn bank_picks_which_32_entries_the_colour_registers_reach() {
    let v = lisa();
    // "BANK2,1,0 [select one] of 8 32 address banks … 000 COLOR00-COLOR1F
    // … 111 COLORE0-COLORFF" (§2, *Color Lookup Table*).
    for bank in 0..8u16 {
        w(&v, BPLCON3, bank << 13 | PF2OF_DEFAULT);
        w(&v, color(31), 0x0100 | bank);
        w(&v, COLOR00, 0x0200 | bank);
    }
    let st = v.state.lock();
    for bank in 0..8usize {
        let n = bank as u32;
        assert_eq!(st.regs.palette[bank * 32 + 31], 0x0011_0000 | (n * 0x11));
        assert_eq!(st.regs.palette[bank * 32], 0x0022_0000 | (n * 0x11));
        assert_eq!(
            st.regs.palette[bank * 32 + 1],
            0,
            "entries between are untouched"
        );
    }
    assert_eq!(
        st.regs.color, [0; 32],
        "an 8362's twelve-bit registers are not Lisa's"
    );
}

#[test]
fn bank_reaches_all_256_entries_and_eight_planes_select_them() {
    let v = lisa();
    setup(&v, EIGHT);
    // "BANK2,1,0 [select one] of 8 32 address banks": COLOR00-1F, 20-3F, …
    // E0-FF. Each entry was loaded through its own bank and shows as itself.
    let values: Vec<u8> = (0..=255).collect();
    show(&v, V, 0x38, &values);
    for k in 1..256 {
        assert_eq!(lores(&v, V, k), distinct(k as usize), "colour {k:#04x}");
    }
    // Value zero inside the window is colour 0 with no mask.
    assert_eq!(lores(&v, V, 0), distinct(0));
}

#[test]
fn bpu3_is_the_fourth_bit_and_seven_planes_leave_the_eighth_unseen() {
    let v = lisa();
    setup(&v, EIGHT);
    // Plane 8 alone: colour $80.
    show(&v, V, 0x38, &[0x80, 0xff]);
    assert_eq!(lores(&v, V, 0), distinct(0x80));
    assert_eq!(lores(&v, V, 1), distinct(0xff));

    // BPU = 7: plane 8's bit is not fetched into the address.
    w(&v, BPLCON0, 0x7000);
    show(&v, V, 0x38, &[0x80, 0xff]);
    assert_eq!(lores(&v, V, 0), distinct(0), "plane 8 is off: value zero");
    assert_eq!(lores(&v, V, 1), distinct(0x7f));

    // BPU = 15 is not a value the specification defines; eight is all there
    // is to fetch.
    w(&v, BPLCON0, 0x7010);
    show(&v, V, 0x38, &[0xff]);
    assert_eq!(lores(&v, V, 0), distinct(0xff));
}

#[test]
fn bplam_xors_the_bitplane_address_and_leaves_the_border_alone() {
    let v = lisa();
    setup(&v, 0x1000);
    // "This 8 bit field is XOR'ed with the 8 bit plane color address" (§4,
    // BPLCON4); ESPRM/OSPRM stay at their reset 0001.
    w(&v, BPLCON4, 0x8011);
    show(&v, V, 0x38, &[1, 0]);
    assert_eq!(lores(&v, V, 0), distinct(0x81));
    assert_eq!(
        lores(&v, V, 1),
        distinct(0x80),
        "a zero pixel is an address too"
    );
    assert_eq!(
        quarter(&v, V, X81 - 4),
        distinct(0),
        "the border is colour 0"
    );
}

// ---------------------------------------------------------------------------
// playfield modes
// ---------------------------------------------------------------------------

#[test]
fn dual_playfields_have_four_planes_each_and_pf2of_moves_the_second() {
    let v = lisa();
    // Lores, BPU = 8, DPF.
    setup(&v, EIGHT | DBLPF);
    // Pixel 0: planes 1 and 7, playfield 1 value %1001. Pixel 1: plane 8,
    // playfield 2 value %1000. "PFI = odd, FP2 = even bit planes".
    show(&v, V, 0x38, &[0x41, 0x80]);
    assert_eq!(lores(&v, V, 0), distinct(9), "playfield 1 has no offset");
    // "PF2OF … 011 … 8 (default)" (§4, BPLCON3).
    assert_eq!(lores(&v, V, 1), distinct(16));

    // PF2OF = 101: 32.
    w(&v, BPLCON3, 0b101 << 10);
    show(&v, V, 0x38, &[0x41, 0x80]);
    assert_eq!(lores(&v, V, 0), distinct(9));
    assert_eq!(lores(&v, V, 1), distinct(40));

    // PF2OF = 000: none — and playfield 2 then lands on playfield 1's colours.
    w(&v, BPLCON3, 0);
    show(&v, V, 0x38, &[0x41, 0x80]);
    assert_eq!(lores(&v, V, 1), distinct(8));
}

#[test]
fn extra_half_brite_needs_low_resolution_on_lisa() {
    let v = lisa();
    set_rgb(&v, 1, 0x00fe_8042);
    set_rgb(&v, 0x21, 0x0012_3456);
    w(&v, DIWSTRT, 0x2c81);
    w(&v, DIWSTOP, 0x2cc1);
    // Six planes, lores: "EHB is invoked whenever SHRES = HIRES = HAMEN = DPF
    // = 0 and BPU = 6" — colour 1 at half intensity, eight bits a gun.
    w(&v, BPLCON0, 0x6000);
    show(&v, V, 0x38, &[0x21]);
    assert_eq!(lores(&v, V, 0), 0x007f_4021);

    // The same six planes in HIRES are 64 colours (§5: "6 Bitplanes 64
    // colours" and "6 Bitplanes EHB" listed apart; §2 requires HIRES = 0).
    w(&v, BPLCON0, 0xe000);
    show(&v, V, 0x3c, &[0x21]);
    assert_eq!(quarter(&v, V, X81), 0x0012_3456);
    assert_eq!(quarter(&v, V, X81 + 1), 0x0012_3456);

    // KILLEHB, inherited from the 8373, turns it off.
    w(&v, BPLCON0, 0x6000);
    w(&v, BPLCON2, KILLEHB);
    show(&v, V, 0x38, &[0x21]);
    assert_eq!(lores(&v, V, 0), 0x0012_3456);
}

#[test]
fn ham8_selects_one_of_64_bases_and_modifies_the_six_high_bits_of_a_gun() {
    let v = lisa();
    setup(&v, EIGHT | HOMOD);
    // Base registers are the 8-bit plane address with the control bits (planes
    // 1 and 2) at 00: data 5 is register 5 << 2 = 20. The two low bits of
    // every gun here are 11, so a modify that "left [them] unmodified" shows.
    set_rgb(&v, 20, 0x0013_2333);
    set_rgb(&v, 4, 0x00ab_cdef);
    let values = [
        0b0001_0100, // 00: base register 20
        0b1111_1101, // 01: blue  = %111111 << 2 | 11
        0b0000_0010, // 10: red   = %000000 << 2 | 11
        0b1000_0011, // 11: green = %100000 << 2 | 11
        0b0000_0100, // 00: base register 4
    ];
    show(&v, V, 0x38, &values);
    // "BP2 BP1 = 00 select new base register (1 of 64)".
    assert_eq!(lores(&v, V, 0), 0x0013_2333);
    // "01 hold hold modify": blue.
    assert_eq!(lores(&v, V, 1), 0x0013_23ff);
    // "10 modify hold hold": red.
    assert_eq!(lores(&v, V, 2), 0x0003_23ff);
    // "11 hold modify hold": green.
    assert_eq!(lores(&v, V, 3), 0x0003_83ff);
    assert_eq!(lores(&v, V, 4), 0x00ab_cdef);
}

#[test]
fn ham8_shows_colours_no_palette_holds() {
    // "allows creation of all 16,777,216 colors simultaneously": a ramp of
    // blue across a line, each step a colour that is not in the table.
    let v = lisa();
    setup(&v, EIGHT | HOMOD);
    set_rgb(&v, 0, 0);
    let values: Vec<u8> = (0..64u8).map(|b| b << 2 | 0b01).collect();
    show(&v, V, 0x38, &values);
    for (k, b) in (0..64u32).enumerate() {
        // Blue's two low bits come from colour 0's, which are 00.
        assert_eq!(lores(&v, V, k as i32), b << 2, "step {k}");
    }
}

#[test]
fn ham6_works_in_hires_on_lisa_and_holds_the_low_nibble() {
    let v = lisa();
    // HIRES, BPU = 6, HAM: "The old 6 bitplane HAM mode, unlike before, works
    // in HIRES and SHRES resolutions" (§2).
    setup(&v, 0xe800);
    set_rgb(&v, 3, 0x001a_2b3c);
    let values = [
        0b00_0011, // 00: base register 3
        0b01_1111, // 01: blue = $F in the high nibble
        0b10_0000, // 10: red = $0 in the high nibble
    ];
    // A high-resolution word fetched at $3C lands at $81, two quarters a pixel.
    show(&v, V, 0x3c, &values);
    let hires = |k: i32| {
        let a = quarter(&v, V, X81 + 2 * k);
        assert_eq!(quarter(&v, V, X81 + 2 * k + 1), a);
        a
    };
    assert_eq!(hires(0), 0x001a_2b3c);
    // The inference: four bits into the four most significant, the rest held.
    assert_eq!(hires(1), 0x001a_2bfc);
    assert_eq!(hires(2), 0x000a_2bfc);
}

// ---------------------------------------------------------------------------
// scroll and fetch width
// ---------------------------------------------------------------------------

/// Where a fetch's first pixel lands, at every bandwidth, as Kickstart 3.1
/// programs its own screens for it (`fetch_block` has the measurement).
///
/// Each row is one screen the ROM opened: `DDFSTRT $38`, `DIWSTRT $2C81`,
/// `BPLCON1 0`, and the bitplane pointer set `back` words before the bitmap's
/// first pixel. That pixel must land on the window's first quarter — which is
/// what the ROM arranged, and what a real A1200 shows.
#[test]
fn a_wide_fmode_delays_the_first_pixel_by_what_kickstart_3_1_programs_for() {
    const LORES: u16 = 0x1000;
    const HIRES_1: u16 = 0x9000;
    const SHRES_1: u16 = 0x1040;
    #[rustfmt::skip]
    let screens: [(&str, u16, u16, usize); 12] = [
        // resolution, BPLCON0, FMODE, words the ROM backs the pointer up
        ("lores", LORES, 0, 0),   ("lores", LORES, 1, 0),
        ("lores", LORES, 2, 0),   ("lores", LORES, 3, 0),
        ("hires", HIRES_1, 0, 1), ("hires", HIRES_1, 1, 0),
        ("hires", HIRES_1, 2, 0), ("hires", HIRES_1, 3, 0),
        ("shres", SHRES_1, 0, 3), ("shres", SHRES_1, 1, 2),
        ("shres", SHRES_1, 2, 2), ("shres", SHRES_1, 3, 0),
    ];
    for (name, bplcon0, fmode, back) in screens {
        let v = lisa();
        setup(&v, bplcon0);
        w(&v, FMODE, fmode);
        // `back` whole words of what precedes the bitmap, then its first
        // pixel lit.
        let mut values = vec![0u8; 16 * back];
        values.push(1);
        show(&v, V, 0x38, &values);
        let first = (X81 - 128..X81 + 128)
            .find(|&q| quarter(&v, V, q) == distinct(1))
            .unwrap_or_else(|| panic!("{name} FMODE {fmode}: the pixel is nowhere"));
        assert_eq!(
            first - X81,
            0,
            "{name} at FMODE {fmode}: the bitmap's first pixel is {} quarters from the window's \
             edge",
            first - X81
        );
    }
}

/// The block itself, against the table `fetch_block` documents.
#[test]
fn a_fetch_block_is_the_one_times_block_stretched_up_to_eight_counts() {
    // (SHRES, HIRES) -> blocks at FMODE 0, 1, 2, 3.
    let table = [
        ((false, false), [8, 8, 8, 8]),
        ((false, true), [4, 8, 8, 8]),
        ((true, false), [2, 4, 4, 8]),
    ];
    for ((shres, hires), blocks) in table {
        for (fmode, want) in blocks.into_iter().enumerate() {
            assert_eq!(
                super::fetch_block(shres, hires, fmode as u16),
                want,
                "shres={shres} hires={hires} FMODE={fmode}"
            );
        }
    }
    // Only BPL32 and BPAGEM count; the sprite and scan-double bits do not.
    assert_eq!(super::fetch_block(false, true, 0xc00c), 4);
}

#[test]
fn bplcon1_scrolls_in_35ns_steps() {
    let v = lisa();
    setup(&v, 0x1000);
    let one = |v: &Video| show(v, V, 0x38, &[1]);
    let lit = |v: &Video, q: i32| quarter(v, V, q) == distinct(1);

    one(&v);
    assert!(!lit(&v, X81 - 1) && lit(&v, X81) && lit(&v, X81 + 3) && !lit(&v, X81 + 4));

    // PF1H0, bit 8: "PFyH0 = LSB = 35ns SHRES pixel" (§4, BPLCON1).
    w(&v, BPLCON1, 0x0100);
    one(&v);
    assert!(!lit(&v, X81) && lit(&v, X81 + 1) && lit(&v, X81 + 4) && !lit(&v, X81 + 5));

    // PF1H1, bit 9: two quarters.
    w(&v, BPLCON1, 0x0200);
    one(&v);
    assert!(!lit(&v, X81 + 1) && lit(&v, X81 + 2) && !lit(&v, X81 + 6));

    // The old field in bits 3-0, "old PFyH0 now PFyH2": whole lores pixels, as
    // on an 8362.
    w(&v, BPLCON1, 0x0003);
    one(&v);
    assert_eq!(lores(&v, V, 3), distinct(1));
    assert_eq!(lores(&v, V, 2), distinct(0));

    // Playfield 2's bits — 15-12 and 7-4 — leave playfield 1 where it was.
    w(&v, BPLCON1, 0xf0f0);
    one(&v);
    assert_eq!(lores(&v, V, 0), distinct(1));
}

#[test]
fn a_64_bit_fetch_scrolls_through_all_63_pixels() {
    // §5's table: at 4× bandwidth a LORES playfield scrolls "0-63" pixels. PF1H7
    // and PF1H6, bits 11 and 10, are 32 and 16 low-resolution pixels.
    let v = lisa();
    setup(&v, 0x1000);
    w(&v, FMODE, 0x0003);
    w(&v, BPLCON1, 0x0c0f);
    // A 64-bit fetch: four words, the lit pixel the very first.
    line(
        &v,
        V,
        0x38,
        [&[0x8000, 0, 0, 0], &[], &[], &[], &[], &[], &[], &[]],
    );
    assert_eq!(lores(&v, V, 32 + 16 + 15), distinct(1));
    assert_eq!(lores(&v, V, 32 + 16 + 14), distinct(0));
    assert_eq!(lores(&v, V, 32 + 16 + 16), distinct(0));
}

#[test]
fn each_fmode_width_is_the_same_bit_stream() {
    // "the parallel to serial conversion is triggered whenever bit plane #1 is
    // written, indicating the completion of all bit planes for that word
    // (16/32/64 pixels). The MSB is output first" (§4, BPLxDAT): a 16-, 32- or
    // 64-bit fetch is that many consecutive pixels, so the stream Alice hands
    // over is one or two or four words a fetch slot, and Lisa shows the same
    // picture for all three.
    let words = [0xa5a5, 0x0ff0, 0xffff, 0x8001];
    let mut pictures = Vec::new();
    // FMODE's BPL32/BPAGEM: 00 is 16 bits, 01 and 10 are 32, 11 is 64 (§4,
    // FMODE, "Bitplane Fetch … By 2 bytes / 4 bytes / 4 bytes / 8 bytes").
    for (fmode, per_fetch) in [(0u16, 1usize), (1, 2), (2, 2), (3, 4)] {
        let v = lisa();
        setup(&v, 0x1000);
        w(&v, FMODE, fmode);
        line(&v, V, 0x38, [&words, &[], &[], &[], &[], &[], &[], &[]]);
        // The first fetch slot's pixels: 16 × the words it carried, in order.
        for k in 0..16 * per_fetch {
            let bit = words[k / 16] >> (15 - k % 16) & 1;
            let expect = if bit != 0 { distinct(1) } else { distinct(0) };
            assert_eq!(lores(&v, V, k as i32), expect, "FMODE {fmode}, pixel {k}");
        }
        let mut row = vec![0u32; v.geometry().0 as usize];
        v.read_row_rgb(2 * u32::from(V - 0x1d), &mut row);
        pictures.push(row);
    }
    assert!(pictures.windows(2).all(|p| p[0] == p[1]));
}

// ---------------------------------------------------------------------------
// sprites
// ---------------------------------------------------------------------------

/// Sprite `n` at `pos`/`ctl` with sixteen-bit `data`/`datb` through the
/// register bus, `CTL` before `DATA` so it ends up armed.
fn sprite(v: &Video, n: u16, pos: u16, ctl: u16, data: u16, datb: u16) {
    w(v, spr_pos(n), pos);
    w(v, spr_pos(n) + 2, ctl);
    w(v, spr_pos(n) + 6, datb);
    w(v, spr_pos(n) + 4, data);
}

/// A window and a palette, no bitplanes: inside the window every pixel is
/// either colour 0 or a sprite.
fn sprites_only(v: &Video) {
    setup(v, 0x0000);
}

#[test]
fn a_sprite_is_positioned_to_the_35ns_quarter() {
    let v = lisa();
    sprites_only(&v);
    // SH10-SH3 = $40 (SPRxPOS 7-0), SH2 = 1 (CTL 0), SH1 = 1 (CTL 4), SH0 = 1
    // (CTL 3): 512 + 4 + 2 + 1 = 519 quarters (§4, SPRxPOS and SPRxCTL).
    sprite(&v, 0, 0x0040, 0x0019, 0x8000, 0);
    line(&v, V, 0x38, Default::default());
    // "ESPRM … Default value is 0001": sprite 0, value 1, is colour 17.
    assert_eq!(quarter(&v, V, 518), distinct(0));
    for q in 519..523 {
        assert_eq!(quarter(&v, V, q), distinct(17), "quarter {q}");
    }
    assert_eq!(quarter(&v, V, 523), distinct(0));
}

#[test]
fn spres_sets_the_sprite_pixel_width_whatever_the_playfield() {
    // A sprite whose pixels go 1, 0, 1: the width of each is the resolution.
    let width_of = |bplcon0: u16, bplcon3: u16| {
        let v = lisa();
        sprites_only(&v);
        w(&v, BPLCON0, bplcon0);
        w(&v, BPLCON3, bplcon3 | PF2OF_DEFAULT);
        sprite(&v, 0, 0x0050, 0, 0xa000, 0);
        line(&v, V, 0x38, Default::default());
        let at = 0x50 << 3;
        let lit: Vec<bool> = (0..16)
            .map(|i| quarter(&v, V, at + i) == distinct(17))
            .collect();
        let first_gap = lit.iter().position(|l| !l).expect("a gap");
        assert!(lit[..first_gap].iter().all(|l| *l));
        first_gap
    };
    // "SPRES … 00 ECS defaults (LORES, HIRES = 140ns, SHRES = 70ns); 01 LORES
    // (140ns); 10 HIRES (70ns); 11 SHRES (35ns)" (§4, BPLCON3).
    assert_eq!(width_of(0x0000, 0), 4, "lores, ECS default");
    assert_eq!(width_of(0x8000, 0), 4, "hires, ECS default");
    assert_eq!(width_of(0x0040, 0), 2, "superhires, ECS default");
    assert_eq!(
        width_of(0x0040, 1 << 6),
        4,
        "140 ns sprites on a superhires screen"
    );
    assert_eq!(width_of(0x0000, 2 << 6), 2, "70 ns");
    assert_eq!(
        width_of(0x0000, 3 << 6),
        1,
        "35 ns sprites on a lores screen"
    );
}

#[test]
fn fmode_makes_a_sprite_16_32_or_64_pixels_wide() {
    // "Sprites are either 16, 32, or 64 bits wide" (§5); SPAGEM and SPR32 are
    // FMODE bits 3 and 2, "By 2 bytes / 4 bytes / 4 bytes / 8 bytes" (§4).
    for (fmode, pixels) in [(0x0u16, 16i32), (0x4, 32), (0x8, 32), (0xc, 64)] {
        let v = lisa();
        sprites_only(&v);
        w(&v, FMODE, fmode);
        w(&v, spr_pos(0), 0x0048);
        w(&v, spr_pos(0) + 2, 0);
        // The wide data comes from Alice in one transfer.
        v.sprite_dma(0, true, 0);
        v.sprite_dma(0, false, u64::MAX);
        line(&v, V, 0x38, Default::default());
        let start = 0x48 << 3;
        let lit = |k: i32| quarter(&v, V, start + 4 * k) == distinct(17);
        assert!(
            (0..pixels).all(lit),
            "FMODE {fmode:#x}: {pixels} pixels lit"
        );
        assert!(!lit(pixels), "FMODE {fmode:#x}: and no more");
    }
}

#[test]
fn a_sixteen_bit_register_write_is_the_top_of_a_wide_buffer() {
    // A processor write still reaches SPRxDATA "at any time" (§4, SPRxDAT);
    // with a 32-bit sprite its word is the first sixteen pixels and the rest
    // are whatever the buffer's low bits hold — zero after the write.
    let v = lisa();
    sprites_only(&v);
    w(&v, FMODE, 0x4);
    sprite(&v, 0, 0x0048, 0, 0xffff, 0);
    line(&v, V, 0x38, Default::default());
    let start = 0x48 << 3;
    assert_eq!(quarter(&v, V, start + 4 * 15), distinct(17));
    assert_eq!(quarter(&v, V, start + 4 * 16), distinct(0));
}

#[test]
fn esprm_and_osprm_relocate_the_sprite_colours() {
    let v = lisa();
    sprites_only(&v);
    // ESPRM = 5, OSPRM = 3: "the 4 high order color table address bits" for
    // even and odd sprites (§4, BPLCON4).
    w(&v, BPLCON4, 0x0053);
    sprite(&v, 0, 0x0048, 0, 0x8000, 0); // even, value 1
    sprite(&v, 1, 0x004a, 0, 0, 0x8000); // odd, value 2
    // Pair 2/3 attached: "In the case of attached sprites OSPRM bits are
    // used." Value %1011: sprite 3 gives the high two bits.
    sprite(&v, 2, 0x004c, 0, 0x8000, 0x8000);
    sprite(&v, 3, 0x004c, ATTACH, 0, 0x8000);
    line(&v, V, 0x38, Default::default());
    assert_eq!(quarter(&v, V, 0x48 << 3), distinct(0x51));
    assert_eq!(quarter(&v, V, 0x4a << 3), distinct(0x32));
    assert_eq!(quarter(&v, V, 0x4c << 3), distinct(0x3b));
}

#[test]
fn attached_sprites_work_in_superhires() {
    // "Sprites can be attatched in any mode (formerly could not attach
    // sprites in the ECS SHRES 35ns resolution mode)" (§5).
    let v = lisa();
    sprites_only(&v);
    w(&v, BPLCON0, SHRES);
    w(&v, BPLCON3, PF2OF_DEFAULT | 3 << 6); // 35 ns sprites
    sprite(&v, 0, 0x0048, 0, 0x8000, 0x4000);
    sprite(&v, 1, 0x0048, ATTACH, 0x4000, 0x8000);
    line(&v, V, 0x38, Default::default());
    // Pixel 0: even %01, odd %10 → %1001 = 9; pixel 1: even %10, odd %01 → 6.
    assert_eq!(quarter(&v, V, 0x48 << 3), distinct(16 | 9));
    assert_eq!(quarter(&v, V, (0x48 << 3) + 1), distinct(16 | 6));
    assert_eq!(quarter(&v, V, (0x48 << 3) + 2), distinct(0));
}

#[test]
fn brdsprt_shows_sprites_in_the_border_once_ecsena_is_set() {
    let v = lisa();
    sprites_only(&v);
    // A sprite at $70, left of the window at $81.
    let border_sprite = |v: &Video| {
        sprite(v, 0, 0x0038, 0, 0x8000, 0);
        line(v, V, 0x38, Default::default());
        quarter(v, V, 0x38 << 3)
    };
    assert_eq!(border_sprite(&v), distinct(0), "outside the window: border");
    // BRDSPRT is "disabled when ESCENA low" (§4, BPLCON3).
    w(&v, BPLCON3, PF2OF_DEFAULT | BRDSPRT);
    assert_eq!(border_sprite(&v), distinct(0));
    w(&v, BPLCON0, ENBPLCN3);
    assert_eq!(border_sprite(&v), distinct(17));
    // And BRDRBLNK blanks the border under the same gate.
    w(&v, BPLCON3, PF2OF_DEFAULT | BRDRBLNK);
    line(&v, V, 0x38, Default::default());
    assert_eq!(quarter(&v, V, X81 - 8), 0);
}

#[test]
fn sscan2_takes_sh10_out_of_the_comparison() {
    // SH10 (SPRxPOS bit 7) set: HSTART = $A0 << 3 = 1280 quarters.
    let at = |fmode: u16, q: i32| {
        let v = lisa();
        sprites_only(&v);
        // Sprites in the border, so the first match shows too.
        w(&v, BPLCON0, ENBPLCN3);
        w(&v, BPLCON3, PF2OF_DEFAULT | BRDSPRT);
        w(&v, FMODE, fmode);
        sprite(&v, 0, 0x00a0, 0, 0x8000, 0);
        line(&v, V, 0x38, Default::default());
        quarter(&v, V, q) == distinct(17)
    };
    assert!(at(0, 1280), "the sprite is where SH10 says");
    assert!(!at(0, 256), "and only there");
    // "If SSCAN2 bit in FMODE is set, then disable SH10 horizontal coincidence
    // detect" (§4, SPRxPOS): the other ten bits match at 256 as well.
    assert!(at(SSCAN2, 256));
    assert!(at(SSCAN2, 1280));
}

/// A beam that stays where a test puts it.
#[derive(Debug)]
struct Fixed(Mutex<BeamPosition>);

impl Beam for Fixed {
    fn position(&self) -> BeamPosition {
        *self.0.lock()
    }
}

#[test]
fn a_wide_sprite_transfer_keeps_its_place_behind_the_ctl_write_before_it() {
    // What Alice's sprite DMA does in one slot: POS and CTL through the
    // register bus — the CTL write disarms the sprite — and then the wide
    // data, which arms it again. With a beam connected all three are timed,
    // and the data must land after the CTL write, not before it.
    let d = Denise::new(&props()).unwrap();
    let v = &**d.video();
    sprites_only(v);
    w(v, FMODE, 0xc);
    let beam = Arc::new(Fixed(Mutex::new(BeamPosition {
        vpos: V,
        hpos: 0x10,
    })));
    v.connect_beam(beam.clone());
    w(v, spr_pos(0), 0x0048);
    w(v, spr_pos(0) + 2, 0);
    v.sprite_dma(0, true, 0);
    v.sprite_dma(0, false, u64::MAX);
    line(v, V, 0x38, Default::default());
    let start = 0x48 << 3;
    assert_eq!(
        quarter(v, V, start),
        distinct(17),
        "armed, and 64 pixels wide"
    );
    assert_eq!(quarter(v, V, start + 4 * 63), distinct(17));

    // And a queued wide transfer survives a snapshot, in its place.
    *beam.0.lock() = BeamPosition {
        vpos: V + 1,
        hpos: 0x10,
    };
    w(v, spr_pos(0) + 2, 0);
    v.sprite_dma(0, false, 0x8000_0000_0000_0001);
    assert!(
        v.state.lock().pending.iter().any(|c| c.offset & WIDE != 0),
        "queued, not applied"
    );
    round_trip(&d);
}

#[test]
fn clxcon2_brings_planes_7_and_8_in_and_a_clxcon_write_clears_it() {
    let clxdat = |v: &Video| CustomChip::read(v, reg(CLXDAT), Origin::cpu());
    let v = lisa();
    setup(&v, EIGHT);
    // Sprite 0 over a pixel whose value is %0100_0001: planes 1 and 7.
    sprite(&v, 0, 0x0040, 1, 0x8000, 0);
    let frame = |v: &Video| show(v, V, 0x38, &[0x41]);

    // Plane 1 enabled, match 1 (CLXCON bits 6 and 0).
    w(&v, CLXCON, 0x0041);
    clxdat(&v);
    frame(&v);
    assert_ne!(clxdat(&v) & 1 << 1, 0, "playfield 1 to sprite 0");

    // ENBP7 with MVBP7 = 0: plane 7 is 1, so playfield 1 no longer matches.
    w(&v, CLXCON2, 0x0040);
    frame(&v);
    assert_eq!(clxdat(&v) & 1 << 1, 0);

    // MVBP7 = 1: it matches again.
    w(&v, CLXCON2, 0x0041);
    frame(&v);
    assert_ne!(clxdat(&v) & 1 << 1, 0);

    // "Contents of this register are reset by a write to CLXCON."
    w(&v, CLXCON2, 0x0040);
    w(&v, CLXCON, 0x0041);
    frame(&v);
    assert_ne!(clxdat(&v) & 1 << 1, 0);
}

// ---------------------------------------------------------------------------
// an old program on a new chip
// ---------------------------------------------------------------------------

/// The same register writes and lines, to an 8362 and to Lisa, and both
/// pictures back: the 8362's as 12-bit words, Lisa's as 24-bit ones.
fn both(scene: impl Fn(&Video)) -> (Vec<Vec<u16>>, Vec<Vec<u32>>) {
    let ocs = Video::new(Standard::Pal);
    let aga = lisa();
    scene(&ocs);
    scene(&aga);
    let mut old = Vec::new();
    let mut new = Vec::new();
    for y in 0..Standard::Pal.height() {
        let mut row = vec![0u16; WIDTH as usize];
        ocs.read_row(y, &mut row);
        old.push(row);
        let mut row = vec![0u32; 2 * WIDTH as usize];
        aga.read_row_rgb(y, &mut row);
        new.push(row);
    }
    (old, new)
}

/// An old program: only 3rd-edition registers, 12-bit colours.
fn old_program(v: &Video, bplcon0: u16) {
    w(v, DIWSTRT, 0x2c81);
    w(v, DIWSTOP, 0x2cc1);
    for i in 0..32 {
        w(v, color(i), 0x0100 + 0x0111 * (i % 8) + (i / 8));
    }
    w(v, BPLCON0, bplcon0);
    w(v, BPLCON2, 0x0024);
    sprite(v, 0, 0x0060, 0, 0xf0f0, 0xff00);
    sprite(v, 2, 0x0070, 0, 0xffff, 0x0f0f);
    sprite(v, 3, 0x0070, ATTACH, 0x3333, 0x5555);
    let planes: [&[u16]; 8] = [
        &[0xff00, 0x0ff0, 0xf00f, 0x1234],
        &[0xf0f0, 0xcccc, 0xaaaa, 0x5678],
        &[0xcccc, 0xffff, 0x0000, 0x9abc],
        &[0xaaaa, 0x00ff, 0xff00, 0xdef0],
        &[0x5555, 0xf0f0, 0x0f0f, 0x1357],
        &[],
        &[],
        &[],
    ];
    for vpos in 0..313 {
        line(v, vpos, 0x38, planes);
    }
}

#[test]
fn an_old_program_shows_the_8362s_colours_on_lisa() {
    // The two resets that make this work are the specification's own: PF2OF
    // defaults to 8 and ESPRM/OSPRM to 0001, "so that old copper lists" keep
    // their colours. Five planes, and then dual playfields.
    for bplcon0 in [0x5000u16, 0x6400] {
        let (old, new) = both(|v| old_program(v, bplcon0));
        for (y, (a, b)) in old.iter().zip(&new).enumerate() {
            for (c, px) in a.iter().enumerate() {
                // An 8362's column is a high-resolution half pixel: two
                // quarters.
                let want = rgb12(*px);
                assert_eq!(
                    b[2 * c],
                    want,
                    "BPLCON0 {bplcon0:#06x}, row {y}, column {c}"
                );
                assert_eq!(b[2 * c + 1], want);
            }
        }
    }
}

#[test]
fn an_old_ham6_or_ehb_picture_is_the_same_on_the_twelve_old_pins() {
    // HAM6 holds the low nibble and EHB halves eight-bit guns, so the low
    // halves differ from an 8362's n × 17; the top four bits of each gun —
    // what the 8362's twelve pins carried — do not.
    for bplcon0 in [0x5800u16, 0x6800, 0x6000] {
        let (old, new) = both(|v| old_program(v, bplcon0));
        for (a, b) in old.iter().zip(&new) {
            for (c, px) in a.iter().enumerate() {
                assert_eq!(rgb12_of(b[2 * c]), *px, "BPLCON0 {bplcon0:#06x}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

fn props() -> Props {
    Props::new()
        .with("custom", Value::Link(Link::new("custom").unwrap()))
        .with("revision", Value::from("aga"))
}

#[test]
fn revision_aga_is_a_property_value() {
    let d = Denise::new(&props()).expect("aga is accepted");
    assert_eq!(d.video().revision(), Revision::Aga);
    assert_eq!(
        d.video().geometry(),
        (1600, 568),
        "35 ns columns from power-on"
    );
    let bad = Props::new()
        .with("custom", Value::Link(Link::new("custom").unwrap()))
        .with("revision", Value::from("aaa"));
    assert!(Denise::new(&bad).is_err());
}

#[test]
fn a_reset_restores_the_aa_defaults() {
    let d = Denise::new(&props()).unwrap();
    let v = d.video();
    w(v, BPLCON3, 0);
    w(v, BPLCON4, 0xff00);
    w(v, FMODE, 0xc00f);
    Device::reset(&d, ResetKind::Cold);
    let st = v.state.lock();
    // "RST_pin resets all bits in all registers new to AA" (§2), and the
    // register pages give PF2OF = 011 and ESPRM = OSPRM = 0001.
    assert_eq!(st.regs.bplcon3, PF2OF_DEFAULT);
    assert_eq!(st.regs.bplcon4, SPRM_DEFAULT);
    assert_eq!(st.regs.fmode, 0);
}

fn snapshot(d: &Denise) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("denise", CLASS_NAME).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("denise", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(d, &mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

fn restore(into: &Denise, bytes: &[u8]) -> Result<()> {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("denise", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(into, &mut chunk.reader())
}

/// Save `saved`, load it into a fresh Lisa, and check the two agree — then
/// and after one more line each.
fn round_trip(saved: &Denise) {
    let bytes = snapshot(saved);
    let restored = Denise::new(&props()).unwrap();
    restore(&restored, &bytes).expect("a Lisa snapshot loads into a Lisa");
    assert_eq!(
        snapshot(&restored),
        bytes,
        "identical state after a round trip"
    );
    for d in [saved, &restored] {
        show(d.video(), V + 1, 0x38, &[0x15; 40]);
    }
    assert_eq!(
        snapshot(&restored),
        snapshot(saved),
        "and it carries on identically"
    );
}

#[test]
fn a_lisa_snapshot_round_trips_to_identical_state() {
    let saved = Denise::new(&props()).unwrap();
    let v = saved.video();
    // The whole table through every bank, then the AA registers, a T bit, a
    // half-written LOCT pass and a picture.
    setup(v, 0x5000);
    w(v, BPLCON4, 0x3c5a);
    w(v, FMODE, 0x000f);
    w(v, CLXCON2, 0x00c3);
    w(v, BPLCON3, PF2OF_DEFAULT | LOCT | 0xa000);
    w(v, COLOR00, 0x8abc);
    show(v, V, 0x38, &(0..32).collect::<Vec<u8>>());
    v.field(false);
    round_trip(&saved);
}

#[test]
fn a_lisa_snapshot_keeps_the_wide_sprite_buffers() {
    let saved = Denise::new(&props()).unwrap();
    let v = saved.video();
    setup(v, EIGHT | HOMOD);
    w(v, FMODE, 0x000c);
    v.sprite_dma(5, false, 0x0123_4567_89ab_cdef);
    v.sprite_dma(5, true, 0xfedc_ba98_7654_3210);
    show(v, V, 0x38, &(0..=255).collect::<Vec<u8>>());
    round_trip(&saved);
}

#[test]
fn a_lisa_snapshot_is_refused_by_an_8373_and_the_other_way_round() {
    let ecs_props = || {
        Props::new()
            .with("custom", Value::Link(Link::new("custom").unwrap()))
            .with("revision", Value::from("ecs"))
    };
    let aga = Denise::new(&props()).unwrap();
    let ecs = Denise::new(&ecs_props()).unwrap();
    assert!(restore(&ecs, &snapshot(&aga)).is_err());
    assert!(restore(&aga, &snapshot(&ecs)).is_err());
}
