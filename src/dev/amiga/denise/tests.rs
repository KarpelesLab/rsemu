//! Denise, driven the way Agnus will drive it: register writes through
//! [`CustomChip`], lines through [`Video::line`], fields through
//! [`Video::field`].
//!
//! Every expected pixel below is worked out from the manual's own numbers — the
//! data-fetch arithmetic of Chapter 3, Tables 3-17 to 3-19, Table 4-6 and
//! Tables 7-2 to 7-4 — and the comment beside it says which.

use super::*;
use crate::core::props::{Link, Value};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::amiga::custom::Custom;
use crate::dev::amiga::regs;

/// A PAL line's length in colour clocks.
const CLOCKS: u16 = 227;

/// A line inside the standard PAL window (`$2C` to `$12C`).
const V: u16 = 100;

/// The standard window: Chapter 3, "The normal PAL DIWSTRT is ($2C81)" and
/// "DIWSTOP is ($2CC1)".
const PAL_DIWSTRT: u16 = 0x2c81;
const PAL_DIWSTOP: u16 = 0x2cc1;

fn video() -> Video {
    Video::new(Standard::Pal)
}

fn reg(offset: u16) -> &'static Reg {
    regs::lookup(offset).expect("a register Appendix B declares")
}

/// A processor write.
fn w(v: &Video, offset: u16, value: u16) {
    CustomChip::write(v, reg(offset), value, Origin::cpu());
}

fn color(i: u16) -> u16 {
    COLOR00 + 2 * i
}

/// A line with no bitplane data.
fn blank(v: &Video, vpos: u16) {
    v.line(&Line {
        vpos,
        clocks: CLOCKS,
        fetch: Fetch::default(),
    });
}

/// A line whose fetch began at `start` with `planes` words.
fn fetched(v: &Video, vpos: u16, start: u16, planes: [&[u16]; 6]) {
    v.line(&Line {
        vpos,
        clocks: CLOCKS,
        fetch: Fetch { start, planes },
    });
}

/// The picture at line `vpos`, low-resolution `x`, half-pixel `sub`.
fn px(v: &Video, vpos: u16, x: u16, sub: u16) -> u16 {
    let row = 2 * u32::from(vpos - Standard::Pal.first_line());
    let mut words = vec![0u16; WIDTH as usize];
    v.read_row(row, &mut words);
    words[usize::from(2 * (x - OUTPUT_LEFT) + sub)]
}

/// Both half-pixels of a low-resolution pixel, which must agree.
fn lores(v: &Video, vpos: u16, x: u16) -> u16 {
    let a = px(v, vpos, x, 0);
    assert_eq!(
        a,
        px(v, vpos, x, 1),
        "a low-resolution pixel is two equal halves"
    );
    a
}

/// A window, a palette whose every entry is distinct, and `bplcon0`.
fn setup(v: &Video, bplcon0: u16) {
    w(v, DIWSTRT, PAL_DIWSTRT);
    w(v, DIWSTOP, PAL_DIWSTOP);
    for i in 0..32 {
        w(v, color(i), 0x0100 + i);
    }
    w(v, BPLCON0, bplcon0);
}

// ---------------------------------------------------------------------------
// colour and the playfield
// ---------------------------------------------------------------------------

#[test]
fn the_border_is_colour_zero_and_the_top_nibble_goes_nowhere() {
    let v = video();
    w(&v, COLOR00, 0xf6fe);
    blank(&v, V);
    // "Bits 15 - 12 Unused" (Table 3-3).
    assert_eq!(lores(&v, V, OUTPUT_LEFT), 0x06fe);
    assert_eq!(lores(&v, V, 300), 0x06fe);
}

#[test]
fn a_low_resolution_word_fetched_at_38_lands_on_the_window_at_81() {
    let v = video();
    setup(&v, 0x2200); // two planes, COLOR on — Chapter 3's own example
    // "$81/2 - 8.5 = $38": the first bit of a word fetched at $38 is x = $81.
    fetched(&v, V, 0x38, [&[0x8000], &[0x4000], &[], &[], &[], &[]]);
    assert_eq!(lores(&v, V, 0x81), 0x0101, "plane 1 alone selects COLOR01");
    assert_eq!(lores(&v, V, 0x82), 0x0102, "plane 2 alone selects COLOR02");
    assert_eq!(lores(&v, V, 0x83), 0x0100);
}

#[test]
fn the_window_clips_the_playfield_on_both_axes() {
    let v = video();
    setup(&v, 0x1200);
    // One word early: its sixteen pixels are x = $71..$80, all left of the
    // window, and the second word's first pixel is the window's first.
    fetched(&v, V, 0x30, [&[0xffff, 0x8000], &[], &[], &[], &[], &[]]);
    for x in 0x71..=0x80 {
        assert_eq!(lores(&v, V, x), 0x0100, "x = {x:#x} is border");
    }
    assert_eq!(lores(&v, V, 0x81), 0x0101);

    // HSTOP is $1C1: "the normal HSTOP value is ($1C1) but is written as
    // ($C1)". A word whose pixels straddle it is cut there.
    let v = video();
    setup(&v, 0x1200);
    let start = (0x1c1 - 17 - 8) / 2; // pixels $1B9..$1C8
    fetched(&v, V, start, [&[0xffff], &[], &[], &[], &[], &[]]);
    assert_eq!(lores(&v, V, 0x1c0), 0x0101);
    assert_eq!(lores(&v, V, 0x1c1), 0x0100);

    // Vertically: VSTART $2C, and VSTOP $2C whose V8 is "the complement of
    // the next MSB" — line $12C.
    for (vpos, inside) in [(0x2b, false), (0x2c, true), (0x12b, true), (0x12c, false)] {
        fetched(&v, vpos, 0x38, [&[0x8000], &[], &[], &[], &[], &[]]);
        let want = if inside { 0x0101 } else { 0x0100 };
        assert_eq!(lores(&v, vpos, 0x81), want, "line {vpos:#x}");
    }
}

#[test]
fn a_high_resolution_word_fetched_at_3c_lands_at_81_a_pixel_per_half() {
    let v = video();
    setup(&v, 0xa200); // HIRES, two planes
    // "$81/2 - 4.5 = $3C".
    fetched(&v, V, 0x3c, [&[0xa000], &[], &[], &[], &[], &[]]);
    assert_eq!(px(&v, V, 0x81, 0), 0x0101);
    assert_eq!(px(&v, V, 0x81, 1), 0x0100);
    assert_eq!(px(&v, V, 0x82, 0), 0x0101);
    assert_eq!(px(&v, V, 0x82, 1), 0x0100);
}

#[test]
fn bplcon1_delays_each_playfield_by_its_own_count() {
    let v = video();
    setup(&v, 0x2200);
    // PF1H = 3, PF2H = 1 (Chapter 3, "Specifying Amount of Delay").
    w(&v, BPLCON1, 0x0013);
    fetched(&v, V, 0x38, [&[0x8000], &[0x8000], &[], &[], &[], &[]]);
    assert_eq!(lores(&v, V, 0x81), 0x0100);
    assert_eq!(lores(&v, V, 0x82), 0x0102, "plane 2, one pixel late");
    assert_eq!(lores(&v, V, 0x84), 0x0101, "plane 1, three pixels late");

    // In high resolution a unit is still a low-resolution pixel: "scrolling is
    // in increments of 2 pixels".
    let v = video();
    setup(&v, 0x9200);
    w(&v, BPLCON1, 0x0001);
    fetched(&v, V, 0x3c, [&[0x8000], &[], &[], &[], &[], &[]]);
    assert_eq!(px(&v, V, 0x81, 0), 0x0100);
    assert_eq!(px(&v, V, 0x82, 0), 0x0101);
}

#[test]
fn five_planes_select_all_thirty_two_registers() {
    let v = video();
    setup(&v, 0x5200);
    // Figure 3-4's four sample pixels, planes 5 down to 1:
    // 11100, 10011, 01011, 00110 -> COLOR28, COLOR18, COLOR11, COLOR6.
    let p1 = [0b0010u16 << 12];
    let p2 = [0b0111u16 << 12];
    let p3 = [0b1001u16 << 12];
    let p4 = [0b1010u16 << 12];
    let p5 = [0b1100u16 << 12];
    fetched(&v, V, 0x38, [&p1, &p2, &p3, &p4, &p5, &[]]);
    assert_eq!(lores(&v, V, 0x81), 0x0100 + 28);
    assert_eq!(lores(&v, V, 0x82), 0x0100 + 18);
    assert_eq!(lores(&v, V, 0x83), 0x0100 + 11);
    assert_eq!(lores(&v, V, 0x84), 0x0100 + 6);
}

#[test]
fn planes_bplcon0_does_not_enable_are_not_seen() {
    let v = video();
    setup(&v, 0x1200); // one plane
    fetched(&v, V, 0x38, [&[0x8000], &[0xc000], &[], &[], &[], &[]]);
    assert_eq!(lores(&v, V, 0x81), 0x0101);
    assert_eq!(lores(&v, V, 0x82), 0x0100, "plane 2 was fetched but is off");
}

#[test]
fn dual_playfields_take_their_own_registers_and_pf2pri_swaps_them() {
    let v = video();
    setup(&v, 0x6600); // six planes, DBLPF
    // x = $81: planes 1 and 5 -> playfield 1 value 101 = COLOR05.
    // x = $82: plane 4 -> playfield 2 value 010 = COLOR10.
    // x = $83: both at once.
    let p1 = [0xa000u16];
    let p4 = [0x6000u16];
    let p5 = [0xa000u16];
    fetched(&v, V, 0x38, [&p1, &[], &[], &p4, &p5, &[]]);
    assert_eq!(lores(&v, V, 0x81), 0x0105, "Table 3-10");
    assert_eq!(lores(&v, V, 0x82), 0x010a, "Table 3-11");
    assert_eq!(
        lores(&v, V, 0x83),
        0x0105,
        "playfield 1 normally has priority"
    );
    assert_eq!(lores(&v, V, 0x84), 0x0100, "000 in both is transparent");

    w(&v, BPLCON2, 0x0040);
    fetched(&v, V + 1, 0x38, [&p1, &[], &[], &p4, &p5, &[]]);
    assert_eq!(lores(&v, V + 1, 0x83), 0x010a, "PF2PRI");
}

#[test]
fn dual_playfields_in_high_resolution_use_two_planes_each() {
    let v = video();
    setup(&v, 0xc600); // HIRES, four planes, DBLPF
    // Table 3-12: planes 3,1 -> COLOR1-3; planes 4,2 -> COLOR9-11.
    fetched(
        &v,
        V,
        0x3c,
        [&[0x8000], &[0x4000], &[0x8000], &[0x4000], &[], &[]],
    );
    assert_eq!(px(&v, V, 0x81, 0), 0x0103);
    assert_eq!(px(&v, V, 0x81, 1), 0x010b);
}

#[test]
fn hold_and_modify_holds_the_pixel_to_the_left_and_changes_one_gun() {
    let v = video();
    setup(&v, 0x6a00); // six planes, HOMOD
    w(&v, color(3), 0x0123);
    // Low-resolution pixels at $81.., as (plane 6, plane 5, planes 4-1):
    //   00 0011  COLOR03                     -> $123
    //   01 1111  hold, blue  = $F  (Table 3-19) -> $12F
    //   10 1010  hold, red   = $A            -> $A2F
    //   11 0000  hold, green = $0            -> $A0F
    //   00 0001  COLOR01                     -> $101
    let p1 = [0b11001u16 << 11];
    let p2 = [0b11100u16 << 11];
    let p3 = [0b01000u16 << 11];
    let p4 = [0b01100u16 << 11];
    let p5 = [0b01010u16 << 11];
    let p6 = [0b00110u16 << 11];
    fetched(&v, V, 0x38, [&p1, &p2, &p3, &p4, &p5, &p6]);
    assert_eq!(lores(&v, V, 0x81), 0x0123);
    assert_eq!(lores(&v, V, 0x82), 0x012f);
    assert_eq!(lores(&v, V, 0x83), 0x0a2f);
    assert_eq!(lores(&v, V, 0x84), 0x0a0f);
    assert_eq!(lores(&v, V, 0x85), 0x0101);
}

#[test]
fn hold_and_modify_needs_low_resolution_single_playfield_and_five_planes() {
    // Chapter 3 lists all four conditions. With HIRES set it is not HAM, and
    // plane 5 simply selects COLOR16 and up.
    let v = video();
    setup(&v, 0xda00); // HIRES + five planes + HOMOD
    fetched(&v, V, 0x3c, [&[], &[], &[], &[], &[0x8000], &[]]);
    assert_eq!(px(&v, V, 0x81, 0), 0x0110);

    // With five planes it is HAM, and plane 6 reads as zero: "If only five
    // bitplanes are used, the data from the sixth plane is automatically
    // supplied with the value as 0."
    let v = video();
    setup(&v, 0x5a00);
    w(&v, color(2), 0x0456);
    fetched(
        &v,
        V,
        0x38,
        [&[0x4000], &[0x8000], &[], &[], &[0x4000], &[0xffff]],
    );
    assert_eq!(lores(&v, V, 0x81), 0x0456, "00 0010: COLOR02");
    assert_eq!(lores(&v, V, 0x82), 0x0451, "01 0001: blue = 1");
}

#[test]
fn extra_half_brite_halves_each_gun() {
    let v = video();
    setup(&v, 0x6200); // six planes, no HOMOD, no DBLPF
    w(&v, color(5), 0x0fa4);
    let p1 = [0xc000u16];
    let p3 = [0xc000u16];
    let p6 = [0x4000u16];
    fetched(&v, V, 0x38, [&p1, &[], &p3, &[], &[], &p6]);
    assert_eq!(lores(&v, V, 0x81), 0x0fa4);
    // "shifted to half-intensity by the sixth bitplane": F/2, A/2, 4/2.
    assert_eq!(lores(&v, V, 0x82), 0x0752);
}

// ---------------------------------------------------------------------------
// sprites
// ---------------------------------------------------------------------------

/// Put sprite `n` at low-resolution `x` with the given word pair, the way
/// Chapter 4's "Manual Mode" says: position, control, B, then A to arm.
fn sprite(v: &Video, n: u16, x: u16, data: u16, datb: u16, ctl: u16) {
    let base = SPR0POS + 8 * n;
    w(v, base, x >> 1);
    w(v, base + 2, ctl | (x & 1));
    w(v, base + 6, datb);
    w(v, base + 4, data);
}

#[test]
fn a_sprite_appears_at_its_nine_bit_hstart_in_its_pairs_colours() {
    let v = video();
    setup(&v, 0x0200);
    // HSTART = $A1: SH8-SH1 in SPRxPOS, SH0 in SPRxCTL bit 0.
    sprite(&v, 0, 0xa1, 0x8001, 0x0001, 0);
    blank(&v, V);
    assert_eq!(lores(&v, V, 0xa0), 0x0100);
    assert_eq!(lores(&v, V, 0xa1), 0x0111, "01: COLOR17 (Table 4-6)");
    assert_eq!(lores(&v, V, 0xa2), 0x0100, "00 is transparent");
    assert_eq!(lores(&v, V, 0xb0), 0x0113, "11: COLOR19, sixteen pixels on");
    assert_eq!(lores(&v, V, 0xb1), 0x0100, "and nothing after");

    // Sprite 5 is in the third pair: COLOR25-27.
    sprite(&v, 5, 0xc0, 0x0000, 0x8000, 0);
    blank(&v, V + 1);
    assert_eq!(lores(&v, V + 1, 0xc0), 0x011a, "10: COLOR26");
}

#[test]
fn ctl_disarms_data_rearms_and_an_armed_sprite_repeats_on_every_line() {
    let v = video();
    setup(&v, 0x0200);
    sprite(&v, 0, 0xa0, 0x8000, 0, 0);
    blank(&v, V);
    blank(&v, V + 1);
    assert_eq!(lores(&v, V, 0xa0), 0x0111);
    assert_eq!(
        lores(&v, V + 1, 0xa0),
        0x0111,
        "a vertical bar, as the manual warns"
    );

    w(&v, SPR0POS + 2, 0);
    blank(&v, V + 2);
    assert_eq!(lores(&v, V + 2, 0xa0), 0x0100, "writing SPRxCTL disarms");

    w(&v, SPR0POS + 4, 0x8000);
    blank(&v, V + 3);
    assert_eq!(lores(&v, V + 3, 0xa0), 0x0111, "writing SPRxDATA arms");

    // A DMA write is the same register write.
    CustomChip::write(&v, reg(SPR0POS + 2), 0, Origin::dma());
    blank(&v, V + 4);
    assert_eq!(lores(&v, V + 4, 0xa0), 0x0100);
}

#[test]
fn lower_numbered_sprites_are_in_front() {
    let v = video();
    setup(&v, 0x0200);
    sprite(&v, 2, 0xa0, 0x8000, 0, 0); // COLOR21
    sprite(&v, 1, 0xa0, 0x8000, 0, 0); // COLOR17
    sprite(&v, 0, 0xa0, 0, 0x8000, 0); // COLOR18
    blank(&v, V);
    assert_eq!(lores(&v, V, 0xa0), 0x0112, "sprite 0 in front of 1 and 2");
    w(&v, SPR0POS + 2, 0);
    blank(&v, V + 1);
    assert_eq!(lores(&v, V + 1, 0xa0), 0x0111, "then sprite 1");
}

#[test]
fn attached_sprites_are_one_four_bit_object() {
    let v = video();
    setup(&v, 0x0200);
    // Sprite 1 attached to sprite 0 (ATT, bit 7, on the odd sprite).
    sprite(&v, 0, 0xa0, 0x8000, 0x0000, 0); // low two bits 01
    sprite(&v, 1, 0xa0, 0xc000, 0x8000, ATTACH); // high two bits 11, then 01
    blank(&v, V);
    // Table 4-7: 1101 -> COLOR29; the next pixel 0100 -> COLOR20, which is
    // "the higher numbered sprite ... from color registers 20, 24, and 28".
    assert_eq!(lores(&v, V, 0xa0), 0x011d);
    assert_eq!(lores(&v, V, 0xa1), 0x0114);
}

#[test]
fn in_a_single_playfield_pf2p_places_it_among_the_sprites() {
    let v = video();
    setup(&v, 0x1200);
    sprite(&v, 0, 0x81, 0x8000, 0, 0);
    let planes: [&[u16]; 6] = [&[0xffff], &[], &[], &[], &[], &[]];

    fetched(&v, V, 0x38, planes);
    assert_eq!(lores(&v, V, 0x81), 0x0101, "code 0: playfield in front");

    // PF1P = 4 changes nothing: "PF2P2 - PF2P0, bits 5-3, are the priority
    // bits for normal (non-dual) playfields."
    w(&v, BPLCON2, 0x0004);
    fetched(&v, V + 1, 0x38, planes);
    assert_eq!(lores(&v, V + 1, 0x81), 0x0101);

    w(&v, BPLCON2, 0x0008); // PF2P = 1
    fetched(&v, V + 2, 0x38, planes);
    assert_eq!(lores(&v, V + 2, 0x81), 0x0111, "SP01 PF SP23 ...");
    assert_eq!(
        lores(&v, V + 2, 0x82),
        0x0101,
        "a transparent sprite pixel shows the playfield"
    );

    // A sprite behind a playfield shows through its colour-0 pixels.
    w(&v, BPLCON2, 0);
    fetched(&v, V + 3, 0x38, [&[0x7fff], &[], &[], &[], &[], &[]]);
    assert_eq!(lores(&v, V + 3, 0x81), 0x0111);
}

#[test]
fn chapter_sevens_unusual_priority_example() {
    // BPLCON2 = PF2PRI | PF2P 010 | PF1P 000: "PF1 SP01 SP23 PF2 SP45 SP67",
    // with playfield 2 in front of playfield 1 "where playfield 2 is not
    // blocked by sprites 0 through 3".
    let v = video();
    setup(&v, 0x2600); // two planes, dual: plane 1 is PF1, plane 2 is PF2
    w(&v, BPLCON2, 0x0050);
    sprite(&v, 0, 0x81, 0xc000, 0, 0); // x = $81, $82
    sprite(&v, 4, 0x85, 0x8000, 0, 0); // x = $85
    // x:      81   82   83   84   85
    // PF1:     1    0    1    0    0
    // PF2:     1    1    1    0    1
    let pf1 = [0xa000u16];
    let pf2 = [0xe800u16];
    fetched(&v, V, 0x38, [&pf1, &pf2, &[], &[], &[], &[]]);
    assert_eq!(
        lores(&v, V, 0x81),
        0x0101,
        "sprite 0 blocks PF2; PF1 is in front of it"
    );
    assert_eq!(lores(&v, V, 0x82), 0x0111, "sprite 0 in front of PF2");
    assert_eq!(lores(&v, V, 0x83), 0x0109, "PF2 in front of PF1");
    assert_eq!(lores(&v, V, 0x85), 0x0109, "PF2 in front of sprite 4");
}

// ---------------------------------------------------------------------------
// collisions
// ---------------------------------------------------------------------------

fn clxdat(v: &Video, debug: bool) -> u16 {
    CustomChip::read(v, reg(CLXDAT), Origin::cpu().for_debug(debug))
}

#[test]
fn a_sprite_over_a_matching_plane_collides_and_the_read_clears_it() {
    let v = video();
    setup(&v, 0x1200);
    w(&v, CLXCON, 0x0041); // ENBP1, MVBP1 = 1
    sprite(&v, 0, 0x90, 0x8000, 0, 0);
    fetched(&v, V, 0x38, [&[0xffff], &[], &[], &[], &[], &[]]); // $81..$90
    // Bit 1, "Playfield 1 to sprite 0 (or 1)"; bit 5, "Even bitplanes to
    // sprite 0", too, because no even plane is enabled to prevent it.
    assert_eq!(clxdat(&v, true), 0x0023, "a debugger sees it");
    assert_eq!(clxdat(&v, true), 0x0023, "and seeing it cleared nothing");
    assert_eq!(clxdat(&v, false), 0x0023);
    assert_eq!(clxdat(&v, false), 0, "read and clear (Chapter 7)");
}

#[test]
fn a_disabled_plane_cannot_prevent_a_collision() {
    let v = video();
    setup(&v, 0x0200);
    w(&v, CLXCON, 0);
    blank(&v, V);
    // "if all bitplanes are disabled, collisions will be continuous".
    assert_eq!(clxdat(&v, false), 0x0001);

    // Match value 1 with the plane enabled, and nothing drawn: no playfield
    // collision anywhere.
    w(&v, CLXCON, 0x0fff);
    blank(&v, V + 1);
    assert_eq!(clxdat(&v, false), 0);
}

#[test]
fn odd_sprites_count_only_when_enabled_and_groups_collide_with_each_other() {
    let v = video();
    setup(&v, 0x0200);
    w(&v, CLXCON, 0x0fc0); // every plane enabled, match value 0: matches empty playfield
    sprite(&v, 1, 0xa0, 0x8000, 0, 0);
    sprite(&v, 2, 0xa0, 0x8000, 0, 0);
    blank(&v, V);
    // Sprite 1 is not counted without ENSP1: only group 1 (sprite 2) against
    // both playfields — bits 2 and 6 — and the playfields against each other.
    assert_eq!(clxdat(&v, false), 0x0045);

    w(&v, CLXCON, 0x1fc0); // ENSP1
    blank(&v, V + 1);
    // Now group 0 too: bits 1 and 5, and bit 9, "Sprite 0 (or 1) to sprite 2
    // (or 3)".
    assert_eq!(clxdat(&v, false), 0x0267);
}

#[test]
fn nothing_collides_outside_the_window() {
    let v = video();
    setup(&v, 0x0200);
    w(&v, CLXCON, 0);
    blank(&v, 0x20); // above VSTART
    assert_eq!(clxdat(&v, false), 0);
}

// ---------------------------------------------------------------------------
// the beam
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct Fixed(Mutex<BeamPosition>);

impl Fixed {
    fn at(&self, vpos: u16, hpos: u16) {
        *self.0.lock() = BeamPosition { vpos, hpos };
    }
}

impl Beam for Fixed {
    fn position(&self) -> BeamPosition {
        *self.0.lock()
    }
}

#[test]
fn without_a_beam_a_write_lands_on_the_next_line_rendered() {
    let v = video();
    w(&v, COLOR00, 0x0111);
    blank(&v, V);
    w(&v, COLOR00, 0x0222);
    assert_eq!(
        lores(&v, V, 0x100),
        0x0111,
        "the line already drawn keeps its colour"
    );
    blank(&v, V + 1);
    assert_eq!(lores(&v, V + 1, OUTPUT_LEFT), 0x0222);
}

#[test]
fn with_a_beam_a_mid_line_write_lands_mid_line() {
    let v = video();
    let beam = Arc::new(Fixed(Mutex::new(BeamPosition::default())));
    v.connect_beam(beam.clone());

    beam.at(V, 0);
    w(&v, COLOR00, 0x0111);
    // A copper MOVE at hpos $60 on this line: x = $C0.
    beam.at(V, 0x60);
    w(&v, COLOR00, 0x0f00);
    // And one on a line not yet drawn.
    beam.at(V + 2, 0x70);
    w(&v, COLOR00, 0x00f0);

    blank(&v, V);
    assert_eq!(lores(&v, V, 0xbf), 0x0111);
    assert_eq!(lores(&v, V, 0xc0), 0x0f00);
    blank(&v, V + 1);
    assert_eq!(
        lores(&v, V + 1, 0x100),
        0x0f00,
        "nothing was due on this line"
    );
    blank(&v, V + 2);
    assert_eq!(lores(&v, V + 2, 0xdf), 0x0f00);
    assert_eq!(lores(&v, V + 2, 0xe0), 0x00f0);
}

#[test]
fn a_write_stamped_past_the_end_of_its_line_opens_the_next_one() {
    let v = video();
    let beam = Arc::new(Fixed(Mutex::new(BeamPosition::default())));
    v.connect_beam(beam.clone());
    beam.at(V, 250);
    w(&v, COLOR00, 0x0abc);
    blank(&v, V);
    assert_eq!(lores(&v, V, 0x1c0), 0, "hpos 250 is past a 227-clock line");
    blank(&v, V + 1);
    assert_eq!(lores(&v, V + 1, OUTPUT_LEFT), 0x0abc);
}

#[test]
fn a_mid_line_sprite_arm_takes_effect_from_its_pixel() {
    // Arm at hpos $50 (x = $A0): a sprite at $90 has already been passed, one
    // at $B0 has not.
    let v = video();
    setup(&v, 0x0200);
    let beam = Arc::new(Fixed(Mutex::new(BeamPosition::default())));
    v.connect_beam(beam.clone());
    beam.at(V, 0);
    w(&v, SPR0POS, 0x90 >> 1);
    w(&v, SPR0POS + 8, 0xb0 >> 1);
    beam.at(V, 0x50);
    w(&v, SPR0POS + 4, 0x8000);
    w(&v, SPR0POS + 12, 0x8000);
    blank(&v, V);
    assert_eq!(lores(&v, V, 0x90), 0x0100);
    assert_eq!(lores(&v, V, 0xb0), 0x0111);
}

/// A beam that, like a lazily advanced Agnus, catches up by pushing a line
/// into Denise from inside `position` — which is called from inside a register
/// write.
#[derive(Debug)]
struct CatchUp {
    video: Mutex<Weak<Video>>,
    pushed: Mutex<u16>,
}

impl Beam for CatchUp {
    fn position(&self) -> BeamPosition {
        let video = self.video.lock().upgrade();
        let mut pushed = self.pushed.lock();
        let vpos = *pushed;
        *pushed += 1;
        drop(pushed);
        if let Some(v) = video {
            blank(&v, vpos);
        }
        BeamPosition {
            vpos: vpos + 1,
            hpos: 0,
        }
    }
}

#[test]
fn a_beam_may_push_lines_from_inside_a_register_write() {
    let v = Arc::new(video());
    let beam = Arc::new(CatchUp {
        video: Mutex::new(Arc::downgrade(&v)),
        pushed: Mutex::new(V),
    });
    v.connect_beam(beam);
    // Would deadlock if Denise held its own lock while asking the beam.
    w(&v, COLOR00, 0x0555);
    assert_eq!(
        lores(&v, V, OUTPUT_LEFT),
        0,
        "the caught-up line predates the write"
    );
    blank(&v, V + 1);
    assert_eq!(lores(&v, V + 1, OUTPUT_LEFT), 0x0555);
}

// ---------------------------------------------------------------------------
// fields, interlace, geometry
// ---------------------------------------------------------------------------

#[test]
fn a_non_interlaced_line_fills_two_rows_and_an_interlaced_field_one() {
    let v = video();
    let row = |v: &Video, y: u32| {
        let mut words = vec![0u16; WIDTH as usize];
        v.read_row(y, &mut words);
        words[0]
    };
    let base = 2 * u32::from(V - Standard::Pal.first_line());

    w(&v, COLOR00, 0x0111);
    blank(&v, V);
    assert_eq!((row(&v, base), row(&v, base + 1)), (0x0111, 0x0111));

    w(&v, BPLCON0, LACE);
    w(&v, COLOR00, 0x0222);
    v.field(true);
    blank(&v, V);
    assert_eq!(
        (row(&v, base), row(&v, base + 1)),
        (0x0222, 0x0111),
        "long field: even rows"
    );
    w(&v, COLOR00, 0x0333);
    v.field(false);
    blank(&v, V);
    assert_eq!(
        (row(&v, base), row(&v, base + 1)),
        (0x0222, 0x0333),
        "short field: odd rows"
    );
}

#[test]
fn lines_in_vertical_blank_draw_nothing_and_fields_count_their_clocks() {
    let v = video();
    w(&v, COLOR00, 0x0fff);
    for vpos in 0..312 {
        blank(&v, vpos);
    }
    assert_eq!(v.fields(), 0);
    assert_eq!(v.field_clocks(), 0, "no field has finished");
    v.field(true);
    assert_eq!(v.fields(), 1);
    assert_eq!(v.field_clocks(), 312 * u64::from(CLOCKS));

    let mut top = vec![0u16; WIDTH as usize];
    v.read_row(0, &mut top);
    assert!(
        top.iter().all(|&p| p == 0x0fff),
        "line $1D is the first row"
    );
}

#[test]
fn geometry_follows_the_standard() {
    assert_eq!(Video::new(Standard::Pal).geometry(), (800, 568));
    assert_eq!(Video::new(Standard::Ntsc).geometry(), (800, 484));
    let mut words = vec![7u16; 4];
    Video::new(Standard::Ntsc).read_row(484, &mut words);
    assert_eq!(words, [7; 4], "past the bottom leaves dst alone");
}

// ---------------------------------------------------------------------------
// the rest of the register block
// ---------------------------------------------------------------------------

#[test]
fn joytest_writes_the_top_six_bits_of_all_four_counters() {
    let v = video();
    w(&v, JOYTEST, 0xffff);
    let joy0 = CustomChip::read(&v, reg(JOY0DAT), Origin::cpu());
    let joy1 = CustomChip::read(&v, reg(JOY1DAT), Origin::cpu());
    assert_eq!((joy0, joy1), (0xfcfc, 0xfcfc));
}

#[test]
fn deniseid_on_an_original_denise_is_whatever_was_on_the_bus() {
    let custom = Custom::new(&Props::new()).unwrap();
    let v = Arc::new(video());
    custom.bus().attach(v.clone()).unwrap();
    *v.bus.lock() = Arc::downgrade(custom.bus());
    custom.bus().write(COLOR00, 0x0abc, Origin::cpu());
    assert_eq!(custom.bus().read(DENISEID, Origin::cpu()), 0x0abc);
}

#[test]
fn the_bus_routes_diwstrt_to_denise() {
    // regs.rs had DIWSTRT as Agnus-only; Appendix C gives it to both.
    let custom = Custom::new(&Props::new()).unwrap();
    let v = Arc::new(video());
    custom.bus().attach(v.clone()).unwrap();
    assert!(
        custom
            .bus()
            .write(DIWSTRT, PAL_DIWSTRT, Origin::copper(false))
    );
    assert_eq!(v.state.lock().regs.diwstrt, PAL_DIWSTRT);
}

#[test]
fn a_reset_clears_the_registers_and_the_picture() {
    let v = video();
    w(&v, COLOR00, 0x0fff);
    blank(&v, V);
    v.field(true);
    v.reset();
    assert_eq!(v.fields(), 0);
    blank(&v, V);
    assert_eq!(lores(&v, V, OUTPUT_LEFT), 0);
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

fn props() -> Props {
    Props::new().with("custom", Value::Link(Link::new("custom").unwrap()))
}

#[test]
fn the_properties_are_checked() {
    assert!(Denise::new(&props()).is_ok());
    assert!(Denise::new(&Props::new()).is_err(), "custom is required");
    let ntsc = Denise::new(&props().with("standard", Value::from("ntsc"))).unwrap();
    assert_eq!(ntsc.video().standard(), Standard::Ntsc);
    assert!(Denise::new(&props().with("standard", Value::from("secam"))).is_err());
    assert!(Denise::new(&props().with("agnus", Value::from(1u64))).is_err());
}

#[test]
fn the_video_handle_is_published() {
    let d = Denise::new(&props()).unwrap();
    let export = d.export(ExportId::AMIGA_VIDEO).expect("published");
    let handle = Arc::clone(export.opaque().unwrap())
        .downcast::<Video>()
        .expect("a Video");
    assert!(Arc::ptr_eq(&handle, d.video()));
    assert!(d.export(ExportId::CUSTOM_BUS).is_none());
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

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = Denise::new(&props()).unwrap();
    let v = saved.video();
    setup(v, 0x6a00);
    w(v, CLXCON, 0x1041);
    w(v, JOYTEST, 0x1234);
    sprite(v, 3, 0x99, 0x1234, 0x5678, 0);
    fetched(v, V, 0x38, [&[0xf0f0], &[0x0ff0], &[1], &[2], &[3], &[4]]);
    v.field(false);
    fetched(v, V, 0x38, [&[0xffff], &[], &[], &[], &[], &[]]);
    // And something queued against the beam.
    let beam = Arc::new(Fixed(Mutex::new(BeamPosition {
        vpos: V + 5,
        hpos: 9,
    })));
    v.connect_beam(beam);
    w(v, COLOR00, 0x0987);
    let bytes = snapshot(&saved);

    let restored = Denise::new(&props()).unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("denise", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(
        snapshot(&restored),
        bytes,
        "identical state after a round trip"
    );
    assert_eq!(frame_hash(restored.video()), frame_hash(saved.video()));

    // And it carries on identically: the queued change lands on the same pixel.
    for d in [&saved, &restored] {
        blank(d.video(), V + 5);
    }
    assert_eq!(snapshot(&restored), snapshot(&saved));
}

#[test]
fn a_snapshot_for_the_other_standard_is_refused() {
    let pal = Denise::new(&props()).unwrap();
    let bytes = snapshot(&pal);
    let ntsc = Denise::new(&props().with("standard", Value::from("ntsc"))).unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("denise", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&ntsc, &mut chunk.reader()).is_err());
}

// ---------------------------------------------------------------------------
// the golden
// ---------------------------------------------------------------------------

/// FNV-1a over every word of the picture, low byte first.
fn frame_hash(v: &Video) -> u64 {
    let (width, height) = v.geometry();
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut row = vec![0u16; width as usize];
    for y in 0..height {
        v.read_row(y, &mut row);
        for word in &row {
            for byte in word.to_le_bytes() {
                h ^= u64::from(byte);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

/// A deterministic word: not random, only uninteresting to a human.
fn pattern(a: u32, b: u32, c: u32) -> u16 {
    (a.wrapping_mul(0x9e37_79b9) ^ b.wrapping_mul(0x85eb_ca6b) ^ c.wrapping_mul(0xc2b2_ae35))
        .rotate_left(a % 13) as u16
}

/// One full PAL field of a synthetic scene, three bands deep:
///
/// * lines up to `$95`: a five-plane single playfield with sprites 0 and 1
///   attached, sprite 4 alone, `PF2P = 2`;
/// * `$96`–`$DB`: hold-and-modify, six planes, with a scroll delay of 5;
/// * `$DC` on: dual playfields with `PF2PRI`, playfield 1 at code 0 and
///   playfield 2 at code 3, each scrolled differently, collisions enabled.
///
/// The window is the standard one, so every band has border either side and
/// the top and bottom lines are border too.
fn paint_field(v: &Video) {
    w(v, DIWSTRT, PAL_DIWSTRT);
    w(v, DIWSTOP, PAL_DIWSTOP);
    for i in 0..32 {
        w(v, color(i), pattern(i.into(), 1, 2) & 0x0fff);
    }
    w(v, BPLCON0, 0x5200);
    w(v, BPLCON2, 0x0010);
    w(v, CLXCON, 0x3fc0);
    let mut words = [[0u16; 21]; 6];
    for vpos in 0..312u16 {
        match vpos {
            0x96 => {
                w(v, BPLCON0, 0x6a00);
                w(v, BPLCON1, 0x0055);
            }
            0xdc => {
                w(v, BPLCON0, 0x6600);
                w(v, BPLCON1, 0x0027);
                w(v, BPLCON2, 0x0058);
            }
            _ => {}
        }
        if vpos % 8 == 0 {
            // Sprite DMA, the way Agnus delivers it: through the register bus.
            let dma = Origin::dma();
            let x = 0x90 + vpos / 2;
            for (n, ctl) in [(0u16, 0u16), (1, ATTACH), (4, 0)] {
                let base = SPR0POS + 8 * n;
                let at = x + 3 * n;
                CustomChip::write(v, reg(base), at >> 1, dma);
                CustomChip::write(v, reg(base + 2), ctl | (at & 1), dma);
                CustomChip::write(v, reg(base + 6), pattern(vpos.into(), n.into(), 7), dma);
                CustomChip::write(v, reg(base + 4), pattern(vpos.into(), n.into(), 9), dma);
            }
        }
        for (p, plane) in words.iter_mut().enumerate() {
            for (i, word) in plane.iter_mut().enumerate() {
                *word = pattern(vpos.into(), p as u32, i as u32);
            }
        }
        fetched(
            v,
            vpos,
            0x30,
            [
                &words[0], &words[1], &words[2], &words[3], &words[4], &words[5],
            ],
        );
    }
    v.field(true);
}

/// **Golden.** What [`paint_field`] draws.
///
/// It covers the colour table, the standard window's clipping on all four
/// sides, a five-plane single playfield, hold-and-modify with a scroll delay,
/// dual playfields with `PF2PRI` and per-playfield delays, three sprites
/// delivered as DMA register writes with one attached pair, and both kinds of
/// sprite-versus-playfield priority. Every one of those has a pixel-level test
/// above with the manual's reasoning; this hash is what notices when a change
/// moves any of them. If it moves, that is a finding: say which band and why.
const GOLDEN_FIELD: u64 = 0xd28e_0878_ade8_1485;

#[test]
fn the_golden_field_renders_byte_identically() {
    let first = video();
    paint_field(&first);
    let second = video();
    paint_field(&second);
    let hash = frame_hash(&first);
    assert_eq!(hash, frame_hash(&second), "two runs, one picture");
    assert_eq!(hash, GOLDEN_FIELD, "the golden moved: {hash:#018x}");
    // The collisions it raised are part of what is fixed.
    assert_ne!(first.peek_clxdat(), 0);
    assert_eq!(first.peek_clxdat(), second.peek_clxdat());
}
