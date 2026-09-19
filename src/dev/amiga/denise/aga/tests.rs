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

// ---------------------------------------------------------------------------
// the colour table
// ---------------------------------------------------------------------------

#[test]
fn lisaid_is_f8_and_lisa_drives_it() {
    // "Lisa returns hex (f8)" (§4, LISAID); the reserved byte reads as ones.
    let v = lisa();
    assert!(CustomChip::drives(&v, reg(DENISEID)));
    assert_eq!(CustomChip::read(&v, reg(DENISEID), Origin::cpu()), 0xfff8);
    assert_eq!(v.revision(), Revision::Aga);
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

// ---------------------------------------------------------------------------
// playfield modes
// ---------------------------------------------------------------------------

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
