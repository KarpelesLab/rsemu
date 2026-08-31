//! Colour tables for display devices that emit indices rather than RGB.
//!
//! At the moment that means the NES: the 2C02 has no colours in it at all. It
//! emits a composite waveform, and what a television made of that waveform is
//! the *television's* business — which is why [`Pixel`](crate::dev::ppu::Pixel)
//! carries a 6-bit index and three emphasis bits and stops there.
//!
//! # There is no correct NES palette
//!
//! [NESdev's PPU palette page](https://www.nesdev.org/wiki/PPU_palettes) says
//! so plainly: every table in circulation is an *approximation*, they differ
//! visibly from one another, and real hardware differs from all of them
//! depending on the TV. So this file says which approximation it is and how it
//! was produced, rather than pretending to be the palette.
//!
//! # How [`NES_PALETTE`] was generated
//!
//! From the signal model, not from another emulator's table — a table lifted
//! from a GPL emulator would be exactly the expression `CLAUDE.md` forbids,
//! while the composite signal the chip emits is documented hardware behaviour.
//! The model is [NESdev's NTSC video
//! page](https://www.nesdev.org/wiki/NTSC_video):
//!
//! 1. Each pixel is a square wave of twelve phases. Its two voltage levels are
//!    picked by the colour's luma row (`$0x`-`$3x`), measured relative to the
//!    sync level: low `0.350, 0.518, 0.962, 1.550`, high
//!    `1.094, 1.506, 1.962, 1.962`.
//! 2. Hue is the wave's phase: hue `h` is high while `(h + phase) mod 12 < 6`.
//!    Hue `$x0` emits the high level throughout and hues `$xD`-`$xF` the low
//!    level, which is why they are greys and blacks; `$xE`/`$xF` additionally
//!    force luma row 1, so they are black in every row.
//! 3. Colour emphasis attenuates the signal to `0.746` of itself over the half
//!    of the wave centred on the emphasised hue — bit 0 around hue `$x0`, bit 1
//!    around `$x4`, bit 2 around `$x8`. Emphasis is therefore *in the signal*
//!    here rather than a per-channel multiply applied afterwards, which is why
//!    the table has all 512 entries.
//! 4. The twelve samples are demodulated the way NTSC specifies: `Y` is their
//!    mean, `I` and `Q` are the quadrature means against a reference whose `I`
//!    axis sits **123°** from the colour burst (`123° / 30° = 4.1` phases), and
//!    `RGB` comes from the FCC matrix
//!    (`R = Y + 0.956 I + 0.621 Q`, `G = Y − 0.272 I − 0.647 Q`,
//!    `B = Y − 1.106 I + 1.703 Q`), with `0.518` as black and `1.962` as white.
//!
//! Two numbers in that pipeline are choices rather than facts, and they are the
//! two every palette differs on:
//!
//! * **Chroma gain `1.5`.** The square wave's fundamental would be `2.0`, which
//!   drives the whole `$2x` row outside sRGB — `$24` and `$26` clip hard. `1.5`
//!   keeps every one of the 512 entries in range.
//! * **Gamma `1.2`**, applied to each channel after decoding. Composite video
//!   is encoded for a CRT, and without a little correction the greys come out
//!   noticeably brighter than the tables everyone recognises.
//!
//! Those two settle it: `$00` is `(85, 85, 85)`, `$0F` is black, `$30` is white,
//! `$11` is `(12, 81, 200)` and `$16` is `(168, 35, 30)` — within a few units of
//! the commonly quoted values, which is about as close as two approximations of
//! this get. `tests::the_table_is_what_the_signal_model_produces` regenerates
//! the whole thing in floating point and checks it, so the derivation above is
//! executable rather than a story about where the numbers came from.
//!
//! # Why a table and not a computation
//!
//! Frame hashes have to match across hosts (`ROADMAP.md` §11.6). `cos` is not
//! bit-identical between platforms, so a palette computed at runtime would make
//! a frame hash a property of the host's libm. The table is the fixed point;
//! the float model only ever runs in a test, which allows a tolerance of one
//! unit for exactly that reason.

/// The 2C02's colours as sRGB triples, indexed by
/// [`Pixel`](crate::dev::ppu::Pixel)`.0` — `emphasis << 6 | index`, all 512
/// combinations. See the module docs for the derivation.
// Four hues to a line and sixteen lines to an emphasis block, because that is
// the shape of the palette every NESdev page draws it in; one entry per line
// would be 512 lines nobody could check against anything.
#[rustfmt::skip]
pub static NES_PALETTE: [[u8; 3]; 512] = [
    // emphasis 000
    [ 85,  85,  85], [  0,  31, 111], [ 10,  13, 146], [ 40,   1, 146],
    [ 70,   0, 111], [ 89,   0,  53], [ 89,   3,   1], [ 70,  18,   0],
    [ 40,  37,   0], [ 10,  54,   0], [  0,  63,   0], [  0,  62,   1],
    [  0,  50,  53], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [162, 162, 162], [ 12,  81, 200], [ 47,  52, 252], [ 95,  30, 252],
    [140,  19, 200], [168,  21, 115], [168,  35,  30], [140,  60,   0],
    [ 95,  90,   0], [ 47, 116,   0], [ 12, 130,   0], [  0, 127,  30],
    [  0, 109, 115], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [255, 255, 255], [ 81, 166, 255], [126, 132, 255], [181, 105, 255],
    [232,  91, 255], [255,  93, 204], [255, 111, 105], [232, 141,  31],
    [181, 175,   0], [126, 204,   0], [ 81, 220,  31], [ 57, 217, 105],
    [ 57, 197, 204], [ 62,  62,  62], [  0,   0,   0], [  0,   0,   0],
    [255, 255, 255], [179, 217, 255], [200, 203, 255], [224, 190, 255],
    [245, 184, 255], [255, 185, 234], [255, 193, 191], [245, 207, 155],
    [224, 221, 135], [200, 234, 135], [179, 240, 155], [168, 239, 191],
    [168, 231, 234], [170, 170, 170], [  0,   0,   0], [  0,   0,   0],
    // emphasis 001
    [ 86,  51,  49], [  0,   8,  69], [ 11,   0, 101], [ 39,   0, 103],
    [ 68,   0,  77], [ 87,   0,  33], [ 89,   0,   0], [ 70,   6,   0],
    [ 42,  18,   0], [ 13,  29,   0], [  0,  34,   0], [  0,  31,   0],
    [  0,  19,  21], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [164, 109, 107], [ 13,  42, 136], [ 48,  23, 183], [ 93,   9, 187],
    [137,   4, 148], [165,   7,  82], [169,  21,  16], [141,  38,   0],
    [ 97,  59,   0], [ 51,  77,   0], [ 16,  84,   0], [  0,  79,   3],
    [  0,  61,  64], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [255, 181, 178], [ 84, 105, 210], [127,  81, 255], [180,  63, 255],
    [228,  56, 223], [255,  61, 150], [255,  79,  72], [233, 100,  10],
    [184, 125,   0], [131, 145,   0], [ 88, 153,   3], [ 63, 147,  55],
    [ 59, 127, 130], [ 63,  34,  33], [  0,   0,   0], [  0,   0,   0],
    [255, 181, 178], [183, 149, 191], [203, 138, 211], [226, 130, 213],
    [246, 127, 196], [255, 129, 166], [255, 137, 132], [248, 147, 101],
    [227, 157,  84], [204, 166,  82], [185, 169,  97], [173, 167, 124],
    [171, 158, 158], [173, 116, 113], [  0,   0,   0], [  0,   0,   0],
    // emphasis 010
    [ 43,  77,  30], [  0,  26,  61], [  0,   9,  86], [ 11,   0,  83],
    [ 31,   0,  53], [ 47,   0,  13], [ 50,   1,   0], [ 38,  14,   0],
    [ 17,  33,   0], [  0,  50,   0], [  0,  61,   0], [  0,  57,   0],
    [  0,  44,  19], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [ 97, 149,  77], [  0,  72, 123], [ 13,  44, 161], [ 46,  22, 157],
    [ 78,  10, 111], [104,  15,  49], [108,  29,   0], [ 89,  54,   0],
    [ 56,  83,   0], [ 22, 110,   0], [  0, 126,   0], [  0, 120,   1],
    [  0, 100,  59], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [164, 237, 135], [ 35, 152, 185], [ 66, 119, 226], [106,  92, 222],
    [143,  76, 173], [171,  82, 103], [175, 101,  32], [155, 131,   0],
    [118, 164,   0], [ 77, 194,   0], [ 45, 212,   0], [ 23, 205,  42],
    [ 20, 183, 115], [ 28,  55,  17], [  0,   0,   0], [  0,   0,   0],
    [164, 237, 135], [107, 201, 155], [122, 187, 172], [140, 175, 170],
    [155, 168, 150], [167, 170, 122], [169, 179,  90], [160, 192,  64],
    [145, 207,  50], [127, 219,  52], [112, 227,  68], [101, 224,  95],
    [ 99, 215, 127], [104, 157,  82], [  0,   0,   0], [  0,   0,   0],
    // emphasis 011
    [ 49,  48,  26], [  0,   7,  51], [  0,   0,  75], [ 13,   0,  75],
    [ 32,   0,  51], [ 49,   0,  11], [ 52,   0,   0], [ 39,   4,   0],
    [ 19,  16,   0], [  0,  27,   0], [  0,  34,   0], [  0,  30,   0],
    [  0,  19,  15], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [106, 105,  69], [  0,  41, 108], [ 19,  22, 144], [ 50,   8, 144],
    [ 81,   2, 108], [107,   5,  46], [111,  18,   0], [ 92,  35,   0],
    [ 60,  55,   0], [ 27,  73,   0], [  4,  83,   0], [  0,  78,   0],
    [  0,  60,  52], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [177, 175, 124], [ 45, 102, 167], [ 76,  79, 205], [114,  61, 205],
    [149,  51, 167], [177,  56,  98], [182,  75,  28], [160,  95,   0],
    [125, 119,   0], [ 86, 140,   0], [ 55, 151,   0], [ 32, 144,  33],
    [ 29, 124, 105], [ 32,  32,  13], [  0,   0,   0], [  0,   0,   0],
    [177, 175, 124], [119, 144, 142], [133, 134, 157], [150, 126, 157],
    [165, 121, 142], [177, 124, 113], [179, 132,  82], [170, 141,  59],
    [155, 152,  46], [138, 160,  46], [124, 165,  59], [113, 162,  85],
    [111, 154, 116], [113, 112,  74], [  0,   0,   0], [  0,   0,   0],
    // emphasis 100
    [ 54,  56, 107], [  0,  19, 120], [  4,   6, 154], [ 25,   0, 151],
    [ 47,   0, 117], [ 60,   0,  63], [ 57,   0,  11], [ 40,   1,   0],
    [ 14,  13,   0], [  0,  28,   0], [  0,  38,   0], [  0,  39,  13],
    [  0,  31,  66], [  0,   0,   0], [  0,   0,   5], [  0,   0,   5],
    [115, 117, 196], [  3,  60, 214], [ 34,  39, 255], [ 70,  18, 255],
    [104,   7, 210], [123,   6, 131], [119,  14,  50], [ 93,  30,   0],
    [ 53,  50,   0], [ 19,  74,   0], [  0,  89,   0], [  0,  91,  53],
    [  0,  80, 135], [  0,   0,   5], [  0,   0,   5], [  0,   0,   5],
    [189, 192, 255], [ 56, 128, 255], [ 97, 104, 255], [139,  78, 255],
    [177,  63, 255], [198,  61, 231], [194,  72, 138], [165,  92,  65],
    [119, 116,  27], [ 78, 144,  30], [ 46, 161,  69], [ 30, 163, 142],
    [ 33, 151, 236], [ 37,  38,  81], [  0,   0,   5], [  0,   0,   5],
    [189, 192, 255], [131, 165, 255], [150, 155, 255], [168, 143, 255],
    [184, 136, 255], [193, 135, 255], [191, 140, 233], [179, 149, 198],
    [159, 160, 178], [141, 172, 180], [126, 179, 200], [118, 180, 234],
    [120, 175, 255], [122, 124, 206], [  0,   0,   5], [  0,   0,   5],
    // emphasis 101
    [ 54,  38,  61], [  0,   5,  73], [  4,   0, 105], [ 24,   0, 105],
    [ 45,   0,  80], [ 57,   0,  39], [ 57,   0,   2], [ 40,   0,   0],
    [ 14,   8,   0], [  0,  19,   0], [  0,  25,   0], [  0,  24,   0],
    [  0,  16,  25], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [115,  90, 126], [  3,  36, 143], [ 34,  17, 190], [ 68,   4, 190],
    [100,   0, 153], [119,   0,  93], [119,   7,  31], [ 93,  22,   0],
    [ 53,  42,   0], [ 21,  59,   0], [  0,  68,   0], [  0,  67,  13],
    [  0,  55,  70], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [189, 153, 205], [ 56,  93, 223], [ 97,  70, 255], [137,  52, 255],
    [173,  42, 234], [194,  44, 168], [194,  56,  96], [165,  76,  29],
    [119,  99,   0], [ 81, 119,   0], [ 50, 130,  21], [ 33, 128,  73],
    [ 33, 114, 142], [ 37,  24,  43], [  0,   0,   0], [  0,   0,   0],
    [189, 153, 205], [131, 128, 212], [150, 118, 233], [167, 109, 233],
    [182, 105, 217], [191, 106, 189], [191, 111, 158], [179, 120, 126],
    [160, 131, 107], [143, 139, 107], [128, 144, 122], [120, 143, 148],
    [120, 137, 178], [122,  96, 133], [  0,   0,   0], [  0,   0,   0],
    // emphasis 110
    [ 34,  51,  51], [  0,  15,  69], [  0,   3,  94], [  8,   0,  91],
    [ 28,   0,  60], [ 40,   0,  22], [ 40,   0,   0], [ 28,   1,   0],
    [  9,  11,   0], [  0,  26,   0], [  0,  36,   0], [  0,  35,   0],
    [  0,  27,  30], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [ 83, 109, 110], [  0,  53, 137], [ 10,  33, 173], [ 42,  12, 169],
    [ 73,   2, 123], [ 92,   3,  65], [ 92,  12,   9], [ 73,  28,   0],
    [ 43,  47,   0], [ 11,  72,   0], [  0,  87,   0], [  0,  85,  18],
    [  0,  73,  78], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [144, 181, 182], [ 29, 118, 212], [ 57,  94, 251], [ 97,  68, 247],
    [133,  53, 196], [153,  55, 132], [153,  67,  64], [133,  88,  14],
    [ 99, 111,   0], [ 59, 139,   0], [ 29, 156,  24], [ 14, 154,  77],
    [ 14, 140, 146], [ 20,  34,  34], [  0,   0,   0], [  0,   0,   0],
    [144, 181, 182], [ 93, 154, 194], [107, 144, 210], [124, 132, 208],
    [139, 125, 188], [148, 126, 161], [148, 132, 131], [139, 141, 106],
    [125, 151,  92], [107, 163,  93], [ 93, 171, 112], [ 85, 170, 137],
    [ 85, 164, 167], [ 89, 116, 117], [  0,   0,   0], [  0,   0,   0],
    // emphasis 111
    [ 38,  38,  38], [  0,   5,  56], [  0,   0,  79], [ 10,   0,  79],
    [ 29,   0,  56], [ 41,   0,  18], [ 41,   0,   0], [ 29,   0,   0],
    [ 10,   8,   0], [  0,  19,   0], [  0,  25,   0], [  0,  24,   0],
    [  0,  16,  18], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [ 90,  90,  90], [  0,  36, 116], [ 14,  17, 151], [ 45,   4, 151],
    [ 76,   0, 116], [ 94,   0,  59], [ 94,   7,   4], [ 76,  22,   0],
    [ 45,  42,   0], [ 14,  59,   0], [  0,  68,   0], [  0,  67,   4],
    [  0,  55,  59], [  0,   0,   0], [  0,   0,   0], [  0,   0,   0],
    [153, 153, 153], [ 36,  92, 182], [ 66,  70, 221], [103,  52, 221],
    [138,  42, 182], [158,  44, 118], [158,  56,  52], [137,  76,   5],
    [103,  99,   0], [ 66, 119,   0], [ 36, 130,   5], [ 20, 128,  52],
    [ 20, 114, 118], [ 24,  24,  24], [  0,   0,   0], [  0,   0,   0],
    [153, 153, 153], [102, 128, 165], [116, 118, 181], [132, 109, 181],
    [147, 105, 165], [155, 106, 139], [155, 111, 110], [147, 120,  85],
    [132, 130,  72], [116, 139,  72], [102, 143,  85], [ 94, 143, 110],
    [ 94, 137, 139], [ 96,  96,  96], [  0,   0,   0], [  0,   0,   0],
];

/// How many entries [`NES_PALETTE`] has: 64 colours × 8 emphasis states.
pub const NES_PALETTE_LEN: usize = 512;

/// The sRGB triple for a raw [`Pixel`](crate::dev::ppu::Pixel) value.
///
/// The argument is the pixel's whole 9-bit value — `emphasis << 6 | index` —
/// and anything above that is masked off rather than refused, so a host cannot
/// index out of the table.
///
/// The greyscale bit of `$2001` is **not** applied here: the PPU already
/// applies it when it latches the pixel, because on hardware it is a property
/// of the chip's output and not of the television.
#[inline]
#[must_use]
pub fn nes_rgb(pixel: u16) -> [u8; 3] {
    NES_PALETTE[usize::from(pixel & 0x01ff)]
}

/// The sRGB triple for a palette index (0-63) and emphasis bits (`0bBGR`).
#[inline]
#[must_use]
pub fn nes_rgb_parts(index: u8, emphasis: u8) -> [u8; 3] {
    nes_rgb((u16::from(emphasis & 0x07) << 6) | u16::from(index & 0x3f))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The landmarks the module docs claim.
    #[test]
    fn the_named_colours_are_where_the_docs_say() {
        assert_eq!(nes_rgb(0x00), [85, 85, 85]);
        assert_eq!(nes_rgb(0x0f), [0, 0, 0]);
        assert_eq!(nes_rgb(0x11), [12, 81, 200]);
        assert_eq!(nes_rgb(0x16), [168, 35, 30]);
        assert_eq!(nes_rgb(0x30), [255, 255, 255]);
    }

    /// Hues `$xE` and `$xF` are black in every row and under every emphasis.
    ///
    /// They are the colours a game paints its borders with; if the table got
    /// them wrong the picture would have a coloured frame around it.
    ///
    /// Exactly black without emphasis. *With* emphasis the decode leaves a few
    /// units of tint, and that is the model being honest rather than a bug:
    /// these hues sit at the blanking level, emphasis attenuates the signal
    /// below it, and a chroma term then survives the per-channel clamp at zero.
    /// A television clamps sub-black and shows nothing; five parts in 255 is
    /// invisible, and special-casing the table to hide it would make the
    /// generator and the shipped numbers disagree.
    #[test]
    fn the_forced_blacks_are_black_everywhere() {
        for emphasis in 0..8u8 {
            for row in 0..4u8 {
                for hue in [0x0e, 0x0f] {
                    let index = row * 0x10 + hue;
                    let rgb = nes_rgb_parts(index, emphasis);
                    let bound = if emphasis == 0 { 0 } else { 8 };
                    assert!(
                        rgb.iter().all(|c| *c <= bound),
                        "index {index:#04x} emphasis {emphasis:03b} is {rgb:?}"
                    );
                }
            }
        }
    }

    /// Emphasis never brightens a channel it attenuates, and full emphasis
    /// darkens every one of them.
    #[test]
    fn emphasis_only_ever_takes_light_away() {
        for index in 0..64u8 {
            let plain = nes_rgb_parts(index, 0);
            let all = nes_rgb_parts(index, 0b111);
            for channel in 0..3 {
                assert!(
                    all[channel] <= plain[channel],
                    "index {index:#04x} channel {channel} went from {} to {}",
                    plain[channel],
                    all[channel]
                );
            }
        }
    }

    /// The whole table, regenerated from the floating-point signal model the
    /// module documents.
    ///
    /// A tolerance of one unit, because `cos`/`powf` are not bit-identical
    /// across platforms — which is the reason the shipped palette is a table in
    /// the first place.
    #[cfg(feature = "std")]
    #[test]
    fn the_table_is_what_the_signal_model_produces() {
        /// Voltage levels relative to sync: four low, then four high.
        const LEVELS: [f64; 8] = [0.350, 0.518, 0.962, 1.550, 1.094, 1.506, 1.962, 1.962];
        const ATTENUATION: f64 = 0.746;
        /// The I axis, 123° from the burst, in twelfths of a turn.
        const PHASE_OFFSET: f64 = 4.1;
        const CHROMA_GAIN: f64 = 1.5;
        const GAMMA: f64 = 1.2;

        fn in_phase(hue: i32, phase: i32) -> bool {
            (hue + phase).rem_euclid(12) < 6
        }

        fn signal(pixel: u16, phase: i32) -> f64 {
            let hue = i32::from(pixel & 0x0f);
            let emphasis = (pixel >> 6) & 0x07;
            let mut row = usize::from((pixel >> 4) & 0x03);
            if hue > 13 {
                row = 1; // $xE and $xF are black in every row
            }
            let mut low = LEVELS[row];
            let mut high = LEVELS[4 + row];
            if hue == 0 {
                low = high; // $x0 emits the high level throughout
            }
            if hue > 12 {
                high = low; // $xD-$xF emit the low level throughout
            }
            let mut level = if in_phase(hue, phase) { high } else { low };
            if (emphasis & 1 != 0 && in_phase(0, phase))
                || (emphasis & 2 != 0 && in_phase(4, phase))
                || (emphasis & 4 != 0 && in_phase(8, phase))
            {
                level *= ATTENUATION;
            }
            (level - LEVELS[1]) / (LEVELS[7] - LEVELS[1])
        }

        fn channel(v: f64) -> u8 {
            let v = if v <= 0.0 { 0.0 } else { v.powf(GAMMA) } * 255.0;
            if v < 0.0 {
                0
            } else if v > 255.0 {
                255
            } else {
                (v + 0.5) as u8
            }
        }

        for pixel in 0..NES_PALETTE_LEN {
            let pixel = pixel as u16;
            let (mut y, mut i, mut q) = (0.0f64, 0.0f64, 0.0f64);
            for phase in 0..12 {
                let s = signal(pixel, phase);
                let angle = core::f64::consts::PI * (f64::from(phase) + PHASE_OFFSET) / 6.0;
                y += s;
                i += s * angle.cos();
                q += s * angle.sin();
            }
            y /= 12.0;
            i = i * CHROMA_GAIN / 12.0;
            q = q * CHROMA_GAIN / 12.0;
            let want = [
                channel(y + 0.956 * i + 0.621 * q),
                channel(y - 0.272 * i - 0.647 * q),
                channel(y - 1.106 * i + 1.703 * q),
            ];
            let got = NES_PALETTE[usize::from(pixel)];
            for c in 0..3 {
                assert!(
                    got[c].abs_diff(want[c]) <= 1,
                    "pixel {pixel:#05x} channel {c}: table {got:?}, model {want:?}"
                );
            }
        }
    }
}
