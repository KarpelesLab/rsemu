//! The vector floating-point forms' tables and number formats (§8).
//!
//! Every table here was read off the guest's `useasm` encoder run as a black
//! box: each input swizzle or format was fed in and the bits it produced
//! recorded. Component codes: `0`–`3` are `x`–`w`; `4` and `5` are the two
//! extra codes the encoder accepts, taken as the constants 0.0 and 1.0
//! \[G\].

/// `xyzw` as component codes.
pub(super) const XYZW: [u8; 4] = [0, 1, 2, 3];

const X: u8 = 0;
const Y: u8 = 1;
const Z: u8 = 2;
const W: u8 = 3;

/// The 16-entry swizzle table `vop` `src2`, `vmov` and the op-3 sources
/// index (§8.1). Entry 15 is `xyz` with code 5 \[G: 1.0\] in `w`.
pub(super) const SW16: [[u8; 4]; 16] = [
    [X, X, X, X],
    [Y, Y, Y, Y],
    [Z, Z, Z, Z],
    [W, W, W, W],
    [X, Y, Z, W],
    [Y, Z, W, W],
    [X, Y, Z, Z],
    [X, X, Y, Z],
    [X, Y, X, Y],
    [X, Y, W, Z],
    [Z, X, Y, W],
    [Z, W, Z, W],
    [Y, Z, X, Z],
    [X, X, Y, Y],
    [X, Z, W, W],
    [X, Y, Z, 5],
];

/// The op-3 sources' 32-entry table: [`SW16`], then these 16 (§8.3).
pub(super) const SW32: [[u8; 4]; 32] = {
    const EXT: [[u8; 4]; 16] = [
        [Y, Z, X, W],
        [Z, W, X, Y],
        [X, Z, W, Y],
        [Y, Y, W, W],
        [W, Y, Z, W],
        [W, Z, W, Z],
        [X, Y, Z, X],
        [Z, Z, W, W],
        [X, W, Z, X],
        [Y, Y, Y, X],
        [Y, Y, Y, Z],
        [X, Z, Y, W],
        [X, X, X, Y],
        [Z, Y, X, W],
        [Y, Y, Z, Z],
        [Z, Z, Z, Y],
    ];
    let mut t = [[0; 4]; 32];
    let mut i = 0;
    while i < 16 {
        t[i] = SW16[i];
        t[i + 16] = EXT[i];
        i += 1;
    }
    t
};

/// `vmad` `src0`'s eight swizzles, index `{b53, b19:18}` (§8.2).
pub(super) const SW_MAD0: [[u8; 4]; 8] = [
    [X, X, X, X],
    [Y, Y, Y, Y],
    [Z, Z, Z, Z],
    [W, W, W, W],
    [X, Y, Z, W],
    [Y, Z, X, W],
    [X, Y, W, W],
    [Z, W, X, Y],
];

/// `vmad` `src1`'s, index `{b44, b21:20}`.
pub(super) const SW_MAD1: [[u8; 4]; 8] = [
    [X, X, X, X],
    [Y, Y, Y, Y],
    [Z, Z, Z, Z],
    [W, W, W, W],
    [X, Y, Z, W],
    [X, Y, Y, Z],
    [Y, Y, W, W],
    [W, Y, Z, W],
];

/// `vmad` `src2`'s, index `b47:45`.
pub(super) const SW_MAD2: [[u8; 4]; 8] = [
    [X, X, X, X],
    [Y, Y, Y, Y],
    [Z, Z, Z, Z],
    [W, W, W, W],
    [X, Y, Z, W],
    [X, Z, W, W],
    [X, X, Y, Z],
    [X, Y, Z, Z],
];

/// A `pck` lane format, `b43:41` (destination) and `b40:38` (source)
/// (§8.6). The structure is exact — which pairs the encoder converts —
/// the names \[G\].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Fmt {
    U8,
    S8,
    O8,
    U16,
    S16,
    F16,
    F32,
    C10,
}

impl Fmt {
    pub(super) fn of(code: u64) -> Fmt {
        [
            Fmt::U8,
            Fmt::S8,
            Fmt::O8,
            Fmt::U16,
            Fmt::S16,
            Fmt::F16,
            Fmt::F32,
            Fmt::C10,
        ][(code & 7) as usize]
    }

    pub(super) fn name(self) -> &'static str {
        match self {
            Fmt::U8 => "u8",
            Fmt::S8 => "s8",
            Fmt::O8 => "o8",
            Fmt::U16 => "u16",
            Fmt::S16 => "s16",
            Fmt::F16 => "f16",
            Fmt::F32 => "f32",
            Fmt::C10 => "c10",
        }
    }

    /// Lane `lane`'s place: the 32-bit word it sits in, counted from the
    /// operand's register, its bit offset and width. `None` for the
    /// formats whose packing is not known.
    pub(super) fn lane(self, lane: u32) -> Option<(u32, u32, u32)> {
        Some(match self {
            Fmt::U8 | Fmt::S8 => (0, 8 * lane, 8),
            Fmt::U16 | Fmt::S16 | Fmt::F16 => (lane / 2, 16 * (lane % 2), 16),
            Fmt::F32 => (lane, 0, 32),
            // [G] Four 10-bit lanes over a register pair: three in the
            // first word from the bottom, the fourth at the second's.
            Fmt::C10 if lane < 3 => (0, 10 * lane, 10),
            Fmt::C10 if lane == 3 => (1, 0, 10),
            Fmt::O8 | Fmt::C10 => return None,
        })
    }

    /// A lane's bits as a float. \[G\] `norm` scales the integer formats
    /// to 0..1 (signed: -1..1).
    pub(super) fn decode(self, v: u32, norm: bool) -> f32 {
        match self {
            Fmt::U8 | Fmt::U16 => {
                let max = if self == Fmt::U8 { 255.0 } else { 65535.0 };
                if norm { v as f32 / max } else { v as f32 }
            }
            Fmt::S8 => {
                let x = f32::from(v as u8 as i8);
                if norm { (x / 127.0).max(-1.0) } else { x }
            }
            Fmt::S16 => {
                let x = f32::from(v as u16 as i16);
                if norm { (x / 32767.0).max(-1.0) } else { x }
            }
            Fmt::F16 => f16_to_f32(v as u16),
            Fmt::F32 => f32::from_bits(v),
            // [G] Unsigned, 1023 being 1.0.
            Fmt::C10 => (v & 0x3ff) as f32 / 1023.0,
            Fmt::O8 => 0.0,
        }
    }

    /// A float as a lane's bits: integers rounded to nearest (halves away
    /// from zero \[G\]) and saturated.
    pub(super) fn encode(self, x: f32, norm: bool) -> u32 {
        let int = |x: f32, lo: f32, hi: f32| -> u32 {
            let x = if x.is_nan() { 0.0 } else { x.clamp(lo, hi) };
            (if x < 0.0 { x - 0.5 } else { x + 0.5 }) as i32 as u32
        };
        match self {
            Fmt::U8 if norm => int(x * 255.0, 0.0, 255.0),
            Fmt::U8 => int(x, 0.0, 255.0),
            Fmt::S8 if norm => int(x * 127.0, -127.0, 127.0) & 0xff,
            Fmt::S8 => int(x, -128.0, 127.0) & 0xff,
            Fmt::U16 if norm => int(x * 65535.0, 0.0, 65535.0),
            Fmt::U16 => int(x, 0.0, 65535.0),
            Fmt::S16 if norm => int(x * 32767.0, -32767.0, 32767.0) & 0xffff,
            Fmt::S16 => int(x, -32768.0, 32767.0) & 0xffff,
            Fmt::F16 => u32::from(f32_to_f16(x)),
            Fmt::F32 => x.to_bits(),
            Fmt::C10 => int(x * 1023.0, 0.0, 1023.0),
            Fmt::O8 => 0,
        }
    }
}

/// IEEE binary16 to binary32, exact.
pub(super) fn f16_to_f32(h: u16) -> f32 {
    let sign = u32::from(h >> 15) << 31;
    let exp = u32::from(h >> 10 & 0x1f);
    let man = u32::from(h & 0x3ff);
    let bits = match (exp, man) {
        (0, 0) => sign,
        (0, m) => {
            // Subnormal: normalise.
            let shift = m.leading_zeros() - 21;
            let m = (m << shift) & 0x3ff;
            sign | (113 - shift) << 23 | m << 13
        }
        (0x1f, m) => sign | 0xff << 23 | m << 13,
        (e, m) => sign | (e + 112) << 23 | m << 13,
    };
    f32::from_bits(bits)
}

/// IEEE binary32 to binary16, rounding to nearest even.
pub(super) fn f32_to_f16(x: f32) -> u16 {
    let b = x.to_bits();
    let sign = (b >> 16 & 0x8000) as u16;
    let exp = (b >> 23 & 0xff) as i32;
    let man = b & 0x7f_ffff;
    if exp == 0xff {
        return sign | 0x7c00 | if man != 0 { 0x200 } else { 0 };
    }
    let e = exp - 127 + 15;
    if e >= 0x1f {
        return sign | 0x7c00;
    }
    let (m, shift) = if e <= 0 {
        if e < -10 {
            return sign;
        }
        (man | 0x80_0000, (14 - e) as u32)
    } else {
        (man, 13)
    };
    let half = 1u32 << (shift - 1);
    let rest = m & ((1 << shift) - 1);
    let mut r = m >> shift;
    if rest > half || (rest == half && r & 1 != 0) {
        r += 1;
    }
    let base = if e <= 0 { 0 } else { (e as u32) << 10 };
    // A mantissa carry rolls into the exponent, as it should.
    sign | (base + r) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn half_floats_round_trip() {
        for h in [0u16, 0x3c00, 0xbc00, 0x7bff, 0x0001, 0x03ff, 0x7c00, 0x3555] {
            assert_eq!(f32_to_f16(f16_to_f32(h)), h, "{h:#x}");
        }
        assert_eq!(f32_to_f16(1.0), 0x3c00);
        assert_eq!(f32_to_f16(65520.0), 0x7c00);
        assert_eq!(f16_to_f32(0x3555), 0.333_251_95);
    }

    #[test]
    fn unorm8_packs_and_unpacks() {
        assert_eq!(Fmt::U8.encode(1.0, true), 255);
        assert_eq!(Fmt::U8.encode(0.5, true), 128);
        assert_eq!(Fmt::U8.encode(-3.0, true), 0);
        assert_eq!(Fmt::U8.decode(255, true), 1.0);
        assert_eq!(Fmt::S16.encode(-2.6, false), 0xfffd);
    }

    #[test]
    fn the_extended_table_follows_the_first() {
        assert_eq!(SW32[4], XYZW);
        assert_eq!(SW32[17], [Z, W, X, Y]);
    }
}
