//! The fixed-function back half of the SGX, as arithmetic: triangle
//! coverage, attribute interpolation, the depth test, blending and the
//! pixel formats a render target is written in.
//!
//! Nothing here is specific to how the guest programs the hardware — that is
//! the render pipeline's job — so everything here is plain graphics arithmetic,
//! testable on its own.
//!
//! # Determinism
//!
//! Coverage is decided in exact integer arithmetic on vertex positions
//! snapped to 1/16 pixel, so whether a pixel is covered never depends on
//! rounding, and two triangles sharing an edge cover each pixel along it
//! exactly once (the top-left rule). Interpolation uses `f32` addition,
//! multiplication and division only, which IEEE 754 defines exactly, so the
//! result is the same on every host.
//!
//! # Conventions
//!
//! Window coordinates: x right, y down, pixel `(x, y)` covers
//! `[x, x + 1) × [y, y + 1)` and is sampled at its centre. Colours are
//! `[r, g, b, a]` in `0.0..=1.0`.

/// Sub-pixel precision of snapped vertex positions, in bits.
const SUBPIXEL_BITS: u32 = 4;
const SUBPIXEL: f32 = (1 << SUBPIXEL_BITS) as f32;

/// The most attributes a vertex carries through interpolation.
pub const MAX_ATTRS: usize = 32;

/// A vertex in window space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    /// Window x, in pixels.
    pub x: f32,
    /// Window y, in pixels.
    pub y: f32,
    /// Depth, `0.0..=1.0`.
    pub z: f32,
    /// `1 / w` of the clip-space position: 1.0 for an affine primitive.
    pub inv_w: f32,
    /// The attributes to interpolate; only the first `count` are used.
    pub attrs: [f32; MAX_ATTRS],
}

impl Default for Vertex {
    fn default() -> Self {
        Vertex {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            inv_w: 1.0,
            attrs: [0.0; MAX_ATTRS],
        }
    }
}

/// An inclusive-exclusive pixel rectangle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    /// Left edge.
    pub x0: i32,
    /// Top edge.
    pub y0: i32,
    /// One past the right edge.
    pub x1: i32,
    /// One past the bottom edge.
    pub y1: i32,
}

impl Rect {
    /// The intersection, possibly empty.
    #[must_use]
    pub fn intersect(self, o: Rect) -> Rect {
        Rect {
            x0: self.x0.max(o.x0),
            y0: self.y0.max(o.y0),
            x1: self.x1.min(o.x1),
            y1: self.y1.min(o.y1),
        }
    }

    /// Whether no pixel is inside.
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.x0 >= self.x1 || self.y0 >= self.y1
    }
}

/// Which way round a triangle is, in window space (y down).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Winding {
    /// Clockwise on screen.
    Clockwise,
    /// Counter-clockwise on screen.
    CounterClockwise,
}

/// One covered pixel: where, its depth, and its interpolated attributes.
#[derive(Clone, Copy, Debug)]
pub struct Fragment<'a> {
    /// Pixel column.
    pub x: i32,
    /// Pixel row.
    pub y: i32,
    /// Interpolated depth.
    pub z: f32,
    /// Interpolated attributes, perspective-correct.
    pub attrs: &'a [f32],
}

/// Snaps a window coordinate to the sub-pixel grid. Out-of-range values
/// saturate, which keeps the edge arithmetic in range for any input.
fn snap(v: f32) -> i64 {
    let s = v * SUBPIXEL;
    if s.is_nan() {
        0
    } else if s >= 1.0e9 {
        1_000_000_000
    } else if s <= -1.0e9 {
        -1_000_000_000
    } else {
        // Round half away from zero without `f32::round` (std only).
        let t = s as i64;
        let frac = s - t as f32;
        if frac >= 0.5 {
            t + 1
        } else if frac <= -0.5 {
            t - 1
        } else {
            t
        }
    }
}

/// The edge function of `(a, b)` at `p`: twice the signed area of
/// `a, b, p`, positive when `p` is to the right of `a → b` on screen.
fn edge(a: (i64, i64), b: (i64, i64), p: (i64, i64)) -> i64 {
    (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0)
}

/// Whether the edge `a → b` of a clockwise triangle is a top or left edge,
/// which own the pixels exactly on them.
fn top_left(a: (i64, i64), b: (i64, i64)) -> bool {
    // Clockwise with y down: a top edge runs exactly left-to-right
    // (dy == 0, dx > 0); a left edge runs upwards (dy < 0).
    (a.1 == b.1 && b.0 > a.0) || b.1 < a.1
}

/// The winding of a triangle, or `None` when it has no area at sub-pixel
/// precision.
#[must_use]
pub fn winding(v: &[Vertex; 3]) -> Option<Winding> {
    let p = v.map(|v| (snap(v.x), snap(v.y)));
    match edge(p[0], p[1], p[2]) {
        0 => None,
        a if a > 0 => Some(Winding::Clockwise),
        _ => Some(Winding::CounterClockwise),
    }
}

/// Rasterises a triangle: calls `each` once for every pixel inside `clip`
/// whose centre the triangle covers, in row order. `count` attributes are
/// interpolated. Either winding is drawn; culling is the caller's choice.
pub fn triangle(v: &[Vertex; 3], count: usize, clip: Rect, mut each: impl FnMut(&Fragment<'_>)) {
    let count = count.min(MAX_ATTRS);
    let mut v = *v;
    let mut p = v.map(|v| (snap(v.x), snap(v.y)));
    let area = edge(p[0], p[1], p[2]);
    if area == 0 {
        return;
    }
    // Make it clockwise, so "inside" is "all edge functions positive".
    if area < 0 {
        v.swap(1, 2);
        p.swap(1, 2);
    }
    let area = edge(p[0], p[1], p[2]);
    // The bounding box, in whole pixels, clipped.
    let lo = |a: i64, b: i64, c: i64| a.min(b).min(c) >> SUBPIXEL_BITS;
    let hi = |a: i64, b: i64, c: i64| (a.max(b).max(c) >> SUBPIXEL_BITS) + 1;
    let bounds = Rect {
        x0: lo(p[0].0, p[1].0, p[2].0) as i32,
        y0: lo(p[0].1, p[1].1, p[2].1) as i32,
        x1: hi(p[0].0, p[1].0, p[2].0) as i32,
        y1: hi(p[0].1, p[1].1, p[2].1) as i32,
    }
    .intersect(clip);
    if bounds.is_empty() {
        return;
    }
    // Edge i is the one opposite vertex i; its bias excludes pixels exactly
    // on a non-top-left edge.
    let edges = [(p[1], p[2]), (p[2], p[0]), (p[0], p[1])];
    let bias = edges.map(|(a, b)| if top_left(a, b) { 0 } else { -1 });
    // Perspective: interpolate attr/w and 1/w, divide per pixel.
    let mut pa = [[0f32; MAX_ATTRS]; 3];
    for (out, vert) in pa.iter_mut().zip(&v) {
        for (o, a) in out.iter_mut().zip(&vert.attrs).take(count) {
            *o = a * vert.inv_w;
        }
    }
    let area_f = area as f32;
    let half = 1i64 << (SUBPIXEL_BITS - 1);
    let mut attrs = [0f32; MAX_ATTRS];
    for y in bounds.y0..bounds.y1 {
        let py = (i64::from(y) << SUBPIXEL_BITS) + half;
        for x in bounds.x0..bounds.x1 {
            let px = (i64::from(x) << SUBPIXEL_BITS) + half;
            let w = [
                edge(edges[0].0, edges[0].1, (px, py)),
                edge(edges[1].0, edges[1].1, (px, py)),
                edge(edges[2].0, edges[2].1, (px, py)),
            ];
            if w[0] + bias[0] < 0 || w[1] + bias[1] < 0 || w[2] + bias[2] < 0 {
                continue;
            }
            let b = w.map(|w| w as f32 / area_f);
            let z = b[0] * v[0].z + b[1] * v[1].z + b[2] * v[2].z;
            let iw = b[0] * v[0].inv_w + b[1] * v[1].inv_w + b[2] * v[2].inv_w;
            let rw = if iw != 0.0 { 1.0 / iw } else { 0.0 };
            for (i, out) in attrs.iter_mut().enumerate().take(count) {
                *out = (b[0] * pa[0][i] + b[1] * pa[1][i] + b[2] * pa[2][i]) * rw;
            }
            each(&Fragment {
                x,
                y,
                z,
                attrs: &attrs[..count],
            });
        }
    }
}

/// A depth (or stencil) comparison, as the eight the API offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Compare {
    /// Never passes.
    Never,
    /// `new < old`.
    Less,
    /// `new == old`.
    Equal,
    /// `new <= old`.
    LessEqual,
    /// `new > old`.
    Greater,
    /// `new != old`.
    NotEqual,
    /// `new >= old`.
    GreaterEqual,
    /// Always passes.
    Always,
}

impl Compare {
    /// Whether `new` passes against `old`.
    #[must_use]
    pub fn test(self, new: f32, old: f32) -> bool {
        match self {
            Compare::Never => false,
            Compare::Less => new < old,
            Compare::Equal => new == old,
            Compare::LessEqual => new <= old,
            Compare::Greater => new > old,
            Compare::NotEqual => new != old,
            Compare::GreaterEqual => new >= old,
            Compare::Always => true,
        }
    }
}

/// A blend factor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Factor {
    /// 0.
    Zero,
    /// 1.
    One,
    /// The source colour.
    SrcColor,
    /// 1 − the source colour.
    OneMinusSrcColor,
    /// The source alpha.
    SrcAlpha,
    /// 1 − the source alpha.
    OneMinusSrcAlpha,
    /// The destination colour.
    DstColor,
    /// 1 − the destination colour.
    OneMinusDstColor,
    /// The destination alpha.
    DstAlpha,
    /// 1 − the destination alpha.
    OneMinusDstAlpha,
    /// `min(As, 1 − Ad)` for colour, 1 for alpha.
    SrcAlphaSaturate,
}

impl Factor {
    fn weight(self, src: [f32; 4], dst: [f32; 4], channel: usize) -> f32 {
        let c = channel;
        match self {
            Factor::Zero => 0.0,
            Factor::One => 1.0,
            Factor::SrcColor => src[c],
            Factor::OneMinusSrcColor => 1.0 - src[c],
            Factor::SrcAlpha => src[3],
            Factor::OneMinusSrcAlpha => 1.0 - src[3],
            Factor::DstColor => dst[c],
            Factor::OneMinusDstColor => 1.0 - dst[c],
            Factor::DstAlpha => dst[3],
            Factor::OneMinusDstAlpha => 1.0 - dst[3],
            Factor::SrcAlphaSaturate => {
                if c == 3 {
                    1.0
                } else {
                    src[3].min(1.0 - dst[3])
                }
            }
        }
    }
}

/// Additive blending with separate colour and alpha factors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Blend {
    /// Source factor for r, g, b.
    pub src_rgb: Factor,
    /// Destination factor for r, g, b.
    pub dst_rgb: Factor,
    /// Source factor for a.
    pub src_a: Factor,
    /// Destination factor for a.
    pub dst_a: Factor,
}

impl Blend {
    /// Replace: the source wins.
    pub const REPLACE: Blend = Blend {
        src_rgb: Factor::One,
        dst_rgb: Factor::Zero,
        src_a: Factor::One,
        dst_a: Factor::Zero,
    };

    /// `src · Fs + dst · Fd`, per channel, clamped to `0.0..=1.0`.
    #[must_use]
    pub fn apply(self, src: [f32; 4], dst: [f32; 4]) -> [f32; 4] {
        let mut out = [0f32; 4];
        for (c, o) in out.iter_mut().enumerate() {
            let (fs, fd) = if c == 3 {
                (self.src_a, self.dst_a)
            } else {
                (self.src_rgb, self.dst_rgb)
            };
            let v = src[c] * fs.weight(src, dst, c) + dst[c] * fd.weight(src, dst, c);
            *o = v.clamp(0.0, 1.0);
        }
        out
    }
}

/// A render-target or texture pixel format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// 32 bits, `A[31:24] R[23:16] G[15:8] B[7:0]`.
    Argb8888,
    /// 32 bits, `A[31:24] B[23:16] G[15:8] R[7:0]`.
    Abgr8888,
    /// 16 bits, `R[15:11] G[10:5] B[4:0]`.
    Rgb565,
    /// 16 bits, `A[15:12] R[11:8] G[7:4] B[3:0]`.
    Argb4444,
    /// 16 bits, `A[15] R[14:10] G[9:5] B[4:0]`.
    Argb1555,
}

impl Format {
    /// Bytes per pixel.
    #[must_use]
    pub fn bytes(self) -> u32 {
        match self {
            Format::Argb8888 | Format::Abgr8888 => 4,
            Format::Rgb565 | Format::Argb4444 | Format::Argb1555 => 2,
        }
    }

    /// Packs a colour, rounding to nearest.
    #[must_use]
    pub fn pack(self, c: [f32; 4]) -> u32 {
        let q = |v: f32, max: u32| -> u32 {
            let v = v.clamp(0.0, 1.0) * max as f32 + 0.5;
            (v as u32).min(max)
        };
        let [r, g, b, a] = c;
        match self {
            Format::Argb8888 => q(a, 255) << 24 | q(r, 255) << 16 | q(g, 255) << 8 | q(b, 255),
            Format::Abgr8888 => q(a, 255) << 24 | q(b, 255) << 16 | q(g, 255) << 8 | q(r, 255),
            Format::Rgb565 => q(r, 31) << 11 | q(g, 63) << 5 | q(b, 31),
            Format::Argb4444 => q(a, 15) << 12 | q(r, 15) << 8 | q(g, 15) << 4 | q(b, 15),
            Format::Argb1555 => q(a, 1) << 15 | q(r, 31) << 10 | q(g, 31) << 5 | q(b, 31),
        }
    }

    /// Unpacks a pixel. Formats without alpha read as opaque.
    #[must_use]
    pub fn unpack(self, v: u32) -> [f32; 4] {
        let f = |v: u32, max: u32| (v & max) as f32 / max as f32;
        match self {
            Format::Argb8888 => [f(v >> 16, 255), f(v >> 8, 255), f(v, 255), f(v >> 24, 255)],
            Format::Abgr8888 => [f(v, 255), f(v >> 8, 255), f(v >> 16, 255), f(v >> 24, 255)],
            Format::Rgb565 => [f(v >> 11, 31), f(v >> 5, 63), f(v, 31), 1.0],
            Format::Argb4444 => [f(v >> 8, 15), f(v >> 4, 15), f(v, 15), f(v >> 12, 15)],
            Format::Argb1555 => [f(v >> 10, 31), f(v >> 5, 31), f(v, 31), f(v >> 15, 1)],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    fn vert(x: f32, y: f32) -> Vertex {
        Vertex {
            x,
            y,
            ..Vertex::default()
        }
    }

    const SCREEN: Rect = Rect {
        x0: 0,
        y0: 0,
        x1: 16,
        y1: 16,
    };

    fn coverage(tris: &[[Vertex; 3]]) -> Vec<u32> {
        let mut hits = vec![0u32; 16 * 16];
        for t in tris {
            triangle(t, 0, SCREEN, |f| hits[(f.y * 16 + f.x) as usize] += 1);
        }
        hits
    }

    #[test]
    fn a_quad_split_on_its_diagonal_covers_every_pixel_once() {
        let (a, b, c, d) = (
            vert(0.0, 0.0),
            vert(16.0, 0.0),
            vert(16.0, 16.0),
            vert(0.0, 16.0),
        );
        // Opposite windings, sharing the diagonal: still exactly once each.
        let hits = coverage(&[[a, b, c], [a, d, c]]);
        assert!(hits.iter().all(|&h| h == 1), "{hits:?}");
    }

    #[test]
    fn a_fan_of_odd_triangles_leaves_no_gap_and_no_overlap() {
        let centre = vert(7.3, 8.9);
        let ring = [
            vert(0.0, 0.0),
            vert(16.0, 0.0),
            vert(16.0, 16.0),
            vert(0.0, 16.0),
        ];
        let tris: Vec<[Vertex; 3]> = (0..4)
            .map(|i| [centre, ring[i], ring[(i + 1) % 4]])
            .collect();
        let hits = coverage(&tris);
        assert!(hits.iter().all(|&h| h == 1), "{hits:?}");
    }

    #[test]
    fn pixels_are_sampled_at_their_centres() {
        // The hypotenuse x + y = 4 passes exactly through pixel centres; it
        // is a bottom-right edge, so those centres are not covered.
        let hits = coverage(&[[vert(0.0, 0.0), vert(4.0, 0.0), vert(0.0, 4.0)]]);
        let row = |y: usize| hits[y * 16..y * 16 + 5].to_vec();
        assert_eq!(row(0), vec![1, 1, 1, 0, 0]);
        assert_eq!(row(1), vec![1, 1, 0, 0, 0]);
        assert_eq!(row(3), vec![0, 0, 0, 0, 0]);
        // A left edge through column 0's centres does cover them.
        let hits = coverage(&[[vert(0.5, 0.0), vert(4.0, 0.0), vert(0.5, 4.0)]]);
        assert_eq!((0..4).map(|y| hits[y * 16]).sum::<u32>(), 4);
        assert_eq!(hits[4 * 16], 0, "below the bottom vertex");
    }

    #[test]
    fn the_clip_rectangle_bounds_coverage() {
        let clip = Rect {
            x0: 2,
            y0: 3,
            x1: 5,
            y1: 4,
        };
        let mut n = 0;
        triangle(
            &[vert(0.0, 0.0), vert(16.0, 0.0), vert(0.0, 16.0)],
            0,
            clip,
            |f| {
                assert!((2..5).contains(&f.x) && f.y == 3);
                n += 1;
            },
        );
        assert_eq!(n, 3);
    }

    #[test]
    fn attributes_interpolate_linearly_without_perspective() {
        let mut v = [vert(0.0, 0.0), vert(16.0, 0.0), vert(0.0, 16.0)];
        v[1].attrs[0] = 16.0;
        v[2].attrs[1] = 16.0;
        triangle(&v, 2, SCREEN, |f| {
            assert_eq!(f.attrs[0], f.x as f32 + 0.5);
            assert_eq!(f.attrs[1], f.y as f32 + 0.5);
        });
    }

    #[test]
    fn perspective_divides_by_the_interpolated_w() {
        // Same attribute at every vertex: perspective must not distort it.
        let mut v = [vert(0.0, 0.0), vert(16.0, 0.0), vert(0.0, 16.0)];
        for (k, w) in [1.0f32, 4.0, 0.5].into_iter().enumerate() {
            v[k].inv_w = 1.0 / w;
            v[k].attrs[0] = 0.25;
        }
        triangle(&v, 1, SCREEN, |f| {
            assert!((f.attrs[0] - 0.25).abs() < 1e-6, "{}", f.attrs[0]);
        });
    }

    #[test]
    fn degenerate_triangles_draw_nothing() {
        let hits = coverage(&[[vert(0.0, 0.0), vert(8.0, 8.0), vert(16.0, 16.0)]]);
        assert!(hits.iter().all(|&h| h == 0));
        assert_eq!(
            winding(&[vert(0.0, 0.0), vert(8.0, 8.0), vert(16.0, 16.0)]),
            None
        );
    }

    #[test]
    fn winding_is_reported_on_screen() {
        let cw = [vert(0.0, 0.0), vert(4.0, 0.0), vert(0.0, 4.0)];
        assert_eq!(winding(&cw), Some(Winding::Clockwise));
        let ccw = [cw[0], cw[2], cw[1]];
        assert_eq!(winding(&ccw), Some(Winding::CounterClockwise));
    }

    #[test]
    fn source_over_blending() {
        let over = Blend {
            src_rgb: Factor::SrcAlpha,
            dst_rgb: Factor::OneMinusSrcAlpha,
            src_a: Factor::One,
            dst_a: Factor::OneMinusSrcAlpha,
        };
        let out = over.apply([1.0, 0.0, 0.0, 0.25], [0.0, 0.0, 1.0, 1.0]);
        assert_eq!(out, [0.25, 0.0, 0.75, 1.0]);
        assert_eq!(Blend::REPLACE.apply([0.5; 4], [1.0; 4]), [0.5; 4]);
    }

    #[test]
    fn formats_round_trip_their_own_values() {
        for f in [
            Format::Argb8888,
            Format::Abgr8888,
            Format::Rgb565,
            Format::Argb4444,
            Format::Argb1555,
        ] {
            let bits = f.bytes() * 8;
            for v in [0u32, 0x1234_5678, 0xffff_ffff, 0x8421_7bde] {
                let v = if bits == 32 { v } else { v & 0xffff };
                // A format without alpha drops nothing else on the way.
                let back = f.pack(f.unpack(v));
                assert_eq!(back, v, "{f:?} {v:#x}");
            }
        }
        assert_eq!(Format::Rgb565.pack([1.0, 0.0, 0.0, 1.0]), 0xf800);
        assert_eq!(Format::Argb8888.pack([0.0, 0.0, 1.0, 1.0]), 0xff00_00ff);
    }
}
