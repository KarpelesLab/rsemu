//! Turning a [`Surface`] into a FramebufferUpdate (RFC 6143 §7.6.1, §7.7.1).
//!
//! One of these per connection, because what the client has already been sent
//! is per-connection state: two viewers of the same machine are at different
//! points in its history, and the second one to attach needs a whole frame
//! while the first needs three changed rows.
//!
//! # Damage, in bands
//!
//! An incremental update sends the rows that changed, coalesced into runs — a
//! full-width rectangle per run. Not the tightest possible damage (a
//! single-character change in a text mode still sends a 720-pixel row) and
//! deliberately so: a per-pixel bounding box costs a column scan of the whole
//! frame to save bytes on a loopback socket, and the row scan is a `memcmp`
//! the CPU does at cache speed. When a compressed encoding lands, the tighter
//! rectangle is worth computing; with Raw it is not.
//!
//! # Resizing
//!
//! A guest changes video mode whenever it likes and the framebuffer changes
//! shape underneath the connection. A client that asked for the DesktopSize
//! pseudo-encoding (§7.8.2) is told, with a zero-data rectangle carrying the
//! new geometry, and then given a full frame. A client that did not ask is
//! **not** resized — the RFC gives no other way to say it — so it keeps seeing
//! the geometry it was promised in ServerInit, with the new picture cropped
//! into it and any uncovered area black. Wrong-looking, but honest and stable;
//! silently sending rectangles outside the framebuffer the client allocated is
//! how a viewer segfaults.

use alloc::vec::Vec;

use crate::host::display::{PixelFormat as SurfaceFormat, Surface};

use super::proto::{PixelFormat, encoding, rect_header, update_header};

/// The per-connection encoder.
#[derive(Debug)]
pub struct FrameEncoder {
    /// The pixel format the client last asked for.
    format: PixelFormat,
    /// The geometry the client believes the framebuffer has.
    announced: (u16, u16),
    /// Whether the client asked for the DesktopSize pseudo-encoding.
    resizable: bool,
    /// A copy of the last surface this client was sent, in *surface* layout.
    /// Empty when the client has been sent nothing it can be compared against.
    last: Vec<u8>,
    /// The shape `last` is in.
    last_shape: (SurfaceFormat, u32, u32),
}

impl FrameEncoder {
    /// An encoder for a client that has been told the framebuffer is
    /// `width × height` and has been sent nothing yet.
    #[must_use]
    pub fn new(format: PixelFormat, width: u16, height: u16) -> FrameEncoder {
        FrameEncoder {
            format,
            announced: (width, height),
            resizable: false,
            last: Vec::new(),
            last_shape: (SurfaceFormat::RGBA8888, 0, 0),
        }
    }

    /// The format updates are produced in.
    #[must_use]
    pub const fn format(&self) -> PixelFormat {
        self.format
    }

    /// Change the pixel format (§7.5.1).
    ///
    /// Everything the client has is now in the wrong format, so the next update
    /// is a whole frame whatever it asks for.
    pub fn set_format(&mut self, format: PixelFormat) {
        self.format = format;
        self.invalidate();
    }

    /// Note which encodings the client understands (§7.5.2).
    pub fn set_encodings(&mut self, list: &[i32]) {
        self.resizable = list.contains(&encoding::DESKTOP_SIZE);
    }

    /// Whether this client can be told the framebuffer changed size.
    #[must_use]
    pub const fn resizable(&self) -> bool {
        self.resizable
    }

    /// The geometry this client believes the framebuffer has.
    #[must_use]
    pub const fn announced(&self) -> (u16, u16) {
        self.announced
    }

    /// Forget what the client has, so the next update is a whole frame.
    pub fn invalidate(&mut self) {
        self.last.clear();
        self.last_shape = (SurfaceFormat::RGBA8888, 0, 0);
    }

    /// An update for `surface`, or `None` when an incremental request has
    /// nothing to say.
    ///
    /// A non-incremental request always produces a frame, even an identical
    /// one: the client asked, and RFC 6143 §7.5.3 promises it an answer.
    pub fn update(&mut self, surface: &Surface, incremental: bool) -> Option<Vec<u8>> {
        let shape = (surface.format(), surface.width(), surface.height());
        let comparable = incremental && !self.last.is_empty() && self.last_shape == shape;

        // The guest resized. Tell a client that can hear it; otherwise keep
        // presenting the geometry it was promised.
        let resized = self.resizable
            && (fits(surface.width()), fits(surface.height())) != self.announced
            && surface.width() > 0
            && surface.height() > 0;

        let mut rects: Vec<Vec<u8>> = Vec::new();
        if resized {
            let (w, h) = (fits(surface.width()), fits(surface.height()));
            rects.push(rect_header(0, 0, w, h, encoding::DESKTOP_SIZE).to_vec());
            self.announced = (w, h);
        }

        let (aw, ah) = self.announced;
        if comparable && !resized {
            for (top, bottom) in self.damaged_bands(surface) {
                let height = bottom - top;
                let top16 = fits(top);
                rects.push(self.raw_rect(surface, 0, top16, aw, fits(height)));
            }
            if rects.is_empty() {
                return None;
            }
        } else {
            rects.push(self.raw_rect(surface, 0, 0, aw, ah));
        }

        self.remember(surface);

        #[allow(clippy::cast_possible_truncation)]
        let count = rects.len().min(usize::from(u16::MAX)) as u16;
        let mut out = update_header(count).to_vec();
        for rect in rects {
            out.extend_from_slice(&rect);
        }
        Some(out)
    }

    /// The runs of rows that differ from what the client has.
    ///
    /// Half-open `[top, bottom)` pairs, in order. Only called when the previous
    /// frame has the same shape, so a row's byte range is the same in both.
    fn damaged_bands(&self, surface: &Surface) -> Vec<(u32, u32)> {
        let stride = surface.stride() as usize;
        let mut bands = Vec::new();
        let mut run: Option<u32> = None;
        for y in 0..surface.height() {
            let at = y as usize * stride;
            let now = surface.row(y).unwrap_or(&[]);
            let before = self.last.get(at..at + stride).unwrap_or(&[]);
            let same = now == before;
            match (same, run) {
                (false, None) => run = Some(y),
                (true, Some(top)) => {
                    bands.push((top, y));
                    run = None;
                }
                _ => {}
            }
        }
        if let Some(top) = run {
            bands.push((top, surface.height()));
        }
        bands
    }

    /// One Raw-encoded rectangle (§7.7.1), header included.
    ///
    /// Pixels outside the surface are black rather than absent: the client was
    /// promised a rectangle of this size and has allocated for it.
    fn raw_rect(&self, surface: &Surface, x: u16, y: u16, width: u16, height: u16) -> Vec<u8> {
        let bpp = self.format.bytes_per_pixel();
        let mut out = rect_header(x, y, width, height, encoding::RAW).to_vec();
        out.reserve(usize::from(width) * usize::from(height) * bpp);

        // The common case by construction: the session allocates its surface in
        // BGRA8888, and BGRA8888 is byte for byte what the default RFB pixel
        // format asks for. A row is then a copy rather than a repack, which is
        // the difference between one memcpy and 288 000 shifts on a VGA frame.
        let direct = surface.format() == SurfaceFormat::BGRA8888
            && self.format == PixelFormat::DEFAULT
            && x == 0
            && u32::from(width) == surface.width();

        for row in 0..u32::from(height) {
            let sy = u32::from(y) + row;
            if direct && let Some(bytes) = surface.row(sy) {
                out.extend_from_slice(bytes);
                continue;
            }
            for column in 0..u32::from(width) {
                let rgb = surface.get(u32::from(x) + column, sy).unwrap_or([0, 0, 0]);
                self.format.put(self.format.pack(rgb), &mut out);
            }
        }
        out
    }

    /// Keep a copy of what the client now has.
    fn remember(&mut self, surface: &Surface) {
        self.last.clear();
        self.last.extend_from_slice(surface.pixels());
        self.last_shape = (surface.format(), surface.width(), surface.height());
    }
}

/// A pixel count as RFB carries it: sixteen bits, saturating.
///
/// A guest can program a CRTC for a width no client could allocate; clamping is
/// the only thing that keeps the wire format honest, and a 65 535-pixel-wide
/// framebuffer is not a case anybody has.
#[inline]
const fn fits(value: u32) -> u16 {
    if value > u16::MAX as u32 {
        u16::MAX
    } else {
        value as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn surface(width: u32, height: u32, fill: [u8; 3]) -> Surface {
        let mut s = Surface::new(SurfaceFormat::BGRA8888, width, height);
        s.fill(fill);
        s
    }

    #[test]
    fn the_first_update_is_a_whole_frame_even_when_incremental_was_asked() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        let s = surface(4, 2, [1, 2, 3]);
        let update = enc
            .update(&s, true)
            .expect("a client with nothing gets a frame");
        assert_eq!(&update[..4], update_header(1), "one rectangle");
        assert_eq!(
            update.len(),
            4 + 12 + 4 * 2 * 4,
            "header, rect header, and 4x2 32-bit pixels"
        );
        // BGRA8888 in, the default RFB format out: byte for byte the same.
        assert_eq!(&update[16..20], [3, 2, 1, 0xff]);
    }

    #[test]
    fn an_unchanged_frame_produces_nothing_incrementally_and_a_frame_otherwise() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        let s = surface(4, 2, [1, 2, 3]);
        enc.update(&s, false).expect("the first one");
        assert!(enc.update(&s, true).is_none(), "nothing changed");
        assert!(
            enc.update(&s, false).is_some(),
            "a full request is always answered"
        );
    }

    #[test]
    fn only_the_changed_rows_are_sent() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 4);
        let mut s = surface(4, 4, [0, 0, 0]);
        enc.update(&s, false).expect("the first one");
        s.put(2, 1, [0xff, 0, 0]);
        let update = enc.update(&s, true).expect("row 1 changed");
        assert_eq!(&update[..4], update_header(1));
        // y = 1, height = 1, full width.
        assert_eq!(&update[4..16], rect_header(0, 1, 4, 1, encoding::RAW));
        assert_eq!(update.len(), 4 + 12 + 4 * 4);
    }

    #[test]
    fn two_separated_changes_become_two_rectangles() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 5);
        let mut s = surface(4, 5, [0, 0, 0]);
        enc.update(&s, false).expect("the first one");
        s.put(0, 0, [1, 1, 1]);
        s.put(0, 4, [1, 1, 1]);
        let update = enc.update(&s, true).expect("two bands changed");
        assert_eq!(&update[..4], update_header(2));
        assert_eq!(&update[4..16], rect_header(0, 0, 4, 1, encoding::RAW));
        let second = 16 + 4 * 4;
        assert_eq!(
            &update[second..second + 12],
            rect_header(0, 4, 4, 1, encoding::RAW)
        );
    }

    #[test]
    fn a_changed_format_forces_a_whole_frame() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        let s = surface(4, 2, [1, 2, 3]);
        enc.update(&s, false).expect("the first one");
        assert!(enc.update(&s, true).is_none());
        let mut other = PixelFormat::DEFAULT;
        other.red_shift = 0;
        other.blue_shift = 16;
        enc.set_format(other);
        let update = enc
            .update(&s, true)
            .expect("everything it has is wrong now");
        assert_eq!(&update[16..20], [1, 2, 3, 0], "R G B in memory now");
    }

    #[test]
    fn a_resize_tells_a_client_that_asked_and_crops_for_one_that_did_not() {
        // Asked for it.
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        enc.set_encodings(&[encoding::RAW, encoding::DESKTOP_SIZE]);
        assert!(enc.resizable());
        enc.update(&surface(4, 2, [0, 0, 0]), false).expect("first");
        let update = enc
            .update(&surface(8, 4, [9, 9, 9]), true)
            .expect("resized");
        assert_eq!(&update[..4], update_header(2), "DesktopSize, then pixels");
        assert_eq!(
            &update[4..16],
            rect_header(0, 0, 8, 4, encoding::DESKTOP_SIZE)
        );
        assert_eq!(enc.announced(), (8, 4));

        // Did not ask.
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        enc.set_encodings(&[encoding::RAW]);
        assert!(!enc.resizable());
        enc.update(&surface(4, 2, [0, 0, 0]), false).expect("first");
        let update = enc
            .update(&surface(8, 4, [9, 9, 9]), true)
            .expect("changed");
        assert_eq!(&update[..4], update_header(1));
        assert_eq!(&update[4..16], rect_header(0, 0, 4, 2, encoding::RAW));
        assert_eq!(
            update.len(),
            4 + 12 + 4 * 2 * 4,
            "still the geometry it was promised"
        );
        assert_eq!(enc.announced(), (4, 2));
    }

    #[test]
    fn a_shrunken_surface_pads_with_black_rather_than_running_off_the_end() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 4, 2);
        let update = enc
            .update(&surface(2, 1, [0xff, 0xff, 0xff]), false)
            .expect("a frame");
        assert_eq!(update.len(), 4 + 12 + 4 * 2 * 4);
        // Row 1 does not exist in the surface, so it is black.
        let row1 = 16 + 4 * 4;
        assert_eq!(&update[row1..row1 + 4], [0, 0, 0, 0]);
    }

    #[test]
    fn an_empty_surface_is_not_a_panic() {
        let mut enc = FrameEncoder::new(PixelFormat::DEFAULT, 1, 1);
        let update = enc.update(&Surface::empty(), false).expect("a frame");
        assert_eq!(update.len(), 4 + 12 + 4);
    }

    #[test]
    fn a_sixteen_bit_client_gets_two_bytes_a_pixel() {
        let rgb565 = PixelFormat {
            bits_per_pixel: 16,
            depth: 16,
            big_endian: false,
            true_colour: true,
            red_max: 31,
            green_max: 63,
            blue_max: 31,
            red_shift: 11,
            green_shift: 5,
            blue_shift: 0,
        };
        let mut enc = FrameEncoder::new(rgb565, 2, 2);
        let update = enc
            .update(&surface(2, 2, [0xff, 0xff, 0xff]), false)
            .expect("a frame");
        assert_eq!(update.len(), 4 + 12 + 2 * 2 * 2);
        assert_eq!(&update[16..18], [0xff, 0xff]);
    }
}
