//! Headless capture: a [`Surface`] as a PNG, a sequence of them as an APNG.
//!
//! This is how CI proves a machine draws a picture without a window, how
//! `docs/` gets screenshots that are regenerated rather than drawn, and how a
//! frame-hash regression reports *what* changed instead of only that the hash
//! moved (`ROADMAP.md` §12).
//!
//! # The encoder
//!
//! [`oxideav-png`](https://github.com/OxideAV/oxideav-png) — first-party, MIT,
//! pure Rust, and with `default-features = false` its whole dependency tree is
//! `compcol`, which the policy already permits (`CLAUDE.md`). It is behind the
//! `display-png` feature, so the default `cargo tree` is still just `rsemu`.
//!
//! It also does APNG, which is why [`encode_animation`] exists: a recorded run
//! is a real file format rather than a directory of numbered stills.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use oxideav_png::{PngImage, PngPixelFormat, encode_apng, encode_png_image};

use super::{PixelFormat, Surface};
use crate::core::error::{Error, Result};

/// Encode one surface as a PNG file.
///
/// # Errors
///
/// [`Error::Config`] if the encoder refuses the image. It carries
/// `display::png` as its location because the crate-level [`Error`] has no
/// codec variant yet and `core/` is not this module's to change; when it grows
/// one, this is the line that changes.
pub fn encode(surface: &Surface) -> Result<Vec<u8>> {
    let image = to_image(surface);
    encode_png_image(&image).map_err(|e| failed(&format!("{e}")))
}

/// Encode a sequence of equally-shaped surfaces as an animated PNG.
///
/// `delay_centiseconds` is APNG's own unit — hundredths of a second, applied to
/// every frame. A NES frame is 1.66 of them, so `2` is the honest rounding for
/// a 60 Hz capture and nothing here pretends otherwise.
///
/// # Errors
///
/// [`Error::Config`] if `frames` is empty, if the frames disagree on shape, or
/// if the encoder refuses them.
pub fn encode_animation(frames: &[Surface], delay_centiseconds: u16) -> Result<Vec<u8>> {
    let Some(first) = frames.first() else {
        return Err(failed("an animation needs at least one frame"));
    };
    for (i, frame) in frames.iter().enumerate() {
        if frame.width() != first.width() || frame.height() != first.height() {
            return Err(failed(&format!(
                "frame {i} is {}x{} but the first is {}x{}; APNG has one IHDR for the whole file",
                frame.width(),
                frame.height(),
                first.width(),
                first.height()
            )));
        }
    }
    let images: Vec<PngImage> = frames.iter().map(to_image).collect();
    encode_apng(&images, delay_centiseconds, 0).map_err(|e| failed(&format!("{e}")))
}

/// Build the encoder's view of a surface, repacking only when the byte order
/// is not one PNG has a colour type for.
fn to_image(surface: &Surface) -> PngImage {
    let (pixel_format, data) = match surface.format() {
        PixelFormat::RGB888 => (PngPixelFormat::Rgb24, surface.pixels().to_vec()),
        PixelFormat::RGBA8888 => (PngPixelFormat::Rgba, surface.pixels().to_vec()),
        // BGRA — and any format a future backend adds — goes through the
        // surface's own accessor, so this stays correct without knowing the
        // layout.
        _ => {
            let mut data = Vec::with_capacity((surface.width() as usize) * 4);
            for y in 0..surface.height() {
                for x in 0..surface.width() {
                    let rgb = surface.get(x, y).unwrap_or([0, 0, 0]);
                    data.extend_from_slice(&[rgb[0], rgb[1], rgb[2], 0xff]);
                }
            }
            (PngPixelFormat::Rgba, data)
        }
    };
    let stride = (surface.width() as usize) * pixel_format.bytes_per_pixel();
    PngImage {
        width: surface.width(),
        height: surface.height(),
        pixel_format,
        stride,
        data,
        palette: Vec::new(),
    }
}

/// One place the codec's complaint becomes the crate's error.
fn failed(message: &str) -> Error {
    Error::Config {
        at: String::from("display::png"),
        message: String::from(message),
    }
}
