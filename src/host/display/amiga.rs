//! The Amiga side of the scanout seam: Denise's picture as host pixels.
//!
//! [`DeniseScanout`] holds an `Arc<Video>` and does the two things Denise
//! deliberately does not: expand its 12-bit `0RGB` words — four bits a gun,
//! the colour register's encoding and what the chip's twelve RGB pins carry —
//! to eight bits a channel, and turn a field's length into a frame period.
//!
//! # Why this is not `host::display::panel`
//!
//! `crate::dev::amiga::denise` has the argument at length: a smart panel's
//! counter counts content changes and it has no frame rate, while Denise emits
//! a field every 20 ms whether or not anything changed and a host has to
//! advance the machine by exactly one field to see the next one. So
//! [`Scanout::frame_counter`] here is [`Video::fields`], and
//! [`Scanout::frame_period_ns`] is real.
//!
//! # The frame period
//!
//! A field lasts as many colour clocks as the beam source pushed in it
//! ([`Video::field_clocks`]) — 227 or 228 per line, times the field's lines,
//! both of which are Agnus's business. A colour clock is two ticks of 7M, and
//! Denise's `clock` domain is its 7M pin (Appendix J: pin 35 `7M`, pin 36
//! `CCK`). So the period is `2 × clocks` ticks of that domain, converted to
//! virtual nanoseconds with the domain's exact rational frequency
//! (`CLAUDE.md`, determinism). A board that gives Denise no `clock`, or a
//! machine in which no field has finished yet, reports `0`.
//!
//! # Getting hold of the chip
//!
//! As for every other adapter here, and for the same reason: `Device` has no
//! `Any` in its supertrait chain, so the host keeps its handle at construction.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::amiga::capture::install(&mut options)?;      // intercept amiga.denise
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::amiga::capture::take(&options.realize.hosts, &machine);
//! ```
//!
//! This is a seam and it is marked as one. When `Device` grows a scanout hook,
//! every line of [`capture`] deletes.

use alloc::sync::Arc;
use alloc::vec;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::amiga::denise::Video;

/// Four bits of gun to eight bits of channel: `n × 17` maps `0..=15` exactly
/// onto `0..=255`.
#[must_use]
#[inline]
pub const fn rgb12_to_rgb888(word: u16) -> [u8; 3] {
    [
        ((word >> 8) & 0xf) as u8 * 17,
        ((word >> 4) & 0xf) as u8 * 17,
        (word & 0xf) as u8 * 17,
    ]
}

/// A [`Scanout`] over a Denise.
#[derive(Debug, Clone)]
pub struct DeniseScanout {
    video: Arc<Video>,
    /// The frequency of Denise's `clock` domain as `(numerator, denominator)`
    /// hertz, or `None` if the board gave it none.
    rate: Option<(u64, u64)>,
}

impl DeniseScanout {
    /// Watch `video`, with the frequency of its 7M clock if there is one.
    #[must_use]
    pub fn new(video: Arc<Video>, rate: Option<(u64, u64)>) -> DeniseScanout {
        DeniseScanout { video, rate }
    }

    /// The chip being watched.
    #[must_use]
    pub fn video(&self) -> &Arc<Video> {
        &self.video
    }
}

impl Scanout for DeniseScanout {
    fn info(&self) -> SurfaceInfo {
        let (width, height) = self.video.geometry();
        SurfaceInfo::new(width, height, PixelFormat::RGB888)
    }

    fn frame_counter(&self) -> u64 {
        self.video.fields()
    }

    fn frame_period_ns(&self) -> u64 {
        let Some((num, den)) = self.rate else {
            return 0;
        };
        if num == 0 {
            return 0;
        }
        // Two 7M ticks per colour clock. u128 because a field is some 142,000
        // ticks and the denominator and 10⁹ multiply it further.
        let ticks = u128::from(self.video.field_clocks()) * 2;
        let ns = ticks * u128::from(den) * 1_000_000_000 / u128::from(num);
        u64::try_from(ns).unwrap_or(u64::MAX)
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);
        // The counter before the pixels, for the reason every adapter here
        // gives: a serial never ahead of its pixels errs toward one extra
        // redraw rather than a missed one.
        let serial = self.video.fields();
        let mut row = vec![0u16; info.width as usize];
        for y in 0..info.height {
            self.video.read_row(y, &mut row);
            for (x, word) in row.iter().enumerate() {
                dst.put(x as u32, y, rgb12_to_rgb888(*word));
            }
        }
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<Video>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use alloc::sync::Arc;

    use super::DeniseScanout;
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::amiga::denise::{CLASS_NAME, Denise, Video};
    use crate::machine::{BuildOptions, Machine};

    /// Replace `amiga.denise`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Video>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let denise = Arc::new(Denise::new(props)?);
            seen.push(denise.video());
            Ok(denise)
        });
        Ok(())
    }

    /// The Denise this build constructed, with its 7M rate resolved from
    /// `machine`'s clock forest.
    ///
    /// The most recent one, for a machine with several. `None` if this build
    /// has no Denise in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<DeniseScanout> {
        let seen = hosts
            .get::<Captured<Video>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        let video = seen.take()?;
        let rate = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)
            .and_then(|d| d.domain())
            .and_then(|domain| machine.clocks().domain_frequency(domain).ok())
            .map(|f| (f.num(), f.den()));
        Some(DeniseScanout::new(video, rate))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn four_bits_a_gun_stretch_to_the_full_byte() {
        assert_eq!(rgb12_to_rgb888(0x0000), [0, 0, 0]);
        assert_eq!(rgb12_to_rgb888(0x0fff), [255, 255, 255]);
        assert_eq!(rgb12_to_rgb888(0x0f80), [255, 136, 0]);
        assert_eq!(rgb12_to_rgb888(0x0123), [17, 34, 51]);
    }
}
