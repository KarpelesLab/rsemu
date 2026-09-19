//! The Amiga side of the scanout seam: Denise's picture as host pixels.
//!
//! [`DeniseScanout`] holds an `Arc<Video>` and does the two things Denise
//! deliberately does not: lay its `0x00RR_GGBB` picture words out as host
//! bytes, and turn a field's length into a frame period.
//!
//! The picture is eight bits a gun because Lisa's is (the AA chip set's
//! "256 colors deep and 25 bits wide (8 RED, 8 GREEN, 8 BLUE, 1 GENLOCK)",
//! *Specification for the Advanced Amiga (AA) Chip Set*, §1). An 8362's or an
//! 8373's four-bit guns reach it already expanded by [`rgb12_to_rgb888`]'s
//! `n × 17` — the expansion this adapter used to make itself — so their host
//! bytes are what they always were.
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
//! both of which are Agnus's business, and both of which an ECS Agnus's
//! programmable beam can change at any field (productivity mode's 114-count
//! lines, 525 to a field). Counting what was pushed rather than assuming a
//! standard is what keeps the period exact through that. A colour clock is two ticks of 7M, and
//! Denise's `clock` domain is its 7M pin (Appendix J: pin 35 `7M`, pin 36
//! `CCK`). So the period is `2 × clocks` ticks of that domain, converted to
//! virtual nanoseconds with the domain's exact rational frequency
//! (`CLAUDE.md`, determinism). A board that gives Denise no `clock`, or a
//! machine in which no field has finished yet, reports `0`.
//!
//! # The geometry
//!
//! Whatever Denise's current field is laid out as ([`Video::geometry`]):
//! 800 × 568 for a PAL A500 always, and for an ECS machine whatever its beam
//! and SuperHires make it, which can change between two fields. So
//! [`Scanout::capture`] takes the picture, its size and its field count from
//! Denise in one call ([`Video::copy_frame_rgb`]) and reshapes the host's surface
//! to it.
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
use alloc::vec::Vec;

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

/// A picture word, `0x00RR_GGBB`, as host bytes.
#[must_use]
#[inline]
pub const fn rgb24_to_rgb888(word: u32) -> [u8; 3] {
    [(word >> 16) as u8, (word >> 8) as u8, word as u8]
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
        // The picture, its size and its field count in one moment: an ECS
        // beam can change the geometry at any field, and a size read before
        // the rows could belong to a different field than the rows do. The
        // serial is taken with the pixels, never after them, for the reason
        // every adapter here gives: a serial never ahead of its pixels errs
        // toward one extra redraw rather than a missed one.
        let mut words = Vec::new();
        let (width, height, serial) = self.video.copy_frame_rgb(&mut words);
        dst.reshape(dst.format(), width, height);
        for (i, word) in words.iter().enumerate() {
            let (x, y) = (i as u32 % width, i as u32 / width);
            dst.put(x, y, rgb24_to_rgb888(*word));
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

    /// A programmable beam changes both the picture's shape and the field's
    /// length; the adapter follows both, and the period stays the exact
    /// rational of colour clocks over the 7M rate.
    #[test]
    fn a_programmed_raster_changes_the_geometry_and_the_period_stays_exact() {
        use crate::dev::amiga::denise::{Fetch, Line, Raster, Revision, Standard};

        let video = Arc::new(Video::with_revision(Standard::Pal, Revision::Ecs));
        // An A500's crystal over four.
        let scanout = DeniseScanout::new(Arc::clone(&video), Some((28_375_160, 4)));
        let field = |lines: u16, clocks: u16| {
            for vpos in 0..lines {
                video.line(&Line {
                    vpos,
                    clocks,
                    fetch: Fetch::default(),
                });
            }
            video.field(true);
        };
        field(313, 227);
        let info = scanout.info();
        assert_eq!((info.width, info.height), (800, 568));
        // 313 × 227 counts × 2 ticks of 7 093 790 Hz.
        assert_eq!(scanout.frame_period_ns(), 20_031_887);

        video.raster(Raster {
            first_line: 30,
            lines: 480,
            first_clock: 20,
            clocks: 90,
            line_clocks: 114,
        });
        field(525, 114);
        let info = scanout.info();
        assert_eq!((info.width, info.height), (360, 480));
        assert_eq!(scanout.frame_period_ns(), 16_873_913);
        let mut surface = Surface::new(PixelFormat::RGB888, 1, 1);
        scanout.capture(&mut surface);
        assert_eq!(
            (surface.info().width, surface.info().height),
            (360, 480),
            "capture reshapes to the field's picture"
        );
    }

    /// An 8362's picture reaches the host as it always did: the chip now
    /// expands its four-bit guns itself, and the adapter's old expansion of the
    /// same colour register gives the same bytes.
    #[test]
    fn an_ocs_picture_comes_out_byte_for_byte_as_before() {
        use crate::dev::amiga::denise::{Fetch, Line, Standard};

        let video = Arc::new(Video::new(Standard::Pal));
        let scanout = DeniseScanout::new(Arc::clone(&video), None);
        // A colour written the way a copper would write it, then one pixel's
        // bytes compared against the adapter's old expansion of it.
        let custom = crate::dev::amiga::regs::lookup(0x180).expect("COLOR00");
        crate::dev::amiga::custom::CustomChip::write(
            &*video,
            custom,
            0x0f80,
            crate::dev::amiga::custom::Origin::cpu(),
        );
        for vpos in 0..313 {
            video.line(&Line {
                vpos,
                clocks: 227,
                fetch: Fetch::default(),
            });
        }
        let mut surface = Surface::new(PixelFormat::RGB888, 1, 1);
        scanout.capture(&mut surface);
        let mut row = [0u16; 1];
        video.read_row(100, &mut row);
        assert_eq!(row[0], 0x0f80, "the twelve-bit view is the register");
        assert_eq!(surface.get(0, 100), Some(rgb12_to_rgb888(0x0f80)));
    }

    #[test]
    fn four_bits_a_gun_stretch_to_the_full_byte() {
        assert_eq!(rgb12_to_rgb888(0x0000), [0, 0, 0]);
        assert_eq!(rgb12_to_rgb888(0x0fff), [255, 255, 255]);
        assert_eq!(rgb12_to_rgb888(0x0f80), [255, 136, 0]);
        assert_eq!(rgb12_to_rgb888(0x0123), [17, 34, 51]);
    }
}
