//! The Amiga side of the audio seam: Paula's four channels as host frames.
//!
//! [`PaulaAudio`] holds an `Arc<Paula>` and does the two things the chip
//! deliberately does not: it says at what *rate* the frames come out, and it
//! scales the board's analogue sum into the seam's signed 16-bit unit.
//!
//! # The rate
//!
//! One stereo frame every [`AUDIO_SAMPLE_DIVISOR`] colour clocks
//! ([`dev::amiga::paula`](crate::dev::amiga::paula), which argues for that
//! divisor), so a PAL A500 produces 3 546 895 / 32 = 110 840.46875 frames a
//! second — not an integer, which is exactly what [`StreamInfo`]'s rational is
//! for, and not a constant either: it is resolved from the machine's clock
//! forest by [`capture::take`], because a colour clock is a fact about the
//! board's crystal (PAL `clk / 8` of 28.375160 MHz, NTSC of 28.636360) rather
//! than about this file. [`COLOUR_CLOCK_PAL`] is the fallback for a caller with
//! no machine to ask.
//!
//! Chapter 5 of the *Amiga Hardware Reference Manual* bounds what the rate has
//! to carry: a channel plays one sample every `AUDxPER` colour clocks and the
//! minimum period is "124 color clocks", so the fastest anything can come out
//! of the chip is 3 546 895 / 124 ≈ 28 604 samples a second on a PAL machine
//! and 3 579 545 / 124 ≈ 28 867 on an NTSC one. The frame rate is 3.87 times
//! that.
//!
//! # Stereo, because the machine is
//!
//! Chapter 5 wires **channels 0 and 3 to the left output and 1 and 2 to the
//! right**, so this is a two-channel source and a mono mixdown would throw away
//! the one thing an Amiga is famous for. Paula hands over the two mixer sums
//! already separated; interleaving them is the whole of the conversion, bar the
//! scale.
//!
//! # The scale
//!
//! A device frame is the board's analogue sum: `sample × volume` for each of a
//! side's two channels, an 8-bit signed sample and a volume of 0 to 64, so
//! ±16 384 — fifteen bits and a sign. The seam's unit is a full-scale `i16`, so
//! each side is **doubled**, which is exact: −16 384 becomes exactly
//! [`i16::MIN`] and the largest positive sum, 2 × 127 × 64, becomes 32 512. The
//! clamp is belt and braces.
//!
//! # No analogue stage
//!
//! [`StreamInfo::output_stage`] is empty, and that is a decision rather than an
//! omission: the fixed RC low-pass and the switchable "LED" low-pass are on the
//! board rather than in Paula, the switchable one is switched by CIA-A's `PA1`
//! while this seam's stage is fixed when the resampler is built, and the
//! Hardware Reference Manual gives neither corner. `dev::amiga::paula`'s module
//! documentation has the full argument, and [`gb`](super::gb) set the
//! precedent: a corner nobody measured would be an invented measurement.
//!
//! # Getting hold of the chip
//!
//! Exactly as [`audio::gb::capture`](super::gb::capture) gets hold of a DMG's
//! APU, and for the same reason: `machine::build` hands back `Arc<dyn Device>`
//! and `Device` keeps `Any` out of its supertrait chain on purpose, so the host
//! takes its handle at the one moment the concrete type exists —
//! construction — by replacing the class's constructor.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, SampleFormat, StreamInfo, gcd};
use crate::dev::amiga::paula::{AUDIO_SAMPLE_DIVISOR, Paula};

/// A PAL Amiga's colour clock, in hertz: `clk / 8` of the 28.375160 MHz
/// crystal, which the manual writes as 3.546895 MHz (Chapter 5, and the
/// timings throughout).
pub const COLOUR_CLOCK_PAL: u64 = 3_546_895;

/// An [`AudioSource`] over an Amiga's Paula.
#[derive(Debug)]
pub struct PaulaAudio {
    paula: Arc<Paula>,
    /// The exact frame rate, as a rational in lowest terms.
    rate: (u64, u64),
}

impl PaulaAudio {
    /// Listen to `paula` at a PAL A500's rate.
    #[must_use]
    pub fn new(paula: Arc<Paula>) -> PaulaAudio {
        PaulaAudio::with_rate(paula, COLOUR_CLOCK_PAL, AUDIO_SAMPLE_DIVISOR)
    }

    /// Listen to `paula` at exactly `num / den` hertz.
    ///
    /// For a board whose colour clock is not a PAL A500's — an NTSC one, or
    /// anything else that instantiates the class. Reduced to lowest terms on
    /// the way in, because an unreduced rational costs the resampler `u64`
    /// range for nothing; a zero denominator is corrected to one, as
    /// [`StreamInfo::new`] corrects one.
    #[must_use]
    pub fn with_rate(paula: Arc<Paula>, num: u64, den: u64) -> PaulaAudio {
        let (num, den) = (num.max(1), den.max(1));
        let g = gcd(num, den);
        PaulaAudio {
            paula,
            rate: (num / g, den / g),
        }
    }

    /// The chip being listened to, for a host that wants its registers too.
    #[must_use]
    pub fn paula(&self) -> &Arc<Paula> {
        &self.paula
    }
}

impl AudioSource for PaulaAudio {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(self.rate.0, self.rate.1, 2, SampleFormat::S16)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        let frames = self.paula.take_audio();
        out.reserve(frames.len() * 2);
        for (left, right) in &frames {
            out.push(scale(*left));
            out.push(scale(*right));
        }
        frames.len() as u64
    }

    fn dropped(&self) -> u64 {
        self.paula.audio_dropped()
    }
}

/// One side's analogue sum as a full-scale host sample. See *The scale*.
#[inline]
fn scale(sum: i16) -> i16 {
    (i32::from(sum) * 2).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// The interception that gets a host an `Arc<Paula>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{AUDIO_SAMPLE_DIVISOR, Arc, Paula, PaulaAudio};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::amiga::paula::CLASS_NAME;
    use crate::machine::{BuildOptions, Machine};

    /// Replace `amiga.paula`'s constructor in `options` with one that keeps a
    /// handle and switches recording on.
    ///
    /// The one call a host makes between `catalog::build_options` and
    /// `machine::build`.
    ///
    /// **Switching recording on is the only thing this changes about the
    /// machine**, and it has to change it: `machines/amiga-a500.machine`
    /// leaves `record` at its default of false, so a host that did not ask
    /// would get a chip that never fills its ring. Nothing guest-visible
    /// depends on the flag — the ring is output rather than architectural
    /// state, it is absent from the snapshot, and no register reports its
    /// depth.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Paula>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let paula = Arc::new(Paula::new(props)?);
            paula.set_recording(true);
            seen.push(&paula);
            Ok(paula)
        });
        Ok(())
    }

    /// The Paula this build constructed, as a [`PaulaAudio`] at the rate
    /// `machine`'s clock forest says its colour clock runs at.
    ///
    /// The most recent one, for a machine with more than one. `None` if this
    /// build has no Paula in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<PaulaAudio> {
        let seen = hosts
            .get::<Captured<Paula>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        let paula = seen.take()?;
        match resolve_rate(machine) {
            Some((num, den)) => Some(PaulaAudio::with_rate(paula, num, den)),
            None => Some(PaulaAudio::new(paula)),
        }
    }

    /// The chip's clock domain frequency divided by the sample divisor.
    ///
    /// The same resolution `display::amiga::capture` does for Denise's pixel
    /// rate and for the same reason: the colour clock is a fact about the
    /// oscillator forest, read from the forest rather than written twice. A
    /// machine with several Paulas is matched by class, taking the last —
    /// which is the one [`take`] returned.
    fn resolve_rate(machine: &Machine) -> Option<(u64, u64)> {
        let entry = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)?;
        let domain = entry.domain()?;
        let freq = machine.clocks().domain_frequency(domain).ok()?;
        Some((freq.num(), freq.den().saturating_mul(AUDIO_SAMPLE_DIVISOR)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::chardev::CharPort;
    use alloc::string::String;

    fn chip() -> Arc<Paula> {
        Arc::new(Paula::with_port(
            String::from("custom"),
            Arc::new(CharPort::new()),
            String::from("serial"),
        ))
    }

    #[test]
    fn a_pal_a500s_rate_is_the_colour_clock_over_the_divisor() {
        let audio = PaulaAudio::new(chip());
        let info = audio.info();
        // 3 546 895 / 32, in lowest terms: the numerator is odd, so there is
        // nothing to reduce and the rate is not a whole number of hertz.
        assert_eq!((info.rate_num, info.rate_den), (3_546_895, 32));
        assert_eq!(info.rate_hz(), 110_840);
        assert_eq!(info.channels, 2, "0 and 3 left, 1 and 2 right");
        assert!(
            info.output_stage.is_empty(),
            "the board's filters are not modelled, and this says so"
        );
    }

    #[test]
    fn the_frame_rate_carries_the_fastest_a_channel_can_be_driven() {
        // Chapter 5: the minimum period is 124 colour clocks, so a channel
        // reaches 28 604 samples a second on a PAL machine.
        assert_eq!(
            (COLOUR_CLOCK_PAL + 62) / 124,
            28_604,
            "rounded: 124 · 28 604 is 3 546 896"
        );
        let info = PaulaAudio::new(chip()).info();
        let frames = info.rate_num / info.rate_den;
        assert!(
            frames > 2 * (COLOUR_CLOCK_PAL / 124),
            "{frames} frames a second does not carry 28 604 samples a second"
        );
    }

    #[test]
    fn an_ntsc_colour_clock_gives_another_rate_in_lowest_terms() {
        // 28 636 360 / 8 / 32 — the NTSC crystal, as the clock forest would
        // hand it over before reduction.
        let audio = PaulaAudio::with_rate(chip(), 28_636_360, 256);
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), (3_579_545, 32));
    }

    #[test]
    fn draining_a_silent_chip_appends_nothing() {
        let audio = PaulaAudio::new(chip());
        let mut out = Vec::new();
        assert_eq!(audio.drain(&mut out), 0);
        assert!(out.is_empty());
        assert_eq!(audio.dropped(), 0);
    }

    #[test]
    fn the_analogue_sum_doubles_into_the_seams_unit() {
        // A side at rest, at full negative and at full positive.
        assert_eq!(scale(0), 0);
        assert_eq!(
            scale(-16_384),
            i16::MIN,
            "two channels at -128 and volume 64"
        );
        assert_eq!(scale(127 * 64 * 2), 32_512);
    }
}
