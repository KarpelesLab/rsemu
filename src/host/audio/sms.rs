//! The Master System side of the audio seam: an SN76489's four channels as host
//! frames.
//!
//! [`SmsAudio`] holds an `Arc<SmsPsg>` and does the two things the chip does
//! not: it says at what *rate* the samples come out, and it presents the chip's
//! one output as the seam's frames.
//!
//! # The rate, and why it needs the machine
//!
//! The PSG takes a sample every [`SAMPLE_DIVISOR`] input ticks
//! ([`dev::sms::psg`](crate::dev::sms::psg)), and its input is the Z80's own
//! clock domain — the console's master crystal divided by three. Neither region
//! gives an integer:
//!
//! | Region | Z80 | ÷ 80 | Sample rate |
//! | --- | --- | --- | --- |
//! | NTSC | 315 000 000 / 88 Hz | | 3 937 500 / 88 Hz ≈ 44 744.32 |
//! | PAL | 3 546 895 Hz | | 709 379 / 16 Hz = 44 336.19 |
//!
//! which is exactly why [`StreamInfo`] carries a rational (`CLAUDE.md`,
//! determinism). And unlike the NES's APU, this chip exposes no region
//! accessor, so the adapter cannot work the rate out from the device alone: it
//! is resolved from the realized machine's clock forest, the same way
//! `display::lcd::capture` resolves a frame period. [`NTSC_SAMPLE_RATE`] is the
//! fallback for a caller with no machine to ask.
//!
//! # Mono, and no analogue stage
//!
//! An SN76489 has one output pin. The Game Gear's stereo pan register at `$06`
//! is not modelled — `dev::sms::psg` says so — so the honest channel count is
//! **1** rather than the chip's own sample duplicated into two, and a host that
//! wants two gets them from the rest of the path rather than from a lie here.
//! No [`Pole`](super::Pole) is declared for the same reason as the Game Boy's:
//! the console's output network is not modelled, and inventing a corner would
//! be inventing a measurement.
//!
//! # What the device does not offer
//!
//! The same two gaps the Game Boy has, and they are `dev/`'s to close rather
//! than this module's: no dropped-sample counter (the ring pops its oldest frame
//! in silence, so [`AudioSource::dropped`] can only answer zero) and no
//! host-sizable ring (`RING_FRAMES` is a `const` of 16 384 frames, about a third
//! of a second, and the only property the class takes is `record`). A headless
//! recording is therefore drained as the run goes rather than at the end; see
//! `src/bin/rsemu.rs` for why that is still honest.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, SampleFormat, StreamInfo, gcd};
use crate::dev::sms::psg::{SAMPLE_DIVISOR, SmsPsg};

/// The NTSC console's sample rate, as an exact rational: 3 937 500 / 88 Hz.
///
/// The Master System's NTSC master crystal is 10 738 636.36… Hz — 315/88 MHz
/// times three, the same colour-burst-derived number every NTSC console of the
/// era uses — and the Z80 runs at a third of it, 315 000 000 / 88 Hz. Divided by
/// [`SAMPLE_DIVISOR`], that is 3 937 500 / 88, about 44 744.32 Hz.
///
/// A fallback, not an assumption: [`capture::take`] reads the real number out of
/// the machine, which is what makes a PAL console come out at 44 336.19 Hz
/// instead.
pub const NTSC_SAMPLE_RATE: (u64, u64) = (3_937_500, 88);

/// An [`AudioSource`] over a Master System's PSG.
#[derive(Debug)]
pub struct SmsAudio {
    psg: Arc<SmsPsg>,
    /// The exact sample rate, as a rational. Resolved from the machine's clock
    /// forest by [`capture::take`], or [`NTSC_SAMPLE_RATE`] otherwise.
    rate: (u64, u64),
}

impl SmsAudio {
    /// Listen to `psg`, assuming an NTSC console.
    #[must_use]
    pub fn new(psg: Arc<SmsPsg>) -> SmsAudio {
        SmsAudio {
            psg,
            rate: NTSC_SAMPLE_RATE,
        }
    }

    /// Listen to `psg` at exactly `num / den` hertz.
    ///
    /// Reduced to lowest terms on the way in, because an unreduced rational
    /// costs the resampler `u64` range for nothing; a zero denominator is
    /// corrected to one, as [`StreamInfo::new`] corrects one.
    #[must_use]
    pub fn with_rate(psg: Arc<SmsPsg>, num: u64, den: u64) -> SmsAudio {
        let (num, den) = (num.max(1), den.max(1));
        let g = gcd(num, den);
        SmsAudio {
            psg,
            rate: (num / g, den / g),
        }
    }

    /// The chip being listened to, for a host that wants its registers too.
    #[must_use]
    pub fn psg(&self) -> &Arc<SmsPsg> {
        &self.psg
    }
}

impl AudioSource for SmsAudio {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(self.rate.0, self.rate.1, 1, SampleFormat::S16)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        // The device rings its mono sample into both halves of a pair, because
        // its ring is typed for a stereo console it might one day be in. One
        // channel is what an SN76489 has, so the left half is the sample and
        // the right half is the same number again: taking one is not throwing
        // information away.
        let frames = self.psg.take_samples();
        out.reserve(frames.len());
        for (left, _right) in &frames {
            out.push(*left);
        }
        frames.len() as u64
    }
}

/// The interception that gets a host an `Arc<SmsPsg>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, SAMPLE_DIVISOR, SmsAudio, SmsPsg};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::sms::psg::CLASS;
    use crate::machine::{BuildOptions, Machine};

    /// Replace `sms.psg`'s constructor in `options` with one that keeps a handle
    /// and switches recording on.
    ///
    /// **Switching recording on is the only thing this changes about the
    /// machine.** Both region files default `record` to false, so a host that
    /// did not ask would get a chip that generates its samples and throws every
    /// one away. Nothing guest-visible depends on the flag: the ring is output
    /// rather than architectural state, it is left out of the snapshot
    /// deliberately, and no port reports its depth.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<SmsPsg>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let psg = Arc::new(SmsPsg::from_props(props)?);
            psg.set_recording(true);
            seen.push(&psg);
            Ok(psg)
        });
        Ok(())
    }

    /// The PSG this build constructed, as an [`SmsAudio`] at the rate
    /// `machine`'s clock forest says it runs at.
    ///
    /// The most recent one, for a machine with more than one. `None` if this
    /// build has no PSG in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<SmsAudio> {
        let seen = hosts
            .get::<Captured<SmsPsg>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?;
        let psg = seen.take()?;
        match resolve_rate(machine) {
            Some((num, den)) => Some(SmsAudio::with_rate(psg, num, den)),
            None => Some(SmsAudio::new(psg)),
        }
    }

    /// The chip's clock domain frequency divided by the sample divisor.
    fn resolve_rate(machine: &Machine) -> Option<(u64, u64)> {
        let entry = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS.name)?;
        let domain = entry.domain()?;
        let freq = machine.clocks().domain_frequency(domain).ok()?;
        Some((freq.num(), freq.den().saturating_mul(SAMPLE_DIVISOR)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ntsc_rate_is_the_z80s_own_clock_over_eighty() {
        // 315 000 000 / 88 is the NTSC Z80's exact frequency; the divisor is
        // sixteen input ticks a tone step, five steps a sample.
        assert_eq!(SAMPLE_DIVISOR, 80);
        assert_eq!(NTSC_SAMPLE_RATE, (315_000_000 / 80, 88));
        let audio = SmsAudio::new(Arc::new(SmsPsg::new()));
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), NTSC_SAMPLE_RATE);
        assert_eq!(info.channels, 1, "an SN76489 has one output pin");
        assert!(info.output_stage.is_empty());
    }

    #[test]
    fn a_pal_console_is_a_different_rational() {
        // 3 546 895 / 80 = 709 379 / 16, and it comes out reduced.
        let audio = SmsAudio::with_rate(Arc::new(SmsPsg::new()), 3_546_895, 80);
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), (709_379, 16));
    }

    #[test]
    fn draining_a_silent_chip_appends_nothing() {
        let audio = SmsAudio::new(Arc::new(SmsPsg::new()));
        let mut out = Vec::new();
        assert_eq!(audio.drain(&mut out), 0);
        assert!(out.is_empty());
        assert_eq!(audio.dropped(), 0, "the chip has no counter to report");
    }
}
