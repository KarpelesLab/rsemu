//! The SNES side of the audio seam: the S-DSP's 32 kHz stereo as host frames.
//!
//! [`SnesAudio`] holds an `Arc<SnesApu>` and says at what rate the frames come
//! out. The S-DSP produces one stereo sample per 32 SMP cycles, and the SMP
//! runs at the audio unit's own crystal divided by 24 — 24.576 MHz / 24 / 32 =
//! **32 000 Hz exactly** with the nominal crystal. That crystal is independent
//! of the S-CPU's and real units vary (`ROADMAP.md` §4.2), so the rate is
//! resolved from the realized machine's clock forest when there is one;
//! [`NOMINAL_SAMPLE_RATE`] is the fallback.
//!
//! No analogue output stage is declared: the console's post-DAC filter is not
//! modelled, and inventing a corner would be inventing a measurement.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, SampleFormat, StreamInfo, gcd};
use crate::dev::snes::apu::{SAMPLE_DIVISOR, SnesApu};

/// 32 000 / 1 Hz: the nominal 24.576 MHz crystal over 24 × 32.
pub const NOMINAL_SAMPLE_RATE: (u64, u64) = (32_000, 1);

/// An [`AudioSource`] over a SNES audio unit.
#[derive(Debug)]
pub struct SnesAudio {
    apu: Arc<SnesApu>,
    rate: (u64, u64),
}

impl SnesAudio {
    /// Listen to `apu` at the nominal 32 kHz.
    #[must_use]
    pub fn new(apu: Arc<SnesApu>) -> SnesAudio {
        SnesAudio {
            apu,
            rate: NOMINAL_SAMPLE_RATE,
        }
    }

    /// Listen at exactly `num / den` hertz, reduced on the way in.
    #[must_use]
    pub fn with_rate(apu: Arc<SnesApu>, num: u64, den: u64) -> SnesAudio {
        let (num, den) = (num.max(1), den.max(1));
        let g = gcd(num, den);
        SnesAudio {
            apu,
            rate: (num / g, den / g),
        }
    }

    /// The unit being listened to.
    #[must_use]
    pub fn apu(&self) -> &Arc<SnesApu> {
        &self.apu
    }
}

impl AudioSource for SnesAudio {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(self.rate.0, self.rate.1, 2, SampleFormat::S16)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        let frames = self.apu.take_samples();
        out.reserve(frames.len() * 2);
        for (l, r) in &frames {
            out.push(*l);
            out.push(*r);
        }
        frames.len() as u64
    }
}

/// The interception that gets a host an `Arc<SnesApu>` out of a described
/// machine, the same seam `audio::sms::capture` is.
pub mod capture {
    use super::{Arc, SAMPLE_DIVISOR, SnesApu, SnesAudio};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::snes::apu::CLASS;
    use crate::machine::{BuildOptions, Machine};

    /// Replace `snes.apu`'s constructor with one that keeps a handle and
    /// switches recording on. Recording changes nothing guest-visible: the
    /// ring is not architectural state and is not snapshotted.
    ///
    /// # Errors
    ///
    /// If something else already claimed this build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<SnesApu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let apu = Arc::new(SnesApu::from_props(props)?);
            apu.set_recording(true);
            seen.push(&apu);
            Ok(apu)
        });
        Ok(())
    }

    /// The unit this build constructed, at the rate the clock forest gives.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<SnesAudio> {
        let seen = hosts
            .get::<Captured<SnesApu>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?;
        let apu = seen.take()?;
        let rate = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS.name)
            .and_then(|d| d.domain())
            .and_then(|dom| machine.clocks().domain_frequency(dom).ok())
            .map(|f| (f.num(), f.den().saturating_mul(SAMPLE_DIVISOR)));
        Some(match rate {
            Some((n, d)) => SnesAudio::with_rate(apu, n, d),
            None => SnesAudio::new(apu),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dev::snes::apu::NullDsp;
    use alloc::boxed::Box;

    #[test]
    fn the_nominal_rate_is_32_khz_stereo() {
        let audio = SnesAudio::new(Arc::new(SnesApu::new(Box::new(NullDsp::default()))));
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), (32_000, 1));
        assert_eq!(info.channels, 2);
        assert!(info.output_stage.is_empty());
        // 1.024 MHz / 32, reduced.
        let a = SnesAudio::with_rate(Arc::clone(audio.apu()), 1_024_000, 32);
        assert_eq!((a.info().rate_num, a.info().rate_den), (32_000, 1));
    }

    #[test]
    fn draining_interleaves_left_and_right() {
        let apu = Arc::new(SnesApu::new(Box::new(
            crate::dev::snes::dsp::SDspChip::new(),
        )));
        apu.set_recording(true);
        apu.run_budget(32 * 10);
        let audio = SnesAudio::new(Arc::clone(&apu));
        let mut out = Vec::new();
        let n = audio.drain(&mut out);
        assert!(n >= 9);
        assert_eq!(out.len() as u64, n * 2);
    }
}
