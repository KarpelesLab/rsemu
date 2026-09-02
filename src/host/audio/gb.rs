//! The Game Boy side of the audio seam: a DMG's four channels as host frames.
//!
//! [`GbAudio`] holds an `Arc<GbApu>` and does the two things the chip
//! deliberately does not: it says at what *rate* those samples come out, and it
//! flattens the device's stereo pairs into the interleaved buffer the rest of
//! the path speaks.
//!
//! # The rate
//!
//! One frame every [`SAMPLE_DIVISOR`] crystal periods
//! ([`dev::gb::apu`](crate::dev::gb::apu)), so a DMG at 4 194 304 Hz produces
//! exactly 32 768 frames a second — an integer, unlike every rate on the NES,
//! because the Game Boy's crystal is a power of two. It is still resolved from
//! the machine's clock forest rather than written down here: the rate is a fact
//! about the oscillator the APU is on, and a board built around a different
//! crystal is entitled to a different answer (`CLAUDE.md`, determinism). The
//! constant is the fallback for a caller that has no machine to ask.
//!
//! # Stereo, and no analogue stage
//!
//! The chip mixes to two channels — NR51 pans each of the four to either side
//! and NR50 sets a master volume per side — so this is the seam's first stereo
//! source, and [`StreamInfo::channels`] is 2. There is no [`Pole`] declared,
//! and that is a statement rather than an omission: `dev::gb::apu` says in as
//! many words that the DMG's output high-pass is not modelled, so declaring a
//! corner here would be inventing a measurement. When the chip grows one, it is
//! one line in [`GbAudio::info`].
//!
//! [`Pole`]: super::Pole
//!
//! # What the device does not offer
//!
//! Two things, both of which the NES's APU does and both of which would be a
//! change in `dev/` rather than here:
//!
//! * **No dropped-sample counter.** The ring pops its oldest frame in silence,
//!   so [`AudioSource::dropped`] can only answer zero. A host that fell behind
//!   learns nothing.
//! * **No host-sizable ring.** `RING_FRAMES` is a `const` — 8 192 frames, a
//!   quarter of a second — and the only property the class takes is `record`.
//!   The NES sizes its ring for the whole run under `--record-audio`; this one
//!   cannot, so a headless recording has to be drained as the run goes. The
//!   binary does exactly that, and `src/bin/rsemu.rs` argues why that is still
//!   honest: `Machine::run_for` is additive (`ROADMAP.md` §11.6), so a run cut
//!   into slices reaches the same state as the same run taken whole.
//!
//! # Getting hold of the APU
//!
//! Exactly as [`audio::nes::capture`](super::nes::capture) gets hold of an
//! RP2A03, and for the same reason: `machine::build` hands back `Arc<dyn Device>`
//! and `Device` keeps `Any` out of its supertrait chain on purpose. So the host
//! takes its handle at the one moment the concrete type exists — construction —
//! by replacing the class's constructor. A seam, marked as one: when `Device`
//! grows an audio hook, every line of [`capture`] deletes.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, SampleFormat, StreamInfo, gcd};
use crate::dev::gb::apu::{GbApu, SAMPLE_DIVISOR, SAMPLE_RATE};

/// An [`AudioSource`] over a Game Boy's sound chip.
#[derive(Debug)]
pub struct GbAudio {
    apu: Arc<GbApu>,
    /// The exact sample rate, as a rational. Resolved from the machine's clock
    /// forest by [`capture::take`], or [`SAMPLE_RATE`] for a caller with no
    /// machine to ask.
    rate: (u64, u64),
}

impl GbAudio {
    /// Listen to `apu` at the DMG's own rate.
    #[must_use]
    pub fn new(apu: Arc<GbApu>) -> GbAudio {
        GbAudio {
            apu,
            rate: (SAMPLE_RATE, 1),
        }
    }

    /// Listen to `apu` at exactly `num / den` hertz.
    ///
    /// For a board whose crystal is not a DMG's. Reduced to lowest terms on the
    /// way in, because an unreduced rational costs the resampler `u64` range for
    /// nothing; a zero denominator is corrected to one, as [`StreamInfo::new`]
    /// corrects one.
    #[must_use]
    pub fn with_rate(apu: Arc<GbApu>, num: u64, den: u64) -> GbAudio {
        let (num, den) = (num.max(1), den.max(1));
        let g = gcd(num, den);
        GbAudio {
            apu,
            rate: (num / g, den / g),
        }
    }

    /// The chip being listened to, for a host that wants its registers too.
    #[must_use]
    pub fn apu(&self) -> &Arc<GbApu> {
        &self.apu
    }
}

impl AudioSource for GbAudio {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(self.rate.0, self.rate.1, 2, SampleFormat::S16)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        // `take_samples` allocates a `Vec` of pairs of its own — the device's
        // API, and not one this module may change from here (`dev/` is not
        // ours to edit for a host convenience). Interleaving it is the whole
        // of the conversion: the chip's samples are already signed and already
        // centred, so unlike the NES there is nothing to shift.
        let frames = self.apu.take_samples();
        out.reserve(frames.len() * 2);
        for (left, right) in &frames {
            out.push(*left);
            out.push(*right);
        }
        frames.len() as u64
    }
}

/// The interception that gets a host an `Arc<GbApu>` out of a described machine.
/// See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, GbApu, GbAudio, SAMPLE_DIVISOR};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::gb::apu::CLASS;
    use crate::machine::{BuildOptions, Machine};

    /// Replace `gb.apu`'s constructor in `options` with one that keeps a handle
    /// and switches recording on.
    ///
    /// The one call a host makes between `catalog::build_options` and
    /// `machine::build`.
    ///
    /// **Switching recording on is the only thing this changes about the
    /// machine**, and it has to change it: `machines/gameboy.machine` defaults
    /// `record` to false, so a host that did not ask would get a chip that
    /// generates its samples and throws every one away. Nothing guest-visible
    /// depends on the flag — the ring is output rather than architectural state,
    /// it is absent from the snapshot, and no register reports its depth — which
    /// `host::audio::tests` asserts by hashing a machine with it on against the
    /// same machine with it off.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<GbApu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let apu = Arc::new(GbApu::from_props(props)?);
            apu.set_recording(true);
            seen.push(&apu);
            Ok(apu)
        });
        Ok(())
    }

    /// The APU this build constructed, as a [`GbAudio`] at the rate `machine`'s
    /// clock forest says it runs at.
    ///
    /// The most recent one, for a machine with more than one. `None` if this
    /// build has no Game Boy sound chip in it — a machine with no sound, which a
    /// host must be able to play silence for.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<GbAudio> {
        let seen = hosts
            .get::<Captured<GbApu>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?;
        let apu = seen.take()?;
        match resolve_rate(machine) {
            Some((num, den)) => Some(GbAudio::with_rate(apu, num, den)),
            None => Some(GbAudio::new(apu)),
        }
    }

    /// The chip's clock domain frequency divided by the sample divisor.
    ///
    /// The same resolution `display::lcd::capture` does for a frame rate and for
    /// the same reason: the rate is a fact about the oscillator forest, read
    /// from the forest rather than written twice in a machine file. A machine
    /// with several sound chips is matched by class, taking the last — which is
    /// the one [`take`] returned.
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
    fn a_dmgs_rate_is_exactly_32768_hertz() {
        // 4 194 304 / 128. The one console in the tree whose sample rate is an
        // integer, and the reason is that its crystal is a power of two.
        assert_eq!(SAMPLE_RATE, 32_768);
        assert_eq!(SAMPLE_DIVISOR, 128);
        let audio = GbAudio::new(Arc::new(GbApu::new()));
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), (32_768, 1));
        assert_eq!(info.channels, 2, "NR51 pans, so the seam is stereo");
        assert!(
            info.output_stage.is_empty(),
            "the DMG's analogue stage is not modelled, and this says so"
        );
    }

    #[test]
    fn a_board_with_another_crystal_gets_another_rate_in_lowest_terms() {
        let audio = GbAudio::with_rate(Arc::new(GbApu::new()), 8_388_608, 128);
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), (65_536, 1));
    }

    #[test]
    fn draining_a_silent_chip_appends_nothing() {
        let audio = GbAudio::new(Arc::new(GbApu::new()));
        let mut out = Vec::new();
        assert_eq!(audio.drain(&mut out), 0);
        assert!(out.is_empty());
        assert_eq!(audio.dropped(), 0, "the chip has no counter to report");
    }
}
