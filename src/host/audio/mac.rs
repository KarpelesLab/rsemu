//! The Macintosh side of the audio seam: the pulse-width circuit's samples as
//! host frames.
//!
//! [`MacAudio`] holds an `Arc<Speaker>` and does the one thing the circuit
//! deliberately does not: it says at what *rate* the samples come out. The
//! scale is already the seam's — `mac.sound` converts a duty-cycle byte into a
//! signed 16-bit level with the volume bits applied, because both of those are
//! facts about the board rather than about a host.
//!
//! # The rate
//!
//! **One sample per scan line**, and a Macintosh Plus's scan line is 704 dot
//! clocks of its 15.6672 MHz crystal
//! ([`dev::mac::video`](crate::dev::mac::video)):
//!
//! ```text
//!   15 667 200 / 704 = 244 800 / 11 = 22 254.5454… Hz
//! ```
//!
//! which is not a whole number of hertz and is exactly what [`StreamInfo`]'s
//! rational is for. It is resolved from the machine's clock forest by
//! [`capture::take`], for the reason `audio::amiga` gives: the dot clock is a
//! fact about the board's crystal rather than about this file, and
//! `machines/mac-plus.machine` gives the circuit `clk / 704` so that one tick
//! of its clock is one sample. [`LINE_RATE`] is the fallback for a caller with
//! no machine to ask.
//!
//! # Mono, because the machine is
//!
//! One modulator, one speaker. The Plus has no stereo anything — that arrived
//! with the Macintosh II's Apple Sound Chip — so this is a one-channel source.
//!
//! # No analogue stage
//!
//! [`StreamInfo::output_stage`] is empty. The reconstruction filter between the
//! modulator and the speaker is on the board and the *Guide to the Macintosh
//! Family Hardware* gives no corner frequency for it; [`gb`](super::gb) set the
//! precedent that a corner nobody measured would be an invented measurement.
//!
//! # Getting hold of the circuit
//!
//! Exactly as [`audio::amiga::capture`](super::amiga::capture) gets hold of a
//! Paula, and for the same reason: `machine::build` hands back
//! `Arc<dyn Device>` and `Device` keeps `Any` out of its supertrait chain on
//! purpose, so the host takes its handle at the one moment the concrete type
//! exists — construction — by replacing the class's constructor.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, SampleFormat, StreamInfo, gcd};
use crate::dev::mac::sound::Speaker;

/// A Macintosh Plus's dot clock, in hertz: the board's one crystal.
pub const DOT_CLOCK: u64 = 15_667_200;

/// The horizontal line rate as an exact rational, in lowest terms:
/// 244 800 / 11 Hz, which is 22 254.5454… — not a whole number of hertz.
pub const LINE_RATE: (u64, u64) = (244_800, 11);

/// An [`AudioSource`] over a Macintosh's sound circuit.
#[derive(Debug)]
pub struct MacAudio {
    speaker: Arc<Speaker>,
    /// The exact sample rate, as a rational in lowest terms.
    rate: (u64, u64),
}

impl MacAudio {
    /// Listen to `speaker` at a Macintosh Plus's line rate.
    #[must_use]
    pub fn new(speaker: Arc<Speaker>) -> MacAudio {
        MacAudio::with_rate(speaker, LINE_RATE.0, LINE_RATE.1)
    }

    /// Listen to `speaker` at exactly `num / den` hertz.
    ///
    /// Reduced to lowest terms on the way in, because an unreduced rational
    /// costs the resampler `u64` range for nothing; a zero denominator is
    /// corrected to one, as [`StreamInfo::new`] corrects one.
    #[must_use]
    pub fn with_rate(speaker: Arc<Speaker>, num: u64, den: u64) -> MacAudio {
        let (num, den) = (num.max(1), den.max(1));
        let g = gcd(num, den);
        MacAudio {
            speaker,
            rate: (num / g, den / g),
        }
    }

    /// The circuit being listened to.
    #[must_use]
    pub fn speaker(&self) -> &Arc<Speaker> {
        &self.speaker
    }
}

impl AudioSource for MacAudio {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(self.rate.0, self.rate.1, 1, SampleFormat::S16)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        let samples = self.speaker.take_audio();
        out.extend_from_slice(&samples);
        samples.len() as u64
    }

    fn dropped(&self) -> u64 {
        self.speaker.dropped()
    }
}

/// The interception that gets a host an `Arc<Speaker>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, MacAudio};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::mac::sound::{CLASS_NAME, Sound, Speaker};
    use crate::machine::{BuildOptions, Machine};

    /// Replace `mac.sound`'s constructor in `options` with one that keeps a
    /// handle and switches recording on.
    ///
    /// The one call a host makes between `catalog::build_options` and
    /// `machine::build`.
    ///
    /// **Switching recording on is the only thing this changes about the
    /// machine**, and it has to change it: `machines/mac-plus.machine` leaves
    /// `record` at its default of false, so a host that did not ask would get a
    /// circuit that never fills its ring — and, here, one that names no
    /// scheduler event either. Nothing guest-visible depends on the flag: the
    /// ring is output rather than architectural state, it is absent from the
    /// snapshot, and the circuit has no registers for a guest to read it
    /// through.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Speaker>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let sound = Arc::new(Sound::new(props)?);
            sound.set_recording(true);
            seen.push(&sound.speaker());
            Ok(sound)
        });
        Ok(())
    }

    /// The sound circuit this build constructed, as a [`MacAudio`] at the rate
    /// `machine`'s clock forest says its clock runs at.
    ///
    /// The most recent one, for a machine with more than one. `None` if this
    /// build has no Macintosh sound circuit in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<MacAudio> {
        let seen = hosts
            .get::<Captured<Speaker>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        let speaker = seen.take()?;
        match resolve_rate(machine) {
            Some((num, den)) => Some(MacAudio::with_rate(speaker, num, den)),
            None => Some(MacAudio::new(speaker)),
        }
    }

    /// The circuit's own clock domain frequency, which *is* the sample rate.
    ///
    /// The same resolution `display::mac::capture` does for the dot clock and
    /// for the same reason: the rate is a fact about the oscillator forest,
    /// read from the forest rather than written twice. One tick of this
    /// device's clock is one sample, so there is no divisor to apply.
    fn resolve_rate(machine: &Machine) -> Option<(u64, u64)> {
        let entry = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)?;
        let domain = entry.domain()?;
        let freq = machine.clocks().domain_frequency(domain).ok()?;
        Some((freq.num(), freq.den()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::{Link, Props, Value};
    use crate::core::space::{RamStore, Region, RegionRef};
    use crate::core::value::Endian;
    use crate::dev::mac::sound::Sound;
    use crate::dev::mac::video::DOTS_PER_LINE;

    fn circuit() -> Sound {
        let sound = Sound::new(&Props::new().with("ram", Value::Link(Link::new("dram").unwrap())))
            .expect("`ram` is enough");
        let store = Arc::new(RamStore::new(0x10_0000));
        let ram: RegionRef = Arc::new(Region::ram("dram", store).with_endian(Endian::Big));
        sound.attach(&ram, 24).expect("it maps");
        sound
    }

    #[test]
    fn the_rate_is_the_line_rate_and_it_is_not_a_whole_hertz() {
        let audio = MacAudio::new(circuit().speaker());
        let info = audio.info();
        assert_eq!((info.rate_num, info.rate_den), LINE_RATE);
        assert_eq!(info.rate_hz(), 22_255, "22 254.5454… rounded");
        assert_eq!(info.channels, 1, "one modulator, one speaker");
        assert!(
            info.output_stage.is_empty(),
            "the board's filter is not modelled, and this says so"
        );
        // And that rational really is the dot clock over a scan line.
        assert_eq!(DOT_CLOCK * LINE_RATE.1, LINE_RATE.0 * DOTS_PER_LINE);
    }

    #[test]
    fn the_clock_forest_can_hand_over_an_unreduced_rational() {
        // What `resolve_rate` would produce for `clk / 704` before reduction.
        let audio = MacAudio::with_rate(circuit().speaker(), DOT_CLOCK, DOTS_PER_LINE);
        assert_eq!(audio.info().rate_num, LINE_RATE.0);
        assert_eq!(audio.info().rate_den, LINE_RATE.1);
    }

    #[test]
    fn draining_a_circuit_nobody_switched_on_appends_nothing() {
        let audio = MacAudio::new(circuit().speaker());
        let mut out = Vec::new();
        assert_eq!(audio.drain(&mut out), 0);
        assert!(out.is_empty());
        assert_eq!(audio.dropped(), 0);
    }

    #[test]
    fn a_recorded_circuit_hands_over_the_buffer_it_read() {
        let sound = circuit();
        sound.set_recording(true);
        sound.set_sndenb(false);
        sound.set_volume(7);
        let audio = MacAudio::new(sound.speaker());
        sound.advance_to(3);
        let mut out = Vec::new();
        assert_eq!(audio.drain(&mut out), 3);
        // A megabyte of fresh RAM is zero, which is the narrowest pulse.
        assert_eq!(out, [i16::MIN; 3]);
    }
}
