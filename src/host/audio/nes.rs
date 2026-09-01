//! The NES side of the audio seam: an RP2A03's samples as host frames.
//!
//! [`NesAudio`] holds an `Arc<Apu>` and does the two things the APU
//! deliberately refuses to do — centre its unsigned Q16 mixer output into a
//! signed sample, and declare the console's analogue output network so the host
//! can filter with it.
//!
//! # The rate
//!
//! One sample per APU cycle, and an APU cycle is two CPU cycles, so the rate is
//! the board's master crystal divided by twice the CPU divider
//! ([NESdev, "Cycle reference chart"](https://www.nesdev.org/wiki/Cycle_reference_chart)):
//!
//! | Region | Master | ÷ | Sample rate |
//! | --- | --- | --- | --- |
//! | NTSC (RP2A03) | 236 250 000 / 11 Hz | 24 | 9 843 750 / 11 Hz ≈ 894 886.36 |
//! | PAL (RP2A07) | 53 203 425 / 2 Hz | 32 | 53 203 425 / 64 Hz = 831 303.52 |
//! | Dendy (UA6527P) | 53 203 425 / 2 Hz | 30 | 3 546 895 / 4 Hz = 886 723.75 |
//!
//! None of those is an integer, which is exactly why [`StreamInfo`] carries a
//! rational (`CLAUDE.md`, determinism): rounding an NTSC NES to 894 886 Hz
//! would be a real, if inaudible, pitch error, and it would throw away the
//! exactness the oscillator forest exists to preserve.
//!
//! # The analogue stage
//!
//! A NES does not put its DAC output on the RF modulator directly. Per
//! [NESdev, "APU Mixer"](https://www.nesdev.org/wiki/APU_Mixer), the console's
//! output passes through a first-order high-pass at 90 Hz, a second at 440 Hz
//! and a first-order low-pass at 14 kHz. The Famicom's network is different
//! again, which is precisely why this is *declared* rather than baked into the
//! mixer: it is a property of the board, not of the chip.
//!
//! Without the high-passes the stream would carry the mixer's very large DC
//! offset — the 2A03's output is unsigned and idles well above zero — and a
//! sound card would reproduce it as a thump and several dB of wasted headroom.
//!
//! # Getting hold of the APU
//!
//! Exactly as [`display::nes::capture`](crate::host::display::nes::capture)
//! gets hold of the PPU, and for exactly the same reason: a machine built from
//! a description hands back `Arc<dyn Device>` and there is no route from there
//! to `Arc<Apu>`. So the host takes its handle at the one moment the concrete
//! type exists — device construction — by replacing the class's constructor.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! audio::nes::capture::install(&mut options, 65_536)?;  // intercept nes.apu
//! let machine = machine::build(name, source, &registry, &options)?;
//! let source = audio::nes::capture::take(&options.realize.hosts);
//! ```
//!
//! **This is a seam, and it is marked as one.** When `Device` grows an audio
//! hook beside `Device::region`, every line of [`capture`] deletes and nothing
//! else here changes. The capture table belongs to the build rather than to the
//! process, so two consoles built in one process do not swap chips.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{AudioSource, Pole, SampleFormat, StreamInfo};
use crate::core::sync::{LockRank, Mutex};
use crate::dev::apu::Apu;

/// The console's analogue output network, as NESdev's "APU Mixer" describes it.
///
/// Order matters only in that a chain of first-order sections is applied in
/// sequence; these are written down in the order the wiki lists them.
static NES_OUTPUT_STAGE: &[Pole] = &[
    Pole::high_pass(90),
    Pole::high_pass(440),
    Pole::low_pass(14_000),
];

/// An [`AudioSource`] over a NES APU.
#[derive(Debug)]
pub struct NesAudio {
    apu: Arc<Apu>,
    /// The `u16` buffer [`Apu::take_samples`] fills, kept between calls so the
    /// pull path allocates nothing after it has warmed up.
    ///
    /// [`LockRank::LEAF`], and — importantly — never held across the call into
    /// the APU, which takes [`LockRank::DEVICE`]: a leaf lock held while an
    /// outer one is acquired is precisely the inversion `core::sync` checks
    /// for. [`NesAudio::drain`] moves the `Vec` out and puts it back.
    scratch: Mutex<Vec<u16>>,
}

impl NesAudio {
    /// Listen to `apu`.
    #[must_use]
    pub fn new(apu: Arc<Apu>) -> NesAudio {
        NesAudio {
            apu,
            scratch: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The chip being listened to, for a host that wants its registers too.
    #[must_use]
    pub fn apu(&self) -> &Arc<Apu> {
        &self.apu
    }
}

impl AudioSource for NesAudio {
    fn info(&self) -> StreamInfo {
        let region = self.apu.tv_region();
        let (num, den) = region.master_clock();
        // Two CPU cycles to an APU cycle, one sample per APU cycle.
        let den = den * region.cpu_divider() * 2;
        let g = gcd(num, den);
        StreamInfo::new(num / g, den / g, 1, SampleFormat::S16).with_output_stage(NES_OUTPUT_STAGE)
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        // Take the buffer out from under its lock before touching the APU; see
        // the field's own comment for why that is not merely tidy.
        let mut raw = core::mem::take(&mut *self.scratch.lock());
        raw.clear();
        self.apu.take_samples(&mut raw);
        let taken = raw.len() as u64;
        out.reserve(raw.len());
        for sample in &raw {
            // The mixer's output is unsigned Q16 with silence at zero and full
            // scale at 65 535 (`dev::apu::mixer`), so subtracting half scale
            // centres it. The result is a signed sample with a large standing
            // offset, which the 90 Hz high-pass then removes — that offset is
            // real, and it is what the console's coupling capacitor is for.
            out.push((i32::from(*sample) - 32_768) as i16);
        }
        *self.scratch.lock() = raw;
        taken
    }

    fn dropped(&self) -> u64 {
        self.apu.samples_dropped()
    }
}

/// Greatest common divisor, so the rate is reported in lowest terms.
///
/// Reducing is not cosmetic: [`resample`](super::resample) multiplies the
/// denominator by the host rate, and an unreduced 236 250 000 / 264 would push
/// that product 24 times further up the `u64` range for nothing.
const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    if a == 0 { 1 } else { a }
}

/// The interception that gets a host an `Arc<Apu>` out of a described machine.
/// See the module docs: a seam, not a design.
pub mod capture {
    use super::{Apu, Arc, NesAudio};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::apu::{APU_CLASS, MAX_SAMPLE_BUFFER};
    use crate::machine::BuildOptions;

    /// Replace `nes.apu`'s constructor in `options` with one that keeps a
    /// handle and asks for an output ring of at least `capacity` samples.
    ///
    /// The one call a host makes between [`catalog::build_options`] and
    /// [`machine::build`].
    ///
    /// **Sizing the ring is a host concern**, and it is the *only* thing this
    /// interception changes about the machine. It has to be: how many samples
    /// may accumulate before somebody drains them is a fact about the front
    /// end's cadence — one video frame in a browser, a whole run under
    /// `--record-audio` — and a machine description cannot know it. A
    /// `capacity` of `0` leaves whatever the description asked for alone.
    ///
    /// Nothing guest-visible depends on it. The ring is output rather than
    /// architectural state, it is absent from the snapshot, and no register
    /// reports its depth, so a machine recorded and a machine ignored produce
    /// the same state hash. `host::audio::tests` asserts exactly that.
    ///
    /// The capacity is a *captured* value rather than a static, which is the
    /// whole reason [`InstanceCtor`](crate::machine::realize::InstanceCtor) is a
    /// closure: two builds may want two different rings, and a `static` gave
    /// them one.
    ///
    /// [`catalog::build_options`]: crate::machine::catalog::build_options
    /// [`machine::build`]: crate::machine::build
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions, capacity: u64) -> Result<()> {
        let seen: Arc<Captured<Apu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, APU_CLASS.name, Captured::new)?;
        let wanted = capacity.min(MAX_SAMPLE_BUFFER);
        options.bindings.replace(APU_CLASS.name, move |props| {
            let asked = props
                .get("sample-buffer")
                .and_then(crate::core::props::Value::as_uint);
            let apu = match asked {
                // A machine that named a capacity and named a big enough one, or
                // a host that asked for nothing: build exactly what was written.
                _ if wanted == 0 => Arc::new(Apu::new(props)?),
                Some(have) if have >= wanted => Arc::new(Apu::new(props)?),
                _ => Arc::new(Apu::new(&props.clone().with("sample-buffer", wanted))?),
            };
            seen.push(&apu);
            Ok(apu)
        });
        Ok(())
    }

    /// The APU this build constructed, as a [`NesAudio`].
    ///
    /// The most recent one, for a machine with an expansion-audio chip alongside
    /// the console's own. `None` if this build has no APU in it — a machine with
    /// no sound, which a host must be able to play silence for.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<NesAudio> {
        let seen = hosts
            .get::<Captured<Apu>>(HostKind::CAPTURE, APU_CLASS.name)
            .ok()
            .flatten()?;
        seen.take().map(NesAudio::new)
    }
}
