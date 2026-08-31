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
//! let source = audio::nes::capture::take();             // the Arc it kept
//! ```
//!
//! **This is a seam, and it is marked as one.** When `Device` grows an audio
//! hook beside `Device::region`, every line of [`capture`] deletes and nothing
//! else here changes. Until then the table is process-wide, so build one
//! machine at a time or [`capture::clear`] between them.

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
    use super::{Apu, Arc, NesAudio, Vec};
    use crate::core::error::Result;
    use crate::core::props::Props;
    use crate::core::sync::{AtomicU64, Global, LockRank, Ordering};
    use crate::dev::apu::{APU_CLASS, MAX_SAMPLE_BUFFER};
    use crate::machine::realize::Instance;
    use crate::machine::{Bindings, BuildOptions};

    /// Every APU constructed since the last [`take`] or [`clear`], oldest
    /// first. A `Vec` rather than a single slot because a machine with an
    /// expansion-audio chip alongside the console's own is not this module's
    /// business to refuse.
    static CONSTRUCTED: Global<Vec<Arc<Apu>>> = Global::with_rank(LockRank::LEAF, Vec::new());

    /// The smallest output ring the host will accept, in samples; `0` leaves
    /// whatever the machine description asked for alone.
    ///
    /// **Sizing the ring is a host concern**, and it is the *only* thing this
    /// interception changes about the machine. It has to be: how many samples
    /// may accumulate before somebody drains them is a fact about the front
    /// end's cadence — one video frame in a browser, a whole run under
    /// `--record-audio` — and a machine description cannot know it.
    ///
    /// Nothing guest-visible depends on it. The ring is output rather than
    /// architectural state, it is absent from the snapshot, and no register
    /// reports its depth, so a machine recorded and a machine ignored produce
    /// the same state hash. `host::audio::tests` asserts exactly that.
    static WANTED: AtomicU64 = AtomicU64::new(0);

    /// Construct an APU and keep a reference to it.
    ///
    /// An `InstanceCtor` is a bare `fn` that can capture nothing, which is why
    /// both the table above and the requested capacity are statics.
    fn construct(props: &Props) -> Result<Arc<dyn Instance>> {
        let wanted = WANTED.load(Ordering::Relaxed).min(MAX_SAMPLE_BUFFER);
        let asked = props
            .get("sample-buffer")
            .and_then(crate::core::props::Value::as_uint);
        let apu = match asked {
            // A machine that named a capacity and named a big enough one, or a
            // host that asked for nothing: build exactly what was written.
            _ if wanted == 0 => Arc::new(Apu::new(props)?),
            Some(have) if have >= wanted => Arc::new(Apu::new(props)?),
            _ => Arc::new(Apu::new(&props.clone().with("sample-buffer", wanted))?),
        };
        CONSTRUCTED.lock().push(Arc::clone(&apu));
        Ok(apu)
    }

    /// Replace `nes.apu`'s constructor in `bindings` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if a class turns out to be bound twice, which
    /// would be a bug in the caller's binding table rather than here.
    pub fn intercept(bindings: &Bindings) -> Result<Bindings> {
        let mut out = Bindings::new();
        let mut replaced = false;
        let classes: Vec<&'static str> = bindings.classes().collect();
        for class in classes {
            if class == APU_CLASS.name {
                out.bind(class, construct)?;
                replaced = true;
            } else if let Some(ctor) = bindings.get(class) {
                out.bind(class, ctor)?;
            }
        }
        if !replaced {
            // The APU's own `bind` was never called — a build with the device
            // feature but a machine that does not use it. Binding it here is
            // still correct: an unused binding costs nothing.
            out.bind(APU_CLASS.name, construct)?;
        }
        Ok(out)
    }

    /// Point `options` at intercepted bindings, in place, asking for an output
    /// ring of at least `capacity` samples.
    ///
    /// The one call a host makes between [`catalog::build_options`] and
    /// [`machine::build`].
    ///
    /// [`catalog::build_options`]: crate::machine::catalog::build_options
    /// [`machine::build`]: crate::machine::build
    ///
    /// # Errors
    ///
    /// As [`intercept`].
    pub fn install(options: &mut BuildOptions, capacity: u64) -> Result<()> {
        WANTED.store(capacity, Ordering::Relaxed);
        options.bindings = intercept(&options.bindings)?;
        Ok(())
    }

    /// Take the most recently constructed APU as a [`NesAudio`], forgetting
    /// every earlier one.
    ///
    /// `None` if no machine with an APU has been built since the last call — a
    /// machine with no sound, which a host must be able to play silence for.
    #[must_use]
    pub fn take() -> Option<NesAudio> {
        let mut table = CONSTRUCTED.lock();
        let last = table.pop();
        table.clear();
        last.map(NesAudio::new)
    }

    /// Forget every kept handle, so the next [`take`] cannot return an APU from
    /// a machine that has already been dropped.
    pub fn clear() {
        CONSTRUCTED.lock().clear();
    }
}
