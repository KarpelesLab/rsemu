//! Rate conversion, with an exact integer phase.
//!
//! A sound chip runs at whatever its crystal gives it — an NTSC RP2A03 emits a
//! sample every APU cycle, which is 9 843 750 / 11 Hz — and a host sink wants
//! 44 100 or 48 000. Bridging those two is this module, and **it is the reason
//! the seam exists**: doing it inside the device would put a host's sample rate
//! into the emulated machine, and a machine whose behaviour depended on the
//! audio backend would not be deterministic (`ROADMAP.md` §0).
//!
//! # How the phase stays exact
//!
//! The device rate is a rational `in_num / in_den`; the host rate is an integer
//! `out`. One output frame is due every `in_num / (out · in_den)` input frames,
//! so the accumulator adds `step = out · in_den` per input frame and emits
//! whenever it reaches `period = in_num`. That is **integer arithmetic with no
//! drift at all** — after a million frames the phase is exactly where the
//! rational says it should be, which a floating-point `pos += 1.0 / ratio`
//! would not be. No float here is ever a time.
//!
//! # The filter
//!
//! Decimating 894 886 Hz to 48 000 by picking every 18.6th sample would fold
//! everything from 24 kHz to 447 kHz back into the audible band, which is a
//! wall of aliasing rather than a NES. So each output frame is the **mean of
//! every input frame that belongs to it** — a box filter, which is a moving
//! average and therefore a genuine low-pass, evaluated for free because the
//! accumulator is already running. It is not a windowed sinc and does not
//! pretend to be; combined with the console's own 14 kHz analogue roll-off
//! ([`filter`](super::filter)) it is clean enough that what comes out sounds
//! like the machine.
//!
//! Upsampling (a host rate above the device rate — rare, but a Game Boy channel
//! or a slow PSG can do it) holds the last value, which is the honest thing to
//! do with no samples to average.

use alloc::vec::Vec;

use super::filter::Chain;
use super::{AudioBuffer, StreamInfo};

/// One channel's accumulator and analogue stage.
#[derive(Debug, Clone)]
struct Channel {
    chain: Chain,
    /// Sum of the filtered input samples belonging to the frame being built.
    sum: f32,
    /// How many are in that sum.
    count: u32,
    /// The last value emitted, held when an output frame gets no input at all.
    held: f32,
}

/// A rational-rate box decimator with the device's analogue stage in front.
#[derive(Debug, Clone)]
pub struct Resampler {
    channels: Vec<Channel>,
    /// Phase added per input frame: `out_rate × in_den`.
    step: u64,
    /// Phase one output frame costs: `in_num`.
    period: u64,
    phase: u64,
    /// Scratch for one output frame, so `process` allocates nothing.
    frame: Vec<f32>,
}

impl Resampler {
    /// Convert a stream shaped like `info` to `out_rate` hertz.
    #[must_use]
    pub fn new(info: StreamInfo, out_rate: u32) -> Resampler {
        let channels = usize::from(info.channels);
        let chain = Chain::for_stream(info);
        Resampler {
            channels: (0..channels)
                .map(|_| Channel {
                    chain: chain.clone(),
                    sum: 0.0,
                    count: 0,
                    held: 0.0,
                })
                .collect(),
            step: u64::from(out_rate.max(1)).saturating_mul(info.rate_den.max(1)),
            period: info.rate_num.max(1),
            // Zero, so the first output frame is the mean of a *full* window
            // rather than of however many samples happened to arrive first.
            phase: 0,
            frame: alloc::vec![0.0; channels],
        }
    }

    /// How many channels it converts.
    #[must_use]
    pub fn channels(&self) -> usize {
        self.channels.len()
    }

    /// Input frames per output frame, as the exact pair `(period, step)`.
    ///
    /// Exposed for tests and diagnostics: nothing needs to divide these.
    #[must_use]
    pub const fn ratio(&self) -> (u64, u64) {
        (self.period, self.step)
    }

    /// Forget every accumulator and filter, as a rate change does.
    pub fn reset(&mut self) {
        self.phase = 0;
        for channel in &mut self.channels {
            channel.chain.reset();
            channel.sum = 0.0;
            channel.count = 0;
            channel.held = 0.0;
        }
    }

    /// Convert `input` — interleaved signed 16-bit at the device rate — and
    /// append the result to `out`.
    ///
    /// A partial trailing frame in `input` is ignored rather than padded: a
    /// device adapter always hands over whole frames, and inventing a channel's
    /// worth of silence to complete one would be a click.
    pub fn process(&mut self, input: &[i16], out: &mut AudioBuffer) {
        let channels = self.channels.len();
        if channels == 0 || input.is_empty() {
            return;
        }
        for chunk in input.chunks_exact(channels) {
            for (channel, sample) in self.channels.iter_mut().zip(chunk) {
                let x = f32::from(*sample) / 32768.0;
                channel.sum += channel.chain.step(x);
                channel.count += 1;
            }
            self.phase += self.step;
            while self.phase >= self.period {
                self.phase -= self.period;
                for (slot, channel) in self.frame.iter_mut().zip(self.channels.iter_mut()) {
                    if channel.count > 0 {
                        channel.held = channel.sum / channel.count as f32;
                        channel.sum = 0.0;
                        channel.count = 0;
                    }
                    *slot = channel.held;
                }
                out.push_normalised(&self.frame);
            }
        }
    }
}
