//! The non-linear mixer and the audio output surface.
//!
//! Source: [NESdev APU Mixer](https://www.nesdev.org/wiki/APU_Mixer).
//!
//! # No floats, anywhere
//!
//! The wiki gives the mixer as two rational expressions over the channel
//! levels:
//!
//! ```text
//! pulse_out = 95.52  / (8128  / (pulse1 + pulse2) + 100)
//! tnd_out   = 163.67 / (24329 / (3*triangle + 2*noise + dmc) + 100)
//! ```
//!
//! Multiplying numerator and denominator through by the index turns each into a
//! plain ratio of integers — `95.52n / (8128 + 100n)`, and likewise
//! `163.67n / (24329 + 100n)` — so the two lookup tables can be evaluated with
//! `u64` arithmetic in a `const fn` at compile time. Nothing here, and nothing
//! anywhere near the frame counter, is a float.
//!
//! Levels are unsigned Q16: `65536` would be full scale. The loudest reachable
//! combination (both pulses at 15, triangle 15, noise 15, DMC 127) sums to
//! 65534, so a sample always fits a `u16`; a `debug_assert` in
//! [`mix`] holds that claim.

use alloc::vec::Vec;

/// Fixed-point scale: one unit of output is `1 / 65536`.
pub const SCALE: u32 = 1 << 16;

/// `95.52 * n / (8128 + 100 * n)`, in Q16, truncated toward zero.
const fn pulse_level(n: u64) -> u16 {
    if n == 0 {
        return 0;
    }
    // 9552 and the trailing /100 keep the two decimal places of 95.52 exact.
    let num = 9552 * n * SCALE as u64;
    let den = 100 * (8128 + 100 * n);
    (num / den) as u16
}

/// `163.67 * n / (24329 + 100 * n)`, in Q16, truncated toward zero.
const fn tnd_level(n: u64) -> u16 {
    if n == 0 {
        return 0;
    }
    let num = 16367 * n * SCALE as u64;
    let den = 100 * (24329 + 100 * n);
    (num / den) as u16
}

/// Build the pulse table at compile time.
const fn build_pulse_table() -> [u16; 31] {
    let mut t = [0u16; 31];
    let mut i = 0;
    while i < 31 {
        t[i] = pulse_level(i as u64);
        i += 1;
    }
    t
}

/// Build the triangle/noise/DMC table at compile time.
const fn build_tnd_table() -> [u16; 203] {
    let mut t = [0u16; 203];
    let mut i = 0;
    while i < 203 {
        t[i] = tnd_level(i as u64);
        i += 1;
    }
    t
}

/// `pulse1 + pulse2`, 0..=30, to a Q16 level.
pub static PULSE_TABLE: [u16; 31] = build_pulse_table();

/// `3 * triangle + 2 * noise + dmc`, 0..=202, to a Q16 level.
pub static TND_TABLE: [u16; 203] = build_tnd_table();

/// Mix one set of channel levels into a Q16 sample.
///
/// `pulse1`, `pulse2`, `triangle` and `noise` are 0..=15; `dmc` is 0..=127.
/// Out-of-range inputs are clamped rather than panicking, because the mixer is
/// on the hot path and a channel bug should not take the machine down.
pub fn mix(pulse1: u8, pulse2: u8, triangle: u8, noise: u8, dmc: u8) -> u16 {
    let p = usize::from(pulse1.min(15) + pulse2.min(15));
    let t = usize::from(triangle.min(15)) * 3 + usize::from(noise.min(15)) * 2 + usize::from(dmc);
    let t = t.min(TND_TABLE.len() - 1);
    let sum = u32::from(PULSE_TABLE[p]) + u32::from(TND_TABLE[t]);
    debug_assert!(sum <= u32::from(u16::MAX), "mixer output overflowed Q16");
    sum.min(u32::from(u16::MAX)) as u16
}

/// A bounded ring of Q16 samples produced at the APU's own rate.
///
/// One sample per APU cycle — 894 886 Hz on NTSC, exactly half the CPU clock.
/// Resampling to a host rate is the host layer's job (`ROADMAP.md` §15,
/// invariant 4): nothing here reads a wall clock or interpolates.
///
/// The ring is *output*, not architectural state, so it is deliberately absent
/// from the snapshot (`ROADMAP.md` §4.5). A machine that restores a save state
/// resumes producing samples; it does not replay the ones already handed out.
#[derive(Debug, Default)]
pub struct SampleRing {
    buf: Vec<u16>,
    head: usize,
    len: usize,
    dropped: u64,
}

impl SampleRing {
    /// A ring holding `capacity` samples. A capacity of zero disables output.
    pub fn with_capacity(capacity: usize) -> SampleRing {
        SampleRing {
            buf: alloc::vec![0u16; capacity],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// How many samples the ring can hold.
    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    /// How many samples are waiting.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether nothing is waiting.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// How many samples have been overwritten because nobody drained them.
    ///
    /// A diagnostic for a host that is not keeping up, not architectural state.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Append one sample, overwriting the oldest if the ring is full.
    pub fn push(&mut self, sample: u16) {
        let cap = self.buf.len();
        if cap == 0 {
            return;
        }
        if self.len == cap {
            self.buf[self.head] = sample;
            self.head = (self.head + 1) % cap;
            self.dropped += 1;
        } else {
            let index = (self.head + self.len) % cap;
            self.buf[index] = sample;
            self.len += 1;
        }
    }

    /// Move every waiting sample into `out`, oldest first.
    pub fn drain_into(&mut self, out: &mut Vec<u16>) {
        let cap = self.buf.len();
        if cap == 0 {
            return;
        }
        out.reserve(self.len);
        for i in 0..self.len {
            out.push(self.buf[(self.head + i) % cap]);
        }
        self.head = 0;
        self.len = 0;
    }

    /// Discard everything waiting, as a reset does.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.dropped = 0;
    }
}
