//! First-order RC filters: the host's half of a device's analogue output stage.
//!
//! A [`Pole`] is a fact a device declares — "there is a coupling
//! capacitor with a 90 Hz corner between my DAC and the speaker". This module
//! is the arithmetic that realises it, and it lives on the host side of the
//! seam for the same reason the palette does: the device says what the silicon
//! is, the host says what it sounds like.
//!
//! # The equations
//!
//! These are the textbook difference equations for a single-pole RC network,
//! obtained by taking `RC dv/dt + v = x` and replacing the derivative with a
//! backward difference. With `dt = 1 / f_s` and `RC = 1 / (2π f_c)`:
//!
//! ```text
//!   a = RC / (RC + dt) = 1 / (1 + 2π f_c / f_s)
//!
//!   high-pass:  y[n] = a · (y[n-1] + x[n] − x[n-1])
//!   low-pass:   y[n] = y[n-1] + (1 − a) · (x[n] − y[n-1])
//! ```
//!
//! Nothing here is specific to any machine, and nothing here is transcendental:
//! the coefficient is one division, so this module needs no `libm` and compiles
//! in a `no_std` build.
//!
//! # Why floats are allowed here and nowhere below
//!
//! This is `host/`. A coefficient is an amplitude, an amplitude is never a
//! duration, and no value computed here ever reaches the guest — see the
//! module docs of [`audio`](super) for the whole argument.

use alloc::vec::Vec;

use super::{Pole, PoleKind, StreamInfo};

/// τ = 2π, to the precision an `f64` holds. The one constant in the module.
const TAU: f64 = core::f64::consts::TAU;

/// One first-order section, with its state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnePole {
    kind: PoleKind,
    /// `RC / (RC + dt)`, precomputed.
    a: f32,
    prev_in: f32,
    prev_out: f32,
}

impl OnePole {
    /// A section realising `pole` for a stream running at exactly
    /// `rate_num / rate_den` hertz.
    ///
    /// A corner at or above half the sample rate, or a zero corner, produces a
    /// pass-through rather than an unstable filter: a device that declares a
    /// 14 kHz low-pass and is then sampled at 8 kHz should sound wrong, not
    /// explode.
    #[must_use]
    pub fn new(pole: Pole, rate_num: u64, rate_den: u64) -> OnePole {
        let rate = if rate_den == 0 {
            0.0
        } else {
            rate_num as f64 / rate_den as f64
        };
        let corner = f64::from(pole.corner_hz);
        let a = if rate <= 0.0 || corner <= 0.0 || corner * 2.0 >= rate {
            // A corner a one-pole section at this rate cannot realise. Both
            // forms below are the identity at a coefficient of 1: the high-pass
            // becomes `y = y' + x - x'`, which telescopes to `x`, and the
            // low-pass becomes `y += x - y`, which is `x`.
            1.0
        } else {
            let ratio = 1.0 / (1.0 + TAU * corner / rate);
            match pole.kind {
                // The low-pass wants `1 - RC/(RC+dt)` as its mixing
                // coefficient, and doing that subtraction here keeps `step` to
                // one multiply.
                PoleKind::LOW_PASS => 1.0 - ratio,
                _ => ratio,
            }
        };
        OnePole {
            kind: pole.kind,
            a: a as f32,
            prev_in: 0.0,
            prev_out: 0.0,
        }
    }

    /// Push one sample through, returning the filtered value.
    #[inline]
    #[must_use]
    pub fn step(&mut self, x: f32) -> f32 {
        match self.kind {
            PoleKind::LOW_PASS => {
                // `a` is already `1 - RC/(RC+dt)` here; see `new`.
                self.prev_out += self.a * (x - self.prev_out);
                self.prev_out
            }
            _ => {
                let y = self.a * (self.prev_out + x - self.prev_in);
                self.prev_in = x;
                self.prev_out = y;
                y
            }
        }
    }

    /// Forget the history, as a reset or a rate change does.
    pub const fn reset(&mut self) {
        self.prev_in = 0.0;
        self.prev_out = 0.0;
    }

    /// The precomputed coefficient, for a test that wants to check the
    /// arithmetic rather than the sound.
    #[inline]
    #[must_use]
    pub const fn coefficient(&self) -> f32 {
        self.a
    }
}

/// A device's whole analogue output stage: its poles, in declaration order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Chain {
    sections: Vec<OnePole>,
}

impl Chain {
    /// Build the chain `info` declares, at `info`'s own exact rate.
    ///
    /// The device rate, deliberately — an RC network is on the board, before
    /// anything resamples, so filtering after decimation would put the corners
    /// in the wrong place and alias everything above the new Nyquist into the
    /// band first.
    #[must_use]
    pub fn for_stream(info: StreamInfo) -> Chain {
        Chain {
            sections: info
                .output_stage
                .iter()
                .map(|pole| OnePole::new(*pole, info.rate_num, info.rate_den))
                .collect(),
        }
    }

    /// A chain that does nothing.
    #[must_use]
    pub const fn passthrough() -> Chain {
        Chain {
            sections: Vec::new(),
        }
    }

    /// How many sections it has.
    #[must_use]
    pub fn len(&self) -> usize {
        self.sections.len()
    }

    /// Whether it has none, in which case [`step`](Self::step) is the identity.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.sections.is_empty()
    }

    /// Push one sample through every section in order.
    #[inline]
    #[must_use]
    pub fn step(&mut self, x: f32) -> f32 {
        let mut y = x;
        for section in &mut self.sections {
            y = section.step(y);
        }
        y
    }

    /// Forget every section's history.
    pub fn reset(&mut self) {
        for section in &mut self.sections {
            section.reset();
        }
    }
}
