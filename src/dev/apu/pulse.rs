//! The two pulse channels and their sweep units — `$4000`–`$4007`.
//!
//! Sources: [NESdev APU Pulse](https://www.nesdev.org/wiki/APU_Pulse) and
//! [NESdev APU Sweep](https://www.nesdev.org/wiki/APU_Sweep).

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

use super::units::{Envelope, LengthCounter};

/// The four duty cycles, as the waveform the sequencer emits.
///
/// The hardware sequencer counts *downward* and so reads its lookup table in
/// the order 0, 7, 6, …, 1; the table below is the resulting output waveform,
/// which is what the wiki's "Output waveform" column shows and what an
/// upward-counting index reproduces exactly.
const DUTY: [[u8; 8]; 4] = [
    [0, 1, 0, 0, 0, 0, 0, 0], // 12.5%
    [0, 1, 1, 0, 0, 0, 0, 0], // 25%
    [0, 1, 1, 1, 1, 0, 0, 0], // 50%
    [1, 0, 0, 1, 1, 1, 1, 1], // 25% negated
];

/// A pulse channel's sweep unit: a divider plus a reload flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sweep {
    enabled: bool,
    negate: bool,
    shift: u8,
    /// `P` from `$4001`/`$4005`; the divider's period is `P + 1` half frames.
    period: u8,
    divider: u8,
    reload: bool,
    /// True for pulse 1, whose adder has no carry input and therefore adds the
    /// ones' complement (`-c - 1`) where pulse 2 adds the two's complement
    /// (`-c`). This one bit is the *only* difference between the channels.
    ones_complement: bool,
}

impl Sweep {
    /// A powered-on sweep unit for the channel identified by `ones_complement`.
    pub const fn new(ones_complement: bool) -> Sweep {
        Sweep {
            enabled: false,
            negate: false,
            shift: 0,
            period: 0,
            divider: 0,
            reload: false,
            ones_complement,
        }
    }

    /// Apply an `EPPP NSSS` write to `$4001`/`$4005`.
    pub fn write(&mut self, value: u8) {
        self.enabled = value & 0x80 != 0;
        self.period = (value >> 4) & 0x07;
        self.negate = value & 0x08 != 0;
        self.shift = value & 0x07;
        self.reload = true;
    }

    /// The target period for the given current period.
    ///
    /// The change amount is the period shifted right by the shift count,
    /// negated when the negate flag is set — as the ones' complement on pulse 1
    /// and the two's complement on pulse 2 — and the sum is clamped at zero.
    /// The result deliberately is *not* clamped at `$7FF`: exceeding it is the
    /// muting condition, so the caller has to be able to see it.
    pub fn target(&self, period: u16) -> u16 {
        let change = i32::from(period >> self.shift);
        let sum = if self.negate {
            i32::from(period) - change - i32::from(self.ones_complement)
        } else {
            i32::from(period) + change
        };
        // The adder is 11 bits plus carry, so the sum cannot exceed 0xFFE and
        // the cast is lossless; the clamp is the documented behaviour, not a
        // defensive measure.
        sum.max(0) as u16
    }

    /// Whether the sweep unit is silencing the channel.
    ///
    /// Both conditions hold regardless of whether the unit is enabled and
    /// regardless of whether the divider is producing a clock, which is why a
    /// program that wants the sweep truly out of the way must set the negate
    /// flag (write `$08`).
    pub fn muting(&self, period: u16) -> bool {
        period < 8 || self.target(period) > 0x7FF
    }

    /// One half-frame clock, possibly updating `period` in place.
    pub fn clock(&mut self, period: &mut u16) {
        if self.divider == 0 && self.enabled && self.shift != 0 && !self.muting(*period) {
            *period = self.target(*period);
        }
        if self.divider == 0 || self.reload {
            self.divider = self.period;
            self.reload = false;
        } else {
            self.divider -= 1;
        }
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        w.write_bool(self.enabled)?;
        w.write_bool(self.negate)?;
        w.write_u8(self.shift)?;
        w.write_u8(self.period)?;
        w.write_u8(self.divider)?;
        w.write_bool(self.reload)
    }

    /// Restore what [`Sweep::save`] wrote.
    ///
    /// `ones_complement` is channel identity, not state, so it is not stored.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.enabled = r.read_bool()?;
        self.negate = r.read_bool()?;
        self.shift = r.read_u8()?;
        self.period = r.read_u8()?;
        self.divider = r.read_u8()?;
        self.reload = r.read_bool()?;
        Ok(())
    }
}

/// One pulse channel: envelope, sweep, timer, 8-step sequencer, length counter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pulse {
    /// The envelope generator, which also owns the loop/halt flag.
    pub envelope: Envelope,
    /// The sweep unit.
    pub sweep: Sweep,
    /// The length counter.
    pub length: LengthCounter,
    duty: u8,
    /// The 11-bit raw timer period `t`.
    period: u16,
    /// The timer's down counter, in APU cycles.
    timer: u16,
    /// Position in the 8-step duty sequence.
    step: u8,
}

impl Pulse {
    /// A powered-on pulse channel. `first` selects pulse 1's sweep negation.
    pub const fn new(first: bool) -> Pulse {
        Pulse {
            envelope: Envelope::new(),
            sweep: Sweep::new(first),
            length: LengthCounter::new(),
            duty: 0,
            period: 0,
            timer: 0,
            step: 0,
        }
    }

    /// The raw 11-bit timer period.
    #[inline]
    pub const fn period(&self) -> u16 {
        self.period
    }

    /// `$4000`/`$4004`: duty, length halt, constant volume flag, volume.
    pub fn write_control(&mut self, value: u8) {
        self.duty = value >> 6;
        self.envelope.write_control(value);
        self.length.set_halt(self.envelope.loop_flag());
    }

    /// `$4001`/`$4005`: sweep setup.
    pub fn write_sweep(&mut self, value: u8) {
        self.sweep.write(value);
    }

    /// `$4002`/`$4006`: the low 8 bits of the timer period.
    pub fn write_period_low(&mut self, value: u8) {
        self.period = (self.period & 0x0700) | u16::from(value);
    }

    /// `$4003`/`$4007`: length counter load and the high 3 bits of the period.
    ///
    /// Side effects, per the wiki: the sequencer restarts at the first value of
    /// the current duty and the envelope restarts. The timer's own divider is
    /// **not** reset.
    pub fn write_period_high(&mut self, value: u8) {
        self.period = (self.period & 0x00FF) | (u16::from(value & 0x07) << 8);
        self.length.load(value);
        self.step = 0;
        self.envelope.restart();
    }

    /// Clock the timer. Called once per APU cycle (every second CPU cycle).
    pub fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.period;
            self.step = (self.step + 1) & 7;
        } else {
            self.timer -= 1;
        }
    }

    /// The 4-bit level this channel sends to the mixer.
    ///
    /// Zero when the sequencer output is low, when the sweep unit is muting
    /// (which subsumes the `t < 8` case), or when the length counter is zero.
    pub fn output(&self) -> u8 {
        if !self.length.active() || self.sweep.muting(self.period) {
            return 0;
        }
        if DUTY[usize::from(self.duty)][usize::from(self.step)] == 0 {
            return 0;
        }
        self.envelope.volume()
    }

    /// One half-frame clock: the sweep unit.
    pub fn clock_sweep(&mut self) {
        self.sweep.clock(&mut self.period);
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        self.envelope.save(w)?;
        self.sweep.save(w)?;
        self.length.save(w)?;
        w.write_u8(self.duty)?;
        w.write_u16(self.period)?;
        w.write_u16(self.timer)?;
        w.write_u8(self.step)
    }

    /// Restore what [`Pulse::save`] wrote.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.envelope.load(r)?;
        self.sweep.load(r)?;
        self.length.load_state(r)?;
        self.duty = r.read_u8()? & 3;
        self.period = r.read_u16()? & 0x07FF;
        self.timer = r.read_u16()?;
        self.step = r.read_u8()? & 7;
        Ok(())
    }
}
