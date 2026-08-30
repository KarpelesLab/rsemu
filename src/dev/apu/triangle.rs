//! The triangle channel — `$4008`–`$400B`.
//!
//! Source: [NESdev APU Triangle](https://www.nesdev.org/wiki/APU_Triangle).

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

use super::units::LengthCounter;

/// The 32-step sequence the triangle sends to the mixer.
///
/// A hardware fact: 15 down to 0, then 0 up to 15.
const SEQUENCE: [u8; 32] = [
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, //
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
];

/// The triangle channel: timer, length counter, linear counter, sequencer.
///
/// It has no volume control — the waveform is either cycling or suspended — and
/// unlike the pulse and noise channels its timer is clocked on *every* CPU
/// cycle rather than every APU cycle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Triangle {
    /// The length counter.
    pub length: LengthCounter,
    /// The linear counter's current value.
    linear: u8,
    /// `$4008` bits 6-0: what the linear counter reloads to.
    linear_reload_value: u8,
    /// Set by a `$400B` write, consumed by a quarter-frame clock.
    linear_reload: bool,
    /// `$4008` bit 7, which is also the length counter's halt flag.
    control: bool,
    /// The 11-bit raw timer period `t`.
    period: u16,
    /// The timer's down counter, in CPU cycles.
    timer: u16,
    /// Position in the 32-step sequence.
    step: u8,
    /// When set, the sequencer is not clocked while `t < 2`.
    ///
    /// [NESdev APU Triangle](https://www.nesdev.org/wiki/APU_Triangle) suggests
    /// halting the channel at ultrasonic frequencies "at the expense of
    /// accuracy" to suppress the popping some games (Mega Man 2) produce by
    /// silencing the triangle with a zero period. Off by default, because this
    /// project measures accuracy rather than pleasantness.
    halt_ultrasonic: bool,
}

impl Triangle {
    /// A powered-on triangle channel.
    pub const fn new(halt_ultrasonic: bool) -> Triangle {
        Triangle {
            length: LengthCounter::new(),
            linear: 0,
            linear_reload_value: 0,
            linear_reload: false,
            control: false,
            period: 0,
            timer: 0,
            step: 0,
            halt_ultrasonic,
        }
    }

    /// The linear counter's current value.
    #[inline]
    pub const fn linear(&self) -> u8 {
        self.linear
    }

    /// The raw 11-bit timer period.
    #[inline]
    pub const fn period(&self) -> u16 {
        self.period
    }

    /// `$4008`: control flag (also the length halt flag) and reload value.
    pub fn write_linear(&mut self, value: u8) {
        self.control = value & 0x80 != 0;
        self.length.set_halt(self.control);
        self.linear_reload_value = value & 0x7F;
    }

    /// `$400A`: the low 8 bits of the timer period.
    pub fn write_period_low(&mut self, value: u8) {
        self.period = (self.period & 0x0700) | u16::from(value);
    }

    /// `$400B`: length counter load, timer high bits, and the reload flag.
    pub fn write_period_high(&mut self, value: u8) {
        self.period = (self.period & 0x00FF) | (u16::from(value & 0x07) << 8);
        self.length.load(value);
        self.linear_reload = true;
    }

    /// One quarter-frame clock of the linear counter.
    ///
    /// The reload flag is cleared only when the control flag is clear, so a
    /// `$4008` write with both set is reloaded again at every clock.
    pub fn clock_linear(&mut self) {
        if self.linear_reload {
            self.linear = self.linear_reload_value;
        } else if self.linear > 0 {
            self.linear -= 1;
        }
        if !self.control {
            self.linear_reload = false;
        }
    }

    /// Clock the timer. Called once per CPU cycle.
    ///
    /// The timer runs unconditionally; the sequencer only advances while both
    /// counters are non-zero, which is why silencing the triangle freezes it at
    /// its current output level rather than snapping it to zero.
    pub fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.period;
            if self.length.active() && self.linear > 0 && !self.ultrasonic() {
                self.step = (self.step + 1) & 31;
            }
        } else {
            self.timer -= 1;
        }
    }

    /// Whether the optional ultrasonic halt is currently engaged.
    #[inline]
    const fn ultrasonic(&self) -> bool {
        self.halt_ultrasonic && self.period < 2
    }

    /// The 4-bit level this channel sends to the mixer.
    pub fn output(&self) -> u8 {
        SEQUENCE[usize::from(self.step)]
    }

    /// Reset the sequencer phase, as a console reset does.
    ///
    /// [NESdev CPU power up
    /// state](https://www.nesdev.org/wiki/CPU_power_up_state): the triangle's
    /// phase is reset to 0 — output 15 — by a reset, though it is unspecified
    /// at power-up.
    pub fn reset_phase(&mut self) {
        self.step = 0;
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        self.length.save(w)?;
        w.write_u8(self.linear)?;
        w.write_u8(self.linear_reload_value)?;
        w.write_bool(self.linear_reload)?;
        w.write_bool(self.control)?;
        w.write_u16(self.period)?;
        w.write_u16(self.timer)?;
        w.write_u8(self.step)
    }

    /// Restore what [`Triangle::save`] wrote.
    ///
    /// `halt_ultrasonic` is configuration, not state, so it is not stored.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.length.load_state(r)?;
        self.linear = r.read_u8()?;
        self.linear_reload_value = r.read_u8()? & 0x7F;
        self.linear_reload = r.read_bool()?;
        self.control = r.read_bool()?;
        self.period = r.read_u16()? & 0x07FF;
        self.timer = r.read_u16()?;
        self.step = r.read_u8()? & 31;
        Ok(())
    }
}
