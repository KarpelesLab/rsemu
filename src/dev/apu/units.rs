//! The building blocks shared by more than one channel: the envelope
//! generator, the length counter and its lookup table.
//!
//! Sources: [NESdev APU Envelope](https://www.nesdev.org/wiki/APU_Envelope) and
//! [NESdev APU Length Counter](https://www.nesdev.org/wiki/APU_Length_Counter).

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

/// The length counter lookup table, indexed by the top five bits of `$4003`,
/// `$4007`, `$400B` or `$400F`.
///
/// Straight from the table on [NESdev APU Length
/// Counter](https://www.nesdev.org/wiki/APU_Length_Counter). These are hardware
/// facts measured off the 2A03's internal mask ROM, and the page also explains
/// their structure: with bit 0 of the index set the remaining bits select a
/// linear length, otherwise the value is a note length based on 10 (index bit 4
/// clear) or 12 (set).
pub const LENGTH_TABLE: [u8; 32] = [
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, //
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
];

/// The volume envelope generator of a pulse or noise channel.
///
/// Contains a start flag, a divider and a decay level counter, exactly as
/// [NESdev APU Envelope](https://www.nesdev.org/wiki/APU_Envelope) describes.
/// It is clocked by the frame counter's quarter-frame signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Envelope {
    /// Set by a write to the channel's length-counter register; the next
    /// quarter-frame clock consumes it.
    start: bool,
    /// The divider's counter. Its reload value is [`Envelope::period`].
    divider: u8,
    /// The decay level counter, 15 down to 0.
    decay: u8,
    /// `V` from `$4000`/`$4004`/`$400C`: both the constant volume and the
    /// divider's reload value (the period is `V + 1` quarter frames).
    period: u8,
    /// The envelope loop flag, which shares a register bit with the length
    /// counter halt flag.
    loop_flag: bool,
    /// When set, `V` is the volume directly and the decay level is ignored —
    /// but still updated.
    constant: bool,
}

impl Envelope {
    /// A powered-on envelope: everything zero.
    pub const fn new() -> Envelope {
        Envelope {
            start: false,
            divider: 0,
            decay: 0,
            period: 0,
            loop_flag: false,
            constant: false,
        }
    }

    /// Apply the `--LC VVVV` half of a `$4000`/`$4004`/`$400C` write.
    pub fn write_control(&mut self, value: u8) {
        self.loop_flag = value & 0x20 != 0;
        self.constant = value & 0x10 != 0;
        self.period = value & 0x0F;
    }

    /// Whether the loop flag is set. It doubles as the length counter's halt
    /// flag, which is why the caller needs to see it.
    #[inline]
    pub const fn loop_flag(&self) -> bool {
        self.loop_flag
    }

    /// Set the start flag, as a write to the length-counter register does.
    pub fn restart(&mut self) {
        self.start = true;
    }

    /// One quarter-frame clock.
    ///
    /// If the start flag is clear the divider is clocked; otherwise the start
    /// flag is cleared, the decay level is loaded with 15 and the divider is
    /// reloaded. Clocking the divider at zero reloads it and clocks the decay
    /// level, which decrements or — if the loop flag is set — wraps to 15.
    pub fn clock(&mut self) {
        if self.start {
            self.start = false;
            self.decay = 15;
            self.divider = self.period;
        } else if self.divider == 0 {
            self.divider = self.period;
            if self.decay > 0 {
                self.decay -= 1;
            } else if self.loop_flag {
                self.decay = 15;
            }
        } else {
            self.divider -= 1;
        }
    }

    /// The 4-bit volume this envelope presents to its channel's gate.
    #[inline]
    pub const fn volume(&self) -> u8 {
        if self.constant {
            self.period
        } else {
            self.decay
        }
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        w.write_bool(self.start)?;
        w.write_u8(self.divider)?;
        w.write_u8(self.decay)?;
        w.write_u8(self.period)?;
        w.write_bool(self.loop_flag)?;
        w.write_bool(self.constant)
    }

    /// Restore what [`Envelope::save`] wrote.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.start = r.read_bool()?;
        self.divider = r.read_u8()?;
        self.decay = r.read_u8()?;
        self.period = r.read_u8()?;
        self.loop_flag = r.read_bool()?;
        self.constant = r.read_bool()?;
        Ok(())
    }
}

/// The automatic duration control shared by the pulse, triangle and noise
/// channels.
///
/// Clocked by the frame counter's half-frame signal. The channel is silent
/// while the counter is zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LengthCounter {
    value: u8,
    halt: bool,
    enabled: bool,
}

impl LengthCounter {
    /// A powered-on length counter: disabled and expired.
    pub const fn new() -> LengthCounter {
        LengthCounter {
            value: 0,
            halt: false,
            enabled: false,
        }
    }

    /// The current count. Zero means the channel is silenced.
    #[inline]
    pub const fn value(&self) -> u8 {
        self.value
    }

    /// Whether the counter is non-zero, which is what `$4015` reports.
    #[inline]
    pub const fn active(&self) -> bool {
        self.value > 0
    }

    /// Whether counting is halted.
    #[inline]
    pub const fn halted(&self) -> bool {
        self.halt
    }

    /// Set the halt flag (`$4000`/`$400C` bit 5, `$4008` bit 7).
    pub fn set_halt(&mut self, halt: bool) {
        self.halt = halt;
    }

    /// Apply the channel's `$4015` enable bit.
    ///
    /// Clearing it forces the counter to zero and blocks reloads until it is
    /// set again; the previous value is lost. Setting it has no immediate
    /// effect ([NESdev APU Length
    /// Counter](https://www.nesdev.org/wiki/APU_Length_Counter)).
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.value = 0;
        }
    }

    /// Whether the channel is enabled through `$4015`.
    #[inline]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// Load from the table using the top five bits of a length-load write.
    ///
    /// A write while the channel is disabled is discarded — that is the quirk
    /// the `$4015` enable bit exists to produce.
    pub fn load(&mut self, value: u8) {
        if self.enabled {
            self.value = LENGTH_TABLE[usize::from(value >> 3)];
        }
    }

    /// One half-frame clock: decrement unless halted or already zero.
    pub fn clock(&mut self) {
        if !self.halt && self.value > 0 {
            self.value -= 1;
        }
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        w.write_u8(self.value)?;
        w.write_bool(self.halt)?;
        w.write_bool(self.enabled)
    }

    /// Restore what [`LengthCounter::save`] wrote.
    pub fn load_state<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.value = r.read_u8()?;
        self.halt = r.read_bool()?;
        self.enabled = r.read_bool()?;
        Ok(())
    }
}
