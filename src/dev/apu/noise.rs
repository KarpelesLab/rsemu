//! The noise channel — `$400C`–`$400F`.
//!
//! Source: [NESdev APU Noise](https://www.nesdev.org/wiki/APU_Noise).

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

use super::frame::Region;
use super::units::{Envelope, LengthCounter};

/// Noise timer periods in **CPU cycles**, indexed by `$400E` bits 3-0.
///
/// The wiki gives these as the number of CPU cycles between shift-register
/// clocks; they are all even because the timer is clocked once per APU cycle.
/// The `$F` NTSC entry is 4068 on every 2A03 revision that has the mode flag;
/// 2046 belongs to the earliest revisions, which had no mode flag at all and
/// are not modelled here.
const NTSC_PERIODS: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
];

/// Noise timer periods in CPU cycles for the RP2A07.
///
/// A genuinely different table, not the NTSC one rescaled: the wiki lists both
/// ([NESdev APU Noise](https://www.nesdev.org/wiki/APU_Noise)), and the ratios
/// between corresponding entries are not constant.
const PAL_PERIODS: [u16; 16] = [
    4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778,
];

/// The period table for a console variant, in CPU cycles.
///
/// The wiki has no Dendy table, and does not need one: the UA6527P is a 2A03
/// clone, so its dividers are the NTSC ones and only the rate they are clocked
/// at differs — the same reading the 59 Hz frame-counter rate forces in
/// [`Region::four_step`](super::frame::Region::four_step).
pub const fn periods(region: Region) -> [u16; 16] {
    match region {
        Region::Ntsc | Region::Dendy => NTSC_PERIODS,
        Region::Pal => PAL_PERIODS,
    }
}

/// The noise channel: envelope, timer, 15-bit LFSR, length counter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Noise {
    /// The envelope generator, which also owns the loop/halt flag.
    pub envelope: Envelope,
    /// The length counter.
    pub length: LengthCounter,
    region: Region,
    /// `$400E` bit 7: short mode, which taps bit 6 instead of bit 1.
    mode: bool,
    /// `$400E` bits 3-0.
    period_index: u8,
    /// The timer's down counter, in APU cycles.
    timer: u16,
    /// The 15-bit linear feedback shift register.
    shift: u16,
}

impl Noise {
    /// A powered-on noise channel.
    ///
    /// [NESdev APU Noise](https://www.nesdev.org/wiki/APU_Noise) states the
    /// shift register is loaded with 1 on power-up. (The [CPU power up
    /// state](https://www.nesdev.org/wiki/CPU_power_up_state) page reports a
    /// measured `$0000` with a first clock shifting in a 1; the two agree on
    /// the sequence from the second clock onward, and 1 is the value the
    /// channel documentation specifies, so that is what is used.)
    pub const fn new(region: Region) -> Noise {
        Noise {
            envelope: Envelope::new(),
            length: LengthCounter::new(),
            region,
            mode: false,
            period_index: 0,
            timer: 0,
            shift: 1,
        }
    }

    /// The shift register's current contents.
    #[inline]
    pub const fn shift(&self) -> u16 {
        self.shift
    }

    /// The timer period in CPU cycles, as the wiki's table gives it.
    #[inline]
    pub fn period_cycles(&self) -> u16 {
        periods(self.region)[usize::from(self.period_index)]
    }

    /// The divider's reload value, in APU cycles.
    ///
    /// A down counter reloaded with `P` has a period of `P + 1`, and the table
    /// is in CPU cycles, so the reload is `cycles / 2 - 1`. The smallest entry
    /// is 4, so this cannot underflow.
    #[inline]
    fn reload(&self) -> u16 {
        self.period_cycles() / 2 - 1
    }

    /// `$400C`: length halt, constant volume flag, volume/envelope period.
    pub fn write_control(&mut self, value: u8) {
        self.envelope.write_control(value);
        self.length.set_halt(self.envelope.loop_flag());
    }

    /// `$400E`: mode flag and period index.
    pub fn write_period(&mut self, value: u8) {
        self.mode = value & 0x80 != 0;
        self.period_index = value & 0x0F;
    }

    /// `$400F`: length counter load and envelope restart.
    pub fn write_length(&mut self, value: u8) {
        self.length.load(value);
        self.envelope.restart();
    }

    /// Clock the timer. Called once per APU cycle.
    pub fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.reload();
            self.clock_shift();
        } else {
            self.timer -= 1;
        }
    }

    /// One shift of the LFSR.
    ///
    /// Feedback is bit 0 XOR bit 6 (mode set) or bit 1 (mode clear); the
    /// register shifts right and the feedback becomes the new bit 14.
    fn clock_shift(&mut self) {
        let tap = if self.mode { 6 } else { 1 };
        let feedback = (self.shift & 1) ^ ((self.shift >> tap) & 1);
        self.shift >>= 1;
        self.shift |= feedback << 14;
    }

    /// The 4-bit level this channel sends to the mixer.
    ///
    /// Silent when bit 0 of the shift register is set or the length counter is
    /// zero.
    pub fn output(&self) -> u8 {
        if !self.length.active() || self.shift & 1 != 0 {
            return 0;
        }
        self.envelope.volume()
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        self.envelope.save(w)?;
        self.length.save(w)?;
        w.write_bool(self.mode)?;
        w.write_u8(self.period_index)?;
        w.write_u16(self.timer)?;
        w.write_u16(self.shift)
    }

    /// Restore what [`Noise::save`] wrote.
    ///
    /// The region is machine configuration, not state, so it is not stored: a
    /// snapshot never changes which console it is being restored on.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.envelope.load(r)?;
        self.length.load_state(r)?;
        self.mode = r.read_bool()?;
        self.period_index = r.read_u8()? & 0x0F;
        self.timer = r.read_u16()?;
        self.shift = r.read_u16()? & 0x7FFF;
        Ok(())
    }
}
