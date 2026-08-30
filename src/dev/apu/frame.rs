//! The frame counter (frame sequencer) — `$4017`.
//!
//! Source: [NESdev APU Frame
//! Counter](https://www.nesdev.org/wiki/APU_Frame_Counter).
//!
//! # Why this counts CPU cycles
//!
//! The wiki states the schedule in APU cycles, with "an additional delay of one
//! CPU cycle for the quarter and half frame signals". An APU cycle is two CPU
//! cycles, its first half is a *get* cycle and its second a *put* cycle, so APU
//! cycle `n` covers CPU cycles `2n` (get) and `2n + 1` (put). Substituting into
//! the wiki's tables gives the schedule in CPU cycles directly, which is the
//! only resolution at which the rest of the register set can be described: the
//! `$4017` reset delay is 3 *or* 4 CPU cycles, and the DMC's rate table is in
//! CPU cycles too. Counting in APU cycles and patching the odd ones back in
//! afterwards would reintroduce exactly the ambiguity this table removes.
//!
//! Worked, for NTSC mode 0: `3728 PUT` is CPU `2 * 3728 + 1 = 7457`;
//! `14914 GET` is CPU `29828`, `14914 PUT` is `29829`, and the wrap at
//! `0 (14915) GET` is `29830`. That reproduces the classic 7457 / 14913 /
//! 22371 / 29828 / 29829 / 29830 table without copying it from anywhere.

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

/// Which console the APU is modelling.
///
/// A real enum rather than the `#[repr(transparent)]` newtype `CLAUDE.md`
/// prescribes for extensible enumerations, because exhaustiveness genuinely is
/// wanted: every table in this module has one entry per variant, and a new
/// variant must not silently fall through to NTSC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Timing {
    /// RP2A03, 1.789773 MHz CPU.
    #[default]
    Ntsc,
    /// RP2A07, 1.662607 MHz CPU.
    Pal,
}

impl Timing {
    /// Parse the `timing` property.
    pub fn from_name(name: &str) -> Option<Timing> {
        match name {
            "ntsc" => Some(Timing::Ntsc),
            "pal" => Some(Timing::Pal),
            _ => None,
        }
    }

    /// The name this variant is written with in a `.machine` file.
    pub const fn name(self) -> &'static str {
        match self {
            Timing::Ntsc => "ntsc",
            Timing::Pal => "pal",
        }
    }

    /// The six mode-0 landmarks, in CPU cycles since the sequence restarted.
    ///
    /// `[quarter, quarter+half, quarter, irq, quarter+half+irq, irq+wrap]`.
    pub const fn four_step(self) -> [u32; 6] {
        match self {
            Timing::Ntsc => [7457, 14913, 22371, 29828, 29829, 29830],
            Timing::Pal => [8313, 16627, 24939, 33252, 33253, 33254],
        }
    }

    /// The mode-1 landmarks, in CPU cycles since the sequence restarted.
    ///
    /// `[quarter, quarter+half, quarter, nothing, quarter+half, wrap]`. Index 3
    /// is the wiki's step 4, which clocks nothing at all; it is kept so the
    /// table lines up with the documented sequence rather than hiding a step.
    pub const fn five_step(self) -> [u32; 6] {
        match self {
            Timing::Ntsc => [7457, 14913, 22371, 29829, 37281, 37282],
            Timing::Pal => [8313, 16627, 24939, 33253, 41565, 41566],
        }
    }
}

/// Which sequence the frame counter is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Mode {
    /// `$4017` bit 7 clear: four steps, with the frame IRQ.
    #[default]
    FourStep,
    /// `$4017` bit 7 set: five steps, no IRQ ever.
    FiveStep,
}

/// What one CPU cycle of the sequencer produced.
///
/// Half-frame signals always coincide with a quarter-frame signal on hardware,
/// so both flags are set together; nothing in the APU relies on seeing a half
/// frame alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameEvent {
    /// Clock the envelopes and the triangle's linear counter.
    pub quarter: bool,
    /// Clock the length counters and the sweep units.
    pub half: bool,
}

impl FrameEvent {
    /// Nothing happened this cycle.
    pub const NONE: FrameEvent = FrameEvent {
        quarter: false,
        half: false,
    };
    /// Envelopes and the linear counter only.
    pub const QUARTER: FrameEvent = FrameEvent {
        quarter: true,
        half: false,
    };
    /// Envelopes, linear counter, length counters and sweeps.
    pub const BOTH: FrameEvent = FrameEvent {
        quarter: true,
        half: true,
    };
}

/// The frame counter: a divider, a looping sequencer, and the frame IRQ flag.
#[derive(Debug, Clone)]
pub struct FrameCounter {
    timing: Timing,
    mode: Mode,
    /// `$4017` bit 6. While set, the frame interrupt flag cannot be set.
    inhibit: bool,
    /// The frame interrupt flag, wired to the CPU's IRQ line.
    irq: bool,
    /// The APU tick at which `irq` was last set, so that a `$4015` read on the
    /// very cycle the flag is set returns 1 without clearing it.
    irq_set_at: u64,
    /// CPU cycles since the sequence last restarted.
    cycle: u32,
    /// CPU cycles left before a pending `$4017` write resets the sequence, or
    /// zero when no write is pending.
    reset_delay: u8,
}

impl FrameCounter {
    /// A powered-on frame counter: mode 0, interrupts enabled, sequence at 0.
    ///
    /// [NESdev CPU power up
    /// state](https://www.nesdev.org/wiki/CPU_power_up_state) records `$4017`
    /// as 0 at power-up, which is to say the frame IRQ is enabled.
    pub const fn new(timing: Timing) -> FrameCounter {
        FrameCounter {
            timing,
            mode: Mode::FourStep,
            inhibit: false,
            irq: false,
            irq_set_at: u64::MAX,
            cycle: 0,
            reset_delay: 0,
        }
    }

    /// Which console this counter is timed for.
    #[inline]
    pub const fn timing(&self) -> Timing {
        self.timing
    }

    /// The sequence currently selected.
    #[inline]
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// The frame interrupt flag.
    #[inline]
    pub const fn irq(&self) -> bool {
        self.irq
    }

    /// CPU cycles since the sequence restarted. Exposed for tests.
    #[inline]
    pub const fn cycle(&self) -> u32 {
        self.cycle
    }

    /// Whether a `$4017` write is still waiting out its 3-or-4-cycle delay.
    #[inline]
    pub const fn reset_pending(&self) -> bool {
        self.reset_delay > 0
    }

    /// Read and conditionally clear the flag, as a `$4015` read does.
    ///
    /// `now` is the current APU tick. Per [NESdev
    /// APU](https://www.nesdev.org/wiki/APU), a flag set at the same moment as
    /// the read reads back as 1 and is *not* cleared. `peek` is true for a
    /// `MemAttrs::debug` access, which must have no side effect at all.
    pub fn read_irq(&mut self, now: u64, peek: bool) -> bool {
        let was = self.irq;
        if !peek && self.irq_set_at != now {
            self.irq = false;
        }
        was
    }

    /// Clear the flag unconditionally (a `$4017` write with bit 6 set).
    pub fn clear_irq(&mut self) {
        self.irq = false;
    }

    /// Apply a `$4017` write.
    ///
    /// `on_put_cycle` says whether the write lands on the second CPU cycle of
    /// an APU cycle. The mode and inhibit bits take effect immediately; the
    /// sequence reset — and, in mode 1, the immediate quarter- and half-frame
    /// clocks — are delayed by 3 or 4 CPU cycles.
    ///
    /// The wiki gives the delay as "3 CPU cycles if the write occurs during an
    /// APU cycle, 4 if between APU cycles". Both readings of that sentence
    /// agree on the observable consequence, which is the one modelled here: the
    /// reset always lands on a *get* cycle, so the restarted sequence keeps the
    /// phase the CPU-cycle tables above assume (even offsets are get cycles).
    pub fn write(&mut self, value: u8, on_put_cycle: bool) {
        self.mode = if value & 0x80 != 0 {
            Mode::FiveStep
        } else {
            Mode::FourStep
        };
        self.inhibit = value & 0x40 != 0;
        if self.inhibit {
            // A flag set this very cycle is still cleared by the inhibit bit:
            // the same-cycle race documented for $4015 is a property of the
            // read path, not of the flag.
            self.irq = false;
        }
        self.reset_delay = if on_put_cycle { 3 } else { 4 };
    }

    /// Advance one CPU cycle and report what the sequencer produced.
    ///
    /// `now` is the APU tick this cycle corresponds to; it is recorded when the
    /// interrupt flag is set so [`FrameCounter::read_irq`] can detect the
    /// same-cycle race.
    pub fn tick(&mut self, now: u64) -> FrameEvent {
        if self.reset_delay > 0 {
            self.reset_delay -= 1;
            if self.reset_delay == 0 {
                self.cycle = 0;
                // "If the mode flag is set, then both quarter frame and half
                // frame signals are also generated" — APU Frame Counter.
                return match self.mode {
                    Mode::FiveStep => FrameEvent::BOTH,
                    Mode::FourStep => FrameEvent::NONE,
                };
            }
        }

        self.cycle += 1;
        match self.mode {
            Mode::FourStep => {
                let t = self.timing.four_step();
                if self.cycle == t[0] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[1] {
                    FrameEvent::BOTH
                } else if self.cycle == t[2] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[3] {
                    self.set_irq(now);
                    FrameEvent::NONE
                } else if self.cycle == t[4] {
                    self.set_irq(now);
                    FrameEvent::BOTH
                } else if self.cycle == t[5] {
                    self.set_irq(now);
                    self.cycle = 0;
                    FrameEvent::NONE
                } else {
                    FrameEvent::NONE
                }
            }
            Mode::FiveStep => {
                let t = self.timing.five_step();
                if self.cycle == t[0] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[1] {
                    FrameEvent::BOTH
                } else if self.cycle == t[2] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[4] {
                    FrameEvent::BOTH
                } else if self.cycle == t[5] {
                    self.cycle = 0;
                    FrameEvent::NONE
                } else {
                    // t[3] is the step that clocks nothing.
                    FrameEvent::NONE
                }
            }
        }
    }

    /// Set the interrupt flag unless inhibited, remembering when.
    fn set_irq(&mut self, now: u64) {
        if !self.inhibit {
            self.irq = true;
            self.irq_set_at = now;
        }
    }

    /// Return to the power-on state, keeping the timing variant.
    pub fn reset_cold(&mut self) {
        *self = FrameCounter::new(self.timing);
    }

    /// A `$4017`-preserving reset.
    ///
    /// [NESdev CPU power up
    /// state](https://www.nesdev.org/wiki/CPU_power_up_state) says `$4017` is
    /// unchanged by a reset, so the mode and inhibit bits survive; the sequence
    /// position and a pending write do not.
    pub fn reset_warm(&mut self) {
        self.irq = false;
        self.irq_set_at = u64::MAX;
        self.cycle = 0;
        self.reset_delay = 0;
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        w.write_u8(match self.mode {
            Mode::FourStep => 0,
            Mode::FiveStep => 1,
        })?;
        w.write_bool(self.inhibit)?;
        w.write_bool(self.irq)?;
        w.write_u64(self.irq_set_at)?;
        w.write_u32(self.cycle)?;
        w.write_u8(self.reset_delay)
    }

    /// Restore what [`FrameCounter::save`] wrote.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.mode = if r.read_u8()? == 0 {
            Mode::FourStep
        } else {
            Mode::FiveStep
        };
        self.inhibit = r.read_bool()?;
        self.irq = r.read_bool()?;
        self.irq_set_at = r.read_u64()?;
        self.cycle = r.read_u32()?;
        self.reset_delay = r.read_u8()?;
        Ok(())
    }
}
