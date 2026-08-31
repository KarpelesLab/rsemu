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
/// A **construction property**, never a `#[cfg]` (`region = "pal"`): one build
/// runs all three, and only the tables below and the CPU divider change.
///
/// A real enum rather than the `#[repr(transparent)]` newtype `CLAUDE.md`
/// prescribes for extensible enumerations, because exhaustiveness genuinely is
/// wanted: every table in this module has one entry per variant, and a new
/// variant must not silently fall through to NTSC.
///
/// This is deliberately *not* the same type as
/// [`ppu::Region`](crate::dev::ppu): `dev-nes-apu` and `dev-nes-ppu` are
/// independent features and neither may require the other (`CLAUDE.md`, "crate
/// shape"). The two agree on the names a machine file writes, which is the only
/// place they meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Region {
    /// RP2A03, 1.789773 MHz CPU, frame counter at 60 Hz.
    #[default]
    Ntsc,
    /// RP2A07, 1.662607 MHz CPU, frame counter at 50 Hz.
    Pal,
    /// UMC UA6527P, the "Dendy" famiclone: 1.773448 MHz CPU, 59 Hz.
    ///
    /// The chip is a 2A03 clone hung off a PAL crystal divided by 15 rather
    /// than 16, so its *sequence* is NTSC's and only the rate it is clocked at
    /// differs. See [`Region::four_step`] for why that is the reading of the
    /// wiki's 59 Hz.
    Dendy,
}

impl Region {
    /// Every name [`Region::from_name`] accepts, for `or_enum` and
    /// `rsemu describe`.
    pub const NAMES: &'static [&'static str] = &["ntsc", "pal", "dendy"];

    /// Parse the `region` property.
    pub fn from_name(name: &str) -> Option<Region> {
        match name {
            "ntsc" => Some(Region::Ntsc),
            "pal" => Some(Region::Pal),
            "dendy" => Some(Region::Dendy),
            _ => None,
        }
    }

    /// The name this variant is written with in a `.machine` file.
    pub const fn name(self) -> &'static str {
        match self {
            Region::Ntsc => "ntsc",
            Region::Pal => "pal",
            Region::Dendy => "dendy",
        }
    }

    /// The part number of the CPU whose audio half this is.
    pub const fn part_number(self) -> &'static str {
        match self {
            Region::Ntsc => "RP2A03",
            Region::Pal => "RP2A07",
            Region::Dendy => "UA6527P",
        }
    }

    /// The board's master crystal, as an exact `(numerator, denominator)` in
    /// hertz.
    ///
    /// Neither is an integer number of hertz — NTSC is 236.25 MHz ÷ 11 and PAL
    /// is 26.6017125 MHz, both *by definition*
    /// ([NESdev cycle reference chart](https://www.nesdev.org/wiki/Cycle_reference_chart))
    /// — which is exactly the case `ROADMAP.md` §4.2's rational oscillator
    /// literals exist for.
    pub const fn master_clock(self) -> (u64, u64) {
        match self {
            Region::Ntsc => (236_250_000, 11),
            // Dendy is a PAL board: same crystal, a different divider.
            Region::Pal | Region::Dendy => (53_203_425, 2),
        }
    }

    /// Master clocks per CPU cycle, which is also per APU tick.
    ///
    /// 12, 16 and 15 respectively. The APU counts CPU cycles, so this is the
    /// only rate figure it needs; it is here rather than in the CPU core
    /// because a board building the clock forest wants it from whichever NES
    /// device it has.
    pub const fn cpu_divider(self) -> u64 {
        match self {
            Region::Ntsc => 12,
            Region::Pal => 16,
            Region::Dendy => 15,
        }
    }

    /// The six mode-0 landmarks, in CPU cycles since the sequence restarted.
    ///
    /// `[quarter, quarter+half, quarter, irq, quarter+half+irq, irq+wrap]`.
    ///
    /// The PAL row is the wiki's PAL table — APU cycles 4156.5, 8313.5,
    /// 12469.5, 16626, 16626.5, 16627 — doubled, exactly as the NTSC row is
    /// derived in this module's header.
    ///
    /// **Dendy uses the NTSC sequence.** The wiki has no Dendy frame-counter
    /// table; what it has is a measured rate of 59 Hz (cycle reference chart,
    /// citing a nesdev forum post by Eugene.S). The NTSC sequence at Dendy's
    /// 1773448 Hz CPU is 1773448 / 29830 = 59.45 Hz, which rounds to 59; the
    /// PAL sequence would give 53.3 Hz, which does not. The rate is therefore
    /// the *consequence* of a Famicom-compatible chip on a slower clock, not a
    /// third table.
    pub const fn four_step(self) -> [u32; 6] {
        match self {
            Region::Ntsc | Region::Dendy => [7457, 14913, 22371, 29828, 29829, 29830],
            Region::Pal => [8313, 16627, 24939, 33252, 33253, 33254],
        }
    }

    /// The mode-1 landmarks, in CPU cycles since the sequence restarted.
    ///
    /// `[quarter, quarter+half, quarter, nothing, quarter+half, wrap]`. Index 3
    /// is the wiki's step 4, which clocks nothing at all; it is kept so the
    /// table lines up with the documented sequence rather than hiding a step.
    ///
    /// Dendy is NTSC here for the reason [`Region::four_step`] gives.
    pub const fn five_step(self) -> [u32; 6] {
        match self {
            Region::Ntsc | Region::Dendy => [7457, 14913, 22371, 29829, 37281, 37282],
            Region::Pal => [8313, 16627, 24939, 33253, 41565, 41566],
        }
    }
}

impl core::fmt::Display for Region {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Region::Ntsc => "NTSC",
            Region::Pal => "PAL",
            Region::Dendy => "Dendy",
        })
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
    region: Region,
    mode: Mode,
    /// `$4017` bit 6. While set, the frame interrupt flag cannot be set.
    inhibit: bool,
    /// A `$4015` read has armed the interrupt flag's clear.
    ///
    /// The clear is not applied by the read: it happens inside the frame
    /// counter, on the next **get** cycle strictly after the read, and a set
    /// signal on that same cycle wins. AccuracyCoin's "Frame Counter IRQ"
    /// codes 6 and 7 are exactly this — two `$4015` reads on consecutive
    /// cycles, and whether the second sees the flag depends only on which of
    /// them landed on the get.
    clear_armed: bool,
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
    pub const fn new(region: Region) -> FrameCounter {
        FrameCounter {
            region,
            mode: Mode::FourStep,
            inhibit: false,
            clear_armed: false,
            irq: false,
            irq_set_at: u64::MAX,
            cycle: 0,
            reset_delay: 0,
        }
    }

    /// Which console this counter is timed for.
    #[inline]
    pub const fn tv_region(&self) -> Region {
        self.region
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

    /// Read the flag, arming its clear, as a `$4015` read does.
    ///
    /// The read reports what the flag is *now* and does not clear it: it arms
    /// a clear that the counter applies on its next get cycle, where a set
    /// signal on the same cycle overrides it. The ROM says outright where this
    /// belongs — "you probably want to clear bit 6 inside the APU cycle code of
    /// your emulator, and not in your 'read $4015' code" (`AccuracyCoin.asm`,
    /// MIT, © 2025 Chris Siebert).
    ///
    /// `peek` is true for a `MemAttrs::debug` access, which must have no side
    /// effect at all.
    pub fn read_irq(&mut self, peek: bool) -> bool {
        if !peek {
            self.clear_armed = true;
        }
        self.irq
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
            // Setting the inhibit bit clears the flag outright — unlike a
            // `$4015` read, which only arms a clear for the next get cycle.
            self.irq = false;
            self.clear_armed = false;
        }
        self.reset_delay = if on_put_cycle { 3 } else { 4 };
    }

    /// Advance one CPU cycle and report what the sequencer produced.
    ///
    /// `now` is the APU tick this cycle corresponds to; it is recorded when the
    /// interrupt flag is set so [`FrameCounter::read_irq`] can detect the
    /// same-cycle race.
    pub fn tick(&mut self, now: u64, on_put_cycle: bool) -> FrameEvent {
        // An armed `$4015` clear lands here, on a get cycle, and the sequencer
        // below may set the flag again on this very cycle — which is what makes
        // a read on the last cycle before the flag is set leave it set.
        if self.clear_armed && !on_put_cycle {
            self.irq = false;
            self.clear_armed = false;
        }
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
                let t = self.region.four_step();
                if self.cycle == t[0] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[1] {
                    FrameEvent::BOTH
                } else if self.cycle == t[2] {
                    FrameEvent::QUARTER
                } else if self.cycle == t[3] {
                    // The flag is raised whether or not interrupts are
                    // inhibited: what the inhibit bit gates is the *line*, and
                    // a program that reads `$4015` on one of these two cycles
                    // sees the flag even with interrupts suppressed. The third
                    // cycle is where the inhibit finally wins.
                    self.set_irq(now);
                    FrameEvent::NONE
                } else if self.cycle == t[4] {
                    self.set_irq(now);
                    FrameEvent::BOTH
                } else if self.cycle == t[5] {
                    if self.inhibit {
                        self.irq = false;
                    } else {
                        self.set_irq(now);
                    }
                    self.cycle = 0;
                    FrameEvent::NONE
                } else {
                    FrameEvent::NONE
                }
            }
            Mode::FiveStep => {
                let t = self.region.five_step();
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

    /// Raise the interrupt flag, remembering when.
    ///
    /// Not gated on the inhibit bit: that gates the IRQ *line*
    /// ([`FrameCounter::inhibited`]), and the flag is readable through `$4015`
    /// either way.
    fn set_irq(&mut self, now: u64) {
        self.irq = true;
        self.irq_set_at = now;
        // A set on this cycle beats a clear armed by an earlier read.
        self.clear_armed = false;
    }

    /// Whether `$4017` bit 6 is suppressing the IRQ line.
    #[inline]
    pub const fn inhibited(&self) -> bool {
        self.inhibit
    }

    /// Return to the power-on state, keeping the timing variant.
    pub fn reset_cold(&mut self) {
        *self = FrameCounter::new(self.region);
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
        w.write_bool(self.clear_armed)?;
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
        self.clear_armed = r.read_bool()?;
        self.irq = r.read_bool()?;
        self.irq_set_at = r.read_u64()?;
        self.cycle = r.read_u32()?;
        self.reset_delay = r.read_u8()?;
        Ok(())
    }
}
