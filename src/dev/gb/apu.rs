//! The Game Boy's sound hardware — four channels at `$FF10`-`$FF3F`.
//!
//! ```text
//!   $FF10-$FF14  channel 1  square wave with a frequency sweep
//!   $FF16-$FF19  channel 2  square wave
//!   $FF1A-$FF1E  channel 3  a 32-sample 4-bit waveform from $FF30-$FF3F
//!   $FF20-$FF23  channel 4  pseudo-random noise from a shift register
//!   $FF24-$FF26  master volume, stereo panning, power and status
//! ```
//!
//! # The frame sequencer is not this device's clock
//!
//! Every channel's length counter, volume envelope and frequency sweep is
//! stepped by a **512 Hz frame sequencer**, and that sequencer is clocked off
//! **bit 12 of the divider** — the same counter `$FF04` reads the top of and
//! that any write to `$FF04` resets (Pan Docs, *Audio Details*).
//!
//! So this device does not generate that clock. It takes it, on the
//! [`DIV_APU_PIN`] input, from [`crate::dev::gb::timer`], and the machine file
//! wires the two together. Writing `$FF04` is therefore audible — it shifts the
//! phase of every envelope in the machine — which is a genuine cross-device
//! relationship the wire graph expresses without `core::` learning what a Game
//! Boy is.
//!
//! # Output
//!
//! Samples are taken every [`SAMPLE_DIVISOR`] crystal periods, giving exactly
//! 4194304/128 = **32 768 Hz**, stereo. The divisor is a power of two on purpose:
//! `ROADMAP.md` §4.2 forbids floating point in the time path, and an integer
//! divisor of the crystal keeps the sample clock exactly related to everything
//! else in the tree rather than nearly related to it.
//!
//! [`GbApu::take_samples`] drains the ring; a machine with no audio sink simply
//! never drains it and the oldest samples are dropped.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2). Channels are not stepped one crystal
//! period at a time — each holds a countdown and is advanced in whole waveform
//! periods, so the cost is proportional to the number of edges rather than to
//! the number of clocks.
//!
//! # What is not modelled
//!
//! Written down rather than discovered. The APU is the part of this machine that
//! is **complete in shape but not conformance-gated**: `ROADMAP.md` §13's phase-4
//! gate names the mooneye and blargg *CPU and timing* suites, not blargg's
//! `dmg_sound`, and none of the following is exercised by anything rsemu
//! currently runs.
//!
//! * **Wave-RAM access conflicts.** On a DMG, reading `$FF30`-`$FF3F` while
//!   channel 3 is playing returns the byte the channel happens to be on, and
//!   only within a narrow window. Here the RAM always reads normally.
//! * **The "zombie" envelope**, where writing `NRx2` while a channel plays
//!   changes the volume by rules that differ between console revisions.
//! * **The obscure trigger timing** — the extra length clock when a channel is
//!   triggered on a frame-sequencer step that does not clock length, and
//!   channel 3's first-sample delay.
//! * **The high-pass filter** on the analogue output.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0) — *Audio*, *Audio Registers*,
//! *Audio Details*. No emulator source was consulted.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

/// Where the register block starts.
pub const REGISTER_BASE: u64 = 0xff10;

/// How far it runs: `$FF10`-`$FF3F`, wave RAM included.
pub const REGISTER_LEN: u64 = 0x30;

/// The name a `map` statement reaches it by.
pub const REGISTER_REGION: &str = "regs";

/// The input pin the divider's 512 Hz output drives.
pub const DIV_APU_PIN: &str = "div-apu";

/// How many crystal periods there are between output samples.
///
/// 4 194 304 / 128 = 32 768 Hz, exactly.
pub const SAMPLE_DIVISOR: u64 = 128;

/// The output sample rate that divisor produces.
pub const SAMPLE_RATE: u64 = 4_194_304 / SAMPLE_DIVISOR;

/// How many stereo frames the output ring holds before the oldest are dropped.
pub const RING_FRAMES: usize = SAMPLE_RATE as usize / 4;

/// The four duty cycles channels 1 and 2 can take, as eight-step patterns.
const DUTY: [[u8; 8]; 4] = [
    [0, 0, 0, 0, 0, 0, 0, 1], // 12.5%
    [1, 0, 0, 0, 0, 0, 0, 1], // 25%
    [1, 0, 0, 0, 0, 1, 1, 1], // 50%
    [0, 1, 1, 1, 1, 1, 1, 0], // 75%
];

/// The noise channel's divisor table. Index 0 is a special case worth half of
/// index 1, which is why it is 8 rather than 0.
const NOISE_DIVISOR: [u32; 8] = [8, 16, 32, 48, 64, 80, 96, 112];

/// A volume envelope, shared by channels 1, 2 and 4.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Envelope {
    /// The register as written: initial volume, direction, period.
    reg: u8,
    /// The current volume, 0-15.
    volume: u8,
    /// Steps left before the next change.
    timer: u8,
}

impl Envelope {
    fn period(self) -> u8 {
        self.reg & 0x07
    }

    fn increasing(self) -> bool {
        self.reg & 0x08 != 0
    }

    /// Whether the channel's digital-to-analogue converter is powered.
    ///
    /// The top five bits of the register drive it directly: all zero and the DAC
    /// is off, which silences the channel *and* clears its status bit.
    fn dac_on(self) -> bool {
        self.reg & 0xf8 != 0
    }

    fn trigger(&mut self) {
        self.volume = self.reg >> 4;
        self.timer = self.period();
    }

    fn step(&mut self) {
        if self.period() == 0 {
            return;
        }
        if self.timer > 0 {
            self.timer -= 1;
        }
        if self.timer != 0 {
            return;
        }
        self.timer = self.period();
        if self.increasing() && self.volume < 15 {
            self.volume += 1;
        } else if !self.increasing() && self.volume > 0 {
            self.volume -= 1;
        }
    }
}

/// A length counter. Channels 1, 2 and 4 count 64 steps; channel 3 counts 256.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Length {
    counter: u16,
    enabled: bool,
}

impl Length {
    fn set(&mut self, value: u16, max: u16) {
        self.counter = max - value;
    }

    /// Step, returning whether the channel should switch off.
    fn step(&mut self) -> bool {
        if !self.enabled || self.counter == 0 {
            return false;
        }
        self.counter -= 1;
        self.counter == 0
    }

    fn trigger(&mut self, max: u16) {
        if self.counter == 0 {
            self.counter = max;
        }
    }
}

/// A square-wave channel. Channel 1 additionally owns the sweep.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Square {
    enabled: bool,
    /// `NRx1`: duty in the top two bits, initial length in the rest.
    duty_reg: u8,
    envelope: Envelope,
    length: Length,
    /// The 11-bit period value. The waveform's period is `(2048 - freq) * 4`
    /// crystal periods.
    freq: u16,
    /// Crystal periods until the next step of the duty pattern.
    timer: u32,
    /// Where in the eight-step duty pattern the channel is.
    phase: u8,

    // -- channel 1 only -----------------------------------------------------
    /// `NR10`: sweep period, direction and shift.
    sweep_reg: u8,
    sweep_timer: u8,
    sweep_shadow: u16,
    sweep_on: bool,
    /// Whether a decreasing sweep calculation has happened, which is what makes
    /// switching to increasing mode afterwards disable the channel.
    sweep_negated: bool,
}

impl Square {
    fn period(&self) -> u32 {
        (2048 - u32::from(self.freq & 0x7ff)) * 4
    }

    /// Advance by `clocks`, stepping the duty pattern for each whole period.
    fn advance(&mut self, clocks: u64) {
        if !self.enabled {
            return;
        }
        let mut left = clocks;
        while left > 0 {
            let step = left.min(u64::from(self.timer));
            self.timer -= step as u32;
            left -= step;
            if self.timer == 0 {
                self.timer = self.period();
                self.phase = (self.phase + 1) & 7;
            }
        }
    }

    /// The channel's digital output, 0-15.
    fn output(&self) -> u8 {
        if !self.enabled || !self.envelope.dac_on() {
            return 0;
        }
        DUTY[usize::from(self.duty_reg >> 6)][usize::from(self.phase)] * self.envelope.volume
    }

    fn trigger(&mut self) {
        self.enabled = self.envelope.dac_on();
        self.length.trigger(64);
        self.envelope.trigger();
        self.timer = self.period();
        // Channel 1's sweep is armed here, and a shift of zero with a period of
        // zero leaves it inert.
        self.sweep_shadow = self.freq & 0x7ff;
        self.sweep_timer = if self.sweep_reg >> 4 & 7 == 0 {
            8
        } else {
            self.sweep_reg >> 4 & 7
        };
        self.sweep_on = (self.sweep_reg >> 4) & 7 != 0 || self.sweep_reg & 7 != 0;
        self.sweep_negated = false;
        if self.sweep_reg & 7 != 0 && self.sweep_next().is_none() {
            self.enabled = false;
        }
    }

    /// The frequency a sweep step would produce, or `None` if it overflows —
    /// which switches the channel off.
    fn sweep_next(&mut self) -> Option<u16> {
        let shift = self.sweep_reg & 7;
        let delta = self.sweep_shadow >> shift;
        if self.sweep_reg & 0x08 != 0 {
            self.sweep_negated = true;
            Some(self.sweep_shadow.wrapping_sub(delta) & 0x7ff)
        } else {
            let next = self.sweep_shadow + delta;
            (next <= 2047).then_some(next)
        }
    }

    /// One step of the 128 Hz sweep clock.
    fn step_sweep(&mut self) {
        if !self.sweep_on {
            return;
        }
        if self.sweep_timer > 0 {
            self.sweep_timer -= 1;
        }
        if self.sweep_timer != 0 {
            return;
        }
        let period = (self.sweep_reg >> 4) & 7;
        self.sweep_timer = if period == 0 { 8 } else { period };
        if period == 0 {
            return;
        }
        match self.sweep_next() {
            Some(next) if self.sweep_reg & 7 != 0 => {
                self.sweep_shadow = next;
                self.freq = next;
                // The overflow check happens twice per step; the second one is
                // what actually silences a runaway sweep.
                if self.sweep_next().is_none() {
                    self.enabled = false;
                }
            }
            Some(_) => {}
            None => self.enabled = false,
        }
    }
}

/// Channel 3: a 32-sample, 4-bit waveform held in `$FF30`-`$FF3F`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Wave {
    enabled: bool,
    /// `NR30` bit 7: the DAC's power.
    dac: bool,
    /// `NR32` bits 5-6: 0 mute, 1 full, 2 half, 3 quarter.
    level: u8,
    length: Length,
    freq: u16,
    timer: u32,
    /// Which of the 32 nibbles is next.
    position: u8,
    /// The nibble most recently read out, which is what the channel outputs.
    sample: u8,
    ram: [u8; 16],
}

impl Wave {
    /// Twice as fast as a square channel's, which is why channel 3 reaches an
    /// octave higher for the same register value.
    fn period(&self) -> u32 {
        (2048 - u32::from(self.freq & 0x7ff)) * 2
    }

    fn advance(&mut self, clocks: u64) {
        if !self.enabled {
            return;
        }
        let mut left = clocks;
        while left > 0 {
            let step = left.min(u64::from(self.timer));
            self.timer -= step as u32;
            left -= step;
            if self.timer == 0 {
                self.timer = self.period();
                self.position = (self.position + 1) & 31;
                let byte = self.ram[usize::from(self.position / 2)];
                self.sample = if self.position.is_multiple_of(2) {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
            }
        }
    }

    fn output(&self) -> u8 {
        if !self.enabled || !self.dac {
            return 0;
        }
        match self.level {
            0 => 0,
            1 => self.sample,
            2 => self.sample >> 1,
            _ => self.sample >> 2,
        }
    }

    fn trigger(&mut self) {
        self.enabled = self.dac;
        self.length.trigger(256);
        self.timer = self.period();
        self.position = 0;
    }
}

/// Channel 4: a linear-feedback shift register clocked at a programmable rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Noise {
    enabled: bool,
    envelope: Envelope,
    length: Length,
    /// `NR43`: shift in the top nibble, width in bit 3, divisor in the low three.
    poly: u8,
    timer: u32,
    /// The shift register. Fifteen bits, or seven when `NR43` bit 3 is set.
    lfsr: u16,
}

impl Default for Noise {
    fn default() -> Self {
        Noise {
            enabled: false,
            envelope: Envelope::default(),
            length: Length::default(),
            poly: 0,
            timer: 0,
            lfsr: 0x7fff,
        }
    }
}

impl Noise {
    fn period(&self) -> u32 {
        NOISE_DIVISOR[usize::from(self.poly & 7)] << (self.poly >> 4)
    }

    fn advance(&mut self, clocks: u64) {
        if !self.enabled {
            return;
        }
        let mut left = clocks;
        while left > 0 {
            let step = left.min(u64::from(self.timer));
            self.timer -= step as u32;
            left -= step;
            if self.timer == 0 {
                self.timer = self.period().max(1);
                // The exclusive-OR of the low two bits is fed back into bit 14,
                // and into bit 6 as well in the narrow mode — which is what
                // makes the narrow mode buzz rather than hiss.
                let feedback = (self.lfsr ^ (self.lfsr >> 1)) & 1;
                self.lfsr = (self.lfsr >> 1) | (feedback << 14);
                if self.poly & 0x08 != 0 {
                    self.lfsr = (self.lfsr & !0x40) | (feedback << 6);
                }
            }
        }
    }

    fn output(&self) -> u8 {
        if !self.enabled || !self.envelope.dac_on() {
            return 0;
        }
        // Bit 0 *inverted*: the channel is loud when the register's low bit is
        // clear.
        u8::from(self.lfsr & 1 == 0) * self.envelope.volume
    }

    fn trigger(&mut self) {
        self.enabled = self.envelope.dac_on();
        self.length.trigger(64);
        self.envelope.trigger();
        self.timer = self.period().max(1);
        self.lfsr = 0x7fff;
    }
}

/// The whole sound unit.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Engine {
    powered: bool,
    ch1: Square,
    ch2: Square,
    ch3: Wave,
    ch4: Noise,
    /// `NR50`: the two output amplifiers' volumes, and the unused `VIN` bits.
    nr50: u8,
    /// `NR51`: which channel reaches which output.
    nr51: u8,
    /// Which of the frame sequencer's eight steps is next.
    frame_step: u8,
    /// Crystal periods until the next output sample.
    sample_timer: u64,
}

impl Default for Engine {
    fn default() -> Self {
        Engine {
            powered: false,
            ch1: Square::default(),
            ch2: Square::default(),
            ch3: Wave::default(),
            ch4: Noise::default(),
            nr50: 0,
            nr51: 0,
            frame_step: 0,
            sample_timer: SAMPLE_DIVISOR,
        }
    }
}

impl Engine {
    /// `NR52` as the guest reads it: power in bit 7, the four status bits in the
    /// low nibble, and the three unimplemented bits as ones.
    fn read_nr52(&self) -> u8 {
        let mut byte = 0x70;
        if self.powered {
            byte |= 0x80;
        }
        byte |= u8::from(self.ch1.enabled);
        byte |= u8::from(self.ch2.enabled) << 1;
        byte |= u8::from(self.ch3.enabled) << 2;
        byte |= u8::from(self.ch4.enabled) << 3;
        byte
    }

    /// One step of the 512 Hz frame sequencer.
    ///
    /// The eight-step pattern is the whole of Pan Docs' *Audio Details* table:
    /// length on the even steps, sweep on 2 and 6, the envelope on 7.
    fn step_frame(&mut self) {
        if !self.powered {
            return;
        }
        let step = self.frame_step;
        self.frame_step = (self.frame_step + 1) & 7;
        if step.is_multiple_of(2) {
            if self.ch1.length.step() {
                self.ch1.enabled = false;
            }
            if self.ch2.length.step() {
                self.ch2.enabled = false;
            }
            if self.ch3.length.step() {
                self.ch3.enabled = false;
            }
            if self.ch4.length.step() {
                self.ch4.enabled = false;
            }
        }
        if step == 2 || step == 6 {
            self.ch1.step_sweep();
        }
        if step == 7 {
            self.ch1.envelope.step();
            self.ch2.envelope.step();
            self.ch4.envelope.step();
        }
    }

    /// One stereo frame, as signed 16-bit samples.
    ///
    /// The mix is deliberately linear. A real DMG's four DACs feed one analogue
    /// summing node with a non-linear response and a high-pass filter, and none
    /// of that is modelled — see the module documentation.
    fn sample(&self) -> (i16, i16) {
        if !self.powered {
            return (0, 0);
        }
        let outs = [
            self.ch1.output(),
            self.ch2.output(),
            self.ch3.output(),
            self.ch4.output(),
        ];
        let mut right = 0i32;
        let mut left = 0i32;
        for (i, out) in outs.iter().enumerate() {
            // A digital 0-15 becomes -15..+15 around the DAC's midpoint.
            let signed = i32::from(*out) * 2 - 15;
            if self.nr51 & (1 << i) != 0 {
                right += signed;
            }
            if self.nr51 & (1 << (i + 4)) != 0 {
                left += signed;
            }
        }
        // The master volume is 0-7 meaning 1x-8x.
        let vol_right = i32::from(self.nr50 & 7) + 1;
        let vol_left = i32::from((self.nr50 >> 4) & 7) + 1;
        // Four channels of +-15 at 8x is +-480; scaling by 68 fills the range
        // without clipping.
        (
            (left * vol_left * 68) as i16,
            (right * vol_right * 68) as i16,
        )
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

struct Shared {
    engine: Mutex<Engine>,
    ring: Mutex<VecDeque<(i16, i16)>>,
    lazy: Mutex<Option<LazyHandle>>,
    tick: AtomicU64,
    /// Whether output frames are kept at all. A machine with no audio sink pays
    /// nothing for the ring.
    recording: AtomicBool,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("engine", &self.engine)
            .finish_non_exhaustive()
    }
}

impl Shared {
    fn publish(&self, _engine: &Engine, now: u64) {
        self.tick.store(now, Ordering::Relaxed);
    }

    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }
}

/// The Game Boy's sound unit.
pub struct GbApu {
    shared: Arc<Shared>,
    regs_region: RegionRef,
    /// The frame-sequencer input pin, kept alive here: a net holds only a weak
    /// reference to its sinks.
    pin: Mutex<Option<Arc<FramePin>>>,
}

impl fmt::Debug for GbApu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbApu")
            .field("engine", &self.shared.engine)
            .finish_non_exhaustive()
    }
}

impl Default for GbApu {
    fn default() -> Self {
        GbApu::new()
    }
}

impl GbApu {
    /// A sound unit in its power-on state, which is powered *off*.
    #[must_use]
    pub fn new() -> GbApu {
        let shared = Arc::new(Shared {
            engine: Mutex::with_rank(LockRank::DEVICE, Engine::default()),
            ring: Mutex::new(VecDeque::new()),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            recording: AtomicBool::new(false),
        });
        let regs_region = Arc::new(MmioRegion::io(
            "gb.apu.regs",
            REGISTER_LEN,
            Arc::new(ApuPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbApu {
            shared,
            regs_region,
            pin: Mutex::new(None),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or is unknown.
    pub fn from_props(props: &Props) -> Result<GbApu> {
        let mut r = props.reader();
        let record = r.or("record", false)?;
        r.finish()?;
        let apu = GbApu::new();
        apu.set_recording(record);
        Ok(apu)
    }

    /// Whether output frames are kept for [`take_samples`](GbApu::take_samples).
    pub fn set_recording(&self, on: bool) {
        self.shared.recording.store(on, Ordering::Relaxed);
        if !on {
            self.shared.ring.lock().clear();
        }
    }

    /// Drain the output ring: stereo frames at [`SAMPLE_RATE`], left then right.
    pub fn take_samples(&self) -> Vec<(i16, i16)> {
        let mut ring = self.shared.ring.lock();
        ring.drain(..).collect()
    }

    /// How many frames are waiting.
    #[must_use]
    pub fn queued_samples(&self) -> usize {
        self.shared.ring.lock().len()
    }

    /// `NR52` as the guest reads it.
    #[must_use]
    pub fn status(&self) -> u8 {
        self.shared.engine.lock().read_nr52()
    }

    /// Whether the unit is powered.
    #[must_use]
    pub fn powered(&self) -> bool {
        self.shared.engine.lock().powered
    }

    /// Which of the frame sequencer's eight steps is next.
    #[must_use]
    pub fn frame_step(&self) -> u8 {
        self.shared.engine.lock().frame_step
    }

    /// Step the frame sequencer, as the divider's 512 Hz output does.
    ///
    /// Public because a test wires the pin by hand; a described machine drives
    /// it through the wire graph.
    pub fn step_frame_sequencer(&self) {
        self.shared.engine.lock().step_frame();
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Read one register by offset from `$FF10` — for a test.
    #[must_use]
    pub fn read_register(&self, offset: u64) -> u8 {
        read_reg(&self.shared.engine.lock(), offset)
    }

    /// Write one register by offset from `$FF10` — for a test.
    pub fn write_register(&self, offset: u64, value: u8) {
        write_reg(&mut self.shared.engine.lock(), offset, value);
    }

    /// Advance to `target` crystal periods since reset.
    pub fn advance_to(&self, target: u64) {
        let mut engine = self.shared.engine.lock();
        let mut now = self.shared.tick.load(Ordering::Relaxed);
        if target <= now {
            return;
        }
        let recording = self.shared.recording.load(Ordering::Relaxed);
        while now < target {
            let step = (target - now).min(engine.sample_timer);
            engine.ch1.advance(step);
            engine.ch2.advance(step);
            engine.ch3.advance(step);
            engine.ch4.advance(step);
            engine.sample_timer -= step;
            now += step;
            if engine.sample_timer == 0 {
                engine.sample_timer = SAMPLE_DIVISOR;
                if recording {
                    let frame = engine.sample();
                    let mut ring = self.shared.ring.lock();
                    if ring.len() >= RING_FRAMES {
                        ring.pop_front();
                    }
                    ring.push_back(frame);
                }
            }
        }
        self.shared.publish(&engine, now);
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// The bits of each register a read returns as ones, indexed from `$FF10`.
///
/// Nearly every sound register has write-only fields, and a program reading one
/// back gets ones where they were. Pan Docs, *Audio Registers*, tabulates this;
/// the values are copied from that table and nowhere else.
const READ_MASK: [u8; 0x17] = [
    0x80, // NR10
    0x3f, // NR11 — the length is write-only
    0x00, // NR12
    0xff, // NR13 — write-only
    0xbf, // NR14 — only the length enable reads back
    0xff, // $FF15 — nothing here
    0x3f, // NR21
    0x00, // NR22
    0xff, // NR23
    0xbf, // NR24
    0x7f, // NR30
    0xff, // NR31 — write-only
    0x9f, // NR32
    0xff, // NR33
    0xbf, // NR34
    0xff, // $FF1F — nothing here
    0xff, // NR41 — write-only
    0x00, // NR42
    0x00, // NR43
    0xbf, // NR44
    0x00, // NR50
    0x00, // NR51
    0x70, // NR52 — handled specially
];

fn read_reg(engine: &Engine, offset: u64) -> u8 {
    let raw = match offset {
        0x00 => engine.ch1.sweep_reg,
        0x01 => engine.ch1.duty_reg,
        0x02 => engine.ch1.envelope.reg,
        0x03 => (engine.ch1.freq & 0xff) as u8,
        0x04 => u8::from(engine.ch1.length.enabled) << 6,
        0x06 => engine.ch2.duty_reg,
        0x07 => engine.ch2.envelope.reg,
        0x08 => (engine.ch2.freq & 0xff) as u8,
        0x09 => u8::from(engine.ch2.length.enabled) << 6,
        0x0a => u8::from(engine.ch3.dac) << 7,
        0x0c => engine.ch3.level << 5,
        0x0d => (engine.ch3.freq & 0xff) as u8,
        0x0e => u8::from(engine.ch3.length.enabled) << 6,
        0x11 => engine.ch4.envelope.reg,
        0x12 => engine.ch4.poly,
        0x13 => u8::from(engine.ch4.length.enabled) << 6,
        0x14 => engine.nr50,
        0x15 => engine.nr51,
        0x16 => return engine.read_nr52(),
        // Wave RAM. On a DMG this is not always readable while channel 3 is
        // playing; see the module documentation.
        0x20..=0x2f => return engine.ch3.ram[(offset - 0x20) as usize],
        _ => 0,
    };
    let mask = READ_MASK.get(offset as usize).copied().unwrap_or(0xff);
    raw | mask
}

fn write_reg(engine: &mut Engine, offset: u64, value: u8) {
    // Wave RAM is writable whatever the power state, and so is `NR52` itself.
    if (0x20..=0x2f).contains(&offset) {
        engine.ch3.ram[(offset - 0x20) as usize] = value;
        return;
    }
    if offset == 0x16 {
        let on = value & 0x80 != 0;
        if on == engine.powered {
            return;
        }
        engine.powered = on;
        if !on {
            // Powering off zeroes every register and silences every channel.
            // Wave RAM survives, which is the one exception (Pan Docs, *Audio
            // Registers*).
            let ram = engine.ch3.ram;
            let sample_timer = engine.sample_timer;
            *engine = Engine::default();
            engine.ch3.ram = ram;
            engine.sample_timer = sample_timer;
        }
        return;
    }
    if !engine.powered {
        // Every other register ignores writes while the unit is off.
        return;
    }
    match offset {
        0x00 => {
            engine.ch1.sweep_reg = value;
            // Clearing the direction bit after a decreasing calculation has
            // already happened switches the channel off.
            if value & 0x08 == 0 && engine.ch1.sweep_negated {
                engine.ch1.enabled = false;
            }
        }
        0x01 => {
            engine.ch1.duty_reg = value;
            engine.ch1.length.set(u16::from(value & 0x3f), 64);
        }
        0x02 => {
            engine.ch1.envelope.reg = value;
            if !engine.ch1.envelope.dac_on() {
                engine.ch1.enabled = false;
            }
        }
        0x03 => engine.ch1.freq = (engine.ch1.freq & 0x700) | u16::from(value),
        0x04 => {
            engine.ch1.freq = (engine.ch1.freq & 0xff) | (u16::from(value & 7) << 8);
            engine.ch1.length.enabled = value & 0x40 != 0;
            if value & 0x80 != 0 {
                engine.ch1.trigger();
            }
        }
        0x06 => {
            engine.ch2.duty_reg = value;
            engine.ch2.length.set(u16::from(value & 0x3f), 64);
        }
        0x07 => {
            engine.ch2.envelope.reg = value;
            if !engine.ch2.envelope.dac_on() {
                engine.ch2.enabled = false;
            }
        }
        0x08 => engine.ch2.freq = (engine.ch2.freq & 0x700) | u16::from(value),
        0x09 => {
            engine.ch2.freq = (engine.ch2.freq & 0xff) | (u16::from(value & 7) << 8);
            engine.ch2.length.enabled = value & 0x40 != 0;
            if value & 0x80 != 0 {
                engine.ch2.trigger();
            }
        }
        0x0a => {
            engine.ch3.dac = value & 0x80 != 0;
            if !engine.ch3.dac {
                engine.ch3.enabled = false;
            }
        }
        0x0b => engine.ch3.length.set(u16::from(value), 256),
        0x0c => engine.ch3.level = (value >> 5) & 3,
        0x0d => engine.ch3.freq = (engine.ch3.freq & 0x700) | u16::from(value),
        0x0e => {
            engine.ch3.freq = (engine.ch3.freq & 0xff) | (u16::from(value & 7) << 8);
            engine.ch3.length.enabled = value & 0x40 != 0;
            if value & 0x80 != 0 {
                engine.ch3.trigger();
            }
        }
        0x10 => engine.ch4.length.set(u16::from(value & 0x3f), 64),
        0x11 => {
            engine.ch4.envelope.reg = value;
            if !engine.ch4.envelope.dac_on() {
                engine.ch4.enabled = false;
            }
        }
        0x12 => engine.ch4.poly = value,
        0x13 => {
            engine.ch4.length.enabled = value & 0x40 != 0;
            if value & 0x80 != 0 {
                engine.ch4.trigger();
            }
        }
        0x14 => engine.nr50 = value,
        0x15 => engine.nr51 = value,
        _ => {}
    }
}

/// The `$FF10`-`$FF3F` register block.
struct ApuPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for ApuPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ApuPort").finish_non_exhaustive()
    }
}

impl MemOps for ApuPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        *byte = read_reg(&self.shared.engine.lock(), offset);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A write here triggers channels and can power the unit down.
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        write_reg(&mut self.shared.engine.lock(), offset, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The frame-sequencer input: the divider's 512 Hz output.
///
/// Edge-triggered on the way up, like the CPU's interrupt pins, because the
/// divider drives it as a pulse.
#[derive(Debug)]
pub struct FramePin {
    shared: Arc<Shared>,
    inputs: FanIn,
    last: AtomicBool,
}

impl WireSink for FramePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let now = self.inputs.resolve(Resolve::Or).is_high();
        let was = self.last.swap(now, Ordering::AcqRel);
        if now && !was {
            self.shared.engine.lock().step_frame();
        }
    }
}

/// The `gb.apu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.apu",
    version: 1,
    summary: "Game Boy sound unit: two square channels, wave, noise, and the 512 Hz sequencer",
    properties: &[PropertySpec {
        name: "record",
        kind: ValueKind::Bool,
        required: false,
        summary: "keep output frames in a ring for a host audio sink to drain",
    }],
    construct: |props| Ok(Box::new(GbApu::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbApu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs_region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != DIV_APU_PIN {
            return None;
        }
        let pin = Arc::new(FramePin {
            shared: Arc::clone(&self.shared),
            inputs: FanIn::new(sources),
            last: AtomicBool::new(false),
        });
        *self.pin.lock() = Some(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn reset(&self, _kind: ResetKind) {
        // The tick is the clock domain's position, not this device's state, and
        // `Machine::reset` does not rewind domains.
        let now = self.shared.tick.load(Ordering::Relaxed);
        let mut engine = self.shared.engine.lock();
        *engine = Engine::default();
        self.shared.publish(&engine, now);
        drop(engine);
        self.shared.ring.lock().clear();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let engine = self.shared.engine.lock().clone();
        w.write_bool(engine.powered)?;
        w.write_u8(engine.nr50)?;
        w.write_u8(engine.nr51)?;
        w.write_u8(engine.frame_step)?;
        w.write_u64(engine.sample_timer)?;
        for ch in [&engine.ch1, &engine.ch2] {
            w.write_bool(ch.enabled)?;
            w.write_u8(ch.duty_reg)?;
            w.write_u8(ch.envelope.reg)?;
            w.write_u8(ch.envelope.volume)?;
            w.write_u8(ch.envelope.timer)?;
            w.write_u16(ch.length.counter)?;
            w.write_bool(ch.length.enabled)?;
            w.write_u16(ch.freq)?;
            w.write_u32(ch.timer)?;
            w.write_u8(ch.phase)?;
            w.write_u8(ch.sweep_reg)?;
            w.write_u8(ch.sweep_timer)?;
            w.write_u16(ch.sweep_shadow)?;
            w.write_bool(ch.sweep_on)?;
            w.write_bool(ch.sweep_negated)?;
        }
        w.write_bool(engine.ch3.enabled)?;
        w.write_bool(engine.ch3.dac)?;
        w.write_u8(engine.ch3.level)?;
        w.write_u16(engine.ch3.length.counter)?;
        w.write_bool(engine.ch3.length.enabled)?;
        w.write_u16(engine.ch3.freq)?;
        w.write_u32(engine.ch3.timer)?;
        w.write_u8(engine.ch3.position)?;
        w.write_u8(engine.ch3.sample)?;
        w.write_bytes(&engine.ch3.ram)?;
        w.write_bool(engine.ch4.enabled)?;
        w.write_u8(engine.ch4.envelope.reg)?;
        w.write_u8(engine.ch4.envelope.volume)?;
        w.write_u8(engine.ch4.envelope.timer)?;
        w.write_u16(engine.ch4.length.counter)?;
        w.write_bool(engine.ch4.length.enabled)?;
        w.write_u8(engine.ch4.poly)?;
        w.write_u32(engine.ch4.timer)?;
        w.write_u16(engine.ch4.lfsr)?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut engine = Engine {
            powered: r.read_bool()?,
            nr50: r.read_u8()?,
            nr51: r.read_u8()?,
            frame_step: r.read_u8()?,
            sample_timer: r.read_u64()?,
            ..Engine::default()
        };
        for index in 0..2 {
            let ch = Square {
                enabled: r.read_bool()?,
                duty_reg: r.read_u8()?,
                envelope: Envelope {
                    reg: r.read_u8()?,
                    volume: r.read_u8()?,
                    timer: r.read_u8()?,
                },
                length: Length {
                    counter: r.read_u16()?,
                    enabled: r.read_bool()?,
                },
                freq: r.read_u16()?,
                timer: r.read_u32()?,
                phase: r.read_u8()?,
                sweep_reg: r.read_u8()?,
                sweep_timer: r.read_u8()?,
                sweep_shadow: r.read_u16()?,
                sweep_on: r.read_bool()?,
                sweep_negated: r.read_bool()?,
            };
            if index == 0 {
                engine.ch1 = ch;
            } else {
                engine.ch2 = ch;
            }
        }
        engine.ch3.enabled = r.read_bool()?;
        engine.ch3.dac = r.read_bool()?;
        engine.ch3.level = r.read_u8()?;
        engine.ch3.length.counter = r.read_u16()?;
        engine.ch3.length.enabled = r.read_bool()?;
        engine.ch3.freq = r.read_u16()?;
        engine.ch3.timer = r.read_u32()?;
        engine.ch3.position = r.read_u8()?;
        engine.ch3.sample = r.read_u8()?;
        let ram = r.read_bytes()?;
        if ram.len() != 16 {
            return Err(crate::core::Error::State(String::from(
                "the sound unit's snapshot has the wrong wave-RAM size",
            )));
        }
        engine.ch3.ram.copy_from_slice(ram);
        engine.ch4.enabled = r.read_bool()?;
        engine.ch4.envelope.reg = r.read_u8()?;
        engine.ch4.envelope.volume = r.read_u8()?;
        engine.ch4.envelope.timer = r.read_u8()?;
        engine.ch4.length.counter = r.read_u16()?;
        engine.ch4.length.enabled = r.read_bool()?;
        engine.ch4.poly = r.read_u8()?;
        engine.ch4.timer = r.read_u32()?;
        engine.ch4.lfsr = r.read_u16()?;
        let tick = r.read_u64()?;
        let mut slot = self.shared.engine.lock();
        *slot = engine;
        self.shared.publish(&slot, tick);
        Ok(())
    }

    // -- lazily advanced ----------------------------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        GbApu::advance_to(self, tick);
    }

    /// **None**, and that is a decision rather than an omission.
    ///
    /// `next_event_tick` is what bounds the scheduler's quantum, and it should
    /// report the next tick at which something a *program* can observe changes.
    /// Nothing here qualifies: every register is sampled through
    /// sync-on-access when it is read, the frame sequencer is clocked by a
    /// wire rather than by this device's own clock, and the output ring is read
    /// by a host audio sink that does not care which tick a frame was produced
    /// on. Reporting the next *sample* instant instead would clamp every quantum
    /// in the machine to 128 crystal periods — thirty-two thousand quanta a
    /// second, all of them to compute one stereo frame nobody was waiting for.
    fn next_event_tick(&self) -> Option<u64> {
        None
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        GbApu::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for GbApu {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(GbApu::from_props(props)?)))
}

/// What the validator should know about `gb.apu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("record", ValueKind::Bool))
        .port(DIV_APU_PIN, PortDir::In)
        .region(REGISTER_REGION)
}
