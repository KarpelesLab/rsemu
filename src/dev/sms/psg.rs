//! The SN76489 programmable sound generator.
//!
//! Three square-wave channels and one noise channel, each with its own four-bit
//! attenuator, reached through **one write-only port**. Texas Instruments' part;
//! Sega integrated it into the Master System's VDP package, which is why it has
//! no address lines of its own — the board decodes `$40`-`$7F` and every write
//! in that range lands here.
//!
//! ```text
//!   1 c c t d d d d    latch: channel cc, type t (0 tone, 1 volume), low 4 bits
//!   0 x d d d d d d    data: six more bits, into whatever was last latched
//! ```
//!
//! A tone register is ten bits and takes both writes; a volume register is four
//! and is complete after the first. So `$8E $3F` sets channel 0's period, and
//! `$9F` silences it — which is why every driver's first act is four `$9F`-style
//! writes.
//!
//! # The counters
//!
//! The chip divides its input clock by 16 and runs four counters off the result.
//! A tone counter reloads from its ten-bit register and **toggles an output
//! flip-flop** each time it underflows, so the frequency is `clock / (32 n)`.
//! A register of zero is treated as one: the counter never sits still, and the
//! resulting 100-odd kHz is silence in practice rather than a divide by zero.
//!
//! The noise channel shifts a 16-bit register instead of toggling. Its tap
//! configuration is what distinguishes the "white" setting from the "periodic"
//! one: white feeds back the parity of two taps and produces the hiss, periodic
//! feeds back one bit and produces a buzzing tone at a fifteenth of the rate.
//! Writing the noise control register **resets the shift register** to its seed,
//! which is what makes a drum hit start the same way every time.
//!
//! # Volume is attenuation
//!
//! Zero is loudest and fifteen is off, in nominal 2 dB steps. [`VOLUME`] is that
//! curve as linear amplitudes, so the mixer is a sum of integers and there is no
//! float anywhere in the time path (`CLAUDE.md`).
//!
//! # Time
//!
//! **Lazily advanced** on the CPU's own domain — the PSG takes the same
//! 3.58 MHz — and caught up before every write, so a driver that changes a
//! register mid-line changes it at the right sample. Samples are taken every
//! [`SAMPLE_DIVISOR`] input ticks, which is exactly five of the chip's internal
//! steps: the output rate is an exact ratio of the chip's own counter clock
//! rather than nearly one (`ROADMAP.md` §4.2).
//!
//! [`SmsPsg::take_samples`] drains the ring; a machine with no audio sink never
//! drains it, and the oldest frames are dropped.
//!
//! # What is not modelled
//!
//! * **The Game Gear's stereo port** (`$06`), which pans each channel left or
//!   right. A Game Gear is a different machine file and this is where the
//!   difference would land.
//! * **The YM2413 FM board**, an accessory on Japanese machines.
//! * The analogue output stage: no filter, no clipping curve, no DC offset.
//!
//! # Sources
//!
//! [SMS Power!'s development documents](https://www.smspower.org/Development/Documents)
//! — the SN76489 description, the register format, the noise tap and the
//! volume table — and the SN76489 datasheet. No emulator source of any licence
//! was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;

/// How many input ticks the chip's internal counters take for one step.
pub const CLOCK_DIVIDER: u64 = 16;

/// How many input ticks there are between output samples.
///
/// Exactly five internal steps. At the Master System's 3.579545 MHz that is
/// about 44.7 kHz on an NTSC machine and 44.3 kHz on a PAL one — the *rate*
/// differs between regions because the crystal does, which is the honest answer
/// rather than resampling one to the other inside a device.
pub const SAMPLE_DIVISOR: u64 = CLOCK_DIVIDER * 5;

/// How many stereo frames the output ring holds before the oldest are dropped.
pub const RING_FRAMES: usize = 16_384;

/// The name a `map` statement reaches the write port by.
pub const PORT_REGION: &str = "port";

/// The four-bit attenuator as linear amplitudes.
///
/// Nominal 2 dB a step, 0 loudest and 15 silent. Integers, because
/// `ROADMAP.md` §4.2 keeps floating point out of the time path and a volume
/// table is squarely in it.
///
/// Source: the SN76489 datasheet's attenuation steps.
pub const VOLUME: [i16; 16] = [
    8191, 6507, 5168, 4105, 3261, 2590, 2057, 1642, 1298, 1031, 819, 650, 516, 410, 326, 0,
];

/// The shift register's power-on seed.
const LFSR_SEED: u16 = 0x8000;

/// The white-noise feedback taps: bits 0 and 3.
///
/// Source: SMS Power!, the PSG documentation. The tap pair is the one thing
/// that differs between SN76489 variants, and this is the Master System's.
const WHITE_TAPS: u16 = 0x0009;

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct Engine {
    /// Ten-bit period for channels 0-2; three-bit noise control for 3.
    tone: [u16; 4],
    /// Four-bit attenuation, one a channel.
    volume: [u8; 4],
    /// Down-counters, one a channel.
    counter: [u16; 4],
    /// The three tone flip-flops.
    output: [bool; 3],
    /// The noise shift register.
    lfsr: u16,
    /// Which register a bare data byte completes: `channel << 1 | type`.
    latched: u8,
    /// Input ticks until the next internal step.
    divider: u64,
    /// Input ticks until the next output sample.
    sample_timer: u64,
    /// Ticks executed since reset. The device's own clock.
    ticks: u64,
}

impl Default for Engine {
    fn default() -> Engine {
        Engine {
            tone: [0; 4],
            // Silent at power-on is a choice, and the right one: the part comes
            // up with undefined attenuators and every driver writes all four
            // before it writes anything else, so the alternative is a burst of
            // noise on reset that no real machine makes audible.
            volume: [0x0f; 4],
            counter: [0; 4],
            output: [false; 3],
            lfsr: LFSR_SEED,
            latched: 0,
            divider: CLOCK_DIVIDER,
            sample_timer: SAMPLE_DIVISOR,
            ticks: 0,
        }
    }
}

impl Engine {
    /// A byte written to the chip's one port.
    fn write(&mut self, value: u8) {
        if value & 0x80 != 0 {
            self.latched = (value >> 4) & 0x07;
            let channel = (self.latched >> 1) as usize;
            if self.latched & 1 != 0 {
                self.volume[channel] = value & 0x0f;
            } else if channel == 3 {
                self.write_noise_control(value & 0x0f);
            } else {
                self.tone[channel] = (self.tone[channel] & 0x3f0) | u16::from(value & 0x0f);
            }
            return;
        }
        let channel = (self.latched >> 1) as usize;
        if self.latched & 1 != 0 {
            // A volume register is four bits wide; the second write is not
            // "the high bits", it simply replaces it.
            self.volume[channel] = value & 0x0f;
        } else if channel == 3 {
            self.write_noise_control(value & 0x0f);
        } else {
            self.tone[channel] = (self.tone[channel] & 0x00f) | (u16::from(value & 0x3f) << 4);
        }
    }

    /// The noise control register, and the reset that comes with it.
    fn write_noise_control(&mut self, value: u8) {
        self.tone[3] = u16::from(value & 0x07);
        self.lfsr = LFSR_SEED;
    }

    /// The reload value for a tone counter. Zero behaves as one.
    fn period(&self, channel: usize) -> u16 {
        match self.tone[channel] & 0x3ff {
            0 => 1,
            n => n,
        }
    }

    /// The reload value for the noise counter.
    ///
    /// The two low control bits pick 16, 32 or 64 internal steps; the third
    /// setting borrows channel 2's period, which is how a driver sweeps the
    /// noise pitch without a register of its own.
    fn noise_period(&self) -> u16 {
        match self.tone[3] & 0x03 {
            0 => 0x10,
            1 => 0x20,
            2 => 0x40,
            _ => self.period(2),
        }
    }

    /// One internal step: every counter, and the flip-flops they drive.
    fn step(&mut self) {
        for channel in 0..3 {
            if self.counter[channel] == 0 {
                self.counter[channel] = self.period(channel);
                self.output[channel] = !self.output[channel];
            }
            self.counter[channel] -= 1;
        }
        if self.counter[3] == 0 {
            self.counter[3] = self.noise_period();
            let feedback = if self.tone[3] & 0x04 != 0 {
                // White: the parity of the tapped bits.
                (self.lfsr & WHITE_TAPS).count_ones() as u16 & 1
            } else {
                // Periodic: one bit, so the register cycles in fifteen steps.
                self.lfsr & 1
            };
            self.lfsr = (self.lfsr >> 1) | (feedback << 15);
        }
        self.counter[3] -= 1;
    }

    /// The mixed output, as one signed sample.
    ///
    /// Bipolar rather than 0-to-peak: a channel at full volume with its output
    /// low would otherwise contribute a constant offset, and four of those is a
    /// thump every time a driver keys a note.
    fn sample(&self) -> i16 {
        let mut sum = 0i32;
        for channel in 0..3 {
            let level = i32::from(VOLUME[self.volume[channel] as usize]);
            sum += if self.output[channel] { level } else { -level };
        }
        let level = i32::from(VOLUME[self.volume[3] as usize]);
        sum += if self.lfsr & 1 != 0 { level } else { -level };
        // Four channels of 8191 cannot overflow an i32; the divide keeps the
        // sum inside an i16 with the same headroom every time.
        (sum / 4) as i16
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

struct Shared {
    engine: Mutex<Engine>,
    lazy: Mutex<Option<LazyHandle>>,
    ring: Mutex<VecDeque<(i16, i16)>>,
    /// Whether output frames are kept at all. A machine with no audio sink pays
    /// nothing for the ring it never drains.
    recording: AtomicBool,
    /// [`Engine::ticks`], republished lock-free for the scheduler.
    ticks: AtomicU64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("engine", &self.engine)
            .finish_non_exhaustive()
    }
}

impl Shared {
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

/// The Master System's sound chip.
pub struct SmsPsg {
    shared: Arc<Shared>,
    port_region: RegionRef,
}

impl fmt::Debug for SmsPsg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SmsPsg")
            .field("engine", &self.shared.engine)
            .finish_non_exhaustive()
    }
}

impl Default for SmsPsg {
    fn default() -> Self {
        SmsPsg::new()
    }
}

impl SmsPsg {
    /// A chip in its power-on state, with every channel silent.
    #[must_use]
    pub fn new() -> SmsPsg {
        let shared = Arc::new(Shared {
            engine: Mutex::with_rank(LockRank::DEVICE, Engine::default()),
            lazy: Mutex::new(None),
            ring: Mutex::with_rank(LockRank::LEAF, VecDeque::new()),
            recording: AtomicBool::new(false),
            ticks: AtomicU64::new(0),
        });
        // Two bytes, not one. The board decodes A7 and A6 and nothing finer, so
        // every write in `$40`-`$7F` reaches the chip — and a two-byte aperture
        // is what lets it be `split()` against the VDP's two counters, which
        // occupy the read half of the same pair.
        let port_region = Arc::new(MmioRegion::io(
            "sms.psg.port",
            2,
            Arc::new(PsgPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        SmsPsg {
            shared,
            port_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If an unknown property was given.
    pub fn from_props(props: &Props) -> Result<SmsPsg> {
        let mut r = props.reader();
        let record = r.or("record", false)?;
        r.finish()?;
        let psg = SmsPsg::new();
        psg.set_recording(record);
        Ok(psg)
    }

    /// Whether output frames are kept for [`take_samples`](SmsPsg::take_samples).
    pub fn set_recording(&self, on: bool) {
        self.shared.recording.store(on, Ordering::Relaxed);
    }

    /// Whether output frames are being kept.
    #[must_use]
    pub fn recording(&self) -> bool {
        self.shared.recording.load(Ordering::Relaxed)
    }

    /// Connect the catch-up handle the port syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Drain every queued output frame.
    #[must_use]
    pub fn take_samples(&self) -> alloc::vec::Vec<(i16, i16)> {
        self.shared.ring.lock().drain(..).collect()
    }

    /// How many frames are queued.
    #[must_use]
    pub fn queued_samples(&self) -> usize {
        self.shared.ring.lock().len()
    }

    /// Ticks executed since reset.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Write one byte to the chip's port, as the guest would.
    pub fn write(&self, value: u8) {
        self.shared.engine.lock().write(value);
    }

    /// One channel's ten-bit period register, or the noise control for 3.
    #[must_use]
    pub fn tone(&self, channel: usize) -> u16 {
        self.shared.engine.lock().tone[channel & 3]
    }

    /// One channel's four-bit attenuation. Zero is loudest.
    #[must_use]
    pub fn volume(&self, channel: usize) -> u8 {
        self.shared.engine.lock().volume[channel & 3]
    }

    /// The noise shift register.
    #[must_use]
    pub fn lfsr(&self) -> u16 {
        self.shared.engine.lock().lfsr
    }

    /// The mixed output right now, without advancing anything.
    #[must_use]
    pub fn sample(&self) -> i16 {
        self.shared.engine.lock().sample()
    }

    /// Run the chip to absolute tick `target`.
    pub fn advance_to(&self, target: u64) {
        let mut engine = self.shared.engine.lock();
        if target <= engine.ticks {
            return;
        }
        let recording = self.shared.recording.load(Ordering::Relaxed);
        while engine.ticks < target {
            // Step to whichever comes first: the next internal counter step or
            // the next output sample. Both are tens of ticks away, so this walks
            // the interval in a handful of iterations rather than one a tick.
            let step = (target - engine.ticks)
                .min(engine.divider)
                .min(engine.sample_timer);
            engine.ticks += step;
            engine.divider -= step;
            engine.sample_timer -= step;
            if engine.divider == 0 {
                engine.divider = CLOCK_DIVIDER;
                engine.step();
            }
            if engine.sample_timer == 0 {
                engine.sample_timer = SAMPLE_DIVISOR;
                if recording {
                    let value = engine.sample();
                    let mut ring = self.shared.ring.lock();
                    if ring.len() >= RING_FRAMES {
                        ring.pop_front();
                    }
                    // Mono chip, stereo seam: the Game Gear's pan register is
                    // where a difference between the two would come from, and
                    // it is not modelled.
                    ring.push_back((value, value));
                }
            }
        }
        self.shared.ticks.store(engine.ticks, Ordering::Relaxed);
    }

    /// Run the chip forward by `ticks`.
    pub fn advance_by(&self, ticks: u64) {
        let target = self.shared.ticks.load(Ordering::Relaxed) + ticks;
        self.advance_to(target);
    }
}

/// The chip's one write port.
struct PsgPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for PsgPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PsgPort").finish_non_exhaustive()
    }
}

impl MemOps for PsgPort {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Write-only. A board maps the read side of these addresses to the VDP's
        // counters with `split()`, so this is only reached when it did not — and
        // then the data bus floats, which a Z80 board pulls up.
        *byte = 0xff;
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would key a note nobody asked for.
            return Ok(());
        }
        self.shared.sync(attrs);
        self.shared.engine.lock().write(*value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `sms.psg` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "sms.psg",
    version: 1,
    summary: "SN76489 PSG: three square channels, one noise channel, one write port",
    properties: &[PropertySpec {
        name: "record",
        kind: ValueKind::Bool,
        required: false,
        summary: "keep output frames in a ring for a host audio sink to drain",
    }],
    construct: |props| Ok(Box::new(SmsPsg::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SmsPsg {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: no pins, and the port answers from the region it
        // published before realize ran.
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == PORT_REGION).then(|| Arc::clone(&self.port_region))
    }

    fn reset(&self, _kind: ResetKind) {
        let mut engine = self.shared.engine.lock();
        let ticks = engine.ticks;
        *engine = Engine::default();
        // The device's clock does not restart: the scheduler owns it, and a
        // device that rewound its own tick would be told to advance backwards.
        engine.ticks = ticks;
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let engine = self.shared.engine.lock();
        w.write_u32(STATE_VERSION)?;
        for value in engine.tone {
            w.write_u16(value)?;
        }
        for value in engine.volume {
            w.write_u8(value)?;
        }
        for value in engine.counter {
            w.write_u16(value)?;
        }
        for value in engine.output {
            w.write_bool(value)?;
        }
        w.write_u16(engine.lfsr)?;
        w.write_u8(engine.latched)?;
        w.write_u64(engine.divider)?;
        w.write_u64(engine.sample_timer)?;
        w.write_u64(engine.ticks)?;
        // The output ring is *not* saved. It is derived state on its way to a
        // host sink, and a snapshot that carried it would replay audio that has
        // already been heard (`CLAUDE.md`).
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the PSG's snapshot is version {version}, this build writes {STATE_VERSION}"
            )));
        }
        let mut engine = self.shared.engine.lock();
        for value in &mut engine.tone {
            *value = r.read_u16()?;
        }
        for value in &mut engine.volume {
            *value = r.read_u8()?;
        }
        for value in &mut engine.counter {
            *value = r.read_u16()?;
        }
        for value in &mut engine.output {
            *value = r.read_bool()?;
        }
        engine.lfsr = r.read_u16()?;
        engine.latched = r.read_u8()?;
        engine.divider = r.read_u64()?;
        engine.sample_timer = r.read_u64()?;
        engine.ticks = r.read_u64()?;
        self.shared.ticks.store(engine.ticks, Ordering::Relaxed);
        Ok(())
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        SmsPsg::advance_to(self, tick);
    }

    /// **None**, deliberately.
    ///
    /// `next_event_tick` bounds the scheduler's quantum, and this chip's next
    /// counter step is a few dozen ticks away *always*. Reporting it would pin
    /// the quantum to that and cost far more than it buys: nothing the guest can
    /// read changes when a flip-flop toggles, and the port catches the chip up
    /// before every write. The same reasoning as the Game Boy's sound unit.
    fn next_event_tick(&self) -> Option<u64> {
        None
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        SmsPsg::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for SmsPsg {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(SmsPsg::from_props(props)?)))
}

/// What the validator should know about `sms.psg`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("record", ValueKind::Bool))
        .region(PORT_REGION)
}
