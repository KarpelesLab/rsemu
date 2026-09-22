//! The Macintosh's sound circuit: one pulse-width byte a scan line, read out
//! of a buffer in main memory, gated by the VIA and attenuated by it.
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition:
//!
//! * The "Sound" chapter: the Macintosh Plus drives its speaker from a
//!   **pulse-width modulator** fed one byte per horizontal scan line out of a
//!   buffer in main memory, so a buffer is **370 bytes** — one per line of the
//!   370-line raster — and the rate is the horizontal line rate.
//! * The same chapter, and chapter 3's memory map: the buffer holds 370
//!   **words**, and the sound byte is the **high-order byte** of each; the
//!   low-order byte is the 400K drive's speed. The two are interleaved because
//!   one DMA reads both.
//! * Chapter 3's memory map: the main sound buffer starts `$0300` below the top
//!   of installed memory and the alternate one `$5F00` below it, beside the two
//!   screen buffers at `$5900` and `$D900`.
//! * The VIA port-assignment tables: port A bits **0-2** are the sound volume,
//!   port A bit **3** is `SNDPG2` — low selects the alternate buffer — and port
//!   B bit **7** is `SNDENB`, **low** enabling the sound.
//!
//! No emulator source was consulted and the ROM was not disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).
//!
//! # The rate, exactly
//!
//! One sample a scan line, and `mac.video` says what a scan line is: 704 dot
//! clocks of the board's 15.6672 MHz crystal. So
//!
//! ```text
//!   15 667 200 / 704 = 244 800 / 11 = 22 254.5454… Hz
//! ```
//!
//! which is not a whole number of hertz, and is exactly what
//! [`StreamInfo`](crate::host::audio::StreamInfo)'s rational is for. **One tick
//! of this device's clock is one sample**: `machines/mac-plus.machine` gives it
//! `clk / 704`, so the ratio is an exact division inside the board's one
//! oscillator tree rather than a rate written out in hertz and rounded
//! (`CLAUDE.md`, Determinism). 370 samples is one frame, which is why
//! `ticks % 370` is the index into the buffer and why a waveform whose period
//! divides 370 comes out seamless across a frame boundary — which is how the
//! ROM's startup chime is built.
//!
//! # What a byte is worth
//!
//! The byte is a **duty cycle**, not a signed sample: `$00` is the narrowest
//! pulse and `$FF` the widest, and the low-pass between the modulator and the
//! speaker turns that into a level. Silence is therefore the middle, and the
//! ROM says so — after its startup chime it fills all 370 words with `$80` and
//! leaves them there (measured, `docs/platforms/mac-plus.md`). So a sample is
//! `(byte − 128) × 256`, which puts `$00` at [`i16::MIN`] and `$FF` at 32 512.
//!
//! # The volume is linear in the setting, and that is a convention
//!
//! The Guide gives port A's three bits as "the sound volume" and does not give
//! the ladder they drive, so the attenuation per step is not a documented
//! number. This model scales by `volume / 7`, which makes **0 silence** — a
//! Macintosh really is muted at volume 0, which is what the Sound control panel
//! does — and 7 full scale. `host::audio::gb` set the precedent for saying so
//! rather than inventing a measurement.
//!
//! # Nothing is produced unless somebody is listening
//!
//! `record` is off by default and [`Sound::set_recording`] turns it on, exactly
//! as `amiga.paula` does, and for one more reason than Paula has: **this device
//! names an event per sample**. A byte has to be read at the tick the line
//! scanned it, because the ROM rewrites the whole buffer once a frame, and a
//! round boundary that fell anywhere else would read some of a frame's samples
//! out of the next frame's waveform — the defect `mac.iwm` had and
//! `docs/platforms/mac-plus.md` records. 22 254 scheduler rounds a second is
//! the price of that, and a machine nobody is recording pays none of it:
//! [`Device::next_event_tick`] is `None` and the circuit simply skips forward.
//!
//! # No analogue stage
//!
//! The reconstruction filter between the modulator and the speaker is on the
//! board and the Guide gives no corner frequency for it, so
//! [`StreamInfo::output_stage`](crate::host::audio::StreamInfo::output_stage)
//! is empty. A corner nobody measured would be an invented measurement.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AddressSpace, MemAttrs, RegionRef, RequesterId, UnassignedPolicy};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.sound";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// Samples in one buffer: one per scan line of the 370-line raster.
pub const SAMPLES_PER_FRAME: u64 = 370;

/// How far below the top of memory the main sound buffer starts.
pub const MAIN_OFFSET: u64 = 0x0300;
/// How far below the top of memory the alternate sound buffer starts.
pub const ALT_OFFSET: u64 = 0x5f00;

/// The value a sample of silence holds: the middle of the duty cycle.
pub const SILENCE: u8 = 0x80;

/// How many samples the output ring holds before the oldest are dropped.
///
/// A third of a second at 22 254 Hz, which is the depth `amiga.paula` settled
/// on for the same reason: the binary drains every ten milliseconds of virtual
/// time, so this is thirty times the deepest backlog a slice can leave.
pub const RING_SAMPLES: usize = 1 << 13;

/// The input pin the VIA's `PB7` drives: **low** enables the sound.
const SNDENB_PIN: &str = "sndenb";
/// The input pins the VIA's `PA0`-`PA2` drive: the volume, bit 0 first.
const VOLUME_PINS: [&str; 3] = ["snd0", "snd1", "snd2"];
/// The input pin the VIA's `PA3` drives: low selects the alternate buffer.
const SNDPG2_PIN: &str = "sndpg2";

/// `next_event` when there is nothing to wake up for.
const NO_EVENT: u64 = u64::MAX;

/// Which wire arrived, as a line number on the sink pin.
const LINE_SNDENB: u32 = 0;
const LINE_VOL0: u32 = 1;
const LINE_SNDPG2: u32 = 4;

/// One sample as the seam's unit, given the byte and the volume setting.
///
/// See *What a byte is worth* and *The volume is linear in the setting*.
/// Integer throughout: no float ever touches a sample here.
#[inline]
#[must_use]
pub const fn sample(byte: u8, volume: u8) -> i16 {
    let level = (byte as i32 - SILENCE as i32) * 256;
    let scaled = level * (volume & 7) as i32 / 7;
    // The widest pulse is 32 512 and the narrowest is exactly `i16::MIN`, so
    // the clamp can only bite if `volume` were ever out of range. Belt and
    // braces, as `host::audio::amiga`'s is.
    if scaled < i16::MIN as i32 {
        i16::MIN
    } else if scaled > i16::MAX as i32 {
        i16::MAX
    } else {
        scaled as i16
    }
}

/// What the pins say and where the sampler has got to.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Samples taken since reset: one per tick of this device's clock.
    ticks: u64,
    /// `SNDENB` as the VIA drives it. High — the idle pull-up — is *disabled*.
    sndenb: bool,
    /// The three volume pins, bit 0 first.
    volume: u8,
    /// `SNDPG2`. High selects the main buffer.
    sndpg2: bool,
    /// Whether samples are being kept for a host.
    record: bool,
    /// The output ring, oldest first. Derived state: never snapshotted.
    out: VecDeque<i16>,
    /// Samples the ring dropped because nobody drained it. Diagnostic.
    dropped: u64,
}

impl State {
    fn fresh(ticks: u64) -> State {
        State {
            ticks,
            // The pins idle at their pull-ups until the VIA drives them, which
            // is a muted speaker and the main buffer — what a Macintosh does
            // with port A and port B still all inputs.
            sndenb: true,
            volume: 7,
            sndpg2: true,
            record: false,
            out: VecDeque::new(),
            dropped: 0,
        }
    }

    /// Where the buffer being played starts, relative to the top of memory.
    const fn offset(&self) -> u64 {
        if self.sndpg2 { MAIN_OFFSET } else { ALT_OFFSET }
    }

    fn push(&mut self, value: i16) {
        if self.out.len() >= RING_SAMPLES {
            // Keep the newest, as Paula does: a host this far behind wants the
            // sound still coming rather than the history it already missed.
            self.out.pop_front();
            self.dropped += 1;
        }
        self.out.push_back(value);
    }
}

/// Everything the device and its host adapter share.
struct Shared {
    state: Mutex<State>,
    /// Main memory, in a private space of this object's own at base zero.
    bus: Mutex<Option<Arc<AddressSpace>>>,
    /// How many bytes of it there are — the top the buffers hang below.
    ram_len: AtomicU64,
    /// The requester the reads carry.
    requester: Mutex<RequesterId>,
    /// Published without a lock, for the scheduler.
    ticks: AtomicU64,
    next_event: AtomicU64,
    /// The catch-up handle a drain syncs through (`ROADMAP.md` §4.2).
    lazy: Mutex<Option<LazyHandle>>,
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Sound");
        s.field("ram", &self.ram_len.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s
                .field("ticks", &state.ticks)
                .field("sndenb", &state.sndenb)
                .field("volume", &state.volume)
                .field("sndpg2", &state.sndpg2)
                .field("queued", &state.out.len())
                .finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        // One event per sample while recording, and none at all otherwise:
        // the circuit drives no wire, so a machine nobody is listening to has
        // nothing this device could be late for.
        self.next_event.store(
            if state.record {
                state.ticks + 1
            } else {
                NO_EVENT
            },
            Ordering::Relaxed,
        );
    }

    /// The byte the line beginning at `tick` reads.
    ///
    /// The reads carry `MemAttrs::debug` for the reason `mac.video`'s do: a
    /// bus master walking main memory must not pop a FIFO or advance anybody's
    /// pointer, and on this board the only thing behind the private space is
    /// the DRAM region itself.
    fn byte_at(&self, offset: u64, tick: u64) -> u8 {
        let Some(bus) = self.bus.lock().clone() else {
            return SILENCE;
        };
        let top = self.ram_len.load(Ordering::Relaxed);
        // The sound byte is the high-order byte of word `n`, and a 68000 board
        // is big-endian, so it is the even address.
        let at = top.saturating_sub(offset) + (tick % SAMPLES_PER_FRAME) * 2;
        let attrs = MemAttrs::DEBUG.with_requester(*self.requester.lock());
        let mut byte = [0u8; 1];
        if bus.read_bytes(at, &mut byte, attrs).is_err() {
            return SILENCE;
        }
        byte[0]
    }

    fn advance_to(&self, target: u64) {
        loop {
            // Read outside the lock: `byte_at` goes through the address space,
            // and holding this device's lock across it would be a call into
            // another device under our own (`CLAUDE.md`, re-entrancy).
            let next = {
                let mut state = self.state.lock();
                if target <= state.ticks {
                    return;
                }
                if !state.record {
                    state.ticks = target;
                    self.publish(&state);
                    return;
                }
                (state.ticks, state.offset(), state.sndenb, state.volume)
            };
            let (tick, offset, muted, volume) = next;
            // A disabled modulator drives nothing at the speaker, so the level
            // is the rest level however loud the volume bits say. Not reading
            // at all while muted also keeps the circuit off the bus, which is
            // what `SNDENB` low really does.
            let value = if muted {
                0
            } else {
                sample(self.byte_at(offset, tick), volume)
            };
            let mut state = self.state.lock();
            state.ticks = tick + 1;
            state.push(value);
            self.publish(&state);
        }
    }

    /// Bring the circuit up to date before a drain.
    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
    }
}

/// The sound circuit, as a host adapter sees it.
///
/// A separate type for the reason `mac.video`'s [`Screen`](super::video::Screen)
/// is one: `Device` keeps `Any` out of its supertrait chain, so a host takes
/// its handle at construction.
#[derive(Debug)]
pub struct Speaker {
    shared: Arc<Shared>,
}

impl Speaker {
    /// Drain the output ring: samples at the horizontal line rate, oldest
    /// first, in the seam's signed 16-bit unit.
    #[must_use]
    pub fn take_audio(&self) -> Vec<i16> {
        self.shared.sync();
        self.shared.state.lock().out.drain(..).collect()
    }

    /// How many samples the ring dropped because nobody drained it.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.shared.state.lock().dropped
    }

    /// Whether samples are being kept.
    #[must_use]
    pub fn recording(&self) -> bool {
        self.shared.state.lock().record
    }

    /// Samples taken since reset, which is also ticks of the device's clock.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }
}

/// The five input pins, on one sink.
#[derive(Debug)]
struct SoundPin {
    shared: Arc<Shared>,
    line: u32,
    inputs: FanIn,
}

impl WireSink for SoundPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        let mut state = self.shared.state.lock();
        match self.line {
            LINE_SNDENB => state.sndenb = high,
            LINE_SNDPG2 => state.sndpg2 = high,
            bit => {
                let mask = 1u8 << (bit - LINE_VOL0);
                if high {
                    state.volume |= mask;
                } else {
                    state.volume &= !mask;
                }
            }
        }
    }
}

/// The Macintosh sound circuit.
#[derive(Debug)]
pub struct Sound {
    shared: Arc<Shared>,
    speaker: Arc<Speaker>,
    /// The object named by `ram`, resolved at bind.
    ram_path: String,
    /// The pins, kept alive here: a net holds only a `Weak` to its sinks.
    pins: Mutex<Vec<Arc<SoundPin>>>,
}

impl Sound {
    /// Build the sound circuit.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `ram` is missing or of the wrong kind, or if a
    /// property this class does not know was given.
    pub fn new(props: &Props) -> Result<Sound> {
        let mut r = props.reader();
        let ram_path = r.require_link("ram")?.as_str().to_string();
        let record = r.or("record", false)?;
        r.finish()?;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(0)),
            bus: Mutex::with_rank(LockRank::LEAF, None),
            ram_len: AtomicU64::new(0),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId::ANONYMOUS),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        {
            let mut state = shared.state.lock();
            state.record = record;
            shared.publish(&state);
        }
        let speaker = Arc::new(Speaker {
            shared: Arc::clone(&shared),
        });
        Ok(Sound {
            shared,
            speaker,
            ram_path,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    /// The handle a host adapter keeps.
    #[must_use]
    pub fn speaker(&self) -> Arc<Speaker> {
        Arc::clone(&self.speaker)
    }

    /// Start or stop keeping samples, discarding anything already kept.
    ///
    /// Nothing guest-visible: the ring is output, it is absent from the
    /// snapshot, and the circuit has no registers at all.
    pub fn set_recording(&self, on: bool) {
        let mut state = self.shared.state.lock();
        state.record = on;
        state.out.clear();
        self.shared.publish(&state);
    }

    /// Point the circuit at main memory, once the machine layer has resolved
    /// it.
    ///
    /// # Errors
    ///
    /// If the region cannot be mapped into the private space, or if it is too
    /// small to hold a sound buffer where a Macintosh puts one.
    pub fn attach(&self, ram: &RegionRef, bits: u32) -> Result<()> {
        if ram.len() < ALT_OFFSET {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "`ram` holds {:#x} bytes and the alternate sound buffer starts \
                     {ALT_OFFSET:#x} below the top of memory",
                    ram.len()
                ),
            });
        }
        let space = AddressSpace::new(format!("{CLASS_NAME}.ram"), bits)
            .with_unassigned(UnassignedPolicy::OPEN_BUS);
        space.topology().map(Arc::clone(ram), 0)?;
        self.shared.ram_len.store(ram.len(), Ordering::Relaxed);
        *self.shared.bus.lock() = Some(Arc::new(space));
        Ok(())
    }

    /// Run the circuit until `target` samples have been taken in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Set the level the VIA is driving onto a pin, for a test with no wire
    /// graph. `SNDENB` is low to enable.
    pub fn set_sndenb(&self, high: bool) {
        self.shared.state.lock().sndenb = high;
    }

    /// The same for the three volume bits, as a value of 0 to 7.
    pub fn set_volume(&self, volume: u8) {
        self.shared.state.lock().volume = volume & 7;
    }

    /// The same for `SNDPG2`: high is the main buffer.
    pub fn set_sndpg2(&self, high: bool) {
        self.shared.state.lock().sndpg2 = high;
    }
}

impl Device for Sound {
    fn class(&self) -> &'static DeviceClass {
        &SOUND_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: this circuit has no registers and drives no wire.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let mut state = self.shared.state.lock();
        let (record, ticks) = (state.record, state.ticks);
        // What the VIA is driving is the VIA's and survives our reset.
        let (sndenb, volume, sndpg2) = (state.sndenb, state.volume, state.sndpg2);
        *state = State::fresh(ticks);
        state.record = record;
        state.sndenb = sndenb;
        state.volume = volume;
        state.sndpg2 = sndpg2;
        self.shared.publish(&state);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Where the sampler is and what the pins say. The ring is output
        // rather than architectural state and is deliberately absent (§4.5),
        // as is `record`, which belongs to whoever is listening.
        let state = self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_bool(state.sndenb)?;
        w.write_u8(state.volume)?;
        w.write_bool(state.sndpg2)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        state.ticks = r.read_u64()?;
        state.sndenb = r.read_bool()?;
        state.volume = r.read_u8()? & 7;
        state.sndpg2 = r.read_bool()?;
        // Derived state, thrown away rather than restored.
        state.out.clear();
        self.shared.publish(&state);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            SNDENB_PIN => LINE_SNDENB,
            SNDPG2_PIN => LINE_SNDPG2,
            _ => LINE_VOL0 + VOLUME_PINS.iter().position(|p| *p == port)? as u32,
        };
        let pin = Arc::new(SoundPin {
            shared: Arc::clone(&self.shared),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes, and it names an event per sample while it is recording. See the
    /// module docs: a byte read at a round boundary rather than at the tick the
    /// line scanned it is the defect `mac.iwm` had.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Sound::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Sound {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bits = ctx.space().map_or(32, |s| s.bits());
        let ram = ctx.region(&self.ram_path, "").map_err(|e| Error::Config {
            at: ctx.path().to_string(),
            message: format!("`ram` has to name the memory the buffer is in: {e}"),
        })?;
        *self.shared.requester.lock() = ctx.requester();
        self.attach(&ram, bits)
    }
}

/// The `mac.sound` device class.
pub static SOUND_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Macintosh sound circuit: one pulse-width byte a scan line out of main memory, \
              gated and attenuated by the VIA",
    properties: &[
        PropertySpec {
            name: "ram",
            kind: ValueKind::Link,
            required: true,
            summary: "main memory, whose top the sound buffers hang below",
        },
        PropertySpec {
            name: "record",
            kind: ValueKind::Bool,
            required: false,
            summary: "keep samples for a host to drain (default false)",
        },
    ],
    construct: |props| Ok(Box::new(Sound::new(props)?)),
};

/// Add [`SOUND_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SOUND_CLASS)
}

/// Bind [`SOUND_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sound::new(props)?)))
}

/// What the validator should know about `mac.sound`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("ram", ValueKind::Link).required())
        .prop(PropSchema::new("record", ValueKind::Bool))
        .port(SNDENB_PIN, PortDir::In)
        .port(SNDPG2_PIN, PortDir::In);
    for pin in VOLUME_PINS {
        schema = schema.port(pin, PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests;
