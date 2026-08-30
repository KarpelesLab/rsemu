//! The NES APU — the audio half of the RP2A03 / RP2A07.
//!
//! Five channels (two pulse, triangle, noise, DMC), a frame counter that clocks
//! their modulators and can raise an IRQ, and a non-linear mixer. Written from
//! the [NESdev wiki](https://www.nesdev.org/wiki/APU) — see the per-module
//! source citations — and from nothing else; `CLAUDE.md`'s provenance rule
//! forbids the obvious alternatives.
//!
//! # Where the time comes from
//!
//! The APU is clocked by the CPU clock, which on the NES is the master
//! oscillator divided by 12. It is therefore a domain in the machine's
//! oscillator forest ([`crate::core::clock`]) and never reads a host clock. The
//! device is **lazily advanced** (`ROADMAP.md` §4.2): it holds a tick counter
//! and the machine calls [`Apu::advance_to`] before dispatching any access to
//! it, so a `$4015` read taken in the middle of a long CPU budget still sees
//! the frame counter at exactly the right cycle.
//!
//! Everything internal counts **CPU cycles**. An APU cycle is two of them — the
//! first is a *get* cycle, the second a *put* cycle — and the units that run at
//! APU rate (both pulse timers, the noise timer, the DMC timer) are clocked on
//! the get half. The triangle's timer runs on every CPU cycle. The reason for
//! choosing CPU cycles as the unit is set out in [`frame`].
//!
//! # What this module does not own
//!
//! The DMC's sample fetch stalls the CPU for 1–4 cycles, and the exact halt /
//! dummy / alignment / get choreography — including its interaction with OAM
//! DMA and the documented abort bugs — belongs to the CPU, which is the device
//! that is actually halted. The APU's side of that contract is
//! [`Apu::dma_request`], [`Apu::dma_is_pending`] and [`Apu::dma_complete`]; see
//! [`dmc::DmaRequest`].
//!
//! Resampling is likewise not here. [`Apu::take_samples`] hands out Q16 samples
//! at the APU's own rate and the host layer decides what to do with them
//! (`ROADMAP.md` §15, invariant 4).
//!
//! # Deliberately not modelled
//!
//! - The RC filter chain after the DACs (90 Hz and 440 Hz high-pass, 14 kHz
//!   low-pass). It is a host-side filter over the sample stream, and the
//!   Famicom's is different again.
//! - The pre-mode-flag 2A03 revisions, whose noise channel had no short mode
//!   and whose rate `$F` lasted 2046 cycles.
//! - The `$4011` write-versus-timer-clock race, which has no documented rule.
//! - The `RP2A03H` "unexpected reload DMA" bug. The aborted-DMA case *is*
//!   reachable, because withdrawing a scheduled fetch is exactly what
//!   [`Apu::dma_is_pending`] reports.
//!
//! # Example
//!
//! ```
//! use rsemu::core::props::Props;
//! use rsemu::dev::apu::Apu;
//!
//! let apu = Apu::new(&Props::new()).unwrap();
//! apu.write(0x15, 0x0F); // enable the four waveform channels
//! apu.write(0x00, 0x9F); // pulse 1: 50% duty, halt, constant volume 15
//! apu.write(0x03, 0x08); // length load, timer high bits
//! apu.advance(29830); // one full 4-step sequence
//! assert!(apu.read(0x15) & 0x40 != 0, "the frame IRQ should have fired");
//! ```

pub mod dmc;
pub mod frame;
pub mod mixer;
pub mod noise;
pub mod pulse;
pub mod triangle;
pub mod units;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::DomainId;
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{AtomicU8, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};

pub use dmc::{DmaKind, DmaRequest};
pub use frame::{Mode, Timing};

use dmc::Dmc;
use frame::FrameCounter;
use mixer::SampleRing;
use noise::Noise;
use pulse::Pulse;
use triangle::Triangle;

/// Register indices, as offsets from `$4000`.
mod reg {
    /// Pulse 1 duty, halt, volume.
    pub(super) const PULSE1_CTRL: u8 = 0x00;
    /// Pulse 1 sweep setup.
    pub(super) const PULSE1_SWEEP: u8 = 0x01;
    /// Pulse 1 timer low.
    pub(super) const PULSE1_LO: u8 = 0x02;
    /// Pulse 1 length load and timer high.
    pub(super) const PULSE1_HI: u8 = 0x03;
    /// Pulse 2 duty, halt, volume.
    pub(super) const PULSE2_CTRL: u8 = 0x04;
    /// Pulse 2 sweep setup.
    pub(super) const PULSE2_SWEEP: u8 = 0x05;
    /// Pulse 2 timer low.
    pub(super) const PULSE2_LO: u8 = 0x06;
    /// Pulse 2 length load and timer high.
    pub(super) const PULSE2_HI: u8 = 0x07;
    /// Triangle linear counter setup.
    pub(super) const TRI_LINEAR: u8 = 0x08;
    /// Triangle timer low.
    pub(super) const TRI_LO: u8 = 0x0A;
    /// Triangle length load and timer high.
    pub(super) const TRI_HI: u8 = 0x0B;
    /// Noise halt, volume.
    pub(super) const NOISE_CTRL: u8 = 0x0C;
    /// Noise mode and period.
    pub(super) const NOISE_PERIOD: u8 = 0x0E;
    /// Noise length load.
    pub(super) const NOISE_LEN: u8 = 0x0F;
    /// DMC flags and rate.
    pub(super) const DMC_CTRL: u8 = 0x10;
    /// DMC direct load.
    pub(super) const DMC_LOAD: u8 = 0x11;
    /// DMC sample address.
    pub(super) const DMC_ADDR: u8 = 0x12;
    /// DMC sample length.
    pub(super) const DMC_LEN: u8 = 0x13;
    /// Channel enables and status.
    pub(super) const STATUS: u8 = 0x15;
    /// Frame counter control.
    pub(super) const FRAME: u8 = 0x17;
}

/// Everything the APU keeps that a snapshot has to reproduce.
#[derive(Debug)]
struct Core {
    frame: FrameCounter,
    pulse1: Pulse,
    pulse2: Pulse,
    triangle: Triangle,
    noise: Noise,
    dmc: Dmc,
    /// CPU cycles since power-on, in this device's clock domain.
    ticks: u64,
    /// The power-on alignment between CPU and APU cycles: 0 when CPU cycle 0 is
    /// a get cycle, 1 when it is a put cycle.
    ///
    /// Random on real hardware ([NESdev DMA](https://www.nesdev.org/wiki/DMA)),
    /// and therefore a machine property here rather than something sampled: a
    /// non-deterministic input has to cross the record/replay seam or it is a
    /// determinism bug (`ROADMAP.md` §4.5).
    phase: u64,
    samples: SampleRing,
}

impl Core {
    fn new(timing: Timing, phase: u64, halt_ultrasonic: bool, capacity: usize) -> Core {
        Core {
            frame: FrameCounter::new(timing),
            pulse1: Pulse::new(true),
            pulse2: Pulse::new(false),
            triangle: Triangle::new(halt_ultrasonic),
            noise: Noise::new(timing),
            dmc: Dmc::new(timing),
            ticks: 0,
            phase,
            samples: SampleRing::with_capacity(capacity),
        }
    }

    /// Whether the most recently processed CPU cycle was a put cycle.
    #[inline]
    fn on_put_cycle(&self) -> bool {
        (self.ticks.wrapping_sub(1).wrapping_add(self.phase)) & 1 == 1
    }

    /// Advance one CPU cycle.
    fn tick(&mut self) {
        self.ticks += 1;
        let now = self.ticks;

        let event = self.frame.tick(now);
        if event.quarter {
            self.clock_quarter_frame();
        }
        if event.half {
            self.clock_half_frame();
        }

        self.triangle.tick_timer();

        if !self.on_put_cycle() {
            self.pulse1.tick_timer();
            self.pulse2.tick_timer();
            self.noise.tick_timer();
            self.dmc.tick_timer();
            let sample = self.mix();
            self.samples.push(sample);
        }
    }

    /// Envelopes and the triangle's linear counter.
    fn clock_quarter_frame(&mut self) {
        self.pulse1.envelope.clock();
        self.pulse2.envelope.clock();
        self.noise.envelope.clock();
        self.triangle.clock_linear();
    }

    /// Length counters and sweep units.
    fn clock_half_frame(&mut self) {
        self.pulse1.length.clock();
        self.pulse1.clock_sweep();
        self.pulse2.length.clock();
        self.pulse2.clock_sweep();
        self.triangle.length.clock();
        self.noise.length.clock();
    }

    /// The current mixed output level, in Q16.
    fn mix(&self) -> u16 {
        mixer::mix(
            self.pulse1.output(),
            self.pulse2.output(),
            self.triangle.output(),
            self.noise.output(),
            self.dmc.output(),
        )
    }

    /// Whether either interrupt flag is asserting the IRQ line.
    #[inline]
    fn irq_asserted(&self) -> bool {
        self.frame.irq() || self.dmc.irq()
    }

    fn write(&mut self, index: u8, value: u8) {
        match index {
            reg::PULSE1_CTRL => self.pulse1.write_control(value),
            reg::PULSE1_SWEEP => self.pulse1.write_sweep(value),
            reg::PULSE1_LO => self.pulse1.write_period_low(value),
            reg::PULSE1_HI => self.pulse1.write_period_high(value),
            reg::PULSE2_CTRL => self.pulse2.write_control(value),
            reg::PULSE2_SWEEP => self.pulse2.write_sweep(value),
            reg::PULSE2_LO => self.pulse2.write_period_low(value),
            reg::PULSE2_HI => self.pulse2.write_period_high(value),
            reg::TRI_LINEAR => self.triangle.write_linear(value),
            reg::TRI_LO => self.triangle.write_period_low(value),
            reg::TRI_HI => self.triangle.write_period_high(value),
            reg::NOISE_CTRL => self.noise.write_control(value),
            reg::NOISE_PERIOD => self.noise.write_period(value),
            reg::NOISE_LEN => self.noise.write_length(value),
            reg::DMC_CTRL => self.dmc.write_control(value),
            reg::DMC_LOAD => self.dmc.write_output(value),
            reg::DMC_ADDR => self.dmc.write_address(value),
            reg::DMC_LEN => self.dmc.write_length(value),
            reg::STATUS => self.write_status(value),
            reg::FRAME => self.frame.write(value, self.on_put_cycle()),
            // $4009, $400D and $4014/$4016 are not APU registers. The first two
            // are unimplemented on the chip; the last two belong to the PPU's
            // OAM DMA port and the controller port and are never routed here.
            _ => {}
        }
    }

    /// `$4015` write: `---D NT21`.
    fn write_status(&mut self, value: u8) {
        self.pulse1.length.set_enabled(value & 0x01 != 0);
        self.pulse2.length.set_enabled(value & 0x02 != 0);
        self.triangle.length.set_enabled(value & 0x04 != 0);
        self.noise.length.set_enabled(value & 0x08 != 0);
        self.dmc.clear_irq();
        self.dmc.set_enabled(value & 0x10 != 0);
    }

    /// `$4015` read: `IF-D NT21`, with bit 5 from the open bus.
    fn read_status(&mut self, open_bus: u8, peek: bool) -> u8 {
        let mut value = open_bus & 0x20;
        if self.pulse1.length.active() {
            value |= 0x01;
        }
        if self.pulse2.length.active() {
            value |= 0x02;
        }
        if self.triangle.length.active() {
            value |= 0x04;
        }
        if self.noise.length.active() {
            value |= 0x08;
        }
        if self.dmc.active() {
            value |= 0x10;
        }
        if self.frame.read_irq(self.ticks, peek) {
            value |= 0x40;
        }
        if self.dmc.irq() {
            value |= 0x80;
        }
        value
    }

    fn reset(&mut self, kind: ResetKind, timing: Timing, halt_ultrasonic: bool) {
        match kind {
            ResetKind::Cold => {
                let phase = self.phase;
                let capacity = self.samples.capacity();
                *self = Core::new(timing, phase, halt_ultrasonic, capacity);
            }
            ResetKind::Warm | ResetKind::Bus => {
                // A reset is documented as a $4015 write of $00 plus a handful
                // of chip-specific effects; $4017 is left alone
                // (NESdev CPU power up state).
                self.write_status(0x00);
                self.frame.reset_warm();
                self.dmc.reset_warm();
                self.triangle.reset_phase();
                self.samples.clear();
            }
        }
    }

    fn save(&self, w: &mut dyn crate::core::state::Sink) -> Result<()> {
        self.frame.save(w)?;
        self.pulse1.save(w)?;
        self.pulse2.save(w)?;
        self.triangle.save(w)?;
        self.noise.save(w)?;
        self.dmc.save(w)?;
        w.write_u64(self.ticks)?;
        w.write_u64(self.phase)
    }

    fn load<'a>(&mut self, r: &mut dyn crate::core::state::Source<'a>) -> Result<()> {
        self.frame.load(r)?;
        self.pulse1.load(r)?;
        self.pulse2.load(r)?;
        self.triangle.load(r)?;
        self.noise.load(r)?;
        self.dmc.load(r)?;
        self.ticks = r.read_u64()?;
        self.phase = r.read_u64()? & 1;
        // Samples already handed to the host are not replayed, and the ring is
        // output rather than architectural state (`ROADMAP.md` §4.5).
        self.samples.clear();
        Ok(())
    }
}

/// The shared interior of an [`Apu`], so that its I/O ports can outlive a
/// borrow of the device.
struct ApuState {
    core: Mutex<Core>,
    /// The IRQ output port, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The last value the CPU's external data bus held.
    ///
    /// `$4015` bit 5 reads back from here. The register is internal to the CPU,
    /// so a `$4015` read neither drives the external bus nor updates it — the
    /// bus layer must not feed the result of a `$4015` read back in through
    /// [`Apu::set_open_bus`].
    open_bus: AtomicU8,
    /// The clock domain whose ticks [`Apu::advance_to`] is called with.
    ///
    /// Recorded rather than used: the APU counts its own CPU cycles, and the
    /// handle is here so a board can say which domain those cycles belong to
    /// and so a monitor can report it. The machine assembly layer will set it
    /// from `RealizeCtx` once that layer can hand out the forest.
    domain: Mutex<Option<DomainId>>,
    timing: Timing,
    halt_ultrasonic: bool,
}

impl fmt::Debug for ApuState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ApuState")
            .field("timing", &self.timing)
            .field("open_bus", &self.open_bus.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl ApuState {
    /// Whether either interrupt flag is asserting the IRQ line.
    fn irq_level(&self) -> Level {
        if self.core.lock().irq_asserted() {
            Level::High
        } else {
            Level::Low
        }
    }

    /// Drive the IRQ line to match the interrupt flags.
    ///
    /// The device lock is released before the wire is touched, per the
    /// re-entrancy contract in [`crate::core::device`]: a sink is free to call
    /// back into anything, including this device.
    fn refresh_irq(&self) {
        let level = self.irq_level();
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    fn write(&self, index: u8, value: u8) {
        self.core.lock().write(index, value);
        self.refresh_irq();
    }

    fn read_with(&self, index: u8, peek: bool) -> u8 {
        let open_bus = self.open_bus.load(Ordering::Relaxed);
        if index != reg::STATUS {
            return open_bus;
        }
        let value = self.core.lock().read_status(open_bus, peek);
        if !peek {
            self.refresh_irq();
        }
        value
    }
}

/// The NES APU.
///
/// Construct with [`Apu::new`], map its [`Apu::regions`] into the CPU's address
/// space, connect [`Apu::connect_irq`] to the CPU's IRQ line, and advance it
/// with [`Apu::advance_to`] before every access.
#[derive(Debug)]
pub struct Apu {
    state: Arc<ApuState>,
}

impl Apu {
    /// Validate properties and allocate. Performs no outward action.
    ///
    /// Properties: `timing` (`ntsc` or `pal`), `sample-buffer` (capacity of the
    /// output ring in samples; 0 disables it), `halt-ultrasonic`, and
    /// `put-phase` (0 or 1).
    pub fn new(props: &Props) -> Result<Apu> {
        let mut reader = props.reader();
        let timing_name = reader.or_str("timing", "ntsc")?;
        let timing = Timing::from_name(timing_name).ok_or_else(|| {
            Error::Property(alloc::format!(
                "property `timing` must be `ntsc` or `pal`, not `{timing_name}`"
            ))
        })?;
        let capacity = reader.or_range::<u64>("sample-buffer", 8192, 0..=1 << 24)?;
        let halt_ultrasonic = reader.or("halt-ultrasonic", false)?;
        let phase = reader.or_range::<u64>("put-phase", 0, 0..=1)?;
        reader.finish()?;

        Ok(Apu {
            state: Arc::new(ApuState {
                core: Mutex::with_rank(
                    LockRank::DEVICE,
                    Core::new(timing, phase, halt_ultrasonic, capacity as usize),
                ),
                irq: Mutex::with_rank(LockRank::WIRE, None),
                open_bus: AtomicU8::new(0),
                domain: Mutex::with_rank(LockRank::LEAF, None),
                timing,
                halt_ultrasonic,
            }),
        })
    }

    /// Which console variant this APU is timed for.
    pub fn timing(&self) -> Timing {
        self.state.timing
    }

    // -- Wiring -------------------------------------------------------------

    /// Connect the IRQ output.
    ///
    /// The APU drives one line from two flags — the frame interrupt and the DMC
    /// interrupt — combined internally, because on the chip they really are one
    /// output. Fan-in with other IRQ sources (a cartridge, say) is the wire's
    /// job, which is why [`crate::core::wire::WireSink::set_level`] carries the
    /// source id.
    pub fn connect_irq(&self, source: WireSource) {
        *self.state.irq.lock() = Some(source);
        self.refresh_irq();
    }

    /// The level the APU is currently driving on its IRQ output.
    pub fn irq_level(&self) -> Level {
        self.state.irq_level()
    }

    /// Drive the IRQ line to match the interrupt flags.
    fn refresh_irq(&self) {
        self.state.refresh_irq();
    }

    // -- Time ---------------------------------------------------------------

    /// Record which clock domain drives this APU.
    ///
    /// On the NES that is the CPU domain — the master oscillator divided by 12
    /// — and the intended call is
    /// `apu.advance_to(forest.ticks(apu.clock_domain().unwrap())?)` before an
    /// access. The APU never reads the forest itself, and never a host clock.
    pub fn attach_clock(&self, domain: DomainId) {
        *self.state.domain.lock() = Some(domain);
    }

    /// The clock domain [`Apu::attach_clock`] was given, if any.
    pub fn clock_domain(&self) -> Option<DomainId> {
        *self.state.domain.lock()
    }

    /// CPU cycles elapsed in this device's clock domain since power-on.
    pub fn ticks(&self) -> u64 {
        self.state.core.lock().ticks
    }

    /// Which sequence the frame counter is running.
    pub fn frame_mode(&self) -> Mode {
        self.state.core.lock().frame.mode()
    }

    /// CPU cycles since the frame sequence last restarted.
    ///
    /// Not guest-visible — the frame counter has no readable position — but the
    /// monitor and the tests both need to see it.
    pub fn frame_cycle(&self) -> u32 {
        self.state.core.lock().frame.cycle()
    }

    /// The DMC's 7-bit output level, as `$4011` last set it or playback left it.
    ///
    /// Also not guest-visible: `$4011` is write-only.
    pub fn dmc_output(&self) -> u8 {
        self.state.core.lock().dmc.output()
    }

    /// Advance by `cycles` CPU cycles.
    pub fn advance(&self, cycles: u64) {
        {
            let mut core = self.state.core.lock();
            for _ in 0..cycles {
                core.tick();
            }
        }
        self.refresh_irq();
    }

    /// Advance to CPU cycle `tick`, doing nothing if already there or past it.
    ///
    /// This is the catch-up hook of `ROADMAP.md` §4.2: the address space calls
    /// it before dispatching an access so that a sampled register — `$4015`
    /// above all — is read at the cycle the CPU actually reads it.
    pub fn advance_to(&self, tick: u64) {
        {
            let mut core = self.state.core.lock();
            while core.ticks < tick {
                core.tick();
            }
        }
        self.refresh_irq();
    }

    // -- Registers ----------------------------------------------------------

    /// Write a register by its offset from `$4000`.
    pub fn write(&self, index: u8, value: u8) {
        self.state.write(index, value);
    }

    /// Read a register by its offset from `$4000`.
    ///
    /// Only `$4015` is readable; everything else returns the open-bus value.
    pub fn read(&self, index: u8) -> u8 {
        self.read_with(index, false)
    }

    /// Read a register without side effects, for a debugger or the monitor.
    ///
    /// Honours `MemAttrs::debug` (`ROADMAP.md` §15, invariant 5): the frame
    /// interrupt flag is reported but not cleared.
    pub fn peek(&self, index: u8) -> u8 {
        self.read_with(index, true)
    }

    fn read_with(&self, index: u8, peek: bool) -> u8 {
        self.state.read_with(index, peek)
    }

    /// Record the value the CPU's external data bus last held.
    ///
    /// `$4015` bit 5 reads back from here. Do **not** call this with the result
    /// of a `$4015` read: that register is internal to the CPU and the external
    /// bus is disconnected for it, so the open-bus value must come from the
    /// last cycle that read something else ([NESdev
    /// APU](https://www.nesdev.org/wiki/APU)).
    pub fn set_open_bus(&self, value: u8) {
        self.state.open_bus.store(value, Ordering::Relaxed);
    }

    /// The open-bus value currently latched.
    pub fn open_bus(&self) -> u8 {
        self.state.open_bus.load(Ordering::Relaxed)
    }

    // -- DMC DMA ------------------------------------------------------------

    /// The sample fetch the CPU should perform, if the DMC wants one.
    pub fn dma_request(&self) -> Option<DmaRequest> {
        self.state.core.lock().dmc.dma_request()
    }

    /// Whether the request identified by `serial` is still outstanding.
    ///
    /// Returns false once playback has stopped, which is how a CPU that has
    /// already begun the halt sequence learns it is servicing an aborted DMA.
    pub fn dma_is_pending(&self, serial: u64) -> bool {
        self.state.core.lock().dmc.dma_is_pending(serial)
    }

    /// Hand the DMC the byte the CPU fetched.
    ///
    /// Returns false if the request was withdrawn before the get cycle, in
    /// which case nothing changed and the byte is discarded.
    pub fn dma_complete(&self, serial: u64, byte: u8) -> bool {
        let accepted = self.state.core.lock().dmc.dma_complete(serial, byte);
        if accepted {
            self.refresh_irq();
        }
        accepted
    }

    // -- Audio --------------------------------------------------------------

    /// The current mixed output level, in Q16 (65536 would be full scale).
    pub fn output(&self) -> u16 {
        self.state.core.lock().mix()
    }

    /// Move every buffered sample into `out`, oldest first.
    ///
    /// Samples are produced one per APU cycle — half the CPU clock — and are
    /// not resampled, filtered or rate-controlled here.
    pub fn take_samples(&self, out: &mut Vec<u16>) {
        self.state.core.lock().samples.drain_into(out);
    }

    /// How many samples have been dropped because nobody drained the ring.
    pub fn samples_dropped(&self) -> u64 {
        self.state.core.lock().samples.dropped()
    }

    // -- Memory-mapped ports ------------------------------------------------

    /// The device's I/O regions, each with its offset from `$4000`.
    ///
    /// Three regions rather than one 24-byte block because `$4014` (OAM DMA)
    /// and `$4016` (the controller port) sit inside that range and belong to
    /// other devices. Mapping three exact windows means a machine description
    /// cannot accidentally give the APU an address it does not decode.
    pub fn regions(&self) -> Vec<(u64, Region)> {
        alloc::vec![
            (
                0x00,
                Region::io("apu.channels", 0x14, self.port(reg::PULSE1_CTRL)),
            ),
            (0x15, Region::io("apu.status", 1, self.port(reg::STATUS))),
            (0x17, Region::io("apu.frame", 1, self.port(reg::FRAME))),
        ]
    }

    /// A [`MemOps`] view whose offset 0 is register `first`.
    fn port(&self, first: u8) -> Arc<dyn MemOps> {
        Arc::new(ApuPort {
            state: Arc::clone(&self.state),
            first,
        })
    }
}

/// One memory-mapped window onto an [`Apu`].
struct ApuPort {
    state: Arc<ApuState>,
    /// The register index this window's offset 0 corresponds to.
    first: u8,
}

impl fmt::Debug for ApuPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ApuPort")
            .field("first", &self.first)
            .finish_non_exhaustive()
    }
}

impl ApuPort {
    /// The register index an access at `offset` reaches, if it is in range.
    fn index(&self, offset: u64) -> Option<u8> {
        u8::try_from(offset)
            .ok()
            .and_then(|o| self.first.checked_add(o))
    }
}

impl MemOps for ApuPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(crate::core::error::BusError::BadAccess);
        };
        let index = self
            .index(offset)
            .ok_or(crate::core::error::BusError::BadAccess)?;
        *byte = self.state.read_with(index, attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(crate::core::error::BusError::BadAccess);
        };
        let index = self
            .index(offset)
            .ok_or(crate::core::error::BusError::BadAccess)?;
        if attrs.debug {
            // A debug write would have every side effect a real one does; the
            // monitor has to go through the device's own API to say it meant it.
            return Ok(());
        }
        self.state.write(index, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Device for Apu {
    fn class(&self) -> &'static DeviceClass {
        &APU_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward yet: the machine assembly layer that hands a device
        // its address space, its clock domain and its wires is still being
        // built (`core::mod` says so). Until it exists a board wires the APU up
        // through `regions`, `connect_irq` and `advance_to`, which is exactly
        // what realize will do once it can.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        {
            let mut core = self.state.core.lock();
            let timing = self.state.timing;
            let halt = self.state.halt_ultrasonic;
            core.reset(kind, timing, halt);
        }
        self.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.state.core.lock().save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.state.core.lock().load(r)?;
        self.refresh_irq();
        Ok(())
    }
}

/// The properties [`APU_CLASS`] accepts.
static APU_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "timing",
        kind: ValueKind::Str,
        required: false,
        summary: "console variant: `ntsc` (RP2A03) or `pal` (RP2A07)",
    },
    PropertySpec {
        name: "sample-buffer",
        kind: ValueKind::Uint,
        required: false,
        summary: "audio ring capacity in samples; 0 produces no audio at all",
    },
    PropertySpec {
        name: "halt-ultrasonic",
        kind: ValueKind::Bool,
        required: false,
        summary: "halt the triangle when its period is below 2, trading accuracy for less popping",
    },
    PropertySpec {
        name: "put-phase",
        kind: ValueKind::Uint,
        required: false,
        summary: "CPU/APU cycle alignment at power-on (0 or 1); random on hardware",
    },
];

/// The device class, as `nes.apu` in a machine description.
pub static APU_CLASS: DeviceClass = DeviceClass {
    name: "nes.apu",
    version: 1,
    summary: "NES APU (RP2A03 audio): two pulse, triangle, noise and DMC channels",
    properties: APU_PROPERTIES,
    construct: |props| Ok(Box::new(Apu::new(props)?) as Box<dyn Device>),
};

/// Add the APU to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4).
pub fn register(registry: &mut Registry) -> Result<()> {
    registry.add(&APU_CLASS)
}
