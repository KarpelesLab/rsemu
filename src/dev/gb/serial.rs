//! The Game Boy's serial link — `$FF01` and `$FF02`.
//!
//! Two registers and one shift register:
//!
//! ```text
//!   $FF01  SB  the byte being shifted out, and the byte shifted in
//!   $FF02  SC  bit 7 starts a transfer, bit 0 picks the internal clock
//! ```
//!
//! With a cable attached, the two consoles shift into each other. With nothing
//! attached — which is every case rsemu currently models — the line floats high,
//! so the byte shifted *in* is `$FF`. The transfer still takes its eight bit
//! periods and still raises the serial interrupt at the end, because the clock
//! is the console's own.
//!
//! # Why a test harness cares
//!
//! Blargg's Game Boy test ROMs write their results here as well as to the
//! screen, one character at a time, precisely so that a headless emulator can
//! read them without a PPU. [`GbSerial::transcript`] is that channel: every byte
//! a transfer sent, in order. It is not a hack bolted onto the device — it is
//! what a printer or a second console on the other end of the cable would have
//! received, kept because nothing else is listening.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) on the crystal's domain. The internal
//! clock is 8192 bits a second on a DMG, so a byte takes
//! `4194304 / 8192 * 8` = [`TRANSFER_CLOCKS`] crystal periods, exactly — no
//! rounding, because the ratio is an integer.
//!
//! An *external*-clock transfer (`SC` bit 0 clear) with nothing on the other end
//! never completes: the other console supplies the clock, and there is no other
//! console. That is the hardware behaviour, and a program that waits on it
//! hangs on real hardware too.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0), *Serial Data Transfer*. No
//! emulator source was consulted.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

/// Where the two registers sit in the CPU's address space.
pub const REGISTER_BASE: u64 = 0xff01;

/// How many bytes they cover.
pub const REGISTER_LEN: u64 = 2;

/// The name a `map` statement reaches them by.
pub const REGISTER_REGION: &str = "regs";

/// The serial-interrupt output pin.
pub const IRQ_PIN: &str = "irq";

/// How many crystal periods one byte takes on the internal clock.
///
/// 8192 bits a second out of 4 194 304 periods a second is 512 periods a bit,
/// and eight bits to the byte.
pub const TRANSFER_CLOCKS: u64 = 512 * 8;

/// How many bytes of transcript are kept before the oldest are dropped.
///
/// Blargg's longest transcript is a few hundred bytes; this is generous enough
/// that nothing real is lost and bounded so a runaway program cannot exhaust
/// memory.
pub const TRANSCRIPT_LIMIT: usize = 64 * 1024;

#[derive(Debug, Clone, Copy, Default)]
struct Regs {
    /// `SB`.
    data: u8,
    /// `SC`, low two bits only.
    control: u8,
    /// Crystal periods left in the transfer in flight, or zero when idle.
    remaining: u64,
}

impl Regs {
    fn transferring(&self) -> bool {
        self.control & 0x80 != 0
    }

    fn internal_clock(&self) -> bool {
        self.control & 0x01 != 0
    }
}

struct Shared {
    regs: Mutex<Regs>,
    transcript: Mutex<Vec<u8>>,
    irq: Mutex<Option<WireSource>>,
    lazy: Mutex<Option<LazyHandle>>,
    tick: AtomicU64,
    /// The tick this device's own next event falls on.
    ///
    /// `u64::MAX` stands for "nothing pending", so the atomic needs no
    /// companion flag — which matters because the scheduler reads this with its
    /// slot held at [`LockRank::LEAF`] and cannot take a second one.
    next_event: AtomicU64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared").field("regs", &self.regs).finish()
    }
}

impl Shared {
    fn publish(&self, regs: &Regs, now: u64) {
        self.tick.store(now, Ordering::Relaxed);
        let event = if regs.transferring() && regs.internal_clock() {
            now + regs.remaining.max(1)
        } else {
            u64::MAX
        };
        self.next_event.store(event, Ordering::Relaxed);
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

    /// Append one byte to the transcript, dropping the oldest when it is full.
    fn push(&self, byte: u8) {
        let mut transcript = self.transcript.lock();
        if transcript.len() >= TRANSCRIPT_LIMIT {
            transcript.remove(0);
        }
        transcript.push(byte);
    }

    fn drive_irq(&self) {
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            // A pulse: the completion is an event, and the CPU's pin latches
            // the rising edge into `IF`.
            source.set(Level::High);
            source.set(Level::Low);
        }
    }
}

/// The serial link as a device.
pub struct GbSerial {
    shared: Arc<Shared>,
    regs_region: RegionRef,
}

impl fmt::Debug for GbSerial {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbSerial")
            .field("regs", &self.shared.regs)
            .finish_non_exhaustive()
    }
}

impl Default for GbSerial {
    fn default() -> Self {
        GbSerial::new()
    }
}

impl GbSerial {
    /// A link in its power-on state, with nothing on the other end.
    #[must_use]
    pub fn new() -> GbSerial {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::default()),
            // A leaf: the transcript is appended to from inside the register
            // lock when a transfer completes, and nothing is ever taken while
            // *it* is held. Ranking it `DEVICE` alongside the registers would
            // be `DEVICE <= DEVICE` and the order check would fire on the first
            // byte a test ROM printed — correctly, since two locks at one rank
            // is a cycle waiting to happen.
            transcript: Mutex::new(Vec::new()),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let regs_region = Arc::new(MmioRegion::io(
            "gb.serial.regs",
            REGISTER_LEN,
            Arc::new(SerialPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbSerial {
            shared,
            regs_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or is unknown.
    pub fn from_props(props: &Props) -> Result<GbSerial> {
        let mut r = props.reader();
        // Accepted and ignored for now: there is exactly one thing that can be
        // on the other end of the cable, and it is nothing.
        let _ = r.or_enum("link", "none", &["none"])?;
        r.finish()?;
        Ok(GbSerial::new())
    }

    /// Every byte a transfer has sent, in order.
    ///
    /// This is how a headless runner reads a blargg test ROM's result.
    #[must_use]
    pub fn transcript(&self) -> Vec<u8> {
        self.shared.transcript.lock().clone()
    }

    /// The transcript as text, with anything that is not UTF-8 replaced.
    #[must_use]
    pub fn transcript_text(&self) -> String {
        let bytes = self.transcript();
        bytes.iter().map(|b| *b as char).collect()
    }

    /// Forget the transcript so far.
    pub fn clear_transcript(&self) {
        self.shared.transcript.lock().clear();
    }

    /// `SB`.
    #[must_use]
    pub fn data(&self) -> u8 {
        self.shared.regs.lock().data
    }

    /// `SC`, as the guest reads it.
    #[must_use]
    pub fn control(&self) -> u8 {
        // Bits 1-6 are not implemented and read as ones.
        self.shared.regs.lock().control | 0x7e
    }

    /// Connect the serial-interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Advance to `target` crystal periods since reset.
    pub fn advance_to(&self, target: u64) {
        let completed = {
            let mut regs = self.shared.regs.lock();
            let now = self.shared.tick.load(Ordering::Relaxed);
            let elapsed = target.saturating_sub(now);
            let mut completed = false;
            if regs.transferring() && regs.internal_clock() {
                if elapsed >= regs.remaining {
                    regs.remaining = 0;
                    // Nothing on the other end of the cable, so the line floats
                    // high and every bit shifted in is a one.
                    regs.data = 0xff;
                    regs.control &= !0x80;
                    completed = true;
                } else {
                    regs.remaining -= elapsed;
                }
            }
            self.shared.publish(&regs, target);
            completed
        };
        if completed {
            // Outward action after the critical section, per the re-entrancy
            // contract.
            self.shared.drive_irq();
        }
    }
}

/// The `$FF01`-`$FF02` register block.
struct SerialPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for SerialPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SerialPort").finish_non_exhaustive()
    }
}

impl MemOps for SerialPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let regs = self.shared.regs.lock();
        *byte = if offset == 0 {
            regs.data
        } else {
            regs.control | 0x7e
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Writing `SC` starts a transfer, which is a side effect a debugger
            // must not cause (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let mut regs = self.shared.regs.lock();
        if offset == 0 {
            regs.data = *value;
        } else {
            // Bit 7 set starts a transfer, and does so even when one is already
            // running: hardware simply reloads the shift register, and a program
            // that writes `SC` twice in quick succession really does lose the
            // first byte down the cable.
            let starting = *value & 0x80 != 0;
            let byte = regs.data;
            regs.control = *value & 0x81;
            if starting {
                regs.remaining = TRANSFER_CLOCKS;
                // The transcript records the byte at the moment it *starts* to
                // go out, not when the eight bit periods are up. That is the
                // difference between a channel that keeps every character a
                // program sent and one that keeps only the last of each burst —
                // and the whole reason the transcript exists is to be read.
                // The transfer itself still takes its full time and still raises
                // the interrupt at the end.
                drop(regs);
                self.shared.push(byte);
                regs = self.shared.regs.lock();
            }
        }
        let now = self.shared.tick.load(Ordering::Relaxed);
        self.shared.publish(&regs, now);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `gb.serial` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.serial",
    version: 1,
    summary: "Game Boy serial link ($FF01-$FF02), with nothing on the other end",
    properties: &[PropertySpec {
        name: "link",
        kind: ValueKind::Str,
        required: false,
        summary: "what is on the other end of the cable; only `none` exists",
    }],
    construct: |props| Ok(Box::new(GbSerial::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbSerial {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs_region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!("the serial link drives only `{IRQ_PIN}`"),
            });
        }
        self.attach_irq(source);
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The tick is the clock domain's position, not this device's state, and
        // `Machine::reset` does not rewind domains.
        let now = self.shared.tick.load(Ordering::Relaxed);
        let mut regs = self.shared.regs.lock();
        *regs = Regs::default();
        self.shared.publish(&regs, now);
        drop(regs);
        if kind == ResetKind::Cold {
            self.clear_transcript();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        w.write_u8(regs.data)?;
        w.write_u8(regs.control)?;
        w.write_u64(regs.remaining)?;
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        // The transcript is architectural in the only sense that matters: a
        // snapshot taken mid-test and reloaded must report the same result.
        w.write_bytes(&self.shared.transcript.lock())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let regs = Regs {
            data: r.read_u8()?,
            control: r.read_u8()?,
            remaining: r.read_u64()?,
        };
        let tick = r.read_u64()?;
        let transcript = r.read_bytes()?;
        *self.shared.transcript.lock() = transcript.to_vec();
        let mut slot = self.shared.regs.lock();
        *slot = regs;
        self.shared.publish(&regs, tick);
        Ok(())
    }

    // -- lazily advanced -----------------------------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        GbSerial::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let event = self.shared.next_event.load(Ordering::Relaxed);
        (event != u64::MAX).then_some(event)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        GbSerial::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for GbSerial {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(GbSerial::from_props(props)?))
    })
}

/// What the validator should know about `gb.serial`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("link", ValueKind::Str).values(&["none"]))
        .port(IRQ_PIN, PortDir::Out)
        .region(REGISTER_REGION)
}
