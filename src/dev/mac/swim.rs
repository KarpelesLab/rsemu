//! `mac.swim`: the Sander-Wozniak Integrated Machine, the disk controller a
//! Macintosh Classic has where a Plus has an IWM.
//!
//! ```text
//!   osc    diskbit = 1000000 Hz          # one tick is one MFM cell
//!   object swim "mac.swim" { drives = 1, clock = diskbit, image = "floppy" }
//!   map mem 0xC00000 size 0x200000 = mirror(swim) { endian = "big" }
//!   wire via.pa5 -> swim.sel { pull = "up" }
//! ```
//!
//! # What a SWIM is
//!
//! A **superset of the IWM**. It powers up answering the IWM's sixteen soft
//! switches, so a machine's startup code drives it exactly as it would drive
//! the older part, and software switches it into **ISM mode** — the chip's own
//! register file, with the MFM separator and the sector-search engine a
//! 1.44 MB disk needs — when it wants high-density media.
//!
//! So this device *is* [`super::iwm`] while it is in IWM mode: it owns one and
//! forwards every access to it, rather than carrying a second copy of the
//! sixteen switches, the drive register file and the GCR shifter. What it adds
//! is the **SuperDrive** on the cable ([`Iwm::with_superdrives`]), which
//! answers the drive register at `CA2:CA1:CA0 = 101` where an 800K mechanism
//! leaves the cable's pull-up, and the ability to take a 1.44 MB image at all
//! ([`super::disk::Reader::Swim`]).
//!
//! # The clock is 1 MHz here, and a GCR disk gets every other tick
//!
//! One tick is one **MFM** cell, because that is the faster of the two rates
//! this controller has to carry: MFM spends two cells on a data bit, so
//! 500 kbit/s is a 1 MHz cell rate ([`super::mfm`]). An Apple GCR disk's cells
//! are 500 kHz, so the GCR side of the chip is advanced on every *second*
//! tick — exact integer arithmetic within one oscillator, which is what
//! `CLAUDE.md`'s determinism rule asks for, rather than two oscillators whose
//! ratio would have to be stated somewhere else.
//!
//! The two rates come out at the right spindle speeds with no further number:
//!
//! ```text
//!   GCR, zone 0:  500,000 cells/s / 76,140 cells a revolution = 394 rpm
//!   MFM:        1,000,000 cells/s / 200,000 cells a revolution = 300 rpm
//! ```
//!
//! # What is **not** here, and it is the reason this board does not boot yet
//!
//! **ISM mode.** The chip's own register file — the one the MFM separator and
//! the sector-search engine live behind — is not modelled, because no document
//! available to this project states it and `CLAUDE.md` forbids reading any
//! emulator's source or disassembling Apple's ROM to find out. What *is*
//! recorded is the measurement, in `docs/platforms/mac-classic.md`: which
//! addresses a real Macintosh Classic ROM touches in this window, in what
//! order, and with what values. An ISM register table invented to fit would be
//! exactly the mistake that cost this board three sessions the last time
//! (`docs/platforms/mac-plus.md`, "The drive's register file was invented"), so
//! there is not one here.
//!
//! The consequence is honest and narrow: a 1.44 MB disk goes into the drive,
//! becomes MFM cells, and turns under the head at 300 rpm, and the ROM can see
//! that the mechanism is a SuperDrive — but the chip cannot yet hand it a
//! sector, so the machine reaches the insert-disk screen and stays there.
//!
//! No emulator source was consulted and no ROM was disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).

use alloc::boxed::Box;
use alloc::sync::Arc;

use super::disk::{Density, Disk, Reader};
use super::iwm::{self, Iwm};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionKind, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, Ordering};
use crate::core::wire::WireId;
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.swim";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register file occupies: the board puts
/// the register selects on A9-A12, exactly as it does the IWM's.
pub const REGISTER_SPAN: u64 = iwm::REGISTER_SPAN;

/// The input pin the VIA's `PA5` drives: the drive register file's fourth
/// address bit.
pub const SEL_PIN: &str = "sel";

/// How many of this device's ticks one GCR cell is. See the module docs.
const GCR_DIVISOR: u64 = 2;

/// The chip's register file forwarded to the IWM it is a superset of.
///
/// A separate type rather than the `Iwm`'s own region, because the address
/// space has to dispatch to *this* device — a snapshot names devices by their
/// path and a machine file maps `swim`, not the IWM inside it.
#[derive(Debug)]
struct Ports {
    ops: Arc<dyn MemOps>,
}

impl MemOps for Ports {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.ops.read(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.ops.write(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        self.ops.constraints()
    }
}

/// A SWIM and the SuperDrives on its cable.
#[derive(Debug)]
pub struct Swim {
    /// The IWM this chip is a superset of, with SuperDrive mechanisms.
    iwm: Arc<Iwm>,
    region: RegionRef,
    /// Ticks of this device's clock — MFM cells — simulated.
    ticks: AtomicU64,
}

impl Swim {
    /// Build the chip.
    ///
    /// `drives` says how many mechanisms are on the cable: 1 for a Macintosh
    /// Classic with only its internal drive, 2 with an external one plugged in.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `drives` is not 1 or 2, or if a property this
    /// class does not know was given; [`Error::Config`] if the image in the
    /// `image` slot is not a disk a SuperDrive can read.
    pub fn new(props: &Props) -> Result<Swim> {
        let mut r = props.reader();
        let drives = r.or("drives", 1u64)?;
        let image = r.optional_media("image")?.map(|m| m.bytes().to_vec());
        r.finish()?;
        if drives == 0 || drives > 2 {
            return Err(Error::Property(alloc::format!(
                "property `drives`: a SWIM's cable takes one or two mechanisms, not {drives}"
            )));
        }
        let swim = Swim::with_drives([true, drives == 2]);
        // An empty slot is an empty drive rather than a bad image: a Macintosh
        // with no disk in it is the ordinary case and the one the ROM draws a
        // picture for.
        if let Some(bytes) = image.filter(|b| !b.is_empty()) {
            swim.insert(0, Disk::from_image_for(&bytes, Reader::Swim)?);
        }
        Ok(swim)
    }

    /// The same, saying exactly which cable positions are occupied.
    #[must_use]
    pub fn with_drives(installed: [bool; 2]) -> Swim {
        let iwm = Arc::new(Iwm::with_superdrives(installed));
        // The IWM's own aperture, forwarded rather than copied. A region's
        // MMIO ops are reachable through its kind, which is what makes this a
        // delegation and not a second implementation of sixteen soft switches.
        let inner = iwm.region("").expect("an IWM has a register file");
        let ops = match inner.kind() {
            RegionKind::Io(ops) => Arc::clone(ops),
            _ => unreachable!("an IWM's register file is MMIO"),
        };
        let region = Arc::new(
            Region::io(
                CLASS_NAME,
                REGISTER_SPAN,
                Arc::new(Ports { ops }) as Arc<dyn MemOps>,
            )
            .with_constraints(inner.constraints()),
        );
        Swim {
            iwm,
            region,
            ticks: AtomicU64::new(0),
        }
    }

    /// The IWM this chip is a superset of, for a test that wants to look at the
    /// drive or the shifter.
    #[must_use]
    pub fn iwm(&self) -> &Arc<Iwm> {
        &self.iwm
    }

    /// Read a switch the way the address space would, for a test.
    #[must_use]
    pub fn peek(&self, index: u8) -> u8 {
        self.iwm.peek(index)
    }

    /// Write one the same way.
    pub fn poke(&self, index: u8, value: u8) {
        self.iwm.poke(index, value);
    }

    /// Set the level the VIA is driving onto `SEL`, for a test with no wire
    /// graph.
    pub fn set_sel(&self, high: bool) {
        self.iwm.set_sel(high);
    }

    /// Put `disk` in drive `which`, taking out whatever was there.
    pub fn insert(&self, which: usize, disk: Disk) {
        self.iwm.insert(which, disk);
    }

    /// Take the disk out of drive `which`.
    pub fn eject(&self, which: usize) {
        self.iwm.eject(which);
    }

    /// Whether drive `which` has a disk in it.
    #[must_use]
    pub fn has_disk(&self, which: usize) -> bool {
        self.iwm.has_disk(which)
    }

    /// Whether the motor of drive `which` is running.
    #[must_use]
    pub fn motor(&self, which: usize) -> bool {
        self.iwm.motor(which)
    }

    /// Which cylinder drive `which`'s head is over.
    #[must_use]
    pub fn track(&self, which: usize) -> u8 {
        self.iwm.track(which)
    }

    /// How the disk in drive `which` is written, if there is one.
    #[must_use]
    pub fn density(&self, which: usize) -> Option<Density> {
        self.iwm.disk(which).map(|d| d.density())
    }

    /// Ticks of this device's clock — MFM cells — simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// Shift the medium past the head until `target` of this chip's cells have
    /// gone by.
    ///
    /// A GCR disk's cells are half this rate, so it gets every second tick —
    /// see the module docs. Which rate applies is a property of the **disk**,
    /// not of the computer, so it comes from what is in the drive.
    pub fn advance_to(&self, target: u64) {
        self.ticks.store(target, Ordering::Relaxed);
        self.iwm.advance_to(self.inner_tick(target));
    }

    /// This chip's tick count in the units the IWM inside it counts.
    fn inner_tick(&self, tick: u64) -> u64 {
        match self.density(0) {
            Some(Density::Mfm) => tick,
            // No disk, or a GCR one: 500 kHz cells.
            _ => tick / GCR_DIVISOR,
        }
    }

    /// And the other way, so an event the IWM names lands on one of this
    /// chip's ticks.
    fn outer_tick(&self, tick: u64) -> u64 {
        match self.density(0) {
            Some(Density::Mfm) => tick,
            _ => tick.saturating_mul(GCR_DIVISOR),
        }
    }
}

impl Device for Swim {
    fn class(&self) -> &'static DeviceClass {
        &SWIM_CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Device::realize(&*self.iwm, ctx)
    }

    fn reset(&self, kind: ResetKind) {
        Device::reset(&*self.iwm, kind);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.ticks.load(Ordering::Relaxed))?;
        Device::save(&*self.iwm, w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        Device::load(&*self.iwm, r)?;
        self.ticks.store(ticks, Ordering::Relaxed);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Swim::advance_to(self, tick);
    }

    /// The cell the shifter will next latch a byte on, in **this** chip's
    /// ticks.
    ///
    /// The IWM names it in its own, which are half as fast while a GCR disk is
    /// turning, so the number has to be scaled back — an event named a tick
    /// early would have the scheduler cut the round in the middle of a cell,
    /// and one named late loses the byte, which is the defect
    /// `Iwm::next_event_tick` exists to stop.
    fn next_event_tick(&self) -> Option<u64> {
        Device::next_event_tick(&*self.iwm).map(|t| self.outer_tick(t))
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Device::attach_lazy(&*self.iwm, handle);
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The IWM's `sel` pin, under this class's name for it.
        Device::sink(&*self.iwm, port, sources)
    }
}

impl Instance for Swim {}

/// The `mac.swim` device class.
pub static SWIM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a SWIM: the IWM's sixteen soft switches, and the SuperDrives on its cable that \
              take 400K, 800K and 1.44 MB media",
    properties: &[
        PropertySpec {
            name: "drives",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many mechanisms are on the cable, 1 or 2 (default 1)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the disk in the internal drive; empty is no disk",
        },
    ],
    construct: |props| Ok(Box::new(Swim::new(props)?)),
};

/// Add [`SWIM_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SWIM_CLASS)
}

/// Bind [`SWIM_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Swim::new(props)?)))
}

/// What the validator should know about `mac.swim`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("drives", ValueKind::Uint).range(1, 2))
        .prop(PropSchema::new("image", ValueKind::Media))
        .region("")
        .region("regs")
        .port(SEL_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
