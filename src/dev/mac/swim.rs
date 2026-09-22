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
//! # ISM mode, and how software asks for it
//!
//! The chip's own register file — the one the MFM separator and the
//! sector-search engine live behind — is [`ism`], and every register, bit and
//! rule in it carries the sentence it came from in Apple's *SWIM Chip User's
//! Reference*, revision 1.5 (11 January 1988), with its page.
//!
//! Getting in is four writes, page 12:
//!
//! > To select the ISM set, you must write to the GCR mode register **four
//! > times in a row** with this bit set to "1", "0", "1","1", respectively.
//!
//! `1, 0, 1, 1`, and that is exactly what a Macintosh Classic ROM writes:
//! `$57`, `$17`, `$57`, `$57`. [`ism::Switch`] is the four-entry shift register
//! that watches for it; clearing bit 6 of the ISM mode register switches back.
//!
//! The separator itself is **not** here but in [`super::iwm`], because that is
//! where the medium is: the head position, the cylinder under it and the motor
//! all live in the mechanism, and a second copy of them so that this file could
//! frame its own bytes is the duplication this device exists to avoid. What
//! this file owns is the register file, and what that register file reaches
//! for — the phase lines, the enables, `SENSE` — is the same drive an IWM
//! drives.
//!
//! No emulator source was consulted and no ROM was disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).

use alloc::boxed::Box;
use alloc::sync::Arc;

use self::ism::{Ism, Switch};
use super::disk::{Density, Disk, Reader};
use super::iwm::{self, Iwm};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionKind, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::WireId;
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.swim";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 3;

/// How many bytes of address space the register file occupies: the board puts
/// the register selects on A9-A12, exactly as it does the IWM's.
pub const REGISTER_SPAN: u64 = iwm::REGISTER_SPAN;

/// The input pin the VIA's `PA5` drives: the drive register file's fourth
/// address bit.
pub const SEL_PIN: &str = "sel";

/// How many of this device's ticks one GCR cell is. See the module docs.
const GCR_DIVISOR: u64 = 2;

/// Where the ISM's register file sits in the lock ladder.
///
/// **Below** [`LockRank::DEVICE`], because a register access takes it and then
/// reaches into the `Iwm` this chip owns, which takes its own `DEVICE`-ranked
/// lock: the ISM's phase lines, enables and `SENSE` are the mechanism's, and
/// the mechanism lives there. The reverse edge does not exist — nothing in
/// `mac.iwm` knows this register file is here — so it is a rank and not a
/// cycle (`CLAUDE.md`, *Concurrency*).
pub const ISM_RANK: LockRank = LockRank::new(0x4e00);

/// Which of the two register sets is answering, and the watcher that switches
/// between them.
#[derive(Debug, Clone, Copy, Default)]
struct Selected {
    /// `Some` while the ISM register set answers; `None` while the chip is
    /// still pretending to be an IWM.
    ism: Option<Ism>,
    /// The four-write sequence that asks for ISM mode.
    switch: Switch,
}

/// The chip's register file: the IWM's sixteen soft switches, or the ISM's
/// sixteen registers once software has asked for them.
///
/// A separate type rather than the `Iwm`'s own region, because the address
/// space has to dispatch to *this* device — a snapshot names devices by their
/// path and a machine file maps `swim`, not the IWM inside it — and because
/// this is where the two register sets are told apart.
struct Ports {
    /// The IWM's own aperture, which every access in IWM mode goes through
    /// unchanged.
    ops: Arc<dyn MemOps>,
    /// The chip the ISM shares its cable, head and medium with.
    iwm: Arc<Iwm>,
    selected: Mutex<Selected>,
}

impl core::fmt::Debug for Ports {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Swim.regs");
        match self.selected.try_lock() {
            Some(sel) => s.field("selected", &*sel).finish(),
            None => s.field("selected", &"<in use>").finish(),
        }
    }
}

impl Ports {
    /// Which of the sixteen registers an offset names. The same decode the
    /// IWM's switches use, because it is the same decode: the board puts the
    /// selects on A9-A12.
    fn index(offset: u64) -> u8 {
        ((offset / ism::REGISTER_STRIDE) & 0xf) as u8
    }

    /// Enter ISM mode, carrying the phase lines over.
    fn enter_ism(&self, sel: &mut Selected) {
        sel.switch.forget(&self.iwm);
        let ism = Ism::entered(&self.iwm);
        sel.ism = Some(ism);
        ism.apply(&self.iwm);
    }

    /// Leave it, which is what clearing mode bit 6 does.
    fn leave_ism(&self, sel: &mut Selected) {
        sel.ism = None;
        sel.switch.forget(&self.iwm);
        // Page 23: "MotorOn should be disabled before switching back to the
        // IWM register set." The chip does not enforce it and neither does
        // this, but the separator has no meaning outside ISM mode.
        self.iwm.set_mfm_framing(false, ism::CRC_SEED);
    }
}

impl MemOps for Ports {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut sel = self.selected.lock();
        if let Some(ism) = sel.ism.as_mut() {
            let [byte] = dst else {
                // Anything but a byte is the odd half of a word arriving
                // here, which the space's own policy answers; a chip on one
                // lane cannot serve it.
                return Err(BusError::BadAccess);
            };
            // The head has to be where it is at the cycle the guest looks: a
            // guest polling the handshake register is asking exactly that.
            self.iwm.sync(attrs.debug);
            if !attrs.debug {
                // `note_overrun` *takes* the separator's flag, so a debugger
                // looking at the error or handshake register would consume the
                // one thing the guest was about to be told (invariant 5).
                ism.note_overrun(&self.iwm);
            }
            *byte = ism.read(Ports::index(offset), &self.iwm, attrs.debug);
            return Ok(());
        }
        drop(sel);
        let result = self.ops.read(offset, dst, attrs);
        // A *read* of an IWM address moves a soft switch but never loads the
        // mode register, so it cannot complete the sequence — the watcher is
        // asked anyway, because what it watches is the chip's own write
        // counter and asking costs one comparison.
        if !attrs.debug {
            let mut sel = self.selected.lock();
            if sel.switch.observe(&self.iwm) {
                self.enter_ism(&mut sel);
            }
        }
        result
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut sel = self.selected.lock();
        if let Some(ism) = sel.ism.as_mut() {
            let [value] = src else {
                return Err(BusError::BadAccess);
            };
            if attrs.debug {
                // Every ISM address either loads a register or moves a
                // counter, so there is no harmless debug write — the same
                // answer the IWM's own aperture gives.
                return Err(BusError::BadAccess);
            }
            self.iwm.sync(false);
            ism.note_overrun(&self.iwm);
            let left = ism.write(Ports::index(offset), *value, &self.iwm, false);
            let after = *ism;
            if left {
                self.leave_ism(&mut sel);
            } else {
                after.apply(&self.iwm);
            }
            return Ok(());
        }
        drop(sel);
        let result = self.ops.write(offset, src, attrs);
        if !attrs.debug {
            let mut sel = self.selected.lock();
            if sel.switch.observe(&self.iwm) {
                self.enter_ism(&mut sel);
            }
        }
        result
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
    /// The register file, kept so that reset, save and load can reach which
    /// of the two register sets is selected.
    ports: Arc<Ports>,
    region: RegionRef,
    /// Ticks of this device's clock — MFM cells — simulated.
    ticks: AtomicU64,
    /// Whether what is in drive 0 is MFM, cached without a lock.
    ///
    /// [`Swim::advance_to`] runs once a scheduler round — fifty thousand times
    /// a virtual second while a disk is turning — and asking the drive would
    /// mean taking a lock and **cloning the disk**, which for a 1.44 MB image
    /// is a megabyte and a half a round. So the one bit that matters is kept
    /// here and refreshed wherever the medium can change: insert, eject, reset
    /// and load.
    mfm: AtomicBool,
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
        let image2 = r.optional_media("image2")?.map(|m| m.bytes().to_vec());
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
        // **No shipped machine file names this slot yet**, and the reason is
        // `machine::realize`'s rule rather than anything here: a slot a board
        // names and nothing binds is an error, so adding `image2 = "floppy2"`
        // to `mac-classic.machine` means every test that assembles the board
        // has to bind zero bytes for it — and one of them,
        // `tests/m68k_lift_rate.rs`, belongs to another subsystem. A test that
        // wants a disk in the external drive puts it there through
        // [`Swim::insert`], which is what the property does anyway.
        if let Some(bytes) = image2.filter(|b| !b.is_empty()) {
            if drives < 2 {
                return Err(Error::Property(alloc::format!(
                    "property `image2`: there is a disk for the second drive and `drives` is                      {drives}; a cable with one mechanism on it has nowhere to put it"
                )));
            }
            swim.insert(1, Disk::from_image_for(&bytes, Reader::Swim)?);
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
        let ports = Arc::new(Ports {
            ops,
            iwm: Arc::clone(&iwm),
            selected: Mutex::with_rank(ISM_RANK, Selected::default()),
        });
        let region = Arc::new(
            Region::io(
                CLASS_NAME,
                REGISTER_SPAN,
                Arc::clone(&ports) as Arc<dyn MemOps>,
            )
            .with_constraints(inner.constraints()),
        );
        Swim {
            iwm,
            ports,
            region,
            ticks: AtomicU64::new(0),
            mfm: AtomicBool::new(false),
        }
    }

    /// The IWM this chip is a superset of, for a caller that wants to look at
    /// the drive or the shifter.
    ///
    /// **Look, do not insert.** [`Swim::insert`] and [`Swim::eject`] refresh
    /// the density this chip's tick path reads; going round them through here
    /// would leave a 1.44 MB disk being shifted at an 800K disk's rate.
    #[must_use]
    pub fn iwm(&self) -> &Arc<Iwm> {
        &self.iwm
    }

    /// Whether the **ISM** register set is the one answering now.
    ///
    /// The chip comes up as an IWM and software asks for the ISM set with the
    /// four mode writes page 12 describes; this says which side of that the
    /// chip is on, for a test and for a trace.
    #[must_use]
    pub fn ism_selected(&self) -> bool {
        self.ports.selected.lock().ism.is_some()
    }

    /// The ISM's register file as it stands, or `None` in IWM mode.
    #[must_use]
    pub fn ism(&self) -> Option<Ism> {
        self.ports.selected.lock().ism
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
        self.note_density();
    }

    /// Take the disk out of drive `which`.
    pub fn eject(&self, which: usize) {
        self.iwm.eject(which);
        self.note_density();
    }

    /// Refresh the cached density of drive 0. Called wherever the medium can
    /// change, and never on the tick path.
    fn note_density(&self) {
        let mfm = self
            .iwm
            .disk(0)
            .is_some_and(|d| d.density() == Density::Mfm);
        self.mfm.store(mfm, Ordering::Relaxed);
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

    /// Whether what is in drive 0 is MFM, off the cached bit rather than out of
    /// the drive. The tick path's question.
    #[must_use]
    pub fn is_mfm(&self) -> bool {
        self.mfm.load(Ordering::Relaxed)
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
        if self.is_mfm() {
            tick
        } else {
            // No disk, or a GCR one: 500 kHz cells.
            tick / GCR_DIVISOR
        }
    }

    /// And the other way, so an event the IWM names lands on one of this
    /// chip's ticks.
    fn outer_tick(&self, tick: u64) -> u64 {
        if self.is_mfm() {
            tick
        } else {
            tick.saturating_mul(GCR_DIVISOR)
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
        {
            // `/RESET` "Initializes the registers in the chip" (*SWIM Chip
            // User's Reference*, page 4), and a chip that has been reset is
            // answering as an IWM again: page 12 makes the ISM set something
            // software has to ask for, four writes at a time.
            let mut sel = self.ports.selected.lock();
            sel.ism = None;
            sel.switch.forget(&self.iwm);
        }
        // A disk stays in the drive across a reset, but the cache is derived
        // state and is rebuilt rather than assumed.
        self.note_density();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.ticks.load(Ordering::Relaxed))?;
        // Which register set is answering is chip state: a machine snapshotted
        // in the middle of reading a sector comes back mid-sector.
        let selected = *self.ports.selected.lock();
        w.write_bool(selected.ism.is_some())?;
        selected.ism.unwrap_or_else(Ism::fresh).save(w)?;
        selected.switch.save(w)?;
        Device::save(&*self.iwm, w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        let live = r.read_bool()?;
        let ism = Ism::load(r)?;
        let switch = Switch::load(r)?;
        Device::load(&*self.iwm, r)?;
        *self.ports.selected.lock() = Selected {
            ism: live.then_some(ism),
            switch,
        };
        self.ticks.store(ticks, Ordering::Relaxed);
        // Derived state, never serialized (`CLAUDE.md`, *Devices*).
        self.note_density();
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
        PropertySpec {
            name: "image2",
            kind: ValueKind::Media,
            required: false,
            summary: "the same for the external drive on the cable, which needs `drives = 2`; \
                      no shipped board names it (see `Swim::new`)",
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
        .prop(PropSchema::new("image2", ValueKind::Media))
        .region("")
        .region("regs")
        .port(SEL_PIN, PortDir::In)
}

pub mod ism;

#[cfg(test)]
mod tests;
