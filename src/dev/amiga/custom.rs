//! The Amiga custom-chip register space — the decode, and the seam the chips
//! plug into.
//!
//! One class, `amiga.custom`. It is **not** a chip: it is the 512-byte window
//! at `$DFF000` that Agnus, Denise and Paula all answer inside, and the thing
//! that decides which of them a given address belongs to. The chips themselves
//! are separate objects in the machine file and are not in this build yet.
//!
//! # Why the decode is its own object
//!
//! Three chips share one aperture, and at twenty-three addresses they share a
//! *register*: `DMACON` at `$096` is written to Agnus, Denise and Paula alike,
//! `DMACONR` at `$002` — "DMA control (and blitter status) read" — is answered
//! by Agnus and Paula together, and every sprite's `POS` and `CTL` belong to
//! both Agnus and Denise. A model in which each chip maps its own scatter of
//! sub-windows would have to express that as three overlapping mappings with a
//! wired-or combine policy, re-derive the address map three times, and answer
//! the question "what is at `$07C`" differently depending on which chips the
//! board instantiated.
//!
//! So the aperture is one region with one table ([`regs`], which
//! is Appendix B of the hardware manual as data), and a chip is a *subscriber*.
//!
//! # The seam
//!
//! ```text
//!   object custom "amiga.custom" { }
//!   object agnus  "amiga.agnus"  { custom = custom }     # not in this build
//!
//!   map mem 0xDFF000 size 0x200 = custom
//! ```
//!
//! `amiga.custom` publishes an [`Arc<CustomBus>`](CustomBus) as
//! [`ExportId::CUSTOM_BUS`]. A chip names it with a link-valued property, picks
//! it up in its own `bind` with
//! [`BindCtx::export_as`](crate::machine::BindCtx::export_as), and calls
//! [`CustomBus::attach`] with an `Arc<dyn CustomChip>` of its own register
//! block. The consumer names its source in the machine file, which is the rule
//! (`ROADMAP.md` §4.4) and the reason a two-Agnus board would be an error
//! rather than a coin toss.
//!
//! A chip therefore implements exactly this:
//!
//! ```ignore
//! impl CustomChip for AgnusRegs {
//!     fn which(&self) -> ChipId { ChipId::AGNUS }
//!     fn read(&self, reg: &Reg, from: Origin) -> u16 { /* … */ }
//!     fn write(&self, reg: &Reg, value: u16, from: Origin) { /* … */ }
//! }
//! ```
//!
//! and nothing else. By the time either method is called the bus has already:
//!
//! * rejected a byte or odd access, and an offset past `$1FE`;
//! * looked the offset up in the appendix's table, so `reg.name`, `reg.chip`
//!   and `reg.access` are to hand and no chip re-derives them;
//! * established that this chip is one of the register's owners;
//! * established that the direction is one the appendix allows — a read only
//!   reaches a chip for a register marked `R`, a write only for one marked `W`
//!   or `S`;
//! * checked the copper's privilege against the `*` and `~` columns, if the
//!   write came from the copper.
//!
//! Read results from several owners are **or**ed, because that is what several
//! chips driving one 16-bit bus does and what `DMACONR` needs. Writes are
//! delivered to every owner, in Agnus-Denise-Paula order.
//!
//! The **copper** drives this same seam from the other side: Agnus's copper
//! holds the same `Arc<CustomBus>` its register block was attached through and
//! calls [`CustomBus::write`] with [`Origin::copper`], so the `COPCON` danger
//! rule is enforced in one place rather than inside Agnus.
//!
//! # Access width
//!
//! Every entry in the appendix is a **word**, so the region declares exactly
//! that: 16 bits, naturally aligned, big-endian, no bulk transfers. A byte
//! access is refused by [`AccessConstraints`] before any handler runs.
//!
//! That is a deliberate refusal rather than a modelling decision: what a
//! single-byte cycle does to a word-wide custom register is not in the manual.
//! Refusing it makes a guest that tries one fail loudly here, where the ledger
//! can record it, instead of quietly getting a value somebody guessed. A later
//! round with a better source than the appendix should replace the refusal,
//! not work around it.
//!
//! # What a read of a write-only register answers
//!
//! **This is a placeholder and is the one piece of invented behaviour in this
//! file.** The appendix says which registers are readable and does not say what
//! the other two hundred do when read. On real hardware the answer is whatever
//! Agnus last drove onto the chip data bus, which is a function of the DMA
//! cycle that just happened — and there is no DMA here yet.
//!
//! So [`CustomBus`] keeps a single `floating` word, updated by every write that
//! reaches it, and answers an unreadable or unclaimed address with that. It is
//! deterministic, it is snapshot-carried, and it is wrong in a way that will
//! only become visible once Agnus drives real cycles. [`CustomBus::unclaimed`]
//! counts how often it has been relied on, so a board can assert it is zero.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//! Appendix B ("Register Summary — Address Order") for the table and its
//! legend, Appendix D ("System Memory Maps") for the `$DFF000` base. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1); every Amiga
//! emulator the author is aware of is GPL and is off limits.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

use crate::core::device::{Device, DeviceClass, Export, ExportId, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

use super::regs::{self, ChipId, Reg, SPAN, copper_may_write};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.custom";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// who is driving the access
// ---------------------------------------------------------------------------

/// What is making an access into the register space.
///
/// Three drivers rather than one because the register file answers them
/// differently: the copper's writes are filtered by the appendix's `*` and `~`
/// columns, and a chip's own DMA cycle writes registers (`BPL1DAT`, `AUD0DAT`)
/// that the appendix marks `&` — used by a DMA channel only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Driver {
    /// The 68000, through the `$DFF000` window.
    Cpu,
    /// Agnus's copper, executing a `MOVE` instruction. `danger` is `COPCON`'s
    /// copper danger bit, which the copper knows and the bus does not.
    Copper {
        /// Whether `COPCON`'s danger bit is set.
        danger: bool,
    },
    /// A chip's own DMA channel, writing the register its transfer lands in.
    Dma,
}

/// One access's provenance: who, and whether it must have no side effects.
///
/// A struct rather than two arguments so that a later round can add a fact —
/// the beam position, say — without every chip model changing signature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Origin {
    /// What is driving the access.
    pub driver: Driver,
    /// The access comes from a debugger and **must have no side effects**
    /// (`ROADMAP.md` §15, invariant 5) — no `CLXDAT` clear, no `INTREQR`
    /// latch, no pointer advance.
    pub debug: bool,
}

impl Origin {
    /// The processor, in the ordinary way.
    #[must_use]
    pub const fn cpu() -> Origin {
        Origin {
            driver: Driver::Cpu,
            debug: false,
        }
    }

    /// The copper, with `COPCON`'s danger bit in the state it is in.
    #[must_use]
    pub const fn copper(danger: bool) -> Origin {
        Origin {
            driver: Driver::Copper { danger },
            debug: false,
        }
    }

    /// A DMA channel's own transfer.
    #[must_use]
    pub const fn dma() -> Origin {
        Origin {
            driver: Driver::Dma,
            debug: false,
        }
    }

    /// The same origin, marked as a debugger's access.
    #[must_use]
    pub const fn for_debug(mut self, debug: bool) -> Origin {
        self.debug = debug;
        self
    }
}

// ---------------------------------------------------------------------------
// what a chip implements
// ---------------------------------------------------------------------------

/// A custom chip, as the register space sees it.
///
/// Implement this on the chip's **register block** — the `Arc`-held inner
/// struct, not the `Device` — for the same reason `st.syscfg` does: `bind`
/// takes `&self` and cannot hand out an `Arc` of the device, while an inner
/// block the device already holds by `Arc` can be cloned into the bus.
///
/// Neither method returns a `Result`. A register that does not exist, that the
/// caller may not touch, or that this chip does not own never reaches here —
/// see the module documentation for the full list of what the bus has already
/// settled.
pub trait CustomChip: Send + Sync + fmt::Debug {
    /// Which chip this is. One bit: a model that claims two is rejected at
    /// attach.
    fn which(&self) -> ChipId;

    /// Read the register the appendix calls `reg.name`.
    ///
    /// Only ever called for a register marked `R` that this chip owns. Where
    /// two chips own one readable register the results are **or**ed, so a chip
    /// must return zeroes in the bits it does not drive rather than a whole
    /// word of its own guess.
    fn read(&self, reg: &Reg, from: Origin) -> u16;

    /// Write the register the appendix calls `reg.name`.
    ///
    /// Only ever called for a register marked `W` or `S` that this chip owns,
    /// and for a copper write only when the `*`/`~` columns allow it. For a
    /// strobe the `value` is meaningless — the address is the event.
    fn write(&self, reg: &Reg, value: u16, from: Origin);
}

// ---------------------------------------------------------------------------
// the bus
// ---------------------------------------------------------------------------

/// The custom-chip register bus: the table, the subscribers, and the decode.
///
/// Held by `amiga.custom`, published as [`ExportId::CUSTOM_BUS`], and reached
/// by the address space on one side and by the copper on the other.
pub struct CustomBus {
    /// The attached chips, at most one per [`ChipId`] bit.
    ///
    /// A `Mutex` and not an `RwLock` because it is written once per machine at
    /// bind and read on every access; the read path clones the handful of
    /// `Arc`s it needs and releases the lock **before** calling into any chip,
    /// which is the re-entrancy contract (`CLAUDE.md`, *Concurrency*) — a chip
    /// write can reach the copper, which writes here again.
    chips: Mutex<Vec<Arc<dyn CustomChip>>>,
    /// The last word driven onto the chip data bus, as far as this model knows
    /// it. See the module documentation: a placeholder for Agnus's DMA.
    floating: AtomicU64,
    /// How many accesses have fallen through to [`floating`](Self::floating) —
    /// an unclaimed offset, an unreadable register, or a register whose owner
    /// is not in this build.
    unclaimed: AtomicU64,
    /// How many copper writes the `*`/`~` columns have refused.
    refused: AtomicU64,
}

impl fmt::Debug for CustomBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CustomBus")
            .field("chips", &self.chips.lock().len())
            .field("floating", &self.floating.load(Ordering::Relaxed))
            .field("unclaimed", &self.unclaimed.load(Ordering::Relaxed))
            .field("refused", &self.refused.load(Ordering::Relaxed))
            .finish()
    }
}

impl CustomBus {
    /// An empty bus: the table, and nobody subscribed to it.
    fn new() -> CustomBus {
        CustomBus {
            // LEAF: taken inside an MMIO handler, released before any chip is
            // called, and holding nothing else.
            chips: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            floating: AtomicU64::new(0),
            unclaimed: AtomicU64::new(0),
            refused: AtomicU64::new(0),
        }
    }

    /// Subscribe a chip to the registers the appendix says it owns.
    ///
    /// Called from the chip's own `bind`, once. The chip is asked
    /// [`CustomChip::which`] rather than told, so a machine file cannot wire
    /// Denise into Agnus's slot.
    ///
    /// # Errors
    ///
    /// If the chip claims no chip or more than one, or if something has already
    /// claimed that chip — a board with two Agnuses is a mistake, not a
    /// feature.
    pub fn attach(&self, chip: Arc<dyn CustomChip>) -> Result<()> {
        let which = chip.which();
        if which.0.count_ones() != 1 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "a custom chip claims to be `{which}`; it has to be exactly one of Agnus, \
                     Denise or Paula"
                ),
            });
        }
        let mut chips = self.chips.lock();
        if chips.iter().any(|c| c.which() == which) {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!("{which} is already attached to this custom-chip space"),
            });
        }
        chips.push(chip);
        // Agnus, then Denise, then Paula — the appendix's own order, so a
        // register two chips latch is delivered in a fixed sequence rather than
        // in the order the machine file happened to declare them
        // (`CLAUDE.md`, *Determinism*).
        chips.sort_by_key(|c| c.which());
        Ok(())
    }

    /// Which chips are attached, as a set.
    #[must_use]
    pub fn attached(&self) -> ChipId {
        self.chips
            .lock()
            .iter()
            .fold(ChipId::NONE, |set, c| set.union(c.which()))
    }

    /// The last word this model believes was driven on the chip data bus.
    #[must_use]
    pub fn floating(&self) -> u16 {
        self.floating.load(Ordering::Relaxed) as u16
    }

    /// How many accesses have fallen through to the floating word.
    ///
    /// A board with every chip present and a guest that only touches real
    /// registers leaves this at zero; anything else is a measurement of how
    /// much of the chipset is still missing.
    #[must_use]
    pub fn unclaimed(&self) -> u64 {
        self.unclaimed.load(Ordering::Relaxed)
    }

    /// How many copper writes have been refused by the `*` or `~` columns.
    #[must_use]
    pub fn refused_copper_writes(&self) -> u64 {
        self.refused.load(Ordering::Relaxed)
    }

    /// The owners of `reg` that are actually attached, cloned out so the lock
    /// is released before any of them is called.
    fn owners(&self, reg: &Reg) -> Vec<Arc<dyn CustomChip>> {
        self.chips
            .lock()
            .iter()
            .filter(|c| reg.chip.contains(c.which()))
            .map(Arc::clone)
            .collect()
    }

    /// Read the word register at `offset`.
    ///
    /// `offset` is relative to the custom-chip base. Anything the appendix does
    /// not declare readable — an unclaimed offset, a write-only register, a
    /// register whose chip this build does not have — answers with the floating
    /// word and moves [`unclaimed`](Self::unclaimed).
    pub fn read(&self, offset: u16, from: Origin) -> u16 {
        let Some(reg) = regs::lookup(offset).filter(|r| r.readable()) else {
            return self.fall_through(from);
        };
        let owners = self.owners(reg);
        if owners.is_empty() {
            return self.fall_through(from);
        }
        // Wired-or, which is what several chips driving one 16-bit bus is. Each
        // owner returns zeroes in the bits it does not drive (`CustomChip::read`).
        let value = owners.iter().fold(0u16, |acc, c| acc | c.read(reg, from));
        if !from.debug {
            self.floating.store(u64::from(value), Ordering::Relaxed);
        }
        value
    }

    /// Write the word register at `offset`.
    ///
    /// Returns whether anything took it: `false` for an unclaimed offset, a
    /// read-only register, a copper write the `*`/`~` columns refuse, or a
    /// register whose chip this build does not have.
    pub fn write(&self, offset: u16, value: u16, from: Origin) -> bool {
        // The data bus carries the word whether or not anything latches it,
        // which is the half of the floating-word model that is not a guess.
        if !from.debug {
            self.floating.store(u64::from(value), Ordering::Relaxed);
        }
        let Some(reg) = regs::lookup(offset).filter(|r| r.writable()) else {
            if !from.debug {
                self.unclaimed.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        };
        if let Driver::Copper { danger } = from.driver
            && !copper_may_write(reg.access, danger)
        {
            if !from.debug {
                self.refused.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        }
        let owners = self.owners(reg);
        if owners.is_empty() {
            if !from.debug {
                self.unclaimed.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        }
        for chip in &owners {
            chip.write(reg, value, from);
        }
        true
    }

    /// Answer with the floating word, counting it unless this is a debugger.
    fn fall_through(&self, from: Origin) -> u16 {
        if !from.debug {
            self.unclaimed.fetch_add(1, Ordering::Relaxed);
        }
        self.floating()
    }

    /// Back to power-on: nothing driven, nothing counted.
    ///
    /// The subscriber list is **wiring, not guest state**
    /// ([`Device::export`]'s contract) and survives.
    fn reset(&self) {
        self.floating.store(0, Ordering::Relaxed);
        self.unclaimed.store(0, Ordering::Relaxed);
        self.refused.store(0, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// the aperture
// ---------------------------------------------------------------------------

/// The `$DFF000` window, as something an address space can dispatch to.
#[derive(Debug)]
struct Window {
    bus: Arc<CustomBus>,
}

impl MemOps for Window {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 2 || offset & 1 != 0 || offset >= SPAN {
            return Err(BusError::BadAccess);
        }
        let from = Origin::cpu().for_debug(attrs.debug);
        // Big-endian on the wire; `AccessConstraints` says so too, so the
        // dispatcher will not have reordered anything for us.
        let value = self.bus.read(offset as u16, from);
        dst.copy_from_slice(&value.to_be_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 2 || offset & 1 != 0 || offset >= SPAN {
            return Err(BusError::BadAccess);
        }
        let from = Origin::cpu().for_debug(attrs.debug);
        let value = u16::from_be_bytes([src[0], src[1]]);
        self.bus.write(offset as u16, value, from);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Word, aligned, big-endian, no bursts. Every entry in the appendix is
        // a word, and a byte access has no documented meaning; see the module
        // docs.
        AccessConstraints::word(Width::U16, Endian::Big)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The custom-chip register space at `$DFF000`.
#[derive(Debug)]
pub struct Custom {
    bus: Arc<CustomBus>,
    region: RegionRef,
}

impl Custom {
    /// A register space with nothing subscribed to it.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<Custom> {
        props.reader().finish()?;
        let bus = Arc::new(CustomBus::new());
        let window = Arc::new(Window {
            bus: Arc::clone(&bus),
        });
        let region = Arc::new(Region::io("amiga.custom", SPAN, window as Arc<dyn MemOps>));
        Ok(Custom { bus, region })
    }

    /// The bus, for a chip that is attaching to it or a test that is watching
    /// it.
    #[must_use]
    pub fn bus(&self) -> &Arc<CustomBus> {
        &self.bus
    }
}

impl Device for Custom {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and each chip
        // subscribes from its own `bind`.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. There is no battery behind any of this, and the chips
        // reset themselves — what is here is the bus between them.
        self.bus.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u16(self.bus.floating())?;
        w.write_u64(self.bus.unclaimed())?;
        w.write_u64(self.bus.refused_copper_writes())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let floating = r.read_u16()?;
        let unclaimed = r.read_u64()?;
        let refused = r.read_u64()?;
        self.bus
            .floating
            .store(u64::from(floating), Ordering::Relaxed);
        self.bus.unclaimed.store(unclaimed, Ordering::Relaxed);
        self.bus.refused.store(refused, Ordering::Relaxed);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.region)),
            _ => None,
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::CUSTOM_BUS)
            .then(|| Export::Opaque(Arc::clone(&self.bus) as Arc<dyn Any + Send + Sync>))
    }
}

impl Instance for Custom {}

/// The `amiga.custom` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Amiga custom-chip register space at $DFF000: the decode Agnus, Denise and \
              Paula answer inside",
    properties: &[],
    construct: |props| Ok(Box::new(Custom::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Custom::new(props)?)))
}

/// What the validator should know about `amiga.custom`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME).region("").region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec;

    /// A stand-in chip that records what reached it and answers reads with a
    /// fixed pattern. Obviously not a model of anything: it exists to prove the
    /// routing, and the routing is all this file promises.
    #[derive(Debug)]
    struct Probe {
        which: ChipId,
        answer: u16,
        seen: Mutex<Vec<(u16, Option<u16>, Origin)>>,
    }

    impl Probe {
        fn new(which: ChipId, answer: u16) -> Arc<Probe> {
            Arc::new(Probe {
                which,
                answer,
                seen: Mutex::new(Vec::new()),
            })
        }

        fn seen(&self) -> Vec<(u16, Option<u16>, Origin)> {
            self.seen.lock().clone()
        }
    }

    impl CustomChip for Probe {
        fn which(&self) -> ChipId {
            self.which
        }

        fn read(&self, reg: &Reg, from: Origin) -> u16 {
            self.seen.lock().push((reg.offset, None, from));
            self.answer
        }

        fn write(&self, reg: &Reg, value: u16, from: Origin) {
            self.seen.lock().push((reg.offset, Some(value), from));
        }
    }

    /// A stand-in that records only the order it was written in.
    #[derive(Debug)]
    struct Ordered {
        which: ChipId,
        order: Arc<Mutex<Vec<ChipId>>>,
    }

    impl CustomChip for Ordered {
        fn which(&self) -> ChipId {
            self.which
        }

        fn read(&self, _: &Reg, _: Origin) -> u16 {
            0
        }

        fn write(&self, _: &Reg, _: u16, _: Origin) {
            self.order.lock().push(self.which);
        }
    }

    fn custom() -> Custom {
        Custom::new(&Props::new()).expect("no properties to get wrong")
    }

    /// `DMACONR`: readable, Agnus and Paula.
    const DMACONR: u16 = 0x002;
    /// `DMACON`: writable, all three.
    const DMACON: u16 = 0x096;
    /// `COLOR00`: writable, Denise only.
    const COLOR00: u16 = 0x180;
    /// `COPCON`: writable, below $040, so never by the copper.
    const COPCON: u16 = 0x02e;
    /// `BLTCON0`: writable, in the band the copper needs CDANG for.
    const BLTCON0: u16 = 0x040;

    #[test]
    fn a_read_owned_by_two_chips_is_the_or_of_both() {
        let c = custom();
        let agnus = Probe::new(ChipId::AGNUS, 0x4000);
        let paula = Probe::new(ChipId::PAULA, 0x0201);
        c.bus().attach(paula.clone()).unwrap();
        c.bus().attach(agnus.clone()).unwrap();
        assert_eq!(c.bus().read(DMACONR, Origin::cpu()), 0x4201);
        assert_eq!(agnus.seen().len(), 1);
        assert_eq!(paula.seen().len(), 1);
        assert_eq!(c.bus().unclaimed(), 0);
    }

    #[test]
    fn a_write_owned_by_three_chips_reaches_each_in_table_order() {
        let c = custom();
        let order = Arc::new(Mutex::new(Vec::new()));
        // Attached out of order on purpose: delivery follows the appendix, not
        // the machine file.
        for which in [ChipId::PAULA, ChipId::AGNUS, ChipId::DENISE] {
            c.bus()
                .attach(Arc::new(Ordered {
                    which,
                    order: Arc::clone(&order),
                }))
                .unwrap();
        }
        assert!(c.bus().write(DMACON, 0x8200, Origin::cpu()));
        assert_eq!(
            *order.lock(),
            vec![ChipId::AGNUS, ChipId::DENISE, ChipId::PAULA]
        );
    }

    #[test]
    fn a_chip_hears_only_its_own_registers_in_directions_the_table_allows() {
        let c = custom();
        let agnus = Probe::new(ChipId::AGNUS, 0xffff);
        c.bus().attach(agnus.clone()).unwrap();

        // Denise's register: not Agnus's business.
        assert!(!c.bus().write(COLOR00, 0x0f00, Origin::cpu()));
        // A write to a read-only register, and a read of a write-only one.
        assert!(!c.bus().write(DMACONR, 0x1234, Origin::cpu()));
        let _ = c.bus().read(DMACON, Origin::cpu());
        assert!(agnus.seen().is_empty(), "{:?}", agnus.seen());
        assert_eq!(c.bus().unclaimed(), 3);
    }

    #[test]
    fn an_unreadable_address_answers_with_the_floating_word() {
        let c = custom();
        assert!(!c.bus().write(COLOR00, 0x0abc, Origin::cpu()));
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0x0abc);
        // An offset the appendix leaves blank, and `$1FE`, which it does not
        // list either.
        assert_eq!(c.bus().read(0x068, Origin::cpu()), 0x0abc);
        assert_eq!(c.bus().read(0x1fe, Origin::cpu()), 0x0abc);
        assert_eq!(c.bus().unclaimed(), 4);
    }

    #[test]
    fn the_copper_is_held_to_the_star_and_tilde_columns() {
        let c = custom();
        let agnus = Probe::new(ChipId::AGNUS, 0);
        c.bus().attach(agnus.clone()).unwrap();

        // Below $040: never, danger bit or not.
        assert!(!c.bus().write(COPCON, 2, Origin::copper(false)));
        assert!(!c.bus().write(COPCON, 2, Origin::copper(true)));
        // $040-$07E: only with the danger bit.
        assert!(!c.bus().write(BLTCON0, 1, Origin::copper(false)));
        assert!(c.bus().write(BLTCON0, 1, Origin::copper(true)));
        // $080 and up: always. COP1LCH.
        assert!(c.bus().write(0x080, 0, Origin::copper(false)));
        // And the processor is not the copper.
        assert!(c.bus().write(COPCON, 2, Origin::cpu()));

        assert_eq!(c.bus().refused_copper_writes(), 3);
        let offsets: Vec<u16> = agnus.seen().iter().map(|s| s.0).collect();
        assert_eq!(offsets, vec![BLTCON0, 0x080, COPCON]);
    }

    #[test]
    fn a_debugger_moves_no_counter_and_disturbs_no_bus() {
        let c = custom();
        let paula = Probe::new(ChipId::PAULA, 0x0010);
        c.bus().attach(paula.clone()).unwrap();
        c.bus().write(COLOR00, 0x0555, Origin::cpu());
        let unclaimed = c.bus().unclaimed();

        let debug = Origin::cpu().for_debug(true);
        assert_eq!(c.bus().read(COLOR00, debug), 0x0555);
        assert_eq!(c.bus().read(DMACONR, debug), 0x0010);
        c.bus().write(COLOR00, 0x0999, debug);

        assert_eq!(c.bus().unclaimed(), unclaimed);
        assert_eq!(c.bus().floating(), 0x0555);
        // The chip was told it was a debugger, which is the half of the
        // contract only the chip can keep.
        assert!(paula.seen().iter().all(|s| s.2.debug));
    }

    #[test]
    fn attach_refuses_a_second_chip_of_a_kind_and_a_chip_that_is_two() {
        let c = custom();
        c.bus().attach(Probe::new(ChipId::AGNUS, 0)).unwrap();
        assert!(c.bus().attach(Probe::new(ChipId::AGNUS, 0)).is_err());
        assert!(
            c.bus()
                .attach(Probe::new(ChipId::DENISE | ChipId::PAULA, 0))
                .is_err()
        );
        assert!(c.bus().attach(Probe::new(ChipId::NONE, 0)).is_err());
        assert_eq!(c.bus().attached(), ChipId::AGNUS);
    }

    #[test]
    fn the_window_takes_words_and_nothing_else() {
        let c = custom();
        assert_eq!(c.region("").unwrap().len(), 0x200);
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };

        ops.write(0x180, &[0x0f, 0x00], MemAttrs::DEFAULT).unwrap();
        let mut word = [0u8; 2];
        ops.read(0x180, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(word, [0x0f, 0x00], "big-endian on the wire");

        let mut byte = [0u8; 1];
        assert!(ops.read(0x180, &mut byte, MemAttrs::DEFAULT).is_err());
        assert!(ops.read(0x181, &mut word, MemAttrs::DEFAULT).is_err());
        assert!(ops.read(0x200, &mut word, MemAttrs::DEFAULT).is_err());
        let k = ops.constraints();
        assert_eq!(
            (k.min, k.max, k.endian),
            (Width::U16, Width::U16, Endian::Big)
        );
    }

    #[test]
    fn the_bus_is_published_and_the_region_is_named() {
        let c = custom();
        let export = c.export(ExportId::CUSTOM_BUS).expect("published");
        let bus = Arc::clone(export.opaque().unwrap())
            .downcast::<CustomBus>()
            .expect("a CustomBus");
        assert!(Arc::ptr_eq(&bus, c.bus()));
        assert!(c.export(ExportId::TIMEBASE).is_none());
        assert!(c.region("regs").is_some());
        assert!(c.region("agnus").is_none());
    }

    #[test]
    fn a_reset_clears_the_bus_and_keeps_the_subscribers() {
        let c = custom();
        c.bus().attach(Probe::new(ChipId::DENISE, 0)).unwrap();
        c.bus().write(0x068, 0x1234, Origin::cpu());
        Device::reset(&c, ResetKind::Cold);
        assert_eq!(c.bus().floating(), 0);
        assert_eq!(c.bus().unclaimed(), 0);
        assert_eq!(c.bus().attached(), ChipId::DENISE, "wiring, not state");
    }

    fn snapshot(c: &Custom) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("custom", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("custom", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(c, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = custom();
        saved.bus().attach(Probe::new(ChipId::AGNUS, 0)).unwrap();
        saved.bus().write(0x068, 0x0bad, Origin::cpu());
        saved.bus().write(COPCON, 0, Origin::copper(false));
        let bytes = snapshot(&saved);

        let restored = custom();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("custom", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        assert_eq!(
            snapshot(&restored),
            bytes,
            "identical state after a round trip"
        );
        assert_eq!(
            restored.bus().floating(),
            0x0000,
            "the copper's refused word"
        );
        assert_eq!(restored.bus().unclaimed(), 1);
        assert_eq!(restored.bus().refused_copper_writes(), 1);
    }

    #[test]
    fn the_class_constructs_through_the_registry_and_rejects_a_typo() {
        let mut reg = crate::core::Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        assert_eq!(
            reg.create(CLASS_NAME, &Props::new()).unwrap().class().name,
            CLASS_NAME
        );
        let typo = Props::new().with("agnus", crate::core::props::Value::from(1u64));
        assert!(Custom::new(&typo).is_err());
    }
}
