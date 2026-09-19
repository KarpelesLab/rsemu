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
//! * turned a byte access into the word access the chips actually see (see
//!   *Access width* below), and rejected a word at an odd offset and an offset
//!   past `$1FE`;
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
//! # Access width, and what a byte access does
//!
//! Every entry in the appendix is a word, and the region takes words. It also
//! takes **bytes**, because a 68000 can make a byte access to any address and
//! the A500 does nothing to stop one reaching the chips. Kickstart 2.04 makes
//! one: a byte read of `$DFF07D`, the low half of `DENISEID`. What happens then
//! is settled by three documents, none of which is Appendix B:
//!
//! * **The chips never see a data strobe.** Denise and Paula have no `UDS`,
//!   `LDS` or even `R/W` pin: their register interface is `D15`–`D0` and
//!   `RGA8`–`RGA1` and nothing else (*Amiga Hardware Reference Manual*, 3rd
//!   ed., Appendix J, "Custom Chip Pin Allocation List"). Agnus has `UDS*` and
//!   `LDS*`, and its own pin description says each "is enabled only during a
//!   processor DRAM access", where it picks `CASU*` or `CASL*`; a register
//!   access is `AS*` and `RGEN*` with `A1`–`A8` (*A500/A2000 Technical
//!   Reference Manual*, Commodore, Table 6-1 and the Fat Agnus description
//!   after it).
//! * **The board buffers all sixteen lines on one enable.** On the A500
//!   (schematic #312511-02, sheet 2 of 9) the processor's data bus reaches the
//!   chip data bus through two 74LS244s — `U12` for the upper byte, `U10` for
//!   the lower — whose output enables are both Gary's `_OEB`, and comes back
//!   through two 74LS373s (`U13`, `U11`) that share Gary's `_OEL` and
//!   `_LATCH`. There is no per-byte enable that could leave half the bus
//!   undriven. The A2000's PAL listing of the same logic (Technical Reference
//!   Manual §7.3, `/CDR` and `/CDW`) enables the chip data buffers from the
//!   register decode alone; the strobes gate only `UCEN`/`LCEN`, the RAM's CAS.
//! * **A byte write drives the byte onto both halves.** MC68000 User's Manual
//!   (M68000UM/AD rev. 8), Table 3-1, *Data Strobe Control of Data Bus*: with
//!   `R/W` low and only `LDS` asserted, `D15`–`D8` carry "Valid Data Bits
//!   7–0" as well as `D7`–`D0`; with only `UDS`, `D7`–`D0` carry bits 15–8.
//!   The table marks those two rows as "a result of current implementation",
//!   and that implementation is the part an A500 carries.
//!
//! So:
//!
//! * **A byte read** is a word read of the register — with every side effect a
//!   word read has, since the chip cannot tell the difference — and the
//!   processor keeps the half its strobe selected (§5.1.1: it "internally
//!   positions the byte appropriately"): the upper byte at an even address,
//!   the lower at an odd one.
//! * **A byte write** is a word write of the register with **the same byte in
//!   both halves**, whichever of the two addresses it was made at.
//!   `MOVE.B #$20,$DFF09B` stores `$2020` — not `$0020`, and not `$20` into the
//!   low half with the high half kept. That is the dangerous half of the rule
//!   and it is the hardware's: a register cannot keep the half nobody
//!   addressed, because nothing tells it which half that was.
//!
//! A word at an odd offset, anything wider than a word, and any offset past
//! `$1FE` are still refused.
//!
//! # What a read of a write-only register answers
//!
//! The appendix says which registers are readable and does not say what the
//! other two hundred do when read. Nothing answers: no chip drives `D15`–`D0`
//! in that cycle, and the processor latches the lines as they are. Appendix C
//! (p. 299) calls that "whatever value is left over on the bus from the last
//! cycle", and makes it the documented way to tell an 8362 — which has no
//! `DENISEID` — from an 8373, which does.
//!
//! So [`CustomBus`] invents nothing and keeps nothing of its own. It holds the
//! [`ChipDataBus`] Agnus's DMA drives, handed over at bind, drives it from
//! this side too — every write, and every read a chip answers, is a cycle that
//! puts a word on those sixteen lines — and answers an unreadable or unclaimed
//! address out of it. A read **nothing** answers is a cycle nothing drove, so
//! it takes the word rather than leaving it: the lines are floating
//! afterwards, and a second such read in a row does *not* get the same answer.
//! That is the whole point of Appendix C's test, and it is why a placeholder
//! that answered stably made Kickstart 2.04 find an ECS Denise on an A500.
//! [`ChipDataBus`]'s own documentation has the full rule and marks what in it
//! is inference and what is choice.
//!
//! [`CustomBus::unclaimed`] counts how often that fall-through has been relied
//! on, so a board can assert it is zero.
//!
//! A **debug** access is not a cycle: it drives nothing, takes nothing and is
//! not counted. A debugger's read of a write-only register shows the guest
//! exactly what it would have latched and leaves the lines as it found them.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//! Appendix B ("Register Summary — Address Order") for the table and its
//! legend, Appendix D ("System Memory Maps") for the `$DFF000` base, Appendix C
//! (p. 299) for an absent `DENISEID`, Appendix J for the chips' pins. For byte
//! access: the *A500/A2000 Technical Reference Manual* (Commodore), Table 6-1
//! and §7.3; A500 schematic #312511-02 rev. 5, sheet 2; MC68000 User's Manual
//! (M68000UM/AD rev. 8), Table 3-1 and §5.1. No emulator source of any licence
//! was consulted (`ROADMAP.md` §1); every Amiga emulator the author is aware of
//! is GPL and is off limits.

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

use super::dma::ChipDataBus;
use super::regs::{self, ChipId, Reg, SPAN, copper_may_write, ecs_copper_may_write};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.custom";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 2;

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
        /// Whether the copper is an Enhanced Chip Set Agnus's, which Appendix
        /// C holds to a wider rule than the `*`/`~` columns
        /// ([`regs::ecs_copper_may_write`]).
        ecs: bool,
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
            driver: Driver::Copper { danger, ecs: false },
            debug: false,
        }
    }

    /// An Enhanced Chip Set Agnus's copper, with `COPCON`'s danger bit in the
    /// state it is in.
    #[must_use]
    pub const fn ecs_copper(danger: bool) -> Origin {
        Origin {
            driver: Driver::Copper { danger, ecs: true },
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
    /// Only ever called for a register marked `R` that this chip owns **and
    /// [`drives`](Self::drives)**. Where two chips own one readable register
    /// the results are **or**ed, so a chip must return zeroes in the bits it
    /// does not drive rather than a whole word of its own guess.
    fn read(&self, reg: &Reg, from: Origin) -> u16;

    /// Whether this part drives `D15`–`D0` at all when `reg` is read.
    ///
    /// The appendix's table is the same for every part; a *part* can be
    /// missing a register the table gives its chip. An 8362 has no `DENISEID`
    /// — "The original Denise (8362) does not have this register" (Appendix C,
    /// p. 299) — so it answers by not driving, and the bus falls through to
    /// the chip data lines, which is what makes Commodore's own test work.
    ///
    /// The default is `true`: a chip drives every readable register it owns.
    /// Only consulted on a read.
    fn drives(&self, _reg: &Reg) -> bool {
        true
    }

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
    /// The chips and the sixteen data lines, behind **one** lock.
    ///
    /// Both are written once per machine at bind and read on every access, and
    /// keeping them together is what holds an access to one lock acquisition:
    /// [`wiring_for`](Self::wiring_for) takes it, clones out the handful of
    /// `Arc`s the access needs, and releases it **before** any chip is called,
    /// which is the re-entrancy contract (`CLAUDE.md`, *Concurrency*) — a chip
    /// write can reach the copper, which writes here again.
    ///
    /// A `Mutex` and not an `RwLock` because the critical section is a `Vec`
    /// filter and two `Arc` clones.
    wiring: Mutex<Wiring>,
    /// How many accesses have fallen through to the chip data bus — an
    /// unclaimed offset, an unreadable register, or a register whose owner is
    /// not in this build.
    unclaimed: AtomicU64,
    /// How many copper writes the `*`/`~` columns have refused.
    refused: AtomicU64,
}

/// What a machine file wired into this bus: the chips, and the data lines they
/// all share.
///
/// **Wiring, not guest state** ([`Device::export`]'s contract): it survives a
/// reset and is not snapshotted. The *word* on those lines is guest state, and
/// belongs to the chip that drives it.
#[derive(Debug, Default)]
struct Wiring {
    /// The attached chips, at most one per [`ChipId`] bit, in Appendix B's
    /// Agnus-Denise-Paula order.
    chips: Vec<Arc<dyn CustomChip>>,
    /// `D15`–`D0`, driven by Agnus's DMA and by this bus. `None` on a board
    /// with no DMA engine, which runs no chip-bus cycles at all.
    data: Option<Arc<ChipDataBus>>,
}

impl fmt::Debug for CustomBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Taken and released *before* the builder, not inside it: a guard in a
        // method chain lives to the end of the statement.
        let (chips, floating) = {
            let w = self.wiring.lock();
            (w.chips.len(), w.data.as_ref().map_or(0, |d| d.word()))
        };
        f.debug_struct("CustomBus")
            .field("chips", &chips)
            .field("floating", &floating)
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
            wiring: Mutex::with_rank(LockRank::LEAF, Wiring::default()),
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
        let mut wiring = self.wiring.lock();
        if wiring.chips.iter().any(|c| c.which() == which) {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!("{which} is already attached to this custom-chip space"),
            });
        }
        wiring.chips.push(chip);
        // Agnus, then Denise, then Paula — the appendix's own order, so a
        // register two chips latch is delivered in a fixed sequence rather than
        // in the order the machine file happened to declare them
        // (`CLAUDE.md`, *Determinism*).
        wiring.chips.sort_by_key(|c| c.which());
        Ok(())
    }

    /// Which chips are attached, as a set.
    #[must_use]
    pub fn attached(&self) -> ChipId {
        self.wiring
            .lock()
            .chips
            .iter()
            .fold(ChipId::NONE, |set, c| set.union(c.which()))
    }

    /// Give the register space the chip data bus a DMA engine drives.
    ///
    /// Agnus calls this from its own bind, with the bus its [`ChipDma`] holds.
    /// It is **wiring, not guest state**: it survives a reset and is not
    /// snapshotted, and the word itself belongs to the chip that drives it.
    ///
    /// [`ChipDma`]: super::dma::ChipDma
    pub fn attach_data_bus(&self, data: Arc<ChipDataBus>) {
        self.wiring.lock().data = Some(data);
    }

    /// The word on `D15`–`D0`, looked at without touching it: what a guest
    /// read of a write-only register would be answered with, and what a
    /// **debugger's** read is answered with. Zero on a board with no DMA
    /// engine, which runs no chip-bus cycles.
    #[must_use]
    pub fn floating(&self) -> u16 {
        self.wiring.lock().data.as_ref().map_or(0, |d| d.word())
    }

    /// How many accesses have fallen through to the chip data bus.
    ///
    /// A board with every chip present and a guest that only touches real
    /// registers leaves this at zero; anything else is a measurement of how
    /// much of the chipset is still missing, or of a guest reading a
    /// write-only register.
    #[must_use]
    pub fn unclaimed(&self) -> u64 {
        self.unclaimed.load(Ordering::Relaxed)
    }

    /// How many copper writes have been refused by the `*` or `~` columns.
    #[must_use]
    pub fn refused_copper_writes(&self) -> u64 {
        self.refused.load(Ordering::Relaxed)
    }

    /// One lock acquisition for a whole access: the owners of `reg` that are
    /// actually attached, and the data lines, cloned out so the lock is
    /// released before any chip is called or any word driven.
    ///
    /// `reg` is `None` for an offset the appendix does not list, which drives
    /// no chip and still reaches the data lines.
    fn wiring_for(
        &self,
        reg: Option<&Reg>,
    ) -> (Vec<Arc<dyn CustomChip>>, Option<Arc<ChipDataBus>>) {
        let wiring = self.wiring.lock();
        let owners = match reg {
            Some(reg) => wiring
                .chips
                .iter()
                .filter(|c| reg.chip.contains(c.which()))
                .map(Arc::clone)
                .collect(),
            None => Vec::new(),
        };
        (owners, wiring.data.clone())
    }

    /// Read the word register at `offset`.
    ///
    /// `offset` is relative to the custom-chip base. A register a chip answers
    /// leaves that chip's word on the data lines. Anything the appendix does
    /// not declare readable — an unclaimed offset, a write-only register, a
    /// register whose chip this build does not have — is a cycle **nothing**
    /// drives: it answers with what was on the lines, leaves them floating
    /// ([`ChipDataBus::take`]), and moves [`unclaimed`](Self::unclaimed).
    pub fn read(&self, offset: u16, from: Origin) -> u16 {
        let reg = regs::lookup(offset).filter(|r| r.readable());
        let (owners, data) = self.wiring_for(reg);
        // A part that owns the address but has not got the register drives
        // nothing, and is not an owner for this purpose (`CustomChip::drives`).
        let driving = reg.is_some_and(|reg| owners.iter().any(|c| c.drives(reg)));
        let Some(reg) = reg.filter(|_| driving) else {
            if !from.debug {
                self.unclaimed.fetch_add(1, Ordering::Relaxed);
            }
            // A debugger looks at the lines; a guest's cycle takes them.
            return data.map_or(0, |d| if from.debug { d.word() } else { d.take() });
        };
        // Wired-or, which is what several chips driving one 16-bit bus is. Each
        // owner returns zeroes in the bits it does not drive (`CustomChip::read`).
        let value = owners
            .iter()
            .filter(|c| c.drives(reg))
            .fold(0u16, |acc, c| acc | c.read(reg, from));
        if !from.debug
            && let Some(data) = data
        {
            data.drive(value);
        }
        value
    }

    /// Write the word register at `offset`.
    ///
    /// The word goes onto the chip data lines whatever happens to it
    /// afterwards — the board's buffers put it there before any chip decides
    /// to latch it (`src/dev/amiga/dma.rs`, [`ChipDataBus`]).
    ///
    /// Returns whether anything took it: `false` for an unclaimed offset, a
    /// read-only register, a copper write the `*`/`~` columns refuse, a
    /// register whose chip this build does not have, or `NO-OP` — the one of
    /// those that is not counted as unclaimed.
    pub fn write(&self, offset: u16, value: u16, from: Origin) -> bool {
        let reg = regs::lookup(offset);
        let (owners, data) = self.wiring_for(reg);
        if !from.debug
            && let Some(data) = data
        {
            data.drive(value);
        }
        let Some(reg) = reg else {
            if !from.debug {
                self.unclaimed.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        };
        if reg.no_op() {
            // `NO-OP(NULL)` at `$1FE`: the appendix's own name for an address a
            // write reaches and nothing latches. Not a missing chip, so not
            // counted.
            return false;
        }
        if !reg.writable() {
            if !from.debug {
                self.unclaimed.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        }
        if let Driver::Copper { danger, ecs } = from.driver
            && !(if ecs {
                ecs_copper_may_write(reg.offset, danger)
            } else {
                copper_may_write(reg.access, danger)
            })
        {
            if !from.debug {
                self.refused.fetch_add(1, Ordering::Relaxed);
            }
            return false;
        }
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

    /// Back to power-on: nothing counted.
    ///
    /// The subscriber list and the chip data bus are **wiring, not guest
    /// state** ([`Device::export`]'s contract) and survive; the word on that
    /// bus is the driving chip's to clear, and Agnus's reset does.
    fn reset(&self) {
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
        if offset >= SPAN {
            return Err(BusError::BadAccess);
        }
        let from = Origin::cpu().for_debug(attrs.debug);
        // Big-endian on the wire; `AccessConstraints` says so too, so the
        // dispatcher will not have reordered anything for us.
        match dst.len() {
            2 if offset & 1 == 0 => {
                let value = self.bus.read(offset as u16, from);
                dst.copy_from_slice(&value.to_be_bytes());
            }
            1 => {
                // The chip drives the whole word — it has no strobe to tell it
                // otherwise — and the processor keeps one half of it.
                let word = self.bus.read(offset as u16 & !1, from).to_be_bytes();
                dst[0] = word[(offset & 1) as usize];
            }
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= SPAN {
            return Err(BusError::BadAccess);
        }
        let from = Origin::cpu().for_debug(attrs.debug);
        let value = match src.len() {
            2 if offset & 1 == 0 => u16::from_be_bytes([src[0], src[1]]),
            // MC68000UM Table 3-1: a byte write puts the byte on both halves of
            // the data bus, and the board passes all sixteen lines to a chip
            // that latches all sixteen. See the module docs.
            1 => u16::from_be_bytes([src[0], src[0]]),
            _ => return Err(BusError::BadAccess),
        };
        self.bus.write(offset as u16 & !1, value, from);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A byte or a word, big-endian, no bursts. Alignment is checked here
        // rather than by the dispatcher, because a byte may land on either
        // half of a register and a word may not.
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U16, Endian::Big)
        }
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

    /// The two counters and nothing else.
    ///
    /// The word on the chip data bus used to be written here. It is not this
    /// device's: `amiga.custom` is the decode, it drives nothing, and the
    /// chip that does — Agnus — carries the word in its own chunk. Writing it
    /// in both would be two authorities for one value.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.bus.unclaimed())?;
        w.write_u64(self.bus.refused_copper_writes())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let unclaimed = r.read_u64()?;
        let refused = r.read_u64()?;
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

    /// The same, with a chip data bus attached: what Agnus's bind does, minus
    /// Agnus. The handle is the caller's to drive a DMA cycle through.
    fn custom_with_dma() -> (Custom, Arc<ChipDataBus>) {
        let c = custom();
        let data = Arc::new(ChipDataBus::new());
        c.bus().attach_data_bus(Arc::clone(&data));
        (c, data)
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
    fn an_unreadable_address_answers_with_the_last_cycles_word_once() {
        let (c, data) = custom_with_dma();
        // A bitplane fetch, say: Agnus drove this word onto D15-D0.
        data.drive(0xa55a);
        // A write-only register answers with it...
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0xa55a);
        // ...and that read was itself a cycle nothing drove, so the lines are
        // floating now and the next one gets nothing. A guest polling a
        // write-only register therefore does *not* get a stable answer it
        // could mistake for a real register's, which is exactly the test
        // Appendix C documents.
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0x0000);
        assert_eq!(c.bus().unclaimed(), 2);

        // An offset the appendix leaves blank, and `$1FE`, which it lists as
        // `NO-OP` with no access letter, behave the same way.
        data.drive(0x0ff0);
        assert_eq!(c.bus().read(0x068, Origin::cpu()), 0x0ff0);
        data.drive(0x0ff0);
        assert_eq!(c.bus().read(0x1fe, Origin::cpu()), 0x0ff0);
        assert_eq!(c.bus().unclaimed(), 4);
    }

    /// A register a chip answers *does* drive the lines, so the word a guest
    /// finds at a write-only address afterwards is that chip's.
    #[test]
    fn a_read_and_a_write_both_drive_the_data_lines() {
        let (c, data) = custom_with_dma();
        c.bus().attach(Probe::new(ChipId::PAULA, 0x4321)).unwrap();
        assert_eq!(c.bus().read(DMACONR, Origin::cpu()), 0x4321);
        assert_eq!(data.word(), 0x4321, "the chip drove its answer");
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0x4321);

        // And a write puts its word there whether or not anything latches it:
        // the board's buffers do that before any chip decides.
        c.bus().write(COLOR00, 0x0765, Origin::cpu());
        assert_eq!(data.word(), 0x0765, "no Denise, and still on the lines");
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0x0765);
    }

    /// A board with no DMA engine has no chip-bus cycles at all. Nothing
    /// drives `D15`–`D0`, the fall-through answers zero, and the counter says
    /// how often that was relied on.
    #[test]
    fn a_board_with_no_dma_engine_reads_zero_and_counts_it() {
        let c = custom();
        assert!(!c.bus().write(COLOR00, 0x0abc, Origin::cpu()));
        assert_eq!(c.bus().read(COLOR00, Origin::cpu()), 0x0000);
        assert_eq!(c.bus().read(0x068, Origin::cpu()), 0x0000);
        assert_eq!(c.bus().floating(), 0x0000);
        // The write to Denise's register with no Denise is a fall-through too.
        assert_eq!(c.bus().unclaimed(), 3);
    }

    #[test]
    fn a_write_to_no_op_is_dropped_and_not_counted() {
        // The copper's padding address. A board with every chip present and a
        // copper list that pads with it must still read zero unclaimed.
        let (c, data) = custom_with_dma();
        c.bus().attach(Probe::new(ChipId::AGNUS, 0)).unwrap();
        data.drive(0x5555);
        assert!(!c.bus().write(0x1fe, 0x0000, Origin::copper(false)));
        assert!(!c.bus().write(0x1fe, 0x1234, Origin::cpu()));
        assert_eq!(c.bus().unclaimed(), 0);
        assert_eq!(c.bus().refused_copper_writes(), 0);
        assert_eq!(
            c.bus().floating(),
            0x1234,
            "nothing latched it, and the buffers put it on the lines anyway"
        );
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
    fn an_ecs_copper_is_held_to_appendix_c_instead() {
        let c = custom();
        let agnus = Probe::new(ChipId::AGNUS, 0);
        c.bus().attach(agnus.clone()).unwrap();

        // Danger clear: "$DFF03E through $DFF07E", the blitter block among
        // them, and everything above; nothing below.
        assert!(!c.bus().write(COPCON, 2, Origin::ecs_copper(false)));
        assert!(c.bus().write(BLTCON0, 1, Origin::ecs_copper(false)));
        assert!(c.bus().write(0x080, 0, Origin::ecs_copper(false)));
        // Danger set: "all of the Amiga chip registers".
        assert!(c.bus().write(COPCON, 2, Origin::ecs_copper(true)));

        assert_eq!(c.bus().refused_copper_writes(), 1);
        let offsets: Vec<u16> = agnus.seen().iter().map(|s| s.0).collect();
        assert_eq!(offsets, vec![BLTCON0, 0x080, COPCON]);
    }

    #[test]
    fn a_debugger_moves_no_counter_and_disturbs_no_bus() {
        let (c, data) = custom_with_dma();
        let paula = Probe::new(ChipId::PAULA, 0x0010);
        c.bus().attach(paula.clone()).unwrap();
        data.drive(0x0555);
        let unclaimed = c.bus().unclaimed();

        // A debugger sees exactly what the guest would — the undriven lines
        // are what the guest would latch, so hiding them would be the lie —
        // and is not a cycle: it drives nothing, takes nothing, counts
        // nothing. Reading the same write-only address three times over says
        // so, where a guest's second read would already have got zero.
        let debug = Origin::cpu().for_debug(true);
        for _ in 0..3 {
            assert_eq!(c.bus().read(COLOR00, debug), 0x0555);
        }
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
    fn the_window_takes_words_and_bytes_and_nothing_wider_or_odd() {
        let (c, data) = custom_with_dma();
        assert_eq!(c.region("").unwrap().len(), 0x200);
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };

        // `JOY0DAT` at $00A, Denise's and readable, so the byte order on the
        // wire is a register's answer rather than the undriven lines'.
        c.bus().attach(Probe::new(ChipId::DENISE, 0x0f00)).unwrap();
        let mut word = [0u8; 2];
        ops.read(0x00a, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(word, [0x0f, 0x00], "big-endian on the wire");
        // And a write-only register answers with the lines, big-endian too.
        data.drive(0x0f00);
        ops.read(0x180, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(word, [0x0f, 0x00]);

        let mut byte = [0u8; 1];
        assert!(ops.read(0x180, &mut byte, MemAttrs::DEFAULT).is_ok());
        assert!(ops.read(0x181, &mut word, MemAttrs::DEFAULT).is_err());
        assert!(ops.read(0x200, &mut word, MemAttrs::DEFAULT).is_err());
        assert!(ops.read(0x200, &mut byte, MemAttrs::DEFAULT).is_err());
        let mut long = [0u8; 4];
        assert!(ops.read(0x180, &mut long, MemAttrs::DEFAULT).is_err());
        assert!(ops.write(0x181, &[1, 2], MemAttrs::DEFAULT).is_err());
        let k = ops.constraints();
        assert_eq!(
            (k.min, k.max, k.endian, k.allow_bulk),
            (Width::U8, Width::U16, Endian::Big, false)
        );
    }

    #[test]
    fn a_byte_read_is_a_word_read_that_keeps_the_strobed_half() {
        let c = custom();
        let paula = Probe::new(ChipId::PAULA, 0x1234);
        c.bus().attach(paula.clone()).unwrap();
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };
        let mut byte = [0u8; 1];
        // `DMACONR` at $002: UDS alone (the even address) is the upper half...
        ops.read(0x002, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0x12]);
        // ...and LDS alone (the odd address) the lower.
        ops.read(0x003, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0x34]);
        // Each was a whole read of the register as far as the chip knows: it
        // has no strobe to say which half was wanted, so a register that
        // clears on read clears on either byte.
        let seen = paula.seen();
        assert_eq!(seen.len(), 2);
        assert!(seen.iter().all(|s| s.0 == 0x002 && s.1.is_none()));
    }

    #[test]
    fn a_byte_write_stores_the_byte_in_both_halves_at_either_address() {
        let c = custom();
        let denise = Probe::new(ChipId::DENISE, 0);
        c.bus().attach(denise.clone()).unwrap();
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };
        // `COLOR00` at $180. A byte at the even address is not "the high half,
        // low half kept", and a byte at the odd one is not "the low half, high
        // half kept": both drive one word with the byte twice.
        ops.write(0x180, &[0x0a], MemAttrs::DEFAULT).unwrap();
        ops.write(0x181, &[0x05], MemAttrs::DEFAULT).unwrap();
        let writes: Vec<(u16, Option<u16>)> = denise.seen().iter().map(|s| (s.0, s.1)).collect();
        assert_eq!(
            writes,
            vec![(0x180, Some(0x0a0a)), (0x180, Some(0x0505))],
            "MC68000UM Table 3-1: the byte on D15-D8 and D7-D0 alike"
        );
    }

    #[test]
    fn a_byte_read_of_an_undriven_register_is_half_of_the_floating_bus() {
        // The access Kickstart 2.04 makes: `$DFF07D`, the low half of
        // `DENISEID`, on a board where nothing answers it. Appendix C (p. 299):
        // "whatever value is left over on the bus from the last cycle" -- and
        // the last cycle is Agnus's, not the processor's own previous write.
        let (c, data) = custom_with_dma();
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };
        data.drive(0x0fc3);
        let mut byte = [0u8; 1];
        ops.read(0x07d, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0xc3]);
        // The whole word was taken, not half of it: the strobe selects which
        // half the processor keeps, and the lines are floating either way.
        data.drive(0x0fc3);
        ops.read(0x07c, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0x0f]);
        ops.read(0x07c, &mut byte, MemAttrs::DEFAULT).unwrap();
        assert_eq!(byte, [0x00], "and a second read gets nothing");
    }

    #[test]
    fn a_debugger_byte_access_disturbs_nothing() {
        let (c, data) = custom_with_dma();
        let paula = Probe::new(ChipId::PAULA, 0x00ff);
        c.bus().attach(paula.clone()).unwrap();
        data.drive(0x0555);
        let ops = Window {
            bus: Arc::clone(c.bus()),
        };
        let debug = MemAttrs {
            debug: true,
            ..MemAttrs::DEFAULT
        };
        let mut byte = [0u8; 1];
        ops.read(0x003, &mut byte, debug).unwrap();
        assert_eq!(byte, [0xff]);
        ops.write(0x181, &[0x99], debug).unwrap();
        assert_eq!(c.bus().floating(), 0x0555);
        assert!(paula.seen().iter().all(|s| s.2.debug));
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
    fn a_reset_clears_the_counters_and_keeps_the_wiring() {
        let (c, data) = custom_with_dma();
        c.bus().attach(Probe::new(ChipId::DENISE, 0)).unwrap();
        c.bus().write(0x068, 0x1234, Origin::cpu());
        assert_eq!(c.bus().unclaimed(), 1);
        data.drive(0x1234);
        Device::reset(&c, ResetKind::Cold);
        assert_eq!(c.bus().unclaimed(), 0);
        assert_eq!(c.bus().attached(), ChipId::DENISE, "wiring, not state");
        // The chip data bus survives too: it is the attachment that is wiring,
        // and the word on it belongs to the chip that drives it -- Agnus
        // clears it in its own reset, and this device drives nothing.
        assert_eq!(c.bus().floating(), 0x1234);
        data.clear();
        assert_eq!(c.bus().floating(), 0);
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
        // The word on the chip data bus is not in this chunk: it belongs to
        // the chip that drives it, and a board with no Agnus never had one.
        assert_eq!(restored.bus().floating(), 0x0000);
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
