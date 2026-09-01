//! Base Address Registers: the mappings a PCI function moves from inside its
//! own configuration write.
//!
//! # What a BAR is
//!
//! *PCI Local Bus Specification, Revision 2.1* **§6.2.5.1, "Base Addresses"**.
//! A function that decodes anything beyond configuration space says so with one
//! or more of the six 32-bit registers at offsets `10h`-`27h`. Bit 0 says which
//! space:
//!
//! ```text
//!   memory   31                             4   3   2  1   0
//!           +-------------------------------+---+-----+---+
//!           |          base address         |pre| type| 0 |
//!           +-------------------------------+---+-----+---+
//!
//!   I/O      31                                 2   1   0
//!           +-----------------------------------+---+---+
//!           |            base address           | r | 1 |
//!           +-----------------------------------+---+---+
//! ```
//!
//! * **type** (bits 2:1) is `00` for a register that may be placed anywhere in
//!   32-bit memory space, `01` for one that must go below 1 MiB, and `10` for a
//!   **64-bit** register — which is two consecutive BARs, the second holding
//!   the upper 32 bits of the address. `11` is reserved.
//! * **pre** (bit 3) marks the range **prefetchable**: reading it has no side
//!   effects and a bridge may merge writes and read more than was asked for.
//! * The address bits below the window's size are **hardwired to zero**, and
//!   that is how software learns the size:
//!
//!   > In order to determine the amount of address space required, software
//!   > writes a value of all 1's to the register and then reads the value back.
//!   > The device will return 0's in all don't-care address bits, effectively
//!   > specifying the address space required.
//!
//!   So the size is `!(value & mask) + 1` with the low format bits cleared. A
//!   BAR that is not implemented reads back as all zeroes, which is how software
//!   knows to stop looking.
//!
//! Memory windows are a minimum of 16 bytes and I/O windows a minimum of 4
//! (§6.2.5.1), and every window is a power of two aligned to its own size.
//!
//! # The expansion ROM
//!
//! §6.2.5.2, "Expansion ROM Base Address Register", offset `30h`:
//!
//! ```text
//!   31                                 11   10          1   0
//!  +-----------------------------------+----------------+---+
//!  |      expansion ROM base address   |    reserved    |ena|
//!  +-----------------------------------+----------------+---+
//! ```
//!
//! Bit 0 is the **enable**; the address bits start at 11, so a ROM window is a
//! multiple of 2 KiB. The spec is explicit about the precedence, and it matters
//! because firmware sets one bit and not the other:
//!
//! > the Memory Space bit in the Command register has precedence over the
//! > Expansion ROM Enable bit. A device must respond to accesses to its
//! > expansion ROM only if both the Memory Space bit and the Expansion ROM
//! > Enable bit are set to 1.
//!
//! Which is also true of an ordinary memory BAR against `COMMAND[1]`, and of an
//! I/O BAR against `COMMAND[0]` (§6.2.2). A function comes out of reset with
//! both clear, so nothing decodes until firmware has finished sizing.
//!
//! # Moving a mapping from inside a configuration write
//!
//! This is the hard part, and it is the case `core::space`'s module docs open
//! with. A configuration write arrives as an `OUT` to `0xcfc`, so the **I/O**
//! space's topology lock is already held for reading and the CPU's `BUS`-ranked
//! lock above it. [`AddressSpace::topology`] on the memory space would take a
//! second `TOPOLOGY`-ranked lock and `core::sync`'s ladder panics on that in
//! debug builds.
//!
//! [`super::super::super::dev::pc::pmc`] hit exactly this for the PAM registers
//! and answered it with [`AddressSpace::try_topology`], which is *order-exempt
//! by construction* — a try-lock cannot wait, so it cannot be half of a deadlock
//! cycle — plus a `stale` flag re-applied at the next configuration access. A
//! BAR is the same shape and takes the same answer, with one difference worth
//! naming: PAM only ever calls [`reprotect`], while a BAR also calls
//! [`remap`]. Both are methods on the same guard, so the guard is what the
//! argument is about and the argument does not change.
//!
//! What does change is the *consequence of failing*. A stale PAM window decodes
//! the ROM for one instant longer than it should; a stale BAR decodes at its old
//! address, which is where nothing is looking. Neither is silently swallowed:
//! [`Bars::is_stale`] is set, and the function re-applies at its next
//! configuration access — of which there is always at least one more, because
//! firmware writes `COMMAND` *after* it writes the BARs.
//!
//! # I/O BARs decode nothing here, and cannot yet
//!
//! [`Bars`] models an I/O BAR's register completely — the indicator bit, the
//! sizing read-back, the base — so firmware can size and place one. It refuses
//! to *map* one, at [`Bars::install`], with an error saying why:
//!
//! A configuration cycle **travels through the I/O space**. Retopologising the
//! space an access is currently travelling through is the one case the
//! try-lock cannot serve: the read guard is held by this very access, so the
//! try always fails, and the retry at the next configuration access fails for
//! the same reason, for ever. The escape is a
//! [`Deferred`](crate::core::device::Deferred) action, which lands a scheduler
//! quantum later; nothing in this tree has an I/O BAR yet, so that trade is not
//! made on a guess. When something does, this is the paragraph to argue with.
//!
//! [`AddressSpace::topology`]: crate::core::space::AddressSpace::topology
//! [`AddressSpace::try_topology`]: crate::core::space::AddressSpace::try_topology
//! [`remap`]: crate::core::space::TopologyGuard::remap
//! [`reprotect`]: crate::core::space::TopologyGuard::reprotect

use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::config::{COMMAND_IO, COMMAND_MEMORY};
use crate::core::error::{Error, Result};
use crate::core::space::{AddressSpace, Mapping, MappingId, Perms, RegionRef};
use crate::core::sync::{LockRank, Mutex};

/// Which space a base address register places its window in, and how.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarKind {
    /// Memory space (Rev 2.1 §6.2.5.1). Gated by `COMMAND[1]`.
    Memory,
    /// I/O space (§6.2.5.1). Gated by `COMMAND[0]`.
    Io,
    /// The expansion ROM at offset `30h` (§6.2.5.2). Gated by `COMMAND[1]`
    /// *and* by its own enable bit.
    ExpansionRom,
}

/// One base address register, as the function that owns it declares it.
///
/// Built with [`Bar::memory`], [`Bar::io`] or [`Bar::rom`] and handed to
/// [`Bars::with`].
#[derive(Clone)]
pub struct Bar {
    kind: BarKind,
    len: u64,
    /// 64-bit decode: this register and the next one are one address
    /// (§6.2.5.1, type `10`). Memory only.
    wide: bool,
    /// Bit 3: reading the window has no side effects. Memory only.
    prefetchable: bool,
    /// What answers inside the window, if this crate maps it.
    region: Option<RegionRef>,
    /// The terms it answers on while it is decoding.
    perms: Perms,
}

impl fmt::Debug for Bar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bar")
            .field("kind", &self.kind)
            .field("len", &self.len)
            .field("wide", &self.wide)
            .field("prefetchable", &self.prefetchable)
            .field("region", &self.region.as_ref().map(|r| r.name()))
            .field("perms", &self.perms)
            .finish()
    }
}

impl Bar {
    /// A memory window of `len` bytes, 32-bit, not prefetchable.
    #[must_use]
    pub fn memory(len: u64) -> Bar {
        Bar {
            kind: BarKind::Memory,
            len,
            wide: false,
            prefetchable: false,
            region: None,
            perms: Perms::RW,
        }
    }

    /// An I/O window of `len` bytes.
    #[must_use]
    pub fn io(len: u64) -> Bar {
        Bar {
            kind: BarKind::Io,
            len,
            wide: false,
            prefetchable: false,
            region: None,
            perms: Perms::RW,
        }
    }

    /// An expansion ROM window of `len` bytes (§6.2.5.2).
    ///
    /// Read-only and executable by default, because that is what the window is
    /// for: firmware copies the image out of it and jumps into the copy.
    #[must_use]
    pub fn rom(len: u64) -> Bar {
        Bar {
            kind: BarKind::ExpansionRom,
            len,
            wide: false,
            prefetchable: false,
            region: None,
            perms: Perms::RX,
        }
    }

    /// Make it a 64-bit register: this BAR and the next one are one address.
    ///
    /// Ignored for an I/O or expansion ROM register, neither of which has a
    /// 64-bit form.
    #[must_use]
    pub fn wide(mut self) -> Bar {
        self.wide = self.kind == BarKind::Memory;
        self
    }

    /// Set bit 3, prefetchable. Memory only, and ignored elsewhere.
    #[must_use]
    pub fn prefetchable(mut self) -> Bar {
        self.prefetchable = self.kind == BarKind::Memory;
        self
    }

    /// What answers inside the window, and on what terms.
    ///
    /// A BAR with no region is a register and nothing else: the function
    /// decodes the window itself, or nothing does.
    #[must_use]
    pub fn decoding(mut self, region: RegionRef, perms: Perms) -> Bar {
        self.region = Some(region);
        self.perms = perms;
        self
    }

    /// How many bytes the window covers.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the window covers no bytes — it never does; [`Bars::with`]
    /// refuses a zero length.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Which space it places its window in.
    #[must_use]
    pub fn kind(&self) -> BarKind {
        self.kind
    }

    /// The low bits the guest cannot move: the format field (§6.2.5.1).
    fn fixed_bits(&self) -> u32 {
        match self.kind {
            // Bit 0 clear (memory), bits 2:1 the type, bit 3 prefetchable.
            BarKind::Memory => {
                let ty = if self.wide { 0b100 } else { 0b000 };
                ty | if self.prefetchable { 0x8 } else { 0x0 }
            }
            // Bit 0 set marks an I/O register; bit 1 is reserved and reads zero.
            BarKind::Io => 0x1,
            // Bit 0 is the enable, which the guest *may* write, so it is not
            // fixed. Bits 10:1 are reserved and read as zero.
            BarKind::ExpansionRom => 0x0,
        }
    }

    /// Which bits of the low dword a guest write keeps.
    fn write_mask(&self) -> u32 {
        // The address bits below the window size are hardwired to zero, which
        // is what turns an all-ones write into a size report (§6.2.5.1).
        let size = !(self.len.wrapping_sub(1)) as u32;
        match self.kind {
            BarKind::Memory => size & 0xffff_fff0,
            BarKind::Io => size & 0xffff_fffc,
            // Plus bit 0, the enable, which is writable (§6.2.5.2).
            BarKind::ExpansionRom => (size & 0xffff_f800) | 0x1,
        }
    }

    /// Which bits of the *upper* dword a guest write keeps, for a 64-bit BAR.
    fn high_write_mask(&self) -> u32 {
        (!(self.len.wrapping_sub(1)) >> 32) as u32
    }
}

/// The smallest memory window §6.2.5.1 allows.
const MIN_MEMORY_LEN: u64 = 16;
/// The smallest I/O window §6.2.5.1 allows.
const MIN_IO_LEN: u64 = 4;
/// The smallest expansion ROM window §6.2.5.2's address field can express.
const MIN_ROM_LEN: u64 = 2048;

/// Configuration offset of the first base address register.
const BAR0: u16 = 0x10;
/// Configuration offset of the expansion ROM base address register.
const ROM_OFFSET: u16 = 0x30;

/// The priority a BAR's window is mapped at.
///
/// Above the board's own decode *and* above the chipset shadow the host bridge
/// maps at 1: a card that has been given an address decodes it, and whatever
/// the motherboard put there is what the address was taken from. Nothing on a
/// PC ever programs a BAR over the shadow window — firmware allocates from a
/// hole well above RAM — so this is a tie-break rule rather than a behaviour.
const BAR_PRIORITY: i32 = 2;

/// Where a window went.
#[derive(Debug, Clone)]
struct Placed {
    space: Arc<AddressSpace>,
    ids: BTreeMap<u8, MappingId>,
}

/// The six base address registers, the expansion ROM register, and the windows
/// they move.
///
/// Held by a [`PciFunction`](super::PciFunction) beside its
/// [`ConfigSpace`](super::ConfigSpace): [`config_read`](Bars::config_read) and
/// [`config_write`](Bars::config_write) claim the offsets that belong here, and
/// everything else is the function's.
///
/// # Locks
///
/// Three, all at [`LockRank::LEAF`] and none ever held across another: the
/// latches, where the windows went, and the stale flag. A `LEAF` lock may not
/// be taken while another is held (`core::sync`), so every method below reads
/// what it needs, drops the guard, and only then does the outward thing.
pub struct Bars {
    /// What the function declared, keyed by register index. Index 6 is the
    /// expansion ROM; a 64-bit BAR's upper half occupies the following index
    /// and has no entry of its own.
    specs: BTreeMap<u8, Bar>,
    /// The raw latch for every one of the seven registers, including the upper
    /// half of a 64-bit pair.
    values: Mutex<[u32; Bars::COUNT as usize]>,
    /// Where the windows went. `None` until [`Bars::install`].
    placed: Mutex<Option<Placed>>,
    /// Set when a retopology could not happen at the instant it was asked for.
    /// Derived state: never serialized, and a load re-applies unconditionally.
    stale: Mutex<bool>,
}

impl fmt::Debug for Bars {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Bars");
        s.field("specs", &self.specs);
        match self.values.try_lock() {
            Some(v) => s.field("values", &*v),
            None => s.field("values", &"<in use>"),
        };
        s.field("placed", &self.placed.try_lock().map(|p| p.is_some()))
            .finish()
    }
}

impl Default for Bars {
    fn default() -> Bars {
        Bars::new()
    }
}

impl Bars {
    /// The register index of the expansion ROM base address register.
    ///
    /// Not a seventh BAR on the wire — it is at `30h`, not `28h` — but it
    /// behaves like one in every way this module cares about, so it is indexed
    /// like one.
    pub const ROM: u8 = 6;

    /// How many registers there are, counting the expansion ROM.
    pub const COUNT: u8 = 7;

    /// A function with no base address registers at all.
    ///
    /// Every register reads back as zero, which is Rev 2.1 §6.2.5.1's own way
    /// of saying "not implemented".
    #[must_use]
    pub fn new() -> Bars {
        Bars {
            specs: BTreeMap::new(),
            values: Mutex::with_rank(LockRank::LEAF, [0; Bars::COUNT as usize]),
            placed: Mutex::with_rank(LockRank::LEAF, None),
            stale: Mutex::with_rank(LockRank::LEAF, false),
        }
    }

    /// Declare `bar` at register `index`, or [`Bars::ROM`] for the expansion
    /// ROM.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] for an index a register does not exist at, a kind that
    /// does not belong at that index, a length that is not a power of two or is
    /// below the minimum §6.2.5.1 states for its kind, a 64-bit register in the
    /// last slot (there would be no upper half), or an index already claimed —
    /// including by the upper half of a 64-bit register below it.
    pub fn with(mut self, index: u8, bar: Bar) -> Result<Bars> {
        let at = |message: &str| Error::Config {
            at: alloc::format!("BAR{index}"),
            message: String::from(message),
        };
        match (index, bar.kind) {
            (Bars::ROM, BarKind::ExpansionRom) => {}
            (0..=5, BarKind::Memory | BarKind::Io) => {}
            (Bars::ROM, _) => {
                return Err(at("register 6 is the expansion ROM and holds nothing else"));
            }
            (_, BarKind::ExpansionRom) => {
                return Err(at("the expansion ROM register is index 6, not this one"));
            }
            _ => return Err(at("a Type 00h header has six base address registers, 0-5")),
        }
        let min = match bar.kind {
            BarKind::Memory => MIN_MEMORY_LEN,
            BarKind::Io => MIN_IO_LEN,
            BarKind::ExpansionRom => MIN_ROM_LEN,
        };
        if !bar.len.is_power_of_two() || bar.len < min {
            return Err(at(
                "a window is a power of two, and no smaller than its kind's minimum: 16 bytes \
                 of memory, 4 of I/O, 2048 of expansion ROM (Rev 2.1 §6.2.5.1, §6.2.5.2)",
            ));
        }
        if bar.wide && index == 5 {
            return Err(at(
                "a 64-bit register is two consecutive BARs, so it cannot be the last one",
            ));
        }
        if self.claimed(index) || (bar.wide && self.claimed(index + 1)) {
            return Err(at("something already answers at this register"));
        }
        self.specs.insert(index, bar);
        Ok(self)
    }

    /// Whether `index` is spoken for, by a declaration or by the upper half of
    /// a 64-bit register below it.
    fn claimed(&self, index: u8) -> bool {
        if self.specs.contains_key(&index) {
            return true;
        }
        index > 0
            && self
                .specs
                .get(&(index - 1))
                .is_some_and(|b| b.wide && b.kind == BarKind::Memory)
    }

    /// What the function declared at `index`, if anything.
    #[must_use]
    pub fn spec(&self, index: u8) -> Option<&Bar> {
        self.specs.get(&index)
    }

    /// Which register a configuration offset falls in, if any.
    fn register_of(offset: u16) -> Option<u8> {
        match offset {
            0x10..=0x27 => Some(((offset - BAR0) / 4) as u8),
            _ if (ROM_OFFSET..ROM_OFFSET + 4).contains(&offset) => Some(Bars::ROM),
            _ => None,
        }
    }

    /// The 32 bits a read of register `index` sees.
    fn value(&self, index: u8) -> u32 {
        let raw = self.values.lock()[index as usize];
        match self.specs.get(&index) {
            Some(bar) => raw | bar.fixed_bits(),
            // Either the upper half of a 64-bit register — which has no format
            // bits of its own — or a register that does not exist, which reads
            // as zero and whose latch is therefore always zero.
            None => raw,
        }
    }

    /// Answer a configuration read of the bytes these registers own.
    ///
    /// Bytes outside them are **left as the caller had them**, which is how a
    /// function splits its 256 bytes without a second table: it fills `dst`
    /// from its own [`ConfigSpace`](super::ConfigSpace) — where `0x10`-`0x27`
    /// and `0x30`-`0x33` are read-only zeroes, because it never made them
    /// writable — and lets this overwrite the bytes that are really here.
    pub fn config_read(&self, offset: u16, dst: &mut [u8]) {
        for (i, slot) in dst.iter_mut().enumerate() {
            let at = offset.saturating_add(i as u16);
            let Some(index) = Bars::register_of(at) else {
                continue;
            };
            let dword = self.value(index);
            let byte = (at & 0x3) as u32;
            *slot = (dword >> (byte * 8)) as u8;
        }
    }

    /// Take a configuration write, honouring each register's hardwired bits.
    ///
    /// Reports whether any latch moved, so a function can skip a retopology for
    /// the writes that changed nothing — firmware rewrites the same base
    /// constantly while it sizes.
    pub fn config_write(&self, offset: u16, src: &[u8]) -> bool {
        let mut changed = false;
        let mut values = self.values.lock();
        for (i, byte) in src.iter().enumerate() {
            let at = offset.saturating_add(i as u16);
            let Some(index) = Bars::register_of(at) else {
                continue;
            };
            let mask = self.mask_of(index);
            let shift = (at & 0x3) * 8;
            let byte_mask = (mask >> shift) as u8;
            if byte_mask == 0 {
                continue;
            }
            let slot = &mut values[index as usize];
            let keep = u32::from(byte_mask) << shift;
            let updated = (*slot & !keep) | ((u32::from(*byte) << shift) & keep);
            if updated != *slot {
                *slot = updated;
                changed = true;
            }
        }
        changed
    }

    /// Which bits of register `index` a guest write keeps.
    fn mask_of(&self, index: u8) -> u32 {
        if let Some(bar) = self.specs.get(&index) {
            return bar.write_mask();
        }
        // The upper half of a 64-bit register below this one.
        if index > 0
            && let Some(bar) = self.specs.get(&(index - 1))
            && bar.wide
        {
            return bar.high_write_mask();
        }
        0
    }

    /// Where register `index`'s window currently sits, and whether it decodes.
    ///
    /// `command` is the function's Command register: §6.2.2's space-enable bits
    /// gate every window, and §6.2.5.2's ROM enable gates the ROM on top of
    /// them.
    #[must_use]
    pub fn window(&self, index: u8, command: u16) -> Option<(u64, bool)> {
        let bar = self.specs.get(&index)?;
        let values = self.values.lock();
        let low = values[index as usize];
        let base = match bar.kind {
            BarKind::Memory if bar.wide => {
                let high = values[index as usize + 1];
                u64::from(low & bar.write_mask()) | (u64::from(high & bar.high_write_mask()) << 32)
            }
            _ => u64::from(low & bar.write_mask() & !0x1),
        };
        let decoding = match bar.kind {
            BarKind::Memory => command & COMMAND_MEMORY != 0,
            BarKind::Io => command & COMMAND_IO != 0,
            // Both bits, in that order of precedence (§6.2.5.2).
            BarKind::ExpansionRom => command & COMMAND_MEMORY != 0 && low & 0x1 != 0,
        };
        Some((base, decoding))
    }

    /// Adopt `space` as where this function's memory windows go, and place
    /// whatever currently decodes. **Retopology**, and legal only where nothing
    /// is in flight: [`Instance::bind`](crate::machine::realize::Instance::bind),
    /// which is what calls it.
    ///
    /// Out of reset that is nothing at all — `COMMAND` is zero, so no window
    /// decodes and none is mapped. **A window that does not decode is not in
    /// the map**, rather than being mapped with [`Perms::NONE`]: a function
    /// that does not respond is *absent*, so the address falls through to
    /// whatever else the board decodes there and, failing that, to the space's
    /// unassigned policy — which on a PC is "reads as ones". A no-permission
    /// mapping would instead raise
    /// [`BusError::Protected`](crate::core::error::BusError::Protected) at an
    /// address the machine is supposed to read `0xff` from, and firmware probes
    /// exactly such addresses.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a declared I/O BAR carries a region — see the
    /// module docs, which say why that cannot work yet.
    pub fn install(&self, space: &Arc<AddressSpace>, command: u16) -> Result<()> {
        for (index, bar) in &self.specs {
            if bar.region.is_some() && bar.kind == BarKind::Io {
                return Err(Error::Config {
                    at: alloc::format!("BAR{index}"),
                    message: String::from(
                        "an I/O BAR cannot carry a region yet: a configuration cycle travels \
                         through the I/O space, so retopologising it from inside one is the \
                         case the order-exempt try-lock cannot serve",
                    ),
                });
            }
        }
        *self.placed.lock() = Some(Placed {
            space: Arc::clone(space),
            ids: BTreeMap::new(),
        });
        self.sync(command, true);
        Ok(())
    }

    /// Bring the windows into line with the registers. Reports whether it could
    /// be done.
    ///
    /// `blocking` picks the guard: the blocking one where nothing is in flight
    /// — bind, reset, a snapshot load — and the order-exempt try-lock from
    /// inside a configuration write, where the blocking one would invert
    /// `core::sync`'s ladder. A failed try sets [`is_stale`](Bars::is_stale)
    /// rather than being swallowed.
    pub fn sync(&self, command: u16, blocking: bool) -> bool {
        // Cloned out and the lock released: nothing of this device's is held
        // while the space is retopologised, which is the re-entrancy contract.
        let Some(placed) = self.placed.lock().clone() else {
            // Not installed yet. Not stale either: `install` places every
            // window with the right terms in the first place.
            return true;
        };
        // Everything the registers ask for, computed before any guard is held.
        let wanted: Vec<(u8, RegionRef, u64, Perms)> = self
            .specs
            .iter()
            .filter_map(|(index, bar)| {
                let region = bar.region.clone()?;
                let (base, decoding) = self.window(*index, command)?;
                decoding.then_some((*index, region, base, bar.perms))
            })
            .collect();
        let guard = if blocking {
            Some(placed.space.topology())
        } else {
            placed.space.try_topology()
        };
        let Some(mut topo) = guard else {
            *self.stale.lock() = true;
            return false;
        };
        let mut ids = placed.ids.clone();
        // Whatever no longer decodes leaves the map entirely, before anything
        // that does is placed: a window that moved out of the way has to be
        // gone before the one moving in can claim its address.
        let gone: Vec<u8> = ids
            .keys()
            .copied()
            .filter(|index| !wanted.iter().any(|(i, ..)| i == index))
            .collect();
        for index in gone {
            if let Some(id) = ids.remove(&index) {
                // The only error is "not a mapping of this space", which cannot
                // happen: every id here came from this space.
                let _ = topo.unmap(id);
            }
        }
        for (index, region, base, perms) in wanted {
            match ids.get(&index) {
                // Already in the map: move it, and take it back out if the base
                // firmware wrote does not fit the space. That is a card
                // decoding an address the machine cannot drive, which decodes
                // nothing.
                Some(id) => {
                    if topo.remap(*id, base).is_err() {
                        let _ = topo.unmap(*id);
                        ids.remove(&index);
                    }
                }
                None => {
                    if let Ok(id) = topo.map_with(
                        Mapping::new(region, base)
                            .with_priority(BAR_PRIORITY)
                            .with_perms(perms),
                    ) {
                        ids.insert(index, id);
                    }
                }
            }
        }
        drop(topo);
        *self.placed.lock() = Some(Placed {
            space: Arc::clone(&placed.space),
            ids,
        });
        *self.stale.lock() = false;
        true
    }

    /// Whether a retopology is owed because one could not be done when it was
    /// asked for.
    #[must_use]
    pub fn is_stale(&self) -> bool {
        *self.stale.lock()
    }

    /// Every latch, for a snapshot.
    #[must_use]
    pub fn latches(&self) -> [u32; Bars::COUNT as usize] {
        *self.values.lock()
    }

    /// Restore the latches from a snapshot, masked exactly as a guest write is.
    ///
    /// A snapshot cannot install bits the hardware could never hold, for the
    /// same reason [`ConfigSpace::restore`](super::ConfigSpace::restore) will
    /// not let one change a vendor ID.
    pub fn set_latches(&self, values: &[u32]) {
        let mut slots = self.values.lock();
        for (index, slot) in slots.iter_mut().enumerate() {
            let mask = self.mask_of(index as u8);
            *slot = values.get(index).copied().unwrap_or(0) & mask;
        }
    }

    /// Clear every latch, as `PCIRST#` does.
    pub fn reset(&self) {
        *self.values.lock() = [0; Bars::COUNT as usize];
    }
}
