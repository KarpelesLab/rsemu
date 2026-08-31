//! Sega's standard cartridge mapper — the one almost every Master System
//! cartridge carries.
//!
//! Four registers at the very top of memory, three 16 KiB windows, and a
//! 1 KiB hole that never moves:
//!
//! ```text
//!   $0000-$03FF   bank 0, always. The reset vector, the RSTs and $0038 live
//!                 here, so the mapper is wired never to move them.
//!   $0400-$3FFF   slot 0, the rest of the bank $FFFD selects
//!   $4000-$7FFF   slot 1, the bank $FFFE selects
//!   $8000-$BFFF   slot 2, the bank $FFFF selects — or cartridge RAM
//!   $C000-$DFFF   the console's 8 KiB of work RAM (not ours)
//!   $E000-$FFFF   the same RAM again; the board decodes thirteen bits
//!   $FFFC-$FFFF   these four registers
//! ```
//!
//! `$FFFC` is the odd one:
//!
//! ```text
//!   bit 7   ROM write enable
//!   bit 4   which 16 KiB half of the cartridge RAM slot 2 sees
//!   bit 3   map cartridge RAM into $8000-$BFFF instead of ROM
//!   bit 2   map cartridge RAM into $C000-$FFFF as well
//!   bits 1-0  bank shift
//! ```
//!
//! # Two slots are aliases, and one is not
//!
//! Slots 0 and 1 are published as **rebasable aliases** onto the cartridge's
//! ROM, so a bank switch is [`AddressSpace::rebase`]: one atomic store and a
//! refresh of the cached offsets, with no retopology, no generation bump and no
//! translation block invalidated. That is what that mechanism exists for
//! (`ROADMAP.md` §4.1), and it means a guest instruction fetch out of either
//! slot costs a table lookup and a memcpy.
//!
//! Slot 2 cannot be, and the reason is worth writing down because it is the
//! same gap the Game Boy's cartridge hit (see the head of `src/dev/gb/cart.rs`).
//! When `$FFFC` bit 3 is set the window stops being ROM and becomes writable
//! RAM. One region cannot answer reads from one store and writes to another,
//! and swapping the region is a *retopology* — which a device may not perform
//! from inside its own write handler (the lock ladder forbids it, and
//! deliberately). So slot 2 is an I/O handler that routes each access itself,
//! and it pays a virtual call per byte that slots 0 and 1 do not.
//!
//! **TODO(space)**: the generic fix is a region kind whose reads resolve to one
//! leaf and whose writes reach another, chosen by an atomic the device owns —
//! the same primitive `gb.cart` wants. Phase 4 is explicitly not the place to
//! change `core::space` (`ROADMAP.md` §13), so this is written down rather than
//! done, and the cost is stated rather than hidden.
//!
//! # Reading the registers
//!
//! On hardware `$FFFC`-`$FFFF` are write-only and the console's RAM answers a
//! read there, because the RAM chip is selected too. This device answers with
//! the values it was last written instead — identical for any program that
//! reaches those addresses only through `$FFFC`, and different only for one that
//! writes the RAM through its `$DFFC` mirror and expects to read it back at
//! `$FFFC`. Modelling the hardware exactly would need a wired-OR of two regions,
//! which is a `map` form the DSL does not have.
//!
//! # What is not modelled
//!
//! * **`$FFFC` bit 2**, cartridge RAM at `$C000`, which would fight the
//!   console's own work RAM. No commercial cartridge uses it.
//! * **`$FFFC` bit 7**, ROM write enable, which only matters to a development
//!   cartridge with static RAM in the ROM sockets.
//! * The **Codemasters**, **Korean** and **4 Pak** mappers. They are different
//!   boards and would be different device classes; the header heuristics that
//!   pick between them belong with them.
//!
//! # Sources
//!
//! [SMS Power!'s development documents](https://www.smspower.org/Development/Documents),
//! the memory-map and mapper pages. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, Mapping, MemAttrs, MemOps, MemResult, RamStore,
    Region as MmioRegion, RegionRef, RomStore, RomWrite,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;

/// How big one bank is.
pub const BANK_SIZE: u64 = 0x4000;

/// How much of bank 0 the mapper is wired never to move.
///
/// The Z80 resets to `$0000`, the eight `RST` targets are in the first 64 bytes
/// and the mode-1 interrupt vector is `$0038`. A cartridge that could bank those
/// away would be able to bank away its own interrupt handler.
pub const FIXED_LEN: u64 = 0x0400;

/// How much address space the three slots cover: `$0000`-`$BFFF`.
pub const WINDOW_LEN: u64 = 0xc000;

/// Where the four registers sit.
pub const REGISTER_BASE: u64 = 0xfffc;

/// The default cartridge RAM: two banks of 16 KiB, the most the mapper can see.
pub const DEFAULT_RAM: u64 = 0x8000;

/// The name a `map` statement reaches the three slots by.
pub const ROM_REGION: &str = "rom";

/// The name a `map` statement reaches the four registers by.
pub const REGISTER_REGION: &str = "regs";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// The four mapper registers, as the guest last wrote them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Registers {
    /// `$FFFC`: RAM enable, RAM bank, ROM write enable, bank shift.
    control: u8,
    /// `$FFFD`, `$FFFE`, `$FFFF`: the three slot banks.
    bank: [u8; 3],
}

impl Default for Registers {
    fn default() -> Registers {
        // Power-on leaves the three slots showing the first three banks, so a
        // cartridge is a flat 48 KiB until it says otherwise — which is exactly
        // what a 32 KiB game that never touches the mapper relies on.
        Registers {
            control: 0,
            bank: [0, 1, 2],
        }
    }
}

impl Registers {
    /// Whether slot 2 shows cartridge RAM rather than ROM.
    fn ram_enabled(&self) -> bool {
        self.control & 0x08 != 0
    }

    /// Which 16 KiB half of the cartridge RAM slot 2 shows.
    fn ram_bank(&self) -> u64 {
        u64::from(self.control & 0x10 != 0)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

struct Shared {
    regs: Mutex<Registers>,
    space: Mutex<Option<Arc<AddressSpace>>>,
    rom: Arc<RomStore>,
    ram: Arc<RamStore>,
    /// How many 16 KiB banks the padded image has. A bank register is taken
    /// modulo this, which is what an incompletely decoded ROM address bus does.
    banks: u64,
    /// The two rebasable windows, kept so a bank write can slide them.
    slot0: RegionRef,
    slot1: RegionRef,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("regs", &self.regs)
            .field("banks", &self.banks)
            .finish_non_exhaustive()
    }
}

impl Shared {
    /// Slide slots 0 and 1 to the banks the registers name.
    ///
    /// Called with **no** lock of this device held: `rebase` reaches into the
    /// address space, and the re-entrancy contract says an outward call happens
    /// after the critical section, not inside it (`ROADMAP.md` §4.4).
    fn apply(&self, regs: Registers) {
        let space = self.space.lock().clone();
        let Some(space) = space else {
            return;
        };
        let bank0 = u64::from(regs.bank[0]) % self.banks;
        let bank1 = u64::from(regs.bank[1]) % self.banks;
        // Slot 0's window starts FIXED_LEN into its bank, because the first
        // kilobyte of the address space is a separate, unmoving alias.
        let _ = space.rebase(&self.slot0, bank0 * BANK_SIZE + FIXED_LEN);
        let _ = space.rebase(&self.slot1, bank1 * BANK_SIZE);
    }
}

/// A cartridge on Sega's standard mapper board.
pub struct SegaMapper {
    shared: Arc<Shared>,
    rom_region: RegionRef,
    regs_region: RegionRef,
}

impl fmt::Debug for SegaMapper {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SegaMapper")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl SegaMapper {
    /// Build a board around `image`, with `ram_len` bytes of cartridge RAM.
    ///
    /// The image is padded up to a whole number of 16 KiB banks, because the
    /// windows are aliases and an alias that ran off the end of its target is a
    /// configuration error rather than a wrap.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if `image` is empty, or if the regions cannot be built
    /// — which would mean the padding arithmetic above is wrong.
    pub fn new(image: &[u8], ram_len: u64) -> Result<SegaMapper> {
        if image.is_empty() {
            return Err(Error::Config {
                at: String::from("rom"),
                message: String::from("a Master System cartridge image cannot be empty"),
            });
        }
        let padded = (image.len() as u64).next_multiple_of(BANK_SIZE);
        let mut bytes = Vec::with_capacity(padded as usize);
        bytes.extend_from_slice(image);
        // $FF, not zero: an unpopulated ROM socket floats high, and a program
        // that runs off the end of its image meets `RST 38h` either way.
        bytes.resize(padded as usize, 0xff);
        let banks = padded / BANK_SIZE;

        let rom = Arc::new(RomStore::new(bytes));
        let ram = Arc::new(RamStore::new(ram_len.max(BANK_SIZE)));
        let leaf: RegionRef = Arc::new(MmioRegion::rom(
            "sms.cart.image",
            Arc::clone(&rom),
            RomWrite::Ignore,
        ));

        let fixed: RegionRef = Arc::new(MmioRegion::alias(
            "sms.cart.fixed",
            Arc::clone(&leaf),
            0,
            FIXED_LEN,
        )?);
        let slot0: RegionRef = Arc::new(MmioRegion::alias(
            "sms.cart.slot0",
            Arc::clone(&leaf),
            FIXED_LEN,
            BANK_SIZE - FIXED_LEN,
        )?);
        let slot1: RegionRef = Arc::new(MmioRegion::alias(
            "sms.cart.slot1",
            Arc::clone(&leaf),
            // Bank 1, or bank 0 again on a one-bank image.
            (1 % banks) * BANK_SIZE,
            BANK_SIZE,
        )?);

        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(LockRank::DEVICE, Registers::default()),
            space: Mutex::new(None),
            rom,
            ram,
            banks,
            slot0: Arc::clone(&slot0),
            slot1: Arc::clone(&slot1),
        });

        let slot2: RegionRef = Arc::new(MmioRegion::io(
            "sms.cart.slot2",
            BANK_SIZE,
            Arc::new(Slot2 {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));

        let rom_region: RegionRef = Arc::new(MmioRegion::container(
            "sms.cart.rom",
            WINDOW_LEN,
            alloc::vec![
                Mapping::new(fixed, 0),
                Mapping::new(slot0, FIXED_LEN),
                Mapping::new(slot1, BANK_SIZE),
                Mapping::new(slot2, BANK_SIZE * 2),
            ],
        ));
        let regs_region: RegionRef = Arc::new(MmioRegion::io(
            "sms.cart.regs",
            4,
            Arc::new(MapperRegs {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));

        Ok(SegaMapper {
            shared,
            rom_region,
            regs_region,
        })
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If the `rom` media slot is missing, or the image is empty.
    pub fn from_props(props: &Props) -> Result<SegaMapper> {
        let mut r = props.reader();
        let media = r.require_media("rom")?;
        let ram = r.or_size("ram", DEFAULT_RAM)?;
        r.finish()?;
        SegaMapper::new(media.bytes(), ram)
    }

    /// How many 16 KiB banks the padded image has.
    #[must_use]
    pub fn banks(&self) -> u64 {
        self.shared.banks
    }

    /// One of the three slot bank registers, 0-2.
    #[must_use]
    pub fn bank(&self, slot: usize) -> u8 {
        self.shared.regs.lock().bank[slot % 3]
    }

    /// `$FFFC` as it was last written.
    #[must_use]
    pub fn control(&self) -> u8 {
        self.shared.regs.lock().control
    }

    /// Whether slot 2 currently shows cartridge RAM.
    #[must_use]
    pub fn ram_enabled(&self) -> bool {
        self.shared.regs.lock().ram_enabled()
    }

    /// The cartridge's battery-backed RAM, for a host that saves it.
    #[must_use]
    pub fn ram(&self) -> &Arc<RamStore> {
        &self.shared.ram
    }

    /// Connect the address space the windows live in.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        *self.shared.space.lock() = Some(space);
        let regs = *self.shared.regs.lock();
        self.shared.apply(regs);
    }

    /// Write one register by index, 0-3, as the guest would.
    pub fn write_register(&self, index: usize, value: u8) {
        let regs = {
            let mut regs = self.shared.regs.lock();
            match index % 4 {
                0 => regs.control = value,
                n => regs.bank[n - 1] = value,
            }
            *regs
        };
        self.shared.apply(regs);
    }
}

// ---------------------------------------------------------------------------
// The apertures
// ---------------------------------------------------------------------------

/// `$8000`-`$BFFF`: ROM, or cartridge RAM when `$FFFC` bit 3 says so.
struct Slot2 {
    shared: Arc<Shared>,
}

impl fmt::Debug for Slot2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Slot2").finish_non_exhaustive()
    }
}

impl MemOps for Slot2 {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // Neither store has a side effect on read, so a debug read needs no
        // special case here.
        let regs = *self.shared.regs.lock();
        if regs.ram_enabled() {
            let base = regs.ram_bank() * BANK_SIZE + offset;
            let len = self.shared.ram.len();
            if len == 0 {
                return Err(BusError::BadAccess);
            }
            for (i, byte) in dst.iter_mut().enumerate() {
                *byte = self.shared.ram.read_u8((base + i as u64) % len)?;
            }
            return Ok(());
        }
        let bytes = self.shared.rom.as_bytes();
        let base = (u64::from(regs.bank[2]) % self.shared.banks) * BANK_SIZE + offset;
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = bytes[((base + i as u64) as usize) % bytes.len()];
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let regs = *self.shared.regs.lock();
        let bank = regs.ram_bank();
        if !regs.ram_enabled() {
            // A mask ROM swallows a write. It is not an error, and a great deal
            // of software writes here by accident.
            return Ok(());
        }
        let len = self.shared.ram.len();
        if len == 0 {
            return Err(BusError::BadAccess);
        }
        let base = bank * BANK_SIZE + offset;
        for (i, byte) in src.iter().enumerate() {
            self.shared.ram.write_u8((base + i as u64) % len, *byte)?;
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// `$FFFC`-`$FFFF`.
struct MapperRegs {
    shared: Arc<Shared>,
}

impl fmt::Debug for MapperRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MapperRegs").finish_non_exhaustive()
    }
}

impl MemOps for MapperRegs {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // No side effect, so no debug special case. See the module docs for why
        // this answers with the register rather than with the RAM underneath.
        let regs = self.shared.regs.lock();
        *byte = match offset & 3 {
            0 => regs.control,
            n => regs.bank[(n - 1) as usize],
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write here would rebank the guest's own code out from
            // under it. A monitor that wants that can call `write_register`.
            return Ok(());
        }
        let regs = {
            let mut regs = self.shared.regs.lock();
            match offset & 3 {
                0 => regs.control = *value,
                n => regs.bank[(n - 1) as usize] = *value,
            }
            *regs
        };
        // Outside the critical section: `apply` reaches into the address space.
        self.shared.apply(regs);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `sms.mapper` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "sms.mapper",
    version: 1,
    summary: "Sega's standard SMS mapper: three 16 KiB slots, a fixed first KiB, cartridge RAM",
    properties: &[
        PropertySpec {
            name: "rom",
            kind: ValueKind::Media,
            required: true,
            summary: "the cartridge image, as the name of a media slot (`rom = \"cart\"`)",
        },
        PropertySpec {
            name: "ram",
            kind: ValueKind::Size,
            required: false,
            summary: "cartridge RAM, up to two 16 KiB banks (default 32K)",
        },
    ],
    construct: |props| Ok(Box::new(SegaMapper::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SegaMapper {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The windows are slid in `Instance::bind`, which is where the address
        // space arrives. Realize runs before it and has nothing to reach.
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            ROM_REGION => Some(Arc::clone(&self.rom_region)),
            REGISTER_REGION => Some(Arc::clone(&self.regs_region)),
            _ => None,
        }
    }

    fn reset(&self, _kind: ResetKind) {
        let regs = {
            let mut regs = self.shared.regs.lock();
            *regs = Registers::default();
            *regs
        };
        self.shared.apply(regs);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        w.write_u32(STATE_VERSION)?;
        w.write_u8(regs.control)?;
        for bank in regs.bank {
            w.write_u8(bank)?;
        }
        // The cartridge's RAM is the save file. The ROM is not state.
        let len = self.shared.ram.len();
        let mut bytes = alloc::vec![0u8; len as usize];
        self.shared.ram.read_at(0, &mut bytes).map_err(Error::Bus)?;
        w.write_bytes(&bytes)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the mapper's snapshot is version {version}, this build writes {STATE_VERSION}"
            )));
        }
        let control = r.read_u8()?;
        let bank = [r.read_u8()?, r.read_u8()?, r.read_u8()?];
        let bytes = r.read_bytes()?;
        if bytes.len() as u64 != self.shared.ram.len() {
            return Err(Error::State(String::from(
                "the mapper's snapshot has a different amount of cartridge RAM",
            )));
        }
        self.shared.ram.write_at(0, bytes).map_err(Error::Bus)?;
        let regs = {
            let mut regs = self.shared.regs.lock();
            *regs = Registers { control, bank };
            *regs
        };
        // The window offsets are derived from the registers and must follow
        // them, or the guest resumes reading the bank it was in before.
        self.shared.apply(regs);
        Ok(())
    }
}

/// The machine layer's half: the board needs the address space its own windows
/// live in, because a bank switch is a rebase performed on that space.
impl crate::machine::Instance for SegaMapper {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the mapper needs the address space its windows are mapped into, because a \
                 bank switch slides them: add `space = mem` to the object",
            ),
        })?;
        self.attach_space(Arc::clone(space));
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(SegaMapper::from_props(props)?))
    })
}

/// What the validator should know about `sms.mapper`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .prop(PropSchema::new("ram", ValueKind::Size))
        .region(ROM_REGION)
        .region(REGISTER_REGION)
}
