//! NROM — iNES mapper 0.
//!
//! Source: the NESdev wiki pages [NROM](https://www.nesdev.org/wiki/NROM),
//! [Mapper](https://www.nesdev.org/wiki/Mapper) and
//! [Cartridge connector](https://www.nesdev.org/wiki/Cartridge_connector).
//!
//! NROM is not a mapper at all — it is the absence of one. The board wires the
//! cartridge connector straight to two mask ROMs and ties CIRAM A10 to one PPU
//! address line, and that is the whole of it. There is no bank register, so
//! there is nothing to write and nothing to snapshot but the RAM.
//!
//! ```text
//!   CPU space                             PPU space
//!   ─────────                             ─────────
//!   $6000-$7FFF  work RAM (Family Basic)  $0000-$1FFF  CHR ROM or CHR RAM
//!   $8000-$BFFF  PRG ROM                  $2000-$2FFF  nametables (CIRAM)
//!   $C000-$FFFF  PRG ROM, or $8000 again  $3000-$3EFF  the same, mirrored
//! ```
//!
//! # Everything here is a region, not a handler
//!
//! The 16 KiB variant (NROM-128) presents its ROM twice because address line
//! A14 is simply not connected. That is a **repeating window**
//! ([`Region::mirror`]), so it flattens to one dispatch entry with a modulus
//! rather than to two copies of the data or to an I/O handler that masks an
//! address. Nametable mirroring is four 1 KiB aliases onto the console's CIRAM,
//! which is likewise what the hardware is: one address line, wired one way or
//! the other.
//!
//! The result is that a guest read of `$FFFC` costs a table lookup and a memcpy,
//! with no virtual call anywhere on the path.
//!
//! # CIRAM: the console's RAM, the cartridge's wiring
//!
//! The 2 KiB of nametable RAM is on the console's board, not on the cartridge;
//! the cartridge only drives one of its address lines (and, for a four-screen
//! board, supplies 2 KiB of its own alongside it). The *arrangement*, though,
//! is entirely the cartridge's — it is a solder pad on the board — and it is
//! recorded in byte 6 of the iNES header.
//!
//! Which is why the board holds the store. A machine description reaches an
//! aperture through [`Device::region`], which is answered while the memory map
//! is being built — before `Instance::bind`, and long before any device has
//! been told about any other. There is no moment at which the cartridge could
//! be handed the console object's RAM and still publish a region named by a
//! `map` statement. So the choice is between the board owning 2 KiB that
//! belongs to the console, and every `.machine` file hard-coding a mirroring
//! the cartridge already knows — and the second is a bug in every horizontally
//! mirrored game.
//!
//! [`Nrom::ciram`] hands the store back out, [`Nrom::save`](Device::save)
//! carries it, and the four-screen VRAM — which *is* cartridge state — sits
//! beside it.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, Mapping, MappingId, RamStore, Region, RegionRef, RomWrite};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::machine::realize::Instance;

use super::ines::{Cartridge, Chr};

/// The CPU window a cartridge's work RAM is decoded into: `$6000-$7FFF`.
const WORK_RAM_BASE: u64 = 0x6000;
/// Size of that window.
const WORK_RAM_WINDOW: u64 = 0x2000;
/// The CPU window PRG ROM is decoded into: `$8000-$FFFF`.
const PRG_BASE: u64 = 0x8000;
/// Size of that window — and the most PRG ROM NROM can address.
const PRG_WINDOW: u64 = 0x8000;
/// The PPU window the pattern tables live in: `$0000-$1FFF`.
const CHR_BASE: u64 = 0x0000;
/// Size of that window.
const CHR_WINDOW: u64 = 0x2000;
/// Where the nametables are decoded: `$2000-$2FFF`.
const NAMETABLE_BASE: u64 = 0x2000;
/// Four 1 KiB nametables.
const NAMETABLE_WINDOW: u64 = 0x1000;
/// One nametable.
const NAMETABLE_SIZE: u64 = 0x0400;
/// Where the nametables appear again: `$3000-$3EFF`. The last 256 bytes of the
/// PPU's 14-bit space are palette RAM, which is inside the PPU and not the
/// cartridge's to map.
const NAMETABLE_MIRROR_BASE: u64 = 0x3000;
/// Size of that second window.
const NAMETABLE_MIRROR_WINDOW: u64 = 0x0f00;
/// The console's nametable RAM.
const CIRAM_LEN: u64 = 0x0800;
/// How much address space a 6502 has.
const CPU_SPACE_LEN: u64 = 0x1_0000;
/// How much the PPU decodes.
const PPU_SPACE_LEN: u64 = 0x4000;

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// Version 2 added CIRAM to the chunk: the board now holds the console's
/// nametable RAM so that it can publish a wired-up `nametables` region, and a
/// store the device owns is a store the device snapshots.
const STATE_VERSION: u32 = 2;

/// The class name a machine description would use.
const CLASS_NAME: &str = "nes.nrom";

/// What [`Nrom::install`] added, so it can be undone.
///
/// Returned rather than remembered inside the device: a mapping id belongs to
/// the space that issued it, and a device that cached one would be asserting it
/// knows which space that was. Hand this back to [`Nrom::uninstall`].
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CartMappings {
    /// Mappings added to the CPU's address space.
    pub cpu: Vec<MappingId>,
    /// Mappings added to the PPU's address space.
    pub ppu: Vec<MappingId>,
}

/// An NROM cartridge, ready to be mapped into a machine.
///
/// Two-phase like every device (`ROADMAP.md` §4.4): [`Nrom::new`] validates the
/// cartridge against what the board can actually decode and builds the region
/// tree, and nothing is observable until [`Nrom::install`] places it in an
/// address space.
#[derive(Debug)]
pub struct Nrom {
    cart: Cartridge,
    /// `$8000-$FFFF`, repeating if the ROM is smaller than the window.
    prg_window: RegionRef,
    /// `$0000-$1FFF` in the PPU's space.
    chr_window: RegionRef,
    /// `$6000-$7FFF`, if the board has any work RAM.
    work_ram_window: Option<RegionRef>,
    /// The extra 2 KiB a four-screen board carries. Genuinely cartridge state.
    vram: Option<Arc<RamStore>>,
    /// That same VRAM as a mappable region, so a machine file can place it.
    vram_region: Option<RegionRef>,
    /// The console's 2 KiB of nametable RAM. See the [module docs](self) for
    /// why the board holds it.
    ciram: Arc<RamStore>,
    /// `$2000-$2FFF`: four 1 KiB windows onto [`Nrom::ciram`] (and, on a
    /// four-screen board, onto [`Nrom::vram`]), wired the way this cartridge
    /// wires CIRAM A10.
    nametables: RegionRef,
}

impl Nrom {
    /// Build the board for `cart`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] when the cartridge is not something an NROM board
    /// could be: a different mapper number, more PRG or CHR ROM than the fixed
    /// windows can address, or a size that is not a power of two and so cannot
    /// be produced by leaving address lines unconnected.
    pub fn new(cart: Cartridge) -> Result<Nrom> {
        if cart.mapper() != 0 {
            return Err(config(format!(
                "cartridge names mapper {}, which is not NROM (0)",
                cart.mapper()
            )));
        }

        let prg = cart.prg_rom();
        check_window("PRG ROM", prg.len(), PRG_WINDOW)?;
        let prg_rom = Arc::new(Region::rom(
            "nes.nrom.prg-rom",
            prg.clone(),
            RomWrite::Ignore,
        ));
        // A repeating window rather than two mappings: NROM-128 leaves A14
        // disconnected, so the same 16 KiB answers at $8000 and $C000. One flat
        // entry with a modulus is that, exactly, and it is also what makes the
        // 32 KiB case the identical code path with a period of one.
        let prg_window = Arc::new(Region::mirror("nes.nrom.prg", prg_rom, PRG_WINDOW)?);

        let chr_window = match cart.chr() {
            Chr::Rom(rom) => {
                check_window("CHR ROM", rom.len(), CHR_WINDOW)?;
                let region = Arc::new(Region::rom(
                    "nes.nrom.chr-rom",
                    rom.clone(),
                    RomWrite::Ignore,
                ));
                Arc::new(Region::mirror("nes.nrom.chr", region, CHR_WINDOW)?)
            }
            Chr::Ram(ram) => {
                check_window("CHR RAM", ram.len(), CHR_WINDOW)?;
                let region = Arc::new(Region::ram("nes.nrom.chr-ram", ram.clone()));
                Arc::new(Region::mirror("nes.nrom.chr", region, CHR_WINDOW)?)
            }
        };

        let work_ram_window = match cart.work_ram() {
            None => None,
            Some(ram) => {
                check_window("work RAM", ram.len(), WORK_RAM_WINDOW)?;
                let region = Arc::new(Region::ram("nes.nrom.work-ram", ram.clone()));
                Some(Arc::new(Region::mirror(
                    "nes.nrom.work",
                    region,
                    WORK_RAM_WINDOW,
                )?))
            }
        };

        let vram = if cart.mirroring().needs_cartridge_vram() {
            Some(Arc::new(RamStore::new(CIRAM_LEN)))
        } else {
            None
        };

        let vram_region = vram
            .as_ref()
            .map(|v| Arc::new(Region::ram("nes.nrom.vram", Arc::clone(v))) as RegionRef);

        // Built here rather than in `Device::region`, for the reason every
        // other window is: two `map` statements naming one aperture must get
        // one region, or the machine has two identities for one piece of RAM.
        let ciram = Arc::new(RamStore::new(CIRAM_LEN));
        let nametables = Arc::new(nametables(cart.mirroring(), &ciram, vram.as_ref())?);

        Ok(Nrom {
            cart,
            prg_window,
            chr_window,
            work_ram_window,
            vram,
            vram_region,
            ciram,
            nametables,
        })
    }

    /// Build the board from an iNES or NES 2.0 image.
    ///
    /// # Errors
    ///
    /// Everything [`Cartridge::from_ines`] rejects, plus everything
    /// [`Nrom::new`] does.
    pub fn from_image(bytes: &[u8]) -> Result<Nrom> {
        Nrom::new(Cartridge::from_ines(bytes)?)
    }

    /// Build the board from machine-description properties.
    ///
    /// The image arrives as the `rom` property, which a machine file writes as
    /// the *name* of a media slot (`rom = "cart"`) and the realizer replaces
    /// with the bytes bound to that slot — see
    /// [`MediaTable`](crate::machine::MediaTable). A caller assembling `Props`
    /// itself puts a [`Value::Media`](crate::core::props::Value::Media) there
    /// directly.
    ///
    /// # Errors
    ///
    /// If `rom` is missing or unbound, if the image does not parse, or if the
    /// cartridge is not something an NROM board could be.
    pub fn from_props(props: &Props) -> Result<Nrom> {
        let mut r = props.reader();
        let image = r.require_media("rom")?.to_bytes();
        r.finish()?;
        Nrom::from_image(&image)
    }

    /// The cartridge this board holds.
    #[must_use]
    pub const fn cartridge(&self) -> &Cartridge {
        &self.cart
    }

    /// The extra nametable RAM a four-screen board carries.
    #[must_use]
    pub const fn cartridge_vram(&self) -> Option<&Arc<RamStore>> {
        self.vram.as_ref()
    }

    /// The console's 2 KiB of nametable RAM, which this board wires and holds.
    ///
    /// See the [module docs](self): the RAM is the console's, but the *wiring*
    /// is the cartridge's, and a region cannot be published without both.
    #[must_use]
    pub const fn ciram(&self) -> &Arc<RamStore> {
        &self.ciram
    }

    /// Map the board into a CPU space and a PPU space. **Retopology.**
    ///
    /// Everything the board decodes, including the four nametable windows onto
    /// [`Nrom::ciram`] — see the [module docs](self).
    ///
    /// This is what [`Device::realize`] will call once `RealizeCtx` can hand a
    /// device its address spaces; until then it is the seam a machine builder
    /// uses directly. A machine described in the DSL does not need it: it names
    /// `cart.prg`, `cart.chr` and `cart.nametables` in `map` statements.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if either space is too small to be a NES bus, or if a
    /// mapping does not fit.
    pub fn install(&self, cpu: &AddressSpace, ppu: &AddressSpace) -> Result<CartMappings> {
        if cpu.size() < CPU_SPACE_LEN {
            return Err(config(format!(
                "CPU space `{}` is {:#x} bytes; a 6502 bus is {CPU_SPACE_LEN:#x}",
                cpu.name(),
                cpu.size()
            )));
        }
        if ppu.size() < PPU_SPACE_LEN {
            return Err(config(format!(
                "PPU space `{}` is {:#x} bytes; the PPU decodes {PPU_SPACE_LEN:#x}",
                ppu.name(),
                ppu.size()
            )));
        }

        // Build every region before mapping any of them, so a failure leaves
        // the spaces untouched rather than half-populated.
        let nametables = Arc::clone(&self.nametables);
        let nametable_mirror = Arc::new(Region::alias(
            "nes.nrom.nametables-mirror",
            nametables.clone(),
            0,
            NAMETABLE_MIRROR_WINDOW,
        )?);

        // One topology guard per space, and never both at once: two locks at
        // the same rank is a lock-order violation (`core::sync`), so a
        // cross-space install is two sequential batches rather than one atomic
        // step. Regions were all built above, so the first batch cannot fail
        // for a reason the second would not have caught.
        let mut mappings = CartMappings::default();
        {
            let mut topo = cpu.topology();
            if let Some(work) = &self.work_ram_window {
                mappings.cpu.push(topo.map(work.clone(), WORK_RAM_BASE)?);
            }
            mappings
                .cpu
                .push(topo.map(self.prg_window.clone(), PRG_BASE)?);
        }
        {
            let mut topo = ppu.topology();
            mappings
                .ppu
                .push(topo.map(self.chr_window.clone(), CHR_BASE)?);
            mappings.ppu.push(topo.map(nametables, NAMETABLE_BASE)?);
            mappings
                .ppu
                .push(topo.map(nametable_mirror, NAMETABLE_MIRROR_BASE)?);
        }
        Ok(mappings)
    }

    /// Undo an [`install`](Nrom::install). **Retopology.**
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a mapping is not one of that space's.
    pub fn uninstall(
        &self,
        cpu: &AddressSpace,
        ppu: &AddressSpace,
        mappings: &CartMappings,
    ) -> Result<()> {
        // Sequential guards, for the same reason `install` uses them.
        {
            let mut topo = cpu.topology();
            for id in &mappings.cpu {
                topo.unmap(*id)?;
            }
        }
        {
            let mut topo = ppu.topology();
            for id in &mappings.ppu {
                topo.unmap(*id)?;
            }
        }
        Ok(())
    }

    /// Every store this device owns that a snapshot has to carry, in a fixed
    /// order.
    ///
    /// Fixed order because it is the wire format: work RAM, then CHR RAM, then
    /// four-screen VRAM, then CIRAM, each length-prefixed and each possibly
    /// absent. CIRAM is never absent, but it is encoded the same way as the
    /// rest so that the reader stays one loop.
    fn mutable_stores(&self) -> [Option<&Arc<RamStore>>; 4] {
        [
            self.cart.work_ram(),
            self.cart.chr().as_ram(),
            self.vram.as_ref(),
            Some(&self.ciram),
        ]
    }
}

/// The `$2000-$2FFF` container: four 1 KiB windows onto CIRAM, wired the way
/// the board wires CIRAM A10.
///
/// A free function rather than a method because it runs inside [`Nrom::new`],
/// before there is an `Nrom` to call it on — the region has to exist by the
/// time a `map` statement asks for it (see the [module docs](self)).
fn nametables(
    mirroring: super::ines::Mirroring,
    ciram: &Arc<RamStore>,
    vram: Option<&Arc<RamStore>>,
) -> Result<Region> {
    let console = Arc::new(Region::ram("nes.ciram", ciram.clone()));
    let cart_vram = vram.map(|v| Arc::new(Region::ram("nes.nrom.vram", v.clone())));

    let mut children = Vec::with_capacity(4);
    for (slot, bank) in mirroring.banks().into_iter().enumerate() {
        let (target, index) = if bank < 2 {
            (&console, u64::from(bank))
        } else {
            let vram = cart_vram.as_ref().ok_or_else(|| {
                config(String::from(
                    "four-screen mirroring needs cartridge VRAM, which this board has none of",
                ))
            })?;
            (vram, u64::from(bank) - 2)
        };
        let name = match slot {
            0 => "nes.nrom.nt0",
            1 => "nes.nrom.nt1",
            2 => "nes.nrom.nt2",
            _ => "nes.nrom.nt3",
        };
        let window = Region::alias(name, target.clone(), index * NAMETABLE_SIZE, NAMETABLE_SIZE)?;
        children.push(Mapping::new(window, slot as u64 * NAMETABLE_SIZE));
    }
    Ok(Region::container(
        "nes.nrom.nametables",
        NAMETABLE_WINDOW,
        children,
    ))
}

/// Reject a ROM or RAM size an NROM board could not present in `window`.
///
/// Power of two because the only way a board makes a small ROM fill a large
/// window is by leaving high address lines unconnected, which halves the period
/// each time.
fn check_window(what: &str, len: u64, window: u64) -> Result<()> {
    if len == 0 {
        return Err(config(format!(
            "NROM needs some {what}; the cartridge has none"
        )));
    }
    if len > window {
        return Err(config(format!(
            "{what} is {len:#x} bytes, more than NROM's {window:#x} window can address"
        )));
    }
    if !len.is_power_of_two() {
        return Err(config(format!(
            "{what} is {len:#x} bytes, which no arrangement of address lines produces"
        )));
    }
    Ok(())
}

fn config(message: String) -> Error {
    Error::Config {
        at: String::from(CLASS_NAME),
        message,
    }
}

/// Copy a RAM store out into a `Vec` for snapshotting.
fn read_store(store: &RamStore) -> Result<Vec<u8>> {
    let len = usize::try_from(store.len())
        .map_err(|_| Error::State(String::from("RAM larger than the host address space")))?;
    let mut buf = alloc::vec![0u8; len];
    store
        .read_at(0, &mut buf)
        .map_err(|e| Error::State(format!("cannot read cartridge RAM: {e}")))?;
    Ok(buf)
}

/// The device class, for the registry and for `rsemu describe`.
pub static NROM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "NES NROM cartridge (iNES mapper 0): fixed PRG and CHR windows, no banking",
    properties: &[PropertySpec {
        name: "rom",
        kind: ValueKind::Media,
        required: true,
        summary: "the iNES image, as the name of a media slot (`rom = \"cart\"`)",
    }],
    construct: |props| Ok(Box::new(Nrom::from_props(props)?)),
};

/// Add [`NROM_CLASS`] to a registry.
///
/// Registration is explicit per feature (`ROADMAP.md` §4.4) — there is no
/// link-time magic here and there will not be.
///
/// # Errors
///
/// [`Error::Config`] if the class name is already taken.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&NROM_CLASS)
}

impl Device for Nrom {
    fn class(&self) -> &'static DeviceClass {
        &NROM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Deliberately empty. The board's windows are placed by `map`
        // statements, which the realizer runs after every device has realized
        // — the memory map is a statement in the machine file rather than a
        // decision inside the cartridge (`ROADMAP.md` §5). `Nrom::install` is
        // still there for a caller assembling a NES without the DSL.
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        // Named windows only: a cartridge has no single aperture — it decodes
        // in two different address spaces — so `map … = cart` with no region
        // would have to guess which one was meant.
        match name {
            "prg" => Some(Arc::clone(&self.prg_window)),
            "chr" => Some(Arc::clone(&self.chr_window)),
            "work" => self.work_ram_window.clone(),
            "vram" => self.vram_region.clone(),
            // The whole point of the exercise: `$2000-$2FFF` already wired for
            // *this* cartridge's mirroring, so a machine file asks the board
            // which way the pads are cut instead of guessing. Map it twice —
            // at `$2000` and again, clipped to `$0F00`, at `$3000`.
            "nametables" => Some(Arc::clone(&self.nametables)),
            _ => None,
        }
    }

    fn reset(&self, kind: ResetKind) {
        // A reset line does not clear RAM — only power does. Cold reset zeroes
        // the volatile stores rather than leaving them at whatever the last run
        // left, because "undefined at power-on" and "deterministic" cannot both
        // be true and determinism is the non-negotiable one (`ROADMAP.md` §0).
        if kind != ResetKind::Cold {
            return;
        }
        if let Some(chr) = self.cart.chr().as_ram() {
            let _ = chr.fill(0, chr.len(), 0);
        }
        if let Some(vram) = &self.vram {
            let _ = vram.fill(0, vram.len(), 0);
        }
        let _ = self.ciram.fill(0, self.ciram.len(), 0);
        if let Some(work) = self.cart.work_ram() {
            // Battery-backed RAM survives a power cycle. That is the entire
            // point of the battery.
            if !self.cart.battery() {
                let _ = work.fill(0, work.len(), 0);
            }
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // ROM contents are not architectural state: they come from the image
        // and cannot change, so serializing them would put megabytes of
        // constant into every snapshot (`ROADMAP.md` §4.5).
        for store in self.mutable_stores() {
            match store {
                Some(s) => {
                    w.write_bool(true)?;
                    w.write_bytes(&read_store(s)?)?;
                }
                None => w.write_bool(false)?,
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        for (i, store) in self.mutable_stores().into_iter().enumerate() {
            let name = ["work RAM", "CHR RAM", "four-screen VRAM", "CIRAM"][i];
            let present = r.read_bool()?;
            match (present, store) {
                (false, None) => {}
                (true, Some(s)) => {
                    let bytes = r.read_bytes()?;
                    if bytes.len() as u64 != s.len() {
                        return Err(Error::State(format!(
                            "snapshot has {} byte(s) of {name}, but this cartridge has {}",
                            bytes.len(),
                            s.len()
                        )));
                    }
                    s.write_at(0, bytes)
                        .map_err(|e| Error::State(format!("cannot restore {name}: {e}")))?;
                }
                (true, None) => {
                    return Err(Error::State(format!(
                        "snapshot has {name}, but this cartridge has none"
                    )));
                }
                (false, Some(_)) => {
                    return Err(Error::State(format!(
                        "snapshot has no {name}, but this cartridge has some"
                    )));
                }
            }
        }
        Ok(())
    }
}

/// The machine layer's half: NROM has no clock, no pins and no space of its
/// own, so binding it is nothing at all.
///
/// The `impl` still has to exist — a class with no [`Instance`] publishes no
/// regions to the machine graph, and `map cpubus 0x8000 = cart.prg` would be
/// told the class publishes none.
impl Instance for Nrom {}

/// Bind [`NROM_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Nrom::from_props(props)?)))
}

/// What the validator should know about `nes.nrom`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .region("prg")
        .region("chr")
        .region("work")
        .region("vram")
        .region("nametables")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Media;
    use crate::core::space::MemAttrs;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::value::Width;
    use crate::dev::cart::ines::Mirroring;
    use alloc::vec;

    /// An iNES 1.0 image with `prg_units` × 16 KiB of PRG and `chr_units` × 8
    /// KiB of CHR, each byte set to its own low address so a mirror is visible.
    fn image(prg_units: u8, chr_units: u8, flags6: u8) -> Vec<u8> {
        let mut v = vec![0u8; 16];
        v[..4].copy_from_slice(b"NES\x1a");
        v[4] = prg_units;
        v[5] = chr_units;
        v[6] = flags6;
        let prg_len = usize::from(prg_units) * 16384;
        for i in 0..prg_len {
            // A byte that identifies its own offset, wrapping every 256.
            v.push((i >> 8) as u8 ^ (i as u8));
        }
        let chr_len = usize::from(chr_units) * 8192;
        for i in 0..chr_len {
            v.push(!((i >> 8) as u8 ^ (i as u8)));
        }
        v
    }

    fn board(prg_units: u8, chr_units: u8, flags6: u8) -> Nrom {
        let cart = Cartridge::from_ines(&image(prg_units, chr_units, flags6)).expect("valid image");
        Nrom::new(cart).expect("an NROM board")
    }

    struct Bus {
        cpu: AddressSpace,
        ppu: AddressSpace,
    }

    fn bus() -> Bus {
        Bus {
            cpu: AddressSpace::new("cpu", 16),
            ppu: AddressSpace::new("ppu", 14),
        }
    }

    fn rd(space: &AddressSpace, addr: u64) -> u8 {
        space
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("read {addr:#06x}: {e}")) as u8
    }

    fn wr(space: &AddressSpace, addr: u64, value: u8) {
        space
            .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("write {addr:#06x}: {e}"));
    }

    #[test]
    fn sixteen_kib_of_prg_answers_in_both_banks() {
        let nrom = board(1, 1, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");

        for offset in [0u64, 1, 0x1234, 0x3ffc, 0x3fff] {
            let low = rd(&b.cpu, PRG_BASE + offset);
            let high = rd(&b.cpu, PRG_BASE + 0x4000 + offset);
            assert_eq!(low, high, "offset {offset:#06x} must mirror");
            let want = ((offset >> 8) as u8) ^ (offset as u8);
            assert_eq!(low, want, "offset {offset:#06x}");
        }

        // The reset vector is the one that actually matters: a 6502 fetches it
        // from $FFFC, which on NROM-128 is the last two bytes of a 16 KiB ROM.
        assert_eq!(rd(&b.cpu, 0xfffc), rd(&b.cpu, 0xbffc));

        // One flat entry, not two copies of the data.
        let view = b.cpu.view();
        let idx = view.locate(PRG_BASE).expect("mapped");
        let entry = view.flat_view().entry(idx).expect("entry");
        assert_eq!(entry.start(), PRG_BASE);
        assert_eq!(entry.len(), PRG_WINDOW);
    }

    #[test]
    fn thirty_two_kib_of_prg_is_contiguous() {
        let nrom = board(2, 1, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");

        for offset in [0u64, 0x3fff, 0x4000, 0x7fff] {
            let want = ((offset >> 8) as u8) ^ (offset as u8);
            assert_eq!(rd(&b.cpu, PRG_BASE + offset), want, "offset {offset:#06x}");
        }
        // $8000 and $C000 are now different bytes, unlike the 16 KiB case.
        assert_ne!(rd(&b.cpu, 0x8001), rd(&b.cpu, 0xc001));
    }

    #[test]
    fn prg_rom_ignores_writes() {
        let nrom = board(2, 1, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        let before = rd(&b.cpu, 0x8000);
        wr(&b.cpu, 0x8000, before.wrapping_add(1));
        assert_eq!(rd(&b.cpu, 0x8000), before, "a mask ROM swallows writes");
    }

    #[test]
    fn chr_rom_is_readable_and_chr_ram_is_writable() {
        let nrom = board(1, 1, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        for offset in [0u64, 0x1000, 0x1fff] {
            let want = !(((offset >> 8) as u8) ^ (offset as u8));
            assert_eq!(rd(&b.ppu, CHR_BASE + offset), want, "chr {offset:#06x}");
        }
        wr(&b.ppu, 0x0100, 0x99);
        assert_ne!(rd(&b.ppu, 0x0100), 0x99, "CHR ROM is not writable");

        // A CHR-RAM cartridge (CHR size 0) is.
        let nrom = board(1, 0, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        assert!(nrom.cartridge().chr().is_ram());
        wr(&b.ppu, 0x0100, 0x99);
        assert_eq!(rd(&b.ppu, 0x0100), 0x99);
    }

    #[test]
    fn work_ram_is_mapped_and_mirrored() {
        let nrom = board(1, 1, 0);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        wr(&b.cpu, 0x6000, 0x42);
        assert_eq!(rd(&b.cpu, 0x6000), 0x42);
        // 8 KiB of RAM in an 8 KiB window: no mirroring, but the far end works.
        wr(&b.cpu, 0x7fff, 0x24);
        assert_eq!(rd(&b.cpu, 0x7fff), 0x24);
        assert_eq!(rd(&b.cpu, 0x6000), 0x42);
    }

    #[test]
    fn a_board_with_no_work_ram_maps_none() {
        // NES 2.0 can say "no PRG RAM", which iNES 1.0 cannot.
        let mut h = [0u8; 16];
        h[..4].copy_from_slice(b"NES\x1a");
        h[4] = 1;
        h[7] = 0x08;
        h[11] = 0x07; // 8 KiB CHR RAM, no PRG RAM
        let mut img = h.to_vec();
        img.extend(core::iter::repeat_n(0u8, 16384));
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert!(cart.work_ram().is_none());
        let nrom = Nrom::new(cart).expect("board");
        let b = bus();
        let m = nrom.install(&b.cpu, &b.ppu).expect("installs");
        assert_eq!(m.cpu.len(), 1, "only the PRG window");
        assert!(b.cpu.locate(0x6000).is_none(), "$6000 is open bus");
    }

    // -- nametable mirroring ----------------------------------------------

    /// Write a marker into each of the four nametable slots in turn and record
    /// what all four slots read back as, which is the mirroring made visible.
    fn nametable_pattern(nrom: &Nrom) -> [[u8; 4]; 4] {
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        let mut out = [[0u8; 4]; 4];
        for (written, row) in out.iter_mut().enumerate() {
            // Clear, then mark one slot, so each row is independent.
            for slot in 0..4u64 {
                wr(&b.ppu, NAMETABLE_BASE + slot * NAMETABLE_SIZE, 0);
            }
            wr(
                &b.ppu,
                NAMETABLE_BASE + written as u64 * NAMETABLE_SIZE,
                0x80 | written as u8,
            );
            for (slot, cell) in row.iter_mut().enumerate() {
                *cell = rd(&b.ppu, NAMETABLE_BASE + slot as u64 * NAMETABLE_SIZE);
            }
        }
        out
    }

    #[test]
    fn horizontal_mirroring_pairs_the_first_two_nametables() {
        // flags6 bit 0 clear: CIRAM A10 = PPU A11, so $2000/$2400 are one screen.
        let pattern = nametable_pattern(&board(1, 1, 0x00));
        assert_eq!(pattern[0], [0x80, 0x80, 0, 0]);
        assert_eq!(pattern[2], [0, 0, 0x82, 0x82]);
    }

    #[test]
    fn vertical_mirroring_pairs_alternate_nametables() {
        // flags6 bit 0 set: CIRAM A10 = PPU A10, so $2000/$2800 are one screen.
        let pattern = nametable_pattern(&board(1, 1, 0x01));
        assert_eq!(pattern[0], [0x80, 0, 0x80, 0]);
        assert_eq!(pattern[1], [0, 0x81, 0, 0x81]);
    }

    #[test]
    fn four_screen_mirroring_keeps_all_four_distinct() {
        let nrom = board(1, 1, 0x08);
        assert_eq!(nrom.cartridge().mirroring(), Mirroring::FourScreen);
        assert!(nrom.cartridge_vram().is_some());
        let pattern = nametable_pattern(&nrom);
        for (i, row) in pattern.iter().enumerate() {
            let mut want = [0u8; 4];
            want[i] = 0x80 | i as u8;
            assert_eq!(*row, want, "slot {i}");
        }
    }

    /// The published `nametables` region is the wiring, not a bare 4 KiB.
    ///
    /// This is the region a `.machine` file names, and the whole reason it
    /// exists: mapping it puts *this cartridge's* arrangement on the PPU bus,
    /// where `mirror(ciram)` in the machine file put whichever one its author
    /// happened to write down.
    #[test]
    fn the_published_nametable_region_carries_the_cartridge_wiring() {
        for (flags6, want_slot1_of_slot0) in [(0x00u8, true), (0x01, false)] {
            let nrom = board(1, 1, flags6);
            let region = nrom
                .region("nametables")
                .expect("the board publishes its nametables");
            assert_eq!(region.len(), NAMETABLE_WINDOW);

            // Map it exactly as a machine file does — once at $2000 and again,
            // clipped, at $3000 — and check the arrangement through the space.
            let b = bus();
            {
                let mut topo = b.ppu.topology();
                topo.map(Arc::clone(&region), NAMETABLE_BASE).expect("maps");
                topo.map(
                    Arc::new(
                        Region::alias("mirror", region, 0, NAMETABLE_MIRROR_WINDOW).expect("clips"),
                    ),
                    NAMETABLE_MIRROR_BASE,
                )
                .expect("maps");
            }
            wr(&b.ppu, 0x2000, 0x5a);
            assert_eq!(
                rd(&b.ppu, 0x2400) == 0x5a,
                want_slot1_of_slot0,
                "{flags6:#04x}"
            );
            assert_eq!(
                rd(&b.ppu, 0x2800) == 0x5a,
                !want_slot1_of_slot0,
                "{flags6:#04x}"
            );
            // And the $3000 alias is the same storage.
            assert_eq!(rd(&b.ppu, 0x3000), 0x5a);
        }
    }

    #[test]
    fn the_two_ciram_banks_are_distinct_storage() {
        // No iNES header can name the single-screen arrangements — no mapper-0
        // board wires them — so the wiring is asserted at the `banks()` level
        // for the mappers that will select them at run time, and the two CIRAM
        // banks themselves are checked to be real, separate kilobytes.
        assert_eq!(Mirroring::SingleScreenLower.banks(), [0; 4]);
        assert_eq!(Mirroring::SingleScreenUpper.banks(), [1; 4]);

        let nrom = board(1, 1, 0x00);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        // Horizontal, so slot 0 is CIRAM bank 0 and slot 2 is bank 1: the two
        // banks are genuinely distinct storage.
        wr(&b.ppu, 0x2000, 0x11);
        wr(&b.ppu, 0x2800, 0x22);
        assert_eq!(nrom.ciram().read_u8(0).expect("in range"), 0x11);
        assert_eq!(nrom.ciram().read_u8(0x400).expect("in range"), 0x22);
    }

    #[test]
    fn the_nametables_appear_again_at_3000() {
        let nrom = board(1, 1, 0x01);
        let b = bus();
        nrom.install(&b.cpu, &b.ppu).expect("installs");
        wr(&b.ppu, 0x2000, 0x5a);
        assert_eq!(rd(&b.ppu, 0x3000), 0x5a);
        wr(&b.ppu, 0x3eff, 0xa5);
        assert_eq!(rd(&b.ppu, 0x2eff), 0xa5);
        // Palette RAM is the PPU's, not the cartridge's, so $3F00 stays unmapped.
        assert!(b.ppu.locate(0x3f00).is_none());
    }

    #[test]
    fn uninstall_puts_the_spaces_back() {
        let nrom = board(1, 1, 0);
        let b = bus();
        let m = nrom.install(&b.cpu, &b.ppu).expect("installs");
        assert!(b.cpu.locate(0x8000).is_some());
        nrom.uninstall(&b.cpu, &b.ppu, &m).expect("unmaps");
        assert!(b.cpu.locate(0x8000).is_none());
        assert!(b.ppu.locate(0x0000).is_none());
        assert!(b.ppu.locate(0x2000).is_none());
    }

    // -- rejection ---------------------------------------------------------

    #[test]
    fn a_non_nrom_cartridge_is_rejected() {
        let mut img = image(1, 1, 0);
        img[6] |= 0x10; // mapper 1
        let cart = Cartridge::from_ines(&img).expect("valid image");
        let err = Nrom::new(cart).expect_err("not NROM");
        assert!(alloc::format!("{err}").contains("mapper 1"), "{err}");
    }

    #[test]
    fn more_prg_than_the_window_can_address_is_rejected() {
        let cart = Cartridge::from_ines(&image(4, 1, 0)).expect("valid image");
        let err = Nrom::new(cart).expect_err("64 KiB does not fit");
        assert!(alloc::format!("{err}").contains("PRG ROM"), "{err}");
    }

    #[test]
    fn a_non_power_of_two_rom_is_rejected() {
        // NES 2.0 exponent sizes can name 3 KiB, which no address decoding
        // produces.
        let mut h = [0u8; 16];
        h[..4].copy_from_slice(b"NES\x1a");
        h[7] = 0x08;
        h[4] = (10 << 2) | 1; // 2^10 * 3 = 3 KiB
        h[9] = 0x0f;
        h[11] = 0x07;
        let mut img = h.to_vec();
        img.extend(core::iter::repeat_n(0u8, 3072));
        let cart = Cartridge::from_ines(&img).expect("valid image");
        let err = Nrom::new(cart).expect_err("3 KiB is not a power of two");
        assert!(alloc::format!("{err}").contains("address lines"), "{err}");
    }

    #[test]
    fn a_space_that_is_not_a_nes_bus_is_rejected() {
        let nrom = board(1, 1, 0);
        let cpu = AddressSpace::new("cpu", 15);
        let ppu = AddressSpace::new("ppu", 14);
        assert!(nrom.install(&cpu, &ppu).is_err());

        let cpu = AddressSpace::new("cpu", 16);
        let ppu = AddressSpace::new("ppu", 13);
        assert!(nrom.install(&cpu, &ppu).is_err());
    }

    #[test]
    fn construction_needs_a_bound_rom() {
        // No `rom` at all: the message has to say how to supply one, because
        // "missing required property" alone does not tell you that a media
        // slot is a thing.
        let err = (NROM_CLASS.construct)(&Props::new())
            .expect_err("needs an image")
            .to_string();
        assert!(err.contains("rom") && err.contains("media"), "{err}");

        // A bare string is a slot name nothing was bound to. Realize
        // substitutes bound slots before construction, so one that survives is
        // an unbound one and the message must say so rather than complain
        // about a type.
        let err = (NROM_CLASS.construct)(&Props::new().with("rom", "cart"))
            .expect_err("nothing bound")
            .to_string();
        assert!(err.contains("cart"), "{err}");
    }

    #[test]
    fn a_bound_image_constructs_the_board() {
        let bytes: Arc<[u8]> = image(2, 1, 0).into();
        let props = Props::new().with("rom", Media::new("cart", bytes));
        let device = (NROM_CLASS.construct)(&props).expect("a real image");
        assert_eq!(device.class().name, CLASS_NAME);
        // And the regions a `map` statement names are there.
        assert_eq!(device.region("prg").expect("prg").len(), PRG_WINDOW);
        assert_eq!(device.region("chr").expect("chr").len(), CHR_WINDOW);
        assert!(device.region("").is_none(), "no single aperture");
        assert!(device.region("nonesuch").is_none());
    }

    #[test]
    fn a_truncated_image_is_refused_by_name() {
        let bytes: Arc<[u8]> = alloc::vec![0u8; 8].into();
        let props = Props::new().with("rom", Media::new("cart", bytes));
        assert!((NROM_CLASS.construct)(&props).is_err(), "eight bytes");
    }

    #[test]
    fn the_class_registers_once() {
        let mut reg = crate::core::Registry::new();
        register(&mut reg).expect("first registration");
        assert!(reg.get(CLASS_NAME).is_some());
        assert!(register(&mut reg).is_err(), "twice is a feature collision");
    }

    // -- reset and snapshot ------------------------------------------------

    #[test]
    fn a_cold_reset_clears_volatile_ram_but_not_a_battery() {
        let nrom = board(1, 0, 0x02); // CHR RAM, battery
        let chr = nrom.cartridge().chr().as_ram().expect("chr ram").clone();
        let work = nrom.cartridge().work_ram().expect("work ram").clone();
        chr.write_u8(0, 0xaa).expect("in range");
        work.write_u8(0, 0xbb).expect("in range");

        nrom.reset(ResetKind::Warm);
        assert_eq!(
            chr.read_u8(0).expect("in range"),
            0xaa,
            "a reset line is not power"
        );

        nrom.reset(ResetKind::Cold);
        assert_eq!(chr.read_u8(0).expect("in range"), 0x00);
        assert_eq!(
            work.read_u8(0).expect("in range"),
            0xbb,
            "battery-backed RAM survives a power cycle"
        );

        // Without a battery it does not.
        let nrom = board(1, 0, 0x00);
        let work = nrom.cartridge().work_ram().expect("work ram").clone();
        work.write_u8(0, 0xbb).expect("in range");
        nrom.reset(ResetKind::Cold);
        assert_eq!(work.read_u8(0).expect("in range"), 0x00);
    }

    fn snapshot(nrom: &Nrom) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("cart", CLASS_NAME).expect("unique path");
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer
                .chunk("cart", CLASS_NAME, STATE_VERSION)
                .expect("one chunk");
            nrom.save(&mut chunk).expect("saves");
        }
        writer.to_vec().expect("encodes")
    }

    fn restore(nrom: &Nrom, bytes: &[u8]) {
        let reader = StateReader::new(bytes).expect("decodes");
        let chunk = reader
            .load("cart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("finds the chunk");
        let mut r = chunk.reader();
        nrom.load(&mut r).expect("loads");
    }

    #[test]
    fn state_round_trips_to_an_identical_hash() {
        // Four-screen so that all three mutable stores are present.
        let nrom = board(1, 0, 0x08 | 0x01);
        let chr = nrom.cartridge().chr().as_ram().expect("chr ram").clone();
        let work = nrom.cartridge().work_ram().expect("work ram").clone();
        let vram = nrom.cartridge_vram().expect("four-screen vram").clone();
        for (i, store) in [&chr, &work, &vram].into_iter().enumerate() {
            for off in 0..64u64 {
                store
                    .write_u8(off, (off as u8).wrapping_mul(7).wrapping_add(i as u8))
                    .expect("in range");
            }
        }
        let saved = snapshot(&nrom);

        // A second, identical board with different contents.
        let other = board(1, 0, 0x08 | 0x01);
        other
            .cartridge()
            .chr()
            .as_ram()
            .expect("chr ram")
            .write_u8(0, 0xff)
            .expect("in range");
        assert_ne!(snapshot(&other), saved, "the boards start out different");

        restore(&other, &saved);
        assert_eq!(snapshot(&other), saved, "state hash must match after load");

        // ...and the bytes really are the same, not just the encoding.
        let mut a = [0u8; 64];
        let mut b = [0u8; 64];
        chr.read_at(0, &mut a).expect("in range");
        other
            .cartridge()
            .chr()
            .as_ram()
            .expect("chr ram")
            .read_at(0, &mut b)
            .expect("in range");
        assert_eq!(a, b);
    }

    #[test]
    fn a_snapshot_does_not_carry_rom() {
        // 32 KiB of PRG and 8 KiB of CHR, all of it constant: the chunk must be
        // the work RAM and nothing else.
        let nrom = board(2, 1, 0);
        let mut shape = MachineShape::new();
        shape.add_device("cart", CLASS_NAME).expect("unique path");
        let mut writer = StateWriter::new(shape);
        let mut chunk = writer
            .chunk("cart", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        nrom.save(&mut chunk).expect("saves");
        // 8 KiB of work RAM and 2 KiB of CIRAM, each with a length prefix,
        // plus four presence bytes. No ROM.
        assert_eq!(chunk.len(), 8192 + 8 + 2048 + 8 + 4);
    }

    #[test]
    fn a_snapshot_from_a_differently_shaped_board_is_refused() {
        let with_vram = board(1, 0, 0x08);
        let saved = snapshot(&with_vram);
        let without = board(1, 0, 0x00);
        let reader = StateReader::new(&saved).expect("decodes");
        let chunk = reader
            .load("cart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("finds the chunk");
        let mut r = chunk.reader();
        let err = without.load(&mut r).expect_err("shapes disagree");
        assert!(alloc::format!("{err}").contains("VRAM"), "{err}");
    }

    #[test]
    fn a_truncated_chunk_is_an_error_not_a_panic() {
        let nrom = board(1, 0, 0);
        let mut shape = MachineShape::new();
        shape.add_device("cart", CLASS_NAME).expect("unique path");
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer
                .chunk("cart", CLASS_NAME, STATE_VERSION)
                .expect("one chunk");
            nrom.save(&mut chunk).expect("saves");
        }
        let bytes = writer.to_vec().expect("encodes");
        // Every prefix of the encoded snapshot: either it decodes or it errors.
        for n in 0..bytes.len().min(256) {
            let _ = StateReader::new(&bytes[..n]);
        }
        // And a chunk cut short mid-payload.
        let reader = StateReader::new(&bytes).expect("decodes");
        let (_, _, data) = reader.load_raw("cart").expect("raw chunk");
        for n in 0..data.len().min(64) {
            let mut r = ChunkReader::new(&data[..n]);
            let _ = nrom.load(&mut r);
        }
    }

    #[test]
    fn the_device_trait_is_wired_up() {
        let nrom = board(1, 1, 0);
        assert_eq!(nrom.class().name, CLASS_NAME);
        assert_eq!(nrom.class().version, STATE_VERSION);
        let mut deferred = crate::core::device::Deferred::new();
        let mut ctx = RealizeCtx::new("cart", crate::core::space::RequesterId(1), &mut deferred);
        nrom.realize(&mut ctx).expect("realizes");
        nrom.unrealize(&mut ctx).expect("unrealizes");
    }

    #[test]
    fn a_board_is_send_and_sync() {
        // Devices are `Send + Sync` from the first commit (`ROADMAP.md` §0), so
        // assert it here rather than discovering it when threading lands.
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Nrom>();
        assert_send_sync::<Cartridge>();
    }
}
