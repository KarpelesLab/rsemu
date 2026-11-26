//! A SNES cartridge: ROM, optional battery-backed SRAM, and the address
//! decoding its board wiring implies.
//!
//! A SNES cartridge has no mapper chip in the NES sense. Which 24-bit A-bus
//! address reaches which ROM byte is fixed by how the board routes address
//! lines to the ROM and SRAM chips, and the three wirings nearly every
//! cartridge uses have names:
//!
//! | Map | ROM | SRAM |
//! | --- | --- | --- |
//! | **LoROM** (mode `$20`) | 32 KiB per bank at `$8000–$FFFF` of banks `$00–$7D`/`$80–$FF`; the A15 line is skipped, so bank `n` holds ROM `n × 32 KiB` | banks `$70–$7D`/`$F0–$FF`, `$0000–$7FFF` |
//! | **HiROM** (mode `$21`) | all 64 KiB of banks `$40–$7D`/`$C0–$FF`; their upper halves again at `$8000–$FFFF` of `$00–$3F`/`$80–$BF` | banks `$20–$3F`/`$A0–$BF`, `$6000–$7FFF`, 8 KiB a bank |
//! | **ExHiROM** (mode `$25`) | HiROM with A23 inverted onto ROM A22: banks `$C0–$FF` (and `$80–$BF` high halves) are the first 4 MiB, `$40–$7D` (and `$00–$3F` high halves) the next 4 MiB | banks `$80–$BF`, `$6000–$7FFF` |
//!
//! The device decodes the **whole 24-bit address** itself, so the board maps
//! its one region wherever the cartridge port is decoded — every bank's
//! `$8000–$FFFF`, all of banks `$40–$7D`/`$C0–$FF`, and the `$6000–$7FFF`
//! windows of the system banks — with the region offset equal to the A-bus
//! address. An address the cartridge does not drive answers
//! [`BusError::Unassigned`], so the CPU sees its own open bus there rather
//! than a value this file made up.
//!
//! # Which map
//!
//! The internal header — 64 bytes ending at `$FFFF` of the CPU's view, so at
//! ROM `$7FC0` for LoROM, `$FFC0` for HiROM and `$40FFC0` for ExHiROM —
//! carries the map mode at `+$15`, the cartridge type (whether there is SRAM
//! and a battery) at `+$16`, the ROM and SRAM sizes as powers of two at `+$17`
//! and `+$18`, the destination code at `+$19`, a checksum and its complement
//! at `+$1C–$1F`, and the vectors in the 32 bytes after. Images in the wild
//! come with no flag saying which header to trust, so each candidate is
//! scored on what a real one must satisfy — a map-mode byte that names the
//! map the header sits at, checksum + complement = `$FFFF`, a reset vector
//! inside the ROM window, a printable title — and the best wins. The `map`
//! property overrides the guess. A 512-byte copier header (an image whose
//! length is 512 more than a multiple of 1 KiB) is stripped first.
//!
//! # Mirroring
//!
//! A ROM whose size is not a power of two is two chips (or one chip with an
//! unused upper part): the larger power-of-two part answers the lower half of
//! the decoded window and the remainder is mirrored to fill the upper half,
//! recursively. [`mirror`] says it in code; a 3 MiB image reads its last
//! 1 MiB twice across the upper 2 MiB.
//!
//! # Sources
//!
//! nocash **fullsnes**, "SNES Cartridges" (the LoROM/HiROM/ExHiROM memory
//! maps, the cartridge header and its field encodings, SRAM placement); the
//! **SNESdev wiki**, "Memory map" and "ROM header". No emulator source of any
//! licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::value::Width;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "snes.cart";

/// The one region: the A-bus, decoded by full 24-bit address.
pub const BUS_REGION: &str = "bus";

/// The size of the A-bus the region spans.
pub const BUS_SIZE: u64 = 1 << 24;

/// The largest SRAM a header can declare that this model accepts: 256 KiB
/// (`$08`), past anything a commercial cartridge carries.
const MAX_SRAM_SHIFT: u8 = 8;

/// The snapshot chunk version.
const STATE_VERSION: u32 = 1;

/// How the cartridge board routes the address lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapMode {
    /// 32 KiB banks at `$8000–$FFFF`; map mode `$20`/`$30`.
    LoRom,
    /// 64 KiB banks at `$40–$7D`/`$C0–$FF`; map mode `$21`/`$31`.
    HiRom,
    /// HiROM for images over 4 MiB; map mode `$25`/`$35`.
    ExHiRom,
}

impl MapMode {
    /// Where this map's internal header sits in the ROM image.
    #[must_use]
    pub const fn header_offset(self) -> usize {
        match self {
            MapMode::LoRom => 0x7FC0,
            MapMode::HiRom => 0xFFC0,
            MapMode::ExHiRom => 0x40_FFC0,
        }
    }

    /// The map-mode byte's low nibble this map is announced with.
    const fn mode_nibble(self) -> u8 {
        match self {
            MapMode::LoRom => 0x0,
            MapMode::HiRom => 0x1,
            MapMode::ExHiRom => 0x5,
        }
    }

    /// The name the `map` property spells this map with.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            MapMode::LoRom => "lorom",
            MapMode::HiRom => "hirom",
            MapMode::ExHiRom => "exhirom",
        }
    }
}

/// What the internal header says, once a map has been chosen.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    /// The map the image is decoded with.
    pub map: MapMode,
    /// The 21-character title, trailing spaces trimmed, non-ASCII as `?`.
    pub title: String,
    /// The map-mode byte (`+$15`); bit 4 is the FastROM flag.
    pub map_byte: u8,
    /// The cartridge type byte (`+$16`).
    pub cart_type: u8,
    /// SRAM in bytes: zero if the type byte says there is none.
    pub sram_len: u64,
    /// Whether the type byte says the SRAM has a battery.
    pub battery: bool,
    /// The destination code (`+$19`): `$00` Japan, `$01` North America,
    /// `$02`–`$0C` the PAL territories.
    pub region: u8,
    /// Whether checksum + complement made `$FFFF`.
    pub checksum_ok: bool,
}

impl Header {
    /// Whether the header asks for 3.58 MHz ROM access (`MEMSEL` is still
    /// what the S-CPU obeys; this is only what the game intends to set).
    #[must_use]
    pub const fn fast_rom(&self) -> bool {
        self.map_byte & 0x10 != 0
    }

    /// Whether the destination code is a PAL territory (fullsnes, "SNES
    /// Cartridge ROM Header": `$02`–`$0C` are Europe, Scandinavia, France,
    /// Netherlands, Spain, Germany, Italy, China, Indonesia, Korea and the
    /// rest of the world... of which `$0D` onward are NTSC again).
    #[must_use]
    pub const fn is_pal(&self) -> bool {
        matches!(self.region, 0x02..=0x0C)
    }
}

/// A parsed image: the bytes (copier header removed) and what the header says.
#[derive(Clone)]
pub struct Image {
    rom: Arc<[u8]>,
    header: Header,
}

impl fmt::Debug for Image {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Image")
            .field("len", &self.rom.len())
            .field("header", &self.header)
            .finish()
    }
}

impl Image {
    /// Parse an image, guessing the map unless `force` names one.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image is empty or too short to hold the
    /// header of the map it was forced to.
    pub fn parse(bytes: &[u8], force: Option<MapMode>) -> Result<Image> {
        let bytes = if bytes.len() % 1024 == 512 {
            &bytes[512..]
        } else {
            bytes
        };
        if bytes.is_empty() {
            return Err(config("a SNES cartridge image cannot be empty"));
        }
        let map = match force {
            Some(map) => map,
            None => [MapMode::LoRom, MapMode::HiRom, MapMode::ExHiRom]
                .into_iter()
                .filter(|m| m.header_offset() + 0x40 <= bytes.len())
                .max_by_key(|m| score(bytes, *m))
                // An image too small for any header is a 32 KiB-or-less
                // LoROM test program: say so and decode it as one.
                .unwrap_or(MapMode::LoRom),
        };
        let header = read_header(bytes, map);
        Ok(Image {
            rom: Arc::from(bytes),
            header,
        })
    }

    /// What the header says.
    #[must_use]
    pub fn header(&self) -> &Header {
        &self.header
    }

    /// The ROM bytes, copier header removed.
    #[must_use]
    pub fn rom(&self) -> &[u8] {
        &self.rom
    }
}

fn config(message: &str) -> Error {
    Error::Config {
        at: CLASS_NAME.into(),
        message: message.into(),
    }
}

/// How plausible it is that `bytes` carries its real header at `map`'s spot.
fn score(bytes: &[u8], map: MapMode) -> i32 {
    let h = map.header_offset();
    let Some(head) = bytes.get(h..h + 0x40) else {
        return i32::MIN;
    };
    let mut s = 0;
    let mode = head[0x15];
    if mode & 0xE0 == 0x20 && mode & 0x0F == map.mode_nibble() {
        s += 4;
    }
    let sum = u16::from_le_bytes([head[0x1E], head[0x1F]]);
    let complement = u16::from_le_bytes([head[0x1C], head[0x1D]]);
    if sum.wrapping_add(complement) == 0xFFFF {
        s += 4;
    }
    // The emulation-mode reset vector must point into the ROM window.
    let reset = u16::from_le_bytes([head[0x3C], head[0x3D]]);
    if reset >= 0x8000 {
        s += 2;
    }
    if head[..21].iter().all(|b| (0x20..0x7F).contains(b)) {
        s += 2;
    }
    if head[0x17] != 0 && head[0x17] <= 0x0D {
        s += 1;
    }
    s
}

fn read_header(bytes: &[u8], map: MapMode) -> Header {
    let h = map.header_offset();
    let head: Vec<u8> = (0..0x40)
        .map(|i| bytes.get(h + i).copied().unwrap_or(0))
        .collect();
    let title: String = head[..21]
        .iter()
        .map(|&b| {
            if (0x20..0x7F).contains(&b) {
                b as char
            } else {
                '?'
            }
        })
        .collect::<String>()
        .trim_end()
        .into();
    let cart_type = head[0x16];
    // The type byte's low nibble: 0 ROM only, 1 ROM+RAM, 2 ROM+RAM+battery,
    // 3–6 the same with a coprocessor (3 ROM+co, 4 +RAM, 5 +RAM+battery,
    // 6 +battery).
    let (has_ram, battery) = match cart_type & 0x0F {
        0x1 | 0x4 => (true, false),
        0x2 | 0x5 => (true, true),
        _ => (false, false),
    };
    let shift = head[0x18].min(MAX_SRAM_SHIFT);
    let sram_len = if has_ram && head[0x18] != 0 {
        1024u64 << shift
    } else {
        0
    };
    let sum = u16::from_le_bytes([head[0x1E], head[0x1F]]);
    let complement = u16::from_le_bytes([head[0x1C], head[0x1D]]);
    Header {
        map,
        title,
        map_byte: head[0x15],
        cart_type,
        sram_len,
        battery,
        region: head[0x19],
        checksum_ok: sum.wrapping_add(complement) == 0xFFFF,
    }
}

/// Reduce a decoded ROM offset to one inside an image of `len` bytes.
///
/// The decoded window is a power of two. A `len` that is one too is simply
/// wrapped; otherwise the largest power-of-two part `p` answers `[0, p)` and
/// whatever is left is a smaller ROM mirrored across `[p, 2p)` by the same
/// rule.
#[must_use]
pub fn mirror(offset: u64, len: u64) -> u64 {
    if len == 0 {
        return 0;
    }
    let p = 1u64 << (63 - len.leading_zeros());
    let offset = offset % (2 * p);
    if offset < p {
        offset % p
    } else if len == p {
        offset - p
    } else {
        p + mirror(offset - p, len - p)
    }
}

/// What an A-bus address reaches on this cartridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Target {
    Rom(u64),
    Sram(u64),
    None,
}

fn decode(map: MapMode, addr: u32, rom_len: u64, sram_len: u64) -> Target {
    let bank = (addr >> 16) as u8;
    let low = u64::from(addr & 0xFFFF);
    let b = u64::from(bank);
    let sram = |offset: u64| {
        if sram_len == 0 {
            Target::None
        } else {
            Target::Sram(offset % sram_len)
        }
    };
    match map {
        MapMode::LoRom => {
            if (0x70..=0x7D).contains(&(bank & 0x7F)) && low < 0x8000 {
                // Banks $70–$7D and $F0–$FF, the lower half: SRAM, 32 KiB
                // a bank.
                return sram(((b & 0x0F) << 15) | low);
            }
            if low >= 0x8000 && bank & 0x7F != 0x7E && bank & 0x7F != 0x7F {
                return Target::Rom(mirror(((b & 0x7F) << 15) | (low & 0x7FFF), rom_len));
            }
            // $40–$6F lower halves: the same ROM byte as the upper half
            // (A15 is not wired).
            if (0x40..=0x6F).contains(&(bank & 0x7F)) {
                return Target::Rom(mirror(((b & 0x7F) << 15) | (low & 0x7FFF), rom_len));
            }
            Target::None
        }
        MapMode::HiRom => {
            let half = bank & 0x7F;
            if (0x20..=0x3F).contains(&half) && (0x6000..0x8000).contains(&low) {
                return sram(((b & 0x1F) << 13) | (low - 0x6000));
            }
            if (0x40..=0x7D).contains(&half) || bank >= 0xC0 {
                return Target::Rom(mirror(((b & 0x3F) << 16) | low, rom_len));
            }
            if half < 0x40 && low >= 0x8000 {
                return Target::Rom(mirror(((b & 0x3F) << 16) | low, rom_len));
            }
            Target::None
        }
        MapMode::ExHiRom => {
            // A23 inverted onto ROM A22: the $80–$FF half is the first
            // 4 MiB.
            let upper = if bank & 0x80 != 0 { 0 } else { 0x40_0000 };
            let half = bank & 0x7F;
            if (0x80..=0xBF).contains(&bank) && (0x6000..0x8000).contains(&low) {
                return sram(((b & 0x1F) << 13) | (low - 0x6000));
            }
            if (0x40..=0x7D).contains(&half) || bank >= 0xC0 || (half < 0x40 && low >= 0x8000) {
                return Target::Rom(mirror(upper | ((b & 0x3F) << 16) | low, rom_len));
            }
            Target::None
        }
    }
}

/// The `snes.cart` device.
pub struct SnesCart {
    image: Image,
    sram: Option<Arc<RamStore>>,
    bus: Arc<Bus>,
    region: RegionRef,
}

impl fmt::Debug for SnesCart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SnesCart")
            .field("image", &self.image)
            .finish_non_exhaustive()
    }
}

struct Bus {
    map: MapMode,
    rom: Arc<[u8]>,
    sram: Option<Arc<RamStore>>,
}

impl fmt::Debug for Bus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bus")
            .field("map", &self.map)
            .finish_non_exhaustive()
    }
}

impl Bus {
    fn target(&self, offset: u64) -> Target {
        let sram_len = self.sram.as_ref().map_or(0, |s| s.len());
        decode(self.map, offset as u32, self.rom.len() as u64, sram_len)
    }
}

impl MemOps for Bus {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        match self.target(offset) {
            Target::Rom(at) => *byte = self.rom[at as usize],
            Target::Sram(at) => {
                *byte = self.sram.as_ref().expect("decoded to SRAM").read_u8(at)?;
            }
            Target::None => return Err(BusError::Unassigned),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = src else {
            return Err(BusError::BadAccess);
        };
        match self.target(offset) {
            // A write to ROM goes nowhere; the bus cycle still completes.
            Target::Rom(_) => Ok(()),
            Target::Sram(at) => self
                .sram
                .as_ref()
                .expect("decoded to SRAM")
                .write_u8(at, *byte),
            Target::None => Err(BusError::Unassigned),
        }
    }

    fn constraints(&self) -> AccessConstraints {
        // The A-bus is eight bits wide; the CPU makes every access a byte.
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

impl SnesCart {
    /// Build a cartridge around a parsed image.
    #[must_use]
    pub fn new(image: Image) -> SnesCart {
        let sram_len = image.header.sram_len;
        let sram = (sram_len > 0).then(|| Arc::new(RamStore::new(sram_len)));
        let bus = Arc::new(Bus {
            map: image.header.map,
            rom: Arc::clone(&image.rom),
            sram: sram.clone(),
        });
        let region = Arc::new(MmioRegion::io(
            "snes.cart",
            BUS_SIZE,
            Arc::clone(&bus) as Arc<dyn MemOps>,
        ));
        SnesCart {
            image,
            sram,
            bus,
            region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If the `rom` media slot is missing, `map` names no map, or the image
    /// does not parse.
    pub fn from_props(props: &Props) -> Result<SnesCart> {
        let mut r = props.reader();
        let media = r.require_media("rom")?;
        let force = match r.or_enum("map", "auto", &["auto", "lorom", "hirom", "exhirom"])? {
            "lorom" => Some(MapMode::LoRom),
            "hirom" => Some(MapMode::HiRom),
            "exhirom" => Some(MapMode::ExHiRom),
            _ => None,
        };
        r.finish()?;
        Ok(SnesCart::new(Image::parse(&media.to_bytes(), force)?))
    }

    /// The parsed image and its header.
    #[must_use]
    pub fn image(&self) -> &Image {
        &self.image
    }

    /// What the cartridge drives at A-bus address `addr`, or `None` where it
    /// drives nothing. Side-effect free: a monitor or a test may call it.
    #[must_use]
    pub fn peek(&self, addr: u32) -> Option<u8> {
        let mut b = [0u8];
        self.bus
            .read(u64::from(addr), &mut b, MemAttrs::DEBUG)
            .ok()
            .map(|()| b[0])
    }
}

/// The `snes.cart` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "SNES cartridge: LoROM, HiROM and ExHiROM decoding, battery SRAM, header detection",
    properties: &[
        PropertySpec {
            name: "rom",
            kind: ValueKind::Media,
            required: true,
            summary: "the cartridge image, as the name of a media slot (`rom = \"cart\"`)",
        },
        PropertySpec {
            name: "map",
            kind: ValueKind::Str,
            required: false,
            summary: "`auto` (default: scored from the internal header), `lorom`, `hirom` or \
                      `exhirom`",
        },
    ],
    construct: |props| Ok(Box::new(SnesCart::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SnesCart {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | BUS_REGION).then(|| Arc::clone(&self.region))
    }

    fn reset(&self, kind: ResetKind) {
        // A battery keeps SRAM through anything; without one, a cold start
        // is a power cycle and the cells come up cleared.
        if kind == ResetKind::Cold
            && !self.image.header.battery
            && let Some(sram) = &self.sram
        {
            let _ = sram.fill(0, sram.len(), 0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // SRAM is architectural state (and with a battery, the player's
        // save): a snapshot that dropped it would lose it.
        match &self.sram {
            Some(sram) => {
                let mut bytes = vec![0u8; sram.len() as usize];
                sram.read_at(0, &mut bytes)
                    .map_err(|e| Error::State(alloc::format!("cartridge SRAM: {e}")))?;
                w.write_bytes(&bytes)
            }
            None => w.write_bytes(&[]),
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes = r.read_bytes()?;
        let len = self.sram.as_ref().map_or(0, |s| s.len());
        if bytes.len() as u64 != len {
            return Err(Error::State(alloc::format!(
                "snapshot holds {} bytes of cartridge SRAM, this cartridge has {len}",
                bytes.len()
            )));
        }
        if let Some(sram) = &self.sram {
            sram.write_at(0, bytes)
                .map_err(|e| Error::State(alloc::format!("cartridge SRAM: {e}")))?;
        }
        Ok(())
    }
}

impl crate::machine::Instance for SnesCart {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(SnesCart::from_props(props)?))
    })
}

/// What the validator should know about `snes.cart`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .prop(PropSchema::new("map", ValueKind::Str).values(&["auto", "lorom", "hirom", "exhirom"]))
        .region("")
        .region(BUS_REGION)
}

/// Build a minimal image with a valid header for `map`: `len` bytes, the
/// title, the type and SRAM-size bytes, the checksum pair, and a reset vector
/// at `$8000`. For tests and demonstrations.
#[must_use]
pub fn test_image(map: MapMode, len: usize, cart_type: u8, sram_shift: u8) -> Vec<u8> {
    let mut rom = vec![0u8; len];
    let h = map.header_offset();
    rom[h..h + 21].copy_from_slice(b"RSEMU TEST CART      ");
    rom[h + 0x15] = 0x20 | map.mode_nibble();
    rom[h + 0x16] = cart_type;
    rom[h + 0x17] = (len.trailing_zeros() as u8).saturating_sub(10);
    rom[h + 0x18] = sram_shift;
    rom[h + 0x19] = 0x01;
    rom[h + 0x3C] = 0x00;
    rom[h + 0x3D] = 0x80;
    let sum = rom
        .iter()
        .fold(0u16, |acc, &b| acc.wrapping_add(u16::from(b)))
        .wrapping_add(0x01FE);
    rom[h + 0x1C..h + 0x1E].copy_from_slice(&(!sum).to_le_bytes());
    rom[h + 0x1E..h + 0x20].copy_from_slice(&sum.to_le_bytes());
    rom
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(cart: &SnesCart, addr: u32) -> MemResult<u8> {
        let mut b = [0u8];
        cart.bus
            .read(u64::from(addr), &mut b, MemAttrs::DEFAULT)
            .map(|()| b[0])
    }

    fn write(cart: &SnesCart, addr: u32, v: u8) -> MemResult {
        cart.bus.write(u64::from(addr), &[v], MemAttrs::DEFAULT)
    }

    fn marked(map: MapMode, len: usize, cart_type: u8, sram: u8) -> Vec<u8> {
        let mut rom = test_image(map, len, cart_type, sram);
        // A marker every 32 KiB, away from the header.
        for (i, chunk) in rom.chunks_mut(0x8000).enumerate() {
            chunk[0x100] = i as u8;
        }
        rom
    }

    #[test]
    fn the_header_picks_the_map() {
        for map in [MapMode::LoRom, MapMode::HiRom] {
            let image = Image::parse(&test_image(map, 0x10_0000, 0, 0), None).unwrap();
            assert_eq!(image.header().map, map);
            assert!(image.header().checksum_ok);
            assert_eq!(image.header().title, "RSEMU TEST CART");
        }
        let image = Image::parse(&test_image(MapMode::ExHiRom, 0x60_0000, 0, 0), None).unwrap();
        assert_eq!(image.header().map, MapMode::ExHiRom);
    }

    #[test]
    fn a_copier_header_is_stripped() {
        let mut bytes = vec![0xAA; 512];
        bytes.extend(test_image(MapMode::HiRom, 0x8_0000, 0, 0));
        let image = Image::parse(&bytes, None).unwrap();
        assert_eq!(image.rom().len(), 0x8_0000);
        assert_eq!(image.header().map, MapMode::HiRom);
    }

    #[test]
    fn lorom_banks_are_32k_at_the_top_of_each_bank() {
        let cart =
            SnesCart::new(Image::parse(&marked(MapMode::LoRom, 0x8_0000, 0, 0), None).unwrap());
        assert_eq!(read(&cart, 0x00_8100).unwrap(), 0);
        assert_eq!(read(&cart, 0x01_8100).unwrap(), 1);
        assert_eq!(read(&cart, 0x0F_8100).unwrap(), 15);
        // $80+ is the same ROM (the FastROM half), and 512 KiB mirrors.
        assert_eq!(read(&cart, 0x81_8100).unwrap(), 1);
        assert_eq!(read(&cart, 0x10_8100).unwrap(), 0);
        // The lower half of a system bank is not the cartridge's.
        assert_eq!(read(&cart, 0x00_6000), Err(BusError::Unassigned));
    }

    #[test]
    fn lorom_sram_answers_at_70_and_survives_a_warm_reset() {
        let cart =
            SnesCart::new(Image::parse(&marked(MapMode::LoRom, 0x8_0000, 2, 3), None).unwrap());
        assert_eq!(cart.image().header().sram_len, 8 * 1024);
        write(&cart, 0x70_0000, 0x5A).unwrap();
        assert_eq!(read(&cart, 0x70_0000).unwrap(), 0x5A);
        // 8 KiB mirrors through the bank.
        assert_eq!(read(&cart, 0x70_2000).unwrap(), 0x5A);
        assert_eq!(read(&cart, 0xF0_0000).unwrap(), 0x5A);
        Device::reset(&cart, ResetKind::Cold);
        assert_eq!(read(&cart, 0x70_0000).unwrap(), 0x5A, "battery-backed");
    }

    #[test]
    fn hirom_maps_64k_banks_and_sram_at_6000() {
        let cart =
            SnesCart::new(Image::parse(&marked(MapMode::HiRom, 0x10_0000, 1, 1), None).unwrap());
        // Bank $C1 is ROM $10000, whose second 32 KiB chunk is marker 3.
        assert_eq!(read(&cart, 0xC1_8100).unwrap(), 3);
        assert_eq!(read(&cart, 0xC1_0100).unwrap(), 2);
        assert_eq!(read(&cart, 0x41_0100).unwrap(), 2);
        // The upper halves again in the system banks.
        assert_eq!(read(&cart, 0x01_8100).unwrap(), 3);
        assert_eq!(read(&cart, 0x01_0100), Err(BusError::Unassigned));
        write(&cart, 0x30_6000, 0x42).unwrap();
        assert_eq!(read(&cart, 0xB0_6000).unwrap(), 0x42);
        Device::reset(&cart, ResetKind::Cold);
        assert_eq!(read(&cart, 0x30_6000).unwrap(), 0, "no battery: cleared");
    }

    #[test]
    fn exhirom_puts_the_first_4m_at_c0() {
        let cart =
            SnesCart::new(Image::parse(&marked(MapMode::ExHiRom, 0x60_0000, 0, 0), None).unwrap());
        assert_eq!(read(&cart, 0xC0_0100).unwrap(), 0);
        // $40 is ROM $400000: chunk 128.
        assert_eq!(read(&cart, 0x40_0100).unwrap(), 128);
    }

    #[test]
    fn mirroring_follows_the_chip_split() {
        let len = 3 << 20;
        assert_eq!(mirror(0x10_0000, len), 0x10_0000);
        assert_eq!(mirror(0x20_0000, len), 0x20_0000);
        assert_eq!(mirror(0x30_0000, len), 0x20_0000, "the 1 MiB part twice");
        assert_eq!(mirror(0x40_0000, len), 0);
        assert_eq!(mirror(0x1234, 0x8000), 0x1234);
        assert_eq!(mirror(0x9234, 0x8000), 0x1234);
    }

    #[test]
    fn sram_survives_a_snapshot() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
        let image = Image::parse(&marked(MapMode::LoRom, 0x8_0000, 2, 1), None).unwrap();
        let saved = SnesCart::new(image.clone());
        write(&saved, 0x70_0123, 0x77).unwrap();
        let mut shape = MachineShape::new();
        shape.add_device("cart", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("cart", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let restored = SnesCart::new(image);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("cart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(read(&restored, 0x70_0123).unwrap(), 0x77);
    }

    #[test]
    fn a_forced_map_wins() {
        let image = Image::parse(
            &test_image(MapMode::LoRom, 0x10_0000, 0, 0),
            Some(MapMode::HiRom),
        )
        .unwrap();
        assert_eq!(image.header().map, MapMode::HiRom);
        assert_eq!(MapMode::HiRom.name(), "hirom");
    }
}
