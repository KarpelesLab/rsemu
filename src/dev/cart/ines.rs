//! iNES and NES 2.0 cartridge images.
//!
//! Sources: the NESdev wiki pages [iNES](https://www.nesdev.org/wiki/INES),
//! [NES 2.0](https://www.nesdev.org/wiki/NES_2.0) and
//! [Cartridge connector](https://www.nesdev.org/wiki/Cartridge_connector).
//! Those describe the hardware and a community file format, which is the only
//! kind of source this crate is allowed to work from (`ROADMAP.md` §1).
//!
//! # The format, in one table
//!
//! A 16-byte header, an optional 512-byte trainer, PRG ROM, CHR ROM, and
//! whatever a ripper appended afterwards.
//!
//! | Byte | iNES 1.0 | NES 2.0 (byte 7 bits 2-3 == `0b10`) |
//! | --- | --- | --- |
//! | 0-3 | `NES\x1a` | same |
//! | 4 | PRG ROM, 16 KiB units | low 8 bits of the same, or an exponent |
//! | 5 | CHR ROM, 8 KiB units | low 8 bits of the same, or an exponent |
//! | 6 | mirroring, battery, trainer, mapper bits 0-3 | same |
//! | 7 | console type, format marker, mapper bits 4-7 | same |
//! | 8 | PRG RAM, 8 KiB units | mapper bits 8-11, submapper |
//! | 9 | TV system | high 4 bits of the PRG and CHR ROM sizes |
//! | 10 | unofficial | PRG RAM / EEPROM shift counts |
//! | 11 | padding | CHR RAM / NVRAM shift counts |
//! | 12 | padding | CPU/PPU timing |
//! | 13 | padding | Vs. System or extended console type |
//! | 14 | padding | miscellaneous ROM count |
//! | 15 | padding | default expansion device |
//!
//! # This is a parser on untrusted input
//!
//! Every accessor is bounds-checked, every size is computed with checked
//! arithmetic, and **no allocation is sized from a header field before the
//! bytes it claims have been shown to exist**. A malformed image produces a
//! [`RomError`] naming the part that is wrong and the numbers that disagree; it
//! never panics and never allocates unboundedly. The unit tests at the bottom
//! include a fuzz-shaped sweep that truncates a good image at every length and
//! feeds pseudo-random garbage through the same door.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::Error;
use crate::core::space::{RamStore, RomStore};

/// The four bytes every image starts with.
const MAGIC: [u8; 4] = *b"NES\x1a";

/// Length of the header, in bytes.
const HEADER_LEN: u64 = 16;

/// Length of the optional trainer, in bytes. Historically loaded at `$7000`.
const TRAINER_LEN: u64 = 512;

/// PRG ROM size unit: 16 KiB.
const PRG_UNIT: u64 = 16 * 1024;

/// CHR ROM size unit: 8 KiB.
const CHR_UNIT: u64 = 8 * 1024;

/// The 8 KiB an iNES 1.0 image implies wherever it does not say.
const DEFAULT_8K: u64 = 8 * 1024;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Which part of an image an error is about.
///
/// Carried in [`RomError`] because "truncated" on its own sends the reader back
/// to a hex editor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RomPart {
    /// The 16-byte header.
    Header,
    /// The 512-byte trainer.
    Trainer,
    /// Program ROM, mapped into the CPU's address space.
    PrgRom,
    /// Character ROM, mapped into the PPU's address space.
    ChrRom,
}

impl RomPart {
    /// The name used in error messages.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            RomPart::Header => "header",
            RomPart::Trainer => "trainer",
            RomPart::PrgRom => "PRG ROM",
            RomPart::ChrRom => "CHR ROM",
        }
    }
}

impl fmt::Display for RomPart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Why an image could not be read.
///
/// A dedicated enum rather than a bare [`Error`] because these are the errors a
/// caller genuinely branches on — a front end retries a bad-magic file as a raw
/// binary, and a fuzz harness asserts *which* rejection it got. It converts
/// into the crate-level [`Error`] at the API boundary, so `?` still works
/// everywhere else.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum RomError {
    /// The file is shorter than the 16-byte header.
    HeaderTooShort {
        /// How many bytes the file has.
        have: u64,
    },
    /// The file does not begin with `NES\x1a`.
    BadMagic {
        /// The four bytes that were there instead.
        found: [u8; 4],
    },
    /// The header describes a cartridge with no program ROM.
    ///
    /// Such a cartridge cannot supply a reset vector, so it cannot boot. iNES
    /// 1.0 has no way to spell "256 units" either, so a zero here is a broken
    /// dump rather than a large one.
    NoPrgRom,
    /// A NES 2.0 exponent-encoded size is larger than a `u64` can hold.
    ///
    /// The encoding is `2^exponent * (2 * multiplier + 1)` with a 6-bit
    /// exponent, so it can name sizes no machine will ever have.
    SizeOverflow {
        /// Which size.
        what: RomPart,
        /// The 6-bit exponent from the size byte.
        exponent: u8,
        /// The 2-bit multiplier code from the size byte.
        multiplier: u8,
    },
    /// The file is shorter than the header says it is.
    Truncated {
        /// Which part is missing or incomplete.
        what: RomPart,
        /// Where it was supposed to start.
        offset: u64,
        /// How many bytes it was supposed to be.
        need: u64,
        /// How many bytes the file actually has.
        have: u64,
    },
    /// A size fits in a `u64` but not in this host's `usize`.
    ///
    /// Separated from [`RomError::Truncated`] because it is a property of the
    /// host, not of the file: the same image loads on a 64-bit build.
    TooLargeForHost {
        /// Which size.
        what: RomPart,
        /// How large it claims to be.
        len: u64,
    },
}

impl fmt::Display for RomError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RomError::HeaderTooShort { have } => write!(
                f,
                "not an iNES image: {have} byte(s), which is shorter than the {HEADER_LEN}-byte header"
            ),
            RomError::BadMagic { found } => write!(
                f,
                "not an iNES image: starts with {:02x?}, not {:02x?}",
                found, MAGIC
            ),
            RomError::NoPrgRom => f.write_str("the header describes 0 bytes of PRG ROM"),
            RomError::SizeOverflow {
                what,
                exponent,
                multiplier,
            } => write!(
                f,
                "{what} size 2^{exponent} * {} overflows a 64-bit byte count",
                multiplier * 2 + 1
            ),
            RomError::Truncated {
                what,
                offset,
                need,
                have,
            } => write!(
                f,
                "truncated image: {what} needs {need} byte(s) at offset {offset}, \
                 but the file is {have} byte(s)"
            ),
            RomError::TooLargeForHost { what, len } => write!(
                f,
                "{what} is {len} byte(s), which does not fit in this host's address space"
            ),
        }
    }
}

impl From<RomError> for Error {
    fn from(e: RomError) -> Error {
        use alloc::string::ToString;
        Error::Config {
            at: String::from("ines"),
            message: e.to_string(),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RomError {}

// ---------------------------------------------------------------------------
// Header fields
// ---------------------------------------------------------------------------

/// How the four nametables are wired to the console's 2 KiB of CIRAM.
///
/// The cartridge decides this, not the PPU: the console routes one CIRAM
/// address line to the cartridge connector and the board ties it to whichever
/// PPU address line it wants (NESdev, *Cartridge connector*). Everything else
/// about nametable mirroring follows from that one wire.
///
/// The single-screen variants cannot appear in an iNES header — no mapper-0
/// board wires them — but mappers from MMC1 onward select them at run time, so
/// this is the type that has to carry them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mirroring {
    /// CIRAM A10 = PPU A11. `$2000`/`$2400` are one screen, `$2800`/`$2C00` the
    /// other. What a vertically scrolling game wants.
    Horizontal,
    /// CIRAM A10 = PPU A10. `$2000`/`$2800` are one screen, `$2400`/`$2C00` the
    /// other. What a horizontally scrolling game wants.
    Vertical,
    /// All four nametables are the lower 1 KiB of CIRAM.
    SingleScreenLower,
    /// All four nametables are the upper 1 KiB of CIRAM.
    SingleScreenUpper,
    /// The cartridge supplies its own 2 KiB so that all four nametables are
    /// distinct.
    FourScreen,
}

impl Mirroring {
    /// Which 1 KiB bank each of the four nametable slots resolves to.
    ///
    /// Slot `i` is the kilobyte at `$2000 + i * $400`. Banks 0 and 1 are the
    /// console's CIRAM; banks 2 and 3 only occur under
    /// [`Mirroring::FourScreen`] and come from the cartridge's own VRAM.
    ///
    /// This *is* the CIRAM A10 wiring, written out: horizontal ties it to PPU
    /// A11 (`slot >> 1`), vertical to PPU A10 (`slot & 1`).
    #[must_use]
    pub const fn banks(self) -> [u8; 4] {
        match self {
            Mirroring::Horizontal => [0, 0, 1, 1],
            Mirroring::Vertical => [0, 1, 0, 1],
            Mirroring::SingleScreenLower => [0, 0, 0, 0],
            Mirroring::SingleScreenUpper => [1, 1, 1, 1],
            Mirroring::FourScreen => [0, 1, 2, 3],
        }
    }

    /// Whether the cartridge has to supply nametable RAM of its own.
    #[must_use]
    pub const fn needs_cartridge_vram(self) -> bool {
        matches!(self, Mirroring::FourScreen)
    }
}

impl fmt::Display for Mirroring {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Mirroring::Horizontal => "horizontal",
            Mirroring::Vertical => "vertical",
            Mirroring::SingleScreenLower => "single-screen (lower)",
            Mirroring::SingleScreenUpper => "single-screen (upper)",
            Mirroring::FourScreen => "four-screen",
        };
        f.write_str(s)
    }
}

/// Which dialect of the header an image is written in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HeaderFormat {
    /// The original iNES header, plus the fields later added to it.
    Ines,
    /// The same header with byte 7 bits 2-3 set to `0b10` and bytes 8-15
    /// redefined.
    Nes2,
}

impl fmt::Display for HeaderFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            HeaderFormat::Ines => "iNES",
            HeaderFormat::Nes2 => "NES 2.0",
        })
    }
}

/// What machine the cartridge is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConsoleKind {
    /// NES or Famicom.
    Nes,
    /// Vs. System arcade hardware.
    VsSystem,
    /// PlayChoice-10 arcade hardware.
    Playchoice10,
    /// Something named by NES 2.0's extended console type
    /// ([`InesHeader::extended_console`]).
    Extended,
}

impl fmt::Display for ConsoleKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ConsoleKind::Nes => "NES/Famicom",
            ConsoleKind::VsSystem => "Vs. System",
            ConsoleKind::Playchoice10 => "PlayChoice-10",
            ConsoleKind::Extended => "extended console",
        })
    }
}

/// The CPU/PPU timing the cartridge expects.
///
/// A region, not a clock rate: the rates belong to the machine's oscillator
/// forest (`ROADMAP.md` §4.2), and a cartridge only says which one it was
/// written for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimingMode {
    /// RP2C02, 60 Hz.
    Ntsc,
    /// RP2C07, 50 Hz.
    Pal,
    /// The cartridge works on either.
    MultiRegion,
    /// UA6538 ("Dendy") clones.
    Dendy,
}

impl fmt::Display for TimingMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            TimingMode::Ntsc => "NTSC",
            TimingMode::Pal => "PAL",
            TimingMode::MultiRegion => "multi-region",
            TimingMode::Dendy => "Dendy",
        })
    }
}

// ---------------------------------------------------------------------------
// The header
// ---------------------------------------------------------------------------

/// A parsed iNES or NES 2.0 header, with every field already in bytes.
///
/// Sizes are stored in bytes rather than in the header's units so that no
/// caller has to remember which field is counted in 16 KiB and which in 64-byte
/// shift counts. The unit arithmetic happens once, here, where it is tested.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct InesHeader {
    /// Which dialect this header is written in.
    pub format: HeaderFormat,
    /// Mapper number: 8 bits for iNES, 12 for NES 2.0.
    pub mapper: u16,
    /// NES 2.0 submapper, or 0.
    pub submapper: u8,
    /// PRG ROM size in bytes. Never zero — a zero is rejected at parse time.
    pub prg_rom_len: u64,
    /// CHR ROM size in bytes. Zero means the board carries CHR RAM instead.
    pub chr_rom_len: u64,
    /// Volatile cartridge work RAM at `$6000-$7FFF`, in bytes.
    pub prg_ram_len: u64,
    /// Battery-backed cartridge work RAM or EEPROM, in bytes. NES 2.0 only.
    pub prg_nvram_len: u64,
    /// Volatile CHR RAM, in bytes.
    pub chr_ram_len: u64,
    /// Battery-backed CHR RAM, in bytes. NES 2.0 only.
    pub chr_nvram_len: u64,
    /// How the nametables are wired.
    pub mirroring: Mirroring,
    /// Whether the board carries a battery over its work RAM.
    pub battery: bool,
    /// Whether a 512-byte trainer sits between the header and the PRG ROM.
    pub trainer: bool,
    /// What machine this is for.
    pub console: ConsoleKind,
    /// Which region's timing it expects.
    pub timing: TimingMode,
    /// Vs. System PPU type (NES 2.0 byte 13 bits 0-3), or 0.
    pub vs_ppu: u8,
    /// Vs. System hardware type (NES 2.0 byte 13 bits 4-7), or 0.
    pub vs_hardware: u8,
    /// Extended console type (NES 2.0 byte 13 bits 0-3 when
    /// [`ConsoleKind::Extended`]), or 0.
    pub extended_console: u8,
    /// How many miscellaneous ROMs follow the CHR ROM (NES 2.0), or 0.
    pub misc_roms: u8,
    /// Default expansion device (NES 2.0 byte 15 bits 0-5), or 0.
    pub expansion_device: u8,
    /// True when byte 7's high nibble was ignored because bytes 12-15 look like
    /// a ripper's signature rather than header fields.
    ///
    /// Recorded rather than hidden: it changes the mapper number, and a user
    /// staring at "mapper 0" for a file whose byte 7 says `0x40` deserves to be
    /// told why.
    pub archaic: bool,
    /// Offset of the trainer, if [`InesHeader::trainer`] is set.
    pub trainer_offset: u64,
    /// Offset of the PRG ROM in the file.
    pub prg_rom_offset: u64,
    /// Offset of the CHR ROM in the file. Meaningless when
    /// [`InesHeader::chr_rom_len`] is zero.
    pub chr_rom_offset: u64,
    /// Total bytes the header accounts for. A longer file is not an error —
    /// PlayChoice hint screens, NES 2.0 miscellaneous ROMs and title blocks all
    /// live past this point.
    pub image_len: u64,
}

impl InesHeader {
    /// Parse the 16-byte header at the start of `bytes`.
    ///
    /// Only the header is examined; the body is not required to be present.
    /// [`Cartridge::from_ines`] is what checks that.
    ///
    /// # Errors
    ///
    /// [`RomError::HeaderTooShort`] for a file under 16 bytes,
    /// [`RomError::BadMagic`] for one that is not an iNES image,
    /// [`RomError::NoPrgRom`] for a header describing no program ROM, and
    /// [`RomError::SizeOverflow`] for a NES 2.0 exponent size that no `u64` can
    /// express.
    pub fn parse(bytes: &[u8]) -> Result<InesHeader, RomError> {
        let head: [u8; 16] = match bytes.get(..16) {
            Some(s) => match <[u8; 16]>::try_from(s) {
                Ok(h) => h,
                // Unreachable: `get(..16)` yields exactly 16 bytes. Written as a
                // match anyway so this function contains no panicking path at
                // all, which is the property the fuzz sweep is asserting.
                Err(_) => {
                    return Err(RomError::HeaderTooShort {
                        have: bytes.len() as u64,
                    });
                }
            },
            None => {
                return Err(RomError::HeaderTooShort {
                    have: bytes.len() as u64,
                });
            }
        };

        let magic = [head[0], head[1], head[2], head[3]];
        if magic != MAGIC {
            return Err(RomError::BadMagic { found: magic });
        }

        let flags6 = head[6];
        let flags7 = head[7];

        // NES 2.0 identifies itself in byte 7 bits 2-3. Anything else — 0 for a
        // real iNES header, 1 or 3 for a corrupt one — is read as iNES.
        let format = if flags7 & 0x0c == 0x08 {
            HeaderFormat::Nes2
        } else {
            HeaderFormat::Ines
        };
        let nes2 = format == HeaderFormat::Nes2;

        // Rippers of the early 1990s wrote their names across bytes 7-15, which
        // is why byte 7's mapper nibble cannot be trusted on its own. NESdev's
        // iNES page recommends the check this implements: if bytes 12-15 are not
        // all zero, the header predates those fields and only the low mapper
        // nibble is real. NES 2.0 defines all of bytes 12-15, so it is exempt.
        let archaic = !nes2 && head[12..16].iter().any(|&b| b != 0);

        let mut mapper = u16::from(flags6 >> 4);
        if !archaic {
            mapper |= u16::from(flags7 & 0xf0);
        }
        if nes2 {
            mapper |= u16::from(head[8] & 0x0f) << 8;
        }

        let mirroring = if flags6 & 0x08 != 0 {
            // Bit 3 overrides bit 0 entirely: the board supplies its own VRAM
            // and the CIRAM A10 wiring stops mattering.
            Mirroring::FourScreen
        } else if flags6 & 0x01 != 0 {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };

        let (prg_rom_len, chr_rom_len) = if nes2 {
            (
                rom_size(head[4], head[9] & 0x0f, PRG_UNIT, RomPart::PrgRom)?,
                rom_size(head[5], head[9] >> 4, CHR_UNIT, RomPart::ChrRom)?,
            )
        } else {
            // Both products are bounded by 255 * 16384, so neither can overflow.
            (u64::from(head[4]) * PRG_UNIT, u64::from(head[5]) * CHR_UNIT)
        };

        if prg_rom_len == 0 {
            return Err(RomError::NoPrgRom);
        }

        let battery = flags6 & 0x02 != 0;

        let (prg_ram_len, prg_nvram_len, chr_ram_len, chr_nvram_len) = if nes2 {
            (
                shift_size(head[10] & 0x0f),
                shift_size(head[10] >> 4),
                shift_size(head[11] & 0x0f),
                shift_size(head[11] >> 4),
            )
        } else {
            // iNES byte 8 counts 8 KiB units and "0 infers 8 KiB for
            // compatibility", so almost every real file lands on 8 KiB. An
            // archaic header's byte 8 is part of a name, so it gets the default
            // too.
            let units = if archaic {
                1
            } else {
                u64::from(head[8]).max(1)
            };
            let chr_ram = if chr_rom_len == 0 { DEFAULT_8K } else { 0 };
            (units * DEFAULT_8K, 0, chr_ram, 0)
        };

        let console = if archaic {
            ConsoleKind::Nes
        } else {
            match flags7 & 0x03 {
                0 => ConsoleKind::Nes,
                1 => ConsoleKind::VsSystem,
                2 => ConsoleKind::Playchoice10,
                _ => ConsoleKind::Extended,
            }
        };

        let timing = if nes2 {
            match head[12] & 0x03 {
                0 => TimingMode::Ntsc,
                1 => TimingMode::Pal,
                2 => TimingMode::MultiRegion,
                _ => TimingMode::Dendy,
            }
        } else if !archaic && head[9] & 0x01 != 0 {
            TimingMode::Pal
        } else {
            TimingMode::Ntsc
        };

        let (vs_ppu, vs_hardware) = if nes2 && console == ConsoleKind::VsSystem {
            (head[13] & 0x0f, head[13] >> 4)
        } else {
            (0, 0)
        };
        let extended_console = if nes2 && console == ConsoleKind::Extended {
            head[13] & 0x0f
        } else {
            0
        };

        let trainer = flags6 & 0x04 != 0;
        let trainer_offset = HEADER_LEN;
        // At most 16 + 512; no overflow is possible here.
        let prg_rom_offset = HEADER_LEN + if trainer { TRAINER_LEN } else { 0 };
        // These two can overflow, because a NES 2.0 exponent size can be within
        // one doubling of u64::MAX. An overflow means the file cannot possibly
        // contain the body, which is exactly `Truncated`.
        let chr_rom_offset =
            prg_rom_offset
                .checked_add(prg_rom_len)
                .ok_or(RomError::Truncated {
                    what: RomPart::PrgRom,
                    offset: prg_rom_offset,
                    need: prg_rom_len,
                    have: bytes.len() as u64,
                })?;
        let image_len = chr_rom_offset
            .checked_add(chr_rom_len)
            .ok_or(RomError::Truncated {
                what: RomPart::ChrRom,
                offset: chr_rom_offset,
                need: chr_rom_len,
                have: bytes.len() as u64,
            })?;

        Ok(InesHeader {
            format,
            mapper,
            submapper: if nes2 { head[8] >> 4 } else { 0 },
            prg_rom_len,
            chr_rom_len,
            prg_ram_len,
            prg_nvram_len,
            chr_ram_len,
            chr_nvram_len,
            mirroring,
            battery,
            trainer,
            console,
            timing,
            vs_ppu,
            vs_hardware,
            extended_console,
            misc_roms: if nes2 { head[14] & 0x03 } else { 0 },
            expansion_device: if nes2 { head[15] & 0x3f } else { 0 },
            archaic,
            trainer_offset,
            prg_rom_offset,
            chr_rom_offset,
            image_len,
        })
    }

    /// Total cartridge work RAM at `$6000-$7FFF`, volatile plus battery-backed.
    ///
    /// One number because one window decodes it: a board with both an EEPROM
    /// and a static RAM still presents 8 KiB to the CPU.
    #[must_use]
    pub const fn work_ram_len(&self) -> u64 {
        self.prg_ram_len + self.prg_nvram_len
    }

    /// Total CHR RAM, volatile plus battery-backed.
    #[must_use]
    pub const fn chr_ram_total(&self) -> u64 {
        self.chr_ram_len + self.chr_nvram_len
    }
}

/// Decode one NES 2.0 ROM size field.
///
/// `msb == 0xF` switches the LSB byte from a count of units to `EEEEEEMM`, an
/// exponent and a multiplier giving `2^E * (2M + 1)` **bytes** (NESdev,
/// *NES 2.0*). Every other value is a 12-bit count of `unit`-sized blocks.
fn rom_size(lsb: u8, msb: u8, unit: u64, what: RomPart) -> Result<u64, RomError> {
    if msb == 0x0f {
        let exponent = lsb >> 2;
        let multiplier = lsb & 0x03;
        // The exponent is 6 bits, so the shift is always in range; the multiply
        // by up to 7 is what can overflow.
        1u64.checked_shl(u32::from(exponent))
            .and_then(|base| base.checked_mul(u64::from(multiplier) * 2 + 1))
            .ok_or(RomError::SizeOverflow {
                what,
                exponent,
                multiplier,
            })
    } else {
        // At most 0xFFF units of 16 KiB — 64 MiB, nowhere near overflowing.
        Ok(((u64::from(msb) << 8) | u64::from(lsb)) * unit)
    }
}

/// Decode a NES 2.0 RAM shift count: `64 << n` bytes, or none for zero.
const fn shift_size(shift: u8) -> u64 {
    if shift == 0 {
        0
    } else {
        // `shift` is a nibble, so the largest result is 64 << 15 == 2 MiB.
        64u64 << shift
    }
}

// ---------------------------------------------------------------------------
// The cartridge
// ---------------------------------------------------------------------------

/// What is wired to the PPU's pattern-table pins: mask ROM or RAM.
///
/// Not an `Option<Rom>` plus an `Option<Ram>`: a board has one or the other,
/// never both and never neither, and a type that can express "neither" makes
/// every consumer handle a case that no cartridge presents.
#[derive(Debug, Clone)]
pub enum Chr {
    /// Character ROM read from the image.
    Rom(Arc<RomStore>),
    /// Character RAM the board carries; the image supplies no contents.
    Ram(Arc<RamStore>),
}

impl Chr {
    /// Size in bytes.
    #[must_use]
    pub fn len(&self) -> u64 {
        match self {
            Chr::Rom(r) => r.len(),
            Chr::Ram(r) => r.len(),
        }
    }

    /// Whether there is no character memory at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Whether the board carries RAM rather than ROM.
    #[must_use]
    pub const fn is_ram(&self) -> bool {
        matches!(self, Chr::Ram(_))
    }

    /// The RAM store, if this is CHR RAM.
    #[must_use]
    pub const fn as_ram(&self) -> Option<&Arc<RamStore>> {
        match self {
            Chr::Ram(r) => Some(r),
            Chr::Rom(_) => None,
        }
    }

    /// The ROM store, if this is CHR ROM.
    #[must_use]
    pub const fn as_rom(&self) -> Option<&Arc<RomStore>> {
        match self {
            Chr::Rom(r) => Some(r),
            Chr::Ram(_) => None,
        }
    }
}

/// A cartridge: its header, its ROMs, and the RAM its board carries.
///
/// This is the *image* half of the split described in the
/// [module docs](super) — everything a mapper needs to know, with no opinion
/// about how any of it is decoded. [`Nrom`](super::nrom::Nrom) and its
/// successors take one of these and turn it into address-space regions.
///
/// The stores are `Arc`-shared rather than owned so that a mapper can hand the
/// same PRG ROM to several windows without copying it, which is the whole point
/// of bank switching.
#[derive(Debug, Clone)]
pub struct Cartridge {
    header: InesHeader,
    prg_rom: Arc<RomStore>,
    chr: Chr,
    work_ram: Option<Arc<RamStore>>,
    trainer: Option<Arc<Vec<u8>>>,
}

impl Cartridge {
    /// Parse an iNES or NES 2.0 image.
    ///
    /// Bytes past what the header accounts for are ignored: PlayChoice hint
    /// screens, NES 2.0 miscellaneous ROMs and appended title blocks are all
    /// legal and none of them are cartridge state.
    ///
    /// # Errors
    ///
    /// Everything [`InesHeader::parse`] rejects, plus [`RomError::Truncated`]
    /// when the file is shorter than the header's own layout and
    /// [`RomError::TooLargeForHost`] when a size fits in a `u64` but not in this
    /// host's `usize`.
    pub fn from_ines(bytes: &[u8]) -> Result<Cartridge, RomError> {
        let header = InesHeader::parse(bytes)?;

        let trainer = if header.trainer {
            let slice = slice_of(bytes, header.trainer_offset, TRAINER_LEN, RomPart::Trainer)?;
            Some(Arc::new(slice.to_vec()))
        } else {
            None
        };

        let prg = slice_of(
            bytes,
            header.prg_rom_offset,
            header.prg_rom_len,
            RomPart::PrgRom,
        )?;
        let prg_rom = Arc::new(RomStore::new(prg.to_vec()));

        let chr = if header.chr_rom_len > 0 {
            let bytes = slice_of(
                bytes,
                header.chr_rom_offset,
                header.chr_rom_len,
                RomPart::ChrRom,
            )?;
            Chr::Rom(Arc::new(RomStore::new(bytes.to_vec())))
        } else {
            // Bounded by two nibble shift counts — at most 4 MiB — so this
            // allocation cannot be driven anywhere dangerous by the header.
            Chr::Ram(Arc::new(RamStore::new(header.chr_ram_total())))
        };

        let work_ram = match header.work_ram_len() {
            0 => None,
            len => Some(Arc::new(RamStore::new(len))),
        };

        Ok(Cartridge {
            header,
            prg_rom,
            chr,
            work_ram,
            trainer,
        })
    }

    /// The parsed header.
    #[must_use]
    pub const fn header(&self) -> &InesHeader {
        &self.header
    }

    /// The mapper number the header names.
    #[must_use]
    pub const fn mapper(&self) -> u16 {
        self.header.mapper
    }

    /// How the nametables are wired.
    #[must_use]
    pub const fn mirroring(&self) -> Mirroring {
        self.header.mirroring
    }

    /// Whether the board's work RAM is battery-backed.
    #[must_use]
    pub const fn battery(&self) -> bool {
        self.header.battery
    }

    /// Program ROM.
    #[must_use]
    pub const fn prg_rom(&self) -> &Arc<RomStore> {
        &self.prg_rom
    }

    /// Character ROM or RAM.
    #[must_use]
    pub const fn chr(&self) -> &Chr {
        &self.chr
    }

    /// Cartridge work RAM at `$6000-$7FFF`, if the board has any.
    #[must_use]
    pub const fn work_ram(&self) -> Option<&Arc<RamStore>> {
        self.work_ram.as_ref()
    }

    /// The 512-byte trainer, if the image carries one.
    ///
    /// Kept rather than dropped because it is part of the image, but not
    /// mapped: a trainer is a patch some copier expected the *loader* to place
    /// at `$7000`, not something the board decodes. Whoever emulates that
    /// loader can ask for it.
    #[must_use]
    pub fn trainer(&self) -> Option<&[u8]> {
        self.trainer.as_ref().map(|t| t.as_slice())
    }
}

/// Borrow `len` bytes at `offset`, or say precisely why that is impossible.
fn slice_of(bytes: &[u8], offset: u64, len: u64, what: RomPart) -> Result<&[u8], RomError> {
    let have = bytes.len() as u64;
    let truncated = || RomError::Truncated {
        what,
        offset,
        need: len,
        have,
    };
    let end = offset.checked_add(len).ok_or_else(truncated)?;
    if end > have {
        return Err(truncated());
    }
    // Both fit in a usize because they are bounded by `bytes.len()`, but the
    // conversion is checked rather than cast: this function's contract is that
    // it cannot panic.
    let start = usize::try_from(offset).map_err(|_| RomError::TooLargeForHost { what, len })?;
    let end = usize::try_from(end).map_err(|_| RomError::TooLargeForHost { what, len })?;
    bytes.get(start..end).ok_or_else(truncated)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// Build an image with an iNES 1.0 header.
    fn ines1(prg_units: u8, chr_units: u8, flags6: u8, flags7: u8) -> Vec<u8> {
        let mut v = vec![0u8; 16];
        v[..4].copy_from_slice(&MAGIC);
        v[4] = prg_units;
        v[5] = chr_units;
        v[6] = flags6;
        v[7] = flags7;
        if flags6 & 0x04 != 0 {
            v.extend(core::iter::repeat_n(0xaa, TRAINER_LEN as usize));
        }
        v.extend(core::iter::repeat_n(0x5a, usize::from(prg_units) * 16384));
        v.extend(core::iter::repeat_n(0xa5, usize::from(chr_units) * 8192));
        v
    }

    /// The rejection a malformed image produces.
    ///
    /// `Cartridge` holds `Arc`s and is deliberately not `PartialEq`, so the
    /// rejection tests compare the error rather than the whole `Result`.
    fn reject(bytes: &[u8]) -> RomError {
        Cartridge::from_ines(bytes).expect_err("must be rejected")
    }

    /// Build a bare 16-byte NES 2.0 header; the caller appends a body.
    fn nes2_header(bytes: [u8; 16]) -> [u8; 16] {
        let mut h = bytes;
        h[..4].copy_from_slice(&MAGIC);
        h[7] = (h[7] & 0xf3) | 0x08;
        h
    }

    #[test]
    fn a_minimal_ines1_image_parses() {
        let img = ines1(1, 1, 0, 0);
        let cart = Cartridge::from_ines(&img).expect("valid image");
        let h = cart.header();
        assert_eq!(h.format, HeaderFormat::Ines);
        assert_eq!(h.mapper, 0);
        assert_eq!(h.prg_rom_len, 16384);
        assert_eq!(h.chr_rom_len, 8192);
        assert_eq!(h.mirroring, Mirroring::Horizontal);
        assert!(!h.battery);
        assert!(!h.trainer);
        assert_eq!(h.console, ConsoleKind::Nes);
        assert_eq!(h.timing, TimingMode::Ntsc);
        assert_eq!(h.image_len, 16 + 16384 + 8192);
        // iNES byte 8 of zero means 8 KiB, not "no RAM".
        assert_eq!(h.work_ram_len(), 8192);
        assert_eq!(cart.chr().len(), 8192);
        assert!(!cart.chr().is_ram());
    }

    #[test]
    fn chr_size_zero_means_the_board_carries_chr_ram() {
        let img = ines1(1, 0, 0, 0);
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert_eq!(cart.header().chr_rom_len, 0);
        assert_eq!(cart.header().chr_ram_total(), 8192);
        assert!(cart.chr().is_ram());
        assert_eq!(cart.chr().len(), 8192);
    }

    #[test]
    fn the_mapper_number_comes_from_both_nibbles() {
        // Low nibble in flags 6 bits 4-7, high nibble in flags 7 bits 4-7.
        let img = ines1(1, 0, 0xa0, 0xb0);
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert_eq!(cart.mapper(), 0xba);
        assert!(!cart.header().archaic);
    }

    #[test]
    fn an_archaic_header_ignores_byte_sevens_mapper_nibble() {
        // A ripper's name across bytes 12-15 is the documented tell.
        let mut img = ines1(1, 0, 0xa0, 0x40);
        img[12] = b'D';
        img[13] = b'i';
        img[14] = b'z';
        img[15] = b'!';
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert!(cart.header().archaic);
        assert_eq!(cart.mapper(), 0x0a, "byte 7 must not contribute");
        // ...and byte 8 is part of the name too, so the RAM size falls back.
        assert_eq!(cart.header().work_ram_len(), 8192);
    }

    #[test]
    fn battery_and_trainer_flags_move_the_prg_offset() {
        let img = ines1(1, 0, 0x02 | 0x04, 0);
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert!(cart.battery());
        assert!(cart.header().trainer);
        assert_eq!(cart.header().prg_rom_offset, 16 + 512);
        assert_eq!(cart.trainer().map(<[u8]>::len), Some(512));
        assert_eq!(cart.trainer().and_then(|t| t.first()).copied(), Some(0xaa));
        // The PRG ROM has to be the bytes *after* the trainer, not through it.
        let mut prg = [0u8; 4];
        cart.prg_rom().read_at(0, &mut prg).expect("in range");
        assert_eq!(prg, [0x5a; 4]);
    }

    #[test]
    fn the_four_mirroring_arrangements() {
        for (flags6, want) in [
            (0x00, Mirroring::Horizontal),
            (0x01, Mirroring::Vertical),
            (0x08, Mirroring::FourScreen),
            // Bit 3 wins over bit 0: the board supplies its own VRAM, so the
            // CIRAM A10 wiring has nothing left to decide.
            (0x09, Mirroring::FourScreen),
        ] {
            let img = ines1(1, 0, flags6, 0);
            let cart = Cartridge::from_ines(&img).expect("valid image");
            assert_eq!(cart.mirroring(), want, "flags6 = {flags6:#04x}");
        }
    }

    #[test]
    fn mirroring_banks_are_the_ciram_a10_wiring() {
        assert_eq!(Mirroring::Horizontal.banks(), [0, 0, 1, 1]);
        assert_eq!(Mirroring::Vertical.banks(), [0, 1, 0, 1]);
        assert_eq!(Mirroring::SingleScreenLower.banks(), [0, 0, 0, 0]);
        assert_eq!(Mirroring::SingleScreenUpper.banks(), [1, 1, 1, 1]);
        assert_eq!(Mirroring::FourScreen.banks(), [0, 1, 2, 3]);
        assert!(Mirroring::FourScreen.needs_cartridge_vram());
        assert!(!Mirroring::Vertical.needs_cartridge_vram());
    }

    #[test]
    fn ines1_reports_pal_from_byte_nine() {
        let mut img = ines1(1, 0, 0, 0);
        img[9] = 0x01;
        assert_eq!(
            Cartridge::from_ines(&img).expect("valid").header().timing,
            TimingMode::Pal
        );
    }

    #[test]
    fn nes2_is_detected_and_widens_the_mapper() {
        let mut h = nes2_header([0; 16]);
        h[4] = 2; // 32 KiB PRG
        h[6] = 0x10; // mapper bits 0-3 = 1
        h[7] |= 0x20; // mapper bits 4-7 = 2
        h[8] = 0x53; // mapper bits 8-11 = 3, submapper = 5
        let mut img = h.to_vec();
        img.extend(core::iter::repeat_n(0u8, 32768));
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert_eq!(cart.header().format, HeaderFormat::Nes2);
        assert_eq!(cart.mapper(), 0x321);
        assert_eq!(cart.header().submapper, 5);
        assert!(!cart.header().archaic, "NES 2.0 defines bytes 12-15");
    }

    #[test]
    fn nes2_takes_the_high_size_bits_from_byte_nine() {
        let mut h = nes2_header([0; 16]);
        h[4] = 0x02;
        h[5] = 0x01;
        h[9] = 0x00;
        let mut img = h.to_vec();
        img.extend(core::iter::repeat_n(0u8, 32768 + 8192));
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert_eq!(cart.header().prg_rom_len, 32768);
        assert_eq!(cart.header().chr_rom_len, 8192);

        // Now with a non-zero MSB nibble the counts become 12-bit.
        let mut h = nes2_header([0; 16]);
        h[4] = 0x00;
        h[9] = 0x01; // PRG MSB = 1 -> 0x100 units of 16 KiB = 4 MiB
        let header = InesHeader::parse(&h).expect("header alone parses");
        assert_eq!(header.prg_rom_len, 0x100 * 16384);
    }

    #[test]
    fn nes2_exponent_sizes_decode() {
        // MSB nibble 0xF switches the LSB byte to EEEEEEMM: 2^E * (2M + 1).
        let mut h = nes2_header([0; 16]);
        h[4] = (10 << 2) | 1; // 2^10 * 3 = 3072 bytes
        h[9] = 0x0f;
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.prg_rom_len, 3072);

        let mut h = nes2_header([0; 16]);
        h[4] = 1; // one 16 KiB unit of PRG, so the header is otherwise sane
        h[5] = 12 << 2; // exponent 12, multiplier code 0: 2^12 * 1 = 4096 bytes of CHR
        h[9] = 0xf0;
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.chr_rom_len, 4096);
    }

    #[test]
    fn an_exponent_size_that_overflows_is_rejected() {
        let mut h = nes2_header([0; 16]);
        h[4] = (63 << 2) | 3; // 2^63 * 7
        h[9] = 0x0f;
        assert_eq!(
            InesHeader::parse(&h),
            Err(RomError::SizeOverflow {
                what: RomPart::PrgRom,
                exponent: 63,
                multiplier: 3,
            })
        );
    }

    #[test]
    fn nes2_ram_shift_counts_decode() {
        let mut h = nes2_header([0; 16]);
        h[4] = 1;
        h[10] = 0x70; // no volatile PRG RAM, 64 << 7 = 8 KiB of EEPROM
        h[11] = 0x07; // 8 KiB of CHR RAM, no CHR NVRAM
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.prg_ram_len, 0);
        assert_eq!(header.prg_nvram_len, 8192);
        assert_eq!(header.work_ram_len(), 8192);
        assert_eq!(header.chr_ram_len, 8192);
        assert_eq!(header.chr_nvram_len, 0);

        // Shift count 0 means none, not 64 bytes.
        let mut h = nes2_header([0; 16]);
        h[4] = 1;
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.work_ram_len(), 0);
        assert_eq!(header.chr_ram_total(), 0);
    }

    #[test]
    fn nes2_extended_fields_decode() {
        let mut h = nes2_header([0; 16]);
        h[4] = 1;
        h[7] |= 0x01; // Vs. System
        h[12] = 0x03; // Dendy
        h[13] = 0x42; // Vs. hardware 4, Vs. PPU 2
        h[14] = 0x02; // two miscellaneous ROMs
        h[15] = 0x2f; // expansion device
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.console, ConsoleKind::VsSystem);
        assert_eq!(header.timing, TimingMode::Dendy);
        assert_eq!(header.vs_ppu, 2);
        assert_eq!(header.vs_hardware, 4);
        assert_eq!(header.extended_console, 0);
        assert_eq!(header.misc_roms, 2);
        assert_eq!(header.expansion_device, 0x2f);

        let mut h = nes2_header([0; 16]);
        h[4] = 1;
        h[7] |= 0x03; // extended console type
        h[13] = 0x09;
        let header = InesHeader::parse(&h).expect("valid header");
        assert_eq!(header.console, ConsoleKind::Extended);
        assert_eq!(header.extended_console, 9);
        assert_eq!(header.vs_ppu, 0);
    }

    // -- rejection paths ---------------------------------------------------

    #[test]
    fn a_file_shorter_than_the_header_is_rejected_at_every_length() {
        let img = ines1(1, 0, 0, 0);
        for n in 0..16 {
            assert_eq!(
                reject(&img[..n]),
                RomError::HeaderTooShort { have: n as u64 },
                "length {n}"
            );
        }
    }

    #[test]
    fn bad_magic_is_rejected() {
        let mut img = ines1(1, 0, 0, 0);
        img[3] = 0x1b;
        assert_eq!(
            reject(&img),
            RomError::BadMagic {
                found: [b'N', b'E', b'S', 0x1b]
            }
        );
        // And an empty-but-long file, which is the common "wrong file" case.
        let zeros = vec![0u8; 64];
        assert!(matches!(reject(&zeros), RomError::BadMagic { .. }));
    }

    #[test]
    fn a_header_with_no_prg_rom_is_rejected() {
        let img = ines1(0, 1, 0, 0);
        assert_eq!(reject(&img), RomError::NoPrgRom);
    }

    #[test]
    fn a_truncated_body_is_rejected_with_the_numbers() {
        let img = ines1(2, 1, 0, 0);
        let short = &img[..16 + 32768 + 4096];
        assert_eq!(
            reject(short),
            RomError::Truncated {
                what: RomPart::ChrRom,
                offset: 16 + 32768,
                need: 8192,
                have: 16 + 32768 + 4096,
            }
        );

        let short = &img[..16 + 1000];
        assert_eq!(
            reject(short),
            RomError::Truncated {
                what: RomPart::PrgRom,
                offset: 16,
                need: 32768,
                have: 16 + 1000,
            }
        );
    }

    #[test]
    fn a_missing_trainer_is_named_as_the_missing_part() {
        let img = ines1(1, 0, 0x04, 0);
        let short = &img[..16 + 100];
        assert_eq!(
            reject(short),
            RomError::Truncated {
                what: RomPart::Trainer,
                offset: 16,
                need: 512,
                have: 16 + 100,
            }
        );
    }

    #[test]
    fn an_enormous_nes2_size_is_truncated_not_an_allocation() {
        // 0xFFF units of 16 KiB is 64 MiB the file does not have. The point of
        // the test is that nothing tries to allocate it first.
        let mut h = nes2_header([0; 16]);
        h[4] = 0xff;
        h[9] = 0x0e; // MSB nibble 0xE, which is a count and not the exponent escape
        let err = Cartridge::from_ines(&h).expect_err("cannot fit");
        assert!(matches!(
            err,
            RomError::Truncated {
                what: RomPart::PrgRom,
                ..
            }
        ));
    }

    #[test]
    fn trailing_bytes_are_allowed() {
        // PlayChoice hint screens, NES 2.0 misc ROMs and title blocks all sit
        // past the end of what the header accounts for.
        let mut img = ines1(1, 1, 0, 0);
        let accounted = img.len() as u64;
        img.extend(core::iter::repeat_n(0xcc, 8192));
        let cart = Cartridge::from_ines(&img).expect("valid image");
        assert_eq!(cart.header().image_len, accounted);
    }

    #[test]
    fn errors_say_something_useful() {
        let text = alloc::format!("{}", RomError::HeaderTooShort { have: 3 });
        assert!(text.contains('3'), "{text}");
        let text = alloc::format!(
            "{}",
            RomError::Truncated {
                what: RomPart::ChrRom,
                offset: 16,
                need: 8192,
                have: 20
            }
        );
        assert!(text.contains("CHR ROM"), "{text}");
        assert!(text.contains("8192"), "{text}");
        // ...and it survives the trip into the crate-level error type.
        let e: Error = RomError::NoPrgRom.into();
        assert!(alloc::format!("{e}").contains("PRG ROM"));
    }

    // -- fuzz-shaped robustness -------------------------------------------

    #[test]
    fn no_truncation_of_a_good_image_panics() {
        for flags6 in [0x00u8, 0x01, 0x04, 0x08, 0x0f] {
            let img = ines1(2, 1, flags6, 0);
            for n in 0..=img.len() {
                // The result does not matter; not panicking does.
                let _ = Cartridge::from_ines(&img[..n]);
            }
        }
    }

    #[test]
    fn no_header_bit_pattern_panics() {
        // Every single-byte mutation of every header byte, over a body long
        // enough that some of them are satisfiable and most are not.
        let base = ines1(1, 1, 0, 0);
        for byte in 0..16usize {
            for value in 0..=255u8 {
                let mut img = base.clone();
                img[byte] = value;
                let _ = Cartridge::from_ines(&img);
                let _ = InesHeader::parse(&img);
            }
        }
    }

    #[test]
    fn garbage_does_not_panic() {
        // A deterministic xorshift, so a failure is reproducible — CLAUDE.md
        // forbids run-dependent behaviour even in a test.
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for len in [0usize, 1, 4, 15, 16, 17, 528, 529, 1024] {
            for _ in 0..64 {
                let mut buf = vec![0u8; len];
                for b in &mut buf {
                    *b = next() as u8;
                }
                let _ = Cartridge::from_ines(&buf);
                // Half the time, make the magic valid so the rest of the parser
                // is actually reached rather than short-circuited.
                if buf.len() >= 4 {
                    buf[..4].copy_from_slice(&MAGIC);
                    let _ = Cartridge::from_ines(&buf);
                }
            }
        }
    }

    // -- a real ROM --------------------------------------------------------

    /// Parse a real cartridge, when one is available.
    ///
    /// `AccuracyCoin` is MIT (© 2025 Chris Siebert) and *may* be redistributed,
    /// but `CLAUDE.md` keeps conformance corpora out of the repository and
    /// behind an environment variable regardless — the rule is about size and
    /// reproducibility as much as licensing. Point `RSEMU_NES_TEST_ROM` at an
    /// image to run this; without it the test passes trivially, so
    /// `cargo test` never needs a download.
    #[cfg(feature = "std")]
    #[test]
    fn a_real_cartridge_parses() {
        let Ok(path) = std::env::var("RSEMU_NES_TEST_ROM") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("RSEMU_NES_TEST_ROM is readable");
        let cart = Cartridge::from_ines(&bytes).expect("a real image parses");
        let h = cart.header();
        assert_eq!(h.format, HeaderFormat::Ines);
        assert_eq!(h.mapper, 0, "AccuracyCoin is an NROM cart");
        assert_eq!(h.prg_rom_len, 32768);
        assert_eq!(h.chr_rom_len, 8192);
        assert_eq!(h.mirroring, Mirroring::Vertical);
        assert!(!h.battery);
        assert!(!h.trainer);
        assert_eq!(h.image_len, 16 + 32768 + 8192);
        assert_eq!(h.image_len, bytes.len() as u64);
    }
}
