//! Register numbers, bit definitions, and the PPU's I/O latch (open bus).
//!
//! Sources: [NESdev PPU registers](https://www.nesdev.org/wiki/PPU_registers)
//! for every bit meaning and for the latch, and
//! [NESdev PPU frame timing](https://www.nesdev.org/wiki/PPU_frame_timing) for
//! the timing constants that appear alongside them.

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};

// ---------------------------------------------------------------------------
// Register numbering
// ---------------------------------------------------------------------------

/// `$2000` PPUCTRL — write only.
pub const PPUCTRL: u8 = 0;
/// `$2001` PPUMASK — write only.
pub const PPUMASK: u8 = 1;
/// `$2002` PPUSTATUS — read only.
pub const PPUSTATUS: u8 = 2;
/// `$2003` OAMADDR — write only.
pub const OAMADDR: u8 = 3;
/// `$2004` OAMDATA — read/write.
pub const OAMDATA: u8 = 4;
/// `$2005` PPUSCROLL — write only, twice.
pub const PPUSCROLL: u8 = 5;
/// `$2006` PPUADDR — write only, twice.
pub const PPUADDR: u8 = 6;
/// `$2007` PPUDATA — read/write.
pub const PPUDATA: u8 = 7;

// ---------------------------------------------------------------------------
// PPUCTRL ($2000)
// ---------------------------------------------------------------------------

/// Base nametable select, bits 0-1: also `t` bits 10-11.
pub const CTRL_NAMETABLE: u8 = 0b0000_0011;
/// VRAM address increment per `$2007` access: clear = 1, set = 32.
pub const CTRL_INCREMENT: u8 = 0b0000_0100;
/// Sprite pattern table for 8x8 sprites: clear = `$0000`, set = `$1000`.
pub const CTRL_SPRITE_TABLE: u8 = 0b0000_1000;
/// Background pattern table: clear = `$0000`, set = `$1000`.
pub const CTRL_BG_TABLE: u8 = 0b0001_0000;
/// Sprite height: clear = 8x8, set = 8x16.
pub const CTRL_SPRITE_16: u8 = 0b0010_0000;
/// Master/slave select — drives the EXT pins, which nothing on a stock NES uses.
pub const CTRL_MASTER: u8 = 0b0100_0000;
/// Generate an NMI at the start of vertical blanking.
pub const CTRL_NMI: u8 = 0b1000_0000;

// ---------------------------------------------------------------------------
// PPUMASK ($2001)
// ---------------------------------------------------------------------------

/// Greyscale: the palette index is ANDed with `$30` on the way out.
pub const MASK_GREYSCALE: u8 = 0b0000_0001;
/// Show the background in the leftmost 8 pixels.
pub const MASK_BG_LEFT: u8 = 0b0000_0010;
/// Show sprites in the leftmost 8 pixels.
pub const MASK_SPRITE_LEFT: u8 = 0b0000_0100;
/// Enable background rendering.
pub const MASK_BG: u8 = 0b0000_1000;
/// Enable sprite rendering.
pub const MASK_SPRITE: u8 = 0b0001_0000;
/// Emphasize red (green on PAL/Dendy).
pub const MASK_EMPHASIS_R: u8 = 0b0010_0000;
/// Emphasize green (red on PAL/Dendy).
pub const MASK_EMPHASIS_G: u8 = 0b0100_0000;
/// Emphasize blue.
pub const MASK_EMPHASIS_B: u8 = 0b1000_0000;
/// Either rendering enable: the PPU is "rendering" when either is set.
pub const MASK_RENDERING: u8 = MASK_BG | MASK_SPRITE;

// ---------------------------------------------------------------------------
// PPUSTATUS ($2002)
// ---------------------------------------------------------------------------

/// Sprite overflow — set by the buggy evaluation of a ninth sprite.
pub const STATUS_OVERFLOW: u8 = 0b0010_0000;
/// Sprite 0 hit.
pub const STATUS_SPRITE0: u8 = 0b0100_0000;
/// Vertical blank has started.
pub const STATUS_VBLANK: u8 = 0b1000_0000;
/// The three bits `$2002` actually drives; bits 4-0 are open bus.
pub const STATUS_DRIVEN: u8 = STATUS_OVERFLOW | STATUS_SPRITE0 | STATUS_VBLANK;

// ---------------------------------------------------------------------------
// OAM attribute byte (byte 2 of a sprite)
// ---------------------------------------------------------------------------

/// Sprite palette select, bits 0-1.
pub const SPRITE_PALETTE: u8 = 0b0000_0011;
/// Priority: set means the sprite is drawn behind opaque background pixels.
pub const SPRITE_BEHIND: u8 = 0b0010_0000;
/// Flip horizontally.
pub const SPRITE_FLIP_X: u8 = 0b0100_0000;
/// Flip vertically.
pub const SPRITE_FLIP_Y: u8 = 0b1000_0000;
/// Bits 2-4 are unimplemented in OAM and always read back as 0
/// ([NESdev PPU OAM](https://www.nesdev.org/wiki/PPU_OAM)); this is the mask of
/// the bits that do exist.
pub const SPRITE_ATTR_IMPLEMENTED: u8 = 0b1110_0011;

// ---------------------------------------------------------------------------
// The I/O latch (open bus)
// ---------------------------------------------------------------------------

/// The PPU's 8-bit dynamic I/O latch, one decay deadline per bit.
///
/// Every read from and write to any `$2000`-`$2007` port drives this latch, and
/// reading a bit the PPU does not drive (all of `$2000`, `$2001`, `$2003`,
/// `$2005`, `$2006`; bits 4-0 of `$2002`; bits 7-6 of a palette read through
/// `$2007`) returns what the latch still holds. It is a capacitance on long
/// board traces rather than a register, so bits decay back to 0 independently —
/// see [NESdev PPU registers](https://www.nesdev.org/wiki/PPU_registers), "PPU
/// I/O latch / open bus".
///
/// Decay is keyed on the PPU dot counter rather than on host time, because a
/// value that depends on how fast the emulator ran is a determinism bug
/// (`CLAUDE.md`, "Determinism").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoLatch {
    value: u8,
    /// Dot at which each bit was last refreshed.
    refreshed: [u64; 8],
    /// How many dots a bit holds its charge.
    ttl: u64,
}

impl IoLatch {
    /// A cleared latch that decays after `ttl` dots.
    pub const fn new(ttl: u64) -> IoLatch {
        IoLatch {
            value: 0,
            refreshed: [0; 8],
            ttl,
        }
    }

    /// The decay interval, in PPU dots.
    pub const fn ttl(&self) -> u64 {
        self.ttl
    }

    /// The latch contents at dot `now`, with decayed bits read back as 0.
    ///
    /// Takes `&mut self` because a decayed bit is gone: folding it away here
    /// keeps [`IoLatch::value`] and what a guest observes from disagreeing.
    pub fn read(&mut self, now: u64) -> u8 {
        for bit in 0..8 {
            if now.saturating_sub(self.refreshed[bit]) > self.ttl {
                self.value &= !(1u8 << bit);
            }
        }
        self.value
    }

    /// The latch contents at dot `now` without folding the decay away.
    ///
    /// Same answer as [`IoLatch::read`], no mutation — what a debugger read
    /// needs (`ROADMAP.md` §15, invariant 5).
    pub const fn peek(&self, now: u64) -> u8 {
        let mut value = self.value;
        let mut bit = 0;
        while bit < 8 {
            if now.saturating_sub(self.refreshed[bit]) > self.ttl {
                value &= !(1u8 << bit);
            }
            bit += 1;
        }
        value
    }

    /// The stored charge without applying decay — for snapshots and tests.
    pub const fn value(&self) -> u8 {
        self.value
    }

    /// Drive the bits set in `mask` to the matching bits of `value`, refreshing
    /// their charge; bits outside `mask` keep both their value and their age.
    pub fn refresh(&mut self, now: u64, value: u8, mask: u8) {
        self.value = (self.value & !mask) | (value & mask);
        for bit in 0..8 {
            if mask & (1u8 << bit) != 0 {
                self.refreshed[bit] = now;
            }
        }
    }

    /// Serialize the latch, decay deadlines included.
    ///
    /// The deadlines are architectural, not derived: restoring a snapshot must
    /// not hand the guest a freshly charged latch it had not earned.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u8(self.value)?;
        w.write_u64(self.ttl)?;
        for age in self.refreshed {
            w.write_u64(age)?;
        }
        Ok(())
    }

    /// Restore what [`IoLatch::save`] wrote.
    pub fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.value = r.read_u8()?;
        self.ttl = r.read_u64()?;
        for age in &mut self.refreshed {
            *age = r.read_u64()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refreshed_bit_survives_and_an_old_one_does_not() {
        let mut latch = IoLatch::new(100);
        latch.refresh(0, 0xff, 0xff);
        assert_eq!(latch.read(100), 0xff);
        // Only bit 0 is driven again, so at dot 201 it is the sole survivor.
        latch.refresh(150, 0x01, 0x01);
        assert_eq!(latch.read(201), 0x01);
    }

    #[test]
    fn masked_bits_keep_their_own_age() {
        let mut latch = IoLatch::new(10);
        latch.refresh(0, 0xf0, 0xf0);
        latch.refresh(9, 0x0f, 0x0f);
        // At dot 12 the high nibble is 12 dots old and the low nibble 3.
        assert_eq!(latch.read(12), 0x0f);
    }
}
