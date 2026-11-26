//! `$2100`-`$213F`: the register file and its access latches.
//!
//! Sources, register by register: nocash fullsnes ("SNES PPU Registers",
//! "SNES PPU Timers and Status") and the SNESdev wiki's *PPU registers* page.
//! The two-write rules are the wiki's:
//!
//! ```text
//!   BGnHOFS  HOFS = (value << 8) | (bgofs_latch & !7) | ((HOFS >> 8) & 7); bgofs_latch = value
//!   BGnVOFS  VOFS = (value << 8) | bgofs_latch;                            bgofs_latch = value
//!   M7x      M7x  = (value << 8) | m7_latch;                               m7_latch    = value
//! ```
//!
//! One `bgofs_latch` is shared by all eight BG scroll registers and one
//! `m7_latch` by `$211B`-`$2120` **and** the mode-7 halves of `$210D`/`$210E`,
//! which write both their BG1 scroll and the mode-7 scroll at once.

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter};

use super::engine::{Engine, Field};

/// The registers whose reads answer from PPU1's own data-bus latch rather
/// than leaving the S-CPU's bus floating: `$21x4`-`$21x6` and `$21x8`-`$21xA`
/// for x = 0, 1, 2 (fullsnes, "SNES PPU Registers", the open-bus note).
pub const PPU1_OPEN_BUS: [u8; 18] = [
    0x04, 0x05, 0x06, 0x08, 0x09, 0x0A, 0x14, 0x15, 0x16, 0x18, 0x19, 0x1A, 0x24, 0x25, 0x26, 0x28,
    0x29, 0x2A,
];

/// `$2134`: the multiplier result's low byte.
pub const MPYL: u8 = 0x34;
/// `$2137`: the software H/V latch.
pub const SLHV: u8 = 0x37;
/// `$2138`: OAM data read.
pub const RDOAM: u8 = 0x38;
/// `$2139`: VRAM data read, low byte.
pub const RDVRAML: u8 = 0x39;
/// `$213A`: VRAM data read, high byte.
pub const RDVRAMH: u8 = 0x3A;
/// `$213B`: CGRAM data read.
pub const RDCGRAM: u8 = 0x3B;
/// `$213C`: the latched H counter.
pub const OPHCT: u8 = 0x3C;
/// `$213D`: the latched V counter.
pub const OPVCT: u8 = 0x3D;
/// `$213E`: PPU1 status and version.
pub const STAT77: u8 = 0x3E;
/// `$213F`: PPU2 status and version.
pub const STAT78: u8 = 0x3F;

macro_rules! regs {
    ($($(#[$doc:meta])* $name:ident: $t:ty,)*) => {
        /// The register file and every access latch behind it.
        ///
        /// Plain data. Listed once, here, so the snapshot's save and load
        /// cannot disagree about a field.
        #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
        pub struct Regs {
            $($(#[$doc])* pub $name: $t,)*
        }

        impl Regs {
            pub(super) fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
                $(self.$name.put(w)?;)*
                Ok(())
            }

            pub(super) fn load(r: &mut ChunkReader<'_>) -> Result<Regs> {
                Ok(Regs { $($name: <$t as Field>::get(r)?,)* })
            }
        }
    };
}

regs! {
    /// `$2100`: forced blank (bit 7) and brightness.
    inidisp: u8,
    /// `$2101`: sprite sizes and name bases.
    obsel: u8,
    /// `$2102`/`$2103`: the OAM *word* address software last wrote, 9 bits.
    oam_reload: u16,
    /// `$2103` bit 7: OBJ priority rotation.
    oam_priority: bool,
    /// The internal OAM *byte* address, 10 bits.
    oam_addr: u16,
    /// The low byte an even OAM write latched.
    oam_latch: u8,
    /// `$2105`.
    bgmode: u8,
    /// `$2106`.
    mosaic: u8,
    /// `$2107`-`$210A`.
    bgsc: [u8; 4],
    /// `$210B`/`$210C`.
    bgnba: [u8; 2],
    /// BG1-4 horizontal scroll, 10 bits.
    hofs: [u16; 4],
    /// BG1-4 vertical scroll, 10 bits.
    vofs: [u16; 4],
    /// The BG scroll registers' shared previous-write latch.
    bgofs_latch: u8,
    /// The mode-7 registers' shared previous-write latch.
    m7_latch: u8,
    /// Mode-7 horizontal scroll, 13-bit signed.
    m7hofs: u16,
    /// Mode-7 vertical scroll, 13-bit signed.
    m7vofs: u16,
    /// `$2115`.
    vmain: u8,
    /// `$2116`/`$2117`: the VRAM word address.
    vmaddr: u16,
    /// The VRAM read prefetch.
    vram_prefetch: u16,
    /// `$211A`.
    m7sel: u8,
    /// `$211B`-`$211E`: M7A-M7D, 16-bit signed.
    m7: [u16; 4],
    /// `$211F`: the mode-7 centre X, 13-bit signed.
    m7x: u16,
    /// `$2120`: the mode-7 centre Y, 13-bit signed.
    m7y: u16,
    /// `$2121`: the CGRAM word address.
    cgadd: u8,
    /// Whether the next CGRAM access is the high byte.
    cg_high: bool,
    /// The low byte a CGRAM write latched.
    cg_latch: u8,
    /// `$2123`-`$2125`: W12SEL, W34SEL, WOBJSEL.
    wsel: [u8; 3],
    /// `$2126`-`$2129`: W1 left, W1 right, W2 left, W2 right.
    wh: [u8; 4],
    /// `$212A`.
    wbglog: u8,
    /// `$212B`.
    wobjlog: u8,
    /// `$212C`: main screen designation.
    tm: u8,
    /// `$212D`: sub screen designation.
    ts: u8,
    /// `$212E`: main screen window mask enable.
    tmw: u8,
    /// `$212F`: sub screen window mask enable.
    tsw: u8,
    /// `$2130`.
    cgwsel: u8,
    /// `$2131`.
    cgadsub: u8,
    /// `$2132`: the fixed colour, red, green, blue, five bits each.
    fixed: [u8; 3],
    /// `$2133`.
    setini: u8,
    /// PPU1's data-bus latch: the last byte PPU1 drove on a read.
    ppu1_mdr: u8,
    /// PPU2's data-bus latch.
    ppu2_mdr: u8,
    /// The latched H counter, 9 bits.
    hlatch: u16,
    /// The latched V counter, 9 bits.
    vlatch: u16,
    /// `$213C`'s low/high flip-flop.
    oph_ff: bool,
    /// `$213D`'s low/high flip-flop.
    opv_ff: bool,
    /// `STAT78` bit 6: the counters have been latched since `$213F` was read.
    latched: bool,
    /// `STAT77` bit 7: more than 34 sprite tiles on a line.
    time_over: bool,
    /// `STAT77` bit 6: more than 32 sprites on a line.
    range_over: bool,
}

impl Regs {
    /// The power-on register file.
    ///
    /// The hardware leaves most of it undefined; zero is what this model
    /// picks, with **forced blank on**, so a machine that never programs the
    /// PPU shows black rather than whatever an all-zero CGRAM draws, and VRAM
    /// is writable from the first instruction.
    pub fn power_on() -> Regs {
        Regs {
            inidisp: 0x80,
            ..Regs::default()
        }
    }
}

/// Sign-extend a 13-bit value.
pub(super) const fn sext13(v: u16) -> i32 {
    ((v as i32) << 19) >> 19
}

impl Engine {
    // -- VRAM ---------------------------------------------------------------

    /// The `VMAIN` address translation: bits 2-3 rotate the low 8, 9 or 10
    /// bits of the word address left by three (fullsnes, VMAIN).
    pub fn vram_remap(&self, addr: u16) -> usize {
        let a = addr;
        let out = match (self.r.vmain >> 2) & 3 {
            0 => a,
            1 => (a & 0xFF00) | ((a & 0x001F) << 3) | ((a >> 5) & 7),
            2 => (a & 0xFE00) | ((a & 0x003F) << 3) | ((a >> 6) & 7),
            _ => (a & 0xFC00) | ((a & 0x007F) << 3) | ((a >> 7) & 7),
        };
        usize::from(out & 0x7FFF)
    }

    fn vram_step(&mut self) {
        let inc = match self.r.vmain & 3 {
            0 => 1,
            1 => 32,
            _ => 128,
        };
        self.r.vmaddr = self.r.vmaddr.wrapping_add(inc);
    }

    /// VRAM accepts writes only in vertical blank or forced blank; during the
    /// active display the PPU owns the bus and a write is lost (fullsnes,
    /// VMDATA).
    fn vram_writable(&self) -> bool {
        self.forced_blank() || self.vblank
    }

    // -- OAM ----------------------------------------------------------------

    /// The byte an internal OAM address reaches: the 512-byte low table, then
    /// the 32-byte high table mirrored through `$200`-`$3FF`.
    pub fn oam_index(addr: u16) -> usize {
        let a = usize::from(addr & 0x3FF);
        if a < 0x200 { a } else { 0x200 | (a & 0x1F) }
    }

    fn oam_write(&mut self, value: u8) {
        let a = self.r.oam_addr;
        if a & 1 == 0 {
            self.r.oam_latch = value;
        }
        if a < 0x200 {
            // The low table takes words: the even write is held, the odd one
            // commits both bytes together.
            if a & 1 == 1 {
                self.oam[usize::from(a - 1)] = self.r.oam_latch;
                self.oam[usize::from(a)] = value;
            }
        } else {
            self.oam[Engine::oam_index(a)] = value;
        }
        self.r.oam_addr = (a + 1) & 0x3FF;
    }

    // -- the counters -------------------------------------------------------

    /// Latch the H and V counters, as a `$2137` read, a `WRIO` bit-7 fall or
    /// a light gun does, and raise `STAT78` bit 6.
    pub fn latch_counters(&mut self) {
        self.r.hlatch = self.h_dot();
        self.r.vlatch = self.v;
        self.r.latched = true;
    }

    /// Drive the `EXTLATCH` input. A falling edge latches the counters
    /// (fullsnes, SLHV: "Switching WRIO Bit7 from 1-to-0", and a light gun's
    /// high-to-low on the same line).
    pub fn set_extlatch(&mut self, level: bool) {
        if self.extlatch && !level {
            self.latch_counters();
        }
        self.extlatch = level;
    }

    /// The multiplier: M7A × the last byte written to M7B, 24-bit signed.
    pub fn product(&self) -> i32 {
        let a = i32::from(self.r.m7[0] as i16);
        let b = i32::from((self.r.m7[1] >> 8) as u8 as i8);
        a * b
    }

    // -- the ports ----------------------------------------------------------

    /// A write to `$21xx`, `offset` = `xx` & `$3F`.
    pub fn write_register(&mut self, offset: u8, value: u8) {
        let r = &mut self.r;
        match offset & 0x3F {
            0x00 => r.inidisp = value,
            0x01 => r.obsel = value,
            0x02 => {
                r.oam_reload = (r.oam_reload & 0x100) | u16::from(value);
                r.oam_addr = r.oam_reload << 1;
            }
            0x03 => {
                r.oam_reload = (r.oam_reload & 0xFF) | (u16::from(value & 1) << 8);
                r.oam_priority = value & 0x80 != 0;
                r.oam_addr = r.oam_reload << 1;
            }
            0x04 => self.oam_write(value),
            0x05 => r.bgmode = value,
            0x06 => r.mosaic = value,
            0x07..=0x0A => r.bgsc[usize::from(offset - 0x07)] = value,
            0x0B => r.bgnba[0] = value,
            0x0C => r.bgnba[1] = value,
            0x0D..=0x14 => {
                let bg = usize::from((offset - 0x0D) / 2);
                let value16 = u16::from(value);
                if (offset - 0x0D).is_multiple_of(2) {
                    r.hofs[bg] =
                        ((value16 << 8) | u16::from(r.bgofs_latch & !7) | ((r.hofs[bg] >> 8) & 7))
                            & 0x3FF;
                    if offset == 0x0D {
                        r.m7hofs = ((value16 << 8) | u16::from(r.m7_latch)) & 0x1FFF;
                    }
                } else {
                    r.vofs[bg] = ((value16 << 8) | u16::from(r.bgofs_latch)) & 0x3FF;
                    if offset == 0x0E {
                        r.m7vofs = ((value16 << 8) | u16::from(r.m7_latch)) & 0x1FFF;
                    }
                }
                r.bgofs_latch = value;
                if offset == 0x0D || offset == 0x0E {
                    r.m7_latch = value;
                }
            }
            0x15 => r.vmain = value,
            0x16 | 0x17 => {
                if offset == 0x16 {
                    r.vmaddr = (r.vmaddr & 0xFF00) | u16::from(value);
                } else {
                    r.vmaddr = (r.vmaddr & 0x00FF) | (u16::from(value) << 8);
                }
                // Setting the address fills the read prefetch (fullsnes,
                // VMADD; the wiki's "VRAM read prefetch").
                let a = self.vram_remap(self.r.vmaddr);
                self.r.vram_prefetch = self.vram[a];
            }
            0x18 | 0x19 => {
                if self.vram_writable() {
                    let a = self.vram_remap(self.r.vmaddr);
                    let w = &mut self.vram[a];
                    *w = if offset == 0x18 {
                        (*w & 0xFF00) | u16::from(value)
                    } else {
                        (*w & 0x00FF) | (u16::from(value) << 8)
                    };
                }
                // The address steps after the byte VMAIN bit 7 names, whether
                // or not the write landed.
                let high = self.r.vmain & 0x80 != 0;
                if high == (offset == 0x19) {
                    self.vram_step();
                }
            }
            0x1A => r.m7sel = value,
            0x1B..=0x20 => {
                let word = (u16::from(value) << 8) | u16::from(r.m7_latch);
                match offset {
                    0x1B..=0x1E => r.m7[usize::from(offset - 0x1B)] = word,
                    0x1F => r.m7x = word & 0x1FFF,
                    _ => r.m7y = word & 0x1FFF,
                }
                r.m7_latch = value;
            }
            0x21 => {
                r.cgadd = value;
                r.cg_high = false;
            }
            0x22 => {
                if r.cg_high {
                    self.cgram[usize::from(r.cgadd)] =
                        (u16::from(value & 0x7F) << 8) | u16::from(r.cg_latch);
                    r.cgadd = r.cgadd.wrapping_add(1);
                } else {
                    r.cg_latch = value;
                }
                r.cg_high = !r.cg_high;
            }
            0x23..=0x25 => r.wsel[usize::from(offset - 0x23)] = value,
            0x26..=0x29 => r.wh[usize::from(offset - 0x26)] = value,
            0x2A => r.wbglog = value,
            0x2B => r.wobjlog = value,
            0x2C => r.tm = value,
            0x2D => r.ts = value,
            0x2E => r.tmw = value,
            0x2F => r.tsw = value,
            0x30 => r.cgwsel = value,
            0x31 => r.cgadsub = value,
            0x32 => {
                for (bit, channel) in [(0x20u8, 0usize), (0x40, 1), (0x80, 2)] {
                    if value & bit != 0 {
                        r.fixed[channel] = value & 0x1F;
                    }
                }
            }
            0x33 => r.setini = value,
            // $2134-$213F are read-only; nothing latches a write.
            _ => {}
        }
    }

    /// A read of `$21xx`. `bus` is the S-CPU's own open-bus byte, which is
    /// what a register that drives nothing returns. `debug` reads change
    /// nothing at all: no latch, no flip-flop, no address, no MDR.
    pub fn read_register(&mut self, offset: u8, bus: u8, debug: bool) -> u8 {
        let offset = offset & 0x3F;
        let value = match offset {
            MPYL..=0x36 => {
                let p = self.product() as u32;
                let v = (p >> (8 * u32::from(offset - MPYL))) as u8;
                if !debug {
                    self.r.ppu1_mdr = v;
                }
                return v;
            }
            SLHV => {
                // Latches only while EXTLATCH is high (fullsnes: "only if
                // WRIO.Bit7 is (or was) set"), and drives nothing.
                if !debug && self.extlatch {
                    self.latch_counters();
                }
                return bus;
            }
            RDOAM => {
                let v = self.oam[Engine::oam_index(self.r.oam_addr)];
                if !debug {
                    self.r.oam_addr = (self.r.oam_addr + 1) & 0x3FF;
                }
                v
            }
            RDVRAML | RDVRAMH => {
                let high = offset == RDVRAMH;
                let v = if high {
                    (self.r.vram_prefetch >> 8) as u8
                } else {
                    self.r.vram_prefetch as u8
                };
                // The incrementing half refills the prefetch from the current
                // address *then* steps it.
                if !debug && (self.r.vmain & 0x80 != 0) == high {
                    let a = self.vram_remap(self.r.vmaddr);
                    self.r.vram_prefetch = self.vram[a];
                    self.vram_step();
                }
                v
            }
            RDCGRAM => {
                let word = self.cgram[usize::from(self.r.cgadd)];
                let v = if self.r.cg_high {
                    ((word >> 8) as u8 & 0x7F) | (self.r.ppu2_mdr & 0x80)
                } else {
                    word as u8
                };
                if !debug {
                    if self.r.cg_high {
                        self.r.cgadd = self.r.cgadd.wrapping_add(1);
                    }
                    self.r.cg_high = !self.r.cg_high;
                    self.r.ppu2_mdr = v;
                }
                return v;
            }
            OPHCT | OPVCT => {
                let (latch, ff) = if offset == OPHCT {
                    (self.r.hlatch, self.r.oph_ff)
                } else {
                    (self.r.vlatch, self.r.opv_ff)
                };
                let v = if ff {
                    ((latch >> 8) as u8 & 1) | (self.r.ppu2_mdr & 0xFE)
                } else {
                    latch as u8
                };
                if !debug {
                    if offset == OPHCT {
                        self.r.oph_ff = !ff;
                    } else {
                        self.r.opv_ff = !ff;
                    }
                    self.r.ppu2_mdr = v;
                }
                return v;
            }
            STAT77 => {
                (u8::from(self.r.time_over) << 7)
                    | (u8::from(self.r.range_over) << 6)
                    | (self.r.ppu1_mdr & 0x10)
                    | (self.ppu1_version & 0x0F)
            }
            STAT78 => {
                let v = (u8::from(self.field) << 7)
                    | (u8::from(self.r.latched) << 6)
                    | (self.r.ppu2_mdr & 0x20)
                    | (u8::from(self.region == super::timing::Region::Pal) << 4)
                    | (self.ppu2_version & 0x0F);
                if !debug {
                    // fullsnes, STAT78: the read "resets the latch flag (bit6),
                    // and resets the two OPHCT/OPVCT 1st/2nd-read flipflops".
                    self.r.latched = false;
                    self.r.oph_ff = false;
                    self.r.opv_ff = false;
                    self.r.ppu2_mdr = v;
                }
                return v;
            }
            o if PPU1_OPEN_BUS.contains(&o) => return self.r.ppu1_mdr,
            _ => return bus,
        };
        // Everything that reaches here is a PPU1 read.
        if !debug {
            self.r.ppu1_mdr = value;
        }
        value
    }
}
