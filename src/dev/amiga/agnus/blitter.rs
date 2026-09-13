//! The blitter: four DMA channels, a function generator, area fill and line
//! drawing.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, 3rd edition: chapter 6 ("Blitter
//! Hardware") throughout, Appendix A's `BLTCON0`/`BLTCON1`, `BLTSIZE`,
//! `BLTxPT`, `BLTxMOD`, `BLTxDAT` and `BLTAFWM`/`BLTALWM` entries, and
//! Appendix C, *Big Blits* and *Other ECS Modifications*.
//!
//! # Area mode, word by word
//!
//! Chapter 6's *Blitter Key Points*: "The order of operations in the blitter
//! is masking, shifting, logical combination of sources, area fill, and zero
//! flag setting." For each word of each row:
//!
//! 1. Each **enabled** source is fetched and its pointer moves by two bytes —
//!    up in ascending mode, down in descending (`BLTCON1` `DESC`). A disabled
//!    source contributes the constant in its data register.
//! 2. **A** is ANDed with `BLTAFWM` on the first word of the row and `BLTALWM`
//!    on the last, both when the row is one word wide. In descending mode "the
//!    first word mask masks the last word in a row (which is still the first
//!    word fetched)".
//! 3. **A and B are shifted** by `ASH` (`BLTCON0` 15–12) and `BSH` (`BLTCON1`
//!    15–12): right in ascending mode, left in descending, with the bits
//!    shifted out of the previous word shifted in — "for the first word of the
//!    blit, zeros are shifted in; for each subsequent word of the same blit, the
//!    data shifted out from the previous word is shifted in", across rows too.
//! 4. The **minterm** byte (`BLTCON0` 7–0) selects which of the eight
//!    combinations of A, B and C produce a one (table 6-1's truth table:
//!    bit 0 is A̅B̅C̅, bit 7 is ABC).
//! 5. **Fill**, if `IFE` or `EFE` is set: "The blitter uses the fill carry-in
//!    bit as the starting fill state beginning at the rightmost edge of each
//!    line. For each '1' bit in the source area, the blitter flips the fill
//!    state." Inclusive fill keeps the line bits; exclusive fill outputs the
//!    fill state alone. The manual's worked patterns (`00100100-00011000` and
//!    its three fills) are this module's tests.
//! 6. **Zero detect** sees the result, "even if those destination bits are not
//!    written due to the D DMA channel being disabled".
//! 7. **D** is written if enabled, one word late: "the first two sets of
//!    sources are fetched before the first destination is written" (chapter 6,
//!    *Pipeline Register*).
//!
//! At the end of each row every enabled channel's modulo is added to its
//! pointer (subtracted in descending mode) — "the signed 16-bit modulo value
//! for that DMA channel is added to the address pointer" — and the fill state
//! goes back to `FCI`.
//!
//! # Line mode
//!
//! The manual gives the registers' meanings and the octant table but not the
//! stepping inside the chip, so the stepping below is the Bresenham walk those
//! registers are **set up for**, and the parts that are inference are marked:
//!
//! * `BLTAPTL` is the accumulator, preloaded with 4·dy − 2·dx, and `SIGN`
//!   (`BLTCON1` bit 6) its sign. Each pixel: if `SIGN` is clear the minor axis
//!   steps and `BLTAMOD` (4·(dy − dx)) is added; otherwise `BLTBMOD` (4·dy)
//!   is added. The major axis always steps. `SIGN` then follows the
//!   accumulator.
//! * The octant bits, from table 6-3 and the octant figure: **`SUD`** set
//!   means the major axis is horizontal ("sometimes up or down"), **`SUL`**
//!   set means the minor step goes up or left, **`AUL`** set means the major
//!   step does.
//! * The pixel is `BLTADAT` shifted right by `ASH` — the manual has `BLTADAT`
//!   preloaded with `$8000` and `ASH` holding the x coordinate within the word.
//!   A horizontal step moves `ASH`, carrying into the C and D pointers by a
//!   word; a vertical step adds or subtracts `BLTCMOD` and `BLTDMOD`.
//! * The texture bit is bit `BSH` of `BLTBDAT`, fed to the function generator
//!   as a whole word of that bit. **Inference:** `BSH` then counts down, wrapping
//!   from 0 to 15, so successive pixels take successive less significant bits.
//! * `SING` (`ONEDOT`) draws "only a single bit per horizontal line".
//!   **Inference:** the first pixel of each row is the one drawn.
//! * **Inference:** line mode has no pipeline delay; each pixel reads C and
//!   writes D at once.
//!
//! When the line is finished `ASH`, `BSH`, `SIGN`, the accumulator and the C
//! and D pointers are left where the walk ended, as the pointer entries say
//! pointers are after any blit.
//!
//! # Timing
//!
//! Chapter 6, *Blitter Speed*: "The minimum blitter cycle is four ticks; the
//! maximum is eight ticks. Use of the A register is always free. Use of the B
//! register always adds two ticks to the blitter cycle. Use of either C or D is
//! free, but use of both adds another two ticks ... When in line mode, each
//! pixel takes eight ticks." The ticks are the system clock, twice the colour
//! clock Agnus counts in, so a word costs 2, 3 or 4 counts and a line pixel 4.
//!
//! `BBUSY` is set by the write to `BLTSIZE` itself: "Starting with the Fat
//! Agnus the blitter busy bit has been fixed to be set as soon as you write to
//! BLTSIZE", and an A500's Agnus is a fat one.
//!
//! # Not modelled
//!
//! * **Contention.** No display, disk or audio slot is ever taken from the
//!   blitter and it never takes one from the 68000, so a blit always runs at
//!   the speed table's rate and `BLTPRI` does nothing.
//! * **Load-time shifting.** "The act of loading one of the data registers
//!   'draws' the data through the machine and shifts it", so a constant
//!   loaded before a shift change keeps the old shift. Here the shift is
//!   applied as each word is used.
//! * **`IFE` and `EFE` together.** Unspecified; exclusive wins.

/// `BLTCON0` bit 11: use source A.
pub const USEA: u16 = 1 << 11;
/// `BLTCON0` bit 10: use source B.
pub const USEB: u16 = 1 << 10;
/// `BLTCON0` bit 9: use source C.
pub const USEC: u16 = 1 << 9;
/// `BLTCON0` bit 8: use destination D.
pub const USED: u16 = 1 << 8;

/// `BLTCON1` bit 0: line mode.
pub const LINE: u16 = 1 << 0;
/// `BLTCON1` bit 1, area mode: descending.
pub const DESC: u16 = 1 << 1;
/// `BLTCON1` bit 2, area mode: fill carry input.
pub const FCI: u16 = 1 << 2;
/// `BLTCON1` bit 3, area mode: inclusive fill.
pub const IFE: u16 = 1 << 3;
/// `BLTCON1` bit 4, area mode: exclusive fill.
pub const EFE: u16 = 1 << 4;
/// `BLTCON1` bit 7: disable the D output (Appendix C).
pub const DOFF: u16 = 1 << 7;
/// `BLTCON1` bit 1, line mode: one dot per horizontal line.
pub const SING: u16 = 1 << 1;
/// `BLTCON1` bit 2, line mode: always up or left.
pub const AUL: u16 = 1 << 2;
/// `BLTCON1` bit 3, line mode: sometimes up or left.
pub const SUL: u16 = 1 << 3;
/// `BLTCON1` bit 4, line mode: sometimes up or down — the major axis is x.
pub const SUD: u16 = 1 << 4;
/// `BLTCON1` bit 6, line mode: the accumulator's sign.
pub const SIGN: u16 = 1 << 6;

/// Channel indices into the pointer, modulo and data arrays.
pub const A: usize = 0;
/// Source B.
pub const B: usize = 1;
/// Source C.
pub const C: usize = 2;
/// Destination D.
pub const D: usize = 3;

/// What the blitter reads and writes chip RAM with.
pub trait Memory {
    /// Read a word.
    fn read(&mut self, addr: u32) -> u16;
    /// Write a word.
    fn write(&mut self, addr: u32, value: u16);
}

/// The blitter's registers and the state of a blit in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Blitter {
    /// `BLTCON0`.
    pub con0: u16,
    /// `BLTCON1`.
    pub con1: u16,
    /// `BLTAFWM`.
    pub afwm: u16,
    /// `BLTALWM`.
    pub alwm: u16,
    /// `BLTAPT`, `BLTBPT`, `BLTCPT`, `BLTDPT`, indexed by [`A`]…[`D`].
    pub ptr: [u32; 4],
    /// `BLTAMOD`…`BLTDMOD`.
    pub modulo: [u16; 4],
    /// `BLTADAT`, `BLTBDAT`, `BLTCDAT`; the fourth slot is the last D output
    /// (`BLTDDAT`, which nothing can read).
    pub data: [u16; 4],
    /// `BLTSIZV`'s height, for an ECS big blit.
    pub sizv: u16,

    /// `BBUSY`.
    pub busy: bool,
    /// `BZERO`: every output word so far was zero.
    pub zero: bool,
    /// Rows in this blit.
    pub height: u32,
    /// Words per row.
    pub width: u32,
    /// Rows finished.
    pub row: u32,
    /// Words of the current row finished.
    pub col: u32,
    /// The previous masked A word, for the shifter.
    pub prev_a: u16,
    /// The previous B word.
    pub prev_b: u16,
    /// The fill state.
    pub fill: bool,
    /// The D write the pipeline is holding: address and word.
    pub pending: Option<(u32, u16)>,
    /// Counts accumulated toward the next word.
    pub credit: u32,
    /// Line mode: a dot has been drawn on the current row.
    pub dotted: bool,
}

impl Default for Blitter {
    fn default() -> Blitter {
        Blitter::new()
    }
}

impl Blitter {
    /// Power-on: every register zero and nothing running.
    #[must_use]
    pub const fn new() -> Blitter {
        Blitter {
            con0: 0,
            con1: 0,
            afwm: 0,
            alwm: 0,
            ptr: [0; 4],
            modulo: [0; 4],
            data: [0; 4],
            sizv: 0,
            busy: false,
            zero: false,
            height: 0,
            width: 0,
            row: 0,
            col: 0,
            prev_a: 0,
            prev_b: 0,
            fill: false,
            pending: None,
            credit: 0,
            dotted: false,
        }
    }

    /// A write to `BLTSIZE`: "h9..h0, w5..w0", zero meaning 1024 rows or 64
    /// words (chapter 6, *Blitter counting*).
    pub fn start_bltsize(&mut self, value: u16) {
        let height = match u32::from(value >> 6) {
            0 => 1024,
            h => h,
        };
        let width = match u32::from(value & 0x3f) {
            0 => 64,
            w => w,
        };
        self.start(height, width);
    }

    /// A write to `BLTSIZH` after `BLTSIZV`: Appendix C's 15-bit height and
    /// 11-bit width.
    ///
    /// **Inference:** a zero in either field is read the way `BLTSIZE` reads
    /// one — as the largest value — because Appendix C does not say.
    pub fn start_bltsizh(&mut self, value: u16) {
        let height = match u32::from(self.sizv & 0x7fff) {
            0 => 0x8000,
            h => h,
        };
        let width = match u32::from(value & 0x07ff) {
            0 => 0x800,
            w => w,
        };
        self.start(height, width);
    }

    fn start(&mut self, height: u32, width: u32) {
        self.busy = true;
        self.zero = true;
        self.height = height;
        self.width = width;
        self.row = 0;
        self.col = 0;
        self.prev_a = 0;
        self.prev_b = 0;
        self.fill = self.con1 & FCI != 0;
        self.pending = None;
        self.credit = 0;
        self.dotted = false;
    }

    /// Whether the current blit is a line.
    #[must_use]
    pub const fn line_mode(&self) -> bool {
        self.con1 & LINE != 0
    }

    /// Colour-clock counts per word (or per pixel, in line mode).
    #[must_use]
    pub const fn cost(&self) -> u32 {
        if self.line_mode() {
            return 4;
        }
        let mut ticks = 4;
        if self.con0 & USEB != 0 {
            ticks += 2;
        }
        if self.con0 & USEC != 0 && self.con0 & USED != 0 {
            ticks += 2;
        }
        ticks / 2
    }

    /// Words (or pixels) still to do.
    #[must_use]
    pub const fn remaining(&self) -> u64 {
        if !self.busy {
            return 0;
        }
        if self.line_mode() {
            return (self.height - self.row) as u64;
        }
        (self.height - self.row) as u64 * self.width as u64 - self.col as u64
    }

    /// Counts until the blit completes, if it runs uninterrupted.
    #[must_use]
    pub const fn ticks_to_finish(&self) -> u64 {
        let remaining = self.remaining();
        if remaining == 0 {
            return 0;
        }
        remaining * self.cost() as u64 - self.credit as u64
    }

    /// Spend one count. Returns `true` if this count finished the blit.
    pub fn tick(&mut self, mem: &mut impl Memory) -> bool {
        if !self.busy {
            return false;
        }
        self.credit += 1;
        if self.credit < self.cost() {
            return false;
        }
        self.credit = 0;
        if self.line_mode() {
            self.pixel(mem);
            self.row += 1;
            if self.row >= self.height {
                self.busy = false;
                return true;
            }
        } else {
            self.word(mem);
            if self.row >= self.height {
                if let Some((addr, value)) = self.pending.take() {
                    mem.write(addr, value);
                }
                self.busy = false;
                return true;
            }
        }
        false
    }

    /// Run the whole blit at once, ignoring time. For tests.
    pub fn run_to_completion(&mut self, mem: &mut impl Memory) {
        while self.busy {
            self.credit = self.cost().saturating_sub(1);
            self.tick(mem);
        }
    }

    /// One word of an area-mode blit.
    fn word(&mut self, mem: &mut impl Memory) {
        let desc = self.con1 & DESC != 0;
        let step = if desc { 2u32.wrapping_neg() } else { 2 };
        let first = self.col == 0;
        let last = self.col + 1 == self.width;

        for (channel, bit) in [(A, USEA), (B, USEB), (C, USEC)] {
            if self.con0 & bit != 0 {
                self.data[channel] = mem.read(self.ptr[channel]);
                self.ptr[channel] = self.ptr[channel].wrapping_add(step);
            }
        }
        // The pipeline: the previous word's destination lands after this
        // word's sources were fetched.
        if let Some((addr, value)) = self.pending.take() {
            mem.write(addr, value);
        }

        let mut a = self.data[A];
        if first {
            a &= self.afwm;
        }
        if last {
            a &= self.alwm;
        }
        let ash = u32::from(self.con0 >> 12);
        let bsh = u32::from(self.con1 >> 12);
        let a_out = shift(a, self.prev_a, ash, desc);
        self.prev_a = a;
        let b = self.data[B];
        let b_out = shift(b, self.prev_b, bsh, desc);
        self.prev_b = b;

        let mut d = minterm(self.con0 as u8, a_out, b_out, self.data[C]);
        if self.con1 & (IFE | EFE) != 0 {
            d = fill(d, &mut self.fill, self.con1 & EFE != 0);
        }
        if d != 0 {
            self.zero = false;
        }
        self.data[D] = d;
        if self.con0 & USED != 0 {
            if self.con1 & DOFF == 0 {
                self.pending = Some((self.ptr[D], d));
            }
            self.ptr[D] = self.ptr[D].wrapping_add(step);
        }

        self.col += 1;
        if self.col >= self.width {
            self.col = 0;
            self.row += 1;
            for (channel, bit) in [(A, USEA), (B, USEB), (C, USEC), (D, USED)] {
                if self.con0 & bit != 0 {
                    let modulo = i32::from(self.modulo[channel] as i16) as u32;
                    self.ptr[channel] = if desc {
                        self.ptr[channel].wrapping_sub(modulo)
                    } else {
                        self.ptr[channel].wrapping_add(modulo)
                    };
                }
            }
            self.fill = self.con1 & FCI != 0;
        }
    }

    /// One pixel of a line.
    fn pixel(&mut self, mem: &mut impl Memory) {
        let ash = u32::from(self.con0 >> 12);
        let bsh = u32::from(self.con1 >> 12);
        let a = self.data[A] >> ash;
        let b = if (self.data[B] >> bsh) & 1 != 0 {
            0xffff
        } else {
            0
        };
        let c = mem.read(self.ptr[C]);
        self.data[C] = c;
        if self.con1 & SING == 0 || !self.dotted {
            let d = minterm(self.con0 as u8, a, b, c);
            if d != 0 {
                self.zero = false;
            }
            self.data[D] = d;
            if self.con0 & USED != 0 && self.con1 & DOFF == 0 {
                mem.write(self.ptr[D], d);
            }
            self.dotted = true;
        }
        // The texture moves on whether or not the dot was drawn.
        let bsh = (bsh + 15) & 15;
        self.con1 = (self.con1 & 0x0fff) | ((bsh as u16) << 12);

        let x_major = self.con1 & SUD != 0;
        let sometimes_negative = self.con1 & SUL != 0;
        let always_negative = self.con1 & AUL != 0;
        let accumulator = self.ptr[A] as u16 as i16;
        let accumulator = if self.con1 & SIGN == 0 {
            // The minor axis steps.
            if x_major {
                self.step_vertical(sometimes_negative);
            } else {
                self.step_horizontal(sometimes_negative);
            }
            accumulator.wrapping_add(self.modulo[A] as i16)
        } else {
            accumulator.wrapping_add(self.modulo[B] as i16)
        };
        self.ptr[A] = (self.ptr[A] & !0xffff) | u32::from(accumulator as u16);
        if accumulator < 0 {
            self.con1 |= SIGN;
        } else {
            self.con1 &= !SIGN;
        }
        if x_major {
            self.step_horizontal(always_negative);
        } else {
            self.step_vertical(always_negative);
        }
    }

    fn step_horizontal(&mut self, left: bool) {
        let ash = self.con0 >> 12;
        let (ash, carry) = if left {
            if ash == 0 {
                (15, true)
            } else {
                (ash - 1, false)
            }
        } else if ash == 15 {
            (0, true)
        } else {
            (ash + 1, false)
        };
        self.con0 = (self.con0 & 0x0fff) | (ash << 12);
        if carry {
            let step = if left { 2u32.wrapping_neg() } else { 2 };
            self.ptr[C] = self.ptr[C].wrapping_add(step);
            self.ptr[D] = self.ptr[D].wrapping_add(step);
        }
    }

    fn step_vertical(&mut self, up: bool) {
        for channel in [C, D] {
            let modulo = i32::from(self.modulo[channel] as i16) as u32;
            self.ptr[channel] = if up {
                self.ptr[channel].wrapping_sub(modulo)
            } else {
                self.ptr[channel].wrapping_add(modulo)
            };
        }
        self.dotted = false;
    }
}

/// The barrel shifter: `sh` bits right (ascending) or left (descending), with
/// the bits shifted out of `prev` shifted in.
#[must_use]
#[inline]
pub fn shift(cur: u16, prev: u16, sh: u32, desc: bool) -> u16 {
    if sh == 0 {
        return cur;
    }
    let (cur, prev) = (u32::from(cur), u32::from(prev));
    let out = if desc {
        (cur << sh) | (prev >> (16 - sh))
    } else {
        (cur >> sh) | (prev << (16 - sh))
    };
    out as u16
}

/// The function generator: minterm bit `n` of `lf` selects the combination of
/// A, B, C whose binary value is `n`, A most significant (table 6-1).
#[must_use]
#[inline]
pub const fn minterm(lf: u8, a: u16, b: u16, c: u16) -> u16 {
    let mut d = 0u16;
    let mut n = 0;
    while n < 8 {
        if lf & (1 << n) != 0 {
            let ta = if n & 4 != 0 { a } else { !a };
            let tb = if n & 2 != 0 { b } else { !b };
            let tc = if n & 1 != 0 { c } else { !c };
            d |= ta & tb & tc;
        }
        n += 1;
    }
    d
}

/// Area fill over one word, right to left, carrying `state` in and out.
#[must_use]
pub fn fill(word: u16, state: &mut bool, exclusive: bool) -> u16 {
    let mut out = 0u16;
    for bit in 0..16 {
        let line = (word >> bit) & 1 != 0;
        if line {
            *state = !*state;
        }
        let set = if exclusive { *state } else { line || *state };
        if set {
            out |= 1 << bit;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::BTreeMap;

    /// Chip RAM as a sparse map, recording the order of writes.
    #[derive(Default)]
    struct Ram {
        words: BTreeMap<u32, u16>,
        writes: alloc::vec::Vec<u32>,
    }

    impl Memory for Ram {
        fn read(&mut self, addr: u32) -> u16 {
            *self.words.get(&(addr & !1)).unwrap_or(&0)
        }
        fn write(&mut self, addr: u32, value: u16) {
            self.words.insert(addr & !1, value);
            self.writes.push(addr & !1);
        }
    }

    #[test]
    fn the_minterm_table_matches_the_manuals_common_functions() {
        let (a, b, c) = (0xf0f0u16, 0xcccc, 0xaaaa);
        assert_eq!(minterm(0xf0, a, b, c), a, "D = A");
        assert_eq!(minterm(0xcc, a, b, c), b, "D = B");
        assert_eq!(minterm(0xaa, a, b, c), c, "D = C");
        assert_eq!(minterm(0x0f, a, b, c), !a);
        assert_eq!(minterm(0xfc, a, b, c), a | b, "D = A + B");
        assert_eq!(minterm(0xca, a, b, c), (a & b) | (!a & c), "cookie cut");
        assert_eq!(minterm(0x80, a, b, c), a & b & c);
        // "AC + B = ... BLTCON0 bit positions of 6, 4, 7, 3, and 2"
        let lf = (1 << 6) | (1 << 4) | (1 << 7) | (1 << 3) | (1 << 2);
        assert_eq!(minterm(lf, a, b, c), (a & !c) | b);
    }

    #[test]
    fn the_manuals_fill_examples() {
        // "00100100-00011000 filled with inclusive fill, yields
        // 00111100-00011000; with exclusive fill ... 00011100-00001000", and
        // with FCI set "11100111-11111111" and "11100011-11110111".
        let pattern = 0b0010_0100_0001_1000;
        assert_eq!(fill(pattern, &mut false, false), 0b0011_1100_0001_1000);
        assert_eq!(fill(pattern, &mut false, true), 0b0001_1100_0000_1000);
        assert_eq!(fill(pattern, &mut true, false), 0b1110_0111_1111_1111);
        assert_eq!(fill(pattern, &mut true, true), 0b1110_0011_1111_0111);
    }

    #[test]
    fn a_shift_carries_the_previous_word_in() {
        // Chapter 6's three-word, four-bit example.
        assert_eq!(
            shift(0x1234, 0x0000, 4, false),
            0x0123,
            "zeros for the first"
        );
        assert_eq!(
            shift(0x5678, 0x1234, 4, false),
            0x4567,
            "then the low nibble"
        );
        assert_eq!(
            shift(0x1234, 0xabcd, 4, true),
            0x234a,
            "left when descending"
        );
        assert_eq!(shift(0x1234, 0xabcd, 0, true), 0x1234);
    }

    #[test]
    fn the_speed_table() {
        let mut b = Blitter::new();
        b.con0 = USEA | USED;
        assert_eq!(b.cost(), 2, "A and D: four ticks");
        b.con0 = USEB | USED;
        assert_eq!(b.cost(), 3, "B and D: six");
        b.con0 = USEB | USEC | USED;
        assert_eq!(b.cost(), 4, "B, C and D: eight");
        b.con1 = LINE;
        assert_eq!(b.cost(), 4, "a line pixel: eight");
    }

    #[test]
    fn a_copy_moves_a_rectangle_and_adds_the_modulo_per_row() {
        let mut ram = Ram::default();
        for i in 0..6 {
            ram.words.insert(0x100 + i * 2, 0x1111 * (i as u16 + 1));
        }
        let mut b = Blitter::new();
        b.con0 = USEA | USED | 0xf0;
        b.afwm = 0xffff;
        b.alwm = 0xffff;
        b.ptr[A] = 0x100;
        b.ptr[D] = 0x200;
        b.modulo[A] = 2;
        b.modulo[D] = 0;
        b.start_bltsize((2 << 6) | 2);
        assert!(b.busy && b.zero);
        b.run_to_completion(&mut ram);
        assert!(!b.busy);
        assert!(!b.zero);
        // Row 0 is words 0,1; the modulo skips word 2; row 1 is words 3,4.
        assert_eq!(ram.words[&0x200], 0x1111);
        assert_eq!(ram.words[&0x202], 0x2222);
        assert_eq!(ram.words[&0x204], 0x4444);
        assert_eq!(ram.words[&0x206], 0x5555);
        assert_eq!(b.ptr[A], 0x10c, "last address plus increment and modulo");
        assert_eq!(b.ptr[D], 0x208);
    }

    #[test]
    fn the_pipeline_lets_a_one_word_overlap_copy_ascend() {
        // "This allows you to shift a bitmap up to one word to the right using
        // ascending mode ... even though normally parts of the destination
        // would be overwritten before they were fetched."
        let mut ram = Ram::default();
        for i in 0..4u32 {
            ram.words.insert(i * 2, 0x1000 + i as u16);
        }
        let mut b = Blitter::new();
        b.con0 = USEA | USED | 0xf0;
        b.afwm = 0xffff;
        b.alwm = 0xffff;
        b.ptr[A] = 0;
        b.ptr[D] = 2;
        b.start_bltsize((1 << 6) | 3);
        b.run_to_completion(&mut ram);
        assert_eq!(
            [ram.words[&0], ram.words[&2], ram.words[&4], ram.words[&6]],
            [0x1000, 0x1000, 0x1001, 0x1002]
        );
    }

    #[test]
    fn a_zero_result_leaves_bzero_set_and_d_disabled_still_counts() {
        let mut ram = Ram::default();
        ram.words.insert(0, 0xff00);
        ram.words.insert(0x10, 0x00ff);
        let mut b = Blitter::new();
        b.con0 = USEA | USEC | 0x80 | 0x40; // AB with B preloaded ones => AC-ish
        b.data[B] = 0xffff;
        b.con0 = USEA | USEC | 0xa0; // D = AC
        b.afwm = 0xffff;
        b.alwm = 0xffff;
        b.ptr[A] = 0;
        b.ptr[C] = 0x10;
        b.start_bltsize((1 << 6) | 1);
        b.run_to_completion(&mut ram);
        assert!(b.zero, "no overlap, so the zero flag stays true");
        assert!(ram.writes.is_empty(), "D disabled writes nothing");
    }

    #[test]
    fn descending_fill_crosses_words_right_to_left() {
        // Two words, one outline bit in each: bit 0 of the left word and bit 15
        // of the right... filled from the right edge, the fill runs from the
        // right word's bit 12 to the left word's bit 3.
        let mut ram = Ram::default();
        ram.words.insert(0x40, 0x0008); // left word: bit 3
        ram.words.insert(0x42, 0x1000); // right word: bit 12
        let mut b = Blitter::new();
        b.con0 = USEA | USED | 0xf0;
        b.con1 = DESC | IFE;
        b.afwm = 0xffff;
        b.alwm = 0xffff;
        b.ptr[A] = 0x42;
        b.ptr[D] = 0x42;
        b.start_bltsize((1 << 6) | 2);
        b.run_to_completion(&mut ram);
        assert_eq!(ram.words[&0x42], 0xf000, "from bit 12 leftwards");
        assert_eq!(ram.words[&0x40], 0x000f, "to bit 3");
    }

    /// Draw a line the way chapter 6's register summary sets one up.
    fn line(ram: &mut Ram, x1: i32, y1: i32, x2: i32, y2: i32, onedot: bool) -> Blitter {
        const WIDTH_BYTES: u16 = 8; // a 64-pixel-wide plane at address 0
        let (adx, ady) = ((x2 - x1).abs(), (y2 - y1).abs());
        let (dx, dy) = (adx.max(ady), adx.min(ady));
        let x_major = adx >= ady;
        let right = x2 >= x1;
        let down = y2 >= y1;
        let (sud, sul, aul) = if x_major {
            (true, !down, !right)
        } else {
            (false, !right, !down)
        };
        let mut b = Blitter::new();
        let accum = 4 * dy - 2 * dx;
        b.data[A] = 0x8000;
        b.data[B] = 0xffff;
        b.afwm = 0xffff;
        b.alwm = 0xffff;
        b.modulo[A] = (4 * (dy - dx)) as u16;
        b.modulo[B] = (4 * dy) as u16;
        b.modulo[C] = WIDTH_BYTES;
        b.modulo[D] = WIDTH_BYTES;
        b.ptr[A] = accum as u16 as u32;
        let word = (y1 as u32) * u32::from(WIDTH_BYTES) + ((x1 as u32) / 16) * 2;
        b.ptr[C] = word;
        b.ptr[D] = word;
        b.con0 = (((x1 % 16) as u16) << 12) | USEA | USEC | USED | 0xca;
        b.con1 = LINE
            | if sud { SUD } else { 0 }
            | if sul { SUL } else { 0 }
            | if aul { AUL } else { 0 }
            | if accum < 0 { SIGN } else { 0 }
            | if onedot { SING } else { 0 };
        b.start_bltsize((((dx + 1) as u16) << 6) | 2);
        b.run_to_completion(ram);
        b
    }

    fn pixel(ram: &Ram, x: u32, y: u32) -> bool {
        let word = ram.words.get(&(y * 8 + (x / 16) * 2)).copied().unwrap_or(0);
        word & (0x8000 >> (x % 16)) != 0
    }

    #[test]
    fn a_line_in_every_octant_hits_both_ends_and_one_pixel_per_major_step() {
        let ends = [
            (10, 10, 40, 20),
            (10, 10, 20, 40),
            (40, 10, 10, 20),
            (20, 10, 10, 40),
            (40, 20, 10, 10),
            (20, 40, 10, 10),
            (10, 20, 40, 10),
            (10, 40, 20, 10),
            (5, 30, 60, 30),
            (33, 2, 33, 45),
        ];
        for (x1, y1, x2, y2) in ends {
            let mut ram = Ram::default();
            let b = line(&mut ram, x1, y1, x2, y2, false);
            assert!(
                pixel(&ram, x1 as u32, y1 as u32),
                "start of {x1},{y1}->{x2},{y2}"
            );
            assert!(
                pixel(&ram, x2 as u32, y2 as u32),
                "end of {x1},{y1}->{x2},{y2}"
            );
            let count: u32 = ram.words.values().map(|w| w.count_ones()).sum();
            let length = (x2 - x1).abs().max((y2 - y1).abs()) as u32 + 1;
            assert_eq!(count, length, "{x1},{y1}->{x2},{y2}");
            assert!(!b.busy && !b.zero);
        }
    }

    #[test]
    fn onedot_leaves_one_bit_per_row() {
        let mut ram = Ram::default();
        line(&mut ram, 2, 5, 50, 15, true);
        for y in 5..=15u32 {
            let row: u32 = (0..64).filter(|&x| pixel(&ram, x, y)).count() as u32;
            assert_eq!(row, 1, "row {y}");
        }
    }
}
