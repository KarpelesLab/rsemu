//! The Amiga custom-chip register table, as the hardware manual prints it.
//!
//! Appendix B of the *Amiga Hardware Reference Manual* (Commodore-Amiga,
//! 3rd edition), "Register Summary — Address Order", is a table of about two
//! hundred word-wide registers at offsets `$000`…`$1FE` from the custom-chip
//! base. This module is that table as data, plus the four facts the appendix
//! records about each entry and nothing else:
//!
//! * **which chip owns it** — `A` Agnus, `D` Denise, `P` Paula, and the
//!   combinations `AP`, `AD`, `ADP`, `DP`, because a register really can be
//!   two chips' business at one address;
//! * **how it may be accessed** — `R` read-only, `W` write-only, `ER` an
//!   *early read* dummy address a DMA channel transfers through, `S` a strobe
//!   ("write address with no register bits");
//! * **who drives it** — `&` DMA channel only, `%` usually the DMA channel and
//!   sometimes the processor, `+` one half of an address register pair;
//! * **whether the copper may write it** — `*` never, `~` only when the
//!   *copper danger bit* in `COPCON` is set, unmarked always.
//!
//! The descriptions the appendix prints beside each name are not reproduced: a
//! register's address, its name and its access rules are facts about the
//! silicon, and the prose explaining what it does is the manual's to publish.
//! Read it there.
//!
//! # What the table is for
//!
//! [`custom`](super::custom) decodes with it and nothing else. A chip model
//! does not repeat any of it — it is handed a [`Reg`] and a word, and the
//! decode, the ownership routing, the copper-privilege check and the
//! write-only/read-only split have already happened. That is what makes adding
//! Agnus, Denise or Paula an exercise in *behaviour* rather than one in
//! re-deriving an address map three times.
//!
//! # Where this table departs from Appendix B, and why
//!
//! * **`DIWSTRT` and `DIWSTOP` belong to Agnus *and* Denise.** Appendix B, and
//!   Appendix A's entry for the pair, print the chip column as `A`. Appendix C,
//!   "Display Window Specification", prints both as `W A D`, and Chapter 3 says
//!   the window's horizontal resolution is one low-resolution pixel — a
//!   comparison only the chip that serializes pixels can make; Agnus counts
//!   the beam in colour clocks, two pixels each. The manual contradicts
//!   itself, and the reading that lets Denise clip its own output is the one
//!   taken. A table that routed these writes to Agnus alone would leave Denise
//!   no way to learn its own window.
//!
//! # `$1FE`, `SPRHDAT`, and the copy this was checked against
//!
//! The table was first transcribed through a summarising tool and has since
//! been checked, row by row, against the Appendix B text of the Amiga
//! Developer CD 2.1 edition of the manual. Every row matched except that two
//! were missing:
//!
//! * **`SPRHDAT` at `$078`** — "Ext. logic UHRES sprite pointer and data id",
//!   `~`, `W`, `A(E)`. An ECS row; nothing on an original-chip-set board acts
//!   on it, but the decode is the appendix's and so is this row.
//! * **`NO-OP(NULL)` at `$1FE`**, the appendix's last line, with no access
//!   letter and no chip. It is declared with neither, and
//!   [`custom`](super::custom) treats a write to such a row as the no-op it is
//!   named for: dropped, and *not* counted as unclaimed, because it is the
//!   address a copper list is written to pad with.
//!
//! # Byte accesses
//!
//! Every entry is a word, and the chips only ever see words: a byte access is
//! turned into the word access the hardware makes by
//! [`custom`](super::custom), whose documentation has the sources.

use core::fmt;

/// Which chip answers at an address — a set, not a choice.
///
/// A bitmask because the appendix's `AP`, `AD`, `ADP` and `DP` are real —
/// twenty-three rows name more than one chip. `DMACONR`, "DMA control (and
/// blitter status) read", is answered by Agnus and Paula together, and `DMACON`
/// is written to all three.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ChipId(pub u8);

impl ChipId {
    /// Nobody — the value of an offset with no register at it.
    pub const NONE: ChipId = ChipId(0);
    /// Agnus: DMA, the blitter, the copper, the beam counters.
    pub const AGNUS: ChipId = ChipId(1);
    /// Denise: bitplanes, sprites, the colour table, collisions.
    pub const DENISE: ChipId = ChipId(2);
    /// Paula: audio, the disk, the serial port, interrupts, the pot ports.
    pub const PAULA: ChipId = ChipId(4);

    /// Whether `other`'s chips are all in this set.
    #[must_use]
    #[inline]
    pub const fn contains(self, other: ChipId) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether the two sets share a chip.
    #[must_use]
    #[inline]
    pub const fn intersects(self, other: ChipId) -> bool {
        self.0 & other.0 != 0
    }

    /// The union of two sets.
    #[must_use]
    #[inline]
    pub const fn union(self, other: ChipId) -> ChipId {
        ChipId(self.0 | other.0)
    }
}

impl fmt::Display for ChipId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 == 0 {
            return f.write_str("nothing");
        }
        let mut first = true;
        for (bit, name) in [
            (ChipId::AGNUS, "Agnus"),
            (ChipId::DENISE, "Denise"),
            (ChipId::PAULA, "Paula"),
        ] {
            if self.contains(bit) {
                if !first {
                    f.write_str("+")?;
                }
                f.write_str(name)?;
                first = false;
            }
        }
        Ok(())
    }
}

impl core::ops::BitOr for ChipId {
    type Output = ChipId;

    fn bitor(self, rhs: ChipId) -> ChipId {
        self.union(rhs)
    }
}

/// The appendix's access column and its three symbol columns, as flags.
///
/// One type rather than four fields because every one of them is a property of
/// the same table row, and a decoder wants to ask "is this readable" without
/// caring which column the answer came out of.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Access(pub u16);

impl Access {
    /// The empty set.
    pub const NONE: Access = Access(0);
    /// `R` — the processor may read it.
    pub const READ: Access = Access(1 << 0);
    /// `W` — the processor may write it.
    pub const WRITE: Access = Access(1 << 1);
    /// `ER` — an *early read* dummy address, through which a DMA channel
    /// transfers to RAM. Not a register the processor reads for a value.
    pub const EARLY_READ: Access = Access(1 << 2);
    /// `S` — a strobe: "write address with no register bits". The write's data
    /// is ignored; the *address* is the event.
    pub const STROBE: Access = Access(1 << 3);
    /// `&` — used by a DMA channel only.
    pub const DMA_ONLY: Access = Access(1 << 4);
    /// `%` — used by a DMA channel usually, the processor sometimes.
    pub const DMA_USUAL: Access = Access(1 << 5);
    /// `+` — one half of an address register pair, whose other half is the
    /// adjacent offset. The pair must end up even and point into chip memory.
    pub const PAIR: Access = Access(1 << 6);
    /// `*` — the copper may never write this address.
    pub const COPPER_NEVER: Access = Access(1 << 7);
    /// `~` — the copper may write it only with the copper danger bit set.
    pub const COPPER_DANGER: Access = Access(1 << 8);
    /// `(E)` — changed or new in the Enhanced Chip Set.
    pub const ECS: Access = Access(1 << 9);

    /// Whether every flag in `other` is set here.
    #[must_use]
    #[inline]
    pub const fn contains(self, other: Access) -> bool {
        self.0 & other.0 == other.0
    }

    /// The union of two sets.
    #[must_use]
    #[inline]
    pub const fn union(self, other: Access) -> Access {
        Access(self.0 | other.0)
    }
}

impl core::ops::BitOr for Access {
    type Output = Access;

    fn bitor(self, rhs: Access) -> Access {
        self.union(rhs)
    }
}

/// Whether a copper write to this address is allowed, given `COPCON`'s danger
/// bit.
///
/// The appendix marks `$000`…`$03E` with `*` and `$040`…`$07E` with `~`, which
/// is the rule written as a table rather than as a sentence; this reads the
/// table.
#[must_use]
#[inline]
pub const fn copper_may_write(access: Access, danger: bool) -> bool {
    if access.contains(Access::COPPER_NEVER) {
        false
    } else if access.contains(Access::COPPER_DANGER) {
        danger
    } else {
        true
    }
}

/// One row of the appendix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reg {
    /// Byte offset from the custom-chip base, always even.
    pub offset: u16,
    /// The name the manual prints — `BLTCON0`, `COLOR00`.
    pub name: &'static str,
    /// Which chips answer here.
    pub chip: ChipId,
    /// The access column and the symbol columns.
    pub access: Access,
}

impl Reg {
    /// Whether the processor may read a value out of it.
    #[must_use]
    #[inline]
    pub const fn readable(&self) -> bool {
        self.access.contains(Access::READ)
    }

    /// Whether a write means anything here — a data write or a strobe.
    #[must_use]
    #[inline]
    pub const fn writable(&self) -> bool {
        self.access.contains(Access::WRITE) || self.access.contains(Access::STROBE)
    }

    /// Whether this row is the appendix's `NO-OP(NULL)`: an address with no chip
    /// and no access, which a write reaches and nothing latches.
    #[must_use]
    #[inline]
    pub const fn no_op(&self) -> bool {
        self.chip.0 == 0 && self.access.0 == 0
    }

    /// Whether this address is a strobe rather than a register.
    #[must_use]
    #[inline]
    pub const fn strobe(&self) -> bool {
        self.access.contains(Access::STROBE)
    }
}

/// How many word registers the custom-chip decode covers.
///
/// `$000`…`$1FE` inclusive, in steps of two. Not every slot is claimed — the
/// appendix has gaps at `$068`…`$06E`, `$076`…`$07A` and a dozen other places —
/// but the *decode* covers all of it, which is why this is the region size and
/// not the row count.
pub const REGISTERS: usize = 256;

/// The number of bytes the register file occupies: [`REGISTERS`] words.
pub const SPAN: u64 = REGISTERS as u64 * 2;

// The declaration the table is generated from. Column order matches the
// appendix: offset, name, chip, access. The one-letter aliases are the
// appendix's own column headings, which is the whole point of using them.
#[allow(non_upper_case_globals)]
const A: ChipId = ChipId::AGNUS;
#[allow(non_upper_case_globals)]
const D: ChipId = ChipId::DENISE;
#[allow(non_upper_case_globals)]
const P: ChipId = ChipId::PAULA;

const R: Access = Access::READ;
const W: Access = Access::WRITE;
const ER: Access = Access::EARLY_READ;
const S: Access = Access::STROBE;
const DMA_ONLY: Access = Access::DMA_ONLY;
const DMA_USUAL: Access = Access::DMA_USUAL;
const PAIR: Access = Access::PAIR;
const COP_NEVER: Access = Access::COPPER_NEVER;
const COP_DANGER: Access = Access::COPPER_DANGER;
const ECS: Access = Access::ECS;

/// One row, in the appendix's column order: offset, name, the chips in its
/// chip column, and the letters and symbols in its access and symbol columns.
const fn reg(offset: u16, name: &'static str, chips: &[ChipId], flags: &[Access]) -> Reg {
    let mut chip = ChipId::NONE;
    let mut i = 0;
    while i < chips.len() {
        chip = chip.union(chips[i]);
        i += 1;
    }
    let mut access = Access::NONE;
    let mut i = 0;
    while i < flags.len() {
        access = access.union(flags[i]);
        i += 1;
    }
    Reg {
        offset,
        name,
        chip,
        access,
    }
}

/// Appendix B, in address order.
///
/// Kept one row per line and exempt from `rustfmt`, because the point of the
/// layout is that it can be checked against the printed page column by column.
#[rustfmt::skip]
static DECLARED: &[Reg] = &[
    reg(0x000, "BLTDDAT",  &[A],       &[ER, DMA_ONLY, COP_NEVER]),
    reg(0x002, "DMACONR",  &[A, P],    &[R, COP_NEVER]),
    reg(0x004, "VPOSR",    &[A],       &[R, COP_NEVER, ECS]),
    reg(0x006, "VHPOSR",   &[A],       &[R, COP_NEVER]),
    reg(0x008, "DSKDATR",  &[P],       &[ER, DMA_ONLY, COP_NEVER]),
    reg(0x00a, "JOY0DAT",  &[D],       &[R, COP_NEVER]),
    reg(0x00c, "JOY1DAT",  &[D],       &[R, COP_NEVER]),
    reg(0x00e, "CLXDAT",   &[D],       &[R, COP_NEVER]),
    reg(0x010, "ADKCONR",  &[P],       &[R, COP_NEVER]),
    reg(0x012, "POT0DAT",  &[P],       &[R, COP_NEVER, ECS]),
    reg(0x014, "POT1DAT",  &[P],       &[R, COP_NEVER, ECS]),
    reg(0x016, "POTGOR",   &[P],       &[R, COP_NEVER]),
    reg(0x018, "SERDATR",  &[P],       &[R, COP_NEVER]),
    reg(0x01a, "DSKBYTR",  &[P],       &[R, COP_NEVER]),
    reg(0x01c, "INTENAR",  &[P],       &[R, COP_NEVER]),
    reg(0x01e, "INTREQR",  &[P],       &[R, COP_NEVER]),
    reg(0x020, "DSKPTH",   &[A],       &[W, PAIR, COP_NEVER, ECS]),
    reg(0x022, "DSKPTL",   &[A],       &[W, PAIR, COP_NEVER]),
    reg(0x024, "DSKLEN",   &[P],       &[W, COP_NEVER]),
    reg(0x026, "DSKDAT",   &[P],       &[W, DMA_ONLY, COP_NEVER]),
    reg(0x028, "REFPTR",   &[A],       &[W, DMA_ONLY, COP_NEVER]),
    reg(0x02a, "VPOSW",    &[A],       &[W, COP_NEVER]),
    reg(0x02c, "VHPOSW",   &[A],       &[W, COP_NEVER]),
    reg(0x02e, "COPCON",   &[A],       &[W, COP_NEVER, ECS]),
    reg(0x030, "SERDAT",   &[P],       &[W, COP_NEVER]),
    reg(0x032, "SERPER",   &[P],       &[W, COP_NEVER]),
    reg(0x034, "POTGO",    &[P],       &[W, COP_NEVER]),
    reg(0x036, "JOYTEST",  &[D],       &[W, COP_NEVER]),
    reg(0x038, "STREQU",   &[D],       &[S, DMA_ONLY, COP_NEVER]),
    reg(0x03a, "STRVBL",   &[D],       &[S, DMA_ONLY, COP_NEVER]),
    reg(0x03c, "STRHOR",   &[D, P],    &[S, DMA_ONLY, COP_NEVER]),
    reg(0x03e, "STRLONG",  &[D],       &[S, DMA_ONLY, COP_NEVER, ECS]),
    reg(0x040, "BLTCON0",  &[A],       &[W, COP_DANGER]),
    reg(0x042, "BLTCON1",  &[A],       &[W, COP_DANGER, ECS]),
    reg(0x044, "BLTAFWM",  &[A],       &[W, COP_DANGER]),
    reg(0x046, "BLTALWM",  &[A],       &[W, COP_DANGER]),
    reg(0x048, "BLTCPTH",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x04a, "BLTCPTL",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x04c, "BLTBPTH",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x04e, "BLTBPTL",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x050, "BLTAPTH",  &[A],       &[W, PAIR, COP_DANGER, ECS]),
    reg(0x052, "BLTAPTL",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x054, "BLTDPTH",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x056, "BLTDPTL",  &[A],       &[W, PAIR, COP_DANGER]),
    reg(0x058, "BLTSIZE",  &[A],       &[W, COP_DANGER]),
    reg(0x05a, "BLTCON0L", &[A],       &[W, COP_DANGER, ECS]),
    reg(0x05c, "BLTSIZV",  &[A],       &[W, COP_DANGER, ECS]),
    reg(0x05e, "BLTSIZH",  &[A],       &[W, COP_DANGER, ECS]),
    reg(0x060, "BLTCMOD",  &[A],       &[W, COP_DANGER]),
    reg(0x062, "BLTBMOD",  &[A],       &[W, COP_DANGER]),
    reg(0x064, "BLTAMOD",  &[A],       &[W, COP_DANGER]),
    reg(0x066, "BLTDMOD",  &[A],       &[W, COP_DANGER]),
    reg(0x070, "BLTCDAT",  &[A],       &[W, DMA_USUAL, COP_DANGER]),
    reg(0x072, "BLTBDAT",  &[A],       &[W, DMA_USUAL, COP_DANGER]),
    reg(0x074, "BLTADAT",  &[A],       &[W, DMA_USUAL, COP_DANGER]),
    reg(0x078, "SPRHDAT",  &[A],       &[W, COP_DANGER, ECS]),
    reg(0x07c, "DENISEID", &[D],       &[R, COP_DANGER, ECS]),
    reg(0x07e, "DSKSYNC",  &[P],       &[W, COP_DANGER]),
    reg(0x080, "COP1LCH",  &[A],       &[W, PAIR, ECS]),
    reg(0x082, "COP1LCL",  &[A],       &[W, PAIR]),
    reg(0x084, "COP2LCH",  &[A],       &[W, PAIR, ECS]),
    reg(0x086, "COP2LCL",  &[A],       &[W, PAIR]),
    reg(0x088, "COPJMP1",  &[A],       &[S]),
    reg(0x08a, "COPJMP2",  &[A],       &[S]),
    reg(0x08c, "COPINS",   &[A],       &[W]),
    reg(0x08e, "DIWSTRT",  &[A, D],    &[W]),
    reg(0x090, "DIWSTOP",  &[A, D],    &[W]),
    reg(0x092, "DDFSTRT",  &[A],       &[W]),
    reg(0x094, "DDFSTOP",  &[A],       &[W]),
    reg(0x096, "DMACON",   &[A, D, P], &[W]),
    reg(0x098, "CLXCON",   &[D],       &[W]),
    reg(0x09a, "INTENA",   &[P],       &[W]),
    reg(0x09c, "INTREQ",   &[P],       &[W]),
    reg(0x09e, "ADKCON",   &[P],       &[W]),
    reg(0x0a0, "AUD0LCH",  &[A],       &[W, PAIR, ECS]),
    reg(0x0a2, "AUD0LCL",  &[A],       &[W, PAIR]),
    reg(0x0a4, "AUD0LEN",  &[P],       &[W]),
    reg(0x0a6, "AUD0PER",  &[P],       &[W, ECS]),
    reg(0x0a8, "AUD0VOL",  &[P],       &[W]),
    reg(0x0aa, "AUD0DAT",  &[P],       &[W, DMA_ONLY]),
    reg(0x0b0, "AUD1LCH",  &[A],       &[W, PAIR]),
    reg(0x0b2, "AUD1LCL",  &[A],       &[W, PAIR]),
    reg(0x0b4, "AUD1LEN",  &[P],       &[W]),
    reg(0x0b6, "AUD1PER",  &[P],       &[W]),
    reg(0x0b8, "AUD1VOL",  &[P],       &[W]),
    reg(0x0ba, "AUD1DAT",  &[P],       &[W, DMA_ONLY]),
    reg(0x0c0, "AUD2LCH",  &[A],       &[W, PAIR]),
    reg(0x0c2, "AUD2LCL",  &[A],       &[W, PAIR]),
    reg(0x0c4, "AUD2LEN",  &[P],       &[W]),
    reg(0x0c6, "AUD2PER",  &[P],       &[W]),
    reg(0x0c8, "AUD2VOL",  &[P],       &[W]),
    reg(0x0ca, "AUD2DAT",  &[P],       &[W, DMA_ONLY]),
    reg(0x0d0, "AUD3LCH",  &[A],       &[W, PAIR]),
    reg(0x0d2, "AUD3LCL",  &[A],       &[W, PAIR]),
    reg(0x0d4, "AUD3LEN",  &[P],       &[W]),
    reg(0x0d6, "AUD3PER",  &[P],       &[W]),
    reg(0x0d8, "AUD3VOL",  &[P],       &[W]),
    reg(0x0da, "AUD3DAT",  &[P],       &[W, DMA_ONLY]),
    reg(0x0e0, "BPL1PTH",  &[A],       &[W, PAIR]),
    reg(0x0e2, "BPL1PTL",  &[A],       &[W, PAIR]),
    reg(0x0e4, "BPL2PTH",  &[A],       &[W, PAIR]),
    reg(0x0e6, "BPL2PTL",  &[A],       &[W, PAIR]),
    reg(0x0e8, "BPL3PTH",  &[A],       &[W, PAIR]),
    reg(0x0ea, "BPL3PTL",  &[A],       &[W, PAIR]),
    reg(0x0ec, "BPL4PTH",  &[A],       &[W, PAIR]),
    reg(0x0ee, "BPL4PTL",  &[A],       &[W, PAIR]),
    reg(0x0f0, "BPL5PTH",  &[A],       &[W, PAIR]),
    reg(0x0f2, "BPL5PTL",  &[A],       &[W, PAIR]),
    reg(0x0f4, "BPL6PTH",  &[A],       &[W, PAIR]),
    reg(0x0f6, "BPL6PTL",  &[A],       &[W, PAIR]),
    reg(0x100, "BPLCON0",  &[A, D],    &[W, ECS]),
    reg(0x102, "BPLCON1",  &[D],       &[W]),
    reg(0x104, "BPLCON2",  &[D],       &[W, ECS]),
    reg(0x106, "BPLCON3",  &[D],       &[W, ECS]),
    reg(0x108, "BPL1MOD",  &[A],       &[W]),
    reg(0x10a, "BPL2MOD",  &[A],       &[W]),
    reg(0x110, "BPL1DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x112, "BPL2DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x114, "BPL3DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x116, "BPL4DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x118, "BPL5DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x11a, "BPL6DAT",  &[D],       &[W, DMA_ONLY]),
    reg(0x120, "SPR0PTH",  &[A],       &[W, PAIR]),
    reg(0x122, "SPR0PTL",  &[A],       &[W, PAIR]),
    reg(0x124, "SPR1PTH",  &[A],       &[W, PAIR]),
    reg(0x126, "SPR1PTL",  &[A],       &[W, PAIR]),
    reg(0x128, "SPR2PTH",  &[A],       &[W, PAIR]),
    reg(0x12a, "SPR2PTL",  &[A],       &[W, PAIR]),
    reg(0x12c, "SPR3PTH",  &[A],       &[W, PAIR]),
    reg(0x12e, "SPR3PTL",  &[A],       &[W, PAIR]),
    reg(0x130, "SPR4PTH",  &[A],       &[W, PAIR]),
    reg(0x132, "SPR4PTL",  &[A],       &[W, PAIR]),
    reg(0x134, "SPR5PTH",  &[A],       &[W, PAIR]),
    reg(0x136, "SPR5PTL",  &[A],       &[W, PAIR]),
    reg(0x138, "SPR6PTH",  &[A],       &[W, PAIR]),
    reg(0x13a, "SPR6PTL",  &[A],       &[W, PAIR]),
    reg(0x13c, "SPR7PTH",  &[A],       &[W, PAIR]),
    reg(0x13e, "SPR7PTL",  &[A],       &[W, PAIR]),
    reg(0x140, "SPR0POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x142, "SPR0CTL",  &[A, D],    &[W, DMA_USUAL, ECS]),
    reg(0x144, "SPR0DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x146, "SPR0DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x148, "SPR1POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x14a, "SPR1CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x14c, "SPR1DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x14e, "SPR1DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x150, "SPR2POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x152, "SPR2CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x154, "SPR2DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x156, "SPR2DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x158, "SPR3POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x15a, "SPR3CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x15c, "SPR3DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x15e, "SPR3DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x160, "SPR4POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x162, "SPR4CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x164, "SPR4DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x166, "SPR4DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x168, "SPR5POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x16a, "SPR5CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x16c, "SPR5DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x16e, "SPR5DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x170, "SPR6POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x172, "SPR6CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x174, "SPR6DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x176, "SPR6DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x178, "SPR7POS",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x17a, "SPR7CTL",  &[A, D],    &[W, DMA_USUAL]),
    reg(0x17c, "SPR7DATA", &[D],       &[W, DMA_USUAL]),
    reg(0x17e, "SPR7DATB", &[D],       &[W, DMA_USUAL]),
    reg(0x180, "COLOR00",  &[D],       &[W]),
    reg(0x182, "COLOR01",  &[D],       &[W]),
    reg(0x184, "COLOR02",  &[D],       &[W]),
    reg(0x186, "COLOR03",  &[D],       &[W]),
    reg(0x188, "COLOR04",  &[D],       &[W]),
    reg(0x18a, "COLOR05",  &[D],       &[W]),
    reg(0x18c, "COLOR06",  &[D],       &[W]),
    reg(0x18e, "COLOR07",  &[D],       &[W]),
    reg(0x190, "COLOR08",  &[D],       &[W]),
    reg(0x192, "COLOR09",  &[D],       &[W]),
    reg(0x194, "COLOR10",  &[D],       &[W]),
    reg(0x196, "COLOR11",  &[D],       &[W]),
    reg(0x198, "COLOR12",  &[D],       &[W]),
    reg(0x19a, "COLOR13",  &[D],       &[W]),
    reg(0x19c, "COLOR14",  &[D],       &[W]),
    reg(0x19e, "COLOR15",  &[D],       &[W]),
    reg(0x1a0, "COLOR16",  &[D],       &[W]),
    reg(0x1a2, "COLOR17",  &[D],       &[W]),
    reg(0x1a4, "COLOR18",  &[D],       &[W]),
    reg(0x1a6, "COLOR19",  &[D],       &[W]),
    reg(0x1a8, "COLOR20",  &[D],       &[W]),
    reg(0x1aa, "COLOR21",  &[D],       &[W]),
    reg(0x1ac, "COLOR22",  &[D],       &[W]),
    reg(0x1ae, "COLOR23",  &[D],       &[W]),
    reg(0x1b0, "COLOR24",  &[D],       &[W]),
    reg(0x1b2, "COLOR25",  &[D],       &[W]),
    reg(0x1b4, "COLOR26",  &[D],       &[W]),
    reg(0x1b6, "COLOR27",  &[D],       &[W]),
    reg(0x1b8, "COLOR28",  &[D],       &[W]),
    reg(0x1ba, "COLOR29",  &[D],       &[W]),
    reg(0x1bc, "COLOR30",  &[D],       &[W]),
    reg(0x1be, "COLOR31",  &[D],       &[W]),
    reg(0x1c0, "HTOTAL",   &[A],       &[W, ECS]),
    reg(0x1c2, "HSSTOP",   &[A],       &[W, ECS]),
    reg(0x1c4, "HBSTRT",   &[A],       &[W, ECS]),
    reg(0x1c6, "HBSTOP",   &[A],       &[W, ECS]),
    reg(0x1c8, "VTOTAL",   &[A],       &[W, ECS]),
    reg(0x1ca, "VSSTOP",   &[A],       &[W, ECS]),
    reg(0x1cc, "VBSTRT",   &[A],       &[W, ECS]),
    reg(0x1ce, "VBSTOP",   &[A],       &[W, ECS]),
    reg(0x1dc, "BEAMCON0", &[A],       &[W, ECS]),
    reg(0x1de, "HSSTRT",   &[A],       &[W, ECS]),
    reg(0x1e0, "VSSTRT",   &[A],       &[W, ECS]),
    reg(0x1e2, "HCENTER",  &[A],       &[W, ECS]),
    reg(0x1e4, "DIWHIGH",  &[A, D],    &[W, ECS]),
    reg(0x1fe, "NO-OP",    &[],        &[]),
];

/// The declaration above, indexed by `offset / 2`.
///
/// Built at compile time from the one declaration rather than written out
/// twice, so a decoder and a monitor cannot disagree about what is at an
/// address (`CLAUDE.md`, *CPU cores* — the same argument applies to a register
/// file).
static TABLE: [Option<Reg>; REGISTERS] = build();

const fn build() -> [Option<Reg>; REGISTERS] {
    let mut table = [None; REGISTERS];
    let mut i = 0;
    while i < DECLARED.len() {
        let entry = DECLARED[i];
        table[(entry.offset >> 1) as usize] = Some(entry);
        i += 1;
    }
    table
}

/// The register at `offset`, if the appendix declares one there.
///
/// `offset` is relative to the custom-chip base and must be even and below
/// [`SPAN`]; anything else is `None`, which is the honest answer for an address
/// the appendix leaves blank.
#[must_use]
#[inline]
pub fn lookup(offset: u16) -> Option<&'static Reg> {
    if offset & 1 != 0 || u64::from(offset) >= SPAN {
        return None;
    }
    TABLE[(offset >> 1) as usize].as_ref()
}

/// Every register the appendix declares, in address order.
#[must_use]
pub fn declared() -> &'static [Reg] {
    DECLARED
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn the_table_is_in_address_order_with_no_repeats() {
        let mut last: Option<u16> = None;
        for reg in declared() {
            assert_eq!(reg.offset & 1, 0, "{} is at an odd offset", reg.name);
            assert!(u64::from(reg.offset) < SPAN, "{} is off the end", reg.name);
            if let Some(prev) = last {
                assert!(prev < reg.offset, "{} is out of order", reg.name);
            }
            last = Some(reg.offset);
        }
    }

    #[test]
    fn every_row_is_reachable_through_the_index() {
        for reg in declared() {
            assert_eq!(lookup(reg.offset).expect("declared, so indexed"), reg);
        }
        assert!(lookup(0x001).is_none(), "an odd offset is not a register");
        assert!(lookup(0x200).is_none(), "past the end of the file");
        // A gap the appendix really leaves: $068-$06E sits between BLTDMOD and
        // BLTCDAT with nothing in it.
        assert!(lookup(0x068).is_none());
    }

    #[test]
    fn the_copper_privilege_split_falls_where_the_appendix_puts_it() {
        // The `*` and `~` columns are a rule written as a table, and this is
        // the rule: nothing below $040, the blitter block only with CDANG,
        // everything from $080 up unconditionally. A row that breaks it is a
        // transcription error, which is why it is worth asserting.
        for reg in declared() {
            let free = copper_may_write(reg.access, false);
            let danger = copper_may_write(reg.access, true);
            if reg.offset < 0x040 {
                assert!(!free && !danger, "{} is below $040", reg.name);
            } else if reg.offset < 0x080 {
                assert!(!free && danger, "{} is in the blitter block", reg.name);
            } else {
                assert!(free && danger, "{} is at $080 or above", reg.name);
            }
        }
    }

    #[test]
    fn a_register_is_readable_or_writable_but_the_two_are_separate_questions() {
        // The appendix's access column is one letter per row, so no row is
        // both — which is exactly why a decoder must ask twice rather than
        // assuming a register answers a read with what was last written to it.
        for reg in declared() {
            assert!(
                !(reg.readable() && reg.writable()),
                "{} claims to be both readable and writable",
                reg.name
            );
        }
        assert!(lookup(0x002).expect("DMACONR").readable());
        assert!(!lookup(0x002).expect("DMACONR").writable());
        assert!(lookup(0x096).expect("DMACON").writable());
        assert!(!lookup(0x096).expect("DMACON").readable());
    }

    #[test]
    fn the_display_window_reaches_denise_as_appendix_c_prints_it() {
        // Appendix B says `A`; Appendix C, "Display Window Specification",
        // says `W A D`. See the module documentation for why C is followed.
        for offset in [0x08e, 0x090] {
            let reg = lookup(offset).expect("DIWSTRT and DIWSTOP");
            assert!(reg.chip.contains(ChipId::AGNUS), "{}", reg.name);
            assert!(reg.chip.contains(ChipId::DENISE), "{}", reg.name);
        }
        let shared = declared()
            .iter()
            .filter(|r| r.chip.0.count_ones() > 1)
            .count();
        assert_eq!(shared, 23, "the count the ChipId documentation quotes");
    }

    #[test]
    fn a_register_can_belong_to_more_than_one_chip() {
        let dmacon = lookup(0x096).expect("DMACON");
        assert!(dmacon.chip.contains(ChipId::AGNUS));
        assert!(dmacon.chip.contains(ChipId::DENISE));
        assert!(dmacon.chip.contains(ChipId::PAULA));
        let dmaconr = lookup(0x002).expect("DMACONR");
        assert!(dmaconr.chip.contains(ChipId::AGNUS));
        assert!(dmaconr.chip.contains(ChipId::PAULA));
        assert!(!dmaconr.chip.contains(ChipId::DENISE));
        assert!(dmaconr.chip.intersects(ChipId::PAULA));
        assert_eq!(dmacon.chip.to_string(), "Agnus+Denise+Paula");
        assert_eq!(ChipId::NONE.to_string(), "nothing");
    }

    #[test]
    fn the_strobes_are_where_the_appendix_puts_them() {
        for offset in [0x038, 0x03a, 0x03c, 0x03e, 0x088, 0x08a] {
            let reg = lookup(offset).expect("a strobe");
            assert!(reg.strobe(), "{} is not marked a strobe", reg.name);
            assert!(!reg.readable(), "{} is a strobe, not a read", reg.name);
            assert!(reg.writable(), "{} is reached by writing it", reg.name);
        }
    }

    #[test]
    fn the_pointer_pairs_are_adjacent_and_both_halves_are_marked() {
        for reg in declared() {
            if !reg.access.contains(Access::PAIR) {
                continue;
            }
            let other = if reg.name.ends_with('H') {
                reg.offset + 2
            } else {
                reg.offset - 2
            };
            let other = lookup(other).expect("a pointer pair has two halves");
            assert!(
                other.access.contains(Access::PAIR),
                "{} is paired with {}, which is not marked",
                reg.name,
                other.name
            );
        }
    }
}
