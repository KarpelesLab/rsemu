//! Gayle: the A600's gate array — its IDE port, its four registers, its
//! identification register and the overlay it keeps for itself.
//!
//! One class, `amiga.gayle`. On the A600 (and, as "AA Gayle", the A1200)
//! Gayle replaces the A500's Gary and takes on the new board's new jobs: the
//! chip selects for an IDE drive, the PCMCIA credit-card slot, and a register
//! file that manages both slots' interrupts. What is modelled here is what a
//! guest can see of it on an A600 with a hard disk and nothing in the card
//! slot.
//!
//! # The IDE port: address decode and a byte swap, nothing else
//!
//! "IDE" means the controller is on the drive, so what Gayle contributes is two
//! chip selects, a pair of strobes and an interrupt input. The drive is
//! [`crate::dev::ata::disk`], a separate object in the machine file, and the
//! split `src/dev/ata/mod.rs` sets holds here as it does in
//! [`crate::dev::pc::ide`]: **this file contains no ATA command opcode, no
//! `IDENTIFY` word index and no status- or error-register bit**, and the drive
//! contains no register offset. What is left is the decode:
//!
//! ```text
//!   A13 A12   window              chip select             Gayle spec 7.0
//!    0   0    $DA0000-$DA0FFF     -IDE_CS1 (CS1FX-)       "8 bit" timing
//!    0   1    $DA1000-$DA1FFF     -IDE_CS2 (CS3FX-)       "8 bit" timing
//!    1   0    $DA2000-$DA2FFF     -IDE_CS1 (CS1FX-)       "16 bit" timing
//!    1   1    $DA3000-$DA3FFF     -IDE_CS2 (CS3FX-)       "16 bit" timing
//!
//!   DA0 = A2, DA1 = A3, DA2 = A4                          Gayle spec 7.3
//! ```
//!
//! So the command block's register *n* is at `$DA2000 + 4n` (and again at
//! `$DA0000 + 4n`), and the control block's Device Control / Alternate Status
//! — register 6 — is at `$DA3018` (and `$DA1018`). `A13` changes the cycle's
//! timing and nothing a model without wait states can see; `A1` is not
//! decoded at all.
//!
//! Which select is which comes from the A600 schematic (#315987 rev. C, sheet
//! 12, "IDE DRIVE", `CN16`): `_IDE_CS(1)` goes to pin 37, which is `CS1FX-` —
//! the command block — and `_IDE_CS(2)` to pin 38, `CS3FX-`, the control
//! block (the A1200 functional specification's connector table, A3.1, names
//! those pins). The address lines are on the same sheet: `A2` to pin 35
//! (`DA0`), `A3` to pin 33 (`DA1`), `A4` to pin 36 (`DA2`). The register table
//! printed in section 7.3 of the (July 1991, draft) Gayle specification puts
//! the control block at `$DA0018` and the command block at `$DA1004`, which
//! contradicts both its own section 7.0 and the schematic; the schematic is
//! the board that shipped, and Kickstart's own accesses agree with it (see
//! `docs/platforms/amiga.md`, "A600").
//!
//! ## The data bus is byte-swapped
//!
//! The same sheet carries a note in capitals: "WARNING: BYTE SWAPPED". The
//! drive's `DD7`–`DD0` are the processor's `D15`–`D8`, and `DD15`–`DD8` are
//! `D7`–`D0`. Two consequences, both modelled:
//!
//! * The eight-bit registers, which ATA puts on `DD7`–`DD0`, are on the
//!   68000's *upper* byte — the even address. `$DA201C` is Status; the odd
//!   byte beside it is undriven and floats.
//! * A data word the drive puts out as `DD15..DD0 = w` arrives in memory as
//!   `w`'s low byte at the even address and its high byte at the odd one.
//!   That is a PC's byte order, which is why a disk written on a PC reads
//!   correctly here and why an Amiga hard-disk image — an HDF, whose first
//!   four bytes are `RDSK` — is the drive's sectors in order, with no
//!   swapping anywhere.
//!
//! And one consequence of the 68000 rather than of the board: a byte *write*
//! puts the byte on both halves of the bus (MC68000 User's Manual, M68000UM/AD
//! rev. 8, Table 3-1, *Data Strobe Control of Data Bus* — the rows it marks "a
//! result of current implementation", which `amiga.custom` relies on too). So
//! a byte written at the odd address of a register still reaches `DD7`–`DD0`.
//!
//! ## `MemAttrs::debug`
//!
//! As in [`crate::dev::pc::ide`]: a debug read of Status or Data is passed to
//! the drive as a flag and neither acknowledges nor advances anything, the
//! Alternate Status register has no side effect to begin with, and a debug
//! write is refused.
//!
//! # The four registers at `$DA8000`
//!
//! Section 19 of the specification: "Four registers are included that
//! facilitate support of the cartridge slot and IDE interrupt management. All
//! registers are set to zero at reset time." Gayle has no address pins below
//! `A12` (pin list, section 1.3), so each register fills a 4 KiB page; and its
//! data pins are the processor's `D15`–`D8` (schematic sheet 2, `U5`), so the
//! specification draws each as bits 15–8 and each is on the even byte.
//!
//! ```text
//!   $DA8000  status     7 IDE int   6 CC det   5 BVD2/DA  4 BVD1/SC
//!                       3 WR enable 2 BSY/IRQ  1 dig. audio enable  0 CC disable
//!   $DA9000  change     7..2 the same six lines' change latches
//!                       1 reset on CC status change  0 bus error on access after it
//!   $DAA000  enable     7 IDE int2   6 CC det int6   5 BVD2 int   4 BVD1 int
//!                       3 WR int2    2 BSY int       1 BVD level  0 BSY level (1: int6)
//!   $DAB000  config     3 slow memory  2 delay write  1 12V  0 5V   (7..4 read 0)
//! ```
//!
//! (The specification numbers them 15..8; the byte a 68000 reads puts them at
//! 7..0, which is how the constants below are written.)
//!
//! * **Status**: bits 7–2 read the six lines; writing a 1 to one of them "allows
//!   the software to force GAYLE to behave as if the specified line is asserted
//!   (including returning a '1' when this register is read)". Bits 1–0 are
//!   plain read/write.
//! * **Change**: a bit goes high when its line "has changed value", "remains
//!   high (and the interrupt line remains active) until a '0' is written to
//!   that bit. Writing a '1' will cause a bit to be unchanged."
//! * **Enable**: bit 7 "enables generation of an int2 [the draft misprints
//!   int3] on status change of the interrupt line of the IDE interface".
//!
//! So an IDE interrupt reaches the 68000 like this: the drive raises `INTRQ`
//! (`CN16` pin 31, `_IDE_IRQ`); Gayle latches the change in bit 7 of `$DA9000`;
//! with bit 7 of `$DAA000` set, Gayle pulls `_INT2` (pin 84, open collector,
//! the same net CIA-A's `/IRQ` is on — schematic sheet 7 and sheet 12's
//! expansion connector); and Paula sets `PORTS`, level 2 (Hardware Reference
//! Manual, chapter 7). Kickstart's handler reads `$DA9000`, reads the drive's
//! Status (which drops `INTRQ` — a second change, latched in the same bit) and
//! then writes `$7C` to `$DA9000`: a 0 to bit 7 lets go of the interrupt.
//!
//! # PCMCIA is out of scope
//!
//! The credit-card slot is not modelled. Its six input lines read as **no
//! card**: `CC det` is low, as are `BVD1`, `BVD2`, `WR` and `BSY`, which is
//! also what the specification says the lines look like with no card in
//! ("If the credit card is not inserted ... the credit card lines will all
//! appear to be negated"). Forcing one through the status register works, and
//! raises the change and interrupt it would, because that is Gayle's own
//! logic rather than the card's. The card's memory, attribute and I/O windows
//! are not decoded; with no card they are Zorro II space on the real board,
//! and here the board's space floats them.
//!
//! # The identification register at `$DE1000`
//!
//! Not in the July 1991 draft (it lists `$DE0000`–`$DEFFFF` as "Not used"),
//! nor in the A1200 functional specification's map, so what is here is
//! **black-box**: what Kickstart does there, and what it has to find.
//!
//! Kickstart 3.1 (40.063) and 2.05 (37.350) both write the register — a word
//! and then a byte of zero — and then read it byte-wide, four times in a row
//! (3.1 also once eight times). Only bit 7 of each read matters: with the four
//! bit-7s reading 1, 1, 0, 1, the ROM goes on to probe the IDE drive. With any
//! other first four tried — `$0`, `$5`, `$8`, `$9`, `$A`, `$C`, `$E` and `$F`
//! on 3.1, `$0` on 2.05 — it never touches `$DA0000` at all and waits at the
//! insert-disk screen, and reads five to eight (`$D0`, `$D1`, `$DF`) changed
//! nothing. So: a write restarts the sequence, and read *n*
//! returns bit *7 − n* of [`ID`] in bit 7, with zero after the eighth and in
//! the other seven bits. That is the least that is consistent with the trace,
//! not a claim about what the rest of the silicon holds.
//!
//! # The overlay
//!
//! "The ROMs are also selected in the range from $0000000 to $01FFFFF when the
//! internal overlay signal (OVL) is high ... The internal OVL signal becomes
//! asserted at reset, and negates on the first write to CIA1 (address range of
//! $BFD000 to $BFDFFF)" (section 2.0). On the A600, CIA-A's `PA0` is not
//! connected to anything (schematic sheet 7: `U7` pin 2 has no net), so the
//! A500's `wire cia_a.pa0 -> gary.ovl` has no counterpart: Gayle decodes the
//! CIAs itself (`-EVEN-CIA`, `-ODD-CIA`, section 5.0) and sees the write.
//!
//! **Either CIA's write negates it here, not only `$BFD000`'s**, and that is a
//! black-box correction to the draft. Kickstart 3.1's first CIA write is to
//! CIA-A (`$BFE001`); it goes on to use chip RAM at zero long
//! before it first writes CIA-B (`$BFD200`), and with the overlay held until
//! that write it never gets past its first loop. A real A600 boots that ROM,
//! and on the A600 the CIA-A write can only reach the overlay through Gayle —
//! so the shipped part negates on a write to the odd CIA. Whether it also does
//! on the even one, as the draft says, no trace here can tell; both are
//! modelled, and the first write to either wins.
//!
//! This class does not re-implement the overlay decoder; `amiga.gary` already
//! is one. Gayle drives an `ovl` output — high out of reset, low from the
//! first write through its `cia-odd` or `cia-even` region — and the board
//! wires it to Gary's `ovl` input. Each region forwards to the decode its link
//! names (the board's `amiga.cia-decode` objects), so the CIAs are unchanged.
//!
//! # Sources
//!
//! *GAYLE Gate array for A300/A500+ Specification*, Commodore, July 10, 1991
//! (sections 1.3, 2.0, 7.0, 7.3, 10.0, 16.0 and 19.0); *A600 System
//! Schematics*, Commodore, schematic #315987 rev. C, sheets 2, 7 and 12;
//! *A1200/A1200HD Advanced Amiga 1200 System Functional Specification*,
//! Commodore-Amiga, rev. 1.6 (the memory map, section 5, and the internal IDE
//! connector, A3.1); the MC68000 User's Manual (M68000UM/AD rev. 8), Table
//! 3-1, for the byte-write data bus; the *Amiga Hardware Reference Manual*,
//! 3rd ed., chapter 7, for `INT2`; and black-box traces of what Kickstart 3.1
//! and 2.05 read, write and wait on, for the identification register and for
//! which CIA write drops the overlay.
//! **No emulator source of any licence was consulted, no AROS source, and no
//! Kickstart disassembly** (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
    UnassignedPolicy,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::ata::bays::{self, Bay};
use crate::dev::ata::disk::{AtaDisk, Reg};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.gayle";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The region a `map` statement places at `$DA0000`: both IDE chip selects at
/// both timings.
pub const IDE_REGION: &str = "ide";

/// How much the IDE region decodes: `$DA0000`–`$DA3FFF`, "16 KB IDE drive"
/// (section 17.0's map). `$DA4000`–`$DA7FFF` selects nothing (section 7.0,
/// "None"), so it is not part of this region.
pub const IDE_WINDOW_LEN: u64 = 0x4000;

/// The region a `map` statement places at `$DA8000`: the four registers.
pub const REGS_REGION: &str = "regs";

/// How much the register region decodes: `$DA8000`–`$DAFFFF`, "Credit Card &
/// IDE configuration registers". Four registers of 4 KiB each, repeated once.
pub const REGS_WINDOW_LEN: u64 = 0x8000;

/// The region a `map` statement places at `$DE1000`: the identification
/// register.
pub const ID_REGION: &str = "id";

/// How much the identification region decodes: one 4 KiB page, because Gayle
/// sees nothing below `A12`.
pub const ID_WINDOW_LEN: u64 = 0x1000;

/// How much each CIA pass-through decodes: the 4 KiB window the board's
/// decode for that chip answers.
pub const CIA_WINDOW_LEN: u64 = 0x1000;

/// One of Gayle's two CIA selects (section 5.0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CiaSelect {
    /// `-ODD-CIA`: A12 low, data on odd addresses — CIA-A at `$BFE001`.
    Odd,
    /// `-EVEN-CIA`: A13 low, data on even addresses — CIA-B at `$BFD000`.
    Even,
}

impl CiaSelect {
    /// Both, in the order the device keeps them.
    pub const ALL: [CiaSelect; 2] = [CiaSelect::Odd, CiaSelect::Even];

    /// The name of the pass-through region, and of the link property naming
    /// the decode it forwards to.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            CiaSelect::Odd => "cia-odd",
            CiaSelect::Even => "cia-even",
        }
    }

    const fn index(self) -> usize {
        match self {
            CiaSelect::Odd => 0,
            CiaSelect::Even => 1,
        }
    }
}

/// The `INT2` output: open collector onto the net CIA-A's `/IRQ` is on.
pub const INT2_PIN: &str = "int2";

/// The `INT6` output: open collector onto the net CIA-B's `/IRQ` is on.
pub const INT6_PIN: &str = "int6";

/// The overlay output: high out of reset, low after the first write to a CIA.
pub const OVL_PIN: &str = "ovl";

/// The bay device 0 is fitted in when a machine file does not say.
pub const DEFAULT_MASTER_BAY: &str = "ata0";

/// The bay device 1 is fitted in when a machine file does not say.
pub const DEFAULT_SLAVE_BAY: &str = "ata1";

// -- register bits, as the even byte a 68000 reads carries them --------------

/// The IDE drive's interrupt line; its change latch; its `INT2` enable.
pub const IDE: u8 = 0x80;
/// Credit card detect; its change latch; its `INT6` enable.
pub const CC_DET: u8 = 0x40;
/// Battery voltage detect 2 / digital audio.
pub const BVD2: u8 = 0x20;
/// Battery voltage detect 1 / status change.
pub const BVD1: u8 = 0x10;
/// The card's write enable.
pub const WR: u8 = 0x08;
/// The card's busy / interrupt request.
pub const BSY: u8 = 0x04;
/// The six input lines, bits 7–2 of the status and change registers.
pub const LINES: u8 = 0xFC;
/// The two plain control bits at the bottom of the status and change
/// registers.
pub const CONTROL: u8 = 0x03;
/// In the enable register: the level the `BVD` lines interrupt on — set, int6.
pub const BVD_LEVEL6: u8 = 0x02;
/// In the enable register: the level the `BSY` line interrupts on — set, int6.
pub const BSY_LEVEL6: u8 = 0x01;
/// The configuration register's implemented bits. "Bits 12–15 are for the
/// currently unimplemented credit card page registers. You can tell they are
/// unimplemented because they do not read back what is written."
pub const CONFIG_BITS: u8 = 0x0F;

/// The identification Gayle reports at `$DE1000`, one bit a read, most
/// significant first, in bit 7.
///
/// Black-box: Kickstart 3.1 and 2.05 use the IDE port only when the first four
/// reads are 1, 1, 0, 1 — `$D` — and nothing they do depends on the next
/// four, which read 0 here. See the module documentation.
pub const ID: u8 = 0xD0;

/// Where the identification bit appears in the byte a read returns.
pub const ID_BIT: u8 = 0x80;

/// Which register of the four an offset in the register window selects.
#[must_use]
#[inline]
pub const fn register_at(offset: u64) -> u8 {
    ((offset >> 12) & 3) as u8
}

/// Which IDE chip select an offset in the IDE window asserts: `A12`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Select {
    /// `-IDE_CS1`, the drive's `CS1FX-`: the command block.
    Command,
    /// `-IDE_CS2`, the drive's `CS3FX-`: the control block.
    Control,
}

/// The chip select and the three drive address lines (`A4`–`A2`) an offset
/// in the IDE window decodes to.
#[must_use]
#[inline]
pub const fn ide_decode(offset: u64) -> (Select, u8) {
    let select = if offset & 0x1000 == 0 {
        Select::Command
    } else {
        Select::Control
    };
    (select, ((offset >> 2) & 7) as u8)
}

/// What `DA2`–`DA0` select in the command block.
///
/// The entire ATA content of this file, and the same eight names
/// [`crate::dev::pc::ide::register_at`] has; only where they are differs.
#[must_use]
pub const fn command_register(da: u8) -> Reg {
    match da & 7 {
        0 => Reg::Data,
        1 => Reg::Feature,
        2 => Reg::SectorCount,
        3 => Reg::LbaLow,
        4 => Reg::LbaMid,
        5 => Reg::LbaHigh,
        6 => Reg::Device,
        _ => Reg::Command,
    }
}

/// `DA2`–`DA0` of the control block's one register: Device Control on a
/// write, Alternate Status on a read (the spec's `$DA0018`/"3F6" row).
pub const CONTROL_REGISTER: u8 = 6;

// ---------------------------------------------------------------------------
// the registers
// ---------------------------------------------------------------------------

/// Everything Gayle holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Regs {
    /// Status bits 7–2 software has forced high.
    forced: u8,
    /// Status bits 1–0: digital audio enable, credit card disable.
    status_ctl: u8,
    /// Change latches, bits 7–2.
    change: u8,
    /// Change register bits 1–0: reset / bus error on a card status change.
    change_ctl: u8,
    /// The interrupt enable register.
    enable: u8,
    /// The configuration register's four implemented bits.
    config: u8,
    /// The lines as they were last seen, forced bits included — what a change
    /// is detected against.
    lines: u8,
    /// Where the identification sequence is: how many reads since the last
    /// write.
    id_reads: u8,
    /// The internal overlay signal.
    ovl: bool,
}

impl Regs {
    /// Out of reset: "all internal states in GAYLE are reset, including the
    /// registers, which are all set to '0'" (section 10.0), with the overlay
    /// asserted (section 2.0).
    fn reset() -> Regs {
        Regs {
            ovl: true,
            ..Regs::default()
        }
    }

    /// The status register's six lines: the drive's interrupt, no card, and
    /// whatever software forced.
    fn effective(&self, ide: bool) -> u8 {
        (if ide { IDE } else { 0 }) | (self.forced & LINES)
    }

    /// Take a new view of the lines, latching whatever changed.
    fn observe(&mut self, ide: bool) {
        let now = self.effective(ide);
        self.change |= (now ^ self.lines) & LINES;
        self.lines = now;
    }

    /// The two interrupt outputs, `(int2, int6)`.
    fn outputs(&self) -> (bool, bool) {
        let pending = self.change & self.enable & LINES;
        let bvd6 = self.enable & BVD_LEVEL6 != 0;
        let bsy6 = self.enable & BSY_LEVEL6 != 0;
        let mut int2 = pending & (IDE | WR) != 0;
        let mut int6 = pending & CC_DET != 0;
        if pending & (BVD1 | BVD2) != 0 {
            if bvd6 { int6 = true } else { int2 = true }
        }
        if pending & BSY != 0 {
            if bsy6 { int6 = true } else { int2 = true }
        }
        (int2, int6)
    }

    /// Read register `n`.
    fn read(&self, n: u8) -> u8 {
        match n {
            0 => self.lines | (self.status_ctl & CONTROL),
            1 => self.change | (self.change_ctl & CONTROL),
            2 => self.enable,
            _ => self.config & CONFIG_BITS,
        }
    }

    /// Write register `n`.
    fn write(&mut self, n: u8, value: u8) {
        match n {
            0 => {
                self.forced = value & LINES;
                self.status_ctl = value & CONTROL;
            }
            1 => {
                // "The bit remains high ... until a '0' is written to that bit.
                // Writing a '1' will cause a bit to be unchanged."
                self.change &= value | !LINES;
                self.change_ctl = value & CONTROL;
            }
            2 => self.enable = value,
            _ => self.config = value & CONFIG_BITS,
        }
    }
}

// ---------------------------------------------------------------------------
// the chip
// ---------------------------------------------------------------------------

/// The shared half of the device: the registers, the cable and the pins.
struct Chip {
    regs: Mutex<Regs>,
    bays: [Arc<Bay>; 2],
    names: [String; 2],
    int2: Mutex<Option<WireSource>>,
    int6: Mutex<Option<WireSource>>,
    ovl: Mutex<Option<WireSource>>,
    /// Each CIA's decode, in a private space at zero, odd first. `None` until
    /// bind, and forever if that select's link was not given.
    cias: [Mutex<Option<Arc<AddressSpace>>>; 2],
}

impl fmt::Debug for Chip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Chip")
            .field("regs", &*self.regs.lock())
            .field("master", &self.names[0])
            .field("slave", &self.names[1])
            .finish_non_exhaustive()
    }
}

impl Chip {
    /// Both drives, looked up with the bay locks released.
    fn drives(&self) -> [Option<Arc<AtaDisk>>; 2] {
        [self.bays[0].drive(), self.bays[1].drive()]
    }

    /// The drive that answers a read, if any.
    fn answering(drives: &[Option<Arc<AtaDisk>>; 2]) -> Option<&Arc<AtaDisk>> {
        drives.iter().flatten().find(|drive| drive.is_selected())
    }

    /// The drive's `INTRQ`, as `CN16` pin 31 carries it.
    fn intrq(&self) -> bool {
        let drives = self.drives();
        Chip::answering(&drives).is_some_and(|drive| drive.irq_asserted())
    }

    /// Look at the IDE line, latch any change, and drive both interrupt pins.
    ///
    /// The drive is asked first with nothing of Gayle's held, then Gayle's
    /// registers are updated in a short critical section, then the pins are
    /// driven with nothing held at all — mutate, release, then call outward.
    fn refresh(&self) {
        let ide = self.intrq();
        let (int2, int6) = {
            let mut regs = self.regs.lock();
            regs.observe(ide);
            regs.outputs()
        };
        drive(&self.int2, int2);
        drive(&self.int6, int6);
    }

    /// Drive the overlay pin from the registers.
    fn refresh_ovl(&self) {
        let ovl = self.regs.lock().ovl;
        drive(&self.ovl, ovl);
    }

    // -- the IDE cable -------------------------------------------------------

    fn read_command(&self, reg: Reg, debug: bool) -> Option<u16> {
        let drives = self.drives();
        match Chip::answering(&drives) {
            Some(drive) => Some(drive.read_reg(reg, debug)),
            // The selected position is empty but the other is not: the drive
            // that is there answers for it with zeroes, which is how a driver
            // is told there is nothing at that position (ATA-1 5.2.2; the
            // same rule `pc.ide` follows).
            None if drives.iter().any(Option::is_some) => Some(0),
            // An empty cable: nothing drives the bus at all.
            None => None,
        }
    }

    fn write_command(&self, reg: Reg, value: u16) {
        for drive in self.drives().iter().flatten() {
            drive.write_reg(reg, value);
        }
    }

    fn read_alt_status(&self) -> Option<u8> {
        let drives = self.drives();
        match Chip::answering(&drives) {
            Some(drive) => Some(drive.read_alt_status()),
            None if drives.iter().any(Option::is_some) => Some(0),
            None => None,
        }
    }

    fn write_control(&self, value: u8) {
        for drive in self.drives().iter().flatten() {
            drive.write_device_control(value);
        }
    }
}

/// Set an output pin, if it is connected, with no lock held while the net
/// delivers.
fn drive(pin: &Mutex<Option<WireSource>>, high: bool) {
    let source = pin.lock().clone();
    if let Some(source) = source {
        source.set(Level::from_bool(high));
    }
}

/// Constraints shared by every Gayle window: a byte or a word, big-endian,
/// either address — a 68000 on a sixteen-bit bus. A longword arrives as two
/// word cycles, each decoded on its own address, as on the board.
const fn bus_constraints() -> AccessConstraints {
    AccessConstraints {
        min: Width::U8,
        natural_alignment: false,
        ..AccessConstraints::word(Width::U16, Endian::Big)
    }
}

// ---------------------------------------------------------------------------
// the windows
// ---------------------------------------------------------------------------

/// `$DA0000`–`$DA3FFF`: the drive.
#[derive(Debug)]
struct IdeWindow(Arc<Chip>);

impl MemOps for IdeWindow {
    /// One bus cycle's read: a word is both lanes, a byte one of them, and a
    /// lane nothing drives floats.
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let chip = &self.0;
        let float = attrs.bus;
        let (select, da) = ide_decode(offset);
        // The two halves of the drive's sixteen data lines, as the processor
        // sees them: `DD7..DD0` on the even byte, `DD15..DD8` on the odd.
        let (even, odd) = match select {
            Select::Command if command_register(da) == Reg::Data => {
                // One -IOR, one word out of the sector buffer, whichever of the
                // processor's strobes were asserted.
                let word = chip.read_command(Reg::Data, attrs.debug);
                word.map_or((float, float), |w| (w as u8, (w >> 8) as u8))
            }
            Select::Command => {
                let byte = chip.read_command(command_register(da), attrs.debug);
                (byte.map_or(float, |b| b as u8), float)
            }
            Select::Control if da == CONTROL_REGISTER => {
                (chip.read_alt_status().unwrap_or(float), float)
            }
            // The control block's other addresses: not a register an ATA drive
            // drives (ATA-1 7.2, the drive address register is the drive's
            // alone and this drive has none), so the bus floats.
            Select::Control => (float, float),
        };
        match (dst.len(), offset & 1) {
            (2, 0) => {
                dst[0] = even;
                dst[1] = odd;
            }
            (1, 0) => dst[0] = even,
            (1, _) => dst[0] = odd,
            _ => return Err(BusError::BadAccess),
        }
        if !attrs.debug {
            chip.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A write to Command starts a command, one to Data fills a sector
            // buffer and one to Device Control can reset the drives. None of
            // them can be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let chip = &self.0;
        // What is on `D15..D8` and `D7..D0`. A 68000 byte write drives the
        // same byte on both halves (MC68000UM Table 3-1).
        let (high, low) = match (src.len(), offset & 1) {
            (2, 0) => (src[0], src[1]),
            (1, _) => (src[0], src[0]),
            _ => return Err(BusError::BadAccess),
        };
        // Byte-swapped: `D15..D8` is `DD7..DD0`.
        let word = u16::from(high) | u16::from(low) << 8;
        let (select, da) = ide_decode(offset);
        match select {
            Select::Command => chip.write_command(command_register(da), word),
            Select::Control if da == CONTROL_REGISTER => chip.write_control(high),
            Select::Control => {}
        }
        chip.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        bus_constraints()
    }
}

/// `$DA8000`–`$DAFFFF`: the four registers, on the even byte.
#[derive(Debug)]
struct RegsWindow(Arc<Chip>);

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // No register has a read side effect, so a debugger's read is an
        // ordinary one.
        let value = self.0.regs.lock().read(register_at(offset));
        match (dst.len(), offset & 1) {
            (2, 0) => {
                dst[0] = value;
                dst[1] = attrs.bus;
            }
            (1, 0) => dst[0] = value,
            (1, _) => dst[0] = attrs.bus,
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Clearing a change latch lets go of an interrupt.
            return Err(BusError::BadAccess);
        }
        let value = match (src.len(), offset & 1) {
            (2, 0) | (1, _) => src[0],
            _ => return Err(BusError::BadAccess),
        };
        self.0.regs.lock().write(register_at(offset), value);
        // A forced line is a changed line, and a cleared latch or a new enable
        // changes the pins.
        self.0.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        bus_constraints()
    }
}

/// `$DE1000`: the identification register.
#[derive(Debug)]
struct IdWindow(Arc<Chip>);

impl MemOps for IdWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let value = {
            let mut regs = self.0.regs.lock();
            let n = regs.id_reads;
            if !attrs.debug {
                regs.id_reads = n.saturating_add(1);
            }
            if n < 8 && (ID << n) & 0x80 != 0 {
                ID_BIT
            } else {
                0
            }
        };
        match (dst.len(), offset & 1) {
            (2, 0) => {
                dst[0] = value;
                dst[1] = attrs.bus;
            }
            (1, 0) => dst[0] = value,
            (1, _) => dst[0] = attrs.bus,
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        if !matches!(src.len(), 1 | 2) {
            return Err(BusError::BadAccess);
        }
        self.0.regs.lock().id_reads = 0;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        bus_constraints()
    }
}

/// `$BFE000` or `$BFD000`: a CIA, passed through, with the overlay watching.
#[derive(Debug)]
struct CiaWindow(Arc<Chip>, CiaSelect);

impl CiaWindow {
    fn target(&self) -> Option<Arc<AddressSpace>> {
        self.0.cias[self.1.index()].lock().clone()
    }
}

impl MemOps for CiaWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let Some(space) = self.target() else {
            return Err(BusError::Unassigned);
        };
        space.read_bytes(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(space) = self.target() else {
            return Err(BusError::Unassigned);
        };
        let result = space.write_bytes(offset, src, attrs);
        if !attrs.debug {
            // The first write to either CIA negates it; only the first
            // changes anything (see the module documentation for why either).
            let was = core::mem::replace(&mut self.0.regs.lock().ovl, false);
            if was {
                self.0.refresh_ovl();
            }
        }
        result
    }

    fn constraints(&self) -> AccessConstraints {
        // The decode behind it decides, as Gary's overlay lets its memories.
        AccessConstraints::IO
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The A600's gate array.
#[derive(Debug)]
pub struct Gayle {
    chip: Arc<Chip>,
    ide: RegionRef,
    regs: RegionRef,
    id: RegionRef,
    /// The two CIA pass-throughs, odd first.
    cia_regions: [RegionRef; 2],
    /// The objects `cia-odd` and `cia-even` name, resolved at bind.
    cia_paths: [Option<String>; 2],
}

impl Gayle {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown;
    /// [`Error::Config`] if `master` and `slave` name one bay.
    pub fn new(props: &Props) -> Result<Gayle> {
        let mut r = props.reader();
        let master = r.or_str("master", DEFAULT_MASTER_BAY)?.to_string();
        let slave = r.or_str("slave", DEFAULT_SLAVE_BAY)?.to_string();
        let odd = r
            .optional_link(CiaSelect::Odd.name())?
            .map(|l| l.as_str().to_string());
        let even = r
            .optional_link(CiaSelect::Even.name())?
            .map(|l| l.as_str().to_string());
        r.finish()?;
        if master == slave {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "`master` and `slave` are two positions on one cable and cannot both be \
                     `{master}`"
                ),
            });
        }
        // Opening a bay is allocation, not an outward action (`pc.ide`).
        let bays = [bays::attach(props, &master)?, bays::attach(props, &slave)?];
        Ok(Gayle::with_bays(bays, [master, slave], [odd, even]))
    }

    /// Build one around bays the caller already has.
    #[must_use]
    pub fn with_bays(bays: [Arc<Bay>; 2], names: [String; 2], cias: [Option<String>; 2]) -> Gayle {
        let chip = Arc::new(Chip {
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::reset()),
            bays,
            names,
            int2: Mutex::with_rank(LockRank::LEAF, None),
            int6: Mutex::with_rank(LockRank::LEAF, None),
            ovl: Mutex::with_rank(LockRank::LEAF, None),
            cias: [
                Mutex::with_rank(LockRank::LEAF, None),
                Mutex::with_rank(LockRank::LEAF, None),
            ],
        });
        let region = |name: &str, len: u64, ops: Arc<dyn MemOps>| -> RegionRef {
            Arc::new(Region::io(format!("{CLASS_NAME}.{name}"), len, ops))
        };
        Gayle {
            ide: region(
                IDE_REGION,
                IDE_WINDOW_LEN,
                Arc::new(IdeWindow(Arc::clone(&chip))),
            ),
            regs: region(
                REGS_REGION,
                REGS_WINDOW_LEN,
                Arc::new(RegsWindow(Arc::clone(&chip))),
            ),
            id: region(
                ID_REGION,
                ID_WINDOW_LEN,
                Arc::new(IdWindow(Arc::clone(&chip))),
            ),
            cia_regions: CiaSelect::ALL.map(|which| {
                region(
                    which.name(),
                    CIA_WINDOW_LEN,
                    Arc::new(CiaWindow(Arc::clone(&chip), which)),
                )
            }),
            chip,
            cia_paths: cias,
        }
    }

    /// Give one CIA pass-through the decode it forwards to, in a private
    /// space of its own — what `bind` does with the `cia-odd` and `cia-even`
    /// links.
    ///
    /// # Errors
    ///
    /// If the region cannot be mapped into the private space.
    pub fn attach_cia(&self, which: CiaSelect, region: &RegionRef, bits: u32) -> Result<()> {
        let space = AddressSpace::new(format!("{CLASS_NAME}.{}", which.name()), bits)
            .with_unassigned(UnassignedPolicy::OPEN_BUS);
        space.topology().map(Arc::clone(region), 0)?;
        *self.chip.cias[which.index()].lock() = Some(Arc::new(space));
        Ok(())
    }

    /// The drive in one of the two positions, if there is one.
    #[must_use]
    pub fn drive(&self, position: crate::dev::ata::Position) -> Option<Arc<AtaDisk>> {
        let index = usize::from(position == crate::dev::ata::Position::Device1);
        self.chip.bays[index].drive()
    }

    /// Whether the internal overlay signal is asserted.
    #[must_use]
    pub fn overlaid(&self) -> bool {
        self.chip.regs.lock().ovl
    }

    /// The two interrupt outputs as Gayle drives them, `(int2, int6)`.
    #[must_use]
    pub fn interrupts(&self) -> (bool, bool) {
        self.chip.regs.lock().outputs()
    }
}

/// The `amiga.gayle` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the A600's gate array: the IDE port's chip selects, the card and IDE interrupt \
              registers, the identification register and the internal overlay",
    properties: &[
        PropertySpec {
            name: "master",
            kind: ValueKind::Str,
            required: false,
            summary: "the drive bay device 0 is fitted in (default `ata0`)",
        },
        PropertySpec {
            name: "slave",
            kind: ValueKind::Str,
            required: false,
            summary: "the drive bay device 1 is fitted in (default `ata1`)",
        },
        PropertySpec {
            name: "cia-odd",
            kind: ValueKind::Link,
            required: false,
            summary: "CIA-A's decode, which the `cia-odd` region passes through; a write to it \
                      clears the overlay",
        },
        PropertySpec {
            name: "cia-even",
            kind: ValueKind::Link,
            required: false,
            summary: "CIA-B's decode, which the `cia-even` region passes through; a write to it \
                      clears the overlay",
        },
    ],
    construct: |props| Ok(Box::new(Gayle::new(props)?)),
};

impl Device for Gayle {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` statements place the regions, the wire graph
        // brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: `-RESET` resets every internal state (section 10.0), and
        // the overlay comes back so the processor finds its reset vector.
        // The drives are their own devices, declared first, so the line
        // observed here is the one they now drive.
        *self.chip.regs.lock() = Regs::reset();
        let ide = self.chip.intrq();
        self.chip.regs.lock().lines = if ide { IDE } else { 0 };
        self.chip.refresh();
        self.chip.refresh_ovl();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let r = *self.chip.regs.lock();
        for byte in [
            r.forced,
            r.status_ctl,
            r.change,
            r.change_ctl,
            r.enable,
            r.config,
            r.lines,
            r.id_reads,
        ] {
            w.write_u8(byte)?;
        }
        w.write_bool(r.ovl)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut bytes = [0u8; 8];
        for byte in &mut bytes {
            *byte = r.read_u8()?;
        }
        let ovl = r.read_bool()?;
        let [
            forced,
            status_ctl,
            change,
            change_ctl,
            enable,
            config,
            lines,
            id_reads,
        ] = bytes;
        *self.chip.regs.lock() = Regs {
            forced: forced & LINES,
            status_ctl: status_ctl & CONTROL,
            change: change & LINES,
            change_ctl: change_ctl & CONTROL,
            enable,
            config: config & CONFIG_BITS,
            lines: lines & LINES,
            id_reads,
            ovl,
        };
        // A restore does not re-run the wire graph; put the pins where the
        // registers say. `observe` is not called: the saved `lines` is the
        // level the drive's own restored state drives.
        let (int2, int6) = self.chip.regs.lock().outputs();
        drive(&self.chip.int2, int2);
        drive(&self.chip.int6, int6);
        self.chip.refresh_ovl();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | IDE_REGION => Some(Arc::clone(&self.ide)),
            REGS_REGION => Some(Arc::clone(&self.regs)),
            ID_REGION => Some(Arc::clone(&self.id)),
            "cia-odd" => Some(Arc::clone(&self.cia_regions[0])),
            "cia-even" => Some(Arc::clone(&self.cia_regions[1])),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let slot = match port {
            INT2_PIN => &self.chip.int2,
            INT6_PIN => &self.chip.int6,
            OVL_PIN => &self.chip.ovl,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("Gayle drives three pins: `int2`, `int6` and `ovl`"),
                });
            }
        };
        *slot.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        match port {
            INT2_PIN | INT6_PIN => self.chip.refresh(),
            OVL_PIN => self.chip.refresh_ovl(),
            _ => {}
        }
    }
}

/// The machine layer's half: the pass-through has to be told what it passes
/// to.
impl Instance for Gayle {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bits = ctx.space().map_or(32, |s| s.bits());
        for which in CiaSelect::ALL {
            let Some(path) = &self.cia_paths[which.index()] else {
                continue;
            };
            let region = ctx.region(path, "").map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!("`{}` has to name a CIA's decode: {e}", which.name()),
            })?;
            self.attach_cia(which, &region, bits)?;
        }
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Gayle::new(props)?)))
}

/// What the validator should know about `amiga.gayle`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("master", ValueKind::Str))
        .prop(PropSchema::new("slave", ValueKind::Str))
        .prop(PropSchema::new("cia-odd", ValueKind::Link))
        .prop(PropSchema::new("cia-even", ValueKind::Link))
        .region("")
        .region(IDE_REGION)
        .region(REGS_REGION)
        .region(ID_REGION)
        .region("cia-odd")
        .region("cia-even")
        .port(INT2_PIN, PortDir::Out)
        .port(INT6_PIN, PortDir::Out)
        .port(OVL_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
